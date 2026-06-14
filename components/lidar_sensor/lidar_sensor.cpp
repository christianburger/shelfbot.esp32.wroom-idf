// lidar_sensor.cpp
//
// Flat LiDAR component for the LYDSTO LDS02RR spinning LiDAR.
// Architecture follows motor_control: one FreeRTOS task, direct driver access,
// no abstraction layers.
//
// Scan accumulation strategy
// ─────────────────────────
// The LYDSTO fires ~15 packets per revolution, each covering a ~24° arc with
// 12 sample points.  We accumulate points from each packet into the "building"
// scan.  When a wrap-around is detected (previous end_angle near 360° and the
// new start_angle near 0°) the completed scan is swapped into a "ready" slot
// that callers can read with lidar_get_latest_scan().
//
// Double-buffer swap (fixes the suggestion's drop-points bug):
//   building_idx XOR 1 → ready_idx
//   After swap, points from the wrap-around packet are added to the NEW
//   building scan rather than discarded.

#include "lidar_sensor.hpp"
#include "lidar_packet_parser.hpp"
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <cstring>

static const char* TAG = "LidarSensor";

// ============================================================================
//  Internal state
// ============================================================================

static LYDSTO_Driver* s_driver         = nullptr;
static TaskHandle_t   s_read_task      = nullptr;
static LidarState     s_state          = LidarState::UNINIT;

// Double-buffer: scans[0] and scans[1] alternate between "building" and "ready".
static LidarScan      s_scans[2];
static volatile int   s_building_idx   = 0;   // index of the scan being built
static volatile bool  s_scan_ready     = false;
static SemaphoreHandle_t s_scan_mutex  = nullptr;

static uint32_t       s_scan_count     = 0;
static float          s_last_end_angle = -1.0f;  // end_angle of the previous packet

// ============================================================================
//  Internal helpers
// ============================================================================

/**
 * @brief Interpolate the angle of sample @p i within a packet.
 *
 * Handles the 359° → 0° wrap so that angles within a single packet are
 * always interpolated on the shorter arc.
 */
static float interpolate_angle(float start_deg, float end_deg,
                                int sample_idx, int sample_count)
{
    if (sample_count <= 1) return start_deg;

    float span = end_deg - start_deg;
    // Normalise span to (-180, +180] so we always interpolate the short way
    // round.  In practice packets span ~24° so this only matters near 0/360.
    if (span >  180.0f) span -= 360.0f;
    if (span < -180.0f) span += 360.0f;

    float angle = start_deg + span * static_cast<float>(sample_idx)
                                   / static_cast<float>(sample_count - 1);

    // Normalise result to [0, 360)
    if (angle <   0.0f) angle += 360.0f;
    if (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/**
 * @brief Add all 12 sample points from @p parsed into @p scan.
 *
 * Silently drops points that would overflow MAX_POINTS (shouldn't happen in
 * normal operation, but prevents buffer overrun if the sensor misbehaves).
 */
static void accumulate_points(LidarScan& scan, const LidarParsedPacket& parsed,
                               int64_t timestamp_us)
{
    if (scan.point_count == 0) {
        scan.start_time_us = timestamp_us;
    }

    const int n = static_cast<int>(parsed.distances_mm.size()); // always 12
    for (int i = 0; i < n; ++i) {
        if (scan.point_count >= LidarScan::MAX_POINTS) {
            ESP_LOGW(TAG, "Scan buffer full – dropping remaining points");
            break;
        }
        const uint16_t idx = scan.point_count;
        scan.distances_mm[idx] = parsed.distances_mm[i];
        scan.confidences[idx]  = parsed.confidences[i];
        scan.angles_deg[idx]   = interpolate_angle(parsed.start_angle_deg,
                                                    parsed.end_angle_deg,
                                                    i, n);
        scan.point_count++;
    }
    scan.end_time_us = timestamp_us;
}

/**
 * @brief Detect rotation wrap-around.
 *
 * We consider a wrap to have occurred when the previous packet ended near
 * 360° and the current packet starts near 0°.  A generous ±20° guard is used
 * to handle sensors that don't land exactly on 0/360.
 */
static bool is_wrap_around(float prev_end_deg, float new_start_deg)
{
    return (prev_end_deg >= 340.0f && new_start_deg <= 20.0f);
}

/**
 * @brief Core packet processing: parse one raw LYDSTO packet and accumulate.
 *
 * Called from the read task for every successfully received packet.
 */
static void process_raw_packet(const uint8_t* raw47, int64_t timestamp_us)
{
    LidarParsedPacket parsed{};
    if (!LidarPacketParser::parse(raw47, 47, parsed)) {
        return;   // malformed header
    }
    if (!parsed.crc_valid) {
        ESP_LOGD(TAG, "CRC mismatch (got 0x%02X expected 0x%02X) – dropping packet",
                 parsed.crc, parsed.crc_calculated);
        return;
    }

    const float start_deg = parsed.start_angle_deg;
    const float end_deg   = parsed.end_angle_deg;

    // ── Wrap-around detection ─────────────────────────────────────────────
    if (s_last_end_angle >= 0.0f && is_wrap_around(s_last_end_angle, start_deg)) {
        // Complete the current building scan
        int building = s_building_idx;
        s_scans[building].complete = true;

        // Publish: swap building ↔ ready under the mutex
        if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_building_idx = building ^ 1;   // flip buffer
            s_scan_ready   = true;
            s_scan_count++;
            xSemaphoreGive(s_scan_mutex);
        } else {
            // Couldn't get the mutex quickly — skip this swap to avoid blocking
            // the read task.  The scan will be overwritten next revolution.
            ESP_LOGW(TAG, "Scan mutex timeout during swap – scan lost");
        }

        // Reset the new building scan
        s_scans[s_building_idx].clear();
    }

    // ── Accumulate this packet's points into the building scan ────────────
    accumulate_points(s_scans[s_building_idx], parsed, timestamp_us);
    s_last_end_angle = end_deg;
}

// ============================================================================
//  Read task
// ============================================================================

static void lidar_read_task(void* /*arg*/)
{
    ESP_LOGI(TAG, "Read task started");

    uint8_t raw[47];

    while (true) {
        // read_sensor() internally calls readPacket() which uses the two-phase
        // drain strategy: non-blocking drain of any backlogged packets, then
        // a blocking 500 ms wait if the FIFO is empty.
        LYDSTO_Driver::MeasurementResult result{};
        const bool ok = s_driver->read_sensor(result);

        if (!ok) {
            // Timeout or parse failure – keep going; read_sensor already logs.
            continue;
        }

        // Grab the raw packet bytes that produced this result and re-parse
        // them through LidarPacketParser so we get per-point distances and
        // confidences (read_sensor only gives us the minimum-distance point).
        if (s_driver->get_last_packet(raw, sizeof(raw))) {
            process_raw_packet(raw, result.timestamp_us);
        }

        // Yield briefly so higher-priority tasks can run.  The LYDSTO fires
        // ~4500 packets/min (~13 ms between packets), so 1 ms here is fine.
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Unreachable in normal operation
    vTaskDelete(nullptr);
}

// ============================================================================
//  Lifecycle task (mirrors motor_control pattern)
// ============================================================================

static void lidar_lifecycle_task(void* /*arg*/)
{
    static constexpr uint32_t RETRY_MS = 1000;

    // ── Step 1: advance to IDLE ───────────────────────────────────────────
    while (!StateMachine::isInState("lidar_sensor", stateToString(LidarSensorState::IDLE))) {
        StateMachine::advance("lidar_sensor");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "State: IDLE");

    // ── Step 2: initialise the UART driver ───────────────────────────────
    s_driver = new LYDSTO_Driver(LIDAR_UART_PORT, LIDAR_TX_PIN, LIDAR_RX_PIN, LIDAR_BAUD_RATE);
    const char* init_err = s_driver->init();
    if (init_err) {
        ESP_LOGE(TAG, "LYDSTO init failed: %s", init_err);
        delete s_driver;
        s_driver = nullptr;
        s_state  = LidarState::ERROR;
        StateMachine::recover();
        vTaskDelete(nullptr);
        return;
    }

    // ── Step 3: advance to RUNNING ────────────────────────────────────────
    while (!StateMachine::isInState("lidar_sensor", stateToString(LidarSensorState::RUNNING))) {
        StateMachine::advance("lidar_sensor");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "State: RUNNING – spawning read task");

    // Initialise scan buffers
    s_scans[0].clear();
    s_scans[1].clear();
    s_building_idx  = 0;
    s_scan_ready    = false;
    s_last_end_angle = -1.0f;
    s_scan_count    = 0;
    s_state         = LidarState::RUNNING;

    xTaskCreate(lidar_read_task, "lidar_read", 4096, nullptr,
                tskIDLE_PRIORITY + 2, &s_read_task);

    // Lifecycle task is done; keep alive so vTaskDelete behaves correctly.
    vTaskDelete(nullptr);
}

// ============================================================================
//  Public API
// ============================================================================

void lidar_setup()
{
    ESP_LOGI(TAG, "lidar_setup – spawning lifecycle task");

    s_scan_mutex = xSemaphoreCreateMutex();
    configASSERT(s_scan_mutex);

    s_state = LidarState::UNINIT;

    xTaskCreate(lidar_lifecycle_task, "lidar_lifecycle", 4096, nullptr, 3, nullptr);
}

void lidar_stop()
{
    if (s_read_task) {
        vTaskDelete(s_read_task);
        s_read_task = nullptr;
    }
    if (s_driver) {
        delete s_driver;
        s_driver = nullptr;
    }
    s_state = LidarState::IDLE;
    ESP_LOGI(TAG, "Stopped");
}

LidarState lidar_get_state() { return s_state; }

bool lidar_is_running() { return s_state == LidarState::RUNNING; }

bool lidar_get_latest_scan(LidarScan& out)
{
    if (!s_scan_ready) return false;

    if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;

    if (!s_scan_ready) {          // re-check under lock
        xSemaphoreGive(s_scan_mutex);
        return false;
    }

    // The ready scan is in the buffer that is NOT currently being built.
    const int ready_idx = s_building_idx ^ 1;
    out = s_scans[ready_idx];     // copy
    s_scan_ready = false;

    xSemaphoreGive(s_scan_mutex);
    return true;
}

bool lidar_get_last_raw_packet(uint8_t* out, size_t len)
{
    if (!s_driver) return false;
    return s_driver->get_last_packet(out, len);
}

uint32_t lidar_get_packet_count()
{
    return s_driver ? s_driver->get_packet_count() : 0u;
}

uint32_t lidar_get_scan_count()
{
    return s_scan_count;
}
