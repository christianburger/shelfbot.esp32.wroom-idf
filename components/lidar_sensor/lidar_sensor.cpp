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
static LidarSensorState s_state        = LidarSensorState::SETUP;

static LidarScan      s_scans[2];
static volatile int   s_building_idx   = 0;
static volatile bool  s_scan_ready     = false;
static SemaphoreHandle_t s_scan_mutex  = nullptr;

static uint32_t       s_scan_count     = 0;
static float          s_last_end_angle = -1.0f;

// ============================================================================
//  Internal helpers
// ============================================================================

static float interpolate_angle(float start_deg, float end_deg,
                                int sample_idx, int sample_count)
{
    if (sample_count <= 1) return start_deg;
    float span = end_deg - start_deg;
    if (span >  180.0f) span -= 360.0f;
    if (span < -180.0f) span += 360.0f;
    float angle = start_deg + span * static_cast<float>(sample_idx)
                                   / static_cast<float>(sample_count - 1);
    if (angle <   0.0f) angle += 360.0f;
    if (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

static void accumulate_points(LidarScan& scan, const LidarParsedPacket& parsed,
                               int64_t timestamp_us)
{
    if (scan.point_count == 0) {
        scan.start_time_us = timestamp_us;
    }
    const int n = static_cast<int>(parsed.distances_mm.size());
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
 * @brief Detect a revolution boundary between consecutive packets.
 *
 * A new revolution is indicated when the sensor's angle stream wraps back
 * through 0°.  The old test (prev_end >= 340 AND new_start <= 20) is too
 * brittle: if the last packet of a revolution ends at, say, 330° the wrap is
 * missed entirely and points accumulate across multiple revolutions until the
 * 2000-point buffer overflows.
 *
 * Robust test:
 *   1. The new packet's start angle is numerically less than the previous
 *      packet's end angle (raw backward step — required for a 0°-crossing).
 *   2. The "forward gap" — how many degrees forward through 0° it would take
 *      to reach new_start from prev_end — is less than 90°.
 *
 * This catches wrap-arounds regardless of where the last packet of the
 * revolution ends, while correctly rejecting large backward jumps (sensor
 * errors or mid-revolution restarts).
 *
 * Example values and outcomes:
 *   prev=350 start=5   gap= 15° → true  (revolution, normal)
 *   prev=330 start=5   gap= 35° → true  (revolution, late wrap, was broken before)
 *   prev=270 start=5   gap= 95° → false (too large a gap, not a revolution)
 *   prev=200 start=5   gap=165° → false (large backward jump, not a revolution)
 *   prev=45  start=48  n/a      → false (going forward, first check exits early)
 */
static bool is_wrap_around(float prev_end_deg, float new_start_deg)
{
    if (new_start_deg >= prev_end_deg) {
        return false;   // angles are going forward — definitely not a wrap
    }
    const float forward_gap = 360.0f - prev_end_deg + new_start_deg;
    return forward_gap < 90.0f;
}

static void process_raw_packet(const uint8_t* raw47, int64_t timestamp_us)
{
    LidarParsedPacket parsed{};
    if (!LidarPacketParser::parse(raw47, 47, parsed)) {
        return;
    }
    if (!parsed.crc_valid) {
        ESP_LOGD(TAG, "CRC mismatch – dropping packet");
        return;
    }

    const float start_deg = parsed.start_angle_deg;
    const float end_deg   = parsed.end_angle_deg;

    if (s_last_end_angle >= 0.0f && is_wrap_around(s_last_end_angle, start_deg)) {
        int building = s_building_idx;
        s_scans[building].complete = true;

        if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_building_idx = building ^ 1;
            s_scan_ready   = true;
            s_scan_count++;
            xSemaphoreGive(s_scan_mutex);
        } else {
            ESP_LOGW(TAG, "Scan mutex timeout during swap – scan lost");
        }
        s_scans[s_building_idx].clear();
    }

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
        LYDSTO_Driver::MeasurementResult result{};
        const bool ok = s_driver->read_sensor(result);
        if (!ok) {
            continue;
        }
        if (s_driver->get_last_packet(raw, sizeof(raw))) {
            process_raw_packet(raw, result.timestamp_us);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(nullptr);
}

// ============================================================================
//  Lifecycle task
// ============================================================================

static void lidar_lifecycle_task(void* /*arg*/)
{
    static constexpr uint32_t RETRY_MS = 1000;

    while (!StateMachine::isInState("lidar_sensor", stateToString(LidarSensorState::INIT))) {
        StateMachine::advance("lidar_sensor");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "State: INIT");

    s_driver = new LYDSTO_Driver(LIDAR_UART_PORT, LIDAR_TX_PIN, LIDAR_RX_PIN, LIDAR_BAUD_RATE);
    const char* init_err = s_driver->init();
    if (init_err) {
        ESP_LOGE(TAG, "LYDSTO init failed: %s", init_err);
        delete s_driver;
        s_driver = nullptr;
        s_state = LidarSensorState::ERROR;
        StateMachine::recover();
        vTaskDelete(nullptr);
        return;
    }

    while (!StateMachine::isInState("lidar_sensor", stateToString(LidarSensorState::RUNNING))) {
        StateMachine::advance("lidar_sensor");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "State: RUNNING – spawning read task");

    s_scans[0].clear();
    s_scans[1].clear();
    s_building_idx  = 0;
    s_scan_ready    = false;
    s_last_end_angle = -1.0f;
    s_scan_count    = 0;
    s_state         = LidarSensorState::RUNNING;

    xTaskCreate(lidar_read_task, "lidar_read", 4096, nullptr,
                tskIDLE_PRIORITY + 2, &s_read_task);

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
    s_state = LidarSensorState::SETUP;
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
    s_state = LidarSensorState::STOPPED;
    ESP_LOGI(TAG, "Stopped");
}

LidarSensorState lidar_get_state() { return s_state; }

bool lidar_is_running() { return s_state == LidarSensorState::RUNNING; }

bool lidar_get_latest_scan(LidarScan& out)
{
    if (!s_scan_ready) return false;
    if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    if (!s_scan_ready) {
        xSemaphoreGive(s_scan_mutex);
        return false;
    }
    const int ready_idx = s_building_idx ^ 1;
    out = s_scans[ready_idx];
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
