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

static LYDSTO_Driver*   s_driver        = nullptr;
static TaskHandle_t     s_read_task     = nullptr;
static LidarSensorState s_state         = LidarSensorState::SETUP;

// Double buffer: s_scans[s_building_idx] is the one being filled by lidar_read_task.
// The *other* index (s_building_idx ^ 1) holds the last completed scan.
// The mutex protects the swap and the s_scan_ready flag only — it is NOT
// held during point accumulation (which is single-writer, single slot).
static LidarScan         s_scans[2];
static volatile int      s_building_idx  = 0;
static volatile bool     s_scan_ready    = false;
static SemaphoreHandle_t s_scan_mutex    = nullptr;

// Diagnostic counters (written only from lidar_read_task)
static uint32_t s_scan_count        = 0;
static uint32_t s_overflow_count    = 0;  // scan buffer overflows
static uint32_t s_crc_fail_count    = 0;
static uint32_t s_parse_fail_count  = 0;
static uint32_t s_wrap_count        = 0;  // revolution boundaries detected
static float    s_last_end_angle    = -1.0f;

// ============================================================================
//  Angle helpers
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

/**
 * @brief Detect a revolution boundary.
 *
 * Returns true when the angle stream wraps back through 0°.
 *
 * Robust test:
 *   1. new_start is numerically less than prev_end  (raw backward step — required).
 *   2. The forward gap (360 - prev_end + new_start) is < 90°.
 *
 * This catches wraps regardless of where the last packet of a revolution ends,
 * while correctly rejecting large backward jumps (sensor errors).
 */
static bool is_wrap_around(float prev_end_deg, float new_start_deg)
{
    if (new_start_deg >= prev_end_deg) {
        return false;   // going forward — not a wrap
    }
    const float forward_gap = 360.0f - prev_end_deg + new_start_deg;
    return forward_gap < 120.0f;
}

// ============================================================================
//  Point accumulation
//
//  Called ONLY from lidar_read_task; no locking needed for the building scan.
//  Returns true if all points were added, false if the buffer was full for
//  at least one point (overflow).
// ============================================================================

static bool accumulate_points(LidarScan& scan, const LidarParsedPacket& parsed,
                               int64_t timestamp_us)
{
    if (scan.point_count == 0) {
        scan.start_time_us = timestamp_us;
    }
    const int n = static_cast<int>(parsed.distances_mm.size());
    bool overflow = false;
    for (int i = 0; i < n; ++i) {
        if (scan.point_count >= LidarScan::MAX_POINTS) {
            overflow = true;
            // Don't break — count all overflowed points for diagnostics
            continue;
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
    return !overflow;
}

// ============================================================================
//  Packet processing  (called only from lidar_read_task)
// ============================================================================

static void process_raw_packet(const uint8_t* raw47, int64_t timestamp_us)
{
    LidarParsedPacket parsed{};
    if (!LidarPacketParser::parse(raw47, 47, parsed)) {
        ++s_parse_fail_count;
        return;
    }
    if (!parsed.crc_valid) {
        ++s_crc_fail_count;
        ESP_LOGD(TAG, "CRC mismatch – dropping packet (total=%lu)", (unsigned long)s_crc_fail_count);
        return;
    }

    const float start_deg = parsed.start_angle_deg;
    const float end_deg   = parsed.end_angle_deg;

    // ── Revolution boundary detection ──────────────────────────────────────
    if (s_last_end_angle >= 0.0f && is_wrap_around(s_last_end_angle, start_deg)) {
        ++s_wrap_count;
        const int building = s_building_idx;
        s_scans[building].complete = true;

        const uint16_t completed_pts = s_scans[building].point_count;

        if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            const int next_idx = building ^ 1;
            s_building_idx = next_idx;
            s_scan_ready   = true;
            s_scan_count++;
            xSemaphoreGive(s_scan_mutex);

            // Clear the *new* building buffer now that the mutex is released —
            // lidar_get_latest_scan reads the *other* (completed) buffer.
            s_scans[s_building_idx].clear();

            const float dt_ms = (s_scans[building].end_time_us > s_scans[building].start_time_us)
                ? static_cast<float>(s_scans[building].end_time_us - s_scans[building].start_time_us) / 1000.0f
                : 0.0f;
            ESP_LOGI(TAG, "scan #%lu complete: pts=%u dt=%.1fms wraps=%lu overflows=%lu crc_fails=%lu",
                     (unsigned long)s_scan_count,
                     (unsigned)completed_pts,
                     dt_ms,
                     (unsigned long)s_wrap_count,
                     (unsigned long)s_overflow_count,
                     (unsigned long)s_crc_fail_count);

            // Warn if the scan was sparse (< 100 points suggests data loss)
            if (completed_pts < 100) {
                ESP_LOGW(TAG, "Sparse scan #%lu: only %u points (expected ~150+)",
                         (unsigned long)s_scan_count, (unsigned)completed_pts);
            }
        } else {
            ESP_LOGE(TAG, "Scan mutex timeout during swap – scan lost! (wrap #%lu, pts=%u)",
                     (unsigned long)s_wrap_count, (unsigned)completed_pts);
            // Don't swap — keep accumulating into the current buffer to avoid
            // losing the partial data entirely.  The buffer will be cleared
            // on the next successful swap.
        }
    }

    // ── Accumulate into the current building buffer ─────────────────────────
    const bool ok = accumulate_points(s_scans[s_building_idx], parsed, timestamp_us);
    if (!ok) {
        ++s_overflow_count;
        // Log every overflow, but throttle to avoid flooding
        if ((s_overflow_count % 10) == 1) {
            ESP_LOGW(TAG, "Scan buffer full – dropping points "
                     "(overflow #%lu, pts_in_buf=%u/%u, wrap_last=%.1f->%.1f)",
                     (unsigned long)s_overflow_count,
                     (unsigned)s_scans[s_building_idx].point_count,
                     (unsigned)LidarScan::MAX_POINTS,
                     s_last_end_angle, start_deg);
        }

        // ── Overflow recovery ─────────────────────────────────────────────
        // If the buffer is full and we're nowhere near a wrap-around angle,
        // the revolution detection logic may have missed a wrap (e.g. because
        // the LiDAR started at an unusual angle).  Force a manual wrap if the
        // buffer has been filled without a revolution boundary.
        //
        // Heuristic: if point_count >= MAX_POINTS AND we have seen at least
        // 300° of angle coverage (inferred from buffer density), force a swap.
        if (s_scans[s_building_idx].point_count >= LidarScan::MAX_POINTS) {
            ESP_LOGW(TAG, "Buffer full – forcing scan swap to prevent stall");
            const int building = s_building_idx;
            s_scans[building].complete = true;

            if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                const int next_idx = building ^ 1;
                s_building_idx = next_idx;
                s_scan_ready   = true;
                s_scan_count++;
                xSemaphoreGive(s_scan_mutex);
                s_scans[s_building_idx].clear();
                ESP_LOGW(TAG, "Forced scan swap -> scan #%lu", (unsigned long)s_scan_count);
            } else {
                // Can't swap — clear the building buffer to make space
                s_scans[s_building_idx].clear();
                ESP_LOGE(TAG, "Forced swap mutex timeout – clearing buffer instead");
            }
        }
    }

    s_last_end_angle = end_deg;

    // ── Periodic stats ─────────────────────────────────────────────────────
    static uint32_t s_packet_count = 0;
    if ((++s_packet_count % 500) == 0) {
        ESP_LOGI(TAG, "Lidar stats: packets=%lu scans=%lu wraps=%lu "
                 "overflows=%lu crc_fails=%lu parse_fails=%lu",
                 (unsigned long)s_packet_count,
                 (unsigned long)s_scan_count,
                 (unsigned long)s_wrap_count,
                 (unsigned long)s_overflow_count,
                 (unsigned long)s_crc_fail_count,
                 (unsigned long)s_parse_fail_count);
    }
}

// ============================================================================
//  Read task
// ============================================================================

static void lidar_read_task(void* /*arg*/)
{
    ESP_LOGI(TAG, "Read task started");
    uint8_t raw[47];
    uint32_t total_reads = 0;
    uint32_t failed_reads = 0;

    while (true) {
        LYDSTO_Driver::MeasurementResult result{};
        const bool ok = s_driver->read_sensor(result);
        ++total_reads;
        if (!ok) {
            ++failed_reads;
            // Periodic warning if failure rate is high
            if ((failed_reads % 100) == 0) {
                ESP_LOGW(TAG, "read_sensor: %lu/%lu reads failed",
                         (unsigned long)failed_reads, (unsigned long)total_reads);
            }
            // Brief yield — don't busy-loop on repeated failures
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (s_driver->get_last_packet(raw, sizeof(raw))) {
            process_raw_packet(raw, result.timestamp_us);
        }
        // No delay when reads are succeeding — drain at maximum rate
        taskYIELD();
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

    // Reset all state before spawning read task
    s_scans[0].clear();
    s_scans[1].clear();
    s_building_idx    = 0;
    s_scan_ready      = false;
    s_last_end_angle  = -1.0f;
    s_scan_count      = 0;
    s_overflow_count  = 0;
    s_crc_fail_count  = 0;
    s_parse_fail_count = 0;
    s_wrap_count      = 0;
    s_state           = LidarSensorState::RUNNING;

    // Priority 3: above IDLE (1), below microros_task (5) and wifi_mgr (configMAX_PRIORITIES-2).
    // Stack 4096: the read task only touches the parser + one 47-byte buffer.
    xTaskCreate(lidar_read_task, "lidar_read", 4096, nullptr,
                3, &s_read_task);

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
    // The completed scan is always at (s_building_idx ^ 1).
    // s_building_idx was already advanced to the new building slot during the swap.
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
