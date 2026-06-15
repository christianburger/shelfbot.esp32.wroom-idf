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

static const char* TAG     = "LidarSensor";
static const char* TAG_RAW = "raw_lidar";

// ============================================================================
//  Internal state
// ============================================================================

static LYDSTO_Driver*    s_driver       = nullptr;
static TaskHandle_t      s_read_task    = nullptr;
static LidarSensorState  s_state        = LidarSensorState::SETUP;

static LidarScan         s_scans[2];
static volatile int      s_building_idx = 0;
static volatile bool     s_scan_ready   = false;
static SemaphoreHandle_t s_scan_mutex   = nullptr;

// Diagnostic counters
static uint32_t s_scan_count      = 0;
static uint32_t s_overflow_count  = 0;
static uint32_t s_crc_fail_count  = 0;
static uint32_t s_parse_fail_count = 0;
static uint32_t s_wrap_count      = 0;

// ============================================================================
//  Timestamp-based angular velocity tracker
//
//  Why timestamps instead of the encoder angle pair for per-measurement angles:
//
//  The LYDSTO sends one start_angle and one end_angle for a batch of 12
//  measurements.  Dividing the arc evenly across 12 samples (uniform
//  reconstruction) is correct on average but carries the quantisation error
//  of the encoder update rate (~13.8° per packet).  The sensor's internal
//  clock, however, runs the motor at a stable speed — the angular velocity
//  is nearly constant over many revolutions and can be estimated precisely
//  from the wall-clock timestamps of successive revolution boundaries.
//
//  Once angular_velocity_deg_per_us is known, the angle of measurement i
//  within a packet is:
//
//    angle_i = (t_packet_start + i * T_SAMPLE - t_revolution_start)
//              × angular_velocity_deg_per_us
//
//  where T_SAMPLE = UART transmission time for one 3-byte sample:
//    115200 baud, 10 bits/byte → 86.8 µs/byte → 3 bytes → ~260 µs/sample
//
//  This gives sub-packet angle resolution that improves with motor stability,
//  independent of the encoder's angular quantisation.
//
//  Bootstrap: for the first revolution we have no angular velocity estimate.
//  We fall back to uniform angular reconstruction from the encoder angles,
//  identical to the previous behaviour, and switch to timestamp-based
//  assignment once the first revolution completes.
// ============================================================================

// Time of the most recent revolution start (esp_timer_get_time(), microseconds)
static int64_t  s_rev_start_us         = -1;
// Estimated angular velocity from the last complete revolution (deg/µs)
static float    s_ang_vel_deg_per_us   = 0.0f;
// Whether we have a valid angular velocity estimate
static bool     s_ang_vel_valid        = false;
// Accumulated angle within the current revolution (used for wrap detection)
static float    s_accumulated_deg      = 0.0f;
// End angle of the previous packet (encoder value, for fallback and logging)
static float    s_last_end_angle       = -1.0f;

// UART: 115200 baud, 10 bits per byte (8N1).
// One 3-byte measurement field takes 3 × (1/115200 × 10) = 260.4 µs.
static constexpr float BYTES_PER_SAMPLE    = 3.0f;
static constexpr float BAUD_RATE           = 115200.0f;
static constexpr float US_PER_SAMPLE       =
    BYTES_PER_SAMPLE * 10.0f / BAUD_RATE * 1e6f;   // ≈ 260 µs

// ============================================================================
//  Angle assignment
// ============================================================================

/**
 * @brief Assign physical angles to all 12 measurements in a packet.
 *
 * Primary method (when angular velocity is known):
 *   Uses the wall-clock timestamp of the packet read and the estimated
 *   angular velocity to place each measurement at its true angular position
 *   within the current revolution.
 *
 * Fallback (first revolution, no velocity estimate):
 *   Uniformly reconstructs angles from the encoder-reported start/end pair.
 *   This is NOT interpolation between estimated values — the start/end angles
 *   are the encoder's record of where the batch began and ended; distributing
 *   12 samples uniformly between them recovers the physically correct angles
 *   because the sensor hardware samples at equal angular intervals.
 *
 * @param parsed        Decoded packet (angles in degrees, timestamp in ms).
 * @param packet_time_us  Wall-clock time at which this packet was read
 *                        (esp_timer_get_time(), microseconds).
 * @param out_angles    Output array of 12 angles in degrees [0, 360).
 */
static void assign_angles(const LidarParsedPacket& parsed,
                           int64_t packet_time_us,
                           float out_angles[12])
{
    if (s_ang_vel_valid && s_rev_start_us >= 0) {
        // ── Primary: timestamp-based assignment ───────────────────────────
        for (int i = 0; i < 12; ++i) {
            const int64_t t_sample_us = packet_time_us +
                static_cast<int64_t>(i * US_PER_SAMPLE);
            float angle = static_cast<float>(t_sample_us - s_rev_start_us)
                          * s_ang_vel_deg_per_us;
            // Clamp to [0, 360) — small overshoots can occur near the boundary
            while (angle >= 360.0f) angle -= 360.0f;
            while (angle <    0.0f) angle += 360.0f;
            out_angles[i] = angle;
        }
    } else {
        // ── Fallback: uniform reconstruction from encoder angles ──────────
        // The sensor records start_angle and end_angle from its optical
        // encoder for the batch.  The 12 samples were taken at equal angular
        // steps between these two positions — distributing them uniformly
        // recovers the correct physical angles.
        float span = parsed.end_angle_deg - parsed.start_angle_deg;
        if (span >  180.0f) span -= 360.0f;
        if (span < -180.0f) span += 360.0f;
        for (int i = 0; i < 12; ++i) {
            float angle = parsed.start_angle_deg +
                span * static_cast<float>(i) / 11.0f;
            while (angle >= 360.0f) angle -= 360.0f;
            while (angle <    0.0f) angle += 360.0f;
            out_angles[i] = angle;
        }
    }
}

// ============================================================================
//  Point accumulation
// ============================================================================

static bool accumulate_points(LidarScan& scan, const LidarParsedPacket& parsed,
                               const float angles_deg[12], int64_t timestamp_us)
{
    if (scan.point_count == 0) {
        scan.start_time_us = timestamp_us;
    }
    bool overflow = false;
    for (int i = 0; i < 12; ++i) {
        if (scan.point_count >= LidarScan::MAX_POINTS) {
            overflow = true;
            continue;
        }
        const uint16_t idx     = scan.point_count;
        scan.distances_mm[idx] = parsed.distances_mm[i];
        scan.confidences[idx]  = parsed.confidences[i];
        scan.angles_deg[idx]   = angles_deg[i];
        scan.point_count++;
    }
    scan.end_time_us = timestamp_us;
    return !overflow;
}

// ============================================================================
//  Revolution boundary detection and swap
// ============================================================================

static void complete_revolution(int64_t wrap_time_us, uint16_t completed_pts)
{
    // Update angular velocity from this revolution's duration
    if (s_rev_start_us >= 0) {
        const int64_t rev_duration_us = wrap_time_us - s_rev_start_us;
        if (rev_duration_us > 50000 && rev_duration_us < 500000) {
            // Sanity: 50 ms < duration < 500 ms (2–20 Hz)
            const float new_vel = 360.0f / static_cast<float>(rev_duration_us);
            // Low-pass filter: blend 20% new, 80% old to damp noise
            s_ang_vel_deg_per_us = s_ang_vel_valid
                ? (0.8f * s_ang_vel_deg_per_us + 0.2f * new_vel)
                : new_vel;
            s_ang_vel_valid = true;

            ESP_LOGD(TAG_RAW,
                     "rev complete: dur=%lldus vel=%.4f deg/us pts=%u",
                     (long long)rev_duration_us,
                     s_ang_vel_deg_per_us,
                     (unsigned)completed_pts);
        } else {
            ESP_LOGW(TAG, "Implausible revolution duration %lldus – ignoring",
                     (long long)rev_duration_us);
        }
    }
    s_rev_start_us      = wrap_time_us;
    s_accumulated_deg   = 0.0f;

    // Swap double buffer
    const int building = s_building_idx;
    s_scans[building].complete = true;

    if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_building_idx = building ^ 1;
        s_scan_ready   = true;
        s_scan_count++;
        xSemaphoreGive(s_scan_mutex);
        s_scans[s_building_idx].clear();

        const float dt_ms = (s_scans[building].end_time_us > s_scans[building].start_time_us)
            ? static_cast<float>(s_scans[building].end_time_us -
                                  s_scans[building].start_time_us) / 1000.0f
            : 0.0f;
        ESP_LOGI(TAG, "scan #%lu complete: pts=%u dt=%.1fms vel=%.4fdeg/us "
                 "wraps=%lu overflows=%lu crc_fails=%lu",
                 (unsigned long)s_scan_count,
                 (unsigned)completed_pts,
                 dt_ms,
                 s_ang_vel_deg_per_us,
                 (unsigned long)s_wrap_count,
                 (unsigned long)s_overflow_count,
                 (unsigned long)s_crc_fail_count);

        if (completed_pts < 100) {
            ESP_LOGW(TAG, "Sparse scan #%lu: only %u points (expected ~150+)",
                     (unsigned long)s_scan_count, (unsigned)completed_pts);
        }
    } else {
        ESP_LOGE(TAG, "Scan mutex timeout during swap – scan lost (wrap #%lu, pts=%u)",
                 (unsigned long)s_wrap_count, (unsigned)completed_pts);
    }
}

// ============================================================================
//  Packet processing
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
        ESP_LOGD(TAG, "CRC mismatch – dropping (total=%lu)", (unsigned long)s_crc_fail_count);
        return;
    }

    // ── Assign angles ──────────────────────────────────────────────────────
    float angles[12];
    assign_angles(parsed, timestamp_us, angles);

    // ── Wrap detection ─────────────────────────────────────────────────────
    // Primary: track accumulated angle within the revolution.
    // When it reaches or exceeds 360° we have completed one revolution.
    //
    // The angular span of this packet (from encoder or from velocity estimate):
    float pkt_span;
    if (s_ang_vel_valid) {
        // Use velocity × time for the 12-sample window
        pkt_span = s_ang_vel_deg_per_us * (12.0f * US_PER_SAMPLE);
    } else {
        // Fallback: use encoder span, handling wrap
        pkt_span = parsed.end_angle_deg - parsed.start_angle_deg;
        if (pkt_span <   0.0f) pkt_span += 360.0f;
        if (pkt_span > 180.0f) pkt_span  = 360.0f - pkt_span; // safety
    }

    // Also log encoder-based wrap signals for diagnostics
    const bool encoder_wrap = (s_last_end_angle >= 0.0f) &&
                              (parsed.start_angle_deg < s_last_end_angle) &&
                              ((360.0f - s_last_end_angle + parsed.start_angle_deg) < 120.0f);

    s_accumulated_deg += pkt_span;

    const bool accumulated_wrap = (s_accumulated_deg >= 360.0f);

    if (accumulated_wrap || encoder_wrap) {
        if (accumulated_wrap != encoder_wrap) {
            // Log when the two methods disagree — useful for tuning
            ESP_LOGD(TAG_RAW,
                     "wrap detection mismatch: accumulated=%s encoder=%s "
                     "accum_deg=%.1f last_end=%.2f new_start=%.2f",
                     accumulated_wrap ? "YES" : "no",
                     encoder_wrap     ? "YES" : "no",
                     s_accumulated_deg,
                     s_last_end_angle,
                     parsed.start_angle_deg);
        }
        ++s_wrap_count;
        complete_revolution(timestamp_us, s_scans[s_building_idx].point_count);
    }

    s_last_end_angle = parsed.end_angle_deg;

    // ── Accumulate points ─────────────────────────────────────────────────
    const bool ok = accumulate_points(s_scans[s_building_idx], parsed,
                                       angles, timestamp_us);
    if (!ok) {
        ++s_overflow_count;
        if ((s_overflow_count % 10) == 1) {
            ESP_LOGW(TAG, "Scan buffer overflow #%lu (pts=%u/%u accum=%.1fdeg)",
                     (unsigned long)s_overflow_count,
                     (unsigned)s_scans[s_building_idx].point_count,
                     (unsigned)LidarScan::MAX_POINTS,
                     s_accumulated_deg);
        }
        // Force swap — buffer full
        if (s_scans[s_building_idx].point_count >= LidarScan::MAX_POINTS) {
            ESP_LOGW(TAG, "Buffer full – forcing scan swap");
            ++s_wrap_count;
            complete_revolution(timestamp_us, s_scans[s_building_idx].point_count);
        }
    }

    // ── Periodic stats ─────────────────────────────────────────────────────
    static uint32_t s_packet_count = 0;
    if ((++s_packet_count % 500) == 0) {
        ESP_LOGI(TAG, "stats: pkts=%lu scans=%lu vel=%.4fdeg/us "
                 "wraps=%lu overflows=%lu crc_fails=%lu parse_fails=%lu",
                 (unsigned long)s_packet_count,
                 (unsigned long)s_scan_count,
                 s_ang_vel_deg_per_us,
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
    uint8_t  raw[47];
    uint32_t total_reads  = 0;
    uint32_t failed_reads = 0;

    while (true) {
        LYDSTO_Driver::MeasurementResult result{};
        const bool ok = s_driver->read_sensor(result);
        ++total_reads;
        if (!ok) {
            ++failed_reads;
            if ((failed_reads % 100) == 0) {
                ESP_LOGW(TAG, "read_sensor: %lu/%lu failed",
                         (unsigned long)failed_reads, (unsigned long)total_reads);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (s_driver->get_last_packet(raw, sizeof(raw))) {
            process_raw_packet(raw, result.timestamp_us);
        }
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
        s_state  = LidarSensorState::ERROR;
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
    s_building_idx      = 0;
    s_scan_ready        = false;
    s_scan_count        = 0;
    s_overflow_count    = 0;
    s_crc_fail_count    = 0;
    s_parse_fail_count  = 0;
    s_wrap_count        = 0;
    s_rev_start_us      = -1;
    s_ang_vel_deg_per_us = 0.0f;
    s_ang_vel_valid     = false;
    s_accumulated_deg   = 0.0f;
    s_last_end_angle    = -1.0f;
    s_state             = LidarSensorState::RUNNING;

    xTaskCreate(lidar_read_task, "lidar_read", 4096, nullptr, 3, &s_read_task);
    vTaskDelete(nullptr);
}

// ============================================================================
//  Public API
// ============================================================================

void lidar_setup()
{
    ESP_LOGI(TAG, "lidar_setup");
    s_scan_mutex = xSemaphoreCreateMutex();
    configASSERT(s_scan_mutex);
    s_state = LidarSensorState::SETUP;
    xTaskCreate(lidar_lifecycle_task, "lidar_lifecycle", 4096, nullptr, 3, nullptr);
}

void lidar_stop()
{
    if (s_read_task) { vTaskDelete(s_read_task); s_read_task = nullptr; }
    if (s_driver)    { delete s_driver;           s_driver    = nullptr; }
    s_state = LidarSensorState::STOPPED;
    ESP_LOGI(TAG, "Stopped");
}

LidarSensorState lidar_get_state()  { return s_state; }
bool             lidar_is_running() { return s_state == LidarSensorState::RUNNING; }

bool lidar_get_latest_scan(LidarScan& out)
{
    if (!s_scan_ready) return false;
    if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    if (!s_scan_ready) { xSemaphoreGive(s_scan_mutex); return false; }
    out = s_scans[s_building_idx ^ 1];
    s_scan_ready = false;
    xSemaphoreGive(s_scan_mutex);
    return true;
}

bool     lidar_get_last_raw_packet(uint8_t* out, size_t len) {
    return s_driver ? s_driver->get_last_packet(out, len) : false;
}
uint32_t lidar_get_packet_count() { return s_driver ? s_driver->get_packet_count() : 0u; }
uint32_t lidar_get_scan_count()   { return s_scan_count; }
