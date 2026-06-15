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
//  Timing constants
// ============================================================================

// Expected revolution period at nominal 6 Hz.
static constexpr int64_t REVOLUTION_PERIOD_US = 167000LL;  // 167 ms

// Force scan completion if the building buffer has been accumulating for
// longer than this.  1.5× revolution period leaves enough headroom for the
// sensor to run slightly slow while still catching a missed angular wrap.
// If CRC failures cause the angular wrap to be missed, this ensures we never
// accumulate more than ~1.5 revolutions into one scan.
static constexpr int64_t MAX_SCAN_DURATION_US = (REVOLUTION_PERIOD_US * 3) / 2;  // 250 ms

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
static uint32_t s_overflow_count    = 0;
static uint32_t s_crc_fail_count    = 0;
static uint32_t s_parse_fail_count  = 0;
static uint32_t s_wrap_count        = 0;
static uint32_t s_time_wrap_count   = 0;  // wraps triggered by time fallback
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
 * @brief Detect a revolution boundary between consecutive packets.
 *
 * The LYDSTO sends packets with monotonically increasing start_angle.
 * A revolution completes when start_angle wraps back past 0°, which appears
 * as new_start_deg < prev_end_deg in the angle stream.
 *
 * THRESHOLD RATIONALE
 * ───────────────────
 * The angular gap threshold (forward_gap_limit) must be > 0° and < 360°.
 * Too small: CRC packet loss near 0°/360° can produce a forward gap larger
 *   than the threshold, causing wrap detection to MISS the revolution end.
 *   Example: packets at 200°→255° arrive, then CRC failures eat 255°→360°→20°,
 *   next valid packet at 20°: forward_gap = 360-255+20 = 125° > 120° → MISSED.
 *   With 120° threshold at 6Hz and ~144 packets/rev, losing > 48 consecutive
 *   packets (1/3 of a revolution) breaks detection.
 * Too large (> 360°): impossible.
 * Too close to 360° (e.g. 350°): an ordinary forward step of 2.5° between
 *   adjacent packets in the SAME revolution could be mistaken for a wrap if
 *   prev_end is very close to 0° (e.g. prev_end=2°, new_start=1° → ✓ wrap).
 *   Actually, within-revolution steps are always FORWARD (new > prev), so they
 *   never enter the wrap check at all (guarded by new_start < prev_end).
 *
 * 300° allows up to 300° of consecutive CRC packet loss (83% of a revolution)
 * while still correctly rejecting any in-revolution backward jitter.
 * The only false-positive risk is a 60°+ backward jump within a revolution,
 * which LYDSTO does not produce under normal operation.
 */
static bool is_wrap_around(float prev_end_deg, float new_start_deg)
{
    // If the new packet starts at or after where the last one ended, we're
    // still going forward in the same revolution — definitely not a wrap.
    if (new_start_deg >= prev_end_deg) {
        return false;
    }
    // The apparent backward step could be either:
    //   (a) a genuine 360°→0° revolution wrap, or
    //   (b) a large-gap backward anomaly from the sensor.
    // We distinguish by the "forward arc distance" around the wrap point:
    //   forward_gap = 360° - prev_end + new_start
    // For a genuine wrap this is always the small gap (< one revolution).
    // Threshold: allow up to 300° of packet loss around the wrap boundary.
    static constexpr float WRAP_GAP_LIMIT_DEG = 300.0f;
    const float forward_gap = 360.0f - prev_end_deg + new_start_deg;
    return forward_gap < WRAP_GAP_LIMIT_DEG;
}

// ============================================================================
//  Scan completion (common path for angular wrap and time-based fallback)
// ============================================================================

// Called only from lidar_read_task (single writer).
// reason: "angular" or "time" for the log.
static void complete_scan(const char* reason, float trigger_start_deg)
{
    const int building = s_building_idx;
    s_scans[building].complete = true;

    const uint16_t completed_pts = s_scans[building].point_count;
    const float dt_ms = (s_scans[building].end_time_us > s_scans[building].start_time_us)
        ? static_cast<float>(s_scans[building].end_time_us - s_scans[building].start_time_us)
          / 1000.0f
        : 0.0f;

    if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        const int next_idx = building ^ 1;
        s_building_idx = next_idx;
        s_scan_ready   = true;
        s_scan_count++;
        xSemaphoreGive(s_scan_mutex);

        // Clear the new building buffer now that the completed one is published.
        s_scans[s_building_idx].clear();

        ESP_LOGI(TAG,
            "scan #%lu [%s] pts=%u dt=%.1fms last_end=%.1f° next_start=%.1f° "
            "wraps=%lu time_wraps=%lu crc_fails=%lu overflows=%lu",
            (unsigned long)s_scan_count, reason,
            (unsigned)completed_pts, dt_ms,
            s_last_end_angle, trigger_start_deg,
            (unsigned long)s_wrap_count,
            (unsigned long)s_time_wrap_count,
            (unsigned long)s_crc_fail_count,
            (unsigned long)s_overflow_count);

        if (completed_pts < 100) {
            ESP_LOGW(TAG,
                "Sparse scan #%lu: only %u points (expected ~150+). "
                "CRC fails=%lu — check UART signal integrity.",
                (unsigned long)s_scan_count,
                (unsigned)completed_pts,
                (unsigned long)s_crc_fail_count);
        }
    } else {
        ESP_LOGE(TAG,
            "Scan mutex timeout during %s swap – scan lost! "
            "(pts=%u last_end=%.1f° next_start=%.1f°)",
            reason, (unsigned)completed_pts,
            s_last_end_angle, trigger_start_deg);
        // Don't swap — keep accumulating to avoid losing partial data.
    }
}

// ============================================================================
//  Point accumulation
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
        ESP_LOGD(TAG, "CRC mismatch – dropping packet (total=%lu)",
                 (unsigned long)s_crc_fail_count);
        return;
    }

    const float start_deg = parsed.start_angle_deg;
    const float end_deg   = parsed.end_angle_deg;

    // ── 1. Angular revolution boundary detection ──────────────────────────
    if (s_last_end_angle >= 0.0f && is_wrap_around(s_last_end_angle, start_deg)) {
        ++s_wrap_count;
        complete_scan("angular", start_deg);
    }

    // ── 2. Time-based fallback ────────────────────────────────────────────
    // If no angular wrap was detected but the building scan has been running
    // for longer than MAX_SCAN_DURATION_US, the wrap was likely missed due to
    // CRC failures eating >300° of packets around the 0°/360° boundary.
    // Force completion so we never accumulate more than ~1.5 revolutions.
    //
    // Only triggered if:
    //   - We have at least a few points (scan is actually running), and
    //   - The scan duration has exceeded the limit, and
    //   - We have NOT already just done an angular wrap this packet (the
    //     angular wrap check above already called complete_scan if needed).
    else {
        const int bidx = s_building_idx;
        if (s_scans[bidx].point_count > 0 &&
            s_scans[bidx].start_time_us > 0 &&
            (timestamp_us - s_scans[bidx].start_time_us) > MAX_SCAN_DURATION_US)
        {
            ++s_time_wrap_count;
            ESP_LOGW(TAG,
                "Time-based scan wrap (scan_dur=%.1fms > %.1fms, "
                "last_end=%.1f° cur_start=%.1f°, "
                "pts=%u time_wraps=%lu)",
                (float)(timestamp_us - s_scans[bidx].start_time_us) / 1000.0f,
                (float)MAX_SCAN_DURATION_US / 1000.0f,
                s_last_end_angle, start_deg,
                (unsigned)s_scans[bidx].point_count,
                (unsigned long)s_time_wrap_count);
            complete_scan("time", start_deg);
        }
    }

    // ── 3. Accumulate into current building buffer ────────────────────────
    const bool ok = accumulate_points(s_scans[s_building_idx], parsed, timestamp_us);
    if (!ok) {
        ++s_overflow_count;
        if ((s_overflow_count % 10) == 1) {
            ESP_LOGW(TAG,
                "Scan buffer full (overflow #%lu, pts=%u/%u, "
                "last_end=%.1f° cur_start=%.1f°) — forcing time wrap",
                (unsigned long)s_overflow_count,
                (unsigned)s_scans[s_building_idx].point_count,
                (unsigned)LidarScan::MAX_POINTS,
                s_last_end_angle, start_deg);
        }
        // Buffer full is also treated as a forced completion, same as time wrap.
        if (s_scans[s_building_idx].point_count >= LidarScan::MAX_POINTS) {
            ++s_time_wrap_count;
            complete_scan("overflow", start_deg);
        }
    }

    s_last_end_angle = end_deg;

    // ── 4. Periodic stats ─────────────────────────────────────────────────
    static uint32_t s_packet_count = 0;
    if ((++s_packet_count % 500) == 0) {
        ESP_LOGI(TAG,
            "Lidar stats: packets=%lu scans=%lu wraps=%lu "
            "time_wraps=%lu overflows=%lu crc_fails=%lu parse_fails=%lu",
            (unsigned long)s_packet_count,
            (unsigned long)s_scan_count,
            (unsigned long)s_wrap_count,
            (unsigned long)s_time_wrap_count,
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
    uint32_t total_reads  = 0;
    uint32_t failed_reads = 0;

    while (true) {
        LYDSTO_Driver::MeasurementResult result{};

        // Capture timestamp BEFORE the blocking read so that if read_sensor()
        // blocks in phase 2 for up to 500ms, the timestamp still reflects
        // approximately when we asked for data.  After the call succeeds we
        // get the actual packet time from esp_timer.
        const bool ok = s_driver->read_sensor(result);
        // Capture timestamp immediately after the read returns so it reflects
        // when the packet was actually received (phase 1 path) or unblocked (phase 2).
        const int64_t rx_time_us = esp_timer_get_time();

        ++total_reads;
        if (!ok) {
            ++failed_reads;
            if ((failed_reads % 100) == 0) {
                ESP_LOGW(TAG, "read_sensor: %lu/%lu reads failed",
                         (unsigned long)failed_reads, (unsigned long)total_reads);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (s_driver->get_last_packet(raw, sizeof(raw))) {
            process_raw_packet(raw, rx_time_us);
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
    s_building_idx     = 0;
    s_scan_ready       = false;
    s_last_end_angle   = -1.0f;
    s_scan_count       = 0;
    s_overflow_count   = 0;
    s_crc_fail_count   = 0;
    s_parse_fail_count = 0;
    s_wrap_count       = 0;
    s_time_wrap_count  = 0;
    s_state            = LidarSensorState::RUNNING;

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
bool lidar_is_running()            { return s_state == LidarSensorState::RUNNING; }

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
