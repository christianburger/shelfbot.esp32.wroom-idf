// lidar_sensor.cpp
//
// Task model changes
// ──────────────────
// • lidar_read_task_fn runs until s_stop_requested is set, then calls
//   vTaskDelete(nullptr) on itself.  lidar_stop() sets the flag and gives
//   the task ~20 ms to exit before releasing the driver handle.
//   This replaces the previous vTaskDelete(s_read_task) which was unsafe:
//   deleting a task from outside while it may hold a mutex or be blocked
//   inside UART I/O leaves primitives permanently locked.
//
// • lidar_lifecycle_task_fn is a one-shot init task that terminates itself
//   with vTaskDelete(nullptr) on both the error path and on successful
//   completion (once RUNNING is reached the read task is already running and
//   the lifecycle task has nothing more to do).
//
// Advance / retry model
// ─────────────────────
// Both while-loops in lidar_lifecycle_task_fn call advance() and delay 1 s
// on every iteration.  If advance() returns false because shelfbot is not yet
// RUNNING (prerequisite for SETUP→INIT), the task simply keeps retrying.
// No manual prerequisite check is needed — advance() reports the block via
// LOGD and the caller just waits.
//
// Memory model — unchanged
// ────────────────────────
// • LYDSTO_Driver is a static instance (BSS); no heap allocation.
// • LidarScan double-buffer (s_scans[2]) is static BSS.
// • s_scan_mutex is created in lidar_setup() before any task runs.

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
static constexpr int64_t REVOLUTION_PERIOD_US = 167000LL;
static constexpr int64_t MAX_SCAN_DURATION_US = (REVOLUTION_PERIOD_US * 3) / 2;

// ============================================================================
//  Internal state — all static (BSS), no heap
// ============================================================================

static LYDSTO_Driver s_driver_instance(LIDAR_UART_PORT,
                                       LIDAR_TX_PIN,
                                       LIDAR_RX_PIN,
                                       LIDAR_BAUD_RATE);
static LYDSTO_Driver*   s_driver        = nullptr;
static TaskHandle_t     s_read_task     = nullptr;
static LidarSensorState s_state         = LidarSensorState::SETUP;

// Stop flag for the read task.  Set by lidar_stop(); checked by
// lidar_read_task_fn on every iteration.  Declared volatile so the compiler
// does not cache it in a register across the loop body.
static volatile bool     s_stop_requested = false;

static LidarScan         s_scans[2];
static volatile int      s_building_idx  = 0;
static volatile bool     s_scan_ready    = false;
static SemaphoreHandle_t s_scan_mutex    = nullptr;

static uint32_t s_scan_count        = 0;
static uint32_t s_overflow_count    = 0;
static uint32_t s_crc_fail_count    = 0;
static uint32_t s_parse_fail_count  = 0;
static uint32_t s_wrap_count        = 0;
static uint32_t s_time_wrap_count   = 0;
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

static bool is_wrap_around(float prev_end_deg, float new_start_deg)
{
    if (new_start_deg >= prev_end_deg) return false;
    static constexpr float WRAP_GAP_LIMIT_DEG = 300.0f;
    const float forward_gap = 360.0f - prev_end_deg + new_start_deg;
    return forward_gap < WRAP_GAP_LIMIT_DEG;
}

// ============================================================================
//  Scan completion
// ============================================================================

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

    if (s_last_end_angle >= 0.0f && is_wrap_around(s_last_end_angle, start_deg)) {
        ++s_wrap_count;
        complete_scan("angular", start_deg);
    } else {
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
        if (s_scans[s_building_idx].point_count >= LidarScan::MAX_POINTS) {
            ++s_time_wrap_count;
            complete_scan("overflow", start_deg);
        }
    }

    s_last_end_angle = end_deg;

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
//  lidar_read_task_fn  (exposed for shelfbot.cpp task creation)
//
//  Stack budget: 2560 words (10 KB) — tight UART read loop, no std::string.
//
//  Termination: checks s_stop_requested on every iteration.  When set (by
//  lidar_stop()), the task clears s_read_task and calls vTaskDelete(nullptr)
//  to delete itself cleanly.  This is the ONLY safe way to terminate a task
//  that may be blocked inside UART I/O — external vTaskDelete() is unsafe
//  because it can leave the UART driver mutex permanently locked.
// ============================================================================
void lidar_read_task_fn(void*) {
    ESP_LOGI(TAG, "Read task started");

    uint8_t raw[47];
    uint32_t total_reads  = 0;
    uint32_t failed_reads = 0;

    while (!s_stop_requested) {
        // Gate: don't attempt reads until the lifecycle task has initialised
        // the driver and set s_state = RUNNING.
        if (!lidar_is_running()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        LYDSTO_Driver::MeasurementResult result{};
        const bool ok = s_driver->read_sensor(result);
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

    // Stop was requested — exit cleanly.
    ESP_LOGI(TAG, "Read task: stop requested, exiting (reads=%lu failed=%lu)",
             (unsigned long)total_reads, (unsigned long)failed_reads);
    s_read_task = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
//  lidar_lifecycle_task_fn  (exposed for shelfbot.cpp task creation)
//
//  Stack budget: 2560 words (10 KB).
//
//  This is a ONE-SHOT task: it advances the state machine from SETUP to
//  RUNNING, waits for the driver to initialise, then calls vTaskDelete(nullptr)
//  because it has nothing more to do.  The actual ongoing work is done by
//  lidar_read_task_fn (a permanent task).
//
//  Retry model: advance() returns false when prerequisites are not yet met
//  (shelfbot must be RUNNING for SETUP→INIT).  The task delays 1 s and retries
//  — this is the correct pattern for a condition that changes asynchronously.
// ============================================================================
void lidar_lifecycle_task_fn(void* /*arg*/)
{
    static constexpr uint32_t RETRY_MS = 1000;

    // SETUP → INIT
    // Prerequisite: shelfbot >= RUNNING.
    // Retries every RETRY_MS until satisfied.
    while (!StateMachine::isInState("lidar_sensor", stateToString(LidarSensorState::INIT))) {
        StateMachine::advance("lidar_sensor");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "State: INIT");

    // Initialise the driver (static instance — no heap allocation).
    s_driver = &s_driver_instance;
    const char* init_err = s_driver->init();
    if (init_err) {
        ESP_LOGE(TAG, "LYDSTO init failed: %s (free heap=%u)",
                 init_err, (unsigned)esp_get_free_heap_size());
        s_driver = nullptr;
        s_state = LidarSensorState::ERROR;
        StateMachine::changeState("lidar_sensor",
            stateToString(LidarSensorState::ERROR), true);
        StateMachine::recover();
        vTaskDelete(nullptr);
        return;
    }

    // INIT → RUNNING
    // No prerequisites — should succeed immediately.
    while (!StateMachine::isInState("lidar_sensor", stateToString(LidarSensorState::RUNNING))) {
        StateMachine::advance("lidar_sensor");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "State: RUNNING — read task already created by shelfbot");

    // Reset accumulator state.
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

    // lidar_read_task_fn is already running (created by shelfbot.cpp) and
    // will start producing valid packets now that s_driver is set and
    // s_state == RUNNING.
    //
    // This one-shot lifecycle task is done; delete itself.
    vTaskDelete(nullptr);
}

// ============================================================================
//  Public API
// ============================================================================

void lidar_setup()
{
    ESP_LOGI(TAG, "lidar_setup (free heap=%u)", (unsigned)esp_get_free_heap_size());
    s_scan_mutex      = xSemaphoreCreateMutex();
    s_stop_requested  = false;
    configASSERT(s_scan_mutex);
    s_state = LidarSensorState::SETUP;
}

// ---------------------------------------------------------------------------
// lidar_stop — signal the read task to exit cleanly.
//
// Sets s_stop_requested = true and waits briefly for lidar_read_task_fn to
// notice and call vTaskDelete(nullptr) on itself.  We do NOT call
// vTaskDelete(s_read_task) from here: if the task is blocked inside
// uart_read_bytes() or holding the UART ISR spinlock, an external delete
// leaves those resources permanently locked.
//
// The 50 ms wait is generous: the read loop either exits uart_read_bytes()
// within its 20 ms blocking timeout, or returns immediately from taskYIELD().
// ---------------------------------------------------------------------------
void lidar_stop()
{
    if (s_read_task == nullptr && !lidar_is_running()) {
        ESP_LOGD(TAG, "lidar_stop: already stopped");
        return;
    }

    ESP_LOGI(TAG, "lidar_stop: requesting read task stop");
    s_stop_requested = true;

    // Give the read task up to 100 ms to notice the flag and exit.
    // The UART read timeout is 20 ms, so one or two iterations suffice.
    for (int i = 0; i < 10 && s_read_task != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (s_read_task != nullptr) {
        ESP_LOGW(TAG, "lidar_stop: read task did not exit within 100 ms — "
                 "handle may be stale (task may have already self-deleted)");
    }

    // Driver dereference is now safe: the read task is no longer calling
    // s_driver->read_sensor().
    s_driver = nullptr;
    s_state  = LidarSensorState::STOPPED;
    ESP_LOGI(TAG, "Stopped");
}

void lidar_set_read_task_handle(TaskHandle_t h)
{
    s_read_task = h;
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
