// lidar_sensor.cpp
//
// Memory model changes
// ────────────────────
// • LYDSTO_Driver is now a static instance (BSS) instead of heap-allocated.
//   new/delete removed; ~13 bytes of pointer + vtable overhead gone, but
//   more importantly we never call operator new which could fail at runtime.
//
// • lidar_lifecycle_task and lidar_read_task are no longer spawned here.
//   Their function pointers (lidar_lifecycle_task_fn / lidar_read_task_fn)
//   are exposed via lidar_sensor.hpp so shelfbot.cpp can create all tasks in
//   one place with consistent stack sizing and error handling.
//
// • s_scan_mutex is created in lidar_setup() before any task runs — safe
//   because lidar_setup() is called from shelfbot.cpp before tasks start.

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

// Static driver instance — no heap allocation
static LYDSTO_Driver s_driver_instance(LIDAR_UART_PORT,
                                       LIDAR_TX_PIN,
                                       LIDAR_RX_PIN,
                                       LIDAR_BAUD_RATE);
static LYDSTO_Driver*   s_driver        = nullptr;   // set to &s_driver_instance after init
static TaskHandle_t     s_read_task     = nullptr;
static LidarSensorState s_state         = LidarSensorState::SETUP;

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
    if (new_start_deg >= prev_end_deg) {
        return false;
    }
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

//  Read task (exposed for shelfbot.cpp task creation)
void lidar_read_task_fn(void*) {
  ESP_LOGI(TAG, "Read task started");

  uint8_t raw[47];
  uint32_t total_reads  = 0;
  uint32_t failed_reads = 0;

  while (true) {
    // Wait until the state machine says we are RUNNING
    if (!lidar_is_running()) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // At this point s_driver is guaranteed to be non-NULL
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
  vTaskDelete(nullptr);
}

// ============================================================================
//  Lifecycle task (exposed for shelfbot.cpp task creation)
// ============================================================================

void lidar_lifecycle_task_fn(void* /*arg*/)
{
    static constexpr uint32_t RETRY_MS = 1000;

    while (!StateMachine::isInState("lidar_sensor", stateToString(LidarSensorState::INIT))) {
        StateMachine::advance("lidar_sensor");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "State: INIT");

    // Use the static driver instance — no heap allocation
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

    while (!StateMachine::isInState("lidar_sensor", stateToString(LidarSensorState::RUNNING))) {
        StateMachine::advance("lidar_sensor");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "State: RUNNING — read task already created by shelfbot");

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

    // Note: lidar_read_task_fn is already running (created by shelfbot.cpp)
    // and will start producing valid packets now that s_driver is set and
    // s_state == RUNNING.

    vTaskDelete(nullptr);
}

// ============================================================================
//  Public API
// ============================================================================

void lidar_setup()
{
    ESP_LOGI(TAG, "lidar_setup (free heap=%u)", (unsigned)esp_get_free_heap_size());
    s_scan_mutex = xSemaphoreCreateMutex();
    configASSERT(s_scan_mutex);
    s_state = LidarSensorState::SETUP;
    // Tasks are created by shelfbot.cpp — do NOT spawn here.
}

void lidar_stop()
{
    if (s_read_task) {
        vTaskDelete(s_read_task);
        s_read_task = nullptr;
    }
    // Static driver — just mark uninitialised, destructor called at program end
    s_driver = nullptr;
    s_state = LidarSensorState::STOPPED;
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
    // Do NOT clear s_scan_ready here — the HTTP server and micro-ROS both
    // poll this independently.  The lidar_sensor produces a new scan every
    // ~167 ms which will overwrite the slot naturally.  Clearing it here
    // would cause the HTTP server or micro-ROS to miss every other scan.
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
