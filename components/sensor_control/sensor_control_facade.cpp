#include "sensor_control_facade.hpp"
#include <vector>
#include "esp_log.h"
#include "freertos/task.h"

static const char* TAG = "SensorControlFacade";

// ---------------------------------------------------------------------------
// Singleton accessor
// ---------------------------------------------------------------------------
SensorControlFacade& SensorControlFacade::instance() {
    static SensorControlFacade the_one;
    return the_one;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void SensorControlFacade::init() {
    auto& self = instance();

    if (self.sensor_control_) {
        ESP_LOGW(TAG, "init() called twice — ignored");
        return;
    }

    self.mutex_ = xSemaphoreCreateMutex();
    if (!self.mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Configure two ultrasonic sensors.  Pin assignments mirror what the rest
    // of the project already uses; adjust here if hardware changes.
    SensorControl::Config cfg;
    cfg.ultrasonic_configs = {
        { .trig_pin = 2,  .echo_pin = 4,  .timeout_us = 30000, .max_distance_mm = 5000 },
        { .trig_pin = 5,  .echo_pin = 6,  .timeout_us = 30000, .max_distance_mm = 5000 }
    };
    cfg.ultrasonic_read_interval_ms = 100;
    cfg.tof_read_interval_ms        = 200;

    self.sensor_control_ = std::make_unique<SensorControl>(cfg);

    esp_err_t err = self.sensor_control_->initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SensorControl::initialize failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "SensorControl initialised");
    }
}

void SensorControlFacade::start_task() {
    auto& self = instance();

    if (!self.sensor_control_) {
        ESP_LOGE(TAG, "start_task() before init()");
        return;
    }
    if (self.task_handle_) {
        ESP_LOGW(TAG, "start_task() called twice — ignored");
        return;
    }

    BaseType_t ok = xTaskCreate(
        task_entry,
        "sensor_facade_poll",
        4096,
        &self,
        tskIDLE_PRIORITY + 2,
        &self.task_handle_
    );

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        self.task_handle_ = nullptr;
    } else {
        ESP_LOGI(TAG, "Polling task started");
    }
}

bool SensorControlFacade::get_latest_data(SensorDataPacket& out) {
    auto& self = instance();
    if (!self.mutex_) return false;

    if (xSemaphoreTake(self.mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    out = self.latest_;                         // plain struct copy
    xSemaphoreGive(self.mutex_);

    return out.data_ready;
}

// ---------------------------------------------------------------------------
// Background polling task
// ---------------------------------------------------------------------------
void SensorControlFacade::task_entry(void* arg) {
    static_cast<SensorControlFacade*>(arg)->run();
    vTaskDelete(nullptr);   // should never reach here, but be safe
}

void SensorControlFacade::run() {
    constexpr TickType_t POLL_PERIOD = pdMS_TO_TICKS(100);

    while (true) {
        // --- read ultrasonic -------------------------------------------
        std::vector<uint16_t> us_dist;
        esp_err_t us_err = sensor_control_->read_ultrasonic(us_dist);

        // --- read ToF --------------------------------------------------
        uint16_t tof_dist_mm = 0;
        bool     tof_valid   = false;
        uint8_t  tof_status  = 0;
        esp_err_t tof_err    = sensor_control_->read_tof(tof_dist_mm, tof_valid, tof_status);

        // --- build packet under lock ------------------------------------
        SensorDataPacket pkt{};

        if (us_err == ESP_OK) {
            for (int i = 0; i < NUM_ULTRASONIC_SENSORS && i < static_cast<int>(us_dist.size()); ++i) {
                pkt.ultrasonic_distances_cm[i] = static_cast<float>(us_dist[i]) / 10.0f;  // mm -> cm
                pkt.ultrasonic_valid[i]        = (us_dist[i] > 0);
            }
        }

        if (tof_err == ESP_OK) {
            pkt.tof_distances_cm[0] = static_cast<float>(tof_dist_mm) / 10.0f;  // mm -> cm
            pkt.tof_valid[0]        = tof_valid;
        }

        pkt.data_ready = true;

        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            latest_ = pkt;
            xSemaphoreGive(mutex_);
        }

        vTaskDelay(POLL_PERIOD);
    }
}
