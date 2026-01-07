#include "shelfbot.hpp"
#include "sensor_control.hpp"
#include "motor_control.hpp"
#include "esp_log.h"

// The ESP-IDF framework requires a C-style `app_main` entry point.
// We use `extern "C"` to prevent C++ name mangling.
extern "C" void app_main(void)
{
    ESP_LOGI("app_main", "App starting...");
    Shelfbot shelfbot;
    ESP_LOGI("app_main", "Shelfbot object created.");
    shelfbot.begin();
    ESP_LOGI("app_main", "Shelfbot begin() returned. Entering safety loop.");

    // This loop is now responsible for handling critical safety events
    // int stop_signal;
    // while (1) {
    //     // Wait indefinitely for a message on the motor_stop_queue
    //     if (xQueueReceive(motor_stop_queue, &stop_signal, portMAX_DELAY) == pdPASS) {
    //         ESP_LOGW("app_main", "EMERGENCY STOP SIGNAL RECEIVED! Stopping all motors.");
    //         motor_control_stop_all_motors();
    //     }
    // }
    // For this test, we will revert to the old behavior of just sleeping.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
