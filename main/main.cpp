#include <shelfbot.hpp>

extern "C" [[noreturn]] void app_main(void) {
    ESP_LOGI("app_main", "App starting...");
    Shelfbot& shelfbot = Shelfbot::get_instance();
    if (const esp_err_t err = shelfbot.begin(); err != ESP_OK) {
        ESP_LOGE("app_main", "Shelfbot initialisation failed: %s", esp_err_to_name(err));
        // Loop forever if init fails
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI("app_main", "Shelfbot initialisation complete. Deleting main task.");
    vTaskDelete(nullptr);

    // Explicit infinite loop so the compiler knows this never returns
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
