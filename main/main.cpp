#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "wifi_station.h"
#include "http_server.h"
#include "motor_control.h"
#include "Arduino.h"
#include "esp_log.h"

static const char* TAG = "main";

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main()");
    initArduino();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_control_begin();
    wifi_init_sta();
    start_webserver();
    ESP_LOGI(TAG, "app_main() finished.");
}
