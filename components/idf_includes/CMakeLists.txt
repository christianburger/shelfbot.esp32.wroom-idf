idf_component_register(
        SRCS "."
        INCLUDE_DIRS "include"
        REQUIRES
        freertos
        log
        esp_system
        esp_timer
        esp_common
        esp_event
        esp_netif
        esp_http_server
        esp_hw_support
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_gptimer
        esp_driver_uart
        nvs_flash
        lwip
        espressif__mdns
)