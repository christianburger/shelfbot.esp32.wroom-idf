idf_component_register(SRCS "sensor_control.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_timer esp_driver_gptimer freertos)
