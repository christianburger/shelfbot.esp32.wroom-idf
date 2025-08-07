idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash wifi_station http_server motor_control log)
