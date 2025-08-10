idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES wifi_station http_server motor_control log arduino nvs_flash micro_ros_espidf_component mdns)
