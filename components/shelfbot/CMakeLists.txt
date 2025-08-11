idf_component_register(SRCS "shelfbot.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component
                    PRIV_REQUIRES wifi_station http_server motor_control log nvs_flash mdns)
