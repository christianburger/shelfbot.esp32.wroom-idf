idf_component_register(SRCS "wifi_station.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_event log freertos)