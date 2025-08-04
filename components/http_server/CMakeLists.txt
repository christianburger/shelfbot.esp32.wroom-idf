idf_component_register(SRCS "http_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES motor_control esp_http_server
                    PRIV_REQUIRES json)
