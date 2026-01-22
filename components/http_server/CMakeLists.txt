idf_component_register(SRCS "http_server.cpp"
                     INCLUDE_DIRS
                                  "."
                                  "include"
                                  "../sensor_control/include"
                     REQUIRES
                          motor_control
                          esp_http_server
                          sensor_control
                          led_control
                          json)

target_add_binary_data(
    ${COMPONENT_LIB}
    "index.html"
    BINARY
)
