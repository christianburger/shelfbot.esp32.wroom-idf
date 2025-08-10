idf_component_register(SRCS "http_server.cpp"
                     INCLUDE_DIRS "."
                     REQUIRES motor_control esp_http_server json log)

target_add_binary_data(
    ${COMPONENT_LIB}
    "index.html"
    BINARY
)
