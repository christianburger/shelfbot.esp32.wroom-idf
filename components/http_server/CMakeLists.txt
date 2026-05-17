idf_component_register(SRCS "http_server.cpp"
        INCLUDE_DIRS "." "include" "../sensor_control/include"
        REQUIRES
        idf_includes
        motor_control
        sensor_control
        led_control
        json
        firmware_version
)

target_add_binary_data(${COMPONENT_LIB} "index.html" BINARY)
target_add_binary_data(${COMPONENT_LIB} "lidar.html" BINARY)
target_add_binary_data(${COMPONENT_LIB} "lidar_viz.js" BINARY)