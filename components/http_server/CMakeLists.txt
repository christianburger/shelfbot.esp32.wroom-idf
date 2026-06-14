idf_component_register(
    SRCS "http_server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        idf_includes
        firmware_version
        lidar_sensor
        json
)

target_add_binary_data(${COMPONENT_LIB} "lidar.html" BINARY)
target_add_binary_data(${COMPONENT_LIB} "lidar_viz.js" BINARY)