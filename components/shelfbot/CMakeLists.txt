idf_component_register(SRCS "shelfbot.cpp"
        INCLUDE_DIRS "."
        REQUIRES
        idf_includes
        micro_ros_espidf_component
        motor_control
        wifi_station
        http_server
        led_control
        sensor_control
        firmware_version
)