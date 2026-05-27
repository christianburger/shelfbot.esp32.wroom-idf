idf_component_register(
    SRCS "microros_sync.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        idf_includes
        micro_ros_espidf_component
        motor_control
        led_control
        wifi_manager
        sensor_control
    PRIV_REQUIRES
        firmware_version
)