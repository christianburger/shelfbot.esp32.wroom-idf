idf_component_register(
        SRCS
        "ultrasonic_sensor.cpp"
        "tof_sensor.cpp"
        "lidar_sensor.cpp"
        "lidar_packet_parser.cpp"
        "sensor_control.cpp"
        "sensor_manager.cpp"
        "i2c_scanner.cpp"
        INCLUDE_DIRS "include" "."
        REQUIRES
        idf_includes
        vl53l0x_driver
        vl53l1_driver
        vl53l1_modbus_driver
        lydsto_driver
        firmware_version
        duart_modbus
        microros_sync      # <-- added
)