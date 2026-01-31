idf_component_register(SRCS "motor_control.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES
                    FastAccelStepper
                    freertos
                    arduino
                    idf_includes
                    )
