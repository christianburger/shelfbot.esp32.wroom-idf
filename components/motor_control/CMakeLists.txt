idf_component_register(SRCS "motor_control.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES FastAccelStepper log arduino)
