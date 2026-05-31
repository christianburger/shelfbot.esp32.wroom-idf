idf_component_register(SRCS "motor_control.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES
                       freertos
                       idf_includes
                       state_machine
                      )