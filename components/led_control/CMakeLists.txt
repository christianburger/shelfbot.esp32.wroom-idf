idf_component_register(
    SRCS "led_control.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES state_machine
    REQUIRES idf_includes
)