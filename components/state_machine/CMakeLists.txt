idf_component_register(
    SRCS "state_machine.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        idf_includes
        esp_timer
)