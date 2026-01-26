    idf_component_register(    
        SRCS "vl53l0x.cpp"    
        INCLUDE_DIRS "include"    
        REQUIRES "driver" 
        "esp_timer"
    )
