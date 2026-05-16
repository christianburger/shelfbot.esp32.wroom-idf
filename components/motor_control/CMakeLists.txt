idf_component_register(SRCS "motor_control.cpp"
                    	INCLUDE_DIRS "."
                    	REQUIRES
    		 	gin66__fastaccelstepper
                    	freertos
                    	idf_includes
                    )
