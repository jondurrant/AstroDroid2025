add_executable(${NAME}
        main.cpp
        
        Agent.cpp
        
        freertos_allocators.c
        pico_usb_transport.c
        PubEntities.cpp
        uRosBridge.cpp
        uRosEntities.cpp
        
        ConfigEntity.cpp
        NVSJson.cpp
        AstroEntities.cpp
        
        Stepper.cpp
        JointAgent.cpp
        LX16A.cpp
        
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	 pico_stdlib
	 pico_unique_id
	 hardware_watchdog
     FreeRTOS-Kernel-Heap4 
     freertos_config
     hardware_pwm
     micro_ros
     pico_rand
     onboard_nvs
     tiny_json
     json_maker
     PicoLed
	)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
     )
     

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
