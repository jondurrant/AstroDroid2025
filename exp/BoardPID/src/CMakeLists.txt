add_executable(${NAME}
        main.cpp
        
        Agent.cpp
        BlinkAgent.cpp
       
        
        GPIOInputMgr.cpp
        GPIOObserver.cpp
        MotorPID.cpp
        MotorMgr.cpp
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	 pico_stdlib
     FreeRTOS-Kernel-Heap4 
     freertos_config
     hardware_pwm
	)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
     )
     

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 0)
pico_enable_stdio_uart(${NAME} 1)
