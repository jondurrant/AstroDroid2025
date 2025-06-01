if (DEFINED NVS_PATH)
	message("Using Given NVS_PATH '${NVS_PATH}')")
else ()
	set(NVS_PATH "${CMAKE_CURRENT_LIST_DIR}lib/RPIPicoOnboardNVS/")
    message("Using local NVS_PATH '${NVS_PATH}')")
endif ()

include(${NVS_PATH}/src/files.cmake)

add_library(onboard_nvs STATIC)

target_sources(onboard_nvs PUBLIC 
	${NVS_PATH}/src/NVSOnboard.cpp
#	${NVS_PATH}/src/NVSAgent.cpp
)


# Add include directory
target_include_directories(onboard_nvs PUBLIC 
    ${NVS_PATH}/src
)

# Add the standard library to the build
target_link_libraries(onboard_nvs PUBLIC 
	pico_stdlib 
	hardware_flash
)

