cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME Neck) # <-- Name your project/executable here!

include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialize the SDK
pico_sdk_init()

#include Libraries
SET(FREERTOS_KERNEL_PATH "$ENV{PICO_FREERTOS}" CACHE STRING "Common Lib")
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include("${FREERTOS_CONFIG_FILE_DIRECTORY}/freertos_config.cmake") 
include(FreeRTOS_Kernel_import.cmake)

SET(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/../../lib/micro_ros_raspberrypi_pico_sdk/" CACHE STRING "Common Lib")
include(micro_ros.cmake)

set(NVS_PATH "${CMAKE_CURRENT_LIST_DIR}/../../lib/RPIPicoOnboardNVS/")
    message("Using local NVS_PATH '${NVS_PATH}')")
include(picoOnboardNVS.cmake)

set(TINY_JSON_DIR "${CMAKE_CURRENT_LIST_DIR}/../../lib/tiny-json")
include(tiny-json.cmake)

set(JSON_MAKER_DIR "${CMAKE_CURRENT_LIST_DIR}/../../lib/json-maker")
include(json-maker.cmake)

include(../../lib/PicoLED/PicoLed.cmake)

#Add main source directory
add_subdirectory(src)

#Set up files for the release packages
install(CODE "execute_process(COMMAND $ENV{HOME}/bin/picoDeploy.sh ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.elf)")

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
