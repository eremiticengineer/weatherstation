set(PROJECT sample)

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel" CACHE STRING "Path to the FreeRTOS Kernel")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(${PROJECT} C CXX ASM)

# https://forums.raspberrypi.com/viewtopic.php?t=350982
add_executable(Pioasm IMPORTED)
set_property(TARGET Pioasm PROPERTY IMPORTED_LOCATION
  /home/pi/bin/pioasm
)
set(Pioasm_FOUND 1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)

pico_sdk_init()

add_subdirectory(
  ${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-FAT-CLI-for-RPi-Pico/src/FreeRTOS+FAT+CLI build
)

add_subdirectory(lib/fmt)

add_executable(${PROJECT}
  src/main.cpp
  src/cbsd/cbsd.h
  src/cbsd/cbsd.cpp
  src/cbsd/hw_config.c
  src/cblora/cblora.h
  src/cblora/cblora.cpp
  src/BME280Sensor/BME280Sensor.h
  src/BME280Sensor/BME280Sensor.cpp
  src/VEML7700/VEML7700.h
  src/VEML7700/VEML7700.cpp
  src/DS3231/DS3231.h
  src/DS3231/DS3231.cpp
)

pico_enable_stdio_usb(${PROJECT} 1)
pico_enable_stdio_uart(${PROJECT} 0)

# https://forums.raspberrypi.com/viewtopic.php?t=362404
# if("${PIOASM_PATH}" STREQUAL "")
#   set(PIOASM_PATH  /home/pi/bin/pioasm)
# endif()
# if(NOT "${PIOASM_PATH}" STREQUAL "")
#   if(EXISTS "${PIOASM_PATH}" AND NOT IS_DIRECTORY "${PIOASM_PATH}")
#     add_executable(Pioasm IMPORTED)
#     set_property(TARGET Pioasm PROPERTY IMPORTED_LOCATION ${PIOASM_PATH})
#     set(Pioasm_FOUND 1)
#   endif()
# endif()

# Note, you should pick the FreeRTOS library that suits you best:
# 
# FreeRTOS-Kernel-Heap1 thru FreeRTOS-Kernel_Heap4
# or 
# FreeRTOS-Kernel-Static
target_link_libraries(${PROJECT}
  pico_stdlib
  hardware_spi
  hardware_i2c
  hardware_uart
  pico_btstack_ble
  pico_btstack_cyw43
  pico_cyw43_arch_none
  FreeRTOS-Kernel-Heap4
  FreeRTOS+FAT+CLI
  fmt::fmt-header-only
)

pico_btstack_make_gatt_header(${PROJECT} PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/src/bt/temp_sensor.gatt"
)

# FreeRTOSConfig.h needs to be in the include path
target_include_directories(${PROJECT} PUBLIC
  include/
  src/cbsd
  src/bt
  src/cblora
  src/BME280Sensor
  src/VEML7700
  src/DS3231
)

target_compile_options(${PROJECT} PUBLIC
 -Wall 
 -Wextra 
 -Wshadow 
 -Wstack-usage=2048 
 -fanalyzer 
)

target_compile_definitions(${PROJECT} PUBLIC
  # this replaces "pico" with "-sdk 1" in the path
  #pico-sdk
  PICO_USE_STACK_GUARDS=1
  PICO_MALLOC_PANIC=1
  PICO_MAX_SHARED_IRQ_HANDLERS=6
)

target_compile_definitions(${PROJECT} PRIVATE
  USE_PRINTF
)

set_property(TARGET ${PROJECT} APPEND_STRING PROPERTY LINK_FLAGS
  "-Wl,--print-memory-usage"
)

pico_add_extra_outputs(${PROJECT})
