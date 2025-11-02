# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# == Keep your VS Code Pico extension preamble ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(pico_sdk_import.cmake)

project(INF2004-AAI2 C CXX ASM)

# Check if wifi_credentials.h exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/wifi_credentials.h")
    message(STATUS "✓ Using wifi_credentials.h for WiFi settings")
    add_compile_definitions(USE_WIFI_CREDENTIALS_FILE)
else()
    message(WARNING "⚠ wifi_credentials.h not found. Using default placeholders.")
    message(WARNING "   Copy wifi_credentials.h.template to wifi_credentials.h and edit it.")
endif()

pico_sdk_init()

add_executable(INF2004-AAI2
    src/main.c
    src/fsm.c
    src/drivers/motor.c
    src/drivers/encoder.c
    src/control/pid.c
    src/drivers/ir_line_follower.c
    src/drivers/ir_barcode_scanner.c
    src/drivers/ultrasonic.c
    src/drivers/magnetometer.c
    src/networking/wifi_mqtt.c
)

target_include_directories(INF2004-AAI2 PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(INF2004-AAI2
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_i2c
    hardware_irq
    hardware_pwm
    hardware_timer
    pico_cyw43_arch_lwip_poll
    pico_lwip_mqtt
)

pico_enable_stdio_usb(INF2004-AAI2 1)
pico_enable_stdio_uart(INF2004-AAI2 0)
pico_add_extra_outputs(INF2004-AAI2)