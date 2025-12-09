# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(Embarcatech_WeatherStation C CXX ASM)
pico_sdk_init()

include_directories( ${CMAKE_SOURCE_DIR}/lib ) # Inclui os files .h na pasta lib

add_executable(${PROJECT_NAME}  
        Embarcatech_WeatherStation.c 
        lib/dht.c
        lib/bmp280.c 
        lib/ssd1306.c
        )

pico_generate_pio_header(Embarcatech_WeatherStation ${CMAKE_CURRENT_LIST_DIR}/dht.pio)


target_link_libraries(${PROJECT_NAME} 
        pico_stdlib 
        hardware_i2c
        hardware_clocks
        hardware_dma
        hardware_pio
        pico_cyw43_arch_lwip_threadsafe_background
        )

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})
