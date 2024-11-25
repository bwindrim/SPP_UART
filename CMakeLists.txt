# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(SPP_UART C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(SPP_UART spp_uart.c)

pico_set_program_name(SPP_UART "SPP_UART")
pico_set_program_version(SPP_UART "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(SPP_UART 1)
pico_enable_stdio_usb(SPP_UART 0)

# Add the standard library to the build
target_link_libraries(SPP_UART
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(SPP_UART PRIVATE
  /home/brian/.pico-sdk/sdk/2.0.0/lib/btstack/src
  /home/brian/.pico-sdk/sdk/2.0.0/lib/btstack/3rd-party/bluedroid/encoder/include
  /home/brian/.pico-sdk/sdk/2.0.0/lib/btstack/3rd-party/bluedroid/decoder/include
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(SPP_UART 
    pico_stdlib
    pico_btstack_classic
    pico_btstack_cyw43
    pico_cyw43_arch_none
   )

pico_add_extra_outputs(SPP_UART)

set(ENABLE_CLASSIC)
set(BTSTACK_EXAMPLE_TYPE poll)
