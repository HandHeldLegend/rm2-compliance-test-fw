# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(RM2_COMPLIANCE C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(RM2_COMPLIANCE rm2-compliance.c )

pico_set_program_name(RM2_COMPLIANCE "RM2_COMPLIANCE")
pico_set_program_version(RM2_COMPLIANCE "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(RM2_COMPLIANCE 0)
pico_enable_stdio_usb(RM2_COMPLIANCE 1)

# Add the standard library to the build
target_link_libraries(RM2_COMPLIANCE
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(RM2_COMPLIANCE PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/EDIT_RM2_GPIO_PINS.cmake")
    message("Including EDIT_RM2_GPIO_PINS.cmake for ${PROJECT_NAME}")
    include(${CMAKE_CURRENT_LIST_DIR}/EDIT_RM2_GPIO_PINS.cmake)
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE
        CYW43_DEFAULT_PIN_WL_REG_ON=${BT_ON_PIN}
        CYW43_DEFAULT_PIN_WL_DATA_OUT=${BT_DATA_OUT_PIN}
        CYW43_DEFAULT_PIN_WL_DATA_IN=${BT_DATA_IN_PIN}
        CYW43_DEFAULT_PIN_WL_HOST_WAKE=${BT_WAKE_PIN}
        CYW43_DEFAULT_PIN_WL_CLOCK=${BT_CLOCK_PIN}
        CYW43_DEFAULT_PIN_WL_CS=${BT_CS_PIN}
    )
endif()

target_compile_definitions(RM2_COMPLIANCE PRIVATE
    CYW43_LWIP=0
    )

# Add any user requested libraries
target_link_libraries(RM2_COMPLIANCE 
        pico_cyw43_arch_threadsafe_background
        pico_btstack_ble
        pico_btstack_classic
        pico_btstack_cyw43
        pico_btstack_hci_transport_cyw43
        )

pico_add_extra_outputs(RM2_COMPLIANCE)

