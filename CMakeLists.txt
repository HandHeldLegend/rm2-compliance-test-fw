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

set(RM2_CORE_SOURCES
    main.c
    platform.c
    test_ui.c
    test_session.c
    wifi_lwip_stubs.c
)

set(RM2_BT_SOURCES
    hci_common.c
    test_btc.c
    test_ble.c
)

set(RM2_WIFI_SOURCES
    test_wifi.c
)

# Prefer the MFG blob known to bring up under cyw43-driver (7.95.39 / Data 9.10.39).
# The newer picow-wifi-mfg-tester 7.95.49 extract currently fails bus_init (STA never up).
set(RM2_MFG_FW_HEADER "")
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/firmware/mfg_cyw43_fw.h")
    set(RM2_MFG_FW_HEADER "mfg_cyw43_fw.h")
elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/firmware/mfg_cyw43_fw_picow_7_95_49.h")
    set(RM2_MFG_FW_HEADER "mfg_cyw43_fw_picow_7_95_49.h")
endif()

# One firmware binary per board_pinouts/*.cmake × variant (bt stock / wifi MFG).
# VARIANT is "bt" (stock CYW43 + Bluetooth tests) or "wifi" (MFG CYW43 + WiFi tests).
function(rm2_add_compliance_firmware BOARD_ID PINOUT_FILE VARIANT)
    if(NOT VARIANT STREQUAL "bt" AND NOT VARIANT STREQUAL "wifi")
        message(FATAL_ERROR "VARIANT must be 'bt' or 'wifi' (got '${VARIANT}')")
    endif()

    set(TARGET_NAME "RM2_COMPLIANCE_${BOARD_ID}_${VARIANT}")

    set(_sources ${RM2_CORE_SOURCES})
    if(VARIANT STREQUAL "bt")
        list(APPEND _sources ${RM2_BT_SOURCES})
    else()
        list(APPEND _sources ${RM2_WIFI_SOURCES})
    endif()

    add_executable(${TARGET_NAME} ${_sources})

    pico_set_program_name(${TARGET_NAME} "${TARGET_NAME}")
    pico_set_program_version(${TARGET_NAME} "0.1")

    pico_enable_stdio_uart(${TARGET_NAME} 0)
    pico_enable_stdio_usb(${TARGET_NAME} 1)

    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    # Pinout file sets BOARD_NAME and BT_* pin variables
    include(${PINOUT_FILE})
    if(NOT DEFINED BOARD_NAME OR BOARD_NAME STREQUAL "")
        message(FATAL_ERROR
            "board_pinouts/${BOARD_ID}.cmake must set BOARD_NAME (display name for the UI)")
    endif()

    # Stringy banner fields go through a generated header so spaces are safe on Windows.
    set(RM2_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_NAME}")
    file(MAKE_DIRECTORY "${RM2_GEN_DIR}")
    if(VARIANT STREQUAL "bt")
        set(_rm2_variant_str "Bluetooth (stock WiFi+BT firmware)")
    else()
        set(_rm2_variant_str "WiFi MFG (WLTEST pkteng firmware)")
    endif()
    file(WRITE "${RM2_GEN_DIR}/rm2_build_info.h"
        "/* Generated file, do not edit */\n"
        "#pragma once\n"
        "#define RM2_BOARD_ID \"${BOARD_ID}\"\n"
        "#define RM2_BOARD_NAME \"${BOARD_NAME}\"\n"
        "#define RM2_BUILD_VARIANT \"${_rm2_variant_str}\"\n"
    )
    target_include_directories(${TARGET_NAME} PRIVATE "${RM2_GEN_DIR}")

    target_compile_definitions(${TARGET_NAME} PRIVATE
        CYW43_LWIP=0
    )

    if(NOT RM2_USE_SDK_CYW43_PINS)
        foreach(_pin_var BT_ON_PIN BT_DATA_OUT_PIN BT_DATA_IN_PIN BT_WAKE_PIN BT_CLOCK_PIN BT_CS_PIN)
            if(NOT DEFINED ${_pin_var})
                message(FATAL_ERROR
                    "board_pinouts/${BOARD_ID}.cmake must set ${_pin_var} (or RM2_USE_SDK_CYW43_PINS 1)")
            endif()
        endforeach()
        target_compile_definitions(${TARGET_NAME} PRIVATE
            CYW43_DEFAULT_PIN_WL_REG_ON=${BT_ON_PIN}
            CYW43_DEFAULT_PIN_WL_DATA_OUT=${BT_DATA_OUT_PIN}
            CYW43_DEFAULT_PIN_WL_DATA_IN=${BT_DATA_IN_PIN}
            CYW43_DEFAULT_PIN_WL_HOST_WAKE=${BT_WAKE_PIN}
            CYW43_DEFAULT_PIN_WL_CLOCK=${BT_CLOCK_PIN}
            CYW43_DEFAULT_PIN_WL_CS=${BT_CS_PIN}
        )
    else()
        message(STATUS "Board '${BOARD_NAME}' (${BOARD_ID}) using Pico SDK CYW43 pin defaults")
    endif()

    if(VARIANT STREQUAL "bt")
        target_compile_definitions(${TARGET_NAME} PRIVATE
            RM2_ENABLE_BT=1
            RM2_ENABLE_WIFI=0
        )
        target_link_libraries(${TARGET_NAME}
            pico_stdlib
            pico_cyw43_arch_threadsafe_background
            pico_btstack_ble
            pico_btstack_classic
            pico_btstack_cyw43
            pico_btstack_hci_transport_cyw43
        )
        message(STATUS "Board '${BOARD_NAME}' (${BOARD_ID}) BT/stock -> ${TARGET_NAME}.uf2")
    else()
        if(RM2_MFG_FW_HEADER STREQUAL "")
            message(FATAL_ERROR
                "WiFi/MFG target ${TARGET_NAME} needs a local MFG firmware header.\n"
                "NDA holders: obtain picow-wifi-mfg-tester.uf2 from Raspberry Pi, then run:\n"
                "  python tools/extract_mfg_fw_from_uf2.py path/to/picow-wifi-mfg-tester.uf2\n"
                "See firmware/README.md. Do not commit the extracted files.")
        endif()
        target_compile_definitions(${TARGET_NAME} PRIVATE
            RM2_ENABLE_BT=0
            RM2_ENABLE_WIFI=1
            "CYW43_CHIPSET_FIRMWARE_INCLUDE_FILE=\"${RM2_MFG_FW_HEADER}\""
        )
        target_include_directories(${TARGET_NAME} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/firmware
        )
        target_link_libraries(${TARGET_NAME}
            pico_stdlib
            pico_cyw43_arch_threadsafe_background
        )
        message(STATUS "Board '${BOARD_NAME}' (${BOARD_ID}) WiFi/MFG (${RM2_MFG_FW_HEADER}) -> ${TARGET_NAME}.uf2")
    endif()

    pico_add_extra_outputs(${TARGET_NAME})
endfunction()

file(GLOB RM2_PINOUT_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/board_pinouts/*.cmake"
)
list(SORT RM2_PINOUT_FILES)

if(NOT RM2_PINOUT_FILES)
    message(FATAL_ERROR
        "No board pinouts found in board_pinouts/*.cmake. See board_pinouts/README.md")
endif()

foreach(PINOUT_FILE ${RM2_PINOUT_FILES})
    get_filename_component(BOARD_ID "${PINOUT_FILE}" NAME_WE)
    rm2_add_compliance_firmware("${BOARD_ID}" "${PINOUT_FILE}" "bt")
    rm2_add_compliance_firmware("${BOARD_ID}" "${PINOUT_FILE}" "wifi")
endforeach()
