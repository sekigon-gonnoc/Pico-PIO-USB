cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(device_host_stdio_example C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
add_executable(${PROJECT_NAME})

# add pico-PIO-USB to the library path
# Before this will work, you must first download Pico-PIO-USB where it's expected:
#     after downloading pico-sdk, navigatge to SDK_DIR/lib/tinyusb and run
#     python3 tools/get_deps.py rp2040
#
# This downloads Pico-PIO-USB and puts it at hw/mcu/raspberry_pi/Pico-PIO-USB in the SDK.
# (I imagine you would replace "rp2040" with "rp2350" for pico2 but I haven't tested this)
set(PICO_PIO_USB_DIR "${PICO_TINYUSB_PATH}/hw/mcu/raspberry_pi/Pico-PIO-USB")
add_subdirectory(${PICO_PIO_USB_DIR} pico_pio_usb)

target_compile_definitions(${PROJECT_NAME} PRIVATE PIO_USB_USE_TINYUSB)

target_sources(${PROJECT_NAME} PUBLIC
        src/main.c
        src/usb_descriptors.c
        ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(${PROJECT_NAME} PUBLIC src)

# Enable the reset interface for picoloader
target_compile_definitions(${PROJECT_NAME} PUBLIC
        PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
        PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_MS_OS_20_DESCRIPTOR=1
        PICO_STDIO_USB_RESET_INTERFACE_MS_OS_20_DESCRIPTOR_ITF=4
        PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL=1
        PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT=1
        PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK=0
        PICO_STDIO_USB_RESET_RESET_TO_FLASH_DELAY_MS=100

# it seems debugging can only be changed here; trying to redefine CFG_TUSB_DEBUG in tusb_config.h doesn't work.
#        CFG_TUSB_DEBUG=3
#        CFG_TUD_LOG_LEVEL=0
#        CFG_TUH_LOG_LEVEL=0
#        CFG_TUH_CDC_LOG_LEVEL=3
)

target_link_libraries(${PROJECT_NAME} PUBLIC pico_stdlib pico_unique_id hardware_rtc hardware_watchdog pico_usb_reset_interface pico_usb_reset_interface_headers pico_pio_usb tinyusb_device tinyusb_host)

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})
