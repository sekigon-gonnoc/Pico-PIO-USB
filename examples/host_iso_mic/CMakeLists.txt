set(target_name host_iso_mic)
add_executable(${target_name})

target_sources(${target_name} PRIVATE
 host_iso_mic.c
 usb_descriptors.c
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
 )

# print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)

# Host isochronous IN support. With a ring buffer the host collects packets
# every frame and the application drains them with normal endpoint
# transfers. Set to 0 (cmake -DHOST_ISO_MIC_RING_SIZE=0) for packet mode,
# where each transfer returns exactly one packet.
set(HOST_ISO_MIC_RING_SIZE 4096 CACHE STRING "Isochronous ring buffer size in bytes, 0 for packet mode")

# pico_pio_usb is an INTERFACE library, so these definitions apply to its
# sources too.
target_compile_definitions(${target_name} PRIVATE
  # use tinyusb implementation
  PIO_USB_USE_TINYUSB
  PIO_USB_HOST_ISOCHRONOUS=1
  PIO_USB_ISO_RING_SIZE=${HOST_ISO_MIC_RING_SIZE}
  )

option(HOST_ISO_MIC_DEBUG "Print low-level endpoint state once a second" OFF)
if (HOST_ISO_MIC_DEBUG)
  target_compile_definitions(${target_name} PRIVATE ISO_MIC_DEBUG)
endif()

# needed so tinyusb can find tusb_config.h
target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_pio_usb tinyusb_device tinyusb_host)
pico_add_extra_outputs(${target_name})
