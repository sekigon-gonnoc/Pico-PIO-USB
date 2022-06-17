set(target_name host_hid_to_device_cdc)
add_executable(${target_name})

set(PICO_PIO_USB_SRC "${CMAKE_CURRENT_LIST_DIR}/../../src")

pico_generate_pio_header(${target_name} ${PICO_PIO_USB_SRC}/usb_tx.pio)
pico_generate_pio_header(${target_name} ${PICO_PIO_USB_SRC}/usb_rx.pio)

target_sources(${target_name} PRIVATE
 host_hid_to_device_cdc.c
 usb_descriptors.c
 ${PICO_PIO_USB_SRC}/pio_usb.c
 ${PICO_PIO_USB_SRC}/pio_usb_device.c
 ${PICO_PIO_USB_SRC}/pio_usb_host.c
 ${PICO_PIO_USB_SRC}/usb_crc.c
 # can use 'tinyusb_pico_pio_usb' library later when pico-sdk is updated
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
 )
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)
target_include_directories(${target_name} PRIVATE ${PICO_PIO_USB_SRC} ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore hardware_pio hardware_dma tinyusb_device tinyusb_host)
pico_add_extra_outputs(${target_name})

