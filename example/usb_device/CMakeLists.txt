set(target_name usb_device)
add_executable(${target_name})

pico_enable_stdio_usb(${target_name} 1) 

set(PICO_PIO_USB_SRC "${CMAKE_CURRENT_LIST_DIR}/../../src")

pico_generate_pio_header(${target_name} ${PICO_PIO_USB_SRC}/usb_tx.pio)
pico_generate_pio_header(${target_name} ${PICO_PIO_USB_SRC}/usb_rx.pio)

target_sources(${target_name} PRIVATE
  usb_device.c
  ${PICO_PIO_USB_SRC}/pio_usb.c
  ${PICO_PIO_USB_SRC}/pio_usb_device.c
  ${PICO_PIO_USB_SRC}/pio_usb_host.c
  ${PICO_PIO_USB_SRC}/usb_crc.c
)
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)
target_include_directories(${target_name} PRIVATE ${PICO_PIO_USB_SRC})

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore hardware_pio hardware_dma)
pico_add_extra_outputs(${target_name})

