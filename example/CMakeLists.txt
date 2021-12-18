cmake_minimum_required(VERSION 3.13)
include (pico_sdk_import.cmake)
project(pio_usb)

pico_sdk_init()

set(target_name capture_hid_report)
add_executable(${target_name})

pico_enable_stdio_usb(${target_name} 1) 

pico_generate_pio_header(${target_name} ${CMAKE_CURRENT_LIST_DIR}/../usb_tx.pio)
pico_generate_pio_header(${target_name} ${CMAKE_CURRENT_LIST_DIR}/../usb_rx.pio)

target_sources(${target_name} PRIVATE capture_hid_report.c ../pio_usb.c ../usb_crc.c)
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)
target_include_directories(${target_name} PRIVATE ../)

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore hardware_pio hardware_dma)
pico_add_extra_outputs(${target_name})

