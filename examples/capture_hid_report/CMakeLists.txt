set(target_name capture_hid_report)
add_executable(${target_name})

pico_enable_stdio_usb(${target_name} 1) 

target_sources(${target_name} PRIVATE
 capture_hid_report.c
 )

# print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore pico_pio_usb)
pico_add_extra_outputs(${target_name})
