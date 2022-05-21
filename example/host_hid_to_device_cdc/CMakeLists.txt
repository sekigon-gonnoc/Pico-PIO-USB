set(target_name host_hid_to_device_cdc)
add_executable(${target_name})

target_sources(${target_name} PRIVATE
 host_hid_to_device_cdc.c
 usb_descriptors.c
 )

 # print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)

# use tinyusb implementation
target_compile_definitions(${target_name} PRIVATE PIO_USB_USE_TINYUSB)

# needed so tinyusb can find tusb_config.h
target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_pio_usb tinyusb_device tinyusb_host)
pico_add_extra_outputs(${target_name})

