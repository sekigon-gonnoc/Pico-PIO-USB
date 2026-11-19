# HID host to CDC

The default example forwards keyboard and mouse reports from the PIO USB host
port to the native USB CDC port.

## Packet capture

To use the CDC port for the [capture GUI](../../capture-gui/) instead, configure
the examples with the capture option:

```sh
cmake -S examples -B build -DUSB_PACKET_CAPTURE_DEBUG=ON
cmake --build build --target host_hid_to_device_cdc
```

Flash the example and connect the board's native USB port to the computer.
Connect the USB device being measured to the PIO USB host port as usual. Open
the capture GUI and select the board's CDC port. The example continues to poll
boot HID devices and uses the CDC connection for binary capture responses.

Capture uses one state machine on a spare PIO block, two DMA channels, and a
16 KiB sample ring. With the option off, the original HID-to-CDC behavior is
used and the sampler and capture protocol are excluded from the build.
