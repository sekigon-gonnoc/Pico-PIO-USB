# Pico-PIO-USB

USB host implementation using PIO of raspberry pi pico (RP2040).

You can add additional USB port to RP2040.

🚧 **This library is WIP. API may be changed drastically in future.** 🚧

## Demo

https://user-images.githubusercontent.com/43873124/146642806-bdf34af6-4342-4a95-bfca-229cdc4bdca2.mp4

## Project status

|Planned Features|Status|
|-|-|
|FS Host|✔|
|LS Host|✔|
|Hub support|✔|
|Multi port|✔|
|FS Device|🚧|

## Examples

`capture_hid_report.c` is a sample program which print HID reports received from device.

```bash
cd example
mkdir build
cd build
cmake ..
make
# Copy UF2 file to RPiPico and open serial port
```

Another sample program for split keyboard with QMK

[https://github.com/sekigon-gonnoc/qmk_firmware/tree/rp2040/keyboards/pico_pico_usb](https://github.com/sekigon-gonnoc/qmk_firmware/tree/rp2040/keyboards/pico_pico_usb)

## Resource Usage

- Two PIO
  - One PIO is for USB transmitter using 22 instruction and one state machine
  - Another PIO is for USB receiver using 29 instruction and two state machine
- Two GPIO for D+/D- (Series 22ohm resitors are better)
- 15KB ROM and RAM
