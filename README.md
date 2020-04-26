# USB Status Light

Firmware for the [USB Status Light v2 project](https://hackaday.io/project/165524-usb-status-light).

## Building

```sh
# Pull in libopencm3
git submodule init
git submodule update

# Build
cd src
make

# Flash
make flash
```

## USB Protocol

The status light device defines an interface with one read and one write endpoint:

| Endpoint | Type      | Address | Description                            |
| -------- | --------- | ------------------------------------------------ |
| OUT 1    | Interrupt | 0x81    | Data from the host to the status light |
| IN 1     | Interrupt | 0x01    | Data from the status light to the host |

The OUT (host to device) endpoint takes 8-bit binary commands with variable
length argument bytes as described below. The IN (device to host) endpoint is
used to respond to commands that request device data.

### Commands

| Byte | Arguments | Name                                                            |
| ---- | --------- | --------------------------------------------------- |
| 0xC0 | 4x 32-bit | Set the colour of each LED from left to right       |