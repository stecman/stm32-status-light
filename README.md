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