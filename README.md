# `matrixmirror` - LED Matrix Platform

This project is designed to control a HUB75 RGB LED Matrix in real time from a
linux-based computer. In its default configuration, it uses a camera stream from a
Raspberry Pi Camera (via [picamera2](https://github.com/raspberrypi/picamera2)) and displays
it in pixelated form on an LED matrix.

The underlying software is very flexible and allows for a variety of architectures.

This project consists of two main components: the Pi Pico-based firmware (written in C++)
and the host software (written in Python).
The host software is currently optimized for running on a Raspberry Pi 3B+ or newer,
but can also run on any Linux system (though you would need to change the camera source).

The host software generates arbitrary frames and sends these to the firmware over one of
the following transport protocols:
- SPI (best performance, up to ~75FPS with sufficient compute and very stable)
- TCP (requires Pi Pico W, flaky and only about 15FPS)
- Serial (both direct and tunneled via [tio](https://github.com/tio/tio), stable but slow)

## Features

Below is a short list of features. See the respective sections for more detail.

- Low-flicker (matrix refreshes at ~600Hz, well above flicker threshold for human vision and most cameras)
- Low-latency
- Highly configurable
  - Matrix size and scan factor
  - Camera source, cropping and filters
  - Communication medium between host and firmware
  - Alternative video sources, as long as OpenCV-style numpy arrays are provided
- Easy to setup
  - Scripts for almost fully automatic installation of host software
  - Ready to install firmware UF2 images
- Reliable
  - Firmware uses watchdog to prevent freezes
  - Properly wired and shielded SPI is very reliable
  - Host software supports autostart and auto-restart using systemd
- Secure
  - no networking or cloud needed after initial setup (only required for dependency installation)
  - if using SPI or serial, all critical data (e.g. camera frames) stays entirely local

## Hardware

This project only works with HUB75-style LED matrices. This should be almost all commercially
available video-wall type matrices. Note that WS2812-based (or other addressable LEDs)
do not work.

By default, the firmware is compiled for 64x64 1/32 scan (e.g. five row pins, A-E) matrices.
The firmware also supports other formats by changing preprocessor defines. Non-square matrices
may encounter issues, but could still work if the missing half of the matrix is faked.

For the microcontroller, only the Pi Pico and Pi Pico W have been tested. The Pi Pico W
is only needed if you want to use the TCP transport.
The Pi Pico 2 (or other RP2350 boards) would require some changes, since they have a slower
clock (150MHz max. vs the 200MHz that the RP2040 has since a recent SDK update). The
additional processing power of the Pi Pico 2 is not needed, but the clock rate directly
influences how fast the PIO can push out pixel data. The slower clock would also limit the
maximum SPI speed.

Additionally, level shifting from the Pico's 3.3V to the HUB75 5V is needed. Depending on
the panel, you may get away without this, but this is not recommended. I used two 74HCT245
octal transceivers. Many other 74HCT series logic ICs can also be used. Note that the series
is important here. If in doubt, check the datasheet for the input threshold voltages.

The host software can run on any Linux system (likely even anywhere Python runs). The
default configuration uses a Raspberry Pi Camera for input, so will only work on a Raspberry
Pi. In theory, any model would work, but anything older than the Raspberry Pi 3B+ is likely
too slow for smooth display. Future optimizations may push this minimum recommendation lower.

The default configuration also uses the SPI transport. This is easily done on a Raspberry Pi
as well as any other SBC with an accessible 3.3V-level SPI interface that is exposed
to userspace (may require some configuration depending on OS).

### Wiring

TODO: adopt from [particlesim-pico](https://github.com/not-na/particlesim-pico?tab=readme-ov-file#electrical-connections)
and add SPI and level shifter

TODO: shutdown button

## Firmware

TODO: compilation and installation instructions

## Host Software

TODO: installation

## Case

TODO: design and link