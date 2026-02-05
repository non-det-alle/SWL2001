# Dragino LoRa GPS HAT + Raspberry Pi

WARNING: You need the Dragino LoRa GPS HAT version v1.4. Previous hardware versions are missing the GPIO bridge to an interrupt pin of the LoRa chip required for reception interrupts.

## Cross compilation dependencies

Assuming you installed a 64bit version of Raspberry Pi OS, for cross compilation you need the `gcc-aarch64-linux-gnu` package. It will install the GCC compiler for 64bit ARM architectures under `/usr/aarch64-linux-gnu`.

### Other dependencies installation

The code depends on the `pigpio.h` library. To install it

- open a terminal somewhere and run `git clone https://github.com/joan2937/pigpio.git && cd pigpio`
- open `Makefile` and set `CROSS_PREFIX = aarch64-linux-gnu-` on top of the file
- run `make` and then `sudo DESTDIR=/usr/aarch64-linux-gnu make install` to build and install the library for cross compilation

## Build

Clone this repo, run `cd SWL2001/lbm_drag_rpi` followed by `make full_sx1276` to build a simple example (the default main file is under `main_examples/main_periodical_uplink.c`). The built executable is `build_sx1276_drpi/app_sx1276.elf` and can be transferred into your Raspberry Pi (for example over `ssh` using `scp`).

### Cmake

Building with cmake and ninja:

```bash
cmake -B build/ -G Ninja -DLBM_RADIO=sx1276 -DAPP=periodical_uplink
ninja -C build/
```
