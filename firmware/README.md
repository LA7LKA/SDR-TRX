# Prebuilt firmware

For anyone who wants to try the radio without setting up an ARM toolchain.

Built 2026-07-24 for the **STM32F746** (Nucleo-F746ZG pinout, see `qrp-sdr-trx.ioc`).

| File | Use |
| --- | --- |
| `qrp-sdr-trx.hex` | Intel HEX, carries its own load address |
| `qrp-sdr-trx.bin` | Raw binary, must be flashed to `0x08000000` |

## What is in this build

- FreeDV 1600, 700D and 700E (HF, on SSB) and FreeDV 2400B (VHF/FM) receive
- SSB and NBFM receive
- Starts up in FreeDV 700D
- Telemetry on USART3 / ST-Link VCP, 115200 8N1

227 KB flash, 231 KB RAM.

## Flashing

With [stlink](https://github.com/stlink-org/stlink):

```sh
st-flash --format ihex write qrp-sdr-trx.hex
```

or, using the raw binary:

```sh
st-flash write qrp-sdr-trx.bin 0x08000000
```

Then open the virtual COM port to see what it is doing:

```sh
minicom -D /dev/ttyACM0 -b 115200
```

## Note on licensing

This image is GPL-3.0 (see the top-level LICENSE) and statically links codec2,
which is LGPL 2.1. If you redistribute it, the LGPL requires you to also make
available whatever a recipient needs to relink the firmware against their own
build of codec2 — typically the object files, or the full sources and build
system. The sources and Makefile in this repository are intended to cover that.
None of this is legal advice.
