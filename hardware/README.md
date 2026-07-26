# Hardware

The Mk2 analog front end and RF chain — design work in progress, nothing built
yet. See [doc/architecture.md](../doc/architecture.md) for the signal chain
this hardware implements (band-pass bank, double conversion to a 12 kHz IF,
QSK T/R switching) and the Mk1/Mk2 split: none of this is required to run the
firmware, which is developed and tested against Mk1 (a GNU Radio-generated
12 kHz IF, see [firmware/GNURadio/](../firmware/GNURadio/)).

## Build strategy

Modular, not one monolithic board first: separate small PCBs per functional
block, connected by SMA/coax, so each card is validated standalone on the
bench (NanoVNA / scope / LimeSDR) before integration. A bug then means
respinning one small cheap board, not the whole radio. Module boundaries
follow the same interfaces the firmware uses (the 12 kHz IF, etc.), and each
boundary doubles as a natural shield-can boundary.

## Card map

~6 custom cards plus a Nucleo-F746ZG as the MCU/DSP "card":

| Card | Contents |
| --- | --- |
| Front-end | RX band-pass filter bank + LNA |
| RF / conversion | ADE-1 shared bidirectional 1st mixer, 45 MHz crystal filter, BCM847 2nd mixers, LO2 (fixed ~44.988 MHz) |
| LO1 | AD9851 DDS, isolated from the front end on its own card |
| IF | THAT2162 (RX AGC + TX ALC), op-amps, anti-alias/reconstruction LPFs, buffers to ADC1/DAC2 |
| PA | Driver + push-pull RD16HHF1 |
| Antenna interface | TX low-pass filter bank + T/R PIN switch |
| lf-audio | Electret mic preamp → ADC2, LM386 speaker/headphone output from DAC1 — the first card being built, since it is pure audio and testable against the existing firmware with no RF involved |

RX and TX filter banks are separate on purpose: RX wants band-pass (reject
out-of-band and image), TX wants low-pass (reject harmonics), and it avoids
routing 20 W through the low-loss RX preselector.

All six HF bands (160–10 m) are the goal, but populated incrementally: 40 m
and 10 m first to validate the LNA → mixer → IF chain end to end, then the
remaining bands as repeats of a known-good filter design.

## Status

Design only. `lf-audio` is next: KiCad schematic + layout, fabbed as a cheap
board (Seeed Fusion), tested against the current firmware with no code
changes (`amtone`, the `mic` gain command, and telemetry's `adc_x1000`).
