# Hardware

The Mk2 analog front end and RF chain — design work in progress, nothing built
yet. See [doc/architecture.md](../doc/architecture.md) for the signal chain
this hardware implements (band-pass bank, double conversion to a 12 kHz IF,
QSK T/R switching) and the Mk1/Mk2 split: none of this is required to run the
firmware, which is developed and tested against Mk1 (a GNU Radio-generated
12 kHz IF, see [firmware/GNURadio/](../firmware/GNURadio/)).

## Build strategy

Modular, not one monolithic board first: one small PCB per functional block,
every one connected to its neighbours by SMA/coax rather than traces across a
shared board, so each card is validated standalone on the bench (NanoVNA /
scope / LimeSDR) before integration. A bug then means respinning one small
cheap board, not the whole radio. Module boundaries follow the same
interfaces the firmware uses (the 12 kHz IF, etc.), and each boundary doubles
as a natural shield-can boundary.

Going card-per-block rather than card-per-stage-group also pays off later:
the whole HF front end (BPF/LPF/LNA/mixer/LO1/45 MHz filter/2nd
mixer+LO2/IF) is swappable as a unit behind the same 12 kHz IF and SMA
interconnect, so a future VHF/UHF handheld variant reuses the Nucleo, LF and
PIN T/R cards unchanged and only needs a new, much simpler front end — no
image-rejection up-conversion required on a single narrow band. This is the
hardware side of the same principle [doc/architecture.md](../doc/architecture.md#the-12-khz-if-is-the-interface)
already states for firmware: "the 12 kHz IF is the interface."

## Card map

One card per block, plus a Nucleo-F746ZG as the MCU/DSP "card":

| Card | Contents |
| --- | --- |
| BPF | RX band-pass filter bank |
| LNA | Band-switched RX low-noise amp |
| 1st mixer | ADE-1, shared bidirectional 1st mixer (RF ↔ 45 MHz) |
| LO1 | AD9851 DDS, variable, isolated from the front end on its own card |
| 45 MHz filter | The crystal filter shared by both conversions (roofing + 2nd-conversion image rejection) |
| 2nd mixer + LO2 | BCM847 2nd mixers (RX and TX), LO2 fixed ~44.988 MHz |
| IF | THAT2162 (RX AGC + TX ALC), op-amps, anti-alias/reconstruction LPFs, buffers to ADC1/DAC2 |
| PA-driver | Driver + push-pull RD16HHF1 |
| LPF | TX low-pass filter bank |
| PIN T/R | Antenna transmit/receive switch, PIN diodes for QSK |
| LF | Electret mic preamp → ADC2, LM386 speaker/headphone output from DAC1 — the first card being built, since it is pure audio and testable against the existing firmware with no RF involved |

RX and TX filter banks are separate cards on purpose: RX wants band-pass
(reject out-of-band and image), TX wants low-pass (reject harmonics), and it
avoids routing 20 W through the low-loss RX preselector.

All six HF bands (160–10 m) are the goal, but populated incrementally: 40 m
and 10 m first to validate the LNA → mixer → IF chain end to end, then the
remaining bands as repeats of a known-good filter design.

## Status

Design only. `lf-audio` is next: KiCad schematic + layout, fabbed as a cheap
board (Seeed Fusion), tested against the current firmware with no code
changes (`amtone`, the `mic` gain command, and telemetry's `adc_x1000`).
