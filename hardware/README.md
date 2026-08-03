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
| 2nd mixer + LO2 | BCM847 2nd mixers (RX and TX), LO2 fixed ~44.988 MHz — a second AD9851, not a crystal or Si5351 |
| IF | THAT2162 (RX AGC + TX ALC), op-amps, anti-alias/reconstruction LPFs, buffers to ADC1/DAC2 |
| PA-driver | RD16HHF1 exciter, ~5 W out (pure QRP — Mk2 is not sized to drive an external amplifier); push-pull vs. single device still open |
| LPF | TX low-pass filter bank |
| SWR bridge | Directional coupler (fwd/rev), feeding ADC3's control scan for PA protection/foldback |
| PIN T/R | Antenna transmit/receive switch, PIN diodes for QSK |
| LF | Electret mic preamp → ADC2, LM386 speaker/headphone output from DAC1 — the first card being built, since it is pure audio and testable against the existing firmware with no RF involved |
| HMI | Rotary encoder + 10 buttons + SSD1306 OLED + PTT, on the front panel — firmware side (`firmware/Core/Src/hmi.c`) proven on a Nucleo test board 2026-08-03: encoder/buttons/OLED/PTT/mode select all drive the real radio, not just a bench diagnostic |
| BLE bridge | nRF52840 (reused USB dongle), bridging a free STM32 UART to the [flutter-app/](../flutter-app/) over Bluetooth Low Energy |

The Nucleo's onboard USB (`USB_OTG_FS`) gives a second, wired control path —
the same CDC-ACM console already used over the ST-Link VCP, so a PC/laptop
gets full-menu control with no extra hardware. BLE (phone) and USB (PC) are
the two transports for the same menu/CAT-style control plus HF text
messaging; see [flutter-app/](../flutter-app/).

**Both LOs are AD9851, not an AD9851 + a fixed oscillator.** LO2 only needs a
fixed ~44.988 MHz, but a plain crystal there would need TCXO-grade stability
anyway — LO2 drift maps 1:1 onto the 12 kHz IF centre, and a garden-variety
±20–50 ppm crystal at 45 MHz can drift over 1 kHz across temperature, a real
bite out of a 14 kHz roofing filter. A custom-frequency TCXO to fix that is a
low-volume special order (MOQ, weeks of lead time) for an oddball 44.988 MHz.
A second AD9851 instead reuses the LO1 card's buffer amp, LPF and driver code
nearly unchanged, and — the actual win — lets both DDS chips share **one**
reference oscillator, so the whole radio needs only one precision reference,
at a standard frequency, rather than two. LO2's exact output can then be
trimmed in firmware instead of needing an exact crystal cut.

RX and TX filter banks are separate cards on purpose: RX wants band-pass
(reject out-of-band and image), TX wants low-pass (reject harmonics), and it
avoids routing TX power through the low-loss RX preselector.

**5 W keeps the TX-power-carrying cards cheap and small, not just the PA.**
RMS current into 50 Ω drops from ~0.63 A at 20 W to ~0.32 A at 5 W, and RMS
voltage from ~32 V to ~16 V — thinner wire and smaller, lower-saturation-risk
toroids in the LPF bank across all six bands, lower-voltage-rated capacitors,
and a simpler SWR bridge (a couple of small binocular cores, resistors and
diodes is enough at QRP power — the classic homebrew directional coupler,
no real power handling to design around).

All six HF bands (160–10 m) are the goal, but populated incrementally: 40 m
and 10 m first to validate the LNA → mixer → IF chain end to end, then the
remaining bands as repeats of a known-good filter design.

## Status

Design only. `lf-audio` is next: KiCad schematic + layout, fabbed as a cheap
board (Seeed Fusion), tested against the current firmware with no code
changes (`amtone`, the `mic` gain command, and telemetry's `adc_x1000`).
