# firmware-h7 (NUCLEO-H755ZI-Q port)

Next-hardware-revision port to STM32H755ZI-Q (dual-core Cortex-M7 @480MHz +
Cortex-M4 @240MHz), replacing the STM32F746ZG used by `firmware/`. Lives
alongside `firmware/` rather than replacing it — see the project's own
notes on why (F7 tree stays the reference while this ports over).

## Why this chip

The F7 build is CPU-marginal running FreeDV/codec2 decode against USB Audio
Class and HMI on a single core. The H755's real prize isn't clock speed,
it's the dual-core split: a dedicated core for codec2 decode instead of
time-slicing it against USB/HMI/telemetry on one core.

## Layout

CubeMX generates **separate CM7 and CM4 sub-projects** for a dual-core part
like the H755 — not one combined tree. This directory is structured around
that from the start:

```
firmware-h7/
  CM7/    Cortex-M7 project — DSP: demod/modulator chains, FreeDV/codec2 decode
  CM4/    Cortex-M4 project — HMI (OLED/encoder/buttons), iambic keyer, and
          other non-DSP control work
```

Generate the CubeMX project with **Project Manager → Project → Toolchain:
Makefile**, targeting these two directories directly, so both live under
version control the same way `firmware/` does.

## Porting plan (expected to carry over from `firmware/`)

CM7 (DSP):
- DSP/demod math (`dsp.c`)
- `freedv_chain.c`'s codec2 glue
- the SSB/CW/FM demod/modulator chains currently in `main.c`

CM4 (HMI/control):
- HMI/OLED/encoder/button code
- iambic keyer / CW paddle handling (`cw_paddle.c`)

Not yet assigned to a core:
- USB Audio Class logic (needs a real look before assuming a straight
  copy — H7 has both OTG_FS and OTG_HS, unlike F7's OTG_FS-only setup;
  also needs to end up wherever the DSP chain it feeds lives)

Expected **not** to carry over as-is: CubeMX scaffolding itself, anything
tied to F7-specific memory layout (DTCM/ITCM sizes differ on H7).

Inter-core communication (DSP samples/state between CM7 and CM4) is not
yet designed — HSEM/mailbox vs. shared SRAM region, TBD once both CubeMX
projects exist.

## Status

Not started. This is scaffolding only, ahead of running CubeMX by hand for
the clock tree / pin config / dual-core boot split.

See `doc/h7-dfu-bootloader-checklist.md` for the bootloader work planned to
happen first, before the application port itself.

## Automatic I/Q correction (RX, `iq` mode)

`CM7/Core/Src/iq_corr.c` removes DC and corrects the gain and phase error
between I and Q before the samples go to the PC over USB. It needs no test
signal: what arrives at the antenna at +f and -f is uncorrelated, so after
correction I and Q must have equal power and zero correlation. The
estimate averages over about 4 s, so it follows drift (temperature, band,
gain steps). Only one tap, so it assumes the error is flat across the IF
band. On the bench that holds (0.4-0.6 deg over 3-18 kHz). TX is not
corrected yet; that needs a loopback (see the comment at the top of
`iq_corr.c`).

Controls and readouts are globals, for the debugger (CubeIDE Live
Expressions, or gdb via openocd):

| Variable | |
|---|---|
| `iqc_enable` | 1 = correct (default), 0 = raw ADC data as before |
| `iqc_freeze` | 1 = keep the current correction, stop adapting |
| `iqc_reset` | write 1 to restart the estimate |
| `iqc_gain_db`, `iqc_phase_deg` | measured Q-vs-I gain and phase error |
| `iqc_dc_i`, `iqc_dc_q` | removed DC, full scale = 1.0 |
| `iqc_irr_in_db`, `iqc_irr_out_db` | estimated image rejection before/after (floor ~50-60 dB from the averaging) |

Host test (synthetic 8 dB / 0.5 deg error, DC offsets, three tones): image
rejection 7.3 dB raw, 81-97 dB corrected.

Bench check with the UM2 and adapter board:

1. In the GNU Radio flow, set `q_trim_db` to `0.0`, so the 8 dB
   imbalance reaches the H7 uncorrected.
2. With `iqc_enable = 0`, send a tone and note the image in the PC's FFT
   (expect ~7 dB rejection, as measured 2026-10-25). `iqc_gain_db` should
   read about -8 dB and `iqc_phase_deg` about 0.5 deg.
3. Set `iqc_enable = 1` and wait ~5 s. The image should drop by tens of
   dB. Step the tone across 3-18 kHz to check it holds over the band.
4. Optional: set `q_trim_db` back to -8 and check that `iqc_gain_db` goes
   to about 0 dB.
