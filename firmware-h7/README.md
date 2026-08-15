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
