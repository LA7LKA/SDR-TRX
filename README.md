# LA7LKA QRP HF SDR Transceiver
Under development — early-stage firmware-defined HF transceiver project.

## Current Status
The project is actively being developed. Core DSP and RF architecture is still evolving, but several modes are already functional:

- SSB (LSB/USB) — stable RX/TX chain with working AGC and audio pipeline
- FM/NBFM — working demodulator and modulator
- Basic IF chain operational (IF=12Khz)
- Initial mixer/NCO pipeline implemented

## Planned Features
- FreeDV (700D / 1600)
- CW with iambic keyer
- Full filterbank (FIR/FFT)
- Improved AGC and noise reduction
- PA control and protection logic
- CAT control
- Waterfall/spectrum display (optional)

## Notes
This repository is a work-in-progress. Expect rapid changes, refactoring, and experimental branches.
