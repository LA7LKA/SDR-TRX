# LA7LKA QRP SDR Transceiver

A QRP transceiver built around an STM32F746 doing the demodulation in software,
with FreeDV digital voice integrated alongside the analog modes rather than
bolted on as an external adaptor.

Work in progress. Expect rapid changes, refactoring and experimental branches.

## Status — 2026-07-24

Working and tested against live signals:

| Mode | Status | Notes |
| --- | --- | --- |
| SSB (USB/LSB) | RX working | Phasing method, Hilbert + complex mixer |
| NBFM | RX working | TX exists but is currently out of the main loop |
| **FreeDV 1600** | **RX working** | HF, carried on SSB. ~79 % CPU |
| **FreeDV 2400B** | **RX working** | VHF/10 m, carried on FM. ~57 % CPU |

Resource use with both FreeDV modes compiled in: **99 KB flash of 1024 KB**,
**224 KB RAM of 320 KB**.

## Architecture

### Analog front end

Triple conversion, ending at a 12 kHz third IF so the MCU can sample it
directly:

```
RF --> 45 MHz --> 455 kHz --> 12 kHz --> ADC (48 kHz)
```

12 kHz is exactly Fs/4 at 48 kHz sampling, which is convenient: the complex
mixer collapses to sign flips and I/Q swaps if it ever needs optimising.

### DSP

Everything runs at 48 kHz from a TIM6-triggered ADC into a circular DMA buffer.
The main loop processes half a buffer at a time; the DMA callbacks only set a
flag.

Block size is chosen per mode, because the requirements genuinely differ:

| Mode | Block | Period | Why |
| --- | --- | --- | --- |
| FreeDV 1600 | 1920 | 40 ms | 1920/6 = 320 = exactly one FDMDV frame |
| FreeDV 2400B | 1920 | 40 ms | Exactly one fmfsk frame |
| Analog | 256 | 5.3 ms | Low enough latency for CW break-in |

Making a block equal exactly one modem frame matters more than it looks. With a
mismatched block size the per-block cost alternates between cheap and expensive,
and the expensive one overruns the period even when average load is well under
100 %.

### FreeDV signal paths

The two modes take different routes through the radio, which is the point of
having both:

- **1600** is an HF mode carried on SSB. Its modem runs at 8 kHz, so the SSB
  audio is decimated 48k->8k, demodulated, and the decoded speech interpolated
  back to 48 kHz for the DAC.
- **2400B** is designed to pass through a commodity FM radio's audio path. Its
  modem runs at 48 kHz natively, so the FM discriminator output goes straight
  in with no resampling. Only the speech output is interpolated.

Note that 2400**A** is *not* the FM mode despite the name — it targets SDR
radios with 5 kHz RF bandwidth and needs about 75 KB of heap against 2400B's
8 KB.

The FM path for 2400B uses a dedicated discriminator with no de-emphasis and no
voice AGC. The voice FM demodulator's 500 us de-emphasis rolls off 11.8 dB at
1200 Hz and 23.6 dB at 4800 Hz, which tilts a data waveform badly across its own
band.

## Flashing a prebuilt image

If you just want to try it, `firmware/` holds a prebuilt image so you do not
need an ARM toolchain at all:

```sh
st-flash --format ihex write firmware/qrp-sdr-trx.hex
minicom -D /dev/ttyACM0 -b 115200
```

See [firmware/README.md](firmware/README.md) for what is in that build.

## Building

```sh
make
```

Requires `arm-none-eabi-gcc`.

### codec2 prerequisite

codec2 comes in as a git submodule, pinned to a known-good commit — the build
depends on generated sources and on API details that have moved between
versions, so the pin is deliberate.

```sh
git clone --recursive https://github.com/LA7LKA/SDR-TRX.git
```

or, if you already cloned without `--recursive`:

```sh
git submodule update --init
```

Then build codec2 once. The firmware consumes the generated codebook sources
from `codec2/build/src/` along with the generated `config.h` and `version.h`,
so this step is required even though we never link libcodec2 itself:

```sh
cd codec2 && mkdir -p build && cd build && cmake .. && make
```

### Build flags that matter

Getting codec2 to run in real time on this part needed several non-obvious
settings, all in the Makefile:

| Flag | Why |
| --- | --- |
| `-D__EMBEDDED__` | Drops `MODEM_STATS`'s GUI scatter-plot buffer. `struct freedv` goes from 140 KB to 480 B — without it `freedv_open()` cannot allocate at all. Also switches codec2 to `codec2_malloc`/`codec2_free`, provided in `freedv_chain.c`. |
| `-DFREEDV_MODE_EN_DEFAULT=0` plus per-mode enables | Building every mode overflows flash; the OFDM modes' LDPC matrices alone are hundreds of KB. |
| `-fsingle-precision-constant` (codec2 only) | The FPU is `fpv5-sp-d16`, single precision only. Unsuffixed literals promote expressions to `double`, which is then emulated in software. This removed 830 of 957 soft-float calls. |
| `-O3` (codec2 only) | The project builds at `-Og` for debuggability. A modem that has to keep up with real time does not benefit from that. |

Also required, in `main.c`: `SCB_EnableICache()`, plus `ART_ACCELERATOR_ENABLE`
and `PREFETCH_ENABLE` in `stm32f7xx_hal_conf.h`. At 216 MHz flash runs with 7
wait states, so without them every instruction fetch stalls and the DSP loop
runs several times slower than it should.

Stack and heap in `STM32F746XX_FLASH.ld` are raised well above the CubeMX
defaults: `fdmdv_demod()` puts variable-length arrays on the stack, and codec2
allocates its modem state with `malloc()`.

One more that is easy to miss: the chain calls `freedv_set_squelch_en(fdv, true)`.
With squelch off, codec2 passes the modem input through while unsynced by taking
every Nth sample with no anti-alias filter, which at 48 kHz in and 8 kHz out
folds everything above 4 kHz into the speech band and sounds like badly
resampled audio.

## Telemetry

One line per second on USART3, which is the ST-Link virtual COM port
(`/dev/ttyACM0`), 115200 8N1:

```
mode=4 blocks=25 ovr=0 load_pct=57 us_max=23019 us_fdv=13663 adc_x1000=733 peak_x1000=104 rms_x1000=31 sync=1 snr_x10=57
```

| Field | Meaning |
| --- | --- |
| `blocks` / `ovr` | Blocks processed and overruns per second. `ovr` must be 0 — FreeDV cannot hold sync through gaps in the sample stream. |
| `load_pct` / `us_max` | DWT-measured worst-case block time against the block period. |
| `us_fdv` | Of that, how much was inside the FreeDV chain. |
| `adc_x1000` | Peak at the ADC, x1000. |
| `peak_x1000` / `rms_x1000` | Peak and RMS at the modem input. In FM modes a peak near 1000 means the discriminator is at the +-pi wrap point, i.e. the deviation is far too high. |
| `sync` / `snr_x10` | Modem sync and SNR estimate. SNR is not comparable between modes — the estimators differ. |

This is the main debugging instrument for the receive path.

## Bench setup

Signals are generated with GNU Radio on a PC rather than off the air:

- **1600**: mixed with a 12 kHz LO so the FDMDV carriers land at 12.9–14.1 kHz,
  and the firmware's NCO brings them back to 900–2100 Hz audio.
- **2400B**: fed to an FM modulator at **about 2.5 kHz deviation**. This matters
  — at 10x that, the discriminator sits at the wrap point and every wrap is an
  audible click.

## Roadmap

- AM (nearly free — complex baseband is already there, AM is `arm_cmplx_mag_f32`)
- CW with iambic keyer
- FreeDV 700D for poor HF conditions (needs `codec2_math_arm.c` wired in; CPU is
  the binding constraint, not flash)
- FreeDV TX
- Full filterbank (FIR/FFT), improved AGC and noise reduction
- PA control and protection logic
- CAT control
- Waterfall/spectrum display

## Licensing

This project is licensed under the **GNU General Public License v3.0** — see
[LICENSE](LICENSE).

codec2 remains under its own licence, LGPL 2.1, and is not redistributed here.
LGPL 2.1 permits linking from a GPL-licensed program, so the combination is
fine; the FSF licence compatibility list is the reference if you need the
detail.

If you redistribute a built image such as the one in `firmware/`, note that
linking LGPL code statically carries a relink obligation: a recipient has to be
able to rebuild the firmware against their own copy of codec2. The sources and
Makefile here are meant to cover that.

None of the above is legal advice.

## A note on how this was built

Substantial parts of the firmware, and most of the codec2 integration, were
written with AI assistance (Claude). The RF architecture, hardware, IF plan and
all on-air testing are mine, as are several of the corrections that made it
work. Worth stating openly rather than not.

73 de LA7LKA
