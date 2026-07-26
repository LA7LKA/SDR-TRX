# LA7LKA QRP SDR Transceiver

A QRP transceiver built around an STM32F746 doing the demodulation in software,
with FreeDV digital voice integrated alongside the analog modes rather than
bolted on as an external adaptor.

Work in progress. Expect rapid changes, refactoring and experimental branches.

## Repository layout

| Directory | Contents |
| --- | --- |
| [firmware/](firmware/) | STM32F746 firmware, the codec2 submodule, the GNU Radio bench flowgraphs and a prebuilt image |
| [hardware/](hardware/) | PCB and RF front-end design (Mk2) |
| [flutter-app/](flutter-app/) | Companion mobile app |
| [doc/](doc/) | Architecture and design documentation |

## Status — 2026-07-24

Working and tested against live signals:

| Mode | Status | Notes |
| --- | --- | --- |
| SSB (USB/LSB) | RX + TX working | Phasing method; sideband labelled to match the air after up-conversion (checked against an FT-857D) |
| NBFM | RX working | TX exists but is currently out of the main loop |
| **FreeDV 1600** | **RX + TX working** | HF, carried on SSB |
| **FreeDV 2400B** | **RX + TX working** | VHF/10 m, carried on FM |
| **FreeDV 700D** | **RX + TX working** | HF, weak-signal OFDM + LDPC |
| **FreeDV 700E** | **RX + TX working** | As 700D with a shorter frame, so it reacquires faster |
| **AM** | **RX + TX working** | Envelope detection in, carrier + both sidebands out |
| **CW** | **RX working** | SSB into a 250 Hz filter at 700 Hz, 5.3 ms blocks |
| **CW transmit** | **working** | Shaped keying, beacon on DAC1 |
| **SSB transmit** | **working** | Phasing method; unwanted sideband below the bench noise floor |
| **NBFM transmit** | **working** | Deviation set by mic gain, reported in telemetry |


Resource use with all four FreeDV modes compiled in: **227 KB flash of
1024 KB**, **231 KB RAM of 320 KB**, leaving about 14 KB of heap headroom.

## Architecture

The whole radio in one block diagram, the Mk1/Mk2 front-end split, the DSP
block/FIFO design and the QSK T/R switching are written up in
[doc/architecture.md](doc/architecture.md).

## Flashing a prebuilt image

If you just want to try it, `firmware/prebuilt/` holds a prebuilt image so you
do not need an ARM toolchain at all:

```sh
st-flash --format ihex write firmware/prebuilt/qrp-sdr-trx.hex
minicom -D /dev/ttyACM0 -b 115200
```

See [firmware/prebuilt/README.md](firmware/prebuilt/README.md) for what is in
that build.

## Building

```sh
cd firmware && make
```

Requires `arm-none-eabi-gcc`.

### codec2 prerequisite

codec2 comes in as a git submodule under `firmware/codec2`, pinned to a
known-good commit — the build depends on generated sources and on API details
that have moved between versions, so the pin is deliberate.

```sh
git clone --recursive https://github.com/LA7LKA/SDR-TRX.git
```

or, if you already cloned without `--recursive`:

```sh
git submodule update --init
```

Then build codec2 once. The firmware consumes the generated codebook sources
from `firmware/codec2/build/src/` along with the generated `config.h` and
`version.h`, so this step is required even though we never link libcodec2
itself:

```sh
cd firmware/codec2 && mkdir -p build && cd build && cmake .. && make
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
| `-D__FPU_PRESENT=1` (codec2 only) | `codec2_math_arm.c`, which the OFDM modes need for `codec2_complex_dot_product_f32`, includes `arm_math.h` without a device header first, so the FPU would otherwise look absent. |

The OFDM modes also need `firmware/codec2/src/codec2_math_arm.c` in the source list;
without it `ofdm.c` will not link.

One trap worth knowing about if you are short of RAM: `run_ldpc_decoder()`
allocates roughly 32 KB across some 340 blocks **on every call**, not once at
open time. Starve the heap and its `CALLOC` returns NULL, `assert()` fires, and
`abort()` ends up in the `while (1)` inside newlib's `_exit()` — the board just
stops, with no output and no fault message. Budget for the transient, not only
for what `freedv_open()` reports.

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

## Console

Commands over the same ST-Link VCP as the telemetry, so modes and transmit can
be driven without reflashing. Deliberately built before the front panel: an
OLED and switches then become a second way to reach commands that already work,
rather than a second thing to debug at the same time as transmit.

```
> mode          list modes
> mode 8        select by number
> tx            key the CW beacon
> rx            back to receive
> mic 15        microphone gain
> amtone        AM 1 kHz test tone, to prove the modulator without a mic
> wpm 25        CW speed
> stat          current state
```

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

Ready-to-run GNU Radio 3.10 flowgraphs for both are in
[firmware/GNURadio/](firmware/GNURadio/README.md).

## Roadmap

- Mk1 minimal front end, so the radio can be built without a PCB
- Programmable LO (AD9851 on I2C1), switched band-pass filter bank and VFO
  logic, behind a hardware abstraction so Mk1 and Mk2 keep sharing one core
- CW pitch and bandwidth as front panel controls; the filter is a biquad
  cascade rather than a FIR so retuning is five coefficients, not a redesign
- CW offset and sidetone, which must track the pitch: a tone heard at 700 Hz
  means the carrier is 700 Hz off the dial, and the sidetone has to match the
  RX pitch or zero-beating lands you beside the other station
- CW iambic keyer
- FreeDV TX
- Full filterbank (FIR/FFT), improved AGC and noise reduction
- PA control and protection logic
- CAT control
- Waterfall/spectrum display

### Further out: M17

M17 is an open digital voice protocol built on Codec2, so most of the hard part
here — getting Codec2 to run in real time on this MCU — is already done. Its
vocoder mode costs almost nothing to add:

| | Flash | RAM |
| --- | --- | --- |
| Current (1600 + 2400B) | 101 264 B | 228 588 B |
| Enabling `CODEC2_MODE_3200_EN` | 104 508 B | 228 588 B |
| **Difference** | **+3 244 B** | **0 B** |

3200 shares `sine`, `nlp`, `lpc` and `quantise` with 1300; only the
quantisation path differs.

M17's 40 ms frame is 1920 samples at 48 kHz, which is exactly the block size
the FreeDV modes already use, so the per-mode block machinery fits unchanged.
What is missing is the protocol layer: an RRC matched filter, symbol timing
recovery at 4800 sym/s, a Viterbi decoder for the K=5 rate-1/2 convolutional
code, and M17 framing. `libm17` exists as an embeddable C reference, and
OpenRTX already runs M17 on an STM32F405 at 168 MHz, so an F746 with the
instruction cache enabled has room to spare.

The natural target for M17 is VHF/UHF rather than HF, which would mean a third
front end alongside Mk1 and Mk2 — and a much simpler one, since image rejection
on a single narrow band does not need up-conversion. The 12 kHz IF interface
means the DSP core would not change.

## Licensing

This project is licensed under the **GNU General Public License v3.0** — see
[LICENSE](LICENSE).

codec2 remains under its own licence, LGPL 2.1, and is not redistributed here.
LGPL 2.1 permits linking from a GPL-licensed program, so the combination is
fine; the FSF licence compatibility list is the reference if you need the
detail.

If you redistribute a built image such as the one in `firmware/prebuilt/`, note
that linking LGPL code statically carries a relink obligation: a recipient has
to be able to rebuild the firmware against their own copy of codec2. The
sources and Makefile under `firmware/` are meant to cover that.

None of the above is legal advice.

## A note on how this was built

Substantial parts of the firmware, and most of the codec2 integration, were
written with AI assistance (Claude). The RF architecture, hardware, IF plan and
all on-air testing are mine, as are several of the corrections that made it
work. Worth stating openly rather than not.

73 de LA7LKA
