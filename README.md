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

### The 12 kHz IF is the interface

The firmware ingests **one real ADC channel carrying a 12 kHz IF**, and does
not care how that IF was produced. Everything above it — mixing, filtering,
demodulation, FreeDV — is the same regardless of the front end.

That makes two builds possible from one codebase:

| | Mk1 | Mk2 |
| --- | --- | --- |
| Front end | Minimal, or none at all | Triple conversion |
| Signal source | GNU Radio, a cheap SDR, or a sound card | Antenna |
| Strong-signal performance | Modest | Roofing filter, good dynamic range |
| Hardware needed | Breadboard | PCB, shielding, alignment |
| Firmware | **Identical** | **Identical** |

Mk1 exists so the project can be reproduced and developed without a board, and
it is also the sensible build order: get the DSP and FreeDV chain solid against
a known-clean signal first, then take on the analog design knowing the firmware
side is already verified.

12 kHz is also exactly Fs/4 at 48 kHz sampling, which is convenient — the
complex mixer collapses to sign flips and I/Q swaps if it ever needs optimising.

### Mk1 — minimal front end

Any source that delivers a 12 kHz IF works. The GNU Radio flowgraphs in
[GNURadio/](GNURadio/README.md) generate one directly, so the entry cost is a
sound card output and a bias network to shift its swing into the ADC's 0–3.3 V
range centred on 1.65 V. No PCB, no alignment, no mixers.

### Mk2 — triple conversion

```
RF --> 45 MHz --> 455 kHz --> 12 kHz --> ADC (48 kHz)
```

Up-converting to 45 MHz first puts the image at **f + 90 MHz** for every HF
band — 92 to 118 MHz, far outside anything an HF input filter passes. Single
conversion to 455 kHz would put the image 910 kHz away on 20 m, needing a
tracking preselector with an impossible Q, which is why general-coverage
receivers went to up-conversion in the first place.

#### Band-pass filter bank

Up-conversion's own weakness is that it leaves the front end wideband: without
a preselector the first mixer sees the entire HF spectrum at once, including
broadcast stations tens of dB stronger than any amateur signal. Switched
per-band filters restore selectivity ahead of the mixer, and that is what
separates a good up-converting receiver from a mediocre one. It also disposes
of the 92–118 MHz image band thoroughly, since a filter centred on an amateur
band has enormous attenuation up there.

Six filters cover all ten HF bands, every one with a bandwidth ratio well under
2:1, so simple 3–5 pole LC sections are enough:

| Filter | Bands | Range |
| --- | --- | --- |
| 1 | 160 m | 1.81–2.00 MHz |
| 2 | 80 m | 3.50–3.80 MHz |
| 3 | 60 + 40 m | 5.35–7.20 MHz |
| 4 | 30 + 20 m | 10.10–14.35 MHz |
| 5 | 17 + 15 m | 18.07–21.45 MHz |
| 6 | 12 + 10 m | 24.89–29.70 MHz |

Relay switching is preferable to PIN diodes here: the whole point of the filter
bank is dynamic range, and diode bias current is one more thing that can
generate intermodulation. At QRP power the same bank can plausibly serve
transmit harmonic filtering as well, which halves the parts count — worth
deciding early, since it affects the relay and layout choice.

Three LOs are needed, but **only the first one tunes**:

| LO | Frequency | Role |
| --- | --- | --- |
| LO1 | 46–75 MHz, variable | RF to 45 MHz |
| LO2 | 45.455 MHz, fixed | 45 MHz to 455 kHz |
| LO3 | 443 kHz, fixed | 455 kHz to 12 kHz |

A Si5351A covers all three from one I2C part, which is what makes this
affordable. Its square-wave output is not a problem for receive: the third
harmonic of LO1 lands at 138–225 MHz, and the band-pass filter ahead of the
mixer has already removed anything that could mix down from there.

The real constraint is **LO1 phase noise**. It sits at the first mixer, seeing
the whole HF spectrum at once, so its noise sidebands reciprocal-mix strong
nearby signals straight into the IF. That, rather than the roofing filter, will
set close-in dynamic range on a crowded band — and it is the one place where
cheap parts undo the reason for choosing this architecture. A Si5351 is fine to
get running; a cleaner synthesiser for LO1 is worth it if Mk2 is meant to earn
its complexity.

LO control and band selection belong behind one thin hardware abstraction so
the core stays shared. A single `set frequency` entry point works out which
band the frequency falls in, selects the filter, and programs LO1; Mk1
implements it as a no-op or a single output, and the DSP layer never knows the
difference.

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

Ready-to-run GNU Radio 3.10 flowgraphs for both are in
[GNURadio/](GNURadio/README.md).

## Roadmap

- Mk1 minimal front end, so the radio can be built without a PCB
- Programmable LO (Si5351 on I2C1), switched band-pass filter bank and VFO
  logic, behind a hardware abstraction so Mk1 and Mk2 keep sharing one core
- AM (nearly free — complex baseband is already there, AM is `arm_cmplx_mag_f32`)
- CW with iambic keyer
- FreeDV 700D for poor HF conditions (needs `codec2_math_arm.c` wired in; CPU is
  the binding constraint, not flash)
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
