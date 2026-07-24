# GNU Radio bench flowgraphs

Test-signal generators for the receiver, so the firmware can be exercised
without an antenna or a real transmitter. Built with **GNU Radio 3.10**.

| File | What it does |
| --- | --- |
| `freedv_test.grc` | Generates a FreeDV signal on the PC, FM-modulates it and mixes it up to the 12 kHz IF the radio samples. Uses GNU Radio's built-in `vocoder_freedv_tx_ss`, so no external FreeDV tools are needed. |
| `if_test.grc` | Transmits over the air through a LimeSDR (`soapy_limesdr_sink`), for testing the full RF chain rather than feeding the IF directly. |

## Settings that matter

These mirror the numbers in the top-level README, and are already set in the
flowgraphs:

- **IF = 12 kHz.** The signal source mixes the modem audio up to 12 kHz, which
  is where the radio's NCO expects it.
- **FM deviation = 2.5 kHz** (`max_dev: 2.5e3` in the NBFM TX block). This is
  the one to watch for 2400B: at much higher deviation the firmware's
  discriminator sits at the atan2 wrap point and every wrap is an audible click.
  If you retune the flowgraph and 2400B starts hiccuping, check this first.

`freedv_test.grc` is currently set up for **FreeDV 2400B**. Change the mode in
the `vocoder_freedv_tx_ss` block, and switch the firmware to the matching mode,
to test 1600 instead.

## Feeding the firmware

`freedv_test.grc` produces the 12 kHz IF at the sound-card / SDR output. Route
that into the radio's ADC input the same way a real 12 kHz third IF would arrive
— see the block diagram in the top-level README.
