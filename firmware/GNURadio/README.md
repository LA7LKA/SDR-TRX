# GNU Radio bench flowgraphs

Test-signal generators for the receiver, so the firmware can be exercised
without an antenna or a real transmitter. Built with **GNU Radio 3.10**.

## Active flows: one per mode

`sdrtrx_{am,cw,fm,ssb,freedv1600,freedv2400B,freedv700D,freedv700E}.grc` —
one flowgraph per mode, each exercising both RX and TX (see "Combined RX/TX
bench flows" below for the pattern). This is the current, maintained set;
open one of these, not anything in `old/`.

`old/` holds everything these superseded — the earlier single-purpose files
(`am_rx.grc`, `freedv_test.grc`, `if_test.grc`, etc.) and the `ssb.grc`
pilot/`ptt_serial.py` from when the combined-flow pattern was first proven
out. Kept for reference, not maintained.

## Settings that matter

These mirror the numbers in the top-level README, and are already set in the
flowgraphs:

- **IF = 12 kHz.** The signal source mixes the modem audio up to 12 kHz, which
  is where the radio's NCO expects it.
- **FM deviation = 2.5 kHz** (`max_dev: 2.5e3` in the NBFM TX block). This is
  the one to watch for 2400B: at much higher deviation the firmware's
  discriminator sits at the atan2 wrap point and every wrap is an audible click.
  If you retune the flowgraph and 2400B starts hiccuping, check this first.

`sdrtrx_freedv2400B.grc` and `sdrtrx_freedv1600.grc` cover 1600/2400B; switch
the firmware to the matching mode when testing either.

## Feeding the firmware

Each `sdrtrx_*.grc` produces its mode's 12 kHz IF at the sound-card output.
Route that into the radio's ADC input the same way a real 12 kHz third IF
would arrive — see the block diagram in the top-level README.

## Combined RX/TX bench flows

Since the firmware split RX and TX onto separate ADC/DAC pairs (RX: PC0 in,
PA4 out; TX: PA3 in, PA5 out — see `doc/architecture.md`), each `sdrtrx_*.grc`
exercises both directions at once instead of needing separate RX/TX files: an
RX test-signal generator and a TX-verify listener run continuously on two
channels of the same sound card, and the firmware's own `tx_active` picks
which one it's actually sampling, same as real hardware.

Device convention, consistent across all of them:

- **I/Q sound card**, the `iq_dev` variable in each flow (default the
  Behringer UM2, `plughw:CARD=CODEC,DEV=0`), wired to the H7 I/Q adapter
  board, stereo both ways with **L = I, R = Q**:
  output L → PF11 (I in, Zio A5), output R → PA6 (Q in, Zio D12);
  input L ← PA4 (I out, Zio D24), input R ← PA5 (Q out, Zio D13).
  The RX generator's 12 kHz IF leaves as its analytic (single-sided) I/Q
  pair via a 257-tap Hilbert; the TX-verify listener takes the I/Q pair in
  directly where it used to synthesise Q with a Hilbert (left disabled in
  the flow). `tools_iq_convert.py` did this conversion from the earlier mono
  real-IF flows; set `iq_dev` to change card.
- **Jabra headset**, `plughw:CARD=J380,DEV=0`: mic → passed straight through
  as the TX-test microphone; headphones ← the RX or TX-verify demod.

The radio's own USB sound card (`CARD=Audio`) is stereo too: in firmware
mode `iq` it carries raw I/Q (L = I, R = Q) both ways and the PC does all
the DSP; in every other mode it carries demodulated audio, the same on L and R.

Earlier wiring (F746 / pre-adapter H7), for reference: the integrated
`Generic` card, output L → PC0 (RX IF in), R → PA3 (TX mic in); input
L ← PA4 (RX audio out), R ← PA5 (TX IF out).

PTT for these bench flows is the real front-panel PTT button now that it
works (see the top-level README) — key the STM32 directly rather than
simulating PTT from the PC side.
