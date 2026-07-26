# Companion app

Not started. A Flutter app that turns a phone into the radio's screen and
keyboard, connected over Bluetooth serial (an HC-05 on one of the STM32's
free UARTs — no firmware change needed, since the existing UART console would
just appear on the phone).

## Planned scope

- **HF text messaging over FreeDV data modes** (DATAC1/3/4), for traffic that
  needs to get through where SSB voice cannot: DATAC4 decodes at about -4 dB
  SNR against roughly +6 to +10 dB for voice. Unencrypted by design — the
  primary audience is licensed radio amateurs, where encryption is prohibited.
- **Phone-as-terminal for voice modes too**: mode select, TX, telemetry — the
  same commands the UART console already exposes.

## Why not build it in with encryption

Text is exact where a noisy voice link garbles names, coordinates and
quantities, and it logs itself. Encryption is a known future capability (e.g.
for an organisation with its own licensed network and a legal duty to protect
casualty data) but must not be a dependency of the amateur build, where it
would be illegal to use.

## Status

Design direction only — see [doc/architecture.md](../doc/architecture.md) for
the DSP core this app will talk to. Protocol layer (addressing,
acknowledgement, retransmission, queueing) and the app itself are both
unstarted.
