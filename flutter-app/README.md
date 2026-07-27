# Companion app

Not started. A Flutter app that turns a phone into the radio's screen and
keyboard, connected over Bluetooth Low Energy to an nRF52840 (a reused USB
dongle rather than a new part) bridging one of the STM32's free UARTs — see
[hardware/README.md](../hardware/README.md). No STM32 firmware change is
needed, since the existing UART console is what comes through the bridge
either way; unlike a classic-Bluetooth module (e.g. HC-05), the app side talks
BLE GATT — the Nordic UART Service (NUS), which is what the nRF52840 side
will run — rather than opening a plain OS serial port.

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
