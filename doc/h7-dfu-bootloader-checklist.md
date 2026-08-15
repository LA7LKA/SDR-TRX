# H755 DFU bootloader — setup checklist

Decision (already made, see project memory): use STM32's **built-in ROM
DFU bootloader** in protected System Memory, not a custom bootloader.
Rationale: zero firmware risk (it's ST's own tested silicon code — a bug in
a *custom* bootloader can brick the device), standard tooling
(`dfu-util` / STM32CubeProgrammer), and — unlike a custom bootloader — it
needs no vector-table relocation in the application, since the app still
boots from its normal flash base address either way.

This is a checklist to work through, not a finished spec — several items
need confirming against ST's own docs for the H755 specifically before
wiring anything on hardware.

## 1. Confirm entry mechanism (AN2606, H755-specific table)

- Read **AN2606** ("STM32 microcontroller system memory boot mode"), the
  STM32H755 section specifically — confirms which BOOT pins/option bytes
  select System Memory (bootloader) vs. main Flash, and which interfaces
  the H7 System Bootloader actually answers on (USB DFU is one of several
  — UART, I2C, SPI, etc. — the H7 bootloader is not USB-only, unlike some
  older parts).
- Confirm whether H755 needs just `BOOT0` or also touches `BOOT1`/option
  bytes for the dual-core case — do not assume the F7's single-core
  BOOT0-only behavior carries over unchanged.

## 2. BOOT0 entry path on the NUCLEO-H755ZI-Q dev board

- Nucleo boards typically expose BOOT0 via a solder bridge/jumper, not a
  GPIO-controlled path — check the board's user manual (UM2408) for how
  BOOT0 is wired on this specific board before assuming it matches other
  Nucleos.
- For now, entering DFU mode by physical jumper + reset is fine for
  bring-up.

## 3. GPIO-controlled BOOT0 — a PCB requirement, not a firmware one

The end goal (per project decisions) is entering DFU mode via a console
command rather than a physical jumper — e.g. `bootloader` command pulls a
GPIO wired to BOOT0 high, then issues a self-reset, so BOOT0 reads high at
the reset sample point.

**This requires BOOT0 to be wired to a GPIO output on the actual PCB**,
not just the Nucleo dev board — flag this as a requirement for the next
main-board PCB revision (see project's PCB planning notes), not something
firmware alone can retrofit onto the Nucleo without extra wiring.

## 4. Dual-core wrinkle — resolve before implementing

CM7 is the boot processor by default; CM4 is held in reset until CM7
releases it (exact mechanism — option bytes vs. RCC register — needs
confirming against **RM0468**, don't assume from memory of other parts).

**Decision (updated): both cores must be field-upgradeable.** Earlier
draft of this doc leaned toward "CM4 ships fixed" on the grounds that
CM4's role was still TBD — that's resolved now: CM4 owns the HMI
(OLED/encoder/buttons) and the iambic keyer, real functionality an
owner/user would reasonably need bugfixes or updates for over the rig's
life, same as CM7's DSP side. So the rig's owner needs to be able to
field-upgrade *either* core's firmware without a programmer, via the same
DFU bootloader entry.

Open question still to resolve before implementing: does the ROM
bootloader handle writing both cores' flash banks in one DFU session, or
does updating both need two separate DFU passes (CM7 flash, then
release-and-flash CM4)? Confirm against RM0468/AN2606 and test on
hardware once BOOT0 entry itself is working.

## 5. Tooling

- `dfu-util` (Linux-native, already the general approach) or
  STM32CubeProgrammer (ST's own GUI/CLI tool, sometimes has better H7
  dual-bank/dual-core support — worth trying both).
- Confirm `lsusb` shows the expected DFU VID:PID (`0483:df11`, ST's
  standard bootloader ID) once BOOT0 entry is working on hardware.

## Explicit non-goals here

- Not building a custom bootloader — already decided against.
- Not doing the CM7/CM4 application port itself — this is bootloader-only,
  ahead of that work per the agreed sequencing.
