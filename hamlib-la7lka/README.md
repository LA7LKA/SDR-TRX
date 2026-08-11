# Hamlib backend for the LA7LKA QRP SDR TRX

A [Hamlib](https://github.com/Hamlib/Hamlib) rig backend for this project's
own CAT control protocol, developed and tested against upstream Hamlib
commit `148feba9862feb7b7329aee72f2baf94f661b4b3` (2026-08-02). Not
committed as a Hamlib fork on purpose - see the CAT control memory notes /
project history for why (Hamlib's internal APIs aren't stable even within
its own `5.0.0~git` dev tree, so a maintained fork would need periodic
rebasing to avoid going stale; this patch is small enough that reapplying
it to a fresh checkout is easier than maintaining a fork).

Its own standalone backend family (`RIG_LA7LKA` = 45, model
`RIG_MODEL_LA7LKA_TRX` = 45001), not nested inside an existing family like
Kenwood - this radio speaks a small, self-designed, Kenwood-*flavored*
ASCII subset (`;`-terminated, `MD`/`FA`/`FB`/`TX`/`RX`/`IF`/`ID`/`MG`/`KS`/
`PT`/`AS`), not any real commercial rig's protocol, and shares no code
with `kenwood.c` or any other existing backend. See
`firmware/Core/Src/main.c`'s `cat_exec()` for the authoritative protocol
implementation - this backend's wire format must stay in sync with that.

Also exposes mic gain, CW keying speed, and CW pitch as the standard
Hamlib levels `RIG_LEVEL_MICGAIN`/`RIG_LEVEL_KEYSPD`/`RIG_LEVEL_CWPITCH`,
and audio source (analog ADC/DAC vs USB - a bench-setup concept with no
standard Hamlib equivalent) as a custom `AUDIOSRC` ext_level combo.

## Applying to a fresh Hamlib checkout

```sh
git clone https://github.com/Hamlib/Hamlib.git
cd Hamlib
git checkout 148feba9862feb7b7329aee72f2baf94f661b4b3   # or a later commit, if the patches still apply cleanly
git apply /path/to/hamlib-la7lka/hamlib.patch
git apply /path/to/hamlib-la7lka/hamlib-keyspd-nullderef-bugfix.patch
mkdir rigs/la7lka
cp /path/to/hamlib-la7lka/rigs-la7lka/la7lka.c rigs/la7lka/
cp /path/to/hamlib-la7lka/rigs-la7lka/Makefile.am rigs/la7lka/
./bootstrap && ./configure && make
```

`./bootstrap`/`./configure` are required (not just `make`) because
`configure.ac` changes. `tests/rigctl -l` should then list:

```
45001  LA7LKA  QRP SDR TRX  0.1  Beta  RIG_MODEL_LA7LKA_TRX
```

## Contents

- `hamlib.patch` - the diff against upstream `configure.ac`,
  `include/hamlib/riglist.h`, and `src/register.c` (registration
  plumbing only, ~22 lines total).
- `hamlib-keyspd-nullderef-bugfix.patch` - **an unrelated, genuine upstream
  bug fix**, found while testing `RIG_LEVEL_KEYSPD` here, not specific to
  this backend. `rig_set_level()` (`src/settings.c`) unconditionally calls
  `morse_data_handler_set_keyspd()` whenever any backend's `KEYSPD` level
  is set, dereferencing `STATE(rig)->morse_data_handler_priv_data` with no
  NULL check - that pointer is only ever allocated by
  `morse_data_handler_start()` (the `send_morse()`/morse-over-data path),
  so a plain `rigctl ... L KEYSPD nn` on ANY backend supporting `KEYSPD`
  segfaults `rigctl` outright if that subsystem was never started (the
  common case). Confirmed via `catchsegv` backtrace. Worth reporting/PRing
  upstream to Hamlib directly - tracked here for now since it's needed to
  use this backend's own `KEYSPD` support without crashing the client.
- `rigs-la7lka/la7lka.c` - the backend itself. Own `write_block()`/
  `read_string()` I/O (modeled on `rigs/dummy/netrigctl.c`'s pattern, not
  `kenwood_transaction()`), own `rig_caps`, no reuse of any `kenwood_*()`
  function.
- `rigs-la7lka/Makefile.am` - goes in the new `rigs/la7lka/` directory
  alongside `la7lka.c`.

## Testing

- `firmware/tools/cat_test.py` exercises the wire protocol directly
  (no Hamlib involved) - the ground-truth test.
- `tests/rigctl -m 45001 -r /dev/ttyACM1 -s 9600` (baud is cosmetic over
  the radio's USB CDC-ACM port) exercises this backend specifically -
  standard levels via `l`/`L` (e.g. `l MICGAIN`, `L KEYSPD 25`), the custom
  `AUDIOSRC` ext_level the same way (`l AUDIOSRC`, `L AUDIOSRC 1`).
- [grig](https://github.com/fillods/grig) (GUI Hamlib test client) works
  too, built from source against a local Hamlib checkout with this patch
  applied - note it needs `-p`/`--enable-ptt` on the command line, PTT is
  disabled by default in grig itself, unrelated to this backend.
