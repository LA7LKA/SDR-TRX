# QRP Labs BPF/LPF reference designs

Component-value reference copied from QRP Labs' published kit documentation
(`doc/qrp-labs/bpf3.pdf`, `doc/qrp-labs/assembly_A4.pdf`), so this project's
own RX band-pass and TX low-pass filter banks (see
[doc/architecture.md](../doc/architecture.md#band-pass-filter-bank) and this
directory's [README](README.md) card map) don't re-derive values from
scratch — real calculators and a proven, published design get you very
close on the first cut for both topologies.

**Not a direct drop-in.** Both QRP Labs kits are one filter per single
amateur band. This project's BPF bank combines adjacent bands per filter
(6 filters covering the 10 HF bands — see the table in
[doc/architecture.md](../doc/architecture.md#band-pass-filter-bank)), so the
combined-band filters still need their own component values worked out
(wider bandwidth, different loaded Q). These per-band tables are the
starting reference/cross-check for that work, not values to solder in
directly, except where a filter ends up single-band anyway (e.g. if the TX
LPF bank stays one-per-band).

## Band-pass filter (RX preselector)

Double-tuned circuit, **2-pole** (two LC resonators, T1/T2, coupled through
one capacitor position C3/C4 — not 5-pole, easy to miscount from the PCB's 6
capacitor footprints since most of those are trimmers/transformer windings,
not independent filter poles). Target was ~1/10th center-frequency
bandwidth, <2 dB insertion loss. Topology: `IN -- T1(L+C1+C2) -- C3‖C4
(coupling) -- T2(L+C5+C6) -- OUT`, with C1/C6 as 30 pF trimmers for fine
centering and C2/C5 as the fixed padding capacitors.

| Band | C1, C6 (trimmer) | C2, C5 | C3 (coupling) | C4 (coupling) | T1, T2 (toroid, inductance, main turns) | Extra |
| --- | --- | --- | --- | --- | --- | --- |
| 160m | 30p | 820p | 56p | 22p | T50-2, 8.56µH, 7:40t | 12p |
| 80m | 30p | 470p | 39p | 10p | T37-2, 3.83µH, 6:30t | 12p |
| 60m | 30p | 220p | 10p | 5p | T37-2, 4.12µH, 6:31t | — |
| 40m | 30p | 150p | 10p | 5p | T37-6, 3.02µH, 6:31t | — |
| 30m | 30p | 100p | 8p | — | T37-6, 2.11µH, 5:25t | 10p |
| 20m (56p variant) | 30p | 56p | 6p | — | T37-6, 1.79µH, 4:24t | — |
| 20m (68p variant) | 30p | 68p | 6p | — | T37-6, 1.53µH, 4:21t | 10p |
| 17m | 30p | 47p | 4p | — | T37-6, 1.41µH, 3:19t | 10p |
| 15m | 30p | 33p | 3p | — | T37-6, 1.14µH, 3:18t | 10p |
| 12m | 30p | 22p | 3p | — | T37-6, 1.10µH, 3:17t | 10p |
| 10m | 30p | — | 2p | — | T37-6, 1.40µH, 3:16t | 10p |

C1/C6 trimmer is always 30 pF max. T1/T2 turns notation `main:coupling` —
e.g. 20m's "4:21t" means a T37-6 core, 21 turns on the main (thin-wire)
winding, 4 turns on the coupling (thick-wire) winding that sets the ~50 Ω
in/out impedance. Toroid colors: T50-2/T37-2 red, T37-6 yellow.

**Coupling capacitor (C3/C4) sets bandwidth vs. insertion loss** — the one
knob most worth re-deriving for this project's wider, combined-band
filters. QRP Labs' own measured trade-off tables (useful as a sanity check
on any re-derivation):

| Band | Coupling cap | Bandwidth | Insertion loss |
| --- | --- | --- | --- |
| 12m | 4pF | 3.46 MHz | 1.23 dB |
| 12m | 3pF (chosen) | 2.61 MHz | 1.17 dB |
| 12m | 2.5pF | 2.54 MHz | 1.35 dB |
| 12m | 2pF | 2.12 MHz | 1.80 dB |
| 40m | 15pF (chosen) | 0.779 MHz | 1.53 dB |
| 40m | 12pF | 0.643 MHz | 1.53 dB |
| 40m | 10pF | 0.550 MHz | 1.68 dB |
| 80m | 47pF (chosen) | 465 kHz | 1.27 dB |
| 80m | 42pF | 428 kHz | 1.12 dB |
| 80m | 32pF | 352 kHz | 1.00 dB |
| 80m | 22pF | 246 kHz | 2.27 dB |
| 80m | 12pF | 197 kHz | 5.45 dB |
| 160m | 78pF | 231 kHz | 1.17 dB |
| 160m | 68pF (chosen) | 226 kHz | 1.25 dB |
| 160m | 56pF | 180 kHz | 1.15 dB |

Wider bandwidth generally costs insertion loss, but not monotonically at
the extremes (80m's 22/12pF rows show loss rising sharply once bandwidth is
squeezed too far) — worth remembering when trading off the wider
combined-band filters this project actually needs.

**Measured performance of the built single-band kits** (for reference —
what the values above actually achieve in practice, not just theory):

| Band | Center freq | Bandwidth | Insertion loss |
| --- | --- | --- | --- |
| 160m | 1.867 MHz | 0.226 MHz | 1.45 dB |
| 80m | 3.540 MHz | 0.465 MHz | 1.27 dB |
| 60m | 5.243 MHz | 0.486 MHz | 1.48 dB |
| 40m | 7.207 MHz | 0.793 MHz | 1.53 dB |
| 30m | 9.891 MHz | 1.15 MHz | 1.35 dB |
| 20m | 14.15 MHz | 1.44 MHz | 1.75 dB |
| 17m | 18.22 MHz | 1.63 MHz | 1.95 dB |
| 15m | 21.00 MHz | 1.538 MHz | 1.10 dB |
| 12m | 25.53 MHz | 2.87 MHz | 1.55 dB |
| 10m | 28.99 MHz | 3.01 MHz | 2.52 dB |

**Bring-up/tuning method** (directly reusable regardless of final component
values): trimmer caps (C1/C6) first, then adjust toroid turn count if a
trimmer runs out of range — remove a turn if it maxes out "closed" (too
much inductance), add a turn if it maxes out "open" (too little). Wind a
few turns extra on the lower bands (160/80/60/40m) since removing a turn is
easy but adding one means an untidy splice — QRP Labs recommends +4 turns
over the target, then trim down while peaking signal strength.

## Low-pass filter (TX harmonic suppression)

**7-element** (4 shunt capacitors C1–C4 + 3 series inductors L1–L3,
classic Chebyshev pi-ladder = 7th order) — published Ed Wetherhold (W3NQN)
/ G-QRP club design, not a QRP Labs original, decades-proven for TX
harmonic suppression. Topology: `IN -- C1 -- L1 -- C2 -- L2 -- C3 -- L3 --
C4 -- OUT`, all shunt caps to ground. L1 and L3 use the same turns count;
L2 has more.

| Band | C1, C4 | C2, C3 | L1, L3 (turns, inductance) | L2 (turns, inductance) | Toroid |
| --- | --- | --- | --- | --- | --- |
| 160m | 820 pF | 2200 pF | 30t, 4.44µH | 34t, 5.61µH | T50-2 (red) |
| 80m | 470 pF | 1200 pF | 25t, 2.42µH | 27t, 3.01µH | T37-2 (red) |
| 60m | 680 pF | 1200 pF | 23t, 2.12µH | 24t, 2.30µH | T37-2 (red) |
| 40m | 270 pF | 680 pF | 21t, 1.38µH | 24t, 1.70µH | T37-6 (yellow) |
| 30m | 270 pF | 560 pF | 19t, 1.09µH | 20t, 1.26µH | T37-6 (yellow) |
| 20m | 180 pF | 390 pF | 16t, 773nH | 17t, 904nH | T37-6 (yellow) |
| 17m | 100 pF (110 pF ideal, 100 pF substituted) | 270 pF | 13t, 548nH | 15t, 668nH | T37-6 (yellow) |
| 15m | 82 pF | 220 pF | 12t, 444nH | 14t, 561nH | T37-6 (yellow) |
| 12m | 100 pF | 220 pF | 12t, 438nH | 13t, 515nH | T37-6 (yellow) |
| 10m | 56 pF | 150 pF | 10t, 303nH | 11t, 382nH | T37-6 (yellow) |

(6m/4m/2m/222MHz rows exist in the source kit but are omitted here — out
of scope for this project's HF-only Mk2, see
[[rf-hardware-chain]]/[doc/architecture.md](../doc/architecture.md) for why
6m is parked for a possible Mk3.)

Winding note: aim to fill ~90% of the core (330°), leaving a small gap
(~30°) between winding ends to limit end-to-end capacitance. All three
inductors on one filter card share one continuous length of supplied wire,
divided into three pieces.

## Source

QRP Labs kit page: `qrp-labs.com/bpfkit` (BPF), `qrp-labs.com/lpfkit`
(LPF). Full assembly manuals with photos, PCB layout and tuning walkthrough
saved locally at `doc/qrp-labs/bpf3.pdf` and `doc/qrp-labs/assembly_A4.pdf`.
