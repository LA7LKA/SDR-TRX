#!/usr/bin/python3
"""
Convert the sdrtrx_*.grc bench flows from a mono real-IF sound-card link to
a stereo I/Q link (L = I, R = Q), for the H7 I/Q adapter board.

  radio-IF out (generator -> radio ADC):
      X -> audio_sink(1ch)   becomes   X -> hilbert_fc -> complex_to_float -> audio_sink(2ch)
      i.e. the exact analytic (single-sided) version of the real IF the flow
      already produced, so every modulator upstream stays as it is.

  radio-IF in (radio DAC -> TX-verify demod):
      audio_source(1ch) -> hilbert_fc -> Y   becomes   audio_source(2ch) -> float_to_complex -> Y
      The I/Q pair replaces the Hilbert transform that used to synthesise Q;
      the old Hilbert block is left in place, disabled. A source that fed a
      real-valued block instead gets I back via complex_to_real.

Only enabled audio blocks on the bench sound card (the old Generic /
Generic_1 cards) are touched; headset (J380) and radio-USB (CARD=Audio)
blocks are left alone. Both converted ends point at a new `iq_dev` variable
(default: the Behringer UM2, ALSA card CODEC), so the card is set in one place.

The YAML is only parsed to decide what to change; the file itself is edited
as text, so GRC's own formatting is kept and diffs show just the change.

Usage: tools_iq_convert.py FILE.grc [...]   (rewrites in place, idempotent)
"""
import re
import sys

import yaml

IQ_DEV = "plughw:CARD=CODEC,DEV=0"
BENCH_CARD = re.compile(r"(CARD=)?Generic(_1)?,DEV=0")


def block_text(name, bid, params, xy, comment=""):
    quoted = "'" + comment.replace("'", "''") + "'"   # comments are free text - always quote
    p = {"affinity": "''", "alias": "''", "comment": quoted,
         "maxoutbuf": "'0'", "minoutbuf": "'0'", **params}
    lines = [f"- name: {name}", f"  id: {bid}", "  parameters:"]
    lines += [f"    {k}: {p[k]}" for k in sorted(p)]
    lines += ["  states:", "    bus_sink: false", "    bus_source: false",
              "    bus_structure: null", f"    coordinate: [{xy[0]}, {xy[1]}]",
              "    rotation: 0", "    state: enabled"]
    return "\n".join(lines) + "\n"


def conn_line(c):
    return f"- [{c[0]}, '{c[1]}', {c[2]}, '{c[3]}']"


class GrcText:
    """Minimal in-place editor for one .grc file's text."""

    def __init__(self, text):
        self.t = text

    def _span(self, name):
        m = re.search(rf"^- name: {re.escape(name)}\n", self.t, re.M)
        end = re.search(r"^(- name: |connections:)", self.t[m.end():], re.M)
        return m.start(), m.end() + end.start()

    def set_param(self, name, key, value):
        a, b = self._span(name)
        blk = re.sub(rf"^(    {key}: ).*$", lambda m: m.group(1) + value, self.t[a:b], count=1, flags=re.M)
        self.t = self.t[:a] + blk + self.t[b:]

    def disable(self, name):
        a, b = self._span(name)
        self.t = self.t[:a] + self.t[a:b].replace("    state: enabled", "    state: disabled", 1) + self.t[b:]

    def add_blocks(self, text):
        i = self.t.index("\nconnections:")
        self.t = self.t[:i].rstrip("\n") + "\n" + text + self.t[i:]

    def replace_conn(self, old, new):
        self.t = self.t.replace(conn_line(old) + "\n", (conn_line(new) + "\n") if new else "", 1)

    def add_conns(self, conns):
        i = self.t.index("connections:\n") + len("connections:\n")
        self.t = self.t[:i] + "".join(conn_line(c) + "\n" for c in conns) + self.t[i:]


def convert(path):
    text = open(path).read()
    g = yaml.safe_load(text)
    ed = GrcText(text)
    by_name = {b["name"]: b for b in g["blocks"]}
    conns = g["connections"]
    report, new_blocks, new_conns = [], "", []

    for b in g["blocks"]:
        p = b["parameters"]
        if b.get("states", {}).get("state", "enabled") != "enabled":
            continue
        if not BENCH_CARD.search(str(p.get("device_name", ""))):
            continue
        name, xy = b["name"], b["states"]["coordinate"]

        if b["id"] == "audio_sink" and p.get("num_inputs") == "1":
            hil, c2f = f"{name}_iq_hilbert", f"{name}_iq_c2f"
            new_blocks += block_text(hil, "hilbert_fc",
                                     {"beta": "'6.76'", "num_taps": "'257'", "win": "window.WIN_HAMMING"},
                                     (xy[0] - 360, xy[1] + 80), "real IF -> analytic, I/Q of the old mono IF")
            new_blocks += block_text(c2f, "blocks_complex_to_float", {"vlen": "'1'"}, (xy[0] - 180, xy[1] + 80))
            for c in conns:
                if c[2] == name and c[3] == "0":
                    ed.replace_conn(c, [c[0], c[1], hil, "0"])
            new_conns += [[hil, "0", c2f, "0"], [c2f, "0", name, "0"], [c2f, "1", name, "1"]]
            ed.set_param(name, "device_name", "iq_dev")
            ed.set_param(name, "num_inputs", "'2'")
            report.append(f"sink   {name}: -> hilbert -> complex_to_float -> 2ch iq_dev")

        elif b["id"] == "audio_source" and p.get("num_outputs") == "1":
            f2c = f"{name}_iq_f2c"
            new_blocks += block_text(f2c, "blocks_float_to_complex", {"vlen": "'1'"},
                                     (xy[0] + 220, xy[1] + 80), "L = I, R = Q from the radio's DAC pair")
            new_conns += [[name, "0", f2c, "0"], [name, "1", f2c, "1"]]
            for c in [c for c in conns if c[0] == name and c[1] == "0"]:
                tgt = by_name.get(c[2])
                if tgt is not None and tgt["id"] == "hilbert_fc":
                    ed.replace_conn(c, None)
                    for d in conns:
                        if d[0] == tgt["name"]:
                            ed.replace_conn(d, [f2c, "0", d[2], d[3]])
                    ed.disable(tgt["name"])
                    report.append(f"source {name}: 2ch iq_dev -> float_to_complex (replaces {tgt['name']})")
                else:
                    c2r = f"{name}_iq_c2r"
                    if c2r not in new_blocks:
                        new_blocks += block_text(c2r, "blocks_complex_to_real", {"vlen": "'1'"},
                                                 (xy[0] + 400, xy[1] + 80))
                        new_conns.append([f2c, "0", c2r, "0"])
                    ed.replace_conn(c, [c2r, "0", c[2], c[3]])
                    report.append(f"source {name}: 2ch iq_dev, {c[2]} gets I via complex_to_real")
            ed.set_param(name, "device_name", "iq_dev")
            ed.set_param(name, "num_outputs", "'2'")

    if report:
        if "iq_dev" not in by_name:
            new_blocks += block_text("iq_dev", "variable", {"value": f"'''{IQ_DEV}'''"}, (200, 12),
                                     "stereo I/Q sound card on the I/Q adapter board (L = I, R = Q)")
        ed.add_blocks(new_blocks)
        ed.add_conns(new_conns)
        open(path, "w").write(ed.t)
    return report


Q_TRIM_DB = -8.0   # measured 2026-10-25: UM2-L/adapter I path 8.0 dB down on Q, flat 1-18 kHz


def add_q_trim(path):
    """Insert a gain trim on Q between each converted generator's
    complex_to_float and its I/Q sink, set by a `q_trim_db` variable.

    Compensates the bench's analog I/Q gain imbalance ahead of the radio's
    ADC pair, so the radio sees a balanced I/Q stimulus. Q is turned down
    rather than I up so the sound card can't clip. -8.0 dB took image
    rejection from 7.3 dB to 44-48 dB across 6-15 kHz on the bench."""
    text = open(path).read()
    g = yaml.safe_load(text)
    names = {b["name"] for b in g["blocks"]}
    ed = GrcText(text)
    new_blocks, report = "", []
    for c in g["connections"]:
        src, port, sink = c[0], c[1], c[2]
        if not (src.endswith("_iq_c2f") and port == "1" and c[3] == "1"):
            continue
        trim = f"{sink}_iq_qtrim"
        if trim in names:
            continue
        xy = next(b["states"]["coordinate"] for b in g["blocks"] if b["name"] == src)
        new_blocks += block_text(trim, "blocks_multiply_const_vxx",
                                 {"const": "10**(q_trim_db/20)", "type": "float", "vlen": "'1'"},
                                 (xy[0] + 40, xy[1] + 100), "Q gain trim, see q_trim_db")
        ed.replace_conn(c, [src, "1", trim, "0"])
        ed.add_conns([[trim, "0", sink, "1"]])
        report.append(f"q trim {sink}: complex_to_float Q -> x10**(q_trim_db/20) -> {sink}:1")
    if report:
        if "q_trim_db" not in names:
            new_blocks += block_text("q_trim_db", "variable", {"value": f"'{Q_TRIM_DB}'"}, (360, 12),
                                     "I/Q gain balance - dB applied to Q before the I/Q sound card")
        ed.add_blocks(new_blocks)
        open(path, "w").write(ed.t)
    return report


if __name__ == "__main__":
    for path in sys.argv[1:]:
        print(f"{path}:")
        for line in (convert(path) + add_q_trim(path)) or ["(nothing to convert)"]:
            print("   ", line)
