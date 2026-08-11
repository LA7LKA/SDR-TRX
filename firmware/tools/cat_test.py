#!/usr/bin/env python3
"""CAT protocol regression test - exercises every command cat_exec()
implements (firmware/Core/Src/main.c), directly over the CDC-ACM port with
no Hamlib layer involved. This is the ground truth for what the firmware
actually does, byte for byte - the Hamlib backend (Hamlib/rigs/la7lka/,
not tracked in this repo, see .gitignore) sits on top of exactly this wire
protocol and should be spot-checked separately with rigctl once this
passes, since a bug here would otherwise look like a Hamlib-side bug.

Requires: pip install --user pyserial

Usage: python3 cat_test.py [/dev/ttyACM1]

Leaves the radio in a known-clean state when done (USB, 14.2 MHz, RX) -
safe to run against the live bench setup, but note it briefly keys PTT
(TX;/RX;) to verify transmit-status reporting.
"""
import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
results = []


def send(ser, cmd, expect_reply, timeout=1.0):
    """Send cmd (';' appended if missing), read a reply up to '\\n' if
    expect_reply, else just confirm nothing comes back."""
    if not cmd.endswith(";"):
        cmd += ";"
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()

    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            if b"\n" in buf:
                break
        else:
            time.sleep(0.02)

    return buf.decode("ascii", errors="replace").strip()


def check(name, cmd, expect_reply, predicate, note=""):
    reply = send(ser, cmd, expect_reply)
    ok = predicate(reply) if expect_reply else (reply == "")
    results.append((name, cmd, reply if reply else "(no reply)", "PASS" if ok else "FAIL", note))
    return reply


ser = serial.Serial(PORT, 9600, timeout=0.1)
time.sleep(0.3)
ser.reset_input_buffer()

# --- ID ---
check("Get ID", "ID", True, lambda r: r == "ID019;")

# --- Mode: set every digit, verify readback ---
mode_map = {"1": "LSB", "2": "USB", "3": "CW", "4": "FM", "5": "AM"}
for digit, name in mode_map.items():
    send(ser, f"MD{digit}", expect_reply=False)   # SET: no reply expected
    reply = send(ser, "MD", expect_reply=True)
    ok = reply == f"MD{digit};"
    results.append((f"Set+get mode {name}", f"MD{digit}; / MD;", reply, "PASS" if ok else "FAIL", ""))

# Invalid mode digit - should be silently ignored, previous mode (AM='5') unchanged
send(ser, "MD9", expect_reply=False)
reply = send(ser, "MD", expect_reply=True)
ok = reply == "MD5;"
results.append(("Invalid mode digit ignored", "MD9;", reply, "PASS" if ok else "FAIL",
                 "expect mode unchanged from last valid set (AM)"))

# Put mode back to USB for the rest of the tests / for a sane end state
send(ser, "MD2", expect_reply=False)

# --- Frequency: FA/FB get+set, aliasing ---
check("Get freq (FA)", "FA", True, lambda r: len(r) == 14 and r.startswith("FA") and r.endswith(";"))
send(ser, "FA00007100000", expect_reply=False)
reply = send(ser, "FA", expect_reply=True)
ok = reply == "FA00007100000;"
results.append(("Set+get freq via FA", "FA00007100000; / FA;", reply, "PASS" if ok else "FAIL", ""))

send(ser, "FB00014074000", expect_reply=False)
reply_fa = send(ser, "FA", expect_reply=True)
reply_fb = send(ser, "FB", expect_reply=True)
ok = reply_fa == "FA00014074000;" and reply_fb == "FB00014074000;"
results.append(("FB set aliases FA (single VFO)", "FB00014074000; / FA; / FB;",
                 f"{reply_fa} {reply_fb}", "PASS" if ok else "FAIL", ""))

# --- PTT / TX-RX / IF status ---
reply_rx = send(ser, "IF", expect_reply=True)
rx_ptt_ok = len(reply_rx) >= 15 and reply_rx[14] == "0"
results.append(("IF; reports RX (PTT=0) before keying", "IF;", reply_rx, "PASS" if rx_ptt_ok else "FAIL", ""))

send(ser, "TX", expect_reply=False)
reply_tx_if = send(ser, "IF", expect_reply=True)
tx_ok = len(reply_tx_if) >= 15 and reply_tx_if[14] == "1"
results.append(("TX; keys PTT, IF; reports PTT=1", "TX; / IF;", reply_tx_if, "PASS" if tx_ok else "FAIL", ""))

send(ser, "RX", expect_reply=False)
reply_rx2_if = send(ser, "IF", expect_reply=True)
rx2_ok = len(reply_rx2_if) >= 15 and reply_rx2_if[14] == "0"
results.append(("RX; un-keys PTT, IF; reports PTT=0", "RX; / IF;", reply_rx2_if, "PASS" if rx2_ok else "FAIL", ""))

# --- IF; full format sanity (freq + mode + ptt fields line up) ---
send(ser, "FA00021200000", expect_reply=False)
send(ser, "MD3", expect_reply=False)     # CW
reply_if = send(ser, "IF", expect_reply=True)
expected_if = "IF" + "00021200000" + "3" + "0" + ";"
if_ok = reply_if == expected_if
results.append(("IF; full field layout (freq+mode+ptt)", "FA00021200000;/MD3;/IF;",
                 f"{reply_if} (expected {expected_if})", "PASS" if if_ok else "FAIL", ""))

# --- Unknown command / empty line: silently ignored ---
check("Unknown command ignored", "ZZ", False, None)

# --- Mic gain: MG;/MGnnn;, range 1-200, 3 digits ---
send(ser, "MG150", expect_reply=False)
reply = send(ser, "MG", expect_reply=True)
ok = reply == "MG150;"
results.append(("Set+get mic gain", "MG150; / MG;", reply, "PASS" if ok else "FAIL", ""))

# --- CW keying speed: KS;/KSnn;, range 5-60, 2 digits ---
send(ser, "KS35", expect_reply=False)
reply = send(ser, "KS", expect_reply=True)
ok = reply == "KS35;"
results.append(("Set+get CW speed (WPM)", "KS35; / KS;", reply, "PASS" if ok else "FAIL", ""))

# --- CW pitch: PT;/PTnnnn;, range 300-1000, 4 digits ---
send(ser, "PT0550", expect_reply=False)
reply = send(ser, "PT", expect_reply=True)
ok = reply == "PT0550;"
results.append(("Set+get CW pitch (Hz)", "PT0550; / PT;", reply, "PASS" if ok else "FAIL", ""))

# Out-of-range pitch should clamp (cw_set_pitch() clamps internally to 300-1000)
send(ser, "PT9999", expect_reply=False)
reply = send(ser, "PT", expect_reply=True)
ok = reply == "PT1000;"
results.append(("CW pitch clamps to max (1000 Hz)", "PT9999; / PT;", reply, "PASS" if ok else "FAIL",
                 "cw_set_pitch() clamps internally"))

# --- Audio source: AS;/AS0;/AS1; ---
send(ser, "AS1", expect_reply=False)
reply = send(ser, "AS", expect_reply=True)
ok = reply == "AS1;"
results.append(("Set+get audio source USB", "AS1; / AS;", reply, "PASS" if ok else "FAIL", ""))

send(ser, "AS0", expect_reply=False)
reply = send(ser, "AS", expect_reply=True)
ok = reply == "AS0;"
results.append(("Set+get audio source analog", "AS0; / AS;", reply, "PASS" if ok else "FAIL", ""))

# --- CW message: KY; (get), KY<text>; (set + one-shot send) ---
# Message text persists across power cycles of the test only within this
# run - KY; just echoes whatever's currently stored, which may be left
# over from a previous manual test, not necessarily the firmware's
# power-on default ("LA7LKA DE LB2S LB2S LB2S"). Just confirm it's a
# well-formed, non-empty reply here; the round-trip below is the real test.
DEFAULT_MSG = "LA7LKA DE LB2S LB2S LB2S"
reply = send(ser, "KY", expect_reply=True)
ok = reply.startswith("KY") and reply.endswith(";") and len(reply) > 3
results.append(("Get current CW message (well-formed)", "KY;", reply, "PASS" if ok else "FAIL", ""))

# Speed it up so the one-shot send doesn't eat the test's wall-clock budget
send(ser, "KS30", expect_reply=False)

TEST_MSG = "TEST"
send(ser, f"KY{TEST_MSG}", expect_reply=False)
reply_tx_if = send(ser, "IF", expect_reply=True)
tx_ok = len(reply_tx_if) >= 15 and reply_tx_if[14] == "1"
results.append(("KY<text>; keys PTT (one-shot send starts)", f"KY{TEST_MSG}; / IF;",
                 reply_tx_if, "PASS" if tx_ok else "FAIL", ""))

# Poll until the one-shot send completes and PTT drops back to RX on its own
deadline = time.time() + 15.0
ptt = "1"
while time.time() < deadline:
    r = send(ser, "IF", expect_reply=True)
    if len(r) >= 15:
        ptt = r[14]
    if ptt == "0":
        break
    time.sleep(0.2)
results.append(("KY one-shot send auto-returns to RX", "IF; (polled)", f"PTT={ptt}",
                 "PASS" if ptt == "0" else "FAIL", "waited up to 15s"))

reply = send(ser, "KY", expect_reply=True)
ok = reply == f"KY{TEST_MSG};"
results.append(("KY message persists after send", "KY;", reply, "PASS" if ok else "FAIL", ""))

# --- restore a sane default end state: USB, RX (already RX), 20 m,
#     default mic gain / CW speed / pitch / audio source / CW message ---
send(ser, "MD2", expect_reply=False)
send(ser, "FA00014200000", expect_reply=False)
send(ser, "MG001", expect_reply=False)
send(ser, "KS20", expect_reply=False)
send(ser, "PT0700", expect_reply=False)
send(ser, "AS0", expect_reply=False)
send(ser, f"KY{DEFAULT_MSG}", expect_reply=False)  # restores text; also fires a one-shot send
time.sleep(0.5)
send(ser, "RX", expect_reply=False)  # cut it short, we only needed the text restored

ser.close()

# --- print results table ---
name_w = max(len(r[0]) for r in results)
cmd_w = max(len(r[1]) for r in results)
reply_w = max(len(r[2]) for r in results)
print(f"{'TEST':{name_w}}  {'COMMAND':{cmd_w}}  {'OBSERVED':{reply_w}}  RESULT  NOTE")
fails = 0
for name, cmd, reply, status, note in results:
    if status == "FAIL":
        fails += 1
    print(f"{name:{name_w}}  {cmd:{cmd_w}}  {reply:{reply_w}}  {status}  {note}")

print()
print(f"{len(results)} checks, {len(results)-fails} passed, {fails} failed")
sys.exit(1 if fails else 0)
