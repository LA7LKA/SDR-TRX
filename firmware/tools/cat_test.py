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

# --- restore a sane default end state: USB, RX (already RX), 20 m ---
send(ser, "MD2", expect_reply=False)
send(ser, "FA00014200000", expect_reply=False)
send(ser, "RX", expect_reply=False)

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
