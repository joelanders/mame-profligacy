#!/usr/bin/env python3
"""Generate a dense 'F6-style' MIDI load for the Prophecy busy-load benchmark.

Format-0 SMF, 480 tpqn, 120 bpm (1 beat = 0.5s, 1 tick = ~1.0417ms).
Content (all on MIDI channel 1 / status channel 0):
  - 8 notes/sec alternating across two octaves (mono synth -> constant retrigger,
    each retrigger = full envelope/coefficient reload)
  - CC1 (mod wheel) triangle ramp, one event every 20ms
  - pitch bend sine-ish sweep, one event every 20ms (interleaved 10ms after CC1)
  - channel aftertouch ramp every 40ms
Duration: 60s of events. midiin starts playback at emulated t=10s.
"""
import math, struct, sys

TPQN = 480
TICKS_PER_SEC = TPQN * 2  # 120 bpm

events = []  # (tick, bytes)

DUR = 60.0
# notes: 8/s, alternating pitches, 100ms gate
notes = [48, 60, 55, 67, 52, 64, 59, 71]
t = 2.0  # small pad after sequence start
i = 0
while t < DUR:
    n = notes[i % len(notes)]
    on = int(t * TICKS_PER_SEC)
    off = int((t + 0.100) * TICKS_PER_SEC)
    events.append((on, bytes([0x90, n, 100])))
    events.append((off, bytes([0x80, n, 64])))
    t += 0.125
    i += 1

# mod wheel every 20ms, triangle 0..127 with 2s period
t = 2.0
while t < DUR:
    phase = (t % 2.0) / 2.0
    v = int(127 * (2 * phase if phase < 0.5 else 2 - 2 * phase))
    events.append((int(t * TICKS_PER_SEC), bytes([0xB0, 1, v])))
    t += 0.020

# pitch bend every 20ms (offset 10ms), sine, +-4096
t = 2.010
while t < DUR:
    v = 8192 + int(4096 * math.sin(2 * math.pi * t / 3.0))
    v = max(0, min(16383, v))
    events.append((int(t * TICKS_PER_SEC), bytes([0xE0, v & 0x7F, (v >> 7) & 0x7F])))
    t += 0.020

# channel aftertouch every 40ms, saw 0..127 with 1.5s period
t = 2.005
while t < DUR:
    v = int(127 * ((t % 1.5) / 1.5))
    events.append((int(t * TICKS_PER_SEC), bytes([0xD0, v])))
    t += 0.040

events.sort(key=lambda e: e[0])

def vlq(n):
    out = [n & 0x7F]
    n >>= 7
    while n:
        out.append(0x80 | (n & 0x7F))
        n >>= 7
    return bytes(reversed(out))

track = bytearray()
track += vlq(0) + bytes([0xFF, 0x51, 0x03]) + struct.pack(">I", 500000)[1:]  # tempo 120bpm
prev = 0
for tick, ev in events:
    track += vlq(tick - prev) + ev
    prev = tick
track += vlq(0) + bytes([0xFF, 0x2F, 0x00])  # end of track

out = bytearray()
out += b"MThd" + struct.pack(">IHHH", 6, 0, 1, TPQN)
out += b"MTrk" + struct.pack(">I", len(track)) + track

path = sys.argv[1] if len(sys.argv) > 1 else "dense.mid"
with open(path, "wb") as f:
    f.write(out)
print(f"{path}: {len(events)} events over {DUR}s ({len(events)/DUR:.0f}/s), {len(out)} bytes")
