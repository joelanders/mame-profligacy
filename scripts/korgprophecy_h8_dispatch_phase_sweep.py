#!/usr/bin/env python3
"""Sweep one control event across a complete H8 service period.

The runner intentionally keeps ROM images, program dumps, and the expected DSP
command outside the source tree.  It generates only a short standard MIDI file,
runs an explicitly supplied Prophecy binary, and records when the requested
five-byte H8 host command reaches the selected DSP slot.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
from datetime import datetime, timezone
from pathlib import Path


TPQN = 32767
TICKS_PER_SECOND = TPQN * 2
DEFAULT_PERIOD_US = 4062.5
HOST_WRITE = re.compile(
    r"KPROP_H8DSP,T=([0-9.]+),EV=HOSTW,PC=[0-9A-F]+,A=[0-9A-F]+,"
    r"SLOT=([0-9]+),DSP=([0-9]+),D=([0-9A-F]{2})"
)


def vlq(value: int) -> bytes:
    encoded = bytearray((value & 0x7F,))
    value >>= 7
    while value:
        encoded.insert(0, 0x80 | (value & 0x7F))
        value >>= 7
    return bytes(encoded)


def parse_tick_set(specification: str) -> list[int]:
    ticks: set[int] = set()
    for item in specification.split(","):
        item = item.strip()
        if not item:
            continue
        if "-" in item:
            first_text, last_text = item.split("-", 1)
            first, last = int(first_text), int(last_text)
            if last < first:
                raise ValueError(f"descending tick range: {item}")
            ticks.update(range(first, last + 1))
        else:
            ticks.add(int(item))
    return sorted(ticks)


def make_midi(phase_tick: int) -> tuple[bytes, float]:
    event_tick = round(7.0 * TICKS_PER_SECOND) + phase_tick
    events = [
        (0, bytes.fromhex("FF510307A120")),
        (round(5.5 * TICKS_PER_SECOND), bytes((0xB0, 2, 32))),
        (round(6.0 * TICKS_PER_SECOND), bytes((0x90, 84, 100))),
        (event_tick, bytes((0xB0, 2, 96))),
        (round(11.5 * TICKS_PER_SECOND), bytes((0x80, 84, 0))),
        (round(12.0 * TICKS_PER_SECOND), bytes.fromhex("FF2F00")),
    ]
    track = bytearray()
    previous = 0
    for when, payload in events:
        track += vlq(when - previous) + payload
        previous = when
    midi = (
        b"MThd"
        + struct.pack(">IHHH", 6, 0, 1, TPQN)
        + b"MTrk"
        + struct.pack(">I", len(track))
        + track
    )
    return midi, 10.0 + event_tick / TICKS_PER_SECOND


def parse_command_time(log: Path, command: bytes, slot: int) -> float:
    writes: list[tuple[float, int, int]] = []
    with log.open(errors="replace") as source:
        for line in source:
            match = HOST_WRITE.search(line)
            if match and int(match.group(2)) == slot:
                writes.append(
                    (float(match.group(1)), int(match.group(3)), int(match.group(4), 16))
                )
    for index in range(len(writes) - len(command) + 1):
        group = writes[index : index + len(command)]
        if bytes(item[2] for item in group) == command:
            return group[0][0]
    raise RuntimeError(f"command {command.hex()} not found in {log}")


def classify_extra_ticks(
    rows: list[dict[str, object]], period_us: float
) -> tuple[list[int], list[float]]:
    """Return phase ticks that take a later pass and each row's excursion."""
    command_times = [float(row["command_time_s"]) for row in rows]
    suffix_min = [0.0] * len(command_times)
    minimum = float("inf")
    for index in range(len(command_times) - 1, -1, -1):
        minimum = min(minimum, command_times[index])
        suffix_min[index] = minimum
    excursions_us = [
        (command_time - future_minimum) * 1_000_000.0
        for command_time, future_minimum in zip(command_times, suffix_min)
    ]
    extra_ticks = [
        int(row["phase_tick"])
        for row, excursion in zip(rows, excursions_us)
        if excursion > period_us / 2.0
    ]
    return extra_ticks, excursions_us


def run_phase(args: argparse.Namespace, phase_tick: int) -> dict[str, object]:
    directory = args.output / f"tick-{phase_tick:03d}"
    directory.mkdir()
    for child in ("cfg", "nvram", "snap"):
        (directory / child).mkdir()
    midi, event_time = make_midi(phase_tick)
    midi_path = directory / "stimulus.mid"
    midi_path.write_bytes(midi)

    environment = os.environ.copy()
    environment.update(
        {
            "SDL_AUDIODRIVER": "dummy",
            "SDL_VIDEODRIVER": "dummy",
            "KPROP_INJECT_SYSEX": f"{args.program}:11.5",
            "KPROP_TRACE_H8_CONTROL_EVENTS": "1",
            # The normal command can land less than 3 ms after the MIDI event
            # over part of the service period.  Begin just before the event so
            # the measurement window cannot censor the low-latency branch.
            "KPROP_TRACE_H8_CONTROL_START": f"{event_time - 0.001:.9f}",
            "KPROP_TRACE_H8_CONTROL_END": f"{event_time + 0.014:.9f}",
        }
    )
    command_line = [
        str(args.binary),
        args.system,
        "-rompath",
        str(args.rompath),
        "-cfg_directory",
        "cfg",
        "-nvram_directory",
        "nvram",
        "-snapshot_directory",
        "snap",
        "-midiin",
        midi_path.name,
        "-seconds_to_run",
        f"{args.seconds_to_run:g}",
        "-video",
        "none",
        "-nothrottle",
        "-log",
    ]
    return_codes: list[int] = []
    stderr_paths: list[Path] = []
    for attempt in range(args.retries + 1):
        stderr_path = directory / f"stderr-attempt-{attempt + 1}.log"
        stderr_paths.append(stderr_path)
        with stderr_path.open("wb") as stderr:
            completed = subprocess.run(
                command_line,
                cwd=directory,
                env=environment,
                stdout=subprocess.DEVNULL,
                stderr=stderr,
                check=False,
            )
        return_codes.append(completed.returncode)
        if not completed.returncode:
            break
        if attempt < args.retries:
            for child in ("cfg", "nvram", "snap"):
                shutil.rmtree(directory / child)
                (directory / child).mkdir()
    if return_codes[-1]:
        raise RuntimeError(
            f"tick {phase_tick}: emulator exits {return_codes}; "
            f"stderr retained at {stderr_paths[-1]}"
        )
    command_time = parse_command_time(directory / "error.log", args.command, args.slot)
    latency_us = (command_time - event_time) * 1_000_000.0
    if not args.keep_logs:
        for child in ("cfg", "nvram", "snap"):
            shutil.rmtree(directory / child)
        (directory / "error.log").unlink()
        for stderr_path in stderr_paths:
            stderr_path.unlink()
    return {
        "phase_tick": phase_tick,
        "phase_us": phase_tick * 1_000_000.0 / TICKS_PER_SECOND,
        "event_time_s": event_time,
        "command_time_s": command_time,
        "latency_us": latency_us,
        "midi_sha256": hashlib.sha256(midi).hexdigest(),
        "emulator_return_codes": return_codes,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--rompath", type=Path, required=True)
    parser.add_argument("--program", type=Path, required=True)
    parser.add_argument("--command-hex", required=True)
    parser.add_argument(
        "--expect-extra-ticks",
        help="comma-separated ticks or inclusive ranges, for example 147-149",
    )
    parser.add_argument("--slot", type=int, default=1)
    parser.add_argument("--system", default="korgpro20")
    parser.add_argument("--period-us", type=float, default=DEFAULT_PERIOD_US)
    parser.add_argument("--first-tick", type=int, default=0)
    parser.add_argument("--last-tick", type=int)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--retries", type=int, default=1)
    parser.add_argument("--seconds-to-run", type=float, default=22.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--keep-logs", action="store_true")
    args = parser.parse_args()

    args.binary = args.binary.resolve()
    args.rompath = args.rompath.resolve()
    args.program = args.program.resolve()
    args.output = args.output.resolve()
    try:
        args.command = bytes.fromhex(args.command_hex)
    except ValueError as error:
        parser.error(f"invalid --command-hex: {error}")
    if not args.command:
        parser.error("--command-hex must not be empty")
    if not args.binary.is_file() or not os.access(args.binary, os.X_OK):
        parser.error(f"binary is not executable: {args.binary}")
    if not args.rompath.is_dir():
        parser.error(f"ROM path is not a directory: {args.rompath}")
    if not args.program.is_file():
        parser.error(f"program file not found: {args.program}")
    if args.slot < 0 or args.slot > 2:
        parser.error("--slot must be between 0 and 2")
    if args.period_us <= 0.0:
        parser.error("--period-us must be positive")
    if args.jobs <= 0:
        parser.error("--jobs must be positive")
    if args.seconds_to_run <= 0.0:
        parser.error("--seconds-to-run must be positive")
    if args.retries < 0:
        parser.error("--retries must be non-negative")
    try:
        expected_extra_ticks = (
            None
            if args.expect_extra_ticks is None
            else parse_tick_set(args.expect_extra_ticks)
        )
    except ValueError as error:
        parser.error(f"invalid --expect-extra-ticks: {error}")
    period_last_tick = int(args.period_us * TICKS_PER_SECOND / 1_000_000.0)
    last_tick = period_last_tick if args.last_tick is None else args.last_tick
    if args.first_tick < 0 or last_tick < args.first_tick or last_tick > period_last_tick:
        parser.error(
            "tick range must satisfy 0 <= first-tick <= last-tick <= period"
        )
    if expected_extra_ticks is not None and any(
        tick < args.first_tick or tick > last_tick for tick in expected_extra_ticks
    ):
        parser.error("--expect-extra-ticks contains a tick outside the selected range")
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    args.output.mkdir(parents=True)
    ticks = tuple(range(args.first_tick, last_tick + 1))
    started = datetime.now(timezone.utc).isoformat(timespec="microseconds")
    rows: list[dict[str, object]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = {executor.submit(run_phase, args, tick): tick for tick in ticks}
        for future in concurrent.futures.as_completed(futures):
            row = future.result()
            rows.append(row)
            print(
                f"tick {row['phase_tick']:03d} phase={row['phase_us']:9.3f} us "
                f"latency={row['latency_us']:9.3f} us",
                flush=True,
            )
    rows.sort(key=lambda row: int(row["phase_tick"]))
    latencies = [float(row["latency_us"]) for row in rows]
    # A normal service boundary advances monotonically to a later command slot.
    # The missed-dispatch race is different: a narrow phase island uses a later
    # slot, then a still-later stimulus returns to the earlier slot.  Compare
    # each command time with the earliest command time at or after that phase;
    # an excursion larger than half a service period is an extra-pass island.
    extra_ticks, excursions_us = classify_extra_ticks(rows, args.period_us)
    receipt = {
        "schema": "korgprophecy-h8-dispatch-full-phase-sweep-v2",
        "started_at": started,
        "completed_at": datetime.now(timezone.utc).isoformat(timespec="microseconds"),
        "binary": str(args.binary),
        "binary_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        "period_us": args.period_us,
        "first_tick": args.first_tick,
        "last_tick": last_tick,
        "midi_tick_us": 1_000_000.0 / TICKS_PER_SECOND,
        "command_hex": args.command.hex(),
        "slot": args.slot,
        "minimum_latency_us": min(latencies),
        "maximum_latency_us": max(latencies),
        "classification_method": "command-time suffix-minimum excursion",
        "classification_threshold_us": args.period_us / 2.0,
        "classification_largest_excursion_us": max(excursions_us, default=0.0),
        "extra_pass_ticks": extra_ticks,
        "expected_extra_pass_ticks": expected_extra_ticks,
        "expectation_matches": (
            None if expected_extra_ticks is None else extra_ticks == expected_extra_ticks
        ),
        "rows": rows,
    }
    (args.output / "phase-sweep-receipt.json").write_text(
        json.dumps(receipt, indent=2) + "\n"
    )
    print(f"extra-pass ticks: {extra_ticks}")
    if expected_extra_ticks is not None and extra_ticks != expected_extra_ticks:
        print(f"expected extra-pass ticks: {expected_extra_ticks}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
