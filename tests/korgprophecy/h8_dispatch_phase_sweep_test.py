#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""ROM-free unit tests for the Prophecy H8 dispatch-phase runner."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile


ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = ROOT / "scripts/korgprophecy_h8_dispatch_phase_sweep.py"
SPEC = importlib.util.spec_from_file_location("h8_dispatch_phase_sweep", RUNNER_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    check(RUNNER.parse_tick_set("3,1-2,2") == [1, 2, 3], "tick-set parsing")
    try:
        RUNNER.parse_tick_set("4-2")
    except ValueError:
        pass
    else:
        raise AssertionError("descending tick range was accepted")

    midi_zero, event_zero = RUNNER.make_midi(0)
    midi_one, event_one = RUNNER.make_midi(1)
    check(midi_zero.startswith(b"MThd"), "generated file lacks MIDI header")
    check(midi_zero != midi_one, "neighboring ticks generated identical MIDI")
    check(
        abs((event_one - event_zero) - 1.0 / RUNNER.TICKS_PER_SECOND) < 1e-12,
        "event time does not advance by one MIDI tick",
    )

    with tempfile.TemporaryDirectory(prefix="h8-phase-unit-") as temp_name:
        log = Path(temp_name) / "error.log"
        log.write_text(
            "KPROP_H8DSP,T=1.000000,EV=HOSTW,PC=1,A=2,SLOT=0,DSP=1,D=AA\n"
            "KPROP_H8DSP,T=2.000000,EV=HOSTW,PC=1,A=2,SLOT=1,DSP=2,D=B2\n"
            "KPROP_H8DSP,T=2.000001,EV=HOSTW,PC=1,A=2,SLOT=1,DSP=2,D=49\n"
            "KPROP_H8DSP,T=2.000002,EV=HOSTW,PC=1,A=2,SLOT=1,DSP=2,D=1C\n",
            encoding="utf-8",
        )
        check(
            RUNNER.parse_command_time(log, bytes.fromhex("B2491C"), 1) == 2.0,
            "command parser chose the wrong slot or byte group",
        )

    rows = [
        {"phase_tick": 0, "command_time_s": 1.000000},
        {"phase_tick": 1, "command_time_s": 1.004000},
        {"phase_tick": 2, "command_time_s": 1.000020},
    ]
    extra, excursions = RUNNER.classify_extra_ticks(rows, 4062.5)
    check(extra == [1], f"unexpected extra-pass classification: {extra}")
    check(excursions[1] > 3900.0, "extra-pass excursion was not retained")
    print("PASS: H8 dispatch phase runner unit tests")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
