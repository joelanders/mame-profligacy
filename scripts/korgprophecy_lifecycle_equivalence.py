#!/usr/bin/env python3
"""Compare the boot-time DSP serial lifecycle of two Prophecy builds.

This is deliberately an equivalence gate, not a golden-generator.  A mismatch
stays red and reports its first divergent serial write.  Each run gets fresh
NVRAM/config directories and a dummy SDL backend so it cannot open a window.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


TRACE_RE = re.compile(
    r"KPROP_DSPSEROUT,T=(?P<time>[0-9.]+),TAG=(?P<tag>[^,]+),"
    r"EV=(?P<event>[^,]+).*?IDX=(?P<index>-?[0-9]+),"
    r"VAL=(?P<value>[0-9A-Fa-f]+)"
)

BASE_ENV = {
    "SDL_VIDEODRIVER": "dummy",
    "SDL_AUDIODRIVER": "dummy",
    "KPROP_V55_ADC_BANKSW_EXPERIMENT": "6",
    "KPROP_DSP_HOST_MAP": "1,2,3",
    "KPROP_DSP2_INPUT_ROUTE": "normal",
    "KPROP_DSP_PERFRAME": "0",
    "KPROP_TRACE_DSP_SERIAL_OUTPUT_TAG": "dsp1",
    "KPROP_TRACE_DSP_SERIAL_OUTPUT_START": "0.174",
    "KPROP_TRACE_DSP_SERIAL_OUTPUT_END": "0.177",
    "KPROP_TRACE_DSP_SERIAL_OUTPUT_MAX": "1000",
}


def parse_trace(path: Path) -> list[tuple[str, str, int, str]]:
    rows: list[tuple[str, str, int, str]] = []
    for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
        match = TRACE_RE.search(line)
        if match and "dsp1" in match.group("tag"):
            rows.append(
                (
                    match.group("time"),
                    match.group("event"),
                    int(match.group("index")),
                    match.group("value").upper().zfill(6),
                )
            )
    return rows


def run_build(binary: Path, rompath: Path, label: str) -> list[tuple[str, str, int, str]]:
    with tempfile.TemporaryDirectory(prefix=f"kprop-lifecycle-{label}-") as temp_name:
        temp = Path(temp_name)
        for name in ("cfg", "nvram", "snap"):
            (temp / name).mkdir()

        env = os.environ.copy()
        env.update(BASE_ENV)
        command = [
            str(binary),
            "korgprop",
            "-rompath",
            str(rompath),
            "-cfg_directory",
            str(temp / "cfg"),
            "-nvram_directory",
            str(temp / "nvram"),
            "-snapshot_directory",
            str(temp / "snap"),
            "-wavwrite",
            str(temp / f"{label}.wav"),
            "-seconds_to_run",
            "1",
            "-video",
            "none",
            "-sound",
            "none",
            "-videodriver",
            "dummy",
            "-nothrottle",
            "-skip_gameinfo",
            "-log",
        ]
        completed = subprocess.run(
            command,
            cwd=temp,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        error_log = temp / "error.log"
        if completed.returncode != 0:
            raise RuntimeError(
                f"{label} exited {completed.returncode}:\n{completed.stdout[-4000:]}"
            )
        if not error_log.is_file():
            raise RuntimeError(f"{label} did not produce {error_log}")
        rows = parse_trace(error_log)
        if not rows:
            raise RuntimeError(f"{label} produced no DSP1 serial trace rows")
        return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-bin", type=Path, required=True)
    parser.add_argument("--candidate-bin", type=Path, required=True)
    parser.add_argument("--rompath", type=Path, required=True)
    args = parser.parse_args()

    reference_bin = args.reference_bin.expanduser().resolve()
    candidate_bin = args.candidate_bin.expanduser().resolve()
    rompath = args.rompath.expanduser().resolve()
    for label, path in (
        ("reference binary", reference_bin),
        ("candidate binary", candidate_bin),
    ):
        if not path.is_file():
            parser.error(f"{label} not found: {path}")
    if not rompath.is_dir():
        parser.error(f"ROM path not found: {rompath}")

    try:
        reference = run_build(reference_bin, rompath, "reference")
        candidate = run_build(candidate_bin, rompath, "candidate")
    except RuntimeError as error:
        print(f"ERROR lifecycle_equivalence {error}")
        return 2

    common = min(len(reference), len(candidate))
    mismatch = next(
        (index for index in range(common) if reference[index] != candidate[index]),
        None,
    )
    if mismatch is None and len(reference) == len(candidate):
        print(
            "PASS lifecycle_equivalence "
            f"rows={len(reference)} trace_window=0.174..0.177"
        )
        return 0

    if mismatch is None:
        mismatch = common
    reference_row = reference[mismatch] if mismatch < len(reference) else None
    candidate_row = candidate[mismatch] if mismatch < len(candidate) else None
    print(
        "FAIL lifecycle_equivalence "
        f"reference_rows={len(reference)} candidate_rows={len(candidate)} "
        f"first_mismatch={mismatch}"
    )
    print(f"  reference={reference_row}")
    print(f"  candidate={candidate_row}")
    print("  oracle=reference equivalence; do not rebaseline this mismatch")
    return 1


if __name__ == "__main__":
    sys.exit(main())
