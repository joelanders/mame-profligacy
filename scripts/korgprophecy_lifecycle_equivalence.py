#!/usr/bin/env python3
"""Compare the boot-time DSP serial lifecycle of two Prophecy builds.

This is deliberately an equivalence gate, not a golden-generator.  A mismatch
stays red and reports its first divergent serial write.  Each run gets fresh
NVRAM/config directories and a dummy SDL backend so it cannot open a window.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
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


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_output(root: Path, *arguments: str) -> str:
    completed = subprocess.run(
        ["git", *arguments],
        cwd=root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if completed.returncode:
        raise RuntimeError(
            f"git {' '.join(arguments)} failed: {completed.stderr.strip()}"
        )
    return completed.stdout.strip()


def validate_source(
    root: Path,
    reference_commit: str,
    candidate_commit: str | None,
    allow_dirty: bool,
) -> dict[str, object]:
    top = Path(git_output(root, "rev-parse", "--show-toplevel")).resolve()
    if top != root:
        raise RuntimeError(f"source root is not the Git top level: {root}")
    head = git_output(root, "rev-parse", "HEAD")
    candidate = candidate_commit or head
    for label, commit in (("reference", reference_commit), ("candidate", candidate)):
        resolved = git_output(root, "rev-parse", f"{commit}^{{commit}}")
        if resolved != commit:
            raise RuntimeError(
                f"{label} source commit must be the full resolved hash: {resolved}"
            )
    if candidate != head:
        raise RuntimeError(
            f"candidate source commit {candidate} is not checked-out HEAD {head}"
        )
    status = git_output(root, "status", "--porcelain", "--untracked-files=no")
    diff = subprocess.run(
        ["git", "diff", "--binary", "HEAD"],
        cwd=root,
        check=True,
        stdout=subprocess.PIPE,
    ).stdout
    if status and not allow_dirty:
        raise RuntimeError(
            "source tree has tracked changes; commit them or use "
            "--allow-dirty-source for a diagnostic run"
        )
    return {
        "reference_commit": reference_commit,
        "candidate_commit": candidate,
        "candidate_tracked_tree_clean": not bool(status),
        "candidate_tracked_status_lines": status.splitlines(),
        "candidate_tracked_diff_sha256": hashlib.sha256(diff).hexdigest(),
    }


def rows_digest(rows: list[tuple[str, str, int, str]]) -> str:
    canonical = json.dumps(rows, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


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


def run_build(
    binary: Path, rompath: Path, system: str, label: str
) -> list[tuple[str, str, int, str]]:
    with tempfile.TemporaryDirectory(prefix=f"kprop-lifecycle-{label}-") as temp_name:
        temp = Path(temp_name)
        for name in ("cfg", "nvram", "snap"):
            (temp / name).mkdir()

        env = os.environ.copy()
        env.update(BASE_ENV)
        command = [
            str(binary),
            system,
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
    parser.add_argument("--system", default="korgprop")
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument("--reference-source-commit", required=True)
    parser.add_argument("--candidate-source-commit")
    parser.add_argument("--allow-dirty-source", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    started = datetime.now(timezone.utc).isoformat(timespec="microseconds")
    reference_bin = args.reference_bin.expanduser().resolve()
    candidate_bin = args.candidate_bin.expanduser().resolve()
    rompath = args.rompath.expanduser().resolve()
    source_root = args.source_root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    for label, path in (
        ("reference binary", reference_bin),
        ("candidate binary", candidate_bin),
    ):
        if not path.is_file():
            parser.error(f"{label} not found: {path}")
    if not rompath.is_dir():
        parser.error(f"ROM path not found: {rompath}")
    if output.exists():
        parser.error(f"output already exists: {output}")
    if not output.parent.is_dir():
        parser.error(f"output parent not found: {output.parent}")
    try:
        source = validate_source(
            source_root,
            args.reference_source_commit,
            args.candidate_source_commit,
            args.allow_dirty_source,
        )
    except (RuntimeError, subprocess.CalledProcessError) as error:
        parser.error(str(error))

    try:
        reference = run_build(reference_bin, rompath, args.system, "reference")
        candidate = run_build(candidate_bin, rompath, args.system, "candidate")
    except RuntimeError as error:
        print(f"ERROR lifecycle_equivalence {error}")
        return 2

    common = min(len(reference), len(candidate))
    mismatch = next(
        (index for index in range(common) if reference[index] != candidate[index]),
        None,
    )
    passed = mismatch is None and len(reference) == len(candidate)
    if mismatch is None and not passed:
        mismatch = common
    runner = Path(__file__).resolve()
    receipt = {
        "schema": "korgprophecy-lifecycle-equivalence-v2",
        "started_at": started,
        "completed_at": datetime.now(timezone.utc).isoformat(timespec="microseconds"),
        "source": source,
        "runner": {"name": runner.name, "sha256": sha256_file(runner)},
        "reference_binary": {
            "name": reference_bin.name,
            "size": reference_bin.stat().st_size,
            "sha256": sha256_file(reference_bin),
        },
        "candidate_binary": {
            "name": candidate_bin.name,
            "size": candidate_bin.stat().st_size,
            "sha256": sha256_file(candidate_bin),
        },
        "rom_input": {
            "system": args.system,
            "payload_or_path_copied_to_receipt": False,
        },
        "invocation": [
            "python3", runner.name,
            "--reference-bin", "<reference-binary>",
            "--candidate-bin", "<candidate-binary>",
            "--rompath", "<rom-directory>",
            "--system", args.system,
            "--reference-source-commit", args.reference_source_commit,
            "--candidate-source-commit", str(source["candidate_commit"]),
            "--output", "<new-receipt.json>",
        ],
        "trace_environment": BASE_ENV,
        "trace_window_s": [0.174, 0.177],
        "reference_rows": len(reference),
        "candidate_rows": len(candidate),
        "reference_rows_sha256": rows_digest(reference),
        "candidate_rows_sha256": rows_digest(candidate),
        "first_mismatch_index": mismatch,
        "passed": passed,
        "trace_payload_copied_to_receipt": False,
    }
    output.write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")

    if passed:
        print(
            "PASS lifecycle_equivalence "
            f"rows={len(reference)} trace_window=0.174..0.177 "
            f"receipt={output}"
        )
        return 0

    assert mismatch is not None
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
