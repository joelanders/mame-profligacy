#!/usr/bin/env python3
"""Deterministic interpreter-versus-pooled-dynarec acceptance gate.

Every render gets isolated config, snapshot, and NVRAM directories.  Equality
is byte-for-byte WAV equality; a discrepancy remains a failure and is recorded
in receipt.json.  Pooled runs must also prove that native frames compiled and
ran.  The unsafe-mode and forced-midframe cases have explicit runtime evidence,
so an interpreter fallback cannot accidentally satisfy this gate.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time


PF4_RE = re.compile(
    r"\[pf4\] calls=(\d+) runs=(\d+) fallbacks=(\d+) compiles=(\d+)"
    r"(?: forced_midframe=(\d+))?"
)
SPEED_RE = re.compile(r"Average speed:\s*([0-9.]+)%")
SIZE_RE = re.compile(r"\[pooled-size\] pooled frame: (\d+) bytes")
F5_BENIGN_RE = re.compile(
    r"\[pooled\] F5: branch pc ([0-9a-f]+) -> pc ([0-9a-f]+) .* "
    r"is BENIGN .* \(prog ([0-9a-f]+)\)",
    re.IGNORECASE,
)

CONTROLLED_ENV = {
    "KPROP_DSP_PERFRAME",
    "KPROP_PF4_STATS",
    "KPROP_PF4_MODE_UNSAFE_COMPILE",
    "KPROP_PF4_FORCE_MIDFRAME_FALLBACK",
    "KPROP_PF4_CMEM_DEOPT",
    "KPROP_PF4_FORCE_CMEM_UNSAFE",
    "KPROP_INJECT_NOTE",
}


@dataclass(frozen=True)
class RunSpec:
    name: str
    seconds: int
    perframe: int | None
    extra_env: dict[str, str] = field(default_factory=dict)
    extra_args: tuple[str, ...] = ()


@dataclass
class RunResult:
    name: str
    command: list[str]
    environment: dict[str, str]
    returncode: int
    wall_seconds: float
    average_speed_percent: float | None
    wav_bytes: int
    wav_sha256: str
    pooled_frame_sizes: list[int]
    max_runs: int
    max_fallbacks: int
    max_compiles: int
    max_forced_midframe: int
    benign_mode_refinements: list[str]
    log: str


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tree_fingerprint(root: Path | None) -> dict[str, object] | None:
    if root is None:
        return None
    rows: list[dict[str, object]] = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        rows.append(
            {
                "path": str(path.relative_to(root)),
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return {"payload_copied_to_receipt": False, "files": rows}


def sanitize_command(
    command: list[str], binary: Path, rompath: Path, output: Path
) -> list[str]:
    """Keep a reproducible command in receipts without local absolute paths."""
    sanitized: list[str] = []
    for argument in command:
        if argument == str(binary):
            sanitized.append("<binary>")
        elif argument == str(rompath):
            sanitized.append("<rom-directory>")
        elif argument.startswith(f"{output}{os.sep}"):
            relative = Path(argument).relative_to(output)
            sanitized.append(f"<output>/{relative.as_posix()}")
        else:
            sanitized.append(argument)
    return sanitized


def first_difference(left: Path, right: Path) -> int | None:
    offset = 0
    with left.open("rb") as lhs, right.open("rb") as rhs:
        while True:
            a = lhs.read(1024 * 1024)
            b = rhs.read(1024 * 1024)
            if a == b:
                if not a:
                    return None
                offset += len(a)
                continue
            common = min(len(a), len(b))
            for index in range(common):
                if a[index] != b[index]:
                    return offset + index
            return offset + common


def copy_nvram(seed: Path | None, destination: Path) -> None:
    if seed is None:
        destination.mkdir()
    else:
        shutil.copytree(seed, destination)


def parse_runtime(log_text: str) -> dict[str, object]:
    stats = [tuple(int(value or 0) for value in match.groups()) for match in PF4_RE.finditer(log_text)]
    speeds = [float(match.group(1)) for match in SPEED_RE.finditer(log_text)]
    return {
        "average_speed_percent": speeds[-1] if speeds else None,
        "pooled_frame_sizes": sorted({int(match.group(1)) for match in SIZE_RE.finditer(log_text)}),
        "max_runs": max((row[1] for row in stats), default=0),
        "max_fallbacks": max((row[2] for row in stats), default=0),
        "max_compiles": max((row[3] for row in stats), default=0),
        "max_forced_midframe": max((row[4] for row in stats), default=0),
        "benign_mode_refinements": [
            f"pc {match.group(1)} -> pc {match.group(2)} prog {match.group(3)}"
            for match in F5_BENIGN_RE.finditer(log_text)
        ],
    }


def run_one(
    spec: RunSpec,
    binary: Path,
    system: str,
    rompath: Path,
    nvram_seed: Path | None,
    output: Path,
) -> RunResult:
    run_dir = output / "runs" / spec.name
    run_dir.mkdir(parents=True)
    nvram = run_dir / "nvram"
    cfg = run_dir / "cfg"
    snapshot = run_dir / "snapshot"
    copy_nvram(nvram_seed, nvram)
    cfg.mkdir()
    snapshot.mkdir()
    wav = run_dir / "audio.wav"
    log = run_dir / "run.log"

    command = [
        str(binary),
        system,
        "-rompath",
        str(rompath),
        "-nvram_directory",
        str(nvram),
        "-cfg_directory",
        str(cfg),
        "-snapshot_directory",
        str(snapshot),
        "-video",
        "none",
        "-sound",
        "none",
        "-videodriver",
        "dummy",
        "-nothrottle",
        "-skip_gameinfo",
        "-seconds_to_run",
        str(spec.seconds),
        "-wavwrite",
        str(wav),
        *spec.extra_args,
    ]
    env = os.environ.copy()
    for name in CONTROLLED_ENV:
        env.pop(name, None)
    env.update(
        {
            "SDL_VIDEODRIVER": "dummy",
            "SDL_AUDIODRIVER": "dummy",
            "KPROP_PF4_STATS": "1",
        }
    )
    if spec.perframe is not None:
        env["KPROP_DSP_PERFRAME"] = str(spec.perframe)
    env.update(spec.extra_env)

    start = time.perf_counter()
    with log.open("w", encoding="utf-8") as sink:
        completed = subprocess.run(
            command,
            cwd=run_dir,
            env=env,
            stdout=sink,
            stderr=subprocess.STDOUT,
            check=False,
        )
    wall = time.perf_counter() - start
    log_text = log.read_text(encoding="utf-8", errors="replace")
    runtime = parse_runtime(log_text)
    if completed.returncode != 0:
        raise RuntimeError(f"{spec.name} exited {completed.returncode}; see {log}")
    if not wav.is_file() or wav.stat().st_size <= 44:
        raise RuntimeError(f"{spec.name} produced no usable WAV; see {log}")

    controlled = {
        name: env[name]
        for name in sorted(CONTROLLED_ENV | {"SDL_VIDEODRIVER", "SDL_AUDIODRIVER"})
        if name in env
    }
    return RunResult(
        name=spec.name,
        command=sanitize_command(command, binary, rompath, output),
        environment=controlled,
        returncode=completed.returncode,
        wall_seconds=round(wall, 6),
        wav_bytes=wav.stat().st_size,
        wav_sha256=sha256_file(wav),
        log=str(log.relative_to(output)),
        **runtime,
    )


def add_once(specs: list[RunSpec], spec: RunSpec) -> None:
    if all(existing.name != spec.name for existing in specs):
        specs.append(spec)


def build_specs(args: argparse.Namespace, dense_midi: Path) -> tuple[list[RunSpec], list[tuple[str, str, str]]]:
    specs: list[RunSpec] = []
    comparisons: list[tuple[str, str, str]] = []
    note_env = {"KPROP_INJECT_NOTE": "11.0:13.5:60:100"}

    if "boot" in args.cases:
        add_once(specs, RunSpec("boot.interp", args.boot_seconds, 0))
        add_once(specs, RunSpec("boot.default", args.boot_seconds, None))
        add_once(specs, RunSpec("boot.pf4", args.boot_seconds, 4))
        add_once(
            specs,
            RunSpec("boot.unsafe", args.boot_seconds, 4, {"KPROP_PF4_MODE_UNSAFE_COMPILE": "1"}),
        )
        comparisons.extend(
            [
                ("boot.default", "boot.interp", "shipping-default"),
                ("boot.pf4", "boot.interp", "forced-pf4"),
                ("boot.unsafe", "boot.interp", "unsafe-override"),
            ]
        )

    if "note" in args.cases:
        add_once(specs, RunSpec("note.interp", args.note_seconds, 0, note_env))
        add_once(specs, RunSpec("note.pf4", args.note_seconds, 4, note_env))
        add_once(
            specs,
            RunSpec(
                "note.unsafe",
                args.note_seconds,
                4,
                {**note_env, "KPROP_PF4_MODE_UNSAFE_COMPILE": "1"},
            ),
        )
        comparisons.extend(
            [
                ("note.pf4", "note.interp", "note-pf4"),
                ("note.unsafe", "note.interp", "note-unsafe"),
            ]
        )

    if "midfb" in args.cases:
        add_once(specs, RunSpec("note.interp", args.note_seconds, 0, note_env))
        add_once(
            specs,
            RunSpec(
                "note.midfb",
                args.note_seconds,
                4,
                {**note_env, "KPROP_PF4_FORCE_MIDFRAME_FALLBACK": str(args.midframe_pc)},
            ),
        )
        comparisons.append(("note.midfb", "note.interp", "forced-midframe"))

    if "cmem" in args.cases:
        add_once(specs, RunSpec("note.interp", args.note_seconds, 0, note_env))
        add_once(
            specs,
            RunSpec(
                "note.cmem_deopt",
                args.note_seconds,
                4,
                {**note_env, "KPROP_PF4_CMEM_DEOPT": "1"},
            ),
        )
        comparisons.append(("note.cmem_deopt", "note.interp", "cmem-deopt"))

    if "dense" in args.cases:
        dense_args = ("-midiin", str(dense_midi))
        add_once(specs, RunSpec("dense.interp", args.dense_seconds, 0, extra_args=dense_args))
        add_once(specs, RunSpec("dense.pf4", args.dense_seconds, 4, extra_args=dense_args))
        comparisons.append(("dense.pf4", "dense.interp", "dense-midi"))

    return specs, comparisons


def require_pooled(result: RunResult, failures: list[str]) -> None:
    if result.max_runs <= 0 or result.max_compiles <= 0 or not result.pooled_frame_sizes:
        failures.append(
            f"{result.name}: no native pooled execution proof "
            f"(runs={result.max_runs}, compiles={result.max_compiles}, sizes={result.pooled_frame_sizes})"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("cases", nargs="*", metavar="CASE")
    parser.add_argument("--binary", type=Path, default=Path("./propmin"))
    parser.add_argument("--system", default="korgprop")
    parser.add_argument("--rompath", type=Path, default=Path("../mame/00-roms"))
    parser.add_argument("--nvram-seed", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--boot-seconds", type=int, default=15)
    parser.add_argument("--note-seconds", type=int, default=15)
    parser.add_argument("--dense-seconds", type=int, default=30)
    parser.add_argument("--midframe-pc", type=lambda value: int(value, 0), default=0)
    parser.add_argument("--require-mode-conflict", action="store_true")
    parser.add_argument("--require-arm64", action="store_true")
    args = parser.parse_args()
    valid_cases = {"boot", "note", "midfb", "cmem", "dense"}
    unknown_cases = sorted(set(args.cases) - valid_cases)
    if unknown_cases:
        parser.error(f"unknown case(s): {', '.join(unknown_cases)}")
    if not args.cases:
        args.cases = ["boot", "note", "midfb", "cmem", "dense"]
    if ("note" in args.cases or "midfb" in args.cases or "cmem" in args.cases) and args.note_seconds < 14:
        parser.error("note/midfb/cmem cases require --note-seconds >= 14")

    binary = args.binary.expanduser().resolve()
    rompath = args.rompath.expanduser().resolve()
    output = args.output.expanduser().resolve()
    nvram_seed = args.nvram_seed.expanduser().resolve() if args.nvram_seed else None
    if not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error(f"binary is not executable: {binary}")
    if not rompath.is_dir():
        parser.error(f"ROM path is not a directory: {rompath}")
    if nvram_seed is not None and not nvram_seed.is_dir():
        parser.error(f"NVRAM seed is not a directory: {nvram_seed}")
    if args.require_arm64 and platform.machine().lower() not in {"arm64", "aarch64"}:
        parser.error(f"native ARM64 required, running on {platform.machine()}")
    try:
        output.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error(f"output already exists (stale results are forbidden): {output}")

    dense_midi = output / "dense.mid"
    if "dense" in args.cases:
        generator = Path(__file__).with_name("korgprophecy_gen_dense_midi.py")
        completed = subprocess.run([sys.executable, str(generator), str(dense_midi)], check=False)
        if completed.returncode != 0 or not dense_midi.is_file():
            print("FAIL could not generate deterministic dense MIDI", file=sys.stderr)
            return 2

    specs, comparisons = build_specs(args, dense_midi)
    results: dict[str, RunResult] = {}
    failures: list[str] = []
    print(f"PF4 gate: arch={platform.machine()} runs={len(specs)} output={output}")
    for spec in specs:
        try:
            result = run_one(spec, binary, args.system, rompath, nvram_seed, output)
        except RuntimeError as error:
            failures.append(str(error))
            print(f"  FAIL {error}")
            continue
        results[spec.name] = result
        speed = "?" if result.average_speed_percent is None else f"{result.average_speed_percent:.2f}%"
        print(
            f"  {spec.name}: wall={result.wall_seconds:.2f}s speed={speed} "
            f"runs={result.max_runs} compiles={result.max_compiles} "
            f"forced={result.max_forced_midframe} sha={result.wav_sha256[:12]}"
        )

    checks: list[dict[str, object]] = []
    for candidate_name, oracle_name, label in comparisons:
        candidate = results.get(candidate_name)
        oracle = results.get(oracle_name)
        if candidate is None or oracle is None:
            checks.append({"name": label, "pass": False, "reason": "missing run"})
            failures.append(f"{label}: missing run")
            continue
        candidate_wav = (output / candidate.log).with_name("audio.wav")
        oracle_wav = (output / oracle.log).with_name("audio.wav")
        offset = first_difference(candidate_wav, oracle_wav)
        passed = offset is None
        checks.append(
            {
                "name": label,
                "pass": passed,
                "candidate": candidate_name,
                "oracle": oracle_name,
                "first_difference": offset,
            }
        )
        if passed:
            print(f"  PASS {label}: byte-identical")
        else:
            failures.append(f"{label}: first WAV difference at byte {offset}")
            print(f"  FAIL {label}: first WAV difference at byte {offset}")

    for name, result in results.items():
        if name.endswith((".default", ".pf4", ".unsafe", ".midfb", ".cmem_deopt")):
            require_pooled(result, failures)
        if name.endswith(".interp") and (result.max_runs or result.max_compiles):
            failures.append(f"{name}: interpreter oracle unexpectedly executed pooled code")
        if name.endswith(".midfb") and result.max_forced_midframe <= 0:
            failures.append(f"{name}: forced midframe fallback recorded no runtime hits")

    # Before the F5 refinement, the unsafe override had to compile a frame that the
    # conservative run refused.  Once the refinement proves the known edge benign,
    # conservative PF4 is expected to compile the same frame, so there is deliberately
    # no unsafe-only code size.  Accept either form, but require explicit runtime evidence
    # that the mode-conflict path was exercised; byte equality above remains the oracle.
    for stem in ("boot", "note"):
        unsafe = results.get(f"{stem}.unsafe")
        safe = results.get(f"{stem}.pf4")
        if unsafe is None:
            continue
        if safe is None:
            failures.append(f"{stem}.unsafe: missing corresponding conservative PF4 run")
            continue
        added_sizes = sorted(set(unsafe.pooled_frame_sizes) - set(safe.pooled_frame_sizes))
        benign = safe.benign_mode_refinements
        exercised = bool(added_sizes or benign)
        checks.append(
            {
                "name": f"{stem}-mode-conflict-path-exercised",
                "pass": exercised if args.require_mode_conflict else None,
                "required": args.require_mode_conflict,
                "unsafe_only_pooled_frame_sizes": added_sizes,
                "benign_mode_refinements": benign,
            }
        )
        if added_sizes:
            print(f"  PASS {stem}-unsafe-native-compile: unsafe-only frame sizes={added_sizes}")
        elif benign:
            print(f"  PASS {stem}-F5-benign-native-compile: {', '.join(benign)}")
        elif args.require_mode_conflict:
            failures.append(f"{stem}: mode-conflict path produced neither F5-benign nor unsafe-only compile evidence")
        else:
            print(f"  SKIP {stem}-mode-conflict-path: current fixture has no conflict")

    receipt = {
        "schema": 1,
        "status": "PASS" if not failures else "FAIL",
        "arch": platform.machine(),
        "binary": {
            "name": binary.name,
            "size": binary.stat().st_size,
            "sha256": sha256_file(binary),
        },
        "system": args.system,
        "require_mode_conflict": args.require_mode_conflict,
        "rom_input": {
            "system": args.system,
            "payload_or_path_copied_to_receipt": False,
        },
        "nvram_seed": tree_fingerprint(nvram_seed),
        "cases": list(args.cases),
        "runs": {name: asdict(result) for name, result in sorted(results.items())},
        "checks": checks,
        "failures": failures,
    }
    receipt_path = output / "receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if failures:
        print(f"FAIL PF4 equivalence ({len(failures)} failures); receipt={receipt_path}")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print(f"PASS PF4 equivalence; receipt={receipt_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
