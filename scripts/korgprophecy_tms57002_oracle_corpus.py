#!/usr/bin/env python3
"""Derive and run repo-local TMS57002 hardware-trace replay fixtures."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FIXTURE = ROOT / "tests/korgprophecy/tms57002_oracle_replays_v1.json"
DEFAULT_ORACLE = Path(os.environ.get("TMS57002_ORACLE_ROOT", "00-dsp-oracle"))
DEFAULT_HARNESS = DEFAULT_ORACLE / "dsp-test-harness"
DEFAULT_INSTR = ROOT / "src/devices/cpu/tms57002/tmsinstr.lst"

SOURCE_RUN_NAMES = (
    "20260529-202302-dual-mac-to-domh-0",
    "20260529-202311-dual-mac-to-domh-1",
    "20260529-202321-dual-mac-to-domh-2",
    # The fourth MAC/DOMH capture needs five leading SO1 words discarded and
    # leaves only 27 comparable words.  Keep the three stronger siblings.
    "20260529-202358-dual-mpy-to-smhd-2",
    "20260529-202407-dual-mpy-to-smhd-3",
    # The zero/one-NOP SMHD captures contain unstable/anomalous SO1/SO0 words;
    # the explicitly seeded follow-up family below covers those store slots.
    # The zero-NOP SMLD run is retained as an optional fully declared recapture;
    # it is outside the release denominator and has stronger seeded replacements.
    "20260529-202425-dual-mpy-to-smld-1",
    "20260529-202434-dual-mpy-to-smld-2",
    "20260529-202444-dual-mpy-to-smld-3",
    "20260531-191345-followup-smhc-slot-mpy-read-0",
    "20260531-191355-followup-smhc-slot-mpy-read-1",
    "20260531-191406-followup-smhc-slot-mpy-read-2",
    "20260531-191416-followup-smhc-slot-mpy-read-3",
    "20260531-191427-followup-smhc-slot-mpy-read-4",
    "20260531-191438-followup-smhc-slot-mpy-read-5",
    "20260531-191448-followup-smhc-slot-mac-read-0",
    "20260531-191459-followup-smhc-slot-mac-read-1",
    "20260531-191509-followup-smhc-slot-mac-read-2",
    "20260531-191520-followup-smhc-slot-mac-read-3",
    "20260531-191530-followup-smhc-slot-mac-read-4",
    "20260531-191541-followup-smhc-slot-mac-read-5",
    "20260531-191557-followup-store-smhd-mpy-read-0",
    "20260531-191613-followup-store-smhd-mpy-read-1",
    "20260531-191629-followup-store-smhd-mpy-read-2",
    "20260531-191645-followup-store-smhd-mpy-read-3",
    "20260531-191701-followup-store-smhd-mpy-read-4",
    "20260531-191717-followup-store-smhd-mpy-read-5",
    "20260531-191733-followup-store-smld-mpy-read-0",
    "20260531-191907-followup-store-slmh-mpy-read-0",
    "20260531-191923-followup-store-slmh-mpy-read-1",
    "20260531-191939-followup-store-slmh-mpy-read-2",
    "20260531-191955-followup-store-slmh-mpy-read-3",
    "20260531-192012-followup-store-slmh-mpy-read-4",
    "20260531-192028-followup-store-slmh-mpy-read-5",
    "20260531-192044-followup-store-slml-mpy-read-0",
    "20260531-192100-followup-store-slml-mpy-read-1",
    "20260531-192116-followup-store-slml-mpy-read-2",
    "20260531-192132-followup-store-slml-mpy-read-3",
    "20260531-192148-followup-store-slml-mpy-read-4",
    "20260531-192204-followup-store-slml-mpy-read-5",
    "20260531-192220-followup-store-smhd-mac-read-0",
    "20260531-192236-followup-store-smhd-mac-read-1",
    "20260531-192252-followup-store-smhd-mac-read-2",
    "20260531-192308-followup-store-smhd-mac-read-3",
    "20260531-192324-followup-store-smhd-mac-read-4",
    "20260531-192340-followup-store-smhd-mac-read-5",
    "20260531-192356-followup-store-smld-mac-read-0",
    "20260531-192531-followup-store-slmh-mac-read-0",
    "20260531-192546-followup-store-slmh-mac-read-1",
    "20260531-192602-followup-store-slmh-mac-read-2",
    "20260531-192618-followup-store-slmh-mac-read-3",
    "20260531-192634-followup-store-slmh-mac-read-4",
    "20260531-192651-followup-store-slmh-mac-read-5",
    "20260531-192706-followup-store-slml-mac-read-0",
    "20260531-192722-followup-store-slml-mac-read-1",
    "20260531-192738-followup-store-slml-mac-read-2",
    "20260531-192754-followup-store-slml-mac-read-3",
    "20260531-192810-followup-store-slml-mac-read-4",
    "20260531-192827-followup-store-slml-mac-read-5",
    "20260604-140449-acc-order-direct-d61",
    "20260604-140455-acc-order-lacd-sacd-next",
    "20260604-140500-acc-order-zacc-sacd-next-zero",
    "20260604-140505-acc-order-zacc-nop-sacd-zero",
    "20260604-140510-acc-order-lacc-zacc-sacd-next-zero",
)

# These four captures were originally catalogued with the harness-requested
# ST1 value, but silicon arbitration established that the host-side ST1 upload
# never reached the chip.  Keep the stale recipe value visible in provenance
# while replaying the state the physical runs actually used.
MEASURED_STATE_CORRECTIONS = {
    run_name: {
        "field": "st1",
        "recipe_value": 0x000020,
        "measured_value": 0x000000,
        "evidence": (
            "notes/fable/2026-07-15_COMB_BUG_standalone_repro_and_ca_inc_lead.md"
            "#8-the-silicon-arbitration-same-night--w510-powered-on-a02-tail-runs-pulled--graded"
        ),
        "reason": "the requested ST1 upload did not land on silicon",
    }
    for run_name in (
        "20260531-183524-followup-a02-tail-observable-sine-ish",
        "20260531-183539-followup-a02-tail-observable-impulse-ish",
        "20260531-183555-followup-a02-tail-observable-positive",
        "20260531-183611-followup-a02-tail-observable-negative",
    )
}

# These same A02 captures started CLKIN while the target was released and
# before the first LRCK edge.  The 20-word program reaches IDLE during that
# interval, so silicon executes exactly one unsynchronised pass before frame
# zero.  Derive and validate this fact from each retained raw capture instead
# of relying on the replay implementation's incidental reset state.
MEASURED_PRE_SYNC_RUNS = frozenset(MEASURED_STATE_CORRECTIONS)


def source_runs(source_fixture: Path) -> list[tuple[str, str]]:
    """Return the committed capture selection, with the seed tranche as fallback."""
    if source_fixture.is_file():
        fixture = json.loads(source_fixture.read_text(encoding="utf-8"))
        return [
            (str(case["id"]), str(case["hardware_run"]))
            for case in fixture["cases"]
        ]
    return [
        (run_name.split("-", 2)[2], run_name)
        for run_name in SOURCE_RUN_NAMES
    ]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def decoded_words(path: Path) -> list[int]:
    report = json.loads(path.read_text(encoding="utf-8"))
    return [int(word["word"]) & 0x00FFFFFF for word in report["words"]]


def decoded_sides(path: Path) -> tuple[list[int], list[int]]:
    report = json.loads(path.read_text(encoding="utf-8"))
    left = [int(word["word"]) & 0x00FFFFFF for word in report["words"]
            if word.get("channel") == "L"]
    right = [int(word["word"]) & 0x00FFFFFF for word in report["words"]
             if word.get("channel") == "R"]
    return left, right


def measured_pre_sync_launch(
    run_name: str, run_dir: Path, program: list[int],
) -> dict[str, object] | None:
    if run_name not in MEASURED_PRE_SYNC_RUNS:
        return None

    capture_path = run_dir / "capture.lac"
    document = json.loads(capture_path.read_text(encoding="utf-8"))
    settings = document["Settings"]
    total_samples = int(settings["TotalSamples"])
    channels: dict[str, bytes] = {}
    for item in settings["CaptureChannels"]:
        name = str(item.get("ChannelName") or "").lower()
        samples = base64.b64decode(item.get("Samples") or "")
        if len(samples) != total_samples:
            raise ValueError(
                f"{run_name}: {name} has {len(samples)} samples, "
                f"expected {total_samples}"
            )
        channels[name] = samples

    required = {"clkin", "bcko", "lrcko", "pload", "rs"}
    missing = sorted(required - channels.keys())
    if missing:
        raise ValueError(f"{run_name}: launch capture lacks {', '.join(missing)}")

    def rising_edges(name: str, stop: int) -> list[int]:
        samples = channels[name]
        return [
            index for index in range(1, min(stop, len(samples)))
            if samples[index - 1] == 0 and samples[index] != 0
        ]

    lrck_rises = rising_edges("lrcko", total_samples)
    if not lrck_rises:
        raise ValueError(f"{run_name}: launch capture has no LRCK rising edge")
    first_lrck_rise = lrck_rises[0]
    clkin_rises = rising_edges("clkin", first_lrck_rise)
    bck_rises = rising_edges("bcko", first_lrck_rise)
    if any(value == 0 for value in channels["pload"][:first_lrck_rise]):
        raise ValueError(f"{run_name}: PLOAD was asserted before first LRCK")
    if any(value == 0 for value in channels["rs"][:first_lrck_rise]):
        raise ValueError(f"{run_name}: /RS was asserted before first LRCK")
    if not program or (int(program[-1]) & 0x00ffffff) != 0x00fc4000:
        raise ValueError(f"{run_name}: pre-SYNC program does not end in IDLE")
    if len(clkin_rises) < len(program):
        raise ValueError(
            f"{run_name}: only {len(clkin_rises)} pre-SYNC clocks for "
            f"{len(program)} instructions"
        )

    return {
        "pre_sync_passes": 1,
        "evidence": {
            "source_file": "capture.lac",
            "sha256": sha256(capture_path),
            "first_lrck_rising_sample": first_lrck_rise,
            "clkin_rising_edges_before_first_lrck": len(clkin_rises),
            "bck_rising_edges_before_first_lrck": len(bck_rises),
            "pload_released_before_first_lrck": True,
            "reset_released_before_first_lrck": True,
            "program_instructions_through_idle": len(program),
            "reason": (
                "released silicon executes the program once on CLKIN, then "
                "IDLE holds it until the first LRCK/SYNC"
            ),
        },
    }


def load_recipes(
    test_names: list[str], harness: Path, instr: Path, cache_root: Path | None,
) -> dict[str, dict[str, object]]:
    recipes: dict[str, dict[str, object]] = {}
    missing = []
    for test_name in test_names:
        cache_path = cache_root / test_name / "recipe.json" if cache_root else None
        if cache_path and cache_path.is_file():
            recipes[test_name] = json.loads(cache_path.read_text(encoding="utf-8"))
        else:
            missing.append(test_name)
    if not missing:
        return recipes

    # Import the large oracle harness once.  Starting it once per recipe turns
    # a mechanical corpus refresh into many minutes of repeated assembler setup.
    code = (
        "import json,sys; from dataclasses import asdict; import main; "
        "names=json.loads(sys.argv[1]); aliases={}; "
        "[aliases.__setitem__(recipe.name,asdict(recipe)) "
        "for suite in main.suite_names() for case in main.suite_cases(suite) "
        "for recipe in (case.build(),)]; "
        "print(json.dumps({name:aliases.get(name) or asdict(main.get_recipe(name)) "
        "for name in names}))"
    )
    env = dict(os.environ, TMS57002_INSTR_LST=str(instr))
    completed = subprocess.run(
        [sys.executable, "-c", code, json.dumps(missing)],
        cwd=harness,
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )
    recipes.update(json.loads(completed.stdout))
    return recipes


def derive_case(
    case_id: str,
    run_name: str,
    oracle: Path,
    harness: Path,
    instr: Path,
    recipe: dict[str, object],
) -> dict[str, object]:
    run_dir = oracle / run_name
    test_name = run_name.split("-", 2)[2]
    si0_path = run_dir / "serial-input-decode.json"
    si1_path = run_dir / "serial-input-si1-decode.json"
    si0_left, si0_right = decoded_sides(si0_path) if si0_path.is_file() else ([], [])
    si1_left, si1_right = decoded_sides(si1_path) if si1_path.is_file() else ([], [])
    if si0_left and si0_right:
        frame_count = min(len(si0_left), len(si0_right))
    else:
        frame_count = 0
    if si1_left and si1_right:
        frame_count = min(frame_count, len(si1_left), len(si1_right))
    else:
        si1_left = [0] * frame_count
        si1_right = [0] * frame_count
    input_frames = [
        [si0_left[index], si0_right[index], si1_left[index], si1_right[index]]
        for index in range(frame_count)
    ]

    output_paths = (run_dir / "serial-decode.json", run_dir / "serial-so1-decode.json")
    output_wires = []
    for path in output_paths:
        words = decoded_words(path) if path.is_file() else []
        output_wires.append(words[: len(words) & ~1])

    # Some output-only recipes have no physical SI channels in the capture.
    # Feed zeros for the same number of frames as the captured SO0 duration.
    if not input_frames:
        input_frames = [[0, 0, 0, 0] for _ in range(max(len(output_wires[0]) // 2, 1))]

    source_paths = tuple(path for path in (si0_path, si1_path, *output_paths)
                         if path.is_file())
    so1_driven = output_paths[1].is_file() and not (
        "smhc-slot-" in test_name
        or test_name.startswith("final-domh-")
        or test_name.startswith("followup-domh-")
        or test_name.startswith("mame-domh-")
        or test_name.startswith("mame-a02-tail-")
        or test_name.startswith("followup-a02-tail-force-")
    )
    correction = MEASURED_STATE_CORRECTIONS.get(run_name)
    st1 = int(recipe["st1"])
    if correction:
        expected_recipe_value = int(correction["recipe_value"])
        if st1 != expected_recipe_value:
            raise ValueError(
                f"{run_name}: recipe ST1 changed from the correction's recorded "
                f"0x{expected_recipe_value:06x} to 0x{st1:06x}"
            )
        st1 = int(correction["measured_value"])

    provenance: dict[str, object] = {
        "source_files": [
            {"name": path.name, "sha256": sha256(path)} for path in source_paths
        ]
    }
    if correction:
        provenance["measured_state_correction"] = correction

    launch = measured_pre_sync_launch(
        run_name, run_dir, [int(word) for word in recipe["program"]]
    )

    case: dict[str, object] = {
        "id": case_id,
        "hardware_run": run_name,
        "test_name": test_name,
        "program": recipe["program"],
        "st0": recipe["st0"],
        "st1": st1,
        "cmem": recipe.get("cmem") or [],
        "dmem0": recipe.get("dmem0") or [],
        "dmem1": recipe.get("dmem1") or [],
        "frame_clocks": round(int(recipe["clkin_hz"]) / int(recipe["lr_hz"])),
        "input_frames": input_frames,
        "output_wires": output_wires,
        # These recipes execute DOS on SO0 only.  SO1 is whatever the physical
        # output latch retained from the preceding bench program and is
        # intentionally provenance rather than a result of this recipe.
        "validated_wires": [0, 1] if so1_driven else [0],
        "alignment": {
            "max_hardware_skip_words": 4,
            "max_mame_skip_words": 8,
            # The exhaustive DOMH captures contain 23--25 decoded words.  All
            # 436 retain at least 23 exact words after zero/one-word alignment.
            "minimum_compared_words_per_wire": (
                20 if test_name.startswith("final-domh-")
                else 26 if test_name == "mame-parallel-dual-mpy-dis-oldnew"
                else 28
            ),
        },
        "provenance": provenance,
    }
    if launch:
        case["launch"] = launch
    return case


def derive_fixture(args: argparse.Namespace) -> int:
    selected_runs = (
        [(run_name.split("-", 2)[2], run_name) for run_name in args.hardware_run]
        if args.hardware_run
        else source_runs(args.source_fixture)
    )
    test_names = [run_name.split("-", 2)[2] for _case_id, run_name in selected_runs]
    recipes = load_recipes(test_names, args.harness_root, args.instr, args.cache_root)
    cases = [
        derive_case(
            case_id,
            run_name,
            args.oracle_root,
            args.harness_root,
            args.instr,
            recipes[run_name.split("-", 2)[2]],
        )
        for case_id, run_name in selected_runs
    ]
    fixture = {
        "schema": "tms57002-hardware-oracle-replays-v1",
        "suite_id": "hardware-oracle-replays-v1",
        "claim": (
            "Decoded SI/SO words and tiny programs are retained from physical "
            "TMS57002 captures; each case must replay word-exact after only "
            "bounded leading-state alignment."
        ),
        "cases": cases,
    }
    rendered = json.dumps(fixture, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0


def write_hex_words(path: Path, words: list[int], width: int, count: int) -> None:
    values = list(words[:count]) + [0] * max(0, count - len(words))
    mask = (1 << (width * 4)) - 1
    path.write_text(
        "".join(f"0x{value & mask:0{width}x}\n" for value in values),
        encoding="utf-8",
    )


def write_replay_inputs(work: Path, case: dict[str, object], total_frames: int) -> None:
    write_hex_words(work / "pmem.hex", list(case["program"]), 6, len(case["program"]))
    write_hex_words(work / "cmem.hex", list(case["cmem"]), 8, 256)
    write_hex_words(work / "dmem0.hex", list(case["dmem0"]), 8, 256)
    write_hex_words(work / "dmem1.hex", list(case["dmem1"]), 8, 256)
    frames = list(case["input_frames"])
    lines = ["frame\tsi0\tsi1\tsi2\tsi3"]
    for index in range(total_frames):
        frame = frames[index % len(frames)]
        lines.append(
            f"{index}\t" + "\t".join(f"0x{int(word) & 0xFFFFFF:06x}" for word in frame)
        )
    (work / "serial.tsv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (work / "cload.tsv").write_text("frame\tsa\tvalue\n", encoding="utf-8")


def read_mame_wires(path: Path) -> tuple[list[int], list[int]]:
    wires: tuple[list[int], list[int]] = ([], [])
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        fields = line.split("\t")
        if len(fields) < 9:
            continue
        outputs = [int(value, 16) & 0x00FFFFFF for value in fields[5:9]]
        wires[0].extend(outputs[0:2])
        wires[1].extend(outputs[2:4])
    return wires


def best_alignment(
    hardware: list[int],
    mame: list[int],
    max_hardware_skip: int,
    max_mame_skip: int,
) -> dict[str, int]:
    best = {"hardware_skip": 0, "mame_skip": 0, "matches": -1, "compared": 0}
    for hardware_skip in range(min(max_hardware_skip, len(hardware) - 1) + 1):
        for mame_skip in range(min(max_mame_skip, len(mame) - 1) + 1):
            compared = min(len(hardware) - hardware_skip, len(mame) - mame_skip)
            matches = sum(
                hardware[hardware_skip + index] == mame[mame_skip + index]
                for index in range(compared)
            )
            exact = matches == compared
            best_exact = best["matches"] == best["compared"] and best["compared"] > 0
            if (
                (exact and not best_exact)
                or (exact and best_exact and compared > best["compared"])
                or (
                    not exact
                    and not best_exact
                    and (
                        matches > best["matches"]
                        or (matches == best["matches"] and compared > best["compared"])
                    )
                )
            ):
                best = {
                    "hardware_skip": hardware_skip,
                    "mame_skip": mame_skip,
                    "matches": matches,
                    "compared": compared,
                }
    return best


def run_case(binary: Path, worktree: Path, case: dict[str, object]) -> dict[str, object]:
    hardware_wires = tuple(list(wire) for wire in case["output_wires"])
    total_frames = max(len(case["input_frames"]) + 16, 1)
    with tempfile.TemporaryDirectory(prefix="tms57002-oracle-") as temp:
        work = Path(temp)
        write_replay_inputs(work, case, total_frames)
        output_path = work / "mame.tsv"
        env = dict(
            os.environ,
            SDL_VIDEODRIVER="dummy",
            SDL_AUDIODRIVER="dummy",
            TMS57TEST_ONLY="filter-replay",
            TMS57TEST_REPLAY_PMEM=str(work / "pmem.hex"),
            TMS57TEST_REPLAY_CMEM=str(work / "cmem.hex"),
            TMS57TEST_REPLAY_DMEM0=str(work / "dmem0.hex"),
            TMS57TEST_REPLAY_DMEM1=str(work / "dmem1.hex"),
            TMS57TEST_REPLAY_SERIAL=str(work / "serial.tsv"),
            TMS57TEST_REPLAY_CLOAD=str(work / "cload.tsv"),
            TMS57TEST_REPLAY_OUT=str(output_path),
            TMS57TEST_REPLAY_EXECUTION="interpreter",
            TMS57TEST_REPLAY_ST0=f"{int(case['st0']):06x}",
            TMS57TEST_REPLAY_ST1=f"{int(case['st1']):06x}",
            TMS57TEST_REPLAY_FRAME_CLOCKS=str(case["frame_clocks"]),
            TMS57TEST_REPLAY_MAX_CYCLES=str(case["frame_clocks"]),
            TMS57TEST_REPLAY_TOTAL_FRAMES=str(total_frames),
            TMS57TEST_REPLAY_PRIME_SERIAL_ZERO="1",
            TMS57TEST_REPLAY_SYNC_RISING="0",
            TMS57TEST_REPLAY_WRITE_HANDOFF="1",
            TMS57TEST_REPLAY_PRE_SYNC_PASSES=str(
                int(dict(case.get("launch") or {}).get("pre_sync_passes", 0))
            ),
        )
        completed = subprocess.run(
            [str(binary), "tms57002test", "-video", "none", "-sound", "none",
             "-seconds_to_run", "1"],
            cwd=worktree,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        if not output_path.is_file():
            return {
                "id": case["id"], "status": "FAIL", "exit_code": completed.returncode,
                "reason": "MAME replay output missing", "wires": [],
                "output": completed.stdout + completed.stderr,
            }
        mame_wires = read_mame_wires(output_path)

    alignment = case["alignment"]
    wire_results = []
    passed = True
    validated_wires = [int(wire) for wire in case.get("validated_wires", [0, 1])]
    for wire in validated_wires:
        hardware = hardware_wires[wire]
        mame = mame_wires[wire]
        result = best_alignment(
            hardware,
            mame,
            int(alignment["max_hardware_skip_words"]),
            int(alignment["max_mame_skip_words"]),
        )
        result["wire"] = wire
        minimum = int(alignment["minimum_compared_words_per_wire"])
        result["minimum_compared"] = minimum
        result["exact"] = result["matches"] == result["compared"]
        result["passed"] = bool(result["exact"] and result["compared"] >= minimum)
        result["first_mismatches"] = [
            {
                "word": index,
                "hardware": hardware[result["hardware_skip"] + index],
                "mame": mame[result["mame_skip"] + index],
            }
            for index in range(result["compared"])
            if hardware[result["hardware_skip"] + index]
            != mame[result["mame_skip"] + index]
        ][:4]
        passed = passed and bool(result["passed"])
        wire_results.append(result)
    return {
        "id": case["id"],
        "hardware_run": case["hardware_run"],
        "status": "PASS" if passed else "FAIL",
        "exit_code": completed.returncode,
        "wires": wire_results,
    }


def run_fixture(args: argparse.Namespace) -> int:
    fixture = json.loads(args.fixture.read_text(encoding="utf-8"))
    selected = [
        case for case in fixture["cases"]
        if not args.case or str(case["id"]) in args.case
    ]
    missing = sorted(set(args.case) - {str(case["id"]) for case in selected})
    if missing:
        raise SystemExit(f"unknown fixture case(s): {', '.join(missing)}")
    results = [run_case(args.binary.resolve(), args.worktree.resolve(), case)
               for case in selected]
    passed = sum(result["status"] == "PASS" for result in results)
    for result in results:
        detail = " ".join(
            f"SO{wire['wire']}={wire['matches']}/{wire['compared']}"
            f"(hw+{wire['hardware_skip']},mame+{wire['mame_skip']})"
            for wire in result["wires"]
        )
        print(f"{result['status']} {result['id']} {detail}")
        if result["status"] == "FAIL":
            for wire in result["wires"]:
                for mismatch in wire["first_mismatches"]:
                    print(
                        f"  SO{wire['wire']} word={mismatch['word']} "
                        f"hardware=0x{mismatch['hardware']:06x} "
                        f"mame=0x{mismatch['mame']:06x}"
                    )
    print(
        f"TMS57002_ORACLE_SUMMARY label={args.label} cases={len(results)} "
        f"passed={passed} failed={len(results) - passed}"
    )
    if args.json_out:
        report = {
            "schema": "tms57002-hardware-oracle-run-v1",
            "suite_id": fixture["suite_id"],
            "label": args.label,
            "binary": str(args.binary.resolve()),
            "worktree": str(args.worktree.resolve()),
            "summary": {"cases": len(results), "passed": passed,
                        "failed": len(results) - passed},
            "results": results,
        }
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if passed == len(results) else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    derive = subparsers.add_parser("derive")
    derive.add_argument("--oracle-root", type=Path, default=DEFAULT_ORACLE)
    derive.add_argument("--harness-root", type=Path, default=DEFAULT_HARNESS)
    derive.add_argument("--instr", type=Path, default=DEFAULT_INSTR)
    derive.add_argument("--cache-root", type=Path)
    derive.add_argument("--source-fixture", type=Path, default=DEFAULT_FIXTURE)
    derive.add_argument("--hardware-run", action="append", default=[])
    derive.add_argument("--output", type=Path)

    run = subparsers.add_parser("run")
    run.add_argument("--binary", type=Path, required=True)
    run.add_argument("--worktree", type=Path, required=True)
    run.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    run.add_argument("--label", default="core")
    run.add_argument("--json-out", type=Path)
    run.add_argument("--case", action="append", default=[])

    args = parser.parse_args()
    if args.command == "derive":
        return derive_fixture(args)
    return run_fixture(args)


if __name__ == "__main__":
    sys.exit(main())
