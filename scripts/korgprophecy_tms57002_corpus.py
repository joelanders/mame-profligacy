#!/usr/bin/env python3
"""Run a manifest of focused TMS57002 semantic tests against one core build."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "tests/korgprophecy/tms57002_known_semantics_v1.json"
PASS_RE = re.compile(r"^PASS\s+", re.MULTILINE)
FAIL_RE = re.compile(r"^FAIL\s+", re.MULTILINE)


def run_case(binary: Path, worktree: Path, case: dict[str, object]) -> dict[str, object]:
    mode = str(case["mode"])
    env = os.environ.copy()
    env.update(
        {
            "SDL_VIDEODRIVER": "dummy",
            "SDL_AUDIODRIVER": "dummy",
            "TMS57TEST_ONLY": mode,
        }
    )
    command = [
        str(binary),
        "tms57002test",
        "-video",
        "none",
        "-sound",
        "none",
        "-seconds_to_run",
        "1",
    ]
    completed = subprocess.run(
        command,
        cwd=worktree,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    output = completed.stdout
    passed_checks = len(PASS_RE.findall(output))
    failed_checks = len(FAIL_RE.findall(output))
    expected_checks = int(case["expected_checks"])
    normal_completion = (
        completed.returncode == 3
        and "Fatal error: Focused TMS57002 test completed" in output
    )
    passed = (
        normal_completion
        and failed_checks == 0
        and passed_checks == expected_checks
    )
    reasons: list[str] = []
    if not normal_completion:
        reasons.append(f"abnormal focused-test completion (exit {completed.returncode})")
    if failed_checks:
        reasons.append(f"{failed_checks} assertion(s) failed")
    if passed_checks + failed_checks != expected_checks:
        reasons.append(
            f"executed {passed_checks + failed_checks}/{expected_checks} expected assertions"
        )
    return {
        "id": case["id"],
        "mode": mode,
        "status": "PASS" if passed else "FAIL",
        "passed_checks": passed_checks,
        "failed_checks": failed_checks,
        "expected_checks": expected_checks,
        "exit_code": completed.returncode,
        "reasons": reasons,
        "output": output,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--worktree", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--label", default="core")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    binary = args.binary.expanduser().resolve()
    worktree = args.worktree.expanduser().resolve()
    manifest_path = args.manifest.expanduser().resolve()
    if not binary.is_file():
        parser.error(f"binary not found: {binary}")
    if not worktree.is_dir():
        parser.error(f"worktree not found: {worktree}")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    results = [run_case(binary, worktree, case) for case in manifest["cases"]]
    passed = sum(result["status"] == "PASS" for result in results)
    report = {
        "schema": "tms57002-known-semantics-run-v1",
        "suite_id": manifest["suite_id"],
        "label": args.label,
        "binary": str(binary),
        "worktree": str(worktree),
        "summary": {
            "cases": len(results),
            "passed": passed,
            "failed": len(results) - passed,
        },
        "results": results,
    }

    for result in results:
        print(
            f"{result['status']} {result['id']} "
            f"checks={result['passed_checks']}/{result['expected_checks']} "
            f"failed={result['failed_checks']} exit={result['exit_code']}"
        )
        if result["status"] != "PASS":
            for line in str(result["output"]).splitlines():
                if line.startswith("FAIL ") or line.startswith("Fatal error:"):
                    print(f"  {line}")
    print(
        f"TMS57002_CORPUS_SUMMARY label={args.label} cases={len(results)} "
        f"passed={passed} failed={len(results) - passed}"
    )

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
