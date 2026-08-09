#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright Joe Landers
"""Run every redistributable, no-ROM Prophecy publication gate."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
FOCUSED = re.compile(
    r"TMS57TEST_ALL_SUMMARY tests=23 passed=23 failed=0 "
    r"errored=0 blocked=0 failed_checks=0"
)
COMPACT = re.compile(
    r"TMS57002_CORPUS_SUMMARY label=no-rom-ci cases=10 passed=10 failed=0"
)
ORACLE = re.compile(
    r"TMS57002_ORACLE_SUMMARY label=no-rom-ci cases=554 passed=554 failed=0"
)


def run(command: list[str], *, env: dict[str, str] | None = None, capture: bool = False) -> str:
    print("+", " ".join(command), flush=True)
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=env,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
        text=True,
        check=False,
    )
    output = completed.stdout or ""
    if completed.returncode:
        if output:
            print(output[-12000:])
        raise RuntimeError(f"command exited {completed.returncode}: {' '.join(command)}")
    return output


def require(pattern: re.Pattern[str], output: str, name: str) -> None:
    match = pattern.search(output)
    if not match:
        print(output[-12000:])
        raise RuntimeError(f"{name} summary did not match the reviewed ledger")
    print(match.group(0))


def executable(name: str) -> str:
    for filename in (name, f"{name}.exe"):
        candidate = ROOT / filename
        if candidate.is_file() and (os.name == "nt" or os.access(candidate, os.X_OK)):
            return f"./{filename}"
    raise RuntimeError(f"missing executable: {name}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-oracle", action="store_true")
    parser.add_argument(
        "--content-only", action="store_true",
        help="skip local Git topology checks in detached CI clones",
    )
    args = parser.parse_args()

    audit = [sys.executable, "scripts/korgprophecy_publication_audit.py"]
    if args.content_only:
        audit.append("--content-only")
    run(audit)
    run([sys.executable, "scripts/korgprophecy_control_audit.py"])
    run([sys.executable, "scripts/korgprophecy_corpus_policy.py"])
    run(["git", "diff", "--check"])

    tms57test = executable("tms57test")
    propmin = executable("propmin")

    harness_env = os.environ.copy()
    harness_env.update(
        {
            "SDL_VIDEODRIVER": "dummy",
            "SDL_AUDIODRIVER": "dummy",
            "TMS57TEST_LEGACY_ASSET_ROOT": "/nonexistent/korgprophecy-public-assets",
            "TMS57TEST_ONLY": "all",
        }
    )
    focused = run(
        [
            tms57test, "tms57002test", "-video", "none", "-sound", "none",
            "-seconds_to_run", "1",
        ],
        env=harness_env,
        capture=True,
    )
    require(FOCUSED, focused, "focused suite")

    compact = run(
        [
            sys.executable, "scripts/korgprophecy_tms57002_corpus.py",
            "--binary", tms57test, "--worktree", ".", "--label", "no-rom-ci",
        ],
        capture=True,
    )
    require(COMPACT, compact, "compact corpus")

    if not args.skip_oracle:
        oracle = run(
            [
                sys.executable, "scripts/korgprophecy_tms57002_oracle_corpus.py", "run",
                "--binary", tms57test, "--worktree", ".", "--label", "no-rom-ci",
            ],
            capture=True,
        )
        require(ORACLE, oracle, "hardware-oracle corpus")

    validate_env = os.environ.copy()
    validate_env.update({"SDL_VIDEODRIVER": "dummy", "SDL_AUDIODRIVER": "dummy"})
    run([propmin, "-validate", "korgprop"], env=validate_env)
    print("KORGPROPHECY_NO_ROM_GATE status=PASS")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except RuntimeError as error:
        print(f"KORGPROPHECY_NO_ROM_GATE status=FAIL error={error}", file=sys.stderr)
        sys.exit(1)
