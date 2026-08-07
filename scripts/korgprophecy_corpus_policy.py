#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Enforce the public TMS57002 corpus classification and denominator contract."""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def load(relative: str) -> dict:
    with (ROOT / relative).open(encoding="utf-8") as source:
        return json.load(source)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def main() -> int:
    replays = load("tests/korgprophecy/tms57002_oracle_replays_v1.json")
    gaps = load("tests/korgprophecy/tms57002_oracle_gaps_v1.json")
    retired = load("tests/korgprophecy/tms57002_retired_research_tests_v1.json")
    legacy_assets = load("tests/korgprophecy/tms57002_legacy_assets_v1.json")

    replay_cases = replays["cases"]
    replay_ids = {case["id"] for case in replay_cases}
    require(len(replay_cases) == 554, "hardware replay denominator must remain 554")
    require(len(replay_ids) == 554, "hardware replay ids must be unique")

    expected_gap_summary = {
        "decoded_captures": 600,
        "promoted_exact": 554,
        "legacy_superseded": 42,
        "recapture_optional": 4,
        "release_blocking": 0,
    }
    require(gaps["summary"] == expected_gap_summary, "historical gap summary changed")
    require(len(gaps["cases"]) == 46, "historical gap ledger must preserve 46 rows")
    statuses = Counter(case.get("status") for case in gaps["cases"])
    require(statuses == {"legacy_superseded": 42, "recapture_optional": 4},
            f"historical gap status counts changed: {dict(statuses)}")

    expected_optional = {
        "dual-mac-to-domh-3",
        "dual-mpy-to-smhd-0",
        "dual-mpy-to-smhd-1",
        "dual-mpy-to-smld-0",
    }
    actual_optional = {
        case["test_name"] for case in gaps["cases"]
        if case["status"] == "recapture_optional"
    }
    require(actual_optional == expected_optional, "optional-recapture identity changed")
    for case in gaps["cases"]:
        require(case.get("release_blocking") is False,
                f"archived case re-entered release denominator: {case['hardware_run']}")
        require(case.get("wire_results"), f"mismatch evidence missing: {case['hardware_run']}")
        require(case.get("provenance", {}).get("source_files"),
                f"capture provenance missing: {case['hardware_run']}")
        require(all(source.get("sha256") for source in case["provenance"]["source_files"]),
                f"capture hash missing: {case['hardware_run']}")
        group = case.get("replacement_group")
        replacement_ids = gaps.get("replacement_groups", {}).get(group, [])
        require(bool(replacement_ids), f"replacement evidence missing: {case['hardware_run']}")
        unknown = set(replacement_ids) - replay_ids
        require(not unknown, f"replacement ids are not passing replay cases: {sorted(unknown)}")

    expected_retired = {
        "dsp3_a02_sine_su480003_snapshot_replay",
        "dsp3_a02_su480001_pc68_mac_matches_input",
        "dsp3_a02_su480002_input_sensitivity",
        "dsp3_a02_su480003_input_sensitivity",
        "dsp3_a02_su480004_input_sensitivity",
        "dsp3_a02_su480004_snapshot_replay",
        "dsp3_a02_su480005_input_sensitivity",
        "dsp3_a02_su480005_snapshot_replay",
        "dsp3_a02_su480006_input_sensitivity",
        "dsp3_a02_su480006_snapshot_replay",
        "dsp3_a02_su480082_snapshot_replay",
        "dsp3_a02_f8_ff_tail_chain",
    }
    retired_ids = {test["id"] for test in retired["tests"]}
    require(retired_ids == expected_retired, "retired research-test inventory changed")
    require(len(retired["tests"]) == 12, "retired research-test count must remain 12")
    require(all(test.get("release_denominator") is False for test in retired["tests"]),
            "retired research test re-entered release denominator")
    require(retired["summary"] == {
        "retired_tests": 12,
        "legacy_asset_dependent": 11,
        "incomplete_synthetic": 1,
        "release_denominator": 0,
    }, "retired research-test summary changed")
    require(legacy_assets["summary"].get("legacy_research_tests") == 11,
            "legacy asset-dependent test count changed")
    require(legacy_assets["summary"].get("public_release_denominator") == 0,
            "legacy assets re-entered public release denominator")

    docs = "\n".join(
        (ROOT / relative).read_text(encoding="utf-8")
        for relative in (
            "tests/korgprophecy/README.md",
            "PUBLIC_ACCEPTANCE.md",
            "PUBLIC_PROVENANCE.md",
        )
    ).lower()
    for misleading in (
        "46 decoded red gaps",
        "46 retained red gaps",
        "46 red gaps",
        "12 blocked focused tests",
        "other 46 decoded captures that do not pass",
    ):
        require(misleading not in docs,
                f"public documentation describes archived evidence as a current failure: {misleading}")

    print(
        "KORGPROPHECY_CORPUS_POLICY status=PASS promoted=554 "
        "legacy_superseded=42 recapture_optional=4 focused=23 retired=12 "
        "release_blockers=0"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyError, OSError, RuntimeError, ValueError) as error:
        print(f"KORGPROPHECY_CORPUS_POLICY status=FAIL error={error}", file=sys.stderr)
        sys.exit(1)
