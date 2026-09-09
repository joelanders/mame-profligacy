#!/usr/bin/env python3
"""Require every public Prophecy control/label token to have one disposition."""

from __future__ import annotations

import json
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "tests/korgprophecy/public_controls_v1.json"
SOURCES = (
    *sorted((ROOT / "scripts").glob("korgprophecy_*")),
    *sorted((ROOT / "src/devices/cpu/h8").glob("*")),
    *sorted((ROOT / "src/devices/cpu/nec").glob("*")),
    *sorted((ROOT / "src/devices/cpu/tms57002").rglob("*")),
    ROOT / "src/mame/korg/korgprophecy.cpp",
)
TOKEN = re.compile(r"KPROP_[A-Z0-9_]+")


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    occurrences: dict[str, list[str]] = {}
    for path in SOURCES:
        if not path.is_file() or path == MANIFEST or path.name == Path(__file__).name:
            continue
        try:
            source = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        for name in TOKEN.findall(source):
            occurrences.setdefault(name, []).append(str(path.relative_to(ROOT)))

    declared: dict[str, str] = {}
    duplicates: list[str] = []
    categories = manifest["categories"]
    for category, record in categories.items():
        for name in record["names"]:
            if name in declared:
                duplicates.append(name)
            declared[name] = category

    actual = set(occurrences)
    missing = sorted(actual - set(declared))
    stale = sorted(set(declared) - actual)
    failures = len(missing) + len(stale) + len(duplicates)
    for name in missing:
        print(f"UNCLASSIFIED {name} files={','.join(sorted(set(occurrences[name])))}")
    for name in stale:
        print(f"STALE {name}")
    for name in sorted(set(duplicates)):
        print(f"DUPLICATE {name}")
    counts = " ".join(
        f"{category}={len(record['names'])}"
        for category, record in categories.items()
    )
    print(
        f"KORGPROPHECY_CONTROL_AUDIT status={'PASS' if not failures else 'FAIL'} "
        f"tokens={len(actual)} failures={failures} {counts}"
    )
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
