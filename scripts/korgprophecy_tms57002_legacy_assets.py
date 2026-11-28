#!/usr/bin/env python3
"""Audit the external assets used by the TMS57002 legacy replay tests."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "tests/korgprophecy/tms57002_legacy_assets_v1.json"
DEFAULT_SOURCE = ROOT / "src/mame/skeleton/tms57002test.cpp"
PATH_RE = re.compile(r'"((?:00-extracted|tmp)/[^"%]+)')


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--root", type=Path)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    assets = manifest["assets"]
    declared = {str(asset["asset_path"]) for asset in assets}
    source_paths = set(PATH_RE.findall(args.source.read_text(encoding="utf-8")))
    source_missing = sorted(source_paths - declared)
    source_stale = sorted(declared - source_paths)

    root = args.root
    if root is None:
        value = os.environ.get(str(manifest["bundle_env"]), "")
        root = Path(value) if value else None

    results = []
    for asset in assets:
        path = root / str(asset["bundle_path"]) if root else None
        expected_kind = str(asset["kind"])
        present = bool(
            path
            and (path.is_dir() if expected_kind == "directory" else path.is_file())
        )
        expected_hash = asset.get("sha256")
        actual_hash = sha256(path) if present and expected_kind == "file" else None
        hash_ok = expected_hash is None or actual_hash == expected_hash
        results.append({
            "asset_path": asset["asset_path"],
            "bundle_path": asset["bundle_path"],
            "resolved_path": str(path) if path else None,
            "present": present,
            "expected_sha256": expected_hash,
            "actual_sha256": actual_hash,
            "hash_ok": hash_ok,
        })

    present = sum(bool(result["present"]) for result in results)
    bad_hashes = sum(
        bool(result["present"] and not result["hash_ok"]) for result in results
    )
    inventory_ok = not source_missing and not source_stale
    complete = bool(root and present == len(results) and bad_hashes == 0 and inventory_ok)
    status = "PASS" if complete else "BLOCKED"
    report = {
        "schema": "tms57002-legacy-asset-audit-v1",
        "status": status,
        "root": str(root) if root else None,
        "summary": {
            "assets": len(results),
            "present": present,
            "missing": len(results) - present,
            "bad_hashes": bad_hashes,
            "source_inventory_ok": inventory_ok,
        },
        "source_missing_from_manifest": source_missing,
        "manifest_stale_paths": source_stale,
        "results": results,
    }

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    if source_missing:
        for path in source_missing:
            print(f"SOURCE_ONLY {path}")
    if source_stale:
        for path in source_stale:
            print(f"MANIFEST_ONLY {path}")
    if root:
        for result in results:
            if not result["present"]:
                print(f"MISSING {result['resolved_path']}")
            elif not result["hash_ok"]:
                print(f"HASH_MISMATCH {result['resolved_path']}")
    else:
        print(f"BLOCKED set {manifest['bundle_env']} or pass --root")
    print(
        f"TMS57002_LEGACY_ASSET_SUMMARY status={status} assets={len(results)} "
        f"present={present} missing={len(results) - present} "
        f"bad_hashes={bad_hashes} source_inventory={'PASS' if inventory_ok else 'FAIL'}"
    )
    return 0 if complete else 2


if __name__ == "__main__":
    sys.exit(main())
