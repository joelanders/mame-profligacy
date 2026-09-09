#!/usr/bin/env python3
"""Audit the publishable Prophecy delta and its reachable Git topology."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_BASE = "a60f95ea04a4f9b4bd950c88068cce1dcdc04b6e"
OFFICIAL_FETCH = "https://github.com/mamedev/mame.git"
# Alpha.4 was published with GitHub's merge-commit button despite the release
# procedure requiring a fast-forward.  Treat that already-public commit as the
# immutable history baseline, but continue rejecting any new merge commits so
# the mistake cannot silently repeat.
PUBLIC_HISTORY_BASE = "b801549f45ddfffba79dd8403f592254af8a3764"

ALLOWED_EXACT = {
    ".github/workflows/ci-linux.yml",
    ".github/workflows/ci-macos.yml",
    ".github/workflows/ci-windows.yml",
    ".github/workflows/korgprophecy-no-rom.yml",
    "PUBLIC_ACCEPTANCE.md",
    "PUBLIC_PROVENANCE.md",
    "scripts/src/cpu.lua",
    "scripts/src/main.lua",
    "scripts/tms57002test_build.sh",
    # Reviewed, source-compiled HD44780 A00 datasheet reconstruction used by
    # korgprophecy.cpp so the public build does not require a redistributed ROM.
    "src/devices/video/hd44780.cpp",
    "src/devices/video/hd44780.h",
    # Host-selected mixer delivery period; the default preserves MAME's cadence.
    "src/emu/sound.cpp",
    "src/osd/osdepend.h",
    "src/mame/korg/korgprophecy.cpp",
    "src/mame/mame.lst",
    "src/mame/skeleton/tms57002test.cpp",
}
ALLOWED_PREFIXES = (
    "scripts/korgprophecy_",
    "src/devices/cpu/h8/",
    "src/devices/cpu/nec/",
    "src/devices/cpu/tms57002/",
    "tests/korgprophecy/",
)
FORBIDDEN_PATH_PARTS = (
    "tms57002_poc_passes.inc",
    "prophecy_hle/",
    "00-extracted/",
    "00-pdfs/",
    "notes/",
)
FORBIDDEN_SUFFIXES = (
    ".7z", ".bin", ".dmg", ".iso", ".pdf", ".rar", ".rom", ".tar",
    ".tgz", ".wav", ".zip",
)
FORBIDDEN_CONTENT = (
    b"/Users/",
    b"/home/",
    b"mame-fuck",
    b"mame-cleanv2-probe",
    b"DO NOT COMMIT",
    b"DO NOT DISTRIBUTE",
    b"tms57002_poc_passes.inc",
    b"prophecy_hle",
)
CONTENT_MARKER_DOCUMENTS = {
    "PUBLIC_PROVENANCE.md",
    "scripts/korgprophecy_publication_audit.py",
}


def git(*args: str, check: bool = True) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        ["git", *args], cwd=ROOT, capture_output=True, check=check,
    )


def text(*args: str, check: bool = True) -> str:
    return git(*args, check=check).stdout.decode("utf-8", errors="replace").strip()


def allowed(path: str) -> bool:
    return path in ALLOWED_EXACT or path.startswith(ALLOWED_PREFIXES)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cached", action="store_true",
        help="audit the staged snapshot rather than HEAD",
    )
    parser.add_argument(
        "--content-only", action="store_true",
        help="skip local branch/remote topology checks (for detached CI clones)",
    )
    args = parser.parse_args()

    failures: list[str] = []
    reference = ":" if args.cached else "HEAD"
    diff_args = ["diff", "--cached"] if args.cached else ["diff"]
    paths = text(*diff_args, "--name-only", "--diff-filter=ACMR", OFFICIAL_BASE)
    delta_paths = [path for path in paths.splitlines() if path]

    if text("merge-base", OFFICIAL_BASE, "HEAD") != OFFICIAL_BASE:
        failures.append("HEAD does not descend from the pinned official base")

    if text("merge-base", PUBLIC_HISTORY_BASE, "HEAD") != PUBLIC_HISTORY_BASE:
        failures.append("HEAD does not descend from the current public history baseline")

    merge_commits = text("rev-list", "--merges", f"{PUBLIC_HISTORY_BASE}..HEAD")
    if merge_commits:
        failures.append("new merge commits exist above the public history baseline")

    if not args.content_only:
        heads = set(text("for-each-ref", "--format=%(refname:short)", "refs/heads").splitlines())
        if "profligacy-public-v1" not in heads:
            failures.append("publication branch is missing")
        unexpected_heads = heads - {"master", "profligacy-public-v1"}
        if unexpected_heads:
            failures.append(f"unexpected local branches: {', '.join(sorted(unexpected_heads))}")
        if "master" in heads and text("rev-parse", "master") != text("rev-parse", "upstream/master"):
            failures.append("local master does not match official upstream/master")

        fetch = text("config", "--get", "remote.upstream.url", check=False)
        push = text("config", "--get", "remote.upstream.pushurl", check=False)
        if fetch != OFFICIAL_FETCH:
            failures.append(f"unexpected upstream fetch URL: {fetch or '(none)'}")
        if push != "DISABLED":
            failures.append(f"upstream push URL is not disabled: {push or '(none)'}")

    for path in delta_paths:
        if not allowed(path):
            failures.append(f"path outside publication allowlist: {path}")
        if any(part in path for part in FORBIDDEN_PATH_PARTS):
            failures.append(f"forbidden path: {path}")
        if path.lower().endswith(FORBIDDEN_SUFFIXES):
            failures.append(f"forbidden asset extension: {path}")

        spec = f":{path}" if args.cached else f"HEAD:{path}"
        blob = git("show", spec, check=False)
        if blob.returncode:
            failures.append(f"cannot read snapshot blob: {path}")
            continue
        if path not in CONTENT_MARKER_DOCUMENTS:
            for marker in FORBIDDEN_CONTENT:
                if marker.lower() in blob.stdout.lower():
                    failures.append(
                        f"forbidden content marker {marker.decode(errors='replace')!r}: {path}"
                    )

    status = "PASS" if not failures else "FAIL"
    for failure in failures:
        print(f"FAIL {failure}")
    print(
        f"KORGPROPHECY_PUBLICATION_AUDIT status={status} "
        f"snapshot={'index' if args.cached else 'HEAD'} paths={len(delta_paths)} "
        f"failures={len(failures)}"
    )
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
