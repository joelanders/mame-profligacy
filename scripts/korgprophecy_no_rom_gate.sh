#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PYTHON="${PYTHON:-python3}"

./scripts/tms57002test_build.sh
./scripts/korgprophecy_build.sh
exec "$PYTHON" scripts/korgprophecy_no_rom_gate.py "$@"
