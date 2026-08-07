#!/usr/bin/env bash
# Compatibility entry point for the deterministic PF4 acceptance gate.
# The output directory must be new; stale evidence is rejected, never overwritten.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

cmd=(
	/usr/bin/python3 scripts/korgprophecy_pf4_equivalence.py
	--binary "${KPROP_GATE_MAME:-./propmin}"
	--rompath "${KPROP_GATE_ROMPATH:-../mame/00-roms}"
	--output "${KPROP_GATE_OUT:-/tmp/kprop_pf4_gate}"
)

if [[ -d "${KPROP_GATE_NVRAM:-}" ]]; then
	cmd+=(--nvram-seed "$KPROP_GATE_NVRAM")
fi
if [[ "${KPROP_GATE_REQUIRE_ARM64:-0}" == 1 ]]; then
	cmd+=(--require-arm64)
fi

exec "${cmd[@]}" "$@"
