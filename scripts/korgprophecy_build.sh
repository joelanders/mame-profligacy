#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

JOBS="${JOBS:-4}"
REGENIE="${REGENIE:-0}"
SUBTARGET="${SUBTARGET:-propmin}"
OSD="${OSD:-sdl}"
USE_LIBSDL="${USE_LIBSDL:-1}"
NO_OPENGL="${NO_OPENGL:-1}"
NO_USE_MIDI="${NO_USE_MIDI:-1}"
NO_USE_PORTAUDIO="${NO_USE_PORTAUDIO:-1}"
USE_QTDEBUG="${USE_QTDEBUG:-0}"
SOURCES="src/mame/korg/korgprophecy.cpp"

if [[ "${1:-}" == "--regen" ]]; then
	REGENIE=1
	shift
fi

MAKE_ARGS=(
	"-j${JOBS}"
	"REGENIE=${REGENIE}"
	"PRECOMPILE=0"
	"SUBTARGET=${SUBTARGET}"
	"SOURCES=${SOURCES}"
	"OSD=${OSD}"
	"USE_QTDEBUG=${USE_QTDEBUG}"
	"IGNORE_GIT=1"
)

if [[ "${OSD}" != "windows" ]]; then
	MAKE_ARGS+=(
		"USE_LIBSDL=${USE_LIBSDL}"
		"NO_OPENGL=${NO_OPENGL}"
		"NO_USE_MIDI=${NO_USE_MIDI}"
		"NO_USE_PORTAUDIO=${NO_USE_PORTAUDIO}"
	)
fi

# Focused publication builds are warning-clean by default.  Set NOWERROR=1
# explicitly only when diagnosing an inherited toolchain warning.
if [[ -n "${NOWERROR:-}" ]]; then
	MAKE_ARGS+=("NOWERROR=${NOWERROR}")
fi

exec make "${MAKE_ARGS[@]}" "$@"
