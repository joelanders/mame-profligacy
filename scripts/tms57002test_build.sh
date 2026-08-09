#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

JOBS="${JOBS:-4}"
REGENIE="${REGENIE:-0}"
# Match the portable SDL2 configuration used by korgprophecy_build.sh.
# Windows CI overrides this with OSD=windows and USE_SDL3=1.
OSD="${OSD:-sdl}"
USE_LIBSDL="${USE_LIBSDL:-1}"
NO_OPENGL="${NO_OPENGL:-1}"
NO_USE_MIDI="${NO_USE_MIDI:-1}"
NO_USE_PORTAUDIO="${NO_USE_PORTAUDIO:-1}"

SOURCES="src/mame/skeleton/tms57002test.cpp,src/devices/cpu/tms57002/tms57002.cpp,src/devices/cpu/tms57002/tmsops.cpp,src/devices/cpu/tms57002/tms57kdec.cpp,src/devices/cpu/tms57002/57002dsm.cpp"

if [[ "${1:-}" == "--regen" ]]; then
	REGENIE=1
	shift
fi

MAKE_ARGS=(
	"-j${JOBS}"
	"REGENIE=${REGENIE}"
	"PRECOMPILE=0"
	"SUBTARGET=tms57test"
	"SOURCES=${SOURCES}"
	"OSD=${OSD}"
	"USE_QTDEBUG=0"
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
