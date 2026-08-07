#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

JOBS="${JOBS:-4}"
REGENIE="${REGENIE:-0}"
# Same OSD/audio flags as korgprophecy_build.sh: without an explicit OSD=sdl this tree's default
# resolves to the SDL3 OSD, which this machine has no headers for (SDL3/SDL.h not found).
OSD="${OSD:-sdl}"
USE_LIBSDL="${USE_LIBSDL:-1}"
NO_OPENGL="${NO_OPENGL:-1}"
NO_USE_MIDI="${NO_USE_MIDI:-1}"
NO_USE_PORTAUDIO="${NO_USE_PORTAUDIO:-1}"
NOWERROR="${NOWERROR:-1}"

SOURCES="src/mame/skeleton/tms57002test.cpp,src/devices/cpu/tms57002/tms57002.cpp,src/devices/cpu/tms57002/tmsops.cpp,src/devices/cpu/tms57002/tms57kdec.cpp,src/devices/cpu/tms57002/57002dsm.cpp"

if [[ "${1:-}" == "--regen" ]]; then
	REGENIE=1
	shift
fi

exec make \
	"-j${JOBS}" \
	"REGENIE=${REGENIE}" \
	"PRECOMPILE=0" \
	"SUBTARGET=tms57test" \
	"SOURCES=${SOURCES}" \
	"OSD=${OSD}" \
	"USE_LIBSDL=${USE_LIBSDL}" \
	"NO_OPENGL=${NO_OPENGL}" \
	"NO_USE_MIDI=${NO_USE_MIDI}" \
	"NO_USE_PORTAUDIO=${NO_USE_PORTAUDIO}" \
	"USE_QTDEBUG=0" \
	"NOWERROR=${NOWERROR}" \
	"IGNORE_GIT=1" \
	"$@"
