#!/usr/bin/env bash
# Host-Test des Kameratreibers (CameraHAL) bauen und ausfuehren.
#   tools/camera_sim/run.sh
#   CAM_SIM_VERBOSE=1 tools/camera_sim/run.sh   # mit Firmware-Log
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/maggie_camera_sim"
g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter \
    -I "$HERE" -I "$ROOT/include" \
    "$HERE/camera_sim.cpp" "$ROOT/src/hal/camera_hal.cpp" \
    -o "$OUT"
"$OUT"
