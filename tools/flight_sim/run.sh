#!/usr/bin/env bash
# Host-Simulation der Flug-Trajektorie bauen und ausfuehren.
#   tools/flight_sim/run.sh                  # alle Szenarien
#   FLIGHT_SIM_VERBOSE=1 tools/flight_sim/run.sh   # mit Firmware-Log
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/maggie_flight_sim"
g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter \
    -I "$HERE" -I "$ROOT/include" \
    "$HERE/flight_sim.cpp" "$ROOT/src/arm_flight_player.cpp" "$ROOT/src/hal/arm_hal.cpp" \
    -o "$OUT"
"$OUT"
