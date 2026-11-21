#!/usr/bin/env bash
# Host-Test des SD-Loggers bauen und ausfuehren.
#   tools/sd_sim/run.sh
#   SD_SIM_VERBOSE=1 tools/sd_sim/run.sh   # mit Firmware-Log
#   SD_SIM_KEEP=1 tools/sd_sim/run.sh      # Kartenordner des Normalbetriebs behalten
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/maggie_sd_sim"
g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter \
    -I "$HERE" -I "$ROOT/include" \
    "$HERE/sd_sim.cpp" "$ROOT/src/storage/sd_logger.cpp" "$ROOT/src/statemachine/state_machine.cpp" \
    -o "$OUT"
"$OUT"
