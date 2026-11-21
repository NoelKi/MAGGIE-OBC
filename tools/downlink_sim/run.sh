#!/usr/bin/env bash
# Host-Test der Downlink-Stufen bauen und ausfuehren - fuer die aktuelle
# Baudrate (38400) und zur Probe fuer 115200 (finales Experiment).
#   tools/downlink_sim/run.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/maggie_downlink_sim"
rc=0
for BAUD in 38400 115200; do
    g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -DMAGGIE_DOWNLINK_BAUDRATE=$BAUD \
        -I "$HERE" -I "$ROOT/include" \
        "$HERE/downlink_sim.cpp" "$ROOT/src/hal/telemetry_hal.cpp" "$ROOT/src/hal/arm_hal.cpp" \
        -o "$OUT"
    "$OUT" || rc=1
    echo
done
exit $rc
