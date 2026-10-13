#!/bin/bash
# Differential test: run scenario.txt against the ORIGINAL PowerPC library
# (emulated with Unicorn) and against libdsdual4, and require identical
# command/data traces.
#   ORIGINAL_LIB=/path/to/MFSBaseLib2891 DSD4_FIRMWARE=/path/to/DS_Dual4.BIN tests/run_diff.sh
set -eu
abspath() { [ -n "$1" ] && echo "$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"; }
LIB=$(abspath "${ORIGINAL_LIB:?set ORIGINAL_LIB to the extracted MFSBaseLib2891}")
FW=$(abspath "${DSD4_FIRMWARE:-}")
PY=${PYTHON:-python3}
cd "$(dirname "$0")/.."
"$PY" -c "import unicorn" 2>/dev/null || { echo "skipped: pip install unicorn"; exit 0; }
make -s build/libdsdual4.a
clang -O1 -ffp-contract=off -Iinclude -Isrc tests/diff/port_runner.c build/libdsdual4.a -framework IOKit -framework CoreFoundation -o build/port_runner
clang -shared -O1 -Isrc src/sim.c tests/diff/simlib.c -o build/libsimx.dylib
export DSD4_SIM_FW_PENDING=1 DSD4_TIME_SCALE=0.001
T=$(mktemp -d)
(cd tests/diff && ../../build/port_runner scenario.txt "$FW" > "$T/port.trace")
(cd tests/diff && "$PY" ppc_runner.py "$LIB" scenario.txt "$FW" > "$T/ppc.trace")
if diff -q "$T/port.trace" "$T/ppc.trace" >/dev/null; then
    echo "IDENTICAL: $(grep -c '^CMD' "$T/port.trace") scanner commands and $(grep -c '^IMG' "$T/port.trace") image reads match the original driver"
else
    diff "$T/port.trace" "$T/ppc.trace" | cut -c1-200 | head -40
    echo "MISMATCH (traces in $T)"
    exit 1
fi
