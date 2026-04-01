#!/bin/bash
#
# MAVLink forwarding mutex benchmark runner.
#
# Builds the benchmark tool, starts PX4 SITL SIH, runs the benchmark,
# then collects perf counter data.
#
# Usage:
#   ./run_forwarding_bench.sh [--duration 60] [--rate 200] [--connections 2] [--label baseline]
#
# The --label flag tags the output files for before/after comparison.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$SCRIPT_DIR/build_bench"
RESULTS_DIR="$PX4_DIR/bench_results"

DURATION=60
RATE=200
CONNECTIONS=2
LABEL="run"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2 ;;
        --rate) RATE="$2"; shift 2 ;;
        --connections) CONNECTIONS="$2"; shift 2 ;;
        --label) LABEL="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 [--duration N] [--rate N] [--connections N] [--label NAME]"
            exit 0
            ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULT_PREFIX="${RESULTS_DIR}/${LABEL}_${TIMESTAMP}"

mkdir -p "$RESULTS_DIR"

echo "=== Step 1: Build PX4 SITL SIH ==="
cd "$PX4_DIR"
make px4_sitl_sih 2>&1 | tail -5

echo ""
echo "=== Step 2: Build benchmark tool ==="
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
# Symlink the standalone CMakeLists and source into build dir
ln -sf "$SCRIPT_DIR/CMakeLists_bench.txt" CMakeLists.txt
ln -sf "$SCRIPT_DIR/test_mavlink_forwarding_bench.cpp" .
cmake -DCMAKE_BUILD_TYPE=Release .
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)

echo ""
echo "=== Step 3: Start PX4 SITL SIH ==="
cd "$PX4_DIR"

# Start PX4 in background, capture PID
PX4_SIM_MODEL=sihsim_quadx $PX4_DIR/build/px4_sitl_sih/bin/px4 \
    -s $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/rcS \
    -w $PX4_DIR/build/px4_sitl_sih/rootfs \
    $PX4_DIR/build/px4_sitl_sih/etc \
    > "${RESULT_PREFIX}_px4.log" 2>&1 &
PX4_PID=$!

echo "PX4 started (PID=$PX4_PID), waiting 10s for boot..."
sleep 10

# Verify PX4 is running
if ! kill -0 "$PX4_PID" 2>/dev/null; then
    echo "ERROR: PX4 failed to start. Check ${RESULT_PREFIX}_px4.log"
    exit 1
fi

echo ""
echo "=== Step 4: Run benchmark ==="
echo "  Duration:    ${DURATION}s"
echo "  Rate:        ${RATE} Hz/connection"
echo "  Connections: ${CONNECTIONS}"
echo ""

"$BUILD_DIR/mavlink_forwarding_bench" \
    --duration "$DURATION" \
    --rate "$RATE" \
    --connections "$CONNECTIONS" \
    --report "${RESULT_PREFIX}_sender.csv" \
    2>&1 | tee "${RESULT_PREFIX}_bench.log"

echo ""
echo "=== Step 5: Collect perf counters ==="

# Send 'perf' command to PX4 via mavlink shell (or direct if available)
sleep 2

echo ""
echo "=== Step 6: Stop PX4 and extract perf data ==="
kill -INT "$PX4_PID" 2>/dev/null || true
sleep 3

# PX4 dumps perf counters on exit in some configurations.
# Extract perf lines from the log.
if grep -q "mavlink: fwd_msg" "${RESULT_PREFIX}_px4.log"; then
    echo "Perf counters from PX4 log:"
    grep -E "mavlink: (fwd_|comp_seen|unsigned_cb)" "${RESULT_PREFIX}_px4.log" | tee "${RESULT_PREFIX}_perf.txt"
else
    echo "NOTE: Perf counters not found in PX4 log."
    echo "Run 'perf' in the PX4 console before stopping to capture data."
    echo ""
    echo "Manual workflow:"
    echo "  1. Terminal 1: PX4_SIM_MODEL=sihsim_quadx make px4_sitl_sih"
    echo "  2. Terminal 2: $BUILD_DIR/mavlink_forwarding_bench --duration $DURATION --rate $RATE --connections $CONNECTIONS"
    echo "  3. Terminal 1 (PX4 console): perf"
    echo "  4. Copy the perf output to ${RESULT_PREFIX}_perf.txt"
fi

echo ""
echo "=== Results saved ==="
echo "  PX4 log:      ${RESULT_PREFIX}_px4.log"
echo "  Bench log:    ${RESULT_PREFIX}_bench.log"
echo "  Sender CSV:   ${RESULT_PREFIX}_sender.csv"
echo "  Perf data:    ${RESULT_PREFIX}_perf.txt (if available)"
echo ""
echo "To compare runs:"
echo "  diff <baseline>_perf.txt <after_fix>_perf.txt"
