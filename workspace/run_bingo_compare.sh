#!/usr/bin/env bash
set -euo pipefail

# Paths
ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
GEM5_BIN="$ROOT_DIR/build/X86/gem5.opt"
SCRIPT="$ROOT_DIR/configs/deprecated/example/se.py"
BENCH_SRC="$ROOT_DIR/workspace/bingo_stride.c"
BENCH_BIN="$ROOT_DIR/workspace/bingo_stride.x86"

# Build the benchmark for X86 Linux
build_bench() {
    echo "Building benchmark..."
    gcc -O2 -march=x86-64 -o "$BENCH_BIN" "$BENCH_SRC"
}

run_case() {
    local name="$1"; shift
    local outdir="$ROOT_DIR/workspace/out_${name}"
    rm -rf "$outdir"
    "$GEM5_BIN" -d "$outdir" "$SCRIPT" \
        --cpu-type=TimingSimpleCPU --caches --l2cache "$@" \
        --cmd="$BENCH_BIN"
}

summarize() {
    local name="$1"
    local outdir="$ROOT_DIR/workspace/out_${name}"
    echo "=== ${name} ==="
    if [[ ! -f "$outdir/stats.txt" ]]; then
        echo "stats.txt not found"
        return
    fi
    awk '
        /^simTicks/ {sim_ticks=$2}
        /^simSeconds/ {sim_seconds=$2}
        /system.cpu.dcache.overallHits::total/ {l1_hits=$2}
        /system.cpu.dcache.overallMisses::total/ {l1_misses=$2}
        /system.l2.overallHits::total/ {l2_hits=$2}
        /system.l2.overallMisses::total/ {l2_misses=$2}
        END {
            printf("sim_ticks: %s\n", sim_ticks);
            printf("sim_seconds: %s\n", sim_seconds);
            if (l1_hits + l1_misses > 0)
                printf("l1d_miss_rate: %.6f (%s/%s)\n",
                       l1_misses / (l1_hits + l1_misses),
                       l1_misses, l1_hits + l1_misses);
            if (l2_hits + l2_misses > 0)
                printf("l2_miss_rate: %.6f (%s/%s)\n",
                       l2_misses / (l2_hits + l2_misses),
                       l2_misses, l2_hits + l2_misses);
        }
    ' "$outdir/stats.txt"
    echo
}

main() {
    build_bench
    echo "Running without Bingo (baseline)..."
    run_case "no_bingo"
    echo "Running with Bingo on L2..."
    run_case "bingo_l2" --l2-hwp-type=BingoPrefetcher
    echo "Running with Bingo multi-table on L2..."
    run_case "bingo_l2_multi" \
        --l2-hwp-type=BingoPrefetcher \
        --param='system.l2.prefetcher.multi_table_modes=["pc+addr","pc_offset"]'
    summarize "no_bingo"
    summarize "bingo_l2"
    summarize "bingo_l2_multi"
    echo "Outputs in workspace/out_no_bingo, out_bingo_l2, out_bingo_l2_multi"
}

main "$@"
