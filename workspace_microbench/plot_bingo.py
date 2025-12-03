#!/usr/bin/env python3
"""
Visualize Bingo prefetcher impact using stats from workspace_microbench/out_no_bingo and
workspace_microbench/out_bingo_l2. Generates a PNG with key metrics and prints a summary.
"""
import math
import os
from pathlib import Path
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
BASE = ROOT / "out_no_bingo" / "stats.txt"
HWP = ROOT / "out_bingo_l2" / "stats.txt"
HWP_MULTI = ROOT / "out_bingo_l2_multi" / "stats.txt"


def load_stats(path: Path) -> dict:
    stats = {}
    if not path.exists():
        return stats
    with path.open() as fh:
        for line in fh:
            if line.startswith("-") or line.strip() == "" or line.startswith("----------"):
                continue
            parts = line.strip().split()
            if len(parts) < 2:
                continue
            key, val = parts[0], parts[1]
            try:
                stats[key] = float(val)
            except ValueError:
                continue
    return stats


def ratio(num: float, den: float) -> float:
    return num / den if den else math.nan


def extract(stats: dict) -> dict:
    l1_hits = stats.get("system.cpu.dcache.overallHits::total", 0.0)
    l1_misses = stats.get("system.cpu.dcache.overallMisses::total", 0.0)
    l2_hits = stats.get("system.l2.overallHits::total", 0.0)
    l2_misses = stats.get("system.l2.overallMisses::total", 0.0)
    demand_misses_l2 = stats.get("system.l2.demandMisses::total", l2_misses)
    issued = stats.get("system.l2.prefetcher.issuedPrefetches",
                       stats.get("system.l2.prefetcher.numPrefetchesIssued", 0.0))
    useful = stats.get("system.l2.prefetcher.usefulPrefetches",
                       stats.get("system.l2.prefetcher.numUsefulPrefetches", 0.0))
    cov_key = stats.get("system.l2.prefetcher.coverage", None)
    acc_key = stats.get("system.l2.prefetcher.accuracy", None)

    sim_ticks = stats.get("simTicks", math.nan)
    sim_seconds = stats.get("simSeconds", math.nan)
    sim_insts = stats.get("simInsts", math.nan)
    num_cycles = stats.get("system.cpu.numCycles", math.nan)
    ipc = ratio(sim_insts, num_cycles) if not math.isnan(num_cycles) else math.nan

    # coverage = cov_key if cov_key is not None else ratio(useful, demand_misses_l2)
    coverage = stats.get("system.l2.prefetcher.coverage", math.nan)
    accuracy = acc_key if acc_key is not None else ratio(useful, issued)
    l1_mr = ratio(l1_misses, l1_hits + l1_misses)
    l2_mr = ratio(l2_misses, l2_hits + l2_misses)

    return {
        "sim_ticks": sim_ticks,
        "sim_seconds": sim_seconds,
        "ipc": ipc,
        "l1_miss_rate": l1_mr,
        "l2_miss_rate": l2_mr,
        "coverage": coverage,
        "accuracy": accuracy,
        "prefetch_issued": issued,
        "prefetch_useful": useful,
    }


def plot_metrics(base: dict, hwp: dict, hwp_multi: dict, out_path: Path):
    metrics = [
        ("sim_ticks", "Sim Ticks (lower better)"),
        ("ipc", "IPC (higher better)"),
        ("l1_miss_rate", "L1D Miss Rate"),
        ("l2_miss_rate", "L2 Miss Rate"),
        ("coverage", "Prefetch Coverage (useful/demand_misses)"),
        ("accuracy", "Prefetch Accuracy (useful/issued)"),
    ]

    fig, axes = plt.subplots(3, 2, figsize=(11, 10))
    axes = axes.flatten()
    width = 0.25

    for ax, (key, title) in zip(axes, metrics):
        bval = base.get(key, math.nan)
        hval = hwp.get(key, math.nan)
        mval = hwp_multi.get(key, math.nan)
        xs = [-width, 0, width]
        ax.bar(xs[0], bval, width, label="No Bingo")
        ax.bar(xs[1], hval, width, label="L2 Bingo")
        ax.bar(xs[2], mval, width, label="L2 Bingo (multi)")
        ax.set_title(title)
        ax.set_xticks([])
        ax.grid(axis="y", linestyle="--", alpha=0.5)
        ax.legend()
        # annotate values
        for x, val in zip(xs, [bval, hval, mval]):
            if math.isnan(val):
                txt = "nan"
            elif abs(val) >= 1e4 or abs(val) < 1e-3:
                txt = f"{val:.3e}"
            else:
                txt = f"{val:.3g}"
            ax.text(x, val, txt, ha="center",
                    va="bottom" if val >= 0 else "top",
                    fontsize=8, rotation=0)

    fig.suptitle("Bingo Prefetcher Effect (L2)")
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig.savefig(out_path, dpi=120)


def main():
    base_stats = load_stats(BASE)
    hwp_stats = load_stats(HWP)
    hwp_multi_stats = load_stats(HWP_MULTI)
    if not base_stats or not hwp_stats or not hwp_multi_stats:
        print("Missing stats. Run workspace_microbench/run_bingo_compare.sh first.")
        return

    base = extract(base_stats)
    hwp = extract(hwp_stats)
    hwp_multi = extract(hwp_multi_stats)

    out_img = ROOT / "bingo_compare.png"
    plot_metrics(base, hwp, hwp_multi, out_img)

    def fmt(val):
        return "nan" if math.isnan(val) else f"{val:.6g}"

    print("=== Summary ===")
    for key in ["sim_ticks", "sim_seconds", "ipc",
                "l1_miss_rate", "l2_miss_rate",
                "coverage", "accuracy",
                "prefetch_issued", "prefetch_useful"]:
        print(f"{key:16} no_bingo={fmt(base.get(key, math.nan))}  "
              f"bingo={fmt(hwp.get(key, math.nan))}  "
              f"bingo_multi={fmt(hwp_multi.get(key, math.nan))}")
    print(f"\nPlot saved to {out_img}")


if __name__ == "__main__":
    main()
