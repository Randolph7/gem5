#!/usr/bin/env python3
"""Run selected SPEC CPU2017 rate workloads under several prefetch configs.

The script launches gem5 (se.py) three times per workload: without any
prefetcher, with the Bingo L2 prefetcher in single-table mode, and with the
multi-table mode enabled. After each campaign it parses stats.txt, emits a JSON
summary, and produces a per-workload bar chart that highlights the delta in
simTicks, IPC, and cache miss rates.

Example usage (Linux host):

    cd /users/Zecheng/ssd/uarch/gem5
    python3 workspace_speccpu/run_spec_prefetch_compare.py \
        --inst-count 10000 \
        --workloads mcf_r,leela_r

Use --skip-run if you only want to regenerate plots from existing stats.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
from pathlib import Path
from typing import Dict, Iterable, List

import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parents[1]
WS_ROOT = Path(__file__).resolve().parent
RESULTS_ROOT = WS_ROOT / "results"
DEFAULT_GEM5_BIN = REPO_ROOT / "build" / "X86" / "gem5.opt"
DEFAULT_SCRIPT = REPO_ROOT / "configs" / "deprecated" / "example" / "se.py"
DEFAULT_SPEC_ROOT = Path("/users/Zecheng/ssd/uarch/spec2017")

WORKLOAD_TEMPLATES = {
    "mcf_r": {
        "cmd_rel": "505.mcf_r/build/build_base_mytest-m64.0000/mcf_r",
        "options": {
            "kind": "path",
            "value": "505.mcf_r/run/run_base_refrate_mytest-m64.0000/inp.in",
        },
        "short": "mcf",
    },
    "deepsjeng_r": {
        "cmd_rel": "531.deepsjeng_r/build/build_base_mytest-m64.0000/deepsjeng_r",
        "options": {
            "kind": "path",
            "value": "531.deepsjeng_r/run/run_base_refrate_mytest-m64.0000/ref.txt",
        },
        "short": "deep",
    },
    "leela_r": {
        "cmd_rel": "541.leela_r/build/build_base_mytest-m64.0000/leela_r",
        "options": {
            "kind": "path",
            "value": "541.leela_r/run/run_base_refrate_mytest-m64.0000/ref.sgf",
        },
        "short": "leela",
    },
    "nab_r": {
        "cmd_rel": "544.nab_r/build/build_base_mytest-m64.0000/nab_r",
        "options": {
            "kind": "string",
            "value": "1am0 1122214447 122",
        },
        "short": "nab",
    },
}

MODES = {
    "no_prefetch": {
        "label": "No Prefetch",
        "args": [],  # se.py uses no hardware prefetcher when this list is empty
    },
    "bingo_single": {
        "label": "Bingo L2",
        "args": ["--l2-hwp-type=BingoPrefetcher"],
    },
    "bingo_multi": {
        "label": "Bingo L2 (multi)",
        "args": [
            "--l2-hwp-type=BingoPrefetcher",
            '--param=system.l2.prefetcher.multi_table_modes=["pc+addr","pc_offset"]',
        ],
    },
}

PLOT_METRICS = [
    ("simTicks", "Sim Ticks (lower better)"),
    ("ipc", "IPC (higher better)"),
    ("system.cpu.dcache.overallMisses::total", "L1D Misses"),
    ("system.l2.overallMisses::total", "L2 Misses"),
    ("coverage", "Prefetch Coverage"),
    ("accuracy", "Prefetch Accuracy"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gem5-bin",
        type=Path,
        default=DEFAULT_GEM5_BIN,
        help="Path to gem5.opt",
    )
    parser.add_argument(
        "--se-script",
        type=Path,
        default=DEFAULT_SCRIPT,
        help="Path to configs/deprecated/example/se.py",
    )
    parser.add_argument(
        "--spec-root",
        type=Path,
        default=DEFAULT_SPEC_ROOT,
        help="Root of the SPEC CPU2017 installation",
    )
    parser.add_argument(
        "--inst-count",
        type=int,
        default=100_000_000,
        help="Max instructions per run (-I)",
    )
    parser.add_argument(
        "--workloads",
        type=str,
        default="",
        help="Comma-separated subset of workloads (default: all templates)",
    )
    parser.add_argument(
        "--skip-run",
        action="store_true",
        help="Only parse existing stats and regenerate plots",
    )
    return parser.parse_args()


def setup_workloads(names: Iterable[str], spec_root: Path) -> List[Dict[str, str]]:
    if not names:
        names = WORKLOAD_TEMPLATES.keys()
    selected = []
    for name in names:
        name = name.strip()
        if not name:
            continue
        if name not in WORKLOAD_TEMPLATES:
            raise ValueError(f"Unknown workload '{name}'. Available: {sorted(WORKLOAD_TEMPLATES)}")
        template = WORKLOAD_TEMPLATES[name]
        cmd = spec_root / template["cmd_rel"]
        if not cmd.exists():
            raise FileNotFoundError(f"Missing binary for {name}: {cmd}")
        opt_cfg = template["options"]
        if opt_cfg["kind"] == "path":
            options = str((spec_root / opt_cfg["value"]).resolve())
        else:
            options = opt_cfg["value"]
        selected.append({
            "name": name,
            "cmd": str(cmd.resolve()),
            "options": options,
            "short": template["short"],
        })
    return selected


def run_gem5(gem5_bin: Path, script: Path, workload: Dict[str, str], mode: Dict[str, List[str]],
              outdir: Path, inst_count: int) -> None:
    cmd = [
        str(gem5_bin),
        "-d",
        str(outdir),
        str(script),
        "--num-cpus=1",
        "--cpu-type=TimingSimpleCPU",
        "--mem-size=4GB",
        "--caches",
        "--l2cache",
        f"--cmd={workload['cmd']}",
        f"--options={workload['options']}" if workload["options"] else "",
        "-I",
        str(inst_count),
    ] + mode["args"]
    outdir.parent.mkdir(parents=True, exist_ok=True)
    if outdir.exists():
        shutil.rmtree(outdir)
    print(f"\n[{workload['name']}] => {mode['label']}")
    subprocess.run([arg for arg in cmd if arg], check=True)


def parse_stats(path: Path) -> Dict[str, float]:
    stats: Dict[str, float] = {}
    if not path.exists():
        return stats
    with path.open() as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("-"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            key, value = parts[0], parts[1]
            try:
                stats[key] = float(value)
            except ValueError:
                continue
    return stats


def extract_metrics(stats: Dict[str, float]) -> Dict[str, float]:
    l1_hits = stats.get("system.cpu.dcache.overallHits::total", 0.0)
    l1_misses = stats.get("system.cpu.dcache.overallMisses::total", 0.0)
    l2_hits = stats.get("system.l2.overallHits::total", 0.0)
    l2_misses = stats.get("system.l2.overallMisses::total", 0.0)
    issued = stats.get("system.l2.prefetcher.issuedPrefetches", 0.0)
    useful = stats.get("system.l2.prefetcher.usefulPrefetches", 0.0)
    demand = stats.get("system.l2.demandMisses::total", l2_misses)
    cycles = stats.get("system.cpu.numCycles", math.nan)
    sim_insts = stats.get("simInsts", math.nan)
    ipc = math.nan
    if cycles and not math.isnan(cycles) and not math.isnan(sim_insts):
        ipc = sim_insts / cycles if cycles else math.nan
    # coverage = useful / demand if demand else math.nan
    coverage = stats.get("system.l2.prefetcher.coverage", math.nan)
    # accuracy = useful / issued if issued else math.nan
    accuracy = stats.get("system.l2.prefetcher.accuracy", math.nan)
    return {
        "simTicks": stats.get("simTicks", math.nan),
        "simSeconds": stats.get("simSeconds", math.nan),
        "ipc": ipc,
        "system.cpu.dcache.overallMisses::total": l1_misses,
        "system.l2.overallMisses::total": l2_misses,
        "coverage": coverage,
        "accuracy": accuracy,
    }


def format_metric(val: float) -> str:
    if math.isnan(val):
        return "nan"
    if abs(val) >= 1e4 or (abs(val) > 0 and abs(val) < 1e-3):
        return f"{val:.3e}"
    return f"{val:.4g}"


def plot_workload(workload: str, metrics: Dict[str, Dict[str, float]], outdir: Path) -> None:
    labels = [mode_cfg["label"] for mode_cfg in MODES.values()]
    figsize = (11, 10)
    fig, axes = plt.subplots(3, 2, figsize=figsize)
    axes = axes.flatten()

    for ax, (key, title) in zip(axes, PLOT_METRICS):
        series = [metrics.get(mode_name, {}).get(key, math.nan) for mode_name in MODES]
        ax.bar(labels, series, color=["#555555", "#1f77b4", "#d62728"], alpha=0.85)
        for idx, val in enumerate(series):
            y_val = val if not math.isnan(val) else 0.0
            ax.text(idx, y_val, format_metric(val),
                    ha="center", va="bottom" if (not math.isnan(val) and val >= 0) else "top",
                    fontsize=8)
        ax.set_title(title)
        ax.grid(axis="y", linestyle="--", alpha=0.4)

    fig.suptitle(f"Prefetch impact on {workload}")
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    outdir.mkdir(parents=True, exist_ok=True)
    image_path = outdir / f"{workload}_prefetch.png"
    fig.savefig(image_path, dpi=120)
    plt.close(fig)
    print(f"Saved plot to {image_path}")


def main() -> None:
    args = parse_args()
    requested = [w.strip() for w in args.workloads.split(",") if w.strip()]
    workloads = setup_workloads(requested, args.spec_root) if requested else setup_workloads(
        WORKLOAD_TEMPLATES.keys(), args.spec_root
    )

    RESULTS_ROOT.mkdir(exist_ok=True)
    summary = {}

    for workload in workloads:
        workload_results = {}
        for mode_name, mode_cfg in MODES.items():
            outdir = RESULTS_ROOT / workload["name"] / mode_name
            stats_path = outdir / "stats.txt"
            if not args.skip_run:
                run_gem5(args.gem5_bin, args.se_script, workload, mode_cfg, outdir, args.inst_count)
            stats = parse_stats(stats_path)
            if not stats:
                print(f"Warning: missing stats for {workload['name']} ({mode_name})")
                continue
            metrics = extract_metrics(stats)
            workload_results[mode_name] = metrics
        if workload_results:
            summary[workload["name"]] = workload_results
            plot_workload(workload["name"], workload_results, RESULTS_ROOT / workload["name"])
            summary_path = RESULTS_ROOT / workload["name"] / "summary.json"
            summary_path.parent.mkdir(parents=True, exist_ok=True)
            summary_path.write_text(json.dumps(workload_results, indent=2))
            print(f"Wrote summary to {summary_path}")

    (RESULTS_ROOT / "all_results.json").write_text(json.dumps(summary, indent=2))
    print(f"Aggregated summary saved to {RESULTS_ROOT / 'all_results.json'}")


if __name__ == "__main__":
    main()
