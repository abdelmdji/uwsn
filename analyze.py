#!/usr/bin/env python3
"""
Aggregates the raw per-run CSVs into means with 95% confidence intervals and
draws the six scenario figures.

    python3 analyze.py results/

Two things this deliberately does NOT do:

  * It does not invent a lifetime for censored runs. If a configuration never
    lost a node inside the evaluation window, its lifetime is reported as
    censored and the point is drawn hollow with an annotation, not plotted at
    an arbitrary ceiling. Use run_lifetime.sh to get real numbers.

  * It does not hide the spread. Every point carries a 95% CI computed from
    the seeds actually run. If the CIs of two protocols overlap, they are not
    distinguishable and the text must not claim otherwise.
"""

import sys
import os
import math
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PROTOCOLS = ["adaptive", "vbf", "hhvbf", "dbr"]
LABELS = {"adaptive": "Adaptive-IoUT-VBF", "vbf": "Classic-VBF",
          "hhvbf": "HHVBF", "dbr": "DBR"}
COLORS = {"adaptive": "#0173B2", "vbf": "#DE8F05",
          "hhvbf": "#029E73", "dbr": "#D55E00"}
MARKERS = {"adaptive": "o", "vbf": "s", "hhvbf": "^", "dbr": "D"}
STYLES = {"adaptive": "-", "vbf": "--", "hhvbf": "-.", "dbr": ":"}

SCENARIOS = [
    ("s1_density", "nNodes",         "Number of nodes"),
    ("s2_speed",   "maxSpeed",       "Node speed (m/s)"),
    ("s3_area",    "fieldSize",      "Field size (m)"),
    ("s4_radius",  "basePipeR",      "Base pipe radius (m)"),
    ("s5_traffic", "packetInterval", "Packet interval (s)"),
    ("s6_depth",   "waterDepth",     "Water depth (m)"),
]

METRICS = [
    ("throughput_kbps", "Throughput (kbps)",   "(a) Throughput"),
    ("pdr_pct",         "PDR (%)",             "(b) Packet delivery ratio"),
    ("mean_delay_s",    "End-to-end delay (s)", "(c) Delay"),
    ("mean_hops",       "Average hop count",   "(d) Average hop count"),
    ("energy_gap_J",    "Energy gap (J)",      "(e) Energy gap"),
    ("lifetime_s",      "Lifetime to 1st death (s)", "(f) Network lifetime"),
]


def ci95(series):
    """Mean and 95% CI half-width. Returns (mean, halfwidth, n)."""
    v = series.dropna().values
    n = len(v)
    if n == 0:
        return (np.nan, np.nan, 0)
    m = v.mean()
    if n < 2:
        return (m, np.nan, n)
    # t-critical for 95%, small-sample table then normal approximation
    tcrit = {2: 12.706, 3: 4.303, 4: 3.182, 5: 2.776, 6: 2.571, 7: 2.447,
             8: 2.365, 9: 2.306, 10: 2.262, 15: 2.145, 20: 2.093,
             30: 2.045}.get(n, 1.96)
    return (m, tcrit * v.std(ddof=1) / math.sqrt(n), n)


def aggregate(df, xcol, metric):
    """Returns per-protocol dict of (x, mean, ci, n, censored_fraction)."""
    out = {}
    for p in PROTOCOLS:
        sub = df[df.protocol == p]
        if sub.empty:
            continue
        xs, ms, cs, ns, cen = [], [], [], [], []
        for x, grp in sub.groupby(xcol):
            if metric == "lifetime_s":
                frac_cens = grp.lifetime_censored.mean()
                alive = grp[grp.lifetime_censored == 0].lifetime_s
                m, c, n = ci95(alive)
                cen.append(frac_cens)
            else:
                m, c, n = ci95(grp[metric])
                cen.append(0.0)
            xs.append(x); ms.append(m); cs.append(c); ns.append(n)
        order = np.argsort(xs)
        out[p] = dict(x=np.array(xs)[order], mean=np.array(ms)[order],
                      ci=np.array(cs)[order], n=np.array(ns)[order],
                      cens=np.array(cen)[order])
    return out


def plot_scenario(df, scen, xcol, xlabel, outdir):
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    any_censored = False
    for ax, (metric, ylabel, title) in zip(axes.ravel(), METRICS):
        agg = aggregate(df, xcol, metric)
        for p, d in agg.items():
            ok = ~np.isnan(d["mean"])
            ax.errorbar(d["x"][ok], d["mean"][ok], yerr=d["ci"][ok],
                        label=LABELS[p], color=COLORS[p], marker=MARKERS[p],
                        linestyle=STYLES[p], capsize=3, markersize=5, lw=1.4)
            if metric == "lifetime_s" and (d["cens"] > 0).any():
                any_censored = True
                for xv, cf in zip(d["x"], d["cens"]):
                    if cf > 0:
                        ax.annotate(f"{cf*100:.0f}% censored", (xv, ax.get_ylim()[1]),
                                    fontsize=6, rotation=90, va="top",
                                    color=COLORS[p], alpha=0.7)
        ax.set_xlabel(xlabel, fontsize=9)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=8)
    axes[0][0].legend(fontsize=8, frameon=False)
    note = "Error bars: 95% CI. "
    note += ("Some lifetime points are censored (no node died in the window) "
             "and are excluded from the mean." if any_censored
             else "No lifetime censoring in this scenario.")
    fig.suptitle(f"{scen}   |   {note}", fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(outdir, f"fig_{scen}.pdf")
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def write_summary(df, scen, xcol, outdir):
    rows = []
    for metric, ylabel, _ in METRICS:
        agg = aggregate(df, xcol, metric)
        for p, d in agg.items():
            for i, xv in enumerate(d["x"]):
                rows.append(dict(scenario=scen, parameter=xcol, value=xv,
                                 protocol=LABELS[p], metric=metric,
                                 mean=d["mean"][i], ci95=d["ci"][i],
                                 n_runs=int(d["n"][i]),
                                 censored_fraction=d["cens"][i]))
    out = pd.DataFrame(rows)
    path = os.path.join(outdir, f"summary_{scen}.csv")
    out.to_csv(path, index=False)
    return path


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    d = sys.argv[1]
    outdir = os.path.join(d, "analysis")
    os.makedirs(outdir, exist_ok=True)

    print(f"{'scenario':<14} {'runs':>6} {'seeds/point':>12}  figure")
    for scen, xcol, xlabel in SCENARIOS:
        path = os.path.join(d, f"{scen}.csv")
        if not os.path.exists(path):
            print(f"{scen:<14} {'--':>6}  missing ({path})")
            continue
        df = pd.read_csv(path)
        if df.empty:
            print(f"{scen:<14} {'0':>6}  empty")
            continue
        per_point = df.groupby(["protocol", xcol]).size()
        fig = plot_scenario(df, scen, xcol, xlabel, outdir)
        write_summary(df, scen, xcol, outdir)
        print(f"{scen:<14} {len(df):>6} {per_point.min():>6}-{per_point.max():<5} {fig}")

        if per_point.min() < 10:
            print(f"   WARNING: only {per_point.min()} seeds at some points. "
                  f"Confidence intervals will be wide and unreliable.")

    # Sanity check that would have caught the earlier problem
    lt = os.path.join(d, "lifetime.csv")
    if os.path.exists(lt):
        df = pd.read_csv(lt)
        cens = df.lifetime_censored.mean()
        print(f"\nlifetime sweep: {len(df)} runs, {cens*100:.1f}% still censored")

    print(f"\nOutputs in {outdir}/")


if __name__ == "__main__":
    main()
