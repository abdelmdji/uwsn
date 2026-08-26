#!/usr/bin/env python3
"""
Resumable, parallel sweep runner.

Free cloud sessions get killed (Colab ~90 min idle / 12 h max, Kaggle 12 h).
This runner is built for that: it enumerates every (scenario, protocol,
parameter, seed) job, skips the ones already present in the output CSVs, and
stops cleanly when its time budget runs out. Re-run it in the next session and
it picks up exactly where it left off.

    python3 run_resumable.py --seeds 20 --jobs 4 --budget 39600

    --seeds   seeds per configuration (20 gives usable confidence intervals)
    --jobs    parallel workers (use $(nproc))
    --budget  seconds to run before stopping cleanly (39600 = 11 h)
    --dry-run print the job count and exit
"""

import argparse
import csv
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from threading import Lock

SCENARIOS = {
    "s1_density": ("nNodes",         [10, 25, 50, 75, 100, 150, 200]),
    "s2_speed":   ("maxSpeed",       [1, 2, 3, 5, 7, 10]),
    "s3_area":    ("fieldSize",      [200, 300, 400, 500, 600, 800, 1000]),
    "s4_radius":  ("basePipeR",      [50, 75, 100, 125, 150, 200, 250]),
    "s5_traffic": ("packetInterval", [2, 4, 6, 8, 10, 12, 15]),
    "s6_depth":   ("waterDepth",     [100, 200, 300, 500, 700, 1000]),
}
PROTOCOLS = ["adaptive", "vbf", "hhvbf", "dbr"]

write_lock = Lock()


def done_jobs(path, param):
    """Set of (protocol, param_value, seed) already recorded in the CSV."""
    done = set()
    if not os.path.exists(path):
        return done
    try:
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                try:
                    done.add((row["protocol"], float(row[param]), int(row["seed"])))
                except (KeyError, ValueError):
                    continue
    except Exception as e:
        print(f"  warning: could not read {path}: {e}")
    return done


def build_jobs(outdir, seeds):
    jobs = []
    for scen, (param, values) in SCENARIOS.items():
        path = os.path.join(outdir, f"{scen}.csv")
        already = done_jobs(path, param)
        for v in values:
            for p in PROTOCOLS:
                for s in range(1, seeds + 1):
                    if (p, float(v), s) in already:
                        continue
                    jobs.append(dict(scen=scen, param=param, value=v,
                                     protocol=p, seed=s, out=path))
    return jobs


def run_one(job, ns3root, timeout):
    """Runs a single simulation. Returns (ok, message)."""
    extra = f"--{job['param']}={job['value']}"
    # Scenario 3 scales the box in all three dimensions.
    if job["scen"] == "s3_area":
        extra += f" --waterDepth={job['value']}"
    cmd = (f'./ns3 run --no-build "adaptive-iout-vbf-experiment '
           f'--protocol={job["protocol"]} --seed={job["seed"]} {extra} '
           f'--outFile={job["out"]} --label={job["scen"]}"')
    try:
        r = subprocess.run(cmd, shell=True, cwd=ns3root, timeout=timeout,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if r.returncode != 0:
            tail = r.stdout.decode(errors="replace")[-300:]
            return False, f"{job['scen']} {job['protocol']} v={job['value']} " \
                          f"seed={job['seed']}: rc={r.returncode} {tail}"
        return True, ""
    except subprocess.TimeoutExpired:
        return False, f"{job['scen']} {job['protocol']} v={job['value']} " \
                      f"seed={job['seed']}: TIMEOUT"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--jobs", type=int, default=max(1, os.cpu_count() or 1))
    ap.add_argument("--budget", type=int, default=39600,
                    help="seconds before stopping cleanly")
    ap.add_argument("--timeout", type=int, default=1800,
                    help="per-run timeout (s)")
    ap.add_argument("--outdir", default="results")
    ap.add_argument("--ns3root", default=".")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    os.makedirs(a.outdir, exist_ok=True)
    jobs = build_jobs(a.outdir, a.seeds)
    total_target = sum(len(v[1]) for v in SCENARIOS.values()) * len(PROTOCOLS) * a.seeds

    print(f"target runs : {total_target}")
    print(f"already done: {total_target - len(jobs)}")
    print(f"remaining   : {len(jobs)}")
    if a.dry_run or not jobs:
        if not jobs:
            print("\nNothing left to run. Next: python3 analyze.py results/")
        return

    print(f"workers     : {a.jobs}")
    print(f"budget      : {a.budget}s\n")

    start = time.time()
    completed = failed = 0
    stop = False

    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        futures = {}
        it = iter(jobs)
        # Prime the pool.
        for _ in range(a.jobs * 2):
            try:
                j = next(it)
            except StopIteration:
                break
            futures[pool.submit(run_one, j, a.ns3root, a.timeout)] = j

        while futures:
            for fut in as_completed(list(futures)):
                j = futures.pop(fut)
                ok, msg = fut.result()
                completed += 1
                if not ok:
                    failed += 1
                    if failed <= 10:
                        print(f"  FAIL {msg}")
                    elif failed == 11:
                        print("  (further failures suppressed)")

                elapsed = time.time() - start
                if completed % 25 == 0 or completed == len(jobs):
                    rate = completed / elapsed if elapsed else 0
                    left = len(jobs) - completed
                    eta = left / rate / 3600 if rate else float("inf")
                    print(f"  {completed}/{len(jobs)} done "
                          f"({rate*3600:.0f}/h, ~{eta:.1f}h left, "
                          f"{failed} failed)")

                if elapsed > a.budget:
                    stop = True

                if not stop:
                    try:
                        nj = next(it)
                        futures[pool.submit(run_one, nj, a.ns3root, a.timeout)] = nj
                    except StopIteration:
                        pass
                break  # re-evaluate as_completed over the updated set

    elapsed = time.time() - start
    print(f"\nran {completed} ({failed} failed) in {elapsed/60:.1f} min")
    if stop:
        print("Time budget reached. Re-run this script to continue "
              "-- completed runs are skipped automatically.")
    else:
        print("Sweep complete. Next: python3 analyze.py results/")

    if failed and failed == completed:
        print("\nEVERY run failed. Do not sweep further -- fix the build first.")
        sys.exit(1)


if __name__ == "__main__":
    main()
