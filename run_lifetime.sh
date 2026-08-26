#!/usr/bin/env bash
# Separate sweep that runs to actual first node death, removing the
# right-censoring that limits the 200 s runs.
set -u
SEEDS=${1:-20}
OUT=${2:-results}
mkdir -p "$OUT"
for p in adaptive vbf hhvbf dbr; do
  for i in 2 4 6 8 10 12 15; do
    for s in $(seq 1 "$SEEDS"); do
      ./ns3 run "adaptive-iout-vbf-experiment --protocol=$p --seed=$s \
        --runToDeath=true --packetInterval=$i \
        --outFile=$OUT/lifetime.csv --label=lifetime" >/dev/null 2>&1
    done
  done
done
echo "Lifetime sweep complete."
