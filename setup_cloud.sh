#!/usr/bin/env bash
#
# One-command setup for ns-3.36 + AquaSim-NG + Adaptive-IoUT-VBF.
# Works on Google Colab, Kaggle, GitHub Codespaces, Gitpod, or any Ubuntu box.
#
#   bash setup_cloud.sh [WORKDIR]
#
# Idempotent: safe to re-run. Skips steps already done, so if a session dies
# mid-build you just run it again.

set -euo pipefail
WORK=${1:-$HOME/uwsn}
NS3="$WORK/ns-3-dev"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== workdir: $WORK"
mkdir -p "$WORK"

# ---------------------------------------------------------------- deps ----
if ! command -v cmake >/dev/null 2>&1; then
  echo "=== installing build dependencies"
  (sudo apt-get update -qq || apt-get update -qq) >/dev/null
  (sudo apt-get install -y -qq g++ python3 python3-dev cmake ninja-build git \
      pkg-config libsqlite3-dev ccache \
   || apt-get install -y -qq g++ python3 python3-dev cmake ninja-build git \
      pkg-config libsqlite3-dev ccache) >/dev/null
fi
python3 -m pip install -q pandas numpy matplotlib 2>/dev/null || true

# ---------------------------------------------------------------- ns-3 ----
if [ ! -d "$NS3" ]; then
  echo "=== cloning ns-3.36 (shallow)"
  git clone -q --depth 1 --branch ns-3.36 \
      https://gitlab.com/nsnam/ns-3-dev.git "$NS3" \
    || git clone -q --depth 1 --branch ns-3.36 \
      https://github.com/nsnam/ns-3-dev-git.git "$NS3"
fi

# ------------------------------------------------------------ aqua-sim ----
if [ ! -d "$NS3/contrib/aqua-sim-ng" ]; then
  echo "=== cloning AquaSim-NG"
  git clone -q --depth 1 \
      https://github.com/rmartin5/aqua-sim-ng.git "$NS3/contrib/aqua-sim-ng"
fi

# ------------------------------------------------------------- module -----
echo "=== installing Adaptive-IoUT-VBF"
cp "$SRC/aqua-sim-routing-adaptive-vbf.h"  "$NS3/contrib/aqua-sim-ng/model/"
cp "$SRC/aqua-sim-routing-adaptive-vbf.cc" "$NS3/contrib/aqua-sim-ng/model/"
cp "$SRC/adaptive-iout-vbf-experiment.cc"  "$NS3/scratch/"
cp "$SRC/analyze.py" "$SRC/run_resumable.py" "$NS3/" 2>/dev/null || true

python3 - "$NS3/contrib/aqua-sim-ng/CMakeLists.txt" <<'PY'
import sys, re
path = sys.argv[1]
s = open(path).read()
changed = False
for anchor, addition in [
    ("model/aqua-sim-routing-vbf.cc", "model/aqua-sim-routing-adaptive-vbf.cc"),
    ("model/aqua-sim-routing-vbf.h",  "model/aqua-sim-routing-adaptive-vbf.h"),
]:
    if addition in s:
        continue
    if anchor not in s:
        sys.exit(f"ERROR: anchor {anchor} not found in CMakeLists.txt")
    s = s.replace(anchor, anchor + "\n        " + addition, 1)
    changed = True
if changed:
    open(path, "w").write(s)
    print("    CMakeLists.txt patched")
else:
    print("    CMakeLists.txt already patched")
PY

# -------------------------------------------------------------- build -----
cd "$NS3"
if [ ! -f "$NS3/.lock-ns3_linux_build" ] || [ ! -d "$NS3/build" ]; then
  echo "=== configuring (aqua-sim-ng only, optimized)"
  ./ns3 configure --enable-modules=aqua-sim-ng \
                  --build-profile=optimized \
                  --disable-examples --disable-tests >/dev/null
fi
echo "=== building (this is the slow part: 15-40 min)"
./ns3 build 2>&1 | tail -5

# ------------------------------------------------------------- verify -----
echo
echo "=== smoke test"
cd "$NS3"
./ns3 run "adaptive-iout-vbf-experiment --protocol=adaptive --nNodes=50 --seed=1 --outFile=/tmp/smoke.csv" \
  || { echo "SMOKE TEST FAILED"; exit 1; }

echo
echo "=== setup complete."
echo "    ns-3 root: $NS3"
echo "    next:  cd $NS3 && python3 run_resumable.py --seeds 20 --jobs \$(nproc)"
