#!/usr/bin/env python3
import re, subprocess, sys, os
NS3 = "/home/claude/ns3"
PATTERN = re.compile(
    r"^(" + re.escape(NS3) + r"/[^:]+\.(?:h|cc)):\d+:\d+: error: ."
    r"(?:u?int(?:8|16|32|64)_t|size_t|uintmax_t|intptr_t)\b", re.M)
patched = set()
for attempt in range(60):
    r = subprocess.run("ninja -C cmake-cache -j1", shell=True, cwd=NS3,
                       capture_output=True, text=True)
    log = r.stdout + r.stderr
    open("/home/claude/ninja.log", "w").write(log)
    if r.returncode == 0:
        print(f"BUILD OK after {len(patched)} header patches"); sys.exit(0)
    hits = PATTERN.findall(log)
    if not hits:
        print("NON-CSTDINT ERRORS -- stopping:")
        for line in [l for l in log.splitlines() if "error:" in l][:12]:
            print("   ", line)
        sys.exit(1)
    target = hits[0]
    if target in patched:
        print(f"already patched but still failing: {target}")
        for line in [l for l in log.splitlines() if "error:" in l][:8]:
            print("   ", line)
        sys.exit(1)
    src = open(target).read()
    if "#include <cstdint>" not in src:
        lines = src.split("\n")
        pos = 0
        for i, line in enumerate(lines):
            if line.startswith("#define ") or line.startswith("#include "):
                pos = i; break
        lines.insert(pos + 1, "#include <cstdint>")
        lines.insert(pos + 2, "#include <cstddef>")
        open(target, "w").write("\n".join(lines))
    patched.add(target)
    print(f"[{attempt}] patched {os.path.relpath(target, NS3)}", flush=True)
print("gave up"); sys.exit(1)
