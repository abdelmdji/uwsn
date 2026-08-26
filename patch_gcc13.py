import re, subprocess, sys, os
NS3 = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/uwsn/ns-3-dev")
JOBS = str(os.cpu_count() or 1)
PATTERN = re.compile(r"^(" + re.escape(NS3) + r"/[^:]+\.(?:h|cc)):\d+:\d+: error: ."
                     r"(?:u?int(?:8|16|32|64)_t|size_t|uintmax_t|intptr_t)\b", re.M)
patched = set()
for attempt in range(80):
    r = subprocess.run(f"ninja -C cmake-cache -j{JOBS}", shell=True, cwd=NS3,
                       capture_output=True, text=True)
    log = r.stdout + r.stderr
    open(os.path.join(NS3, "build.log"), "w").write(log)
    if r.returncode == 0:
        print(f"\n=== BUILD OK ({len(patched)} headers patched) ==="); sys.exit(0)
    hits = PATTERN.findall(log)
    if not hits:
        print("\n=== NON-CSTDINT ERROR - stopping ===")
        for l in [x for x in log.splitlines() if "error:" in x][:12]: print("  ", l)
        sys.exit(1)
    t = hits[0]
    if t in patched:
        print(f"still failing after patch: {t}"); sys.exit(1)
    s = open(t).read()
    if "#include <cstdint>" not in s:
        lines = s.split("\n")
        pos = next((i for i, l in enumerate(lines)
                    if l.startswith("#define ") or l.startswith("#include ")), 0)
        lines.insert(pos + 1, "#include <cstdint>")
        lines.insert(pos + 2, "#include <cstddef>")
        open(t, "w").write("\n".join(lines))
    patched.add(t)
    print(f"[{attempt}] patched {os.path.relpath(t, NS3)}", flush=True)
