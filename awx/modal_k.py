#!/usr/bin/env python3
"""Modal app: fan the K-ladder chain out, ONE TASK PER (arm, K).

    modal run awx/modal_k.py --arms awx/arms.example.json

Why: an arm is ~5-22 min of one core, and the ladder wants every idea on
K35/K41/K51 at least. Run four locally and they contend -- a K51 that
takes 9 min alone takes 22 alongside three siblings. One container each
means the wall clock is the SLOWEST SINGLE ARM, whatever the sweep's
width.

THE STACK IS PINNED TO THE LOCAL ONE on purpose (see PINS). This repo has
measured that a python/numpy change moves routed copper (math.fsum), so an
image on a different stack would be a new baseline era and every recorded
K number (K35 58, K41 80, K51 137) would stop meaning anything here. The
`baseline` arms in arms.example.json re-measure those three in the cloud;
until they reproduce, compare cloud ONLY to cloud.

The working tree is shipped, uncommitted and all -- that is the point,
since the whole campaign lives in the tree.
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import time
from pathlib import Path

import modal

REPO = "/opt/krt"
_src = Path(__file__).resolve().parents[1]

# The LOCAL stack, exactly (python3.14 is not a Modal base yet; 3.13 is the
# nearest, and the baseline arms are what prove whether that matters).
PINS = ("numpy==2.3.3", "scipy==1.16.2", "shapely==2.1.2", "ortools==9.15.6755")

image = (
    modal.Image.debian_slim(python_version="3.13")
    .apt_install("curl", "procps", "build-essential")
    .pip_install(*PINS)
    # the source-build fallback for grid_router; before add_local_dir so a
    # source edit does not repay the rustup download
    .run_commands("curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal")
    .add_local_dir(str(_src), REPO, copy=True, ignore=[
        "**/.git/**", "**/__pycache__/**", "**/target/**",
        "**/.claude/worktrees/**", "**/tmp/**",
        # the local .so is macOS arm64 and would shadow the linux build
        "**/*.so", "**/*.dylib",
    ])
    .run_commands(
        f"cd {REPO} && (python3 build_router.py"
        f" || (. $HOME/.cargo/env && python3 build_router.py --from-source))",
        # prove it imports the way obstacle_map will, or fail the BUILD
        f"cd {REPO}/py_router && python3 -c \"import obstacle_map, grid_router;"
        f" print('grid_router', grid_router.__version__)\"",
        f"cd {REPO}/awx && python3 -c \"import braid, fanout_from_plan;"
        f" print('awx imports ok')\"",
    )
)

app = modal.App("bus622-kladder", image=image)

# the chain environment every arm shares (the item-5 joint solve)
BASE_ENV = {
    "SRC_ROUNDS": "0", "SEL_RETRY": "6", "EXACT_LANE": "1", "DST_FACE_ASK": "1",
    "DST_WALK": "3", "SF_SWIM": "30", "BRAID_EXIT_GUARD": "1",
    "BRAID_SWIM_HOLD": "1", "SEL_XING": "2", "SF_EQUIV": "2",
    "SF_JUDGE": "braid", "BRAID_ONE_DIVE": "5", "DST_RESIDUE": "3",
    "DST_RESIDUE_POOL": "displaced", "DST_RESIDUE_CANDS": "4",
    "BRAID_ALT_SOLVER": "cpsat", "BRAID_CPSAT_DET": "40",
    # one BLAS thread, as chain_k.sh does: a pool sized from the machine
    # makes a reduction's summation order machine-dependent
    "OMP_NUM_THREADS": "1", "VECLIB_MAXIMUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
# Nothing here raises a time budget any more: there are none. Every loop
# is capped in JUDGE CALLS and every solve in nodes or deterministic
# time, so a container answers exactly what the laptop answers, however
# much slower it is. That is what made the first cloud sweep unreadable
# -- two identical runs of the K35 baseline came back 72 vias / 1436
# segs and 58 / 1840.

KEEP = re.compile(
    r"GRADE |REFUSED |refusal reasons|pattern seed|seat repair|not asked again|"
    r"destination pass \d+: planner judge|destination re-plan|berth audit|"
    r"launch order:|target order:|re-lay rungs|dp: |wrote ")


# memory: a cap-8 choice instance is 505k rows and peaked at 1.29 GB
# SINGLE-threaded; CP-SAT runs 4 workers, and one cap-8 arm died on
# SIGABRT with no output in a 4 GB container.
@app.function(cpu=4, memory=12288, timeout=7200, max_containers=64)
def run_arm(arm: dict) -> dict:
    """One (tag, K) chain, graded, with the lines worth reading back."""
    tag, K = arm["tag"], int(arm["K"])
    env = dict(os.environ)
    env.update(BASE_ENV)
    env.update({k: str(v) for k, v in (arm.get("env") or {}).items()})
    wd = f"{REPO}/awx"
    t0 = time.time()
    p = subprocess.run(["bash", "chain_k.sh", tag, str(K)], cwd=wd, env=env,
                       capture_output=True, text=True, errors="replace")
    secs = round(time.time() - t0)
    out = p.stdout + p.stderr
    logs = {}
    for suffix in ("_fo_k%d.log" % K, "_k%d.log" % K):
        f = Path(wd) / "tmp" / (tag + suffix)
        if f.exists():
            txt = f.read_text(errors="replace").splitlines()
            logs[suffix] = [ln for ln in txt if KEEP.search(ln)][-400:]
    grade = next((ln for ln in reversed(out.splitlines()) if "GRADE" in ln), "")
    return {"tag": tag, "K": K, "secs": secs, "rc": p.returncode,
            "grade": grade.strip(), "env": arm.get("env") or {},
            "chain_out": out.splitlines()[-60:], "logs": logs}


@app.local_entrypoint()
def main(arms: str = "awx/arms.example.json", out: str = ""):
    spec = json.loads(Path(arms).read_text())
    jobs = [{"tag": a["tag"], "K": k, "env": a.get("env") or {}}
            for a in spec for k in a["K"]]
    print(f"{len(jobs)} task(s): " + ", ".join(f'{j["tag"]}/K{j["K"]}' for j in jobs))
    t0 = time.time()
    res = list(run_arm.map(jobs))
    res.sort(key=lambda r: (r["tag"], r["K"]))
    print(f"\n=== {len(res)} arm(s) in {round(time.time()-t0)}s wall clock\n")
    for r in res:
        e = " ".join(f"{k}={v}" for k, v in r["env"].items()) or "(baseline)"
        print(f'{r["tag"]:10s} K{r["K"]:<3d} {r["secs"]:5d}s  {r["grade"] or "NO GRADE rc=%d" % r["rc"]}')
        print(f'{"":10s}      [{e}]')
    if out:
        Path(out).write_text(json.dumps(res, indent=1))
        print(f"\nfull logs -> {out}")
