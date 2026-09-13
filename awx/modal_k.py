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
    # Error/Traceback FIRST: a cloud arm that dies returns "NO GRADE" and
    # chain_k.sh prints only the File line, so without these the actual
    # exception never leaves the container and the run is undiagnosable.
    r"Error|Traceback|Exception|line \d+, in |"
    r"GRADE |REFUSED |refusal reasons|pattern seed|seat repair|not asked again|"
    # the instance size, so a sweep can say what a model change did to the
    # model and not only to the answer
    r"profiles5 alts|"
    # the ASK: "plan (destination pass 0): 35 berth(s) to lay down:14,..." --
    # a ')' sits between the number and the text, so the
    # "destination pass \d+: planner judge" alternative below never matched
    # it. Without the ask, a sweep cannot tell an upstream (berth CHOICE)
    # divergence from a downstream (fanout LAY) one.
    r"plan \(destination pass|residue choice: \d+ residue|"
    r"destination pass \d+: planner judge|destination re-plan|berth audit|"
    r"launch order:|target order:|re-lay rungs|dp: |wrote ")


# memory: REQUEST 1 GB, LIMIT 3 GB (2026-09-12, cut 4x on request).
# The request is the bill; it sits just above the 640 MB an ordinary arm
# actually peaks at. The limit still covers a ~1.3 GB cap-8 build with
# room over. WHAT THIS PUTS AT RISK, stated so it is not a surprise: an
# arm needing more than 3 GB is KILLED, not throttled. The only member
# measured near that is the WHOLE-MENU arm (DST_RESIDUE_CANDS=99) at K51,
# which already hung a container on a 900 s heartbeat timeout and has been
# dropped from the sweeps. A killed arm surfaces as NO GRADE and costs
# only itself -- per-arm writes plus --resume mean a re-run at higher
# memory picks up exactly where it stopped.
# The history below is why the old number was what it was.
# PREVIOUSLY: REQUEST 4 GB, LIMIT 12 GB. The flat 12 GB was a
# guess standing in for a diagnosis -- a cap-8 arm died on SIGABRT with no
# output in a 4 GB container and nothing ever confirmed that as an OOM.
# Measured since, in situ: the production K35 joint arm peaks at 640 MB
# for the WHOLE fanout stage, and the climb is one `_alts5` call (303 ->
# 640 MB across five CP-SAT solves of the same model, because CP-SAT never
# returns its arena; HiGHS on the same harness is flat). A cap-8 build is
# ~1.3 GB single-threaded. So the request is the bill and the limit is the
# safety net: an arm that really needs more still gets it, and 27 arms stop
# reserving 324 GB for a workload whose worst measured member is ~2.5 GB.
# cpu: REQUEST 0.125 core, LIMIT 4 (2026-09-12, on request). 0.1 was asked
# for and REFUSED by Modal -- "Function CPU request out of bounds. Must be
# between 0.125 and 64 cores" -- so 0.125 is the floor, not a choice.
# Modal bills the
# GREATER of the request and actual usage, so the request is a floor on the
# bill, not a cap on it -- an arm that really burns 4 cores is still billed
# for 4. What the low request buys is SCHEDULING: containers pack far more
# densely, so a wide sweep starts sooner. The limit stays 4 because CP-SAT
# runs num_workers=4 and a hard 0.1-core cap would stretch a 600 s arm past
# the 7200 s timeout.
# DETERMINISM IS NOT AT RISK: CP-SAT is bounded by max_deterministic_time
# with num_workers pinned, so a slower container returns the SAME answer,
# just later. The one thing speed can still change is whether an arm
# finishes inside `timeout` -- and that timeout is itself a wall clock, so
# a starved arm surfaces as NO GRADE rather than as wrong copper.
@app.function(cpu=(0.125, 4), memory=(1024, 3072), timeout=7200, max_containers=200)
def run_arm(arm: dict) -> dict:
    """One (tag, K) chain, graded, with the lines worth reading back."""
    tag, K = arm["tag"], int(arm["K"])
    env = dict(os.environ)
    env.update(BASE_ENV)
    env.update({k: str(v) for k, v in (arm.get("env") or {}).items()})
    wd = f"{REPO}/awx"
    # ARMS MUST BE INDEPENDENT (2026-09-12). detect_buses keeps a taut-string
    # memo on disk (awx/tmp/taut_memo, 257 shards / 464 MB locally) that is
    # written on a 60-SECOND WALL-CLOCK THROTTLE with staleness eviction --
    # so what it contains depends on timing. Modal REUSES a container across
    # inputs, so arm N inherits whatever arm N-1 happened to flush, and the
    # arms in a sweep are not independent samples. Measured symptom: K35
    # baselines land on exactly two reproducible states (60v/1583segs and
    # 70v/1561segs), both appearing inside ONE sweep from ONE image.
    # Start every arm from a cold cache. It costs recomputation; an
    # experiment whose arms contaminate each other costs the whole sweep.
    import shutil
    shutil.rmtree(Path(wd) / "tmp" / "taut_memo", ignore_errors=True)
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
    # WHERE it ran (2026-09-12). Three empty-env arms in ONE sweep from ONE
    # image returned K35 60/1583, 60/1583 and 70/1561 -- same experiment,
    # different answers. CP-SAT is pinned (num_workers=4, deterministic
    # time) and BLAS is pinned, and a 1-ULP perturbation of the lane
    # separations is measurably inert, so the cause is none of those. The
    # next suspect is the MACHINE: Modal reuses containers across inputs
    # and may place them on different CPUs, and this records enough to
    # cluster the answers by host and by instruction set.
    import platform, socket
    try:
        flags = subprocess.run(["bash", "-c", "grep -m1 ^flags /proc/cpuinfo"],
                               capture_output=True, text=True).stdout
        model = subprocess.run(
            ["bash", "-c", "grep -m1 -E '^(model name|Model|vendor_id)' /proc/cpuinfo"
             " || sed -n '1,6p' /proc/cpuinfo"],
            capture_output=True, text=True).stdout.strip()
    except Exception:
        flags = model = ""
    where = {"host": socket.gethostname(), "machine": platform.machine(),
             "cpus": os.cpu_count(), "model": model[:80],
             "fma": "fma" in flags, "avx512": "avx512f" in flags,
             "py": platform.python_version()}
    return {"tag": tag, "K": K, "secs": secs, "rc": p.returncode,
            "grade": grade.strip(), "env": arm.get("env") or {},
            "where": where,
            "chain_out": out.splitlines()[-60:], "logs": logs}


@app.local_entrypoint()
def main(arms: str = "awx/arms.example.json", out: str = "", dedupe: bool = True):
    spec = json.loads(Path(arms).read_text())
    jobs = [{"tag": a["tag"], "K": k, "env": a.get("env") or {}}
            for a in spec for k in a["K"]]
    # DEDUPE BY (env, K), NOT BY (tag, K) (2026-09-12). Merging three arms
    # files that each carried their own baseline queued the SAME experiment
    # under four tags -- 12 duplicate arms, because the merge compared tags
    # and the tag is just a label. The environment IS the experiment.
    # A tag whose env+K another tag already covers is dropped and named, so
    # a deliberate repeat (a determinism control) is visible as a removal
    # rather than silently honoured.
    # --no-dedupe: a REPEAT of one configuration is the whole point when
    # measuring a noise floor (N identical arms). The dedupe below would
    # collapse such a sweep to one arm and quietly answer nothing.
    seen, uniq, dropped = {}, [], []
    for j in (jobs if dedupe else []):
        sig = (tuple(sorted(j["env"].items())), j["K"])
        if sig in seen:
            dropped.append(f'{j["tag"]}/K{j["K"]} (= {seen[sig]})')
            continue
        seen[sig] = f'{j["tag"]}/K{j["K"]}'
        uniq.append(j)
    if dropped:
        print(f"dropped {len(dropped)} duplicate arm(s) by (env, K): "
              + ", ".join(dropped), flush=True)
    if dedupe:
        jobs = uniq
    else:
        print(f"dedupe OFF: running all {len(jobs)} arm(s) as given", flush=True)
    # RESUME (2026-09-12): with the per-arm write above, an interrupted
    # sweep leaves a partial `out`. Skip what it already holds, so a
    # killed client costs only the arms that were in flight rather than
    # the whole sweep -- the second half of the lesson that cost 112 arms.
    res = []
    if out and Path(out).exists():
        try:
            res = json.loads(Path(out).read_text())
        except Exception:
            res = []
        done = {(r["tag"], int(r["K"])) for r in res if r.get("grade")}
        if done:
            before = len(jobs)
            jobs = [j for j in jobs if (j["tag"], j["K"]) not in done]
            print(f"resuming: {len(done)} arm(s) already in {out}, "
                  f"{before} -> {len(jobs)} to run", flush=True)
    print(f"{len(jobs)} task(s): " + ", ".join(f'{j["tag"]}/K{j["K"]}' for j in jobs),
          flush=True)
    t0 = time.time()
    # INCREMENTAL WRITE (2026-09-12). `modal run` makes an EPHEMERAL app
    # tied to this client, so if the client dies the whole sweep dies with
    # it -- measured the hard way: an out-of-memory kill took three
    # concurrent sweeps (112 arms) and wrote nothing, because the results
    # were only saved after the last arm returned. `.map` yields as arms
    # finish, so persist after every one: a client death then costs only
    # the arms still in flight.
    # order_outputs=False: `.map` otherwise yields IN INPUT ORDER, and with
    # backpressure a single slow arm stalls every arm behind it -- measured,
    # one K51 whole-menu arm that died on a 900 s runner heartbeat timeout
    # held up 90 others and dropped effective parallelism to 2.9x.
    # return_exceptions=True: and an arm that RAISES must not take the sweep
    # with it; it comes back as a value to record and move past.
    for r in run_arm.map(jobs, order_outputs=False, return_exceptions=True):
        if isinstance(r, BaseException):
            print(f'  ARM RAISED: {type(r).__name__}: {str(r)[:160]}', flush=True)
            continue
        res.append(r)
        if out:
            Path(out).write_text(json.dumps(
                sorted(res, key=lambda r: (r["tag"], r["K"])), indent=1))
        print(f'  [{len(res)}/{len(jobs)}] {r["tag"]:9s} K{r["K"]:<3d} '
              f'{r["secs"]:5d}s {(r["grade"] or "NO GRADE rc=%s" % r["rc"])[:70]}',
              flush=True)
    res.sort(key=lambda r: (r["tag"], r["K"]))
    print(f"\n=== {len(res)} arm(s) in {round(time.time()-t0)}s wall clock\n")
    for r in res:
        e = " ".join(f"{k}={v}" for k, v in r["env"].items()) or "(baseline)"
        print(f'{r["tag"]:10s} K{r["K"]:<3d} {r["secs"]:5d}s  {r["grade"] or "NO GRADE rc=%d" % r["rc"]}')
        print(f'{"":10s}      [{e}]')
    if out:
        Path(out).write_text(json.dumps(res, indent=1))
        print(f"\nfull logs -> {out}")
