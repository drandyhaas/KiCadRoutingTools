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

# The LOCAL stack, exactly. 3.13 was "the nearest Modal base" when this was
# written; **Modal now offers 3.14, which is the laptop's own** -- and the
# baseline arms that were meant to prove whether the difference matters say
# it DOES: under 3.13 the K-ladder baseline reads 38 / 70 / 88 / 124 where
# the laptop reads 34 / 60 / 80 / 115 (2026-09-15, 36-container sweep). So
# the version is a knob now, and `MODAL_K_PY=3.14` is how a cloud sweep is
# compared with a local number at all. The default stays 3.13 so an old
# sweep's numbers keep meaning what they meant.
PY_VERSION = os.environ.get("MODAL_K_PY", "3.13")
PINS = ("numpy==2.3.3", "scipy==1.16.2", "shapely==2.1.2", "ortools==9.15.6755")

image = (
    modal.Image.debian_slim(python_version=PY_VERSION)
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

# DETERMINISM HYGIENE, always applied -- exactly what `chain_k.sh` exports
# for itself. One BLAS thread: a pool sized from the machine makes a
# reduction's summation order machine-dependent. This is not an arm and
# has no opinion about routing.
DETERMINISM_ENV = {
    "OMP_NUM_THREADS": "1", "VECLIB_MAXIMUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}

# THE JOINT-SOLVE ARM -- seventeen chain flags, and **OPT-IN since
# 2026-09-16**. It used to be applied to every arm unconditionally, under
# the name BASE_ENV, ON TOP of whatever the arms file set. Nothing in an
# arms file said so, and a caller writing {"env": {"PLAN_JUDGE": "count"}}
# reasonably believed that was the whole configuration -- so every cloud
# number this repo has recorded is the joint-solve arm, and NO cloud run
# had ever executed the arm the laptop runs.
#
# It cost a whole 36-container sweep: cloud K28 came back 38 vias where the
# laptop's jcl is 34, and that was read as a PLATFORM difference. It is
# not. Running the chain LOCALLY under these flags gives 38 vias exactly
# (2026-09-16, `baseenv28`), so the flags are the entire via difference.
#
# `--base joint-solve` (or MODAL_K_BASE=joint-solve) brings it back for a
# deliberate comparison with the older recorded numbers; the DEFAULT is
# now "nothing", i.e. the cloud runs what the laptop runs.
JOINT_SOLVE_ARM = {
    "SRC_ROUNDS": "0", "SEL_RETRY": "6", "EXACT_LANE": "1", "DST_FACE_ASK": "1",
    "DST_WALK": "3", "SF_SWIM": "30", "BRAID_EXIT_GUARD": "1",
    "BRAID_SWIM_HOLD": "1", "SEL_XING": "2", "SF_EQUIV": "2",
    "SF_JUDGE": "braid", "BRAID_ONE_DIVE": "5", "DST_RESIDUE": "3",
    "DST_RESIDUE_POOL": "displaced", "DST_RESIDUE_CANDS": "4",
    "BRAID_ALT_SOLVER": "cpsat", "BRAID_CPSAT_DET": "40",
}
BASES = {"none": {}, "joint-solve": JOINT_SOLVE_ARM}
BASE_NAME = os.environ.get("MODAL_K_BASE", "none")
# Nothing here raises a time budget any more: there are none. Every loop
# is capped in JUDGE CALLS and every solve in nodes or deterministic
# time, so a container answers exactly what the laptop answers, however
# much slower it is. That is what made the first cloud sweep unreadable
# -- two identical runs of the K35 baseline came back 72 vias / 1436
# segs and 58 / 1840.

# What to read back from a replan round. `replan: best` is the verdict;
# the round lines say KEPT or rejected and why; the audit lines are the
# ones that decide --apply=refan vs strip, because they are where a probe
# and its apply DISAGREE (asked one berth, laid another).
RP_KEEP = re.compile(
    r"replan: best|round \d+:|KEPT|rejected|not exact|CO-MOVED|"
    r"unfaithful|not clean|ends DIFFER|LAYER |KIND |GAP off|braid \d+ s:")


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
    # WHERE THE VIAS CAME FROM. A cloud via count could be READ but never
    # ATTRIBUTED, because the lines that say which lanes were closed by the
    # rescue chain -- and at what price -- were all filtered out. The excess
    # over the human is concentrated in exactly that TAIL population (lanes
    # refused on the first pass, then closed by the x4 budget, the last call,
    # or a blocker rip, at 5-8 vias where a first-pass lane costs 2), so a
    # sweep without these lines cannot tell a plan change from a tail change
    # and reads as noise.
    r"lanes: \d+/\d+|rescued at x4|last call|kept attempt|rip \[|econ re-lay|"
    r"unplaced|NOT escaped|cannot be reached by the spine|"
    r"destination pass \d+: planner judge|destination re-plan|berth audit|"
    r"launch order:|target order:|re-lay rungs|dp: |wrote |"
    # THE CANARY (2026-09-16). The pages-first solve line carries the
    # instance size AND the objective+bound the solver stopped at, and it
    # is THE diagnostic for a cloud arm that disagrees with the laptop:
    # same instance + different objective is the solver, a different
    # instance is the menus upstream of it. It was not kept, so a cloud
    # run could not be bisected against a local one at all -- the only
    # pages-first line that survived did so by accident, because it
    # happens to contain the word "unplaced".
    r"pages-first: \d+ nets, |pages-first: iteration|pages-first: model vias|"
    # the SMOOTHER's own stats (#536). Two runs agreeing on vias and
    # completion can still differ in SEGMENT COUNT by a third, and the
    # only line that says why is this one.
    r"smooth_octolinear_chains|"
    # THE BRAID-TIER JUDGE's own verdicts (PLAN_PAGES_TIER). Without these
    # a tier arm is a black box: the boards move and nothing says whether
    # the judge fired, how often, or which way it decided -- and an absent
    # line reads as "it never ran", which is a FILTER artifact, not a fact.
    r"braid tier|tier round|tier: ")


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
# RESERVED memory is what you PAY for; the second number is only a CEILING,
# and a ceiling costs nothing until it is used. (1024, 3072) was the worst of
# both -- a gigabyte reserved per container AND a 3 GB cap over a 169k-row
# CP-SAT solve that ratchets its arena. Reserve little, cap generously.
@app.function(cpu=(0.125, 4), memory=(256, 8192), timeout=7200, max_containers=200)
def run_arm(arm: dict) -> dict:
    """One (tag, K) chain, graded, with the lines worth reading back."""
    tag, K = arm["tag"], int(arm["K"])
    env = dict(os.environ)
    env.update(DETERMINISM_ENV)
    env.update(BASES[arm.get("base", BASE_NAME)])
    env.update({k: str(v) for k, v in (arm.get("env") or {}).items()})
    # WHICH PLANNER THIS ARM ACTUALLY RAN (2026-09-15). Nothing sets
    # PLAN_PAGES for you, while the local runner exports `PLAN_PAGES=1` --
    # so an arms file that forgets it quietly runs a DIFFERENT PLANNER from
    # the laptop and every PLAN_PAGES_* flag in that arm is inert. It is invisible in the
    # result, because the chain grades fine and just answers a different
    # question: a 36-container sweep was read as "the pages-first gains do
    # not reproduce in the cloud" when not one container had run
    # pages-first, and the give-away -- every arm bit-identical, because
    # the braid portfolio's two arms differ only by a `pages_first` marker
    # that is never written -- read as a finding. The planner now travels
    # with the grade.
    # SHORT, because the progress line truncates the grade at 70 characters
    # and a long stamp pushes `vias=` off the end -- which is the number the
    # sweep exists to report.
    planner = "pf" if env.get("PLAN_PAGES", "0") not in ("", "0") else "OLD-PLANNER"
    base = arm.get("base", BASE_NAME)
    if base != "none":
        planner += "+" + base
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
    # FROM STDOUT, ANCHORED. `out` concatenates stdout+stderr, so scanning it
    # in reverse reads ALL OF STDERR FIRST -- and pick_braid.py prints
    # "braid A/B: <file>: NO GRADE -- ..." to stderr for any candidate that
    # did not grade. That substring contains "GRADE", so a arm whose chain
    # succeeded but which had one dud braid candidate reported NO GRADE for
    # the whole arm, and --resume then treated it as done and never re-ran
    # it. Live since the braid portfolio became default-ON.
    grade = next((ln for ln in reversed(p.stdout.splitlines())
                  if ln.startswith("GRADE ")), "")
    grade = f"[{planner}] {grade}" if grade else grade
    # THE REPLAN ROUND, optional (arm["replan"] = extra argv for replan.py).
    # replan.py reads the chain's own outputs -- tmp/TAG_fo_kK.kicad_pcb and
    # tmp/TAG_kK.kicad_pcb -- so it can only run AFTER the chain, in the same
    # container, and only if the chain produced them.
    rp = arm.get("replan")
    rp_grade, rp_secs, rp_lines, rp_rc = "", 0, [], None
    if rp and p.returncode == 0:
        t1 = time.time()
        q = subprocess.run(["python3", "-u", "replan.py", tag, str(K)] + [str(a) for a in rp],
                           cwd=wd, env=env, capture_output=True, text=True, errors="replace")
        rp_secs, rp_rc = round(time.time() - t1), q.returncode
        rtxt = (q.stdout + q.stderr).splitlines()
        rp_grade = next((ln for ln in reversed(rtxt) if ln.startswith("replan: best")), "")
        rp_lines = [ln for ln in rtxt if RP_KEEP.search(ln)][-300:]
        # A FILTER CANNOT REPORT WHAT IT FILTERS OUT. RP_KEEP keeps the
        # round verdicts, so a traceback matched nothing and a crashed
        # replan came back as rc1 with an EMPTY line list -- silence where
        # the error was. On a non-zero rc keep the raw tail as well.
        if q.returncode != 0:
            rp_lines = rp_lines + ["--- tail (rc %d) ---" % q.returncode] + rtxt[-25:]
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
    # RETURN THE BOARD when asked. Without this a cloud arm's copper can be
    # counted but never LOOKED AT -- and the cloud is the pipeline that plans
    # better, so its boards are the ones worth rendering. Base64 because the
    # result is JSON; ~300 KB a board, so ask per-arm, not by default.
    board_b64 = None
    if arm.get("return_board"):
        import base64
        bf = Path(wd) / "tmp" / f"{tag}_k{K}.kicad_pcb"
        if bf.exists():
            board_b64 = base64.b64encode(bf.read_bytes()).decode()
    # ...and ANY other artifact the arm names, as tmp/-relative globs, so a
    # DIAGNOSIS on the cloud is possible at all: the filtered `logs` above
    # cannot carry a plan sidecar or a judge dump, and the cloud is the only
    # place some phenomena exist (K44's SF_KEY_COST regression is inert
    # locally -- both arms bit-identical -- so it can only be debugged here).
    files_b64 = {}
    for pat in (arm.get("return_files") or []):
        import base64
        for f in sorted((Path(wd) / "tmp").glob(pat)):
            if f.is_file() and f.stat().st_size < 40_000_000:
                files_b64[f.name] = base64.b64encode(f.read_bytes()).decode()
    return {"tag": tag, "K": K, "secs": secs, "rc": p.returncode,
            "board_b64": board_b64, "files_b64": files_b64,
            "grade": grade.strip(), "env": arm.get("env") or {},
            "replan": list(rp) if rp else None, "replan_rc": rp_rc,
            "replan_secs": rp_secs, "replan_grade": rp_grade.strip(),
            "replan_lines": rp_lines,
            "where": where,
            "chain_out": out.splitlines()[-60:], "logs": logs}


@app.local_entrypoint()
def main(arms: str = "awx/arms.example.json", out: str = "", dedupe: bool = True):
    spec = json.loads(Path(arms).read_text())
    # ...and REFUSE a mixed sweep before it costs anything. Arms that run
    # different planners are not comparable, and the failure mode above was
    # a whole sweep of them read as one experiment.
    pf = {bool(str((a.get("env") or {}).get("PLAN_PAGES", "0")) not in ("", "0"))
          for a in spec}
    if len(pf) > 1:
        raise SystemExit(
            "modal_k: this arms file MIXES planners -- some arms set PLAN_PAGES "
            "and some do not, and nothing sets it for them. Arms on different "
            "planners are not comparable; set PLAN_PAGES explicitly on every arm.")
    bases = {a.get("base", BASE_NAME) for a in spec}
    print(f"modal_k: image python {PY_VERSION}; base arm(s) {sorted(bases)} "
          f"({'the LAPTOP configuration' if bases == {'none'} else 'NOT the laptop configuration'})")
    if pf == {False}:
        print("modal_k: WARNING -- no arm sets PLAN_PAGES, so every arm runs the "
              "OLD planner. The local chain runs pages-first; these numbers "
              "are NOT comparable with it.")
    jobs = [{"tag": a["tag"], "K": k, "env": a.get("env") or {},
             "replan": a.get("replan"), "return_board": a.get("return_board"),
             "return_files": a.get("return_files")}
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
        # the REPLAN ARGV is part of the experiment too: two replan arms can
        # differ only there (--apply=refan vs strip) with an identical env,
        # and keying on (env, K) alone would silently drop one of them --
        # the same class of mistake as keying on the tag.
        sig = (tuple(sorted(j["env"].items())), j["K"],
               tuple(j["replan"]) if j["replan"] else None)
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
              f'{r["secs"]:5d}s {(r["grade"] or "NO GRADE rc=%s" % r["rc"])[:96]}',
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
