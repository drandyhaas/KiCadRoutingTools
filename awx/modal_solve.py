#!/usr/bin/env python3
"""Is CP-SAT reproducible ON MODAL, at num_workers=4, on the instance the
chain actually builds?

WHY THIS EXISTS. The chain's K35 cloud result is bistable -- (60,1583) or
(70,1561), 19 of 72 identical-config arms -- and the divergence is one
solve: #2 of the residue search returns objective 44.12 in the arms that
end one way and 44.46 in the others, from a BYTE-IDENTICAL instance, with
status FEASIBLE. Locally CP-SAT is reproducible (3/3 identical solution
vectors at both 1 and 4 workers) -- but the chain is reproducible locally
too, so a local test cannot see the phenomenon at all. It has to run in a
container.

WHAT IT MEASURES, and the distinction that matters:
  WITHIN a container -- solve the same instance REPS times in one process.
  ACROSS containers  -- compare those answers between containers.
A split that is within-container-stable but across-container-variable is a
machine/scheduling effect; one that varies within a container is CP-SAT
itself. The chain cannot tell these apart; this can.

The instance is BUILT in the container by running the real fanout stage
with BRAID_L5_ALT_DUMP, so it is the chain's own model, not a synthetic.

    modal run awx/modal_solve.py --containers 6 --reps 3
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import modal

# Modal mounts the ENTRYPOINT FILE ALONE, at /root/ -- so in a container this
# module's own directory is /root, which holds no modal_k.py, and a plain
# `dirname(__file__)` on sys.path does not fix the import (measured: it still
# died on ModuleNotFoundError, traceback naming /root/modal_solve.py). The
# REPO is a separate copy the IMAGE makes, at /opt/krt, so the sibling lives
# at /opt/krt/awx. Locally it is beside this file. Probe for the file rather
# than branching on "am I remote", which nothing here can ask reliably.
for _d in (os.path.dirname(os.path.abspath(__file__)), "/opt/krt/awx"):
    if os.path.isfile(os.path.join(_d, "modal_k.py")):
        sys.path.insert(0, _d)
        break
from modal_k import REPO, image          # the same pinned image the sweeps use

app = modal.App("bus622-solve", image=image)


# the SMALL memory the sweeps use (1 GB request / 3 GB limit). A 169k-row
# CP-SAT solve is the heaviest thing in this repo and three of them in one
# process may not fit -- CP-SAT never returns its arena, so it ratchets.
# If a container dies, that is a measurement too, and a cheap one.
# RESERVED memory is what you PAY for; the second number is only a CEILING,
# and a ceiling costs nothing until it is used. (1024, 3072) was the worst of
# both -- a gigabyte reserved per container AND a 3 GB cap over a 169k-row
# CP-SAT solve that ratchets its arena. Reserve little, cap generously.
@app.function(cpu=(0.125, 4), memory=(256, 8192), timeout=3600, max_containers=32)
def probe(arg: dict) -> dict:
    """Build the chain's own alt instance here, then solve it REPS times."""
    import glob, hashlib, pickle, platform, socket, time
    import numpy as np
    reps = int(arg.get("reps", 3))
    workers = int(arg.get("workers", 4))
    wd = f"{REPO}/awx"
    sys.path.insert(0, wd)
    os.chdir(wd)
    env = dict(os.environ)
    env.update({
        "SRC_ROUNDS": "0", "SEL_RETRY": "6", "EXACT_LANE": "1", "DST_FACE_ASK": "1",
        "DST_WALK": "3", "SF_SWIM": "30", "BRAID_EXIT_GUARD": "1",
        "BRAID_SWIM_HOLD": "1", "SEL_XING": "2", "SF_EQUIV": "2", "SF_JUDGE": "braid",
        "BRAID_ONE_DIVE": "5", "DST_RESIDUE": "3", "DST_RESIDUE_POOL": "displaced",
        "DST_RESIDUE_CANDS": "4", "BRAID_ALT_SOLVER": "cpsat", "BRAID_CPSAT_DET": "40",
        "OMP_NUM_THREADS": "1", "VECLIB_MAXIMUM_THREADS": "1",
        "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1",
        "BRAID_L5_ALT_DUMP": "/tmp/inst/k35",
    })
    os.makedirs("/tmp/inst", exist_ok=True)
    nets = subprocess.run([sys.executable, "coherent_nets.py", "35"],
                          capture_output=True, text=True, env=env).stdout.strip()
    t0 = time.time()
    p = subprocess.run([sys.executable, "-u", "fanout_from_plan.py",
                        "/tmp/dump_fo_k35.kicad_pcb", "35",
                        "--board=fb_t2q_fresh.kicad_pcb"],
                       capture_output=True, text=True, env=env)
    files = sorted(glob.glob("/tmp/inst/*.pkl"), key=os.path.getsize)
    # NOT the hostname: every Modal container reports "modal", so counting
    # distinct hostnames collapses six containers to one and the run reports
    # that it could not test the across-container question when it just did.
    # MODAL_TASK_ID is per-container; the CPU vendor is per-MACHINE, and a
    # split across vendors is the strongest form of the answer.
    where = {"host": socket.gethostname(), "cpus": os.cpu_count(),
             "task": os.environ.get("MODAL_TASK_ID", ""),
             "py": platform.python_version()}
    try:
        vend = subprocess.run(["bash", "-c", "grep -m1 vendor_id /proc/cpuinfo"],
                              capture_output=True, text=True).stdout.strip()
        where["vendor"] = vend.split(":")[-1].strip()
    except Exception:
        pass
    if not files:
        return {"error": f"no dump (rc={p.returncode})", "tail": p.stdout[-500:],
                "where": where}
    import braid as te
    with open(files[-1], "rb") as fh:
        dump = pickle.load(fh)
    inst = dump["inst"]; x0 = dump.get("x0")
    cvec = np.asarray(inst["cvec"], float); nv = len(cvec)
    te.CPSAT_WORKERS = workers
    outs = []
    for _ in range(reps):
        t = time.perf_counter()
        x, msg, ok = te._milp_solve(cvec, inst["rows"], inst["lb"], inst["ub"],
                                    inst["integ"], np.zeros(nv), np.ones(nv),
                                    0, te.L5_GAP, x0=x0, solver="cpsat")
        outs.append({
            "obj": round(float(cvec @ x), 4) if ok else None,
            "sha": hashlib.sha1(np.round(x, 6).tobytes()).hexdigest()[:12] if ok else None,
            "secs": round(time.perf_counter() - t, 1), "msg": msg[:24]})
    return {"rows": len(inst["rows"]), "vars": nv, "workers": workers,
            "fanout_secs": round(time.time() - t0), "outs": outs, "where": where}


@app.local_entrypoint()
def main(containers: int = 6, reps: int = 3, workers: int = 4, out: str = ""):
    jobs = [{"reps": reps, "workers": workers} for _ in range(containers)]
    # return_exceptions: an OOM is a PLAUSIBLE outcome here (three 169k-row
    # CP-SAT solves in one 3 GB process), and without this one dead container
    # takes the whole map down and the survivors' answers are never written.
    res = list(probe.map(jobs, return_exceptions=True))
    res = [r if isinstance(r, dict) else {"error": f"{type(r).__name__}: {r}"}
           for r in res]
    if out:
        from pathlib import Path
        Path(out).write_text(json.dumps(res, indent=1))
    print(f"\n=== {len(res)} job(s), {reps} solve(s) each, num_workers={workers}\n")
    allsha = set(); hosts = set(); vendors = set()
    for i, r in enumerate(res):
        if r.get("error"):
            print(f"  job {i}: ERROR {r['error']}"); continue
        shas = [o["sha"] for o in r["outs"]]
        objs = sorted({o["obj"] for o in r["outs"]})
        allsha |= set(shas)
        w = r["where"]; hosts.add(w.get("task") or w.get("host"))
        vendors.add(w.get("vendor"))
        print(f"  job {i}: {r['rows']}x{r['vars']}  {w.get('vendor','?')} "
              f"{w.get('cpus')}cpu host {str(w.get('host'))[:12]}  "
              f"solves {[o['secs'] for o in r['outs']]}s  "
              f"obj {objs}  {'SAME' if len(set(shas))==1 else '*** DIFFERS WITHIN ***'}")
    print()
    # The DENOMINATOR, printed, because Modal's autoscaler decides how many
    # containers serve N inputs -- it may run every job in ONE. "6 containers
    # agreed" would then really be "one container agreed with itself", which
    # is the within-container question, already answered locally.
    print(f"  {len(hosts)} DISTINCT container(s) over {len(vendors)} CPU vendor(s) "
          f"{sorted(v for v in vendors if v)} served {len(res)} job(s); "
          f"{len(allsha)} distinct solution(s)")
    if not allsha:
        print("  VERDICT: nothing solved -- read the errors above; this measured nothing.")
    elif len(allsha) > 1:
        print(f"  VERDICT: {len(allsha)} distinct solutions -- CP-SAT is NOT reproducible")
        print("           here at this worker count. That is the bug, at the solver.")
    elif len(hosts) < 2 and len(vendors) < 2:
        print("  VERDICT: unanimous, but on ONE container and one CPU vendor -- this")
        print("           does NOT test the across-container question. Re-run bigger.")
    else:
        print(f"  VERDICT: unanimous across {len(hosts)} container(s) / "
              f"{len(vendors)} CPU vendor(s) -- CP-SAT is")
        print("           reproducible here, and the chain's bistability comes from")
        print("           somewhere EARLIER than this solve.")
