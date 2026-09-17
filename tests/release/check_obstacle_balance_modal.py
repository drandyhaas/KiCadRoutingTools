"""Run the obstacle ref-count release gate on Modal, for a laptop that cannot.

DIAGNOSTICS SURVIVE A FAILURE. Every intermediate board, its .kicad_pro
sibling, and every step's FULL log are written to the `krt-obsbal-work`
Volume under a per-run id, so a red gate can be diagnosed from the copper
and the routing log rather than from a pass/fail bit:

    modal volume ls  krt-obsbal-work
    modal volume get krt-obsbal-work /<run-id> <destdir>

    modal run --detach tests/release/check_obstacle_balance_modal.py
    modal run --detach tests/release/check_obstacle_balance_modal.py --mode whole

TWO MODES, `per-step` the default: one container PER CHAIN STEP with the work
dir carried between them on a Volume, so a preemption costs ONE step instead of
the whole chain -- measured, "Container terminated due to preemption. Your
Function will be restarted with the same input", twice, both times inside a
route step. `--mode whole` runs the entire chain in one container, closest to
what a maintainer runs locally. Both end in the SAME audit
(`check_obstacle_balance.parse_audit` / `verdict`, imported, never
reimplemented), so the two modes cannot drift into disagreeing about the gate.

**`--mode whole` DOES NOT FIT THIS BOARD.** Modal's timeout is per FUNCTION
CALL, and the measured chain is 3753 s against a 3600 s ceiling (see TIMEOUT_S),
so the whole-chain mode is killed ~40 s from the end having banked nothing.
Per-step is the default for that reason and not merely for nicer logs: its
longest single call is step 8 at 1467 s, 41% of the same budget. Keep `whole`
for a SMALLER board or a raised timeout; on glasgow it is a 62-minute way to
learn nothing.

RETRIES exist for that same preemption: the sweep's own big-board tier
(`modal_app.replay_big`), which routes this same board, carries max_retries=3.
The first cut of this file copied its memory tier and not its retry policy.

USE `--detach`. Without it the app is EPHEMERAL: it lives only as long as the
local `modal run` client, which spends the whole gate (62 min of container
time -- measured; the "~6 min" the local gate quotes is a LAPTOP figure and
does not survive the cloud) doing nothing but waiting. On a memory-constrained machine that idle
client is exactly what a background-process reaper kills, and the app goes with
it -- measured 2026-09-17: six launches, every one dying at `Created function
run_gate` having produced no output, while `modal app list` showed the app
ephemeral with its task RUNNING. The work reaches the cloud; only the client
dies. `--detach` keeps the app alive independently, and
`modal app logs <app-id>` reads the result back.

Bisected before concluding, because "the cloud run failed" has several possible
causes and only one was true here: a trivial `modal run` succeeded (Modal is
fine), and a probe using THIS image at THIS memory tier returned the router
version and confirmed the gate file (image and tier are fine).

WHY THIS EXISTS. `check_obstacle_balance.py` routes `glasgow_revC` because that
board REPRODUCES the stranded-cell bug -- its own docstring records that the
fast in-repo board reports every invariant BALANCED with the fix disabled, so a
cheaper board makes the gate vacuous. The cost of that choice is memory: the
corpus arms measure glasgow at **13.9 GB peak RSS**. On an 8 GB machine the
final route step is killed every time (measured 2026-09-17: four attempts,
dying at steps 6, 7, 8, 8, identically when detached from the shell, so it is
the board's footprint and not a harness artifact). A mandatory release gate
that cannot run on the release machine is not a gate.

WHAT IT DOES. Runs the SAME script, unmodified, in one container on the suite's
own image -- `run_all_modal.image`, imported rather than copied, so the
faithful-checkout assertion and the `build_router.py` step stay in one place.
The gate's exit code is the verdict, exactly as locally; this driver adds no
judgement of its own and parses no counts, for the reason the suite driver
gives: a container that dies prints no summary, and a driver deciding on parsed
output would read that silence as success.

MEMORY. 16 GB by default, above the 13.9 GB measured peak. Modal containers do
burst above the request -- the sweep's `replay_big` asks for 8 GB and routes
this same board -- but a gate should not depend on bursting, so the request is
sized from the measurement.
"""
import os
import sys
import time

import modal

#: Where the image puts the checkout. A CONSTANT, not imported: this module is
#: also executed INSIDE the container, where tests/stress/modal_suite is not on
#: the path -- importing at module scope there is a ModuleNotFoundError that
#: kills the run before the function is even called (measured).
REPO = "/opt/kicad-routing-tools"

if modal.is_local():
    # The suite's image: a clean checkout of HEAD with the router built and the
    # corpus asserted complete. IMPORTED rather than re-declared, so a change
    # to how the image is built reaches this gate for free, and the gate cannot
    # silently run against a different tree than the suite does.
    _HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(_HERE)),
        "tests", "stress", "modal_suite"))
    from run_all_modal import image, GIT_SHA          # noqa: E402
else:
    image, GIT_SHA = None, os.environ.get("KICAD_SWEEP_GIT", "unknown")

app = modal.App("kicad-obstacle-balance")
work_vol = modal.Volume.from_name("krt-obsbal-work", create_if_missing=True)
WORK = "/work"

#: The in-repo board this gate is pinned to, and its REQUIRED sibling project:
#: the .kicad_pro carries min_hole_to_hole 0.25, the rule the violating via pair
#: breaks, and routing without it resolves a looser floor from the stock
#: netclass (#441).
BOARD = "kicad_files/glasgow_revC.kicad_pcb"
PROJ = "kicad_files/glasgow_revC.kicad_pro"

GATE = "tests/release/check_obstacle_balance.py"
#: MEASURED on Modal 2026-09-17 (per-step, 9 steps, this board, this budget):
#:
#:     steps 1-6  planes / fanouts / clearance / diff      54 s total
#:     step 7     route.py                               1452.8 s
#:     step 8     route.py, the load-bearing duplicate   1466.8 s
#:     step 9     route.py retry                          779.9 s
#:     -------------------------------------------------------------
#:     total                                              3753 s  (62.5 min)
#:
#: Three things follow from those numbers, and the first is why `main` defaults
#: to per-step:
#:   * 3753 s EXCEEDS this 3600 s timeout, so `run_whole` cannot finish this
#:     chain -- the timeout is per FUNCTION CALL, so splitting the chain is not
#:     merely a nicer log, it is the only mode that fits. Per-step's longest
#:     call is step 8 at 1467 s, 41% of the budget.
#:   * A cloud container is NOT faster than the laptop here; it is ~3-5x
#:     SLOWER per route step. The "~6 minutes" the local gate's own docstring
#:     quotes is a local, uninstrumented figure -- budget an hour.
#:   * Steps 7 and 8 are the same command and land within 1% of each other,
#:     confirming the duplicate is load-bearing rather than a copy-paste slip;
#:     the retry is about half a full route step.
TIMEOUT_S = 3600
RETRIES = modal.Retries(max_retries=3)

#: MEASURED, not guessed. The first cut asked for 16 GB on the strength of a
#: corpus `peak_rss_mb` of 14216 for glasgow's route.py -- but that field reads
#: 11.2 GB for esp_prog too, a 17-net board that finishes in 25 s, so it is not
#: a per-process figure and 16 GB was never a requirement. Watched live, the
#: routing step holds about 1 GB.
MEM_MB = 2048
CPUS = 1.0


def _gate_module():
    """The gate itself, imported IN the container, so the chain and the verdict
    have ONE definition shared by both modes."""
    sys.path.insert(0, os.path.join(REPO, "tests", "release"))
    import check_obstacle_balance as G
    return G


@app.function(image=image, memory=MEM_MB, cpu=CPUS, timeout=TIMEOUT_S,
              retries=RETRIES)
def run_gate() -> dict:
    """Run the gate once, STREAMING its output to the container's stdout.

    Deliberately NOT `capture_output=True`. Two reasons, both measured:

      * With `--detach` the local client may die (this machine loses its Modal
        connection often enough that it happened twice in one session -- a DNS
        failure and a grpclib `'Connection' object has no attribute
        '_transport'`). A verdict RETURNED to a dead client is lost; a verdict
        PRINTED is in `modal app logs <app-id>` forever.
      * Captured output appears only when the function returns, so a ~10 minute
        gate shows nothing at all and reads as a stall -- which is exactly what
        made this runner hard to tell apart from a hang.
    """
    import subprocess
    t0 = time.time()
    p = subprocess.run([sys.executable, "-u", GATE], cwd=REPO)
    secs = round(time.time() - t0, 1)
    # Printed, so `modal app logs` carries the verdict even with no client.
    print(f"\n=== OBSTACLE BALANCE GATE: rc={p.returncode} in {secs}s "
          f"on {os.environ.get('KICAD_SWEEP_GIT', '?')} ===")
    print("GATE VERDICT: " + ("ALL GREEN" if p.returncode == 0 else "FAILED"))
    return {"rc": p.returncode, "seconds": secs}


# ---------------------------------------------------------------- per-step

@app.function(image=image, memory=MEM_MB, cpu=CPUS, timeout=600,
              retries=RETRIES, volumes={WORK: work_vol})
def seed(run_id: str) -> str:
    """Put the board and its REQUIRED sibling project in the shared work dir."""
    import shutil
    d = os.path.join(WORK, run_id)
    os.makedirs(d, exist_ok=True)
    for rel in (BOARD, PROJ):
        srcp = os.path.join(REPO, rel)
        # The gate's own guard: a missing or EMPTY input means the chain tests
        # nothing, and must abort rather than grade a partial log.
        if not os.path.isfile(srcp) or os.path.getsize(srcp) == 0:
            raise SystemExit(f"ABORT: missing or empty {rel}")
        shutil.copy2(srcp, os.path.join(d, "glasgow_revC" + os.path.splitext(rel)[1]))
    work_vol.commit()
    print(f"seeded {d}: {sorted(os.listdir(d))}")
    return d


@app.function(image=image, memory=MEM_MB, cpu=CPUS, timeout=TIMEOUT_S,
              retries=RETRIES, volumes={WORK: work_vol})
def run_step(spec: tuple) -> dict:
    """One chain step, from the gate's OWN CHAIN. Returns its log to be audited.

    `capture_output=True` here (unlike `whole`) because the caller assembles
    every step's text into the one audit; the step ALSO prints its own tail on
    failure, so a preempted or aborted step is legible in `modal app logs`
    without the caller."""
    import subprocess
    i, n, run_id = spec
    G = _gate_module()
    work_vol.reload()                       # see what the previous step wrote
    d = os.path.join(WORK, run_id)
    src = os.path.join(d, "glasgow_revC.kicad_pcb")
    argv = G.CHAIN[i].format(D=d, SRC=src).split()
    argv = [sys.executable, "-u", "-X", "utf8",
            os.path.join(REPO, "py_router", argv[0])] + argv[1:]
    env = dict(os.environ, KICAD_OBSTACLE_AUDIT="1",
               KICAD_OBSTACLE_LEDGER="1", KICAD_RESIDENCY_STATS="1")
    print(f"  [{i + 1}/{n}] {os.path.basename(argv[4])}", flush=True)
    t0 = time.time()
    p = subprocess.run(argv, cwd=d, env=env, capture_output=True, text=True)
    log = p.stdout + p.stderr
    secs = round(time.time() - t0, 1)
    # PERSIST the step's full log beside the boards, BEFORE the commit that
    # hands them on. The log is also returned to the caller, but a caller can
    # die -- this client died three times in one session -- and a verdict with
    # no log behind it cannot be diagnosed. The boards were already durable
    # (they have to be, to feed the next step); the logs were not.
    logs_dir = os.path.join(d, "logs")
    os.makedirs(logs_dir, exist_ok=True)
    with open(os.path.join(logs_dir, f"step{i + 1:02d}.log"), "w",
              encoding="utf-8", errors="replace") as fh:
        fh.write(f"$ {' '.join(argv)}\n\nrc={p.returncode} in {secs}s\n\n")
        fh.write(log)
    work_vol.commit()                       # hand the boards AND logs on
    if p.returncode != 0:
        print(f"ABORT: step {i + 1} exited {p.returncode}:")
        for t in log.strip().splitlines()[-6:]:
            print("    " + t)
    else:
        print(f"  [{i + 1}/{n}] ok in {secs}s", flush=True)
    return {"i": i, "rc": p.returncode, "log": log, "seconds": secs}


def _gate_local():
    """The gate module on the LOCAL side, for the chain length and the audit."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import check_obstacle_balance as G
    return G


@app.local_entrypoint()
def main(mode: str = "per-step"):
    print(f"obstacle ref-count gate on Modal ({mode}): source {GIT_SHA}, "
          f"16 GB, timeout {TIMEOUT_S}s, retries 3")
    if mode == "whole":
        r = run_gate.remote()
        print("=" * 66)
        print(f"gate exited {r['rc']} in {r['seconds']}s on {GIT_SHA}")
        if r["rc"] != 0:
            raise SystemExit(f"OBSTACLE BALANCE GATE FAILED (rc={r['rc']})")
        print("ALL GREEN")
        return

    G = _gate_local()
    n = len(G.CHAIN)
    run_id = f"r{int(time.time())}"
    seed.remote(run_id)
    # SEQUENTIAL by construction: every step consumes the previous step's
    # board, so this cannot be a .map(). The win is not parallelism -- it is
    # that a preemption costs one step instead of the whole chain.
    logs, total = [], 0.0
    for i in range(n):
        r = run_step.remote((i, n, run_id))
        total += r["seconds"]
        logs.append(r["log"])
        if r["rc"] != 0:
            raise SystemExit(f"OBSTACLE BALANCE GATE ABORTED at step {i + 1} "
                             f"(rc={r['rc']}) -- a chain that dies before "
                             f"routing tests nothing")
    print(f"\nchain complete in {total:.0f}s of container time; auditing")
    print(f"boards and per-step logs kept on the volume; fetch with:\n"
          f"    modal volume get krt-obsbal-work /{run_id} <destdir>")
    # The GATE'S OWN parser and verdict, never a second copy.
    a = G.parse_audit("".join(logs))
    G.report(a)
    # verdict() returns (ok, failures) -- NOT an exit code, unlike every other
    # rc in this file. Read as one, the tuple (True, []) is truthy and a
    # PASSING gate reports FAILED. That shipped here once and was caught only
    # because the audit line above said 23 BALANCED / 0 LEAK headers while the
    # verdict line said FAILED: when the summary and the verdict disagree, the
    # verdict is the one to distrust.
    ok, failures = G.verdict(a)
    print(f"\n=== OBSTACLE BALANCE GATE (per-step) on {GIT_SHA} ===")
    print("GATE VERDICT: " + ("ALL GREEN" if ok else "FAILED"))
    for msg in failures:
        print(f"  {msg}")
    if not ok:
        raise SystemExit("OBSTACLE BALANCE GATE FAILED: " + "; ".join(failures))
