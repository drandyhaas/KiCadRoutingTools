#!/usr/bin/env python3
"""Modal app: the WHOLE ROUTE's K ladder (whole_route.py), one container per rung.

    modal run awx/modal_whole.py::main --ks 15,28,35,41,51 --out DIR   (::main: the app has two entrypoints)
    modal run awx/modal_whole.py::main --ks 18,26,32,38,42,44 --out DIR --env "BASE=tmp/zynq/zynqF.kicad_pcb;DEST=U2" \
        --ins tmp/zynq/zynqF.kicad_pcb,tmp/zynq/zynqF.kicad_pro,tmp/zynq/zynqF.ladder.txt   (the zynq article)

Each rung runs whole_route.py (the fanout on the whole route's ends, the solve, the loop, the route, the checks) in
its own container and sends back its log and its routed board, written to DIR/kK.log and DIR/kK_seq.kicad_pcb, so a
cloud ladder can be held against the laptop's copper for copper. The stack is the LAPTOP's: its Python (3.14) and
the same pinned numpy / scipy / shapely / ortools (modal_k.py: a python or numpy change moves routed copper). The
working tree is shipped as it stands, uncommitted edits and all.

    MODAL_WHOLE_KICAD=1 modal run awx/modal_whole.py::stage --cmd 'zsh CHAIN.sh ...' --ins ... --outs ...

runs a whole routing chain (the route step's KiCad legs and grades want pcbnew and kicad-cli) in an image built on
KiCad's own, with the same pins on KiCad's system python.

    modal run awx/modal_whole.py::boards --seeds 2,3,4 --out DIR [--cases zynq,sdi] [--rounds R] [--sets "K=V;K=V"]

runs the BUS BOARDS check (tests/stress/bus_boards.py), one container per board and SEED: the bus step's one seed
(BUS_SEED, cp_seed -- every solve of a round searches under it, so another seed is another draw of the same board).
Each run's log, the route's own log, its report row and the board it hands on come back under DIR/seedS/CASE.
"""
from __future__ import annotations

import io
import os
import signal
import subprocess
import tarfile
import time
from pathlib import Path

import modal

REPO = "/opt/krt"
_src = Path(__file__).resolve().parents[1]
PY_VERSION = os.environ.get("MODAL_WHOLE_PY", "3.14")
PINS = ("numpy==2.3.3", "scipy==1.16.2", "shapely==2.1.2", "ortools==9.15.6755",
        "pillow==12.0.0")      # (the renders: synth_layers.py draws every case, and run through ::stage it had none)
# MODAL_WHOLE_KICAD=1 (read here, client side): KiCad in the image, for a whole chain whose route step and grades use
# pcbnew and kicad-cli -- the stress app's recipe (tests/stress/modal_sweep/modal_app.py). It runs on the KiCad
# image's own system python, the one that imports the distro's pcbnew, not on the laptop's 3.14.
KICAD = os.environ.get("MODAL_WHOLE_KICAD", "") in ("1", "true", "on")
KICAD_IMAGE = "kicad/kicad:10.0.6"


# The ROUTE's own python (run_board): uv's standalone build of the same version -- Clang, the tail-call interpreter,
# BOLT -- with the same pins, beside the image's (Docker's, built by GCC, which the Modal runtime and the other
# functions keep). Measured in one container: the interpreter's own work 1.2 to 1.6 times as fast (an int loop 0.49 s
# against 0.31, 200k small numpy calls 0.45 against 0.37), CP-SAT and the big-array numpy the same; the JIT on, no
# faster. The router's extension is one build for both: the same CPython ABI.
ROUTE_PY = "/opt/uvpy/bin/python"


def _base_image():
    if not KICAD:
        return (modal.Image.debian_slim(python_version=PY_VERSION)
                .apt_install("curl", "procps", "build-essential", "zsh")
                .pip_install(*PINS)
                .run_commands("curl -LsSf https://astral.sh/uv/install.sh | sh",
                              f"/root/.local/bin/uv python install {PY_VERSION}",
                              f"/root/.local/bin/uv venv /opt/uvpy --python {PY_VERSION} --python-preference only-managed",
                              "/root/.local/bin/uv pip install --python /opt/uvpy/bin/python " + " ".join(PINS)))
    # USER root: the image runs as USER kicad; python-is-python3: Modal's builder runs `python -m pip`;
    # --break-system-packages: PEP 668 guards the system python
    return (modal.Image.from_registry(KICAD_IMAGE, setup_dockerfile_commands=["USER root"])
            .apt_install("curl", "procps", "build-essential", "zsh", "python3-pip", "python-is-python3")
            .pip_install(*PINS, extra_options="--break-system-packages")
            .run_commands('python3 -c "import pcbnew, sys; print(\'pcbnew\', pcbnew.GetBuildVersion(), '
                          '\'on python\', sys.version.split()[0])"', "kicad-cli version"))


image = (
    _base_image()
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
        f"cd {REPO}/py_router && python3 -c \"import obstacle_map, grid_router;"
        f" print('grid_router', grid_router.__version__)\"",
        f"cd {REPO}/awx && python3 -c \"import fanout_from_plan, whole_solve;"
        f" print('awx imports ok')\"",
        *(() if KICAD else (
            f"cd {REPO}/py_router && {ROUTE_PY} -c \"import sys, obstacle_map, grid_router;"
            f" print('route python', sys.version.split()[0], 'grid_router', grid_router.__version__)\"",
            f"cd {REPO}/awx && {ROUTE_PY} -c \"import fanout_from_plan, whole_solve; print('awx imports ok')\"",)),
    )
)

app = modal.App("bus622-whole-ladder", image=image)


@app.function(cpu=(0.125, 4), memory=(256, 8192), timeout=21600, max_containers=20)
def run_rung(K: int, rounds: int = 3, env: dict | None = None, tgz: bytes = b"", cap: int = 10800) -> dict:
    """One rung of the whole route, graded; its log and its routed board back. `env`: settings for the run (a bench
    of its own: BASE, DEST); `tgz`: files under awx/ the image leaves out (its tmp/: a bench built on the laptop);
    `cap`: seconds before the run and every stage under it are stopped -- its log and its best board still come back
    (the function's own timeout, past it, would return nothing)"""
    wd = f"{REPO}/awx"
    out = f"/tmp/whole_k{K}"
    if tgz:
        with tarfile.open(fileobj=io.BytesIO(tgz), mode="r:gz") as t:
            t.extractall(wd, filter="data")
    t0 = time.time()
    p = subprocess.Popen(["python3", "whole_route.py", str(K), out, str(rounds)], cwd=wd,
                         env=dict(os.environ, **(env or {})), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, errors="replace", start_new_session=True)
    try:
        log, _ = p.communicate(timeout=cap)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)        # the driver and every stage it started: one session
        log, _ = p.communicate()
        log += f"\n(stopped at the {cap} s cap)\n"
    board = ""
    # the run's result: its best round's board (whole_route: OUTDIR/best.kicad_pcb), else the last round's
    for f in [Path(out) / "best.kicad_pcb"] + [Path(out) / f"r{r}" / "seq.kicad_pcb" for r in range(rounds, 0, -1)]:
        if f.exists():
            board = f.read_text(errors="replace")
            break
    # every stage's output beside the board (the fanout's intermediate source boards aside), to find where two machines part
    tb = io.BytesIO()
    with tarfile.open(fileobj=tb, mode="w:gz") as t:
        t.add(out, arcname=f"k{K}", filter=lambda ti: None if "_srcres" in ti.name else ti)
    cpu = subprocess.run(["sh", "-c", "grep -m1 'model name' /proc/cpuinfo"], capture_output=True, text=True).stdout
    return {"K": K, "rc": p.returncode, "secs": round(time.time() - t0), "log": log,
            "board": board, "cpu": cpu.strip(), "tgz": tb.getvalue()}


@app.function(cpu=(0.125, 4), memory=(256, 8192), timeout=7200, max_containers=50)
def run_synth(tag: str) -> dict:
    """one synth_handoff.py case in its own container (synth_handoff.py --modal): its graded row and its files"""
    import csv
    out = "/tmp/synth"
    t0 = time.time()
    p = subprocess.run(["python3", "synth_handoff.py", "--only", tag, "--jobs", "1", "--outdir", out],
                       cwd=f"{REPO}/awx", capture_output=True, text=True, errors="replace")
    tsv = Path(out) / "handoff.tsv"
    rows = list(csv.DictReader(tsv.open(), delimiter="\t")) if tsv.exists() else []
    tb = io.BytesIO()
    with tarfile.open(fileobj=tb, mode="w:gz") as t:
        if (Path(out) / tag).is_dir():
            t.add(str(Path(out) / tag), arcname=tag)
    return {"tag": tag, "rc": p.returncode, "secs": round(time.time() - t0), "log": p.stdout + p.stderr,
            "row": rows[0] if rows else None, "tgz": tb.getvalue()}


@app.local_entrypoint()
def main(ks: str = "15,28,35,41,51", out: str = "modal_whole_out", rounds: int = 3, env: str = "", ins: str = "",
         cap: int = 10800):
    """`env` K=V;K=V for every rung (the zynq article: BASE=tmp/zynq/zynqF.kicad_pcb;DEST=U2); `ins` files under awx/,
    comma separated, shipped to every rung at the same relative paths (the zynq bench: its board, project and ladder);
    `cap` seconds a rung may run (3 h: the cloud's cores are slower than a laptop's)"""
    d = Path(out)
    d.mkdir(parents=True, exist_ok=True)
    Ks = [int(k) for k in ks.split(",") if k]
    E = dict(kv.split("=", 1) for kv in env.split(";") if kv)
    tb = io.BytesIO()
    if ins:
        with tarfile.open(fileobj=tb, mode="w:gz") as t:
            for f in [x for x in ins.split(",") if x]:
                t.add(str(_src / "awx" / f), arcname=f)
    for res in run_rung.map(Ks, kwargs={"rounds": rounds, "env": E, "tgz": tb.getvalue(), "cap": cap}):
        K = res["K"]
        (d / f"k{K}.log").write_text(res["log"])
        if res["board"]:
            (d / f"k{K}_seq.kicad_pcb").write_text(res["board"])
        if res.get("tgz"):
            (d / f"k{K}_run.tgz").write_bytes(res["tgz"])
        grade = next((ln for ln in reversed(res["log"].splitlines()) if ln.startswith("WHOLE ")), "(no grade)")
        print(f"K{K}: rc {res['rc']}, {res['secs']} s on {res['cpu'][:60]} -- {grade}", flush=True)


@app.function(cpu=(2, 8), memory=(256, 16384), timeout=86400, max_containers=40)
def run_board(case: str, seed: int = 1, rounds: int = 1, sets: list | None = None, cap: int = 79200) -> dict:
    """One board of the BUS BOARDS check (tests/stress/bus_boards.py --cases CASE) under the bus step's seed (BUS_SEED),
    in its own container: its report row, its log, the route's own log and the board it hands on. `sets`: K=V settings
    for the run; `cap`: seconds the board may take before the check stops it (its row says so). (Two cores asked, eight
    allowed: a single bus's fanout runs on about one, buses negotiated at once on one each; and a day allowed -- a
    board's fanout took more than 3.5 times as long here as on the desk it was written at, a four-bus board ten
    hours)"""
    import json
    out = "/tmp/bb"
    cmd = [ROUTE_PY if os.path.exists(ROUTE_PY) else "python3", "-u", f"{REPO}/tests/stress/bus_boards.py",
           "--cases", case, "--out", out, "-j", "1",
           "--rounds", str(rounds), "--timeout", str(cap), "--set", f"BUS_SEED={seed}"]
    for kv in sets or []:
        cmd += ["--set", kv]
    t0 = time.time()
    p = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace",
                         start_new_session=True)
    try:
        log, _ = p.communicate(timeout=cap + 1800)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        log, _ = p.communicate()
        log += f"\n(stopped {cap + 1800} s on)\n"
    rep = Path(out) / "report.json"
    row = json.loads(rep.read_text())["cases"].get(case) if rep.exists() else None
    tb = io.BytesIO()
    with tarfile.open(fileobj=tb, mode="w:gz") as t:
        for f in ("run.log", "bus.kicad_pcb", "bus.kicad_pro", "bus.kicad_dru"):
            if (Path(out) / case / f).exists():
                t.add(str(Path(out) / case / f), arcname=f"{case}/{f}")
    cpu = subprocess.run(["sh", "-c", "grep -m1 'model name' /proc/cpuinfo"], capture_output=True, text=True).stdout
    return {"case": case, "seed": seed, "rc": p.returncode, "secs": round(time.time() - t0), "log": log, "row": row,
            "cpu": cpu.strip(), "tgz": tb.getvalue()}


@app.local_entrypoint()
def boards(seeds: str = "1", cases: str = "", out: str = "modal_boards_out", rounds: int = 1, sets: str = ""):
    """modal run awx/modal_whole.py::boards --seeds 2,3,4 --out DIR [--cases a,b] [--rounds R] [--sets "K=V;K=V"]: the
    BUS BOARDS check, one container per board and seed -- every board (or `cases`) under each of `seeds`; each run's
    files under DIR/seedS, DIR/report.json ({case/seed: its row}) written as the runs come back, and the table"""
    import json
    man = json.loads((_src / "tests" / "fixtures" / "bus_boards" / "manifest.json").read_text())["cases"]
    cs = [c for c in cases.split(",") if c] or list(man)
    ss = [int(x) for x in seeds.split(",") if x]
    kv = [x for x in sets.split(";") if x]
    d = Path(out)
    d.mkdir(parents=True, exist_ok=True)
    print(f"{len(cs) * len(ss)} run(s): {', '.join(cs)} under seed(s) {', '.join(map(str, ss))} -> {d}", flush=True)
    rows = {}
    for res in run_board.starmap([(c, s_, rounds, kv) for s_ in ss for c in cs], order_outputs=False,
                                 return_exceptions=True):
        if isinstance(res, BaseException):
            print(f"a run failed in its container: {res!r}", flush=True)
            continue
        c, s_, r = res["case"], res["seed"], res["row"] or {}
        sd = d / f"seed{s_}"
        sd.mkdir(exist_ok=True)
        (sd / f"{c}.log").write_text(res["log"])
        if res["tgz"]:
            with tarfile.open(fileobj=io.BytesIO(res["tgz"]), mode="r:gz") as t:
                t.extractall(sd, filter="data")
        rows[f"{c}/{s_}"] = dict(r, rc=res["rc"], wall=res["secs"], cpu=res["cpu"])
        (d / "report.json").write_text(json.dumps(rows, indent=1))
        print(f"{c} seed {s_}: " + (f"{len(r['open'])} open, drc {r['drc']}, {len(r['walled'])} walled"
                                    if r else f"NO REPORT (rc {res['rc']})")
              + f" -- {res['secs']} s on {res['cpu'][:48]}", flush=True)
    print("\nboard        " + "  ".join(f"seed {s_:<10d}" for s_ in ss) + "   (open/drc/walled secs)")
    bad = 0
    for c in cs:
        cells = []
        for s_ in ss:
            r = rows.get(f"{c}/{s_}")
            ok = bool(r) and r.get("ok") and not r.get("why")
            bad += 0 if ok else 1
            cells.append(f"{len(r['open'])}/{r['drc']}/{len(r['walled'])} {r['secs']:>5d}s" if r and "open" in r
                         else "none")
        print(f"{c:12s} " + "  ".join(f"{x:<15s}" for x in cells))
    print(f"\n{'every run clean' if not bad else f'{bad} run(s) NOT clean'} -- {d / 'report.json'}", flush=True)


@app.function(cpu=(0.125, 4), memory=(256, 8192), timeout=14400)
def run_stage(cmd: str, env: dict, tgz: bytes, outs: list) -> dict:
    """One command in the image on files shipped from the laptop AT THEIR OWN ABSOLUTE PATHS (a stage's JSON names the
    files it read), `outs` sent back the same way: a stage the two machines disagree on, replayed from the same bytes"""
    with tarfile.open(fileobj=io.BytesIO(tgz), mode="r:gz") as t:
        t.extractall("/", filter="fully_trusted")
    for o in outs:
        os.makedirs(os.path.dirname(o), exist_ok=True)
    p = subprocess.run(["zsh", "-c", cmd], cwd=f"{REPO}/awx", env=dict(os.environ, **env),
                       capture_output=True, text=True, errors="replace")
    ob = io.BytesIO()
    with tarfile.open(fileobj=ob, mode="w:gz") as t:
        for o in outs:
            if os.path.exists(o):
                t.add(o, arcname=o.lstrip("/"))
    return {"rc": p.returncode, "log": p.stdout + p.stderr, "tgz": ob.getvalue()}


@app.local_entrypoint()
def stage(cmd: str, ins: str, outs: str, env: str = "", into: str = "modal_stage_out"):
    """modal run awx/modal_whole.py::stage --cmd '...' --ins a,b --outs c,d [--env K=V;K=V] [--into DIR]: `ins` (files or
    directories, absolute) go up at their own paths, `outs` (absolute) come back under DIR at theirs"""
    tb = io.BytesIO()
    with tarfile.open(fileobj=tb, mode="w:gz") as t:
        for f in [x for x in ins.split(",") if x]:
            t.add(f, arcname=os.path.abspath(f).lstrip("/"))
    E = dict(kv.split("=", 1) for kv in env.split(";") if kv)
    res = run_stage.remote(cmd, E, tb.getvalue(), [x for x in outs.split(",") if x])
    Path(into).mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(res["tgz"]), mode="r:gz") as t:
        t.extractall(into, filter="data")
    (Path(into) / "stage.log").write_text(res["log"])
    print(f"rc {res['rc']}; outputs under {into}; log {into}/stage.log", flush=True)
