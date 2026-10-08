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
KICAD_IMAGE = "kicad/kicad:10.0.0"


def _base_image():
    if not KICAD:
        return (modal.Image.debian_slim(python_version=PY_VERSION)
                .apt_install("curl", "procps", "build-essential", "zsh")
                .pip_install(*PINS))
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
