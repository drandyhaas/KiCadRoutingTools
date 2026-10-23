#!/usr/bin/env python3
"""A/B an engine change on the boards where per-net via sizes are LIVE.

#530 gives a net its own via size only when route.py runs WITHOUT
--via-size; 396 of the 403 recorded route steps pass it, so a stock corpus
replay (cloud_replay_sets.py) never exercises per-net vias at all. This
replays one recorded route.py step per board with --via-size/--via-drill
stripped, on two arms, and grades both with the BASE arm's checkers.

    python3 tests/stress/pernet_via_ab.py select  [--max-recorded-s 7200]
    python3 tests/stress/pernet_via_ab.py launch  --label NAME   # base=HEAD, fix=HEAD+`git diff HEAD`
    python3 tests/stress/pernet_via_ab.py collect --label NAME   # download, compare, stop the app

The image is the sweep's (tests/stress/modal_sweep/modal_app.py): a clean
`git archive HEAD` with KiCad (oracle legs live). The base arm runs it as is;
the fix arm applies the working tree's `git diff HEAD` inside its container,
so launch BEFORE committing the change under test. Every call writes its own
result to the `kicad-pernet-ab` volume (deploy + spawn: a dropped client link
does not cancel anything), and `collect` stops the app once every board is in.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path

import modal

REPO = "/opt/kicad-routing-tools"
VOL = "/ab"
_here = Path(__file__).resolve().parent
_repo_root = _here.parent.parent
CORPUS = Path(os.path.expanduser(os.environ.get("KICAD_STRESS_ROOT", "~/Documents/kicad_stress_test")))
LOCAL = CORPUS / "pernet_ab"            # results live beside the corpus, not in the repo

APP = os.environ.get("PERNET_AB_APP", "pernet-via-ab")
app = modal.App(APP)
vol = modal.Volume.from_name("kicad-pernet-ab", create_if_missing=True)
# copy_board.SIBLING_EXTS: the brief carries keep_away rules route.py reads (#1253)
SIBLINGS = (".kicad_pro", ".kicad_prl", ".kicad_dru", ".design-brief.json")

# The image (a clean `git archive HEAD` + KiCad) is only needed to DEPLOY and
# inside the container -- not for select/collect, whose worker processes
# re-import this module.
_NEED_IMAGE = (not modal.is_local()) or os.environ.get("PERNET_AB_DEPLOY") == "1"
if _NEED_IMAGE:
    if modal.is_local():
        os.environ.setdefault("KICAD_SWEEP_WITH_KICAD", "1")
        sys.path.insert(0, str(_here / "modal_sweep"))
    else:
        sys.path.insert(0, f"{REPO}/tests/stress/modal_sweep")
    from modal_app import image  # noqa: E402  the sweep's image

    @app.function(image=image, memory=6144, timeout=4 * 3600, volumes={VOL: vol})
    def replay(task: dict) -> dict:
        return _replay(task)


# ----------------------------------------------------------------- container
def _replay(task: dict) -> dict:
    """One (board, arm): write the input, route, grade with the BASE checkers."""
    import shutil
    out_dir = Path(VOL) / task["label"] / task["arm"] / task["board"]
    if (out_dir / "done.json").exists():
        return json.loads((out_dir / "done.json").read_text())
    wd = Path("/tmp/work") / task["arm"] / task["board"]
    shutil.rmtree(wd, ignore_errors=True)
    wd.mkdir(parents=True)
    for name, data in task["files"].items():
        (wd / name).write_bytes(data)
    code = REPO
    if task["arm"] == "fix":
        code = "/tmp/fixrepo"
        if not Path(code).exists():
            shutil.copytree(REPO, code, symlinks=True)
            p = subprocess.run(["patch", "-p1", "--forward"], input=task["patch"], text=True,
                               cwd=code, capture_output=True)
            if p.returncode != 0:
                raise RuntimeError("patch did not apply:\n" + p.stdout + p.stderr)
    args = list(task["args"])
    cmd = [sys.executable, "-u", "-X", "utf8",
           f"{code}/py_router/route.py"] + args + ["--json-out", str(wd / "route.json")]
    t0 = time.time()
    with open(wd / "route.log", "w") as lf:
        p = subprocess.Popen(cmd, cwd=wd, stdout=lf, stderr=subprocess.STDOUT,
                             env=dict(os.environ, **(task.get("env") or {})))
        _, status, ru = os.wait4(p.pid, 0)    # this child's own peak RSS (KB on Linux)
        p.returncode = os.waitstatus_to_exitcode(status)
    rc = p.returncode
    secs = time.time() - t0
    log = (wd / "route.log").read_text(errors="replace")
    res = dict(arm=task["arm"], board=task["board"], rc=rc, secs=round(secs),
               rss_gb=round(ru.ru_maxrss / 2**20, 2),
               per_net_line=(re.search(r"Per-net via sizes for[^\n]*", log) or [None])[0],
               rung_retry=len(re.findall(r"unblocked by rung-", log)),
               casualty=len(re.findall(r"[Cc]asualty", log)))
    out = wd / "out.kicad_pcb"
    if out.exists():
        for name, extra in (("drc", ["--json", str(wd / "drc.json")]), ("conn", [])):
            tool = "check_drc.py" if name == "drc" else "check_connected.py"
            g = subprocess.run([sys.executable, f"{REPO}/py_router/{tool}", str(out), "--quiet"] + extra,
                               cwd=wd, capture_output=True, text=True)
            (wd / f"{name}.log").write_text(g.stdout + g.stderr)
        if task["arm"] == "base":   # what the run inherited, graded once
            inp = wd / task["input_name"]
            subprocess.run([sys.executable, f"{REPO}/py_router/check_drc.py", str(inp), "--quiet",
                            "--json", str(wd / "input_drc.json")], cwd=wd, capture_output=True)
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in ("route.log", "route.json", "drc.json", "drc.log", "conn.log", "input_drc.json",
              "out.kicad_pcb", "out.kicad_pro"):
        if (wd / f).exists():
            shutil.copy2(wd / f, out_dir / f)
    (out_dir / "done.json").write_text(json.dumps(res, indent=1))
    vol.commit()
    return res


# --------------------------------------------------------------------- local
def _route_steps(rd: Path):
    """[(input, args)] of the recorded route.py steps in a run dir, file order."""
    man = rd / "redo_commands.sh"
    if not man.exists():
        return []
    steps, seen = [], set()
    for line in man.read_text().splitlines():
        line = line.strip()
        if line.startswith("#") or "py_router/route.py" not in line or "--help" in line:
            continue
        try:
            toks = shlex.split(line)
        except ValueError:          # a manifest line with unbalanced quotes
            continue
        k = [i for i, t in enumerate(toks) if t.endswith("py_router/route.py")][0]
        args = toks[k + 1:]
        # #1302: the step's recorded `NAME=value` env knobs, replayed with it.
        env = dict(t.split("=", 1) for t in toks[:k]
                   if re.match(r"^[A-Za-z_][A-Za-z0-9_]*=", t))
        outs = [args[i + 1] for i, t in enumerate(args[:-1]) if t == "--output"] or \
               [a for a in args[1:2] if a.endswith(".kicad_pcb")]
        if not outs or os.path.basename(outs[0]) in seen:
            continue
        seen.add(os.path.basename(outs[0]))
        steps.append((args[0], args, os.path.basename(outs[0]), env))
    return steps


def _recorded_secs(rd: Path, out_name: str):
    t = rd / "redo_commands.timings.jsonl"
    if not t.exists():
        return None
    for line in t.read_text().splitlines():
        if not line.strip():
            continue
        x = json.loads(line)
        a = x.get("argv", [])
        if any(s.endswith("py_router/route.py") for s in a) and \
                any(os.path.basename(s) == out_name for s in a):
            return x.get("seconds")
    return None


def _per_net(path: str):
    """{net_id: (dia, drill)} route.py resolves with --via-size omitted."""
    sys.path[:0] = [str(_repo_root / "py_router"), str(_repo_root / "rust_router")]
    from kicad_parser import parse_kicad_pcb
    from list_nets import board_default_netclass_param, fab_floors
    from design_rules import DesignRules
    import routing_defaults as defaults
    vs = board_default_netclass_param(path, "via_diameter") or defaults.VIA_SIZE
    vd = board_default_netclass_param(path, "via_drill") or defaults.VIA_DRILL
    pcb = parse_kicad_pcb(path)
    ncl = len(pcb.board_info.copper_layers or []) or 4
    dr = DesignRules.from_project(pcb, path, fab_floor=fab_floors(ncl),
                                  copper_layers=list(pcb.board_info.copper_layers or []))
    per = {}
    for nid in pcb.nets:
        if nid == 0:
            continue
        a, b = dr.draw_size("via_diameter", nid, default=None), dr.draw_size("hole_size", nid, default=None)
        if a and b and b < a and (abs(a - vs) > 1e-9 or abs(b - vd) > 1e-9):
            per[nid] = (round(a, 4), round(b, 4))
    return per


def _candidate(rd: str, max_s: float):
    """The board's main recorded route step, if per-net vias go live on it."""
    rd = Path(rd)
    steps = _route_steps(rd)
    if not steps:
        return None
    main = next((s for s in steps if "'*'" in shlex.join(s[1]) or "*" in s[1]), steps[0])
    inp, args, out_name, env = main
    src = Path(inp) if os.path.isabs(inp) else rd / inp
    secs = _recorded_secs(rd, out_name)
    if not src.exists() or (secs and secs > max_s):
        return None
    try:
        per = _per_net(str(src))
    except Exception:                                          # noqa: BLE001
        return None
    if not per:
        return None
    return dict(board=rd.name, set=rd.parent.name, src=str(src), args=args, out_name=out_name,
                recorded_s=secs, per_net=len(per), env=env)


def cmd_select(a):
    from concurrent.futures import ProcessPoolExecutor
    dirs = sorted(str(d) for d in CORPUS.glob("runs_set*/*") if (d / "redo_commands.sh").exists())
    with ProcessPoolExecutor(max_workers=a.jobs) as ex:
        rows = [r for r in ex.map(_candidate, dirs, [a.max_recorded_s] * len(dirs), chunksize=4) if r]
    LOCAL.mkdir(parents=True, exist_ok=True)
    (LOCAL / "tasks.json").write_text(json.dumps(rows, indent=1))
    print(f"{len(rows)} boards with live per-net vias on their main route step "
          f"(of {len(dirs)} recorded runs) -> {LOCAL / 'tasks.json'}")
    for r in rows:
        print(f"  {r['set']:16s} {r['board']:28s} {r['per_net']:4d} per-net nets, "
              f"recorded {r['recorded_s']}s")


def _task(row, arm, label, patch):
    src = Path(row["src"])
    files = {src.name: src.read_bytes()}
    for ext in SIBLINGS:
        sib = src.with_suffix(ext)
        if sib.exists():
            files[sib.name] = sib.read_bytes()
    args, skip = [], 0
    for t in row["args"]:
        if skip:
            skip -= 1
            continue
        if t in ("--via-size", "--via-drill"):
            skip = 1
            continue
        args.append(t)
    args[0] = src.name
    if "--output" in args:
        args[args.index("--output") + 1] = "out.kicad_pcb"
    else:
        args[1] = "out.kicad_pcb"
    return dict(label=label, arm=arm, board=row["board"], files=files, args=args,
                input_name=src.name, patch=patch if arm == "fix" else "",
                env=dict(row.get("env") or {}))


def cmd_launch(a):
    rows = json.loads((LOCAL / "tasks.json").read_text())
    if a.only:
        keep = set(a.only.split(","))
        rows = [r for r in rows if r["board"] in keep]
    patch = subprocess.run(["git", "-C", str(_repo_root), "diff", "HEAD"],
                           capture_output=True, text=True, check=True).stdout
    if not patch.strip():
        raise SystemExit("no working-tree change to A/B (git diff HEAD is empty)")
    sha = subprocess.run(["git", "-C", str(_repo_root), "rev-parse", "--short", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    print(f"deploying {APP} (base = clean {sha}; fix = {sha} + git diff HEAD, "
          f"{len(patch.splitlines())} lines)")
    subprocess.run(["modal", "deploy", __file__], check=True,
                   env=dict(os.environ, PERNET_AB_APP=APP, PERNET_AB_DEPLOY="1"))
    fn = modal.Function.from_name(APP, "replay")
    calls = {}
    for row in rows:
        for arm in ("base", "fix"):
            c = fn.spawn(_task(row, arm, a.label, patch))
            calls[f"{arm}/{row['board']}"] = c.object_id
    meta = dict(label=a.label, app=APP, base=sha, boards=[r["board"] for r in rows], calls=calls)
    out = LOCAL / a.label
    out.mkdir(parents=True, exist_ok=True)
    (out / "launch.json").write_text(json.dumps(meta, indent=1))
    (out / "patch.diff").write_text(patch)
    print(f"spawned {len(calls)} calls ({len(rows)} boards x 2 arms); collect with --label {a.label}")


def _drc(p: Path):
    if not p.exists():
        return None
    d = json.loads(p.read_text())
    return dict(n=d["violations"], by=d["by_type"], shorts=sum(1 for i in d.get("items", []) if i.get("short")),
                keys={(i["type"], i.get("net1"), i.get("net2"),
                       tuple(round(v, 2) for v in (i.get("via_loc") or i.get("loc") or [])))
                      for i in d.get("items", []) if i.get("short")})


def _conn(p: Path):
    if not p.exists():
        return None
    t = p.read_text()
    m = re.search(r"FAILED \((\d+) issues\)", t)
    return int(m.group(1)) if m else (0 if "EXIT=0" in t else None)


def cmd_collect(a):
    out = LOCAL / a.label
    meta = json.loads((out / "launch.json").read_text())
    subprocess.run(["modal", "volume", "get", "--force", "kicad-pernet-ab", f"/{a.label}", str(out)],
                   capture_output=True)
    root = out / a.label if (out / a.label).exists() else out
    done = {b: all((root / arm / b / "done.json").exists() for arm in ("base", "fix"))
            for b in meta["boards"]}
    print(f"{sum(done.values())}/{len(done)} boards complete in both arms")
    tot = dict(base_new_shorts=0, fix_new_shorts=0, base_conn=0, fix_conn=0, base_s=0, fix_s=0)
    for b in meta["boards"]:
        if not done[b]:
            continue
        rb = json.loads((root / "base" / b / "done.json").read_text())
        rf = json.loads((root / "fix" / b / "done.json").read_text())
        di = _drc(root / "base" / b / "input_drc.json")
        db, df = _drc(root / "base" / b / "drc.json"), _drc(root / "fix" / b / "drc.json")
        cb, cf = _conn(root / "base" / b / "conn.log"), _conn(root / "fix" / b / "conn.log")
        nb = len(db["keys"] - di["keys"]) if db and di else None
        nf = len(df["keys"] - di["keys"]) if df and di else None
        same = "" if (db and df and db["by"] == df["by"] and cb == cf) else "  <-- differs"
        print(f"  {b:28s} DRC {db['n'] if db else '?':>5} -> {df['n'] if df else '?':<5} "
              f"new shorts {nb} -> {nf}  conn {cb} -> {cf}  time {rb['secs']} -> {rf['secs']}s{same}")
        for k, v in (("base_new_shorts", nb), ("fix_new_shorts", nf), ("base_conn", cb), ("fix_conn", cf),
                     ("base_s", rb["secs"]), ("fix_s", rf["secs"])):
            tot[k] += v or 0
    print("TOTAL", tot)
    if all(done.values()) and not a.keep_app:
        subprocess.run(["modal", "app", "stop", meta["app"], "-y"], capture_output=True)
        print(f"all boards in: stopped {meta['app']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    s = sp.add_parser("select")
    s.add_argument("--max-recorded-s", type=float, default=7200)
    s.add_argument("--jobs", type=int, default=3)
    l = sp.add_parser("launch")
    l.add_argument("--label", required=True)
    l.add_argument("--only", default="", help="comma-separated board names (a smoke test)")
    c = sp.add_parser("collect")
    c.add_argument("--label", required=True)
    c.add_argument("--keep-app", action="store_true")
    a = ap.parse_args()
    {"select": cmd_select, "launch": cmd_launch, "collect": cmd_collect}[a.cmd](a)


if __name__ == "__main__":
    main()
