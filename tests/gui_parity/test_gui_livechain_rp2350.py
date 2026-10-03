#!/usr/bin/env python3
"""GRADE-parity gate on a set11-class board (rp2350_fpga_eensy).

The copper-identity harness (test_gui_engine_parity.py) proved GUI-vs-CLI on a
route/planes/repair chain but measures copper-overlap %, which on a chaotic
rip-up router diverges even when both fronts grade clean (#362). The invariant
that actually matters is GRADE parity: the GUI must not introduce DRC the CLI
doesn't.

This gate chains the rp2350 PLANE sub-chain on ONE live pcbnew board --
exactly as the Claude-tab plan executor does, in-memory across steps --
starting from the recorded CLI pre-plane board, and asserts that no stage, on
either front, INTRODUCES DRC. Each stage is graded against its own project,
and the INPUT is graded under that same project: what the input already had
is counted apart and printed, never blamed on the chain (rp2350's own
Net-(D2-K) and SWDIO tracks sit ~0.06 mm from J2's NPTH hole, under both the
input's stock 0.25 mm and the chain's 0.1 mm min_hole_clearance). On this
front headless_plan saves the per-step GUI snapshots WITHOUT a project
(aSkipSettings -- the live floors sit in the board's in-memory settings until
the plan ends), so the GUI leg, and its input re-grade, get no project rule.

RESHAPED for #562 (pours-first). The chain used to be create -> repair ->
reconnect route -> repair2, and this test still carried that shape after the
architecture change: the executor skips `repair_planes` steps as no-ops
while the CLI leg still shelled route_disconnected_planes.py, so the GUI leg
ran TWO real stages and the CLI leg FOUR -- the per-stage table compared
different chains and its "parity" meant nothing. Both legs are now the
current architecture: pour -> ONE route step whose in-run plane finalize
does the weld/repair/oracle. The plane nets ride in the route step's net
list (NOT only in the pour): the finalize filters its zone-net scope by the
route's --nets, so a route naming only the signal nets would exclude the
pours from the finalize BY PLAN, and under #562 the pour alone connects
nothing (it places no taps -- the route step's pour-launch is the weld).

MIGRATED off the shim harness (2026-07-26). The GUI leg used to bind real tab
methods onto plain shim objects and hand-build the engine config, which has a
structural blind spot: anything between a dialog CONTROL and the engine
argument never executes. That is not hypothetical -- the same shim style made
test_gui_engine_parity report a phantom 73-segment plane-tap "divergence" on
splitflap that does not exist in the real GUI (the shim never ran
_effective_track_width(), so it passed defaults.TRACK_WIDTH 0.3 where the real
dialog resolves the board's 0.127). Now it runs the REAL headless
swig_gui.RoutingDialog driven by the REAL ai_plan.PlanExecutor, via
replay_plan_vs_run.replay() -- the same machinery the corpus driver uses, which
only needs {'input_board': path}, so it works on a checked-in board.

It caught the swig_gui route-apply width-rounding bug (0.0762 -> 0.076 fab-floor
violations, 42 of them at the reconnect route step; #362). Per-step isolation
on CLI inputs did NOT catch it -- only chaining on a live board did, because
the bug rides the GUI's in-memory apply path.

The pre-plane input board (rp2350_fpga_eensy_prePlane.kicad_pcb, the recorded
step4b_retry) is checked into kicad_files/, so the gate is self-contained.
Needs KiCad's python (pcbnew); skips (exit 0) if pcbnew is absent.
Run: python3 tests/gui_parity/test_gui_livechain_rp2350.py
"""

# ---------------------------------------------------------------------------
# macOS: if this HANGS at ~0% CPU, it is NOT wx, machine load, or a deadlock.
#
# After any wx process here is killed (a pkill, a timeout, a crash), macOS
# decides the app "quit unexpectedly", and the NEXT headless launch stops inside
# NSApplication bootstrap showing the restore-windows alert you cannot see:
#     -[NSPersistentUIRestorer promptToIgnorePersistentState]
#         -> -[NSAlert runModal]
# Headless, nobody can click it, so it waits forever: process state SN accruing
# ~0.3s of CPU over many minutes, which reads exactly like a hang. This cost a
# full session of ".gui-parity-checked" markers recording "wx blocked, gate NOT
# RUN" -- the gates were fine the whole time.
#
#   diagnose:  sample <pid> 3 -mayDie | grep -E "NSAlert|PersistentUI"
#   fix:       defaults write -g ApplePersistenceIgnoreState -bool YES
#
# A sandboxed HOME does NOT help -- cfprefsd serves that pref per-user
# regardless of HOME. With the default set, test_gui_engine_parity.py runs ~90s.
# ---------------------------------------------------------------------------
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
START_BOARD = os.path.join(REPO, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')

# Every versioned install, newest first by NUMERIC version (a string sort
# puts KiCad\9.0 above KiCad\10.0).
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import pcbnew'],
                              capture_output=True).returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    # os.execv re-splits argv on spaces on Windows, and the
                    # interpreter lives under "Program Files".
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


#: path -> the violation RECORDS the chain introduced at that stage, so a
#: non-zero stage can say WHAT it found. The gate used to report a bare count
#: and then delete the workdir, which left "GUI 9 / CLI 0" as a number with no
#: way to act on it short of re-running the whole 6-minute chain.
_INTRODUCED = {}
#: path -> how many of its violations the INPUT already had (see _grade).
_INHERITED = {}


def _drc_items(pcb, clr, baseline=True):
    """check_drc's violation records for `pcb` (its `--json` `items`), graded
    against the board's own sibling project. None when the grade did not run.

    --baseline the start board (#962): rp2350 ships 27 of its own vias in
    paste openings, unprotected; they are the input's, not the chain's.
    """
    js = pcb + '.drc_items.json'
    # THIS interpreter (KiCad's), as the CLI legs run. A 'python3' child of
    # KiCad's python on Windows inherits its PYTHONUSERBASE (KiCad's 3rdparty
    # dir), loses its own user site and cannot import numpy: every grade read
    # -1 and the gate FAILED on both legs, whatever the copper.
    cmd = [sys.executable, os.path.join(REPO, 'py_router', 'check_drc.py'), pcb,
           '--clearance', str(clr), '--hole-to-hole-clearance', '0.2',
           '--clearance-margin', '0.1', '--json', js]
    if baseline:
        cmd += ['--baseline', START_BOARD]
    subprocess.run(cmd, capture_output=True, text=True)
    try:
        with open(js, encoding='utf-8') as fh:
            items = json.load(fh)['items']
    except Exception:                                           # noqa: BLE001
        return None
    # `items` also lists what --baseline ACCEPTED (the record carries an
    # `accepted` reason); those are not violations, and counting them was the
    # first cut's bug (27 "introduced" via-in-paste the input already had).
    return [i for i in items if not i.get('accepted')]


def _item_key(item):
    """A violation record's identity: every field, floats to 0.1 um. The same
    copper graded under the same rules gives the same key; a violation that
    MOVED -- a rerouted track, a shifted via -- gives a new one."""
    def norm(v):
        if isinstance(v, float):
            return round(v, 4)
        if isinstance(v, (list, tuple)):
            return [norm(x) for x in v]
        return v
    return json.dumps({k: norm(v) for k, v in item.items()}, sort_keys=True)


def _inherited_keys(stage_pcb, clr):
    """The violations the INPUT already had, graded under `stage_pcb`'s OWN
    project (`.kicad_pro` / `.kicad_dru`).

    Under the rules the chain wrote, copper the chain never touched can
    violate: rp2350's Net-(D2-K) and SWDIO tracks sit ~0.06 mm from J2's NPTH
    hole, under the plane step's 0.1 mm min_hole_clearance AND under the
    input's own stock 0.25. That is the fixture, not either front, and the
    gate's question is what the CHAIN did. The input is re-graded per stage
    because each stage's project can differ.
    """
    src = _STAGED['board']
    d = os.path.join(os.path.dirname(stage_pcb), 'inherited')
    os.makedirs(d, exist_ok=True)
    stem = os.path.splitext(os.path.basename(stage_pcb))[0]
    dst = os.path.join(d, stem + '_input.kicad_pcb')
    shutil.copy(src, dst)
    for ext in ('.kicad_pro', '.kicad_dru'):
        s = os.path.splitext(stage_pcb)[0] + ext
        if os.path.isfile(s):
            shutil.copy(s, os.path.splitext(dst)[0] + ext)
    items = _drc_items(dst, clr, baseline=False)
    return None if items is None else {_item_key(i) for i in items}


def _grade(pcb, clr=0.09):
    """How many violations the chain INTRODUCED at this stage (-1: the grade
    did not run). What the input already had under the same rules is counted
    apart, in _INHERITED, and disclosed rather than blamed on the chain."""
    items = _drc_items(pcb, clr)
    inherited = _inherited_keys(pcb, clr)
    if items is None or inherited is None:
        return -1
    new = [i for i in items if _item_key(i) not in inherited]
    _INTRODUCED[pcb] = new
    _INHERITED[pcb] = len(items) - len(new)
    return len(new)


def _self_test():
    """The subtraction, in milliseconds, before the 6-minute chain: an
    inherited record is excused, and the SAME violation anywhere else --
    the negative control -- is not."""
    a = {'type': 'track-hole', 'net2': 'Net-(D2-K)', 'hole_loc': [151.04, 96.0],
         'seg_loc': [150.3, 95.75, 150.35, 95.7], 'overlap_mm': 0.0379038}
    moved = dict(a, seg_loc=[150.31, 95.75, 150.36, 95.7])
    noisy = dict(a, overlap_mm=0.0379038 + 1e-9)
    inherited = {_item_key(a)}
    ok = (_item_key(noisy) in inherited
          and _item_key(moved) not in inherited
          and len([i for i in (a, moved) if _item_key(i) not in inherited]) == 1)
    if not ok:
        print("FAIL: the inherited-violation subtraction is broken "
              "(self-test); the stage grades below could not be trusted.")
    return ok


def _print_violations(tag, pcb, limit=20):
    """The violations a failing stage INTRODUCED, before the workdir goes."""
    new = _INTRODUCED.get(pcb) or []
    print(f"  --- {tag}: {len(new)} violation(s) introduced in "
          f"{os.path.basename(pcb)} ---")
    for it in new[:limit]:
        where = {k: v for k, v in it.items() if k.endswith('_loc') or k == 'layer'}
        print(f"    {it.get('type')}: {it.get('net1', '')} <-> "
              f"{it.get('net2', '')} {where}")
    if len(new) > limit:
        print(f'    ... {len(new) - limit} more')


def _cli_chain(work):
    """Run the EQUIVALENT CLI file chain and grade each stage.

    #495: this gate used to grade only the GUI stages and then assert, in its
    failure message, that "the CLI file chain does not" introduce the DRC --
    without ever running the CLI. That let a CLI-side defect (8 track-through-
    NPTH-hole violations from the plane repair's oracle recheck) sit green here
    while the board it grades was demonstrably dirty. Measure both fronts.

    Runs under THIS interpreter (sys.executable, i.e. KiCad's python when the
    gate re-execs into it) so the comparison is not contaminated by the
    interpreter-dependent routing #493 fixed.
    """
    py = sys.executable
    b0 = os.path.join(work, 'cli_start.kicad_pcb')
    shutil.copy(_STAGED['board'], b0)
    for ext in ('.kicad_pro', '.kicad_dru'):
        s = os.path.splitext(_STAGED['board'])[0] + ext
        if os.path.isfile(s):
            shutil.copy(s, os.path.splitext(b0)[0] + ext)
    layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu']
    planes = os.path.join(work, 'cli_planes.kicad_pcb')
    final = os.path.join(work, 'cli_final.kicad_pcb')
    # py_router/, not the repo root (#522 reorg): the CLI scripts moved, and
    # this leg silently became "python3 <missing file>" -> rc=2 -> no output ->
    # every CLI stage graded -1 and the gate FAILED on the CLI leg alone.
    R = lambda s: os.path.join(REPO, 'py_router', s)
    # Mirrors the GUI stages below: the #562 chain -- a bare pour, then ONE
    # route step covering the reconnect nets AND the plane nets, whose in-run
    # finalize is the weld/repair/oracle (route_disconnected_planes.py is no
    # longer a chain step). grid_step 0.025 is the recorded reconnect grid;
    # the finalize inherits it.
    steps = [
        ('create', planes, [py, '-X', 'utf8', R('route_planes.py'), b0, planes,
                            '--nets', 'GND', '+3V3',
                            '--plane-layers', 'In1.Cu', 'In4.Cu',
                            '--via-size', '0.45', '--via-drill', '0.2',
                            '--track-width', '0.09', '--clearance', '0.10',
                            '--hole-to-hole-clearance', '0.2', '--grid-step', '0.05',
                            '--power-nets', 'VIN', '--power-nets-widths', '0.3']),
        ('route', final, [py, '-X', 'utf8', R('route.py'), planes, final,
                          '--nets', '+1V1', '/T8F49I2X/PIN.5', 'GND', '+3V3',
                          '--layers'] + layers + [
                          '--no-bga-zones', '--clearance', '0.09',
                          '--track-width', '0.0762', '--via-size', '0.25',
                          '--via-drill', '0.15', '--hole-to-hole-clearance', '0.2',
                          '--grid-step', '0.025', '--max-ripup', '10',
                          '--max-iterations', '1000000']),
    ]
    grades = {}
    for tag, out, cmd in steps:
        p = subprocess.run(cmd, capture_output=True, text=True, cwd=work)
        if not os.path.exists(out):
            print(f"  CLI stage {tag} produced no output (rc={p.returncode})")
            print((p.stdout or '')[-1500:])
            print((p.stderr or '')[-1500:])
            grades[tag] = -1
            break
        grades[tag] = _grade(out)
        _CLI_OUTS[tag] = out
    return grades


# The GUI leg as a real Claude-tab PLAN -- the same JSON shape manifest_to_plan
# emits and the plan executor consumes. Mirrors _cli_chain() step for step.
# No `repair_planes` steps: the executor skips them as #562 no-ops, and
# carrying them here while the CLI leg shelled route_disconnected_planes.py
# is exactly the misalignment this reshape removes. The route step names the
# plane nets alongside the reconnect nets -- see the module docstring.
PLANE_ASSIGNMENTS = [{'nets': ['GND'], 'layer': 'In1.Cu'},
                     {'nets': ['+3V3'], 'layer': 'In4.Cu'}]
_GP = {'power_nets': ['VIN'], 'power_nets_widths': [0.3],
       'hole_to_hole_clearance': 0.2}
STAGE_TAGS = ['create', 'route']
PLAN = [
    {'action': 'route_planes',
     'params': dict(via_size=0.45, via_drill=0.2, clearance=0.10,
                    track_width=0.09, grid_step=0.05, **_GP),
     'assignments': PLANE_ASSIGNMENTS},
    {'action': 'route',
     'params': dict(clearance=0.09, track_width=0.0762, via_size=0.25,
                    via_drill=0.15, grid_step=0.025, max_ripup=10,
                    max_iterations=1000000, no_bga_zone=True,
                    hole_to_hole_clearance=0.2,
                    layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu']),
     'nets': ['+1V1', '/T8F49I2X/PIN.5', 'GND', '+3V3']},
]


# The staged (project-carrying) input board, shared by both legs. Set by
# main(); _cli_chain reads it so both legs start from the SAME bytes.
_STAGED = {}

#: stage tag -> the board each leg graded, for _print_violations.
_GUI_SNAPS = {}
_CLI_OUTS = {}


def main():
    start_board = START_BOARD
    if not os.path.exists(start_board):
        print(f"SKIP: checked-in board not found at {start_board}")
        return 0
    if not _self_test():
        return 1

    # The REAL headless dialog + REAL PlanExecutor. replay() touches `info` only
    # for input_board, so the corpus driver works unchanged on a repo board.
    import replay_plan_vs_run as R

    work = tempfile.mkdtemp(prefix='rp2350_livechain_')
    # KICAD_LIVECHAIN_KEEP=1: leave the workdir behind. Both legs'
    # boards at every stage are the only way to answer WHY a stage
    # diverged, and re-running to get them back costs ~6 minutes.
    keep = bool(os.environ.get('KICAD_LIVECHAIN_KEEP'))
    _rm = ((lambda *a, **k: print(f'  (kept: {work})')) if keep
           else shutil.rmtree)

    # Stage the input WITH a sibling .kicad_pro (the checked-in fixture has
    # none). A project-less board makes the two fronts legitimately diverge:
    # the CLI seeds a minimal project pinned to the fab floors while the live
    # pcbnew board carries KiCad's stock defaults, so the two legs would
    # grade against DIFFERENT floors -- measuring the fixture, not the
    # engines (the copper-parity gate hit exactly this; see stage_board in
    # test_gui_engine_parity). pcbnew authors the project itself. We run
    # under KiCad's python here (the gate re-execs), so pcbnew is available.
    staged = os.path.join(work, 'staged_start.kicad_pcb')
    src_pro = os.path.splitext(start_board)[0] + '.kicad_pro'
    if os.path.isfile(src_pro):
        shutil.copy(start_board, staged)
        shutil.copy(src_pro, os.path.splitext(staged)[0] + '.kicad_pro')
    else:
        import pcbnew
        pcbnew.SaveBoard(staged, pcbnew.LoadBoard(start_board))
        print("staged the input WITH a KiCad-authored .kicad_pro "
              "(the fixture has none)")
    _STAGED['board'] = staged
    print(f"running the GUI plan through the real dialog ({len(PLAN)} steps)...",
          flush=True)
    res = R.replay({'input_board': staged}, PLAN, work, snapshots=True)
    if res.get('aborted'):
        print(f"FAIL: GUI plan aborted: {res['aborted']}")
        _rm(work, ignore_errors=True)
        return 1
    if res.get('completed', 0) != len(PLAN):
        print(f"FAIL: GUI plan ran {res.get('completed')} of {len(PLAN)} steps.")
        _rm(work, ignore_errors=True)
        return 1

    # replay() snapshots each completed step as gui_stepNN.kicad_pcb.
    stages = {}
    for i, tag in enumerate(STAGE_TAGS, 1):
        snap = os.path.join(work, f'gui_step{i:02d}.kicad_pcb')
        if not os.path.exists(snap):
            print(f"FAIL: no GUI snapshot for stage {tag}")
            _rm(work, ignore_errors=True)
            return 1
        stages[tag] = _grade(snap)
        _GUI_SNAPS[tag] = snap

    # #495: actually RUN the CLI chain instead of asserting it is clean.
    print("\nrunning the equivalent CLI file chain for comparison...", flush=True)
    cli = _cli_chain(work)

    print("\nrp2350 live-chain grade parity (DRC @ 0.09), violations the chain "
          "INTRODUCED (+ the input's own, under the same stage's rules):")
    print(f"  {'stage':<12} {'GUI':>10} {'CLI':>10}")
    gui_bad, cli_bad = [], []
    for tag, n in stages.items():
        c = cli.get(tag, -1)
        gi = _INHERITED.get(_GUI_SNAPS.get(tag, ''), 0)
        ci = _INHERITED.get(_CLI_OUTS.get(tag, ''), 0)
        print(f"  {tag:<12} {n:>4} (+{gi:<3}) {c:>4} (+{ci:<3})  "
              f"[{'OK' if n == 0 else 'FAIL'}/{'OK' if c == 0 else 'FAIL'}]")
        if n != 0:
            gui_bad.append(tag)
        if c != 0:
            cli_bad.append(tag)
    # Say WHAT each failing stage found, on both legs, while the boards still
    # exist. A divergence is a question about violation CLASSES -- the same
    # count from different causes is a different bug -- and the answer was
    # being deleted three lines later.
    for tag in gui_bad:
        _print_violations('GUI ' + tag, _GUI_SNAPS.get(tag, ''))
    for tag in cli_bad:
        _print_violations('CLI ' + tag, _CLI_OUTS.get(tag, ''))
    _rm(work, ignore_errors=True)

    rc = 0
    if cli_bad:
        # Measured, not assumed: a CLI-side defect fails this gate on its own
        # (#495 defect 1 -- the plane repair's oracle recheck shipped 8 GND
        # straps through J1's NPTH mounting hole at an unvalidated width).
        print(f"\nFAIL: the CLI file chain introduced DRC at stage(s) {cli_bad}.")
        rc = 1
    if gui_bad:
        extra = [t for t in gui_bad if stages[t] > cli.get(t, 0)]
        print(f"\nFAIL: GUI live-chain introduced DRC at stage(s) {gui_bad}"
              + (f"; worse than the CLI at {extra} (#362)." if extra else "."))
        rc = 1
    if rc:
        return rc
    print("\nPASS: neither the GUI nor the CLI chain introduces DRC at any "
          "stage (the input's own violations, graded under each stage's "
          "rules, are counted apart above).")
    return 0


if __name__ == "__main__":
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(main())
