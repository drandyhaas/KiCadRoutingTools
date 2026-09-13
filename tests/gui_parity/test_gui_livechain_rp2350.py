#!/usr/bin/env python3
"""GRADE-parity gate on a set11-class board (rp2350_fpga_eensy).

The copper-identity harness (test_gui_engine_parity.py) proved GUI-vs-CLI on a
route/planes/repair chain but measures copper-overlap %, which on a chaotic
rip-up router diverges even when both fronts grade clean (#362). The invariant
that actually matters is GRADE parity: the GUI must not introduce DRC the CLI
doesn't.

This gate chains the rp2350 PLANE sub-chain on ONE live pcbnew board --
exactly as the Claude-tab plan executor does, in-memory across steps --
starting from the recorded CLI pre-plane board, and asserts every stage
grades 0 DRC like the CLI file chain.

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
import os
import re
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

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import wx, kipy, pcbnew'],
                              capture_output=True).returncode == 0:
                os.execv(cand, [cand, os.path.abspath(__file__)] + sys.argv[1:])
    print("SKIP: no python with wx + kipy + pcbnew found")
    sys.exit(0)


#: path -> the check_drc stdout that produced its grade, so a non-zero stage
#: can say WHAT it found. The gate used to report a bare count and then delete
#: the workdir, which left "GUI 9 / CLI 0" as a number with no way to act on
#: it short of re-running the whole 6-minute chain with a patched copy.
_REPORTS = {}


def _grade(pcb, clr=0.09):
    r = subprocess.run(['python3', os.path.join(REPO, 'py_router', 'check_drc.py'), pcb,
                        '--clearance', str(clr), '--hole-to-hole-clearance', '0.2',
                        '--clearance-margin', '0.1'], capture_output=True, text=True)
    _REPORTS[pcb] = r.stdout
    m = re.search(r'FOUND (\d+) DRC', r.stdout)
    return 0 if 'NO DRC' in r.stdout else (int(m.group(1)) if m else -1)


def _print_violations(tag, pcb, limit=20):
    """The violation lines behind a non-zero stage, before the workdir goes.

    check_drc prints a header line per violation class and then one line per
    violation; everything before the FOUND banner is progress chatter, so the
    banner is the anchor rather than a pattern for the lines themselves --
    which differ per class and would drift.
    """
    out = _REPORTS.get(pcb, '')
    idx = out.find('FOUND ')
    if idx < 0:
        return
    body = [ln for ln in out[idx:].splitlines() if ln.strip()]
    print(f"  --- {tag}: what check_drc found in {os.path.basename(pcb)} ---")
    for ln in body[:limit]:
        print('    ' + ln)
    if len(body) > limit:
        print(f'    ... {len(body) - limit} more line(s)')


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

    print("\nrp2350 live-chain grade parity (DRC @ 0.09):")
    print(f"  {'stage':<12} {'GUI':>6} {'CLI':>6}")
    gui_bad, cli_bad = [], []
    for tag, n in stages.items():
        c = cli.get(tag, -1)
        print(f"  {tag:<12} {n:>6} {c:>6}   "
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
    print("\nPASS: GUI and CLI chains both grade clean at every stage.")
    return 0


if __name__ == "__main__":
    # Probe ALL THREE, not kipy alone. This gate needs wx (the real dialog),
    # kipy (the adapter) AND pcbnew: main() stages its project-less fixture with a pcbnew-authored `.kicad_pro`.
    # With `kipy` as the only condition, an ordinary python3 that happens to
    # have kicad-python installed -- which is any machine where someone ran
    # `pip install kicad-python` -- satisfies the guard, never re-execs, and
    # dies later on `ModuleNotFoundError: No module named 'pcbnew'`. That is
    # the flagship parity gate reporting an ENVIRONMENT accident with the same
    # exit code as a real divergence, on the machine most likely to be able to
    # run it.
    try:
        import wx, kipy, pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(main())
