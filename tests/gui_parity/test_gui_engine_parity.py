#!/usr/bin/env python3
"""Headless GUI-vs-CLI engine parity harness (claude-tab plan execution).

The claude tab's promise: "run selected steps" in the GUI should leave the
live board in the same state as running the stress chain's CLI steps on the
board FILE and loading the final output. This harness measures that promise
on a real board without pressing buttons:

  CLI leg   -- runs the recorded chain's commands file->file (subprocesses),
               exactly like a stress replay.
  GUI leg   -- GUI_PLAN below, run through the REAL headless
               swig_gui.RoutingDialog driven by the REAL
               ai_plan.PlanExecutor (via replay_plan_vs_run.replay), on ONE
               live board carried in memory across steps -- the same
               parse_plan_result -> reset_params_to_defaults ->
               apply_step_params -> tab._on_*() path the buttons run.

MIGRATED off the shim harness (2026-07-26). The GUI leg used to bind real tab
methods onto plain shim objects (_Shim/_Stub/_borrow) and hand-build the engine
config. That has a structural blind spot -- **anything between a dialog CONTROL
and the engine argument never executes** -- and it did not just miss bugs, it
INVENTED one: because the shim hand-built the plane config it never ran
_effective_track_width(), so the engine's own
`config.get('track_width', defaults.TRACK_WIDTH)` fallback supplied 0.3 where
the real dialog resolves the board's Default net class to 0.127, and this gate
reported 73 phantom "divergent" GND plane-tap segments that do not exist in the
real GUI. Nothing is mirrored now, so a converter/apply bug shows up as a board
difference instead of being hidden or fabricated.

Both finals are then graded (check_connected / check_drc / kicad-cli DRC
unconnected) and the divergence is printed. This harness REPORTS the gap; it
is not (yet) a pass/fail gate -- known deliberate divergences exist (the
CLI mains' kicad-oracle recheck, clean_plane_copper, end-of-run
reconciliation, .kicad_pro floor carryover, and plan-parameter whitelist).

Needs pcbnew; re-execs into KiCad's python automatically.

    python3 tests/gui_parity/test_gui_engine_parity.py [board.kicad_pcb] [--workdir DIR]
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
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))  # replay_plan_vs_run

# KiCad 9/10 install under a VERSIONED directory on Windows
# (C:\Program Files\KiCad\10.0\bin), which a bare .../KiCad/bin path misses.
# Shared with the engine (#647) so the layouts are described once -- and so
# this list gets the NUMERIC version sort: the local `sorted(..., reverse=True)`
# this replaced ordered "9.0" above "10.0" and handed a dual-install machine
# its older KiCad. The loop below still does its own pcbnew verification.
from kicad_exact_fill import kicad_python_candidates  # noqa: E402

KICAD_PYTHONS = kicad_python_candidates()


def _host_env():
    """Environment for the CLI subprocesses, with KiCad's python wiring removed.

    KiCad's bundled interpreter exports PYTHONUSERBASE (its own 3rdparty tree).
    Once this gate re-execs into it, a plain `python3 py_router/route.py` child inherits
    that and resolves USER site-packages inside KiCad's tree instead of its own,
    so the host interpreter reports numpy/scipy/shapely missing and every CLI
    step fails -- which reads as "no wx/pcbnew in this session" when in fact both
    are present and working.
    """
    return {k: v for k, v in os.environ.items()
            if k not in ('PYTHONUSERBASE', 'PYTHONHOME', 'PYTHONPATH')}

DEFAULT_BOARD = os.path.join(REPO, "kicad_files", "splitflap_driver.kicad_pcb")

# The splitflap chain is expressed twice below: run_cli_leg() as the recorded
# stress commands, GUI_PLAN as the equivalent Claude-tab plan. They must stay
# in step with each other.


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable:
            continue
        if os.path.exists(cand):
            r = subprocess.run([cand, '-c', 'import wx, kipy'],
                               capture_output=True)
            if r.returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    # os.execv goes through the CRT on Windows and re-splits the
                    # argument vector on spaces, so a path like
                    # "C:\Program Files\KiCad\10.0\bin\python.exe" is torn in two.
                    # subprocess quotes it correctly.
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("ERROR: no python with wx + kipy found")
    sys.exit(2)


def stage_board(board, workdir):
    """Copy the board into the workdir, ENSURING it has a sibling .kicad_pro.

    A board with no project is the one input that makes these two fronts
    legitimately disagree, and it is not a product bug -- it is the #441
    hazard CLAUDE.md warns about, landing on a test fixture:

      * the CLI has no project to read, so fix_project_for_output SEEDS a
        minimal one and pins the fab floors into it (copper-to-edge 0.2);
      * the GUI works on a live pcbnew board, which always carries KiCad's
        own defaults (copper-to-edge 0.5), and its write-back only ever
        LOOSENS, so it never tightens to the fab floor.

    A 0.3mm difference in the copper-to-edge rule is not cosmetic: the plane
    step insets the pour OUTLINE by it, which changes the exact fill, which
    feeds the plane-fragility cost field, which re-prices the A* -- measured
    on splitflap_driver as 12 cli-only / 8 gui-only GND segments and a
    relocated via, with both boards still DRC- and connectivity-clean.
    Give the same board a project and the copper sets are IDENTICAL.

    Every real board (and every corpus board) carries a project, so grading
    the project-less case measured the fixture, not the engines. pcbnew
    authors the project itself -- KiCad's own defaults, nothing hardcoded
    here.
    """
    dst = os.path.join(workdir, 'src_' + os.path.basename(board))
    src_pro = os.path.splitext(board)[0] + '.kicad_pro'
    if os.path.isfile(src_pro):
        shutil.copy(board, dst)
        shutil.copy(src_pro, os.path.splitext(dst)[0] + '.kicad_pro')
        for ext in ('.kicad_dru',):
            s = os.path.splitext(board)[0] + ext
            if os.path.isfile(s):
                shutil.copy(s, os.path.splitext(dst)[0] + ext)
        return dst
    import pcbnew
    pcbnew.SaveBoard(dst, pcbnew.LoadBoard(board))
    print(f"  staged {os.path.basename(board)} WITH a KiCad-authored "
          f".kicad_pro (the source had none -- see stage_board)")
    return dst


def run_cli_leg(board, workdir):
    py = shutil.which('python3') or sys.executable
    steps = []
    s1 = os.path.join(workdir, 'cli_step1.kicad_pcb')
    steps.append([py, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route.py'), board, s1,
                  '--nets', '*', '!GND', '--clearance', '0.15',
                  '--track-width', '0.127', '--via-size', '0.45',
                  '--via-drill', '0.2', '--power-nets', '+3V3', '+12V',
                  '--power-nets-widths', '0.4', '0.4',
                  '--max-ripup', '10', '--max-iterations', '1000000'])
    s2 = os.path.join(workdir, 'cli_step2.kicad_pcb')
    steps.append([py, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route_planes.py'), s1, s2,
                  '--nets', 'GND', '--plane-layers', 'B.Cu',
                  '--clearance', '0.15', '--via-size', '0.45',
                  '--via-drill', '0.2'])
    # (No standalone repair step since #562: the final route step's in-run
    # plane finalize runs the repair engine + cleanup + kicad-oracle verify.)
    s4 = os.path.join(workdir, 'cli_final.kicad_pcb')
    steps.append([py, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route.py'), s2, s4,
                  '--nets', 'GND', '--clearance', '0.127',
                  '--track-width', '0.127', '--via-size', '0.45',
                  '--via-drill', '0.2', '--max-ripup', '10',
                  '--max-iterations', '1000000'])
    for i, cmd in enumerate(steps):
        print(f"[cli] step {i + 1}/{len(steps)} ...", flush=True)
        r = subprocess.run(cmd, capture_output=True, text=True, cwd=workdir,
                           env=_host_env())
        if r.returncode != 0:
            print(r.stdout[-3000:])
            print(r.stderr[-2000:])
            raise RuntimeError(f"CLI step {i + 1} failed")
    return s4


# The GUI leg as a real Claude-tab PLAN -- same JSON shape manifest_to_plan
# emits and the plan executor consumes. Mirrors run_cli_leg() step for step.
#
# NOTE what is deliberately ABSENT: the plane step carries no track_width,
# exactly like its CLI command (`route_planes.py ... --clearance 0.15
# --via-size 0.45 --via-drill 0.2`, no --track-width). Both fronts must then
# resolve it from the board's Default net class. The OLD shim harness could not
# express that -- it hand-built the config, so the engine's own
# `config.get('track_width', defaults.TRACK_WIDTH)` fallback supplied 0.3 while
# the CLI resolved 0.127, and this gate reported 73 phantom "divergent" GND
# plane-tap segments for it. The real dialog runs _effective_track_width(),
# which reads the board's Default class when the override checkbox is
# unchecked, and lands on 0.127 like the CLI.
GUI_PLAN = [
    {'action': 'route',
     'params': dict(clearance=0.15, track_width=0.127, via_size=0.45,
                    via_drill=0.2, power_nets=['+3V3', '+12V'],
                    power_nets_widths=[0.4, 0.4], max_ripup=10,
                    max_iterations=1000000),
     'nets': ['*', '!GND']},
    {'action': 'route_planes',
     'params': dict(clearance=0.15, via_size=0.45, via_drill=0.2),
     'assignments': [{'nets': ['GND'], 'layer': 'B.Cu'}]},
    {'action': 'route',
     'params': dict(clearance=0.127, track_width=0.127, via_size=0.45,
                    via_drill=0.2, max_ripup=10, max_iterations=1000000),
     'nets': ['GND']},
]


def run_gui_leg(board_path, workdir):
    """Run GUI_PLAN through the REAL headless RoutingDialog + PlanExecutor.

    replay() touches its `info` argument only for input_board, so the corpus
    driver runs unchanged on a checked-in board.
    """
    import replay_plan_vs_run as R
    res = R.replay({'input_board': board_path}, GUI_PLAN, workdir,
                   snapshots=True)
    if res.get('aborted'):
        raise RuntimeError(f"GUI plan aborted: {res['aborted']}")
    if res.get('completed', 0) != len(GUI_PLAN):
        raise RuntimeError(f"GUI plan ran {res.get('completed')} of "
                           f"{len(GUI_PLAN)} steps")
    out = os.path.join(workdir, 'gui_final.kicad_pcb')
    shutil.copy(res['live_board'], out)
    return out


def grade(pcb, label):
    py = shutil.which('python3') or sys.executable
    conn = subprocess.run([py, '-X', 'utf8',
                           os.path.join(REPO, 'py_router', 'check_connected.py'), pcb],
                          capture_output=True, text=True)
    conn_full = 'ALL NETS FULLY CONNECTED' in conn.stdout
    import re
    m = re.search(r'FOUND (\d+) ISSUES', conn.stdout)
    conn_issues = int(m.group(1)) if m else 0
    drc = subprocess.run([py, '-X', 'utf8', os.path.join(REPO, 'py_router', 'check_drc.py'),
                          pcb, '--clearance-margin', '0.1', '-c', '0.127'],
                         capture_output=True, text=True)
    m = re.search(r'FOUND (\d+) DRC', drc.stdout)
    drc_n = 0 if 'NO DRC' in drc.stdout else (int(m.group(1)) if m else -1)
    kicad_n = -1
    for cand in KICAD_PYTHONS[:1]:
        cli = cand.replace(
            'Frameworks/Python.framework/Versions/Current/bin/python3',
            'MacOS/kicad-cli')
        if os.path.exists(cli):
            out = pcb + '.drc.json'
            subprocess.run([cli, 'pcb', 'drc', pcb, '--format', 'json', '-o',
                            out, '--severity-all', '--refill-zones'],
                           capture_output=True)
            try:
                import json
                kicad_n = len(json.load(open(out)).get('unconnected_items', []))
            except Exception:
                pass
    print(f"  {label:10s} conn_full={conn_full} conn_issues={conn_issues} "
          f"drc={drc_n} kicad_unconnected={kicad_n}")
    return dict(conn_full=conn_full, conn_issues=conn_issues, drc=drc_n,
                kicad=kicad_n)


def compare_copper(cli_pcb, gui_pcb):
    """Canonical copper-set comparison: segments as (net NAME, layer,
    sorted rounded endpoints, width), vias as (net, pos, size, drill).
    UUIDs are per-run random, so byte comparison is meaningless by design;
    set equality is the strongest meaningful identity bar."""
    from kicad_parser import parse_kicad_pcb

    def canon(path, dp=3):
        pcb = parse_kicad_pcb(path)
        names = {nid: net.name for nid, net in pcb.nets.items()}
        segs = set()
        for s in pcb.segments:
            a = (round(s.start_x, dp), round(s.start_y, dp))
            b = (round(s.end_x, dp), round(s.end_y, dp))
            segs.add((names.get(s.net_id, s.net_id), s.layer,
                      min(a, b), max(a, b), round(s.width, dp)))
        vias = set()
        for v in pcb.vias:
            vias.add((names.get(v.net_id, v.net_id), round(v.x, dp),
                      round(v.y, dp), round(v.size, dp), round(v.drill, dp)))
        return segs, vias

    s1, v1 = canon(cli_pcb)
    s2, v2 = canon(gui_pcb)
    print(f"\n=== COPPER-SET COMPARISON (UUID-independent) ===")
    print(f"  segments: CLI={len(s1)} GUI={len(s2)} common={len(s1 & s2)} "
          f"cli-only={len(s1 - s2)} gui-only={len(s2 - s1)}")
    print(f"  vias:     CLI={len(v1)} GUI={len(v2)} common={len(v1 & v2)} "
          f"cli-only={len(v1 - v2)} gui-only={len(v2 - v1)}")
    # SUB-MICRON CHECK. The sets above canonicalize at 3 dp = 1 um, so a
    # QUANTISATION fork -- both fronts routing the same path but writing the
    # coordinate differently -- is 1000x below this gate's resolution and reads
    # as IDENTICAL. That is not hypothetical: the IPC adapter built every
    # coordinate with kipy's `Vector2.from_xy_mm`, whose `from_mm` is a bare
    # `int(mm * 1_000_000)`, so 66.1mm landed at 66099999 nm. On THIS board that
    # moved 15 segments, and this gate reported "copper sets identical: True"
    # both before and after the fix -- measured, by re-running it with the bug
    # restored. Report the 1 nm view too, so the next such fork is visible here
    # rather than only in replay_plan_vs_run (which canonicalizes at 6 dp).
    #
    # Reported, not failed: this file reports divergence by design, and a
    # sub-micron delta breaks no DRC or fab rule. It is still a real fork, and
    # A* tie-breaks are where sub-micron differences stop being invisible.
    n1, w1 = canon(cli_pcb, dp=6)
    n2, w2 = canon(gui_pcb, dp=6)
    if (n1 - n2) or (n2 - n1) or (w1 - w2) or (w2 - w1):
        print(f"  SUB-MICRON FORK: identical at 1um but NOT at 1nm -- "
              f"segments cli-only={len(n1 - n2)} gui-only={len(n2 - n1)}, "
              f"vias cli-only={len(w1 - w2)} gui-only={len(w2 - w1)}")
        print(f"    a coordinate CONVERSION difference, not different routing; "
              f"check the mm->nm rounding on both fronts "
              f"(see kicad_ipc_adapter.vec_mm)")
    else:
        print("  ...and still identical at 1nm (no sub-micron fork)")
    for tag, diff in (('cli-only seg', sorted(s1 - s2)[:3]),
                      ('gui-only seg', sorted(s2 - s1)[:3])):
        for d in diff:
            print(f"    {tag}: {d}")
    identical = (s1 == s2 and v1 == v2)
    print(f"  copper sets identical: {identical}")
    return identical


def main():
    board = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('--') \
        else DEFAULT_BOARD
    workdir = None
    if '--workdir' in sys.argv:
        workdir = sys.argv[sys.argv.index('--workdir') + 1]
    if workdir is None:
        workdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'work')
    workdir = os.path.abspath(workdir)
    # PURGE the workdir first. Both legs route to FIXED output names in here, and
    # route*.py reads back a sibling <output>.kicad_pro DRC floor written by the
    # PREVIOUS run -- so re-running into a dirty workdir silently re-routes at a
    # different floor and the gate reports a copper divergence that is pure
    # carry-over, not a regression. Measured: first run on a clean dir gives
    # IDENTICAL copper sets (1307/1307 segs, 144/144 vias); the 2nd and 3rd runs
    # in the SAME dir reported 3 then 624/644 divergent segments with NO code
    # change, reproducibly. That false failure is exactly the phantom a parity
    # gate must never manufacture (see CLAUDE.md: fresh output path per run).
    if os.path.isdir(workdir):
        shutil.rmtree(workdir)
    os.makedirs(workdir, exist_ok=True)

    board = stage_board(board, workdir)
    # KICAD_SAVELOG=<path>: log every in-process SaveBoard call + stack.
    # Ground truth for "which snapshot sites actually run in this gate" --
    # #627's mechanism hunt stalled on instrumentation that concluded
    # _live_fill/_stage_live_board never execute here; this hook shows both
    # do (via plane_fragility and the in-run finalize oracle staging).
    if os.environ.get('KICAD_SAVELOG'):
        import pcbnew as _pn
        import traceback as _tb
        _orig_save = _pn.SaveBoard

        def _logged_save(*a, **k):
            with open(os.environ['KICAD_SAVELOG'], 'a') as _f:
                _f.write(f"SaveBoard args={a[:1]} kwargs={k}\n")
                _tb.print_stack(file=_f)
                _f.write('---\n')
            return _orig_save(*a, **k)
        _pn.SaveBoard = _logged_save
    cli_final = run_cli_leg(board, workdir)
    gui_final = run_gui_leg(board, workdir)

    print("\n=== PARITY REPORT ===")
    a = grade(cli_final, 'CLI')
    b = grade(gui_final, 'GUI')
    same = (a['conn_full'] == b['conn_full'] and a['kicad'] == b['kicad']
            and a['drc'] == b['drc'])
    identical = compare_copper(cli_final, gui_final)
    print(f"\nVERDICT: {'PARITY' if same else 'DIVERGENT'} "
          f"(kicad unconnected CLI={a['kicad']} GUI={b['kicad']}, "
          f"drc CLI={a['drc']} GUI={b['drc']}); "
          f"copper sets {'IDENTICAL' if identical else 'DIFFER'}")
    return 0


if __name__ == '__main__':
    try:
        import kipy  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(main())
