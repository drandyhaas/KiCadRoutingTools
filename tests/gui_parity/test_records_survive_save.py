#!/usr/bin/env python3
"""What a GUI step records survives the user's KiCad save, and the CLI reads it.

    python3 tests/gui_parity/test_records_survive_save.py

(re-execs into KiCad's bundled python automatically, like its siblings, and
does NOT self-skip without it: exit 2)

THE GAP. The GUI's writers put their records -- ``kicad_routing_tools.
fab_floor_origin``, ``protected_nets``, ``net_impedance``,
``pour_served_pads`` -- into the board's ``.kicad_pro`` ON DISK. KiCad's own
project save rewrites that file from the copy it loaded when the board was
opened, so the user's next save dropped every one of them, and the next CLI
step read a project that had lost them: the origin re-seeded from the lowered
floors (no FAB FLOOR RELAXED), protected nets rippable, impedance widths gone,
pour promises unaudited. The plan executor's end-of-plan writeback has the
same shape for RULES: it wrote the project with fix_project_for_output, and a
rule it did not also set in memory reverted on save.

WHAT THIS DRIVES, on real boards through real pcbnew saves:

  1. flat_hierarchy (a project): a manual GUI step's writeback
     (apply_targets_to_board, then gui_utils.update_live_drc_floors, with the
     engine's protection / impedance / pour notes pending), then
     ``pcbnew.SaveBoard`` with its project -- what KiCad's File > Save does.
     Every record is read back by the CLI's own readers (declared_fab_floor,
     read_protected_nets, read_impedance_specs, read_pour_served_pads, the
     obstacle map's hole-floor origin), and a CLI step on the saved board
     still says FAB FLOOR RELAXED against the board's original floors.
  2. splitflap_driver with NO project: the same, where the records have no
     project to live in at all until KiCad's save creates one.
  3. the plan executor's REAL ``_write_drc_floors`` on flat_hierarchy, then a
     save: every rule its fix_project_for_output wrote is still in the saved
     project, and the class clearances too.
"""
import contextlib
import glob
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]
DECLARING = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
PROJECTLESS = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
STEP = {'clearance': 0.15, 'track_width': 0.15, 'via_size': 0.4, 'via_drill': 0.2}


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew + wx found; this gate does not "
          "self-skip, because a gate that exits 0 without running reports "
          "everything it guards as checked.")
    sys.exit(2)


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    sys.path.insert(0, REPO)
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    sys.path.insert(0, os.path.dirname(REPO))
    app = wx.App(False)  # noqa: F841
    import pcbnew as _pcbnew
    import protected_nets as P
    from copy_board import copy_board
    import tool_records
    import fix_kicad_drc_settings as _F
    from fix_kicad_drc_settings import (apply_targets_to_board, compute_targets,
                                        declared_fab_floor, fix_project_for_output)

    def declared_project_rule(pcb, key):
        """The rule the next CLI step reads (fix_kicad_drc_settings.
        declared_project_rule; on code without it, the project's rules)."""
        fn = getattr(_F, 'declared_project_rule', None)
        if fn is not None:
            return fn(pcb, key)
        with open(os.path.splitext(pcb)[0] + '.kicad_pro', encoding='utf-8') as f:
            v = json.load(f)['board']['design_settings']['rules'].get(key)
        return float(v) if isinstance(v, (int, float)) else None
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.gui_utils import (board_minima_from_live,
                                                update_live_drc_floors)
    from obstacle_map import declared_hole_clearance

    failures, tmpdirs = [], []

    def check(name, cond, info=""):
        if not cond:
            failures.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {info}" if info else ""))

    def stage(src, with_project):
        d = tempfile.mkdtemp(prefix='records_save_')
        tmpdirs.append(d)
        dst = os.path.join(d, 'b.kicad_pcb')
        if with_project:
            _quiet(copy_board, src, dst)
        else:
            shutil.copyfile(src, dst)
        return dst

    PROT = {'/RECORD_A': 'length-matched'}
    IMP = {'/RECORD_A': {'ohms': 50.0}}
    POUR = {'U1.1': {'net': 'GND', 'layer': 'F.Cu', 'how': 'pour'}}

    def gui_step(board):
        """One manual GUI routing step's writeback, as the signal tab runs it,
        with the engine's notes pending."""
        P.note_protection_candidates(PROT)
        P.note_impedance_specs(IMP)
        P.note_pour_served_pads(POUR)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            apply_targets_to_board(board, compute_targets(
                clearance=STEP['clearance'], track_width=STEP['track_width'],
                via_diameter=STEP['via_size'], via_drill=STEP['via_drill'],
                minima=board_minima_from_live(board)), {})
            update_live_drc_floors(board, clearance=STEP['clearance'],
                                   track_width=STEP['track_width'],
                                   via_size=STEP['via_size'],
                                   via_drill=STEP['via_drill'])
        return out.getvalue()

    def read_back(path, label, origin_expect):
        pro = os.path.splitext(path)[0] + '.kicad_pro'
        o = declared_fab_floor(path, 'min_via_diameter')
        check(f"{label}: the CLI reads the fab-floor origin", o == origin_expect,
              f"min_via_diameter origin {o}, expected {origin_expect}")
        check(f"{label}: ... the protected nets",
              P.read_protected_nets(pro) == PROT, f"{P.read_protected_nets(pro)}")
        check(f"{label}: ... the impedance specs",
              P.read_impedance_specs(pro) == IMP, f"{P.read_impedance_specs(pro)}")
        check(f"{label}: ... the pour-served balls",
              P.read_pour_served_pads(pro) == POUR, f"{P.read_pour_served_pads(pro)}")

    print("--- 1. a project board: GUI step, then KiCad's save")
    path = stage(DECLARING, True)
    with open(os.path.splitext(path)[0] + '.kicad_pro', encoding='utf-8') as f:
        declared_via = json.load(f)['board']['design_settings']['rules'][
            'min_via_diameter']
    board = _pcbnew.LoadBoard(path)
    _pcbnew.GetBoard = lambda: board
    gui_step(board)
    check("before the save, the project on disk carries the records",
          P.read_protected_nets(os.path.splitext(path)[0] + '.kicad_pro') == PROT)
    check("pcbnew saves the board and its project", bool(_pcbnew.SaveBoard(path, board)))
    with open(os.path.splitext(path)[0] + '.kicad_pro', encoding='utf-8') as f:
        saved = json.load(f)
    check("(the save really rewrote the project from KiCad's copy: no "
          "protected_nets left in it)",
          'protected_nets' not in (saved.get('kicad_routing_tools') or {}),
          f"{saved.get('kicad_routing_tools')}")
    read_back(path, "after the save", declared_via)
    hc = declared_hole_clearance(parse_kicad_pcb(path))
    check("after the save: the obstacle map's hole floor still takes the "
          "board's original 0.25", abs(hc[0] - 0.25) < 1e-9, f"{hc}")
    cli_out = os.path.join(os.path.dirname(path), 'next.kicad_pcb')
    shutil.copyfile(path, cli_out)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fix_project_for_output(cli_out, input_pcb=path, clearance=0.12,
                               track_width=0.12, via_diameter=0.35, via_drill=0.18)
    txt = buf.getvalue()
    check("a CLI step on the saved board says FAB FLOOR RELAXED against the "
          f"ORIGINAL via floor {declared_via:g}",
          'FAB FLOOR RELAXED' in txt and f'via diameter: {declared_via:g} -> 0.35'
          in txt, txt[-500:])
    check("and carries the records on in its own project",
          P.read_protected_nets(os.path.splitext(cli_out)[0] + '.kicad_pro') == PROT)
    from kicad_parser import build_pcb_data_from_board, compare_pcb_data
    reloaded = _pcbnew.LoadBoard(path)
    diffs = compare_pcb_data(build_pcb_data_from_board(reloaded),
                             parse_kicad_pcb(path))
    check("the saved board, carrying its records, parses the same on both "
          "fronts (records and copper-to-hole floor included)", not diffs,
          f"{diffs[:4]}")
    from kicad_writer import add_tracks_and_vias_to_pcb
    written = os.path.join(os.path.dirname(path), 'written.kicad_pcb')
    _quiet(add_tracks_and_vias_to_pcb, path, written, [], [])
    with open(written, encoding='utf-8') as f:
        carried = '(property "kicad_routing_tools"' in f.read()
    check("and the CLI's board writer carries the board's copy into its output",
          carried)

    print("--- 2. a project-less board: GUI step, then KiCad's save")
    bare = stage(PROJECTLESS, False)
    board2 = _pcbnew.LoadBoard(bare)
    _pcbnew.GetBoard = lambda: board2
    gui_step(board2)                              # step 1: no origin, as the CLI
    STEP_2 = {'clearance': 0.12, 'track_width': 0.12, 'via_size': 0.35,
              'via_drill': 0.18}
    _quiet(apply_targets_to_board, board2, compute_targets(
        clearance=STEP_2['clearance'], track_width=STEP_2['track_width'],
        via_diameter=STEP_2['via_size'], via_drill=STEP_2['via_drill'],
        minima=board_minima_from_live(board2)), {})
    _quiet(update_live_drc_floors, board2, **STEP_2)
    check("pcbnew saves it (creating its project)", bool(_pcbnew.SaveBoard(bare, board2)))
    read_back(bare, "after the save", 0.4)

    print("--- 3. the plan executor's end-of-plan writeback, then KiCad's save")
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from kicad_routing_plugin.ai_plan import PlanExecutor
    plan = stage(DECLARING, True)
    board3 = _pcbnew.LoadBoard(plan)
    _pcbnew.GetBoard = lambda: board3
    dlg = RoutingDialog(None, parse_kicad_pcb(plan), plan)
    dlg._suppress_completion_popups = True
    steps = [{'action': 'route', 'params': dict(STEP)}]
    ex = PlanExecutor(dlg, steps, [0], lambda *a: None, lambda *a: None)
    ex._completed = 1    # the plan's step ran; the write reads the steps that did
    _quiet(ex._write_drc_floors)
    with open(os.path.splitext(plan)[0] + '.kicad_pro', encoding='utf-8') as f:
        written = json.load(f)
    check("pcbnew saves the board and its project", bool(_pcbnew.SaveBoard(plan, board3)))
    dlg.Destroy()
    with open(os.path.splitext(plan)[0] + '.kicad_pro', encoding='utf-8') as f:
        after = json.load(f)
    w_rules = written['board']['design_settings']['rules']
    # Read back the way the next CLI step reads them: the project's rules,
    # with the records' copy of a repo-private rule (min_via_drill, which
    # KiCad's save drops when it was written after load).
    a_rules = {k: declared_project_rule(plan, k) for k in w_rules}
    lost = {k: (w_rules[k], a_rules.get(k)) for k in w_rules
            if isinstance(w_rules[k], (int, float))
            and (not isinstance(a_rules.get(k), (int, float))
                 or abs(w_rules[k] - a_rules[k]) > 1e-6)}
    check("every rule the plan's writeback wrote survives the save", not lost,
          f"(written, after save): {lost}")
    w_cls = {c['name']: c.get('clearance') for c in written['net_settings']['classes']}
    a_cls = {c['name']: c.get('clearance') for c in after['net_settings']['classes']}
    lost_c = {k: (w_cls[k], a_cls.get(k)) for k in w_cls
              if w_cls[k] is not None and (a_cls.get(k) is None
                                           or abs(w_cls[k] - a_cls[k]) > 1e-6)}
    check("and every class clearance", not lost_c, f"(written, after save): {lost_c}")
    o = declared_fab_floor(plan, 'min_via_diameter')
    check("and the origin it recorded", o == declared_via, f"{o}")

    def cli_twin(src, with_project, **kw):
        """The CLI step's writeback over a fresh copy of the same board --
        what that step would have written."""
        dst = stage(src, with_project)
        _quiet(fix_project_for_output, dst, input_pcb=src, **kw)
        return dst

    print("--- 4. #581: a manual step's same-net pad via clearance, then a save")
    snpc = stage(DECLARING, True)
    board4 = _pcbnew.LoadBoard(snpc)
    _pcbnew.GetBoard = lambda: board4
    try:
        _quiet(update_live_drc_floors, board4, clearance=STEP['clearance'],
               same_net_pad_clearance=0.15)
    except TypeError:            # code whose step boundary takes no #581 value
        _quiet(update_live_drc_floors, board4, clearance=STEP['clearance'])
    _pcbnew.SaveBoard(snpc, board4)
    cli4 = stage(DECLARING, True)
    _quiet(P.persist_same_net_pad_clearance,
           os.path.splitext(cli4)[0] + '.kicad_pro', 0.15)   # route.py's call
    g4 = P.read_same_net_pad_clearance(os.path.splitext(snpc)[0] + '.kicad_pro')
    c4 = P.read_same_net_pad_clearance(os.path.splitext(cli4)[0] + '.kicad_pro')
    check("the CLI's reader returns the value the CLI step writes",
          g4 == c4 == 0.15, f"GUI {g4}, CLI {c4}")
    import ast
    missing = []
    for mod in ('swig_gui', 'differential_gui', 'planes_gui', 'fanout_gui'):
        src = os.path.join(REPO, 'kicad_routing_plugin', mod + '.py')
        tree = ast.parse(open(src, encoding='utf-8').read())
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(
                    node.func, 'id', getattr(node.func, 'attr', None)) \
                    == 'update_live_drc_floors':
                if 'same_net_pad_clearance' not in {k.arg for k in node.keywords}:
                    missing.append(f"{mod}:{node.lineno}")
    check("every tab's step boundary passes its #581 value (each CLI routing "
          "step records it)", not missing, f"{missing}")

    from fix_kicad_drc_settings import severity_plan
    from placement.legality import courtyard_severity_of
    for label, src, with_pro in (("a project board", DECLARING, True),
                                 ("a project-less board", PROJECTLESS, False)):
        print(f"--- 5. #856: relaxed severities on {label}, then a save")
        sev = stage(src, with_pro)
        board5 = _pcbnew.LoadBoard(sev)
        _pcbnew.GetBoard = lambda: board5
        out5 = io.StringIO()
        with contextlib.redirect_stdout(out5):
            apply_targets_to_board(board5, compute_targets(
                clearance=STEP['clearance'],
                minima=board_minima_from_live(board5)), severity_plan())
            update_live_drc_floors(board5, clearance=STEP['clearance'])
        _pcbnew.SaveBoard(sev, board5)
        cli5 = cli_twin(src, with_pro, clearance=STEP['clearance'],
                        relax_severities=True)
        g5 = tool_records.effective_namespace(sev).get('saved_severities')
        c5 = tool_records.effective_namespace(cli5).get('saved_severities')
        bds5 = board5.GetDesignSettings()
        if hasattr(bds5, 'SetSeverity') and hasattr(_pcbnew, 'DRCE_ANNULAR_WIDTH'):
            check("the saved severities are the ones the CLI step records",
                  bool(c5) and g5 == c5, f"GUI {g5}, CLI {c5}")
            g5c, c5c = (courtyard_severity_of(sev, 'courtyards_overlap'),
                        courtyard_severity_of(cli5, 'courtyards_overlap'))
            check("and the CLI's own reader (the courtyard grade) reads the same",
                  g5c == c5c, f"GUI {g5c}, CLI {c5c}")
        else:
            # This pcbnew cannot change a severity (KiCad 10.0.0: no
            # SetSeverity, no DRCE_* names), so the GUI changes none: a
            # saved_severities record would claim a change that never happened.
            check("this pcbnew cannot change a severity, so the GUI records "
                  "none (the CLI, which changed them, records its originals)",
                  not g5 and bool(c5), f"GUI {g5}, CLI {c5}")
            check("and the GUI says the severities were NOT relaxed",
                  'severities NOT relaxed' in out5.getvalue(),
                  out5.getvalue()[-300:])

    print("--- 6. #1160: a descent that lowers the Default class, then a save")
    import fab_tiers
    from fix_kicad_drc_settings import CLASS_CLEARANCE_RELAXED_KEY
    net6 = next(n for n in parse_kicad_pcb(DECLARING).nets if n)

    def descent():
        """One clearance descent of this run, as an engine notes it."""
        fab_tiers.set_escalation_policy(fab_tiers.DEFAULT_ESCALATION)
        fab_tiers.note_narrowing(net6, 'clearance', 0.2, 0.15, 'test descent',
                                 net_name='/RECORD_D')
    cls6 = stage(DECLARING, True)
    board6 = _pcbnew.LoadBoard(cls6)
    _pcbnew.GetBoard = lambda: board6
    descent()
    out6 = io.StringIO()
    with contextlib.redirect_stdout(out6):
        apply_targets_to_board(board6, compute_targets(
            clearance=0.15, minima=board_minima_from_live(board6)), {})
        update_live_drc_floors(board6, clearance=0.15)
    _pcbnew.SaveBoard(cls6, board6)
    descent()
    cli6 = cli_twin(DECLARING, True, clearance=0.15)
    g6 = tool_records.effective_namespace(cls6).get(CLASS_CLEARANCE_RELAXED_KEY)
    c6 = tool_records.effective_namespace(cli6).get(CLASS_CLEARANCE_RELAXED_KEY)
    check("the class-descent record is the one the CLI step records",
          bool(c6) and g6 == c6, f"GUI {g6}, CLI {c6}")
    check("and the GUI step said so, as the writeback does",
          'Default' in out6.getvalue() and '0.15' in out6.getvalue()
          and 'RECORD_D' in out6.getvalue(), out6.getvalue()[-300:])
    fab_tiers.set_escalation_policy(fab_tiers.DEFAULT_ESCALATION)

    print("--- 7. the repo's via-drill floor, then a save, then the next CLI step")
    vd = stage(DECLARING, True)
    board7 = _pcbnew.LoadBoard(vd)
    _pcbnew.GetBoard = lambda: board7
    _quiet(update_live_drc_floors, board7, **STEP)
    _pcbnew.SaveBoard(vd, board7)
    cli7 = cli_twin(DECLARING, True, **{'clearance': STEP['clearance'],
                                        'track_width': STEP['track_width'],
                                        'via_diameter': STEP['via_size'],
                                        'via_drill': STEP['via_drill']})
    g7, c7 = declared_project_rule(vd, 'min_via_drill'), \
        declared_project_rule(cli7, 'min_via_drill')
    check("the CLI reads the via-drill floor the CLI step writes",
          g7 is not None and g7 == c7, f"GUI {g7}, CLI {c7}")
    nxt = {}
    for tag, prev in (('GUI', vd), ('CLI', cli7)):
        out = os.path.join(os.path.dirname(prev), 'next.kicad_pcb')
        shutil.copyfile(prev, out)
        shutil.copyfile(os.path.splitext(prev)[0] + '.kicad_pro',
                        os.path.splitext(out)[0] + '.kicad_pro')
        _quiet(fix_project_for_output, out, input_pcb=prev, via_drill=0.3)
        nxt[tag] = declared_project_rule(out, 'min_via_drill')
    check("and a next CLI step at a larger drill leaves both at the same floor",
          nxt['GUI'] == nxt['CLI'] == c7, f"{nxt}")

    for d in tmpdirs:
        shutil.rmtree(d, ignore_errors=True)
    print()
    if failures:
        print(f"FAILED ({len(failures)}): " + "; ".join(failures))
        return 1
    print("PASS: a GUI step's records and rules survive the user's KiCad save, "
          "and the CLI reads them")
    return 0


if __name__ == '__main__':
    sys.exit(main())
