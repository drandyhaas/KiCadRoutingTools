#!/usr/bin/env python3
"""#1195 on the GUI front: the fanout tab writes only the floors of copper it drew.

    python3 tests/gui_parity/test_1195_qfn_floors_gui.py

(re-execs into KiCad's bundled python automatically, like its siblings)

THE GAP. #1195 changed qfn_fanout's main: a run that changed no copper writes no
floors, and a run that placed no via (stub mode never does) leaves the via and
hole floors alone. The GUI fanout tab's twin -- `_apply_fanout_results` calling
`update_live_drc_floors` -- kept writing the step's via size and drill on every
QFN run, so after a stub fanout the live board's Board Setup carried a via floor
the CLI's project did not, and every later step sized its vias from it. Both
fronts now read `fix_kicad_drc_settings.fanout_written_floors`.

What this drives: the REAL `FanoutTab._apply_fanout_results` of a headless
RoutingDialog on a REAL pcbnew board (flat_hierarchy, which declares via 0.5 /
drill 0.3 / track 0.2 and carries no vias, so nothing on the board can lower a
via floor by itself), with the step's config asking for via 0.3 / drill 0.2 /
track 0.15:

  1. QFN, one track, no via: the via and drill floors stay 0.5 / 0.3, and the
     track floor drops to 0.15 -- the writeback ran (non-vacuity).
  2. QFN, no copper: nothing moves, the track floor included.
  3. BGA, no copper: the via floors drop to 0.3 / 0.2, as bga_fanout's main
     writes them on every run -- the control that proves the arms can see a
     via floor move at all.
  4. Both call sites read the shared rule (source check), so the CLI half
     cannot drift from what arms 1-3 measure.
"""
import ast
import glob
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
MM = 1e6
CFG = {'fix_drc_settings': True, 'clearance': 0.15, 'track_width': 0.15,
       'via_size': 0.3, 'via_drill': 0.2, 'clearance_ceiling': None}


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew+wx found")
    sys.exit(0)


def _calls(path, func, name):
    """True when `func` in `path` calls `name`."""
    tree = ast.parse(open(path, encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == func:
            return any(isinstance(n, ast.Call) and getattr(n.func, 'id', None) == name
                       for n in ast.walk(node))
    return False


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
    wx.Log.SetActiveTarget(wx.LogStderr())
    import pcbnew as _pcbnew
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog
    board_path = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')

    failures, tmpdirs = [], []

    def check(name, cond, info=""):
        if not cond:
            failures.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {info}" if info else ""))

    def fresh_board(tag):
        # One path per arm: a second LoadBoard of one path hands back the first
        # load's in-memory project settings (see test_782's fresh_board).
        d = tempfile.mkdtemp(prefix=f'test1195_{tag}_')
        tmpdirs.append(d)
        dst = os.path.join(d, 'b.kicad_pcb')
        shutil.copyfile(board_path, dst)
        shutil.copyfile(os.path.splitext(board_path)[0] + '.kicad_pro',
                        os.path.splitext(dst)[0] + '.kicad_pro')
        return _pcbnew.LoadBoard(dst)

    def floors(board):
        s = board.GetDesignSettings()
        return {'via': s.m_ViasMinSize / MM, 'drill': s.m_MinThroughDrill / MM,
                'track': s.m_TrackMinWidth / MM, 'clearance': s.m_MinClearance / MM}

    def apply(tag, kind, tracks, vias):
        live = fresh_board(tag)
        if any(t.GetClass() == 'PCB_VIA' for t in live.GetTracks()):
            raise SystemExit(f"fixture carries vias: arm {tag} cannot isolate the writeback")
        before = floors(live)
        dlg = RoutingDialog(None, parse_kicad_pcb(board_path), board_path)
        dlg._suppress_completion_popups = True
        tab = dlg.fanout_tab
        tab.on_fanout_complete = None
        real_getboard = _pcbnew.GetBoard
        _pcbnew.GetBoard = lambda: live
        try:
            tab._apply_fanout_results(tracks, vias, failed_nets=[],
                                      fanout_config=dict(CFG), fanout_kind=kind)
        finally:
            _pcbnew.GetBoard = real_getboard
            dlg.Destroy()
        return before, floors(live)

    net = next(n for n in parse_kicad_pcb(board_path).nets if n)
    track = {'start': (100.0, 100.0), 'end': (101.0, 100.0), 'width': 0.15,
             'layer': 'F.Cu', 'net_id': net}

    print("-- 1. QFN, one track, no via --")
    b, a = apply('qfn_stub', 'qfn', [track], [])
    info = f"via {b['via']}->{a['via']}, drill {b['drill']}->{a['drill']}, track {b['track']}->{a['track']}"
    check("the via and drill floors stay", a['via'] == b['via'] and a['drill'] == b['drill'], info)
    check("the track floor drops to 0.15 (the writeback ran)", abs(a['track'] - 0.15) < 1e-9, info)

    print("-- 2. QFN, no copper --")
    b, a = apply('qfn_none', 'qfn', [], [])
    check("nothing moves", a == b, f"{b} -> {a}")

    print("-- 3. BGA, no copper (control) --")
    b, a = apply('bga_none', 'bga', [], [])
    info = f"via {b['via']}->{a['via']}, drill {b['drill']}->{a['drill']}"
    check("the via floors drop to 0.3 / 0.2",
          abs(a['via'] - 0.3) < 1e-9 and abs(a['drill'] - 0.2) < 1e-9, info)

    print("-- 4. both fronts read the shared rule --")
    # fanout_fix_kwargs is the one fanout writeback decision, built on
    # fanout_written_floors (#1195), which both mains and the tab call.
    check("fanout_gui._apply_fanout_results calls fanout_fix_kwargs",
          _calls(os.path.join(REPO, 'kicad_routing_plugin', 'fanout_gui.py'),
                 '_apply_fanout_results', 'fanout_fix_kwargs'))
    check("qfn_fanout's main calls fanout_fix_kwargs",
          _calls(os.path.join(REPO, 'py_router', 'qfn_fanout', '__init__.py'),
                 'main', 'fanout_fix_kwargs'))

    for d in tmpdirs:
        shutil.rmtree(d, ignore_errors=True)
    print()
    if failures:
        print(f"FAILED ({len(failures)}): " + ", ".join(failures))
        return 1
    print("ALL PASS -- the fanout tab writes only the floors of copper it drew")
    return 0


if __name__ == '__main__':
    sys.exit(main())
