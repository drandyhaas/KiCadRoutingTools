#!/usr/bin/env python3
"""The Planes tab replaces, or keeps, a net's existing pour exactly as
route_planes.py does with and without --skip-existing-zones.

Without the flag (the CLI default) route_planes REPLACES a net's single-layer
pour on the target layer: the writer drops it from the file and pours a new one
at the run's clearance and minimum width. With it, the pour is kept and no new
zone is made. The Planes tab used to hard-code the flag ON, so a replayed step
that omitted it kept a pour the CLI had replaced. Now the tab has a "Skip
existing zones" box, unticked by default, and the applier carries a
replacement out on the live board (each returned zone says `replace_existing`).

This gate runs the REAL Planes tab on a REAL headless dialog -- create_plane on
the live board's PCBData, then the tab's own applier on a live pcbnew board --
and route_planes.py on the same file, and compares the GND pours on B.Cu that
each leaves behind (old or new, layers, teardrop, clearance, minimum width,
priority):

  replace    an old single-layer GND pour (clearance 0.5, width 0.25) and a
             GND teardrop on B.Cu, no flag: the old pour is gone, ONE new pour
             at 0.3 / 0.15 replaces it, the teardrop survives (#1164: it is no
             pour). Negative control: the applier handed zones without their
             `replace_existing` mark keeps the old pour.
  skip       the same board with the flag / box: the old pour is kept as it is
             and nothing new is poured on B.Cu.
  multi      a GND pour covering F.Cu AND B.Cu: never deleted to replace B.Cu
             alone; the new pour is created beside it at a HIGHER priority. The
             GUI read the live pour as single-layer until its PCBData carried
             zone uuids, and then priced it as a replacement.
  teardrop   only a GND teardrop on B.Cu: the pour must still be added (the
             applier's duplicate guard used to count the teardrop as a pour).

Needs KiCad python (wx + pcbnew); re-execs into it, and exits 2 rather than
self-skipping when there is none. Seconds.

    python3 tests/gui_parity/test_skip_existing_zones_gui.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]

OLD = '00000000-0000-4000-8000-00000000a001'      # the pre-existing pour
TEAR = '00000000-0000-4000-8000-00000000a002'     # a GND teardrop on B.Cu
MULTI = '00000000-0000-4000-8000-00000000a003'    # a GND pour on F.Cu + B.Cu
FIXTURE_UUIDS = {OLD: 'old', TEAR: 'teardrop', MULTI: 'multi'}
ZONE_CLR, MIN_W = 0.3, 0.15                       # this run's pour settings

FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" -- {detail}" if detail else ''))
    if not ok:
        FAILS.append(name)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        r = subprocess.run([cand, '-c', 'import pcbnew, wx'], capture_output=True)
        if r.returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew + wx found; this gate does not "
          "self-skip, because a gate that exits 0 without running reports "
          "everything it guards as checked.")
    sys.exit(2)


# ------------------------------------------------------------------ fixtures

def _zone(uuid, layers, clearance, min_w, pts, priority=0, teardrop=False):
    lay = (f'(layer "{layers[0]}")' if len(layers) == 1 else
           '(layers ' + ' '.join(f'"{l}"' for l in layers) + ')')
    attr = '    (attr (teardrop (type padvia)))\n' if teardrop else ''
    xy = ' '.join(f'(xy {x} {y})' for x, y in pts)
    return (f'  (zone (net 1) (net_name "GND") {lay} (uuid "{uuid}") '
            f'(hatch edge 0.5)\n'
            f'    (priority {priority})\n{attr}'
            f'    (connect_pads (clearance {clearance}))\n'
            f'    (min_thickness {min_w}) (filled_areas_thickness no)\n'
            f'    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
            f'    (polygon (pts {xy}))\n  )\n')


OLD_POUR = _zone(OLD, ['B.Cu'], 0.5, 0.25,
                 [(2, 2), (8, 2), (8, 8), (2, 8)])
TEARDROP = _zone(TEAR, ['B.Cu'], 0, 0.0254,
                 [(14, 14), (15, 14), (15, 15)], priority=30000, teardrop=True)
MULTI_POUR = _zone(MULTI, ['F.Cu', 'B.Cu'], 0.5, 0.25,
                   [(2, 12), (8, 12), (8, 18), (2, 18)], priority=1)


def _board(zones):
    return ('(kicad_pcb (version 20240108) (generator "test")\n'
            '  (general (thickness 1.6))\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (setup (pad_to_mask_clearance 0))\n'
            '  (net 0 "")\n  (net 1 "GND")\n  (net 2 "SIG")\n'
            '  (footprint "t:TH" (layer "F.Cu") (at 10 10)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" thru_hole circle (at -2 0) (size 1.6 1.6) (drill 0.8)\n'
            '      (layers "*.Cu") (net 1 "GND"))\n'
            '    (pad "2" thru_hole circle (at 2 0) (size 1.6 1.6) (drill 0.8)\n'
            '      (layers "*.Cu") (net 2 "SIG")))\n'
            + ''.join(zones) +
            '  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default))\n'
            '    (fill none) (layer "Edge.Cuts"))\n)\n')


def _pours(board):
    """Canonical GND zones on B.Cu: (which, layers, teardrop, clearance, min
    width, priority) -- `which` is the fixture's name for an input zone, or
    'new' for one this run poured."""
    import pcbnew
    out = []
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != 'GND':
            continue
        seq = list(z.GetLayerSet().Seq())
        if pcbnew.B_Cu not in seq:
            continue
        try:
            prio = int(z.GetAssignedPriority())
        except Exception:
            prio = int(z.GetPriority())
        out.append((FIXTURE_UUIDS.get(z.m_Uuid.AsString(), 'new'),
                    tuple(board.GetLayerName(l) for l in seq),
                    bool(z.IsTeardropArea()),
                    round(pcbnew.ToMM(z.GetLocalClearance()), 4),
                    round(pcbnew.ToMM(z.GetMinThickness()), 4),
                    prio))
    return sorted(out)


# ------------------------------------------------------------------ the fronts

def _gui(board_path, skip, strip_marks=False):
    """The Planes tab's create + apply on a live board; its GND B.Cu pours."""
    import pcbnew
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui
    board = pcbnew.LoadBoard(board_path)
    pcbnew.GetBoard = lambda: board
    dialog = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board),
                                    board_path)
    tab = dialog.planes_tab
    config = {'assignments': [(['GND'], ['B.Cu'])],
              'via_size': 0.6, 'via_drill': 0.3, 'clearance': 0.2,
              'track_width': 0.2, 'grid_step': 0.1, 'add_gnd_vias': False,
              'zone_clearance': ZONE_CLR, 'min_thickness': MIN_W,
              'skip_existing_zones': skip}
    try:
        tab._run_create_planes(config)
        if strip_marks:
            for zd in tab._new_zones or []:
                zd.pop('replace_existing', None)
        tab._apply_results_to_board_body()
        return _pours(board)
    finally:
        dialog.Destroy()


def _cli(d, board_path, skip):
    """route_planes.py on the same file; the output's GND B.Cu pours."""
    import pcbnew
    out = os.path.join(d, 'out_cli.kicad_pcb')
    if os.path.exists(out):
        os.remove(out)
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(REPO, 'py_router', 'route_planes.py'),
            board_path, out, '--nets', 'GND', '--plane-layers', 'B.Cu',
            '--via-size', '0.6', '--via-drill', '0.3', '--clearance', '0.2',
            '--track-width', '0.2', '--grid-step', '0.1',
            '--zone-clearance', str(ZONE_CLR), '--min-thickness', str(MIN_W)]
    if skip:
        argv.append('--skip-existing-zones')
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO)
    if r.returncode != 0 or not os.path.exists(out):
        print(r.stdout[-2000:] + r.stderr[-2000:])
        return None
    return _pours(pcbnew.LoadBoard(out))


NEW = ('new', ('B.Cu',), False, ZONE_CLR, MIN_W)
OLD_ROW = ('old', ('B.Cu',), False, 0.5, 0.25, 0)
TEAR_ROW = ('teardrop', ('B.Cu',), True, 0.0, 0.0254, 30000)
MULTI_ROW = ('multi', ('F.Cu', 'B.Cu'), False, 0.5, 0.25, 1)

ARMS = (
    dict(name='replace', zones=[OLD_POUR, TEARDROP], skip=False,
         want=lambda p: (OLD_ROW not in p and TEAR_ROW in p
                         and [r[:5] for r in p if r[0] == 'new'] == [NEW]),
         what='the old pour gone, one new pour at the run settings, the '
              'teardrop kept'),
    dict(name='skip', zones=[OLD_POUR, TEARDROP], skip=True,
         want=lambda p: p == sorted([OLD_ROW, TEAR_ROW]),
         what='the old pour and the teardrop kept, nothing new'),
    dict(name='multi', zones=[MULTI_POUR], skip=False,
         want=lambda p: (MULTI_ROW in p
                         and [r[:5] for r in p if r[0] == 'new'] == [NEW]
                         and all(r[5] > MULTI_ROW[5] for r in p if r[0] == 'new')),
         what='the multi-layer pour kept, one new pour above its priority'),
    dict(name='teardrop', zones=[TEARDROP], skip=False,
         want=lambda p: (TEAR_ROW in p
                         and [r[:5] for r in p if r[0] == 'new'] == [NEW]),
         what='the teardrop kept and the pour added beside it'),
)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import wx
    for p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools')):
        sys.path.insert(0, p)
    app = wx.App(False)          # noqa: F841 - must outlive the dialogs
    wx.Log.SetActiveTarget(wx.LogStderr())
    wx.MessageBox = lambda *a, **k: wx.OK

    for arm in ARMS:
        name = arm['name']
        print(f"--- {name} ({'with' if arm['skip'] else 'without'} "
              f"--skip-existing-zones)")
        d = tempfile.mkdtemp(prefix=f'skipzones_{name}_')
        try:
            path = os.path.join(d, 'b.kicad_pcb')
            with open(path, 'w', encoding='utf-8') as f:
                f.write(_board(arm['zones']))
            cli = _cli(d, path, arm['skip'])
            check(f"[{name}] route_planes.py produced a board", cli is not None)
            gui = _gui(path, arm['skip'])
            print(f"    CLI: {cli}\n    GUI: {gui}")
            if cli is not None:
                check(f"[{name}] CLI: {arm['what']}", arm['want'](cli), f"{cli}")
                check(f"[{name}] the Planes tab leaves the same GND pours on "
                      f"B.Cu as the CLI", gui == cli)
            check(f"[{name}] GUI: {arm['what']}", arm['want'](gui), f"{gui}")
            if name == 'replace':
                ctl = _gui(path, False, strip_marks=True)
                check("[replace] negative control: without the engine's "
                      "replace_existing marks the applier keeps the old pour",
                      OLD_ROW in ctl, f"{ctl}")
        finally:
            shutil.rmtree(d, ignore_errors=True)

    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("PASS: the Planes tab replaces or keeps an existing pour exactly as "
          "route_planes.py does with and without --skip-existing-zones")
    return 0


if __name__ == '__main__':
    sys.exit(main())
