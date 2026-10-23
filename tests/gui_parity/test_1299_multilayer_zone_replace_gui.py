#!/usr/bin/env python3
"""The Planes tab replaces a multi-layer pour the way the CLI does (#1299).

A KiCad pour can span several layers -- `(layers "F.Cu" "B.Cu")` is ONE zone.
route_planes decided "replace the same-net pour?" one layer at a time, so a
run pouring GND on F.Cu AND B.Cu kept the 2-layer GND pour and stacked a new
zone on each layer. The engine now decides with every layer the call pours
the net on, and the Planes tab's apply deletes a live pour only when this run
replaced EVERY copper layer it covers -- it used to delete single-layer pours
only, so even a correct engine decision left the 2-layer pour on the board.

Arms, each against `route_planes.py` on the same file:
  both  GND on F.Cu + B.Cu: the 2-layer pour is gone and one GND pour per
        layer is added;
  one   GND on B.Cu only: the 2-layer pour stays (deleting it would take the
        F.Cu copper) and a B.Cu pour is added over it.
Negative control: the engine's check given no call layers (the pre-fix
reading) keeps the 2-layer pour in the "both" arm, so a pass means the fix
did the work.

Needs KiCad python (wx + pcbnew); re-execs into it like its siblings, and
does not self-skip.

    python3 tests/gui_parity/test_1299_multilayer_zone_replace_gui.py
"""
import glob
import os
import re
import shutil
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # main() orders its own sys.path below
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]

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


BOARD = '''(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "GND")
  (footprint "t:TH" (layer "F.Cu") (at 10 10)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)
      (layers "*.Cu") (net 1 "GND")))
  (zone (net 1) (net_name "GND") (layers "F.Cu" "B.Cu") (uuid "designer-pour")
    (hatch edge 0.5)
    (connect_pads (clearance 0.5))
    (min_thickness 0.25) (filled_areas_thickness no)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 1 1) (xy 19 1) (xy 19 19) (xy 1 19))))
  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default))
    (fill none) (layer "Edge.Cuts"))
)
'''


def _stage(d):
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    return path


class _Patched:
    """Swap one module attribute for the duration of a `with`."""

    def __init__(self, module, name, value):
        self.module, self.name, self.value = module, name, value

    def __enter__(self):
        self.orig = getattr(self.module, self.name)
        setattr(self.module, self.name, self.value)
        return self

    def __exit__(self, *exc):
        setattr(self.module, self.name, self.orig)
        return False


class _Nothing:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _per_layer_check():
    """The pre-fix reading: the check never sees the rest of the call."""
    import route_planes
    real = route_planes.check_existing_zones

    def per_layer(*a, **k):
        k.pop('call_layers', None)
        return real(*a, **k)
    return _Patched(route_planes, 'check_existing_zones', per_layer)


def _gui_zones(path, layers, control):
    """[(net, (copper layers...))] on the live board after Create + apply."""
    import pcbnew
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui
    board = pcbnew.LoadBoard(path)
    pcbnew.GetBoard = lambda: board
    dialog = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board), path)
    tab = dialog.planes_tab
    config = {'assignments': [(['GND'], list(layers))],
              'via_size': 0.6, 'via_drill': 0.3, 'clearance': 0.2,
              'track_width': 0.2, 'grid_step': 0.1, 'add_gnd_vias': False}
    names = {pcbnew.F_Cu: 'F.Cu', pcbnew.B_Cu: 'B.Cu'}
    try:
        with control():
            tab._run_create_planes(config)
            tab._apply_results_to_board_body()
        return sorted((z.GetNetname(),
                       tuple(sorted(names.get(l, str(l))
                                    for l in z.GetLayerSet().Seq())))
                      for z in board.Zones() if not z.GetIsRuleArea())
    finally:
        dialog.Destroy()


def _cli_zones(path, layers, out):
    """The same, from route_planes.py's written board."""
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(REPO, 'py_router', 'route_planes.py'), path, out,
                        '--nets'] + ['GND'] * len(layers)
                       + ['--plane-layers'] + list(layers),
                       capture_output=True, text=True, cwd=REPO)
    if r.returncode != 0 or not os.path.isfile(out):
        return None, (r.stdout + r.stderr)[-1500:]
    text = open(out, encoding='utf-8').read()
    zones = []
    for m in re.finditer(r'\(zone\b', text):
        hdr = text[m.start():text.find('(polygon', m.start())]
        if '(keepout' in hdr:
            continue
        name = re.search(r'\(net_name\s+"([^"]*)"\)', hdr) or \
            re.search(r'\(net\s+"([^"]*)"\)', hdr)
        lay = re.search(r'\(layers?\s+([^()]*)\)', hdr)
        zones.append((name.group(1) if name else '',
                      tuple(sorted(t.strip('"') for t in lay.group(1).split()))))
    return sorted(zones), ''


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
    wx.MessageBox = lambda *a, **k: wx.OK

    d = tempfile.mkdtemp(prefix='multilayer1299_')
    try:
        path = _stage(d)
        print("--- both layers: the 2-layer pour is replaced")
        want = [('GND', ('B.Cu',)), ('GND', ('F.Cu',))]
        gui = _gui_zones(path, ['F.Cu', 'B.Cu'], _Nothing)
        cli, log = _cli_zones(path, ['F.Cu', 'B.Cu'], os.path.join(d, 'both.kicad_pcb'))
        check("route_planes wrote the board", cli is not None, log)
        check("the CLI drops the 2-layer pour and pours each layer", cli == want,
              f"{cli}")
        check("the Planes tab does the same", gui == want, f"{gui}")

        print("--- one layer: the 2-layer pour is kept")
        want = [('GND', ('B.Cu',)), ('GND', ('B.Cu', 'F.Cu'))]
        gui = _gui_zones(path, ['B.Cu'], _Nothing)
        cli, log = _cli_zones(path, ['B.Cu'], os.path.join(d, 'one.kicad_pcb'))
        check("the CLI keeps the 2-layer pour beside the new B.Cu pour",
              cli == want, f"{cli}")
        check("the Planes tab does the same", gui == want, f"{gui}")

        print("--- negative control: the per-layer check")
        bad = _gui_zones(path, ['F.Cu', 'B.Cu'], _per_layer_check)
        check("without the call's layers the 2-layer pour stays under the new ones",
              ('GND', ('B.Cu', 'F.Cu')) in bad and len(bad) == 3, f"{bad}")
    finally:
        shutil.rmtree(d, ignore_errors=True)

    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("PASS: the Planes tab replaces a multi-layer pour exactly as the CLI "
          "does (#1299)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
