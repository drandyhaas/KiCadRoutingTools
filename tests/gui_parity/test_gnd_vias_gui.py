#!/usr/bin/env python3
"""The Planes tab's GND return vias keep the clearances route_planes' do.

`route_planes.py --add-gnd-vias` and the Planes tab's "Add GND vias near signal
vias" are two code paths around one engine call (add_gnd_vias_to_existing_board),
and until this gate nothing ran the GUI one: #1030's GUI half shipped
compile-checked only. The 2026-09-24 parity audit then found three more places
where the two paths built the GND-via config differently, and one where the
CLI's own half did nothing. This gate drives the REAL PlanesTab on a REAL
headless RoutingDialog, with the real create_plane (nothing mocked), and runs
route_planes.py on the same files:

  class   HV sits in a 0.8 mm net class (Default 0.2): every return via must
          clear the HV pad by 0.8 (#1030). Negative control: the obstacle map
          built without net_clearances, i.e. the GUI before #1030.
  dru     no class; a .kicad_dru rule sets F.Cu clearance to 0.8 mm (#498).
          The GUI never installed the rules. The CLI did, but read them beside
          the OUTPUT board, which gets its copy only after the GND-via block,
          so its return vias ignored them too. Negative control:
          kicad_dru.install_layer_clearances made a no-op.
  layers  4 copper layers, a SIG2 track on In1.Cu where the first return via
          would go. A through via must clear it, but the GUI checked only the
          plane step's outer + pour layers (the bitaxe fix, 5c4a9f8d, reached
          the CLI only).

In every arm the GUI and CLI must place the SAME vias, and the GUI's GND-via
config is read back: all copper layers, an edge clearance at or above the fab
copper-to-edge floor (the CLI's effective_board_edge_clearance), and the
installed layer rules.

Needs KiCad python (wx + pcbnew); re-execs into it like its siblings.

    python3 tests/gui_parity/test_gnd_vias_gui.py
"""
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

# The real dialog builds about_tab, whose wxEXPAND|wxALIGN_* sizer flags trip a
# fatal assert on wx debug builds. Must be set before wx is imported.
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Every versioned install, newest first by NUMERIC version (a string sort
# puts KiCad\9.0 above KiCad\10.0).
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

BASE, HV, VIA, DRILL = 0.2, 0.8, 0.6, 0.3
HV_PAD = (11.8, 10.0, 0.6)                 # x, y, square size
IN1 = ((10.4, 10.2), (11.4, 10.2), 0.2)    # start, end, width
LAYER_IDS = {'F.Cu': 0, 'In1.Cu': 1, 'In2.Cu': 2, 'B.Cu': 31}

SIG_VIA = f'  (via (at 10 10) (size {VIA}) (drill {DRILL}) (layers "F.Cu" "B.Cu") (net 2))\n'
HV_FP = (f'  (footprint "t:SMD" (layer "F.Cu") (at {HV_PAD[0]} {HV_PAD[1]})\n'
         f'    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
         f'    (pad "1" smd rect (at 0 0) (size {HV_PAD[2]} {HV_PAD[2]})\n'
         f'      (layers "F.Cu") (net 3 "HV")))\n')
IN1_TRACK = (f'  (segment (start {IN1[0][0]} {IN1[0][1]}) (end {IN1[1][0]} {IN1[1][1]}) '
             f'(width {IN1[2]}) (layer "In1.Cu") (net 4))\n')
DRU_RULE = ('(version 1)\n'
            '(rule "fcu_wide" (layer "F.Cu") (constraint clearance (min 0.8mm)))\n')

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
            # os.execv re-splits argv on spaces on Windows ("Program Files").
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew + wx found; this gate does not "
          "self-skip, because a gate that exits 0 without running reports "
          "everything it guards as checked.")
    sys.exit(2)


# ------------------------------------------------------------------ fixtures

def _board(copper, items):
    layers = ' '.join(f'({LAYER_IDS[n]} "{n}" signal)' for n in copper)
    return ('(kicad_pcb (version 20240108) (generator "test")\n'
            '  (general (thickness 1.6))\n'
            f'  (layers {layers} (44 "Edge.Cuts" user))\n'
            '  (setup (pad_to_mask_clearance 0))\n'
            '  (net 0 "")\n  (net 1 "GND")\n  (net 2 "SIG")\n  (net 3 "HV")\n'
            '  (net 4 "SIG2")\n'
            '  (footprint "t:TH" (layer "F.Cu") (at 18 18)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)\n'
            '      (layers "*.Cu") (net 1 "GND")))\n'
            + ''.join(items) +
            '  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default))\n'
            '    (fill none) (layer "Edge.Cuts"))\n)\n')


def _project(hv_class):
    classes = [{"name": "Default", "clearance": BASE, "track_width": 0.2,
                "via_diameter": VIA, "via_drill": DRILL}]
    if hv_class:
        classes.append({"name": "HV", "clearance": HV, "track_width": 0.2,
                        "via_diameter": VIA, "via_drill": DRILL})
    # A 0 copper-to-edge rule, as 80 of 184 corpus boards declare below the
    # 0.2 mm fab floor: the live board then reads 0, and only the floor the
    # CLI applies (effective_board_edge_clearance) keeps vias off the edge.
    return {"board": {"design_settings": {"rules": {"min_copper_edge_clearance": 0.0}}},
            "net_settings": {"meta": {"version": 3}, "classes": classes,
                             "netclass_assignments": {"HV": "HV"} if hv_class else {}},
            "meta": {"version": 1}}


def _gap_to_hv_pad(v):
    px, py, s = HV_PAD
    dx = max(abs(v.x - px) - s / 2, 0.0)
    dy = max(abs(v.y - py) - s / 2, 0.0)
    return math.hypot(dx, dy) - v.size / 2


def _gap_to_in1_track(v):
    (x1, y1), (x2, y2), w = IN1
    t = max(0.0, min(1.0, ((v.x - x1) * (x2 - x1) + (v.y - y1) * (y2 - y1))
                     / ((x2 - x1) ** 2 + (y2 - y1) ** 2)))
    d = math.hypot(v.x - (x1 + t * (x2 - x1)), v.y - (y1 + t * (y2 - y1)))
    return d - w / 2 - v.size / 2


ARMS = (
    dict(name='class', copper=['F.Cu', 'B.Cu'], items=[SIG_VIA, HV_FP],
         hv_class=True, dru=False, gap=_gap_to_hv_pad, need=HV, what='the HV pad'),
    dict(name='dru', copper=['F.Cu', 'B.Cu'], items=[SIG_VIA, HV_FP],
         hv_class=False, dru=True, gap=_gap_to_hv_pad, need=HV, what='the HV pad'),
    dict(name='layers', copper=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
         items=[SIG_VIA, IN1_TRACK], hv_class=False, dru=False,
         gap=_gap_to_in1_track, need=BASE, what='the In1.Cu track'),
)


def _stage(d, arm):
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(_board(arm['copper'], arm['items']))
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump(_project(arm['hv_class']), f, indent=2)
    if arm['dru']:
        with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(DRU_RULE)
    return path


# ------------------------------------------------------------------ the fronts

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


def _gui(board_path, off=None):
    """The Planes tab's GND vias ({(x, y): via}) and the config it used."""
    import pcbnew
    import add_gnd_vias
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui

    board = pcbnew.LoadBoard(board_path)
    pcbnew.GetBoard = lambda: board
    dialog = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board),
                                    board_path)
    tab = dialog.planes_tab
    seen = {}
    real = add_gnd_vias.add_gnd_vias_to_existing_board

    def spy(pcb_data, net, dist, config, obstacles, coord):
        seen['config'] = config
        return real(pcb_data, net, dist, config, obstacles, coord)

    config = {'assignments': [(['GND'], ['B.Cu'])],
              'via_size': VIA, 'via_drill': DRILL, 'clearance': BASE,
              'track_width': 0.2, 'grid_step': 0.1, 'add_gnd_vias': True,
              'gnd_via_distance': 3.0, 'gnd_via_net': 'GND'}
    try:
        with _Patched(add_gnd_vias, 'add_gnd_vias_to_existing_board', spy):
            if off is None:
                tab._run_create_planes(config)
            else:
                with off():
                    tab._run_create_planes(config)
        gnd = {n.net_id for n in tab.pcb_data.nets.values() if n.name == 'GND'}
        vias = {(round(v['x'], 3), round(v['y'], 3)): SimpleNamespace(**v)
                for v in tab._new_vias if v['net_id'] in gnd}
    finally:
        dialog.Destroy()
    return vias, seen.get('config')


def _cli(d, board_path):
    """route_planes.py's GND vias on the same files ({(x, y): via}), or None."""
    from kicad_parser import parse_kicad_pcb
    out = os.path.join(d, 'out_cli.kicad_pcb')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route_planes.py'),
         board_path, out, '--nets', 'GND', '--plane-layers', 'B.Cu',
         '--clearance', str(BASE), '--via-size', str(VIA), '--via-drill', str(DRILL),
         '--track-width', '0.2', '--grid-step', '0.1',
         '--add-gnd-vias', '--gnd-via-distance', '3.0'],
        capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=REPO)
    if r.returncode != 0 or not os.path.exists(out):
        print(r.stdout[-2000:] + r.stderr[-2000:])
        return None
    pcb = parse_kicad_pcb(out)
    gnd = {nid for nid, n in pcb.nets.items() if n.name == 'GND'}
    return {(round(v.x, 3), round(v.y, 3)): v for v in pcb.vias if v.net_id in gnd}


# ------------------------------------------------------------------ main

def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import wx
    for p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools')):
        sys.path.insert(0, p)
    import kicad_dru
    import obstacle_map
    from fix_kicad_drc_settings import fab_edge_floor

    app = wx.App(False)          # noqa: F841 - must outlive the dialogs
    wx.MessageBox = lambda *a, **k: wx.OK

    real_map = obstacle_map.build_base_obstacle_map

    def map_without_classes(*a, **k):
        k.pop('net_clearances', None)
        return real_map(*a, **k)

    controls = {
        'class': lambda: _Patched(obstacle_map, 'build_base_obstacle_map',
                                  map_without_classes),
        'dru': lambda: _Patched(kicad_dru, 'install_layer_clearances',
                                lambda *a, **k: None),
    }
    for arm in ARMS:
        name, need, gap, what = arm['name'], arm['need'], arm['gap'], arm['what']
        print(f"--- {name}")
        d = tempfile.mkdtemp(prefix=f'gndgui_{name}_')
        try:
            board = _stage(d, arm)
            gui, cfg = _gui(board)
            for p, v in sorted(gui.items()):
                print(f"    GUI GND via at ({p[0]:.2f},{p[1]:.2f}): "
                      f"{gap(v):.3f} mm to {what}")
            check(f"[{name}] the Planes tab placed GND return via(s)", bool(gui))
            bad = {p: round(gap(v), 3) for p, v in gui.items() if gap(v) < need - 1e-6}
            check(f"[{name}] every GUI return via clears {what} by {need} mm",
                  not bad, f"{bad}")

            check(f"[{name}] the GND-via config checks every copper layer",
                  cfg is not None and list(cfg.layers) == arm['copper'],
                  f"{getattr(cfg, 'layers', None)}")
            floor = fab_edge_floor(board)
            check(f"[{name}] its edge clearance is at or above the fab floor",
                  cfg is not None and cfg.board_edge_clearance >= floor - 1e-9,
                  f"{getattr(cfg, 'board_edge_clearance', None)} vs {floor}")
            if arm['dru']:
                lc = dict(getattr(cfg, 'layer_clearances', None) or {})
                check(f"[{name}] it carries the .kicad_dru F.Cu rule",
                      abs(lc.get('F.Cu', 0) - HV) < 1e-9, f"{lc}")

            if name in controls:
                ctl, _ = _gui(board, controls[name])
                check(f"[{name}] negative control: switched off, a via lands "
                      f"inside {need} mm of {what}",
                      any(gap(v) < need - 1e-6 for v in ctl.values()),
                      f"{sorted((p, round(gap(v), 3)) for p, v in ctl.items())}")

            cli = _cli(d, board)
            check(f"[{name}] route_planes.py produced a board", cli is not None)
            if cli is not None:
                bad_cli = {p: round(gap(v), 3) for p, v in cli.items()
                           if gap(v) < need - 1e-6}
                check(f"[{name}] every CLI return via clears {what} by {need} mm",
                      bool(cli) and not bad_cli, f"{sorted(cli)} {bad_cli}")
                check(f"[{name}] the GUI places the same GND vias as the CLI",
                      set(gui) == set(cli),
                      f"GUI {sorted(gui)} vs CLI {sorted(cli)}")
        finally:
            shutil.rmtree(d, ignore_errors=True)

    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("PASS: the Planes tab's GND return vias keep class, .kicad_dru and "
          "every-layer clearance, and match route_planes.py")
    return 0


if __name__ == '__main__':
    sys.exit(main())
