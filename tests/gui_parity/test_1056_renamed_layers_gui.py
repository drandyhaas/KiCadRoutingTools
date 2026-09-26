#!/usr/bin/env python3
"""A copper layer renamed in Board Setup keeps its copper on the live board (#1056).

KiCad lets a layer carry a display name (Board Setup > Board Editor Layers:
In1.Cu shown as "GND"). board.GetLayerName() returns that name, while the
engine speaks only the canonical one ("In1.Cu"), so every live-board site that
mapped a layer through GetLayerName missed on a renamed layer:

  pour     the Planes tab's apply keyed its name -> id table by display name,
           so a pour for In1.Cu fell back to F.Cu, where the board's GND/+5V
           through-hole pads connect to nothing.
  rerun    its duplicate-zone guard compared the canonical layer with the
           display name, so a second Create poured every zone again.
  rip      the Route tab's ripped-copper strip keyed live tracks by display
           name, so a ripped track on a renamed layer stayed on the board.
  fill     live_fill_islands keyed its islands ('GND', 'GND'), a key no
           engine lookup of ('GND', 'In1.Cu') finds.
  builder  build_pcb_data_from_board fell back to the display name for a
           non-copper layer, so a User.1 renamed "In1.Cu" (the reporter's
           workaround) read as copper on In1.Cu, which the text parse never
           does.

Every arm drives the real code on a real headless RoutingDialog (nothing
mocked) and has a negative control: the pre-fix display-name mapping patched
back in must reproduce the bug, so a pass means the fix did the work.

Needs KiCad python (wx + pcbnew); re-execs into it like its siblings.

    python3 tests/gui_parity/test_1056_renamed_layers_gui.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

# The real dialog builds about_tab, whose wxEXPAND|wxALIGN_* sizer flags trip a
# fatal assert on wx debug builds. Must be set before wx is imported.
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

SIG_TRACK = ((4.0, 15.0), (12.0, 15.0))

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


# ------------------------------------------------------------------ fixture

def _board(user1_named_in1):
    """The reporter's board in miniature: 4 copper layers, In1.Cu shown as
    "GND" and In2.Cu as "+5V", GND/+5V on through-hole pads, a SIG track on
    In1.Cu. With user1_named_in1, also their workaround: User.1 shown as
    "In1.Cu", with a line drawn on it."""
    user1 = ' (50 "User.1" user "In1.Cu")' if user1_named_in1 else ''
    user1_line = ('  (gr_line (start 2 2) (end 18 2) (stroke (width 0.2) (type default))\n'
                  '    (layer "User.1"))\n') if user1_named_in1 else ''
    (x1, y1), (x2, y2) = SIG_TRACK
    return ('(kicad_pcb (version 20240108) (generator "test")\n'
            '  (general (thickness 1.6))\n'
            '  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal "GND")\n'
            '          (2 "In2.Cu" signal "+5V") (31 "B.Cu" signal)\n'
            f'          (44 "Edge.Cuts" user){user1})\n'
            '  (setup (pad_to_mask_clearance 0))\n'
            '  (net 0 "")\n  (net 1 "GND")\n  (net 2 "+5V")\n  (net 3 "SIG")\n'
            '  (footprint "t:TH" (layer "F.Cu") (at 10 8)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" thru_hole circle (at -2 0) (size 1.6 1.6) (drill 0.8)\n'
            '      (layers "*.Cu") (net 1 "GND"))\n'
            '    (pad "2" thru_hole circle (at 2 0) (size 1.6 1.6) (drill 0.8)\n'
            '      (layers "*.Cu") (net 2 "+5V")))\n'
            f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.2) '
            '(layer "In1.Cu") (net 3))\n'
            + user1_line +
            '  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default))\n'
            '    (fill none) (layer "Edge.Cuts"))\n)\n')


def _stage(d, user1_named_in1=False):
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(_board(user1_named_in1))
    return path


# ------------------------------------------------------------------ controls

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


def _display_name_mappings():
    """The pre-fix mapping: every layer by board.GetLayerName(), exactly as
    planes_gui built it, and the display name the rip strips keyed by."""
    import pcbnew
    board = pcbnew.GetBoard()
    name_to_id = {}
    for i in range(pcbnew.PCB_LAYER_ID_COUNT):
        name = board.GetLayerName(i)
        if name:
            name_to_id[name] = i
    return name_to_id, {i: board.GetLayerName(i)
                        for i in range(pcbnew.PCB_LAYER_ID_COUNT)}


def _old_mapping():
    from kicad_routing_plugin import swig_gui
    return _Patched(swig_gui, '_build_layer_mappings', _display_name_mappings)


def _no_lset_name():
    """The builder before the fix: no canonical token for a non-copper layer,
    so it falls through to the display name."""
    import pcbnew

    def refuse(*a, **k):
        raise AttributeError('LSET.Name')
    return _Patched(pcbnew.LSET, 'Name', staticmethod(refuse))


def _no_copper_names():
    import kicad_parser
    return _Patched(kicad_parser, 'pcbnew_copper_layer_names', lambda: {})


# ------------------------------------------------------------------ arms

def _load(board_path):
    import pcbnew
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui
    board = pcbnew.LoadBoard(board_path)
    pcbnew.GetBoard = lambda: board
    dialog = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board),
                                    board_path)
    return board, dialog


def _zones(board):
    """[(net, layer id)] of the board's copper zones."""
    return sorted((z.GetNetname(), z.GetLayer()) for z in board.Zones()
                  if not z.GetIsRuleArea())


def _pour(board_path, control):
    """Create + apply on the Planes tab, then apply the same zones again (a
    second Create against an engine that re-emits them). Returns the zones
    after each apply, the second apply's (added, skipped), and the board."""
    board, dialog = _load(board_path)
    tab = dialog.planes_tab
    config = {'assignments': [(['GND'], ['In1.Cu']), (['+5V'], ['In2.Cu'])],
              'via_size': 0.6, 'via_drill': 0.3, 'clearance': 0.2,
              'track_width': 0.2, 'grid_step': 0.1, 'add_gnd_vias': False}
    try:
        with control():
            tab._run_create_planes(config)
            emitted = sorted((z.get('net_name'), z.get('layer'))
                             for z in tab._new_zones if not z.get('keepout'))
            again = list(tab._new_zones)
            tab._apply_results_to_board_body()
            first = _zones(board)
            tab._new_zones = again
            tab._apply_results_to_board_body()
            second = _zones(board)
            counts = tab._last_zone_counts
        return emitted, first, second, counts, board
    finally:
        dialog.Destroy()


def _fill_keys(board, control):
    import kicad_exact_fill
    with control():
        return sorted(kicad_exact_fill.live_fill_islands(board))


def _rip(board_path, control):
    """The Route tab's apply stripping the SIG track on renamed In1.Cu.
    Returns the SIG tracks left on the board."""
    import pcbnew
    board, dialog = _load(board_path)
    try:
        sig = [s for s in dialog.pcb_data.segments
               if dialog.pcb_data.nets[s.net_id].name == 'SIG']
        with control():
            dialog._apply_results_to_board_body(
                {'segments_to_remove': sig}, 0, 0, 0.0, {})
        return sig, [t for t in board.GetTracks()
                     if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetname() == 'SIG']
    finally:
        dialog.Destroy()


def _builder_copper(board_path, control):
    """(GUI, CLI) graphic copper segments' layers on the workaround board."""
    import pcbnew
    from kicad_parser import build_pcb_data_from_board, parse_kicad_pcb
    board = pcbnew.LoadBoard(board_path)
    with control():
        gui = build_pcb_data_from_board(board)
    cli = parse_kicad_pcb(board_path)

    def graphic(p):
        return sorted(s.layer for s in p.segments if getattr(s, 'graphic', False))
    return graphic(gui), graphic(cli)


# ------------------------------------------------------------------ main

def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import wx
    import pcbnew
    for p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools')):
        sys.path.insert(0, p)

    app = wx.App(False)          # noqa: F841 - must outlive the dialogs
    wx.MessageBox = lambda *a, **k: wx.OK
    want = sorted([('+5V', pcbnew.In2_Cu), ('GND', pcbnew.In1_Cu)])

    d = tempfile.mkdtemp(prefix='renamed1056_')
    try:
        path = _stage(d)
        probe = pcbnew.LoadBoard(path)
        check("fixture: In1.Cu/In2.Cu carry the display names GND/+5V",
              (probe.GetLayerName(pcbnew.In1_Cu), probe.GetLayerName(pcbnew.In2_Cu))
              == ('GND', '+5V'),
              f"{probe.GetLayerName(pcbnew.In1_Cu)!r}, "
              f"{probe.GetLayerName(pcbnew.In2_Cu)!r}")

        print("--- pour / rerun / fill")
        emitted, first, second, counts, board = _pour(path, _Nothing)
        check("the engine emits GND on In1.Cu and +5V on In2.Cu",
              emitted == [('+5V', 'In2.Cu'), ('GND', 'In1.Cu')], f"{emitted}")
        check("the Planes tab pours them on In1.Cu and In2.Cu",
              first == want, f"{first} vs {want}")
        check("a second apply of the same zones adds none",
              second == first and counts == (0, 2),
              f"{second}, (added, skipped) = {counts}")
        keys = _fill_keys(board, _Nothing)
        check("live_fill_islands keys the pours by canonical layer",
              ('GND', 'In1.Cu') in keys and ('+5V', 'In2.Cu') in keys,
              f"{keys}")

        _, bad_first, bad_second, bad_counts, _ = _pour(path, _old_mapping)
        check("negative control: the display-name mapping pours off In1/In2",
              bad_first and all(lid not in (pcbnew.In1_Cu, pcbnew.In2_Cu)
                                for _, lid in bad_first),
              f"{[(n, pcbnew.LayerName(l)) for n, l in bad_first]}")
        check("negative control: ... and a second apply duplicates them",
              len(bad_second) == 2 * len(bad_first),
              f"{len(bad_first)} -> {len(bad_second)} zones, counts {bad_counts}")
        bad_keys = _fill_keys(board, _no_copper_names)
        check("negative control: display names key the fill ('GND', 'GND')",
              ('GND', 'GND') in bad_keys, f"{bad_keys}")

        print("--- rip")
        sig, left = _rip(path, _Nothing)
        check("the fixture's SIG track parses on In1.Cu",
              [s.layer for s in sig] == ['In1.Cu'], f"{[s.layer for s in sig]}")
        check("the Route tab strips the ripped SIG track", not left,
              f"{len(left)} left")
        _, bad_left = _rip(path, _old_mapping)
        check("negative control: keyed by display name, the track stays",
              len(bad_left) == 1, f"{len(bad_left)} left")

        print("--- builder (User.1 shown as \"In1.Cu\")")
        d2 = os.path.join(d, 'workaround')
        os.makedirs(d2)
        path2 = _stage(d2, user1_named_in1=True)
        gui, cli = _builder_copper(path2, _Nothing)
        check("the GUI builder models the same graphic copper as the text parse",
              gui == cli, f"GUI {gui} vs CLI {cli}")
        bad_gui, _ = _builder_copper(path2, _no_lset_name)
        check("negative control: by display name, User.1's line becomes In1.Cu copper",
              'In1.Cu' in bad_gui, f"{bad_gui}")
    finally:
        shutil.rmtree(d, ignore_errors=True)

    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("PASS: renamed copper layers keep their pours, rip strips, fill keys "
          "and parse on the live board (#1056)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
