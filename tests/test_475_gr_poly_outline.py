#!/usr/bin/env python3
"""Regression tests for issue #475 -- routing a board whose Edge.Cuts outline is
a free-form ``gr_poly`` (curved/organic shape) crashed the GUI in two unrelated
ways, plus a latent unbound-local bug it exposed:

1. PARSER (root cause of the `route` crash): the pcbnew builder's Edge.Cuts
   extractor (`_shape_segments`) handled SEGMENT/RECT/ARC/CIRCLE/BEZIER but NOT
   SHAPE_T_POLY, so a gr_poly outline contributed zero segments -- the GUI lost
   the outline entirely (only a stray mounting-hole Edge.Cuts shape survived),
   while the text parser read it fine.

2. ROUTING_COMMON (latent bug the above detonated): `warn_targets_outside_board`
   referenced a local `outline` that is only bound on the bbox-fallback path, so
   any pad outside a real polygon outline raised UnboundLocalError.

3. REPAIR (the `route_planes` crash -- INDEPENDENT of the outline):
   `route_disconnected_planes.route_planes` did not resolve `zone_clearance=None`
   (the GUI zone-clearance "auto" value), so None threaded into
   find_disconnected_zone_regions' layer_clearance and detonated pad_rect_halfspan
   as `float + None`.

    python3 tests/test_475_gr_poly_outline.py
"""
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

fails = []


def check(name, cond):
    print(("  ok  " if cond else " FAIL ") + name)
    if not cond:
        fails.append(name)


# --------------------------------------------------------------------------
# 1. routing_common.warn_targets_outside_board: polygon outline + off-board pad
#    must NOT raise (was UnboundLocalError on `outline`), and must classify the
#    pad as 'outside'.
# --------------------------------------------------------------------------
def test_routing_common_polygon_off_board_pad():
    from types import SimpleNamespace
    from routing_common import warn_targets_outside_board

    # A small square outline; one pad inside, one clearly outside.
    outline = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    pad_in = SimpleNamespace(global_x=5.0, global_y=5.0, component_ref='U1',
                             pad_number='1')
    pad_out = SimpleNamespace(global_x=50.0, global_y=50.0, component_ref='U1',
                              pad_number='2')
    pcb = SimpleNamespace(
        board_info=SimpleNamespace(board_bounds=(0.0, 0.0, 10.0, 10.0),
                                   board_outlines=[outline],
                                   board_outline=outline),
        pads_by_net={7: [pad_in, pad_out]},
    )
    try:
        flagged = warn_targets_outside_board(pcb, [('NET7', 7)], label='target')
        raised = False
    except Exception as e:  # noqa: BLE001
        raised = True
        flagged = []
        print("    raised:", type(e).__name__, e)
    check("warn_targets_outside_board: polygon+off-board pad does not raise",
          not raised)
    kinds = {where: kind for (_n, where, _x, _y, kind) in flagged}
    check("warn_targets_outside_board: outside pad flagged 'outside'",
          kinds.get('U1.2') == 'outside')
    check("warn_targets_outside_board: inside pad not flagged",
          'U1.1' not in kinds)


# --------------------------------------------------------------------------
# 2. _extract_board_contours_from_pcbnew: a SHAPE_T_POLY Edge.Cuts drawing must
#    yield the polygon outline. Runs with an INJECTED fake `pcbnew` so it does
#    not need KiCad's bundled python.
# --------------------------------------------------------------------------
def test_shape_t_poly_extractor():
    import kicad_parser

    ring_pts = [(0.0, 0.0), (30.0, 0.0), (30.0, 20.0), (15.0, 25.0), (0.0, 20.0)]

    class _Pt:
        def __init__(self, x, y):
            self.x, self.y = x, y

    class _Ring:
        def PointCount(self):
            return len(ring_pts)

        def CPoint(self, i):
            return _Pt(*ring_pts[i])

    class _PolySet:
        def OutlineCount(self):
            return 1

        def Outline(self, i):
            return _Ring()

    class _PolyDrawing:
        def GetLayer(self):
            return 44

        def GetClass(self):
            return "PCB_SHAPE"

        def GetShape(self):
            return 4  # matches fake pcbnew.SHAPE_T_POLY

        def GetPolyShape(self):
            return _PolySet()

    class _Board:
        def GetDrawings(self):
            return [_PolyDrawing()]

        def GetFootprints(self):
            return []

    fake = types.ModuleType('pcbnew')
    fake.Edge_Cuts = 44
    fake.SHAPE_T_SEGMENT = 0
    fake.SHAPE_T_RECT = 1
    fake.SHAPE_T_ARC = 2
    fake.SHAPE_T_CIRCLE = 3
    fake.SHAPE_T_POLY = 4
    fake.SHAPE_T_BEZIER = 5
    saved = sys.modules.get('pcbnew')
    sys.modules['pcbnew'] = fake
    try:
        outers, cutouts = kicad_parser._extract_board_contours_from_pcbnew(
            _Board(), to_mm=lambda v: v)
    finally:
        if saved is None:
            del sys.modules['pcbnew']
        else:
            sys.modules['pcbnew'] = saved

    check("SHAPE_T_POLY extractor: one outer ring", len(outers) == 1)
    got = outers[0] if outers else []
    # The chainer may rotate the start vertex / drop a closing duplicate, so
    # compare as a vertex SET at the given precision.
    check("SHAPE_T_POLY extractor: all polygon vertices recovered",
          set((round(x, 3), round(y, 3)) for x, y in got) ==
          set((round(x, 3), round(y, 3)) for x, y in ring_pts))


# --------------------------------------------------------------------------
# 3. route_disconnected_planes.route_planes: zone_clearance=None (the GUI "auto"
#    value) must be resolved, not threaded into a `float + None` crash. Needs the
#    Rust router; skipped (not failed) when it is unavailable.
# --------------------------------------------------------------------------
def test_repair_zone_clearance_none():
    try:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rust_router'))
        import grid_router  # noqa: F401
    except Exception as e:  # noqa: BLE001
        print("  SKIP repair zone_clearance=None (grid_router unavailable:",
              type(e).__name__, ")")
        return

    import tempfile
    import route_planes
    import route_disconnected_planes as rdp
    from kicad_parser import parse_kicad_pcb

    # A minimal 2-net, 4-layer board with two GND pads and two +3V3 pads far
    # apart so the plane step has real copper, then repair with zone_clearance
    # unset (None).
    board = '''(kicad_pcb (version 20260206) (generator test)
 (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal)
  (44 "Edge.Cuts" user))
 (gr_poly (pts (xy 0 0) (xy 40 0) (xy 40 30) (xy 20 34) (xy 0 30)) (layer "Edge.Cuts") (width 0.1))
 (footprint "test:r" (layer "F.Cu") (at 6 15)
  (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0 30) (size 1 0.6) (layers "F.Cu" "In1.Cu") (net 1 "GND"))
  (pad "2" smd rect (at 2 0) (size 1 0.6) (layers "F.Cu" "In2.Cu") (net 2 "+3V3")))
 (footprint "test:r" (layer "F.Cu") (at 34 15)
  (property "Reference" "U2" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 0.6) (layers "F.Cu" "In1.Cu") (net 1 "GND"))
  (pad "2" smd rect (at 2 0) (size 1 0.6) (layers "F.Cu" "In2.Cu") (net 2 "+3V3")))
 (footprint "test:r" (layer "F.Cu") (at 20 8)
  (property "Reference" "U3" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0 30) (size 1 0.6) (layers "F.Cu" "In1.Cu") (net 2 "+3V3")))
 (net 0 "") (net 1 "GND") (net 2 "+3V3")
)'''
    # NOTE: U3's rotated +3V3 pad on In1.Cu (foreign to the GND plane there) is
    # what forces _build_layer_blocked_set -> _block_pad_cells -> pad_rect_halfspan
    # to be reached with the plane-layer clearance -- the exact path that
    # detonated `float + None` when zone_clearance stayed None. Without a foreign
    # pad on the plane layer the crash path is never entered and this test would
    # pass even on the unfixed engine.
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, 'b.kicad_pcb')
        planed = os.path.join(d, 'b_planed.kicad_pcb')
        with open(src, 'w') as f:
            f.write(board)
        try:
            route_planes.create_plane(
                input_file=src, output_file=planed,
                net_names=['GND', '+3V3'], plane_layers=['In1.Cu', 'In2.Cu'],
                via_size=0.45, via_drill=0.2, track_width=0.15, clearance=0.1,
                zone_clearance=0.1, grid_step=0.1)
            raised = None
            rdp.route_planes(
                input_file=planed, output_file="",
                net_names=['GND', '+3V3'], plane_layers=['In1.Cu', 'In2.Cu'],
                track_width=0.15, clearance=0.1,
                zone_clearance=None,          # <-- the regression trigger
                via_size=0.45, via_drill=0.2, grid_step=0.1, dry_run=True)
        except TypeError as e:
            raised = e
        except Exception as e:  # noqa: BLE001 -- other engine errors aren't this bug
            print("    (non-TypeError engine error, ignored for this test:",
                  type(e).__name__, str(e)[:80], ")")
            raised = None
        check("repair zone_clearance=None: no 'float + None' crash",
              raised is None)


def run():
    test_routing_common_polygon_off_board_pad()
    test_shape_t_poly_extractor()
    test_repair_zone_clearance_none()
    print()
    if fails:
        print("FAILED: %d check(s): %s" % (len(fails), fails))
        return 1
    print("All checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
