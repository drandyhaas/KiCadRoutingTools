#!/usr/bin/env python3
"""check_drc flags same-net SOFT JOINTS: a dangling free end (a segment terminus
that is not a shared vertex, a via, or an own pad) that reaches the rest of the
net only by cap-overlapping another dangling free end.

A clean route ends every piece at a coincident vertex / via / pad. A free end
floating in copper means the real connecting segment was ripped and never
restored (butterstick DQ5), leaving the net held by a sliver of overlap -- a
fragile near-open. Parallel tracks and normal bends (shared vertices, or ends at
pads/vias) must NOT be flagged.

    python3 tests/test_soft_joint.py
"""
import math
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522

from kicad_writer import generate_segment_sexpr, generate_via_sexpr


def _run(body):
    """Return the count of same-net soft-joint WARNINGS check_drc reports (they are
    surfaced as warnings, not counted violations)."""
    text = f'''(kicad_pcb
 (version 20221018)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal))
 (net 0 "")
 (net 1 "/A")
 {body}
)'''
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(text)
        path = f.name
    try:
        r = subprocess.run([sys.executable,
                            os.path.join('py_router', 'check_drc.py'),  # #522
                            path, '-c', '0.1'],
                           capture_output=True, text=True, cwd=REPO)
        out = r.stdout + r.stderr
        # #320 step 3: soft joints are COUNTED violations (per-type header),
        # EXCEPT the sub-coincidence band (<= COINCIDENCE_TOL) -> warning line.
        m = re.search(r'SEGMENT-ENDPOINT-GAP violations \((\d+)\)', out)
        w = re.search(r'sub-coincidence endpoint gap: (\d+)', out)
        return (int(m.group(1)) if m else 0), (int(w.group(1)) if w else 0)
    finally:
        os.unlink(path)


def _seg(x1, y1, x2, y2, w=0.1, net=1):
    return generate_segment_sexpr((x1, y1), (x2, y2), w, 'F.Cu', net)


def _pad_fp(cx, cy, w=1.0, h=0.6, layer='F.Cu', ref='U1', net=1):
    """A one-pad SMD footprint, so a board can carry a real landing pad."""
    return (f'(footprint "lp:P" (layer "F.Cu") (at {cx} {cy}) (attr smd)'
            f' (pad "1" smd rect (at 0 0) (size {w} {h}) (layers "{layer}")'
            f' (net {net} "/A")))')


def _run_pcb(body):
    """(counted segment-endpoint-gap violations, sub-coincidence warnings) for a
    board carrying footprints as well as copper."""
    return _run(body)


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # 1. DANGLING soft joint: two free ends 0.07mm apart, caps (0.1) overlap.
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.05, 0.05, 2.0, 0.05)
    n, _w = _run(body)
    check("dangling free ends bridged by overlap -> flagged", n == 1, f"got {n}")

    # 2. Clean COINCIDENT joint (a bend): shared vertex -> NOT flagged.
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.0, 0.0, 1.5, 0.5)
    n, _w = _run(body)
    check("coincident vertex (bend) -> not flagged", n == 0, f"got {n}")

    # 3. Free ends anchored at a VIA -> NOT flagged (legitimate terminus).
    via = generate_via_sexpr(1.02, 0.02, 0.3, 0.2, ['F.Cu', 'B.Cu'], 1)
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.05, 0.05, 2.0, 0.05) + via
    n, _w = _run(body)
    check("free ends on a via -> not flagged", n == 0, f"got {n}")

    # 4. Far apart (no overlap) -> NOT flagged (genuine disconnection, not a soft joint).
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.5, 0.0, 2.5, 0.0)
    n, _w = _run(body)
    check("free ends beyond cap overlap -> not flagged", n == 0, f"got {n}")

    # 5. SUB-COINCIDENCE band: gap <= COINCIDENCE_TOL (0.02) is quantization-
    # level contact every gate/cleanup treats as connected -> reported as a
    # WARNING, never a counted violation (kuchen /USBH_DN).
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.015, 0.0, 2.0, 0.0)
    n, w = _run(body)
    check("sub-coincidence gap (15um) -> not a counted violation", n == 0, f"got {n}")
    check("sub-coincidence gap (15um) -> warning line", w == 1, f"got {w}")

    # --- close_soft_joints repair pass ---
    from kicad_parser import Segment, Net, PCBData, BoardInfo
    from pcb_modification import close_soft_joints
    from routing_config import GridRouteConfig

    def _cfg():
        c = GridRouteConfig()
        c.clearance = 0.1
        c.layers = ['F.Cu', 'B.Cu']
        return c

    # two dangling free ends 0.07mm apart (caps 0.1 overlap) -> one tiny bridge.
    segs = [Segment(start_x=0.0, start_y=0.0, end_x=1.0, end_y=0.0, width=0.1, layer='F.Cu', net_id=1),
            Segment(start_x=1.05, start_y=0.05, end_x=2.0, end_y=0.05, width=0.1, layer='F.Cu', net_id=1)]
    pcb = PCBData(footprints={}, nets={1: Net(1, '/A')}, segments=segs, vias=[],
                  board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
                  pads_by_net={})
    results = []
    n = close_soft_joints(results, pcb, None, _cfg())
    check("close_soft_joints bridges the soft joint", n == 1, f"added {n}")
    bridge = [s for s in pcb.segments if abs(s.start_x - 1.0) < 1e-6 and abs(s.end_x - 1.05) < 1e-6]
    check("bridge connects the two exact endpoints (coincident)", len(bridge) == 1)
    check("bridge is TINY (< a track width)",
          bridge and math.hypot(bridge[0].end_x - bridge[0].start_x,
                                bridge[0].end_y - bridge[0].start_y) < 0.1)
    # idempotent: re-running adds nothing (the joint is now coincident, count 2).
    n2 = close_soft_joints([], pcb, None, _cfg())
    check("close_soft_joints is idempotent", n2 == 0, f"added {n2}")


    # --- #722: an end whose CAP lands on pad copper is ANCHORED -------------
    # check_weird's soft-joint anchor was fixed for this (#695/#722) but
    # check_drc's copy and the close_soft_joints repair pass kept crediting a
    # pad by its CENTRE, so the three deliberately-coupled copies disagreed:
    # check_weird called the board clean while check_drc counted a violation
    # on it and the repair pass wrote a bridge across copper both ends already
    # touched. close_soft_joints' own docstring says it uses check_drc's exact
    # definition "so detection and repair agree" -- that is the invariant.
    #
    # Two 0.25mm stubs stopping 0.05mm outside a 1.0x0.6 pad: both caps
    # (r=0.125) physically overlap its copper, and check_connected -- the
    # authority -- grades the net JOINED.
    from check_connected import check_net_connectivity
    from kicad_parser import Pad

    def _land(ex):
        return (_seg(-3, -0.06, ex, -0.06, w=0.25)
                + _seg(-3, 0.06, ex, 0.06, w=0.25)
                + _pad_fp(0, 0) + _pad_fp(-3, 0, ref='U2'))

    # BRANCH GUARD: the fixture must sit on the CAP branch, not the centre one
    # it retires -- outside the pad copper by more than COINCIDENCE_TOL, and
    # inside the end's own cap radius. Otherwise it passes for the wrong reason.
    from check_drc import point_to_pad_distance
    from connectivity import COINCIDENCE_TOL
    _lp = Pad(component_ref='U1', pad_number='1', global_x=0.0, global_y=0.0,
              local_x=0, local_y=0, size_x=1.0, size_y=0.6, shape='rect',
              layers=['F.Cu'], net_id=1, net_name='/A', rotation=0.0, drill=0.0)
    _g_land = point_to_pad_distance(-0.55, -0.06, _lp)
    _g_clear = point_to_pad_distance(-0.70, -0.06, _lp)
    check("#722 fixture is ON the cap branch (outside COINCIDENCE_TOL, inside "
          "the cap)", COINCIDENCE_TOL < _g_land < 0.125, f"gap {_g_land:.4f}")
    check("#722 control is OFF the cap branch (beyond the cap)",
          _g_clear > 0.125, f"gap {_g_clear:.4f}")

    n, _w = _run(_land(-0.55))
    check("check_drc: stubs landing on pad copper -> NOT a soft joint (#722)",
          n == 0, f"got {n}")
    # NEGATIVE CONTROL: push the ends out until the caps CLEAR the copper.
    # Still a real fragile near-open -- without this row the one above also
    # passes on a checker that anchors on anything.
    n, _w = _run(_land(-0.70))
    check("check_drc: stubs whose caps CLEAR the pad -> still a soft joint",
          n == 1, f"got {n}")

    # ABSOLUTE agreement with the authority, both arms. Assert what EACH model
    # must say absolutely, never merely that the two differ: a `differ` form
    # passes when both are wrong, which is what a correlated revert of the two
    # mirrored margins produces.
    def _pcb_land(ex, pad_layer='F.Cu', seg_layer='F.Cu'):
        pads = [Pad(component_ref='U1', pad_number='1', global_x=0.0,
                    global_y=0.0, local_x=0, local_y=0, size_x=1.0, size_y=0.6,
                    shape='rect', layers=[pad_layer], net_id=1, net_name='/A',
                    rotation=0.0, drill=0.0),
                Pad(component_ref='U2', pad_number='1', global_x=-3.0,
                    global_y=0.0, local_x=0, local_y=0, size_x=1.0, size_y=0.6,
                    shape='rect', layers=[seg_layer], net_id=1, net_name='/A',
                    rotation=0.0, drill=0.0)]
        segs = [Segment(start_x=-3, start_y=-0.06, end_x=ex, end_y=-0.06,
                        width=0.25, layer=seg_layer, net_id=1),
                Segment(start_x=-3, start_y=0.06, end_x=ex, end_y=0.06,
                        width=0.25, layer=seg_layer, net_id=1)]
        pcb = PCBData(footprints={}, nets={1: Net(1, '/A')}, segments=segs,
                      vias=[], board_info=BoardInfo(
                          layers={}, copper_layers=['F.Cu', 'B.Cu']),
                      pads_by_net={1: pads})
        return pcb, segs, pads

    for _label, _ex, _want_joined in (('caps overlap the pad', -0.55, True),
                                      ('caps clear the pad', -0.70, False)):
        _pcb, _segs, _pads = _pcb_land(_ex)
        _c = check_net_connectivity(1, _segs, [], _pads, [])
        _joined = _c['num_components'] == 1 and not _c['disconnected_pads']
        check(f"check_connected joins the landing pad ({_label}): expected "
              f"{_want_joined}", _joined is _want_joined)
        _n = close_soft_joints([], _pcb, None, _cfg())
        check(f"close_soft_joints writes {0 if _want_joined else 1} bridge "
              f"({_label})", _n == (0 if _want_joined else 1), f"added {_n}")

    # LAYER guard: crediting a pad by CAP RADIUS without checking the pad
    # actually carries copper on the end's layer buys a new UNDER-report --
    # the direction that ships broken copper. B.Cu ends 0.05mm from an
    # F.Cu-ONLY pad are genuinely split (check_connected iterates the pad's own
    # copper layers) and must stay flagged.
    _pcb, _segs, _pads = _pcb_land(-0.55, pad_layer='F.Cu', seg_layer='B.Cu')
    _c = check_net_connectivity(1, _segs, [], _pads, [])
    check("an other-layer pad anchors nothing: check_connected calls it SPLIT",
          _c['num_components'] == 2 and len(_c['disconnected_pads']) == 1)
    check("close_soft_joints still bridges it (the layer guard holds)",
          close_soft_joints([], _pcb, None, _cfg()) == 1)


    # --- graphic DEGREE parity between detection and repair ------------------
    # close_soft_joints skipped graphics when counting endpoint DEGREE while
    # check_drc counted them, so a track end sharing a vertex with copper art
    # was degree 1 here and degree 2 there: check_drc reported nothing and this
    # pass wrote a bridge anyway. The contract is "check_drc's exact soft-joint
    # definition, so detection and repair agree" -- which needs the same degree.
    _gsegs = [Segment(start_x=0.0, start_y=0.0, end_x=1.0, end_y=0.0,
                      width=0.1, layer='F.Cu', net_id=1),
              Segment(start_x=1.0, start_y=0.0, end_x=1.5, end_y=0.5,
                      width=0.1, layer='F.Cu', net_id=1, graphic=True),
              Segment(start_x=1.05, start_y=0.05, end_x=2.0, end_y=0.05,
                      width=0.1, layer='F.Cu', net_id=1)]
    _gpcb = PCBData(footprints={}, nets={1: Net(1, '/A')}, segments=_gsegs,
                    vias=[], board_info=BoardInfo(
                        layers={}, copper_layers=['F.Cu', 'B.Cu']),
                    pads_by_net={})
    check("a track end sharing a vertex with copper ART is not a free end",
          close_soft_joints([], _gpcb, None, _cfg()) == 0,
          "bridges written")
    # Control: the identical geometry with the art absent IS a soft joint, so
    # the row above is the degree count and not a blanket refusal.
    _ngpcb = PCBData(footprints={}, nets={1: Net(1, '/A')},
                     segments=[_gsegs[0], _gsegs[2]], vias=[],
                     board_info=BoardInfo(layers={},
                                          copper_layers=['F.Cu', 'B.Cu']),
                     pads_by_net={})
    check("without the art the same pair IS bridged (control)",
          close_soft_joints([], _ngpcb, None, _cfg()) == 1)

    # --- the bridge must clear a same-net UNPLATED hole -----------------------
    # _seg_foreign_hole_dist skips own-net holes -- right for a PLATED barrel
    # (that copper IS the net), wrong for an unplated one, which has no copper
    # whatever net it is tagged with (#328). A net-tied M3 mounting hole is the
    # hole a same-net bridge is most likely to cross, and nothing was looking.
    _npth = Pad(component_ref='H1', pad_number='1', global_x=1.025,
                global_y=0.025, local_x=0, local_y=0, size_x=3.2, size_y=3.2,
                shape='circle', layers=['*.Cu'], net_id=1, net_name='/A',
                rotation=0.0, drill=3.2)
    _npth.pad_type = 'np_thru_hole'
    _hsegs = [Segment(start_x=-4.0, start_y=0.0, end_x=1.0, end_y=0.0,
                      width=0.1, layer='F.Cu', net_id=1),
              Segment(start_x=1.05, start_y=0.05, end_x=6.0, end_y=0.05,
                      width=0.1, layer='F.Cu', net_id=1)]
    _hpcb = PCBData(footprints={}, nets={1: Net(1, '/A')}, segments=_hsegs,
                    vias=[], board_info=BoardInfo(
                        layers={}, copper_layers=['F.Cu', 'B.Cu']),
                    pads_by_net={1: [_npth]})
    check("a bridge is refused across a same-net UNPLATED drill",
          close_soft_joints([], _hpcb, None, _cfg()) == 0, "bridges written")
    # Control: the SAME unplated hole moved away from the joint does not
    # block it -- so the row above is a distance test, not a blanket refusal
    # whenever an NPTH pad exists on the net.
    _far_npth = Pad(component_ref='H1', pad_number='1', global_x=1.025,
                    global_y=9.0, local_x=0, local_y=0, size_x=3.2,
                    size_y=3.2, shape='circle', layers=['*.Cu'], net_id=1,
                    net_name='/A', rotation=0.0, drill=3.2)
    _far_npth.pad_type = 'np_thru_hole'
    _fpcb = PCBData(footprints={}, nets={1: Net(1, '/A')},
                    segments=[Segment(start_x=-4.0, start_y=0.0, end_x=1.0,
                                      end_y=0.0, width=0.1, layer='F.Cu',
                                      net_id=1),
                              Segment(start_x=1.05, start_y=0.05, end_x=6.0,
                                      end_y=0.05, width=0.1, layer='F.Cu',
                                      net_id=1)],
                    vias=[], board_info=BoardInfo(
                        layers={}, copper_layers=['F.Cu', 'B.Cu']),
                    pads_by_net={1: [_far_npth]})
    check("the same hole far from the joint does NOT block it (control)",
          close_soft_joints([], _fpcb, None, _cfg()) == 1, "bridges written")

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())
