#!/usr/bin/env python3
"""Tests for check_weird.py (read-only copper hygiene checker).

Synthetic Segment/Via/Pad cases:
  * a dangling spur end            -> dangling-end flagged
  * a fanout stub ending on a pad  -> NOT flagged (clean)
  * a soft joint (cap overlap)     -> soft-joint flagged (and NOT dangling)
  * a duplicate segment            -> stacked-copper flagged
  * a square loop of segments      -> redundant-cycle flagged
  * a clean two-pad net            -> no findings at all
  * a half-segment tail past a mid-body via anchor -> dangling-end (tail)
  * a floating via                 -> unsupported-via flagged
  * a corner-graze terminal cap    -> narrow-pad-joint flagged (#696/#416)
  * the same cap landing in the pad body -> NOT flagged (clean)
  * an off-centre via-in-pad (centre outside the pad, barrel overlapping)
    -> NOT flagged, and its verdict AGREES with check_net_connectivity (#695)
  * the same via moved until the barrel clears the pad -> dangling-via

Plus two guards on the REPORTER, which is what #696 actually broke:
  * CATEGORIES is exactly the set of categories _finding(...) emits
    (both directions: an unregistered emission AND a stale registration)
  * print_report names a category that is NOT in CATEGORIES, and its headline
    count equals the sum of the per-category counts

    python3 tests/test_check_weird.py
"""
import ast
import io
import math
import os
import re
import sys
from collections import Counter
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from kicad_parser import Pad, Segment, Via, Zone, Net, BoardInfo, PCBData
import check_weird as check_weird_mod
from check_weird import check_weird, print_report, CATEGORIES
from check_connected import check_net_connectivity
from check_drc import point_to_pad_distance
from connectivity import COINCIDENCE_TOL
from routing_constants import SOFT_JOINT_MIN_GAP

NET = 1
NAME = '/TEST'


def _pad(x, y, size=0.6, layers=('F.Cu',), drill=0.0, num='1', ref='U1'):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=size, size_y=size, shape='circle',
               layers=list(layers), net_id=NET, net_name=NAME,
               rotation=0.0, drill=drill)


def _rect_pad(x, y, w, h, layers=('F.Cu',), num='1', ref='U1'):
    # _check_terminal_web only considers rect/roundrect/oval pads -- a circle
    # has no corner to graze, so _pad() above cannot exercise it.
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=w, size_y=h, shape='rect',
               layers=list(layers), net_id=NET, net_name=NAME,
               rotation=0.0, drill=0.0)


def _emitted_categories():
    """Every category literal any `_finding(...)` call in check_weird.py emits,
    read from the source. This is how #696 is caught at test time: the category
    existed and the emission worked, and only the CATEGORIES entry was missing
    -- so nothing that merely RUNS the checker could see the gap."""
    tree = ast.parse(io.open(check_weird_mod.__file__, encoding='utf-8').read())
    out = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == '_finding' and node.args
                and isinstance(node.args[0], ast.Constant)
                and isinstance(node.args[0].value, str)):
            out.add(node.args[0].value)
    return out


def _seg(x1, y1, x2, y2, layer='F.Cu', width=0.2):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=width, layer=layer, net_id=NET)


def _via(x, y, size=0.6, drill=0.3):
    return Via(x=x, y=y, size=size, drill=drill,
               layers=['F.Cu', 'B.Cu'], net_id=NET)


def _pcb(segments, vias=(), pads=(), zones=()):
    return PCBData(
        board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
        nets={NET: Net(net_id=NET, name=NAME, pads=list(pads))},
        footprints={},
        vias=list(vias),
        segments=list(segments),
        pads_by_net={NET: list(pads)},
        zones=list(zones))


def _cats(findings):
    return Counter(f['category'] for f in findings)


def _pad_geometry_askers():
    """Functions in check_weird.py that ask pad/via CONTACT geometry directly
    instead of through the shared predicate.

    #722's thesis is that ONE question answered several ways is the defect,
    not any single coordinate -- so the guard has to be STRUCTURAL. Nothing
    that merely RUNS the checker can notice a fifth spelling appearing; this
    can, the same way _emitted_categories() catches an unregistered category.
    Nested functions are attributed to their parent, so an offender inside a
    closure fails this too."""
    tree = ast.parse(io.open(check_weird_mod.__file__, encoding='utf-8').read())
    out = set()
    for fn in ast.walk(tree):
        if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for node in ast.walk(fn):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id in ('point_to_pad_distance',
                                         '_point_in_pad')):
                out.add(fn.name)
    return out


def main():
    results = []

    # 1. Dangling spur: pad-to-pad trunk plus a spur teeing into its middle;
    #    the spur's free end at (5, 3) connects nothing.
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0), _seg(5, 0, 5, 3)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    dangles = [x for x in f if x['category'] == 'dangling-end']
    results.append(("dangling spur end flagged", c['dangling-end'] == 1))
    results.append(("dangle reported at the free end (5, 3)",
                    len(dangles) == 1 and abs(dangles[0]['x'] - 5) < 1e-6
                    and abs(dangles[0]['y'] - 3) < 1e-6))
    results.append(("spur root (T-junction) NOT flagged as dangling",
                    all((x['x'], x['y']) != (5.0, 0.0) for x in dangles)))

    # 2. Fanout stub ending on a pad (via F.Cu stub + B.Cu run): clean.
    pads = [_pad(0, 0, num='1'), _pad(3, 0, layers=('B.Cu',), num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 1, 0), _seg(1, 0, 3, 0, layer='B.Cu')],
               vias=[_via(1, 0)], pads=pads)
    f, _ = check_weird(pcb)
    results.append(("fanout stub ending on pad/via NOT flagged", len(f) == 0))

    # 3. Soft joint: two collinear tracks whose endpoints stop 0.05mm short of
    #    each other -- caps overlap ((w1+w2)/2 = 0.2 > 0.05 > min gap 0.01).
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 5, 0), _seg(5.05, 0, 10, 0)], pads=pads)
    # gap 0.05 < default tolerance 0.1: detection is exercised at tolerance=0,
    # and the default-filter behavior is asserted right after.
    f, _ = check_weird(pcb, tolerance=0)
    c = _cats(f)
    results.append(("soft joint flagged", c['soft-joint'] == 1))
    results.append(("soft-joint ends not double-reported as dangling",
                    c['dangling-end'] == 0))

    # 4. Duplicate segment (stacked copper).
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0), _seg(0, 0, 10, 0)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    results.append(("duplicate segment flagged as stacked-copper",
                    c['stacked-copper'] == 1))
    results.append(("each duplicate is individually removable",
                    c['removable-segment'] == 2))

    # 5. Square loop: 4 edges between two pads; one edge is a redundant cycle.
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0), _seg(10, 0, 10, 10),
                _seg(10, 10, 0, 10), _seg(0, 10, 0, 0)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    results.append(("square loop flagged as redundant-cycle",
                    c['redundant-cycle'] == 1))
    results.append(("every loop edge individually removable (superset)",
                    c['removable-segment'] == 4))
    results.append(("loop has no dangling ends", c['dangling-end'] == 0))

    # 6. Clean two-pad net: one segment pad to pad; nothing weird.
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0)], pads=pads)
    f, _ = check_weird(pcb)
    results.append(("clean two-pad net has no findings", len(f) == 0))

    # 7. Half-segment tail past a mid-body via anchor (#347 class): the trunk
    #    is load-bearing THROUGH the via at (6, 0), but 4mm of copper past it
    #    dangles.
    pads = [_pad(0, 0, num='1'), _pad(6, 5, layers=('B.Cu',), num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0), _seg(6, 0, 6, 5, layer='B.Cu')],
               vias=[_via(6, 0)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    dangles = [x for x in f if x['category'] == 'dangling-end']
    results.append(("half-segment tail flagged as dangling-end",
                    c['dangling-end'] == 1))
    results.append(("tail reported past the body anchor with its length",
                    len(dangles) == 1
                    and 'body anchor' in dangles[0]['detail']
                    and '4.000mm' in dangles[0]['detail']
                    and abs(dangles[0]['x'] - 10) < 1e-6))
    results.append(("half-dangle case has no other findings", len(f) == 1))

    # 8. Floating via far from any copper.
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0)], vias=[_via(20, 20)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    results.append(("floating via flagged as unsupported-via",
                    c['unsupported-via'] == 1))

    # 9. Coincident same-net vias (stacked copper, via variant).
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0)],
               vias=[_via(10, 0), _via(10.005, 0)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    results.append(("coincident vias flagged as stacked-copper",
                    c['stacked-copper'] == 1))

    # Soft joints carry size=None deliberately (check_weird.py, the note in
    # _check_soft_joints: a SMALLER gap is still fragile, so filtering by size
    # would invert severity, and on <=0.1mm routing the whole category vanished
    # at the default). So the default 0.1mm tolerance must NOT hide the 0.05mm
    # joint of case 3. This assertion previously ran on the coincident-VIAS
    # board above -- which has no soft joint at all -- so it passed vacuously
    # while claiming the opposite of the shipped behavior.
    soft_pcb = _pcb([_seg(0, 0, 5, 0), _seg(5.05, 0, 10, 0)],
                    pads=[_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')])
    f_tol, _ = check_weird(soft_pcb)
    results.append(("0.05mm soft joint survives the default 0.1mm tolerance",
                    len([x for x in f_tol if x['category'] == 'soft-joint']) == 1))

    # Orphan island: two joined segments + via, nowhere near any pad of the
    # net (pads at 0/10, island at 50) -> flagged with its total length; a
    # sub-tolerance island is hidden by the default 0.1mm filter.
    pcb = _pcb([_seg(0, 0, 10, 0),
                _seg(50, 5, 52, 5), _seg(52, 5, 52, 7, layer='B.Cu')],
               vias=[_via(52, 5)],
               pads=[_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')])
    f, _ = check_weird(pcb)
    isl = [x for x in f if x['category'] == 'orphan-island']
    results.append(("orphan pad-less island flagged",
                    len(isl) == 1 and '4.00mm' in isl[0]['detail']))
    pcb2 = _pcb([_seg(0, 0, 10, 0), _seg(50, 5, 50.05, 5)],
                pads=[_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')])
    f2, _ = check_weird(pcb2)
    results.append(("sub-tolerance orphan island hidden by default",
                    not [x for x in f2 if x['category'] == 'orphan-island']))

    # 10. Narrow pad joint (#416): a 0.2mm track whose free cap overlaps a
    #     1.0x1.0mm rect pad only at the CORNER. The floor is the board's
    #     thinnest track (0.2), so erosion by 0.1 parts the cap from the pad:
    #     a sub-floor web. Flagged, and NOT dropped by the default 0.1mm
    #     tolerance (size=None is deliberate -- for a web, thinner is worse).
    pads = [_rect_pad(0, 0, 1.0, 1.0, num='1'),
            _rect_pad(10, 0, 1.0, 1.0, num='2', ref='U2')]
    pcb = _pcb([_seg(0.55, 0.55, 2.0, 2.0)], pads=pads)
    f, _ = check_weird(pcb)
    necks = [x for x in f if x['category'] == 'narrow-pad-joint']
    results.append(("corner-graze terminal cap flagged as narrow-pad-joint",
                    len(necks) == 1))
    results.append(("narrow-pad-joint reported at the cap (0.55, 0.55)",
                    len(necks) == 1 and abs(necks[0]['x'] - 0.55) < 1e-6
                    and abs(necks[0]['y'] - 0.55) < 1e-6))
    results.append(("narrow-pad-joint survives the default 0.1mm tolerance",
                    len(necks) == 1 and necks[0]['size'] is None))
    #     Negative control, same pad and same track: a cap landing in the pad
    #     BODY joins through full-width copper and is NOT flagged. Without it
    #     the case above would also pass on a check that flagged everything.
    pcb = _pcb([_seg(0.0, 0.0, 2.0, 2.0)], pads=pads)
    f, _ = check_weird(pcb)
    results.append(("cap landing in the pad body NOT flagged",
                    not [x for x in f if x['category'] == 'narrow-pad-joint']))

    # 12. Via-in-pad credited by the BARREL, not the centre (#695). The
    #     router puts vias in pads on purpose (QFN allow_via_in_pad, plane
    #     taps, BGA underpad), and an off-centre one has its centre just
    #     OUTSIDE the pad outline while the barrel still overlaps the copper.
    #     Crediting the centre only made this checker report `dangling-via`
    #     on a joint check_net_connectivity -- the authoritative model, and
    #     KiCad -- grades connected; check_weird's exit code is chain-blocking,
    #     so that false positive cost a reroute lap.
    #
    #     Offset and pad are the kuchen case quoted in check_connected's own
    #     comment (0.42mm circle pad, via centre 0.283mm away, so the centre
    #     sits 0.073mm OUTSIDE the copper). The via here is _via()'s 0.6mm
    #     default rather than kuchen's 0.42mm, so the barrel overlaps by
    #     0.21 + 0.30 - 0.283 = 0.227mm, not kuchen's 0.137mm -- same class,
    #     different number, and the guard below derives its bound from the
    #     fixture instead of restating either. The via also carries a B.Cu run
    #     to a second pad, so the pad is the only thing that can supply F.Cu.
    def _via_in_pad_board(vx):
        pads = [_pad(0, 0, size=0.42, num='1'),
                _pad(5, 0, layers=('B.Cu',), num='2', ref='U2')]
        segs = [_seg(vx, 0, 5, 0, layer='B.Cu')]
        v = _via(vx, 0)
        return _pcb(segs, vias=[v], pads=pads), segs, v, pads

    # The row must be ON the branch it names: assert the via centre really is
    # outside the pad copper by more than the old COINCIDENCE_TOL credit, or
    # this passes for the wrong reason (the centre test would credit it too).
    _gap = point_to_pad_distance(0.283, 0, _pad(0, 0, size=0.42))
    _r = _via(0, 0).size / 2.0
    results.append(("the #695 via centre is OUTSIDE the pad copper "
                    "(guard is on the barrel branch, not the centre one)",
                    COINCIDENCE_TOL < _gap < _r))

    pcb, segs, v, pads = _via_in_pad_board(0.283)
    f, _ = check_weird(pcb)
    results.append(("via-in-pad whose barrel overlaps the pad NOT flagged",
                    not [x for x in f if x['category']
                         in ('dangling-via', 'unsupported-via')]))
    #     Negative control: move the via until the barrel clears the copper
    #     (0.55 -> 0.34mm gap > the 0.30 radius). Still a real dangling via.
    #     Without this the row above would also pass on a check that credited
    #     every pad unconditionally.
    pcb_far, segs_far, v_far, pads_far = _via_in_pad_board(0.55)
    f_far, _ = check_weird(pcb_far)
    results.append(("via whose barrel does NOT reach the pad still flagged",
                    len([x for x in f_far
                         if x['category'] == 'dangling-via']) == 1))

    #     The thesis of #695: this checker must not contradict the
    #     AUTHORITATIVE connectivity model on the same geometry. Assert what
    #     each model must say ABSOLUTELY, not merely that the two differ:
    #     `joined != dangling` passes when BOTH are wrong (revert the margin
    #     in check_weird AND check_connected -- the likely future edit, since
    #     the fix mirrors them -- and it still holds), and it is not even the
    #     right invariant, since it reads a whole-net verdict as a per-via one
    #     and so goes red on any fixture that adds a third, unrouted pad.
    for _label, (_pcb_, _segs_, _v_, _pads_), _want_joined in (
            ('barrel overlaps', (pcb, segs, v, pads), True),
            ('barrel clear', (pcb_far, segs_far, v_far, pads_far), False)):
        _conn = check_net_connectivity(NET, _segs_, [_v_], _pads_, [])
        _joined = (_conn['num_components'] == 1
                   and not _conn['disconnected_pads'])
        _dangling = any(x['category'] == 'dangling-via'
                        for x in check_weird(_pcb_)[0])
        results.append((f"check_connected joins this via to the pad "
                        f"({_label}): expected {_want_joined}",
                        _joined is _want_joined))
        results.append((f"check_weird agrees with it ({_label}): "
                        f"dangling must be {not _want_joined}",
                        _dangling is (not _want_joined)))

    # 13. The SAME centre-vs-barrel asymmetry, in the soft-joint anchor.
    #     `at_anchor` credited a via by its BARREL radius and a pad by centre
    #     containment, four lines apart -- so a stub whose round cap (r =
    #     width/2) physically overlaps a pad, but whose endpoint sits outside
    #     the outline, was counted as a free end. Two such ends facing each
    #     other are then reported as `soft-joint` on copper check_connected
    #     grades as ONE component (its #285 endpoint-cap rule unions a track
    #     end into a pad at max(width/2 - 1e-6, tolerance)). `soft-joint`
    #     carries size=None, so --tolerance cannot filter it away, and
    #     check_weird's exit code is chain-blocking through check_complete.
    #
    #     1.0 x 0.6 pad at the origin; two 0.25mm stubs whose near ends sit
    #     0.05mm outside its right edge, 0.12mm apart, each running away to
    #     its own far pad so the FAR ends anchor and cannot confound the row.
    def _soft_anchor_board(nx):
        p1 = _rect_pad(0, 0, 1.0, 0.6, num='1', ref='U1')
        p2 = _pad(3, 2, num='2', ref='U2')
        p3 = _pad(3, -2, num='3', ref='U3')
        segs = [_seg(nx, 0.06, 3, 2, width=0.25),
                _seg(nx, -0.06, 3, -2, width=0.25)]
        return _pcb(segs, pads=[p1, p2, p3]), segs, [p1, p2, p3]

    _cap_r = 0.25 / 2.0
    _p1 = _rect_pad(0, 0, 1.0, 0.6)
    _near = point_to_pad_distance(0.55, 0.06, _p1)
    _far = point_to_pad_distance(0.65, 0.06, _p1)
    #     The rows must be ON the branch they name, in BOTH directions: the
    #     near end outside the old centre credit but inside the cap (so the
    #     centre test cannot pass it), the far end outside the cap too (so the
    #     control is a real free end, not a fixture that merely moved).
    results.append(("the soft-joint stub end is outside the pad copper but "
                    "inside its own cap (guard is on the cap branch)",
                    COINCIDENCE_TOL < _near < _cap_r))
    results.append(("the control stub end is outside the cap as well",
                    _far > _cap_r))
    #     ...and that the PAIR condition itself is satisfied, or 'no
    #     soft-joint' below would pass for the wrong reason.
    _gap, _cap = 0.12, 0.25
    results.append(("the two stub ends do form a soft-joint pair "
                    "(gap within the overlapping caps)",
                    SOFT_JOINT_MIN_GAP < _gap < _cap - 1e-6))

    pcb_soft, segs_soft, pads_soft = _soft_anchor_board(0.55)
    f_soft, _ = check_weird(pcb_soft)
    results.append(("stub caps overlapping a pad NOT reported as soft-joint",
                    not [x for x in f_soft if x['category'] == 'soft-joint']))
    #     Negative control: same pair, moved until the caps clear the copper.
    #     Still a genuine soft joint -- without this the row above would also
    #     pass on a check that anchored every endpoint unconditionally.
    pcb_gap, segs_gap, pads_gap = _soft_anchor_board(0.65)
    f_gap, _ = check_weird(pcb_gap)
    results.append(("stub caps clear of the pad still reported as soft-joint",
                    len([x for x in f_gap
                         if x['category'] == 'soft-joint']) == 1))

    #     Same thesis as #695 above, and asserted the same way: what each
    #     model must say ABSOLUTELY. In the overlapping case every pad is
    #     joined through the centre pad; in the clear case that pad is
    #     stranded, which is what makes the soft-joint report correct there.
    for _label, (_pcb_, _segs_, _pads_), _want_joined in (
            ('caps overlap the pad', (pcb_soft, segs_soft, pads_soft), True),
            ('caps clear the pad', (pcb_gap, segs_gap, pads_gap), False)):
        _conn = check_net_connectivity(NET, _segs_, [], _pads_, [])
        _joined = (_conn['num_components'] == 1
                   and not _conn['disconnected_pads'])
        _soft = any(x['category'] == 'soft-joint'
                    for x in check_weird(_pcb_)[0])
        results.append((f"check_connected joins the stubs to the pad "
                        f"({_label}): expected {_want_joined}",
                        _joined is _want_joined))
        results.append((f"check_weird agrees with it ({_label}): "
                        f"soft-joint must be {not _want_joined}",
                        _soft is (not _want_joined)))

    # 14. The three things the cap credit alone did NOT fix (#722). Each is a
    #     way this anchor still disagreed with check_connected after the cap
    #     radius landed, and each row carries its own control.
    #
    #     (a) LAYER. at_anchor credited ANY same-net pad whatever layer it sat
    #         on, and widening the reach from 0.02mm to width/2 WIDENED that
    #         hole rather than closing it. check_connected iterates the pad's
    #         OWN copper layers, so B.Cu ends near an F.Cu-only pad are
    #         genuinely SPLIT there while this called them anchored -- a false
    #         NEGATIVE, the direction that ships broken copper.
    def _land(pad_layer='F.Cu', seg_layer='F.Cu', ex=-0.55, w=0.25, pad=None):
        lp = pad if pad is not None else _rect_pad(
            0, 0, 1.0, 0.6, layers=(pad_layer,), num='1')
        pads = [lp, _rect_pad(-3, 0, 1.0, 0.6, layers=(seg_layer,), num='2',
                              ref='U2')]
        segs = [_seg(-3, -0.06, ex, -0.06, layer=seg_layer, width=w),
                _seg(-3, 0.06, ex, 0.06, layer=seg_layer, width=w)]
        return _pcb(segs, pads=pads), segs, pads

    def _sj(board):
        return len([x for x in check_weird(board, tolerance=0)[0]
                    if x['category'] == 'soft-joint'])

    pcb, segs, pads = _land(pad_layer='F.Cu', seg_layer='B.Cu')
    conn = check_net_connectivity(NET, segs, [], pads, [])
    results.append(("an other-layer pad anchors nothing: check_connected "
                    "calls this net SPLIT",
                    conn['num_components'] == 2
                    and len(conn['disconnected_pads']) == 1))
    results.append(("...and check_weird agrees: still a soft joint",
                    _sj(pcb) == 1))
    #     Control: the SAME pad on the ends' own layer anchors both and the
    #     board is clean -- so the row above is the LAYER test firing, not the
    #     predicate refusing everything.
    results.append(("the same pad on the ends' own layer anchors them: clean",
                    _sj(_land(pad_layer='F.Cu', seg_layer='F.Cu')[0]) == 0))

    #     (b) NPTH. An unplated hole has no copper even when its layer list
    #         says *.Cu, so it anchors nothing. Its PLATED twin -- identical
    #         geometry -- must still anchor, or the row would be passing
    #         because the predicate refuses everything.
    _npth = _rect_pad(0, 0, 1.0, 0.6, layers=('*.Cu',), num='1')
    _npth.drill, _npth.pad_type = 0.8, 'np_thru_hole'
    _plated = _rect_pad(0, 0, 1.0, 0.6, layers=('*.Cu',), num='1')
    _plated.drill, _plated.pad_type = 0.8, 'thru_hole'
    results.append(("a PLATED through pad anchors the ends (control)",
                    _sj(_land(pad=_plated)[0]) == 0))
    results.append(("an NPTH pad has no copper and anchors nothing",
                    _sj(_land(pad=_npth)[0]) == 1))

    #     (c) The VIA branch was cap-blind in exactly the way the pad branch
    #         was: `vr + 0.01` credits the barrel but not the end's own cap,
    #         where check_connected credits (via_size + track_width)/2. On a
    #         0.6mm via and a 0.25mm track that is 0.310mm against 0.425mm.
    def _via_land(dx, w=0.25):
        pads = [_pad(-3, 0, num='1'),
                _pad(3, 0, layers=('B.Cu',), num='2', ref='U2')]
        segs = [_seg(-3, 0, -dx, -0.06, width=w),
                _seg(-dx, 0.06, -3, 0, width=w),
                _seg(0, 0, 3, 0, layer='B.Cu', width=w)]
        v = _via(0, 0)
        return _pcb(segs, vias=[v], pads=pads), segs, [v], pads

    _d_band = math.hypot(0.35, 0.06)
    results.append(("the via row is ON the cap band (clears barrel+0.01, "
                    "inside barrel+cap)",
                    0.3 + 0.01 < _d_band <= 0.3 + 0.125))
    pcb_v, segs_v, vias_v, pads_v = _via_land(0.35)
    conn_v = check_net_connectivity(NET, segs_v, vias_v, pads_v, [])
    results.append(("check_connected joins through the barrel+cap overlap",
                    conn_v['num_components'] == 1
                    and not conn_v['disconnected_pads']))
    results.append(("...and check_weird agrees: not a soft joint",
                    _sj(pcb_v) == 0))
    pcb_f, segs_f, vias_f, pads_f = _via_land(0.50)
    conn_f = check_net_connectivity(NET, segs_f, vias_f, pads_f, [])
    results.append(("an end beyond barrel+cap is genuinely split "
                    "(check_connected)",
                    conn_f['num_components'] > 1
                    or bool(conn_f['disconnected_pads'])))
    results.append(("...and check_weird agrees: still a soft joint",
                    _sj(pcb_f) == 1))

    # 15. Copper-layer GRAPHICS (#337) are immutable input art. They were
    #     soft-joint CANDIDATES here and nowhere else -- _check_dangles refuses
    #     them and check_connected refuses them outright ("graphics never
    #     conduct") -- so a graphic pair produced an unactionable finding in a
    #     size=None category that --tolerance can never drop.
    _gpads = [_pad(-5, 0, num='1'), _pad(-6, 0, num='2', ref='U2')]
    _real = [_seg(-6, 0, -5, 0), _seg(3, 0, 5, 0), _seg(3.12, 0, 6, 0.5)]
    _art = [_real[0]] + [Segment(start_x=s.start_x, start_y=s.start_y,
                                 end_x=s.end_x, end_y=s.end_y, width=s.width,
                                 layer=s.layer, net_id=NET, graphic=True)
                         for s in _real[1:]]
    _w_art = check_net_connectivity(NET, _art, [], _gpads, [])
    _wo_art = check_net_connectivity(NET, [_real[0]], [], _gpads, [])
    results.append(("graphics are invisible to check_connected (identical "
                    "verdict with and without them)",
                    (_w_art['num_components'], _w_art['disconnected_pads'])
                    == (_wo_art['num_components'],
                        _wo_art['disconnected_pads'])))
    results.append(("a graphic near-open is NOT a soft joint",
                    _sj(_pcb(_art, pads=_gpads)) == 0))
    #     Control: the SAME coordinates as real track copper still are one, so
    #     the graphic flag is the only difference between these two rows.
    results.append(("the same pair as TRACK copper still is a soft joint",
                    _sj(_pcb(_real, pads=_gpads)) == 1))

    #     (d) A size-less via claims the 0.6 default, as every other consumer
    #         already does; `or 0` gave it radius ZERO, so it anchored nothing
    #         at all. Not reachable through check_weird() -- a size=None via
    #         raises in check_net_connectivity long before at_anchor sees it --
    #         so this is a DIRECT row on the predicate, and the change is a
    #         consistency fix rather than a behaviour one. The control pins
    #         that a genuinely tiny barrel still does not reach.
    from connectivity import endpoint_reaches_via
    _sizeless = _via(0, 0)
    _sizeless.size = None
    results.append(("a size-less via claims the 0.6 default's radius, not 0",
                    endpoint_reaches_via(0.29, 0, 0.0, _sizeless,
                                         ('F.Cu',)) is True))
    results.append(("a genuinely 0.02mm barrel still does not reach 0.29mm "
                    "away (control)",
                    endpoint_reaches_via(0.29, 0, 0.0,
                                         _via(0, 0, size=0.02),
                                         ('F.Cu',)) is False))

    # 16. STRUCTURAL: only the shared predicate may ask pad-contact geometry.
    #     This is the row that makes #722 a CLASS fix rather than a coordinate
    #     fix -- nothing that merely runs the checker can see a fifth spelling
    #     appear, which is exactly how #695 and #722 shipped four lines apart.
    _ALLOWED = {
        # _check_terminal_web is a deliberate exception, and it is NOT asking
        # this question: it selects the pad an EROSION model runs against, and
        # that model is undefined unless the cap strictly overlaps the pad body
        # (`< r - 1e-6`, with NO COINCIDENCE_TOL floor -- a floor would turn a
        # sub-0.04mm cap that does not touch into a target). Measured while
        # fixing #722: routing it through the shared predicate flips 0 of the
        # terminal ends in kicad_files/, so this is a recorded scope choice.
        '_check_terminal_web',
        # _check_unsupported_vias asks the same question about a BARREL and is
        # the next site to fold in; #695 fixed its margin in place.
        '_check_unsupported_vias',
    }
    results.append(("the pad-geometry guard is not vacuous",
                    len(_pad_geometry_askers()) >= 1))
    results.append(("only the shared predicate asks pad-contact geometry "
                    "(#722)", _pad_geometry_askers() == _ALLOWED))

    # 17. Two under-reports this branch introduced and a review caught. Both
    #     are the SAME mistake the fix exists to correct, made while making it:
    #     a credit widened without the guard that keeps it honest.
    #
    #     (a) VIA LAYERS. Crediting the endpoint's cap against a via barrel
    #         (correct) while ignoring which layers that barrel occupies
    #         (wrong) makes a layer-blind credit ~37% WIDER than the
    #         `vr + 0.01` it replaced. A blind F.Cu/In1.Cu via then anchors
    #         B.Cu track ends it carries no copper for, silencing a real open.
    def _blind_via_board(seg_layer):
        pads = [_pad(-3, 0, layers=(seg_layer,), num='1'),
                _pad(3, 0, layers=(seg_layer,), num='2', ref='U2')]
        segs = [_seg(-3, 0, -0.35, -0.06, layer=seg_layer, width=0.25),
                _seg(-0.35, 0.06, -3, 0, layer=seg_layer, width=0.25)]
        v = Via(x=0, y=0, size=0.6, drill=0.3, layers=['F.Cu', 'In1.Cu'],
                net_id=NET)
        pcb = _pcb(segs, vias=[v], pads=pads)
        pcb.board_info.copper_layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
        return pcb, segs, [v], pads

    pcb_b, segs_b, vias_b, pads_b = _blind_via_board('B.Cu')
    conn_b = check_net_connectivity(NET, segs_b, vias_b, pads_b, [])
    results.append(("a blind F.Cu/In1.Cu via carries no B.Cu copper: "
                    "check_connected calls this net SPLIT",
                    conn_b['num_components'] > 1
                    or bool(conn_b['disconnected_pads'])))
    results.append(("...and check_weird agrees: still a soft joint",
                    _sj(pcb_b) == 1))
    #     Control: the same via and the same geometry on a layer the barrel
    #     DOES occupy anchors the ends, so the row above is the LAYER test
    #     firing and not the predicate refusing every via.
    results.append(("the same ends on a layer the barrel occupies are "
                    "anchored: clean", _sj(_blind_via_board('F.Cu')[0]) == 0))

    #     (b) A soft joint is a PAIR. Excluding a copper-layer GRAPHIC as a
    #         CANDIDATE drops the TRACK end paired with it, and nothing else
    #         picks that end up -- so a genuinely open net went from exit 1 to
    #         exit 0, which is what check_complete gates DONE on. The art half
    #         is unactionable (#337); the track half is not.
    _gp = [_pad(-6, 0, num='1'), _pad(6, 0, num='2', ref='U2')]
    _track = _seg(-6, 0, -0.05, 0, width=0.25)
    _artseg = Segment(start_x=0.05, start_y=0, end_x=6, end_y=0, width=0.25,
                      layer='F.Cu', net_id=NET, graphic=True)
    _mixed = _pcb([_track, _artseg], pads=_gp)
    _cm = check_net_connectivity(NET, [_track, _artseg], [], _gp, [])
    results.append(("a track ending short of ART is a real open "
                    "(check_connected)",
                    _cm['num_components'] > 1 or bool(_cm['disconnected_pads'])))
    results.append(("...and the track-vs-art pair is still reported",
                    _sj(_mixed) == 1))
    _rep = [x for x in check_weird(_mixed, tolerance=0)[0]
            if x['category'] == 'soft-joint']
    results.append(("...reported at the TRACK end, where the fix goes",
                    len(_rep) == 1 and abs(_rep[0]['x'] - (-0.05)) < 1e-6))
    #     Control: art meeting art has no actionable end, and is dropped.
    _art2 = Segment(start_x=-6, start_y=0, end_x=-0.05, end_y=0, width=0.25,
                    layer='F.Cu', net_id=NET, graphic=True)
    results.append(("art meeting art is NOT reported (nothing to act on)",
                    _sj(_pcb([_art2, _artseg], pads=_gp)) == 0))

    # 18. The pad LAYER SET comes from net_queries.expand_pad_layers -- the
    #     same expansion check_connected uses -- not from a local reading of
    #     pad.layers. Two ways a local reading goes wrong, both caught by
    #     review rather than by any board in kicad_files/.
    from connectivity import endpoint_reaches_pad
    from check_connected import check_net_connectivity as _cnc

    #     (a) `*.Mask` is not copper. A pad carrying it is an ordinary
    #         single-layer SMD pad, and 641 pads in kicad_files/ carry one --
    #         but `any('*' in L)` reads the wildcard as "every copper layer"
    #         and hands a B.Cu end an F.Cu-only pad.
    _mask = _rect_pad(0, 0, 1.0, 0.6, layers=('F.Cu', '*.Mask', '*.Paste'),
                      num='1')
    results.append(("a mask wildcard is not copper: the pad reaches F.Cu",
                    endpoint_reaches_pad(0, 0, 0.125, ('F.Cu',), _mask)
                    == {'F.Cu'}))
    results.append(("...and reaches NO other layer (a mask wildcard is not "
                    "an all-copper pad)",
                    endpoint_reaches_pad(0, 0, 0.125, ('B.Cu',), _mask)
                    == set()
                    and endpoint_reaches_pad(0, 0, 0.125, ('In1.Cu',), _mask)
                    == set()))
    #         Control: the copper wildcard DOES span every layer asked about,
    #         so the row above is the *.Cu/*.Mask distinction firing and not
    #         the predicate refusing wildcards outright.
    _cuwild = _rect_pad(0, 0, 1.0, 0.6, layers=('*.Cu', '*.Mask'), num='1')
    results.append(("a *.Cu wildcard spans every layer asked about (control)",
                    endpoint_reaches_pad(0, 0, 0.125,
                                         ('F.Cu', 'In1.Cu', 'B.Cu'), _cuwild)
                    == {'F.Cu', 'In1.Cu', 'B.Cu'}))

    #     (b) A DRILLED pad occupies the layers it declares, not every layer.
    #         `drill > 0 -> all layers` over-credits a PTH pad declared
    #         ("F.Cu" "B.Cu") on a 4-layer board; check_connected grades an
    #         In1.Cu end as not reaching it, and the agreement row pins that
    #         absolutely rather than merely asserting the two differ.
    _pth = _rect_pad(0, 0, 1.0, 0.6, layers=('F.Cu', 'B.Cu', '*.Mask'),
                     num='1')
    _pth.drill, _pth.pad_type = 0.4, 'thru_hole'
    results.append(("a drilled pad declared F.Cu/B.Cu reaches B.Cu",
                    endpoint_reaches_pad(0, 0, 0.125, ('B.Cu',), _pth)
                    == {'B.Cu'}))
    results.append(("...and does NOT reach In1.Cu, which it never declared",
                    endpoint_reaches_pad(0, 0, 0.125, ('In1.Cu',), _pth)
                    == set()))
    _far = _rect_pad(-3, 0, 1.0, 0.6, layers=('In1.Cu',), num='2', ref='U2')
    _in1 = [_seg(-3, 0, 0, 0, layer='In1.Cu', width=0.25)]
    _c = _cnc(NET, _in1, [], [_pth, _far], [])
    results.append(("check_connected agrees the In1.Cu end does not reach it: "
                    "the net is SPLIT",
                    _c['num_components'] > 1 or bool(_c['disconnected_pads'])))

    # 19. _point_anchored's pad test is a bounding CIRCLE of radius
    #     max(size_x, size_y)/2, and in _check_dangles it ran BEFORE the exact
    #     one -- so it could only ADD credit and the exact test never got to
    #     refuse. A 0.3mm track ending 0.575mm from a 1.8x0.45 pad's copper is
    #     0.9mm from its centre, inside the bounding circle and nowhere near
    #     the copper. Pads are answered by endpoint_reaches_pad alone now.
    _long = _rect_pad(0, 0, 1.8, 0.45, num='1')
    _lp2 = [_long, _rect_pad(-6, 0, 1.8, 0.45, num='2', ref='U2')]
    _lseg = [_seg(-6, 0, -0.9, 0.42, width=0.3)]
    results.append(("the long-pad row is ON its branch: inside the bounding "
                    "circle, outside the copper",
                    point_to_pad_distance(-0.9, 0.42, _long) > 0.15
                    and math.hypot(-0.9, 0.42) < 1.8 / 2 + 0.15))
    _lc = check_net_connectivity(NET, _lseg, [], _lp2, [])
    results.append(("check_connected calls that end unconnected to the pad",
                    _lc['num_components'] > 1 or bool(_lc['disconnected_pads'])))
    results.append(("...and check_weird no longer credits it by the bounding "
                    "circle: a dangling end",
                    len([x for x in check_weird(_pcb(_lseg, pads=_lp2),
                                                tolerance=0)[0]
                         if x['category'] == 'dangling-end']) >= 1))
    #     Control: an end genuinely ON that pad's copper is still anchored, so
    #     the row above is the exact test firing and not a blanket refusal.
    results.append(("an end on the long pad's real copper is still anchored",
                    not [x for x in check_weird(
                        _pcb([_seg(-6, 0, -0.5, 0, width=0.3)], pads=_lp2),
                        tolerance=0)[0]
                        if x['category'] == 'dangling-end']))

    # 20. narrow-pad-joint on a ROUND pad (#416 follow-through). The shape gate
    #     skipped circles -- "a circle has no corner to graze". It has no
    #     corner, but it has a RIM, and the same connection_width hazard lives
    #     there: a cap landing just outside the rim joins through a lens whose
    #     chord is far below the floor. _pad_web_polygon has handled circles all
    #     along; only the pre-filter and the gate did not.
    def _rim(d, shape='circle', w=0.2, R=0.5, floor=0.1):
        pad = _pad(0, 0, size=2 * R, num='1')
        pad.shape = shape
        far = _pad(6, 0, size=2 * R, num='2', ref='U2')
        far.shape = shape
        segs = [_seg(6, 0, d, 0, width=w),
                _seg(-9, -9, -8, -9, width=floor)]   # sets the min-track floor
        return _pcb(segs, pads=[pad, far])

    results.append(("a cap grazing a ROUND pad's rim is a narrow-pad-joint",
                    len([x for x in check_weird(_rim(0.59), tolerance=0)[0]
                         if x['category'] == 'narrow-pad-joint']) == 1))
    #     Control: the same cap biting deeper joins through full copper.
    results.append(("the same cap deeper into the round pad is NOT flagged",
                    not [x for x in check_weird(_rim(0.52), tolerance=0)[0]
                         if x['category'] == 'narrow-pad-joint']))

    # 11. The reporter, which is what #696 actually broke: a finding whose
    #     category is missing from CATEGORIES counted toward the headline and
    #     the exit code but printed nothing, so the board was blocked by a
    #     defect that was never named. Both halves are pinned here.
    emitted = _emitted_categories()
    results.append(("the source emits categories at all (guard not vacuous)",
                    len(emitted) >= 8))
    # Set EQUALITY, not containment, so the guard holds in both directions.
    # A registered category nobody emits is the mirror defect: it prints a
    # permanent `: 0` line that no board can ever produce, and it is exactly
    # what a rename like this one leaves behind when only one side is edited.
    results.append(("CATEGORIES is exactly the set of emitted categories",
                    emitted == set(CATEGORIES)))

    stray = [{'category': 'brand-new-category', 'net': NAME, 'layer': 'F.Cu',
              'x': 1.0, 'y': 2.0, 'detail': 'a category nobody registered',
              'size': None}]
    buf = io.StringIO()
    with redirect_stdout(buf):
        print_report(stray, [])
    out = buf.getvalue()
    results.append(("print_report names a category missing from CATEGORIES",
                    'brand-new-category: 1' in out
                    and 'a category nobody registered' in out))
    counts = [int(m) for m in re.findall('^  [^ ]+: ([0-9]+)$', out, re.M)]
    head = re.search('FOUND ([0-9]+) WEIRD THINGS', out)
    results.append(("the headline count equals the sum of printed counts",
                    head is not None and bool(counts)
                    and int(head.group(1)) == sum(counts) == 1))

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} check_weird tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
