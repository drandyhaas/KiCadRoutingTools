#!/usr/bin/env python3
"""#734: a narrow --nets run must not route or strip the rest of the board.

The bug: default-on pre-existing rip candidacy registers every small OPEN
pre-existing net (<=30 seg / <=6 via) into routed_results as a custody
entry. On a mid-chain board (fanout stubs everywhere) the summary audit
and the fragment sweep then minted failed_multipoint entries for ALL of
them, and the final reconciliation enrolled and ROUTED the whole board --
measured on allwinner: a 2-net run routed 48 extras (the reconcile
sub-run also prices layers on its own AUTO ladder, so the collateral
copper is routed under different economics than the requested nets). The
GUI shares batch_route, so a one-net GUI selection on such a board did
the same.

The fix: candidacy makes a net rip-ELIGIBLE; responsibility transfers at
an ACTUAL rip. Untouched registered candidates are exempt from the
connectivity audit's failure minting (with their stale failed_pads_info
cleared, not just skipped) and from the fragment sweep's re-enrollment.

This test builds a small board with 6 two-pad nets, each carrying an
open fanout-style stub (the candidacy trigger), routes ONE net, and
asserts the other nets' copper is byte-identical in the output. It also
asserts candidacy actually REGISTERED the bystander nets -- without that
the gate under test is never exercised and the pass would be vacuous
(the run_utils doctrine: a test whose trigger never fired proves
nothing).
"""
import os
import io
import math
import sys
import tempfile
import contextlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'py_router'))

fails = []


def check(name, cond):
    print(("  ok  " if cond else " FAIL ") + name)
    if not cond:
        fails.append(name)


def _board_text():
    """6 nets, one SMD pad pair each (left column U1, right column U2),
    every net with a short open stub off its left pad -- exactly the
    mid-chain fanout shape that triggers pre-existing rip candidacy."""
    nets = ''.join(f' (net {i} "N{i}")\n' for i in range(1, 7))
    pads_l = ''.join(
        f'  (pad "{i}" smd rect (at 0 {i * 2}) (size 1 0.6) '
        f'(layers "F.Cu") (net {i} "N{i}"))\n' for i in range(1, 7))
    pads_r = ''.join(
        f'  (pad "{i}" smd rect (at 0 {i * 2}) (size 1 0.6) '
        f'(layers "F.Cu") (net {i} "N{i}"))\n' for i in range(1, 7))
    # Stubs START ON the pad (a pad-less island would be legitimately
    # removed by the late orphan sweep, not by the bug under test).
    stubs = ''.join(
        f' (segment (start 6 {15 + i * 2}) (end 9 {15 + i * 2}) '
        f'(width 0.15) (layer "F.Cu") (net {i}) '
        f'(uuid "00000000-0000-0000-0000-00000000000{i}"))\n'
        for i in range(1, 7))
    return (
        '(kicad_pcb (version 20240108) (generator test)\n'
        ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
        ' (net 0 "")\n' + nets +
        ' (gr_rect (start 0 0) (end 46 42) (layer "Edge.Cuts") (width 0.1))\n'
        ' (footprint "test:conn" (layer "F.Cu") (at 6 15)\n'
        '  (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
        + pads_l + ' )\n'
        ' (footprint "test:conn" (layer "F.Cu") (at 40 15)\n'
        '  (property "Reference" "U2" (at 0 0) (layer "F.SilkS"))\n'
        + pads_r + ' )\n'
        + stubs + ')\n')


def _copper_mm(pcb, nid):
    """Total routed copper on a net, in mm.

    The "did the requested net route" assertion below used to count SEGMENTS,
    which is a proxy, not the property. #811's collinear merge joins a route to
    the collinear pre-existing stub it continues, and this fixture is the worst
    case for that proxy: N1's two pads are on one horizontal line and its stub
    lies along it, so the whole routed net legitimately collapses to a SINGLE
    segment. Measured, both arms: N1 copper 3.0 -> 34.0 mm, while the count
    went 1 -> 2 with the merge off and 1 -> 1 with it on. Counting segments
    called the second one a routing failure. Copper is what "gained copper"
    means, so measure copper.
    """
    return math.fsum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                     for s in pcb.segments if s.net_id == nid)


def _seg_sigs(pcb, nid):
    out = set()
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        a = (round(s.start_x, 4), round(s.start_y, 4))
        b = (round(s.end_x, 4), round(s.end_y, 4))
        out.add((min(a, b), max(a, b), s.layer, round(s.width, 4)))
    return out


def test_narrow_scope():
    import route
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, 'in.kicad_pcb')
        dst = os.path.join(td, 'out.kicad_pcb')
        with open(src, 'w') as f:
            f.write(_board_text())
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            route.batch_route(src, dst, ['N1'],
                              layers=['F.Cu', 'B.Cu'],
                              track_width=0.15, clearance=0.15,
                              grid_step=0.1)
        out = buf.getvalue()

        # Vacuity guard: candidacy must have REGISTERED the bystanders,
        # or the gate under test never ran and this pass proves nothing.
        check("candidacy registered pre-existing nets (gate exercised)",
              'registered as main-pass rip candidates' in out)

        before = parse_kicad_pcb(src)
        after = parse_kicad_pcb(dst)

        def _ids(pcb):
            out = {}
            for fp in pcb.footprints.values():
                for p in fp.pads:
                    if p.net_name:
                        out[p.net_name] = p.net_id
            return out
        ids_b = _ids(before)
        ids_a = _ids(after)

        # The requested net routed. Graded on COPPER, not segment count --
        # see _copper_mm for why the count is the wrong instrument here.
        _n1_before = _copper_mm(before, ids_b['N1'])
        _n1_after = _copper_mm(after, ids_a['N1'])
        check(f"requested net N1 gained copper "
              f"({_n1_before:.2f} -> {_n1_after:.2f} mm)",
              _n1_after > _n1_before + 1e-6)

        # Every bystander's copper is byte-identical: not routed by the
        # reconciliation, not trimmed by the sweeps.
        for i in range(2, 7):
            check(f"bystander N{i} copper untouched",
                  _seg_sigs(before, ids_b[f'N{i}'])
                  == _seg_sigs(after, ids_a[f'N{i}']))

        # And the reconciliation was not launched for the bystanders.
        check("no whole-board reconciliation",
              'Final reconciliation: retrying' not in out
              or all(f'N{i}' not in
                     out.split('Final reconciliation: retrying', 1)[1]
                     .split('\n', 1)[0]
                     for i in range(2, 7)))


if __name__ == '__main__':
    test_narrow_scope()
    if fails:
        print(f"\n{len(fails)} FAILURE(S)")
        sys.exit(1)
    print("\nALL PASS")
