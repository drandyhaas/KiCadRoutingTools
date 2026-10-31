#!/usr/bin/env python3
"""#1550: a span stacked twice counts once when a soft joint is looked for.

Two plane-repair taps laid along one corridor stack every segment of it twice
(same net, same span, often in opposite directions). The soft-joint
detectors -- the graze prune's gate (`pcb_modification._soft_joint_pairs`),
check_drc's `segment-endpoint-gap` and check_weird's `soft-joint` -- told a
free end (degree 1) from a shared vertex by counting the segments that END
there, so where a run was stacked, every end read degree 2. The graze prune
then removed both copies of a short grazing span with the net still
connected (pads only), and the two ends it left facing each other touched by
their caps alone: a soft joint its own gate exists to refuse, invisible
because each end still had its twin. The end-of-run dedup (#162) then kept
one copy of each surviving span and shipped the track as 0.1 mm shards
joined by cap overlap. The degree is now counted over DISTINCT spans
(`connectivity.span_endpoint_degree`), in all three. What each case pins:

* the shared degree counts a stacked span once, either direction;
* the three detectors see the soft joint between two stacked tails exactly
  as they see it between two lone ones, and still see no joint where the
  stacked run is whole;
* the graze prune, handed a stacked run whose short middle span grazes a
  foreign pad, no longer removes both copies of it (it necks the span clear
  instead), so it opens no soft joint; the pads stay connected.

    python3 tests/test_1550_stacked_copy_soft_joint.py [case-substring ...]
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))


def _board(path, segments, pads):
    """segments: (x1, y1, x2, y2, w, net); pads: (ref, x, y, sx, sy, net)."""
    nets = sorted({s[5] for s in segments} | {p[5] for p in pads})
    nid = {n: i + 1 for i, n in enumerate(nets)}
    fps = ''.join(f'''
 (footprint "L:P" (layer "F.Cu") (at 0 0)
  (property "Reference" "{ref}" (at 0 -3 0) (layer "F.SilkS") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
  (pad "1" smd rect (at {x} {y}) (size {sx} {sy}) (layers "F.Cu" "F.Mask") (net {nid[n]} "{n}")))'''
                  for ref, x, y, sx, sy, n in pads)
    segs = ''.join(f'\n (segment (start {a} {b}) (end {c} {d}) (width {w}) '
                   f'(layer "F.Cu") (net {nid[n]}))'
                   for a, b, c, d, w, n in segments)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(f'''(kicad_pcb (version 20241229) (generator "pcbnew") (general (thickness 1.6))
 (paper "A4")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (37 "F.SilkS" user "F.Silkscreen")
  (39 "F.Mask" user) (44 "Edge.Cuts" user))
 (setup)
 (net 0 ""){''.join(f' (net {i} "{n}")' for n, i in nid.items())}
 (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts")){fps}{segs}
)
''')


def _stacked(points, w, net):
    """A run through `points`, laid twice: once forward, once reversed."""
    run = [(a[0], a[1], b[0], b[1], w, net) for a, b in zip(points, points[1:])]
    return run + [(c, d, a, b, w2, n) for a, b, c, d, w2, n in run]


def test_span_degree_counts_a_stacked_span_once():
    from types import SimpleNamespace as NS
    from connectivity import span_endpoint_degree
    segs = [NS(layer='F.Cu', start_x=0, start_y=0, end_x=1, end_y=0),
            NS(layer='F.Cu', start_x=1, start_y=0, end_x=0, end_y=0),
            NS(layer='F.Cu', start_x=1, start_y=0, end_x=2, end_y=0),
            NS(layer='B.Cu', start_x=1, start_y=0, end_x=2, end_y=0)]
    deg = span_endpoint_degree(segs)
    assert deg[('F.Cu', (0, 0))] == 1, deg
    assert deg[('F.Cu', (1, 0))] == 2, deg
    assert deg[('B.Cu', (1, 0))] == 1, deg
    print("  PASS: a stacked span counts once, either direction; layers apart")


# Two tails of net A whose ends face each other 0.2 mm apart (caps of 0.3 mm
# track overlap by 0.1): a soft joint. Each tail ends on its own pad.
TAIL_PTS = ([(2.0, 5.0), (3.0, 5.0), (4.9, 5.0)],
            [(5.1, 5.0), (7.0, 5.0), (8.0, 5.0)])
TAIL_PADS = [('P1', 2.0, 5.0, 0.6, 0.6, 'A'), ('P2', 8.0, 5.0, 0.6, 0.6, 'A')]


def _detectors(path):
    """(prune gate pairs, check_drc soft joints, check_weird soft joints)."""
    from kicad_parser import parse_kicad_pcb
    from pcb_modification import _soft_joint_pairs
    from check_drc import run_drc
    from check_weird import check_weird
    pcb = parse_kicad_pcb(path)
    a = next(i for i, n in pcb.nets.items() if n.name == 'A')
    gate = _soft_joint_pairs([s for s in pcb.segments if s.net_id == a], [],
                             pcb.pads_by_net.get(a, []))
    with contextlib.redirect_stdout(io.StringIO()):
        drc = run_drc(path, clearance=0.1, quiet=True, print_summary=False,
                      check_sizes=False)
        weird, _skip = check_weird(pcb, quiet=True, tolerance=0.0)
    drc_sj = [v for v in drc if v.get('type') == 'segment-endpoint-gap']
    weird_sj = [f for f in weird if f.get('category') == 'soft-joint']
    return len(gate), len(drc_sj), len(weird_sj)


def test_detectors_see_a_joint_between_stacked_tails():
    with tempfile.TemporaryDirectory() as td:
        lone = os.path.join(td, 'lone.kicad_pcb')
        segs = [(a[0], a[1], b[0], b[1], 0.3, 'A')
                for pts in TAIL_PTS for a, b in zip(pts, pts[1:])]
        _board(lone, segs, TAIL_PADS)
        assert _detectors(lone) == (1, 1, 1), \
            f"fixture: the lone tails are one soft joint {_detectors(lone)}"
        stacked = os.path.join(td, 'stacked.kicad_pcb')
        _board(stacked, [s for pts in TAIL_PTS for s in _stacked(pts, 0.3, 'A')],
               TAIL_PADS)
        got = _detectors(stacked)
        assert got == (1, 1, 1), \
            f"stacked tails: (gate, check_drc, check_weird) = {got}, want 1 each"
        whole = os.path.join(td, 'whole.kicad_pcb')
        _board(whole, _stacked([(2.0, 5.0), (3.0, 5.0), (4.9, 5.0), (5.1, 5.0),
                                (7.0, 5.0), (8.0, 5.0)], 0.3, 'A'), TAIL_PADS)
        assert _detectors(whole) == (0, 0, 0), _detectors(whole)
    print("  PASS: one soft joint between stacked tails, as between lone ones, "
          "in the gate, check_drc and check_weird; none on the whole run")


def test_graze_prune_opens_no_joint_in_a_stacked_run():
    """A stacked run P1 -> P2 along y=5 whose 0.2 mm middle span (0.5 wide)
    grazes foreign pad B; its 0.3 wide neighbours do not. A second, lone
    track below joins P1 and P2, so pad-wise the middle span is redundant."""
    from kicad_parser import parse_kicad_pcb
    from pcb_modification import prune_grazing_segments, _soft_joint_pairs
    from check_connected import check_net_connectivity
    pts = [(2.0, 5.0), (3.0, 5.0), (4.9, 5.0), (5.1, 5.0), (7.0, 5.0),
           (8.0, 5.0)]
    run = []
    for a, b in zip(pts, pts[1:]):
        w = 0.5 if a[0] == 4.9 else 0.3
        run.append((a[0], a[1], b[0], b[1], w, 'A'))
    run = run + [(c, d, a, b, w, n) for a, b, c, d, w, n in run]
    alt = [(2.0, 5.0, 2.0, 8.0, 0.3, 'A'), (2.0, 8.0, 8.0, 8.0, 0.3, 'A'),
           (8.0, 8.0, 8.0, 5.0, 0.3, 'A')]
    # B's pad: its bottom edge 0.5 above the run's centre line. The middle
    # span's threshold is 0.3 + 0.25 = 0.55 (grazes); its neighbours' is
    # 0.3 + 0.15 = 0.45 and their nearest point is >= 0.5 away (clear).
    pads = TAIL_PADS + [('B1', 5.0, 4.4, 0.1, 0.2, 'B'),
                        ('B2', 15.0, 15.0, 0.6, 0.6, 'B')]
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'run.kicad_pcb')
        _board(path, run + alt, pads)
        pcb = parse_kicad_pcb(path)
        a = next(i for i, n in pcb.nets.items() if n.name == 'A')
        mine = [s for s in pcb.segments if s.net_id == a]
        middle = [s for s in mine if {round(s.start_x, 3), round(s.end_x, 3)}
                  == {4.9, 5.1}]
        assert len(middle) == 2, f"fixture: two copies of the middle span"
        results = [{'net_id': a, 'new_segments': list(mine)}]
        base = _soft_joint_pairs(mine, [], pcb.pads_by_net[a])
        with contextlib.redirect_stdout(io.StringIO()):
            n, _nets, _strip = prune_grazing_segments(
                results, pcb, {a}, clearance=0.3, check_foreign_segments=True)
        kept = [s for s in pcb.segments if s.net_id == a]
        kept_mid = [s for s in kept if any(s is m for m in middle)]
        assert kept_mid, \
            f"both copies of the grazing middle span were removed ({n} " \
            f"segment(s)): the stacked tails left facing each other"
        new = _soft_joint_pairs(kept, [], pcb.pads_by_net[a]) - base
        assert not new, f"the prune opened a soft joint: {new}"
        r = check_net_connectivity(a, kept, [], pcb.pads_by_net[a], [],
                                   pcb_data=pcb)
        assert r.get('connected'), r
    print(f"  PASS: the prune removed {n} segment(s), kept the middle span "
          f"(width {kept_mid[0].width}), opened no soft joint")


TESTS = [test_span_degree_counts_a_stacked_span_once,
         test_detectors_see_a_joint_between_stacked_tails,
         test_graze_prune_opens_no_joint_in_a_stacked_run]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
