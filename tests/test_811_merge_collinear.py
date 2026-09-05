#!/usr/bin/env python3
"""Issue #811: a dead-straight track shipped as two or three separate segments.

`simplify_path` collapses collinear points at PATH level, before copper is
emitted; everything after that can re-introduce a collinear joint, and nothing
joined them back. `merge_collinear_segments` is that missing final pass.

Measured sources of the joints it cleans up, per-pass instrumented on
splitflap_driver (25 at the end of the pipeline): smoothing +10 (its elbow lands
collinear with the neighbouring kept segment), route emission 11 (terminal
exact-pad stubs `_merge_terminal_to_exact` declined, and multipoint links
simplified INDEPENDENTLY then joined), cycle prune +4, soft-joint close +1.

WHAT THIS TEST IS REALLY PINNING is that the pass MOVES NO COPPER. That is the
whole licence for running it after `close_soft_joints`, for skipping the
clearance/connectivity guards every other shape pass carries, and for being on
by default with no per-front switch. Two independent statements of it here:

  * the union invariant, on every synthetic case -- total copper length is
    unchanged and each original capsule still lies inside an emitted one;
  * the refusals, which are the interesting half. A vertex is dropped only when
    it lies within `max_deviation` (1 nm, KiCad's own internal unit) of the line
    joining its neighbours. The population this pass exists for is bit-exactly
    collinear -- deviation <= 1e-12 mm on all 322/25/81 joints measured across
    routed_output / splitflap_driver / interf_u_routed -- while genuine
    sub-degree KINKS sit at 1.2-2.4 um. So the tolerance separates the two
    classes by six orders of magnitude, and `a_kink_is_not_a_split` pins the
    real interf_u geometry that must survive.

The via / tee refusals are NOT geometry: merging across either leaves the board
identical but strands that branch in a connectivity model that joins copper at
ENDPOINTS. The pad rule is the same concern resolved the other way -- pads are
carried as per-vertex CUSTODY rather than a veto, because pad-covered joints are
15 of splitflap's 25 (the terminal stubs that are the issue's own screenshot),
so vetoing them outright would have left the reported case unfixed.

    python3 tests/test_811_merge_collinear.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pcb_modification import merge_collinear_segments
from synth import make_pad, make_pcb, make_seg, make_via

W = 0.2


def _pcb(segs, pads=None, vias=()):
    return make_pcb(segments=list(segs), vias=list(vias),
                    pads_by_net=(pads if pads is not None else {}))


def _len(segs):
    return math.fsum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                     for s in segs)


def _inside(s, t, tol=1e-6):
    """Is capsule `s` contained in capsule `t` (same width/layer assumed)?"""
    dx, dy = t.end_x - t.start_x, t.end_y - t.start_y
    L = math.hypot(dx, dy)
    if L < 1e-12:
        return False
    ux, uy = dx / L, dy / L
    for px, py in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
        perp = abs(-uy * (px - t.start_x) + ux * (py - t.start_y))
        along = ux * (px - t.start_x) + uy * (py - t.start_y)
        if perp > tol or along < -tol or along > L + tol:
            return False
    return True


def _run_pass(segs, pads=None, vias=(), **kw):
    """Run the pass on a fresh board; return (board, before, stats)."""
    before = [make_seg(s.start_x, s.start_y, s.end_x, s.end_y,
                       layer=s.layer, net_id=s.net_id, width=s.width)
              for s in segs]
    pcb = _pcb(segs, pads, vias)
    results = []
    n, nets, strip, added, stats = merge_collinear_segments(
        results, pcb, None, **kw)
    return pcb, before, stats, results, strip, added


def _copper_preserved(check, label, pcb, before):
    """The union invariant: same total length, and every original capsule is
    still inside an emitted one."""
    same_len = abs(_len(pcb.segments) - _len(before)) < 1e-9
    covered = all(any(_inside(s, t) for t in pcb.segments
                      if t.layer == s.layer and abs(t.width - s.width) < 1e-9)
                  for s in before)
    check(f'{label}: copper length unchanged '
          f'({_len(before):.6f} -> {_len(pcb.segments):.6f} mm)', same_len)
    check(f'{label}: every original capsule still covered', covered)


def a_straight_run_becomes_one_segment(check):
    """Three collinear pieces -- the #811 report -- collapse to one track."""
    segs = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
            make_seg(1.0, 0.0, 1.1, 0.0, width=W),      # the one-grid-step crumb
            make_seg(1.1, 0.0, 5.0, 0.0, width=W)]
    pcb, before, stats, _r, _s, _a = _run_pass(segs)
    check('3 collinear pieces -> 1 segment', len(pcb.segments) == 1)
    check('  and both joints are counted', stats['joints'] == 2)
    s = pcb.segments[0]
    check('  spanning the original extent',
          abs(s.start_x - 0.0) < 1e-9 and abs(s.end_x - 5.0) < 1e-9)
    _copper_preserved(check, '  straight run', pcb, before)


def a_kink_is_not_a_split(check):
    """The real near-collinear outliers measured on interf_u_routed: 0.75 deg /
    1.24 um and 1.44 deg / 2.40 um off the joining line. Those are BENDS -- the
    copper genuinely turns -- and merging them would move copper by microns."""
    for ang_deg, dev in ((0.7502, 1.243280e-3), (1.4448, 2.396555e-3)):
        # Symmetric vee reproducing BOTH measured numbers: the half-angle sets
        # the bearing, and the leg length is then whatever puts the apex the
        # measured distance off the line joining the two far ends.
        half = math.radians(ang_deg) / 2.0
        leg = dev / math.sin(half)
        segs = [make_seg(-leg * math.cos(half), 0.0, 0.0, dev, width=W),
                make_seg(0.0, dev, leg * math.cos(half), 0.0, width=W)]
        pcb, before, stats, _r, _s, _a = _run_pass(segs)
        check(f'a {ang_deg:.4f} deg kink ({dev*1000:.2f} um off the line) is '
              f'NOT merged', len(pcb.segments) == 2 and stats['joints'] == 0)
        check(f'  (deviation {dev*1000:.3f} um = {dev/1e-6:.0f}x the 1 nm '
              f'tolerance; the real #811 population sits under 1e-6 um)',
              dev > 1e-6)
        _copper_preserved(check, f'  {ang_deg:.4f} deg kink', pcb, before)


def doubling_back_is_never_merged(check):
    """Two legs pointing opposite ways: their union is SHORTER than the joined
    span, so 'merging' would ADD copper. That is same-net overlapping copper
    (#606), a different defect with a different fix."""
    segs = [make_seg(0.0, 0.0, 2.0, 0.0, width=W),
            make_seg(2.0, 0.0, 1.5, 0.0, width=W)]        # backtracks
    pcb, before, stats, _r, _s, _a = _run_pass(segs)
    check('a doubled-back pair is left alone (#606, not #811)',
          len(pcb.segments) == 2 and stats['joints'] == 0)
    _copper_preserved(check, '  doubled-back', pcb, before)


def a_via_or_tee_holds_the_joint(check):
    """Not geometry -- model custody. Merging across either leaves the copper
    identical but strands that branch in a connectivity model that joins at
    ENDPOINTS."""
    segs = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
            make_seg(1.0, 0.0, 2.0, 0.0, width=W)]
    pcb, _b, stats, _r, _s, _a = _run_pass(list(segs),
                                           vias=[make_via(1.0, 0.0, net_id=1)])
    check('a same-net VIA on the joint holds it', len(pcb.segments) == 2)

    tee = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
           make_seg(1.0, 0.0, 2.0, 0.0, width=W),
           make_seg(1.0, 0.0, 1.0, 1.0, width=W)]          # third endpoint
    pcb, _b, stats, _r, _s, _a = _run_pass(tee)
    check('a same-net TEE on the joint holds it', len(pcb.segments) == 3)

    # a tee in ANOTHER width/layer group still counts: `inc` spans all of them
    tee2 = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
            make_seg(1.0, 0.0, 2.0, 0.0, width=W),
            make_seg(1.0, 0.0, 1.0, 1.0, width=0.5)]
    pcb, _b, stats, _r, _s, _a = _run_pass(tee2)
    check('  including a tee of a DIFFERENT width', len(pcb.segments) == 3)


def width_and_layer_changes_are_deliberate(check):
    """A width change is the neck-down (`_split_segment_at`); a layer change is
    two different tracks. Neither is a redundant split."""
    neck = [make_seg(0.0, 0.0, 1.0, 0.0, width=0.5),
            make_seg(1.0, 0.0, 2.0, 0.0, width=W)]
    pcb, _b, _st, _r, _s, _a = _run_pass(neck)
    check('a width change (neck-down) is never merged away',
          len(pcb.segments) == 2)

    lay = [make_seg(0.0, 0.0, 1.0, 0.0, width=W, layer='F.Cu'),
           make_seg(1.0, 0.0, 2.0, 0.0, width=W, layer='B.Cu')]
    pcb, _b, _st, _r, _s, _a = _run_pass(lay)
    check('a layer change is never merged away', len(pcb.segments) == 2)


def locked_copper_is_never_touched(check):
    """KiCad `(locked yes)`: the user pinned that exact track (#521 doctrine --
    locked copper has no override anywhere else either)."""
    segs = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
            make_seg(1.0, 0.0, 2.0, 0.0, width=W)]
    segs[1].locked = True
    pcb, _b, stats, _r, _s, _a = _run_pass(segs)
    check('a LOCKED segment is never merged', len(pcb.segments) == 2)


def pad_custody_is_carried_not_vetoed(check):
    """The terminal exact-pad stub -- 15 of splitflap's 25 joints, and the
    issue's own screenshot. The joint sits INSIDE the pad copper, so smoothing's
    anchor rule would veto it; here the pad still covers the surviving endpoint,
    so the merge is safe and happens."""
    pad = make_pad(net_id=1, x=2.0, y=0.0, size_x=0.8, size_y=0.8)
    segs = [make_seg(0.0, 0.0, 1.94, 0.0, width=W),
            make_seg(1.94, 0.0, 2.0, 0.0, width=W)]      # the 0.06mm pad stub
    pcb, before, stats, _r, _s, _a = _run_pass(segs, pads={1: [pad]})
    check('a terminal pad stub IS merged (the pad still holds the endpoint)',
          len(pcb.segments) == 1)
    _copper_preserved(check, '  pad stub', pcb, before)

    # ... but a pad whose ONLY contact is the dropped vertex keeps it.
    mid = make_pad(net_id=1, x=1.0, y=0.0, size_x=0.3, size_y=0.3)
    far = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
           make_seg(1.0, 0.0, 8.0, 0.0, width=W)]
    pcb, _b, stats, _r, _s, _a = _run_pass(far, pads={1: [mid]})
    check('a pad covering ONLY the dropped vertex holds that vertex',
          len(pcb.segments) == 2)


def the_ledger_stays_balanced(check):
    """Pipeline contract: this-run copper is swapped inside `results` (the write
    list), INPUT copper is reported for the writer to strip. The board and the
    write model must not disagree."""
    routed = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
              make_seg(1.0, 0.0, 2.0, 0.0, width=W)]
    pcb = _pcb(routed)
    results = [{'new_segments': list(routed), 'new_vias': []}]
    n, nets, strip, added, stats = merge_collinear_segments(results, pcb, None)
    wl = [s for r in results for s in (r.get('new_segments') or [])]
    check('routed copper: write-list == board after the merge',
          len(wl) == 1 and len(pcb.segments) == 1 and wl[0] is pcb.segments[0])
    check('  and nothing is reported as an INPUT strip', not strip)

    # the same geometry with NO result referencing it = input-file copper
    orig = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
            make_seg(1.0, 0.0, 2.0, 0.0, width=W)]
    pcb = _pcb(orig)
    results = []
    n, nets, strip, added, stats = merge_collinear_segments(results, pcb, None)
    wl = [s for r in results for s in (r.get('new_segments') or [])]
    check('input copper: both originals are reported for the strip',
          len(strip) == 2)
    check('  and the merged replacement is emitted for the writer',
          len(wl) == 1 and len(pcb.segments) == 1)

    # keep_input_copper freezes it instead
    orig2 = [make_seg(0.0, 0.0, 1.0, 0.0, width=W),
             make_seg(1.0, 0.0, 2.0, 0.0, width=W)]
    pcb = _pcb(orig2)
    n, nets, strip, added, stats = merge_collinear_segments(
        [], pcb, None, keep_input_copper=True)
    check('keep_input_copper freezes input copper (no merge, no strip)',
          len(pcb.segments) == 2 and not strip)


def scope_is_respected(check):
    """Out-of-scope nets are not this step's business."""
    segs = [make_seg(0.0, 0.0, 1.0, 0.0, width=W, net_id=1),
            make_seg(1.0, 0.0, 2.0, 0.0, width=W, net_id=1),
            make_seg(0.0, 5.0, 1.0, 5.0, width=W, net_id=2),
            make_seg(1.0, 5.0, 2.0, 5.0, width=W, net_id=2)]
    pcb = _pcb(segs)
    merge_collinear_segments([], pcb, {1})
    check('an out-of-scope net keeps its split',
          len([s for s in pcb.segments if s.net_id == 2]) == 2
          and len([s for s in pcb.segments if s.net_id == 1]) == 1)


def run():
    fails = []

    def check(name, cond):
        print(('PASS' if cond else 'FAIL') + f': {name}')
        if not cond:
            fails.append(name)

    print('#811 merge_collinear_segments -- the pass must join redundant '
          'collinear splits WITHOUT moving copper\n')
    a_straight_run_becomes_one_segment(check)
    a_kink_is_not_a_split(check)
    doubling_back_is_never_merged(check)
    a_via_or_tee_holds_the_joint(check)
    width_and_layer_changes_are_deliberate(check)
    locked_copper_is_never_touched(check)
    pad_custody_is_carried_not_vetoed(check)
    the_ledger_stays_balanced(check)
    scope_is_respected(check)

    print()
    if fails:
        print(f'{len(fails)} FAILURE(S):')
        for f in fails:
            print(f'  - {f}')
        return 1
    print('All #811 merge-collinear checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(run())
