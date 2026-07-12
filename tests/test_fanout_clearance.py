"""
Tests for the fanout-clearance placement repair (issue #130).

The repair runs AFTER a fanout: it nudges decoupling caps so their pads clear
foreign-net fanout vias, pulls each pad toward same-net balls, and never lets
caps overlap each other. We use kicad_files/orangecrab_ext_pll.kicad_pcb and
inject synthetic vias (instead of depending on a real fanout) so the cases
are deterministic and fast.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb
from placement.fanout_clearance import (_Repair, _rotate_local_bounds,
                                        repair_fanout_clearance)
from placement.parser import extract_courtyard_bboxes

BOARD = os.path.join(os.path.dirname(__file__), '..', 'kicad_files',
                     'orangecrab_ext_pll.kicad_pcb')
CLEAR = 0.1

# C7 is a back-side 0402 near a BGA; pad 1 = GND(net 5) @ (143.26,109.83).
CAP = 'C7'
PAD1 = (143.26, 109.83)
GND = 5
FOREIGN = 2


def _new_repair(pcb):
    return _Repair(pcb, BOARD, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C', set())


def test_cap_rect_uses_absolute_rotation():
    """Guards the bug where the courtyard was rotated by delta-from-seed
    instead of the absolute angle (phantom overlaps on rotated caps)."""
    pcb = parse_kicad_pcb(BOARD)
    cy = extract_courtyard_bboxes(BOARD)
    st = _new_repair(pcb)
    for ref, cap in st.caps.items():
        fp = pcb.footprints[ref]
        lb = cy[ref]
        b = _rotate_local_bounds(*lb, fp.rotation)
        expect = (fp.x + b[0], fp.y + b[1], fp.x + b[2], fp.y + b[3])
        got = cap.rect()
        assert all(abs(g - e) < 1e-6 for g, e in zip(got, expect)), ref


def test_foreign_via_is_cleared():
    """The board is already partially routed; C7 collides with foreign vias."""
    pcb = parse_kicad_pcb(BOARD)
    st = _new_repair(pcb)
    assert st.via_penalty(st.caps[CAP], st.caps[CAP].x, st.caps[CAP].y,
                          st.caps[CAP].rot) > 1e-6, "fixture should start dirty"
    result = repair_fanout_clearance(pcb, BOARD, clearance=CLEAR)
    assert CAP in result['resolved']
    assert CAP not in result['unresolved']


def test_same_net_via_is_not_a_violation():
    """A cap pad may sit on a via of its OWN net (via-in-pad sharing), but a
    foreign-net via at the same spot is a violation."""
    pcb = parse_kicad_pcb(BOARD)
    st = _new_repair(pcb)
    cap = st.caps[CAP]
    args = (cap, cap.x, cap.y, cap.rot)
    st.vias = [(PAD1[0], PAD1[1], FOREIGN, 0.35 / 2 + CLEAR)]
    assert st.via_penalty(*args) > 1e-6
    st.vias = [(PAD1[0], PAD1[1], GND, 0.35 / 2 + CLEAR)]
    assert st.via_penalty(*args) <= 1e-6


def test_no_new_cap_cap_overlap():
    pcb = parse_kicad_pcb(BOARD)
    base = _new_repair(parse_kicad_pcb(BOARD))  # baseline overlaps
    result = repair_fanout_clearance(pcb, BOARD, clearance=CLEAR)
    from placement.writer import write_placed_output
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, 'o.kicad_pcb')
        write_placed_output(BOARD, out, result['placements'])
        after = _Repair(parse_kicad_pcb(out), out, CLEAR, 0.1, 0.55, 1.0,
                        2.0, 0.3, 'C', set())
    refs = list(after.caps)
    for i, ra in enumerate(refs):
        for rb in refs[i + 1:]:
            if after.caps[ra].side != after.caps[rb].side:
                continue
            ov = after._overlap(after.caps[ra].rect(), after.caps[rb].rect())
            b = base.base_cap.get(frozenset((ra, rb)), 0.0)
            assert ov <= b + 1e-6, f"new overlap {ra}-{rb}: {ov:.3f} > {b:.3f}"


def test_no_vias_means_noop():
    pcb = parse_kicad_pcb(BOARD)
    pcb.vias = []  # pretend pre-fanout: nothing to clear or anchor to
    result = repair_fanout_clearance(pcb, BOARD, clearance=CLEAR)
    assert result['placements'] == []


def test_foreign_pad_blocks_move_into_short():
    """#235: a candidate that slides a cap pad onto a foreign-net component
    pad (a PAD-PAD short) is hard-blocked; the same pad on the cap's own net
    is not (touching same-net copper is fine)."""
    pcb = parse_kicad_pcb(BOARD)
    st = _new_repair(pcb)
    cap = st.caps[CAP]
    # A nearby in-reach position the cap could otherwise move to.
    tx, ty, tr = cap.seed_x + 0.3, cap.seed_y, cap.rot
    # Where pad 1 lands at that position.
    pad = next(p for p in cap.pad_rects(tx, ty, tr) if p[4] == GND)
    pcx, pcy = (pad[0] + pad[2]) / 2.0, (pad[1] + pad[3]) / 2.0
    # Foreign-net SMD pad on the cap's side, sitting right on that pad.
    foreign = (pcx - 0.25, pcy - 0.25, pcx + 0.25, pcy + 0.25, FOREIGN, cap.side)
    st.cap_foreign_pads[CAP] = [foreign]
    st.base_pad[CAP] = st.pad_penalty(CAP, cap, cap.x, cap.y, cap.rot)
    assert st.pad_penalty(CAP, cap, tx, ty, tr) > 1e-6
    assert st.hard_blocked(CAP, cap, tx, ty, tr)
    # Same-net foreign pad at the same spot is NOT a violation.
    st.cap_foreign_pads[CAP] = [(foreign[0], foreign[1], foreign[2],
                                 foreign[3], GND, cap.side)]
    assert st.pad_penalty(CAP, cap, tx, ty, tr) <= 1e-6


def test_via_clear_fallback_rescues_stuck_caps():
    """#213: a cap the normal cost descent leaves grazing a foreign via is
    relocated to a via-clearing spot by the fallback. Isolate it by freezing
    the descent (zero budget, single pass) so the fallback is the ONLY thing
    that can resolve a violation."""
    common = dict(clearance=CLEAR, max_displacement=0.0, max_passes=1)
    off = repair_fanout_clearance(parse_kicad_pcb(BOARD), BOARD,
                                  via_clear_fallback=False, **common)
    on = repair_fanout_clearance(parse_kicad_pcb(BOARD), BOARD,
                                 via_clear_fallback=True, **common)
    # With the descent frozen, caps stay stuck without the fallback (the only
    # zero-budget candidate is the grid-snapped seed, which may clear the odd
    # hairline track graze but no via)...
    assert off['unresolved'], "fixture should leave caps stuck without fallback"
    # ...and the fallback resolves strictly more of them.
    assert set(on['unresolved']) < set(off['unresolved'])
    assert set(on['resolved']) > set(off['resolved'])


def test_via_clear_fallback_respects_hard_clearances():
    """The fallback must not create a new foreign-pad short to clear a via: no
    cap it moves ends up grazing a foreign pad worse than at its seed (the
    hard_blocked guard is enforced inside the fallback's cost() gate)."""
    seed = _new_repair(parse_kicad_pcb(BOARD))  # base_pad at seed placement
    pcb = parse_kicad_pcb(BOARD)
    res = repair_fanout_clearance(pcb, BOARD, clearance=CLEAR,
                                  max_displacement=0.0, max_passes=1,
                                  via_clear_fallback=True)
    from placement.writer import write_placed_output
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, 'o.kicad_pcb')
        write_placed_output(BOARD, out, res['placements'])
        after = _Repair(parse_kicad_pcb(out), out, CLEAR, 0.1, 0.55, 1.0,
                        2.0, 0.3, 'C', set())
    for ref, cap in after.caps.items():
        got = after.pad_penalty(ref, cap, cap.x, cap.y, cap.rot)
        assert got <= seed.base_pad.get(ref, 0.0) + 1e-6, \
            f"{ref} grazes a foreign pad worse than seed after fallback"


def test_foreign_pad_other_side_smd_ignored():
    """An SMD foreign pad on the opposite side does not constrain the cap."""
    pcb = parse_kicad_pcb(BOARD)
    st = _new_repair(pcb)
    cap = st.caps[CAP]
    pad = next(p for p in cap.pad_rects(cap.x, cap.y, cap.rot) if p[4] == GND)
    pcx, pcy = (pad[0] + pad[2]) / 2.0, (pad[1] + pad[3]) / 2.0
    other = 'F' if cap.side == 'B' else 'B'
    st.cap_foreign_pads[CAP] = [(pcx - 0.25, pcy - 0.25, pcx + 0.25, pcy + 0.25,
                                 FOREIGN, other)]
    assert st.pad_penalty(CAP, cap, cap.x, cap.y, cap.rot) <= 1e-6
    # A through-hole foreign pad (side=None) at the same spot DOES block.
    st.cap_foreign_pads[CAP] = [(pcx - 0.25, pcy - 0.25, pcx + 0.25, pcy + 0.25,
                                 FOREIGN, None)]
    assert st.pad_penalty(CAP, cap, cap.x, cap.y, cap.rot) > 1e-6


def test_seed_track_graze_is_resolved():
    """#278: a cap whose pad ALREADY grazes a foreign-net escape track at its
    seed placement (the under-pad fanout routes through movable caps' zones on
    purpose) is a violation to fix, not a baseline to preserve - the repair
    must move the cap clear."""
    pcb = parse_kicad_pcb(BOARD)
    st = _new_repair(pcb)
    cap = st.caps[CAP]
    # Foreign-net track on the cap's side, running 50um inside pad 1's
    # clearance zone (like ulx3s C19 vs GN3): a real PAD-SEGMENT graze.
    pad = next(p for p in cap.pad_rects() if p[4] == GND)
    graze_y = pad[1] - (0.1 / 2 + CLEAR) + 0.050   # track edge 50um short
    seg = (pad[0] - 1.0, graze_y, pad[2] + 1.0, graze_y,
           FOREIGN, 0.1 / 2 + CLEAR, cap.side)
    pcb.segments = []          # isolate: this graze is the only copper issue
    pcb.vias = []
    st = _new_repair(pcb)
    st.cap_segs[CAP] = [seg]
    st.base_seg[CAP] = st.seg_penalty(CAP, st.caps[CAP],
                                      st.caps[CAP].x, st.caps[CAP].y,
                                      st.caps[CAP].rot)
    assert st.base_seg[CAP] > 1e-6, "fixture should start with a seed graze"
    assert st.graze_penalty(CAP, st.caps[CAP], st.caps[CAP].x,
                            st.caps[CAP].y, st.caps[CAP].rot) > 1e-6
    # A move away from the track (track sits at y < pad min-y) clears the
    # penalty and costs less.
    c = st.caps[CAP]
    away = st.seg_penalty(CAP, c, c.x, c.y + 0.5, c.rot)
    assert away <= 1e-6, "moving off the track should zero the penalty"
    assert (st.cost(CAP, c, c.x, c.y + 0.5, c.rot)
            < st.cost(CAP, c, c.x, c.y, c.rot)), \
        "objective must prefer the graze-free position"


def test_seg_to_rect_dist_exact():
    """The exact rect-vs-segment distance replaces the centre+half-diagonal
    model: an elongated pad no longer manufactures phantom grazes for a track
    that is actually clear, and a real graze is measured at its true depth."""
    from placement.fanout_clearance import _seg_to_rect_dist
    rect = (0.0, 0.0, 2.0, 0.5)   # elongated pad rect
    # Track along y=0.8: true gap 0.3 (half-diag model would call this ~0.73
    # from centre vs req including 1.03 half-diagonal - a phantom graze).
    assert abs(_seg_to_rect_dist(-1.0, 0.8, 3.0, 0.8, rect) - 0.3) < 1e-9
    # Segment crossing the rect -> 0.
    assert _seg_to_rect_dist(1.0, -1.0, 1.0, 1.0, rect) == 0.0
    # Endpoint inside -> 0.
    assert _seg_to_rect_dist(1.0, 0.25, 5.0, 5.0, rect) == 0.0
    # Diagonal approach to a corner.
    d = _seg_to_rect_dist(3.0, 1.5, 4.0, 1.5, rect)
    assert abs(d - ((1.0 ** 2 + 1.0 ** 2) ** 0.5)) < 1e-9


def test_mover_pad_short_is_hard_blocked():
    """#275 (fpga_sdram C11/FB1): a move that lands a cap pad on ANOTHER
    MOVER's different-net pad must be hard-blocked even when the courtyard
    baseline would tolerate the overlap (pre-existing tight seed placements).
    The courtyard/seg/static-pad baselines are inflated so ONLY the new
    mover-vs-mover pad guard can reject the pose."""
    pcb = parse_kicad_pcb(BOARD)
    st = _new_repair(pcb)
    for ref, cap in st.caps.items():
        for oref in st.cap_caps[ref]:
            other = st.caps[oref]
            for (ox0, oy0, ox1, oy1, onet) in other.pad_rects():
                for (px0, py0, px1, py1, pnet) in cap.pad_rects():
                    if pnet == onet:
                        continue
                    # pose that puts this pad's centre on the other mover's pad
                    x = cap.x + (ox0 + ox1) / 2.0 - (px0 + px1) / 2.0
                    y = cap.y + (oy0 + oy1) / 2.0 - (py0 + py1) / 2.0
                    # neutralize every other hard constraint for this pose
                    st.base_cap[frozenset((ref, oref))] = float('inf')
                    for i, _r in st.cap_static[ref]:
                        st.base_static[(ref, i)] = float('inf')
                    st.base_seg[ref] = float('inf')
                    st.base_pad[ref] = float('inf')
                    assert st.hard_blocked(ref, cap, x, y, cap.rot), \
                        f"{ref} pad allowed onto {oref} pad (different nets)"
                    # sanity: the guard is baseline-relative, not absolute --
                    # tolerating the same encroachment when present at seed
                    st.base_cap_pad[frozenset((ref, oref))] = float('inf')
                    assert not st.hard_blocked(ref, cap, x, y, cap.rot), \
                        f"{ref}/{oref}: guard not baseline-relative"
                    return
    raise AssertionError("fixture has no same-side mover pair with "
                         "different-net pads")


if __name__ == '__main__':
    test_cap_rect_uses_absolute_rotation()
    test_foreign_via_is_cleared()
    test_same_net_via_is_not_a_violation()
    test_no_new_cap_cap_overlap()
    test_no_vias_means_noop()
    test_foreign_pad_blocks_move_into_short()
    test_foreign_pad_other_side_smd_ignored()
    test_via_clear_fallback_rescues_stuck_caps()
    test_via_clear_fallback_respects_hard_clearances()
    test_seed_track_graze_is_resolved()
    test_seg_to_rect_dist_exact()
    test_mover_pad_short_is_hard_blocked()
    test_mover_pad_short_is_hard_blocked()
    print("ALL PASS")
