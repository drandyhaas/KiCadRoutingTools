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


if __name__ == '__main__':
    test_cap_rect_uses_absolute_rotation()
    test_foreign_via_is_cleared()
    test_same_net_via_is_not_a_violation()
    test_no_new_cap_cap_overlap()
    test_no_vias_means_noop()
    print("ALL PASS")
