#!/usr/bin/env python3
"""Regression test for issue #628 (LIVE half) -- a candidate rect that SWALLOWS a
milled interior contour whole was judged LEGAL.

`BoardOutlineGate`'s two exact tests work from a rect's four corners and four
edges. A rect large enough to contain an entire interior ring passes both: every
corner is on-board (a milled contour is not a cutout, so it does not make its
interior off-board) and no rect edge comes within the margin of a ring edge. The
"swallowed ring" probe exists precisely for that case -- but it iterated
`self.cutouts` only, so it never saw `board_edge_contours`, the milled Edge.Cuts
geometry `drop_pad_containing_cutouts` reclassifies (#505). A part could be
placed straight over a milled slot and `violation()` reported 0.0.

The boundary-CROSSING case was already handled (the rect's own edges come within
the margin of the milled ring's edges, and `board_edge_geometry` puts those rings
in `rings`); this test pins that as a control so the fix is not mistaken for it.

NOT part of the fix, deliberately: milled contours are given no CONTAINMENT
semantics. `board_edge_contours` mixes a real inner OUTLINE (interior = board:
crkbd, bus_pirate5) with a milled SLOT (interior = not board) and the parser
cannot tell them apart, so declaring the interior off-board re-opens #291
(bus_pirate5 lost all 870 pads). "A part may not swallow a milled ring" is true
under both readings. The final check below pins that non-rule.

    python3 tests/test_628_milled_contour_legality.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.legality import BoardOutlineGate, grade_pad_legality  # noqa: E402
from placement.quench import QuenchState  # noqa: E402

fails = []


def check(name, cond):
    print(("  ok  " if cond else " FAIL ") + name)
    if not cond:
        fails.append(name)


# The QuenchState floors; board_edge_clearance is the gate margin.
KNOBS = dict(clearance=0.25, board_edge_clearance=0.55)
MARGIN = KNOBS['board_edge_clearance']


def _violation(st, ref, *pose):
    """A pose's total illegality, board + overlap: 0.0 exactly when legal."""
    return sum(st.violation_parts(ref, *pose))


def _rect_edge(x1, y1, x2, y2, tag):
    """Four Edge.Cuts gr_lines forming a closed axis-aligned rectangle."""
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return "\n".join(
        f' (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % 4][0]} {pts[(i + 1) % 4][1]}) '
        f'(layer "Edge.Cuts") (width 0.05) (uuid "{tag}{i}"))' for i in range(4))


def _board(inner_pads: bool) -> str:
    """Board outline 0..60 x 0..40 with an interior Edge.Cuts ring 20..24 x 19..21.

    `inner_pads` puts two pads inside that ring, which is what makes
    drop_pad_containing_cutouts RECLASSIFY it from a cutout into a milled
    contour (board_edge_contours). Without them it stays a genuine cutout --
    the control fixture for the half of the probe that already worked.

    SW17 is a front-side part with an 8x8mm courtyard, seeded at (30, 20) where
    it is clear of everything; the pose under test is (22, 20), whose courtyard
    18..26 x 16..24 contains the whole interior ring. The pads inside the ring
    live on a BACK-side part, so no courtyard overlap can confound the verdict.
    """
    inner = ''
    if inner_pads:
        inner = '''
 (footprint "test:slot" (layer "B.Cu") (at 22 20)
  (property "Reference" "B1" (at 0 -2) (layer "B.SilkS"))
  (fp_rect (start -0.3 -0.3) (end 0.3 0.3) (layer "B.CrtYd") (uuid "cyB1"))
  (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "B.Cu") (net 1 "NA") (uuid "pB1a"))
  (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "B.Cu") (net 1 "NA") (uuid "pB1b"))
 )'''
    return f'''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "NA")
 (net 2 "NB")
{_rect_edge(0, 0, 60, 40, 'o')}
{_rect_edge(20, 19, 24, 21, 'm')}{inner}
 (footprint "test:sw" (layer "F.Cu") (at 30 20)
  (property "Reference" "SW17" (at 0 -2) (layer "F.SilkS"))
  (fp_rect (start -4 -4) (end 4 4) (layer "F.CrtYd") (uuid "cySW"))
  (pad "1" smd rect (at -1 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "NB") (uuid "pSWa"))
  (pad "2" smd rect (at 1 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "NB") (uuid "pSWb"))
 )
)
'''


def _write(inner_pads: bool):
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(_board(inner_pads))
    return path


SWALLOW = (18.0, 16.0, 26.0, 24.0)   # SW17's courtyard at (22, 20)
CROSSING = (22.0, 16.0, 30.0, 24.0)  # straddles the ring's right edge
CLEAR = (40.0, 5.0, 44.0, 9.0)       # nowhere near anything


# --------------------------------------------------------------------------
# 1. Fixture sanity. If the reclassification ever stops happening, the interior
#    ring degrades into a plain board_cutouts entry and the swallow probe would
#    catch it for the WRONG reason -- these two checks make that a failure.
# --------------------------------------------------------------------------
def test_fixture_is_a_milled_contour_not_a_cutout():
    path = _write(inner_pads=True)
    try:
        bi = parse_kicad_pcb(path).board_info
        ec = [c for c in (getattr(bi, 'board_edge_contours', None) or [])
              if len(c) >= 3]
        cut = [c for c in (bi.board_cutouts or []) if len(c) >= 3]
        check("fixture: exactly one milled contour (board_edge_contours == 1)",
              len(ec) == 1)
        check("fixture: NOT a cutout (board_cutouts == 0)", len(cut) == 0)
        gate = BoardOutlineGate(bi, MARGIN)
        check("fixture: gate is active (outline + milled ring = 2 rings)",
              gate.active and len(gate.rings) == 2)
        # getattr, not attribute access: on the unfixed gate this attribute does
        # not exist and the test must report a clean FAIL here rather than
        # aborting before it reaches the substantive checks below.
        check("gate.milled carries the contour",
              len(getattr(gate, 'milled', None) or []) == 1)
        check("fixture: the swallow rect is INSIDE the bbox inset "
              "(so only the ring tests can reject it)",
              gate.bbox_outside_amount(SWALLOW) == 0.0)
    finally:
        os.unlink(path)


# --------------------------------------------------------------------------
# 2. The bug: a rect that swallows the milled contour whole.
# --------------------------------------------------------------------------
def test_swallowed_milled_contour_is_illegal():
    path = _write(inner_pads=True)
    try:
        gate = BoardOutlineGate(parse_kicad_pcb(path).board_info, MARGIN)
        check("swallowed milled contour -> rect_blocked",
              gate.rect_blocked(SWALLOW) is True)
        amt = gate.rect_outside_amount(SWALLOW)
        check(f"swallowed milled contour -> rect_outside_amount > 0 ({amt})",
              amt > 0.0)
        # The two must agree, or `violation()==0` stops implying the hard gate
        # passes and the unfreeze branch can walk a part into a blocked pose.
        check("rect_blocked and rect_outside_amount agree on the swallow",
              gate.rect_blocked(SWALLOW) == (amt > 0.0))
    finally:
        os.unlink(path)


# --------------------------------------------------------------------------
# 3. Controls: the CROSSING case must stay caught (it already was, #505), and a
#    rect nowhere near the geometry must stay legal (no over-rejection).
# --------------------------------------------------------------------------
def test_crossing_and_clear_rects_are_unchanged():
    path = _write(inner_pads=True)
    try:
        gate = BoardOutlineGate(parse_kicad_pcb(path).board_info, MARGIN)
        check("control: a rect CROSSING the milled contour is still blocked",
              gate.rect_blocked(CROSSING) is True)
        check("control: ...and still scores a violation",
              gate.rect_outside_amount(CROSSING) > 0.0)
        check("control: a rect clear of everything is still legal",
              gate.rect_blocked(CLEAR) is False)
        check("control: ...and still scores 0",
              gate.rect_outside_amount(CLEAR) == 0.0)
    finally:
        os.unlink(path)


# --------------------------------------------------------------------------
# 4. The cutout half of the probe is untouched: the same geometry WITHOUT the
#    enclosed pads stays a genuine board cutout and is still swallow-detected.
# --------------------------------------------------------------------------
def test_genuine_cutout_swallow_still_detected():
    path = _write(inner_pads=False)
    try:
        bi = parse_kicad_pcb(path).board_info
        cut = [c for c in (bi.board_cutouts or []) if len(c) >= 3]
        check("control fixture: no enclosed pads -> stays a real cutout",
              len(cut) == 1)
        gate = BoardOutlineGate(bi, MARGIN)
        check("control: a swallowed CUTOUT is blocked (pre-existing behaviour)",
              gate.rect_blocked(SWALLOW) is True)
        check("control: ...and scores a violation",
              gate.rect_outside_amount(SWALLOW) > 0.0)
    finally:
        os.unlink(path)


# --------------------------------------------------------------------------
# 5. End to end through the optimizer's hard gate: SW17 is seeded clear at
#    (30, 20) and must not be allowed to move onto the milled slot.
# --------------------------------------------------------------------------
def test_quench_candidate_over_a_milled_slot_is_refused():
    path = _write(inner_pads=True)
    try:
        pcb = parse_kicad_pcb(path)
        st = QuenchState(pcb, path, **KNOBS)
        check("fixture: SW17's seed pose is legal",
              _violation(st, 'SW17') == 0.0)
        check("fixture: the candidate pose is inside the bbox inset",
              st.usable[0] <= SWALLOW[0] and st.usable[1] <= SWALLOW[1]
              and SWALLOW[2] <= st.usable[2] and SWALLOW[3] <= st.usable[3])
        check("candidate_valid refuses a pose swallowing the milled slot",
              not st.candidate_valid('SW17', 22.0, 20.0, 0.0))
        check("violation() scores that pose as illegal",
              _violation(st, 'SW17', 22.0, 20.0, 0.0) > 0.0)
        # ... and a genuinely clear pose is still accepted.
        check("a clear pose is still accepted",
              st.candidate_valid('SW17', 45.0, 20.0, 0.0))
    finally:
        os.unlink(path)


# --------------------------------------------------------------------------
# 6. The #291 trap: the fix must add NO containment rule. A milled contour's
#    interior stays ON-BOARD -- a small part sitting inside the slot is judged
#    by clearance to the milled edges, never by "you are in a hole".
# --------------------------------------------------------------------------
def test_milled_interior_is_not_declared_off_board():
    path = _write(inner_pads=True)
    try:
        bi = parse_kicad_pcb(path).board_info
        gate = BoardOutlineGate(bi, MARGIN)
        check("milled contour is NOT in gate.cutouts (the on-board test)",
              not gate.cutouts)
        from check_drc import _point_on_board  # noqa: E402
        check("a point INSIDE the milled contour is still on-board (#291)",
              _point_on_board(22.0, 20.0, gate.outer, gate.cutouts))
        # A tiny rect wholly inside the ring swallows nothing and leaves the
        # board nowhere; only the margin to the milled edges may bite it. At
        # 22 +/- 0.2 x 20 +/- 0.2 the nearest milled edge is 0.8mm away.
        tiny = (21.8, 19.8, 22.2, 20.2)
        check("a small part inside the milled slot is judged by CLEARANCE only",
              gate.rect_blocked(tiny) is False
              and gate.rect_outside_amount(tiny) == 0.0)
    finally:
        os.unlink(path)


# --------------------------------------------------------------------------
# 7-10. The OWNER half. A milled contour is reclassified out of board_cutouts
#    precisely BECAUSE it encloses >= 2 pad centres, so such a ring ALWAYS has
#    a part living on it -- the shape below is a connector with a milled relief
#    in its middle and pins around it. Charging that part for swallowing its
#    own relief is what the first cut of this fix did, and it is worse than
#    noise: a genuinely sub-clearance edge pose scores LOWER than the seed, so
#    candidate_valid's unfreeze branch (quench.py, "moves strictly back toward
#    the board") ACCEPTS walking the connector off the board edge.
# --------------------------------------------------------------------------
def _owner_board() -> str:
    """J1 owns the relief: two pads INSIDE it (which is what reclassifies the
    ring) and four pins AROUND it, so BOTH its courtyard (17.5..26.5 x
    16.5..23.5) and its pad extent (17.7..26.3 x 16.7..23.3) swallow the ring
    20..24 x 19..21. SW17 is the control, seeded clear at (40, 20) and owning
    nothing."""
    pads = "\n".join(
        f'  (pad "{n}" smd rect (at {dx} {dy}) (size 0.6 0.6) '
        f'(layers "F.Cu") (net 1 "NA") (uuid "pJ1{n}"))'
        for n, dx, dy in ((1, -0.5, 0), (2, 0.5, 0),      # inside the relief
                          (3, -4, 0), (4, 4, 0),          # pins around it
                          (5, 0, -3), (6, 0, 3)))
    return f'''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "NA")
 (net 2 "NB")
{_rect_edge(0, 0, 60, 40, 'o')}
{_rect_edge(20, 19, 24, 21, 'm')}
 (footprint "test:conn" (layer "F.Cu") (at 22 20)
  (property "Reference" "J1" (at 0 -5) (layer "F.SilkS"))
  (fp_rect (start -4.5 -3.5) (end 4.5 3.5) (layer "F.CrtYd") (uuid "cyJ1"))
{pads}
 )
 (footprint "test:sw" (layer "F.Cu") (at 40 20)
  (property "Reference" "SW17" (at 0 -2) (layer "F.SilkS"))
  (fp_rect (start -4 -4) (end 4 4) (layer "F.CrtYd") (uuid "cySW"))
  (pad "1" smd rect (at -1 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "NB") (uuid "pSWa"))
  (pad "2" smd rect (at 1 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "NB") (uuid "pSWb"))
 )
)
'''


def _write_owner():
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(_owner_board())
    return path


OWNER_SEED = (17.5, 16.5, 26.5, 23.5)   # J1's courtyard at its own (22, 20)


def test_owner_fixture_and_gate_scoping():
    """The gate charges the owner's rect WITHOUT skip_rings and clears it WITH
    -- so the exemption is what does it, not a fixture accident."""
    path = _write_owner()
    try:
        pcb = parse_kicad_pcb(path)
        gate = BoardOutlineGate(pcb.board_info, MARGIN)
        check("owner fixture: the ring is a milled contour", len(gate.milled) == 1)
        own = gate.rings_enclosing([(21.5, 20.0), (22.5, 20.0)])
        check("rings_enclosing names the ring the pads sit in", len(own) == 1)
        check("a part that owns NOTHING gets an empty set",
              gate.rings_enclosing([(40.0, 20.0)]) == frozenset())
        check("unscoped: the owner's rect is still charged (guard intact)",
              gate.rect_blocked(OWNER_SEED) is True
              and gate.rect_outside_amount(OWNER_SEED) > 0.0)
        check("scoped: its own relief is exempt",
              gate.rect_blocked(OWNER_SEED, skip_rings=own) is False
              and gate.rect_outside_amount(OWNER_SEED, skip_rings=own) == 0.0)
        check("the exemption is per-RING: a foreign rect is still charged",
              gate.rect_blocked(SWALLOW, skip_rings=frozenset()) is True)
    finally:
        os.unlink(path)


def test_owner_is_legal_at_its_own_seed_pose():
    path = _write_owner()
    try:
        pcb = parse_kicad_pcb(path)
        st = QuenchState(pcb, path, **KNOBS)
        check("the ring owner is legal at its own hand-placed pose",
              _violation(st, 'J1') == 0.0)
        check("...and the control part is too", _violation(st, 'SW17') == 0.0)
    finally:
        os.unlink(path)


def test_owner_is_not_walked_off_the_board_edge():
    """THE regression. With the owner falsely scored 0.55 at its seed, a pose
    0.45mm from a board edge that requires 0.55 scores LOWER (0.40), and the
    unfreeze branch accepts the move."""
    path = _write_owner()
    try:
        pcb = parse_kicad_pcb(path)
        st = QuenchState(pcb, path, **KNOBS)
        walk_x = 0.45 + 4.5          # courtyard half-width 4.5 -> x0 = 0.45
        check("a sub-clearance edge pose IS a violation",
              _violation(st, 'J1', walk_x, 20.0, 0.0) > 0.0)
        check("candidate_valid refuses to walk the owner to the board edge",
              not st.candidate_valid('J1', walk_x, 20.0, 0.0))
    finally:
        os.unlink(path)


def test_owner_scores_clean_on_both_scorecards():
    """#456: a correct hand placement must not report oob. Two INDEPENDENT
    paths reach it -- quench's legality_metrics (courtyard) and
    grade_pad_legality (pad extent) -- and both must be scoped."""
    path = _write_owner()
    try:
        pcb = parse_kicad_pcb(path)
        st = QuenchState(pcb, path, **KNOBS)
        lm = st.legality_metrics()
        check("legality_metrics: no phantom oob_count",
              lm['oob_count'] == 0 and lm['oob_amount'] == 0.0)
        g = grade_pad_legality(pcb, KNOBS['clearance'], edge_margin=MARGIN)
        check("grade_pad_legality: no phantom oob_pad_count",
              g['oob_pad_count'] == 0 and not g['oob_pad_refs'])
    finally:
        os.unlink(path)


def run():
    for t in (test_fixture_is_a_milled_contour_not_a_cutout,
              test_swallowed_milled_contour_is_illegal,
              test_crossing_and_clear_rects_are_unchanged,
              test_genuine_cutout_swallow_still_detected,
              test_quench_candidate_over_a_milled_slot_is_refused,
              test_milled_interior_is_not_declared_off_board,
              test_owner_fixture_and_gate_scoping,
              test_owner_is_legal_at_its_own_seed_pose,
              test_owner_is_not_walked_off_the_board_edge,
              test_owner_scores_clean_on_both_scorecards):
        print(f"--- {t.__name__}")
        t()
    print()
    if fails:
        print("FAILED: %d check(s): %s" % (len(fails), fails))
        return 1
    print("All checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
