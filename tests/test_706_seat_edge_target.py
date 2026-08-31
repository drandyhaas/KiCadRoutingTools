"""`_seat_edge` aims at the declaration, and rotates only under three gates (#706).

The defect: the along-edge search started at the part's current projection and
returned the FIRST legal seat, so `df = 0.0` meant "keep the coordinate you came
in with". On a repaired board that coordinate is the damaged pose's own, carried
verbatim into the shipped output.

The two assertions that carry this file:

* `test_the_seat_is_deterministic_once_declared` -- driven from 11 different
  incoming poses, an UNDECLARED J17 lands at four different x values and a
  declared one lands at one. The scatter is the defect; a single seat is the
  fix. A one-pose test would pass in both directions.

* `test_the_rotation_gate_is_closed_on_the_corpus` -- the three tracked parts a
  bare "no seat, so try 90/180/270" gate would have turned sideways. This is
  the assertion that stops the rotation loop from becoming a footgun, and its
  mutation (delete `_already_on_its_edge`) is the one worth re-running.

Inertness is proved by DIFF, not by inspection: `wk/` is gitignored so the
recorded baseline lives here, taken by running `upstream/main`'s own
`_seat_edge` over the same lattice.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _d in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), _d))

import routing_defaults as rd                        # noqa: E402
from kicad_parser import parse_kicad_pcb             # noqa: E402
from placement import floorplan as fp                # noqa: E402
from placement import seeder                         # noqa: E402
from placement.quench import QuenchState             # noqa: E402

RUN_ALL_TIMEOUT = 900

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(REPO, 'kicad_files')
SPLIT = os.path.join(KF, 'splitflap_driver.kicad_pcb')


#: own `_seat_edge` before any of this landed. Every entry is (ok, x, y, rot).
#: This is the inertness proof: the current code must reproduce it exactly.


def _state(board=SPLIT):
    pcb = parse_kicad_pcb(board)
    return QuenchState(pcb, board, clearance=rd.CLEARANCE,
                       board_edge_clearance=0.55, crossing_penalty=10.0,
                       halo_base=0.5, halo_coef=0.25, halo_weight=2.0,
                       edge_halo=2.0, edge_weight=2.0,
                       grid_step=rd.GRID_STEP, length_weight=1.0)


def _sweep(entry, board=SPLIT, n=11):
    """Seat `entry['ref']` from `n` evenly spaced incoming along-edge poses.

    A fresh `QuenchState` per pose, so one seat cannot contaminate the next.
    """
    out = []
    ref = entry['ref']
    for i in range(n):
        st = _state(board)
        p = st.parts[ref]
        x0, y0, x1, y1 = st.board
        st.apply_move(ref, round(x0 + (x1 - x0) * i / (n - 1.0), 3),
                      round(p.y, 3), p.rot)
        notes = []
        ok = seeder._seat_edge(st, ref, dict(entry), set(), notes)
        q = st.parts[ref]
        out.append({'frac': round(i / (n - 1.0), 2), 'ok': ok,
                    'x': round(q.x, 3), 'y': round(q.y, 3), 'rot': q.rot,
                    'notes': list(notes)})
    return out


def test_declared_frac_is_none_unless_something_is_declared():
    """The switch every inertness claim in this change rests on."""
    assert seeder._declared_frac({'ref': 'J1', 'edge': 'north'}) is None
    assert seeder._declared_frac({'center_on_edge': {'tolerance_mm': 1.0}}) == 0.5
    assert seeder._declared_frac(
        {'along_edge_band': {'from': 0.10, 'to': 0.30}}) == 0.2
    assert seeder._declared_frac_window({'ref': 'J1'}, 100.0) is None
    assert seeder._declared_frac_window(
        {'along_edge_band': {'from': 0.1, 'to': 0.3}}, 100.0) == (0.1, 0.3)
    # A tolerance in MM becomes a window in FRACTIONS, so it needs the span.
    lo, hi = seeder._declared_frac_window(
        {'center_on_edge': {'tolerance_mm': 5.0}}, 100.0)
    assert abs(lo - 0.45) < 1e-9 and abs(hi - 0.55) < 1e-9, (lo, hi)
    print("  PASS: None with nothing declared; 0.5 / band midpoint otherwise")


def test_the_seat_is_deterministic_once_declared():
    """The headline. Same board, same part, 11 incoming poses.

    UNDECLARED the seat follows the pose it arrived with; DECLARED it does
    not. A single-pose test cannot see this at all.
    """
    base = {'ref': 'J17', 'edge': 'north',
            'overhang_mm': {'min': 0.0, 'max': 1.0}}
    plain = _sweep(base)
    xs_plain = sorted({r['x'] for r in plain if r['ok']})
    assert len(xs_plain) >= 3, xs_plain      # anti-vacuity: it really scatters

    centred = _sweep(dict(base, center_on_edge={'tolerance_mm': 1.0}))
    xs_c = sorted({r['x'] for r in centred if r['ok']})
    assert all(r['ok'] for r in centred), centred
    assert len(xs_c) == 1, xs_c

    # 0.85-0.95, because splitflap_driver's north edge carries NINE
    # connectors and most bands are already occupied -- a fixture has to use
    # a band the board can actually offer, or it measures the neighbours.
    banded = _sweep(dict(base, along_edge_band={'from': 0.85, 'to': 0.95}))
    xs_b = sorted({r['x'] for r in banded if r['ok']})
    assert all(r['ok'] for r in banded), banded
    assert len(xs_b) == 1, xs_b
    assert xs_b != xs_c, (xs_b, xs_c)        # the two declarations differ

    # And the declared seat is where it was DECLARED, measured in the same
    # currency the grade uses -- the part's courtyard centre, not its origin.
    # Asserting the origin fraction here would have accepted the 2.54mm
    # currency error the agreement sweep caught.
    st = _state()
    x0, _, x1, _ = st.board
    off = seeder._centre_offset_frac(st.parts['J17'], st.board, 'north')
    centre_frac = (xs_c[0] - x0) / (x1 - x0) + off
    assert abs(centre_frac - 0.5) < 0.005, (centre_frac, off)
    print(f"  PASS: undeclared lands at {len(xs_plain)} distinct x "
          f"{xs_plain[:4]}; centred at {xs_c[0]} (courtyard centre at frac "
          f"{centre_frac:.4f}); banded at {xs_b[0]}")


def test_undeclared_seating_is_bit_identical_to_upstream():
    """INERTNESS, by diff.

    33 seat calls -- three connectors x eleven incoming fractions -- against
    the values `upstream/main`'s own `_seat_edge` produced. Recorded rather
    than re-derived: re-deriving from the current code would assert that the
    code equals itself.
    """
    specs = [
        {'ref': 'J17', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 1.0}},
        {'ref': 'J16', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 1.5}},
        {'ref': 'J5', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 0.8}},
    ]
    got = [_sweep(s) for s in specs]
    want = json.loads(_BASELINE)
    assert len(got) == len(want) == 3
    for spec, g, w in zip(specs, got, want):
        assert g == w, (spec['ref'],
                        [(a, b) for a, b in zip(g, w) if a != b][:2])
    total = sum(len(g) for g in got)
    assert total == 33, total
    print(f"  PASS: {total} undeclared seat calls identical to upstream/main")


def test_a_declared_window_that_cannot_be_met_refuses_by_name():
    """Never a silent clamp outside the declared band.

    A seat the search accepts that the grade then flags is the round-trip
    break `edge_seat_ok` and `keepout_hit` both exist to prevent.
    """
    st = _state()
    notes = []
    # J5's own half-extent puts its legal window at [0.0654, 0.9851] on this
    # 198.12mm edge -- measured, not assumed, because a band chosen from a
    # guess can overlap the window and then this test asserts nothing.
    lo, hi = seeder._edge_frac_bounds(st.parts['J5'], st.board, 'north')
    off = seeder._centre_offset_frac(st.parts['J5'], st.board, 'north')
    assert abs(hi - 0.9851) < 0.001, (lo, hi)
    # The NOTE reports both windows in the declaration's own currency (rect
    # centre), not the ladder's (origin), or its two numbers would mean
    # different things in one sentence.
    assert abs((hi + off) - 0.9597) < 0.002, (hi, off)
    ok = seeder._seat_edge(
        st, 'J5', {'ref': 'J5', 'edge': 'north',
                   'overhang_mm': {'min': 0.0, 'max': 0.8},
                   'along_edge_band': {'from': 0.99, 'to': 1.0}},
        set(), notes)
    assert ok is False
    assert notes and 'does not intersect the legal one' in notes[0], notes
    assert '[0.990, 1.000]' in notes[0], notes[0]
    assert '[0.040, 0.960]' in notes[0], notes[0]
    print(f"  PASS: {notes[0][:96]}...")


def test_the_seat_and_the_grade_agree_over_a_lattice():
    """The agreement test (#797's model, arm N of test_702).

    If the seat search and `rule_edge_connector` ever disagree, the seeder is
    placing to a rule the grader does not have -- the failure both #701 and
    #702 exist to prevent, and this change adds a new rule AND a new actor for
    it, written in two places.

    This sweep EARNED its place immediately: it caught the seeder aiming the
    part's origin while the grade measured its courtyard centre, a 2.54mm
    disagreement on splitflap_driver J17 that no amount of reading the two
    functions had shown.

    Several boards, because one crowded edge yields too few seats to be
    evidence -- splitflap_driver's north edge carries nine connectors.
    """
    cases = [
        ('splitflap_driver', 'J17', 'north', {'along_edge_band': {'from': 0.05, 'to': 0.15}}),
        ('splitflap_driver', 'J17', 'north', {'along_edge_band': {'from': 0.85, 'to': 0.95}}),
        ('splitflap_driver', 'J17', 'north', {'center_on_edge': {'tolerance_mm': 1.0}}),
        ('ulx3s', 'J1', 'west', {'center_on_edge': {'tolerance_mm': 1.0}}),
        ('ulx3s', 'J2', 'east', {'center_on_edge': {'tolerance_mm': 1.0}}),
        ('tigard', 'J1', 'west', {'center_on_edge': {'tolerance_mm': 2.0}}),
        ('tigard', 'J7', 'south', {'along_edge_band': {'from': 0.10, 'to': 0.40}}),
    ]
    checked, skipped = 0, []
    for board, ref, edge, claim in cases:
        path = os.path.join(KF, board + '.kicad_pcb')
        entry = dict({'ref': ref, 'edge': edge,
                      'overhang_mm': {'min': 0.0, 'max': 1.0}}, **claim)
        st = _state(path)
        notes = []
        if not seeder._seat_edge(st, ref, dict(entry), set(), notes):
            skipped.append((board, ref, notes[:1]))
            continue
        # Grade the pose the seeder just produced, through the real rule.
        pcb2 = parse_kicad_pcb(path)
        pcb2.footprints[ref].x = st.parts[ref].x
        pcb2.footprints[ref].y = st.parts[ref].y
        pcb2.footprints[ref].rotation = st.parts[ref].rot
        doc = fp.emit_intent(pcb2, path, declare_classes=False)
        doc['edge_connectors'] = [dict(entry)]
        r = fp.grade(fp.intent_from_dict(doc, path), pcb2, path)
        bad = [v for v in r.violations
               if v.ref == ref and ('along_edge_fraction' in (v.measured or {})
                                    or 'along_edge_offset_mm' in (v.measured or {}))]
        assert not bad, (board, ref, claim, [v.message for v in bad])
        checked += 1
    # Anti-vacuity: an agreement test where nothing seated agrees trivially.
    assert checked >= 4, (checked, skipped)
    print(f"  PASS: {checked}/{len(cases)} declared seats taken across 3 "
          f"boards, and the grade agrees with every one "
          f"({len(skipped)} could not seat and are not evidence either way)")


def test_the_rotation_gate_is_closed_on_the_corpus():
    """The three parts a bare gate would have turned sideways.

    `_edge_frac_bounds` refuses at their current rotation while they already
    overhang their boundary by 11.99 / 11.99 / 26.55mm -- full board-width
    connectors, correctly oriented, refused for a different reason.
    """
    cases = [('ulx3s', 'J1', 11.99), ('ulx3s', 'J2', 11.99),
             ('sonde_u', 'J1', 26.55)]
    for board, ref, want_overhang in cases:
        path = os.path.join(KF, board + '.kicad_pcb')
        st = _state(path)
        part = st.parts[ref]
        rot0 = part.rot
        over = st.edge_gate.rect_outside_amount(part.rect())
        assert abs(over - want_overhang) < 0.05, (board, ref, over)
        # The guard MEASURES -- not a swallowed exception answering False.
        assert seeder._already_on_its_edge(st, part) is True, (board, ref)
        notes = []
        seeder._seat_edge(st, ref,
                          {'ref': ref, 'edge': 'east',
                           'overhang_mm': {'min': 0.0, 'max': 1.0},
                           'center_on_edge': {'tolerance_mm': 1.0}},
                          set(), notes)
        assert st.parts[ref].rot == rot0, (board, ref, rot0, st.parts[ref].rot)
    print(f"  PASS: {len(cases)} corpus parts overhang 11.99/11.99/26.55mm, "
          f"guard True on all, none rotated")


def test_the_rotation_loop_fires_only_where_declared_and_only_on_failure():
    """It opens where it should, and both extra gates hold."""
    base = {'ref': 'J17', 'edge': 'north',
            'overhang_mm': {'min': 0.0, 'max': 1.0}}

    # (a) UNDECLARED: never rotates, whatever the ladder does. Measured, this
    # is the case that caught the missing gate: at incoming fraction 0.3 an
    # undeclared J17 used to come back rotated 0 -> 90.
    for r in _sweep(base):
        assert r['rot'] == 0.0, r
        assert not any('rotation' in n for n in r['notes']), r

    # (b) DECLARED, INTERIOR, and the current rotation cannot seat: it
    # rotates, and says so with BOTH angles.
    #
    # It has to be interior. Every declared edge connector on this board
    # already overhangs its edge by 0.40-0.64mm, so `_already_on_its_edge`
    # blocks all of them -- which is the guard working, and it means the
    # rotation loop is reachable only for a part that is NOT yet on its edge.
    # That is exactly #706's case: a receptacle parked in the middle of the
    # board, where its current orientation carries no information.
    st = _state()
    part = st.parts['J17']
    st.apply_move('J17', part.x, round((st.board[1] + st.board[3]) / 2.0, 3),
                  part.rot)
    assert not seeder._already_on_its_edge(st, st.parts['J17'])
    notes = []
    ok = seeder._seat_edge(
        st, 'J17', dict(base, along_edge_band={'from': 0.10, 'to': 0.20}),
        set(), notes)
    rotated = ok and st.parts['J17'].rot != 0.0
    assert rotated, (ok, st.parts['J17'].rot, notes)
    n = [x for x in notes if 'CHECK THIS' in x]
    assert n, notes
    assert 'own rotation 0deg' in n[0] and 'seated at 90deg' in n[0], n[0]

    # (c) DECLARED and the current rotation CAN seat: the current rotation
    # wins, so the loop is not merely "try everything".
    for r in _sweep(dict(base, center_on_edge={'tolerance_mm': 1.0})):
        assert r['ok'] and r['rot'] == 0.0, r
    print(f"  PASS: undeclared never rotates; a DECLARED interior part that "
          f"cannot seat at rot 0 is seated at 90 and says so; "
          f"declared+seatable keeps its rotation")


def test_stage_one_honours_the_same_declaration():
    """The from-scratch path never calls `_seat_edge`.

    Its even distribution gives splitflap_driver's north connectors
    (k+1)/(n+1), none of which is 0.5 -- so without this a declared
    `center_on_edge` is seated wrong from scratch and `place_seed` exits 4
    grading its own output against the intent it was built from.
    """
    import inspect
    src = inspect.getsource(seeder.seed_from_intent)
    assert '_declared_frac(c)' in src, \
        "stage 1 does not consult the declaration"
    assert 'frac = _dec if _dec is not None else (k + 1) / (len(specs) + 1)' \
        in src, "stage 1's even distribution is not overridden by a declaration"
    assert '_declared_frac_window(' in src, \
        "stage 1 does not clamp to the declared window"
    print("  PASS: stage 1 reads the declaration, overrides its even "
          "distribution and clamps to the declared window")


#: Recorded from `upstream/main`'s `_seat_edge`, three connectors x eleven
#: incoming fractions. See `test_undeclared_seating_is_bit_identical_to_upstream`.
_BASELINE = r'''[[{"frac": 0.0, "notes": [], "ok": true, "rot": 0.0, "x": 37.846, "y": 31.65}, {"frac": 0.1, "notes": [], "ok": true, "rot": 0.0, "x": 35.306, "y": 31.65}, {"frac": 0.2, "notes": [], "ok": true, "rot": 0.0, "x": 35.306, "y": 31.65}, {"frac": 0.3, "notes": [], "ok": false, "rot": 0.0, "x": 84.836, "y": 31.75}, {"frac": 0.4, "notes": [], "ok": false, "rot": 0.0, "x": 104.648, "y": 31.75}, {"frac": 0.5, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.65}, {"frac": 0.6, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.65}, {"frac": 0.7, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.65}, {"frac": 0.8, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.65}, {"frac": 0.9, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.65}, {"frac": 1.0, "notes": [], "ok": true, "rot": 0.0, "x": 205.994, "y": 31.65}], [{"frac": 0.0, "notes": [], "ok": true, "rot": 0.0, "x": 37.846, "y": 31.4}, {"frac": 0.1, "notes": [], "ok": true, "rot": 0.0, "x": 35.306, "y": 31.4}, {"frac": 0.2, "notes": [], "ok": true, "rot": 0.0, "x": 35.306, "y": 31.4}, {"frac": 0.3, "notes": [], "ok": false, "rot": 0.0, "x": 84.836, "y": 31.75}, {"frac": 0.4, "notes": [], "ok": false, "rot": 0.0, "x": 104.648, "y": 31.75}, {"frac": 0.5, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.4}, {"frac": 0.6, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.4}, {"frac": 0.7, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.4}, {"frac": 0.8, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.4}, {"frac": 0.9, "notes": [], "ok": true, "rot": 0.0, "x": 203.708, "y": 31.4}, {"frac": 1.0, "notes": [], "ok": true, "rot": 0.0, "x": 205.994, "y": 31.4}], [{"frac": 0.0, "notes": [], "ok": false, "rot": 180.0, "x": 25.4, "y": 29.21}, {"frac": 0.1, "notes": [], "ok": true, "rot": 180.0, "x": 45.212, "y": 29.45}, {"frac": 0.2, "notes": [], "ok": true, "rot": 180.0, "x": 45.212, "y": 29.45}, {"frac": 0.3, "notes": [], "ok": true, "rot": 180.0, "x": 45.212, "y": 29.45}, {"frac": 0.4, "notes": [], "ok": true, "rot": 180.0, "x": 45.212, "y": 29.45}, {"frac": 0.5, "notes": [], "ok": true, "rot": 180.0, "x": 183.896, "y": 29.45}, {"frac": 0.6, "notes": [], "ok": true, "rot": 180.0, "x": 183.896, "y": 29.45}, {"frac": 0.7, "notes": [], "ok": true, "rot": 180.0, "x": 183.896, "y": 29.45}, {"frac": 0.8, "notes": [], "ok": true, "rot": 180.0, "x": 183.896, "y": 29.45}, {"frac": 0.9, "notes": [], "ok": true, "rot": 180.0, "x": 213.614, "y": 29.45}, {"frac": 1.0, "notes": [], "ok": true, "rot": 180.0, "x": 180.946, "y": 29.45}]]'''


TESTS = [
    test_declared_frac_is_none_unless_something_is_declared,
    test_the_seat_is_deterministic_once_declared,
    test_undeclared_seating_is_bit_identical_to_upstream,
    test_a_declared_window_that_cannot_be_met_refuses_by_name,
    test_the_seat_and_the_grade_agree_over_a_lattice,
    test_the_rotation_gate_is_closed_on_the_corpus,
    test_the_rotation_loop_fires_only_where_declared_and_only_on_failure,
    test_stage_one_honours_the_same_declaration,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
