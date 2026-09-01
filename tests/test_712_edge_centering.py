"""WHERE ALONG the edge: the `edge_connector` centring conjunct (#712).

`rule_edge_connector` graded the overhang band, the nearest-edge identity and a
setback, and all three are satisfied ANYWHERE along the edge. Measured on the
tracked corpus: esp_prog's USB1 sits 1.75 mm off the centre of its 14.50 mm
east edge -- 12.07 % -- and graded exactly as well as a centred one.

Two things this file is written to hold down, because both are easy to lose:

* THE HUMAN BOARD MUST NOT FAIL. `tigard`'s three declared connectors sit at
  +16.09 %, -25.40 % and -28.71 % off their edge centres. Any default
  threshold fails a good board three times out of three, which is why the
  conjunct grades only what an author DECLARED. `test_a_human_board_is_not
  _failed_by_accident` is the regression guard on that decision, and it is the
  assertion most likely to be "simplified" away by someone adding a default.

* THE SPAN MUST COME FROM THE RING. `interf_u_unrouted_placed`'s bounding-box
  south side spans 115.570 mm where the board's real south edge spans
  81.280 mm, and `BUS1` sits EXACTLY on the real centre. Measured against the
  bbox it reads 5.715 mm off. That is the whole argument for `edge_span`, and
  `test_the_ring_basis_is_load_bearing` is the only place it is nailed down.

Every positive assertion here has a NEGATIVE partner on the same board and the
same edge -- a tolerance that passes and one that fails, a band that contains
and one that does not -- because a single-arm assertion here would pass in both
directions.

The mutation battery that defends this file is `tests/mutate_711.py`. MEASURED,
from the run:

    35 rows: 34 killed, 1 survived, 0 broken, 0 disagreeing with expectation

The survivor is a recorded finding. Stage 1 of `seed_from_intent` carries a
declared position through two paths -- `_dec` sets the preferred start, the
window clamp bounds the result -- and measured on splitflap_driver J17 with
`center_on_edge {0.5}`: both live -> x 121.92, the declared CENTRE; `_dec`
killed -> 121.42, the near EDGE of the window; the window killed -> 121.92
again, because the start alone names the centre; BOTH killed -> 91.44, frac
1/3, the even distribution. So the start is load-bearing and the window is
redundant, and a third row disabling both KILLS.

Getting there took four rounds, and every correction was to a claim of mine
rather than to the code under test:

  * the first run reported 26 killed and SIX disagreeing. Four `#712` schema
    rows survived because the along-edge REFUSALS were tested nowhere --
    `KEY_SETS` pins the two new names, so the keys were covered and the
    validation was not. Pinning a key is not testing what it refuses;
  * `rotation-guard-deleted` survived because the three corpus parts it
    asserts on cannot seat at ANY rotation, so "they did not rotate" held with
    or without the guard;
  * the stage-1 fixture used a SINGLE declared connector, and the even
    distribution for one entry is (0+1)/(1+1) = 0.5 -- exactly what
    `center_on_edge` asks for -- so it could not tell the declaration from the
    default;
  * and THE BOUND WAS THE ANSWER, twice. `<= 1.0 + 1e-6` passed on a mutation
    that moved the part exactly 1.000mm; tightened to 0.5 it passed on one
    that moved it exactly 0.500mm, because the window clamp lands on the near
    edge of the declared window, which IS the tolerance by construction. The
    assertion is on the CENTRE now, which is what the declaration asks for.

The battery also poisoned itself once: `READER_VERSION = 2` -> `= 1` is the
SAME NUMBER OF BYTES, and CPython caches bytecode on (mtime, size), so a
restore inside one timestamp tick left every later import reading the mutant.
It clears `__pycache__` around every row and restores byte-exactly.
"""
import copy
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _d in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), _d))

from kicad_parser import parse_kicad_pcb            # noqa: E402
from placement import floorplan as fp               # noqa: E402

RUN_ALL_TIMEOUT = 900

KF = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                  'kicad_files')

#: Measured on this corpus while #712 was written. Kept as a table because a
#: number in a comment drifts and a number in an assertion does not.
#:   board                     ref   edge   span    offset      pct
MEASURED = {
    ('tigard', 'J1'): ('west', 46.00, 7.3988, 16.09),
    ('tigard', 'J5'): ('east', 46.00, -11.6812, -25.40),
    ('tigard', 'J7'): ('south', 46.00, -13.2038, -28.71),
    ('interf_u_unrouted_placed', 'BUS1'): ('south', 81.28, 0.0, 0.0),
}

_cache = {}


def _emit(board):
    """(pcb, emitted intent doc). Cached: emit_intent quenches, and this file
    grades the same four boards from several angles."""
    if board not in _cache:
        path = os.path.join(KF, board + '.kicad_pcb')
        pcb = parse_kicad_pcb(path)
        _cache[board] = (pcb, path,
                         fp.emit_intent(pcb, path, declare_classes=True))
    return _cache[board]


def _grade(board, mutate=None):
    pcb, path, doc = _emit(board)
    doc = copy.deepcopy(doc)
    if mutate:
        mutate(doc)
    return fp.grade(fp.intent_from_dict(doc, path), pcb, path)


def _declare(ref, **kw):
    def m(doc):
        hit = [c for c in doc['edge_connectors'] if c['ref'] == ref]
        assert hit, f"{ref} is not an emitted edge connector"
        hit[0].update(kw)
    return m


def _centring(r, ref):
    return [v for v in r.violations
            if v.ref == ref and 'along_edge_offset_mm' in (v.measured or {})]


def _seat(r, ref):
    return [e for e in r.edge_seating if e['ref'] == ref]


def test_the_offset_is_measured_with_nothing_declared():
    """The measurement is not opt-in.

    A number nobody has to ask for is what catches a defect nobody suspected,
    so `edge_seating` carries every entry that names an edge whether or not a
    claim was made about it.
    """
    r = _grade('tigard')
    rows = {e['ref']: e for e in r.edge_seating}
    assert set(rows) == {'J1', 'J5', 'J7'}, sorted(rows)
    for (board, ref), (edge, span, off, pct) in MEASURED.items():
        if board != 'tigard':
            continue
        e = rows[ref]
        assert e['edge'] == edge and not e['declared'], e
        assert abs(e['span_mm'] - span) < 0.01, e
        assert abs(e['along_edge_offset_mm'] - off) < 0.001, e
        assert abs(e['along_edge_offset_pct'] - pct) < 0.02, e
    print(f"  PASS: 3 offsets measured with no declaration: "
          + ', '.join(f"{k} {rows[k]['along_edge_offset_pct']:+.1f}%"
                      for k in sorted(rows)))


def test_a_human_board_is_not_failed_by_accident():
    """THE regression guard on the declared-only decision.

    tigard is a human-authored board whose connectors sit 16-29% off centre.
    A conjunct with any default threshold flags all three. This asserts the
    grade is byte-identical to one taken before the conjunct existed.
    """
    r = _grade('tigard')
    assert not _centring(r, 'J1') and not _centring(r, 'J5') \
        and not _centring(r, 'J7')
    # `not r.errors`, not `r.passed`: since #713 item 5 `passed` also requires
    # that nothing DECLARED went ungraded, and tigard's emitted intent
    # withholds `overlap_area`. The claim here is about the centring conjunct
    # not manufacturing a violation, which is the error list.
    assert not r.errors, [v.message for v in r.errors]
    worst = max(abs(e['along_edge_offset_pct']) for e in r.edge_seating)
    assert worst > 25.0, worst   # anti-vacuity: it really is far off centre
    print(f"  PASS: worst offset {worst:.1f}% and the board still passes -- "
          f"no default threshold exists")


def test_a_declared_tolerance_passes_and_fails_on_the_same_part():
    """Both arms, same board, same ref. ulx3s J1 sits 0.020mm off centre."""
    e = _seat(_grade('ulx3s'), 'J1')[0]
    assert abs(e['along_edge_offset_mm']) < 0.05, e

    ok = _grade('ulx3s', _declare('J1', center_on_edge={'tolerance_mm': 0.5}))
    assert not _centring(ok, 'J1'), [v.message for v in ok.violations]

    bad = _grade('ulx3s', _declare('J1', center_on_edge={'tolerance_mm': 0.01}))
    v = _centring(bad, 'J1')
    assert len(v) == 1, [x.message for x in bad.violations]
    assert 'mm' in v[0].message and '%' in v[0].message, v[0].message
    assert abs(v[0].measured['along_edge_offset_mm']) > 0.01
    # `_charge` in the repair census reads a magnitude off `measured` and
    # falls back to 1.0mm; without this key a centring error sorts below
    # every pad graze.
    assert 'outside_mm' in v[0].measured, v[0].measured
    print(f"  PASS: tol 0.5 clean, tol 0.01 fires -- {v[0].message[:72]}...")


def test_a_declared_band_contains_and_excludes():
    """tigard J5 sits at fraction 0.246 of its east edge."""
    inside = _grade('tigard',
                    _declare('J5', along_edge_band={'from': 0.20, 'to': 0.30}))
    assert not _centring(inside, 'J5')

    outside = _grade('tigard',
                     _declare('J5', along_edge_band={'from': 0.50, 'to': 0.60}))
    v = _centring(outside, 'J5')
    assert len(v) == 1, [x.message for x in outside.violations]
    assert abs(v[0].measured['along_edge_fraction'] - 0.246) < 0.005, v[0].measured
    assert v[0].measured['outside_mm'] > 10.0, v[0].measured
    print(f"  PASS: band 20-30% clean, 50-60% fires at fraction "
          f"{v[0].measured['along_edge_fraction']}")


def test_the_ring_basis_is_load_bearing():
    """The bbox and the ring DISAGREE, and the ring is right.

    `interf_u_unrouted_placed` is a T: bbox south side 115.570mm, real south
    edge 81.280mm. BUS1 sits exactly on the real centre. If this ever reports
    -5.715 the conjunct has fallen back to the bounding box, and a correctly
    centred part is being called 4.9% off.
    """
    r = _grade('interf_u_unrouted_placed')
    e = _seat(r, 'BUS1')[0]
    assert e['basis'] == 'ring', e
    assert abs(e['span_mm'] - 81.28) < 0.01, e
    assert abs(e['along_edge_offset_mm']) < 0.001, e

    # And the claim that a bbox measurement would be wrong, computed here so
    # the number in the docstring is not folklore.
    bounds = r.outline['bounds']
    bbox_span = bounds[2] - bounds[0]
    assert abs(bbox_span - 115.57) < 0.01, bbox_span
    assert abs(bbox_span - e['span_mm'] - 34.29) < 0.02, bbox_span - e['span_mm']

    # What a bbox measurement WOULD have said, computed from the real ring
    # endpoints rather than asserted from memory. BUS1 sits at the ring
    # midpoint, so its bbox offset is the distance between the two midpoints.
    pcb, path, _ = _emit('interf_u_unrouted_placed')
    from placement.quench import QuenchState
    import routing_defaults as rd
    st = QuenchState(pcb, path, clearance=rd.CLEARANCE,
                     board_edge_clearance=0.55, crossing_penalty=10.0,
                     halo_base=0.5, halo_coef=0.25, halo_weight=2.0,
                     edge_halo=2.0, edge_weight=2.0, grid_step=rd.GRID_STEP,
                     length_weight=1.0)
    lo, hi, basis = fp.edge_span(st.edge_gate, bounds, 'south', r.outline)
    assert basis == 'ring' and abs((hi - lo) - 81.28) < 0.01, (lo, hi, basis)
    bbox_off = (lo + hi) / 2.0 - (bounds[0] + bbox_span / 2.0)
    assert abs(bbox_off + 5.715) < 0.01, bbox_off
    # A declared centring claim on the RING basis passes.
    ok = _grade('interf_u_unrouted_placed',
                _declare('BUS1', center_on_edge={'tolerance_mm': 0.5}))
    assert not _centring(ok, 'BUS1'), [v.message for v in ok.violations]
    print(f"  PASS: BUS1 reads {e['along_edge_offset_mm']:+.3f}mm on the ring "
          f"(span {e['span_mm']:.2f}mm); the same part measured against the "
          f"{bbox_span:.2f}mm bbox side would read {bbox_off:+.3f}mm")


def test_a_non_rectangular_board_still_measures_where_it_can():
    """Per EDGE, not per board.

    watchy is not a simple rectangle -- 21 Edge.Cuts segments, 2 cutouts --
    and all four of its edges still resolve to one ring run each. Abstaining
    per board would throw away 13 correct measurements.
    """
    r = _grade('watchy')
    assert not r.outline['simple_rectangle'], r.outline
    assert r.outline['cutouts'] == 2, r.outline
    rows = [e for e in r.edge_seating
            if e.get('along_edge_offset_mm') is not None]
    assert len(rows) >= 10, len(rows)
    assert all(e['basis'] == 'ring' for e in rows), rows[:2]
    assert not r.budget_abstained or 'overlap_area' in r.budget_abstained
    print(f"  PASS: {len(rows)} entries measured on a 21-segment, 2-cutout "
          f"board, all on the ring basis")


def test_the_abstain_names_its_reason_and_is_not_a_pass():
    """No tracked board abstains, so the branch is exercised synthetically.

    That is worth stating rather than hiding: the abstain is a GUARD, not a
    common path. `edge_span` is called directly with an outline that resolves
    to nothing, and the reason has to name what was found.
    """
    class _Gate:
        rings = [[(0, 0), (10, 0), (10, 10), (0, 10)]]

        def edges(self):
            # Two disjoint runs on the north side: a notch that reaches the
            # bbox extreme, which is the shape 'along the edge' cannot mean.
            return [(0.0, 0.0, 3.0, 0.0), (7.0, 0.0, 10.0, 0.0)]

    outline = {'simple_rectangle': False, 'cutouts': 0, 'edge_segments': 6}
    lo, hi, why = fp.edge_span(_Gate(), (0.0, 0.0, 10.0, 10.0), 'north',
                               outline)
    assert lo is None and hi is None, (lo, hi)
    assert 'separate runs' in why, why

    class _NoRun(_Gate):
        def edges(self):
            return [(0.0, 5.0, 10.0, 5.0)]

    lo, hi, why = fp.edge_span(_NoRun(), (0.0, 0.0, 10.0, 10.0), 'north',
                               outline)
    assert lo is None and 'no Edge.Cuts segment lies on the north side' in why

    # No rings AND not a simple rectangle -> abstain, not a silent bbox.
    class _Bare:
        rings = []

        def edges(self):
            return []

    lo, hi, why = fp.edge_span(_Bare(), (0.0, 0.0, 10.0, 10.0), 'north',
                               outline)
    assert lo is None and 'not a simple rectangle' in why, why

    # ... and WITH a simple rectangle the bbox is used, because it IS the
    # outline. This is the branch most of the corpus takes.
    lo, hi, basis = fp.edge_span(_Bare(), (0.0, 0.0, 10.0, 10.0), 'north',
                                 {'simple_rectangle': True, 'cutouts': 0})
    assert (lo, hi, basis) == (0.0, 10.0, 'bbox')
    print("  PASS: split run, absent run and no-ring-no-rectangle all abstain "
          "with a stated reason; a simple rectangle uses the bbox")


def test_a_claim_with_no_edge_abstains_rather_than_inferring_one():
    """The failure mode is inferring the edge to grade the claim against.

    `_nearest_edge` would happily supply one, and then a declared centring
    claim would be graded against an edge nobody named -- the exact inversion
    this whole channel exists to prevent.
    """
    r = _grade('esp_prog', _declare('CON1',
                                    center_on_edge={'tolerance_mm': 0.5}))
    assert not _centring(r, 'CON1'), [v.message for v in r.violations]
    keys = [k for k in r.budget_abstained if 'CON1' in k]
    assert keys, sorted(r.budget_abstained)
    assert 'declares no `edge`' in r.budget_abstained[keys[0]]
    assert 'NOT DERIVABLE' in fp.format_text(r)
    print(f"  PASS: abstained -- {r.budget_abstained[keys[0]][:70]}...")


def test_the_emitter_never_writes_either_field():
    """Structural pin on "no existing intent changes verdict".

    If `emit_intent` ever writes one, every emitted document gains a graded
    claim nobody declared and the round trip stops being clean by
    construction.
    """
    seen = []
    for board in ('tigard', 'ulx3s', 'watchy', 'esp_prog',
                  'interf_u_unrouted_placed'):
        _, _, doc = _emit(board)
        for c in doc['edge_connectors']:
            seen.append(set(c) & {'center_on_edge', 'along_edge_band'})
    assert seen, "no emitted connectors at all -- this test proves nothing"
    assert not any(seen), seen
    print(f"  PASS: {len(seen)} emitted connector(s) across 5 boards, none "
          f"carrying either field")


def test_the_loader_refuses_every_malformed_along_edge_claim():
    """The refusals, which nothing else in this repo exercised.

    Added because the mutation battery found the hole: `KEY_SETS` and the
    every-known-key fixture in test_549_floorplan_schema.py mention both new
    names, so the keys are PINNED -- but four mutations that gutted the
    validation (accept both forms on one entry, default `tolerance_mm`,
    accept an inverted band, leave READER_VERSION at 1) all SURVIVED the whole
    suite. Pinning a key is not testing what it refuses.
    """
    base = {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm'}

    def ec(**kw):
        return dict(base, edge_connectors=[dict(kw, ref='J1', edge='north')])

    def rejects(raw, why):
        try:
            fp.intent_from_dict(raw)
        except fp.IntentError as exc:
            return str(exc)
        raise AssertionError(f"NOT rejected: {why}")

    cases = [
        (ec(center_on_edge={'tolerance_mm': 1.0},
            along_edge_band={'from': 0.1, 'to': 0.3}),
         'declares BOTH center_on_edge and along_edge_band'),
        (ec(center_on_edge={}), 'needs `tolerance_mm`'),
        (ec(center_on_edge={'tolerence_mm': 1.0}), 'unknown key(s) tolerence_mm'),
        (ec(center_on_edge={'tolerance_mm': True}), 'expected a number'),
        (ec(center_on_edge={'tolerance_mm': -1}), 'expected >= 0.0'),
        (ec(center_on_edge=[1.0]), 'center_on_edge expects'),
        (ec(along_edge_band={'from': 0.9, 'to': 0.1}), 'is not less than to'),
        (ec(along_edge_band={'from': 0.1, 'to': 0.1}), 'is not less than to'),
        (ec(along_edge_band={'from': 0.1, 'to': 1.5}), 'expected [0.0, 1.0]'),
        (ec(along_edge_band={'from': 0.1}), 'needs `to`'),
        (ec(along_edge_band={'form': 0.1, 'to': 0.3}), 'unknown key(s) form'),
        (ec(along_edge_band=[0.1, 0.3]), 'along_edge_band expects'),
    ]
    for raw, want in cases:
        msg = rejects(raw, want)
        assert want in msg, (want, msg)

    # Both forms load individually, or the refusals above would be vacuous.
    assert fp.intent_from_dict(ec(center_on_edge={'tolerance_mm': 0.5}))
    assert fp.intent_from_dict(ec(along_edge_band={'from': 0.1, 'to': 0.3}))

    # READER_VERSION is the number an author copies into `min_reader`. At 1 a
    # document could claim a reader-1 build acts on a claim it has never heard
    # of, which is a false statement in the one field whose job is to be true.
    assert fp.READER_VERSION == 2, fp.READER_VERSION
    assert fp.intent_from_dict(dict(base, min_reader=2))
    msg = rejects(dict(base, min_reader=3), 'a claim this build cannot act on')
    assert 'this build is reader 2' in msg, msg
    print(f"  PASS: {len(cases)} malformed along-edge claims refused by "
          f"reason; both forms load alone; READER_VERSION 2")


def test_rules_run_bookkeeping_is_untouched():
    """A conjunct, not a rule. `RULES` / `_wants` / `_SKIP_REASON` unchanged."""
    plain = _grade('ulx3s')
    declared = _grade('ulx3s',
                      _declare('J1', center_on_edge={'tolerance_mm': 0.01}))
    assert sorted(plain.rules_run) == sorted(declared.rules_run)
    assert sorted(plain.rules_skipped) == sorted(declared.rules_skipped)
    assert 'edge_connector' in plain.rules_run
    names = {name for name, _ in fp.RULES}
    assert 'center_on_edge' not in names and 'along_edge' not in names
    s = fp.summary(declared)
    assert s['edge_seating_declared'] >= 1 and s['edge_seating_rows'] >= 1
    print(f"  PASS: rules_run identical ({len(plain.rules_run)}), summary "
          f"carries edge_seating_rows={s['edge_seating_rows']}")


TESTS = [
    test_the_offset_is_measured_with_nothing_declared,
    test_a_human_board_is_not_failed_by_accident,
    test_a_declared_tolerance_passes_and_fails_on_the_same_part,
    test_a_declared_band_contains_and_excludes,
    test_the_ring_basis_is_load_bearing,
    test_a_non_rectangular_board_still_measures_where_it_can,
    test_the_abstain_names_its_reason_and_is_not_a_pass,
    test_a_claim_with_no_edge_abstains_rather_than_inferring_one,
    test_the_emitter_never_writes_either_field,
    test_the_loader_refuses_every_malformed_along_edge_claim,
    test_rules_run_bookkeeping_is_untouched,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
