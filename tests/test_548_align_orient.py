"""
Alignment and orientation in the placement cost kernel (issue #548).

#548 asks for the two things a human does constantly that the quench does not:
put parts that belong together on a shared axis, and turn a part toward the net
it exists to serve.

A PREMISE CORRECTION FIRST, because the issue states it wrongly and the fix
depends on which way round it is. #548 proposes to "score airwires from the
actual pad the net lands on". The cost path ALREADY does that: `_net_points`
emits one MST node per connected pad from `pad_globals()`, full rotation
applied, and there is no centroid anywhere in the objective. The real gap is
NUMERIC -- a ~1mm pad offset perturbs a ~20mm net's MST length by a fraction of
a mm, less once the tree re-roots -- so the directional signal is present and
drowned. What was missing is a term that is directional at PART scale with its
own weight, not a different geometry.

`test_the_length_term_alone_is_directionally_blind` is that correction made
falsifiable: with only the length term, the four rotations of a part whose one
connected pad faces away from its anchor differ by fractions of a millimetre.

BOTH TERMS DEFAULT TO OFF, and the first test is what earns that. The existing
numbers in test_458_* are not goldens to refresh -- they are exact fixtures that
deliberately zero every geometry knob they know about (`KNOBS`,
`_state(... 0,0,0,0,0 ...)`) so the objective is isolated enough to assert
`total == 30.0`. A new term with a nonzero default is invisible to those
fixtures and silently corrupts the isolation, which is degrading a correctness
test rather than re-baselining a golden.
"""

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from kicad_parser import parse_kicad_pcb
from placement.quench import QuenchState, quench

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(ROOT, 'kicad_files')

_CY = ('    (fp_line (start -0.5 -0.35) (end 0.5 -0.35) (stroke (width 0.05)'
       ' (type default)) (layer "F.CrtYd"))\n'
       '    (fp_line (start 0.5 -0.35) (end 0.5 0.35) (stroke (width 0.05)'
       ' (type default)) (layer "F.CrtYd"))\n'
       '    (fp_line (start 0.5 0.35) (end -0.5 0.35) (stroke (width 0.05)'
       ' (type default)) (layer "F.CrtYd"))\n'
       '    (fp_line (start -0.5 0.35) (end -0.5 -0.35) (stroke (width 0.05)'
       ' (type default)) (layer "F.CrtYd"))\n')
_BIG = ('    (fp_line (start -1 -1) (end 1 -1) (stroke (width 0.05)'
        ' (type default)) (layer "F.CrtYd"))\n'
        '    (fp_line (start 1 -1) (end 1 1) (stroke (width 0.05)'
        ' (type default)) (layer "F.CrtYd"))\n'
        '    (fp_line (start 1 1) (end -1 1) (stroke (width 0.05)'
        ' (type default)) (layer "F.CrtYd"))\n'
        '    (fp_line (start -1 1) (end -1 -1) (stroke (width 0.05)'
        ' (type default)) (layer "F.CrtYd"))\n')


def _edges(x0, y0, x1, y1):
    box = ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1),
           (x0, y1, x0, y0))
    return '\n'.join(
        f'  (gr_line (start {a} {b}) (end {c} {d}) (stroke (width 0.1)'
        f' (type default)) (layer "Edge.Cuts"))' for a, b, c, d in box)


def _write(text):
    d = tempfile.mkdtemp(prefix='t548_')
    p = os.path.join(d, 'b.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return p


ROW_YS = [99.8, 100.0, 100.2, 100.3]


def _row_board():
    """Four identical caps on almost-but-not-quite one y -- #548 item 1's
    complaint, verbatim: each offset is an improvement by the cost function and
    collectively they are a mess, because the function is indifferent between
    "four caps on a line" and "four caps within 0.3mm of that line"."""
    fps = []
    for i, y in enumerate(ROW_YS, start=1):
        fps.append(
            f'  (footprint "C_0402" (layer "F.Cu") (at {100 + i * 3} {y})\n'
            + _CY
            + f'    (property "Reference" "C{i}" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd rect (at -0.25 0) (size 0.3 0.4)'
            ' (layers "F.Cu") (net 1 "VCC"))\n'
            '    (pad "2" smd rect (at 0.25 0) (size 0.3 0.4)'
            ' (layers "F.Cu") (net 2 "GND")))')
    return ('(kicad_pcb (version 20221018) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n' + _edges(90, 90, 130, 115)
            + '\n  (net 0 "") (net 1 "VCC") (net 2 "GND")\n'
            + '\n'.join(fps) + '\n)')


def _orient_board():
    """One cap with exactly ONE connected pad, sitting on its right at rot 0,
    whose anchor is 20mm to the LEFT. Only one pad, so the four rotations are
    unambiguously ordered: 180 faces the anchor, 0 faces away, 90/270 are
    perpendicular. A second netted pad would introduce a tie -- measured: with
    a GND pad anchored straight down, rot 90 and rot 180 both cost 1.25."""
    return ('(kicad_pcb (version 20221018) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n' + _edges(80, 90, 130, 120)
            + '\n  (net 0 "") (net 1 "VCC")\n'
            '  (footprint "C_0402" (layer "F.Cu") (at 110 100)\n' + _CY
            + '    (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd rect (at 0.25 0) (size 0.3 0.4)'
            ' (layers "F.Cu") (net 1 "VCC"))\n'
            '    (pad "2" smd rect (at -0.25 0) (size 0.3 0.4)'
            ' (layers "F.Cu")))\n'
            '  (footprint "U_SRC" (layer "F.Cu") (at 90 100)\n' + _BIG
            + '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd rect (at 0 0) (size 1 1)'
            ' (layers "F.Cu") (net 1 "VCC")))\n)')


_FLAT = dict(clearance=0.15, board_edge_clearance=0.3, crossing_penalty=0.0,
             halo_base=0.0, halo_coef=0.0, halo_weight=0.0, edge_halo=0.0,
             edge_weight=0.0, grid_step=0.05)


def _state(path, **kw):
    args = dict(_FLAT, length_weight=0.0)
    args.update(kw)
    return QuenchState(parse_kicad_pcb(path), path, **args)


def _poses(results):
    return sorted((r['reference'], round(r['new_x'], 6), round(r['new_y'], 6),
                   round(r['new_rotation'], 6)) for r in results)


# ---------------------------------------------------------------------------

def test_defaults_are_bit_identical_on_real_boards():
    """The claim the off-by-default decision rests on. Not "close": at weight
    0.0 both hooks return before touching any geometry, `_peers` is `{}`, and
    `pen + 0.0 == pen` exactly for non-negative floats."""
    checked = 0
    for name in ('interf_u_unrouted', 'splitflap_driver'):
        p = os.path.join(KF, name + '.kicad_pcb')
        if not os.path.exists(p):
            continue
        base, off = {}, {}
        a = quench(parse_kicad_pcb(p), p, max_displacement=3.0, step=1.0,
                   max_passes=3, metrics_out=base)
        b = quench(parse_kicad_pcb(p), p, max_displacement=3.0, step=1.0,
                   max_passes=3, align_weight=0.0, orient_weight=0.0,
                   metrics_out=off)
        assert _poses(a) == _poses(b), name
        for phase in ('before', 'after'):
            assert base[phase] == off[phase], (name, phase)
            assert base[phase]['align'] == 0.0
            assert base[phase]['orient'] == 0.0
        checked += 1
    assert checked >= 2
    print(f"  PASS: {checked} real boards, identical poses and metrics; "
          f"align and orient exactly 0.0")


def test_twelve_positional_args_still_bind():
    """Mechanises the append-after-net_weights rule, so a future insertion
    fails HERE rather than silently rebinding three other test files."""
    p = os.path.join(KF, 'interf_u_unrouted.kicad_pcb')
    st = QuenchState(parse_kicad_pcb(p), p, 0.25, 0.55, 30.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0)
    assert st.crossing_penalty == 30.0 and st.length_weight == 0.0
    assert st.align_weight == 0.0 and st.orient_weight == 0.0
    assert st._peers == {}
    print("  PASS: 12 positional args bind unchanged; both weights default 0.0")


def test_metrics_stay_inside_before_and_after():
    """test_504_ratsnest_metrics.py asserts the EXACT top-level key set.
    New NUMBERS go inside `before`/`after`, never alongside. `board_grid`
    (#708) is provenance rather than a number -- which lattice this run
    resolved and why -- so it sits at the top level like `intent_gate`."""
    p = os.path.join(KF, 'interf_u_unrouted.kicad_pcb')
    m = {}
    quench(parse_kicad_pcb(p), p, max_displacement=1.0, step=1.0, max_passes=1,
           align_weight=5.0, orient_weight=1.0, metrics_out=m)
    assert set(m) == {'before', 'after', 'legality', 'board_grid'}, sorted(m)
    for phase in ('before', 'after'):
        for k in ('total', 'length', 'crossings', 'halo', 'edge', 'hpwl',
                  'align', 'orient'):
            assert k in m[phase], (phase, k)
    print(f"  PASS: top-level still {sorted(m)}; align/orient inside both phases")


def test_four_caps_collapse_onto_one_axis():
    """#548 item 1's acceptance test."""
    p = _write(_row_board())
    kw = dict(_FLAT, max_displacement=1.0, step=0.05, length_weight=1.0,
              crossing_penalty=10.0, max_passes=6)
    kw.pop('grid_step', None)

    def ys(**extra):
        res = quench(parse_kicad_pcb(p), p, grid_step=0.05, **kw, **extra)
        pos = {r['reference']: round(r['new_y'], 4) for r in res}
        return [pos.get(f'C{i}', ROW_YS[i - 1]) for i in range(1, 5)]

    off = ys()
    on = ys(align_weight=50.0, align_radius=0.5)
    assert len(set(off)) >= 3, off
    assert len(set(on)) == 1, on
    print(f"  PASS: y {off} -> {on} ({len(set(off))} distinct -> 1)")


def test_the_length_term_alone_is_directionally_blind():
    """The premise correction, made falsifiable.

    The cost path already scores from exact per-pad globals, so the four
    rotations DO differ -- by a fraction of a millimetre across a 20mm net,
    which is the whole point. Nothing at that magnitude survives contact with
    a real board's other terms.
    """
    p = _write(_orient_board())
    st = _state(p, length_weight=1.0)
    part = st.parts['C1']
    lens = {}
    for rot in (0.0, 90.0, 180.0, 270.0):
        subset = {n: st._build_net_airwires(
            n, override_ref='C1', override_pads=part.pad_globals(
                part.x, part.y, rot)) for n in part.nets}
        lens[rot] = st._weighted_length(
            __import__('numpy').asarray(
                [aw for v in subset.values() for aw in v], dtype=float))
    spread = max(lens.values()) - min(lens.values())
    assert spread < 0.6, lens
    assert min(lens.values()) > 19.0, lens
    print(f"  PASS: rotations differ by {spread:.3f}mm across a "
          f"{min(lens.values()):.1f}mm net -- present and drowned")


def test_orient_turns_a_part_toward_the_net_it_serves():
    """#548 item 2's acceptance test, with the length term switched OFF so the
    only directional force in the objective is the new one."""
    p = _write(_orient_board())
    kw = dict(_FLAT, max_displacement=0.0, step=1.0, length_weight=0.0,
              max_passes=4, lock_refs=['U1'])
    kw.pop('grid_step', None)

    off = quench(parse_kicad_pcb(p), p, grid_step=0.05, **kw)
    assert off == [], f"something rotated with no directional term: {off}"

    on = quench(parse_kicad_pcb(p), p, grid_step=0.05, orient_weight=5.0, **kw)
    rot = {r['reference']: r['new_rotation'] for r in on}
    assert rot.get('C1') == 180.0, rot
    print("  PASS: nothing rotates without the term; with it C1 -> 180deg, "
          "putting its one connected pad on the side its anchor is")


def test_the_orient_cost_is_ordered_by_facing():
    """White box: 0 facing the anchor, maximal facing away, perpendicular in
    between -- and bounded by 2*w*|r|, so it breaks a tie rather than
    outranking a real length win."""
    p = _write(_orient_board())
    st = _state(p, orient_weight=1.0)
    part = st.parts['C1']
    c = {rot: st._orient_cost('C1', part.x, part.y, rot)
         for rot in (0.0, 90.0, 180.0, 270.0)}
    assert c[180.0] < 1e-9, c
    assert c[0.0] > c[90.0] > c[180.0], c
    assert abs(c[90.0] - c[270.0]) < 1e-9, c
    r = math.hypot(*[a - b for a, b in
                     zip(part.pad_globals()[0][:2], (part.x, part.y))])
    assert c[0.0] <= 2.0 * r + 1e-9, (c[0.0], r)
    print(f"  PASS: facing={c[180.0]:.3f} perp={c[90.0]:.3f} "
          f"away={c[0.0]:.3f}, bounded by 2|r|={2 * r:.3f}")


def test_align_penalty_is_continuous_and_saturating():
    """A charge-inside/zero-outside shape has a cliff at the radius that pays a
    part to FLEE the row rather than join it. Asserted as a property rather
    than left as a comment."""
    p = _write(_row_board())
    st = _state(p, align_weight=1.0, align_radius=0.5)
    a, b = st.parts['C1'], st.parts['C2']
    prev, jumps = None, []
    vals = []
    for off in [i * 0.02 for i in range(60)]:
        rb = b.rect(b.x, a.y + off, b.rot)
        v = st._align_pair_penalty(a, a.rect(), b, rb)
        vals.append((off, v))
        if prev is not None and abs(v - prev) > 0.02:
            jumps.append(off)
        prev = v
    assert not jumps, f"discontinuity at offsets {jumps[:3]}"
    assert vals[0][1] < 1e-9, vals[0]
    tail = [v for off, v in vals if off > 0.6]
    assert max(tail) - min(tail) < 1e-9, "not saturating past the radius"
    print(f"  PASS: 0.0 on-axis, no jump > 0.02 across 60 offsets, flat "
          f"at {tail[0]:.4f} beyond the radius")


def test_total_cost_counts_each_align_pair_once():
    """total_cost sums unordered pairs (matching halo) while
    part_geometry_cost sums one part's pairs from its own side. The factor of 2
    is deliberate and cancels between candidates; the REPORT must show each
    physical pair once."""
    p = _write(_row_board())
    st = _state(p, align_weight=3.0, align_radius=0.5)
    refs = sorted(st.parts)
    want = 0.0
    for i, ra in enumerate(refs):
        for rb in refs[i + 1:]:
            if rb in st._peers.get(ra, ()):
                want += st._align_pair_penalty(st.parts[ra], st.parts[ra].rect(),
                                               st.parts[rb], st.parts[rb].rect())
    got = st.total_cost()['align']
    assert abs(got - want) < 1e-9, (got, want)
    assert got > 0, "the fixture produced no alignment cost at all"
    print(f"  PASS: total_cost align={got:.4f} == one term per unordered pair")


def test_peers_are_built_without_neighbor_lists():
    """test_quench_neighbor_lists.py:60 asserts an unbuilt state has
    `_neighbors is None`, and :79-82 that its part_geometry_cost EQUALS the
    built one's. A peer index created only on the built path would break that
    equality as a correctness failure."""
    p = _write(_row_board())
    plain = _state(p, align_weight=3.0)
    built = _state(p, align_weight=3.0)
    built.build_neighbor_lists(1.0 + 0.05)
    assert plain._neighbors is None and built._neighbors is not None
    assert plain._peers and plain._peers == built._peers
    for ref in sorted(plain.parts):
        assert abs(plain.part_geometry_cost(ref)
                   - built.part_geometry_cost(ref)) < 1e-12, ref
    print(f"  PASS: _peers built in __init__ ({len(plain._peers)} refs); "
          f"pruned and unpruned geometry costs identical")


def test_peers_are_same_footprint_and_span_limited():
    p = _write(_row_board())
    near = _state(p, align_weight=1.0, align_span=20.0)
    far = _state(p, align_weight=1.0, align_span=1.0)
    assert near._peers, "no peers at a 20mm span on a 3mm-pitch row"
    for ref, peers in near._peers.items():
        fp = near.parts[ref].footprint_name
        assert all(near.parts[o].footprint_name == fp for o in peers), ref
        assert peers == sorted(peers), "peer order is not deterministic (#457)"
    assert sum(len(v) for v in far._peers.values()) < \
        sum(len(v) for v in near._peers.values())
    print(f"  PASS: same-footprint only, sorted; span 20mm -> "
          f"{sum(len(v) for v in near._peers.values())} pairings, "
          f"1mm -> {sum(len(v) for v in far._peers.values())}")


def test_align_never_licenses_an_illegal_pose():
    """watchy seeds 81 of 82 parts in courtyard violation, so it is the board
    where a new attractive term could plausibly pull parts into each other."""
    p = os.path.join(KF, 'watchy.kicad_pcb')
    if not os.path.exists(p):
        print("  SKIP: watchy absent")
        return
    off, on = {}, {}
    quench(parse_kicad_pcb(p), p, max_displacement=1.0, step=0.5, max_passes=2,
           metrics_out=off)
    quench(parse_kicad_pcb(p), p, max_displacement=1.0, step=0.5, max_passes=2,
           align_weight=5.0, orient_weight=1.0, metrics_out=on)
    assert on['legality']['overlap_area'] <= off['legality']['overlap_area'] + 1e-6
    assert on['legality']['oob_count'] <= off['legality']['oob_count']
    print(f"  PASS: overlap {off['legality']['overlap_area']:.2f} -> "
          f"{on['legality']['overlap_area']:.2f}mm2, oob "
          f"{off['legality']['oob_count']} -> {on['legality']['oob_count']}")


TESTS = [
    test_defaults_are_bit_identical_on_real_boards,
    test_twelve_positional_args_still_bind,
    test_metrics_stay_inside_before_and_after,
    test_four_caps_collapse_onto_one_axis,
    test_the_length_term_alone_is_directionally_blind,
    test_orient_turns_a_part_toward_the_net_it_serves,
    test_the_orient_cost_is_ordered_by_facing,
    test_align_penalty_is_continuous_and_saturating,
    test_total_cost_counts_each_align_pair_once,
    test_peers_are_built_without_neighbor_lists,
    test_peers_are_same_footprint_and_span_limited,
    test_align_never_licenses_an_illegal_pose,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
