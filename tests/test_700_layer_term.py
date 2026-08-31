#!/usr/bin/env python3
"""#700 item 2: the escape ledger gains a layer term, and `supply` does not move.

The ledger counted lanes ALONG a face and stopped there, so a 2-layer board and
a 6-layer one got the same answer. It now also reports what the other signal
layers could take -- bounded by the via slots in the escape band, which is the
issue's own suggestion -- under NEW keys, with `supply` and `deficit` untouched.

The first test is the one that matters most. Five independent places depend on
`deficit == demand - supply` (`FaceLedger.deficit`, `options.deficit_totals`,
`options.move_blocker`'s lanes-to-millimetres conversion, `routability`'s
`escape_worst_deficit`, `board_brief`'s POSITION_DEPENDENT allowlist), and
`test_capacity_options` pins `span_needed_mm == deficit_lanes * lane_pitch_mm`
to 1e-6. Additive keys are the only safe shape, and "additive" has to be
measured rather than intended.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import escape                                # noqa: E402

RUN_ALL_TIMEOUT = 600

BOARDS = ('rp2350_fpga_eensy_prePlane', 'glasgow_revC', 'tigard', 'watchy',
          'ulx3s', 'esp_prog', 'kit-dev-coldfire-xilinx_5213',
          'interf_u_plane', 'lvds_converter_dualclk', 'qfn_csi_underpad_diff')


def _board(name):
    p = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
    return (parse_kicad_pcb(p), p) if os.path.isfile(p) else (None, None)


def t_supply_and_deficit_are_untouched_on_every_board():
    """The non-regression. `supply` is computed the same way it always was,
    so the layer term cannot have moved it -- asserted anyway, because "cannot
    have" is what everybody says before a published number moves."""
    seen = 0
    for name in BOARDS:
        pcb, path = _board(name)
        if pcb is None:
            continue
        for pe in escape.escape_ledger(pcb, pcb_file=path):
            for f in pe.faces:
                # The identity every downstream consumer relies on.
                assert f.deficit == max(0, f.demand - f.supply), (name, pe.ref)
                assert f.supply == (int((f.span_mm - f.blocked_mm)
                                        // f.lane_pitch_mm)
                                    if f.lane_pitch_mm > 0 else 0), \
                    (name, pe.ref, f.face)
                seen += 1
    # 264 across the fixture list above; a floor rather than an exact count,
    # so a board leaving kicad_files/ is caught but a board joining it is not
    # a failure.
    assert seen > 200, f'only {seen} faces checked; the corpus went missing'
    print(f"  PASS: {seen} faces, supply and deficit unchanged by construction")


def t_the_layer_term_never_tightens_a_verdict():
    """`deficit_floor` is a LOWER bound, so it can never exceed `deficit`.

    This is the invariant that a negative `signal_layers - 1` would break, and
    interf_u_plane -- 2 copper layers, both 98% poured -- is the board that
    produces one if the clamp is removed.
    """
    seen = 0
    for name in BOARDS:
        pcb, path = _board(name)
        if pcb is None:
            continue
        for pe in escape.escape_ledger(pcb, pcb_file=path):
            for f in pe.faces:
                assert f.deficit_floor <= f.deficit, (name, pe.ref, f.face)
                assert f.supply_other_max >= 0, (name, pe.ref, f.face)
                assert f.other_layer_lanes >= 0, (name, pe.ref, f.face)
                assert f.via_slots >= 0, (name, pe.ref, f.face)
                seen += 1
    print(f"  PASS: {seen} faces, deficit_floor <= deficit everywhere")


def t_a_fully_poured_two_layer_board_clamps_instead_of_going_negative():
    """interf_u_plane: 2 copper layers, a named-net pour covering 98% of the
    board on BOTH. The raw subtraction is 0, and `(L-1) * lanes` would be
    NEGATIVE -- an upper bound tightening a verdict."""
    pcb, path = _board('interf_u_plane')
    if pcb is None:
        print("  SKIP: interf_u_plane not in kicad_files/")
        return
    n, source, planes = escape.signal_layer_count(pcb)
    assert n == 1, n
    assert source == 'zones', source
    assert set(planes) == {'F.Cu', 'B.Cu'}, planes
    # ...and the clamp is not silently hiding what it found.
    led = escape.escape_ledger(pcb, pcb_file=path,
                               refs=sorted(pcb.footprints)[:1])
    assert led and list(led[0].to_dict()['plane_layers_found']) == \
        ['B.Cu', 'F.Cu'], led[0].to_dict()['plane_layers_found']
    print("  PASS: both layers poured -> clamped to 1, and both are named")


def t_a_tiny_unnamed_zone_is_not_a_plane():
    """lvds_converter_dualclk carries a 0.4mm2 B.Cu zone with an EMPTY net
    name. Real planes in this corpus cover 0.94-0.98 of the board; that one
    covers 0.0002. Treating it as a plane would silently halve the board's
    signal-layer count on the strength of a sliver."""
    pcb, _path = _board('lvds_converter_dualclk')
    if pcb is None:
        print("  SKIP: lvds_converter_dualclk not in kicad_files/")
        return
    assert pcb.zones, 'the fixture lost its zone; this test proves nothing now'
    n, source, planes = escape.signal_layer_count(pcb)
    assert planes == (), planes
    assert n == 2 and source == 'copper_layers', (n, source)
    print("  PASS: a 0.4mm2 unnamed zone is not a plane")


def t_a_real_plane_stack_is_observed():
    """kit-dev-coldfire: GND on B.Cu and In1.Cu, +3.3V on In2.Cu, on a 4-layer
    board -- so ONE signal layer, observed rather than declared."""
    pcb, _path = _board('kit-dev-coldfire-xilinx_5213')
    if pcb is None:
        print("  SKIP: kit-dev-coldfire not in kicad_files/")
        return
    n, source, planes = escape.signal_layer_count(pcb)
    assert n == 1, n
    assert source == 'zones', source
    assert set(planes) == {'B.Cu', 'In1.Cu', 'In2.Cu'}, planes
    print("  PASS: 4 copper layers, 3 pours observed -> 1 signal layer")


def t_declared_planes_and_a_declared_count_outrank_observation():
    """The channel that matters on a board being placed, which has no pours."""
    pcb, _path = _board('kit-dev-coldfire-xilinx_5213')
    if pcb is None:
        print("  SKIP: kit-dev-coldfire not in kicad_files/")
        return
    n, source, planes = escape.signal_layer_count(pcb, plane_layers=['In1.Cu'])
    assert (n, source, planes) == (3, 'declared_planes', ('In1.Cu',)), \
        (n, source, planes)
    n, source, planes = escape.signal_layer_count(pcb, signal_layers=2)
    assert (n, source) == (2, 'declared_count'), (n, source)
    # A declared plane that is not a copper layer is ignored, not counted.
    n, _s, planes = escape.signal_layer_count(pcb, plane_layers=['F.SilkS'])
    assert n == 4 and planes == (), (n, planes)
    print("  PASS: declared_count > declared_planes > zones > copper_layers")


def t_a_board_with_no_layer_list_reports_todays_numbers():
    """`unknown` returns 1, which makes the layer term contribute exactly
    zero -- so the hand-built fixtures in test_escape_ledger.py, which have no
    board_info at all, keep their old answers."""
    class _Pcb:
        footprints = {}
        source_path = None

    n, source, planes = escape.signal_layer_count(_Pcb())
    assert (n, source, planes) == (1, 'unknown', ()), (n, source, planes)
    f = escape.FaceLedger(ref='U1', face='north', span_mm=10.0,
                          lane_pitch_mm=0.5, supply=20, demand=25,
                          blocked_mm=0.0, blockers=(), nets=tuple(range(25)))
    assert f.signal_layers == 1 and f.via_pitch_mm == 0.0
    assert f.supply_other_max == 0 and f.deficit_floor == f.deficit == 5
    assert f.supply_bound == 'no_other_layer'
    print("  PASS: no layer list -> the term is inert, not wrong")


def t_the_other_layer_term_survives_a_fully_blocked_face():
    """The defect in the obvious formulation, pinned as a test.

    `(signal_layers - 1) * supply` vanishes wherever `supply` is 0 -- and a
    face in deficit is overwhelmingly one whose channel was eaten. rp2350's
    U6 has supply 0 on ALL FOUR faces with demand 13/13/14/14, so the obvious
    form scores the worst part on the 6-layer board that motivated #700 a
    layer term of exactly zero.
    """
    pcb, path = _board('rp2350_fpga_eensy_prePlane')
    if pcb is None:
        print("  SKIP: rp2350 not in kicad_files/")
        return
    led = escape.escape_ledger(pcb, pcb_file=path, refs=['U6'])
    assert led, 'U6 not on this board any more'
    u6 = led[0]
    assert all(f.supply == 0 for f in u6.faces), \
        'U6 is no longer fully blocked; pick another fixture'
    assert all(f.demand > 0 for f in u6.faces), u6.to_dict()
    # The term must still say something here. This is the whole point.
    assert all(f.supply_other_max > 0 for f in u6.faces), \
        [f.to_dict() for f in u6.faces]
    assert all(f.deficit_floor < f.deficit for f in u6.faces), \
        [(f.face, f.deficit, f.deficit_floor) for f in u6.faces]
    print(f"  PASS: U6 supply 0 on 4 faces, layer term still "
          f"{u6.faces[0].supply_other_max} -- deficit "
          f"{u6.faces[0].deficit} -> {u6.faces[0].deficit_floor}")


def t_the_saturation_is_asserted_rather_than_discovered():
    """`via_slots` does not depend on the layer count, so `supply_other_max`
    is the SAME at 2 signal layers and at 6 on every corpus board.

    That is a real property of the bound, not a bug -- but it is exactly the
    complaint #700 makes one level up ("nothing behaves differently on a
    2-layer board than on a 6-layer one"), so it is pinned here. If a future
    change makes the layer count bind, this goes red and someone re-reads the
    design instead of quietly shipping a different meaning.
    """
    pcb, path = _board('glasgow_revC')
    if pcb is None:
        print("  SKIP: glasgow_revC not in kicad_files/")
        return
    two = escape.escape_ledger(pcb, pcb_file=path, signal_layers=2)
    six = escape.escape_ledger(pcb, pcb_file=path, signal_layers=6)
    pairs = [(a, b) for pa, pb in zip(two, six)
             for a, b in zip(pa.faces, pb.faces) if a.demand]
    assert pairs, 'no demanded faces to compare'
    same = [(a, b) for a, b in pairs if a.supply_other_max == b.supply_other_max]
    assert len(same) == len(pairs), [
        (a.ref, a.face, a.supply_other_max, b.supply_other_max)
        for a, b in pairs if a.supply_other_max != b.supply_other_max]
    bound = {a.supply_bound for a, _b in pairs}
    assert bound == {'via_slots'}, bound
    print(f"  PASS: {len(pairs)} demanded faces identical at 2 and 6 signal "
          f"layers; every one bounded by via_slots")


def t_the_via_pitch_obeys_both_the_copper_and_the_drill_rule():
    """`fab_tiers.min_via_center_distance`, resolved BOARD-FIRST like the lane
    pitch beside it -- not fab-floored. Fab-flooring one half against a
    board-first other half takes the ledger's summed deficit to ZERO on four
    of six real boards, which is a supply term erasing the finding it
    annotates."""
    pcb, path = _board('glasgow_revC')
    if pcb is None:
        print("  SKIP: glasgow_revC not in kicad_files/")
        return
    # Board's own: via 0.6 + clearance 0.2 = 0.8 copper; drill 0.3 + h2h.
    got = escape.via_pitch(pcb, path)
    assert abs(got - 0.8) < 1e-9, got
    # The drill rule must be able to WIN, or this only tests one branch.
    drill_bound = escape.via_pitch(pcb, path, clearance=0.05,
                                   via_diameter=0.3, via_drill=0.25,
                                   hole_to_hole=0.5)
    assert abs(drill_bound - 0.75) < 1e-9, drill_bound
    print(f"  PASS: {got}mm board-first, and the drill rule can bind")


def t_the_two_halves_of_a_row_use_one_clearance():
    """The lane half and the via half must price the same board at the same
    clearance. They agree on all 27 in-repo boards by coincidence today; the
    ledger passes `lane_pitch`'s resolved value in so they agree by
    construction."""
    pcb, path = _board('tigard')
    if pcb is None:
        print("  SKIP: tigard not in kicad_files/")
        return
    tw, clr = escape.lane_pitch_parts(pcb, path)
    assert abs((tw + clr) - escape.lane_pitch(pcb, path)) < 1e-12
    # An explicit clearance must move BOTH halves, not just the lane one.
    a = escape.escape_ledger(pcb, pcb_file=path, clearance=0.05)
    b = escape.escape_ledger(pcb, pcb_file=path, clearance=0.40)
    fa, fb = a[0].faces[0], b[0].faces[0]
    assert fa.lane_pitch_mm < fb.lane_pitch_mm, (fa.lane_pitch_mm,
                                                fb.lane_pitch_mm)
    assert fa.via_pitch_mm < fb.via_pitch_mm, (fa.via_pitch_mm, fb.via_pitch_mm)
    print(f"  PASS: clearance moves lane {fa.lane_pitch_mm}->{fb.lane_pitch_mm} "
          f"AND via {fa.via_pitch_mm}->{fb.via_pitch_mm}")


def t_worst_floor_is_a_property_not_only_a_dict_key():
    """`worst_deficit` exists only as a to_dict key, so
    `getattr(p, 'worst_deficit', 0)` silently returned 0 and turned "38 faces
    are short" into "0". One monument to that is enough."""
    pcb, path = _board('rp2350_fpga_eensy_prePlane')
    if pcb is None:
        print("  SKIP: rp2350 not in kicad_files/")
        return
    led = escape.escape_ledger(pcb, pcb_file=path)
    hit = [p for p in led if p.worst_floor]
    assert hit, 'no part is short at the floor; pick another fixture'
    p = hit[0]
    assert p.worst_floor.deficit_floor > 0
    assert p.to_dict()['worst_deficit_floor'] == p.worst_floor.deficit_floor
    assert p.to_dict()['worst_face_floor'] == p.worst_floor.face
    # A part with no floor deficit reports 0 / None, never a stale face.
    clean = [q for q in led if not q.worst_floor]
    if clean:
        assert clean[0].to_dict()['worst_deficit_floor'] == 0
        assert clean[0].to_dict()['worst_face_floor'] is None
    print(f"  PASS: worst_floor is a property; {len(hit)} part(s) short at "
          f"the floor")


def t_deficit_totals_can_sum_either_field_and_refuses_a_third():
    """An allowlist, not a getattr with a default -- the silent 0 is the exact
    failure this function's docstring is about."""
    from placement.options import deficit_totals
    pcb, path = _board('rp2350_fpga_eensy_prePlane')
    if pcb is None:
        print("  SKIP: rp2350 not in kicad_files/")
        return
    led = escape.escape_ledger(pcb, pcb_file=path)
    own = deficit_totals(led)
    floor = deficit_totals(led, field='deficit_floor')
    assert own['lanes'] > floor['lanes'] > 0, (own, floor)
    assert own['examined'] == floor['examined'] == len(led)
    # `parts` counted as "any face short" is provably the same as the old
    # `worst`-based count for the own-layer field.
    assert own['parts'] == len([p for p in led if p.worst]), own
    try:
        deficit_totals(led, field='worst_deficit')
    except ValueError as exc:
        assert 'worst_deficit' in str(exc), exc
    else:
        raise AssertionError('a mistyped field must raise, not return zeros')
    print(f"  PASS: {own['lanes']} own-layer lanes, {floor['lanes']} at the "
          f"floor, and a bad field name raises")


def t_the_health_block_reports_both_and_gates_on_neither():
    """`routability.health` is report-only by its own contract, and the layer
    keys must not change that."""
    import routing_defaults as defaults
    from placement import routability
    from placement.quench import QuenchState
    pcb, path = _board('rp2350_fpga_eensy_prePlane')
    if pcb is None:
        print("  SKIP: rp2350 not in kicad_files/")
        return
    # A real QuenchState, as test_549_routability builds one. `health` reads
    # the ledger off `pcb_data.source_path`, which `parse_kicad_pcb` sets --
    # there is no pcb_file parameter.
    state = QuenchState(pcb, path, clearance=defaults.CLEARANCE,
                        board_edge_clearance=0.55, crossing_penalty=10.0,
                        halo_base=0.5, halo_coef=0.15, halo_weight=2.0,
                        edge_halo=2.0, edge_weight=2.0,
                        grid_step=defaults.GRID_STEP, length_weight=1.0)
    out = routability.health(state, pcb, {}, {})
    for key in ('escape_worst_deficit', 'escape_worst_deficit_floor',
                'escape_deficit_parts', 'escape_deficit_parts_all_layers',
                'escape_signal_layers', 'escape_signal_layers_source',
                'escape_supply_bound'):
        assert key in out, (key, sorted(out))
    assert out['escape_worst_deficit_floor'] <= out['escape_worst_deficit']
    assert out['escape_signal_layers'] == 6, out['escape_signal_layers']
    assert out['escape_signal_layers_source'] == 'copper_layers', out
    assert out['escape_supply_bound'] == 'via_slots', out['escape_supply_bound']
    print(f"  PASS: worst {out['escape_worst_deficit']} own-layer / "
          f"{out['escape_worst_deficit_floor']} at the floor, bound "
          f"{out['escape_supply_bound']}")


TESTS = (t_supply_and_deficit_are_untouched_on_every_board,
         t_the_layer_term_never_tightens_a_verdict,
         t_a_fully_poured_two_layer_board_clamps_instead_of_going_negative,
         t_a_tiny_unnamed_zone_is_not_a_plane,
         t_a_real_plane_stack_is_observed,
         t_declared_planes_and_a_declared_count_outrank_observation,
         t_a_board_with_no_layer_list_reports_todays_numbers,
         t_the_other_layer_term_survives_a_fully_blocked_face,
         t_the_saturation_is_asserted_rather_than_discovered,
         t_the_via_pitch_obeys_both_the_copper_and_the_drill_rule,
         t_the_two_halves_of_a_row_use_one_clearance,
         t_worst_floor_is_a_property_not_only_a_dict_key,
         t_deficit_totals_can_sum_either_field_and_refuses_a_third,
         t_the_health_block_reports_both_and_gates_on_neither)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('t_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(fn.__name__)
        fn()
    print('\nALL PASS')
