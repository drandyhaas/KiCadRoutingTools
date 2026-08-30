"""
What tells you the FLOORPLAN is wrong, not the routing (issue #549, #407).

Discussion #407 names two scars, both floorplan errors a router can only grind
against: a magnetics block ~80mm from BOTH its own endpoints, and ~22 residual
nets that no knob could fix because the answer was re-floorplanning a quadrant.
Both were visible from move zero to anyone looking at the right number.

The tests that matter here are the ones about POWER, because they are what makes
the difference between a signal and a number that always says the same thing:

  * GND owns 96 of ulx3s's parts and +3V3 owns 45, out of 329 nets whose MEDIAN
    is 2. Left in, they dominate every block's foreign-pad set, the "net
    centroid" collapses onto the board centroid, and block displacement
    degenerates into "distance from the middle of the board".
  * The same nets cross EVERY corridor, because a 96-part MST sprays airwires
    over the whole board. Left in, the corridor signal reports "power crosses
    this bus" on every bus of every board and distinguishes nothing.

Both are asserted as A/B comparisons rather than as thresholds, since the point
is that the filtered and unfiltered numbers are DIFFERENT, not that either has a
particular value.

A signal that cannot be computed says so. `blocked_cell_share` needs #409's
blocker JSON, which only exists after a routing attempt; `convergence` needs a
declared list of critical classes, because placement has no net-class notion and
"critical" is design intent rather than a fact in the file. Neither is guessed.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from kicad_parser import parse_kicad_pcb
from placement.groups import derive_groups, short_name
from placement.quench import QuenchState
from placement.routability import (Corridor, DISPLACEMENT_MAX_FANOUT,
                                   block_displacements, corridor_intrusions,
                                   corridors_from_intent, foreign_crossings,
                                   health)
import routing_defaults as defaults

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ULX = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')

_CACHE = {}


def _state(path=ULX):
    if path not in _CACHE:
        pcb = parse_kicad_pcb(path)
        st = QuenchState(pcb, path, clearance=defaults.CLEARANCE,
                         board_edge_clearance=0.55, crossing_penalty=10.0,
                         halo_base=0.5, halo_coef=0.25, halo_weight=2.0,
                         edge_halo=2.0, edge_weight=2.0,
                         grid_step=defaults.GRID_STEP, length_weight=1.0)
        _CACHE[path] = (pcb, st, derive_groups(pcb, ('kicad', 'sheet')))
    return _CACHE[path]


def test_power_nets_would_otherwise_swamp_the_displacement_signal():
    """The A/B that justifies DISPLACEMENT_MAX_FANOUT existing at all.

    Measured on ulx3s. Unfiltered, 8 of 10 blocks report a foreign-pad count
    within 10% of the median 332 -- they are all seeing the same thing, the
    board's power nets, so the "net centroid" is really the board centroid.
    Filtered, the median drops to 40 and only 4 of 10 cluster, because each
    block is finally being measured against its OWN connectivity.
    """
    import statistics
    _pcb, st, blocks = _state()
    raw = block_displacements(st, blocks, max_fanout=0)
    cut = block_displacements(st, blocks)
    assert raw and cut

    def clustered(rows):
        vals = [d.foreign_pads for d in rows]
        med = statistics.median(vals)
        return sum(1 for v in vals if abs(v - med) <= 0.1 * med), med

    raw_n, raw_med = clustered(raw)
    cut_n, cut_med = clustered(cut)
    assert raw_n >= 7 * len(raw) // 10, [d.foreign_pads for d in raw]
    assert cut_n < raw_n, (raw_n, cut_n)
    assert raw_med > 4 * cut_med, (raw_med, cut_med)

    order_raw = [d.block for d in sorted(raw, key=lambda d: -d.distance_mm)]
    order_cut = [d.block for d in sorted(cut, key=lambda d: -d.distance_mm)]
    assert order_raw[:3] != order_cut[:3], "the cut changed nothing"
    print(f"  PASS: {raw_n}/{len(raw)} blocks cluster at median {raw_med:.0f} "
          f"foreign pads unfiltered vs {cut_n}/{len(cut)} at {cut_med:.0f}; "
          f"top-3 ranking changes")


def test_the_fanout_threshold_separates_this_boards_rails_cleanly():
    """Not a magic number: 329 nets, median 2 owning parts, exactly two above
    the threshold."""
    _pcb, st, _blocks = _state()
    counts = sorted(len(refs) for refs in st.net_refs.values())
    median = counts[len(counts) // 2]
    above = [n for n in counts if n > DISPLACEMENT_MAX_FANOUT]
    assert median <= 3, median
    assert len(above) <= 4, above
    assert max(counts) > 4 * DISPLACEMENT_MAX_FANOUT, max(counts)
    print(f"  PASS: {len(counts)} nets, median {median} parts, "
          f"{len(above)} above {DISPLACEMENT_MAX_FANOUT} (max {max(counts)})")


def test_an_explicit_ignore_list_is_honoured_like_ignore_nets():
    """The caller can name the plane nets, exactly as --ignore-nets does
    elsewhere, instead of relying on the fanout backstop."""
    pcb, st, blocks = _state()
    gnd = [nid for nid, n in pcb.nets.items() if n.name == 'GND']
    assert gnd, "no GND net on ulx3s"
    a = block_displacements(st, blocks, max_fanout=0)
    b = block_displacements(st, blocks, ignore_net_ids=gnd, max_fanout=0)
    assert [d.foreign_pads for d in a] != [d.foreign_pads for d in b]
    assert all(x.foreign_pads >= y.foreign_pads for x, y in zip(a, b))
    print(f"  PASS: naming GND alone drops "
          f"{sum(d.foreign_pads for d in a) - sum(d.foreign_pads for d in b)} "
          f"foreign pads")


def test_moving_a_block_away_from_its_nets_raises_its_displacement():
    """The metric has to respond to the thing it claims to measure.

    The shift is along the block's OWN displacement vector, away from its net
    centroid. A fixed +20mm in x is not a valid probe: on the first block tried
    it moved the block TOWARDS its partners and the distance correctly fell
    17.07 -> 4.71mm.
    """
    _pcb, st, blocks = _state()
    rows = {d.block: d for d in block_displacements(st, blocks)}
    target = min(rows, key=lambda k: len(blocks[k]))
    d0 = rows[target]
    vx = d0.centroid[0] - d0.net_centroid[0]
    vy = d0.centroid[1] - d0.net_centroid[1]
    n = max(1e-9, (vx * vx + vy * vy) ** 0.5)
    shift = 20.0
    for ref in blocks[target]:
        if ref in st.parts:
            p = st.parts[ref]
            st.apply_move(ref, p.x + shift * vx / n, p.y + shift * vy / n, p.rot)

    after = {d.block: d.distance_mm for d in block_displacements(st, blocks)}
    grew = after[target] - d0.distance_mm
    assert grew > shift * 0.5, (d0.distance_mm, after[target])

    # Other blocks move too, and that is CORRECT rather than leakage: the parts
    # that just moved are foreign pads on every block sharing a net with them,
    # so their net centroids shift as well. The property worth asserting is that
    # the block that was moved is affected MOST.
    others = {k: after[k] - rows[k].distance_mm for k in rows if k != target}
    assert grew > max(abs(v) for v in others.values()), \
        (grew, sorted(others.items(), key=lambda kv: -abs(kv[1]))[:3])
    _CACHE.clear()                       # the state was mutated; do not reuse
    print(f"  PASS: {short_name(target)} {d0.distance_mm:.2f} -> "
          f"{after[target]:.2f}mm (+{grew:.2f}) after a {shift}mm shift along "
          f"its own displacement vector -- the largest change of any block")


def test_a_block_with_no_foreign_pads_is_omitted_not_reported_as_zero():
    """'Connects to nothing outside itself' and 'sits exactly on its partners'
    are different facts, and 0.0 reads as the second."""
    _pcb, st, blocks = _state()
    name = min(blocks, key=lambda k: len(blocks[k]))
    one = {name: blocks[name]}

    kept = block_displacements(st, one)
    assert len(kept) == 1 and kept[0].foreign_pads > 0, kept

    # Ignore every net this block owns: it now connects to nothing outside
    # itself, and must vanish from the report rather than appear at 0.0mm.
    own = {nid for ref in blocks[name] if ref in st.parts
           for _x, _y, nid in st.parts[ref].pad_globals()}
    gone = block_displacements(st, one, ignore_net_ids=sorted(own))
    assert gone == [], gone
    assert all(d.foreign_pads > 0 for d in block_displacements(st, blocks))
    print(f"  PASS: {short_name(name)} reports {kept[0].foreign_pads} foreign "
          f"pads, and vanishes (not 0.0mm) once its nets are ignored")


def test_a_corridor_counts_crossings_with_the_quenchs_own_kernel():
    pcb, st, _blocks = _state()
    corr = corridors_from_intent(st, pcb, [
        {'name': 'sdram', 'nets': ['SDRAM_*'], 'width_mm': 8.0}])
    assert corr, "SDRAM_* matched no corridor on ulx3s"
    c = corr[0]
    assert len(c.net_ids) > 4 and c.length_mm > 1.0
    assert len(c.side_edges()) == 2
    n, guilty = foreign_crossings(st, c)
    assert n > 0 and guilty
    assert not (set(guilty) & set(c.net_ids)), "a corridor crossed itself"
    print(f"  PASS: corridor {c.name} {c.length_mm:.1f}mm x {c.width_mm}mm, "
          f"{n} foreign crossing(s) from {len(guilty)} net(s)")


def test_power_would_otherwise_cross_every_corridor():
    """A 96-part MST sprays airwires over the whole board, so unfiltered the
    top offenders are the same rails on every bus of every board."""
    pcb, st, _blocks = _state()
    c = corridors_from_intent(st, pcb, [
        {'name': 'sdram', 'nets': ['SDRAM_*'], 'width_mm': 8.0}])[0]
    raw_n, raw_guilty = foreign_crossings(st, c, max_fanout=0)
    cut_n, cut_guilty = foreign_crossings(st, c)
    names = [pcb.nets[i].name for i in raw_guilty[:3] if i in pcb.nets]
    assert raw_n > cut_n, (raw_n, cut_n)
    assert any(n in ('GND', '+3V3') for n in names), names
    cut_names = [pcb.nets[i].name for i in cut_guilty[:3] if i in pcb.nets]
    assert 'GND' not in cut_names, cut_names
    print(f"  PASS: {raw_n} crossings worst={names} unfiltered -> "
          f"{cut_n} worst={cut_names} filtered")


def test_a_corridor_is_deterministic():
    """#457: the endpoint clustering picks extreme pads, and its orientation is
    fixed rather than left to iteration order."""
    pcb, st, _blocks = _state()
    spec = [{'name': 'sdram', 'nets': ['SDRAM_*'], 'width_mm': 8.0}]
    a = corridors_from_intent(st, pcb, spec)[0]
    b = corridors_from_intent(st, pcb, spec)[0]
    assert (a.a, a.b, a.net_ids) == (b.a, b.b, b.net_ids)
    print(f"  PASS: corridor endpoints stable at "
          f"({a.a[0]:.3f},{a.a[1]:.3f})-({a.b[0]:.3f},{a.b[1]:.3f})")


def test_convergence_is_skipped_rather_than_guessed():
    """Placement has no net-class notion, and 'critical' is design intent, not
    a fact in the board file. Without a declared list this must not invent one."""
    pcb, st, blocks = _state()
    h = health(st, pcb, blocks, {
        'bus_corridors': [{'name': 'sdram', 'nets': ['SDRAM_*']}]})
    assert 'convergence' not in h
    assert 'convergence' in h['skipped']
    assert 'design intent' in h['skipped']['convergence']

    h2 = health(st, pcb, blocks, {
        'bus_corridors': [{'name': 'sdram', 'nets': ['SDRAM_*']}],
        'classes': {'USB': ['usb_*'], 'SDRAM': ['SDRAM_*']}})
    assert 'convergence' in h2 and 'convergence' not in h2['skipped']
    print("  PASS: no classes -> skipped with the reason; classes -> measured")


def test_the_post_route_signal_says_it_needs_a_route():
    """A signal that cannot be computed must say so rather than report 0."""
    pcb, st, blocks = _state()
    h = health(st, pcb, blocks, {})
    assert 'blocked_cell_share' not in h
    assert 'needs a routing attempt' in h['skipped']['blocked_cell_share']
    assert '409' in h['skipped']['blocked_cell_share']
    print(f"  PASS: {h['skipped']['blocked_cell_share'][:70]}")


def test_health_reports_nothing_it_was_not_asked_for():
    pcb, st, blocks = _state()
    h = health(st, pcb, blocks, {})
    assert 'bus_corridors' not in h and 'bus_corridors' in h['skipped']
    assert 'block_displacement' in h          # blocks exist, so this one runs
    h2 = health(st, pcb, {}, {})
    assert 'block_displacement' not in h2
    assert 'no blocks resolved' in h2['skipped']['block_displacement']
    print(f"  PASS: {len(h['skipped'])} signal(s) skipped with a reason each; "
          f"no blocks -> displacement skipped too")


def test_a_part_body_parked_in_the_channel_is_reported_with_its_depth():
    """foreign_crossings sees airwires; it cannot see a COURTYARD parked in
    the corridor (run 5's R10: 1.03mm INSIDE the QSPI channel, measured by
    hand). corridor_intrusions makes that number mechanical: a synthetic
    corridor driven straight through a known part must report that part with
    a positive depth, and a corridor routed through empty space reports
    nothing."""
    pcb, st, _blocks = _state()
    ref, part = next((r, p) for r, p in sorted(st.parts.items())
                     if p.pin_count >= 2)
    x1, y1, x2, y2 = part.rect()
    cy = (y1 + y2) / 2
    through = Corridor(name='probe', net_ids=(),
                       a=(x1 - 5.0, cy), b=(x2 + 5.0, cy), width_mm=2.0)
    rows = corridor_intrusions(st, through)
    hit = next((r for r in rows if r['ref'] == ref), None)
    assert hit is not None, f"{ref} sits in the corridor but is not reported"
    assert 0 < hit['depth_mm'] <= 2.0 and hit['along_mm'] > 0, hit
    # empty space: a corridor far off-board reports nothing
    empty = Corridor(name='void', net_ids=(), a=(-500.0, -500.0),
                     b=(-400.0, -500.0), width_mm=2.0)
    assert corridor_intrusions(st, empty) == []
    print(f"  PASS: {ref} reported {hit['depth_mm']}mm inside the channel; "
          f"an empty corridor reports nothing")


TESTS = [
    test_power_nets_would_otherwise_swamp_the_displacement_signal,
    test_a_part_body_parked_in_the_channel_is_reported_with_its_depth,
    test_the_fanout_threshold_separates_this_boards_rails_cleanly,
    test_an_explicit_ignore_list_is_honoured_like_ignore_nets,
    test_a_corridor_counts_crossings_with_the_quenchs_own_kernel,
    test_power_would_otherwise_cross_every_corridor,
    test_a_corridor_is_deterministic,
    test_convergence_is_skipped_rather_than_guessed,
    test_the_post_route_signal_says_it_needs_a_route,
    test_health_reports_nothing_it_was_not_asked_for,
    test_a_block_with_no_foreign_pads_is_omitted_not_reported_as_zero,
    # mutates the cached state, so it runs last
    test_moving_a_block_away_from_its_nets_raises_its_displacement,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
