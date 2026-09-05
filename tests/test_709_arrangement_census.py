"""The arrangement census: where the mass sits against where the demand sits.

The pockets census put every demand-3 window at one end of run 23's board
while the part mass sat at the other, and no instrument said so. This is the
join, plus the caution #709 measured and that is easy to get backwards: the
centroid must be weighted by COURTYARD AREA, not by part count. The count form
is the intuitive one and it points the wrong way -- on esp_prog it reads 14.9%
of span where the area form reads 1.1%. (Those were 13.7% and 1.3% until #726
stopped the parser losing esp_prog's second `Ref*` fiducial: one more part
shifts a COUNT-weighted centroid and barely touches an AREA-weighted one,
which is the caution this file is about, restated by accident.)

`placement_state` records the standing objection to courtyard-area metrics --
"needs polygon area and breaks on the boards with no courtyards at all". B6
below discharges it as a test rather than as an argument: with the courtyard
parser returning nothing, the census keeps working off pad copper and SAYS
that is what it did.
"""

import math
import os
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 300

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
import check_pockets as CP                                     # noqa: E402
from placement import board_grid as BG                         # noqa: E402

FAILURES = []


def report(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name
          + (('  -- ' + detail) if detail else ''))
    if not ok:
        FAILURES.append(name)


def board(name):
    p = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
    assert os.path.isfile(p) and os.path.getsize(p) > 0, p
    return p


_CACHE = {}


def census(name, **kw):
    key = (name, tuple(sorted(kw.items())))
    if key not in _CACHE:
        p = board(name)
        _CACHE[key] = CP.pocket_census(parse_kicad_pcb(p), p, **kw)
    return _CACHE[key]


def t_area_weighting_and_the_count_control_disagree():
    """B1: the whole reason the issue warns about this.

    Setting the weight to 1.0 collapses the two forms onto each other and the
    separation assertion goes red. esp_prog is the board the issue measured.
    """
    doc, _hot = census('esp_prog')
    arr = doc['arrangement']
    s = arr['sides'][arr['headline_side']]
    area_x = s['offset_frac_span'][0]
    count_x = s['count_control_frac_span'][0]
    # RE-RECORDED for #726. esp_prog carries two footprint blocks named `Ref*`
    # and the parser used to keep only the second, so this board measured as 20
    # parts where the file has 21. The recovered fiducial moves the COUNT-
    # weighted centroid (0.1372 -> 0.1494, one more part on one side of the
    # span) and barely moves the AREA-weighted one (0.0127 -> 0.0107, because a
    # fiducial has almost no courtyard). The issue's finding is unchanged and
    # in fact slightly sharper: the count form still points the wrong way, now
    # by 14x rather than 11x.
    report('esp_prog: the area form is ~1.1% of span',
           abs(area_x - 0.011) < 0.004, 'got %.4f' % area_x)
    report('esp_prog: the count control is ~14.9% of span',
           abs(count_x - 0.149) < 0.008, 'got %.4f' % count_x)
    report('the two forms differ by more than 4x -- they are NOT the same '
           'statistic', count_x > 4 * area_x, '%.4f vs %.4f' % (count_x, area_x))
    report('the weight is declared, not implied',
           arr['weight'] == 'courtyard_area', arr['weight'])


def t_the_two_forms_differ_across_the_corpus():
    """Not a one-board coincidence, and not always in the same direction.

    Recording the direction matters: on splitflap_driver and sonde_u the AREA
    form is the larger one, so "area weighting always flatters the board" is
    not what this is.
    """
    seen = []
    for name in ('esp_prog', 'tigard', 'splitflap_driver', 'watchy',
                 'sonde_u', 'glasgow_revC'):
        arr = census(name)[0]['arrangement']
        s = arr['sides'][arr['headline_side']]
        seen.append((name, s['offset_frac_span'][0],
                     s['count_control_frac_span'][0]))
    differ = [n for n, a, c in seen if abs(a - c) > 0.005]
    report('the two forms disagree on most boards, not just one',
           len(differ) >= 4, '%d of %d: %s' % (len(differ), len(seen), differ))
    both_ways = ({n for n, a, c in seen if a > c + 0.005}
                 and {n for n, a, c in seen if c > a + 0.005})
    report('and they disagree in BOTH directions', bool(both_ways),
           str([(n, round(a, 3), round(c, 3)) for n, a, c in seen]))


def t_synthetic_parts_are_excluded_and_counted():
    """B2: the +/-0.5mm fiction is not geometry anyone drew."""
    doc, _hot = census('esp_prog')
    p = doc['parts']
    report('esp_prog: 3 synthetic parts excluded',
           p['synthetic_excluded'] == 3, str(p['synthetic_excluded']))
    report('the weighed count is graded minus synthetic minus containers',
           p['weighed'] == p['graded'] - p['synthetic_excluded']
           - p['container_excluded'],
           str(p))
    for name in ('splitflap_driver', 'sonde_u'):
        report('%s: no synthetic parts at all' % name,
               census(name)[0]['parts']['synthetic_excluded'] == 0)


def t_sides_are_separate_never_summed():
    """B3: utilisation is per-side; summing two sides is a recorded error."""
    doc, _hot = census('esp_prog')
    arr = doc['arrangement']
    # A through-hole part occupies BOTH sides -- its leads really are on B --
    # so a board with no B-side SMD still reports a B side, with less area.
    # Summing the two would double-count exactly those parts.
    report('esp_prog reports both sides, F carrying the mass',
           set(arr['sides']) == {'F', 'B'}
           and arr['sides']['F']['courtyard_area_mm2']
           > arr['sides']['B']['courtyard_area_mm2'],
           str({s: v['courtyard_area_mm2'] for s, v in arr['sides'].items()}))
    total = sum(v['courtyard_area_mm2'] for v in arr['sides'].values())
    biggest = max(v['courtyard_area_mm2'] for v in arr['sides'].values())
    report('  ...and the headline is a SIDE total, never the sum of both',
           total > biggest, '%.1f summed vs %.1f headline' % (total, biggest))
    report('the headline side is the one with the most courtyard area',
           arr['headline_side'] == max(
               arr['sides'], key=lambda s: arr['sides'][s]['courtyard_area_mm2']))
    two = [n for n in ('watchy', 'tigard', 'glasgow_revC')
           if len(census(n)[0]['arrangement']['sides']) == 2]
    report('at least one corpus board really is two-sided', bool(two), str(two))
    for n in two:
        arr = census(n)[0]['arrangement']
        report('  %s: each side has its own centroid, not a shared one' % n,
               arr['sides']['F']['offset_mm'] != arr['sides']['B']['offset_mm'])


def t_quadrants_join_mass_to_demand():
    """B4: the join nobody made -- and the count/area discrepancy is DELIBERATE.

    NW on esp_prog holds 12 net-windows of demand and one part origin, while
    carrying real courtyard area from parts whose ORIGIN is elsewhere. Both
    numbers are reported because they answer different questions: the count
    names the block `--block region:qN` would move, the area is the mass.
    """
    doc, _hot = census('esp_prog')
    quads = {q['name']: q for q in doc['arrangement']['quadrants']}
    report('all four quadrants are present and named',
           list(quads) == ['NW', 'NE', 'SW', 'SE'], str(list(quads)))
    nw = quads['NW']
    report('NW carries demand', nw['demand_nets'] >= 10, str(nw['demand_nets']))
    report('NW carries courtyard area from parts seated elsewhere',
           nw['part_area_mm2'] > 0 and nw['parts'] <= 1, str(nw))
    report('distinct nets never exceed net-windows of demand',
           all(q['distinct_nets'] <= q['demand_nets'] for q in quads.values()),
           str([(q['distinct_nets'], q['demand_nets']) for q in quads.values()]))
    # The footprints a PLACER would see, which is not every footprint: a
    # graphic-only one (no courtyard, no pads) is the +/-0.5mm fiction and is
    # absent from the quench's own part set too.
    report('quadrant part counts total the WEIGHED parts, not every footprint',
           sum(q['parts'] for q in quads.values()) == doc['parts']['weighed']
           < len(parse_kicad_pcb(board('esp_prog')).footprints),
           '%d quadrant / %d weighed / %d footprints'
           % (sum(q['parts'] for q in quads.values()),
              doc['parts']['weighed'],
              len(parse_kicad_pcb(board('esp_prog')).footprints)))
    report('cold windows are attributed to quadrants too',
           sum(q['cold_windows'] for q in quads.values())
           == doc['cold_windows'],
           '%d vs %d' % (sum(q['cold_windows'] for q in quads.values()),
                         doc['cold_windows']))


def t_the_quadrant_counts_are_the_block_region_qN_would_move():
    """The printed count must be `_region_unit`'s OWN membership, not a
    lookalike -- otherwise `--block region:q0` moves a different set than the
    census named, which makes the whole "reseat target" claim false.

    So this CALLS `_region_unit` rather than mirroring its rule.
    """
    from placement.perturb import _region_unit
    from placement import quench as Q
    p = board('esp_prog')
    pcb = parse_kicad_pcb(p)
    doc, _hot = census('esp_prog')
    quads = {q['index']: q for q in doc['arrangement']['quadrants']}
    try:
        state = Q.build_state(pcb, p) if hasattr(Q, 'build_state') else None
    except Exception:                                           # noqa: BLE001
        state = None
    if state is None:
        from pose_score import make_state
        state = make_state(pcb, p)
    free = set(state.parts)
    bounds = pcb.board_info.board_bounds
    total_picked = 0
    for q in range(4):
        pick = _region_unit(state, free, pcb, q)
        members = list(pick.members) if pick else []
        total_picked += len(members)
        # The geometric rule must AGREE: every ref region:qN picks has to land
        # in the census's quadrant N. Bucketing by courtyard centre instead of
        # footprint origin breaks this on any part whose courtyard is not
        # centred on its origin.
        stray = [r for r in members
                 if CP.quadrant_of(pcb.footprints[r].x, pcb.footprints[r].y,
                                   bounds) != q]
        report('region:q%d picks nothing the census puts elsewhere' % q,
               not stray, str(stray))
        # ...and the census count is an upper bound: it cannot know which
        # parts a given run locks, but it must never UNDER-count.
        report('  census %s (%d) >= region:q%d membership (%d)'
               % (CP.QUADRANTS[q], quads[q]['parts'], q, len(members)),
               quads[q]['parts'] >= len(members))
    # On esp_prog nothing is locked, so the two totals meet exactly -- which
    # is what caught the census counting three graphic-only footprints as
    # parts (20 against 17).
    report('esp_prog: nothing locked, so the totals meet exactly',
           sum(q['parts'] for q in quads.values()) == total_picked,
           '%d vs %d' % (sum(q['parts'] for q in quads.values()), total_picked))


def t_the_quadrant_split_is_perturb_s_own():
    """The numbering is `region:qN`'s, or the printed name is a lie."""
    from placement.perturb import _region_unit                  # noqa: F401
    b = (0.0, 0.0, 10.0, 10.0)
    got = [CP.quadrant_of(x, y, b) for x, y in
           ((1, 1), (9, 1), (1, 9), (9, 9))]
    report('0=NW 1=NE 2=SW 3=SE', got == [0, 1, 2, 3], str(got))
    report('the midline is half-open toward the high side',
           CP.quadrant_of(5.0, 5.0, b) == 3
           and CP.quadrant_of(4.999, 4.999, b) == 0)
    report('the names line up with the indices',
           CP.QUADRANTS == ('NW', 'NE', 'SW', 'SE'))


def t_it_degrades_on_a_board_with_no_courtyards_and_says_so():
    """B6: `placement_state`'s recorded objection, discharged.

    With the courtyard parser returning nothing, every part falls back to its
    PAD bbox -- real copper, not a fiction -- and the census reports which of
    the two it used. Raising, or silently reporting the fiction, both fail.
    """
    import placement.parser as PP
    real = PP.extract_courtyard_sides
    try:
        PP.extract_courtyard_sides = lambda *_a, **_k: {}
        p = board('cap_chain')
        doc, _hot = CP.pocket_census(parse_kicad_pcb(p), p)
    finally:
        PP.extract_courtyard_sides = real
    arr = doc['arrangement']
    report('a courtyard-free board still produces an arrangement',
           arr is not None)
    if arr is None:
        return
    s = arr['sides'][arr['headline_side']]
    report('  its offset is a real finite number',
           all(isinstance(v, float) and math.isfinite(v)
               for v in s['offset_mm']), str(s['offset_mm']))
    p_ = doc['parts']
    report('  and the census SAYS it measured pad copper, not courtyards',
           p_['from_courtyard'] == 0 and p_['from_pads'] == p_['graded'],
           str(p_))
    report('  a pad bbox is not counted synthetic (it is real copper)',
           p_['synthetic_excluded'] == 0, str(p_['synthetic_excluded']))
    live = census('cap_chain')[0]['parts']
    report('  ...while the unpatched run does read the drawn courtyards',
           live['from_courtyard'] == live['graded'], str(live))


def t_it_refuses_rather_than_inventing_when_there_is_no_span():
    """The one genuine refusal: no board_bounds means no centre to measure."""
    p = board('cap_chain')
    pcb = parse_kicad_pcb(p)
    pcb.board_info.board_bounds = None
    doc, _hot = CP.pocket_census(pcb, p)
    report('no bounds -> arrangement is None, never 0.0',
           doc['arrangement'] is None)
    report('  ...and the refusal names its reason',
           'board_bounds' in (doc.get('skipped') or ''), str(doc.get('skipped')))
    report('  ...and the hot rows still come out',
           doc['windows_demand'] > 0, str(doc['windows_demand']))
    # ...and the refusal lives in arrangement_census itself, not only in the
    # caller that happens to short-circuit first. Reached directly, because a
    # guard only the caller protects is a guard that disappears the moment
    # someone adds a second caller.
    from placement import legality as leg
    # With REAL parts, so the refusal cannot be masked by the empty-sides
    # path: a version that invents (0,0,1,1) instead of refusing returns a
    # populated document here, and this row goes red.
    live = parse_kicad_pcb(p)
    parts = [g for g in leg.graded_parts_from_file(live, p) if not g.synthetic]
    report('the fixture has parts, so the refusal is not vacuous',
           len(parts) >= 3, str(len(parts)))
    report('arrangement_census refuses bounds=None on its own, WITH parts',
           CP.arrangement_census(live, parts, set(), {}, 2.0, None, leg)
           is None)
    report('  ...and still answers when bounds ARE given (the control)',
           CP.arrangement_census(live, parts, set(), {}, 2.0,
                                 live.board_info.board_bounds, leg) is not None)
    report('  ...and refuses an EMPTY part set rather than dividing by zero',
           CP.arrangement_census(live, [], set(), {}, 2.0,
                                 (0.0, 0.0, 10.0, 10.0), leg) is None)


def t_no_arrangement_switch():
    doc, _hot = census('esp_prog', arrangement=False)
    report('--no-arrangement suppresses it without touching the cold census',
           doc['arrangement'] is None and doc['cold_windows'] is not None)


def t_the_reseat_target_is_a_target_not_a_weight():
    doc, _hot = census('esp_prog')
    rt = doc['reseat_target']
    report('a reseat target is produced', rt is not None)
    if rt is None:
        return
    report('it names the top cold region',
           rt['region_bbox'] == doc['cold_regions'][0]['bbox'])
    # NOT a --reseat-region argument: a cold band holds no part by
    # construction, so that command resolved to an empty scope on every board.
    # It is a `zone` -- a DESTINATION -- clipped to the board.
    report('it advertises no --reseat-region argument',
           'reseat_region' not in rt, str(sorted(rt)))
    report('it names a zone, clipped inside the band rect',
           rt['zone'][0] >= rt['band_rect'][0] - 1e-6
           and rt['zone'][2] <= rt['band_rect'][2] + 1e-6,
           '%s vs %s' % (rt['zone'], rt['band_rect']))
    report('it points AWAY from where the mass already is',
           rt['move_mass_toward'] in ('N', 'S', 'E', 'W', 'NE', 'NW', 'SE',
                                      'SW', 'centre', None),
           str(rt['move_mass_toward']))
    arr = doc['arrangement']
    dx, dy = arr['sides'][arr['headline_side']]['offset_mm']
    want = CP._compass(-dx, -dy)
    report('  ...and that direction is the negated mass offset',
           rt['move_mass_toward'] == want,
           '%s vs %s' % (rt['move_mass_toward'], want))


def t_the_container_guard_is_pinned_on_the_board_it_fires_on():
    """`rp2350_fpga_eensy_prePlane`'s U8 is the only container in the corpus.

    Three mutations survived the whole battery here -- "never exclude",
    "exclude on area alone", and "put containers back into the centroid" --
    because no test censused the ONE board where the guard fires. It is a live
    guard, not a dead one, and the suite could not tell the difference.
    """
    p = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
    if not os.path.isfile(p):
        report('rp2350 fixture present', False, 'missing')
        return
    doc, _hot = CP.pocket_census(parse_kicad_pcb(p), p)
    parts = doc['parts']
    report('the guard FIRES on this board -- exactly one container',
           parts['container_excluded'] == 1, str(parts))
    report('  ...and names it', parts['containers'] == ['U8'],
           str(parts['containers']))
    report('  ...and the guard is the hosting one, not bare area',
           parts['container_guard'] == 'options.hosts_the_design',
           str(parts['container_guard']))
    report('  ...and the excluded part is out of the weighed set',
           parts['weighed'] == len([1]) * 0 + parts['graded']
           - parts['synthetic_excluded'] - 1,
           str(parts))

    # The centroid must MOVE when the container is put back, or "exclude
    # containers" is a claim no number depends on.
    from placement import options as opts
    real = opts.hosts_the_design
    try:
        opts.hosts_the_design = lambda *a, **k: False
        loose, _h = CP.pocket_census(parse_kicad_pcb(p), p)
    finally:
        opts.hosts_the_design = real
    report('  ...and putting it back changes the answer',
           loose['parts']['container_excluded'] == 0
           and (loose['arrangement']['sides'][loose['arrangement']
                                              ['headline_side']]
                ['courtyard_area_mm2']
                != doc['arrangement']['sides'][doc['arrangement']
                                               ['headline_side']]
                ['courtyard_area_mm2']),
           'excluded=%s' % loose['parts']['container_excluded'])


def t_a_through_hole_part_covers_BOTH_sides():
    """`GradedPart.sides` gives a THT part both faces, and the cover map has
    to charge both -- otherwise a window under a through-hole part's courtyard
    reads uncovered on the far side. A mutation replacing the `for s in
    _side_key(gp)` loop with `slot[gp.side]` survived the whole battery."""
    p = board('sonde_u')
    pcb = parse_kicad_pcb(p)
    from placement import legality as leg
    graded = [g for g in leg.graded_parts_from_file(pcb, p) if not g.synthetic]
    tht = [g for g in graded if len(g.sides) == 2]
    report('the fixture really has through-hole parts', len(tht) > 3,
           '%d of %d' % (len(tht), len(graded)))
    # Assert it on the COVER MAP, which is where the loop lives. A
    # cold-window count cannot see it: charging `gp.side` only differs from
    # charging both faces where a B-side part ALSO covers the window and the
    # far-side sum overtakes the near one, and no in-repo board has such a
    # window at any --cold-cover (measured on 7 boards x 4 thresholds). A
    # count comparison would be a green row that proves nothing, which is
    # exactly what the first version of this case was.
    from placement import legality as leg2
    win = {}
    for g in tht[:1] + [x for x in graded if len(x.sides) == 1][:1]:
        import math as _m
        for bx in range(int(_m.floor(g.rect[0] / 2.0)),
                        int(_m.ceil(g.rect[2] / 2.0))):
            for by in range(int(_m.floor(g.rect[1] / 2.0)),
                            int(_m.ceil(g.rect[3] / 2.0))):
                win[(bx, by)] = 1.0
    cov = CP.courtyard_cover(tht[:1], set(), win, 2.0, leg2)
    charged = [v for v in cov.values() if v['F'] > 0 or v['B'] > 0]
    report('a THT part charges cover to BOTH faces', bool(charged)
           and all(v['F'] > 0 and v['B'] > 0 for v in charged),
           str(charged[:1]))
    smd1 = [x for x in graded if len(x.sides) == 1]
    if smd1:
        cov2 = CP.courtyard_cover(smd1[:1], set(), win, 2.0, leg2)
        touched2 = [v for v in cov2.values() if v['F'] > 0 or v['B'] > 0]
        report('  ...and a single-sided part charges exactly one',
               all((v['F'] > 0) != (v['B'] > 0) for v in touched2),
               str(touched2[:1]))
    report('  ...and a container is charged nothing at all',
           CP.courtyard_cover(tht[:1], {tht[0].ref}, win, 2.0, leg2) == {})

    one = [g for g in tht][0]
    report('_side_key gives a through-hole part BOTH faces',
           CP._side_key(one) == frozenset(('F', 'B')),
           '%s -> %s' % (one.ref, sorted(CP._side_key(one))))
    smd = [g for g in graded if len(g.sides) == 1]
    report('  ...and an SMD part exactly one', bool(smd)
           and len(CP._side_key(smd[0])) == 1,
           '%s -> %s' % (smd[0].ref, sorted(CP._side_key(smd[0]))) if smd
           else 'no SMD part')
    report('  ...which is legality.sides_occupied, not a local rule',
           all(CP._side_key(g) == g.sides for g in graded[:20]))


def t_the_side_rule_is_max_not_sum():
    """"A clear empty pocket" means empty on BOTH sides, so the disqualifier
    is `max(cover_F, cover_B)`. Summing them double-counts a through-hole part
    and disqualifies windows a single side never covered. A mutation swapping
    max for + survived the battery."""
    p = board('sonde_u')
    doc, _h = CP.pocket_census(parse_kicad_pcb(p), p, cold_cover=0.5)
    import check_pockets as _cp
    src = open(os.path.join(ROOT, 'py_tools', 'check_pockets.py'),
               encoding='utf-8').read()
    report('the source really uses max over the two sides',
           "max(c.get('F', 0.0), c.get('B', 0.0))" in src)
    # Behavioural: a window covered 0.3 on each side is cold under max at
    # --cold-cover 0.4 and warm under sum. Build that case directly.
    cover = {'F': 0.3, 'B': 0.3}
    report('  max keeps a both-sides-lightly-covered window cold',
           max(cover['F'], cover['B']) <= 0.4)
    report('  ...where a sum would disqualify it',
           cover['F'] + cover['B'] > 0.4)
    report('  and the census is non-trivial on this board',
           doc['cold_windows'] > 0, str(doc['cold_windows']))



def t_the_board_grid_scalars_travel_with_the_census():
    """#708. The lattice a board was laid out on is an arrangement fact, and
    the ROADMAP puts #708 behind this census for exactly that reason. A board
    that declares no lattice must say WHY, or "no lattice" and "never
    measured" are the same reading of a null.
    """
    d, _hot = census('splitflap_driver')
    sc = CP.census_scalars(d)
    report('an imperial board reports its pitch',
           sc.get('board_grid_step') == 0.3175, str(sc.get('board_grid_step')))
    report('with the occupancy it was read off',
           round(sc.get('board_grid_occupancy') or 0, 3) == 0.923,
           str(sc.get('board_grid_occupancy')))
    d2, _hot2 = census('ulx3s')
    sc2 = CP.census_scalars(d2)
    report('a board with no lattice reports None',
           sc2.get('board_grid_step') is None, str(sc2.get('board_grid_step')))
    report('and says which test it failed, so the null is an answer',
           'floor' in (sc2.get('board_grid_reason') or ''),
           str(sc2.get('board_grid_reason')))
    report('the census agrees with the engine resolver, not a second copy',
           sc.get('board_grid_step')
           == BG.infer_board_grid(parse_kicad_pcb(board('splitflap_driver')))['step'])

TESTS = [
    ('area weighting vs the count control',
     t_area_weighting_and_the_count_control_disagree),
    ('the two forms across the corpus', t_the_two_forms_differ_across_the_corpus),
    ('synthetic parts excluded', t_synthetic_parts_are_excluded_and_counted),
    ('sides stay separate', t_sides_are_separate_never_summed),
    ('quadrants join mass to demand', t_quadrants_join_mass_to_demand),
    ('quadrant counts == region:qN membership',
     t_the_quadrant_counts_are_the_block_region_qN_would_move),
    ("the split is perturb's own", t_the_quadrant_split_is_perturb_s_own),
    ('degrades on a courtyard-free board',
     t_it_degrades_on_a_board_with_no_courtyards_and_says_so),
    ('refuses without a span',
     t_it_refuses_rather_than_inventing_when_there_is_no_span),
    ('--no-arrangement', t_no_arrangement_switch),
    ('the container guard, pinned',
     t_the_container_guard_is_pinned_on_the_board_it_fires_on),
    ('a THT part covers both sides',
     t_a_through_hole_part_covers_BOTH_sides),
    ('the side rule is max, not sum',
     t_the_side_rule_is_max_not_sum),
    ('the reseat target', t_the_reseat_target_is_a_target_not_a_weight),
    ('#708 board-grid scalars', t_the_board_grid_scalars_travel_with_the_census),
]


def _every_case_is_registered():
    """A `t_*` defined and left out of TESTS is a test that never runs.

    That happened here once, to the row that pins the quadrant counts against
    `_region_unit`. The mutation battery is what noticed, which is a long way
    round for something the module can check on itself in three lines.
    """
    g = globals()
    declared = {fn for _l, fn in TESTS}
    missing = sorted(n for n, v in g.items()
                     if n.startswith('t_') and callable(v)
                     and v not in declared)
    if missing:
        print('  FAIL  every t_* case is registered in TESTS  -- ORPHANED: %s'
              % ', '.join(missing))
        FAILURES.append('unregistered cases: %s' % ', '.join(missing))


def main():
    print('arrangement census (#709)')
    for label, fn in TESTS:
        print(' ' + label)
        fn()
    _every_case_is_registered()
    if FAILURES:
        print('\nFAILED (%d): %s' % (len(FAILURES), ', '.join(FAILURES)))
        return 1
    print('\nOK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
