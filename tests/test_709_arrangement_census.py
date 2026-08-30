"""The arrangement census: where the mass sits against where the demand sits.

The pockets census put every demand-3 window at one end of run 23's board
while the part mass sat at the other, and no instrument said so. This is the
join, plus the caution #709 measured and that is easy to get backwards: the
centroid must be weighted by COURTYARD AREA, not by part count. The count form
is the intuitive one and it points the wrong way -- on esp_prog it reads 13.7%
of span where the area form reads 1.3%.

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
    report('esp_prog: the area form is ~1.3% of span',
           abs(area_x - 0.013) < 0.004, 'got %.4f' % area_x)
    report('esp_prog: the count control is ~13.7% of span',
           abs(count_x - 0.137) < 0.008, 'got %.4f' % count_x)
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
    report('quadrant part counts total the footprints on the board',
           sum(q['parts'] for q in quads.values())
           == len(parse_kicad_pcb(board('esp_prog')).footprints),
           str(sum(q['parts'] for q in quads.values())))
    report('cold windows are attributed to quadrants too',
           sum(q['cold_windows'] for q in quads.values())
           == doc['cold_windows'],
           '%d vs %d' % (sum(q['cold_windows'] for q in quads.values()),
                         doc['cold_windows']))


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
    report('its --reseat-region argument IS the band rect',
           rt['reseat_region'] == rt['band_rect'], str(rt['reseat_region']))
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


TESTS = [
    ('area weighting vs the count control',
     t_area_weighting_and_the_count_control_disagree),
    ('the two forms across the corpus', t_the_two_forms_differ_across_the_corpus),
    ('synthetic parts excluded', t_synthetic_parts_are_excluded_and_counted),
    ('sides stay separate', t_sides_are_separate_never_summed),
    ('quadrants join mass to demand', t_quadrants_join_mass_to_demand),
    ("the split is perturb's own", t_the_quadrant_split_is_perturb_s_own),
    ('degrades on a courtyard-free board',
     t_it_degrades_on_a_board_with_no_courtyards_and_says_so),
    ('refuses without a span',
     t_it_refuses_rather_than_inventing_when_there_is_no_span),
    ('--no-arrangement', t_no_arrangement_switch),
    ('the reseat target', t_the_reseat_target_is_a_target_not_a_weight),
]


def main():
    print('arrangement census (#709)')
    for label, fn in TESTS:
        print(' ' + label)
        fn()
    if FAILURES:
        print('\nFAILED (%d): %s' % (len(FAILURES), ', '.join(FAILURES)))
        return 1
    print('\nOK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
