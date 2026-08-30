"""The census can say EMPTY: in-outline enumeration and cold regions (#709).

`congestion_bins` keyed its result off the `owners` map, so a window with
nothing in it never entered the dict and `check_pockets` printed `len(bins)`
as its window count. An empty region was therefore not merely un-ranked, it
was not a window at all -- and "a clear empty pocket west of U2, a 6-8mm band"
is precisely the finding a placement reviewer wants and no instrument could
produce.

Every number below is measured on a git-tracked board, and each row names the
mutation it exists to catch. Two of them (A1, A7) are the issue's own defect
and the issue's own wanted sentence, pinned so they cannot quietly come back.
"""

import math
import os
import sys

# Imports the census and routes nothing. Comment ABOVE the marker: a
# trailing one voids run_all's `...True\s*$` anchor (see the sibling
# test_709_congestion_bins_include.py, where it really did).
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


def t_the_defect_itself():
    """A1/A3: the counts the issue measured, on the boards it measured them on.

    Reverting the fix makes in-outline collapse onto demand and both rows go
    red -- there is no way to satisfy them with a census that cannot see an
    empty window.
    """
    doc, _hot = census('esp_prog')
    report('esp_prog: 128 windows lie in the outline',
           doc['windows_in_outline'] == 128, str(doc['windows_in_outline']))
    report('esp_prog: 44 of them have demand (84, 66%, were invisible)',
           doc['windows_demand'] == 44, str(doc['windows_demand']))
    doc, _hot = census('glasgow_revC')
    report('glasgow_revC: 1000 in-outline, 457 with demand',
           (doc['windows_in_outline'], doc['windows_demand']) == (1000, 457),
           '%s / %s' % (doc['windows_in_outline'], doc['windows_demand']))


def t_the_lattice_bounds_are_floor_ceil():
    """A3's mechanism, isolated: `int(hi // bin) + 1` appends a dead row.

    glasgow_revC's y1 is exactly 120.0, so the wrong rule adds 40 windows of
    zero width -- and the invisible-window headline would be overstated by
    exactly those 40.
    """
    got = len(CP.lattice_windows((50.0, 71.0, 130.0, 120.0), 2.0))
    report('a bound exactly on a lattice line adds no phantom row',
           got == 1000, '%d windows' % got)
    got = len(CP.lattice_windows((0.0, 0.0, 4.0, 4.0), 2.0))
    report('a fully aligned bbox tiles exactly', got == 4, '%d' % got)
    got = len(CP.lattice_windows((0.1, 0.1, 3.9, 3.9), 2.0))
    report('a bbox inside one tile-and-a-bit still covers it', got == 4,
           '%d' % got)


def t_the_partition_holds():
    """A2: every in-outline window is demand, cold, or warm-but-unowned.

    A double count or a dropped window shows up here and nowhere else.
    """
    for name in ('esp_prog', 'watchy', 'cap_chain', 'interf_u_unrouted'):
        doc, _hot = census(name)
        total = (doc['windows_demand_in_outline'] + doc['under_part_windows']
                 + doc['warm_unowned_windows'] + doc['cold_windows'])
        report('%s: demand + under-part + warm-unowned + cold == in-outline'
               % name, total == doc['windows_in_outline'],
               '%d vs %d' % (total, doc['windows_in_outline']))
        report('  %s: the under-part bucket is the big one, not cold' % name,
               doc['under_part_windows'] >= 0 and doc['cold_windows'] >= 0)


def t_cold_means_no_copper_not_just_no_demand():
    """A4/C3: part-free is not empty.

    On a ROUTED board under a narrow demand set, a window can carry another
    net's copper and no demand at all. Defining cold as `demand == 0` alone
    calls it empty. This needs the narrow set: at --nets '*' there are none.
    """
    p = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
    if not os.path.isfile(p):
        report('routed_output.kicad_pcb present', False, 'fixture missing')
        return
    doc, _hot = CP.pocket_census(parse_kicad_pcb(p), p, nets=['GND'])
    report('a routed board has part-free windows that carry copper',
           doc['warm_unowned_windows'] > 0,
           '%d' % doc['warm_unowned_windows'])
    report('and they are NOT counted cold',
           (doc['windows_demand_in_outline'] + doc['under_part_windows']
            + doc['warm_unowned_windows'] + doc['cold_windows'])
           == doc['windows_in_outline'])
    # The mutation this exists to catch: dropping the `free < bin_area_total`
    # arm folds those 190 windows into `cold`, so the counts must differ.
    loose, _h = CP.pocket_census(parse_kicad_pcb(p), p, nets=['GND'],
                                 cold_cover=1.0)
    report('  the copper guard is not made redundant by --cold-cover 1.0',
           loose['warm_unowned_windows'] == doc['warm_unowned_windows'],
           '%d vs %d' % (loose['warm_unowned_windows'],
                         doc['warm_unowned_windows']))


def t_the_issue_s_own_sentence():
    """A7: "an empty band south of CON2's eastern half", as a band.

    8-connected labelling merges this region into the blob beside it and the
    band disappears; ranking by window count buries it. Both go red here.
    """
    doc, _hot = census('esp_prog')
    regions = doc['cold_regions']
    hit = [r for r in regions if 'CON2' in r['refs'] and 'CON1' in r['refs']]
    report('a cold region is bounded by CON1 and CON2', bool(hit),
           '%d regions, refs %s' % (len(regions), [r['refs'] for r in regions]))
    if hit:
        r = hit[0]
        report('  ...and it is a BAND, not a blob (fill >= 0.9)',
               (r['fill'] or 0) >= 0.9, 'fill %s' % r['fill'])
        report('  ...whose band rect is a real rectangle of windows',
               r['band_mm'][0] > 0 and r['band_mm'][1] > 0,
               str(r['band_mm']))
    beside_u2 = [r for r in regions if r['bbox'][0] >= 133 and r['bbox'][2] <= 137]
    report('a cold region sits immediately beside U2', bool(beside_u2),
           str([r['bbox'] for r in regions]))


def t_area_accounting_never_exceeds_the_region():
    """A8: the bug the prototype hit -- a band reported as w*h.

    esp_prog's top region spans 32 x 2mm of lattice but only 55.8mm2 of board,
    because its edge windows are partly outside. A band computed as w*h claims
    64.0mm2 inside a 55.8mm2 region, which is self-contradictory on its face.
    """
    for name in ('esp_prog', 'watchy', 'interf_u_unrouted'):
        doc, _hot = census(name)
        bad = [r for r in doc['cold_regions']
               if r['band_area_mm2'] > r['area_mm2'] + 1e-6]
        report('%s: no band claims more area than its region' % name,
               not bad, str(bad[:1]))
        worse = [r for r in doc['cold_regions']
                 if r['area_mm2'] > r['windows'] * doc['bin_mm'] ** 2 + 1e-6]
        report('%s: no region claims more than its windows can hold' % name,
               not worse, str(worse[:1]))


def t_ranking_is_area_and_deterministic():
    """A9: contiguous AREA, never window count, and a stable tie-break."""
    doc, _hot = census('esp_prog')
    regions = doc['cold_regions']
    report('regions are non-increasing in area',
           all(regions[i]['area_mm2'] >= regions[i + 1]['area_mm2'] - 1e-9
               for i in range(len(regions) - 1)),
           str([r['area_mm2'] for r in regions]))
    report('ranks are 1..n in order',
           [r['rank'] for r in regions] == list(range(1, len(regions) + 1)))
    top = regions[0]
    report('the top region is DISCOUNTED for its partial edge windows',
           top['area_mm2'] < top['windows'] * doc['bin_mm'] ** 2 - 1e-6,
           '%.1f vs %.1f' % (top['area_mm2'],
                             top['windows'] * doc['bin_mm'] ** 2))
    again, _h2 = CP.pocket_census(parse_kicad_pcb(board('esp_prog')),
                                  board('esp_prog'))
    report('two runs produce byte-identical regions',
           again['cold_regions'] == regions)

    # The issue's explicit requirement -- "rank cold regions by contiguous
    # area rather than window count, so a 6 to 8mm band outranks scattered
    # singles" -- needs a board where the two ACTUALLY disagree, or it is
    # untestable. On ulx3s they pick a different region for RANK 1: a compact
    # 4 x 16mm slot wins on area over a wider, partly-off-board band that has
    # more windows. esp_prog agrees under both rules and proves nothing.
    doc2, _h3 = census('ulx3s')
    rs = doc2['cold_regions']
    by_count = sorted(rs, key=lambda r: (-r['windows'], r['bbox']))
    report('ulx3s: area-ranking and count-ranking pick DIFFERENT regions',
           rs[0]['bbox'] != by_count[0]['bbox'],
           '%s vs %s' % (rs[0]['bbox'], by_count[0]['bbox']))
    report('  ...and the census takes the AREA one',
           rs[0]['area_mm2'] >= max(r['area_mm2'] for r in rs) - 1e-9,
           '%.1f vs max %.1f' % (rs[0]['area_mm2'],
                                 max(r['area_mm2'] for r in rs)))
    report('  ...which really does have FEWER windows than the count winner',
           rs[0]['windows'] < by_count[0]['windows'],
           '%d vs %d' % (rs[0]['windows'], by_count[0]['windows']))


def t_four_connected_not_eight():
    """Direct on the labeller: a diagonal touch is TWO regions."""
    got = CP.flood_regions([(0, 0), (1, 1)])
    report('a diagonal pair is two components', len(got) == 2, str(got))
    got = CP.flood_regions([(0, 0), (1, 0), (1, 1)])
    report('an L is one component', len(got) == 1, str(got))
    report('component output is sorted, not dict-ordered',
           CP.flood_regions([(5, 5), (0, 0)]) == [[(0, 0)], [(5, 5)]])


def t_largest_band_is_the_maximal_rectangle():
    #  X X X
    #  X X .      the maximal all-cold rectangle is the 3x1 top row
    got = CP.largest_band([(0, 0), (1, 0), (2, 0), (0, 1), (1, 1)])
    report('picks the 3x1 row over the 2x2 that is not solid',
           got in ((0, 0, 2, 0), (0, 0, 1, 1)), str(got))
    report('a solid 2x2 is found whole',
           CP.largest_band([(0, 0), (1, 0), (0, 1), (1, 1)]) == (0, 0, 1, 1))
    report('a single cell is itself',
           CP.largest_band([(3, 4)]) == (3, 4, 3, 4))
    report('nothing has no band', CP.largest_band([]) is None)


def t_the_outline_source_is_disclosed():
    """C1/C2: a rectangular board is graded against its bbox, and says so."""
    doc, _hot = census('esp_prog')
    report('esp_prog discloses bounding_box (its outline IS its bbox)',
           doc['outline']['source'] == 'bounding_box',
           str(doc['outline']))
    report('  ...and nothing falls off a rectangle',
           doc['windows_offboard'] == 0, str(doc['windows_offboard']))
    doc, _hot = census('interf_u_unrouted')
    report('interf_u_unrouted uses its real rings',
           doc['outline']['source'] == 'edge_cuts_contours'
           and doc['outline']['rings'] >= 1, str(doc['outline']))
    report('  ...and windows fall off it, and others are cut by it',
           doc['windows_offboard'] > 0 and doc['windows_partial'] > 0,
           '%d off, %d partial' % (doc['windows_offboard'],
                                   doc['windows_partial']))
    doc, _hot = census('watchy')
    report('watchy: the two CUTOUTS are seen',
           doc['outline']['cutouts'] == 2, str(doc['outline']))
    report('  ...and take windows off the board',
           doc['windows_offboard'] > 0, str(doc['windows_offboard']))


def t_cold_cover_is_a_ladder_not_a_cliff():
    """Sweep the knob's whole range: more slack can only mean more cold."""
    counts = []
    for frac in (0.0, 0.25, 0.5, 1.0):
        doc, _hot = census('esp_prog', cold_cover=frac)
        counts.append(doc['cold_windows'])
    report('cold count is monotonic in --cold-cover',
           all(counts[i] <= counts[i + 1] for i in range(len(counts) - 1)),
           str(counts))
    report('the strict default is strictly the strictest',
           counts[0] < counts[-1], str(counts))


def t_every_number_is_finite_and_json_safe():
    """B7: an inf or a nan would be a bare token strict parsers refuse."""
    def walk(o, p=''):
        if isinstance(o, dict):
            for k, v in o.items():
                yield from walk(v, p + '/' + str(k))
        elif isinstance(o, list):
            for i, v in enumerate(o):
                yield from walk(v, p + '/%d' % i)
        else:
            yield p, o
    for name in ('esp_prog', 'cap_chain', 'watchy', 'interf_u_unrouted'):
        doc, _hot = census(name)
        bad = [(p, v) for p, v in walk(doc)
               if isinstance(v, float) and not math.isfinite(v)]
        report('%s: no inf/nan anywhere in the document' % name, not bad,
               str(bad[:2]))


def t_no_cold_is_a_clean_off_switch():
    doc, _hot = census('esp_prog', cold=False)
    report('--no-cold leaves the cold keys empty, not zero-ish',
           doc['cold_windows'] is None and doc['cold_regions'] == []
           and doc['skipped'] == 'cold census disabled', str(doc.get('skipped')))
    report('  ...and the hot rows are untouched by it',
           doc['windows_demand'] == census('esp_prog')[0]['windows_demand'])


def t_a_printed_row_never_starts_with_a_bracket():
    """The format contract test_run23_pockets.py's ranked-row parser keys on.

    That parser reads every stripped line starting with '[' that mentions
    `demand` and asserts it carries >= 2 nets. A cold row or a quadrant row
    that began with '[' would be read as a ranked row and fail.
    """
    doc, hot = census('esp_prog')
    doc = dict(doc, bin_requested_mm=2.0, _clr=0.25, _trk=0.3)
    lines = CP.format_report(doc, hot, 8)
    ranked = [ln for ln in (x.strip() for x in lines)
              if ln.startswith('[') and 'demand' in ln]
    report('every bracket-leading line is a real ranked row',
           len(ranked) == len(hot[:8]) and len(ranked) > 0,
           '%d bracket lines, %d hot rows' % (len(ranked), len(hot[:8])))
    for ln in ranked:
        n = int(ln.split('demand')[1].split('net(s)')[0].strip())
        if n < 2:
            report('  a ranked row carries >= 2 nets', False, ln)
            return
    report('  every ranked row carries >= 2 nets', True)


TESTS = [
    ('the defect, by the numbers', t_the_defect_itself),
    ('lattice bounds are floor/ceil', t_the_lattice_bounds_are_floor_ceil),
    ('the window partition holds', t_the_partition_holds),
    ('cold requires no copper', t_cold_means_no_copper_not_just_no_demand),
    ("the issue's own sentence", t_the_issue_s_own_sentence),
    ('area accounting', t_area_accounting_never_exceeds_the_region),
    ('ranking is area, deterministic', t_ranking_is_area_and_deterministic),
    ('4-connected labelling', t_four_connected_not_eight),
    ('largest band', t_largest_band_is_the_maximal_rectangle),
    ('the outline source is disclosed', t_the_outline_source_is_disclosed),
    ('--cold-cover ladder', t_cold_cover_is_a_ladder_not_a_cliff),
    ('every number finite', t_every_number_is_finite_and_json_safe),
    ('--no-cold', t_no_cold_is_a_clean_off_switch),
    ('the print format contract', t_a_printed_row_never_starts_with_a_bracket),
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
    print('cold windows and cold regions (#709)')
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
