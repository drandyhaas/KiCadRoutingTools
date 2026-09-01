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

sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import check                                     # noqa: E402
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
        # Not "under-part is the biggest" -- measured, that is FALSE on
        # watchy (86 under-part vs 150 cold) and cap_chain (12 vs 134). What
        # IS true is that they are disjoint counts of one universe.
        report('  %s: every bucket is a non-negative count' % name,
               all(isinstance(doc[k], int) and doc[k] >= 0
                   for k in ('windows_demand_in_outline', 'under_part_windows',
                             'warm_unowned_windows', 'cold_windows')),
               str({k: doc[k] for k in ('windows_demand_in_outline',
                                        'under_part_windows',
                                        'warm_unowned_windows',
                                        'cold_windows')}))


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
        # 5e-4, not 1e-6: `area_mm2` is rounded to 3 decimals, so the
        # comparison has to allow the rounding grain or it is red on
        # glasgow_revC / orangecrab / ulx3s at --bin 0.25 for a reason that
        # is not the claim.
        worse = [r for r in doc['cold_regions']
                 if r['area_mm2'] > r['windows'] * doc['bin_mm'] ** 2 + 5e-4]
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
    # Two singletons cannot see `sorted(comp)`; an L can. Discovery order
    # from the seed (0,0) is (0,0),(1,0),(0,1); sorted is (0,0),(0,1),(1,0).
    report('component MEMBERS are sorted, not discovery-ordered',
           CP.flood_regions([(0, 0), (1, 0), (0, 1)])
           == [[(0, 0), (0, 1), (1, 0)]],
           str(CP.flood_regions([(0, 0), (1, 0), (0, 1)])))
    report('  ...and the components themselves are ordered too',
           CP.flood_regions([(5, 5), (0, 0)]) == [[(0, 0)], [(5, 5)]])


def t_largest_band_is_the_maximal_rectangle():
    #  X X X
    #  X X .      the maximal all-cold rectangle is the 3x1 top row
    #  X X X
    #  X X .    -- the solid 2x2 (area 4) beats the 3x1 row (area 3), and
    #              accepting EITHER answer, as the first version did, made the
    #              flagship maximality row assert nothing.
    got = CP.largest_band([(0, 0), (1, 0), (2, 0), (0, 1), (1, 1)])
    report('picks the maximal-AREA rectangle, not the longest run',
           got == (0, 0, 1, 1), str(got))
    #  X X X
    #  . . .    -- now the row IS maximal.
    report('  ...and the row when the row is maximal',
           CP.largest_band([(0, 0), (1, 0), (2, 0)]) == (0, 0, 2, 0),
           str(CP.largest_band([(0, 0), (1, 0), (2, 0)])))
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


def t_a_track_crossing_a_window_is_not_cold():
    """`free` is midpoint accounting; a crossing track has no midpoint here.

    congestion_bins charges a segment's whole area to the bin holding its
    MIDPOINT, which is right for a demand/capacity ratio and wrong for "is
    this window empty". Measured before the swept test:
    rp2350_fpga_eensy_prePlane at --bin 1.0 -- the bin the docs' own example
    uses -- reported 159 cold windows of which 27 had a track running through
    them. The census's whole claim about them is "no copper".
    """
    p = os.path.join(ROOT, 'kicad_files',
                     'rp2350_fpga_eensy_prePlane.kicad_pcb')
    if not os.path.isfile(p):
        report('rp2350 fixture present', False, 'missing')
        return
    pcb = parse_kicad_pcb(p)
    report('the fixture is routed, or this proves nothing',
           len(pcb.segments) > 100, '%d segments' % len(pcb.segments))
    for bm in (2.0, 1.0, 0.5):
        doc, _h = CP.pocket_census(parse_kicad_pcb(p), p, bin_mm=bm)
        touched = CP.copper_touched_bins(pcb, doc['bin_mm'])
        # A region BBOX may legitimately span a touched bin (an L-shape wraps
        # around copper); the BAND cannot -- every cell in it is cold.
        worse = [r for r in doc['cold_regions']
                 if any((bx, by) in touched
                        for bx in range(int(round(r['band_rect'][0] / bm)),
                                        int(round(r['band_rect'][2] / bm)))
                        for by in range(int(round(r['band_rect'][1] / bm)),
                                        int(round(r['band_rect'][3] / bm))))]
        report('bin %g: no cold BAND overlaps swept copper' % bm, not worse,
               str(worse[:1]))
    doc, _h = CP.pocket_census(parse_kicad_pcb(p), p, bin_mm=1.0)
    report('and the count really moved (was 159 cold at bin 1.0)',
           doc['cold_windows'] < 100, str(doc['cold_windows']))


def t_a_cold_band_holds_no_part_and_the_report_says_so():
    """The census's headline output used to be a command that lifts nothing.

    A cold window cannot contain a pad centre BY CONSTRUCTION -- a pad's area
    is charged to its bin, so a window holding one is warm, never cold -- so
    `refs_in_rect(band)` is empty for every band this can ever name. The report
    printed `--reseat-region <that band>` as its actionable line, and it
    resolved to an empty scope on every board.
    """
    from placement.utility import refs_in_rect
    for name in ('esp_prog', 'watchy', 'tigard', 'glasgow_revC', 'sonde_u'):
        doc, _hot = census(name)
        rt = doc.get('reseat_target')
        if not rt:
            report('%s: has a reseat target to check' % name, False)
            continue
        pcb = parse_kicad_pcb(board(name))
        got = refs_in_rect(pcb, tuple(rt['zone']))
        report('%s: the named landing site holds NO part' % name,
               got == [], str(got))
        report('  %s: and the record says so rather than implying otherwise'
               % name, rt.get('contains_parts') is False)
        b = doc['outline']['bounds']
        report('  %s: the zone is CLIPPED to the board' % name,
               rt['zone'][0] >= b[0] - 1e-6 and rt['zone'][1] >= b[1] - 1e-6
               and rt['zone'][2] <= b[2] + 1e-6
               and rt['zone'][3] <= b[3] + 1e-6,
               '%s vs bounds %s' % (rt['zone'], b))
        report('  %s: no --reseat-region command is advertised' % name,
               'reseat_region' not in rt)
    doc, hot = census('esp_prog')
    txt = '\n'.join(CP.format_report(
        dict(doc, bin_requested_mm=2.0, _clr=0.25, _trk=0.3), hot, 8))
    report('the printed block calls it a DESTINATION, not a scope',
           'DESTINATION, not a scope' in txt, txt[-400:])
    # Not "the token never appears" -- the prose names the flag in order to
    # warn about it, and a grep for the bare token is satisfied by prose. What
    # must not appear is a RUNNABLE command: a line carrying both the tool and
    # the flag.
    cmd = [ln for ln in txt.splitlines()
           if 'place_seed.py' in ln and '--reseat-region' in ln]
    report('  ...and prints no runnable --reseat-region command for it',
           not cmd, str(cmd[:1]))
    report('  ...while still naming the flag to warn about it',
           '--reseat-region' in txt)


def t_unreadable_part_geometry_is_disclosed_not_absorbed():
    """"I could not measure the parts" must not read as "there are none".

    With graded_parts_from_file raising, esp_prog reported 81 cold windows
    instead of 29 and a 260mm2 "empty pocket" on top of U1 and USB1 -- and
    said nothing, because the provenance line only prints when there IS an
    arrangement, and there is none without parts.
    """
    from placement import legality as leg
    real = leg.graded_parts_from_file
    p = board('esp_prog')
    try:
        leg.graded_parts_from_file = (
            lambda *a, **k: (_ for _ in ()).throw(RuntimeError('no parts')))
        doc, hot = CP.pocket_census(parse_kicad_pcb(p), p)
    finally:
        leg.graded_parts_from_file = real
    report('the failure is recorded with its exception',
           'no parts' in (doc['parts'].get('grading_error') or ''),
           str(doc['parts'].get('grading_error')))
    txt = '\n'.join(CP.format_report(
        dict(doc, bin_requested_mm=2.0, _clr=0.25, _trk=0.3), hot, 8))
    report('  ...and the REPORT says so, loudly and unconditionally',
           'PART GEOMETRY UNREADABLE' in txt, txt[:400])
    report('  ...and tells the reader not to act on the pockets',
           'Do not act on them' in txt)
    clean, _h = census('esp_prog')
    report('  ...while a healthy run records no error',
           clean['parts'].get('grading_error') is None)


def t_a_non_finite_bin_is_refused_at_the_cli():
    """`--bin inf` wrote a bare `Infinity` into the --json document.

    congestion_bins' max(0.25, bin) passes inf straight through, the window
    rectangles serialise as Infinity/NaN, and strict parsers refuse the file --
    which is the exact failure the `ratio` comment and test_run23_pockets
    exist to prevent, arriving through a different door.
    """
    b = board('esp_prog')
    tool = os.path.join(ROOT, 'py_tools', 'check_pockets.py')
    for bad, why in (('inf', 'finite'), ('nan', 'finite'),
                     ('0', 'positive'), ('-1', 'positive')):
        check([sys.executable, '-X', 'utf8', tool, b, '--bin', bad],
              code=2, refuse=why, allow=('error: argument',))
    report('a non-finite or non-positive --bin is refused by its reason', True)
    report('  ...and a legitimate sub-floor --bin still WORKS',
           CP.pocket_census(parse_kicad_pcb(b), b, bin_mm=0.1)[0]['bin_mm']
           == 0.25)
    check([sys.executable, '-X', 'utf8', tool, b, '--outline-samples', '999'],
          code=2, refuse='quadratic', allow=('error: argument',))
    report('  ...and an unbounded --outline-samples is refused too', True)


def t_no_cold_suppresses_only_the_cold_half():
    """--no-cold used to return before the outline sweep, taking the
    arrangement census, the parts provenance and the reseat target with it --
    while the docs and the argparse help both describe --no-arrangement as the
    separate switch for those."""
    doc, _hot = census('esp_prog', cold=False)
    report('the cold numbers are gone',
           doc['cold_windows'] is None and doc['cold_regions'] == []
           and doc['reseat_target'] is None)
    report('  ...but the ARRANGEMENT survives',
           doc['arrangement'] is not None and bool(doc['arrangement']['sides']),
           str(doc['arrangement'] is None))
    # `graded` was 20 until #726: esp_prog carries TWO footprint blocks named
    # `Ref*` and the parser kept only the second, so this board reported one
    # part fewer than the file has. 21 is the block count.
    report('  ...and so do the outline sweep and the parts provenance',
           bool(doc['outline']) and doc['windows_in_outline'] == 128
           and doc['parts']['graded'] == 21,
           'windows_in_outline=%s graded=%s'
           % (doc.get('windows_in_outline'), doc.get('parts', {}).get('graded')))
    live = census('esp_prog')[0]
    report('  ...and they agree with the full run',
           doc['arrangement'] == live['arrangement'])


def t_the_lattice_survives_inexact_float_division():
    """`math.ceil(21.0 / 0.7)` is 31: the quotient is 30.000000000000004.

    The same defect floor/ceil replaced, moved into the division. Measured, it
    invented 59 off-board windows on a 20x20mm rectangular board at --bin 0.7.
    """
    got = len(CP.lattice_windows((1.0, 1.0, 21.0, 21.0), 0.7))
    report('a 20mm span at bin 0.7 tiles in exactly 29x29',
           got == 29 * 29, '%d windows' % got)
    for name in ('qfn_diffpair_escape', 'qfn_interior_pads'):
        p = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
        if not os.path.isfile(p):
            continue
        for bm in (0.5, 0.7, 0.3):
            doc, _h = CP.pocket_census(parse_kicad_pcb(p), p, bin_mm=bm)
            report('%s bin %g: no phantom off-board window on a rectangle'
                   % (name, bm), doc['windows_offboard'] == 0,
                   str(doc['windows_offboard']))


def t_the_swept_test_is_a_superset_of_the_free_area_rule():
    """Why the cold predicate keeps two copper terms, and why one is redundant.

    `congestion_bins` charges a segment to its MIDPOINT bin and a pad to its
    CENTRE bin; the swept stamp covers a segment's whole path and a pad's whole
    bbox, so swept should be a strict superset of "free-area says occupied".
    Measured here across boards and bin sizes: zero counterexamples.

    That is worth pinning rather than assuming, for two reasons. It is what
    makes a mutation dropping the `free` arm an EQUIVALENT MUTANT rather than a
    hole in the battery -- and it is the assumption under which keeping that
    arm is a cheap cross-check on the newer, more intricate swept code instead
    of dead weight. The day the relation stops holding, one of the two terms is
    wrong, and this row says which direction.
    """
    from congestion_field import congestion_bins
    from net_queries import expand_net_patterns
    total = 0
    for name in ('esp_prog', 'routed_output', 'rp2350_fpga_eensy_prePlane',
                 'tigard', 'watchy'):
        p = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
        if not os.path.isfile(p):
            continue
        for bm in (2.0, 1.0):
            pcb = parse_kicad_pcb(p)
            layers = len(pcb.board_info.copper_layers or ['F.Cu'])
            names = set(expand_net_patterns(pcb, ['*']))
            ids = [n for n, v in pcb.nets.items() if v.name in names]
            bins, _t, bm2 = congestion_bins(pcb, ids, layers, bm)
            cap = bm2 * bm2 * layers
            touched = CP.copper_touched_bins(pcb, bm2)
            occupied = [k for k, (free, _o) in bins.items()
                        if free < cap - 1e-9]
            stray = [k for k in occupied if k not in touched]
            total += len(occupied)
            report('%s bin %g: every free-area-occupied bin is swept-touched'
                   % (name, bm2), not stray,
                   '%d of %d stray, e.g. %s' % (len(stray), len(occupied),
                                                stray[:2]))
    report('and the check was not vacuous -- there were bins to check',
           total > 500, '%d occupied bins examined' % total)


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
    ('swept copper subsumes the free-area rule',
     t_the_swept_test_is_a_superset_of_the_free_area_rule),
    ('a crossing track is not cold',
     t_a_track_crossing_a_window_is_not_cold),
    ('a cold band holds no part',
     t_a_cold_band_holds_no_part_and_the_report_says_so),
    ('unreadable parts are disclosed',
     t_unreadable_part_geometry_is_disclosed_not_absorbed),
    ('a non-finite --bin is refused',
     t_a_non_finite_bin_is_refused_at_the_cli),
    ('--no-cold keeps the arrangement',
     t_no_cold_suppresses_only_the_cold_half),
    ('inexact float division',
     t_the_lattice_survives_inexact_float_division),
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
