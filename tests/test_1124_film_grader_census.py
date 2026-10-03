#!/usr/bin/env python3
"""#1124: the film's LEGALITY panel plots render's grader census, not the box one.

#1119 made `render_placement`'s pad-clearance list the grader's census (the
checklist, `--gate`, the overlay, the caption). The film kept plotting
`metrics.pad_conflict_pairs` -- the quench's bounding-box currency -- against
`metrics.locked_contact_pairs` as its floor, while `docs/route-animation.md`
credited the panel to `render_placement --json-out`. On glasgow_revC the film
said 10 pairs where render's checklist names 1; every glasgow board counts
six FID/MK box contacts as locked, and wherever those six were every pair
left (run 32's placed boards and every board routed from them) it drew
"floor 6 = locked parts" for phantoms the grader confirms none of.

All three LEGALITY series now come from render's checklist: the gating
off-outline parts, the grader's pad-clearance pairs (floor: those with a
KiCad-locked member), and the courtyard census area.

Run: python3 -X utf8 tests/test_1124_film_grader_census.py [case ...]
"""
import json
import os
import sys
import tempfile
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_placer', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import movie_placement as MP  # noqa: E402

RUN_ALL_TIMEOUT = 900

KF = os.path.join(ROOT, 'kicad_files')
REVC = os.path.join(KF, 'glasgow_revC.kicad_pcb')
RP2350 = os.path.join(KF, 'rp2350_fpga_eensy_prePlane.kicad_pcb')
RENDER = os.path.join(ROOT, 'py_tools', 'render_placement.py')


def _checklist(board, td):
    j = os.path.join(td, os.path.basename(board) + '.json')
    run_utils.check([sys.executable, '-X', 'utf8', RENDER, board, '-o',
                     os.path.join(td, 'x.png'), '--json-out', j, '--size',
                     '400', '--no-describe', '--quiet'], accept=True)
    with open(run_utils.evidence(j), encoding='utf-8') as fh:
        doc = json.load(fh)
    return doc['checklist'], doc['metrics']


def test_the_film_reads_renders_checklist():
    """rp2350 separates both channels: render ranks U8 as pad copper off the
    outline but gates none, and the box census reads 5 pairs where the grader
    reads 2."""
    with tempfile.TemporaryDirectory() as td:
        for board in (REVC, RP2350):
            ck, metrics = _checklist(board, td)
            locked = set(ck['c_locked_refs'])
            want = {'off_outline': len(ck['a_off_outline']['pad_copper_gating']),
                    'conflict_pairs': len(ck['b_pad_clearance_pairs']),
                    'locked_pairs': sum(1 for a, b, *_r in
                                        ck['b_pad_clearance_pairs']
                                        if a in locked or b in locked),
                    'overlap_mm2': ck['b_courtyard_overlap_mm2']}
            got = MP.measure_board(board, cache={})
            assert {k: got[k] for k in want} == want, (board, got, want)
            print("  %s: %r (box metric pairs %s)"
                  % (os.path.basename(board), want,
                     metrics.get('pad_conflict_pairs')))
    assert want['conflict_pairs'] < metrics['pad_conflict_pairs'], \
        'rp2350 no longer separates the grader from the box census'


def test_no_phantom_floor_on_glasgow():
    import render_placement as RP
    from kicad_parser import parse_kicad_pcb
    model = RP.PlacementModel(parse_kicad_pcb(REVC), REVC, exact=True,
                              quench_kwargs={'clearance': None,
                                             'ignore_net_ids': None})
    box_floor = model.metrics['locked_contact_pairs']
    got = MP.measure_board(REVC, cache={})
    assert box_floor == 6 and got['locked_pairs'] == 0, (box_floor, got)
    print("  glasgow_revC: box floor %d, the grader's 0" % box_floor)


def test_a_box_phantom_draws_no_floor():
    """Two KiCad-locked parts whose round pads' BOXES sit inside the pads'
    0.8 mm clearance while the pads themselves keep 1.0 mm: the quench counts
    a locked contact, the grader no pair -- and no floor is drawn."""
    pad = ('    (pad "1" smd circle (at 0 0) (size 1 1) (layers "F.Cu")'
           ' (clearance 0.8))\n')
    fps = ''.join(
        '  (footprint "t:%s" (locked yes) (layer "F.Cu") (at %s %s)\n'
        '    (property "Reference" "%s" (at 0 0) (layer "F.SilkS"))\n%s  )\n'
        % (ref, x, y, ref, pad)
        for ref, x, y in (('A', 10, 10), ('B', 11.414214, 11.414214)))
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        with open(b, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                     '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
                     '(44 "Edge.Cuts" user))\n  (net 0 "")\n'
                     '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1)'
                     ' (type default)) (layer "Edge.Cuts"))\n' + fps + ')\n')
        import render_placement as RP
        from kicad_parser import parse_kicad_pcb
        model = RP.PlacementModel(parse_kicad_pcb(b), b, exact=True,
                                  quench_kwargs={'clearance': None,
                                                 'ignore_net_ids': None})
        assert model.metrics.get('locked_contact_pairs'), (
            'the fixture no longer makes a box phantom: %r' % model.metrics)
        m = MP.measure_board(b, cache={})
    assert (m['conflict_pairs'], m['locked_pairs']) == (0, 0), m
    beat = MP.Beat('b', 'b', 0, m['off_outline'], m['conflict_pairs'],
                   m['overlap_mm2'], m['crossings'], m['hpwl'],
                   m['locked_pairs'], None, '', None)
    track = MP.PlacementTrack((beat, beat), None, '', (), (), None, '')
    assert MP._floor(track) is None
    print("  box locked contacts %s, grader pairs 0, no floor"
          % model.metrics['locked_contact_pairs'])


def test_a_locked_member_on_either_side_counts():
    class _M:
        no_outline = False

        class state:
            legality_ctx = object()
    fnd = {'pad_conflict_pairs_refs': [['A', 'B', 0.1], ['C', 'D', 0.2]],
           'oob_refs_pad_copper_gating': [], 'courtyard_overlap_mm2': 1.0,
           'courtyard_census_error': None}
    for locked, want in ((['B'], 1), (['C'], 1), (['A', 'D'], 2), ([], 0)):
        got = MP._legality_census(_M, dict(fnd, locked_refs=locked))
        assert got['locked_pairs'] == want, (locked, got)
    print("  either member locked counts; none counts none")


def test_no_outline_is_unmeasured():
    """A board with no Edge.Cuts has nothing to be off: off-outline is
    None, not 0, while the pair census still runs."""
    pad = ('    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") '
           '(net 1 "N1"))\n')
    fps = ''.join(
        '  (footprint "t:%s" (layer "F.Cu") (at %s 10)\n'
        '    (property "Reference" "%s" (at 0 0) (layer "F.SilkS"))\n%s  )\n'
        % (ref, x, ref, pad) for ref, x in (('A', 10), ('B', 20)))
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        with open(b, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                     '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
                     '(44 "Edge.Cuts" user))\n  (net 0 "") (net 1 "N1")\n'
                     + fps + ')\n')
        m = MP.measure_board(b, cache={})
    assert m['off_outline'] is None, m
    assert m['conflict_pairs'] == 0, m
    print("  no outline: off-outline None, pairs %s" % m['conflict_pairs'])


def test_the_floor_needs_every_pair_locked():
    """The floor is drawn only when every pair left has a locked member:
    one locked pair among three is not a floor."""
    def track(conflict, locked):
        beat = MP.Beat('b', 'b', 0, 0, conflict, 1.0, 0, 0.0, locked, None,
                       '', None)
        return MP.PlacementTrack((beat, beat), None, '', (), (), None, '')
    assert MP._floor(track(3, 1)) is None
    assert MP._floor(track(2, 2)) == 2
    assert MP._floor(track(0, 0)) is None
    print("  3 pairs / 1 locked: no floor; 2 / 2: floor 2")


def test_not_measured_is_none():
    """Without a legality context the pad lists sit at their empty defaults:
    the film reports them unmeasured, and the courtyard census -- which does
    not need one -- is still read."""
    import render_placement as RP
    real = RP.PlacementModel

    def no_ctx(pcb, path, **kw):
        q = dict(kw.get('quench_kwargs') or {}, pad_legality=False)
        return real(pcb, path, **dict(kw, quench_kwargs=q))
    with patch.object(RP, 'PlacementModel', no_ctx):
        m = MP.measure_board(REVC, cache={})
    assert m['off_outline'] is None and m['conflict_pairs'] is None \
        and m['locked_pairs'] is None, m
    assert m['overlap_mm2'] is not None, m
    print("  no legality context: pairs/off-outline/floor None, overlap %s"
          % m['overlap_mm2'])


def test_a_census_error_is_none():
    from placement import legality

    def boom(*a, **k):
        raise RuntimeError('census unavailable')
    with patch.object(legality, 'grade_body_overlap', boom):
        m = MP.measure_board(REVC, cache={})
    assert m.get('overlap_mm2') is None, m
    assert m.get('conflict_pairs') == 1, m
    print("  courtyard census raised: overlap None, pairs still %s"
          % m['conflict_pairs'])


def test_run32_draws_no_phantom_floor():
    """With run 32's boards (`KRT_RUN32_DIR`, or this checkout's wk/run32):
    placed_v3 ended on a box count of 6 = 6 locked, so the film drew
    "floor 6"; the grader's census leaves nothing to floor."""
    run32 = os.environ.get('KRT_RUN32_DIR') or os.path.join(ROOT, 'wk',
                                                            'run32')
    boards = [os.path.join(run32, b + '.kicad_pcb')
              for b in ('placed_v2', 'placed_v3')]
    if not all(os.path.isfile(b) for b in boards):
        print("  SKIP: run-32 boards absent (set KRT_RUN32_DIR)")
        return
    steps = [(os.path.splitext(os.path.basename(b))[0], b, None)
             for b in boards]
    t, why = MP.build_track(steps, [], ledger=os.path.join(run32,
                                                           'ledger.jsonl'))
    assert t is not None, why
    assert [bt.conflict_pairs for bt in t.beats] == [0, 0], t.beats
    assert MP._floor(t) is None
    print("  run 32: conflict pairs 0 / 0, no floor drawn")


TESTS = [
    test_the_film_reads_renders_checklist,
    test_no_phantom_floor_on_glasgow,
    test_a_box_phantom_draws_no_floor,
    test_a_locked_member_on_either_side_counts,
    test_no_outline_is_unmeasured,
    test_the_floor_needs_every_pair_locked,
    test_not_measured_is_none,
    test_a_census_error_is_none,
    test_run32_draws_no_phantom_floor,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
