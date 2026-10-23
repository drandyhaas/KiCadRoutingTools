#!/usr/bin/env python3
"""Below blocking 0, open connections outrank vias and copper (#1340).

`blocking` counts an unrouted net as 1 whatever its size, and the skill (and
the GUI's candidates table, and the seed judge) broke a `blocking` tie by
vias, then copper. On StickHub, board_score gave a board missing the whole
17-pad +3.3V rail BLOCKING=1 / 117 vias / 704.04 mm and a board with one
stranded pad BLOCKING=1 / 132 vias / 773.42 mm: the rule kept the rail-less
board, its missing copper reading as quality. board_score published no pad
count for an unrouted net that could break the tie; check_connected prints
it and board_score dropped it.

Checks:
  1. board_score keeps each unrouted net's pad count and publishes
     `joins_needed` = (pads - 1) per unrouted net + the broken joins;
  2. quick_score publishes the same over the nets without a zone;
  3. `quick_score.judge_key` and the GUI's `candidate_key` keep the one-pad
     board over the rail-less one, and a score written before
     `joins_needed` existed still ranks (0 joins at blocking 0, last above);
  4. end to end, board_score on a real unrouted board publishes the pads and
     a `joins_needed` that adds up.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('', 'py_router', 'py_tools', 'py_placer', 'kicad_routing_plugin'):
    _p = os.path.join(ROOT, _d) if _d else ROOT
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.path.insert(0, HERE)

import board_score as bs                                      # noqa: E402
import placement_run                                          # noqa: E402
import quick_score                                            # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

# check_connected's report: the 17-pad rail unrouted, a 2-pad net unrouted,
# a signal split in two, and a poured GND split in three
REPORT = """Found 1330 segments, 108 vias, 210 pads, 1 zones
FOUND 4 ISSUES:
  Unrouted nets (2):
    +3.3V (17 pads)
    /SCL (2 pads)

  Connectivity issues (2):

  /CS (net 2):
    Segments: 9, Vias: 0, Pads: 3
    Disconnected components: 2
    Disconnected pads:
      (120.49, 78.50) on F.Cu [R1]

  GND (net 4):
    Segments: 618, Vias: 38, Pads: 55
    Disconnected components: 3
    Disconnected pads:
      (137.72, 66.05) on F.Cu [SW1]
"""


def conn(report=REPORT, poured=('GND',)):
    real_run, real_poured = bs.run_tool, bs.poured_net_names
    bs.run_tool = lambda root, tool, board, *a, **k: (1, report)
    bs.poured_net_names = lambda board: set(poured)
    try:
        return bs.score_connectivity('.', 'board.kicad_pcb')
    finally:
        bs.run_tool, bs.poured_net_names = real_run, real_poured


def test_board_score_counts_the_joins():
    c = conn()
    assert c['unrouted_pads'] == {'+3.3V': 17, '/SCL': 2}, c['unrouted_pads']
    assert c['unrouted'] == 2 and c['broken'] == 3, (c['unrouted'], c['broken'])
    assert c['joins_needed'] == 16 + 1 + 1 + 2, c['joins_needed']
    clean = conn('ALL NETS FULLY CONNECTED!\n')
    assert clean['joins_needed'] == 0 and clean['unrouted_pads'] == {}, clean
    print('  PASS: board_score keeps each unrouted net\'s pads; joins_needed 20 '
          '(16 + 1 unrouted, 1 + 2 broken)')


def test_quick_score_counts_the_unpoured_joins():
    c, real = conn(), bs.score_connectivity
    bs.score_connectivity = lambda root, board: c
    try:
        blk, by, poured, joins = quick_score.connectivity('fast.kicad_pcb')
    finally:
        bs.score_connectivity = real
    assert blk == 3 and joins == 16 + 1 + 1, (blk, joins)
    assert poured['broken'] == 2, poured
    print('  PASS: quick_score joins_needed 18 over the unpoured nets (GND apart)')


# the StickHub pair, as #1340 measured it
RAIL = {'blocking': 1, 'joins_needed': 16, 'blocking_by': {'unrouted': 1, 'broken': 0},
        'quality': {'vias': 117, 'copper_mm': 704.04, 'segments': 1100}}
ONE_PAD = {'blocking': 1, 'joins_needed': 1, 'blocking_by': {'unrouted': 0, 'broken': 1},
           'quality': {'vias': 132, 'copper_mm': 773.42, 'segments': 1250}}


def gui_order(docs):
    with tempfile.TemporaryDirectory() as td:
        rows = []
        for name, d in docs.items():
            p = os.path.join(td, f'{name}.score.json')
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(d, fh)
            sc = placement_run._read_score(p)
            assert sc is not None, (name, d)
            rows.append(dict(zip(('blocking', 'vias', 'copper_mm', 'segments', 'floorplan',
                                  'joins'), sc), name=name))
    return [r['name'] for r in sorted(rows, key=placement_run.candidate_key)]


def test_rankings_keep_the_one_pad_board():
    docs = {'rail_missing': RAIL, 'one_pad': ONE_PAD}
    by_judge = sorted(docs, key=lambda n: quick_score.judge_key(docs[n]))
    assert by_judge == ['one_pad', 'rail_missing'], by_judge
    assert gui_order(docs) == by_judge, gui_order(docs)
    # a score written before joins_needed: 0 at blocking 0, so a finished old
    # score still beats an unfinished new one on blocking; above 0 it sorts
    # after a new score of equal blocking, and two old ones fall to vias
    old_rail = {k: v for k, v in RAIL.items() if k != 'joins_needed'}
    old_pad = {k: v for k, v in ONE_PAD.items() if k != 'joins_needed'}
    old_done = {'blocking': 0, 'quality': {'vias': 200, 'copper_mm': 900.0, 'segments': 1}}
    mix = {'old_rail': old_rail, 'old_pad': old_pad, 'one_pad': ONE_PAD, 'old_done': old_done}
    by_judge = sorted(mix, key=lambda n: quick_score.judge_key(mix[n]))
    assert by_judge == ['old_done', 'one_pad', 'old_rail', 'old_pad'], by_judge
    assert gui_order(mix) == by_judge, gui_order(mix)
    print('  PASS: judge_key and the GUI keep the one-pad board (1 join, 132 vias) '
          'over the rail-less one (16 joins, 117 vias)')


def test_board_score_end_to_end():
    evidence(BOARD, 'fixture board')
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'score.json')
        check([sys.executable, '-X', 'utf8', tool('board_score.py'), BOARD,
               '--json', out, '--quiet'], refuse='joins_needed=', code=4)
        doc = json.load(open(evidence(out, 'board_score JSON'), encoding='utf-8'))
    pads = doc['components']['unrouted']['pads']
    assert len(pads) == doc['blocking_by']['unrouted'], (len(pads), doc['blocking_by'])
    want = sum(n - 1 for n in pads.values()) + doc['blocking_by']['broken']
    assert doc['joins_needed'] == want, (doc['joins_needed'], want)
    assert doc['joins_needed'] > doc['blocking_by']['unrouted'], doc['joins_needed']
    print(f'  PASS: board_score on a real board: {len(pads)} unrouted nets, '
          f'joins_needed {doc["joins_needed"]}')


def main():
    test_board_score_counts_the_joins()
    test_quick_score_counts_the_unpoured_joins()
    test_rankings_keep_the_one_pad_board()
    test_board_score_end_to_end()
    print('test_1340_joins_tiebreak: all passed')


if __name__ == '__main__':
    main()
