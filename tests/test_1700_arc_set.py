#!/usr/bin/env python3
"""#1700: the arc-intact replay set is added beside the corpus and changes nothing
in it.

The ordinary prep rebuilds Edge.Cuts as chained lines, so a corpus replay barely
exercises an arc outline. `prep_set1arc.py` adds a few chains whose input keeps its
arcs. None of this asks KiCad or routes; a throwaway corpus stands in for the real
one:

  1. the set's name is outside every range `--sets` can spell, and `set1`'s run dirs
     do not take its directory in;
  2. every manifest row names a real corpus board, under a short name no other set
     uses, because several tools key a board by its bare name across sets;
  3. staging reads the source's chain and input and writes neither; the input keeps
     its arc with no routing left, and the repointed chain reads it;
  4. a chain that never reads the source's input is refused for that reason, as is
     an input with no arc on its outline;
  5. arcs are counted on Edge.Cuts alone, and a side file is told from a flag value.

    python3 tests/test_1700_arc_set.py
"""
import glob
import json
import os
import sys
import tempfile
from pathlib import Path

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
STRESS_TOOLS = os.path.join(TESTS, 'stress')
sys.path.insert(0, TESTS)
sys.path.insert(0, STRESS_TOOLS)
sys.path.insert(0, os.path.join(STRESS_TOOLS, 'modal_sweep'))

import prep_set1arc as prep  # noqa: E402
import sweep_lib  # noqa: E402
from cloud_replay_sets import expand_sets  # noqa: E402

RUN_ALL_TIMEOUT = 120
EDGE_ARC = '(gr_arc (start 0 0) (mid 10 -3) (end 20 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))\n'
EDGE_CLOSE = ('(gr_line (start 20 0) (end 20 20) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))\n'
              '(gr_line (start 20 20) (end 0 20) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))\n'
              '(gr_line (start 0 20) (end 0 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))\n')
TRACK = '(segment (start 1 1) (end 2 2) (width 0.2) (layer "F.Cu") (net 0))\n'


def board(shapes):
    return ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n' + shapes + ')\n')


def corpus(root, *, outline=EDGE_ARC + EDGE_CLOSE, chain=None):
    """One recorded board `src` in set 900, routed and with its chain."""
    stress = Path(root) / 'stress'
    run = stress / 'runs_set900' / 'src'
    run.mkdir(parents=True)
    (stress / 'boards_set900').mkdir()
    (stress / 'boards_unrouted_set900').mkdir()
    (stress / 'boards_set900' / 'src.kicad_pcb').write_text(board(outline + TRACK))
    (stress / 'boards_set900' / 'src.kicad_pro').write_text('{"routed": 1}\n')
    (stress / 'boards_unrouted_set900' / 'src.kicad_pcb').write_text(board(EDGE_CLOSE))
    (stress / 'boards_unrouted_set900' / 'src.kicad_pro').write_text('{"twin": 1}\n')
    if chain is None:
        chain = (f'# cwd={run}\n'
                 f'python3 -X utf8 /tools/py_router/route.py {stress}/boards_unrouted_set900/src.kicad_pcb '
                 f'step1.kicad_pcb --nets \'*\' --net-clearances side.json --clearance 0.25\n')
    (run / 'redo_commands.sh').write_text(chain)
    (run / 'side.json').write_text('{}\n')
    return stress


ROW = {'source_set': 'set900', 'source_board': 'src', 'short_name': 'src_arc'}


def test_name_stays_out_of_ranges():
    assert prep.SET not in expand_sets('set1-set30'), 'a --sets range picks the set up'
    assert expand_sets(f'{prep.SET},set10-set11') == [prep.SET, 'set10', 'set11']
    with tempfile.TemporaryDirectory() as td:
        for d in ('runs_set1', f'runs_{prep.SET}'):
            (Path(td) / d).mkdir()
        assert [p.name for p in sweep_lib.run_dirs_for(Path(td), 'set1')] == ['runs_set1']
    print('  ok: no range spells the set, and set1 does not take in its run dir')


def test_manifest_rows():
    rows = json.loads(prep.MANIFEST.read_text(encoding='utf-8'))
    assert 4 <= len(rows) <= 6, len(rows)
    others = {}
    for m in glob.glob(os.path.join(STRESS_TOOLS, 'manifest_set*.json')):
        if os.path.basename(m) != prep.MANIFEST.name:
            with open(m, encoding='utf-8') as f:
                others[os.path.basename(m)] = {r.get('short_name') for r in json.load(f)}
    every = set().union(*others.values())
    for r in rows:
        assert r['set'] == prep.SET, r
        assert r['short_name'] not in every, f"{r['short_name']} collides with another set"
        src = others[f"manifest_{r['source_set']}.json"]
        assert r['source_board'] in src, f"{r['source_board']} is not in {r['source_set']}"
        assert r.get('outline'), r
    assert len({r['short_name'] for r in rows}) == len(rows), rows
    print(f'  ok: {len(rows)} rows, each a real corpus board under a name no set uses')


def test_stage():
    with tempfile.TemporaryDirectory() as td:
        stress = corpus(td)
        src_chain = (stress / 'runs_set900' / 'src' / 'redo_commands.sh').read_text()
        src_input = (stress / 'boards_unrouted_set900' / 'src.kicad_pcb').read_text()
        stale = stress / f'results_{prep.SET}' / 'src_arc.json'
        stale.parent.mkdir()
        stale.write_text('{}\n')
        run = prep.stage(ROW, stress)
        assert not stale.exists(), 'a result from before says a recording finished that has not'
        assert (stress / 'runs_set900' / 'src' / 'redo_commands.sh').read_text() == src_chain
        assert (stress / 'boards_unrouted_set900' / 'src.kicad_pcb').read_text() == src_input
        inp = stress / f'boards_unrouted_{prep.SET}' / 'src_arc.kicad_pcb'
        text = inp.read_text()
        assert prep.edge_arc_count(text) == 1, text
        assert '(segment' not in text, text
        assert (inp.with_suffix('.kicad_pro')).read_text() == '{"twin": 1}\n', 'not the chain\'s project'
        assert (stress / f'boards_{prep.SET}' / 'src_arc.kicad_pro').read_text() == '{"routed": 1}\n'
        chain = (run / 'redo_commands.sh').read_text()
        assert f'# cwd={stress}/runs_{prep.SET}/src_arc' in chain, chain
        assert str(inp) in chain and 'set900' not in chain, chain
        assert '/tools/py_router/route.py' in chain, 'the tool path is the replay\'s to remap'
        assert (run / 'side.json').is_file(), 'the side file did not come along'
    print('  ok: staging writes only the new set, keeps the arc, and repoints the chain')


def refused(call, reason):
    try:
        call()
    except SystemExit as ex:
        assert reason in str(ex), str(ex)
        return
    raise AssertionError(f'not refused: expected {reason!r}')


def test_refusals():
    with tempfile.TemporaryDirectory() as td:
        stress = corpus(td, chain='# cwd=/x/runs_set900/src\npython3 /tools/py_router/route.py /elsewhere.kicad_pcb o.kicad_pcb\n')
        refused(lambda: prep.stage(ROW, stress), 'never reads')
    with tempfile.TemporaryDirectory() as td:
        stress = corpus(td, outline=EDGE_CLOSE)
        refused(lambda: prep.stage(ROW, stress), 'no arc on Edge.Cuts')
    with tempfile.TemporaryDirectory() as td:
        refused(lambda: prep.stage(ROW, Path(td)), 'fetch and prep that set first')
    print('  ok: a chain on another input, an arc-less outline and a missing source are refused')


def test_counts():
    copper = EDGE_ARC.replace('Edge.Cuts', 'F.Cu')
    in_text = '(gr_text "(gr_arc Edge.Cuts)" (layer "F.SilkS"))\n'
    circle = '(gr_circle (center 5 5) (end 6 5) (layer "Edge.Cuts"))\n'
    assert prep.edge_arc_count(board(copper + in_text)) == 0
    assert prep.edge_arc_count(board(EDGE_ARC + circle + copper)) == 2
    chain = "# cwd=/x/ignored.json\npython3 route.py a.kicad_pcb b.kicad_pcb --net-clearances n.json --clearance 0.25 --nets +3.3V\n"
    assert prep.side_files(chain) == ['n.json'], prep.side_files(chain)
    print('  ok: arcs on Edge.Cuts alone, and a side file is told from a flag value')


def main():
    test_name_stays_out_of_ranges()
    test_manifest_rows()
    test_stage()
    test_refusals()
    test_counts()
    print('test_1700_arc_set: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
