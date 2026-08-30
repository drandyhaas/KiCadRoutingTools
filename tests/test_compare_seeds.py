#!/usr/bin/env python3
"""compare_seeds.py: the seed-axis comparator, one command instead of a
hand-rolled afternoon.

Run 7's measured lesson: crossings CANNOT rank seeds (they ranked seed 0
first; identical full-board probes measured seed 2 at 39 failures vs seed 0's
53, and the run shipped on seed 2). The comparator generates each seed with
place_seed, refuses to rank seeds that fail their own intent gate, and probes
every survivor with the SAME full-board net patterns and route args.

Also under test: the 8bd1344 lesson -- caller-relative paths must be
absolutized at the door, because every subprocess runs with cwd=ROOT.
"""
import json
import os
import subprocess
import sys
import tempfile

#: #716: this test gives its OWN subprocess `timeout=3600` (below), so under
#: run_all's 600 s default it was killed at 600 s -- long before its internal
#: limit could fire -- and reported a machine-speed fact as a code fact. The
#: budget declaration added in #691 was written FOR this test and
#: test_obstacle_map_balance, and neither actually declared one; measured on the
#: #691 branch, this still timed out at 600 s. Matches the internal budget so
#: the subprocess limit is the one that governs.
#:
#: Measured 455 s ALONE (2026-08-22, exit 0, ALL PASS) -- UNDER the 600 s
#: default, yet it timed out in-suite. Same shape as test_431_board_gates
#: (516 s alone, also timing out in-suite): run_all defaults to -j 4, so this
#: competes for CPU with three other processes while that clock runs. This is
#: a budget for the CONTENDED case, and the standalone number is the evidence
#: that the test finishes rather than hangs -- do not read a future timeout
#: here as "it is stuck" without re-measuring it alone first.
RUN_ALL_TIMEOUT = 3600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _pile(d):
    """Stack every footprint at board center: the unplaced-pile fixture."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    pcb = parse_kicad_pcb(BOARD)
    b = pcb.board_info.board_bounds
    cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
    pile = os.path.join(d, 'pile.kicad_pcb')
    write_placed_output(BOARD, pile, [
        {'reference': ref, 'new_x': cx, 'new_y': cy, 'new_rotation': 0}
        for ref, fp in sorted(pcb.footprints.items()) if fp.pads])
    return pile, pcb


def test_end_to_end_ranked_table_from_relative_paths():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as d:
        pile, pcb = _pile(d)
        intent_path = os.path.join(d, 'intent.json')
        from placement.floorplan import emit_intent
        with open(intent_path, 'w', encoding='utf-8') as f:
            json.dump(emit_intent(pcb, BOARD), f)

        # Deliberately RELATIVE paths from a cwd that is neither ROOT nor the
        # temp dir's parent: without absolutize-at-the-door every place_seed/
        # probe subprocess (cwd=ROOT) would silently miss them.
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_placer', 'compare_seeds.py'),
             os.path.relpath(pile, d),
             '--intent', os.path.relpath(intent_path, d),
             '--seeds', '0', '1',
             '--out-dir', 'seedcmp'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=d, timeout=3600,
            env=dict(os.environ, PYTHONIOENCODING='utf-8'))
        out = r.stdout + r.stderr
        assert r.returncode in (0, 4), out[-1500:]

        doc_path = os.path.join(d, 'seedcmp', 'seeds.json')
        assert os.path.isfile(doc_path), (
            "seeds.json missing -- relative --out-dir was not absolutized:\n"
            + out[-800:])
        doc = json.load(open(doc_path, encoding='utf-8'))
        assert [row['seed'] for row in doc['rows']] == [0, 1]
        assert sorted(doc['ranking']) == [0, 1]
        for row in doc['rows']:
            assert os.path.isfile(row['board']), f"seed board missing: {row}"
            if row.get('probe'):
                assert row['probe']['probe_kind'] == 'full'
                assert isinstance(row['probe']['failures'], int), row['probe']
        line = next(l for l in r.stdout.splitlines()
                    if l.startswith('JSON_SUMMARY:'))
        s = json.loads(line.split(':', 1)[1])
        assert s['seeds'] == 2
        if r.returncode == 0:
            assert s['best_seed'] in (0, 1)
            best_row = next(x for x in doc['rows']
                            if x['seed'] == s['best_seed'])
            assert not best_row.get('gated'), (
                "a seed that fails its own intent gate must never rank best")
        else:
            assert s['best_seed'] is None
    print("  PASS: ranked table + seeds.json from caller-relative paths")


def test_duplicate_seeds_refused():
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'compare_seeds.py'), BOARD,
         '--intent', 'x.json', '--seeds', '1', '1', '--out-dir', 'y'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT)
    assert r.returncode == 2 and 'duplicates' in r.stderr
    print("  PASS: duplicate --seeds refused")


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"--- {fn.__name__}")
        fn()
    print("ALL PASS")
