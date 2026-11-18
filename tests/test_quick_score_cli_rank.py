#!/usr/bin/env python3
"""quick_score --json carries the seed judge's sort key (`judge_key`), with a
non-finite entry written as null, and changes nothing else: the score in the
file, the exit code and the printed line are those of the run without it.

    python3 tests/test_quick_score_cli_rank.py
"""
import contextlib
import importlib.util
import io
import json
import sys
import tempfile
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('quick_score_cli', ROOT / 'py_tools/quick_score.py')
quick_score = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(quick_score)

CASES = [
    # blocking, blocking_by, joins_needed, quality, expected judge_key, exit code
    (2, {}, 7, {'vias': 3, 'copper_mm': 12.5, 'segments': 9}, [0, 2, 7, 3, 12.5, 9], 4),
    (0, {}, None, {'vias': 0, 'copper_mm': 0, 'segments': 0}, [0, 0, 0, 0, 0, 0], 0),
    (None, {}, None, {}, [0, None, None, None, None, None], 2),
    (2, {}, None, {'vias': False}, [0, 2, None, None, None, None], 4),
    (5, {'floorplan': 1, 'assembly': 2}, 7, {}, [3, 5, 7, None, None, None], 4),
]


def test_json_exports_the_seed_judges_rank_without_changing_the_score():
    for blocking, by, joins, quality, expected, exit_code in CASES:
        score = {
            'blocking': blocking,
            'blocking_by': by,
            'joins_needed': joins,
            'quality': quality,
            'poured': {},
            'ungraded': ['floorplan'],
            'components': {'floorplan': {'violations': []},
                           'assembly': {'count_basis': 'issues'}},
        }

        def measure(board, intent=None, baseline=None, _score=score):
            return _score

        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(quick_score, 'quick_score', measure):
            output = Path(td) / 'score.json'
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                rc = quick_score.main(['candidate.kicad_pcb', '--intent', 'intent.json',
                                       '--baseline', 'source.kicad_pcb',
                                       '--json', str(output)])
            raw = output.read_text(encoding='utf-8')
        case = f"blocking={blocking} by={by} joins={joins}"
        assert rc == exit_code, f"{case}: exit {rc}, expected {exit_code}"
        assert 'Infinity' not in raw and 'NaN' not in raw, f"{case}: non-finite JSON: {raw}"
        result = json.loads(raw)
        assert result.pop('judge_key') == expected, f"{case}: judge_key"
        assert result == score, f"{case}: the score changed"
        assert 'judge_key' not in score, f"{case}: the score was mutated"
        assert out.getvalue().startswith(f'blocking={blocking} joins_needed={joins}'), \
            f"{case}: printed line {out.getvalue()[:80]!r}"
        print(f"  ok  {case}: judge_key {expected}, exit {rc}")


TESTS = [test_json_exports_the_seed_judges_rank_without_changing_the_score]

if __name__ == '__main__':
    for t in TESTS:
        t()
    print('PASS: test_quick_score_cli_rank')
    sys.exit(0)
