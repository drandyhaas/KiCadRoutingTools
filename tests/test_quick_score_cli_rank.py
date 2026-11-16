import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('quick_score_cli', ROOT / 'py_tools/quick_score.py')
quick_score = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(quick_score)


@pytest.mark.parametrize(
    'blocking,by,joins,quality,expected,exit_code',
    [
        (2, {}, 7, {'vias': 3, 'copper_mm': 12.5, 'segments': 9}, [0, 2, 7, 3, 12.5, 9], 4),
        (0, {}, None, {'vias': 0, 'copper_mm': 0, 'segments': 0}, [0, 0, 0, 0, 0, 0], 0),
        (None, {}, None, {}, [0, None, None, None, None, None], 2),
        (2, {}, None, {'vias': False}, [0, 2, None, None, None, None], 4),
        (5, {'floorplan': 1, 'assembly': 2}, 7, {}, [3, 5, 7, None, None, None], 4),
    ],
)
def test_json_exports_the_seed_judges_rank_without_changing_the_score(
    tmp_path, monkeypatch, capsys, blocking, by, joins, quality, expected, exit_code
):
    score = {
        'blocking': blocking,
        'blocking_by': by,
        'joins_needed': joins,
        'quality': quality,
        'poured': {},
        'ungraded': ['floorplan'],
        'components': {'floorplan': {'violations': []}, 'assembly': {'count_basis': 'issues'}},
    }

    def measure(board, intent=None, baseline=None):
        return score

    monkeypatch.setattr(quick_score, 'quick_score', measure)
    output = tmp_path / 'score.json'
    assert quick_score.main([
        'candidate.kicad_pcb',
        '--intent',
        'intent.json',
        '--baseline',
        'source.kicad_pcb',
        '--json',
        str(output),
    ]) == exit_code
    raw = output.read_text(encoding='utf-8')
    assert 'Infinity' not in raw
    assert 'NaN' not in raw
    result = json.loads(raw)
    assert result.pop('judge_key') == expected
    assert result == score
    assert 'judge_key' not in score
    assert capsys.readouterr().out.startswith(f'blocking={blocking} joins_needed={joins}')
