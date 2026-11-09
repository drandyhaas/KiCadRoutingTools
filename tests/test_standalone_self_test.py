import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def test_self_test_cli_exercises_native_operations_without_files(tmp_path):
    result = subprocess.run(
        [sys.executable, str(ROOT / 'krt_cli.py'), 'self-test'],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=True,
    )
    report = json.loads(result.stdout)
    assert report['status'] == 'passed'
    assert set(report['versions']) == {'grid_router', 'numpy', 'scipy', 'shapely', 'pillow', 'ortools'}
    assert report['checks']['rust_mst'] == {'length': 7.0, 'edges': 2}
    assert report['checks']['scipy_assignment'] == {'rows': [0, 1], 'columns': [0, 1]}
    assert report['checks']['shapely_polygon'] == {'area': 4.0}
    assert report['checks']['ortools_cp_sat'] == {'status': 'OPTIMAL', 'x': 4}
    assert report['checks']['pillow_png']['pixel'] == [12, 34, 56]
    assert list(tmp_path.iterdir()) == []
    assert result.stderr == ''


def test_self_test_cli_rejects_arguments(tmp_path):
    result = subprocess.run(
        [sys.executable, str(ROOT / 'krt_cli.py'), 'self-test', 'unexpected'],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert 'self-test takes no arguments' in result.stderr
    assert result.stdout == ''
