import pathlib
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from krt_standalone import process
from krt_standalone.tools import resolve_script


def test_source_cli_help_and_version():
    result = subprocess.run(
        [sys.executable, str(ROOT / 'krt_cli.py'), '--version'],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout.strip() == 'KiCadRoutingTools 0.23.8'


@pytest.mark.parametrize('command', ['bga-fanout', 'qfn-fanout', 'list-nets'])
def test_cli_help_without_interactive_exit_helper(command):
    result = subprocess.run([
        sys.executable, '-c',
        'import builtins, runpy, sys; del builtins.exit; '
        'sys.argv = sys.argv[1:]; runpy.run_path(sys.argv[0], run_name="__main__")',
        str(ROOT / 'krt_cli.py'), command, '--help',
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'usage:' in result.stdout.lower()


def test_path_boundary_rejects_parent_escape():
    try:
        resolve_script('../krt_cli.py')
    except ValueError as error:
        assert str(error) == 'KRT_TOOL_PATH_NOT_ALLOWED'
    else:
        raise AssertionError('parent path was accepted')


def test_frozen_task_argv_maps_known_script(monkeypatch):
    executable = str(ROOT / 'dist' / 'krt.exe')
    script = str(ROOT / 'py_tools' / 'quick_score.py')
    monkeypatch.setattr(sys, 'executable', executable)
    monkeypatch.setattr(process, 'frozen', lambda: True)
    assert process.task_argv([executable, '-u', '-X', 'utf8', script, 'board.kicad_pcb']) == [
        executable,
        '_tool',
        'py_tools/quick_score.py',
        'board.kicad_pcb',
    ]


def test_frozen_task_argv_rejects_python_code(monkeypatch):
    executable = str(ROOT / 'dist' / 'krt.exe')
    monkeypatch.setattr(sys, 'executable', executable)
    monkeypatch.setattr(process, 'frozen', lambda: True)
    try:
        process.task_argv([executable, '-X', 'utf8', '-c', 'print(1)'])
    except ValueError as error:
        assert str(error) == 'KRT_FROZEN_PYTHON_MODE_NOT_ALLOWED'
    else:
        raise AssertionError('arbitrary Python code was accepted')
