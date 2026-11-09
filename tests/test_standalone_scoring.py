import builtins
from pathlib import Path
import runpy
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'py_tools'))

import board_score
from krt_standalone import process, tools


def test_frozen_scoring_launches_private_checker_dispatch(monkeypatch):
    commands = []
    allowed_scripts = tools.bundled_scripts(ROOT)

    class Child:
        returncode = 4

        def __init__(self, argv, **kwargs):
            self.args = argv
            commands.append((argv, kwargs))

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def communicate(self, *args, **kwargs):
            return 'FOUND 17 ISSUES', None

        def poll(self):
            return self.returncode

    monkeypatch.setattr(sys, 'frozen', True, raising=False)
    monkeypatch.setattr(process, 'runtime_root', lambda: ROOT)
    monkeypatch.setattr(tools, 'runtime_root', lambda: ROOT)
    monkeypatch.setattr(tools, 'bundled_scripts', lambda root: allowed_scripts)
    monkeypatch.setattr(process, '_job', SimpleNamespace(attach=lambda child: None))
    monkeypatch.setattr(process, '_children', set())
    monkeypatch.setattr(subprocess, 'Popen', Child)
    rc, output = board_score.run_tool(str(ROOT), 'check_connected.py', 'board with spaces.kicad_pcb')

    assert rc == 4
    assert output == 'FOUND 17 ISSUES'
    assert commands[0][0] == [
        sys.executable, '_tool', 'py_router/check_connected.py', 'board with spaces.kicad_pcb',
    ]
    assert commands[0][1]['env']['KRT_NO_BANNER'] == '1'
    if sys.platform == 'win32':
        assert commands[0][1]['creationflags'] & subprocess.CREATE_NO_WINDOW


def test_source_scoring_keeps_interpreter_and_external_clone_paths(monkeypatch):
    commands = []

    def run(argv, **kwargs):
        commands.append((argv, kwargs))
        return subprocess.CompletedProcess(argv, 0, stdout='ALL NETS FULLY CONNECTED')

    monkeypatch.delattr(sys, 'frozen', raising=False)
    monkeypatch.setattr(subprocess, 'run', run)
    external_root = ROOT.parent / 'external-clone'
    rc, output = board_score.run_tool(str(external_root), 'check_connected.py', 'source.kicad_pcb')

    assert rc == 0
    assert output == 'ALL NETS FULLY CONNECTED'
    assert commands[0][0] == [
        sys.executable, '-X', 'utf8', str(external_root / 'check_connected.py'), 'source.kicad_pcb',
    ]


@pytest.mark.parametrize('script,entry,args', [
    ('py_router/autoplan.py', None, ()),
    ('py_placer/judge_seeds.py', None, ()),
    ('awx/whole_route.py', None, ()),
    ('py_placer/place_simple.py', '_many_seeds', (None, None)),
    ('py_placer/place_simple.py', '_preview', (None, None)),
])
def test_frozen_driver_never_falls_back_when_adapter_import_breaks(monkeypatch, script, entry, args):
    original_import = builtins.__import__

    def importing(name, *args, **kwargs):
        if name == 'krt_standalone.process':
            raise ImportError('broken frozen process adapter')
        return original_import(name, *args, **kwargs)

    monkeypatch.syspath_prepend(str(ROOT / 'py_router'))
    monkeypatch.syspath_prepend(str(ROOT / 'py_placer'))
    monkeypatch.setattr(sys, 'frozen', True, raising=False)
    monkeypatch.setattr(builtins, '__import__', importing)
    with pytest.raises(ImportError, match='broken frozen process adapter'):
        namespace = runpy.run_path(str(ROOT / script), run_name='standalone_import_test')
        if entry:
            namespace[entry](*args)
