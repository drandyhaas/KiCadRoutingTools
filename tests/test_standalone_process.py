import ctypes
from ctypes import wintypes
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from krt_standalone import process


def test_source_process_preserves_capture_input_and_failure():
    command = [sys.executable, '-c',
               'import sys; print(sys.stdin.read()); print("failure", file=sys.stderr); sys.exit(7)']
    result = process.run(command, input='payload', capture_output=True, text=True)
    assert result.returncode == 7
    assert result.stdout.strip() == 'payload'
    assert result.stderr.strip() == 'failure'
    with pytest.raises(subprocess.CalledProcessError) as failure:
        process.run(command, input='payload', capture_output=True, text=True, check=True)
    assert failure.value.returncode == 7
    assert failure.value.stderr.strip() == 'failure'


def test_frozen_interpreter_requires_a_known_script(monkeypatch):
    monkeypatch.setattr(process, 'frozen', lambda: True)
    with pytest.raises(ValueError, match='KRT_FROZEN_PYTHON_MODE_NOT_ALLOWED'):
        process.task_argv([sys.executable])
    with pytest.raises(ValueError, match='KRT_FROZEN_PYTHON_MODE_NOT_ALLOWED'):
        process.task_argv([sys.executable, '-m', 'arbitrary'])


def test_frozen_preview_resolves_contained_parent_segments(monkeypatch):
    monkeypatch.setattr(process, 'frozen', lambda: True)
    script = str(ROOT / 'py_placer' / '..' / 'py_tools' / 'render_placement.py')
    args = [sys.executable, '-X', 'utf8', script, 'input.kicad_pcb', '-o', 'preview.png']
    assert process.task_argv(args) == [
        sys.executable, '_tool', 'py_tools/render_placement.py',
        'input.kicad_pcb', '-o', 'preview.png',
    ]
    outside = str(ROOT / 'py_placer' / '..' / '..' / 'outside.py')
    with pytest.raises(ValueError, match='KRT_TOOL_PATH_NOT_ALLOWED'):
        process.task_argv([sys.executable, outside])


@pytest.mark.skipif(os.name != 'nt', reason='Windows Job Objects')
def test_windows_job_closes_child_and_grandchild():
    from krt_standalone.windows_job import WindowsJob

    child_code = (
        'import subprocess, sys, time\n'
        'sys.stdin.readline()\n'
        'p = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])\n'
        'print(p.pid, flush=True)\n'
        'time.sleep(60)\n'
    )
    api = ctypes.WinDLL('kernel32', use_last_error=True)
    api.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    api.OpenProcess.restype = wintypes.HANDLE
    api.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    api.WaitForSingleObject.restype = wintypes.DWORD
    api.CloseHandle.argtypes = [wintypes.HANDLE]
    api.CloseHandle.restype = wintypes.BOOL
    job = WindowsJob()
    grandchild_handle = None
    child = subprocess.Popen([sys.executable, '-u', '-c', child_code],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                             creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        job.attach(child)
        child.stdin.write('start\n')
        child.stdin.flush()
        grandchild_pid = int(child.stdout.readline().strip())
        grandchild_handle = api.OpenProcess(0x00100000, False, grandchild_pid)
        assert grandchild_handle
        assert api.WaitForSingleObject(grandchild_handle, 0) == 0x102
        job.close()
        child.wait(timeout=10)
        assert child.poll() is not None
        assert api.WaitForSingleObject(grandchild_handle, 10000) == 0
    finally:
        job.close()
        if child.poll() is None:
            child.kill()
            child.wait(timeout=10)
        if grandchild_handle:
            api.CloseHandle(grandchild_handle)
        child.stdin.close()
        child.stdout.close()


@pytest.mark.skipif(os.name != 'nt', reason='Windows Job Objects')
def test_frozen_process_boundary_owns_and_closes_spawned_child(monkeypatch):
    monkeypatch.setattr(process, 'frozen', lambda: True)
    monkeypatch.setattr(process, 'task_argv', lambda argv: list(argv))
    child = process.popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                          creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    try:
        assert child.poll() is None
        process.close_children()
        assert child.poll() is not None
    finally:
        process.close_children()


@pytest.mark.skipif(os.name != 'nt', reason='Windows frozen process boundary')
def test_frozen_run_preserves_output_timeout_and_exit_code(monkeypatch):
    monkeypatch.setattr(process, 'frozen', lambda: True)
    monkeypatch.setattr(process, 'task_argv', lambda argv: list(argv))
    try:
        result = process.run([sys.executable, '-c',
                              'import sys; print(sys.stdin.read()); print("err", file=sys.stderr)'],
                             capture_output=True, input='hello', text=True, timeout=10)
        assert result.returncode == 0
        assert result.stdout.strip() == 'hello'
        assert result.stderr.strip() == 'err'
        with pytest.raises(subprocess.CalledProcessError) as failure:
            process.run([sys.executable, '-c', 'import sys; sys.exit(9)'], check=True)
        assert failure.value.returncode == 9
        with pytest.raises(subprocess.TimeoutExpired):
            process.run([sys.executable, '-c', 'import time; time.sleep(60)'], timeout=0.2)
    finally:
        process.close_children()
