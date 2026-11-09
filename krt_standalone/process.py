"""Narrow process boundary used by the frozen KRT command line."""

from __future__ import annotations

import atexit
import os
import subprocess
import sys
import threading
from typing import Sequence

from .tools import resolve_script, runtime_root


_lock = threading.Lock()
_job = None
_children = set()


def frozen() -> bool:
    return bool(getattr(sys, 'frozen', False))


def _script_in(argv: Sequence[str]) -> tuple[int, str] | None:
    for index, value in enumerate(argv):
        if isinstance(value, str) and value.endswith('.py'):
            return index, value
    return None


def task_argv(argv: Sequence[str]) -> list[str]:
    """Translate a Python script command to the private frozen dispatcher."""
    result = list(argv)
    if not frozen() or not result or result[0] != sys.executable:
        return result
    found = _script_in(result[1:])
    if found is None:
        raise ValueError('KRT_FROZEN_PYTHON_MODE_NOT_ALLOWED')
    index, script = found
    index += 1
    relative = resolve_script(script).relative_to(runtime_root().resolve()).as_posix()
    options = result[1:index]
    invalid = [item for item in options if item not in ('-u', '-X', 'utf8', 'utf-8')]
    if invalid:
        raise ValueError('KRT_FROZEN_PYTHON_OPTION_NOT_ALLOWED: ' + ' '.join(invalid))
    return [sys.executable, '_tool', relative, *result[index + 1:]]


def child_kwargs() -> dict[str, object]:
    if frozen() and os.name == 'nt':
        return {'creationflags': getattr(subprocess, 'CREATE_NO_WINDOW', 0)}
    return {}


def run(argv: Sequence[str], **kwargs):
    if not frozen() or os.name != 'nt':
        return subprocess.run(task_argv(argv), **kwargs)
    input_data = kwargs.pop('input', None)
    capture_output = kwargs.pop('capture_output', False)
    timeout = kwargs.pop('timeout', None)
    check = kwargs.pop('check', False)
    if input_data is not None:
        if kwargs.get('stdin') is not None:
            raise ValueError('stdin and input arguments may not both be used.')
        kwargs['stdin'] = subprocess.PIPE
    if capture_output:
        if kwargs.get('stdout') is not None or kwargs.get('stderr') is not None:
            raise ValueError('stdout and stderr arguments may not be used with capture_output.')
        kwargs['stdout'] = kwargs['stderr'] = subprocess.PIPE
    with popen(argv, **kwargs) as process:
        try:
            stdout, stderr = process.communicate(input_data, timeout=timeout)
        except subprocess.TimeoutExpired as error:
            process.kill()
            error.stdout, error.stderr = process.communicate()
            raise
        except BaseException:
            process.kill()
            process.wait()
            raise
        if check and process.returncode:
            raise subprocess.CalledProcessError(process.returncode, process.args,
                                                output=stdout, stderr=stderr)
        return subprocess.CompletedProcess(process.args, process.returncode, stdout, stderr)


def popen(argv: Sequence[str], **kwargs):
    global _job
    argv = task_argv(argv)
    if not frozen() or os.name != 'nt':
        return subprocess.Popen(argv, **kwargs)
    flags = child_kwargs()['creationflags']
    kwargs['creationflags'] = kwargs.get('creationflags', 0) | flags
    with _lock:
        if _job is None:
            from .windows_job import WindowsJob
            _job = WindowsJob()
        process = subprocess.Popen(argv, **kwargs)
        try:
            _job.attach(process)
        except BaseException:
            process.kill()
            process.wait()
            raise
        _children.add(process)
        return process


def close_children():
    global _job
    with _lock:
        job, _job = _job, None
        children = tuple(_children)
        _children.clear()
        if job is not None:
            job.close()
    for process in children:
        process.wait(timeout=10)


atexit.register(close_children)
