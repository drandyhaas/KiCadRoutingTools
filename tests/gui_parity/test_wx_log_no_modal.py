#!/usr/bin/env python3
"""A logged wx error does not block a headless gate.

wx's default log target shows a logged error as a modal box from the next idle
event, or at exit (`wxLogGui::Flush -> wxDialog::ShowModal`). Headless, nobody can
dismiss it, so a gate sits there at ~0 CPU: one did for 85 minutes. So every
headless `wx.App` is followed at once by the stderr target: `wx.App` resets the
target, so sooner has no effect, and changing it flushes the old one, so after an
error is too late.

  1. with the target set after the app, a logged error reaches stderr and the
     event loop and the process both end;
  2. control: set BEFORE the app, or not at all, the same process does not end;
  3. every `wx.App(` in tests/gui_parity and `headless_plan.py` is followed by it.

Each child runs under a hard wall limit. Needs wx; re-execs into KiCad's python.
Exits 2 when none is found, rather than 0.

    python3 tests/gui_parity/test_wx_log_no_modal.py
"""
import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
MARK = 'probe-wx-error'
TARGET = 'wx.Log.SetActiveTarget(wx.LogStderr())'
LIMIT_S = 8
CHILD = f"""
import sys, wx
arm = sys.argv[1]
if arm == 'before':
    {TARGET}
app = wx.App(False)
if arm == 'after':
    {TARGET}
wx.LogError({MARK!r})
wx.CallLater(300, app.ExitMainLoop)
app.MainLoop()
"""
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def child(arm):
    """`(ended, what it wrote to stderr)`, inside the limit."""
    p = subprocess.Popen([sys.executable, '-X', 'utf8', '-c', CHILD, arm],
                         stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
                         errors='replace')
    try:
        _out, err = p.communicate(timeout=LIMIT_S)
    except subprocess.TimeoutExpired:
        p.kill()
        _out, err = p.communicate()
        return False, err
    return p.returncode == 0, err


def without_target():
    """`path:line` of each `wx.App(` whose next line is not the stderr target."""
    bad = []
    files = [p for p in glob.glob(os.path.join(HERE, '*.py')) if p != os.path.abspath(__file__)]
    files.append(os.path.join(REPO, 'py_router', 'headless_plan.py'))
    for path in files:
        with open(path, encoding='utf-8') as f:
            lines = f.read().splitlines()
        for i, ln in enumerate(lines):
            if re.match(r'\s*(\w+ = )?wx\.App\(', ln):
                following = next((x for x in lines[i + 1:] if not x.strip().startswith('#')), '')
                if following.strip() != TARGET:
                    bad.append(f'{os.path.basename(path)}:{i + 1}')
    return bad


def main():
    try:
        import wx  # noqa: F401
    except ImportError:
        from kicad_locate import kicad_python_candidates
        for py in kicad_python_candidates():
            if py != sys.executable and os.path.exists(py) and subprocess.run(
                    [py, '-c', 'import wx'], capture_output=True).returncode == 0:
                sys.exit(subprocess.run([py, '-X', 'utf8', os.path.abspath(__file__)]).returncode)
        print('NOT RUN: no python with wx found')
        sys.exit(2)
    ended, err = child('after')
    check('set after the app: the process ends', ended)
    check('...and the error is on stderr', MARK in err, err[-200:])
    check('control: set before the app, the process does not end', not child('before')[0])
    check('control: not set at all, it does not end', not child('none')[0])
    bad = without_target()
    check('every headless wx.App is followed by the target', not bad, str(bad))
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
