#!/usr/bin/env python3
"""The KiCad refill subprocess sends wx's log to stderr before LoadBoard.

pcbnew reports a load warning through wx's log, whose default target is a
modal message box. In the refill child nobody closes it, so LoadBoard never
returns and the refill burns its whole timeout. A board naming a font the
machine lacks is enough ("Font 'X' not found; substituting 'Noto Sans
Regular'": three 300 s refill timeouts, the plane-tap step killed at 1163 s;
with the log on stderr the refill takes 0.8 s).

Invariants:
1. (any python) `kicad_exact_fill._REFILL_SCRIPT`, run against stand-in `wx`
   and `pcbnew` modules, has a wx.App and a wx.LogStderr target in place when
   it calls LoadBoard, and finishes REFILL_OK.
2. (any python) with no display, where wx.App raises SystemExit (headless
   Linux), the same script still sets the stderr target and finishes
   REFILL_OK instead of exiting before LoadBoard.
   The stand-in pcbnew refuses to import once a wx.App exists, as the real
   one does with a display (KiCad 10.0.6, wxPython 4.2.5 GTK3: "SystemError:
   initialization of _pcbnew raised unreported exception"), so 1 also holds
   the order: pcbnew is imported before the app is made.
3. (KiCad's python) the real refill of watchy with a text in a font that does
   not exist finishes REFILL_OK well inside its timeout, and the substitution
   warning arrives on stderr. Run only where KiCad's python is found; the
   check is NOT RUN, not a pass, elsewhere.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PY_ROUTER = os.path.join(REPO, 'py_router')
sys.path.insert(0, PY_ROUTER)

import kicad_exact_fill  # noqa: E402

_fails = []


def check(ok, what, detail=''):
    print(('PASS: ' if ok else 'FAIL: ') + what + ('' if ok else f' -- {detail}'))
    if not ok:
        _fails.append(what)


FAKE_WX = '''\
class _Target:
    pass
class LogStderr(_Target):
    pass
class Log:
    active = None
    @staticmethod
    def SetActiveTarget(t):
        Log.active = t
_app = None
import os as _os
NO_DISPLAY = bool(_os.environ.get('FAKE_WX_NO_DISPLAY'))
class App:
    def __init__(self, redirect=True):
        global _app
        if NO_DISPLAY:             # wx's own message and exception
            raise SystemExit("Unable to access the X Display, is $DISPLAY set properly?")
        _app = self
        Log.active = None          # wx.App resets the target, as the real one does
def GetApp():
    return _app
'''

FAKE_PCBNEW = '''\
import sys
import wx
if wx.GetApp() is not None:
    # the real _pcbnew fails to initialise once a wx.App exists (GTK3, display)
    raise SystemError("initialization of _pcbnew raised unreported exception")
def LoadBoard(path):
    # with no display no app can exist, so no modal can open
    if ((wx.GetApp() is None and not wx.NO_DISPLAY)
            or not isinstance(wx.Log.active, wx.LogStderr)):
        print("MODAL: LoadBoard with the default wx log target")
        sys.exit(5)
    return _Board()
class _Zones(list):
    pass
class _Board:
    def Zones(self):
        return _Zones()
class ZONE_FILLER:
    def __init__(self, board):
        pass
    def Fill(self, zones):
        pass
def SaveBoard(dst, board, aSkipSettings=False):
    open(dst, 'w').close()
'''

FAKE_TOOL_RECORDS = '''\
def save_board(save, dst, board, **kw):
    save(dst, board, **kw)
'''


def _fake_refill(td, no_display):
    fakes = os.path.join(td, 'fakes')
    if not os.path.isdir(fakes):
        os.makedirs(fakes)
        for name, src in (('wx', FAKE_WX), ('pcbnew', FAKE_PCBNEW),
                          ('tool_records', FAKE_TOOL_RECORDS)):
            with open(os.path.join(fakes, name + '.py'), 'w', encoding='utf-8') as f:
                f.write(src)
    script = os.path.join(td, 'refill.py')
    with open(script, 'w', encoding='utf-8') as f:
        f.write(kicad_exact_fill._REFILL_SCRIPT)
    env = dict(os.environ, PYTHONPATH=fakes)
    env.pop('FAKE_WX_NO_DISPLAY', None)
    if no_display:
        env['FAKE_WX_NO_DISPLAY'] = '1'
    # argv[3] is the fakes dir, so the script's kicad_exact_fill import fails
    # and the repin is skipped, as it degrades in the field.
    return subprocess.run([sys.executable, script, os.path.join(td, 'in.kicad_pcb'),
                           os.path.join(td, 'out.kicad_pcb'), fakes],
                          capture_output=True, text=True, timeout=60, env=env)


def part1(td):
    print('1. the refill script quiets wx before LoadBoard')
    r = _fake_refill(td, no_display=False)
    check('REFILL_OK' in r.stdout and 'MODAL' not in r.stdout,
          'LoadBoard runs with a wx.App and the stderr log target',
          f'rc={r.returncode} {r.stdout[-300:]} {r.stderr[-300:]}')


def part2(td):
    print('2. no display: wx.App raises SystemExit, the refill goes on')
    r = _fake_refill(td, no_display=True)
    check('REFILL_OK' in r.stdout and 'MODAL' not in r.stdout,
          'with no display the refill still reaches LoadBoard and finishes',
          f'rc={r.returncode} {r.stdout[-300:]} {r.stderr[-300:]}')


MISSING_FONT_TEXT = ('\t(gr_text "missing font"\n\t\t(at 20 20 0)\n\t\t(layer "F.SilkS")\n'
                     '\t\t(uuid "1982aaaa-0000-4000-8000-000000001982")\n'
                     '\t\t(effects\n\t\t\t(font\n\t\t\t\t(face "KRT No Such Font 1982")\n'
                     '\t\t\t\t(size 1 1)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)\n')


def part3(td):
    print('3. the real refill of a board with a missing font')
    kpy = kicad_exact_fill.find_kicad_python()
    if not kpy:
        print('NOT RUN: KiCad python not found (not a pass)')
        return
    with open(os.path.join(REPO, 'kicad_files', 'watchy.kicad_pcb'), encoding='utf-8') as f:
        text = f.read()
    end = text.rstrip().rfind(')')
    src = os.path.join(td, 'watchy_font.kicad_pcb')
    with open(src, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text[:end] + MISSING_FONT_TEXT + text[end:])
    dst = os.path.join(td, 'watchy_font_out.kicad_pcb')
    script = os.path.join(td, 'refill_real.py')
    with open(script, 'w', encoding='utf-8') as f:
        f.write(kicad_exact_fill._REFILL_SCRIPT)
    kw = {}
    if sys.platform == 'win32':
        kw['creationflags'] = subprocess.CREATE_NO_WINDOW
    try:
        r = subprocess.run([kpy, script, src, dst, PY_ROUTER], capture_output=True, text=True,
                           timeout=120, **kw)
    except subprocess.TimeoutExpired:
        check(False, 'the refill finishes', 'timed out after 120 s (a modal wx dialog?)')
        return
    check('REFILL_OK' in r.stdout, 'the refill finishes REFILL_OK',
          f'rc={r.returncode} {r.stdout[-300:]} {r.stderr[-300:]}')
    check(re.search(r"KRT No Such Font 1982.*not found", r.stderr) is not None,
          'the missing-font warning went to stderr, not a dialog', r.stderr[-300:])


def main():
    td = tempfile.mkdtemp(prefix='t1982_')
    try:
        part1(td)
        part2(td)
        part3(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if _fails:
        print(f'FAIL: {len(_fails)}: ' + '; '.join(_fails))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
