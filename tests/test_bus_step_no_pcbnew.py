#!/usr/bin/env python3
"""The bus step never loads KiCad's pcbnew in its processes (swig_compat.keep_pcbnew_out): OR-Tools, which it solves
with, cannot share a process with pcbnew (Windows: WinError 127 either way; KiCad's bundled python on macOS crashed).

A stand-in `pcbnew` on PYTHONPATH writes a mark each time it is imported, so a check sees whether a process loaded it:

1. the control: a process that imports kicad_parser loads the stand-in (so a missing mark below is the block's);
2. route_bus.py, run as the step runs (`--find` here), never loads it -- and still does its work;
3. a process started with NO_PCBNEW in its environment (as the step's stage processes are) imports kicad_parser
   without loading it, and `import pcbnew` raises ImportError there;
4. a process started with swig_compat.pcbnew_env() (KiCad's exact zone fill) does load it;
5. kicad_exact_fill.find_kicad_python takes the running interpreter for KiCad's python only for a REAL pcbnew already
   loaded, never for the block's placeholder.

    python3 tests/test_bus_step_no_pcbnew.py
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYR = os.path.join(ROOT, 'py_router')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" + (f" -- {detail}" if detail and not ok else ''), flush=True)
    if not ok:
        FAILS.append(name)


def main():
    sys.path.insert(0, PYR)
    import swig_compat
    from run_utils import evidence  # noqa: F401  (tests/ on the path below)
    tmp = tempfile.mkdtemp(prefix='krt_nopcb_')
    fake = os.path.join(tmp, 'fake')
    os.makedirs(fake)
    mark = os.path.join(tmp, 'mark.txt')
    with open(os.path.join(fake, 'pcbnew.py'), 'w') as f:
        f.write("import os\nopen(os.environ['FAKE_PCBNEW_MARK'], 'a').write('imported\\n')\n"
                "class SwigPyIterator:\n    pass\n")
    base = {k: v for k, v in os.environ.items() if k not in (swig_compat.NO_PCBNEW, 'PYTHONHOME')}
    env = dict(base, PYTHONPATH=os.pathsep.join([fake, PYR]), FAKE_PCBNEW_MARK=mark)

    def marked():
        return os.path.isfile(mark) and open(mark).read().count('imported')

    def clear():
        if os.path.isfile(mark):
            os.remove(mark)

    print('1. the control')
    r = subprocess.run([sys.executable, '-c', 'import kicad_parser'], env=env, capture_output=True, text=True)
    if r.returncode != 0:
        raise AssertionError(f"BROKEN TEST: kicad_parser did not import: {r.stderr[-600:]}")
    check('a process importing kicad_parser loads the stand-in', marked() >= 1, r.stderr[-300:])

    print('2. route_bus.py, as the step runs')
    clear()
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'awx', 'route_bus.py'), BOARD, '--find'],
                       env=env, capture_output=True, text=True, timeout=600)
    check('route_bus.py --find did its work', r.returncode == 0 and 'the step:' in r.stdout,
          f"rc={r.returncode} {r.stderr[-400:]}")
    check('...and never loaded pcbnew', not marked(), open(mark).read() if os.path.isfile(mark) else '')

    print('3. a stage process (NO_PCBNEW in its environment)')
    clear()
    code = ("import kicad_parser\ntry:\n    import pcbnew\n    print('LOADED')\n"
            "except ImportError:\n    print('BLOCKED')\n")
    r = subprocess.run([sys.executable, '-c', code], env=dict(env, **{swig_compat.NO_PCBNEW: '1'}),
                       capture_output=True, text=True)
    check('kicad_parser imports, `import pcbnew` raises ImportError, the stand-in never loads',
          r.returncode == 0 and r.stdout.strip() == 'BLOCKED' and not marked(), f"{r.stdout} {r.stderr[-300:]}")

    print('4. a pcbnew process (pcbnew_env)')
    clear()
    e4 = swig_compat.pcbnew_env(dict(env, **{swig_compat.NO_PCBNEW: '1'}))
    check('pcbnew_env drops the setting', swig_compat.NO_PCBNEW not in e4)
    r = subprocess.run([sys.executable, '-c', 'import pcbnew'], env=e4, capture_output=True, text=True)
    check('a process started with it loads pcbnew', r.returncode == 0 and marked() >= 1, r.stderr[-300:])

    print("5. find_kicad_python and the block's placeholder")
    code = ("import sys\nsys.path.insert(0, %r)\nimport swig_compat, kicad_exact_fill as kx\n"
            "swig_compat.keep_pcbnew_out()\nkx.kicad_python_candidates = lambda: [sys.executable]\n"
            "kx._KICAD_PYTHON_MEMO.clear()\nprint('FOUND' if kx.find_kicad_python() == sys.executable else 'NONE')\n"
            % PYR)
    real = subprocess.run([sys.executable, '-c', 'import pcbnew'], env=base, capture_output=True).returncode == 0
    r = subprocess.run([sys.executable, '-c', code], env=base, capture_output=True, text=True)
    want = 'FOUND' if real else 'NONE'
    check(f"this interpreter {'can' if real else 'cannot'} load a real pcbnew: {want}",
          r.stdout.strip() == want, f"{r.stdout} {r.stderr[-400:]}")

    print(f"\n{'FAIL: ' + '; '.join(FAILS) if FAILS else 'PASS: the bus step keeps pcbnew out of its processes'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    sys.exit(main())
