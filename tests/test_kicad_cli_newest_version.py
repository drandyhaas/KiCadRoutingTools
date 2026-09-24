#!/usr/bin/env python3
"""With KiCad 9 AND 10 installed side by side, discovery must pick 10.

Windows installs KiCad under a versioned directory (`Program Files/KiCad/10.0`),
and a plain string sort ranks `9.0` ABOVE `10.0` ('9' > '1'). Two copies of
that sort shipped:

  * `py_tools/kicad_unconnected.find_kicad_cli` -- so converge's `--oracle`
    ran KiCad 9's kicad-cli (no `--refill-zones`), and fill_for_delivery's
    unconnected delta ran one that cannot read the KiCad-10 board it had just
    filled. `kicad_oracle.find_kicad_cli` sorted numerically all along; the
    copy now delegates to it.
  * ten `KICAD_PYTHONS` lists in the gui_parity gates and measure_* scripts
    ("newest first" via `sorted(..., reverse=True)`), so every one of them ran
    under KiCad 9's pcbnew. They now sort on `kicad_locate.path_version_key`,
    as do five more that hard-coded one version (10.0, or 10.0 and 9.0) and
    so could never have found KiCad 11. tests/stress/board_image.py, which
    knew no Windows path at all, now asks kicad_oracle too.

Checks, none needing KiCad (the platform and filesystem are faked), over
8.0 / 9.0 / 9.99 (a KiCad 10 nightly) / 10.0 / 11.0:

  1. the KICAD_PYTHONS sort ranks them newest first, 11.0 on top;
  2. kicad_oracle.find_kicad_cli picks the numerically newest install;
  3. kicad_unconnected and board_image return that answer, not their own;
  4. nothing in the repo sorts a glob of KiCad install dirs without a key=
     -- with the old kicad_unconnected spelling as the scanner's own
     negative control, so a scanner that sees nothing cannot pass.

    python3 tests/test_kicad_cli_newest_version.py
"""
import ast
import os
import subprocess
import sys
import warnings
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import kicad_oracle as ko  # noqa: E402
import kicad_unconnected as ku  # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" ({detail})" if detail else ''))
    if not ok:
        FAILS.append(name)


# --- 1. the one finder picks the numerically newest install ---------------

FAKE_ROOT = os.path.join(os.sep, 'fake', 'Program Files')


#: Directory listing order, i.e. what a Windows glob hands back. 11.0 is the
#: next major, where a string sort still loses (every single-digit version
#: beats every two-digit one); 9.99 is how a KiCad 10 nightly installs, and
#: must rank below the 10.0 release.
VERSIONS = ('10.0', '11.0', '8.0', '9.0', '9.99')
NEWEST = '11.0'


def _install(ver, exe='kicad-cli.exe'):
    return os.path.join(FAKE_ROOT, 'KiCad', ver, 'bin', exe)


def test_gate_lists_sort_newest_first():
    """The exact expression the ten KICAD_PYTHONS lists now use."""
    from kicad_locate import path_version_key
    got = sorted((_install(v, 'python.exe') for v in VERSIONS),
                 key=path_version_key, reverse=True)
    order = [p.split(os.sep)[-3] for p in got]
    check("KICAD_PYTHONS order is 11.0 > 10.0 > 9.99 > 9.0 > 8.0",
          order == ['11.0', '10.0', '9.99', '9.0', '8.0'], ' > '.join(order))


def test_oracle_finder_picks_the_newest():
    listed = [_install(v) for v in VERSIONS]
    check("premise: a string sort of the fixture picks 9.99",
          sorted(listed)[-1] == _install('9.99'), sorted(listed)[-1])

    def fake_glob(pattern, *a, **k):
        return list(listed) if pattern.startswith(FAKE_ROOT) else []

    env = {k: v for k, v in os.environ.items() if k != 'KICAD_CLI'}
    env.update({'ProgramFiles': FAKE_ROOT, 'ProgramFiles(x86)': '',
                'ProgramW6432': '', 'LOCALAPPDATA': ''})
    with mock.patch.dict(os.environ, env, clear=True), \
            mock.patch.object(ko, 'KICAD_CLI_CANDIDATES', []), \
            mock.patch.object(sys, 'platform', 'win32'), \
            mock.patch('glob.glob', fake_glob):
        got = ko.find_kicad_cli()
    check(f"kicad_oracle.find_kicad_cli picks KiCad {NEWEST} of {len(VERSIONS)}",
          got == _install(NEWEST), got)


# --- 2. the other finders have no discovery of their own ------------------

def test_finders_delegate():
    sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
    try:
        import board_image as bi
    finally:
        del sys.path[0]
    sentinel = os.path.join(FAKE_ROOT, 'whatever-the-oracle-says')
    with mock.patch.object(ko, 'find_kicad_cli', lambda: sentinel):
        got = {'kicad_unconnected': ku.find_kicad_cli(),
               'board_image': bi.find_kicad_cli()}
    for who, answer in got.items():
        check(f"{who}.find_kicad_cli returns kicad_oracle's answer",
              answer == sentinel, answer)


# --- 3. no unkeyed sort of a KiCad install glob anywhere -------------------

def _callee(call):
    f = call.func
    return f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', '')


def _is_kicad_glob(node):
    """Does `node` contain a glob()/iglob() call over a KiCad install path?"""
    for n in ast.walk(node):
        if isinstance(n, ast.Call) and _callee(n) in ('glob', 'iglob'):
            if any(isinstance(c, ast.Constant) and isinstance(c.value, str)
                   and 'KiCad' in c.value for c in ast.walk(n)):
                return True
    return False


def unkeyed_kicad_sorts(src, label):
    """Every sorted()/.sort() over a KiCad install glob that has no key=.

    Names are tracked module-wide, not per scope: a false positive is loud and
    easy to fix, where a scope-exact scanner would be a second thing to trust.
    """
    tree = ast.parse(src)
    fed = set()          # names assigned (or +=) from a KiCad glob
    for n in ast.walk(tree):
        if isinstance(n, (ast.Assign, ast.AugAssign)) and _is_kicad_glob(n.value):
            for t in (n.targets if isinstance(n, ast.Assign) else [n.target]):
                if isinstance(t, ast.Name):
                    fed.add(t.id)
    hits = []
    for n in ast.walk(tree):
        if not isinstance(n, ast.Call) or any(k.arg == 'key' for k in n.keywords):
            continue
        if _callee(n) == 'sorted' and n.args:
            arg = n.args[0]
            if _is_kicad_glob(arg) or {x.id for x in ast.walk(arg)
                                       if isinstance(x, ast.Name)} & fed:
                hits.append(f"{label}:{n.lineno}")
        elif (_callee(n) == 'sort' and isinstance(n.func, ast.Attribute)
              and isinstance(n.func.value, ast.Name) and n.func.value.id in fed):
            hits.append(f"{label}:{n.lineno}")
    return hits


# The two spellings that shipped, verbatim but for the separators.
OLD_UNCONNECTED = '''
def find_kicad_cli():
    import glob
    hits = sorted(glob.glob('C:/Program Files/KiCad/*/bin/kicad-cli.exe'))
    return hits[-1]
'''
OLD_GATE_LIST = '''
import glob
KICAD_PYTHONS = [
    *sorted(glob.glob('C:/Program Files/KiCad/*/bin/python.exe'), reverse=True),
]
'''


def _repo_py_files():
    try:
        out = subprocess.run(['git', 'ls-files', '*.py'], cwd=ROOT,
                             capture_output=True, text=True, check=True).stdout
        files = out.split()
    except (OSError, subprocess.CalledProcessError):
        files = []
    if not files:        # no git index (a bare export): walk instead
        for d, dirs, names in os.walk(ROOT):
            dirs[:] = [x for x in dirs if not x.startswith('.')
                       and x not in ('__pycache__', 'target')]
            files += [os.path.relpath(os.path.join(d, x), ROOT)
                      for x in names if x.endswith('.py')]
    return files


def test_no_unkeyed_kicad_sort():
    check("negative control: the old kicad_unconnected spelling is flagged",
          len(unkeyed_kicad_sorts(OLD_UNCONNECTED, 'old')) == 1)
    check("negative control: the old gate candidate list is flagged",
          len(unkeyed_kicad_sorts(OLD_GATE_LIST, 'old')) == 1)
    hits, globbing = [], 0
    for rel in _repo_py_files():
        try:
            with open(os.path.join(ROOT, rel), encoding='utf-8') as fh:
                src = fh.read()
            with warnings.catch_warnings():
                # Another file's invalid escape is not this test's finding.
                warnings.simplefilter('ignore', SyntaxWarning)
                tree = ast.parse(src)
                hits += unkeyed_kicad_sorts(src, rel.replace(os.sep, '/'))
        except (OSError, SyntaxError, UnicodeDecodeError, ValueError):
            continue
        if _is_kicad_glob(tree):
            globbing += 1
    # Non-vacuity: kicad_oracle, kicad_locate and the ten candidate lists
    # all glob KiCad installs; a scan that found none read nothing.
    check("the scan reached the files that glob KiCad installs",
          globbing >= 10, f"{globbing} file(s)")
    check("no sorted()/.sort() of a KiCad install glob without key=",
          not hits, ', '.join(hits))


if __name__ == '__main__':
    for t in (test_gate_lists_sort_newest_first,
              test_oracle_finder_picks_the_newest, test_finders_delegate,
              test_no_unkeyed_kicad_sort):
        print(f"--- {t.__name__}")
        t()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'PASS: the newest KiCad wins everywhere discovery sorts'}")
    sys.exit(1 if FAILS else 0)
