#!/usr/bin/env python3
"""A run that is killed must not leave something that looks like a result.

Run-7 A11, three faces of one defect in place_reconstruct:

  * the output board was written BEFORE the legalize stage ran, so a kill in
    between left a board at the caller's chosen path that reads as finished and
    is not (legalize sweeps caps for minutes and moves parts);
  * `--dry-run --stages legalize` returned before legalize, so the preview said
    nothing about the stage it was asked to preview -- a worker recorded
    "legalize: no-op" for a stage that repaired 7 parts when it really ran;
  * stdout redirected to a file is BLOCK-buffered, so a killed run's log ends
    mid-buffer. One board's legalize log ended at 200 bytes.

Run: python3 -X utf8 tests/test_run8_write_order.py
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

import run_utils  # tool_env: PYTHONPATH for `python -c` children (#522)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def run(args, **kw):
    proc = subprocess.run([sys.executable, '-X', 'utf8'] + args,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, **kw)
    return proc.returncode, (proc.stdout or '') + (proc.stderr or '')


def summary(out):
    for line in reversed(out.splitlines()):
        if line.startswith('JSON_SUMMARY: '):
            return json.loads(line[len('JSON_SUMMARY: '):])
    return {}


def main():
    script = os.path.join(ROOT, 'py_placer', 'place_reconstruct.py')

    with tempfile.TemporaryDirectory() as tmp:
        # An input carrying siblings: the .kicad_pro is the DRC floor a later
        # chain step reads, so promotion has to move it too (#441).
        src = os.path.join(tmp, 'in.kicad_pcb')
        shutil.copy(BOARD, src)
        with open(os.path.join(tmp, 'in.kicad_pro'), 'w', encoding='utf-8') as fh:
            fh.write('{"board": {"design_settings": {}}}\n')

        out = os.path.join(tmp, 'out.kicad_pcb')
        code, log = run([script, src, out, '--dry-run'])
        rep = summary(log)
        check('dry-run exits 0', code == 0, log[-400:])
        check('dry-run writes NO output board', not os.path.exists(out))
        check('dry-run previews the legalize stage it was asked to run',
              isinstance(rep.get('legalize'), dict)
              and rep['legalize'].get('preview') is True,
              json.dumps(rep.get('legalize')))
        check('the preview names what legalize would move',
              'would_move' in (rep.get('legalize') or {}))
        check('dry-run leaves no staging files',
              not glob.glob(os.path.join(tmp, '*staging*')),
              str(os.listdir(tmp)))

        code, log = run([script, src, out])
        check('real run exits 0', code == 0, log[-400:])
        check('real run writes the output board', os.path.isfile(out))
        check('promotion carries the .kicad_pro sibling',
              os.path.isfile(os.path.join(tmp, 'out.kicad_pro')),
              str(sorted(os.listdir(tmp))))
        check('no staging board survives a completed run',
              not glob.glob(os.path.join(tmp, '*staging*')),
              str(sorted(os.listdir(tmp))))
        check('no legalize temp survives a completed run',
              not glob.glob(os.path.join(tmp, '*.legalize')),
              str(sorted(os.listdir(tmp))))

    # The forensic half: a redirected log is line-buffered, so whatever ran
    # before a kill is already on disk.
    src_banner = open(os.path.join(ROOT, 'py_router', 'cli_banner.py'), encoding='utf-8').read()
    check('cli_banner line-buffers redirected output',
          'line_buffering=True' in src_banner)
    with tempfile.TemporaryDirectory() as tmp:
        log_path = os.path.join(tmp, 'log.txt')
        prog = ('import cli_banner; cli_banner.install();'
                'print("early line"); import os; os._exit(9)')
        with open(log_path, 'w', encoding='utf-8') as fh:
            # tool_env, because a `python -c` child gets sys.path[0] = cwd and
            # NOTHING else: `cli_banner` lives in py_router/ since the #522
            # reorg, so the child died with ModuleNotFoundError and printed no
            # line at all. This check then passed anyway -- see below.
            subprocess.run([sys.executable, '-X', 'utf8', '-c', prog],
                           stdout=fh, stderr=subprocess.STDOUT, cwd=ROOT,
                           env=run_utils.tool_env())
        text = open(log_path, encoding='utf-8').read()
        # An EXACT LINE, not `in text`, and that is the whole point. Python
        # 3.13+ echoes the offending source line in a traceback, and this
        # program's source CONTAINS the string `early line` -- so
        # `'early line' in text` was satisfied by the failure message itself.
        # The check passed on 3.13+ while the child was not running at all,
        # and failed on 3.12 only because its tracebacks carry no source echo:
        # a green that meant "this Python echoes source" and a red that meant
        # "this one does not", neither of them about the thing under test.
        # The echoed source is indented and carries the whole statement, so it
        # can never equal a bare `early line`.
        lines = [ln.strip() for ln in text.splitlines()]
        check('the probe child actually ran (no import failure)',
              'Traceback (most recent call last):' not in lines, repr(text))
        check('a hard-killed run still leaves its printed lines on disk',
              'early line' in lines, repr(text))

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
