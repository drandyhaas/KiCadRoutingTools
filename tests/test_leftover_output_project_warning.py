#!/usr/bin/env python3
"""An output project left by an earlier run is kept, and said so.

Every routing writer seeds the output's siblings from the input before the
board is written (seed_project_for_output), and an existing output sibling is
never overwritten (test_passthrough_carries_project, #1190). So a step rerun
into the same output path keeps the earlier attempt's .kicad_pro, and the floor
writeback lowers from ITS floors: a PCB-chat routing plan run twice in one
folder on 2026-10-22 kept a 0.127 mm track floor from its first attempt, and
the only sign was "unchanged by this step but still below the board's
ORIGINAL". The writer now warns when the output's project is a leftover whose
rules differ from the input's.

Asserted:
  1. a leftover with different rules warns, and is still kept as it was;
  2. once per output per process: the step's own writeback since is not a
     leftover;
  3. no warning for an identical leftover, for an output with no project yet
     (it is seeded from the input, as before), or in place;
  4. route.py, end to end, warns when its output has a leftover project and
     not when it has none.

    python3 tests/test_leftover_output_project_warning.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import fix_kicad_drc_settings as fds                             # noqa: E402
from run_utils import check, evidence                            # noqa: E402

failures = []
WARNING = 'is left from an earlier run and is kept'


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def project(track):
    return {'board': {'design_settings': {'rules': {'min_track_width': track}}},
            'meta': {'filename': 'x.kicad_pro', 'version': 1}}


def write_json(path, doc):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f)


def read_json(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def seed(out_pcb, in_pcb):
    """seed_project_for_output's console output."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fds.seed_project_for_output(out_pcb, in_pcb)
    return buf.getvalue()


def stage(td, name='in'):
    pcb = os.path.join(td, f'{name}.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20240108))\n')
    write_json(os.path.join(td, f'{name}.kicad_pro'), project(0.2))
    return pcb


def main():
    print("1-2: a leftover with different rules")
    with tempfile.TemporaryDirectory() as td:
        inp, out = stage(td), os.path.join(td, 'out.kicad_pcb')
        write_json(os.path.join(td, 'out.kicad_pro'), project(0.127))
        log = seed(out, inp)
        ok("warns", WARNING in log, log.strip()[-200:])
        ok("names the output's project", 'out.kicad_pro' in log)
        ok("keeps it as it was",
           read_json(os.path.join(td, 'out.kicad_pro')) == project(0.127))
        write_json(os.path.join(td, 'out.kicad_pro'), project(0.1))
        ok("once per output: the step's own writeback since is not a leftover",
           WARNING not in seed(out, inp))

    print("3: no warning")
    with tempfile.TemporaryDirectory() as td:
        inp, out = stage(td), os.path.join(td, 'out.kicad_pcb')
        write_json(os.path.join(td, 'out.kicad_pro'), project(0.2))
        ok("an identical leftover", WARNING not in seed(out, inp))
    with tempfile.TemporaryDirectory() as td:
        inp, out = stage(td), os.path.join(td, 'out.kicad_pcb')
        log = seed(out, inp)
        ok("an output with no project yet", WARNING not in log, log.strip())
        ok("...which is seeded from the input, as before",
           read_json(os.path.join(td, 'out.kicad_pro')) == project(0.2))
    with tempfile.TemporaryDirectory() as td:
        inp = stage(td)
        ok("in place", WARNING not in seed(inp, inp))

    print("4: route.py end to end")
    board = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
    tmp = tempfile.mkdtemp(prefix='tleftover_')
    try:
        for leftover in (False, True):
            run_dir = os.path.join(tmp, 'leftover' if leftover else 'fresh')
            os.makedirs(run_dir)
            inp = os.path.join(run_dir, 'in.kicad_pcb')
            shutil.copyfile(board, inp)
            write_json(os.path.join(run_dir, 'in.kicad_pro'), project(0.2))
            out = os.path.join(run_dir, 'out.kicad_pcb')
            if leftover:
                write_json(os.path.join(run_dir, 'out.kicad_pro'), project(0.1))
            evidence(inp, 'the input board')
            r = check([sys.executable, '-X', 'utf8',
                       os.path.join(ROOT, 'py_router', 'route.py'), inp, out,
                       '--nets', '/DTR'], accept=True)
            log = r.stdout + r.stderr
            ok(f"{'warns' if leftover else 'no warning'} with "
               f"{'a leftover' if leftover else 'no'} output project",
               (WARNING in log) == leftover,
               '' if (WARNING in log) == leftover else log[-300:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
