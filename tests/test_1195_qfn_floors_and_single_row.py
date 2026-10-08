#!/usr/bin/env python3
"""#1195: qfn_fanout writes only the floors of copper it drew, and refuses a
single-row part by name.

1. A stub-mode run (no vias) passed the --via-size/--via-drill defaults to
   the writeback anyway, so glasgow_revC's U1 fan-out lowered the declared
   via diameter 0.5 -> 0.45 and hole 0.3 -> 0.25 for vias that do not exist.
2. A run that changed no copper still ran the writeback.
3. interf_u's BUS1 (62 edge fingers in one row) was analysed as a QFN with
   a 0 edge tolerance and a 0.5 mm fallback pitch: "Found 0 pads to fanout",
   the board written through, floors lowered, exit 0.

Checks:
  1. glasgow U1, stub mode: tracks and no vias; the via and hole floors are
     the input's, and the track floor (copper it did draw) was written.
  2. A run whose net filter matches nothing writes the board through with
     the input's project byte-identical.
  3. interf_u BUS1 exits 1 naming it "not a QFN/QFP", and writes nothing.

    python3 tests/test_1195_qfn_floors_and_single_row.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from copy_board import copy_board                              # noqa: E402

GLASGOW = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
INTERF = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted.kicad_pcb')
VIA_KEYS = ('min_via_diameter', 'min_through_hole_diameter', 'min_via_drill')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def fan(inp, out, *extra):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py',
                        inp, '--output', out] + list(extra), cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    return r.returncode, r.stdout + r.stderr


def rules(board):
    with open(os.path.splitext(board)[0] + '.kicad_pro') as f:
        return json.load(f)['board']['design_settings']['rules']


def stage(src, tmp, name):
    dst = os.path.join(tmp, name)
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(src, dst)
    return dst


def main():
    tmp = tempfile.mkdtemp(prefix='t1195_')
    try:
        g = stage(GLASGOW, tmp, 'g.kicad_pcb')

        # 1. Stub mode draws no via, so it writes no via floor.
        out = os.path.join(tmp, 'g_fan.kicad_pcb')
        rc, log = fan(g, out, '--component', 'U1')
        check('glasgow U1 fans out with tracks and no vias',
              rc == 0 and 'tracks and 0 vias' in log, f'rc={rc}')
        before, after = rules(g), rules(out)
        check('the via and hole floors are the input\'s',
              all(after.get(k) == before.get(k) for k in VIA_KEYS),
              str({k: (before.get(k), after.get(k)) for k in VIA_KEYS}))
        check('the track floor of the copper it drew was written',
              (after.get('min_track_width') or 9) < (before.get('min_track_width') or 0),
              f"{before.get('min_track_width')} -> {after.get('min_track_width')}")

        # 2. No copper, no writeback.
        out2 = os.path.join(tmp, 'g_none.kicad_pcb')
        rc, log = fan(g, out2, '--component', 'U1', '--nets', '/NO_SUCH_NET_1195')
        with open(g[:-len('.kicad_pcb')] + '.kicad_pro', 'rb') as a, \
                open(out2[:-len('.kicad_pcb')] + '.kicad_pro', 'rb') as b:
            same = a.read() == b.read()
        check('a run that drew nothing carries the project byte-identical',
              rc == 0 and 'No fanout tracks generated' in log and same,
              f'rc={rc} same={same}')

        # 3. A single row is refused by name.
        b = stage(INTERF, tmp, 'b.kicad_pcb')
        out3 = os.path.join(tmp, 'b_out.kicad_pcb')
        rc, log = fan(b, out3, '--component', 'BUS1', '--layer', 'F.Cu',
                      '--width', '0.3', '--clearance', '0.254',
                      '--nets', '*', '!GND', '!VCC')
        check('interf_u BUS1 is refused by name with exit 1',
              rc == 1 and 'BUS1' in log and 'not a QFN/QFP' in log
              and 'single line' in log, log.strip().splitlines()[-1] if log else '')
        check('...and nothing is written',
              not os.path.exists(out3)
              and not os.path.exists(out3[:-len('.kicad_pcb')] + '.kicad_pro'))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
