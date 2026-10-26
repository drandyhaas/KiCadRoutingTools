#!/usr/bin/env python3
"""#1415: --no-fix-drc-settings stops the DRC-floor writeback, NOT the tool
records.

The #521 protected-nets / impedance / same-net-pad-clearance records sat inside
the same `if not args.no_fix_drc_settings` block as the floor writeback, so a
chain that passed the flag (to keep a step from lowering min_hole_clearance)
silently voided the protection of its routed diff pairs: the next route.py
ripped and re-routed them single-ended.

Routes a differential pair with route_diff under the flag and asserts the output
project still carries the protected-nets record, and that the flag still keeps
the floor writeback off (the Default class is left as it was).

    python3 tests/test_1415_no_fix_drc_keeps_tool_records.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']


def records(pro):
    with open(evidence(pro, 'output project'), encoding='utf-8') as f:
        return (json.load(f).get('kicad_routing_tools') or {})


def main():
    evidence(BOARD, 'diff-pair board')
    tmp = tempfile.mkdtemp(prefix='t1415_')
    try:
        from copy_board import copy_board
        inb = os.path.join(tmp, 'in.kicad_pcb')
        copy_board(BOARD, inb)
        # a chain step always has a project beside its input (#441); the
        # records are written into the project the writer seeded from it
        with open(os.path.join(tmp, 'in.kicad_pro'), 'w') as f:
            json.dump({'meta': {'version': 1}}, f)
        out = os.path.join(tmp, 'o.kicad_pcb')
        r = check(PY + [tool('route_diff.py'), inb, out, '--nets', 'CSI_*',
                        '--no-fix-drc-settings'], accept=True)
        evidence(out, 'routed board')
        rec = records(os.path.join(tmp, 'o.kicad_pro'))
        assert rec.get('protected_nets'), (
            '--no-fix-drc-settings voided the protected-nets record: '
            f'{rec!r}')
        # the flag still does its own job: no floor writeback line
        assert 'net_class[Default]' not in (r.stdout or ''), \
            'the floor writeback ran despite --no-fix-drc-settings'
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('PASS')


if __name__ == '__main__':
    main()
