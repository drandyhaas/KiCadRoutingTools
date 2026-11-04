#!/usr/bin/env python3
"""#1395: a net the operator names EXACTLY in --rip-existing-nets (the #521
override) is rippable for the whole run, not only its main pass.

The in-run plane finalize built its never-rip set from the raw protection map
(`PROTECTED net(s) excluded from blocker rip-up`), its nested reroute sub-run
filtered the net out of pre-existing rip candidacy again, and the failure
hints still said "The router will NEVER rip these" -- so a plane tap was
refused on a net the run had made rippable, and the improvement gate rejected
the step (the board shipped its bare pour).

Unit half: `protected_nets.effective_protection_map` is the protection map
minus the stashed exact-name overrides, and never lifts 'locked' / `sense:`.

Integration half: the repo's own plane chain (bare pour, then ONE route step
whose in-run finalize taps the planes). With a signal net recorded as
protected in the project and named exactly in --rip-existing-nets, the
finalize must rip it for the tap instead of refusing it as protected.

    python3 tests/test_1395_rip_override_reaches_finalize.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']
NET = '/T8F49I2X/PIN.21'


def unit_half():
    import protected_nets as pn

    class _Net:
        def __init__(self, name):
            self.name = name

    class _P:
        nets = {1: _Net('A'), 2: _Net('B'), 3: _Net('C')}
        segments, vias = [], []
        source_path = None

    pcb = _P()
    base = {'A': 'user', 'B': 'locked', 'C': 'length-matched'}
    orig = pn.protection_map
    pn.protection_map = lambda pcb_data, input_file=None: dict(base)
    try:
        assert pn.effective_protection_map(pcb) == base, 'nothing stashed'
        pn.stash_rip_overrides(pcb, ['A', 'B', 'C*'])   # C* is a glob: no override
        got = pn.effective_protection_map(pcb)
        assert got == {'B': 'locked', 'C': 'length-matched'}, got
    finally:
        pn.protection_map = orig


def integration_half():
    evidence(BOARD, 'plane-chain board')
    tmp = tempfile.mkdtemp(prefix='t1395_')
    try:
        r0 = os.path.join(tmp, 'r0.kicad_pcb')
        r1 = os.path.join(tmp, 'r1.kicad_pcb')
        b = os.path.join(tmp, 'B.kicad_pcb')
        check(PY + [tool('copy_board.py'), BOARD, r0], accept=True)
        check(PY + [tool('route_planes.py'), r0, r1, '--nets', 'GND', '+3V3',
                    '--plane-layers', 'In1.Cu', 'In4.Cu', '--via-size', '0.45',
                    '--via-drill', '0.2', '--track-width', '0.09',
                    '--clearance', '0.10', '--hole-to-hole-clearance', '0.2',
                    '--grid-step', '0.05', '--power-nets', 'VIN',
                    '--power-nets-widths', '0.3'], accept=True)
        check(PY + [tool('copy_board.py'), r1, b], accept=True)
        pro = os.path.join(tmp, 'B.kicad_pro')
        with open(evidence(pro, 'project'), encoding='utf-8') as f:
            d = json.load(f)
        d.setdefault('kicad_routing_tools', {})['protected_nets'] = {
            NET: 'length-matched'}
        with open(pro, 'w', encoding='utf-8') as f:
            json.dump(d, f, indent=2)
        r = check(PY + [tool('route.py'), b, os.path.join(tmp, 'out.kicad_pcb'),
                        '--nets', '+1V1', '/T8F49I2X/PIN.5', 'GND', '+3V3',
                        '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu',
                        'In4.Cu', 'B.Cu', '--no-bga-zones', '--clearance',
                        '0.09', '--track-width', '0.0762', '--via-size',
                        '0.25', '--via-drill', '0.15',
                        '--hole-to-hole-clearance', '0.2', '--grid-step',
                        '0.025', '--max-ripup', '10', '--max-iterations',
                        '1000000', '--rip-existing-nets', NET], accept=True,
                  timeout=1500)
        out = r.stdout + (r.stderr or '')
        assert re.search(r'eligible for rip-up: ' + re.escape(NET), out), \
            'the main pass did not lift the override (broken test)'
        for bad in ('PROTECTED net(s) excluded from blocker rip-up',
                    'PROTECTED net(s) excluded from pre-existing rip candidacy',
                    'The router will NEVER rip these',
                    NET + ' (protected, cannot rip)'):
            assert bad not in out, f'the override was ignored: {bad!r}'
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    unit_half()
    print('unit half ok')
    integration_half()
    print('PASS')
