#!/usr/bin/env python3
"""#906: what route_diff records in the project, and what it must not.

TWO record gaps, landed together because both are route_diff's project-record
writing.

1. A pair routed by the DIRECT HYBRID escape was never recorded as protected
   (#521), so the next `route.py` lap treated it as ordinary pre-existing copper
   and rewrote it. The #521 candidate loop tested the result dict's SHAPE
   (`is_diff_pair`), and the hybrid result deliberately does not carry that key
   -- it means "a COUPLED constructor committed this", which `diff_pair_custody`
   reads to decide whether a 'partial' pair kept a coupled trunk. Its neighbour
   45 lines up (#766) had already diagnosed exactly this about the SAME dict and
   moved off the shape test; the protection loop was left on it.

   Measured, run 25 (/D_P /D_N): "DIRECT HYBRID" in the log, no "Protected nets
   ... recorded" line; the next lap registered 2 unprotected pre-existing nets
   as rip candidates and smoothing collapsed 16 spans / 10 nets INCLUDING the
   pair. After a hand `persist_protected_nets`: 14 spans / 8 nets, pair intact.
   The #521 mechanism worked; its recording gate is what failed.

2. A `net_impedance` spec was recorded even when no stackup made it computable.
   With no stackup `layer_widths` stays empty, the config never receives
   `layer_widths`/`impedance_target`, and the pair routes at the plain track
   width -- so the record described copper that was never drawn, the reapply
   branch (stackup-gated) could never use it, and `check_impedance` auto-reads
   it and grades those nets against an impedance the router never attempted.
   `route.py` carried the identical asymmetry for single-ended nets; both are
   gated here.

ARM B IS THE SLOW ONE and it is the only one that proves the hybrid path is
really taken. It asserts "HYBRID" in the router's own output BEFORE asserting
the record -- a hybrid test that quietly took the coupled path would otherwise
pass for free, which is the vacuity this repo has been bitten by.
"""
import ast
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 1800

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import run_utils                                              # noqa: E402
from route_diff import protection_candidates                  # noqa: E402
from protected_nets import read_protected_nets, read_impedance_specs, \
    pro_path_for_board                                        # noqa: E402
from test_meander_demo_boards import write_synth_board        # noqa: E402

fails = []


def check(label, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}"
          + (f"   [{detail}]" if not ok and detail != '' else ''))
    if not ok:
        fails.append(label)


class _Net:
    def __init__(self, name):
        self.name = name


class _PCB:
    def __init__(self, names):
        self.nets = {i: _Net(n) for i, n in enumerate(names, 1)}


# --------------------------------------------------------------------------
# 0. ON THE BRANCH: the real hybrid producer really omits `is_diff_pair`.
#    Without this, arm 1 is a test of a dict this file invented.
# --------------------------------------------------------------------------
def test_the_real_hybrid_result_carries_no_is_diff_pair():
    print('\n-- 0. the producer, read from the source --')
    src = open(os.path.join(ROOT, 'py_router', 'diff_pair_routing.py'),
               encoding='utf-8').read()
    found = []
    for node in ast.walk(ast.parse(src)):
        if not isinstance(node, ast.Dict):
            continue
        keys = [k.value for k in node.keys
                if isinstance(k, ast.Constant) and isinstance(k.value, str)]
        if 'hybrid_escape' in keys:
            found.append(keys)
    check('exactly one dict literal carries hybrid_escape', len(found) == 1,
          found)
    check('and it carries NO is_diff_pair -- so the shape test could not see it',
          found and 'is_diff_pair' not in found[0], found)
    check('nor a failed key (it is never built for a failure)',
          found and 'failed' not in found[0], found)


def test_protection_candidates_admits_the_hybrid():
    print('\n-- 1. protection_candidates --')
    pcb = _PCB(['/D_P', '/D_N', '/SE'])
    hybrid = {'new_segments': [1], 'new_vias': [], 'iterations': 3,
              'path_length': 9, 'hybrid_escape': True}
    coupled = {'is_diff_pair': True, 'p_net_id': 1, 'n_net_id': 2,
               'new_segments': [1]}
    got = protection_candidates({1: hybrid, 2: hybrid}, pcb)
    check('a hybrid pair is recorded, both members',
          got == {'/D_P': 'diff-pair', '/D_N': 'diff-pair'}, got)
    check('a coupled pair still is',
          protection_candidates({1: coupled, 2: coupled}, pcb)
          == {'/D_P': 'diff-pair', '/D_N': 'diff-pair'})
    check('a FAILED result is not',
          protection_candidates({1: dict(hybrid, failed=True)}, pcb) == {})
    check('a single-ended result is not',
          protection_candidates({3: {'new_segments': [1]}}, pcb) == {})
    check('an empty result is not',
          protection_candidates({1: None, 2: {}}, pcb) == {})
    check('a net id the board does not have is not',
          protection_candidates({99: hybrid}, pcb) == {})


# --------------------------------------------------------------------------
# 2. END TO END on the board that really takes the hybrid path.
# --------------------------------------------------------------------------
def _run(argv, verbose=False):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + argv, cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    return r.stdout + r.stderr


def test_a_hybrid_pair_is_recorded_in_the_project():
    print('\n-- 2. watchy USB_D, the known hybrid pair, end to end --')
    board = os.path.join(ROOT, 'kicad_files', 'watchy.kicad_pcb')
    run_utils.evidence(board, 'the watchy board')
    td = tempfile.mkdtemp()
    fan = os.path.join(td, 'fan.kicad_pcb')
    out = os.path.join(td, 'diff.kicad_pcb')
    # The SAME chain tests/test_watchy_diff_hybrid_escape.py runs -- that file
    # is what established this board takes the hybrid path, and inventing my
    # own fanout arguments produced no board at all on the first attempt.
    _run(['py_router/qfn_fanout.py', board, '--component', 'U4',
          '--nets', '*', '!GND', '!+3V3', '--width', '0.1', '--output', fan])
    if not (os.path.exists(fan) and os.path.getsize(fan) > 0):
        check('qfn_fanout produced a board', False)
        return
    txt = _run(['py_router/route_diff.py', fan, '--nets', 'USB_D+', 'USB_D-',
                '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
                '--track-width', '0.1', '--diff-pair-gap', '0.15',
                '--clearance', '0.1', '--via-size', '0.3', '--via-drill', '0.2',
                '--no-gnd-vias', '--grid-step', '0.05', '--output', out])
    # ON THE BRANCH FIRST: a run that took the COUPLED path would satisfy the
    # record assertion below without ever exercising the fix.
    check('the router really took the hybrid path', 'HYBRID' in txt,
          txt[-400:])
    m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", txt)
    summary = json.loads(m.group(1)) if m else {}
    check('the pair routed 1/1', summary.get('successful') == 1
          and not summary.get('failed'), summary.get('failed'))
    pro = pro_path_for_board(out)
    check('a sibling project exists to record into', os.path.isfile(pro))
    prot = read_protected_nets(pro) if os.path.isfile(pro) else {}
    check('BOTH members are recorded protected, reason diff-pair',
          prot.get('USB_D+') == 'diff-pair' and prot.get('USB_D-') == 'diff-pair',
          prot)
    check('and the run SAID so (a silent record is how this was missed)',
          'Protected nets' in txt, txt[-300:])
    shutil.rmtree(td, ignore_errors=True)


# --------------------------------------------------------------------------
# 3. The impedance record, both fronts.
# --------------------------------------------------------------------------
def _synth(td, name, stackup=True):
    p = os.path.join(td, name + '.kicad_pcb')
    write_synth_board(p)
    if not stackup:
        txt = open(p, encoding='utf-8').read()
        i = txt.index('\t\t(stackup')
        j = txt.index('\t\t)\n', i) + len('\t\t)\n')
        open(p, 'w', encoding='utf-8').write(txt[:i] + txt[j:])
        check('the stackup was really removed', '(stackup' not in
              open(p, encoding='utf-8').read())
    return p


def test_no_impedance_record_without_a_stackup():
    print('\n-- 3. --impedance on a board with no stackup --')
    with tempfile.TemporaryDirectory() as td:
        src = _synth(td, 'nostack', stackup=False)
        out = os.path.join(td, 'r.kicad_pcb')
        txt = _run(['py_router/route_diff.py', src, '--nets', 'DP_A_P',
                    'DP_A_N', '--impedance', '90', '--output', out])
        check('the router said the board has no stackup',
              'No stackup' in txt, txt[-300:])
        specs = read_impedance_specs(pro_path_for_board(out)) \
            if os.path.isfile(pro_path_for_board(out)) else {}
        check('no net_impedance spec was recorded', specs == {}, specs)
        check('and it said why', 'not recording' in txt, txt[-300:])

    print('\n-- 4. control: the same call WITH a stackup still records --')
    with tempfile.TemporaryDirectory() as td:
        src = _synth(td, 'stack', stackup=True)
        out = os.path.join(td, 'r.kicad_pcb')
        _run(['py_router/route_diff.py', src, '--nets', 'DP_A_P', 'DP_A_N',
              '--impedance', '90', '--output', out])
        specs = read_impedance_specs(pro_path_for_board(out)) \
            if os.path.isfile(pro_path_for_board(out)) else {}
        check('the spec IS recorded when it was computable',
              any(float(s.get('ohms', 0)) == 90 for s in specs.values()), specs)


def test_the_single_ended_twin_is_gated_too():
    print('\n-- 5. route.py --impedance, the same asymmetry --')
    with tempfile.TemporaryDirectory() as td:
        src = _synth(td, 'nostack_se', stackup=False)
        out = os.path.join(td, 'r.kicad_pcb')
        txt = _run(['py_router/route.py', src, '--nets', 'SE1',
                    '--impedance', '50', '--output', out])
        specs = read_impedance_specs(pro_path_for_board(out)) \
            if os.path.isfile(pro_path_for_board(out)) else {}
        check('no net_impedance spec on a stackup-less board', specs == {},
              specs)
        check('and it said why', 'not recording' in txt, txt[-400:])
    with tempfile.TemporaryDirectory() as td:
        src = _synth(td, 'stack_se', stackup=True)
        out = os.path.join(td, 'r.kicad_pcb')
        _run(['py_router/route.py', src, '--nets', 'SE1', '--impedance', '50',
              '--output', out])
        specs = read_impedance_specs(pro_path_for_board(out)) \
            if os.path.isfile(pro_path_for_board(out)) else {}
        check('control: it IS recorded with a stackup',
              any(float(s.get('ohms', 0)) == 50 for s in specs.values()), specs)


def main():
    test_the_real_hybrid_result_carries_no_is_diff_pair()
    test_protection_candidates_admits_the_hybrid()
    test_no_impedance_record_without_a_stackup()
    test_the_single_ended_twin_is_gated_too()
    test_a_hybrid_pair_is_recorded_in_the_project()
    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s) failed: {fails}")
        return 1
    print('PASS: a hybrid-escaped pair is recorded protected, and no impedance '
          'spec is recorded for copper that was never drawn (#906)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
