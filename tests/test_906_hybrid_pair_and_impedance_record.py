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

2. A `net_impedance` spec was recorded as if achieved even when no width was
   ever solved -- the router falls back to the plain track width, so the record
   described copper that was never drawn and `check_impedance` auto-reads it and
   grades those nets against an impedance the router never attempted.
   `route.py` carried the identical asymmetry for single-ended nets.

   THE OBVIOUS FIX IS WRONG, and the first cut of this file shipped it. The
   issue proposes gating the record on the board having a stackup, like the
   reapply branch. That is a PROXY: a stackup listing copper with no adjacent
   dielectric solves nothing, every layer falls back, and the spec is recorded
   anyway -- measured on a board whose only edit was deleting one dielectric
   line. It also breaks the workflow the record exists for ("route now, add the
   stackup, re-run without --impedance and let it recompute"), which the very
   paragraph of docs/length-matching.md it edits still promises, and it silently
   changed shipped copper: route.py's smoothing skip-list reads the same in-run
   note, so not recording let the #536 pass rewrite those nets.

   So: the DECLARATION is always recorded, and carries `applied`. False means no
   layer width was solved. `check_impedance` skips an unapplied declaration; the
   reapply branch still finds it. `hollow` below is the stackup-present,
   unsolvable case the proxy gets wrong.

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
    """Scoped honestly: this walks EVERY py_router module, not one file, and it
    finds dict LITERALS plus `x['hybrid_escape'] = ...` stores, because a
    producer written the second way would otherwise pass unseen.

    It deliberately does NOT claim anything about the callers' rejecting paths
    -- an earlier draft captioned "nor a failed key" as "it is never built for
    a failure", which is a statement about call sites the walk never inspects,
    and it was vacuous: a success-path literal has no `failed` key by
    construction. The accepted-COMPROMISE path that claim would have had to
    cover is the self-graze fallback, and it is arm 1b below that covers it.
    """
    print('\n-- 0. the producer, read from the source --')
    found, stores = [], []
    for root, _dirs, names in os.walk(os.path.join(ROOT, 'py_router')):
        if '__pycache__' in root:
            continue
        for name in names:
            if not name.endswith('.py'):
                continue
            path = os.path.join(root, name)
            tree = ast.parse(open(path, encoding='utf-8').read())
            for node in ast.walk(tree):
                if isinstance(node, ast.Dict):
                    keys = [k.value for k in node.keys
                            if isinstance(k, ast.Constant)
                            and isinstance(k.value, str)]
                    if 'hybrid_escape' in keys:
                        found.append((name, keys))
                elif (isinstance(node, ast.Subscript)
                      and isinstance(node.slice, ast.Constant)
                      and node.slice.value == 'hybrid_escape'
                      and isinstance(getattr(node, 'ctx', None), ast.Store)):
                    stores.append(name)
    check('the walk found the producer at all (non-vacuity)', bool(found),
          found)
    check('exactly one dict literal in the whole engine carries hybrid_escape',
          len(found) == 1, found)
    check('and nothing assigns the key separately',
          stores == [], stores)
    check('the literal carries NO is_diff_pair -- so the shape test in the '
          '#521 loop could not see it',
          found and 'is_diff_pair' not in found[0][1], found)


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
    # 1b. The accepted COMPROMISE, which is not a failure and so has no
    # `failed` key: the self-graze fallback keeps the least-bad candidate when
    # no layer couples cleanly, and ships P/N copper BELOW clearance.
    # Protecting it would make the chain step whose job is to fix that skip the
    # pair. The producer marks it; this asserts the marker is honoured AND that
    # the marker is really what the producer writes.
    check('a self-grazing hybrid is NOT protected',
          protection_candidates({1: dict(hybrid, selfgraze=3)}, pcb) == {})
    prod = open(os.path.join(ROOT, 'py_router', 'diff_pair_routing.py'),
                encoding='utf-8').read()
    check("and the producer really writes that marker on the fallback path",
          "['selfgraze'] = _selfgraze_fallback[0]" in prod)


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
def _synth(td, name, stackup='full'):
    """`full` solves; `none` has no stackup block; `hollow` KEEPS the block but
    deletes its dielectric line -- copper with nothing between it, which the
    model cannot solve. `hollow` is the case a stackup-PRESENCE gate calls
    computable and gets wrong."""
    p = os.path.join(td, name + '.kicad_pcb')
    write_synth_board(p)
    txt = open(p, encoding='utf-8').read()
    if stackup == 'none':
        i = txt.index('\t\t(stackup')
        j = txt.index('\t\t)\n', i) + len('\t\t)\n')
        txt = txt[:i] + txt[j:]
        open(p, 'w', encoding='utf-8').write(txt)
        check(f'[{name}] the stackup block was really removed',
              '(stackup' not in txt)
    elif stackup == 'hollow':
        lines = [l for l in txt.split('\n') if '"dielectric 1"' not in l]
        txt = '\n'.join(lines)
        open(p, 'w', encoding='utf-8').write(txt)
        check(f'[{name}] the block survives but its dielectric is gone',
              '(stackup' in txt and 'dielectric 1' not in txt)
    return p


def _specs_of(out):
    pro = pro_path_for_board(out)
    # A missing project also reads {}, so an arm asserting "no spec" must prove
    # the project EXISTS -- otherwise a crashed route satisfies it for free.
    return (os.path.isfile(pro), read_impedance_specs(pro) if os.path.isfile(pro) else {})


def test_an_unsolvable_impedance_is_recorded_as_not_applied():
    print('\n-- 3. --impedance where no width can be solved --')
    for label, kind, tool, nets, ohms in (
            ('no stackup at all', 'none', 'route_diff.py', ['DP_A_P', 'DP_A_N'], '90'),
            ('a stackup with no dielectric', 'hollow', 'route_diff.py',
             ['DP_A_P', 'DP_A_N'], '90'),
            ('route.py, no stackup', 'none', 'route.py', ['SE1'], '50'),
            ('route.py, hollow stackup', 'hollow', 'route.py', ['SE1'], '50')):
        with tempfile.TemporaryDirectory() as td:
            src = _synth(td, kind + tool[:5], stackup=kind)
            out = os.path.join(td, 'r.kicad_pcb')
            txt = _run(['py_router/' + tool, src, '--nets', *nets,
                        '--impedance', ohms, '--output', out])
            exists, specs = _specs_of(out)
            check(f'[{label}] the project was written (the arm is not vacuous)',
                  exists)
            check(f'[{label}] the declaration is KEPT -- a later step with a '
                  f'usable stackup must be able to recompute it',
                  all(float(s.get('ohms', 0)) == float(ohms)
                      for s in specs.values()) and specs, specs)
            check(f'[{label}] and it is recorded as NOT applied',
                  specs and all(s.get('applied') is False
                                for s in specs.values()), specs)
            check(f'[{label}] and the run said so', 'NOT APPLIED' in txt,
                  txt[-400:])


def test_check_impedance_does_not_grade_an_unapplied_declaration():
    print('\n-- 4. the consumer the record misled --')
    with tempfile.TemporaryDirectory() as td:
        src = _synth(td, 'hollow_audit', stackup='hollow')
        out = os.path.join(td, 'r.kicad_pcb')
        _run(['py_router/route_diff.py', src, '--nets', 'DP_A_P', 'DP_A_N',
              '--impedance', '90', '--output', out])
        txt = _run(['py_tools/check_impedance.py', out])
        check('check_impedance skips it and says so',
              'NOT APPLIED' in txt and 'skipped' in txt, txt[-500:])
        check('and it grades ZERO declarations from that project',
              'Auto-read 0 net impedance' in txt, txt[-500:])


def test_a_solvable_board_still_records_applied():
    print('\n-- 5. control: a stackup that really solves --')
    for tool, nets, ohms in (('route_diff.py', ['DP_A_P', 'DP_A_N'], 90),
                             ('route.py', ['SE1'], 50)):
        with tempfile.TemporaryDirectory() as td:
            src = _synth(td, 'ok' + tool[:5], stackup='full')
            out = os.path.join(td, 'r.kicad_pcb')
            txt = _run(['py_router/' + tool, src, '--nets', *nets,
                        '--impedance', str(ohms), '--output', out])
            _, specs = _specs_of(out)
            check(f'[{tool}] the spec is recorded',
                  any(float(s.get('ohms', 0)) == ohms for s in specs.values()),
                  specs)
            check(f'[{tool}] and marked APPLIED',
                  specs and all(s.get('applied') is True
                                for s in specs.values()), specs)
            check(f'[{tool}] the run did NOT claim it was unapplied',
                  'NOT APPLIED' not in txt)


def main():
    test_the_real_hybrid_result_carries_no_is_diff_pair()
    test_protection_candidates_admits_the_hybrid()
    test_an_unsolvable_impedance_is_recorded_as_not_applied()
    test_check_impedance_does_not_grade_an_unapplied_declaration()
    test_a_solvable_board_still_records_applied()
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
