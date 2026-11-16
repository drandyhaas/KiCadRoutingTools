#!/usr/bin/env python3
"""#1762: a length-match group whose members are all already connected
records every member's verdict and names --force-reroute.

route.py drops a connected net before routing (filter_already_routed), so a
later `route.py --nets P N X --length-match-group P N` routed neither member,
printed "Length matching group: no nets routed this run, skipping", wrote no
`member_status` (docs/length-matching.md promises one per member) and named
no remedy, while the pair shipped 0.500 mm out of its 0.127 mm group.

The board is run32's placed_v2; step 1 routes the pair single-ended (P 8.625,
N 9.125 mm), step 2 names it in a group alongside a net that still routes.

Rows:
  1. step 2 exits 0 (disclosed, not refused);
  2. its JSON_SUMMARY group carries member_status: N `matched`, P
     `unmatched_board_copper` with its shortfall;
  3. the skip lines -- route.py's and the matcher's -- name --force-reroute
     with the group's members;
  4. the unmatched group is NOT recorded protected (#521), so a later glob
     --force-reroute may still take it.

    python3 tests/test_1762_board_copper_group_status.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []
P, N = '/IO_Banks/Z11_P', '/IO_Banks/Z11_N'
L = ['--no-bga-zones', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def py(*argv):
    return run_check([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                     accept=True)


def main():
    tmp = tempfile.mkdtemp(prefix='t1762_')
    try:
        inp, se, lm = (os.path.join(tmp, f'{t}.kicad_pcb') for t in ('in', 'se', 'lm'))
        py(tool('copy_board.py'),
           os.path.join(ROOT, 'tests', 'fixtures', 'run32', 'placed_v2.kicad_pcb'), inp)
        py(tool('route.py'), evidence(inp, 'fixture copy'), se, '--nets', P, N, *L)
        print("1. the length-match step over the connected pair")
        r = py(tool('route.py'), evidence(se, 'pair routed single-ended'), lm,
               '--nets', P, N, '/IO_Banks/Z0_P', '--length-match-group', P, N,
               '--length-match-tolerance', '0.127', *L)
        log = r.stdout + r.stderr
        if 'Skipping 2 already-routed net(s)' not in log:
            print("  [BROKEN TEST] step 2 did not drop the pair as connected: the "
                  "fixture does not reach the all-board-copper path")
            return 1
        check("step 2 exits 0", r.returncode == 0)
        line = next((l for l in log.splitlines() if l.startswith('JSON_SUMMARY:')), '')
        groups = json.loads(line.split(':', 1)[1]).get('length_match_groups') if line else []
        ms = (groups or [{}])[0].get('member_status') or {}
        print("2. member_status")
        check("N is matched (it sets the target)", (ms.get(N) or {}).get('status') == 'matched',
              str(ms.get(N)))
        sp = ms.get(P) or {}
        check("P is unmatched_board_copper with its shortfall",
              sp.get('status') == 'unmatched_board_copper'
              and abs((sp.get('shortfall_mm') or 0) - 0.5) < 0.01, str(sp))
        print("3. the skip lines name the re-route")
        want = f"--force-reroute and --length-match-group '{P}' '{N}'"
        check("route.py's already-connected NOTE names it",
              any(l.startswith('NOTE:') and '--length-match-group member(s)' in l
                  and want in l for l in log.splitlines()))
        check("the matcher's skip line names it",
              any('Length matching group: no member was routed this run' in l
                  and want in l for l in log.splitlines()))
        print("4. no protection for an unmatched group")
        pro = os.path.join(tmp, 'lm.kicad_pro')
        with open(evidence(pro, 'output project'), encoding='utf-8') as fh:
            prot = (json.load(fh).get('kicad_routing_tools') or {}).get('protected_nets') or {}
        check("neither member is recorded protected", P not in prot and N not in prot,
              str(prot))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
