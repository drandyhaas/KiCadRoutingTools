#!/usr/bin/env python3
"""#1670: diagnose-routing-failures retries with the step's own `--nets`, and
says when naming nets is right, what it gives up, and when a retry is wrong.

A named retry and a wildcard retry attempt the same nets, because route.py skips
what is connected, but they differ in rip authority: route.py's reconciliation
keeps a hinted blocker of any size inside `--nets` and drops a large or poured
one outside them. Step 4 retried "only the failed nets" and plan-pcb-routing says
to apply it; on a four-layer board at thin geometry that left the net open where
the wildcard closed it, and the whole step gave the best board.

Checks: the mechanism the skills cite is the one route.py's source has, and the
skills say it, once each. Static.

    python3 -X utf8 tests/test_1670_retry_scope.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIAG = os.path.join(ROOT, '.claude', 'skills', 'diagnose-routing-failures', 'SKILL.md')
PLAN = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')
ROUTE = os.path.join(ROOT, 'py_router', 'route.py')
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def flat(path):
    with open(path, encoding='utf-8') as f:
        return re.sub(r'\s+', ' ', f.read())


def main():
    src = flat(ROUTE)
    print('A. the mechanism the skills cite')
    check('a blocker inside --nets is kept at any size', 'if _mnf(_bn, net_names): _keep103.append(_bn)' in src)
    check('one with a pour outside --nets is dropped', 'if _bid is None or _bid in _zone_ids103: continue' in src)
    check('so is one over 30 segments or 6 vias', 'if _bsegs > 30 or _bvias > 6: continue' in src)

    d = flat(DIAG)
    step4 = d.split('## Step 4: Output the Retry Command', 1)[1]
    print('B. diagnose-routing-failures Step 4')
    check('it no longer retries only the failed nets', 'retries **only the failed nets**' not in d)
    check('the speed claim is gone', 'much faster than a full re-run' not in d)
    check('it retries with the step\'s own selection', "the failing step's own `--nets` selection" in step4)
    route_cmd = step4.split('```bash', 1)[1].split('```', 1)[0]
    check('its example command is a wildcard', '--nets "*"' in route_cmd, route_cmd[:120])
    check('it says what naming gives up', 'with less rip authority' in step4
          and 'at most 30 segments and 6 vias' in step4)
    check('it says when naming is right', 'Name nets only when naming is the point' in step4
          and "#1399's reason" in step4)
    check('a retry that names nets adds the poured nets', 'a retry that names nets adds the poured nets too' in step4)
    check('thin geometry re-routes the whole step',
          "re-route the whole step from that step's input instead of retrying" in step4)
    print('C. its capacity row')
    row = next((ln for ln in open(DIAG, encoding='utf-8') if 'Genuine capacity problem' in ln), '')
    check('it spells the ceiling', '`--clearance-ceiling`' in row and '`--clearance`/' not in row, row[:120])
    check('it re-routes the whole step', 're-routing the whole step rather than retrying (Step 4)' in row)
    print('D. the pair retry still names its members')
    pair = step4.split('**For a pair step, the retry is a `route_diff.py` call**', 1)[1]
    check('the pair example names them', '--nets "/USB_D+" "/USB_D-"' in pair)

    p = flat(PLAN)
    print('E. plan-pcb-routing')
    check('the ordinary retry gives the rip-authority reason',
          'it keeps the run\'s rip authority' in p and 'An ordinary retry should use `--nets \'*\'`' in p)
    check('the manifest reason stays', 'a manifest that names nets cannot be A/B\'d afterwards' in p)


if __name__ == '__main__':
    main()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print(f'  - {f}')
        sys.exit(1)
    print('ALL PASS')
