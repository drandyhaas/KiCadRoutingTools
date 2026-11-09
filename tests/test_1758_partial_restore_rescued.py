#!/usr/bin/env python3
"""#1758 follow-up: the per-net rescue takes a casualty put back as a partial
restore, after every other candidate.

The casualty reconcile's branch D (diff_pair_custody.run_casualty_reconcile)
puts back the pieces of a ripped net's pre-rip route that collide with
nothing, "PARTIAL for a later pass". Since #1758 that restore is the net's
result in routed_results, stamped `partial_restore_134`, with no
`failed_pads_info`. `rescue_failed_nets` picked its candidates by that key
alone -- a net with no result was 'failed', a result with a pad debt
'partial' -- so the restore was neither, and the pass it was left for never
saw it. Before #1758 the restore was no result, and the rescue took the net as
'failed': on ottercast_audio's route step it recovered BT_UART_CTS, which
after #1758 shipped in two pieces in the cloud replay.

Rows (rescue_candidates, the rescue's candidate selection and order):
  1. a net with no result is 'failed';
  2. a result with a pad debt is 'partial';
  3. a casualty's partial restore with no pad debt is 'partial' (the fix);
  4. a finished result, and a net with fewer than two pads, are not candidates;
  5. route.py's #134 last-resort restore is not a candidate, as it has not been
     since #508 finding 8. Taking it too (b4be70fd) put 22 more nets ahead of
     the final reconciliation on cparti_fpga's route step;
  6. a casualty's restore is attempted after every other candidate, which keep
     their name order. The rescue's copper is additive and each net's windows
     see the copper of the nets before it: taken in name order (b4be70fd),
     ottercast's WL_SDIO_D1 went from reconnected to improved behind
     UART1_RX's rescue, and on ulx3s seven nets the rescue used to reconnect
     failed once the restores were taken in among them;
  7. the casualty reconcile stamps its restore `casualty_restore` and route.py's
     #134 last resort does not (source anchors: the stamp is the only thing
     that tells the two restores apart).

    python3 tests/test_1758_partial_restore_rescued.py
"""
import os
import re
import sys
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from net_rescue import rescue_candidates                         # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def _read(rel):
    with open(os.path.join(ROOT, rel), encoding='utf-8') as fh:
        return fh.read()


def main():
    names = {1: '/NONE', 2: '/DEBT', 3: '/RESTORE', 4: '/DONE', 5: '/ONEPAD',
             6: '/A_RESTORE', 7: '/Z_NONE', 8: '/LAST_RESORT'}
    pcb = SimpleNamespace(
        nets={i: SimpleNamespace(name=n) for i, n in names.items()},
        pads_by_net={1: [0, 1], 2: [0, 1], 3: [0, 1], 4: [0, 1], 5: [0],
                     6: [0, 1], 7: [0, 1], 8: [0, 1]})
    restore = {'net_id': 3, 'new_segments': ['s'], 'new_vias': [],
               'partial_restore_134': True, 'casualty_restore': True}
    routed = {
        2: {'net_id': 2, 'new_segments': ['s'],
            'failed_pads_info': [{'component_ref': 'U1', 'pad_number': '1'}]},
        3: restore,
        4: {'net_id': 4, 'new_segments': ['s'], 'failed_pads_info': []},
        6: dict(restore, net_id=6),
        8: {'net_id': 8, 'new_segments': ['s'], 'new_vias': [],
            'partial_restore_134': True},
    }
    scope = [(n, i) for i, n in names.items()]
    rows = rescue_candidates(pcb, routed, scope)
    got = {nid: kind for _nm, nid, kind in rows}
    print(f"candidates: { {names[k]: v for k, v in got.items()} }")

    check("1. a net with no result is 'failed'", got.get(1) == 'failed', str(got.get(1)))
    check("2. a result with a pad debt is 'partial'", got.get(2) == 'partial',
          str(got.get(2)))
    check("3. a casualty's partial restore with no pad debt is 'partial'",
          got.get(3) == 'partial' and got.get(6) == 'partial',
          f"{got.get(3)}, {got.get(6)}")
    check("4. a finished result and a one-pad net are not candidates",
          4 not in got and 5 not in got, str(sorted(got)))
    check("5. a #134 last-resort restore is not a candidate", 8 not in got,
          str(got.get(8)))

    order = [nm for nm, _nid, _kind in rows]
    check("6. a casualty's restore comes last, the rest in name order",
          order == ['/DEBT', '/NONE', '/Z_NONE', '/A_RESTORE', '/RESTORE'],
          str(order))

    custody = _read('py_router/diff_pair_custody.py')
    route = _read('py_router/route.py')
    stamp_d = re.search(r"pruned\['partial_restore_134'\] = True\s*\n"
                        r"(?:\s*#[^\n]*\n)*\s*pruned\['casualty_restore'\] = True",
                        custody)
    check("7. branch D stamps casualty_restore, the #134 last resort does not",
          bool(stamp_d) and "casualty_restore" not in route
          and route.count("pruned['partial_restore_134'] = True") == 1,
          f"branch D stamp {'found' if stamp_d else 'MISSING'}; route.py "
          f"mentions casualty_restore: {'casualty_restore' in route}")

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
