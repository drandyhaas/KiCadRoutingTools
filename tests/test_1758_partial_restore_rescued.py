#!/usr/bin/env python3
"""#1758 follow-up: the per-net rescue takes a net put back as a partial restore.

A partial restore (route.py's #134 last resort, and since #1758 the casualty
reconcile's branch D) becomes the net's result in routed_results, stamped
`partial_restore_134`, "PARTIAL for a later pass". It carries no
`failed_pads_info`, and `rescue_failed_nets` picked its candidates by that key
alone -- a net with no result was 'failed', a result with a pad debt
'partial' -- so a partial restore was neither, and the later pass it was left
for never saw it.

Measured on ottercast_audio's route step (sets 1-10 A/B of the #1755-#1775
batch): branch D put back 9 pieces of BT_UART_CTS and of Net-(R82-Pad1). Before
#1758 the restore was no result, so the rescue took both as 'failed' (30
candidates) and recovered BT_UART_CTS; once #1758 registered the restore the
rescue had 28 candidates, neither of them, and BT_UART_CTS shipped in two
pieces in the cloud replay.

Rows (rescue_candidates, the rescue's candidate selection):
  1. a net with no result is 'failed';
  2. a result with a pad debt is 'partial';
  3. a partial restore with no pad debt is 'partial' (the fix);
  4. a finished result, and a net with fewer than two pads, are not candidates;
  5. the rescue and the cleanup's protect set (route.py _unfinished_net_ids)
     read a result's unfinished state the same way.

    python3 tests/test_1758_partial_restore_rescued.py
"""
import os
import sys
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from net_rescue import rescue_candidates                         # noqa: E402
from route import _unfinished_net_ids                            # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def main():
    names = {1: '/NONE', 2: '/DEBT', 3: '/RESTORE', 4: '/DONE', 5: '/ONEPAD'}
    pcb = SimpleNamespace(
        nets={i: SimpleNamespace(name=n) for i, n in names.items()},
        pads_by_net={1: [0, 1], 2: [0, 1], 3: [0, 1], 4: [0, 1], 5: [0]})
    restore = {'net_id': 3, 'new_segments': ['s'], 'new_vias': [],
               'partial_restore_134': True}
    routed = {
        2: {'net_id': 2, 'new_segments': ['s'],
            'failed_pads_info': [{'component_ref': 'U1', 'pad_number': '1'}]},
        3: restore,
        4: {'net_id': 4, 'new_segments': ['s'], 'failed_pads_info': []},
        5: None,
    }
    routed = {k: v for k, v in routed.items() if v is not None}
    scope = [(n, i) for i, n in names.items()]
    got = {nid: kind for _nm, nid, kind in rescue_candidates(pcb, routed, scope)}
    print(f"candidates: { {names[k]: v for k, v in got.items()} }")

    check("1. a net with no result is 'failed'", got.get(1) == 'failed', str(got.get(1)))
    check("2. a result with a pad debt is 'partial'", got.get(2) == 'partial',
          str(got.get(2)))
    check("3. a partial restore with no pad debt is 'partial'",
          got.get(3) == 'partial', str(got.get(3)))
    check("4. a finished result and a one-pad net are not candidates",
          4 not in got and 5 not in got, str(sorted(got)))

    unfinished = _unfinished_net_ids(routed, scope)
    check("5. the rescue attempts every unfinished net the cleanup protects",
          {i for i in unfinished if len(pcb.pads_by_net[i]) >= 2} == set(got),
          f"protected {sorted(unfinished)}, rescued {sorted(got)}")

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
