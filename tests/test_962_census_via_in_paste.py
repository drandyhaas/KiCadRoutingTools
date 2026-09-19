#!/usr/bin/env python3
"""#962 D6: the PR's via-in-paste census, pinned so the claim expires loudly.

The PR states where check_drc's new `via-in-paste` violation fires on the
tracked corpus (`run_utils.corpus_boards`). The count is of human-routed vias
in a paste opening of their own net that are not filled+capped:
- orangecrab_ext_pll: 136;
- routed_output: 339;
- rp2350_fpga_eensy_prePlane: 27;
- every other tracked board: 0.

No tracked board declares filled+capped, so nothing is accepted as protected.
This test runs check_drc's OWN pass (`check_drc._via_in_paste_pass`, the code
`run_drc` calls) on every tracked board, and asserts:
1. The per-board unprotected counts equal the table above. If a board or the
   grade changes, the PR's census claim has EXPIRED: re-measure with
   `tests/measure_962_via_in_paste_census.py`, then update this table and the
   PR text together.
2. `--baseline <the board itself>` accepts every one of them as
   `inherited-via-in-paste`. This is the checkpoint decision: pre-existing vias
   are inherited, not held against a run.
3. Witness: at least one board fires, so a pass that silently found nothing
   anywhere cannot satisfy (1) by also returning zeros.

Run:
    python3 tests/test_962_census_via_in_paste.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from check_drc import _via_in_paste_pass  # noqa: E402
from run_utils import corpus_boards  # noqa: E402

RUN_ALL_TIMEOUT = 900
FAILS = []

#: board basename -> unprotected via-in-paste count (the PR's table)
PINNED = {
    'orangecrab_ext_pll.kicad_pcb': 136,
    'routed_output.kicad_pcb': 339,
    'rp2350_fpga_eensy_prePlane.kicad_pcb': 27,
}


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def grade(pcb, baseline=None):
    viol, acc = [], []
    _via_in_paste_pass(pcb, None, baseline, viol, acc, quiet=True)
    return viol, acc


def main():
    boards = corpus_boards()
    if not boards:
        print('SKIP: git cannot list the tracked corpus, so there is no fixed set '
              'to pin against (not a pass)')
        return 0
    got, fired = {}, 0
    for path in boards:
        name = os.path.basename(path)
        pcb = parse_kicad_pcb(path)
        viol, acc = grade(pcb)
        got[name] = len(viol)
        fired += bool(viol)
        protected = [a for a in acc if a.get('accepted') == 'protected-via-in-paste']
        if protected:
            check('%s: no tracked board declares filled+capped' % name, False,
                  '%d protected' % len(protected))
        if viol:
            v2, a2 = grade(pcb, baseline=pcb)
            check('2. %s: --baseline itself accepts all %d as inherited' % (name, len(viol)),
                  not v2 and len([a for a in a2 if a.get('accepted')
                                  == 'inherited-via-in-paste']) == len(viol),
                  '%d left, %d inherited' % (len(v2), len(a2)))
    want = {os.path.basename(p): PINNED.get(os.path.basename(p), 0) for p in boards}
    diff = {k: (want[k], got[k]) for k in want if want[k] != got[k]}
    check('1. per-board unprotected via-in-paste == the PR table (%d boards)' % len(boards),
          not diff, 'EXPIRED (pinned, measured): %s' % diff)
    missing = sorted(set(PINNED) - set(want))
    check('1. every pinned board is still tracked', not missing, str(missing))
    check('3. witness: the grade fires on at least one tracked board', fired >= 1)
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
