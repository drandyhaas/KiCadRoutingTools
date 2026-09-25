#!/usr/bin/env python3
"""#1031 repro on the run-32 board (glasgow_revC placed_v2).

Needs `wk/run32/placed_v2.kicad_pcb` (+ .kicad_pro) from
edgehero/KiCadRoutingTools@run32-assets `repro/run32/`. `wk/` is gitignored,
so this SELF-SKIPS (exit 77) on a clean clone; the synthetic-board test
`tests/test_1031_keepout_legality.py` always runs.

What it pins, all measured on that board at --clearance 0.2 (track 0.2 from
the board netclass, band 0.3 mm):

1. grade_pad_legality names the parts whose pads the router cannot land a
   track on: C89, D12, R11, R12, U14, U9 (11 pads). Routed ONE NET AT A TIME
   on the otherwise empty board (route.py --nets, 2026-09-24), the nets on
   R11.1, R12.2, D12.1, U14.5 and C89.1 failed "boxed in"; U9.3's routed --
   this channel may falsely reject, never falsely accept.
2. D4, D6, D11, U31 are NOT named: the issue's census flagged their pads (any
   copper within the band), and the same one-net routes connected all four --
   a track lands on the pad's inward half.
3. check_assembly still reads buildable (the channel is reported, #937), and
   check_reachability --pad R12.2 is CAGED where the issue measured PASSABLE;
   --pad D4.2 stays PASSABLE.

Run:
    python3 tests/test_1031_keepout_run32_repro.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

BOARD = os.path.join(ROOT, 'wk', 'run32', 'placed_v2.kicad_pcb')

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def main():
    if not (os.path.isfile(BOARD)
            and os.path.isfile(BOARD[:-len('.kicad_pcb')] + '.kicad_pro')):
        print(f"SKIP: {BOARD} (+ .kicad_pro) absent -- wk/ is gitignored; "
              f"fetch repro/run32 from edgehero/KiCadRoutingTools@run32-assets")
        return 77
    from kicad_parser import parse_kicad_pcb
    from placement.legality import grade_pad_legality
    from run_utils import check as run_check, evidence
    evidence(BOARD, 'run-32 board')

    g = grade_pad_legality(parse_kicad_pcb(BOARD), 0.2, pcb_file=BOARD)
    refs = [r[0] for r in g['oob_keepout_copper_refs']]
    pads = {(r[0], r[1]) for r in g['keepout_copper_pads']}
    check('1. the illegal parts are C89 D12 R11 R12 U14 U9',
          refs == ['C89', 'D12', 'R11', 'R12', 'U14', 'U9'], str(refs))
    check('1. 11 pads, R12.2 among them',
          len(pads) == 11 and ('R12', '2') in pads, str(sorted(pads)))
    check('1. band 0.3 = 0.2 + board-netclass track 0.2 / 2',
          abs(g['keepout_copper_band_mm'] - 0.3) < 1e-9
          and g['keepout_copper_track_width']['source'] == 'board netclass',
          str((g['keepout_copper_band_mm'], g['keepout_copper_track_width'])))
    check('2. D4 D6 D11 U31 (routed one-net on this board) are not named',
          not {'D4', 'D6', 'D11', 'U31'} & set(refs))
    check('2. the mounting holes are reported, not failed',
          {r[0] for r in g['keepout_copper_tht_refs']}
          == {'MK1', 'MK2', 'MK3', 'MK4'},
          str(sorted({r[0] for r in g['keepout_copper_tht_refs']})))

    r = run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'check_assembly.py'), BOARD,
                   '--clearance', '0.2'], accept=True)
    check('3. check_assembly still reads buildable, and prints the band',
          'VERDICT: buildable' in r.stdout
          and 'pads in a rule-area keep-out band' in r.stdout,
          r.stdout[-400:])
    r = run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'check_reachability.py'),
                   BOARD, '--pad', 'R12.2'], refuse='VERDICT    CAGED', code=1)
    check('3. check_reachability --pad R12.2 is CAGED', 'CAGED' in r.stdout)
    r = run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'check_reachability.py'),
                   BOARD, '--pad', 'D4.2'], accept=True)
    check('3. check_reachability --pad D4.2 stays PASSABLE',
          'PASSABLE' in r.stdout)

    if FAILS:
        print('\nFAILED: %d check(s): %s' % (len(FAILS), FAILS))
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
