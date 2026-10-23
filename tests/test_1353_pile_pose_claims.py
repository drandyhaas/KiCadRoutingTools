#!/usr/bin/env python3
"""#1353: board_brief and board_context make no pose claims off a pile.

#1103 withholds what emit_intent reads off a pile's staging poses
(`context.pose_claims_withheld`); these two tools had no such rule.

* board_brief told gp14 CM5 (a pile) "12 of them sit alone at their
  coordinate, so `at` is assertable with place_fixed". `pose_shared_with`
  is empty on the repo's own pile format -- `make_unplaced.py` spreads every
  part on a grid off the outline, each one alone -- so the per-part check
  could not fire. And `place_fixed` / `place_at` / `place_pack` exist
  nowhere in this repository.
* board_context printed 653 pin-order verdicts (124 CROSSED), EMC leg
  distances, mating edges and cap elections on StickHub's pile, all
  measured at staging poses, with no pile notice.

Asserts, on esp_prog placed and piled by make_unplaced:
  1. board_brief: a pile's mechanical parts are `staged`, the text says so
     and names no tool that does not exist; the placed board's are not
     staged and stay declarable;
  2. board_context: a pile carries `pile` with what it withheld -- no
     pin-order row, leg distance, mating edge or cap election survives for
     unlocked parts -- and the sheet says it first; the placed board's sheet
     is unchanged in kind (`pile` None, its rows measured);
  3. _withhold_staged keeps a claim whose parts are all LOCKED (#1103).

    python3 tests/test_1353_pile_pose_claims.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb       # noqa: E402
import board_brief                             # noqa: E402
import board_context                           # noqa: E402
from run_utils import check, evidence          # noqa: E402

SRC = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
PILER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts',
                     'make_unplaced.py')
NOT_TOOLS = ('place_fixed', 'place_at', 'place_pack')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


def legs(doc):
    return [g for r in (doc.get('emc_roles') or {}).get('roles') or ()
            for pr in r.get('proximity') or () for g in pr.get('legs') or ()]


def main():
    tmp = tempfile.mkdtemp(prefix='t1353_')
    try:
        pile = os.path.join(tmp, 'pile.kicad_pcb')
        check([sys.executable, '-X', 'utf8', PILER, SRC, pile], accept=True)
        evidence(pile, 'the pile board')

        print("1: board_brief")
        placed_b = quiet(board_brief.build_brief, parse_kicad_pcb(SRC), SRC)
        pile_b = quiet(board_brief.build_brief, parse_kicad_pcb(pile), pile)
        ok("precondition: the pile reads as a pile, the board does not",
           pile_b['state']['pile'] and not placed_b['state']['pile'])
        pm = (pile_b.get('mechanical') or {}).get('refs') or {}
        bm = (placed_b.get('mechanical') or {}).get('refs') or {}
        ok("precondition: esp_prog has mechanical parts", bool(pm) and bool(bm),
           str(sorted(bm)))
        ok("on the pile no mechanical part is alone-and-SHARED, yet each is "
           "staged", all(not v['pose_shared_with'] and v['staged']
                         for v in pm.values()),
           str({r: (v['pose_shared_with'], v['staged']) for r, v in pm.items()}))
        ok("on the placed board none is staged",
           all(v['staged'] is False for v in bm.values()))
        pt = board_brief.format_text(pile_b)
        bt = board_brief.format_text(placed_b)
        ok("the pile's text says the poses are staging",
           'STAGING pose' in pt and f"0 of them sit alone" in pt,
           next((l for l in pt.splitlines() if 'mechanical' in l), ''))
        ok("no text names a tool that does not exist",
           not any(t in pt or t in bt for t in NOT_TOOLS)
           and not any(t in str(pile_b['mechanical'].get('note'))
                       for t in NOT_TOOLS))
        ok("the placed board's mechanical poses stay declarable",
           f"{len(bm)} of them sit alone at their coordinate" in bt)

        print("2: board_context")
        placed_c = quiet(board_context.build_context, parse_kicad_pcb(SRC),
                         SRC, clearance=0.2, track_width=0.2)
        pile_c = quiet(board_context.build_context, parse_kicad_pcb(pile),
                       pile, clearance=0.2, track_width=0.2)
        ok("the placed board carries no pile block", placed_c['pile'] is None)
        ok("precondition: the placed board HAS pin-order rows, legs, mating "
           "and serves to withhold",
           placed_c['pin_order']['rows'] and legs(placed_c)
           and any(p['mating'] for p in placed_c['parts'])
           and any(p['serves'] for p in placed_c['parts']))
        pw = (pile_c['pile'] or {}).get('withheld') or {}
        ok("the pile carries a pile block naming what it withheld",
           bool(pile_c['pile']) and pw.get('pin_order.rows'), str(pw)[:200])
        ok("no pin-order verdict survives on the pile",
           not [r for r in pile_c['pin_order']['rows'] if not r.get('error')])
        ok("no leg distance survives on the pile",
           legs(pile_c) and all(g['gap_mm'] is None for g in legs(pile_c)))
        ok("no mating edge or cap election survives on the pile",
           not any(p['mating'] or p['serves'] for p in pile_c['parts']))
        md = board_context.format_md(pile_c)
        ok("the sheet says PILE before any number, and the rows withheld",
           md.index('This board is a PILE') < md.index('## Pin-order')
           and 'pin-order row(s) WITHHELD' in md)
        ok("the placed sheet carries no pile notice",
           'PILE' not in board_context.format_md(placed_c))

        print("3: a locked part's pose is a claim (#1103)")
        po = {'rows': [{'a': 'U1', 'b': 'J1', 'verdict': 'CROSSED'},
                       {'a': 'U1', 'b': 'R1', 'verdict': 'AGREES'}]}
        emc = {'roles': [{'proximity': [{'legs': [
            {'from': {'ref': 'U1'}, 'to': {'ref': 'J1'}, 'gap_mm': 1.0},
            {'from': {'ref': 'U1'}, 'to': {'ref': 'R1'}, 'gap_mm': 2.0}],
            'worst_leg_mm': 2.0}]}]}
        locked = {'U1', 'J1'}
        p = {'withheld': {}}
        board_context._withhold_staged(
            po, emc, lambda *refs: all(r in locked for r in refs), p)
        ok("the all-locked row and leg are kept, the other withheld",
           [r['b'] for r in po['rows']] == ['J1']
           and [g['gap_mm'] for g in emc['roles'][0]['proximity'][0]['legs']]
           == [1.0, None]
           and emc['roles'][0]['proximity'][0]['worst_leg_mm'] is None,
           str(p['withheld']))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
