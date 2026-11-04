#!/usr/bin/env python3
"""#1579: under --escalation board no descent rung offers a via whose annular
ring is below the board's min_via_annular_width.

The board policy read the ring as a floor key (`BOARD_RULE_TO_FLOOR_KEY`
'annular') and `_apply_board_floors` raised each key ALONE, while every descent
site reads only a rung's diameter and drill. On a board declaring ring 0.12,
via 0.25 and hole 0.15 the fine rung read `annular` 0.12 beside a 0.30/0.15
via (ring 0.075) and the advanced one beside 0.25/0.15 (ring 0.05): route.py
shipped 16 such vias and the writeback lowered min_via_annular_width to match,
and qfn_fanout clamped a via-in-pad to 0.25/0.15 under a warning saying the
policy "never reaches the advanced 0.25/0.15 via". Now:

  1. under board every rung rings at least the board's ring (the diameter is
     raised to drill + 2 * ring, as list_nets sizes the working via, #1349),
     on 2 and 4 layers, after a per-net drill floor, and with the writeback
     off (graded floors, #1287, KiCad's factory ring where none is declared).
  2. --escalation fab is unchanged: its rungs are the tier's, ring and all.
  3. clamp_via_to_pad holds the ring when it clamps, and keeps the REQUEST
     when every rung is larger than it (a larger rung would bulge further);
     under fab it is unchanged.
  4. the board warning states the via it delivered, not the advanced one.
  5. qfn_fanout end to end (the issue's case): no 0.25/0.15 via-in-pad, the
     request kept, no ring below the request's own.

    python3 tests/test_1579_board_ring_ladder.py
"""
import collections
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)

import fab_tiers as ft                                         # noqa: E402
from copy_board import copy_board                              # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402

RULES = dict(min_clearance=0.16, min_hole_clearance=0.2, min_via_annular_width=0.12,
             min_via_diameter=0.25, min_through_hole_diameter=0.15, min_track_width=0.1)
RING = 0.12
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def ring(r):
    return round((r['via_diameter'] - r['via_drill']) / 2.0, 6)


def vias(rungs):
    return [(r['via_diameter'], r['via_drill']) for r in rungs]


def board(floors=None, policy='board'):
    ft.set_default_fab_tier('auto')
    ft.set_escalation_policy(policy, ft.board_floors_from_rules(RULES)
                             if floors is None else floors)


def ladder():
    print('1. every board rung holds the ring')
    for n in (2, 4):
        board()
        rungs = ft.escalation_rungs(n)
        check(f'{n} layers: every rung rings >= {RING}',
              all(ring(r) >= RING - 1e-9 for r in rungs), str(vias(rungs)))
        check(f'{n} layers: the standard 0.45/0.20 rung is kept',
              vias(rungs)[0] == (0.45, 0.2), str(vias(rungs)))
    board()
    check('4 layers: the descents are 0.39/0.15 (0.15 + 2 x 0.12)',
          set(vias(ft.escalation_rungs(4))[1:]) == {(0.39, 0.15)},
          str(vias(ft.escalation_rungs(4))))
    rungs = ft.escalation_rungs(4, extra_floors={'via_drill': 0.2})
    check('a per-net drill floor raised after keeps the ring',
          all(ring(r) >= RING - 1e-9 for r in rungs), str(vias(rungs)))
    # an odd ring: the rounded diameter must not ring a micron under it
    board(dict(ft.board_floors_from_rules(RULES), annular=0.0762))
    rungs = ft.escalation_rungs(4)
    check('a 3-mil ring is held to the micron (rounded up)',
          all(round(r['via_diameter'], 3) - r['via_drill'] >= 2 * 0.0762 - 1e-9
              for r in rungs), str(vias(rungs)))
    # writeback off: the graded floors, KiCad's factory ring where undeclared
    graded = ft.graded_board_floors({k: v for k, v in RULES.items()
                                     if k != 'min_via_annular_width'})
    board(graded)
    rungs = ft.escalation_rungs(4)
    check('writeback off, ring undeclared: the factory 0.1 ring holds',
          graded.get('annular') == 0.1 and all(ring(r) >= 0.1 - 1e-9 for r in rungs),
          f"{graded.get('annular')} {vias(rungs)}")


def fab_unchanged():
    print('2. --escalation fab is unchanged')
    ft.set_default_fab_tier('auto')
    ft.set_escalation_policy('fab', {})
    bare = ft.escalation_rungs(4)
    board(policy='fab')
    got = ft.escalation_rungs(4)
    check('fab with the board floors = fab without them', got == bare,
          f'{vias(got)} vs {vias(bare)}')
    check('...and still reaches the 0.25/0.15 via', (0.25, 0.15) in vias(got),
          str(vias(got)))


def clamp():
    print('3. the via-in-pad clamp')
    from bga_fanout.geometry import clamp_via_to_pad

    def pad(s):
        return SimpleNamespace(size_x=s, size_y=s, global_x=0.0, global_y=0.0, net_id=1)
    board()
    floors = ft.escalation_rungs(4)
    s, d, st, rung = clamp_via_to_pad(0.45, 0.2, pad(0.40), floors)
    check('a 0.40 pad: clamped to 0.40, ring >= 0.12',
          st == 'clamped' and abs(s - 0.40) < 1e-9 and (s - d) / 2 >= RING - 1e-9,
          f'{s}/{d} {st}')
    s, d, st, rung = clamp_via_to_pad(0.45, 0.2, pad(0.28), floors)
    check('a 0.28 pad: held at the deepest rung, 0.39/0.15',
          st == 'floor' and (round(s, 4), round(d, 4)) == (0.39, 0.15), f'{s}/{d} {st}')
    s, d, st, rung = clamp_via_to_pad(0.3, 0.15, pad(0.22), floors)
    check('a 0.30/0.15 request in a 0.22 pad keeps the request (no rung is smaller)',
          (s, d, st, rung) == (0.3, 0.15, 'floor', 0), f'{s}/{d} {st} {rung}')
    board(policy='fab')
    s, d, st, rung = clamp_via_to_pad(0.3, 0.15, pad(0.22), ft.escalation_rungs(4))
    check('under fab the same pad still takes the advanced 0.25/0.15',
          (round(s, 4), round(d, 4), st) == (0.25, 0.15, 'floor'), f'{s}/{d} {st}')


def warning():
    print('4. the board warning states the via it delivered')
    board()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ft.warn_fab_escalation('4 via-in-pad(s) (sub-0.45mm pads) #1579 test',
                               vias=[(0.4, 0.16), (0.39, 0.15)])
    txt = buf.getvalue()
    check('it names the smallest via delivered and its ring',
          'smallest via delivered 0.39/0.15 mm, ring 0.12 mm' in txt, txt)
    check('and no longer claims the advanced 0.25/0.15 is out of reach',
          'never reaches the advanced' not in txt, txt)
    ft.set_escalation_policy('fab', {})
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ft.warn_fab_escalation('fab arm #1579 test', vias=[(0.25, 0.15)])
    check('the fab warning is unchanged',
          'escalated standard->advanced fab floor (0.25/0.15 via etc.' in buf.getvalue(),
          buf.getvalue())


def qfn_end_to_end(td):
    print('5. qfn_fanout --escalation board, a 0.30/0.15 via-in-pad (the issue)')
    src = os.path.join(td, 'q.kicad_pcb')
    copy_board(os.path.join(REPO, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb'), src)
    evidence(src, 'the qfn fixture copy')
    pro = os.path.splitext(src)[0] + '.kicad_pro'
    doc = json.load(open(pro)) if os.path.isfile(pro) else {
        "board": {"design_settings": {"rules": {}, "rule_severities": {}}},
        "meta": {"filename": "q.kicad_pro", "version": 1},
        "net_settings": {"classes": [{"name": "Default", "clearance": 0.16,
                                      "track_width": 0.2, "via_diameter": 0.45,
                                      "via_drill": 0.2}], "meta": {"version": 3}}}
    doc["board"]["design_settings"]["rules"].update(RULES)
    json.dump(doc, open(pro, 'w'), indent=2)
    out = os.path.join(td, 'qA.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', '-B', os.path.join('py_router', 'qfn_fanout.py'),
                   src, '--output', out, '--component', 'U1', '--escape-method', 'underpad',
                   '--allow-via-in-pad', '--via-size', '0.3', '--via-drill', '0.15',
                   '--escalation', 'board'], accept=True)
    log = r.stdout + r.stderr
    evidence(out, 'the fanned board')
    from kicad_parser import parse_kicad_pcb
    got = collections.Counter((round(v.size, 4), round(v.drill, 4))
                              for v in parse_kicad_pcb(out).vias)
    check('the escape placed vias (the case under test)', sum(got.values()) > 0, str(got))
    check('no 0.25/0.15 via (ring 0.05) ships', (0.25, 0.15) not in got, str(got))
    check('no via rings below the request\'s own 0.075',
          all((s - d) / 2 >= 0.075 - 1e-9 for s, d in got), str(got))
    check('the false "never reaches the advanced" claim is gone',
          'never reaches the advanced' not in log, log[-800:])


def main():
    saved = (ft.get_default_fab_tier(), ft.get_escalation_policy())
    td = tempfile.mkdtemp(prefix='krt1579_')
    try:
        ladder()
        fab_unchanged()
        clamp()
        warning()
        qfn_end_to_end(td)
    finally:
        ft.set_default_fab_tier(*saved[0])
        ft.set_escalation_policy(*saved[1])
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
