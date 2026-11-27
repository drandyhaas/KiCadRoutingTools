#!/usr/bin/env python3
"""A .kicad_dru clearance rule never becomes the board's floor for
`--escalation board` (#2060).

The DRC-floor writeback caps rules.min_clearance at the smallest .kicad_dru
clearance rule (#498: KiCad's minimum is an absolute floor that outranks custom
rules). The rule binds only the items its condition names -- one layer, the
coupled pairs, a rule area -- and those route at the rule whatever the
escalation. Yet the next `--escalation board` step read the capped value as
"the board's own Board Setup minimum", and could narrow every OTHER net down
to it: on a board declaring 0.2 with a 0.15 pair rule, a Default net could
descend to 0.15.

The writeback now records the cap in the same provenance record #1369 keeps
for a pad override (`pad_override_min_clearance`: {rule, board}), and the
escalation resolver reads the board's own minimum while the rule still holds
the capped value. KiCad still grades at the capped rule.

Rows:
  A. a layer rule: the cap holds, is recorded, and the board floor is 0.2
  B. a coupled-pair rule: the same
  C. down a chain: a later step routed at 0.12 makes 0.12 the board minimum
  D. a rule changed since the record (by hand) is the board's own
  E. the live-board writer records the same

    python3 tests/test_2060_dru_cap_not_a_board_floor.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import run_utils                                                   # noqa: E402
import fab_tiers                                                   # noqa: E402
import fix_kicad_drc_settings as fx                                # noqa: E402
import tool_records                                                # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
DECLARED = 0.2
RULE = 0.15
KEY = 'pad_override_min_clearance'
LAYER_RULE = ('(version 1)\n(rule "bottom relaxed" (layer B.Cu)\n'
              f'  (constraint clearance (min {RULE}mm)))\n')
PAIR_RULE = ('(version 1)\n(rule "pair gap"\n'
             f'  (constraint clearance (min {RULE}mm))\n'
             '  (condition "AB.isCoupledDiffPair()"))\n')
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def near(a, b, tol=1e-6):
    return a is not None and b is not None and abs(float(a) - float(b)) <= tol


def _copy(src, dst):
    subprocess.run([sys.executable, '-X', 'utf8', run_utils.tool('copy_board.py'),
                    src, dst], check=True, capture_output=True, text=True)


def stage(tmp, name, dru_text):
    """BOARD with a project declaring min_clearance and the Default class at
    DECLARED, and DRU_TEXT as its .kicad_dru."""
    dst = os.path.join(tmp, name + '.kicad_pcb')
    _copy(BOARD, dst)
    pro = os.path.splitext(dst)[0] + '.kicad_pro'
    with open(pro, encoding='utf-8') as f:
        proj = json.load(f)
    proj.setdefault('board', {}).setdefault('design_settings', {}) \
        .setdefault('rules', {})['min_clearance'] = DECLARED
    for c in proj.setdefault('net_settings', {}).setdefault('classes', []):
        if c.get('name') == 'Default':
            c['clearance'] = DECLARED
    proj.get('kicad_routing_tools', {}).pop(KEY, None)
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)
    with open(os.path.splitext(dst)[0] + '.kicad_dru', 'w', encoding='utf-8') as f:
        f.write(dru_text)
    return dst


def writeback(src, dst, clearance):
    _copy(src, dst)
    with contextlib.redirect_stdout(io.StringIO()):
        fx.fix_project_for_output(dst, input_pcb=src, clearance=clearance)
    return dst


def rules(pcb):
    with open(os.path.splitext(pcb)[0] + '.kicad_pro', encoding='utf-8') as f:
        return json.load(f)['board']['design_settings']['rules']


def record(pcb):
    return tool_records.effective_namespace(pcb).get(KEY)


def board_floor(pcb, clearance=0.4):
    args = NS(escalation='board', no_fix_drc_settings=False, clearance=clearance,
              track_width=None, via_size=None, via_drill=None,
              hole_to_hole_clearance=None, board_edge_clearance=None)
    with contextlib.redirect_stdout(io.StringIO()):
        floors = fab_tiers.set_policy_from_args(args, pcb)
    return floors.get('clearance')


def one_rule(tmp, tag, dru_text):
    src = stage(tmp, f'{tag}_in', dru_text)
    check(f'{tag}: the input declares min_clearance {DECLARED}',
          near(rules(src).get('min_clearance'), DECLARED),
          rules(src).get('min_clearance'))
    check(f'{tag}: --escalation board reads {DECLARED} on the input',
          near(board_floor(src), DECLARED), board_floor(src))
    s1 = writeback(src, os.path.join(tmp, f'{tag}_s1.kicad_pcb'), DECLARED)
    check(f'{tag}: #498 still caps rules.min_clearance at the rule',
          near(rules(s1).get('min_clearance'), RULE),
          rules(s1).get('min_clearance'))
    rec = record(s1)
    check(f'{tag}: the cap is recorded with the board value',
          isinstance(rec, dict) and near(rec.get('rule'), RULE)
          and near(rec.get('board'), DECLARED), rec)
    check(f'{tag}: --escalation board reads {DECLARED}, not the rule\'s {RULE}',
          near(board_floor(s1), DECLARED), board_floor(s1))
    return s1


def test_chain():
    run_utils.evidence(BOARD)
    with tempfile.TemporaryDirectory() as tmp:
        print('\n-- A. a layer rule --')
        s1 = one_rule(tmp, 'layer', LAYER_RULE)
        print('\n-- B. a coupled-pair rule --')
        one_rule(tmp, 'pair', PAIR_RULE)

        print('\n-- C. down a chain --')
        s2 = writeback(s1, os.path.join(tmp, 's2.kicad_pcb'), 0.12)
        check('a later step routed at 0.12 writes rules.min_clearance 0.12',
              near(rules(s2).get('min_clearance'), 0.12),
              rules(s2).get('min_clearance'))
        check('...drops the record (the rule is now the board\'s own)',
              record(s2) is None, record(s2))
        check('...and --escalation board reads 0.12',
              near(board_floor(s2), 0.12), board_floor(s2))

        print('\n-- D. a rule changed since the record --')
        pro = os.path.splitext(s1)[0] + '.kicad_pro'
        with open(pro, encoding='utf-8') as f:
            proj = json.load(f)
        proj['board']['design_settings']['rules']['min_clearance'] = 0.1
        with open(pro, 'w', encoding='utf-8') as f:
            json.dump(proj, f)
        check('a hand-set 0.1 is the board floor (the record no longer matches)',
              near(board_floor(s1), 0.1), board_floor(s1))


class _Settings:
    def __init__(self, nm):
        self.m_MinClearance = nm


class _Board:
    """A stand-in live board: no file, records held in its properties."""
    def __init__(self, nm):
        self._ds = _Settings(nm)
        self._props = {}

    def GetFileName(self):
        return ''

    def GetDesignSettings(self):
        return self._ds

    def GetProperties(self):
        return dict(self._props)

    def SetProperties(self, props):
        self._props = dict(props)


def test_live_twin():
    print('\n-- E. the live writer --')
    b = _Board(round(RULE * 1e6))            # the writer lowered it to the rule
    fx.record_live_pad_override_floor(b, DECLARED, DECLARED)
    ns = tool_records.live_namespace(b)
    rec = ns.get(KEY)
    check('the live board records the dru cap',
          isinstance(rec, dict) and near(rec.get('board'), DECLARED), rec)
    check('the reader unmasks the live rule',
          near(fab_tiers.board_declared_min_clearance(
              b.GetDesignSettings().m_MinClearance / 1e6, ns), DECLARED))


if __name__ == '__main__':
    test_chain()
    test_live_twin()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASS'}")
    sys.exit(1 if FAILS else 0)
