#!/usr/bin/env python3
"""A pad clearance override never becomes a board floor for escalation
(#1369 part d).

The DRC-floor writeback caps rules.min_clearance at the smallest pad
clearance override the router honoured (#530; KiCad floors an override at
that rule). On a board whose parts carry a 2 mil library override that wrote
rules.min_clearance 0.15 -> 0.0508, and the next `--escalation board` step
read 0.0508 as "the board's own declared minimum" and could rescue nets that
carry no override down to it.

The writeback now records the provenance in the kicad_routing_tools records
(`pad_override_min_clearance`: {rule, board}, through tool_records), and the
escalation resolver reads the board's own minimum while the rule still holds
the capped value.

Rows:
  A. a writeback on an override board keeps the #530 cap AND records it
  B. --escalation board reads the board's minimum (0.15), not the override
  C. down a chain: a later step's lower clearance becomes the board's
     minimum; a clearance below the override drops the record
  D. a rule changed since the record (by hand) is the board's own
  E. the live-board writer records the same, and the reader unmasks it

    python3 tests/test_1369d_pad_override_not_a_floor.py
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

BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')
OVERRIDE = 0.0508
DECLARED = 0.15
KEY = 'pad_override_min_clearance'
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def near(a, b, tol=1e-6):
    return a is not None and b is not None and abs(float(a) - float(b)) <= tol


def stage(tmp, name):
    dst = os.path.join(tmp, name + '.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', run_utils.tool('copy_board.py'),
                    BOARD, dst], check=True, capture_output=True, text=True)
    return dst


def writeback(src, dst, clearance):
    if src != dst:
        subprocess.run([sys.executable, '-X', 'utf8',
                        run_utils.tool('copy_board.py'), src, dst],
                       check=True, capture_output=True, text=True)
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


def test_chain():
    run_utils.evidence(BOARD)
    with tempfile.TemporaryDirectory() as tmp:
        src = stage(tmp, 'in')
        print('\n-- A. the writeback keeps the cap and records it --')
        check('the input declares min_clearance 0.15',
              near(rules(src).get('min_clearance'), DECLARED),
              rules(src).get('min_clearance'))
        check('...and --escalation board reads 0.15 on it',
              near(board_floor(src), DECLARED), board_floor(src))
        s1 = writeback(src, os.path.join(tmp, 's1.kicad_pcb'), 0.2)
        check('#530: rules.min_clearance is capped at the pad override',
              near(rules(s1).get('min_clearance'), OVERRIDE),
              rules(s1).get('min_clearance'))
        rec = record(s1)
        check('the cap is recorded as a pad override',
              isinstance(rec, dict) and near(rec.get('rule'), OVERRIDE)
              and near(rec.get('board'), DECLARED), rec)

        print('\n-- B. --escalation board reads the board minimum --')
        check('the board floor is 0.15, not the 0.0508 override',
              near(board_floor(s1), DECLARED), board_floor(s1))

        print('\n-- C. down a chain --')
        s2 = writeback(s1, os.path.join(tmp, 's2.kicad_pcb'), 0.12)
        check('a later step at 0.12: the rule still holds the override',
              near(rules(s2).get('min_clearance'), OVERRIDE))
        check('...and the board minimum is now the routed 0.12',
              near((record(s2) or {}).get('board'), 0.12), record(s2))
        check('...which --escalation board reads',
              near(board_floor(s2), 0.12), board_floor(s2))
        s3 = writeback(s2, os.path.join(tmp, 's3.kicad_pcb'), 0.03)
        check('a step routed BELOW the override: the rule is its 0.03',
              near(rules(s3).get('min_clearance'), 0.03),
              rules(s3).get('min_clearance'))
        check('...the record is dropped', record(s3) is None, record(s3))
        check('...and the board floor is 0.03', near(board_floor(s3), 0.03),
              board_floor(s3))

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
    if not hasattr(fx, 'record_live_pad_override_floor'):
        check('record_live_pad_override_floor exists', False)
        return
    b = _Board(round(OVERRIDE * 1e6))          # the writer lowered it to 50800 nm
    fx.record_live_pad_override_floor(b, DECLARED, 0.2)
    ns = tool_records.live_namespace(b)
    rec = ns.get(KEY)
    check('the live board records the override',
          isinstance(rec, dict) and near(rec.get('board'), DECLARED), rec)
    check('the reader unmasks the live rule',
          near(fab_tiers.board_declared_min_clearance(
              b.GetDesignSettings().m_MinClearance / 1e6, ns), DECLARED))
    b._ds.m_MinClearance = round(0.03 * 1e6)
    fx.record_live_pad_override_floor(b, OVERRIDE, 0.03)
    check('a live write below the override drops it',
          tool_records.live_namespace(b).get(KEY) is None,
          tool_records.live_namespace(b).get(KEY))


if __name__ == '__main__':
    test_chain()
    test_live_twin()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASS'}")
    sys.exit(1 if FAILS else 0)
