#!/usr/bin/env python3
"""A Default class clearance lowered by an automatic descent is bounded under
--escalation board and disclosed everywhere else (#1160).

complex_hierarchy: a rescue reconnected ONE net at 0.2567 under a 0.3 Default
class, the writeback stored 0.2567 as the CLASS, and step 2 routed every
Default net there -- 16 pad-segment grazes at the authored 0.3, while every
checker graded the new class and read clean, and nothing said so. interf_u: a
board with min_clearance 0 and class 0.254 rescued nine gaps down to 0.127
under --escalation board, because the board floors read rules.min_* only.

The writeback ratchet itself stays (#489 section 2 was declined). Rows:

  A. --escalation board takes the Default class as its clearance floor when
     min_clearance is unset, and the descent ladder stops there
  B. a descent is a `clearance` row in design_rules
  C. the writeback records a class lowered by a DESCENT and says so; a later
     step inherits the record; a class lowered by the step's own request is
     not recorded
  D. check_complete --authored-from counts it; a deliberate ceiling is not
  E. a later step's start says so before routing at the lowered class

    python3 tests/test_1160_class_clearance_descent.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, ROOT)

import fab_tiers                                                   # noqa: E402
import fix_kicad_drc_settings as fx                                # noqa: E402
from plane_pad_tap import note_clearance_used, fab_floor_clearance_track  # noqa: E402
from check_complete import fab_floor_integrity                     # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def project(path, class_clr, min_clr=0.0, extra=None):
    proj = {'board': {'design_settings': {'rules': {'min_clearance': min_clr},
                                          'rule_severities': {}}},
            'meta': {'filename': os.path.basename(path), 'version': 1},
            'net_settings': {'classes': [{'name': 'Default', 'clearance': class_clr,
                                          'track_width': 0.2, 'via_diameter': 0.6,
                                          'via_drill': 0.3}],
                             'meta': {'version': 3}}}
    if extra:
        proj['kicad_routing_tools'] = extra
    with open(path, 'w') as f:
        json.dump(proj, f)


def stage(tmp, name, class_clr, **kw):
    pcb = os.path.join(tmp, name + '.kicad_pcb')
    shutil.copy(BOARD, pcb)
    project(os.path.splitext(pcb)[0] + '.kicad_pro', class_clr, **kw)
    return pcb


def read_pro(pcb):
    with open(os.path.splitext(pcb)[0] + '.kicad_pro') as f:
        return json.load(f)


def writeback(pcb, clearance):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fx.fix_project_for_output(pcb, clearance=clearance, verbose=True)
    return buf.getvalue()


prev = fab_tiers.get_escalation_policy()
tmp = tempfile.mkdtemp(prefix='t1160_')
try:
    print('A. --escalation board floors the clearance at the Default class')
    check('board_floors_from_rules: class 0.254 when min_clearance is 0',
          fab_tiers.board_floors_from_rules({'min_clearance': 0.0}, 0.254)
          == {'clearance': 0.254})
    check('...a declared min_clearance still wins',
          fab_tiers.board_floors_from_rules({'min_clearance': 0.1}, 0.254)
          == {'clearance': 0.1})
    pcb = stage(tmp, 'interf', 0.254)
    floors = fab_tiers.set_policy_from_args(NS(escalation='board'), pcb)
    check('set_policy_from_args reads the class from the project',
          floors.get('clearance') == 0.254, str(floors))
    fc, _ft = fab_floor_clearance_track(NS(board_info=NS(copper_layers=['F.Cu', 'B.Cu'])))
    check('the rescue ladder stops at the class (fab_floor_clearance_track)',
          abs(fc - 0.254) < 1e-9, f'{fc}')
    fab_tiers.set_policy_from_args(NS(escalation='fab'), pcb)
    fc, _ft = fab_floor_clearance_track(NS(board_info=NS(copper_layers=['F.Cu', 'B.Cu'])))
    check('control: under fab the ladder still reaches the tier floor', fc < 0.254, f'{fc}')

    print('B. a descent is a design_rules clearance row')
    fab_tiers.set_escalation_policy('fab')
    pcb_data = NS(board_info=NS(min_clearance_used=None),
                  nets={7: NS(name='Net-(D204-K)')})
    note_clearance_used(pcb_data, 0.2567, net_id=7, requested=0.3, site='net rescue')
    rows = [r for r in fab_tiers.escalation_summary()['narrowed'] if r['kind'] == 'clearance']
    check('one clearance row, named', len(rows) == 1 and rows[0]['net_name'] == 'Net-(D204-K)'
          and rows[0]['requested'] == 0.3 and rows[0]['delivered'] == 0.2567, str(rows))
    check('...and the end-of-run line names it',
          'smallest clearance 0.2567' in fab_tiers.escalation_report_line(),
          fab_tiers.escalation_report_line())

    print('C. the writeback records and says it')
    out1 = stage(tmp, 'g5', 0.3)
    log = writeback(out1, 0.2567)
    p1 = read_pro(out1)
    rec = (p1.get('kicad_routing_tools') or {}).get(fx.CLASS_CLEARANCE_RELAXED_KEY)
    check('the class is lowered (the ratchet stays)',
          fab_tiers.project_default_class_clearance(p1) == 0.2567)
    check('the descent is recorded', rec == {'from': 0.3, 'to': 0.2567,
                                             'nets': ['Net-(D204-K)']}, str(rec))
    check('...and announced', 'DEFAULT CLASS CLEARANCE LOWERED BY A DESCENT -- 0.3 -> 0.2567'
          in log and '--clearance 0.3' in log, log[-600:])
    # Step 2: the project travels with the board; this step descends nothing.
    out2 = os.path.join(tmp, 'g7.kicad_pcb')
    shutil.copy(out1, out2)
    shutil.copy(os.path.splitext(out1)[0] + '.kicad_pro',
                os.path.splitext(out2)[0] + '.kicad_pro')
    fab_tiers.set_escalation_policy('off')          # a new run: a fresh ledger
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        said = fx.warn_if_class_clearance_relaxed(out2)
    check("a later step's start says so", said and 'below the 0.3 mm' in buf.getvalue(),
          buf.getvalue())
    log2 = writeback(out2, 0.2567)
    rec2 = (read_pro(out2).get('kicad_routing_tools') or {}).get(fx.CLASS_CLEARANCE_RELAXED_KEY)
    check('...the record is carried forward', rec2 == rec, str(rec2))
    check('...and the writeback repeats it', 'unchanged by this step but below the 0.3 mm'
          in log2, log2[-600:])

    print('   control: a class lowered by the step\'s own request')
    out3 = stage(tmp, 'ceiling', 0.3)
    fab_tiers.set_escalation_policy('fab')
    log3 = writeback(out3, 0.2)                     # --clearance-ceiling 0.2, no descent
    p3 = read_pro(out3)
    check('lowered to the request, NOT recorded or announced',
          fab_tiers.project_default_class_clearance(p3) == 0.2
          and fx.CLASS_CLEARANCE_RELAXED_KEY not in (p3.get('kicad_routing_tools') or {})
          and 'BY A DESCENT' not in log3)
    out4 = stage(tmp, 'ceiling_rescue', 0.3)
    fab_tiers.set_escalation_policy('fab')
    note_clearance_used(pcb_data, 0.15, net_id=7, requested=0.2, site='net rescue')
    writeback(out4, 0.15)
    rec4 = (read_pro(out4).get('kicad_routing_tools') or {}).get(fx.CLASS_CLEARANCE_RELAXED_KEY)
    check('a descent BELOW the request is recorded from the request, not the class',
          rec4 and rec4['from'] == 0.2 and rec4['to'] == 0.15, str(rec4))

    print('D. check_complete --authored-from')
    authored = stage(tmp, 'authored', 0.3)
    fl = fab_floor_integrity(out2, authored)
    keys = [r['key'] for r in fl.get('relaxed', [])]
    check('the descended class is a relaxed floor', 'net_class.Default.clearance' in keys,
          str(fl.get('relaxed')))
    fl = fab_floor_integrity(out3, authored)
    check('...a deliberate ceiling is not',
          'net_class.Default.clearance' not in [r['key'] for r in fl.get('relaxed', [])],
          str(fl.get('relaxed')))
finally:
    fab_tiers.set_escalation_policy(*prev)
    shutil.rmtree(tmp, ignore_errors=True)

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
