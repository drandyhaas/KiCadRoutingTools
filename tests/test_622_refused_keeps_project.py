#!/usr/bin/env python3
"""A fanout board the audit refuses is set aside WITH its project (fanout_from_plan.set_aside_refused, which
escape_plan's audit calls too).
  python3 tests/test_622_refused_keeps_project.py
The whole route's last resort lays the set-aside board (BENCH = fo.refused.kicad_pcb). Set aside by a rename of the
board alone, it had no .kicad_pro beside it, the partial plan was laid under a stock project, and route_bus's OUT
carried that project: on sdi_mipi_conv's SDI bus the board's 29 DRC exclusions and every rule severity were lost and
its edge clearance went 0.4 -> 0.2 mm (#441's hazard). Checks, on a board and project copied to a temp dir:
1. the board is moved to OUT.refused.kicad_pcb, nothing left at OUT.kicad_pcb;
2. OUT.refused.kicad_pro is there, byte for byte the project OUT had (and the .kicad_dru, the rules, likewise);
3. OUT.refused.json carries the findings (splits, far, stops) for the chain's feedback;
4. OUT.refused.plan.json is there, the plan sidecar OUT had (the solve's fronts, the braid's corridors, and on more
   than two layers the via ends moved onto the solve's layers -- without it they stayed as laid and 12 of the SDI
   bus's 14 bands met no tooth, TOOTH-OUT), and OUT.plan.json is left where it was (the last resort reads it).
"""
import json
import os
import shutil
import sys
import tempfile
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import fanout_from_plan as fp  # noqa: E402
fails = []


def check(cond, msg):
    print(('PASS ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


SRC = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy')
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'fo.kicad_pcb')
    shutil.copy(SRC + '.kicad_pcb', out)
    shutil.copy(SRC + '.kicad_pro', os.path.join(d, 'fo.kicad_pro'))
    with open(os.path.join(d, 'fo.kicad_dru'), 'w') as f:
        f.write('(version 1)\n')
    side = {'ends': {'/A': {}}, 'ends_model': {'front': {}}}
    with open(os.path.join(d, 'fo.plan.json'), 'w') as f:
        json.dump(side, f)
    held = fp.set_aside_refused(out, [('/A', 'src', ('F.Cu', 'B.Cu'))], ['/B'], ['a stop'])
    check(held == os.path.join(d, 'fo.refused.kicad_pcb') and os.path.isfile(held), '1. the board set aside')
    check(not os.path.exists(out), '1. nothing left at OUT')
    pro = os.path.join(d, 'fo.refused.kicad_pro')
    check(os.path.isfile(pro) and open(pro, 'rb').read() == open(SRC + '.kicad_pro', 'rb').read(),
          '2. its project beside it, byte for byte')
    check(os.path.isfile(os.path.join(d, 'fo.refused.kicad_dru')), '2. its rules beside it')
    rs = os.path.join(d, 'fo.refused.plan.json')
    check(os.path.isfile(rs) and json.load(open(rs)) == side, '4. its plan sidecar beside it')
    check(os.path.isfile(os.path.join(d, 'fo.plan.json')), '4. the round\'s own sidecar left where it was')
    rj = json.load(open(os.path.join(d, 'fo.refused.json')))
    check(rj == {'splits': [['/A', 'src', ['F.Cu', 'B.Cu']]], 'far': ['/B'], 'stops': ['a stop']},
          '3. the findings for the feedback')
print(f'\n{len(fails)} failure(s)')
sys.exit(1 if fails else 0)
