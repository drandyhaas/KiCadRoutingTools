#!/usr/bin/env python3
"""#1032 end to end: the improvement gate does not judge a poured net that a
scoped call's --nets leaves to a later step -- on BOTH fronts.

test_600 covers compare_connectivity/format_report in isolation, and its
engine part forces the verdict, so the WIRING in batch_route (read
summary['finalize_excluded_nets'], else derive it with
plan_excluded_net_names; resolve names to ids on the board the maps were
built from) was untested: passing `excluded_ids=None` there left test_600
green. This drives the real engine:

  A. (always) kicad_files/lvds_converter_dualclk_gnd -- a TRACKED routed
     board with a /GND pour on B.Cu -- re-routing a few signal nets with
     /GND outside --nets:
       * CLI front (output file), finalize ON  -> GND excluded by plan;
       * GUI front (return_results=True)        -> GND excluded by plan;
       * CLI front with KICAD_PLANE_FINALIZE=0  -> the fallback derivation
         excludes GND too;
       * control: the same call WITH GND in the net list -> GND is judged.
     A spy on improvement_gate.compare_connectivity records what the engine
     passed and what came back.
  B. (when wk/run32 is staged; else that half self-skips) the issue's own
     K3A_it1_g lap, which CUTS the In1 GND pour: the gate lists GND under
     excluded_by_plan with more disconnected pads after than before, and
     neither `lost` nor `worsened` names it.

    python3 tests/test_1032_gate_excludes_plan_nets.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from run_utils import evidence  # noqa: E402

# TRACKED (git ls-files), routed, with a /GND pour on B.Cu. Not
# flat_hierarchy_routed: that one is gitignored and only exists after
# test_flat_hierarchy.py --overwrite, so a clean clone died on it.
BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')
GND = '/GND'
K3A = os.path.join(ROOT, 'wk', 'run32', 'K3A_it1_g.kicad_pcb')

fails = []


def check(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name
          + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def part_a():
    import improvement_gate as ig
    import route as _route
    from copy_board import copy_board
    from kicad_parser import parse_kicad_pcb

    evidence(BOARD, 'lvds_converter_dualclk_gnd board')
    tmp = tempfile.mkdtemp(prefix='gate1032_')
    try:
        src = os.path.join(tmp, 'in.kicad_pcb')
        copy_board(BOARD, src)
        pcb = parse_kicad_pcb(src)
        gnd = next(n for n, v in pcb.nets.items() if v.name == GND)
        check('fixture: GND is poured', any(z.net_id == gnd for z in pcb.zones))
        with_cu = {s.net_id for s in pcb.segments}
        names = sorted(v.name for n, v in pcb.nets.items()
                       if n in with_cu and v.name and v.name != GND)[:4]
        check('fixture: real routed signal nets to re-route', len(names) >= 2,
              names)

        calls = []
        orig = ig.compare_connectivity

        def spy(before, after, net_name, excluded_ids=None):
            res = orig(before, after, net_name, excluded_ids=excluded_ids)
            calls.append({'excluded': set(excluded_ids or ()),
                          'in_map': gnd in before, 'res': res})
            return res

        kw = dict(track_width=0.2, clearance=0.2, grid_step=0.1,
                  force_reroute=True)

        def one(label, nets, gui=False, finalize=True):
            calls.clear()
            old = os.environ.get('KICAD_PLANE_FINALIZE')
            if not finalize:
                os.environ['KICAD_PLANE_FINALIZE'] = '0'
            try:
                out = '' if gui else os.path.join(tmp, f'{label}.kicad_pcb')
                _route.batch_route(src, out, nets, return_results=gui, **kw)
            finally:
                if old is None:
                    os.environ.pop('KICAD_PLANE_FINALIZE', None)
                else:
                    os.environ['KICAD_PLANE_FINALIZE'] = old
            if not calls:
                check(f'{label}: the gate ran', False)
                return None
            return calls[-1]

        ig.compare_connectivity = spy
        try:
            for label, gui, fin in (('cli', False, True), ('gui', True, True),
                                    ('cli_finalize_off', False, False)):
                c = one(label, names, gui=gui, finalize=fin)
                if c is None:
                    continue
                ex = [n for n, _b, _a in c['res']['excluded_by_plan']]
                check(f'{label}: GND id passed as excluded and present in '
                      f'the connectivity map', gnd in c['excluded']
                      and c['in_map'], (sorted(c['excluded']), c['in_map']))
                check(f'{label}: excluded_by_plan names GND, and GND is '
                      f'neither lost nor worsened', ex == [GND]
                      and GND not in c['res']['lost']
                      and GND not in [n for n, _b, _a
                                      in c['res']['worsened']],
                      c['res'])
            c = one('control_gnd_in_scope', names + [GND])
            if c is not None:
                check('control: GND inside --nets is JUDGED, not excluded',
                      gnd not in c['excluded']
                      and not c['res']['excluded_by_plan'], c['res'])
        finally:
            ig.compare_connectivity = orig
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def part_b():
    if not os.path.isfile(K3A):
        print('SKIP (part B): wk/run32/K3A_it1_g.kicad_pcb absent')
        return False
    tmp = tempfile.mkdtemp(prefix='gate1032k3a_')
    try:
        out = os.path.join(tmp, 'busV3.kicad_pcb').replace('\\', '/')
        env = dict(os.environ, MSYS2_ARG_CONV_EXCL='*',
                   KICAD_GLOBAL_PLAN='1', KICAD_GLOBAL_PLAN_SEQ='1',
                   KICAD_GLOBAL_PLAN_SEQ_COST='1.5',
                   KICAD_GLOBAL_PLAN_VIA_COST='20',
                   KICAD_GLOBAL_PLAN_ITERS='50000',
                   KICAD_GLOBAL_PLAN_ATTRACT='1', KICAD_ATTRACT_POTENTIAL='65',
                   KICAD_GLOBAL_PLAN_RIVER='1', KICAD_FINALIZE_REAUDIT='1',
                   KICAD_PACK_INLINE='1')
        argv = [sys.executable, '-X', 'utf8',
                os.path.join(ROOT, 'py_router', 'route.py'), K3A, out,
                '--no-bga-zones', '--max-ripup', '5',
                '--ripped-route-avoidance-cost', '3',
                '--track-proximity-cost', '2', '--track-width', '0.127',
                '--clearance-ceiling', '0.1', '--via-size', '0.5',
                '--via-drill', '0.3', '--hole-to-hole-clearance', '0.25',
                '--board-edge-clearance', '0.5',
                '--power-nets', 'GND', '+3V3', '+5V', '+1V2', '/xVBUS',
                '--power-nets-widths', '0.3', '0.3', '0.4', '0.3', '0.4',
                '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
                '--layer-costs', '1.0', '6.0', '1.5', '1.0',
                '--nets', '/D1', '/D2', '/D3', '/D4', '/D6', '/D7', '/FLAGB',
                '/FPGA_DONE', 'Net-(U30A-IOT_219)', 'Net-(U30A-IOT_170)',
                '--rip-existing-nets', '+3V3', '+1V2', '/xVBUS',
                'Net-(U30A-IOT_172)', '/CLKREF', '/SDA', '/D0', '/D1', '/D2',
                '/D3', '/D4', '/D5', '/D6', '/D7', '/FLAGA', '/FLAGB',
                '/FLAGC', '/FPGA_DONE', 'Net-(U30A-IOT_219)',
                'Net-(U30A-IOT_170)', '--ordering', 'original',
                '--layer-costs', '2.0', '6.0', '1.0', '1.0',
                '--max-ripup', '10', '--via-cost', '25']
        r = subprocess.run(argv, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', env=env,
                           timeout=3600)
        log = (r.stdout or '') + (r.stderr or '')
        m = re.search(r'JSON_IMPROVEMENT_GATE: (\{.*\})', log)
        if r.returncode != 0 or 'Traceback' in log or not m:
            check('K3A: the lap ran and the gate reported', False, log[-1500:])
            return True
        g = json.loads(m.group(1))
        ex = {n: (b, a) for n, b, a in g.get('excluded_by_plan', [])}
        check('K3A: GND is excluded by plan', 'GND' in ex, g)
        check('K3A: the lap CUT the GND pour (more open pads after)',
              'GND' in ex and ex['GND'][1] > ex['GND'][0], ex.get('GND'))
        check('K3A: GND is neither lost nor worsened',
              'GND' not in g['lost']
              and 'GND' not in [n for n, _b, _a in g['worsened']], g)
        check('K3A: the head line labels its lists',
              re.search(r'IMPROVEMENT GATE: this run broke \d+ previously-'
                        r'connected net\(s\).*, worsened \d+.*, connected \d+',
                        log) is not None)
        return True
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    part_a()
    ran_b = part_b()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        sys.exit(1)
    print('all checks passed' + ('' if ran_b else ' (part B skipped)'))
    sys.exit(0 if ran_b else 77)
