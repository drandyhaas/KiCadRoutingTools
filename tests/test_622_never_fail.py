#!/usr/bin/env python3
"""The whole route never ends with nothing: the pieces that rank and feed back a round's result.

  python3 tests/test_622_never_fail.py

whole_route.chain lays every round it can -- a plan the loop held when it did
not pass, an unproved solve's plan, a partial plan when no round laid anything --
and returns the best: the fewest open nets, then the fewest vias. route_bus then hands on the nets laid connected and puts
the open ones back as they came. This pins the parts that read and write what
those decisions rest on, without a bench to route:

1. count_open: check_connected's log read as the nets open -- unrouted (no copper)
   and in pieces both count;
2. route_bus.open_nets: the same log read as the open nets' names (short, the
   bench's '/DDR3 16x1/SDQ15' read as SDQ15);
3. add_over: the lanes an unproved plan leaves over two vias merged into the
   fanout's feedback ('over'), the lanes it newly names returned, a lane raised
   only by more, and the audits' own feedback (pairs, avoid) left as it was;
4. held_plan: a loop's snapped plan when it has one; nothing when it held none;
5. whole_feedback --refused: the fanout audit's split pair (SDQS0 at its berth, SDQ3 between its tips) fed back
   as the two lanes' berths not to be chosen together -- the next round, incremental, frees both -- and a tooth on
   the source's far face (SDQ7) as an end to avoid;
6. whole_feedback --name: a round that left nets open names their lanes (a leg read as its pair), and one that
   laid nothing names lanes from the ends model's reading -- its nets over two first, then (named again) its lanes
   loaded past LOAD_OK, then its most crossed -- both ends of each to avoid, the lanes kept in 'named' in the order
   named, and nothing new once every tier is spent;
7. partial_drops: the last resort's sets of lanes left out, each larger -- the named, then to a quarter, then to half
   by crossings -- and never every lane;
8. route_bus.drc_nets: the nets check_drc's log names in its violations -- both sides of a line between two items
   (a kind prefix, a pad's ref, a [SHORT], a measurement stripped), a net's own line -- and none from before the
   violations or from the warnings after them: the nets route_bus puts back rather than refuse the whole bus.
"""
import subprocess
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import whole_route as wr  # noqa: E402
import route_bus as rb  # noqa: E402

DRC_LOG = """Checking 9 nets matching: ['*SDQ15']
  /DDR3 16x1/SDQ1 (a line before the violations)
============================================================
FOUND 4 DRC VIOLATIONS:

SEGMENT-SEGMENT violations (1):
----------------------------------------
  Seg:/DDR3 16x1/SDQ15 <-> Seg:/DDR3 16x1/SA4 [SHORT]
    Layer: F.Cu, at (120.0, 64.0)
VIA-PAD violations (1):
----------------------------------------
  Pad:/DDR3 16x1/SCKP (U1.A1) <-> Via:GND
SEGMENT-ENDPOINT-GAP violations (1):
----------------------------------------
  /DDR3 16x1/SBA1 (same-net soft joint)
    Layer: F.Cu, endpoint gap: 0.080mm (cap overlap only 0.018mm)
    Ends: (91.200,-48.500) <-> (91.154,-48.434)
  /DDR3 16x1/SDQ2 <-> /DDR3 16x1/SDQ3: dist=0.1000mm, required=0.150mm, violation=0.0500mm

LISTING: 4 of 4 violation(s) shown

WARNINGS (1, not DRC failures):
  /DDR3 16x1/SDQ9 same-net self-crossing: 1
"""

LOG = """Checking 6 nets matching: ['*SDQ15', '*SA4']

============================================================
FOUND 3 ISSUES:

  Unrouted nets (1):
    /DDR3 16x1/SA4 (2 pads)

  Connectivity issues (2):

  /DDR3 16x1/SCKP (net 12):
    Segments: 4, Vias: 1, Pads: 2
    Disconnected components: 2
    Disconnected pads:
      (120.10, 64.00) on F.Cu [U1]
  /DDR3 16x1/SCKN (net 13):
    Segments: 4, Vias: 1, Pads: 2
    Disconnected components: 2
============================================================
"""


def main():
    print('=' * 60)
    print('never fail: ranking and feeding back a round')
    print('=' * 60)
    fails = []
    with tempfile.TemporaryDirectory() as td:
        cl = os.path.join(td, 'conn.log')
        open(cl, 'w').write(LOG)
        n = wr.count_open(cl)
        if n != 3:
            fails.append(f'count_open: {n}, want 3 (1 unrouted + 2 in pieces)')
        dl = os.path.join(td, 'drc.log')
        open(dl, 'w').write(DRC_LOG)
        got = rb.drc_nets(dl)
        if got != {'SDQ15', 'SA4', 'SCKP', 'GND', 'SBA1', 'SDQ2', 'SDQ3'}:
            fails.append(f'drc_nets: {sorted(got)}, want GND SA4 SBA1 SCKP SDQ15 SDQ2 SDQ3')
        names = rb.open_nets(cl)
        if names != {'SA4', 'SCKP', 'SCKN'}:
            fails.append(f'open_nets: {sorted(names)}, want SA4 SCKN SCKP')
        fb = os.path.join(td, 'feedback.json')
        json.dump({'pairs': [['a', 'b']], 'avoid': [{'lane': 'SA1', 'end': 1}]}, open(fb, 'w'))
        new = wr.add_over(fb, {'SCK': 1})
        if new != ['SCK']:
            fails.append(f'add_over: first {new}, want [SCK]')
        if wr.add_over(fb, {'SCK': 1}) != []:
            fails.append('add_over: the same lane at the same count named again')
        new = wr.add_over(fb, {'SCK': 2, 'SA4': 1})
        if new != ['SA4', 'SCK']:
            fails.append(f'add_over: {new}, want [SA4, SCK] (SA4 new, SCK raised)')
        j = json.load(open(fb))
        if j.get('over') != {'SCK': 2, 'SA4': 1} or j.get('pairs') != [['a', 'b']] or len(j.get('avoid', [])) != 1:
            fails.append(f'add_over: the feedback file reads {j}')
        if wr.add_over(fb, None) != [] or wr.add_over(fb, {}) != []:
            fails.append('add_over: a proved plan (no over_nets) named lanes')
        loopd = os.path.join(td, 'loop')
        os.makedirs(loopd)
        if wr.held_plan(loopd, dict(os.environ)) is not None:
            fails.append('held_plan: a loop that held nothing gave a plan')
        open(os.path.join(loopd, 'plan.json'), 'w').write('{}')
        hp = wr.held_plan(loopd, dict(os.environ))
        if hp != os.path.join(loopd, 'plan.json'):
            fails.append(f'held_plan: {hp}, want the loop\'s snapped plan.json')
        side = os.path.join(td, 'fo.plan.json')
        json.dump({'chi': 1, 'ends': {'SDQS0P': [[127.0, 66.0], [139.0, 68.2]], 'SDQS0N': [[127.0, 66.3],
                                                                                         [139.8, 68.2]],
                                      'SDQ3': [[127.0, 67.8], [139.4, 68.2]]},
                   'tooth_layer': {'SDQS0P': 'F.Cu', 'SDQS0N': 'F.Cu', 'SDQ3': 'F.Cu'},
                   'dest_layer': {'SDQS0P': 'F.Cu', 'SDQS0N': 'F.Cu', 'SDQ3': 'F.Cu'}}, open(side, 'w'))
        ref = os.path.join(td, 'fo.refused.json')
        json.dump({'splits': [['SDQS0', 'berth', ['SDQ3']]], 'far': ['SDQ3'], 'stops': []}, open(ref, 'w'))
        fb2 = os.path.join(td, 'feedback2.json')
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'awx', 'whole_feedback.py'), '--refused', side, fb2, ref],
                           capture_output=True, text=True, cwd=os.path.join(ROOT, 'awx'),
                           env=dict(os.environ, PLAN_PAIRS='1', BRAID_PAIRS='1'))
        got = json.load(open(fb2)) if os.path.isfile(fb2) else {}
        items = got.get('pairs', [])
        want = {('SDQ3', 1), ('SDQS0', 1)}
        if r.returncode != 0 or 'whole_feedback: 2 new' not in r.stdout or len(items) != 1 or \
                {(e['lane'], e['end']) for e in items[0]} != want or len(items[0][1]['points']) != 2 or \
                [(e['lane'], e['end']) for e in got.get('avoid', [])] != [('SDQ3', 0)]:
            fails.append(f'whole_feedback --refused: exit {r.returncode}, {r.stdout.strip()[:120]} {r.stderr[-300:]}, '
                         f'items {items}, avoid {got.get("avoid")}')
        # --name, round after round on the same ends: the open nets given (SDQ3, and SDQS0N read as its pair), then
        # with none given the ends model's tiers in turn, then nothing new
        S = json.load(open(side))
        S['ends'].update({'SA4': [[127.0, 68.5], [139.0, 69.0]], 'SBA1': [[127.0, 69.5], [139.0, 70.0]]})
        for nm in ('SA4', 'SBA1'):
            S['tooth_layer'][nm] = S['dest_layer'][nm] = 'F.Cu'
        S['ends_model'] = {'over': {'SDQS0': 1}, 'load': {'SDQS0': 0.9, 'SDQ3': 0.7, 'SA4': 0.6, 'SBA1': 0.2},
                           'x': {'SBA1': 9, 'SA4': 8, 'SDQ3': 7, 'SDQS0': 5}}
        json.dump(S, open(side, 'w'))
        fb3 = os.path.join(td, 'feedback3.json')
        seq = []
        for k_ in range(5):
            r = subprocess.run([sys.executable, os.path.join(ROOT, 'awx', 'whole_feedback.py'), '--name', side, fb3]
                               + (['SDQ3', 'SDQS0N'] if k_ == 0 else []),
                               capture_output=True, text=True, cwd=os.path.join(ROOT, 'awx'),
                               env=dict(os.environ, PLAN_PAIRS='1', BRAID_PAIRS='1'))
            got = json.load(open(fb3)) if os.path.isfile(fb3) else {}
            seq.append((r.returncode, r.stdout.strip(), list(got.get('named', [])),
                        sorted((e['lane'], e['end']) for e in got.get('avoid', []))))
        want_np = [['SDQ3', 'SDQS0'], ['SDQ3', 'SDQS0', 'SA4'], ['SDQ3', 'SDQS0', 'SA4', 'SBA1'],
                   ['SDQ3', 'SDQS0', 'SA4', 'SBA1'], ['SDQ3', 'SDQS0', 'SA4', 'SBA1']]
        want_new = ['4 new', '2 new', '2 new', '0 new', '0 new']
        for k, (rc_, out_, np_, av_) in enumerate(seq):
            if rc_ != 0 or np_ != want_np[k] or f'whole_feedback: {want_new[k]}' not in out_ or \
                    len(av_) != 2 * len(np_) or {e for e, _k in av_} != set(np_):
                fails.append(f'whole_feedback --name, call {k + 1}: exit {rc_}, {out_[:160]}, named {np_} (want '
                             f'{want_np[k]}), avoid {av_}')
                break
        # the last resort's drop sets
        lanes = [f'L{i}' for i in range(10)]
        xs = {f'L{i}': 10 - i for i in range(10)}
        got = wr.partial_drops(['L7'], xs, lanes)
        if got != [['L7'], ['L7', 'L0', 'L1'], ['L7', 'L0', 'L1', 'L2', 'L3']]:
            fails.append(f'partial_drops (named L7): {got}')
        got = wr.partial_drops([], xs, lanes)
        if got != [['L0', 'L1', 'L2'], ['L0', 'L1', 'L2', 'L3', 'L4']]:
            fails.append(f'partial_drops (none named): {got}')
        got = wr.partial_drops(['L0', 'L1'], xs, ['L0', 'L1'])
        if got != [['L0']]:
            fails.append(f'partial_drops (two lanes, both named): {got} -- never every lane')
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: open nets counted and named, over-two lanes fed back, the held plan found, a round that laid '
          'nothing named its lanes, the last resort\'s drop sets, the nets a violation names')
    return 0


if __name__ == '__main__':
    sys.exit(main())
