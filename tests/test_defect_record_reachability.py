#!/usr/bin/env python3
"""The defect record: what the chain measured, in a shape something can read.

Run 20 measured a cage at pad U4.54 -- throat 0.409 mm against 0.450 needed,
blocked by U4.53 and R7.2 -- and that finding lived in three tools' stdout and
had to be hand-assembled into an English paragraph inside a ledger `lever`
string and a subagent prompt. Nothing downstream could consume it.

Every number in the record was ALREADY COMPUTED and thrown away at the point of
formatting. `widest_path` held the throat cell in a local and returned a bare
float; `pad_reachability` knew the clearance the field was built at and did not
publish it. This is plumbing, not new measurement, and the tests say so by
checking the values against the geometry rather than against each other.

Two traps this file exists to pin:

  * `bottleneck_mm` is in SLACK space (`slack = 2*(dist - clearance)`), so the
    physical gap is `bottleneck + 2*clearance`. Run 20's ledger put a gap-space
    pair (0.409/0.450) beside a track-space margin (-37.69 um) in one sentence
    as if they were the same measurement. The record carries both, labelled.
  * `to_dict()` is PURELY ADDITIVE. Its consumers predate the record.
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_placer'),
           os.path.join(REPO, 'py_tools'), os.path.join(REPO, 'tests')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

SKIP_EXIT = 77
try:
    import numpy  # noqa: F401
    import scipy  # noqa: F401
except ImportError as exc:
    print(f'SKIP: reachability needs numpy+scipy ({exc})')
    sys.exit(SKIP_EXIT)

from placement import reachability as RE                          # noqa: E402
from kicad_parser import BoardInfo, Footprint                     # noqa: E402
import synth                                                      # noqa: E402


def _fp(ref, pads):
    return Footprint(reference=ref, footprint_name='t:x',
                     x=pads[0].global_x, y=pads[0].global_y, rotation=0.0,
                     layer='F.Cu', pads=pads)


def _bi(layers):
    return BoardInfo(layers={i * 2: n for i, n in enumerate(layers)},
                     copper_layers=list(layers), board_bounds=(0, 0, 10, 10),
                     stackup=[], board_outline=[], board_cutouts=[],
                     board_outlines=[], board_edge_contours=[])

passed = failed = 0


def check(label, cond, detail=''):
    global passed, failed
    if cond:
        passed += 1
        print(f'  OK   {label}')
    else:
        failed += 1
        print(f'  FAIL {label} -- {detail}')


# A throat with KNOWN geometry, and it has to be a real WALL: two tall foreign
# pads spanning the whole view with a 0.40 mm slot between them, so the only way
# from the seed island to the target island is through the slot. A wall of two
# small pads is not a throat -- the path simply goes around it, the bottleneck
# is the view's own size, and every assertion below passes vacuously. (The first
# version of this fixture did exactly that, and `caged` was True only because
# the target island was outside the view: a NO-TARGET reported as a cage.)
GAP = 0.40
TALL = 3.0                              # each wall pad's height
PITCH = TALL + GAP                      # centre-to-centre, vertically
SEED = (4.0, 5.0)


def _board():
    own_a = synth.make_pad(1, SEED[0], SEED[1], ref='U1', num='1',
                           net_name='SIG', size_x=0.6, size_y=0.6)
    own_b = synth.make_pad(1, 6.0, 5.0, ref='U2', num='1', net_name='SIG',
                           size_x=0.6, size_y=0.6)
    wall_a = synth.make_pad(2, 5.0, 5.0 - PITCH / 2.0, ref='R7', num='2',
                            net_name='GND', size_x=0.6, size_y=TALL)
    wall_b = synth.make_pad(3, 5.0, 5.0 + PITCH / 2.0, ref='U4', num='3',
                            net_name='VCC', size_x=0.6, size_y=TALL)
    fps = {'U1': _fp('U1', [own_a]), 'U2': _fp('U2', [own_b]),
           'R7': _fp('R7', [wall_a]), 'U4': _fp('U4', [wall_b])}
    nets = {1: synth.make_net(1, 'SIG'), 2: synth.make_net(2, 'GND'),
            3: synth.make_net(3, 'VCC')}
    return synth.make_pcb(nets=nets, footprints=fps,
                          board_info=_bi(['F.Cu']))


CLR = 0.05
TRACK = 0.40                            # deliberately wider than fits
pcb = _board()
r = RE.pad_reachability(pcb, SEED, net_id=1, layers=('F.Cu',),
                        track_mm=TRACK, via_mm=0.6, base_clearance=CLR,
                        step=0.01, margin_mm=2.0)

print('--- the throat is WHERE the geometry says it is ---')
# `measured` FIRST. `caged` is True whenever `bottleneck_mm` is None, which
# includes "nothing of this net to reach inside the view" -- so asserting
# `caged` alone passes on a fixture that measured nothing at all.
check('the question was answerable (there IS a target island in the view)',
      r.measured, f'target_cells={r.target_cells} note={r.note!r}')
check('a path exists, so the bottleneck is a real number not a None',
      r.bottleneck_mm is not None,
      'None means no positive-slack path at any width -- a walled-in seed, '
      'not a throat, and there is nothing to point at')
check('the measurement is CAGED (0.40mm track will not fit a 0.40mm slot '
      'at 0.05 clearance)', r.caged, r.to_dict()['verdict'])
_t = r.throat
check('a throat is reported at all', _t is not None,
      'widest_path held this cell in a local and returned a bare float')
if _t:
    check('and it lies INSIDE the gap, not on either pad',
          abs(_t['x'] - 5.0) < 0.35 and abs(_t['y'] - 5.0) < GAP,
          f"{_t} -- the gap runs through (5.0, 5.0)")
    check('on the layer the field was built on', _t['layer'] == 'F.Cu', str(_t))

print('--- the two SPACES, both published, and convertible ---')
check('gap_mm is the physical gap, within one raster step of the truth',
      r.gap_mm is not None and abs(r.gap_mm - GAP) <= r.step_mm + 1e-9,
      f'{r.gap_mm} vs an analytic {GAP} at step {r.step_mm}')
check('and it is exactly bottleneck + 2*clearance, not a second measurement',
      abs(r.gap_mm - (r.bottleneck_mm + 2 * r.clearance_mm)) < 1e-12,
      f'{r.gap_mm} vs {r.bottleneck_mm} + 2*{r.clearance_mm}')
check('the clearance the field was built at is published',
      abs(r.clearance_mm - CLR) < 1e-12, str(r.clearance_mm))

print('--- who is forming the throat ---')
_refs = {n.get('ref') for n in r.near}
check('the two nearest foreign objects are named, by REF.PAD',
      _refs == {'R7', 'U4'}, str(r.near))
check('and their distances are sorted nearest-first',
      all(r.near[i]['dist'] <= r.near[i + 1]['dist']
          for i in range(len(r.near) - 1)), str([n['dist'] for n in r.near]))

print('--- to_dict stays additive ---')
_LEGACY = {'net', 'seed', 'layers', 'step_mm', 'track_mm', 'via_mm',
           'bottleneck_mm', 'wide_open', 'verdict', 'measured', 'margin_um',
           'target_cells', 'via_legal_fraction', 'grid', 'view', 'note'}
d = r.to_dict()
check('every pre-existing key survives',
      _LEGACY <= set(d),
      f'missing {sorted(_LEGACY - set(d))} -- consumers of this payload '
      f'predate the defect record and must not be broken by it')
check('and the new keys are there beside them',
      {'throat', 'clearance_mm', 'gap_mm', 'near'} <= set(d),
      sorted(set(d) - _LEGACY))

print('--- the record itself ---')
rec = r.defect_record(board='/tmp/x.kicad_pcb', board_sha='deadbeef',
                      floors={'clearance': {'value': CLR,
                                            'source': 'board netclass'}})
check('a CAGED measurement produces a record', isinstance(rec, dict), str(rec))
_d0 = (rec or {}).get('defects', [{}])[0]
check('it is a defect-record v1 with one defect',
      rec.get('kind') == 'defect-record' and rec.get('version') == 1
      and rec.get('count') == 1 and len(rec.get('defects') or []) == 1,
      str(rec)[:200])
check('the defect is a throat with the CAGED verdict',
      _d0.get('kind') == 'throat' and _d0.get('verdict') == 'CAGED', str(_d0)[:200])
_m = _d0.get('measure') or {}
check('short_mm is exactly need - have, not a separate measurement',
      abs(_m.get('short_mm', 0) - (_m['need_mm'] - _m['have_mm'])) < 1e-9,
      str(_m))
check('the measure names its SPACE and its resolution',
      _m.get('space') == 'track_width' and _m.get('resolution_mm') == r.step_mm,
      str(_m))
check('gap space is carried alongside, with what converts them',
      _m.get('gap_mm') is not None and _m.get('gap_need_mm') is not None
      and 'clearance' in (_m.get('derived_from') or ''), str(_m))
check('the blocking refs and pads are named',
      set(_d0.get('refs') or ()) == {'R7', 'U4'}
      and len(_d0.get('pads') or ()) == 2, str(_d0.get('pads')))
check('the record is BOUND to the board it was measured on',
      _d0 and rec.get('board_sha') == 'deadbeef' and rec.get('board'),
      'a record without a sha can be drawn over a board it does not describe')
check('the floors it graded at travel with it, with their sources',
      (_d0.get('instrument') or {}).get('floors', {})
      .get('clearance', {}).get('source') == 'board netclass',
      str(_d0.get('instrument')))

print('--- and NOT produced when there is no defect ---')
r2 = RE.pad_reachability(pcb, SEED, net_id=1, layers=('F.Cu',),
                         track_mm=0.05, via_mm=0.6, base_clearance=CLR,
                         step=0.01, margin_mm=2.0)
check('a PASSABLE measurement makes no record',
      not r2.caged and r2.defect_record() is None,
      f'caged={r2.caged} record={r2.defect_record()}')

# A seed with nothing to reach is NO-TARGET, a third state -- and it must not
# produce a record either, or the loop re-enters placement over a question
# nobody answered (run 15: 7 CAGED reported where 3 was the truth).
r3 = RE.pad_reachability(pcb, SEED, net_id=1, layers=('F.Cu',),
                         track_mm=TRACK, via_mm=0.6, base_clearance=CLR,
                         step=0.01, margin_mm=0.5)
check('a NO-TARGET measurement makes no record either',
      not r3.measured and r3.defect_record() is None,
      f'measured={r3.measured} record={r3.defect_record()}')

print('--- relief_move, the "so what do I do" half ---')
from placement.routability import relief_move                     # noqa: E402
from placement.legality import rect_gap                           # noqa: E402

# The run-20 fixture, from the plan: dx=0.3915, dy=0.12, need=0.45.
_a = (0.0, 0.0, 1.0, 1.0)
_b = (1.0 + 0.3915, 1.0 + 0.12, 2.0, 2.0)
check('the fixture gap is the hand-measured 0.409478',
      abs(rect_gap(_a, _b) - 0.409478) < 1e-6, str(rect_gap(_a, _b)))
_mv = relief_move(_a, _b, 0.45)
_east = next((m for m in _mv if m['dir'] == 'east'), None)
check('and the eastward relief is 0.0422mm',
      _east and abs(_east['min_mm'] - 0.0422) < 1e-4, str(_mv))
check('every result is stamped as a LOWER bound',
      all(m['bound'] == 'lower' for m in _mv),
      'moving a part changes the whole field; this clears THIS pair only')
_moved = (_b[0] + _east['min_mm'], _b[1], _b[2] + _east['min_mm'], _b[3])
check('and the bound is real -- applying it reaches the need',
      rect_gap(_a, _moved) >= 0.45 - 1e-9, str(rect_gap(_a, _moved)))

# Overlapping rects: the gap is negative, so every direction needs a real move.
_ov = relief_move((0.0, 0.0, 2.0, 2.0), (1.0, 1.0, 3.0, 3.0), 0.45)
check('overlapping rects still get an answer', bool(_ov), str(_ov))
check('and none of those answers is zero',
      all(m['min_mm'] > 0 for m in _ov), str(_ov))

# Infeasible within the cap: omitted, never reported as a large number.
_inf = relief_move((0.0, 0.0, 100.0, 1.0), (0.0, 1.05, 100.0, 2.0), 0.45,
                   limit_mm=0.1)
check('a direction that cannot reach the need within the cap is omitted',
      not any(m['dir'] in ('north', 'south') for m in _inf), str(_inf))

# Already clear: zero, not a bisection artefact.
_clear = relief_move((0.0, 0.0, 1.0, 1.0), (2.0, 0.0, 3.0, 1.0), 0.45)
check('a pair already at the need reports 0.0',
      _clear and all(m['min_mm'] == 0.0 for m in _clear), str(_clear))

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
