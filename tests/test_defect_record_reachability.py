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

import math                                                       # noqa: E402
import numpy as np                                                # noqa: E402
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


def _board(walls=True, gap=GAP, tall=TALL, layer='F.Cu', layers=('F.Cu',),
           extra=(), wall_layers=None, wall_rot=0.0):
    """The two-wall fixture, with the knobs the later probes vary.

    `gap`/`tall` size the slot and the walls, `layer` puts the whole thing on
    one copper face, `wall_layers` overrides just the WALLS' layer list (a pad
    declared `*.Cu` is a different thing to the layer predicate), `wall_rot`
    gives them a residual rect tilt, and `extra` adds foreign footprints (used
    to park copper on the OTHER face).
    """
    pitch = tall + gap
    own_a = synth.make_pad(1, SEED[0], SEED[1], ref='U1', num='1',
                           net_name='SIG', size_x=0.6, size_y=0.6,
                           layers=(layer,))
    own_b = synth.make_pad(1, 6.0, 5.0, ref='U2', num='1', net_name='SIG',
                           size_x=0.6, size_y=0.6, layers=(layer,))
    fps = {'U1': _fp('U1', [own_a]), 'U2': _fp('U2', [own_b])}
    nets = {1: synth.make_net(1, 'SIG'), 2: synth.make_net(2, 'GND'),
            3: synth.make_net(3, 'VCC')}
    if walls:
        _wl = tuple(wall_layers) if wall_layers else (layer,)
        wall_a = synth.make_pad(2, 5.0, 5.0 - pitch / 2.0, ref='R7', num='2',
                                net_name='GND', size_x=0.6, size_y=tall,
                                layers=_wl, rect_rotation=wall_rot)
        wall_b = synth.make_pad(3, 5.0, 5.0 + pitch / 2.0, ref='U4', num='3',
                                net_name='VCC', size_x=0.6, size_y=tall,
                                layers=_wl, rect_rotation=wall_rot)
        fps['R7'] = _fp('R7', [wall_a])
        fps['U4'] = _fp('U4', [wall_b])
    for _ref, _pad in extra:
        fps[_ref] = _fp(_ref, [_pad])
    return synth.make_pcb(nets=nets, footprints=fps,
                          board_info=_bi(list(layers)))


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
check('and it is exactly bottleneck + the two blocking clearances, not a '
      'second measurement',
      abs(r.gap_mm - (r.bottleneck_mm + sum(r.binding_clearances))) < 1e-12,
      f'{r.gap_mm} vs {r.bottleneck_mm} + {r.binding_clearances}')
check('which here, with both walls on the base clearance, IS 2*clearance',
      abs(r.gap_mm - (r.bottleneck_mm + 2 * r.clearance_mm)) < 1e-12,
      f'{r.gap_mm} vs {r.bottleneck_mm} + 2*{r.clearance_mm}')
check('the clearance the field was built at is published',
      abs(r.clearance_mm - CLR) < 1e-12, str(r.clearance_mm))
check('gap_need_mm is the same conversion applied to the track width',
      r.gap_need_mm is not None
      and abs(r.gap_need_mm - (TRACK + sum(r.binding_clearances))) < 1e-12,
      f'{r.gap_need_mm} vs {TRACK} + {r.binding_clearances}')

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
      {'throat', 'clearance_mm', 'gap_mm', 'gap_need_mm', 'near'} <= set(d),
      sorted(set(d) - _LEGACY))
check('gap_need_mm rides beside gap_mm, because the conversion is no longer '
      'derivable from clearance_mm alone',
      d['gap_need_mm'] is not None and d['gap_mm'] is not None, str(d))

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
# `instrument` is a real parameter, not a decorative default: the CLI passes
# its own name. It was defaulted and never passed at any call site, which is
# how a default becomes a hardcoded string nobody notices is wrong.
check('the instrument name is what the caller passed, not a hardcoded default',
      (r.defect_record(instrument='a-probe') or {})['defects'][0]
      ['instrument']['source'] == 'a-probe',
      str(r.defect_record(instrument='a-probe')))
import inspect                                                     # noqa: E402
_sig = set(inspect.signature(RE.nearest_foreign).parameters)
check('nearest_foreign has no never-varied k / radius_mm knobs',
      not (_sig & {'k', 'radius_mm'}),
      f'{sorted(_sig)} -- radius_mm=2.0 read as a knob and was not one; it '
      f'silently dropped every blocker bigger than about 4mm')
_src = open(os.path.join(REPO, 'py_tools', 'check_reachability.py'),
            encoding='utf-8').read()
check('and the CLI passes instrument= at its call site',
      "instrument='check_reachability'" in _src,
      'the only writer of a record must name itself explicitly')
check('and no longer imports a sibling CLI\'s private _components',
      'from net_forensics import' not in _src,
      'py_tools is not on sys.path via _path, so that ImportError left as '
      'exit 1 -- the CAGED verdict')

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

print('--- a WIDE-OPEN result has no throat at all ---')
# The same two own pads with the walls removed: the widest path never nears
# foreign copper, so the bottleneck is the view's own size. The cell that
# closed the path is then just the last free cell Kruskal activated -- a point
# in open board, not a throat -- and the first version reported it anyway,
# with the copper "nearest" to it and a view-sized "gap": check_reachability
# printed "There is no throat here to measure" and, two lines later,
# "throat (...) gap 34.18mm vs 0.55mm needed".
r4 = RE.pad_reachability(_board(walls=False), SEED, net_id=1,
                         layers=('F.Cu',), track_mm=TRACK, via_mm=0.6,
                         base_clearance=CLR, step=0.01, margin_mm=2.0)
check('the fixture is measured, wide open and PASSABLE',
      r4.measured and r4.wide_open and not r4.caged,
      f'measured={r4.measured} wide_open={r4.wide_open} caged={r4.caged} '
      f'bottleneck={r4.bottleneck_mm} open_room={r4.open_room_mm}')
check('so there is no throat', r4.throat is None and r4.throat_cell is None,
      str(r4.throat))
check('no copper "nearest" to a point that is not a throat', r4.near == (),
      str(r4.near))
check('and no gap: a gap is between two pieces of copper, and there are none',
      r4.gap_mm is None, str(r4.gap_mm))
d4 = r4.to_dict()
check('the dict says the same (throat/near/gap null, wide_open set)',
      d4['throat'] is None and d4['near'] == [] and d4['gap_mm'] is None
      and d4['wide_open'] and d4['verdict'] == 'PASSABLE', str(d4))
check('bottleneck_mm still reports the view-bounded number it always did',
      d4['bottleneck_mm'] is not None and d4['bottleneck_mm'] > 1.0,
      str(d4['bottleneck_mm']))
check('and no record', r4.defect_record() is None, str(r4.defect_record()))
# The caged fixture, for contrast: it is NOT wide open, so the throat stays.
check('the caged fixture is not wide open and keeps its throat',
      not r.wide_open and r.throat is not None, f'wide_open={r.wide_open}')

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

print('--- the gap is in the clearance the BLOCKERS are on, not the base one ---')
# The whole point of the per-net map `slack_field` already builds: clearance is
# PAIRWISE. Same 1.00mm slot, same base clearance, twice -- once with the walls
# on the Default class and once with them on a 0.3mm class. The GEOMETRY does
# not change, so the physical gap must not either. The first version computed
# `bottleneck + 2 * base_clearance` and reported 1.00 then 0.50: off by
# 2 * (class - base), and in the direction that turns a passable slot into a
# reported cage.
WIDE = 1.00
_pcbw = _board(gap=WIDE, tall=3.0)
_g = {}
for _label, _nc in (('base', None), ('class 0.3', {2: 0.3, 3: 0.3})):
    _rw = RE.pad_reachability(_pcbw, SEED, net_id=1, layers=('F.Cu',),
                              track_mm=0.2, via_mm=0.6, base_clearance=CLR,
                              net_clearances=_nc, step=0.01, margin_mm=2.0)
    _g[_label] = _rw
    check(f'the {WIDE}mm slot measures {WIDE}mm edge-to-edge with the walls '
          f'on the {_label} clearance',
          _rw.gap_mm is not None and abs(_rw.gap_mm - WIDE) <= _rw.step_mm + 1e-9,
          f'gap_mm={_rw.gap_mm} bottleneck={_rw.bottleneck_mm} '
          f'binding={_rw.binding_clearances} -- the geometry is identical in '
          f'both runs, so the physical gap must be too')
check('the two runs agree on the gap to within a raster step',
      abs(_g['base'].gap_mm - _g['class 0.3'].gap_mm) <= 0.01 + 1e-9,
      f"{_g['base'].gap_mm} vs {_g['class 0.3'].gap_mm}")
check('and they DISAGREE on the bottleneck, which is the point: a track has '
      'less room on the wider class',
      _g['class 0.3'].bottleneck_mm < _g['base'].bottleneck_mm - 0.4,
      f"{_g['base'].bottleneck_mm} vs {_g['class 0.3'].bottleneck_mm}")
check('the clearance in force at each blocker is published on the entry',
      all(abs(n['clearance'] - 0.3) < 1e-12 for n in _g['class 0.3'].near),
      str([n.get('clearance') for n in _g['class 0.3'].near]))
check('binding_clearances reads them back',
      _g['class 0.3'].binding_clearances == (0.3, 0.3),
      str(_g['class 0.3'].binding_clearances))
_recw = _g['class 0.3'].defect_record(board='/tmp/x.kicad_pcb')
check('a PASSABLE wide slot still makes no record', _recw is None, str(_recw))
# ...and the record's own conversion, on a CAGED reading of the same slot.
_rwc = RE.pad_reachability(_pcbw, SEED, net_id=1, layers=('F.Cu',),
                           track_mm=0.8, via_mm=0.6, base_clearance=CLR,
                           net_clearances={2: 0.3, 3: 0.3}, step=0.01,
                           margin_mm=2.0)
_mw = (_rwc.defect_record(board='/tmp/x.kicad_pcb') or {}).get(
    'defects', [{}])[0].get('measure', {})
check('the record carries the same gap the property does',
      _mw.get('gap_mm') is not None and abs(_mw['gap_mm'] - WIDE) <= 0.01 + 1e-9,
      str(_mw))
check('and its gap_need_mm converts the NEED with the same two clearances',
      abs(_mw.get('gap_need_mm', 0) - (0.8 + 0.6)) < 1e-9, str(_mw))
check('and derived_from names the two blockers, not twice the base clearance',
      'span.a.clearance' in (_mw.get('derived_from') or ''), str(_mw))

print('--- a blocker bigger than the search radius is still NAMED ---')
# Centre distances dropped it. A 5mm-tall wall pad 0.2mm from the throat has
# its centre 2.7mm away, so a CAGED verdict shipped `near: ()`, `refs: []`,
# `pads: []`, no span and no relief -- the record's entire purpose. Any pad or
# connector wider than about 4mm hit this.
_pcbb = _board(gap=0.40, tall=5.0)
_rb = RE.pad_reachability(_pcbb, SEED, net_id=1, layers=('F.Cu',),
                          track_mm=TRACK, via_mm=0.6, base_clearance=CLR,
                          step=0.01, margin_mm=3.0)
check('the big-wall fixture is CAGED (that is what the record describes)',
      _rb.measured and _rb.caged, _rb.to_dict()['verdict'])
check('and the 5mm walls are named, at their EDGE distance',
      {n.get('ref') for n in _rb.near} == {'R7', 'U4'},
      f'{_rb.near} -- the wall centres are 2.7mm away; their edges are 0.2mm')
check('the reported distances are edge distances, not centre distances',
      all(n['dist'] < 0.5 for n in _rb.near),
      str([n['dist'] for n in _rb.near]))
_recb = (_rb.defect_record(board='/tmp/x.kicad_pcb') or {})
_db = (_recb.get('defects') or [{}])[0]
check('so the record names the refs, the pads and the span',
      set(_db.get('refs') or ()) == {'R7', 'U4'}
      and len(_db.get('pads') or ()) == 2 and _db.get('span'),
      f"refs={_db.get('refs')} pads={_db.get('pads')} span={_db.get('span')}")

print('--- only copper on the THROAT\'s own layer can bound it ---')
# Probed on a two-layer board: a B.Cu throat whose near[0] was an F.Cu SMD pad.
# `check_reachability._relief_for` consumes near[0]/near[1], so that became
# advice to move a part on the other side of the board.
_far = synth.make_pad(2, 5.0, 5.0, ref='F9', num='1', net_name='GND',
                      size_x=6.0, size_y=6.0, layers=('F.Cu',))
_pcbl = _board(gap=0.40, tall=3.0, layer='B.Cu', layers=('B.Cu', 'F.Cu'),
               extra=[('F9', _far)])
_rl = RE.pad_reachability(_pcbl, SEED, net_id=1, layers=('B.Cu', 'F.Cu'),
                          track_mm=TRACK, via_mm=0.6, base_clearance=CLR,
                          step=0.01, margin_mm=2.0)
check('the throat is on B.Cu (F.Cu is covered, so the path must use the slot)',
      (_rl.throat or {}).get('layer') == 'B.Cu', str(_rl.throat))
check('and nothing on F.Cu is named as forming it',
      _rl.near and all(n['layer'] == 'B.Cu' for n in _rl.near),
      f"{_rl.near} -- F9 is a 6x6mm F.Cu pad centred ON the throat and cannot "
      f"bound a B.Cu throat")
check('the B.Cu walls are what is named',
      {n.get('ref') for n in _rl.near} == {'R7', 'U4'}, str(_rl.near))

print('--- a `*.Cu` pad forms the throat, so it must be NAMED as one ---')
# Blocker 3 through a second door. The naming walk had its own layer test
# (`set(p.layers) & lay`) instead of `_pad_on_layer`, the predicate the FIELD
# stamps with. A pad declared on `*.Cu` is stamped, forms the throat, and was
# then refused by the naming walk: same CAGED verdict, near=[] refs=[] again.
_star = {}
for _lbl, _wl in (('F.Cu', ('F.Cu',)), ('*.Cu', ('*.Cu',))):
    _rs = RE.pad_reachability(_board(wall_layers=_wl), SEED, net_id=1,
                              layers=('F.Cu',), track_mm=TRACK, via_mm=0.6,
                              base_clearance=CLR, step=0.01, margin_mm=2.0)
    _star[_lbl] = _rs
    _ds = ((_rs.defect_record(board='/tmp/x.kicad_pcb') or {})
           .get('defects') or [{}])[0]
    check(f'walls declared on {_lbl}: the same walls are named',
          {n.get('ref') for n in _rs.near} == {'R7', 'U4'}
          and set(_ds.get('refs') or ()) == {'R7', 'U4'}
          and bool(_ds.get('span')),
          f"near={_rs.near} refs={_ds.get('refs')} span={bool(_ds.get('span'))}")
check('and the two declarations measure the same bottleneck, because the '
      'FIELD never disagreed -- only the naming walk did',
      abs(_star['F.Cu'].bottleneck_mm - _star['*.Cu'].bottleneck_mm) < 1e-12,
      f"{_star['F.Cu'].bottleneck_mm} vs {_star['*.Cu'].bottleneck_mm}")

print('--- a pad\'s residual tilt is part of its geometry ---')
# `_point_rect_dist` rotates the point into the pad frame with the SAME
# convention `_rect` stamps with. Ignoring the tilt puts the blocker distance
# (and therefore which two objects are named) on a different rectangle to the
# one the bottleneck was measured from.
_ANALYTIC = 0.9 * math.sqrt(0.5) - 0.1     # a 2.0 x 0.2 pad turned 45 degrees
check('a point 0.9mm off centre is INSIDE an axis-aligned 2.0x0.2 pad',
      RE._point_rect_dist(0.9, 0.0, 0.0, 0.0, 2.0, 0.2, 0.0) == 0.0,
      str(RE._point_rect_dist(0.9, 0.0, 0.0, 0.0, 2.0, 0.2, 0.0)))
check('and OUTSIDE the same pad turned 45 degrees, by the analytic amount',
      abs(RE._point_rect_dist(0.9, 0.0, 0.0, 0.0, 2.0, 0.2, 45.0)
          - _ANALYTIC) < 1e-9,
      f'{RE._point_rect_dist(0.9, 0.0, 0.0, 0.0, 2.0, 0.2, 45.0)} vs '
      f'{_ANALYTIC}')
check('the tilt turns the same way for -45 (the distance is symmetric here)',
      abs(RE._point_rect_dist(0.9, 0.0, 0.0, 0.0, 2.0, 0.2, -45.0)
          - _ANALYTIC) < 1e-9,
      str(RE._point_rect_dist(0.9, 0.0, 0.0, 0.0, 2.0, 0.2, -45.0)))
# ...and the raster agrees, which is the property that matters: one geometry,
# two consumers. Stamp a 45-degree pad and compare the occupancy against the
# zero-set of `_point_rect_dist` over every cell.
_occ = np.zeros((60, 60), dtype=bool)
_view = (-1.5, -1.5, 1.5, 1.5)
RE._rect(_occ, 0.0, 0.0, 2.0, 0.2, _view, 0.05, 45.0)
_mismatch = 0
for _iy in range(60):
    for _ix in range(60):
        _px = _view[0] + (_ix + 0.5) * 0.05
        _py = _view[1] + (_iy + 0.5) * 0.05
        _inside = RE._point_rect_dist(_px, _py, 0.0, 0.0, 2.0, 0.2, 45.0) <= 0.0
        if bool(_occ[_iy, _ix]) != _inside:
            _mismatch += 1
check('the distance function and the rasteriser agree on the tilted pad, '
      'cell for cell', _mismatch == 0,
      f'{_mismatch} of 3600 cells disagree -- the naming walk and the field '
      f'would be measuring two different rectangles')
_rrot = RE.pad_reachability(_board(wall_rot=30.0), SEED, net_id=1,
                            layers=('F.Cu',), track_mm=TRACK, via_mm=0.6,
                            base_clearance=CLR, step=0.01, margin_mm=2.0)
check('a tilted wall still names itself at a tilt-aware distance',
      _rrot.near and all(
          abs(n['dist'] - RE._point_rect_dist(
              _rrot.throat['x'], _rrot.throat['y'], n['x'], n['y'],
              0.6, TALL, 30.0)) < 2e-4 for n in _rrot.near),
      str(_rrot.near))

print('--- how many blockers the gap conversion actually knew about ---')
check('the two-wall fixture knows both, so gap_mm IS an edge-to-edge distance',
      r.gap_blockers == 2 and d['gap_blockers'] == 2,
      f'{r.gap_blockers} / {d.get("gap_blockers")}')
# One blocker: the only foreign copper in the view is a single small pad
# between the seed and its partner, so the widest path detours round it and
# the throat has exactly one named side. `gap_mm` is then twice the room on
# that side, and the count says so rather than the label claiming two objects
# that were never identified.
_one_nets = {1: synth.make_net(1, 'SIG'), 2: synth.make_net(2, 'GND')}
_one = synth.make_pcb(
    nets=_one_nets, board_info=_bi(['F.Cu']),
    footprints={
        'U1': _fp('U1', [synth.make_pad(1, 4.0, 5.0, ref='U1', num='1',
                                        net_name='SIG', size_x=0.6,
                                        size_y=0.6)]),
        'U2': _fp('U2', [synth.make_pad(1, 4.8, 5.0, ref='U2', num='1',
                                        net_name='SIG', size_x=0.6,
                                        size_y=0.6)]),
        'R9': _fp('R9', [synth.make_pad(2, 4.4, 5.0, ref='R9', num='1',
                                        net_name='GND', size_x=0.2,
                                        size_y=0.2)])})
_r1 = RE.pad_reachability(_one, SEED, net_id=1, layers=('F.Cu',),
                          track_mm=1.2, via_mm=0.6, base_clearance=CLR,
                          step=0.01, margin_mm=1.0)
check('the single-blocker fixture is measured, not wide open, and CAGED',
      _r1.measured and not _r1.wide_open and _r1.caged,
      f'measured={_r1.measured} wide_open={_r1.wide_open} caged={_r1.caged}')
check('a single-blocker throat reports gap_blockers 1, not 2',
      _r1.gap_blockers == 1 and len(_r1.near) == 1,
      f'gap_blockers={_r1.gap_blockers} near={_r1.near}')
check('and to_dict publishes the count beside the gap',
      _r1.to_dict()['gap_blockers'] == 1, str(_r1.to_dict()['gap_blockers']))
_m1 = (((_r1.defect_record(board='/tmp/x.kicad_pcb') or {})
        .get('defects') or [{}])[0].get('measure') or {})
check('and the record says the far side was ASSUMED, not measured',
      _m1.get('gap_blockers') == 1
      and 'assumed' in (_m1.get('derived_from') or ''), str(_m1))
check('while the two-wall record still names both blockers as the source',
      (_d0.get('measure') or {}).get('gap_blockers') == 2
      and 'span.a.clearance' in ((_d0.get('measure') or {})
                                 .get('derived_from') or ''),
      str(_d0.get('measure')))

# ---------------------------------------------------------------------------
# The SEVEREST caged result -- a seed ringed by copper, so the search never
# gets out and there is no throat at all -- still writes a record, and that
# record must not describe a derivation nobody made. `gap_mm` is null there,
# so `derived_from` is null too: a sentence naming "the ONE blocker found"
# beside `gap_blockers: 0` is exactly the phantom this instrument exists to
# stop, and it is the reading a grader would act on.
_ring_nets = {1: synth.make_net(1, 'SIG'), 2: synth.make_net(2, 'GND')}
_ring_wall = []
for _i, (_wx, _wy, _sx, _sy) in enumerate((
        (5.0, 4.3, 2.0, 0.4), (5.0, 5.7, 2.0, 0.4),
        (4.3, 5.0, 0.4, 2.0), (5.7, 5.0, 0.4, 2.0))):
    _ring_wall.append(synth.make_pad(2, _wx, _wy, ref='W1', num=str(_i + 1),
                                     net_name='GND', size_x=_sx, size_y=_sy))
_ring = synth.make_pcb(
    nets=_ring_nets, board_info=_bi(['F.Cu']),
    footprints={
        'U1': _fp('U1', [synth.make_pad(1, 5.0, 5.0, ref='U1', num='1',
                                        net_name='SIG', size_x=0.3,
                                        size_y=0.3)]),
        # INSIDE the view, or this fixture is NO-TARGET rather than a ring,
        # `defect_record` returns None, and every assertion below passes
        # against an empty dict. That vacuity is the trap this file exists to
        # avoid, so the target sits where the margin reaches it.
        'U2': _fp('U2', [synth.make_pad(1, 6.8, 5.0, ref='U2', num='1',
                                        net_name='SIG', size_x=0.3,
                                        size_y=0.3)]),
        'W1': _fp('W1', _ring_wall)})
_rr = RE.pad_reachability(_ring, (5.0, 5.0), net_id=1, layers=('F.Cu',),
                          track_mm=0.2, via_mm=0.4, base_clearance=CLR,
                          step=0.02, margin_mm=2.5)
check('the ringed seed is MEASURED and CAGED with no throat at all',
      _rr.measured and _rr.caged and _rr.bottleneck_mm is None
      and _rr.gap_mm is None and _rr.gap_blockers == 0,
      f'measured={_rr.measured} caged={_rr.caged} '
      f'bottleneck={_rr.bottleneck_mm} gap={_rr.gap_mm}')
_dr = ((_rr.defect_record(board='/tmp/x.kicad_pcb') or {})
       .get('defects') or [{}])[0]
_mr = _dr.get('measure') or {}
check('it still writes a record (the severest caged is the one worth having)',
      bool(_dr) and bool(_mr), str(_dr)[:200])
check('and derived_from is null, not a sentence about a blocker never found',
      'derived_from' in _mr and _mr['derived_from'] is None,
      f"gap_blockers={_mr.get('gap_blockers')} "
      f"derived_from={_mr.get('derived_from')!r} -- with no gap derived, a "
      f"sentence here would describe arithmetic that did not happen")


print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
