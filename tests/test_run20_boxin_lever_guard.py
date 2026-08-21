#!/usr/bin/env python3
"""A grid lever the router has already said cannot help.

Run 20's router printed "boxed in by static obstacles (neighboring pads +
clearance), not by congestion" on every residual failure. Three grid
refinements were spent against it -- 0.05 -> 0.025 -> 0.0125, about 40 minutes
-- with `unrouted` at exactly BUSY / Net-(U4-XTAL_P) / SCK at every resolution.

Two separate defects made that possible:

  * the HINT advised the geometry already in force. It hardcoded
    `--clearance 0.15 --track-width 0.15` regardless of the config, and the run
    was routing at exactly 0.15/0.15 -- so the only novel token in the whole
    sentence was the grid.
  * nothing GATED on the verdict. Warn-only was tried by circumstance: the
    warning was on screen every time and the laps were spent anyway.

A finer grid resolves the SAME obstacles more precisely. It does not make a gap
wider. When the geometry is at the board's declared floor there is nothing left
to pair it with, and the parameter family is spent.

    python3 tests/test_run20_boxin_lever_guard.py
"""
import hashlib
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)
DRIVER = os.path.join(REPO, '.claude', 'skills',
                      'plan-pcb-placement-and-routing', 'scripts',
                      'loop_driver.py')
CONVERGE = os.path.join(REPO, 'py_placer', 'converge.py')

import routing_diagnostics as RD                                   # noqa: E402

passed = failed = 0


def check(label, cond, detail=''):
    global passed, failed
    if cond:
        passed += 1
        print(f'  OK   {label}')
    else:
        failed += 1
        print(f'  FAIL {label} -- {detail}')


_D = tempfile.mkdtemp()

print('--- the hint stops advising the geometry already in use ---')

_HB = os.path.join(_D, 'hint.kicad_pcb')
with open(_HB, 'w', encoding='utf-8') as fh:
    fh.write('(kicad_pcb (version 20260206) (generator t)\n'
             ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal))\n (net 0 ""))')
# BOTH floors declared. The first version of this fixture declared only a
# track floor -- and then asserted `at_floor is True`, which is exactly the
# defect below: one readable floor is not enough to say the geometry is spent.
# The fixture was relying on the bug, so it had to change with the fix.
with open(os.path.join(_D, 'hint.kicad_pro'), 'w', encoding='utf-8') as fh:
    json.dump({'board': {'design_settings': {
        'rules': {'min_track_width': 0.15},
        'defaults': {'track_width': 0.15}}},
        'net_settings': {'classes': [
            {'name': 'Default', 'clearance': 0.15, 'track_width': 0.15}]}}, fh)


class _Cfg:
    grid_step, clearance, track_width, via_diameter = 0.05, 0.15, 0.15, 0.6


class _Pcb:
    source_path = _HB

    class board_info:
        board_bounds = (0, 0, 40, 30)


_RESULT = {'iterations_forward': 400, 'iterations_backward': 400}
hint, verdict = RD.static_boxin_hint(_RESULT, _Cfg(), _Pcb(),
                                     return_verdict=True)
check('at the declared floor the hint says the parameter family is spent',
      'ALREADY at this board' in hint and 'PLACEMENT/FLOORPLAN FINDING' in hint,
      hint)
check('and it does NOT advise the values already in force',
      '--track-width 0.15' not in hint and '--clearance 0.15' not in hint,
      hint + ' -- run 20\'s hint advised exactly what was running, so the only '
             'novel token in it was the grid')
check('the verdict says so as DATA, not only as prose',
      verdict.get('at_floor') is True, str(verdict))
check('and carries the floors it compared against',
      (verdict.get('floors') or {}).get('track_width') == 0.15,
      str(verdict.get('floors')))


class _Cfg2(_Cfg):
    clearance, track_width = 0.25, 0.3


hint2, verdict2 = RD.static_boxin_hint(_RESULT, _Cfg2(), _Pcb(),
                                       return_verdict=True)
check('with travel left it advises the geometry, paired with the grid',
      '--track-width 0.15' in hint2 and '--grid-step' in hint2, hint2)
check('and says the grid is the COMPLEMENT, never the lever alone',
      'COMPLEMENT' in hint2, hint2)
check('the verdict agrees', verdict2.get('at_floor') is False, str(verdict2))


class _NoSrc:
    source_path = None

    class board_info:
        board_bounds = (0, 0, 40, 30)


hint3, verdict3 = RD.static_boxin_hint(_RESULT, _Cfg(), _NoSrc(),
                                       return_verdict=True)
check('with no readable floor it says UNKNOWN rather than guessing',
      verdict3.get('at_floor') is None and 'unknown' in hint3.lower(), hint3)

print('--- at_floor needs BOTH floors, not either ---')
# `or` here asserted "at the floor" from ONE readable floor. A board declaring
# min_track_width but no netclass has NO clearance floor (clearance has no
# board-constraint fallback in list_nets._FLOOR_SOURCES), so a run at
# clearance 0.90 against a 0.15 track floor reported at_floor TRUE -- and L4
# REFUSED the parameter lever, when dropping `--clearance 0.9` was the working
# move. An axis whose floor cannot be read may have travel.
_HB2 = os.path.join(_D, 'trackonly.kicad_pcb')
with open(_HB2, 'w', encoding='utf-8') as fh:
    fh.write('(kicad_pcb (version 20260206) (generator t) (net 0 ""))')
with open(os.path.join(_D, 'trackonly.kicad_pro'), 'w', encoding='utf-8') as fh:
    json.dump({'board': {'design_settings': {
        'rules': {'min_track_width': 0.15}}}}, fh)


class _P2:
    source_path = _HB2

    class board_info:
        board_bounds = (0, 0, 40, 30)


class _CWide(_Cfg):
    clearance, track_width = 0.90, 0.15


_h, _v = RD.static_boxin_hint(_RESULT, _CWide(), _P2(), return_verdict=True)
check('one readable floor is NOT enough to claim at_floor',
      _v.get('at_floor') is None,
      f"{_v.get('at_floor')} floors={_v.get('floors')} -- clearance 0.90 with "
      f"an unreadable clearance floor; asserting at-floor here refuses the one "
      f"lever that works")
check('and the hint does not tell the operator the family is spent',
      'PLACEMENT/FLOORPLAN FINDING' not in _h, _h[:300])

# ...and True is still REACHABLE, or the guard would never fire again.
with open(os.path.join(_D, 'both.kicad_pro'), 'w', encoding='utf-8') as fh:
    json.dump({'board': {'design_settings': {'rules': {'min_track_width': 0.15}}},
               'net_settings': {'classes': [
                   {'name': 'Default', 'clearance': 0.15,
                    'track_width': 0.15}]}}, fh)
_HB3 = os.path.join(_D, 'both.kicad_pcb')
with open(_HB3, 'w', encoding='utf-8') as fh:
    fh.write('(kicad_pcb (version 20260206) (generator t) (net 0 ""))')


class _P3:
    source_path = _HB3

    class board_info:
        board_bounds = (0, 0, 40, 30)


_h, _v = RD.static_boxin_hint(_RESULT, _Cfg(), _P3(), return_verdict=True)
check('with BOTH floors read and neither having travel, at_floor is True',
      _v.get('at_floor') is True, str(_v.get('floors')))


class _CTravel(_Cfg):
    clearance, track_width = 0.30, 0.15


_h, _v = RD.static_boxin_hint(_RESULT, _CTravel(), _P3(), return_verdict=True)
check('and travel on EITHER axis makes it False',
      _v.get('at_floor') is False, str(_v.get('floors')))

print('--- and L4 refuses the lever, rather than warning about it ---')

_B = os.path.join(_D, 'r.kicad_pcb')
with open(_B, 'w', encoding='utf-8') as fh:
    fh.write('(kicad_pcb (version 20260206) (generator t)\n'
             ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
             ' (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts") (width 0.1))\n'
             ' (net 0 ""))')
_SHA = hashlib.sha256(open(_B, 'rb').read()).hexdigest()
_LED = os.path.join(_D, 'l.jsonl')
open(_LED, 'w').close()
subprocess.run([sys.executable, CONVERGE, 'record', '--ledger', _LED,
                '--board', _B, '--kind', 'completion', '--lever', 'the route'],
               capture_output=True, text=True, timeout=300)


def _w(name, doc):
    p = os.path.join(_D, name)
    with open(p, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return p


_SC = _w('s.json', {'kind': 'board-score', 'blocking': 10, 'board_sha': _SHA,
                    'blocking_by': {'unrouted': 3}, 'quality': {},
                    'components': {}, 'ungraded': [], 'unknown': []})
_CG = _w('cg.json', {'metrics': {'crossings': 100, 'hpwl': 500.0},
                     'instrument': {'board': os.path.abspath(_B)}})
_CGB = _w('cgb.json', {'metrics': {'crossings': 400, 'hpwl': 900.0},
                       'instrument': {'board': os.path.abspath(_B)}})


def _boxsum(name, at_floor, geom):
    return _w(name, {'boxed_in': [
        {'net': 'BUSY', 'verdict': 'boxed_in_static', 'iterations': 812,
         'geometry': geom, 'floors': {'clearance': 0.15, 'track_width': 0.15},
         'at_floor': at_floor}]})


_ATF = _boxsum('atf.json', True,
               {'grid_step': 0.05, 'clearance': 0.15, 'track_width': 0.15})
_TRV = _boxsum('trv.json', False,
               {'grid_step': 0.05, 'clearance': 0.25, 'track_width': 0.3})
_UNK = _boxsum('unk.json', None,
               {'grid_step': 0.05, 'clearance': 0.15, 'track_width': 0.15})
_NONE = _w('none.json', {'blockers': []})


def _l4(*extra):
    p = subprocess.run(
        [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'L4',
         '--shape', 'parameter', '--board', _B, '--ledger', _LED,
         '--score', _SC, '--no-delegate',
         '--congestion-json', _CG, '--congestion-baseline', _CGB] + list(extra),
        capture_output=True, text=True, timeout=600)
    return (p.stdout or '') + (p.stderr or '')


out = _l4('--route-summary', _ATF)
check('a static-boxin verdict at the floor REFUSES the parameter lever',
      '<error>' in out and 'boxed in by STATIC obstacles' in out
      and 'ALREADY at this board' in out, out[:400])
check('the refusal names the nets and the geometry that was running',
      'BUSY' in out and 'grid 0.05' in out, out[:500])
check('it quotes the measurement rather than asserting the rule',
      '0.05 -> 0.025 -> 0.0125' in out and '40 min' in out, out[:900])
check('and it hands over the placement measurement to take instead',
      'check_reachability' in out and 'COPPER-FREE' in out
      and '--defect-json' in out, out[:1200])

out = _l4('--route-summary', _TRV)
check('with travel left it REPORTS and does not refuse',
      '<error>' not in out and 'boxed in by STATIC' in out, out[:400])
check('...and the note reaches the stage body, not just the console',
      'complement, never the lever alone' in out, out[:800])

out = _l4('--route-summary', _UNK)
check('an UNKNOWN floor does not refuse -- "I could not tell" is not "you '
      'failed"',
      '<error>' not in out and 'UNKNOWN' in out, out[:500])

out = _l4('--route-summary', _NONE)
check('no box-in verdict at all is silent', '<error>' not in out
      and 'boxed in by STATIC' not in out, out[:300])

out = _l4()
check('and with no --route-summary the guard is a no-op',
      '<error>' not in out and 'boxed in by STATIC' not in out, out[:300])

_WHY = '0.4mm pitch pads; a via-in-pad fanout is planned and changes the field'
out = _l4('--route-summary', _ATF, '--accept-boxin', _WHY)
check('--accept-boxin proceeds',
      '<error>' not in out, out[:400])
check('and the REASON is echoed into the body, so it reaches the ledger',
      _WHY in out, out[:600])

out = _l4('--route-summary', _ATF, '--accept-boxin', '   ')
check('a blank reason does NOT waive it',
      '<error>' in out and 'ALREADY at this board' in out, out[:300])

_BAD = os.path.join(_D, 'missing.json')
out = _l4('--route-summary', _BAD)
check('a summary that cannot be read is a NOTE, never a refusal',
      '<error>' not in out and 'no such file' in out and 'not checked' in out,
      out[:400] + ' -- the loader now says WHICH way it failed (missing / not '
      'JSON / no JSON_SUMMARY line) instead of one word for all three')

print('--- --route-summary must accept the artifact L2 tells you to make ---')
# L2's contract hands the operator `route.log -- the one carrying JSON_SUMMARY`.
# `--route-summary` used to json.load it, fail, and SKIP the box-in
# precondition with a NOTE -- so the guard that exists to stop 40 minutes of
# futile grid laps was off for anyone who passed the artifact the loop asked
# for, and it failed OPEN.
_LOG = os.path.join(_D, 'route.log')
with open(_LOG, 'w', encoding='utf-8') as fh:
    fh.writelines([
        'Loading board...',
        os.linesep,
        'JSON_SUMMARY: ' + json.dumps({
            'failed_single': ['BUSY'], 'routed_single': [],
            'boxed_in': [{'net': 'BUSY', 'verdict': 'boxed_in_static',
                          'iterations': 812,
                          'geometry': {'grid_step': 0.05, 'clearance': 0.15,
                                       'track_width': 0.15},
                          'floors': {'clearance': 0.15,
                                     'track_width': 0.15},
                          'at_floor': True}]}),
        os.linesep,
        'EXIT=1',
        os.linesep,
    ])
out = _l4('--route-summary', _LOG)
check('a route LOG drives the guard, not just a bare JSON summary',
      '<error>' in out and 'boxed in by STATIC obstacles' in out,
      out[:400] + ' -- it used to skip the check with a NOTE and open the stage')

_NEITHER = os.path.join(_D, 'notes.txt')
with open(_NEITHER, 'w', encoding='utf-8') as fh:
    fh.write('just some prose')
out = _l4('--route-summary', _NEITHER)
check('a file that is neither says WHICH it is not, and does not refuse',
      '<error>' not in out and 'neither a summary nor a route log' in out,
      out[:400])

print('--- boxed_in survives the reconcile merge ---')
# `merge_summaries` rebuilt `blockers` from the first pass when the last one
# lacked it and did nothing for `boxed_in` -- so on any run that fires the
# reconciliation sub-pass (the common case) the key vanished and the guard went
# silent on exactly the run it exists for.
from route_summary import merge_summaries                          # noqa: E402
_first = {'failed_single': ['BUSY', 'SCK'], 'routed_single': [],
          'blockers': [{'net': 'SCK', 'blocked_by': []}],
          'boxed_in': [{'net': 'BUSY', 'at_floor': True},
                       {'net': 'FIXED', 'at_floor': True}]}
_last = {'failed_single': ['BUSY', 'SCK'], 'routed_single': []}
_m = merge_summaries([_first, _last]) or {}
check('a merged summary keeps boxed_in, as it already kept blockers',
      [e['net'] for e in (_m.get('boxed_in') or ())] == ['BUSY'],
      f"{_m.get('boxed_in')} -- and FIXED is dropped because it is no longer "
      f"failing, so a reconciled net carries no stale accusation")
check('...and blockers still behaves exactly as before',
      [e['net'] for e in (_m.get('blockers') or ())] == ['SCK'],
      str(_m.get('blockers')))

print('--- the skill says the same thing, in the row that misdirected ---')
_SK = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')
_t = open(_SK, encoding='utf-8').read()
check('the box-in row is SPLIT on whether the geometry has travel',
      'geometry has travel' in _t and 'geometry at the declared floor' in _t,
      'one row that says "try a finer grid" for both cases is the misdirection')
check('and the at-floor half sends the reader to placement',
      'the parameter family is spent' in _t, '')
check('`ripup budget` is gone from the box-in row',
      'ripup budget' not in _t.split('boxed_in` names the net')[-1][:1200],
      'run 20 measured the rip lever at 10 -> 15 on exactly this verdict, and '
      '"no rippable blockers found" means the blockers are not rippable copper')
check('the measurement that produced the rule is quoted with it',
      '0.05 -> 0.025 -> 0.0125' in _t, 'a rule without its measurement becomes '
      'folklore and gets deleted by the next reader')

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
