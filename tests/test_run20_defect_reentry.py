#!/usr/bin/env python3
"""The loop stops asking for work it is already holding.

L3 printed three shapes and four measurement commands and asked the operator to
run check_channels, check_reachability and two renders, read the exit codes and
decide. Every one of those measurements is exactly what a defect record and a
route summary contain -- so the stage the whole loop turns on was asking for
work the loop had already done.

The rule this file pins hardest: L3 PROPOSES, it does not decide and it does not
refuse. `--shape` on L4 stays the operator's assertion -- the driver's own text
calls that "the guard that matters most" -- and it must keep costing a
deliberate act. What changed is that the assertion is now made against a printed
derivation instead of from memory, and a contradiction is printed loudly at L4.

    python3 tests/test_run20_defect_reentry.py
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVER = os.path.join(REPO, '.claude', 'skills',
                      'plan-pcb-placement-and-routing', 'scripts',
                      'loop_driver.py')
CONVERGE = os.path.join(REPO, 'py_placer', 'converge.py')

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
_BOARD = os.path.join(_D, 'routed.kicad_pcb')
with open(_BOARD, 'w', encoding='utf-8') as fh:
    fh.write('(kicad_pcb (version 20260206) (generator test)\n'
             ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
             ' (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts") (width 0.1))\n'
             ' (net 0 "")\n)')
_LEDGER = os.path.join(_D, 'ledger.jsonl')
open(_LEDGER, 'w').close()
# L3 refuses a board the ledger does not know, and it is right to: everything
# downstream reads the ledger rather than the board. Record it, exactly as an
# operator would, so the fixture exercises the derivation rather than the gate.
subprocess.run([sys.executable, CONVERGE, 'record', '--ledger', _LEDGER,
                '--board', _BOARD, '--kind', 'completion',
                '--lever', 'the route that failed'],
               capture_output=True, text=True, timeout=300)


def _w(name, doc):
    p = os.path.join(_D, name)
    with open(p, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return p


# The run-20 measurement.
_DEFECT = _w('defect.json', {
    'kind': 'defect-record', 'version': 1, 'count': 1,
    'defects': [{
        'kind': 'throat', 'verdict': 'CAGED', 'net': 'Net-(U4-XTAL_P)',
        'at': {'x': 87.4825, 'y': 91.235, 'layer': 'F.Cu'},
        'refs': ['R7', 'U4'], 'pads': ['R7.2', 'U4.53'],
        'measure': {'space': 'track_width', 'have_mm': 0.11231,
                    'need_mm': 0.15, 'short_mm': 0.03769,
                    'resolution_mm': 0.01, 'gap_mm': 0.41231,
                    'gap_need_mm': 0.45},
        'relief': [{'dir': 'east', 'min_mm': 0.0422, 'bound': 'lower',
                    'ref': 'R7', 'against': 'U4'}]}]})
_PASSABLE = _w('passable.json', {
    'kind': 'defect-record', 'version': 1, 'count': 0, 'defects': []})
_BLOCKERS = _w('blockers.json', {
    'blockers': [{'net': 'SCK', 'stage': 'single_ended',
                  'blocked_by': [{'net': 'GND'}]}]})
_BOXED = _w('boxed.json', {
    'boxed_in': [{'net': 'BUSY', 'verdict': 'boxed_in_static',
                  'iterations': 812,
                  'geometry': {'grid_step': 0.05, 'clearance': 0.15,
                               'track_width': 0.15}}]})
_BOTH = _w('both.json', {
    'blockers': [{'net': 'SCK', 'stage': 'single_ended',
                  'blocked_by': [{'net': 'GND'}]}],
    'boxed_in': []})
import hashlib                                                     # noqa: E402
# The REAL sha. L3 board-binds its score, correctly -- classifying board A from
# board B's score is exactly the confusion the binding exists to stop -- so a
# placeholder here refuses every L3 case and 11 assertions "fail" on the fixture
# rather than on the code.
_SHA = hashlib.sha256(open(_BOARD, 'rb').read()).hexdigest()
_SCORE = _w('score.json', {
    'kind': 'board-score', 'blocking': 10, 'board_sha': _SHA,
    'blocking_by': {'unrouted': 3, 'broken': 7}, 'quality': {},
    'components': {}, 'ungraded': [], 'unknown': []})
_RENDER = _w('render.json', {
    'instrument': {'board': os.path.abspath(_BOARD), 'summary_json': 'x.log'},
    'checklist': {'a_off_outline': {'pad_copper': []}}, 'moved_refs': []})


def _run(*extra):
    p = subprocess.run(
        [sys.executable, '-X', 'utf8', DRIVER, '--board', _BOARD,
         '--ledger', _LEDGER, '--no-delegate'] + list(extra),
        capture_output=True, text=True, timeout=600)
    return (p.stdout or '') + (p.stderr or '')


_L3 = ['--stage', 'L3', '--score', _SCORE, '--render-json', _RENDER]

print('--- L3 derives a shape from what it already holds ---')

out = _run(*_L3, '--defect-json', _DEFECT)
check('a CAGED record proposes placement',
      'proposed shape: placement' in out,
      out[out.find('DERIVED'):][:400])
check('and it names the net and the margin as its evidence',
      'Net-(U4-XTAL_P)' in out and '-37.69 um' in out,
      out[out.find('DERIVED'):][:400])
check('with the copper-free precondition stated, not assumed',
      'COPPER-FREE' in out,
      'a CAGED verdict measured on ROUTED copper is rip-up depth, not '
      'placement -- three genuine cages once became three PASSABLE once the '
      'copper was removed')

out = _run(*_L3, '--route-summary', _BLOCKERS)
check('a summary with blockers proposes parameter',
      'proposed shape: parameter' in out and 'placement' not in
      out[out.find('DERIVED'):out.find('This is a PROPOSAL')],
      out[out.find('DERIVED'):][:400])

out = _run(*_L3, '--route-summary', _BOXED)
check('a box-in verdict proposes parameter OR placement, decided by the floor',
      'parameter OR placement' in out and 'at the board floor' in out,
      out[out.find('DERIVED'):][:500])
check('and it prints the geometry that was in force',
      'grid 0.05' in out and 'clearance 0.15' in out,
      out[out.find('DERIVED'):][:500])

out = _run(*_L3, '--defect-json', _DEFECT, '--route-summary', _BLOCKERS)
check('records that DISAGREE propose nothing and print both',
      'proposed shape: NONE' in out and 'DISAGREE' in out
      and 'Net-(U4-XTAL_P)' in out and 'blockers' in out,
      out[out.find('DERIVED'):][:500])
check('...and say not to average them',
      'do not average' in out, out[out.find('DERIVED'):][:500])

out = _run(*_L3, '--defect-json', _PASSABLE)
check('a record with no CAGED defect derives nothing, and says so',
      'nothing to derive from' in out, out[out.find('DERIVED'):][:300])

out = _run(*_L3, '--defect-json', os.path.join(_D, 'missing.json'))
check('an unreadable record is a NOTE, never a refusal',
      'NOTE' in out and '<error>' not in out, out[-400:])

print('--- and with neither flag, L3 is byte-for-byte what it was ---')
plain = _run(*_L3)
check('no derivation block appears at all', 'DERIVED FROM' not in plain,
      plain[-300:])
check('and the stage still opens', plain.startswith('<stage_instructions'),
      plain[:200])

print('--- it PROPOSES; L4 still demands the assertion ---')
out = _run('--stage', 'L4', '--score', _SCORE, '--defect-json', _DEFECT)
check('L4 without --shape still refuses, derivation or not',
      '<error>' in out and 'needs a measured shape' in out, out[:300])

out = _run('--stage', 'L4', '--shape', 'parameter', '--score', _SCORE,
           '--defect-json', _DEFECT)
check('a --shape that CONTRADICTS the derivation is called out',
      'you asserted --shape parameter' in out and 'derives placement' in out,
      out[:500])
check('...and it proceeds anyway -- the assertion is the operator\'s',
      'Proceeding with parameter' in out, out[:500])

out = _run('--stage', 'L4', '--shape', 'placement', '--score', _SCORE,
           '--defect-json', _DEFECT)
check('a --shape that AGREES draws no complaint',
      'you asserted' not in out, out[:400])

print('--- L4-placement carries the target, in both spaces ---')
check('the brief names the throat and both spaces',
      'gap space' in out and 'track space' in out
      and '0.4123' in out and '0.11231' in out, out[:900])
check('and it states the TARGET as a move with a direction and a bound',
      'TARGET: move R7 >= 0.0422 mm east' in out and 'lower bound' in out,
      out[:900])
check('with the two-body caveat, so it is not read as a solution',
      'may create another' in out, out[:900])

out = _run('--stage', 'L4', '--shape', 'placement', '--score', _SCORE)
check('without a record L4-placement still works, and never refuses',
      '<error>' not in out and 'WHAT THIS RE-ENTRY IS AIMED AT' not in out,
      out[:300])

print('--- the ledger keeps the measurement and the shape ---')
p = subprocess.run(
    [sys.executable, CONVERGE, 'record', '--ledger', _LEDGER,
     '--board', _BOARD, '--kind', 'placement', '--shape', 'placement',
     '--defect-json', _DEFECT, '--lever', 'reseat R7 east'],
    capture_output=True, text=True, timeout=300)
check('converge record accepts --defect-json and --shape', p.returncode == 0,
      (p.stdout or '') + (p.stderr or ''))
rows = [json.loads(x) for x in open(_LEDGER, encoding='utf-8') if x.strip()]
_last = rows[-1] if rows else {}
check("the lap records entry['defects']",
      _last.get('defects') == [_DEFECT],
      f'{_last.get("defects")} -- a ledger that keeps the MEASUREMENT can tell '
      f'a later reader what the lap was for; one that keeps a paragraph about '
      f'it cannot')
check("and entry['shape'], which L4 demanded and the ledger never kept",
      _last.get('shape') == 'placement', str(_last.get('shape')))

print('--- the incommensurable trade is expressible and documented ---')
_SK = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')
_txt = open(_SK, encoding='utf-8').read()
check('--accept-incommensurable is documented beside the accept rule',
      '--accept-incommensurable' in _txt, 'it existed and was used once in run '
      '20 and appeared in neither skill')
check('with the run-20 case as the worked example',
      'annular' in _txt and '10 -> 13' in _txt,
      'a rule without the measurement that produced it becomes folklore')
# The example states the RECORDED numbers and names WHY no grader credited
# the removal (check_drc grades via diameter and drill separately, so a
# ring-0 via passes both). It must not claim a re-grade the current graders
# cannot reproduce: annular-width grading is not on main.
check('and the numbers are presented as RECORDED, with the blind spot named',
      'as RECORDED' in _txt and 'ring-0 via' in _txt,
      'a worked example claiming a re-grade nobody can reproduce teaches the '
      'wrong lesson twice')
check('and the rule is "the comparison is VOID", not "pick the smaller number"',
      'comparison is\nVOID' in _txt or 'comparison is VOID' in _txt, '')

_bs = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts',
                   'board_score.py')
p = subprocess.run([sys.executable, _bs, _BOARD, '--quiet'],
                   capture_output=True, text=True, timeout=900)
_line = [ln for ln in (p.stdout or '').splitlines() if ln.startswith('BLOCKING=')]
check('the score prints the COMPONENT SET on its identity line',
      _line and 'over' in _line[0] and 'components' in _line[0],
      (_line[0] if _line else (p.stdout or '')[-300:])
      + ' -- run 20 produced blocking 12, 13 and 18 for one board under three '
        'flag sets and nothing on the line said so')

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
