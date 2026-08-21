#!/usr/bin/env python3
"""L2's off-board gate reads the coarse channel while holding the precise one.

`check_assembly`'s `oob_pad_count` is a part pad AABB -- pads PLUS NPTH drill
circles -- tested against an outline INFLATED by the grading clearance. Its own
`oob_pad_basis` string says so, and adds that the per-pad, margin-0 measure is
`render_placement`'s `checklist.a_off_outline.pad_copper`.

L2 refused on the coarse number while already being handed the render that
carries the precise one. On run 20 they disagreed -- coarse [R4, SW2], per-pad
[SW2] -- which is why `--accept-residue oob_pad_count` had to be used in BOTH
cycles, and a waiver raised for one part also covers the other.

R4 IS NOT A PHANTOM, and an earlier version of this file said it was. Its copper
is on the outline, so `pad_copper` is right to omit it -- but
`check_drc --check-pad-edge` reports R4.1 and R4.2 in CONTACT with the board
edge. The two channels measure different requirements: copper OFF the outline
at margin 0, versus the pad AABB against an outline inflated by the grading
clearance, which is board-edge clearance. Reading their difference as a false
positive discards a real violation, so the gate names it as an edge finding and
prints the command that lists it.
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


def _w(name, doc):
    p = os.path.join(_D, name)
    with open(p, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return p


# A board file the driver can stat. L2 board-binds its documents, so the paths
# have to agree; content does not matter to this gate.
_BOARD = os.path.join(_D, 'placed.kicad_pcb')
with open(_BOARD, 'w', encoding='utf-8') as fh:
    fh.write('(kicad_pcb (version 20260206) (generator test)\n'
             ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
             ' (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts") (width 0.1))\n'
             ' (net 0 "")\n)')

# The run-20 disagreement, reduced: two parts by the coarse measure, one by the
# precise one.
_ASSEMBLY = _w('assembly.json', {
    'board': os.path.abspath(_BOARD), 'blocking': 0, 'buildable': True,
    'verdict': 'buildable', 'locked_contacts': 0, 'oob_pad_count': 2,
    'oob_pad_basis': 'part pad AABB against an outline inflated by the '
                     'GRADING CLEARANCE'})
_RENDER = _w('render.json', {
    'instrument': {'board': os.path.abspath(_BOARD), 'summary_json': 'x.log'},
    'checklist': {'a_off_outline': {
        'courtyard': [['J2', 5.1492], ['R4', 0.6897], ['SW2', 2.3292]],
        'pad_copper': [['SW2', 1.5672]]}},
    'moved_refs': []})
_RENDER_CLEAN = _w('render_clean.json', {
    'instrument': {'board': os.path.abspath(_BOARD), 'summary_json': 'x.log'},
    'checklist': {'a_off_outline': {'courtyard': [['R4', 0.6897]],
                                    'pad_copper': []}},
    'moved_refs': []})
_LEDGER = os.path.join(_D, 'ledger.jsonl')
open(_LEDGER, 'w').close()


def _l2(*extra):
    p = subprocess.run(
        [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'L2',
         '--board', _BOARD, '--ledger', _LEDGER,
         '--placement-report', _ASSEMBLY, '--no-delegate'] + list(extra),
        capture_output=True, text=True, timeout=300)
    return (p.stdout or '') + (p.stderr or '')


print('--- with the render, the gate names the part whose copper is off ---')
out = _l2('--render-json', _RENDER)
check('it still refuses -- there IS a real off-board part',
      '<error>' in out and 'pad copper OFF the board' in out, out[:400])
check('and it names SW2',
      'SW2' in out, out[:600])
check('but NOT R4, whose copper was on the board the whole time',
      'R4' not in out.split('Named:')[-1].split('.')[0] if 'Named:' in out
      else 'R4' not in out,
      'the coarse channel inflates the outline by the grading clearance, so a '
      'part near the edge reads as off it; the waiver that answer forces also '
      'covers the real finding')
check('the refusal says WHICH measure it used',
      'a_off_outline.pad_copper' in out and 'per-pad' in out, out[:600])
check('and discloses the coarse number as corroboration, named as coarse',
      'coarse' in out and '2' in out, out[:800])
# The coarse channel is NOT a phantom, and the message must not say it is. On
# run 20 the difference was R4, whose copper IS on the outline -- and which
# `check_drc --check-pad-edge` reports in CONTACT with the board edge. The two
# channels measure different requirements; calling the difference a false
# positive discards a real violation.
check('the difference is named as a BOARD-EDGE finding, not dismissed',
      'BOARD-EDGE CLEARANCE' in out and '--check-pad-edge' in out,
      out[:900] + ' -- the coarse measure inflates the outline by the grading '
      'clearance, so it also catches copper that is on the board but too close '
      'to its edge')

print('--- when the precise channel says zero, the gate does not fire ---')
out = _l2('--render-json', _RENDER_CLEAN)
check('a board whose pad copper is all on the outline passes this gate',
      'pad copper OFF the board' not in out,
      out[:500] + ' -- the coarse count is 2 here and must not be believed '
      'over the margin-0 per-pad measure')

print('--- with no render, behaviour is exactly what it was ---')
out = _l2()
check('the coarse count still refuses when no render is available',
      '<error>' in out and 'pad copper OFF the board' in out, out[:400])
check('and the basis line says it is the coarse one',
      'coarse' in out and 'check_assembly' in out, out[:600])

print('--- a render it cannot use does not silently claim precision ---')
_BAD = os.path.join(_D, 'nope.json')
out = _l2('--render-json', _BAD)
check('a missing render falls back and SAYS the fallback happened',
      'coarse' in out and 'no such file' in out, out[:600])
_NOCHK = _w('nochk.json', {'instrument': {'board': os.path.abspath(_BOARD)}})
out = _l2('--render-json', _NOCHK)
check('a render without the checklist key does too',
      'coarse' in out and 'a_off_outline.pad_copper' in out, out[:600])

print('--- the render must be OF THIS BOARD, or it does not count ---')
# THE FAIL-OPEN DIRECTION. This channel REPLACES the coarse count rather than
# max()ing it, so every way of getting the wrong document turns a refusal into
# a pass. L3 board-binds its render; L2 did not, and was the only stage reading
# a document it never checked the identity of.
_OTHER = os.path.join(_D, 'other.kicad_pcb')
with open(_OTHER, 'w', encoding='utf-8') as fh:
    fh.write('(kicad_pcb (version 20260206) (generator test) (net 0 ""))')
_FOREIGN = _w('foreign.json', {
    'instrument': {'board': os.path.abspath(_OTHER), 'summary_json': 'x.log'},
    'checklist': {'a_off_outline': {'pad_copper': []}}, 'moved_refs': []})
out = _l2('--render-json', _FOREIGN)
check('a render of a DIFFERENT board does not silence the gate',
      '<error>' in out and 'pad copper OFF the board' in out,
      out[:400] + ' -- an empty pad_copper from another board would otherwise '
      'read as "this board is clean"')
check('and the basis line says WHY it was not used',
      'DIFFERENT board' in out, out[:600])
check('...falling back to the coarse count, which is still a real measurement',
      'coarse' in out, out[:600])

_NOINST = _w('noinst.json', {
    'checklist': {'a_off_outline': {'pad_copper': []}}})
out = _l2('--render-json', _NOINST)
check('a render with no instrument.board is refused the same way',
      '<error>' in out and 'no instrument.board' in out, out[:600])

print('--- and a malformed render is a fallback, never a crash ---')
for _name, _doc in (('list_top.json', [1, 2, 3]),
                    ('list_checklist.json',
                     {'instrument': {'board': os.path.abspath(_BOARD)},
                      'checklist': ['not', 'a', 'dict']}),
                    ('str_off.json',
                     {'instrument': {'board': os.path.abspath(_BOARD)},
                      'checklist': {'a_off_outline': 'nope'}}),
                    ('inst_list.json',
                     {'instrument': ['x'], 'checklist': {}})):
    out = _l2('--render-json', _w(_name, _doc))
    check(f'{_name} does not crash the stage',
          'Traceback' not in out and 'coarse' in out, out[:300])

print('--- the waiver still works, and still covers only what it names ---')
out = _l2('--render-json', _RENDER, '--accept-residue', 'oob_pad_count')
check('--accept-residue oob_pad_count still waives this gate',
      'pad copper OFF the board' not in out, out[:400])

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
