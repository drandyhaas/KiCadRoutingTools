#!/usr/bin/env python3
"""`ungraded` conflated "not applicable" with "not measured".

L5 refused to close run 20 because `length` and `net_widths` were "never
examined", and closing needed `--accept-unclosed ungraded`. But the two are
different facts. That board has no length-matched group AT ALL, so there is
nothing to grade and passing `--length-groups` would INVENT a requirement --
whereas `net_widths` was genuinely measurable and simply unasked-for.

One waiver covered both. A standing false waiver on every simple board is how a
real one stops being read.

`board_score.skipped(reason)` already carried a reason string; nothing
downstream read it. It now carries the distinction as data, and only `ungraded`
blocks a close-out. Both lists stay in the payload and on the printed line --
"not applicable" must be VISIBLE, not silently dropped, or a reader cannot tell
it from a gap.

The applicability test is the chain's OWN record (#521): matched groups and
routed diff pairs live under `protected_nets`, impedance specs under
`net_impedance`, in the sibling project. Conservative in the right direction --
anything unreadable answers "applicable", so a component stays `ungraded`
(blocking) rather than being silently excused.

    python3 tests/test_run20_not_applicable.py
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BS = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts',
                  'board_score.py')
CC = os.path.join(REPO, 'check_complete.py')

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
_BOARD_TXT = '''(kicad_pcb (version 20260206) (generator t)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))
 (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts") (width 0.1))
 (net 0 "") (net 1 "N1")
 (footprint "t:r" (layer "F.Cu") (at 10 15)
  (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1")))
 (footprint "t:r" (layer "F.Cu") (at 30 15)
  (property "Reference" "R2" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1")))
 (segment (start 10 15) (end 30 15) (width 0.2) (layer "F.Cu") (net 1))
)'''


def _stage(name, krt=None):
    d = os.path.join(_D, name)
    os.makedirs(d, exist_ok=True)
    b = os.path.join(d, 'b.kicad_pcb')
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(_BOARD_TXT)
    pro = {'board': {'design_settings': {'rules': {'min_track_width': 0.15}}}}
    if krt:
        pro['kicad_routing_tools'] = krt
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(pro, fh)
    return b


def _score(board):
    p = subprocess.run([sys.executable, BS, board, '--quiet'],
                       capture_output=True, text=True, timeout=900)
    out = (p.stdout or '')
    ln = [x for x in out.splitlines() if x.startswith('SCORE_JSON=')]
    return (json.loads(ln[0][len('SCORE_JSON='):]) if ln else {}), out


print('--- a board with no such requirement ---')
_plain = _stage('plain')
sc, out = _score(_plain)
check('length is NOT APPLICABLE, not ungraded',
      'length' in (sc.get('not_applicable') or [])
      and 'length' not in (sc.get('ungraded') or []),
      f"ungraded={sc.get('ungraded')} n/a={sc.get('not_applicable')}")
check('and so is impedance',
      'impedance' in (sc.get('not_applicable') or []), str(sc.get('not_applicable')))
check('but net_widths stays UNGRADED -- it was measurable and unasked',
      'net_widths' in (sc.get('ungraded') or []),
      f"{sc.get('ungraded')} -- there is no board-side declaration for per-net "
      f"widths, so 'nobody asked' is the whole truth about it")
check('the distinction is PRINTED, not only in the JSON',
      'NOT APPLICABLE' in out and 'UNGRADED' in out, out[-500:])
check('and it reaches the identity line, where a comparison is made',
      any(x.startswith('BLOCKING=') and 'n/a:' in x
          for x in out.splitlines()),
      [x for x in out.splitlines() if x.startswith('BLOCKING=')])

print('--- a board that DOES declare one ---')
_decl = _stage('declared', krt={'protected_nets': ['N1'],
                                'net_impedance': {'N1': {'ohms': 50}}})
sc2, out2 = _score(_decl)
check('a declared match group makes length UNGRADED again',
      'length' in (sc2.get('ungraded') or [])
      and 'length' not in (sc2.get('not_applicable') or []),
      f"ungraded={sc2.get('ungraded')} n/a={sc2.get('not_applicable')}")
check('and a declared impedance net makes impedance UNGRADED again',
      'impedance' in (sc2.get('ungraded') or []), str(sc2.get('ungraded')))

print('--- the fallback is conservative in the right direction ---')
_noproj = os.path.join(_D, 'noproj.kicad_pcb')
with open(_noproj, 'w', encoding='utf-8') as fh:
    fh.write(_BOARD_TXT)
sc3, _ = _score(_noproj)
check('a project-less board does not silently excuse anything it cannot read',
      'net_widths' in (sc3.get('ungraded') or []), str(sc3.get('ungraded')))

_bad = _stage('badproj')
with open(os.path.join(os.path.dirname(_bad), 'b.kicad_pro'), 'w',
          encoding='utf-8') as fh:
    fh.write('{ this is not json')
sc4, _ = _score(_bad)
check('an UNREADABLE project leaves both blocking, never excused',
      'length' in (sc4.get('ungraded') or [])
      and 'impedance' in (sc4.get('ungraded') or []),
      f"{sc4.get('ungraded')} / {sc4.get('not_applicable')} -- unreadable must "
      f"mean 'still applicable', or a corrupt file waives a real requirement")

print('--- and the close-out stops demanding a waiver for a non-question ---')
p = subprocess.run([sys.executable, '-X', 'utf8', CC, _plain, '--skip-slow'],
                   capture_output=True, text=True, timeout=900)
o = (p.stdout or '') + (p.stderr or '')
_v = [x for x in o.splitlines() if x.startswith('VERDICT:')]
check('check_complete prints NOT APPLICABLE beside UNEXAMINED',
      _v and 'NOT APPLICABLE' in _v[0], (_v[0] if _v else o[-400:]))
check('and does not list length among the unexamined',
      _v and ('UNEXAMINED' not in _v[0]
              or 'length' not in _v[0].split('UNEXAMINED')[1].split('.')[0]),
      (_v[0] if _v else o[-400:]) + ' -- run 20 needed --accept-unclosed '
      'ungraded to close, covering a real gap and a non-question at once')

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
