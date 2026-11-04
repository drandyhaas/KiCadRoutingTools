#!/usr/bin/env python3
"""#1586: a declared `mating_face` outranks a WEAK body reading.

`placement.mating` reads a connector's mating face from where its drawn body
sits on its pads, so the pad layout can set it: KiCad 10's stock
RJ45_Amphenol_RJHSE5380 reads +y (0.69 mm, weak) where the jack opens at -y,
while its twin RJ45_Amphenol_RJHSE538X -- the same housing plus four LED
pads -- reads -y (1.72 mm, weak), rightly. A declared `mating_face` was used
only when the body abstained, so the 5380 declared `mating_face: "-y"` on
the north edge was still turned mouth-into-board by the seat, and graded
`facing north [courtyard+fab, weak] pass` with nothing said.

Pinned here, on boards built from those footprints' geometry (copied from
KiCad 10's Connector_RJ.pretty; KiCad itself is not needed):
  1. the reading itself: the 5380 reads +y weak, the 538X -y weak;
  2. the seat turns the 5380 by its DECLARED face and its note says the
     declaration outranks the body's weak reading; undeclared, the seat is
     unchanged;
  3. the grade reads the declared face too: the mouth-into-board pose fails
     at ERROR, the right pose passes, and both carry a WARN that the
     declared face and the body's reading disagree; undeclared, unchanged;
  4. a STRONG reading still outranks a declaration (a shell 7 mm in front of
     its pads), and the seat note and the grade say they disagree.

    python3 tests/test_1586_declared_mating_face_outranks_weak.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
import pose_score                                              # noqa: E402
from placement import edge_seat, floorplan                     # noqa: E402
from placement.body import board_bodies                        # noqa: E402
from placement.mating import mating_face_local                 # noqa: E402
from run_utils import evidence                                 # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def pad(n, x, y, kind='thru_hole', s=1.5, d=0.89):
    return (f'    (pad "{n}" {kind} circle (at {x} {y}) (size {s} {s}) (drill {d}) '
            f'(layers "*.Cu" "*.Mask"))\n')


# KiCad 10 Connector_RJ.pretty: RJ45_Amphenol_RJHSE5380 (no LEDs) and
# RJ45_Amphenol_RJHSE538X (the same housing and contacts + 4 LED pads)
RJ_BASE = (pad('', -2.79, -2.54, 'np_thru_hole', 3.25, 3.25)
           + pad('', 9.91, -2.54, 'np_thru_hole', 3.25, 3.25)
           + ''.join(pad(i + 1, round(1.016 * i, 3), 1.78 * (i % 2)) for i in range(8))
           + pad('SH', -4.57, 0.89, s=2.3, d=1.57) + pad('SH', 11.69, 0.89, s=2.3, d=1.57))
RJ_LEDS = ''.join(pad(9 + i, x, 6.6) for i, x in enumerate((-3.3, -1.01, 8.13, 10.42)))
RJ_DRAW = ('    (fp_rect (start -4.7 -8) (end 11.81 7.75) (stroke (width 0.1) (type default)) '
           '(layer "F.Fab"))\n'
           '    (fp_rect (start -6.22 -8.5) (end 13.34 8.25) (stroke (width 0.05) (type default)) '
           '(layer "F.CrtYd"))\n')
# a shell 7 mm in front of its pads: a STRONG +y face
SHELL = ('    (fp_rect (start -4.5 -1) (end 4.5 8) (stroke (width 0.1) (type default)) '
         '(layer "F.Fab"))\n'
         '    (fp_rect (start -5 -1.5) (end 5 8.5) (stroke (width 0.05) (type default)) '
         '(layer "F.CrtYd"))\n'
         '    (pad "1" smd rect (at -1 0) (size 0.6 1.2) (layers "F.Cu" "F.Mask"))\n'
         '    (pad "2" smd rect (at 1 0) (size 0.6 1.2) (layers "F.Cu" "F.Mask"))\n')


def board(path, item, body, rot=0):
    cuts = ''.join(f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
                   '(stroke (width 0.05) (type default)) (layer "Edge.Cuts"))\n'
                   for a, b in (((0, 0), (80, 0)), ((80, 0), (80, 60)),
                                ((80, 60), (0, 60)), ((0, 60), (0, 0))))
    chips = ''.join(
        f'  (footprint "Package_SO:SOIC-8" (layer "F.Cu") (at {x} 50)\n'
        f'    (property "Reference" "{r}" (at 0 0) (layer "F.SilkS") (hide yes))\n'
        '    (fp_rect (start -3 -3) (end 3 3) (stroke (width 0.05) (type default)) '
        '(layer "F.CrtYd"))\n'
        '    (pad "1" smd rect (at -2 0) (size 1 1) (layers "F.Cu" "F.Mask"))\n'
        '    (pad "2" smd rect (at 2 0) (size 1 1) (layers "F.Cu" "F.Mask"))\n  )\n'
        for r, x in (('U1', 20), ('U2', 60)))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t1586")\n  (net 0 "")\n' + cuts
                 + f'  (footprint "Connector_RJ:{item}" (layer "F.Cu") (at 40 30 {rot})\n'
                 '    (property "Reference" "J1" (at 3.56 -9.5) (layer "F.SilkS"))\n'
                 + body + '  )\n' + chips + ')\n')
    evidence(path, 'synthetic board')
    return path


def intent(**extra):
    return floorplan.intent_from_dict({
        'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
        'edge_connectors': [dict({'ref': 'J1', 'edge': 'north', 'class': 'edge_receptacle'},
                                 **extra)]}, source_path='t')


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def reading(path):
    pcb = quiet(parse_kicad_pcb, path)
    return mating_face_local(pcb.footprints['J1'], quiet(board_bodies, pcb, path)['J1'])


def seat(path, it):
    """`(face, turns, note)` of J1's seat."""
    pcb = quiet(parse_kicad_pcb, path)
    state = quiet(pose_score.make_state, pcb, path, clearance=0.25, board_edge_clearance=0.2)
    out, notes, unseated = quiet(edge_seat.seats, state, pcb, path, it, {'J1'})
    note = next((n for n in notes if n.startswith('edge connector J1')), '')
    if 'J1' not in out:
        return None, [], f'unseated: {unseated}'
    return out['J1'].face, sorted(out['J1'].lines), note


def grade(path, it):
    """`(facing evidence, [(severity, message) naming J1's face])`."""
    pcb = quiet(parse_kicad_pcb, path)
    res = quiet(floorplan.grade, it, pcb, path)
    row = next((e for e in res.edge_connector_evidence if e['ref'] == 'J1'), {})
    said = [(v.severity, v.message) for v in res.violations
            if v.ref == 'J1' and ('mating face' in v.message or 'mating_face' in v.message)
            and 'seated' not in v.message]          # J1 stands mid-board: its setback is not this test's
    return row.get('facing') or {}, said, floorplan.format_text(res)


def main():
    tmp = tempfile.mkdtemp(prefix='t1586_')
    try:
        p5380 = board(os.path.join(tmp, '5380.kicad_pcb'), 'RJ45_Amphenol_RJHSE5380', RJ_DRAW + RJ_BASE)
        p538x = board(os.path.join(tmp, '538X.kicad_pcb'), 'RJ45_Amphenol_RJHSE538X',
                      RJ_DRAW + RJ_BASE + RJ_LEDS)
        lf, lx = reading(p5380), reading(p538x)
        check('the 5380 reads +y, weak (the pad layout, not the housing)',
              lf.face == '+y' and not lf.strong, f'{lf.face} {lf.offset_mm} {lf.strong}')
        check('its twin, the 538X, reads -y, weak', lx.face == '-y' and not lx.strong,
              f'{lx.face} {lx.offset_mm} {lx.strong}')

        face, turns, note = seat(p5380, intent())
        check('undeclared: the seat turns the 5380 by its body reading (unchanged)',
              face == '+y' and turns == [180.0], f'{face} {turns} {note}')
        face, turns, note = seat(p5380, intent(mating_face='-y'))
        check('declared -y: the seat turns it by the declaration, mouth out of the north edge',
              face == '-y' and turns == [0.0], f'{face} {turns} {note}')
        check('...and its note says the declaration outranks the weak reading',
              'facing it (-y)' in note and 'declared mating_face -y outranks the +y face' in note
              and 'weak' in note, note)

        hand = board(os.path.join(tmp, '5380_r0.kicad_pcb'), 'RJ45_Amphenol_RJHSE5380',
                     RJ_DRAW + RJ_BASE, rot=0)
        wrong = board(os.path.join(tmp, '5380_r180.kicad_pcb'), 'RJ45_Amphenol_RJHSE5380',
                      RJ_DRAW + RJ_BASE, rot=180)
        f, said, _t = grade(wrong, intent())
        check('undeclared: mouth-into-board grades as before (the weak reading passes)',
              f.get('disposition') == 'pass' and not said, f'{f} {said}')
        f, said, text = grade(wrong, intent(mating_face='-y'))
        check('declared: the mouth-into-board pose fails the facing at ERROR',
              f.get('disposition') == 'fail' and f.get('basis') == 'declared'
              and any(s == 'error' and 'points south' in m for s, m in said), f'{f} {said}')
        check('...with a WARN that the declared face and the body reading disagree',
              any(s == 'warn' and 'declared mating_face -y outranks' in m for s, m in said),
              str(said))
        f, said, text = grade(hand, intent(mating_face='-y'))
        line = next((ln for ln in text.splitlines() if ln.strip().startswith('J1 north: overhang')), '')
        check('declared: the right pose passes the facing on the declared face',
              f.get('disposition') == 'pass' and f.get('basis') == 'declared'
              and not any(s == 'error' for s, _m in said), f'{f} {said}')
        check('...the WARN says so there too, whichever pose',
              any(s == 'warn' and 'declared mating_face -y outranks' in m for s, m in said),
              str(said))
        check('...and the evidence line names both faces',
              'facing north [declared, strong] pass (declared -y over the body\'s weak +y)' in line,
              line)

        # 4. a STRONG reading still decides
        pshell = board(os.path.join(tmp, 'shell.kicad_pcb'), 'Shell', SHELL, rot=180)
        ls = reading(pshell)
        check('the shell reads +y, strong', ls.face == '+y' and ls.strong,
              f'{ls.face} {ls.offset_mm} {ls.strong}')
        face, turns, note = seat(pshell, intent(mating_face='-y'))
        check('declared -y against a strong +y: the seat keeps the body reading',
              face == '+y' and turns == [180.0], f'{face} {turns}')
        check('...and its note says the strong reading overrules the declaration',
              'overrules the declared mating_face -y' in note, note)
        f, said, _t = grade(pshell, intent(mating_face='-y'))
        check('the grade reads the strong face too (facing north: pass)',
              f.get('disposition') == 'pass' and f.get('local_face') == '+y', str(f))
        check('...and warns that it overrules the declaration',
              any(s == 'warn' and 'overrules the declared mating_face -y' in m for s, m in said),
              str(said))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
