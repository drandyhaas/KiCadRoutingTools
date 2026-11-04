#!/usr/bin/env python3
"""#1652: a declared `mating_face` a weak reading agrees with is not evidence.

#1586 made a declared `mating_face` outrank a WEAK body reading, and where
the weak reading AGREED with the declaration the grade printed the face
`strong` -- the word that means "a wrong turn fails at ERROR". On a library
copy of a jack drawn turned 180 degrees in its own frame, the body reads the
wrong face weakly, an author can declare that same wrong face, and the grade
printed `facing north [courtyard+fab+declared, strong] pass` with the mouth
into the board, while the RIGHT declaration was the one that warned.

The severity and the evidence are now two things. `strong` on the facing
row stays the severity (a declared face's wrong turn still fails at ERROR);
the clause's second word is `evidence` (`mating.face_evidence`): `strong` /
`weak` for the body's own reading, `marker` for a declaration the
footprint's PCB-edge marker confirms, `unverified` for any other declared
face, whichever way the weak reading agrees. Pinned here:
  1. the turned copy, declared wrong (agreeing with its weak reading): the
     clause reads `[courtyard+fab+declared, unverified]`, the row is still
     `strong` (severity), and the opposite turn still fails at ERROR;
  2. declared right (overruling the weak reading): `[declared, unverified]`;
  3. undeclared: `[courtyard+fab, weak]`, as before;
  4. a strong reading reads `strong`;
  5. a body that abstains with a PCB-edge marker: a declaration the marker
     confirms reads `[declared, marker]`, one against it reads `unverified`
     and is said at WARN.

    python3 tests/test_1652_declared_face_evidence.py
"""
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TESTS_DIR)
import test_1586_declared_mating_face_outranks_weak as T      # noqa: E402

pad, board, intent, grade, reading = T.pad, T.board, T.intent, T.grade, T.reading

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


# The stock jack's geometry (T.RJ_BASE / T.RJ_DRAW) turned 180 degrees in
# its own frame: this copy opens at +y, on its pegs' side, and reads -y weak.
TURNED_BASE = (pad('', 2.79, 2.54, 'np_thru_hole', 3.25, 3.25)
               + pad('', -9.91, 2.54, 'np_thru_hole', 3.25, 3.25)
               + ''.join(pad(i + 1, -round(1.016 * i, 3), -1.78 * (i % 2)) for i in range(8))
               + pad('SH', 4.57, -0.89, s=2.3, d=1.57) + pad('SH', -11.69, -0.89, s=2.3, d=1.57))
TURNED_DRAW = ('    (fp_rect (start -11.81 -7.75) (end 4.7 8) (stroke (width 0.1) (type default)) '
               '(layer "F.Fab"))\n'
               '    (fp_rect (start -13.34 -8.25) (end 6.22 8.5) (stroke (width 0.05) '
               '(type default)) (layer "F.CrtYd"))\n')
TURNED = TURNED_DRAW + TURNED_BASE
# A part drawn symmetric about its pads (the body abstains) with a "PCB Edge"
# marker line on its -y side: the marker puts the board edge at -y.
MARKED = ('    (fp_rect (start -3 -3) (end 3 3) (stroke (width 0.05) (type default)) '
          '(layer "F.CrtYd"))\n'
          '    (fp_text user "PCB Edge" (at 0 -5) (layer "Dwgs.User"))\n'
          '    (fp_line (start -4 -5.5) (end 4 -5.5) (stroke (width 0.1) (type default)) '
          '(layer "Dwgs.User"))\n'
          '    (pad "1" smd rect (at -1 0) (size 0.6 1.2) (layers "F.Cu" "F.Mask"))\n'
          '    (pad "2" smd rect (at 1 0) (size 0.6 1.2) (layers "F.Cu" "F.Mask"))\n')


def clause(text):
    line = next((ln for ln in text.splitlines() if ln.strip().startswith('J1 north: overhang')), '')
    return line[line.find('facing'):].strip()


def main():
    tmp = tempfile.mkdtemp(prefix='t1652_')
    try:
        lf = reading(board(os.path.join(tmp, 'r.kicad_pcb'), 'RJ45_turned_copy', TURNED))
        check('the turned copy reads -y, weak (it opens at +y)',
              lf.face == '-y' and not lf.strong, f'{lf.face} {lf.offset_mm} {lf.strong}')

        # 1. the WRONG declaration, which the weak reading agrees with
        at0 = board(os.path.join(tmp, 'g0.kicad_pcb'), 'RJ45_turned_copy', TURNED, rot=0)
        f, said, text = grade(at0, intent(mating_face='-y'))
        check('declared -y (agrees with the weak reading): the clause says unverified, '
              'not strong',
              clause(text) == 'facing north [courtyard+fab+declared, unverified] pass', clause(text))
        check('...the row carries the evidence apart from the severity',
              f.get('evidence') == 'unverified' and f.get('strong') is True, str(f))
        at180 = board(os.path.join(tmp, 'g180.kicad_pcb'), 'RJ45_turned_copy', TURNED, rot=180)
        f, said, text = grade(at180, intent(mating_face='-y'))
        check('...and a wrong turn of the declared face still fails at ERROR',
              f.get('disposition') == 'fail'
              and any(s == 'error' and 'points south' in m for s, m in said), f'{f} {said}')

        # 2. the RIGHT declaration, which overrules the weak reading
        f, said, text = grade(at180, intent(mating_face='+y'))
        check('declared +y (overrules the weak reading): unverified too',
              clause(text).startswith('facing north [declared, unverified] pass'), clause(text))

        # 3. undeclared: unchanged
        f, said, text = grade(at0, intent())
        check('undeclared: the weak reading reads weak, as before',
              clause(text) == 'facing north [courtyard+fab, weak] pass' and not said,
              f'{clause(text)} {said}')

        # 4. a strong reading is evidence
        shell = board(os.path.join(tmp, 'shell.kicad_pcb'), 'Shell', T.SHELL, rot=180)
        f, said, text = grade(shell, intent())
        check('a strong reading reads strong', f.get('evidence') == 'strong'
              and '[courtyard+fab, strong]' in clause(text), clause(text))

        # 5. the PCB-edge marker, where the body abstains
        lm = reading(board(os.path.join(tmp, 'm.kicad_pcb'), 'Marked', MARKED))
        check('the marked part: the body abstains, the marker reads -y',
              lm.face is None and lm.marker is not None and lm.marker.face == '-y',
              f'{lm.face} {lm.reason} {lm.marker}')
        marked = board(os.path.join(tmp, 'm0.kicad_pcb'), 'Marked', MARKED, rot=0)
        f, said, text = grade(marked, intent(mating_face='-y'))
        check('a declaration the marker confirms reads marker',
              clause(text) == 'facing north [declared, marker] pass' and not said,
              f'{clause(text)} {said}')
        f, said, text = grade(board(os.path.join(tmp, 'm180.kicad_pcb'), 'Marked', MARKED,
                                    rot=180), intent(mating_face='+y'))
        check('a declaration against the marker reads unverified',
              clause(text) == 'facing north [declared, unverified] pass', clause(text))
        check('...and is said at WARN',
              any(s == 'warn' and 'against its PCB-edge marker' in m for s, m in said), str(said))

        # 6. an unverified face is said at WARN until `mating_face_source`
        #    records what it was read off
        src = {'mating_face_source': "the footprint's 3D model"}
        f, said, text = grade(at0, intent(mating_face='-y'))
        check('declared -y, no source: a WARN says it is unverified and asks for the source',
              [s for s, m in said] == ['warn'] and 'is unverified' in said[0][1]
              and 'mating_face_source' in said[0][1], str(said))
        f, said, text = grade(at0, intent(mating_face='-y', **src))
        check('...with a source: sourced, and nothing said',
              clause(text) == 'facing north [courtyard+fab+declared, sourced] pass'
              and not said and f.get('declared_source') == src['mating_face_source'],
              f'{clause(text)} {said}')
        f, said, text = grade(at180, intent(mating_face='-y', **src))
        check('...a wrong turn of a sourced face still fails at ERROR',
              any(s == 'error' and 'points south' in m for s, m in said), str(said))
        f, said, text = grade(at180, intent(mating_face='+y'))
        check('declared +y over the weak -y, no source: ONE WARN, the disagreement and the ask',
              [s for s, m in said] == ['warn'] and 'outranks' in said[0][1]
              and 'mating_face_source' in said[0][1], str(said))
        f, said, text = grade(at180, intent(mating_face='+y', **src))
        check('...with a source: nothing said, the clause still names both faces',
              not said and clause(text) == ("facing north [declared, sourced] pass "
                                            "(declared +y over the body's weak -y)"),
              f'{clause(text)} {said}')
        f, said, text = grade(shell, intent(mating_face='-y', **src))
        check('a source does not silence a STRONG reading overruling the declaration',
              any(s == 'warn' and 'overrules the declared mating_face -y' in m
                  for s, m in said), str(said))
        f, said, text = grade(board(os.path.join(tmp, 'm180b.kicad_pcb'), 'Marked', MARKED,
                                    rot=180), intent(mating_face='+y', **src))
        check('...nor the PCB-edge marker against it',
              any(s == 'warn' and 'against its PCB-edge marker' in m for s, m in said), str(said))
        f, said, text = grade(marked, intent(mating_face='-y'))
        check('a marker-confirmed face needs no source', not said, str(said))

        # 7. the field's contract
        for bad, why in (({'mating_face_source': 'model'}, 'without a mating_face'),
                         ({'mating_face': '-y', 'mating_face_source': ' '}, 'non-empty string')):
            try:
                intent(**bad)
                check(f'refused: {bad}', False, 'accepted')
            except T.floorplan.IntentError as exc:
                check(f'refused: {bad}', why in str(exc), str(exc))
        from placement import design_brief
        brief = design_brief.brief_from_dict({
            'schema': design_brief.SCHEMA_VERSION, 'kind': design_brief.KIND,
            'interfaces': [{'ref': 'J1', 'edge': 'north', 'mating_face': '+y',
                            'mating_face_source': 'drawing p.3'}]})
        frag, _rep = design_brief.compile_brief(brief, board_refs=('J1',))
        e = (frag.get('edge_connectors') or [{}])[0]
        check('a brief carries mating_face_source into the compiled entry',
              e.get('mating_face') == '+y' and e.get('mating_face_source') == 'drawing p.3', str(e))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
