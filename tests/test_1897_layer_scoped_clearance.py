#!/usr/bin/env python3
"""#1897: a class-scoped `.kicad_dru` clearance rule with a `(layer ...)` clause.

The scoped channel (#1491) took a clearance rule scoped by net class, item
type or net name only when it had no `(layer ...)` clause. With one, the rule
was routed to by nothing and listed `NOT GRADED` by check_drc.

Measured with kicad-cli 10.0.8 on tests/fixtures/1897/pairs (Default and BUS
class clearance 0.1 mm; A1, A2 in BUS, X in Default; every gap 0.2 mm; the
same copper drawn on In1.Cu and on F.Cu):

    (rule "bus out" (layer "In1.Cu")
      (condition "A.hasNetclass('BUS') && !B.hasNetclass('BUS')")
      (constraint clearance (min 0.3mm)))

binds a track against a track on In1.Cu, a via against a track on In1.Cu and
a via against a via: 3 errors, none for the copper on F.Cu and none inside
the class. A layer-only rule for In1.Cu LATER in the file replaces it (0
errors), one EARLIER does not (3).

Routing fixtures: A1 has a through-hole pad either side of a wall of locked X
tracks, drawn on In1.Cu and F.Cu alike, with one slot 0.4 mm wide. A 0.1 mm
track passes it at a 0.1 mm clearance and not at 0.3.
  * wall_tight: classes 0.1, the rule 0.3 on In1.Cu.
  * wall_relax: classes 0.3, the rule 0.1 on In1.Cu.
The unfixed step routed wall_tight on In1.Cu with 2 KiCad clearance errors.

Checks:
  1. check_drc grades the pairs KiCad does, by the rule's name, with the
     clause as a layer name and as `inner`, and in file order against a
     layer-only rule; the rule is no longer `NOT GRADED`;
  2. wall_tight: not routed on In1.Cu, routed on F.Cu; with the rule file
     emptied it is routed on In1.Cu and check_drc fails that board once the
     rule is back;
  3. wall_relax: routed on In1.Cu, not on F.Cu;
  4. wall_tight with a layer-only 0.1 mm rule for In1.Cu after the scoped
     rule: routed on In1.Cu;
  5. the run names the rule with its layer, and the map's reference counts
     balance (KICAD_OBSTACLE_AUDIT).
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1897')
FAILS = []
NO_RULES = '(version 1)\n'
COND = "A.hasNetclass('BUS') && !B.hasNetclass('BUS')"


def scoped(mm, clause='"In1.Cu"'):
    return (f'(rule "bus out"\n  (layer {clause})\n  (condition "{COND}")\n'
            f'  (constraint clearance (min {mm}mm)))\n')


def layer_only(mm):
    return f'(rule "in1"\n  (layer "In1.Cu")\n  (constraint clearance (min {mm}mm)))\n'


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru=None):
    os.makedirs(td, exist_ok=True)
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        shutil.copy(evidence(os.path.join(FIX, name + ext), 'the fixture'),
                    os.path.join(td, name + ext))
    if dru is not None:
        with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return os.path.join(td, name + '.kicad_pcb')


def drc(board, flagged=False):
    argv = PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                 evidence(board, 'the board'), '--clearance-margin-mm', '0.0005']
    r = (check(argv, refuse='0.3000mm', code=1) if flagged else check(argv, accept=True))
    return r.stdout or ''


def route(td, board, tag, layer, env=None):
    out = os.path.join(td, tag + '.kicad_pcb')
    summary = os.path.join(td, tag + '.json')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out,
                    '--nets', 'A1', '--layers', layer, '--grid-step', '0.01',
                    '--escalation', 'off', '--json-out', summary],
              accept=True, timeout=900, env=env)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return (out, sorted(doc.get('routed_single') or []),
            sorted(doc.get('failed_single') or []), (r.stdout or '') + (r.stderr or ''))


def grade(td):
    print("1. check_drc grades the pairs KiCad does")
    for tag, dru, rows in (
            ('the rule', '(version 1)\n' + scoped(0.3), 3),
            ('the rule on (layer inner)', '(version 1)\n' + scoped(0.3, 'inner'), 3),
            ('a layer-only rule after it', '(version 1)\n' + scoped(0.3) + layer_only(0.15), 0),
            ('a layer-only rule before it', '(version 1)\n' + layer_only(0.15) + scoped(0.3), 3),
            ('no rule', NO_RULES, 0)):
        board = stage(os.path.join(td, 'g_' + tag.replace(' ', '_').replace('(', '').replace(')', '')),
                      'pairs', dru=dru)
        text = drc(board, flagged=bool(rows))
        if rows:
            found = f'FOUND {rows} DRC VIOLATIONS' in text
            kinds = all(k in text for k in ('SEGMENT-SEGMENT violations (1)',
                                            'VIA-SEGMENT violations (1)',
                                            'VIA-VIA violations (1)'))
            ok(found and kinds, f"{tag}: one track/track, one via/track and one via/via row")
            ok('Layer: F.Cu' not in text, f"{tag}: none of them on F.Cu")
        else:
            ok('NO DRC VIOLATIONS FOUND' in text, f"{tag}: clean")
        ok('NOT GRADED (clearance)' not in text, f"{tag}: nothing is listed as not graded")


def main():
    with tempfile.TemporaryDirectory() as td:
        grade(td)

        print("2. a rule that tightens In1.Cu")
        sub = os.path.join(td, 'tight')
        board = stage(sub, 'wall_tight')
        _o, routed, failed, log = route(sub, board, 'in1', 'In1.Cu')
        ok(routed == [] and failed == ['A1'], f"on In1.Cu A1 is not routed: {routed} / {failed}")
        ok("'bus out' 0.3mm" in log and 'on In1.Cu' in log,
           "the run names the rule with its layer")
        out, routed, failed, _log = route(sub, board, 'f', 'F.Cu')
        ok(routed == ['A1'], f"on F.Cu it is routed: {routed} / {failed}")
        ok('NO DRC VIOLATIONS FOUND' in drc(out), "and check_drc is clean")
        sub2 = os.path.join(td, 'tight_norule')
        out, routed, failed, _log = route(sub2, stage(sub2, 'wall_tight', dru=NO_RULES),
                                          'in1', 'In1.Cu')
        ok(routed == ['A1'], f"with the rule file emptied it is routed on In1.Cu: {routed}")
        shutil.copy(os.path.join(FIX, 'wall_tight.kicad_dru'), os.path.join(sub2, 'in1.kicad_dru'))
        ok('SEGMENT-SEGMENT violations' in drc(out, flagged=True),
           "and check_drc fails that board at the rule once it is back")

        print("3. a rule that relaxes In1.Cu")
        sub = os.path.join(td, 'relax')
        board = stage(sub, 'wall_relax')
        out, routed, failed, _log = route(sub, board, 'in1', 'In1.Cu')
        ok(routed == ['A1'], f"on In1.Cu A1 is routed: {routed} / {failed}")
        ok('NO DRC VIOLATIONS FOUND' in drc(out), "and check_drc is clean")
        _o, routed, failed, _log = route(sub, board, 'f', 'F.Cu')
        ok(routed == [] and failed == ['A1'], f"on F.Cu it is not: {routed} / {failed}")

        print("4. a layer-only rule later in the file wins")
        sub = os.path.join(td, 'order')
        board = stage(sub, 'wall_tight', dru='(version 1)\n' + scoped(0.3) + layer_only(0.1))
        out, routed, failed, _log = route(sub, board, 'in1', 'In1.Cu')
        ok(routed == ['A1'], f"A1 is routed on In1.Cu: {routed} / {failed}")
        ok('NO DRC VIOLATIONS FOUND' in drc(out), "and check_drc is clean")

        print("5. the map's reference counts balance")
        sub = os.path.join(td, 'audit')
        env = dict(os.environ, KICAD_OBSTACLE_AUDIT='1')
        _o, _r, _f, log = route(sub, stage(sub, 'wall_relax'), 'audit', 'In1.Cu', env=env)
        ok('[OBSTACLE AUDIT]' in log and 'BALANCED' in log,
           "the working map returns exactly to the base map")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
