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
     balance (KICAD_OBSTACLE_AUDIT);
  6. the "keep pairs apart" shape, `... && !A.isCoupledDiffPair()`, between
     two nets of the class (kicad-cli: A1 against A2 on both layers with no
     clause, on In1.Cu alone with one, and nothing once the two nets are one
     pair's P and N): graded the same, with or without a layer clause;
  7. and routed to: the wall drawn in a net of the class keeps A1 out of the
     slot on the layer the rule names;
  8. a pad against a via, on a board whose only per-layer rule is the scoped
     one: graded as kicad-cli 10.0.6 grades it, a through-hole pad on the
     rule's inner layer and an SMD pad on F.Cu (check_drc used to crash);
  9. route_diff.coupled_pair_floors takes such a rule binding a pair's P
     against its N on the layer it names, and floors the gap at it only when
     the pair routes on that layer alone.
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


APART = "A.hasNetclass('BUS') && B.hasNetclass('BUS') && !A.isCoupledDiffPair()"


def apart(mm, clause=None):
    lay = f'  (layer {clause})\n' if clause else ''
    return (f'(rule "bus in"\n{lay}  (condition "{APART}")\n'
            f'  (constraint clearance (min {mm}mm)))\n')


def rename(board, pairs):
    """Rewrite net names in a staged board."""
    with open(board, encoding='utf-8') as fh:
        text = fh.read()
    for a, b in pairs:
        text = text.replace(f'(net "{a}")', f'(net "{b}")')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return board


def _fp(ref, x, y, pad):
    return (f'\t(footprint "fixture:{ref}"\n\t\t(layer "F.Cu")\n\t\t(at {x} {y})\n'
            f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0 0)\n\t\t\t(layer "F.SilkS")\n'
            f'\t\t\t(hide yes)\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n'
            f'\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n{pad}\t)\n')


def _via(x, y, net):
    return (f'\t(via\n\t\t(at {x} {y})\n\t\t(size 0.5)\n\t\t(drill 0.25)\n'
            f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n\t)\n')


def with_pads(board):
    """Add two pads of net X, each 0.2 mm from an A1 via: a through-hole pad
    (copper on In1.Cu too) and an F.Cu SMD pad."""
    tht = ('\t\t(pad "1" thru_hole circle\n\t\t\t(at 0 0)\n\t\t\t(size 0.6 0.6)\n'
           '\t\t\t(drill 0.3)\n\t\t\t(layers "*.Cu" "*.Mask")\n'
           '\t\t\t(remove_unused_layers no)\n\t\t\t(net "X")\n\t\t)\n')
    smd = ('\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.6 0.6)\n'
           '\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "X")\n\t\t)\n')
    add = (_fp('P1', 22, 17.5, tht) + _via(22.75, 17.5, 'A1')
           + _fp('P2', 30, 17.5, smd) + _via(30.75, 17.5, 'A1'))
    with open(board, encoding='utf-8') as fh:
        text = fh.read()
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(text.replace('\t(embedded_fonts no)', add + '\t(embedded_fonts no)'))
    return board


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


def grade_pads(td):
    print("8. a pad against a via, where the layered rule is the board's only layer rule")
    # kicad-cli 10.0.6 on `with_pads`: on In1.Cu the through-hole pad is
    # flagged against the A1 via and the F.Cu SMD pad is not; on F.Cu both.
    # check_drc used to crash on the first pad pair it priced (a pad's
    # layers were not passed when the board had no #498 layer rule).
    for tag, dru, rows, pad_via in (('on In1.Cu', scoped(0.3), 4, 1),
                                    ('on F.Cu', scoped(0.3, '"F.Cu"'), 5, 2)):
        board = with_pads(stage(os.path.join(td, 'pad_' + tag.replace(' ', '_').replace('.', '')),
                                'pairs', dru='(version 1)\n' + dru))
        text = drc(board, flagged=True)
        ok(f'FOUND {rows} DRC VIOLATIONS' in text
           and f'PAD-VIA violations ({pad_via})' in text,
           f"{tag}: {rows} rows, {pad_via} of them pad against via")


def pair_floor(td):
    print("9. route_diff reads a layered scoped rule binding a pair's P against its N")
    from types import SimpleNamespace
    from kicad_parser import parse_kicad_pcb
    from route_diff import coupled_pair_floors
    dru = ('(version 1)\n(rule "pn in1"\n  (layer "In1.Cu")\n'
           "  (condition \"A.hasNetclass('BUS') && B.hasNetclass('BUS')\")\n"
           '  (constraint clearance (min 0.15mm)))\n')
    board = rename(stage(os.path.join(td, 'pn'), 'pairs', dru=dru),
                   (('A1', 'AD_P'), ('A2', 'AD_N')))
    pcb = parse_kicad_pcb(board)
    ids = {n.name: i for i, n in pcb.nets.items()}
    pair = SimpleNamespace(p_net_id=ids['AD_P'], n_net_id=ids['AD_N'])
    key = frozenset((ids['AD_P'], ids['AD_N']))
    floors, cmap = coupled_pair_floors(pcb, board, [pair], ['In1.Cu'])
    got = cmap.get(key) or {}
    ok(got.get('track') == {'In1.Cu': 0.15} and got.get('stack') is None,
       f"P<->N governed on In1.Cu alone, at the rule: {got}")
    ok(floors.get(ids['AD_P'], (None,))[:2] == (0.15, ['pn in1']),
       f"routed on In1.Cu only, the pair's gap is floored at the rule: {floors}")
    floors, _c = coupled_pair_floors(pcb, board, [pair], ['F.Cu', 'In1.Cu'])
    ok(floors == {}, f"routed on F.Cu too, no one gap floor (the rule does not bind F.Cu): {floors}")


def grade_apart(td):
    print('6. the "keep pairs apart" shape is graded as KiCad grades it')
    for tag, dru, layers in (('no layer clause', apart(0.3), ['In1.Cu', 'F.Cu']),
                             ('on In1.Cu', apart(0.3, '"In1.Cu"'), ['In1.Cu'])):
        board = stage(os.path.join(td, 'a_' + tag.replace(' ', '_')), 'pairs',
                      dru='(version 1)\n' + dru)
        text = drc(board, flagged=True)
        ok(f'FOUND {len(layers)} DRC VIOLATIONS' in text
           and f'SEGMENT-SEGMENT violations ({len(layers)})' in text
           and all(f'Layer: {l}' in text for l in layers)
           and 'A1 <-> A2' in text,
           f"{tag}: A1 against A2 on {' and '.join(layers)}, and nothing else")
        ok('NOT GRADED (clearance)' not in text, f"{tag}: nothing is listed as not graded")
        board = rename(stage(os.path.join(td, 'p_' + tag.replace(' ', '_')), 'pairs',
                             dru='(version 1)\n' + dru),
                       (('A1', 'AD_P'), ('A2', 'AD_N')))
        ok('NO DRC VIOLATIONS FOUND' in drc(board),
           f"{tag}: clean once the two nets are one pair's P and N")


def route_apart(td):
    print('7. and routed to')
    sub = os.path.join(td, 'apart')
    board = rename(stage(sub, 'wall_tight', dru='(version 1)\n' + apart(0.3, '"In1.Cu"')),
                   (('X', 'A9'),))
    _o, routed, failed, _log = route(sub, board, 'in1', 'In1.Cu')
    ok(routed == [] and failed == ['A1'],
       f"a wall of the class closes the slot on In1.Cu: {routed} / {failed}")
    out, routed, failed, _log = route(sub, board, 'f', 'F.Cu')
    ok(routed == ['A1'], f"and leaves it open on F.Cu: {routed} / {failed}")
    ok('NO DRC VIOLATIONS FOUND' in drc(out), "check_drc is clean")


def whole_board(td):
    """The post-route passes on a real board: every pass that moves copper
    prices a pair at the rule on the layer where the two items meet. The via
    nudge read the scoped rules without their (layer ...) clause, and moved a
    via 0.379 mm from a track the rule holds at 0.4."""
    print("1e. splitflap_driver routed under a rule that widens F.Cu")
    sub = os.path.join(td, 'splitflap')
    os.makedirs(sub, exist_ok=True)
    src = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
    board = os.path.join(sub, 'in.kicad_pcb')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                evidence(src, 'the board'), board], accept=True)
    with open(os.path.join(sub, 'in.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n(rule "front wide"\n  (layer "F.Cu")\n'
                 '  (condition "A.hasNetclass(\'Default\') && B.hasNetclass(\'Default\')")\n'
                 '  (constraint clearance (min 0.4mm)))\n')
    out = os.path.join(sub, 'out.kicad_pcb')
    summary = os.path.join(sub, 'out.json')
    check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out,
                '--json-out', summary], accept=True, timeout=900)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    ok(not (doc.get('failed_single') or []) and len(doc.get('routed_single') or []) > 50,
       f"every net routes: {len(doc.get('routed_single') or [])} routed, "
       f"{doc.get('failed_single')} failed")
    r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                    evidence(out, 'the routed board')], accept=True)
    ok('NO DRC VIOLATIONS FOUND' in (r.stdout or '') and "'front wide':0.4" in (r.stdout or ''),
       "check_drc grades the rule and finds the board clean")


def main():
    with tempfile.TemporaryDirectory() as td:
        grade(td)
        grade_pads(td)
        grade_apart(td)
        route_apart(td)
        pair_floor(td)
        whole_board(td)

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
