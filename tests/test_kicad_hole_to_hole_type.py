#!/usr/bin/env python3
"""kicad_drc_compare matches check_drc's drill pairs against KiCad 10's
`hole_to_hole` / `holes_co_located` items.

check_drc grades two drills too close as `via-drill-hole` (and
`pad-drill-via-drill[-same-net]`). KiCad 10 reports the same pair as
`hole_to_hole`, or `holes_co_located` when the centres coincide. The
comparator's copper-type list named only the older `hole_near_hole`, so KiCad's
item was dropped and every such pair read `checkdrc_only`, never matched:
two Net-(C1-Pad1) vias 0.18 mm apart on one board (latest wave), /GNDPLL0
and +3V3 pairs on another (old main).

Part 1 (always runs, no KiCad): the report parser keeps both types at error
and still drops an out-of-scope class (control); a KiCad 10 hole_to_hole item
pairs with check_drc's via-drill-hole; staging lifts a project's `warning`
severity for both to error (precondition: the fixture project says warning,
as the measured board's does).

Part 2 (kicad-cli): a board with the measured pair and a co-located pair is
graded end to end by the comparator CLI: KiCad reports both types, both
match, the board is CONSISTENT. Control: the same board graded with the old
list reproduces the defect (checkdrc_only=2, matched=0).

    python3 tests/test_kicad_hole_to_hole_type.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STRESS = os.path.join(ROOT, 'tests', 'stress')
for p in (HERE, STRESS, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools'),
          os.path.join(ROOT, 'py_placer'), ROOT):
    sys.path.insert(0, p)

from run_utils import check, evidence  # noqa: E402
import kicad_drc_compare as kdc  # noqa: E402

COMPARE = os.path.join(STRESS, 'kicad_drc_compare.py')
NET = 'Net-(C1-Pad1)'
HOLE_TYPES = {'hole_to_hole', 'holes_co_located'}

fails = []


def ok(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name
          + (f'  {detail}' if detail and not cond else ''))
    if not cond:
        fails.append(name)


def _item(desc, x, y):
    return {'description': desc, 'pos': {'x': x, 'y': y}, 'uuid': str(uuid.uuid4())}


# kicad-cli 10.0.0's item for the measured pair, at the error staging gives it
# (the board's own project says warning).
H2H = {'type': 'hole_to_hole', 'severity': 'error',
       'description': 'Drilled hole too close to other hole (board setup '
                      'constraints min 0.2495 mm; actual 0.0000 mm)',
       'items': [_item(f'Via [{NET}] on F.Cu - B.Cu', 112.5, 78.25),
                 _item(f'Via [{NET}] on F.Cu - B.Cu', 112.4, 78.1)]}
COLOC = {'type': 'holes_co_located', 'severity': 'error',
         'description': 'Drilled holes co-located',
         'items': [_item('Via [GND] on F.Cu - B.Cu', 20.0, 10.0),
                   _item('Via [GND] on F.Cu - B.Cu', 20.0, 10.0)]}


def part1(tmp):
    report = {'violations': [
        H2H, COLOC,
        dict(H2H, severity='warning'),
        {'type': 'solder_mask_bridge', 'severity': 'error', 'description': 'x',
         'items': [_item(f'Via [{NET}] on F.Cu - B.Cu', 1.0, 1.0)]},
    ]}
    got = kdc.parse_kicad_report(report)
    types = sorted(v['type'] for v in got)
    ok('parse keeps hole_to_hole and holes_co_located at error',
       types == ['hole_to_hole', 'holes_co_located'], types)
    ok('control: an out-of-scope class (solder_mask_bridge) is still dropped',
       'solder_mask_bridge' not in types, types)
    ok('the copper channel stays error-only: the warning twin is dropped, one kept',
       types.count('hole_to_hole') == 1, types)
    h2h = next((v for v in got if v['type'] == 'hole_to_hole'), None)
    ok("the item carries KiCad's net and position",
       h2h is not None and h2h['nets'] == frozenset({NET})
       and h2h['pos'] == (112.5, 78.25), h2h)

    # check_drc's row for the same pair, as run_check_drc shapes it
    cd = [{'type': 'via-drill-hole', 'nets': frozenset({NET}), 'pos': (112.4, 78.1)}]
    matched, k_only, c_only = kdc.match([h2h] if h2h else [], cd)
    ok("check_drc's via-drill-hole matches KiCad's hole_to_hole",
       len(matched) == 1 and not k_only and not c_only,
       (len(matched), k_only, c_only))

    # staging lifts the project's own warning to error
    pcb = os.path.join(tmp, 'stage.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(board_text(far=True))
    write_pro(pcb)
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', encoding='utf-8') as f:
        src = json.load(f)['board']['design_settings']['rule_severities']
    ok('precondition: the fixture project says warning for both (as measured)',
       all(src.get(k) == 'warning' for k in HOLE_TYPES), src)
    d, b2 = kdc._staged_copy(pcb, 0.2)
    try:
        with open(evidence(b2[:-len('.kicad_pcb')] + '.kicad_pro', 'staged project'),
                  encoding='utf-8') as f:
            sev = json.load(f)['board']['design_settings']['rule_severities']
    finally:
        shutil.rmtree(d, ignore_errors=True)
    ok('staging lifts both to error', all(sev.get(k) == 'error' for k in HOLE_TYPES),
       {k: sev.get(k) for k in HOLE_TYPES})


_HEAD = ('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
         '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n'
         '\t\t(legacy_teardrops no)\n\t)\n\t(paper "A4")\n\t(layers\n'
         '\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n\t\t(1 "F.Mask" user)\n'
         '\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n\t)\n'
         '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n')


def board_text(far=False):
    """The measured pair (0.45/0.3 and 0.25/0.15 vias, 0.18 mm apart) and two
    GND vias at one point; `far` spreads both pairs 3 mm apart (clean)."""
    n = [0]

    def u():
        n[0] += 1
        return str(uuid.UUID(int=n[0]))

    def via(x, y, size, drill, net):
        return (f'\t(via\n\t\t(at {x} {y})\n\t\t(size {size})\n\t\t(drill {drill})\n'
                f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n\t\t(uuid "{u()}")\n\t)\n')

    dx = 3.0 if far else 0.0
    body = [via(10.0, 10.0, 0.45, 0.3, NET),
            via(10.1 + dx, 10.15, 0.25, 0.15, NET),
            via(20.0, 10.0, 0.6, 0.3, 'GND'),
            via(20.0 + dx, 10.0, 0.6, 0.3, 'GND')]
    for (x1, y1, x2, y2) in ((0, 0, 30, 0), (30, 0, 30, 20), (30, 20, 0, 20), (0, 20, 0, 0)):
        body.append(f'\t(gr_line\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
                    f'\t\t(stroke (width 0.05) (type default))\n\t\t(layer "Edge.Cuts")\n'
                    f'\t\t(uuid "{u()}")\n\t)\n')
    return _HEAD + ''.join(body) + ')\n'


def write_pro(pcb):
    pro = {'board': {'design_settings': {
               'rules': {'min_clearance': 0.2, 'min_hole_to_hole': 0.25,
                         'min_copper_edge_clearance': 0.5},
               'rule_severities': {'hole_to_hole': 'warning',
                                   'holes_co_located': 'warning'}}},
           'net_settings': {'classes': [{'name': 'Default', 'clearance': 0.2,
                                         'track_width': 0.2, 'via_diameter': 0.6,
                                         'via_drill': 0.3}]}}
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump(pro, f)


def part2(tmp):
    if not (os.path.isfile(kdc.KICAD_CLI) or shutil.which(kdc.KICAD_CLI)):
        print(f'SKIP: part 2 needs kicad-cli (not found at {kdc.KICAD_CLI}); '
              f'part 1 asserted the parser, matcher and staging')
        return
    pcb = os.path.join(tmp, 'pairs.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(board_text())
    write_pro(pcb)
    evidence(pcb, 'fixture board')

    # Precondition, read RAW (not through the parser under test): these are
    # the names kicad-cli itself gives the two pairs.
    d, b2 = kdc._staged_copy(pcb, 0.2)
    try:
        out = os.path.join(d, 'raw.json')
        subprocess.run([kdc.KICAD_CLI, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '--output', out, b2], capture_output=True, text=True, timeout=600)
        with open(evidence(out, 'kicad-cli drc json'), encoding='utf-8') as f:
            raw = json.load(f)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    hole_rows = sorted(v['type'] for v in raw.get('violations', [])
                       if 'hole' in v['type'])
    ok('precondition: kicad-cli reports the pair as hole_to_hole and the '
       'co-located vias as holes_co_located, nothing else hole-related',
       hole_rows == ['hole_to_hole', 'holes_co_located'], hole_rows)

    items, err = kdc.kicad_items_for(pcb, 0.2)
    kt = sorted(v['type'] for v in (items or []))
    ok('the comparator keeps both KiCad items', not err and kt == sorted(HOLE_TYPES),
       err or kt)

    r = check([sys.executable, '-X', 'utf8', COMPARE, pcb], accept=True)
    line = next((ln for ln in r.stdout.splitlines()
                 if ln.startswith('pairs.kicad_pcb:')), '')
    ok('comparator CLI: both pairs matched, nothing one-sided, CONSISTENT',
       'check_drc=2 ' in line and 'matched=2 kicad_only=0 checkdrc_only=0' in line
       and line.rstrip().endswith('CONSISTENT'), line or r.stdout[-800:])

    # control: the list before this fix reproduces the defect on this board
    full = kdc.KICAD_COPPER_TYPES
    kdc.KICAD_COPPER_TYPES = full - HOLE_TYPES
    try:
        old = kdc.compare_board_data(pcb)
    finally:
        kdc.KICAD_COPPER_TYPES = full
    ok('control: with the old list the same pairs read checkdrc_only=2, matched=0',
       old and old.get('checkdrc_only') == 2 and old.get('matched') == 0
       and old.get('kicad') == 0,
       {k: (old or {}).get(k) for k in ('kicad', 'check_drc', 'matched', 'checkdrc_only')})


def part1b(tmp):
    """The in-process grade prices drill pairs at the board's min_hole_to_hole,
    as staging leaves it for kicad-cli and as the check_drc CLI pins it."""
    pcb = os.path.join(tmp, 'gap.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(board_text(far=True).replace(
            '(at 13.1 10.15)', '(at 10.0 10.53)').replace(
            '(size 0.25)\n\t\t(drill 0.15)', '(size 0.45)\n\t\t(drill 0.3)'))
    write_pro(pcb)
    pairs = [v for v in kdc.run_check_drc(evidence(pcb, 'gap board'), 0.2)
             if v['type'] == 'via-drill-hole']
    ok("a pair 0.23 mm apart under the board's 0.25 is graded",
       len(pairs) == 1 and pairs[0]['nets'] == frozenset({NET}), pairs)
    pro = pcb[:-len('.kicad_pcb')] + '.kicad_pro'
    with open(pro, encoding='utf-8') as f:
        doc = json.load(f)
    del doc['board']['design_settings']['rules']['min_hole_to_hole']
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    ok('control: with no board floor the same pair clears the 0.20 default',
       not [v for v in kdc.run_check_drc(pcb, 0.2) if v['type'] == 'via-drill-hole'])


def main():
    with tempfile.TemporaryDirectory() as tmp:
        part1(tmp)
        part1b(tmp)
        part2(tmp)
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
