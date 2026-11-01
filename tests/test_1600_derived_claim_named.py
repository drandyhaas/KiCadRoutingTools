"""#1600: a derived crystal claim left past its limit is named, called
derived, and its severity disclosed.

A crystal's claim at its IC's oscillator pins is a row the toolchain derives
(`context.source` `placement.crystals`), graded WARN whatever
`severity.proximity` says (by design: docs/floorplan-intent.md), and priced,
not held, by place_simple. So a seed can leave the crystal far past its
limit while the judge ranks only errors -- and three things hid it:
place_simple counted `1 claim limit(s) broken` without naming the part
(decaps and links are named), check_floorplan called the derived corpus
default "the declared ... proximity limit", and `severity.proximity: error`
was accepted and silently did not reach the row.

A synthetic board: U1 a QFN locked at one corner, its crystal Y1 locked
40 mm away, the load caps C1 / C2 free. Pinned here:

  1. the emitter writes the derived rows, Y1's at 8.1 mm;
  2. place_simple names the broken claim (`WARNING: claim Y1 near U1 ...
     past its derived 8.1 mm limit`) and lists it in `claims_left`;
  3. check_floorplan says `past the derived` for a derived row and
     `past the declared` for the same row with its source dropped;
  4. with `severity.proximity` set it prints a `note:` that the severity
     does not reach the derived rows, the rows stay warn, and nothing is
     printed without the key.

    python3 tests/test_1600_derived_claim_named.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check as run_check, evidence, tool      # noqa: E402

FAILS = []
NETS = ['GND', 'XI', 'XO', 'VDD', 'IO1']
ID = {n: i + 1 for i, n in enumerate(NETS)}


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _fp(ref, name, x, y, pads, locked=False):
    out = [f'  (footprint "{name}"' + (' (locked yes)' if locked else '')
           + f' (layer "F.Cu") (at {x} {y})',
           f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") '
           f'(hide yes))',
           '    (fp_rect (start -2 -1.6) (end 2 1.6) (stroke (width 0.05) '
           '(type default)) (layer "F.CrtYd"))']
    for k, net in enumerate(pads):
        out.append(f'    (pad "{k + 1}" smd rect (at {-1.2 + k * 0.6:.2f} 0) '
                   f'(size 0.4 0.8) (layers "F.Cu" "F.Paste" "F.Mask") '
                   f'(net {ID[net]} "{net}"))')
    return '\n'.join(out) + '\n  )\n'


def _board(path):
    c = 'Capacitor_SMD:C_0402_1005Metric'
    parts = [
        _fp('U1', 'Package_DFN_QFN:QFN-16_3x3mm_P0.5mm', 10, 10,
            ['XI', 'XO', 'GND', 'VDD', 'IO1'], locked=True),
        _fp('Y1', 'Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm', 50, 30,
            ['XI', 'GND', 'XO', 'GND'], locked=True),
        _fp('C1', c, 30, 20, ['XI', 'GND']),
        _fp('C2', c, 30, 25, ['XO', 'GND']),
        _fp('C3', c, 20, 30, ['VDD', 'GND'])]
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t1600")\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
                 '(44 "Edge.Cuts" user))\n  (net 0 "")\n'
                 + ''.join(f'  (net {ID[n]} "{n}")\n' for n in NETS)
                 + '  (gr_rect (start 0 0) (end 60 40) (stroke (width 0.1) '
                 '(type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')


def _grade(board, intent, refuse=None):
    """check_floorplan's text; `refuse`: the error it must exit 4 for."""
    argv = [sys.executable, tool('check_floorplan.py'), board, '--intent', intent]
    r = (run_check(argv, refuse=refuse, code=4) if refuse
         else run_check(argv, accept=True))
    return r.stdout


def main():
    tmp = tempfile.mkdtemp(prefix='t1600_')
    try:
        board = os.path.join(tmp, 'b.kicad_pcb')
        _board(board)
        intent = os.path.join(tmp, 'intent.json')
        run_check([sys.executable, tool('check_floorplan.py'),
                   evidence(board, 'board'), '--emit-intent', intent,
                   '--allow-unplaced'], accept=True)
        with open(evidence(intent, 'emitted intent'), encoding='utf-8') as fh:
            doc = json.load(fh)
        rows = {(p['ref'], p['near']): p for p in doc.get('proximity') or ()}
        y1 = rows.get(('Y1', 'U1')) or {}
        check('1. the emitter derives the crystal claim at 8.1 mm',
              y1.get('max_mm') == 8.1
              and (y1.get('context') or {}).get('source') == 'placement.crystals',
              json.dumps(doc.get('proximity')))

        # 2. place_simple names it
        groups = os.path.join(tmp, 'groups.json')
        with open(groups, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'groups': [
                {'name': 'all', 'refs': ['C1', 'C2', 'C3', 'U1', 'Y1']}]}, fh)
        placed = os.path.join(tmp, 's0.kicad_pcb')
        rep = os.path.join(tmp, 's0.json')
        r = run_check([sys.executable, tool('place_simple.py'), board, placed,
                       '--groups', groups, '--intent', intent, '--seed', '0',
                       '--json', rep], accept=True)
        line = next((l for l in r.stdout.splitlines()
                     if 'WARNING: claim Y1 near U1' in l), '')
        check('2. place_simple names the claim Y1 leaves past its limit',
              re.search(r'Y1 pad \d+ is [\d.]+ mm from U1, past its derived 8\.1 '
                        r'mm limit', line) is not None
              and 'severity.proximity' in line, line or r.stdout[-1200:])
        with open(evidence(rep, 'place_simple report'), encoding='utf-8') as fh:
            left = json.load(fh).get('claims_left') or []
        got = [c for c in left if c['ref'] == 'Y1' and c['near'] == 'U1']
        check('...and lists it in claims_left, derived, past 8.1 mm',
              len(got) == 1 and got[0]['derived'] is True
              and got[0]['gap_mm'] > 8.1, json.dumps(left))

        # 3. derived vs declared
        out = _grade(evidence(placed, 'placed board'), intent)
        prox = [l for l in out.splitlines() if 'proximity: Y1 pad' in l]
        check('3. the grade says "past the derived" for the derived row, at warn',
              prox and all('past the derived 8.10mm proximity limit' in l
                           and '[warn ]' in l for l in prox), '\n'.join(prox))
        check('...and prints no severity note when the intent sets none',
              'severity.proximity is' not in out, out[-600:])
        own = json.loads(json.dumps(doc))
        for p in own['proximity']:
            p['context'].pop('source', None)
        own_p = os.path.join(tmp, 'own.json')
        with open(own_p, 'w', encoding='utf-8') as fh:
            json.dump(own, fh)
        out = _grade(placed, own_p, refuse='past the declared 8.10mm proximity limit')
        prox = [l for l in out.splitlines() if 'proximity: Y1 pad' in l]
        check('...and "past the declared" once its source is dropped, an error',
              prox and all('past the declared 8.10mm proximity limit' in l
                           and '[ERROR]' in l for l in prox), '\n'.join(prox))

        # 4. severity.proximity disclosed
        sev = dict(doc, severity=dict(doc.get('severity') or {}, proximity='error'))
        sev_p = os.path.join(tmp, 'sev.json')
        with open(sev_p, 'w', encoding='utf-8') as fh:
            json.dump(sev, fh)
        out = _grade(placed, sev_p)
        note = next((l for l in out.splitlines() if l.strip().startswith('note:')
                     and 'severity.proximity' in l), '')
        n = sum(1 for p in doc['proximity']
                if (p.get('context') or {}).get('source') == 'placement.crystals')
        check('4. severity.proximity set: a note says it does not reach the '
              'derived rows',
              f"does not reach the {n} derived proximity row(s)" in note, note
              or out[-800:])
        prox = [l for l in out.splitlines() if 'proximity: Y1 pad' in l]
        check('...and the derived rows stay warn (the design is unchanged)',
              prox and all('[warn ]' in l for l in prox), '\n'.join(prox))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
