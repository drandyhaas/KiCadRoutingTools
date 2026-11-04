"""#1416: fit_template writes a template row into an intent without asking
whether its members hold that intent's decap limits.

A member seated where the datasheet draws it, but past `decaps.max_distance_mm`
(or `max_pin_distance_mm`) of the chip it decouples, grades ERROR on every
placement that holds the row as drawn. fit_template now measures each member at
its template pose with the tether gate's numbers (the grade's own pairings) when
given `--intent`, and WARNS by name: the member, the number, the limit.

  W. The sketch's bulk cap sits 4.68 mm from the chip against a 3.95 mm limit:
     the run WARNS naming it, the row carries it (`fitted.decap_limits`), the
     JSON_SUMMARY lists it (`decap_over`); the cap at the pin is not named.
  C. Controls: with a limit it meets (6 mm) nothing is named; with no `--intent`
     nothing is measured.

    python3 tests/test_1416_fit_template_decap_limits.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []
SRC = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
SKETCH = {"anchor": "IC2", "kind": "switching stage",
          "source": "a supply-pin cap and the bulk cap beside it",
          "members": [
              {"ref": "C2", "pads": {"C2": ["1", "2"], "IC2": ["14", "7"]}, "side": "+x"},
              {"ref": "C5", "pads": {"C5": ["1", "2"], "C2": ["1", "2"]}}]}


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _fit(td, tag, intent_path, extra=()):
    row = os.path.join(td, f'{tag}_row.json')
    r = check([sys.executable, '-X', 'utf8', tool('fit_template.py'), SRC, '--sketch',
               os.path.join(td, 'sketch.json'), '-o', row] + list(extra), accept=True)
    evidence(row, 'the fitted row')
    summary = next(json.loads(l[len('JSON_SUMMARY: '):]) for l in r.stdout.splitlines()
                   if l.startswith('JSON_SUMMARY: '))
    return r.stdout, json.load(open(row)), summary


def main():
    with tempfile.TemporaryDirectory(prefix='t1416_') as td:
        json.dump(SKETCH, open(os.path.join(td, 'sketch.json'), 'w'))
        base = os.path.join(td, 'intent.json')
        check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), SRC,
               '--emit-intent', base, '--decaps-from', SRC], accept=True)
        evidence(base, 'the emitted intent')
        doc = json.load(open(base))
        ok('the emitted intent carries a 3.95 mm cap-to-chip limit',
           abs(doc['decaps']['max_distance_mm'] - 3.95) < 1e-6, doc.get('decaps'))

        out, row, summary = _fit(td, 'tight', base, ['--intent', base, '--write-intent',
                                                     os.path.join(td, 'tight_out.json')])
        ok('the cap past the limit is WARNED by name, with its number and the limit',
           'WARNING: C5 at its template pose reads 4.68 mm against the intent\'s '
           'decap_distance limit of 3.95 mm' in out, out[-900:])
        ok('... and the cap at the pin is not named', 'WARNING: C2 ' not in out)
        ok('the row and the summary carry it',
           [f['ref'] for f in row['fitted']['decap_limits']] == ['C5']
           and summary['decap_over'] == ['C5'], (row['fitted'].get('decap_limits'), summary))

        loose = dict(doc, decaps=dict(doc['decaps'], max_distance_mm=6.0))
        loose_path = os.path.join(td, 'loose.json')
        json.dump(loose, open(loose_path, 'w'))
        out, row, summary = _fit(td, 'loose', loose_path, ['--intent', loose_path,
                                                           '--write-intent',
                                                           os.path.join(td, 'loose_out.json')])
        ok('a limit the row meets names nobody',
           'decap_distance limit' not in out and summary['decap_over'] == []
           and row['fitted']['decap_limits'] == [], out[-600:])

        out, row, summary = _fit(td, 'none', None)
        ok('with no --intent nothing is measured',
           'decap_distance limit' not in out and 'decap_limits' not in row['fitted'], row['fitted'])
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
