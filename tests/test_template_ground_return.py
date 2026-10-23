#!/usr/bin/env python3
"""A switching stage's template has each member cap's ground return graded.

A template holds its members where it was fitted, so a member exactly at
its pose passes the drift grade even when the template itself is wrong. On
fusb302_pd_dev the LMR36006's output caps, fitted with only their VOUT pad
named, sat past the inductor with their ground 10.5 mm from the chip's,
where the datasheet's Figure 57 joins them to the input caps' ground beside
the PGND pins. `templates.ground_returns` measures each member cap's ground
return -- the least sum of pad-edge gaps from its ground pad to the chip's
ground pins or a member transistor's, through other members' pads on that
ground -- and `check_floorplan` warns past `ground_return_mm` (3 by
default) on a row whose `kind` is `switching stage`.

Checks:
  1. on a stub board, the return chains through other members' ground
     pads, ends at a member transistor's ground where that is nearer, and
     leaves out a resistor and a cap with no pad on the chip's ground;
  2. on esp_prog (its regulator U2 declared a switching stage), the caps
     are graded: under the limit nothing fires; past it one
     `template_ground_return` WARN names the worst; `severity` promotes it
     to an error; a row with no `kind`, or an `rf front end`, is not
     measured;
  3. the loader refuses an unknown `kind` and a non-positive
     `ground_return_mm`;
  4. `fit_template.py` prints each cap's return and a NOTE past the limit
     (C3, named by its supply pad alone, seats with its ground away from
     the chip's), copies the sketch's `kind` into the row and stamps
     `min_reader` 14.

    python3 tests/test_template_ground_return.py
"""
import json
import os
import sys
import tempfile
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, HERE)

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import floorplan as fp                          # noqa: E402
from placement import templates                                # noqa: E402
from run_utils import check, evidence, tool                    # noqa: E402

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
GND, VIN, OUT, SIG = 1, 2, 3, 4


def _part(name, value, *nets):
    pads = [SimpleNamespace(pad_number=str(i + 1), net_id=n, pinfunction='', pintype='',
                            pad_type='smd', layers=['F.Cu'], drill=0)
            for i, n in enumerate(nets)]
    return SimpleNamespace(footprint_name=name, value=value, pads=pads, attr='smd')


def test_stub_chain():
    cap = 'Capacitor_SMD:C_0402_1005Metric'
    fps = {'U1': _part('Package_SO:SOIC-8_3.9x4.9mm_P1.27mm', 'TPS1', GND, VIN),
           'C1': _part(cap, '10u', VIN, GND),
           'C2': _part(cap, '22u', OUT, GND),
           'C3': _part(cap, '22u', OUT, GND),
           'C4': _part(cap, '10n', SIG, OUT),
           'R1': _part('Resistor_SMD:R_0402_1005Metric', '10k', SIG, GND),
           'Q1': _part('Package_TO_SOT_SMD:SOT-23', 'BSS138', SIG, OUT, GND)}
    rect = {('U1', GND): (0, 0, 1, 1), ('U1', VIN): (0, 2, 1, 3),
            ('C1', VIN): (1.5, 2, 2.5, 3), ('C1', GND): (1.5, 0, 2.5, 1),      # 0.5 to U1
            ('C2', OUT): (3, 2, 4, 3), ('C2', GND): (3, 0, 4, 1),              # 0.5 + 0.5 via C1
            ('C3', OUT): (20, 2, 21, 3), ('C3', GND): (20, 0, 21, 1),          # 1.0 to Q1
            ('C4', SIG): (5, 5, 6, 6), ('C4', OUT): (7, 5, 8, 6),
            ('R1', SIG): (1.5, -2, 2.5, -1), ('R1', GND): (1.5, -1, 2.5, -0.5),
            ('Q1', SIG): (22, 2, 23, 3), ('Q1', OUT): (22, 4, 23, 5), ('Q1', GND): (22, 0, 23, 1)}
    pcb = SimpleNamespace(footprints=fps, nets={GND: SimpleNamespace(name='GND'),
                                                VIN: SimpleNamespace(name='VIN'),
                                                OUT: SimpleNamespace(name='VOUT'),
                                                SIG: SimpleNamespace(name='Net-(Q1-G)')})
    decl = {'anchor': 'U1', 'members': [{'ref': r} for r in ('C1', 'C2', 'C3', 'C4', 'R1', 'Q1')]}
    rows = {r['ref']: r for r in templates.ground_return_rows(
        pcb, decl, lambda r: [(p.net_id, rect[(r, p.net_id)]) for p in fps[r].pads])}
    assert set(rows) == {'C1', 'C2', 'C3'}, rows
    assert rows['C1']['return_mm'] == 0.5 and rows['C1']['via'] == [] and rows['C1']['to'] == 'U1', rows['C1']
    assert rows['C2']['return_mm'] == 1.0 and rows['C2']['via'] == ['C1'], rows['C2']
    assert rows['C3']['return_mm'] == 1.0 and rows['C3']['to'] == 'Q1', rows['C3']
    print('  PASS: the return chains through C1, ends at the FET Q1 where nearer, '
          'and leaves out R1 and the off-ground C4')


def _intent(row, severity=None):
    raw = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm', 'templates': [row]}
    if severity:
        raw['severity'] = severity
    return fp.intent_from_dict(raw, '')


def _row(**kw):
    row = {'anchor': 'U2', 'members': [{'ref': 'C1', 'at': [0.0, 3.0], 'rot': 0},
                                       {'ref': 'C3', 'at': [2.0, 3.0], 'rot': 0}]}
    row.update(kw)
    return row


def test_grade_on_esp_prog():
    pcb = parse_kicad_pcb(evidence(ESP, 'esp_prog'))
    caps = {c['ref']: c['return_mm'] for c in templates.ground_returns(pcb, _row())}
    assert set(caps) == {'C1', 'C3'} and all(v > 0 for v in caps.values()), caps
    worst = max(caps, key=caps.get)

    def grade(row, severity=None):
        res = fp.grade(_intent(row, severity), pcb, ESP)
        return [v for v in res.violations if v.rule == 'template_ground_return'], res

    hits, res = grade(_row(kind='switching stage', ground_return_mm=caps[worst] + 1.0))
    assert not hits, hits
    g = res.template_measured[0]['ground_return']
    assert {c['ref']: c['return_mm'] for c in g['caps']} == caps and g['over'] == 0, g
    hits, _ = grade(_row(kind='switching stage', ground_return_mm=min(caps.values()) / 2))
    assert len(hits) == 1 and hits[0].severity == fp.WARN and hits[0].ref == worst, hits
    assert hits[0].measured['over'] == 2, hits[0].measured
    hits, _ = grade(_row(kind='switching stage', ground_return_mm=min(caps.values()) / 2),
                    {'template_ground_return': 'error'})
    assert len(hits) == 1 and hits[0].severity == fp.ERROR, hits
    for row in (_row(ground_return_mm=0.001), _row(kind='rf front end', ground_return_mm=0.001)):
        hits, res = grade(row)
        assert not hits and 'ground_return' not in res.template_measured[0], row
    print(f"  PASS: esp_prog U2's caps graded ({', '.join(f'{r} {v:.2f}' for r, v in sorted(caps.items()))} mm): "
          f"none past a wide limit, one WARN naming {worst} past a tight one, promotable; "
          f"no kind and an RF front end not measured")


def test_loader_refusals():
    for bad, why in (({'kind': 'buck'}, 'kind'), ({'ground_return_mm': 0}, 'ground_return_mm')):
        try:
            _intent(_row(**bad))
        except ValueError as ex:
            assert why in str(ex), ex
        else:
            raise AssertionError(f"accepted {bad}")
    print('  PASS: an unknown kind and a zero ground_return_mm are refused at load')


def test_fit_template_prints_and_stamps():
    sketch = {'anchor': 'U2', 'kind': 'switching stage', 'source': 'test',
              'members': [{'ref': 'C1', 'pads': {'C1': ['1', '2'], 'U2': ['2', '1']}},
                          {'ref': 'C3', 'pads': {'C3': ['1'], 'U2': ['3']}, 'side': '+x',
                           'reach': 8}]}
    with tempfile.TemporaryDirectory() as td:
        sk = os.path.join(td, 'sketch.json')
        intent = os.path.join(td, 'intent.json')
        with open(sk, 'w', encoding='utf-8') as fh:
            json.dump(sketch, fh)
        with open(intent, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'}, fh)
        out = os.path.join(td, 'out.json')
        r = check([sys.executable, '-X', 'utf8', tool('fit_template.py'), ESP, '--sketch', sk,
                   '--intent', intent, '--write-intent', out], accept=True)
        doc = json.load(open(evidence(out, 'written intent'), encoding='utf-8'))
    text = r.stdout
    assert 'ground return C1' in text and 'ground return C3' in text, text
    # C3 names only its supply pad's partner, so nothing holds its ground
    # pad by the chip's: the case the check is for
    assert 'NOTE: C3 returns its ground' in text and 'NOTE: C1 returns' not in text, text
    row = doc['templates'][0]
    assert row['kind'] == 'switching stage' and 'ground_return_mm' not in row, row
    gr = row['fitted']['ground_return_mm']
    assert gr['C1'] <= templates.GROUND_RETURN_MM < gr['C3'], gr
    assert doc['min_reader'] == fp.TEMPLATE_KIND_READER == 14, doc['min_reader']
    print(f"  PASS: fit_template prints each return (C1 {gr['C1']:.2f}, C3 {gr['C3']:.2f} mm), "
          f"a NOTE for C3 only, keeps the kind, stamps min_reader 14")


def main():
    test_stub_chain()
    test_grade_on_esp_prog()
    test_loader_refusals()
    test_fit_template_prints_and_stamps()
    print('test_template_ground_return: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
