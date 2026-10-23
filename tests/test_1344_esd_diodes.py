#!/usr/bin/env python3
"""#1344: a discrete ESD / TVS diode at its connector is an emc_roles role.

`emc_roles._esd_arrays` reads an ESD ARRAY by its I/O pin functions; a
two-terminal diode has none, so StickHub's sixteen D_0402 TVS diodes (USB
D+/D- to GND at eight ports) were no role, no `proximity` row was proposed,
and nothing placed or graded them at their connector. `_esd_diodes` now
reports a part the role table calls a diode (not an LED footprint), two
copper pads on two nets, one ground, whose other net reaches exactly one
connector, with a ready row (`max_mm` null, report only, #902).

Cases:
* glasgow_revC: D11 -> J4, D12/D13 -> J10, D16 -> J3, D17 -> J2, and no
  other diode; each row is the shape `rule_proximity` grades, passes at its
  worst leg and fails below it;
* refusals on edited copies of D11: an LED footprint, a line on a second
  connector, no ground pad -- each drops it, and nothing else changes.

    python3 -X utf8 tests/test_1344_esd_diodes.py [case ...]
"""
import contextlib
import copy
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 600

GLASGOW = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
WANT = {'D11': 'J4', 'D12': 'J10', 'D13': 'J10', 'D16': 'J3', 'D17': 'J2'}
_CACHE = {}


def _parse(path):
    if path not in _CACHE:
        from kicad_parser import parse_kicad_pcb
        run_utils.evidence(path, 'tracked board')
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            _CACHE[path] = parse_kicad_pcb(path)
    return _CACHE[path]


def _diodes(pcb):
    from placement import emc_roles as E
    out = []
    E._esd_diodes(E._Board(pcb), out)
    return out


def test_glasgow_reports_its_esd_diodes():
    from placement import floorplan as fp
    from placement.emc_roles import emc_roles
    pcb = _parse(GLASGOW)
    sec = emc_roles(pcb)
    got = {r['part']: r for r in sec['roles'] if r['role'] == 'esd diode'}
    assert {p: r['connector'] for p, r in got.items()} == WANT, \
        {p: r['connector'] for p, r in got.items()}
    assert sec['counts'].get('esd diode') == len(WANT), sec['counts']
    rows = [pr for r in got.values() for pr in r['proximity']]
    for pr in rows:
        assert pr['row']['max_mm'] is None, pr
        assert len(pr['row']['pads'][pr['row']['ref']]) == 1, pr

    def grade(slack):
        claims = [dict(pr['row'], max_mm=round(pr['worst_leg_mm'] + slack, 3))
                  for pr in rows]
        intent = fp.intent_from_dict(
            {'schema': 1, 'kind': fp.KIND, 'units': 'mm',
             'proximity': claims}, GLASGOW)
        with contextlib.redirect_stdout(io.StringIO()):
            return fp.grade(intent, pcb, GLASGOW)
    ok = grade(+0.002)
    assert not [v for v in ok.violations if v.rule.startswith('proximity')], \
        [v.message for v in ok.violations if v.rule.startswith('proximity')]
    bad = grade(-0.01)
    failed = {v.ref for v in bad.violations if v.rule == 'proximity'}
    assert failed == set(WANT), failed
    print(f"  PASS: glasgow {', '.join(f'{d}->{j}' for d, j in WANT.items())}; "
          f"{len(rows)} rows pass at their worst leg and fail below it")


def test_each_guard_refuses():
    pcb0 = _parse(GLASGOW)
    d11 = pcb0.footprints['D11']
    line = next(p.net_id for p in d11.pads
                if p.net_id > 0 and 'GND' not in pcb0.nets[p.net_id].name)

    def edited(mutate):
        pcb = copy.deepcopy(pcb0)
        mutate(pcb, pcb.footprints['D11'])
        return {r['part'] for r in _diodes(pcb)}

    def led(pcb, d):
        d.footprint_name = 'LED_SMD:LED_0603_1608Metric'

    def second_connector(pcb, d):
        j = pcb.footprints['J10']
        next(p for p in j.pads if p.net_id > 0).net_id = line

    def no_ground(pcb, d):
        g = next(p for p in d.pads if p.net_id != line)
        g.net_id = next(n for n in sorted(pcb.nets) if n > 0 and n != line
                        and 'GND' not in pcb.nets[n].name)

    for name, fn in (('led', led), ('second_connector', second_connector),
                     ('no_ground', no_ground)):
        got = edited(fn)
        assert 'D11' not in got, (name, got)
        assert got >= set(WANT) - {'D11', 'D12'}, (name, got)
    print("  PASS: an LED footprint, a line on two connectors and a diode "
          "with no ground pad are no ESD diode")


CASES = [test_glasgow_reports_its_esd_diodes, test_each_guard_refuses]

if __name__ == '__main__':
    want = set(sys.argv[1:])
    for t in CASES:
        if not want or t.__name__ in want:
            print(t.__name__)
            t()
    print('ALL PASS')
