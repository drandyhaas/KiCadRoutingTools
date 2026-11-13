#!/usr/bin/env python3
"""#1846: check_drc grades a `.kicad_dru` length / skew bound written in ps
against the delays of the project's tuning profiles.

Such a rule was NOT GRADED ("a time-domain bound (ps/ns) is not modelled"),
and nothing read the `.kicad_pro` tuning profiles KiCad 10 times a net with.

The fixture (tests/fixtures/1846) is synthetic: net class `TIMED` (nets `P`,
20 mm, and `Q`, 21 mm, on F.Cu pad to pad) with tuning profile `PROF6`
(F.Cu 6.0 ps/mm, via 6.0 ps/mm, time-domain tuning on) and the rule
`length (min 100ps) (max 140ps)` + `skew (max 5ps)` on the class. Its
stackup alone gives 5.53 ps/mm, so the two sources can be told apart.

Every row of `ROWS` is one variant with the delays kicad-cli 10.0.8 reports
for P and Q (measured, read off a `length (min 1000ps)` rule's rows):
  * a layer entry's delay is attoseconds per mm;
  * a via adds the via delay times its barrel length (1.54 mm here);
  * a track on a layer the profile has no entry for adds nothing;
  * a class that names no profile measures 0 ps, a pad-to-die delay
    included;
  * a profile with time-domain tuning off gives no track or via delay, but a
    pad's pad-to-die delay still counts.
The two pad-to-die rows (a 10 ps `die_delay` on one of Q's pads) were
measured on kicad-cli 10.0.6, which also gives every other row's values.

Checks (no routing):
  1. the fixture: one violation, P 6.00 ps earlier than Q against 5 ps, the
     length window met, and no NOT GRADED line;
  2. each row's delays are KiCad's, through check_drc's own length rows;
  3. the readers: a time bound is a 'ps' rule only for a reader that asks,
     `DelayModel` says why a net measures 0 ps, and a project that cannot
     be parsed gives no profiles with a warning rather than silently.
"""
import contextlib
import copy
import io
import json
import os
import re
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_dru import parse_dru_length_rules  # noqa: E402
from tuning_profiles import DelayModel, read_tuning_profiles  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(TESTS, 'fixtures', '1846', 'time_rule')
FLOOR = ('(version 1)\n(rule "PQ timing"\n  (constraint length (min 1000ps))\n'
         '  (condition "A.hasNetclass(\'TIMED\')"))\n')
BCU = {"signal_layer": "B.Cu", "top_reference_layer": "In2.Cu",
       "bottom_reference_layer": "", "delay": 7000000, "diff_pair_gap": 0,
       "width": 100000}


def _seg(x1, x2, layer):
    return (f'\t(segment\n\t\t(start {x1} 6)\n\t\t(end {x2} 6)\n\t\t(width 0.1)\n'
            f'\t\t(layer "{layer}")\n\t\t(net "Q")\n\t)\n')


def _via(x):
    return (f'\t(via\n\t\t(at {x} 6)\n\t\t(size 0.46)\n\t\t(drill 0.2)\n'
            f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "Q")\n\t)\n')


#: Q as 7 mm on F.Cu, a via, 7 mm on B.Cu, a via, 7 mm on F.Cu
VIAS = _seg(3, 10, 'F.Cu') + _via(10) + _seg(10, 17, 'B.Cu') + _via(17) + _seg(17, 24, 'F.Cu')


def _profile(proj):
    return proj['tuning_profiles']['tuning_profiles_impedance_geometric'][0]


def _bcu(proj):
    _profile(proj)['layer_entries'].append(dict(BCU))


def _bcu_slow_via(proj):
    _bcu(proj)
    _profile(proj)['via_prop_delay'] = 20000000


def _faster(proj):
    _profile(proj)['layer_entries'][0]['delay'] = 7500000


def _no_profile(proj):
    for c in proj['net_settings']['classes']:
        c.pop('tuning_profile', None)


def _off(proj):
    _profile(proj)['enable_time_domain_tuning'] = False


#: name -> (Q rerouted through vias, project edit, KiCad's (P ps, Q ps),
#: a 10 ps pad-to-die delay on one of Q's pads)
ROWS = {
    'as drawn': (False, None, (120.00, 126.00), False),
    'F.Cu at 7.5 ps/mm': (False, _faster, (150.00, 157.50), False),
    'Q through two vias, B.Cu at 7 ps/mm': (True, _bcu, (120.00, 151.48), False),
    'and the via delay at 20 ps/mm': (True, _bcu_slow_via, (120.00, 194.60), False),
    'Q through two vias, no B.Cu entry': (True, None, (120.00, 102.48), False),
    'the class names no profile': (False, _no_profile, (0.00, 0.00), False),
    'time-domain tuning off': (False, _off, (0.00, 0.00), False),
    'a 10 ps pad-to-die delay on Q': (False, None, (120.00, 136.00), True),
    'and time-domain tuning off: the pad-to-die delay alone': (
        False, _off, (0.00, 10.00), True),
    'and the class names no profile: 0 ps': (False, _no_profile, (0.00, 0.00), True),
}

#: one of Q's pads, as the fixture writes it
Q_PAD = '(layers "F.Cu" "F.Mask")\n\t\t\t(net "Q")'

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru=None, vias=False, edit=None, die=False):
    with open(evidence(BOARD + '.kicad_pcb'), encoding='utf-8') as fh:
        t = fh.read()
    if die:
        assert t.count(Q_PAD) == 2, "the fixture has Q's two pads"
        t = t.replace(Q_PAD, '(die_delay 10)\n\t\t\t' + Q_PAD, 1)
    if vias:
        t, n = re.subn(r'\t\(segment\s*\(start 3 6\).*?\n\t\)\n', lambda m: VIAS, t,
                       flags=re.S)
        assert n == 1, "the fixture has Q's one track"
    b = os.path.join(td, name + '.kicad_pcb')
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(t)
    with open(evidence(BOARD + '.kicad_pro'), encoding='utf-8') as fh:
        proj = json.load(fh)
    proj = copy.deepcopy(proj)
    if edit:
        edit(proj)
    with open(os.path.join(td, name + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(proj, fh, indent=2)
    if dru is None:
        with open(evidence(BOARD + '.kicad_dru'), encoding='utf-8') as fh:
            dru = fh.read()
    with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(dru)
    return evidence(b, 'the staged board')


def grade(path):
    out = path[:-len('.kicad_pcb')] + '.drc.json'
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), path,
                             '--json', out],
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    text = (r.stdout or '') + (r.stderr or '')
    if 'Traceback' in text or r.returncode not in (0, 1):
        raise AssertionError(f"BROKEN TEST: check_drc died\n{text[-1500:]}")
    with open(evidence(out, 'the check_drc report'), encoding='utf-8') as fh:
        return json.load(fh), text


def fixture(td):
    print("1. the issue's board")
    doc, text = grade(stage(td, 'f'))
    rows = [v for v in doc['items'] if v['type'] in ('skew-out-of-range',
                                                     'length-out-of-range')]
    ok(len(rows) == 1 and doc['violations'] == 1
       and rows[0]['type'] == 'skew-out-of-range'
       and (rows[0]['net1'], rows[0]['net2']) == ('P', 'Q')
       and (rows[0].get('skew_ps'), rows[0].get('delay_ps'), rows[0].get('target_ps'),
            rows[0].get('limit_ps')) == (6.0, 120.0, 126.0, 5.0),
       f"one violation, P 6.00 ps earlier than Q (120 vs 126) against 5: {rows}")
    ok('NOT GRADED' not in text and not doc.get('not_graded'), "no rule is NOT GRADED")
    ok('P: 6.00 ps earlier than Q' in text and "tuning profiles" in text,
       "the listing is in ps and the run names the tuning profiles")


def rows(td):
    print("2. each net's delay is KiCad's")
    for i, (name, (vias, edit, want, die)) in enumerate(ROWS.items()):
        doc, text = grade(stage(td, f'r{i}', FLOOR, vias, edit, die))
        got = {v['net1']: v.get('delay_ps') for v in doc['items']
               if v['type'] == 'length-out-of-range'}
        ok((got.get('P'), got.get('Q')) == want,
           f"{name}: P {got.get('P')} ps, Q {got.get('Q')} ps"
           + ('' if (got.get('P'), got.get('Q')) == want else f" want {want}"))
        if edit in (_no_profile, _off):
            ok('get no track or via delay, as in KiCad' in text,
               "   and the run says why they get no track or via delay")


def readers(td):
    print("3. the readers")
    with open(BOARD + '.kicad_dru', encoding='utf-8') as fh:
        text = fh.read()
    plain, _ = parse_dru_length_rules(text, kinds=('length', 'skew'))
    timed, _ = parse_dru_length_rules(text, kinds=('length', 'skew'), time_domain=True)
    ok(all(r.terms is None and 'time-domain' in r.unsupported for r in plain),
       "a reader that does not ask keeps the rule unsupported")
    ok([(r.kind, r.unit, r.min_mm, r.max_mm) for r in timed]
       == [('length', 'ps', 100.0, 140.0), ('skew', 'ps', None, 5.0)],
       f"a reader that asks gets ps bounds: {[(r.kind, r.unit, r.min_mm, r.max_mm) for r in timed]}")
    mixed, _ = parse_dru_length_rules(
        '(version 1)\n(rule "m"\n  (constraint length (min 10mm) (max 140ps)))\n',
        time_domain=True)
    ok(mixed[0].terms is None and 'mix' in mixed[0].unsupported,
       "bounds that mix time and length stay unsupported")
    profiles, classes = read_tuning_profiles(BOARD + '.kicad_pcb')
    ok(profiles['PROF6'].layer_ps_per_mm == {'F.Cu': 6.0}
       and profiles['PROF6'].via_ps_per_mm == 6.0 and profiles['PROF6'].time_domain
       and [(c, p) for _prio, c, p in classes] == [('TIMED', 'PROF6')],
       "the profile and the class that names it are read from the project")
    path = stage(td, 'm', FLOOR, False, _no_profile)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    nid = next(i for i, n in pcb.nets.items() if n.name == 'P')
    ps, why = DelayModel(path, pcb).net_delay(nid)
    ok(ps == 0.0 and 'no tuning profile' in why, f"no profile: 0 ps, {why!r}")
    bad = os.path.join(td, 'bad.kicad_pcb')
    with open(bad, 'w', encoding='utf-8') as fh:
        fh.write('')
    with open(os.path.join(td, 'bad.kicad_pro'), 'w', encoding='utf-8') as fh:
        fh.write('{"tuning_profiles": ')
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        got = read_tuning_profiles(bad)
    ok(got == ({}, []) and 'could not read' in err.getvalue()
       and 'tuning profiles are not read' in err.getvalue(),
       f"a project that cannot be parsed gives no profiles and says so: {err.getvalue()!r}")
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        got = read_tuning_profiles(os.path.join(td, 'none.kicad_pcb'))
    ok(got == ({}, []) and not err.getvalue(), "no project: no profiles, no warning")


def main():
    with tempfile.TemporaryDirectory() as td:
        fixture(td)
        rows(td)
        readers(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
