#!/usr/bin/env python3
"""#1630 follow-up: --impedance-declare-only records the target ohms and
changes no copper; check_impedance grades the copper against them; no later
step recomputes widths from such a record.

The fixture is test_1630_pair_class_one_gap's: one pair on F.Cu over a GND
plane, its class PAIRS at width 0.15 / gap 0.12 (94.3 ohm differential on the
tool's own model).

  A. route_diff with `--impedance 75 --impedance-declare-only` ships the same
     copper as route_diff with no --impedance (every segment's net, layer,
     ends and width; check_drc and check_connected agree), and records
     {ohms 75, differential, pair_gap 0.12, applied false, declare_only true}.
  B. check_impedance grades that record: both nets named off target (94 vs
     75 ohm), marked declare-only, exit code unchanged (disclosure).
  C. route.py the same way: `--impedance 50 --impedance-declare-only` on one
     net ships the plain copper and records a single-ended declare-only spec.
  D. A later step without --impedance on nets carrying a declare-only record
     reapplies nothing (route_diff and route.py), while the same step over
     an ordinary (applied) record does -- the control that shows the test can
     see a reapply.

    python3 tests/test_1630_declare_only_impedance.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check as run_check, evidence             # noqa: E402
from test_1630_pair_class_one_gap import build                 # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def run(tool, tmp, tag, *flags, records=None):
    board = build(os.path.join(tmp, tag), False)
    if records is not None:
        pro = os.path.join(tmp, tag, 'x.kicad_pro')
        with open(pro, encoding='utf-8') as f:
            doc = json.load(f)
        doc['kicad_routing_tools'] = {'net_impedance': records}
        with open(pro, 'w', encoding='utf-8') as f:
            json.dump(doc, f)
    out = os.path.join(tmp, tag, 'o.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', tool),
                   board, out, *flags], accept=True)
    return evidence(out, f'{tag} output'), r.stdout + r.stderr


def copper(path):
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    names = {i: n.name for i, n in pcb.nets.items()}
    q = lambda v: round(v, 4)
    segs = sorted((names.get(s.net_id), s.layer, q(s.width),
                   tuple(sorted(((q(s.start_x), q(s.start_y)), (q(s.end_x), q(s.end_y))))))
                  for s in pcb.segments)
    vias = sorted((names.get(v.net_id), q(v.x), q(v.y), q(v.size), q(v.drill))
                  for v in pcb.vias)
    return segs, vias


def grades(path):
    """(check_drc violations, check_connected exit and disconnected lines)."""
    j = path + '.drc.json'
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                    path, '--json', j], capture_output=True, text=True, cwd=ROOT)
    with open(evidence(j, 'the check_drc JSON'), encoding='utf-8') as f:
        drc = json.load(f)['violations']
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_connected.py'), path],
                       capture_output=True, text=True, cwd=ROOT)
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError(f'BROKEN TEST: check_connected died\n{out[-800:]}')
    return drc, r.returncode, sum('disconnect' in l.lower() for l in out.splitlines())


def record(tmp, tag):
    with open(os.path.join(tmp, tag, 'o.kicad_pro'), encoding='utf-8') as f:
        return (json.load(f).get('kicad_routing_tools') or {}).get('net_impedance') or {}


def main():
    tmp = tempfile.mkdtemp(prefix='t1630d_')
    try:
        diff = ('route_diff.py', '--nets', '/X_*', '--layers', 'F.Cu', 'B.Cu')
        # A
        plain, _ = run(diff[0], tmp, 'plain', *diff[1:])
        decl, log = run(diff[0], tmp, 'decl', *diff[1:], '--impedance', '75',
                        '--impedance-declare-only')
        check('A: the run says it declares without changing geometry',
              'WITHOUT changing their geometry' in log)
        cp, cd = copper(plain), copper(decl)
        check('A: every segment and via is the plain run\'s', cp == cd and cp[0],
              f'{len(cp[0])} vs {len(cd[0])} segments')
        check('A: the pair keeps its class width 0.15',
              {w for _n, _l, w, _e in cd[0] if _n in ('/X_P', '/X_N')} == {0.15})
        gp, gd = grades(plain), grades(decl)
        check('A: check_drc and check_connected agree with the plain run', gp == gd,
              f'{gp} vs {gd}')
        rec = record(tmp, 'decl').get('/X_P') or {}
        check('A: a declare-only record at the built gap',
              rec.get('declare_only') is True and rec.get('applied') is False
              and rec.get('ohms') == 75.0 and rec.get('differential') is True
              and abs((rec.get('pair_gap') or 0) - 0.12) < 1e-9, str(rec))

        # B
        tool = os.path.join(ROOT, 'py_tools', 'check_impedance.py')
        j = os.path.join(tmp, 'decl', 'imp.json')
        r = run_check([sys.executable, '-X', 'utf8', tool, decl, '--json', j], accept=True)
        with open(j, encoding='utf-8') as f:
            d = json.load(f)
        rows = {x['net_name']: x for x in d['nets']}
        check('B: check_impedance grades the declare-only pair and names it off target',
              d['totals']['nets_ohms_off_target'] == ['/X_N', '/X_P'],
              str(d['totals']['nets_ohms_off_target']))
        check('B: at the class geometry\'s ohms, marked declare-only',
              abs((rows['/X_P'].get('shipped_ohms') or 0) - 94.3) < 1.5
              and rows['/X_P'].get('declare_only') is True,
              str({k: rows['/X_P'].get(k) for k in ('shipped_ohms', 'declare_only')}))
        check('B: and says so', 'declare-only' in r.stdout)

        # C
        se = ('route.py', '--nets', '/X_P', '--layers', 'F.Cu', 'B.Cu')
        p2, _ = run(se[0], tmp, 'se_plain', *se[1:])
        d2, log = run(se[0], tmp, 'se_decl', *se[1:], '--impedance', '50',
                      '--impedance-declare-only')
        check('C: route.py says it declares without changing widths',
              'WITHOUT changing their widths' in log)
        check('C: route.py ships the plain copper', copper(p2) == copper(d2))
        rec = record(tmp, 'se_decl').get('/X_P') or {}
        check('C: a single-ended declare-only record',
              rec.get('declare_only') is True and rec.get('differential') is False
              and rec.get('ohms') == 50.0, str(rec))

        # D
        def spec(ohms, gap, only):
            s = {'ohms': ohms, 'differential': True, 'pair_gap': gap,
                 'coplanar_gap': 0.0, 'applied': not only}
            if only:
                s['declare_only'] = True
            return s
        for tag, only in (('later_decl', True), ('later_applied', False)):
            recs = {n: spec(75.0, 0.12, only) for n in ('/X_P', '/X_N')}
            out, log = run(diff[0], tmp, tag, *diff[1:], records=recs)
            reapplied = 'Reapplying stored' in log
            if only:
                check('D: route_diff reapplies nothing from a declare-only record',
                      not reapplied and copper(out) == cp)
            else:
                check('D (control): route_diff reapplies an applied record', reapplied)
            recs = {'/X_P': dict(spec(50.0, 0.0, only), differential=False)}
            out, log = run(se[0], tmp, tag + '_se', *se[1:], records=recs)
            reapplied = 'Reapplying stored' in log
            if only:
                check('D: route.py reapplies nothing from a declare-only record',
                      not reapplied and copper(out) == copper(p2))
            else:
                check('D (control): route.py reapplies an applied record', reapplied)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"FAIL: {len(failures)}: {failures}")
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
