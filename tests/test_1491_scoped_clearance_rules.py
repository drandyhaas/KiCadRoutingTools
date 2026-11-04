"""#1491: .kicad_dru clearance rules scoped by net class or item type are
honoured by the router and graded by check_drc.

A rule like `A.NetClass == 'CSI' && B.NetClass == 'CSI'` (tightening an
intra-class gap) or `A.NetClass == 'HS' && B.Type == 'Pad'` (relaxing a pad
escape) was skipped by every channel: the router routed at the class value
and check_drc priced the copper there too, while KiCad enforced the rule.
`kicad_dru.ScopedClearanceRules` now resolves them pair-exact through
`design_rules`' condition engine (commutative, last match wins, replacing the
class value); the router prices each obstacle at the max over the routed
set (`ScopedClearanceInstall`), so a relaxing rule lowers a stamp only as far
as every routed net allows, and check_drc grades the pair exactly. Between a
scoped rule and a layer rule (#498) the LATER one in the file wins, on the
router and in check_drc alike (`ScopedClearanceRules.resolve_pair`).

    python3 tests/test_1491_scoped_clearance_rules.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(TESTS, 'oracle'))
import run_utils  # noqa: E402

from constraint_agreement import write_board, pcb_data_for  # noqa: E402

RUN_ALL_TIMEOUT = 900
PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


CSI = "(condition \"A.NetClass == 'CSI' && B.NetClass == 'CSI'\")"
HS_PAD = "(condition \"A.NetClass == 'HS' && B.Type == 'Pad'\")"


def selection(td):
    print('1. which rules the scoped channel takes')
    from kicad_dru import ScopedClearanceRules
    dru = ' '.join([
        f'(rule "csi gap" (constraint clearance (min 0.3mm)) {CSI})',
        f'(rule "hs pad" (constraint clearance (min 0.2mm)) {HS_PAD})',
        '(rule "trk" (constraint clearance (min 0.4mm)) '
        "(condition \"A.NetClass == 'CSI' && A.Type == 'track' && B.Type == 'track'\"))",
        '(rule "lay" (layer "B.Cu") (constraint clearance (min 0.4mm)) '
        "(condition \"A.NetClass == 'CSI'\"))",
        '(rule "onl" (constraint clearance (min 0.4mm)) (condition "A.onLayer(\'F.Cu\')"))',
        '(rule "cpl" (constraint clearance (min 0.1mm)) '
        "(condition \"A.NetClass == 'CSI' && AB.isCoupledDiffPair()\"))",
        '(rule "nm" (constraint clearance (min 0.4mm)) (condition "A.NetName == \'c\'"))'])
    b = os.path.join(td, 'sel.kicad_pcb')
    write_board(b, classes=[{'name': 'CSI', 'clearance': 0.2, 'priority': 0}],
                patterns=[('A', 'CSI'), ('B', 'CSI')], dru=dru)
    sc = ScopedClearanceRules.for_board(pcb_data_for(b), b)
    check('the class-pair, the Type and (#1528) the NetName rules, and no other',
          sc is not None and sorted(sc.names) == ['csi gap', 'hs pad', 'nm'],
          f'{sc and sc.names}')
    got = sc.pair_value(1, 'track', 2, 'track')
    check('A/B commutative: CSI track vs CSI track -> 0.3', got and got[0] == 0.3, f'{got}')
    got = sc.pair_value(1, 'track', 3, 'track')
    check("#1528: NetName binds net C as KiCad compares it (case-insensitive), "
          "commutative -> 0.4", got and got[0] == 0.4 and got[1] == 'nm', f'{got}')


def _config(pcb, routed, ncl):
    from routing_config import GridRouteConfig
    from kicad_dru import install_track_clearances
    cfg = GridRouteConfig()
    cfg.clearance = 0.2
    cfg.set_net_clearances(ncl, routed)
    install_track_clearances(cfg, None, pcb.source_path, pcb, routed_net_ids=routed)
    return cfg


def stamps(td):
    print('2. the router\'s stamps')
    # nets: 1 = A, 2 = B, 3 = C (write_board's)
    b = os.path.join(td, 'tight.kicad_pcb')
    write_board(b, classes=[{'name': 'CSI', 'clearance': 0.2, 'priority': 0}],
                patterns=[('A', 'CSI'), ('B', 'CSI')],
                dru=f'(rule "csi gap" (constraint clearance (min 0.3mm)) {CSI})')
    pcb = pcb_data_for(b)
    cfg = _config(pcb, [1], {})
    check('tightening: routing CSI A, the CSI obstacle B is stamped at 0.3',
          abs(cfg.obstacle_clearance(2) - 0.3) < 1e-9, f'{cfg.obstacle_clearance(2)}')
    check('...and the non-member C keeps the class 0.2',
          abs(cfg.obstacle_clearance(3) - 0.2) < 1e-9, f'{cfg.obstacle_clearance(3)}')
    check('...and a verdict on the pair reads 0.3',
          abs(cfg.pair_clearance(1, 2, kind='track') - 0.3) < 1e-9)
    check('...a hot loop is told the pair prices are not flat, and a prefilter '
          'reaches 0.3', not cfg.pair_clearance_inert()
          and abs(cfg.max_pair_clearance() - 0.3) < 1e-9, f'{cfg.max_pair_clearance()}')

    b = os.path.join(td, 'relax.kicad_pcb')
    write_board(b, classes=[{'name': 'HS', 'clearance': 0.5, 'priority': 0},
                            {'name': 'X', 'clearance': 0.4, 'priority': 1}],
                patterns=[('A', 'HS'), ('C', 'X')], rules={'min_clearance': 0.05},
                dru=f'(rule "hs pad" (constraint clearance (min 0.2mm)) {HS_PAD})')
    pcb = pcb_data_for(b)
    pad = SimpleNamespace(net_id=2, pad_type='smd')
    cfg = _config(pcb, [1], {1: 0.5, 3: 0.4})
    check('relaxing: routing HS alone, a foreign pad is stamped at the rule 0.2',
          abs(cfg.pad_obstacle_clearance(pad) - 0.2) < 1e-9, f'{cfg.pad_obstacle_clearance(pad)}')
    check('...and foreign tracks keep the HS class 0.5 (the rule binds pads only)',
          abs(cfg.obstacle_clearance(2) - 0.5) < 1e-9, f'{cfg.obstacle_clearance(2)}')
    cfg = _config(pcb, [1, 3], {1: 0.5, 3: 0.4})
    from kicad_dru import scoped_clearance_report
    check('with a class-X net (0.4) routed too, the pad keeps 0.4 ...',
          abs(cfg.pad_obstacle_clearance(pad) - 0.4) < 1e-9, f'{cfg.pad_obstacle_clearance(pad)}')
    check('...and the run says the rule was held up',
          [r['rule'] for r in scoped_clearance_report(cfg)] == ['hs pad'],
          json.dumps(scoped_clearance_report(cfg)))


LAYER_F = '(rule "f" (layer "F.Cu") (constraint clearance (min {v}mm)))'


def _ordered_board(td, name, first, second, segments=()):
    b = os.path.join(td, name)
    write_board(b, classes=[{'name': 'CSI', 'clearance': 0.2, 'priority': 0}],
                patterns=[('A', 'CSI'), ('B', 'CSI')], rules={'min_clearance': 0.05},
                dru=f'{first} {second}', segments=segments)
    return b


def _drc(b):
    return subprocess.run(PY + [os.path.join('py_router', 'check_drc.py'), b,
                                '--clearance', '0.1'],
                          capture_output=True, text=True, cwd=REPO)


def file_order(td):
    print('3. a layer rule and a scoped rule: the later one in the file wins')
    from kicad_dru import install_layer_clearances
    csi = f'(rule "csi gap" (constraint clearance (min 0.3mm)) {CSI})'
    for label, first, second, f_cu in (
            ('layer rule, then the scoped rule', LAYER_F.format(v=0.15), csi, 0.3),
            ('scoped rule, then the layer rule', csi, LAYER_F.format(v=0.15), 0.15)):
        b = _ordered_board(td, f'ord{f_cu}.kicad_pcb', first, second)
        pcb = pcb_data_for(b)
        from routing_config import GridRouteConfig
        from kicad_dru import install_track_clearances
        cfg = GridRouteConfig()
        cfg.clearance = 0.2
        cfg.layers = ['F.Cu', 'B.Cu']
        cfg.set_net_clearances({}, [1])
        install_layer_clearances(cfg, None, b, pcb)
        install_track_clearances(cfg, None, b, pcb, routed_net_ids=[1])
        got = cfg.pair_clearance(1, 2, 'F.Cu', kind='track')
        check(f'{label}: the CSI pair on F.Cu is {f_cu}', abs(got - f_cu) < 1e-9, f'{got}')
        got = cfg.layer_clearance('F.Cu', cfg.obstacle_clearance(2), 2)
        check(f'{label}: ...and the router stamps CSI B on F.Cu at {f_cu}',
              abs(got - f_cu) < 1e-9, f'{got}')
        got = cfg.layer_clearance('F.Cu', cfg.obstacle_clearance(3), 3)
        check(f'{label}: ...a non-CSI net there keeps the layer rule 0.15',
              abs(got - 0.15) < 1e-9, f'{got}')
        got = cfg.pair_clearance(1, 2, 'B.Cu', kind='track')
        check(f'{label}: ...and on unruled B.Cu the scoped rule 0.3 holds',
              abs(got - 0.3) < 1e-9, f'{got}')
        segs = [(5, 10, 15, 10, 0.2, 'F.Cu', 1), (5, 10.45, 15, 10.45, 0.2, 'F.Cu', 2)]
        b = _ordered_board(td, f'ordg{f_cu}.kicad_pcb', first, second, segs)
        r = _drc(b)
        flagged = r.returncode != 0
        check(f'{label}: check_drc on CSI tracks 0.25 mm apart on F.Cu '
              f'{"flags" if f_cu > 0.25 else "passes"} them',
              flagged == (f_cu > 0.25), r.stdout[-400:])

    # a relaxing pad rule after a tightening layer rule: the pad stamp relaxes
    b = os.path.join(td, 'ordpad.kicad_pcb')
    write_board(b, classes=[{'name': 'HS', 'clearance': 0.5, 'priority': 0}],
                patterns=[('A', 'HS')], rules={'min_clearance': 0.05},
                dru=f'{LAYER_F.format(v=0.4)} (rule "hs pad" (constraint clearance '
                    f'(min 0.2mm)) {HS_PAD})')
    pcb = pcb_data_for(b)
    from routing_config import GridRouteConfig
    from kicad_dru import install_track_clearances
    cfg = GridRouteConfig()
    cfg.clearance = 0.2
    cfg.layers = ['F.Cu', 'B.Cu']
    cfg.set_net_clearances({1: 0.5}, [1])
    install_layer_clearances(cfg, None, b, pcb)
    install_track_clearances(cfg, None, b, pcb, routed_net_ids=[1])
    pad = SimpleNamespace(net_id=2, pad_type='smd', layers=['F.Cu'])
    got = cfg.layer_clearance('F.Cu', cfg.pad_obstacle_clearance(pad), pad=pad)
    check('layer rule 0.4, then the HS pad rule 0.2: an HS route stamps a pad on '
          'F.Cu at 0.2', abs(got - 0.2) < 1e-9, f'{got}')
    got = cfg.layer_clearance('F.Cu', cfg.obstacle_clearance(2), 2)
    check('...and a track on F.Cu at the layer rule 0.4', abs(got - 0.4) < 1e-9, f'{got}')


def grader(td):
    print('4. check_drc grades the pair exactly')
    b = os.path.join(td, 'g1.kicad_pcb')
    write_board(b, classes=[{'name': 'CSI', 'clearance': 0.2, 'priority': 0}],
                patterns=[('A', 'CSI'), ('B', 'CSI')],
                dru=f'(rule "csi gap" (constraint clearance (min 0.3mm)) {CSI})',
                segments=[(5, 10, 15, 10, 0.2, 'F.Cu', 1), (5, 10.45, 15, 10.45, 0.2, 'F.Cu', 2)])
    r = subprocess.run(PY + [os.path.join('py_router', 'check_drc.py'), b, '--clearance', '0.1'],
                       capture_output=True, text=True, cwd=REPO)
    check('two CSI tracks 0.25 mm apart under the 0.3 mm rule: flagged',
          r.returncode != 0 and 'NOT GRADED' not in r.stdout, r.stdout[-400:])
    b = os.path.join(td, 'g2.kicad_pcb')
    write_board(b, classes=[{'name': 'HS', 'clearance': 0.5, 'priority': 0}],
                patterns=[('B', 'HS')], rules={'min_clearance': 0.05},
                dru=f'(rule "hs pad" (constraint clearance (min 0.2mm)) {HS_PAD})',
                footprints=[{'ref': 'U1', 'x': 10, 'y': 10, 'net_id': 1, 'net_name': 'A'}],
                segments=[(5, 10.9, 15, 10.9, 0.2, 'F.Cu', 2)])
    r = subprocess.run(PY + [os.path.join('py_router', 'check_drc.py'), b, '--clearance', '0.1'],
                       capture_output=True, text=True, cwd=REPO)
    check('an HS track 0.3 mm from a pad (class 0.5, the rule 0.2): clean',
          r.returncode == 0, r.stdout[-400:])


def routed(td):
    print('5. routed: the pair gap and the relaxed pad escape')
    b = os.path.join(td, 'csi.kicad_pcb')
    subprocess.run(PY + [os.path.join('py_router', 'copy_board.py'),
                         os.path.join('kicad_files', 'qfn_csi_underpad_diff.kicad_pcb'), b],
                   capture_output=True, cwd=REPO)
    run_utils.evidence(b)
    pro = os.path.splitext(b)[0] + '.kicad_pro'
    json.dump({'board': {'design_settings': {'rules': {'min_clearance': 0.0}}},
               'net_settings': {'meta': {'version': 4}, 'classes': [
                   {'name': 'Default', 'clearance': 0.2, 'track_width': 0.2,
                    'diff_pair_gap': 0.2, 'diff_pair_width': 0.2, 'priority': 2147483647},
                   {'name': 'CSI', 'clearance': 0.2, 'track_width': 0.2,
                    'diff_pair_gap': 0.2, 'diff_pair_width': 0.2, 'priority': 0}],
                   'netclass_patterns': [{'netclass': 'CSI', 'pattern': 'CSI_*'}]},
               'meta': {'version': 3}}, open(pro, 'w'))
    open(os.path.splitext(b)[0] + '.kicad_dru', 'w').write(
        f'(version 1)\n(rule "csi gap" (constraint clearance (min 0.3mm)) {CSI})\n')
    r = subprocess.run(PY + [os.path.join('py_router', 'route_diff.py'), b,
                             os.path.join(td, 'csi_o.kicad_pcb'), '--nets', 'CSI_*'],
                       capture_output=True, text=True, cwd=REPO)
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith('JSON_SUMMARY:')), None)
    gaps = []
    if line:
        doc = json.loads(line[len('JSON_SUMMARY:'):])
        gaps = [p.get('copper', {}).get('gap_mm') for p in doc.get('pair_reports', [])
                if p.get('copper')]
    check('route_diff builds the CSI pair at the rule\'s 0.3 mm gap', gaps and all(
        g is not None and abs(g - 0.3) < 1e-6 for g in gaps), f'{gaps}')
    c = subprocess.run(PY + [os.path.join('py_router', 'check_drc.py'),
                             os.path.join(td, 'csi_o.kicad_pcb'), '--clearance', '0.1'],
                       capture_output=True, text=True, cwd=REPO)
    tracks = [ln for ln in c.stdout.splitlines() if 'SEGMENT-SEGMENT' in ln.upper()]
    check('...and check_drc finds no track pair under the rule', not tracks, '\n'.join(tracks))


def main():
    td = tempfile.mkdtemp(prefix='krt1491_')
    try:
        for t in (selection, stamps, file_order, grader, routed):
            t(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
