"""#1492: under --clearance-ceiling each NON-Default net class is written back
at min(class, ceiling) -- the value the router priced it at -- not at the
Default class's routed clearance, which a descent on one Default net dragged
every class down to. A descent on a class's OWN net lowers that class, is
recorded per class (`class_clearance_relaxed_classes`) and named by
check_complete --authored-from.

  1. the project writer: Default 0.2 / Wide 0.4 / Low 0.15, ceiling 0.3 ->
     Wide 0.3, Low 0.15, Default 0.2; no ceiling (a fanout front) -> the
     routed value, as before; a Wide-net descent to 0.25 lowers Wide alone
     and is returned to be recorded.
  2. a bare GND pour on flat_hierarchy (Default 0.2, Wide 0.4) under
     --clearance-ceiling 0.3 writes Wide 0.3 (it wrote 0.2).
  3. nondefault_class_descents maps a ledger row to its net's class.
  4. check_complete --authored-from names a class the record lowered.

    python3 tests/test_1492_class_ceiling_writeback.py
"""
import copy
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, REPO)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

import fix_kicad_drc_settings as fk  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def proj(classes):
    return {'board': {'design_settings': {'rules': {}}},
            'net_settings': {'meta': {'version': 4},
                             'classes': [{'name': n, 'clearance': c} for n, c in classes],
                             'netclass_patterns': [{'netclass': 'Wide', 'pattern': '/V*'}]}}


def cls(p):
    return {c['name']: c['clearance'] for c in p['net_settings']['classes']}


def writer():
    print('1. the project writer')
    base = proj([('Default', 0.2), ('Wide', 0.4), ('Low', 0.15)])
    t = fk.compute_targets(clearance=0.2)
    p = copy.deepcopy(base)
    fk.apply_targets_to_project(p, t, {}, clamp_nondefault_netclasses=True, class_ceiling=0.3)
    check('ceiling 0.3: Wide 0.3, Low 0.15, Default 0.2',
          cls(p) == {'Default': 0.2, 'Wide': 0.3, 'Low': 0.15}, json.dumps(cls(p)))
    p = copy.deepcopy(base)
    fk.apply_targets_to_project(p, fk.compute_targets(clearance=0.18), {},
                                clamp_nondefault_netclasses=True, class_ceiling=0.3)
    check('a Default descent to 0.18 leaves Wide at the ceiling',
          cls(p) == {'Default': 0.18, 'Wide': 0.3, 'Low': 0.15}, json.dumps(cls(p)))
    p = copy.deepcopy(base)
    fk.apply_targets_to_project(p, t, {}, clamp_nondefault_netclasses=True)
    check('no ceiling known (a fanout front): the routed value, as before',
          cls(p)['Wide'] == 0.2, json.dumps(cls(p)))
    p = copy.deepcopy(base)
    low = {}
    fk.apply_targets_to_project(p, t, {}, clamp_nondefault_netclasses=True, class_ceiling=0.3,
                                class_descents={'Wide': (0.25, ['/V1'])}, lowered_classes=low)
    check('a descent on a Wide net to 0.25 lowers Wide alone',
          cls(p) == {'Default': 0.2, 'Wide': 0.25, 'Low': 0.15}, json.dumps(cls(p)))
    check('...and is returned to be recorded, from the ceiling',
          low == {'Wide': (0.3, 0.25, ['/V1'])}, json.dumps(low))
    fk.record_nondefault_class_descents(p, low)
    rec = p['kicad_routing_tools'][fk.CLASS_CLEARANCE_RELAXED_CLASSES_KEY]
    check('the record names the class, its nets and both values',
          rec == {'Wide': {'from': 0.3, 'to': 0.25, 'nets': ['/V1']}}, json.dumps(rec))
    check('the run prints it', any('NET CLASS Wide CLEARANCE LOWERED BY A DESCENT' in ln
                                   for ln in fk.nondefault_class_descent_disclosure(low)))


def pour(td):
    print('2. a bare pour under --clearance-ceiling 0.3 (flat_hierarchy)')
    src = os.path.join(td, 'in.kicad_pcb')
    out = os.path.join(td, 'pour.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'copy_board.py'),
                    os.path.join('kicad_files', 'flat_hierarchy.kicad_pcb'), src],
                   capture_output=True, cwd=REPO)
    run_utils.evidence(src)
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'route_planes.py'),
                    src, out, '--nets', 'GND', '--plane-layers', 'B.Cu',
                    '--clearance-ceiling', '0.3'], capture_output=True, cwd=REPO)
    pro = os.path.splitext(out)[0] + '.kicad_pro'
    run_utils.evidence(pro)
    got = cls(json.load(open(pro)))
    check('Wide written at 0.3 = min(0.4, 0.3), Default kept at 0.2',
          got.get('Wide') == 0.3 and got.get('Default') == 0.2, json.dumps(got))
    return src


def descents(src):
    print('3. a ledger row mapped to its net\'s class')
    import fab_tiers
    fab_tiers._LEDGER['narrowed'].clear()
    fab_tiers.note_narrowing(None, 'clearance', 0.3, 0.25, 'test', net_name='VPP')
    fab_tiers.note_narrowing(None, 'clearance', 0.3, 0.27, 'test', net_name='GND')
    got = fk.nondefault_class_descents(src)
    fab_tiers._LEDGER['narrowed'].clear()
    check('VPP (Wide) -> Wide 0.25; GND (Default) not a non-Default class',
          got == {'Wide': (0.25, ['VPP'])}, json.dumps(got))


def complete(td, src):
    print('4. check_complete --authored-from names the class')
    import check_complete
    board = os.path.join(td, 'out.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'copy_board.py'),
                    src, board], capture_output=True, cwd=REPO)
    pro = os.path.splitext(board)[0] + '.kicad_pro'
    d = json.load(open(pro))
    for c in d['net_settings']['classes']:
        if c['name'] == 'Wide':
            c['clearance'] = 0.25
    d.setdefault('kicad_routing_tools', {})[fk.CLASS_CLEARANCE_RELAXED_CLASSES_KEY] = {
        'Wide': {'from': 0.3, 'to': 0.25, 'nets': ['/VPP']}}
    json.dump(d, open(pro, 'w'))
    r = check_complete.fab_floor_integrity(board, src, hole_grader=lambda b, mm: {'count': 0})
    rows = [x for x in r.get('relaxed') or () if x.get('key') == 'net_class.Wide.clearance']
    check('a Wide row, authored 0.4 -> 0.25 on the board, naming /VPP',
          len(rows) == 1 and rows[0]['authored'] == 0.4 and rows[0]['on_board'] == 0.25
          and '/VPP' in rows[0]['label'], json.dumps(r.get('relaxed')))


def main():
    writer()
    td = tempfile.mkdtemp(prefix='krt1492_')
    try:
        src = pour(td)
        descents(src)
        complete(td, src)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
