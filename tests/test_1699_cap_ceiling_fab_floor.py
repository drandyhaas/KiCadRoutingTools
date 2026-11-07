#!/usr/bin/env python3
"""#1699: place_fanout_clearance.py --clearance holds the physical fab floor.

The routing mains pin a sub-floor ceiling up (#1691) and this step did not: on a
2-layer board with a wider second class, --clearance 0.0889 wrote every class at
0.0889 into the project the later steps read. It now goes through the same pin and
says the same thing, and the #768/#769 contract stands around it.

  A. the pin: the floor by layer count, the log, what the pass priced and wrote,
     on the path that moves nothing and the one that moves a cap.
  B. what moves it: an overrides file, a value at or above the floor.
  C. what it leaves alone: the omitted branch, and what the board itself declares
     below the floor (a class, a .kicad_dru rule, a pad override).
  D. the refusal, --no-fix-drc-settings, and the flag in a converted plan.

The GUI half is tests/gui_parity/test_1699_cap_ceiling_fab_floor_gui.py.

    python3 tests/test_1699_cap_ceiling_fab_floor.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

CLI = os.path.join(REPO, 'py_placer', 'place_fanout_clearance.py')
FLAT = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
# The tracked board that moves caps at file poses; it carries no project.
MOVER = os.path.join(REPO, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
PINNED = '--clearance 0.0889 is below the physical fab floor 0.1'
failures = []


def check(label, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'} {label}{('  ' + str(detail)) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def stage(td, name, board=FLAT, *, wide=None, default=None, classes=None, four_layers=False,
          dru=None, pad_clearance=None):
    d = os.path.join(td, name)
    os.makedirs(d)
    src = os.path.join(d, 'in.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'copy_board.py'), board, src],
                   capture_output=True, cwd=REPO)
    run_utils.evidence(src)
    if four_layers or pad_clearance is not None:
        with open(src, encoding='utf-8') as f:
            text = f.read()
        if four_layers:
            new = re.sub(r'(\(0 "F\.Cu" signal[^)]*\))',
                         r'\1\n    (1 "In1.Cu" signal)\n    (2 "In2.Cu" signal)', text, count=1)
            if new == text:
                raise AssertionError('BROKEN TEST: no F.Cu layer row to add inner layers beside')
            text = new
        if pad_clearance is not None:
            new = re.sub(r'(\(pad "[^"]+" (?:smd|thru_hole) \w+\s*\(at [^)]*\))',
                         rf'\1 (clearance {pad_clearance})', text, count=1)
            if new == text:
                raise AssertionError('BROKEN TEST: no pad to give an override')
            text = new
        with open(src, 'w', encoding='utf-8') as f:
            f.write(text)
    pro = os.path.splitext(src)[0] + '.kicad_pro'
    if classes is not None:
        with open(pro, 'w', encoding='utf-8') as f:
            json.dump({'board': {'design_settings': {'rules': {}}}, 'meta': {'version': 1},
                       'net_settings': {'meta': {'version': 3}, 'classes': classes}}, f)
    elif wide is not None or default is not None:
        with open(pro, encoding='utf-8') as f:
            doc = json.load(f)
        for c in doc['net_settings']['classes']:
            if c['name'] == 'Wide' and wide is not None:
                c['clearance'] = wide
            if c['name'] == 'Default' and default is not None:
                c['clearance'] = default
        with open(pro, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1)
    if dru is not None:
        with open(os.path.splitext(src)[0] + '.kicad_dru', 'w', encoding='utf-8') as f:
            f.write(dru)
    return src


def run(src, *flags, name='out'):
    out = os.path.join(os.path.dirname(src), f'{name}.kicad_pcb')
    r = run_utils.check([sys.executable, '-X', 'utf8', CLI, src, out, *[str(f) for f in flags]],
                        accept=True, cwd=REPO)
    run_utils.evidence(out)
    return out, r.stdout + r.stderr


def project(board):
    with open(run_utils.evidence(os.path.splitext(board)[0] + '.kicad_pro'), encoding='utf-8') as f:
        doc = json.load(f)
    return {c['name']: c['clearance'] for c in doc['net_settings']['classes']}, doc['board']['design_settings']['rules']


def the_pin(td):
    print('A. the pin')
    out, log = run(stage(td, 'two'), '--clearance', 0.0889)
    classes, rules = project(out)
    check('2 layers: the run says it pinned, as the routing mains do', PINNED in log, log[-400:])
    check('2 layers: it priced the pairs at the floor', 'cap pair clearance: 0.1mm (cli)' in log, log[:400])
    check('2 layers: Default written at the floor', classes.get('Default') == 0.1, classes)
    check('2 layers: Wide written at the floor', classes.get('Wide') == 0.1, classes)
    check('2 layers: the board minimum written at the floor', rules.get('min_clearance') == 0.1, rules)
    check('the path that moves nothing wrote it: this run moved no cap', 'No caps moved' in log, log[-400:])

    out, log = run(stage(td, 'four', four_layers=True), '--clearance', 0.08)
    check('4 layers: its own floor', '--clearance 0.08 is below the physical fab floor 0.09' in log, log[-400:])
    check('4 layers: Wide written at that floor', project(out)[0].get('Wide') == 0.09, project(out)[0])

    classes4 = [{'name': 'Default', 'clearance': 0.2, 'track_width': 0.2, 'via_diameter': 0.6, 'via_drill': 0.4,
                 'priority': 2147483647},
                {'name': 'Wide', 'clearance': 0.4, 'track_width': 0.4, 'via_diameter': 0.8, 'via_drill': 0.4,
                 'priority': 1}]
    out, log = run(stage(td, 'mover', board=MOVER, classes=classes4), '--clearance', 0.05)
    check('precondition: this run moved a cap', 'No caps moved' not in log, log[-400:])
    said = re.search(r'--clearance 0.05 is below the physical fab floor ([0-9.]+)', log)
    check('the path that moves a cap: pinned', said is not None, log[-400:])
    if said is None:
        return
    floor = float(said.group(1))
    check('the path that moves a cap: priced at the floor', f'cap pair clearance: {floor:g}mm (cli)' in log)
    check('the path that moves a cap: written at the floor', project(out)[0].get('Default') == floor, project(out)[0])


def moved_by(td):
    print('B. what moves it')
    fab = os.path.join(td, 'fine.fab')
    with open(fab, 'w', encoding='utf-8') as f:
        f.write('clearance = 0.08\n')
    out, log = run(stage(td, 'overrides'), '--clearance', 0.0889, '--fab-overrides', fab)
    check('an overrides file below the value: it stands', project(out)[0].get('Wide') == 0.0889, project(out)[0])
    check('...priced as asked', 'cap pair clearance: 0.0889mm (cli)' in log)
    check('...and no pin line', 'physical fab floor' not in log, log[-400:])

    out, log = run(stage(td, 'at'), '--clearance', 0.1)
    check('at the floor: no pin line', 'physical fab floor' not in log)
    out, log = run(stage(td, 'above'), '--clearance', 0.3)
    check('above the floor: Wide at the ceiling', project(out)[0].get('Wide') == 0.3, project(out)[0])
    check('...Default as declared', project(out)[0].get('Default') == 0.2, project(out)[0])
    check('...and no pin line', 'physical fab floor' not in log)


def left_alone(td):
    print('C. what it leaves alone')
    out, log = run(stage(td, 'omitted'))
    check('omitted: every class as declared', project(out)[0] == {'Default': 0.2, 'Wide': 0.4}, project(out)[0])
    check('...and no pin line', 'physical fab floor' not in log)

    out, log = run(stage(td, 'wide', wide=0.05), '--clearance', 0.0889)
    check('a class the board declares below the floor stays as declared', project(out)[0].get('Wide') == 0.05,
          project(out)[0])

    out, log = run(stage(td, 'default', default=0.05), '--clearance', 0.0889)
    check('as the Default class does', project(out)[0].get('Default') == 0.05, project(out)[0])
    check('...priced at the declared number', 'cap pair clearance: 0.05mm (cli)' in log, log[:400])
    check('...and disclosed as below the floor', 'below the 2-layer fab' in log, log[:600])

    dru = '(version 1)\n(rule "fine_back" (layer "B.Cu") (constraint clearance (min 0.05mm)))\n'
    src = stage(td, 'dru', dru=dru)
    out, log = run(src, '--clearance', 0.0889)
    with open(run_utils.evidence(os.path.splitext(out)[0] + '.kicad_dru'), encoding='utf-8') as f:
        check('a .kicad_dru rule stays as declared', f.read() == dru)
    check('...and caps the board minimum at it, as it does without a pin',
          project(out)[1].get('min_clearance') == 0.05, project(out)[1].get('min_clearance'))
    check('...while the classes hold the floor', project(out)[0].get('Wide') == 0.1, project(out)[0])

    src = stage(td, 'pad', pad_clearance=0.05)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    from kicad_parser import parse_kicad_pcb
    before = [p.local_clearance for fp in parse_kicad_pcb(src).footprints.values() for p in fp.pads
              if p.local_clearance]
    if before != [0.05]:
        raise AssertionError(f'BROKEN TEST: the staged pad override read back as {before}')
    out, log = run(src, '--clearance', 0.0889)
    after = [p.local_clearance for fp in parse_kicad_pcb(out).footprints.values() for p in fp.pads
             if p.local_clearance]
    check('a pad override stays as declared', after == [0.05], after)
    check('...while the classes hold the floor', project(out)[0].get('Wide') == 0.1, project(out)[0])


def the_rest(td):
    print('D. the refusal, --no-fix-drc-settings, and the flag in a converted plan')
    src = stage(td, 'refuse')
    r = run_utils.check([sys.executable, '-X', 'utf8', CLI, src, os.path.join(td, 'never.kicad_pcb'),
                         '--clearance', '0.1', '--fab-overrides', os.path.join(td, 'absent.fab')],
                        refuse='--fab-overrides file not found', code=1, cwd=REPO)
    check('a missing overrides file refuses, for that reason', r.returncode == 1)
    check('...writing nothing', not os.path.exists(os.path.join(td, 'never.kicad_pcb')))

    out, log = run(stage(td, 'nofix'), '--clearance', 0.0889, '--no-fix-drc-settings')
    check('--no-fix-drc-settings: the project carried through as it was',
          project(out)[0] == {'Default': 0.2, 'Wide': 0.4}, project(out)[0])
    check('...still pinned, and said so', PINNED in log, log[-400:])

    sys.path.insert(0, os.path.join(TESTS, 'stress'))
    import manifest_to_plan as m2p
    step = m2p.cap_optimization_step(['python3', 'place_fanout_clearance.py', 'a.kicad_pcb', 'b.kicad_pcb',
                                      '--clearance', '0.1', '--fab-overrides', 'fine.fab'], cwd=td)
    check('a recorded --fab-overrides reaches the plan step, read from the command\'s folder',
          step.get('params', {}).get('fab_overrides_path') == os.path.normpath(os.path.join(td, 'fine.fab')),
          step)


if __name__ == '__main__':
    td = tempfile.mkdtemp(prefix='krt1699_')
    try:
        the_pin(td)
        moved_by(td)
        left_alone(td)
        the_rest(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print()
    if failures:
        print(f'FAILED: {len(failures)} check(s)')
        for f_ in failures:
            print(f'  {f_}')
        sys.exit(1)
    print('PASS: #1699 the placement ceiling holds the physical fab floor')
