"""Independent #966 public-board baseline reproduction; run with KiCad Python.

Arguments: --fixture-dir directory of matched zero/positive board+project+plan
pairs, --output-dir a NEW evidence directory. Commands use this script's repo.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pcbnew

p = argparse.ArgumentParser()
p.add_argument('--fixture-dir', type=Path, required=True)
p.add_argument('--output-dir', type=Path, required=True)
p.add_argument('--repo', type=Path, required=True)
p.add_argument('--expected-zero-gui', type=float, required=True)
a = p.parse_args()
root = a.repo.resolve()
out = a.output_dir.resolve()
out.mkdir(parents=True, exist_ok=False)
records = []
def run(argv, name):
    result = subprocess.run([str(v) for v in argv], cwd=root, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
    (out / (name + '.log')).write_text(result.stdout, encoding='utf-8')
    records.append({'command': [str(v) for v in argv], 'returncode': result.returncode,
                    'log': name + '.log'})
    (out / 'commands.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
    assert result.returncode == 0, (name, result.stdout[-1500:])
    return result.stdout

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def point(v):
    return [v.x, v.y]

def snapshot(path):
    board = pcbnew.LoadBoard(str(path))
    return {
        'board_sha256': sha(path), 'project_sha256': sha(path.with_suffix('.kicad_pro')),
        'footprints': sorted((f.GetReference(), point(f.GetPosition()), f.GetOrientationDegrees(),
                              f.GetLayer(), f.IsLocked()) for f in board.GetFootprints()),
        'pads': sorted((f.GetReference(), pad.GetNumber(), point(pad.GetPosition()),
                       point(pad.GetSize()), point(pad.GetDrillSize()), pad.GetOrientationDegrees(),
                       str(pad.GetLayerSet().FmtBin()), pad.GetNetname())
                      for f in board.GetFootprints() for pad in f.Pads()),
        'outline': sorted((d.GetShapeStr(), point(d.GetStart()), point(d.GetEnd()), d.GetWidth())
                          for d in board.GetDrawings() if d.GetLayerName() == 'Edge.Cuts'),
        'copper': sorted((t.GetNetname(), t.GetLayer(), point(t.GetStart()), point(t.GetEnd()),
                          t.GetWidth()) for t in board.GetTracks()),
    }

report = {'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
          'kicad': pcbnew.GetBuildVersion(), 'cases': {}}
for case in ('zero', 'positive'):
    for suffix in ('.kicad_pcb', '.kicad_pro'):
        shutil.copyfile(a.fixture_dir / (case + suffix), out / (case + suffix))
    shutil.copyfile(a.fixture_dir / (case + '-plan.json'), out / (case + '-plan.json'))
    source = out / (case + '.kicad_pcb')
    before = snapshot(source)
    case_report = {'input': before}
    report['cases'][case] = case_report
    for front in ('cli', 'gui'):
        name = case + '-' + front
        target = out / (name + '.kicad_pcb')
        cmd = [sys.executable, '-X', 'utf8', root / 'py_router' / ('route.py' if front == 'cli' else 'run_plan.py'), source]
        if front == 'cli':
            cmd += [target, '--nets', 'Net-(D1-A)', '--layers', 'F.Cu', 'B.Cu', '--max-ripup', '0']
        else:
            cmd += [out / (case + '-plan.json'), '-o', target, '--timeout', '180']
        log = run(cmd, name)
        assert target.is_file() and target.stat().st_size > 0
        summary_line = [line for line in log.splitlines() if line.startswith('JSON_SUMMARY_MIN: ')][-1]
        summary = json.loads(summary_line.partition(': ')[2])
        assert summary['failed'] == 0 and summary['routed'] == 1 and not summary['open_single']
        after = snapshot(target)
        for key in ('footprints', 'pads', 'outline'):
            assert before[key] == after[key], (name, key)
        assert len(after['copper']) == 2 and all(t[0] == 'Net-(D1-A)' for t in after['copper'])
        project_before = json.loads(source.with_suffix('.kicad_pro').read_text())
        project_after = json.loads(target.with_suffix('.kicad_pro').read_text())
        assert project_before['net_settings'] == project_after['net_settings'], name
        run([sys.executable, '-X', 'utf8', root / 'py_router/check_connected.py', target, '--nets', 'Net-(D1-A)'], name + '-connected')
        drc = out / (name + '-native-drc.json')
        run([Path(sys.executable).with_name('kicad-cli.exe'), 'pcb', 'drc', '--format', 'json', '-o', drc, target], name + '-native-drc')
        native = json.loads(drc.read_text(encoding='utf-8'))
        target_open = [v for v in native['unconnected_items'] if any('Net-(D1-A)' in str(i) for i in v['items'])]
        assert not target_open, (name, target_open)
        case_report[front] = {'summary': summary, 'output': after, 'net_settings_preserved': True,
                             'native_drc_types': dict(collections.Counter(v['type'] for v in native['violations'])),
                             'native_unconnected_count': len(native['unconnected_items']),
                             'native_target_unconnected_count': len(target_open)}
        print(name, summary['min_clearance_used'], len(after['copper']), 'tracks; target connected', flush=True)
    (out / 'inspection.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
assert report['cases']['zero']['cli']['summary']['min_clearance_used'] == 0.1
assert report['cases']['zero']['gui']['summary']['min_clearance_used'] == a.expected_zero_gui
assert report['cases']['positive']['cli']['summary']['min_clearance_used'] == 0.2
assert report['cases']['positive']['gui']['summary']['min_clearance_used'] == 0.2
print('ZERO GUI router-reported used rule:', report['cases']['zero']['gui']['summary']['min_clearance_used'])
