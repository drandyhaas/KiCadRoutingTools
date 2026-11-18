"""Bounded refinement through the file-to-file command."""

import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'py_placer' / 'place_refine.py'
sys.path.insert(0, str(ROOT / 'py_router'))

from kicad_parser import parse_kicad_pcb


def footprint(ref, x, y, locked=False):
    return (
        f'(footprint "test:R_0603" (layer "F.Cu") (at {x} {y})'
        + (' (locked yes)' if locked else '')
        + f' (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))'
        + ' (fp_rect (start -1 -1) (end 1 1) (stroke (width 0.05)'
        ' (type default)) (layer "F.CrtYd"))'
        ' (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu"))'
        ' (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu")))\n'
    )


def board(path, overlap=True):
    path.write_text(
        '(kicad_pcb (version 20240108) (generator pcbnew)\n'
        '(layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))'
        '(net 0 "")'
        '(gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
        ' (type default)) (layer "Edge.Cuts"))\n'
        + footprint('A', 15, 15)
        + footprint('B', 15 if overlap else 23, 15)
        + footprint('HOLD', 30, 25)
        + footprint('LOCK', 5, 5, locked=True)
        + ')', encoding='utf-8',
    )
    return path


def run(source, output, report, *args):
    return subprocess.run(
        [sys.executable, '-B', '-X', 'utf8', str(SCRIPT), str(source), str(output),
         '--refs', 'A', 'B', 'LOCK', '--radius', '4', '--moves', '400', '--seed', '2',
         '--json', str(report), *args],
        cwd=ROOT, capture_output=True, text=True, encoding='utf-8', timeout=120,
    )


def poses(path):
    pcb = parse_kicad_pcb(str(path))
    return {ref: (fp.x, fp.y, fp.rotation or 0.0, fp.layer)
            for ref, fp in pcb.footprints.items()}


def test_refinement_moves_only_selected_unlocked_parts_within_radius(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    project = source.with_suffix('.kicad_pro')
    rules = source.with_suffix('.kicad_dru')
    project.write_text('{}', encoding='utf-8')
    rules.write_text('(version 1)', encoding='utf-8')
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    result = run(source, output, report)
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads(report.read_text())
    assert data['improved']
    assert data['before']['illegal_refs']
    assert data['after']['illegal_refs'] == []
    assert data['held_refs'] == ['LOCK']
    before, after = poses(source), poses(output)
    assert after['LOCK'] == before['LOCK']
    assert after['HOLD'] == before['HOLD']
    assert any(after[ref] != before[ref] for ref in ('A', 'B'))
    assert all(math.dist(after[ref][:2], before[ref][:2]) <= 4 + 1e-6 for ref in ('A', 'B'))
    assert all(after[ref][3] == before[ref][3] for ref in after)
    assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
    assert output.with_suffix('.kicad_pro').read_bytes() == project.read_bytes()
    assert output.with_suffix('.kicad_dru').read_bytes() == rules.read_bytes()
    repeated = tmp_path / 'repeated.kicad_pcb'
    again = run(source, repeated, tmp_path / 'repeated.json')
    assert again.returncode == 0, again.stdout + again.stderr
    assert repeated.read_bytes() == output.read_bytes()


def test_no_improvement_copies_the_input_exactly(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb', overlap=False)
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    result = run(source, output, report)
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads(report.read_text())
    assert not data['improved']
    assert data['changes'] == []
    assert output.read_bytes() == source.read_bytes()


def test_invalid_request_does_not_write_an_output(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    original = source.read_bytes()
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    for args, reason in [
        (['--refs', 'UNKNOWN'], 'unknown footprint keys'),
        (['--refs', 'A', 'A'], 'unique footprint keys'),
        (['--radius', 'nan'], 'finite and positive'),
        (['--radius', '-1'], 'finite and positive'),
        (['--moves', '0'], 'moves must be positive'),
    ]:
        result = run(source, output, report, *args)
        assert result.returncode == 2, result.stdout + result.stderr
        assert reason in result.stderr
        assert 'Traceback' not in result.stderr
        assert not output.exists()
        assert not report.exists()
        assert source.read_bytes() == original


def test_routed_input_is_refused(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    routed = source.read_text(encoding='utf-8')[:-1] + (
        '(segment (start 20 20) (end 22 20) (width 0.25) (layer "F.Cu") (net 0)))'
    )
    source.write_text(routed, encoding='utf-8')
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    result = run(source, output, report)
    assert result.returncode == 2
    assert 'refine an unrouted board' in result.stderr
    assert not output.exists()
    assert not report.exists()
    assert source.read_text(encoding='utf-8') == routed


def test_footprint_graphic_copper_is_not_routing(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    source.write_text(source.read_text(encoding='utf-8').replace(
        footprint('HOLD', 30, 25),
        footprint('HOLD', 30, 25)[:-2] + ' (fp_line (start -0.5 0.6) (end 0.5 0.6)'
        ' (stroke (width 0.2) (type solid)) (layer "F.Cu")))\n'), encoding='utf-8')
    assert parse_kicad_pcb(str(source)).segments, 'the fixture lost its graphic copper'
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    result = run(source, output, report)
    assert result.returncode == 0, result.stdout + result.stderr
    assert json.loads(report.read_text())['improved']


def test_a_part_the_writer_cannot_move_is_refused_before_the_search(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    source.write_text(source.read_text(encoding='utf-8').replace(
        footprint('B', 15, 15),
        footprint('B', 15, 15)[:-2] + ' (zone (net 0) (net_name "") (layer "F.Cu")'
        ' (hatch edge 0.5) (polygon (pts (xy 14 14) (xy 16 14) (xy 16 16)))'
        ' (fill_segments (pts (xy 14 14) (xy 16 14)))))\n'), encoding='utf-8')
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    result = run(source, output, report)
    assert result.returncode == 2, result.stdout + result.stderr
    assert 'the board writer cannot move B' in result.stderr, result.stderr
    assert 'Traceback' not in result.stderr
    assert not output.exists()
    assert not report.exists()


def test_output_and_report_cannot_overwrite_inputs(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    original = source.read_bytes()
    project = source.with_suffix('.kicad_pro')
    project.write_text('{}', encoding='utf-8')
    result = run(source, source, tmp_path / 'refined.json')
    assert result.returncode == 2
    assert 'output board must differ from the input' in result.stderr
    result = run(source, tmp_path / 'refined.kicad_pcb', source)
    assert result.returncode == 2
    assert 'JSON report must differ' in result.stderr
    result = run(source, tmp_path / 'refined.kicad_pcb', project)
    assert result.returncode == 2
    assert 'JSON report must differ' in result.stderr
    output = tmp_path / 'refined.kicad_pcb'
    result = run(source, output, output.with_suffix('.kicad_pro'))
    assert result.returncode == 2
    assert 'JSON report must differ' in result.stderr
    assert not output.exists()
    assert project.read_text() == '{}'
    assert source.read_bytes() == original


def test_impossible_radius_keeps_original_board_and_reports_illegal_selection(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    result = run(source, output, report, '--radius', '0.01')
    assert result.returncode == 4, result.stdout + result.stderr
    data = json.loads(report.read_text())
    assert not data['improved']
    assert data['before'] == data['after']
    assert data['after']['illegal_refs']
    assert output.read_bytes() == source.read_bytes()


def test_project_copy_cannot_overwrite_the_supplied_intent(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    source.with_suffix('.design-brief.json').write_text('{}', encoding='utf-8')
    intent = tmp_path / 'refined.design-brief.json'
    original = json.dumps({'schema': 1, 'kind': 'floorplan-intent'})
    intent.write_text(original, encoding='utf-8')
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    result = run(source, output, report, '--intent', str(intent))
    assert result.returncode == 2
    assert 'project/rule destinations must differ from input files' in result.stderr
    assert not output.exists()
    assert not report.exists()
    assert intent.read_text() == original


def test_intent_fixed_footprint_is_held_during_refinement(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb')
    intent = tmp_path / 'intent.json'
    intent.write_text(json.dumps({'schema': 1, 'kind': 'floorplan-intent', 'fixed_poses': [{
        'ref': 'A', 'x': 15, 'y': 15, 'rot': 0, 'side': 'F',
        'basis': 'declared', 'why': 'the connector remains at its input pose',
    }]}), encoding='utf-8')
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    result = run(source, output, report, '--intent', str(intent))
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads(report.read_text())
    assert data['held_refs'] == ['A', 'LOCK']
    assert poses(output)['A'] == poses(source)['A']
    assert data['after']['illegal_refs'] == []


def test_native_edge_seating_cannot_escape_the_input_radius(tmp_path):
    source = board(tmp_path / 'input.kicad_pcb', overlap=False)
    source.write_text(source.read_text().replace(
        footprint('B', 23, 15),
        footprint('TP1', 23, 15).replace('test:R_0603', 'TestPoint:TestPoint_Loop'),
    ), encoding='utf-8')
    intent = tmp_path / 'intent.json'
    intent.write_text(json.dumps({'schema': 1, 'kind': 'floorplan-intent', 'edge_connectors': [{
        'ref': 'TP1', 'edge': 'north', 'side': 'F',
        'why': 'the connector belongs on the north edge',
    }]}), encoding='utf-8')
    output, report = tmp_path / 'refined.kicad_pcb', tmp_path / 'refined.json'
    result = run(source, output, report, '--intent', str(intent), '--radius', '0.01',
                 '--refs', 'A', 'TP1', 'LOCK')
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads(report.read_text())
    assert not data['improved']
    assert 'TP1' not in data['before']['illegal_refs']
    assert output.read_bytes() == source.read_bytes()


if __name__ == '__main__':
    for name, test in list(globals().items()):
        if name.startswith('test_'):
            with tempfile.TemporaryDirectory() as directory:
                test(Path(directory))
            print(f'PASS {name}', flush=True)
