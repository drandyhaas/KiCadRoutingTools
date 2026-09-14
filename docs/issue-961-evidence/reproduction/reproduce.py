"""Independent #961 reviewer: native fixture readback, real API/CLI/DRC matrix.

Run with KiCad Python: reproduce.py --root REPO --out OUTPUT.
Outputs are new copies; original board and any siblings are never modified.
"""
import argparse
import copy
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument('--root', type=Path, required=True)
ap.add_argument('--out', type=Path, required=True)
a = ap.parse_args()
root, out = a.root.resolve(), a.out.resolve()
out.mkdir(parents=True, exist_ok=True)
sys.path[:0] = [str(root / x) for x in ('py_router', 'py_placer')]
import pcbnew
from copy_board import copy_board, SIBLING_EXTS
from kicad_parser import parse_kicad_pcb
from placement import floorplan as fl
from placement.body import board_bodies
from placement.legality import BoardOutlineGate, grade_pad_legality
from placement.quench import QuenchState
from placement.writer import write_placed_output

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def dump(p, value):
    p.write_text(json.dumps(value, indent=2, default=str) + '\n', encoding='utf8')

commands = []
def run(args, log):
    argv = [sys.executable, '-X', 'utf8', *map(str, args)]
    r = subprocess.run(argv, cwd=root, text=True, encoding='utf8', capture_output=True)
    log.write_text(r.stdout + r.stderr, encoding='utf8')
    commands.append({'argv': argv, 'cwd': str(root), 'exit': r.returncode,
                     'log': str(log.relative_to(out))})
    assert 'Traceback' not in r.stdout + r.stderr, (argv, r.stderr)
    return r.returncode

def xy(p):
    return [pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)]

def snapshot(p):
    b = pcbnew.LoadBoard(str(p))
    return {f.GetReference(): {'uuid': str(f.m_Uuid.AsString()),
        'xy': xy(f.GetPosition()), 'rot': f.GetOrientationDegrees(),
        'layer': f.GetLayerName(), 'locked': f.IsLocked(),
        'pads': sorted([(x.GetNumber(), xy(x.GetPosition()), xy(x.GetSize()),
                         x.GetShape(), x.GetNetname()) for x in f.Pads()], key=str)}
        for f in b.GetFootprints()}

source = root / 'kicad_files/esp_prog.kicad_pcb'
source_identity = {str(source.name): sha(source)}
for ext in SIBLING_EXTS:
    p = source.with_suffix(ext)
    source_identity[p.name] = sha(p) if p.exists() else None
original = snapshot(source)
result = {'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
          'python': sys.version, 'kicad': pcbnew.Version(), 'source_identity': source_identity,
          'source_pose': original['USB1'], 'variants': [], 'commands': commands}
bands = {'positive': {'min': .05, 'max': .20}, 'zero': {'min': 0., 'max': .65},
         'legal_body': {'min': 1.40, 'max': 1.50}}
for name, band in bands.items():
    dump(out / (name + '.intent.json'), {'schema': fl.SCHEMA_VERSION, 'kind': fl.KIND,
         'units': 'mm', 'edge_connectors': [{'ref': 'USB1', 'edge': 'west',
         'class': 'edge_receptacle', 'overhang_mm': band, 'max_setback_mm': 0.0}]})

for name, dx in [('original', 0.), ('minus145', -1.45), ('minus210', -2.10)]:
    vd = out / name
    vd.mkdir(exist_ok=True)
    board = vd / 'esp_prog.kicad_pcb'
    copied = copy_board(str(source), str(board))
    if dx:
        pose = original['USB1']
        write_placed_output(str(board), str(board), [{'reference': 'USB1',
            'new_x': pose['xy'][0] + dx, 'new_y': pose['xy'][1], 'new_rotation': pose['rot']}])
    sibling_identity = {ext: sha(board.with_suffix(ext)) if board.with_suffix(ext).exists() else None
                        for ext in SIBLING_EXTS}
    assert all(sibling_identity[ext] == source_identity[source.with_suffix(ext).name]
               for ext in SIBLING_EXTS), sibling_identity
    snap = snapshot(board)
    assert all(snap[r] == old for r, old in original.items() if r != 'USB1')
    assert abs(snap['USB1']['xy'][0] - original['USB1']['xy'][0] - dx) < 1e-6
    b = pcbnew.LoadBoard(str(board))
    fp = next(f for f in b.GetFootprints() if f.GetReference() == 'USB1')
    fab = [g for g in fp.GraphicalItems() if g.GetLayerName() == 'F.Fab' and isinstance(g, pcbnew.PCB_SHAPE)]
    assert all(g.GetShape() == pcbnew.SHAPE_T_SEGMENT for g in fab)
    # Centerlines, excluding line stroke: a physical contract must state which.
    points = [xy(v) for g in fab for v in (g.GetStart(), g.GetEnd())]
    fab_bounds = [min(p[0] for p in points), min(p[1] for p in points),
                  max(p[0] for p in points), max(p[1] for p in points)]
    pad_boxes = [p.GetBoundingBox() for p in fp.Pads()]
    pad_west = min(pcbnew.ToMM(p.GetLeft()) for p in pad_boxes)
    pad_bounds = [pad_west, min(pcbnew.ToMM(p.GetTop()) for p in pad_boxes),
                  max(pcbnew.ToMM(p.GetRight()) for p in pad_boxes),
                  max(pcbnew.ToMM(p.GetBottom()) for p in pad_boxes)]
    data = parse_kicad_pcb(str(board))
    gate0 = BoardOutlineGate(data.board_info, 0.)
    q = QuenchState(data, str(board), clearance=.25, board_edge_clearance=.25,
       crossing_penalty=10., halo_base=.5, halo_coef=.25, halo_weight=2.,
       edge_halo=2., edge_weight=2., grid_step=.1, length_weight=1.)
    part = next(p for p in q.graded_parts() if p.ref == 'USB1')
    drawn, basis = fl.drawn_body_rect(board_bodies(data, str(board)).get('USB1'), data.footprints['USB1'])
    v = {'name': name, 'translation_x_mm': dx, 'board_sha256': sha(board),
         'copied': copied, 'sibling_identity': sibling_identity,
         'native_pose': snap['USB1'], 'native_fab_centerline_bounds': fab_bounds,
         'native_fab_overhang_west_mm': max(0., 114. - fab_bounds[0]),
         'native_pad_bbox_west_gap_mm': pad_west - 114., 'outline_bounds': data.board_info.board_bounds,
         'native_pad_bbox_mm': pad_bounds,
         'legacy_part_rect': part.rect, 'existing_drawn_body_rect': drawn, 'existing_drawn_basis': basis,
         'zero_margin_padbox_outside_mm': gate0.rect_outside_amount(part.rect), 'matrix': []}
    for clearance, edge in [(.25, 0.), (.25, .25), (.25, .55), (.55, .25), (.55, .55)]:
        for band in bands:
            tag = f'{band}-c{clearance}-e{edge}'
            intent = fl.load_intent(str(out / (band + '.intent.json')))
            gr = fl.grade(intent, data, str(board), clearance=clearance, board_edge_clearance=edge)
            doc = fl.to_json(gr)
            dump(vd / (tag + '.api.json'), doc)
            code = run([root / 'py_tools/check_floorplan.py', board, '--intent', intent.source_path,
                        '--clearance', clearance, '--board-edge-clearance', edge,
                        '--json', vd / (tag + '.cli.json')], vd / (tag + '.cli.log'))
            assert code in (0, 4)
            cli = json.loads((vd / (tag + '.cli.json')).read_text(encoding='utf8'))
            # The CLI additionally knows original flags/source; direct grade
            # receives already-resolved values. Compare measured behavior and
            # verify these deliberate provenance enrichments separately.
            cli_rows = copy.deepcopy(cli['edge_seating'])
            for api_row, cli_row in zip(doc['edge_seating'], cli_rows):
                if 'measurements' in cli_row:
                    for key in ('pad_copper_edge_gap', 'copper_edge_shortfall'):
                        got = cli_row['measurements'][key]['requirement_source']
                        assert got == {'source': 'cli', 'value': edge}, got
                        cli_row['measurements'][key]['requirement_source'] = api_row['measurements'][key]['requirement_source']
                    params = cli_row['clearance_parameters']
                    assert params.pop('requested_board_edge_clearance_mm') == edge
                    assert params.pop('requested_copper_clearance_mm') == clearance
            assert cli_rows == doc['edge_seating']
            assert cli['violations'] == doc['violations']
            qq = QuenchState(data, str(board), clearance=clearance, board_edge_clearance=edge,
                crossing_penalty=10., halo_base=.5, halo_coef=.25, halo_weight=2.,
                edge_halo=2., edge_weight=2., grid_step=.1, length_weight=1.)
            row = {'band': band, 'requested_clearance_mm': clearance,
                   'requested_edge_clearance_mm': edge, 'effective_legacy_gate_margin_mm': qq.edge_gate.margin,
                   'legacy_gate_amount_mm': qq.edge_gate.rect_outside_amount(part.rect),
                   'api_pass': doc['pass'], 'api_complete': doc['complete'], 'cli_exit': code,
                   'edge_seating': doc['edge_seating'], 'cli_edge_seating': cli['edge_seating'],
                   'violations': doc['violations']}
            v['matrix'].append(row)
    for edge in [.10, .25, .55]:
        tag = f'drc-edge{edge}'
        code = run([root / 'py_router/check_drc.py', board, '--check-pad-edge',
                    '--board-edge-clearance', edge, '--clearance-margin', 0,
                    '--json', vd / (tag + '.json')], vd / (tag + '.log'))
        assert code in (0, 1)
    v['drc'] = {str(e): json.loads((vd / f'drc-edge{e}.json').read_text(encoding='utf8')) for e in [.10, .25, .55]}
    from fix_kicad_drc_settings import fab_edge_floor
    v['drc_effective_parameters'] = {str(e): {
        'requested_clearance_mm': None, 'requested_board_edge_clearance_mm': e,
        'resolved_clearance_mm': v['drc'][str(e)]['graded_at']['clearance'],
        'resolved_board_edge_clearance_mm': v['drc'][str(e)]['graded_at']['board_edge_clearance'],
        'fab_edge_floor_mm': fab_edge_floor(),
        'effective_edge_check_mm': max(v['drc'][str(e)]['graded_at']['board_edge_clearance'], fab_edge_floor()),
        'basis': 'check_drc.check_drc effective_board_edge_clearance: max(positive scalar, fab_edge_floor)'}
        for e in [.10, .25, .55]}
    v['placement_aggregate'] = grade_pad_legality(data, .25, edge_margin=.25, pcb_file=str(board), worst_n=100)
    result['variants'].append(v)
    dump(out / 'results.json', result)
assert sha(source) == source_identity[source.name]
dump(out / 'results.json', result)
print(json.dumps({'revision': result['revision'], 'variants': len(result['variants']),
                  'commands': len(commands), 'output': str(out)}, indent=2))
