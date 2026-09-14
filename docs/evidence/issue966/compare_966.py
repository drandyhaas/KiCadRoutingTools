"""Summarize independently generated baseline/candidate routing inspections."""
import argparse
import hashlib
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('baseline', type=Path)
p.add_argument('candidate', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
result = {'cases': {}, 'notes': [
    'min_clearance_used is the router-reported used rule; it is not a geometric nearest-copper distance.',
    'Raw default0 is pinned to physical0.1; raw default0.2 is retained. No clearance override or ceiling supplied.',
    'The public sparse board reproduces divergent rule resolution; both baseline routes succeed with identical copper.',
    'DDR3 boxed-in/ripup and hypothetical routing quality gains are not reproduced by these cases.',
]}
old = json.loads(a.baseline.read_text())
new = json.loads(a.candidate.read_text())
result['baseline_revision'] = old['revision']
result['candidate_revision'] = new['revision']
result['kicad'] = new['kicad']
for case in ('zero', 'positive'):
    src = old['cases'][case]['input']
    assert src == new['cases'][case]['input'], case
    cr = {'source_board_sha256': src['board_sha256'], 'source_project_sha256': src['project_sha256'],
          'footprint_count': len(src['footprints']), 'pad_count': len(src['pads']),
          'outline_count': len(src['outline']), 'source_copper_count': len(src['copper']), 'runs': {}}
    result['cases'][case] = cr
    reference_copper = None
    for phase, report in [('baseline', old), ('candidate', new)]:
        for front in ('cli', 'gui'):
            run = report['cases'][case][front]
            output = run['output']
            for mechanic in ('footprints', 'pads', 'outline'):
                assert src[mechanic] == output[mechanic], (phase, front, mechanic)
            if reference_copper is None:
                reference_copper = output['copper']
            assert reference_copper == output['copper'], (phase, front, 'copper geometry')
            cr['runs'][phase + '-' + front] = {
                'router_reported_min_clearance_used_mm': run['summary']['min_clearance_used'],
                'board_sha256': output['board_sha256'], 'project_sha256': output['project_sha256'],
                'copper_net_layer_start_end_width_nm': output['copper'],
                'mechanics_exactly_preserved': True,
                'net_settings_exactly_preserved': run['net_settings_preserved'],
                'native_drc_types': run['native_drc_types'],
                'native_unconnected_count': run['native_unconnected_count'],
                'native_target_unconnected_count': run['native_target_unconnected_count'],
            }
assert result['cases']['zero']['runs']['baseline-gui']['router_reported_min_clearance_used_mm'] == .25
assert result['cases']['zero']['runs']['candidate-gui']['router_reported_min_clearance_used_mm'] == .1
for case in ('zero', 'positive'):
    expected = .1 if case == 'zero' else .2
    for phase, front in [('baseline','cli'), ('candidate','cli'), ('candidate','gui')]:
        assert result['cases'][case]['runs'][phase+'-'+front]['router_reported_min_clearance_used_mm'] == expected
a.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
print(json.dumps({'baseline': old['revision'], 'candidate': new['revision'], 'runs_verified':8}))
