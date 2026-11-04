#!/usr/bin/env python3
"""#1636: a declared cover's hole sites are kept clear BEFORE routing.

`covers[].holes[]` says where a module covering a connector is screwed down,
and check_assembly's `mountable` grades each site's screw-head ring -- after
routing, when a track through the ring means a re-route. Nothing before the
route read the sites: `add_rule_area --mounting-holes` rings the board's own
holes only. `--intent` now also rings each declared site, where the grade
reads it, so the router (either front: it is a rule area on the board) keeps
tracks and vias out.

  1. with --intent, one area per site, named <name>_<connector>_H<n>, on every
     copper layer, tracks and vias not allowed, centred on the site the grade
     reads, at least its ring wide;
  2. a site where the board's own hole is fitted is left to that hole;
  3. without --intent the step is unchanged (control);
  4. --intent without --mounting-holes, and an intent that cannot be loaded,
     refuse for that reason.

    python3 tests/test_1636_cover_hole_keepout.py
"""
import contextlib
import io
import json
import math
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

from run_utils import check as run_check, evidence  # noqa: E402

PY = sys.executable
BOARD = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
TOOL = os.path.join(REPO, 'py_router', 'add_rule_area.py')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def intent(path, holes):
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm', 'min_reader': 22,
           'covers': [{'connector': 'CON2', 'side': 'F', 'axis_deg': 0, 'width_mm': 3,
                       'length_mm': 13, 'underside_mm': 6, 'removable': True,
                       'source': 'test', 'holes': holes}]}
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    return path


def keepouts(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return {k.get('name'): k for k in pcb.board_info.keepouts or ()}, pcb


def main():
    evidence(BOARD, 'the board')
    import mounting_holes as mh
    from placement.floorplan import cover_hole_sites, load_intent
    td = tempfile.mkdtemp(prefix='t1636_')
    try:
        one = intent(os.path.join(td, 'one.json'),
                     [{'length_mm': 11.5, 'off_mm': 0, 'diameter_mm': 2.2}])
        out = os.path.join(td, 'one.kicad_pcb')
        r = run_check([PY, '-X', 'utf8', TOOL, BOARD, out, '--name', 'MOUNT',
                       '--mounting-holes', '--intent', one], accept=True)
        evidence(out, 'the output')
        kos, pcb = keepouts(out)
        site = cover_hole_sites(load_intent(one), pcb)
        print('one declared site:')
        ok('the grade reads one site, with no standoff', len(site) == 1
           and site[0]['standoff'] is None, str(site))
        ko = kos.get('MOUNT_CON2_H1')
        ok('an area named for the connector and the hole', ko is not None, str(sorted(kos)))
        if ko is not None and site:
            x, y = site[0]['at']
            poly = ko['polygon']
            cx = sum(p[0] for p in poly) / len(poly)
            cy = sum(p[1] for p in poly) / len(poly)
            rmin = min(math.hypot(p[0] - x, p[1] - y) for p in poly)
            ok('centred on the site', math.hypot(cx - x, cy - y) < 1e-3, f'({cx}, {cy}) vs ({x}, {y})')
            ok('at least the ring the grade reads', rmin >= mh.reach(2.2) - 1e-6,
               f'{rmin} vs {mh.reach(2.2)}')
            ok('tracks and vias not allowed, pour and pads allowed',
               ko['tracks_allowed'] is False and ko['vias_allowed'] is False
               and ko['copper_pour_allowed'] is not False and ko['pads_allowed'] is not False,
               str({k: ko[k] for k in ko if k.endswith('_allowed')}))
            ok('on every copper layer', set(ko['layers']) >= set(pcb.board_info.copper_layers),
               str(ko['layers']))
        ok('the run says what it wrote', 'cover hole 1 of CON2' in r.stdout, r.stdout[-300:])

        print("a site with the board's own hole fitted:")
        x, y = site[0]['at'] if site else (0.0, 0.0)
        with open(BOARD, encoding='utf-8', newline='') as f:
            text = f.read()
        close = text.rstrip().rfind(')')
        circ = (f'\t(gr_circle (center {x:.4f} {y:.4f}) (end {x + 1.1:.4f} {y:.4f}) '
                f'(stroke (width 0.05) (type solid)) (fill no) (layer "Edge.Cuts") '
                f'(uuid "00000000-0000-4000-8000-000000001636"))\n')
        fitted = os.path.join(td, 'fitted.kicad_pcb')
        with open(fitted, 'w', encoding='utf-8', newline='') as f:
            f.write(text[:close] + circ + text[close:])
        out2 = os.path.join(td, 'fitted_out.kicad_pcb')
        r = run_check([PY, '-X', 'utf8', TOOL, fitted, out2, '--name', 'MOUNT',
                       '--mounting-holes', '--intent', one], accept=True)
        kos2, _p = keepouts(out2)
        ok('the site is left to the hole, which is ringed as itself',
           'MOUNT_CON2_H1' not in kos2 and 'MOUNT_CUT1' in kos2
           and 'left to it: CON2_H1 (CUT1)' in r.stdout, f'{sorted(kos2)}; {r.stdout[-300:]}')

        print('controls and refusals:')
        out3 = os.path.join(td, 'plain.kicad_pcb')
        r = run_check([PY, '-X', 'utf8', TOOL, BOARD, out3, '--name', 'MOUNT',
                       '--mounting-holes'], accept=True)
        kos3, _p = keepouts(out3)
        ok('without --intent nothing is written for the cover',
           not any(n and n.startswith('MOUNT') for n in kos3)
           and 'no mounting hole to write a ring for' in r.stdout, r.stdout[-200:])
        run_check([PY, '-X', 'utf8', TOOL, BOARD, os.path.join(td, 'x.kicad_pcb'),
                   '--name', 'K', '--rect', '0', '0', '1', '1', '--intent', one],
                  refuse='--intent reads the cover hole sites for --mounting-holes',
                  code=2, allow=('usage:',))
        ok('--intent without --mounting-holes is refused', True)
        bad = os.path.join(td, 'bad.json')
        with open(bad, 'w', encoding='utf-8') as f:
            f.write('{')
        run_check([PY, '-X', 'utf8', TOOL, BOARD, os.path.join(td, 'y.kicad_pcb'),
                   '--name', 'M', '--mounting-holes', '--intent', bad],
                  refuse='cannot load intent', code=2)
        ok('an intent that cannot be loaded is refused', True)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
