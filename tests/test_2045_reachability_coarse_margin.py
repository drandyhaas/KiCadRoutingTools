#!/usr/bin/env python3
"""#2045: a coarse raster's margin inside its own resolution is no PASSABLE.

A net-tie pad (a 0.19 x 0.9 mm sense land bridged by footprint copper to a
0.2 mm pad of a 0.4 mm-clearance net) is to reach a resistor pad at a 0.16 mm
track. At `--step 0.04611` the tool said PASSABLE by +0.95 um -- a margin 49x
smaller than its own step -- and exited 0, while `--step 0.005` said CAGED by
-89 um, which is what the router found. A raster bottleneck stands up to
`RESOLUTION_STEPS` steps from the continuous one, and here every step from 7.5
to 50 um read 0.6 to 2.7 steps optimistic. Such a reading is now refined at
halved steps until it leaves its own resolution:

  1. coarse, --step 0.04611: CAGED (exit 1), not PASSABLE, and the JSON names
     the +0.95 um reading it refined away;
  2. fine, --step 0.005: CAGED by more than 80 um, decided at the step asked
     for, so nothing is refined (only the uncertain case pays);
  3. the refinement's bisection finds exactly the bottleneck and the cell value
     Kruskal does, on this field at three steps;
  4. a reading the budget cannot settle is INCONCLUSIVE, exit 3, with no defect
     record -- never PASSABLE or CAGED.

No routing, no KiCad; the board is built here.

    python3 tests/test_2045_reachability_coarse_margin.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
TOOL = os.path.join(REPO, 'py_tools', 'check_reachability.py')
ARGS = ['--pad', 'J1.2', '--track', '0.16', '--layers', 'F.Cu']
FAILS = []


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


_EFFECTS = '(effects (font (size 1 1) (thickness 0.15)))'


def build(d):
    """J1: a net tie at (100, 100) turned 90 deg -- pad 1 (SOURCE, a 0.2 mm
    circle) and pad 2 (KELVIN, a 0.19 x 0.9 mm oval) joined by a filled
    0.4 x 0.16 mm F.Cu bridge -- and R1, an 0603 whose pad 1 is the KELVIN
    pad J1.2 must reach. SOURCE is in a 0.4 mm class, KELVIN and the rest
    0.2 mm."""
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')
    b = ['(kicad_pcb (version 20260206) (generator "pcbnew") '
         '(generator_version "10.0")',
         ' (general (thickness 1.6))', ' (paper "A4")',
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))',
         ' (setup (pad_to_mask_clearance 0))',
         ' (footprint "R_0603" (layer "F.Cu") (at 102.5 99.8)',
         f'  (property "Reference" "R1" (at 0 -1.43 0) (layer "F.SilkS") '
         f'{_EFFECTS})',
         '  (attr smd)',
         '  (pad "1" smd roundrect (at -0.825 0) (size 0.8 0.95) '
         '(layers "F.Cu" "F.Mask" "F.Paste") (roundrect_rratio 0.25) '
         '(net "KELVIN"))',
         '  (pad "2" smd roundrect (at 0.825 0) (size 0.8 0.95) '
         '(layers "F.Cu" "F.Mask" "F.Paste") (roundrect_rratio 0.25) '
         '(net "OTHER")))',
         ' (footprint "NetTie" (layer "F.Cu") (at 100 100 90)',
         f'  (property "Reference" "J1" (at 0 -1.2 90) (layer "F.SilkS") '
         f'(hide yes) {_EFFECTS})',
         '  (attr exclude_from_pos_files exclude_from_bom '
         'allow_missing_courtyard)',
         '  (net_tie_pad_groups "1, 2")',
         '  (fp_poly (pts (xy -0.2 -0.08) (xy 0.2 -0.08) (xy 0.2 0.08) '
         '(xy -0.2 0.08)) (stroke (width 0) (type solid)) (fill yes) '
         '(layer "F.Cu"))',
         '  (pad "1" smd circle (at -0.2 0 90) (size 0.2 0.2) (layers "F.Cu") '
         '(net "SOURCE"))',
         '  (pad "2" smd oval (at 0.2 0 90) (size 0.19 0.9) (layers "F.Cu") '
         '(net "KELVIN")))']
    for x1, y1, x2, y2 in ((97, 97, 106, 97), (106, 97, 106, 103),
                           (106, 103, 97, 103), (97, 103, 97, 97)):
        b.append(f' (gr_line (start {x1} {y1}) (end {x2} {y2}) '
                 f'(stroke (width 0.05) (type default)) (layer "Edge.Cuts"))')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write('\n'.join(b) + '\n)\n')

    def cl(name, c, tw, vd, vdr, prio):
        return dict(name=name, clearance=c, track_width=tw, via_diameter=vd,
                    via_drill=vdr, priority=prio)
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w',
              encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': {
                       'min_clearance': 0.15, 'min_track_width': 0.16,
                       'min_via_diameter': 0.6, 'min_via_drill': 0.3,
                       'min_hole_clearance': 0.3, 'min_annular_width': 0.15,
                       'min_edge_clearance': 0.5}}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {
                       'classes': [cl('Default', 0.2, 0.25, 0.6, 0.3, -1),
                                   cl('Sense', 0.2, 0.25, 0.6, 0.3, 3),
                                   cl('BulkPower', 0.4, 3.5, 0.9, 0.4, 5)],
                       'meta': {'version': 4},
                       'netclass_patterns': [
                           {'netclass': 'Sense', 'pattern': '*KELVIN*'},
                           {'netclass': 'BulkPower', 'pattern': 'SOURCE'}]}},
                  f, indent=1)
    return pcb


def _doc(path):
    with open(run_utils.evidence(path, 'check_reachability --json-out'),
              encoding='utf-8') as f:
        return json.load(f)


def main():
    tmp = tempfile.mkdtemp(prefix='t2045_')
    try:
        pcb = build(os.path.join(tmp, 'b'))

        # 1. the coarse step
        js = os.path.join(tmp, 'coarse.json')
        r = run_utils.check(PY + [TOOL, pcb] + ARGS + ['--step', '0.04611',
                                                      '--json-out', js],
                            refuse='VERDICT    CAGED', code=1)
        doc = _doc(js)
        ok('1. coarse: not PASSABLE -- CAGED', doc['verdict'] == 'CAGED',
           doc['verdict'])
        rf = doc.get('refined_from') or []
        ok('1. coarse: the +0.95 um reading at 0.04611 is named as refined '
           'away', bool(rf) and rf[0]['step_mm'] == 0.04611
           and 0 < rf[0]['margin_um'] < 2.0, rf)
        ok('1. coarse: decided at a finer step, outside its resolution',
           doc['step_mm'] < 0.04611
           and -doc['margin_um'] > 1000.0 * doc['resolution_mm'],
           (doc['step_mm'], doc['margin_um'], doc['resolution_mm']))
        ok('1. coarse: the text says what it refined',
           'refined    from +0.9 um at step 0.04611mm' in r.stdout,
           r.stdout[-800:])

        # 2. the fine step
        js = os.path.join(tmp, 'fine.json')
        run_utils.check(PY + [TOOL, pcb] + ARGS + ['--step', '0.005',
                                                  '--json-out', js],
                        refuse='VERDICT    CAGED', code=1)
        doc = _doc(js)
        ok('2. fine: CAGED by more than 80 um', doc['verdict'] == 'CAGED'
           and doc['margin_um'] < -80.0, (doc['verdict'], doc['margin_um']))
        ok('2. fine: decided at the step asked for, nothing refined',
           doc['step_mm'] == 0.005 and doc.get('refined_from') == [],
           (doc['step_mm'], doc.get('refined_from')))

        # 3. bisection == Kruskal on the fixture's own field
        sys.path[:0] = [os.path.join(REPO, d)
                        for d in ('py_router', 'py_placer', 'py_tools')]
        import numpy as np
        from scipy import ndimage
        from kicad_parser import parse_kicad_pcb
        from placement import reachability as RE
        board = parse_kicad_pcb(pcb)
        kel = next(i for i, n in board.nets.items() if n.name == 'KELVIN')
        src = next(i for i, n in board.nets.items() if n.name == 'SOURCE')
        seed, view = (100.0, 99.8), (96.0, 95.8, 104.0, 103.8)
        same = []
        for step in (0.04611, 0.023055, 0.0175):
            s, o, _D = RE.slack_field(board, kel, 'F.Cu', view, 0.2,
                                      {src: 0.4}, step)
            lab, _k = ndimage.label(o)
            sx, sy = (int((seed[0] - view[0]) / step),
                      int((seed[1] - view[1]) / step))
            tgt = (lab > 0) & (lab != lab[sy, sx])
            via = np.zeros_like(o)
            a = RE.widest_path([s], [tgt], via, seed, 0, view, step,
                               return_throat=True)
            b = RE.widest_path_by_threshold([s], [tgt], via, seed, 0, view,
                                            step)
            ax, ay = (int((a[1][1] - view[0]) / step),
                      int((a[1][2] - view[1]) / step))
            bx, by = (int((b[1][1] - view[0]) / step),
                      int((b[1][2] - view[1]) / step))
            same.append(a[0] == b[0] and s[ay, ax] == s[by, bx] == a[0])
        ok('3. bisection == Kruskal: the bottleneck, and a throat cell at it',
           all(same), same)

        # 4. a reading the budget cannot settle
        import check_reachability as cr
        js = os.path.join(tmp, 'inconclusive.json')
        dj = os.path.join(tmp, 'defect.json')
        keep = RE.REFINE_CELL_BUDGET
        RE.REFINE_CELL_BUDGET = 0
        out, err = io.StringIO(), io.StringIO()
        try:
            with contextlib.redirect_stdout(out), \
                    contextlib.redirect_stderr(err):
                rc = cr.main([pcb] + ARGS + ['--step', '0.04611', '--json-out',
                                             js, '--defect-json', dj])
        finally:
            RE.REFINE_CELL_BUDGET = keep
        doc = _doc(js)
        ok('4. no budget: INCONCLUSIVE, exit 3',
           rc == 3 and doc['verdict'] == 'INCONCLUSIVE',
           (rc, doc['verdict'], err.getvalue()[-400:]))
        ok('4. ... said so in the text, and no PASSABLE verdict or advice',
           'VERDICT    INCONCLUSIVE' in out.getvalue()
           and 'VERDICT    PASSABLE' not in out.getvalue()
           and 'PASSABLE means a route EXISTS' not in out.getvalue(),
           out.getvalue()[-600:])
        ok('4. ... and no defect record', not os.path.exists(dj),
           'a defect record was written for an INCONCLUSIVE reading')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)} check(s): {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
