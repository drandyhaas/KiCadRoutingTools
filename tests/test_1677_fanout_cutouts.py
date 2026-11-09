#!/usr/bin/env python3
"""#1677: a BGA fanout keeps its copper off the board's Edge.Cuts.

None of the fanout's engines read the board's edges: the under-pad grid stamped
no Edge.Cuts at all, the channel engine's collision checks read tracks and vias
only, and a via's own gates read no outline. So on a part standing over internal
cut-outs an escape was laid straight across the opening, the step reported every
ball escaped, and KiCad reported the copper at 0 mm from the edge.

The fixture: a 20 x 16 ball array with a 3.6 mm slot of no balls across it, cut
out of the board. Only the row just above the slot is fanned, so its way out is
either the 8 mm across the slot or 10 mm or more the other three ways.

Checks, for each engine:
  1. No track or via crosses the cut-out or comes inside the edge clearance:
     the board's declared minimum when it has one, and the fab floor otherwise.
  2. Every ball still escapes or is named failed.
  3. The same board with no cut-out escapes as many balls (the control), and
     the escape that base code laid across the slot is laid there again.

    python3 tests/test_1677_fanout_cutouts.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb                 # noqa: E402
from bga_fanout import generate_bga_fanout               # noqa: E402
from geometry_utils import point_to_segment_distance, segment_to_segment_distance  # noqa: E402

failures = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f' -- {detail}' if detail else ''))
    if not cond:
        failures.append(name)


COLS, ROWS, SLOT_ROWS = 20, 16, (12, 13, 14, 15)
PITCH = 1.0
X0, Y0 = 20.0, 20.0
# Rows of balls 0..11 and 16..19 stand above and below the slot.
CUT = (X0 - 0.5, Y0 + 11.7, X0 + COLS + 0.5, Y0 + 15.3)
FANNED_ROW = 11


def board_text(cut_out):
    nets, pads, parts = [(0, '')], [], []
    nid = 1
    for r in range(ROWS + 4):
        if r in SLOT_ROWS:
            continue
        for c in range(COLS):
            nets.append((nid, f'N{r}_{c}'))
            net = f'(net {nid} "N{r}_{c}")'
            pads.append(f'  (pad "{r}_{c}" smd circle (at {c * PITCH} {r * PITCH}) '
                        f'(size 0.45 0.45) (layers "F.Cu" "F.Mask") {net})')
            if r == FANNED_ROW:
                # a partner far off, so the net has somewhere to go
                parts.append(f' (footprint "t:R" (layer "F.Cu") (at {5 + c * 3} 60)\n'
                             f'  (property "Reference" "R{nid}" (at 0 0) (layer "F.SilkS"))\n'
                             f'  (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask") {net}))')
            nid += 1
    cut = ''
    if cut_out:
        x0, y0, x1, y1 = CUT
        cut = (f' (gr_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.05) '
               f'(type default)) (fill no) (layer "Edge.Cuts"))\n')
    net_lines = '\n'.join(f' (net {i} "{n}")' for i, n in nets)
    return ('(kicad_pcb (version 20260206) (generator test)\n'
            ' (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal)'
            ' (25 "Edge.Cuts" user))\n'
            f'{net_lines}\n'
            ' (gr_rect (start 0 0) (end 80 70) (stroke (width 0.05) (type default)) (fill no)'
            ' (layer "Edge.Cuts"))\n'
            f'{cut}'
            f' (footprint "t:BGA" (layer "F.Cu") (at {X0} {Y0})\n'
            '  (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
            + '\n'.join(pads) + '\n )\n'
            + '\n'.join(parts) + '\n)\n')


def fan(td, name, cut_out, method, project=None):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(board_text(cut_out))
    if project is not None:
        with open(path[:-4] + '_pro', 'w', encoding='utf-8') as f:
            json.dump({'board': {'design_settings': {'rules': {
                'min_copper_edge_clearance': project}}}}, f)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        tracks, vias, _rm, failed = generate_bga_fanout(
            pcb.footprints['U1'], pcb, net_filter=[f'N{FANNED_ROW}_*'],
            layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], track_width=0.1, clearance=0.1,
            via_size=0.45, via_drill=0.2, escape_method=method, plane_drop='off')
    return pcb, tracks, vias, failed


def rect_edges(r):
    x0, y0, x1, y1 = r
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def gap_to_cut(tracks, vias):
    """Smallest edge-to-copper gap; negative when copper enters the opening."""
    best = float('inf')
    x0, y0, x1, y1 = CUT

    def inside(x, y):
        return x0 < x < x1 and y0 < y < y1
    for t in tracks:
        (ax, ay), (bx, by) = t['start'], t['end']
        if inside(ax, ay) or inside(bx, by):
            return -1.0
        for e in rect_edges(CUT):
            d = segment_to_segment_distance(ax, ay, bx, by, *e)
            best = min(best, d - t['width'] / 2)
    for v in vias:
        if inside(v['x'], v['y']):
            return -1.0
        for e in rect_edges(CUT):
            best = min(best, point_to_segment_distance(v['x'], v['y'], *e) - v['size'] / 2)
    return best


def crosses(tracks):
    x0, y0, x1, y1 = CUT
    return [t for t in tracks
            if any(segment_to_segment_distance(*t['start'], *t['end'], *e) < 1e-9
                   for e in rect_edges(CUT))]


def main():
    print('#1677: a BGA fanout keeps its copper off Edge.Cuts')
    with tempfile.TemporaryDirectory() as td:
        n_row = COLS
        for method, want in (('dogbone', True), ('underpad', True), ('channel', False)):
            print(f'-- {method}')
            _p, tr0, vi0, fl0 = fan(td, f'ctl_{method}', False, method)
            ok('control: with no cut-out the row escapes',
               not fl0 and {t['net_id'] for t in tr0}, f'failed {fl0}')
            if want:
                ok('control: a track runs where the cut-out goes (the board tests something)',
                   crosses(tr0), f'{len(crosses(tr0))} crossing')
            pcb, tracks, vias, failed = fan(td, f'cut_{method}', True, method)
            g = gap_to_cut(tracks, vias)
            # no project: the fab floor (0.20) over the 0.10 clearance
            ok('no copper crosses the cut-out', not crosses(tracks),
               f'{len(crosses(tracks))} track(s) cross')
            ok('copper holds the fab edge floor off the cut-out', g >= 0.20 - 1e-3, f'gap {g:.3f}')
            nets = {t['net_id'] for t in tracks}
            names = {n.name for i, n in pcb.nets.items() if i in nets}
            missing = {f'N{FANNED_ROW}_{c}' for c in range(n_row)} - names - set(failed)
            ok('every ball escapes or is named failed', not missing, f'{sorted(missing)[:4]}')
            ok('the row still escapes', len(names) >= n_row - 2,
               f'{len(names)} of {n_row}, failed {failed}')
        print('-- a declared minimum')
        _p, tracks, vias, _f = fan(td, 'declared', True, 'dogbone', project=0.45)
        g = gap_to_cut(tracks, vias)
        ok("the board's own min_copper_edge_clearance is held", g >= 0.45 - 1e-3, f'gap {g:.3f}')

        print('-- the shared gate')
        from check_drc import BoardEdgeGate
        pcb, tracks, vias, failed = fan(td, 'grade', True, 'dogbone')
        gate = BoardEdgeGate(pcb, 0.2)
        ok('the shared gate refuses a track across the opening',
           gate.segment_refused(30.0, CUT[1] - 1, 30.0, CUT[3] + 1, 0.1))
        ok('the shared gate refuses a via in the opening',
           gate.via_refused(30.0, (CUT[1] + CUT[3]) / 2, 0.45))
        ok('the shared gate lets a via stand clear of it',
           not gate.via_refused(30.0, CUT[1] - 1.0, 0.45))
    print('FAILED: ' + '; '.join(failures) if failures else 'All checks passed')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
