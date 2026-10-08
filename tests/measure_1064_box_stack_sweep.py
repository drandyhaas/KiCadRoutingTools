#!/usr/bin/env python3
"""#1064 / #1127: where the quench's BOX stack gate and check_assembly's EXACT
channel disagree, on esp_prog's C4 turned to -45 beside Y1's corner.

    python3 -B -X utf8 tests/measure_1064_box_stack_sweep.py [--out rows.csv]
    # defaults: x 135.86..136.26, y 103.42..103.82, step 0.04 (11 x 11),
    # rot 315 -- the grid behind "35 of 121" in #1064's PR and #1127

NOT named `test_*`, so `run_all.py` never collects it: 121 written boards,
a few minutes.

For every pose: the box side is a real `QuenchState` (built the way
`render_placement.PlacementModel` builds it, board-first floors) after
`apply_move`, read through `legality_ctx.pair_shortfall(C4, Y1).stack`; the
exact side writes the pose (`write_placed_output`), re-parses it and asks
`grade_body_overlap` with check_assembly's arguments whether C4/Y1 is a
blocking pair. The headline is the count of poses the box gate calls a stack
and the exact channel calls clean -- the poses #1064 would have refused
had place_pose gated on `PairShortfall.stack`, as the issue proposed.
Measured at 7000a455: 35 of 121 (and 0 the other way).
"""
import argparse
import csv
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', default=None, help='write the rows as CSV')
    ap.add_argument('--x0', type=float, default=135.86)
    ap.add_argument('--x1', type=float, default=136.26)
    ap.add_argument('--y0', type=float, default=103.42)
    ap.add_argument('--y1', type=float, default=103.82)
    ap.add_argument('--step', type=float, default=0.04)
    ap.add_argument('--rot', type=float, default=315.0)
    a = ap.parse_args()
    os.chdir(ROOT)
    sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
    __import__("_path")  # sets up sys.path
    import routing_defaults as defaults
    from kicad_parser import parse_kicad_pcb
    from list_nets import board_floor_knobs, board_floor
    from placement.quench import QuenchState
    from placement.legality import grade_body_overlap
    from placement.writer import write_placed_output

    board = os.path.join('kicad_files', 'esp_prog.kicad_pcb')
    ref, other = 'C4', 'Y1'
    pcb = parse_kicad_pcb(board)
    clr, edge, _k = board_floor_knobs(board, clearance=None,
                                      board_edge_clearance=None,
                                      clearance_default=defaults.CLEARANCE,
                                      edge_default=0.55)
    st = QuenchState(pcb, board, clearance=clr, board_edge_clearance=edge,
                     crossing_penalty=10.0, halo_base=0.5, halo_coef=0.25,
                     halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
                     grid_step=defaults.GRID_STEP, length_weight=1.0)
    ca_clr, _src = board_floor(board, 'clearance', None, defaults.CLEARANCE)
    tmpd = tempfile.mkdtemp(prefix='m1064_')
    tmpf = os.path.join(tmpd, 'cand.kicad_pcb')
    nx = int(round((a.x1 - a.x0) / a.step)) + 1
    ny = int(round((a.y1 - a.y0) / a.step)) + 1
    rows = []
    try:
        for j in range(ny):
            y = round(a.y0 + j * a.step, 4)
            for i in range(nx):
                x = round(a.x0 + i * a.step, 4)
                st.apply_move(ref, x, y, a.rot)
                box = bool(st.legality_ctx.pair_shortfall(ref, other).stack)
                write_placed_output(board, tmpf, [{
                    'reference': ref, 'new_x': x, 'new_y': y,
                    'new_rotation': a.rot}])
                g = grade_body_overlap(parse_kicad_pcb(tmpf), ca_clr,
                                       intent_waivers=(), pcb_file=tmpf,
                                       courtyard_severity='auto')
                exact = any({q.a, q.b} == {ref, other}
                            for q in g['blocking_pairs'])
                rows.append({'x': x, 'y': y, 'rot': a.rot,
                             'box_stack': int(box), 'exact_blocking': int(exact)})
    finally:
        shutil.rmtree(tmpd, ignore_errors=True)
    if a.out:
        with open(a.out, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    over = sum(r['box_stack'] and not r['exact_blocking'] for r in rows)
    under = sum(r['exact_blocking'] and not r['box_stack'] for r in rows)
    print('%s at %g beside %s, %d poses: box stack but exact clean %d; '
          'exact blocking but no box stack %d'
          % (ref, a.rot, other, len(rows), over, under))


if __name__ == '__main__':
    main()
