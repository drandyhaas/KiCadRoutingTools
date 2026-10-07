#!/usr/bin/env python3
"""fa10 P1: does the SEAT predicate admit poses the GRADER gates?

The seeder seats parts with `seeder.pose_ok`, on the state `place_seed`
builds (`pose_score.make_state`, called here, not copied). check_assembly
grades the written board with `legality.CourtyardCensus`. When they price a
part with different geometry the seeder writes boards the grader calls NOT
BUILDABLE (#1182: pad boxes vs drawn bodies; #1184/#1212: containers the
search never tests), and nothing in either tool says so. This instrument
puts the two side by side, pose by pose.

For every movable part it samples a grid of poses around the part's file
pose (everything else stays where the file has it), and asks both:

  * generator: `pose_ok(state, ref, x, y, rot, exclude=set())`;
  * grader:    `census.grade_ref(ref, pose, moved={ref}).gating` -- the
               courtyard pairs check_assembly --baseline would GATE with
               this part moved here (and, since the containers rule,
               the pin_in_courtyard pairs).

and counts

  * UNDER: the generator admits a pose the grader gates. A seat that ships
    a defect. Every one is listed with its pair.
  * OVER:  the generator refuses on its COURTYARD conjunct a pose the
    grader has no gating pair for against that blocker. A seat the search
    forgoes for nothing. Expected to be non-zero by design: the seat
    demands a clearance GAP between rects where the grader has area and
    depth floors. Reported per veto check, never gated.

    python3 tests/measure_gate_vs_grader.py [boards ...] [--corpus]
        [--step 1.0] [--radius 2] [--max-parts N] [--json out]

Exit 0 always on a completed run (it is a measurement); 2 when no board
could be graded.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))


def audit_board(board: str, *, step: float = 1.0, radius: int = 2,
                max_parts: int = 0, clearance=None, state=None,
                census=None) -> dict:
    """The UNDER/OVER census of one board. `state`/`census` may be handed in
    (a test manufactures a disagreement through them); by default both are
    built the way place_seed and check_assembly build theirs."""
    from kicad_parser import parse_kicad_pcb
    from placement import legality, seeder
    import pose_score
    pcb = parse_kicad_pcb(board)
    if clearance is None:
        from list_nets import board_floor_knobs
        clearance, _edge, _k = board_floor_knobs(board, None, None)
    if state is None:
        state = pose_score.make_state(pcb, board, clearance=clearance)
    if census is None:
        census = legality.CourtyardCensus(pcb, board)
    offs = [(dx * step, dy * step) for dx in range(-radius, radius + 1)
            for dy in range(-radius, radius + 1)]
    refs = [r for r in sorted(state.parts)
            if not state.parts[r].locked and r in census.lbs]
    if max_parts:
        refs = refs[:max_parts]
    samples = admitted = 0
    under, over = [], {}
    for ref in refs:
        fp = pcb.footprints[ref]
        x0, y0, rot = fp.x, fp.y, (fp.rotation or 0.0)
        for dx, dy in offs:
            pose = (round(x0 + dx, 4), round(y0 + dy, 4), rot)
            samples += 1
            ok = seeder.pose_ok(state, ref, *pose, exclude=set())
            gating = census.grade_ref(ref, pose, moved={ref}).gating
            if ok:
                admitted += 1
                if gating:
                    under.append({'ref': ref, 'pose': list(pose),
                                  'pairs': [[q.a, q.b, q.kind, q.area_mm2,
                                             q.depth_mm] for q in gating]})
                continue
            veto = state.candidate_veto(ref, *pose)
            check = veto[0] if veto else 'pose_ok'
            blocker = veto[1] if veto else None
            if check != 'courtyard':
                continue
            if not any(blocker in (q.a, q.b) for q in gating):
                over[check] = over.get(check, 0) + 1
    return {'board': board, 'clearance': clearance, 'parts': len(refs),
            'samples': samples, 'admitted': admitted, 'under': len(under),
            'under_cases': under, 'over': over}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('boards', nargs='*')
    ap.add_argument('--corpus', action='store_true',
                    help='add the git-tracked corpus boards')
    ap.add_argument('--step', type=float, default=1.0)
    ap.add_argument('--radius', type=int, default=2)
    ap.add_argument('--max-parts', type=int, default=0)
    ap.add_argument('--json')
    args = ap.parse_args(argv)
    boards = list(args.boards)
    if args.corpus:
        import run_utils
        boards += [os.path.join(ROOT, b) for b in run_utils.corpus_boards()]
    rows = []
    for b in boards:
        try:
            row = audit_board(b, step=args.step, radius=args.radius,
                              max_parts=args.max_parts)
        except Exception as exc:                             # noqa: BLE001
            row = {'board': b, 'error': f'{type(exc).__name__}: {exc}'}
            print(f"ERROR  {os.path.basename(b)}: {row['error']}")
            rows.append(row)
            continue
        rows.append(row)
        print(f"{os.path.basename(b):40s} parts {row['parts']:4d} samples "
              f"{row['samples']:6d} admitted {row['admitted']:6d} "
              f"UNDER {row['under']:4d} OVER {row['over']}")
        for c in row['under_cases'][:5]:
            print(f"    UNDER {c['ref']} at {c['pose']}: {c['pairs']}")
    graded = [r for r in rows if 'error' not in r]
    print(f"\n{len(graded)} board(s): UNDER "
          f"{sum(r['under'] for r in graded)}, OVER "
          f"{sum(sum(r['over'].values()) for r in graded)}")
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(rows, fh, indent=1)
    return 0 if graded else 2


if __name__ == '__main__':
    sys.exit(main())
