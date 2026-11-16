#!/usr/bin/env python3
"""Refine selected footprints near their input positions.

    place_refine.py BOARD OUT --refs U1 C1 --radius 3 --moves 2000 --json report.json

The search reuses place_simple's cost model and proposals. Selected unlocked
footprints may translate within the radius and turn in quarter turns. Other
footprints retain their input poses. The result reports placement geometry
and objective cost; routing and KiCad DRC remain separate checks.

Exit 0 means the selected footprints are legal, 4 leaves them illegal, and
2 means invalid input. With no improving legal arrangement, OUT is an exact
copy of BOARD and the report says improved=false.
"""

KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'actor'}

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile

import _path  # noqa: F401


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('board', help='input board; routed boards are refused')
    parser.add_argument('out', help='result board, distinct from the input')
    parser.add_argument('--refs', nargs='+', required=True,
                        help='unique footprint keys to refine; locked parts remain fixed')
    parser.add_argument('--radius', type=float, required=True, metavar='MM',
                        help='maximum distance of each origin from its input position')
    parser.add_argument('--moves', type=int, default=2000,
                        help='number of bounded native proposals (default 2000)')
    parser.add_argument('--seed', type=int, default=0, help='reproducible random stream')
    parser.add_argument('--groups', help='optional native functional groups file')
    parser.add_argument('--intent', help='optional native floorplan intent')
    parser.add_argument('--json', help='write geometry, cost and movement report')
    return parser


def same_file(first, second):
    return (Path(first).resolve() == Path(second).resolve()
            or (Path(first).exists() and Path(second).exists()
                and os.path.samefile(first, second)))


def legal(model, ref):
    from placement.edge_seat import legal_on_edge
    from placement.simple import legal as pose_legal
    state = model.state
    part = state.parts[ref]
    pose = (part.x, part.y, part.rot)
    fits = (legal_on_edge(state, ref, pose) if ref in model.edge_seats
            else pose_legal(state, ref, *pose))
    return fits and state.intent_clear(ref, part.grade_rects(*pose))


def restore(model, poses):
    state = model.state
    for ref, pose in poses.items():
        state.parts[ref].ensure_rotation(pose[2])
        state.apply_move(ref, *pose)
    model.reprice_all()
    model.total()


def measure(model, selected):
    cost = model.total()['cost']
    if not math.isfinite(cost):
        raise ValueError('the native placement objective is not finite')
    return {'cost': cost, 'illegal_refs': [ref for ref in selected if not legal(model, ref)]}


def search(model, selected, radius, moves, seed):
    from placement.simple_anneal import Annealer, LAM_END, LAM_START, T_END
    annealer = Annealer(model, selected, seed=seed, seats=model.edge_seats)
    original = {ref: annealer.pose(ref) for ref in model.state.parts}
    before = measure(model, selected)
    held_legal = [ref for ref in original if ref not in selected and legal(model, ref)]
    best_poses = original
    best = before
    temperature = annealer.start_temperature() if selected else 1.0
    kinds = annealer.kinds()
    accepted = 0
    for index in range(moves if selected and kinds else 0):
        fraction = index / moves
        annealer.lam = LAM_START * (LAM_END / LAM_START) ** fraction
        reach = max(0.01, radius * (1.0 - fraction))
        proposal = annealer.propose(annealer.rng.choice(kinds), reach)
        if not proposal or any(
            ref not in selected or math.dist(pose[:2], original[ref][:2]) > radius
            for ref, pose in proposal.items()
        ):
            continue
        kept, _cost_delta, _overlap_delta = annealer.trial(
            proposal, temperature * (T_END / temperature) ** fraction)
        if not kept:
            continue
        accepted += 1
        cost = model.total()['cost']
        if not math.isfinite(cost):
            continue
        if not before['illegal_refs'] and cost >= best['cost'] - 1e-9:
            continue
        if any(not legal(model, ref) for ref in selected + held_legal):
            continue
        if best['illegal_refs'] or cost < best['cost'] - 1e-9:
            best = {'cost': cost, 'illegal_refs': []}
            best_poses = {ref: annealer.pose(ref) for ref in original}
    restore(model, best_poses)
    return original, best_poses, before, best, accepted


def refine(args):
    import place_simple
    from copy_board import SIBLING_EXTS
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.groups import parse_sources
    from placement.grouping import load_groups, validate_doc
    from placement.parser import extract_locked_refs
    from placement.portfolio import copy_siblings
    from placement.writer import write_placed_output

    if not math.isfinite(args.radius) or args.radius <= 0 or args.moves <= 0:
        raise ValueError('--radius must be finite and positive; --moves must be positive')
    if len(args.refs) != len(set(args.refs)):
        raise ValueError('--refs must contain unique footprint keys')
    if same_file(args.board, args.out):
        raise ValueError('the output board must differ from the input')
    siblings = [str(Path(args.board).with_suffix(ext)) for ext in SIBLING_EXTS]
    inputs = [args.board, *siblings, *[value for value in (args.groups, args.intent) if value]]
    if any(same_file(args.out, value) for value in inputs):
        raise ValueError('the output board must differ from all inputs')
    outputs = [args.out, *[str(Path(args.out).with_suffix(ext)) for ext in SIBLING_EXTS]]
    sibling_writes = [destination for source, destination in zip(siblings, outputs[1:])
                      if Path(source).is_file() and not same_file(source, destination)]
    if any(same_file(destination, value) for destination in sibling_writes for value in inputs):
        raise ValueError('project/rule destinations must differ from input files')
    if args.json and any(same_file(args.json, value) for value in inputs + outputs):
        raise ValueError('the JSON report must differ from the board and input files')

    pcb = parse_kicad_pcb(args.board)
    input_poses = {ref: (part.x, part.y, part.rotation or 0.0)
                   for ref, part in pcb.footprints.items()}
    if pcb.segments or pcb.vias:
        raise ValueError('refine an unrouted board; existing tracks or vias cannot follow moves')
    unknown = sorted(set(args.refs) - set(pcb.footprints))
    if unknown:
        raise ValueError('unknown footprint keys: ' + ', '.join(unknown))
    doc = (load_groups(args.groups) if args.groups else validate_doc({
        'schema': 1, 'groups': [{'name': ref, 'refs': [ref]} for ref in pcb.footprints],
    }))
    base = json.loads(Path(args.intent).read_text(encoding='utf-8')) if args.intent else {}
    intent = floorplan.load_intent(args.intent) if args.intent else None
    blocks = floorplan.resolve_blocks(intent, pcb, parse_sources('auto'))[0] if intent else None
    declared = {}
    if intent:
        sides, conflicts = place_simple.face_targets(intent, blocks, doc, pcb)
        if conflicts:
            raise ValueError('contradictory face declarations in the intent or groups')
        declared, changes = floorplan.declared_moves(intent, sides, pcb)
        if changes:
            raise ValueError('the input must already satisfy declared fixed poses and faces')
    held = (set(pcb.footprints) - set(args.refs)) | set(declared) | extract_locked_refs(args.board)
    defaults = place_simple.build_parser().parse_args([
        args.board, args.out, '--groups', args.groups or '-', '--intent', args.intent or '-',
    ])
    state, model, movable, notes = place_simple.build(
        pcb, args.board, doc, base, defaults, intent, blocks, held)
    selected = sorted(set(args.refs).intersection(movable))
    restore(model, {ref: input_poses[ref] for ref in state.parts})
    original, best, before, after, accepted = search(
        model, selected, args.radius, args.moves, args.seed)
    placements = [
        {'reference': ref, 'new_x': pose[0], 'new_y': pose[1], 'new_rotation': pose[2]}
        for ref, pose in best.items() if pose != original[ref]
    ]
    report = {
        'schema': 1, 'improved': bool(placements), 'seed': args.seed,
        'moves': args.moves if selected else 0, 'accepted_proposals': accepted,
        'radius_mm': args.radius, 'selected_refs': selected,
        'held_refs': sorted(set(args.refs) - set(selected)), 'before': before, 'after': after,
        'changes': [
            {'ref': ref, 'before': list(original[ref]), 'after': list(best[ref]),
             'distance_mm': math.dist(original[ref][:2], best[ref][:2])}
            for ref in sorted(best) if best[ref] != original[ref]
        ],
        'measurement': 'native placement objective and geometric legality; routing and DRC unmeasured',
        'notes': notes,
    }
    encoded_report = json.dumps(report, indent=2, allow_nan=False)
    output = Path(args.out)
    with tempfile.TemporaryDirectory(prefix='place_refine_', dir=output.parent) as work:
        result = Path(work) / output.name
        if placements:
            write_placed_output(args.board, str(result), placements)
            actual = parse_kicad_pcb(str(result))
            if actual.footprints.keys() != pcb.footprints.keys():
                raise ValueError('the written board has a different footprint set')
            for ref, expected in pcb.footprints.items():
                pose = best.get(ref, input_poses[ref])
                found = actual.footprints[ref]
                if (math.dist((found.x, found.y), pose[:2]) > 1e-5
                        or abs(((found.rotation or 0) - pose[2] + 180) % 360 - 180) > 1e-5
                        or found.layer != expected.layer):
                    raise ValueError('the written board differs from the selected poses')
        else:
            shutil.copyfile(args.board, result)
        os.replace(result, output)
    copy_siblings(args.board, args.out)
    if args.json:
        Path(args.json).write_text(encoded_report, encoding='utf-8')
    print(f"place_refine: {'improved' if placements else 'no improvement'}; "
          f"{len(placements)} moved; {len(after['illegal_refs'])} selected footprint(s) illegal")
    return 4 if after['illegal_refs'] else 0


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return refine(args)
    except (OSError, ValueError) as error:
        print(f'place_refine: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
