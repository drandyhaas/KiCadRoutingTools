"""
Router-in-the-loop placement repair.

Routes the board, reads the failure diagnostics (failed nets + the blocker
nets named in the router's frontier analysis), and micro-quenches ONLY the
parts that could help those routes succeed:

  - parts owning pads of the failed nets (move the endpoint out of the
    congested pocket), and
  - parts owning pads of the blocker nets (move the anchor so the blocking
    wall re-routes),

with the failed nets' airwires given extra weight in the quench cost.
High-pin-count parts are excluded from targeting (--max-target-pins) - moving
a resistor that anchors a blocker net is low-risk; dragging a 144-pin QFP to
fix one net is how placements get destroyed.

The new placement is accepted only if a real re-route improves
(failures, router iterations), otherwise it is reverted and the next round
widens the displacement cap.

Usage:
  python place_route_loop.py input.kicad_pcb output.kicad_pcb \
      --route-args '--nets "/*" "Net-*" --track-width 0.2 ...' \
      [quench options]
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys

from kicad_parser import parse_kicad_pcb
import routing_defaults as defaults
from placement.quench import quench
from placement.writer import write_placed_output


def run_route(pcb_file: str, routed_file: str, route_args: str, log_file: str):
    """Run route.py, return (metrics dict, log text)."""
    cmd = [sys.executable, 'route.py', pcb_file, routed_file] + \
        shlex.split(route_args)
    with open(log_file, 'w') as f:
        subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    log = open(log_file).read()

    m = re.search(r'JSON_SUMMARY: (\{.*\})', log)
    if not m:
        raise RuntimeError(f"route.py produced no JSON_SUMMARY (see {log_file})")
    summary = json.loads(m.group(1))

    failed_nets = list(summary.get('failed_single', []))
    # failed_multipoint entries are dicts {net_name, failed_pads}; keep just the
    # name so failed_nets is uniformly net-name strings (downstream uses them as
    # dict keys -> a dict here raises "unhashable type: 'dict'").
    failed_nets += [d['net_name'] if isinstance(d, dict) else d
                    for d in summary.get('failed_multipoint', [])]
    mp_deficit = (summary.get('multipoint_pads_total', 0)
                  - summary.get('multipoint_pads_connected', 0))
    failures = len(summary.get('failed_single', [])) + mp_deficit

    # Blocker nets from frontier diagnostics. Prefer the structured
    # JSON_SUMMARY 'blockers' key (#409): the last-wins attribution of nets
    # still failed at END of run, capped 10/net -- a narrower, more targeted
    # move-candidate set. Fallback for older logs: scrape every transient
    # "  1. /MD1: 46 (31.7%) ..." line in the whole log (includes blockers of
    # nets that later routed and every N-retry re-analysis).
    jb = summary.get('blockers')
    if jb:
        blockers = {b['net'] for e in jb for b in e.get('blocked_by', [])}
    else:
        blockers = set(re.findall(r'^\s+\d+\.\s+(\S+?):\s+\d+\s+\(', log, re.M))

    return {
        'failures': failures,
        'failed_nets': failed_nets,
        'blockers': sorted(blockers),
        'iterations': summary.get('total_iterations', 0),
        'vias': summary.get('total_vias', 0),
    }


def nets_to_refs(pcb_data, net_names, max_pins, locked_patterns):
    """Map net names to the movable component refs that own their pads."""
    import fnmatch
    name_to_id = {net.name: nid for nid, net in pcb_data.nets.items()}
    refs = set()
    for name in net_names:
        nid = name_to_id.get(name)
        if nid is None:
            continue
        for pad in pcb_data.nets[nid].pads:
            refs.add(pad.component_ref)
    out = set()
    for ref in refs:
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        pins = len([p for p in fp.pads if p.net_id > 0])
        if pins > max_pins:
            continue
        if locked_patterns and any(fnmatch.fnmatch(ref, p)
                                   for p in locked_patterns):
            continue
        out.add(ref)
    return out


def better(a, b):
    """Is metrics a better than b? Failures first, then iterations."""
    if a['failures'] != b['failures']:
        return a['failures'] < b['failures']
    return a['iterations'] < b['iterations'] * 0.95


def main():
    parser = argparse.ArgumentParser(
        description="Router-in-the-loop placement repair.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input_file")
    parser.add_argument("output_file")
    parser.add_argument("--route-args", required=True,
                        help="Arguments passed to route.py (quoted string)")
    parser.add_argument("--rounds", type=int, default=5,
                        help="Max repair rounds (default: 5)")
    parser.add_argument("--max-displacement", type=float, default=3.0,
                        help="Initial displacement cap per round in mm "
                             "(default: 3; widened 1.5x after a rejected round)")
    parser.add_argument("--max-target-pins", type=int, default=40,
                        help="Don't move parts with more connected pins than "
                             "this (default: 40)")
    parser.add_argument("--failed-net-weight", type=float, default=3.0,
                        help="Airwire weight multiplier for failed nets "
                             "(default: 3.0)")
    parser.add_argument("--step", type=float, default=0.5,
                        help="Candidate grid step in mm (default: 0.5)")
    parser.add_argument("--length-weight", type=float, default=0.3)
    parser.add_argument("--crossing-penalty", type=float, default=30.0)
    parser.add_argument("--halo-base", type=float, default=0.5)
    parser.add_argument("--halo-coef", type=float, default=0.15)
    parser.add_argument("--halo-weight", type=float, default=2.0)
    parser.add_argument("--edge-halo", type=float, default=2.0)
    parser.add_argument("--edge-weight", type=float, default=2.0)
    parser.add_argument("--clearance", type=float, default=defaults.CLEARANCE)
    parser.add_argument("--board-edge-clearance", type=float, default=0.55)
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP)
    parser.add_argument("--ignore-nets", nargs="+", default=None)
    parser.add_argument("--lock", nargs="+", default=None)
    parser.add_argument("--work-dir", default=None,
                        help="Directory for intermediate files "
                             "(default: alongside output)")
    args = parser.parse_args()

    work = args.work_dir or os.path.dirname(os.path.abspath(args.output_file))
    os.makedirs(work, exist_ok=True)

    cur_file = os.path.join(work, 'loop_round0.kicad_pcb')
    shutil.copy(args.input_file, cur_file)

    print("Round 0: routing initial placement...")
    best = run_route(cur_file, os.path.join(work, 'loop_round0_routed.kicad_pcb'),
                     args.route_args, os.path.join(work, 'loop_round0_route.log'))
    print(f"  failures={best['failures']} iterations={best['iterations']:,}"
          f" vias={best['vias']}")

    max_disp = args.max_displacement

    for rnd in range(1, args.rounds + 1):
        if best['failures'] == 0:
            print("No failures left - stopping.")
            break

        pcb_data = parse_kicad_pcb(cur_file)
        targets = nets_to_refs(pcb_data,
                               best['failed_nets'] + best['blockers'],
                               args.max_target_pins, args.lock)
        if not targets:
            print("No movable target parts - stopping.")
            break

        name_to_id = {net.name: nid for nid, net in pcb_data.nets.items()}
        net_weights = {name_to_id[n]: args.failed_net_weight
                       for n in best['failed_nets'] if n in name_to_id}

        print(f"Round {rnd}: failed={best['failed_nets']}")
        print(f"  blockers={best['blockers'][:8]}"
              f"{'...' if len(best['blockers']) > 8 else ''}")
        print(f"  targeting {len(targets)} parts"
              f" (max_disp={max_disp:.1f}mm): {', '.join(sorted(targets))}")

        placements = quench(
            pcb_data, pcb_file=cur_file,
            max_displacement=max_disp, step=args.step,
            grid_step=args.grid_step, clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            crossing_penalty=args.crossing_penalty,
            length_weight=args.length_weight,
            halo_base=args.halo_base, halo_coef=args.halo_coef,
            halo_weight=args.halo_weight,
            edge_halo=args.edge_halo, edge_weight=args.edge_weight,
            ignore_nets=args.ignore_nets, lock_refs=args.lock,
            move_refs=targets, net_weights=net_weights,
        )

        if not placements:
            print("  Quench found no improving moves - widening cap.")
            max_disp *= 1.5
            continue

        cand_file = os.path.join(work, f'loop_round{rnd}.kicad_pcb')
        write_placed_output(cur_file, cand_file, placements)

        metrics = run_route(
            cand_file, os.path.join(work, f'loop_round{rnd}_routed.kicad_pcb'),
            args.route_args, os.path.join(work, f'loop_round{rnd}_route.log'))
        print(f"  -> failures={metrics['failures']}"
              f" iterations={metrics['iterations']:,} vias={metrics['vias']}")

        if better(metrics, best):
            print(f"  ACCEPTED (was failures={best['failures']},"
                  f" iterations={best['iterations']:,})")
            best = metrics
            cur_file = cand_file
            max_disp = args.max_displacement
        else:
            print("  REJECTED - reverting, widening displacement cap.")
            max_disp *= 1.5

    shutil.copy(cur_file, args.output_file)
    print(f"Final: failures={best['failures']} iterations={best['iterations']:,}"
          f" vias={best['vias']}")
    print(f"Wrote {args.output_file}")


if __name__ == "__main__":
    main()
