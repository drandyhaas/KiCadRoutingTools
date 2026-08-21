#!/usr/bin/env python3
"""Pocket census: windowed routing demand vs free copper area (run-23).

Every per-net handoff instrument passed run 23's board -- check_channels: 0
starved faces; check_reachability: every failing pad PASSABLE copper-free;
crossings below the damaged baseline -- and two nets then died across ~20
route laps in ONE pocket (the RN7/U6/J2 region), where committed copper
finally wrapped RN7 at 0.095mm against a 0.45mm corridor need. Per-net tests
are blind to SIMULTANEOUS routability by construction; this tool reports the
aggregate: the board binned at --bin mm, each bin's distinct demanding nets
against its free area, top offenders named with their nets and parts.

REPORT-ONLY, deliberately: a new metric mis-thresholded is noise, so nothing
gates on it this sprint. The boundary review (the blind-first visual gate)
must DISPOSITION every window this prints -- in writing, against a named
plan -- or refuse the handoff itself.

    python3 -X utf8 py_tools/check_pockets.py <board> [--nets GLOB...]
        [--bin MM] [--top N] [--threshold NETS_PER_MM2] [--json PATH]

Exit codes: 0 always (report), 2 usage/load error.
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import sys


def main():
    p = argparse.ArgumentParser(
        description="Windowed demand/free-area census of a board.")
    p.add_argument("board")
    p.add_argument("--nets", nargs='+', default=['*'], metavar='GLOB',
                   help="the DEMAND set (route.py glob syntax, '!' excludes). "
                        "Default: every net. Pass the set the route step will "
                        "carry so the census asks the same question")
    p.add_argument("--bin", type=float, default=2.0, metavar='MM',
                   help="window size (default 2.0mm -- about three 0.15/0.15 "
                        "lanes plus a via, the scale run 23's pocket failed "
                        "at; the router's own congestion cost uses 1.0)")
    p.add_argument("--top", type=int, default=8, metavar='N',
                   help="how many windows to print (default 8)")
    p.add_argument("--threshold", type=float, default=None,
                   metavar='NETS_PER_MM2',
                   help="report only windows above this demand/free-area "
                        "ratio. Default: none -- print the top N whatever "
                        "their ratio, because an absolute threshold is "
                        "exactly what this tool is too young to own")
    p.add_argument("--json", default=None, metavar='PATH',
                   help="write the full census as JSON")
    args = p.parse_args()

    from congestion_field import congestion_bins
    from kicad_parser import parse_kicad_pcb
    from net_queries import expand_net_patterns

    try:
        pcb = parse_kicad_pcb(args.board)
    except Exception as exc:
        print(f"cannot parse {args.board}: {exc}", file=sys.stderr)
        return 2

    names = expand_net_patterns(pcb, args.nets)
    name_by_id = {n.net_id: n.name for n in pcb.nets.values()}
    ids = [nid for nid, n in pcb.nets.items() if n.name in set(names)]
    layers = list(getattr(pcb.board_info, 'copper_layers', []) or []) or ['F.Cu']

    # The corridor arithmetic a reader needs beside the ratios: one lane =
    # track + 2*clearance at the board's own floors (the 0.45mm number run
    # 23's forensics measured RN7's cage against).
    try:
        from list_nets import board_floor
        import routing_defaults as defaults
        clr, _s1 = board_floor(args.board, 'clearance', None,
                               defaults.CLEARANCE)
        trk, _s2 = board_floor(args.board, 'track_width', None,
                               defaults.TRACK_WIDTH)
    except Exception:                                           # noqa: BLE001
        clr = trk = None

    bins, _terminals = congestion_bins(pcb, ids, len(layers), args.bin)
    rows = []
    for (bx, by), (free, owners) in bins.items():
        ratio = len(owners) / free if free > 0 else float('inf')
        rows.append({
            'window': [round(bx * args.bin, 2), round(by * args.bin, 2),
                       round((bx + 1) * args.bin, 2),
                       round((by + 1) * args.bin, 2)],
            'demand_nets': len(owners),
            'free_area_mm2': round(free, 3),
            'ratio': round(ratio, 4),
            'nets': sorted(name_by_id.get(n, str(n)) for n in owners)[:12],
        })
    rows.sort(key=lambda r: -r['ratio'])
    # Simultaneous routability needs >= 2 nets by definition: a single-net
    # sliver (a bin inside one part's own pad field) tops any ratio ranking
    # and means nothing here. Reported in the JSON, never in the ranking.
    hot = [r for r in rows if r['demand_nets'] >= 2
           and (args.threshold is None or r['ratio'] > args.threshold)]

    # Parts per window, for the top rows only (the fix target is a ref).
    pads = [(pad.global_x, pad.global_y, pad.component_ref)
            for fp in pcb.footprints.values() for pad in fp.pads]
    for r in hot[:args.top]:
        w = r['window']
        r['refs'] = sorted({ref for x, y, ref in pads
                            if w[0] <= x < w[2] and w[1] <= y < w[3] and ref})

    lane = (f"one lane = track {trk:g} + 2 x clearance {clr:g} = "
            f"{trk + 2 * clr:g}mm" if clr is not None and trk is not None
            else "board floors unreadable")
    print(f"Pocket census of {args.board}: {len(ids)} demand net(s), "
          f"{len(bins)} window(s) at {args.bin}mm, {len(layers)} layer(s); "
          f"{lane}")
    print(f"REPORT-ONLY: nothing gates on these numbers. The boundary review "
          f"dispositions each window below, in writing.")
    for r in hot[:args.top]:
        w = r['window']
        print(f"  [{w[0]:g},{w[1]:g}]-[{w[2]:g},{w[3]:g}]  demand "
              f"{r['demand_nets']:>3} net(s) / free {r['free_area_mm2']:g}mm2 "
              f"= {r['ratio']:g}/mm2")
        print(f"      nets: {', '.join(r['nets'])}"
              + (" ..." if r['demand_nets'] > len(r['nets']) else ""))
        if r.get('refs'):
            print(f"      refs: {', '.join(r['refs'][:14])}"
                  + (" ..." if len(r['refs']) > 14 else ""))

    if args.json:
        doc = {'board': args.board, 'bin_mm': args.bin,
               'demand_nets': len(ids), 'layers': len(layers),
               'lane_mm': (round(trk + 2 * clr, 4)
                           if clr is not None and trk is not None else None),
               'threshold': args.threshold,
               'windows': rows[:max(args.top, 32)]}
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
        print(f"  JSON -> {args.json}")
    return 0


if __name__ == "__main__":
    import cli_banner
    cli_banner.install()   # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())
