#!/usr/bin/env python3
"""Net blockage/connectivity forensics (#424 debug machinery).

For each requested net on a board, reports to stdout (and --log FILE):
  1. ISLANDS: connected components of the net's copper (segments/vias/pads
     joined by touching endpoints / pad proximity), which pads sit in each,
     island sizes and layers.
  2. GAPS: for the two largest islands, the closest approach between them
     (the unclosed MST edge, endpoint coordinates and distance).
  3. WALLS: the foreign-copper inventory around each gap endpoint within
     --radius (default 1.0mm): net name, item kind, layer, distance --
     sorted nearest-first, so the boxing copper is named per layer.
  4. HISTORY (--route-log): the net's routing/rip/restore trail grepped
     from a chain or step log (MST edge attempts, stuck iterations,
     Blocking obstacles, rip ladder events, rescue outcomes).

Usage:
  python3 net_forensics.py board.kicad_pcb --nets BT_PCM_SYNC USB_D_P \
      [--radius 1.0] [--route-log step11.log] [--log forensics.txt] \
      [--json-out forensics.json]

Born from the 2026-07-21 ottercast pocket forensics: every residual
failure traced to escape-stub tips boxed on all four layers by neighbors'
destination traffic; this tool automates that diagnosis.
"""

from __future__ import annotations
import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import math
import re
import sys
from collections import defaultdict


# The island walk and the island-to-island gap now live in `list_nets`, where
# `check_reachability`'s auto-widen can reach them: it used to import
# `_components` out of THIS file, a sibling CLI's private name from a directory
# `_path` does not put on sys.path.
from list_nets import net_islands, island_gap    # noqa: E402


def _describe(pcb, net, radius, out):
    """Print the islands/gap/walls report; RETURN the islands it walked.

    Returned rather than re-walked: `net_data` wants the same components, and
    calling `net_islands` again for it walked every net twice on a --json-out
    run. The gap loop was de-duplicated first and this is the other half.
    """
    comps = net_islands(pcb, net.net_id)
    out(f"\n=== {net.name} (net {net.net_id}): {len(comps)} island(s) ===")
    for i, c in enumerate(comps):
        pads = [f"{o.component_ref}.{o.pad_number}" for k, o, _ in c if k == 'pad']
        layers = sorted({o.layer for k, o, _ in c if k == 'seg'})
        out(f"  island {i}: {sum(1 for k,_,_ in c if k=='seg')} segs, "
            f"{sum(1 for k,_,_ in c if k=='via')} vias, pads={pads} layers={layers}")
    if len(comps) < 2:
        out("  (fully connected)")
        return comps
    # gap between the two largest islands -- ONE implementation, shared with
    # net_data(), which had a byte-for-byte copy of this loop written as a dict
    # builder. Two copies of a measurement drift, and only one of them prints.
    d, p1, p2 = island_gap(comps[0], comps[1])
    out(f"  GAP island0<->island1: {d:.2f}mm  from ({p1[0]:.2f},{p1[1]:.2f}) "
        f"to ({p2[0]:.2f},{p2[1]:.2f})")
    for label, (gx, gy) in (('island0-end', p1), ('island1-end', p2)):
        inv = defaultdict(list)
        for s in pcb.segments:
            if s.net_id == net.net_id:
                continue
            dd = min(math.hypot(s.start_x - gx, s.start_y - gy),
                     math.hypot(s.end_x - gx, s.end_y - gy),
                     math.hypot((s.start_x + s.end_x) / 2 - gx,
                                (s.start_y + s.end_y) / 2 - gy))
            if dd < radius:
                nm = pcb.nets[s.net_id].name if s.net_id in pcb.nets else str(s.net_id)
                inv[nm].append((round(dd, 2), f"seg {s.layer}"))
        for v in pcb.vias:
            if v.net_id != net.net_id and math.hypot(v.x - gx, v.y - gy) < radius:
                nm = pcb.nets[v.net_id].name if v.net_id in pcb.nets else str(v.net_id)
                inv[nm].append((round(math.hypot(v.x - gx, v.y - gy), 2), "via ALL"))
        for fp in pcb.footprints.values():
            for p in fp.pads:
                if p.net_id == net.net_id:
                    continue
                dd = math.hypot(p.global_x - gx, p.global_y - gy)
                if dd < radius:
                    nm = (pcb.nets[p.net_id].name if p.net_id in pcb.nets
                          else f"net{p.net_id}")
                    kind = 'PTH' if p.drill else p.layers[0] if p.layers else '?'
                    inv[nm].append((round(dd, 2),
                                    f"pad {fp.reference}.{p.pad_number} {kind}"))
        out(f"  WALL around {label} ({gx:.2f},{gy:.2f}), r={radius}mm:")
        for nm in sorted(inv, key=lambda n: min(x[0] for x in inv[n]))[:8]:
            entries = sorted(inv[nm])[:3]
            out(f"    {nm}: " + ", ".join(f"{k}@{d}mm" for d, k in entries))
    return comps


def _history(net_name, log_path, out):
    pat = re.compile(r'\x1b\[[0-9;]*m')
    keep = re.compile(r'MST edge|stuck|Blocking obstacles|Ripping|Ripped|'
                      r'restor|RESTORE|RELOCATION|rescue|Retry|FAILED|abandon|'
                      r'Coverage:', re.I)
    out(f"\n--- history of {net_name} in {log_path} ---")
    n = 0
    try:
        with open(log_path, errors='replace') as f:
            in_section = False
            for line in f:
                line = pat.sub('', line.rstrip())
                if net_name in line:
                    in_section = True
                elif in_section and line.startswith(('[', '=')) and net_name not in line:
                    in_section = False
                if (net_name in line or in_section) and keep.search(line):
                    out(f"  {line.strip()[:150]}")
                    n += 1
                    if n > 60:
                        out("  ... (truncated)")
                        return
    except OSError as e:
        out(f"  (log unreadable: {e})")


def net_data(pcb, net, comps=None):
    """The islands-and-gap facts as DATA (run-23): what _describe prints.

    Exists because this tool was text-only, so run 23's teammates re-derived
    the island gap by hand to feed their rip-set decisions -- the number
    (/TXLED's 20.27mm span) lived in a paragraph nothing could consume.

    `comps` takes the islands `_describe` already walked. Without it this
    walked every net a second time on the one kind of run that asks for it.
    """
    comps = net_islands(pcb, net.net_id) if comps is None else comps
    doc = {'net': net.name, 'net_id': net.net_id, 'islands': [], 'gap': None}
    for c in comps:
        doc['islands'].append({
            'segs': sum(1 for k, _, _ in c if k == 'seg'),
            'vias': sum(1 for k, _, _ in c if k == 'via'),
            'pads': [f"{o.component_ref}.{o.pad_number}"
                     for k, o, _ in c if k == 'pad'],
            'layers': sorted({o.layer for k, o, _ in c if k == 'seg'})})
    if len(comps) >= 2:
        d, p1, p2 = island_gap(comps[0], comps[1])
        doc['gap'] = {'mm': round(d, 3),
                      'from': [round(p1[0], 3), round(p1[1], 3)],
                      'to': [round(p2[0], 3), round(p2[1], 3)]}
    return doc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board')
    ap.add_argument('--nets', nargs='+', required=True)
    ap.add_argument('--radius', type=float, default=1.0)
    ap.add_argument('--route-log', action='append', default=[])
    ap.add_argument('--log', help='also append the report to this file')
    # --json-out, not --json: every other tool in this repo that writes a JSON
    # document to a path spells it --json-out (route.py, converge.py,
    # place_route_loop.py, pose_score.py), and in check_reachability --json
    # means "print the document on stdout". Nothing in the tree called the
    # flag by its old name, so there is no alias to keep.
    ap.add_argument('--json-out', metavar='PATH', default=None,
                    help='also write {nets: [islands+gap docs]} here '
                         '(run-23: the gap number was text-only and had to '
                         'be re-derived by hand to feed rip-set decisions)')
    args = ap.parse_args()

    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(args.board)
    sink = open(args.log, 'a') if args.log else None

    def out(line):
        print(line)
        if sink:
            sink.write(line + '\n')

    docs = []
    out(f"# net_forensics: {args.board}")
    for nm in args.nets:
        net = next((x for x in pcb.nets.values() if x.name == nm), None)
        if net is None:
            out(f"\n=== {nm}: NOT FOUND ===")
            if args.json_out:
                docs.append({'net': nm, 'error': 'not found'})
            continue
        # Only when someone is going to read it, and reusing the islands
        # `_describe` just walked. Calling `net_data` unconditionally, on its
        # own fresh walk, made every run pay twice -- and every text-only run,
        # which is nearly all of them, threw the second walk away.
        _comps = _describe(pcb, net, args.radius, out)
        if args.json_out:
            docs.append(net_data(pcb, net, _comps))
        for lp in args.route_log:
            _history(nm, lp, out)
    if args.json_out:
        import json as _json
        with open(args.json_out, 'w', encoding='utf-8') as f:
            _json.dump({'board': args.board, 'nets': docs}, f, indent=1,
                       sort_keys=True)
        out(f"  JSON -> {args.json_out}")
    if sink:
        sink.close()


if __name__ == '__main__':
    main()
