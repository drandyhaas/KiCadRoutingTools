#!/usr/bin/env python3
"""Signal quality after a route: detours, vias, vias in pads, pair coupling and pair names.

A board can pass DRC with every pad joined and still route its signals badly:
USB data at twice its straight line with the two halves apart, QSPI lines
through four vias each, vias dropped into SMD pads. This instrument reads one
routed board and reports, read-only:

  NETS          every signal net (rails, power-named nets and poured nets
                left out): routed track length over the length of the
                minimum spanning tree of its pads (the straight line for a
                two-pad net), its vias, and its vias in pads. A net is
                flagged past `--max-detour` (1.5) or `--max-vias` (2). The
                detour is judged on a tree of 3 mm or more only, where a pad
                escape does not decide it.
  VIAS IN PADS  every via whose barrel overlaps a same-net SMD pad
                (`fab_notes.via_in_pad_sites`, the fab note's own test), on
                any net: solder wicks down an open via whatever the net.
  DIFF PAIRS    every pair route_diff pairs (`net_queries.find_differential_pairs`):
                the coupled length of its track copper as KiCad's DRC counts
                it (`net_queries.diff_pair_copper`) over the longer member,
                with P and N at most `--pair-gap-max` apart (default the
                pair's declared diff_pair_gap max, else twice its class gap),
                the P/N skew, the vias, and each coupled layer's differential
                impedance at its measured width and gap
                (`impedance.calculate_impedance_for_layer`; needs a stackup).
                A routed pair is flagged below `--min-coupled` (0.8).
  PAIR NAMES    net names that read as one pair but that route_diff or KiCad
                does not pair (`net_queries.near_miss_diff_pairs`), each with
                the rename that makes both pair them.

    python3 -X utf8 py_tools/signal_review.py routed.kicad_pcb
    python3 -X utf8 py_tools/signal_review.py routed.kicad_pcb --nets 'USB*' 'QSPI*' --json signal.json

`--nets` limits every section to the nets it selects (route.py's glob rules),
and the text then lists every selected net, not only the flagged ones. Plane
layers carrying signal copper are route_review's. Exit codes: 0 reviewed
(findings or not), 2 unreadable input.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import statistics
import sys
from collections import defaultdict

from kicad_parser import parse_kicad_pcb, pour_zones  # noqa: E402
from placement.grouping import BoardView  # noqa: E402
from route_review import mst_length  # noqa: E402
from routing_utils import segment_length  # noqa: E402

#: Below this pad tree a net's detour is its pad escapes, not its routing.
DETOUR_MIN_TREE_MM = 3.0


def _pair_gap_window(rules, p_net_id, gap_max):
    from routing_defaults import DIFF_PAIR_GAP
    gap = rules.resolve('diff_pair_gap', rules.item_for_net(p_net_id, 'track'))
    lo = (gap.kicad_min if gap.kicad_min is not None else gap.min) or 0.0
    if gap_max is None:
        gap_max = gap.max if gap.max is not None else 2 * (gap.opt or lo or DIFF_PAIR_GAP)
    return lo, gap_max


def _zdiff_by_layer(pcb, tracks, gap_by_layer):
    from impedance import calculate_impedance_for_layer
    if not pcb.board_info.stackup:
        return {}
    out = {}
    for layer, gap in gap_by_layer.items():
        widths = [s.width for s in tracks if s.layer == layer]
        if not widths or gap <= 0:
            continue
        z = calculate_impedance_for_layer(pcb, layer, statistics.median(widths), spacing=gap)
        if z.get('zdiff'):
            out[layer] = round(z['zdiff'], 1)
    return out


def review(pcb_file, nets=None, max_detour=1.5, max_vias=2, min_coupled=0.8,
           pair_gap_max=None, pcb=None):
    """The review as a dict (the `--json` document)."""
    from design_rules import DesignRules
    from fab_notes import via_in_pad_sites
    from list_nets import power_name_kind
    from net_queries import (CopperIndex, diff_pair_copper, find_differential_pairs,
                             matches_net_filter, near_miss_diff_pairs)
    pcb = pcb if pcb is not None else parse_kicad_pcb(pcb_file)
    view = BoardView(pcb)
    name = view.name_of
    poured = {z.net_id for z in pour_zones(pcb.zones) if z.net_id}
    tracks = defaultdict(list)
    for s in pcb.segments:
        if s.net_id and not getattr(s, 'graphic', False):
            tracks[s.net_id].append(s)
    vias = defaultdict(int)
    for v in pcb.vias:
        vias[v.net_id] += 1

    def selected(nid):
        return matches_net_filter(name(nid), nets) if nets else True

    # ---- vias in SMD pads, every net
    in_pad = defaultdict(list)
    for via, pad in via_in_pad_sites(pcb.vias, pcb.pads_by_net):
        if selected(via.net_id):
            in_pad[via.net_id].append(f"{pad.component_ref}.{pad.pad_number}")

    # ---- per signal net
    rows = []
    for nid, pads in sorted((pcb.pads_by_net or {}).items(), key=lambda kv: name(kv[0])):
        if nid == 0 or len(pads) < 2 or not selected(nid):
            continue
        if not nets and (nid in view.rails or nid in poured or power_name_kind(name(nid))):
            continue
        routed = sum(segment_length(s) for s in tracks[nid])
        tree = mst_length([(p.global_x, p.global_y) for p in pads])
        detour = round(routed / tree, 2) if tree > 0.5 and routed else None
        flags = []
        if detour and tree >= DETOUR_MIN_TREE_MM and detour > max_detour:
            flags.append('detour')
        if vias[nid] > max_vias:
            flags.append('vias')
        rows.append({'net': name(nid), 'pads': len(pads), 'routed_mm': round(routed, 1),
                     'tree_mm': round(tree, 1), 'detour': detour, 'vias': vias[nid],
                     'vias_in_pads': len(in_pad.get(nid, ())),
                     'layers': sorted({s.layer for s in tracks[nid]}), 'flags': flags})
    rows.sort(key=lambda r: (-len(r['flags']), -(r['detour'] or 0), -r['vias'], r['net']))

    # ---- differential pairs, by route_diff's own pairing
    pairs = []
    rules = index = None
    found = find_differential_pairs(pcb, nets or ['*'])
    for base, pr in sorted(found.items()):
        if rules is None:
            rules, index = DesignRules.from_project(pcb, pcb_file), CopperIndex(pcb)
        lo, hi = _pair_gap_window(rules, pr.p_net_id, pair_gap_max)
        rec = diff_pair_copper(pcb, pr.p_net_id, pr.n_net_id, lo, hi, index=index)
        longer = max(rec['length_p_mm'], rec['length_n_mm'])
        frac = round(rec['coupled_mm'] / longer, 3) if longer > 0 else None
        pairs.append({'pair': base, 'p_net': pr.p_net_name, 'n_net': pr.n_net_name,
                      'length_p_mm': round(rec['length_p_mm'], 2),
                      'length_n_mm': round(rec['length_n_mm'], 2),
                      'coupled_mm': round(rec['coupled_mm'], 2), 'coupled_fraction': frac,
                      'coupled_by_layer': rec['coupled_by_layer'],
                      'gap_by_layer': rec['gap_by_layer'],
                      'gap_window_mm': [lo, round(hi, 4)],
                      'skew_mm': rec['skew_mm'],
                      'vias': [vias[pr.p_net_id], vias[pr.n_net_id]],
                      'zdiff_by_layer': _zdiff_by_layer(
                          pcb, tracks[pr.p_net_id] + tracks[pr.n_net_id], rec['gap_by_layer']),
                      'flags': ['unrouted'] if frac is None else
                               ['uncoupled'] if frac < min_coupled else []})

    # ---- names that read as a pair and do not pair
    names = [n.name for n in pcb.nets.values() if n.name]
    misses = [m for m in near_miss_diff_pairs(names)
              if not nets or any(matches_net_filter(m[k], nets) for k in ('p', 'n'))]

    return {'board': pcb_file, 'nets_filter': nets or [],
            'thresholds': {'max_detour': max_detour, 'max_vias': max_vias,
                           'min_coupled': min_coupled,
                           'detour_min_tree_mm': DETOUR_MIN_TREE_MM},
            'nets': rows,
            'vias_in_pads': [{'net': name(nid), 'count': len(p), 'pads': sorted(set(p))}
                             for nid, p in sorted(in_pad.items(), key=lambda kv: -len(kv[1]))],
            'diff_pairs': pairs, 'near_miss_pairs': misses,
            'counts': {'nets': len(rows), 'flagged_nets': sum(1 for r in rows if r['flags']),
                       'vias_in_pads': sum(len(p) for p in in_pad.values()),
                       'nets_with_vias_in_pads': len(in_pad),
                       'diff_pairs': len(pairs),
                       'uncoupled_pairs': sum(1 for p in pairs if 'uncoupled' in p['flags']),
                       'unrouted_pairs': sum(1 for p in pairs if 'unrouted' in p['flags']),
                       'near_miss_pairs': len(misses)}}


def _print(r):
    c, t = r['counts'], r['thresholds']
    print(f"signal_review: {c['flagged_nets']} of {c['nets']} net(s) flagged "
          f"(detour > {t['max_detour']}x, vias > {t['max_vias']}); "
          f"{c['vias_in_pads']} via(s) in SMD pads on {c['nets_with_vias_in_pads']} net(s); "
          f"{c['uncoupled_pairs']} of {c['diff_pairs']} diff pair(s) below "
          f"{t['min_coupled']:.0%} coupled, {c['unrouted_pairs']} unrouted; "
          f"{c['near_miss_pairs']} pair name(s) that do not pair")
    shown = r['nets'] if r['nets_filter'] else [n for n in r['nets'] if n['flags']]
    if shown:
        print("== nets (routed / pad tree)" if r['nets_filter'] else "== flagged nets (routed / pad tree)")
        for n in shown:
            d = f"x{n['detour']:4.2f}" if n['detour'] else '   - '
            print(f"   {n['net'][:32]:32s} {d}  routed {n['routed_mm']:6.1f} mm  tree {n['tree_mm']:6.1f}"
                  f"  vias {n['vias']:2d}  in pads {n['vias_in_pads']:2d}  {','.join(n['layers'])}"
                  + (f"  [{', '.join(n['flags'])}]" if n['flags'] else ''))
    if r['vias_in_pads']:
        print("== vias in SMD pads (barrel over a same-net SMD pad)")
        for v in r['vias_in_pads']:
            more = len(v['pads']) - 10
            print(f"   {v['net'][:32]:32s} {v['count']:3d}  {', '.join(v['pads'][:10])}"
                  + (f", +{more} more" if more > 0 else ''))
    if r['diff_pairs']:
        print("== diff pairs (coupled track over the longer member, in the diff_pair_gap window)")
        for p in r['diff_pairs']:
            if 'unrouted' in p['flags']:
                print(f"   {p['p_net']} / {p['n_net']}: no track copper  [unrouted]")
                continue
            z = ', '.join(f"{k} {v:g} ohm" for k, v in p['zdiff_by_layer'].items())
            skew = f"{p['skew_mm']:.2f} mm" if p['skew_mm'] is not None else '-'
            print(f"   {p['p_net']} / {p['n_net']}: coupled {p['coupled_mm']:.1f} of "
                  f"{max(p['length_p_mm'], p['length_n_mm']):.1f} mm ({p['coupled_fraction']:.0%}), "
                  f"gap {p['gap_window_mm'][0]:g}-{p['gap_window_mm'][1]:g}, skew {skew}, "
                  f"vias {p['vias'][0]}/{p['vias'][1]}" + (f", Zdiff {z}" if z else '')
                  + (f"  [{', '.join(p['flags'])}]" if p['flags'] else ''))
    if r['near_miss_pairs']:
        from net_queries import near_miss_text
        print("== pair names that do not pair")
        for m in r['near_miss_pairs']:
            print(f"   {near_miss_text(m)}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board', help='the routed .kicad_pcb')
    ap.add_argument('--nets', nargs='+', metavar='PATTERN',
                    help="review only these nets (globs, e.g. 'USB*' 'QSPI*')")
    ap.add_argument('--max-detour', type=float, default=1.5,
                    help='routed / pad-tree length that flags a net (default 1.5)')
    ap.add_argument('--max-vias', type=int, default=2, help='vias that flag a net (default 2)')
    ap.add_argument('--min-coupled', type=float, default=0.8,
                    help='coupled fraction under which a pair is flagged (default 0.8)')
    ap.add_argument('--pair-gap-max', type=float, default=None,
                    help='widest P/N gap that counts as coupled, mm (default: the '
                         "pair's diff_pair_gap max, else twice its class gap)")
    ap.add_argument('--json', help='write the review here')
    a = ap.parse_args(argv)
    try:
        pcb = parse_kicad_pcb(a.board)
    except (OSError, ValueError) as e:
        print(f"signal_review: cannot read {a.board}: {e}", file=sys.stderr)
        return 2
    r = review(a.board, nets=a.nets, max_detour=a.max_detour, max_vias=a.max_vias,
               min_coupled=a.min_coupled, pair_gap_max=a.pair_gap_max, pcb=pcb)
    _print(r)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(r, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
