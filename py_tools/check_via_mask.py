#!/usr/bin/env python3
"""Vias whose copper ring reaches into, or crowds, a pad's solder-mask opening (#1232).

Two categories, per via side (F.Mask for a via reaching F.Cu, B.Mask likewise):

  via-mask-contact  the via's ring (radius size/2) touches or overlaps a pad's
                    solder-mask opening, the via's centre outside that pad's
                    copper. Solder on the pad reaches the via. Any net counts.
                    A via already graded `via-in-paste` by check_drc (barrel in
                    a paste opening of its own net, fab_notes.via_paste_sites)
                    is left to that grade and not reported here, and a via
                    whose centre is in the pad is via-in-pad, not this.
  via-mask-web      a via PLUGGED with soldermask on that side (KiCad
                    `plugging`, resolved per side by
                    fab_notes.effective_via_protection) whose ring is closer
                    to a pad mask opening than --plug-web (default
                    fab_tiers.JLC_PLUGGED_VIA_MASK_WEB_MM). A contact is
                    reported once, as via-mask-contact.

Pad mask openings are mask_apertures.pad_mask_openings: the pad's copper grown
by KiCad's resolved solder-mask expansion.

--baseline BOARD accepts a finding the baseline board already had: the same
category at the same pad and side, from a via on the same net NAME within half
the smaller via diameter (fab_notes' rule for "the input's own via"). Accepted
findings are counted and listed, never reported as findings.

Read-only. Exit 0 when nothing is found, 1 when anything is.

    python3 py_tools/check_via_mask.py board.kicad_pcb [--baseline INPUT]
                                       [--plug-web MM] [--json PATH]
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import json
import math
import sys
from typing import Dict, List, Optional

CATEGORIES = ('via-mask-contact', 'via-mask-web')
_SIDES = (('F.Mask', 'F.Cu', 'front'), ('B.Mask', 'B.Cu', 'back'))


def via_mask_findings(pcb_data, plug_web: Optional[float] = None) -> List[Dict]:
    """Every via-mask finding on `pcb_data`, as dicts with category, net,
    via (x, y, size), side, pad, pad_net and distance_mm (ring edge to the
    opening edge; 0 for a contact)."""
    from fab_notes import effective_via_protection, via_paste_sites
    from fab_tiers import JLC_PLUGGED_VIA_MASK_WEB_MM
    from geometry_utils import via_on_layer
    from mask_apertures import pad_mask_openings, opening_distance, side_enabled
    from check_drc import point_to_pad_distance

    web = JLC_PLUGGED_VIA_MASK_WEB_MM if plug_web is None else float(plug_web)
    setup = getattr(pcb_data.board_info, 'via_protection_setup', None) or {}
    in_paste = {id(v) for v, _a, _p in via_paste_sites(pcb_data.vias, pcb_data)}
    openings = pad_mask_openings(pcb_data)
    reach = max(web, 0.0)
    out = []
    for via in pcb_data.vias:
        r = (via.size or 0.6) / 2.0
        eff = effective_via_protection(getattr(via, 'tenting_attrs', None), setup)
        net = pcb_data.nets.get(via.net_id)
        net_name = net.name if net is not None else ''
        for mask, copper, side in _SIDES:
            if not via_on_layer(via, copper):
                continue
            plugged = side_enabled(eff.get('plugging'), side)
            for op in openings:
                if op.layer != mask:
                    continue
                d = opening_distance(via.x, via.y, op) - r
                if d > reach:
                    continue
                pad = op.pad
                contact = d <= 1e-6
                if contact:
                    if id(via) in in_paste:
                        continue
                    if point_to_pad_distance(via.x, via.y, pad) <= 0.0:
                        continue
                    cat = 'via-mask-contact'
                elif plugged and d < web:
                    cat = 'via-mask-web'
                else:
                    continue
                out.append({
                    'category': cat, 'net': net_name,
                    'x': via.x, 'y': via.y, 'size': via.size, 'side': mask,
                    'pad': '%s.%s' % (pad.component_ref, pad.pad_number),
                    'pad_net': pad.net_name or '',
                    'distance_mm': round(max(d, 0.0), 4)})
    out.sort(key=lambda f: (f['category'], f['net'], f['x'], f['y'], f['side'], f['pad']))
    return out


def split_inherited(findings: List[Dict], baseline: List[Dict]):
    """(new, inherited): a finding is inherited when the baseline has the same
    category, side and pad from a via on the same net name within half the
    smaller via diameter."""
    new, inherited = [], []
    for f in findings:
        hit = any(b['category'] == f['category'] and b['side'] == f['side']
                  and b['pad'] == f['pad'] and b['net'] == f['net']
                  and math.hypot(b['x'] - f['x'], b['y'] - f['y'])
                  <= max(1e-3, min(b['size'] or 0.6, f['size'] or 0.6) / 2.0)
                  for b in baseline)
        (inherited if hit else new).append(f)
    return new, inherited


def counts(findings: List[Dict]) -> Dict[str, int]:
    return {c: sum(1 for f in findings if f['category'] == c) for c in CATEGORIES}


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Vias whose ring reaches into, or (when plugged) crowds, "
                    "a pad's solder-mask opening. Read-only.")
    ap.add_argument('pcb', help='Board to check')
    ap.add_argument('--baseline', metavar='BOARD', default=None,
                    help='Accept findings this board (e.g. the unrouted input) '
                         'already had')
    ap.add_argument('--plug-web', type=float, default=None, metavar='MM',
                    help='Minimum ring-to-opening distance for a soldermask-'
                         'plugged via (default: fab_tiers.'
                         'JLC_PLUGGED_VIA_MASK_WEB_MM)')
    ap.add_argument('--json', metavar='PATH', default=None,
                    help='Write {"counts", "findings", "inherited"} here')
    ap.add_argument('--max-print', type=int, default=20,
                    help='Findings printed per category (<=0 prints all)')
    args = ap.parse_args(argv)

    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(args.pcb)
    findings = via_mask_findings(pcb, args.plug_web)
    inherited = []
    if args.baseline:
        base = via_mask_findings(parse_kicad_pcb(args.baseline), args.plug_web)
        findings, inherited = split_inherited(findings, base)

    by = counts(findings)
    for cat in CATEGORIES:
        items = [f for f in findings if f['category'] == cat]
        print(f"  {cat}: {by[cat]}")
        limit = len(items) if args.max_print <= 0 else args.max_print
        for f in items[:limit]:
            extra = '' if cat == 'via-mask-contact' else f", {f['distance_mm']:.3f} mm"
            print(f"    net {f['net']} via ({f['x']:.4f}, {f['y']:.4f}) "
                  f"{f['side']}: pad {f['pad']} ({f['pad_net']}){extra}")
        if len(items) > limit:
            print(f"    ... and {len(items) - limit} more (use --max-print 0 to show all)")
    if args.baseline:
        print(f"  accepted (the baseline had them): {len(inherited)}")
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump({'counts': by, 'findings': findings,
                       'inherited': inherited}, fh, indent=1)
    return 1 if findings else 0


if __name__ == '__main__':
    sys.exit(main())
