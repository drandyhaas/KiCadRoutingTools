#!/usr/bin/env python3
"""Plane-fragility as a PLACEMENT score (#118 follow-up, run-7 wave).

The placement stack judges candidates by crossings/hpwl/inversions -- all
signal-net proxies. "Routing is as much about planes as traces" (#118): a
placement that fragments the ground pour or squeezes it through necks loses
at the PLANE step, and nothing in the candidate score sees it coming, while
the router-side #424 machinery already prices exact fill damage.

This module closes that half: pour the declared plane nets on a SCRATCH copy
of a candidate board (full-outline zones, KiCad ZONE_FILLER refill via
kicad_exact_fill -- the same truth #424 consumes), then run the #424
fragility field over the fill and reduce it to two placement-comparable
numbers:

    islands    how many pieces the pour lands in (fragmentation -- a
               categorical loss; the repair step will have to stitch)
    neck_sum   the fragility field summed over the fill, in mm-equivalents
               (necks are wall-to-wall high-cost cells; wide pour is ~free)

Both are meaningful RELATIVELY, candidate vs candidate on the same board --
absolute values scale with outline/boundary length. Returns None when no
KiCad python exists for the refill (the score is then honestly unavailable,
never guessed from drawn outlines).
"""
from __future__ import annotations
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

import math
import os
import shutil
import tempfile
from types import SimpleNamespace
from typing import Dict, List, Optional, Sequence, Tuple

REFERENCE_GRID_STEP = 0.1


def parse_plane_specs(tokens: Sequence[str], copper_layers: Sequence[str]):
    """['GND', '3V3:F.Cu'] -> [(net, layer)]; a bare net pours the LAST
    copper layer (the classic bottom ground pour)."""
    out = []
    for tok in tokens:
        if ':' in tok:
            net, layer = tok.split(':', 1)
        else:
            net, layer = tok, copper_layers[-1]
        out.append((net, layer))
    return out


def plane_fragility_score(board: str, specs: Sequence[Tuple[str, str]],
                          grid_step: float = 0.1,
                          keep_staged: Optional[str] = None) -> Optional[Dict]:
    """Score one board. `specs` is [(net_name, layer), ...].

    Returns {'islands', 'neck_sum', 'cells', 'per_net'} or None when the
    KiCad refill is unavailable (no pcbnew python on this machine).
    """
    from kicad_parser import board_uses_name_nets, parse_kicad_pcb
    from kicad_writer import generate_zone_sexpr
    from kicad_exact_fill import refill_islands

    pcb = parse_kicad_pcb(board)
    bounds = pcb.board_info.board_bounds
    if bounds is None:
        return None
    copper = list(pcb.board_info.copper_layers or ['F.Cu', 'B.Cu'])
    name_to_id = {n.name: nid for nid, n in pcb.nets.items()}
    with open(board, encoding='utf-8', errors='replace') as f:
        content = f.read()
    use_names = board_uses_name_nets(content)

    x0, y0, x1, y1 = bounds
    outline = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    blocks = []
    for i, (net, layer) in enumerate(specs):
        nid = name_to_id.get(net)
        if nid is None:
            continue
        blocks.append(generate_zone_sexpr(
            nid, net, layer, outline, use_net_name=use_names,
            # distinct priorities: overlapping same-priority zones tie-break
            # on UUID and the fill varies run to run (kicad_writer note)
            priority=i))
    if not blocks:
        return None

    tmp = tempfile.mkdtemp(prefix='plane_score_')
    try:
        staged = os.path.join(tmp, os.path.basename(board))
        idx = content.rindex(')')
        with open(staged, 'w', encoding='utf-8') as f:
            f.write(content[:idx] + '\n'.join(blocks) + '\n' + content[idx:])
        from copy_board import SIBLING_EXTS   # ONE list (#711)
        for ext in SIBLING_EXTS:
            sib = os.path.splitext(board)[0] + ext
            if os.path.isfile(sib):
                shutil.copyfile(sib, os.path.splitext(staged)[0] + ext)
        if keep_staged:
            shutil.copyfile(staged, keep_staged)

        fills = refill_islands(staged)
        if fills is None:
            return None

        wanted = {(net, layer) for net, layer in specs}
        per_net = {}
        for (net, layer), polys in fills.items():
            if (net, layer) in wanted:
                per_net[f'{net}:{layer}'] = len(polys)
        islands = sum(per_net.values())

        # The #424 field over the poured board. The staged pcb_data gets the
        # ALREADY-COMPUTED fill as its provider so the field prices the exact
        # fill without a second refill subprocess.
        from plane_fragility import compute_plane_fragility_cells
        pcb2 = parse_kicad_pcb(staged)
        pcb2.exact_fill_provider = lambda: fills
        config = SimpleNamespace(
            grid_step=grid_step, layers=copper,
            cell_cost=lambda mm: int(mm * 1000 / REFERENCE_GRID_STEP))
        cells = compute_plane_fragility_cells(pcb2, config)
        # cost units -> mm-equivalents (cell_cost inverse), summed
        neck_sum = float(cells[:, 3].sum()) * REFERENCE_GRID_STEP / 1000.0 \
            if len(cells) else 0.0
        return {'islands': islands,
                'neck_sum': round(neck_sum, 3),
                'cells': int(len(cells)),
                'per_net': per_net}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main(argv=None):
    import argparse
    import json as _json
    p = argparse.ArgumentParser(
        description="Plane-fragility placement score: pour the named nets "
                    "on a scratch copy and price the fill's necks/islands.")
    p.add_argument('board')
    p.add_argument('--plane-nets', nargs='+', required=True,
                   metavar='NET[:LAYER]',
                   help='nets to pour (bare NET pours the last copper layer)')
    p.add_argument('--grid-step', type=float, default=0.1)
    p.add_argument('--keep-staged', default=None)
    args = p.parse_args(argv)
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(args.board)
    specs = parse_plane_specs(args.plane_nets,
                              list(pcb.board_info.copper_layers
                                   or ['F.Cu', 'B.Cu']))
    score = plane_fragility_score(args.board, specs,
                                  grid_step=args.grid_step,
                                  keep_staged=args.keep_staged)
    if score is None:
        print("plane_score: unavailable (no KiCad python for the refill, "
              "no outline, or no named net exists)")
        return 3
    print("JSON_SUMMARY: " + _json.dumps(score, sort_keys=True))
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
