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
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

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


class PlaneScoreStatus(NamedTuple):
    """Why `plane_fragility_score_ex` produced a score, or did not (#713).

    `reason` is `'ok'`, one of this module's own two causes (`no_bounds`,
    `no_named_net`), or a `kicad_exact_fill.REFILL_REASONS` value forwarded
    from the refill.

    `uniform` is the property a candidate RANKING has to branch on, and it is
    the reason this type exists rather than a bare reason string.
    """
    reason: str
    detail: str = ''
    elapsed_s: Optional[float] = None

    @property
    def ok(self) -> bool:
        return self.reason == 'ok'

    @property
    def is_timeout(self) -> bool:
        return self.reason == 'timeout'

    @property
    def uniform(self) -> bool:
        """Would every candidate of a portfolio hit this same cause?

        `no_bounds` and `no_named_net` are facts about the board and the
        `--plane-score` spec, and a portfolio's candidates differ only in
        component POSES -- so if one candidate hits them, all of them do.
        `no_kicad_python` is a fact about the machine, likewise uniform.

        A `timeout` is NOT uniform: it can strike candidate 7 and spare
        candidate 1 on the same run. That asymmetry is what makes dropping
        the plane terms on a timeout a machine-speed decision rather than a
        capability one, and it is #713 item 1. `refill_failed`, `error` and
        `empty_fill` are per-candidate accidents and are non-uniform for the
        same reason -- `empty_fill` most clearly, since what KiCad manages to
        pour is exactly what moving the parts changes.
        """
        return self.reason in ('no_bounds', 'no_named_net', 'no_kicad_python')

    def why(self) -> str:
        base = {
            'ok': 'the plane score was computed',
            'no_bounds': 'the board declares no outline bounds',
            'no_named_net': ('none of the --plane-score nets exists on this '
                             'board'),
            'empty_fill': ('the KiCad refill poured no copper for any '
                           '--plane-score net'),
        }.get(self.reason)
        if base is None:
            return _refill_why(self)
        # `detail` is rendered for THIS module's own reasons too. The first
        # draft dropped it here while forwarded refill reasons kept theirs, so
        # `no_named_net` carefully built the list of nets it wanted and then
        # threw it away -- the one fact the reader needs to fix the flag.
        return f'{base} ({self.detail})' if self.detail else base


def _refill_why(status: 'PlaneScoreStatus') -> str:
    """Forward a refill reason to `RefillStatus.why()` rather than restating
    it -- one wording, so the two layers cannot drift apart."""
    from kicad_exact_fill import RefillStatus
    return RefillStatus(status.reason, status.detail, status.elapsed_s).why()


def plane_fragility_score(board: str, specs: Sequence[Tuple[str, str]],
                          grid_step: float = 0.1,
                          keep_staged: Optional[str] = None) -> Optional[Dict]:
    """The score alone. See `plane_fragility_score_ex` for why there is none."""
    return plane_fragility_score_ex(board, specs, grid_step=grid_step,
                                    keep_staged=keep_staged)[0]


def plane_fragility_score_ex(
        board: str, specs: Sequence[Tuple[str, str]],
        grid_step: float = 0.1,
        keep_staged: Optional[str] = None
) -> Tuple[Optional[Dict], PlaneScoreStatus]:
    """Score one board. `specs` is [(net_name, layer), ...].

    Returns ({'islands', 'neck_sum', 'cells', 'per_net'}, status) -- the score
    is None whenever `status.ok` is False, and the status says WHICH of the
    several causes it was. Before #713 every one of them was a bare None, so a
    caller could not tell a capability gap (the same answer on every candidate)
    from a 300 s timeout (this machine, this run).
    """
    from kicad_parser import board_uses_name_nets, parse_kicad_pcb
    from kicad_writer import generate_zone_sexpr
    from kicad_exact_fill import refill_islands_ex

    pcb = parse_kicad_pcb(board)
    bounds = pcb.board_info.board_bounds
    if bounds is None:
        return None, PlaneScoreStatus('no_bounds')
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
        return None, PlaneScoreStatus(
            'no_named_net',
            'wanted ' + ', '.join(f'{n}:{l}' for n, l in specs))

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

        fills, _rst = refill_islands_ex(staged)
        if fills is None:
            # Forwarded verbatim, reason and all: the caller's branch between
            # "strip the plane terms" and "refuse" turns on whether this was a
            # capability fact or a clock, and re-deriving it here would be a
            # second opinion that can drift from the first.
            return None, PlaneScoreStatus(_rst.reason, _rst.detail,
                                          _rst.elapsed_s)
        if not fills:
            # The refill RAN and poured nothing. Scoring it would report
            # `islands: 0` -- which is the OPTIMUM of a term that sorts before
            # hpwl -- while `neck_sum` came from drawn zone outlines via
            # compute_plane_fragility_cells' own fallback. This module's
            # docstring says the score is "never guessed from drawn outlines",
            # so an empty fill is no score at all rather than the best one.
            return None, PlaneScoreStatus('empty_fill', _rst.detail,
                                          _rst.elapsed_s)

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
        return ({'islands': islands,
                 'neck_sum': round(neck_sum, 3),
                 'cells': int(len(cells)),
                 'per_net': per_net},
                PlaneScoreStatus('ok', _rst.detail, _rst.elapsed_s))
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
