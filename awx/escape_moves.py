"""The menu of realistic escape moves for a berth pad.

The braid's entry assignment hard-codes four ways into the destination
field -- a row-street run west, a dogbone in one of the two vertical
diagonal cells, a via-in-pad, and (only when the caller names it by
hand) a run around the north or south flank. Those are not the moves a
board offers; they are the moves this board needed. Everything they
share is that they go WEST, toward the corridor.

This module enumerates the moves a pad in a grid array actually has, so
the homotopy solver can CHOOSE rather than be handed one:

  surface     leave on the pad's own layer through the row gap (left /
              right) or the column gap (up / down). 0 vias.
  dogbone     45-degree stub into one of the FOUR diagonal inter-ball
              cells, via there, travel on another layer. 1 via.
  via_in_pad  via in the ball itself, travel on another layer. 1 via.

Each move reports the point where it leaves the array -- `exit_pt`, on
the array's bounding box -- and the layer it is travelling on when it
gets there. That pair is the whole interface to the corridor: the
corridor's job is to deliver the net to `exit_pt` on `layer`, and it
does not care how the pad got there.

Moves that go AWAY from the corridor are included deliberately: a deep
ball escaping to the far side and being met there is how the human's
reference routes its deep nets, and it is just another direction here.

The array's pitch and extent are measured from the footprint, never
passed in.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

Pt = Tuple[float, float]


@dataclass
class Grid:
    """The destination array's own geometry, measured not assumed."""
    xs: List[float]                 # sorted distinct pad columns
    ys: List[float]                 # sorted distinct pad rows
    pitch_x: float
    pitch_y: float
    bbox: Tuple[float, float, float, float]

    def col_of(self, x: float) -> int:
        return min(range(len(self.xs)), key=lambda i: abs(self.xs[i] - x))

    def row_of(self, y: float) -> int:
        return min(range(len(self.ys)), key=lambda i: abs(self.ys[i] - y))


@dataclass
class Move:
    """One way a pad can leave the array."""
    net: str
    kind: str                       # surface | dogbone | via_in_pad
    direction: str                  # left | right | up | down
    layer: str                      # layer travelled on at exit_pt
    exit_pt: Pt
    vias: int
    legs: List[Tuple[Pt, Pt, str]] = field(default_factory=list)
    site: Optional[Pt] = None       # via location, if any

    def __repr__(self) -> str:
        s = (f'{self.kind}/{self.direction}/{self.layer[0]} '
             f'exit=({self.exit_pt[0]:.2f},{self.exit_pt[1]:.2f}) '
             f'vias={self.vias}')
        if self.site:
            s += f' site=({self.site[0]:.2f},{self.site[1]:.2f})'
        return s


def grid_of(footprint) -> Grid:
    """Measure the array from its pads. A single row or column is fine;
    a non-grid part gives a degenerate Grid the caller can reject."""
    xs = sorted({round(p.global_x, 3) for p in footprint.pads})
    ys = sorted({round(p.global_y, 3) for p in footprint.pads})

    def _pitch(v):
        d = [b - a for a, b in zip(v, v[1:])]
        return min(d) if d else 0.0

    px, py = _pitch(xs), _pitch(ys)
    allx = [p.global_x for p in footprint.pads]
    ally = [p.global_y for p in footprint.pads]
    return Grid(xs, ys, px, py,
                (min(allx), min(ally), max(allx), max(ally)))


DIRS = {'left': (-1, 0), 'right': (1, 0), 'up': (0, -1), 'down': (0, 1)}
DIAGS = {'left': ((-1, -1), (-1, 1)), 'right': ((1, -1), (1, 1)),
         'up': ((-1, -1), (1, -1)), 'down': ((-1, 1), (1, 1))}


def enumerate_moves(pad, grid: Grid, layers: Sequence[str],
                    clear: Callable[[Pt, Pt, str], bool],
                    via_clear: Callable[[Pt, str], bool] = None,
                    margin: float = 0.0) -> List[Move]:
    """Every escape move this pad has. `clear(p, q, layer)` says whether
    a track from p to q on `layer` is free of foreign copper;
    `via_clear(p, layer)` whether a via barrel fits at p (checked on
    every layer by the caller). Moves whose geometry is blocked are not
    returned, so an empty list means this pad is boxed in."""
    net = getattr(pad, 'net_name', '') or ''
    net = net.split('/')[-1]
    px, py = pad.global_x, pad.global_y
    home = next((L for L in layers if L in pad.layers), layers[0])
    others = [L for L in layers if L != home]
    x0, y0, x1, y1 = grid.bbox
    hx, hy = grid.pitch_x / 2.0, grid.pitch_y / 2.0
    out: List[Move] = []

    def edge(direction: str) -> Pt:
        dx, dy = DIRS[direction]
        if dx:
            return (x0 - hx - margin if dx < 0 else x1 + hx + margin, py)
        return (px, y0 - hy - margin if dy < 0 else y1 + hy + margin)

    # --- surface: leave on the pad's own layer along an ADJACENT GAP.
    # Not along the pad's own row/column -- that is where the other
    # balls are, and running down it only ever works for an edge pad.
    # The escape stubs half a pitch into the gap between rows (for a
    # left/right escape) or between columns (up/down), then runs out.
    for d, (dx, dy) in DIRS.items():
        for sgn in (-1, 1):
            if dx:
                gate = (px, py + sgn * hy)          # into the row gap
                e = (edge(d)[0], gate[1])
            else:
                gate = (px + sgn * hx, py)          # into the column gap
                e = (gate[0], edge(d)[1])
            if clear((px, py), gate, home) and clear(gate, e, home):
                out.append(Move(net, 'surface', d, home, e, 0,
                                [((px, py), gate, home),
                                 (gate, e, home)]))

    # --- via_in_pad: dive where the pad is, leave on another layer
    for L in others:
        if via_clear and not all(via_clear((px, py), lay)
                                 for lay in layers):
            break
        for d in DIRS:
            e = edge(d)
            if clear((px, py), e, L):
                out.append(Move(net, 'via_in_pad', d, L, e, 1,
                                [((px, py), e, L)], site=(px, py)))

    # --- dogbone: 45 stub into a diagonal inter-ball cell, via, leave
    for d in DIRS:
        for (sx, sy) in DIAGS[d]:
            site = (px + sx * hx, py + sy * hy)
            if via_clear and not all(via_clear(site, lay) for lay in layers):
                continue
            if not clear((px, py), site, home):
                continue
            for L in others:
                e = edge(d)
                # the run leaves from the SITE, so its exit tracks the
                # site's own row/column, not the pad's
                e = (e[0], site[1]) if DIRS[d][0] else (site[0], e[1])
                if clear(site, e, L):
                    out.append(Move(net, 'dogbone', d, L, e, 1,
                                    [((px, py), site, home),
                                     (site, e, L)], site=site))
    return out


def summarise(moves: List[Move]) -> str:
    if not moves:
        return 'BOXED IN'
    by = {}
    for m in moves:
        by.setdefault(m.kind, []).append(m)
    return '  '.join(f'{k}:{len(v)}' for k, v in sorted(by.items()))
