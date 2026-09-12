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

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

Pt = Tuple[float, float]


@dataclass
class Grid:
    """The destination array's own geometry, measured not assumed."""
    xs: List[float]                 # sorted distinct pad columns
    ys: List[float]                 # sorted distinct pad rows
    pitch_x: float
    pitch_y: float
    bbox: Tuple[float, float, float, float]
    # a BLOCK of a banded array (blocks_of): its own pads, the whole
    # array's bbox, and its cell (column run, row run) in the split
    pads: list = field(default_factory=list)
    array_bbox: Optional[Tuple[float, float, float, float]] = None
    cell: Tuple[int, int] = (0, 0)



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
    climb: int = 0                  # rows/columns the run travels ALONG the
                                    # array before leaving (enumerate_moves climb=)
    walk: int = 0                   # pitches the SURFACE stub walks along a lane
                                    # from its diagonal elbow to the via site
                                    # (enumerate_moves walk=; the human's DU1)
    off_array: bool = False         # the via site lies OUTSIDE the ball field,
                                    # beside the array (enumerate_moves walk_off=)

    def __repr__(self) -> str:
        s = (f'{self.kind}/{self.direction}/{self.layer[0]} '
             f'exit=({self.exit_pt[0]:.2f},{self.exit_pt[1]:.2f}) '
             f'vias={self.vias}')
        if self.site:
            s += f' site=({self.site[0]:.2f},{self.site[1]:.2f})'
        if self.off_array:
            s += ' off-array'
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


def _runs(vals: List[float], pitch: float, gap_ratio: float) -> List[int]:
    """Run index per sorted value: a new run starts after a gap wider
    than `gap_ratio` pitches."""
    run, out = 0, []
    for i, v in enumerate(vals):
        if i and v - vals[i - 1] > gap_ratio * pitch:
            run += 1
        out.append(run)
    return out


def blocks_of(footprint, gap_ratio: float = 1.5) -> List[Grid]:
    """The SUB-ARRAYS of a footprint whose ball grid has depopulated
    bands, each measured as a Grid of its own (rows, columns, bbox) with
    its pads. A DDR3/DDR4 FBGA is two blocks of three ball columns either
    side of a three-pitch-wide empty band; fanned out as one array the
    band is interior and no stub may end in it, fanned out as two the
    band is a face of each block -- the street between the blocks that
    the human's riders descend into. A gap between consecutive pad rows
    or columns wider than `gap_ratio` pitches is a band; the blocks are
    the occupied cells of the (column runs) x (row runs) product, in
    file order of first pad. A solid array is ONE block whose Grid equals
    grid_of's in every measured field. Nothing here names a part or a
    pitch: the bands are read off the pads."""
    g = grid_of(footprint)
    if not g.xs or not g.ys or g.pitch_x <= 0 or g.pitch_y <= 0:
        return [g]
    rx = dict(zip(g.xs, _runs(g.xs, g.pitch_x, gap_ratio)))
    ry = dict(zip(g.ys, _runs(g.ys, g.pitch_y, gap_ratio)))
    cells: dict = {}
    for p in footprint.pads:
        k = (rx[round(p.global_x, 3)], ry[round(p.global_y, 3)])
        cells.setdefault(k, []).append(p)
    if len(cells) == 1:
        g.pads = list(footprint.pads)
        g.array_bbox = g.bbox
        return [g]
    out = []
    for k in sorted(cells):
        ps = cells[k]
        xs = sorted({round(p.global_x, 3) for p in ps})
        ys = sorted({round(p.global_y, 3) for p in ps})
        out.append(Grid(xs, ys, g.pitch_x, g.pitch_y,
                        (min(p.global_x for p in ps), min(p.global_y for p in ps),
                         max(p.global_x for p in ps), max(p.global_y for p in ps)),
                        pads=ps, array_bbox=g.bbox, cell=k))
    return out


def block_of(pad, blocks: List[Grid]) -> Grid:
    """The block a pad belongs to (by position; the pad may come from
    another parse of the same board)."""
    if len(blocks) == 1:
        return blocks[0]
    k = (round(pad.global_x, 3), round(pad.global_y, 3))
    for b in blocks:
        if any((round(p.global_x, 3), round(p.global_y, 3)) == k for p in b.pads):
            return b
    x, y = pad.global_x, pad.global_y
    return min(blocks, key=lambda b: max(b.bbox[0] - x, x - b.bbox[2],
                                         b.bbox[1] - y, y - b.bbox[3]))


def bands_of(blocks: List[Grid]) -> List[Tuple[float, float, float, float]]:
    """The empty strips between adjacent blocks, as boxes on the ball
    lines that bound them: (x0, y0, x1, y1). A band between two blocks
    stacked in y spans their common x extent from the upper block's
    last row to the lower block's first; likewise in x."""
    out = []
    for a in blocks:
        for b in blocks:
            if a is b:
                continue
            if a.cell[0] == b.cell[0] and b.cell[1] == a.cell[1] + 1:
                out.append((max(a.bbox[0], b.bbox[0]), a.bbox[3],
                            min(a.bbox[2], b.bbox[2]), b.bbox[1]))
            elif a.cell[1] == b.cell[1] and b.cell[0] == a.cell[0] + 1:
                out.append((a.bbox[2], max(a.bbox[1], b.bbox[1]),
                            b.bbox[0], min(a.bbox[3], b.bbox[3])))
    return out


DIRS = {'left': (-1, 0), 'right': (1, 0), 'up': (0, -1), 'down': (0, 1)}
LAYERS = ('F.Cu', 'B.Cu')
# escape_moves owns both: it imports nothing of ours, so every module
# can take them from here instead of keeping its own copy


def site_contention(menu: Dict[str, List[Move]], reach: float) -> Dict[str, Dict[Tuple[float, float], int]]:
    """How many OTHER nets want the room each via site takes.

    A barrel does not just cost a via -- it takes an inter-ball site, and
    under a ball field those sites are the scarcest room on the board:
    every escape that wanted that site must now go round. The selection
    cost already believes this about a move's TRACK ("the move's own run
    occupies a channel INSIDE the array, which is scarcer than corridor
    length", select_moves.cost) and charges its length; the VIA was free
    wherever it sat.

    Contention is read off the menus themselves, so it needs no model of
    the array: a site's contention is the number of other nets that have
    some move wanting a site within `reach` of it. Measured on DU1 at
    K41 (763 distinct sites, 41 nets): a site INSIDE the ball field is
    wanted by **6.14** other nets on average, one OUTSIDE it by **2.37**.
    That 4-net difference is what the human buys by putting the corner
    nets' vias outside the array, and what our cost could not see."""
    cell = max(reach, 1e-6)
    buckets: Dict[Tuple[int, int], List[Tuple[float, float, str]]] = {}
    sites: Dict[str, set] = {}
    for nm, ms in menu.items():
        seen = set()
        for m in ms:
            if not m.site:
                continue
            key = (round(m.site[0], 3), round(m.site[1], 3))
            if key in seen:
                continue
            seen.add(key)
            buckets.setdefault((int(key[0] // cell), int(key[1] // cell)),
                               []).append((key[0], key[1], nm))
        sites[nm] = seen
    out: Dict[str, Dict[Tuple[float, float], int]] = {}
    for nm, seen in sites.items():
        d = {}
        for (sx, sy) in seen:
            cx, cy = int(sx // cell), int(sy // cell)
            others = set()
            for ix in (cx - 1, cx, cx + 1):
                for iy in (cy - 1, cy, cy + 1):
                    for (tx, ty, on) in buckets.get((ix, iy), ()):
                        if on != nm and (tx - sx) ** 2 + (ty - sy) ** 2 < reach * reach:
                            others.add(on)
            d[(sx, sy)] = len(others)
        out[nm] = d
    return out


def enumerate_moves(pad, grid: Grid, layers: Sequence[str],
                    clear: Callable[[Pt, Pt, str], bool],
                    via_clear: Callable[[Pt, str], bool] = None,
                    margin: float = 0.0, climb: int = 0,
                    walk: int = 0, walk_off: int = 0) -> List[Move]:
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

    def exit_from(site: Pt, direction: str) -> Pt:
        """Where a run leaving `site` in `direction` ends: the array's own
        edge line, but never nearer than half a pitch plus the margin from
        the site ITSELF. For a site inside the field that is the edge line
        exactly (so this is a no-op for every in-array move); for a site
        already outside it, the edge line is behind the via and the run
        would be a few microns long -- which the braid reads as no tooth
        at all (the K28 SODT0 lesson, recorded on the dogbone above)."""
        e = edge(direction)
        dx, dy = DIRS[direction]
        if dx:
            far = site[0] + dx * (hx + margin)
            return (max(e[0], far) if dx > 0 else min(e[0], far), site[1])
        far = site[1] + dy * (hy + margin)
        return (site[0], max(e[1], far) if dy > 0 else min(e[1], far))

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
                if not (y0 < gate[1] < y1):
                    continue        # the gap outside the outer row is no gap
            else:
                gate = (px + sgn * hx, py)          # into the column gap
                e = (gate[0], edge(d)[1])
                if not (x0 < gate[0] < x1):
                    continue        # the gap outside the outer column is no gap
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

    # --- dogbone: 45 stub into a diagonal inter-ball cell, via, leave.
    # The SITE and the exit DIRECTION are independent: a via in the
    # east diagonal can still be met by a run heading west, which is
    # exactly what the old code's westward B approach did. Tying the
    # site to the direction hid half the options.
    for (sx, sy) in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
        site = (px + sx * hx, py + sy * hy)
        # the site must be an INTER-ball gap: an edge ball's outward
        # diagonal lies on the boundary line, where the run from the via
        # to the exit is a few microns long and the braid reads no tooth
        # (K28 SODT0: a corridor of one net, refused)
        if not (x0 < site[0] < x1 and y0 < site[1] < y1):
            continue
        if via_clear and not all(via_clear(site, lay) for lay in layers):
            continue
        if not clear((px, py), site, home):
            continue
        for d in DIRS:
            e = edge(d)
            # the run leaves from the SITE, so its exit tracks the
            # site's own row/column, not the pad's
            e = (e[0], site[1]) if DIRS[d][0] else (site[0], e[1])
            for L in others:
                if clear(site, e, L):
                    out.append(Move(net, 'dogbone', d, L, e, 1,
                                    [((px, py), site, home),
                                     (site, e, L)], site=site))

    # --- WALKED dog-bone (2026-09-10, the human's DU1): the surface stub
    # goes to a diagonal elbow and then ALONG the lane through it -- a
    # row gap, or a band's edge line -- to a via site up to `walk` pitches
    # away, and the run on the other layer leaves from THAT site in any
    # direction. Measured on the human's board: 24 vias inside DU1, 14 of
    # them in the band, the first via a median 1.3 mm from its ball (max
    # 6.7); the via field, not the faces, is the destination. The engine
    # lays it as its own #652 lane-walk (underpad._dogbone_path_valid).
    if walk > 0:
        for (sx, sy) in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            elbow = (px + sx * hx, py + sy * hy)
            if not (x0 < elbow[0] < x1 and y0 < elbow[1] < y1):
                continue
            if not clear((px, py), elbow, home):
                continue
            for (ux, uy) in ((sx, 0), (0, sy)):
                for k in range(1, walk + 1):
                    site = (elbow[0] + ux * k * grid.pitch_x,
                            elbow[1] + uy * k * grid.pitch_y)
                    # THE OFF-ARRAY SITE (walk_off, 2026-09-11): the walk
                    # used to stop dead at the ball field's boundary, and
                    # that is where the human puts the corner nets' vias --
                    # one step further, in the clear margin BESIDE the
                    # array, where there are no balls at all. The human's
                    # DU1 corner nets (SA13 SA14 SA15 SA6 SA11) all escape
                    # that way and we had no move of the class at all
                    # (README, "the off-array walk site is the class to
                    # add"). Exactly ONE step beyond is offered: the site
                    # lands half a pitch outside the outer ball line, still
                    # beside the array, and the walk then stops.
                    off = not (x0 < site[0] < x1 and y0 < site[1] < y1)
                    if off and not walk_off:
                        break           # off the array: no site beyond
                    if not clear(elbow, site, home):
                        break           # the lane is blocked from here on
                    if via_clear and not all(via_clear(site, lay) for lay in layers):
                        if off:
                            break       # the one site beyond is taken
                        continue        # this site is taken; the next may be free
                    for d in DIRS:
                        e = exit_from(site, d)
                        for L in others:
                            if clear(site, e, L):
                                out.append(Move(net, 'dogbone', d, L, e, 1,
                                                [((px, py), elbow, home),
                                                 (elbow, site, home),
                                                 (site, e, L)],
                                                site=site, walk=k,
                                                off_array=off))
                    if off:
                        break           # one site beyond the boundary, no more

    # --- CLIMB (2026-09-10): a dog-bone or via-in-pad whose run on the
    # other layer first travels ALONG the array -- up a column gap for a
    # left/right exit, along a row gap for up/down -- and leaves the face
    # at a CHOSEN row or column, up to `climb` pitches from its own. The
    # layer it runs on has no pads under a BGA, only via barrels to
    # clear, which is why the human's riders can do it: on allwinner's K51
    # eight of the eleven north riders dive beside the ball, run 3-4 mm
    # north along a column gap on B and leave the east face nested at
    # rows 5-10 mm from their own (measured 2026-09-10: SA11 2.7 mm at
    # x 125.78, SA12 4.0 at 126.86, SA15 3.1 at 126.07, SBA1 4.2 at
    # 127.14). Off at climb=0: the menu is then byte-identical.
    if climb > 0:
        starts = []      # (kind, site, first legs): where the run begins
        for (sx, sy) in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            site = (px + sx * hx, py + sy * hy)
            if not (x0 < site[0] < x1 and y0 < site[1] < y1):
                continue
            if via_clear and not all(via_clear(site, lay) for lay in layers):
                continue
            if not clear((px, py), site, home):
                continue
            starts.append(('dogbone', site, [((px, py), site, home)]))
        if not (via_clear and not all(via_clear((px, py), lay)
                                      for lay in layers)):
            starts.append(('via_in_pad', (px, py), []))
        for kind, site, legs0 in starts:
            for L in others:
                for d, (dx, dy) in DIRS.items():
                    e0 = edge(d)
                    # the gaps the run may climb along: a dog-bone's site
                    # is already in one; a via-in-pad steps half a pitch
                    # into the gap on either side first
                    if kind == 'dogbone':
                        gaps = [(site, [])]
                    elif dx:
                        gaps = [((px + g * hx, py), [((px, py), (px + g * hx, py), L)])
                                for g in (-1, 1) if x0 < px + g * hx < x1]
                    else:
                        gaps = [((px, py + g * hy), [((px, py), (px, py + g * hy), L)])
                                for g in (-1, 1) if y0 < py + g * hy < y1]
                    for (gx, gy), legs_in in gaps:
                        if any(not clear(a, b, l) for a, b, l in legs_in):
                            continue
                        for s in (-1, 1):
                            # HALF-pitch steps: the run layer has no pads
                            # under the array (a BGA's back), so the run may
                            # leave along a row LINE as well as a gap midline
                            # -- on a face carrying two teeth per pitch every
                            # gap midline is taken and the row lines between
                            # them are the free exits (the human's nested
                            # riders at K51 leave at half-pitch spacing)
                            for h in range(1, 2 * climb + 1):
                                k = (h + 1) // 2
                                if dx:
                                    ey = gy + s * h * hy
                                    if not (y0 < ey < y1):
                                        break       # beyond the outer row
                                    turn, e = (gx, ey), (e0[0], ey)
                                else:
                                    ex = gx + s * h * hx
                                    if not (x0 < ex < x1):
                                        break
                                    turn, e = (ex, gy), (ex, e0[1])
                                if not clear((gx, gy), turn, L):
                                    break           # the gap is blocked from here on
                                if not clear(turn, e, L):
                                    continue        # this row's exit is; the next may not be
                                out.append(Move(net, kind, d, L, e, 1,
                                                legs0 + legs_in
                                                + [((gx, gy), turn, L), (turn, e, L)],
                                                site=(site if kind == 'dogbone' else (px, py)),
                                                climb=k))
    return out


