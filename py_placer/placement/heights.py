"""How tall each part is, and what fits under it (#1443).

A part's 3D model says how high its body stands above its own face and, cell
by cell, how high its LOWEST point is -- the room under it. A shield can's lid
stands 2 mm over the parts inside it, a module on headers 7-10 mm over the
board, an M.2 card over the parts under it; a QFP's body sits on its leads.

Two rules read it, for two parts on one face whose heights are both KNOWN:

  * a part may stand inside a cover's occupancy (the box the seat and the
    grade give it -- its courtyard, else its fab outline or pads) where the
    cover's model is over it at least the part's height plus `MARGIN_MM`
    higher, and never under `ROOM_FLOOR_MM` (a measured underside: a declared
    one has no floor, `Cover.floor`), and nowhere inside a courtyard
    the cover DRAWS (KiCad grades that
    one, and the human boards never put a part in one);
  * a part may NOT stand under a cover's raised body outside that occupancy
    (a module's overhang, a card) where the two models meet, cell by cell
    (`OVERHANG_MARGIN_MM`); a part with no model has no shape to judge
    there and is judged on its occupancy alone.

A part whose height is unknown -- no model kicad-cli resolves, and no height
in its IPC-7351 name -- is judged as before, on its occupancy alone: nothing
is risked on a guess. So is every pair where neither part has room under it.
A part something reaches from above -- a connector its plug mates with, a
switch, a test point, a fiducial, a mounting hole (`part_class` gives it a
class, or its designator is one of those) -- never goes under a cover,
whatever its height, and admits nothing inside its own occupancy: the room
its model shows under it is set aside, and every reader names the part
(`Heights.notices`). A declared `heights[].underside_mm` is the exception:
the datasheet outranks a designator, so a shield can named J7 with a
declared underside has room under it (and the notice says it is applied
over the rule).

Only the surfaces of a model that face the board (or stand on it) say how
much room there is under it: a model drawn without a bottom face would
otherwise read as hollow under its top.

Measured on the human corpus (#1443): of 2819 parts that 157 boards stand
inside another part's outline, KiCad flags 87 -- the covers draw no
courtyard where parts go under them. The tightest gap measured between a
part's top and a cover's underside was 0.52 mm (a 2.0 mm shield lid);
`MARGIN_MM` is 0.5. Graded on the 56 human boards with a cover the installed
library models, the overhang rule refuses only parts whose models
interpenetrate (two connectors drawn over each other, either-or footprints).

An intent may DECLARE what the models cannot say (`with_declared`):
`covers[]`, a body a connector carries over the board -- a card in a socket,
a module on a stacking connector -- whose underside depends on the socket
and on the card fitted, so it is never read off a model; and `heights[]`, a
part's height or the room under its body, from its datasheet. A `heights[]`
row's `ref` is a reference, else a reference glob (`fnmatch`, as a block's
`refs`); a row naming a part literally outranks a glob, and of two globs the
earlier row wins. A row that names no part on the board is named in
`Heights.notices`, never dropped in silence. A declared
body moves with its connector, and under it a part must be KNOWN to fit
(`MARGIN_MM`): a part of unknown height is refused there. So is a part
reached from above, except a test point under a body declared `removable`
(a card comes out of its socket for test and rework): it is judged by its
height like any other part.

The heights come from one `kicad-cli pcb export glb` of the board, cached by
the footprints' models (not their poses) in `~/.cache/krt_heights`, so the
seeds of one placement export once. Without kicad-cli every height is
unknown and nothing changes.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import struct
import subprocess
import tempfile
from typing import Dict, NamedTuple, Optional, Tuple

#: the room a part needs between its top and a cover's underside to be
#: ADMITTED under it, mm
MARGIN_MM = 0.5
#: ...and to be REFUSED under a raised body past the cover's occupancy, the
#: two bodies must meet: the human boards tuck a part's edge under a raised
#: body with 0.06-0.47 mm to spare (a ferrite under a tantalum's moulded
#: edge, a resistor at a module's), and never through one
OVERHANG_MARGIN_MM = 0.0
#: a body at least this high over the board is raised: a part under it has to
#: fit under it, wherever its occupancy is
LIFT_MIN_MM = 0.5
#: ...and the lowest underside that ADMITS a part. Every underside under this
#: the corpus census found was a model's offset or a body's standoff, not
#: room: an SMD module soldered flat whose model floats 1.0 mm (about its own
#: board's thickness), a radio module's board at 0.8 mm, tantalum and DIP
#: standoffs, a QFP's lead shoulders. The lowest real cover measured was a
#: 2.0 mm shield lid.
ROOM_FLOOR_MM = 1.5
#: the raster cell, mm
CELL_MM = 0.25
#: the room a part must have under it, or the raised overhang past its own
#: occupancy, to be a cover at all: less is a body's curve or step, mm^2
ROOM_MIN_MM2 = 1.0
#: the shortest part the room under a cover is measured for, mm (an 0201 is
#: 0.3 mm tall)
_SHORTEST_MM = 0.2
#: geometry lower than this above its face is the part's pins or seat, not body
_ABOVE_MM = 0.05
#: a surface whose normal points this much away from the board faces up: it
#: is a top, and says nothing about the room under it -- a model drawn
#: without a bottom face would otherwise read hollow (cos 78 degrees)
_UP_FACING = 0.2
#: kicad-cli's export, a hang guard (a 500-part board takes under a minute)
EXPORT_TIMEOUT_S = 600
#: bump to invalidate every cached measurement
_CACHE_VERSION = 4

#: IPC-7351 land-pattern names end their body dimensions with the height in
#: 1/100 mm: CAPC1005X55N, QFN50P400X400X80-25N, SOIC127P600X175-8N,
#: BGA324C80P18X18_1500X1500X150
_IPC = re.compile(r'^(?:CAP|RES|IND|DIO|LED|FUS|OSC|XTAL|SO|SOT|SOD|SOIC|SOP|SSOP|TSOP|'
                  r'MSOP|QFN|DFN|SON|QFP|LQFP|TQFP|BGA|LGA|TO|SC|PLCC|CSP|TS|HTS|VS|HS|PQFP|MQFP|'
                  r'VQFN|WQFN|UQFN|XQFN|WSON|VSON|USON)[A-Z]*\d')
_IPC_H = re.compile(r'X(\d{2,4})(?=[A-Z_\-]|$)')


def ipc_height(name: str) -> Optional[float]:
    """The body height an IPC-7351 name carries, mm, or None."""
    stem = os.path.splitext(os.path.basename((name or '').replace('\\', '/')))[0]
    stem = stem.split(':')[-1].upper()
    if not _IPC.match(stem):
        return None
    hs = _IPC_H.findall(stem)
    if not hs:
        return None
    h = int(hs[-1]) / 100.0
    return h if 0.05 <= h <= 30.0 else None


class Decl(NamedTuple):
    """A DECLARED body (`covers[]`), in its connector's local frame: its
    `rect`, its `under`side (mm over the face), the refs it does not judge
    (`skip`: its standoffs and pad-less footprints), the test points judged
    by height under it when it is removable (`probed`), and since #1504 its
    `top` (None: undeclared), whether it is `removable`, and the `sweep` it
    goes in by (`_sweep`, None without an `insertion`)."""
    rect: Tuple[float, float, float, float]
    under: float
    skip: frozenset
    probed: frozenset
    top: Optional[float] = None
    removable: bool = False
    sweep: Optional[dict] = None


class Cover:
    """One part's room-under raster, in its footprint's LOCAL frame (the frame
    its courtyard and pads are stored in; board = pose + R(-rot) local).

    `lo[i, j]` is the lowest point of the model over cell (i, j), mm above the
    part's face, `inf` where the model is not over it; `court[i, j]` marks a
    cell nothing may stand in: one the drawn courtyard reaches, or the part's
    own pad copper -- its pins stand there, and a model need not draw them (a
    module's model is often its board alone, floating over its headers).
    `solid`: nothing may stand inside the part's occupancy, whatever the
    model says -- a part something reaches from above (a connector's model
    is the likeliest to stand off its footprint); its raised body past the
    occupancy still binds.
    `floor`: the lowest underside that admits a part. `ROOM_FLOOR_MM` for a
    measured one, whose low readings are a model's offset or standoff; 0 for
    a declared `underside_mm`, which is the datasheet's room, not a guess."""

    __slots__ = ('ref', 'x0', 'y0', 'lo', 'court', 'lift_box', 'solid', 'decl',
                 'floor')

    def __init__(self, ref, x0, y0, lo, court=None, solid=False, decl=None,
                 floor=ROOM_FLOOR_MM):
        import numpy as np
        self.ref = ref
        self.solid = solid
        self.floor = floor
        self.x0, self.y0 = x0, y0
        self.lo = lo
        self.court = court if court is not None else np.zeros(lo.shape, bool)
        #: a DECLARED body (`covers[]`): (its rect, local frame; its
        #: underside, mm; the refs it does not judge -- its standoffs and pad-less
        #: footprints, a logo or a drawing; the test points judged by their
        #: height under it, when the body is removable), or None
        self.decl = decl
        raised = np.isfinite(lo) & (lo >= LIFT_MIN_MM)
        if raised.any():
            ii, jj = np.nonzero(raised)
            self.lift_box = (x0 + ii.min() * CELL_MM, y0 + jj.min() * CELL_MM,
                             x0 + (ii.max() + 1) * CELL_MM, y0 + (jj.max() + 1) * CELL_MM)
        else:
            self.lift_box = None
        if decl is not None:
            for r in (decl[0], _sweep_rect(getattr(decl, 'sweep', None))):
                if r is None:
                    continue
                lb = self.lift_box or r
                self.lift_box = (min(lb[0], r[0]), min(lb[1], r[1]),
                                 max(lb[2], r[2]), max(lb[3], r[3]))

    def _window(self, box):
        """The cell index ranges over local `box`, clipped, and whether the box
        reaches past the raster."""
        nx, ny = self.lo.shape
        i0 = math.floor((box[0] - self.x0) / CELL_MM)
        i1 = math.ceil((box[2] - self.x0) / CELL_MM)
        j0 = math.floor((box[1] - self.y0) / CELL_MM)
        j1 = math.ceil((box[3] - self.y0) / CELL_MM)
        past = i0 < 0 or j0 < 0 or i1 > nx or j1 > ny
        return max(0, i0), min(nx, i1), max(0, j0), min(ny, j1), past

    def blocked(self, occ, box, top, grow=0.0, inside=True, body=None) -> float:
        """How much of a part the cover blocks, mm^2; 0.0 when it fits:
        `inside_area` (when `inside`) plus `over_area`."""
        a = self.inside_area(occ, box, top, grow) if inside else None
        return (a or 0.0) + self.over_area(body)

    def inside_area(self, occ, box, top, grow=0.0) -> Optional[float]:
        """Inside the cover's occupancy `occ`, the part's occupancy `box`
        (local to this cover), judged as a part `top` mm tall (`inf` for one
        that may never be covered): None when the part keeps `grow` (the
        seat's clearance, measured as `legality.rect_gap` measures it) from
        `occ` -- nothing to judge -- else 0.0 when the model is over all of it,
        high enough (`MARGIN_MM`, `ROOM_FLOOR_MM`), with no drawn courtyard
        or pad there, else the area it reaches into, mm^2."""
        from placement.legality import rect_gap
        g = _grown(box, grow)
        ix = (max(g[0], occ[0]), max(g[1], occ[1]),
              min(g[2], occ[2]), min(g[3], occ[3]))
        if not (ix[2] > ix[0] and ix[3] > ix[1] and rect_gap(box, occ) < grow + 1e-9):
            return None
        i0, i1, j0, j1, past = self._window(ix)
        ok = not past and i1 > i0 and j1 > j0 and not self.solid
        if ok:
            w = self.lo[i0:i1, j0:j1]
            # a cell the model is not over (inf) is unknown, not room
            ok = bool((w < math.inf).all() and (w >= max(top + MARGIN_MM, self.floor)).all()
                      and not self.court[i0:i1, j0:j1].any())
        return 0.0 if ok else (ix[2] - ix[0]) * (ix[3] - ix[1])

    def over_area(self, body) -> float:
        """Anywhere, the cover's RAISED body against the part's own: `body` =
        (points, heights), the part's model cell by cell in this cover's frame;
        mm^2 of the cells where the cover's underside is below the part's top
        (`OVERHANG_MARGIN_MM`). 0.0 for a part with no model (None): it has no
        shape to judge there, its courtyard box being no body."""
        if self.lift_box is None or body is None:
            return 0.0
        import numpy as np
        pts, hv = body
        if not len(pts):
            return 0.0
        nx, ny = self.lo.shape
        ii = np.floor((pts[:, 0] - self.x0) / CELL_MM).astype(int)
        jj = np.floor((pts[:, 1] - self.y0) / CELL_MM).astype(int)
        ok = (ii >= 0) & (ii < nx) & (jj >= 0) & (jj < ny)
        if not ok.any():
            return 0.0
        w = self.lo[ii[ok], jj[ok]]
        hit = (w >= LIFT_MIN_MM) & (w < hv[ok] + OVERHANG_MARGIN_MM)
        if self.decl is not None:
            # the declared body is judged by `declared_area`, not the model
            r = self.decl[0]
            p = pts[ok]
            hit &= ~((p[:, 0] >= r[0]) & (p[:, 0] <= r[2]) & (p[:, 1] >= r[1]) & (p[:, 1] <= r[3]))
        return int(hit.sum()) * CELL_MM * CELL_MM

    def declared_area(self, box, top, body=None) -> float:
        """The declared body (`covers[]`) against a part: 0.0 when the part
        fits under it (its top `MARGIN_MM` under the declared underside), else
        mm^2 of the part under it -- its model cell by cell where it has one
        (`body`, this cover's frame), else its occupancy `box` (local to this
        cover). `top` is `inf` for a part whose height is unknown: nothing is
        risked under a declared body on a guess."""
        if self.decl is None:
            return 0.0
        r, under = self.decl[0], self.decl[1]
        if top + MARGIN_MM <= under:
            return 0.0
        if body is not None and math.isfinite(top):
            pts, hv = body
            if not len(pts):
                return 0.0
            inside = cells_in_rect(pts, r) & (hv + MARGIN_MM > under)
            return int(inside.sum()) * CELL_MM * CELL_MM
        w = min(box[2], r[2]) - max(box[0], r[0])
        h = min(box[3], r[3]) - max(box[1], r[1])
        return w * h if w > 0 and h > 0 else 0.0


def cells_in_rect(pts, r):
    """Which of a body's cell centres `pts` (n, 2) lie in the local rect `r`:
    the one test of a model being under a declared body, shared by the
    verdict (`Cover.declared_area`) and the report (`Heights.under_declared`,
    #1539)."""
    return ((pts[:, 0] >= r[0]) & (pts[:, 0] <= r[2]) & (pts[:, 1] >= r[1])
            & (pts[:, 1] <= r[3]))


def to_local(box, pose):
    """Board-frame `box` as a box in the local frame of a part at `pose`
    (x, y, rot): the inverse of `legality.rotate_local_bounds`."""
    x, y, rot = pose
    r = math.radians(rot % 360)
    c, s = math.cos(r), math.sin(r)
    if abs(s) < 1e-12 or abs(c) < 1e-12:
        c, s = round(c), round(s)
    xs, ys = [], []
    for px, py in ((box[0], box[1]), (box[2], box[1]), (box[0], box[3]), (box[2], box[3])):
        dx, dy = px - x, py - y
        xs.append(dx * c - dy * s)
        ys.append(dx * s + dy * c)
    return min(xs), min(ys), max(xs), max(ys)


def to_board(box, pose):
    """Local `box` of a part at `pose` as a board-frame box (its bbox)."""
    from placement.legality import rotate_local_bounds
    x, y, rot = pose
    b = rotate_local_bounds(*box, rot)
    return x + b[0], y + b[1], x + b[2], y + b[3]


class Heights:
    """The board's heights: `top[ref]` (mm, how it was known) and `covers[ref]`
    for the parts with room under them. Empty when nothing was measured."""

    def __init__(self, top=None, covers=None, note='', reached=(), bodies=None,
                 set_aside=None, notices=()):
        self.top: Dict[str, Tuple[float, str]] = dict(top or {})
        #: each modelled part's body, local frame: (cell centres (n, 2), the
        #: model's top over each (n,), their bounding box)
        self.bodies = {r: _with_extent(v) for r, v in (bodies or {}).items()}
        self.covers: Dict[str, Cover] = dict(covers or {})
        self.note = note
        #: the parts reached from above, which nothing may cover
        self.reached = frozenset(reached)
        #: ...and why each is (`_reached`), when the caller said
        self.reached_why: Dict[str, str] = dict(reached) if isinstance(reached, dict) else {}
        #: the parts whose model shows room under them that the reached rule
        #: keeps solid: {ref: why it is reached}
        self.set_aside: Dict[str, str] = dict(set_aside or {})
        #: what was set aside or not applied, one line each, for every reader
        #: to print: a model's room kept solid, a declaration applied over
        #: the reached rule, a `heights[]` row that names no part
        self.notices: Tuple[str, ...] = tuple(notices)

    def __bool__(self):
        return bool(self.covers)

    def room_refs(self):
        """The covers a part may stand under: not solid, or carrying a
        declared body -- never one the reached rule keeps solid."""
        return sorted(r for r, c in self.covers.items() if not c.solid or c.decl is not None)

    def height(self, ref) -> Optional[float]:
        t = self.top.get(ref)
        return t[0] if t else None

    def under_height(self, ref) -> Optional[float]:
        """How tall `ref` is when something would cover it: its height, or
        `inf` for a part reached from above; None when unknown."""
        return math.inf if ref in self.reached else self.height(ref)

    def verdict(self, a, pose_a, occ_a, side_a, b, pose_b, occ_b, side_b,
                grow=0.0, inside=True):
        """What the heights say about two parts at these poses, or None when
        they cannot judge the pair (another face, no cover, a height unknown):
        `(cleared, over, inside)` -- `cleared`, one stands inside the other's
        occupancy with room to spare, so the occupancy test does not apply;
        `over`, mm^2 of the two bodies meeting under a raised body; `inside`,
        mm^2 the occupancy test would price when not cleared. `occ_*` are the
        parts' LOCAL occupancy boxes; `grow` the seat's clearance; `inside`
        False judges the raised bodies alone."""
        if side_a != side_b:
            return None
        ca, cb = self.covers.get(a), self.covers.get(b)
        if ca is None and cb is None:
            return None
        ta, tb = self.under_height(a), self.under_height(b)
        judged = cleared = False
        over = 0.0
        ins = None
        for cov, x, px, ox, y, py, oy, ty in ((ca, a, pose_a, occ_a, b, pose_b, occ_b, tb),
                                              (cb, b, pose_b, occ_b, a, pose_a, occ_a, ta)):
            if cov is None:
                continue
            # a declared body judges a part of unknown height too: refused
            decl = cov.decl is not None and y not in cov.decl[2]
            if ty is None and not decl:
                continue
            judged = True
            box = to_local(to_board(oy, py), px)
            if ty is not None and inside and not (
                    box[0] >= ox[2] + grow or ox[0] >= box[2] + grow
                    or box[1] >= ox[3] + grow or ox[1] >= box[3] + grow):
                v = cov.inside_area(ox, box, ty, grow)
                if v is not None:
                    cleared = cleared or v == 0.0
                    ins = v if ins is None else min(ins, v)
            lb = cov.lift_box
            bb = self.bodies.get(y)
            meet = 0.0
            if lb is not None and bb is not None:
                ext = to_local(to_board(bb[2], py), px)
                if ext[0] < lb[2] and lb[0] < ext[2] and ext[1] < lb[3] and lb[1] < ext[3]:
                    meet = cov.over_area(self._body_in(y, py, px))
            if decl:
                r = cov.decl[0]
                if box[0] < r[2] and r[0] < box[2] and box[1] < r[3] and r[1] < box[3]:
                    td = self.height(y) if y in cov.decl[3] else ty
                    meet += cov.declared_area(
                        box, math.inf if td is None else td,
                        self._body_in(y, py, px) if bb is not None else None)
                # #1504: past the body's far end, the path the card goes in
                # by passes over the part, and must clear it too
                sw = getattr(cov.decl, 'sweep', None)
                if sw is not None:
                    meet += sweep_part_area(sw, box, ty)
            over = max(over, meet)
        # #1504: two declared bodies over one another, and each one's way in
        # under the other
        bb_over = body_pair_area(ca.decl if ca else None, pose_a,
                                 cb.decl if cb else None, pose_b)
        if bb_over is not None:
            judged = True
            over = max(over, bb_over)
        if not judged:
            return None
        return cleared, over, ins or 0.0

    def pair(self, a, pose_a, occ_a, side_a, b, pose_b, occ_b, side_b,
             grow=0.0, inside=True) -> Optional[float]:
        """The overlap the two parts' bodies make, mm^2 -- 0.0 when one fits
        under the other or they do not meet -- or None when the heights
        cannot judge the pair (`verdict`)."""
        v = self.verdict(a, pose_a, occ_a, side_a, b, pose_b, occ_b, side_b, grow, inside)
        if v is None:
            return None
        cleared, over, ins = v
        return over + (0.0 if cleared else ins)

    def under_declared(self, a, pose_a, b, pose_b, occ_b) -> Optional[dict]:
        """Part `b` under `a`'s DECLARED body (`covers[]`), judged on the shape
        and height `verdict` judges it by (#1539), or None when it is not
        under it: its model's cells inside the body's rect where it has a
        model and a known height, else its occupancy box `occ_b` (local)
        overlapping the rect -- never a box test for a part the verdict judges
        cell by cell, or a part whose body stays beside the cover would be
        reported under it with negative room while the verdict passes it.
        `{'height_mm', 'room_mm', 'on'}`: on a model, the tallest of its cells
        inside and the declared underside less that; on its occupancy, the
        height it is judged by (`room_mm` None when unknown, and `reached`
        True for a part reached from above, which nothing may cover). None
        too for a part the body does not judge (a standoff, a pad-less
        footprint)."""
        cov = self.covers.get(a)
        decl = getattr(cov, 'decl', None)
        if decl is None or b in decl[2]:
            return None
        r, under = decl[0], decl[1]
        box = to_local(to_board(occ_b, pose_b), pose_a)
        if not (box[0] < r[2] and r[0] < box[2] and box[1] < r[3] and r[1] < box[3]):
            return None
        top = self.height(b) if b in decl[3] else self.under_height(b)
        if b in self.bodies and top is not None and math.isfinite(top):
            pts, hv = self._body_in(b, pose_b, pose_a)
            inside = cells_in_rect(pts, r) if len(pts) else None
            if inside is None or not inside.any():
                return None
            tall = float(hv[inside].max())
            return {'height_mm': round(tall, 3), 'room_mm': round(under - tall, 3),
                    'on': 'model'}
        out = {'height_mm': self.height(b), 'on': 'occupancy',
               'room_mm': (round(under - top, 3) if top is not None and math.isfinite(top)
                           else None)}
        if top is not None and not math.isfinite(top):
            out['reached'] = True
        return out

    def _body_in(self, ref, pose, frame):
        """`ref`'s body at `pose`, cell by cell, in the local frame of a part
        at `frame`, or None when it has no model."""
        b = self.bodies.get(ref)
        if b is None:
            return None
        import numpy as np
        pts, hv = b[0], b[1]
        r1 = math.radians(-(pose[2] % 360))
        r2 = math.radians(frame[2] % 360)
        c1, s1, c2, s2 = math.cos(r1), math.sin(r1), math.cos(r2), math.sin(r2)
        bx = pose[0] + pts[:, 0] * c1 - pts[:, 1] * s1 - frame[0]
        by = pose[1] + pts[:, 0] * s1 + pts[:, 1] * c1 - frame[1]
        return np.stack([bx * c2 - by * s2, bx * s2 + by * c2], 1), hv

    def reach(self, ref, pose, occ):
        """The board box a cover's rule can reach at `pose`: its occupancy and
        its raised body. None for a part with no room under it."""
        c = self.covers.get(ref)
        if c is None:
            return None
        b = to_board(occ, pose)
        if c.lift_box is None:
            return b
        lb = to_board(c.lift_box, pose)
        return (min(b[0], lb[0]), min(b[1], lb[1]), max(b[2], lb[2]), max(b[3], lb[3]))


def _with_extent(body):
    """A body as (cell centres, heights, their bounding box)."""
    if len(body) >= 3:
        return body
    pts, hv = body
    return (pts, hv, (float(pts[:, 0].min()) - CELL_MM, float(pts[:, 1].min()) - CELL_MM,
                      float(pts[:, 0].max()) + CELL_MM, float(pts[:, 1].max()) + CELL_MM))


def _grown(box, g):
    return (box[0] - g, box[1] - g, box[2] + g, box[3] + g) if g else box


EMPTY = Heights(note='no heights')
_MEMO: Dict[str, Heights] = {}


def for_board(pcb_data, pcb_file: Optional[str] = None, declared=None) -> Heights:
    """The board's `Heights`, measured once per process and cached on disk,
    with the intent's `declared` covers and heights over it
    (`with_declared`). Never raises: a board it cannot measure has no
    measured heights, and every pair keeps its occupancy test."""
    try:
        h = _for_board(pcb_data, pcb_file)
    except Exception as ex:                                  # noqa: BLE001
        h = Heights(note=f'no heights: the measurement failed ({type(ex).__name__}: '
                         f'{str(ex)[:120]})')
    return with_declared(h, declared, pcb_data, pcb_file)


def _axis(row):
    """A `covers[]` row's axis and its left, unit vectors in the connector's
    local frame (`axis_deg` is a quarter turn)."""
    a = math.radians(float(row['axis_deg']))
    ux, uy = round(math.cos(a)), round(math.sin(a))
    return (ux, uy), (-uy, ux)


def cover_rect(row) -> Tuple[float, float, float, float]:
    """A `covers[]` row's body, in its connector's LOCAL frame: from the
    connector's origin along `axis_deg` (0 = local +x, 90 = local +y) to
    `length_mm`, else to its farthest standoff's `length_mm`, `width_mm`
    wide about a centre line `offset_mm` to the axis's left (local +y for an
    axis of 0)."""
    (ux, uy), (vx, vy) = _axis(row)
    length = row.get('length_mm')
    if length is None:
        length = max(float(s['length_mm']) for s in row['standoffs'])
    off, half = float(row.get('offset_mm') or 0.0), float(row['width_mm']) / 2.0
    xs, ys = [], []
    for t in (0.0, float(length)):
        for w in (off - half, off + half):
            xs.append(t * ux + w * vx)
            ys.append(t * uy + w * vy)
    return min(xs), min(ys), max(xs), max(ys)


def standoff_at(row, standoff) -> Tuple[float, float]:
    """Where a `covers[]` row puts one of its standoffs, connector's local
    frame: `length_mm` along the axis, `off_mm` (default 0, #1504) to the
    left of the centre line."""
    (ux, uy), (vx, vy) = _axis(row)
    t = float(standoff['length_mm'])
    w = float(row.get('offset_mm') or 0.0) + float(standoff.get('off_mm') or 0.0)
    return t * ux + w * vx, t * uy + w * vy


def cover_length(row) -> float:
    """How far a `covers[]` row's body reaches along its axis."""
    return float(row['length_mm'] if row.get('length_mm') is not None
                 else max(s['length_mm'] for s in row['standoffs']))


#: the insertion path is sampled at this many angles from flat to the
#: insertion angle, and this many steps of the slide in at that angle
_SWEEP_TURNS = 24
_SWEEP_SLIDES = 8


def _sweep(row) -> Optional[dict]:
    """The path a declared body goes in by (#1504), or None without an
    `insertion`: the card slides in at `angle_deg` for `travel_mm`, then
    turns down about the connector to flat. Each pose is a line through the
    pivot (the connector's origin, at the underside): at a distance `t`
    along the axis its underside is `underside + t tan(angle)` and its top
    `top_mm` above that over `cos(angle)`, for `t` from `s cos(angle)` to
    `(s + length) cos(angle)` (`s` the slide still to go). Lateral extent
    stays the body's."""
    ins = row.get('insertion')
    if not ins:
        return None
    (ux, uy), (vx, vy) = _axis(row)
    th = math.radians(float(ins['angle_deg']))
    tr = float(ins.get('travel_mm') or 0.0)
    poses = [(th * k / _SWEEP_TURNS, 0.0) for k in range(_SWEEP_TURNS + 1)]
    if tr > 0:
        poses += [(th, tr * k / _SWEEP_SLIDES) for k in range(1, _SWEEP_SLIDES + 1)]
    length = cover_length(row)
    return {'u': (ux, uy), 'v': (vx, vy), 'off': float(row.get('offset_mm') or 0.0),
            'half': float(row['width_mm']) / 2.0, 'length': length,
            'under': float(row['underside_mm']), 'top': row.get('top_mm'),
            'poses': poses,
            'reach': max((s + length) * math.cos(p) for p, s in poses)}


def sweep_part_area(sw, box, top) -> float:
    """mm^2 of a part (its local occupancy `box`, `top` mm tall, None or inf
    when unknown or reached from above) past a body's far end that the body's
    insertion path `sw` passes over too low to clear (#1504). Inside the
    body's length the path never comes lower than the body itself, which
    `declared_area` judges, so only the part past it is read here."""
    sp = _span(sw, box)
    if sp is None:
        return 0.0
    (t0, t1), lat = sp
    # strictly past the body: a part that only touches its far end is the
    # body's question (`declared_area`), not the path's
    a, b = max(t0, sw['length'] + 1e-6), min(t1, sw['reach'])
    if b <= a:
        return 0.0
    got = sweep_at(sw, a, b)
    if got is None:
        return 0.0
    tt = math.inf if top is None else top
    return (b - a) * lat if tt + MARGIN_MM > got[0] else 0.0


def _decl_box(d, pose, frame):
    """A declared body's rect at `pose`, in the local frame of `frame`."""
    return to_local(to_board(d.rect, pose), frame)


def body_pair_area(da, pose_a, db, pose_b) -> Optional[float]:
    """mm^2 where two DECLARED bodies (#1504) meet, or None when either is
    undeclared: overlapping in plan, one must clear the other (`MARGIN_MM`
    from the lower one's `top_mm` to the higher one's underside; an unknown
    top never clears), and neither's insertion path may pass through the
    other when that one is fixed (a removable one only orders the two,
    `insertion_order`)."""
    if da is None or db is None:
        return None
    rb = _decl_box(db, pose_b, pose_a)
    ra = da.rect
    w = min(ra[2], rb[2]) - max(ra[0], rb[0])
    h = min(ra[3], rb[3]) - max(ra[1], rb[1])
    out = 0.0
    if w > 0 and h > 0:
        fits = ((db.top is not None and db.top + MARGIN_MM <= da.under)
                or (da.top is not None and da.top + MARGIN_MM <= db.under))
        if not fits:
            out = w * h
    for x, px, y, py in ((da, pose_a, db, pose_b), (db, pose_b, da, pose_a)):
        if x.sweep is not None and not y.removable:
            out = max(out, path_blocked_area(x, y, _decl_box(y, py, px)))
    return out


def path_blocked_area(x, y, yb) -> float:
    """mm^2 of body `y` (its rect `yb`, in `x`'s connector frame) in the way
    of body `x`'s insertion path: over the path where `y` neither stands
    wholly under its lowest point nor wholly over its highest. A path with
    no `top_mm` reaches an unknown height, so any body over it is in the
    way."""
    sw = x.sweep
    sp = _span(sw, yb)
    if sp is None:
        return 0.0
    (t0, t1), lat = sp
    got = sweep_at(sw, t0, t1)
    if got is None:
        return 0.0
    lo, hi = got
    under_it = y.top is not None and y.top + MARGIN_MM <= lo
    over_it = hi is not None and y.under >= hi + MARGIN_MM
    if under_it or over_it:
        return 0.0
    return (min(t1, sw['reach']) - max(t0, 0.0)) * lat


def insertion_order(h, poses) -> list:
    """`[(first, then)]`: pairs of declared bodies where `first`'s insertion
    path passes under removable body `then`, so `first` goes in before it
    (#1504). `poses` maps each connector to its (x, y, rot)."""
    out = []
    decl = {r: c.decl for r, c in h.covers.items()
            if c.decl is not None and r in poses}
    for a, da in decl.items():
        if da.sweep is None:
            continue
        for b, db in decl.items():
            if b != a and db.removable and path_blocked_area(
                    da, db, _decl_box(db, poses[b], poses[a])) > 0:
                out.append((a, b))
    return sorted(out)


def _sweep_rect(sw) -> Optional[Tuple[float, float, float, float]]:
    """The plan box (local frame) the insertion path passes over."""
    if sw is None:
        return None
    (ux, uy), (vx, vy) = sw['u'], sw['v']
    xs, ys = [], []
    for t in (0.0, sw['reach']):
        for w in (sw['off'] - sw['half'], sw['off'] + sw['half']):
            xs.append(t * ux + w * vx)
            ys.append(t * uy + w * vy)
    return min(xs), min(ys), max(xs), max(ys)


def _span(sw, box):
    """A local box as (along-axis range, lateral overlap with the body, mm)
    in the frame of the sweep `sw`, or None when it is beside the body."""
    (ux, uy), (vx, vy) = sw['u'], sw['v']
    ts, ws = [], []
    for x in (box[0], box[2]):
        for y in (box[1], box[3]):
            ts.append(x * ux + y * uy)
            ws.append(x * vx + y * vy - sw['off'])
    lat = min(max(ws), sw['half']) - max(min(ws), -sw['half'])
    return ((min(ts), max(ts)), lat) if lat > 0 else None


def sweep_at(sw, t0, t1) -> Optional[Tuple[float, Optional[float]]]:
    """(the lowest underside, the highest top -- None without `top_mm`) the
    insertion path passes over at along-axis distances [t0, t1], or None
    when it passes nowhere there."""
    a, b = max(t0, 0.0), min(t1, sw['reach'])
    if b < a:
        return None
    n = max(1, int(math.ceil((b - a) / CELL_MM)))
    u, top = sw['under'], sw['top']
    lo, hi, hit = math.inf, -math.inf, False
    for k in range(n + 1):
        x = a + (b - a) * k / n
        for p, s in sw['poses']:
            c = math.cos(p)
            if s * c - 1e-9 <= x <= (s + sw['length']) * c + 1e-9:
                ud = u + x * math.tan(p)
                lo = min(lo, ud)
                if top is not None:
                    hi = max(hi, ud + (float(top) - u) / c)
                hit = True
    return (lo, hi if top is not None else None) if hit else None


def along_axis(row, local_xy) -> Tuple[float, float]:
    """A local point as (distance along the row's axis, distance to the left
    of its centre line): what `standoff_at` inverts."""
    (ux, uy), (vx, vy) = _axis(row)
    x, y = local_xy
    return x * ux + y * uy, x * vx + y * vy - float(row.get('offset_mm') or 0.0)


_DECL_MEMO: Dict[tuple, Heights] = {}


def with_declared(h: Heights, declared, pcb_data, pcb_file=None) -> Heights:
    """`h` with the intent's declarations over it (`Intent.height_declarations`):
    each `heights[]` row's `height_mm` a known height and its `underside_mm`
    the room under the part across its occupancy (outside its drawn courtyard
    and pads); each `covers[]` row a declared body on its connector
    (`Cover.decl`), which moves with it. Declared values win over measured
    ones. A declared underside outranks the reached rule (`_reached`): the
    part is no solid cover, and the notice says so. `h` itself when nothing
    is declared."""
    if not declared or not (declared.get('covers') or declared.get('heights')):
        return h
    key = (id(h), id(pcb_data), json.dumps(declared, sort_keys=True, default=str))
    got = _DECL_MEMO.get(key)
    if got is not None:
        return got
    import numpy as np
    fps = pcb_data.footprints or {}
    # what a probe or a plug reaches from above, whether or not a model was
    # measured: without kicad-cli no measurement named them
    if h.reached:
        reached, why = h.reached, h.reached_why
    else:
        why = _reached(pcb_data)
        reached = frozenset(why)
    top, covers, bodies = dict(h.top), dict(h.covers), dict(h.bodies)
    n_h = n_u = n_c = 0
    rows, unresolved = height_rows(declared.get('heights') or (), fps)
    unders = []
    for r, row in rows.items():
        if row.get('height_mm') is not None:
            d = float(row['height_mm'])
            top[r] = (d, 'declared')
            n_h += 1
            # The declared height is the part's top over its model too, so a
            # declared body, a measured raised body and a height-limited
            # keep-out read one height (#1505): declared taller than the
            # model, every cell stands at it (the model does not show where
            # the tall member is); declared lower, the cells are clipped.
            b = bodies.get(r)
            if b is not None and len(b[1]):
                pts, hv, ext = b
                bodies[r] = (pts, np.full_like(hv, d) if d > float(hv.max())
                             else np.minimum(hv, d), ext)
        if row.get('underside_mm') is not None:
            unders.append((r, row))
    # #1500: a part reached from above that the author declares may stand
    # under a cover, at its declared height, is judged by that height there
    # (an M.2 socket under an SO-DIMM module) -- never one whose height is
    # unknown, and said, as #1462 says the mirror case.
    lifted = sorted(r for r, row in rows.items()
                    if row.get('under_cover') and row.get('height_mm') is not None
                    and r in reached)
    lift_notices = [f"{r}: declared under_cover at height_mm {float(rows[r]['height_mm']):g}: "
                    f"judged under a cover by that height, though it is reached from "
                    f"above ({(why or {}).get(r, 'reached')})" for r in lifted]
    if lifted:
        reached = frozenset(reached) - set(lifted)
        why = {k: v for k, v in (why or {}).items() if k not in lifted}
    applied = {}
    if unders:
        try:
            from placement.body import board_bodies
            geo = board_bodies(pcb_data, pcb_file)
        except Exception:                                    # noqa: BLE001
            geo = {}
        for r, row in unders:
            g = geo.get(r)
            occ = getattr(g, 'occupancy_local', None)
            if occ is None:
                continue
            base = covers.get(r)
            x0, y0, lo = _grid_union(base, occ)
            nx, ny = lo.shape
            cx = x0 + (np.arange(nx) + 0.5) * CELL_MM
            cy = y0 + (np.arange(ny) + 0.5) * CELL_MM
            inb = (((cx >= occ[0]) & (cx <= occ[2]))[:, None]
                   & ((cy >= occ[1]) & (cy <= occ[3]))[None, :])
            lo[inb] = float(row['underside_mm'])
            court = _mask(_no_go(fps[r], getattr(g, 'court_shape_local', None)),
                          x0, y0, nx, ny)
            # the datasheet outranks the designator: never solid, and its
            # room is what it declares, however low (no measured floor)
            covers[r] = Cover(r, x0, y0, lo, court, False,
                              base.decl if base is not None else None, floor=0.0)
            applied[r] = float(row['underside_mm'])
            n_u += 1
    # a footprint with no pads is no part (a logo, a mechanical drawing)
    padless = frozenset(k for k, f in fps.items() if not f.pads)
    # a probe reaches a test point only with a removable body off it
    probed = frozenset(k for k in reached if k in fps and _is_test_point(fps[k], k))
    for row in declared.get('covers') or ():
        r = row.get('connector')
        if r not in fps:
            continue
        decl = Decl(cover_rect(row), float(row['underside_mm']),
                    frozenset(s['ref'] for s in row.get('standoffs') or ()) | padless,
                    probed if row.get('removable') else frozenset(),
                    None if row.get('top_mm') is None else float(row['top_mm']),
                    bool(row.get('removable')), _sweep(row))
        base = covers.get(r)
        if base is not None:
            covers[r] = Cover(r, base.x0, base.y0, base.lo, base.court, base.solid, decl,
                              floor=base.floor)
        else:
            # no measured room: its own occupancy keeps its occupancy test
            covers[r] = Cover(r, 0.0, 0.0, np.full((1, 1), np.inf), None, True, decl)
        n_c += 1
    note = h.note
    said = [f'{n} {w}' for n, w in ((n_c, 'cover(s)'), (n_h, 'height(s)'),
                                     (n_u, 'underside(s)')) if n]
    if said:
        note = f"{note}; declared: {', '.join(said)}"
    set_aside = {r: w for r, w in h.set_aside.items() if r not in applied}
    notices = _set_aside_notices(set_aside)
    notices += [f"{r}: declared underside_mm {u:g} applied, though it is reached "
                f"from above ({why.get(r, 'reached')})"
                for r, u in sorted(applied.items()) if r in reached]
    notices += lift_notices + unresolved
    out = Heights(top, covers, note=note, reached=why or reached, bodies=bodies,
                  set_aside=set_aside, notices=notices)
    if len(_DECL_MEMO) > 64:
        _DECL_MEMO.clear()
    _DECL_MEMO[key] = out
    return out


def is_glob(ref: str) -> bool:
    """Does a `heights[]` ref read as a glob (`*`, `?` or `[`)?"""
    return any(c in ref for c in '*?[')


def height_rows(rows, refs):
    """The `heights[]` rows each part on the board takes, {ref: row}, and a
    line for every row that names none: a reference not on the board, or a
    glob matching nothing. A row naming the part literally outranks a glob;
    of two globs over one part, the earlier row."""
    import fnmatch
    out, lines = {}, []
    for row in rows:
        r = row.get('ref')
        if isinstance(r, str) and r in refs:
            out[r] = row
    for i, row in enumerate(rows):
        r = row.get('ref')
        if not isinstance(r, str) or r in refs:
            continue
        if not is_glob(r):
            lines.append(f"heights[{i}] names {r}, which is not on this board -- "
                         f"the row is not applied")
            continue
        hit = [k for k in refs if fnmatch.fnmatchcase(k, r)]
        if not hit:
            lines.append(f"heights[{i}] {r!r} matches no part on this board -- "
                         f"the row is not applied")
        for k in hit:
            out.setdefault(k, row)
    return out, lines


def _set_aside_notices(set_aside):
    return [f"{r}: model room under it not applied: reached from above ({w}) -- "
            f"declare heights[].underside_mm to override"
            for r, w in sorted(set_aside.items())]


def _grid_union(base, box):
    """(x0, y0, lo) of a raster spanning `base`'s and local `box`, on
    `base`'s cell grid (else one aligned to the cell size), `base`'s cells
    copied in and `inf` elsewhere."""
    import numpy as np
    if base is None:
        x0 = math.floor(box[0] / CELL_MM) * CELL_MM
        y0 = math.floor(box[1] / CELL_MM) * CELL_MM
        nx = int(math.ceil((box[2] - x0) / CELL_MM))
        ny = int(math.ceil((box[3] - y0) / CELL_MM))
        return x0, y0, np.full((max(nx, 1), max(ny, 1)), np.inf)
    bx, by = base.lo.shape
    i0 = min(0, math.floor((box[0] - base.x0) / CELL_MM))
    j0 = min(0, math.floor((box[1] - base.y0) / CELL_MM))
    i1 = max(bx, math.ceil((box[2] - base.x0) / CELL_MM))
    j1 = max(by, math.ceil((box[3] - base.y0) / CELL_MM))
    lo = np.full((i1 - i0, j1 - j0), np.inf)
    lo[-i0:-i0 + bx, -j0:-j0 + by] = base.lo
    return base.x0 + i0 * CELL_MM, base.y0 + j0 * CELL_MM, lo


def _for_board(pcb_data, pcb_file):
    path = pcb_file or getattr(pcb_data, 'source_path', None)
    if not path or not os.path.isfile(path):
        return EMPTY
    try:
        with open(path, encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return EMPTY
    from kicad_parser import iter_footprint_blocks
    from model_export import model_dirs, resolve_cli, stage_models
    names = {}
    for _s, _e, fp, _raw, k in iter_footprint_blocks(text):
        names[k] = [m.group(1) for m in re.finditer(r'\(model\s+"([^"]+)"', fp)]
    cli, _why = resolve_cli()
    if not cli:
        return Heights(top=_ipc_tops(pcb_data, {}, names),
                       note='no kicad-cli: heights from IPC names only')
    board_dir = os.path.dirname(os.path.abspath(path))
    staged, _counts, _missing, _f = stage_models(text, model_dirs(cli, board_dir), board_dir)
    key = _key(staged, cli, iter_footprint_blocks)
    if key in _MEMO:
        return _MEMO[key]
    cache = os.path.join(os.environ.get('XDG_CACHE_HOME') or os.path.expanduser('~/.cache'),
                         'krt_heights', key + '.json')
    raw = None
    if os.path.isfile(cache):
        try:
            with open(cache, encoding='utf-8') as f:
                raw = json.load(f)
        except (OSError, ValueError):
            raw = None
    failed = 'the 3D export failed'
    if raw is None:
        raw = _measure(staged, cli, pcb_data)
        if isinstance(raw, str):
            failed, raw = raw, None
        if raw is not None:
            try:
                os.makedirs(os.path.dirname(cache), exist_ok=True)
                tmp = cache + f'.{os.getpid()}'
                with open(tmp, 'w', encoding='utf-8') as f:
                    json.dump(raw, f)
                os.replace(tmp, cache)
            except OSError:
                pass
    h = _load(raw, pcb_data, path, names) if raw is not None else Heights(
        top=_ipc_tops(pcb_data, {}, names),
        note=f'{failed}: heights from IPC names only')
    #: the measurement as read, every part's raster unfiltered: the facts a
    #: cover is declared from (`py_tools/cover_facts.py`)
    h.raw = raw
    _MEMO[key] = h
    return h


def _key(staged, cli, iter_blocks):
    """What the measurement depends on: each footprint's layer, models (the
    blocks, and the files' sizes and times) and pads, and kicad-cli -- never
    its pose, so every seed of a placement reads one measurement."""
    h = hashlib.sha1(f'v{_CACHE_VERSION}\0{cli}'.encode())
    for _s, _e, fp, _raw, key in iter_blocks(staged):
        m = re.search(r'\(layer\s+"([^"]+)"', fp)
        h.update(f'{key}\0{m.group(1) if m else ""}\0'.encode())
        for tok in ('(model ', '(pad '):
            i = fp.find(tok)
            while i >= 0:
                blk = _sexpr(fp, i)
                h.update(re.sub(r'\s+', ' ', blk).encode())
                if tok == '(model ':
                    f = re.match(r'\(model\s+"([^"]+)"', blk)
                    try:
                        st = os.stat(f.group(1))
                        h.update(f'{st.st_size}:{int(st.st_mtime)}'.encode())
                    except (OSError, AttributeError):
                        h.update(b'?')
                i = fp.find(tok, i + len(blk))
    return h.hexdigest()[:20]


def _sexpr(text, i):
    """The balanced s-expression starting at `text[i]` ('('), strings skipped."""
    depth, j, n = 0, i, len(text)
    while j < n:
        c = text[j]
        if c == '"':
            j += 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == '\\' else 1
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return text[i:j + 1]
        j += 1
    return text[i:]


def _ipc_tops(pcb_data, have, names):
    """{ref: (mm, 'ipc_name')} for the parts `have` leaves out whose model
    file or footprint carries an IPC-7351 name."""
    out = {}
    for ref, fp in (pcb_data.footprints or {}).items():
        if ref in have:
            continue
        h = None
        for name in list(names.get(ref) or ()) + [fp.footprint_name or '']:
            h = ipc_height(name)
            if h:
                break
        if h:
            out[ref] = (h, 'ipc_name')
    return out


def _measure(staged, cli, pcb_data):
    """Export the staged board's parts and measure them: {'top': {ref: mm},
    'covers': {ref: [x0, y0, nx, ny, hex of uint16 centi-mm]}}, None when the
    export failed, or the reason when it timed out (never cached, so the next
    run measures again)."""
    import numpy as np
    td = tempfile.mkdtemp(prefix='krt_heights_')
    try:
        brd = os.path.join(td, 'b.kicad_pcb')
        with open(brd, 'w', encoding='utf-8') as f:
            f.write(staged)
        glb = os.path.join(td, 'b.glb')
        try:
            r = subprocess.run([cli, 'pcb', 'export', 'glb', '--no-dnp', '--subst-models',
                                '--no-board-body', '-f', '-o', glb, brd],
                               capture_output=True, timeout=EXPORT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return f'the 3D export timed out after {EXPORT_TIMEOUT_S} s'
        except OSError:
            return None
        if r.returncode != 0 or not os.path.isfile(glb):
            return None
        with open(glb, 'rb') as f:
            data = f.read()
    finally:
        import shutil
        shutil.rmtree(td, ignore_errors=True)
    from kicad_parser import footprint_at_match, iter_footprint_blocks
    poses = {}
    for _s, _e, fp, _raw, k in iter_footprint_blocks(staged):
        m = footprint_at_match(fp)
        if m:
            poses[k] = (float(m.group(1)), float(m.group(2)),
                        float(m.group(3)) if m.group(3) else 0.0)
    tris, norms, roots = _glb_parts(data, set(pcb_data.footprints or {}) & set(poses))
    if not roots:
        return {'top': {}, 'covers': {}}
    fps = pcb_data.footprints
    refs = sorted(roots)
    T = np.array([roots[r][:3, 3] for r in refs])
    P = np.array([poses[r][:2] for r in refs])
    best = None
    for up in range(3):
        pl = [k for k in range(3) if k != up]
        for a, b in ((pl[0], pl[1]), (pl[1], pl[0])):
            for sx in (1, -1):
                for sy in (1, -1):
                    for u in (1000.0, 1.0):
                        gx, gy = sx * T[:, a] * u, sy * T[:, b] * u
                        ox, oy = np.median(P[:, 0] - gx), np.median(P[:, 1] - gy)
                        err = float(np.median(np.abs(P[:, 0] - gx - ox) + np.abs(P[:, 1] - gy - oy)))
                        if best is None or err < best[0]:
                            best = (err, up, a, b, sx, sy, u, ox, oy)
    err, up, ax, ay, sx, sy, u, ox, oy = best
    if err > 0.01:
        return None
    from placement.legality import footprint_side
    side = {r: footprint_side(fps[r]) for r in refs}
    fz = [roots[r][up, 3] for r in refs if side[r] == 'F']
    bz = [roots[r][up, 3] for r in refs if side[r] == 'B']
    if not fz and not bz:
        return None
    # which way is out of the front face: a front part's body is on that side
    outward = 1.0
    for r in refs:
        if side[r] == 'F' and r in tris:
            zs = tris[r][:, :, up]
            outward = 1.0 if np.median(zs) >= roots[r][up, 3] else -1.0
            break
    else:
        for r in refs:
            if side[r] == 'B' and r in tris:
                zs = tris[r][:, :, up]
                outward = -1.0 if np.median(zs) <= roots[r][up, 3] else 1.0
                break
    top_face = float(np.median(fz)) if fz else None
    bot_face = float(np.median(bz)) if bz else None
    out_top, out_cov, out_body = {}, {}, {}
    for r in refs:
        t = tris.get(r)
        if t is None:
            continue
        fp = fps[r]
        face = top_face if side[r] == 'F' else bot_face
        sgn = outward if side[r] == 'F' else -outward
        bx = sx * t[:, :, ax] * u + ox
        by = sy * t[:, :, ay] * u + oy
        h = sgn * (t[:, :, up] - face) * u
        px, py, prot = poses[r]
        rot = math.radians(prot % 360)
        c, s = math.cos(rot), math.sin(rot)
        dx, dy = bx - px, by - py
        q = np.stack([dx * c - dy * s, dx * s + dy * c, h], 2)
        nh = sgn * norms[r][:, up]           # each triangle's normal, away from the face
        keep = q[:, :, 2].max(1) > _ABOVE_MM
        q, nh = q[keep], nh[keep]
        if not len(q):
            continue
        if not _sits_on(q, fp):
            continue          # a model that is not over its pads says nothing
        out_top[r] = round(float(q[:, :, 2].max()), 3)
        out_body[r] = _raster(q, high=True)
        # the room under a body is read off the surfaces that face the board
        # or stand on it, never off a top
        cov = _raster(q[nh <= _UP_FACING])
        if cov is not None:
            out_cov[r] = cov
    return {'top': out_top, 'covers': out_cov, 'bodies': out_body}


def _sits_on(q, fp):
    """Does the model's plan cover the footprint's pad centre (local frame)?"""
    pads = [p for p in (fp.pads or ())]
    if not pads:
        return True
    cx = sum(p.local_x for p in pads) / len(pads)
    cy = sum(p.local_y for p in pads) / len(pads)
    tol = 0.5
    return (q[:, :, 0].min() - tol <= cx <= q[:, :, 0].max() + tol
            and q[:, :, 1].min() - tol <= cy <= q[:, :, 1].max() + tol)


def _raster(q, high=False):
    """A cover's raster [x0, y0, nx, ny, hex] of its lowest point per cell, or
    None when no cell of the model is raised `LIFT_MIN_MM` -- a body that
    sits on the board. `high`: every part's raster of its HIGHEST point per
    cell instead, never None."""
    import numpy as np
    if not len(q):
        return None
    q = _subdivide(q)
    x0 = math.floor(q[:, :, 0].min() / CELL_MM) * CELL_MM
    y0 = math.floor(q[:, :, 1].min() / CELL_MM) * CELL_MM
    nx = int(math.ceil((q[:, :, 0].max() - x0) / CELL_MM)) + 1
    ny = int(math.ceil((q[:, :, 1].max() - y0) / CELL_MM)) + 1
    lo = np.full((nx, ny), np.inf)
    i0 = np.floor((q[:, :, 0].min(1) - x0) / CELL_MM).astype(int)
    i1 = np.floor((q[:, :, 0].max(1) - x0) / CELL_MM).astype(int)
    j0 = np.floor((q[:, :, 1].min(1) - y0) / CELL_MM).astype(int)
    j1 = np.floor((q[:, :, 1].max(1) - y0) / CELL_MM).astype(int)
    if high:
        hi = np.full((nx, ny), -np.inf)
        hm = q[:, :, 2].max(1)
        for di in (0, 1):
            for dj in (0, 1):
                np.maximum.at(hi, (np.minimum(i0 + di, i1), np.minimum(j0 + dj, j1)), hm)
        cm = np.where(np.isfinite(hi), np.clip(np.round(hi * 100), 0, 65534), 65535).astype('<u2')
        return [round(x0, 4), round(y0, 4), nx, ny, cm.tobytes().hex()]
    hm = np.maximum(0.0, q[:, :, 2].min(1))
    # a subdivided triangle spans at most two cells each way
    for di in (0, 1):
        for dj in (0, 1):
            np.minimum.at(lo, (np.minimum(i0 + di, i1), np.minimum(j0 + dj, j1)), hm)
    fin = np.isfinite(lo)
    if not (fin & (lo >= LIFT_MIN_MM)).any():
        return None
    cm = np.where(fin, np.minimum(np.round(lo * 100), 65534), 65535).astype('<u2')
    return [round(x0, 4), round(y0, 4), nx, ny, cm.tobytes().hex()]


def _subdivide(t):
    """Split (n, 3, 3) triangles (x, y, h) until no plan edge is longer than a
    cell, so a triangle's own bbox stands for the cells it covers."""
    import numpy as np
    out = []
    while len(t):
        e = np.max([np.hypot(*(t[:, i, :2] - t[:, (i + 1) % 3, :2]).T) for i in range(3)], 0)
        small = e <= CELL_MM
        out.append(t[small])
        t = t[~small]
        if not len(t):
            break
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        t = np.concatenate([np.stack(k, 1) for k in ((a, ab, ca), (ab, b, bc), (ca, bc, c),
                                                     (ab, bc, ca))])
    return np.concatenate(out)


def _glb_parts(data, refs):
    """({ref: (n, 3, 3) world triangles}, {ref: (n, 3) their unit normals,
    world frame}, {ref: world matrix of its node}). A triangle's normal is
    its vertices' mean where the mesh gives them, else its winding's."""
    import numpy as np
    if len(data) < 20 or struct.unpack('<I', data[:4])[0] != 0x46546C67:
        return {}, {}, {}
    jl = struct.unpack('<I', data[12:16])[0]
    g = json.loads(data[20:20 + jl])
    off = 20 + jl
    bin_ = b''
    if off + 8 <= len(data):
        bl = struct.unpack('<I', data[off:off + 4])[0]
        bin_ = data[off + 8:off + 8 + bl]
    ct = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
    nc = {'SCALAR': 1, 'VEC3': 3}

    def acc(i):
        a = g['accessors'][i]
        bv = g['bufferViews'][a['bufferView']]
        dt = np.dtype(ct[a['componentType']]).newbyteorder('<')
        n = nc[a['type']]
        start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        stride = bv.get('byteStride', 0)
        if stride and stride != dt.itemsize * n:
            arr = np.ndarray((a['count'], n), dt, bin_, start, (stride, dt.itemsize))
        else:
            arr = np.frombuffer(bin_, dt, a['count'] * n, start).reshape(a['count'], n)
        return np.array(arr, dtype=np.float64 if dt.kind == 'f' else np.int64)

    def local(n):
        if 'matrix' in n:
            return np.array(n['matrix'], float).reshape(4, 4).T
        x, y, z, w = n.get('rotation', [0, 0, 0, 1])
        s = n.get('scale', [1, 1, 1])
        t = n.get('translation', [0, 0, 0])
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        m = np.eye(4)
        m[:3, :3] = r * np.array(s)
        m[:3, 3] = t
        return m

    tris, norms, roots = {}, {}, {}
    stack = [(i, np.eye(4), None) for i in g['scenes'][g.get('scene', 0)]['nodes']]
    while stack:
        i, m, own = stack.pop()
        node = g['nodes'][i]
        w = m @ local(node)
        name = node.get('name')
        # kicad-cli names a part's node by its bare reference; a duplicate
        # reference's later blocks cannot be told apart, so only the first
        if own is None and name in refs and name not in roots:
            own = name
            roots[name] = w
        if 'mesh' in node and own:
            for prim in g['meshes'][node['mesh']]['primitives']:
                if prim.get('mode', 4) != 4:
                    continue
                v = acc(prim['attributes']['POSITION'])
                v = (w[:3, :3] @ v.T).T + w[:3, 3]
                idx = (acc(prim['indices']).reshape(-1) if 'indices' in prim
                       else np.arange(len(v)))
                idx = idx[:len(idx) // 3 * 3].reshape(-1, 3)
                t = v[idx]
                if 'NORMAL' in prim['attributes']:
                    nv = acc(prim['attributes']['NORMAL'])
                    nv = (np.linalg.inv(w[:3, :3]).T @ nv.T).T
                    n = nv[idx].mean(1)
                else:
                    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
                    if np.linalg.det(w[:3, :3]) < 0:
                        n = -n
                n = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
                tris.setdefault(own, []).append(t)
                norms.setdefault(own, []).append(n)
        for ch in node.get('children', []):
            stack.append((ch, w, own))
    return ({r: np.concatenate(v) for r, v in tris.items()},
            {r: np.concatenate(v) for r, v in norms.items()}, roots)


def _load(raw, pcb_data, pcb_file, names) -> Heights:
    """`Heights` from a measurement, with each cover's drawn courtyard marked
    and the IPC names filling the heights the models left unknown."""
    import numpy as np
    top = {r: (float(h), 'model') for r, h in (raw.get('top') or {}).items()
           if r in (pcb_data.footprints or {})}
    top.update(_ipc_tops(pcb_data, top, names))
    covers = {}
    reached = _reached(pcb_data)
    set_aside = {}
    shapes, occ = {}, {}
    if raw.get('covers'):
        try:
            from placement.body import board_bodies
            for r, g in board_bodies(pcb_data, pcb_file).items():
                shapes[r] = g.court_shape_local
                occ[r] = g.occupancy_local
        except Exception:                                    # noqa: BLE001
            shapes = {}
        shapes = {r: _no_go(pcb_data.footprints[r], shapes.get(r))
                  for r in raw['covers'] if r in (pcb_data.footprints or {})}
    for r, (x0, y0, nx, ny, hx) in (raw.get('covers') or {}).items():
        if r not in top:
            continue
        cm = np.frombuffer(bytes.fromhex(hx), '<u2').reshape(nx, ny)
        lo = np.where(cm == 65535, np.inf, cm / 100.0)
        court = _mask(shapes.get(r), x0, y0, nx, ny)
        solid = r in reached
        if solid and _room_inside(lo, court, x0, y0, occ.get(r)) >= ROOM_MIN_MM2:
            # the room its model shows over its occupancy is kept solid:
            # said, never silent (its raised body past it still binds)
            set_aside[r] = reached[r]
        if _room(lo, court, x0, y0, occ.get(r), solid) >= ROOM_MIN_MM2:
            covers[r] = Cover(r, x0, y0, lo, court, solid)
    n_model = sum(1 for v in top.values() if v[1] == 'model')
    n_room = sum(1 for c in covers.values() if not c.solid)
    return Heights(top, covers, note=f'{n_model} part heights from models, '
                                     f'{len(top) - n_model} from IPC names, '
                                     f'{n_room} part(s) with room under them',
                   reached=reached, bodies=_bodies(raw.get('bodies') or {}, top),
                   set_aside=set_aside, notices=_set_aside_notices(set_aside))


def _bodies(raw, top):
    """{ref: (cell centres (n, 2), the top over each (n,))}, local frame."""
    import numpy as np
    out = {}
    for r, (x0, y0, nx, ny, hx) in raw.items():
        if r not in top:
            continue
        cm = np.frombuffer(bytes.fromhex(hx), '<u2').reshape(nx, ny)
        ii, jj = np.nonzero(cm != 65535)
        if not len(ii):
            continue
        pts = np.stack([x0 + (ii + 0.5) * CELL_MM, y0 + (jj + 0.5) * CELL_MM], 1)
        out[r] = (pts, cm[ii, jj] / 100.0,
                  (float(pts[:, 0].min()) - CELL_MM, float(pts[:, 1].min()) - CELL_MM,
                   float(pts[:, 0].max()) + CELL_MM, float(pts[:, 1].max()) + CELL_MM))
    return out


def _is_test_point(fp, ref):
    from placement.part_class import classify_part
    try:
        return classify_part(fp, ref).name == 'testpoint'
    except Exception:                                        # noqa: BLE001
        return False


def _reached(pcb_data):
    """The parts something reaches from above: those `part_class` classes,
    and those whose designator is a connector's, a switch's, a test point's
    or a mounting part's (`part_class`'s prefix tables) -- a connector drawn
    with a library's own name is still a connector, and a crystal called X1
    kept out from under a cover costs nothing. {ref: why}, the reason as a
    reader prints it."""
    from placement.part_class import (ACTUATOR_PREFIX, CONN_PREFIX_LOW, CONN_PREFIX_MED,
                                      _designator, _prefix, classify_part)
    prefixes = set(CONN_PREFIX_MED) | set(CONN_PREFIX_LOW) | set(ACTUATOR_PREFIX)
    out = {}
    for ref, fp in (pcb_data.footprints or {}).items():
        try:
            name = classify_part(fp, ref).name
            if name is not None:
                out[ref] = f'classed {name}'
            elif _prefix(ref) in prefixes:
                out[ref] = f'reference prefix {_prefix(ref)}'
            elif _designator(ref) in prefixes:
                out[ref] = f'designator {_designator(ref)}'
        except Exception:                                    # noqa: BLE001
            out[ref] = 'its class could not be read'
    return out


def _room(lo, court, x0, y0, occ, solid=False):
    """The larger of a raster's room for the shortest part (none in a `solid`
    part) and its raised overhang past the part's own occupancy box, mm^2."""
    import numpy as np
    free = np.isfinite(lo) & ~court
    room = 0 if solid else (free & (lo >= max(MARGIN_MM + _SHORTEST_MM, ROOM_FLOOR_MM))).sum()
    over = 0
    if occ is not None:
        nx, ny = lo.shape
        cx = x0 + (np.arange(nx) + 0.5) * CELL_MM
        cy = y0 + (np.arange(ny) + 0.5) * CELL_MM
        out = ~(((cx >= occ[0]) & (cx <= occ[2]))[:, None] & ((cy >= occ[1]) & (cy <= occ[3]))[None, :])
        over = (free & (lo >= LIFT_MIN_MM) & out).sum()
    return max(room, over) * CELL_MM * CELL_MM


def _room_inside(lo, court, x0, y0, occ):
    """A raster's room for the shortest part over the part's own occupancy
    box (the whole raster when it has none), mm^2: what the reached rule
    keeps solid."""
    import numpy as np
    room = np.isfinite(lo) & ~court & (lo >= max(MARGIN_MM + _SHORTEST_MM, ROOM_FLOOR_MM))
    if occ is not None:
        nx, ny = lo.shape
        cx = x0 + (np.arange(nx) + 0.5) * CELL_MM
        cy = y0 + (np.arange(ny) + 0.5) * CELL_MM
        room &= (((cx >= occ[0]) & (cx <= occ[2]))[:, None]
                 & ((cy >= occ[1]) & (cy <= occ[3]))[None, :])
    return room.sum() * CELL_MM * CELL_MM


def _no_go(fp, court):
    """A cover's no-go shape, local frame: its drawn courtyard united with its
    pads' copper."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from check_pads import custom_pad_copper, pad_outline_polygon
    from placement.legality import _pad_carries_copper
    parts = [court] if court is not None else []
    pose = (fp.x, fp.y, fp.rotation or 0.0)
    r = math.radians(pose[2] % 360)
    c, s = math.cos(r), math.sin(r)
    for pad in (fp.pads or ()):
        if not _pad_carries_copper(pad):
            continue
        try:
            pp = custom_pad_copper(pad)
            if pp is None:
                pts = pad_outline_polygon(pad)
                pp = Polygon(pts) if len(pts) >= 3 else None
        except Exception:                                    # noqa: BLE001
            pp = None
        if pp is None or not pp.is_valid:
            continue
        loc = [((x - pose[0]) * c - (y - pose[1]) * s, (x - pose[0]) * s + (y - pose[1]) * c)
               for x, y in pp.exterior.coords]
        parts.append(Polygon(loc))
    return unary_union(parts) if parts else None


def _mask(shape, x0, y0, nx, ny):
    """The cells a drawn courtyard reaches (any overlap), local frame."""
    import numpy as np
    m = np.zeros((nx, ny), bool)
    if shape is None:
        return m
    try:
        from shapely.geometry import box
        from shapely.prepared import prep
        b = shape.bounds
        ps = prep(shape)
        for i in range(max(0, math.floor((b[0] - x0) / CELL_MM)),
                       min(nx, math.ceil((b[2] - x0) / CELL_MM))):
            for j in range(max(0, math.floor((b[1] - y0) / CELL_MM)),
                           min(ny, math.ceil((b[3] - y0) / CELL_MM))):
                cx, cy = x0 + i * CELL_MM, y0 + j * CELL_MM
                if ps.intersects(box(cx, cy, cx + CELL_MM, cy + CELL_MM)):
                    m[i, j] = True
    except Exception:                                        # noqa: BLE001
        m[:] = True
    return m
