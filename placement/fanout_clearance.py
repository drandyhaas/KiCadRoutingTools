"""
Fanout-clearance placement repair: tidy decoupling caps around BGA fanout.

Issue #130: a BGA fanout drops vias near the ball field. Where a foreign-net
via lands under a decoupling cap, the via copper overlaps the cap pad -> a
real PAD-VIA DRC violation at the clearance floor. The root cause is
*placement*, so the fix is to nudge the caps.

This runs AFTER bga_fanout.py (so the real vias exist), because the fanout
usually only escapes *signal* balls; GND/power balls are connected later by
dropping vias straight down. That gives two complementary goals:

  * AVOID  - a cap pad must clear, by `clearance`, every real fanout via of a
             DIFFERENT net (these are the #130 violations), plus any foreign
             escape track on the cap's own copper side (escapes can land on the
             bottom), plus any foreign-net COMPONENT pad (#235: a move may never
             slide a cap pad onto a neighbour's pad -> a PAD-PAD short).
             Same-net vias/pads are fine - a cap pad may sit right on one
             (via-in-pad / same-net copper sharing).
  * ATTRACT - pull each cap pad toward the nearest BGA ball of its OWN net, so
             that a later GND/power via dropped at the ball also lands on the
             cap pad (one shared via connects ball + cap + plane).

Move set: small nudges within a per-cap displacement budget plus 90-degree
rotations. Cost = foreign-via penetration (strong) + same-net attraction +
displacement (mild, so caps stay near their seed). The board edge,
locked-part courtyards, AND other caps' courtyards are HARD constraints: a
move may never introduce or worsen a courtyard overlap, so caps never end up
on top of each other. A cap that can't clear a foreign via within the base
budget gets its budget grown and rotations enabled until it fits or hits the
displacement cap; if it still can't, it's reported unresolved.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Set, Tuple

from kicad_parser import PCBData, find_components_by_type
from bga_fanout.grid import analyze_bga_grid
from placement.parser import extract_courtyard_bboxes, extract_locked_refs
from placement.utility import compute_footprint_bbox_local, snap_to_grid

ROTATIONS = [0.0, 90.0, 180.0, 270.0]
EPS = 1e-6

# Objective weights. foreign-via clearance dominates everything, then pulling
# pads onto same-net balls, then a mild displacement regularizer (so caps that
# are already fine and far from a same-net ball stay put). Cap-cap /
# locked-part overlap is a HARD constraint (see hard_blocked), not weighted.
VIA_WEIGHT = 50.0
ATTRACT_WEIGHT = 1.0
DISPLACEMENT_WEIGHT = 0.3


def _rotate_local_bounds(lmin_x, lmin_y, lmax_x, lmax_y, rotation):
    """Rotate a local bounding box by the footprint rotation (KiCad sign)."""
    rot = rotation % 360
    if abs(rot) < 0.01:
        return lmin_x, lmin_y, lmax_x, lmax_y
    angle = math.radians(-rot)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    corners = [(lmin_x, lmin_y), (lmax_x, lmin_y),
               (lmin_x, lmax_y), (lmax_x, lmax_y)]
    xs = [x * cos_a - y * sin_a for x, y in corners]
    ys = [x * sin_a + y * cos_a for x, y in corners]
    return min(xs), min(ys), max(xs), max(ys)


def _rect_gap(a, b):
    """Smallest axis-aligned gap between two rects (negative if overlapping)."""
    dx = max(a[0] - b[2], b[0] - a[2])
    dy = max(a[1] - b[3], b[1] - a[3])
    if dx < 0 and dy < 0:
        return max(dx, dy)
    return math.hypot(max(dx, 0), max(dy, 0)) if (dx > 0 and dy > 0) else max(dx, dy)


def _point_to_rect_dist(px, py, rect):
    """Distance from a point to an axis-aligned rect (0 if inside)."""
    dx = max(rect[0] - px, px - rect[2], 0.0)
    dy = max(rect[1] - py, py - rect[3], 0.0)
    return math.hypot(dx, dy)


def _point_to_seg_dist(px, py, x1, y1, x2, y2):
    """Distance from a point to a line segment."""
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


class _Cap:
    """A movable cap: pad offsets + courtyard bbox, in a seed-relative frame.

    Pad offsets and board-resolved half-sizes are captured at the seed
    placement; an additional 90-degree rotation rotates the offsets and swaps
    the half-extents, which is exact for axis-aligned pads (the normal case
    for decoupling caps).
    """

    def __init__(self, fp, courtyard_local):
        self.ref = fp.reference
        self.side = 'B' if (fp.layer or '').startswith('B') else 'F'
        self.seed_x, self.seed_y = fp.x, fp.y
        self.seed_rot = fp.rotation % 360
        self.x, self.y, self.rot = fp.x, fp.y, fp.rotation % 360
        # pads: (off_x, off_y, half_x, half_y, net_id) relative to fp center.
        # Copper pads only -- a footprint may define solder-paste apertures as
        # separate paste-only "pads" (e.g. gkl_misc C_0201_0603Metric), which are
        # not copper and must not enter the clearance/attraction geometry.
        self.pads = []
        for p in fp.pads:
            if not any(str(l).endswith('.Cu') for l in p.layers):
                continue  # paste/mask-only aperture, not copper
            off_x = p.global_x - fp.x
            off_y = p.global_y - fp.y
            tilt = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
            c, s = abs(math.cos(tilt)), abs(math.sin(tilt))
            hx, hy = p.size_x / 2, p.size_y / 2
            half_x = hx * c + hy * s
            half_y = hx * s + hy * c
            self.pads.append((off_x, off_y, half_x, half_y, p.net_id))
        self.local_bounds = courtyard_local
        # Rotation-only geometry is reused across every candidate position
        # (millions of times on a dense board), so memoize it per angle and
        # just translate per call. Keyed by rounded rotation.
        self._rect_cache: Dict[float, Tuple[float, float, float, float]] = {}
        self._pad_cache: Dict[float, list] = {}

    def rect(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        # local_bounds is the footprint-LOCAL courtyard; rotate by the
        # absolute placement angle (matching how static obstacle rects are
        # built), not the delta from seed. The rotation is position-independent
        # -> cache it and translate.
        key = round(rot, 3)
        b = self._rect_cache.get(key)
        if b is None:
            b = _rotate_local_bounds(*self.local_bounds, rot)
            self._rect_cache[key] = b
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def pad_rects(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        key = round((rot - self.seed_rot) % 360, 3)
        cache = self._pad_cache.get(key)
        if cache is None:
            delta = (rot - self.seed_rot) % 360
            rad = math.radians(-delta)
            c, s = math.cos(rad), math.sin(rad)
            swap = round(delta) % 180 == 90
            cache = []
            for off_x, off_y, hx, hy, net in self.pads:
                ox = off_x * c - off_y * s
                oy = off_x * s + off_y * c
                HX, HY = (hy, hx) if swap else (hx, hy)
                cache.append((ox, oy, HX, HY, net))
            self._pad_cache[key] = cache
        out = []
        for ox, oy, HX, HY, net in cache:
            cx, cy = x + ox, y + oy
            out.append((cx - HX, cy - HY, cx + HX, cy + HY, net))
        return out


def _candidate_positions(cap, max_disp, step, grid_step):
    seen = set()
    out = []
    n = int(max_disp / step)
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            cx = cap.seed_x + ix * step
            cy = cap.seed_y + iy * step
            if math.hypot(cx - cap.seed_x, cy - cap.seed_y) > max_disp + 1e-9:
                continue
            cx = snap_to_grid(cx, grid_step)
            cy = snap_to_grid(cy, grid_step)
            key = (round(cx, 4), round(cy, 4))
            if key not in seen:
                seen.add(key)
                out.append((cx, cy))
    return out


class _Repair:
    def __init__(self, pcb_data: PCBData, pcb_file: str,
                 clearance: float, grid_step: float,
                 board_edge_clearance: float, near_margin: float,
                 capture_radius: float, default_via_size: float,
                 cap_prefix: str, extra_locked: Set[str],
                 max_displacement_cap: float = 3.0):
        bounds = pcb_data.board_info.board_bounds
        if bounds is None:
            raise ValueError("No board boundary (Edge.Cuts) found")
        self.board = bounds
        margin = max(clearance, board_edge_clearance)
        self.usable = (bounds[0] + margin, bounds[1] + margin,
                       bounds[2] - margin, bounds[3] - margin)
        self.clearance = clearance
        self.grid_step = grid_step
        self.capture_radius = capture_radius
        # cap_prefix may list several reference prefixes (e.g. "C,R" = caps and
        # resistors); str.startswith() accepts the tuple directly.
        self._cap_prefixes = tuple(p.strip() for p in str(cap_prefix).split(',')
                                   if p.strip()) or ('C',)

        courtyards = extract_courtyard_bboxes(pcb_file)
        locked = set(extract_locked_refs(pcb_file)) | extra_locked
        self.locked_refs = locked

        # --- avoidance: the REAL fanout vias (after fanout) ---
        # Each (x, y, net, keepout) where keepout = via_radius + clearance; a
        # cap pad must clear vias of a DIFFERENT net by this. Through-vias, so
        # they apply to caps on either board side.
        self.vias: List[Tuple[float, float, int, float]] = []
        for v in pcb_data.vias:
            size = v.size if v.size and v.size > 0 else default_via_size
            self.vias.append((v.x, v.y, v.net_id, size / 2.0 + clearance))

        # --- avoidance: foreign-net tracks on the cap's own side ---
        # Fanout escapes can land on the bottom (cap) side; attraction could
        # then pull a cap onto an escape track -> a PAD-SEGMENT violation.
        # Keyed by side so a B.Cu cap only avoids B.Cu tracks.
        self.segments: List[Tuple[float, float, float, float, int, float, str]] = []
        for s in pcb_data.segments:
            side = 'B' if (s.layer or '').startswith('B') else 'F'
            self.segments.append((s.start_x, s.start_y, s.end_x, s.end_y,
                                  s.net_id, s.width / 2.0 + clearance, side))

        # --- attraction: BGA balls grouped by net ---
        # A cap pad is pulled toward the nearest ball of its own net, so a via
        # dropped later at that ball also lands on the cap pad. Restricted to
        # real BGA footprints (detect_package_type == 'BGA').
        self.attract: Dict[int, List[Tuple[float, float]]] = {}
        bga_bboxes: List[Tuple[float, float, float, float]] = []
        self.bga_refs: List[str] = []
        for fp in find_components_by_type(pcb_data, 'BGA'):
            self.bga_refs.append(fp.reference)
            grid = analyze_bga_grid(fp)
            if grid is not None:
                bga_bboxes.append((grid.min_x, grid.min_y, grid.max_x, grid.max_y))
            else:
                lb = compute_footprint_bbox_local(fp)
                b = _rotate_local_bounds(*lb, fp.rotation)
                bga_bboxes.append((fp.x + b[0], fp.y + b[1],
                                   fp.x + b[2], fp.y + b[3]))
            for p in fp.pads:
                if p.net_id > 0:
                    self.attract.setdefault(p.net_id, []).append(
                        (p.global_x, p.global_y))
        # Kept for visualization (animate_fanout_clearance.py): the BGA ball-field
        # bounding boxes that define the region of interest.
        self.bga_bboxes = bga_bboxes

        # --- static obstacle rects (locked parts incl. BGAs) ---
        # and movable caps near a BGA. Courtyard collisions are checked
        # per board side only: a back-side decoupling cap legitimately sits
        # under a top-side BGA (they overlap in XY but not in copper). Via
        # disks are through-vias, so they apply to caps on either side.
        # --- avoidance: foreign-net COMPONENT pads (#235) ---
        # The cap optimizer must not slide a cap pad onto a neighbouring
        # component's pad of a different net -> a PAD-PAD short. Each entry is
        # an axis-aligned pad rect plus its net and side ('F'/'B', or None for
        # a through-hole pad that blocks both sides). Same-net pads are fine.
        self.foreign_pads: List[
            Tuple[float, float, float, float, int, Optional[str]]] = []
        self.caps: Dict[str, _Cap] = {}
        self.static_rects: List[Tuple[Tuple[float, float, float, float], str]] = []
        for ref, fp in pcb_data.footprints.items():
            if not fp.pads:
                continue
            lb = courtyards.get(ref) or compute_footprint_bbox_local(fp)
            # Count COPPER pads only: paste-only apertures (split-paste 0201
            # footprints) would otherwise push a 2-terminal cap past the 2-pad
            # test and wrongly exclude it from placement (#130).
            n_copper = sum(1 for p in fp.pads
                           if any(str(l).endswith('.Cu') for l in p.layers))
            is_cap = (ref.startswith(self._cap_prefixes) and n_copper <= 2
                      and ref not in locked)
            if is_cap:
                cap = _Cap(fp, lb)
                if self._near_any(cap.rect(), bga_bboxes, near_margin):
                    self.caps[ref] = cap
                    continue
            # everything else is a static obstacle
            side = 'B' if (fp.layer or '').startswith('B') else 'F'
            b = _rotate_local_bounds(*lb, fp.rotation)
            self.static_rects.append(((fp.x + b[0], fp.y + b[1],
                                       fp.x + b[2], fp.y + b[3]), side))
            # record this part's copper pads as foreign-pad keep-outs
            for p in fp.pads:
                copper = [l for l in p.layers if str(l).endswith('.Cu')]
                if not copper:
                    continue  # paste/mask-only aperture
                through = (p.drill or 0) > 0
                pside = None if through else (
                    'B' if any(str(l).startswith('B') for l in copper) else 'F')
                tilt = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
                c, s = abs(math.cos(tilt)), abs(math.sin(tilt))
                hx, hy = p.size_x / 2, p.size_y / 2
                half_x = hx * c + hy * s
                half_y = hx * s + hy * c
                self.foreign_pads.append(
                    (p.global_x - half_x, p.global_y - half_y,
                     p.global_x + half_x, p.global_y + half_y,
                     p.net_id, pside))

        # Per-cap spatially-pruned neighbour lists (perf). A cap moves at most
        # max_displacement_cap from its seed, so anything whose seed gap already
        # exceeds that (plus the relevant spans / clearance) can NEVER constrain
        # it -- excluding it is exact, not an approximation. This turns the
        # per-candidate hard_blocked / penalty loops from O(all parts) into
        # O(handful), the dominant cost on dense boards (#213 profiling).
        cap_geom: Dict[str, Tuple[float, float, float, Tuple]] = {}
        for ref, cap in self.caps.items():
            r = cap.rect()
            cx, cy = (r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0
            span = math.hypot(r[2] - cx, r[3] - cy)
            cap_geom[ref] = (cx, cy, span, r)

        self.cap_foreign_pads: Dict[str, List[
            Tuple[float, float, float, float, int, Optional[str]]]] = {}
        # static obstacle rects (with their global index, same side) in reach
        self.cap_static: Dict[str, List[Tuple[int, Tuple]]] = {}
        # other movable caps (refs, same side) that could ever touch this one
        self.cap_caps: Dict[str, List[str]] = {}
        # foreign-net tracks on the cap's side in reach
        self.cap_segs: Dict[str, List[Tuple]] = {}
        # through-vias in reach (each (vx, vy, net, keepout))
        self.cap_vias: Dict[str, List[Tuple[float, float, int, float]]] = {}
        cap_refs = list(self.caps)
        for ref, cap in self.caps.items():
            ccx, ccy, span, crect = cap_geom[ref]
            reach = max_displacement_cap + span + clearance
            near_pads = []
            for fp_pad in self.foreign_pads:
                px = (fp_pad[0] + fp_pad[2]) / 2.0
                py = (fp_pad[1] + fp_pad[3]) / 2.0
                phalf = math.hypot(fp_pad[2] - px, fp_pad[3] - py)
                if math.hypot(px - ccx, py - ccy) <= reach + phalf:
                    near_pads.append(fp_pad)
            self.cap_foreign_pads[ref] = near_pads

            near_static = []
            for idx, (sr, side) in enumerate(self.static_rects):
                if side != cap.side:
                    continue
                if _rect_gap(crect, sr) <= max_displacement_cap + clearance + EPS:
                    near_static.append((idx, sr))
            self.cap_static[ref] = near_static

            near_caps = []
            for oref in cap_refs:
                if oref == ref or self.caps[oref].side != cap.side:
                    continue
                # both caps can move, so the combined reach is 2x the budget
                if (_rect_gap(crect, cap_geom[oref][3])
                        <= 2 * max_displacement_cap + clearance + EPS):
                    near_caps.append(oref)
            self.cap_caps[ref] = near_caps

            near_segs = []
            seg_reach = max_displacement_cap + 2 * span + clearance
            for seg in self.segments:
                if seg[6] != cap.side:
                    continue
                d = _point_to_seg_dist(ccx, ccy, seg[0], seg[1], seg[2], seg[3])
                if d <= seg_reach + seg[5]:
                    near_segs.append(seg)
            self.cap_segs[ref] = near_segs

            near_vias = []
            for v in self.vias:
                # v[3] = via_radius + clearance keep-out; a via that can reach
                # a pad must be within (move + span + keep-out) of the seed.
                if math.hypot(v[0] - ccx, v[1] - ccy) <= (
                        max_displacement_cap + span + v[3]):
                    near_vias.append(v)
            self.cap_vias[ref] = near_vias

        # Baseline same-side overlaps at the seed placement. Collisions are
        # scored RELATIVE to these: the repair must not introduce or worsen a
        # courtyard overlap, but it leaves pre-existing tight placements alone
        # (those aren't this issue's concern, and chasing them causes churn).
        self.base_static: Dict[Tuple[str, int], float] = {}
        for ref, cap in self.caps.items():
            seed_rect = cap.rect()
            for idx, (r, side) in enumerate(self.static_rects):
                if side == cap.side:
                    self.base_static[(ref, idx)] = self._overlap(seed_rect, r)
        self.base_cap: Dict[frozenset, float] = {}
        cap_items = list(self.caps.items())
        for i, (ra, ca) in enumerate(cap_items):
            ra_rect = ca.rect()
            for rb, cb in cap_items[i + 1:]:
                if cb.side == ca.side:
                    self.base_cap[frozenset((ra, rb))] = self._overlap(
                        ra_rect, cb.rect())
        self.base_seg: Dict[str, float] = {
            ref: self.seg_penalty(ref, cap, cap.x, cap.y, cap.rot)
            for ref, cap in self.caps.items()}
        # Baseline foreign-pad encroachment at the seed (#235): the repair may
        # not introduce or worsen a foreign-net pad-pad overlap, but tolerates
        # one already present at the seed (not this step's concern).
        self.base_pad: Dict[str, float] = {
            ref: self.pad_penalty(ref, cap, cap.x, cap.y, cap.rot)
            for ref, cap in self.caps.items()}

    @staticmethod
    def _near_any(rect, bboxes, margin):
        for b in bboxes:
            if _rect_gap(rect, b) <= margin:
                return True
        return False

    def _overlap(self, a, b):
        """Courtyard-clearance shortfall between two rects (0 if clear)."""
        return max(0.0, self.clearance - _rect_gap(a, b))

    def via_penalty(self, cap, x, y, rot, vias=None):
        """Sum of foreign-net via penetration depths for a cap placement
        (how far each pad intrudes inside a different-net via's keep-out).

        vias defaults to the full board list; callers with a ref pass the
        per-cap pruned list (self.cap_vias[ref]), which is exact -- a via that
        can penetrate a pad is necessarily within the cap's reach."""
        vias = self.vias if vias is None else vias
        pen = 0.0
        for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
            for vx, vy, vnet, keepout in vias:
                if vnet == net:
                    continue
                d = _point_to_rect_dist(vx, vy, (bx0, by0, bx1, by1))
                if d < keepout - EPS:
                    pen += (keepout - d)
        return pen

    def seg_penalty(self, ref, cap, x, y, rot):
        """Sum of foreign-net, same-side track penetration for a placement
        (pad modelled as its centre inflated by the pad half-diagonal). Uses
        the per-cap pruned, already-same-side track list."""
        pen = 0.0
        segs = self.cap_segs[ref]
        for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
            cx, cy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0
            half_diag = math.hypot((bx1 - bx0) / 2.0, (by1 - by0) / 2.0)
            for x1, y1, x2, y2, snet, halfw, side in segs:
                if snet == net:
                    continue
                req = halfw + half_diag
                d = _point_to_seg_dist(cx, cy, x1, y1, x2, y2)
                if d < req - EPS:
                    pen += (req - d)
        return pen

    def pad_penalty(self, ref, cap, x, y, rot):
        """Sum of foreign-net component-pad clearance shortfall for a cap
        placement (#235): how far each cap pad intrudes inside a different-net
        pad's clearance keep-out. A through-hole foreign pad blocks both sides;
        an SMD one only its own side. Same-net pads are ignored (a shared via
        / touching same-net copper is fine)."""
        pen = 0.0
        for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
            for (px0, py0, px1, py1, pnet, pside) in self.cap_foreign_pads[ref]:
                if pnet == net:
                    continue
                if pside is not None and pside != cap.side:
                    continue  # SMD pad on the other side
                gap = _rect_gap((bx0, by0, bx1, by1), (px0, py0, px1, py1))
                if gap < self.clearance - EPS:
                    pen += (self.clearance - gap)
        return pen

    def attraction(self, cap, x, y, rot):
        """Sum over pads of the distance to the nearest same-net BGA ball,
        clamped to capture_radius (so a pad with no same-net ball within reach
        contributes a flat constant and creates no long-range pull)."""
        total = 0.0
        for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
            balls = self.attract.get(net)
            if not balls:
                continue
            cx, cy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0
            nearest = min(math.hypot(ax - cx, ay - cy) for ax, ay in balls)
            total += min(nearest, self.capture_radius)
        return total

    def hard_blocked(self, ref, cap, x, y, rot):
        """True if a placement leaves the board or introduces/worsens a
        same-side courtyard overlap (with a locked part OR another movable
        cap) beyond its baseline. Caps may never overlap each other's
        footprints, so any new cap-cap overlap is rejected outright."""
        rect = cap.rect(x, y, rot)
        if (rect[0] < self.usable[0] or rect[1] < self.usable[1]
                or rect[2] > self.usable[2] or rect[3] > self.usable[3]):
            return True
        # only same-side parts/caps within reach can collide (pre-pruned)
        for idx, r in self.cap_static[ref]:
            base = self.base_static.get((ref, idx), 0.0)
            if self._overlap(rect, r) > base + EPS:
                return True
        for other_ref in self.cap_caps[ref]:
            base = self.base_cap.get(frozenset((ref, other_ref)), 0.0)
            if self._overlap(rect, self.caps[other_ref].rect()) > base + EPS:
                return True
        # no new overlap with a foreign-net track on the cap's side
        if self.seg_penalty(ref, cap, x, y, rot) > self.base_seg.get(ref, 0.0) + EPS:
            return True
        # no new overlap with a foreign-net component pad (#235): rejects the
        # cap-onto-neighbour-pad short the via/attraction objective could chase.
        if self.pad_penalty(ref, cap, x, y, rot) > self.base_pad.get(ref, 0.0) + EPS:
            return True
        return False

    def cost(self, ref, cap, x, y, rot):
        if self.hard_blocked(ref, cap, x, y, rot):
            return float('inf')
        disp = math.hypot(x - cap.seed_x, y - cap.seed_y)
        return (VIA_WEIGHT * self.via_penalty(cap, x, y, rot, self.cap_vias[ref])
                + ATTRACT_WEIGHT * self.attraction(cap, x, y, rot)
                + DISPLACEMENT_WEIGHT * disp)


def repair_fanout_clearance(pcb_data: PCBData, pcb_file: str,
                            clearance: float = 0.2,
                            grid_step: float = 0.1,
                            board_edge_clearance: float = 0.55,
                            near_margin: float = 1.0,
                            capture_radius: float = 2.0,
                            default_via_size: float = 0.3,
                            step: float = 0.2,
                            max_displacement: float = 2.0,
                            max_displacement_cap: float = 3.0,
                            displacement_growth: float = 1.5,
                            allow_rotations: bool = True,
                            cap_prefix: str = "C,R,FB",
                            lock_refs: Optional[List[str]] = None,
                            max_passes: int = 30,
                            via_clear_fallback: bool = True,
                            verbose: bool = False,
                            on_move=None) -> Dict:
    """Nudge near-BGA decoupling caps off foreign-net fanout vias and toward
    same-net balls. Run AFTER bga_fanout.py.

    Returns a dict with 'placements' (list of {reference,new_x,new_y,
    new_rotation} for moved caps), 'resolved', 'unresolved' (refs still
    overlapping a foreign via), and 'bga_refs'.

    via_clear_fallback (#213): when True (default), any cap the normal cost
    descent leaves grazing a foreign via is jumped to the nearest position that
    fully clears it (still respecting every hard clearance). It is deliberately
    NOT exposed on the CLI / GUI -- flip this argument in code to disable.

    on_move, if given, is a callback invoked with the internal _Repair state
    once at the seed placement and again after every accepted cap move. It is
    used purely for visualization (animate_fanout_clearance.py) and has no
    effect on the result.
    """
    extra_locked: Set[str] = set()
    if lock_refs:
        import fnmatch
        for ref in pcb_data.footprints:
            if any(fnmatch.fnmatch(ref, pat) for pat in lock_refs):
                extra_locked.add(ref)

    st = _Repair(pcb_data, pcb_file, clearance, grid_step,
                 board_edge_clearance, near_margin, capture_radius,
                 default_via_size, cap_prefix, extra_locked,
                 max_displacement_cap=max_displacement_cap)

    print(f"BGAs: {', '.join(st.bga_refs) or '(none)'}  "
          f"fanout vias: {len(st.vias)}  "
          f"movable near-BGA caps: {len(st.caps)}")
    if not st.vias:
        print("No vias on the board - run this AFTER bga_fanout.py.")
        return {'placements': [], 'resolved': [], 'unresolved': [],
                'bga_refs': st.bga_refs}
    if not st.caps:
        print("No movable caps near a BGA - nothing to do.")
        return {'placements': [], 'resolved': [], 'unresolved': [],
                'bga_refs': st.bga_refs}

    # Initial violators
    violators0 = [r for r, c in st.caps.items()
                  if st.via_penalty(c, c.x, c.y, c.rot, st.cap_vias[r]) > EPS]
    print(f"Caps initially overlapping a foreign via: {len(violators0)}"
          + (f" ({', '.join(sorted(violators0))})" if violators0 else ""))

    # LOCKED parts are excluded from moving by design, but a foreign via inside
    # a locked pad's keep-out is then unfixable here -- surface it instead of
    # silently reporting a clean pass (#254: locked back-side cap under a BGA
    # via-in-pad). The fanout avoids locked copper for NEW vias; this warning
    # catches boards fanned before that fix or vias from other sources.
    locked_hits = []
    for ref in sorted(st.locked_refs):
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        for p in fp.pads:
            if not any(str(l).endswith('.Cu') for l in p.layers):
                continue
            rect = (p.global_x - p.size_x / 2, p.global_y - p.size_y / 2,
                    p.global_x + p.size_x / 2, p.global_y + p.size_y / 2)
            for vx, vy, vnet, keepout in st.vias:
                if vnet == p.net_id:
                    continue
                if _point_to_rect_dist(vx, vy, rect) < keepout - EPS:
                    locked_hits.append((ref, p.pad_number, vx, vy))
    if locked_hits:
        print(f"WARNING: {len(locked_hits)} foreign via(s) inside LOCKED parts' pad "
              f"clearance (cannot move a locked part):")
        for ref, pnum, vx, vy in locked_hits[:10]:
            print(f"    {ref}.{pnum} <-> via at ({vx:.2f},{vy:.2f})")

    if on_move is not None:
        on_move(st)  # seed frame

    budget = {r: max_displacement for r in st.caps}
    rotate = {r: False for r in st.caps}
    # process worst violators first
    order = sorted(st.caps, key=lambda r: st.via_penalty(
        st.caps[r], st.caps[r].x, st.caps[r].y, st.caps[r].rot,
        st.cap_vias[r]), reverse=True)

    for pass_num in range(1, max_passes + 1):
        moves = 0
        for ref in order:
            cap = st.caps[ref]
            current = st.cost(ref, cap, cap.x, cap.y, cap.rot)
            rots = ROTATIONS if (allow_rotations and rotate[ref]) else [cap.rot]
            best = (current, cap.x, cap.y, cap.rot)
            for cx, cy in _candidate_positions(cap, budget[ref], step, grid_step):
                for rot in rots:
                    if (cx, cy, rot) == (cap.x, cap.y, cap.rot):
                        continue
                    c = st.cost(ref, cap, cx, cy, rot)
                    if c < best[0] - EPS:
                        best = (c, cx, cy, rot)
            if best[0] < current - EPS:
                cap.x, cap.y, cap.rot = best[1], best[2], best[3]
                moves += 1
                if on_move is not None:
                    on_move(st)
                if verbose:
                    print(f"  pass {pass_num}: {ref} -> "
                          f"({cap.x:.3f},{cap.y:.3f}) rot {cap.rot:g} "
                          f"cost {best[0]:.3f}")

        residual = {r: st.via_penalty(st.caps[r], st.caps[r].x,
                                      st.caps[r].y, st.caps[r].rot,
                                      st.cap_vias[r])
                    for r in st.caps}
        still = [r for r, p in residual.items() if p > EPS]
        if not still:
            print(f"All vias cleared after {pass_num} pass(es).")
            break
        if moves == 0:
            # stuck: escalate budget / rotations for the remaining violators
            grown = False
            for r in still:
                if not rotate[r] and allow_rotations:
                    rotate[r] = True
                    grown = True
                elif budget[r] < max_displacement_cap - EPS:
                    budget[r] = min(budget[r] * displacement_growth,
                                    max_displacement_cap)
                    grown = True
            if not grown:
                print(f"Stuck with {len(still)} cap(s) still overlapping a via "
                      f"(at the displacement cap): {', '.join(sorted(still))}")
                break
            if verbose:
                print(f"  escalating budget/rotation for {len(still)} cap(s)")

    # Fallback (#213): a cap may still graze a foreign via because the soft cost
    # (same-net attraction + displacement) judged the tiny penetration cheaper
    # than a clear-but-distant spot, so the greedy descent never relocates it.
    # A shipped PAD-VIA short is worse than a displaced decap, so for any cap
    # still in via-conflict, jump it to the best position (within the full
    # displacement budget) that FULLY clears every foreign via -- cost() still
    # rejects any hard-constraint violation (board edge, courtyard, foreign
    # track/pad #235), so this can never introduce a new short/overlap; it only
    # overrides the soft trade-off. If no clean via-clearing spot exists the cap
    # is left as-is and reported unresolved (needs a via re-drop, not a move).
    if via_clear_fallback:
        stuck = [r for r in st.caps
                 if st.via_penalty(st.caps[r], st.caps[r].x, st.caps[r].y,
                                   st.caps[r].rot, st.cap_vias[r]) > EPS]
        rots_all = ROTATIONS if allow_rotations else None
        for ref in stuck:
            cap = st.caps[ref]
            rots = rots_all if rots_all is not None else [cap.rot]
            best = None  # (cost, x, y, rot)
            for cx, cy in _candidate_positions(cap, max_displacement_cap,
                                               step, grid_step):
                for rot in rots:
                    if st.via_penalty(cap, cx, cy, rot, st.cap_vias[ref]) > EPS:
                        continue
                    c = st.cost(ref, cap, cx, cy, rot)  # inf if hard-blocked
                    if c == float('inf'):
                        continue
                    if best is None or c < best[0] - EPS:
                        best = (c, cx, cy, rot)
            if best is not None:
                cap.x, cap.y, cap.rot = best[1], best[2], best[3]
                disp = math.hypot(cap.x - cap.seed_x, cap.y - cap.seed_y)
                if verbose:
                    print(f"  fallback: {ref} relocated to clear via at "
                          f"disp {disp:.2f}mm -> ({cap.x:.3f},{cap.y:.3f}) "
                          f"rot {cap.rot:g}")

    placements = []
    resolved = []
    for ref, cap in st.caps.items():
        moved = (abs(cap.x - cap.seed_x) > EPS or abs(cap.y - cap.seed_y) > EPS
                 or abs(cap.rot - cap.seed_rot) > EPS)
        if moved:
            placements.append({'reference': ref, 'new_x': cap.x,
                               'new_y': cap.y, 'new_rotation': cap.rot})
        if (ref in violators0
                and st.via_penalty(cap, cap.x, cap.y, cap.rot,
                                   st.cap_vias[ref]) <= EPS):
            resolved.append(ref)
    unresolved = [r for r in st.caps
                  if st.via_penalty(st.caps[r], st.caps[r].x,
                                    st.caps[r].y, st.caps[r].rot,
                                    st.cap_vias[r]) > EPS]

    print(f"Moved {len(placements)} cap(s); resolved {len(resolved)}/"
          f"{len(violators0)} initial violations; "
          f"{len(unresolved)} unresolved.")
    if unresolved:
        print(f"  Unresolved (need manual attention): {', '.join(sorted(unresolved))}")

    return {'placements': placements, 'resolved': resolved,
            'unresolved': unresolved, 'bga_refs': st.bga_refs}
