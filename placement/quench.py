"""
Greedy quench placement optimizer: perturbative refinement of an existing
(hand- or AI-made) placement to improve routability.

Starts from the current placement and repeatedly tries, for each component,
small moves within --max-displacement of its seed position plus 90-degree
rotations and same-footprint swaps, accepting only improvements
(zero-temperature anneal). Locked components never move.

Cost = total airwire length
     + crossing_penalty * airwire crossings
     + halo penalty (soft whitespace around parts, scaled by pin count)
     + edge penalty (soft margin inside the board edge)

The halo term spreads apart parts that are not pulled together by shared
nets — things that *can* be far apart may as well be, to leave routing room,
especially around high-pin-count parts.
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple, Set, Optional

import numpy as np

from kicad_parser import PCBData, local_to_global
from connectivity import compute_mst_edges
from placement.parser import extract_courtyard_bboxes, extract_locked_refs
from placement.utility import compute_footprint_bbox_local, snap_to_grid

ROTATIONS = [0.0, 90.0, 180.0, 270.0]
EPS_IMPROVE = 1e-6


def _rotate_local_bounds(lmin_x, lmin_y, lmax_x, lmax_y, rotation):
    """Rotate a local bounding box by the footprint rotation (KiCad sign)."""
    rot = rotation % 360
    if abs(rot) < 0.01:
        return lmin_x, lmin_y, lmax_x, lmax_y
    angle = math.radians(-rot)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    corners = [(lmin_x, lmin_y), (lmax_x, lmin_y), (lmin_x, lmax_y), (lmax_x, lmax_y)]
    xs = [x * cos_a - y * sin_a for x, y in corners]
    ys = [x * sin_a + y * cos_a for x, y in corners]
    return min(xs), min(ys), max(xs), max(ys)


def _rect_gap(a, b):
    """Smallest axis-aligned gap between two rects (negative if overlapping)."""
    dx = max(a[0] - b[2], b[0] - a[2])
    dy = max(a[1] - b[3], b[1] - a[3])
    if dx < 0 and dy < 0:
        return max(dx, dy)  # overlap amount (negative)
    return math.hypot(max(dx, 0), max(dy, 0)) if (dx > 0 and dy > 0) else max(dx, dy)


def _airwires_for_points(points: List[Tuple[float, float]], net_id: int):
    """MST airwires for one net: list of (x1, y1, x2, y2, net_id)."""
    if len(points) < 2:
        return []
    if len(points) == 2:
        (x1, y1), (x2, y2) = points
        return [(x1, y1, x2, y2, net_id)]
    edges = compute_mst_edges(points, use_manhattan=False)
    return [(points[i][0], points[i][1], points[j][0], points[j][1], net_id)
            for i, j, _ in edges]


def _aw_array(airwires) -> np.ndarray:
    if not airwires:
        return np.zeros((0, 5))
    return np.asarray(airwires, dtype=float)


def _count_crossings_np(a: np.ndarray, b: np.ndarray) -> int:
    """Count crossings between airwire sets a (n,5) and b (m,5), skipping
    same-net pairs and pairs sharing an endpoint (within 1um)."""
    if len(a) == 0 or len(b) == 0:
        return 0
    eps = 0.001
    a1x = a[:, 0][:, None]; a1y = a[:, 1][:, None]
    a2x = a[:, 2][:, None]; a2y = a[:, 3][:, None]
    b1x = b[:, 0][None, :]; b1y = b[:, 1][None, :]
    b2x = b[:, 2][None, :]; b2y = b[:, 3][None, :]

    same_net = a[:, 4][:, None] == b[:, 4][None, :]
    shared = (
        ((np.abs(a1x - b1x) < eps) & (np.abs(a1y - b1y) < eps)) |
        ((np.abs(a1x - b2x) < eps) & (np.abs(a1y - b2y) < eps)) |
        ((np.abs(a2x - b1x) < eps) & (np.abs(a2y - b1y) < eps)) |
        ((np.abs(a2x - b2x) < eps) & (np.abs(a2y - b2y) < eps))
    )

    def ccw(ax, ay, bx, by, cx, cy):
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    inter = (
        (ccw(a1x, a1y, b1x, b1y, b2x, b2y) != ccw(a2x, a2y, b1x, b1y, b2x, b2y)) &
        (ccw(a1x, a1y, a2x, a2y, b1x, b1y) != ccw(a1x, a1y, a2x, a2y, b2x, b2y))
    )
    return int(np.count_nonzero(inter & ~same_net & ~shared))


def _count_crossings_within(a: np.ndarray) -> int:
    """Crossings among one airwire set (each unordered pair counted once)."""
    if len(a) < 2:
        return 0
    total = _count_crossings_np(a, a)
    return total // 2


class _Part:
    __slots__ = ('ref', 'pads_local', 'pin_count', 'bounds_by_rot',
                 'seed_x', 'seed_y', 'x', 'y', 'rot', 'locked',
                 'nets', 'halo', 'footprint_name', 'orig_rot')

    def __init__(self, ref, fp, courtyard_bboxes, locked, halo_base, halo_coef):
        self.ref = ref
        self.footprint_name = fp.footprint_name
        self.pads_local = [(p.local_x, p.local_y, p.net_id)
                           for p in fp.pads if p.net_id > 0]
        self.pin_count = len(self.pads_local)
        if ref in courtyard_bboxes:
            lb = courtyard_bboxes[ref]
        else:
            lb = compute_footprint_bbox_local(fp)
        self.bounds_by_rot = {r: _rotate_local_bounds(*lb, r) for r in ROTATIONS}
        # Non-90-degree seed rotations keep their own bounds entry
        if fp.rotation % 90 != 0:
            self.bounds_by_rot[fp.rotation % 360] = _rotate_local_bounds(
                *lb, fp.rotation)
        self.seed_x, self.seed_y = fp.x, fp.y
        self.x, self.y, self.rot = fp.x, fp.y, fp.rotation % 360
        self.orig_rot = fp.rotation % 360
        self.locked = locked
        self.nets = sorted({n for _, _, n in self.pads_local})
        self.halo = halo_base + halo_coef * math.sqrt(max(self.pin_count, 1))

    def rect(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        b = self.bounds_by_rot.get(rot % 360)
        if b is None:
            b = self.bounds_by_rot[0.0]
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def pad_globals(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        return [(*local_to_global(x, y, rot, lx, ly), n)
                for lx, ly, n in self.pads_local]


class QuenchState:
    """Current placement plus cached airwires and cost terms."""

    def __init__(self, pcb_data: PCBData, pcb_file: str,
                 clearance: float, board_edge_clearance: float,
                 crossing_penalty: float,
                 halo_base: float, halo_coef: float, halo_weight: float,
                 edge_halo: float, edge_weight: float,
                 grid_step: float, length_weight: float = 1.0,
                 ignore_net_ids: Optional[Set[int]] = None,
                 extra_locked_refs: Optional[Set[str]] = None,
                 move_refs: Optional[Set[str]] = None,
                 net_weights: Optional[Dict[int, float]] = None):
        bounds = pcb_data.board_info.board_bounds
        if bounds is None:
            raise ValueError("No board boundary (Edge.Cuts) found")
        self.board = bounds
        margin = max(clearance, board_edge_clearance)
        self.usable = (bounds[0] + margin, bounds[1] + margin,
                       bounds[2] - margin, bounds[3] - margin)
        self.clearance = clearance
        self.crossing_penalty = crossing_penalty
        self.length_weight = length_weight
        self.net_weights = net_weights or {}
        self.halo_weight = halo_weight
        self.edge_halo = edge_halo
        self.edge_weight = edge_weight
        self.grid_step = grid_step

        courtyards = extract_courtyard_bboxes(pcb_file)
        locked_refs = set(extract_locked_refs(pcb_file))
        if extra_locked_refs:
            locked_refs |= extra_locked_refs
        ignore = ignore_net_ids or set()

        self.parts: Dict[str, _Part] = {}
        for ref, fp in pcb_data.footprints.items():
            if not fp.pads:
                continue
            locked = (ref in locked_refs
                      or (move_refs is not None and ref not in move_refs))
            self.parts[ref] = _Part(ref, fp, courtyards, locked,
                                    halo_base, halo_coef)
            # Ignored nets (e.g. plane-routed power) don't contribute airwires
            self.parts[ref].nets = [n for n in self.parts[ref].nets
                                    if n not in ignore]

        # net -> refs touching it
        self.net_refs: Dict[int, Set[str]] = {}
        for ref, part in self.parts.items():
            for n in part.nets:
                self.net_refs.setdefault(n, set()).add(ref)

        # Per-net airwires cache
        self.net_airwires: Dict[int, List] = {}
        for net_id in self.net_refs:
            self.net_airwires[net_id] = self._build_net_airwires(net_id)

    # ----- airwire helpers -------------------------------------------------

    def _net_points(self, net_id, override_ref=None, override_pads=None):
        pts = []
        for ref in self.net_refs[net_id]:
            if ref == override_ref:
                pts.extend((gx, gy) for gx, gy, n in override_pads if n == net_id)
            else:
                pts.extend((gx, gy) for gx, gy, n in self.parts[ref].pad_globals()
                           if n == net_id)
        return pts

    def _build_net_airwires(self, net_id, override_ref=None, override_pads=None):
        return _airwires_for_points(
            self._net_points(net_id, override_ref, override_pads), net_id)

    def airwires_excluding(self, nets: Set[int]) -> np.ndarray:
        aws = []
        for net_id, lst in self.net_airwires.items():
            if net_id not in nets:
                aws.extend(lst)
        return _aw_array(aws)

    # ----- cost terms ------------------------------------------------------

    def _halo_pair_penalty(self, part_a: _Part, rect_a, part_b: _Part, rect_b):
        required = part_a.halo + part_b.halo
        gap = _rect_gap(rect_a, rect_b)
        if gap >= required:
            return 0.0
        short = required - max(gap, 0.0)
        return self.halo_weight * short * short

    def _edge_penalty(self, rect):
        pen = 0.0
        gaps = (rect[0] - self.board[0], rect[1] - self.board[1],
                self.board[2] - rect[2], self.board[3] - rect[3])
        for g in gaps:
            if g < self.edge_halo:
                short = self.edge_halo - max(g, 0.0)
                pen += self.edge_weight * short * short
        return pen

    def part_geometry_cost(self, ref, x=None, y=None, rot=None,
                           exclude: Optional[Set[str]] = None):
        """Halo + edge penalty contributions of one part at a position."""
        part = self.parts[ref]
        rect = part.rect(x, y, rot)
        pen = self._edge_penalty(rect)
        for other_ref, other in self.parts.items():
            if other_ref == ref or (exclude and other_ref in exclude):
                continue
            pen += self._halo_pair_penalty(part, rect, other, other.rect())
        return pen

    def candidate_valid(self, ref, x, y, rot, exclude: Optional[Set[str]] = None):
        part = self.parts[ref]
        rect = part.rect(x, y, rot)
        if (rect[0] < self.usable[0] or rect[1] < self.usable[1]
                or rect[2] > self.usable[2] or rect[3] > self.usable[3]):
            return False
        for other_ref, other in self.parts.items():
            if other_ref == ref or (exclude and other_ref in exclude):
                continue
            r = other.rect()
            if not (rect[2] + self.clearance <= r[0] or r[2] + self.clearance <= rect[0]
                    or rect[3] + self.clearance <= r[1] or r[3] + self.clearance <= rect[1]):
                return False
        return True

    def _weighted_length(self, arr: np.ndarray) -> float:
        if len(arr) == 0:
            return 0.0
        lengths = np.hypot(arr[:, 2] - arr[:, 0], arr[:, 3] - arr[:, 1])
        if self.net_weights:
            w = np.array([self.net_weights.get(int(n), 1.0)
                          for n in arr[:, 4]])
            lengths = lengths * w
        return float(np.sum(lengths))

    def nets_cost(self, net_airwires_subset: Dict[int, List],
                  other_airwires: np.ndarray):
        """Length + crossing cost of the given nets' airwires, counting
        crossings against `other_airwires` and among themselves."""
        own = []
        for lst in net_airwires_subset.values():
            own.extend(lst)
        own_arr = _aw_array(own)
        length = self._weighted_length(own_arr)
        crossings = _count_crossings_np(own_arr, other_airwires)
        crossings += _count_crossings_within(own_arr)
        return self.length_weight * length + self.crossing_penalty * crossings, crossings

    # ----- full cost (for reporting) ---------------------------------------

    def total_cost(self):
        all_aw = _aw_array([aw for lst in self.net_airwires.values() for aw in lst])
        length = self._weighted_length(all_aw)
        crossings = _count_crossings_within(all_aw)
        halo = 0.0
        edge = 0.0
        refs = list(self.parts)
        for i, ra in enumerate(refs):
            pa = self.parts[ra]
            rect_a = pa.rect()
            edge += self._edge_penalty(rect_a)
            for rb in refs[i + 1:]:
                pb = self.parts[rb]
                halo += self._halo_pair_penalty(pa, rect_a, pb, pb.rect())
        total = (self.length_weight * length
                 + self.crossing_penalty * crossings + halo + edge)
        return {'total': total, 'length': length, 'crossings': crossings,
                'halo': halo, 'edge': edge}

    # ----- move application -------------------------------------------------

    def apply_move(self, ref, x, y, rot):
        part = self.parts[ref]
        part.x, part.y, part.rot = x, y, rot
        for net_id in part.nets:
            self.net_airwires[net_id] = self._build_net_airwires(net_id)


def _candidate_positions(part: _Part, max_disp: float, step: float,
                         grid_step: float):
    """Grid of candidate centers within max_disp of the seed position."""
    seen = set()
    out = []
    n = int(max_disp / step)
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            cx = part.seed_x + ix * step
            cy = part.seed_y + iy * step
            if math.hypot(cx - part.seed_x, cy - part.seed_y) > max_disp + 1e-9:
                continue
            cx = snap_to_grid(cx, grid_step)
            cy = snap_to_grid(cy, grid_step)
            key = (round(cx, 4), round(cy, 4))
            if key not in seen:
                seen.add(key)
                out.append((cx, cy))
    return out


def quench(pcb_data: PCBData, pcb_file: str,
           max_displacement: float = 10.0,
           step: float = 1.0,
           grid_step: float = 0.1,
           clearance: float = 0.25,
           board_edge_clearance: float = 0.55,
           crossing_penalty: float = 10.0,
           length_weight: float = 1.0,
           halo_base: float = 0.5,
           halo_coef: float = 0.25,
           halo_weight: float = 2.0,
           edge_halo: float = 2.0,
           edge_weight: float = 2.0,
           allow_rotations: bool = True,
           allow_swaps: bool = True,
           max_passes: int = 10,
           ignore_nets: Optional[List[str]] = None,
           lock_refs: Optional[List[str]] = None,
           move_refs: Optional[Set[str]] = None,
           net_weights: Optional[Dict[int, float]] = None,
           verbose: bool = False) -> List[Dict]:
    """Greedy quench: iterate over parts, accept only cost-reducing moves.

    Returns a list of placement dicts (reference/new_x/new_y/new_rotation)
    for every movable part, whether or not it moved.
    """
    ignore_net_ids: Set[int] = set()
    if ignore_nets:
        import fnmatch
        for net_id, net in pcb_data.nets.items():
            if any(fnmatch.fnmatch(net.name, pat) for pat in ignore_nets):
                ignore_net_ids.add(net_id)
        print(f"Ignoring {len(ignore_net_ids)} nets for airwire scoring")

    extra_locked: Set[str] = set()
    if lock_refs:
        import fnmatch
        for ref in pcb_data.footprints:
            if any(fnmatch.fnmatch(ref, pat) for pat in lock_refs):
                extra_locked.add(ref)
        print(f"Locked via --lock: {', '.join(sorted(extra_locked))}")

    state = QuenchState(pcb_data, pcb_file, clearance, board_edge_clearance,
                        crossing_penalty, halo_base, halo_coef, halo_weight,
                        edge_halo, edge_weight, grid_step, length_weight,
                        ignore_net_ids=ignore_net_ids,
                        extra_locked_refs=extra_locked,
                        move_refs=move_refs,
                        net_weights=net_weights)

    before = state.total_cost()
    print(f"Initial: length={before['length']:.1f}mm "
          f"crossings={before['crossings']} halo={before['halo']:.1f} "
          f"edge={before['edge']:.1f} total={before['total']:.1f}")

    movable = [r for r, p in state.parts.items() if not p.locked]
    movable.sort(key=lambda r: state.parts[r].pin_count, reverse=True)

    for pass_num in range(1, max_passes + 1):
        improved = 0.0
        moves = 0

        # --- single-part moves (nudge + rotate) ---
        for ref in movable:
            part = state.parts[ref]
            involved = set(part.nets)
            other_aw = state.airwires_excluding(involved)

            def eval_at(x, y, rot):
                subset = {n: state._build_net_airwires(
                    n, override_ref=ref,
                    override_pads=part.pad_globals(x, y, rot))
                    for n in involved}
                net_cost, _ = state.nets_cost(subset, other_aw)
                geo_cost = state.part_geometry_cost(ref, x, y, rot)
                return net_cost + geo_cost

            current_cost = eval_at(part.x, part.y, part.rot)
            rotations = (ROTATIONS if allow_rotations and part.orig_rot % 90 == 0
                         else [part.rot])

            best = (current_cost, part.x, part.y, part.rot)
            for cx, cy in _candidate_positions(part, max_displacement, step,
                                               grid_step):
                for rot in rotations:
                    if (cx, cy, rot) == (part.x, part.y, part.rot):
                        continue
                    if not state.candidate_valid(ref, cx, cy, rot):
                        continue
                    c = eval_at(cx, cy, rot)
                    if c < best[0] - EPS_IMPROVE:
                        best = (c, cx, cy, rot)

            if best[0] < current_cost - EPS_IMPROVE:
                gain = current_cost - best[0]
                improved += gain
                moves += 1
                state.apply_move(ref, best[1], best[2], best[3])
                if verbose:
                    dx = best[1] - part.seed_x
                    dy = best[2] - part.seed_y
                    print(f"  {ref:>6s}: moved to ({best[1]:.2f}, {best[2]:.2f})"
                          f" rot={best[3]:.0f} (d=({dx:+.1f},{dy:+.1f}))"
                          f" gain={gain:.1f}")

        # --- same-footprint swap moves ---
        if allow_swaps:
            by_fp: Dict[str, List[str]] = {}
            for ref in movable:
                by_fp.setdefault(state.parts[ref].footprint_name, []).append(ref)
            for fp_name, refs in by_fp.items():
                if len(refs) < 2:
                    continue
                for i in range(len(refs)):
                    for j in range(i + 1, len(refs)):
                        ra, rb = refs[i], refs[j]
                        pa, pb = state.parts[ra], state.parts[rb]
                        involved = set(pa.nets) | set(pb.nets)
                        other_aw = state.airwires_excluding(involved)

                        def eval_pair(ax, ay, arot, bx, by, brot):
                            pads_a = pa.pad_globals(ax, ay, arot)
                            pads_b = pb.pad_globals(bx, by, brot)
                            subset = {}
                            for n in involved:
                                pts = []
                                for ref2 in state.net_refs[n]:
                                    if ref2 == ra:
                                        pts.extend((gx, gy) for gx, gy, nn in pads_a
                                                   if nn == n)
                                    elif ref2 == rb:
                                        pts.extend((gx, gy) for gx, gy, nn in pads_b
                                                   if nn == n)
                                    else:
                                        pts.extend(
                                            (gx, gy) for gx, gy, nn in
                                            state.parts[ref2].pad_globals()
                                            if nn == n)
                                subset[n] = _airwires_for_points(pts, n)
                            net_cost, _ = state.nets_cost(subset, other_aw)
                            geo = (state.part_geometry_cost(ra, ax, ay, arot,
                                                            exclude={rb})
                                   + state.part_geometry_cost(rb, bx, by, brot,
                                                              exclude={ra})
                                   + state._halo_pair_penalty(
                                       pa, pa.rect(ax, ay, arot),
                                       pb, pb.rect(bx, by, brot)))
                            return net_cost + geo

                        cur = eval_pair(pa.x, pa.y, pa.rot, pb.x, pb.y, pb.rot)
                        swapped = eval_pair(pb.x, pb.y, pb.rot,
                                            pa.x, pa.y, pa.rot)
                        if swapped < cur - EPS_IMPROVE:
                            gain = cur - swapped
                            improved += gain
                            moves += 1
                            ax, ay, arot = pa.x, pa.y, pa.rot
                            state.apply_move(ra, pb.x, pb.y, pb.rot)
                            state.apply_move(rb, ax, ay, arot)
                            if verbose:
                                print(f"  swap {ra} <-> {rb} gain={gain:.1f}")

        stats = state.total_cost()
        print(f"Pass {pass_num}: {moves} moves, gain {improved:.1f} -> "
              f"length={stats['length']:.1f}mm crossings={stats['crossings']} "
              f"halo={stats['halo']:.1f} edge={stats['edge']:.1f} "
              f"total={stats['total']:.1f}")
        if moves == 0:
            break

    after = state.total_cost()
    print(f"Quench complete: length {before['length']:.1f} -> {after['length']:.1f}mm, "
          f"crossings {before['crossings']} -> {after['crossings']}, "
          f"total {before['total']:.1f} -> {after['total']:.1f}")

    return [{'reference': ref,
             'new_x': p.x, 'new_y': p.y, 'new_rotation': p.rot}
            for ref, p in state.parts.items()
            if not p.locked and (p.x != p.seed_x or p.y != p.seed_y
                                 or p.rot != p.orig_rot % 360)]
