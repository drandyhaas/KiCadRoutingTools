#!/usr/bin/env python3
"""Congested-pad micro-escape (#73).

route.py's rescue gives up on pads whose only exit is a multi-segment escape
at board-minimum width/clearance -- fine-pitch QFN pins and connector pads
walled in by neighbor pads, tracks, and via barrels. A legal escape usually
EXISTS: a short BFS path on the pad's layer to the first spot where a
through-via is legal on every copper layer (or to existing same-net copper).
This tool finds it and injects the stub + via; route.py then finishes the
net normally (the via is a terminal on every routing layer, #72).

Usage:
    python3 escape_stub.py board.kicad_pcb --pad U33.13 --output out.kicad_pcb
    python3 escape_stub.py board.kicad_pcb --pad J50.30 --replace-radius 6 \
        --trace-width 0.1 --via-size 0.25 --via-drill 0.15

--replace-radius R strips the pad's net's existing segments/vias within R mm
of the pad first (redoing a bad hand stub); the strip never touches pads.

Obstacle model: same-layer foreign segments and pads (exact pad geometry via
check_drc.point_to_pad_distance -- rotation/shape aware), all foreign via
barrels at their PER-INSTANCE size, all drill holes (hole-to-hole floor), and
per-net clearances from the board's netclasses (cross-class max(A,B)).
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from collections import deque
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "rust_router"))

from kicad_parser import parse_kicad_pcb
from check_drc import point_to_pad_distance


COPPER_SIGNAL_LAYERS = ("F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu",
                        "In5.Cu", "In6.Cu", "B.Cu")


def _seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    if L <= 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


class EscapeModel:
    """Copper/hole obstacle queries at per-net clearances."""

    def __init__(self, pcb, net_id: int, clearance_map: Dict[int, float],
                 base_clearance: float, hole_to_hole: float,
                 copper_layers: List[str]):
        self.pcb = pcb
        self.net_id = net_id
        self.clr = clearance_map or {}
        self.base = base_clearance
        self.h2h = hole_to_hole
        self.copper_layers = copper_layers
        self.own_clr = self.clr.get(net_id, base_clearance)
        self.segs_by_layer: Dict[str, List] = {}
        for s in pcb.segments:
            if s.net_id == net_id:
                continue
            self.segs_by_layer.setdefault(s.layer, []).append(s)
        self.foreign_vias = [v for v in pcb.vias if v.net_id != net_id]
        self.all_holes = ([(v.x, v.y, v.drill) for v in pcb.vias]
                          + [(p.global_x, p.global_y, max(p.drill or 0.0,
                                                          p.drill_w or 0.0))
                             for pads in pcb.pads_by_net.values() for p in pads
                             if (p.drill or 0.0) > 0 or (p.drill_w or 0.0) > 0])
        self.foreign_pads_by_layer: Dict[str, List] = {}
        for pads in pcb.pads_by_net.values():
            for p in pads:
                if p.net_id == net_id:
                    continue
                for layer in self._pad_layers(p):
                    self.foreign_pads_by_layer.setdefault(layer, []).append(p)

    def _pad_layers(self, pad) -> List[str]:
        out = []
        for l in pad.layers or []:
            if l in ("*.Cu",):
                out.extend(self.copper_layers)
            elif l.endswith(".Cu"):
                out.append(l)
        return out or ([] if pad.pad_type == "smd" else list(self.copper_layers))

    def _pair_clr(self, other_net: int) -> float:
        return max(self.own_clr, self.clr.get(other_net, self.base))

    def copper_free(self, x: float, y: float, layer: str, half_width: float,
                    extra: float = 0.0) -> bool:
        """True if a round object of `half_width` centred at (x,y) on `layer`
        clears all foreign copper there. `extra` widens every clearance."""
        for s in self.segs_by_layer.get(layer, ()):
            need = half_width + self._pair_clr(s.net_id) + s.width / 2 + extra
            if _seg_dist(x, y, s.start_x, s.start_y, s.end_x, s.end_y) < need:
                return False
        for v in self.foreign_vias:
            need = half_width + self._pair_clr(v.net_id) + v.size / 2 + extra
            if math.hypot(x - v.x, y - v.y) < need:
                return False
        for p in self.foreign_pads_by_layer.get(layer, ()):
            need = half_width + self._pair_clr(p.net_id) + extra
            if point_to_pad_distance(x, y, p) < need:
                return False
        return True

    def via_ok(self, x: float, y: float, via_size: float,
               via_drill: float) -> bool:
        for layer in self.copper_layers:
            if not self.copper_free(x, y, layer, via_size / 2):
                return False
        for hx, hy, hd in self.all_holes:
            if hd <= 0:
                continue
            d = math.hypot(x - hx, y - hy)
            if 1e-9 < d < via_drill / 2 + self.h2h + hd / 2:
                return False
        return True

    def touches_own_net(self, x: float, y: float, layer: str,
                        half_width: float) -> bool:
        for s in self.pcb.segments:
            if s.net_id != self.net_id or s.layer != layer:
                continue
            if _seg_dist(x, y, s.start_x, s.start_y,
                         s.end_x, s.end_y) < s.width / 2 + half_width:
                return True
        for v in self.pcb.vias:
            if v.net_id != self.net_id:
                continue
            if math.hypot(x - v.x, y - v.y) < v.size / 2 + half_width:
                return True
        return False


def find_escape(model: EscapeModel, pad, trace_width: float, via_size: float,
                via_drill: float, grid: float = 0.05,
                max_radius: float = 4.5,
                accept_net_touch: bool = True
                ) -> Optional[Tuple[List[Tuple[float, float]], bool]]:
    """BFS from the pad centre on the pad's outer layer. Returns
    (waypoints, needs_via): a rectilinear path whose last point is either a
    legal via spot (needs_via=True) or touching same-net copper."""
    layer = None
    for l in pad.layers or []:
        if l.endswith(".Cu") and not l.startswith("*"):
            layer = l
            break
    if layer is None:
        layer = "F.Cu"
    px, py = pad.global_x, pad.global_y
    hw = trace_width / 2
    N = int(max_radius / grid)
    # via-in-pad first: on ball-grid pads (0.3mm balls at 0.5 pitch) the
    # inter-pad gap cannot pass a trace at all -- the ONLY escape is a via
    # in the pad itself (JLC FPOV).
    if model.via_ok(px, py, via_size, via_drill):
        return [(px, py)], True
    # cells inside the pad's own copper are free launch space
    pad_reach = max(pad.size_x, pad.size_y) / 2

    seen = {(0, 0): None}
    q = deque([(0, 0)])
    goal = None
    needs_via = True
    while q:
        c = q.popleft()
        x, y = px + c[0] * grid, py + c[1] * grid
        far = math.hypot(x - px, y - py) > pad_reach
        if far:
            if model.via_ok(x, y, via_size, via_drill):
                goal = c
                break
            if accept_net_touch and model.touches_own_net(x, y, layer, hw):
                goal = c
                needs_via = False
                break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc = (c[0] + dx, c[1] + dy)
            if nc in seen or abs(nc[0]) > N or abs(nc[1]) > N:
                continue
            nx, ny = px + nc[0] * grid, py + nc[1] * grid
            inside_pad = point_to_pad_distance(nx, ny, pad) <= 0.01
            if not inside_pad and not model.copper_free(nx, ny, layer, hw):
                continue
            seen[nc] = c
            q.append(nc)
    if goal is None:
        return None
    path = []
    c = goal
    while c is not None:
        path.append((px + c[0] * grid, py + c[1] * grid))
        c = seen[c]
    path.reverse()
    merged = [path[0]]
    for i in range(1, len(path) - 1):
        a, p, n2 = merged[-1], path[i], path[i + 1]
        if ((abs(a[0] - p[0]) < 1e-9 and abs(p[0] - n2[0]) < 1e-9)
                or (abs(a[1] - p[1]) < 1e-9 and abs(p[1] - n2[1]) < 1e-9)):
            continue
        merged.append(p)
    merged.append(path[-1])
    return merged, needs_via


def strip_net_near(content: str, pcb, net_id: int, cx: float, cy: float,
                   radius: float) -> Tuple[str, int]:
    """Remove the net's segments/vias with any point within radius of (cx,cy)
    from the board text. Returns (new_content, removed_count)."""
    import re as _re
    removed = 0
    out = []
    i = 0
    s = content
    while i < len(s):
        j1 = s.find("(segment", i)
        j2 = s.find("(via", i)
        j = min(x for x in (j1, j2) if x >= 0) if (j1 >= 0 or j2 >= 0) else -1
        if j < 0:
            out.append(s[i:])
            break
        is_via = (j == j2 and (j1 < 0 or j2 < j1))
        tag = 4 if is_via else 8
        if s[j + tag] not in " \n\t(":
            out.append(s[i:j + tag])
            i = j + tag
            continue
        depth = 0
        k = j
        while k < len(s):
            if s[k] == "(":
                depth += 1
            elif s[k] == ")":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = s[j:k + 1]
        out.append(s[i:j])
        kill = False
        mn = _re.search(r"\(net (\d+)\)", block)
        if mn and int(mn.group(1)) == net_id:
            pts = _re.findall(r"\((?:start|end|at) ([0-9.-]+) ([0-9.-]+)\)",
                              block)
            if any(math.hypot(float(x) - cx, float(y) - cy) <= radius
                   for x, y in pts):
                kill = True
                removed += 1
        if not kill:
            out.append(block)
        i = k + 1
    return "".join(out), removed


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("board")
    ap.add_argument("--pad", required=True,
                    help="REF.PADNUM, e.g. U33.13")
    ap.add_argument("--output", required=True)
    ap.add_argument("--trace-width", type=float, default=0.1)
    ap.add_argument("--via-size", type=float, default=0.25)
    ap.add_argument("--via-drill", type=float, default=0.15)
    ap.add_argument("--clearance", type=float, default=0.1,
                    help="base clearance for nets without a class entry")
    ap.add_argument("--hole-to-hole", type=float, default=0.2)
    ap.add_argument("--grid-step", type=float, default=0.05)
    ap.add_argument("--max-radius", type=float, default=4.5)
    ap.add_argument("--replace-radius", type=float, default=0.0,
                    help="first strip the net's copper within this radius "
                         "of the pad (redo a bad stub)")
    args = ap.parse_args()

    pcb = parse_kicad_pcb(args.board)
    ref, _, num = args.pad.rpartition(".")
    pad = None
    for pads in pcb.pads_by_net.values():
        for p in pads:
            if p.component_ref == ref and str(p.pad_number) == num:
                pad = p
                break
    if pad is None:
        print(f"pad {args.pad} not found")
        sys.exit(1)
    net_id = pad.net_id
    net_name = pcb.nets[net_id].name if net_id in pcb.nets else str(net_id)
    print(f"{args.pad}: net '{net_name}' at ({pad.global_x:.2f}, "
          f"{pad.global_y:.2f})")

    content = open(args.board, encoding="utf-8").read()
    if args.replace_radius > 0:
        content, n = strip_net_near(content, pcb, net_id, pad.global_x,
                                    pad.global_y, args.replace_radius)
        print(f"replace: stripped {n} same-net item(s) within "
              f"{args.replace_radius}mm")
        # re-parse the stripped board so the model reflects the removal
        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".kicad_pcb",
                                         delete=False) as f:
            f.write(content)
            tmp = f.name
        pcb = parse_kicad_pcb(tmp)
        os.unlink(tmp)

    layer_names = [l for l in COPPER_SIGNAL_LAYERS
                   if f'"{l}"' in content]
    clearance_map: Dict[int, float] = {}
    try:
        from list_nets import net_clearance_map_by_id
        clearance_map = net_clearance_map_by_id(
            args.board, {nid: net.name for nid, net in pcb.nets.items()}) or {}
    except Exception:
        pass

    model = EscapeModel(pcb, net_id, clearance_map, args.clearance,
                        args.hole_to_hole, layer_names)
    res = find_escape(model, pad, args.trace_width, args.via_size,
                      args.via_drill, grid=args.grid_step,
                      max_radius=args.max_radius)
    if res is None:
        print("no escape found")
        sys.exit(2)
    path, needs_via = res
    print(f"escape: {len(path)} waypoint(s), "
          f"{'via at (%.2f, %.2f)' % path[-1] if needs_via else 'net touch'}")

    stub_layer = next((l for l in (pad.layers or [])
                       if l.endswith(".Cu") and not l.startswith("*")), "F.Cu")
    add = ""
    for a, b in zip(path, path[1:]):
        add += ('\n\t(segment (start %.4f %.4f) (end %.4f %.4f) '
                '(width %.3f) (layer "%s") (net %d))'
                % (a[0], a[1], b[0], b[1], args.trace_width, stub_layer,
                   net_id))
    if needs_via:
        add += ('\n\t(via (at %.4f %.4f) (size %.3g) (drill %.3g) '
                '(layers "F.Cu" "B.Cu") (net %d))'
                % (path[-1][0], path[-1][1], args.via_size, args.via_drill,
                   net_id))
    i = content.rstrip().rfind(")")
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(content[:i] + add + "\n" + content[i:])
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
