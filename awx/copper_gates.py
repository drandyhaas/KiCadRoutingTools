"""copper_gates.py -- what a track or a via a bus stage lays ITSELF (a berth's stub at a row part or a pin header, a
tooth's run-out, a via in a pad) must keep off beyond other nets' tracks, vias and pads, which each stage measures by
its own means: the board's OUTLINE and cut-outs at the edge clearance (check_drc.BoardEdgeGate), the KEEP-OUT rule
areas that keep tracks off a layer (plan_audit.keepout_areas, as the solve and the audit read them), and FILLED
graphic copper foreign to the net (check_drc.FilledCopperGate) -- the gates the array engines and the checker ask.
One reader for every such stage: Gates(pcb).track_ok / .via_ok."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', 'py_router')))

STEP = 0.05                  # a track read against a keep-out at points this far apart along it (mm)


class Gates:
    def __init__(self, pcb, edge_clearance=None):
        import check_drc as _cd
        import plan_audit as _paa
        import rules as _rules
        ec_ = edge_clearance if edge_clearance is not None else (getattr(_rules.active(), 'edge_clearance', None) or 0.2)
        self.edge = _cd.BoardEdgeGate(pcb, ec_)
        self.copper = _cd.FilledCopperGate(pcb)
        try:
            self.keepouts = _paa.keepout_areas(pcb, list(pcb.board_info.copper_layers or ()))
        except Exception:          # noqa: BLE001 -- (a board the reader cannot take: no areas, as the router's own)
            self.keepouts = []
        self._dist = _paa.keepout_dist

    def _off_keepouts(self, pts, layer, reach):
        """every point of `pts` at least `reach` outside each keep-out on `layer` (every keep-out, layer None)"""
        if not self.keepouts:
            return True
        import numpy as np
        P = np.array(pts, dtype=float)
        for poly_, holes_, lays_, _bb in self.keepouts:
            if (layer is None or layer in lays_) and float(self._dist(P, poly_, holes_).min()) < reach:
                return False
        return True

    def track_ok(self, a, b, layer, width, net_id, clearance):
        """a track a-b of `width` on `layer`, net `net_id`: on the board at its edge clearance, off every keep-out of
        its layer, clear of foreign filled copper by `clearance`"""
        if self.edge and self.edge.segment_refused(a[0], a[1], b[0], b[1], width):
            return False
        n_ = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / STEP)))
        pts = [(a[0] + (b[0] - a[0]) * k_ / n_, a[1] + (b[1] - a[1]) * k_ / n_) for k_ in range(n_ + 1)]
        if not self._off_keepouts(pts, layer, width / 2):
            return False
        return not self.copper or self.copper.segment_hit(a[0], a[1], b[0], b[1], width, layer, net_id,
                                                         clearance) is None

    def via_ok(self, pt, size, net_id, clearance):
        """a through via of `size` at `pt`, net `net_id`: on the board at its edge clearance, off every keep-out, clear
        of foreign filled copper by `clearance`"""
        if self.edge and self.edge.via_refused(pt[0], pt[1], size):
            return False
        if not self._off_keepouts([pt], None, size / 2):
            return False
        return not self.copper or self.copper.via_hit(pt[0], pt[1], size, net_id, clearance) is None

    def move_ok(self, move, net_id, width, via_size, clearance):
        """an escape move (escape_moves.Move): each of its legs a track, its site a via"""
        return (all(self.track_ok(a, b, L, width, net_id, clearance) for a, b, L in move.legs)
                and (move.site is None or self.via_ok(move.site, via_size, net_id, clearance)))
