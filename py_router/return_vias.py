"""Ground return-via coverage: which signal vias have a ground via or a
drilled ground pad of their OWN ground domain within a distance (#1816).

One read-only test with two callers, so the step and the check cannot drift:
`add_gnd_vias.add_gnd_vias_to_existing_board` (route_planes --add-gnd-vias and
the Planes tab) skips a covered via with it, and `py_tools/check_return_vias.py`
and board_score's `advisory.return_vias` report the vias it leaves uncovered.

A SIGNAL via is a via on a net that is neither ground nor power:
- ground is every ground domain (`net_queries.resolve_ground_domains`, #489)
  plus the resolved return net;
- power is `list_nets.power_net_evidence`: a rail NAME, power pins on two or
  more parts, or a pour the board draws for the net. A supply rail's via is
  not a signal and needs no return via of its own.
Return copper is the ground net's vias and its drilled pads (`pad.drill > 0`).
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional


class ReturnViaModel:
    """The board's ground resolution, its signal vias and its return copper.

    gnd_net_name: an explicit return net (--gnd-via-net) pins every via to
    it; empty resolves each via's return net from its own ground domain.
    zones: the zones the board will hold, when `pcb_data.zones` does not hold
    this call's pours yet (they decide which nets read as power)."""

    def __init__(self, pcb_data, gnd_net_name: Optional[str] = None,
                 zones=None):
        from net_queries import (resolve_gnd_net_id, resolve_ground_domains)
        from list_nets import power_net_evidence
        self.pcb_data = pcb_data
        self.explicit_name = (gnd_net_name or "").strip() or None
        # An exact name match first (so an explicit --gnd-via-net is
        # honoured verbatim), then the GND-family resolver.
        self.gnd_net_id = None
        self.inexact_name = None    # resolved name when the explicit one missed
        if self.explicit_name:
            for net_id, net in pcb_data.nets.items():
                if net.name == self.explicit_name:
                    self.gnd_net_id = net_id
                    break
        if self.gnd_net_id is None:
            self.gnd_net_id, resolved = resolve_gnd_net_id(pcb_data,
                                                           self.explicit_name)
            if self.gnd_net_id is not None and self.explicit_name:
                self.inexact_name = resolved
        self.domains = {nid: refs for nid, refs
                        in resolve_ground_domains(pcb_data).items() if refs}
        # Every ground DOMAIN's vias are excluded, not just the resolved one:
        # an AGND via is not a signal via needing a return path.
        self.ground_ids = set(self.domains) or {self.gnd_net_id}
        self.ground_ids.add(self.gnd_net_id)
        self.power_ids = set(power_net_evidence(pcb_data, zones)) \
            - self.ground_ids
        # Existing return copper, per ground net: a via only counts as having
        # a return if the ground copper is in the SAME domain (#489 §5).
        self.existing_by_net: Dict[int, List] = {}
        for nid in self.ground_ids:
            pts = [(v.x, v.y) for v in pcb_data.vias if v.net_id == nid]
            # Drilled pads of that net act as return vias too.
            for pad in pcb_data.pads_by_net.get(nid, []):
                if pad.drill and pad.drill > 0:
                    pts.append((pad.global_x, pad.global_y))
            self.existing_by_net[nid] = pts

    @property
    def found(self) -> bool:
        return self.gnd_net_id is not None

    def return_net_for(self, via) -> int:
        """The ground net a signal via returns on: the pinned net, else its
        own domain's (single-domain boards resolve to the one ground)."""
        if self.explicit_name or len(self.domains) < 2:
            return self.gnd_net_id
        from net_queries import resolve_return_net_id
        dom_id, _name = resolve_return_net_id(self.pcb_data, via.net_id,
                                              self.explicit_name)
        return dom_id if dom_id is not None else self.gnd_net_id

    def is_signal(self, net_id) -> bool:
        return bool(net_id) and net_id not in self.ground_ids \
            and net_id not in self.power_ids

    def signal_vias(self, net_ids: Optional[Iterable[int]] = None) -> list:
        want = None if net_ids is None else set(net_ids)
        return [v for v in self.pcb_data.vias if self.is_signal(v.net_id)
                and (want is None or v.net_id in want)]

    def nearest_return(self, via, extra: Iterable = ()) -> float:
        """Distance from VIA to the nearest return point of its own return
        net (EXTRA: further points of that net, e.g. vias being placed);
        inf when there is none."""
        pts = self.existing_by_net.get(self.return_net_for(via), [])
        return min((math.hypot(via.x - x, via.y - y)
                    for x, y in (*pts, *extra)), default=math.inf)


def return_via_coverage(pcb_data, distance: float, net_ids=None,
                        gnd_net_name: Optional[str] = None) -> dict:
    """The coverage report: every signal via (on NET_IDS when given) with no
    return point of its own ground domain within DISTANCE mm.

    {'distance_mm', 'gnd_net', 'signal_vias', 'uncovered', 'power_vias',
     'by_net': {name: {'signal_vias', 'uncovered'}},
     'uncovered_vias': [{'net', 'x', 'y', 'return_net', 'nearest_mm'}]}
    `uncovered` is None, with a `reason`, when the board has no ground net.
    `power_vias` counts the vias left out as power (on NET_IDS when given)."""
    model = ReturnViaModel(pcb_data, gnd_net_name)
    nets = pcb_data.nets
    want = None if net_ids is None else set(net_ids)

    def name(nid):
        n = nets.get(nid)
        return n.name if n and n.name else str(nid)
    power = sum(1 for v in pcb_data.vias if v.net_id in model.power_ids
                and (want is None or v.net_id in want))
    sig = model.signal_vias(want)
    out = {'distance_mm': distance,
           'gnd_net': name(model.gnd_net_id) if model.found else None,
           'signal_vias': len(sig), 'power_vias': power}
    if not model.found:
        target = model.explicit_name or 'GND family'
        return {**out, 'uncovered': None, 'by_net': {}, 'uncovered_vias': [],
                'reason': f"no ground net found ({target})"}
    by_net: Dict[str, Dict[str, int]] = {}
    bare = []
    for v in sig:
        row = by_net.setdefault(name(v.net_id),
                                {'signal_vias': 0, 'uncovered': 0})
        row['signal_vias'] += 1
        d = model.nearest_return(v)
        if d > distance:
            row['uncovered'] += 1
            bare.append({'net': name(v.net_id), 'x': round(v.x, 4),
                         'y': round(v.y, 4),
                         'return_net': name(model.return_net_for(v)),
                         'nearest_mm': None if math.isinf(d) else round(d, 3)})
    return {**out, 'uncovered': len(bare), 'by_net': by_net,
            'uncovered_vias': bare}
