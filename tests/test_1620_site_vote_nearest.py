#!/usr/bin/env python3
"""The plane-repair site vote gives a site held by a POURED net no vote (#1620).

_via_site_consensus_blocker polls a ring of via sites around a blocked plane
pad and rips the net that holds the most of them, each site counting for its
nearest rippable net. #1620 made every poured net never-rip, so a site whose
nearest net is a pour started voting for the runner-up behind the pour -- a
net whose rip frees none of those sites. On a corpus board with a second pour
that elected a different blocker for a plane-pad tap than before #1620, and
the step ended with more open nets. A pour-held site now casts no vote; a site
held by any other never-rip net (a routed diff pair, locked copper) votes for
its nearest rippable net as before.

Fixture: a rippable net hidden between two strands of a never-rip net, so
every site that reaches it is nearer the never-rip copper; a second rippable
net on the open side holds sites of its own.
  - the never-rip net is a POUR: its sites are silent, the open-side net wins;
  - the never-rip net is a protected NON-pour net: its sites vote for the net
    behind it, as they always have, and the hidden net wins;
  - the pour is not protected at all: it wins its own sites.

Run:  python3 tests/test_1620_site_vote_nearest.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo, Net, PCBData, Pad, Segment, Zone
from routing_config import GridRouteConfig
import repair_planes as rdp

GND, WALL, HIDDEN, OPEN = 2, 3, 5, 6
FAILURES = []


def _check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{(' -- ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(name)


def _seg(x1, y1, x2, y2, net_id):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=0.2, layer="F.Cu", net_id=net_id)


def _board(wall_pours):
    bi = BoardInfo(layers={0: "F.Cu", 31: "B.Cu"}, copper_layers=["F.Cu", "B.Cu"])
    bi.board_bounds = (0.0, 0.0, 20.0, 20.0)
    pad = Pad(component_ref="C1", pad_number="2", global_x=10.0, global_y=10.0,
              local_x=0.0, local_y=0.0, size_x=0.6, size_y=0.6, shape="rect",
              layers=["F.Cu"], net_id=GND, net_name="/GND", pad_type="smd")
    segments = [
        # the never-rip net, two strands either side of the hidden net
        _seg(8.5, 10.6, 11.5, 10.6, WALL),
        _seg(8.5, 10.8, 11.5, 10.8, WALL),
        # every site within reach of this one is nearer a WALL strand
        _seg(9.1, 10.7, 10.9, 10.7, HIDDEN),
        # the open side: a rippable net that holds sites of its own
        _seg(9.9, 9.4, 10.1, 9.4, OPEN),
    ]
    square = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
    zones = [Zone(net_id=GND, net_name="/GND", layer="B.Cu", polygon=square)]
    if wall_pours:
        zones.append(Zone(net_id=WALL, net_name="/WALL", layer="In1.Cu",
                          polygon=square))
    pcb = PCBData(board_info=bi,
                  nets={GND: Net(GND, "/GND"), WALL: Net(WALL, "/WALL"),
                        HIDDEN: Net(HIDDEN, "/HIDDEN"), OPEN: Net(OPEN, "/OPEN")},
                  footprints={}, vias=[], segments=segments,
                  pads_by_net={GND: [pad]}, zones=zones)
    return pcb, pad


def _vote(wall_pours, protected):
    pcb, pad = _board(wall_pours)
    cfg = GridRouteConfig(track_width=0.2, clearance=0.09, via_size=0.45,
                          via_drill=0.2, grid_step=0.05)
    won = rdp._via_site_consensus_blocker(
        pad, pcb, cfg, GND, protected_net_ids=protected,
        exclude_net_ids=set(), max_search_radius=1.2)
    return won, (pcb.nets[won].name if won in pcb.nets else str(won))


def main():
    print("plane-repair site vote: a pour-held site casts no vote (#1620):")
    won, name = _vote(wall_pours=True, protected={GND, WALL})
    _check("never-rip POUR: the net hidden behind it is not elected",
           won != HIDDEN, f"elected {name}")
    _check("never-rip POUR: the open-side net is elected", won == OPEN,
           f"elected {name}")

    won, name = _vote(wall_pours=False, protected={GND, WALL})
    _check("never-rip NON-pour net: its sites vote for the net behind it, as before",
           won == HIDDEN, f"elected {name}")

    won, name = _vote(wall_pours=True, protected={GND})
    _check("a pour that may be ripped wins its own sites", won == WALL,
           f"elected {name}")

    if FAILURES:
        print(f"\nFAIL: {len(FAILURES)} case(s): {', '.join(FAILURES)}")
        return 1
    print("\nPASS: a site held by a never-rip pour casts no vote.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
