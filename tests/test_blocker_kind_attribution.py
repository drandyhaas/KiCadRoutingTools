#!/usr/bin/env python3
"""Blocking-obstacle attribution must distinguish rippable track from static pad.

The defect this guards (test-board run 5, journal [9]/[10] and the audit's item
2): the stuck-probe diagnostic merged segment, via and pad blocking into one
count per net, so a net whose only remaining blocking copper was its unrippable
PAD kept being cited after its tracks were ripped (GPIO7's residue), and the
decisive 1-3 cell track wall ranked under large-perimeter bystanders. Now
_identify_blocking_obstacles reports (name, total, track_count, pad_count), the
print says which kind blocks, and the TRACK owners are stashed on
pcb_data._stuck_wall_blame for the net's own phase-3 rip cascade, where
validator-named blockers sort ahead of every frontier-inferred tier.
"""
import io
import os
import sys
from contextlib import redirect_stdout

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
# #522 moved the engine modules out of the repo root into py_router/ (and the
# placement family into py_placer/, the leaf tools into py_tools/).
for p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
          os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import make_pad, make_seg, make_net, make_pcb  # noqa: E402
from kicad_parser import Footprint  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
import single_ended_routing as ser  # noqa: E402
from routing_config import GridCoord  # noqa: E402


def _pcb():
    # Net 1 is being routed; net 2 owns a track through the wall zone; net 3
    # owns only a PAD there.
    track = make_seg(4.8, 4.0, 4.8, 6.0, net_id=2)
    pad3 = make_pad(net_id=3, x=5.2, y=5.0, ref='U9', num='1', net_name='PADNET')
    fp = Footprint(reference='U9', footprint_name='t:t', x=5.2, y=5.0,
                   rotation=0.0, layer='F.Cu', pads=[pad3])
    return make_pcb(nets={1: make_net(1, 'ME'), 2: make_net(2, 'TRACKNET'),
                          3: make_net(3, 'PADNET')},
                    segments=[track],
                    footprints={'U9': fp},
                    pads_by_net={3: [pad3]})


def test_kind_split_and_blame_stash():
    pcb = _pcb()
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.1,
                          track_width=0.2, clearance=0.2)
    coord = GridCoord(cfg.grid_step)
    gx, gy = coord.to_grid(5.0, 5.0)
    blocked = [(gx, gy, 0)]
    out = ser._identify_blocking_obstacles(blocked, pcb, cfg, current_net_id=1)
    assert 2 in out and 3 in out, f"expected both blockers, got {out}"
    n2, tot2, tc2, pc2 = out[2]
    n3, tot3, tc3, pc3 = out[3]
    assert (n2, pc2) == ('TRACKNET', 0) and tc2 > 0, out[2]
    assert (n3, tc3) == ('PADNET', 0) and pc3 > 0, out[3]
    print(f"  PASS: kinds split (TRACKNET {tc2} track / PADNET {pc3} pad)")


class _FakeObstacles:
    def is_blocked(self, x, y, layer):
        return True  # everything walled: the printer must attribute all 8


def test_printer_stashes_track_blame_only():
    pcb = _pcb()
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.1,
                          track_width=0.2, clearance=0.2)
    buf = io.StringIO()
    with redirect_stdout(buf):
        from routing_config import GridCoord as _GC
        _gx, _gy = _GC(cfg.grid_step).to_grid(5.0, 5.0)
        ser._diagnose_blocked_start(
            _FakeObstacles(), [(_gx, _gy, 0)], 'probe',
            pcb_data=pcb, config=cfg, current_net_id=1)
    text = buf.getvalue()
    assert 'track' in text and 'pad' in text, f"kinds absent from print:\n{text}"
    blame = getattr(pcb, '_stuck_wall_blame', {})
    assert blame.get(1) == {2}, \
        f"blame must carry the TRACK owner only, got {blame}"
    print("  PASS: print names kinds; blame stashes track owners only")


if __name__ == '__main__':
    test_kind_split_and_blame_stash()
    test_printer_stashes_track_blame_only()
