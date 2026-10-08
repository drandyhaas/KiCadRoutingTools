"""Plane fragility is published per pour net, and a net never pays its own.

The field used to go out under one cache key for every pour, merged for every
net, so a plane net routed in the step (the #562 route step takes them, and
the finalize's joins and reconnects do too) paid fragility on its own pour's
necks -- where its joins and stitches belong -- and its vias 10x that. It is
now one entry per pour net (`plane_fragility.fragility_cache_key`) and every
builder leaves the routed net's own out.

Also: a batch handed an in-memory board carves that board's copper into the
field once at registration (the field is rasterized from the FILE's fill),
and the stub-swap rescue refreshes the field for the stub it moved.

Rows:
  - registration writes one key per pour net, and a refresh republishes them;
  - the routing_context helper drops exactly the routed net(s)' own pours;
  - carving in-memory copper raises the cost along a foreign track through a
    pour, and is a no-op without a dynamic field;
  - the stub-swap footprint carries the moved stub on both layers.

    python3 tests/test_fragility_per_pour_net.py
"""

import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import plane_fragility as pf  # noqa: E402
from kicad_parser import BoardInfo, Net, PCBData, Segment, Zone  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

GND, VCC, SIG = 1, 2, 3


def _board():
    """Two pours on F.Cu (GND left, VCC right), no source file, so the field
    is rasterized from the outlines."""
    nets = {GND: Net(GND, 'GND'), VCC: Net(VCC, 'VCC'), SIG: Net(SIG, 'SIG')}
    zones = [Zone(GND, 'GND', 'F.Cu', [(0, 0), (10, 0), (10, 10), (0, 10)]),
             Zone(VCC, 'VCC', 'F.Cu', [(20, 0), (30, 0), (30, 10), (20, 10)])]
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    return PCBData(board_info=bi, nets=nets, footprints={}, vias=[], segments=[],
                   pads_by_net={}, zones=zones)


def _register(pcb, cache):
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.2)
    with contextlib.redirect_stdout(io.StringIO()):
        pf.register_plane_fragility(pcb, cfg, cache)
    return cfg


def test_one_key_per_pour_net():
    pcb, cache = _board(), {}
    cfg = _register(pcb, cache)
    keys = {k for k in cache if isinstance(k, tuple)}
    assert keys == {pf.fragility_cache_key(GND), pf.fragility_cache_key(VCC)}, keys
    assert cfg._fragility_field is not None, "dynamic field not armed"
    gnd_rows = cache[pf.fragility_cache_key(GND)]
    assert gnd_rows[:, 1].max() <= 101, "GND rows reach VCC's pour"
    cfg._fragility_field.publish()
    assert {k for k in cache if isinstance(k, tuple)} == keys, "republish lost keys"


def test_builders_drop_the_own_pour():
    from routing_context import _per_net_cost_sources
    pcb, cache = _board(), {}
    _register(pcb, cache)
    cache[7] = 'track proximity of net 7'
    got = _per_net_cost_sources(cache, (GND,))
    assert pf.fragility_cache_key(GND) not in got, "own pour still priced"
    assert pf.fragility_cache_key(VCC) in got and 7 in got, sorted(map(str, got))
    assert _per_net_cost_sources(cache, (SIG,)) is cache, "copied for nothing"
    pair = _per_net_cost_sources(cache, (GND, VCC))
    assert not any(isinstance(k, tuple) for k in pair), pair.keys()


def test_in_memory_copper_is_carved_once():
    pcb, cache = _board(), {}
    cfg = _register(pcb, cache)
    key = pf.fragility_cache_key(GND)
    before = {(int(r[1]), int(r[2])): int(r[3]) for r in cache[key]}
    # A foreign track across the GND pour, present in memory but not in the
    # fill the field was rasterized from.
    pcb.segments.append(Segment(5.0, 0.5, 5.0, 9.5, 0.2, 'F.Cu', SIG))
    pf.carve_in_memory_copper(cfg, pcb)
    after = {(int(r[1]), int(r[2])): int(r[3]) for r in cache[key]}
    # 0.5 mm from the new track: mid-pour (unpriced) before, the edge of the
    # neck the track opened after. (0.3 mm is inside the carved hole.)
    beside = (45, 50)
    assert beside not in before and after.get(beside, 0) > 0, (
        before.get(beside), after.get(beside))
    static = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    pf.carve_in_memory_copper(static, pcb)       # no field: nothing to do


def test_stub_swap_footprint_covers_both_layers():
    from single_ended_loop import _swap_footprint
    mods = [{'start': (1.0, 2.0), 'end': (3.0, 2.0), 'net_id': SIG,
             'net_name': 'SIG', 'old_layer': 'F.Cu', 'new_layer': 'B.Cu'}]
    fp = _swap_footprint(mods)
    assert sorted(s.layer for s in fp) == ['B.Cu', 'F.Cu'], fp
    assert all((s.start_x, s.end_x) == (1.0, 3.0) for s in fp)


TESTS = [test_one_key_per_pour_net, test_builders_drop_the_own_pour,
         test_in_memory_copper_is_carved_once,
         test_stub_swap_footprint_covers_both_layers]


if __name__ == '__main__':
    fails = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  FAIL {t.__name__}: {e}")
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
