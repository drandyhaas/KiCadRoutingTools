"""Copper edited outside add/remove_route_to_pcb_data invalidates what was
cached against the old copper.

Several memos key on `pcb_data._copper_epoch` -- net_rescue's pristine rescue
maps, the chip-pad escape memo, the block-id geometry and via-placement
failure memos. add/remove_route bump it, but many passes edit the copper
lists in place or replace them (the length-match sync, stub layer switches
and their revert, the cleanup pipeline, the plane finalize, the GUI's board
sync), and a memo that survives them answers for copper that no longer
exists: a rescue routed on a cached map of the old board can cross copper the
map never saw. Those passes now call `pcb_modification.bump_copper_epoch`.

Rows:
  - the length-match sync and a stub-switch revert bump the epoch;
  - the cleanup pipeline bumps it;
  - net_rescue's pristine-map cache hits on an unchanged board and misses
    once the epoch moves.

    python3 tests/test_copper_epoch_invalidation.py
"""

import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import evidence  # noqa: E402

from kicad_parser import Segment, parse_kicad_pcb  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def _board():
    return _quiet(parse_kicad_pcb, evidence(BOARD))


def _epoch(pcb):
    return getattr(pcb, '_copper_epoch', 0)


def test_sync_and_stub_revert_bump():
    from routing_common import sync_pcb_data_segments
    from stub_layer_switching import revert_stub_layer_switch
    pcb = _board()
    nid = next(n for n, net in pcb.nets.items() if net.name == '/SENSOR_A')
    seg = Segment(1.0, 1.0, 2.0, 1.0, 0.2, 'F.Cu', nid)
    e0 = _epoch(pcb)
    _quiet(sync_pcb_data_segments, pcb, {nid: {'new_segments': [seg]}}, set())
    assert _epoch(pcb) > e0, "the length-match sync did not bump the epoch"
    e1 = _epoch(pcb)
    revert_stub_layer_switch(pcb, [], [])
    assert _epoch(pcb) > e1, "a stub-switch revert did not bump the epoch"


def test_cleanup_pipeline_bumps():
    from cleanup_pipeline import run_post_route_cleanup
    pcb = _board()
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    e0 = _epoch(pcb)
    _quiet(run_post_route_cleanup, [], pcb, set(), cfg)
    assert _epoch(pcb) > e0, "the cleanup pipeline did not bump the epoch"


def test_rescue_map_cache_misses_after_a_bump():
    from net_rescue import _pristine_rescue_map
    from pcb_modification import bump_copper_epoch
    pcb = _board()
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    nid = next(n for n, net in pcb.nets.items() if net.name == '/SENSOR_A')
    _quiet(_pristine_rescue_map, pcb, pcb, cfg, nid, None, ('full',))
    first = dict(pcb._rescue_map_cache)
    _quiet(_pristine_rescue_map, pcb, pcb, cfg, nid, None, ('full',))
    assert dict(pcb._rescue_map_cache) == first, "an unchanged board missed"
    bump_copper_epoch(pcb)
    _quiet(_pristine_rescue_map, pcb, pcb, cfg, nid, None, ('full',))
    (key, entry), = pcb._rescue_map_cache.items()
    assert key[2] == _epoch(pcb), key
    assert entry is not next(iter(first.values())), "served the old map"


TESTS = [test_sync_and_stub_revert_bump, test_cleanup_pipeline_bumps,
         test_rescue_map_cache_misses_after_a_bump]


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
