"""The plane finalize must see the route step's netclass widths.

repair_planes built its own config, and its only per-net widths were explicit
--power-nets / --power-nets-widths. A net that is wide by its NETCLASS (the
#435 auto-read, config.net_track_widths) read as the board default there, so
_bound_tap_neck (#1452) did not treat a pad-repair tap at the default width as
a neck: under --escalation off a 0.8 mm class net shipped a 3.25 mm tap at
0.2 mm. route.py now forwards net_track_widths / netclass_width_floors and
repair_planes applies them (apply_net_widths).
"""
import inspect
import os
import sys
from dataclasses import replace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'py_router'))

import fab_tiers  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from route_planes import _bound_tap_neck  # noqa: E402
from routing_defaults import PAD_ENTRY_NECK_MM  # noqa: E402
import repair_planes  # noqa: E402


def _cfg():
    return GridRouteConfig(track_width=0.2, clearance=0.2, via_size=0.6, via_drill=0.3,
                           grid_step=0.05, layers=['F.Cu', 'B.Cu'])


def _tap(length_mm, n=4):
    step = length_mm / n
    return [{'start': (i * step, 0.0), 'end': ((i + 1) * step, 0.0), 'width': 0.2,
             'layer': 'B.Cu', 'net_id': 7} for i in range(n)]


def _off(fn):
    prev = fab_tiers.get_escalation_policy()
    fab_tiers.set_escalation_policy('off')
    try:
        return fn()
    finally:
        fab_tiers.set_escalation_policy(*prev)


def test_without_the_class_widths_a_long_tap_passes():
    # the defect: the repair config does not know net 7 is a 0.8 mm class net
    segs = _tap(PAD_ENTRY_NECK_MM * 3)
    assert _off(lambda: _bound_tap_neck(segs, 7, 'B.Cu', _cfg())) is segs


def test_forwarded_class_widths_bound_the_tap():
    cfg = repair_planes.apply_net_widths(_cfg(), {7: 0.8}, {7: 0.8})
    assert cfg.get_net_track_width(7, 'B.Cu') == 0.8
    assert _off(lambda: _bound_tap_neck(_tap(PAD_ENTRY_NECK_MM * 3), 7, 'B.Cu', cfg)) is None
    short = _tap(1.0)
    assert _off(lambda: _bound_tap_neck(short, 7, 'B.Cu', cfg)) is short


def test_apply_net_widths_keeps_existing_entries_and_ignores_empty():
    cfg = replace(_cfg(), net_track_widths={7: 1.0})
    repair_planes.apply_net_widths(cfg, {7: 0.8, 8: 0.5}, None)
    assert cfg.net_track_widths == {7: 1.0, 8: 0.5}
    before = dict(cfg.net_track_widths)
    repair_planes.apply_net_widths(cfg, None, {})
    assert cfg.net_track_widths == before


def test_repair_entry_accepts_the_widths():
    params = inspect.signature(repair_planes._repair_planes_all).parameters
    assert 'net_track_widths' in params and 'netclass_width_floors' in params


TESTS = [test_without_the_class_widths_a_long_tap_passes,
         test_forwarded_class_widths_bound_the_tap,
         test_apply_net_widths_keeps_existing_entries_and_ignores_empty,
         test_repair_entry_accepts_the_widths]

if __name__ == '__main__':
    for t in TESTS:
        t()
        print(f"PASS {t.__name__}")
