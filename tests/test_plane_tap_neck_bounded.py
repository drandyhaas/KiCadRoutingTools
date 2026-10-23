"""A plane tap for a wide net is a pad-entry neck only while it is short.

cm5-io +5V (2 mm class) shipped 47 taps at the 0.09 mm board default, one
7.25 mm long, under --escalation off. _bound_tap_neck refuses a tap longer than
PAD_ENTRY_NECK_MM under 'off', keeps short ones, records both in the ledger,
and never touches a tap whose net width is the board default.
"""
import os
import sys
from dataclasses import replace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'py_router'))

import fab_tiers  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from route_planes import _bound_tap_neck  # noqa: E402
from routing_defaults import PAD_ENTRY_NECK_MM  # noqa: E402


def _cfg(power=None):
    cfg = GridRouteConfig(track_width=0.09, clearance=0.1, via_size=0.25, via_drill=0.15,
                          grid_step=0.05, layers=['F.Cu', 'B.Cu'])
    return replace(cfg, power_net_widths=dict(power or {}))


def _tap(length_mm, n=4):
    step = length_mm / n
    return [{'start': (i * step, 0.0), 'end': ((i + 1) * step, 0.0), 'width': 0.09,
             'layer': 'F.Cu', 'net_id': 7} for i in range(n)]


def _with_policy(policy, fn):
    prev = fab_tiers.get_escalation_policy()
    fab_tiers.set_escalation_policy(policy)
    try:
        return fn()
    finally:
        fab_tiers.set_escalation_policy(*prev)


def test_long_tap_for_wide_net_refused_under_off():
    cfg = _cfg({7: 2.0})
    out = _with_policy('off', lambda: _bound_tap_neck(_tap(PAD_ENTRY_NECK_MM * 3), 7, 'F.Cu', cfg))
    assert out is None


def test_short_tap_kept_and_recorded():
    cfg = _cfg({7: 2.0})
    segs = _tap(1.0)

    def run():
        # Read the ledger INSIDE the policy scope: set_escalation_policy resets it.
        out = _bound_tap_neck(segs, 7, 'F.Cu', cfg)
        rows = [r for r in fab_tiers._LEDGER['narrowed'] if r['site'] == 'plane tap']
        return out, rows

    out, rows = _with_policy('off', run)
    assert out is segs
    assert rows and rows[-1]['requested'] == 2.0 and rows[-1]['delivered'] == 0.09


def test_default_width_net_untouched():
    cfg = _cfg()
    segs = _tap(9.0)
    out = _with_policy('off', lambda: _bound_tap_neck(segs, 7, 'F.Cu', cfg))
    assert out is segs


def test_fab_policy_keeps_long_tap():
    cfg = _cfg({7: 2.0})
    segs = _tap(9.0)
    out = _with_policy('fab', lambda: _bound_tap_neck(segs, 7, 'F.Cu', cfg))
    assert out is segs


TESTS = [test_long_tap_for_wide_net_refused_under_off, test_short_tap_kept_and_recorded,
         test_default_width_net_untouched, test_fab_policy_keeps_long_tap]

if __name__ == '__main__':
    for t in TESTS:
        t()
        print('ok', t.__name__)
    print('ALL PASS')
