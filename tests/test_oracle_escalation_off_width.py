"""KiCad-oracle links obey --escalation off.

The oracle recheck routed a missing plane link at the run's track width and
shipped it narrow when the net's own width did not fit: under
`--escalation off` a GND asked at 0.6 mm crossed a fence gap at 0.3 mm. Under
`off` the only narrowing allowed is a neck of at most PAD_ENTRY_NECK_MM at a
pad entry (fab_tiers), so a link with more under-width copper, or with
under-width copper away from the net's pads, is declined.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'py_router'))

import fab_tiers  # noqa: E402
from kicad_oracle import off_policy_neck_refusal  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

PAD = SimpleNamespace(global_x=0.0, global_y=0.0, size_x=1.0, size_y=1.0)
BOARD = SimpleNamespace(pads_by_net={1: [PAD]})


def _cfg():
    cfg = GridRouteConfig(track_width=0.3, layers=['F.Cu', 'B.Cu'])
    cfg.power_net_widths = {1: 0.6}
    return cfg


def _run(policy, legs, extra=(), used=0.3):
    prev = fab_tiers.get_escalation_policy()
    try:
        fab_tiers.set_escalation_policy(policy)
        return off_policy_neck_refusal(list(legs), list(extra), used,
                                       BOARD, 1, _cfg())
    finally:
        fab_tiers.set_escalation_policy(*prev)


def test_full_width_link_passes():
    assert _run('off', [(10, 0, 20, 0, 'F.Cu', 0.6)]) == ''


def test_long_narrow_link_is_declined():
    why = _run('off', [(10, 0, 20, 0, 'F.Cu', 0.3)])
    assert 'pad-entry allowance' in why, why


def test_short_neck_away_from_pads_is_declined():
    why = _run('off', [(10, 0, 11, 0, 'F.Cu', 0.3)])
    assert 'away from any pad entry' in why, why


def test_short_neck_at_a_pad_entry_passes():
    assert _run('off', [(0.5, 0, 1.5, 0, 'F.Cu', 0.3),
                        (1.5, 0, 10, 0, 'F.Cu', 0.6)]) == ''


def test_weld_connector_at_the_link_width_counts():
    why = _run('off', [], extra=[((10, 0), (20, 0), 'F.Cu')], used=0.3)
    assert 'pad-entry allowance' in why, why


def test_board_and_fab_do_not_refuse():
    for policy in ('board', 'fab'):
        assert _run(policy, [(10, 0, 20, 0, 'F.Cu', 0.3)]) == '', policy


TESTS = [test_full_width_link_passes, test_long_narrow_link_is_declined,
         test_short_neck_away_from_pads_is_declined,
         test_short_neck_at_a_pad_entry_passes,
         test_weld_connector_at_the_link_width_counts,
         test_board_and_fab_do_not_refuse]

if __name__ == '__main__':
    for t in TESTS:
        t()
    print(f"{len(TESTS)} passed")
