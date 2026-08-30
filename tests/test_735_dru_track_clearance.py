#!/usr/bin/env python3
"""The .kicad_dru TRACK-scoped clearance channel, parser half (#735).

Renamed from test_549_track_clearance.py: it was never about GitHub #549
(a CLOSED placement-skill issue), only about the work-plan label that
commit fa685741 used. #735 is the live tracker for this channel.

The run-5 case this exists for: a spec clause "2.4 mm from unrelated nets"
written as a NETCLASS also binds pad-to-pad and is unsatisfiable on the
receptacle (measured pad gaps 0.5/1.3 mm) — the correct encoding is KiCad's
own track-scoped custom rule, which KRT used to DROP with a note. These tests
pin the parser subset (both A/B orderings, the != mirror → other_only, residue
rejection), the effective-map algebra (shared obstacle map ⇒ max over routed
consumers; other_only exempts sibling stamps in a members-only run), and the
config method's raise-only contract.
"""
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
# #522 moved the engine modules out of the repo root into py_router/ (and the
# placement family into py_placer/, the leaf tools into py_tools/).
for p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
          os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if p not in sys.path:
        sys.path.insert(0, p)

from kicad_dru import (TrackRule, parse_dru_track_clearances,  # noqa: E402
                       parse_dru_layer_clearances,
                       effective_track_clearances)
from routing_config import GridRouteConfig  # noqa: E402


def _rule(cond, val="2.4mm", name="usb_space"):
    return f'(version 1)\n(rule "{name}" (condition "{cond}") ' \
           f'(constraint clearance (min {val})))'


def test_both_orderings_normalize():
    for cond in ("A.Type=='track' && B.Type=='track' && A.NetClass=='USB'",
                 "B.NetClass=='USB' && B.Type=='track' && A.Type=='track'"):
        rules, _ = parse_dru_track_clearances(_rule(cond))
        assert rules == [TrackRule('usb_space', 'USB', False, 2.4)], (cond, rules)
    print("  PASS: A-side and B-side spellings parse to one symmetric rule")


def test_mirror_neq_means_other_only():
    cond = ("A.Type=='track' && B.Type=='track' && "
            "A.NetClass=='USB' && B.NetClass!='USB'")
    rules, _ = parse_dru_track_clearances(_rule(cond))
    assert rules == [TrackRule('usb_space', 'USB', True, 2.4)], rules
    print("  PASS: the != mirror parses as other_only")


def test_residue_and_malformed_shapes_are_skipped_with_the_note():
    bad = [
        # extra predicate: must not guess
        "A.Type=='track' && B.Type=='track' && A.NetClass=='USB' && A.inDiffPair('*')",
        # only one side typed track
        "A.Type=='track' && A.NetClass=='USB'",
        # != on the SAME side as ==
        "A.Type=='track' && B.Type=='track' && A.NetClass=='USB' && A.NetClass!='USB'",
        # mismatched classes between == and !=
        "A.Type=='track' && B.Type=='track' && A.NetClass=='USB' && B.NetClass!='CLK'",
        # two == terms
        "A.Type=='track' && B.Type=='track' && A.NetClass=='USB' && B.NetClass=='CLK'",
    ]
    for cond in bad:
        rules, notes = parse_dru_track_clearances(_rule(cond))
        assert rules == [], (cond, rules)
        assert any('skipped' in n for n in notes), (cond, notes)
    print(f"  PASS: {len(bad)} out-of-subset shapes skipped, note kept")


def test_track_rule_never_leaks_into_the_layer_map():
    text = _rule("A.Type=='track' && B.Type=='track' && A.NetClass=='USB'")
    lmap, notes = parse_dru_layer_clearances(text, ['F.Cu', 'B.Cu'])
    assert lmap == {}, lmap
    assert any('track channel' in n for n in notes), notes
    print("  PASS: layer map stays empty; the note says the track channel has it")


def test_effective_map_algebra():
    r = [TrackRule('usb', 'USB', False, 2.4)]
    mem = {1: {'USB'}, 2: {'USB'}, 3: set(), 4: {'CLK'}}
    all_ids = [1, 2, 3, 4]
    # routing a NON-member: members are priced, other non-members are not
    eff = effective_track_clearances(r, mem, all_ids, routed_net_ids=[3])
    assert eff == {1: 2.4, 2: 2.4}, eff
    # routing a member: everyone is priced (member-vs-member binds too)
    eff = effective_track_clearances(r, mem, all_ids, routed_net_ids=[1])
    assert eff == {1: 2.4, 2: 2.4, 3: 2.4, 4: 2.4}, eff
    print("  PASS: eff map = max over routed consumers, per obstacle net")


def test_other_only_exempts_a_members_only_run():
    """Route-the-pair-tight-first: an XTAL/USB-only run under an other_only
    rule must NOT inflate the pair's own sibling stamps."""
    r = [TrackRule('usb', 'USB', True, 2.4)]
    mem = {1: {'USB'}, 2: {'USB'}, 3: set()}
    eff = effective_track_clearances(r, mem, [1, 2, 3], routed_net_ids=[1, 2])
    # members are NOT priced against a members-only routed set; non-members are
    assert eff == {3: 2.4}, eff
    # but as soon as a non-member routes, members get priced
    eff = effective_track_clearances(r, mem, [1, 2, 3], routed_net_ids=[3])
    assert eff == {1: 2.4, 2: 2.4}, eff
    print("  PASS: other_only exempts sibling stamps in a members-only run")


def test_config_method_is_raise_only_and_inert_when_empty():
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.1,
                          track_width=0.2, clearance=0.2)
    assert cfg.track_obstacle_clearance(7, 0.3) == 0.3  # empty map: no-op
    cfg.track_clearances = {7: 2.4, 8: 0.1}
    assert cfg.track_obstacle_clearance(7, 0.3) == 2.4   # raises
    assert cfg.track_obstacle_clearance(8, 0.3) == 0.3   # sub-resolved: inert
    assert cfg.track_obstacle_clearance(9, 0.3) == 0.3   # unruled net
    print("  PASS: raise-only over the resolved value; empty map short-circuits")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
