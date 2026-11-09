#!/usr/bin/env python3
"""#1727: a via the pad clamp shrinks into its pad keeps the board's `.kicad_dru`
via rules wherever a via of that size can meet them.

`clamp_via_to_pad` sizes a via-in-pad to the pad and keeps the configured drill,
thinning it only as far as the chosen rung's OWN ring needs. On a fine fab's
rules (annular_width 0.12, hole_size 0.15, via_diameter 0.3) a 0.40 pad took the
0.3/0.15 rung and came out 0.40/0.25: ring 0.075, under the rule, although
0.40/0.16 meets every rule and fits the same pad. The ladder's first rung meets
the rules (#1676); fitting the pad is no reason to drop one. So:

  1. a clamped via takes the drill that meets the rules (ring and hole) when a
     via of its size can; the request's drill where that already does.
  2. where no via in the pad can (under the diameter minimum, or too small to
     hold the ring round the hole minimum) the clamp is unchanged: every via
     there breaks a rule, and the clamped one is never worse than its rung.
  3. a net's own rule (`rule_floors`, the `extra_floors` its ladder was built
     with) holds its clamp the same way.
  4. with no rule file, under `--escalation board`, and for a via that fits,
     nothing changes.

    python3 tests/test_1727_pad_clamp_keeps_rule_ring.py
"""
import os
import sys
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import fab_tiers  # noqa: E402
from bga_fanout.geometry import clamp_via_to_pad  # noqa: E402

#: A fine fab's via rules, as `DesignRules.via_floors` reads them.
FINE = {'via_diameter': 0.3, 'via_drill': 0.15, 'annular': 0.12}
#: A fab's 4-layer rules (#1676), where no via under 0.454 meets them.
STD = {'via_diameter': 0.45, 'via_drill': 0.2, 'annular': 0.127}


def pad(size, net=1):
    return SimpleNamespace(size_x=size, size_y=size, global_x=0.0, global_y=0.0,
                           net_id=net, component_ref='U1', pad_number='1')


def clamp(req, p, policy='fab', floors=None, rule_floors=None, n=4, extra=None):
    fab_tiers.set_default_fab_tier('auto')
    fab_tiers.set_escalation_policy(policy, {})
    fab_tiers.set_rule_via_floors(floors or {})
    try:
        lad = fab_tiers.escalation_rungs(n, extra_floors=extra)
        kw = {} if rule_floors is None else {'rule_floors': rule_floors}
        s, d, st, rung = clamp_via_to_pad(req[0], req[1], p, lad, **kw)
        return round(s, 4), round(d, 4), st, rung
    finally:
        fab_tiers.set_rule_via_floors({})
        fab_tiers.set_escalation_policy('fab', {})


def meets(s, d, rules):
    return (s >= rules.get('via_diameter', 0) - 1e-9 and d >= rules.get('via_drill', 0) - 1e-9
            and (s - d) / 2.0 >= rules.get('annular', 0) - 1e-6)


def main():
    print('1. the clamp keeps the rules where a via of its size can')
    got = clamp((0.54, 0.3), pad(0.40), floors=FINE)
    assert got[:3] == (0.40, 0.16, 'clamped'), got
    assert meets(got[0], got[1], FINE), f'ring {(got[0] - got[1]) / 2:.4f} under the rule: {got}'
    got = clamp((0.54, 0.3), pad(0.42), floors=FINE)
    assert got[:2] == (0.42, 0.18) and meets(got[0], got[1], FINE), got
    roomy = clamp((0.45, 0.15), pad(0.40), floors=FINE)
    assert roomy[:2] == (0.40, 0.15), f'the request drill already meets them: {roomy}'
    print('   PASS')

    print('2. no via in the pad meets them: the clamp is as it was')
    assert clamp((0.454, 0.2), pad(0.40), floors=STD)[:3] == (0.40, 0.2, 'clamped')
    assert clamp((0.54, 0.3), pad(0.38), floors=FINE)[:2] == (0.38, 0.23), \
        'too small to hold 0.12 round 0.15'
    print('   PASS')

    print("3. a net's own rule holds its clamp")
    extra = {'first_' + k: v for k, v in FINE.items()}
    held = clamp((0.54, 0.3), pad(0.40), extra=extra, rule_floors=extra)
    assert held[:2] == (0.40, 0.16), held
    print('   PASS')

    print('4. unchanged: no rules, board, a via that fits')
    assert clamp((0.54, 0.3), pad(0.40))[:2] == (0.40, 0.25), 'no rule file'
    assert clamp((0.3, 0.15), pad(0.40), floors=FINE)[2] == 'fits'
    board = clamp((0.54, 0.3), pad(0.40), policy='board', floors=FINE)
    assert meets(board[0], board[1], FINE), board
    print('   PASS')
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
