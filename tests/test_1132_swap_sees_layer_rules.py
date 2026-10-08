"""#1132: route.py's layer-swap pass sees the board's .kicad_dru rules.

`apply_single_ended_layer_swaps` prices its admission checks through the
config's layer rules and shrinks its vias down a ladder floored at the rule
minimums (`config.rule_floors`, which reads `config.rules`). route.py used to
install both only AFTER the swap pass, while route_diff.py installs them
before its own, so the two fronts of the same engine judged one board
differently.

A spy replaces each front's swap pass, records what its config carries when
the pass starts, and stops the run: nothing is routed or written. Each board
is staged with a sibling .kicad_dru holding an F.Cu layer rule.

    python3 tests/test_1132_swap_sees_layer_rules.py
"""

import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))

from run_utils import evidence  # noqa: E402

DRU = ('(version 1)\n(rule "tight_f" (layer "F.Cu") '
       '(constraint clearance (min 0.3mm)))\n')


class _Seen(Exception):
    pass


def _spy(label, seen):
    def f(pcb_data, config, *a, **k):
        seen[label] = {
            'layer_clearances': dict(config.layer_clearances or {}),
            'rules': config.rules is not None,
        }
        raise _Seen
    return f


def _stage(name, td):
    from copy_board import copy_board
    dst = os.path.join(td, name + '.kicad_pcb')
    copy_board(os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb'), dst)
    with open(os.path.splitext(dst)[0] + '.kicad_dru', 'w',
              encoding='utf-8') as fh:
        fh.write(DRU)
    return evidence(dst)


def main():
    import route
    import route_diff
    from kicad_parser import parse_kicad_pcb

    td = tempfile.mkdtemp(prefix='t1132_')
    saved = (route.apply_single_ended_layer_swaps,
             route_diff.apply_diff_pair_layer_swaps)
    seen = {}
    try:
        se = _stage('splitflap_driver', td)   # unrouted single-ended nets
        dp = _stage('cap_chain', td)          # unrouted diff pairs DPA / DPB
        se_nets = sorted(n.name for n in parse_kicad_pcb(se).nets.values()
                         if n.name)
        route.apply_single_ended_layer_swaps = _spy('route.py', seen)
        route_diff.apply_diff_pair_layer_swaps = _spy('route_diff.py', seen)
        for name, call in (
                ('route.py', lambda: route.batch_route(
                    se, os.path.join(td, 'o1.kicad_pcb'), se_nets,
                    enable_layer_switch=True)),
                ('route_diff.py', lambda: route_diff.batch_route_diff_pairs(
                    dp, os.path.join(td, 'o2.kicad_pcb'), ['DPA_P', 'DPA_N'],
                    enable_layer_switch=True))):
            try:
                call()
            except _Seen:
                pass
    finally:
        (route.apply_single_ended_layer_swaps,
         route_diff.apply_diff_pair_layer_swaps) = saved
        shutil.rmtree(td, ignore_errors=True)

    fails = []
    for front in ('route.py', 'route_diff.py'):
        got = seen.get(front)
        print(f"  {front:14s} swap pass sees: {got}")
        if got is None:
            fails.append(f"{front}: the swap pass was never reached "
                         f"(a broken fixture, not a verdict)")
            continue
        if got['layer_clearances'] != {'F.Cu': 0.3}:
            fails.append(f"{front}: layer rules {got['layer_clearances']} "
                         f"!= {{'F.Cu': 0.3}}")
        if not got['rules']:
            fails.append(f"{front}: config.rules not installed, so "
                         f"rule_floors cannot read the board's minimums")
    fails += _track_map_over_the_routed_ids()
    for f in fails:
        print(f"  FAIL {f}")
    if fails:
        return 1
    print("PASS: both fronts' swap passes see the .kicad_dru rules")
    return 0


TRACK_DRU = ('(version 1)\n(rule "x" (constraint clearance (min 0.5mm)) '
             '(condition "A.Type == \'track\' && B.Type == \'track\' && '
             'A.NetClass == \'XTAL\' && B.NetClass != \'XTAL\'"))\n')


def _track_map_over_the_routed_ids():
    """The swap passes' #735 track map is the one over the nets this call
    routes. An other_only rule makes it depend on WHICH side is routed:
    routing the XTAL members prices the non-members, not the members. (The
    early install was once handed batch_route's (name, id) tuples, matched
    no membership, and priced the inverse set.)"""
    import json
    import route
    from constraint_agreement import DEFAULT_CLASS
    from kicad_dru import read_board_track_clearances, \
        effective_track_clearances
    from kicad_parser import parse_kicad_pcb
    from list_nets import net_class_memberships

    td = tempfile.mkdtemp(prefix='t1132t_')
    saved = route.apply_single_ended_layer_swaps
    seen = {}
    try:
        dst = os.path.join(td, 'sf.kicad_pcb')
        shutil.copy(os.path.join(ROOT, 'kicad_files',
                                 'splitflap_driver.kicad_pcb'), dst)
        proj = {"board": {"design_settings": {"rules": {"min_clearance": 0.0}}},
                "meta": {"filename": "sf.kicad_pro", "version": 1},
                "net_settings": {
                    "classes": [dict(DEFAULT_CLASS),
                                dict(DEFAULT_CLASS, name='XTAL', priority=0)],
                    "meta": {"version": 3},
                    "netclass_patterns": [{"pattern": "/LED_*",
                                           "netclass": "XTAL"}]}}
        with open(os.path.join(td, 'sf.kicad_pro'), 'w',
                  encoding='utf-8') as fh:
            json.dump(proj, fh)
        with open(os.path.join(td, 'sf.kicad_dru'), 'w',
                  encoding='utf-8') as fh:
            fh.write(TRACK_DRU)
        pcb = parse_kicad_pcb(evidence(dst))
        led = sorted(n.name for n in pcb.nets.values()
                     if n.name.startswith('/LED_'))
        ids = [nid for nid, n in pcb.nets.items() if n.name in led]

        def spy(pcb_data, config, *a, **k):
            seen['tc'] = dict(config.track_clearances or {})
            raise _Seen
        route.apply_single_ended_layer_swaps = spy
        try:
            route.batch_route(dst, os.path.join(td, 'o.kicad_pcb'), led,
                              enable_layer_switch=True)
        except _Seen:
            pass
        rules, _ = read_board_track_clearances(dst)
        nets = {nid: n.name for nid, n in pcb.nets.items() if n.name}
        want = effective_track_clearances(
            rules, net_class_memberships(dst, nets), nets.keys(), ids)
    finally:
        route.apply_single_ended_layer_swaps = saved
        shutil.rmtree(td, ignore_errors=True)
    got = seen.get('tc')
    if got is None:
        return ["track map: the swap pass was never reached (a broken "
                "fixture, not a verdict)"]
    if not ids or not want:
        return ["track map: the fixture priced nothing (a broken fixture)"]
    print(f"  route.py swap pass track map: {len(got)} net(s) priced, "
          f"want {len(want)} (routing {len(ids)} XTAL member(s))")
    if got != want:
        return [f"track map at the swap passes prices "
                f"{sorted(got)[:6]}..., not the map over the routed ids "
                f"{sorted(want)[:6]}..."]
    return []


if __name__ == '__main__':
    sys.exit(main())
