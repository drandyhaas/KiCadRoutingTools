"""#1145: route_diff raises a pair's coupling gap to a .kicad_dru rule that
binds the pair, as it raises it to the clearance (#441) and the class (#530).

P and N are two nets, so KiCad grades their spacing under a layer clearance
rule (#498) on the layer they meet on, and under a track rule (#735) whose
class takes in both. Before this the coupled run was built at the class gap,
and every coupled segment on a ruled layer was a violation: cap_chain under an
F.Cu rule of 0.3 shipped 28 F.Cu P/N violations with the #215 hybrid off
(kicad-cli: 19, `actual 0.2502 mm`), and 28 under a track rule with the hybrid
on, since the self-checks price no track rule. The hybrid off is what makes
the coupled run the copper that ships; with it on the layer case was only
rescued by running the coupled middle on In1.Cu.

The raise takes the widest rule over the layers the pair may route on (a
forbidden layer does not count) and never lowers the gap. It happens twice:
call-level before the --impedance solve, so the solved widths are those of
the built gap, and per pair, where a pair's own net-class gap (#435, the CLI
default) replaces the call's. The #318 neck now prices P against N at the
pair value, so a diagonal's nanometre rounding is necked under the layer rule
as it is under a class.

Rows:
  - kicad_dru.pair_gap_rule_floor: routed layers only, track rules
    pair-exact (an `other_only` rule exempts the partner);
  - the issue's reproduction (layer rule, hybrid off): graded clean at margin
    0, and the gap line names the rule;
  - a rule on a forbidden layer, and a relaxing rule, raise nothing;
  - the per-pair floor survives a net-class gap below the rule;
  - the per-pair floor keeps the clearance too (#441, found here): with no
    rules file a class gap below the clearance was built as is, and the
    #318 neck shaved P and N to different widths to clear it;
  - a track rule raises the gap; its `other_only` form does not;
  - the --impedance solve sees the raised gap.

    python3 tests/test_1145_pair_gap_rule.py
"""

import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import evidence  # noqa: E402

LAYER_RULE = ('(version 1)\n(rule "tight_f" (layer "F.Cu") '
              '(constraint clearance (min 0.3mm)))\n')


def _track_rule(other_only=False):
    cond = ("A.NetClass == 'HS' && A.Type == 'track' && B.Type == 'track'"
            + (" && B.NetClass != 'HS'" if other_only else ''))
    return ('(version 1)\n(rule "hs_tracks" (condition "' + cond + '") '
            '(constraint clearance (min 0.3mm)))\n')


def _pro(gap=0.25, hs=False):
    cls = {"clearance": 0.25, "track_width": 0.2, "diff_pair_gap": gap,
           "diff_pair_width": 0.2, "via_diameter": 0.6, "via_drill": 0.3}
    ns = {"classes": [dict(cls, name="Default")]}
    if hs:
        ns["classes"].append(dict(cls, name="HS"))
        ns["netclass_patterns"] = [{"netclass": "HS", "pattern": "DP*"}]
    return {"net_settings": ns}


def _route(dru, pro=None, hybrid=False, widths=None, **kw):
    """Route cap_chain's two pairs under `dru` (None: no rules file) and
    `pro`; returns (log, routed, failed, {margin: violations}), and fills
    `widths` with {net name: [segment widths]} when given. The #215 hybrid
    swap is off unless asked for, so the coupled run is what ships."""
    import diff_pair_loop
    import route_diff
    from check_drc import run_drc
    from kicad_parser import parse_kicad_pcb
    swap = diff_pair_loop._maybe_swap_to_hybrid
    if not hybrid:
        diff_pair_loop._maybe_swap_to_hybrid = lambda result, *a, **k: result
    try:
        with tempfile.TemporaryDirectory() as td:
            dst = os.path.join(td, 'cc.kicad_pcb')
            shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'),
                        dst)
            for stem in ('cc', 'o'):
                if dru is not None:
                    with open(os.path.join(td, stem + '.kicad_dru'), 'w',
                              encoding='utf-8') as fh:
                        fh.write(dru)
                if pro is not None:
                    with open(os.path.join(td, stem + '.kicad_pro'), 'w',
                              encoding='utf-8') as fh:
                        json.dump(pro, fh)
            pairs = sorted(n.name for n in parse_kicad_pcb(evidence(dst)).nets
                           .values() if n.name.startswith('DP'))
            out = os.path.join(td, 'o.kicad_pcb')
            log = io.StringIO()
            with contextlib.redirect_stdout(log):
                r = route_diff.batch_route_diff_pairs(
                    dst, out, pairs, enable_layer_switch=True, **kw)
            evidence(out)
            if widths is not None:
                pcb = parse_kicad_pcb(out)
                names = {n.net_id: n.name for n in pcb.nets.values()}
                for s in pcb.segments:
                    nm = names.get(s.net_id, '')
                    if nm.startswith('DP'):
                        widths.setdefault(nm, []).append(s.width)
            with contextlib.redirect_stdout(io.StringIO()):
                viols = {m: run_drc(out, clearance=0.25, quiet=True,
                                    print_summary=False, check_sizes=False,
                                    clearance_margin=m)
                         for m in (0.0, 0.05)}
    finally:
        diff_pair_loop._maybe_swap_to_hybrid = swap
    return log.getvalue(), r[0], r[1], viols


def _lines(log, needle):
    return [ln.strip() for ln in log.splitlines() if needle in ln]


def _kinds(viols):
    return sorted({(v['type'], v.get('layer')) for v in viols})


def test_pair_gap_rule_floor():
    from kicad_dru import TrackRule, pair_gap_rule_floor
    lmap = {'F.Cu': 0.3, 'In1.Cu': 0.2}
    g, why = pair_gap_rule_floor(lmap, ['F.Cu', 'B.Cu'])
    assert g == 0.3 and 'F.Cu' in why, (g, why)
    assert pair_gap_rule_floor(lmap, ['B.Cu']) == (0.0, None), \
        'a rule on a layer the pair does not route on binds nothing'
    assert pair_gap_rule_floor(lmap, ['In1.Cu', 'B.Cu'])[0] == 0.2
    hs = frozenset({'HS'})
    plain = [TrackRule('hs', 'HS', False, 0.4)]
    g, why = pair_gap_rule_floor(lmap, ['F.Cu'], plain, hs, hs)
    assert g == 0.4 and "'hs'" in why, (g, why)
    assert pair_gap_rule_floor({}, ['F.Cu'], plain, frozenset(),
                               frozenset()) == (0.0, None), \
        'a pair outside the class is not bound by its rule'
    other_only = [TrackRule('hs', 'HS', True, 0.4)]
    assert pair_gap_rule_floor({}, ['F.Cu'], other_only, hs, hs) \
        == (0.0, None), 'an other_only rule exempts the partner'
    g, why = pair_gap_rule_floor(lmap, ['F.Cu'],
                                 [TrackRule('hs', 'HS', False, 0.25)], hs, hs)
    assert g == 0.3 and 'F.Cu' in why, \
        f'a track rule below the layer rule leaves it: {(g, why)}'


def test_layer_rule_raises_the_coupled_gap():
    """The issue's reproduction: clean at margin 0, the gap line names the
    rule. On main the same run shipped 28 F.Cu P/N violations."""
    log, ok, bad, viols = _route(LAYER_RULE)
    assert (ok, bad) == (2, 0), f'fixture: both pairs must route {(ok, bad)}'
    raised = _lines(log, 'raising gap to 0.3mm')
    assert raised and 'F.Cu' in raised[0], _lines(log, 'raising gap')
    assert not viols[0.0], _kinds(viols[0.0])


def test_rule_off_the_routed_layers_raises_nothing():
    """A rule on a layer the pair may not route on (appended FORBIDDEN by
    the full-stack normalization), and a rule that RELAXES its layer, leave
    the gap alone."""
    log, *_ = _route(LAYER_RULE, layers=['B.Cu'])
    assert not _lines(log, '#1145'), _lines(log, '#1145')
    relax = LAYER_RULE.replace('0.3mm', '0.15mm')
    log, ok, bad, viols = _route(relax)
    assert not _lines(log, '#1145'), _lines(log, '#1145')
    assert _lines(log, 'raising gap to 0.25mm'), 'the #441 raise still runs'
    assert (ok, bad) == (2, 0) and not viols[0.0], _kinds(viols[0.0])


def test_per_pair_floor_survives_a_class_gap():
    """With --diff-pair-gap omitted (the CLI default) each pair takes its
    net class's gap, which replaces the call's raised one (#435); the pair
    is floored at the rule again there. 0.27 sits between the 0.25
    clearance and the 0.3 rule, so the #441 floor does not move it first."""
    log, ok, bad, viols = _route(LAYER_RULE, pro=_pro(gap=0.27),
                                 diff_pair_gap_from_class=True)
    per_pair = _lines(log, '#1145: DP')
    assert len(per_pair) == 2 and all('0.27 mm raised' in ln and 'F.Cu' in ln
                                      for ln in per_pair), per_pair
    assert not _lines(log, '#441: DP'), _lines(log, '#441: DP')
    assert (ok, bad) == (2, 0) and not viols[0.0], _kinds(viols[0.0])


def test_per_pair_floor_keeps_the_clearance():
    """#441 on the per-pair path: a net-class gap of 0.15 under a 0.25
    clearance, no rules file. The class gap replaced the call's floored one
    and the pair was built inside clearance; the #318 neck then cleared it by
    narrowing P and N, to different widths. Now the gap is floored and both
    members keep their width (a diagonal's 1 nm rounding may still neck one
    by a few hundred nm, hence the tolerance)."""
    from routing_defaults import TRACK_WIDTH
    widths = {}
    log, ok, bad, viols = _route(None, pro=_pro(gap=0.15),
                                 diff_pair_gap_from_class=True, widths=widths)
    floored = _lines(log, '#441: DP')
    assert len(floored) == 2 and all('0.15 mm raised to clearance 0.25'
                                     in ln for ln in floored), floored
    assert (ok, bad) == (2, 0) and not viols[0.0], _kinds(viols[0.0])
    assert sorted(widths) == ['DPA_N', 'DPA_P', 'DPB_N', 'DPB_P'], \
        f'fixture: all four members must carry copper {sorted(widths)}'
    necked = {nm: min(ws) for nm, ws in widths.items()
              if min(ws) < TRACK_WIDTH - 1e-3}
    assert not necked, f'members necked below {TRACK_WIDTH}: {necked}'


def test_track_rule_raises_the_coupled_gap():
    """A track rule on the pair's class binds P against N (on main: 28
    violations with the hybrid ON -- the self-checks price no track rule).
    Graded at check_drc's default margin: the coupled run sits exactly at
    the rule, and a 45-degree run's 1 nm coordinate rounding can leave it
    0.1 nm under, which the #318 neck does not shave for a track rule (it
    prices layer pairs; the router's track map is per obstacle net and would
    over-neck an `other_only` partner). kicad-cli grades that board clean."""
    log, ok, bad, viols = _route(_track_rule(), pro=_pro(hs=True),
                                 hybrid=True)
    raised = _lines(log, 'raising gap to 0.3mm')
    assert raised and "'hs_tracks'" in raised[0], _lines(log, 'raising gap')
    assert (ok, bad) == (2, 0) and not viols[0.05], _kinds(viols[0.05])
    log, ok, bad, viols = _route(_track_rule(other_only=True),
                                 pro=_pro(hs=True))
    assert not _lines(log, '#1145'), \
        f'an other_only rule exempts the partner: {_lines(log, "#1145")}'
    assert (ok, bad) == (2, 0) and not viols[0.0], _kinds(viols[0.0])


class _Solved(Exception):
    pass


def test_impedance_solves_at_the_raised_gap():
    """The --impedance width solve runs after the call-level raise, so it is
    fed the gap the pairs are built at (as it is for #441). cap_chain has no
    stackup, so one is given and the solve intercepted."""
    import route_diff
    from kicad_parser import StackupLayer, parse_kicad_pcb
    seen = []

    def solve(*a, **k):
        seen.append(k.get('spacing'))
        raise _Solved()
    real = route_diff.calculate_layer_widths_for_impedance
    route_diff.calculate_layer_widths_for_impedance = solve
    try:
        with tempfile.TemporaryDirectory() as td:
            dst = os.path.join(td, 'cc.kicad_pcb')
            shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'),
                        dst)
            with open(os.path.join(td, 'cc.kicad_dru'), 'w',
                      encoding='utf-8') as fh:
                fh.write(LAYER_RULE)
            pcb = parse_kicad_pcb(evidence(dst))
            pcb.board_info.stackup = [
                StackupLayer('F.Cu', 'copper', 0.035),
                StackupLayer('dielectric 1', 'core', 1.5, epsilon_r=4.5),
                StackupLayer('B.Cu', 'copper', 0.035)]
            pairs = sorted(n.name for n in pcb.nets.values()
                           if n.name.startswith('DP'))
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    route_diff.batch_route_diff_pairs(
                        dst, os.path.join(td, 'o.kicad_pcb'), pairs,
                        impedance=90, pcb_data=pcb)
            except _Solved:
                pass
    finally:
        route_diff.calculate_layer_widths_for_impedance = real
    assert seen == [0.3], f'the solve must see the raised gap, saw {seen}'


TESTS = [test_pair_gap_rule_floor, test_layer_rule_raises_the_coupled_gap,
         test_rule_off_the_routed_layers_raises_nothing,
         test_per_pair_floor_survives_a_class_gap,
         test_per_pair_floor_keeps_the_clearance,
         test_track_rule_raises_the_coupled_gap,
         test_impedance_solves_at_the_raised_gap]


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
