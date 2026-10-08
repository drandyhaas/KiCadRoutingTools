"""#1134: a diff pair's P/N self-checks grade P against N at the pair's own
class value, as KiCad does.

P and N are two nets, so KiCad grades their spacing like any other pair:
max(clearance, class P, class N), then the .kicad_dru layer rule. route_diff
already raises the coupling gap to that class value (#530), but the checks
that decide whether a coupled route pinches its own pair priced the pinch at
the flat Default clearance, so on a pair whose class is wider than Default
they passed a P/N approach KiCad flags.

The self-checks price the full pair value, a .kicad_dru layer rule
included. Since #1145 route_diff raises the coupling gap to that rule too
(tests/test_1145_pair_gap_rule.py), so on a ruled layer the counts here are
the pinches where the connectors diverge. The rows below hold the checks to
the pair value directly, with configs whose gap sits BELOW a rule -- the
state #1145 removed from route_diff, kept here because the checks must not
lean on it.

Rows, each at the flat value (no class: the verdict is unchanged) and under a
0.35 class on both nets:
  - diff_pair_loop._count_pn_overlaps, on a board check_drc grades, with
    check_drc as the oracle;
  - diff_pair_multipoint._pn_self_overlaps;
  - diff_pair_routing._make_offset_connector_check (the launch leg grazing
    the partner's escape via);
  - diff_pair_routing._collapse_leg_attach_join's intra-pair floor;
  - diff_pair_routing._pn_overlap_count (nested in the hybrid builder, so
    held statically: it must price through pair_clearance).

    python3 tests/test_1134_pn_pair_floor.py
"""

import ast
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', os.path.join('tests', 'oracle'), 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import evidence  # noqa: E402

CLASS = 0.35
BASE = 0.2


def _cfg(cls=None, **kw):
    from routing_config import GridRouteConfig
    c = GridRouteConfig(clearance=BASE, track_width=0.2, via_size=0.6,
                        via_drill=0.3, layers=['F.Cu', 'B.Cu'], **kw)
    if cls:
        c.set_net_clearances({1: cls, 2: cls}, routed_net_ids=[1, 2])
    # the gap route_diff gives the pair for its clearance (#441) and class
    # (#530); a .kicad_dru rule (#1145) is left out, so a rule row prices a
    # gap below it
    c.diff_pair_gap = max(c.diff_pair_gap, c.clearance, cls or 0.0)
    return c


def test_count_pn_overlaps_matches_check_drc():
    """P and N 0.25 mm apart edge to edge, both in a 0.35 class: check_drc
    flags it, so the self-check must count it."""
    from constraint_agreement import write_board
    from kicad_parser import parse_kicad_pcb
    from diff_pair_loop import _count_pn_overlaps
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'pair.kicad_pcb')
        write_board(b, segments=[(5, 10, 15, 10, 0.2, 'F.Cu', 1),
                                 (5, 10.45, 15, 10.45, 0.2, 'F.Cu', 2)],
                    classes=[{'name': 'PAIR', 'clearance': CLASS,
                              'priority': 0}],
                    patterns=[('A', 'PAIR'), ('B', 'PAIR')])
        evidence(b)
        pcb = parse_kicad_pcb(b)
        p = [s for s in pcb.segments if s.net_id == 1]
        n = [s for s in pcb.segments if s.net_id == 2]
        js = os.path.join(td, 'drc.json')
        subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_drc.py'), b,
                        '--clearance', str(BASE), '-q', '--json', js],
                       capture_output=True)
        d = json.load(open(evidence(js), encoding='utf-8'))
    assert d.get('violations') == 1, ('fixture: check_drc must flag the pair',
                                      d.get('by_type'))
    assert _count_pn_overlaps(p, n, _cfg()) == 0, \
        'no class: 0.25 mm clears the 0.2 Default, nothing to count'
    got = _count_pn_overlaps(p, n, _cfg(CLASS))
    assert got == 1, f'0.35 class: check_drc flags the pair, counted {got}'


def test_pn_self_overlaps():
    from synth import make_seg
    from diff_pair_multipoint import _pn_self_overlaps
    segs = [make_seg(5, 10, 15, 10, net_id=1),
            make_seg(5, 10.45, 15, 10.45, net_id=2)]
    assert not _pn_self_overlaps(segs, 1, 2, _cfg())
    assert _pn_self_overlaps(segs, 1, 2, _cfg(CLASS)), \
        'a 0.25 mm P/N gap under a 0.35 class is a self-graze'
    # clears the class: not a graze either way
    wide = [make_seg(5, 10, 15, 10, net_id=1),
            make_seg(5, 10.6, 15, 10.6, net_id=2)]
    assert not _pn_self_overlaps(wide, 1, 2, _cfg(CLASS))


def test_layer_rule_above_the_gap_counts():
    """A coupled run AT a 0.2 gap on F.Cu, under an F.Cu rule of 0.3: KiCad
    flags it, so the self-checks count it (that is what sends the pair to the
    hybrid). Off the ruled layer it does not count, and a RELAXING rule
    replaces the value as check_drc does."""
    from synth import make_seg
    from diff_pair_loop import _count_pn_overlaps
    from diff_pair_multipoint import _pn_self_overlaps
    ruled = _cfg(layer_clearances={'F.Cu': 0.3})
    at_gap = [make_seg(5, 10, 15, 10, net_id=1),
              make_seg(5, 10.4, 15, 10.4, net_id=2)]        # edge gap 0.2
    p = [s for s in at_gap if s.net_id == 1]
    n = [s for s in at_gap if s.net_id == 2]
    assert _count_pn_overlaps(p, n, ruled) == 1
    assert _pn_self_overlaps(at_gap, 1, 2, ruled)
    off = [make_seg(5, 10, 15, 10, net_id=1, layer='B.Cu'),
           make_seg(5, 10.4, 15, 10.4, net_id=2, layer='B.Cu')]
    assert not _pn_self_overlaps(off, 1, 2, ruled), \
        'the F.Cu rule does not bind a B.Cu run'
    pinch = [make_seg(5, 10, 15, 10, net_id=1),
             make_seg(5, 10.35, 15, 10.35, net_id=2)]       # edge gap 0.15
    relaxed = _cfg(layer_clearances={'F.Cu': 0.14})
    assert not _pn_self_overlaps(pinch, 1, 2, relaxed)


def test_cap_chain_routes_off_a_ruled_layer():
    """End to end: cap_chain's pairs with an F.Cu rule of 0.3 above their
    class gap, the #215 hybrid on, graded clean (the flat floor shipped 28
    F.Cu P/N violations here). Since #1145 the coupled run is built at the
    rule; test_1145 grades it with the hybrid off."""
    import contextlib
    import io
    import shutil
    import route_diff
    from check_drc import run_drc
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        dst = os.path.join(td, 'cc.kicad_pcb')
        shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'),
                    dst)
        rules = ('(version 1)' + chr(10) + '(rule "tight_f" (layer "F.Cu") '
                 '(constraint clearance (min 0.3mm)))' + chr(10))
        for stem in ('cc', 'o'):
            with open(os.path.join(td, stem + '.kicad_dru'), 'w',
                      encoding='utf-8') as fh:
                fh.write(rules)
        pairs = sorted(nt.name for nt in parse_kicad_pcb(evidence(dst)).nets
                       .values() if nt.name.startswith('DP'))
        out = os.path.join(td, 'o.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            r = route_diff.batch_route_diff_pairs(dst, out, pairs,
                                                  enable_layer_switch=True)
        assert r[0] == 2 and r[1] == 0, f'fixture: both pairs must route {r}'
        viols = run_drc(evidence(out), clearance=0.25, quiet=True,
                        print_summary=False, check_sizes=False,
                        clearance_margin=0.0)
    assert not viols, [(v['type'], v.get('layer')) for v in viols][:6]


def test_offset_connector_partner_via():
    """The P leg runs from (0, 0) to (2, 0); the partner's escape via (0.6 mm)
    sits 0.62 mm off the leg's centreline. Need: flat 0.2 + 0.1 + 0.3 = 0.6
    (clears); class 0.35 + 0.1 + 0.3 = 0.75 (grazes)."""
    from diff_pair_routing import _make_offset_connector_check
    p_term, n_term = (0.0, 0.0), (0.0, 3.0)
    vias = [(1.0, 0.62, 0.6, 2)]          # N's via, far from both terminals
    # launch at (2, 1.5) heading +x, spacing 1.5: off_a = (2, 3.0) and
    # off_b = (2, 0.0); P takes off_b (the leg (0,0)->(2,0)), N off_a.
    args = (p_term, n_term, vias, 1.5)
    flat = _make_offset_connector_check(*args, _cfg(), 1, 2)
    cls = _make_offset_connector_check(*args, _cfg(CLASS), 1, 2)
    assert flat(2.0, 1.5, 1.0, 0.0), 'flat: 0.62 mm clears the 0.6 need'
    assert not cls(2.0, 1.5, 1.0, 0.0), \
        'class 0.35: the leg grazes the partner via (need 0.75)'
    # a caller that passes no net ids keeps the flat value
    old = _make_offset_connector_check(*args, _cfg(CLASS))
    assert old(2.0, 1.5, 1.0, 0.0)
    # the launch layer, when the caller gives it, prices the pair there: a
    # rule on ANOTHER layer does not refuse the F.Cu launch (the stack bound
    # would), and the rule on the launch layer does
    other = _make_offset_connector_check(
        *args, _cfg(layer_clearances={'B.Cu': 0.5}), 1, 2)
    assert other(2.0, 1.5, 1.0, 0.0, layer='F.Cu')
    assert not other(2.0, 1.5, 1.0, 0.0), 'no layer given: the stack bound'
    assert not other(2.0, 1.5, 1.0, 0.0, layer='B.Cu')
    # a via of the leg's OWN net keeps the flat value
    own = _make_offset_connector_check(
        p_term, n_term, [(1.0, 0.62, 0.6, 1)], 1.5, _cfg(CLASS), 1, 2)
    assert own(2.0, 1.5, 1.0, 0.0)


def test_collapse_join_intra_floor():
    """A hybrid leg's grid corner sits 0.30 mm (edge) from the partner; the
    collapsed corner would sit 0.42 mm away. Flat (0.2): the corner already
    clears, nothing to fix. Under a 0.35 class with the gap route_diff
    raised to it, the corner grazes and the collapse fixes it."""
    from synth import make_seg
    from diff_pair_routing import _collapse_leg_attach_join

    def leg():
        return [make_seg(0, 1.0, 5, 0.5, net_id=1),       # body -> grid corner
                make_seg(5, 0.5, 5.04, 0.62, net_id=1)]   # short join
    partner = [make_seg(0, 0, 10, 0, net_id=2)]
    flat = _cfg()
    flat.diff_pair_gap = 0.2
    cls = _cfg(CLASS)
    cls.diff_pair_gap = CLASS
    got = _collapse_leg_attach_join(leg(), (5.04, 0.62), flat, None, 1,
                                    partner)
    assert len(got) == 2, 'flat: the corner clears 0.2, the join stays'
    got = _collapse_leg_attach_join(leg(), (5.04, 0.62), cls, None, 1,
                                    partner)
    assert len(got) == 1 and abs(got[0].end_y - 0.62) < 1e-9, \
        'class 0.35: the corner grazes the partner, the join collapses'


def test_hybrid_pn_overlap_count_prices_the_pair():
    """`_pn_overlap_count` is nested inside the hybrid builder, so it is held
    statically: its threshold must come from pair_clearance, not the flat
    config.clearance."""
    path = os.path.join(ROOT, 'py_router', 'diff_pair_routing.py')
    tree = ast.parse(open(evidence(path), encoding='utf-8').read())
    fns = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
           and n.name == '_pn_overlap_count']
    assert len(fns) == 1, f'expected one _pn_overlap_count, found {len(fns)}'
    src = ast.unparse(fns[0])
    assert 'pair_clearance' in src, src
    assert 'config.clearance' not in src, src


TESTS = [test_count_pn_overlaps_matches_check_drc, test_pn_self_overlaps,
         test_layer_rule_above_the_gap_counts,
         test_cap_chain_routes_off_a_ruled_layer,
         test_offset_connector_partner_via, test_collapse_join_intra_floor,
         test_hybrid_pn_overlap_count_prices_the_pair]


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
