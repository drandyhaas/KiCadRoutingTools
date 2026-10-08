#!/usr/bin/env python3
"""#1136: `GridRouteConfig.pair_clearance` / `pad_pair_clearance` are the value
check_drc grades a pair at -- with check_drc itself as the oracle.

The router's admit/refuse verdicts (may this stub sit here, may this via go
there) price a pair of nets through these two methods instead of the flat
`config.clearance`. They are only worth anything if they are the
number the grader enforces, so every row here:

  1. writes a tiny board (tests/oracle/constraint_agreement.write_board: nets
     A/B/C, a .kicad_pro with the row's classes, an optional .kicad_dru);
  2. builds the ROUTER's view of it with the production resolvers
     (`list_nets.net_clearance_map_by_id` + `set_net_clearances`,
     `kicad_dru.install_layer_clearances`, `install_track_clearances`);
  3. asks the helper for the pair's clearance `req`;
  4. writes the probe pair at an edge gap of `req - 0.01` and of `req + 0.01`
     and grades both with `check_drc.run_drc` (margin 0): the first must be
     flagged with the pair's own violation type and the second must be clean.

Kinds: track-track, track-via, via-via, pad-track, pad-via, pad-pad. Rule
shapes: none (inert), a class on either net or both, a .kicad_dru layer rule
that tightens and one that relaxes below a class, a track-scoped rule, a pad
override below its class (it REPLACES the class) and one floored at the
board's min_clearance, and a through-hole pad under a rule on one of its
layers only.

Every value sits at or above the 2-layer fab floor (0.127): the router pins
a layer rule up to it and the grader does not, a difference that only ever
makes the router stricter.

    python3 tests/test_1136_pair_clearance_parity.py [row-substring ...]
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', os.path.join('tests', 'oracle'), 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from constraint_agreement import write_board, NETS, W  # noqa: E402
import run_utils                                       # noqa: E402

EPS = 0.01
BASE = 0.2

#: check_drc violation types per pair kind (the ones a clearance breach of
#: THIS pair reports; anything else on the probe board is a fixture fault).
TYPES = {
    'track': {'segment-segment', 'segment-crossing',
              'segment-segment-track-rule'},
    'track_net0': {'segment-segment', 'segment-crossing',
                   'segment-segment-track-rule'},
    'track_via': {'via-segment'},
    'via_via': {'via-via'},
    'pad_track': {'pad-segment'},
    'pad_via': {'pad-via'},
    'pad_pad': {'pad-pad'},
}

HV = {'name': 'HV', 'clearance': 0.35, 'priority': 0}
MV = {'name': 'MV', 'clearance': 0.3, 'priority': 1}
WIDE = {'name': 'W', 'clearance': 0.2, 'priority': 0}

#: name -> write_board kwargs (minus geometry) for one rule shape
SHAPES = {
    'inert': {},
    'class_on_a': dict(classes=[HV], patterns=[('A', 'HV')]),
    'class_on_b': dict(classes=[HV], patterns=[('B', 'HV')]),
    'class_on_both': dict(classes=[HV, MV], patterns=[('A', 'HV'), ('B', 'MV')]),
    'dru_tightens_f': dict(
        dru='(rule "t" (layer "F.Cu") (constraint clearance (min 0.3mm)))'),
    'dru_relaxes_f_below_class': dict(
        classes=[HV], patterns=[('A', 'HV')], rules={'min_clearance': 0.05},
        dru='(rule "r" (layer "F.Cu") (constraint clearance (min 0.15mm)))'),
    'class_on_a_board_min_020': dict(
        classes=[HV], patterns=[('A', 'HV')], rules={'min_clearance': 0.2}),
    'track_rule': dict(
        classes=[WIDE], patterns=[('A', 'W')],
        dru=('(rule "w" (constraint clearance (min 0.5mm)) (condition '
             '"A.Type == \'track\' && B.Type == \'track\' && '
             'A.NetClass == \'W\'"))')),
    # member-vs-NON-member only (#735 `other_only`): the router's map is
    # per obstacle net and depends on which side the routed set is on
    'track_rule_other_only': dict(
        classes=[WIDE], patterns=[('A', 'W')],
        dru=('(rule "w" (constraint clearance (min 0.5mm)) (condition '
             '"A.Type == \'track\' && B.Type == \'track\' && '
             'A.NetClass == \'W\' && B.NetClass != \'W\'"))')),
}


def _probe(kind, gap, pad_clr=None, layer='F.Cu', th=False, other_clr=None):
    """write_board geometry kwargs for one probe pair at edge gap `gap`.
    Net A (1) is the first item, net B (2) the second."""
    if kind == 'track':
        y2 = 10 + W + gap
        return dict(segments=[(5, 10, 15, 10, W, layer, 1),
                              (5, y2, 15, y2, W, layer, 2)])
    if kind == 'track_net0':
        # nameless copper: in no class, absent from the router's track map
        y2 = 10 + W + gap
        return dict(segments=[(5, 10, 15, 10, W, layer, 1),
                              (5, y2, 15, y2, W, layer, 0)])
    if kind == 'track_via':
        # via (B) at (10, 10), size 0.6; a track (A) below it
        y = 10 + 0.3 + gap + W / 2
        return dict(vias=[(10, 10, 0.6, 0.3, 2)],
                    segments=[(5, y, 15, y, W, layer, 1)])
    if kind == 'via_via':
        return dict(vias=[(10, 10, 0.6, 0.3, 1),
                          (10, 10 + 0.6 + gap, 0.6, 0.3, 2)])
    pad = {'ref': 'U1', 'x': 10, 'y': 10, 'net_id': 1, 'net_name': NETS[1],
           'pad_clearance': pad_clr, 'size': 1.0}
    if kind == 'pad_track':
        y = 10 + 0.5 + gap + W / 2
        return dict(footprints=[] if th else [pad],
                    segments=[(5, y, 15, y, W, layer, 2)])
    if kind == 'pad_via':
        return dict(footprints=[] if th else [pad],
                    vias=[(10, 10 + 0.5 + gap + 0.3, 0.6, 0.3, 2)])
    if kind == 'pad_pad':
        other = {'ref': 'U2', 'x': 10, 'y': 10 + 1.0 + gap, 'net_id': 2,
                 'net_name': NETS[2], 'size': 1.0,
                 'pad_clearance': other_clr}
        return dict(footprints=[pad, other])
    raise AssertionError(kind)


_TH_FP = '''
  (footprint "Probe:TH" (layer "F.Cu") (at 10 10)
    (property "Reference" "U1" (at 0 -1.5 0) (layer "F.SilkS") (hide yes)
      (effects (font (size 1 1) (thickness 0.15))))
    (property "Value" "P" (at 0 1.5 0) (layer "F.Fab") (hide yes)
      (effects (font (size 1 1) (thickness 0.15))))
    (attr through_hole)
    (pad "1" thru_hole rect (at 0 0) (size 1 1) (drill 0.4) (layers "*.Cu") (net 1 "A"))
  )'''


def _write(board, shape, kind, gap, pad_clr=None, layer='F.Cu', th=False,
           other_clr=None, **_router_only):
    kw = dict(SHAPES[shape])
    kw.update(_probe(kind, gap, pad_clr, layer, th, other_clr))
    write_board(board, **kw)
    if th:
        # A through-hole `*.Cu` pad: write_board's footprints are SMD only.
        with open(board, encoding='utf-8') as fh:
            text = fh.read()
        cut = text.rstrip().rfind(')')
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(text[:cut] + _TH_FP + '\n)\n')


def _router_config(board, routed=(1,), cfg_layers=('F.Cu', 'B.Cu')):
    """The router's view of `board`, built by the production resolvers, for a
    call routing `routed` on `cfg_layers`."""
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from list_nets import net_clearance_map_by_id
    from kicad_dru import install_layer_clearances, install_track_clearances
    pcb = parse_kicad_pcb(board)
    cfg = GridRouteConfig(clearance=BASE, layers=list(cfg_layers))
    names = {nid: n.name for nid, n in pcb.nets.items() if n.name}
    cfg.set_net_clearances(net_clearance_map_by_id(board, names),
                           routed_net_ids=list(routed))
    install_layer_clearances(cfg, None, board, pcb)
    install_track_clearances(cfg, None, board, pcb,
                             routed_net_ids=list(routed))
    return cfg, pcb


def _pad(pcb, ref='U1'):
    return pcb.footprints[ref].pads[0]


def _required(kind, shape, td, pad_clr=None, layer='F.Cu', th=False,
              other_clr=None, routed=(1,), cfg_layers=('F.Cu', 'B.Cu'),
              no_cu=False):
    """The helper's answer for the row's pair, read off a probe board at a
    nominal gap (the resolvers read rules, not geometry). `no_cu` is a row
    label only: the helper always resolves a pad's layers over the board
    copper list `install_layer_clearances` recorded."""
    board = os.path.join(td, 'resolve.kicad_pcb')
    _write(board, shape, kind, 1.0, pad_clr, layer, th, other_clr)
    cfg, pcb = _router_config(board, routed, cfg_layers)
    if kind == 'track':
        return cfg.pair_clearance(1, 2, layer, kind='track')
    if kind == 'track_net0':
        return cfg.pair_clearance(1, 0, layer, kind='track')
    if kind == 'track_via':
        return cfg.pair_clearance(1, 2, layer)
    if kind == 'via_via':
        return cfg.pair_clearance(1, 2, kind='stack')
    pad = _pad(pcb)
    if kind == 'pad_track':
        return cfg.pad_pair_clearance(pad, 2, layer=layer)
    if kind == 'pad_via':
        return cfg.pad_pair_clearance(pad, 2)
    if kind == 'pad_pad':
        return cfg.pad_pair_clearance(pad, 2, other_pad=_pad(pcb, 'U2'))
    raise AssertionError(kind)


def _grade(board):
    from check_drc import run_drc
    from list_nets import net_clearance_map, read_design_rules
    # check_drc's own main() builds its class map this way.
    rules = read_design_rules(board)
    ncl = (net_clearance_map(board, list(NETS.values()), rules=rules) or None
           if rules.get('classes') else None)
    viols = run_drc(board, clearance=BASE, net_clearances=ncl,
                    clearance_margin=0.0, quiet=True, check_sizes=False,
                    print_summary=False)
    return [str(v.get('type')) for v in viols]


#: (row name, kind, shape, expected value, extra kwargs). The expected value
#: is check_drc's rule written out by hand, so a row reads as the claim.
ROWS = [
    ('track_inert', 'track', 'inert', 0.2, {}),
    ('track_class_on_a', 'track', 'class_on_a', 0.35, {}),
    ('track_class_on_b', 'track', 'class_on_b', 0.35, {}),
    ('track_class_on_both', 'track', 'class_on_both', 0.35, {}),
    ('track_dru_tightens', 'track', 'dru_tightens_f', 0.3, {}),
    ('track_dru_off_layer', 'track', 'dru_tightens_f', 0.2,
     {'layer': 'B.Cu'}),
    ('track_dru_relaxes_below_class', 'track', 'dru_relaxes_f_below_class',
     0.15, {}),
    ('track_track_rule', 'track', 'track_rule', 0.5, {}),
    ('track_rule_other_only_routed_member', 'track', 'track_rule_other_only',
     0.5, {'routed': (1,)}),
    ('track_rule_other_only_routed_non_member', 'track',
     'track_rule_other_only', 0.5, {'routed': (2,)}),
    ('track_rule_other_only_nameless_copper', 'track_net0',
     'track_rule_other_only', 0.5, {'routed': (1,)}),
    ('track_via_inert', 'track_via', 'inert', 0.2, {}),
    ('track_via_class_on_b', 'track_via', 'class_on_b', 0.35, {}),
    ('track_via_dru_relaxes', 'track_via', 'dru_relaxes_f_below_class', 0.15,
     {}),
    ('track_via_track_rule_does_not_bind', 'track_via', 'track_rule', 0.2, {}),
    ('via_via_inert', 'via_via', 'inert', 0.2, {}),
    ('via_via_class_on_both', 'via_via', 'class_on_both', 0.35, {}),
    ('via_via_dru_tightens_stack', 'via_via', 'dru_tightens_f', 0.3, {}),
    ('via_via_dru_relax_keeps_class', 'via_via', 'dru_relaxes_f_below_class',
     0.35, {}),
    ('pad_track_inert', 'pad_track', 'inert', 0.2, {}),
    ('pad_track_class_on_a', 'pad_track', 'class_on_a', 0.35, {}),
    ('pad_track_override_replaces_class', 'pad_track', 'class_on_a', 0.15,
     {'pad_clr': 0.15}),
    ('pad_track_override_floored_at_board_min', 'pad_track',
     'class_on_a_board_min_020', 0.2, {'pad_clr': 0.15}),
    ('pad_track_dru_relaxes', 'pad_track', 'dru_relaxes_f_below_class', 0.15,
     {}),
    ('pad_via_class_on_b', 'pad_via', 'class_on_b', 0.35, {}),
    ('pad_via_smd_dru_tightens', 'pad_via', 'dru_tightens_f', 0.3, {}),
    ('pad_pad_class_on_both', 'pad_pad', 'class_on_both', 0.35, {}),
    ('pad_pad_override_replaces', 'pad_pad', 'class_on_both', 0.15,
     {'pad_clr': 0.15}),
    ('pad_pad_override_on_the_other_pad', 'pad_pad', 'class_on_both', 0.15,
     {'other_clr': 0.15}),
    ('th_pad_via_partial_rule', 'pad_via', 'dru_tightens_f', 0.3,
     {'th': True}),
    ('th_pad_track_unruled_layer', 'pad_track', 'dru_tightens_f', 0.2,
     {'th': True, 'layer': 'B.Cu'}),
    # routing F.Cu only: the TH pad's layers are the BOARD's {F, B} (F ruled 0.15, B not -> the 0.35 class stands), not the
    # routed {F} (all ruled -> 0.15)
    ('th_pad_via_board_copper_not_routed_layers', 'pad_via',
     'dru_relaxes_f_below_class', 0.35,
     {'th': True, 'cfg_layers': ('F.Cu',), 'no_cu': True}),
]


def run_row(name, kind, shape, want, kw):
    with tempfile.TemporaryDirectory() as td:
        req = _required(kind, shape, td, **kw)
        assert abs(req - want) < 1e-9, (name, 'helper', req, 'expected', want)
        board = os.path.join(td, 'probe.kicad_pcb')
        got = {}
        for label, gap in (('under', req - EPS), ('over', req + EPS)):
            _write(board, shape, kind, round(gap, 6), **kw)
            run_utils.evidence(board)
            got[label] = _grade(board)
        hit = [t for t in got['under'] if t in TYPES[kind]]
        assert hit, (name, f'gap {req - EPS:.3f} not flagged', got['under'])
        assert not got['over'], (name, f'gap {req + EPS:.3f} flagged',
                                 got['over'])
    return req


def test_inert_returns_the_floor_itself():
    """No class, no rule: the helper hands back the very float it was given
    (no arithmetic), so a site that swaps its flat term for the call is
    byte-identical on such a board."""
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(clearance=0.2)
    for kind in ('layer', 'track', 'stack'):
        assert cfg.pair_clearance(1, 2, 'F.Cu', kind=kind) is cfg.clearance
    b = 0.1234567
    assert cfg.pair_clearance(1, 2, 'F.Cu', kind='track', base=b) is b
    assert cfg.max_pair_clearance() is cfg.clearance
    try:
        cfg.pair_clearance(1, 2, kind='bogus')
    except ValueError:
        pass
    else:
        raise AssertionError('an unknown kind must raise')


def test_the_verdict_is_not_the_stamp():
    """`obstacle_clearance` is floored at the widest class ROUTED in the call;
    a two-net verdict is not. With a 0.4 class routed, a Default pair is
    still 0.2 (the flat_hierarchy case)."""
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(clearance=0.2)
    cfg.set_net_clearances({1: 0.4}, routed_net_ids=[1, 2, 3])
    assert cfg.obstacle_clearance(3) == 0.4
    assert cfg.pair_clearance(2, 3) == 0.2
    assert cfg.pair_clearance(1, 3) == 0.4
    assert cfg.max_pair_clearance() == 0.4


def test_name_keyed_map():
    from types import SimpleNamespace as NS
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(clearance=0.2)
    cfg.set_net_clearances({1: 0.4, 7: 0.3}, routed_net_ids=[1])
    nets = {1: NS(name='/HV'), 2: NS(name='/B')}
    assert cfg.net_clearances_by_name(nets) == {'/HV': 0.4}, \
        cfg.net_clearances_by_name(nets)


def test_every_row_agrees_with_check_drc():
    only = [a for a in sys.argv[1:] if not a.startswith('-')]
    n = 0
    for name, kind, shape, want, kw in ROWS:
        if only and not any(o in name for o in only):
            continue
        req = run_row(name, kind, shape, want, kw)
        print(f"  {name}: {req:g} -- flagged at -{EPS}, clean at +{EPS}")
        n += 1
    assert n, 'no row ran'
    print(f"  PASS: {n} row(s) bracket check_drc's own threshold")


TESTS = [test_inert_returns_the_floor_itself, test_the_verdict_is_not_the_stamp,
         test_name_keyed_map, test_every_row_agrees_with_check_drc]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
