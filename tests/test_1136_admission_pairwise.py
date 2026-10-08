#!/usr/bin/env python3
"""#1136: copper admit/refuse checks against another net's copper price the
pair at check_drc's clearance, not the flat `config.clearance`.

#980's second sweep found these checks pricing another net's track, via or
pad at one flat scalar, where KiCad requires max(clearance, classA, classB)
and then the .kicad_dru layer rule: the stub-swap validators, the #666
net-rescue escape / via-site / cap checks, the bare-pad swap via fit, the
multipoint fan fit, both meander amplitude searches, the plane tap's
restore test, the #339 unblock-via refit, the terminal exact-endpoint merge
and the foreign half of the collapsed leg join. Each now asks
`config.pair_clearance` / `pad_pair_clearance_before_override` (or
`single_ended_routing._pair_floor`, which hands the foreign-distance
helpers the base and class map they fold the same value from);
tests/test_1136_pair_clearance_parity.py holds the helper to check_drc.

Most cases are three arms on ONE geometry, with a 0.35 class over a 0.2
floor:

* flat   -- nothing declared: the verdict the old code gave, kept;
* other  -- a class on a THIRD routed net only (so the stamp floor
            `obstacle_clearance` is 0.35, but this pair's own value is 0.2):
            the flat verdict, kept -- the site prices the PAIR, not the floor;
* wide   -- the class on the other net (or the partner): the verdict flips.

The wide arm of every case fails on the unfixed code. Plus: the kind and
layer each site prices with (only visible under a .kicad_dru rule), the
prefilter windows reaching a class wider than their fixed size, the swap's
deliberate grading margin kept off the pair value, a same-net item keeping
the flat value, and `stub_clear_of_foreign_pads` sizing each stub at its own
width (the separate bug in the same issue, which is NOT inert: it moves
copper on any board whose stubs are not at the default width).

    python3 tests/test_1136_admission_pairwise.py [test-substring ...]
"""
import math
import os
import sys
import traceback
from types import SimpleNamespace as NS

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import BoardInfo, Net                   # noqa: E402
from routing_config import GridRouteConfig                # noqa: E402
from synth import make_pad, make_seg, make_via, make_pcb  # noqa: E402

OWN, P2, FOREIGN, THIRD = 1, 2, 99, 77
CAP_A, CAP_B = 5, 6
WIDE = 0.35
BI = BoardInfo(layers={}, board_bounds=None, copper_layers=['F.Cu', 'B.Cu'])

#: (test, case label, passed)
RESULTS = []
_CURRENT = ['']


def check(label, got, want):
    RESULTS.append((_CURRENT[0], label, got == want))
    mark = 'PASS' if got == want else 'FAIL'
    print(f"  {mark}: {label}: got {got!r}, want {want!r}")


def cfg(classes=None, routed=(OWN,), **kw):
    c = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                        via_drill=0.3, layers=['F.Cu', 'B.Cu'], grid_step=0.05)
    c.hole_to_hole_clearance = 0.2
    for k, v in kw.items():
        setattr(c, k, v)
    if classes:
        c.set_net_clearances(dict(classes), routed_net_ids=list(routed))
    return c


def arms(net=FOREIGN):
    """(name, config, the verdict's expected sense) for the three arms."""
    return (('flat', cfg(), 'flat'),
            ('other', cfg({THIRD: WIDE}, routed=(OWN, THIRD)), 'flat'),
            ('wide', cfg({net: WIDE}), 'wide'))


def three_ways(label, verdict, flat_admits=True, net=FOREIGN):
    """`verdict(config)` is True when the copper is ADMITTED."""
    for name, c, sense in arms(net):
        want = flat_admits if sense == 'flat' else (not flat_admits)
        check(f"{label} [{name}]", verdict(c), want)


def pcb(segs=(), vias=(), pads=(), footprints=None):
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    fps = footprints if footprints is not None else (
        {'U9': NS(reference='U9', pads=list(pads), locked=False)}
        if pads else {})
    return make_pcb(segments=list(segs), vias=list(vias), pads_by_net=by_net,
                    footprints=fps, board_info=BI,
                    nets={OWN: Net(OWN, '/OWN'), P2: Net(P2, '/OWN_N'),
                          FOREIGN: Net(FOREIGN, '/F'),
                          THIRD: Net(THIRD, '/T')})


# ---- the stub's own width (not the pairwise class) ---------------------------

def test_stub_pad_check_reads_the_stub_width():
    """A 0.5 mm pad whose edge is `gap` from the stub centreline, flat 0.2
    clearance: the requirement is the stub's half width + 0.2."""
    import stub_layer_switching as sls

    def admitted(width, gap):
        board = pcb(pads=[make_pad(FOREIGN, 0.0, gap + 0.25)])
        stub = [make_seg(-1, 0, 1, 0, net_id=OWN, width=width)]
        return sls.stub_clear_of_foreign_pads(stub, 'F.Cu', OWN, board,
                                              cfg(), set())[0]
    check('0.1 mm stub, pad edge 0.27 (needs 0.25): admitted',
          admitted(0.1, 0.27), True)
    check('0.3 mm stub, pad edge 0.32 (needs 0.35): refused',
          admitted(0.3, 0.32), False)


# ---- stub_layer_switching ------------------------------------------------------

def test_stub_validators():
    import stub_layer_switching as sls

    def barrel(board):
        return lambda c: sls.via_barrel_clear_of_foreign_copper(
            0.0, 0.0, OWN, board, c, set())[0]
    # via radius 0.3: a track edge 0.6 out, a via edge 0.6, a pad edge 0.6
    three_ways('via barrel vs a track 0.6 from its centre',
               barrel(pcb(segs=[make_seg(-2, 0.7, 2, 0.7, net_id=FOREIGN)])))
    three_ways('via barrel vs a via',
               barrel(pcb(vias=[make_via(0.0, 0.9, net_id=FOREIGN,
                                         size=0.6)])))
    three_ways('via barrel vs a pad',
               barrel(pcb(pads=[make_pad(FOREIGN, 0.0, 0.85)])))

    stub = [make_seg(-1, 0, 1, 0, net_id=OWN, width=0.2)]

    def pads(board, layer='F.Cu', segs=stub):
        return lambda c: sls.stub_clear_of_foreign_pads(
            segs, layer, OWN, board, c, set())[0]
    three_ways('stub vs a pad edge 0.4 away',
               pads(pcb(pads=[make_pad(FOREIGN, 0.0, 0.65)])))

    def tracks(board):
        return lambda c: sls.stub_clear_of_foreign_tracks(
            stub, 'F.Cu', OWN, board, c, set())[0]
    three_ways('stub vs a track edge 0.4 away',
               tracks(pcb(segs=[make_seg(-2, 0.5, 2, 0.5, net_id=FOREIGN)])))
    three_ways('stub vs a via edge 0.4 away',
               tracks(pcb(vias=[make_via(0.0, 0.7, net_id=FOREIGN,
                                         size=0.6)])))


def test_stub_kind_and_layer():
    """Which layer / kind each stub check prices with only shows under a
    .kicad_dru rule: a swapped stub on its DESTINATION layer (even against a
    through-hole pad that also has copper on a ruled layer), a via barrel on
    the track's layer and the stack against a via, and the #735 track rule
    binds a track, not a via."""
    import stub_layer_switching as sls
    stub = [make_seg(-1, 0, 1, 0, net_id=OWN, width=0.2, layer='F.Cu')]
    pad_b = pcb(pads=[make_pad(FOREIGN, 0.0, 0.6, layers=('B.Cu',))])

    def to_b(c, board=pad_b):
        return sls.stub_clear_of_foreign_pads(stub, 'B.Cu', OWN, board, c,
                                              set())[0]
    check('stub to B.Cu, pad edge 0.35: flat admits', to_b(cfg()), True)
    check('stub to B.Cu under a 0.3 rule on F.Cu: admits',
          to_b(cfg(layer_clearances={'F.Cu': 0.3})), True)
    check('stub to B.Cu under a 0.3 rule on B.Cu: refuses',
          to_b(cfg(layer_clearances={'B.Cu': 0.3})), False)
    th = pcb(pads=[make_pad(FOREIGN, 0.0, 0.6, layers=('*.Cu',), drill=0.3,
                            pad_type='thru_hole')])
    check('stub to B.Cu beside a through-hole pad, a 0.3 rule on F.Cu only: '
          'admits (the destination layer is unruled)',
          to_b(cfg(layer_clearances={'F.Cu': 0.3}), th), True)
    via_b = pcb(vias=[make_via(0.0, 0.9, net_id=FOREIGN, size=0.6)])
    check('via barrel vs a via, a 0.35 rule on B.Cu only: refuses (the '
          'stack)',
          sls.via_barrel_clear_of_foreign_copper(
              0.0, 0.0, OWN, via_b, cfg(layer_clearances={'B.Cu': WIDE}),
              set())[0], False)
    trk_b = pcb(segs=[make_seg(-2, 0.58, 2, 0.58, net_id=FOREIGN,
                               layer='B.Cu')])

    def barrel(c):
        return sls.via_barrel_clear_of_foreign_copper(0.0, 0.0, OWN, trk_b,
                                                      c, set())[0]
    check('via barrel, B.Cu track 0.58 out: flat refuses', barrel(cfg()),
          False)
    check('via barrel, same track under a 0.15 B.Cu rule: admits',
          barrel(cfg(layer_clearances={'B.Cu': 0.15})), True)
    trk = pcb(segs=[make_seg(-2, 0.5, 2, 0.5, net_id=FOREIGN)])
    via = pcb(vias=[make_via(0.0, 0.7, net_id=FOREIGN, size=0.6)])
    rule = cfg(track_clearances={FOREIGN: WIDE})
    check('stub vs a track under a 0.35 track rule: refuses',
          sls.stub_clear_of_foreign_tracks(stub, 'F.Cu', OWN, trk, rule,
                                           set())[0], False)
    check('stub vs a via under the same track rule: admits (tracks only)',
          sls.stub_clear_of_foreign_tracks(stub, 'F.Cu', OWN, via, rule,
                                           set())[0], True)


def test_stub_windows_reach_a_wide_class():
    """The stub checks prefilter by a fixed 1.5 mm window (2.0 mm for the
    barrel's pads). A 2.0 mm class reaches past it; each window now reaches
    the widest pair value. Every geometry is inside its pair value and
    outside the old window."""
    import stub_layer_switching as sls
    hv = cfg({FOREIGN: 2.0})
    stub = [make_seg(0, 0, 1, 0, net_id=OWN, width=0.2)]
    b = pcb(segs=[make_seg(-1, 2.05, 2, 2.05, net_id=FOREIGN, width=0.2)])
    check('a track 2.05 out under a 2.0 class (needs 2.2): refused',
          sls.stub_clear_of_foreign_tracks(stub, 'F.Cu', OWN, b, hv,
                                           set())[0], False)
    b = pcb(pads=[make_pad(FOREIGN, 0.5, 1.8)])
    check('a pad edge 1.55 out under a 2.0 class (needs 2.1): refused',
          sls.stub_clear_of_foreign_pads(stub, 'F.Cu', OWN, b, hv,
                                         set())[0], False)
    b = pcb(pads=[make_pad(FOREIGN, 0.0, 2.4)])
    check('a pad edge 2.15 from a via barrel under a 2.0 class: refused',
          sls.via_barrel_clear_of_foreign_copper(0.0, 0.0, OWN, b, hv,
                                                 set())[0], False)


# ---- net_rescue ------------------------------------------------------------------

def test_rescue_leg():
    from net_rescue import _leg_clear
    pts = [(-1.0, 0.0), (1.0, 0.0)]

    def leg(board):
        return lambda c: _leg_clear(board, pts, 'F.Cu', 0.2, 0.2, OWN,
                                    config=c)
    # leg half width 0.1: an edge 0.3 / 0.3 / 0.3 out
    three_ways('rescue leg vs a track',
               leg(pcb(segs=[make_seg(-2, 0.5, 2, 0.5, net_id=FOREIGN)])))
    three_ways('rescue leg vs a via',
               leg(pcb(vias=[make_via(0.0, 0.65, net_id=FOREIGN,
                                      size=0.6)])))
    three_ways('rescue leg vs a pad',
               leg(pcb(pads=[make_pad(FOREIGN, 0.0, 0.65,
                                      shape='circle')])))
    seg_b = pcb(segs=[make_seg(-2, 0.5, 2, 0.5, net_id=FOREIGN)])
    check('rescue leg, no config (the old call shape): flat, admits',
          _leg_clear(seg_b, pts, 'F.Cu', 0.2, 0.2, OWN), True)


def test_rescue_via_site():
    from net_rescue import _via_site_clear

    def site(board):
        return lambda c: _via_site_clear(board, 0.0, 0.0, c, OWN)
    # via radius 0.3: an edge 0.3 out
    three_ways('rescue via site vs a track',
               site(pcb(segs=[make_seg(-2, 0.7, 2, 0.7, net_id=FOREIGN)])))
    three_ways('rescue via site vs a via',
               site(pcb(vias=[make_via(0.0, 0.9, net_id=FOREIGN,
                                       size=0.6)])))
    three_ways('rescue via site vs a pad',
               site(pcb(pads=[make_pad(FOREIGN, 0.0, 0.8)])))


def _cap():
    pads = [make_pad(CAP_A, -0.5, 0.0, ref='C1', num='1'),
            make_pad(CAP_B, 0.5, 0.0, ref='C1', num='2')]
    return NS(reference='C1', x=0.0, y=0.0, rotation=0.0, pads=pads,
              locked=False)


def test_rescue_cap_relocation():
    """The nearest legal pose for a cap: a foreign item the flat value lets
    the first 0.1 mm candidate clear pushes it further under the class."""
    from net_rescue import _find_cap_relocation
    for label, extra in (
            ('track', dict(segs=[make_seg(-3, 0.58, 3, 0.58,
                                          net_id=FOREIGN)])),
            ('via', dict(vias=[make_via(0.5, 0.75, net_id=FOREIGN,
                                        size=0.6)])),
            ('pad', dict(pads=[make_pad(FOREIGN, 0.5, 0.72, ref='U7')]))):
        moved = {}
        for name, c, _ in arms():
            cap = _cap()
            fps = {'C1': cap}
            if 'pads' in extra:
                fps['U7'] = NS(reference='U7', pads=extra['pads'],
                               locked=False)
            board = pcb(segs=extra.get('segs', ()),
                        vias=extra.get('vias', ()), footprints=fps)
            pos = _find_cap_relocation(board, cap, [], [], 0.2, config=c)
            moved[name] = None if pos is None else round(math.hypot(*pos), 3)
        check(f'cap relocation vs a {label}: flat moves 0.1',
              moved['flat'], 0.1)
        check(f'cap relocation vs a {label}: a third net\'s class, 0.1',
              moved['other'], 0.1)
        check(f'cap relocation vs a {label}: the class moves it further',
              moved['wide'] is not None and moved['wide'] > 0.1, True)


def test_rescue_cap_conflicts():
    """A rescue via 0.8 mm from a movable cap pad (needs 0.75 flat): it
    conflicts only at the pad net's class."""
    from net_rescue import _cap_conflicts
    fan = [{'x': -0.5, 'y': 0.8, 'size': 0.6}]

    def conflicts(c):
        return not _cap_conflicts(pcb(footprints={'C1': _cap()}), fan, OWN,
                                  c)
    three_ways('cap-conflict scan, a rescue via vs a cap pad', conflicts,
               net=CAP_A)


# ---- layer_swap_optimization ------------------------------------------------------

def test_swap_via_fit():
    import layer_swap_optimization as lso
    v = make_via(0.0, 0.0, net_id=OWN, size=0.6)

    def fit(board, vias=None):
        return lambda c: lso._bare_pad_pair_vias_fit(board, vias or [v],
                                                     c)[0]
    # :124 -- need = vr 0.3 + w/2 0.1 + pair - margin 0.05
    three_ways('swap via vs a track 0.62 out',
               fit(pcb(segs=[make_seg(-2, 0.62, 2, 0.62, net_id=FOREIGN)])))
    three_ways('swap via vs a via',
               fit(pcb(vias=[make_via(0.0, 0.88, net_id=FOREIGN,
                                      size=0.6)])))
    three_ways('swap via vs a pad',
               fit(pcb(pads=[make_pad(FOREIGN, 0.0, 0.83)])))
    three_ways('swap P/N vias (the partner in the class)',
               fit(pcb(), [v, make_via(0.0, 0.88, net_id=P2, size=0.6)]),
               net=P2)
    # the deliberate grading margin is kept, taken off the pair value
    wide = cfg({FOREIGN: WIDE})
    check('swap via, a track 0.57 out (flat needs 0.6, margin 0.05): admits',
          fit(pcb(segs=[make_seg(-2, 0.57, 2, 0.57, net_id=FOREIGN)]))(cfg()),
          True)
    # (a diagonal track, so its bounding box holds the via and only the
    # exact distance, 0.707, decides)
    check('swap via, a track 0.707 out (the class needs 0.75): margin admits',
          fit(pcb(segs=[make_seg(1.0, 0.0, 0.0, 1.0, net_id=FOREIGN)]))(wide),
          True)
    check('swap via, a track 0.69 out under the class: refuses',
          fit(pcb(segs=[make_seg(-2, 0.69, 2, 0.69, net_id=FOREIGN)]))(wide),
          False)


def test_swap_and_fan_pad_override():
    """A pad's own override is weighed against the PAIR value, raise-only, as
    it always was (max(value, override); no grading margin when the override
    governs): an override below the class does not lower it."""
    import layer_swap_optimization as lso
    from diff_pair_multipoint import _fans_fit
    v = make_via(0.0, 0.0, net_id=OWN, size=0.6)
    # pad edge 0.53 from the via centre: needs 0.3 + clearance
    low = pcb(pads=[make_pad(FOREIGN, 0.0, 0.78, local_clearance=0.1)])
    wide = cfg({FOREIGN: WIDE})
    check('swap via, a 0.1-override pad under a 0.35 class: refused',
          lso._bare_pad_pair_vias_fit(low, [v], wide)[0], False)
    check('fan via, a 0.1-override pad under a 0.35 class: refused',
          _fans_fit(low, [(v, None)], [], wide), False)
    check('swap via, the same pad flat: admitted',
          lso._bare_pad_pair_vias_fit(low, [v], cfg())[0], True)
    # an override above the class still governs, with no margin
    high = pcb(pads=[make_pad(FOREIGN, 0.0, 0.84, local_clearance=0.4)])
    check('swap via, a 0.4-override pad edge 0.59 out (needs 0.7): refused',
          lso._bare_pad_pair_vias_fit(high, [v], wide)[0], False)


# ---- diff_pair_multipoint -----------------------------------------------------------

def test_fans_fit():
    from diff_pair_multipoint import _fans_fit
    v = make_via(0.0, 0.0, net_id=OWN, size=0.6)

    def ok(board, fans=None):
        return lambda c: _fans_fit(board, fans or [(v, None)], [], c)
    three_ways('fan via vs a track 0.65 out',
               ok(pcb(segs=[make_seg(-2, 0.65, 2, 0.65, net_id=FOREIGN)])))
    three_ways('fan via vs an existing via',
               ok(pcb(vias=[make_via(0.0, 0.88, net_id=FOREIGN,
                                     size=0.6)])))
    three_ways('fan via vs a pad',
               ok(pcb(pads=[make_pad(FOREIGN, 0.0, 0.83)])))
    w = make_via(0.0, 0.88, net_id=P2, size=0.6)
    three_ways('fan via vs the partner fan via',
               ok(pcb(), [(v, None), (w, None)]), net=P2)


def test_fans_fit_same_net_keeps_the_flat_value():
    """The via and pad passes have no net filter, so the fan via is tested
    against its OWN net's existing vias and other pads. check_drc grades no
    clearance there, so no pair value applies: they keep the flat value they
    were tested at, even with the net in a wide class."""
    from diff_pair_multipoint import _fans_fit
    v = make_via(0.0, 0.0, net_id=OWN, size=0.6)
    own_class = cfg({OWN: WIDE})
    check('fan via vs its own net\'s via 0.85 out, own net in a class: '
          'admitted (flat)',
          _fans_fit(pcb(vias=[make_via(0.0, 0.85, net_id=OWN, size=0.6)]),
                    [(v, None)], [], own_class), True)
    check('fan via vs its own net\'s pad, own net in a class: admitted '
          '(flat)',
          _fans_fit(pcb(pads=[make_pad(OWN, 0.0, 0.8)]), [(v, None)], [],
                    own_class), True)


# ---- length_matching ----------------------------------------------------------------

_AMP = dict(cx=5.0, cy=0.0, ux=1.0, uy=0.0, px=0.0, py=1.0, direction=1,
            max_amplitude=1.0, min_amplitude=0.1, layer='F.Cu')


def _idx(board, conf):
    from length_matching import ClearanceIndex
    i = ClearanceIndex()
    i.build(board, conf, None, None)
    return i


def _amp_three_ways(label, amp_of, net=FOREIGN):
    a = {name: amp_of(c) for name, c, _ in arms(net)}
    check(f'{label}: a third net\'s class leaves the amplitude',
          a['other'], a['flat'])
    check(f'{label}: the class shrinks it ({a["flat"]:.3f} -> '
          f'{a["wide"]:.3f})', a['wide'] < a['flat'], True)


def test_meander_amplitude():
    from length_matching import get_safe_amplitude_at_point as amp
    boards = {
        'track': pcb(segs=[make_seg(0, 1.6, 10, 1.6, net_id=FOREIGN)]),
        'via': pcb(vias=[make_via(5.0, 1.65, net_id=FOREIGN, size=0.6)]),
        'pad': pcb(pads=[make_pad(FOREIGN, 5.0, 1.6)]),
    }
    for label, board in boards.items():
        _amp_three_ways(f'meander vs a {label}',
                        lambda c, b=board: amp(pcb_data=b, net_id=OWN,
                                               config=c, **_AMP))
        _amp_three_ways(f'meander vs a {label}, through the index',
                        lambda c, b=board: amp(pcb_data=b, net_id=OWN,
                                               config=c,
                                               clearance_index=_idx(b, c),
                                               **_AMP))
    xs = [make_seg(0, 1.6, 10, 1.6, net_id=FOREIGN)]
    xv = [make_via(5.0, 1.65, net_id=FOREIGN, size=0.6)]
    for label, kw in (('extra_segments', {'extra_segments': xs}),
                      ('extra_vias', {'extra_vias': xv})):
        _amp_three_ways(f'meander vs {label}',
                        lambda c, kw=kw: amp(pcb_data=pcb(), net_id=OWN,
                                             config=c, **kw, **_AMP))
    pair = pcb(segs=[make_seg(0, 1.2, 10, 1.2, net_id=P2)])

    def gap_raised(c):
        # route_diff raises a pair's coupling gap to the clearance (#441),
        # its class (#530) and a .kicad_dru rule on its layer (#1145)
        from dataclasses import replace
        return replace(c, diff_pair_gap=max(c.diff_pair_gap,
                                            c.pair_clearance(OWN, P2, 'F.Cu')))
    _amp_three_ways('intra-pair meander vs its partner (paired_clearance)',
                    lambda c: amp(pcb_data=pair, net_id=OWN,
                                  config=gap_raised(c),
                                  paired_net_id=P2, **_AMP), net=P2)
    # a layer rule binds the partner like a class does: the coupled run is
    # built at a gap raised to it (#1145), so the bump is held to it too
    # (0.4: the amplitude search steps 0.7 -> 0.49, so 0.3 would tie)
    ruled = gap_raised(cfg(layer_clearances={'F.Cu': 0.4}))
    flat_a = amp(pcb_data=pair, net_id=OWN, config=gap_raised(cfg()),
                 paired_net_id=P2, **_AMP)
    ruled_a = amp(pcb_data=pair, net_id=OWN, config=ruled,
                  paired_net_id=P2, **_AMP)
    check('intra-pair meander: a layer rule shrinks the amplitude '
          f'({flat_a:.3f} -> {ruled_a:.3f})', ruled_a < flat_a, True)


def test_meander_own_pad_keeps_the_flat_value():
    """The index path does not filter the meandered net's own pads; they
    keep the flat value with the net in a class."""
    from length_matching import get_safe_amplitude_at_point as amp
    board = pcb(pads=[make_pad(OWN, 5.0, 1.6)])
    flat = amp(pcb_data=board, net_id=OWN, config=cfg(),
               clearance_index=_idx(board, cfg()), **_AMP)
    own = cfg({OWN: WIDE})
    check('meander vs its own pad through the index, own net in a class',
          amp(pcb_data=board, net_id=OWN, config=own,
              clearance_index=_idx(board, own), **_AMP), flat)


def test_meander_queries_reach_a_wide_class():
    """The index is queried at the WIDEST pair value: a 5 mm class is past
    the flat query plus its 2 mm slack, so a track / via 6 mm out sits in
    cells the flat query never visits."""
    from length_matching import get_safe_amplitude_at_point as amp
    hv = cfg({FOREIGN: 5.0})
    for label, board in (
            ('track', pcb(segs=[make_seg(0, 6.0, 10, 6.0, net_id=FOREIGN)])),
            ('via', pcb(vias=[make_via(5.0, 6.0, net_id=FOREIGN,
                                       size=0.6)]))):
        with_idx = amp(pcb_data=board, net_id=OWN, config=hv,
                       clearance_index=_idx(board, hv), **_AMP)
        no_idx = amp(pcb_data=board, net_id=OWN, config=hv, **_AMP)
        check(f'5 mm class, a {label} 6 mm out: the index agrees with the '
              f'full scan', with_idx, no_idx)
        check(f'5 mm class, a {label} 6 mm out: the meander shrinks',
              with_idx < 1.0, True)


def test_diff_pair_meander_amplitude():
    from length_matching import get_safe_amplitude_for_diff_pair as damp
    base = dict(cx=5.0, cy=0.0, ux=1.0, uy=0.0, px=0.0, py=1.0, direction=1,
                max_amplitude=1.0, min_amplitude=0.1, layer=0,
                p_net_id=OWN, n_net_id=P2, spacing_mm=0.3)
    for label, board in (
            ('track', pcb(segs=[make_seg(0, 1.8, 10, 1.8, net_id=FOREIGN)])),
            ('via', pcb(vias=[make_via(5.0, 2.03, net_id=FOREIGN,
                                       size=0.6)])),
            ('pad', pcb(pads=[make_pad(FOREIGN, 5.0, 2.05)]))):
        _amp_three_ways(f'diff-pair meander vs a {label}',
                        lambda c, b=board: damp(pcb_data=b, config=c,
                                                **base))
    board = pcb(segs=[make_seg(0, 1.8, 10, 1.8, net_id=FOREIGN)])
    a_flat = damp(pcb_data=board, config=cfg(), **base)
    a_n = damp(pcb_data=board, config=cfg({P2: WIDE}, routed=(OWN, P2)),
               **base)
    check('diff-pair meander: a class on the N half alone prices the pair',
          a_n < a_flat, True)


# ---- plane_blocker_detection (and its repair_planes callers) ---------------------

def test_restored_piece_collides():
    """Plane-tap restore: a ripped blocker's piece against the tap's new
    plane copper. Admitted = not colliding."""
    from plane_blocker_detection import _restored_piece_collides
    PLANE = THIRD + 1
    seg = {'start': (-1.0, 0.0), 'end': (1.0, 0.0), 'width': 0.2,
           'layer': 'F.Cu'}
    via = {'x': 0.0, 'y': 0.0, 'size': 0.6}
    pv = [{'x': 0.0, 'y': 0.7, 'size': 0.6}]
    pv9 = [{'x': 0.0, 'y': 0.9, 'size': 0.6}]
    ps5 = [{'start': (-1.0, 0.5), 'end': (1.0, 0.5), 'width': 0.2,
            'layer': 'F.Cu'}]
    ps7 = [{'start': (-1.0, 0.7), 'end': (1.0, 0.7), 'width': 0.2,
            'layer': 'F.Cu'}]

    def restored(s, v, vias, segs):
        return lambda c: not _restored_piece_collides(
            s, v, vias, segs, 0.6, 0.2, config=c, piece_net=FOREIGN,
            plane_net=PLANE)
    three_ways('restored track vs a plane via',
               restored(seg, None, pv, []))
    three_ways('restored track vs a plane track',
               restored(seg, None, [], ps5))
    three_ways('restored via vs a plane via', restored(None, via, pv9, []))
    three_ways('restored via vs a plane track', restored(None, via, [], ps7))
    check('restored track, no config (the old call shape): flat, admitted',
          not _restored_piece_collides(seg, None, pv, [], 0.6, 0.2), True)


# ---- single_ended_routing ------------------------------------------------------------

def test_unblock_via_refit():
    """A 0.6 via whose edge has 0.25 mm to another net's copper (the flat
    0.2 is met; the refit's 0.05 margin is taken off either value): refitted
    smaller, or refused, only at the class."""
    from single_ended_routing import _unblock_via_refit
    for label, board in (
            ('track', pcb(segs=[make_seg(-2, 0.65, 2, 0.65,
                                         net_id=FOREIGN)])),
            ('via', pcb(vias=[make_via(0.0, 0.85, net_id=FOREIGN,
                                       size=0.6)])),
            ('pad', pcb(pads=[make_pad(FOREIGN, 0.0, 0.8)]))):
        three_ways(f'unblock via keeps its size vs a {label}',
                   lambda c, b=board: _unblock_via_refit(
                       b, OWN, 0.0, 0.0, (0.6, 0.3), c) == (0.6, 0.3))
    # via against via is the STACK: a 0.15 rule on every layer replaces the
    # class on each layer, but check_drc keeps max(class, rules) for two vias
    ruled = cfg({FOREIGN: WIDE}, layer_clearances={'F.Cu': 0.15,
                                                   'B.Cu': 0.15})
    board = pcb(vias=[make_via(0.0, 0.85, net_id=FOREIGN, size=0.6)])
    check('unblock via vs a class via, every layer ruled 0.15: refitted '
          '(the stack keeps the class)',
          _unblock_via_refit(board, OWN, 0.0, 0.0, (0.6, 0.3), ruled)
          == (0.6, 0.3), False)


def test_merge_terminal_to_exact():
    """The grid cell 0.35 mm from a pad edge clears the flat 0.3 (nothing
    to merge); under the class it does not, and the exact endpoint does --
    so it merges."""
    from single_ended_routing import _merge_terminal_to_exact
    board = pcb(pads=[make_pad(FOREIGN, -0.6, 0.0)])

    def no_merge(c):
        c.grid_step = 0.1
        pts = [(0.0, 0.0), (0.1, 0.0)]
        return not _merge_terminal_to_exact(
            [(0, 0, 0), (1, 0, 0)], 0, 1, (0.12, 0.0, 'F.Cu'), pts, board,
            OWN, c, ['F.Cu', 'B.Cu'])
    three_ways('terminal merge (admitted = the grid cell already clears)',
               no_merge)
    relax = cfg({FOREIGN: WIDE}, layer_clearances={'F.Cu': 0.15})
    check('terminal merge: a 0.15 F.Cu rule replaces the class (no merge)',
          no_merge(relax), True)


# ---- diff_pair_routing ---------------------------------------------------------------

def test_collapse_leg_attach_join_foreign_pad():
    """The leg's grid corner sits 0.18 from its partner (below the 0.2
    intra floor) and the collapsed segment would sit 0.28 from it, so the
    join collapses -- unless the collapsed segment, 0.33 from a foreign
    pad's edge (flat needs 0.3), comes inside that pad's pair clearance."""
    from diff_pair_routing import _collapse_leg_attach_join
    partner = [make_seg(-5, -0.48, 5, -0.48, net_id=P2)]
    board = pcb(pads=[make_pad(FOREIGN, -1.5, 0.58)])

    def collapsed(c):
        c.grid_step = 0.1
        c.diff_pair_gap = 0.2
        leg = [make_seg(-3.0, 0.0, 0.0, -0.1, net_id=OWN),
               make_seg(0.0, -0.1, 0.05, 0.0, net_id=OWN)]
        out = _collapse_leg_attach_join(leg, (0.05, 0.0), c, board, OWN,
                                        partner)
        return len(out) == 1
    three_ways('collapsed leg join vs a foreign pad', collapsed)


# ---- the helper the pad sites call ---------------------------------------------

def test_before_override_is_pad_pair_clearance_short_of_the_override():
    """`pad_pair_clearance_before_override` is `pad_pair_clearance` (which
    test_1136_pair_clearance_parity holds to check_drc) with the override
    step taken off: applying `pad_override_clearance` to it gives back
    `pad_pair_clearance` on every class / rule / override / layer shape."""
    import itertools
    n = bad = 0
    for classes, rule, lc, layer, other in itertools.product(
            ({}, {FOREIGN: 0.35}, {OWN: 0.5}),
            ({}, {'F.Cu': 0.15}, {'F.Cu': 0.6, 'B.Cu': 0.25}),
            (0.0, 0.1, 0.9), (None, 'F.Cu', 'B.Cu'), (False, True)):
        c = cfg(classes or None, layer_clearances=dict(rule))
        pad = make_pad(FOREIGN, 0.0, 0.0, layers=('*.Cu',), drill=0.8,
                       pad_type='thru_hole', local_clearance=lc)
        op = make_pad(OWN, 2.0, 0.0) if other else None
        kw = dict(other_pad=op) if other else {}
        before = c.pad_pair_clearance_before_override(pad, OWN, layer, **kw)
        full = c.pad_pair_clearance(pad, OWN, layer, **kw)
        n += 1
        if abs(c.pad_override_clearance(before, pad, op) - full) > 1e-12:
            bad += 1
    check(f'{n} shapes: override(before_override) == pad_pair_clearance',
          bad, 0)


def test_stub_track_window_reaches_the_items_edge():
    """stub_clear_of_foreign_tracks windows each foreign item; the window
    must reach the item's EDGE, not its centre: a wide via or track whose
    centre is past `seg_half + mp` can still be within its pair value."""
    import stub_layer_switching as sls
    stub = [make_seg(0, 0, 5, 0, net_id=OWN, width=0.2)]
    for wide, vy, vsize in ((1.3, 1.6, 0.6), (2.5, 2.8, 0.8)):
        b = pcb(vias=[make_via(2.5, vy, net_id=FOREIGN, size=vsize)])
        ok, _ = sls.stub_clear_of_foreign_tracks(
            stub, 'F.Cu', OWN, b, cfg({FOREIGN: wide}), set())
        check(f'stub vs a {vsize} via of a {wide} class, edge '
              f'{vy - vsize / 2 - 0.1:.2f} away: refused', ok, False)
    for wide, ty, tw in ((1.3, 1.6, 0.6), (2.5, 3.0, 1.5)):
        b = pcb(segs=[make_seg(0, ty, 5, ty, net_id=FOREIGN, width=tw)])
        ok, _ = sls.stub_clear_of_foreign_tracks(
            stub, 'F.Cu', OWN, b, cfg({FOREIGN: wide}), set())
        check(f'stub vs a {tw} track of a {wide} class, edge '
              f'{ty - tw / 2 - 0.1:.2f} away: refused', ok, False)
    # no class at all: a 3 mm power track overlapping the stub is a short
    b = pcb(segs=[make_seg(0, 1.55, 5, 1.55, net_id=FOREIGN, width=3.0)])
    ok, _ = sls.stub_clear_of_foreign_tracks(stub, 'F.Cu', OWN, b, cfg(),
                                             set())
    check('stub vs an overlapping 3 mm track, no class: refused', ok, False)


def test_custom_pad_gets_the_class_excess():
    """A custom-shape foreign pad of a wide class is priced at its class
    like a rect one: the helpers `_pair_floor` feeds fold the class excess
    into the custom-pad distance too."""
    import single_ended_routing as ser
    c = cfg({FOREIGN: WIDE})
    base, ncl = ser._pair_floor(c, OWN, 'F.Cu')
    got = {}
    for custom in (False, True):
        kw = ({'polygons': [[(9.5, 9.5), (10.5, 9.5), (10.5, 10.5),
                             (9.5, 10.5)]]} if custom else {})
        p = make_pad(FOREIGN, 10, 10, size_x=1.0, size_y=1.0,
                     shape='custom' if custom else 'rect', **kw)
        b = pcb(pads=[p])
        got[custom] = ser._pt_foreign_pad_dist(
            b, OWN, 10.8, 10.0, 'F.Cu', base_clearance=base,
            net_clearances=ncl)
    check(f'custom pad effective distance {got[True]:.3f} == rect '
          f'{got[False]:.3f}', abs(got[True] - got[False]) < 1e-6, True)


TESTS = [test_stub_pad_check_reads_the_stub_width,
         test_stub_validators, test_stub_kind_and_layer,
         test_stub_windows_reach_a_wide_class,
         test_stub_track_window_reaches_the_items_edge,
         test_custom_pad_gets_the_class_excess,
         test_rescue_leg, test_rescue_via_site, test_rescue_cap_relocation,
         test_rescue_cap_conflicts,
         test_swap_via_fit, test_swap_and_fan_pad_override,
         test_fans_fit, test_fans_fit_same_net_keeps_the_flat_value,
         test_meander_amplitude, test_meander_own_pad_keeps_the_flat_value,
         test_meander_queries_reach_a_wide_class,
         test_diff_pair_meander_amplitude,
         test_restored_piece_collides,
         test_unblock_via_refit, test_merge_terminal_to_exact,
         test_collapse_leg_attach_join_foreign_pad,
         test_before_override_is_pad_pair_clearance_short_of_the_override]


def main(argv):
    only = argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        _CURRENT[0] = t.__name__
        print(f"--- {t.__name__}")
        ran += 1
        try:
            t()
        except Exception:                                  # noqa: BLE001
            traceback.print_exc()
            RESULTS.append((t.__name__, 'raised', False))
    if only and not ran:
        print(f"NO TEST matches {only}")
        return 2
    failed = [r for r in RESULTS if not r[2]]
    print(f"{len(RESULTS) - len(failed)}/{len(RESULTS)} cases pass")
    for t, label, _ in failed:
        print(f"  FAILED {t}: {label}")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
