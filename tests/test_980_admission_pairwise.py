#!/usr/bin/env python3
"""#980: every admit/refuse check against FOREIGN copper prices the pair at
check_drc's clearance, not the flat `config.clearance`.

#980's sweep (the maintainer's, then a second one) found these checks pricing
a foreign track, via or pad at one flat scalar: the oracle's sliver weld and
#649 stitching-via admission, the bare-pad swap via fit, the multipoint fan
fit, the two meander amplitude searches, the three stub-swap validators and
the #666 net-rescue escape/cap checks. Each now asks
`config.pair_clearance` / `pad_pair_clearance`, which
tests/test_980_pair_clearance_parity.py holds to check_drc itself.

Every case here is two-directional on one geometry, with a foreign net of a
0.35 class over a 0.2 floor:

* the flat config (nothing declared) ADMITS -- and it is the pre-#980 answer;
* the same config with the class REFUSES.

Plus the cases the sweep changed beyond the class term: a pad override below
its class now REPLACES it (check_drc's rule), the swap's deliberate grading
margin is kept, the meander's spatial query reaches a class wider than its
2 mm slack, and the stub-pad check sizes the stub at its OWN width.

    python3 tests/test_980_admission_pairwise.py [case-substring ...]
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_tools', 'rust_router'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

from types import SimpleNamespace as NS                   # noqa: E402
from kicad_parser import BoardInfo, Net                   # noqa: E402
from routing_config import GridRouteConfig                # noqa: E402
from synth import make_pad, make_seg, make_via, make_pcb  # noqa: E402

OWN, P2, FOREIGN = 1, 2, 99
WIDE = 0.35
BI = BoardInfo(layers={}, board_bounds=None, copper_layers=['F.Cu', 'B.Cu'])


def cfg(wide=False, nets=(FOREIGN,), **kw):
    c = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                        via_drill=0.3, layers=['F.Cu', 'B.Cu'], grid_step=0.05)
    c.hole_to_hole_clearance = 0.2
    for k, v in kw.items():
        setattr(c, k, v)
    if wide:
        c.set_net_clearances({n: WIDE for n in nets}, routed_net_ids=[OWN])
    return c


def pcb(segs=(), vias=(), pads=(), footprints=None):
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    fps = footprints if footprints is not None else (
        {'U9': NS(reference='U9', pads=list(pads), locked=False)}
        if pads else {})
    return make_pcb(segments=segs, vias=vias, pads_by_net=by_net,
                    footprints=fps, board_info=BI,
                    nets={OWN: Net(OWN, '/OWN'), P2: Net(P2, '/OWN_N'),
                          FOREIGN: Net(FOREIGN, '/WIDE')})


def two_ways(label, flat, wide, flat_ok=True):
    """`flat` / `wide` are the verdicts (True = admitted) on one geometry."""
    assert flat is flat_ok, (label, 'flat config', flat)
    assert wide is (not flat_ok), (label, 'wide class', wide)
    print(f"  PASS: {label}: flat {'admits' if flat else 'refuses'}, "
          f"the {WIDE} class {'admits' if wide else 'refuses'}")


# ---- kicad_oracle -------------------------------------------------------------

def test_sliver_weld():
    from kicad_oracle import _direct_sliver_weld

    def w(board, c):
        return _direct_sliver_weld(board, OWN, 0.0, 0.0, 0.5, 0.0, 'F.Cu',
                                   c) is not None
    # need = reach 0.105 + item + clearance
    seg_b = pcb(segs=[make_seg(-1, 0.5, 2, 0.5, net_id=FOREIGN)])
    two_ways('sliver weld vs a foreign track', w(seg_b, cfg()),
             w(seg_b, cfg(True)))
    via_b = pcb(vias=[make_via(0.25, 0.75, net_id=FOREIGN, size=0.6)])
    two_ways('sliver weld vs a foreign via', w(via_b, cfg()),
             w(via_b, cfg(True)))
    pad_b = pcb(pads=[make_pad(FOREIGN, 0.25, 0.7)])
    two_ways('sliver weld vs a foreign pad', w(pad_b, cfg()),
             w(pad_b, cfg(True)))


def test_stitching_via():
    from kicad_oracle import _stitch_via_clear

    def ok(board, c):
        return _stitch_via_clear(board, OWN, 0.0, 0.0, c, 0.2)
    seg_b = pcb(segs=[make_seg(-2, 0.7, 2, 0.7, net_id=FOREIGN)])
    two_ways('#649 via vs a foreign track', ok(seg_b, cfg()),
             ok(seg_b, cfg(True)))
    via_b = pcb(vias=[make_via(0.0, 0.9, net_id=FOREIGN, size=0.6)])
    two_ways('#649 via vs a foreign via', ok(via_b, cfg()),
             ok(via_b, cfg(True)))
    pad_b = pcb(pads=[make_pad(FOREIGN, 0.0, 0.85)])
    two_ways('#649 via vs a foreign pad', ok(pad_b, cfg()),
             ok(pad_b, cfg(True)))
    # a pad override below its class REPLACES it (check_drc's rule): the
    # override 0.1 admits what the class 0.35 alone refuses
    ovr = pcb(pads=[make_pad(FOREIGN, 0.0, 0.7, local_clearance=0.1)])
    plain = pcb(pads=[make_pad(FOREIGN, 0.0, 0.7)])
    assert ok(ovr, cfg(True)) and not ok(plain, cfg(True))
    print("  PASS: #649 via: a pad override below the class replaces it")


# ---- layer_swap_optimization --------------------------------------------------

def test_swap_via_fit():
    import layer_swap_optimization as lso

    def fit(board, vias, c):
        return lso._bare_pad_pair_vias_fit(board, vias, c)[0]
    v = make_via(0.0, 0.0, net_id=OWN, size=0.6)
    # :124 -- need = vr 0.3 + w/2 0.1 + clearance - margin 0.05
    seg_b = pcb(segs=[make_seg(-2, 0.62, 2, 0.62, net_id=FOREIGN)])
    two_ways('swap via vs a foreign track', fit(seg_b, [v], cfg()),
             fit(seg_b, [v], cfg(True)))
    # the deliberate grading margin is kept: 0.57 is inside the 0.6 flat
    # requirement but within its 0.05 margin
    near = pcb(segs=[make_seg(-2, 0.57, 2, 0.57, net_id=FOREIGN)])
    assert fit(near, [v], cfg()), 'the swap margin must still admit'
    print("  PASS: swap via: the deliberate 0.05 grading margin is kept")
    # :83 -- a foreign via
    via_b = pcb(vias=[make_via(0.0, 0.88, net_id=FOREIGN, size=0.6)])
    two_ways('swap via vs a foreign via', fit(via_b, [v], cfg()),
             fit(via_b, [v], cfg(True)))
    # :72 -- the P/N pair's own vias, N in the wide class
    n = make_via(0.0, 0.88, net_id=P2, size=0.6)
    two_ways('swap P/N vias', fit(pcb(), [v, n], cfg()),
             fit(pcb(), [v, n], cfg(True, nets=(P2,))))
    # :100 -- a foreign pad
    pad_b = pcb(pads=[make_pad(FOREIGN, 0.0, 0.83)])
    two_ways('swap via vs a foreign pad', fit(pad_b, [v], cfg()),
             fit(pad_b, [v], cfg(True)))


# ---- diff_pair_multipoint -------------------------------------------------------

def test_fans_fit():
    from diff_pair_multipoint import _fans_fit
    v = make_via(0.0, 0.0, net_id=OWN, size=0.6)

    def ok(board, c, fans=None):
        return _fans_fit(board, fans or [(v, None)], [], c)
    seg_b = pcb(segs=[make_seg(-2, 0.65, 2, 0.65, net_id=FOREIGN)])
    two_ways('fan via vs a foreign track', ok(seg_b, cfg()),
             ok(seg_b, cfg(True)))
    via_b = pcb(vias=[make_via(0.0, 0.88, net_id=FOREIGN, size=0.6)])
    two_ways('fan via vs a foreign via', ok(via_b, cfg()),
             ok(via_b, cfg(True)))
    pad_b = pcb(pads=[make_pad(FOREIGN, 0.0, 0.83)])
    two_ways('fan via vs a foreign pad', ok(pad_b, cfg()),
             ok(pad_b, cfg(True)))
    w = make_via(0.0, 0.88, net_id=P2, size=0.6)
    two_ways('fan via vs the partner fan via', ok(pcb(), cfg(), [(v, None), (w, None)]),
             ok(pcb(), cfg(True, nets=(P2,)), [(v, None), (w, None)]))


# ---- length_matching --------------------------------------------------------------

_AMP = dict(cx=5.0, cy=0.0, ux=1.0, uy=0.0, px=0.0, py=1.0, direction=1,
            max_amplitude=1.0, min_amplitude=0.1, layer='F.Cu')


def test_meander_amplitude():
    from length_matching import get_safe_amplitude_at_point as amp
    board = pcb(segs=[make_seg(0, 1.6, 10, 1.6, net_id=FOREIGN, width=0.2)])
    a_flat = amp(pcb_data=board, net_id=OWN, config=cfg(), **_AMP)
    a_wide = amp(pcb_data=board, net_id=OWN, config=cfg(True), **_AMP)
    assert a_flat > a_wide, (a_flat, a_wide)
    print(f"  PASS: meander vs a foreign track: amplitude {a_flat} flat, "
          f"{a_wide} at the class")
    vb = pcb(vias=[make_via(5.0, 1.65, net_id=FOREIGN, size=0.6)])
    v_flat = amp(pcb_data=vb, net_id=OWN, config=cfg(), **_AMP)
    v_wide = amp(pcb_data=vb, net_id=OWN, config=cfg(True), **_AMP)
    assert v_flat > v_wide, (v_flat, v_wide)
    pb = pcb(pads=[make_pad(FOREIGN, 5.0, 1.6)])
    p_flat = amp(pcb_data=pb, net_id=OWN, config=cfg(), **_AMP)
    p_wide = amp(pcb_data=pb, net_id=OWN, config=cfg(True), **_AMP)
    assert p_flat > p_wide, (p_flat, p_wide)
    # intra-pair: the partner's class prices the partner
    pair = pcb(segs=[make_seg(0, 1.2, 10, 1.2, net_id=P2)])
    i_flat = amp(pcb_data=pair, net_id=OWN, config=cfg(), paired_net_id=P2,
                 **_AMP)
    i_wide = amp(pcb_data=pair, net_id=OWN, config=cfg(True, nets=(P2,)),
                 paired_net_id=P2, **_AMP)
    assert i_flat > i_wide, (i_flat, i_wide)
    print(f"  PASS: meander vs a via ({v_flat} -> {v_wide}), a pad "
          f"({p_flat} -> {p_wide}) and its pair partner "
          f"({i_flat} -> {i_wide})")


def test_meander_query_reaches_a_wide_class():
    """The spatial index is queried at the WIDEST pair clearance. A 5 mm
    class is past the flat value plus the 2 mm slack the query used to add;
    with the index's 2 mm cells (each track registered 1.4 mm wide) the track
    6 mm out sits in cells the flat query never visits."""
    from length_matching import get_safe_amplitude_at_point as amp
    from length_matching import ClearanceIndex
    board = pcb(segs=[make_seg(0, 6.0, 10, 6.0, net_id=FOREIGN, width=0.2)])
    c = cfg()
    c.set_net_clearances({FOREIGN: 5.0}, routed_net_ids=[OWN])
    def _idx(conf):
        i = ClearanceIndex()
        i.build(board, conf, None, None)
        return i
    with_idx = amp(pcb_data=board, net_id=OWN, config=c,
                   clearance_index=_idx(c), **_AMP)
    no_idx = amp(pcb_data=board, net_id=OWN, config=c, **_AMP)
    assert with_idx == no_idx, (with_idx, no_idx)
    assert with_idx < amp(pcb_data=board, net_id=OWN, config=cfg(),
                          clearance_index=_idx(cfg()), **_AMP)
    print(f"  PASS: a 5mm class shrinks the meander to {with_idx} through "
          f"the index as without it")


def test_diff_pair_meander_amplitude():
    from length_matching import get_safe_amplitude_for_diff_pair as damp
    import inspect
    params = set(inspect.signature(damp).parameters)
    board = pcb(segs=[make_seg(0, 1.8, 10, 1.8, net_id=FOREIGN, width=0.2)])
    kw = dict(cx=5.0, cy=0.0, ux=1.0, uy=0.0, px=0.0, py=1.0, direction=1,
              max_amplitude=1.0, min_amplitude=0.1, layer=0,
              pcb_data=board, p_net_id=OWN, n_net_id=P2, spacing_mm=0.3)
    kw = {k: v for k, v in kw.items() if k in params}
    a_flat = damp(config=cfg(), **kw)
    a_wide = damp(config=cfg(True), **kw)
    assert a_flat > a_wide, (a_flat, a_wide)
    print(f"  PASS: diff-pair meander vs a foreign track: {a_flat} -> "
          f"{a_wide}")


# ---- stub_layer_switching -----------------------------------------------------

def test_stub_validators():
    import stub_layer_switching as sls
    seg_b = pcb(segs=[make_seg(-2, 0.7, 2, 0.7, net_id=FOREIGN)])

    def barrel(board, c):
        return sls.via_barrel_clear_of_foreign_copper(
            0.0, 0.0, OWN, board, c, set())[0]
    two_ways('swap via barrel vs a foreign track', barrel(seg_b, cfg()),
             barrel(seg_b, cfg(True)))
    via_b = pcb(vias=[make_via(0.0, 0.9, net_id=FOREIGN, size=0.6)])
    two_ways('swap via barrel vs a foreign via', barrel(via_b, cfg()),
             barrel(via_b, cfg(True)))
    pad_b = pcb(pads=[make_pad(FOREIGN, 0.0, 0.85)])
    two_ways('swap via barrel vs a foreign pad', barrel(pad_b, cfg()),
             barrel(pad_b, cfg(True)))

    stub = [make_seg(-1, 0, 1, 0, net_id=OWN, width=0.2)]

    def pads_ok(board, c, segs=stub):
        return sls.stub_clear_of_foreign_pads(segs, 'F.Cu', OWN, board, c,
                                              set())[0]
    # pad edge 0.4 from the stub centreline: need 0.1 + clearance
    pad_e = pcb(pads=[make_pad(FOREIGN, 0.0, 0.65)])
    two_ways('stub vs a foreign pad', pads_ok(pad_e, cfg()),
             pads_ok(pad_e, cfg(True)))

    def tracks_ok(board, c):
        return sls.stub_clear_of_foreign_tracks(stub, 'F.Cu', OWN, board, c,
                                                set())[0]
    trk = pcb(segs=[make_seg(-2, 0.5, 2, 0.5, net_id=FOREIGN)])
    two_ways('stub vs a foreign track', tracks_ok(trk, cfg()),
             tracks_ok(trk, cfg(True)))
    vtrk = pcb(vias=[make_via(0.0, 0.7, net_id=FOREIGN, size=0.6)])
    two_ways('stub vs a foreign via', tracks_ok(vtrk, cfg()),
             tracks_ok(vtrk, cfg(True)))


def test_stub_pad_check_reads_the_stub_width():
    """The stub-vs-pad check sized every stub at config.track_width (0.2):
    a 0.1 stub was refused where it fits, a 0.3 one admitted where it
    grazes. It now reads the stub's own width, as the track check does."""
    import stub_layer_switching as sls
    c = cfg()

    def ok(width, pad_edge):
        board = pcb(pads=[make_pad(FOREIGN, 0.0, pad_edge + 0.25)])
        return sls.stub_clear_of_foreign_pads(
            [make_seg(-1, 0, 1, 0, net_id=OWN, width=width)], 'F.Cu', OWN,
            board, c, set())[0]
    assert ok(0.1, 0.28), 'a 0.1 stub 0.28 from a pad needs only 0.25'
    assert not ok(0.3, 0.32), 'a 0.3 stub 0.32 from a pad needs 0.35'
    print("  PASS: a 0.1mm stub 0.28mm from a pad is admitted, a 0.3mm one "
          "0.32mm away refused")


# ---- net_rescue ----------------------------------------------------------------

def test_rescue_leg_and_via_site():
    from net_rescue import _leg_clear, _via_site_clear
    seg_b = pcb(segs=[make_seg(-2, 0.5, 2, 0.5, net_id=FOREIGN)])
    pts = [(-1.0, 0.0), (1.0, 0.0)]

    def leg(board, c):
        return _leg_clear(board, pts, 'F.Cu', 0.2, 0.2, OWN, config=c)
    two_ways('rescue leg vs a foreign track', leg(seg_b, cfg()),
             leg(seg_b, cfg(True)))
    # no config: the flat pre-#980 call shape
    assert _leg_clear(seg_b, pts, 'F.Cu', 0.2, 0.2, OWN)
    via_b = pcb(vias=[make_via(0.0, 0.65, net_id=FOREIGN, size=0.6)])
    two_ways('rescue leg vs a foreign via', leg(via_b, cfg()),
             leg(via_b, cfg(True)))
    pad_b = pcb(pads=[make_pad(FOREIGN, 0.0, 0.65, shape='circle')])
    two_ways('rescue leg vs a foreign pad', leg(pad_b, cfg()),
             leg(pad_b, cfg(True)))

    def site(board, c):
        return _via_site_clear(board, 0.0, 0.0, c, OWN)
    s7 = pcb(segs=[make_seg(-2, 0.7, 2, 0.7, net_id=FOREIGN)])
    two_ways('rescue via site vs a foreign track', site(s7, cfg()),
             site(s7, cfg(True)))
    v9 = pcb(vias=[make_via(0.0, 0.9, net_id=FOREIGN, size=0.6)])
    two_ways('rescue via site vs a foreign via', site(v9, cfg()),
             site(v9, cfg(True)))
    p5 = pcb(pads=[make_pad(FOREIGN, 0.0, 0.8)])
    two_ways('rescue via site vs a foreign pad', site(p5, cfg()),
             site(p5, cfg(True)))


def _cap(x=0.0, y=0.0):
    pads = [make_pad(5, x - 0.5, y, ref='C1', num='1'),
            make_pad(6, x + 0.5, y, ref='C1', num='2')]
    return NS(reference='C1', x=x, y=y, rotation=0.0, pads=pads, locked=False)


def test_rescue_cap_relocation_and_conflicts():
    from net_rescue import _find_cap_relocation, _cap_conflicts
    import math
    cap = _cap()
    board = pcb(segs=[make_seg(-3, 0.58, 3, 0.58, net_id=FOREIGN)],
                footprints={'C1': cap})
    board.board_info = BoardInfo(layers={}, board_bounds=None,
                                 copper_layers=['F.Cu', 'B.Cu'])
    flat = _find_cap_relocation(board, cap, [], [], 0.2, config=cfg())
    wide = _find_cap_relocation(board, cap, [], [], 0.2, config=cfg(True))
    assert flat is not None and wide is not None, (flat, wide)
    d_flat, d_wide = math.hypot(*flat), math.hypot(*wide)
    assert d_wide > d_flat + 1e-9, (flat, wide)
    print(f"  PASS: cap relocation moves {d_flat:.2f}mm flat, {d_wide:.2f}mm "
          f"clear of the class")
    # a rescue via 0.8 from a movable cap pad of a wide-class net
    cap2 = _cap()
    b2 = pcb(footprints={'C1': cap2})
    fan = [{'x': -0.5, 'y': 0.8, 'size': 0.6}]
    assert not _cap_conflicts(b2, fan, OWN, cfg())
    assert set(_cap_conflicts(b2, fan, OWN, cfg(True, nets=(5,)))) == {'C1'}
    print("  PASS: a rescue via 0.8mm from a cap pad conflicts only at the "
          "pad net's class")


TESTS = [test_sliver_weld, test_stitching_via, test_swap_via_fit,
         test_fans_fit, test_meander_amplitude,
         test_meander_query_reaches_a_wide_class,
         test_diff_pair_meander_amplitude, test_stub_validators,
         test_stub_pad_check_reads_the_stub_width,
         test_rescue_leg_and_via_site,
         test_rescue_cap_relocation_and_conflicts]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
