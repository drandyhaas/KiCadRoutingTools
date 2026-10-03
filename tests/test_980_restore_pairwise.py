#!/usr/bin/env python3
"""#980: the restore-collision predicate prices each pair of nets at KiCad's
pairwise clearance, and its prefilter box reaches every threshold.

`rip_up_reroute._saved_route_colliders` decides whether a rolled-back net's
saved copper may be put back (15 call sites across rip_up_reroute, route,
diff_pair_custody, repair_planes and routing_context). It priced every
foreign object at one flat `clearance`; with a `config` it now prices a pair
at `config.pair_clearance` -- the restored item's own net against the foreign
item's -- which tests/test_980_pair_clearance_parity.py holds to check_drc.
`plane_blocker_detection._restored_piece_collides`, the #88.1 plane twin,
does the same. What each case pins:

* both directions: a restore 0.25 mm from a 0.35-class net is refused with a
  config and admitted without one; a .kicad_dru rule relaxing the layer to
  0.15 admits a restore the flat 0.2 refuses.
* inert: a config with nothing declared returns exactly the hits of the flat
  path, element for element, on a board with every pair kind.
* a P/N (two-net) restore prices each half at its OWN class.
* the prefilter box: a foreign track whose box lies past the old fixed 1 mm
  but within the pair's threshold is found -- a 2 mm power track (with or
  without a config), and a 1.2 mm class (with one).
* every pre-#980 call shape still works: positional, keyword `clearance`, a
  SimpleNamespace board, `partition_force_restores(..., skip_net_ids=...)`,
  and a duck-typed config without the method (flat).
* the plane twin: refused at the class, admitted without the nets.

    python3 tests/test_980_restore_pairwise.py
"""
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Segment, Via                  # noqa: E402
from routing_config import GridRouteConfig             # noqa: E402
import rip_up_reroute as rr                            # noqa: E402
from plane_blocker_detection import _restored_piece_collides  # noqa: E402


def seg(x1, y1, x2, y2, w, net, layer='F.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=net)


def via(x, y, size, net):
    return Via(x=x, y=y, size=size, drill=size / 2, layers=['F.Cu', 'B.Cu'],
               net_id=net)


def cfg_with(classes=None, layers=None, clearance=0.2):
    c = GridRouteConfig(clearance=clearance)
    if classes:
        c.set_net_clearances(classes, routed_net_ids=[1])
    if layers:
        c.layer_clearances = dict(layers)
    return c


def board(segs=(), vias=()):
    return NS(segments=list(segs), vias=list(vias))


def saved(segs=(), vias=()):
    return {'new_segments': list(segs), 'new_vias': list(vias)}


def test_a_wider_foreign_class_refuses_the_restore():
    # own track net 1 (Default) at y=0; foreign net 2 at edge gap 0.25
    s = saved([seg(0, 0, 5, 0, 0.2, 1)])
    pcb = board([seg(0, 0.45, 5, 0.45, 0.2, 2)])
    flat = rr._saved_route_collides(s, pcb, [1], 0.2)
    pair = rr._saved_route_collides(s, pcb, [1], 0.2,
                                    config=cfg_with({2: 0.35}))
    assert flat is False and pair is True, (flat, pair)
    # and every other pair kind: via-via, track-via, via-track
    v_s = saved(vias=[via(0, 0, 0.6, 1)])
    v_pcb = board(vias=[via(0, 0.85, 0.6, 2)])        # edge gap 0.25
    assert not rr._saved_route_collides(v_s, v_pcb, [1], 0.2)
    assert rr._saved_route_collides(v_s, v_pcb, [1], 0.2,
                                    config=cfg_with({2: 0.35}))
    tv_pcb = board(vias=[via(2.5, 0.65, 0.6, 2)])     # edge gap 0.25 to track
    assert not rr._saved_route_collides(s, tv_pcb, [1], 0.2)
    assert rr._saved_route_collides(s, tv_pcb, [1], 0.2,
                                    config=cfg_with({2: 0.35}))
    vt_pcb = board([seg(-2, 0.65, 2, 0.65, 0.2, 2)])  # via vs track
    assert not rr._saved_route_collides(v_s, vt_pcb, [1], 0.2)
    assert rr._saved_route_collides(v_s, vt_pcb, [1], 0.2,
                                    config=cfg_with({2: 0.35}))
    # the OWN net's class binds too (max of the two)
    assert rr._saved_route_collides(s, pcb, [1], 0.2,
                                    config=cfg_with({1: 0.35}))
    print("  PASS: track-track, via-via, track-via and via-track restores "
          "0.25mm from a 0.35 class are refused with a config, admitted "
          "flat; the own class binds as well")


def test_a_relaxing_layer_rule_admits():
    s = saved([seg(0, 0, 5, 0, 0.2, 1)])
    pcb = board([seg(0, 0.38, 5, 0.38, 0.2, 2)])      # edge gap 0.18
    assert rr._saved_route_collides(s, pcb, [1], 0.2)  # flat 0.2 refuses
    assert not rr._saved_route_collides(
        s, pcb, [1], 0.2, config=cfg_with(layers={'F.Cu': 0.15}))
    # ...on F.Cu only: the same pair on B.Cu is still 0.2
    s_b = saved([seg(0, 0, 5, 0, 0.2, 1, 'B.Cu')])
    pcb_b = board([seg(0, 0.38, 5, 0.38, 0.2, 2, 'B.Cu')])
    assert rr._saved_route_collides(
        s_b, pcb_b, [1], 0.2, config=cfg_with(layers={'F.Cu': 0.15}))
    print("  PASS: a .kicad_dru rule relaxing F.Cu to 0.15 admits a restore "
          "at 0.18 the flat 0.2 refuses; B.Cu is unruled and still refuses")


def _mixed_board():
    """Every pair kind at gaps around the floor, several nets."""
    segs, vias = [], []
    for i, g in enumerate((0.05, 0.15, 0.19, 0.21, 0.3, 0.6, 1.5)):
        y = 0.2 + g + i * 3.0
        segs.append(seg(0, y, 5, y, 0.2, 2 + i % 3))
        vias.append(via(6.0 + i, 0.3 + 0.3 + g, 0.6, 2 + (i + 1) % 3))
    return board(segs, vias)


def test_nothing_declared_is_the_flat_path_exactly():
    s = saved([seg(0, 0.0 + k * 3.0, 5, k * 3.0, 0.2, 1) for k in range(7)]
              + [seg(6.0, 0, 13.0, 0, 0.2, 1)],
              [via(6.0 + k, 0, 0.6, 1) for k in range(7)])
    pcb = _mixed_board()
    flat = rr._saved_route_colliders(s, pcb, [1], 0.2)
    inert = rr._saved_route_colliders(s, pcb, [1], 0.2,
                                      config=GridRouteConfig(clearance=0.2))
    assert flat, 'anti-vacuity: the fixture must collide somewhere'
    assert [(k, id(o)) for k, o in flat] == [(k, id(o)) for k, o in inert], \
        (len(flat), len(inert))
    print(f"  PASS: {len(flat)} hit(s), identical with and without an inert "
          f"config")


def test_a_two_net_restore_prices_each_half_at_its_own_class():
    # restoring nets 1 (Default) and 3 (class 0.35) together; foreign net 2
    # (Default) 0.25 from each piece
    cfg = cfg_with({3: 0.35})
    pcb = board([seg(0, 0.45, 5, 0.45, 0.2, 2)])
    p1 = saved([seg(0, 0, 5, 0, 0.2, 1)])
    p3 = saved([seg(0, 0, 5, 0, 0.2, 3)])
    assert not rr._saved_route_collides(p1, pcb, [1, 3], 0.2, config=cfg)
    assert rr._saved_route_collides(p3, pcb, [1, 3], 0.2, config=cfg)
    print("  PASS: P/N restore: the Default half is admitted at 0.25, the "
          "0.35-class half refused")


def test_the_prefilter_box_reaches_every_threshold():
    # (a) a 2.0 mm power track: half width 1.0 alone fills the old box
    s = saved([seg(0, 0, 5, 0, 2.0, 1)])
    pcb = board([seg(0, 1.25, 5, 1.25, 0.2, 2)])      # edge gap 0.15 < 0.2
    assert rr._saved_route_collides(s, pcb, [1], 0.2)
    # (b) a 1.2 mm class: the threshold passes 1 mm on a thin track
    s2 = saved([seg(0, 0, 5, 0, 0.2, 1)])
    pcb2 = board([seg(0, 1.25, 5, 1.25, 0.2, 2)])     # edge gap 1.05 < 1.2
    assert not rr._saved_route_collides(s2, pcb2, [1], 0.2)
    assert rr._saved_route_collides(s2, pcb2, [1], 0.2,
                                    config=cfg_with({2: 1.2}))
    # (c) a foreign via whose CENTRE is past the box but whose copper is not
    s3 = saved([seg(0, 0, 5, 0, 1.6, 1)])
    pcb3 = board(vias=[via(2.5, 1.45, 1.0, 2)])       # edge gap 0.15
    assert rr._saved_route_collides(s3, pcb3, [1], 0.2)
    print("  PASS: a 2mm track, a 1.2mm class and a via centred outside the "
          "old box are all found")


def test_every_pre_980_call_shape():
    s = saved([seg(0, 0, 5, 0, 0.2, 1)])
    pcb = board([seg(0, 0.35, 5, 0.35, 0.2, 2)])      # edge gap 0.15
    assert rr._saved_route_collides(s, pcb, [1], 0.2)          # positional
    assert rr._saved_route_collides(s, pcb, [1], clearance=0.2)
    hits = rr._saved_route_colliders(s, pcb, [1], 0.2, True)   # first_only
    assert len(hits) == 1, hits
    # a duck-typed config without the method is the flat path
    duck = NS(clearance=0.2)
    assert rr._saved_route_collides(s, pcb, [1], 0.2, config=duck)
    # partition_force_restores: the old keyword form, then with a config
    far = board([seg(0, 0.45, 5, 0.45, 0.2, 2)])      # edge gap 0.25
    fr = {1: ([seg(0, 0, 5, 0, 0.2, 1)], [])}
    far.segments = list(far.segments)
    ok, bad = rr.partition_force_restores(dict(fr), far, clearance=0.2,
                                          skip_net_ids=())
    assert ok == [1] and bad == [], (ok, bad)
    far2 = board([seg(0, 0.45, 5, 0.45, 0.2, 2)])
    ok, bad = rr.partition_force_restores(dict(fr), far2, 0.2,
                                          skip_net_ids=None,
                                          config=cfg_with({2: 0.35}))
    assert ok == [] and bad == [1], (ok, bad)
    print("  PASS: positional, keyword, first_only and duck-typed calls "
          "work; partition_force_restores refuses at the class with a config")


def test_the_plane_twin():
    sd = {'start': (0, 0), 'end': (5, 0), 'width': 0.2, 'layer': 'F.Cu'}
    plane_segs = [{'start': (0, 0.45), 'end': (5, 0.45), 'width': 0.2,
                   'layer': 'F.Cu'}]
    cfg = cfg_with({7: 0.35})
    args = (sd, None, [], plane_segs, 0.6, 0.2)
    assert not _restored_piece_collides(*args)
    assert not _restored_piece_collides(*args, config=cfg)   # no nets given
    assert _restored_piece_collides(*args, config=cfg, piece_net=1,
                                    plane_net=7)
    assert not _restored_piece_collides(*args, config=cfg, piece_net=1,
                                        plane_net=8)          # Default plane
    vd = {'x': 0, 'y': 0, 'size': 0.6}
    pv = [{'x': 0, 'y': 0.85}]                        # via-via edge gap 0.25
    assert not _restored_piece_collides(None, vd, pv, [], 0.6, 0.2)
    assert _restored_piece_collides(None, vd, pv, [], 0.6, 0.2, config=cfg,
                                    piece_net=1, plane_net=7)
    print("  PASS: the plane twin refuses at the plane's class and is flat "
          "without both nets")



def test_an_inherited_graze_is_not_a_collision_the_run_made():
    """Two pieces of copper the step was HANDED that a wider class puts too
    close: refusing the restore would ship the net open where the input had
    it connected. Counted only when the flat clearance flags it, or when the
    foreign copper moved in during the run."""
    own = seg(0, 0, 5, 0, 0.2, 1)
    near = seg(0, 0.45, 5, 0.45, 0.2, 2)          # edge gap 0.25
    c = cfg_with({2: 0.35})
    # the input board had both; net 1 is ripped and comes back
    board = board_ = NS(segments=[own, near], vias=[])
    rr.mark_input_copper(board_)
    board.segments = [near]
    ok, bad = rr.partition_force_restores({1: ([own], [])}, board, 0.2,
                                          config=c)
    assert ok == [1] and bad == [], (ok, bad)
    # control 1: the foreign copper MOVED IN during the run -> refused
    board2 = NS(segments=[own], vias=[])
    rr.mark_input_copper(board2)
    board2.segments = [near]
    ok, bad = rr.partition_force_restores({1: ([own], [])}, board2, 0.2,
                                          config=c)
    assert ok == [] and bad == [1], (ok, bad)
    # control 2: an inherited graze under the FLAT clearance still refuses,
    # as it did before #980
    tight = seg(0, 0.35, 5, 0.35, 0.2, 2)         # edge gap 0.15 < 0.2
    board3 = NS(segments=[own, tight], vias=[])
    rr.mark_input_copper(board3)
    board3.segments = [tight]
    ok, bad = rr.partition_force_restores({1: ([own], [])}, board3, 0.2,
                                          config=c)
    assert ok == [] and bad == [1], (ok, bad)
    print("  PASS: an inherited class graze is restored; copper that moved "
          "in, or a graze under the flat clearance, still refuses")


def test_kinds_under_layer_and_track_rules():
    """Which kind / layer each pair takes only shows under a .kicad_dru rule:
    via-via is the STACK, a restored track meets a via on the track's layer,
    a restored via meets a track on the track's, track-track takes the #735
    track rule. Each geometry is admitted flat and refused under the rule."""
    def coll(saved_, board, c):
        return rr._saved_route_collides(saved_, board, [1], 0.2, config=c)
    vv = (saved(vias=[via(0, 0, 0.6, 1)]),
          board(vias=[via(0, 0.9, 0.6, 2)]))        # gap 0.3
    assert not coll(*vv, cfg_with())
    assert coll(*vv, cfg_with(layers={'B.Cu': 0.5}))
    tv = (saved([seg(0, 0, 5, 0, 0.2, 1)]),
          board(vias=[via(2.5, 0.7, 0.6, 2)]))      # gap 0.3
    assert not coll(*tv, cfg_with())
    assert coll(*tv, cfg_with(layers={'F.Cu': 0.5}))
    assert not coll(*tv, cfg_with(layers={'B.Cu': 0.5}))
    vt = (saved(vias=[via(0, 0, 0.6, 1)]),
          board([seg(-2, 0.7, 2, 0.7, 0.2, 2)]))    # gap 0.3
    assert coll(*vt, cfg_with(layers={'F.Cu': 0.5}))
    assert not coll(*vt, cfg_with(layers={'B.Cu': 0.5}))
    tt = (saved([seg(0, 0, 5, 0, 0.2, 1)]),
          board([seg(0, 0.5, 5, 0.5, 0.2, 2)]))     # gap 0.3
    c = cfg_with()
    c.track_clearances = {2: 0.5}
    assert not coll(*tt, cfg_with())
    assert coll(*tt, c)
    # the plane twin: via-via is the stack, track-track the track rule
    vd = {'x': 0, 'y': 0, 'size': 0.6}
    pv = [{'x': 0, 'y': 0.9}]
    args = dict(config=cfg_with(layers={'B.Cu': 0.5}), piece_net=1,
                plane_net=7)
    assert not _restored_piece_collides(None, vd, pv, [], 0.6, 0.2)
    assert _restored_piece_collides(None, vd, pv, [], 0.6, 0.2, **args)
    sd = {'start': (0, 0), 'end': (5, 0), 'width': 0.2, 'layer': 'F.Cu'}
    ps = [{'start': (0, 0.5), 'end': (5, 0.5), 'width': 0.2,
           'layer': 'F.Cu'}]
    ct = cfg_with()
    ct.track_clearances = {7: 0.5}
    assert not _restored_piece_collides(sd, None, [], ps, 0.6, 0.2,
                                        config=cfg_with(), piece_net=1,
                                        plane_net=7)
    assert _restored_piece_collides(sd, None, [], ps, 0.6, 0.2, config=ct,
                                    piece_net=1, plane_net=7)
    sv = [{'x': 2.5, 'y': 0.7}]
    assert _restored_piece_collides(sd, None, sv, [], 0.6, 0.2,
                                    config=cfg_with(layers={'F.Cu': 0.5}),
                                    piece_net=1, plane_net=7)
    assert not _restored_piece_collides(sd, None, sv, [], 0.6, 0.2,
                                        config=cfg_with(layers={'B.Cu': 0.5}),
                                        piece_net=1, plane_net=7)
    print("  PASS: via-via takes the stack, track-via and via-track the "
          "track's layer, track-track the track rule (and in the plane twin)")

TESTS = [
    test_a_wider_foreign_class_refuses_the_restore,
    test_a_relaxing_layer_rule_admits,
    test_nothing_declared_is_the_flat_path_exactly,
    test_a_two_net_restore_prices_each_half_at_its_own_class,
    test_the_prefilter_box_reaches_every_threshold,
    test_every_pre_980_call_shape,
    test_the_plane_twin,
    test_an_inherited_graze_is_not_a_collision_the_run_made,
    test_kinds_under_layer_and_track_rules,
]


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
