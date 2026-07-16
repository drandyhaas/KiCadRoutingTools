"""Net-tie footprint support (net_tie_pad_groups -- Kelvin shunts, net-ties).

KiCad exempts the grouped pads' mutual clearance: the partner pad often
physically encloses the tied pad (cynthion R42: the AUX_SENSE- tab sits
inside AUX_VBUS_IN's pad), so treating the partner as a hard obstacle seals
the sense net at birth and check_drc would flag the (KiCad-legal) escape
track. Covers:

  1. parser reads (net_tie_pad_groups "1, 2") into footprint.net_tie_groups
  2. PCBData.net_tie_exempt_pad_ids() maps a tied net to its partner pads
  3. build_base_obstacle_map lifts the partner pad's TRACK stamp for a
     single-net scoped build, keeps it for other nets, and keeps VIA blocking
  4. check_drc does not flag the tied net's track grazing the partner pad,
     but still flags an unrelated net's track doing the same
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

BOARD = """(kicad_pcb (version 20241229) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (25 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "VBUS")
  (net 2 "SENSE")
  (net 3 "OTHER")
  (footprint "test:kelvin_shunt" (layer "F.Cu")
    (at 10 10)
    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (net_tie_pad_groups "1, 2")
    (pad "1" smd roundrect (at 0 0.325) (size 0.95 0.8) (layers "F.Cu")
      (roundrect_rratio 0.25) (net 1 "VBUS"))
    (pad "2" smd rect (at 0 0) (size 0.5 0.15) (layers "F.Cu") (net 2 "SENSE"))
  )
  (footprint "test:res" (layer "F.Cu")
    (at 20 10)
    (property "Reference" "R2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "SENSE"))
    (pad "2" smd rect (at 0 2) (size 0.5 0.5) (layers "F.Cu") (net 3 "OTHER"))
  )
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))
)
"""


def _board():
    from kicad_parser import parse_kicad_pcb
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD)
        path = f.name
    try:
        return parse_kicad_pcb(path)
    finally:
        os.unlink(path)


def test_parser_and_exempt_sets():
    pcb = _board()
    fp = pcb.footprints['R1']
    assert fp.net_tie_groups == [['1', '2']], fp.net_tie_groups
    sense = [i for i, n in pcb.nets.items() if n.name == 'SENSE'][0]
    vbus = [i for i, n in pcb.nets.items() if n.name == 'VBUS'][0]
    other = [i for i, n in pcb.nets.items() if n.name == 'OTHER'][0]
    ex_sense = pcb.net_tie_exempt_pad_ids(sense)
    ex_vbus = pcb.net_tie_exempt_pad_ids(vbus)
    ex_other = pcb.net_tie_exempt_pad_ids(other)
    pad_v = [p for p in fp.pads if p.pad_number == '1'][0]
    pad_s = [p for p in fp.pads if p.pad_number == '2'][0]
    assert id(pad_v) in ex_sense and len(ex_sense) == 1
    assert id(pad_s) in ex_vbus and len(ex_vbus) == 1
    assert not ex_other
    print("  PASS  parser groups + exempt sets (both directions, OTHER empty)")


def test_obstacle_stamp_lifted():
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map
    pcb = _board()
    sense = [i for i, n in pcb.nets.items() if n.name == 'SENSE'][0]
    other = [i for i, n in pcb.nets.items() if n.name == 'OTHER'][0]
    cfg = GridRouteConfig(grid_step=0.05, clearance=0.1, track_width=0.127,
                          via_size=0.45, via_drill=0.2, layers=['F.Cu', 'B.Cu'])
    coord = GridCoord(cfg.grid_step)
    # The KiCad-legal sliver: own SENSE pad (10, 10) 0.5x0.15 shrunk by half
    # the track width -> only the centre row (y = 10.0) qualifies. The partner
    # pad centre (10, 10.325) is OUTSIDE the sliver and must stay blocked.
    g_sliver = coord.to_grid(10.0, 10.0)
    g_center = coord.to_grid(10.0, 10.325)
    obs = build_base_obstacle_map(pcb, cfg, [sense])
    assert not obs.is_blocked(g_sliver[0], g_sliver[1], 0), \
        "own-pad sliver must be passable for SENSE"
    assert obs.is_blocked(g_center[0], g_center[1], 0), \
        "partner pad centre stays blocked (outside the own-pad sliver)"
    assert obs.is_via_blocked(g_sliver[0], g_sliver[1]), "via blocking must NOT be lifted"
    # Routing OTHER: nothing lifted anywhere
    obs2 = build_base_obstacle_map(pcb, cfg, [other])
    assert obs2.is_blocked(g_sliver[0], g_sliver[1], 0), \
        "sliver must still block OTHER"
    print("  PASS  base map lifts only the own-pad sliver for the tied net; vias stay blocked")


def test_prepare_restore_lift():
    import numpy as np
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map
    from routing_context import prepare_obstacles_inplace, restore_obstacles_inplace
    pcb = _board()
    sense = [i for i, n in pcb.nets.items() if n.name == 'SENSE'][0]
    other = [i for i, n in pcb.nets.items() if n.name == 'OTHER'][0]
    cfg = GridRouteConfig(grid_step=0.05, clearance=0.1, track_width=0.127,
                          via_size=0.45, via_drill=0.2, layers=['F.Cu', 'B.Cu'])
    coord = GridCoord(cfg.grid_step)
    gx, gy = coord.to_grid(10.0, 10.0)  # own-pad sliver cell
    # Batch build (both nets to route): tie pad stamped, lift arrays recorded
    obs = build_base_obstacle_map(pcb, cfg, [sense, other])
    assert obs.is_blocked(gx, gy, 0), "batch base map stamps the partner pad"
    _, cells = prepare_obstacles_inplace(
        obs, pcb, cfg, sense, [sense, other], [], {}, {'F.Cu': 0, 'B.Cu': 1}, {})
    assert not obs.is_blocked(gx, gy, 0), "prepare lifts the sliver for SENSE"
    restore_obstacles_inplace(obs, sense, {}, np.empty((0, 2), dtype=np.int32))
    assert obs.is_blocked(gx, gy, 0), "restore re-adds the sliver stamp"
    print("  PASS  prepare/restore lift + re-add the own-pad sliver (balanced)")


def _run_drc_with_segments(seg_sexprs):
    from check_drc import run_drc
    board = BOARD.rstrip()[:-1] + '\n' + '\n'.join(seg_sexprs) + '\n)\n'
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(board)
        path = f.name
    try:
        return run_drc(path, clearance=0.1, quiet=True, print_summary=False,
                       check_sizes=False)
    finally:
        os.unlink(path)


def test_check_drc_exemption():
    # Legal Kelvin exit: straight SOUTH from the sense pad -- the copper's
    # only contact with the partner happens inside the own pad (the shared
    # bottom-edge strip), like the human cynthion escape.
    south = ('(segment (start 10 10) (end 10 8.8) (width 0.127) '
             '(layer "F.Cu") (net 2) (uuid "aaaaaaaa-0000-0000-0000-000000000001"))')
    v = _run_drc_with_segments([south])
    tie_hits = [x for x in v if x.get('type') == 'pad-segment'
                and 'VBUS' in (x.get('net1'), x.get('net2'))
                and 'SENSE' in (x.get('net1'), x.get('net2'))]
    assert not tie_hits, f"local tie contact must not be flagged: {tie_hits}"
    # A SENSE track ploughing NORTH through the partner's heart (deepest
    # contact far outside the own pad) is NOT the KiCad-legal contact.
    north = ('(segment (start 10 10) (end 10 11.2) (width 0.127) '
             '(layer "F.Cu") (net 2) (uuid "aaaaaaaa-0000-0000-0000-000000000002"))')
    v = _run_drc_with_segments([north])
    north_hits = [x for x in v if x.get('type') == 'pad-segment'
                  and 'VBUS' in (x.get('net1'), x.get('net2'))
                  and 'SENSE' in (x.get('net1'), x.get('net2'))]
    assert north_hits, "non-local tie contact must still be flagged"
    # An unrelated net grazing the tie pad is always a violation
    graze = ('(segment (start 9.4 10.5) (end 11.5 10.5) (width 0.127) '
             '(layer "F.Cu") (net 3) (uuid "aaaaaaaa-0000-0000-0000-000000000003"))')
    v = _run_drc_with_segments([graze])
    other_hits = [x for x in v if x.get('type') == 'pad-segment'
                  and 'OTHER' in (x.get('net1'), x.get('net2'))]
    assert other_hits, "unrelated net grazing the tie pad must still be flagged"
    print("  PASS  check_drc waives local tie contact, flags non-local and unrelated")


if __name__ == '__main__':
    test_parser_and_exempt_sets()
    test_obstacle_stamp_lifted()
    test_prepare_restore_lift()
    test_check_drc_exemption()
    print("\n4/4 net-tie tests passed")
