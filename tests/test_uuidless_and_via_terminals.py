#!/usr/bin/env python3
"""PR #534 (adapted): uuid-less copper parses; copper-group vias are terminals.

1. uuid-less segments/vias/arcs must parse -- in BOTH net-token dialects.
   KiCad accepts copper without a (uuid ...) token (it mints one on load), and
   tool-injected copper often omits it. The strict patterns required uuid, so
   such copper was SILENTLY absent from the model: the router planned tracks
   straight through real via barrels. The PR fixed the numeric patterns only;
   the (net "name") KiCad-10 twins and the arc patterns had the same hole
   (the #344/#369 "forgot the v10 twin" class).

2. get_net_endpoints Case 1 (two+ copper groups) must expose each group's
   VIAS as terminals on every routing layer, like Case 2 always has. A group
   whose segments all sit on layers outside --layers (an F.Cu escape stub +
   via from a previous fanout step) otherwise offers only forbidden-layer
   terminals and survives on the rescue ladder alone.

3. End-to-end: the canonical escape-via board routes DIRECTLY through its
   existing vias (no stub-swap rescue, no via-in-pad unblock).

    python3 tests/test_uuidless_and_via_terminals.py
"""
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(ROOT, "rust_router"))

NUMERIC_BOARD = """(kicad_pcb (version 20241229) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "")
  (net 1 "SIG")
  (net 2 "GND")
  (segment (start 10 10) (end 20 10) (width 0.2) (layer "F.Cu") (net 1) (uuid "aaaa"))
  (segment (start 20 10) (end 30 10) (width 0.2) (layer "F.Cu") (net 1))
  (arc (start 30 10) (mid 31 11) (end 32 10) (width 0.2) (layer "F.Cu") (net 1))
  (via (at 30 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (uuid "bbbb"))
  (via (at 15 12) (size 0.25) (drill 0.15) (layers "F.Cu" "B.Cu") (net 2))
)
"""

V10_BOARD = """(kicad_pcb (version 20241229) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "")
  (net 1 "SIG")
  (segment (start 10 10) (end 20 10) (width 0.2) (layer "F.Cu") (net "SIG") (uuid "aaaa"))
  (segment (start 20 10) (end 30 10) (width 0.2) (layer "F.Cu") (net "SIG"))
  (arc (start 30 10) (mid 31 11) (end 32 10) (width 0.2) (layer "F.Cu") (net "SIG"))
  (via (at 30 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "SIG") (uuid "bbbb"))
  (via (at 15 12) (size 0.25) (drill 0.15) (layers "F.Cu" "B.Cu") (net "SIG"))
)
"""

TSTAMP_BOARD = """(kicad_pcb (version 20211014) (generator pcbnew)
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (net 0 "")
  (net 1 "SIG")
  (segment (start 10 10) (end 20 10) (width 0.2) (layer "F.Cu") (net 1) (tstamp 8b1c96e2))
  (via (at 30 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (tstamp 9c2d97f3))
)
"""

ESCAPE_BOARD = """(kicad_pcb (version 20241229) (generator "test") (generator_version "9.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
  (layers
    (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal)
    (31 "B.Cu" signal) (44 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "SIG")
  (footprint "test:R" (layer "F.Cu") (uuid "f1") (at 10 10)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "SIG") (uuid "p1"))
  )
  (footprint "test:R" (layer "F.Cu") (uuid "f2") (at 30 10)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "SIG") (uuid "p2"))
  )
  (segment (start 10 10) (end 12 10) (width 0.2) (layer "F.Cu") (net 1) (uuid "s1"))
  (segment (start 30 10) (end 28 10) (width 0.2) (layer "F.Cu") (net 1) (uuid "s2"))
  (via (at 12 10) (size 0.5) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (uuid "v1"))
  (via (at 28 10) (size 0.5) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (uuid "v2"))
  (gr_rect (start 5 5) (end 35 15) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)) (uuid "e1"))
)
"""


def _parse(text):
    from kicad_parser import parse_kicad_pcb
    with tempfile.NamedTemporaryFile("w", suffix=".kicad_pcb",
                                     delete=False) as f:
        f.write(text)
        path = f.name
    try:
        return parse_kicad_pcb(path)
    finally:
        os.unlink(path)


def test_uuidless_parses():
    for label, text in (("numeric", NUMERIC_BOARD), ("v10", V10_BOARD)):
        pcb = _parse(text)
        # 2 straight segments + the linearized arc's pieces (>= 1)
        straight = [s for s in pcb.segments
                    if (s.start_x, s.start_y) in ((10.0, 10.0), (20.0, 10.0))]
        assert len(straight) == 2, \
            f"{label}: expected both straight segments, got {len(straight)}"
        uuidless_seg = [s for s in straight if s.start_x == 20.0]
        assert uuidless_seg and uuidless_seg[0].uuid == "", label
        arc_pieces = [s for s in pcb.segments if s not in straight]
        assert arc_pieces, f"{label}: uuid-less arc track vanished"
        assert len(pcb.vias) == 2, \
            f"{label}: expected 2 vias (1 uuid-less), got {len(pcb.vias)}"
        assert sorted(v.uuid for v in pcb.vias) == ["", "bbbb"], label
        print(f"  parse[{label}]: uuid-less segment, arc and via all present")

    # KiCad 6/7 wrote (tstamp ...) where later versions write (uuid ...).
    # The required-uuid patterns dropped that era's copper WHOLESALE (the
    # whole routed board invisible: no obstacles, no connectivity credit);
    # optional uuid repairs the dialect as a side effect.
    pcb = _parse(TSTAMP_BOARD)
    assert len(pcb.segments) == 1 and len(pcb.vias) == 1, \
        f"tstamp-era copper must parse, got {len(pcb.segments)}/{len(pcb.vias)}"
    print("  parse[tstamp]: KiCad-6/7 (tstamp) copper parses")


def test_case1_group_vias():
    from connectivity import get_net_endpoints

    def _seg(x1, y1, x2, y2):
        return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                               width=0.1, layer="F.Cu", net_id=5)

    def _via(x, y):
        return SimpleNamespace(x=x, y=y, size=0.25, drill=0.15, net_id=5,
                               layers=["F.Cu", "B.Cu"])

    def _pad(x, y, ref):
        return SimpleNamespace(global_x=x, global_y=y, net_id=5,
                               pad_type="smd", size_x=0.2, size_y=0.7,
                               drill=0.0, layers=["F.Cu"], component_ref=ref,
                               pad_number="1", shape="rect", rect_rotation=0.0,
                               roundrect_rratio=0.0, polygons=None)

    pads = [_pad(10, 10, "J1"), _pad(40, 40, "R2")]
    segs = [_seg(10, 10, 12, 10), _seg(40, 40, 38, 40)]
    vias = [_via(38, 40), _via(12, 10)]  # deliberately unsorted board order
    net = SimpleNamespace(name="THERM", net_id=5, pads=pads)
    pcb = SimpleNamespace(nets={5: net}, pads_by_net={5: pads},
                          segments=segs, vias=vias, zones=[], footprints=[],
                          board_outlines=[], board_info=None)
    cfg = SimpleNamespace(layers=["In2.Cu", "In3.Cu"], grid_step=0.05)

    sources, targets, err = get_net_endpoints(pcb, 5, cfg)
    assert err is None, err
    assert sources and targets, \
        "via-anchored groups must produce terminals on inner layers"
    src_xy = {(round(s[3], 2), round(s[4], 2)) for s in sources}
    tgt_xy = {(round(t[3], 2), round(t[4], 2)) for t in targets}
    assert src_xy in ({(12.0, 10.0)}, {(38.0, 40.0)}), src_xy
    assert tgt_xy in ({(12.0, 10.0)}, {(38.0, 40.0)}) and tgt_xy != src_xy
    layer_idxs = {s[2] for s in sources} | {t[2] for t in targets}
    assert layer_idxs == {0, 1}, \
        f"via terminals must exist on every routing layer, got {layer_idxs}"
    print("  case1: group vias serve as terminals on all routing layers")


def test_escape_via_routes_directly():
    from kicad_parser import parse_kicad_pcb
    tmp = tempfile.mkdtemp(prefix="via_term_")
    board = os.path.join(tmp, "escape.kicad_pcb")
    out = os.path.join(tmp, "routed.kicad_pcb")
    with open(board, "w") as f:
        f.write(ESCAPE_BOARD)
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'route.py'),
                        board, out, "--nets", "SIG",
                        "--layers", "In1.Cu", "In2.Cu"],
                       capture_output=True, text=True, cwd=ROOT)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert '"failed": 0' in r.stdout, "SIG failed to route"
    # The direct route through the existing escape vias, not the rescue
    # ladder (pre-fix: STUB-SWAP RESCUE, 18 segments, 2 new via-in-pads).
    assert "RESCUE" not in r.stdout, "route should not need the rescue ladder"
    pcb = parse_kicad_pcb(out)
    kept = {v.uuid for v in pcb.vias}
    assert {"v1", "v2"} <= kept, "existing escape vias must survive"
    # And NO via of its own. The upfront layer swap used to decide this net
    # "needs a via" by comparing sources[0]'s layer to targets[0]'s -- which in
    # full-stack mode were the F.Cu PAD and an In1.Cu VIA terminal, though the
    # two terminal SETS were identical and both escape vias already reached
    # every routing layer. It then moved the source stub off F.Cu and minted a
    # via IN the pad, orphaning v1; the #622 stub-debris trim later collected
    # that orphan as dangling, which is how this surfaced as "v1 vanished".
    # The other end survived only because targets[0] happened to sort to a via
    # terminal rather than a pad. The verdict now compares the ROUTABLE layer
    # sets, so neither end is swapped and no via is minted.
    assert kept == {"v1", "v2"}, \
        (f"the route must use the existing escape vias, not mint its own: "
         f"expected exactly v1+v2, got {sorted(kept)}")
    assert len(pcb.segments) <= 8, \
        f"expected a direct route, got {len(pcb.segments)} segments"
    on_f = [s for s in pcb.segments if s.layer == "F.Cu"]
    assert len(on_f) == 2, \
        (f"both pad stubs must stay on F.Cu as drawn; got {len(on_f)} F.Cu "
         f"segment(s) -- a stub swapped off F.Cu is the bug above")
    print(f"  e2e: direct route, {len(pcb.segments)} segment(s), "
          f"{len(pcb.vias)} via(s), no rescue, no via minted")


def main():
    test_uuidless_parses()
    test_case1_group_vias()
    test_escape_via_routes_directly()
    print("OK: uuid-less copper parses (both dialects); "
          "Case-1 group vias are terminals")


if __name__ == "__main__":
    main()
