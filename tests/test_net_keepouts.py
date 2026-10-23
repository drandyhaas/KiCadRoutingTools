#!/usr/bin/env python3
"""Net-filtered copper keep-outs read from the board's own .kicad_dru.

A KiCad rule area has no net dimension; a region only some nets may cross is a
NAMED rule area plus a `disallow` rule whose condition names it:

    (rule "corridor" (constraint disallow track via)
      (condition "(A.intersectsArea('corridorA') || A.intersectsArea('corridorB'))
                  && A.NetName != 'Net-(D9-A)'"))

design_rules used to report such a rule NOT honoured and the router laid the
bound nets straight through (KiCad: items_not_allowed). net_keepouts derives
the regions from the rule (no flag, no GUI control: the rules file is the
single source of truth, as for #498) and stamps each while a net it binds
routes.

Fixture: kicad_files/flat_hierarchy.kicad_pcb with two permissive named rule
areas, one straddling Net-(D8-A)'s straight path and one Net-(D9-A)'s, built
per test with the writer's own rule-area emitter; the rule exempts D9. The
diff-pair case uses kicad_files/lvds_converter_dualclk.kicad_pcb's /CLK pair.

Invariants:
1. Derivation: what a rule binds is the resolver's evaluator with the area
   predicates answered per rule area, separately for tracks and vias; a rule
   that is not a union of net-filtered regions, or that names no area of the
   board, stays NOT honoured; an area predicate never reaches the resolver.
2. Stamping balances on every map and rung, an exempt net stamps nothing, a
   rescue window's clipped raster never reaches a whole-board route, and the
   diff-pair paths stamp their own maps (members kept out, not just the
   centreline).
3. End to end, on both CLIs and the GUI-shaped in-memory call: the bound net
   detours, the exempt one crosses, the summary stops listing the rule as
   unsupported, the shipped board is audited (input copper inside a region
   is reported), and KiCad's own DRC agrees (when kicad-cli is installed).

Run:
    python3 tests/test_net_keepouts.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import net_keepouts as nk  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_config import GridCoord, GridRouteConfig  # noqa: E402
from obstacle_map import _polygon_grid_cells  # noqa: E402

BOARD = os.path.join(ROOT, "kicad_files", "flat_hierarchy.kicad_pcb")
LVDS = os.path.join(ROOT, "kicad_files", "lvds_converter_dualclk.kicad_pcb")
GEOM = ["--track-width", "0.5", "--clearance", "0.4",
        "--via-size", "0.6", "--via-drill", "0.5", "--layers", "F.Cu", "B.Cu"]
ENV = dict(os.environ, MSYS2_ARG_CONV_EXCL='*')


class Skip(Exception):
    """A test whose precondition this environment cannot meet."""


def box(x1, y1, x2, y2):
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]


D8_BOX = box(150.6, 76.4, 152.3, 78.5)
D9_BOX = box(150.6, 86.6, 152.3, 88.7)
CLK_BOX = box(86.5, 58.0, 88.5, 60.2)
AREAS = (("corridorA", D8_BOX), ("corridorB", D9_BOX))
CORRIDOR = ('(version 1)\n(rule "corridor"\n\t(constraint disallow track via)\n'
            '\t(condition "(A.intersectsArea(\'corridorA\') || '
            'A.intersectsArea(\'corridorB\')) && A.NetName != \'Net-(D9-A)\'"))\n')


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def _fixture(dru=None, areas=AREAS, board=BOARD, name="b"):
    """A copy of `board` (with its siblings) carrying permissive named rule
    areas, and `dru` as its .kicad_dru when given. Returns the board path."""
    from copy_board import copy_board
    from kicad_writer import generate_rule_area_sexpr
    from kicad_parser import pcb_uses_name_nets
    out = os.path.join(tempfile.mkdtemp(prefix="nko_"), f"{name}.kicad_pcb")
    _quiet(copy_board, board, out)
    with open(out, encoding="utf-8", newline="") as f:
        text = f.read()
    named = pcb_uses_name_nets(_quiet(parse_kicad_pcb, board))
    zones = [generate_rule_area_sexpr(["F.Cu", "B.Cu"], poly, n, not_allowed=(),
                                      use_net_name=named) for n, poly in areas]
    close = text.rstrip().rfind(")")
    text = text[:close].rstrip("\n") + "\n" + "\n".join(zones) + "\n" + text[close:]
    with open(out, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    if dru is not None:
        with open(out[:-len(".kicad_pcb")] + ".kicad_dru", "w", encoding="utf-8") as f:
            f.write(dru)
    return out


def _install(dru, areas=AREAS, routing=True):
    """(pcb_data, DesignRules, report text) for the fixture with `dru`,
    through the install the routing engines run (`routing`), or the one
    every other step runs."""
    from kicad_dru import install_layer_clearances
    path = _fixture(dru, areas)
    pcb = _quiet(parse_kicad_pcb, path)
    cfg = GridRouteConfig(layers=["F.Cu", "B.Cu"])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        install_layer_clearances(cfg, None, path, pcb, net_keepouts=routing)
    return pcb, cfg.rules, buf.getvalue()


def _rule(name, cond, items="track via", extra=""):
    return (f'(version 1)\n(rule "{name}"\n\t(constraint disallow {items}){extra}\n'
            f'\t(condition "{cond}"))\n')


def _region(poly, nets, tracks=True, vias=True, name="r"):
    return {"name": name, "polygon": list(poly), "holes": [], "layers": {"F.Cu", "B.Cu"},
            "tracks_allowed": not tracks, "vias_allowed": not vias,
            "nets": frozenset(nets)}


def _route(board, nets, *extra, tool="route.py"):
    out = os.path.join(tempfile.mkdtemp(prefix="nko_out_"), "o.kicad_pcb")
    geom = GEOM if tool == "route.py" else []
    r = subprocess.run([sys.executable, "-X", "utf8", f"py_router/{tool}", board, out]
                       + geom + ["--nets"] + nets + list(extra),
                       cwd=ROOT, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", env=ENV, timeout=900)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    return out, r.stdout


def _summary(log):
    return json.loads(log.split("JSON_SUMMARY: ", 1)[1].splitlines()[0])


def _cells_in(segs, polys, grid_step=0.1):
    """Grid cells the segments' centre lines cross inside `polys`."""
    from bresenham_utils import walk_line
    coord = GridCoord(grid_step)
    cells = set()
    for s in segs:
        g1, g2 = coord.to_grid(s.start_x, s.start_y), coord.to_grid(s.end_x, s.end_y)
        cells.update(walk_line(g1[0], g1[1], g2[0], g2[1]))
    region = set()
    for p in polys:
        region |= _polygon_grid_cells([tuple(v) for v in p], coord)
    return len(cells & region)


def _inside(board, net_name, polys):
    pcb = _quiet(parse_kicad_pcb, board)
    nid = next(i for i, n in pcb.nets.items() if n.name == net_name)
    segs = [s for s in pcb.segments if s.net_id == nid]
    return _cells_in(segs, polys), bool(segs)


def _connected(board, nets):
    r = subprocess.run([sys.executable, "py_router/check_connected.py", board, "--nets"] + nets,
                       cwd=ROOT, capture_output=True, text=True, env=ENV)
    return "ALL NETS FULLY CONNECTED" in (r.stdout + r.stderr)


def _copper_gap(board, net_name, poly):
    """Smallest distance (mm) from `net_name`'s track copper to polygon
    `poly`'s outline; negative when the copper overlaps the polygon. An
    independent measure: plain segment geometry, no grid, no router code."""
    from geometry_utils import segment_to_segment_distance
    from obstacle_map import point_in_polygon
    pcb = _quiet(parse_kicad_pcb, board)
    nid = next(i for i, n in pcb.nets.items() if n.name == net_name)
    ring = [tuple(v) for v in poly]
    edges = list(zip(ring, ring[1:] + ring[:1]))
    gap = float('inf')
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        if (point_in_polygon(s.start_x, s.start_y, ring)
                or point_in_polygon(s.end_x, s.end_y, ring)):
            return -1.0
        d = min(segment_to_segment_distance(s.start_x, s.start_y, s.end_x, s.end_y,
                                            a[0], a[1], b[0], b[1]) for a, b in edges)
        gap = min(gap, d - s.width / 2)
    return gap


def _kicad_items_not_allowed(board, dru):
    """KiCad's own DRC of `board` graded with `dru` beside it: the number of
    items_not_allowed violations, or None when kicad-cli is not installed."""
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        return None
    with open(board[:-len(".kicad_pcb")] + ".kicad_dru", "w", encoding="utf-8") as f:
        f.write(dru)
    out = board + ".drc.json"
    subprocess.run([cli, "pcb", "drc", "--format", "json", "--severity-all",
                    "--output", out, board], capture_output=True, timeout=600)
    with open(out, encoding="utf-8") as f:
        return sum(v["type"] == "items_not_allowed" for v in json.load(f)["violations"])


# ------------------------------------------------------------- derivation

def test_rule_area_names_are_parsed():
    pcb = _quiet(parse_kicad_pcb, _fixture())
    names = sorted(k["name"] for k in pcb.board_info.keepouts)
    assert names == ["corridorA", "corridorB"], names


def test_corridor_rule_is_honoured():
    pcb, rules, log = _install(CORRIDOR)
    assert "rule 'corridor': honoured as a net-filtered keep-out" in log, log
    assert "NOT honoured" not in log, log
    assert rules.unsupported() == [], rules.unsupported()
    regs = rules.net_keepouts
    assert sorted(r["name"] for r in regs) == ["corridorA", "corridorB"], regs
    all_nets = {n.name for i, n in pcb.nets.items() if i and n.name}
    for r in regs:
        assert r["nets"] == frozenset(all_nets - {"Net-(D9-A)"}), len(r["nets"])
        assert not r["tracks_allowed"] and not r["vias_allowed"]
        assert r["layers"] == {"F.Cu", "B.Cu"}


def test_other_steps_still_report_it_not_honoured():
    """A step that does not stamp keep-outs (planes, fanout) derives none and
    keeps reporting the rule NOT honoured -- saying who does read it."""
    _, rules, log = _install(CORRIDOR, routing=False)
    assert rules.net_keepouts == [], rules.net_keepouts
    (name, why), = rules.unsupported()
    assert name == "corridor" and "route.py and route_diff.py read it" in why, why
    assert "NOT honoured" in log and "honoured as" not in log, log


def test_tracks_and_vias_bind_separately():
    """`disallow track` alone forbids no via; a condition reading A.Type binds
    tracks and vias to different nets, as KiCad evaluates it per item."""
    _, rules, _ = _install(_rule("t", "A.intersectsArea('corridorA')", items="track"))
    (r,) = rules.net_keepouts
    assert not r["tracks_allowed"] and r["vias_allowed"], r
    _, rules, _ = _install(_rule(
        "tv", "A.intersectsArea('corridorA') && (A.Type == 'Via' || A.NetName == 'Net-(D8-A)')"))
    by_kind = {(r["tracks_allowed"], r["vias_allowed"]): r["nets"] for r in rules.net_keepouts}
    assert by_kind.get((False, True)) == frozenset({"Net-(D8-A)"}), by_kind
    vias = by_kind.get((True, False), frozenset())
    assert "Net-(D8-A)" in vias and len(vias) > 100, len(vias)
    assert len(rules.net_keepouts) == 2, "a tracks region (D8) and a vias region (all)"


def test_layer_clause_and_netclass_condition():
    _, rules, _ = _install(_rule("f", "A.intersectsArea('corridorA') && A.NetClass == 'Default'",
                                 extra="\n\t(layer F.Cu)"))
    (r,) = rules.net_keepouts
    assert r["layers"] == {"F.Cu"} and len(r["nets"]) > 100, (r["layers"], len(r["nets"]))


def test_rules_it_cannot_model_stay_not_honoured():
    """Nothing of a rule is applied unless all of it can be: each of these is
    reported NOT honoured, with a reason, and derives no region."""
    cases = {
        "outside the area": (_rule("neg", "!A.intersectsArea('corridorA') && "
                                          "A.NetName == 'Net-(D8-A)'"), "outside every named area"),
        "an intersection": (_rule("and", "A.intersectsArea('corridorA') && "
                                         "A.intersectsArea('corridorB')"), "other than as a union"),
        "no such area": (_rule("gone", "A.intersectsArea('nowhere')"), "does not have"),
        "not routed copper": (_rule("fp", "A.intersectsArea('corridorA')", items="footprint"),
                              "nothing the router lays"),
        "another constraint": ('(version 1)\n(rule "mix"\n\t(constraint disallow track)\n'
                               '\t(constraint clearance (min 1mm))\n'
                               '\t(condition "A.intersectsArea(\'corridorA\')"))\n',
                               "intersectsArea() not modelled"),
    }
    for what, (dru, why) in cases.items():
        _, rules, log = _install(dru)
        (_name, reason), = rules.unsupported()
        assert why in reason and "NOT honoured" in log, (what, reason)
        assert rules.net_keepouts == [], (what, rules.net_keepouts)


def test_one_missing_area_is_disclosed_and_enclosed_is_conservative():
    _, rules, log = _install(_rule("half", "A.intersectsArea('corridorA') || "
                                           "A.enclosedByArea('nowhere')"))
    assert [r["name"] for r in rules.net_keepouts] == ["corridorA"]
    assert "'nowhere', which the board does not have" in log, log
    assert "enclosedByArea read as intersectsArea" in log, log


def test_the_resolver_never_sees_an_area_predicate():
    """A clearance rule on `!A.intersectsArea()` stays unsupported: answering
    the predicate False everywhere would make it match every item."""
    from design_rules import parse_condition, Unsupported
    try:
        parse_condition("A.intersectsArea('x')")
    except Unsupported:
        pass
    else:
        raise AssertionError("the resolver's parse must refuse an area predicate")
    pcb, rules, _ = _install('(version 1)\n(rule "wide"\n\t(constraint clearance (min 2mm))\n'
                             '\t(condition "!A.intersectsArea(\'corridorA\')"))\n')
    d8 = next(i for i, n in pcb.nets.items() if n.name == "Net-(D8-A)")
    d9 = next(i for i, n in pcb.nets.items() if n.name == "Net-(D9-A)")
    c = rules.resolve('clearance', rules.item_for_net(d8), rules.item_for_net(d9), 'F.Cu')
    assert c.min is None or c.min < 2.0, c
    assert [n for n, _ in rules.unsupported()] == ["wide"], rules.unsupported()


# ------------------------------------------------------------- stamping

class _Map:
    """Refcounted stand-in for GridObstacleMap's cell/via batches."""

    def __init__(self):
        self.cells, self.vias = {}, {}

    def _bump(self, d, arr, k):
        for row in map(tuple, arr.tolist()):
            d[row] = d.get(row, 0) + k
            if d[row] == 0:
                del d[row]

    def add_blocked_cells_batch(self, a): self._bump(self.cells, a, 1)
    def remove_blocked_cells_batch(self, a): self._bump(self.cells, a, -1)
    def add_blocked_vias_batch(self, a): self._bump(self.vias, a, 1)
    def remove_blocked_vias_batch(self, a): self._bump(self.vias, a, -1)


def _board_ids():
    pcb = _quiet(parse_kicad_pcb, BOARD)
    d8 = next(i for i, n in pcb.nets.items() if n.name == "Net-(D8-A)")
    d9 = next(i for i, n in pcb.nets.items() if n.name == "Net-(D9-A)")
    return pcb, d8, d9


def _cfg():
    cfg = GridRouteConfig()
    cfg.layers = ["F.Cu", "B.Cu"]
    return cfg


def test_stamp_balances_and_skips_exempt_net():
    pcb, d8, d9 = _board_ids()
    nk.attach(pcb, [_region(D8_BOX, {"Net-(D8-A)"})])
    m = _Map()
    with nk.stamp(m, pcb, (d8,), _cfg()):
        assert m.cells and m.vias, "bound net: region must be stamped"
    assert not m.cells and not m.vias, "stamp must remove exactly what it added"
    with nk.stamp(m, pcb, (d9,), _cfg()):
        assert not m.cells and not m.vias, "exempt net: nothing stamped"


def test_attach_none_clears_a_previous_run():
    """The GUI hands every run the same PCBData: a run that derives no region
    must not keep stamping the previous run's."""
    pcb, d8, _ = _board_ids()
    nk.attach(pcb, [_region(D8_BOX, {"Net-(D8-A)"})])
    m = _Map()
    with nk.stamp(m, pcb, (d8,), _cfg()):
        assert m.cells, "control: the attached region stamps"
    nk.attach(pcb, None)
    with nk.stamp(m, pcb, (d8,), _cfg()):
        assert not m.cells and not m.vias, "a cleared run must stamp nothing"


def test_window_cells_do_not_leak_to_the_whole_board():
    """A rescue window is a shallow copy sharing the whole board's cell cache;
    its clipped raster must not be served to a whole-board route."""
    from plane_pad_tap import make_local_window
    pcb, _, _ = _board_ids()
    nk.attach(pcb, [_region(D8_BOX, {"Net-(D8-A)"})])
    window = make_local_window(pcb, 150.6, 77.45, 0.6)
    assert window.__dict__.get('_net_keepout_cells') is pcb._net_keepout_cells, (
        "precondition: the window shares the board's cache dict")
    w_cells, _ = nk._cells_for(window, (0,), _cfg())
    b_cells, _ = nk._cells_for(pcb, (0,), _cfg())
    assert 0 < len(w_cells) < len(b_cells), (len(w_cells), len(b_cells))


def test_diff_pair_paths_stamp_their_maps():
    """The diff-pair A* paths outside the standard pair route see the region
    too: the terminal connectors' base map, the hybrid's coupled middle (a
    pair: bound if either member is) and its single-net legs (bound by their
    own net). Each body is replaced by a probe that reads the maps it was
    handed; every map is clean again afterwards."""
    from types import SimpleNamespace
    import diff_pair_routing as dpr
    pcb, d8, d9 = _board_ids()
    nk.attach(pcb, [_region(D8_BOX, {"Net-(D8-A)"})])
    cfg = _cfg()
    seen = {}

    def probe(tag, *maps):
        def body(*a, **k):
            seen[tag] = tuple(bool(m.cells) for m in maps)
        return body

    leg_args = (None,) * 7    # layer_names .. pair_vias: the probe reads none
    work, base = _Map(), _Map()
    real = (dpr._route_hybrid_leg.__wrapped__,
            dpr._route_direct_coupled_middle.__wrapped__,
            dpr._route_diff_pair_with_obstacles_impl)
    try:
        dpr._route_hybrid_leg.__wrapped__ = probe('leg', work)
        dpr._route_hybrid_leg(pcb, d8, cfg, work, *leg_args)
        assert seen.pop('leg') == (True,), "a bound net's leg map is stamped"
        dpr._route_hybrid_leg(pcb, d9, cfg, work, *leg_args)
        assert seen.pop('leg') == (False,), "an exempt net's leg is not"

        mixed = SimpleNamespace(p_net_id=d9, n_net_id=d8)
        dpr._route_direct_coupled_middle.__wrapped__ = probe('mid', work)
        dpr._route_direct_coupled_middle(pcb, mixed, cfg, work, None)
        assert seen.pop('mid') == (True,), "pair bound through one member"

        dpr._route_diff_pair_with_obstacles_impl = probe('pair', work, base)
        dpr.route_diff_pair_with_obstacles(pcb, mixed, cfg, work, base, None)
        assert seen.pop('pair') == (True, True), (
            "the pair map and the connectors' base map are both stamped")
    finally:
        (dpr._route_hybrid_leg.__wrapped__,
         dpr._route_direct_coupled_middle.__wrapped__,
         dpr._route_diff_pair_with_obstacles_impl) = real
    assert not (work.cells or work.vias or base.cells or base.vias), (
        "every stamp is removed on exit")


def test_stamp_reaches_every_via_rung_at_its_own_via():
    """On a real map with a #568 small rung and a #530 per-net rung, the region
    blocks vias on every rung while the bound net routes -- the per-net rung
    at ITS via (a cell 0.55 mm off the edge is clear of a 0.6 via's band and
    inside a 0.9 via's) -- and every rung is back as it was afterwards."""
    from obstacle_map import build_base_obstacle_map
    pcb, d8, _ = _board_ids()
    cfg = GridRouteConfig(layers=["F.Cu", "B.Cu"], track_width=0.25, clearance=0.2,
                          grid_step=0.1, via_size=0.6, via_drill=0.3,
                          net_via_sizes={d8: (0.9, 0.45)})
    work = _quiet(build_base_obstacle_map, pcb, cfg, [d8]).clone_fresh()
    n = work.rung_count()
    if n < 3:
        raise Skip(f"map carries {n} rung(s); needs the small and a per-net rung")
    snap = lambda: [sorted(work.blocked_via_cells_at_rung(r)) for r in range(n)]
    before = snap()
    nk.attach(pcb, [_region(D8_BOX, {"Net-(D8-A)"})])
    coord = GridCoord(cfg.grid_step)
    centre = coord.to_grid(151.4, 77.45)
    band = coord.to_grid(150.6 - 0.55, 77.45)
    with nk.stamp(work, pcb, (d8,), cfg):
        assert all(work.is_via_blocked_rung(*centre, r) for r in range(n))
        assert not work.is_via_blocked_rung(*band, 0)
        assert work.is_via_blocked_rung(*band, n - 1), "per-net rung stamped at the run's via"
    assert snap() == before, "stamp must leave every rung as it found it"


def test_audit_finds_bound_copper_inside_a_region():
    """audit() is plain geometry: a bound net's track crossing the region or
    a via inside it is reported; a track merely near it, an exempt net, a
    track on a layer the region is not on, and copper in a hole are not."""
    from types import SimpleNamespace as NS
    pcb, d8, d9 = _board_ids()
    ko = _region(D8_BOX, {"Net-(D8-A)"})
    ko['layers'] = {'F.Cu'}

    def seg(nid, y, layer='F.Cu', w=0.2, x0=149.0, x1=154.0):
        return NS(start_x=x0, start_y=y, end_x=x1, end_y=y, width=w,
                  layer=layer, net_id=nid, graphic=False)

    segs = [seg(d8, 77.0),               # crosses the box
            seg(d8, 76.2),               # copper edge 0.1 mm short of the top edge
            seg(d8, 77.0, 'B.Cu'),       # the region is F.Cu only
            seg(d9, 77.0)]               # exempt
    vias = [NS(x=151.4, y=77.4, size=0.6, net_id=d8, layers=['F.Cu', 'B.Cu']),
            NS(x=151.4, y=77.4, size=0.6, net_id=d9, layers=['F.Cu', 'B.Cu'])]
    got = nk.audit(pcb, segs, vias, keepouts=[ko])
    assert got == [{'region': 'r', 'net': 'Net-(D8-A)', 'tracks': 1, 'vias': 1}], got
    assert nk.audit(pcb, segs[1:3], [], keepouts=[ko]) == [], "near or off-layer is clean"
    holed = dict(ko, holes=[box(150.9, 76.8, 152.0, 78.1)])
    assert nk.audit(pcb, [seg(d8, 77.45, x0=151.2, x1=151.7)], [], keepouts=[holed]) == [], (
        "copper inside a hole of the region is outside it")
    nk.attach(pcb, None)
    assert nk.audit(pcb, segs, vias) is None, "no keep-out: no report at all"


# ------------------------------------------------------------- end to end

def test_route_honours_the_rule():
    """route.py: without the .kicad_dru D8 cuts through its box (and KiCad
    flags it); with it D8 detours and connects, D9 (exempt) still crosses,
    the summary no longer lists the rule as unsupported, --json-out reports
    a clean audit, and KiCad's DRC finds nothing."""
    nets = ["Net-(D8-A)", "Net-(D9-A)"]
    ctrl, _ = _route(_fixture(None), nets)
    inside_ctrl, _ = _inside(ctrl, "Net-(D8-A)", [D8_BOX])
    assert inside_ctrl > 0, "control: without the rule D8 must cut through its box"

    board = _fixture(CORRIDOR)
    js = os.path.join(tempfile.mkdtemp(prefix="nko_"), "o.json")
    out, log = _route(board, nets, "--json-out", js)
    assert "rule 'corridor': honoured as a net-filtered keep-out" in log
    assert "corridor" not in _summary(log)["design_rules"]["unsupported_rules"]
    d8_in, d8_has = _inside(out, "Net-(D8-A)", [D8_BOX, D9_BOX])
    d9_in, _ = _inside(out, "Net-(D9-A)", [D9_BOX])
    assert d8_has and _connected(out, ["Net-(D8-A)"])
    assert d8_in == 0, f"bound net entered the keep-out ({d8_in} cells)"
    assert d9_in > 0, "exempt net must still route through its region"
    with open(js, encoding="utf-8") as f:
        assert json.load(f).get("net_keepouts") == []
    k_ctrl = _kicad_items_not_allowed(ctrl, CORRIDOR)
    if k_ctrl is not None:
        k_out = _kicad_items_not_allowed(out, CORRIDOR)
        assert (k_ctrl, k_out) == (1, 0), (k_ctrl, k_out)


def test_route_diff_member_copper_stays_out():
    """route_diff, a rule binding ONE member of /CLK: the pair detours as a
    pair, and neither member's COPPER touches the region. Stamped at the
    plain run geometry the centreline kept its band while /CLK+, (w+gap)/2
    to the side, ended with its copper 0.5 um over the region's edge."""
    dru = _rule("mid_clk", "A.intersectsArea('mid') && A.NetName == '/CLK+'")
    gaps = {}
    for arm, rules in (("off", None), ("on", dru)):
        board = _fixture(rules, areas=(("mid", CLK_BOX),), board=LVDS, name=arm)
        out, log = _route(board, ["/CLK+", "/CLK-"], tool="route_diff.py")
        assert _summary(log)["routed_diff_pairs"] == ["/CLK"], (arm, _summary(log))
        gaps[arm] = [_copper_gap(out, n, CLK_BOX) for n in ("/CLK+", "/CLK-")]
        if arm == "on":
            assert "rule 'mid_clk': honoured" in log, log[-3000:]
            k = _kicad_items_not_allowed(out, dru)
            assert k in (None, 0), k
    assert min(gaps["off"]) < 0, f"control: the pair must cross the region {gaps}"
    assert min(gaps["on"]) > 0.05, f"a member's copper reached the region {gaps}"


def test_gui_shaped_call_and_forwarded_regions():
    """The GUI's in-memory call (return_results, its own long-lived PCBData)
    reads the same rule; the same PCBData then routed from a board without
    the rule carries no stale region; and a nested run handed its parent's
    resolved regions (the internal net_keepouts kwarg, as #498 forwards its
    map) honours them on a board whose own files have no rule."""
    from route import batch_route
    kw = dict(layers=["F.Cu", "B.Cu"], track_width=0.5, clearance=0.4,
              via_size=0.6, via_drill=0.5, return_results=True)

    def d8_new(res):
        segs = [s for r in res[3].get("results", []) for s in r.get("new_segments") or []]
        return _cells_in(segs, [D8_BOX]), bool(segs)

    board = _fixture(CORRIDOR)
    pcb = _quiet(parse_kicad_pcb, board)
    res = _quiet(batch_route, board, "", ["Net-(D8-A)"], pcb_data=pcb, **kw)
    assert res[3].get("net_keepouts") == [], res[3].get("net_keepouts")
    assert d8_new(res) == (0, True), d8_new(res)

    bare = _fixture(None)
    _quiet(batch_route, bare, "", ["Net-(D9-A)"], pcb_data=pcb, **kw)
    assert not getattr(pcb, "_net_keepouts", None), "a run with no rule keeps no region"

    pcb2 = _quiet(parse_kicad_pcb, bare)
    res = _quiet(batch_route, bare, "", ["Net-(D8-A)"], pcb_data=pcb2,
                 net_keepouts=[_region(D8_BOX, {"Net-(D8-A)"}, name="corridorA")], **kw)
    assert d8_new(res) == (0, True), d8_new(res)


def test_shipped_board_audit_reports_input_copper():
    """Copper the router did not lay is still reported: on a board whose
    Net-(D8-A) already crosses the region, a run routing ANOTHER net says so
    -- the CLI in its --json-out, the GUI's in-memory call in its results --
    with the same row, the one KiCad's DRC flags."""
    from route import batch_route
    from copy_board import copy_board
    ctrl, _ = _route(_fixture(None), ["Net-(D8-A)"])
    board = os.path.join(tempfile.mkdtemp(prefix="nko_"), "c.kicad_pcb")
    _quiet(copy_board, ctrl, board)
    with open(board[:-len(".kicad_pcb")] + ".kicad_dru", "w", encoding="utf-8") as f:
        f.write(CORRIDOR)
    want = [{'region': 'corridorA', 'net': 'Net-(D8-A)', 'tracks': 1, 'vias': 0}]
    js = os.path.join(os.path.dirname(board), "o.json")
    _, log = _route(board, ["Net-(D10-A)"], "--json-out", js)
    assert "bound-net item(s) inside a region on the shipped board" in log, log[-2000:]
    with open(js, encoding="utf-8") as f:
        assert json.load(f).get("net_keepouts") == want
    pcb = _quiet(parse_kicad_pcb, board)
    res = _quiet(batch_route, board, "", ["Net-(D10-A)"], layers=["F.Cu", "B.Cu"],
                 track_width=0.5, clearance=0.4, via_size=0.6, via_drill=0.5,
                 return_results=True, pcb_data=pcb)
    assert res[3].get("net_keepouts") == want, res[3].get("net_keepouts")


TESTS = [
    test_rule_area_names_are_parsed,
    test_corridor_rule_is_honoured,
    test_other_steps_still_report_it_not_honoured,
    test_tracks_and_vias_bind_separately,
    test_layer_clause_and_netclass_condition,
    test_rules_it_cannot_model_stay_not_honoured,
    test_one_missing_area_is_disclosed_and_enclosed_is_conservative,
    test_the_resolver_never_sees_an_area_predicate,
    test_stamp_balances_and_skips_exempt_net,
    test_attach_none_clears_a_previous_run,
    test_window_cells_do_not_leak_to_the_whole_board,
    test_diff_pair_paths_stamp_their_maps,
    test_stamp_reaches_every_via_rung_at_its_own_via,
    test_audit_finds_bound_copper_inside_a_region,
    test_route_honours_the_rule,
    test_route_diff_member_copper_stays_out,
    test_gui_shaped_call_and_forwarded_regions,
    test_shipped_board_audit_reports_input_copper,
]


def main():
    failed = 0
    for t in TESTS:
        try:
            t()
        except Skip as e:
            print(f"  SKIP: {t.__name__}: {e}")
            continue
        except AssertionError as e:
            failed += 1
            print(f"  FAIL: {t.__name__}: {e}")
            continue
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"  ERROR: {t.__name__}: {type(e).__name__}: {e}")
            continue
        print(f"  PASS: {t.__name__}")
    print(f"{len(TESTS) - failed}/{len(TESTS)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
