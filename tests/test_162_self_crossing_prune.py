#!/usr/bin/env python3
"""#162: same-net SELF-CROSSINGS are resolved by DELETION, never by geometry.

`fix_self_intersections` was deleted in #159 (it "fixed" a crossing by
EXTENDING a segment to a far off-grid point, manufacturing long
non-orthonormal diagonals across foreign copper). Nothing replaced it, so a
same-net X survived every cleanup pass -- `prune_redundant_cycles` builds its
node set from segment ENDPOINT clusters and an X has no endpoint at the
intersection, so the loop it closes is invisible to the tree invariant -- and
`check_drc` only warns about it.

`prune_self_crossing_segments` closes that hole DELETE-ONLY: it drops one arm
of the X when that arm is provably redundant copper, and leaves the crossing
alone otherwise. No endpoint ever moves.

What this pins:
  1. the pass exists and fires (synthetic POS + the two tracked in-repo boards
     that carry the defect);
  2. the DELETE-ONLY invariant -- every surviving segment is the SAME object
     with the same geometry; the pass may only shorten the segment list;
  3. the over-removal guards: an X whose both arms are load-bearing is left
     alone (NEG), the #319 soft-joint guard is live (SJ), a net carrying
     KiCad-locked copper is never touched (#521), and a pad-less / unassigned
     net -- where every connectivity gate would pass vacuously -- is skipped;
  4. the pipeline wiring -- the fix arrives via the SHARED
     `run_post_route_cleanup`, so both fronts (CLI and GUI) get it;
  5. THE SHIPPED DELTA, measured the way production runs it: the real boards
     are A/B'd through `run_post_route_cleanup` with the pass stubbed out vs
     live. That is deliberately NOT the same as calling the pass directly on a
     raw board -- on `routed_output` the arm this pass would take is ALSO
     reachable by `collapse_strict_redundant`, so the shipped copper there is
     identical with and without it, and only `rp2350` sees a real delta.

    python3 tests/test_162_self_crossing_prune.py
"""
import collections
import io
import contextlib
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(REPO, 'tests'))

from kicad_parser import BoardInfo, parse_kicad_pcb
from routing_config import GridRouteConfig
from check_drc import segments_cross
from check_connected import check_net_connectivity
import cleanup_pipeline
from cleanup_pipeline import run_post_route_cleanup
from pcb_modification import _soft_joint_pairs

try:
    from pcb_modification import _via_support_model
except ImportError:  # pre-#162 tree
    _via_support_model = None
from synth import make_pad, make_seg, make_via, make_pcb

try:
    from pcb_modification import prune_self_crossing_segments
except ImportError:  # pre-#162 tree
    prune_self_crossing_segments = None

NET = 7

fails = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


def _seg(x1, y1, x2, y2, w=0.2, layer='F.Cu'):
    return make_seg(x1, y1, x2, y2, width=w, layer=layer, net_id=NET)


def _pad(ref, x, y, size=0.4):
    return make_pad(NET, x, y, ref=ref, size_x=size, size_y=size, net_name='N')


def _board(segs, pads):
    return make_pcb(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                         board_bounds=None),
                    segments=list(segs), pads_by_net={NET: list(pads)})


def _sig(s):
    """A segment's full geometric signature -- what DELETE-ONLY must preserve."""
    return (round(s.start_x, 9), round(s.start_y, 9), round(s.end_x, 9),
            round(s.end_y, 9), round(s.width, 9), s.layer, s.net_id)


def n_crossings(segs):
    """check_drc's own same-net self-crossing predicate, counted pairwise."""
    n = 0
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            if segs[i].net_id == segs[j].net_id and segments_cross(segs[i], segs[j])[0]:
                n += 1
    return n


def _cfg():
    return GridRouteConfig(grid_step=0.1, clearance=0.15, via_size=0.6,
                           track_width=0.2, via_drill=0.3,
                           layers=['F.Cu', 'B.Cu'])


# --------------------------------------------------------------------------
# Fixture POS: a short redundant link L crossed by a near-collinear connector
# D. This is the shape measured on the real boards: D's copper covers both of
# L's endpoints, so L adds nothing -- but the endpoint graph sees L as a
# BRIDGE (removing it splits n(1.0,0) from n(1.5,0)), which is exactly why
# prune_redundant_cycles keeps it.
#
#   pad P1 --T1-- (1.0,0) --L-- (1.5,0) --T2-- pad P2
#           D: (0.9,-0.1) ------------> (1.6,0.1)   crosses L at (1.25, 0)
# --------------------------------------------------------------------------
def pos_fixture():
    pads = [_pad('P1', 0, 0), _pad('P2', 3, 0),
            _pad('P3', 0.9, -0.1, size=0.1), _pad('P4', 1.6, 0.1, size=0.1)]
    t1 = _seg(0, 0, 1.0, 0)
    link = _seg(1.0, 0, 1.5, 0)
    t2 = _seg(1.5, 0, 3.0, 0)
    conn = _seg(0.9, -0.1, 1.6, 0.1)
    return [t1, link, t2, conn], pads, link


# --------------------------------------------------------------------------
# Fixture NEG: an X whose BOTH arms are the sole bridge to a pad pair. The
# net is connected (via the side link), so the connectivity gate -- not the
# "already broken, don't touch" early-out -- is what must refuse both arms.
# --------------------------------------------------------------------------
def neg_fixture():
    pads = [_pad('N1', 0, 0), _pad('N2', 2, 2), _pad('N3', 2, 0), _pad('N4', 0, 2)]
    a = _seg(0, 0, 2, 2)
    b = _seg(2, 0, 0, 2)
    side = _seg(2, 2, 2, 0)
    return [a, b, side], pads


# --------------------------------------------------------------------------
# Fixture SJ: the SHORTER arm (a 100um link, tried first) is connectivity-safe
# to remove but its removal leaves two cap-overlapping dangles -- a soft joint
# the #319 guard must refuse. The longer arm is a sole bridge to two pads.
# Net effect: NOTHING is removed and the crossing ships, which is the correct
# delete-only answer.
# --------------------------------------------------------------------------
def sj_fixture():
    pads = [_pad('R1', 0, 0), _pad('R2', 3, 0),
            _pad('R3', 1.0, -1.0), _pad('R4', 1.1, 1.0)]
    w1 = _seg(0, 0, 1.0, 0)
    link = _seg(1.0, 0, 1.1, 0)
    w3 = _seg(1.1, 0, 3.0, 0)
    stem = _seg(1.0, -1.0, 1.1, 1.0)
    return [w1, link, w3, stem], pads, link, stem


def run_synthetic():
    print("Synthetic fixtures (direct pass)")
    if prune_self_crossing_segments is None:
        check("pcb_modification.prune_self_crossing_segments exists", False,
              "ImportError -- nothing in the cleanup pipeline resolves a "
              "same-net X (#162)")
        return
    check("pcb_modification.prune_self_crossing_segments exists", True)

    # --- POS -------------------------------------------------------------
    segs, pads, link = pos_fixture()
    before_sigs = [_sig(s) for s in segs]
    pcb = _board(segs, pads)
    check("POS: fixture really carries one same-net self-crossing",
          n_crossings(pcb.segments) == 1, f"crossings={n_crossings(pcb.segments)}")
    n, nets, strip = prune_self_crossing_segments([], pcb, {NET})
    check("POS: one segment removed", n == 1, f"removed={n} nets={nets}")
    check("POS: the crossing is gone", n_crossings(pcb.segments) == 0,
          f"crossings={n_crossings(pcb.segments)}")
    check("POS: exactly the redundant link was dropped",
          not any(s is link for s in pcb.segments) and len(pcb.segments) == 3)
    check("POS: the removal is reported for the writer's strip list",
          [id(s) for s in strip] == [id(link)], f"strip={len(strip)}")
    c = check_net_connectivity(NET, pcb.segments, pcb.vias, pads)
    check("POS: net still connected, no pad stranded",
          c.get('connected') and (c.get('num_components') or 1) == 1
          and not (c.get('disconnected_pads') or []),
          f"connected={c.get('connected')} comps={c.get('num_components')} "
          f"disc={len(c.get('disconnected_pads') or [])}")
    check("POS: no soft joint manufactured",
          len(_soft_joint_pairs(pcb.segments, pcb.vias, pads)) == 0)
    # DELETE-ONLY: every survivor is the SAME object with identical geometry.
    surviving = [_sig(s) for s in pcb.segments]
    check("POS: DELETE-ONLY -- every survivor's geometry is byte-identical",
          all(sg in before_sigs for sg in surviving)
          and all(any(s is o for o in segs) for s in pcb.segments),
          f"{len(surviving)} survivors")

    # --- NEG -------------------------------------------------------------
    segs, pads = neg_fixture()
    pcb = _board(segs, pads)
    base = check_net_connectivity(NET, pcb.segments, pcb.vias, pads)
    check("NEG: fixture is a CONNECTED net with one crossing",
          n_crossings(pcb.segments) == 1 and base.get('connected') is True,
          f"crossings={n_crossings(pcb.segments)} connected={base.get('connected')}")
    n, nets, strip = prune_self_crossing_segments([], pcb, {NET})
    check("NEG: both arms load-bearing -> nothing removed", n == 0 and strip == [],
          f"removed={n}")
    check("NEG: board untouched", len(pcb.segments) == 3
          and all(any(s is o for o in segs) for s in pcb.segments))

    # --- SJ --------------------------------------------------------------
    segs, pads, link, stem = sj_fixture()
    pcb = _board(segs, pads)
    # Prove the two gates in isolation first, so a later behavior change
    # cannot make this fixture pass for the wrong reason.
    without_link = [s for s in segs if s is not link]
    c_link = check_net_connectivity(NET, without_link, [], pads)
    base_j = _soft_joint_pairs(segs, [], pads)
    new_j = _soft_joint_pairs(without_link, [], pads) - base_j
    check("SJ: dropping the SHORT arm is connectivity-safe...",
          c_link.get('connected') is True
          and not (c_link.get('disconnected_pads') or []),
          f"connected={c_link.get('connected')}")
    check("SJ: ...but manufactures a soft joint (the #319 gate's job)",
          len(base_j) == 0 and len(new_j) == 1, f"new soft-joint pairs={len(new_j)}")
    without_stem = [s for s in segs if s is not stem]
    c_stem = check_net_connectivity(NET, without_stem, [], pads)
    check("SJ: dropping the LONG arm strands pads (connectivity gate's job)",
          c_stem.get('connected') is False
          and len(c_stem.get('disconnected_pads') or []) == 2,
          f"connected={c_stem.get('connected')} "
          f"disc={len(c_stem.get('disconnected_pads') or [])}")
    n, nets, strip = prune_self_crossing_segments([], pcb, {NET})
    check("SJ: no arm is safe -> nothing removed, the crossing ships",
          n == 0 and n_crossings(pcb.segments) == 1, f"removed={n}")


def run_pipeline():
    print("Shared cleanup pipeline (both fronts)")
    segs, pads, link = pos_fixture()
    pcb = _board(segs, pads)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = run_post_route_cleanup([], pcb, {NET}, _cfg())
    check("pipeline: run_post_route_cleanup resolves the crossing",
          n_crossings(pcb.segments) == 0,
          f"crossings={n_crossings(pcb.segments)} segs={len(pcb.segments)}")
    check("pipeline: the removal is counted",
          out.counts.get('self_crossings_pruned') == 1,
          f"counts={out.counts.get('self_crossings_pruned')}")
    check("pipeline: the removed input segment is in input_strip_segments",
          any(s is link for s in out.input_strip_segments),
          f"strip={len(out.input_strip_segments)}")
    c = check_net_connectivity(NET, pcb.segments, pcb.vias, pads)
    check("pipeline: net still connected, no pad stranded",
          c.get('connected') and not (c.get('disconnected_pads') or []))

    # The NEG fixture must survive the WHOLE pipeline unchanged: nothing else
    # may quietly "resolve" a crossing whose arms are both load-bearing.
    segs, pads = neg_fixture()
    pcb = _board(segs, pads)
    with contextlib.redirect_stdout(io.StringIO()):
        out = run_post_route_cleanup([], pcb, {NET}, _cfg())
    check("pipeline: load-bearing X survives the whole pipeline",
          n_crossings(pcb.segments) == 1 and len(pcb.segments) == 3,
          f"crossings={n_crossings(pcb.segments)} segs={len(pcb.segments)}")


# --------------------------------------------------------------------------
# Real in-repo boards, driven through the SHIPPING path.
#
# These two are the ONLY tracked kicad_files/*.kicad_pcb that carry the defect
# (scanned: 22 boards, 6 crossings, all on these two). Production never calls
# the pass on a raw board -- it calls run_post_route_cleanup, where six other
# passes run first and four run after. So the A/B here is the SAME pipeline
# with the pass stubbed out vs live, which is the only comparison that pins
# what actually ships:
#
#   routed_output:  crossings 1 -> 1, copper multiset IDENTICAL. The pass fires
#                   (self_crossings_pruned == 1) but collapse_strict_redundant
#                   reaches the same arm on its own, so nothing changes on the
#                   board. Pinned as identity ON PURPOSE -- claiming a 2 -> 1
#                   win here would be false.
#   rp2350:         crossings 7 -> 5, three fewer segments (the two arms plus
#                   one dead end the sweep then trims). The 7 is not a typo:
#                   close_soft_joints manufactures 3 crossings on this board on
#                   BOTH legs, after this pass has already run.
#
# Asserted as inequalities with the measured numbers as detail strings, so a
# future re-generation of either board cannot turn a real improvement into a
# red -- but a regression of the pass to a no-op on rp2350 WILL go red.
# --------------------------------------------------------------------------
REAL = [('routed_output.kicad_pcb', dict(raw=2, off=1, on=1, pruned=1, same_copper=True)),
        ('rp2350_fpga_eensy_prePlane.kicad_pcb',
         dict(raw=4, off=7, on=5, pruned=2, same_copper=False))]


def _noop_pass(results, pcb_data, scope_net_ids=None, keep_input_copper=False):
    return 0, 0, []


def _pipeline_leg(path, live):
    """Run the real pipeline on a fresh parse, with the #162 pass live or
    stubbed out. Returns (pcb, counts)."""
    pcb = parse_kicad_pcb(path)
    cfg = GridRouteConfig(grid_step=0.1, clearance=0.09, via_size=0.6,
                          track_width=0.2, via_drill=0.3,
                          layers=list(pcb.board_info.copper_layers))
    scope = ({s.net_id for s in pcb.segments} | {v.net_id for v in pcb.vias}) - {0}
    real = cleanup_pipeline.prune_self_crossing_segments
    if not live:
        cleanup_pipeline.prune_self_crossing_segments = _noop_pass
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            out = run_post_route_cleanup([], pcb, scope, cfg)
    finally:
        cleanup_pipeline.prune_self_crossing_segments = real
    return pcb, out.counts


def _copper(pcb):
    return collections.Counter(_sig(s) for s in pcb.segments)


def run_real_boards():
    print("Tracked in-repo boards -- through run_post_route_cleanup (the shipping path)")
    if prune_self_crossing_segments is None:
        check("real boards: pass available", False, "ImportError")
        return
    for name, want in REAL:
        path = os.path.join(REPO, 'kicad_files', name)
        if not os.path.exists(path):
            check(f"{name}: present", False, path)
            continue
        raw = parse_kicad_pcb(path)
        n_raw = n_crossings(raw.segments)
        check(f"{name}: reproduces the #162 warning", n_raw == want['raw'],
              f"same-net self-crossings on the raw board={n_raw} "
              f"(expected {want['raw']})")

        off, c_off = _pipeline_leg(path, live=False)
        on, c_on = _pipeline_leg(path, live=True)
        x_off, x_on = n_crossings(off.segments), n_crossings(on.segments)

        check(f"{name}: the pass fires inside the pipeline",
              c_on.get('self_crossings_pruned', 0) >= 1
              and c_off.get('self_crossings_pruned', 0) == 0,
              f"self_crossings_pruned live={c_on.get('self_crossings_pruned')} "
              f"stubbed={c_off.get('self_crossings_pruned')} "
              f"(measured at authoring time: {want['pruned']})")
        check(f"{name}: the SHIPPED board never gains a crossing", x_on <= x_off,
              f"pipeline crossings stubbed={x_off} live={x_on} "
              f"(measured at authoring time: {want['off']} -> {want['on']})")

        same = _copper(off) == _copper(on)
        if want['same_copper']:
            # Honest pin: here the pass changes NOTHING that ships, because
            # collapse_strict_redundant already removes the same arm.
            check(f"{name}: shipped copper is IDENTICAL with and without the pass",
                  same and x_on == x_off,
                  f"segs {len(off.segments)}/{len(on.segments)}, "
                  f"crossings {x_off}/{x_on}, strict_collapsed "
                  f"{c_off.get('strict_collapsed')} -> {c_on.get('strict_collapsed')}")
        else:
            check(f"{name}: the pass is what removes the crossings",
                  x_on < x_off and not same,
                  f"crossings {x_off} -> {x_on}; segs {len(off.segments)} -> "
                  f"{len(on.segments)}; counts live={dict(sorted(c_on.items()))}")

        # SAFETY, graded on the SHIPPED boards: for every net whose copper the
        # pass changed, the live leg must be no worse than the stubbed leg --
        # connectivity, components, disconnected pads and soft joints.
        def by_net(pcb):
            d = {}
            for s in pcb.segments:
                d.setdefault(s.net_id, []).append(s)
            return d

        seg_off, seg_on = by_net(off), by_net(on)
        via_off, via_on = {}, {}
        for v in off.vias:
            via_off.setdefault(v.net_id, []).append(v)
        for v in on.vias:
            via_on.setdefault(v.net_id, []).append(v)
        changed = sorted(nid for nid in set(seg_off) | set(seg_on)
                         if collections.Counter(_sig(s) for s in seg_off.get(nid, []))
                         != collections.Counter(_sig(s) for s in seg_on.get(nid, [])))
        ok, detail = True, []
        for nid in changed:
            pads = on.pads_by_net.get(nid, [])
            b = check_net_connectivity(nid, seg_off.get(nid, []), via_off.get(nid, []), pads)
            a = check_net_connectivity(nid, seg_on.get(nid, []), via_on.get(nid, []), pads)
            nm = on.nets.get(nid)
            nm = nm.name if nm else str(nid)
            jb = len(_soft_joint_pairs(seg_off.get(nid, []), via_off.get(nid, []), pads))
            ja = len(_soft_joint_pairs(seg_on.get(nid, []), via_on.get(nid, []), pads))
            good = (bool(a.get('connected')) >= bool(b.get('connected'))
                    and (a.get('num_components') or 1) <= (b.get('num_components') or 1)
                    and len(a.get('disconnected_pads') or [])
                    <= len(b.get('disconnected_pads') or []) and ja <= jb)
            ok = ok and good
            detail.append(f"{nm}: connected {b.get('connected')}->{a.get('connected')}, "
                          f"comps {b.get('num_components')}->{a.get('num_components')}, "
                          f"disc {len(b.get('disconnected_pads') or [])}->"
                          f"{len(a.get('disconnected_pads') or [])}, "
                          f"soft joints {jb}->{ja}")
        check(f"{name}: every net the pass changed is no worse than without it", ok,
              "; ".join(detail) or "no net changed")

        # The pass's OWN invariant (the pipeline's other passes do move copper,
        # so this can only be asserted on a direct call): DELETE-ONLY.
        pcb = parse_kicad_pcb(path)
        sigs = {id(s): _sig(s) for s in pcb.segments}
        n, nets, strip = prune_self_crossing_segments([], pcb, None)
        check(f"{name}: DELETE-ONLY -- no survivor moved",
              n >= 1 and all(id(s) in sigs and _sig(s) == sigs[id(s)]
                             for s in pcb.segments),
              f"removed {n} on {nets} net(s), {len(pcb.segments)} segments remain")
        check(f"{name}: every removal is reported to the writer",
              len(strip) == n, f"strip={len(strip)} removed={n}")


# --------------------------------------------------------------------------
# INERTNESS: on a board with no same-net self-crossing the pass must be a
# strict no-op -- it may not even reassign pcb_data.segments.
# --------------------------------------------------------------------------
def run_inertness():
    print("Inertness on crossing-free boards")
    if prune_self_crossing_segments is None:
        check("inertness: pass available", False, "ImportError")
        return
    clean = ['orangecrab_ext_pll.kicad_pcb', 'lvds_converter_dualclk_gnd.kicad_pcb',
             'qfn_diffpair_escape.kicad_pcb', 'qfn_interior_pads.kicad_pcb']
    for name in clean:
        path = os.path.join(REPO, 'kicad_files', name)
        if not os.path.exists(path):
            check(f"{name}: present", False, path)
            continue
        pcb = parse_kicad_pcb(path)
        seg_list_id = id(pcb.segments)
        before = [_sig(s) for s in pcb.segments]
        n, nets, strip = prune_self_crossing_segments([], pcb, None)
        check(f"{name}: strict no-op",
              n == 0 and nets == 0 and strip == []
              and id(pcb.segments) == seg_list_id
              and [_sig(s) for s in pcb.segments] == before,
              f"{len(before)} segments, removed={n}")


# --------------------------------------------------------------------------
# NET-LEVEL SKIPS. Each of these is a shape where the connectivity gate is
# either forbidden (locked) or VACUOUS (no pads to grade), so without an
# explicit skip the pass would delete copper on the strength of a gate that
# cannot fail.
# --------------------------------------------------------------------------
def run_skips():
    print("Net-level skips")
    if prune_self_crossing_segments is None:
        check("skips: pass available", False, "ImportError")
        return

    # #521: KiCad-LOCKED copper makes its NET never-rippable, no override. The
    # lock may sit on EITHER arm -- gating on the candidate alone would just
    # delete the other one.
    for lock_short in (True, False):
        segs, pads, link = pos_fixture()
        (link if lock_short else segs[3]).locked = True
        pcb = _board(segs, pads)
        n, nets, strip = prune_self_crossing_segments([], pcb, {NET})
        where = "the SHORT arm (tried first)" if lock_short else "the LONG arm"
        check(f"LOCK: locked {where} -> the whole net is off-limits",
              n == 0 and nets == 0 and strip == [] and len(pcb.segments) == 4,
              f"removed={n} segs={len(pcb.segments)}")
    # a lock on a VIA of the net counts too, and on copper outside the scope
    segs, pads, link = pos_fixture()
    v = make_via(0.0, 0.0, net_id=NET)
    v.locked = True
    pcb = make_pcb(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=None),
                   segments=list(segs), vias=[v], pads_by_net={NET: list(pads)})
    n, _nets, _strip = prune_self_crossing_segments([], pcb, {NET})
    check("LOCK: a locked VIA also protects the net", n == 0, f"removed={n}")

    # Pad-less / unassigned nets: check_net_connectivity calls a pad-less net
    # connected with num_components 0, so every gate passes vacuously.
    x = [_seg(0, 0, 2, 2), _seg(2, 0, 0, 2), _seg(2, 2, 2, 0)]
    for label, net_id, pads in (("net_id == 0 (unassigned copper)", 0, []),
                                ("a net with ZERO pads", NET, []),
                                ("a net with ONE pad", NET, [_pad('S1', 0, 0)])):
        segs = [make_seg(s.start_x, s.start_y, s.end_x, s.end_y, net_id=net_id)
                for s in x]
        pcb = make_pcb(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                            board_bounds=None),
                       segments=segs,
                       pads_by_net=({net_id: pads} if pads else {}))
        check(f"PADS: {label} really carries a crossing",
              n_crossings(pcb.segments) == 1)
        # scope_net_ids=None is the signature default AND what the docs show.
        n, nets, strip = prune_self_crossing_segments([], pcb, None)
        check(f"PADS: {label} -> nothing deleted",
              n == 0 and nets == 0 and len(pcb.segments) == 3,
              f"removed={n} remaining={len(pcb.segments)}")


# --------------------------------------------------------------------------
# The index-based via-support twin must agree EXACTLY with the list-based one
# it replaced -- that equivalence is the whole basis of the #263-shaped speedup
# (the pass calls it once per candidate instead of rebuilding O(vias x segs)).
# --------------------------------------------------------------------------
def run_via_support_equivalence():
    print("Via-support model: index twin == list original")
    if _via_support_model is None:
        check("via support: helper available", False, "ImportError")
        return
    segs = [_seg(0, 0, 1, 0), _seg(1, 0, 2, 0), _seg(2, 0, 3, 0),
            _seg(1, 0, 1, 1, layer='B.Cu'), _seg(3, 0, 3, 1)]
    vias = [make_via(1.0, 0.0, net_id=NET), make_via(3.0, 0.0, net_id=NET),
            make_via(9.0, 9.0, net_id=NET)]
    pads = [_pad('V1', 0, 0), _pad('V2', 9, 9, size=1.0)]
    m = _via_support_model(vias, pads, segs)
    ok = True
    for mask in range(1 << len(segs)):
        excl = {i for i in range(len(segs)) if mask >> i & 1}
        kept = [s for i, s in enumerate(segs) if i not in excl]
        if m.supported_vias(kept) != m.supported_excluding(excl):
            ok = False
            break
    check("every one of the 32 segment subsets agrees", ok,
          f"{1 << len(segs)} subsets, {len(vias)} vias, {len(pads)} pads")


def run():
    run_synthetic()
    run_skips()
    run_via_support_equivalence()
    run_pipeline()
    run_real_boards()
    run_inertness()
    if fails:
        print(f"\n{len(fails)} FAILED: " + ", ".join(fails))
        return 1
    print("\nAll #162 self-crossing prune checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
