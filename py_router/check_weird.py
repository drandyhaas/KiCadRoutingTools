#!/usr/bin/env python3
"""Read-only checker for "weird" copper hygiene issues that are neither DRC
violations nor opens: dead-end antennas, soft joints, redundant loops,
individually-removable copper, stacked duplicates, and floating vias.

Categories:
  dangling-end       degree-1 segment endpoint not anchored on a same-net pad,
                     via, T-junction, or same-net zone outline (reuses
                     pcb_modification._point_anchored). Includes the
                     half-segment variant: a tail dangling PAST a mid-body
                     anchor (trim_dangles_past_body_anchor geometry,
                     report-only). Reports the dangling length.
  soft-joint         same-net endpoints whose caps overlap but are not
                     coincident (check_drc's 'segment-endpoint-gap' class;
                     reuses routing_constants.SOFT_JOINT_MIN_GAP and approach).
                     An end is ANCHORED -- and so not a free end at all -- by
                     a via barrel or by a pad its own cap overlaps, both by
                     RADIUS (#695); crediting a pad by centre containment made
                     this contradict check_connected's endpoint-cap rule. The
                     anchor is the SHARED predicate (connectivity.
                     endpoint_reaches_pad/_via), so this test, _check_dangles,
                     check_drc's soft-joint detector and the close_soft_joints
                     repair pass cannot drift apart again (#722) -- and it is
                     layer- and NPTH-aware, because a pad that carries no
                     copper on the end's layer anchors nothing. Copper-layer
                     GRAPHICS are not candidates (#337: immutable input art).
  redundant-cycle    same-net loop edges whose removal leaves connectivity
                     identical (pcb_modification._prune_net_cycles machinery,
                     report-only; zoned nets skipped -- planes are meshes).
                     On a net the removal model grades, only edges that
                     removable-segment also reports.
  removable-segment  copper route.py's own cleanup would delete: a segment,
                     or an unbranched run of them, whose removal keeps every
                     pad connected (strict and physical graphs), strands no
                     island, and leaves no new dangling end, soft joint or
                     dangling via. Graded by pcb_modification.
                     StrictRemovalModel -- the SAME predicate
                     collapse_strict_redundant removes by (#1063), so a
                     finding here is always something the cleanup takes out.
                     Nets over STRICT_REMOVAL_MAX_SEGS (500) segments are
                     skipped unless --thorough; zoned and <2-pad nets are
                     skipped (their connectivity result is trivially
                     insensitive).
  stacked-copper     exactly-duplicate segments (same endpoints/layer/net
                     within ~1um) and coincident same-net vias (centers
                     within 0.01mm) -- the duplicate-emission bug class.
  unsupported-via    a via with no same-net track copper reaching its barrel,
                     no same-net pad whose copper that barrel overlaps, and
                     not inside a same-net zone polygon (a floating via).
                     Track and pad are both judged by BARREL OVERLAP (#695);
                     the zone test is centre-in-OUTLINE, which is looser than
                     the authoritative model -- check_net_connectivity credits
                     a zone through its FILL model when a caller hands it
                     pcb_data, so a via in a clearance void inside the outline
                     grades supported here and unsupported there. Pre-existing,
                     and not what #695 asked about.
  dangling-via       a via whose same-net copper reaches it on exactly ONE of
                     the layers it spans, so the barrel joins nothing. This is
                     KiCad's own `via_dangling` rule; it is a strictly weaker
                     condition than unsupported-via, which needs ZERO support.
  orphan-island      a connected group of same-net track copper (segments,
                     possibly with vias) that reaches NO pad of the net --
                     dead copper stranded by a rip or a superseded route.
                     Zone-connected copper counts as connected (the island
                     unions with everything in its zone outline). Size = the
                     island's total copper length.
  narrow-pad-joint   a degree-1 terminal cap that overlaps a same-net pad only
                     near a CORNER, so the two join through a copper web
                     thinner than the board's min-track floor (issue #416).
                     DRC-clean and connected, but the joint can etch open --
                     KiCad's own `connection_width` class. Reported with NO
                     size, so --tolerance never filters it (as for soft-joint:
                     for a web, THINNER is worse, so a size filter would drop
                     the severe findings and keep the marginal ones).
  kicad-dangling     a joint segment whose two ends both lie on other
                     same-net TRACKS that hold the other end too (#1217):
                     KiCad counts such an item for one end only, so its DRC
                     reports track_dangling (a 24-35 um stub on one track)
                     while the T-junction rule credits both ends. Only in
                     that shape -- a via, pad or zone at either end is left to
                     KiCad's own grade -- and with NO size, since KiCad warns
                     at any length. Measured on six routed boards: every
                     finding is one of KiCad's track_dangling warnings.

This script NEVER modifies the board (read-only; nothing is written back).
Net 0 (unconnected) copper is skipped -- "same-net" semantics do not apply.

Usage:
    python3 check_weird.py board.kicad_pcb [--nets PATTERN ...] [--thorough]
                                           [--max-print N]

Exit code 0 when clean, 1 when any findings.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}

import argparse
import math
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

from kicad_parser import parse_kicad_pcb, PCBData
from check_connected import (matches_any_pattern, check_net_connectivity,
                             analyze_conn_excluding, point_in_polygon,
                             _point_in_pad)
from check_drc import point_to_pad_distance
from connectivity import (COINCIDENCE_TOL, endpoint_reaches_pad, strict_joint_roots,
                          endpoint_reaches_via, lands_on_interior,
                          via_copper_layers)
from routing_constants import SOFT_JOINT_MIN_GAP
from pcb_modification import (_point_anchored, _prune_net_cycles, _pt_seg_dist,
                              _restore_soft_joint_bridges,
                              StrictRemovalModel, strict_removable_segments,
                              STRICT_REMOVAL_MAX_SEGS)

CATEGORIES = ['dangling-end', 'soft-joint', 'redundant-cycle',
              'removable-segment', 'stacked-copper', 'unsupported-via',
              'dangling-via', 'orphan-island', 'narrow-pad-joint',
              'kicad-dangling']
# Cost cap for the removable scan (skip unless --thorough): the removal pass's
# own cap (#1063), so the checker never grades a net the pass may not clean.
MAX_SEGS_PER_NET = STRICT_REMOVAL_MAX_SEGS
_CELL = 1.0  # spatial-grid cell (mm) fed to _point_anchored, as in the pruner
_VIA_COINCIDENT_MM = 0.01  # stacked-via center distance
_DUP_SEG_DECIMALS = 3      # ~1um endpoint quantization for exact duplicates


def _finding(category, net, layer, x, y, detail, size=None):
    # size = the finding's characteristic magnitude in mm (dangle length,
    # gap, duplicated-copper length, via diameter ...). The --tolerance
    # filter drops findings smaller than the threshold; None = always report.
    return {'category': category, 'net': net, 'layer': layer,
            'x': x, 'y': y, 'detail': detail, 'size': size}


def _net_name(pcb_data: PCBData, net_id: int) -> str:
    net = pcb_data.nets.get(net_id)
    return net.name if net else f"net_{net_id}"


def _via_span(via, copper_layers) -> set:
    """Copper layers a via connects (through vias span everything).

    Blind/buried vias record only their START/END layers in the file; the
    barrel also connects every copper layer BETWEEN them in stackup order
    (a buried F.Cu-In2.Cu via touches In1.Cu) -- treating the span as the
    two endpoints alone manufactured phantom unsupported-via findings for
    mid-span connections."""
    if via.layers and not ('F.Cu' in via.layers and 'B.Cu' in via.layers):
        order = list(copper_layers)
        idx = [order.index(l) for l in via.layers if l in order]
        if len(idx) >= 2:
            lo, hi = min(idx), max(idx)
            return set(order[lo:hi + 1])
        return set(via.layers)
    return set(copper_layers)


def _check_soft_joints(net_id, name, net_segs, net_vias, net_pads,
                       findings, copper_layers=()):
    """check_drc's 'segment-endpoint-gap' detection (same constants/approach):
    dangling free ends (degree 1, not on a via/own pad) whose caps overlap
    but whose endpoints are not coincident. Returns the set of participating
    endpoint keys (layer, rx, ry) so the dangle check can defer to this,
    more specific, category."""
    def at_anchor(x, y, layer, width):
        """Does this end's own COPPER reach a same-net via barrel or pad?

        The cap is what physically touches, so the cap radius is the credit
        -- for the pad exactly as for the via. Both answers now come from the
        SHARED predicates in connectivity, so the four copies of this test
        (here, _check_dangles, check_drc._at_anchor and
        pcb_modification.at_anchor) cannot drift apart again -- which is how
        #695 and #722 shipped four lines from each other.

        Three things the cap credit alone did not fix (#722):
          * LAYER. This test credited ANY same-net pad whatever layer it sat
            on, and widening the reach from 0.02mm to width/2 widened that
            hole with it. check_connected iterates the pad's OWN copper
            layers, so a B.Cu end near an F.Cu-only pad is genuinely SPLIT
            there while this called it anchored -- a false NEGATIVE, the
            direction that ships broken copper.
          * NPTH. A plated hole is copper; an unplated one is not, whatever
            its layer list says.
          * The VIA branch is cap-blind in the same way the pad branch was:
            `vr + 0.01` credits the barrel but not the cap, where the
            authority credits (via_size + track_width)/2.
        """
        r = (width or 0.0) / 2.0
        for v in net_vias:
            if endpoint_reaches_via(x, y, r, v, (layer,), copper_layers):
                return True
        for pd in net_pads:
            if endpoint_reaches_pad(x, y, r, (layer,), pd):
                return True
        return False

    def rk(x, y):
        return (round(x, 3), round(y, 3))

    ep_count = defaultdict(int)
    for s in net_segs:
        ep_count[(s.layer, rk(s.start_x, s.start_y))] += 1
        ep_count[(s.layer, rk(s.end_x, s.end_y))] += 1

    dangles = defaultdict(list)  # layer -> [(x, y, width)]
    for s in net_segs:
        for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if ep_count[(s.layer, rk(x, y))] != 1:
                continue  # shared vertex = clean joint
            if at_anchor(x, y, s.layer, s.width):
            # A soft joint is a PAIR. Dropping a copper-layer GRAPHIC as a
            # candidate drops the TRACK end paired with it, and nothing
            # else picks that end up -- measured, a genuinely open net
            # (check_connected: 2 components, 1 disconnected pad) went
            # from exit 1 to exit 0, and check_complete gates DONE on that
            # exit code. The ART end is the unactionable half (#337 forbids
            # touching it); the TRACK end is entirely actionable (extend it,
            # or make the art real copper). So a graphic is carried as a
            # flag and only an art-MEETS-art pair is dropped.
                continue  # its copper reaches a via / own pad = legitimate
            dangles[s.layer].append((x, y, s.width, getattr(s, 'graphic', False),
                                     id(s)))

    soft_pts = set()
    roots = []   # check_drc's "ONLY" test (#984), built on the first candidate

    def joined_elsewhere(oa, ob):
        if not roots:
            roots.append(strict_joint_roots(net_segs, net_vias, net_pads,
                                            copper_layers))
        return roots[0].get(oa) == roots[0].get(ob)

    for layer, ends in dangles.items():
        for i in range(len(ends)):
            xi, yi, wi, gi, oi = ends[i]
            for j in range(i + 1, len(ends)):
                xj, yj, wj, gj, oj = ends[j]
                if oi == oj:
                    continue  # #672: one segment's own two ends are not a joint
                gap = math.hypot(xi - xj, yi - yj)
                cap = (wi + wj) / 2.0
                if gi and gj:
                    continue  # art meets art: nothing anyone can act on
                if SOFT_JOINT_MIN_GAP < gap < cap - 1e-6 \
                        and not joined_elsewhere(oi, oj):
                    # size=None: soft joints bypass the --tolerance filter.
                    # Filtering by GAP inverted the severity metric (small
                    # gap = still fragile) and on <=0.1mm-width routing every
                    # representable soft joint has gap < 0.1 -- the whole
                    # category silently vanished at the default tolerance.
                    if gi:  # report where the fix goes: the TRACK end
                        xi, yi, xj, yj = xj, yj, xi, yi
                    findings.append(_finding(
                        'soft-joint', name, layer, xi, yi,
                        f"endpoint gap {gap:.3f}mm to ({xj:.3f}, {yj:.3f}), "
                        f"caps overlap {cap - gap:.3f}mm (fragile near-open)"))
                    k1 = (layer,) + rk(xi, yi)
                    k2 = (layer,) + rk(xj, yj)
                    soft_pts.add(k1)
                    soft_pts.add(k2)
    return soft_pts


def _check_dangles(net_id, name, net_segs, net_vias, net_pads, net_zones,
                   soft_pts, findings, join_tol: float = 0.0, copper_layers=()):
    """Degree-1 endpoints that _point_anchored calls unanchored and that are
    not inside a same-net zone outline. Half-segment tails past a mid-body
    anchor reuse trim_dangles_past_body_anchor's geometry (report-only)."""
    tol = COINCIDENCE_TOL
    track_segs = [s for s in net_segs if not getattr(s, 'graphic', False)]
    if not track_segs:
        return
    # A via anchors only the layers its barrel has copper on (#722's via-layer
    # rule, which _check_soft_joints already applies through
    # endpoint_reaches_via): a blind F.Cu/In1.Cu via anchors no B.Cu end.
    # This credit was layer-blind, and the soft-joint finding used to mask it
    # -- until #984 stopped calling two stubs out of one vertex a soft joint.
    _via_pts_on = {}

    def via_pts_on(layer):
        pts = _via_pts_on.get(layer)
        if pts is None:
            pts = _via_pts_on[layer] = [
                (v.x, v.y, getattr(v, 'size', 0.6) or 0.6) for v in net_vias
                if layer in via_copper_layers(v, copper_layers or None)]
        return pts
    # NO pads are handed to _point_anchored. Its pad test is a bounding CIRCLE
    # of radius max(size_x, size_y)/2, which over-credits every non-square pad
    # -- and because it runs FIRST it can only ADD credit, so the exact test
    # below never gets to refuse. Measured over kicad_files/: it anchors 2
    # endpoints the exact predicate refuses, both a 0.3mm track ending 0.575mm
    # from a 1.8x0.45 pad's copper on lvds_converter_dualclk_gnd -- a board
    # that reports zero findings today, so those are two masked dangling-ends.
    # _point_anchored keeps the VIA and T-junction halves, which this function
    # has no other source for; pads are answered by endpoint_reaches_pad alone.
    # (The bounding circle is left alone in pcb_modification, where the pruner
    # shares it and over-crediting is the safe direction for a REMOVAL gate.)
    pad_pts = []

    def key(x, y, layer):
        return (round(x, 3), round(y, 3), layer)

    # Graphics copper counts as an anchor universe member but is never a
    # candidate (immutable input art, #337).
    degree = defaultdict(int)
    seg_index = defaultdict(list)
    for s in net_segs:
        degree[key(s.start_x, s.start_y, s.layer)] += 1
        degree[key(s.end_x, s.end_y, s.layer)] += 1
        lo_x = int(min(s.start_x, s.end_x) // _CELL)
        hi_x = int(max(s.start_x, s.end_x) // _CELL)
        lo_y = int(min(s.start_y, s.end_y) // _CELL)
        hi_y = int(max(s.start_y, s.end_y) // _CELL)
        for cx in range(lo_x, hi_x + 1):
            for cy in range(lo_y, hi_y + 1):
                seg_index[(s.layer, cx, cy)].append(s)

    zones_by_layer = defaultdict(list)
    for z in net_zones:
        zones_by_layer[z.layer].append(z)

    for s in track_segs:
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        L2 = dx * dx + dy * dy
        free_ends = []
        for free_is_start in (True, False):
            fx, fy = (s.start_x, s.start_y) if free_is_start else (s.end_x, s.end_y)
            if degree[key(fx, fy, s.layer)] != 1:
                continue
            if ((s.layer, round(fx, 3), round(fy, 3)) in soft_pts):
                continue  # already reported as the more specific soft-joint
            if _point_anchored(fx, fy, s.layer, via_pts_on(s.layer), pad_pts,
                               seg_index, _CELL, s, tol):
                continue
            # _point_anchored's pad test is RADIAL (center distance vs the
            # max half-dimension) and misses rectangular pad corners: a stub
            # ending exactly on a 1.4x1.2 crystal pad's corner copper read
            # as a dangle. Exact outline test (glasgow XTALOUT/C11).
            # The exact outline test is the SHARED predicate now --
            # byte-identical to the one _check_soft_joints asks. Two
            # spellings changed, both measured over kicad_files/:
            # COINCIDENCE_TOL became a floor under the cap credit (it bites
            # below 0.04mm track width; the corpus floor is 0.0762) and
            # `'*.Cu' not in p.layers` became `any('*' in L)`, the form every
            # other site already used.
            if any(endpoint_reaches_pad(fx, fy, s.width / 2.0, (s.layer,), p)
                   for p in net_pads):
                continue
            if any(point_in_polygon(fx, fy, z.polygon)
                   for z in zones_by_layer.get(s.layer, ())):
                continue  # lands in a same-net zone fill outline
            # Two long tracks whose ends miss each other by a few um (a
            # nudge/micro-shift split pair) are OFF BY that microgap, not by
            # their segment lengths: within join_tol they are connected
            # copper, not dangles.
            if join_tol > 0:
                joined = False
                cx0, cy0 = int(fx // _CELL), int(fy // _CELL)
                for ncx in (cx0 - 1, cx0, cx0 + 1):
                    for ncy in (cy0 - 1, cy0, cy0 + 1):
                        for o in seg_index.get((s.layer, ncx, ncy), ()):
                            if o is s:
                                continue
                            _cap = (s.width + o.width) / 2 - 1e-6
                            _jt = min(join_tol, _cap)
                            if (math.hypot(o.start_x - fx, o.start_y - fy) <= _jt
                                    or math.hypot(o.end_x - fx, o.end_y - fy) <= _jt):
                                # Width-aware: two fine tracks 0.09mm apart
                                # do NOT overlap caps (real open) and stay
                                # flagged; a flat 0.1 gate hid them.
                                joined = True
                                break
                        if joined:
                            break
                    if joined:
                        break
                if joined:
                    continue
            free_ends.append((free_is_start, fx, fy))
        if not free_ends:
            continue
        seg_len = math.sqrt(L2)
        if len(free_ends) == 2:
            _, fx, fy = free_ends[0]
            _, ox, oy = free_ends[1]
            findings.append(_finding(
                'dangling-end', name, s.layer, fx, fy,
                f"isolated fragment {seg_len:.3f}mm long, "
                f"other end at ({ox:.3f}, {oy:.3f})", size=seg_len))
            continue
        free_is_start, fx, fy = free_ends[0]
        # Mid-body anchors (trim_dangles_past_body_anchor geometry): a same-net
        # via barrel overlapping the centerline, or another same-net segment
        # endpoint teeing into the body.
        cands = []
        if L2 >= 1e-9:
            for vx, vy, vsize in via_pts_on(s.layer):
                t = ((vx - s.start_x) * dx + (vy - s.start_y) * dy) / L2
                if not lands_on_interior(t, L2, tol):          # #1186
                    continue
                cx_, cy_ = s.start_x + t * dx, s.start_y + t * dy
                if math.hypot(vx - cx_, vy - cy_) < (vsize + s.width) / 2 - 1e-6:
                    cands.append(t)
            for o in net_segs:
                if o is s or o.layer != s.layer:
                    continue
                for ox, oy in ((o.start_x, o.start_y), (o.end_x, o.end_y)):
                    t = ((ox - s.start_x) * dx + (oy - s.start_y) * dy) / L2
                    if not lands_on_interior(t, L2, tol):      # #1186
                        continue
                    cx_, cy_ = s.start_x + t * dx, s.start_y + t * dy
                    if math.hypot(ox - cx_, oy - cy_) < (o.width + s.width) / 2 - 1e-6:
                        cands.append(t)
        if cands:
            t_anchor = min(cands) if free_is_start else max(cands)
            nx, ny = s.start_x + t_anchor * dx, s.start_y + t_anchor * dy
            tail = math.hypot(fx - nx, fy - ny)
            if tail <= max(tol, 3 * s.width):
                continue  # sub-visible nib past the anchor (as the trim pass)
            findings.append(_finding(
                'dangling-end', name, s.layer, fx, fy,
                f"half-segment tail dangling {tail:.3f}mm past body anchor "
                f"at ({nx:.3f}, {ny:.3f})", size=tail))
        else:
            findings.append(_finding(
                'dangling-end', name, s.layer, fx, fy,
                f"free end, dangling segment {seg_len:.3f}mm "
                f"(rooted at ({s.end_x if free_is_start else s.start_x:.3f}, "
                f"{s.end_y if free_is_start else s.start_y:.3f}))",
                size=seg_len))


def _check_cycles(net_id, name, net_segs, net_vias, net_pads, has_zone,
                  findings, removable=None):
    """Report-only spanning-tree reduction (prune_redundant_cycles machinery).
    _prune_net_cycles internally validates every proposed removal against
    check_net_connectivity, so reported edges are guaranteed redundant.

    ``removable`` (#1063) is the net's StrictRemovalModel answer when the
    removal pass grades this net: a loop edge is then reported only when that
    pass would remove it, so a loop the pass must keep (closed only through a
    soft joint, a via it may not drop, ...) is not a finding nobody can act
    on. None -- a net the model does not grade (over the segment cap, fewer
    than two pads, or strictly split) -- keeps the verdict of
    prune_redundant_cycles, the pass that owns loops there."""
    if has_zone:
        return  # planes / pours are meshes, not trees (as the pruner)
    track_segs = [s for s in net_segs if not getattr(s, 'graphic', False)]
    if len(track_segs) < 3:
        return
    empty_fgrid = defaultdict(list)  # no grazing preference needed for a report
    kept, removed = _prune_net_cycles(net_id, track_segs, net_vias, net_pads,
                                      empty_fgrid, _CELL, 0.0, 0.1)
    if removable is not None:
        keep = {id(net_segs[i]) for i in removable}
        removed = [s for s in removed if id(s) in keep]
    else:
        # Not a net the removal model grades: mirror prune_redundant_cycles,
        # the pass that owns loops there, which puts back any edge whose
        # removal would open a soft joint (#319). A sub-cell loop edge on a
        # fine-pitch net is usually exactly that, and reporting it named
        # copper no pass will ever take out.
        _k, removed = _restore_soft_joint_bridges(list(kept), list(removed),
                                                  net_vias, net_pads)
    for s in removed:
        mx, my = (s.start_x + s.end_x) / 2.0, (s.start_y + s.end_y) / 2.0
        findings.append(_finding(
            'redundant-cycle', name, s.layer, mx, my,
            f"loop edge ({s.start_x:.3f}, {s.start_y:.3f})-"
            f"({s.end_x:.3f}, {s.end_y:.3f}); removal leaves connectivity "
            f"identical",
            size=math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)))


def _strict_removable(net_id, name, net_segs, net_vias, net_pads, has_zone,
                      copper_layers, thorough, skipped_nets, web_floor=0.0):
    """The removal pass's own verdict on this net (#1063): the segment indices
    ``collapse_strict_redundant`` would remove from the board as it stands,
    graded by the SAME StrictRemovalModel. None when the model does not grade
    the net (zoned, fewer than two pads, over the shared segment cap -- then
    named in ``skipped_nets`` -- or strictly split).

    Three choices make "check_weird calls it removable" imply "the pass
    removes it" on any board, whatever produced it:
      * vias are graded non-droppable here: the pass drops only vias its own
        run placed, and a finished board does not record which those were, so
        a removal that would leave ANY via dangling is not reported;
      * every unlocked, non-graphic segment is a candidate: provenance and
        ``keep_input_copper`` (a chained step's read-only input) cannot be seen
        on the shipped board, and the default pass removes input copper too;
      * the segment cap is the pass's own, STRICT_REMOVAL_MAX_SEGS."""
    if has_zone or len(net_pads) < 2:
        return None
    if not any(not getattr(s, 'graphic', False) for s in net_segs):
        return None
    if len(net_segs) > MAX_SEGS_PER_NET and not thorough:
        skipped_nets.append((name, len(net_segs)))
        return None
    model = StrictRemovalModel(net_id, net_segs, net_vias, net_pads,
                               copper_layers, web_floor=web_floor)
    if not model.valid:
        return None
    return strict_removable_segments(model)


def _check_removable(net_id, name, net_segs, removable, findings):
    """Segments the strict removal pass would delete (#217/#1063): alone, or
    as part of an unbranched run that leaves whole. Graded by
    StrictRemovalModel -- strict width-clamped graph (a via joins a pad only
    by its centre), physical graph, no new dangling end, soft joint or
    dangling via -- the predicate collapse_strict_redundant removes by."""
    for i in sorted(removable or ()):
        s = net_segs[i]
        mx, my = (s.start_x + s.end_x) / 2.0, (s.start_y + s.end_y) / 2.0
        findings.append(_finding(
            'removable-segment', name, s.layer, mx, my,
            f"segment ({s.start_x:.3f}, {s.start_y:.3f})-"
            f"({s.end_x:.3f}, {s.end_y:.3f}) w{s.width:.3f}: removal "
            f"does not change net connectivity",
            size=math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)))


def _check_kicad_dangling(net_id, name, net_segs, net_vias, net_pads,
                          net_zones, copper_layers, findings):
    """A joint segment KiCad's DRC calls ``track_dangling`` although both its
    ends lie on same-net copper (#1217).

    KiCad's rule (CONNECTIVITY_DATA::TestTrackEndpointDangling): each item
    touching the segment -- its shape within half the segment's width of an
    end -- counts for that end, and an item touching BOTH ends counts only for
    the end nearer it (a track by its nearer endpoint). The segment dangles
    unless both ends are counted. So a 24-35 um stub lying on one other track
    -- its far end mid-body, its root in that track's copper -- dangles in
    KiCad, while check_weird's T-junction rule credits both ends.

    Reported only in that unambiguous shape: every item touching the segment
    is a same-net TRACK, each one touches both ends, and no via, pad or zone
    touches it. Where a via, pad or zone is involved KiCad's verdict turns on
    zone-fill and anchor details this model does not reproduce, so it says
    nothing rather than guess. Never size-filtered: KiCad warns at any length.
    """
    for i, s in enumerate(net_segs):
        if getattr(s, 'graphic', False):
            continue
        acc = s.width / 2.0
        ends = ((s.start_x, s.start_y), (s.end_x, s.end_y))
        touchers = []
        for j, o in enumerate(net_segs):
            if j == i or o.layer != s.layer or getattr(o, 'graphic', False):
                continue
            hits = [_pt_seg_dist(x, y, o.start_x, o.start_y, o.end_x, o.end_y)
                    <= o.width / 2.0 + acc + 1e-9 for x, y in ends]
            if any(hits):
                touchers.append(hits)
        if not touchers or not all(h[0] and h[1] for h in touchers):
            continue
        if any(s.layer in via_copper_layers(v, copper_layers)
               and min(math.hypot(v.x - x, v.y - y) for x, y in ends)
               <= v.size / 2.0 + acc + 1e-9 for v in net_vias):
            continue
        if any(endpoint_reaches_pad(x, y, acc, (s.layer,), p)
               for p in net_pads for x, y in ends):
            continue
        if any(z.layer == s.layer and z.polygon
               and any(point_in_polygon(x, y, z.polygon) for x, y in ends)
               for z in net_zones):
            continue
        # Every toucher holds both ends, so each is counted once, at the end
        # nearer one of its own endpoints; the other end is never counted.
        counts = [0, 0]
        for j, o in enumerate(net_segs):
            if j == i or o.layer != s.layer or getattr(o, 'graphic', False):
                continue
            if _pt_seg_dist(ends[0][0], ends[0][1], o.start_x, o.start_y,
                            o.end_x, o.end_y) > o.width / 2.0 + acc + 1e-9:
                continue
            d0, d1 = (min(math.hypot(o.start_x - x, o.start_y - y),
                          math.hypot(o.end_x - x, o.end_y - y)) for x, y in ends)
            counts[0 if d0 < d1 else 1] += 1
        if all(counts):
            continue
        free = ends[0] if counts[0] == 0 else ends[1]
        findings.append(_finding(
            'kicad-dangling', name, s.layer, free[0], free[1],
            f"segment ({s.start_x:.3f}, {s.start_y:.3f})-({s.end_x:.3f}, "
            f"{s.end_y:.3f}) w{s.width:.3f}: both ends lie on other {name} "
            f"track(s) that hold the other end too -- KiCad's DRC reports "
            f"track_dangling at ({free[0]:.3f}, {free[1]:.3f})",
            size=None))


def _check_stacked(net_id, name, net_segs, net_vias, findings):
    """Exactly-duplicate segments (~1um) and coincident same-net vias."""
    groups = defaultdict(list)
    for s in net_segs:
        a = (round(s.start_x, _DUP_SEG_DECIMALS), round(s.start_y, _DUP_SEG_DECIMALS))
        b = (round(s.end_x, _DUP_SEG_DECIMALS), round(s.end_y, _DUP_SEG_DECIMALS))
        lo, hi = (a, b) if a <= b else (b, a)
        groups[(s.layer, lo, hi)].append(s)
    for (layer, lo, hi), ss in groups.items():
        if len(ss) > 1:
            findings.append(_finding(
                'stacked-copper', name, layer, lo[0], lo[1],
                f"{len(ss)} duplicate segments stacked on "
                f"({lo[0]:.3f}, {lo[1]:.3f})-({hi[0]:.3f}, {hi[1]:.3f})",
                size=math.hypot(hi[0] - lo[0], hi[1] - lo[1])))
    # Coincident vias: bucket at the coincidence radius, scan 3x3 neighbors.
    cell = _VIA_COINCIDENT_MM
    grid = defaultdict(list)
    for idx, v in enumerate(net_vias):
        grid[(int(math.floor(v.x / cell)), int(math.floor(v.y / cell)))].append(idx)
    reported = set()
    for i, v in enumerate(net_vias):
        cx = int(math.floor(v.x / cell))
        cy = int(math.floor(v.y / cell))
        for gx in (cx - 1, cx, cx + 1):
            for gy in (cy - 1, cy, cy + 1):
                for j in grid.get((gx, gy), ()):
                    if j <= i:
                        continue
                    o = net_vias[j]
                    d = math.hypot(v.x - o.x, v.y - o.y)
                    if d <= _VIA_COINCIDENT_MM + 1e-9 and (i, j) not in reported:
                        reported.add((i, j))
                        layer_str = ','.join(v.layers) if v.layers else '*.Cu'
                        findings.append(_finding(
                            'stacked-copper', name, layer_str, v.x, v.y,
                            f"coincident vias {d * 1000:.1f}um apart "
                            f"(other at ({o.x:.4f}, {o.y:.4f}))",
                            size=getattr(v, 'size', None)))


def stacked_copper_over_model(segs_by_net, vias_by_net, net_name):
    """The stacked-copper check over a per-net write model (run-7 E3).

    route.py calls this on the copper it is ABOUT to write, after its via
    dedup, so anything still stacked is surfaced in the run summary instead
    of shipping silently (KiCad permits same-net stacks, so no DRC ever
    flags them). `net_name` is a net_id -> str callable. Returns the same
    finding dicts check_weird's CLI reports.
    """
    findings = []
    for nid in sorted(set(segs_by_net) | set(vias_by_net)):
        _check_stacked(nid, net_name(nid), segs_by_net.get(nid, []),
                       vias_by_net.get(nid, []), findings)
    return findings


def via_support_parts(v, net_segs, net_pads, net_zones, copper_layers):
    """WHICH copper reaches via ``v``'s barrel, split so a caller can re-derive
    the support after removing segments (#1063: collapse_strict_redundant must
    never leave a via this checker calls dangling, and it grades that without
    re-running the whole scan per candidate).

    Returns ``(span, fixed, by_seg)``: the via's copper-layer span, the layers
    supported by pads and zones (``fixed``), and ``{segment index: layer}`` for
    every segment whose copper reaches the barrel. The support
    ``_check_unsupported_vias`` grades is ``fixed | set(by_seg.values())``.
    Pad credit is barrel-overlap, not centre-containment (#695) -- see the note
    at the pad loop below."""
    span = _via_span(v, copper_layers)
    r = (getattr(v, 'size', 0.6) or 0.6) / 2.0
    # Collect WHICH layers support the barrel, not merely whether any does.
    # A via exists to join layers, so one supported layer means it joins
    # nothing -- that is KiCad's own `via_dangling` rule ("fewer than two
    # layers connected"), and short-circuiting at the first hit could not
    # express it: run 11 shipped a board KiCad flagged with 64 dangling
    # vias while this check reported none, because every one of them had
    # copper on exactly one end.
    by_seg = {}
    for i, s in enumerate(net_segs):
        if s.layer not in span:
            continue
        if _pt_seg_dist(v.x, v.y, s.start_x, s.start_y,
                        s.end_x, s.end_y) < r + s.width / 2 - 1e-6:
            by_seg[i] = s.layer
    fixed = set()
    for p in net_pads:
        if getattr(p, 'pad_type', '') == 'np_thru_hole':
            continue  # NPTH pads have no copper
        if p.drill and p.drill > 0:
            on = set(span)  # plated barrel spans all copper layers
        else:
            pl = set(p.layers or [])
            on = set(span) if any('*' in L for L in pl) else (span & pl)
        # The barrel has a RADIUS against a track (above), so it has one
        # against a pad too. `margin` inflates the EXACT pad outline, so
        # this reads "the barrel copper overlaps the pad copper" -- the
        # same GEOMETRY as check_connected.py's via-in-pad union and
        # check_drc's via-in-edge-pad exemption, with COINCIDENCE_TOL kept
        # as the floor exactly as it is there. Crediting the CENTRE only
        # (COINCIDENCE_TOL, 0.02mm) made this checker contradict the
        # authoritative connectivity model on copper KiCad grades joined,
        # and check_weird's exit code is chain-blocking: an off-centre
        # via-in-pad read as `dangling via` forced a reroute lap (#695).
        #
        # The LAYER model above is NOT the same, and this is only geometry
        # parity: check_connected expands pad.layers (dropping *.Mask and
        # friends) and unions only on a SHARED copper layer, while `on`
        # here hands a drilled pad -- or one carrying any '*' layer -- the
        # via's whole span. That predates #695 and no board in the corpus
        # has a pad whose copper layers are a strict subset, but a plated
        # pad declaring only F/B.Cu would let a buried via grazing its ring
        # claim an inner layer. Left alone deliberately; fixing it is a
        # different behaviour change from the one this comment describes.
        if on and not on <= fixed and _point_in_pad(
                v.x, v.y, p, margin=max(r - 1e-6, COINCIDENCE_TOL)):
            fixed |= on
    for z in net_zones:
        if z.layer in span and z.layer not in fixed and point_in_polygon(
                v.x, v.y, z.polygon):
            fixed.add(z.layer)
    return span, fixed, by_seg


def _check_unsupported_vias(net_id, name, net_segs, net_vias, net_pads,
                            net_zones, copper_layers, findings):
    """Floating vias: no same-net track copper reaching the barrel, no
    same-net pad whose copper the barrel OVERLAPS, no same-net zone polygon
    around it. The support model is ``via_support_parts``, which the removal
    pass shares (#1063)."""
    for v in net_vias:
        span, fixed, by_seg = via_support_parts(v, net_segs, net_pads,
                                                net_zones, copper_layers)
        sup = set(fixed) | set(by_seg.values())
        layer_str = ','.join(v.layers) if v.layers else '*.Cu'
        if not sup:
            findings.append(_finding(
                'unsupported-via', name, layer_str, v.x, v.y,
                "floating via: no same-net track, pad, or zone reaches it",
                size=getattr(v, 'size', None)))
        elif len(sup) == 1 and len(span) > 1:
            findings.append(_finding(
                'dangling-via', name, layer_str, v.x, v.y,
                f"dangling via: same-net copper reaches it on {next(iter(sup))} "
                f"only, so it spans {len(span)} layer(s) but joins none "
                f"(KiCad via_dangling)",
                size=getattr(v, 'size', None)))


def _check_orphan_islands(net_id, name, net_segs, net_vias, net_pads,
                          net_zones, findings):
    """Connected components of track copper that reach no pad of the net.
    Built on check_net_connectivity's own graph, so vias, T-junctions, cap
    overlaps, and zone-outline membership all count as connections -- an
    island flagged here is one the AUTHORITATIVE model calls pad-less. A
    via-only island is left to unsupported-via."""
    track_segs = [s for s in net_segs if not getattr(s, 'graphic', False)]
    if not track_segs or not net_pads:
        return
    from geometry_utils import UnionFind
    r = check_net_connectivity(net_id, net_segs, net_vias, net_pads,
                               net_zones, return_graph=True)
    graph = r.get('graph')
    if not graph:
        return
    uf = UnionFind()
    for a, b in graph.get('edges', []):
        uf.union(a, b)
    pad_roots = {uf.find(rep) for rep in graph.get('pad_index_repr', {}).values()}
    islands = defaultdict(list)  # root -> [segment, ...]
    for i, s in enumerate(net_segs):
        if getattr(s, 'graphic', False):
            continue
        islands[uf.find(2 * i)].append(s)
    for root, segs in islands.items():
        if root in pad_roots:
            continue
        total = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                    for s in segs)
        cx = sum((s.start_x + s.end_x) / 2 for s in segs) / len(segs)
        cy = sum((s.start_y + s.end_y) / 2 for s in segs) / len(segs)
        findings.append(_finding(
            'orphan-island', name, segs[0].layer, cx, cy,
            f"{len(segs)} segment(s), {total:.2f}mm of copper connected to "
            f"NO pad of the net", size=total))


def _check_terminal_web(pcb_data, net_id, name, net_segs, net_pads, floor,
                        findings):
    """Flag degree-1 terminal endpoints whose cap overlaps a same-net pad only
    near a CORNER, joining through a copper web thinner than the min-track floor
    (issue #416). DRC-clean and connected, but a manufacturability hazard (the
    joint can etch open); KiCad's connection_width class catches it. Uses the
    SAME closed-form erosion criterion (``terminal_pad_web_shortfall``) as the
    close_soft_joints connector that repairs it, so detection and repair agree.
    Read-only."""
    if floor <= 0 or not net_pads or not net_segs:
        return
    from pcb_modification import (terminal_pad_web_shortfall,
                                  terminal_web_neck_exact,
                                  circular_pad_web_shortfall, _is_round_pad)
    from routing_utils import _to_pad_frame
    e = floor / 2.0

    def k(layer, x, y):
        return (layer, round(x, 4), round(y, 4))

    deg = {}
    for s in net_segs:
        if getattr(s, 'graphic', False):
            continue
        deg[k(s.layer, s.start_x, s.start_y)] = deg.get(k(s.layer, s.start_x, s.start_y), 0) + 1
        deg[k(s.layer, s.end_x, s.end_y)] = deg.get(k(s.layer, s.end_x, s.end_y), 0) + 1
    for s in net_segs:
        if getattr(s, 'graphic', False):
            continue
        r = s.width / 2.0
        if r < e - 1e-9:
            continue  # track thinner than the floor: no floor-width web exists
        for (ex, ey, nx, ny) in ((s.start_x, s.start_y, s.end_x, s.end_y),
                                 (s.end_x, s.end_y, s.start_x, s.start_y)):
            if deg.get(k(s.layer, ex, ey), 0) != 1:
                continue  # not a free end
            target = None
            for pad in net_pads:
                if getattr(pad, 'shape', None) not in ('rect', 'roundrect',
                                                       'oval', 'circle'):
                    continue  # custom-polygon pads have no closed-form web
                if not pad.size_x or not pad.size_y:
                    continue
                if not (s.layer in pad.layers or any('*' in L for L in pad.layers)):
                    continue
                if point_to_pad_distance(ex, ey, pad) < r - 1e-6:
                    target = pad
                    break
            if target is None:
                continue
            elx, ely = _to_pad_frame(ex, ey, target)
            nlx, nly = _to_pad_frame(nx, ny, target)
            if _is_round_pad(target):
                # A round pad has no corner, but it has a RIM: a cap landing
                # near the edge joins through a lens chord that can be far
                # below the floor. Skipping circles here reported NOTHING on
                # the same hazard the rect model catches (#416/#722).
                is_neck, _ = circular_pad_web_shortfall(
                    elx, ely, target.size_x / 2.0, r, e)
            else:
                is_neck, _ = terminal_pad_web_shortfall(
                    nlx, nly, elx, ely, target.size_x / 2.0,
                    target.size_y / 2.0, r, e)
            if is_neck and terminal_web_neck_exact(
                    pcb_data, net_id, s.layer, ex, ey, floor) is not False:
                # size=None: narrow pad joints bypass the --tolerance
                # filter, for the same reason soft joints do (see the note in
                # _check_soft_joints). Every magnitude available here is
                # INVERTED against severity -- a THINNER web is a worse
                # etch-open -- so a size filter would drop the severe findings
                # and keep the marginal ones.
                findings.append(_finding(
                    'narrow-pad-joint', name, s.layer, ex, ey,
                    f"terminal cap joins pad {target.component_ref}."
                    f"{target.pad_number} through a copper web below the "
                    f"{floor:.3f}mm min-track floor (connection_width hazard)",
                    size=None))


def check_weird(pcb_data: PCBData, net_patterns: Optional[List[str]] = None,
                thorough: bool = False, quiet: bool = True,
                tolerance: float = 0.1
                ) -> Tuple[List[Dict], List[Tuple[str, int]]]:
    """Run every check. Returns (findings, skipped_nets) where each finding is
    {'category', 'net', 'layer', 'x', 'y', 'detail', 'size'} and skipped_nets
    lists (net_name, segment_count) nets the removable-segment scan skipped.
    Findings whose characteristic size (mm) is below `tolerance` are dropped
    (0 = report everything). Read-only: pcb_data is not modified."""
    findings: List[Dict] = []
    skipped_nets: List[Tuple[str, int]] = []

    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        segs_by_net[s.net_id].append(s)
    vias_by_net = defaultdict(list)
    for v in pcb_data.vias:
        vias_by_net[v.net_id].append(v)
    zones_by_net = defaultdict(list)
    for z in (pcb_data.zones or []):
        zones_by_net[z.net_id].append(z)

    copper_layers = (getattr(pcb_data.board_info, 'copper_layers', None)
                     or ['F.Cu', 'B.Cu'])

    # Connection-width floor for the terminal-web check (#416): the floor the
    # board's project grades connection_width at (the author's min_connection,
    # else min_track_width), the same call kicad_drc_compare stages KiCad's
    # grade from (#1187) -- so re-routing one net cannot flip another's
    # verdict. A board with no project falls back to its thinnest track.
    from fix_kicad_drc_settings import connection_width_floor
    min_track_w = connection_width_floor(pcb_data)

    net_ids = set(segs_by_net) | set(vias_by_net)
    check_ids = []
    for net_id in sorted(net_ids):
        if net_id == 0:
            continue  # unconnected copper has no same-net semantics
        if net_patterns is not None and not matches_any_pattern(
                _net_name(pcb_data, net_id), net_patterns):
            continue
        check_ids.append(net_id)

    if not quiet:
        print(f"Checking {len(check_ids)} nets "
              f"({len(pcb_data.segments)} segments, {len(pcb_data.vias)} vias)...")

    for net_id in check_ids:
        name = _net_name(pcb_data, net_id)
        net_segs = segs_by_net.get(net_id, [])
        net_vias = vias_by_net.get(net_id, [])
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_zones = zones_by_net.get(net_id, [])
        has_zone = bool(net_zones)

        soft_pts = _check_soft_joints(net_id, name, net_segs, net_vias,
                                      net_pads, findings, copper_layers)
        _check_dangles(net_id, name, net_segs, net_vias, net_pads, net_zones,
                       soft_pts, findings, join_tol=tolerance or 0.0,
                       copper_layers=copper_layers)
        _check_orphan_islands(net_id, name, net_segs, net_vias, net_pads,
                              net_zones, findings)
        removable = _strict_removable(net_id, name, net_segs, net_vias,
                                      net_pads, has_zone, copper_layers,
                                      thorough, skipped_nets, min_track_w)
        _check_cycles(net_id, name, net_segs, net_vias, net_pads, has_zone,
                      findings, removable)
        _check_removable(net_id, name, net_segs, removable, findings)
        _check_stacked(net_id, name, net_segs, net_vias, findings)
        _check_kicad_dangling(net_id, name, net_segs, net_vias, net_pads,
                              net_zones, copper_layers, findings)
        _check_unsupported_vias(net_id, name, net_segs, net_vias, net_pads,
                                net_zones, copper_layers, findings)
        _check_terminal_web(pcb_data, net_id, name, net_segs, net_pads,
                            min_track_w, findings)

    if tolerance and tolerance > 0:
        findings = [f for f in findings
                    if f.get('size') is None or f['size'] + 1e-9 >= tolerance]
    return findings, skipped_nets


def print_report(findings: List[Dict], skipped_nets: List[Tuple[str, int]],
                 max_print: int = 20) -> None:
    by_cat = defaultdict(list)
    for f in findings:
        by_cat[f['category']].append(f)
    if findings:
        print(f"\nFOUND {len(findings)} WEIRD THINGS:\n")
        # CATEGORIES is hand-maintained, so a check whose category was
        # never registered would print NOTHING while still counting toward the
        # headline and the exit code (#696: that shipped, and blocked DONE with
        # nothing on screen to act on). Report the strays too, after the
        # registered ones, so a future omission can only ever cost ordering.
        unregistered = [c for c in sorted(by_cat) if c not in CATEGORIES]
        for cat in CATEGORIES + unregistered:
            items = by_cat.get(cat, [])
            print(f"  {cat}: {len(items)}")
            limit = len(items) if (max_print is not None and max_print <= 0) \
                else max_print
            for f in items[:limit]:
                print(f"    net {f['net']} ({f['layer']}) at "
                      f"({f['x']:.3f}, {f['y']:.3f}): {f['detail']}")
            if len(items) > limit:
                print(f"    ... and {len(items) - limit} more "
                      f"(use --max-print 0 to show all)")
    else:
        print("\nNO WEIRD THINGS FOUND!")
    if skipped_nets:
        print(f"\n  removable-segment: skipped {len(skipped_nets)} net(s) "
              f"with >{MAX_SEGS_PER_NET} segments "
              f"(pass --thorough to check them):")
        for nm, cnt in skipped_nets[:10]:
            print(f"    {nm}: {cnt} segments")
        if len(skipped_nets) > 10:
            print(f"    ... and {len(skipped_nets) - 10} more")


def main():
    parser = argparse.ArgumentParser(
        description='Check PCB for weird copper hygiene issues '
                    '(dangles, soft joints, loops, removable/stacked copper, '
                    'floating vias, narrow pad joints). Read-only: never '
                    'modifies the board.')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--nets', '-n', nargs='+', default=None,
                        help='Net name patterns to check (fnmatch wildcards '
                             'supported, e.g., "*lvds*")')
    parser.add_argument('--thorough', action='store_true',
                        help='Run the removable-segment scan on nets with '
                             f'>{MAX_SEGS_PER_NET} segments too (slow)')
    parser.add_argument('--tolerance', type=float, default=0.1,
                        help='Minimum finding size in mm (dangle/tail length, '
                             'gap, duplicated-copper length, via diameter); '
                             'smaller findings are dropped. Default 0.1; use '
                             '0 to report everything. soft-joint, '
                             'narrow-pad-joint and kicad-dangling carry no '
                             'size and are ALWAYS reported -- smaller is '
                             'worse, or KiCad warns at any length.')
    parser.add_argument('--max-print', type=int, default=20,
                        help='Max findings printed per category '
                             '(<=0 prints all; default 20)')
    args = parser.parse_args()

    print(f"Loading PCB file: {args.pcb}")
    pcb_data = parse_kicad_pcb(args.pcb)
    findings, skipped_nets = check_weird(pcb_data, args.nets,
                                          tolerance=args.tolerance,
                                         thorough=args.thorough, quiet=False)
    print_report(findings, skipped_nets, max_print=args.max_print)
    sys.exit(1 if findings else 0)


if __name__ == '__main__':
    main()
