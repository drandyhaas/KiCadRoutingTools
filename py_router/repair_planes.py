#!/usr/bin/env python3
"""
Repair Planes - Connects disconnected regions within power plane zones.

(Renamed from route_disconnected_planes.py, 2026-08-04.) After power planes
are created, regions may be effectively split due to vias and traces from
other nets cutting through the plane. The repair engine detects disconnected
regions and routes wide, short tracks between them to ensure electrical
continuity.

NOT A CHAIN STEP since #562: route.py's in-run plane finalize calls the
`repair_planes` engine function below on every route step, so the default
chain never invokes this file. The CLI remains as a STANDALONE utility for
boards routed outside the chain (e.g. a hand-routed board whose pours need
welding). Recorded manifests that reference route_disconnected_planes.py
are historical records, not runnable chains.

Usage:
    # Auto-detect all zones in PCB:
    python py_router/repair_planes.py input.kicad_pcb output.kicad_pcb

    # Specific nets and layers:
    python py_router/repair_planes.py input.kicad_pcb output.kicad_pcb \\
        --nets GND --plane-layers B.Cu
"""
from __future__ import annotations

import env_knobs
import sys
import os
import math
import argparse
import json
from dataclasses import replace
from typing import List, Tuple, Dict, Optional, Set

# Run startup checks first
from startup_checks import exit_on_error_if_main
# Stays at module scope, ABOVE the heavy imports, so a missing dep is
# reported before numpy/grid_router blow up with something cryptic. But it
# raises instead of exiting when this module is IMPORTED rather than run,
# so pytest can still collect a suite on a checkout with no built router
# (#457 item 3).
exit_on_error_if_main(__name__)

from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via, KICAD_10_MIN_VERSION, pad_is_plated_through
from kicad_writer import generate_segment_sexpr, generate_gr_line_sexpr, generate_via_sexpr
from routing_config import GridRouteConfig
from plane_io import extract_zones
from plane_region_connector import route_disconnected_regions, build_base_obstacles
import plane_pad_tap
from plane_pad_tap import (find_unconnected_plane_pads, tap_pad_with_escalation,
                           SharedViaMaps)
from plane_component_oracle import PlaneComponentOracle
from plane_blocker_detection import find_route_blocker_from_frontier, find_via_position_blocker
from terminal_colors import GREEN, RED, YELLOW, RESET
import routing_defaults as defaults
import re

# Outcome of the end-of-run self-reconnect of rip-blocker-nets casualties
# (#347); read by main() for the JSON_SUMMARY. None = no reconnect ran.
LAST_RIPPED_RECONNECT: Optional[Dict] = None

# Casualty nets that ship STILL OPEN -- ripped to clear a corridor, not
# reconnected, and not restorable. Run-7 finding A10: this state reached a red
# log line and nothing else. The JSON_SUMMARY carried counts without names, and
# main() returned None so the process exited 0 -- a chain step that silently
# consumed two nets was caught only because a later board_score happened to be
# read by a human. Names here, and an exit code from main().
LAST_RIPPED_STILL_OPEN: List[str] = []
LAST_RIPPED_CUSTODY: Optional[Dict] = None


def plane_tap_launch_layers(pad, zone_layers, routing_layers) -> List[str]:
    """Copper layers a last-resort plane tap may launch from, in
    deterministic routing-layer order (#494).

    - NPTH hole: no copper exists at all (#328) -- never a tap source.
    - PLATED barrel / '*.Cu': copper on EVERY layer, so the tap may launch
      from any of them. Prefer this net's ZONE layers, where the pour it
      has to reach lives; fall back to all routing layers when the net's
      zones are not on a routing layer.
    - Otherwise (SMD): its own concrete copper layers.

    The old form resolved ONE layer with a concrete-layer filter
    (``endswith('.Cu') and not startswith('*')``), so a '*.Cu' through-hole
    pad resolved to None and was skipped -- an independent block that
    survives removing the plated guard on its own (#492 measured both).
    """
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return []
    layers = getattr(pad, 'layers', None) or []
    if '*.Cu' in layers or pad_is_plated_through(pad):
        return [l for l in routing_layers if l in (zone_layers or ())] \
            or list(routing_layers)
    return [l for l in routing_layers if l in layers]


def pad_floating_entries(disconnected_pads, pad) -> int:
    """How many still-disconnected entries the authoritative check reports
    for `pad`, across all layers."""
    px, py = round(pad.global_x, 3), round(pad.global_y, 3)
    return sum(1 for (x, y, _l, ref) in (disconnected_pads or [])
               if ref == pad.component_ref
               and round(x, 3) == px and round(y, 3) == py)


def pad_repair_rejected(before_dp, after_dp, pad, legacy=False) -> bool:
    """True if a just-added repair must be UNDONE.

    `legacy=True` is the pre-#494 rule (any remaining entry for the pad
    vetoes), kept so KICAD_NO_SWEEP_PLATED=1 reproduces main exactly and
    the A/B differs by one switch. Otherwise: keep only on progress.
    """
    if legacy:
        return pad_floating_entries(after_dp, pad) > 0
    return not pad_repair_made_progress(
        pad_floating_entries(before_dp, pad),
        pad_floating_entries(after_dp, pad), pad)


def pad_repair_made_progress(before, after, pad) -> bool:
    """True if a just-added repair STRICTLY improved the authoritative
    verdict for `pad` -- the custody test that decides whether to keep it.

    Progress, not absence (#494). Two weaker rules both fail:

    - Keying on the pad alone ("is it listed at all?") vetoes a genuine
      repair on a PLATED pad: check_net_connectivity can list such a pad
      once per copper layer, so a pad truly joined on B.Cu stays listed on
      F.Cu and its good copper is thrown away.
    - Keying on (pad, repaired layer) passes VACUOUSLY whenever the pad's
      entry sits on a different layer than the one the tap launched from.
      Measured on nrfmicro U1.24: its only entry is F.Cu, the sweep
      repaired from B.Cu, so "no B.Cu entry" was trivially true and a via
      that changed nothing (net stayed at 2 components / 1 floating pad)
      was kept as dead copper.

    Counting entries before vs after is immune to both: clearing one of a
    plated pad's several entries counts as progress, while a repair that
    leaves the verdict untouched does not.
    """
    return after < before



def _rip_net_from_pcb(pcb_data: PCBData, rip_net_id: int):
    """Remove a net's segments and vias from pcb_data so a blocked repair can
    re-attempt through the freed space. Returns (segments, vias) removed."""
    # graphic=True copper is immutable input art (#337): never ripped.
    rsegs = [s for s in pcb_data.segments
             if s.net_id == rip_net_id and not getattr(s, 'graphic', False)]
    rvias = [v for v in pcb_data.vias if v.net_id == rip_net_id]
    if rsegs:
        pcb_data.segments = [s for s in pcb_data.segments
                             if s.net_id != rip_net_id or getattr(s, 'graphic', False)]
    if rvias:
        pcb_data.vias = [v for v in pcb_data.vias if v.net_id != rip_net_id]
    return rsegs, rvias


def _via_site_consensus_blocker(pad, pcb_data, blocker_config, net_id,
                                protected_net_ids, exclude_net_ids,
                                max_search_radius):
    """The net most often blocking CANDIDATE VIA SITES around the pad (#329).

    The frontier blocker only names whoever stopped the failed ROUTE attempt;
    on ottercast the C69.2 tap ripped three frontier nets while the net whose
    trace actually denied every good via site (Net-(C63-Pad1), 0.325mm from
    the best spot) was never identified. Vote find_via_position_blocker over a
    coarse ring of sites within the search radius and return the most common
    non-protected, not-yet-ripped net."""
    from collections import Counter
    votes = Counter()
    step = max(blocker_config.grid_step * 4, 0.2)
    # Vote only the NEAR band: distant sites belong to other neighborhoods and
    # dilute the vote toward whatever net dominates the region at large
    # (ottercast: a full-radius vote elected C64 while C63 held every site the
    # pad could actually use). Closer sites also count for more.
    r = min(1.2, max_search_radius) if max_search_radius > 0 else 1.2
    n = max(3, int(r / step))
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            d = math.hypot(i, j) * step
            if d < 0.2 or d > r:
                continue
            b = find_via_position_blocker(
                pad.global_x + i * step, pad.global_y + j * step,
                pcb_data, blocker_config, net_id, protected_net_ids, quiet=True)
            if b is not None and b not in protected_net_ids and b not in exclude_net_ids:
                votes[b] += 1.0 / (0.3 + d)
    return votes.most_common(1)[0][0] if votes else None


# Restore policy for a rip-up blocker whose copper partly conflicts with the new
# tap (#509). Default FULL RIP: drop the whole net and let the mandated reconnect
# re-route it fresh, which is route.py's contract. Piece-level partial restore --
# keep the non-colliding fragments and have the reconnect close the gap -- is the
# old behaviour, kept behind this knob for A/B.
#
# Measured on spartan6_6layer's plane-repair step (all arms DRC-clean at 0.1):
#   piece-level                          connectivity issues 14
#   full rip                             connectivity issues  8
# The reconnect withdrew ~6.8 pieces per net across 36 nets there and re-threaded
# them anyway, so the fragments were mostly wasted work. ONE BOARD, ONE STEP --
# set KICAD_PLANE_PARTIAL_RESTORE=1 to A/B the old policy on a corpus replay.
_PLANE_PARTIAL_RESTORE = env_knobs.PLANE_PARTIAL_RESTORE

# #517 arm 3 (#480): most-constrained-first repair ordering + immediate
# reconnect of ripped nets (ordering and window-shrinking are one knob).
# Experiment knob, default off; every branch below is gated on it.
# 'order-only' keeps the net ordering + two-phase pad schedule but SKIPS the
# per-pad immediate reconnect: repeated in-process batch_route calls route
# against a stale world (arm-D spartan6 hit 885 DRC from grazes all over the
# board; the write-list purge fixed the re-rip double-emission but not this).
# The window-shrink half needs its own debugging session; the ordering half
# is measurable without it.
# #517: repair ordering + reconnect timing. DEFAULT is 'adaptive' -- the
# arm-I winner from the 2026-07-30 A/B ladder: most-constrained-first net
# order, two-phase pads (free-space taps first, rip-requiring pads last),
# and immediate reconnects only when the phase-2 rip queue is SHORT at
# phase start. The G/H split was cleanly predicted by rip pressure: boards
# whose phases held <=12 rip pads won with immediate reconnects (astro +5,
# zynq2 +4, apple2e full), boards with >=19 lost hard (spartan6 -15,
# allwinner -12: each early reconnect locks routes into a still-evolving
# world). Corpus gate (sets 1-5, 75 boards): completion flat, drc -7,
# rips -58%, refusals -49%, ~15% faster.
# KICAD_PLANE_REPAIR_ORDERING: 'adaptive' (default) | 'order-only' (no
# immediate reconnects) | '1'/'full' (always-immediate) | '0'/'off'/'none'
# (pre-#517 behavior, for A/B).
_ORDERING_MODE = os.environ.get('KICAD_PLANE_REPAIR_ORDERING', 'adaptive') \
    .strip().lower() or 'adaptive'
_PLANE_REPAIR_ORDERING = _ORDERING_MODE in ('1', 'full', 'order-only',
                                            'orderonly', 'adaptive')
_PLANE_IMMEDIATE_RECONNECT = _ORDERING_MODE in ('1', 'full')
_PLANE_ADAPTIVE_RECONNECT = _ORDERING_MODE == 'adaptive'
try:
    _PLANE_IMMEDIATE_MAX_PENDING = int(os.environ.get(
        'KICAD_PLANE_IMMEDIATE_MAX_PENDING', '14') or 14)
except ValueError:
    _PLANE_IMMEDIATE_MAX_PENDING = 14

# #540: the rip-pressure count above cannot see CORRIDOR COUPLING. A short
# phase whose rips are bus siblings (allwinner_h3_ddr3: a 9-pad DDR phase, all
# victims /DDR3 16x1/SDQ*, SA*, ...) passes the <=14 gate, and each immediate
# reconnect then locks the shared corridor against the siblings ripped by the
# NEXT pad (44 complete vs 53 under order-only). zynq2's 12-pad phase of
# INDEPENDENT signals (+1V8, PL_CLK, /TX_EN, ...) is a genuine immediate win
# (+4), so the fix is per-net, not a phase flip: a ripped net whose name is
# bus-coupled to any PENDING casualty (an earlier phase's deferred batch) or
# to any net this phase already ripped (even ones whose immediate reconnect
# succeeded -- daisho's one-sibling-per-pad rip sequence) defers to the
# end-of-run batch (where siblings negotiate the corridor together);
# uncoupled nets keep reconnecting immediately. Coupling = shared NON-ROOT sheet prefix (>=2 nets -- '/A5' and
# '/TX1' live in the root sheet and do not couple) or shared digit-stripped
# name stem (>=3 nets, stem >=2 chars, power-rail names excluded so +1V8/+1V0
# never couple). KICAD_PLANE_IMMEDIATE_COUPLING=0 restores the pure count gate.
_PLANE_IMMEDIATE_COUPLING = os.environ.get(
    'KICAD_PLANE_IMMEDIATE_COUPLING', '1').strip().lower() \
    not in ('0', 'off', 'no')
_POWER_RAIL_RE = re.compile(r'^[+-]|^(vcc|vdd|vss|vbus|vbat|gnd)', re.IGNORECASE)
_BUS_STEM_STRIP_RE = re.compile(r'(?:[0-9]+|[PN])+$')


def _bus_group_keys(name: str) -> List[Tuple[str, str]]:
    """Coupling-group keys for one net name: its hierarchical sheet path (if
    not the root sheet) and its trailing-index-stripped stem (SDQ5/SDQ14 ->
    'SDQ', SDQS0P/SDQS0N -> 'SDQS'; names without a trailing index have no
    stem key -- SCAS/SRAS only couple through their sheet)."""
    keys: List[Tuple[str, str]] = []
    base = name
    if '/' in name:
        pfx, base = name.rsplit('/', 1)
        if pfx:
            keys.append(('sheet', pfx))
    if base and not _POWER_RAIL_RE.match(base):
        stem = _BUS_STEM_STRIP_RE.sub('', base)
        if len(stem) >= 2 and stem != base:
            keys.append(('stem', stem))
    return keys


def _corridor_coupled_ids(candidate_ids, pending_rip_ids, pcb_data) -> Set[int]:
    """#540: the subset of candidate_ids that are bus-coupled to another net
    in pending_rip_ids (which includes the candidates themselves). The
    population must be the UNION of two histories:
    - every net currently off the board awaiting reconnect, including earlier
      phases' deferred batches (allwinner's lone VCC-DRAM-phase rip of SDQ1
      had no phase sibling, but dozens of /DDR3 siblings from the GND phase
      sat ripped, and its immediate reconnect claimed their corridors with
      maximum freedom), and
    - every net ripped earlier in THIS phase even if its immediate reconnect
      SUCCEEDED (daisho ripped /ddr2/A1, VREF, DQ11 one pad at a time with a
      successful reconnect between each -- pending-only sees a population of
      one at every decision and never fires, yet each reconnect locks the
      corridor against the sibling the next pad rips)."""
    if not _PLANE_IMMEDIATE_COUPLING:
        return set()

    def _nm(nid):
        return pcb_data.nets[nid].name if nid in pcb_data.nets else ''

    counts: Dict[Tuple[str, str], int] = {}
    for nid in set(pending_rip_ids):
        for k in _bus_group_keys(_nm(nid)):
            counts[k] = counts.get(k, 0) + 1
    coupled = set()
    for nid in set(candidate_ids):
        for k in _bus_group_keys(_nm(nid)):
            if counts.get(k, 0) >= (2 if k[0] == 'sheet' else 3):
                coupled.add(nid)
                break
    return coupled

# #517 arm 4, DEFAULT ON: at the end-of-run reconnect, try a verbatim
# identity-restore of each casualty's ORIGINAL copper before re-routing it
# (only nets whose corridor is still free restore; the rest re-route as
# before). Zero measured regressions; dormant under adaptive ordering but
# covers the flows ordering does not. KICAD_PLANE_RESTORE_FIRST=0 disables.
_PLANE_RESTORE_FIRST = os.environ.get(
    'KICAD_PLANE_RESTORE_FIRST', '1').strip().lower() not in ('0', 'off', 'no')

# Shared with route_planes (#508 finding 1: its GUI reconnect had neither
# reconcile mechanism). Re-exported here so existing call sites and
# tests/test_463_partial_restore_stale_emit.py keep driving the REAL function.
from plane_write_reconcile import (consume_inner_strips,  # noqa: E402,F401
                                   drop_withdrawn_partial_restores)

# #517 arm 2 (#343): reserve vacated ripped-net corridors against OTHER
# claimants. 'soft' = cost stamps in tap/region routing maps + soft via-site
# preference; 'hard' = pre-#342 via-map over-block baseline. None = off
# (default; every hook below no-ops).
from plane_corridor_ghosts import CorridorGhosts, softblock_mode

# #540 item 2: reconnect-side ghost pricing. The #517 soft arm never reached
# the reconnect batch_route calls -- the pass whose corridor claims ROSE in
# the attribution histogram (249 vs 184) -- and its 0.1mm default cell cost
# was likely too weak at repair-pass timescale. When the registry's mode
# enables the pass-through ('soft' or the reconnect-only arm
# KICAD_PLANE_RIP_SOFTBLOCK=reconnect), every reconnect/gate batch_route
# receives the pending casualties' ghosts at this stronger cost.
try:
    _RECONNECT_GHOST_COST = float(os.environ.get(
        'KICAD_PLANE_RECONNECT_GHOST_COST', '0.5') or 0.5)
except ValueError:
    _RECONNECT_GHOST_COST = 0.5
try:
    _RECONNECT_GHOST_RADIUS = float(os.environ.get(
        'KICAD_PLANE_RECONNECT_GHOST_RADIUS', '1.0') or 1.0)
except ValueError:
    _RECONNECT_GHOST_RADIUS = 1.0


def _ghost_kwargs(corridor_ghosts, batch_net_ids):
    """Extra batch_route kwargs pricing pending casualties' corridors
    (#540 item 2), or {} when the arm is off / nothing is pending. The
    avoidance-cost overrides ride along only when ghosts do, so the arm-off
    default batch behavior is untouched."""
    if corridor_ghosts is None or not corridor_ghosts.reconnect_passthrough:
        return {}
    ghosts = corridor_ghosts.external_ghosts(exclude_net_ids=batch_net_ids)
    if not ghosts:
        return {}
    return {'external_ripped_ghosts': ghosts,
            'ripped_route_avoidance_cost': _RECONNECT_GHOST_COST,
            'ripped_route_avoidance_radius': _RECONNECT_GHOST_RADIUS}


def _tap_pad_with_ripup(pad, pad_layer, net_id, pcb_data, tap_config, blocker_config,
                        max_search_radius, via_size, via_drill, max_rip_nets,
                        protected_net_ids, first_failure, ripped_net_ids, verbose,
                        distant_trace_radius=0.0, shared_via_maps=None,
                        partial_restores=None, plane_oracle=None,
                        corridor_ghosts=None, write_lists=None):
    """A plane-net pad too small to drop a via in needs a trace to the plane (or
    to an adjacent same-net pad); if signal nets block that trace, rip them (up
    to max_rip_nets), retry the tap. Identifies the blocker from the failed
    route's frontier (or, when that repeats/runs dry, the consensus net denying
    the candidate via sites, #329), never ripping a protected (plane) net.

    On SUCCESS the ripped net ids are appended to ripped_net_ids for the later
    route.py reconnect pass. On FAILURE every ripped net's copper is restored
    (#329) -- but PIECE BY PIECE, against the live board, exactly like the
    success path above (#495/#496).

    The old failure path restored unconditionally, on the reasoning that "a
    failed tap adds no copper, so an immediate restore cannot create the #141
    restore-shorts". That is false in general: earlier pads' taps and region
    straps placed during this same repair DO land in the freed corridor, and
    the blanket restore then put a failed net's stale copper back on top of
    them (rp2350 /RP2354A/FPGA.MOSI: both RN2.2/RN2.3 taps failed and its via
    went back over a GND strap, 0.114mm overlap at a 0.09 rule). That is
    precisely the behaviour issue #141 removed and this module's own docstring
    says was removed. Colliding pieces are left ripped for the mandated
    follow-up route.py pass (the shared rip_up_reroute.restore_net contract);
    non-colliding copper is still given back, so #329's zero-copper nets do
    not come back either.
    Returns a successful TapResult, or None."""
    failure = first_failure
    ripped_local = []  # (net_id, segments, vias) in rip order
    ripped_ids_local = set()
    for attempt in range(max_rip_nets):
        blocker = None
        # The frontier blocker names whoever stopped the failed ROUTE; the net
        # denying the via SITES can be a different one that the frontier never
        # reaches (ottercast C69.2 ripped 3 fresh frontier nets while C63 held
        # every good site). Spend the LAST rip on the site-consensus net.
        if attempt < max_rip_nets - 1:
            if failure.blocked_cells:
                blocker = find_route_blocker_from_frontier(
                    failure.blocked_cells, pcb_data, blocker_config, net_id, protected_net_ids)
            else:
                blocker = find_via_position_blocker(
                    pad.global_x, pad.global_y, pcb_data, blocker_config, net_id, protected_net_ids)
            if blocker in ripped_ids_local:
                blocker = None  # frontier keeps naming an already-ripped net
        if blocker is None or blocker in protected_net_ids:
            blocker = _via_site_consensus_blocker(
                pad, pcb_data, blocker_config, net_id, protected_net_ids,
                ripped_ids_local, max_search_radius)
        if blocker is None or blocker in protected_net_ids:
            break
        bname = pcb_data.nets[blocker].name if blocker in pcb_data.nets else f"net_{blocker}"
        print(f"{RED}blocked by {bname} - ripping{RESET}...", end=" ", flush=True)
        if shared_via_maps is not None and \
                not (corridor_ghosts is not None and corridor_ghosts.mode == 'hard'):
            # Remove the blocker's stamps from the shared via maps BEFORE its
            # copper leaves pcb_data (the stamps are computed from it), then
            # resync the copper counts after the rip (#263). In #517 arm 2
            # HARD mode the stamps deliberately STAY (pre-#342 over-block
            # baseline; note_net_restored below is gated symmetrically so the
            # refcounts never double-add).
            shared_via_maps.note_net_ripped(blocker)
        rsegs, rvias = _rip_net_from_pcb(pcb_data, blocker)
        if write_lists is not None:
            # A net can already have THIS RUN's copper in the write list (arm
            # 3's immediate reconnect routes mid-pad-loop; a later pad may rip
            # that same net again). The rip edits pcb_data only, so without
            # this purge the output ships the superseded copper on top of
            # whatever routes through the corridor afterwards (muzy_zynq2:
            # /G18 reconnected twice -> 99 kicad-DRC). board == write list.
            _wsegs, _wvias = write_lists
            _wsegs[:] = [d for d in _wsegs if d.get('net_id') != blocker]
            _wvias[:] = [d for d in _wvias if d.get('net_id') != blocker]
        if corridor_ghosts is not None:
            # #517 arm 2: reserve the vacated corridor against other claimants
            # while this net is off the board.
            corridor_ghosts.set_net(blocker, rsegs, rvias)
        ripped_local.append((blocker, rsegs, rvias))
        ripped_ids_local.add(blocker)
        from route_trace import plane_capture as _plane_capture
        _plane_capture(pcb_data, 'plane-rip', blocker, bname)  # individual rip frame
        if shared_via_maps is not None:
            shared_via_maps.resync()
            if plane_pad_tap._TAP_MAP_VERIFY:
                # Rips are the #208-risk removal path: assert the incrementally
                # updated maps match a fresh rebuild of the post-rip board.
                shared_via_maps.verify_maps_full()
        result = tap_pad_with_escalation(
            pad, pad_layer, net_id, pcb_data, tap_config,
            max_search_radius=max_search_radius, via_size=via_size, via_drill=via_drill,
            verbose=verbose, fine_for_all=True, distant_trace_radius=distant_trace_radius,
            shared_via_maps=shared_via_maps, plane_oracle=plane_oracle,
            corridor_ghosts=corridor_ghosts,
            # This tap's own rips freed this corridor FOR the tap: their
            # ghosts must not repel it.
            ghost_exclude_ids=frozenset(ripped_ids_local))
        if result.success:
            # Collision-checked restore on SUCCESS too (#329): give back every
            # ripped net whose copper does not conflict with the NEW tap
            # copper. Only genuinely conflicting nets stay ripped for the
            # route.py reconnect pass -- shipping every rip gambled N routed
            # nets on that pass (which the recorded chains often run with
            # --max-ripup 0): ottercast ended with 5 zero-copper nets when
            # only the true corridor nets actually conflicted.
            from plane_blocker_detection import _restored_piece_collides
            new_segs = list(result.segments or [])
            new_vias = [result.via] if result.via else []
            clr = result.clearance_used or tap_config.clearance
            for blocker, rsegs, rvias in ripped_local:
                keep_segs, keep_vias, dropped = [], [], 0
                for s in rsegs:
                    sd = {'start': (s.start_x, s.start_y),
                          'end': (s.end_x, s.end_y),
                          'width': s.width, 'layer': s.layer}
                    if _restored_piece_collides(sd, None, new_vias, new_segs,
                                                via_size, clr):
                        dropped += 1
                    else:
                        keep_segs.append(s)
                for v in rvias:
                    vd = {'x': v.x, 'y': v.y, 'size': v.size}
                    if _restored_piece_collides(None, vd, new_vias, new_segs,
                                                via_size, clr):
                        dropped += 1
                    else:
                        keep_vias.append(v)
                from pcb_modification import drop_orphan_restore_pieces
                dropped += drop_orphan_restore_pieces(
                    keep_segs, keep_vias, blocker, pcb_data)
                if not keep_segs and not keep_vias:
                    # Nothing restorable: honest full rip for the reconnect pass.
                    if blocker not in ripped_net_ids:
                        ripped_net_ids.append(blocker)
                    continue  # (#517 arm 2: ghost stays FULL, set at the rip)
                pcb_data.segments.extend(keep_segs)
                pcb_data.vias.extend(keep_vias)
                if shared_via_maps is not None and \
                        not (corridor_ghosts is not None
                             and corridor_ghosts.mode == 'hard'):
                    # (hard mode never removed the stamps at the rip)
                    shared_via_maps.note_net_restored(blocker)
                if dropped:
                    # Partial: the writer must strip the net's input copper and
                    # emit the kept pieces (board==file), and the reconnect
                    # pass closes the small gap instead of re-threading the
                    # whole net through congestion.
                    if partial_restores is not None:
                        partial_restores.append((blocker, keep_segs, keep_vias, dropped))
                        bn = pcb_data.nets[blocker].name if blocker in pcb_data.nets else blocker
                        print(f"(partial restore {bn}: -{dropped} piece(s))", end=" ", flush=True)
                        if corridor_ghosts is not None:
                            # Restored copper is its own obstacle; the ghost
                            # shrinks to the genuinely dropped legs (#517).
                            corridor_ghosts.set_net(
                                blocker,
                                [s for s in rsegs if s not in keep_segs],
                                [v for v in rvias if v not in keep_vias])
                    elif blocker not in ripped_net_ids:
                        # No partial channel (defensive): fall back to full rip.
                        pcb_data.segments = [x for x in pcb_data.segments if x not in keep_segs]
                        pcb_data.vias = [x for x in pcb_data.vias if x not in keep_vias]
                        ripped_net_ids.append(blocker)
                        # (#517 arm 2: ghost stays FULL)
                elif corridor_ghosts is not None:
                    # Fully restored: nothing vacated, nothing to reserve.
                    corridor_ghosts.drop_net(blocker)
            from route_trace import plane_capture as _plane_capture
            _plane_capture(pcb_data, 'plane-restore', net_id)  # restored non-conflicting pieces
            return result
        failure = result
    # FINAL FAILURE: restore every ripped net's copper (#329), but only the
    # pieces that do not collide with whatever is on the board NOW (#495/#496).
    # Reuses the shared rip-up/restore collision test (rip_up_reroute, #134)
    # rather than a private copy: a piece is refused exactly when foreign
    # copper moved into the corridor while this net was ripped.
    if ripped_local:
        from rip_up_reroute import _saved_route_collides
        from pcb_modification import drop_orphan_restore_pieces
        # All nets ripped in THIS transaction are "own": they coexisted on the
        # input board, so a sibling's copper is not a collision to refuse (and
        # siblings restored earlier in this loop must not fail the later ones).
        _own = [b for b, _s, _v in ripped_local]
        clr = tap_config.clearance
        n_restored = 0
        for blocker, rsegs, rvias in reversed(ripped_local):
            keep_segs = [s for s in rsegs
                         if not _saved_route_collides(
                             {'new_segments': [s], 'new_vias': []},
                             pcb_data, _own, clr)]
            keep_vias = [v for v in rvias
                         if not _saved_route_collides(
                             {'new_segments': [], 'new_vias': [v]},
                             pcb_data, _own, clr)]
            dropped = (len(rsegs) - len(keep_segs)) + (len(rvias) - len(keep_vias))
            dropped += drop_orphan_restore_pieces(
                keep_segs, keep_vias, blocker, pcb_data)
            if not keep_segs and not keep_vias:
                # Nothing restorable: honest full rip for the reconnect pass.
                if blocker not in ripped_net_ids:
                    ripped_net_ids.append(blocker)
                continue  # (#517 arm 2: ghost stays FULL, set at the rip)
            pcb_data.segments.extend(keep_segs)
            pcb_data.vias.extend(keep_vias)
            if shared_via_maps is not None and \
                    not (corridor_ghosts is not None
                         and corridor_ghosts.mode == 'hard'):
                # (hard mode never removed the stamps at the rip)
                shared_via_maps.note_net_restored(blocker)
            if corridor_ghosts is not None:
                # Ghost shrinks to what stayed off the board (empty = drop).
                corridor_ghosts.set_net(
                    blocker,
                    [s for s in rsegs if s not in keep_segs],
                    [v for v in rvias if v not in keep_vias])
            n_restored += 1
            if dropped:
                # Partial: the writer must strip the net's input copper and
                # emit the kept pieces (board==file); the reconnect pass closes
                # the gap. Mirrors the success path above.
                if partial_restores is not None:
                    partial_restores.append((blocker, keep_segs, keep_vias, dropped))
                    bn = pcb_data.nets[blocker].name if blocker in pcb_data.nets else blocker
                    print(f"(partial restore {bn}: -{dropped} piece(s))", end=" ", flush=True)
                elif blocker not in ripped_net_ids:
                    # No partial channel (defensive): fall back to full rip.
                    pcb_data.segments = [x for x in pcb_data.segments if x not in keep_segs]
                    pcb_data.vias = [x for x in pcb_data.vias if x not in keep_vias]
                    ripped_net_ids.append(blocker)
                    n_restored -= 1
                    if corridor_ghosts is not None:
                        # Back to a full rip: reserve the whole footprint again.
                        corridor_ghosts.set_net(blocker, rsegs, rvias)
        print(f"(restored {n_restored}/{len(ripped_local)} ripped net(s))",
              end=" ", flush=True)
        from route_trace import plane_capture as _plane_capture
        _plane_capture(pcb_data, 'plane-restore')  # tap failed: non-conflicting copper back
    return None


def _report_unrouted_ripped_nets(pcb_data, ripped_net_ids):
    """Report the nets ripped to clear blocked pad repairs and left UNROUTED.

    repair_planes no longer re-routes ripped nets in-step (issue #141
    reverted -- its restore-on-failure put a failed net's original copper back on
    top of whatever had been routed through its freed corridor, shorting them). The
    ripped nets are stripped from the output and reconnected by a route.py pass run
    afterward, which handles rip-up/restore safely against the live obstacle map.
    """
    ripped_names = [pcb_data.nets[r].name for r in ripped_net_ids if r in pcb_data.nets]
    if not ripped_names:
        return
    print(f"\nNote: {len(ripped_names)} net(s) ripped to clear pad repairs were left "
          f"UNROUTED; reconnect them with route.py (matching signal width): "
          f"{', '.join(ripped_names)}")


def extract_zone_properties(pcb_data) -> Dict[Tuple[str, str], Dict]:
    """Per-zone (clearance, min_thickness) keyed by (net_name, layer).

    Reads ``pcb_data.zones`` -- the parser already parses these fields. This used
    to re-implement zone parsing as a regex over the FILE, which had two
    independent problems (#493 follow-up):

    1. It was DEAD on every modern board. The pattern required
       ``(zone\n (net <int>)`` and then looked up ``(net_name "...")``, but
       KiCad 10 collapsed both into a single quoted ``(net "/GND")``. Every one
       of the 406 corpus boards is the new format, so it returned {} for all 185
       of them that carry zones (2651 zone stanzas, none matched). The caller
       then fell back to the single ``zone_clearance`` for every zone, so the
       fill model -- which decides what the pour already connects, and therefore
       every repair decision -- ran on the wrong inset wherever a zone's real
       clearance differed (measured on real boards: 0.2, 0.25, 0.508mm).
       Self-generated chains mostly got away with it because route_planes.py
       writes zones at the same clearance the repair falls back to; boards with
       PRE-EXISTING pours (the GUI's normal case) did not.
    2. It read ``input_file``. The GUI passes the live board's path, whose
       on-disk content is whatever was last saved -- for a plan run, the
       untouched input board.

    Both go away by reading the already-parsed board. A zone whose clearance is
    0/None is "not specified" in KiCad (use the net class), so it is omitted and
    the caller's fallback applies. Where several zones share a (net, layer), the
    LARGEST clearance wins -- the conservative fill estimate.
    """
    props: Dict[Tuple[str, str], Dict] = {}
    for z in (getattr(pcb_data, 'zones', None) or []):
        if not z.net_name or not z.layer:
            continue
        clr = getattr(z, 'clearance', None)
        if not clr or clr <= 0:
            continue  # unspecified -> caller falls back to zone_clearance
        mt = getattr(z, 'min_thickness', None) or 0.1
        key = (z.net_name, z.layer)
        prev = props.get(key)
        if prev is None or clr > prev['clearance']:
            props[key] = {'clearance': clr, 'min_thickness': mt}
    return props


def auto_detect_zones(
    input_file: str,
    filter_nets: Optional[List[str]] = None,
    filter_layers: Optional[List[str]] = None
) -> List[Tuple[str, str]]:
    """
    Auto-detect zone net/layer pairs from the PCB file.

    Args:
        input_file: Path to KiCad PCB file
        filter_nets: If provided, only include these nets
        filter_layers: If provided, only include these layers

    Returns:
        List of (net_name, layer) tuples for zones to process
    """
    zones = extract_zones(input_file)

    if not zones:
        return []

    # Build list of (net_name, layer) pairs
    zone_pairs: List[Tuple[str, str]] = []
    seen = set()

    for zone in zones:
        # Apply filters
        if filter_nets and zone.net_name not in filter_nets:
            continue
        if filter_layers and zone.layer not in filter_layers:
            continue

        key = (zone.net_name, zone.layer)
        if key not in seen:
            seen.add(key)
            zone_pairs.append(key)

    return zone_pairs




def repair_planes(
    input_file: str,
    output_file: str,
    net_names: List[str],
    plane_layers: List[str],
    track_width: float = defaults.TRACK_WIDTH,
    clearance: float = defaults.CLEARANCE,
    zone_clearance: float = defaults.PLANE_ZONE_CLEARANCE,
    grid_step: float = defaults.GRID_STEP,
    analysis_grid_step: float = defaults.REPAIR_ANALYSIS_GRID_STEP,
    ripup_blocker_select: str = defaults.RIPUP_BLOCKER_SELECT,
    max_track_width: float = defaults.REPAIR_MAX_TRACK_WIDTH,
    min_track_width: float = defaults.REPAIR_MIN_TRACK_WIDTH,
    track_via_clearance: float = defaults.PLANE_TRACK_VIA_CLEARANCE,
    hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE,
    board_edge_clearance: float = defaults.PLANE_EDGE_CLEARANCE,
    via_size: float = defaults.VIA_SIZE,
    via_drill: float = defaults.VIA_DRILL,
    max_iterations: int = defaults.MAX_ITERATIONS,
    verbose: bool = False,
    dry_run: bool = False,
    debug_lines: bool = False,
    routing_layers: Optional[List[str]] = None,
    pcb_data: Optional[PCBData] = None,
    return_results: bool = False,
    repair_pads: bool = True,
    max_search_radius: float = defaults.PLANE_MAX_SEARCH_RADIUS,
    rip_blocker_nets: bool = False,
    max_rip_nets: int = 3,
    reroute_ripped_nets: bool = False,
    power_nets: Optional[List[str]] = None,
    power_nets_widths: Optional[List[float]] = None,
    no_bga_zone: bool = False,
    progress_callback=None,
    cancel_check=None,
    net_clearances: Optional[dict] = None,
    layer_clearances: Optional[dict] = None,
    clamp_netclasses: bool = True,
    clearance_ceiling: Optional[float] = None,
    add_teardrops: bool = False,
    # #549 B: glob patterns naming nets whose committed-copper corridors via
    # placement should prefer to keep clear. None -> AUTO (the board's
    # protected/impedance records); ['none'] -> off.
) -> Tuple[int, int]:
    """
    Route between disconnected regions in power plane zones.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to process (e.g., ['GND', '+3.3V'])
        plane_layers: List of layers for each net (e.g., ['B.Cu', 'In1.Cu'])
        track_width: Default track width for routing config (mm)
        clearance: Clearance between traces (mm)
        zone_clearance: Zone fill clearance around obstacles (mm)
        grid_step: Routing grid step (mm)
        max_track_width: Maximum track width for region connections (mm)
        min_track_width: Minimum track width for region connections (mm)
        track_via_clearance: Clearance from tracks to other nets' vias (mm)
        hole_to_hole_clearance: Minimum clearance between drill holes (mm)
        board_edge_clearance: Clearance from board edge (mm)
        via_size: Via outer diameter for config (mm)
        via_drill: Via drill diameter for config (mm)
        max_iterations: Maximum A* iterations per route attempt
        verbose: Print detailed debug info
        dry_run: Analyze without writing output
        routing_layers: List of layers that can be used for routing (if None, auto-detect from PCB)
        repair_pads: If True (default), also repair pad-level plane connection
            failures (issue #99): pads of the plane net with no via/segment of
            the net within reach and not directly on a zone layer get a tap
            retry with route_planes' parameter escalation (default params,
            then scoped fine params for fine-pitch pads).
        max_search_radius: Max radius to search for a via position during pad
            repair (mm).
        progress_callback: Optional callable(current, total, label) invoked at
            phase milestones (pad repair k/N, region discovery, per-region
            connects, cleanup, ripped-net reconnect), mirroring batch_route's
            callback (issue #364). (0, 0, label) marks an indeterminate phase.
            Called from whatever thread runs the engine; GUI callers must
            marshal to the UI thread themselves.

    Returns:
        Tuple of (total_routes_added, total_regions_connected)
    """
    # zone_clearance=None means "follow the routed clearance": the GUI planes
    # tab's zone-clearance "auto" checkbox (ON by default) passes None, as does
    # any caller that leaves it unset. create_plane resolves this (via
    # _resolve_zone_clearance); the repair path did NOT, so None threaded down
    # into find_disconnected_zone_regions' layer_clearance and detonated
    # pad_rect_halfspan as `float + None` on the first foreign pad/segment/via
    # (issue #475). Resolve it here in the shared engine so BOTH fronts get it;
    # the CLI passes a real default (never None), so this is a no-op there.
    if zone_clearance is None:
        zone_clearance = clearance if clearance is not None else defaults.PLANE_ZONE_CLEARANCE
    from route import _dump_engine_config
    _dump_engine_config('repair_planes', dict(locals()))
    # Board-setup copper-to-edge rule (#338): engine-side so the GUI planes
    # tab and plan replays inherit it; see batch_route.
    if input_file:
        try:
            from fix_kicad_drc_settings import effective_board_edge_clearance
            _eff_edge = effective_board_edge_clearance(input_file, board_edge_clearance)
            if _eff_edge > (board_edge_clearance or 0.0):
                print(f"Board edge clearance {_eff_edge}mm "
                      f"(project min_copper_edge_clearance)")
                board_edge_clearance = _eff_edge
        except Exception:
            pass
    if pcb_data is None:
        print(f"Loading PCB from {input_file}...")
        pcb_data = parse_kicad_pcb(input_file)

    # #513 item 5: snapshot each net's dominant routed width BEFORE any rip
    # mutates pcb_data, so the end-of-run reconnect of ripped blockers can
    # re-route them at the width they arrived with (a rip-reconcile must not
    # silently drop a power net's width). See the reconnect pass below.
    from routing_common import dominant_net_widths as _dnw513
    _input_net_widths = _dnw513(pcb_data.segments)

    # Canonicalise the starting copper ORDER, before anything reads it.
    # This is the plane-repair twin of the same call in route.batch_route: the
    # GUI adds tracks to a live pcbnew board and the CLI writer emits them from
    # its own lists, so the two fronts hand this engine identical copper in
    # different ORDER, and list position leaks into region/anchor selection.
    # Adding it to batch_route alone took eth_tap steps 1-13 to an exact match
    # and left ONLY repair_planes diverging (7548 vs 7557 segments), because
    # this engine never got the same treatment.
    from kicad_parser import canonicalize_pcb_data_order
    canonicalize_pcb_data_order(pcb_data)

    # #493 item 3: snapshot the board's ORIGINAL copper IN MEMORY, now, before
    # anything mutates pcb_data. The rip-blocker casualty restore below needs
    # "what this net looked like before we touched it", and used to re-read
    # `input_file` for it. That is wrong for the GUI: it passes the live board's
    # path, whose ON-DISK content is whatever was last saved -- for a plan run,
    # the untouched input board. The restore therefore found no original copper,
    # skipped the restore, and KEPT the new repair copper the CLI drops, so the
    # two fronts shipped different boards from byte-identical inputs and kwargs
    # (nano_eeprom_prog: CLI +0 segments, GUI +30, both grading clean).
    _orig_segments = list(pcb_data.segments or [])
    _orig_vias = list(pcb_data.vias or [])

    # Route trace (#482): plane repair adds join tracks/vias and rips blockers
    # OUTSIDE the copper choke points, so record it by snapshot-diffing pcb_data
    # at each phase. Local (not attached) so the internal reconnect batch_route
    # calls don't collide. baseline captured now; deltas captured below.
    from route_trace import start_plane_trace as _start_plane_trace
    _ptrace = _start_plane_trace(pcb_data, output_file)

    # Resolve net IDs
    net_ids = []
    for net_name in net_names:
        net_id = None
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        if net_id is None:
            print(f"Error: Net '{net_name}' not found in PCB")
            return (0, 0)
        net_ids.append(net_id)

    # Get board bounds
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        print("Error: Could not determine board bounds")
        return (0, 0)

    min_x, min_y, max_x, max_y = board_bounds
    print(f"Board bounds: ({min_x:.2f}, {min_y:.2f}) to ({max_x:.2f}, {max_y:.2f})")

    # Zone bounds with edge clearance
    zone_bounds = (
        min_x + board_edge_clearance,
        min_y + board_edge_clearance,
        max_x - board_edge_clearance,
        max_y - board_edge_clearance
    )

    # Build routing config
    config = GridRouteConfig(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        board_edge_clearance=board_edge_clearance,
        ripup_blocker_select=ripup_blocker_select
    )
    # #498: repair copper (region joins, pad taps, reconnects) must obey the
    # board's per-layer .kicad_dru clearance rules like every routed copper.
    # An explicit `layer_clearances` wins and stops the auto-read: route.py's
    # in-run plane finalize (#562) MUST pass its own resolved map, because it
    # calls this engine on the file it is still writing -- that output's
    # .kicad_dru sibling does not exist until fix_project_for_output copies
    # it after batch_route returns, so an auto-read here would find NOTHING
    # and tap/join copper would route blind to the board's layer rules. Same
    # reasoning as the reconciliation sub-run's forwarded map.
    from kicad_dru import install_layer_clearances
    install_layer_clearances(config, layer_clearances, input_file, pcb_data)

    # Cross-class clearance (#434): the repair step's own copper (region joins,
    # pad taps) and its ripped-blocker reconnects were priced at the uniform
    # clearance only, so repair copper landed inside fat-class bands (cparti
    # BTN4 reconnect 0.20-0.31mm from SMA-class copper whose class demands
    # 0.35). Mirror batch_route's contract: auto-read the board's non-Default
    # netclasses from the INPUT's sibling .kicad_pro when no map was passed
    # (id-keyed; all-Default boards -> empty map -> inert), stamp them on the
    # repair config, and forward the map to the reconnect sub-runs below --
    # those route from the OUTPUT file, whose sibling .kicad_pro may not exist
    # yet (same hazard as the #338 edge resolution).
    if net_clearances is None and input_file and os.path.isfile(input_file):
        try:
            from list_nets import net_clearance_map_by_id
            net_clearances = net_clearance_map_by_id(
                input_file, {nid: n.name for nid, n in pcb_data.nets.items()})
            if net_clearances:
                print(f"Auto-read netclass clearances for "
                      f"{len(net_clearances)} net(s) (cross-class max(A,B) "
                      f"respected during plane repair).")
        except Exception as _e:
            print(f"Warning: could not auto-read netclass clearances ({_e}); "
                  f"repairing at the uniform clearance.")
            net_clearances = None
    # #439: --clearance was the CEILING -> cap every class at min(class, ceiling).
    # When not clamping (--clearance omitted), honor the classes in full. The
    # capped map propagates to the reconnect sub-runs below (they reuse it).
    if net_clearances and clamp_netclasses and clearance_ceiling is not None:
        net_clearances = {nid: min(c, clearance_ceiling)
                          for nid, c in net_clearances.items()}
    if net_clearances:
        config.net_clearances = dict(net_clearances)
    # Publish the SAME map to the fill model (#483 item 5): KiCad refills a
    # zone at max(zone clearance, pairwise netclass), so on honor-classes
    # chains a looser foreign class carves copper the model would otherwise
    # predict as fill -- and every repair decision here (region discovery,
    # zone credit, dead-end anchors, the gate) reads that prediction. Must
    # precede every ZoneFillModel build on this board: models are cached.
    from plane_fill_model import set_board_net_clearances
    set_board_net_clearances(pcb_data, net_clearances)

    # Auto-detect routing layers if not specified
    if routing_layers is None:
        routing_layers = pcb_data.board_info.copper_layers
        if not routing_layers:
            routing_layers = ['F.Cu', 'B.Cu']  # Fallback
    # NOTE: unlike batch_route, routing_layers here directly selects layers
    # region joins may PLACE copper on (not cost-driven), so no full-stack
    # append -- the default above is already the whole board, and the
    # off-layer via guard in build_base_obstacle_map/obstacle_cache covers
    # the explicit-subset case for via placement.
    print(f"Routing layers: {', '.join(routing_layers)}")

    # Issue #293 guard: snapshot every multi-pad SIGNAL net's connectivity so a
    # regression this repair pass causes (rip cascades, tap side effects) is
    # caught and reported loudly at the end instead of shipping silently. The
    # per-net union-find over the whole board costs well under a second.
    from check_connected import check_net_connectivity as _cnc293
    _zones_by_net_293: Dict[int, list] = {}
    for _z in (getattr(pcb_data, 'zones', None) or []):
        _zones_by_net_293.setdefault(_z.net_id, []).append(_z)
    _segs_293: Dict[int, list] = {}
    for _s in pcb_data.segments:
        _segs_293.setdefault(_s.net_id, []).append(_s)
    _vias_293: Dict[int, list] = {}
    for _v in pcb_data.vias:
        _vias_293.setdefault(_v.net_id, []).append(_v)
    _plane_net_ids = set(net_ids)
    _pre_connected_293 = set()
    for _nid, _pads in pcb_data.pads_by_net.items():
        if _nid in _plane_net_ids or len(_pads) < 2:
            continue
        if not (_segs_293.get(_nid) or _vias_293.get(_nid) or _zones_by_net_293.get(_nid)):
            continue  # unrouted before us; not ours to regress
        _r = _cnc293(_nid, _segs_293.get(_nid, []), _vias_293.get(_nid, []),
                     _pads, _zones_by_net_293.get(_nid, []))
        if _r.get('connected'):
            _pre_connected_293.add(_nid)

    all_new_segments: List[Dict] = []
    all_new_vias: List[Dict] = []

    # #517 arm 2: per-run corridor-ghost registry (None unless the
    # KICAD_PLANE_RIP_SOFTBLOCK env knob is set; every consumer no-ops on
    # None, so the default path is untouched).
    _sb_mode = softblock_mode()
    corridor_ghosts = CorridorGhosts(_sb_mode) if _sb_mode else None
    if corridor_ghosts is not None:
        print(f"  (#517 corridor ghosts armed: {_sb_mode} mode)")

    # #549 B: SEPARATE seed registry from COMMITTED copper of path-critical
    # nets (auto: the board's protected/impedance records). Soft via-site
    # preference only; independent of KICAD_PLANE_RIP_SOFTBLOCK.

    # #517 instrumentation: which PASS placed each piece of this run's new
    # copper (pad-tap, region-join, partial-restore, reconnect, custody
    # restore), so a custody REFUSED-restore can name the occupier class
    # instead of just "copper routed meanwhile". Geometry-keyed (the same
    # copper exists as pcb_data objects and write-list dicts; rounding
    # matches consume_inner_strips). Anything unmatched is copper that
    # predates this run -- a refusal against THAT means the collision test
    # was conservative, itself a finding.
    _copper_provenance: Dict[tuple, str] = {}

    def _prov_seg(net_id, layer, sx, sy, ex, ey, tag):
        a = (round(sx, 3), round(sy, 3))
        b = (round(ex, 3), round(ey, 3))
        _copper_provenance[(net_id, layer, a, b)] = tag
        _copper_provenance[(net_id, layer, b, a)] = tag

    def _prov_via(net_id, x, y, tag):
        _copper_provenance[(net_id, round(x, 3), round(y, 3))] = tag

    def _prov_lookup(kind, obj):
        if kind == 'segment':
            return _copper_provenance.get(
                (obj.net_id, obj.layer,
                 (round(obj.start_x, 3), round(obj.start_y, 3)),
                 (round(obj.end_x, 3), round(obj.end_y, 3))),
                'pre-existing')
        return _copper_provenance.get(
            (obj.net_id, round(obj.x, 3), round(obj.y, 3)), 'pre-existing')

    file_strip_segments: List = []
    file_strip_vias: List = []

    def _consume_inner_strips(_rd, _label):
        # Shared with route_planes' GUI reconnect (#508 finding 1); see
        # plane_write_reconcile.consume_inner_strips for the full story.
        consume_inner_strips(_rd, all_new_segments, all_new_vias, pcb_data,
                             file_strip_segments, file_strip_vias, _label)

    all_debug_lines: List[str] = []
    total_routes = 0
    total_regions = 0
    total_vias = 0
    total_pads_unconnected = 0
    # Pads the repair taps are fill-unreachable BY DIAGNOSIS -- their tap
    # copper must never be removed by the graze/dead-end cleanup (whose
    # connectivity gate credits the pour OUTLINE and would grade the taps
    # redundant: Andy's bitaxe Q2 shredded-stub opens).
    _tapped_pads = []
    total_pads_repaired = 0
    failed_repair_pads: List[str] = []

    # Extract per-zone clearances and min_thickness from PCB file
    zone_props = extract_zone_properties(pcb_data)
    if verbose:
        print(f"Zone properties:")
        for (net, layer), props in zone_props.items():
            print(f"  {net} on {layer}: clearance={props['clearance']}mm, min_thickness={props['min_thickness']}mm")

    print(f"\n{'='*60}")
    print(f"Routing disconnected plane regions")
    print(f"{'='*60}")

    # Group zones by net - process each net once with all its zone layers
    unique_nets: Dict[int, Tuple[str, Set[str]]] = {}  # net_id -> (net_name, set of layers)
    for net_name, plane_layer, net_id in zip(net_names, plane_layers, net_ids):
        if net_id not in unique_nets:
            unique_nets[net_id] = (net_name, set())
        unique_nets[net_id][1].add(plane_layer)

    # Run-6 A5, warn-only here (MOVED: this ran BEFORE unique_nets existed,
    # so it raised NameError into the bare `except` below on every call and
    # the warning never once fired) (the repair legitimately runs mid-loop on
    # boards with known opens): every tap via this step adds shrinks a bare
    # pad's escape channel further, and tap fields are not rippable copper.
    try:
        from check_connected import bare_pad_nets
        _bare = bare_pad_nets(pcb_data, exclude_net_ids=set(unique_nets))
        if _bare:
            _bn = sorted(pcb_data.nets[i].name for i in _bare
                         if i in pcb_data.nets)
            print(f"  WARNING: {len(_bare)} net(s) still have >=2 pads and "
                  f"ZERO copper ({', '.join(_bn[:6])}"
                  f"{', ...' if len(_bn) > 6 else ''}) -- this step's tap "
                  f"vias will crowd their escape channels; route them first "
                  f"if their pads must connect (the pour is a one-way door "
                  f"for bare pads).")
    except Exception:
        pass


    # --reroute-ripped-nets is deprecated (issue #141 reverted). This step used to
    # rip signal blockers, route plane/pad repairs into the freed space, then
    # re-route the ripped nets -- but a ripped net that FAILED to re-route had its
    # ORIGINAL copper restored on top of whatever had meanwhile been routed through
    # its corridor, creating P-to-N shorts the obstacle map never saw (a restore
    # bypasses it). Rerouting is now left to a route.py pass run AFTER this step,
    # which handles rip-up/restore safely. We still rip blockers for pad repair and
    # leave them UNROUTED (stripped) for that route.py pass to reconnect.
    if reroute_ripped_nets:
        print("Note: --reroute-ripped-nets is deprecated and now a no-op. Ripped "
              "blocker nets are now reconnected in-run (restore-first, an "
              "end-of-run reconnect pass, and custody restore on failure); no "
              "separate route.py pass is needed.")
        reroute_ripped_nets = False

    # Plane nets are never ripped to clear a blocker (--rip-blocker-nets); only
    # signal nets are, and they are left unrouted for a subsequent route.py pass.
    plane_net_ids = set(unique_nets.keys())
    # #521: nets protected in the sibling .kicad_pro (length-matched groups,
    # routed diff pairs) and nets with KiCad-LOCKED copper join the never-rip
    # set -- a blocker rip here strips the net for a later generic route.py
    # reconnect, which cannot reproduce matching/coupling/hand-routing. (The
    # tap simply fails over its other candidates.)
    try:
        from protected_nets import protection_map
        _prot_names = protection_map(pcb_data)
        if _prot_names:
            _prot_ids = {nid for nid, n in pcb_data.nets.items()
                         if n.name in _prot_names}
            _prot_ids -= plane_net_ids
            if _prot_ids and rip_blocker_nets:
                _ex = sorted(pcb_data.nets[i].name for i in _prot_ids)[:4]
                print(f"  {len(_prot_ids)} PROTECTED net(s) excluded from blocker "
                      f"rip-up ({', '.join(_ex)}{'...' if len(_prot_ids) > 4 else ''})")
            plane_net_ids |= _prot_ids
    except Exception:
        pass
    ripped_net_ids: List[int] = []
    # #517 arm 3 (#524 root cause): nets whose immediate reconnect SUCCEEDED
    # leave ripped_net_ids (they are no longer casualties) -- but their
    # ORIGINAL input copper was ripped and replaced by different copper, and
    # ripped_net_ids doubles as the writer's input-copper exclusion list.
    # Without this list the writer RESURRECTED the old copper at its old
    # coordinates alongside the new (file-only ghosts over corridors other
    # passes had since claimed: spartan6 885 DRC / 636 kicad-DRC, all
    # file-vs-board). Members are excluded at write exactly like ripped nets;
    # their live copper ships from the write list.
    inplace_reconnected_ids: List[int] = []
    # (net_id, kept_segs, kept_vias, dropped_count) for nets partially
    # restored by the success-path settle: input copper stripped at write,
    # kept pieces emitted as new copper (board==file), gap left for reconnect.
    partial_restores: List = []
    # Trace-to-existing-plane-copper reaches the full via-search radius
    # (max_search_radius, the --max-search-radius CLI value) so a boxed pad whose
    # nearest existing same-net via sits past a smaller cap is still reachable
    # (issue #180: castor_pollux U5.4's nearest GND via was at 4.62mm).
    # Without --rip-blocker-nets the trace step still runs at STRAP scale
    # (issue #349): an unconnected pad adjacent to an already-repaired same-net
    # pad straps to it instead of drilling another via, so a fine-pitch pad
    # cluster shares one via + short straps.
    distant_radius = (max_search_radius if rip_blocker_nets
                      else min(max_search_radius, defaults.PLANE_PAD_STRAP_RADIUS))

    # Per-net context captured at the round-1 region-join call, so the
    # post-reconnect pass below can re-run the join for a fill the reconnect
    # re-pinched (hex_gateway C201/C205 -- see the reconnect block).
    _round2_ctx: Dict[int, dict] = {}

    if _PLANE_REPAIR_ORDERING and repair_pads and unique_nets:
        # #517 arm 3 (#480): most-constrained-first NET order -- the net with
        # the most disconnected pads repairs first, while contested space is
        # emptiest. Deterministic: count desc, then net name.
        _ord_counts: Dict[int, int] = {}
        for _onid, (_onm, _olyrs) in unique_nets.items():
            try:
                _ord_counts[_onid] = len(find_unconnected_plane_pads(
                    pcb_data, _onid, _olyrs))
            except Exception:
                _ord_counts[_onid] = 0
        unique_nets = dict(sorted(
            unique_nets.items(),
            key=lambda kv: (-_ord_counts.get(kv[0], 0), kv[1][0])))
        print("  (#517 ordering: net repair order = "
              + ", ".join(f"{unique_nets[_n][0]}[{_ord_counts.get(_n, 0)}]"
                          for _n in unique_nets) + ")")

    def _reconnect_ripped_now(_rip_ids):
        """#517 arm 3: reconnect the nets ripped for the tap that JUST
        committed, before any other pad, region join, or reconnect can claim
        their vacated corridors (the window-shrink half of the #480 knob).
        Successes leave ripped_net_ids -- their new copper enters the write
        list here; failures stay queued for the end-of-run reconnect and its
        custody. Mirrors the end-of-run batch: same batch_route call, same
        #513 width preservation, and same #338/#441 edge floor."""
        _names = [pcb_data.nets[_n].name for _n in _rip_ids
                  if _n in pcb_data.nets]
        if not _names:
            return
        print(f"    (#517 ordering: immediate reconnect of {len(_names)} "
              f"ripped net(s): {', '.join(_names)})")
        try:
            from route import batch_route
            try:
                from fix_kicad_drc_settings import effective_board_edge_clearance
                _edge = effective_board_edge_clearance(input_file, 0.0)
            except Exception:
                _edge = 0.0
            _pn = list(power_nets or [])
            _pw = list(power_nets_widths or [])
            if len(_pn) == len(_pw):
                from net_queries import matches_net_filter as _mnf517
                for _cnid in _rip_ids:
                    _cw = _input_net_widths.get(_cnid, 0.0)
                    _cn = (pcb_data.nets[_cnid].name
                           if _cnid in pcb_data.nets else None)
                    if not _cn or _cw <= (track_width or 0.0) + 1e-6:
                        continue
                    if _pn and _mnf517(_cn, _pn):
                        continue
                    _pn.append(_cn)
                    _pw.append(_cw)
            _ok, _fail, _t, _rdata = batch_route(
                input_file, "", _names,
                layers=routing_layers,
                track_width=track_width, clearance=clearance,
                via_size=via_size, via_drill=via_drill,
                grid_step=grid_step, max_iterations=max_iterations,
                power_nets=_pn or None, power_nets_widths=_pw or None,
                board_edge_clearance=_edge,
                disable_bga_zones=([] if no_bga_zone else None),
                net_clearances=net_clearances,
                hole_to_hole_clearance=hole_to_hole_clearance,
                return_results=True, pcb_data=pcb_data,
                # #540 item 2: price the OTHER pending casualties' corridors
                # (this batch's own nets excluded -- theirs to reclaim).
                # #562 RE-ENTRY GUARD: batch_route's plane finalize calls THIS
                # engine, so a sub-run that runs its own finalize recurses:
                # finalize -> repair -> batch_route -> finalize -> ... (measured
                # 57 levels on schoko, ~7 GB before the cap killed it). This
                # sub-run is a repair DETAIL, not a chain step -- never finalize.
                final_reconcile=False,
                **_ghost_kwargs(corridor_ghosts, _rip_ids))

            for _r in _rdata.get('results', []):
                for _s in (_r.get('new_segments') or []):
                    all_new_segments.append(
                        {'start': (_s.start_x, _s.start_y),
                         'end': (_s.end_x, _s.end_y),
                         'width': _s.width, 'layer': _s.layer,
                         'net_id': _s.net_id})
                    _prov_seg(_s.net_id, _s.layer, _s.start_x, _s.start_y,
                              _s.end_x, _s.end_y, 'reconnect')
                for _v in (_r.get('new_vias') or []):
                    all_new_vias.append(
                        {'x': _v.x, 'y': _v.y, 'size': _v.size,
                         'drill': _v.drill, 'layers': _v.layers,
                         'net_id': _v.net_id})
                    _prov_via(_v.net_id, _v.x, _v.y, 'reconnect')
            for _v in (_rdata.get('all_swap_vias') or []):
                all_new_vias.append(
                    {'x': _v.x, 'y': _v.y, 'size': _v.size,
                     'drill': _v.drill, 'layers': _v.layers,
                     'net_id': _v.net_id})
                _prov_via(_v.net_id, _v.x, _v.y, 'reconnect')
            for _s in (_rdata.get('all_swap_segments') or []):
                all_new_segments.append(
                    {'start': (_s.start_x, _s.start_y),
                     'end': (_s.end_x, _s.end_y),
                     'width': _s.width, 'layer': _s.layer,
                     'net_id': _s.net_id})
                _prov_seg(_s.net_id, _s.layer, _s.start_x, _s.start_y,
                          _s.end_x, _s.end_y, 'reconnect')
            # A net is done only if it is CONNECTED now (batch_route's own
            # success flag is not the arbiter -- #479's lesson).
            from check_connected import check_net_connectivity as _cnc517
            for _nid in list(_rip_ids):
                if _nid not in pcb_data.nets:
                    continue
                _r517 = _cnc517(
                    _nid,
                    [s for s in pcb_data.segments if s.net_id == _nid],
                    [v for v in pcb_data.vias if v.net_id == _nid],
                    pcb_data.pads_by_net.get(_nid, []),
                    [z for z in (getattr(pcb_data, 'zones', None) or [])
                     if z.net_id == _nid],
                    pcb_data=pcb_data)
                if _r517.get('connected'):
                    while _nid in ripped_net_ids:
                        ripped_net_ids.remove(_nid)
                    if _nid not in inplace_reconnected_ids:
                        # The writer must still strip this net's INPUT copper
                        # (ripped and replaced); see inplace_reconnected_ids.
                        inplace_reconnected_ids.append(_nid)
                    if corridor_ghosts is not None:
                        corridor_ghosts.drop_net(_nid)
                    print(f"    (#517 ordering: {pcb_data.nets[_nid].name} "
                          f"reconnected in place)")
            # The reconnect mutated copper; drop the cached plane fill models
            # (mirrors the end-of-run reconnect).
            try:
                from plane_fill_model import _CACHE_ATTR as _PFM_CACHE_O
                if hasattr(pcb_data, _PFM_CACHE_O):
                    delattr(pcb_data, _PFM_CACHE_O)
            except Exception:
                pass
        except Exception as _e:
            print(f"{RED}    immediate reconnect failed: {_e}{RESET}")

    for net_id, (net_name, net_zone_layers) in unique_nets.items():
        if cancel_check and cancel_check():
            print("\nPlane repair cancelled")
            break
        # Build per-layer zone clearances for all layers with zones for this net
        # These are used in flood fill to determine what the zone fill connects
        zone_clearances: Dict[str, float] = {}
        for layer in net_zone_layers:
            zk = (net_name, layer)
            if zk in zone_props:
                zone_clearances[layer] = zone_props[zk]['clearance']

        # Use maximum clearance as fallback (per-layer clearances used in flood fill)
        max_zone_clearance = max(zone_clearances.values()) if zone_clearances else zone_clearance

        # Pick first zone layer as "primary" (for plane_layer_idx in routing)
        primary_layer = sorted(net_zone_layers)[0]

        layers_str = ", ".join(sorted(net_zone_layers))
        clearances_str = ", ".join(f"{l}={zone_clearances.get(l, zone_clearance)}mm" for l in sorted(net_zone_layers))
        print(f"\n[{net_name}] on {layers_str} (clearances: {clearances_str}):")

        # Per-pad repair pass (issue #99): reconnect pads route_planes could
        # not via down to the plane. Runs before island repair so the new
        # vias participate in the region connectivity analysis.
        if repair_pads:
            if progress_callback:
                progress_callback(0, 0, f"{net_name}: checking pad-plane connections...")
            unconnected = find_unconnected_plane_pads(pcb_data, net_id, net_zone_layers)
            total_pads_unconnected += len(unconnected)
            if not unconnected:
                print(f"  Pad repair: all {net_name} pads reach the plane")
            else:
                print(f"  Pad repair: {len(unconnected)} pad(s) with no connection to the {net_name} plane:")
                tap_config = replace(
                    config,
                    layers=routing_layers,
                    hole_to_hole_clearance=hole_to_hole_clearance
                )
                # Cross-pad via-obstacle-map reuse for this net's repair pass (#263).
                shared_maps = SharedViaMaps(pcb_data, net_id)
                # T6 mutual-floating-strap guard: components of this net's
                # copper, built ONCE (same union-find as check_connected) and
                # updated incrementally as taps commit. Tap targets not in the
                # MAIN plane component are rejected, so two pads can no longer
                # strap to each other's floating island and both report success.
                plane_oracle = PlaneComponentOracle(pcb_data, net_id)
                if plane_oracle.n_floating_items:
                    print(f"    (plane oracle: {plane_oracle.n_floating_items} "
                          f"floating same-net item(s) excluded as tap targets)")
                # #517 arm 3: under the ordering knob, pads run in two phases:
                # phase 1 tries every pad WITHOUT rips (free-space claims
                # first, none of them contested); pads that would need a rip
                # are deferred to phase 2, where each rip's nets reconnect
                # IMMEDIATELY after the tap commits, so at most one vacated
                # corridor is ever open. Default: single phase, rips inline
                # (allow_rip from the start), exactly the old loop.
                _pad_queue = [(p, l, not _PLANE_REPAIR_ORDERING)
                              for (p, l) in unconnected]
                _deferred_rip: List = []
                _phase2_immediate = False
                _phase_rip_ids: List[int] = []  # #540: phase rip history,
                #   kept even after a successful immediate reconnect
                _pr_idx = -1
                while _pad_queue or _deferred_rip:
                    if not _pad_queue:
                        _phase2_immediate = _PLANE_IMMEDIATE_RECONNECT or (
                            _PLANE_ADAPTIVE_RECONNECT
                            and len(_deferred_rip)
                            <= _PLANE_IMMEDIATE_MAX_PENDING)
                        _mode_note = ""
                        if _PLANE_ADAPTIVE_RECONNECT:
                            _mode_note = (" [adaptive: immediate]"
                                          if _phase2_immediate else
                                          " [adaptive: deferred to end batch]")
                        print(f"    (#517 ordering: phase 2 -- "
                              f"{len(_deferred_rip)} rip-requiring pad(s)"
                              f"{_mode_note})")
                        _pad_queue = [(p, l, True) for (p, l) in _deferred_rip]
                        _deferred_rip = []
                    pad, pad_layer, _allow_rip = _pad_queue.pop(0)
                    _pr_idx += 1
                    if cancel_check and cancel_check():
                        print("    (cancelled)")
                        break
                    _tapped_pads.append(pad)
                    if progress_callback:
                        progress_callback(_pr_idx + 1, len(unconnected),
                                          f"{net_name}: pad repair "
                                          f"{pad.component_ref}.{pad.pad_number}")
                    print(f"    Pad {pad.component_ref}.{pad.pad_number} ({pad_layer})...", end=" ", flush=True)
                    result = tap_pad_with_escalation(
                        pad, pad_layer, net_id, pcb_data, tap_config,
                        max_search_radius=max_search_radius,
                        via_size=via_size,
                        via_drill=via_drill,
                        verbose=verbose,
                        fine_for_all=True,  # last-resort repair: escalate every failed pad
                        distant_trace_radius=distant_radius,
                        shared_via_maps=shared_maps,
                        plane_oracle=plane_oracle,
                        corridor_ghosts=corridor_ghosts)
                    _rips_before = len(ripped_net_ids)
                    if not result.success and rip_blocker_nets:
                        if not _allow_rip:
                            # #517 arm 3 phase 1: this pad needs a rip --
                            # defer it so the free-space pads claim first and
                            # the rips run one-at-a-time with immediate
                            # reconnects in phase 2.
                            _deferred_rip.append((pad, pad_layer))
                            print("(needs rip - deferred to phase 2)")
                            continue
                        # Rip the signal net(s) blocking this pad's trace and retry
                        # (the ripped nets are re-routed after the repair pass).
                        rr = _tap_pad_with_ripup(
                            pad, pad_layer, net_id, pcb_data, tap_config, config,
                            max_search_radius, via_size, via_drill, max_rip_nets,
                            plane_net_ids, result, ripped_net_ids, verbose,
                            distant_trace_radius=distant_radius,
                            shared_via_maps=shared_maps,
                            partial_restores=(partial_restores
                                              if _PLANE_PARTIAL_RESTORE
                                              else None),
                            plane_oracle=plane_oracle,
                            corridor_ghosts=corridor_ghosts,
                            write_lists=(all_new_segments, all_new_vias))
                        if rr is not None:
                            result = rr
                    if result.success:
                        total_pads_repaired += 1
                        params_note = " [fine params]" if result.params_label == 'fine' else ""
                        new_via_objs = []
                        new_seg_objs = []
                        if result.via is not None:
                            all_new_vias.append(result.via)
                            total_vias += 1
                            new_via_objs.append(Via(
                                x=result.via['x'], y=result.via['y'],
                                size=result.via['size'], drill=result.via['drill'],
                                layers=['F.Cu', 'B.Cu'], net_id=net_id
                            ))
                            pcb_data.vias.append(new_via_objs[0])
                            where = f"placed via at ({result.via['x']:.2f}, {result.via['y']:.2f})"
                        else:
                            where = (f"reused via at ({result.reused_via_pos[0]:.2f}, "
                                     f"{result.reused_via_pos[1]:.2f})")
                        for s in result.segments:
                            all_new_segments.append(s)
                            seg_obj = Segment(
                                start_x=s['start'][0], start_y=s['start'][1],
                                end_x=s['end'][0], end_y=s['end'][1],
                                width=s['width'], layer=s['layer'], net_id=s['net_id']
                            )
                            new_seg_objs.append(seg_obj)
                            pcb_data.segments.append(seg_obj)
                        shared_maps.note_pass_copper(new_via_objs, new_seg_objs)
                        for _po in new_seg_objs:
                            _prov_seg(_po.net_id, _po.layer, _po.start_x,
                                      _po.start_y, _po.end_x, _po.end_y,
                                      'pad-tap')
                        for _po in new_via_objs:
                            _prov_via(_po.net_id, _po.x, _po.y, 'pad-tap')
                        # T6: this tap was oracle-verified to reach the main
                        # plane; credit its pad + copper so later pads may
                        # strap to them (transitive, no graph rebuild).
                        plane_oracle.note_tap_committed(pad, new_via_objs,
                                                        new_seg_objs)
                        from route_trace import plane_capture as _plane_capture
                        _plane_capture(pcb_data, 'plane-join', net_id, net_name)  # individual pad-tap frame
                        print(f"{GREEN}{where}, {len(result.segments)} trace segment(s){params_note}{RESET}")
                    else:
                        failed_repair_pads.append(f"{pad.component_ref}.{pad.pad_number} ({net_name})")
                        print(f"{RED}FAILED{RESET}")
                    if (_phase2_immediate and _allow_rip
                            and len(ripped_net_ids) > _rips_before
                            and (return_results or not dry_run)):
                        # #517 arm 3: this pad's rips (tap committed OR failed
                        # with unrestorable pieces) reconnect NOW, while their
                        # corridors are still exactly as the rip left them.
                        # #540: EXCEPT nets bus-coupled to a PENDING casualty
                        # (any phase's deferred batch) or to ANY net this
                        # phase already ripped -- an early sibling's
                        # reconnect locks the shared corridor against the
                        # rest, so coupled nets wait and negotiate it
                        # together in the end-of-run batch.
                        _newly = list(ripped_net_ids[_rips_before:])
                        _phase_rip_ids.extend(_newly)
                        _coupled = _corridor_coupled_ids(
                            _newly, set(ripped_net_ids) | set(_phase_rip_ids),
                            pcb_data)
                        if _coupled:
                            _cnames = sorted(
                                pcb_data.nets[_n].name for _n in _coupled
                                if _n in pcb_data.nets)
                            print(f"    (#540 coupling: deferred "
                                  f"{len(_coupled)} bus-coupled net(s) to end "
                                  f"batch: {', '.join(_cnames)})")
                        _uncoupled = [_n for _n in _newly
                                      if _n not in _coupled]
                        if _uncoupled:
                            _reconnect_ripped_now(_uncoupled)

        # Build obstacle map for this net
        if progress_callback:
            progress_callback(0, 0, f"{net_name}: building obstacle map...")
        print(f"  Building obstacle map...", end=" ", flush=True)
        base_obstacles, layer_map = build_base_obstacles(
            exclude_net_ids={net_id},
            routing_layers=routing_layers,
            pcb_data=pcb_data,
            config=config,
            # Base keep-outs use the min connection width; the widen step then
            # passes the extra half-width to the router, which now does an exact
            # swept-capsule clearance check (issues #156/#173), so a wide (e.g.
            # 0.4mm) connection's diagonal no longer grazes foreign copper.
            track_width=min_track_width,
            track_via_clearance=track_via_clearance,
            hole_to_hole_clearance=hole_to_hole_clearance
        )
        print("done")
        if corridor_ghosts is not None:
            # #517 arm 2: region-join straps were the second-largest occupier
            # of refused-restore corridors (140 items in the arm-1 histogram);
            # steer them away from every corridor vacated so far.
            corridor_ghosts.merge_into_routing_map(base_obstacles, config,
                                                   layer_map)

        _round2_ctx[net_id] = {
            'net_name': net_name, 'primary_layer': primary_layer,
            'zone_bounds': zone_bounds, 'net_zone_layers': net_zone_layers,
            'zone_clearances': zone_clearances,
            'max_zone_clearance': max_zone_clearance,
        }
        region_segments, region_vias, routes_added, route_paths, _ = route_disconnected_regions(
            net_id=net_id,
            net_name=net_name,
            plane_layer=primary_layer,
            zone_bounds=zone_bounds,
            pcb_data=pcb_data,
            config=config,
            base_obstacles=base_obstacles,
            layer_map=layer_map,
            zone_clearance=max_zone_clearance,
            max_track_width=max_track_width,
            min_track_width=min_track_width,
            track_via_clearance=track_via_clearance,
            hole_to_hole_clearance=hole_to_hole_clearance,
            analysis_grid_step=analysis_grid_step,
            max_iterations=max_iterations,
            verbose=verbose,
            zone_layers=net_zone_layers,
            zone_clearances=zone_clearances,
            progress_callback=progress_callback,
            cancel_check=cancel_check
        )

        if routes_added > 0:
            all_new_segments.extend(region_segments)
            all_new_vias.extend(region_vias)
            total_routes += routes_added
            total_regions += routes_added + 1  # N routes connect N+1 regions
            total_vias += len(region_vias)

            # Generate debug lines for this net's routes (on User.4)
            if debug_lines and route_paths:
                for route_path in route_paths:
                    for i in range(len(route_path) - 1):
                        p1, p2 = route_path[i], route_path[i + 1]
                        all_debug_lines.append(generate_gr_line_sexpr(p1, p2, 0.1, "User.4"))

            # Add segments to pcb_data so subsequent nets see them as obstacles
            for s in region_segments:
                start = s['start']
                end = s['end']
                pcb_data.segments.append(Segment(
                    start_x=start[0], start_y=start[1],
                    end_x=end[0], end_y=end[1],
                    width=s['width'], layer=s['layer'], net_id=s['net_id']
                ))
                _prov_seg(s['net_id'], s['layer'], start[0], start[1],
                          end[0], end[1], 'region-join')

            # Add vias to pcb_data so subsequent nets see them as obstacles
            for v in region_vias:
                pcb_data.vias.append(Via(
                    x=v['x'], y=v['y'],
                    size=v['size'], drill=v['drill'],
                    layers=['F.Cu', 'B.Cu'],  # Through-hole vias
                    net_id=v['net_id']
                ))
                _prov_via(v['net_id'], v['x'], v['y'], 'region-join')
            from route_trace import plane_capture as _plane_capture
            _plane_capture(pcb_data, 'plane-join', net_id, net_name)  # individual region-join frame

    # Partial restores: emit kept pieces as new copper and strip the nets'
    # input copper (replacement semantics -- same as route_planes b2557cd).
    # A net can be partially restored more than once (re-ripped by a later
    # pad); only its LATEST kept-set is live in pcb_data -- emitting earlier
    # sets would duplicate copper (the route_planes stale-emission bug).
    _latest: Dict[int, tuple] = {}
    for _entry in partial_restores:
        _latest[_entry[0]] = _entry
    # A net re-ripped later and left FULLY ripped must not emit its stale
    # earlier kept-set: full rip wins (it is in ripped_net_ids, zero copper).
    for _rid in ripped_net_ids:
        _latest.pop(_rid, None)
    partial_ids: List[int] = []
    # Keep a handle on exactly the dicts emitted here: the reconnect below can
    # re-route a partially-restored net and DELETE this copper from pcb_data,
    # after which the write list must not still carry it (see the
    # reconciliation after the reconnect).
    _partial_emitted_segs: List[Dict] = []
    _partial_emitted_vias: List[Dict] = []
    for _pid, _ksegs, _kvias, _dropped in _latest.values():
        if _pid not in partial_ids:
            partial_ids.append(_pid)
        for _ks in _ksegs:
            _d = {'start': (_ks.start_x, _ks.start_y),
                  'end': (_ks.end_x, _ks.end_y),
                  'width': _ks.width, 'layer': _ks.layer,
                  'net_id': _pid}
            all_new_segments.append(_d)
            _partial_emitted_segs.append(_d)
            _prov_seg(_pid, _ks.layer, _ks.start_x, _ks.start_y,
                      _ks.end_x, _ks.end_y, 'partial-restore')
        for _kv in _kvias:
            _d = {'x': _kv.x, 'y': _kv.y, 'size': _kv.size,
                  'drill': _kv.drill, 'layers': _kv.layers,
                  'net_id': _pid}
            all_new_vias.append(_d)
            _partial_emitted_vias.append(_d)
            _prov_via(_pid, _kv.x, _kv.y, 'partial-restore')

    # The ripped signal nets' old copper is excluded from the OUTPUT but still
    # sits in pcb_data here (stripped only at write time). Drop it before the
    # reconnect and the fill-aware sweep so both see the same obstacles as the
    # written board -- else a via site that is clear in the output looks
    # blocked and the pad is wrongly left floating.
    if ripped_net_ids:
        pcb_data.segments = [s for s in pcb_data.segments
                             if s.net_id not in ripped_net_ids]
        pcb_data.vias = [v for v in pcb_data.vias if v.net_id not in ripped_net_ids]

    # Route trace (#482): the island-join tracks/vias added and blocker copper
    # ripped by the per-net repair pass above.
    if _ptrace is not None:
        _ptrace.capture(pcb_data, 'plane-join')

    # #347 (core1106 CLK1P): a net ripped or partially dropped for a pad
    # repair must not depend on a LATER chain step existing to reconnect it --
    # self-run the standard route.py reconnect scoped to this run's own
    # casualties, IN MEMORY, for BOTH fronts. It runs BEFORE the final
    # fill-aware verification below on purpose: the reconnect can re-route a
    # ripped blocker back down the very corridor it was ripped from,
    # re-pinching a plane island the earlier per-pad checks had just passed
    # (hex_gateway C201/C205 shipped stranded behind a "9/10 reconnected"
    # report) -- the sweep below must verify, and repair, the fill AGAINST the
    # reconnected copper.
    global LAST_RIPPED_RECONNECT
    LAST_RIPPED_RECONNECT = None
    _casualties = list(dict.fromkeys(ripped_net_ids + partial_ids))
    # GUI passes dry_run=True meaning 'no file write'; routing already
    # happened, so the in-memory reconnect still runs (return_results). A CLI
    # --dry-run skips it.
    if _casualties and (return_results or not dry_run):
        if _PLANE_RESTORE_FIRST and _casualties:
            # #517 arm 4: BEFORE re-routing, try the cheapest reconnect there
            # is -- put each casualty's ORIGINAL copper back verbatim where its
            # corridor is still free (the rip cleared space for a tap; if the
            # tap's copper does not actually conflict with the original run,
            # nothing ever needed to move). Collision-checked with the shared
            # #134 primitive; conflicting nets fall through to the re-route
            # exactly as before. Runs at reconnect time, not end-of-run like
            # custody, so other casualties' re-routes cannot claim the
            # corridor first.
            _rf = {'restored': 0, 'blocked': 0}
            from rip_up_reroute import _saved_route_collides as _src517
            for _cid in list(_casualties):
                if _cid in partial_ids or _cid not in pcb_data.nets:
                    continue  # partial kept-sets restore via their own channel
                _osegs4 = [s for s in _orig_segments if s.net_id == _cid]
                _ovias4 = [v for v in _orig_vias if v.net_id == _cid]
                if not _osegs4 and not _ovias4:
                    continue
                if _src517({'new_segments': _osegs4, 'new_vias': _ovias4},
                           pcb_data, [_cid], clearance):
                    _rf['blocked'] += 1
                    continue
                pcb_data.segments.extend(_osegs4)
                pcb_data.vias.extend(_ovias4)
                for _po in _osegs4:
                    _prov_seg(_po.net_id, _po.layer, _po.start_x, _po.start_y,
                              _po.end_x, _po.end_y, 'restore-first')
                for _po in _ovias4:
                    _prov_via(_po.net_id, _po.x, _po.y, 'restore-first')
                while _cid in ripped_net_ids:
                    ripped_net_ids.remove(_cid)
                while _cid in inplace_reconnected_ids:
                    # Original copper is back verbatim; the writer must keep
                    # the input text, not strip it (#524 resurrection guard).
                    inplace_reconnected_ids.remove(_cid)
                _casualties.remove(_cid)
                if corridor_ghosts is not None:
                    corridor_ghosts.drop_net(_cid)
                _rf['restored'] += 1
                print(f"  (#517 restore-first: {pcb_data.nets[_cid].name} "
                      f"identity-restored, corridor still free)")
            if _rf['restored'] or _rf['blocked']:
                print(f"  (#517 restore-first: {_rf['restored']} restored "
                      f"verbatim, {_rf['blocked']} corridor(s) occupied -> "
                      f"re-route)")

        _cnames = [pcb_data.nets[n].name for n in _casualties if n in pcb_data.nets]
        if _cnames:
            print(f"\nReconnecting {len(_cnames)} net(s) this run ripped for pad "
                  f"repairs: {', '.join(_cnames)}")
            if progress_callback:
                progress_callback(0, 0, f"Reconnecting {len(_cnames)} ripped net(s)...")
            try:
                from route import batch_route
                # #338/#441: resolve the edge floor from the ORIGINAL input's
                # project (the output board's sibling .kicad_pro may not exist
                # yet), floored at the fab edge minimum. Do NOT forward this
                # function's board_edge_clearance -- that is the plane-zone
                # inset, not an enforced routing floor.
                try:
                    from fix_kicad_drc_settings import effective_board_edge_clearance
                    _edge = effective_board_edge_clearance(input_file, 0.0)
                except Exception:
                    _edge = 0.0
                # #513 item 5: re-route each ripped net at the width it ARRIVED
                # with (snapshotted before any rip), not this invocation's
                # default -- a repair retry that omitted --power-nets used to
                # silently drop a 1.5mm power net to the 0.25 default (nascom
                # VCC). A caller-supplied --power-nets entry for the net wins.
                _pn = list(power_nets or [])
                _pw = list(power_nets_widths or [])
                if len(_pn) == len(_pw):  # only merge when the pair is coherent
                    from net_queries import matches_net_filter as _mnf513
                    for _cnid in _casualties:
                        _cw = _input_net_widths.get(_cnid, 0.0)
                        _cn = (pcb_data.nets[_cnid].name
                               if _cnid in pcb_data.nets else None)
                        if not _cn or _cw <= (track_width or 0.0) + 1e-6:
                            continue
                        if _pn and _mnf513(_cn, _pn):
                            continue  # explicit power width wins
                        _pn.append(_cn)
                        _pw.append(_cw)
                        print(f"  Preserving routed width {_cw}mm for ripped "
                              f"net {_cn} across the reconnect (this run's "
                              f"default is {track_width}mm)")
                _ok, _fail, _t, _rdata = batch_route(
                    input_file, "", _cnames,
                    layers=routing_layers,
                    track_width=track_width, clearance=clearance,
                    via_size=via_size, via_drill=via_drill,
                    grid_step=grid_step, max_iterations=max_iterations,
                    power_nets=_pn or None, power_nets_widths=_pw or None,
                    board_edge_clearance=_edge,
                    disable_bga_zones=([] if no_bga_zone else None),
                    # #434: forward the map resolved from the ORIGINAL input's
                    # project (batch_route's own auto-read would find no
                    # netclasses next to a not-yet-written output).
                    net_clearances=net_clearances,
                    hole_to_hole_clearance=hole_to_hole_clearance,
                    # #527: forward progress/cancel -- a multi-net reconnect
                    # used to run minutes behind one static message.
                    progress_callback=(
                        (lambda c, t, m: progress_callback(
                            c, t, f"Reconnect: {m}"))
                        if progress_callback else None),
                    cancel_check=cancel_check,
                    return_results=True, pcb_data=pcb_data,
                    # #540 item 2: the end-of-run batch routes the casualties
                    # themselves, so only ghosts NOT in this batch remain --
                    # normally none, but a cancel/partial path can leave some.
                    # #562 RE-ENTRY GUARD: batch_route's plane finalize calls THIS
                    # engine, so a sub-run that runs its own finalize recurses:
                    # finalize -> repair -> batch_route -> finalize -> ... (measured
                    # 57 levels on schoko, ~7 GB before the cap killed it). This
                    # sub-run is a repair DETAIL, not a chain step -- never finalize.
                    final_reconcile=False,
                    **_ghost_kwargs(corridor_ghosts, _casualties))

                def _sd(_s):
                    return {'start': (_s.start_x, _s.start_y),
                            'end': (_s.end_x, _s.end_y),
                            'width': _s.width, 'layer': _s.layer,
                            'net_id': _s.net_id}

                def _vd(_v):
                    return {'x': _v.x, 'y': _v.y, 'size': _v.size,
                            'drill': _v.drill, 'layers': _v.layers,
                            'net_id': _v.net_id}
                for _r in _rdata.get('results', []):
                    for _s in (_r.get('new_segments') or []):
                        all_new_segments.append(_sd(_s))
                        _prov_seg(_s.net_id, _s.layer, _s.start_x, _s.start_y,
                                  _s.end_x, _s.end_y, 'reconnect')
                    for _v in (_r.get('new_vias') or []):
                        all_new_vias.append(_vd(_v))
                        _prov_via(_v.net_id, _v.x, _v.y, 'reconnect')
                for _v in (_rdata.get('all_swap_vias') or []):
                    all_new_vias.append(_vd(_v))
                    _prov_via(_v.net_id, _v.x, _v.y, 'reconnect')
                for _s in (_rdata.get('all_swap_segments') or []):
                    all_new_segments.append(_sd(_s))
                    _prov_seg(_s.net_id, _s.layer, _s.start_x, _s.start_y,
                              _s.end_x, _s.end_y, 'reconnect')
                _consume_inner_strips(_rdata, "reconnect")
                # The reconnect mutated copper, and the plane fill models are
                # cached on pcb_data (they subtract foreign-copper halos, so a
                # reconnected blocker can re-pinch a fill they no longer see).
                # Drop the cache so the fill-aware sweep below re-checks
                # against the REAL post-reconnect fill -- without this the
                # sweep passed C201/C205 on hex_gateway while the written
                # board had them stranded on a re-severed island.
                try:
                    from plane_fill_model import _CACHE_ATTR as _PFM_CACHE
                    if hasattr(pcb_data, _PFM_CACHE):
                        delattr(pcb_data, _PFM_CACHE)
                except Exception:
                    pass
                LAST_RIPPED_RECONNECT = {'nets': _cnames,
                                         'successful': _ok, 'failed': _fail}
                if _fail:
                    print(f"{RED}  {_fail} ripped net(s) could NOT be reconnected"
                          f"{RESET}")
            except Exception as _e:
                print(f"{RED}  ripped-net reconnect pass failed: {_e}{RESET}")

            # RESTORE-ON-FAILURE custody (#88 for the repair front): a ripped
            # net whose reconnect failed must get its ORIGINAL copper back --
            # shipping the partial reroute is strictly worse than the input
            # (quickfeather /PDM.DATA: ripped for a pad repair, reconnect
            # failed at R33, shipped broken behind a log line). In-memory and
            # BEFORE the fill-aware sweep/gate, so every later check sees the
            # true board. The restored net's conflicting NEW repair copper is
            # dropped (the rip existed to clear that corridor); its pad may
            # then re-grade disconnected -- the honest pre-existing state.
            _still_open = []
            for _cid in _casualties:
                if _cid in pcb_data.nets:
                    from check_connected import check_net_connectivity as _cnc_r
                    _rr = _cnc_r(_cid,
                                 [s for s in pcb_data.segments
                                  if s.net_id == _cid],
                                 [v for v in pcb_data.vias if v.net_id == _cid],
                                 pcb_data.pads_by_net.get(_cid, []),
                                 [z for z in (getattr(pcb_data, 'zones', None)
                                              or []) if z.net_id == _cid],
                                 pcb_data=pcb_data)
                    if not _rr.get('connected'):
                        _still_open.append(_cid)
            if _still_open:
                # Per-NET try (#509 part 3): the catch used to wrap the whole
                # pass, so one bad net abandoned custody for every other one --
                # and the run still exited rc=0 with only a red line to show
                # for it. A dict/object mix-up in this block silently disabled
                # custody across two full A/B arms and read as "no conflicts
                # arose" (RESTORED=0 REFUSED=0 is indistinguishable from
                # nothing-to-do). route.py's equivalent restore path has NO
                # blanket catch at all and fails loudly; scope it tightly here
                # and always report a tally so a zero is never ambiguous.
                _cust = {'restored': 0, 'refused': 0, 'errored': 0}
                # In-memory snapshot, NOT parse_kicad_pcb(input_file): the
                # GUI's input_file is stale on disk (#493 item 3).
                for _cid in list(_still_open):
                  try:
                      _osegs = [s for s in _orig_segments
                                if s.net_id == _cid]
                      _ovias = [v for v in _orig_vias if v.net_id == _cid]
                      if not _osegs and not _ovias:
                          continue  # nothing to restore
                      _nm = pcb_data.nets[_cid].name \
                          if _cid in pcb_data.nets else f"net_{_cid}"
                      # REFUSE, do not displace (#509 part 1). route.py
                      # brackets rip and restore inside ONE attempt
                      # (single_ended_loop `rips = []` ... `_restore_rips`),
                      # so nothing can move into the vacated corridor and its
                      # collision test is a documented backstop for a case
                      # that "cannot happen". Here the window spans the whole
                      # reconnect batch, so conflicts are routine -- and the
                      # old code resolved them by DELETING the other net's
                      # copper, destroying a route that SUCCEEDED in order to
                      # reinstate one that FAILED. Refuse instead and leave
                      # the net unrouted, exactly as restore_net does via
                      # refused_sink: unrouted beats shorted. Refusing is
                      # safe even when the test is conservative; deleting is
                      # not.
                      #
                      # Test with the SHARED #134 primitive (#509 part 2),
                      # not a local endpoint-sampling copy: the old _collides
                      # measured only the candidate's ENDPOINTS against the
                      # restored segments, so two tracks crossing at right
                      # angles -- both endpoints far apart, intersecting dead
                      # centre -- passed it (spartan6_6layer GPIO-N22 x
                      # GPIO-P20: endpoints 1.600/0.400 vs a 0.230 rule,
                      # true crossing 0.000). It also tested only a
                      # segment's START point against restored vias.
                      from rip_up_reroute import _saved_route_colliders
                      _culprits = _saved_route_colliders(
                          {'new_segments': _osegs, 'new_vias': _ovias},
                          pcb_data, [_cid], clearance)
                      if _culprits:
                          print(f"  {YELLOW}REFUSED restore of {_nm}: copper "
                                f"routed meanwhile occupies its corridor; "
                                f"left unrouted rather than displacing it"
                                f"{RESET}")
                          # #517: name the pass that placed the occupying
                          # copper -- residual refusals point at the next
                          # custody lever. 'pre-existing' means the collision
                          # test flagged copper this run never touched (a
                          # conservative-test finding, not an occupier).
                          _attr: Dict[tuple, int] = {}
                          for _kind, _obj in _culprits:
                              _tag = _prov_lookup(_kind, _obj)
                              _onm = (pcb_data.nets[_obj.net_id].name
                                      if _obj.net_id in pcb_data.nets
                                      else f"net_{_obj.net_id}")
                              _k = (_tag, _onm)
                              _attr[_k] = _attr.get(_k, 0) + 1
                          _parts = [
                              f"{_t} {_n} x{_c}" for (_t, _n), _c in
                              sorted(_attr.items(), key=lambda kv: -kv[1])]
                          print(f"  {YELLOW}  occupied by: "
                                f"{'; '.join(_parts)}{RESET}")
                          _cust['refused'] += 1
                          continue
                      # Corridor still clear: drop the failed reroute and
                      # reinstate the original (in-memory + write-list).
                      pcb_data.segments[:] = [
                          s for s in pcb_data.segments if s.net_id != _cid]
                      pcb_data.vias[:] = [
                          v for v in pcb_data.vias if v.net_id != _cid]
                      all_new_segments[:] = [
                          d for d in all_new_segments
                          if d.get('net_id') != _cid]
                      all_new_vias[:] = [
                          d for d in all_new_vias if d.get('net_id') != _cid]
                      # restore the originals in-memory; the writer keeps
                      # the file copper because the net leaves the exclude
                      # lists below
                      pcb_data.segments.extend(_osegs)
                      pcb_data.vias.extend(_ovias)
                      for _po in _osegs:
                          _prov_seg(_po.net_id, _po.layer, _po.start_x,
                                    _po.start_y, _po.end_x, _po.end_y,
                                    'custody-restore')
                      for _po in _ovias:
                          _prov_via(_po.net_id, _po.x, _po.y,
                                    'custody-restore')
                      for _lst in (ripped_net_ids, partial_ids,
                                   inplace_reconnected_ids):
                          while _cid in _lst:
                              _lst.remove(_cid)
                      _still_open.remove(_cid)
                      _cust['restored'] += 1
                      print(f"  {YELLOW}RESTORED {_nm}: reconnect failed; "
                            f"original copper reinstated{RESET}")
                  except Exception as _e:
                      _cust['errored'] += 1
                      print(f"{RED}  custody FAILED for "
                            f"{pcb_data.nets[_cid].name if _cid in pcb_data.nets else _cid}"
                            f": {_e}{RESET}")
                try:
                    from plane_fill_model import \
                        _CACHE_ATTR as _PFM_CACHE_R
                    if hasattr(pcb_data, _PFM_CACHE_R):
                        delattr(pcb_data, _PFM_CACHE_R)
                except Exception:
                    pass
                # Unconditional tally: without it a silent zero reads exactly
                # like "nothing to do" (#509 part 3).
                print(f"  custody: {_cust['restored']} restored, "
                      f"{_cust['refused']} refused, {_cust['errored']} errored "
                      f"of {len(_casualties)} casualty net(s)")
                global LAST_RIPPED_CUSTODY
                LAST_RIPPED_CUSTODY = dict(_cust,
                                           casualties=len(_casualties))
            if _still_open:
                _report_unrouted_ripped_nets(pcb_data, _still_open)
            # A10: the still-open set is the run's real damage. Publish the
            # NAMES so the summary, the exit code and the ledger can all carry
            # it -- a count in a log line is not a verdict channel.
            global LAST_RIPPED_STILL_OPEN
            LAST_RIPPED_STILL_OPEN = [
                (pcb_data.nets[_cid].name if _cid in pcb_data.nets
                 else f"net_{_cid}") for _cid in _still_open]
            if corridor_ghosts is not None:
                # #517 arm 2: custody-defined lifetime -- a casualty that is
                # connected again (reconnected, or custody-restored) has real
                # copper as its own obstacle; its ghost would only repel the
                # fill-aware sweep below from a corridor that is no longer
                # vacated. Still-open nets keep their reservation through the
                # sweep.
                _open_set = set(_still_open)
                for _cid in _casualties:
                    if _cid not in _open_set:
                        corridor_ghosts.drop_net(_cid)

    # #524 second path: a FAILED immediate reconnect leaves its partial copper
    # in the write list; the end-of-run batch_route then rips that copper
    # INTERNALLY (its own rip/refuse machinery, invisible to the
    # _tap_pad_with_ripup purge), and LATER passes (graze/dead-end cleanup on
    # the failed nets' dangling fragments) withdraw yet more board copper --
    # while the write list still carries it (astro SDA x66 / +3.3V x230
    # file-only ghosts, drc 39/kdrc 27; residual x24/x115 when this ran only
    # once, post-custody). Reconcile the write list against the BOARD for
    # every net any rip/reconnect touched, IMMEDIATELY BEFORE EACH WRITE, so
    # no later withdrawal can slip past it. An emission with no matching
    # board copper is withdrawn copper and must not ship.
    _recon_scope_ids = (set(_casualties) | set(ripped_net_ids)
                        | set(inplace_reconnected_ids))

    def _reconcile_write_list_vs_board(_label):
        # COUNTED multiset, not a set: the immediate reconnect and the
        # end-of-run reconnect can emit the SAME copper twice (batch_route
        # re-reports adopted existing pieces of a re-routed net), and a
        # set-based keep let both copies ship while the board holds one
        # (astro +3.3V x115 / SDA x24 duplicate emissions).
        if not _recon_scope_ids:
            return
        from collections import Counter
        _board_segs = Counter()
        for _s in pcb_data.segments:
            if _s.net_id in _recon_scope_ids:
                _a = (round(_s.start_x, 3), round(_s.start_y, 3))
                _b = (round(_s.end_x, 3), round(_s.end_y, 3))
                _board_segs[(frozenset((_a, _b)), _s.layer, _s.net_id)] += 1
        _board_vias = Counter()
        for _v in pcb_data.vias:
            if _v.net_id in _recon_scope_ids:
                _board_vias[(round(_v.x, 3), round(_v.y, 3), _v.net_id)] += 1
        _gs, _gv = 0, 0
        _kept_s = []
        for _d in all_new_segments:
            if _d.get('net_id') in _recon_scope_ids:
                _a = (round(_d['start'][0], 3), round(_d['start'][1], 3))
                _b = (round(_d['end'][0], 3), round(_d['end'][1], 3))
                _k = (frozenset((_a, _b)), _d['layer'], _d['net_id'])
                if _board_segs.get(_k, 0) <= 0:
                    _gs += 1
                    continue
                _board_segs[_k] -= 1
            _kept_s.append(_d)
        _kept_v = []
        for _d in all_new_vias:
            if _d.get('net_id') in _recon_scope_ids:
                _k = (round(_d['x'], 3), round(_d['y'], 3), _d['net_id'])
                if _board_vias.get(_k, 0) <= 0:
                    _gv += 1
                    continue
                _board_vias[_k] -= 1
            _kept_v.append(_d)
        if _gs or _gv:
            all_new_segments[:] = _kept_s
            all_new_vias[:] = _kept_v
            print(f"  (#524 write-list reconcile [{_label}]: dropped {_gs} "
                  f"segment(s) and {_gv} via(s) withdrawn from the board "
                  f"or duplicated)")

    _reconcile_write_list_vs_board('post-custody')

    # A partial restore's kept-set is emitted into the write list ABOVE, before
    # the reconnect runs, and unconditionally. But the reconnect may RE-ROUTE
    # that same net (it is queued as a casualty) and delete the kept copper
    # from pcb_data -- and the write list still carried it, so the OUTPUT
    # shipped copper the router had legitimately withdrawn. #463
    # spartan6_6layer: /RAM/DDR-LDM was partially restored on In3.Cu, the
    # reconnect moved it to In1.Cu (solo source switch) and handed the corridor
    # to /RAM/DDR-D11, yet the stale In3.Cu run was still written -- a 0.220mm
    # COLLINEAR overlap between two different DDR nets, i.e. a hard short that
    # existed in no in-memory state, only in the file. The existing guards
    # cover a net re-ripped to a full rip, and an earlier kept-set superseded
    # by a later one; neither sees a reconnect reroute.
    #
    # pcb_data is authoritative once the reconnect and its restore-on-failure
    # custody have run: drop any emitted piece no longer live there. Pure
    # write-list filter -- no routing, no obstacle work, no new structures.
    # (Identity, not value: an equal-looking dict may be legitimate copper from
    # another pass. _stale_* holds the references while we filter, so the id()
    # keys cannot be recycled underneath us.)
    _n_s, _n_v, _names = drop_withdrawn_partial_restores(
        _partial_emitted_segs, _partial_emitted_vias,
        all_new_segments, all_new_vias, pcb_data)
    if _n_s or _n_v:
        print(f"  dropped {_n_s} stale partial-restore segment(s) and "
              f"{_n_v} via(s) the reconnect withdrew: {', '.join(_names)}")

    # Post-reconnect join round: the reconnect's copper subtracts from the
    # plane fill, so it can re-sever a region the round-1 joins had connected
    # (hex_gateway C201/C205: the reconnected blocker re-pinched their B.Cu
    # island) -- and a stranded REGION cannot be fixed by the per-pad via
    # sweep below (its forced vias land on the same island). Re-check each
    # plane net fill-aware and re-run the region join for the broken ones
    # against the post-reconnect obstacles. One bounded round: the join adds
    # only plane-net copper and rips nothing, so it cannot undo the reconnect.
    if _casualties and _round2_ctx and (return_results or not dry_run):
        from check_connected import check_net_connectivity as _cnc2
        _zbn: Dict[int, list] = {}
        for _z in (getattr(pcb_data, 'zones', None) or []):
            if getattr(_z, 'net_id', None) is not None:
                _zbn.setdefault(_z.net_id, []).append(_z)
        for _nid, _ctx in _round2_ctx.items():
            _res2 = _cnc2(_nid,
                          [s for s in pcb_data.segments if s.net_id == _nid],
                          [v for v in pcb_data.vias if v.net_id == _nid],
                          pcb_data.pads_by_net.get(_nid, []),
                          _zbn.get(_nid, []), pcb_data=pcb_data)
            if _res2.get('connected'):
                continue
            print(f"\n[{_ctx['net_name']}] fill re-pinched by the ripped-net "
                  f"reconnect -- re-running the region join:")
            _b2, _lm2 = build_base_obstacles(
                exclude_net_ids={_nid},
                routing_layers=routing_layers,
                pcb_data=pcb_data,
                config=config,
                track_width=min_track_width,
                track_via_clearance=track_via_clearance,
                hole_to_hole_clearance=hole_to_hole_clearance)
            _rsegs, _rvias, _radd, _rpaths, _ = route_disconnected_regions(
                net_id=_nid,
                net_name=_ctx['net_name'],
                plane_layer=_ctx['primary_layer'],
                zone_bounds=_ctx['zone_bounds'],
                pcb_data=pcb_data,
                config=config,
                base_obstacles=_b2,
                layer_map=_lm2,
                zone_clearance=_ctx['max_zone_clearance'],
                max_track_width=max_track_width,
                min_track_width=min_track_width,
                track_via_clearance=track_via_clearance,
                hole_to_hole_clearance=hole_to_hole_clearance,
                analysis_grid_step=analysis_grid_step,
                max_iterations=max_iterations,
                verbose=verbose,
                zone_layers=_ctx['net_zone_layers'],
                zone_clearances=_ctx['zone_clearances'],
                progress_callback=progress_callback,
                cancel_check=cancel_check)
            if _radd > 0:
                all_new_segments.extend(_rsegs)
                all_new_vias.extend(_rvias)
                total_routes += _radd
                total_regions += _radd + 1
                total_vias += len(_rvias)
                for _s in _rsegs:
                    pcb_data.segments.append(Segment(
                        start_x=_s['start'][0], start_y=_s['start'][1],
                        end_x=_s['end'][0], end_y=_s['end'][1],
                        width=_s['width'], layer=_s['layer'],
                        net_id=_s['net_id']))
                for _v in _rvias:
                    pcb_data.vias.append(Via(
                        x=_v['x'], y=_v['y'], size=_v['size'],
                        drill=_v['drill'], layers=['F.Cu', 'B.Cu'],
                        net_id=_v['net_id']))
                # Round-2 copper is foreign to any OTHER plane net's fill:
                # refresh the models so the final sweep verifies real fills.
                try:
                    from plane_fill_model import _CACHE_ATTR as _PFM2
                    if hasattr(pcb_data, _PFM2):
                        delattr(pcb_data, _PFM2)
                except Exception:
                    pass

    # GUARANTEED JOIN (#479 duodyne): the join plan and the sweep both lean
    # on zone-fill MODELS (the 0.5mm analysis raster; the cached fill
    # validators), which can disagree with the real pour -- duodyne's raster
    # merged islands KiCad separates, so 20 pads shipped floating behind an
    # all-joins-OK report. Final gate, immune to that quantization: drop the
    # fill-model cache, re-run the AUTHORITATIVE pad union-find, and when a
    # plane net is still split, route it like any multipoint net with an
    # in-memory batch_route -- existing copper (fill, straps, barrels) is
    # terminal credit, so only the true gaps get MST edges. Then re-check
    # and report honestly.
    if repair_pads and (return_results or not dry_run):
        try:
            from plane_fill_model import _CACHE_ATTR as _PFM_CACHE3
            if hasattr(pcb_data, _PFM_CACHE3):
                delattr(pcb_data, _PFM_CACHE3)
        except Exception:
            pass
        from check_connected import check_net_connectivity as _cnc3
        _zbn3: Dict[int, list] = {}
        for _z in (getattr(pcb_data, 'zones', None) or []):
            if getattr(_z, 'net_id', None) is not None:
                _zbn3.setdefault(_z.net_id, []).append(_z)

        # Lazy KiCad-oracle verdict for the gate (see the consult below):
        # [None]=not yet queried, [False]=queried and unavailable, else the
        # link list. One query serves every gate net; a gate repair that
        # adds copper resets it.
        _gate_oracle_links: list = [None]

        def _gate_oracle_query():
            import tempfile
            try:
                _tmp = tempfile.NamedTemporaryFile(
                    suffix='.kicad_pcb', delete=False)
                _tmp.close()
                _nm10 = (pcb_data.net_id_to_name
                         if pcb_data.kicad_version >= KICAD_10_MIN_VERSION
                         else None)
                _reconcile_write_list_vs_board('pre-oracle')
                _write_output(input_file, _tmp.name, all_new_segments,
                              all_new_vias, None,
                              net_id_to_name=_nm10,
                              exclude_net_ids=(ripped_net_ids + partial_ids
                                               + inplace_reconnected_ids))
                _links = None
                # DETERMINISTIC gate source (#490): kicad-cli DRC's threaded
                # connectivity gives different link reports run-to-run on
                # marginal boards, and the gate's verdicts steer RIP
                # decisions -- the top chaos lever in the repair. pcbnew's
                # ZONE_FILLER is measured-deterministic; exact_unconnected
                # clusters its fill truth reproducibly.
                # KICAD_LEGACY_GATE_ORACLE=1 restores kicad-cli for A/B.
                if not env_knobs.LEGACY_GATE_ORACLE:
                    try:
                        from kicad_exact_fill import exact_unconnected
                        _gnames = [pcb_data.nets[g].name
                                   for g in unique_nets
                                   if g in pcb_data.nets]
                        _links = exact_unconnected(
                            _tmp.name, _gnames, project_from=input_file)
                    except Exception as _xe:
                        print(f"  (exact gate source failed: {_xe}; "
                              f"falling back to kicad-cli)")
                        _links = None
                if _links is None:
                    from kicad_oracle import (find_kicad_cli,
                                              kicad_unconnected)
                    _cli = find_kicad_cli()
                    if not _cli:
                        return False
                    _links = kicad_unconnected(_tmp.name, _cli)
                try:
                    os.unlink(_tmp.name)
                except OSError:
                    pass
                try:  # the seeded sibling .kicad_pro (#513 item 12)
                    os.unlink(os.path.splitext(_tmp.name)[0] + '.kicad_pro')
                except OSError:
                    pass
                return False if _links is None else _links
            except Exception as _oe:
                print(f"  (gate oracle unavailable: {_oe})")
                return False

        for _nid, (_nname, _nlayers) in unique_nets.items():
            def _check3(_n=_nid):
                return _cnc3(_n,
                             [s for s in pcb_data.segments if s.net_id == _n],
                             [v for v in pcb_data.vias if v.net_id == _n],
                             pcb_data.pads_by_net.get(_n, []),
                             _zbn3.get(_n, []), pcb_data=pcb_data)
            _r3 = _check3()
            if _r3.get('connected'):
                continue
            # ORACLE VERIFY (#quickfeather U6.29): our checker's fill model
            # has a ~0.05mm quantization floor -- a legal fill corridor
            # narrower than ~3 raster columns reads as a split that KiCad's
            # exact polygon fill does not have. Acting on such a phantom is
            # how healthy nets got ripped chasing repairs KiCad never asked
            # for. Before routing anything, write the CURRENT state to a
            # temp board and ask kicad-cli: if KiCad reports NO unconnected
            # links for this net, the split is model quantization -- skip.
            # Oracle unavailable (no kicad-cli / DRC failed) => behave as
            # before. One DRC serves all gate nets (cached until a gate
            # repair adds copper). KICAD_NO_GATE_ORACLE=1 disables for A/B.
            if not env_knobs.NO_GATE_ORACLE:
                if _gate_oracle_links[0] is None:
                    _gate_oracle_links[0] = _gate_oracle_query()
                _gl = _gate_oracle_links[0]
                if _gl is not False and _gl is not None:
                    _net_links = [lk for lk in _gl if lk[0] == _nname]
                    print(f"  [{_nname}] gate oracle: KiCad reports "
                          f"{len(_net_links)} unconnected link(s) for this "
                          f"net ({len(_gl)} total)")
                    if not _net_links:
                        print(f"  [{_nname}] guaranteed-join gate: checker "
                              f"finds {_r3.get('num_components')} "
                              f"component(s), but KiCad reports the net "
                              f"complete (fill-model quantization) -- "
                              f"skipping repair")
                        continue
            _ncomp = _r3.get('num_components')
            print(f"\n[{_nname}] guaranteed-join gate: authoritative check "
                  f"finds {_ncomp} component(s) after repair -- routing the "
                  f"remaining gaps as a multipoint net:")
            # Which pad groups stayed split tells us WHERE the join plan and
            # the checker disagree (model-vs-checker divergence is the gate's
            # whole reason to exist) -- summarize each residual component.
            _bycomp: Dict[int, list] = {}
            for _loc, _cid in (_r3.get('pad_components') or {}).items():
                _bycomp.setdefault(_cid, []).append(_loc)
            for _cid, _locs in sorted(_bycomp.items(),
                                      key=lambda kv: -len(kv[1]))[:10]:
                _sample = ', '.join(
                    f"{_l[3]}({_l[0]:.1f},{_l[1]:.1f})" for _l in _locs[:3])
                print(f"    group {_cid}: {len(_locs)} pad(s): {_sample}")
            if env_knobs.GATE_DEBUG:
                from plane_fill_model import get_fill_models as _gfm_dbg
                _mods = _gfm_dbg(pcb_data, _nid)
                for _cid, _locs in sorted(_bycomp.items(),
                                          key=lambda kv: -len(kv[1]))[1:8]:
                    _l = _locs[0]
                    _isl = None
                    for _lay, _ms in _mods.items():
                        for _m in _ms:
                            _c = _m.query_component(_l[0], _l[1], size=1.6)
                            if _c:
                                _isl = (_lay, id(_m) % 1000, _c)
                    print(f"      [dbg] {_l[3]}@({_l[0]:.2f},{_l[1]:.2f}) "
                          f"island={_isl}")
                    if _isl is None:
                        continue
                    _hits = 0
                    for _s in pcb_data.segments:
                        if _s.net_id != _nid or _s.layer != _isl[0]:
                            continue
                        for _ex, _ey in ((_s.start_x, _s.start_y),
                                         (_s.end_x, _s.end_y)):
                            for _m in _mods[_isl[0]]:
                                if (id(_m) % 1000 == _isl[1]
                                        and _m.query_component(
                                            _ex, _ey, size=_s.width) == _isl[2]):
                                    _hits += 1
                                    print(f"        seg-endpoint "
                                          f"({_ex:.2f},{_ey:.2f}) w={_s.width} "
                                          f"credits island")
                    if not _hits:
                        print(f"        NO segment endpoint credits island")
            if progress_callback:
                progress_callback(0, 0, f"{_nname}: joining remaining gaps...")
            try:
                from route import batch_route
                _ok3, _fail3, _t3, _rdata3 = batch_route(
                    input_file, "", [_nname],
                    layers=routing_layers,
                    track_width=track_width, clearance=clearance,
                    via_size=via_size, via_drill=via_drill,
                    grid_step=grid_step, max_iterations=max_iterations,
                    power_nets=power_nets, power_nets_widths=power_nets_widths,
                    disable_bga_zones=([] if no_bga_zone else None),
                    net_clearances=net_clearances,
                    # #539: without this the gate's plane-net vias were placed
                    # at batch_route's 0.2 default on a 0.25-h2h board (muzy_
                    # zynq2's residual drill grazes -- same forwarding-gap
                    # class 8920cb0 fixed for the two reconnect calls).
                    hole_to_hole_clearance=hole_to_hole_clearance,
                    # #527: forward progress/cancel into the region-join
                    # sub-route (it can A* for minutes on a big pour).
                    progress_callback=(
                        (lambda c, t, m: progress_callback(
                            c, t, f"Region join: {m}"))
                        if progress_callback else None),
                    cancel_check=cancel_check,
                    return_results=True, pcb_data=pcb_data,
                    # #540 item 2: gate straps must not squat the pending
                    # casualties' corridors either.
                    # #562 RE-ENTRY GUARD: batch_route's plane finalize calls THIS
                    # engine, so a sub-run that runs its own finalize recurses:
                    # finalize -> repair -> batch_route -> finalize -> ... (measured
                    # 57 levels on schoko, ~7 GB before the cap killed it). This
                    # sub-run is a repair DETAIL, not a chain step -- never finalize.
                    final_reconcile=False,
                    **_ghost_kwargs(corridor_ghosts, {_nid}))
                for _r in _rdata3.get('results', []):
                    all_new_segments.extend(
                        {'start': (_s.start_x, _s.start_y),
                         'end': (_s.end_x, _s.end_y),
                         'width': _s.width, 'layer': _s.layer,
                         'net_id': _s.net_id}
                        for _s in (_r.get('new_segments') or []))
                    all_new_vias.extend(
                        {'x': _v.x, 'y': _v.y, 'size': _v.size,
                         'drill': _v.drill, 'layers': _v.layers,
                         'net_id': _v.net_id}
                        for _v in (_r.get('new_vias') or []))
                # Swap channels too (parity audit H1a): the inner route's
                # stub-layer switches append swap copper to pcb_data; the
                # post-gate check then reads a board whose load-bearing swap
                # the write-list lacks -- phantom success. Mirror the adds;
                # loudly flag the channels the plane writer cannot mirror.
                all_new_segments.extend(
                    {'start': (_s.start_x, _s.start_y),
                     'end': (_s.end_x, _s.end_y),
                     'width': _s.width, 'layer': _s.layer,
                     'net_id': _s.net_id}
                    for _s in (_rdata3.get('all_swap_segments') or []))
                all_new_vias.extend(
                    {'x': _v.x, 'y': _v.y, 'size': _v.size,
                     'drill': _v.drill, 'layers': _v.layers,
                     'net_id': _v.net_id}
                    for _v in (_rdata3.get('all_swap_vias') or []))
                for _ch in ('all_segment_modifications', 'pad_swaps',
                            'single_ended_target_swap_info'):
                    if _rdata3.get(_ch):
                        print(f"  {RED}[{_nname}] gate: inner route produced "
                              f"{len(_rdata3[_ch])} {_ch} entr(ies) the plane "
                              f"writer cannot mirror -- board/file may "
                              f"diverge here{RESET}")
                _consume_inner_strips(_rdata3, "gate")
                _gate_oracle_links[0] = None   # state changed; re-query for later nets
                _r3b = _check3()
                if _r3b.get('connected'):
                    print(f"  {GREEN}[{_nname}] guaranteed-join gate: net now "
                          f"fully connected{RESET}")
                else:
                    print(f"  {RED}[{_nname}] guaranteed-join gate: "
                          f"{len(_r3b.get('disconnected_pads') or [])} pad(s) "
                          f"still disconnected -- ships incomplete{RESET}")
            except Exception as _e:
                print(f"  {RED}[{_nname}] guaranteed-join gate failed: {_e}{RESET}")


    # Final fill-aware verification (glasgow U30 U1.27 +3V3). The per-pad check
    # (find_unconnected_plane_pads / _smd_pad_reaches_layer) is layer-aware: it
    # treats a pad as connected once it reaches the zone LAYER, even via a
    # floating island, so a reuse-tap onto an island reports success while the pad
    # never reaches the connected plane FILL. Re-check each plane net with the same
    # zone/fill-aware union-find check_connected uses (cheap - ~0.5s/board, once at
    # the end), and force a real via (disable_reuse) for any pad still floating, so
    # no plane pad is left SILENTLY disconnected after reporting success. It runs
    # AFTER the rip-casualty reconnect above, so a fill region the reconnect
    # re-pinched is re-verified and re-tapped here instead of shipping stranded.
    if repair_pads:
        from check_connected import check_net_connectivity
        zones_by_net: Dict[int, list] = {}
        for z in (getattr(pcb_data, 'zones', None) or []):
            if getattr(z, 'net_id', None) is not None:
                zones_by_net.setdefault(z.net_id, []).append(z)
        # Forced last-resort via sizes, largest first, as fab-manufacturable
        # (diameter, drill) pairs: the configured via, then the active fab-tier floor
        # ladder (nominal floor, then any escalation rung -- the more-costly advanced
        # 0.25/0.15 via 'standard' escalates to, #237). A fine-pitch pad flanked by
        # other-net copper often cannot take the nominal via but fits a smaller
        # fab-legal one; we never go below the deepest fab floor.
        from list_nets import fab_floor_ladder, warn_fab_escalation
        _ncu = len([l for l in (pcb_data.board_info.copper_layers or routing_layers)
                    if l.endswith('.Cu')]) or 2
        _ladder = fab_floor_ladder(_ncu)
        _cands = [(via_size, via_drill, False)]
        _cands += [(f['via_diameter'], f['via_drill'], _i > 0)
                   for _i, f in enumerate(_ladder)]
        via_pairs, _escalated_pairs = [], set()
        for _vd, _dr, _is_esc in _cands:
            _vd, _dr = round(_vd, 3), round(_dr, 3)
            if _dr < _vd <= via_size + 1e-9 and (_vd, _dr) not in via_pairs:
                via_pairs.append((_vd, _dr))
                if _is_esc:
                    _escalated_pairs.add((_vd, _dr))
        for net_id, (net_name, net_zone_layers) in unique_nets.items():
            net_segs = [s for s in pcb_data.segments if s.net_id == net_id]
            net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
            net_pads = pcb_data.pads_by_net.get(net_id, [])
            res = check_net_connectivity(net_id, net_segs, net_vias, net_pads,
                                         zones_by_net.get(net_id, []),
                                         pcb_data=pcb_data)
            if res.get('connected'):
                continue
            # Launch layers per pad (#494) -- see plane_tap_launch_layers.
            # KICAD_NO_SWEEP_PLATED=1 restores the old single-concrete-layer
            # resolution + plated skip, for one-env-var A/B on identical code.
            _no_plated = env_knobs.NO_SWEEP_PLATED
            pad_by_key = {}
            for p in net_pads:
                if _no_plated:
                    pl = next((l for l in p.layers
                               if l.endswith('.Cu')
                               and not l.startswith('*')), None)
                    cands = [pl] if (pl and not pad_is_plated_through(p)
                                     and getattr(p, 'pad_type', '')
                                     != 'np_thru_hole') else []
                else:
                    cands = plane_tap_launch_layers(p, net_zone_layers,
                                                    routing_layers)
                pad_by_key[(round(p.global_x, 3), round(p.global_y, 3),
                            p.component_ref)] = (p, cands)
            # Relax the board-edge clearance for this forced last-resort tap: the
            # pad being repaired is already placed at the edge, so a via INSIDE it
            # is no closer to the edge than the pad itself (the fab accepts that),
            # and an edge pad would otherwise be unconnectable -- which is what made
            # the normal tap fall back to a bogus reuse in the first place.
            tap_config = replace(config, layers=routing_layers,
                                 hole_to_hole_clearance=hole_to_hole_clearance,
                                 board_edge_clearance=0.0)
            # Cross-pad via-map reuse for this net's forced-via sweep (#263). A
            # fresh instance (not the repair pass's): region-connect copper was
            # added since, and this pass's edge-relaxed config keys differ anyway.
            shared_maps = SharedViaMaps(pcb_data, net_id)
            # Fresh T6 oracle for the same reason (copper changed since the
            # repair pass): forces the last-resort via into a MAIN zone outline
            # and keeps the #373 track fallback off floating same-net copper.
            sweep_oracle = PlaneComponentOracle(pcb_data, net_id)
            reported = False
            # Custody baseline (#494): the pad's floating-entry count from
            # the most recent authoritative verdict, advanced as repairs
            # are accepted. A repair is kept only if it STRICTLY reduces
            # this pad's count -- see pad_repair_made_progress.
            _cur_dp = res.get('disconnected_pads') or []
            _sweep_pads = res.get('disconnected_pads', [])
            for _sw_idx, (fx, fy, _flayer, fref) in enumerate(_sweep_pads):
                if cancel_check and cancel_check():
                    print("    (cancelled)")
                    break
                pp = pad_by_key.get((round(fx, 3), round(fy, 3), fref))
                if pp is None:
                    continue
                pad, pad_cands = pp
                # #527: the via ladder below (sizes x layers x full-radius
                # searches) can take many seconds per pad -- report each one.
                if progress_callback:
                    progress_callback(_sw_idx + 1, len(_sweep_pads),
                                      f"{net_name}: forcing via "
                                      f"{pad.component_ref}.{pad.pad_number}")
                # #494: the old guard also skipped PLATED barrels, on the
                # premise "plated barrels are already plane-tied by the
                # fill". That cannot hold here -- this loop iterates
                # res['disconnected_pads'], i.e. pads the fill-aware check
                # JUST reported still floating (see the banner below), so
                # for exactly these pads the fill did not reach the barrel.
                # NPTH still skips: it has no copper at all (#328).
                if not pad_cands:
                    continue
                pad_layer = pad_cands[0]
                name = f"{pad.component_ref}.{pad.pad_number} ({net_name})"
                if not reported:
                    print(f"\n[{net_name}] fill-aware re-check: pad(s) reported tapped but "
                          f"still floating (reached an island, not the plane) -- forcing a via:")
                    reported = True
                print(f"    Pad {pad.component_ref}.{pad.pad_number} ({pad_layer})...",
                      end=" ", flush=True)
                # Try each fab-legal via (largest first): a fine-pitch pad flanked
                # by other-net copper often cannot take the nominal via but fits a
                # smaller fab-floor one. Search the full max_search_radius so a pad
                # whose only open via site is farther out is still reached (the
                # batched grid_router query keeps the wide search cheap). Skip the
                # distant-trace fallback - we want a real via, nearest-first.
                # #438: the last-resort tap may place a via inside this edge pad
                # (fab-acceptable -- no closer to the edge than the pad already
                # is), but its connecting TRACKS must not run CLOSER to the board
                # edge than the pad. Cap the tap's edge clearance at the pad's own
                # edge distance instead of the blanket 0.0 that let a #373 track
                # fallback graze the outline (ulx3s GND on In2.Cu at 0.0mm).
                _pad_edge = 0.0
                _bb = getattr(pcb_data.board_info, 'board_bounds', None)
                if _bb:
                    _mnx, _mny, _mxx, _mxy = _bb
                    _pad_edge = max(0.0, min(pad.global_x - _mnx, _mxx - pad.global_x,
                                             pad.global_y - _mny, _mxy - pad.global_y)
                                    - max(pad.size_x, pad.size_y) / 2.0)
                # A plated barrel can launch from any copper layer, so run
                # the whole via ladder per candidate layer and take the
                # first that lands a via (#494). An SMD pad has exactly one
                # candidate, so this is the old single pass for it.
                result = None
                for _cand in pad_cands:
                    for vtry, dtry in via_pairs:
                        result = tap_pad_with_escalation(
                            pad, _cand, net_id, pcb_data,
                            replace(tap_config, via_size=vtry, via_drill=dtry,
                                    board_edge_clearance=_pad_edge),
                            max_search_radius=max_search_radius, via_size=vtry,
                            via_drill=dtry, verbose=verbose, fine_for_all=True,
                            distant_trace_radius=0.0, disable_reuse=True,
                            shared_via_maps=shared_maps,
                            plane_oracle=sweep_oracle,
                            corridor_ghosts=corridor_ghosts)
                        if result.success and result.via is not None:
                            if (vtry, dtry) in _escalated_pairs:
                                warn_fab_escalation(
                                    f"last-resort plane via for net "
                                    f"{net_id} ({vtry}/{dtry}mm)")
                            break
                    if result is not None and result.success \
                            and result.via is not None:
                        pad_layer = _cand
                        break
                if result.success and result.via is not None:
                    new_via_obj = Via(
                        x=result.via['x'], y=result.via['y'], size=result.via['size'],
                        drill=result.via['drill'], layers=['F.Cu', 'B.Cu'], net_id=net_id)
                    pcb_data.vias.append(new_via_obj)
                    new_seg_objs = []
                    for s in result.segments:
                        seg_obj = Segment(
                            start_x=s['start'][0], start_y=s['start'][1],
                            end_x=s['end'][0], end_y=s['end'][1],
                            width=s['width'], layer=s['layer'], net_id=s['net_id'])
                        new_seg_objs.append(seg_obj)
                        pcb_data.segments.append(seg_obj)
                    # VERIFY the forced via actually joins this pad to the plane
                    # (issue #287, neptune): a "successful" via can land in the
                    # gap between Voronoi cells -- DRC-clean, touching no fill --
                    # so re-run the fill-aware union-find for the pad before
                    # claiming success, and UNDO the via if it is still floating.
                    _v_segs = [s for s in pcb_data.segments if s.net_id == net_id]
                    _v_vias = [v for v in pcb_data.vias if v.net_id == net_id]
                    _v_res = check_net_connectivity(net_id, _v_segs, _v_vias, net_pads,
                                                    zones_by_net.get(net_id, []),
                                                    pcb_data=pcb_data)
                    # Custody by PROGRESS (#494) -- see
                    # pad_repair_made_progress.
                    _dp_after = _v_res.get('disconnected_pads')
                    if pad_repair_rejected(_cur_dp, _dp_after, pad,
                                           legacy=_no_plated):
                        pcb_data.vias.remove(new_via_obj)
                        for seg_obj in new_seg_objs:
                            pcb_data.segments.remove(seg_obj)
                        if name not in failed_repair_pads:
                            failed_repair_pads.append(name)
                            total_pads_repaired = max(0, total_pads_repaired - 1)
                        print(f"{RED}STILL FLOATING (forced via at "
                              f"({result.via['x']:.2f}, {result.via['y']:.2f}) reaches "
                              f"no plane copper - removed){RESET}")
                        continue
                    _cur_dp = _dp_after   # accepted: advance the baseline
                    all_new_vias.append(result.via)
                    total_vias += 1
                    for s in result.segments:
                        all_new_segments.append(s)
                    shared_maps.note_pass_copper([new_via_obj], new_seg_objs)
                    sweep_oracle.note_tap_committed(pad, [new_via_obj], new_seg_objs)
                    if name in failed_repair_pads:
                        failed_repair_pads.remove(name)
                        total_pads_repaired += 1
                    print(f"{GREEN}forced via at ({result.via['x']:.2f}, "
                          f"{result.via['y']:.2f}){RESET}")
                else:
                    # #373 last resort: no via could tie this pad to the plane
                    # (boxed-in pocket, fine-pitch WLCSP, deep-layer pour). Route
                    # a plain track from the pad to the nearest same-net copper /
                    # its own pour on the pad's layer -- the via-or-nothing ladder
                    # otherwise abandons a pad a short trace would connect. The
                    # island-join fill test validates the target; re-run the SAME
                    # fill-aware check and UNDO the track if still floating.
                    # Per candidate launch layer, same as the via ladder
                    # above (#494); one candidate for an SMD pad.
                    connected = False
                    for _cand in pad_cands:
                        track_res = tap_pad_with_escalation(
                            pad, _cand, net_id, pcb_data,
                            # #438/#441: the #373 last-resort track must not run CLOSER
                            # to the board edge than the pad it connects. The blanket
                            # tap_config board_edge_clearance=0.0 falls back to
                            # config.clearance in the edge keep-out, letting the fallback
                            # trace graze the outline sub-fab (ulx3s In2.Cu at 0.0). Cap
                            # it at the pad's own edge distance, as the via tap above does.
                            replace(tap_config, via_size=via_size, via_drill=via_drill,
                                    board_edge_clearance=_pad_edge),
                            max_search_radius=max_search_radius,
                            via_size=via_size, via_drill=via_drill,
                            verbose=verbose, fine_for_all=True, pour_trace_only=True,
                            distant_trace_radius=max_search_radius, disable_reuse=True,
                            plane_oracle=sweep_oracle,
                            corridor_ghosts=corridor_ghosts)
                        if not (track_res.success and track_res.segments):
                            continue
                        new_seg_objs = []
                        for s in track_res.segments:
                            seg_obj = Segment(
                                start_x=s['start'][0], start_y=s['start'][1],
                                end_x=s['end'][0], end_y=s['end'][1],
                                width=s['width'], layer=s['layer'], net_id=s['net_id'])
                            new_seg_objs.append(seg_obj)
                            pcb_data.segments.append(seg_obj)
                        _t_segs = [s for s in pcb_data.segments if s.net_id == net_id]
                        _t_vias = [v for v in pcb_data.vias if v.net_id == net_id]
                        _t_res = check_net_connectivity(net_id, _t_segs, _t_vias, net_pads,
                                                        zones_by_net.get(net_id, []),
                                                        pcb_data=pcb_data)
                        # Custody by PROGRESS (#494).
                        _dp_after = _t_res.get('disconnected_pads')
                        if pad_repair_rejected(_cur_dp, _dp_after, pad,
                                               legacy=_no_plated):
                            for seg_obj in new_seg_objs:
                                pcb_data.segments.remove(seg_obj)
                            continue
                        _cur_dp = _dp_after   # accepted: advance the baseline
                        connected = True
                        for s in track_res.segments:
                            all_new_segments.append(s)
                        shared_maps.note_pass_copper([], new_seg_objs)
                        sweep_oracle.note_tap_committed(pad, [], new_seg_objs)
                        if name in failed_repair_pads:
                            failed_repair_pads.remove(name)
                            total_pads_repaired += 1
                        print(f"{GREEN}connected by track to same-net copper "
                              f"on {_cand}{RESET}")
                        break
                    if not connected:
                        if name not in failed_repair_pads:
                            failed_repair_pads.append(name)
                            total_pads_repaired = max(0, total_pads_repaired - 1)
                        print(f"{RED}STILL FLOATING{RESET}")

    # Drop a redundant plane-repair tap that grazes a foreign pad below clearance,
    # or re-bend a load-bearing one around the pad (#224). A tap that merely bridges
    # two pads already tied into the pour is redundant -- dropping it clears the
    # graze (e.g. ddr5 GND tap grazing the LBDQ connector pad). Connectivity-gated
    # (WITH the pour), so a load-bearing tap is kept and re-bent instead. route.py's
    # reconnect excludes the plane nets, so they are only ever cleaned up here.
    if all_new_segments:
        if progress_callback:
            progress_callback(0, 0, "Cleaning up repair copper (graze prune/nudge)...")
        from pcb_modification import cleanup_plane_taps_grazing
        _scope = {s['net_id'] for s in all_new_segments}
        (all_new_segments, _gz_rm, _gz_nudge, _gz_swept,
         _gz_input_strips) = cleanup_plane_taps_grazing(
            pcb_data, all_new_segments, _scope, clearance=clearance,
            max_shift=config.grid_step / 2, all_new_vias=all_new_vias,
            hole_to_hole=config.hole_to_hole_clearance,
            protected_pads=_tapped_pads)
        if _gz_rm:
            print(f"  Graze prune: removed {_gz_rm} grazing repair segment(s)")
        if _gz_nudge:
            print(f"  Graze nudge: re-bent grazing tap jog(s) on {_gz_nudge} net(s)")
        if _gz_swept:
            print(f"  Dead-end sweep: trimmed {_gz_swept} orphaned repair segment(s)")
        # #508 finding 2: INPUT copper the passes deleted from pcb_data must
        # reach the writer/GUI strip channel or the output re-emits it.
        if _gz_input_strips:
            file_strip_segments.extend(_gz_input_strips)
            print(f"  Graze/sweep passes removed {len(_gz_input_strips)} "
                  f"input-board segment(s); forwarded to the strip channel")

    # Issue #293: re-verify the signal nets that were connected when we started.
    # Ripped nets are excluded (they are honestly reported + stripped for a
    # reconnect pass); anything ELSE this pass broke is a repair bug -- surface
    # it loudly so the pipeline reconnects it instead of shipping it silently.
    _regressed_293 = []
    if _pre_connected_293:
        _segs_now: Dict[int, list] = {}
        for _s in pcb_data.segments:
            _segs_now.setdefault(_s.net_id, []).append(_s)
        _vias_now: Dict[int, list] = {}
        for _v in pcb_data.vias:
            _vias_now.setdefault(_v.net_id, []).append(_v)
        for _nid in sorted(_pre_connected_293):
            if _nid in (ripped_net_ids or []) or \
               _nid in {pr[0] for pr in partial_restores}:
                continue  # ripped/partial nets are reported for reconnect separately
            _pads = pcb_data.pads_by_net.get(_nid, [])
            _r = _cnc293(_nid, _segs_now.get(_nid, []), _vias_now.get(_nid, []),
                         _pads, _zones_by_net_293.get(_nid, []))
            if not _r.get('connected'):
                _net = pcb_data.nets.get(_nid)
                _regressed_293.append(
                    (_net.name if _net else f"net_{_nid}",
                     len(_r.get('disconnected_pads') or [])))
        if _regressed_293:
            print(f"\n{RED}WARNING: this repair pass DISCONNECTED "
                  f"{len(_regressed_293)} previously-connected signal net(s) "
                  f"(issue #293) - re-route them before shipping:{RESET}")
            for _nm, _ndisc in _regressed_293:
                print(f"  {RED}{_nm}: {_ndisc} pad(s) now disconnected{RESET}")

    # Print summary
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"  Zones processed: {len(net_names)}")
    print(f"  Total routes added: {total_routes}")
    if total_vias > 0:
        print(f"  Total vias added: {total_vias}")
    if repair_pads:
        print(f"  Pad repair: {total_pads_repaired}/{total_pads_unconnected} unconnected pad(s) reconnected")
        if failed_repair_pads:
            # Run-6 fix: failed_repair_pads is a historical accumulator graded
            # by the KRT fill model, which under-credits narrow real fill
            # corridors (~0.05mm raster floor) -- it claimed pads unconnected
            # that KiCad's refilled pour connects (C16.2 class). Re-grade the
            # list against the exact/kicad oracle before printing; entries the
            # oracle proves connected are reported separately, not as opens.
            _confirmed, _cleared = list(failed_repair_pads), []
            try:
                if _gate_oracle_links[0] is None:
                    _gate_oracle_links[0] = _gate_oracle_query()
                _olinks = _gate_oracle_links[0]
                if _olinks is not False and _olinks is not None:
                    def _pad_open_per_oracle(entry):
                        # entry format: "REF.PAD (NET)"
                        try:
                            _refpad = entry.split(' ')[0]
                            _ref, _pn = _refpad.rsplit('.', 1)
                            _fp = pcb_data.footprints.get(_ref)
                            _pad = next(p for p in (_fp.pads if _fp else [])
                                        if p.pad_number == _pn)
                        except Exception:
                            return True     # unparseable: keep as open
                        for _net, _a, _b in _olinks:
                            for _e in (_a, _b):
                                if (abs(_e[0] - _pad.global_x) < 0.1
                                        and abs(_e[1] - _pad.global_y) < 0.1):
                                    return True
                        return False
                    _confirmed = [e for e in failed_repair_pads
                                  if _pad_open_per_oracle(e)]
                    _cleared = [e for e in failed_repair_pads
                                if e not in _confirmed]
            except Exception:
                pass    # incl. NameError when the gate closure never built
            if _cleared:
                print(f"  {len(_cleared)} reported-unconnected pad(s) are "
                      f"CONNECTED per KiCad's exact fill (model quantization): "
                      f"{', '.join(_cleared)}")
            if _confirmed:
                print(f"  Pads still unconnected: {', '.join(_confirmed)}")
            else:
                print("  Pads still unconnected: none (all model-only; "
                      "confirmed connected by the oracle)")
    if debug_lines and all_debug_lines:
        print(f"  Debug lines on User.4: {len(all_debug_lines)}")

    # Close soft joints in this run's copper (#334) -- see route_planes;
    # repair/reroute copper never passed through the cleanup pipeline.
    try:
        from pcb_modification import close_soft_joints
        _bridge_results: List[Dict] = []
        _nb = close_soft_joints(_bridge_results, pcb_data, None, config)
        if _nb:
            for _br in _bridge_results:
                for _bs in _br.get('new_segments', []):
                    all_new_segments.append({
                        'start': (_bs.start_x, _bs.start_y),
                        'end': (_bs.end_x, _bs.end_y),
                        'width': _bs.width, 'layer': _bs.layer,
                        'net_id': _bs.net_id})
            print(f"  Closed {_nb} soft joint(s) in repair copper")
    except Exception as _e:
        print(f"  (soft-joint close skipped: {_e})")

    kv10_names = pcb_data.net_id_to_name if pcb_data.kicad_version >= KICAD_10_MIN_VERSION else None

    # GUI (return_results): hand the plane/repair copper + the ripped net ids
    # back; the partial-restore kept pieces were emitted (and the in-memory
    # rip-casualty reconnect ran) before the final fill-aware verification
    # above, for both fronts. No file is written here.

    # Reuse same-net vias that violate hole-to-hole (a region join can place a via a
    # grid cell from an existing same-net one). After ALL vias are collected (incl.
    # partial-restore kept vias above) and before both the GUI-return and file-write
    # paths, so CLI and GUI emit the same merged set.
    from pcb_modification import merge_close_same_net_vias
    merge_close_same_net_vias(all_new_vias, all_new_segments, pcb_data,
                              config.hole_to_hole_clearance)

    # Route trace (#482): the rip-casualty reconnect + round-2 join copper added
    # since the plane-join capture; then write <output>_routetrace.json.
    if _ptrace is not None:
        _ptrace.capture(pcb_data, 'plane-reconnect')
        _ptrace.dump(output_file, pcb_data)

    _reconcile_write_list_vs_board('final')

    if return_results:
        # GUI/stress parity (gap closure): the CLI main runs post-passes on
        # its written file that the GUI path used to skip -- the shared
        # plane-copper cleanup runs here, in memory (the rip-casualty
        # self-reconnect now runs for BOTH fronts before the final fill-aware
        # verification above; the kicad-oracle recheck runs in the planes tab
        # after apply, where the LIVE board can be temp-saved).
        # Shared plane-copper cleanup, in memory (the CLI runs
        # clean_plane_copper on its written file). Removed emissions drop
        # from all_new_* in place; removed INPUT copper is returned in the
        # new strip channel for the GUI applier.
        _strip_segments = []
        try:
            from types import SimpleNamespace
            from cleanup_pipeline import run_post_route_cleanup
            _scope = set()
            for _nname in net_names:
                for _nid, _net in pcb_data.nets.items():
                    if _net.name == _nname:
                        _scope.add(_nid)
            # The emissions here are DICTS for the GUI applier, but this
            # run's copper also lives in pcb_data as real objects (the
            # engine appends as it routes) -- so run the pipeline against
            # pcb_data with an empty write-list: every removal comes back
            # as an input strip, and additions come back as cleanup result
            # entries. Strips matching an emission dict drop that dict
            # (the applier adds AFTER it deletes, so a strip of not-yet-
            # added copper would no-op and the deleted copper would ship);
            # the rest go to the GUI strip channel.
            _res_wrap = []
            _out = run_post_route_cleanup(
                _res_wrap, pcb_data, _scope,
                SimpleNamespace(clearance=clearance, grid_step=grid_step),
                label='Plane ', phantom=False, via_nudge=False, neck=False,
                microshift_max_shift=grid_step)
            for _r in _res_wrap:
                for _s in (_r.get('new_segments') or []):
                    all_new_segments.append(
                        {'start': (_s.start_x, _s.start_y),
                         'end': (_s.end_x, _s.end_y),
                         'width': _s.width, 'layer': _s.layer,
                         'net_id': _s.net_id})
                for _v in (_r.get('new_vias') or []):
                    all_new_vias.append(
                        {'x': _v.x, 'y': _v.y, 'size': _v.size,
                         'drill': _v.drill, 'layers': _v.layers,
                         'net_id': _v.net_id})

            def _skey(_s):
                return (round(_s.start_x, 3), round(_s.start_y, 3),
                        round(_s.end_x, 3), round(_s.end_y, 3), _s.net_id)

            def _dkey(_d):
                return (round(_d['start'][0], 3), round(_d['start'][1], 3),
                        round(_d['end'][0], 3), round(_d['end'][1], 3),
                        _d['net_id'])
            _stripped = {}
            for _s in _out.input_strip_segments:
                _stripped[_skey(_s)] = _s
                _stripped[(_skey(_s)[2], _skey(_s)[3], _skey(_s)[0],
                           _skey(_s)[1], _skey(_s)[4])] = _s
            _kept_dicts = []
            for _d in all_new_segments:
                _hit = _stripped.pop(_dkey(_d), None)
                if _hit is None:
                    _kept_dicts.append(_d)
            all_new_segments[:] = _kept_dicts
            _strip_segments = list(
                {id(_s): _s for _s in _stripped.values()}.values())
            # #508 finding 16: mirror the segment reconcile for VIAS. A
            # stripped via matching an emission dict must DROP that dict
            # (the applier deletes before it adds, so the strip would no-op
            # and the withdrawn via would ship anyway); only strips of
            # genuine input-board vias go to the GUI strip channel.
            _vstripped = {}
            for _v in (getattr(_out, 'input_strip_vias', []) or []):
                _vstripped[(round(_v.x, 3), round(_v.y, 3), _v.net_id)] = _v
            _kept_vdicts = []
            for _d in all_new_vias:
                _hit = _vstripped.pop(
                    (round(_d['x'], 3), round(_d['y'], 3), _d['net_id']), None)
                if _hit is None:
                    _kept_vdicts.append(_d)
            all_new_vias[:] = _kept_vdicts
            _strip_segments += list(_vstripped.values())
            _strip_segments += file_strip_segments + file_strip_vias
        except Exception as _e:
            print(f"{RED}  in-memory plane cleanup failed: {_e}{RESET}")
        # The GUI deletes every returned net's old board copper before adding
        # all_new_*; partial nets' kept pieces ride the emissions, so include
        # them in the deletion set (strip-and-replace parity).
        if progress_callback:
            progress_callback(1, 1, "Plane repair complete")
        return (total_routes, total_regions, all_new_vias, all_new_segments,
                ripped_net_ids + partial_ids + inplace_reconnected_ids,
                _strip_segments)

    if dry_run:
        print("\nDry run - no output file written")
    elif total_routes > 0 or total_pads_repaired > 0 or ripped_net_ids \
            or partial_ids or inplace_reconnected_ids:
        print(f"\nWriting output to {output_file}...")
        # Strip the ripped signal nets' copper from the output - they are left
        # unrouted for a subsequent route.py pass to reconnect (#141 reverted).
        # Partially-restored nets are stripped too; their kept pieces are in
        # all_new_segments/all_new_vias (replacement).
        _write_output(input_file, output_file, all_new_segments, all_new_vias, all_debug_lines,
                      net_id_to_name=kv10_names,
                      exclude_net_ids=(ripped_net_ids + partial_ids
                                       + inplace_reconnected_ids),
                      removed_segments=file_strip_segments,
                      removed_vias=file_strip_vias,
                      add_teardrops=add_teardrops)
        print(f"Output written to {output_file}")
        print("Note: Open in KiCad and press 'B' to refill zones")

        # Board-vs-file ledger (KICAD_BOARD_LEDGER=1, #508): the written file
        # must match pcb_data for every net this run touched -- this engine
        # had NO ledger call and both #463 and the #508 findings sat on
        # unledgered paths. No-op unless the env var is set.
        from cleanup_pipeline import verify_written_file_parity
        _ledger_scope = sorted(set(net_ids)
                               | set(ripped_net_ids) | set(partial_ids)
                               | {d['net_id'] for d in all_new_segments}
                               | {d['net_id'] for d in all_new_vias})
        verify_written_file_parity(output_file, pcb_data, _ledger_scope,
                                   label=' planes-repair')
    else:
        print("\nNo routes added - copying input to output unchanged")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)

    if progress_callback:
        progress_callback(1, 1, "Plane repair complete")
    return (total_routes, total_regions)


def _write_output(input_file: str, output_file: str, segments: List[Dict], vias: List[Dict] = None,
                  debug_lines: List[str] = None, net_id_to_name: Dict = None,
                  exclude_net_ids: List[int] = None,
                  removed_segments: List = None, removed_vias: List = None,
                  add_teardrops: bool = False):
    """Write the output PCB file with new segments, vias, and optional debug lines.

    exclude_net_ids: nets whose existing copper is stripped from the output (used
    for nets ripped to clear a blocked pad repair, which are re-routed separately).
    removed_segments/removed_vias: SPECIFIC input-board copper to delete (#484
    structural root: an in-memory pass can remove a NON-excluded net's input
    copper from pcb_data, and without a per-segment strip channel the writer
    re-emits it from the input text -- board != file).
    """
    # Seed the output's sibling .kicad_pro before the board exists (#513 item
    # 12): peaksat_obc_adcs hit the harness timeout after this board write but
    # before fix_project_for_output, and the next step silently fell back to
    # stock netclass floors. Also gives intermediate tmp boards their floor.
    from fix_kicad_drc_settings import seed_project_for_output
    seed_project_for_output(output_file, input_file)
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    if exclude_net_ids:
        from plane_io import filter_nets_from_content
        names = ([net_id_to_name[n] for n in exclude_net_ids if net_id_to_name and n in net_id_to_name]
                 or None)
        content = filter_nets_from_content(content, exclude_net_ids, names)

    if removed_segments:
        from kicad_writer import remove_segments_from_content
        content, _nrs = remove_segments_from_content(
            content, removed_segments, net_id_to_name=net_id_to_name)
        if _nrs:
            print(f"  Stripped {_nrs} in-memory-removed input segment(s) from the output")
    if removed_vias:
        from kicad_writer import remove_vias_from_content
        content, _nrv = remove_vias_from_content(
            content, removed_vias, net_id_to_name=net_id_to_name)
        if _nrv:
            print(f"  Stripped {_nrv} in-memory-removed input via(s) from the output")

    # Generate segment S-expressions
    segment_sexprs = []
    for seg in segments:
        seg_net_name = net_id_to_name.get(seg['net_id']) if net_id_to_name else None
        sexpr = generate_segment_sexpr(
            start=seg['start'],
            end=seg['end'],
            width=seg['width'],
            layer=seg['layer'],
            net_id=seg['net_id'],
            net_name=seg_net_name
        )
        segment_sexprs.append(sexpr)

    # Generate via S-expressions
    via_sexprs = []
    if vias:
        # Repair vias follow the board's own via protection convention instead
        # of a hardcoded front+back tenting (#489 §8).
        from kicad_writer import prevailing_via_protection_in_text
        _default_via_attrs = prevailing_via_protection_in_text(content)
        for via in vias:
            via_net_name = net_id_to_name.get(via['net_id']) if net_id_to_name else None
            sexpr = generate_via_sexpr(
                x=via['x'],
                y=via['y'],
                size=via['size'],
                drill=via['drill'],
                layers=['F.Cu', 'B.Cu'],  # Through-hole vias
                net_id=via['net_id'],
                net_name=via_net_name,
                tenting_attrs=via.get('tenting_attrs') or _default_via_attrs
            )
            via_sexprs.append(sexpr)

    routing_text = '\n'.join(segment_sexprs + via_sexprs)

    # Add debug lines if provided
    if debug_lines:
        routing_text += '\n' + '\n'.join(debug_lines)

    # Teardrops on pads (#489 §9): this step lays tap traces INTO pads, which is
    # exactly the trace-to-pad junction a teardrop is for, and it had no way to
    # ask for one.
    if add_teardrops:
        from kicad_writer import add_teardrops_to_pads
        print("Adding teardrop settings to pads...")
        content, _td = add_teardrops_to_pads(content)
        print(f"  Added teardrops to {_td} pads" if _td
              else "  All pads already have teardrop settings")

    # Insert before final closing paren
    last_paren = content.rfind(')')
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    # Vias LAST, so the repair vias this run just placed get teardrops too (the
    # same ordering rule as output_writer / plane_io).
    if add_teardrops:
        from kicad_writer import add_teardrops_to_vias
        new_content, _vtd = add_teardrops_to_vias(new_content)
        print(f"  Added teardrops to {_vtd} vias" if _vtd
              else "  No vias needed teardrops (none present, or all already set)")

    # #523 backstop: never write a board whose s-expression structure broke
    # in one of the text transforms above -- a truncated file loads in our
    # text parsers but not in KiCad, so it would ship invisibly.
    from kicad_writer import assert_balanced_sexpr
    assert_balanced_sexpr(new_content, label=output_file)
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)


def main():
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set
    parser = argparse.ArgumentParser(
        description="Route between disconnected regions in power plane zones",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Auto-detect all zones in PCB:
    python py_router/repair_planes.py input.kicad_pcb output.kicad_pcb

    # Only process specific layers (all nets on those layers):
    python py_router/repair_planes.py input.kicad_pcb output.kicad_pcb \\
        --plane-layers B.Cu In1.Cu

    # Only process specific nets (on all layers they have zones):
    python py_router/repair_planes.py input.kicad_pcb output.kicad_pcb \\
        --nets GND +3.3V

    # Specific net/layer pairs (counts must match):
    python py_router/repair_planes.py input.kicad_pcb output.kicad_pcb \\
        --nets GND +3.3V --plane-layers B.Cu In1.Cu \\
        --max-track-width 1.0
"""
    )

    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?", help="Output KiCad PCB file (default: input_routed.kicad_pcb)")
    # #381 D9: accept --output FILE like route.py / route_diff.py.
    parser.add_argument("--output", metavar="FILE",
                        help="Output KiCad PCB file (flag alternative to the positional output)")
    parser.add_argument("--overwrite", "-O", action="store_true",
                        help="Overwrite input file instead of creating _routed copy")

    # Net and layer specification (now optional)
    parser.add_argument("--nets", "-n", nargs="+",
                        help="Net name(s) to process. If omitted, all nets with zones are processed.")
    parser.add_argument("--plane-layers", "-p", nargs="+",
                        help="Plane layer(s) to process. If omitted, all layers with zones are processed.")
    parser.add_argument("--layers", "-l", nargs="+",
                        help="Layer(s) available for routing (e.g., F.Cu B.Cu). If omitted, all copper layers are used.")

    # Track width options
    parser.add_argument("--max-track-width", type=float, default=defaults.REPAIR_MAX_TRACK_WIDTH,
                        help="Maximum track width for connections in mm (default: 2.0)")
    parser.add_argument("--min-track-width", type=float, default=defaults.REPAIR_MIN_TRACK_WIDTH,
                        help="Minimum track width for connections in mm (default: 0.2)")
    parser.add_argument("--track-width", type=float, default=None,
                        help="Default track width for routing config in mm (default: the board Default net-class width, else 0.3)")

    # Clearance options
    parser.add_argument("--clearance", type=float, default=None,
                        help="Trace-to-trace clearance CEILING in mm. When given, every net class (Default included) is capped at min(class, this) and the writeback clamps. When OMITTED, each net routes at its own net-class clearance (base = the board's Default class, else 0.25).")
    parser.add_argument("--zone-clearance", type=float, default=defaults.PLANE_ZONE_CLEARANCE,
                        help="Zone fill clearance around obstacles in mm (default: 0.2)")
    # #381 D9: accept route_planes.py's --plane-track-via-clearance spelling too
    # (same constant; dest stays track_via_clearance).
    parser.add_argument("--track-via-clearance", "--plane-track-via-clearance",
                        type=float, default=defaults.PLANE_TRACK_VIA_CLEARANCE,
                        help="Clearance from tracks to other nets' vias in mm (default: 0.8)")
    parser.add_argument("--board-edge-clearance", type=float, default=None,
                        help=f"Clearance from board edge in mm (default: the board min_copper_edge_clearance, else {defaults.PLANE_EDGE_CLEARANCE})")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=None,
                        help=f"Minimum clearance between drill holes in mm (default: the board min_hole_to_hole, else {defaults.HOLE_TO_HOLE_CLEARANCE})")

    # Via options (for config)
    parser.add_argument("--via-size", type=float, default=None,
                        help="Via outer diameter in mm (default: the board Default net-class via, else 0.5)")
    parser.add_argument("--via-drill", type=float, default=None,
                        help="Via drill diameter in mm (default: the board Default net-class via drill, else 0.3)")

    # Grid step
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                        help="Routing grid step in mm (default: 0.1)")
    parser.add_argument("--ripup-blocker-select",
                        choices=list(defaults.RIPUP_BLOCKER_SELECT_CHOICES),
                        default=defaults.RIPUP_BLOCKER_SELECT,
                        help="""Blocker SELECTION algorithm for the rip-up ladder (see route.py --help / docs/rip-up-reroute.md)""")
    parser.add_argument("--analysis-grid-step", type=float, default=defaults.REPAIR_ANALYSIS_GRID_STEP,
                        help="Grid step for connectivity analysis in mm (coarser = faster, default: 0.5)")

    # Routing options
    parser.add_argument("--no-kicad-recheck", action="store_true",
                        help="Skip the kicad-cli-verified reconnect pass on the output "
                             "(runs by default when kicad-cli is installed)")
    parser.add_argument("--max-iterations", type=int, default=defaults.MAX_ITERATIONS,
                        help="Maximum A* iterations per route attempt (default: 200000)")

    # Pad-level repair (issue #99)
    parser.add_argument("--repair-pads", dest="repair_pads", action="store_true", default=True,
                        help="Repair pad-level plane connection failures: retry a stitching "
                             "via + trace for plane-net pads with no connection to the plane, "
                             "escalating to fine parameters for fine-pitch pads (default: on)")
    parser.add_argument("--no-repair-pads", dest="repair_pads", action="store_false",
                        help="Disable the pad-level repair pass (only reconnect zone islands)")
    parser.add_argument("--max-search-radius", type=float, default=defaults.PLANE_MAX_SEARCH_RADIUS,
                        help=f"Max radius to search for a via position during pad repair in mm "
                             f"(default: {defaults.PLANE_MAX_SEARCH_RADIUS})")

    # Rip-blocker repair (mirror of route_planes): connect a plane-net pad that
    # cannot get its own via by tracing to an adjacent same-net pad, ripping the
    # signal net(s) blocking that trace, then re-routing them with the original
    # signal parameters - which must therefore be passed through.
    parser.add_argument("--rip-blocker-nets", action="store_true",
                        help="When a plane-net pad cannot be connected, trace to a nearby same-net "
                             "pad, ripping the signal net(s) blocking it, then re-route the ripped nets.")
    parser.add_argument("--max-rip-nets", type=int, default=defaults.PLANE_MAX_RIP_NETS,
                        help="Maximum number of blocker nets to rip per pad (default: 3)")
    parser.add_argument("--reroute-ripped-nets", action="store_true",
                        help="DEPRECATED / no-op (issue #141 reverted): ripped blocker nets are now "
                             "reconnected in-run (restore-first, an end-of-run reconnect pass, and "
                             "custody restore on failure); no separate route.py pass is needed. "
                             "Accepted for compatibility.")
    parser.add_argument("--power-nets", nargs="+", default=None,
                        help="Power net names that need wider tracks when re-routing ripped nets.")
    parser.add_argument("--power-nets-widths", nargs="+", type=float, default=None,
                        help="Track width (mm) per --power-nets entry, used when re-routing ripped nets.")
    # #381 D9: accept the plural --no-bga-zones spelling too (route.py uses it).
    parser.add_argument("--no-bga-zone", "--no-bga-zones", action="store_true",
                        help="Disable BGA auto-exclusion zones when re-routing ripped nets "
                             "(match the original signal run's --no-bga-zone).")

    # #489 §9: this step lays tap traces INTO pads and places repair vias, so it
    # needs the same teardrop switch route.py / route_diff.py / route_planes.py
    # have. GUI: the existing "Add teardrops" checkbox feeds it.
    parser.add_argument("--add-teardrops", action="store_true",
                        help="Add teardrop settings to all pads and vias in output file")

    # Debug options
    parser.add_argument("--dry-run", action="store_true",
                        help="Analyze without writing output")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed debug messages")
    parser.add_argument("--debug-lines", action="store_true",
                        help="Add debug lines on User.4 layer showing route paths")
    from fix_kicad_drc_settings import add_drc_fix_args
    add_drc_fix_args(parser)

    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    args = parser.parse_args()
    # #439: identical net-class/clearance model to route.py. --clearance is the
    # clamp switch: GIVEN -> ceiling, every class capped at min(class, ceiling),
    # writeback clamps (_clamp_netclasses True). OMITTED -> each net routes at its
    # own class (base = board Default class), classes preserved. Geometry flags
    # omitted -> default from the board (track/clearance/via from Default net-class,
    # hole/edge from board constraints, else a routing_defaults constant). Planes
    # keep their larger PLANE_EDGE_CLEARANCE fallback when the board declares none.
    # Resolved here, before enforce_fab_floors and every downstream use.
    from list_nets import (board_default_netclass_clearance, board_default_netclass_param,
                           resolve_cli_floor)
    for _pname, _nckey, _fallback in (('track_width', 'track_width', defaults.TRACK_WIDTH),
                                      ('via_size', 'via_diameter', defaults.VIA_SIZE),
                                      ('via_drill', 'via_drill', defaults.VIA_DRILL)):
        if getattr(args, _pname) is None:
            _v = board_default_netclass_param(args.input_file, _nckey)
            setattr(args, _pname, _v if _v is not None else _fallback)
            print(f"--{_pname.replace('_', '-')} not given; using "
                  f"{'the board Default net-class' if _v is not None else 'the fallback'} "
                  f"{getattr(args, _pname)}mm.")
    _ceiling = args.clearance                       # None iff --clearance omitted
    args._clamp_netclasses = _ceiling is not None
    args._clearance_ceiling = _ceiling
    from fix_kicad_drc_settings import warn_if_missing_project_floor
    warn_if_missing_project_floor(args.input_file)  # #441: a dropped sibling .kicad_pro strands the DRC floor
    _dflt_clr = board_default_netclass_clearance(args.input_file)
    if _ceiling is None:
        args.clearance = _dflt_clr if _dflt_clr is not None else defaults.CLEARANCE
        print(f"--clearance not given; honoring net classes with base = "
              f"{'the board Default net-class' if _dflt_clr is not None else 'the fallback'} "
              f"clearance {args.clearance}mm.")
    else:
        args.clearance = min(_dflt_clr, _ceiling) if _dflt_clr is not None else _ceiling
    # Shared resolver (list_nets.resolve_cli_floor); see route_planes.py -- a
    # DECLARED 0.0 is "no edge rule of its own", not a rule of zero, so the
    # plane inset stays PLANE_EDGE_CLEARANCE as the GUI's plane tab already had
    # it.
    args.hole_to_hole_clearance = resolve_cli_floor(
        args.input_file, 'hole_to_hole', args.hole_to_hole_clearance,
        defaults.HOLE_TO_HOLE_CLEARANCE, '--hole-to-hole-clearance')
    args.board_edge_clearance = resolve_cli_floor(
        args.input_file, 'board_edge_clearance', args.board_edge_clearance,
        defaults.PLANE_EDGE_CLEARANCE, '--board-edge-clearance')
    set_default_fab_tier(*fab_tier_from_args(args))
    _pinned_floors = enforce_fab_floors(
        count_copper_layers_in_file(args.input_file),
        track_width=getattr(args, 'track_width', None),
        # The strap NECK floor must also be manufacturable (#513 item 9).
        min_track_width=getattr(args, 'min_track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None),
        board_edge_clearance=getattr(args, 'board_edge_clearance', None))
    # Below-floor params are pinned up to the fab floor (warned); apply the clamps.
    for _pname, _pfloor in _pinned_floors.items():
        setattr(args, _pname, _pfloor)

    # #381 D9: --output FILE overrides the positional (matches route.py/route_diff).
    if getattr(args, 'output', None) is not None:
        if args.output_file is not None and args.output_file != args.output:
            parser.error("both a positional output and --output were given and differ")
        args.output_file = args.output
    # Handle output file: use --overwrite, explicit output, or auto-generate with _routed suffix
    if args.output_file is None:
        if args.overwrite:
            args.output_file = args.input_file
        else:
            # Auto-generate output filename: input.kicad_pcb -> input_routed.kicad_pcb
            base, ext = os.path.splitext(args.input_file)
            args.output_file = base + '_routed' + ext
            print(f"Output file: {args.output_file}")

    # Auto-detect zones if nets/layers not fully specified
    if args.nets and args.plane_layers:
        # Both specified - must match in count
        if len(args.nets) != len(args.plane_layers):
            print(f"Error: When both --nets and --plane-layers are specified, counts must match")
            print(f"  Got {len(args.nets)} net(s) and {len(args.plane_layers)} layer(s)")
            sys.exit(1)
        net_names = args.nets
        plane_layers = args.plane_layers
    else:
        # Auto-detect from PCB zones
        print(f"Auto-detecting zones from {args.input_file}...")
        zone_pairs = auto_detect_zones(
            args.input_file,
            filter_nets=args.nets,
            filter_layers=args.plane_layers
        )

        if not zone_pairs:
            if args.nets or args.plane_layers:
                print("No zones found matching the specified filters")
            else:
                print("No zones found in PCB file")
            sys.exit(1)

        net_names = [pair[0] for pair in zone_pairs]
        plane_layers = [pair[1] for pair in zone_pairs]

        print(f"Found {len(zone_pairs)} zone(s) to process:")
        for net, layer in zone_pairs:
            print(f"  {net} on {layer}")

    _rdp_result = repair_planes(
        input_file=args.input_file,
        output_file=args.output_file,

        net_names=net_names,
        plane_layers=plane_layers,
        track_width=args.track_width,
        clearance=args.clearance,
        zone_clearance=args.zone_clearance,
        grid_step=args.grid_step,
        analysis_grid_step=args.analysis_grid_step,
        ripup_blocker_select=args.ripup_blocker_select,
        max_track_width=args.max_track_width,
        min_track_width=args.min_track_width,
        track_via_clearance=args.track_via_clearance,
        hole_to_hole_clearance=args.hole_to_hole_clearance,
        board_edge_clearance=args.board_edge_clearance,
        via_size=args.via_size,
        via_drill=args.via_drill,
        max_iterations=args.max_iterations,
        verbose=args.verbose,
        dry_run=args.dry_run,
        debug_lines=args.debug_lines,
        routing_layers=args.layers,
        repair_pads=args.repair_pads,
        max_search_radius=args.max_search_radius,
        rip_blocker_nets=args.rip_blocker_nets,
        max_rip_nets=args.max_rip_nets,
        reroute_ripped_nets=args.reroute_ripped_nets,
        power_nets=args.power_nets,
        power_nets_widths=args.power_nets_widths,
        no_bga_zone=args.no_bga_zone,
        clamp_netclasses=args._clamp_netclasses,
        clearance_ceiling=args._clearance_ceiling,
        add_teardrops=args.add_teardrops
    )

    # Dead-end sweep + gap-snap on the repaired plane copper (issue #84), gated
    # against connectivity + pours so it never breaks a net.
    if not args.dry_run:
        from pcb_modification import clean_plane_copper
        _snapped, _removed = clean_plane_copper(args.output_file, net_names,
                                                args.clearance, args.grid_step)
        if _snapped or _removed:
            print(f"Plane cleanup: closed {_snapped} stub gap(s), trimmed {_removed} dead-end segment(s)")
        # Castellated landings (run-6 fix 1.7): reconnect/tap copper that
        # landed in a castellated pad's edge-clearance zone is pulled back to
        # the pad's inner reach.
        try:
            from fix_kicad_drc_settings import effective_board_edge_clearance
            from pcb_modification import retract_castellated_landings
            _edge = effective_board_edge_clearance(args.input_file, 0.0)
            if _edge > 0:
                retract_castellated_landings(args.output_file, _edge)
        except Exception as e:
            print(f"  (skipped castellated-landing retract: {e})")

    # KiCad-oracle recheck (#217): our fill model over-credits, so gaps
    # KiCad's REAL fill produces can survive every model-based pass (castor
    # +3.3VA bare island, lumenpnp U5 pocket). Ask kicad-cli for the exact
    # missing links on the processed nets and route precisely those.
    if not args.dry_run and not args.no_kicad_recheck and args.output_file:
        from kicad_oracle import oracle_reconnect
        from routing_config import GridRouteConfig
        # #338: the oracle pass runs on OUTPUT_FILE, whose sibling .kicad_pro
        # is written only below (fix_project_for_output) -- so oracle_reconnect's
        # own project read finds nothing mid-chain. Resolve the board edge rule
        # from the ORIGINAL input's project here (the plane-zone inset
        # args.board_edge_clearance is NOT an enforced routing floor; see the
        # ripped-net reconnect above).
        try:
            from fix_kicad_drc_settings import effective_board_edge_clearance
            # #441: the oracle must validate at the fab-floor-pinned edge, not the
            # board's raw (possibly sub-fab / 0) rule, so it agrees with the router
            # and grader. cli=0 -> read project rule, floor at fab edge minimum.
            _oracle_edge = effective_board_edge_clearance(args.input_file, 0.0)
        except Exception:
            _oracle_edge = 0.0
        _ocfg = GridRouteConfig(
            clearance=args.clearance, track_width=args.track_width,
            via_size=args.via_size, via_drill=args.via_drill,
            grid_step=args.grid_step,
            board_edge_clearance=_oracle_edge)
        from kicad_dru import install_layer_clearances
        install_layer_clearances(_ocfg, None, args.input_file, None)  # #498
        _orc = oracle_reconnect(args.output_file, net_names, _ocfg,
                                track_via_clearance=args.track_via_clearance,
                                hole_to_hole_clearance=args.hole_to_hole_clearance,
                                verbose=args.verbose,
                                project_from=args.input_file)
        try:
            import json as _json
            print('JSON_ORACLE: ' + _json.dumps(
                {k: v for k, v in _orc.items()
                 if k not in ('new_segments', 'new_vias')}))
        except Exception:
            pass
        if not _orc.get('available'):
            print('NOTE: kicad-cli not found -- the oracle reconnect pass '
                  'did not run; output may differ from machines that have '
                  'KiCad installed (replay-determinism caveat).')

    # Make the output project's DRC design rules consistent with the floors we
    # just routed to (issue #160), mirroring route_planes.py, so a manual DRC in
    # KiCad flags only genuine problems instead of stock-default noise.
    if not args.no_fix_drc_settings and not args.dry_run \
            and args.output_file and os.path.isfile(args.output_file):
        try:
            import clearance_ledger
            eff_clearance = clearance_ledger.effective(args.clearance)
            if eff_clearance < args.clearance:
                print(f"  Min clearance used: {eff_clearance:.4g} mm "
                      f"(below nominal {args.clearance:.4g}; fine-pitch taps) - "
                      f"grading at this floor")
            from fix_kicad_drc_settings import (fix_project_for_output, drc_fix_kwargs,
                                                read_project_edge_clearance)
            # #338: record the PROJECT's edge rule, not the plane-zone inset
            # (see route_planes.py -- openstint 0.3-design/0.5-recorded).
            fix_project_for_output(
                args.output_file, input_pcb=args.input_file,
                clearance=eff_clearance, hole_to_hole=args.hole_to_hole_clearance,
                edge_clearance=read_project_edge_clearance(args.input_file),
                track_width=args.track_width,
                via_diameter=args.via_size, via_drill=args.via_drill,
                **drc_fix_kwargs(args))
        except Exception as e:
            print(f"  (skipped DRC-settings fix: {e})")

    # Machine-readable summary (mirrors route.py/route_diff.py) so an orchestrator
    # and the next pipeline step can read the clearance this step actually used.
    import json as _json, clearance_ledger as _cl
    _routes, _regions = (_rdp_result if isinstance(_rdp_result, tuple)
                         and len(_rdp_result) >= 2 else (0, 0))
    # Informational only: the zone nets this step actually processed (never
    # args.power_nets -- those are track-width hints, not planes; #479).
    _plane_nets = sorted(set(net_names))
    _summary = {
        "total_routes": _routes,
        "total_regions": _regions,
        "min_clearance_used": _cl.effective(args.clearance),
        "plane_nets": _plane_nets,
    }
    if LAST_RIPPED_RECONNECT is not None:
        _summary["ripped_reconnect"] = LAST_RIPPED_RECONNECT
    if LAST_RIPPED_CUSTODY is not None:
        _summary["ripped_custody"] = LAST_RIPPED_CUSTODY
    # A10: nets this step ripped and could neither reconnect nor restore. They
    # ship OPEN, so they belong in the summary by name and in the exit code.
    _summary["ripped_still_open"] = list(LAST_RIPPED_STILL_OPEN)
    print("JSON_SUMMARY: " + _json.dumps(_summary))

    if LAST_RIPPED_STILL_OPEN:
        print(f"{RED}RIP CASUALTIES SHIP OPEN ({len(LAST_RIPPED_STILL_OPEN)}): "
              f"{', '.join(LAST_RIPPED_STILL_OPEN)}{RESET}")
        print("  These nets were ripped to clear a plane corridor and are not "
              "connected on the written board. Reconnect them (the chain's "
              "reconnect step) or re-run without --rip-blocker-nets; do not "
              "read this run as clean.")
        return 4
    return 0


# Naming (#562): this module's engine used to be called route_planes,
# colliding with route_planes.py (the pours-CREATION script) -- the GUI
# already imported it 'as repair_planes'. The old name stays as an alias
# for external callers; new code should import repair_planes. Defined BEFORE
# the __main__ block: run-8 made that block sys.exit(), which would otherwise
# never reach a trailing assignment.
route_planes = repair_planes


if __name__ == "__main__":
    from console_encoding import enable_utf8_console
    enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
    # run-8: propagate main()'s verdict -- it returns 4 when plane repair
    # leaves rip casualties unconnected, which used to be one red line and
    # exit 0.
    sys.exit(main())
