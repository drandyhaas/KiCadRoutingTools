"""
QFN/QFP Fanout Strategy - Creates escape routing for QFN/QFP packages.

Generic module that analyzes actual pad geometry to determine:
- Which side each pad is on (based on position and pad orientation)
- Escape direction (perpendicular to pad's long axis)
- Stub length (based on chip size)
- Fan-out pattern (endpoints maximally separated)

Works with any QFN/QFP package regardless of pin count or size.
"""
from __future__ import annotations

import env_knobs
import math
from typing import List, Dict, Tuple, Optional
from collections import defaultdict

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb, Footprint, PCBData, find_components_by_type
from kicad_writer import add_tracks_and_vias_to_pcb
from qfn_fanout.types import QFNLayout, PadInfo, FanoutStub
from bga_fanout.constants import POSITION_TOLERANCE
from bga_fanout import (single_pad_net_ids, net_filter_misses,
                        filter_miss_lines)
from net_queries import matches_net_filter, literal_net_patterns, board_net_names
from qfn_fanout.layout import analyze_qfn_layout, analyze_pad, single_line_reason
from qfn_fanout.geometry import calculate_fanout_stub

# #621: nets whose escape was never ATTEMPTED because this run's own
# `cancel_check` stopped it -- in practice the GUI's Cancel button or the plan
# executor's Stop, the only cancel sources there are (the CLI passes None).
# Refreshed by every generate_qfn_fanout call and EMPTY unless a cancel
# actually fired.
#
# Deliberately a separate ledger from failed_nets/unescaped_nets: an unfinished
# search has measured nothing about a pad, and folding untried pads into the
# failure list reports a cancel as a routing defect -- which would send the
# planner (or the user) into a pointless tighter-clearance retry.
LAST_CANCEL_SKIPPED: List[str] = []

# #619: what the last under-pad escape's obstacle map ERASED, published so a
# sweep or a test can grade against the set the ENGINE used instead of
# re-deriving it. Re-deriving is a live trap: `fanned_nets` comes from
# `pad_infos`, which drops net 0, `unconnected-*`, net-filter misses and
# `center` pads, so the obvious proxy -- `{p.net_id for p in footprint.pads}`
# -- can report the gate as live on a footprint where the erased set is
# empty and the gate is a constant True. Keys: `nets` (the net-id set handed
# to nets_to_route), `vias`/`segs` (erased counts), `layer` (where the stub
# copper lands), `clearance` (the floor the stub was graded at).
LAST_ERASED_SETS: Dict = {}

# #846: what the last under-pad escape's COMMIT LOOP decided, published for the
# same reason as LAST_ERASED_SETS -- a sweep or a test grades against what the
# engine did rather than re-deriving it. `via_in_pad` is the FAB question (the
# barrel overlaps same-net pad copper, `fab_notes.via_overlaps_pad`), which is
# what the IPC-4761 note counts and what #202's clamp is for; it is NOT the
# 0.001mm centre coincidence this loop used to decide it by. Keys: `via_in_pad`,
# `via_in_pad_offcentre` (the subset #846 was about, which used to ship
# unclamped), `clamped`, `max_stub_mm`, `allow_via_in_pad`. #1804, per PAD:
# `pads_tried`, `pads_reused` (bridged to an existing same-net via, no new
# drill), `vias_placed`, `dropped_pads` [(pad number, net)], and
# `already_served_pads` [(pad number, net)] -- left out before the escape ran
# because each already has its via (`pad_has_via_escape`). Refreshed by every
# generate_qfn_fanout call; EMPTY when the run via-dropped nothing.
LAST_UNDERPAD_REPORT: Dict = {}

# #1834: the surface stub fan's per-PAD ledger, as LAST_UNDERPAD_REPORT is
# the via-drop's: `pads_tried` (stubs planned), `dropped_pads` [(pad number,
# net)] (the straight escape itself could not clear), and
# `already_served_pads` [(pad number, net)] -- left out before the fan was
# planned because each already has its surface escape (`pad_has_stub_escape`).
# Refreshed by every generate_qfn_fanout call; EMPTY when no surface fan ran.
LAST_STUB_REPORT: Dict = {}

# #1328: the surface-fan stubs the last run escaped BELOW its clearance, one
# dict per stub -- `net`, `pad` ("U1.5") and `clearance` (the rung it cleared
# at). Refreshed by every generate_qfn_fanout call; EMPTY when nothing
# escalated. The narrowing itself is in the fab_tiers ledger (one
# ``clearance`` row per stub), which is what JSON_SUMMARY design_rules and the
# end-of-run line read; this list is the per-pad view a test or the GUI can
# read without parsing that.
LAST_ESCALATED: List[Dict] = []

# #1641: the --nets patterns the last run selected no pad with, one dict per
# pattern (bga_fanout.net_filter_misses: `pattern`, `reason`, `nets`, `pads`).
# Refreshed by every generate_qfn_fanout call; EMPTY with no filter or when
# every pattern selected a pad. The CLI's JSON_SUMMARY `unmatched` and the
# fanout tab's completion message read it.
LAST_FILTER_MISSES: List[Dict] = []

# #1717: a stub run via-drops every interior pad its --nets filter selected
# (the surface fan has no edge for them); this names each one the via-drop
# could not place ("U1.34"), left with no copper. Its net is also failed.
# Refreshed by every generate_qfn_fanout call; EMPTY with no filter (an
# unscoped run never via-drops the exposed pad) and on an underpad run, whose
# dropped pads are its failures. The CLI's JSON_SUMMARY `skipped_interior` and
# the fanout tab's completion message read it.
LAST_SKIPPED_INTERIOR: List[str] = []

# #1775: the vias the last run laid below the board's own Board Setup via
# minimums with the project writeback off (`design_rules.board_setup_via_grade`
# rows; printed by the engine). Refreshed by every generate_qfn_fanout call;
# EMPTY when every via meets them. The CLI's JSON_SUMMARY `board_setup_vias`
# and the fanout tab's completion message read it.
LAST_BOARD_SETUP_VIAS: List[Dict] = []


def pad_has_via_escape(pad, segments, vias, escape_layer, copper_layers) -> bool:
    """Does this pad already have the escape a via-drop would give it (#1804)?

    Served means a same-net via that reaches ``escape_layer`` and either
    overlaps the pad's copper with its barrel, or is joined to the pad by one
    same-net track on a layer the pad has copper on (one end on the pad's
    copper, the other on the via's). That is the copper a via-drop lays --
    a via in the pad, or a stub to a via just past it -- so a re-run on its
    own output finds every pad it escaped served. ``segments`` and ``vias``
    are the pad's net's own (the caller filters by net)."""
    from bga_fanout.geometry import endpoint_on_pad
    from fab_notes import via_overlaps_pad
    from geometry_utils import via_on_layer
    from net_queries import expand_pad_layers
    reach = [v for v in vias if via_on_layer(v, escape_layer)]
    if not reach:
        return False
    if any(via_overlaps_pad(pad, v.x, v.y, v.size) for v in reach):
        return True
    pad_layers = set(expand_pad_layers(list(pad.layers or []), list(copper_layers)))

    def _on_via(x, y):
        return any(math.hypot(x - v.x, y - v.y) <= (v.size or 0.0) / 2 + 1e-6
                   for v in reach)
    for s in segments:
        if s.layer not in pad_layers:
            continue
        for (ax, ay), (bx, by) in (((s.start_x, s.start_y), (s.end_x, s.end_y)),
                                   ((s.end_x, s.end_y), (s.start_x, s.start_y))):
            if endpoint_on_pad(pad, ax, ay, POSITION_TOLERANCE) and _on_via(bx, by):
                return True
    return False


def part_pad_hull(footprint):
    """The convex hull of a part's pad copper (shapely), the region a
    surface escape must leave (#1834): every copper pad's rectangle, at its
    residual tilt. None for a part with no copper pad."""
    from shapely.geometry import MultiPoint
    from check_drc import _pad_has_no_copper
    pts = []
    for p in footprint.pads:
        if _pad_has_no_copper(p):
            continue
        hx, hy = (p.size_x or 0.0) / 2, (p.size_y or 0.0) / 2
        rr = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
        c, s = math.cos(rr), math.sin(rr)
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            pts.append((p.global_x + dx * c - dy * s, p.global_y + dx * s + dy * c))
    return MultiPoint(pts).convex_hull if pts else None


def pad_has_stub_escape(pad, segments, vias, part_hull, layer, copper_layers) -> bool:
    """Does this pad already have the surface escape a stub fan on ``layer``
    would give it (#1834)? Served means same-net track copper on a layer the
    pad has copper on, starting with a track end on the pad's copper and
    running on through tracks that touch (an end within the other's
    half-width, as KiCad joins them), either reaches a same-net via or ENDS
    outside the part's pad hull (`part_pad_hull`): a track end there that no
    other track of the net continues -- a stub's tip, a route's far end on
    its pad -- or the pad already has its via escape (`pad_has_via_escape`).
    Copper that stays inside the hull without a via -- a stub cut back to the
    pad, a tie to the next pin, a track inward -- does not serve the pad, nor
    does a closed outline of tracks drawn round the pad that only crosses the
    hull, so it is escaped again. ``segments`` and ``vias`` are the pad's
    net's own (the caller filters by net)."""
    from bga_fanout.geometry import endpoint_on_pad
    from geometry_utils import via_on_layer, point_to_segment_distance
    from net_queries import expand_pad_layers
    from shapely.geometry import Point
    if pad_has_via_escape(pad, segments, vias, layer, copper_layers):
        return True
    pad_layers = set(expand_pad_layers(list(pad.layers or []), list(copper_layers)))
    segs = [s for s in segments if s.layer in pad_layers]

    def _ends(s):
        return ((s.start_x, s.start_y), (s.end_x, s.end_y))

    def _on(x, y, o):
        return point_to_segment_distance(x, y, o.start_x, o.start_y, o.end_x, o.end_y) \
            <= (getattr(o, 'width', 0.0) or 0.0) / 2 + POSITION_TOLERANCE

    def _escapes(s):
        for x, y in _ends(s):
            if any(via_on_layer(v, s.layer)
                   and math.hypot(x - v.x, y - v.y) <= (v.size or 0.0) / 2 + 1e-6
                   for v in vias):
                return True
            if part_hull is not None and not part_hull.covers(Point(x, y)) \
                    and part_hull.distance(Point(x, y)) > POSITION_TOLERANCE \
                    and not any(o is not s and o.layer == s.layer and _on(x, y, o)
                                for o in segs):
                return True
        return False

    def _touch(a, b):
        return a.layer == b.layer and (any(_on(x, y, b) for x, y in _ends(a))
                                       or any(_on(x, y, a) for x, y in _ends(b)))

    frontier = [s for s in segs
                if any(endpoint_on_pad(pad, x, y, POSITION_TOLERANCE) for x, y in _ends(s))]
    seen = {id(s) for s in frontier}
    while frontier:
        s = frontier.pop()
        if _escapes(s):
            return True
        for o in segs:
            if id(o) not in seen and _touch(s, o):
                seen.add(id(o))
                frontier.append(o)
    return False


#: `_underpad_via_escape`'s placement for a pad a via of the same run
#: already sits in (#1804): no copper of its own, not dropped.
SERVED_IN_RUN = ('served-in-run',)


def underpad_pad_counts(ref: str) -> Dict:
    """The last via-drop's per-PAD ledger (#1804), as the CLI's JSON_SUMMARY
    and the fanout tab's completion message carry it; {} when the run
    via-dropped nothing. ``pads_requested`` = ``pads_escaped`` (a new via, a
    reused one, or one this run laid in a stacked pad's copper) +
    ``pads_already_served`` (skipped, the via was on the input) +
    ``len(dropped_pads)`` (no clear via offset, named "REF.PAD")."""
    r = LAST_UNDERPAD_REPORT
    if not r:
        return {}
    dropped = [f"{ref}.{num}" for num, _name in r.get('dropped_pads', ())]
    served = len(r.get('already_served_pads', ()))
    tried = r.get('pads_tried', 0)
    return {'pads_requested': tried + served,
            'pads_escaped': tried - len(dropped),
            'pads_reused': r.get('pads_reused', 0),
            'pads_already_served': served,
            'vias_placed': r.get('vias_placed', 0),
            'dropped_pads': dropped}


def stub_pad_counts(ref: str) -> Dict:
    """The last surface stub fan's per-PAD ledger (#1834), the same keys as
    :func:`underpad_pad_counts`; {} when no surface fan ran. ``pads_escaped``
    counts the stubs laid (one escalated or cut back included),
    ``pads_already_served`` the pads skipped because each had its escape."""
    r = LAST_STUB_REPORT
    if not r:
        return {}
    dropped = [f"{ref}.{num}" for num, _name in r.get('dropped_pads', ())]
    served = len(r.get('already_served_pads', ()))
    tried = r.get('pads_tried', 0)
    return {'pads_requested': tried + served,
            'pads_escaped': tried - len(dropped),
            'pads_reused': 0,
            'pads_already_served': served,
            'vias_placed': 0,
            'dropped_pads': dropped}


def pad_counts(ref: str) -> Dict:
    """The last run's per-PAD ledger, as the CLI's JSON_SUMMARY carries it:
    the surface fan's (#1834) and the via-drop's (#1804, the underpad
    escape or a stub run's interior pads), summed key by key. {} when the
    run escaped no pad either way."""
    parts = [c for c in (stub_pad_counts(ref), underpad_pad_counts(ref)) if c]
    if not parts:
        return {}
    out = dict(parts[0])
    for c in parts[1:]:
        for k, v in c.items():
            out[k] = out[k] + v
    return out


def served_pad_net_names() -> set:
    """The nets of every pad the last run left out because it already had
    its escape (#1804's via-drop, #1834's surface fan)."""
    return {n for r in (LAST_UNDERPAD_REPORT, LAST_STUB_REPORT)
            for _num, n in r.get('already_served_pads', ())}


def skipped_interior_line(pads: List[str]) -> str:
    """The one line naming LAST_SKIPPED_INTERIOR's pads (#1717), for the
    console and the fanout tab's completion message."""
    shown = ', '.join(pads[:8]) + (f" (+{len(pads) - 8} more)" if len(pads) > 8 else '')
    return (f"WARNING: {len(pads)} selected interior pad(s) given no copper: "
            f"{shown} -- the via-drop found no clear via site; a smaller "
            f"--via-size or --allow-via-in-pad may fit")


# #1743: the escape stubs the last run drew at its track width on a net that
# reads as power or ground (`list_nets.power_net_evidence`), one dict per pad:
# `pad` ("U1.5"), `net`, `width` (as drawn), `escape` ('surface' or
# 'via-drop') and `why`. A surface stub of a net --power-nets names is not
# listed: it takes the net's width where it fits (#1318). A via-drop stub
# never does, so one is listed while its named width is wider than drawn.
# Refreshed by every generate_qfn_fanout call. The CLI's JSON_SUMMARY
# `power_stubs_at_width` and the fanout tab's completion message read it.
LAST_POWER_STUBS_AT_WIDTH: List[Dict] = []


def power_stubs_lines(rows: List[Dict]) -> List[str]:
    """The lines naming LAST_POWER_STUBS_AT_WIDTH's stubs (#1743), for the
    console and the fanout tab's completion message: the stubs, then what
    widens them."""
    if not rows:
        return []
    shown = ', '.join(f"{r['pad']} {r['net']} ({r['escape']}, {r['why']})"
                      for r in rows[:8])
    more = f" (+{len(rows) - 8} more)" if len(rows) > 8 else ''
    out = [f"NOTE: {len(rows)} power/ground escape stub(s) drawn at the track "
           f"width {rows[0]['width']:g} mm: {shown}{more}"]
    surface = list(dict.fromkeys(r['net'] for r in rows if r['escape'] == 'surface'))
    if surface:
        out.append(f"  pass --power-nets {' '.join(surface[:8])} with "
                   f"--power-nets-widths to lay the surface stub(s) at that width "
                   f"where it fits")
    if any(r['escape'] == 'via-drop' for r in rows):
        out.append("  a via-drop stub is drawn at --width whether or not "
                   "--power-nets names its net; only a wider --width widens it")
    return out


def _note_power_stubs_at_width(footprint, pcb_data, tracks, infos, track_width,
                               power_nets, power_nets_widths):
    """#1743: fill LAST_POWER_STUBS_AT_WIDTH from the copper this run drew.
    INFOS is [(PadInfo, 'surface' | 'via-drop')]; a pad's stub is the track
    that starts on it. Copper is not changed."""
    starts = {}
    for t in tracks:
        k = (t.get('net_id'), round(t['start'][0], 4), round(t['start'][1], 4))
        starts.setdefault(k, t)
    rows = [(pi.pad, kind, starts.get((pi.pad.net_id, round(pi.pad.global_x, 4),
                                       round(pi.pad.global_y, 4))))
            for pi, kind in infos]
    rows = [(p, kind, t) for p, kind, t in rows
            if t is not None and t['width'] <= track_width + 1e-9]
    if not rows:
        return
    from list_nets import power_net_evidence
    why = power_net_evidence(pcb_data)
    named = {}
    if power_nets:
        from net_queries import identify_power_nets
        named = identify_power_nets(pcb_data, list(power_nets),
                                    list(power_nets_widths or ()))
    ref = getattr(footprint, 'reference', '?')
    seen = set()
    for pad, kind, t in rows:
        nid = pad.net_id
        reason = why.get(nid)
        if nid in named:
            if kind == 'surface' or named[nid] <= t['width'] + 1e-9:
                continue
            reason = f"--power-nets names it at {named[nid]:g} mm"
        label = f"{ref}.{pad.pad_number}"
        if reason is None or label in seen:
            continue
        seen.add(label)
        LAST_POWER_STUBS_AT_WIDTH.append({
            'pad': label, 'net': pad.net_name, 'width': round(t['width'], 4),
            'escape': kind, 'why': reason})
    for line in power_stubs_lines(LAST_POWER_STUBS_AT_WIDTH):
        print(f"  {line}")


#: #1565: (board, part) pairs whose held .kicad_dru rules this call named, so
#: an escape re-run down the clearance ladder says it once
_RULES_SAID: set = set()


def _say_rules(rules, footprint, pcb_data):
    if not rules.live:
        return
    key = (getattr(pcb_data, 'source_path', '') or '', getattr(footprint, 'reference', ''))
    if key not in _RULES_SAID:
        _RULES_SAID.add(key)
        print(f"  {rules.describe()}")


def resolve_qfn_clearance(explicit, default_class, ceiling=None):
    """The clearance a QFN fanout escapes at, and the lines that say why (#1328).

    ONE resolver for both fronts: qfn_fanout.py's main (``--clearance`` /
    ``--clearance-ceiling``) and the GUI QFN panel (its Clearance override box,
    the Basic tab's ceiling). It reads the way route.py reads its own flags:

    * ``explicit`` given (``--clearance``, or the panel's box ticked) -> that;
    * omitted -> the board's Default net class clearance (``default_class``,
      ``list_nets.board_default_netclass_clearance`` on the CLI, the live
      board's Default class in the GUI);
    * no Default class either (a board with no project, or a 0 the project
      leaves as "unset") -> ``routing_defaults.QFN_CLEARANCE``;

    then capped by ``ceiling`` (tighten-only). This used to default to
    QFN_CLEARANCE (0.1) whatever the board declared, and the fanout writeback
    clamps every net class to the clearance it ran at, so the chain's own
    ``--clearance-ceiling 0.15`` lowered steppenprobe's Default and Power
    classes 0.15 -> 0.1 and every later route step routed the board at 0.1
    -- while the same fanout escaped 31/31 pads at 0.15. A pad that cannot
    escape at the resolved value escalates on its own (see
    :func:`qfn_clearance_ladder`), so the floor drops only where a pad needs it.

    Returns ``(clearance, lines)``.
    """
    lines = []
    if explicit is not None:
        value = float(explicit)
    else:
        dc = None
        try:
            dc = float(default_class) if default_class is not None else None
        except (TypeError, ValueError):
            dc = None
        if dc is not None and dc > 0:
            value = dc
            lines.append(f"--clearance not given; escaping at the board Default "
                         f"net-class clearance {value:g}mm.")
        else:
            import routing_defaults as _rd
            value = float(_rd.QFN_CLEARANCE)
            lines.append(f"--clearance not given and the board declares no Default "
                         f"net class; escaping at the fallback {value:g}mm.")
    if ceiling is not None:
        try:
            ceiling = float(ceiling)
        except (TypeError, ValueError):
            ceiling = None
    if ceiling is not None and ceiling < value - 1e-12:
        lines.append(f"--clearance-ceiling {ceiling:g}: clearance capped at it for "
                     f"this run (was {value:g}).")
        value = ceiling
    return value, lines


def resolve_class_clearances(pcb_data, net_clearances=None, ceiling=None):
    """The NON-Default net-class clearances a QFN escape is spaced at (#1529),
    and the line that says so.

    ``net_clearances`` is ``{net_id: class clearance}`` as route.py's map
    (``list_nets.net_clearance_map_by_id``: a net in no class but Default has
    no entry and takes the run clearance). None reads it from the board's
    sibling ``.kicad_pro`` (``pcb_data.source_path``), as ``batch_route`` does
    for a caller that passes none; the GUI passes the live board's classes.
    ``ceiling`` (``--clearance-ceiling``, the GUI's Clearance ceiling) caps
    every class at ``min(class, ceiling)``, route.py's reading.

    Each foreign item is then priced at KiCad's pairwise
    ``max(run clearance, class A, class B)`` (``GridRouteConfig.
    pair_clearance``), so the fanout's copper meets the classes the
    writeback keeps (``fix_kicad_drc_settings.fanout_fix_kwargs``). An empty
    map is inert: every pair at the run clearance, as before.

    Returns ``({net_id: clearance}, lines)``."""
    if net_clearances is None:
        net_clearances = {}
        path = getattr(pcb_data, 'source_path', '') or ''
        if path and os.path.isfile(path):
            try:
                from list_nets import net_clearance_map_by_id
                net_clearances = net_clearance_map_by_id(
                    path, {nid: n.name for nid, n in pcb_data.nets.items()})
            except Exception as e:                           # noqa: BLE001
                return {}, [f"Warning: could not read the net-class clearances "
                            f"({e}); escaping at the run clearance."]
    nc = {int(nid): float(c) for nid, c in (net_clearances or {}).items()
          if c is not None}
    if not nc:
        return {}, []
    if ceiling is not None:
        from fab_tiers import cap_net_clearances
        nc = cap_net_clearances(
            nc, float(ceiling),
            net_names={nid: n.name for nid, n in pcb_data.nets.items()},
            board_path=getattr(pcb_data, 'source_path', '') or None)
    vals = sorted({round(v, 4) for v in nc.values()})
    mode = (f"capped at --clearance-ceiling {float(ceiling):g}"
            if ceiling is not None else "honoured in full (no --clearance-ceiling)")
    return nc, [f"Netclass clearances for {len(nc)} net(s), {mode} (mm: {vals}); "
                f"escape copper spaced at KiCad's pairwise max(A, B)."]


def qfn_clearance_ladder(clearance, copper_layer_count):
    """The clearances BELOW ``clearance`` a stub may escalate to, descending
    (#1328) -- empty when no descent is allowed.

    A surface stub that cannot escape at the run clearance (its straight
    segment grazes foreign copper, so it would be dropped) is retried at these,
    in order, and keeps the first that clears. The policy is the run's
    escalation policy (``--escalation`` / the GUI's Escalation choice, default
    ``routing_defaults.ESCALATION``): nothing under ``off``; down to the
    deepest rung of ``fab_tiers.escalation_rungs`` otherwise -- the tier's fab
    floor under ``fab``, raised to the board's own declared minimum (or its
    Default class when that is unset, #1160) under ``board``.

    The steps are ``routing_defaults.QFN_ESCALATION_STEP`` apart, plus each
    tier rung's own clearance, so the rung a stub keeps is within one step of
    what its copper actually needs -- that is the value the writeback records,
    never the fab floor it was allowed to reach."""
    from fab_tiers import escalation_rungs
    import routing_defaults as _rd
    rungs = escalation_rungs(copper_layer_count)
    if not rungs:
        return []
    floor = min(r['clearance'] for r in rungs)
    if floor >= clearance - 1e-9:
        return []
    step = _rd.QFN_ESCALATION_STEP
    vals = {round(r['clearance'], 4) for r in rungs
            if floor - 1e-9 <= r['clearance'] < clearance - 1e-9}
    k = 1
    while clearance - k * step > floor + 1e-9:
        vals.add(round(clearance - k * step, 4))
        k += 1
    vals.add(round(floor, 4))
    return sorted(vals, reverse=True)


def _record_escalations(footprint, pcb_data, escalated, clearance):
    """#1328: publish the stubs that escaped below ``clearance`` and disclose
    them as route.py discloses its own descents -- a ``clearance`` row per stub
    in the fab_tiers ledger (JSON_SUMMARY ``design_rules`` and the end-of-run
    ``Design rules`` line), a fab-tier escalation when the rung is below the
    tier's nominal floor, and the clearance ledger, whose minimum is the floor
    the writeback records: the rung the stub cleared at, never lower."""
    if not escalated:
        return
    from fab_tiers import note_narrowing, warn_fab_escalation, escalation_rungs
    import clearance_ledger
    rungs = escalation_rungs(len(pcb_data.board_info.copper_layers or ()) or 2)
    nominal = rungs[0]['clearance'] if rungs else None
    ref = footprint.reference
    for stub, cl in escalated:
        name = stub.pad.net_name
        where = f"{ref}.{stub.pad.pad_number}"
        LAST_ESCALATED.append({'net': name, 'pad': where, 'clearance': round(cl, 4)})
        note_narrowing(stub.net_id, 'clearance', clearance, cl, 'qfn escape',
                       net_name=name)
        clearance_ledger.record(cl)
        if nominal is not None and cl < nominal - 1e-9:
            warn_fab_escalation(f"qfn escape {where}")
    lo = min(cl for _, cl in escalated)
    names = ', '.join(f"{e['pad']} {e['net']} {e['clearance']:g}"
                      for e in LAST_ESCALATED)
    print(f"  Clearance escalation (#1328): {len(escalated)} stub(s) could not "
          f"escape at {clearance:g} mm and escaped at a lower clearance "
          f"(smallest {lo:g} mm): {names[:300]}")


def axis_offset_ladder(pad_width, via_size, step, mode='near'):
    """Signed offsets along the escape axis, under KICAD_QFN_ONPAD_REACH (#846).

    `mode` orders them: 'in' sweeps inward (toward the chip) first, 'near'
    alternates nearest first.

    NOT an "on-pad ladder", though it was called `_onpad` and documented as one.
    Only k = 0 is guaranteed on the pad. The increment is the INTER-NET stagger
    -- the centre-to-centre a via needs from a DIFFERENT net's via at this pitch
    -- and on a fine-pitch part that exceeds the pad: on routed_output's QFN-76
    (pitch 0.40, via 0.45, clearance 0.1) step is 0.4275 against a pad whose
    escape-axis extent is 0.875, so rung 1 lands 0.0100 mm inside the pad EDGE,
    rung 2 is 0.8550 mm out and rung 8 reaches +-3.4199 mm.

    Those rungs ARE load-bearing, measured by
    `tests/sweep_846_onpad_ladder.py` -- committed, so this stays checkable
    rather than remembered: confining the ladder regressed escapes on 1 of 5
    boards ('pad': routed_output U2, 15 -> 10) and 3 of 5 ('barrel'), improved
    none, and left drc_grazes identical arm-to-arm. So the default is 'full' --
    the ladder is right and the NAME was wrong.

    It is also where the long stubs come from, which is what #846 reports: on
    routed_output U2 the longest EMITTED stub is 3.0125 mm against a 0.875 mm
    pad. (An earlier draft of this docstring said 2.9924 mm -- that is the
    ladder's requested OFFSET at k = 7, before `snap()` puts the via on the
    routing grid, not the copper that shipped.)

    KICAD_QFN_ONPAD_REACH picks the arm -- 'full' (default), 'pad' (the via
    CENTRE stays on the pad), 'barrel' (the whole barrel does). An unrecognised
    value is 'full', so a typo cannot silently shorten the ladder.

    Module-level, and the engine's only source for these offsets, because a
    test that restates the arithmetic cannot detect the arithmetic changing:
    the first draft of tests/test_846_onpad_ladder_reach.py rebuilt the ladder
    from the same formula and every knob row of tests/mutate_846.py SURVIVED.

    Whether a via that lands here is IN a pad is not decided here -- the commit
    loop asks `via_overlaps_pad`, the fab question.
    """
    seq = [0.0]
    if mode == 'in':
        seq += [-k * step for k in range(1, 9)] + [k * step for k in range(1, 9)]
    else:                                   # 'near'
        for k in range(1, 9):
            seq += [k * step, -k * step]
    reach = {'pad': pad_width / 2.0,
             'barrel': pad_width / 2.0 - via_size / 2.0,
             }.get(env_knobs.QFN_ONPAD_REACH)
    if reach is not None:
        seq = [d for d in seq if abs(d) <= reach + 1e-9]
    return seq


def _snap_tip_on_grid(corner, tip, net_id, grid_step, grazes):
    """Move a shortened fan tip back ONTO the routing grid (#446).

    `corner` is the (on-grid-by-construction) start of the 45 fan, `tip` the
    clearance-shortened end, `grazes(p1, p2, nid)` the caller's foreign-copper
    gate. Returns an on-grid point when one is safe, else `tip` unchanged.

    Why: an off-grid stub terminal cannot be reached exactly by the on-grid
    router, which then stops a cell short and leaves a cap-overlap soft joint.

    Safety contract -- this can never introduce a clearance violation:
      * every candidate is re-tested with the caller's own `grazes` gate;
      * candidates are constrained to lie no FURTHER from the corner than the
        clearing tip (searching inward only, never back out toward the graze);
      * if nothing on-grid clears, the unsnapped clearing tip is returned.
    """
    if not grid_step or grid_step <= 0:
        return tip
    cx, cy = corner
    tx, ty = tip
    span = math.hypot(tx - cx, ty - cy)
    if span < 1e-9:
        return tip  # fan fully collapsed onto the corner; nothing to snap

    def on_grid(v):
        return abs(round(v / grid_step) - v / grid_step) < 1e-6

    if on_grid(tx) and on_grid(ty):
        return tip  # already there

    # Walk inward from the clearing tip; at each step consider the four grid
    # points bracketing that position, nearest first.
    for frac in (1.0, 0.85, 0.7, 0.55, 0.4, 0.25):
        px, py = cx + (tx - cx) * frac, cy + (ty - cy) * frac
        gx0, gy0 = math.floor(px / grid_step), math.floor(py / grid_step)
        cands = []
        for gx in (gx0, gx0 + 1):
            for gy in (gy0, gy0 + 1):
                qx, qy = gx * grid_step, gy * grid_step
                # never further out than the clearing tip
                if math.hypot(qx - cx, qy - cy) > span + 1e-9:
                    continue
                cands.append(((qx - px) ** 2 + (qy - py) ** 2, (qx, qy)))
        for _d, cand in sorted(cands):
            if not grazes(corner, cand, net_id):
                return cand
    return tip  # nothing on-grid is safe: keep the clear (off-grid) tip


# Public API
__all__ = [
    'generate_qfn_fanout',
    'main',
    # Types re-exported for external use
    'QFNLayout',
    'PadInfo',
    'FanoutStub',
]


def check_endpoint_spacing(stubs: List[FanoutStub], min_spacing: float) -> List[Tuple[int, int, float]]:
    """Check for endpoints that are too close together."""
    collisions = []
    for i, s1 in enumerate(stubs):
        for j, s2 in enumerate(stubs[i+1:], i+1):
            if s1.pad.net_id == s2.pad.net_id:
                continue
            dist = math.sqrt((s1.stub_end[0] - s2.stub_end[0])**2 +
                           (s1.stub_end[1] - s2.stub_end[1])**2)
            if dist < min_spacing:
                collisions.append((i, j, dist))
    return collisions


def _terminal_web_target(pcb_data, net_id, layer, legs, q, prev, pad, r, floor):
    """The point inside `pad` that a connector from the stub's new tip `q`
    must reach so the joint is a floor-width web -- or None when the joint
    already is one (#1332). This is close_soft_joints' terminal-web repair
    (#416), asked before the copper ships rather than after: the same
    closed-form pre-filter, confirmed by the same KiCad-exact erosion
    (`terminal_web_neck_exact`, run over the net's existing copper plus this
    stub's `legs`), and the same target. `prev` is the stub vertex before
    `q`; `floor` the connection-width floor the board ships graded at."""
    from types import SimpleNamespace
    from pcb_modification import (terminal_pad_web_shortfall,
                                  circular_pad_web_shortfall, _is_round_pad,
                                  terminal_web_neck_exact)
    from routing_utils import _to_pad_frame
    e = floor / 2.0
    if floor <= 0 or r < e - 1e-9:
        return None          # no floor-width web can exist through this track
    if getattr(pad, 'shape', None) not in ('rect', 'roundrect', 'oval', 'circle') \
            or not pad.size_x or not pad.size_y:
        return None          # custom-polygon pads have no closed-form web
    elx, ely = _to_pad_frame(q[0], q[1], pad)
    nlx, nly = _to_pad_frame(prev[0], prev[1], pad)
    if _is_round_pad(pad):
        is_neck, tloc = circular_pad_web_shortfall(elx, ely, pad.size_x / 2.0,
                                                   r, e)
    else:
        is_neck, tloc = terminal_pad_web_shortfall(
            nlx, nly, elx, ely, pad.size_x / 2.0, pad.size_y / 2.0, r, e)
    if not is_neck:
        return None
    view = SimpleNamespace(
        segments=[s for s in pcb_data.segments
                  if s.net_id == net_id and s.layer == layer]
        + [SimpleNamespace(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                           width=2.0 * r, layer=layer, net_id=net_id)
           for a, b in legs],
        pads_by_net=pcb_data.pads_by_net)
    if terminal_web_neck_exact(view, net_id, layer, q[0], q[1], floor) is False:
        return None
    tlx, tly = tloc
    rad = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
    cc, sn = math.cos(rad), math.sin(rad)
    bx = round(pad.global_x + tlx * cc - tly * sn, 4)
    by = round(pad.global_y + tlx * sn + tly * cc, 4)
    if math.hypot(bx - q[0], by - q[1]) < 1e-4:
        return None
    return (bx, by)


def _end_stubs_at_same_net_pads(stubs: List[FanoutStub], pads, track_width: float,
                                seg_grazes, pcb_data, floor: float,
                                step: float = 0.005) -> int:
    """End every stub at the first SAME-NET pad its copper reaches (#1332).

    The stub geometry and every clearance test before this skip same-net
    pads, so a fan could clip a same-net pad (a pin's own decoupling cap
    beside it) and run on past it; the copper beyond the contact is a
    dangling tail that route.py keeps as input copper. Here each stub's path
    (pad -> corner -> tip) is walked in `step` samples against the exact pad
    copper (`check_drc.point_to_pad_distance`: roundrect corners, rotation,
    custom polygons). At the first sample where the stub's copper overlaps a
    same-net pad -- centreline within half a track width of it -- the stub
    is followed to its DEEPEST point of that contact and ends there.

    A corner graze (steppenprobe C11.1: overlap ~0.04 mm on a roundrect
    corner) joins through a web thinner than the board's connection-width
    floor, which check_weird reports as narrow-pad-joint. When
    `_terminal_web_target` finds one, a closing segment runs from the tip
    into the pad, exactly as close_soft_joints would add it; it is tested
    with `seg_grazes` (the caller's foreign-copper/edge test) and omitted if
    it grazes anything, leaving the overlap as the joint.

    The stub's own pad -- any same-net pad its START already touches -- is
    where it comes from, not where it arrives, and is ignored. Returns the
    number of stubs changed; each carries `joined_pad` ("REF.PAD").
    """
    from check_drc import point_to_pad_distance
    from geometry_utils import point_to_segment_distance
    half_w = track_width / 2.0
    touch = half_w - 1e-6
    changed = 0
    for stub in stubs:
        nid = stub.net_id
        if not nid:
            continue
        legs = [(stub.pad_pos, stub.corner_pos), (stub.corner_pos, stub.stub_end)]
        x0, y0 = stub.pad_pos
        cands = []
        for p in pads:
            if p.net_id != nid or p.pad_type == 'np_thru_hole':
                continue
            if not (p.drill and p.drill > 0) and stub.layer not in (p.layers or []):
                continue
            reach = 0.5 * math.hypot(p.size_x or 0.0, p.size_y or 0.0) + half_w
            if all(point_to_segment_distance(p.global_x, p.global_y,
                                             a[0], a[1], b[0], b[1]) > reach
                   for a, b in legs):
                continue
            if point_to_pad_distance(x0, y0, p) < touch:
                continue
            cands.append(p)
        if not cands:
            continue

        def _samples():
            for li, (a, b) in enumerate(legs):
                n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1])
                                         / step)))
                for k in range(n + 1):
                    t = k / n
                    yield li, (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

        hit = best = None
        for li, (x, y) in _samples():
            if hit is None:
                for p in cands:
                    d = point_to_pad_distance(x, y, p)
                    if d < touch:
                        hit = p
                        best = (d, math.hypot(x - p.global_x, y - p.global_y),
                                li, (x, y))
                        break
                continue
            d = point_to_pad_distance(x, y, hit)
            if d >= touch:
                break
            key = (d, math.hypot(x - hit.global_x, y - hit.global_y))
            if key < best[:2]:
                best = key + (li, (x, y))
        if hit is None:
            continue
        _d, _dc, li, q = best
        if li == 0:
            stub.corner_pos = q
        stub.stub_end = q
        stub.joined_pad = f"{hit.component_ref}.{hit.pad_number}"
        kept = ([(stub.pad_pos, q)] if li == 0
                else [(stub.pad_pos, stub.corner_pos), (stub.corner_pos, q)])
        kept = [(a, b) for a, b in kept
                if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6]
        prev = kept[-1][0] if kept else stub.pad_pos
        target = _terminal_web_target(pcb_data, nid, stub.layer, kept, q, prev,
                                      hit, half_w, floor)
        if target is not None and not seg_grazes(q, target, nid):
            stub.join_end = target
        changed += 1
    return changed


def _board_edge_model(pcb_data, clearance, board_edge_clearance):
    """Edge.Cuts keep-out for fanout copper (issue #288, picodvi's 0.04mm
    board-edge graze -- qfn_fanout had no edge model at all). Returns
    (edge_clear, rings, outer, cutouts); rings is empty when the board has no
    usable outline AND no bounds (then no edge test is possible)."""
    from check_drc import board_edge_geometry
    edge_clear = board_edge_clearance if board_edge_clearance > 0 else clearance
    # #1678: a .kicad_dru edge_clearance rule outranks it, at the strictest
    # floor any net's copper is held to: an escape is laid before the net's
    # own route, and the fanout keeps one number per part.
    from design_rules import strictest_edge_floor
    edge_clear = strictest_edge_floor(pcb_data, edge_clear)
    rings, outer, cutouts = board_edge_geometry(pcb_data.board_info, rect_ring=True)
    return edge_clear, rings, outer, cutouts


def run_output_conflict(vx, vy, net_id, placed, px=None, py=None, *,
                        via_size, via_drill, clearance, track_width,
                        hole_to_hole, adds_via=True, pair_clearance=None):
    """Does a candidate via (and its stub) collide with THIS RUN's own output?

    D10. The underpad escape's `via_clears` tested a candidate against the
    board it was handed -- `foreign_vias` / `foreign_pads` / `foreign_tracks`
    are a snapshot taken once, before anything is placed -- and against the via
    CENTRES emitted so far. It never saw the STUBS the same run emitted, so it
    approved vias sitting on copper it had just laid itself, and those escapes
    came back DRC CONTACTS.

    Measured: U2 (QFN56) reported 39 of 46 escapes under
    `--escape-method underpad --allow-via-in-pad` and the gate rejected every
    one as a contact -- while the geometry was fine (a 1.4 mm moat admitting a
    0.7 mm via at all 56 pads, `pad_via == 0`, a real 0.700 mm via placeable
    DRC-clean in U2.43, true capacity 11 of 14 lanes per side). A valid escape
    strategy reported as impossible, and two cycles skipped fanout on it.

    `placed` entries are ``(via_x, via_y, net_id, pad_x, pad_y)``; the stub
    runs pad -> via. A through-via conflicts with copper on ANY layer, so the
    stubs are tested exactly the way `foreign_tracks` is -- the difference is
    only that these cannot be in that list.

    Module-level and pure so it is testable directly: the caller is a closure
    over fifteen locals, and a check this consequential should not be reachable
    only by routing a whole board.

    ``adds_via=False`` is the REUSE case (#479 audit gap 2): the position is an
    existing board via, so this run adds no drill and no via copper -- only the
    bridging stub. Everything about that via's spacing is a fact of the input
    board, not something this run creates, so pricing it as a NEW via at
    ``config.via_size`` judges two vias that already exist, at a size neither
    has, for a spacing this run does not produce. Measured on
    kicad_files/routed_output.kicad_pcb U2 (B.Cu, track/clearance 0.1, via
    0.45/0.25, --escape-method underpad --allow-via-in-pad): with the via terms
    applied, all 7 reuse candidates were rejected on the different-net floor
    ``via_size + clearance`` = 0.55 against pre-existing 0.30 vias sitting at
    0.400 mm -- legal (0.15+0.15+0.10) and already on the board. That cost
    Net-(U2A-DATA_30) its escape and put a fresh drill where a reuse would have
    served: 28 vias / 12 dropped / 30 tracks became 29 / 13 / 31.

    The VIA-TO-VIA term is not merely wrong there, it is REDUNDANT. Every
    entry in `placed` is either a via this run created -- already tested
    against this reuse target in `via_clears`'s `foreign_vias` loop at exact
    pairwise sizes (``via_size/2 + fs/2 + clearance``, and
    ``(via_drill + fd)/2 + h2h`` for same net) -- or another pre-existing via,
    whose spacing is the board's.

    TERM 1 (the candidate via against a stub this run emitted) is dropped for
    a different and weaker reason, stated here rather than overclaimed: the
    reuse target's POSITION is not this run's doing, so a stub grazing it is
    a defect of that stub, which is already emitted. Rejecting the reuse does
    not remove the graze -- it only forces a fresh drill elsewhere and leaves
    the graze in place. It is a false remedy, not a check.

    That it fires at all exposes a REAL and separate gap, measured rather than
    assumed: an emitted stub is never tested against a pre-existing via on
    another FANNED net, because `build_base_obstacle_map(nets_to_route=...)`
    excludes every net being escaped (obstacle_map.py:105), so
    `check_line_clearance` cannot see it, and `via_clears` tests only the via
    CENTRE against `foreign_vias`, never the stub. Measured on U2: 5 emitted
    stubs sit inside a foreign pre-existing via's floor, one of them at
    0.0000mm, and in all 5 the via's net is a fanned one. That count is
    IDENTICAL at 715c821 and here, so it is pre-existing and untouched by this
    change -- and it wants its own fix in the stub check, not an accidental
    partial cover for the subset of vias that happen to be reuse targets.

    So with ``adds_via=False`` only the new STUB is tested against this run's
    own output, which is the only copper the reuse emits. A stub shorter than
    POSITION_TOLERANCE emits no track at all (the commit loop skips it), so it
    conflicts with nothing.

    ``pair_clearance(net_a, net_b)``, when given, prices each pair at its
    two nets' classes (#1529) and the scoped and track .kicad_dru rules
    (#1565; ``kind='track'`` for the stub-against-stub term); else every pair
    is ``clearance``.

    Returns True when the candidate must be REJECTED.
    """
    from geometry_utils import segment_to_segment_distance
    from obstacle_map import point_to_segment_distance

    track_half = track_width / 2
    if not adds_via and (px is None
                         or math.hypot(px - vx, py - vy) <= POSITION_TOLERANCE):
        return False                    # reuse with no stub emits no copper
    for entry in placed:
        qx, qy, qn = entry[0], entry[1], entry[2]
        qpx = entry[3] if len(entry) > 3 else None
        qpy = entry[4] if len(entry) > 4 else None
        cl = (pair_clearance(net_id, qn) if pair_clearance is not None
              and qn != net_id else clearance)
        via_half = via_size / 2 + cl - 1e-6
        # Via-to-via. Same-net floor was via_size*0.5 -- BELOW drill
        # hole-to-hole for standard vias (#479 audit gap 2); both are via_drill.
        # The drill floor holds for a DIFFERENT-net pair too: KiCad grades
        # every pair of holes. Priced at the copper rule alone, a board
        # declaring min_hole_to_hole 0.3 got 0.25/0.15 vias in the pads of a
        # 0.4 mm-pitch connector, each pair 0.25 hole to hole.
        # Skipped for a REUSE: no drill and no via copper is added, so there is
        # no new pair for this run to space.
        if adds_via:
            floor = max(via_size + cl if qn != net_id
                        else via_size * 0.5, via_drill + hole_to_hole)
            if math.hypot(vx - qx, vy - qy) < floor - 1e-6:
                return True
        if qn == net_id:
            continue                           # own-net copper is no obstacle
        has_stub = (qpx is not None
                    and math.hypot(qpx - qx, qpy - qy) > POSITION_TOLERANCE)
        # 1. the candidate VIA against the stub already emitted for that via.
        #    Dropped for a reuse because the target's POSITION is not this
        #    run's doing: the graze belongs to that already-emitted stub, and
        #    refusing the reuse only buys a fresh drill while leaving it. See
        #    run_output_conflict's docstring for the separate, measured gap
        #    this exposes (stub vs pre-existing via on another FANNED net).
        if adds_via and has_stub \
                and point_to_segment_distance(vx, vy, qpx, qpy, qx, qy) \
                < via_half + track_half:
            return True
        if px is None:
            continue
        # 2. the candidate's own STUB against that placed via, and
        # 3. stub against stub -- the same blindness, the other way round.
        if point_to_segment_distance(qx, qy, px, py, vx, vy) \
                < via_half + track_half:
            return True
        if has_stub and segment_to_segment_distance(
                px, py, vx, vy, qpx, qpy, qx, qy) \
                < track_width + (pair_clearance(net_id, qn, kind='track')
                                 if pair_clearance is not None else cl) - 1e-6:
            return True
    return False


def _underpad_via_escape(footprint, pcb_data, pad_infos, layout, layer,
                         track_width, clearance, via_size, via_drill, grid_step,
                         allow_via_in_pad=False, board_edge_clearance=0.0,
                         progress_callback=None, net_clearances=None,
                         fanned_net_ids=None, same_net_pad_clearance=0.0):
    """Via-drop escape (issue #164): instead of a surface 45-degree fan, run a
    short stub from each pad to a through-via just past the pad edge and let
    signal routing pick the net up on an inner/back layer. This escapes a
    crowded fine-pitch edge where the *surface* is full (a neighbour pair on one
    side, a foreign track on the other) -- the case the surface fan cannot solve.

    "Foreign" copper is anything NOT on a net we're escaping right now, even if
    it sits on the SAME component (issue #161 reopen): a routed neighbour pair
    and a crossing track are usually on the chip's own other nets, so they must
    block the via even though they share the footprint. Each candidate via is
    obstacle-checked against foreign vias, foreign PADS and foreign TRACKS (the
    track check was the gap that let a via land on a neighbour's trace and short
    it -- issue #161); only the via's own net is exempt.

    Adjacent vias are staggered along the escape axis so two neighbours one pitch
    apart still clear each other. With `allow_via_in_pad` the via may sit on its
    own pad, so its candidate offsets are a MIX: on-pad positions (centre, then
    inward toward the chip) *and* off-pad positions (outward past the pad) -- a
    pad that can't escape one way may still escape the other. Without it the via
    stays clear of the pad body and only moves outward.

    Placement is greedy per side, but if the default stagger drops a pad the side
    is re-tried under alternative configurations -- reversed order, and per-pad
    direction biases (e.g. one leg back/inward, its neighbour forward/outward) --
    keeping whichever escapes the most pads (issue #161 follow-up). The default
    (forward, nearest-offset-first) is tried first, so when it already escapes
    every pad nothing changes. A pad with no clear offset under any configuration
    is dropped. Returns (tracks, vias, dropped_net_names).

    Interior ('center') pads form their own by_side group on net-scoped runs
    (issue #410); each uses its own long-axis escape direction from analyze_pad
    -- nothing here assumes a group shares an edge axis.

    ``net_clearances`` (``{net_id: class clearance}``, #1529) prices every
    pair the via and its stub make at KiCad's max(clearance, class A,
    class B), as the surface fan does; empty or None is the one clearance.

    ``fanned_net_ids`` (#1804) adds to the nets the obstacle map treats as
    being escaped: the caller's whole selection, when it has left out pads
    already served, so that leaving a pad out changes which pads are tried
    and nothing about how the rest are measured."""
    from obstacle_map import (build_base_obstacle_map, build_layer_map,
                              point_to_segment_distance)
    from geometry_utils import segment_to_segment_distance
    from bga_fanout.geometry import clamp_via_to_pad, endpoint_on_pad
    from fab_notes import via_overlaps_pad
    from list_nets import fab_floor_min, warn_fab_escalation
    from routing_config import GridRouteConfig

    # Fab floors for the via-in-pad clamp (#202): when a chosen via sits ON its
    # pad, size it to the pad edge so it can't bulge into a neighbouring net. Pass
    # the active fab-tier ladder so the clamp escalates standard->advanced (#237).
    _copper = len(getattr(pcb_data.board_info, 'copper_layers', None) or []) or 4
    from list_nets import escalation_rungs
    # escalation_rungs: empty under --escalation off, raised to the board's
    # own minimums under board (#857).
    floors = escalation_rungs(_copper)
    clamp_n = floor_n = escalated_n = offcentre_n = vip_n = reused_n = 0
    _esc_vias, _floor_vias = [], []    # #1579: what the warnings state

    # Only the nets we're escaping right now are exempt from the obstacle map --
    # the chip's OTHER nets (a routed neighbour pair, a crossing track) must
    # block, so we don't exclude all of the footprint's nets (issue #161).
    fanned_nets = {pi.pad.net_id for pi in pad_infos} | set(fanned_net_ids or ())
    cfg = GridRouteConfig(layers=list(pcb_data.board_info.copper_layers or [layer]),
                          track_width=track_width, clearance=clearance)
    _ref = getattr(footprint, 'reference', '?')

    def _prog(cur, tot, what):
        if progress_callback:
            progress_callback(cur, tot, f"QFN via-drop {_ref}: {what}")

    from kicad_dru import install_layer_clearances
    install_layer_clearances(cfg, None, None, pcb_data)  # #498
    # #1529: the net classes. `_pc(a, b)` is a pair the VIA makes (it spans
    # the stack, so the class pair before any layer rule, as `clearance`
    # was); `_spc(a, b)` one the STUB makes on its mount layer, where a
    # .kicad_dru rule replaces it (`_stub_clr` below). Both are the one
    # clearance on a board with no non-Default class.
    _nc = {int(n): float(c) for n, c in (net_clearances or {}).items()}
    cfg.set_net_clearances(_nc, ())
    _nc_foreign = {n: c for n, c in _nc.items() if n not in fanned_nets}
    # #1565: the class/type-scoped and track-scoped .kicad_dru rules, as the
    # surface fan takes them
    from kicad_dru import install_pair_rule_channels
    _rules = install_pair_rule_channels(cfg, pcb_data, fanned_nets)
    _say_rules(_rules, footprint, pcb_data)
    _priced = bool(_nc) or _rules.live

    def _pc(net_a, net_b, types=None, kind='layer'):
        return (_rules(net_a, net_b, base=clearance, types=types, kind=kind)
                if _priced else clearance)
    layer_map = build_layer_map(cfg.layers)
    _prog(0, 0, "building obstacle map...")
    # Pad copper is measured exactly, not stamped (bga_fanout.reroute.
    # PadFreeMap): the stamps carry the track half-width twice and refused
    # pad -> via stubs that clear the pad (mikoto_nrf52840, beside C2.2).
    from bga_fanout.reroute import PadFreeMap
    # #1529: one map per routing side of the stub's pairs -- its own class
    # raises it (unless the mount layer's rule replaces it) -- each stamping
    # a foreign obstacle at max(that, the obstacle's class).
    _stub_ruled = (getattr(cfg, 'layer_clearances', None) or {}).get(
        footprint.layer) is not None
    _maps = {}

    def _obstacles_for(net_id):
        f = clearance
        c = _nc.get(net_id) if (_nc and not _stub_ruled) else None
        if c is not None and c > f:
            f = c
        hit = _maps.get(f)
        if hit is None:
            if f == clearance:
                _c = cfg
            else:
                _c = GridRouteConfig(layers=list(cfg.layers), track_width=track_width,
                                     clearance=f)
                install_layer_clearances(_c, None, None, pcb_data)  # #498
                _c.set_net_clearances(_nc, ())
                if _rules.live:                                     # #1565
                    install_pair_rule_channels(_c, pcb_data, fanned_nets)
            hit = _maps[f] = (PadFreeMap(build_base_obstacle_map(
                pcb_data, _c, nets_to_route=list(fanned_nets),
                extra_clearance=track_width / 2, net_clearances=_nc_foreign,
                stamp_pads=False), pcb_data, set(fanned_nets), _c), _c)
        return hit
    _obstacles_for(None)
    # #845: the obstacle plane for the STUB, which is emitted on the pad's own
    # mount layer (:766 and :777, deliberately -- putting it on an inner/back
    # escape layer would float it above the pad, #195). This used to resolve
    # `layer`, the ESCAPE layer, which is where the VIA lands and not where the
    # stub's copper is; on the configuration under-pad exists for (an F.Cu part
    # escaped to B.Cu) every stub was clearance-tested against the wrong
    # layer's plane -- grading copper that is not there and ignoring copper
    # that is.
    #
    # It is the only consumer of this index. The VIA is not tested through the
    # map at all (it is a through via, and `via_clears` scans pcb_data
    # geometrically), so there was never a second reader for whom the escape
    # layer was the right answer.
    #
    # Per-layer clearance comes along for free: build_base_obstacle_map stamps
    # each layer at that layer's own rule (#498, installed into cfg at :331),
    # so reading the mount layer's plane also reads the mount layer's floor.
    stub_layer = footprint.layer
    stub_layer_idx = layer_map.get(stub_layer)

    # Foreign obstacles, keyed by net so the via's OWN net is exempt at check
    # time. A through-via spans every copper layer, so foreign tracks on ANY
    # layer matter -- don't filter tracks by layer.
    foreign_vias = [(v.x, v.y, v.size, v.net_id, v.drill) for v in pcb_data.vias]
    foreign_pads = [p for plist in pcb_data.pads_by_net.values() for p in plist]
    # #479 reuse-audit gap 2: existing same-net vias are REUSE targets (a
    # re-run / post-route fanout used to drop a fresh drill ON one), and
    # every drill on the board -- same-net included -- bounds new via holes
    # net-independently (KiCad hole_to_hole).
    _own_via_pos: Dict[int, List[Tuple[float, float]]] = {}
    for v in pcb_data.vias:
        _own_via_pos.setdefault(v.net_id, []).append((v.x, v.y))
    # #619: `nets_to_route` above ERASES every piece of copper on a net we are
    # escaping -- segments, vias AND pads (obstacle_map.py :216 / :299 / :312).
    # Right for a net's OWN copper; wrong for every OTHER net in the same call.
    # The candidate VIA still meets that copper (via_clears scans pcb_data
    # directly), but the pad->via STUB's only channel to the input board is
    # `check_line_clearance` on the holed map -- so stubs shipped straight
    # through the CENTRES of pre-existing vias on sibling fanned nets.
    #
    # The SURFACE fan already closes exactly this hole for itself, geometrically
    # (#257, the tail of `_seg_grazes`); the under-pad path returns ~200 lines
    # earlier and shares none of it. This is that backstop, ported.
    #
    # These lists are EXACTLY the complement of what `nets_to_route` erases:
    # build_base_obstacle_map stamps only segments, vias and pads and never
    # zone copper, so pour copper is invisible to the stub before and after
    # this change and nothing is being silently left out.
    _erased_vias = [v for v in pcb_data.vias if v.net_id in fanned_nets]
    _erased_segs = [s for s in pcb_data.segments if s.net_id in fanned_nets]
    # The pad half needs FOUR exclusions the surface fan's pad loop (:990-996)
    # does not make, or it phantom-rejects. Three are check_drc's own rules,
    # applied here so the gate refuses exactly what the grader flags:
    #   * NPTH carries no copper -- KiCad lists *.Cu on it for hole keep-out,
    #     but an np_thru_hole pad's "size" is only the mask opening. check_drc
    #     skips it for PAD-SEGMENT (`_pad_has_no_copper`, #260); so do we.
    #   * layer scope resolves through `pad_copper_layers`, which expands the
    #     `*.Cu` and `F&B.Cu` wildcards. A bare `layer in pad.layers` misses
    #     both -- zero occurrences in this corpus, but the pcbnew parse path
    #     emits them, so the GUI front would diverge from the CLI one (#722).
    #   * `pad.local_clearance` RAISES the floor: check_drc grades PAD-SEGMENT
    #     at `_pad_pair_cl = max(local_clearance, pair)`. It is per-pad, so it
    #     cannot be hoisted the way the surface fan hoists its `margin` -- that
    #     is precisely why the surface fan cannot honour it and this does.
    # The fourth is a net TIE: `_seg_hits_pad` is net-blind, and obstacle_map's
    # own tie lift only fires when exactly one net is being routed
    # (obstacle_map.py:338-341), so the under-pad path never gets it. Today a
    # tie partner on a fanned net is absent from the map by accident; without
    # this the pad half would turn that accident into a hard block with no lift.
    from check_drc import (_pad_has_no_copper, pad_copper_layers,
                           check_pad_segment_overlap, check_pad_via_overlap,
                           _net_tie_span_waived as _tie_span_waived)
    from kicad_parser import Segment as _Segment, Via as _Via
    _board_cu = list(pcb_data.board_info.copper_layers or [])
    _erased_pads = [p for p in foreign_pads
                    if p.net_id in fanned_nets
                    and not _pad_has_no_copper(p)
                    and footprint.layer in pad_copper_layers(p, _board_cu)]
    # The stub is emitted on `footprint.layer` -- the pad's OWN mount layer,
    # deliberately, so it does not float above the pad (#195) -- and NOT on the
    # `layer` argument. The caller's #498 dru swap resolved `clearance` for the
    # ESCAPE layer, which is where the VIA lands, not where the stub's copper
    # lives, and check_drc grades a segment with `_pair_cl(..., layer=seg.layer)`.
    # So the stub's own pair clearance is the mount layer's rule.
    # PARTIAL, and the limit is worth stating: when footprint.layer IS ruled
    # this is exact, but when it is not, the fallback is `clearance` -- which
    # the caller already rebound to the ESCAPE layer's rule (#498, :900-904).
    # So an unruled mount layer inherits the escape layer's number rather than
    # the base. Closing that means not rebinding the scalar in the CALLER at
    # all, which is a change to the surface fan's path too and is deliberately
    # not attempted here. The map-based test above does not share the problem:
    # build_base_obstacle_map stamps each layer at its own rule, so reading the
    # mount layer's plane already reads the mount layer's floor. Inert on every
    # board this repo ingests (#770: no tracked board carries a .kicad_dru
    # layer rule), and inert on the default path, where --layer IS the mount
    # layer.
    _stub_clr = cfg.layer_clearance(footprint.layer, clearance)

    def _spc(net_a, net_b, kind='layer', types=None):
        """#1529: `_stub_clr` for the stub of `net_a` against `net_b` (#1565:
        and the scoped and track rules for the items' types)."""
        if not _priced:
            return _stub_clr
        return _rules(net_a, net_b, footprint.layer, base=clearance, kind=kind,
                      types=types)
    _TV = (('track',), ('via',))       # #1565: a stub against a via
    # A SET of halves, so an A/B arm can be 'via', 'via,seg', 'off' or 'all'.
    # 'off' wins over everything (an explicit ablation is never partial), and
    # an empty or unrecognised value is 'all' -- a typo must not silently
    # disable the gate.
    _gate = env_knobs.QFN_UNDERPAD_ERASED_GATE
    _gsel = {t for t in _gate.replace(',', ' ').split() if t}
    # ANY unrecognised token means ALL and says so. The earlier form silently
    # dropped a half on a transposition -- 'sge,pad' ran the pad half only,
    # reporting itself as a deliberate two-half arm -- and quietly accepted the
    # PLURAL spellings ('vias', 'segs', 'pads') that LAST_ERASED_SETS itself
    # publishes, as well as 'none'/'0'/'false', which read as ablations and are
    # not. Fail-safe in direction (never silently OFF) and now audible.
    _known = {'all', 'off', 'via', 'seg', 'pad'}
    _bad = _gsel - _known
    if _bad:
        print(f"  WARNING: KICAD_QFN_UNDERPAD_ERASED_GATE={_gate!r} has "
              f"unrecognised token(s) {sorted(_bad)}; running ALL halves. "
              f"Valid: all | off | via | seg | pad (comma-separated).")
        _gsel = {'all'}
    _gate_all = not _gsel or 'all' in _gsel or not (_gsel & {'via', 'seg', 'pad'})
    _gate_via = 'off' not in _gsel and (_gate_all or 'via' in _gsel)
    _gate_seg = 'off' not in _gsel and (_gate_all or 'seg' in _gsel)
    _gate_pad = 'off' not in _gsel and (_gate_all or 'pad' in _gsel)

    def _tie_exempt(net_id):
        f = getattr(pcb_data, 'net_tie_exempt_pad_ids', None)
        return f(net_id) if f else ()

    global LAST_ERASED_SETS
    LAST_ERASED_SETS = {'nets': set(fanned_nets), 'vias': len(_erased_vias),
                        'segs': len(_erased_segs), 'pads': len(_erased_pads),
                        'layer': footprint.layer,
                        'clearance': _stub_clr, 'gate': _gate}
    from kicad_parser import pad_drill_circles as _pdc
    import routing_defaults as _rd
    # BOARD-FIRST, same rule as every other floor: a board declaring
    # min_hole_to_hole above the packaged default was having its drills spaced
    # at the default instead. Discovered via source_path so the GUI inherits it.
    #
    # RAISE-ONLY IN THE CODE, not only in this comment. `board_floor` is
    # board-AUTHORITATIVE, not raise-only -- it returns whatever the board
    # declares once it is positive, with no max() against the fallback, and
    # that is correct for the floors it mostly serves (check_channels and
    # check_assembly must grade at the board's own clearance even when that is
    # BELOW their default, or they manufacture phantom violations). It is a
    # DRILL floor here, so the same freedom is a fab hazard: a project
    # declaring `min_hole_to_hole: 0.10` resolved to (0.1, 'board constraint')
    # and spaced this run's drills below the 0.20 JLC floor. `resolve_hole_clearance`
    # is called raise-only for the same reason, and is raise-only only because
    # ITS consumers wrap it in a max() (obstacle_map.py:1580,
    # plane_obstacle_builder.py:1208) -- this is that wrap. The engine cannot
    # lean on the CLI's enforce_fab_floors: that pins args.hole_to_hole_clearance,
    # a value this code path never reads.
    from list_nets import board_floor
    _h2h_decl, _h2h_src = board_floor(
        getattr(pcb_data, 'source_path', "") or "", 'hole_to_hole',
        None, _rd.HOLE_TO_HOLE_CLEARANCE)
    _h2h_fab = fab_floor_min(_copper).get('hole_to_hole', 0.0)
    _h2h = max(_h2h_decl, _h2h_fab)
    if _h2h_src == 'board constraint' and _h2h_decl > _rd.HOLE_TO_HOLE_CLEARANCE:
        print(f"  Hole-to-hole {_h2h:g}mm (from the board's own "
              f"min_hole_to_hole)")
    elif _h2h_src == 'board constraint' and _h2h_decl < _h2h_fab:
        # Never SILENTLY relaxed -- the whole point of the guard is that a
        # board file cannot lower a fab floor without saying so. A user who
        # genuinely has a finer fab declares it with --fab-tier/--fab-overrides,
        # which is what fab_floor_min reads.
        print(f"  Board min_hole_to_hole {_h2h_decl:g}mm is below the "
              f"{_h2h_fab:g}mm fab hole-to-hole floor; using {_h2h:g}mm.")
    _drilled_pad_holes = [(hx, hy, hd)
                          for p in foreign_pads if p.drill and p.drill > 0
                          for (hx, hy, hd) in _pdc(p)]
    foreign_tracks = [(s.start_x, s.start_y, s.end_x, s.end_y, s.width, s.net_id)
                      for s in pcb_data.segments]

    pitch = layout.pad_pitch or 0.5
    # Min centre-to-centre, different nets: the copper rule, or the drill
    # rule when the board's hole-to-hole asks more (it is net-independent).
    need_cc = max(via_size + clearance, via_drill + _h2h)
    stagger = (math.sqrt(max(0.0, need_cc * need_cc - pitch * pitch)) + 0.05
               if need_cc > pitch else 0.0)
    step = max(stagger, grid_step, 0.05)        # offset increment along the escape axis

    _edge_clear, _edge_rings, _edge_outer, _edge_cutouts = _board_edge_model(
        pcb_data, clearance, board_edge_clearance)
    from check_drc import _point_to_rings_distance as _pt_rings_dist
    from check_drc import _point_on_board as _pt_on_board
    # #1682: a filled graphic's interior, on the nets this run erases from its
    # map too (a fanned net's shape is foreign to the part's other pins).
    from check_drc import FilledCopperGate
    _filled_gate = FilledCopperGate(pcb_data)

    # #1568: each pad with the radius about its centre that holds all its
    # copper, so `via_clears` calls the grader only on a pad the via's
    # keep-out can reach -- a pad whose disc it clears cannot be hit. An NPTH
    # pad carries its drill instead: it has no copper, and its HOLE is what
    # KiCad and check_drc hold a via's copper off.
    from bga_fanout.geometry import pad_reach
    from check_drc import npth_via_clearance, check_via_hole_overlap
    from kicad_parser import pad_drill_capsule
    _pad_discs = [(p, p.global_x, p.global_y, pad_reach(p),
                   (pad_drill_capsule(p) if _pad_has_no_copper(p)
                    and (p.drill or 0) > 0 else None))
                  for p in foreign_pads]
    _via_cu = _board_cu or list(cfg.layers)
    # The board's declared copper-to-hole floor, read as every routing step
    # reads it (min_hole_clearance, or the fab_floor_origin a writeback
    # relaxed; 0 when it declares none).
    from obstacle_map import resolve_hole_clearance
    _hole_clr = resolve_hole_clearance(pcb_data, cfg)
    # #581/#962: under an active same-net pad via clearance the net's solder
    # paste OPENINGS are kept clear too, as on the routing path
    # (`same_net_pad_via_keepout_cells`): a graphic opening, a paste-only
    # window or an opening larger than its pad is not seen by the pad test.
    _paste_aps = {}

    def _paste_hit(vx, vy, net_id):
        if same_net_pad_clearance <= 0:
            return False
        if net_id not in _paste_aps:
            from obstacle_map import paste_keepout_apertures
            _paste_aps[net_id] = paste_keepout_apertures(pcb_data, net_id)
        from paste_apertures import aperture_distance
        return any(aperture_distance(vx, vy, ap) < via_size / 2 + same_net_pad_clearance - 1e-6
                   for ap in _paste_aps[net_id])

    def via_clears(vx, vy, net_id, placed, px=None, py=None):
        """Is a via at (vx, vy) legal, given the board AND this run's own output?

        `foreign_vias` / `foreign_pads` / `foreign_tracks` above are a SNAPSHOT
        of the INPUT board, taken once. `placed` is what this run has emitted so
        far -- and it used to carry only via CENTRES, so the STUBS this run laid
        from each pad to each via were invisible to every later candidate. The
        test therefore approved vias sitting on copper it had just emitted
        itself, and each such escape came back a DRC CONTACT.

        Measured: U2 (QFN, 56 pads) reported 39 of 46 escapes under
        `--escape-method underpad --allow-via-in-pad`, and the gate rejected
        every one of them as a contact. The wall was NOT geometry -- U2 has a
        1.4 mm moat that admits a 0.7 mm via at all 56 pads, `pad_via == 0` in
        every run, and a real 0.700 mm via IS placeable DRC-clean in pad U2.43.
        True capacity is 11 of 14 lanes per side. So a valid escape strategy
        reported as impossible, and two cycles skipped fanout on that premise.

        `px, py` are the candidate's OWN pad, so its stub can be tested too --
        the same blindness in the other direction.
        """
        if _edge_rings:
            if not _pt_on_board(vx, vy, _edge_outer, _edge_cutouts):
                return False
            if _pt_rings_dist(vx, vy, _edge_rings) < via_size / 2 + _edge_clear - 1e-6:
                return False
        if _filled_gate and _filled_gate.via_hit(
                vx, vy, via_size, net_id, lambda sh: _pc(net_id, sh.net_id)) is not None:
            return False
        for fx, fy, fs, fn, fd in foreign_vias:
            # The DRILLS are an obstacle whatever the net: hole-to-hole is
            # net-independent (#282/#479 audit gap 2), and on a board whose
            # min_hole_to_hole outgrows via_size + clearance - via_drill it is
            # the binding rule against a foreign via too.
            if math.hypot(vx - fx, vy - fy) < (via_drill + fd) / 2 + _h2h - 1e-6:
                return False
            if fn == net_id:
                continue                        # same-net copper is no obstacle
            if math.hypot(vx - fx, vy - fy) < via_size / 2 + fs / 2 + _pc(
                    net_id, fn, (('via',), ('via',))) - 1e-6:
                return False
        for hx, hy, hd in _drilled_pad_holes:
            # Net-independent drill floor vs every drilled pad (slot-exact).
            if math.hypot(vx - hx, vy - hy) < (via_drill + hd) / 2 + _h2h - 1e-6:
                return False
        _cand = _Via(x=vx, y=vy, size=via_size, drill=via_drill,
                     layers=list(_via_cu), net_id=net_id)
        if _paste_hit(vx, vy, net_id):
            return False
        for pad, gx, gy, reach, hole in _pad_discs:
            if pad.net_id == net_id:            # own net (incl. via-in-pad) is fine
                # #581: an active same-net pad via clearance keeps the via that far off every
                # same-net SMD pad (an open via beside a pad wicks solder), not only out of it.
                if (same_net_pad_clearance > 0
                        and not (pad.drill and pad.drill > 0)
                        and check_pad_via_overlap(pad, _cand, same_net_pad_clearance,
                                                  _via_cu, 0.0)[0]):
                    return False
                continue
            if _pad_has_no_copper(pad):
                # An NPTH hole: the via's copper is held off the hole wall at
                # check_drc's own requirement (the pad's override above the
                # NPTH floor, else max(pair clearance, the board's declared
                # copper-to-hole floor)) -- before, it was tested as a pad
                # shape at the pair clearance, which a board declaring a
                # wider hole floor grades. Plated holes and vias are held at
                # copper clearance, as the route steps hold them (#1217).
                if hole is not None and check_via_hole_overlap(
                        vx, vy, via_size, hole, npth_via_clearance(
                            getattr(pad, 'local_clearance', 0.0) or 0.0,
                            _pc(net_id, pad.net_id), _hole_clr), 0.0)[0]:
                    return False
                continue
            # #1568: CALL the grader, as the stub half does (#619). A box grown
            # by the clearance with SQUARE corners (`_seg_hits_pad`) overstates
            # the keep-out beside a pad corner by up to margin * (sqrt(2) - 1)
            # and ignores a roundrect's radius, so under a class or rule above
            # the board clearance it refused vias check_drc grades clean.
            # Margin 0: refuse anything below the pair clearance, never only
            # what the grader's tolerance would flag. The via spans the stack,
            # so an SMD pad on any one layer still conflicts. A pad override
            # RAISES the floor, as the stub half prices it.
            _pcl = _pc(net_id, pad.net_id, (('via',), (('pad', pad.pad_type or 'smd'),)))
            _eff = max(_pcl, getattr(pad, 'local_clearance', 0.0) or 0.0)
            if math.hypot(vx - gx, vy - gy) - reach >= via_size / 2 + _eff:
                continue        # the pad's copper lies inside `reach` of its centre
            if check_pad_via_overlap(pad, _cand, _eff, _via_cu, 0.0)[0]:
                return False
        for sx0, sy0, sx1, sy1, sw, sn in foreign_tracks:
            if sn == net_id:
                continue
            if point_to_segment_distance(vx, vy, sx0, sy0, sx1, sy1) \
                    < via_size / 2 + sw / 2 + _pc(net_id, sn, (('via',), ('track',))) - 1e-6:
                return False
        return not run_output_conflict(
            vx, vy, net_id, placed, px, py,
            via_size=via_size, via_drill=via_drill, clearance=clearance,
            track_width=track_width, hole_to_hole=_h2h,
            pair_clearance=_pc if _priced else None)

    def stub_clears_erased(px, py, vx, vy, net_id):
        """#619: is the pad->via stub clear of the copper `nets_to_route` erased?

        The tail of the surface fan's `_seg_grazes` (#257), applied to the
        bridging stub. Returns True when the stub is CLEAR -- the opposite
        polarity to `_seg_grazes`, so it reads like the `stub_ok` it is
        and-ed into.

        Deliberately LAYER-BLIND for vias. `check_drc.check_via_segment_overlap`
        and `obstacle_map._add_via_obstacle` treat a via KiCad types blind,
        buried or micro as copper on its span only (#1268) and every other via
        as through, so this check is a superset of both: it can refuse a stub
        a typed blind via would not have touched, never pass one the grader
        flags.

        Called BEFORE `via_clears`, not after, and that ordering is
        load-bearing rather than cosmetic: `via_clears` scans, per candidate,
        every board via + every board pad (a pad-distance test each)
        + every board segment -- ~20k point tests on routed_output U2, where
        `_erased_vias` is 89. Running this first kills a doomed candidate for
        ~0.5% of that cost. Measured on U2: correctly ordered 7.0s against a
        10.9s baseline (-36%); appended after `via_clears` instead, +18%.
        The gate is a speed-up.
        """
        if stub_layer_idx is not None and _filled_gate and _filled_gate.segment_hit(
                px, py, vx, vy, track_width, stub_layer, net_id,
                lambda sh: _pc(net_id, sh.net_id)) is not None:
            return False
        if math.hypot(vx - px, vy - py) <= POSITION_TOLERANCE:
            # Via centred on the pad: the commit loop emits NO track at all
            # (:604, :616), so there is no stub to test. `run_output_conflict`
            # states the same rule for the reuse case at :213-215; without
            # this, a zero-length reuse -- a pre-existing via-in-pad at the pad
            # centre, the canonical re-run case -- is judged as a degenerate
            # segment and can be rejected for copper it will never emit.
            return True
        if _gate_via:
            for v in _erased_vias:
                if v.net_id == net_id:
                    continue                # own-net copper is no obstacle
                if point_to_segment_distance(v.x, v.y, px, py, vx, vy) \
                        < v.size / 2 + track_width / 2 + _spc(net_id, v.net_id, types=_TV) - 1e-6:
                    return False
        if _gate_seg:
            # Segments, unlike vias, really ARE single-layer objects, so this
            # half filters -- on `footprint.layer`, where the stub's copper is
            # emitted, NOT on `layer`. The surface fan compares against `layer`
            # (:999) only because ITS stubs land there. Measured on U2: the two
            # spellings disagree completely -- `footprint.layer` (F.Cu) finds 25
            # pairs, `layer` (B.Cu) finds 17 DIFFERENT ones.
            for s in _erased_segs:
                if s.net_id == net_id or s.layer != footprint.layer:
                    continue
                if segment_to_segment_distance(px, py, vx, vy,
                                               s.start_x, s.start_y,
                                               s.end_x, s.end_y) \
                        < s.width / 2 + track_width / 2 + _spc(net_id, s.net_id, 'track') - 1e-6:
                    return False
        if _gate_pad and _erased_pads:
            # CALL the grader; do not mirror it. An earlier revision of this
            # used `_seg_hits_pad` with a hand-built margin, which is what the
            # surface fan's pad loop does -- and it was measurably NOT the same
            # predicate. Audited over the whole corpus at 0.1/0.1/0.45, it
            # rejected 83 candidate/pad pairs that `check_pad_segment_overlap`
            # grades CLEAN, the worst 0.234mm clear of the requirement, from
            # two causes it cannot express:
            #   * `_seg_hits_pad` tests the full axis-aligned RECTANGLE and
            #     grows it with square corners, while check_drc resolves a
            #     `corner_radius` for circle/oval/roundrect and calls
            #     `segment_to_rect_distance` with it -- a 1.45mm round pad
            #     becomes a 1.6x1.6 box whose corner is 1.131mm from centre
            #     where the true keep-out is 0.875mm (61 of the 83); and
            #   * it rejects at 1e-6 while the grader forgives `_grade_tol` =
            #     5% of clearance, so a 0.005mm overlap at CL 0.1 is clean to
            #     check_drc and a violation here (the other 22).
            # It is also SAMPLED (`samples=16`), so on a long stub a small pad
            # can fall between two samples: a 0.25mm pad on a 6.95mm stub --
            # reachable at via 0.8 / pitch 0.4, where the ladder's top offset
            # is exactly 6.95mm -- was reported CLEAR while the stub ran
            # through its centre. `check_pad_segment_overlap` is exact.
            seg = _Segment(start_x=px, start_y=py, end_x=vx, end_y=vy,
                           width=track_width, layer=footprint.layer,
                           net_id=net_id)
            _tie = _tie_exempt(net_id)
            for p in _erased_pads:
                if p.net_id == net_id:
                    continue
                # The net-tie waiver is check_drc's BOTH-condition form. The
                # exemption is LOCAL -- KiCad waives the contact only where it
                # lies on the tied net's own pad (DRC_ENGINE::IsNetTieExclusion)
                # -- so skipping the partner pad outright would wave through a
                # real short 2-3mm away, which is exactly the geometry the
                # ladder produces. `id(p) in _tie` alone is NOT the rule.
                _pcl = _spc(net_id, p.net_id,
                            types=(('track',), (('pad', p.pad_type or 'smd'),)))
                if id(p) in _tie and _tie_span_waived(pcb_data, seg, net_id,
                                                     p, _pcl):
                    continue
                # local_clearance is per-PAD and RAISES the floor, exactly as
                # check_drc's `_pad_pair_cl = max(local_clearance, pair)`.
                _eff = max(_pcl, getattr(p, 'local_clearance', 0.0) or 0.0)
                if check_pad_segment_overlap(p, seg, _eff, _board_cu)[0]:
                    return False
        return True

    def snap(v):
        return round(v / grid_step) * grid_step if grid_step > 0 else v

    def candidate_offsets(pad_width, mode):
        # `outward` starts past the pad body and always clears it. With
        # --allow-via-in-pad we ALSO offer the axis ladder, which starts ON the
        # pad centre and steps by the inter-net stagger -- so it reaches on-pad
        # positions AND off-pad ones on the inward side (#846). 'out' prefers
        # the outward ladder and falls back to the axis one; 'near'/'in' prefer
        # the axis ladder and fall back to outward.
        base = pad_width / 2 + via_size / 2 + clearance
        outward = [base + k * step for k in range(0, 9)]
        if not allow_via_in_pad:
            return outward
        axis = axis_offset_ladder(pad_width, via_size, step,
                                  'near' if mode == 'out' else mode)
        if mode == 'out':
            return outward + axis
        return axis + outward

    def place_pin(pi, mode, placed):
        ex, ey = pi.escape_direction
        px, py = pi.pad.global_x, pi.pad.global_y
        _om, _ocfg = _obstacles_for(pi.pad.net_id)          # #1529
        # #479 audit gap 2 (reuse): an existing same-net via within stub
        # reach makes a new drill pointless (and often sub-h2h illegal).
        # Bridge to it instead; the commit loop skips the via emission.
        _best = None
        for (ovx, ovy) in _own_via_pos.get(pi.pad.net_id, ()):
            _d = math.hypot(ovx - px, ovy - py)
            if _d < pi.pad_width / 2 + via_size / 2 + 0.1 \
                    and (_best is None or _d < _best[0]):
                _best = (_d, ovx, ovy)
        if _best is not None:
            _rvx, _rvy = _best[1], _best[2]
            # `check_line_clearance` reads the INPUT-board obstacle map, so on
            # its own this branch is blind to everything this run has emitted
            # -- the reuse path was one of two ways a position is returned, and
            # the only one that never consulted via_clears. The bridging stub
            # it emits can still cross another net's stub or via laid earlier
            # in the same run, and the position was then appended to
            # placed_global as though it had been checked.
            #
            # It calls run_output_conflict directly rather than via_clears:
            # the reused via IS on the board, so it is in `foreign_vias` at
            # distance 0 from itself, and the full test would reject every
            # reuse on its own same-net drill floor. Only this run's output is
            # in question here.
            #
            # And `adds_via=False`, because a reuse adds NO drill and NO via
            # copper -- only the bridging stub. Pricing the existing via as a
            # new one at config.via_size rejected every reuse on this board:
            # 0.30 board vias 0.400mm apart (legal at 0.15+0.15+0.10, and
            # already there) judged against a demanded 0.55. That was measured
            # as Net-(U2A-DATA_30) losing its escape to a fresh drill --
            # 28/12/30 vias/dropped/tracks becoming 29/13/31 on U2.
            #
            # stub_clears_erased is a SEPARATE `and` term, never folded into
            # the `stub_layer_idx is None` `or`: that short-circuit skips the
            # whole clearance test when the escape layer is not in the layer
            # map, and #619 is pure geometry on the mount layer that must run
            # regardless.
            if (stub_layer_idx is None or
                    _om.line_clear(px, py, _rvx, _rvy, stub_layer, _ocfg)) \
                    and stub_clears_erased(px, py, _rvx, _rvy, pi.pad.net_id) \
                    and not run_output_conflict(
                        _rvx, _rvy, pi.pad.net_id, placed, px, py,
                        via_size=via_size, via_drill=via_drill,
                        clearance=clearance, track_width=track_width,
                        hole_to_hole=_h2h, adds_via=False,
                        pair_clearance=_pc if _priced else None):
                return (_rvx, _rvy)
        for d in candidate_offsets(pi.pad_width, mode):
            vx, vy = snap(px + ex * d), snap(py + ey * d)
            stub_ok = (stub_layer_idx is None or
                       _om.line_clear(px, py, vx, vy, stub_layer, _ocfg))
            if stub_ok \
                    and stub_clears_erased(px, py, vx, vy, pi.pad.net_id) \
                    and via_clears(vx, vy, pi.pad.net_id, placed, px, py):
                return (vx, vy)
        return None

    def trial(pis, order, mode_fn, committed):
        # Greedily place a side under one configuration; return idx->pos|None.
        placed = list(committed)
        results = {}
        for idx in order:
            # #1804: a pad whose copper holds the centre of a same-net via
            # of this run (stacked pads of one exposed pad: its rect and the
            # thermal hole at its centre) has its escape; a second drill
            # there is refused on hole-to-hole and read as a dropped pad.
            # The CENTRE, not the barrel: the commit loop may clamp that via
            # to its own pad's edge, and any size of it still lands here.
            _p = pis[idx].pad
            if any(n == _p.net_id and endpoint_on_pad(_p, vx, vy, POSITION_TOLERANCE)
                   for vx, vy, n, _px, _py in placed):
                results[idx] = SERVED_IN_RUN
                continue
            pos = place_pin(pis[idx], mode_fn(idx), placed)
            results[idx] = pos
            if pos is not None:
                # Carry the PAD origin, so the stub this placement implies is
                # visible to every later candidate in the same run (D10).
                placed.append((pos[0], pos[1], pis[idx].pad.net_id,
                               pis[idx].pad.global_x, pis[idx].pad.global_y))
        return results

    # Stagger configurations, tried in order; the most-escaped wins, ties keep
    # the earliest. The default (forward, nearest-offset-first) is first, so a
    # side every pad already escapes is unchanged. Alternatives only matter when
    # a pad failed: reversed order, and per-pad direction biases that stagger one
    # leg back/inward while its neighbour goes forward/outward. Direction biases
    # only do anything with via-in-pad (otherwise every mode is the outward
    # ladder), so skip them then.
    configs = [('fwd', lambda i: 'near'), ('rev', lambda i: 'near')]
    if allow_via_in_pad:
        configs += [
            ('fwd', lambda i: 'out' if i % 2 == 0 else 'in'),
            ('fwd', lambda i: 'in' if i % 2 == 0 else 'out'),
            ('fwd', lambda i: 'in'),
            ('fwd', lambda i: 'out'),
        ]
    # Test/debug hook: pin the search to the default config to measure how many
    # pads the alternative-stagger search rescues. Not a user-facing option.
    if env_knobs.QFN_UNDERPAD_NO_ALT_STAGGER:
        configs = configs[:1]

    tracks, vias, dropped = [], [], []
    dropped_pads = []        # (pad number, net name): `dropped` names nets only
    placed_global = []
    by_side = defaultdict(list)
    for pi in pad_infos:
        by_side[pi.side].append(pi)

    n_alt = 0
    for _si, (side, pis) in enumerate(by_side.items()):
        _prog(_si + 1, len(by_side),
              f"placing via drops, side {side} ({len(pis)} pin(s))")
        pis.sort(key=lambda pi: (pi.pad.global_x, pi.pad.global_y))
        order_fwd = list(range(len(pis)))
        best, best_n, best_ci = None, -1, 0
        for ci, (order_name, mode_fn) in enumerate(configs):
            order = order_fwd if order_name == 'fwd' else order_fwd[::-1]
            results = trial(pis, order, mode_fn, placed_global)
            n_placed = sum(1 for v in results.values() if v is not None)
            if n_placed > best_n:
                best, best_n, best_ci = results, n_placed, ci
            if best_n == len(pis):
                break
        if best_ci != 0:
            n_alt += 1
        # Commit the winning configuration.
        for idx in order_fwd:
            pi = pis[idx]
            pos = best.get(idx)
            if pos is None:
                dropped.append(pi.pad.net_name)
                dropped_pads.append((pi.pad.pad_number, pi.pad.net_name))
                continue
            if pos is SERVED_IN_RUN:
                continue
            vx, vy = pos
            px, py = pi.pad.global_x, pi.pad.global_y
            # Committed across SIDES: side 2's candidates must see side 1's
            # stubs, not only its via centres (D10).
            placed_global.append((vx, vy, pi.pad.net_id, px, py))
            # Reused an existing same-net via (#479 audit gap 2): emit only
            # the bridging stub, never a duplicate drill.
            if any(abs(vx - ox) < 1e-4 and abs(vy - oy) < 1e-4
                   for ox, oy in _own_via_pos.get(pi.pad.net_id, ())):
                reused_n += 1
                if math.hypot(vx - px, vy - py) > POSITION_TOLERANCE:
                    tracks.append({'start': (px, py), 'end': (vx, vy),
                                   'width': track_width,
                                   'layer': footprint.layer,
                                   'net_id': pi.pad.net_id})
                continue
            # Zero-length stub (via centred on the pad) needs no track.
            # The connecting stub bridges the pad to the via, so it must live on
            # the pad's own copper layer (the footprint mount layer) -- the
            # through-via then carries the net down to the `--layer` target.
            # Putting it on `layer` (an inner/back escape layer) would float it
            # above the pad and leave the pad disconnected (issue #195).
            if math.hypot(vx - px, vy - py) > POSITION_TOLERANCE:
                tracks.append({'start': (px, py), 'end': (vx, vy),
                               'width': track_width, 'layer': footprint.layer,
                               'net_id': pi.pad.net_id})
            # Whether a STUB is needed and whether the via is IN THE PAD are
            # two questions, and this loop used to answer both with one 0.001mm
            # centre-coincidence test (#846). They come apart in two ways:
            #
            #  * `snap()` quantises the via COORDINATE to the routing grid
            #    (0.05 by default) while pad centres are off-lattice on real
            #    parts -- 76 of 77 pads on routed_output's QFN-76, 6 of 6 on
            #    qfn_diffpair_escape -- so the genuinely centred rung lands
            #    0.0125mm out, 12.5x POSITION_TOLERANCE. The via-in-pad branch
            #    was unreachable by construction on those boards.
            #  * an offset rung can put the barrel well inside the pad without
            #    the centre being on it at all.
            #
            # Either way the via shipped at NOMINAL size with no #202 clamp,
            # free to bulge past the pad -- while `print_via_in_pad_note` below
            # counted the very same via as needing IPC-4761 Type VII, because
            # it asks the fab question: does the BARREL overlap the copper. The
            # commit loop now calls that same predicate, on the pad this leg is
            # escaping (`via_in_pad_sites` scans every same-net pad and would
            # classify against a NEIGHBOUR's, then clamp to this one's).
            if via_overlaps_pad(pi.pad, vx, vy, via_size):
                vip_n += 1
                # via-in-pad: clamp to the pad edge so it never bulges past it (#202)
                # Pass the via's REAL position: this branch is reached for
                # OFF-CENTRE vias since #846, and a centred clamp does not
                # stop those bulging (#860 follow-up). Off-centre, the
                # honest verdict is usually 'floor' -- the barrel cannot be
                # both manufacturable and inside the pad at that offset.
                v_size, v_drill, status, rung = clamp_via_to_pad(
                    via_size, via_drill, pi.pad, floors, via_x=vx, via_y=vy)
                if status == 'clamped':
                    clamp_n += 1
                elif status == 'floor':
                    floor_n += 1
                    _floor_vias.append((v_size, v_drill))
                if rung > 0:
                    escalated_n += 1
                    _esc_vias.append((v_size, v_drill))
                if math.hypot(vx - px, vy - py) > POSITION_TOLERANCE:
                    offcentre_n += 1
            else:
                v_size, v_drill = via_size, via_drill   # off-pad via: not in a pad
            vias.append({'x': vx, 'y': vy, 'size': v_size, 'drill': v_drill,
                         'layers': ['F.Cu', 'B.Cu'], 'net_id': pi.pad.net_id})

    print(f"  Underpad via-drop: {len(vias)} vias placed, {len(dropped)} dropped "
          f"(pitch {pitch:.2f}, via {via_size:.2f}, stagger {stagger:.3f} mm"
          f"{', via-in-pad' if allow_via_in_pad else ''}"
          f"{f', {n_alt} side(s) used an alternative stagger' if n_alt else ''})")
    if dropped:
        print(f"    dropped (no clear via offset): {dropped}")
        if via_drill + _h2h > via_size + clearance:
            # #618's disclosure, for this escape: the drill rule, not the
            # copper one, set the via spacing that left these pads without one.
            print(f"    the {_h2h:g}mm hole-to-hole floor ({_h2h_src}) sets the"
                  f" via spacing here; the levers are a smaller --via-drill, a"
                  f" fab tier whose floor this pitch can meet, or the board's"
                  f" own min_hole_to_hole")
    if clamp_n:
        print(f"    clamped {clamp_n} via-in-pad(s) to fit their pad edge (#202)")
    # Cleared HERE and repopulated in the same breath, but the engine
    # entry point clears it too: a caller that runs two components in one
    # process must not read the first one's numbers for the second.
    LAST_UNDERPAD_REPORT.clear()
    LAST_UNDERPAD_REPORT.update({
        'via_in_pad': vip_n,
        'via_in_pad_offcentre': offcentre_n,
        'clamped': clamp_n,
        'max_stub_mm': round(max((math.hypot(t['end'][0] - t['start'][0],
                                             t['end'][1] - t['start'][1])
                                  for t in tracks), default=0.0), 4),
        'allow_via_in_pad': bool(allow_via_in_pad),
        'dropped_pads': list(dropped_pads),
        # #1804: per pad, not per net
        'pads_tried': len(pad_infos),
        'pads_reused': reused_n,
        'vias_placed': len(vias),
    })
    if offcentre_n:
        # Disclosed separately, not folded into the line above: these are the
        # vias #846 was about -- overlapping their pad while OFF its centre,
        # which the old test could not see. A reader has to be able to watch
        # this number move.
        print(f"    {offcentre_n} of them sit OFF the pad centre (#846); "
              f"before, those shipped unclamped")
    # The FAB requirement this escape may have just created (#489 §8). Emitted
    # from the shared engine path so the GUI fanout tab reports it too.
    # #962: DECLARED on each via-in-pad, (capping yes) (filling yes), not only
    # printed. The dicts carry it to the CLI writer and the GUI fanout tab.
    # `vias` are all this escape's own (empty input snapshot).
    from fab_notes import (via_protection_stamps, apply_stamps_in_memory,
                           print_via_protection_record)
    _st962, _rec962 = via_protection_stamps(vias, [], pcb_data)
    apply_stamps_in_memory(_st962)
    print_via_protection_record(_rec962, "QFN underpad escape")
    if escalated_n:
        warn_fab_escalation(f"{escalated_n} via-in-pad(s) (sub-0.45mm pads)",
                            vias=_esc_vias)
    if floor_n:
        # Two causes reach here and the message names both (#860 follow-up):
        # the pad really is smaller than anything manufacturable, OR the via
        # sits far enough OFF the pad centre that its remaining reach inside
        # the pad is -- `clamp_via_to_pad` measures min(sx-2|dx|, sy-2|dy|)
        # when it is given the position. Either way the barrel is held at the
        # floor and still crosses the pad edge, which is the honest statement;
        # saying "clamped to fit their pad edge" for these would not be.
        # #1579: the via actually HELD -- the deepest fab rung, or the request
        # itself when no rung is smaller (--escalation off, or a board ring
        # that raises every rung above it)
        _floor_dia = fab_floor_min(_copper)['via_diameter']
        _held = min(_floor_vias)[0] if _floor_vias else _floor_dia
        if abs(_held - _floor_dia) < 1e-9:
            print(f"    WARNING: {floor_n} via-in-pad(s) cannot fit their pad at "
                  f"the fab via floor ({_floor_dia:.2f}mm "
                  f"dia) -- pad smaller than the floor, or the via too far off its "
                  f"centre; via held at the floor and still bulges past the pad edge")
        else:
            print(f"    WARNING: {floor_n} via-in-pad(s) cannot fit their pad, and "
                  f"no via the escalation policy allows is smaller than the "
                  f"{_held:.2f}mm asked for -- via kept at {_held:.2f}mm and "
                  f"still bulges past the pad edge")
    return tracks, vias, dropped


#: A neighbour counts as a CHIP whose escapes this fanout must leave room for
#: when its own fanout would see at least this many perimeter pads -- a QFN,
#: QFP or two-row IC, not a crystal or a SOT.
NEIGHBOUR_MIN_PADS = 8


def _neighbour_escape_reserves(footprint, pcb_data, layer, extension, window):
    """The stubs each NEIGHBOURING chip's own fanout would lay, cut at the
    midline of any gap facing this part -- copper this part's fan must leave
    alone.

    Without this a stub's 45-degree fan grows until it grazes copper that
    already exists, so in the gap between two facing chips whichever is fanned
    FIRST takes the whole gap, and the second finds no room in front of its own
    pins. Measured on rein_r1 (two RP2040s, pin rows 1.7 mm apart): fanned
    first, U1's stubs ran to within 0.4 mm of U3's pins and boxed in U3's USB
    pair; fanned in the other order, the pair routed. With every chip honouring
    the others' reserve, each gets the half of a shared gap nearest it, in
    either order.

    For every other footprint the fanout would treat as a perimeter package
    (`analyze_qfn_layout`, at least `NEIGHBOUR_MIN_PADS` perimeter pads) near
    `window`, each of its pads on `layer` gets the stub `calculate_fanout_stub`
    would give it, clipped so it reaches no further along its escape than half
    the gap to the nearest pad of THIS part ahead of it. Returns
    `[((x1, y1), (x2, y2), net_id), ...]`.
    """
    from qfn_fanout.layout import analyze_qfn_layout as _lay, analyze_pad as _ap
    lo_x, lo_y, hi_x, hi_y = window
    own = [q for q in footprint.pads if getattr(q, 'pad_type', '') != 'np_thru_hole']

    def ext(q, ux, uy):
        hx, hy = (q.size_x or 0.0) / 2, (q.size_y or 0.0) / 2
        return abs(ux) * hx + abs(uy) * hy

    out = []
    for fp in pcb_data.footprints.values():
        if fp is footprint:
            continue
        pads = [q for q in fp.pads if q.net_id > 0]
        if not any(lo_x <= q.global_x <= hi_x and lo_y <= q.global_y <= hi_y
                   for q in pads):
            continue
        lay = _lay(fp)
        if lay is None:
            continue
        infos = [pi for pi in (_ap(q, lay) for q in pads) if pi.side != 'center']
        if len(infos) < NEIGHBOUR_MIN_PADS:
            continue
        maxd = max(lay.width, lay.height) / 3
        side_off: Dict[str, float] = {}
        for pi in infos:
            ex, ey = pi.escape_direction
            o = abs((pi.pad.global_x - lay.center_global_x) * -ey
                    + (pi.pad.global_y - lay.center_global_y) * ex)
            side_off[pi.side] = max(side_off.get(pi.side, 0.0), o)
        for pi in infos:
            q = pi.pad
            if pi.pad.drill <= 0 and layer not in (q.layers or []):
                continue
            corner, end = calculate_fanout_stub(
                pi, lay, pi.pad_width / 2 + extension, maxd, 0.0,
                angle_ref_off=side_off.get(pi.side, 0.0))
            ex, ey = pi.escape_direction
            tip = (q.global_x + ex * pi.pad_width / 2,
                   q.global_y + ey * pi.pad_width / 2)
            reach_lat = abs((end[0] - q.global_x) * -ey
                            + (end[1] - q.global_y) * ex) + pi.pad_length / 2
            gap = None
            for a in own:
                s = (a.global_x - tip[0]) * ex + (a.global_y - tip[1]) * ey
                lat = abs((a.global_x - tip[0]) * -ey + (a.global_y - tip[1]) * ex)
                d = s - ext(a, ex, ey)
                if d > 0 and lat <= reach_lat + ext(a, -ey, ex) and (
                        gap is None or d < gap):
                    gap = d
            pts = [(q.global_x, q.global_y), corner, end]
            if gap is not None:
                lim = gap / 2

                def along(pt):
                    return (pt[0] - tip[0]) * ex + (pt[1] - tip[1]) * ey
                clipped = [pts[0]]
                for a, b in zip(pts, pts[1:]):
                    sa, sb = along(a), along(b)
                    if sb <= lim:
                        clipped.append(b)
                        continue
                    if sa < lim and sb > sa:
                        f = (lim - sa) / (sb - sa)
                        clipped.append((a[0] + (b[0] - a[0]) * f,
                                        a[1] + (b[1] - a[1]) * f))
                    break
                pts = clipped
            for a, b in zip(pts, pts[1:]):
                if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6:
                    out.append((a, b, q.net_id))
    return out


def resolve_escape_layer(footprint: Footprint, layer: Optional[str] = None,
                         ref: Optional[str] = None
                         ) -> Tuple[str, Optional[str], bool]:
    """The layer a QFN fanout's stubs go on, and what to say about it.

    `layer` None (qfn_fanout.py with no --layer, the QFN panel's "Mounted
    layer") is the part's MOUNTED layer (issue #96: B.Cu-mounted parts silently
    got F.Cu stubs floating over their pads, and route.py then reported their
    nets routed while electrically open). Any other layer is honoured, with a
    warning when it is not the mounted one. Both fronts call this, so the
    escape layer and the #96 warning cannot drift apart.

    Returns (layer, message, is_warning); message is None when there is
    nothing to say.
    """
    ref = ref or footprint.reference
    if layer is None:
        layer = footprint.layer or 'F.Cu'
        return layer, f"  Layer: {layer} (from footprint)", False
    if footprint.layer and layer != footprint.layer:
        return layer, (f"  WARNING: --layer {layer} differs from {ref}'s "
                       f"mounted layer {footprint.layer} - stubs will NOT "
                       f"touch the SMD pads unless this is intentional"), True
    return layer, None, False


def _stub_legs(stub):
    """(index, start, end) of a stub's legs: 0 pad -> corner, 1 corner -> tip,
    2 tip -> join_end (#1332); a zero-length leg is not copper."""
    legs = []
    for k, a, b in ((0, stub.pad_pos, stub.corner_pos),
                    (1, stub.corner_pos, stub.stub_end),
                    (2, stub.stub_end, stub.join_end)):
        if b is None:
            continue
        if abs(b[0] - a[0]) > POSITION_TOLERANCE or abs(b[1] - a[1]) > POSITION_TOLERANCE:
            legs.append((k, a, b))
    return legs


def _widen_power_stub_legs(stubs, power_w, track_width, clearance, seg_grazes,
                           hits_reserve, pair_clearance=None):
    """#1318: per-leg widths ``{id(stub): (w0, w1, w2)}`` for the stubs of
    power nets.

    Each stub was laid and cleared at `track_width`. A power net's legs are
    then tried, stub by stub, at the net's width -- only where the pad's
    short side admits it: a stub narrower than the requested width ships
    under width either way, and widening it only takes room from the pins
    around it. A leg keeps that width only if it clears everything the stub
    test checks at that width (`seg_grazes(.., width=)`: board edge, the stub map stamped
    for that track, every pad exactly, the input copper of the fanned nets),
    a neighbouring chip's reserved escape room, and every other net's stub as
    laid so far, at their own widths (stubs are never checked against each
    other at `track_width`: they are spaced by construction, and a wider
    one is not). Otherwise it stays at `track_width` -- narrow only where the
    power width does not fit, as route.py's pad-entry zone is.

    ``pair_clearance(net_a, net_b)``, when given, is the clearance between
    two stubs' nets (their classes, #1529); else every pair is `clearance`."""
    from geometry_utils import segment_to_segment_distance
    cur = {id(s): [track_width] * 3 for s in stubs}
    legs = {id(s): _stub_legs(s) for s in stubs}
    for stub in stubs:
        w_net = power_w.get(stub.net_id)
        if not w_net:
            continue
        w = w_net
        if w <= track_width + 1e-9 or \
                min(stub.pad.size_x, stub.pad.size_y) < w - 1e-9:
            continue
        for k, a, b in legs[id(stub)]:
            if seg_grazes(a, b, stub.net_id, width=w) or \
                    hits_reserve(a, b, stub.net_id, width=w):
                continue
            ok = True
            for other in stubs:
                if other.net_id == stub.net_id or other.layer != stub.layer:
                    continue
                _cl = (pair_clearance(stub.net_id, other.net_id)
                       if pair_clearance is not None else clearance)
                for ko, oa, ob in legs[id(other)]:
                    if segment_to_segment_distance(a[0], a[1], b[0], b[1],
                                                   oa[0], oa[1], ob[0], ob[1]) \
                            < w / 2 + cur[id(other)][ko] / 2 + _cl - 1e-9:
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                cur[id(stub)][k] = w
    return {k: tuple(v) for k, v in cur.items()}


def generate_qfn_fanout(footprint: Footprint,
                        pcb_data: PCBData,
                        net_filter: Optional[List[str]] = None,
                        layer: str = "F.Cu",
                        track_width: float = 0.1,
                        extension: float = 0.1,
                        clearance: float = 0.1,
                        grid_step: float = 0.0,
                        escape_method: str = "stub",
                        via_size: float = 0.45,
                        via_drill: float = 0.25,
                        allow_via_in_pad: bool = False,
                        board_edge_clearance: float = 0.0,
                        # #581: > 0 forbids via-in-pad (overrides
                        # allow_via_in_pad); None auto-reads the .kicad_pro
                        # record a chain step persisted.
                        same_net_pad_clearance: Optional[float] = None,
                        # progress_callback(current, total, label); the fanout
                        # tab otherwise shows one static label for the run.
                        progress_callback=None,
                        cancel_check=None,
                        # route.py's --power-nets / --power-nets-widths: a
                        # power net's stub is laid at its width where it fits.
                        power_nets: Optional[List[str]] = None,
                        power_nets_widths: Optional[List[float]] = None,
                        # #1529: {net_id: class clearance} (None = the
                        # board's .kicad_pro), capped at clearance_ceiling.
                        net_clearances: Optional[Dict[int, float]] = None,
                        clearance_ceiling: Optional[float] = None,
                        # #1795: the via was not asked for (the board's
                        # Default class via, or the fallback), so the
                        # underpad ladder may step it down.
                        via_from_class: bool = False,
                        ) -> Tuple[List[Dict], List[Dict], List[str]]:
    """
    Generate QFN fanout tracks for a footprint.

    Creates two-segment stubs:
    1. Straight segment perpendicular to chip edge
    2. 45 degree segment fanning outward from center

    Edge pads get short straight (just past pad) + long 45 degree for maximum fan.
    Center (interior/EP) pads have no edge for the surface fan; on a
    net-scoped run they escape by the underpad via-drop instead (issue #410;
    #1717: a stub run via-drops them after its surface fan). An unscoped run
    never via-drops them.

    Args:
        footprint: The QFN/QFP footprint
        pcb_data: Full PCB data
        net_filter: Optional list of net patterns to include
        layer: Routing layer (default F.Cu)
        track_width: Width of fanout tracks
        extension: Extension past pad edge before bend (mm)
        power_nets / power_nets_widths: glob patterns and widths, read as
            route.py reads them (identify_power_nets, floored at track_width).
            A surface stub of such a net, on a pad at least that wide, is
            laid leg by leg at the net's width wherever that clears
            everything the stub test checks plus the run's other stubs, and
            at track_width where it does not (#1318: route.py keeps input
            copper, so a stub laid narrow shipped narrow).
        net_clearances / clearance_ceiling: the net classes, read as route.py
            reads them (:func:`resolve_class_clearances`). Every pair the
            escape tests -- stub or via against foreign copper, a pad, another
            fanned net's copper, a neighbouring chip's reserved escape, this
            run's own stubs -- is priced at KiCad's max(clearance, class A,
            class B), and no stub escalates below its own class (#1529).
        via_from_class: the via was not asked for -- the CLI was given no
            --via-size/--via-drill, the GUI's via override is unticked -- so
            it is the board's Default class via (#1795). When the underpad
            via-drop still drops pads at every rung of its clearance ladder,
            it then steps the via down the fab via rungs (#1328's ladder
            again at each), as a class via too large for the part's pitch
            would otherwise drop pads a smaller via escapes. An asked-for
            via is drawn as asked.

    Returns:
        (tracks, vias, failed_nets) - tracks are the segments, vias is empty.
        failed_nets is the deduplicated list of net names whose stubs landed
        too close to another net's stub (endpoint spacing < track_width +
        extension); those tracks are still emitted but flagged as failing
        clearance so the GUI can surface them.

    Cancellation (#621): `cancel_check` is the standard zero-arg cooperative
    predicate (`batch_route` / `create_plane` take the same one), honoured at
    the head of the escape work and at the head of the per-stub clearance loop
    -- the loop that actually costs the time (every stub is checked against the
    obstacle map, every foreign pad and the board edge, then shortened by
    search). It BREAKS the loop, never raises: an exception here dies in this
    package's `except Exception` swallowers. Stubs already kept ship their
    tracks; pads it never reached carry no copper and are NOT added to
    failed_nets -- an unfinished search has measured nothing. The untried pads'
    nets are published as qfn_fanout.LAST_CANCEL_SKIPPED. Passing None (the
    default, and what the CLI passes) is fully inert.
    """
    # #1676: the rungs below start at the board's own via rules
    from design_rules import install_board_via_rule_floors
    install_board_via_rule_floors(pcb_data)
    LAST_CANCEL_SKIPPED.clear()
    LAST_ESCALATED.clear()          # #1328
    LAST_FILTER_MISSES.clear()      # #1641
    LAST_SKIPPED_INTERIOR.clear()   # #1717
    LAST_UNDERPAD_REPORT.clear()    # #1804: a run with no via-drop reports none
    LAST_STUB_REPORT.clear()        # #1834: nor one with no surface fan
    LAST_POWER_STUBS_AT_WIDTH.clear()   # #1743
    LAST_BOARD_SETUP_VIAS.clear()   # #1775
    _RULES_SAID.clear()             # #1565: said once per call
    # #619: and the erased-set report, for the same reason. `generate_qfn_fanout`
    # returns early on several paths that never reach `_underpad_via_escape`
    # (no recognised layout, no pad_infos, a cancel), and the surface fan never
    # reaches it at all -- so without this a later reader gets the PREVIOUS
    # footprint's set, from a different board. Measured over the tracked corpus:
    # 155 of 408 footprints with >=4 pads never enter the escape, and every one
    # of them was being graded against whatever ran before it.
    global LAST_ERASED_SETS
    LAST_ERASED_SETS = {}
    _cancelled = [False]

    def _cancel() -> bool:
        if cancel_check is not None and cancel_check():
            _cancelled[0] = True
            return True
        return False
    layout = analyze_qfn_layout(footprint)
    if layout is None:
        _why = single_line_reason(footprint)
        print(f"Warning: {footprint.reference} doesn't appear to be a QFN/QFP"
              + (f": {_why}" if _why else ""))
        return [], [], []

    # #581: an active (> 0) same-net pad via clearance forbids via-in-pad --
    # it overrides an explicit allow_via_in_pad.
    if same_net_pad_clearance is None:
        from protected_nets import read_snpc_for_pcb_data as _read_snpc581
        same_net_pad_clearance = _read_snpc581(pcb_data)
    if same_net_pad_clearance > 0 and allow_via_in_pad:
        print(f"  Same-net pad via clearance {same_net_pad_clearance:g}mm "
              f"(#581): via-in-pad DISABLED (overrides allow-via-in-pad)")
        allow_via_in_pad = False

    # #498: a .kicad_dru rule for the (single) escape layer REPLACES the pair
    # clearance -- QFN stubs live on exactly one layer, so the scalar swap is
    # exact (tighten or relax). The obstacle maps below get the full per-layer
    # map installed separately for their stack-aware via keep-outs.
    from kicad_dru import board_layer_clearance_map
    _lcl_498 = board_layer_clearance_map(pcb_data)
    if _lcl_498.get(layer) is not None and _lcl_498[layer] != clearance:
        print(f"  .kicad_dru: escape-layer clearance {layer} "
              f"{clearance} -> {_lcl_498[layer]} (#498)")
        clearance = _lcl_498[layer]

    # #1529: the net classes, priced pairwise below. Escaping every pair at the
    # one run clearance (the Default class) put 0.2-class stubs 0.14-0.19 mm
    # apart, which the writeback then hid by lowering every class to it.
    _nc, _nc_lines = resolve_class_clearances(pcb_data, net_clearances,
                                              clearance_ceiling)
    for _l in _nc_lines:
        print(f"  {_l}")

    # Fab-floor clamp (issue #223): an escape stub thinner than the board's
    # minimum manufacturable track width is un-routable at the stated fab class
    # (usb_sniffer's /T_USB_* bus emitted at 0.100mm against a 0.127mm 2-layer
    # floor -> a whole bus of TRACK-WIDTH violations). The width is a parameter,
    # not a search outcome, so clamp it up to the fab floor here -- mirroring the
    # clamp route.py / check_drc.py already apply.
    from list_nets import fab_floors
    ncu = len(pcb_data.board_info.copper_layers) if pcb_data.board_info.copper_layers else 2
    fab_min_track = fab_floors(ncu)['track_width']
    if track_width < fab_min_track - 1e-9:
        print(f"  Track width {track_width:.4f}mm is below the {ncu}-layer fab "
              f"floor {fab_min_track:.4f}mm - clamping escape stubs up (issue #223)")
        track_width = fab_min_track

    # Sanity-check pad geometry before escaping: overlapping same-footprint pads
    # mean the pad rotation/size is modelled wrong, so the stubs would be placed
    # across neighbouring pads (issue: rotated-package fanout). Warn loudly.
    from check_pads import find_pad_overlaps
    try:
        _ov = find_pad_overlaps(pcb_data, component=footprint.reference)
    except Exception as _exc:                                # noqa: BLE001
        # #1111: the check measures a custom pad's copper with shapely now;
        # a geometry failure on one odd pad must not abort a fanout over
        # what is only a warning.
        print(f"  WARNING: the pad-geometry check for {footprint.reference} "
              f"could not run ({type(_exc).__name__}: {_exc})")
        _ov = []
    if _ov:
        print(f"  WARNING: {footprint.reference} has {len(_ov)} overlapping "
              f"different-net pad pair(s) - pad geometry looks wrong, fanout "
              f"stubs may cross pads. Run: python3 py_router/check_pads.py <board> "
              f"--component {footprint.reference}")

    print(f"QFN/QFP Layout Analysis for {footprint.reference}:")
    print(f"  Center: ({layout.center_x:.2f}, {layout.center_y:.2f})")
    print(f"  Bounding box: X[{layout.min_x:.2f}, {layout.max_x:.2f}], Y[{layout.min_y:.2f}, {layout.max_y:.2f}]")
    print(f"  Size: {layout.width:.2f} x {layout.height:.2f} mm")
    print(f"  Detected pad pitch: {layout.pad_pitch:.2f} mm")
    print(f"  Edge tolerance: {layout.edge_tolerance:.2f} mm")
    print(f"  Stub length: pad_width / 2 + extension (clears pad before bend)")
    print(f"  Layer: {layer}")

    # Analyze all pads
    pad_infos: List[PadInfo] = []
    interior_infos: List[PadInfo] = []      # #1717: a stub run's via-drops
    side_counts = defaultdict(int)

    if progress_callback:
        progress_callback(
            0, 0, f"QFN fanout {getattr(footprint, 'reference', '?')}: "
                  f"analyzing {len(footprint.pads)} pad(s)...")

    # #1342: a net with one pad board-wide (a spare / NC pin the schematic
    # still names) has nothing to connect to, so its escape is copper that
    # can only block routing. bga_fanout has skipped these since #122; the
    # QFN fan used to escape them, and on a CM5 module 33 of 70 new vias were
    # on such nets. Same predicate, same disclosure.
    nc_net_ids = single_pad_net_ids(footprint, pcb_data)
    if nc_net_ids:
        print(f"  Skipping {len(nc_net_ids)} single-pad/NC net(s) "
              f"(nothing to connect to)")

    # #1641: a pattern that spells a board net verbatim selects that net
    # alone, so a bus member 'D[3]' fans out itself rather than 'D3'.
    _user_filter = net_filter
    net_filter = literal_net_patterns(net_filter, board_net_names(pcb_data))

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        # Skip unconnected nets (KiCad pins not connected in schematic)
        if pad.net_name.lower().startswith('unconnected-'):
            continue

        if pad.net_id in nc_net_ids:
            continue

        if net_filter and not matches_net_filter(pad.net_name, net_filter):
            continue

        pad_info = analyze_pad(pad, layout)
        if pad_info.side == 'center':
            # Interior/EP pads have no free surface edge for the 45-deg fan,
            # but a via-drop straight down doesn't need one (issue #410). They
            # are escaped ONLY on a net-scoped run: requiring --nets is a
            # deliberate safety guard so an unscoped run never via-drops the
            # exposed/thermal pad.
            if not net_filter:
                continue
            side_counts[pad_info.side] += 1
            if escape_method != "underpad":
                # #1717: a stub run via-drops the interior pads its filter
                # selected, after the surface fan (_serve_interior below).
                interior_infos.append(pad_info)
                continue

        pad_infos.append(pad_info)
        if pad_info.side != 'center':
            side_counts[pad_info.side] += 1

    print(f"  Found {len(pad_infos) + len(interior_infos)} pads to fanout:")
    for side, count in sorted(side_counts.items()):
        print(f"    {side}: {count} pads")
    # #1641: each --nets pattern that selected no pad, and why.
    LAST_FILTER_MISSES.extend(net_filter_misses(footprint, pcb_data, _user_filter))
    for _line in filter_miss_lines(LAST_FILTER_MISSES):
        print(_line)

    # Show sample pad geometry
    if pad_infos:
        sample = pad_infos[0]
        print(f"  Sample pad geometry: {sample.pad_length:.2f} x {sample.pad_width:.2f} mm")

    if not pad_infos and not interior_infos:
        return [], [], []

    def _record_skipped(tracks_, vias_, failed_):
        """#621: the untried complement -- a candidate pad with no copper from
        this call and no entry in failed_. Only built when a cancel fired."""
        if not _cancelled[0]:
            return
        live = ({t.get('net_id') for t in tracks_}
                | {v.get('net_id') for v in vias_})
        done = set(failed_)
        LAST_CANCEL_SKIPPED.extend(sorted(
            {pi.pad.net_name for pi in pad_infos
             if pi.pad.net_name and pi.pad.net_id not in live
             and pi.pad.net_name not in done}))

    # #621 escape-work head: covers BOTH escape methods, so a cancel raised
    # before the first pad stops here with nothing tried instead of running an
    # unbounded escape.
    if _cancel():
        _record_skipped([], [], [])
        return [], [], []

    # Via-drop / underpad escape (issue #164): drop a through-via just past each
    # pad and let signal routing pick the net up on an inner layer, for crowded
    # fine-pitch edges where the surface fan has no room.
    def _via_drop(infos, board=pcb_data):
        """The underpad via-drop of INFOS, down #1328's clearance ladder.
        BOARD is the copper it clears (#1717: with the surface fan's stubs on
        it). Returns (tracks, vias, dropped net names).

        #1804: a pad that already has its via escape (pad_has_via_escape --
        a re-run on this step's own output, or a retry after a dropped pad)
        is left out, so the run drills only for the pads still unserved."""
        _cu = list(board.board_info.copper_layers or [layer])
        _nets = {pi.pad.net_id for pi in infos}
        _segs = defaultdict(list)
        for s in board.segments:
            if s.net_id in _nets:
                _segs[s.net_id].append(s)
        _vias = defaultdict(list)
        for v in board.vias:
            if v.net_id in _nets:
                _vias[v.net_id].append(v)
        served = [pi for pi in infos if pad_has_via_escape(
            pi.pad, _segs[pi.pad.net_id], _vias[pi.pad.net_id], layer, _cu)]
        if served:
            _sid = {id(pi) for pi in served}
            infos = [pi for pi in infos if id(pi) not in _sid]
            print(f"  {len(served)} pad(s) already have a via escape, skipped "
                  f"(#1804); {len(infos)} left to via-drop")

        def _with_served(res_):
            LAST_UNDERPAD_REPORT['already_served_pads'] = [
                (pi.pad.pad_number, pi.pad.net_name) for pi in served]
            return res_
        if not infos:
            LAST_UNDERPAD_REPORT.clear()
            LAST_UNDERPAD_REPORT.update({'pads_tried': 0, 'pads_reused': 0,
                                         'vias_placed': 0, 'dropped_pads': []})
            return _with_served(([], [], []))

        def _drop_run(cl, vs=via_size, vd=via_drill):
            return _underpad_via_escape(footprint, board, infos, layout, layer,
                                        track_width, cl, vs, vd, grid_step,
                                        allow_via_in_pad=allow_via_in_pad,
                                        board_edge_clearance=board_edge_clearance,
                                        progress_callback=progress_callback,
                                        net_clearances=_nc, fanned_net_ids=_nets,
                                        same_net_pad_clearance=same_net_pad_clearance or 0.0)
        res = _drop_run(clearance)
        res_pads = list(LAST_UNDERPAD_REPORT.get('dropped_pads', ()))
        # #1328: the underpad escape escalates as the surface stub does. Its
        # vias are placed greedily per side against each other, so a dropped
        # pad cannot be retried alone: the whole part is re-run down
        # qfn_clearance_ladder, stopping at the first rung that drops nothing,
        # and the run that drops fewest (the higher clearance on a tie) is
        # kept. Every via of that run was placed at its rung, so the rung is
        # the floor recorded. A .kicad_dru rule on the layer does not escalate.
        _ladder = ([] if _lcl_498.get(layer) is not None else
                   qfn_clearance_ladder(clearance,
                                        len(pcb_data.board_info.copper_layers or ()) or 2))
        # #1795: a via nobody asked for may step down the fab via rungs when
        # every clearance rung still drops pads, each rung re-running the
        # clearance ladder; the run that drops fewest is kept (the larger via,
        # then the higher clearance, on a tie).
        _vias = [(via_size, via_drill)]
        if via_from_class:
            from fab_tiers import escalation_rungs as _vrungs
            for _r in _vrungs(len(pcb_data.board_info.copper_layers or ()) or 2):
                _vd, _vh = _r.get('via_diameter'), _r.get('via_drill')
                if (_vd and _vh and _vd < _vias[-1][0] - 1e-9
                        and _vh < _vd - 1e-9):
                    _vias.append((_vd, _vh))
        best_via = (via_size, via_drill)
        if res[2] and (_ladder or len(_vias) > 1):
            best, best_cl, last = res, clearance, (clearance, via_size, via_drill)
            for vi, (vs, vd) in enumerate(_vias):
                for cl in ([] if vi == 0 else [clearance]) + list(_ladder):
                    r = _drop_run(cl, vs, vd)
                    last = (cl, vs, vd)
                    if len(r[2]) < len(best[2]):
                        best, best_cl, best_via = r, cl, (vs, vd)
                    if not r[2]:
                        break
                if not best[2]:
                    break
            if (best_cl, best_via[0], best_via[1]) != last:
                best = _drop_run(best_cl, *best_via)  # LAST_UNDERPAD_REPORT is per run
            if best_via != (via_size, via_drill):
                from fab_tiers import note_narrowing
                for _v in best[1]:
                    note_narrowing(_v.get('net_id'), 'via_diameter', via_size,
                                   _v.get('size', best_via[0]), 'qfn underpad escape')
                print(f"  Via escalation (#1795): underpad escape still dropped pads at the "
                      f"class via {via_size:g}/{via_drill:g} mm at every clearance; at "
                      f"{best_via[0]:g}/{best_via[1]:g} mm it drops {len(best[2])} "
                      f"(at {len(res[2])} before)")
            if best_cl < clearance - 1e-9:
                from fab_tiers import note_narrowing, warn_fab_escalation, escalation_rungs
                import clearance_ledger
                rungs = escalation_rungs(len(pcb_data.board_info.copper_layers or ()) or 2)
                nominal = rungs[0]['clearance'] if rungs else None
                # By PAD, not net name: a two-pad net that escapes one of
                # its pads at the rung is still in the dropped names.
                from collections import Counter
                rescued = list((Counter(res_pads) - Counter(
                    LAST_UNDERPAD_REPORT.get('dropped_pads', ()))).elements())
                by_pad = {(p.pad_number, p.net_name): p for p in footprint.pads}
                for num, name in rescued:
                    pad = by_pad.get((num, name))
                    LAST_ESCALATED.append({'net': name,
                                           'pad': f"{footprint.reference}.{num}",
                                           'clearance': round(best_cl, 4)})
                    note_narrowing(pad.net_id if pad else None, 'clearance', clearance,
                                   best_cl, 'qfn underpad escape', net_name=name)
                clearance_ledger.record(best_cl)
                if nominal is not None and best_cl < nominal - 1e-9:
                    warn_fab_escalation(f"qfn underpad escape {footprint.reference}")
                print(f"  Clearance escalation (#1328): underpad escape dropped "
                      f"{len(res[2])} pad(s) at {clearance:g} mm; re-run at "
                      f"{best_cl:g} mm escapes {len(rescued)} of them "
                      f"({len(best[2])} still dropped)")
            res = best
        # #1775: every via this fanout lays comes through here
        from design_rules import disclose_board_setup_vias
        LAST_BOARD_SETUP_VIAS.extend(disclose_board_setup_vias(pcb_data, res[1]))
        return _with_served(res)

    if escape_method == "underpad":
        print(f"  Escape method: underpad (via-drop), via {via_size:.2f}/{via_drill:.2f} mm"
              f"{', allow via-in-pad' if allow_via_in_pad else ''}")
        res = _via_drop(pad_infos)
        _note_power_stubs_at_width(                                 # #1743
            footprint, pcb_data, res[0], [(pi, 'via-drop') for pi in pad_infos],
            track_width, power_nets, power_nets_widths)
        return res

    def _serve_interior(tracks_, failed_):
        """#1717: the interior pads this stub run's filter selected get the
        underpad via-drop, clearing the surface fan's stubs as well as the
        board. A pad it drops is named in LAST_SKIPPED_INTERIOR and its net
        is failed, as an underpad run fails it. (tracks, vias, failed)."""
        if not interior_infos:
            _record_skipped(tracks_, [], failed_)
            _note_power_stubs(tracks_)
            return tracks_, [], failed_
        print(f"  Interior pads: {len(interior_infos)} selected, escaped by "
              f"via-drop (underpad), via {via_size:.2f}/{via_drill:.2f} mm"
              f"{', allow via-in-pad' if allow_via_in_pad else ''}")
        import copy
        from kicad_parser import Segment
        board = copy.copy(pcb_data)
        board.segments = list(pcb_data.segments) + [
            Segment(start_x=t['start'][0], start_y=t['start'][1],
                    end_x=t['end'][0], end_y=t['end'][1], width=t['width'],
                    layer=t['layer'], net_id=t['net_id']) for t in tracks_]
        itracks, ivias, idropped = _via_drop(interior_infos, board)
        _ref = getattr(footprint, 'reference', '?')
        LAST_SKIPPED_INTERIOR.extend(
            f"{_ref}.{num}" for num, _name in
            LAST_UNDERPAD_REPORT.get('dropped_pads', ()))
        if LAST_SKIPPED_INTERIOR:
            print(f"  {skipped_interior_line(LAST_SKIPPED_INTERIOR)}")
        out = (tracks_ + itracks, ivias,
               list(dict.fromkeys(failed_ + list(idropped))))
        _record_skipped(*out)
        _note_power_stubs(out[0])
        return out

    def _note_power_stubs(tracks_):
        """#1743: the surface fan's and the via-drop's power stubs drawn at
        the track width."""
        _note_power_stubs_at_width(
            footprint, pcb_data, tracks_,
            [(pi, 'surface') for pi in pad_infos]
            + [(pi, 'via-drop') for pi in interior_infos],
            track_width, power_nets, power_nets_widths)

    if not pad_infos:
        return _serve_interior([], [])

    # Build stubs
    stubs: List[FanoutStub] = []

    # Max diagonal length for corner pads = chip_width / 3
    max_diagonal_length = max(layout.width, layout.height) / 3

    # Per-side outermost pad offset (issue #200): normalizes the fan-angle ramp
    # so the corner pad on each side reaches a true 45 deg (the last pad is
    # inset from the bbox corner, so a bbox-half normalization would stop short).
    side_max_off: Dict[str, float] = {}
    for pi in pad_infos:
        ex, ey = pi.escape_direction
        tan_x, tan_y = -ey, ex
        off = abs((pi.pad.global_x - layout.center_global_x) * tan_x +
                  (pi.pad.global_y - layout.center_global_y) * tan_y)
        side_max_off[pi.side] = max(side_max_off.get(pi.side, 0.0), off)

    # #1834: a pad that already has its surface escape (pad_has_stub_escape --
    # a re-run on this step's own output, a retry with other parameters) is
    # left out, as the via-drop leaves out a pad with its via (#1804), so the
    # run lays copper only for the pads still unserved. Its existing stub
    # stays on the board, where every new stub of another net clears it
    # (`_fanned_existing_segs`, #257). The fan angles above are ramped over
    # every selected pad, served or not, so a pad still to escape gets the
    # stub a first run would have given it.
    _hull = part_pad_hull(footprint)
    _cu_1834 = list(pcb_data.board_info.copper_layers or [layer])
    _fan_nets = {pi.pad.net_id for pi in pad_infos}
    _net_segs, _net_vias = defaultdict(list), defaultdict(list)
    for s in pcb_data.segments:
        if s.net_id in _fan_nets:
            _net_segs[s.net_id].append(s)
    for v in pcb_data.vias:
        if v.net_id in _fan_nets:
            _net_vias[v.net_id].append(v)
    _stub_served = [pi for pi in pad_infos if pad_has_stub_escape(
        pi.pad, _net_segs[pi.pad.net_id], _net_vias[pi.pad.net_id], _hull,
        layer, _cu_1834)]
    LAST_STUB_REPORT.update({'pads_tried': 0, 'dropped_pads': [],
                             'already_served_pads': [
                                 (pi.pad.pad_number, pi.pad.net_name)
                                 for pi in _stub_served]})
    if _stub_served:
        _sid = {id(pi) for pi in _stub_served}
        pad_infos = [pi for pi in pad_infos if id(pi) not in _sid]
        print(f"  {len(_stub_served)} pad(s) already have their surface escape, "
              f"skipped (#1834); {len(pad_infos)} left to fan")
        if not pad_infos:
            return _serve_interior([], [])

    for pad_info in pad_infos:
        # Straight stub length = pad_width / 2 + extension (to clear the pad before bending)
        # pad_width is the dimension perpendicular to the chip edge (escape direction)
        straight_length = pad_info.pad_width / 2 + extension
        corner_pos, stub_end = calculate_fanout_stub(
            pad_info, layout, straight_length, max_diagonal_length, grid_step,
            angle_ref_off=side_max_off.get(pad_info.side, 0.0)
        )

        stub = FanoutStub(
            pad=pad_info.pad,
            pad_pos=(pad_info.pad.global_x, pad_info.pad.global_y),
            corner_pos=corner_pos,
            stub_end=stub_end,
            side=pad_info.side,
            layer=layer
        )
        stubs.append(stub)

    # Foreign-pad clearance (issue #123). The QFN fan emits each stub blind to
    # other components' pads, so a stub on a fine-pitch part routed out toward a
    # neighbouring passive grazes it within clearance (PAD-SEGMENT). Mirror the
    # bga_fanout escape clearing: shorten the 45 fan inward to clear the pad
    # (connectivity-neutral - the straight segment still escapes the chip), and
    # if even the straight escape itself grazes a pad, drop that stub and warn.
    from obstacle_map import build_base_obstacle_map, build_layer_map, check_line_clearance
    from routing_config import GridRouteConfig

    # Full obstacle map so a stub is checked against foreign TRACKS and VIAS too,
    # not just foreign pads (issue #149 part 2). The part's own nets are excluded
    # (nets_to_route) so same-net taps stay legal; extra_clearance = track half so
    # the cell test is a clearance test. Checked at stub-creation time (below), so
    # a stub/fan that would extend into a foreign obstacle is shortened or dropped.
    #
    # Pad COPPER is left off this map (stamp_pads=False): every pad near the
    # stubs is measured exactly in _seg_grazes instead. The stamps already
    # carry the track half-width, so extra_clearance counts it twice -- the
    # guard check_line_clearance's nearest-cell sampling needs (0.07 mm at this
    # 0.1 mm grid) -- and a pad stamped that way refused fans that clear it:
    # uhk_trackball's /NCS fan was cut back at a tip 0.188 mm from R2.1 with
    # 0.175 needed. Tracks, vias, holes and keep-outs stay on the map.
    _obs_cfg = GridRouteConfig(layers=list(pcb_data.board_info.copper_layers or [layer]),
                               track_width=track_width, clearance=clearance)
    from kicad_dru import install_layer_clearances
    install_layer_clearances(_obs_cfg, None, None, pcb_data)  # #498
    _obs_cfg.set_net_clearances(_nc, ())                      # #1529
    _obs_layer_map = build_layer_map(_obs_cfg.layers)
    _fanned_net_ids = [p.net_id for p in footprint.pads if p.net_id]
    # #1529: the map stamps each FOREIGN obstacle at max(floor, its class);
    # the floor is the stub's own pair side (`_stub_floor`), one map per
    # value. The fanned nets are not stamped, so their classes stay out of
    # the map's floor, where they would over-block every other stub.
    _fanned_set = set(_fanned_net_ids)
    _nc_foreign = {n: c for n, c in _nc.items() if n not in _fanned_set}
    # #1565: the class/type-scoped (#1491) and track-scoped (#735) .kicad_dru
    # rules -- the map's stamps, and `_rules` for the pairs priced below
    from kicad_dru import install_pair_rule_channels
    _rules = install_pair_rule_channels(_obs_cfg, pcb_data, _fanned_net_ids)
    _say_rules(_rules, footprint, pcb_data)
    _priced = bool(_nc) or _rules.live

    def _pair(cl, net_a, net_b, kind='layer', types=None):
        """#1529: the clearance between this run's copper of `net_a` and
        copper of `net_b` on the escape layer, from the run's clearance `cl`:
        KiCad's max(cl, class A, class B), which a .kicad_dru rule on that
        layer replaces, and a scoped or track rule (#1565) for the items'
        types (`kind` 'track' for two tracks). `cl` itself on a board with
        no non-Default class and no such rule."""
        if not _priced:
            return cl
        return _rules(net_a, net_b, layer, base=cl, kind=kind, types=types)

    def _stub_floor(cl, net_id):
        """The routing side of every pair a stub of `net_id` makes at `cl`:
        its own class raises it, unless the escape layer's rule replaces it."""
        if not _nc or _lcl_498.get(layer) is not None:
            return cl
        c = _nc.get(net_id)
        return c if c is not None and c > cl else cl
    if progress_callback:
        progress_callback(0, 0, "QFN fanout: building obstacle map...")
    _obstacles = build_base_obstacle_map(pcb_data, _obs_cfg, nets_to_route=_fanned_net_ids,
                                         extra_clearance=track_width / 2,
                                         net_clearances=_nc_foreign,
                                         progress_callback=progress_callback,
                                         stamp_pads=False)
    _obs_layer_idx = _obs_layer_map.get(layer)
    # Window the foreign-pad scan by the actual STUB extent (every stub endpoint),
    # not the part's pad bbox: on large packages (LQFP) a straight escape runs
    # several mm past the pad bbox and grazes a nearby foreign component pad (a
    # connector/header) that a pad-bbox+3mm window misses -- and the grid obstacle
    # map is too coarse to catch a sub-grid graze (issue #214). Each pad is
    # included if its OWN footprint can reach the stub span within `_pad_reach`, so
    # the window is correct for large pads too.
    #
    # The part's OWN pads are included too (issue #356): the obstacle map excludes
    # every fanned net, and this scan used to exclude the whole footprint, so an
    # escape threading a dense pad field (AQFN inner grid, hex_gateway U1) shipped
    # 0.006mm from a NEIGHBOUR pad of the same part with no check at all. The
    # per-stub `pad.net_id == net_id` skip below already exempts the stub's own
    # pad(s); every other-net pad -- own part or not -- is real foreign copper.
    from check_drc import segment_pad_gap as _segment_pad_gap, _pad_has_no_copper
    from net_queries import expand_pad_layers
    _base_cl = _obs_cfg.layer_clearance(layer, clearance)

    def _pad_need(pad, cl=clearance):
        base = _base_cl if cl == clearance else _obs_cfg.layer_clearance(layer, cl)
        return max(cl, _obs_cfg.pad_override_clearance(base, pad))
    # The pad window must reach the widest clearance any pad is held to.
    _power_w = {}
    if power_nets:
        from net_queries import identify_power_nets
        _power_w = {nid: max(track_width, w) for nid, w in identify_power_nets(
            pcb_data, list(power_nets), list(power_nets_widths or ())).items()}
    _w_max = max([track_width] + list(_power_w.values()))
    _pad_reach = _w_max / 2 + max(
        [clearance, _base_cl] + list(_nc.values())
        + ([_rules.max_value(clearance)] if _rules.live else [])
        + [getattr(p, 'local_clearance', 0.0) or 0.0
           for pl in pcb_data.pads_by_net.values() for p in pl])
    _sxs = [pt[0] for s in stubs for pt in (s.pad_pos, s.corner_pos, s.stub_end)]
    _sys = [pt[1] for s in stubs for pt in (s.pad_pos, s.corner_pos, s.stub_end)]
    if _sxs:
        _lo_x, _hi_x = min(_sxs), max(_sxs)
        _lo_y, _hi_y = min(_sys), max(_sys)

        def _pad_near(p):
            r = 0.5 * math.hypot(p.size_x or 0.0, p.size_y or 0.0) + _pad_reach
            return (_lo_x - r <= p.global_x <= _hi_x + r
                    and _lo_y - r <= p.global_y <= _hi_y + r)

        foreign_pads = [p for plist in pcb_data.pads_by_net.values() for p in plist
                        if _pad_near(p)]
    else:
        foreign_pads = []

    # Escape room for NEIGHBOURING chips: the stubs their own fanout would lay,
    # each cut at the midline of a gap shared with this part. Only this part's
    # FAN must clear them (below) -- the straight escape is the minimum a pin
    # needs to leave its pad, and is never given up for a neighbour.
    _nb_window = ((_lo_x - 3.0, _lo_y - 3.0, _hi_x + 3.0, _hi_y + 3.0)
                  if _sxs else (0.0, 0.0, 0.0, 0.0))
    _neighbour_reserve = (_neighbour_escape_reserves(
        footprint, pcb_data, layer, extension, _nb_window) if _sxs else [])
    n_nb_short = 0

    # EXISTING copper of the part's OWN nets, checked geometrically per stub:
    # the obstacle map excludes ALL fanned nets (so a stub may touch its own
    # net's copper), but that also hid a NEIGHBOURING chip's already-fanned
    # stubs on a DIFFERENT fanned net -- two facing QFNs fanned back-to-back
    # left foreign-net stub crossings in the gap between them (issue #257,
    # usb_sniffer /T_USB_CLK x /T_USB_NXT). Only copper present BEFORE this
    # fanout is in pcb_data, so this never blocks the part's own new stubs.
    from geometry_utils import segment_to_segment_distance, point_to_segment_distance
    _fanned_existing_segs = [s for s in pcb_data.segments if s.net_id in _fanned_set]
    _fanned_existing_vias = [v for v in pcb_data.vias if v.net_id in _fanned_set]

    # Board-edge keep-out (issue #288): folded into _seg_grazes so an
    # edge-violating straight escape is dropped and an edge-violating 45 fan is
    # shortened by the existing recovery loop below, same as a foreign-pad graze.
    _edge_clear, _edge_rings, _edge_outer, _edge_cutouts = _board_edge_model(
        pcb_data, clearance, board_edge_clearance)
    from check_drc import _segment_to_rings_distance as _seg_rings_dist
    from check_drc import _point_on_board as _pt_on_board
    from check_drc import FilledCopperGate
    _filled_gate = FilledCopperGate(pcb_data)   # #1682: as the under-pad escape

    # #1328: a stub that cannot escape at `clearance` retries at the rungs of
    # qfn_clearance_ladder (empty under --escalation off), and #1318 tries a
    # power net's stub at the net's width. Each (rung, width) gets its own
    # obstacle map and each rung its own edge model, built the first time a
    # stub needs it; the run clearance at --width keeps the ones built above,
    # so a run in which nothing escalates or widens checks exactly what it
    # always did. A .kicad_dru rule on the escape layer REPLACES the pair
    # clearance there (#498, applied to `clearance` above) and would pin
    # every rung's map back to it, so a ruled escape layer does not escalate:
    # the board states that layer's clearance.
    _esc_ladder = ([] if _lcl_498.get(layer) is not None else
                   qfn_clearance_ladder(clearance, len(pcb_data.board_info.copper_layers or ())
                                        or 2))
    _rung_maps = {(clearance, round(track_width, 6)): (_obstacles, _obs_cfg)}
    _rung_edges = {clearance: (_edge_clear, _edge_rings, _edge_outer, _edge_cutouts)}

    def _map_at(cl, width=track_width, net_id=None):
        """The stub map at rung `cl` for a track `width` of `net_id`: stamped
        for that track and the stub's side of the pair (`_stub_floor`, #1529),
        with the same pad-free build and guard as the base map."""
        f = _stub_floor(cl, net_id)
        key = (f, round(width, 6))
        hit = _rung_maps.get(key)
        if hit is None:
            _cfg = GridRouteConfig(layers=list(_obs_cfg.layers), track_width=width,
                                   clearance=f)
            install_layer_clearances(_cfg, None, None, pcb_data)  # #498
            if _rules.live:                                       # #1565
                install_pair_rule_channels(_cfg, pcb_data, _fanned_net_ids)
            hit = (build_base_obstacle_map(pcb_data, _cfg, nets_to_route=_fanned_net_ids,
                                           extra_clearance=track_width / 2,
                                           net_clearances=_nc_foreign,
                                           stamp_pads=False), _cfg)
            _rung_maps[key] = hit
        return hit

    def _edge_at(cl):
        e = _rung_edges.get(cl)
        if e is None:
            e = _rung_edges[cl] = _board_edge_model(pcb_data, cl, board_edge_clearance)
        return e

    def _seg_hits_edge(p1, p2, cl=clearance, width=track_width):
        _ec, _er, _eo, _ecut = _edge_at(cl)
        if not _er:
            return False
        for (x, y) in (p1, p2):
            if not _pt_on_board(x, y, _eo, _ecut):
                return True
        return _seg_rings_dist(p1[0], p1[1], p2[0], p2[1], _er) \
            < width / 2 + _ec - 1e-6

    def _seg_grazes(p1, p2, net_id, cl=clearance, width=track_width):
        # Board edge first (issue #288) - independent of net.
        if _seg_hits_edge(p1, p2, cl, width):
            return True
        if _filled_gate and _filled_gate.segment_hit(
                p1[0], p1[1], p2[0], p2[1], width, layer, net_id, cl) is not None:
            return True
        # Foreign tracks / vias via the shared obstacle map (issue #149 part 2).
        if _obs_layer_idx is not None:
            _om, _oc = _map_at(cl, width, net_id)
            if not check_line_clearance(_om, p1[0], p1[1], p2[0], p2[1],
                                        _obs_layer_idx, _oc):
                return True
        # Foreign pads, exact (issue #123; found in #1328's replays):
        # check_drc's segment_pad_gap on the pad's true copper (round corners,
        # tilt, custom polygons) -- the shape KiCad and check_drc grade -- at
        # the larger of the rung's fan clearance and the clearance the pad is
        # held to (a .kicad_dru layer rule, the pad's own override). The old
        # rectangle grown by the margin with square corners overstated a
        # diagonal fan's need at a neighbour pad's corner by up to
        # margin*(sqrt2-1), enough to collapse fans check_drc grades clean
        # (uhk_trackball U1). NPTH copper-less pads are the map's holes.
        # #1529: every pair below at the two nets' classes too (`_pair`).
        for pad in foreign_pads:
            if pad.net_id == net_id or _pad_has_no_copper(pad):
                continue
            if layer not in expand_pad_layers(pad.layers, [layer]):
                continue
            if _segment_pad_gap(p1[0], p1[1], p2[0], p2[1], pad) - width / 2 \
                    < _pad_need(pad, _pair(cl, net_id, pad.net_id, types=(
                        ('track',), (('pad', pad.pad_type or 'smd'),)))) - 1e-9:
                return True
        # Existing copper of the part's other fanned nets (issue #257).
        for s in _fanned_existing_segs:
            if s.net_id == net_id or s.layer != layer:
                continue
            if segment_to_segment_distance(
                    p1[0], p1[1], p2[0], p2[1],
                    s.start_x, s.start_y, s.end_x, s.end_y) \
                    < s.width / 2 + width / 2 + _pair(cl, net_id, s.net_id, 'track') - 1e-6:
                return True
        for v in _fanned_existing_vias:
            if v.net_id == net_id:
                continue
            if point_to_segment_distance(v.x, v.y, p1[0], p1[1], p2[0], p2[1]) \
                    < v.size / 2 + width / 2 + _pair(cl, net_id, v.net_id, types=(
                        ('track',), ('via',))) - 1e-6:
                return True
        return False

    def _hits_neighbour_reserve(p1, p2, net_id, cl=clearance, width=track_width):
        for a, b, rn in _neighbour_reserve:
            if rn == net_id:
                continue
            if segment_to_segment_distance(p1[0], p1[1], p2[0], p2[1],
                                           a[0], a[1], b[0], b[1]) \
                    < width / 2 + track_width / 2 + _pair(cl, net_id, rn, 'track') - 1e-6:
                return True
        return False

    # This run's OWN stubs. A fan must clear every other-net straight escape
    # (the minimum a pin needs to leave its pad, never given up for a fan) and
    # every other-net fan already kept. The foreign-pad test used to stand in
    # for this by accident: its square-cornered box kept a fan far enough off
    # the neighbour pad's corner that it also missed the neighbour's fan; the
    # exact pad gap does not, so adjacent fans are checked here directly.
    #
    # The square box also kept every TIP `track_width + extension` off its
    # neighbours', the spacing check_endpoint_spacing validates below and
    # reports as failed nets: ft2232h U4 (--extension 0.3) shipped /OSCO's
    # longer fan 0.447 mm from /OSCI's tip and failed both. So a fan's tip
    # also keeps that spacing from every other-net tip already kept and from
    # every other-net straight end (where a fan that collapses ends). Under
    # the default extension the clearance test above already implies it.
    _tip_spacing = track_width + extension - 1e-6

    def _hits_own_stubs(p1, p2, net_id, cl):
        for s in stubs:
            if s.net_id == net_id:
                continue
            a, b = s.pad_pos, s.corner_pos
            if segment_to_segment_distance(
                    p1[0], p1[1], p2[0], p2[1], a[0], a[1], b[0], b[1]) \
                    < track_width + _pair(cl, net_id, s.net_id, 'track') - 1e-6:
                return True
            if math.hypot(p2[0] - b[0], p2[1] - b[1]) < _tip_spacing:
                return True
        for s in kept_stubs:
            if s.net_id == net_id:
                continue
            if segment_to_segment_distance(
                    p1[0], p1[1], p2[0], p2[1], s.corner_pos[0], s.corner_pos[1],
                    s.stub_end[0], s.stub_end[1]) \
                    < track_width + _pair(cl, net_id, s.net_id, 'track') - 1e-6:
                return True
            if math.hypot(p2[0] - s.stub_end[0], p2[1] - s.stub_end[1]) < _tip_spacing:
                return True
        return False

    def _fan_grazes(p1, p2, net_id, cl=clearance):
        return (_seg_grazes(p1, p2, net_id, cl)
                or _hits_neighbour_reserve(p1, p2, net_id, cl)
                or _hits_own_stubs(p1, p2, net_id, cl))

    qfn_dropped: List[str] = []
    n_short = 0
    n_ext_short = 0
    kept_stubs: List[FanoutStub] = []
    _escalated: List[Tuple[FanoutStub, float]] = []      # #1328
    # The per-stub graze test scans every foreign pad; on a crowded board this
    # loop is where the QFN seconds go, so count it out to the status line.
    for _sti, (stub, pad_info) in enumerate(zip(stubs, pad_infos)):
        if _cancel():                                   # #621
            # Untried stubs are simply not kept: no copper, and NOT appended to
            # qfn_dropped -- they were never checked, so they are not failures.
            break
        if progress_callback:
            progress_callback(
                _sti + 1, len(stubs),
                f"QFN fanout {getattr(footprint, 'reference', '?')}: "
                f"clearing stub {stub.pad.net_name or stub.net_id}")
        nid = stub.net_id
        # The straight escape at the run clearance, then (#1328) at each
        # escalation rung below it: the first clearance it clears at is the
        # stub's, and its fan is shortened at that same clearance.
        stub_cl = None
        # #1529: never below the stub's own class (route.py's rule_floors):
        # KiCad grades its net there whatever the Default class is lowered to.
        _own_cls = _nc.get(nid) if _nc else None
        for cl in [clearance] + [r for r in _esc_ladder
                                 if _own_cls is None or r >= _own_cls - 1e-9]:
            if not _seg_grazes(stub.pad_pos, stub.corner_pos, nid, cl):
                stub_cl = cl
                break
            # #513 item 15 (ice4pi): a straight escape toward a nearby board
            # edge used to be DROPPED outright; on an edge-hugging row that
            # left the whole side unescaped unless the operator hand-tuned
            # --extension 0.0. When the ONLY offender is the board edge,
            # shrink the extension toward 0 first -- the escape still clears
            # the pad, it just bends sooner.
            if _seg_hits_edge(stub.pad_pos, stub.corner_pos, cl):
                for _t in [1.0 - i / 9.0 for i in range(1, 9)] + [0.0]:
                    _sl = pad_info.pad_width / 2 + extension * _t
                    _c2, _e2 = calculate_fanout_stub(
                        pad_info, layout, _sl, max_diagonal_length, grid_step,
                        angle_ref_off=side_max_off.get(pad_info.side, 0.0))
                    if not _seg_grazes(stub.pad_pos, _c2, nid, cl):
                        stub.corner_pos, stub.stub_end = _c2, _e2
                        stub_cl = cl
                        n_ext_short += 1
                        break
                if stub_cl is not None:
                    break
        if stub_cl is None:
            # The perpendicular escape itself hits a foreign pad (or even
            # the zero-extension escape violates the edge) - cannot fan out,
            # at the run clearance or at any rung escalation allows.
            if stub.pad.net_name and stub.pad.net_name not in qfn_dropped:
                qfn_dropped.append(stub.pad.net_name)
            LAST_STUB_REPORT['dropped_pads'].append(               # #1834
                (stub.pad.pad_number, stub.pad.net_name))
            continue
        if stub_cl < clearance - 1e-9:
            _escalated.append((stub, stub_cl))
        if _fan_grazes(stub.corner_pos, stub.stub_end, nid, stub_cl):
            if _hits_neighbour_reserve(stub.corner_pos, stub.stub_end, nid, stub_cl) \
                    and not _seg_grazes(stub.corner_pos, stub.stub_end, nid, stub_cl):
                n_nb_short += 1
            # Shorten the 45 fan toward the corner until it clears (worst case
            # collapse it entirely - the straight escape is already clear).
            cx, cy = stub.corner_pos
            ex, ey = stub.stub_end
            new_end = stub.corner_pos
            for i in range(1, 9):
                t = 1.0 - i / 9.0  # walk inward from current tip toward corner
                cand = (cx + (ex - cx) * t, cy + (ey - cy) * t)
                if not _fan_grazes(stub.corner_pos, cand, nid, stub_cl):
                    new_end = cand
                    break
            # Re-snap the shortened tip ON GRID (#446). calculate_fanout_stub
            # lands the tip on the routing grid (#149) so the router gets an
            # on-grid terminal it can END on; this shortening then moved it to
            # an arbitrary ninth of the way in, silently discarding that
            # guarantee. An off-grid terminal cannot be reached exactly by the
            # on-grid router: it stops a cell short, its cap merely OVERLAPS
            # the stub cap (which already reads as "connected"), and the board
            # ships a fragile soft joint -- zynq_ad9364 VCC_3V3, a 0.068mm
            # near-open that close_soft_joints then could not bridge at the
            # run's clearance.
            #
            # Every candidate is re-tested with the SAME _seg_grazes gate, so
            # the #123 clearance guarantee is preserved exactly; candidates are
            # searched INWARD of the clearing point only (never back out toward
            # the graze). If nothing on-grid clears, keep the unsnapped point --
            # a clear-but-off-grid tip is strictly better than a violation.
            new_end = _snap_tip_on_grid(
                stub.corner_pos, new_end, nid, grid_step,
                lambda _p1, _p2, _n, _c=stub_cl: _fan_grazes(_p1, _p2, _n, _c))
            stub.stub_end = new_end
            n_short += 1
        kept_stubs.append(stub)
    if n_nb_short:
        print(f"  Neighbour escape room: {n_nb_short} fan(s) stopped at the "
              f"midline of a gap shared with another chip "
              f"({len(_neighbour_reserve)} reserved stub segment(s))")
    if n_short or n_ext_short or qfn_dropped:
        print(f"  Pad-clearance: shortened {n_short} fan(s), "
              f"{n_ext_short} straight escape(s) near the board edge (#513); "
              f"dropped {len(qfn_dropped)} stub(s) grazing a foreign pad (issue #123)")
    _record_escalations(footprint, pcb_data, _escalated, clearance)
    LAST_STUB_REPORT['pads_tried'] = (len(kept_stubs)                    # #1834
                                      + len(LAST_STUB_REPORT['dropped_pads']))
    stubs = kept_stubs

    # A stub that REACHES a same-net pad ends there (#1332). Every check above
    # skips same-net pads, so a fan that clips its own decoupling cap's pad
    # (steppenprobe U1.64 +1V8 over C11.1) ran on past it, and the copper
    # beyond the contact shipped as a dangling tail: route.py keeps input
    # copper, so nothing trimmed it.
    from fix_kicad_drc_settings import connection_width_floor
    n_joined = _end_stubs_at_same_net_pads(
        stubs, foreign_pads, track_width, _seg_grazes, pcb_data,
        connection_width_floor(pcb_data, track_width, shipped=True) or 0.1)
    if n_joined:
        print(f"  Same-net pads: {n_joined} stub(s) end on a same-net pad they "
              f"reach (#1332): "
              + ', '.join(f'{s.pad.net_name} at {s.joined_pad}'
                          for s in stubs if s.joined_pad)[:300])

    # #1318: a power net's stub at its power width, leg by leg, where it fits.
    _leg_w = _widen_power_stub_legs(
        stubs, _power_w, track_width, clearance, _seg_grazes,
        _hits_neighbour_reserve,
        pair_clearance=lambda a, b: _pair(clearance, a, b, 'track')) if _power_w else {}

    # Generate tracks - two segments per stub
    tracks = []
    for stub in stubs:
        _w3 = _leg_w.get(id(stub), (track_width,) * 3)
        # Segment 1: Straight from pad to corner
        dx1 = abs(stub.corner_pos[0] - stub.pad_pos[0])
        dy1 = abs(stub.corner_pos[1] - stub.pad_pos[1])
        if dx1 > POSITION_TOLERANCE or dy1 > POSITION_TOLERANCE:
            tracks.append({
                'start': stub.pad_pos,
                'end': stub.corner_pos,
                'width': _w3[0],
                'layer': stub.layer,
                'net_id': stub.net_id
            })

        # Segment 2: 45 degree from corner to end
        dx2 = abs(stub.stub_end[0] - stub.corner_pos[0])
        dy2 = abs(stub.stub_end[1] - stub.corner_pos[1])
        if dx2 > POSITION_TOLERANCE or dy2 > POSITION_TOLERANCE:
            tracks.append({
                'start': stub.corner_pos,
                'end': stub.stub_end,
                'width': _w3[1],
                'layer': stub.layer,
                'net_id': stub.net_id
            })

        # Segment 3 (#1332): close a corner-graze contact into the same-net
        # pad the stub ends on.
        if stub.join_end is not None:
            tracks.append({
                'start': stub.stub_end,
                'end': stub.join_end,
                'width': _w3[2],
                'layer': stub.layer,
                'net_id': stub.net_id
            })

    print(f"  Generated {len(tracks)} track segments ({len(stubs)} stubs x 2 segments)")

    # Fine-pitch escape warning (issue #97): the corner stubs reach a full
    # 45 deg fan, so the tightest adjacent pair sits about pitch/sqrt(2) apart
    # (inner stubs diverge wider, issue #200). At common defaults (clearance
    # 0.25) the router cannot launch from or pass between these stubs and every
    # net fails with 'no rippable blockers found'. Tell the user the workable
    # parameters up front instead. (pitch/sqrt2 is the conservative lower bound.)
    if layout.pad_pitch and layout.pad_pitch < 0.8:
        lateral = layout.pad_pitch / math.sqrt(2)
        # Escape at the stub's own width: route_track/2 + clearance +
        # stub_track/2 must fit in `lateral`.
        max_clear = lateral - track_width
        # One 0.05 grid step of margin for quantization, rounded down to 0.05,
        # capped at 0.15 (the combination verified to escape 0.5 mm LQFP fans).
        suggest_clear = min(max(0.05, int((max_clear - 0.05) / 0.05) * 0.05), 0.15)
        print(f"  NOTE: {layout.pad_pitch:.2f} mm pitch keeps adjacent fan stubs only "
              f"{lateral:.3f} mm apart (pitch/sqrt2).")
        print(f"  Routing these nets needs clearance below {max_clear:.2f} mm (with "
              f"{track_width:.2f} mm track) plus grid-quantization margin; the "
              f"default 0.25 clearance / 0.1 grid will fail to escape.")
        print(f"  Suggested: route.py --grid-step 0.05 --clearance {suggest_clear:.2f} "
              f"--track-width {track_width:.2f} for this component's nets.")

    # Validate endpoint spacing. A stub that ends on a same-net pad (#1332)
    # has no tip for the router to launch from -- its net continues from that
    # pad, which every other stub already clears -- so it is not checked.
    min_spacing = track_width + extension
    _tips = [s for s in stubs if not s.joined_pad]
    collisions = check_endpoint_spacing(_tips, min_spacing)

    failed_nets: List[str] = list(qfn_dropped)
    if collisions:
        print(f"  WARNING: {len(collisions)} endpoint pairs too close!")
        for i, j, dist in collisions[:5]:
            print(f"    {_tips[i].pad.net_name} <-> {_tips[j].pad.net_name}: {dist:.3f}mm")
        print(f"  Consider increasing extension")
        # Collect the deduplicated set of nets involved in any collision -
        # these are the "failed" nets the GUI surfaces.
        seen = set()
        for i, j, _dist in collisions:
            for name in (_tips[i].pad.net_name, _tips[j].pad.net_name):
                if name and name not in seen:
                    seen.add(name)
                    failed_nets.append(name)
    else:
        print(f"  Validated: No endpoint collisions")

    return _serve_interior(tracks, failed_nets)



def autopick_rank(fp):
    """The CLI's QFN/QFP auto-pick key: name evidence, then PIN count.

    Pins, not pads (#1148): a thermal pad's split paste windows are not
    pins, and counting them ranked a part by its stencil (tigard J1 read 30
    pads for 22 pins, watchy U4 73 for 57).
    """
    from kicad_parser import non_aperture_pads
    name_hit = any(t in (fp.footprint_name or '').upper()
                   for t in ('QFN', 'QFP', 'DFN', 'MLF'))
    return (1 if name_hit else 0, len(non_aperture_pads(fp)))

def main():
    """Run QFN fanout generation."""
    import argparse

    parser = argparse.ArgumentParser(description='Generate QFN/QFP fanout routing')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='kicad_files/qfn_fanout_test.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--component', '-c', default=None,
                        help='Component reference (auto-detected if not specified)')
    parser.add_argument('--layer', '-l', default=None,
                        help='Routing layer (default: the layer the component '
                             'is mounted on)')
    # --track-width is an ALIAS, not a second knob: bga_fanout spells this
    # same concept --track-width, and two sibling fanout CLIs disagreeing on
    # the name is a trap. It cost a recorded stress run a wasted step
    # (openstint set4 has three qfn_fanout attempts in its manifest -- the
    # middle one is `--track-width 0.08` failing against this parser) and it
    # cost a replay of that manifest another. dest stays `width`.
    parser.add_argument('--width', '--track-width', '-w', type=float,
                        default=0.1, help='Track width in mm (default 0.1; '
                                          '--track-width is an alias)')
    import routing_defaults as defaults
    parser.add_argument('--extension', type=float, default=defaults.QFN_EXTENSION,
                        help=f'Extension past pad edge before bend (mm, default {defaults.QFN_EXTENSION})')
    parser.add_argument('--clearance', type=float, default=None,
                        help='Min clearance to other-net copper (mm); stubs that '
                             'would graze it are shortened, or escalated to a lower '
                             'clearance when --escalation allows (#1328). Default: '
                             "the board's Default net-class clearance (else "
                             f'{defaults.QFN_CLEARANCE}), capped by --clearance-ceiling')
    parser.add_argument('--nets', '-n', nargs='*',
                        help='Net patterns to include')
    parser.add_argument('--grid-step', type=float, default=defaults.GRID_STEP,
                        help='Routing grid step in mm (default: 0.1). Fanned stub ends are '
                             'snapped to this grid so the router gets on-grid terminals (issue '
                             '#149); MATCH the --grid-step you pass to route.py.')
    parser.add_argument('--escape-method', choices=['stub', 'underpad'], default='stub',
                        help="'stub' (default) = surface 45-degree fan; 'underpad' = drop a "
                             "through-via just past each pad and escape on an inner layer "
                             "(issue #164), for crowded fine-pitch edges where the surface is full. "
                             "With --nets either method via-drops the interior pads the filter "
                             "selects (stub: after its fan, #1717); a run without --nets never "
                             "via-drops the exposed/thermal pad.")
    parser.add_argument('--via-size', type=float, default=None,
                        help="Underpad escape via outer diameter (mm, default the board's "
                             "Default net-class via, else 0.45)")
    parser.add_argument('--via-drill', type=float, default=None,
                        help="Underpad escape via drill diameter (mm, default the board's "
                             "Default net-class drill, else 0.25)")
    parser.add_argument('--board-edge-clearance', type=float, default=defaults.BOARD_EDGE_CLEARANCE,
                        help='Min clearance from stub/via copper to the Edge.Cuts '
                             'outline in mm (default 0 = use --clearance). Stubs '
                             'that would graze the board edge are shortened or '
                             'dropped; underpad escape vias near the edge are '
                             'rejected (issue #288).')
    parser.add_argument('--power-nets', nargs='*', default=[],
                        help="Glob patterns for power nets, as route.py takes them "
                             "(e.g. '*GND*' '+3V3'). Must pair with --power-nets-widths. "
                             "A power net's surface stub, on a pad at least that wide, is "
                             "laid at its width wherever that fits, and at --width where it "
                             "does not (#1318): route.py keeps fanout copper as it finds it, "
                             "so pass the route step's own --power-nets here. A via-drop "
                             "stub is drawn at --width; JSON_SUMMARY power_stubs_at_width "
                             "names each power/ground stub drawn at --width (#1743).")
    parser.add_argument('--power-nets-widths', nargs='*', type=float, default=[],
                        help='Track widths in mm for each --power-nets pattern')
    parser.add_argument('--same-net-pad-clearance', type=float, default=None,
                        help='#581: > 0 forbids via-in-pad (overrides --allow-via-in-pad) and '
                             'is recorded in the sibling .kicad_pro so later chain steps '
                             'inherit it. -1 explicitly allows via-in-pad. Default: the '
                             'project record, else allowed.')
    parser.add_argument('--allow-via-in-pad', action='store_true',
                        help='Underpad escape: let the escape via overlap its OWN pad '
                             '(via-in-pad), so a via boxed in on the outward side can '
                             'stagger inward toward the chip instead of being dropped. '
                             'It also enables an INWARD search along the escape axis '
                             'that steps by the inter-net stagger, so on a fine-pitch '
                             'part its later rungs land past the pad edge on the chip '
                             'side, and four extra stagger configurations (#846). A via '
                             'that does overlap its pad is clamped to the pad edge '
                             '(#202) and needs IPC-4761 Type VII. The via still must '
                             'clear other-net pads, vias and tracks.')
    # #489 section 9: fanout is where a teardrop matters most (a 0.1mm trace
    # meeting a 0.25mm via pad), and this step had no way to ask for one.
    parser.add_argument('--add-teardrops', action='store_true',
                        help='Add teardrop settings to all pads and vias in the output file')
    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    # #381 D8: define --no-fix-drc-settings (and the shared DRC-fix flags) that
    # the writeback below already reads via getattr but that was never declared.
    # store_true default keeps no_fix_drc_settings=False => writeback ON =>
    # identical behavior for existing commands.
    from fix_kicad_drc_settings import add_drc_fix_args
    add_drc_fix_args(parser)
    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()
    if len(args.power_nets) != len(args.power_nets_widths):
        parser.error(f"--power-nets ({len(args.power_nets)}) and --power-nets-widths "
                     f"({len(args.power_nets_widths)}) must have the same length")
    from fix_kicad_drc_settings import warn_if_missing_project_floor
    warn_if_missing_project_floor(args.pcb)  # #441: a dropped sibling .kicad_pro strands the DRC floor
    # #1328: an omitted --clearance is the board's Default net class, as on
    # route.py -- not QFN_CLEARANCE, which the writeback then stamped on every
    # class (resolve_qfn_clearance, shared with the GUI QFN panel).
    from list_nets import board_default_netclass_clearance
    args.clearance, _clr_lines = resolve_qfn_clearance(
        args.clearance, board_default_netclass_clearance(args.pcb))
    for _l in _clr_lines:
        print(_l)
    # This front registers --clearance-ceiling (add_fab_tier_args does) and used
    # to IGNORE it, so the spelling CLAUDE.md tells a chain to use did nothing.
    __import__('fab_tiers').apply_clearance_ceiling(args, 'qfn_fanout')
    # #513 item 15: default the edge keep-out to the BOARD'S OWN
    # min_copper_edge_clearance (route.py's documented behavior and the GUI's
    # unchecked-override behavior), not the copper-copper --clearance. ice4pi
    # shipped 7 SEGMENT-BOARD-EDGE violations because stubs were kept 0.09mm
    # from Edge.Cuts while the board's rule (and check_drc) demanded 0.2mm.
    if (args.board_edge_clearance or 0) <= 0:
        try:
            from list_nets import board_constraint
            _edge = board_constraint(args.pcb, 'min_copper_edge_clearance')
        except Exception:
            _edge = None
        if _edge:
            args.board_edge_clearance = _edge
            print(f"--board-edge-clearance not given; using the board "
                  f"min_copper_edge_clearance {_edge}mm.")
    set_default_fab_tier(*fab_tier_from_args(args))
    # #530: --width IS this run's track-width request. The stale-minimum rule
    # (set_policy_from_args) and the physical-floor pin (enforce_fab_floors)
    # both look for `track_width`; without the alias neither saw it, so a
    # stock 0.2 mm board minimum pinned 0.1 mm escape stubs up to 0.2.
    if getattr(args, 'track_width', None) is None:
        args.track_width = args.width
    # #1795: a via nobody asked for starts at the board's Default net-class via,
    # as on route.py and the GUI's QFN panel; #1676: raised to its .kicad_dru rules
    _vs_explicit = args.via_size is not None or args.via_drill is not None
    from list_nets import board_default_netclass_param
    for _pname, _nckey, _fallback in (('via_size', 'via_diameter', 0.45),
                                      ('via_drill', 'via_drill', 0.25)):
        if getattr(args, _pname) is None:
            _v = board_default_netclass_param(args.pcb, _nckey)
            setattr(args, _pname, _v if _v else _fallback)
            print(f"--{_pname.replace('_', '-')} not given; using "
                  f"{'the board Default net-class' if _v else 'the fallback'} "
                  f"{getattr(args, _pname)}mm.")
    if not _vs_explicit:
        from design_rules import run_via_for_rules
        args.via_size, args.via_drill = run_via_for_rules(args.pcb, None, args.via_size,
                                                          args.via_drill, board_setup=False)
    __import__('fab_tiers').set_policy_from_args(args, args.pcb)  # #857
    if not _vs_explicit:
        # #1775: and, under --escalation board, at its Board Setup minimums
        from design_rules import via_for_board_setup
        args.via_size, args.via_drill = via_for_board_setup(args.via_size, args.via_drill)
    _pinned_floors = enforce_fab_floors(
        count_copper_layers_in_file(args.pcb),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        clearance_ceiling=getattr(args, 'clearance_ceiling', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None),
        # #513 item 15: the resolved edge keep-out must be manufacturable too
        # (a board declaring 0.025mm is pinned up to the 0.2 fab edge floor,
        # which is also what check_drc grades at).
        board_edge_clearance=(args.board_edge_clearance or None))
    # Below-floor params are pinned up to the fab floor (warned); apply the clamps
    # (#513 item 1: discarding this dict shipped sub-floor vias after the warning).
    for _pname, _pfloor in _pinned_floors.items():
        setattr(args, _pname, _pfloor)

    print(f"Parsing {args.pcb}...")
    pcb_data = parse_kicad_pcb(args.pcb)
    # the copper this board ARRIVED with: the graze audit below counts only
    # what this step laid (the engine appends its own copper to pcb_data)
    from check_drc import copper_keys
    input_copper = copper_keys(pcb_data)

    # Auto-detect QFN/QFP component if not specified
    if args.component is None:
        qfn_components = find_components_by_type(pcb_data, 'QFN')
        if not qfn_components:
            qfn_components = find_components_by_type(pcb_data, 'QFP')
        if qfn_components:
            # Run-6 ranking: file order picked J1 (a rect-pad USB-C the
            # geometric fallback classifies QFN) over the real 64-pin QFN.
            # Drop connector/marker classes (part_class KB), prefer
            # name-evidenced QFN/QFP footprints, then most pads. Fully
            # generic; prints its reasoning.
            _rank = autopick_rank
            ranked = list(qfn_components)
            try:
                from placement.part_class import classify_part
                keep = [fp for fp in ranked
                        if classify_part(fp, fp.reference).name
                        not in ('edge_receptacle', 'edge_actuator',
                                'mount_hole', 'fiducial', 'testpoint')]
                if keep:
                    ranked = keep
            except Exception:
                pass
            ranked.sort(key=_rank, reverse=True)
            args.component = ranked[0].reference
            print(f"Auto-detected QFN/QFP component: {args.component}")
            if len(ranked) > 1:
                print(f"  (Ranked over: "
                      f"{[fp.reference for fp in ranked[1:]]}; name-evidence "
                      f"then pad count; connector/marker classes dropped)")
        else:
            print("Error: No QFN/QFP components found in PCB")
            print(f"Available components: {list(pcb_data.footprints.keys())[:20]}...")
            return 1

    if args.component not in pcb_data.footprints:
        print(f"Error: Component {args.component} not found")
        print(f"Available: {list(pcb_data.footprints.keys())[:20]}...")
        return 1

    footprint = pcb_data.footprints[args.component]
    _single = single_line_reason(footprint)
    if _single:
        # #1195: refused by name, not analysed as a degenerate QFN that
        # "finds 0 pads", writes the board through and exits 0.
        print(f"Error: {args.component} ({footprint.footprint_name}) is not a "
              f"QFN/QFP: {_single}. Nothing was written.")
        return 1
    print(f"\nFound {args.component}: {footprint.footprint_name}")
    print(f"  Position: ({footprint.x:.2f}, {footprint.y:.2f})")
    print(f"  Rotation: {footprint.rotation}deg")
    print(f"  Pads: {len(footprint.pads)}")

    # Default the stub layer to the component's mounted layer (#96); the QFN
    # panel resolves its layer choice through the same function.
    layer, _layer_note, _ = resolve_escape_layer(footprint, args.layer,
                                                 args.component)
    if _layer_note:
        print(_layer_note)

    # #621: the CLI passes no cancel_check. The engine's cooperative cancel is
    # the GUI's (its Cancel button / the plan executor's Stop); a CLI-side
    # wall-clock budget was removed deliberately -- no result this tool produces
    # may depend on timing, or the same command stops producing the same board.
    tracks, vias, _failed_nets = generate_qfn_fanout(
        footprint,
        pcb_data,
        net_filter=args.nets,
        layer=layer,
        track_width=args.width,
        extension=args.extension,
        clearance=args.clearance,
        grid_step=args.grid_step,
        board_edge_clearance=args.board_edge_clearance,
        escape_method=args.escape_method,
        via_size=args.via_size,
        via_drill=args.via_drill,
        allow_via_in_pad=args.allow_via_in_pad,
        same_net_pad_clearance=args.same_net_pad_clearance,  # #581
        power_nets=args.power_nets or None,                  # #1318
        power_nets_widths=args.power_nets_widths or None,
        # #1529: the board's classes (read from pcb_data.source_path), capped
        # at the ceiling when one was given, as route.py's map is.
        clearance_ceiling=getattr(args, 'clearance_ceiling', None),
        via_from_class=not _vs_explicit,                     # #1795
    )
    # #1641: a --nets list naming no net on the board is an input error (a
    # typo, a stale list), refused as route.py refuses one; a pattern that
    # names a net but selects no pad here is disclosed in `unmatched`.
    from bga_fanout import refuse_unknown_nets
    _refusal = refuse_unknown_nets(LAST_FILTER_MISSES, pcb_data, args.nets)
    if _refusal:
        print(f"Error: {_refusal}")
        return 2

    if tracks or vias:
        # vias can be non-empty with zero tracks: a via-in-pad centred on its
        # pad emits no stub, so an underpad run can be vias-only.
        print(f"\nWriting {len(tracks)} tracks and {len(vias)} vias to {args.output}...")
        net_id_to_name = {nid: net.name for nid, net in pcb_data.nets.items()}
        add_tracks_and_vias_to_pcb(args.pcb, args.output, tracks, vias,
                                   net_id_to_name=net_id_to_name,
                                   add_teardrops=args.add_teardrops)
        print("Done!")
    else:
        print("\nNo fanout tracks generated")
        # Still produce the output file (board unchanged) so a multi-step
        # pipeline can continue - otherwise a fanout that finds nothing to do
        # (e.g. the component is already fanned on a retry) leaves the next step
        # with no input file.
        if getattr(args, 'output', None):
            from pcb_io_utils import passthrough_copy
            passthrough_copy(args.pcb, args.output)
            print(f"Wrote board through to {args.output} (unchanged)")

    # Structured summary + post-fanout DRC so downstream tooling (plan-pcb-routing
    # skill, stress harness) can detect when the escape left sub-clearance grazes
    # behind even though every pad escaped -- e.g. the 45-degree escape stubs of two
    # adjacent pads of a 0.4mm-pitch diff pair clipping at the wrist (issue #179).
    # The planner uses drc_grazes to retry the fanout with a thinner --width (and,
    # for the underpad method, a smaller via) toward the fab floor until it's clean.
    # Mirrors bga_fanout (#130/#122). Best-effort: a DRC hiccup must never fail the
    # fanout. drc_grazes is graded at the clearance the output project is written
    # with: the run clearance, or the lowest rung an escalated stub took (#1328),
    # and that project's net classes (#1529), and counts only the rows THIS
    # step's copper takes part in (check_drc.fanout_drc_grazes).
    import json as _json
    import clearance_ledger as _ledger
    eff_clearance = _ledger.effective(args.clearance)
    # A stub-less via-in-pad escape emits a via but no track, so vias count as
    # escapes too -- tracks alone undercounts and grades those pads as failed.
    escaped_net_ids = ({t['net_id'] for t in tracks if t.get('net_id') is not None}
                       | {v['net_id'] for v in vias if v.get('net_id') is not None})
    # #1804/#1834: and a net whose selected pads already had their escape
    # (the via-drop or the surface fan left them out), so a re-run counts the
    # same nets as the first
    _served = served_pad_net_names()
    escaped_net_ids |= {nid for nid, net in pcb_data.nets.items() if net.name in _served}
    # A net is UNESCAPED only while one of its pads has no copper. The engine
    # also names nets whose stubs escaped but end too close to a neighbour's
    # (the GUI's "stubs too close" list), or a multi-pad net with one pad's
    # escape given up and the rest served: those are flagged_escaped_nets, not
    # failures, and a net is never both escaped and failed.
    from bga_fanout import unserved_nets
    unescaped = sorted(unserved_nets(footprint, pcb_data, sorted(set(_failed_nets)),
                                     tracks, vias, args.width))
    flagged_escaped = sorted(set(_failed_nets) - set(unescaped))
    _un = set(unescaped)
    escaped = len(escaped_net_ids - {nid for nid, net in pcb_data.nets.items()
                                     if net.name in _un})
    # The CLI never cancels (#621: no --deadline, no other CLI cancel source),
    # so LAST_CANCEL_SKIPPED is empty here by construction and every pad was
    # concluded one way or the other. The partial-ledger arithmetic lives in the
    # engine for the GUI, which does have a cancel.
    requested = escaped + len(unescaped)
    out_path = getattr(args, 'output', None)

    # Establish this fanout's clearance floor in the output .kicad_pro so the next
    # pipeline step and check_drc grade at the clearance the fanout used -- only
    # lowers, never tightens (issue #160). #1328: an escalated stub lowers it to
    # the rung that stub cleared at -- what the copper uses, never the fab floor
    # the ladder was allowed to reach -- and fix_project_for_output's #1160
    # record discloses a Default class lowered by that descent.
    from fix_kicad_drc_settings import fanout_fix_kwargs
    # The one fanout writeback decision (fix_kicad_drc_settings), which the GUI
    # fanout tab applies to the live board. #1195: None when this run drew no
    # copper; the via floors only when it drew a via (stub mode never does, and
    # its --via-size/--via-drill defaults lowered the declared via and hole
    # floors on every run). #1529: the escape is spaced at the net classes,
    # so they are kept -- or capped at --clearance-ceiling when one was
    # given -- as route.py's writeback does.
    _fix_kw = fanout_fix_kwargs('qfn', tracks, vias, clearance=eff_clearance,
                                track_width=args.width,
                                via_size=getattr(args, 'via_size', None),
                                via_drill=getattr(args, 'via_drill', None),
                                clearance_ceiling=getattr(args, 'clearance_ceiling', None))
    if out_path and os.path.isfile(out_path) and _fix_kw is None:
        # #1195: a run that changed no copper writes no floors -- the board
        # went through unchanged, so its project does too (with every other
        # sibling), never a writeback of sizes nothing was drawn at.
        try:
            import shutil as _sh
            from copy_board import SIBLING_EXTS
            _ib, _ob = os.path.splitext(args.pcb)[0], os.path.splitext(out_path)[0]
            if os.path.abspath(_ib) != os.path.abspath(_ob):
                for _ext in SIBLING_EXTS:
                    if os.path.isfile(_ib + _ext):
                        _sh.copyfile(_ib + _ext, _ob + _ext)
        except Exception as _e:
            print(f"  (could not carry the project through: {_e})")
    elif out_path and os.path.isfile(out_path) \
            and not getattr(args, 'no_fix_drc_settings', False):
        try:
            from fix_kicad_drc_settings import (fix_project_for_output,
                                                step_hole_clearance)
            # #1568: the copper-to-hole floor this step HELD, lowered only to
            # what its own copper needs (route.py's #1493 writeback)
            _fix_kw['hole_clearance'] = step_hole_clearance(
                args.pcb, out_path, _fix_kw['clearance'])
            fix_project_for_output(out_path, input_pcb=args.pcb, **_fix_kw)
        except Exception as _e:
            print(f"  (skipped DRC-settings fix: {_e})")
    if out_path and os.path.isfile(out_path) \
            and not getattr(args, 'no_fix_drc_settings', False):
        # #581: record an ACTIVE same-net pad via clearance for later steps.
        try:
            from protected_nets import (persist_same_net_pad_clearance,
                                        pro_path_for_board)
            if args.same_net_pad_clearance is not None \
                    and args.same_net_pad_clearance > 0:
                persist_same_net_pad_clearance(
                    pro_path_for_board(out_path), args.same_net_pad_clearance)
        except Exception as _e:
            print(f"  (skipped same-net pad clearance record: {_e})")
    # After the writeback, so the grade reads the project as written: its
    # Default floor and (#1529) its net classes, as check_drc does.
    drc_grazes = {}
    if out_path:
        try:
            from check_drc import fanout_drc_grazes
            drc_grazes = fanout_drc_grazes(out_path, eff_clearance, input_copper,
                                           net_classes=True)
        except Exception as _e:
            drc_grazes = {'error': str(_e)}
    summary = {
        'component': args.component,
        'requested': requested,
        'escaped': escaped,
        'failed': len(unescaped),
        'unescaped_nets': unescaped,
        'flagged_escaped_nets': flagged_escaped,
        # #1342: single-pad (NC) nets the engine skipped, as bga_fanout's key.
        'skipped_nc': len(single_pad_net_ids(footprint, pcb_data)),
        # #1641: each --nets pattern that selected no pad, with its reason
        # (an interior pad the stub escape skips, a net with no pad on the
        # part, ...): requested counts only the pads a pattern selected.
        'unmatched': list(LAST_FILTER_MISSES),
        # #1717: the interior pads the filter selected that the stub run's
        # via-drop could not place ("U1.34"), left with no copper; their
        # nets are in failed / unescaped_nets as well.
        'skipped_interior': list(LAST_SKIPPED_INTERIOR),
        # #1743: the escape stubs drawn at --width on a net that reads as
        # power or ground (pad, net, width, escape, why). Disclosure only.
        'power_stubs_at_width': list(LAST_POWER_STUBS_AT_WIDTH),
        # #1775: the vias laid below the board's own Board Setup via
        # minimums, with the project writeback off (rule, kind, min, vias,
        # below, smallest). Disclosure only.
        'board_setup_vias': list(LAST_BOARD_SETUP_VIAS),
        'clearance': args.clearance,
        'track_width': args.width,
        'escape_method': args.escape_method,
        'via_size': args.via_size,
        'via_drill': args.via_drill,
        'layer': layer,
        # grazes graded at the written floor and the written project's net
        # classes (#1529), scoped to the copper this step laid ('scope'):
        # 'total' is every such row, the segment_segment/via_*/pad_* keys are
        # the fanout-relevant classes.
        'drc_grazes': drc_grazes,
        # Smallest copper clearance any step actually routed at; downstream steps
        # and check_drc grade the board at this floor.
        'min_clearance_used': eff_clearance,
        # #1328: the stubs escaped below `clearance` (pad, net, the rung each
        # cleared at); their rows are also in design_rules.
        'escalated': list(LAST_ESCALATED),
    }
    # #846: what --allow-via-in-pad actually did. Before this, the only
    # machine-readable numbers a fanout run published were escape counts, so
    # neither the flag's effect nor a stub-length claim could be checked
    # without re-parsing the board. `via_in_pad` is the fab question, so it
    # agrees with the IPC-4761 note printed above it.
    summary['allow_via_in_pad'] = bool(getattr(args, 'allow_via_in_pad', False))
    if LAST_UNDERPAD_REPORT:
        summary['via_in_pad'] = LAST_UNDERPAD_REPORT.get('via_in_pad', 0)
        summary['via_in_pad_clamped'] = LAST_UNDERPAD_REPORT.get('clamped', 0)
        summary['via_in_pad_offcentre'] = LAST_UNDERPAD_REPORT.get(
            'via_in_pad_offcentre', 0)
        summary['max_stub_mm'] = LAST_UNDERPAD_REPORT.get('max_stub_mm', 0.0)
    # #1804/#1834: the run's PADS, surface fan and via-drop together
    # (requested/escaped/failed count NETS)
    summary.update(pad_counts(footprint.reference))
    try:                       # #653: env knobs into the machine-readable
        import env_knobs as _ek653   # summary, so a harness can detect a
        summary['env_knobs'] = _ek653.active_env_knobs()   # dirty baseline
    except Exception:          # without re-reading logs
        pass
    # #857: what this run delivered below a requested size (route.py's
    # key), when it delivered anything there.
    from fab_tiers import narrowed_summary, finish_design_rules
    _dr857 = narrowed_summary()
    if _dr857 is not None:
        summary['design_rules'] = _dr857
    print(f"JSON_SUMMARY: {_json.dumps(summary)}")
    return finish_design_rules(args, 0)


if __name__ == '__main__':
    exit(main())
