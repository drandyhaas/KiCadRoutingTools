"""Tracks and vias lying ON another net's pad (#1680): measure, repair, report.

A part the clearance step could not move off fanout copper keeps a foreign
via or track in contact with its pad: a short, which KiCad grades
`shorting_items`. Nothing else in a route step removes it, because the copper
is the input's and a net that reads connected never routes. So:

* ``pad_contacts`` is the one measure, check_drc's own pad arms at zero
  clearance, so a row here is an item check_drc marks ``short``.
* ``rip_pad_contacts`` runs before a route step's already-connected filter. It
  takes the offending via or track out of the board together with the stub
  that leads to it, so its net is open and routes clear of the pad like any
  other net (the router prices the pad as an obstacle). Only copper of a net
  the step was asked to route: nothing else would reconnect it. Locked and
  sense-held copper and protected nets stay, under the rules every other rip
  obeys (#521, #1279), and an open net is preferred to a short: the net is not
  given its copper back if the reroute fails, and the run's own tally names it.
* ``shipped_report`` is the audit of the board the run ships, printed and
  carried in ``JSON_SUMMARY_MIN`` and ``--json-out``: ``pad_contacts``, a short
  whose net was outside the step or that is protected, and ``pad_grazes``, copper
  inside the clearance of another net's pad, measured at the floor the run
  writes back (so a pair a net class raises above it is check_drc's to name). A
  part the clearance step could not clear is thereby said at the end of the run
  as well as in that step's log (#1681). Report-only.
"""
from __future__ import annotations

import math
from typing import Dict, List, Set

from kicad_parser import is_immutable_copper


def _row(pcb, pad, kind, item, net_id, at, overlap, clearance, layer=None):
    return {
        'item': item,
        'short': overlap - clearance > 1e-6,
        'pad': f'{pad.component_ref}.{pad.pad_number}',
        'pad_net': pad.net_name or '',
        'net': pcb.nets[net_id].name if net_id in pcb.nets else f'net_{net_id}',
        'net_id': net_id,
        'kind': kind,
        'at': [round(at[0], 4), round(at[1], 4)],
        'overlap_mm': round(overlap, 4),
        **({'clearance_mm': clearance} if clearance else {}),
        **({'layer': layer} if layer else {}),
    }


def pad_contacts(pcb, segments=None, vias=None, clearance=0.0) -> List[dict]:
    """Every via and track in contact with the copper of a pad of another net,
    sorted by pad; `item` is the copper object. `segments` / `vias` measure a
    copper list other than the board's own (the GUI's write model). A net tie's
    partner pad is exempt, as check_drc and KiCad exempt it. With a `clearance`
    a row also covers copper inside it (`short` false), at the 10% margin
    check_drc takes to filter grid quantization, and `overlap_mm` is measured
    from it."""
    margin = 0.1 if clearance else 0.0
    from check_drc import (_pad_has_no_copper, check_pad_segment_overlap,
                           check_pad_via_overlap)
    segments = pcb.segments if segments is None else segments
    vias = pcb.vias if vias is None else vias
    layers = list(getattr(pcb.board_info, 'copper_layers', None) or [])
    # A pad with no net shorts no two nets (KiCad grades that contact as a
    # clearance item): a board may stitch a shield tab that way on purpose.
    pads = [p for fp in pcb.footprints.values() for p in fp.pads
            if p.net_id and not _pad_has_no_copper(p)]
    if not pads:
        return []
    reach = max(math.hypot(p.size_x, p.size_y) / 2.0 for p in pads)
    cell = max(2.0 * reach, 1.0)
    grid: Dict[tuple, list] = {}
    for p in pads:
        grid.setdefault((int(p.global_x // cell), int(p.global_y // cell)),
                        []).append(p)

    def near(x0, y0, x1, y1, r):
        for cx in range(int((x0 - r) // cell) - 1, int((x1 + r) // cell) + 2):
            for cy in range(int((y0 - r) // cell) - 1, int((y1 + r) // cell) + 2):
                yield from grid.get((cx, cy), ())

    rows = []
    for v in vias:
        if not v.net_id:
            continue    # an orphan barrel: no net for a contact to short
        exempt = pcb.net_tie_exempt_pad_ids(v.net_id)
        for p in near(v.x, v.y, v.x, v.y, v.size / 2.0 + clearance):
            if p.net_id == v.net_id or id(p) in exempt:
                continue
            hit, overlap = check_pad_via_overlap(p, v, clearance, layers, margin)
            if hit:
                rows.append(_row(pcb, p, 'via', v, v.net_id, (v.x, v.y), overlap,
                                 clearance))
    for s in segments:
        if not s.net_id or getattr(s, 'graphic', False):
            continue
        exempt = pcb.net_tie_exempt_pad_ids(s.net_id)
        for p in near(min(s.start_x, s.end_x), min(s.start_y, s.end_y),
                      max(s.start_x, s.end_x), max(s.start_y, s.end_y),
                      s.width / 2.0 + clearance):
            if p.net_id == s.net_id or id(p) in exempt:
                continue
            hit, overlap, pt = check_pad_segment_overlap(p, s, clearance, layers,
                                                         margin)
            if hit:
                rows.append(_row(pcb, p, 'track', s, s.net_id,
                                 pt or (s.start_x, s.start_y), overlap, clearance,
                                 s.layer))
    rows.sort(key=lambda r: (r['pad'], r['net'], r['kind'], r['at']))
    return rows


def _is_node(pcb, net_id, x, y, tracks_here, gone_vias):
    """A run of track stops here: at a pad or a surviving via of its net, or
    at a junction."""
    if tracks_here > 2:
        return True
    for v in pcb.vias:
        if (v.net_id == net_id and id(v) not in gone_vias
                and math.hypot(v.x - x, v.y - y) < v.size / 2.0):
            return True
    from check_drc import point_to_pad_distance
    return any(point_to_pad_distance(x, y, p) <= 1e-3
               for p in pcb.pads_by_net.get(net_id, ()))


def _end(x, y):
    return round(x, 3), round(y, 3)


def _stub(pcb, seed, tracks, gone_vias):
    """The tracks of the run from `seed` out to the first node at either end:
    what the via or track on a pad leaves dangling once it goes."""
    run, stack = {id(seed): seed}, [seed]
    while stack:
        s = stack.pop()
        for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            here = [t for t in tracks if t.layer == s.layer and id(t) != id(s)
                    and _end(x, y) in (_end(t.start_x, t.start_y),
                                       _end(t.end_x, t.end_y))]
            if _is_node(pcb, s.net_id, x, y, len(here) + 1, gone_vias):
                continue
            for t in here:
                if id(t) not in run:
                    run[id(t)] = t
                    stack.append(t)
    return list(run.values())


def rip_pad_contacts(pcb, scope_ids: Set[int], input_file=None,
                     overrides=None) -> List[dict]:
    """Take out of `pcb` the copper in contact with a foreign pad, for the nets
    in `scope_ids`, with the stubs that led to it. Returns one record per
    contact it parted, with what it removed; the input copper's custody (the
    strip from the written file) is the caller's, as for --force-reroute."""
    rows = pad_contacts(pcb)
    if not rows:
        return []
    from protected_nets import filter_rippable_names, protection_map
    prot = protection_map(pcb, input_file)
    names = sorted({r['net'] for r in rows if r['net_id'] in scope_ids})
    keep = set(filter_rippable_names(names, prot,
                                     override_patterns=list(overrides or ()),
                                     context='pad contact')) if prot else set(names)
    parted = []
    removed_v, removed_s = set(), set()
    for r in rows:
        if r['net_id'] not in scope_ids or r['net'] not in keep:
            continue
        nid, item = r['net_id'], r['item']
        if (is_immutable_copper(item)
                or id(item) in (removed_v if r['kind'] == 'via' else removed_s)):
            continue
        by_net = [s for s in pcb.segments
                  if s.net_id == nid and id(s) not in removed_s
                  and not getattr(s, 'graphic', False)]
        if r['kind'] == 'via':
            # What ends in the barrel is what the via served.
            seeds = [s for s in by_net
                     if min(math.hypot(s.start_x - item.x, s.start_y - item.y),
                            math.hypot(s.end_x - item.x, s.end_y - item.y))
                     < item.size / 2.0]
            vias = [item]
        else:
            seeds, vias = [item], []
        gone = removed_v | {id(v) for v in vias}
        stubs = {}
        for s in seeds:
            for t in _stub(pcb, s, by_net, gone):
                stubs[id(t)] = t
        if any(is_immutable_copper(t) for t in stubs.values()):
            continue
        removed_v = gone
        removed_s.update(stubs)
        parted.append({k: r[k] for k in ('pad', 'pad_net', 'net', 'kind', 'at')}
                      | {'removed_vias': len(vias),
                         'removed_tracks': len(stubs)})
    if parted:
        pcb.vias = [v for v in pcb.vias if id(v) not in removed_v]
        pcb.segments = [s for s in pcb.segments if id(s) not in removed_s]
        from pcb_modification import bump_copper_epoch
        bump_copper_epoch(pcb)
    return parted


def format_rows(rows: List[dict], limit: int = 8) -> List[str]:
    """One console line per contact, capped with an explicit remainder."""
    def line(r):
        where = f"{r['kind']} at ({r['at'][0]:.2f}, {r['at'][1]:.2f})"
        if r.get('short', True):
            return (f"{r['pad']} ({r['pad_net']}) on a {r['net']} {where}, "
                    f"overlap {r['overlap_mm'] - r.get('clearance_mm', 0.0):.3f} mm")
        return (f"{r['pad']} ({r['pad_net']}) {r['overlap_mm']:.3f} mm inside "
                f"the {r['clearance_mm']:g} mm clearance of a {r['net']} {where}")
    out = [line(r) for r in rows[:limit]]
    if len(rows) > limit:
        out.append(f"... and {len(rows) - limit} more")
    return out


def shipped_report(pcb, segments=None, vias=None, clearance=0.0) -> List[dict]:
    """`pad_contacts` as JSON-ready rows."""
    return [{k: v for k, v in r.items() if k not in ('net_id', 'item')}
            for r in pad_contacts(pcb, segments, vias, clearance)]


def audit_shipped(pcb, segments=None, vias=None, clearance=0.0):
    """The end of a route run's audit: says what it found and returns (shorts,
    grazes), each [] when clean. An audit that cannot run says so and returns
    ([], []), since it only reports."""
    try:
        rows = shipped_report(pcb, segments, vias, clearance)
    except Exception as exc:                                    # noqa: BLE001
        print(f"  WARNING: pad-contact audit skipped "
              f"({type(exc).__name__}: {exc})")
        return [], []
    shorts = [r for r in rows if r['short']]
    grazes = [r for r in rows if not r['short']]
    from terminal_colors import RED, RESET
    for found, head in ((shorts, "SHORTED on the board this run ships: {n} "
                                 "via(s)/track(s) on another net's pad:"),
                        (grazes, "NOT CLEAR on the board this run ships: {n} "
                                 "via(s)/track(s) inside the clearance of "
                                 "another net's pad:")):
        if found:
            print(f"\n{RED}  {head.format(n=len(found))}{RESET}")
            for line in format_rows(found):
                print(f"  {RED}{line}{RESET}")
    return shorts, grazes
