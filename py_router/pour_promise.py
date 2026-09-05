"""#678: a pour-served ball is a COMMITMENT -- audit it after routing.

The BGA fanout's pour-direct serves a plane-net ball by fill contact instead
of a drop via, and records WHICH balls it promised in the sibling
``.kicad_pro`` (``protected_nets.POUR_SERVED_KEY``, carried down the chain by
the per-step project copy). Routing that runs later in the same chain can
carve such a ball's fill island off the sourced region, and the final refill
then ships it split. The composition-time invariant (#662 3b) cannot see
this: it runs before routing exists. This module is the post-route half of
the same contract.

``audit_pour_promises`` answers, for every promised ball still on the board,
"does its copper reach the net's sourced region?" against the AUTHORITATIVE
source available:

  * ``exact`` -- KiCad's own refill of the board file (``kicad_exact_fill``:
    deterministic, the fill KiCad will ship) clustered with the net's tracks,
    vias and pads by ``exact_clusters``; the sourced region is the primary
    cluster, exactly as the plane-finalize oracle defines it.
  * ``model``  -- the validator-parity raster fill model
    (``check_net_connectivity``'s fill-aware union-find) when no board file
    can be refilled (GUI mid-run without a staged file, no pcbnew).

A detached ball comes back with a WELD LINK anchored at the ball itself --
``(net, (x, y, pad_layer, 'pad'), (bx, by, layer, kind))`` -- the exact shape
the route step's final reconciliation routes as a forced edge with rip
authority (``route_oracle_links``). Anchoring at the BALL rather than at the
island's nearest fill sample matters twice: the ball's centre is the tap site
the fanout reserved for precisely this case (a via there is the drop the
promise displaced), and a pad-anchored endpoint can never be classed as
pad-less debris (#659) and dropped from custody.

Populations are always reported (promised / on board / checked / kept /
detached / stale), never a bare verdict.
"""
import os
from typing import Dict, List, Optional, Set, Tuple


def promised_pads_on_board(pcb_data, promises: Dict[str, dict]):
    """Resolve the promise list against THIS board.

    Returns (found, stale): ``found`` is a list of dicts
    ``{key, pad, net_id, net, layer, how}`` for promises whose ``REF.PAD``
    exists and still carries the promised net; ``stale`` lists
    ``{key, reason}`` with reason ``missing`` (no such pad), ``renamed``
    (the pad's net is not the promised one) or ``no_zone`` (the net owns no
    zone on this board any more -- there is no pour to keep the promise
    with, so the ball is an ordinary routed pad, not a finalize customer).
    """
    by_key = {}
    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            by_key.setdefault(f"{p.component_ref}.{p.pad_number}", p)
    zone_net_ids: Set[int] = {z.net_id for z in (getattr(pcb_data, 'zones', None) or [])}
    name_to_id = {n.name: nid for nid, n in pcb_data.nets.items()}
    found, stale = [], []
    for key in sorted(promises):
        spec = promises[key] or {}
        pad = by_key.get(key)
        if pad is None:
            stale.append({'key': key, 'reason': 'missing'})
            continue
        net_name = spec.get('net')
        if not net_name or pad.net_name != net_name:
            stale.append({'key': key, 'reason': 'renamed'})
            continue
        nid = name_to_id.get(net_name, pad.net_id)
        if nid not in zone_net_ids:
            stale.append({'key': key, 'reason': 'no_zone'})
            continue
        layer = spec.get('layer') or next(
            (l for l in (pad.layers or []) if l.endswith('.Cu')), None)
        found.append({'key': key, 'pad': pad, 'net_id': nid, 'net': net_name,
                      'layer': layer, 'how': spec.get('how', 'pour')})
    return found, stale


def _pad_layer(pad, default: Optional[str]) -> Optional[str]:
    lay = default
    cu = [l for l in (pad.layers or []) if l.endswith('.Cu')]
    if lay is None or (cu and lay not in cu and '*.Cu' not in (pad.layers or [])):
        lay = cu[0] if cu else lay
    return lay


def _link_to_points(entry, target_points, kind_b: str):
    """The weld link from a detached ball to the sourced region's copper:
    pad-anchored source, nearest target point (layer-aware, via points
    resolve to the pad's own layer -- a barrel spans it)."""
    from kicad_exact_fill import nearest_pair
    pad = entry['pad']
    lay = _pad_layer(pad, entry.get('layer'))
    if lay is None or not target_points:
        return None
    a, b = nearest_pair([(pad.global_x, pad.global_y, lay)], target_points)
    if b is None:
        return None
    bl = b[2] if len(b) > 2 and b[2] else lay
    return (entry['net'], (pad.global_x, pad.global_y, lay, 'pad'),
            (float(b[0]), float(b[1]), bl, kind_b))


def _audit_exact(pcb_data, found: List[dict], islands_map) -> Tuple[List[dict], List[dict]]:
    """(kept, detached) against KiCad's exact fill. ``islands_map`` is
    ``refill_islands``' ``{(net, layer): [poly, ...]}``."""
    from kicad_exact_fill import exact_clusters
    by_net: Dict[str, list] = {}
    for (net, layer), polys in islands_map.items():
        by_net.setdefault(net, []).extend((layer, p) for p in polys)
    kept, detached = [], []
    for nid in sorted({e['net_id'] for e in found}):
        entries = [e for e in found if e['net_id'] == nid]
        net_name = entries[0]['net']
        clusters = exact_clusters(pcb_data, nid, by_net.get(net_name, []))
        if len(clusters) <= 1:
            kept.extend(entries)
            continue
        primary = clusters[0]
        kind_b = 'zone' if primary['islands'] else 'track'
        loc = {}
        for ci, c in enumerate(clusters):
            for p in c['pads']:
                loc[(p.component_ref, p.pad_number)] = ci
        for e in entries:
            ci = loc.get((e['pad'].component_ref, e['pad'].pad_number))
            if ci == 0:
                kept.append(e)
                continue
            d = dict(e)
            d['link'] = _link_to_points(e, primary['points'], kind_b)
            detached.append(d)
    return kept, detached


def _audit_model(pcb_data, found: List[dict]) -> Tuple[List[dict], List[dict]]:
    """(kept, detached) against the raster fill model -- the fallback when
    no board file can be refilled. Over-credits in the castor class (a gap
    only KiCad's exact fill sees), so a 'kept' here is weaker evidence than
    an exact one; the caller discloses the source."""
    from check_connected import check_net_connectivity
    from kicad_oracle import _largest_track_component_points
    zones_by_net: Dict[int, list] = {}
    for z in (getattr(pcb_data, 'zones', None) or []):
        zones_by_net.setdefault(z.net_id, []).append(z)
    kept, detached = [], []
    for nid in sorted({e['net_id'] for e in found}):
        entries = [e for e in found if e['net_id'] == nid]
        segs = [s for s in pcb_data.segments if s.net_id == nid]
        vias = [v for v in pcb_data.vias if v.net_id == nid]
        pads = pcb_data.pads_by_net.get(nid, [])
        res = check_net_connectivity(nid, segs, vias, pads,
                                     zones_by_net.get(nid, []),
                                     pcb_data=pcb_data)
        if res.get('connected'):
            kept.extend(entries)
            continue
        bad = {(round(x, 3), round(y, 3), ref)
               for (x, y, _l, ref) in (res.get('disconnected_pads') or [])}
        target = list(_largest_track_component_points(pcb_data, nid) or [])
        if not target:
            # A track-less net (pour-served throughout): the connected
            # pads themselves are the sourced region.
            for p in pads:
                if (round(p.global_x, 3), round(p.global_y, 3),
                        p.component_ref) not in bad:
                    target.append((p.global_x, p.global_y, _pad_layer(p, None)))
        for e in entries:
            p = e['pad']
            if (round(p.global_x, 3), round(p.global_y, 3),
                    p.component_ref) not in bad:
                kept.append(e)
                continue
            d = dict(e)
            d['link'] = _link_to_points(e, target, 'track')
            detached.append(d)
    return kept, detached


def audit_pour_promises(pcb_data, promises: Dict[str, dict],
                        board_file: Optional[str] = None,
                        project_from: Optional[str] = None,
                        zone_net_names: Optional[Set[str]] = None) -> dict:
    """Audit every promised ball on ``pcb_data``.

    ``board_file``: a board file consistent with ``pcb_data`` (the file the
    engine just wrote, or a staged save of the live board) that KiCad can
    refill -- the exact source. None, or a refill failure, falls back to the
    raster model. ``zone_net_names`` scopes the audit to the finalize's zone
    nets when given (a promised net outside this route's --nets scope is
    excluded BY PLAN, like the rest of the finalize, and reported ``stale``
    with reason ``out_of_scope``).

    Returns ``{'source': 'exact'|'model'|'none', 'promised', 'on_board',
    'checked', 'kept': [keys], 'detached': [{key, net, layer, how, x, y,
    link}], 'stale': [{key, reason}], 'why': str|None}``.
    """
    out = {'source': 'none', 'promised': len(promises or {}), 'on_board': 0,
           'checked': 0, 'kept': [], 'detached': [], 'stale': [], 'why': None}
    if not promises:
        return out
    found, stale = promised_pads_on_board(pcb_data, promises)
    out['on_board'] = len(found)
    if zone_net_names is not None:
        in_scope = [e for e in found if e['net'] in zone_net_names]
        stale.extend({'key': e['key'], 'reason': 'out_of_scope'}
                     for e in found if e['net'] not in zone_net_names)
        found = in_scope
    out['stale'] = stale
    if not found:
        return out
    kept = detached = None
    if board_file and os.path.isfile(board_file):
        try:
            import env_knobs
            if not env_knobs.NO_EXACT_FILL:
                from kicad_exact_fill import refill_islands_ex
                islands, status = refill_islands_ex(board_file,
                                                    project_from=project_from)
                if status.ok and islands is not None:
                    kept, detached = _audit_exact(pcb_data, found, islands)
                    out['source'] = 'exact'
                else:
                    out['why'] = f"exact fill unavailable: {status.why()}"
            else:
                out['why'] = 'exact fill disabled (KICAD_NO_EXACT_FILL)'
        except Exception as e:
            out['why'] = f"exact fill failed: {e}"
    elif board_file:
        out['why'] = 'no board file to refill'
    else:
        out['why'] = 'no board file to refill'
    if kept is None:
        try:
            kept, detached = _audit_model(pcb_data, found)
            out['source'] = 'model'
        except Exception as e:
            out['why'] = (out['why'] or '') + f"; model audit failed: {e}"
            return out
    out['checked'] = len(kept) + len(detached)
    out['kept'] = [e['key'] for e in kept]
    out['detached'] = [{'key': e['key'], 'net': e['net'], 'layer': e['layer'],
                        'how': e['how'], 'x': e['pad'].global_x,
                        'y': e['pad'].global_y, 'link': e.get('link')}
                       for e in detached]
    return out


def format_audit(audit: dict, stage: str) -> str:
    """One console line with the populations, never a bare verdict."""
    det = audit.get('detached') or []
    stale = audit.get('stale') or []
    parts = [f"Pour-served balls (#678, {stage}): {audit.get('promised', 0)} "
             f"promised, {audit.get('checked', 0)} checked "
             f"({audit.get('source')}), {len(audit.get('kept') or [])} kept, "
             f"{len(det)} DETACHED"]
    if det:
        parts.append(": " + ", ".join(f"{d['key']} ({d['net']})" for d in det[:12])
                     + ("..." if len(det) > 12 else ""))
    if stale:
        reasons: Dict[str, int] = {}
        for s in stale:
            reasons[s['reason']] = reasons.get(s['reason'], 0) + 1
        parts.append("; stale " + ", ".join(f"{k} {v}" for k, v in sorted(reasons.items())))
    if audit.get('why'):
        parts.append(f" [{audit['why']}]")
    return "".join(parts)


def summary_entry(audit: dict) -> dict:
    """The JSON_SUMMARY-shaped view (no Pad objects, links as lists)."""
    return {
        'source': audit.get('source'),
        'promised': audit.get('promised', 0),
        'checked': audit.get('checked', 0),
        'kept': len(audit.get('kept') or []),
        'detached': [d['key'] for d in (audit.get('detached') or [])],
        'stale': {s['key']: s['reason'] for s in (audit.get('stale') or [])},
        'why': audit.get('why'),
    }
