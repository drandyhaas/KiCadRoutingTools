"""Routing terminals for net-tagged copper ART that reaches none of its pads.

KiCad conducts through a net-tagged copper graphic (#337) and links every
copper cluster of a net, so an art island -- art of the net touching none of
its pads -- is an open net in KiCad's DRC on every run (#659). The router
never targeted art, so such a net stayed open however it was routed; a
one-pad net whose only other copper is art (a test land drawn as two
graphic discs, one per side, which the human joins to the pad with a track
and a via) was not even a routing candidate.

`add_art_terminals` gives each island a terminal ON its own copper: a
one-pad footprint (reference ``art~<net>~<k>``, pad on the island's layer),
added to the PCBData the way a real part is, so every consumer that maps a
pad to its footprint finds one. The net then routes to the island like any
other pad; islands on two layers get a terminal each, so the router joins
them with a via. `remove_art_terminals` takes them out again: batch_route
adds them for its run and removes them before it returns, so neither front's
board keeps them. A net's own art is no obstacle to it (the obstacle map
skips the copper of nets being routed), so the terminal is reachable.

Where on the island: a filled circle's centre, a filled shape's interior
point nearest its outline's centroid, else the middle of the island's
longest stroke -- each a point KiCad's connectivity joins a track end at.
"""
import math

ART_REF_PREFIX = 'art~'


def _anchor(segs):
    """(x, y, size) of a terminal on an art island's copper."""
    from check_connected import point_in_polygon
    for s in segs:
        circ = getattr(s, 'graphic_circle', None)
        if circ is not None and getattr(s, 'graphic_filled', False):
            return circ[0], circ[1], max(min(2.0 * circ[2], 1.0), s.width)
    for s in segs:
        ring = getattr(s, 'graphic_ring', None)
        if ring and getattr(s, 'graphic_filled', False):
            cx = sum(p[0] for p in ring) / len(ring)
            cy = sum(p[1] for p in ring) / len(ring)
            if point_in_polygon(cx, cy, list(ring)):
                return cx, cy, max(s.width, 0.2)
            break
    s = max(segs, key=lambda q: math.hypot(q.end_x - q.start_x, q.end_y - q.start_y))
    return ((s.start_x + s.end_x) / 2.0, (s.start_y + s.end_y) / 2.0,
            max(getattr(s, 'drawn_width', 0.0) or s.width, 0.1))


def art_islands(pcb_data, net_id):
    """[[segment, ...], ...]: the net's art clusters that reach none of its
    pads and none of its pours (one per connected cluster, so one per layer
    unless the art is joined across layers)."""
    from check_connected import check_net_connectivity
    from geometry_utils import UnionFind
    segs = [s for s in pcb_data.segments if s.net_id == net_id]
    if not any(getattr(s, 'graphic', False) for s in segs):
        return []
    pads = pcb_data.pads_by_net.get(net_id, [])
    if not pads:
        return []
    vias = [v for v in pcb_data.vias if v.net_id == net_id]
    zones = [z for z in (getattr(pcb_data, 'zones', None) or []) if z.net_id == net_id]
    r = check_net_connectivity(net_id, segs, vias, pads, zones,
                               return_graph=True, pcb_data=pcb_data)
    g = r.get('graph') or {}
    uf = UnionFind()
    for a, b in g.get('edges', []) or []:
        uf.union(a, b)
    live = {uf.find(x) for x in (g.get('pad_index_repr') or {}).values()} | \
           {uf.find(x) for x in (g.get('zone_index_repr') or {}).values()}
    islands = {}
    for i, s in enumerate(segs):
        if not getattr(s, 'graphic', False) or getattr(s, 'owner_ref', ''):
            continue                     # board-level art only (#337)
        root = uf.find(2 * i)
        if root not in live:
            islands.setdefault(root, []).append(s)
    return list(islands.values())


def add_art_terminals(pcb_data, net_filter=None):
    """Add a terminal footprint per art island of every net matching
    `net_filter` (patterns; None = every net) that has an on-board pad.
    Returns the references added; idempotent (a board already carrying them
    gets none more)."""
    from kicad_parser import Footprint, Pad
    from check_drc import make_off_board_test
    from net_queries import matches_net_filter
    if getattr(pcb_data, '_art_terminal_refs', None):
        return []
    art_nets = {s.net_id for s in pcb_data.segments
                if getattr(s, 'graphic', False) and s.net_id
                and not getattr(s, 'owner_ref', '')}
    if not art_nets:
        return []
    off = make_off_board_test(pcb_data.board_info)
    # A poured net's art is the pour's to join: the fill reaches the art it
    # covers, so a terminal there routes copper the plane already supplies
    # and crowds the nets around it.
    poured = {z.net_id for z in (getattr(pcb_data, 'zones', None) or []) if z.net_id}
    added = []
    for nid in sorted(art_nets - poured):
        net = pcb_data.nets.get(nid)
        if net is None or not net.name:
            continue
        if net_filter and not matches_net_filter(net.name, list(net_filter)):
            continue
        pads = pcb_data.pads_by_net.get(nid, [])
        if not any(off is None or not off(p.global_x, p.global_y) for p in pads):
            continue
        for k, isl in enumerate(art_islands(pcb_data, nid), 1):
            x, y, size = _anchor(isl)
            if off is not None and off(x, y):
                continue
            layer = isl[0].layer
            ref = f'{ART_REF_PREFIX}{net.name}~{k}'
            pad = Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
                      local_x=0.0, local_y=0.0, size_x=size, size_y=size,
                      shape='circle', layers=[layer], net_id=nid,
                      net_name=net.name, pad_type='smd')
            pcb_data.footprints[ref] = Footprint(
                reference=ref, footprint_name='(art island)', x=x, y=y,
                rotation=0.0, layer=layer, pads=[pad])
            pcb_data.pads_by_net.setdefault(nid, []).append(pad)
            net.pads.append(pad)
            added.append(ref)
    if added:
        pcb_data._art_terminal_refs = list(added)
        print(f"  Art islands: {len(added)} net-tagged copper art island(s) "
              f"reaching none of their net's pads get a routing terminal "
              f"(#659): {', '.join(added[:6])}{', ...' if len(added) > 6 else ''}")
    return added


def remove_art_terminals(pcb_data):
    """Take back what `add_art_terminals` added to this board."""
    refs = getattr(pcb_data, '_art_terminal_refs', None)
    if not refs:
        return
    for ref in refs:
        fp = pcb_data.footprints.pop(ref, None)
        if fp is None:
            continue
        for pad in fp.pads:
            lst = pcb_data.pads_by_net.get(pad.net_id)
            if lst is not None:
                lst[:] = [p for p in lst if p is not pad]
            net = pcb_data.nets.get(pad.net_id)
            if net is not None:
                net.pads[:] = [p for p in net.pads if p is not pad]
    pcb_data._art_terminal_refs = None
