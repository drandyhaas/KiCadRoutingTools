"""#1925: the stub a via leaves on each side of the layers its net uses.

A via carries its signal between the outermost copper layers its net joins
it on. The barrel past those layers, out to the via's end on each side, is a
stub. `via_stub_report` measures both stubs of every via through the board's
stackup, with the back-drills the board already declares
(`Via.backdrills`) taken off the barrel. It is a measurement only: it writes
nothing and decides nothing.

What the numbers are:

- A net's copper JOINS a via on a layer where a track of the net touches the
  via's pad there, a pad of the net overlaps it on that layer, or a pour of
  the net holds the via's centre inside its outline (the outline, not the
  fill: a filled island the outline does not reach is not read).
- The top stub runs from the top surface of the highest joined layer up to
  the barrel's top end; the bottom stub from the bottom surface of the lowest
  joined layer down to the barrel's bottom end. A via joined on one layer has
  both; a via joined on none has neither and is listed as `unjoined`.
- A back-drill removes the layers from its start to its end layer, both
  inclusive (KiCad's own reading). A run of removed layers that begins at the
  via's end moves the barrel's end to the far surface of the deepest removed
  layer: the dielectric between it and the next layer still counts as stub,
  because the file does not say where in that dielectric the drill stops. So
  a back-drilled stub is the LONGEST the declared drill can leave.
- A layer the net joins the via on and a back-drill removes is a fault of the
  board (KiCad's DRC: a track connected to a back-drilled layer). It is named
  (`joined_on_drilled`) and the stubs are measured from the joined layers the
  drill leaves.
"""
from typing import Dict, Iterable, List, Optional


def copper_depths(pcb_data) -> Dict[str, tuple]:
    """{copper layer: (depth of its top surface, depth of its bottom surface)}
    in mm from the top of the stackup, or {} for a board with no stackup."""
    out = {}
    z = 0.0
    for layer in (getattr(pcb_data.board_info, 'stackup', None) or []):
        t = float(layer.thickness or 0.0)
        if layer.layer_type == 'copper' or layer.name.endswith('.Cu'):
            out.setdefault(layer.name, (z, z + t))
        z += t
    return out


def drilled_layers(backdrills, copper: List[str]) -> List[str]:
    """The layers of `copper` (stack order) the back-drills remove: each
    drill's start to end layer, inclusive. A drill naming a layer the board
    does not have removes nothing."""
    rank = {name: i for i, name in enumerate(copper)}
    gone = set()
    for bd in (backdrills or {}).values():
        _size, start, end = bd
        if start in rank and end in rank:
            a, b = sorted((rank[start], rank[end]))
            gone.update(copper[a:b + 1])
    return [name for name in copper if name in gone]


def _point_in_ring(x: float, y: float, ring) -> bool:
    inside = False
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i][0], ring[i][1]
        x2, y2 = ring[(i + 1) % n][0], ring[(i + 1) % n][1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def joined_layers(via, segments, pads, zones, copper: List[str]) -> Dict[str, List[str]]:
    """{layer: how the net's copper joins `via` there} over the layers the via
    has copper on, in stack order; each value lists 'track', 'pad' and / or
    'zone'. `segments`, `pads` and `zones` are the via's net's."""
    from geometry_utils import point_to_segment_distance, via_copper_layers
    from net_queries import expand_pad_layers
    from check_drc import point_to_pad_distance
    on = via_copper_layers(via, copper)
    how: Dict[str, List[str]] = {}
    r = via.size / 2.0
    for s in segments:
        if s.layer not in on or 'track' in how.get(s.layer, ()):
            continue
        d = point_to_segment_distance(via.x, via.y, s.start_x, s.start_y,
                                      s.end_x, s.end_y)
        if d < r + s.width / 2.0:
            how.setdefault(s.layer, []).append('track')
    for p in pads:
        if p.pad_type == 'np_thru_hole':
            continue
        if not getattr(p, 'polygons', None) \
                and max(abs(p.global_x - via.x), abs(p.global_y - via.y)) \
                > r + p.size_x + p.size_y:
            continue
        if point_to_pad_distance(via.x, via.y, p) >= r:
            continue
        for layer in expand_pad_layers(p.layers, copper):
            if layer in on and 'pad' not in how.get(layer, ()):
                how.setdefault(layer, []).append('pad')
    for z in zones:
        if z.layer not in on or 'zone' in how.get(z.layer, ()):
            continue
        if _point_in_ring(via.x, via.y, z.polygon) and not any(
                _point_in_ring(via.x, via.y, h) for h in (z.holes or ())):
            how.setdefault(z.layer, []).append('zone')
    return {layer: how[layer] for layer in on if layer in how}


def via_stub(via, joined: Iterable[str], copper: List[str],
             depths: Dict[str, tuple]) -> dict:
    """One via's stubs. `joined` is the layers its net joins it on
    (`joined_layers`), `depths` is `copper_depths`. The mm values are None
    when the stackup does not give every layer of the via a depth, or the via
    is joined on no layer a back-drill leaves."""
    from geometry_utils import via_copper_layers
    on = via_copper_layers(via, copper)
    gone = set(drilled_layers(getattr(via, 'backdrills', None), copper))
    joined = [l for l in on if l in set(joined)]
    kept = [l for l in joined if l not in gone]
    # The barrel's ends: the run of removed layers that starts at each end.
    top = 0
    while top < len(on) and on[top] in gone:
        top += 1
    bottom = len(on) - 1
    while bottom >= 0 and on[bottom] in gone:
        bottom -= 1
    row = {'span': [on[0], on[-1]] if on else [],
           'joined': joined,
           'drilled': [l for l in on if l in gone],
           'joined_on_drilled': [l for l in joined if l in gone],
           'unjoined': not joined,
           'stub_top_mm': None, 'stub_bottom_mm': None,
           'stub_top_layers': [], 'stub_bottom_layers': []}
    if not kept or top > bottom:
        return row
    i, j = on.index(kept[0]), on.index(kept[-1])
    row['stub_top_layers'] = on[top:i]
    row['stub_bottom_layers'] = on[j + 1:bottom + 1]
    if any(l not in depths for l in on):
        return row
    top_end = depths[on[top - 1]][1] if top else depths[on[0]][0]
    bottom_end = (depths[on[bottom + 1]][0] if bottom < len(on) - 1
                  else depths[on[-1]][1])
    row['stub_top_mm'] = round(max(0.0, depths[kept[0]][0] - top_end), 6)
    row['stub_bottom_mm'] = round(max(0.0, bottom_end - depths[kept[-1]][1]), 6)
    return row


def via_stub_report(pcb_data, net_ids=None, max_stub: Optional[float] = None) -> dict:
    """Every via of `net_ids` (None: every net with a via) with its two stubs,
    the longest stub per net, and the vias whose longer stub exceeds
    `max_stub` mm (None: no threshold, `over` stays None).

    Keys: `vias` (one row each: net, x, y, size, drill, via_type, backdrills
    and `via_stub`'s keys, plus `longest_mm` and `over`), `by_net`
    ({net: {vias, longest_mm, over, unjoined, joined_on_drilled}}),
    `measured` / `unmeasured` (via counts), `over` (count, or None),
    `backdrilled_vias`, `pad_backdrills` (the drilled pads that declare a
    back-drill; a pad's stub is not measured), and `reason` when no via could
    be measured in mm."""
    copper = list(pcb_data.board_info.copper_layers or [])
    depths = copper_depths(pcb_data)
    wanted = set(net_ids) if net_ids is not None else None
    vias_by, segs_by, zones_by = {}, {}, {}
    for v in pcb_data.vias:
        if wanted is None or v.net_id in wanted:
            vias_by.setdefault(v.net_id, []).append(v)
    for s in pcb_data.segments:
        if s.net_id in vias_by and not getattr(s, 'graphic', False):
            segs_by.setdefault(s.net_id, []).append(s)
    for z in (getattr(pcb_data, 'zones', None) or []):
        if z.net_id in vias_by:
            zones_by.setdefault(z.net_id, []).append(z)
    pads_by = getattr(pcb_data, 'pads_by_net', None) or {}

    def name(nid):
        net = pcb_data.nets.get(nid)
        return net.name if net is not None else ''

    rows, by_net = [], {}
    for nid in sorted(vias_by, key=lambda n: (name(n), n)):
        net_row = {'vias': 0, 'longest_mm': None, 'over': 0 if max_stub is not None else None,
                   'unjoined': 0, 'joined_on_drilled': 0}
        for v in vias_by[nid]:
            how = joined_layers(v, segs_by.get(nid, ()), pads_by.get(nid) or (),
                                zones_by.get(nid, ()), copper)
            row = via_stub(v, how, copper, depths)
            stubs = [m for m in (row['stub_top_mm'], row['stub_bottom_mm'])
                     if m is not None]
            longest = max(stubs) if stubs else None
            over = (None if max_stub is None or longest is None
                    else longest > max_stub + 1e-9)
            rows.append({'net': name(nid), 'x': v.x, 'y': v.y, 'size': v.size,
                         'drill': v.drill,
                         'via_type': getattr(v, 'via_type', 'through') or 'through',
                         'backdrills': {t: {'size': bd[0], 'start_layer': bd[1],
                                            'end_layer': bd[2]}
                                        for t, bd in (getattr(v, 'backdrills', None)
                                                      or {}).items()},
                         'joined_by': how, **row,
                         'longest_mm': longest, 'over': over})
            net_row['vias'] += 1
            net_row['unjoined'] += row['unjoined']
            net_row['joined_on_drilled'] += bool(row['joined_on_drilled'])
            if longest is not None and (net_row['longest_mm'] is None
                                        or longest > net_row['longest_mm']):
                net_row['longest_mm'] = longest
            if over:
                net_row['over'] += 1
        by_net[name(nid)] = net_row

    measured = sum(1 for r in rows if r['longest_mm'] is not None)
    reason = None
    if rows and not measured:
        if not depths:
            reason = 'the board has no stackup, so a stub has no length in mm'
        elif any(l not in depths for l in copper):
            reason = ('the stackup gives no thickness for '
                      + ', '.join(l for l in copper if l not in depths))
    pad_drills = [{'ref': p.component_ref, 'pad': p.pad_number, 'net': p.net_name,
                   'backdrills': {t: {'size': bd[0], 'start_layer': bd[1],
                                      'end_layer': bd[2]}
                                  for t, bd in p.backdrills.items()}}
                  for fp in pcb_data.footprints.values() for p in fp.pads
                  if getattr(p, 'backdrills', None)
                  and (wanted is None or p.net_id in wanted)]
    return {'max_stub_mm': max_stub, 'vias': rows, 'by_net': by_net,
            'measured': measured, 'unmeasured': len(rows) - measured,
            'over': (sum(1 for r in rows if r['over'])
                     if max_stub is not None else None),
            'backdrilled_vias': sum(1 for r in rows if r['backdrills']),
            'pad_backdrills': pad_drills, 'reason': reason}
