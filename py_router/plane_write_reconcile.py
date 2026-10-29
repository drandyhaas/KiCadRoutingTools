"""Write-list vs pcb_data reconciliation shared by both plane engines (#508).

The plane engines keep two parallel representations of this run's copper:
``pcb_data`` (what the router reasons about) and the write list
(``all_new_segments``/``all_new_vias`` dicts, written to the file or returned
to the GUI). A pass that removes copper from one without the other ships
either stale copper (a possible different-net SHORT: #463 spartan6_6layer)
or a phantom disconnect.

Two reconciliation mechanisms close the gap after an in-memory ripped-net
reconnect (``batch_route(return_results=True, pcb_data=...)``):

- ``consume_inner_strips``: the inner batch_route's cleanup removes
  superseded/orphaned copper from pcb_data and reports it in
  ``segments_to_remove``/``vias_to_remove``. Dropping that channel ships
  copper the board no longer has.
- ``drop_withdrawn_partial_restores``: a partial restore's kept-set is
  emitted into the write list BEFORE the reconnect runs; the reconnect may
  re-route the same net and delete that copper from pcb_data.

Both were built (and corpus-proven) in repair_planes; #508
finding 1 is that route_planes' GUI reconnect had NEITHER. They live here so
each engine drives the SAME function the tests pin (the parity-gate lesson:
a hand-mirrored copy silently drifts).
"""
from typing import Dict, List


def drop_withdrawn_partial_restores(emitted_segs, emitted_vias,
                                    all_new_segments, all_new_vias, pcb_data):
    """Drop partial-restore copper the ripped-net reconnect has since withdrawn.

    A partial restore's kept-set is emitted into the write list BEFORE the
    reconnect runs, and unconditionally -- but the same net is queued as a
    reconnect casualty, so the reconnect may RE-ROUTE it and delete that copper
    from pcb_data. The write list still carried it, so the OUTPUT shipped
    copper present in no in-memory state (#463 spartan6_6layer: /RAM/DDR-LDM's
    In3.Cu run written after the reconnect moved it to In1.Cu and gave the
    corridor to /RAM/DDR-D11 -- a 0.220mm collinear overlap, a hard short).

    pcb_data is authoritative once the reconnect and its restore-on-failure
    custody have run. Pure write-list filter: no routing, no obstacle work, no
    persistent structures, and indexed on the emitted nets only (a whole-board
    index would be ~all copper for a handful of lookups).

    Matching is by IDENTITY, not value: an equal-looking dict may be legitimate
    copper from another pass. `stale_*` holds the references across the filter,
    so the id() keys cannot be recycled underneath us.

    Mutates all_new_segments/all_new_vias in place; returns
    (n_segments_dropped, n_vias_dropped, sorted net names).
    """
    if not emitted_segs and not emitted_vias:
        return 0, 0, []
    pn = ({d['net_id'] for d in emitted_segs}
          | {d['net_id'] for d in emitted_vias})
    live_s = {(s.net_id, round(s.start_x, 3), round(s.start_y, 3),
               round(s.end_x, 3), round(s.end_y, 3), s.layer)
              for s in pcb_data.segments if s.net_id in pn}
    live_v = {(v.net_id, round(v.x, 3), round(v.y, 3))
              for v in pcb_data.vias if v.net_id in pn}
    stale_s = [d for d in emitted_segs
               if (d['net_id'], round(d['start'][0], 3), round(d['start'][1], 3),
                   round(d['end'][0], 3), round(d['end'][1], 3),
                   d['layer']) not in live_s]
    stale_v = [d for d in emitted_vias
               if (d['net_id'], round(d['x'], 3), round(d['y'], 3)) not in live_v]
    if not stale_s and not stale_v:
        return 0, 0, []
    sid = {id(d) for d in stale_s}
    vid = {id(d) for d in stale_v}
    all_new_segments[:] = [d for d in all_new_segments if id(d) not in sid]
    all_new_vias[:] = [d for d in all_new_vias if id(d) not in vid]
    names = sorted({(pcb_data.nets[d['net_id']].name
                     if d['net_id'] in pcb_data.nets else str(d['net_id']))
                    for d in stale_s + stale_v})
    return len(stale_s), len(stale_v), names


def consume_inner_strips(rdata, all_new_segments, all_new_vias, pcb_data,
                         file_strip_segments, file_strip_vias, label):
    """Board == write model (#XTAL_O zombie class): an in-memory batch_route's
    cleanup removes superseded/orphaned copper from pcb_data AND reports it in
    segments_to_remove/vias_to_remove -- dropping that channel ships copper the
    board no longer has (the write-list still carries it via the
    partial-restore emissions). Remove matching write-list entries; casualty
    nets' input-file copper is already wholly excluded by the writer/applier,
    so coordinate matching over the emissions is the complete consumption.

    Input copper of NON-excluded nets (#484): removing it from the write-list
    is not enough -- the writer re-emits input text, so these also go to the
    caller's per-segment strip channel (``file_strip_segments``/``_vias``,
    which the CLI writer and the GUI applier both honor).

    A stripped VALUE the inner run still holds is not removed copper (#1516):
    ``merge_collinear_segments`` rebuilds a whole chain as new objects,
    pieces that did not change included, and strips every original, so an
    unchanged piece's value sits both in the strip channel and in the inner
    run's own output -- which the caller has already appended to the write
    list. Such a value keeps exactly as many copies as that output holds
    (the inner run's own objects in pcb_data, the LAST-appended dicts in the
    write list, i.e. the ones the caller just emitted from it); any further
    copy -- the stale emission of the stripped original, a partial-restore
    duplicate -- goes, as every copy of a value nobody re-created does.

    Returns the number of write-list pieces dropped.
    """
    from collections import Counter
    rs = rdata.get('segments_to_remove') or []
    rv = rdata.get('vias_to_remove') or []
    if not rs and not rv:
        return 0
    file_strip_segments.extend(rs)
    file_strip_vias.extend(rv)

    def seg_key(a, b, layer, net_id):
        a = (round(a[0], 3), round(a[1], 3))
        b = (round(b[0], 3), round(b[1], 3))
        return (min(a, b), max(a, b), layer, net_id)

    def obj_skey(s):
        return seg_key((s.start_x, s.start_y), (s.end_x, s.end_y),
                       s.layer, s.net_id)

    def dict_skey(d):
        return seg_key(d['start'], d['end'], d['layer'], d['net_id'])

    def obj_vkey(v):
        return (round(v.x, 3), round(v.y, 3), v.net_id)

    def dict_vkey(d):
        return (round(d['x'], 3), round(d['y'], 3), d['net_id'])

    # The inner run's own output, exactly what the callers append to the
    # write list from it: every result's new copper plus the swap channels.
    live_s = [s for r in rdata.get('results') or []
              for s in (r.get('new_segments') or [])]
    live_s += list(rdata.get('all_swap_segments') or [])
    live_v = [v for r in rdata.get('results') or []
              for v in (r.get('new_vias') or [])]
    live_v += list(rdata.get('all_swap_vias') or [])

    def consume(items, key_of, stripped, live_keys, live_ids, prefer_live_ids):
        """Drop items keyed in `stripped`, keeping per key at most as many
        as `live_keys` holds -- the inner run's own objects first when
        `prefer_live_ids`, else the last-appended ones."""
        quota = Counter(k for k in live_keys if k in stripped)
        keep = set()
        if prefer_live_ids:
            for i, it in enumerate(items):
                k = key_of(it)
                if k in stripped and id(it) in live_ids and quota[k] > 0:
                    keep.add(i)
                    quota[k] -= 1
        for i in range(len(items) - 1, -1, -1):
            k = key_of(items[i])
            if k in stripped and i not in keep and quota[k] > 0:
                keep.add(i)
                quota[k] -= 1
        items[:] = [it for i, it in enumerate(items)
                    if key_of(it) not in stripped or i in keep]

    skeys = {obj_skey(s) for s in rs}
    vkeys = {obj_vkey(v) for v in rv}
    live_skeys = [obj_skey(s) for s in live_s]
    live_vkeys = [obj_vkey(v) for v in live_v]
    live_ids = {id(x) for x in live_s} | {id(x) for x in live_v}
    n0 = len(all_new_segments) + len(all_new_vias)
    if skeys:
        consume(all_new_segments, dict_skey, skeys, live_skeys, live_ids, False)
    if vkeys:
        consume(all_new_vias, dict_vkey, vkeys, live_vkeys, live_ids, False)
    # mirror out of pcb_data too (inner cleanup already removed its own
    # objects; kept-piece duplicates re-added via partial_restores may remain)
    if skeys:
        consume(pcb_data.segments, obj_skey, skeys, live_skeys, live_ids, True)
    if vkeys:
        consume(pcb_data.vias, obj_vkey, vkeys, live_vkeys, live_ids, True)
    n1 = len(all_new_segments) + len(all_new_vias)
    if n0 != n1:
        print(f"  {label}: consumed inner strip channel -- dropped "
              f"{n0 - n1} superseded write-list piece(s)")
    return n0 - n1
