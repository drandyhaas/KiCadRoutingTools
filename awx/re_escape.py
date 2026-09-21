"""re_escape.py -- THE RE-ESCAPE (2026-09-21, Andy: "the long west tooth can
clearly be removed after the fact by a re-lay").

A lane's source STUB is the fanout's copper and the braid never rips it;
the write-time source trim (braid.note_source_joint) can only splice a
lane that ran back along its own stub. K36's SCAS is the other case: the
fanout escaped its ball 5.5 mm WEST through the array's channels to a
corner tooth pointing away from the DDR, the lane then looped 42.6 mm
round the bottom -- 54 mm of copper for an 18.7 mm airline, where the
human drops a dogbone at the ball and runs south under the array, 38 mm.

At write time, for a lane whose stub + lane exceed the PAD-to-berth
airline by BRAID_RE_ESCAPE mm: the lane and its stub chain are lifted,
the net is routed again from its source PAD to its berth, free, against
every other piece of copper as it stands, and the new copper ships only
when it is cheaper under the economy's rate (ECON_MM_PER_VIA of copper a
via) and the net's scoped DRC is no worse. Pure output economy: the
routed world is untouched, as the trim's is."""
import math
import os

RE_ESCAPE_DEFAULT = 8.0                                             # mm over the pad->berth airline (2026-09-21, on by default)
RE_ESCAPE = float(os.environ.get('BRAID_RE_ESCAPE', str(RE_ESCAPE_DEFAULT)) or 0)   # 0 = off


def _mm(segs):
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)


def excess(ctx, nm, lane):
    """How far the lane + its source stub run over the pad-to-berth
    airline, in mm (0 when the net has no stub chain): the order the pass
    takes the lanes in, largest first -- each re-escape takes room, and
    SCAS's route from its pad was there until SRAS, the adjacent ball
    re-escaped just before it, took the channel (K36)."""
    chain = ctx.src_chain.get(nm) or []
    if not chain or not lane:
        return 0.0
    nid, net = ctx.byname[nm]
    sref = ctx.src_ref.get(nm)
    tooth = ctx.ends[nm][0]
    pads = [p for p in net.pads if p.component_ref == sref] or list(net.pads)
    if not pads:
        return 0.0
    pad = min(pads, key=lambda p: math.hypot(p.global_x - tooth[0], p.global_y - tooth[1]))
    berth = ctx.ends[nm][1]
    air = math.hypot(berth[0] - pad.global_x, berth[1] - pad.global_y)
    lane_ids = {id(s) for s in lane}
    stub = [s for s in ctx.pcb.segments if s.net_id == nid and id(s) not in lane_ids]
    return _mm(lane) + _mm(stub) - air


def re_escape(ctx, nm, lane, vias, board_path, log, cn, rate):
    """Returns the mm saved, 0.0 when nothing was done. `cn` is connect.py,
    `rate` the mm of copper a via is worth."""
    if RE_ESCAPE <= 0 or not lane:
        return 0.0
    chain = ctx.src_chain.get(nm) or []
    if not chain:
        return 0.0
    pcb = ctx.pcb
    nid, net = ctx.byname[nm]
    sref = ctx.src_ref.get(nm)
    tooth = ctx.ends[nm][0]
    pads = [p for p in net.pads if p.component_ref == sref] or list(net.pads)
    if not pads:
        return 0.0
    pad = min(pads, key=lambda p: math.hypot(p.global_x - tooth[0], p.global_y - tooth[1]))
    pad_xy = (pad.global_x, pad.global_y)
    pad_layer = 'F.Cu' if 'F.Cu' in (pad.layers or ()) else ('B.Cu' if 'B.Cu' in (pad.layers or ()) else ctx.tooth_layer[nm])
    berth = ctx.ends[nm][1]
    blayer = ctx.dest_layer[nm]
    air = math.hypot(berth[0] - pad_xy[0], berth[1] - pad_xy[1])
    # THE WHOLE SOURCE STUB: with the lane lifted, the net's static copper
    # falls into the source-side piece (the pad, its dogbone via, the
    # channel escape, the tooth) and the berth-side piece; the source
    # piece is lifted entire -- the tip chain alone left a dogbone via and
    # its pad segment dangling (K36 SDQ0)
    lane_ids = {id(s) for s in lane} | {id(v) for v in vias}
    segs_n = [s for s in pcb.segments if s.net_id == nid and id(s) not in lane_ids]
    vias_n = [v for v in pcb.vias if v.net_id == nid and id(v) not in lane_ids]
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            a = parent[a]
        return a

    def union(a, b):
        parent[find(a)] = find(b)

    def k(x, y):
        return (round(x, 2), round(y, 2))
    for s in segs_n:
        union(('s', id(s)), ('p', k(s.start_x, s.start_y)))
        union(('s', id(s)), ('p', k(s.end_x, s.end_y)))
    for v in vias_n:
        union(('v', id(v)), ('p', k(v.x, v.y)))
        for s in segs_n:
            for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
                if math.hypot(x - v.x, y - v.y) < v.size / 2 + 1e-6:
                    union(('v', id(v)), ('s', id(s)))
    root = None
    for s in segs_n:
        for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if math.hypot(x - pad_xy[0], y - pad_xy[1]) <= max(pad.size_x, pad.size_y) / 2 + 0.05:
                root = find(('s', id(s)))
                break
        if root is not None:
            break
    if root is None:
        return 0.0
    stub = [s for s in segs_n if find(('s', id(s))) == root]
    stub_vias = [v for v in vias_n if find(('v', id(v))) == root]
    m0 = _mm(lane) + _mm(stub)
    v0 = len(vias) + len(stub_vias)
    if m0 - air < RE_ESCAPE:
        return 0.0
    import source_realize as _sr
    try:
        n0 = len(_sr.drc_pairs(board_path, nets=[nm], pcb_data=pcb))
    except Exception as e:      # noqa: BLE001 -- no verdict, no change
        log(f'  re-escape {nm}: no DRC verdict ({e}) -- kept as laid')
        return 0.0
    ids = {id(s) for s in lane} | {id(s) for s in stub}
    vids = {id(v) for v in vias} | {id(v) for v in stub_vias}
    seg0, via0 = list(pcb.segments), list(pcb.vias)
    pcb.segments = [s for s in seg0 if id(s) not in ids]
    pcb.vias = [v for v in via0 if id(v) not in vids]
    res = None
    try:
        for mg in (4.0, 6.0):
            res = cn.connect(pcb, nid, pad_xy, pad_layer, berth, blayer, ctx.cfg,
                             band=None, margin=mg, window_pts=[pad_xy, berth],
                             b_alts=ctx.dest_alts.get(nm))
            if res is not None:
                break
    except Exception as e:      # noqa: BLE001
        log(f'  re-escape {nm}: search failed ({e}) -- kept as laid')
        res = None
    if res is None:
        pcb.segments, pcb.vias = seg0, via0
        log(f'  re-escape {nm}: {m0:.1f} mm for a {air:.1f} mm airline, no route from the pad -- kept as laid')
        return 0.0
    m1, v1 = _mm(res[0]), len(res[1])
    if v1 * rate + m1 >= v0 * rate + m0 - 0.25:
        pcb.segments, pcb.vias = seg0, via0
        log(f'  re-escape {nm}: {v0} via(s) / {m0:.1f} mm -> {v1} / {m1:.1f} mm is not cheaper at {rate:.0f} mm a via -- kept as laid')
        return 0.0
    pcb.segments = pcb.segments + list(res[0])
    pcb.vias = pcb.vias + list(res[1])
    try:
        n1 = len(_sr.drc_pairs(board_path, nets=[nm], pcb_data=pcb))
    except Exception as e:      # noqa: BLE001
        pcb.segments, pcb.vias = seg0, via0
        log(f'  re-escape {nm}: no DRC verdict after ({e}) -- kept as laid')
        return 0.0
    if n1 > n0:
        pcb.segments, pcb.vias = seg0, via0
        log(f'  re-escape {nm}: {v0}/{m0:.1f} -> {v1}/{m1:.1f} would add DRC ({n0} -> {n1}) -- kept as laid')
        return 0.0
    lane[:] = list(res[0])
    vias[:] = list(res[1])
    ctx.ends[nm] = (pad_xy, berth)
    # no stub left to splice onto: the source trim that follows would
    # otherwise splice the new lane onto the lifted chain and cut it off
    # a millimetre from the ball (zynq K47 DQ10/DQ11, open)
    ctx.src_chain[nm] = []
    # the fanout's own vias are in the board FILE the writer starts from
    # (strip_net_segments takes only its segments): the lifted ones are
    # struck from the text at write time (strip_vias_at)
    ctx.re_escape_vias.extend((v.x, v.y, nid, net.name) for v in stub_vias)
    ctx.re_escapes[nm] = (m0 - m1, v0, v1)
    log(f'  re-escape {nm}: from the pad, {v0} via(s) / {m0:.1f} mm (stub {_mm(stub):.1f}, {len(stub_vias)} via) -> {v1} / {m1:.1f} mm, '
        f'airline {air:.1f}')
    return m0 - m1


def strip_vias_at(txt, lifted, tol=0.011):
    """The board text without the `(via ...)` blocks at the lifted
    (x, y, net_id) sites: each block is matched whole, paren-balanced,
    by its (at x y) and (net n)."""
    if not lifted:
        return txt
    import re
    out, i = [], 0
    at_re = re.compile(r'\(at\s+([-\d.]+)\s+([-\d.]+)')
    net_re = re.compile(r'\(net\s+(\d+)\)')                    # the numeric dialect
    name_re = re.compile(r'\(net\s+"((?:[^"\\]|\\.)*)"\)')     # the net-name dialect (the fanout's)
    while True:
        j = txt.find('(via', i)
        if j < 0:
            out.append(txt[i:])
            break
        # the balanced block
        depth, k = 0, j
        while k < len(txt):
            ch = txt[k]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    k += 1
                    break
            k += 1
        blk = txt[j:k]
        m_at, m_net, m_name = at_re.search(blk), net_re.search(blk), name_re.search(blk)
        drop = False
        if m_at and (m_net or m_name):
            x, y = float(m_at.group(1)), float(m_at.group(2))
            n = int(m_net.group(1)) if m_net else None
            nmx = m_name.group(1).replace('\\"', '"') if m_name else None
            drop = any((nn == n if n is not None else nmn == nmx)
                       and abs(x - vx) <= tol and abs(y - vy) <= tol for (vx, vy, nn, nmn) in lifted)
        out.append(txt[i:j])
        if not drop:
            out.append(blk)
        i = k
    return ''.join(out)
