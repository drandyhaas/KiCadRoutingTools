"""joint_lanes.py -- the lanes of buses routed AT ONCE off one shared array (route_bus JOINT_DIR, each bus its own whole
route in a process of its own), each bus's in sight of the others at every stage.

publish(...): a bus's lanes as its loop has them now -- a plan's (whole_geo, whole_polish, whole_snap: each lane's
pieces on their layers, its vias) or a laid board's (its own nets' tracks and vias) -- turned back into the BOARD's
frame and written to JOINT_DIR/lanes/BUS.json, whole (a temporary file replaced), the stage named.

foreign(pcb): every OTHER bus's lanes as published, turned into this bus's flow frame, put on a board this bus's stages
read (braid.setup's parse, whole_route's tracks of a bench) as copper of their own nets: tracks a lane's width wide (a
pair's lane its two legs' whole width), vias a via's (a pair's dive its two). The stages then plan round them as round
any copper that is not theirs -- the geometry, the polish and its audit, the snap and the lay -- and the stage cache,
which keys a stage by every file it opened, plans again when a peer publishes. The board's file is never changed."""
import json
import math
import os

import awx_settings


def _turn(p, q, cx, cy):
    import flow_frame as ff
    d = ff._q(p[0] - cx, p[1] - cy, q)
    return (cx + d[0], cy + d[1])


def _me():
    jd, bus, tr = awx_settings.get('JOINT_DIR'), awx_settings.get('JOINT_BUS'), awx_settings.get('JOINT_TURN')
    if not (jd and bus and tr):
        return None
    q, cx, cy = tr.split(',')
    return jd, bus, (int(float(q)), float(cx), float(cy))


def _write(jd, bus, doc):
    d = os.path.join(jd, 'lanes')
    os.makedirs(d, exist_ok=True)
    tmp = os.path.join(d, f'.{bus}.json.tmp')
    with open(tmp, 'w') as f:
        json.dump(doc, f)
    os.replace(tmp, os.path.join(d, f'{bus}.json'))


def publish(plan_path, stage):
    """the lanes of the plan JSON at `plan_path` (its 'lanes': {lane: {'pieces': [[x0, y0, x1, y1, layer], ..]}}, its
    'vias': [[lane, x, y], ..], in this bus's flow frame) published in the board's frame"""
    me = _me()
    if me is None or not os.path.isfile(plan_path):
        return
    jd, bus, (q, cx, cy) = me
    J = json.load(open(plan_path))
    # (a bus whose pairs read chirality -1 plans on its board TURNED OVER -- braid.setup: y mirrored, F.Cu and B.Cu
    # swapped -- and its plans are in that frame: mirrored back, and their layers, before the turn. The bench's own
    # reading, which every stage of the loop has planned already: whole_ctx's cache)
    import whole_ctx
    ctx, _cs = whole_ctx.plan()
    mir, lay = (lambda x, y: (x, y)), (lambda L: L)
    if getattr(ctx, 'chi', 1) < 0 and getattr(ctx, 'M', None) is not None:
        from bga_fanout.flip_frame import other_layer
        mir, lay = ctx.M, other_layer
    pcs, vs = [], []
    for n, v in (J.get('lanes') or {}).items():
        for x0, y0, x1, y1, L in v.get('pieces') or []:
            a, b = _turn(mir(x0, y0), -q, cx, cy), _turn(mir(x1, y1), -q, cx, cy)
            pcs.append([round(a[0], 4), round(a[1], 4), round(b[0], 4), round(b[1], 4), lay(L), n])
    for n, x, y in J.get('vias') or []:
        p = _turn(mir(x, y), -q, cx, cy)
        vs.append([round(p[0], 4), round(p[1], 4), n])
    # ...and its ENDS as its bench has them: the stubs and vias of its own nets there (the destination's berths, which
    # this bus's fanout lays each round and no other bus's bench has), at their own widths -- the plan's lanes run to
    # the berths' ends, and another bus ran its lane across a berth's stub, the berth crowded every round. Kept apart
    # ('bench'): copper for the other buses' stages (foreign), never a lane the joint solve couples (whole_joint)
    kids = getattr(ctx, 'kids', set()) or set()
    pcs_b, vs_b = _bench_copper(awx_settings.get('BENCH'),
                                {nm for nm, v in (getattr(ctx, 'byname', None) or {}).items() if v[0] in kids},
                                q, cx, cy)
    _write(jd, bus, {'bus': bus, 'stage': stage, 'from': os.path.abspath(plan_path), 'pieces': pcs, 'vias': vs,
                     'bench': {'pieces': pcs_b, 'vias': vs_b}})


def _bench_copper(bench, names, q, cx, cy):
    """([pieces], [vias]) of the nets `names` (the whole route's) on `bench` (this bus's flow frame, as the file
    has it -- never turned over), in the board's frame, each at its own width and size"""
    if not bench or not os.path.isfile(bench) or not names:
        return [], []
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    from coherent_nets import namer
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(bench)
    sn = namer(pcb)
    name = {i: sn(n.name) for i, n in pcb.nets.items() if n.name}
    pcs, vs = [], []
    for s in pcb.segments:
        if name.get(s.net_id) in names and not getattr(s, 'graphic', False):
            a, b = _turn((s.start_x, s.start_y), -q, cx, cy), _turn((s.end_x, s.end_y), -q, cx, cy)
            pcs.append([round(a[0], 4), round(a[1], 4), round(b[0], 4), round(b[1], 4), s.layer, name[s.net_id],
                        s.width])
    for v in pcb.vias:
        if name.get(v.net_id) in names:
            p = _turn((v.x, v.y), -q, cx, cy)
            vs.append([round(p[0], 4), round(p[1], 4), name[v.net_id], v.size])
    return pcs, vs


def publish_board(board, nets, stage):
    """the copper of the run's nets `nets` (the whole route's names) on the laid `board`, in this bus's flow frame,
    published in the board's frame -- tracks at their own width, vias at their own size"""
    me = _me()
    if me is None or not os.path.isfile(board):
        return
    jd, bus, (q, cx, cy) = me
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    from coherent_nets import namer
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    sn = namer(pcb)
    want = set(nets)
    name = {i: sn(n.name) for i, n in pcb.nets.items() if n.name}
    pcs, vs = [], []
    for s in pcb.segments:
        if name.get(s.net_id) in want and not getattr(s, 'graphic', False):
            a, b = _turn((s.start_x, s.start_y), -q, cx, cy), _turn((s.end_x, s.end_y), -q, cx, cy)
            pcs.append([round(a[0], 4), round(a[1], 4), round(b[0], 4), round(b[1], 4), s.layer, name[s.net_id],
                        s.width])
    for v in pcb.vias:
        if name.get(v.net_id) in want:
            p = _turn((v.x, v.y), -q, cx, cy)
            vs.append([round(p[0], 4), round(p[1], 4), name[v.net_id], v.size])
    _write(jd, bus, {'bus': bus, 'stage': stage, 'from': os.path.abspath(board), 'pieces': pcs, 'vias': vs,
                     'laid': True})


def foreign(pcb):
    """every other bus's published lanes put on `pcb` (this bus's flow frame) as copper of their own nets; the count.
    Only under JOINT_FOREIGN: 'all' every other bus's, 'finished' only a bus's whose whole route has ended (its laid
    copper final: a round's first solve, which the joint solve builds again, and the joint solve itself -- the buses
    still routing are coupled there, not copper)"""
    me = _me()
    mode = awx_settings.get('JOINT_FOREIGN')
    if me is None or mode not in ('all', 'finished'):
        return 0
    jd, bus, (q, cx, cy) = me
    d = os.path.join(jd, 'lanes')
    if not os.path.isdir(d):
        return 0
    # (JOINT_FINISHED: the buses ended when a round's first solve read them -- the joint solve builds that solve's model
    # again, and a bus ending between the two made it another model)
    fin = awx_settings.get('JOINT_FINISHED')
    fin = None if fin is None else {b_ for b_ in fin.split(',') if b_}
    from kicad_parser import Segment, Via
    from coherent_nets import namer
    import pairs as _pairs
    import rules as _rules
    R = _rules.active()
    sn = namer(pcb)
    by = {}
    for i, n in pcb.nets.items():
        if n.name:
            by.setdefault(sn(n.name), i)
    base = {b_: legs[0] for b_, legs in _pairs.pair_names(sorted(by), admit_all=True).items()}
    w_pair = _pairs.pitch(R.track) + R.track
    cu = [L for L in (pcb.board_info.copper_layers or [])]
    n_ = 0
    for f in sorted(os.listdir(d)):
        if not f.endswith('.json') or f.startswith('.') or f == f'{bus}.json':
            continue
        if mode == 'finished' and (f[:-len('.json')] not in fin if fin is not None else
                                   not os.path.isfile(os.path.join(jd, f[:-len('.json')] + '.finished'))):
            continue
        try:
            doc = json.load(open(os.path.join(d, f)))
        except (OSError, ValueError):
            continue
        bench = doc.get('bench') or {}
        for row in (doc.get('pieces') or []) + (bench.get('pieces') or []):
            x0, y0, x1, y1, L, ln = row[:6]
            nid = by.get(ln) if ln in by else by.get(base.get(ln))
            if nid is None:
                continue
            w = row[6] if len(row) > 6 else (w_pair if ln in base and ln not in by else R.track)
            a, b = _turn((x0, y0), q, cx, cy), _turn((x1, y1), q, cx, cy)
            if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-6:
                continue
            pcb.segments.append(Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=w, layer=L,
                                        net_id=nid, uuid=f'joint-{doc.get("bus")}-{n_}'))
            n_ += 1
        for row in (doc.get('vias') or []) + (bench.get('vias') or []):
            x, y, ln = row[:3]
            nid = by.get(ln) if ln in by else by.get(base.get(ln))
            if nid is None:
                continue
            sz = row[3] if len(row) > 3 else (R.via_size + (_pairs.pitch(R.track) if ln in base and ln not in by
                                                            else 0.0))
            p = _turn((x, y), q, cx, cy)
            pcb.vias.append(Via(x=p[0], y=p[1], size=sz, drill=R.via_drill, layers=['F.Cu', 'B.Cu'],
                                net_id=nid, uuid=f'joint-{doc.get("bus")}-v{n_}'))
            n_ += 1
    return n_
