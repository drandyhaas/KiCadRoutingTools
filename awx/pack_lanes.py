#!/usr/bin/env python3
"""pack_lanes.py -- THE PACK STAGE: the lanes a round's lay laid, each pulled taut against its neighbour (pack.py).

usage: pack_lanes.py --board BENCH --laid LAID.kicad_pcb --nets N1,..|@FILE --dest REF
                     [--plan PLAN.json] --write OUT.kicad_pcb

The bus step's option (route_bus.py --pack, the setting BUS_PACK=1): after a round's lay the whole route runs this
stage on the laid board and keeps the board it writes only where the chain's own grade finds it no worse than the lay
(whole_route.chain's pack_lay). By itself it is the packer on one laid board, for a bench.

BENCH is the bench the lanes were laid on, read as the lay reads it (whole_ctx.bench_with: the braid's setup -- the
lanes' ends, the run's nets, a board of the other pair chirality turned over), and LAID the board to pack. The copper
LAID has and BENCH has not is the lay's, net by net (laid_copper). The packer (pack.pack_corridor) takes the SINGLE-net
lanes in their order across the bus -- the whole route's frame's (whole_frame.build's `final`: the berths' order round
the destination), else the plan's own -- and relaxes each as a taut string between its two ends, which never move: a
point on a layer against that layer's copper, a via against the copper of every routing layer (ROUTE_LAYERS); the
string is emitted as octilinear copper and validated piece by piece at the true radii, and a lane whose packed copper
fails keeps the router's.

Left as laid, and named:
- a differential PAIR: its two legs are one lane, laid coupled, and packed one by one they would part;
- a lane with a piece that is not the lanes' track wide (its end stretches necked by the lay): the packer emits every
  piece at the lanes' track;
- a lane with a via, on a board with a copper layer that is no routing layer of the run: the via stands through that
  layer too, and the packer holds it off the routing layers' copper only;
- a lane whose copper is no one chain from its tooth to its berth (the packer's own reading).
NOTHING is packed on a board whose rules file gives a routing layer a clearance of its own above the run's (kicad_dru:
`layer_clearances`): the packer packs at one clearance on every layer.

OUT is LAID with each packed lane's copper in place of the router's (write): the router's pieces taken out of the
board's text, the packed ones added as the lay adds its own (route_lanes.write: the frame turned back, the project,
the via stamps). No lane packed, no OUT. Exits 0 when the stage ran, 1 when it could not write what it packed.
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door
import argparse
import collections
import contextlib
import copy
import io
import json
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

FOREIGN = 'joint-'      # (joint_lanes.foreign: another bus's published lanes, copper in memory and in no board file)


def _seg_key(s):
    a, b = (round(s.start_x, 5), round(s.start_y, 5)), (round(s.end_x, 5), round(s.end_y, 5))
    return (s.net_id, s.layer, min(a, b), max(a, b), round(s.width, 5))


def _via_key(v):
    return (v.net_id, round(v.x, 5), round(v.y, 5), round(v.size, 5))


def laid_copper(ctx, laid):
    """(segments, vias, originals, names, gone): the copper the board LAID has and the bench of `ctx` has not, and
    `ctx.pcb` made the laid board -- the bench's own copper that LAID still has, the other buses' published lanes
    (FOREIGN), and LAID's own on top. The copper comes as the parser's Segment and Via objects in the bench's frame (a
    board the braid's setup turned over is turned the same way: ctx.M is its own inverse, and the layers swap) under
    the bench's net ids. `originals` {id(object): its copy as LAID's file has it -- the board's own frame, LAID's own
    net id} and `names` LAID's {net id: name}, for write() to take a piece out of the board's text. `gone` counts the
    bench's pieces LAID no longer has (a mini re-route put a lane's ends back and laid them again): left out of
    `ctx.pcb`"""
    from kicad_parser import parse_kicad_pcb
    from bga_fanout.flip_frame import other_layer
    with contextlib.redirect_stdout(io.StringIO()):
        lp = parse_kicad_pcb(laid)
    M = getattr(ctx, 'M', None)
    turn = M if M is not None else (lambda x, y: (x, y))
    over = other_layer if M is not None else (lambda L: L)
    ids = {n.name: i for i, n in ctx.pcb.nets.items() if n.name}
    names = {i: n.name for i, n in lp.nets.items()}
    orig = {}
    for s in lp.segments:
        orig[id(s)] = copy.copy(s)
        (s.start_x, s.start_y), (s.end_x, s.end_y) = turn(s.start_x, s.start_y), turn(s.end_x, s.end_y)
        s.layer, s.net_id = over(s.layer), ids.get(names.get(s.net_id), s.net_id)
    for v in lp.vias:
        orig[id(v)] = copy.copy(v)
        v.x, v.y = turn(v.x, v.y)
        v.layers, v.net_id = [over(L) for L in v.layers], ids.get(names.get(v.net_id), v.net_id)
    own = lambda o: not str(getattr(o, 'uuid', '') or '').startswith(FOREIGN)    # noqa: E731
    out, gone = [], 0
    for base, items, key in ((ctx.base_segments, lp.segments, _seg_key), (ctx.base_vias, lp.vias, _via_key)):
        have = collections.Counter(key(o) for o in base if own(o))
        added = []
        for o in items:
            k = key(o)
            if have[k] > 0:
                have[k] -= 1
            else:
                added.append(o)
        # (what `have` still counts the bench has and LAID has not: as many of each key's pieces left out)
        kept = []
        for o in base:
            k = key(o)
            if own(o) and have[k] > 0:
                have[k] -= 1
                gone += 1
            else:
                kept.append(o)
        out.append((kept, added))
    (keep_s, add_s), (keep_v, add_v) = out
    ctx.pcb.segments, ctx.pcb.vias = keep_s + add_s, keep_v + add_v
    return add_s, add_v, orig, names, gone


def write(ctx, laid, out, old_s, old_v, new_s, new_v, names):
    """OUT: the board LAID with the pieces `old_s` and `old_v` (as its file has them: laid_copper's originals; `names`
    its {net id: name}) taken out of its text and the packed `new_s` and `new_v` added -- by the lay's own writer
    (route_lanes.write: the copper turned back into the board's frame, the project stamped at the routed floor, a via
    under solder declared), on the board without the old pieces as its bench. Every old piece is found in the text, or
    nothing is written"""
    from kicad_writer import remove_segments_from_content, remove_vias_from_content
    from copy_board import copy_siblings, SIBLING_EXTS
    import route_lanes as rl
    with open(laid, encoding='utf-8') as f:
        txt = f.read()
    miss = []
    txt, n_s = remove_segments_from_content(txt, old_s, net_id_to_name=names, unmatched_out=miss, match_width=True)
    txt, n_v = remove_vias_from_content(txt, old_v, net_id_to_name=names, unmatched_out=miss)
    if miss or n_s != len(old_s) or n_v != len(old_v):
        raise SystemExit(f'pack_lanes: {len(old_s) - n_s} of {len(old_s)} segment(s) and {len(old_v) - n_v} of '
                         f'{len(old_v)} via(s) of the packed lanes not found in {os.path.basename(laid)} -- no '
                         f'board written')
    tmp = out[:-len('.kicad_pcb')] + '.unpacked.kicad_pcb'
    try:
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write(txt)
        with contextlib.redirect_stdout(io.StringIO()):
            copy_siblings(laid, tmp)
        fresh = {id(o) for o in new_s} | {id(o) for o in new_v}
        rl.write(ctx, tmp, out, [s for s in ctx.pcb.segments if id(s) not in fresh],
                 [v for v in ctx.pcb.vias if id(v) not in fresh])
    finally:
        for ext in ('.kicad_pcb',) + tuple(SIBLING_EXTS):
            if os.path.isfile(tmp[:-len('.kicad_pcb')] + ext):
                os.remove(tmp[:-len('.kicad_pcb')] + ext)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--board', required=True, help='the bench the lanes were laid on')
    ap.add_argument('--laid', required=True, help='the laid board to pack')
    ap.add_argument('--nets', required=True, help='the bench nets: N1,N2,.. or @FILE')
    ap.add_argument('--dest', required=True, help="the destination part's reference (several, 'U4+U3')")
    ap.add_argument('--plan', default='', help='the whole-route plan the lanes were laid from (JSON)')
    ap.add_argument('--write', required=True, help='the packed board (.kicad_pcb)')
    ap.add_argument('--hold', default='', help='lanes held as laid, N1,N2,..: their copper the others pack beside')
    a = ap.parse_args(argv)
    for k in ('board', 'laid', 'plan', 'write'):         # whole_ctx works from awx/: every path absolute first
        if getattr(a, k):
            setattr(a, k, os.path.abspath(getattr(a, k)))
    if a.nets.startswith('@'):
        a.nets = '@' + os.path.abspath(a.nets[1:])
    t0 = time.time()
    import whole_ctx
    import braid as bd
    import pack
    from plan_audit import read_nets
    geo = json.load(open(a.plan)) if a.plan else {}
    ctx, _groups = whole_ctx.bench_with(a.board, read_nets(a.nets), a.dest)
    layers = list(ctx.cfg.layers)
    add_s, add_v, orig, names, gone = laid_copper(ctx, a.laid)
    print(f'{os.path.basename(a.laid)} on {os.path.basename(a.board)}: {len(add_s)} segment(s) and {len(add_v)} '
          f'via(s) laid' + (f', {gone} piece(s) of the bench not on it' if gone else '')
          + f'; the routing layers {", ".join(layers)}')
    by_s, by_v = collections.defaultdict(list), collections.defaultdict(list)
    for s in add_s:
        by_s[s.net_id].append(s)
    for v in add_v:
        by_v[v.net_id].append(v)
    nid = lambda n: ctx.byname[n][0]    # noqa: E731
    # the lanes: every single net with copper laid. A PAIR is one lane of two nets and stays as its router laid it
    prs = getattr(ctx, 'pairs', {}) or {}
    singles = [n for n in ctx.tooth_layer if n not in prs and by_s.get(nid(n))]
    n_pairs = sum(1 for legs in prs.values() if any(by_s.get(nid(l_)) for l_ in legs))
    if n_pairs:
        print(f'{n_pairs} pair(s) left as laid: a pair\'s legs are packed together or not at all')
    # ...so does a lane with a piece off the lanes' track: the packer emits at the lanes' track and nothing else
    necked = [n for n in singles if any(abs(s.width - bd.TRACK) > 1e-6 for s in by_s[nid(n)])]
    if necked:
        print(f'{len(necked)} lane(s) left as laid, a piece off the lanes\' track ({bd.TRACK:g} mm): '
              + ', '.join(necked))
    # ...and a lane with a via where the board has a copper layer that is no routing layer: a via is a through via,
    # and the packer's model of what it must clear is the routing layers' copper
    blind = [L for L in (ctx.pcb.board_info.copper_layers or []) if L not in layers]
    viad = [n for n in singles if blind and n not in necked and by_v.get(nid(n))]
    if viad:
        print(f'{len(viad)} lane(s) with a via left as laid: a via stands through {", ".join(blind)} too, no routing '
              f'layer of the run, and the packer holds it off the routing layers\' copper only: ' + ', '.join(viad))
    # ...and a lane with a piece on a layer that is none of the run's (a board laid under other routing layers)
    off = [n for n in singles if any(s.layer not in layers for s in by_s[nid(n)])]
    if off:
        print(f'{len(off)} lane(s) left as laid, a piece on no routing layer of the run: ' + ', '.join(off))
    # ...and the lanes the caller holds (whole_route.pack_lay: a packed lane by a pad the pack walled in)
    hold = [n for n in singles if n in set(x for x in a.hold.split(',') if x)]
    if hold:
        print(f'{len(hold)} lane(s) held as laid: ' + ', '.join(hold))
    members = [n for n in singles if n not in necked and n not in viad and n not in off and n not in hold]
    # a layer with a clearance of its own above the run's (the board's rules file): the packer packs at one
    # clearance, every layer -- nothing is packed there
    own_c = {L: c_ for L, c_ in (getattr(ctx.cfg, 'layer_clearances', None) or {}).items()
             if L in layers and c_ > bd.CLEAR + 1e-9}
    why = ''
    if own_c:
        why = ('the board\'s rules give '
               + ', '.join(f'{L} a clearance of {c_:g} mm' for L, c_ in sorted(own_c.items()))
               + f', above the {bd.CLEAR:g} mm the packer packs at on every layer')
        print('nothing packed: ' + why)
        members = []
    # their order ACROSS the bus: the frame's (the berths' order round the destination), else the plan's own
    try:
        import whole_frame
        with contextlib.redirect_stdout(io.StringIO()):
            order, how = list(whole_frame.build(ctx, a.dest).final), "the frame's"
    except (Exception, SystemExit) as e:
        order, how = list(geo.get('lanes') or ()), "the plan's"
        print(f'no frame of the bench ({type(e).__name__}: {str(e)[:120]}) -- the lanes in the plan\'s order')
    target = [n for n in order if n in members]
    if len(target) < len(members):
        print(f'{len(members) - len(target)} lane(s) with no place in {how} order, left as laid: '
              + ', '.join(n for n in members if n not in target))
    c = types.SimpleNamespace(
        ctx=ctx, members=target, target=target,
        out_segs={n: by_s[nid(n)] for n in target},
        out_vias={n: by_v.get(nid(n), []) for n in target},
        # (the plan's lines: which side of the lane before it a lane lies on, where their copper does not say)
        lane_xy={n: [tuple(map(float, p)) for p in v['xy']] for n, v in (geo.get('lanes') or {}).items()
                 if v.get('xy')})
    was_s, was_v = {id(s) for s in ctx.pcb.segments}, {id(v) for v in ctx.pcb.vias}
    was = {n: {id(s) for s in c.out_segs[n]} for n in target}
    print(f'pack: {len(target)} single lane(s) in {how} order, on {len(layers)} routing layer(s)')
    if target:
        if pack.JUMP:
            # (the jump's pack: again until no lane moves -- pack.pack_until_still)
            pack.pack_until_still(c, print)
        else:
            pack.pack_corridor(c, print)
    now_s, now_v = {id(s) for s in ctx.pcb.segments}, {id(v) for v in ctx.pcb.vias}
    old_s = [orig[id(s)] for s in add_s if id(s) not in now_s]
    old_v = [orig[id(v)] for v in add_v if id(v) not in now_v]
    new_s = [s for s in ctx.pcb.segments if id(s) not in was_s]
    new_v = [v for v in ctx.pcb.vias if id(v) not in was_v]
    packed = [n for n in target if {id(s) for s in c.out_segs[n]} != was[n]]
    kept = [n for n in target if n not in packed]
    if packed:
        write(ctx, a.laid, a.write, old_s, old_v, new_s, new_v, names)
    left = [f'{n_pairs} pair(s)'] * bool(n_pairs) + [f'{len(hold)} held lane(s)'] * bool(hold) \
        + [f'{len(necked)} necked lane(s)'] * bool(necked) \
        + [f'{len(viad)} lane(s) with a via'] * bool(viad) + [f'{len(off)} lane(s) off the routing layers'] * bool(off)
    print(f'SUMMARY pack: {len(packed)} of {len(target)} single lane(s) packed' + (f' ({why})' if why else '')
          + (f', {len(kept)} keep the router\'s copper ({", ".join(kept)})' if kept else '')
          + (f', {" and ".join(left)} left as laid' if left else '')
          + f'; {len(old_s)} -> {len(new_s)} segment(s), {len(new_v)} via(s) laid again; {time.time() - t0:.1f} s')


if __name__ == '__main__':
    main()
