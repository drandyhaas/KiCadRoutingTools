#!/usr/bin/env python3
"""kept_ways.py PLAN.json OUT.kicad_pcb -- THE WAYS OUT OF THE BALLS A PLAN WALLS IN, laid on a copy of its bench
   kept_ways.py --take-up BOARD.kicad_pcb WAYS.json -- those ways taken up again from a board laid on such a bench

A bus's lanes run between two arrays, and an array's ball on no bus net leaves by its own stub, laid with the
fanout. Where a bundle of lanes runs past the end of such a stub the ball is walled in: no track reaches it from
anywhere, and its net ships open past copper no later step moves. The lay finds that only once the lanes are copper
(route_lanes.lift_walls), and what it can still do is lift a lane and route it again -- one lane at a time, each in
its band. A ball whose way out runs between a lane and a fixed pad, a track's room short by a few hundredths of a
millimetre, needs several lanes of the bundle moved over together, and that is the plan's to do, not the lay's.

So the PLAN is asked first (the bench from BENCH / NETS / DEST as every whole_* stage reads it). PLAN's lanes are
read as copper on the bench -- a single's pieces a track wide, a pair's as one track over both its legs -- and the
balls of the bus's parts on no bus net are given the step's own cage test (route_bus.walls_made, at the fab
ladder's finest rung). For each ball that copper walls in and the bench alone does not, its way out is found
BETWEEN the plan's lanes -- route_bus.ball_channel, the cage test's own way from the end of the ball's copper to
the rest of its net or the edge of its view, never across a lane, the singles soft and the pairs walls, since the
pairs are laid first and held -- and written into a copy of the bench, OUT, as a track of the ball's own net: the
rung's track and a cell of the cage test's raster either side wide, which is the room that test asks a way to
have. The loop is then run again on OUT (whole_route.loop's keep_ways): the geometry holds every lane to its side
of the way and moves them off it together, the polish, the snap and their audits hold them there, the solve is
asked again where they find cuts, and the lay routes round it as round any copper. Made room for by the polish
alone, on the plan that walled the ball in, a way between a lane and a fixed pad left the lane beyond it 0.03 mm
short of its bar to the next pad: the corridor was full, and a lane had to leave it. The ways are no copper of the
board's: `--take-up` removes every one of them from the board the lay leaves, or stops.

Writes OUT (its project and the fanout's plan sidecar beside it, as the bench has them) and OUT's `.ways.json`
only where a ball is walled in and has a way; a line a ball either way (`WAY ...`, `NO WAY ...`), and `WAYS n`
last. Exit 0."""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # a research tool (awx), catalogued, shown at no door

import collections
import contextlib
import copy
import io
import json
import math
import os
import shutil
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))


def _stem(board):
    return board[:-len('.kicad_pcb')] if board.endswith('.kicad_pcb') else board


def ways_file(board):
    """the record of the ways laid into the bench `board`, beside it"""
    return _stem(board) + '.ways.json'


def lanes_copper(pcb, plan, nets, track, via_size, via_drill):
    """THE PLAN'S LANES AS COPPER on `pcb`'s nets: ([Segment], [Via], {id of a single's piece or via: its lane}).
    A single's pieces are `track` wide, its changes of layer vias of the run's size. A pair is ONE track along its
    line as wide as its two legs stand (their spread where they hand over, and a track) and its changes one via as
    wide: the pairs are laid first and held, a wall to every way, so their copper is named for no lane. `nets` the
    bench's nets by the whole route's names (a pair's two legs)"""
    from kicad_parser import Segment, Via
    from coherent_nets import by_key
    import pairs as _pairs
    byk = by_key(pcb)
    legs = _pairs.pair_names(list(nets))
    cu = list(pcb.board_info.copper_layers)
    through = [cu[0], cu[-1]]
    segs, vias, soft = [], [], {}
    for lane, L in sorted(plan.get('lanes', {}).items()):
        pair = legs.get(lane)
        name = pair[0] if pair else lane
        if name not in byk:
            continue
        nid, wide = byk[name][0], 0.0
        if pair:
            # (the two legs' spread: where they hand over to the pair's line, else a track and its clearance)
            wide = track + _pairs.pitch(track)
            for e_ in L.get('ends') or ():
                h_ = e_.get('handover') if isinstance(e_, dict) else None
                if h_ and len(h_) == 2:
                    wide = max(wide, track + math.hypot(h_[0][0] - h_[1][0], h_[0][1] - h_[1][1]))
        for x1, y1, x2, y2, layer in L.get('pieces') or ():
            s = Segment(float(x1), float(y1), float(x2), float(y2), wide or track, layer, nid)
            segs.append(s)
            if not pair:
                soft[id(s)] = lane
        for v_ in L.get('vias') or ():
            v = Via(float(v_[0]), float(v_[1]), via_size + (wide - track if pair else 0.0), via_drill, list(through), nid,
                    tenting_attrs={})
            vias.append(v)
            if not pair:
                soft[id(v)] = lane
    return segs, vias, soft


def bus_parts(pcb, bus_ids, dest):
    """the parts a bus runs between: `dest`'s (several, 'A+B'), and the part with the most pads on the bus's nets
    beside them -- its source"""
    dests = [d_ for d_ in dest.split('+') if d_ in pcb.footprints]
    n = collections.Counter(p.component_ref for i in bus_ids for p in pcb.nets[i].pads if p.component_ref not in dests)
    return sorted(set(dests) | ({n.most_common(1)[0][0]} if n else set()))


def with_copper(pcb, segs, vias):
    c = copy.copy(pcb)
    c.segments, c.vias = list(pcb.segments) + list(segs), list(pcb.vias) + list(vias)
    return c


def find(bench, plan, nets, dest):
    """THE WAYS: [{'ball', 'net', 'net_id', 'tracks', 'vias', 'lanes', 'short', 'length', 'end'}] one for each ball of
    the bus's parts on no bus net that `plan`'s lanes wall in on the bench and that has a way out between them, and
    the balls walled in with none [(ball, why)]. `tracks` [{'start', 'end', 'width', 'layer', 'net_id'}] and `vias`
    [{'x', 'y', 'size', 'drill', 'layers', 'net_id'}] as the board's writer takes them"""
    import route_bus as rb
    import rules as _rules
    from coherent_nets import by_key
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(bench)
    R = _rules.active()
    byk = by_key(pcb)
    full = sorted({byk[n][1].name for n in nets if n in byk})
    bus_ids = [byk[n][0] for n in nets if n in byk]
    refs = bus_parts(pcb, bus_ids, dest)
    segs, vias, soft = lanes_copper(pcb, plan, nets, R.track, R.via_size, R.via_drill)
    if not segs:
        return [], []
    board = with_copper(pcb, segs, vias)
    with contextlib.redirect_stdout(io.StringIO()):
        walled = rb.walls_made(pcb, board, refs, full, full, project=bench)
    if not walled:
        return [], []
    fixed = rb._fixed_nets(board, bench, full)
    rung = rb._cage_rung(board)
    wide = rung[0] + 2 * rb.CAGE_STEP
    cu = list(pcb.board_info.copper_layers)
    out, none = [], []
    for k in sorted(walled):
        r, n = k.split('.', 1)
        pad = next(p for p in pcb.footprints[r].pads if p.pad_number == n)
        with contextlib.redirect_stdout(io.StringIO()):
            ch = rb.ball_channel(board, pad, fixed, rung, soft)
        if ch is None:
            none.append((k, 'no way out between the lanes'))
            continue
        tr = [{'start': (round(a[0], 4), round(a[1], 4)), 'end': (round(b[0], 4), round(b[1], 4)), 'width': round(wide, 4),
               'layer': L, 'net_id': pad.net_id} for a, b, L in ch['lines']
              if math.hypot(round(b[0], 4) - round(a[0], 4), round(b[1], 4) - round(a[1], 4)) > 1e-6]
        vs = [{'x': round(x, 4), 'y': round(y, 4), 'size': rung[1], 'drill': min(R.via_drill, rung[1] / 2),
               'layers': [cu[0], cu[-1]], 'net_id': pad.net_id} for x, y in ch['vias']]
        if not tr:
            none.append((k, 'a way out of no length'))
            continue
        out.append({'ball': k, 'net': pad.net_name, 'net_id': pad.net_id, 'tracks': tr, 'vias': vs,
                    'lanes': list(ch['lanes']), 'short': ch['short'], 'end': ch['end'],
                    'length': round(sum(math.hypot(t['end'][0] - t['start'][0], t['end'][1] - t['start'][1])
                                        for t in tr), 3)})
    return out, none


def write(bench, out, ways):
    """the bench with the `ways` laid on it, written to `out`: its project and the fanout's plan sidecar beside it as
    the bench has them, the ways' record beside it (ways_file: these, and those the bench had already)"""
    from copy_board import copy_siblings
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import add_tracks_and_vias_to_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(bench)
        add_tracks_and_vias_to_pcb(bench, out, [dict(t) for w in ways for t in w['tracks']],
                                   [dict(v) for w in ways for v in w['vias']], [],
                                   net_id_to_name={i: n.name for i, n in pcb.nets.items()})
        copy_siblings(bench, out)
    if os.path.isfile(_stem(bench) + '.plan.json'):
        shutil.copyfile(_stem(bench) + '.plan.json', _stem(out) + '.plan.json')
    # (a bench with ways laid on it already -- a plan on it walled another ball in: theirs are taken up with these)
    prior = json.load(open(ways_file(bench)))['ways'] if os.path.isfile(ways_file(bench)) else []
    with open(ways_file(out), 'w', encoding='utf-8') as f:
        json.dump({'bench': os.path.abspath(bench), 'ways': prior + list(ways)}, f, indent=1)


def take_up(board, ways_json):
    """THE WAYS TAKEN UP AGAIN from `board`, a board laid on the bench they were written into: every track and via of
    `ways_json`'s record, or the run stops -- they are no copper of the board's. How many were taken up"""
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import remove_segments_from_content, remove_vias_from_content
    ways = json.load(open(ways_json))['ways']
    tracks = [t for w in ways for t in w['tracks']]
    vias = [v for w in ways for v in w['vias']]
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    names = {i: n.name for i, n in pcb.nets.items()}
    with open(board, encoding='utf-8') as f:
        txt = f.read()
    txt, n_t = remove_segments_from_content(
        txt, [types.SimpleNamespace(start_x=t['start'][0], start_y=t['start'][1], end_x=t['end'][0], end_y=t['end'][1],
                                    layer=t['layer'], net_id=t['net_id'], width=t['width']) for t in tracks],
        net_id_to_name=names, match_width=True)
    n_v = 0
    if vias:
        txt, n_v = remove_vias_from_content(
            txt, [types.SimpleNamespace(x=v['x'], y=v['y'], size=v['size'], drill=v['drill'], layers=v['layers'],
                                        net_id=v['net_id']) for v in vias], net_id_to_name=names)
    if n_t != len(tracks) or n_v != len(vias):
        raise SystemExit(f'kept_ways: {len(tracks) - n_t} of the {len(tracks)} track(s) and {len(vias) - n_v} of the '
                         f'{len(vias)} via(s) of the ways laid for the plan not found again on '
                         f'{os.path.basename(board)} to take up')
    with open(board, 'w', encoding='utf-8') as f:
        f.write(txt)
    return n_t + n_v


def said(w):
    """a way's line for the round's log"""
    return (f"WAY {w['ball']} ({w['net']}): walled in by the plan's lanes -- its way out, {w['length']:g} mm to "
            f"{'the rest of its net' if w['end'] == 'net' else 'the edge of its view'}, kept between the lanes"
            + (f" ({', '.join(w['lanes'])} short of a track's room to it over {w['short']:g} mm)" if w['lanes']
               else ' (none of them short of room to it)'))


def main():
    a = sys.argv[1:]
    if len(a) == 3 and a[0] == '--take-up':
        print(f'kept_ways: {take_up(a[1], a[2])} piece(s) of the ways taken up from {os.path.basename(a[1])}')
        return
    if len(a) != 2 or a[0].startswith('-'):
        print('usage: kept_ways.py PLAN.json OUT.kicad_pcb | --take-up BOARD.kicad_pcb WAYS.json')
        sys.exit(0 if any(x in ('-h', '--help') for x in a) else 2)
    import awx_settings
    plan, out = json.load(open(a[0])), a[1]
    bench, dest = awx_settings.get('BENCH'), awx_settings.get('DEST')
    nets = [n for n in (awx_settings.get('NETS') or '').split(',') if n]
    ways, none = find(bench, plan, nets, dest)
    for k, why in none:
        print(f'NO WAY {k}: walled in by the plan\'s lanes, {why}')
    for w in ways:
        print(said(w))
    if ways:
        write(bench, out, ways)
    print(f'WAYS {len(ways)}')


if __name__ == '__main__':
    main()
