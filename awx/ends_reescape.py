"""ends_reescape.py OUT LANE [--avoid=LAYER] -- a CROWDED END's ball escaped again on the round's fanout bench (BENCH0,
else BENCH): the loop stopped on LANE found too close, right past its own tooth, to copper nothing may move -- another
net's track or pad on the tooth's layer, a locked via on every layer -- and the lane's ball takes another escape from
its source menu (fanout_from_plan.plan_state): one whose first stretch past its exit stands clear of every other net's
copper on its layer by a lane's clearance and a margin, on the same face and within a millimetre of the old tooth (the
solve's order was solved for that tooth: one moved further, or onto another face, crowded its neighbours there in
turn), off LAYER when given, nearest the old tooth, on its layer where it can, fewest vias. Laid by
source_realize.realize alone -- the whole-array plan of that one ball; every other ball, the bus's other teeth and every
earlier step's copper stay as they are, and the lane's copper not joined to its ball (its berth's stub) is put back.
Writes OUT and its plan sidecar (the bench's, the tooth re-measured). Exit 0 laid; 2 no escape clears; 3 none laid."""
import json
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', 'py_router')))
import awx_settings  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
import fanout_from_plan as ffp  # noqa: E402
import source_realize as sr  # noqa: E402

MARGIN = 0.05                # past the lane's clearance, as ends_nudge
REACH = 0.6                  # the stretch past a tooth's exit that must stand clear (mm)
NEAR = 1.0                   # the farthest a tooth moves along its face (mm)
TRIES = 4                    # the escapes realize is asked to lay, best first

args = [a for a in sys.argv[1:] if not a.startswith('--')]
AVOID = next((a.split('=', 1)[1] for a in sys.argv[1:] if a.startswith('--avoid=')), None)
out, lane = args[0], args[1]
bench = awx_settings.get('BENCH0') or awx_settings.req('BENCH')
names = [n for n in awx_settings.req('NETS').split(',') if n]
pcb = parse_kicad_pcb(bench)
st = ffp.plan_state(pcb, names)
byname, src_pad, sref = st['byname'], st['src_pad'], st['sref']
bar = te.TRACK / 2 + te.CLEAR + MARGIN


def seg_d(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    L2 = (bx - ax) ** 2 + (by - ay) ** 2
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / L2))
    return math.hypot(ax + t * (bx - ax) - p[0], ay + t * (by - ay) - p[1])


def foreign_d(q, layer, nid):
    """clearance from point q on `layer` to the nearest copper of another net (tracks, vias, pads)"""
    best = 9e9
    for s in pcb.segments:
        if s.net_id == nid or s.layer != layer:
            continue
        best = min(best, seg_d(q, (s.start_x, s.start_y), (s.end_x, s.end_y)) - s.width / 2)
    for v in pcb.vias:
        if v.net_id != nid:
            best = min(best, math.hypot(v.x - q[0], v.y - q[1]) - v.size / 2)
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if p.net_id != nid and (layer in p.layers or '*.Cu' in p.layers or (p.drill or 0) > 0):
                best = min(best, math.hypot(p.global_x - q[0], p.global_y - q[1]) - max(p.size_x, p.size_y) / 2)
    return best


def clears(m, nid):
    """(ok, its worst clearance): the escape's first REACH past its exit clear of every other net's copper on its
    layer by the bar"""
    dx, dy = em.DIRS[m.direction]
    worst = min(foreign_d((m.exit_pt[0] + dx * k * REACH / 4, m.exit_pt[1] + dy * k * REACH / 4), m.layer, nid)
                for k in range(5))
    return worst >= bar, worst


old = sr.measure_tooth(pcb, lane, src_pad[lane], byname)
if old is None:
    print(f'ends_reescape: {lane} has no tooth on the bench')
    sys.exit(2)
nid = byname[lane][0]
menu = st['smenu'].get(lane) or []
print(f'ends_reescape: {lane} (tooth {old["tooth"]} {old["layer"]} {old["direction"]}), {len(menu)} escapes, '
      f'bar {bar:.3f}' + (f', off {AVOID}' if AVOID else ''))
cands = []
for m in menu:
    if AVOID and m.layer == AVOID:
        continue
    far = math.hypot(m.exit_pt[0] - old['tooth'][0], m.exit_pt[1] - old['tooth'][1])
    if m.layer == old['layer'] and m.direction == old['direction'] and far < 0.1:
        continue                      # (the escape it has)
    if m.direction != old['direction'] or far > NEAR:
        continue                      # (another face, or far along this one)
    ok, d = clears(m, nid)
    if ok:
        cands.append(((m.layer != old['layer'], round(far, 2), m.vias, -d), m, d))
cands.sort(key=lambda c: c[0])
for k, m, d in cands[:TRIES + 1]:
    print(f'  candidate {m!r} clear {d:.3f} key {k}')
if not cands:
    print('ends_reescape: NO escape clears the tooth')
    sys.exit(2)
# (realize strips the whole net and lays its source escape alone: the lane's other copper -- its berth's stub at the
# destination, every piece not joined to its source ball -- is put back after)
_items = [('s', s_) for s_ in pcb.segments if s_.net_id == nid] + [('v', v_) for v_ in pcb.vias if v_.net_id == nid]
_par = list(range(len(_items) + 1))           # (the last: the source ball)


def _f(i):
    while _par[i] != i:
        _par[i] = _par[_par[i]]
        i = _par[i]
    return i


def _ends(it):
    k_, o_ = it
    if k_ == 's':
        return [((o_.start_x, o_.start_y), o_.layer, o_.width / 2), ((o_.end_x, o_.end_y), o_.layer, o_.width / 2)]
    return [((o_.x, o_.y), '*', o_.size / 2)]


_sp = src_pad[lane]
_ball = [((_sp.global_x, _sp.global_y), '*' if (_sp.drill or 0) > 0 else _sp.layers[0], max(_sp.size_x, _sp.size_y) / 2)]
_all = [_ends(it) for it in _items] + [_ball]
for i in range(len(_all)):
    for j in range(i + 1, len(_all)):
        if any((la == '*' or lb == '*' or la == lb) and math.hypot(a[0] - b[0], a[1] - b[1]) <= max(ra, rb) + 1e-6
               for a, la, ra in _all[i] for b, lb, rb in _all[j]):
            _par[_f(i)] = _f(j)
_root = _f(len(_items))
keep_s = [o_ for i, (k_, o_) in enumerate(_items) if k_ == 's' and _f(i) != _root]
keep_v = [o_ for i, (k_, o_) in enumerate(_items) if k_ == 'v' and _f(i) != _root]
print(f'  the lane\'s other copper put back after: {len(keep_s)} track(s), {len(keep_v)} via(s)')
for _k, m, d in cands[:TRIES]:
    res = sr.realize(bench, {lane: m}, src_pad, byname, sref, out, log=lambda s: print('  ' + s), guard_names=())
    if lane in (res.get('ok') or []) and not res.get('rejected'):
        break
    print(f'  realize refused {m!r}: {res.get("rejected") or res.get("restored")}')
else:
    print('ends_reescape: no escape laid')
    sys.exit(3)
if keep_s or keep_v:
    from kicad_writer import add_tracks_and_vias_to_pcb
    _n2n = {i: n.name for i, n in pcb.nets.items()}
    add_tracks_and_vias_to_pcb(
        out, out,
        [{'start': (s_.start_x, s_.start_y), 'end': (s_.end_x, s_.end_y), 'width': s_.width, 'layer': s_.layer,
          'net_id': nid} for s_ in keep_s],
        [{'x': v_.x, 'y': v_.y, 'size': v_.size, 'drill': v_.drill, 'layers': list(v_.layers), 'net_id': nid,
          'tenting_attrs': getattr(v_, 'tenting_attrs', {}), 'inherit_when_unspecified': True} for v_ in keep_v],
        net_id_to_name=_n2n)
new = sr.measure_tooth(parse_kicad_pcb(out), lane, src_pad[lane], byname)
side = json.load(open(os.path.splitext(bench)[0] + '.plan.json'))
side['ends'][lane][0] = list(new['tooth'])
side['tooth_layer'][lane] = new['layer']
side['tooth_dir'][lane] = list(em.DIRS[new['direction']])
json.dump(side, open(os.path.splitext(out)[0] + '.plan.json', 'w'), indent=1)
for ext in ('.kicad_pro', '.kicad_dru'):
    s_ = os.path.splitext(bench)[0] + ext
    if os.path.isfile(s_) and not os.path.isfile(os.path.splitext(out)[0] + ext):
        shutil.copy(s_, os.path.splitext(out)[0] + ext)
print(f'ends_reescape: {lane} re-escaped: tooth {old["tooth"]} {old["layer"]} {old["direction"]} -> {new["tooth"]} '
      f'{new["layer"]} {new["direction"]} (clear {d:.3f})')
