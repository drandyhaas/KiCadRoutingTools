"""THE FIELD'S TIES -- the power pads beneath a group's parts joined to their nets while the field is still open.

The chain makes a plane's ties last (#562): the pour step lays bare zones, and the route step after the buses drops a
via at each pad. Beneath a bus's parts that is too late -- the lanes have taken the layers under the pad, and no later
step may move a bus net: a decap at an array's edge whose pad stands in the channel a lane took on an inner layer, a
pull-up's supply pad beneath a destination part with lanes under it on two layers. On the fanned-out field the same
pads tie at once, most by a stub on their own layer to a via the fanout laid for a ball of their net. So they are tied
THERE, before a lane is planned, and the lanes go round them:

  * the pass is the chain's own (route.py on the poured nets), on a copy of the field in which every other net's
    copper is recorded protected, so it rips nothing, and each lane's WAY OUT of its exit is laid as copper of the
    lane's net (as group_ends.write_field lays them for the nudge), so no tie stands in one;
  * a tie is a stub on an outer layer and a via: the pass has the two outer layers alone, the inner ones the lanes';
  * of what it lays, only the pieces that reach a pad beneath one of the group's parts are kept, and of those only
    the ones that stay within their part's fan (its pads' box, out to its own exits). Every other pad ties as well
    after the buses, with more room to choose from; tied early, it would only stand in a lane's way.
"""
import collections
import contextlib
import io
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PYR = os.path.join(HERE, '..', 'py_router')


def _load(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


def beneath(pcb, parts, nets):
    """the pads to tie: [(pad)] of the parts that are none of `parts`, surface pads on one of `nets` (ids), standing
    within the box of the pads of one of `parts`"""
    boxes = []
    for r in parts:
        f = pcb.footprints.get(r)
        if f is None or not f.pads:
            continue
        xs, ys = [p.global_x for p in f.pads], [p.global_y for p in f.pads]
        boxes.append((min(xs), min(ys), max(xs), max(ys)))
    got = []
    for ref, f in pcb.footprints.items():
        if ref in parts:
            continue
        for p in f.pads:
            if p.net_id in nets and not p.drill and p.pad_type != 'np_thru_hole' and \
                    any(x0 <= p.global_x <= x1 and y0 <= p.global_y <= y1 for x0, y0, x1, y1 in boxes):
                got.append(p)
    return got


def _on(p, s, tol):
    ax, ay, dx, dy = s.start_x, s.start_y, s.end_x - s.start_x, s.end_y - s.start_y
    l2 = dx * dx + dy * dy
    t = 0 if l2 == 0 else max(0, min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy) <= tol


def pieces(segs, vias):
    """the new copper in PIECES: [(kind 's' | 'v', item, its piece)] -- items of one net that meet (an end on a run of
    its layer, a via on a run or on another via) are one piece"""
    items = [('s', s) for s in segs] + [('v', v) for v in vias]
    par = list(range(len(items)))

    def find(i):
        while par[i] != i:
            par[i] = par[par[i]]
            i = par[i]
        return i
    by_net = collections.defaultdict(list)
    for i, (_k, o) in enumerate(items):
        by_net[o.net_id].append(i)
    for idx in by_net.values():
        for n, i in enumerate(idx):
            ki, oi = items[i]
            for j in idx[n + 1:]:
                kj, oj = items[j]
                if ki == 's' and kj == 's':
                    tol = max(oi.width, oj.width) / 2
                    hit = oi.layer == oj.layer and (
                        any(_on(e, oj, tol) for e in ((oi.start_x, oi.start_y), (oi.end_x, oi.end_y)))
                        or any(_on(e, oi, tol) for e in ((oj.start_x, oj.start_y), (oj.end_x, oj.end_y))))
                elif ki == 's':
                    hit = _on((oj.x, oj.y), oi, oj.size / 2)
                elif kj == 's':
                    hit = _on((oi.x, oi.y), oj, oi.size / 2)
                else:
                    hit = math.hypot(oi.x - oj.x, oi.y - oj.y) < (oi.size + oj.size) / 2
                if hit:
                    par[find(i)] = find(j)
    return [(k, o, find(i)) for i, (k, o) in enumerate(items)]


def _at(k, o, p):
    """a new item ON pad `p`: a run of one of its layers ending within it, or a via standing in it"""
    def inside(q):
        return abs(q[0] - p.global_x) <= p.size_x / 2 + 0.01 and abs(q[1] - p.global_y) <= p.size_y / 2 + 0.01
    if k == 's':
        return (o.layer in p.layers or '*.Cu' in p.layers) and \
            (inside((o.start_x, o.start_y)) or inside((o.end_x, o.end_y)))
    return inside((o.x, o.y))


def fans(pcb, parts, wayouts):
    """each part's FAN: [(x0, y0, x1, y1)], the box of its pads' copper grown as far as its own exits stand out of
    it (a way out starts at an exit; each is its nearest part's)"""
    boxes = {}
    for r in parts:
        f = pcb.footprints.get(r)
        if f is not None and f.pads:
            # (the pads' copper, not their centres: a tie ending on a via 0.02 mm past a row part's last row of pad
            # centres, under the pads themselves, was read as having left the part)
            boxes[r] = (min(p.global_x - p.size_x / 2 for p in f.pads), min(p.global_y - p.size_y / 2 for p in f.pads),
                        max(p.global_x + p.size_x / 2 for p in f.pads), max(p.global_y + p.size_y / 2 for p in f.pads))
    gap = lambda b, x, y: math.hypot(max(b[0] - x, 0, x - b[2]), max(b[1] - y, 0, y - b[3]))     # noqa: E731
    deep = dict.fromkeys(boxes, 0.0)
    for w in wayouts:
        if boxes:
            r = min(boxes, key=lambda r_: gap(boxes[r_], *w['start']))
            deep[r] = max(deep[r], gap(boxes[r], *w['start']))
    return [(b[0] - deep[r], b[1] - deep[r], b[2] + deep[r], b[3] + deep[r]) for r, b in boxes.items()]


def _pass(field, nets, pads, wayouts, within=None, outer=False):
    """the pass itself: route.py on the nets `nets` (names) of a copy of `field` -- the ways out laid on it, every
    other net with copper protected -- and of the copper it adds, the pieces
    that reach one of `pads` laid on `field` in place; with `within` [(x0, y0, x1, y1)] only those whose every end
    and via stands in one of the boxes; with `outer` the pass has the two outer layers alone and only pieces with
    no run on an inner one are kept. Returns {'tied': [(ref, pad)], 'segments', 'vias', 'tracks', 'vs' (the copper
    kept), 'strayed': the pieces left out for leaving the boxes or the outer layers, 'exit': the pass's}"""
    import rules as _rules
    import awx_settings
    import ship_vias
    from kicad_writer import add_tracks_and_vias_to_pcb
    from protected_nets import persist_protected_nets, pro_path_for_board
    a = _load(field)
    R = _rules.active()
    names = {i: n.name for i, n in a.nets.items()}
    want = set(nets)
    cu_l = list(a.board_info.copper_layers)
    layers = [cu_l[0], cu_l[-1]] if outer and len(cu_l) > 2 else cu_l
    tmp = tempfile.mkdtemp(prefix='field_ties.', dir=os.path.dirname(os.path.abspath(field)))
    try:
        src, out = os.path.join(tmp, 'in.kicad_pcb'), os.path.join(tmp, 'tied.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            add_tracks_and_vias_to_pcb(field, src, list(wayouts), [], [], net_id_to_name=names)
        pro = field[:-len('.kicad_pcb')] + '.kicad_pro'
        if os.path.exists(pro):
            shutil.copy(pro, pro_path_for_board(src))
        # (every other net with copper on the field protected for the pass: its rip ladder and its reconciliation lap
        # took a fanned ball's stub to make room for a tie, and the bus's own step then refused the field -- "no
        # copper of its net at the end the group's model laid")
        cu = {s.net_id for s in a.segments if not getattr(s, 'graphic', False)} | {v.net_id for v in a.vias}
        persist_protected_nets(pro_path_for_board(src), {names[i]: 'bus' for i in cu if names.get(i) and
                                                         names[i] not in want}, verbose=False)
        # (sizes held to the board's own minimums: left to narrow to the fab's floor, a tie came out thinner than the
        # board allows and the field's measure threw every tie out)
        argv = [sys.executable, '-X', 'utf8', os.path.join(PYR, 'route.py'), src, '--output', out,
                '--nets', *sorted(want), '--no-bga-zone', '--no-stub-layer-swap', '--escalation', 'board',
                '--grid-step', str(2 * R.grid), '--track-width', str(R.fan_track),
                '--clearance-ceiling', str(R.clearance), '--via-size', str(R.via_size),
                '--via-drill', str(R.via_drill), '--layers', *layers]
        r = subprocess.run(argv, env=awx_settings.environ(), capture_output=True, text=True)
        with open(field[:-len('.kicad_pcb')] + '.ties.log', 'w', encoding='utf-8') as f:
            f.write(' '.join(argv) + '\n' + r.stdout + r.stderr)
        if not os.path.exists(out):
            return {'tied': [], 'segments': 0, 'vias': 0, 'exit': r.returncode, 'strayed': 0}
        s0, b = _load(src), _load(out)
        nm = lambda pcb, i: pcb.nets[i].name if i in pcb.nets else ''        # noqa: E731
        key = lambda pcb, s: (nm(pcb, s.net_id), s.layer, round(s.start_x, 3), round(s.start_y, 3),   # noqa: E731
                              round(s.end_x, 3), round(s.end_y, 3))
        had = {key(s0, s) for s in s0.segments if not getattr(s, 'graphic', False)}
        hadv = {(nm(s0, v.net_id), round(v.x, 3), round(v.y, 3)) for v in s0.vias}
        segs = [s for s in b.segments if not getattr(s, 'graphic', False) and nm(b, s.net_id) in want
                and key(b, s) not in had]
        vias = [v for v in b.vias if nm(b, v.net_id) in want
                and (nm(b, v.net_id), round(v.x, 3), round(v.y, 3)) not in hadv]
        by_name = collections.defaultdict(list)
        for p in pads:
            by_name[p.net_name].append(p)
        its = pieces(segs, vias)
        keep, tied = set(), set()
        for k, o, pc in its:
            for p in by_name.get(nm(b, o.net_id), ()):
                if _at(k, o, p):
                    keep.add(pc)
                    tied.add((p.component_ref, p.pad_number))
        strayed = set()
        if outer:
            strayed = {pc for k, o, pc in its if pc in keep and k == 's' and o.layer not in layers}
            keep -= strayed
        if within is not None:
            inb = lambda x, y: any(x0 - 1e-6 <= x <= x1 + 1e-6 and y0 - 1e-6 <= y <= y1 + 1e-6      # noqa: E731
                                   for x0, y0, x1, y1 in within)
            strayed |= {pc for k, o, pc in its if pc in keep and not (
                inb(o.x, o.y) if k == 'v' else inb(o.start_x, o.start_y) and inb(o.end_x, o.end_y))}
            keep -= strayed
        if strayed:
            tied = {(p.component_ref, p.pad_number) for k, o, pc in its if pc in keep
                    for p in by_name.get(nm(b, o.net_id), ()) if _at(k, o, p)}
        ids = {n.name: i for i, n in a.nets.items()}
        tracks = [{'start': (o.start_x, o.start_y), 'end': (o.end_x, o.end_y), 'width': o.width, 'layer': o.layer,
                   'net_id': ids[nm(b, o.net_id)]} for k, o, pc in its if k == 's' and pc in keep]
        vs = [{'x': o.x, 'y': o.y, 'size': o.size, 'drill': o.drill, 'layers': ['F.Cu', 'B.Cu'],
               'net_id': ids[nm(b, o.net_id)]} for k, o, pc in its if k == 'v' and pc in keep]
        if tracks or vs:
            with contextlib.redirect_stdout(io.StringIO()):
                add_tracks_and_vias_to_pcb(field, field, tracks, vs, [], net_id_to_name=names)
            ship_vias.stamp(field, 'group ends')    # (a tie's via in a pad or under solder declared filled and capped)
        return {'tied': sorted(tied), 'segments': len(tracks), 'vias': len(vs), 'exit': r.returncode, 'tracks': tracks,
                'vs': vs, 'strayed': len(strayed)}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def lay(field, parts, wayouts=(), log=None):
    """the pads beneath `parts` (the group's: its arrays and every bus's destinations) on a poured net tied on the
    board `field`, in place. `wayouts` [{'start', 'end', 'width', 'layer', 'net_id'}]: the lanes' ways out, laid on
    the copy the pass works on. Returns {'pads': the pads beneath, 'tied': those the kept copper reaches, 'segments',
    'vias': the kept copper's counts, 'tracks', 'vs': the copper itself, 'strayed': the pieces not kept, 'secs'} --
    None with no pad beneath on a poured net"""
    import time
    t0 = time.time()
    a = _load(field)
    poured = {z.net_id for z in (a.zones or []) if z.net_id}
    pads = beneath(a, set(parts), poured)
    if not pads:
        return None
    # (...and a tie stays in its part's FAN: led out past the exits and round them, one ran 2.7 mm outside an array
    # on an inner layer, half way round it, across where the lanes of two faces leave)
    # (...and on the OUTER layers: a tie is a stub on its pad's face and a via. A rail with no pour beneath the part
    # was run pad to pad on an inner layer along an array's edge, and the lanes leaving that face could not be laid)
    got = _pass(field, {a.nets[p.net_id].name for p in pads}, pads, wayouts, within=fans(a, set(parts), wayouts),
                outer=True)
    if not got['tied'] and got['exit'] and log:
        log(f'  field ties: the pass ended with exit {got["exit"]} -- ({field[:-len(".kicad_pcb")]}.ties.log)')
    return dict(got, pads=pads, secs=time.time() - t0)
