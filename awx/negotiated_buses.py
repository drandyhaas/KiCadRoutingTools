"""negotiated_buses.py -- several buses off ONE shared array routed NEGOTIATED: every bus at the same time, each in
its own frame, on one shared field, their solves coupled (route_bus's buses off one array, by default: route_buses;
--sequential routes them one after another instead).

    python3 route_bus.py BOARD.kicad_pcb OUT.kicad_pcb --bus SRC:DEST1[:MODE[:LAYERS]] --bus SRC:DEST2 ..

Buses that leave one array in different directions -- a star round it -- routed sequentially meet near it: a later
bus's lanes cross an earlier one's routed lanes where the earlier left them no room, and the bus routed first is never
moved for the one after. Negotiated, every bus is routed by its own whole route, in its own frame, all at the same
time, and every step of each is taken with the others in sight:

1. SOLO RUNS: each bus routed by itself on BOARD (route_bus, a process each, all at the same time). Where its escapes
   leave the shared array with nothing else there are its SOLO TEETH (solo_teeth).
2. THE SHARED FIELD (lay_field): the shared array planned ONCE for every bus off it -- joint_escape.plan_array, each
   bus net escaping toward its own destination (escape_to), the array's other nets and plane balls with them -- each
   bus ball preferring its solo tooth, the bus runs stacked as the other balls' are (stack_bus), and each escape priced
   for every other bus's lane its sweep to its destination crosses (sweep, against the solo teeth). Laid on BOARD in
   one call of the under-pad engine, the passives beneath the array nudged off it, and each plain bus's nets RESERVED
   in the shared field's project (protected, 'reserved'), so a whole-array bus's fanout plans round them.
3. NEGOTIATED: every bus routed on the shared field at the same time, a process each, each plain bus keeping its
   escapes there (SOURCE_SHARED), a whole-array bus laying its own again each round round theirs, as in its solo run.
   Their rounds' first plans and every loop's re-solve are one COUPLED SOLVE, the buses in step
   (whole_route.joint_sync and joint_loop_sync, whole_joint.py: where two buses' laid-out lanes cross or come within a
   lane's pitch, their layers differ there or the coupling is paid for), the couplings each coupled solve settles are
   held from there, and every stage plans round the other buses' lanes as they publish them (joint_lanes). A bus whose
   process ends is said ended right away, so no other waits for it.
4. MERGED (merge): OUT is the first bus's board with every other bus's nets' copper from that bus's own board -- those
   it routed and those it put back as they came --, every routed net protected and the reservations lifted -- a
   reserved net its bus did not hand on is the chain's to route. Each DRC violation between two buses' nets, or between a bus's net and another, is named (conflicts.json).

The buses share their source, each goes to parts of its own (DEST 'A+B': one bus to several parts, its nets each
escaping toward its own part -- a group bus, planned as one whole route in one frame), and they share their routing
layers, which the shared field is planned on. The first may be whole-array (its fanout plans the arrays' other balls,
one part), the rest are plain or escape (an escape bus's solo run is its own escape fanout; on the shared field it is
plain, keeping the escapes the field gave it): the merge takes from each other bus only its own nets. Every bus is its
own routing step, in its own process with its own pool of threads, so N buses are N runs at the same time. Everything is
written in OUT's work directory, <OUT stem>.bus/: solo/DEST/, shared_field.kicad_pcb, negotiation/ (the buses' meeting
place, JOINT_DIR), neg/DEST/, conflicts.json and summary.json.

Exits as route_buses does: 0 when every bus is handed on routed and the merge has no cross-bus conflict, 1 otherwise,
2 on a usage error.
"""
import collections
import contextlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
import awx_settings  # noqa: E402

POLL = 5.0          # (s) between looks at the buses' processes
RESERVED = 'reserved'   # (the protection a bus's nets carry on the shared field until the merge)


def check(buses):
    """None when `buses` [(src, dest, mode, layers)], their modes resolved, can be routed negotiated, else why not"""
    if len(buses) < 2:
        return 'two buses or more'
    srcs = sorted({b[0] for b in buses})
    if len(srcs) > 1:
        return f'the buses share one source, not {", ".join(srcs)}'
    parts = [p_ for b in buses for p_ in b[1].split('+')]
    if len(set(parts)) < len(parts):
        return 'each bus goes to parts of its own'
    if buses[0][2] not in ('whole-array', 'plain') or any(b[2] not in ('plain', 'escape') for b in buses[1:]) or \
            (buses[0][2] == 'whole-array' and '+' in buses[0][1]):
        return 'the first bus is whole-array or plain and the rest plain or escape: the merge takes from each other ' \
               'bus only its own nets'
    if len({b[3] for b in buses}) > 1:
        return 'the buses share their routing layers: the shared field is planned on one set'
    return None


def _bus_argv(board, out, bus, k, rounds, inproc, feedback_rounds, sizes):
    """route_bus.py's command line for `bus` by itself, BOARD to OUT, at the run's sizes"""
    src, dest, mode, layers = bus
    argv = [sys.executable, '-u', '-X', 'utf8', os.path.join(HERE, 'route_bus.py'), board, out,
            '--bus', ':'.join([src, dest, mode] + ([layers] if layers else [])), '--rounds', str(rounds)]
    if k:
        argv += ['--k', str(k)]
    if inproc:
        argv.append('--inproc')
    if feedback_rounds:
        argv += ['--feedback-rounds', str(feedback_rounds)]
    for name, v in sorted(sizes.items()):
        argv += ['--' + name.replace('_', '-'), str(v)]
    return argv


def _grade_line(log_path):
    """the last BUS line a bus's log has, or None"""
    got = None
    try:
        with open(log_path, encoding='utf-8', errors='replace') as f:
            for ln in f:
                if ln.startswith('BUS '):
                    got = ln.rstrip('\n')
    except OSError:
        pass
    return got


def _run_all(jobs, log, ended=None):
    """`jobs` [(name, argv, env, log path)] run at the same time, a process each, in awx/; {name: exit code}.
    `ended(name)` is called as each ends. Every process still running when this one stops is stopped with it"""
    procs = {}
    t0 = time.time()
    try:
        for name, argv, env, lp in jobs:
            f = open(lp, 'w', encoding='utf-8')
            procs[name] = (subprocess.Popen(argv, cwd=HERE, env=env, stdout=f, stderr=subprocess.STDOUT), f)
        codes = {}
        while len(codes) < len(procs):
            for name, (p, f) in procs.items():
                if name in codes or p.poll() is None:
                    continue
                f.close()
                codes[name] = p.returncode
                if ended:
                    ended(name)
                g = _grade_line(next(lp for n_, _a, _e, lp in jobs if n_ == name))
                log(f'  {name}: exit {p.returncode} after {int(time.time() - t0)} s' + (f' -- {g}' if g else ''))
            if len(codes) < len(procs):
                time.sleep(POLL)
        return codes
    finally:
        for p, f in procs.values():
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    p.kill()
            f.close()


def solo_teeth(board, ref, nets, grid):
    """{net: {'tooth', 'direction', 'layer', 'kind'}}: where each of `nets` (short names) leaves `ref` on `board`, a
    bus's solo run -- the point its copper, from the ball out, crosses the exit line (`grid`'s box grown escape_plan.
    PAST pitches; the crossing nearest its ball), the face it crosses, the layer, and its site's kind: a surface run,
    a via in the pad, or a dog-bone (its via within a pitch and a half of the ball, inside the array's box)"""
    import escape_plan as ep
    from coherent_nets import namer
    from kicad_parser import parse_kicad_pcb
    x0, y0, x1, y1 = grid.bbox
    ex = (x0 - ep.PAST * grid.pitch_x, y0 - ep.PAST * grid.pitch_y, x1 + ep.PAST * grid.pitch_x,
          y1 + ep.PAST * grid.pitch_y)
    ins = lambda p: ex[0] < p[0] < ex[2] and ex[1] < p[1] < ex[3]   # noqa: E731
    with contextlib.redirect_stdout(io.StringIO()):
        b = parse_kicad_pcb(board)
    sb = namer(b)
    bf = b.footprints[ref]
    out = {}
    for nm in nets:
        ids = {i for i, n in b.nets.items() if n.name and sb(n.name) == nm}
        if not ids:
            continue
        nid = next(iter(ids))
        ball = next((p for p in bf.pads if p.net_id == nid), None)
        if ball is None:
            continue
        best = None
        for s in b.segments:
            if s.net_id != nid:
                continue
            a, c = (s.start_x, s.start_y), (s.end_x, s.end_y)
            if ins(a) == ins(c):
                continue
            pin, pout = (a, c) if ins(a) else (c, a)
            for d, (k, val) in (('left', (0, ex[0])), ('right', (0, ex[2])), ('up', (1, ex[1])), ('down', (1, ex[3]))):
                if (pout[k] - val) * (pin[k] - val) > 0 or abs(pout[k] - pin[k]) < 1e-9:
                    continue
                t = (val - pin[k]) / (pout[k] - pin[k])
                xpt = (pin[0] + t * (pout[0] - pin[0]), pin[1] + t * (pout[1] - pin[1]))
                o = 1 - k
                if not (ex[o] - 1e-6 <= xpt[o] <= ex[o + 2] + 1e-6):
                    continue
                dist = math.hypot(pin[0] - ball.global_x, pin[1] - ball.global_y)
                if best is None or dist < best[0]:
                    best = (dist, xpt, d, s.layer)
        if best is None:
            continue
        site = None
        for v in b.vias:
            if v.net_id == nid and x0 - grid.pitch_x / 2 - 1e-6 <= v.x <= x1 + grid.pitch_x / 2 + 1e-6 and \
                    y0 - grid.pitch_y / 2 - 1e-6 <= v.y <= y1 + grid.pitch_y / 2 + 1e-6 and \
                    math.hypot(v.x - ball.global_x, v.y - ball.global_y) < 1.5 * grid.pitch_x:
                site = (v.x, v.y)
        kind = 'surface' if site is None else \
            ('via_in_pad' if math.hypot(site[0] - ball.global_x, site[1] - ball.global_y) < 0.05 else 'dogbone')
        out[nm] = {'tooth': best[1], 'direction': best[2], 'layer': best[3], 'kind': kind}
    return out


def field_teeth(board, ref, nets):
    """{net: (x, y)}: where each of `nets` (short names) leaves `ref`'s array on a shared field `board` -- its escape's
    copper (a stub's end, a dog-bone's via) farthest from its ball within the array's box grown escape_plan.PAST
    pitches: the shared field's escapes stop at the face, short of the exit line solo_teeth reads"""
    import escape_moves as em
    import escape_plan as ep
    from coherent_nets import namer
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        b = parse_kicad_pcb(board)
    sn = namer(b)
    grid = em.grid_of(b.footprints[ref])
    x0, y0, x1, y1 = grid.bbox
    ex = (x0 - ep.PAST * grid.pitch_x, y0 - ep.PAST * grid.pitch_y, x1 + ep.PAST * grid.pitch_x,
          y1 + ep.PAST * grid.pitch_y)
    ins = lambda q: ex[0] <= q[0] <= ex[2] and ex[1] <= q[1] <= ex[3]   # noqa: E731
    want = set(nets)
    out = {}
    for p in b.footprints[ref].pads:
        nm = sn(p.net_name or '')
        if nm not in want or nm in out:
            continue
        pts = [q for s in b.segments if s.net_id == p.net_id for q in ((s.start_x, s.start_y), (s.end_x, s.end_y))]
        pts += [(v.x, v.y) for v in b.vias if v.net_id == p.net_id]
        pts = [q for q in pts if ins(q)]
        if pts:
            out[nm] = max(pts, key=lambda q: math.hypot(q[0] - p.global_x, q[1] - p.global_y))
    return out


def lay_field(board, out, ref, buses, reserve, clearance, work, log=print, avoid=None):
    """The shared array `ref` on `board` planned once for every bus off it and laid to `out`, the passives beneath it
    nudged off it and the nets of the buses `reserve` names reserved in `out`'s project. `buses` [(dest, nets, teeth)]:
    each bus's routed nets (short names) and their solo teeth (solo_teeth). `avoid` {net: [(x, y), ..]}: teeth the
    buses' rounds found crowded -- such a net prefers no tooth, and an exit near one is paid (joint_escape.C_AVOID).
    Under the run's rules and step policy (route_bus.step_policy). Returns the plan's report"""
    import joint_escape as je
    import route_bus as rb
    import whole_route as wr
    from coherent_nets import namer
    from kicad_parser import parse_kicad_pcb
    from protected_nets import persist_protected_nets, pro_path_for_board
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    sn = namer(pcb)
    foot = pcb.footprints[ref]
    bus_all = sorted({n for _d, ns, _t in buses for n in ns})
    # (each bus net escaping toward ITS part: a bus to several parts, DEST 'A+B' -- one whole route, a trunk to the
    # first and a branch to each other -- has its nets going to each)
    import flow_frame as ff
    part_of = {}
    for d, ns, _t in buses:
        for P_ in ff.dest_refs(d):
            for q in pcb.footprints[P_].pads:
                if q.net_name and sn(q.net_name) in ns:
                    part_of.setdefault(sn(q.net_name), P_)
    escape_to = {n: part_of.get(n, ff.dest_refs(d)[0]) for d, ns, _t in buses for n in ns}
    far = {n: je.far_face(pcb, ref, escape_to[n]) for n in escape_to}
    prefer = {n: g for _d, _ns, t in buses for n, g in t.items() if n not in (avoid or {})}
    zone = {z.net_id for z in (pcb.zones or []) if z.net_id}
    others = sorted(n for n in rb.array_others(pcb, ref, []) if sn(n) not in set(bus_all))
    drops = sorted({p.net_name for p in foot.pads if p.net_id in zone})
    full = {sn(n.name): n.name for n in pcb.nets.values() if n.name}
    bus_full = [full[n] for n in bus_all if n in full]
    layers = list(pcb.board_info.copper_layers)
    t0 = time.time()
    lines = []
    hints, rep = je.plan_array(pcb, ref, bus_full, others, layers, far=far, prefer=prefer, drops=drops,
                               log=lines.append, escape_to=escape_to, exit_rays=True, stack_bus=True, sweep=True,
                               avoid=avoid)
    log(f'  the shared field of {ref}: planned for {len(buses)} buses in {time.time() - t0:.0f} s, {rep.get("status")} '
        f'-- bus {rep.get("bus_escaped")}/{rep.get("bus_balls")} escaped, others {rep.get("others_escaped")} escaped + '
        f'{rep.get("others_strapped")} strapped of {rep.get("others_balls")}, plane {rep.get("dropped")} dropped + '
        f'{rep.get("plane_strapped")} strapped of {rep.get("plane_balls")}')
    for ln in lines:
        if any(w in ln for w in ('INFEASIBLE', 'let go', 'unplanned', 'uncoloured', 'held')):
            log('  ' + ln.strip()[:220])
    with open(os.path.join(work, 'shared_field.lay.log'), 'w', encoding='utf-8') as lf, \
            contextlib.redirect_stdout(lf), contextlib.redirect_stderr(lf):
        nt, nv, failed = je.lay(board, out, ref, bus_full, others, layers, hints)
    log(f'  the shared field laid: {nt} tracks, {nv} vias' + (f', the engine left {failed}' if failed else ''))
    # (the passives beneath the array nudged off the shared field, as an escape bus's step nudges them off its own;
    # those beneath another array are another step's -- joint_escape.passives_beyond, locked)
    lock_ = je.passives_beyond(out, [ref], bus_full)
    caps = os.path.join(work, 'shared_field.caps.log')
    rc = wr.run([os.path.join('..', 'py_router', 'place_fanout_clearance.py'), out, out, '--clearance', str(clearance),
                 '--beneath-only'] + (['--lock'] + lock_ if lock_ else []), awx_settings.environ(), log_path=caps,
                own_process=True)[0]
    for ln in wr.grep(caps, r'^(Moved|Stuck|  Unresolved)'):
        log('  caps: ' + ln.strip()[:200])
    if rc != 0:
        log(f'  caps: the nudge exited {rc} -- the passives stay where they were')
    # (a ROW PART destination's passives moved off its pads' escapes on the shared field, once, as each bus's own step
    # moves them: every bus then starts from the same shared field -- route_bus.clear_row_escapes)
    import make_bench as mb
    for d in [P_ for d_, _ns, _t in buses for P_ in ff.dest_refs(d_)]:
        names_ = set(mb.pair_nets(pcb, ref, d))
        names_ |= set(rb.pulled_nets(pcb, ref, d, sorted(names_)))
        keep_ = {q.component_ref for n in pcb.nets.values() if n.name in names_ for q in n.pads} - {ref, d}
        rb.clear_row_escapes(out, d, sorted(names_), sorted(keep_), work, log=log)
    res = {full[n]: RESERVED for d, ns, _t in buses if d in reserve for n in ns if n in full}
    with contextlib.redirect_stdout(io.StringIO()):
        persist_protected_nets(pro_path_for_board(out), res)
    if res:
        log(f'  reserved on the shared field: {len(res)} nets of {", ".join(reserve)}')
    return rep


def plan_field(board, field, buses, solo, work, log=print, avoid=None, **sizes):
    """The shared field of the buses' source on `board`, laid to `field` (lay_field), from each bus's solo run `solo`
    [(board, summary) or None, as `buses`]: its routed nets and their solo teeth -- and `avoid`, the teeth the buses'
    rounds found crowded (lay_field). Under the run's rules (the chain's `sizes`, route_bus.resolve_rules) and step
    policy, the buses' routing layers. Returns the plan's report"""
    import route_bus as rb
    import rules as _rules
    src, layers = buses[0][0], buses[0][3]
    run_rules = rb.resolve_rules(board, log=lambda *a: None, **sizes)
    prev = _rules.active()
    _rules.install(run_rules)
    try:
        with rb.step_policy(run_rules, {'ROUTE_LAYERS': layers} if layers else None):
            # (the shared field's modules imported under the step's policy: a module reads it when first imported)
            import escape_moves as em
            from kicad_parser import parse_kicad_pcb
            with contextlib.redirect_stdout(io.StringIO()):
                pcb_ = parse_kicad_pcb(board)
            grid = em.grid_of(pcb_.footprints[src])
            import flow_frame as ff
            import make_bench as mb
            from coherent_nets import namer
            sn_ = namer(pcb_)
            got_ = []
            for b, got in zip(buses, solo):
                routed = sorted(json.load(open(got[1])).get('routed') or []) if got else []
                teeth = solo_teeth(got[0], src, routed, grid) if routed else {}
                # (every net of the bus the shared field's, toward its parts -- those its run routed with their teeth
                # to prefer, the rest with none: planned as the array's others, a net its run left open left by any
                # face, never reserved for its bus. Its nets as its step takes them: the pairs and the nets pulled
                # through a part between -- a memory bus's 48, 22 of them pairs)
                nets = set(routed)
                for P_ in ff.dest_refs(b[1]):
                    pn_ = mb.pair_nets(pcb_, src, P_)
                    nets |= {sn_(n) for n in list(pn_) + list(rb.pulled_nets(pcb_, src, P_, sorted(pn_)))}
                nets = sorted(nets)
                log(f'  {b[1]}: {len(nets)} net(s), {len(teeth)} with a tooth from its run'
                    + (f' ({len(routed)} routed there)' if got else ' -- no run: no tooth to prefer'))
                got_.append((b[1], nets, teeth))
            if avoid:
                log(f'  the shared field again, {sum(len(v_) for v_ in avoid.values())} crowded teeth of {len(avoid)} '
                    f'net(s) to avoid: {", ".join(sorted(avoid))}')
            return lay_field(board, field, src, got_, [b[1] for b in buses if b[2] != 'whole-array'],
                             run_rules.clearance, work, log=log, avoid=avoid)
    finally:
        _rules.install(prev)


def _set_protected(pro, mapping):
    """`pro`'s protected nets made `mapping` (persist_protected_nets merges into the map, and a reservation lifted is
    one taken out of it)"""
    import tool_records
    from protected_nets import PRO_KEY, PRO_NAMESPACE
    proj = tool_records.load_project(pro)
    proj.setdefault(PRO_NAMESPACE, {})[PRO_KEY] = dict(sorted(mapping.items()))
    tool_records.stamp_revision(proj)
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)


def merge(out, boards, log=print, field=None):
    """OUT: the first of `boards` [(board, summary)] with each other's nets' copper taken from its own board -- every
    net of that bus, the ones it routed and the ones it put back as they came (its summary's 'routed' and 'refused'): a
    net put back keeps only the pieces of its escape the bus's lanes still clear, which the shared field's whole escape
    on the first board does not --, their copper on the first off first; OUT's project the first's, its protected nets
    every board's but the reservations (a net a bus routed protected as that bus has it); and with `field` (the board
    the buses were routed on), each part another bus moved from where the shared field has it -- a passive off a row
    part's escapes (route_bus.clear_row_escapes) -- where that bus left it. Returns {net short name: the bus (dest) that
    routed it}"""
    import make_bench as mb
    from copy_board import copy_board
    from coherent_nets import net_keys
    from kicad_parser import parse_kicad_pcb
    from protected_nets import read_protected_nets, pro_path_for_board
    base_b, rest = boards[0][0], boards[1:]
    with contextlib.redirect_stdout(io.StringIO()):
        pb = parse_kicad_pcb(base_b)
    txt = open(base_b, encoding='utf-8').read()
    full_of = {}
    for full, short in net_keys(pb).items():
        full_of.setdefault(short, full)
    bus_of, prot = {}, {}
    for b, s in boards:
        j = json.load(open(s))
        for n in j.get('routed') or []:
            bus_of[n] = j['dest']
        for n, why in read_protected_nets(pro_path_for_board(b)).items():
            if why != RESERVED or n not in prot:
                prot[n] = why
    add = []
    for b, s in rest:
        j = json.load(open(s))
        names = {full_of[n] for n in list(j.get('routed') or []) + list(j.get('refused') or {}) if n in full_of}
        txt, _n = mb.strip_pair_copper(txt, pb, names)
        with contextlib.redirect_stdout(io.StringIO()):
            pbb = parse_kicad_pcb(b)
        ids = {i for i, n in pbb.nets.items() if n.name in names}

        def grab(block, ids=ids, names=names):
            m = re.search(r'\(net (\d+)\)', block)
            m2 = re.search(r'\(net "([^"]+)"\)', block)
            if (m and int(m.group(1)) in ids) or (m2 and m2.group(1) in names):
                add.append(block)
            return False
        bt = open(b, encoding='utf-8').read()
        mb.walk_blocks(bt, 'segment', grab)
        mb.walk_blocks(bt, 'via', grab)
        log(f'  merged: {j["dest"]}\'s {len(names)} net(s), {len(j.get("routed") or [])} of them routed')
    end = txt.rstrip().rfind(')')
    txt = txt[:end].rstrip() + '\n' + ''.join(f'\t{b_}\n' for b_ in add) + txt[end:]
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(base_b, out)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(txt)
    if field:
        import whole_route as wr
        was = wr.footprint_poses(field)
        moved = {}
        for b, s in rest:
            moved.update({r_: p_ for r_, p_ in wr.footprint_poses(b).items() if r_ in was and was[r_] != p_})
        if moved:
            got = wr.place_held(out, out[:-len('.kicad_pcb')] + '.posed.kicad_pcb', moved)
            if got != out:
                shutil.copyfile(got, out)
            log(f'  merged: {len(moved)} part(s) where their bus moved them: ' + ', '.join(sorted(moved)))
    _set_protected(pro_path_for_board(out), {n: v for n, v in prot.items() if v != RESERVED})
    return bus_of


def conflicts(out, bus_of, path, log=print):
    """The merged board's DRC (check_drc at its project's floor) read for violations between two buses' nets, or a
    bus's and another net's, written to `path` [{type, net1, bus1, net2, bus2, layer, at}]; their number, or None when
    the check gave nothing to read"""
    from coherent_nets import net_keys
    from kicad_parser import parse_kicad_pcb
    jp = path[:-len('.json')] + '.drc.json'
    subprocess.run([sys.executable, os.path.join(HERE, '..', 'py_router', 'check_drc.py'), out, '--clearance-margin',
                    '0.1', '--max-print', '0', '--json', jp], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        items = [it for it in (json.load(open(jp)).get('items') or []) if not it.get('accepted')]
    except (OSError, ValueError):
        log('  the merged board\'s DRC gave nothing to read')
        return None
    with contextlib.redirect_stdout(io.StringIO()):
        keys = net_keys(parse_kicad_pcb(out))
    sk = lambda n: keys.get(n, (n or '').rsplit('/', 1)[-1])  # noqa: E731

    def at(it):
        for k in ('cross_point', 'closest_pt1', 'via_loc', 'pad_loc'):
            if it.get(k):
                return [round(float(v), 3) for v in it[k][:2]]
        l_ = it.get('loc1')
        if l_:
            return [round((l_[0] + l_[2]) / 2, 3), round((l_[1] + l_[3]) / 2, 3)] if len(l_) == 4 else list(l_[:2])
        return None
    conf = []
    for it in items:
        n1, n2 = sk(it.get('net1')), sk(it.get('net2'))
        b1, b2 = bus_of.get(n1), bus_of.get(n2)
        # (two NETS: a bus net against no net -- the board's edge, a hole -- is its own run's violation, which no
        # merge makes and nothing here settles; counted between buses, it failed the step with nothing to give back)
        if n1 and n2 and '<no net>' not in (n1, n2) and (b1 or b2) and b1 != b2:
            conf.append({'type': it['type'], 'net1': n1, 'bus1': b1, 'net2': n2, 'bus2': b2, 'layer': it.get('layer'),
                         'at': at(it)})
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(conf, f, indent=1)
    pairs = collections.Counter(tuple(sorted((x['net1'], x['net2']))) for x in conf)
    log(f'  the merged board: {len(items)} DRC violation(s), {len(conf)} between buses'
        + (': ' + ', '.join(f'{a}/{b}' for a, b in sorted(pairs))[:600] if conf else ''))
    return len(conf)


def give_back(out, board, path, log=print):
    """The merged board's conflicts between a bus's net and ANOTHER net (conflicts.json at `path`) settled by giving the
    other net back: its copper this step laid -- on `out` and not on the board the step came in with, `board` -- taken
    off `out`, the net left for the rest of the chain to route (another net's copper the step may take up, a bus net's
    never). A whole-array bus lays the shared array's other nets again each round, round its own escapes there, and the
    other buses, routed round the shared field's, never see it: an other net's escape laid onto the layer another bus's
    lane ran on was the merged board's one conflict. Two buses' nets meeting stay as they are, named. Returns the nets given back"""
    import make_bench as mb
    from coherent_nets import net_keys
    from kicad_parser import parse_kicad_pcb
    try:
        conf = json.load(open(path))
    except (OSError, ValueError):
        return []
    give = sorted({c_['net2'] if c_['bus1'] else c_['net1'] for c_ in conf
                   if bool(c_['bus1']) != bool(c_['bus2']) and (c_['net2'] if c_['bus1'] else c_['net1'])})
    if not give:
        return []
    with contextlib.redirect_stdout(io.StringIO()):
        po, pi = parse_kicad_pcb(out), parse_kicad_pcb(board)
    # (never a net with a POUR: its copper here is the whole-array fanout's drops at every array, and given back for
    # one drop that met a lane, every drop of the plane went -- as the step's own grade never puts a zone's net back;
    # its conflict stays named for the rest of the chain)
    keys_ = net_keys(po)
    zone_ = {keys_.get(po.nets[z_.net_id].name, po.nets[z_.net_id].name) for z_ in (po.zones or [])
             if z_.net_id in po.nets}
    kept_ = sorted(set(give) & zone_)
    give = [g_ for g_ in give if g_ not in zone_]
    if kept_:
        log(f'  the merged board: {", ".join(kept_)} met a bus\'s net but pour(s) -- not given back, named')
    if not give:
        return []
    full = {f_ for f_, s_ in keys_.items() if s_ in give}
    ids = {i_ for i_, n_ in po.nets.items() if n_.name in full}
    r3 = lambda p_: (round(p_[0], 3), round(p_[1], 3))   # noqa: E731
    came = {frozenset((r3((s_.start_x, s_.start_y)), r3((s_.end_x, s_.end_y)))) | {s_.layer} for s_ in pi.segments
            if s_.net_id in pi.nets and pi.nets[s_.net_id].name in full}
    came |= {frozenset({r3((v_.x, v_.y)), 'via'}) for v_ in pi.vias
             if v_.net_id in pi.nets and pi.nets[v_.net_id].name in full}
    gone = [0]

    def laid_here(token):
        def f(block):
            m = re.search(r'\(net (\d+)\)', block)
            m2 = re.search(r'\(net "([^"]+)"\)', block)
            if not ((m and int(m.group(1)) in ids) or (m2 and m2.group(1) in full)):
                return False
            a_, b_, L_ = mb.block_geom(block, token)
            key = frozenset((r3(a_), r3(b_))) | {L_} if token == 'segment' else frozenset({r3(a_), 'via'})
            if key in came:
                return False
            gone[0] += 1
            return True
        return f
    txt = open(out, encoding='utf-8').read()
    txt = mb.walk_blocks(txt, 'segment', laid_here('segment'))
    txt = mb.walk_blocks(txt, 'via', laid_here('via'))
    with open(out, 'w', encoding='utf-8') as f:
        f.write(txt)
    log(f'  the merged board: {", ".join(give)} given back -- {gone[0]} piece(s) of copper the step laid them taken off, '
        f'theirs to route in the rest of the chain (each met a bus\'s net there)')
    return give


def route_negotiated(board, out, buses, k=None, rounds=4, inproc=False, log=print, json_out=None, feedback_rounds=0,
                     solo=None, **sizes):
    """The buses `buses` [(src, dest, mode, layers)], their modes resolved, routed negotiated on `board` (the module's
    docstring), `out` the merged board. `solo` [(board, summary) or None, as `buses`]: each bus's run already made --
    route_buses' pass in turn, each bus on the board the ones before it left -- its teeth the shared field's, in place
    of a solo run (None: no teeth to prefer). Returns (exit code, grade lines)"""
    board, out = os.path.abspath(board), os.path.abspath(out)
    if not out.endswith('.kicad_pcb'):
        out += '.kicad_pcb'
    why = check(buses)
    if why:
        log(f'route_bus: the buses off one array, not negotiated -- {why}')
        return 2, []
    work = out[:-len('.kicad_pcb')] + '.bus'
    os.makedirs(work, exist_ok=True)
    src, dests, modes = buses[0][0], [b[1] for b in buses], [b[2] for b in buses]
    t0 = time.time()

    # 1. each bus's solo run -- or its run already made (`solo`)
    log(f'route_bus: {len(buses)} buses off {src} negotiated -- {", ".join(f"{d} ({m})" for d, m in zip(dests, modes))}'
        + ('; each bus\'s teeth from its run in turn' if solo is not None else '; a solo run of each first'))
    if solo is None:
        _run_all([_bus_job(work, 'solo', b, board, _env0(), k, rounds, inproc, feedback_rounds, sizes) for b in buses],
                 log)
        solo = [_bus_out(work, 'solo', b) for b in buses]

    # 2. the shared field
    field = os.path.join(work, 'shared_field.kicad_pcb')
    rep = plan_field(board, field, buses, solo, work, log=log, **sizes)

    # 3, 4. every bus negotiated on the shared field, merged
    code, grades = route_on_field(board, field, out, buses, k, rounds, inproc, log, json_out, feedback_rounds,
                                  field_plan=rep, t0=t0, **sizes)
    return code, grades


def _env0():
    """the environment a bus's process takes: the run's, but none of a meeting's (a JOINT_DIR left over would have a
    solo run wait for buses that never come)"""
    return {k_: v_ for k_, v_ in awx_settings.environ().items()
            if not k_.startswith('JOINT_') and k_ != 'SOURCE_SHARED'}


def _bus_job(work, phase, bus, inp, env, k, rounds, inproc, feedback_rounds, sizes):
    """(name, argv, env, log path): `bus` routed by itself on its own copy of `inp`, in work/PHASE/DEST/"""
    from copy_board import copy_board
    d_ = os.path.join(work, phase, bus[1])
    # (the bus's directory its own run's: a process that died before it wrote left an earlier run's board and summary
    # there for _bus_out to merge as this one's)
    shutil.rmtree(d_, ignore_errors=True)
    os.makedirs(d_, exist_ok=True)
    own = os.path.join(d_, 'input.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(inp, own)
    return (bus[1], _bus_argv(own, os.path.join(d_, 'bus.kicad_pcb'), bus, k, rounds, inproc, feedback_rounds, sizes),
            env, os.path.join(d_, 'bus.log'))


def _bus_out(work, phase, bus):
    """(board, summary) a bus's process wrote in work/PHASE/DEST/, or None"""
    d_ = os.path.join(work, phase, bus[1])
    b_, s_ = os.path.join(d_, 'bus.kicad_pcb'), os.path.join(d_, 'bus.bus', f'{bus[0]}-{bus[1]}.summary.json')
    return (b_, s_) if os.path.isfile(b_) and os.path.isfile(s_) else None


def route_on_field(board, field, out, buses, k=None, rounds=4, inproc=False, log=print, json_out=None,
                   feedback_rounds=0, field_plan=None, t0=None, **sizes):
    """Steps 3 and 4: the buses `buses` routed negotiated on the shared field `field` (lay_field's, from `board`), and
    merged to `out`. Returns (exit code, grade lines)"""
    from copy_board import copy_board
    board, field, out = os.path.abspath(board), os.path.abspath(field), os.path.abspath(out)
    work = out[:-len('.kicad_pcb')] + '.bus'
    os.makedirs(work, exist_ok=True)
    src, dests = buses[0][0], [b[1] for b in buses]
    t0 = t0 or time.time()
    jd = os.path.join(work, 'negotiation')
    shutil.rmtree(jd, ignore_errors=True)
    os.makedirs(jd)
    log(f'  every bus on the shared field, negotiated (meeting in {os.path.relpath(jd, os.path.dirname(out))})')
    # (a whole-array bus keeps the escapes the shared field laid it there too, as a plain bus does: the field planned
    # them with every other bus's. Fanned again round the others' escapes, a memory bus's comb sent an address ball out
    # by the face another bus's lanes leave by, among their exits, and split a strobe pair across two layers)
    # (an ESCAPE bus is plain on the shared field: the shared field escaped its balls toward its destination with every
    # other bus's, as its own escape fanout does in its solo run, and it keeps them there as a plain bus keeps its own;
    # its solo run, which its solo teeth came of, was an escape bus's. Turned away, an escape bus off an array with
    # three other buses was routed sequentially with them, and the two routed after it found their faces taken by its
    # lanes)
    jobs = [_bus_job(work, 'neg', (b[0], b[1], 'plain' if b[2] == 'escape' else b[2], b[3]), field,
                     {**_env0(), 'SOURCE_SHARED': src,
                      'JOINT_DIR': jd, 'JOINT_PEERS': ','.join(dests)},
                     k, rounds, inproc, feedback_rounds, sizes)
            for b in buses]
    codes = _run_all(jobs, log, ended=lambda d: open(os.path.join(jd, f'{d}.finished'), 'w').close())

    # 4. merged
    boards = [_bus_out(work, 'neg', b) for b in buses]
    have = [x for x in boards if x]
    n_conf, given = None, []
    if have:
        bus_of = merge(out, have, log=log, field=field)
        n_conf = conflicts(out, bus_of, os.path.join(work, 'conflicts.json'), log=log)
        if n_conf:
            # (a bus's net meeting another net there: the other net given back, for the chain -- give_back)
            given = give_back(out, board, os.path.join(work, 'conflicts.json'), log=log)
            if given:
                n_conf = conflicts(out, bus_of, os.path.join(work, 'conflicts.json'), log=log)
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(board, out)
        log('route_bus: no bus off the array wrote a board -- OUT is the board as it came')
    # (the merged board graded as a bus step's own board is: every bus net connected on its bus's parts, no DRC the step
    # made, no ball walled in -- each bus graded its own board alone, and the buses' lanes stand together here)
    grade_ = None
    if have:
        import route_bus as rb
        nets_ = []
        for b, x in zip(buses, boards):
            j_ = json.load(open(x[1])) if x else {}
            nets_.append((b[0], b[1].split('+'), list(j_.get('routed') or []) + list(j_.get('refused') or {})))
        grade_ = rb.board_grade(board, out, nets_, work, log=log, tag='merged')
    runs = []
    for b, x in zip(buses, boards):
        j = json.load(open(x[1])) if x else {}
        # (the step's own status is its summary's: the process exits 0 whenever it wrote OUT, #1738)
        ex = j.get('exit', codes.get(b[1])) if x else codes.get(b[1])
        runs.append(dict(src=b[0], dest=b[1], mode=b[2], exit=ex, routed=len(j.get('routed') or []),
                         grade=_grade_line(os.path.join(work, 'neg', b[1], 'bus.log')), summary=x[1] if x else None))
    code = max([1 if r['exit'] is None else r['exit'] for r in runs] + [0 if n_conf == 0 else 1])
    with open(json_out or os.path.join(work, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(board=board, out=out, negotiated=True, src=src, field=field,
                       field_plan={k_: v_ for k_, v_ in (field_plan or {}).items()
                                   if isinstance(v_, (int, float, str))},
                       buses=runs, conflicts=n_conf, given_back=given, grade=grade_, secs=int(time.time() - t0),
                       exit=code), f, indent=1)
    log(f'route_bus: negotiated, {sum(r["routed"] for r in runs)} nets routed by {len(runs)} buses, '
        f'{"no" if n_conf == 0 else n_conf if n_conf is not None else "ungraded"} cross-bus conflict(s), '
        f'{int(time.time() - t0)} s -- {os.path.basename(out)}')
    return code, [r['grade'] for r in runs]
