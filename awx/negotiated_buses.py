"""negotiated_buses.py -- the buses off ONE source array routed as one GROUP (route_bus.route_buses: each source's
buses, found by route_bus.bus_plan or named by --bus; a bus alone on its array is a group of one).

    python3 route_bus.py BOARD.kicad_pcb OUT.kicad_pcb [--bus SRC:DEST1 --bus SRC:DEST2 ..]

route_grouped:
1. CAPTURE: each bus's ends model captured by itself on BOARD (route_bus --src/--dest under ENDS_ONLY and
   ENDS_CAPTURE, a process each, all at the same time, each stopped as soon as its capture is written).
2. THE GROUP'S ENDS (group_ends.py): every bus's teeth and berths chosen in ONE model, with the arrays' fanout -- every
   ball of the shared source and of each destination that is a ball array served -- and laid on BOARD: the SHARED
   FIELD.
3. ON THE FIELD (route_on_field): every bus routed on the shared field at the same time, a process each (GROUP_ENDS:
   its ends the field's), its own whole route in its own frame. With more buses than one their rounds' first plans
   and every loop's re-solve are one COUPLED SOLVE, the buses in step (JOINT_DIR: whole_route.joint_sync and
   joint_loop_sync, whole_joint.py: where two buses' laid-out lanes cross or come within a lane's pitch, their layers
   differ there or the coupling is paid for), and every stage plans round the other buses' lanes as they publish them
   (joint_lanes). A bus whose process ends is said ended right away, so no other waits for it.
4. MERGED (merge): OUT is the first bus's board with every other bus's nets' copper from that bus's own board -- those
   it routed and those it put back as they came --, every routed net protected. Each DRC violation between two buses'
   nets, or between a bus's net and another, is named (conflicts.json); another net's copper in one is given back to
   the chain (give_back).
The group's ROUNDS: a round that leaves the board short of clean sends what its buses' steps found at their ends to the
model (group_ends --from --feedback), which answers from the state it had, and the buses are routed again on that field;
the best round's board is OUT.

Every bus is its own routing step, in its own process with its own pool of threads, so N buses are N runs at the same
time. Everything is written in OUT's work directory, <OUT stem>.bus/: caps/, capture/DEST/, group/ (the model's
output, the shared field), solo/DEST/ or neg/DEST/, negotiation/ (the buses' meeting place), conflicts.json and
summary.json.

Exits as route_buses does: 0 when every bus is handed on routed and the merge has no cross-bus conflict, 1 otherwise,
2 on a usage error.
"""
import collections
import contextlib
import io
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
import awx_settings  # noqa: E402
import bus_events  # noqa: E402
from coherent_nets import block_net_name  # noqa: E402

POLL = 5.0          # (s) between looks at the buses' processes
RESERVED = 'reserved'   # (the protection a bus's nets carry on the shared field until the merge)


def _bus_argv(board, out, bus, k, rounds, inproc, feedback_rounds, sizes):
    """route_bus.py's command line for `bus` by itself, BOARD to OUT, at the run's sizes"""
    src, dest = bus
    # (the bus BY ITSELF: route_bus.route_bus, its summary where _bus_out reads it; its layers the environment's
    # ROUTE_LAYERS, which route_buses gives every bus's process)
    argv = [sys.executable, '-u', '-X', 'utf8', os.path.join(HERE, 'route_bus.py'), board, out,
            '--src', src, '--dest', dest, '--rounds', str(rounds),
            '--json-out', os.path.join(out[:-len('.kicad_pcb')] + '.bus', f'{src}-{dest}.summary.json')]
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


def _run_all(jobs, log, ended=None, src=None):
    """`jobs` [(name, argv, env, log path)] run at the same time, a process each, in awx/; {name: exit code}.
    `ended(name)` is called as each ends. Every process still running when this one stops is stopped with it. `src`:
    the buses' source, for their progress events (bus_events)"""
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
                bus_events.emit('bus', 'exit', group=src, bus=f'{src}:{name}', exit=p.returncode,
                                secs=int(time.time() - t0), log=next(lp for n_, _a, _e, lp in jobs if n_ == name))
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
            m2 = block_net_name(block)
            if (m and int(m.group(1)) in ids) or (m2 is not None and m2 in names):
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
    # (the merged project's floors: the first board's, lowered to the copper every bus laid, as every stage's own board
    # is stamped. A bus whose step narrowed a lane's end to the fab floor wrote that floor to its own project alone,
    # and the merged board, graded at the first bus's, read that bus's tracks as under width)
    import fanout_from_plan as fp_
    with contextlib.redirect_stdout(io.StringIO()):
        fp_.copy_pro(base_b, out)
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
            m2 = block_net_name(block)
            if not ((m and int(m.group(1)) in ids) or (m2 is not None and m2 in full)):
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


def protect_laid(board, out, log=print):
    """The POURED nets the step laid copper of on `out` -- a plane ball's stub and via, a tie beneath a bus's part:
    copper `board`, the board as it came, has not -- recorded protected in `out`'s project beside the bus nets. A
    later step that rips such a net to try one of its taps again takes that copper with it, beneath the buses'
    lanes, where no later step can lay it again. Returns their names"""
    from kicad_parser import parse_kicad_pcb
    from protected_nets import persist_protected_nets, pro_path_for_board, read_protected_nets
    with contextlib.redirect_stdout(io.StringIO()):
        a, b = parse_kicad_pcb(board), parse_kicad_pcb(out)
    poured = {z.net_id for z in (b.zones or []) if z.net_id}
    nm = lambda p, i: p.nets[i].name if i in p.nets else ''        # noqa: E731
    key = lambda p, s: (nm(p, s.net_id), s.layer, round(s.start_x, 3), round(s.start_y, 3),   # noqa: E731
                        round(s.end_x, 3), round(s.end_y, 3))
    had = {key(a, s) for s in a.segments if not getattr(s, 'graphic', False)}
    hadv = {(nm(a, v.net_id), round(v.x, 3), round(v.y, 3)) for v in a.vias}
    laid = {nm(b, s.net_id) for s in b.segments if s.net_id in poured and not getattr(s, 'graphic', False)
            and key(b, s) not in had} | \
        {nm(b, v.net_id) for v in b.vias if v.net_id in poured
         and (nm(b, v.net_id), round(v.x, 3), round(v.y, 3)) not in hadv}
    pro = pro_path_for_board(out)
    new = sorted(laid - set(read_protected_nets(pro)))
    if new:
        persist_protected_nets(pro, {n: 'bus' for n in new}, verbose=False)
        log(f'  protected with the bus nets: {len(new)} poured net(s) the step laid copper of ({", ".join(new)[:160]})')
    return sorted(laid)


# the group's ends, as its buses' runs take them. The CUT round a ball array destination is the ends model's own --
# its far face's middle -- and no variable of the group's model: left to the model to choose with the ends, a
# two-layer bus's search ended a quarter dearer over six seeds (and a hand's far face is cut within 26 degrees of
# its middle on fourteen of sixteen buses read)
GROUP_SET = {'GROUP_LAYERS': 'free', 'GROUP_CUT': 'base'}


def _capture_all(jobs, files, log, src=None):
    """`jobs` (_bus_job's: each bus by itself, ENDS_ONLY, ENDS_CAPTURE) run at the same time, each STOPPED as soon as
    its capture stands (`files` {name: its path}): whole_ends.choose writes a bus's ends model -- its plan state, its
    menus, its run's settings -- before it searches the bus's own ends, and the group's model takes it from there.
    {name: whether its capture stands}. `src`: the buses' source, for their progress events (bus_events)"""
    procs, t0, got = {}, time.time(), {}
    stop = (lambda p: os.killpg(p.pid, signal.SIGTERM)) if hasattr(os, 'killpg') else (lambda p: p.terminate())
    try:
        for name, argv, env, lp in jobs:
            f = open(lp, 'w', encoding='utf-8')
            procs[name] = (subprocess.Popen(argv, cwd=HERE, env=env, stdout=f, stderr=subprocess.STDOUT,
                                            start_new_session=hasattr(os, 'killpg')), f)
        left = set(procs)
        while left:
            for name in sorted(left):
                p = procs[name][0]
                if not os.path.isfile(files[name]) and p.poll() is None:
                    continue
                got[name] = os.path.isfile(files[name])
                left.discard(name)
                log(f'  {name}: its ends model ' + ('captured' if got[name] else f'NOT captured (exit {p.returncode})')
                    + f' after {int(time.time() - t0)} s')
                bus_events.emit('capture', 'end', group=src, bus=f'{src}:{name}', ok=got[name], exit=p.returncode,
                                secs=int(time.time() - t0), log=next(lp for n_, _a, _e, lp in jobs if n_ == name))
            if left:
                time.sleep(POLL)
        return got
    finally:
        for p, f in procs.values():
            if p.poll() is None:
                try:
                    stop(p)
                    p.wait(timeout=30)
                except (OSError, subprocess.TimeoutExpired):
                    p.kill()
            f.close()


def route_grouped(board, out, buses, k=None, rounds=4, inproc=False, log=print, json_out=None, feedback_rounds=0,
                  t0=None, **sizes):
    """The buses off one array routed from THE GROUP'S ENDS (awx/group_ends.py): each bus's ends model
    captured by itself on `board` (work/caps, _capture_all), every bus's ends chosen in ONE model and the arrays
    fanned out once as chosen (work/group, its log work/group.log), then every bus routed negotiated on that field
    with no fanout and no end of its own (route_on_field, GROUP_ENDS), each a plain bus there, one round of its own.
    `rounds` are THE GROUP'S: a round that leaves the board short of clean (a net open, a violation, a ball walled in)
    sends what its buses' steps found at their ends to the model (group_ends --from --feedback: work/groupN), which
    answers them from the state it had, and the buses are routed again on that field (work/roundN.kicad_pcb); the
    board with the fewest nets open, then the fewest violations, then the fewest balls walled in, is `out`. No
    fallback: a bus with no capture, or a first model that ends with no state or no plan, is the step's result,
    named. Returns (exit code, grade lines)"""
    from copy_board import copy_board
    work = out[:-len('.kicad_pcb')] + '.bus'
    caps = os.path.join(work, 'caps')
    shutil.rmtree(caps, ignore_errors=True)
    os.makedirs(caps)
    t0 = time.time() if t0 is None else t0
    name_of = lambda d_: ''.join(c if c.isalnum() or c in '_.-' else '_' for c in d_)     # noqa: E731  (whole_ends.choose's)
    log(f'route_bus: {len(buses)} bus{"" if len(buses) == 1 else "es"} off {buses[0][0]} from the group\'s ends -- '
        + ', '.join(b[1] for b in buses))
    bus_events.emit('capture', 'start', group=buses[0][0], bus=None, buses=[f'{s}:{d}' for s, d in buses])
    got = _capture_all([_bus_job(work, 'capture', b, board, {**_env0(), 'ENDS_ONLY': '1', 'ENDS_CAPTURE': caps},
                                 k, 1, inproc, feedback_rounds, sizes) for b in buses],
                       {b[1]: os.path.join(caps, name_of(b[1]) + '.ends.pkl') for b in buses}, log, src=buses[0][0])
    if not all(got.values()):
        log('route_bus: the group\'s ends: no capture of ' + ', '.join(sorted(n_ for n_, ok_ in got.items() if not ok_))
            + ' -- nothing routed')
        return 1, []
    R = max(1, int(rounds or 1))
    best, done, prev = None, [], None            # prev: (the round before's group, its buses' work)
    for r in range(1, R + 1):
        t_r = time.time()
        group = os.path.join(work, 'group' if r == 1 else f'group{r}')
        shutil.rmtree(group, ignore_errors=True)
        glog = group + '.log'
        bus_events.emit('ends', 'start', group=buses[0][0], bus=None, round=r, rounds=R, log=glog)
        with open(glog, 'w', encoding='utf-8') as f:
            rc = subprocess.run([sys.executable, '-u', '-X', 'utf8', os.path.join(HERE, 'group_ends.py'), caps, group,
                                 '--board=' + board] + ([f'--from={prev[0]}', f'--feedback={prev[1]}'] if prev else []),
                                cwd=HERE, env={**_env0(), **GROUP_SET}, stdout=f, stderr=subprocess.STDOUT).returncode
        for ln in open(glog, encoding='utf-8', errors='replace'):
            if re.match(r'group ends: |  check \d+: |  the field as laid|  \S+: its step\'s findings', ln):
                log('  ' + ln.rstrip()[:300])
        bus_events.emit('ends', 'end', group=buses[0][0], bus=None, round=r, exit=rc, secs=int(time.time() - t_r),
                        log=glog)
        if rc:
            log(f'route_bus: the group\'s ends: the model ended {rc} ({os.path.relpath(glog, work)})'
                + (' -- nothing routed' if r == 1 else f' -- round {best[1]}\'s board stands'))
            if r == 1:
                return 1, []
            break
        log(f'  the group\'s ends in {int(time.time() - t_r)} s' + (f' (round {r})' if R > 1 else ''))
        # (one round alone: straight to `out`; more: each round's board in the work dir, the best one copied out)
        r_out = out if R == 1 else os.path.join(work, f'round{r}.kicad_pcb')
        r_json = json_out if R == 1 else os.path.join(work, f'round{r}.summary.json')
        code, grades = route_on_field(board, os.path.join(group, 'shared_field.kicad_pcb'), r_out, buses, k, 1, inproc,
                                      log, r_json, feedback_rounds, t0=t0, settings={'GROUP_ENDS': group}, **sizes)
        if R == 1:
            return code, grades
        try:
            g_ = json.load(open(r_json)).get('grade') or None
        except (OSError, ValueError):
            g_ = None
        key = (len(g_['open']), g_['drc']['n'] if g_['drc']['n'] is not None else 10 ** 9,
               len(g_['walled']['walled'])) if g_ else (10 ** 9, 10 ** 9, 10 ** 9)
        done.append(dict(round=r, open=key[0], drc=key[1], walled=key[2], secs=int(time.time() - t_r)))
        bus_events.emit('group', 'round', group=buses[0][0], bus=None, **done[-1])
        log(f'  round {r}: ' + (f'{key[0]} open, drc {key[1]}, {key[2]} walled' if g_ else 'no grade')
            + f' in {int(time.time() - t_r)} s')
        if best is None or key < best[0]:
            best = (key, r, r_out, r_json, code, grades)
        if key == (0, 0, 0):
            break
        prev = (group, r_out[:-len('.kicad_pcb')] + '.bus')
    key, r, r_out, r_json, code, grades = best
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(r_out, out)
    try:
        j_ = json.load(open(r_json))
    except (OSError, ValueError):
        j_ = {}
    j_.update(out=out, rounds=done, round=r, secs=int(time.time() - t0))
    with open(json_out or os.path.join(work, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(j_, f, indent=1)
    log(f'route_bus: the group\'s round {r} of {len(done)} routed is the board -- {os.path.basename(out)}')
    return code, grades


def _env0():
    """the environment a bus's process takes: the run's, but none of a meeting's (a JOINT_DIR left over would have a
    solo run wait for buses that never come)"""
    return {k_: v_ for k_, v_ in awx_settings.environ().items()
            if not k_.startswith('JOINT_') and k_ != 'ENDS_ONLY'}


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
                   feedback_rounds=0, t0=None, settings=None, **sizes):
    """The buses `buses` [(src, dest)] routed negotiated on the shared field `field` (the group's model's, from
    `board`), and merged to `out`. `settings`: more for every bus's process (GROUP_ENDS: `field` a group's, every end on it the
    group's model's -- group_ends.py). ONE bus (a group's field for a bus alone on its array): routed by itself on
    the field, nothing to negotiate, graded and summed up the same way. Returns (exit code, grade lines)"""
    from copy_board import copy_board
    board, field, out = os.path.abspath(board), os.path.abspath(field), os.path.abspath(out)
    work = out[:-len('.kicad_pcb')] + '.bus'
    os.makedirs(work, exist_ok=True)
    src, dests = buses[0][0], [b[1] for b in buses]
    t0 = t0 or time.time()
    jd = os.path.join(work, 'negotiation')
    shutil.rmtree(jd, ignore_errors=True)
    os.makedirs(jd)
    one = len(buses) == 1
    phase = 'solo' if one else 'neg'
    log(f'  {buses[0][1]} on the field, by itself' if one else
        f'  every bus on the shared field, negotiated (meeting in {os.path.relpath(jd, os.path.dirname(out))})')
    jobs = [_bus_job(work, phase, b, field,
                     {**_env0(), **({} if one else {'JOINT_DIR': jd, 'JOINT_PEERS': ','.join(dests)}),
                      **(settings or {})},
                     k, rounds, inproc, feedback_rounds, sizes)
            for b in buses]
    bus_events.emit('route', 'start', group=src, bus=None, buses=[f'{src}:{d}' for d in dests], negotiated=not one)
    codes = _run_all(jobs, log, ended=lambda d: open(os.path.join(jd, f'{d}.finished'), 'w').close(), src=src)

    # 4. merged
    boards = [_bus_out(work, phase, b) for b in buses]
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
        protect_laid(board, out, log=log)
        bus_events.emit('merge', 'end', group=src, bus=None, conflicts=n_conf, given_back=given)
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
        bus_events.emit('grade', 'end', group=src, bus=None, open=len(grade_['open']), drc=grade_['drc']['n'],
                        walled=len(grade_['walled']['walled']))
    runs = []
    for b, x in zip(buses, boards):
        j = json.load(open(x[1])) if x else {}
        # (the step's own status is its summary's: the process exits 0 whenever it wrote OUT, #1738)
        ex = j.get('exit', codes.get(b[1])) if x else codes.get(b[1])
        runs.append(dict(src=b[0], dest=b[1], exit=ex, routed=len(j.get('routed') or []),
                         grade=_grade_line(os.path.join(work, phase, b[1], 'bus.log')), summary=x[1] if x else None))
    code = max([1 if r['exit'] is None else r['exit'] for r in runs] + [0 if n_conf == 0 else 1])
    with open(json_out or os.path.join(work, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(board=board, out=out, negotiated=True, src=src, field=field,
                       buses=runs, conflicts=n_conf, given_back=given, grade=grade_, secs=int(time.time() - t0),
                       exit=code), f, indent=1)
    log(f'route_bus: {"on the field" if one else "negotiated"}, {sum(r["routed"] for r in runs)} nets routed by '
        f'{len(runs)} bus{"" if one else "es"}, '
        f'{"no" if n_conf == 0 else n_conf if n_conf is not None else "ungraded"} cross-bus conflict(s), '
        f'{int(time.time() - t0)} s -- {os.path.basename(out)}')
    return code, [r['grade'] for r in runs]
