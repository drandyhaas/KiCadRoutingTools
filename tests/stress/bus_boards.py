#!/usr/bin/env python3
"""bus_boards.py -- the BUS BOARDS check: each board of tests/fixtures/bus_boards routed by the bus step from its pre-bus
input (route_bus.py, the board's recorded arguments: every bus of the board in one call -- routed in turn, or
negotiated where the buses off one array need it, as the step decides), and every bus net CONNECTED on the board the
step hands on (check_connected, read off that board -- not the step's own count).

  python3 tests/stress/bus_boards.py [--cases A,B] [--out DIR] [-j N] [--rounds R] [--set K=V ...]

Not collected by tests/run_all.py: a run is an hour or more of routing (the boards run -j at a time, each a full
route_bus call). Exit 0 when every board's bus nets are all connected, 1 when any is open or a run failed; the table
names each board's open nets, its buses (mode, vias, copper, walled balls) and its time, and OUT/report.json holds it.

  --set K=V   a setting for every run (e.g. ENDS_JOINT_ALL=1): an A/B arm of a change behind a setting
  --rounds R  route_bus --rounds (default 1: a bus closes in its first round)
"""
import argparse
import concurrent.futures
import contextlib
import gzip
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
FIX = os.path.join(ROOT, 'tests', 'fixtures', 'bus_boards')


def stage(case, spec, out):
    """the case's input on disk in its own directory: the board unpacked, its project and rules files beside it"""
    d = os.path.join(out, case)
    os.makedirs(d, exist_ok=True)
    src = os.path.join(FIX, spec['board'])
    dst = os.path.join(d, 'input.kicad_pcb')
    with gzip.open(src, 'rb') as fi, open(dst, 'wb') as fo:
        shutil.copyfileobj(fi, fo)
    stem = spec['board'][:-len('.kicad_pcb.gz')]
    for ext in ('.kicad_pro', '.kicad_dru'):
        if os.path.isfile(os.path.join(FIX, stem + ext)):
            shutil.copy2(os.path.join(FIX, stem + ext), os.path.join(d, 'input' + ext))
    return d, dst


def route(case, spec, out, rounds, sets, timeout):
    """route_bus on the case: (its directory, exit code, seconds)"""
    d, board = stage(case, spec, out)
    cmd = [sys.executable, '-u', '-X', 'utf8', os.path.join(ROOT, 'awx', 'route_bus.py'), board,
           os.path.join(d, 'bus.kicad_pcb'), '--inproc', '--rounds', str(rounds)] + list(spec['args'])
    env = dict(os.environ, **sets)
    t0 = time.time()
    with open(os.path.join(d, 'run.log'), 'w', encoding='utf-8') as log:
        log.write(' '.join(cmd) + '\n' + ''.join(f'{k}={v}\n' for k, v in sorted(sets.items())))
        log.flush()
        try:
            rc = subprocess.run(cmd, cwd=d, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            rc = 'timeout'
    return d, rc, round(time.time() - t0)


def bus_summaries(d):
    """[(the bus's summary path, its entry)] of the run in `d`: the buses a multi-bus call names in its summary, else
    the one bus's own"""
    top = os.path.join(d, 'bus.bus', 'summary.json')
    if os.path.isfile(top):
        return [(b.get('summary'), b) for b in json.load(open(top, encoding='utf-8')).get('buses', [])]
    one = [f for f in sorted(os.listdir(os.path.join(d, 'bus.bus'))) if f.endswith('.summary.json')
           and '.alone.' not in f] if os.path.isdir(os.path.join(d, 'bus.bus')) else []
    return [(os.path.join(d, 'bus.bus', f), {}) for f in one]


def grade(d):
    """{buses: [...], nets, open: [names], why}: every bus net of the run in `d` checked on its output board -- its
    pads on the bus's own parts (the source, the destination's every part) one piece of copper (check_connected's
    test, zones credited). A bus net's pads elsewhere (a termination's, a test point's) are the chain's later steps'"""
    return grade_board(os.path.join(d, 'bus.kicad_pcb'), bus_summaries(d))


def grade_board(out_, summaries):
    """grade's test on the board `out_` for the buses `summaries` [(a bus's summary path, its entry)]"""
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    sys.path.insert(0, os.path.join(ROOT, 'awx'))
    import collections
    import check_connected as cc
    from coherent_nets import by_key
    from kicad_parser import parse_kicad_pcb
    if not os.path.isfile(out_):
        return dict(buses=[], nets=0, open=[], why='no output board')
    buses, asked = [], []
    for path, entry in summaries:
        if not path or not os.path.isfile(path):
            buses.append(dict(bus=f"{entry.get('src')}->{entry.get('dest')}", why='no summary'))
            continue
        s = json.load(open(path, encoding='utf-8'))
        nets = list(s.get('routed') or []) + list(s.get('refused') or {})
        refs = {s.get('src')} | set(str(s.get('dest') or '').split('+'))
        asked += [(k, refs) for k in nets]
        g = s.get('grade') or entry.get('grade') or ''
        buses.append(dict(bus=f"{s.get('src')}->{s.get('dest')}", mode=entry.get('mode'), nets=len(nets),
                          exit=s.get('exit'), walled=len((s.get('walled') or {}).get('walled') or []),
                          grade=g.split(' | ')[0]))
    if not asked:
        return dict(buses=buses, nets=0, open=[], why='no bus nets in the summaries')
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(out_)
    bk = by_key(pcb)
    segs, vias, zones = (collections.defaultdict(list) for _ in range(3))
    for s_ in pcb.segments:
        segs[s_.net_id].append(s_)
    for v_ in pcb.vias:
        vias[v_.net_id].append(v_)
    for z_ in pcb.zones or ():
        zones[z_.net_id].append(z_)
    bad = []
    for k, refs in asked:
        if k not in bk:
            bad.append(k)
            continue
        nid, net = bk[k]
        pads = [p for p in net.pads if p.component_ref in refs]
        if len(pads) < 2:
            continue                                    # (nothing of the bus's to join)
        with contextlib.redirect_stdout(io.StringIO()):
            r = cc.check_net_connectivity(nid, segs[nid], vias[nid], pads, zones[nid], pcb_data=pcb)
        if not r.get('connected'):
            bad.append(net.name)
    return dict(buses=buses, nets=len(asked), open=sorted(set(bad)), why='')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--cases', default=None, help='the cases to run, comma separated (default: all)')
    ap.add_argument('--out', default=None, help='the runs\' directory (default: a new temporary one, kept)')
    ap.add_argument('-j', type=int, default=3, help='boards routed at once (default 3)')
    ap.add_argument('--rounds', type=int, default=1, help='route_bus --rounds (default 1)')
    ap.add_argument('--set', action='append', default=[], metavar='K=V', help='a setting for every run')
    ap.add_argument('--timeout', type=float, default=4 * 3600, help='seconds a board may take (default 4 h)')
    a = ap.parse_args()
    man = json.load(open(os.path.join(FIX, 'manifest.json'), encoding='utf-8'))['cases']
    cases = [c for c in (a.cases.split(',') if a.cases else man) if c]
    unknown = [c for c in cases if c not in man]
    if unknown:
        sys.exit(f'bus_boards: no such case(s): {", ".join(unknown)} (the manifest has {", ".join(man)})')
    sets = dict(kv.partition('=')[::2] for kv in a.set)
    out = os.path.abspath(a.out) if a.out else tempfile.mkdtemp(prefix='bus_boards_')
    os.makedirs(out, exist_ok=True)
    print(f'bus_boards: {len(cases)} board(s) into {out}, {a.j} at once, --rounds {a.rounds}'
          + (f', {" ".join(f"{k}={v}" for k, v in sorted(sets.items()))}' if sets else ''), flush=True)
    rows = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, a.j)) as ex:
        futs = {ex.submit(route, c, man[c], out, a.rounds, sets, a.timeout): c for c in cases}
        for f in concurrent.futures.as_completed(futs):
            c = futs[f]
            d, rc, secs = f.result()
            g = grade(d)
            rows[c] = dict(exit=rc, secs=secs, **g)
            print(f'  {c}: route_bus exit {rc}, {secs} s -- {g["nets"]} bus net(s), {len(g["open"])} open'
                  + (f' ({", ".join(g["open"][:8])}{" ..." if len(g["open"]) > 8 else ""})' if g['open'] else '')
                  + (f' [{g["why"]}]' if g['why'] else ''), flush=True)
    print('\nboard      bus nets  open  secs  buses')
    ok = True
    for c in cases:
        r = rows[c]
        fail = bool(r['open']) or bool(r['why']) or r['exit'] not in (0, 1)
        ok &= not fail
        print(f'{c:10s} {r["nets"]:8d} {len(r["open"]):5d} {r["secs"]:5d}  '
              + '; '.join(f'{b["bus"]} {b.get("mode") or ""} {b.get("grade", b.get("why", ""))[4:] if b.get("grade") else b.get("why", "")}'
                          f'{" walled=" + str(b["walled"]) if b.get("walled") else ""}' for b in r['buses'])
              + ('  FAIL' if fail else ''))
    with open(os.path.join(out, 'report.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(sets=sets, rounds=a.rounds, cases=rows), f, indent=1)
    print(f'\nbus_boards: {"every bus net connected on every board" if ok else "OPEN bus nets or failed runs"} '
          f'-- {os.path.join(out, "report.json")}')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
