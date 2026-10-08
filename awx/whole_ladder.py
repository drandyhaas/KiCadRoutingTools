#!/usr/bin/env python3
"""whole_ladder.py OUT [--bench h3|zynq ...] [--ks K,K,...] [--jobs N] [--cap SECS] [--rounds R] [--set K=V ...] --
the whole route's ladder on this machine: every rung of each bench through whole_route.py, JOBS at a time (the
largest K first), each stopped at CAP seconds with every stage it started. One line per rung as it ends:

  BENCH K<k> exit RC SECS s: WHOLE K=.. round=.. lanes=../.. vias=.. copper=..mm connected=0|1 drc=0|1 secs=.. open=..

also in OUT/ladder.txt; each rung's log and OUTDIR beside it (OUT/BENCH_k<K>.log, OUT/BENCH_k<K>/). Exits 0 when every
rung ends connected and DRC-clean, 1 otherwise.

The benches: `h3` is fb_t2q_pairs (BASE's default, DEST DU1), rungs 15,28,35,41,51; `zynq` is the zynq article in its
flow frame (tmp/zynq/zynqF.kicad_pcb, DEST U2: build it first, README "Building the zynq article"), rungs
18,26,32,38,42,44. `--ks` replaces a bench's rungs; `--set K=V` is a setting for every rung (repeatable). The cloud's
ladder, one container per rung, is modal_whole.py.

A cap stops the rung's whole process group -- the driver and every stage under it -- so nothing is left running past it
(a cap that killed the driver alone left its solve running for half an hour).
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCHES = {
    'h3': ({}, [15, 28, 35, 41, 51]),
    'zynq': ({'BASE': 'tmp/zynq/zynqF.kicad_pcb', 'DEST': 'U2'}, [18, 26, 32, 38, 42, 44]),
}
POLL = 2.0                  # seconds between looks at the running rungs


def _start(cmd, env, log):
    """a rung in a process group of its own, so a cap can stop it whole"""
    kw = dict(cwd=HERE, env=env, stdout=log, stderr=subprocess.STDOUT)
    if os.name == 'nt':
        kw['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kw['start_new_session'] = True
    return subprocess.Popen(cmd, **kw)


def _stop(p):
    """the rung's process and everything it started"""
    if os.name == 'nt':
        subprocess.run(['taskkill', '/T', '/F', '/PID', str(p.pid)], capture_output=True)
    else:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    p.wait()


def grade(log_path):
    """the last WHOLE line of a rung's log, or ''"""
    lines = [ln.strip() for ln in open(log_path, errors='replace') if ln.startswith('WHOLE ')]
    return lines[-1] if lines else ''


def passed(line):
    return bool(re.search(r'connected=1 drc=1', line))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('out')
    ap.add_argument('--bench', action='append', choices=sorted(BENCHES), help='a bench (repeatable; default both)')
    ap.add_argument('--ks', default='', help='the rungs, comma separated (default: each bench\'s ladder)')
    ap.add_argument('--jobs', type=int, default=3)
    ap.add_argument('--cap', type=int, default=3600, help='seconds a rung may run (default 1 h)')
    ap.add_argument('--rounds', type=int, default=3)
    ap.add_argument('--set', action='append', default=[], metavar='K=V', help='a setting for every rung')
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    extra = dict(kv.partition('=')[::2] for kv in a.set)
    todo = []
    for b in a.bench or sorted(BENCHES):
        benv, ks = BENCHES[b]
        if a.ks:
            ks = [int(k) for k in a.ks.split(',') if k]
        todo += [(b, k, benv) for k in ks]
    todo.sort(key=lambda t: -t[1])              # the longest first, so the last to start are the short ones
    running, ok, summary = [], True, open(os.path.join(out, 'ladder.txt'), 'w')
    while todo or running:
        while todo and len(running) < a.jobs:
            b, k, benv = todo.pop(0)
            name = f'{b}_k{k}'
            env = dict(os.environ, **benv, **extra)
            log = open(os.path.join(out, name + '.log'), 'w')
            p = _start([sys.executable, 'whole_route.py', str(k), os.path.join(out, name), str(a.rounds)], env, log)
            running.append((b, k, name, p, log, time.time()))
        time.sleep(POLL)
        for item in list(running):
            b, k, name, p, log, t0 = item
            capped = p.poll() is None and time.time() - t0 > a.cap
            if capped:
                _stop(p)
            if p.poll() is None:
                continue
            running.remove(item)
            log.close()
            g = grade(os.path.join(out, name + '.log'))
            line = (f'{b} K{k} exit {p.returncode} {time.time() - t0:.0f} s: '
                    + (g or '(no grade)') + (f' (stopped at the {a.cap} s cap)' if capped else ''))
            ok = ok and passed(g)
            print(line, flush=True)
            summary.write(line + '\n')
            summary.flush()
    summary.close()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
