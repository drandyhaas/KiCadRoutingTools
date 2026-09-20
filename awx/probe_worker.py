#!/usr/bin/env python3
"""probe_worker.py -- a RESIDENT probe process (#622, 2026-09-18).

A descent's probe used to be three processes a time: the fanout
re-laid in the descent's own process, then `braid.py` spawned to route
the coupled set (interpreter, imports, solver priming, the taut memo's
load: ~1 s of a ~4 s braid), then the grade. This process stays up for
a whole descent: it holds the round's Board (the fanout board, the
menus, both ends measured -- 1.7 s to build at K51), applies the same
advances the parent applies, and runs `replan.probe_run` for each probe
the parent hands it, with the braid called in-process (`braid.run`).
Several of these side by side are the parallel menu: the candidates of
one net are independent (the parent advances only after the net's
menu is judged), so `replan --par=N` probes N of them at once.

Protocol (JSON lines on stdin / stdout; one reply per request):
  {"op": "round", ...}     the round's state: F, R, names, dref, banned,
                           blockers, fan_src, K, base, nets_csv, options
  {"op": "advance", ...}   a standing probe applied: net, its result, R_cur, fan_src
  {"op": "probe", ...}     one probe: R, net, src, dst (moves as JSON), tag, extra_relay
  {"op": "ping"} / {"op": "quit"}
A reply is {"ok": true, ...} or {"ok": false, "error": "..."}; the
worker's own stderr goes where the parent points it.
"""
import contextlib
import io
import json
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import rules as _rules  # noqa: E402
_rules.install_defaults()
_ARGV, sys.argv = sys.argv, sys.argv[:1]     # replan reads its options off argv at import
import replan  # noqa: E402
sys.argv = _ARGV
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
import plan_ends as pe  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402,F401


# ------------------------------------------------------------ moves <-> JSON
move_to_json = replan.move_to_json
move_from_json = replan.move_from_json


def _deep_tuple(x):
    if isinstance(x, list):
        return tuple(_deep_tuple(v) for v in x)
    return x


def _peak_mb():
    import resource
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1048576


def _res_json(res):
    return {k: v for k, v in res.items() if k not in ('src', 'dst')}


# ------------------------------------------------------------------ worker
class Worker:
    def __init__(self):
        self.B = None
        self.names = None
        self.K = self.base = self.nets_csv = None
        self.fan_src = {}
        self.F = None
        self.n_probes = 0

    # the in-process braid: what replan.braid_run does through a subprocess
    def braid_inproc(self, board, out_stem, nets, dref, log_to, probe=False):
        t0 = time.time()
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                te.run(board, nets, dref, out_stem)
        except Exception:                                  # noqa: BLE001
            buf.write('\nbraid: EXCEPTION in-process\n' + traceback.format_exc())
        with open(log_to, 'w') as f:
            f.write(buf.getvalue())
        return os.path.exists(out_stem + '.kicad_pcb'), time.time() - t0

    def op_round(self, q):
        # the parent's options, module state here
        replan.COUPLED = q['coupled']
        replan.WIDEN = int(q['widen'])
        replan.GRADE_MODE = q['grade']
        replan.APPLY_STRIP = bool(q['apply_strip'])
        replan.MEMO_K = None                    # the memo is the parent's
        te.ATTEMPTS = int(replan.PROBE_ATTEMPTS)
        te.BUDGET_X = int(replan.PROBE_BUDGET_X)
        os.environ['BRAID_SMOOTH'] = os.environ.get('PROBE_SMOOTH', '0')
        te.LADDER_MODE = replan.PROBE_LADDER
        replan.braid_run = self.braid_inproc
        self.K, self.base, self.nets_csv = int(q['K']), q['base'], q['nets_csv']
        self.names = list(q['names'])
        self.F = q['F']
        banned = frozenset(_deep_tuple(x) for x in q.get('banned', []))
        self.B = replan.Board(q['F'], self.names, q['dref'], banned=banned, R=q['R'])
        self.B.blockers = q.get('blockers', {})
        self.B.swimmers = set(q.get('swimmers', []))
        pe.RESIDUAL = q.get('resid', {})
        self.fan_src = dict(q.get('fan_src', {}))
        if replan.APPLY_STRIP:
            replan.FAN_PCB = lambda c, _F=self.F: replan.parsed(self.fan_src.get(c, _F))
        else:
            replan.FAN_PCB = None
        return {'ok': True, 'menus': sum(len(replan.dmenu_full(self.B.st)[n]) for n in self.names)}

    def op_advance(self, q):
        B = self.B
        R_cur = q['R_cur']
        B.lanes = replan.lane_items(replan.parsed(R_cur), B.pcb, self.names, B.byname)
        pr = dict(q['pr'])
        for k in ('src_got', 'dst_got'):
            pr[k] = replan._got_tuples(pr.get(k))
        if pr.get('comove_got'):
            pr['comove_got'] = {o: replan._got_tuples(g) for o, g in pr['comove_got'].items()}
        B.advance(q['net'], pr)
        self.fan_src = dict(q.get('fan_src', self.fan_src))
        return {'ok': True}

    def op_probe(self, q):
        src = replan.move_from_json(q.get('src'))
        dst = replan.move_from_json(q.get('dst'))
        t0 = time.time()
        res = replan.probe_run(self.B, q['R'], q['net'], src, dst, q['tag'], self.K, self.base,
                               self.nets_csv, log=lambda *a: None, extra_relay=q.get('extra_relay'),
                               ref=q.get('ref'))
        self.n_probes += 1
        out = _res_json(res)
        out['worker_s'] = time.time() - t0
        return {'ok': True, 'res': out, 'peak_mb': _peak_mb()}

    def op_screen(self, q):
        m = replan.move_from_json(q['move'])
        others = {o: replan.move_from_json(om) for o, om in (q.get('others') or {}).items()}
        got, exact, in_cls, dt = replan.engine_lays_run(self.B, q['net'], m, q['end'], others or None)
        return {'ok': True, 'got': got, 'exact': bool(exact), 'in_cls': bool(in_cls), 'seconds': dt}

    def serve(self):
        # the protocol owns stdout; every stray print (an import's, an
        # engine's) goes to stderr, which the parent points at a file
        proto, sys.stdout = sys.stdout, sys.stderr
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                q = json.loads(line)
                op = q.get('op')
                if op == 'quit':
                    print(json.dumps({'ok': True}), file=proto, flush=True)
                    return 0
                if op == 'ping':
                    resp = {'ok': True, 'pid': os.getpid(), 'probes': self.n_probes}
                elif op == 'round':
                    resp = self.op_round(q)
                elif op == 'advance':
                    resp = self.op_advance(q)
                elif op == 'probe':
                    resp = self.op_probe(q)
                elif op == 'screen':
                    resp = self.op_screen(q)
                else:
                    resp = {'ok': False, 'error': f'unknown op {op!r}'}
            except Exception as e:                         # noqa: BLE001
                resp = {'ok': False, 'error': f'{type(e).__name__}: {str(e)[:300]}',
                        'trace': traceback.format_exc()[-2000:]}
            print(json.dumps(resp, default=str), file=proto, flush=True)
        return 0


def _self_test():
    m = em.Move('N', 'dogbone', 'left', 'B.Cu', (1.0, 2.0), 1, [((0.0, 0.0), (1.0, 2.0), 'F.Cu')],
                site=(0.5, 0.5), climb=2)
    m.comove = ['A', 'B']
    d = json.loads(json.dumps(move_to_json(m)))
    m2 = move_from_json(d)
    assert m2 == m and m2.comove == ['A', 'B'] and isinstance(m2.exit_pt, tuple) and isinstance(m2.legs[0][0], tuple)
    import source_realize as sr
    assert sr.move_sig(m2) == sr.move_sig(m), 'a move survives the wire with its signature'
    print('probe_worker: self-test ok (a move round-trips through JSON)')


if __name__ == '__main__':
    if '--self-test' in _ARGV:
        _self_test()
        sys.exit(0)
    sys.exit(Worker().serve())
