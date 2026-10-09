#!/usr/bin/env python3
"""whole_route.py K OUTDIR [ROUNDS] -- the whole route on rung K, end to end, in Python.
   whole_route.py --loop SOLVE.json OUTDIR [ROUNDS] -- the loop alone on a solve (the bench from BENCH / NETS / DEST).

The same chain as whole_chain.sh and whole_loop.sh, and the same stages run the same way: each stage is its own
process, given the environment and the arguments the shell scripts give it, in the same order, on the same branches.
What changes is only the driver, which no longer needs zsh.

The chain (whole_chain.sh): the fanout choosing every net's tooth and berth with the whole route's ends model
(fanout_from_plan, PLAN_JUDGE=ends), the solve (whole_solve), the loop (below) and the route all at once (route_lanes),
graded (check_connected, check_drc). When the loop does not pass, the ends its audits found crowded (whole_gate --hot ->
whole_feedback) go back to the fanout (FEEDBACK=), which chooses again INCREMENTALLY from the previous round's board;
up to ROUNDS fanouts (default 3). BASE (default fb_t2q_pairs.kicad_pcb) is the bench, DEST (default DU1) its
destination part, both from the environment as the shell script reads them.

The loop (whole_loop.sh), every step fed by a MEASUREMENT of the one before: geometry (whole_geo), polish
(whole_polish), audit (whole_audit through whole_gate); side flips the polish could not avoid go to the geometry again
on the same solve, or with cuts and findings to the solve again; a smooth plan that passes has its pairs laid first
(whole_snap --pairs), the singles fitted round them (whole_polish; SNAP_KEEP where a single is short) and snapped
(whole_snap), audited, gated and linted. A loop that is NOT CONVERGING stops (exit 3), and so does one whose findings
stand at the ENDS (exit 4, the fanout's to change). Each stage goes through stage_cache.py, on by default here as in
whole_loop.sh (STAGE_CACHE=0 runs every stage).

The chain never ends with nothing while a plan for some of the lanes can be laid: a round's solve keeps a plan it cannot
prove (laid all the same; the nets it leaves over two vias go back to the ends, their own ends freed next round), a
loop that does not pass is laid from the plan it held (held_plan; a lane it cannot lay stays open), a board the
fanout's audit refuses is fed back (its split pairs, its teeth on the source's far face), and a round that lays
NOTHING -- no plan from its solve, none its loop held -- or leaves nets OPEN, with nothing new from its audits, sends
its open nets, else the lanes the ends model names, back to the fanout (whole_feedback --name), all for another round,
incremental. The loop's last re-solve keeps a plan it cannot prove. When no round laid anything, the LAST RESORT is a
partial plan on the last ends laid, the named lanes (then more) left out and open. The run's result is its best --
the fewest open nets, then the fewest vias -- in OUTDIR/best.kicad_pcb (and its own rN/ or partialN/seq.kicad_pcb).
The last line is that result's grade:
  WHOLE K=.. round=.. lanes=../.. vias=.. copper=..mm connected=0|1 drc=0|1 secs=.. open=..
Exits 0 when it is connected and DRC-clean, 1 when nets are left open or a violation stands, 2 when nothing could be
laid at all (no fanout board, or no plan even for half the lanes).
--loop exits 0 with OUTDIR/plan.json the snapped plan that passed, 1 when no round got there, 3 when it stopped not
converging, 4 when it stopped at crowded ends.
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import atexit
import collections
import contextlib
import glob
import io
import json
import math
import os
import re
import runpy
import shutil
import subprocess
import sys
import tempfile
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import awx_settings  # noqa: E402
PYR = os.path.normpath(os.path.join(HERE, '..', 'py_router'))
PY = sys.executable
PATIENCE = 2                               # the loop's rounds in a row without a new best
INHERIT = object()                         # run(to=INHERIT): the command's output where the driver's own goes


# =============================================================================== running a stage
class Log:
    """where the driver's own lines go: the console for the chain, the round's loop.log for the loop"""

    def __init__(self, f=None):
        self.f = f

    def __call__(self, s=''):
        if self.f is None:
            print(s, flush=True)
        else:
            self.f.write(s + '\n')
            self.f.flush()


FANOUT_RAISES = 3   # a fanout laying the round before's ends again: its feedback raised, the fanout again
INPROC = False      # --inproc: every stage in this process, as a routing call inside KiCad's own process will run them


@contextlib.contextmanager
def _fds(out_f, err_f):
    """file descriptors 1 and 2 on these open files for the block -- what a child's `> log 2>&1` gives it, the
    compiled code's own writes (the router, the solvers) included"""
    sys.stdout.flush(); sys.stderr.flush()
    saved = os.dup(1), os.dup(2)
    try:
        os.dup2(out_f.fileno(), 1)
        os.dup2(err_f.fileno(), 2)
        yield
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        os.dup2(saved[0], 1); os.dup2(saved[1], 2)
        os.close(saved[0]); os.close(saved[1])


def _stage_exit():
    """what a stage's process does on its way out, done at the end of a stage run in this one: the taut memo's last
    dirty shards saved (detect_buses registers it with atexit). Its REPORT is not printed, here or at exit: it counts
    what this process holds, every stage's shards so far, where a stage's own process holds only its own -- printed
    into a stage's output it became the last line of a gate's or a lint's, whose verdict is read off that line"""
    db = sys.modules.get('detect_buses')
    if db is not None:
        db._memo_save(force=True)
        atexit.unregister(db._memo_report)


def _inproc(argv, env, out_f, err_f):
    """`python3 ARGV...` run in this process as its own would run it: ENV the settings every awx module reads
    (awx_settings.given: the whole of them, as an environment is the whole of a child's -- os.environ is neither read
    by them nor written), the arguments, awx/ its directory, fds 1 and 2 on OUT_F / ERR_F (None: where the driver's
    go); the exit code"""
    # (sys.path is not restored: a module imported by an earlier stage put its paths there once, and a later stage's
    # lazy imports -- py_router's design_rules, say -- find their modules through them)
    saved = list(sys.argv), os.getcwd()
    os.chdir(HERE)
    sys.argv = list(argv)
    sys.path.insert(0, os.path.dirname(os.path.abspath(argv[0])))
    rc = 0
    with awx_settings.given(env), (_fds(out_f, err_f) if out_f is not None else contextlib.nullcontext()):
        try:
            runpy.run_path(argv[0], run_name='__main__')
        except SystemExit as e:
            if isinstance(e.code, str):         # sys.exit('why'): the interpreter prints it and exits 1
                print(e.code, file=sys.stderr)
            rc = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
        except BaseException:
            traceback.print_exc()
            rc = 1
        finally:
            try:
                _stage_exit()
            finally:
                sys.stdout.flush(); sys.stderr.flush()
    sys.argv = saved[0]
    os.chdir(saved[1])
    return rc


def run(argv, env, log_path=None, err=None, to=None, own_process=False):
    """`python3 ARGV...` in awx/ as the shell runs it: stdout and stderr to LOG_PATH (`> log 2>&1`), or both to the
    open file TO (a command inside the loop left unredirected: its lines land in the loop's log), or stdout captured
    and stderr to ERR (the loop's own log, as `$(...)` inside it leaves it); (exit code, stdout). In a process of its
    own, or with INPROC in this one (OWN_PROCESS: in its own whatever INPROC says)."""
    if INPROC and not own_process:
        if log_path is not None:
            with open(log_path, 'w') as f:
                return _inproc(argv, env, f, f), ''
        if to is INHERIT:
            return _inproc(argv, env, None, None), ''
        if to is not None:
            return _inproc(argv, env, to, to), ''
        with tempfile.TemporaryFile('w+') as cap, open(os.devnull, 'w') as dn:
            rc = _inproc(argv, env, cap, err or dn)
            cap.seek(0)
            return rc, cap.read()
    if log_path is not None:
        with open(log_path, 'w') as f:
            return subprocess.run([PY] + argv, cwd=HERE, env=env, stdout=f, stderr=subprocess.STDOUT).returncode, ''
    if to is INHERIT:
        return subprocess.run([PY] + argv, cwd=HERE, env=env).returncode, ''
    if to is not None:
        return subprocess.run([PY] + argv, cwd=HERE, env=env, stdout=to, stderr=to).returncode, ''
    p = subprocess.run([PY] + argv, cwd=HERE, env=env, stdout=subprocess.PIPE, stderr=err or subprocess.DEVNULL,
                       text=True)
    return p.returncode, p.stdout


def lines_of(path):
    try:
        with open(path) as f:
            return f.read().splitlines()
    except OSError:
        return []


def grep(path, pattern):
    """the lines of PATH matching the extended regex PATTERN"""
    r = re.compile(pattern)
    return [ln for ln in lines_of(path) if r.search(ln)]


def source_board_named(fo_log):
    """the source board a fanout's log names last -- its destination pass's '(source board: NAME, ...' line, on either
    path (fanout_from_plan.fanout_destination, joint_destination) -- or None"""
    sb = [m for ln in lines_of(fo_log) for m in re.findall(r'source board: ([^,]+)', ln)]
    return sb[-1] if sb else None


def tail(path, n):
    return lines_of(path)[-n:]


def prefixed(text, prefix):
    """`echo "$text" | sed 's/^/PREFIX/'`: every line prefixed, an empty text one prefixed empty line"""
    return '\n'.join(prefix + ln for ln in (text.split('\n') if text else ['']))


def stripped(s):
    """`$(...)`: the output less its trailing newlines"""
    return s.rstrip('\n')


# =============================================================================== the loop (whole_loop.sh)
def nflips(files):
    """the side flips in a comma list of polish outputs, every file's"""
    return len({tuple(x) for f in files.split(',') if f for x in json.load(open(f)).get('flips', [])})


def findings(gate_line):
    """the findings in a gate line (whole_gate's summary): dive, static, shape, swim, pitch in the plan, band outside
    (0/1); a gate line without its counts (an audit that did not run to its end) counts as many"""
    s = gate_line
    try:
        n = sum(int(re.search(k + r' (\d+)', s).group(1)) for k in ('dive', 'static', 'shape', 'swim'))
        n += int(re.search(r'pitch (\d+) in the plan', s).group(1))
        n += int(re.search(r'band broken (\d+)', s).group(1)) if 'band broken' in s else 0
        n += int(re.search(r'(\d+) lane\(s\) missing', s).group(1)) if 'missing' in s else 0     # a lane a snap could not lay
        return n + (1 if float(re.search(r'band ([\d.]+) mm', s).group(1)) > 0 else 0)
    except AttributeError:
        return 999


class NotConverging(Exception):
    pass


# a lane the snap could not lay is held under an island on one layer when its search stuck within this much of it, mm
SNAP_ISLAND_REACH = 2.0
# a lane the audit found against an array's own pad is held off that layer this far round the place at least, mm
ARRAY_CUT_R = 0.5


def _layer_index(tag):
    """a routing layer's index from its tag in a finding: F or B (the audits' letter), else the layer's name
    (route_layers: F.Cu 0, B.Cu 1, the inner ones after)"""
    if tag in ('F', 'F.Cu'):
        return 0
    if tag in ('B', 'B.Cu'):
        return 1
    import route_layers
    return route_layers.index(tag)


def layer_cut(lane, island, box, layer):
    """the loop's LAYER cut: `lane` held off the island's layers across `island` (its box, whole_geo's island_boxes;
    `blocked` the island's layers, `layer` -- on two -- the one the lane is held on) -- none where the island stands on
    every routing layer, which no change answers (on two: where it stands on both)"""
    import route_layers
    if not box or layer not in box[4] or len(box[4]) >= len(route_layers.layers()):
        return None
    c = {'lane': lane, 'island': island, 'layer': 1 - layer, 'box': list(box[:4])}
    if len(route_layers.layers()) > 2:
        c['blocked'] = sorted(box[4])
    return c


def audit_layer_cuts(audit, geo):
    """the LAYER cuts an audit's STATIC findings give: a lane found against a pad of an island (a geometry's 'islands',
    'REF.PAD' -> island) on one layer, the layer it was found on -- and, on more routing layers than two, a lane found
    against an ARRAY's own pad (no island), off the layer it was found on round that place (ARRAY_CUT_R): it was pushed
    into the pad by the lanes beside it on its layer -- a ring's lanes crowding the channel beside the destination
    (synth wind_rot16_g3_vias: nine lanes up a 2 mm channel on one layer, the innermost into the destination's corner
    ball every round) -- and on another layer it has room. A drilled ball stands on every layer, so this is no island's
    cut, which no change answers"""
    import route_layers
    nl = len(route_layers.layers())
    out, isl, bxs = [], geo.get('islands') or {}, geo.get('island_boxes') or {}
    for ln in lines_of(audit):
        m = re.match(r'STATIC (\S+)\s+([FB]|In\d+\.Cu) ([-+]?[\d.]+)/([\d.]+) (?:pad|hole) (\S+) ', ln)
        if not m:
            continue
        lane, L, bar, pad = m.group(1), _layer_index(m.group(2)), float(m.group(4)), m.group(5)
        if pad in isl:
            c = layer_cut(lane, isl[pad], bxs.get(isl[pad]), L)
        elif nl > 2 and (at := re.search(r' at \(([-\d.]+),\s*([-\d.]+)\)', ln)):
            x, y, r = float(at.group(1)), float(at.group(2)), max(ARRAY_CUT_R, 3 * bar)
            c = {'lane': lane, 'island': pad, 'layer': 0, 'box': [x - r, y - r, x + r, y + r], 'blocked': [L]}
        else:
            c = None
        if c and c not in out:
            out.append(c)
    return out


def _dense(pts, step):
    """a polyline's points `pts` ((u,) x, y rows) with points added every `step` mm of board between them, the leading
    values interpolated too"""
    out = []
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[-2] - a[-2], b[-1] - a[-1])
        m = max(1, int(math.ceil(d / step)))
        out += [[a[j] + (b[j] - a[j]) * i / m for j in range(len(a))] for i in range(m)]
    return out + [list(pts[-1])] if pts else out


def own_spans(cf, geo, plan=None):
    """(more routing layers than two) the cuts of a cut file `cf` held to the stretch of each lane's OWN route that met
    what cut it, read off the geometry's map of its laid lanes (`geo`'s uxy: each lane's columns as route u, x, y). A
    cut is written in route u, and a place in u is a line across the frame: where lanes run steeply across the trunk --
    the zynq's LVDS bus from U1's east face to U5's north ring, the trunk from centre to centre -- a few mm of the lane
    advance it a fraction of that, and a cut over the island's whole box or a via's whole room in u lands on stretches of
    the lane millimetres from the place (a layer cut across five lanes' own teeth, held off their layer from before they
    start; RX_D0's via cut 7 mm from the tooth via window it closed: the solve proved no plan). A VIA cut is held to the
    lane's route within its room of the site (the lane's place at the cut's u), inside the cut it was; a LAYER cut
    carries the `span` of the lane's route where its laid path (`plan`'s, else the geometry's) meets the island's box
    grown by a lane's clearance -- the solve reads it in place of the box's projection. A cut whose lane the map does not
    reach stands as it was"""
    import route_layers
    tab = geo.get('uxy') or {}
    if len(route_layers.layers()) <= 2 or not tab:
        return cf
    import rules as _rules
    R = _rules.active()
    reach = R.track / 2 + R.clearance + R.grid

    def at_u(rows, u):
        for a, b in zip(rows, rows[1:]):
            if a[0] <= u <= b[0]:
                t = (u - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                return a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t
        return None
    dense = {}

    def rows_of(n):
        if n not in dense:
            dense[n] = _dense(tab[n], R.grid / 2) if n in tab else []
        return dense[n]
    out = dict(cf)
    vc = []
    for c in cf.get('vcuts', []):
        rows = rows_of(c['lane'])
        site = at_u(rows, c['u']) if rows else None
        if site is None:
            vc.append(c)
            continue
        i0 = min(range(len(rows)), key=lambda i: abs(rows[i][0] - c['u']))
        lo = hi = i0                    # the run of the lane's route within the via's room of the site, round it
        near = lambda i: math.hypot(rows[i][1] - site[0], rows[i][2] - site[1]) <= c['w']
        while lo > 0 and near(lo - 1):
            lo -= 1
        while hi < len(rows) - 1 and near(hi + 1):
            hi += 1
        a, b = max(rows[lo][0], c['u'] - c['w']), min(rows[hi][0], c['u'] + c['w'])
        vc.append(dict(c, u=round((a + b) / 2, 4), w=round((b - a) / 2, 4)))
    if 'vcuts' in cf:
        out['vcuts'] = vc
    lanes = (plan or geo).get('lanes') or {}
    lc = []
    for c in cf.get('lcuts', []):
        rows, path = rows_of(c['lane']), (lanes.get(c['lane']) or {}).get('xy')
        if not rows or not path or 'box' not in c:
            lc.append(c)
            continue
        x0, y0, x1, y1 = c['box'][:4]
        us = []
        for p in _dense([list(q) for q in path], R.grid / 2):
            if x0 - reach <= p[0] <= x1 + reach and y0 - reach <= p[1] <= y1 + reach:
                # (the laid path's place on the lane's route: its nearest column, the polish having moved it little)
                d, u = min((math.hypot(r_[1] - p[0], r_[2] - p[1]), r_[0]) for r_ in rows)
                if d <= 2 * reach:
                    us.append(u)
        lc.append(dict(c, span=[round(min(us), 4), round(max(us), 4)]) if us else c)
    if 'lcuts' in cf:
        out['lcuts'] = lc
    return out


def snap_layer_cuts(snap_log, geo):
    """the LAYER cuts a snap's failures give: a lane it could not lay, its search stuck within SNAP_ISLAND_REACH of an
    island on one layer, the layer it was on there -- the nearest such island"""
    out, bxs = [], geo.get('island_boxes') or {}
    for ln in lines_of(snap_log):
        m = re.match(r'\s*(\S+)\s+FAILED:.* at \(([-\d.]+), ([-\d.]+)\) on ((?:F|B|In\d+)\.Cu)', ln)
        if not m:
            continue
        x, y, L = float(m.group(2)), float(m.group(3)), _layer_index(m.group(4))
        d = lambda b: math.hypot(max(b[0] - x, 0.0, x - b[2]), max(b[1] - y, 0.0, y - b[3]))
        near = sorted((d(b), k) for k, b in bxs.items() if list(b[4]) == [L] and d(b) <= SNAP_ISLAND_REACH)
        if near:
            c = layer_cut(m.group(1), near[0][1], bxs[near[0][1]], L)
            if c and c not in out:
                out.append(c)
    return out


def loop(solve, out, rounds, env, log):
    """whole_loop.sh SOLVE OUTDIR ROUNDS, its lines to LOG: 0 with OUTDIR/plan.json the snapped plan that passed, 1 when
    no round got there, 3 when it stopped not converging, 4 when it stopped at crowded ends"""
    env = dict(env)
    # the braid's plan environment the planning reads (a pages-first sidecar's paging, its pairs)
    env.update(BRAID_PAIRS='1', BRAID_EXACT_PAGES='0', PLAN_PAGES_SIDERS='2')
    # every expensive stage through stage_cache.py -- on here, a harness's cache (STAGE_CACHE=0 runs them all)
    env['STAGE_CACHE'] = env.get('STAGE_CACHE') or '1'
    env['TAUT_MEMO'] = env.get('TAUT_MEMO') or '1'
    os.makedirs(out, exist_ok=True)
    O = lambda name: os.path.join(out, name)
    solve = os.path.abspath(solve)
    st = dict(flips=env.get('SEED_FLIPS', ''), cuts=env.get('SEED_CUTS', ''), hist=env.get('SEED_HIST', ''),
              best=-1, best_i=0, stall=0, solve=solve)
    # (more routing layers than two, a relayed bench: each solve's plan on its own relayed bench -- the bench each
    # round's plans stand on, for the lay: plan_bench)
    benches = {}

    def bench_is(key):
        benches[key] = env['BENCH']
        json.dump(benches, open(O('benches.json'), 'w'), indent=1)
    # what a command inside the loop leaves unredirected: the round's loop.log in the chain, the terminal on its own
    err = log.f if log.f is not None else sys.stderr
    unredirected = log.f if log.f is not None else INHERIT

    def stage(outs, script, args, logp, **extra):
        """a stage through stage_cache.py, `> LOGP 2>&1`; its exit code. In this process (INPROC) a stage runs
        itself: the stage cache is the harness's, and it ends each stage with atexit's exit functions, which in one
        process are every module's"""
        if INPROC:
            return run([script] + list(args), {**env, **extra}, log_path=logp)[0]
        argv = ['stage_cache.py'] + [a for o in outs for a in ('--out', o)] + ['--', script] + list(args)
        return run(argv, {**env, **extra}, log_path=logp)[0]

    def gate(plan, audit, *more):
        """whole_gate.py PLAN AUDIT [...]: (exit code, stdout), its stderr to the loop's log"""
        rc, so = run(['whole_gate.py', plan, audit] + list(more), env, err=err)
        return rc, stripped(so)

    def addhot(plan, found, hot):
        """an audit's findings as history: its hot places (whole_gate --hot), added when it names any"""
        gate(plan, found, '--hot', hot)
        try:
            if len(json.load(open(hot))['hot']) == 0:
                return False
        except Exception:                                    # (the shell's test reads a python traceback as not "0")
            traceback.print_exc(file=err)
        st['hist'] = (st['hist'] + ',' if st['hist'] else '') + hot
        return True

    def failed(logp):
        for ln in tail(logp, 3):
            log(ln)
        return 1

    def progress(score, i, fresh=False):
        # (fresh: the round found side flips it has not tried -- not a stall, whatever its score)
        if st['best'] < 0 or score < st['best']:
            st['best'], st['best_i'], st['stall'] = score, i, 0
            # (the round a loop that does not pass is laid from: the chain's held plan, held_plan)
            json.dump({'i': i, 'score': score}, open(O('best.json'), 'w'))
        elif not fresh:
            st['stall'] += 1
        if st['stall'] >= PATIENCE:
            log(f"=== round {i}: NOT CONVERGING -- score {score}, the best {st['best']} at round {st['best_i']}, "
                f"{PATIENCE} rounds without a better one")
            raise NotConverging

    def resolve(i):
        """the solve again, warm, with every cut and every audit's history so far -- less an island cut a side flip has
        answered since (the flip puts that lane on the island's other side; the cut would keep holding it off the
        island); None when the solve failed"""
        s2, cuts_out = O(f's{i + 1}.json'), O(f'cuts{i + 1}.json')
        try:
            fl = lambda fs: {tuple(x[:2]) for f in fs if f and os.path.isfile(f)
                             for x in json.load(open(f)).get('flips', [])}
            flipped = fl(st['flips'].split(','))
            cuts, vcuts, lcuts = [], [], []
            for f in [f for f in st['cuts'].split(',') if f]:
                c = json.load(open(f))
                # a cut from the geometry of round k whose flip that geometry had ALREADY been given (a polish of an
                # earlier round found it, or the seed): both sides of the island failed -- the solve hears of it.
                # Only a cut the flip answers, one from before it, is dropped (zynq K42: DQ10's cut at C98, made again
                # after its flip, never reached the solve)
                m = re.search(r'/c(\d+)\.json$', f)
                k = int(m.group(1)) if m else 0
                given = fl(env.get('SEED_FLIPS', '').split(',') + [os.path.join(out, f'{nm}{j}.json')
                                                                   for j in range(1, k) for nm in ('p', 'q', 'qk')])
                cuts += [x for x in c.get('cuts', []) if (x['lane'], x['island']) not in flipped
                         or (x['lane'], x['island']) in given]
                vcuts += c.get('vcuts', [])
                lcuts += [x for x in c.get('lcuts', []) if x not in lcuts]
            json.dump({'cuts': cuts, 'vcuts': vcuts, 'lcuts': lcuts}, open(cuts_out, 'w'))
        except Exception:
            traceback.print_exc(file=err)
        soft = {'SOFT_CUTS': st['soft']} if st.get('soft') else {}
        import route_layers
        if len(route_layers.layers()) > 2:
            # (more routing layers than two: ONE solve, the cuts FIRM -- each held unless broken at a price above
            # anything a plan can buy, so every cut that can hold does and only those no plan holds break -- keeping
            # a plan it cannot prove, as such a solve almost never proves one. The hard pass first spent four minutes
            # of the zynq's LVDS round proving no plan, for two via cuts of 34 that could not hold together, before
            # the soft pass the loop then ran anyway)
            if stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.log', BENCH=env.get('BENCH0') or env['BENCH'],
                     HINT=st['solve'], CUTS=cuts_out if os.path.isfile(cuts_out) else '', HIST=st['hist'],
                     CUTS_FIRM='1', SOLVE_UNPROVED='1', **soft) != 0:
                return failed(s2[:-5] + '.log')
            for ln in grep(s2[:-5] + '.log', r'whole_solve|vias|check|history|firm cuts'):
                log('  ' + ln)
            took(s2, i)
            return None
        if stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.log', BENCH=env.get('BENCH0') or env['BENCH'],
                 HINT=st['solve'], CUTS=cuts_out, HIST=st['hist'], **soft) != 0:
            # the CUTS left no plan: they are absolute, and together they can ask more than any plan gives -- a net
            # over two vias proved necessary, no plan proved (K41 with the stub check per layer: each of round 1's
            # island cuts and via cuts, alone, proved one; all six island cuts at places the polish had cleared).
            # The solve again with them SOFT (whole_solve SOFT_CUTS: a price where they were a wall) and the history,
            # which carries every place the audits found short -- for this solve and the rounds after (as hard cuts
            # they would leave the same solve with no plan again)
            none = O(f'cuts{i + 1}_none.json')
            json.dump({'cuts': [], 'vcuts': []}, open(none, 'w'))
            if not st['cuts'] or not os.path.isfile(cuts_out):
                # no cuts to soften: the same solve, keeping a plan it cannot prove -- the loop goes on with it rather
                # than ending on the plan it held (a plan for every lane before fewer vias)
                log(f"  no proved plan ({', '.join(tail(s2[:-5] + '.log', 1))[:120]}) -- the solve again, keeping one "
                    f"it cannot prove")
                if stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.log', BENCH=env.get('BENCH0') or env['BENCH'],
                         HINT=st['solve'], CUTS=cuts_out if os.path.isfile(cuts_out) else none, HIST=st['hist'],
                         SOLVE_UNPROVED='1', **soft) != 0:
                    return failed(s2[:-5] + '.log')
                for ln in grep(s2[:-5] + '.log', r'whole_solve|vias|check|history'):
                    log('  ' + ln)
                took(s2, i)
                return None
            st['soft'] = (st['soft'] + ',' if st.get('soft') else '') + cuts_out
            log(f"  the cuts left no plan ({', '.join(tail(s2[:-5] + '.log', 1))[:120]}) -- the solve again with "
                f"them SOFT, and the history")
            soft = {'SOFT_CUTS': st['soft']}
            # (that re-solve keeps a plan it cannot prove: refused, K41's -- the cuts soft, best 52 vias against a
            # bound of 45 -- ended the loop on its held plan, 5 nets open)
            if stage([s2], 'whole_solve.py', [s2], s2[:-5] + '.log', BENCH=env.get('BENCH0') or env['BENCH'],
                     HINT=st['solve'], CUTS=none, HIST=st['hist'], SOLVE_UNPROVED='1', **soft) != 0:
                return failed(s2[:-5] + '.log')
            st['cuts'] = ''
        for ln in grep(s2[:-5] + '.log', r'whole_solve|vias|check|history'):
            log('  ' + ln)
        took(s2, i)
        return None

    def took(s2, i):
        """the re-solve s2 the loop's plan from here: on a relayed bench (BENCH0), the fanout's own bench relayed
        afresh for its via ends' layers, the bench the stages after it read"""
        st['solve'] = s2
        if env.get('BENCH0'):
            env['BENCH'] = relay_ends(out, env['BENCH0'], json.load(open(s2)), log, name=f'bench{i + 1}')

    def body():
        for i in range(1, rounds + 1):
            if os.path.exists(O(f'hp{i}.json')):
                os.remove(O(f'hp{i}.json'))    # (a round that passes writes none: an earlier run's must not stand in)
            flips = st['flips']
            bench_is(str(i))
            log(f"=== round {i}: geometry of {os.path.basename(st['solve'])}"
                + (f" (flips from {os.path.basename(flips.split(',')[-1])})" if flips else ''))
            g, p = O(f'g{i}.json'), O(f'p{i}.json')
            if stage([g], 'whole_geo.py', [st['solve'], g], O(f'g{i}.log'), GEO_FLIPS_FROM=flips) != 0:
                return failed(O(f'g{i}.log'))
            if stage([p], 'whole_polish.py', [g, p], O(f'p{i}.log')) != 0:
                return failed(O(f'p{i}.log'))
            if stage([], 'whole_audit.py', [p], O(f'p{i}.audit')) != 0:
                return failed(O(f'p{i}.audit'))
            gl = gate(p, O(f'p{i}.audit'))[1]
            log(prefixed(gl, '  smooth: '))
            f = findings(gl)
            before = nflips(flips) if flips else 0
            after = nflips(p)
            # the round's cuts -- the geometry's islands and via cuts, the polish's via cuts -- less an island cut that
            # one of the round's NEW flips answers (the flip puts that lane on the island's other side)
            gj, pj = json.load(open(g)), json.load(open(p))
            old = {tuple(x) for fl in flips.split(',') if fl for x in json.load(open(fl)).get('flips', [])}
            new = {tuple(x) for x in pj.get('flips', [])} - old
            cuts = [c for c in gj.get('cuts', []) if (c['lane'], c['island']) not in new]
            vcuts = gj.get('vcuts', []) + pj.get('vcuts', [])
            # ...and the LAYER cuts: the audit's findings (a lane found against an island not on every layer), and on
            # more routing layers than two the geometry's and the polish's own -- there a layer cut, not a flip, is the
            # answer to such an island, and it stands whatever flip names the lane and island
            import route_layers
            nl_ = len(route_layers.layers())
            lcuts = [c for c in audit_layer_cuts(O(f'p{i}.audit'), gj) if nl_ > 2 or (c['lane'], c['island']) not in new]
            lcuts += [c for c in gj.get('lcuts', []) + pj.get('lcuts', []) if c not in lcuts]
            if lcuts:
                log(f"  layer cuts: {', '.join(c_['lane'] + ' under ' + c_['island'] for c_ in lcuts)}")
            json.dump(own_spans({'cuts': cuts, 'vcuts': vcuts, 'lcuts': lcuts}, gj, pj), open(O(f'c{i}.json'), 'w'))
            n = len(cuts) + len(vcuts) + len(lcuts)
            passes = gate(p, O(f'p{i}.audit'))[0] == 0
            if not passes and addhot(p, O(f'p{i}.audit'), O(f'hp{i}.json')):
                n += 1
            # a finding at the ENDS standing in two rounds running: the solve had its round and did not move it, the
            # fanout must (whole_feedback --repeat; the fanout's sidecar beside BENCH) -- stopped here rather than
            # solving again and again. Not while the round has new side flips: a flip is the geometry's own answer
            sidecar = env['BENCH'][:-len('.kicad_pcb')] + '.plan.json' if env['BENCH'].endswith('.kicad_pcb') \
                else env['BENCH'] + '.plan.json'
            newflips = after > before
            # (more routing layers than two: the round's LAYER cuts are its own answer, as a side flip is -- a round with
            # them is not one whose findings at the ends no solve moves)
            answered = newflips or (nl_ > 2 and bool(lcuts))
            hp, hp0 = O(f'hp{i}.json'), O(f'hp{i - 1}.json')
            if (not passes and not answered and os.path.isfile(hp) and os.path.isfile(sidecar)
                    and run(['whole_feedback.py', '--now', sidecar, hp], env, to=unredirected)[0] == 0):
                # ...and at once, when a finding there is one no solve moves (a pitch or a static clearance at the ends)
                log(f"=== round {i}: ENDS CROWDED -- findings at the ends no solve moves: the fanout's to change")
                return 4
            if (not passes and not answered and i > 1 and os.path.isfile(hp0) and os.path.isfile(hp)
                    and os.path.isfile(sidecar)
                    and run(['whole_feedback.py', '--repeat', sidecar, hp0, hp], env, to=unredirected)[0] == 0):
                log(f"=== round {i}: ENDS CROWDED -- the same findings at the ends two rounds running: "
                    f"the fanout's to change")
                return 4
            if after > before:
                progress(2000 + f, i, fresh=True)
                st['flips'] = p                       # the polish output carries every flip so far
                if n == 0:
                    log(f"=== round {i}: {after - before} new side flip(s) -> the geometry again on the same solve")
                    continue
                # flips AND cuts or findings: both at once -- the solve with them, then the geometry with the flips
                st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'c{i}.json')
                log(f"=== round {i}: {after - before} new side flip(s), and cuts or findings -> the solve again, "
                    f"then the geometry with the flips")
                rc = resolve(i)
                if rc is not None:
                    return rc
                continue
            if passes:
                rc = smooth_passes(i)
                if rc == 'continue':
                    continue
                return rc
            progress(2000 + f, i)
            if n == 0:
                log(f"=== round {i}: no flips, no cuts and no findings with a place left")
                return 1
            st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'c{i}.json')
            log(f"=== round {i}: cuts or findings -> the solve again")
            rc = resolve(i)
            if rc is not None:
                return rc
        log("=== no round passed")
        return 1

    def smooth_passes(i):
        """the PAIRS first, laid as the pair router moves (its turning radius, its straight dives), then the singles
        fitted round them (the polish, the pairs held) and snapped: an exit code, or 'continue' for the next round"""
        p = O(f'p{i}.json')
        log(f"=== round {i}: the smooth plan passes -> the pairs laid first")
        pairs, pl = O(f'pairs{i}.json'), O(f'pairs{i}.log')
        if stage([pairs], 'whole_snap.py', [p, pairs, '--pairs'], pl) != 0 and not grep(pl, r'^SNAP FAILED'):
            return failed(pl)
        for ln in grep(pl, r'^snap:|FAILED'):
            log('  ' + ln)
        if grep(pl, r'^SNAP FAILED'):
            # a pair the snap cannot lay: its dive nearest where it got stuck goes to the solve as a via cut
            # (whole_snap's dive_cuts), and the solve again; a pair with no dive to move there stops
            nd = None
            try:
                d = json.load(open(pairs)).get('dive_cuts', [])
                json.dump(own_spans({'vcuts': d}, json.load(open(O(f'g{i}.json')))), open(O(f'dc{i}.json'), 'w'))
                nd = len(d)
            except Exception:
                traceback.print_exc(file=err)
            if nd == 0:
                log(f"=== round {i}: a pair cannot be laid")
                return 1
            progress(1500, i)
            st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'dc{i}.json')
            log(f"=== round {i}: a pair cannot be laid at a dive -> the solve again, {nd if nd is not None else ''} "
                f"dive(s) moved (via cuts)")
            rc = resolve(i)
            return 'continue' if rc is None else rc
        q = O(f'q{i}.json')
        if stage([q], 'whole_polish.py', [pairs, q], O(f'q{i}.log')) != 0:
            return failed(O(f'q{i}.log'))
        if stage([], 'whole_audit.py', [q], O(f'q{i}.audit')) != 0:
            return failed(O(f'q{i}.audit'))
        gq = gate(q, O(f'q{i}.audit'))[1]
        log(prefixed(gq, '  pairs held: '))
        chosen = q
        if gate(q, O(f'q{i}.audit'))[0] != 0:
            # the singles do not fit round the pairs: the pairs laid AGAIN keeping each single's room where it was
            # short (whole_snap SNAP_KEEP), then the singles fitted again round those
            gate(q, O(f'q{i}.audit'), '--hot', O(f'hk{i}.json'))
            # (a pair that cannot be laid so is no plan: the pairs laid first stand, and their singles' places go to
            # the solve below; any other failure stops)
            pk, pkl = O(f'pairs{i}k.json'), O(f'pairs{i}k.log')
            if stage([pk], 'whole_snap.py', [p, pk, '--pairs'], pkl, SNAP_KEEP=O(f'hk{i}.json')) != 0:
                if not grep(pkl, r'^SNAP FAILED'):
                    return failed(pkl)
                log(f"  pairs laid again, the singles kept room: {chr(10).join(grep(pkl, r'^SNAP FAILED'))}")
            if not grep(pkl, r'^SNAP FAILED'):
                qk = O(f'qk{i}.json')
                if stage([qk], 'whole_polish.py', [pk, qk], O(f'qk{i}.log')) != 0:
                    return failed(O(f'qk{i}.log'))
                if stage([], 'whole_audit.py', [qk], O(f'qk{i}.audit')) != 0:
                    return failed(O(f'qk{i}.audit'))
                gk = gate(qk, O(f'qk{i}.audit'))[1]
                log(prefixed(gk, '  pairs laid again, the singles kept room: '))
                if gate(qk, O(f'qk{i}.audit'))[0] == 0:
                    chosen = qk
        if chosen == q and gate(q, O(f'q{i}.audit'))[0] != 0:
            # the singles do not fit round the pairs: where they are short goes to the solve as history -- and the
            # side flips the polish found with the pairs held go to the next geometry, as a smooth polish's do
            qf = O(f'qk{i}.json') if os.path.isfile(O(f'qk{i}.json')) else q
            o = {tuple(x) for fl in st['flips'].split(',') if fl for x in json.load(open(fl)).get('flips', [])}
            nq = len({tuple(x) for x in json.load(open(qf)).get('flips', [])} - o)
            if nq != 0:
                st['flips'] = (st['flips'] + ',' if st['flips'] else '') + qf
                log(f"  pairs held: {nq} new side flip(s) for the next geometry")
            progress(1000 + findings(gq), i, fresh=nq != 0)
            if not addhot(q, O(f'q{i}.audit'), O(f'hq{i}.json')):
                log(f"=== round {i}: the singles do not fit round the pairs")
                return 1
            log(f"=== round {i}: the singles do not fit round the pairs -> the solve again, their places priced")
            rc = resolve(i)
            return 'continue' if rc is None else rc
        # (a single the snap cannot lay leaves the plan without it -- the gate fails it -- and where it got stuck goes
        # to the solve with the audit's findings below; any other failure stops)
        plan, sl = O('plan.json'), O('snap.log')
        bench_is('plan')
        if stage([plan], 'whole_snap.py', [chosen, plan], sl) != 0 and not grep(sl, r'^SNAP FAILED'):
            return failed(sl)
        for ln in grep(sl, r'^snap:|FAILED'):
            log('  ' + ln)
        if stage([], 'whole_audit.py', [plan], O('plan.audit')) != 0:
            return failed(O('plan.audit'))
        gs = gate(plan, O('plan.audit'))[1]
        log(prefixed(gs, '  snapped: '))
        run(['whole_lint.py', plan], env, log_path=O('plan.lint'))
        lint = (tail(O('plan.lint'), 1) or [''])[0]
        log(f"  snapped: {lint}")
        if gate(plan, O('plan.audit'))[0] == 0 and lint == 'LINT clean':
            log(f"=== the plan passes: {plan}")
            return 0
        # the snapped plan is short: where goes to the solve as history -- the audit's places and the lint's (a lane
        # folded at its end: the end it folds at), a finding with no place stops
        folded = sum(1 for ln in lines_of(O('plan.lint')) if re.search(r'^LINT \S+ \S+ ', ln)
                     and not re.search(r'^LINT( [a-z-]+ [0-9]+,?)+$', ln))
        progress(findings(gs) + folded, i)
        with open(O('plan.found'), 'w') as ff:
            ff.write(open(O('plan.audit')).read() + open(O('plan.lint')).read())
        if not addhot(plan, O('plan.found'), O(f'hs{i}.json')):
            log(f"=== round {i}: the snapped plan does not pass")
            return 1
        # ...and a lane the snap could not lay beside an island on one layer: held under it (a LAYER cut)
        lc = snap_layer_cuts(sl, json.load(open(O(f'g{i}.json'))))
        if lc:
            json.dump(own_spans({'lcuts': lc}, json.load(open(O(f'g{i}.json'))), json.load(open(chosen))),
                      open(O(f'lc{i}.json'), 'w'))
            st['cuts'] = (st['cuts'] + ',' if st['cuts'] else '') + O(f'lc{i}.json')
            log(f"  layer cuts: {', '.join(c_['lane'] + ' under ' + c_['island'] for c_ in lc)}")
        log(f"=== round {i}: the snapped plan does not pass -> the solve again, its places priced")
        rc = resolve(i)
        return 'continue' if rc is None else rc

    try:
        return body()
    except NotConverging:
        return 3


# =============================================================================== the chain (whole_chain.sh)
def grade_counts(board, nets):
    """(vias, copper mm) of the run's nets on BOARD, counted as the chain counts them"""
    import math
    if PYR not in sys.path:
        sys.path.insert(0, PYR)
    with contextlib.redirect_stdout(io.StringIO()):
        from kicad_parser import parse_kicad_pcb
        p = parse_kicad_pcb(board)
    nm = {i: n.name.split('/')[-1] for i, n in p.nets.items()}
    return (str(sum(1 for v in p.vias if nm.get(v.net_id) in nets)),
            str(round(sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in p.segments
                          if nm.get(s.net_id) in nets))))


def count_open(conn_log):
    """the nets check_connected's log finds open: those with no copper ('Unrouted nets (N)') and those in pieces
    ('Connectivity issues (M)')"""
    n = 0
    for ln in lines_of(conn_log):
        m = re.match(r'\s*(Unrouted nets|Connectivity issues) \((\d+)\)', ln)
        if m:
            n += int(m.group(2))
    return n


def open_nets(conn_log):
    """the nets (short names) check_connected's log finds open: unrouted, or in pieces"""
    out, mode = set(), None
    for ln in lines_of(conn_log):
        s = ln.strip()
        if s.startswith('Unrouted nets'):
            mode = 'u'
        elif s.startswith('Connectivity issues'):
            mode = 'c'
        elif mode == 'u' and s.endswith('pads)'):
            out.add(s.split(' (')[0].split('/')[-1])
        elif mode == 'c' and '(net ' in s and s.endswith(':'):
            out.add(s.split(' (net')[0].split('/')[-1])
    return out


def relay_ends(d, bench, J, say, name='fo_layers'):
    """(more routing layers than two) the bench with every via end on the layer the round's solve chose for it
    (J['layers'], each lane's first and last run; a pair's every leg) -- relayer.py moving each stub's run from its via
    out, where it stands on another -- written as d/fo_layers.kicad_pcb; `bench` itself when none moves"""
    import pairs as _pairs
    sc_ = os.path.splitext(bench)[0] + '.plan.json'
    if not os.path.isfile(sc_):
        say(f"  (no plan sidecar beside {os.path.basename(bench)}: the via ends stay as laid)")
        return bench
    sc = json.load(open(sc_))
    nets = list(sc.get('ends', {}))
    legs = {b_: list(pr_) for b_, pr_ in _pairs.pair_names(nets).items()}
    if PYR not in sys.path:
        sys.path.insert(0, PYR)          # (the relayer reads and writes boards: called inside the loop's own process too)
    # (the solve's layers are the board's as the braid's setup reads it: a board of the other chirality turned over,
    # F and B swapped -- read back into the sidecar's own frame, or a run kept on the board's B moves onto its F)
    from bga_fanout.flip_frame import other_layer
    OL = other_layer if int(sc.get('chi') or 1) < 0 else (lambda L: L)
    moves = []
    for lane, ls in sorted(J['layers'].items()):
        for k_, L1 in ((0, OL(ls[0])), (1, OL(ls[-1]))):
            for leg in legs.get(lane, [lane]):
                L0 = sc['tooth_layer' if k_ == 0 else 'dest_layer'].get(leg)
                if L0 and L1 != L0 and leg in sc['ends']:
                    moves.append((leg, tuple(sc['ends'][leg][k_]), L0, L1, (lane, k_)))     # (a pair end: one group)
    if not moves:
        return bench
    import relayer
    out = os.path.join(d, name + '.kicad_pcb')
    moved, left = relayer.relayer(bench, out, moves, log=lambda s: say('  ' + s))
    if left:
        say(f"  the plan's layers for {', '.join(sorted(set(left)))} could not be laid: their stubs run to no via")
    return out


def relayed(d, bench, J, env, say):
    """(more routing layers than two) the bench a solve's plan `J` is laid on: its VIA ENDS moved onto the layers the
    solve chose for them (relay_ends), `env` pointed at it (BENCH: the geometry, the polish, the snap, the audits and
    the lay read its copper) and at the fanout's own bench (BENCH0: every RE-SOLVE of the loop is the round's solve
    again, its via ends free -- the same model, so the round's proof floors it and its plan warms it, and a re-solve may
    choose an end's layer anew; loop relays its plan in turn). `bench` itself when none moves. Held on the relayed bench
    instead (VIA_ENDS 0), a re-solve was another model: no floor, no warm start, every lane a tooth via, and the first
    solve's end layers stood for the whole loop"""
    if J.get('layers'):
        b2 = relay_ends(d, bench, J, say)
        if b2 != bench:
            env.update(BENCH=b2, BENCH0=bench)
            return b2
    return bench


def plan_bench(loopd, plan, default):
    """the bench a loop's plan stands on (its round's relayed bench, loop's benches.json), else `default`"""
    try:
        b = json.load(open(os.path.join(loopd, 'benches.json')))
    except (OSError, ValueError):
        return default
    nm = os.path.basename(plan or '')
    if nm == 'plan.json':
        return b.get('plan', default)
    m = re.match(r'^[a-z]+(\d+)', nm)
    return b.get(m.group(1), default) if m else default


def held_plan(loopd, env):
    """the plan a round whose loop did not pass is laid from -- the chain never ends with nothing: the loop's snapped
    plan when a round got that far (plan.json, short of the audit or the lint), else its best round's smooth plan
    (best.json, else its last) laid as a passing one is -- the pairs snapped first, the singles polished round them,
    then snapped -- with no gate; a lane the snap cannot lay is left out of it. None when the loop held none"""
    O = lambda name: os.path.join(loopd, name)
    if os.path.isfile(O('plan.json')):
        return O('plan.json')
    ps = sorted((f for f in glob.glob(O('p*.json')) if re.search(r'/p\d+\.json$', f)),
                key=lambda f: int(re.search(r'/p(\d+)\.json$', f).group(1)))
    if not ps:
        return None
    src = ps[-1]
    try:
        b = O(f"p{json.load(open(O('best.json')))['i']}.json")
        if os.path.isfile(b):
            src = b
    except Exception:
        pass
    run(['whole_snap.py', src, O('held_pairs.json'), '--pairs'], env, log_path=O('held_pairs.log'))
    if os.path.isfile(O('held_pairs.json')) and \
            run(['whole_polish.py', O('held_pairs.json'), O('held_q.json')], env, log_path=O('held_q.log'))[0] == 0:
        src = O('held_q.json')
    run(['whole_snap.py', src, O('held_plan.json')], env, log_path=O('held_plan.log'))
    return O('held_plan.json') if os.path.isfile(O('held_plan.json')) else None


def partial_drops(named, xs, lanes):
    """the last resort's sets of lanes to leave out, each larger than the one before: the lanes the rounds named
    (else the three most crossed), then the most crossed besides to a quarter of the lanes, then to half -- never every
    lane. `xs` {lane: its crossings} (the ends model's), `lanes` the run's"""
    L = len(lanes)
    by_x = [ln for ln in sorted(lanes, key=lambda ln: (-xs.get(ln, 0), ln)) if ln not in named]
    drops = []
    for size in (len(named) or 3, -(-L // 4), -(-L // 2)):
        dr = list(named)[:L - 1] + by_x[:max(0, min(size, L - 1) - len(named))]
        if dr and (not drops or len(dr) > len(drops[-1])):
            drops.append(dr)
    return drops


def add_over(fb_path, over_nets):
    """the lanes an UNPROVED plan leaves over two vias (whole_solve 'over_nets'), merged into the fanout's feedback
    (FEEDBACK=, whole_ends 'over': each counted over by at least that much); the lanes it newly names"""
    if not over_nets:
        return []
    fb = json.load(open(fb_path)) if os.path.isfile(fb_path) else {'pairs': [], 'avoid': []}
    ov = fb.setdefault('over', {})
    new = [n for n, k in sorted(over_nets.items()) if int(k) > int(ov.get(n, 0))]
    for n in new:
        ov[n] = int(over_nets[n])
    json.dump(fb, open(fb_path, 'w'), indent=1)
    return new


def stage_env(env):
    """the settings every stage of the chain runs under, set in `env` (and returned): one thread per numeric library,
    the harness's caches, and the plan environment of the fanout on our own ends (resolve_round.py runs one stage
    under the same)"""
    env.update(OMP_NUM_THREADS='1', VECLIB_MAXIMUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               TAUT_MEMO=env.get('TAUT_MEMO') or '1', PROBE_MEMO=env.get('PROBE_MEMO') or '1')
    env.update(PLAN_PAGES='1', PLAN_JUDGE='ends', BRAID_PAIRS='1', PLAN_PAIRS='1',
               BRAID_EXACT_PAGES='0', PLAN_PAGES_SIDERS='2')
    return env


def footprint_poses(board):
    """{reference: [x, y, rotation]} of each footprint on `board` whose reference no other carries"""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    seen = collections.Counter(fp.reference for fp in pcb.footprints.values())
    return {fp.reference: [round(fp.x, 4), round(fp.y, 4), round(fp.rotation or 0.0, 4)]
            for fp in pcb.footprints.values() if seen[fp.reference] == 1}


def place_held(board, out, held):
    """`board` with the footprints `held` ({reference: [x, y, rotation]}) where it says, written to `out` (its project
    beside it); `board` itself when every one of them already stands there"""
    now = footprint_poses(board)
    moves = [{'reference': r, 'new_x': p[0], 'new_y': p[1], 'new_rotation': p[2]}
             for r, p in sorted(held.items()) if r in now and now[r] != p]
    if not moves:
        return board
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'py_placer'))
    from placement.writer import write_placed_output
    import fanout_from_plan as fp_
    with contextlib.redirect_stdout(io.StringIO()):
        write_placed_output(board, out, moves)
    fp_.copy_pro(board, out)
    return out


def chain(K, o, R=3, base=None, dest=None, settings=None):
    """whole_chain.sh K OUTDIR ROUNDS: the exit code, the grade the last line printed. The bench and its destination
    are `base` and `dest` when given (route_bus.py), else BASE and DEST from the environment; `settings` go over the
    environment for every stage (route_bus.py's caches off)"""
    o = os.path.abspath(o)
    os.makedirs(o, exist_ok=True)
    t0 = time.time()
    secs = lambda: int(time.time() - t0)
    env = dict(os.environ)
    env.update(settings or {})
    if base:                                     # (BASE and DEST reach a stage only when given or set)
        env['BASE'] = base
    if dest:
        env['DEST'] = dest
    base = env.get('BASE') or 'fb_t2q_pairs.kicad_pcb'
    dest = env.get('DEST') or 'DU1'
    stage_env(env)
    nets_out = run(['coherent_nets.py', str(K), f'--board={base}'], env)[1]
    NETS = (nets_out.splitlines() or [''])[-1]
    FB = os.path.join(o, 'feedback.json')
    if os.path.exists(FB):
        os.remove(FB)
    # the passives where the joint fanout's cap step left them, after the first round's fanout: nothing moves them
    # again -- every later round starts from them, fans out round them and routes round them
    HELD = os.path.join(o, 'held_parts.json')
    if os.path.exists(HELD):
        os.remove(HELD)
    prev = ''
    say = Log()
    r = 0
    # the best result of the rounds so far, by its open nets, then its vias (and a result with a violation after every
    # clean one): the chain never ends with nothing when a round had a plan to lay
    best = None
    nets_all = [n for n in NETS.split(',') if n]

    def advance(d, base):
        """the next round's base: this round's realized source board (its fo.log names it), else `base` as it was --
        the passives where the first round's cap step left them (HELD); the source board is written before it moves
        them, and a base without them had every round move them again, from where they stood, not always the same"""
        was = base
        sb = source_board_named(os.path.join(d, 'fo.log'))
        if sb and os.path.isfile(os.path.join(d, sb)):
            base = os.path.join(d, sb)
        if os.path.isfile(HELD):
            base = place_held(base, os.path.join(d, 'base_held.kicad_pcb'), json.load(open(HELD)))
        if base != was and 'BASE' in env:
            env['BASE'] = base
        return base

    def lay(r, d, plan, bench, nets=None):
        """route_lanes all at once on PLAN, graded: the result (its key, open nets, violation, grade line, board), or
        None when no board was written. `nets` the nets the plan lays (a partial plan's, the last resort), else the
        run's; open nets are counted over the run's whatever it lays"""
        rr = ['--plan', plan, '--board', bench, '--nets', ','.join(nets) if nets else NETS, '--dest', dest]
        seq = os.path.join(d, 'seq.kicad_pcb')
        run(['route_lanes.py', 'all'] + rr + ['--mode', 'seq', '--write', seq], {**env, 'BRAID_PAIR_SLACKS': '0'},
            log_path=os.path.join(d, 'route_seq.log'))
        sm = '\n'.join(grep(os.path.join(d, 'route_seq.log'), r'^SUMMARY'))
        say(f"  all at once: {sm}")
        if not os.path.isfile(seq):
            return None
        pats = [f'*{n}' for n in nets_all]
        # (the checkers by the path the shell gives them, from awx/: their logs echo it; always in a process of
        # their own: they are the harness's grade, not the route, and a checker run as a script installs its
        # command-line banner -- the CMD / EXIT echo -- for its whole process)
        checker = lambda name: os.path.join('..', 'py_router', name)
        cc = run([checker('check_connected.py'), seq, '--nets'] + pats, env,
                 log_path=os.path.join(d, 'conn.log'), own_process=True)[0]
        dc = run([checker('check_drc.py'), seq, '--nets'] + pats + ['--clearance-margin', '0.1'], env,
                 log_path=os.path.join(d, 'drc.log'), own_process=True)[0]
        nopen = 0 if cc == 0 else max(1, count_open(os.path.join(d, 'conn.log')))
        lanes = '\n'.join(m.split(' ')[0] for m in re.findall(r'[0-9]+/[0-9]+ in band', sm))
        v, c = grade_counts(seq, set(nets_all))
        g = (f"WHOLE K={K} round={r} lanes={lanes} vias={v} copper={c}mm connected={int(cc == 0)} "
             f"drc={int(dc == 0)} secs={secs()} open={nopen}")
        say(g)
        return dict(key=(int(dc != 0), nopen, int(v), int(c)), open=nopen, drc=int(dc != 0), grade=g, board=seq,
                    open_nets=sorted(open_nets(os.path.join(d, 'conn.log'))) if nopen else [])

    def lane_legs():
        """{lane: its nets} as the ends model makes lanes: a pair one lane named by its base (pairs.pair_names, under
        PLAN_PAIRS), every other net its own"""
        import pairs as _pairs
        prs = (_pairs.pair_names(nets_all) if int(env.get('PLAN_PAIRS', env.get('BRAID_PAIRS', '0')) or 0) else {})
        legs = {l_ for pr in prs.values() for l_ in pr}
        return {**{n: (n,) for n in nets_all if n not in legs}, **{b: tuple(pr) for b, pr in prs.items()}}

    def last_resort(r):
        """THE LAST RESORT, when no round laid anything: a PARTIAL plan on the last ends a fanout laid -- the solve,
        its loop and the route all at once on the lanes LESS those the rounds named (whole_feedback --name: open, else
        the ends model's over two, loaded, most crossed), then less the most crossed besides to a quarter of the lanes, then to
        half; the lanes left out stay open (route_bus hands them on as they came). The first that lays anything is the
        result: a run's open nets are never all of them while a plan for some can be laid. A board the fanout's audit
        refused will do, the lanes it named left out with them (its split pairs, its teeth on the far face). None when
        none does (or no round laid a fanout board at all)"""
        bench, refused_ = None, []
        for k in range(r, 0, -1):
            d_ = os.path.join(o, f'r{k}')
            if not os.path.isfile(os.path.join(d_, 'fo.plan.json')):
                continue
            if os.path.isfile(os.path.join(d_, 'fo.kicad_pcb')):
                bench = os.path.join(d_, 'fo.kicad_pcb')
            elif os.path.isfile(os.path.join(d_, 'fo.refused.kicad_pcb')) and \
                    os.path.isfile(os.path.join(d_, 'fo.refused.json')):
                bench = os.path.join(d_, 'fo.refused.kicad_pcb')
                rj = json.load(open(os.path.join(d_, 'fo.refused.json')))
                refused_ = [x[0] for x in rj.get('splits', [])] + list(rj.get('far', []))
            if bench:
                d0 = d_
                break
        if not bench:
            return None
        legs = lane_legs()
        xs = (json.load(open(os.path.join(d0, 'fo.plan.json'))).get('ends_model') or {}).get('x') or {}
        named = [ln for ln in ((json.load(open(FB)).get('named') if os.path.isfile(FB) else None) or []) if ln in legs]
        named = list(dict.fromkeys([ln for ln in refused_ if ln in legs] + named))
        drops = partial_drops(named, xs, list(legs))
        for j, dr in enumerate(drops, 1):
            dj = os.path.join(o, f'partial{j}')
            os.makedirs(dj, exist_ok=True)
            keep = {n for ln, lg in legs.items() if ln not in dr for n in lg}
            sub = [n for n in nets_all if n in keep]
            say(f"=== the last resort {j}: a partial plan on {os.path.relpath(bench, o)}, {len(dr)} of {len(legs)} "
                f"lanes left out: {', '.join(dr)}")
            env_ = {**env, 'BENCH': bench, 'NETS': ','.join(sub), 'DEST': dest}
            solve = os.path.join(dj, 'solve.json')
            run(['whole_solve.py', solve], {**env_, 'SOLVE_UNPROVED': '1'}, log_path=os.path.join(dj, 'solve.log'))
            for ln in grep(os.path.join(dj, 'solve.log'), r'whole_solve:|workers:'):
                say(ln[:200])
            if not os.path.isfile(solve):
                say("  no plan from the solve")
                continue
            # (its via ends onto the solve's layers, as a round's: laid on the fanout's own bench, the plan met stubs
            # still on the layers it had moved them off)
            bench_l = relayed(dj, bench, json.load(open(solve)), env_, say)
            loopd = os.path.join(dj, 'loop')
            with open(os.path.join(dj, 'loop.log'), 'w') as lf:
                rc_ = loop(solve, loopd, 6, env_, Log(lf))
            say(f"  loop exit {rc_} at {secs()} s")
            plan = os.path.join(loopd, 'plan.json') if rc_ == 0 else held_plan(loopd, env_)
            res = lay(f'partial{j}', dj, plan, plan_bench(loopd, plan, bench_l), sub) if plan else None
            if res is not None:
                return res
            say("  nothing laid")
        return None

    for r in range(1, R + 1):
        d = os.path.join(o, f'r{r}')
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'nets.lines'), 'w') as f:
            f.write(NETS.replace(',', '\n') + '\n')
        say(f"=== fanout round {r}")
        fenv = dict(env)
        # (the run's nets, every round: a fanout never re-picks them off a later round's board -- K51's second round,
        # re-picked, dropped its three pairs while their teeth stood on the board, and the solve found them split)
        fenv['FANOUT_NETS'] = NETS
        if os.path.isfile(HELD):
            fenv['FANOUT_PASSIVES_FIXED'] = '1'         # (joint_escape.passives_fixed: the cap step has moved them)
        if os.path.isfile(FB):
            fenv['FEEDBACK'] = FB
        if prev:
            fenv['INCREMENTAL'] = os.path.join(prev, 'fo.plan.json')
        rc = run(['fanout_from_plan.py', os.path.join(d, 'fo.kicad_pcb'), str(K), f'--board={base}'], fenv,
                 log_path=os.path.join(d, 'fo.log'))[0]
        say(f"  fanout exit {rc} at {secs()} s")
        for ln in grep(os.path.join(d, 'fo.log'), r'plan model'):
            say(ln[:220])
        if not os.path.isfile(os.path.join(d, 'fo.kicad_pcb')):
            # a board the FANOUT AUDIT refused (fanout_from_plan: a pair split by other lanes' ends) is fed back -- the
            # split pairs and the lanes between their tips, ends not to be chosen together again -- and another round,
            # incremental, frees those ends; the rounds end only when the refusal names nothing new
            refused = os.path.join(d, 'fo.refused.json')
            if os.path.isfile(refused) and os.path.isfile(os.path.join(d, 'fo.plan.json')):
                for ln in grep(os.path.join(d, 'fo.log'), r'^REFUSED'):
                    say('  ' + ln[:220])
                fbl = stripped(run(['whole_feedback.py', '--refused', os.path.join(d, 'fo.plan.json'), FB, refused],
                                   {**env, 'FB_ROUND': str(r)}, err=sys.stderr)[1])
                say(fbl)
                if not re.search(r'whole_feedback: [1-9]', fbl):
                    say("=== the fanout's refusal names nothing new -- the rounds end here")
                    break
                say("=== the fanout refused its board -- another round, incremental, frees the ends it named")
                base = advance(d, base)
                prev = d
                continue
            say("  no fanout board -- the rounds end here")
            break
        # the same ends as the round before (feedback it priced but did not follow): the rest of the round would be
        # the same -- so the feedback named in that round is raised (whole_feedback --raise: its ends' prices doubled;
        # the second time, the other ends of the lanes named too) and the fanout ALONE runs again, up to FANOUT_RAISES
        # times; the rounds end only when it still lays the same ends (zynq K42 and K44 on Linux: round 3 laid round
        # 2's ends again at the price escalated once, and the rounds ended with a net open)
        def same_ends():
            try:
                return json.load(open(os.path.join(d, 'fo.plan.json'))) == \
                    json.load(open(os.path.join(prev, 'fo.plan.json')))
            except Exception:
                return False
        if prev and same_ends():
            say(f"=== fanout round {r} laid round {r - 1}'s ends again")
            for k_ in range(1, FANOUT_RAISES + 1):
                fbr = stripped(run(['whole_feedback.py', '--raise', os.path.join(prev, 'fo.plan.json'), FB]
                                   + (['--widen'] if k_ == 2 else []), {**env, 'FB_ROUND': str(r - 1)},
                                   err=sys.stderr)[1])
                say('  ' + fbr)
                if not re.search(r'whole_feedback: [1-9]', fbr):
                    break
                os.replace(os.path.join(d, 'fo.log'), os.path.join(d, f'fo.same{k_}.log'))
                rc = run(['fanout_from_plan.py', os.path.join(d, 'fo.kicad_pcb'), str(K), f'--board={base}'], fenv,
                         log_path=os.path.join(d, 'fo.log'))[0]
                say(f"  fanout again, the feedback raised: exit {rc} at {secs()} s")
                for ln in grep(os.path.join(d, 'fo.log'), r'plan model'):
                    say(ln[:220])
                if not os.path.isfile(os.path.join(d, 'fo.kicad_pcb')) or not same_ends():
                    break
            if not os.path.isfile(os.path.join(d, 'fo.kicad_pcb')):
                say("  no fanout board -- the rounds end here")
                r -= 1
                break
            if same_ends():
                say(f"=== fanout round {r} laid round {r - 1}'s ends again, the feedback raised")
                r -= 1
                break
        bench = os.path.join(d, 'fo.kicad_pcb')
        if env.get('FANOUT_JOINT') and os.path.isfile(HELD):
            say('  caps: held where the first round\'s cap step left them -- the fanout went round them')
        elif env.get('FANOUT_JOINT'):
            # the parts the joint fanout lays through -- the movable passives under the arrays, as the chain's own
            # fanout does -- move off its copper now, ONCE, as the chain's cap nudge moves them after its fanout
            # (place_fanout_clearance), and the route holds them fixed where they are left: routed round them where
            # they stood, the zynq's bus on four layers met them under its lanes and its loop's layer cuts left no plan.
            # Where it leaves them is HELD: every later round starts from it and fans out round them
            import rules as _rules
            before = footprint_poses(bench)
            # (--beneath-only: a passive is nudged only where it stays beneath its BGA; one beside the array, in the
            # channel the bus leaves across, stays where it is and the fanout and the route go round it)
            rcn = run([os.path.join('..', 'py_router', 'place_fanout_clearance.py'), bench, bench,
                       '--clearance', str(_rules.active().clearance), '--beneath-only'], env,
                      log_path=os.path.join(d, 'caps.log'), own_process=True)[0]
            for ln in grep(os.path.join(d, 'caps.log'), r'^(Moved|Stuck|  Unresolved)'):
                say('  caps: ' + ln.strip()[:200])
            if rcn != 0:
                say(f"  caps: the nudge exited {rcn} -- the round goes on with them where the fanout left them")
            after = footprint_poses(bench)
            json.dump({r_: p_ for r_, p_ in sorted(after.items()) if before.get(r_) != p_}, open(HELD, 'w'),
                      indent=1)
            # and the fanout's own copper held to them where they now stand, as every later round holds it: a part
            # with nowhere beneath its BGA to go is left on it, and what no longer stands clear is planned again
            # round the parts (fanout_from_plan --hold)
            run(['fanout_from_plan.py', bench, '--hold'], {**env, 'FANOUT_PASSIVES_FIXED': '1'},
                log_path=os.path.join(d, 'hold.log'))
            for ln in grep(os.path.join(d, 'hold.log'), r'joint fanout of'):
                say(ln[:220])
        env.update(BENCH=bench, NETS=NETS, DEST=dest)
        solve = os.path.join(d, 'solve.json')
        # (a round never ends with nothing: its solve keeps a plan it cannot prove -- whole_solve SOLVE_UNPROVED -- and
        # the round lays it; the loop's own re-solves still take only a proved one)
        run(['whole_solve.py', solve], {**env, 'SOLVE_UNPROVED': '1'}, log_path=os.path.join(d, 'solve.log'))
        for ln in grep(os.path.join(d, 'solve.log'), r'whole_solve:|workers:'):
            say(ln[:200])
        J, proved, rc, res, loopd = {}, False, None, None, os.path.join(d, 'loop')
        if not os.path.isfile(solve):
            say("  no plan from the solve -- nothing for this round to lay")
        else:
            J = json.load(open(solve))
            proved = J.get('proved', True)
            bench = relayed(d, bench, J, env, say)
            if not proved:
                u = J.get('unproved') or {}
                say(f"  the solve's plan is UNPROVED ({u.get('over')} over two vias against a bound of "
                    f"{u.get('bound_over')}): laid all the same; its nets over two to the ends: {J.get('over_nets')}")
            with open(os.path.join(d, 'loop.log'), 'w') as lf:
                rc = loop(solve, loopd, 6, env, Log(lf))
            for ln in grep(os.path.join(d, 'loop.log'), r'^=== round|smooth:|NOT CONVERGING|passes'):
                say(ln[:170])
            say(f"  loop exit {rc} at {secs()} s")
            # a loop that did not pass is laid all the same, from the plan it held (held_plan): a lane it cannot lay
            # stays open, and the round's result stands against the others'
            plan = os.path.join(loopd, 'plan.json') if rc == 0 else held_plan(loopd, env)
            if rc != 0:
                say(f"  the loop did not pass -- laid from the plan it held: "
                    f"{os.path.relpath(plan, d) if plan else 'none'}")
            res = lay(r, d, plan, plan_bench(loopd, plan, bench)) if plan else None
            if plan and res is None:
                say("  no board laid from the plan (route_seq.log)")
        del env['BENCH']
        env.pop('VIA_ENDS', None)
        env.pop('BENCH0', None)
        if res is not None and (best is None or res['key'] < best['key']):
            best = res
        # done: a PROVED plan laid connected and clean -- whether or not its loop passed. A loop that did not pass
        # names ends its audits found crowded, and the lay went round them; a later round changes those ends and can
        # better only the vias or the copper, and on the synth handoff bench none did (11 of 117 cases went on after
        # such a round, each one to the same grade), while the zynq DDR's joint fanout, its second round laid clean,
        # spent 360 s on a third
        if proved and res is not None and res['open'] == 0 and res['drc'] == 0:
            break
        # ---- the next round's feedback: the ends its audits found crowded (a loop that did not pass), and the lanes
        # an unproved plan left over two vias, which the ends counted on keeping at two
        hots = []
        if rc is not None and rc != 0:
            for a in sorted(glob.glob(os.path.join(loopd, 'p*.audit'))) + \
                    sorted(glob.glob(os.path.join(loopd, 'q*.audit'))):
                j, h = a[:-len('.audit')] + '.json', a[:-len('.audit')] + '.fbhot.json'
                run(['whole_gate.py', j, a, '--hot', h], env)
                hots.append(h)
            hots += sorted(glob.glob(os.path.join(loopd, 'hs*.json')))     # the snapped plans' places
        new_over = add_over(FB, J.get('over_nets') if J and not proved else None)
        # (the round, so an end named again in a later one is raised, not added: whole_feedback)
        fbl = stripped(run(['whole_feedback.py', os.path.join(d, 'fo.plan.json'), FB] + hots,
                           {**env, 'FB_ROUND': str(r)}, err=sys.stderr)[1])
        say(fbl)
        if new_over:
            say(f"  the ends to count {', '.join(new_over)} over two vias: the solve could not keep them at two")
        nothing = 'whole_feedback: 0 new' in fbl and not new_over
        if (res is not None and res['open']) or (nothing and res is None):
            # a round that leaves nets OPEN names them to the fanout, whatever its audits found -- and one that lays
            # NOTHING (no plan from its solve, none its loop held, no board from its plan) with nothing new from its
            # audits names the lanes the ends model names on these ends (its nets over two, else its lanes loaded on
            # the trunk, else its most crossed): their ends freed and to be avoided -- an open net's at the end where
            # its trouble is, by the round's findings and its connectivity -- so the next round, incremental, chooses
            # them again. Neither ends the rounds (K41: 5 nets open, the audits silent, and the rounds ended there)
            fbn = stripped(run(['whole_feedback.py', '--name', os.path.join(d, 'fo.plan.json'), FB]
                               + (res['open_nets'] if res else []),
                               {**env, 'FB_ROUND': str(r), 'FB_HOTS': ','.join(hots),
                                'FB_CONN': os.path.join(d, 'conn.log')}, err=sys.stderr)[1])
            say(fbn)
            nothing = nothing and not re.search(r'whole_feedback: [1-9]', fbn)
        # nothing new for the fanout: the next round would lay the same ends from the same feedback
        if nothing:
            say("=== the feedback adds nothing new: another fanout lays the same ends")
            break
        base = advance(d, base)
        prev = d
    if best is None:
        best = last_resort(r)
    if best is None:
        g = f"WHOLE K={K} round={r} lanes=0/0 vias=0 copper=0mm connected=0 drc=0 secs={secs()} open={len(nets_all)}"
        say(g)
        return 2, g
    # the best round's board, where a caller finds it (route_bus, modal_whole): OUTDIR/best.kicad_pcb
    for ext in ('.kicad_pcb', '.kicad_pro'):
        src_ = best['board'][:-len('.kicad_pcb')] + ext
        if os.path.isfile(src_):
            shutil.copyfile(src_, os.path.join(o, 'best' + ext))
    g = re.sub(r'secs=\d+', f'secs={secs()}', best['grade'])
    say(('=== the best round: ' if best['open'] or best['drc'] else '') + g)
    return (0 if best['open'] == 0 and best['drc'] == 0 else 1), g


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('first', metavar='K|SOLVE', help="the rung on the bench's ladder, or with --loop the solve JSON")
    ap.add_argument('outdir', metavar='OUTDIR')
    ap.add_argument('rounds', metavar='ROUNDS', nargs='?', type=int,
                    help="the fanout rounds (default 3), or with --loop the loop's rounds (default 6)")
    ap.add_argument('--loop', action='store_true',
                    help='the loop alone on a solve, the bench from BENCH / NETS / DEST as every whole_* stage reads it')
    ap.add_argument('--inproc', action='store_true',
                    help='every stage in this process rather than each in its own')
    a = ap.parse_args()
    global INPROC
    INPROC = a.inproc
    if a.loop:
        sys.exit(loop(a.first, os.path.abspath(a.outdir), a.rounds or 6, dict(os.environ), Log()))
    sys.exit(chain(a.first, a.outdir, a.rounds or 3)[0])


if __name__ == '__main__':
    main()
