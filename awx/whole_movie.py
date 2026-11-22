#!/usr/bin/env python3
"""whole_movie.py RUNDIR OUT.mp4 -- a film of one whole-route run (#622), from the ends to the final copper.

RUNDIR is whole_route.py's OUTDIR, or a bus step's (route_bus.py's BUS.bus/run: the settings of its run are read from
the step's run.json, and what the step gave its chain over them from the chain's own chain.json); the round that
routed (rN/seq.kicad_pcb) is filmed. It draws a run on any number of layers, each copper layer a colour of its own on
the board, in the braid and in the 2-D geometry (F.Cu red, B.Cu blue, an inner layer bright: layer_col); on more than
two a legend in the header names the layers the lanes stand on.

A bus step's run whose ends are its GROUP'S (route_bus under GROUP_ENDS: the group's directory, beside the bus's own
run in SRC-DEST.bus/) opens with the group's ends, each array in a magnifier of its own: the BUSES the step found and
its group, and each bus's ends model CAPTURED (the step's events, OUT.bus/events.jsonl); THE ONE MODEL of the arrays'
fanout and every bus's ends -- the balls by what serves them (a bus's lane, another signal's escape, a plane's drop or
strap: group.json), the menus by tier, the pattern first and widened where no state holds, and the START, its every
lane end drawn as the model held it (group.log; checks.jsonl's states, each end's escape from its own signature); THE
SEARCH, its every round's best on the model's own clock (the step's events) beside the state each solve ended on and
the ends that moved since the one before (checks.jsonl); THE CHECK by each bus's own whole solve (its verdict and its
console), the field's findings at the points they were found, THE REPAIR -- the lanes it freed, and why -- and the
check again; and THE FIELD as laid (shared_field.kicad_pcb over prepared.kicad_pcb: the fanout's copper, the parts
the nudge moved, the field's ties). Those scenes run nothing again: the model's CP-SAT runs on many workers, which no
re-run reproduces byte for byte, so every number and every end they draw is the run's own record. The coda closes on
the group's escalation as the step recorded it: a group left short of clean routed again a rung down the fab ladder,
or, where its grade is clean and no net was delivered below the sizes asked for, that no rung was needed.

Then the bus's own run, in order, on the board -- the ENDS (its teeth and berths: the fanout's, or on the group's field
the field's, and the two orders they fix), the TAUT STRINGS (each lane a string from its tooth to its berth, pulled
tight round the copper), the FRAME (the trunk and the rings: the one route coordinate u, and each string read in it as
a reference), the SOLVE, the GEOMETRY, the polish, the snap (the pairs first, then the singles), the audit and the gate
(and what a failing round sends back), and the LAY (the router's copper, and its wall guard where the lay's log names a
walled ball) -- and the solvers' own decision spaces:

  THE SOLVE (CP-SAT) as a BRAID under the board. Its x is the route coordinate u, which on the trunk IS distance
  along the board (the canonical frame runs source to destination along +x), so the braid shares the board's x
  axis; its y is each lane's ORDER, north to south; the colour is the layer. The solve's variables are where each
  crossing and each layer change sits along u, and its rules can be seen there: two strands that cross differ in
  colour, a white ring is a via (a colour change). The search is replayed from the plans CP-SAT really found, in
  the order it found them, with the bound that proved the last one (or, a plan it kept unproved, said so). On the
  board above, the same plan is drawn in the references' slots (each column's reference offsets, handed out in the
  solve's order): the solve knows order and layers, not millimetres, and the lanes swap exactly where it put each
  crossing.

  THE 2-D GEOMETRY (whole_embed.py, in a run whose loop ran it: its eN.json beside the frame geometry's gN.json) in
  board xy, which is its own space. The frame geometry's layout is then only its START, one short scene: the layout,
  and in red the rules its LP paid. Then the CELLS (each layer's free space cut into triangles on the fixed copper's
  corners, and the vias' own); the lanes' own POINTS (the start read onto them: a point on every cell edge a run
  crosses, free points between); the ROUNDS (the layout after every round's LP, the points that LP held -- it shrinks
  to what still moves -- the places that moved, and each round's events as bars); a closer look at ONE EVENT of each
  kind the rounds had (a via changing cell, a lane's point gone on into the next cell, a run pulled taut, a via slid
  along its lane to a more open place, and a breach where the plan has one); and WHAT STANDS at the end (the
  shortfalls, the margin given up, the folds cleared from the written lines, the cuts for the solve).

  THE FRAME GEOMETRY (whole_geo.py's LP) as FORCES on the board, in a run whose loop had no 2-D geometry. The LP's
  dual gives every rule a SHADOW PRICE; the rules with a price are the ones holding a lane where it is -- a same-layer
  neighbour at the bar, a via's room, a pad box or the board edge, an island's side, a turn limit -- and the price is
  how hard they push. A cursor column shows one column of the LP: its variables are those lanes' offsets, in the
  braid's order at the same u.

The stages are OBSERVED, never re-implemented: the round's root solve, its geometry and its 2-D geometry are run again
in-process under the chain's own environment -- CP-SAT with a solution callback; whole_geo under a profile hook that
keeps each LP pass's matrix, row tags and the dual HiGHS already solves, and, where that re-run relaxes the taut
strings itself (no memo of them on disk: a bus step's run), every string as it stood after rounds of its relaxation;
whole_embed under one that keeps its cells, the layout it starts from and, round by round, the layout, what the
round's LP held, what moved and every event (each count checked against the stage's own). Each re-run must write what
the chain wrote, byte for byte, or the film is refused (--force films it anyway, and says so on screen). The round's
first solve is run on the fanout's own board and its geometry on the board the loop's round stood on (the loop's
benches.json: on more than two layers, the fanout's with its via ends moved onto the plan's layers). A NEGOTIATED bus
(route_bus JOINT_DIR, its group's buses routed together) is re-run as its chain ran it: its first solve as the bus
solved it alone, compared with solve.solo.json -- the coupled solve wrote its own plan for the bus over solve.json, and
a scene of its own shows that plan, the one the loop laid out -- and its loop's stages with the other buses' published
lanes on the board (joint_lanes.foreign), the first solve with those of the buses already ended. The film then shows
the NEGOTIATION too, read from the step's records and never run again (load_negotiation): every bus of the group on one
clock (each one's fanout, first solve, wait for the coupled solve, loop, lay and end, from the step's events), the
coupled solve's couplings at their places (the ones the buses' own plans broke, the ones the coupled plan still breaks),
the others laying their copper while the filmed bus's loop runs -- under its later scenes, faint, from then on -- and,
before its result, every bus's copper on the merged board together, each in a tint of its own, with its numbers. The
traces are
kept in RUNDIR/rN/movie/, and a re-run writes nothing else: no event of its own is appended to the step's events
file (BUS_EVENTS is not given it). --check stops there: it says whether each re-run reproduced the chain's file (exit 1 when
one did not), for a run on any number of layers.

What is drawn BETWEEN two recorded states -- two plans, two rounds of the strings' relaxation or of the 2-D geometry,
before and after an event -- is a blend of the two, not a step of any stage. And where the strings came from a memo,
their relaxation is not filmed at all: the move from each straight line to its string is a blend, and the scene says
so.

With --sound the film gets a sound track (whole_movie_sound.py): a NARRATION of every scene, from the scene's own lines
(Scene.says, beside its captions, every number in them read from the run's files as the captions' are), EFFECTS on
the things the scenes draw (Scene.cues: a plan found, a bound raised, a round of a search, a via changing cell, a lane
laid ..), each at the moment its scene draws it, and MUSIC, all made here (numpy, scipy) and mixed by ffmpeg, the
music ducked under the voice. The film's timeline is FITTED to the narration: a scene's line starts at its moment of
the scene, and where a line is longer than the scene gives it the scene is slowed up to its next line (or to where
what it speaks of leaves the screen), its last frame held after its last, so no line runs into the line or the scene
after it. --scenes with --sound lists every line's and effect's time on the fitted timeline. The voice (--voice) is by
default the neural Kokoro-82M's kokoro:am_puck (whole_movie_sound.DEFAULT_VOICE): its first use installs it into a
venv of its own in the user's cache ($XDG_CACHE_HOME or ~/.cache, krt_tts) and fetches its weights there once, and it
then speaks offline; `python3 whole_movie_sound.py --fetch [kokoro:NAME]` does that ahead of time, for a machine that
will film offline. Where it cannot be had the film stops, saying how to fetch it -- or --voice NAME, a macOS `say`
voice (`say -v ?` lists them), needs nothing installed. Without --sound the film is the same as ever, frame for frame,
and nothing of the sound is needed.

With --short [SECS] the run is not filmed but cut into a SHORT of about SECS (15) seconds: a few of its moments,
each an excerpt of the film cut tighter and drawn as the film draws that stage, all at ONE framing of the bus (the
film's own, its source array to its destination) -- no words, no captions, no title or end card, something moving in
every frame. Which moments play, and for how long, is read from the run's own records (short_clips), so a run gives
the same short every time; its sound is the film's own music voice over a slow pulse the cuts fall on, and the film's
own effects (whole_movie_sound.short_score).

usage: whole_movie.py RUNDIR OUT.mp4 [--fps 30] [--workers 6] [--stills DIR [--at T1,T2,..]] [--human BOARD]
                     [--speed X] [--force] [--scenes] [--sound [--voice kokoro:NAME | SAY_VOICE] | --short [SECS]]
       whole_movie.py RUNDIR --check
The bench is the chain's: BASE (default fb_t2q_pairs.kicad_pcb) and DEST (default DU1) as whole_route.py reads them; a
bus step's run gives its own, and BASE may then name the board from before the bus's ends were laid (the ends' scene
shows them laid). Needs ffmpeg (libx264) for the mp4, on the PATH or under /opt/homebrew or /usr/local; --stills alone
writes PNGs.
"""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import collections
import contextlib
import functools
import io
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PYR = os.path.normpath(os.path.join(HERE, '..', 'py_router'))


# =============================================================================== tracing (runs in a subprocess)
def bus_step(rundir):
    """the record of the bus step (route_bus.py) RUNDIR is the run of -- its BUS.bus/run -- or None: the step's
    run.json (run_trace.provenance: the settings in force for the run -- its rules, its routing layers, its caches off --
    and its destination), with `work` its directory"""
    work = os.path.dirname(os.path.abspath(rundir))
    rj = os.path.join(work, 'run.json')
    if not (os.path.isfile(rj) and os.path.isfile(os.path.join(work, 'frame.kicad_pcb'))):
        return None
    return dict(json.load(open(rj)), work=work)


def chain_record(rundir):
    """what the chain of RUNDIR was asked (whole_route.chain's chain.json: its bench, its destination, and the settings
    given it over the environment), or None -- a run from before the chain kept it"""
    p = os.path.join(os.path.abspath(rundir), 'chain.json')
    return json.load(open(p)) if os.path.isfile(p) else None


def round_bench(round_dir, i):
    """the bench the loop's round i stood on (the loop's benches.json): the fanout's own, or -- a run on more routing
    layers than two -- that bench with its via ends moved onto the round's plan's layers. Looked for in this round's
    directory first, so a run copied elsewhere is read where it now is"""
    fo = os.path.join(round_dir, 'fo.kicad_pcb')
    try:
        b = json.load(open(os.path.join(round_dir, 'loop', 'benches.json'))).get(str(i))
    except (OSError, ValueError):
        return fo
    if not b:
        return fo
    for d in (round_dir, os.path.join(round_dir, 'loop')):
        if os.path.isfile(os.path.join(d, os.path.basename(b))):
            return os.path.join(d, os.path.basename(b))
    return b if os.path.isfile(b) else fo


def solo_ref(round_dir):
    """the file the round's first solve wrote: its solve.json -- or, a NEGOTIATED bus's round (route_bus JOINT_DIR),
    solve.solo.json, where the coupled solve (whole_joint.py) kept it before writing its own plan for the bus over
    solve.json"""
    sj, solo = os.path.join(round_dir, 'solve.json'), os.path.join(round_dir, 'solve.solo.json')
    try:
        coupled = bool(json.load(open(sj)).get('joint'))
    except (OSError, ValueError):
        coupled = False
    return solo if coupled and os.path.isfile(solo) else sj


def joint_finished(round_dir, events, jd):
    """(a negotiated bus) the other buses whose whole route had ended (JOINT_DIR/BUS.finished) when the round's first
    solve started -- by the step's events (its solve's start), else by the round's fanout board"""
    log = os.path.join(os.path.abspath(round_dir), 'solve.log')
    t0 = None
    if events and os.path.isfile(events):
        for ln in open(events):
            try:
                e = json.loads(ln)
            except ValueError:
                continue
            if e.get('stage') == 'solve' and e.get('kind') == 'start' and os.path.abspath(e.get('log') or '') == log:
                t0 = e['t']
    if t0 is None:
        t0 = os.path.getmtime(os.path.join(round_dir, 'fo.kicad_pcb'))
    return sorted(f[:-len('.finished')] for f in os.listdir(jd)
                  if f.endswith('.finished') and os.path.getmtime(os.path.join(jd, f)) < t0)


def _chain_env(round_dir, dest, bench=None):
    """the environment whole_route.py gives a stage of this round: a bus step's round under that step's settings, and
    under what the step gave its chain over them (the chain's own record). `bench`: the board the stage reads (a loop
    round's: round_bench), the fanout's own when not given -- the round's first solve reads that one. A negotiated
    bus's stages (JOINT_DIR) read the other buses' lanes as they published them (joint_lanes.foreign): the loop's
    every one, the round's first solve only those of the buses ended when it started (whole_route.chain)"""
    nets = [ln.strip() for ln in open(os.path.join(round_dir, 'nets.lines')) if ln.strip()]
    env = dict(os.environ)
    rundir = os.path.dirname(os.path.abspath(round_dir))
    step = bus_step(rundir)
    if step:
        # (...and BASE the board the step handed its chain, whatever board the film's ends scene is given)
        env.update(step.get('settings') or {}, BASE=os.path.join(step['work'], 'frame.kicad_pcb'))
    rec = chain_record(rundir)
    if rec:
        # (a bus step gives its chain more than its own environment holds -- the bus's fanout mode, its end layers
        # held or free: the solve reads them)
        env.update(rec.get('settings') or {})
        if rec.get('base'):
            env['BASE'] = rec['base']
    if env.get('JOINT_DIR') and env.get('JOINT_BUS'):
        if bench is None:
            env.update(JOINT_FOREIGN='finished',
                       JOINT_FINISHED=','.join(joint_finished(round_dir, env.get('BUS_EVENTS'), env['JOINT_DIR'])))
        else:
            env['JOINT_FOREIGN'] = 'all'
    # (and nothing of a re-run's appended to the step's own record of its run, its events file: bus_events writes
    # nothing without BUS_EVENTS)
    for k in ('HINT', 'CUTS', 'HIST', 'GEO_FLIPS_FROM', 'SEED_FLIPS', 'SEED_CUTS', 'SEED_HIST', 'SNAP_KEEP',
              'BUS_EVENTS', 'BUS_EVENTS_BUS'):
        env.pop(k, None)
    env.update(OMP_NUM_THREADS='1', VECLIB_MAXIMUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               TAUT_MEMO=env.get('TAUT_MEMO', '1'),
               BRAID_EXACT_PAGES='0', STAGE_CACHE=env.get('STAGE_CACHE', '1'),
               BENCH=bench or os.path.join(round_dir, 'fo.kicad_pcb'), NETS=','.join(nets), DEST=dest)
    return env


def _ix(v):
    return v.Index() if callable(getattr(v, 'Index', None)) else v.index


def _trace_solve(round_dir, out):
    """the round's ROOT solve (its solve.json) run again with every plan CP-SAT finds kept, and the model's own
    reading of the bench (the crossing windows, the face band, the built-in via cuts) from solve()'s locals"""
    sys.path.insert(0, HERE)
    # (a round's first solve, as whole_route.py runs it: a plan it cannot prove is kept, and laid all the same)
    os.environ.update(SOLVE_UNPROVED='1', SOLVE_CROWD='1')
    from ortools.sat.python import cp_model
    runs = []
    orig = cp_model.CpSolver.Solve

    class Plans(cp_model.CpSolverSolutionCallback):
        def __init__(self, model, run):
            super().__init__()
            self.vs = [model.GetIntVarFromProtoIndex(i) for i in range(len(model.Proto().variables))]
            self.run = run

        def on_solution_callback(self):
            # (every variable's value, kept compact: a search of an eight-layer bus finds its plans by the hundred, each
            # of a model of many thousand variables)
            self.run['plans'].append({'t': self.WallTime(), 'obj': self.ObjectiveValue(),
                                      'bound': self.BestObjectiveBound(),
                                      'x': np.fromiter((self.Value(v) for v in self.vs), np.int64, len(self.vs))})

    def observed(self, model, solution_callback=None):
        # (who asks: the search's own runs, or -- on more layers than two -- the plane tie, which chooses a found
        # plan's layers again by the planes they split, its crossings and changes held: no plan of the search)
        run = {'plans': [], 'log': [], 'who': sys._getframe(1).f_code.co_name}
        runs.append(run)
        passed = self.log_callback

        def log(line):
            run['log'].append(line)
            if passed is not None:
                passed(line)
        self.log_callback = log
        t0 = time.time()
        status = orig(self, model, Plans(model, run))
        run['secs'] = time.time() - t0
        return status
    cp_model.CpSolver.Solve = observed
    got = {}

    def hook(frame, event, arg):
        if event == 'return' and frame.f_code.co_name == 'solve' and frame.f_code.co_filename.endswith('whole_solve.py'):
            got.update(frame.f_locals)
    import whole_ctx
    import whole_solve
    ctx, _cs = whole_ctx.plan()
    sys.setprofile(hook)
    try:
        J = whole_solve.solve(ctx, os.environ['DEST'])
    finally:
        sys.setprofile(None)
    if J is None:
        raise SystemExit('whole_movie: the root solve proved no plan')
    J = json.loads(json.dumps(J))
    ref = json.load(open(solo_ref(round_dir)))
    L = got
    Gs = whole_solve.G
    LAYN = list(L.get('LAYN') or ('F.Cu', 'B.Cu'))
    YL = L.get('YL') or {}

    def decode(x):
        x = [int(v) for v in x]
        cross = {f'{a}|{b}': x[_ix(v)] * Gs for (a, b), v in L['t'].items()}
        changes = {n: [x[_ix(cv)] * Gs for cv, av in zip(*L['chg'][n]) if x[_ix(av)]] for n in L['M']}
        # (more layers than two: each run's layer between the changes -- whole_solve's YL, read as its plan writes
        # 'layers'; on two a lane's layer is one bit, flipped by each change)
        layers = {n: [LAYN[x[_ix(YL[n][k])]] for k in range(len(changes[n]) + 1)] for n in L['M'] if n in YL}
        return cross, changes, int(sum(x[_ix(L['over'][n])] for n in L['M'])), layers
    out_runs = []
    for i, r in enumerate(runs):
        plans = []
        for p in r['plans']:
            cross, changes, over, layers = decode(p['x'])
            plans.append(dict({'t': p['t'], 'obj': p['obj'], 'bound': p['bound'], 'cross': cross, 'changes': changes,
                               'over': over, 'vias': sum(len(v) for v in changes.values())},
                              **({'layers': layers} if layers else {})))
        out_runs.append({'workers': 'bound-raising' if i == 0 else 'fallback', 'who': r['who'], 'plans': plans,
                         'secs': r.get('secs', 0.0), 'log': [ln for ln in r['log'] if ln.startswith('#')]})
    fl = lambda d: {k: float(v) for k, v in d.items()}
    res = {'identical': json.dumps(J, sort_keys=True) == json.dumps(ref, sort_keys=True),
           'final': J, 'runs': out_runs, 'G': Gs, 'W_V': whole_solve.W_V, 'W_OVER': L['W_OVER'],
           'KMAX': whole_solve.KMAX, 'M': list(L['M']), 'launch': list(L['Ln']), 'final_order': list(L['Fn']),
           'windows': {f'{a}|{b}': [float(v[0]), float(v[1])] for (a, b), v in L['win'].items()},
           'entry': fl(L['entry']), 'end': fl(L['end']), 'tend': fl(L['tend']), 'tl': L['tl'], 'dl': L['dl'],
           'branch': dict(L['bname']), 'Hn': fl(L['Hn']), 'BAND': float(L['BAND']), 'S_FACE': float(L['S_FACE']),
           'vcuts': [{'lane': c['lane'], 'u': float(c['u']), 'w': float(c['w'])} for c in L['VCUTS']],
           'pairs': sorted(n for n in (L['prs'] or {}) if n in L['M']), 'xo': sorted(L['XO']), 'SV': L['SV'],
           'triples': int(L['nt'])}
    if YL:
        # (more layers than two: the routing layers, and each lane's first and last run's layer by name)
        res.update(layn=LAYN, tooth_layer={n: LAYN[int(L['tli'][n])] for n in L['M']},
                   dest_layer={n: LAYN[int(L['dli'][n])] for n in L['M']})
    json.dump(res, open(out, 'w'))
    print(f'whole_movie: root solve traced -- {sum(len(r["plans"]) for r in out_runs)} plan(s), identical to the '
          f'chain: {res["identical"]}')


def _trace_geo(round_dir, solve_path, flips, gref, out):
    """the round's geometry (whole_geo on its solve and flips) run again under a profile hook: each LP pass's
    matrix, row tags, primal and DUAL (every rule's shadow price), the static sides, and the frame it built"""
    sys.path.insert(0, HERE)
    import filecmp
    import runpy
    import numpy as np
    passes, sides, duals = [], [], []
    relax, nround = [], {}

    def hook(frame, event, arg):
        co = frame.f_code
        if event == 'return' and co.co_name == '_round_b' and co.co_filename.endswith('taut_fast.py'):
            # the taut strings' own relaxation, where this process runs it (no memo of them on disk): every string
            # still moving, after rounds 1 to 4, each power of two and each 25th of a spacing's rounds
            lv = frame.f_back
            tp = lv.f_back.f_back if lv is not None and lv.f_code.co_name == '_level_many' and lv.f_back is not None else None
            if tp is not None and tp.f_code.co_name == 'taut_paths':
                L = lv.f_locals
                k = nround[(id(lv), L['step'])] = nround.get((id(lv), L['step']), 0) + 1
                if k <= 4 or k & (k - 1) == 0 or k % 25 == 0:
                    names, B = [t_[0] for t_ in tp.f_locals['todo']], L['B']
                    relax.append([float(L['step']), k, {names[s_]: [[round(float(x), 4), round(float(y), 4)] for x, y in
                                                                    simplify(B.P[B.off[i]:B.off[i + 1]], 0.002)]
                                                        for i, s_ in enumerate(L['active'])}])
            return
        if not co.co_filename.endswith('whole_geo.py'):
            return
        if event == 'call' and co.co_name == 'build_and_solve':
            g = frame.f_globals
            if not getattr(g['linprog'], '_observed', False):
                lp = g['linprog']

                def observed(*a, **k):
                    r = lp(*a, **k)
                    duals.append(np.array(r.x) if r.status == 0 else None)
                    return r
                observed._observed = True
                g['linprog'] = observed
        elif event == 'return' and co.co_name == 'build_and_solve' and arg is not None:
            L = frame.f_locals
            passes.append(dict(A=L['Aub'], b=np.asarray(L['rhs'], float), tags=list(L['tags']), var=dict(L['var']),
                               x=np.asarray(L['res'].x), sol=arg, dual=duals[-1] if duals else None))
        elif event == 'return' and co.co_name == 'static_sides':
            sides.append(arg)
    GEO = os.path.join(HERE, 'whole_geo.py')
    tmp = out + '.g.json'
    sys.argv = [GEO, solve_path, tmp]
    if flips:
        os.environ['GEO_FLIPS_FROM'] = flips
    buf = io.StringIO()
    sys.setprofile(hook)
    try:
        with contextlib.redirect_stdout(buf):
            G = runpy.run_path(GEO, run_name='__main__')
    finally:
        sys.setprofile(None)
    identical = filecmp.cmp(tmp, gref, shallow=False)
    FR, PIECE, Fr, Gc = G['FR'], G['PIECE'], G['Fr'], G['G']
    svx = G['spine_xy_vec']
    cols = {}

    def col(f, k):
        if (f, k) not in cols:
            s = k * Gc
            X, Y = svx(FR[f]['sp'], s, np.array([0.0, 1.0]))
            cols[(f, k)] = {'f': f, 'k': k, 'u': float(FR[f]['u'](s)), 'b': [float(X[0]), float(Y[0])],
                            'n': [float(X[1] - X[0]), float(Y[1] - Y[0])]}
        return cols[(f, k)]
    route_key = lambda fk: (0 if fk[0] == 'T' else 1, fk[1])
    out_passes = []
    for P in passes:
        o = P['sol']['o']
        lanes = collections.defaultdict(list)
        for (f, n, k), v in o.items():
            col(f, k)
            lanes[n].append([f, k, float(v)])
        for n in lanes:
            lanes[n].sort(key=lambda e: route_key(e))
        A, Ar, b, x, y = P['A'].tocsc(), P['A'].tocsr(), P['b'], P['x'], P['dual']
        slack = b - Ar @ x
        best = {}
        for e, tg in P['tags']:
            rr = A.indices[A.indptr[e]:A.indptr[e + 1]]
            if not len(rr):
                continue
            r = int(rr[0])
            price = float(y[r]) if y is not None else 0.0
            key = repr(tg)
            cur = best.get(key)
            if cur is None or price > cur['price']:
                best[key] = {'tag': tg, 'r': r, 'price': price, 'slack': float(slack[r]),
                             'paid': max(float(x[e]), cur['paid'] if cur else 0.0)}
            else:
                cur['paid'] = max(cur['paid'], float(x[e]))

        def lane_pt(f, n, k):
            v = o.get((f, n, k))
            if v is None:
                return None
            c = col(f, k)
            return [c['b'][0] + v * c['n'][0], c['b'][1] + v * c['n'][1]]
        rows = []
        for R in best.values():
            if R['price'] < 1e-7 and R['paid'] < 1e-4:
                continue
            tg = R['tag']
            kind, f, k = tg[0], tg[1], int(tg[2])
            what, wall = '', 0
            if kind in ('pitch', 'via', 'viavia'):
                lanes_ = [tg[3], tg[4]]
                pts = [lane_pt(f, tg[3], k), lane_pt(f, tg[4], k)]
            elif kind in ('bound', 'static'):
                # a bound on one offset: the lane stands AT it when it binds -- the wall is on the side the
                # coefficient says (+1: an upper bound, the wall at larger offsets)
                n = tg[3]
                j = P['var'].get((f, n, k))
                coef = float(Ar[R['r'], j]) if j is not None else 0.0
                lanes_ = [n]
                pts = [lane_pt(f, n, k)]
                what = str(tg[4])
                wall = (1 if coef > 0 else -1) if coef else 0
            else:                                   # slope, turn, pturn, pdive, approach: a lane's own shape
                lanes_ = [tg[3]]
                pts = [lane_pt(f, tg[3], k)]
            if not pts or pts[0] is None:
                continue
            rows.append({'kind': kind, 'f': f, 'k': k, 'lanes': lanes_, 'price': R['price'], 'paid': R['paid'],
                         'slack': R['slack'], 'pts': [p for p in pts if p is not None], 'what': what,
                         'wall': wall, 'nv': col(f, k)['n']})
        out_passes.append({'lanes': lanes, 'rows': rows, 'nvar': len(P['var']), 'nrow': int(len(b)),
                           'paid': {k: len(v) for k, v in P['sol']['paid'].items()}})
    # each lane's reference offset (the LP's bound picks the free interval nearest it) at every column either pass
    # has -- the second pass starts a ring piece where its trunk ended (whole_geo.reanchor)
    refs = {}
    for P in passes:
        for (f, n, k) in P['var']:
            refs.setdefault(n, {})[(f, k)] = float(PIECE[(f, n)]['ref'](k * Gc))
    refs = {n: sorted([[f, k, o] for (f, k), o in v.items()], key=route_key) for n, v in refs.items()}
    side_of = {}
    for (f, n, k, lo_, hi_, side, what) in (sides[-1] if sides else []):
        e = side_of.setdefault((f, n, what), {'f': f, 'lane': n, 'island': what, 'side': side, 'k0': k, 'k1': k})
        e['k0'], e['k1'] = min(e['k0'], k), max(e['k1'], k)
    ctx = G['ctx']
    Mf = getattr(ctx, 'M', None)
    aff = None
    if Mf is not None:
        o0, ex, ey = Mf(0.0, 0.0), Mf(1.0, 0.0), Mf(0.0, 1.0)
        aff = [list(map(float, o0)), [float(ex[0] - o0[0]), float(ex[1] - o0[1])], [float(ey[0] - o0[0]), float(ey[1] - o0[1])]]
    P2 = lambda p: [float(p[0]), float(p[1])]
    res = {'identical': identical, 'flips': flips, 'G': Gc, 'cols': list(cols.values()), 'passes': out_passes,
           'refs': refs, 'sides': list(side_of.values()),
           'islands': [{'box': [float(s[0]), float(s[1]), float(s[2]), float(s[3])], 'layers': sorted(s[4]),
                        'label': s[5], 'own': s[6] if len(s) > 6 else None} for s in G['STATIC']],
           'M': list(G['M']), 'cls': dict(G['cls']), 'pairs': {n: list(v) for n, v in G['prs'].items() if n in G['M']},
           'tooth': {n: P2(v[0]) for n, v in G['TERM'].items()}, 'berth': {n: P2(v[1]) for n, v in G['TERM'].items()},
           'start': {n: P2(Fr.start[n]) for n in G['M']}, 'land': {n: P2(Fr.land[n]) for n in G['M']},
           'spine': [P2(p) for p in Fr.spine.P], 'rings': {k: [P2(p) for p in r.P] for k, r in Fr.rings.items()},
           'ring_u': {f: [float(G['HK'][f]), float(G['S0C'][f])] for f in G['S0C']},
           'SB': list(map(float, Fr.SB)), 'DB': list(map(float, Fr.DB)), 'H0': float(Fr.H0),
           'Hk': {k: float(v) for k, v in Fr.Hk.items()}, 'src': Fr.src, 'dst': Fr.dst,
           'chi': int(getattr(ctx, 'chi', 1) or 1), 'aff': aff,
           'rules': {'P_MIN': float(G['P_MIN']), 'P_COMF': float(G['P_COMF']), 'TW': float(G['TW']),
                     'CL': float(G['CL']), 'VIA_R': float(G['VIA_R']), 'PP': float(G['PP']),
                     'grid': float(ctx.cfg.grid_step), 'via': float(ctx.cfg.via_size)},
           'log': [ln for ln in buf.getvalue().splitlines() if 'joint LP' in ln or ln.startswith(('pass ', 'cuts', '  PAID'))],
           # each lane's TAUT STRING as every stage reads it (braid.setup's, in the plan's frame): the frame's
           # references are these, read in its own coordinates
           'strings': {n: [P2(p) for p in ctx.paths[n]] for n in G['M'] if n in ctx.paths},
           # ...and their relaxation as this re-run ran it: [spacing, round, {string: its points}] in turn (empty
           # where the strings came from a memo on disk)
           'relax': [[st_, k_, {n: v for n, v in d_.items() if n in G['M']}] for st_, k_, d_ in relax]}
    json.dump(res, open(out, 'w'))
    print(f'whole_movie: geometry traced -- {len(passes)} LP pass(es), identical to the chain: {identical}')


def _trace_embed(round_dir, solve_path, gref, eref, out):
    """the round's 2-D geometry (whole_embed on its solve, from the frame geometry's layout gref) run again under a
    profile hook: its cells, the layout it starts from on its own points, and for every round what its LP held, what
    moved, the layout after it and its events -- a via's change of cell, a lane's point gone on into the next cell,
    a run pulled taut -- and, between the rounds, each via slid along its lane"""
    sys.path.insert(0, HERE)
    import filecmp
    import runpy
    EMB = os.path.join(HERE, 'whole_embed.py')
    tmp = out + '.e.json'
    sys.argv = [EMB, solve_path, tmp, '--ref=' + gref]
    r4 = lambda p: [round(float(p[0]), 4), round(float(p[1]), 4)]
    r3 = lambda p: [round(float(p[0]), 3), round(float(p[1]), 3)]
    nm = lambda key: f'{key[0]}|{key[1]}'
    rec = {'rounds': [], 'slides': [], 'shape': {}}
    held = {}

    def stand(g):
        """the layout as it stands: each run's line and its points' kinds (P on a cell edge, F free, X either side of
        a breach, D the end of a pair's dive: rigid with its via), each via and its cell of the vias' space -- and a
        pair's via as the stage holds it: its barrels' places (a crossover's on either side, the side not yet chosen)"""
        PL = g['layout']()
        VIA_, VH_ = g['VIA'], g['VH']
        return {'xy': {nm(k): [r4(p) for p in v] for k, v in PL.items()},
                'kind': {nm(k): ''.join(p['k'] for p in g['RUN'][k]['pts']) for k in g['KEYS']},
                'via': {nm(k): r4(v) for k, v in VIA_.items()},
                'cell': {nm(k): int(t) for k, t in g['TV'].items()},
                'barrel': {nm(k): [r4((VIA_[k][0] + o[0], VIA_[k][1] + o[1])) for o, kd in g['via_discs'](k) if kd == 'b']
                           for k in VIA_ if k in VH_}}

    def hook(frame, event, arg):
        if event != 'call' and event != 'return':
            return
        co = frame.f_code
        name = co.co_name
        if name not in ('round_of', 'pull_runs', 'slide_vias', 'shape_pass') or not co.co_filename.endswith('whole_embed.py'):
            return
        g = frame.f_globals
        RUN = g['RUN']
        cells = lambda vk: [int(g['TV'][vk]), int(RUN[vk]['last']), int(RUN[(vk[0], vk[1] + 1)]['first'])]
        if name == 'round_of' and event == 'call':
            if 'start' not in rec:
                rec['start'] = stand(g)
            # (the free points themselves are held, each with its cell and place: a point is the same object from
            # round to round, so one whose cell has changed when the round's moves are made went on into the next)
            held.update(pts=[(k, p, p['tri'], tuple(p['xy'])) for k in g['KEYS'] for p in RUN[k]['pts'] if p['k'] == 'F'],
                        kind={k: [p['k'] for p in RUN[k]['pts']] for k in g['KEYS']},
                        via=dict(g['VIA']), cells={vk: cells(vk) for vk in g['VKEYS']}, bulge=[], pull=[], places=[])
        elif name == 'pull_runs' and event == 'call':
            held['bulge'] = [{'run': nm(k), 'L': int(RUN[k]['L']), 'was': r4(q), 'xy': r4(p['xy']), 'from': int(t0),
                              'to': int(p['tri'])} for k, p, t0, q in held['pts'] if p['tri'] != t0]
            held['before'] = g['layout']()
        elif name == 'pull_runs' and event == 'return' and arg is not None:
            now = g['layout']()
            held['pull'] = [{'run': nm(k), 'L': int(RUN[k]['L']), 'before': [r4(p) for p in held['before'][k]],
                             'after': [r4(p) for p in now[k]]} for k in g['KEYS'] if now[k] != held['before'][k]]
            held['places'] = [r4(p) for p in arg]
        elif name == 'round_of' and event == 'return' and arg is not None:
            L = frame.f_locals
            # (own: a pair's via, whose cell is one of its OWN space -- the places its two barrels and its straight run
            # all stand clear -- not of the vias')
            hops = [{'via': nm(vk), 'from': r4(held['via'][vk]), 'to': r4(g['VIA'][vk]), 'was': held['cells'][vk],
                     'now': cells(vk), 'own': vk in g['VH']} for vk in g['VKEYS'] if cells(vk) != held['cells'][vk]]
            cur, live, sf = L['cur'], L['live'], L['sf']
            # (the points the round's program held: not a breach's two, nor a dive's end, which is its via's)
            lit = [r3(cur[k][j]) for k in g['KEYS'] for j in range(1, len(cur[k]) - 1)
                   if live[k][j] and held['kind'][k][j - 1] not in 'XD']
            rec['rounds'].append({
                'it': int(L['it']), 'nlive': int(L['nlive']), 'nvia': len(L['vvar']), 'nvar': len(L['lp'].c),
                'nrow': len(L['lp'].b), 'move': float(arg[1]), 'hops': int(L['hops']), 'nexc': int(L['nexc']),
                'npull': len(L['npull']) // 2, 'length': float(L['ln_']), 'crossings': len(L['xs_']),
                'standing': {k: [len(sf.get(k, [])), float(sf[k][0][0]) if sf.get(k) else 0.0]
                             for k in ('pitch', 'via', 'viavia')},
                'lit': lit, 'hot': [r3(q) for q in (g['HOT'] or [])],
                'vhop': hops, 'bulge': held['bulge'], 'pull': held['pull'], 'pulled': held['places'], 'after': stand(g)})
        elif name == 'slide_vias' and event == 'call':
            held['slid'] = set(g['SLID'])
            held['stand'] = stand(g)
        elif name == 'slide_vias' and event == 'return' and arg:
            rec['slides'].append({'after': len(rec['rounds']), 'vias': [nm(vk) for vk in sorted(g['SLID'] - held['slid'])],
                                  'before': held['stand'], 'now': stand(g)})
        elif name == 'shape_pass' and event == 'call':
            held['own'] = {k: list(v) for k, v in frame.f_locals['OWN'].items()}
        elif name == 'shape_pass' and event == 'return' and arg is not None:
            OWN = frame.f_locals['OWN']
            rec['shape'] = {'out': int(arg[0]), 'left': int(arg[1]),
                            'gone': [r4(p) for k, v in held['own'].items() for p in v if p not in OWN[k]]}
    buf = io.StringIO()
    sys.setprofile(hook)
    try:
        with contextlib.redirect_stdout(buf):
            G = runpy.run_path(EMB, run_name='__main__')
    finally:
        sys.setprofile(None)
    identical = filecmp.cmp(tmp, eref, shallow=False)
    import shapely
    from shapely.geometry import LineString, Point
    PL, VIA, RUN, SF = G['PL'], G['VIA'], G['RUN'], G['SF']

    def where(kind, what):
        """where a shortfall stands: the middle of the shortest line between the two things it is between"""
        a, b = what
        if kind == 'viavia':
            return r4(((VIA[a][0] + VIA[b][0]) / 2, (VIA[a][1] + VIA[b][1]) / 2))
        c = shapely.shortest_line(Point(VIA[a]) if kind == 'via' else LineString(PL[a]), LineString(PL[b])).centroid
        return r4((c.x, c.y))
    mesh = lambda m: {'V': [r4(p) for p in m.V], 'T': [[int(i) for i in t] for t in m.T]}
    # (the counts the stage's own log gives must be the events seen: a round whose events were not all seen is said)
    seen = all(sum(1 for h in r['vhop'] for a, b in zip(h['was'], h['now']) if a != b) == r['hops']
               and len(r['bulge']) == r['nexc'] and len(r['lit']) == r['nlive'] for r in rec['rounds'])
    res = dict(rec, identical=identical, seen=seen, layers=list(G['LNAME']),
               mesh=[mesh(m) for m in G['MESH']], vmesh=mesh(G['VMESH']),
               fixed=[[round(float(o[0]), 4), round(float(o[1]), 4), round(float(o[2]), 4), round(float(o[3]), 4),
                       sorted(int(x) for x in o[4]), str(o[5])] for o in G['FIXED']
                      if not (o[2] < G['WX0'] - 1 or o[0] > G['WX1'] + 1 or o[3] < G['WY0'] - 1 or o[1] > G['WY1'] + 1)],
               runs={nm(k): {'L': int(RUN[k]['L']), 'lead': r4(RUN[k]['lead']) if RUN[k].get('lead') else None,
                             'tail': r4(RUN[k]['tail']) if RUN[k].get('tail') else None} for k in G['KEYS']},
               rules={k: float(G[k]) for k in ('TW', 'PP', 'VIA_R', 'TRUST')}, via_size=float(G['bd'].VIA_SIZE),
               settled=not G['HOT'], slid=[nm(vk) for vk in sorted(G['SLID'])], nmouth=int(G['NMOUTH']),
               breach=[{'run': nm(k), 'L': int(RUN[k]['L']), 'from': r4(pa), 'to': r4(pb), 'copper': list(labs)}
                       for (k, pa, pb, labs) in G['BREACH']],
               short=[{'kind': k, 'mm': float(v), 'what': [nm(w[0]), nm(w[1])], 'at': where(k, w)}
                      for k in ('pitch', 'via', 'viavia') for v, w in SF.get(k, [])],
               tight=[{'kind': 'lane', 'mm': float(v), 'at': where('pitch', w)} for v, w in SF.get('slack-lane', [])]
               + [{'kind': 'via', 'mm': float(v), 'at': r4(VIA[w[0]])} for v, w in SF.get('slack-via', [])],
               faults=int(G['_f0']), log=[ln[:900] for ln in buf.getvalue().splitlines()])
    json.dump(res, open(out, 'w'))
    print(f'whole_movie: 2-D geometry traced -- {len(rec["rounds"])} round(s), {len(rec["slides"])} slide(s), every event '
          f'the stage counted seen: {seen}, identical to the chain: {identical}')


# =============================================================================== the run's files
def _read(path):
    return open(path).read() if os.path.isfile(path) else ''


def _parse(path):
    sys.path.insert(0, PYR)
    with contextlib.redirect_stdout(io.StringIO()):
        from kicad_parser import parse_kicad_pcb
        return parse_kicad_pcb(path)


def loop_rounds(round_dir):
    """the loop's rounds as whole_route.py ran them, replayed from its log: each round's solve and the side flips its
    geometry was given (GEO_FLIPS_FROM), and the round whose plan passed"""
    loop = os.path.join(round_dir, 'loop')
    rounds, flips, cur, passed = {}, [], None, None
    for line in _read(os.path.join(round_dir, 'loop.log')).splitlines():
        m = re.match(r'=== round (\d+): geometry of (\S+)', line)
        if m:
            cur = int(m.group(1))
            s = m.group(2)
            rounds[cur] = {'i': cur, 'solve': os.path.join(round_dir, s) if s == 'solve.json' else os.path.join(loop, s),
                           'flips': list(flips), 'lines': [line]}
            continue
        if cur is not None:
            rounds[cur]['lines'].append(line)
        m = re.match(r'=== round (\d+): (\d+) new side flip', line)
        if m:
            flips = [os.path.join(loop, f'p{m.group(1)}.json')]
        if re.match(r'\s+pairs held: \d+ new side flip', line) and cur is not None:
            qk = os.path.join(loop, f'qk{cur}.json')
            flips.append(qk if os.path.isfile(qk) else os.path.join(loop, f'q{cur}.json'))
        if line.startswith('=== the plan passes'):
            passed = cur
    return rounds, passed


class Story:
    """everything the film shows, read from the run and the traces"""


def load_story(a):
    S = Story()
    rdir = os.path.abspath(a.rundir)
    rs = sorted(int(m.group(1)) for d in os.listdir(rdir) for m in [re.match(r'r(\d+)$', d)] if m)
    routed = [r for r in rs if os.path.isfile(os.path.join(rdir, f'r{r}', 'seq.kicad_pcb'))]
    if not routed:
        raise SystemExit(f'whole_movie: no round of {rdir} routed (rN/seq.kicad_pcb) -- the film is of a run that routed')
    S.rn = routed[-1]
    S.nrounds = len(rs)
    rd = os.path.join(rdir, f'r{S.rn}')
    S.rd, S.loop = rd, os.path.join(rd, 'loop')
    S.dest = os.environ.get('DEST', 'DU1')
    S.base_path = os.environ.get('BASE', 'fb_t2q_pairs.kicad_pcb')
    step, rec = bus_step(rdir), chain_record(rdir)
    if step:
        # (a bus step's run: its own destination, and the board it handed the round -- the bus's ends on it as an
        # earlier step laid them -- unless BASE names the board from before they were laid)
        S.dest = step.get('dest') or S.dest
        S.base_path = os.environ.get('BASE') or os.path.join(step['work'], 'frame.kicad_pcb')
    elif rec:
        # (a chain run by another harness, replay_round.py's: the destination and the bench its chain was asked)
        S.dest = rec.get('dest') or S.dest
        S.base_path = os.environ.get('BASE') or rec.get('base') or S.base_path
    # (the run's name on the title card: a bus step's work directory, else the run's own -- its parent's where it is
    # a harness's OUT/run)
    S.tag = os.path.basename(step['work'] if step else os.path.dirname(rdir) if os.path.basename(rdir) == 'run' else rdir)
    if not os.path.isabs(S.base_path):
        S.base_path = os.path.join(HERE, S.base_path)
    if S.rn > 1:        # the round's own input: the previous round's source board (whole_route.py's BASE for it)
        sb = re.findall(r'source board: ([^,]+)', _read(os.path.join(rdir, f'r{S.rn - 1}', 'fo.log')))
        if sb and os.path.isfile(os.path.join(rdir, f'r{S.rn - 1}', sb[-1].strip())):
            S.base_path = os.path.join(rdir, f'r{S.rn - 1}', sb[-1].strip())
    rounds, passed = loop_rounds(rd)
    if passed is None:
        raise SystemExit(f'whole_movie: {rd}/loop.log names no round whose plan passed')
    S.rounds, S.lr = rounds, passed
    R = rounds[passed]
    # (the routing layers, F.Cu and B.Cu first: the step's ROUTE_LAYERS -- on more than two the solve names each run's
    # layer -- else the two)
    S.layers = [L_.strip() for L_ in (((step or {}).get('settings') or {}).get('ROUTE_LAYERS') or 'F.Cu,B.Cu').split(',')
                if L_.strip()]
    for L_ in sorted({L_ for v in (json.load(open(R['solve'])).get('layers') or {}).values() for L_ in v}):
        if L_ not in S.layers:
            S.layers.append(L_)
    # (a NEGOTIATED bus's round, route_bus JOINT_DIR: its first plan is the coupled solve's, written over the one it
    # solved alone -- the trace re-runs the bus's own solve, and the coupled plan is read from its file)
    sj_ = json.load(open(os.path.join(rd, 'solve.json')))
    S.coupled = sj_ if sj_.get('joint') and solo_ref(rd) != os.path.join(rd, 'solve.json') else None
    S.joint_lines = []
    if S.coupled:
        # (...and what the coupled solve said of its round: its couplings, the buses' own plans side by side, its search)
        jd_ = ((step or {}).get('settings') or {}).get('JOINT_DIR') or ''
        S.joint_lines = [ln.strip() for ln in _read(os.path.join(jd_, f'r{S.coupled["joint"].get("round", 1)}',
                                                                  'joint.log')).splitlines()
                         if re.match(r'\s*(couplings|the buses\' own plans side by side|coupled search):', ln)]
    # ---- the traces (observed re-runs), cached beside the run
    mv = os.path.join(rd, 'movie')
    os.makedirs(mv, exist_ok=True)
    # (the round's first solve on the fanout's own bench; the passing round's geometry on the bench it stood on)
    env, genv = _chain_env(rd, S.dest), _chain_env(rd, S.dest, round_bench(rd, passed))
    if genv.get('JOINT_FOREIGN'):
        # (the other buses' lanes are read as they stand NOW: a bus that published after the passing round's geometry
        # ran gives the re-run other copper than the chain's stage had, and it may not reproduce the chain's file)
        ld_ = os.path.join(genv['JOINT_DIR'], 'lanes')
        late = [f[:-len('.json')] for f in sorted(os.listdir(ld_)) if os.path.isdir(ld_) and f.endswith('.json')
                and not f.startswith('.') and f != f'{genv["JOINT_BUS"]}.json'
                and os.path.getmtime(os.path.join(ld_, f)) > os.path.getmtime(R['solve'])] if os.path.isdir(ld_) else []
        if late:
            print(f'whole_movie: NOTE -- {", ".join(late)} published lanes after round {passed}\'s solve was written: the '
                  f're-run of its geometry reads them as they stand now')
    ts, tg = os.path.join(mv, 'trace_solve.json'), os.path.join(mv, f'trace_geo{passed}.json')
    te = os.path.join(mv, f'trace_embed{passed}.json')
    if not a.force:
        # (a kept trace whose re-run did not reproduce the chain is not the round's: run again, not refused for good --
        # a solve stopped early, by its bound met, can find one plan more or fewer from one run to the next)
        for t_ in (ts, tg, te):
            if os.path.isfile(t_) and not json.load(open(t_)).get('identical'):
                os.remove(t_)
    if not os.path.isfile(ts):
        subprocess.run([sys.executable, os.path.abspath(__file__), '--_trace', 'solve', rd, ts], env=env, check=True)
    if os.path.isfile(tg) and 'relax' not in json.load(open(tg)):
        os.remove(tg)                                      # (a trace from before the film drew the taut strings)
    if not os.path.isfile(tg):
        subprocess.run([sys.executable, os.path.abspath(__file__), '--_trace', 'geo', rd, R['solve'], ','.join(R['flips']),
                        os.path.join(S.loop, f'g{passed}.json'), tg], env=genv, check=True)
    S.T = json.load(open(ts))
    S.Gt = json.load(open(tg))
    # the round's 2-D geometry, where its loop ran one (eN.json beside gN.json: the frame geometry is then only its
    # start, and the round's geometry is this one's)
    S.Et = None
    eref = os.path.join(S.loop, f'e{passed}.json')
    if os.path.isfile(eref):
        if os.path.isfile(te) and 'barrel' not in (json.load(open(te)).get('start') or {}):
            os.remove(te)                                  # (a trace from before the film drew a pair's barrels)
        if not os.path.isfile(te):
            subprocess.run([sys.executable, os.path.abspath(__file__), '--_trace', 'embed', rd, R['solve'],
                            os.path.join(S.loop, f'g{passed}.json'), eref, te], env=genv, check=True)
        S.Et = json.load(open(te))
    same = {'solve': S.T['identical'], 'geometry': S.Gt['identical']}
    if S.Et:
        same['2-D geometry'] = S.Et['identical']
    S.identical = all(same.values())
    if a.check:
        print(f'whole_movie: round {passed} of {rd} run again -- ' + ', '.join(
            f'{k} {"identical to the chain" if v else "NOT what the chain wrote"}' for k, v in same.items())
            + ('' if not S.Et or S.Et['seen'] else '; a round of the 2-D geometry counted cell changes the hook did not all see'))
        raise SystemExit(0 if S.identical else 1)
    if not S.identical and not a.force:
        raise SystemExit('whole_movie: a re-run did not reproduce the chain ('
                         + ', '.join(f'{k} {v}' for k, v in same.items())
                         + ') -- the film would show another run; --force films it anyway')
    if S.Et and not S.Et['seen']:
        print('whole_movie: NOTE -- a round of the 2-D geometry counted cell changes the film\'s hook did not all see; '
              'the counts drawn are the stage\'s own')
    # ---- the stages' outputs (the passing round)
    i = passed
    L = S.loop
    ld = lambda f: json.load(open(os.path.join(L, f))) if os.path.isfile(os.path.join(L, f)) else None
    S.geo, S.pol, S.snapd = ld(f'g{i}.json'), ld(f'p{i}.json'), ld('plan.json')
    S.emb = ld(f'e{i}.json') if S.Et else None
    S.qfile = f'q{i}.json'
    if os.path.isfile(os.path.join(L, f'qk{i}.json')):
        g = subprocess.run([sys.executable, os.path.join(HERE, 'whole_gate.py'), os.path.join(L, f'qk{i}.json'),
                            os.path.join(L, f'qk{i}.audit')], cwd=HERE, env=genv, capture_output=True)
        if g.returncode == 0:
            S.qfile = f'qk{i}.json'
    S.pairsd = ld(f'pairs{i}k.json' if S.qfile.startswith('qk') else f'pairs{i}.json')
    S.held = ld(S.qfile)
    S.solve_round = json.load(open(R['solve']))            # the solve the passing round's geometry was laid on
    S.failed = [rounds[r] for r in sorted(rounds) if r < passed]
    # ---- the logs' numbers
    fo = _read(os.path.join(rd, 'fo.log'))
    S.ends = [(m.group(1), int(m.group(3)), int(m.group(4))) for m in
              re.finditer(r'whole ends: (on the teeth as laid|teeth moved \((\d+)\)): (\d+) vias predicted.*?(\d+) crossings', fo)]
    S.grouped = bool(re.search(r'^group ends:', fo, re.M))      # (the ends laid before this step, by the group's model)
    S.gate = {k: v for k, v in re.findall(r'^\s+(smooth|pairs held|snapped): (.*)$', _read(os.path.join(rd, 'loop.log')), re.M)
              if not v.startswith('LINT')}
    S.lint = re.findall(r'^\s+snapped: (LINT.*)$', _read(os.path.join(rd, 'loop.log')), re.M)
    rl = _read(os.path.join(rd, 'route_seq.log'))
    S.route_lines = [ln for ln in rl.splitlines() if re.match(r'^\S+\s+(pair|page)\s+IN BAND', ln)]
    S.route_order = [ln.split()[0] for ln in S.route_lines]
    S.route_summary = next((ln for ln in rl.splitlines() if ln.startswith('SUMMARY')), '')
    S.walled = [ln.strip() for ln in rl.splitlines() if 'WALLED IN' in ln]     # (the lay's wall guard, route_lanes)
    S.connected = 'ALL NETS FULLY CONNECTED' in _read(os.path.join(rd, 'conn.log'))
    S.drc_clean = 'NO DRC VIOLATIONS' in _read(os.path.join(rd, 'drc.log'))
    # ---- the lanes and their nets
    Gt = S.Gt
    S.M = list(Gt['M'])
    S.pairs = Gt['pairs']
    S.legs = {n: (S.pairs[n] if n in S.pairs else [n]) for n in S.M}
    S.nets = sorted({leg for n in S.M for leg in S.legs[n]})
    S.lane_of = {leg: n for n in S.M for leg in S.legs[n]}
    S.src, S.dst = Gt['src'], Gt['dst']
    # ---- the boards
    S.base = _parse(S.base_path)
    S.fo = _parse(os.path.join(rd, 'fo.kicad_pcb'))
    S.seq = _parse(os.path.join(rd, 'seq.kicad_pcb'))
    S.human = None
    hp = a.human or os.path.join(HERE, 'fb_t2q_human.kicad_pcb')
    if hp and os.path.isfile(hp):
        h = _parse(hp)
        same = all(r in h.footprints and abs(h.footprints[r].x - S.fo.footprints[r].x) < 1e-3 and
                   abs(h.footprints[r].y - S.fo.footprints[r].y) < 1e-3 for r in (S.src, S.dst))
        S.human = h if same else None
        if not same:
            print(f'whole_movie: {hp} does not place {S.src} and {S.dst} where the bench does -- no human comparison')
    S.G = load_group(rdir, step, rec)
    S.N = load_negotiation(S, step)
    return S


def load_group(rdir, step, rec):
    """THE GROUP'S ENDS as the bus step recorded them, or None (a run that is no bus step's, or one from before the
    group's model -- the film then starts at the bus's own ends): the step's events (OUT.bus/events.jsonl: its plan,
    the capture, every round of the model's search with the best it found), the model's every solve, state, check and
    field (group_ends' checks.jsonl), its log (group.log), its result (group.json: every array's exits, drops and
    straps) and the boards it measured and laid (prepared.kicad_pcb, shared_field.kicad_pcb). Every point is turned
    into the bench's frame (ROUND_EDITS_TURN, as whole_route.edit_frame turns a board point) -- the film draws the bench;
    a state's ends, which the model holds in each bus's own frame, by that frame's two source pads (Film._group)"""
    gdir = ((step or {}).get('settings') or {}).get('GROUP_ENDS')
    if not gdir or not os.path.isfile(os.path.join(gdir, 'checks.jsonl')) or not os.path.isfile(os.path.join(gdir, 'group.json')):
        return None
    G = Story()
    G.dir = gdir
    G.work = os.path.dirname(gdir)                         # SRC-DEST.bus: the group's directory
    G.out = os.path.dirname(G.work)                        # OUT.bus: the step's
    sys.path.insert(0, HERE)
    import flow_frame as ff
    t_ = (((rec or {}).get('settings') or {}).get('ROUND_EDITS_TURN') or '0,0,0').split(',')
    q_, cx_, cy_ = int(float(t_[0])), float(t_[1]), float(t_[2])

    def tb(p):
        dx_, dy_ = ff._q(float(p[0]) - cx_, float(p[1]) - cy_, q_)
        return (cx_ + dx_, cy_ + dy_)
    G.tb = tb
    G.recs = [json.loads(ln) for ln in open(os.path.join(gdir, 'checks.jsonl')) if ln.strip()]
    G.log = _read(os.path.join(G.work, 'group.log'))
    G.json = json.load(open(os.path.join(gdir, 'group.json')))
    G.ev = []
    ep = os.path.join(G.out, 'events.jsonl')
    if os.path.isfile(ep):
        for ln in open(ep):
            try:
                G.ev.append(json.loads(ln))
            except ValueError:
                pass
    G.ev = [e for e in G.ev if e.get('group') in (None, step.get('src'))]
    ev1 = lambda st_, kd_: next((e for e in G.ev if e['stage'] == st_ and e['kind'] == kd_), None)
    G.plan = (ev1('step', 'plan') or {}).get('buses') or []
    # (the nets each bus routes, from its first start: the plan counts the nets between its two parts alone, the bus
    # routes those with a pad on a third part too)
    for b in G.plan:
        e = next((e for e in G.ev if e['stage'] == 'bus' and e['kind'] == 'start'
                  and e.get('bus') == f'{b["src"]}:{b["dest"]}' and e.get('bus_nets')), None)
        b['routed'] = e['bus_nets'] if e else b['nets']
    G.step_start = ev1('step', 'start') or {}
    G.cap_end = [e for e in G.ev if e['stage'] == 'capture' and e['kind'] == 'end']
    G.group_end = ev1('group', 'end')
    e0 = ev1('ends', 'start')
    G.t0 = e0['t'] if e0 else None                         # (the model's clock: checks.jsonl's t counts from here)
    # every round's best, as the search reported it: a round starts with its first report, the best not yet found
    G.rounds = []
    cur = None
    for e in G.ev:
        if e['stage'] != 'ends' or e['kind'] != 'progress' or G.t0 is None:
            continue
        if e.get('best') is None:
            if cur and cur['best'] is not None:
                G.rounds.append(cur)
            cur = {'t': e['t'] - G.t0, 'label': e.get('label') or '', 'best': None}
        elif cur is not None:
            cur['best'], cur['t'] = e['best'], e['t'] - G.t0
    if cur and cur['best'] is not None:
        G.rounds.append(cur)
    # the model's solves, each with its state, and what the log says of it (the stretch it was, the search's line)
    G.models = []
    for r_ in G.recs:
        if r_['rec'] == 'model':
            G.models.append({'model': r_, 'states': {}})
        elif r_['rec'] == 'state' and G.models:
            G.models[-1]['states'][r_['bus']] = r_
    labels, searches, sweep, lab = [], [], 1, 'the start'
    search_ln = None
    for ln in G.log.splitlines():
        m = re.match(r"\s+the search's stretch (\d+) of (\d+)", ln)
        if m:
            lab = f'sweep {sweep}, stretch {m.group(1)} of {m.group(2)}'
        if re.match(r"\s+the search's sweep (\d+):", ln):
            sweep += 1
        if re.match(r"\s+the repair's model", ln) or re.match(r'\s+the repair: ', ln):
            lab = 'the repair'
        if re.match(r"\s+the repair's state is dearer", ln):
            lab = 'the repair\'s lanes searched again'
        if re.match(r'\s+the search: ', ln):
            search_ln = ln.strip()
        if re.match(r'group ends: \[\w+\] in ', ln):
            labels.append(lab)
            searches.append(search_ln)
            search_ln = None
    if len(labels) == len(G.models):
        for M_, l_, s_ in zip(G.models, labels, searches):
            M_['label'], M_['search'] = l_, s_
    else:
        for j, M_ in enumerate(G.models):
            M_['label'], M_['search'] = f'solve {j + 1}', None
    for M_ in G.models:
        o = M_['model']['objective']
        # (the model's own value: the rest, in vias, and a change of layer at its weight -- the costly ends apart)
        M_['value'] = sum(o['rest_vias'].values()) + o['changes'] * M_['model']['W_LEX'] / 1000
        M_['costly'] = o['walls'] + o['fans'] + o['far_teeth']
        m = re.search(r'with what the model does not count ([\d.]+)', M_['search'] or '')
        M_['uncounted'] = float(m.group(1)) if m else None
        m = re.search(r': ([\d.]+) \(\d+ s\)$', M_['search'] or '') if M_['label'] == 'the start' else None
        M_['start_value'] = float(m.group(1)) if m else None
        # (the search's own value of the state its rounds ended on -- "A -> B" -- on the scale of its start's)
        m = re.search(r': [\d.]+ -> ([\d.]+),', M_['search'] or '')
        M_['end_value'] = float(m.group(1)) if m else None
    G.verdicts = [r_ for r_ in G.recs if r_['rec'] == 'verdict']
    G.fields = [r_ for r_ in G.recs if r_['rec'] == 'field']
    G.done = next((r_ for r_ in G.recs if r_['rec'] == 'done'), None)
    # the check solves' own consoles
    for v_ in G.verdicts:
        v_['console'] = [ln.rstrip() for ln in _read(os.path.join(gdir, v_.get('solve_log') or '-')).splitlines() if ln.strip()]
    # the log's own lines for each stage of the model, as it wrote them
    lines = G.log.splitlines()
    G.arrays = [ln.strip() for ln in lines if re.match(r'\s+\S+: \d+ bus balls, \d+ other signal balls', ln)]
    G.tiers = [ln.strip() for ln in lines if re.match(r'\s+\S+: \d+ bus balls, their whole menus', ln)]
    G.state_q = []
    for ln in lines:
        m = re.match(r'\s+round (\d+): (no state in the pattern.*|a state holding every rule.*)$', ln)
        if m:
            G.state_q.append((int(m.group(1)), m.group(2)))
        if re.match(r"\s+the search's stretch 1 of", ln):
            break
    G.start_lines = []
    for ln in lines:
        if re.match(r'\s+(the start, |each bus\'s ends in turn|the search: the start alone)', ln):
            G.start_lines.append(ln.strip())
        if re.match(r"\s+the search's stretch 1 of", ln):
            break
    # (the field as the model's last state laid it: each array's other balls as that state served them -- the log says
    # so after every solve, and the last is the field's -- then the fanout, the nudge, the field's measure and its ties)
    G.field_lines = list({ln.strip().split(':')[0]: ln.strip() for ln in lines
                          if re.match(r"\s+\S+'s other balls: ", ln)}.values())
    last_ = lambda rx: next((ln.strip() for ln in reversed(lines) if re.match(rx, ln)), None)
    G.field_lines += [ln for ln in (last_(r'\s+the arrays fanned out as chosen'), last_(r'\s+caps: '),
                                    last_(r'\s+the field as laid'), last_(r'\s+field ties: ')) if ln]
    G.lesson_lines = [ln.strip() for ln in lines if re.match(r'\s+type \S', ln)]
    G.dearer = next((ln.strip() for ln in lines if "the repair's state is dearer" in ln), None)
    G.repair_ln = next((ln.strip() for ln in lines if re.match(r'\s+the repair: ', ln)), None)
    G.secs_line = next((ln.strip() for ln in lines if re.match(r'group ends: \d+ s in all', ln)), None)
    # the boards: the one the menus were measured on, and the field as laid
    G.prep = _parse(os.path.join(gdir, 'prepared.kicad_pcb')) if os.path.isfile(os.path.join(gdir, 'prepared.kicad_pcb')) else None
    fp_ = os.path.join(gdir, 'shared_field.kicad_pcb')
    G.field = _parse(fp_) if os.path.isfile(fp_) else None
    tp_ = os.path.join(gdir, 'field_ties.json')
    G.ties = json.load(open(tp_)) if os.path.isfile(tp_) else None
    sp_ = os.path.join(G.out, 'summary.json')
    G.summary = json.load(open(sp_)) if os.path.isfile(sp_) else {}
    return G


def load_negotiation(S, step):
    """THE NEGOTIATION of a bus step's negotiated group (route_bus JOINT_DIR: the buses off one shared array, each its
    own whole route in a process of its own, their first plans one COUPLED solve), or None: as the step recorded it,
    never run again -- its events (each bus's start, fanout, first solve, its wait for the coupled solve, its loop and the
    loop's re-solves, its lay, its grade and its end, on one clock from the negotiated route's start), the coupled solve's
    report (JOINT_DIR/r1/joint.json: every coupling of two buses' lanes, the ones their own plans broke side by side, the
    ones the coupled plan still breaks), the couplings each bus held through its loop (BUS.cuts.json), the polish steps
    the buses took in step (spolish.*/BUS.done), each bus's summary and its loop's rounds, and the merged board: each
    bus's copper on it, in the bench's frame (the group's turn, G.tb)"""
    import ast
    G = S.G
    st_ = (step or {}).get('settings') or {}
    jd = st_.get('JOINT_DIR')
    if G is None or not jd or not os.path.isdir(jd):
        return None
    r0 = next((e for e in G.ev if e['stage'] == 'route' and e['kind'] == 'start' and e.get('negotiated')), None)
    if r0 is None:
        return None
    N = Story()
    N.dir, N.t0, N.me, src = jd, r0['t'], S.dest, step.get('src')
    N.order = [b for b in (st_.get('JOINT_ORDER') or '').split(',') if b]
    peers = [b for b in (st_.get('JOINT_PEERS') or '').split(',') if b]
    N.buses = [b for b in N.order if b in peers] + [b for b in peers if b not in N.order]
    rel = lambda e: (e['t'] - N.t0) if e else None                         # noqa: E731
    N.ph = {}
    for b in N.buses:
        mine = [e for e in G.ev if e.get('bus') == f'{src}:{b}' and e['t'] >= N.t0 - 1]
        first = lambda st, kd: next((e for e in mine if e['stage'] == st and e['kind'] == kd), None)   # noqa: E731
        solves = [e for e in mine if e['stage'] == 'solve' and e['kind'] == 'end']
        lp0 = first('loop', 'start')
        N.ph[b] = dict(start=first('bus', 'start'), fanout=(rel(first('fanout', 'start')), rel(first('fanout', 'end'))),
                       solve=(rel(first('solve', 'start')), rel(solves[0]) if solves else None),
                       solve_ev=solves[0] if solves else None,
                       wait=(rel(first('coupled', 'start')), rel(first('coupled', 'end'))),
                       coupled=first('coupled', 'end'), loop=(rel(lp0), rel(first('loop', 'end'))),
                       resolves=[rel(e) for e in solves if lp0 and e['t'] > lp0['t']],
                       lay=(rel(first('lay', 'start')), rel(first('lay', 'end'))), laid=first('lay', 'end'),
                       graded=first('bus', 'graded'), end=rel(first('bus', 'end')), end_ev=first('bus', 'end'))
    ev1 = lambda st, kd: next((e for e in G.ev if e['stage'] == st and e['kind'] == kd and e['t'] >= N.t0), None)  # noqa
    N.search, N.merge, N.grade = ev1('coupled', 'search'), ev1('merge', 'end'), ev1('grade', 'end')
    N.t_end = max([rel(e) for e in G.ev if e['t'] >= N.t0] + [1.0])
    # the coupled solve's report, its couplings put in the bench's frame
    try:
        J = json.load(open(os.path.join(jd, 'r1', 'joint.json')))
    except (OSError, ValueError):
        J = {}
    pt = lambda c: G.tb(c[6])                                               # noqa: E731
    key = lambda c: tuple(c[:6])                                           # noqa: E731
    N.couplings = [pt(c) for c in J.get('all') or []]
    N.broken_alone = [pt(c) for c in J.get('broken_alone') or []]
    N.broken = [pt(c) for c in J.get('broken') or []]
    # (the couplings the buses' own plans broke and the coupled plan settled: broken alone, not broken coupled)
    still = {key(c) for c in J.get('broken') or []}
    N.settled = [pt(c) for c in J.get('broken_alone') or [] if key(c) not in still]
    N.joint_secs = J.get('secs')
    N.held, N.mine_broken = {}, {}
    for b in N.buses:
        try:
            N.held[b] = len(json.load(open(os.path.join(jd, 'r1', f'{b}.cuts.json'))).get('lcuts') or [])
        except (OSError, ValueError):
            N.held[b] = 0
        N.mine_broken[b] = sum(1 for c in J.get('broken') or [] if b in (c[0], c[3]))
    # (the steps the buses' loops took in step: each bus's polished plan published, said in spolish.R.L.I/BUS.done)
    N.syncs = {b: [] for b in N.buses}
    for d in sorted(os.listdir(jd)):
        if d.startswith('spolish.') and os.path.isdir(os.path.join(jd, d)):
            for f in os.listdir(os.path.join(jd, d)):
                if f.endswith('.done') and f[:-5] in N.syncs:
                    N.syncs[f[:-5]].append(os.path.getmtime(os.path.join(jd, d, f)) - N.t0)
    # each bus's summary, its loop's rounds, and its copper on the merged board
    gname = os.path.basename(G.work)[:-len('.bus')] if G.work.endswith('.bus') else os.path.basename(G.work)
    gs = {}
    try:
        gs = json.load(open(os.path.join(G.out, f'{gname}.summary.json')))
    except (OSError, ValueError):
        pass
    lit = lambda v: ast.literal_eval(v) if isinstance(v, str) else v       # noqa: E731
    N.group_grade, N.conflicts = lit(gs.get('grade') or {}) or {}, gs.get('conflicts')
    N.sum, N.rounds, nets = {}, {}, {}
    for b in N.buses:
        sp = os.path.join(G.work, 'neg', b, 'bus.bus', f'{src}-{b}.summary.json')
        try:
            N.sum[b] = json.load(open(sp))
        except (OSError, ValueError):
            N.sum[b] = {}
        nets[b] = set(N.sum[b].get('routed') or [])
        lg = _read(os.path.join(G.work, 'neg', b, 'bus.bus', 'run', 'r1', 'loop.log'))
        N.rounds[b] = max([int(m) for m in re.findall(r'^=== round (\d+): geometry', lg, re.M)] or [0])
    N.copper = {}
    mp_ = gs.get('out')
    if mp_ and os.path.isfile(mp_):
        M = _parse(mp_)
        for b in N.buses:
            segs, vias = copper(M, nets[b])
            N.copper[b] = ([(*G.tb(q_[:2]), *G.tb(q_[2:4]), q_[4], q_[5], q_[6]) for q_ in segs],
                           [(*G.tb(v_[:2]), v_[2], v_[3]) for v_ in vias])
    N.nets = nets
    return N


def copper(pcb, names):
    """(segments, vias) of the nets `names` (short names): segments (x0, y0, x1, y1, width, layer, net), vias
    (x, y, size, net)"""
    ids = {i: n.name.split('/')[-1] for i, n in pcb.nets.items() if n.name.split('/')[-1] in names}
    segs = [(s.start_x, s.start_y, s.end_x, s.end_y, s.width, s.layer, ids[s.net_id]) for s in pcb.segments if s.net_id in ids]
    vias = [(v.x, v.y, v.size, ids[v.net_id]) for v in pcb.vias if v.net_id in ids]
    return segs, vias


def _skey(s):
    a, b = (round(s[0] * 1000), round(s[1] * 1000)), (round(s[2] * 1000), round(s[3] * 1000))
    return (min(a, b), max(a, b), s[5], s[6])


def minus(A, B, key):
    """the items of A not in B, as multisets under key"""
    c = collections.Counter(key(x) for x in B)
    out = []
    for x in A:
        k = key(x)
        if c[k]:
            c[k] -= 1
        else:
            out.append(x)
    return out


def copper_len(segs):
    return sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)


# =============================================================================== geometry of the drawing
import numpy as np            # noqa: E402


class Mapper:
    """the plan's frame to the board's: identity, or (a pair chirality of -1) braid.setup's turn back (ctx.M, the
    layers swapped: F.Cu and B.Cu, an inner layer its own)"""

    def __init__(self, Gt):
        self.aff = Gt.get('aff') if Gt.get('chi', 1) < 0 else None

    def layer(self, name):
        """a layer of the plan's frame by its board name"""
        if not self.aff:
            return name
        return {'F.Cu': 'B.Cu', 'B.Cu': 'F.Cu'}.get(name, name)

    def pt(self, x, y):
        if not self.aff:
            return (x, y)
        o, ex, ey = self.aff
        return (o[0] + x * ex[0] + y * ey[0], o[1] + x * ex[1] + y * ey[1])

    def vec(self, dx, dy):
        if not self.aff:
            return (dx, dy)
        _o, ex, ey = self.aff
        return (dx * ex[0] + dy * ey[0], dx * ex[1] + dy * ey[1])

    def bit(self, b):
        return (1 - b) if self.aff else b


def resample(pts, bits, n):
    """a polyline (its per-segment layer bits) resampled to n points by arc length: (n x 2 points, n bits)"""
    P = np.asarray(pts, float)
    if len(P) < 2:
        P = np.vstack([P, P])
        bits = [bits[0] if len(bits) else 0]
    seg = np.hypot(*(P[1:] - P[:-1]).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.linspace(0.0, cum[-1], n)
    i = np.clip(np.searchsorted(cum, t, side='right') - 1, 0, len(seg) - 1)
    f = np.where(seg[i] > 1e-12, (t - cum[i]) / np.where(seg[i] > 1e-12, seg[i], 1.0), 0.0)
    return P[i] + (P[i + 1] - P[i]) * f[:, None], np.asarray(bits, int)[i]


def plen(pts):
    P = np.asarray(pts, float)
    return float(np.hypot(*(P[1:] - P[:-1]).T).sum()) if len(P) > 1 else 0.0


def along(pts, us, u):
    """the point at route coordinate u of a polyline whose vertices carry route coordinates us"""
    for j in range(len(us) - 1):
        if us[j] <= u <= us[j + 1] and us[j + 1] > us[j]:
            f = (u - us[j]) / (us[j + 1] - us[j])
            return (pts[j][0] + (pts[j + 1][0] - pts[j][0]) * f, pts[j][1] + (pts[j + 1][1] - pts[j][1]) * f)
    return tuple(pts[0]) if u < us[0] else tuple(pts[-1])


def simplify(P, eps):
    """a polyline's points with the ones within eps of the chord between their neighbours dropped (Douglas-Peucker)"""
    P = np.asarray(P, float)
    if len(P) < 3:
        return P
    keep = np.zeros(len(P), bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(P) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = P[i], P[j]
        ab = b - a
        L = math.hypot(*ab)
        seg = P[i + 1:j] - a
        d = np.abs(seg[:, 0] * ab[1] - seg[:, 1] * ab[0]) / L if L > 1e-12 else np.hypot(seg[:, 0], seg[:, 1])
        k = int(np.argmax(d))
        if d[k] > eps:
            keep[i + 1 + k] = True
            stack += [(i, i + 1 + k), (i + 1 + k, j)]
    return P[keep]


def nearest(P, p):
    """the point of the (dense) polyline P nearest p"""
    P = np.asarray(P, float)
    return tuple(P[int(np.argmin(((P - np.asarray(p)) ** 2).sum(axis=1)))])


def force_marks(film, rows, st, vs):
    """each priced rule of an LP pass as a mark on the lanes as they are drawn: (row, kind, points). A rule between
    two lanes is drawn ACROSS, lane to lane: the LP's row runs along its column, which a steep lane crosses at a
    slant, so the offsets it separates can stand several bars apart along the column"""
    mp = film.mp
    vias = collections.defaultdict(list)
    for v in vs:
        vias[v[0]].append((v[1], v[2]))
    out = []
    for r in rows:
        pts = [mp.pt(*q) for q in r['pts']]
        ln = r['lanes']
        if r['kind'] in ('pitch', 'via', 'viavia') and len(ln) == 2 and ln[0] in st and ln[1] in st:
            if r['kind'] == 'pitch':
                p0 = nearest(st[ln[0]][0], pts[0])
            else:
                p0 = min(vias[ln[0]], key=lambda v: (v[0] - pts[0][0]) ** 2 + (v[1] - pts[0][1]) ** 2) if vias[ln[0]] else pts[0]
            if r['kind'] == 'viavia' and vias[ln[1]]:
                p1 = min(vias[ln[1]], key=lambda v: (v[0] - p0[0]) ** 2 + (v[1] - p0[1]) ** 2)
            else:
                p1 = nearest(st[ln[1]][0], p0)
            out.append((r, 'strut', [p0, p1]))
        elif r['wall']:
            out.append((r, 'wall', [nearest(st[ln[0]][0], pts[0]) if ln[0] in st else pts[0]]))
        else:
            out.append((r, 'dot', [nearest(st[ln[0]][0], pts[0]) if ln[0] in st else pts[0]]))
    return out


def pl(n, one, many=None):
    """'1 via', '2 vias'"""
    return f'{n} {one if n == 1 else (many or one + "s")}'


def ease(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3 - 2 * t)


def span(t, a, b):
    return min(1.0, max(0.0, (t - a) / (b - a))) if b > a else float(t >= a)


def lerp(a, b, t):
    return a + (b - a) * t


def mix(c1, c2, t):
    return tuple(int(round(lerp(x, y, t))) for x, y in zip(c1, c2))


class Plan:
    """one CP-SAT plan: crossings (frozenset -> u), changes (lane -> [u]), and its braid -- on more layers than two each
    lane's run layers between its changes (lane -> [layer name, in the plan's frame], as the solve writes 'layers')"""

    def __init__(self, cross, changes, S, layers=None):
        # (a solve file's crossing is {'u', 'ring'}, a traced plan's its u alone)
        self.cross = {frozenset(k.split('|')): float(v['u'] if isinstance(v, dict) else v) for k, v in cross.items()}
        self.changes = {n: sorted(map(float, v)) for n, v in changes.items()}
        self.layers = dict(layers or {})
        self.S = S

    def below(self, a, b, u):
        li = self.S.li
        k = frozenset((a, b))
        return (li[a] < li[b]) ^ (k in self.cross and self.cross[k] <= u + 1e-9)

    def run_layers(self, n):
        """lane n's runs' layers, the film's layer indices in the board's frame (None: a two-layer plan's bit)"""
        ls = self.layers.get(n)
        return [self.S.lidx(self.S.mp.layer(L)) for L in ls] if ls else None

    def layer(self, n, u):
        k = sum(1 for c in self.changes.get(n, ()) if c < u)
        rl = self.run_layers(n)
        return rl[min(k, len(rl) - 1)] if rl else self.S.tl[n] ^ (k & 1)


def lerp_plan(A, B, t):
    """the braid between two plans: every crossing and change at its interpolated u (a change the other plan has not
    kept is dropped at the half; a lane's runs' layers are the nearer plan's)"""
    P = Plan({}, {}, A.S)
    P.cross = {k: lerp(A.cross[k], B.cross.get(k, A.cross[k]), t) for k in A.cross}
    for n in set(A.changes) | set(B.changes):
        ca, cb = A.changes.get(n, []), B.changes.get(n, [])
        P.changes[n] = [lerp(x, y, t) for x, y in zip(ca, cb)] if len(ca) == len(cb) else (ca if t < 0.5 else cb)
    P.layers = dict(A.layers if t < 0.5 else B.layers)
    return P


# =============================================================================== colours and fonts
BG = (10, 11, 13)
PANEL = (20, 23, 28)
BODY = (26, 34, 28)
TEXT = (228, 232, 238)
DIM = (140, 148, 160)
ACCENT = (255, 210, 90)
F_COL = (240, 84, 66)
B_COL = (70, 156, 250)
LAYC = (F_COL, B_COL)
# an INNER layer's colour, In1.Cu on: bright, as the 3-D movie draws inner tracks -- yellow, sky, pink, then orange,
# green and violet (its white is a via's ring here)
INNER_COL = ((255, 226, 80), (110, 220, 255), (255, 140, 235), (255, 165, 70), (130, 235, 130), (185, 150, 255))


def _inner_k(name):
    m = re.match(r'In(\d+)\.Cu$', name or '')
    return int(m.group(1)) if m else 0


def layer_col(name):
    """a copper layer's colour: F.Cu red, B.Cu blue, an inner layer its INNER_COL"""
    if name == 'F.Cu':
        return F_COL
    if name == 'B.Cu':
        return B_COL
    k = _inner_k(name)
    return INNER_COL[(k - 1) % len(INNER_COL)] if k else (170, 170, 178)


def draw_order(names):
    """layers bottom to top, as the film draws them one over another: B.Cu, the inner layers from the deepest up, F.Cu"""
    inner = sorted((n for n in names if n not in ('F.Cu', 'B.Cu')), key=lambda n: (-_inner_k(n), n))
    return (['B.Cu'] if 'B.Cu' in names else []) + inner + (['F.Cu'] if 'F.Cu' in names else [])


def stack_order(names):
    """layers top to bottom, as the board stacks them: F.Cu, the inner layers, B.Cu"""
    return draw_order(names)[::-1]


_LAYERS = ['B.Cu', 'F.Cu']      # (the run's layers in draw_order: Film sets it -- the copper of every scene drawn so)
VIA_COL = (245, 245, 245)
PAIR_COL = (250, 214, 64)
FORCE = {'pitch': (255, 90, 210), 'via': (90, 230, 255), 'viavia': (90, 230, 255), 'bound': (255, 160, 40),
         'static': (255, 160, 40), 'turn': (150, 245, 120), 'pturn': (150, 245, 120), 'pdive': (150, 245, 120),
         'slope': (150, 245, 120), 'approach': (150, 245, 120)}
_FONTS = {}


def font(px, bold=False, mono=False):
    from PIL import ImageFont
    key = (px, bold, mono)
    if key not in _FONTS:
        paths = (['/System/Library/Fonts/Menlo.ttc', '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'] if mono else
                 ['/System/Library/Fonts/Supplemental/Arial Bold.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf']
                 if bold else ['/System/Library/Fonts/Supplemental/Arial.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'])
        f = None
        for p in paths:
            if os.path.isfile(p):
                f = ImageFont.truetype(p, px)
                break
        _FONTS[key] = f or ImageFont.load_default(size=px)
    return _FONTS[key]


# =============================================================================== the film's frame
W, H, SS = 1920, 1080, 2
HEAD = (20, 8, W - 20, 58)
CAP = (20, 62, W - 20, 122)
BOARD = (20, 126, W - 20, 700)
BRAID = (20, 750, W - 20, 1006)
FOOT = (20, 1014, W - 20, 1070)
STAGES = ['ends', 'strings', 'frame', 'solve', 'geometry', 'polish', 'snap', 'audit', 'lay']


class Film:
    """the story prepared for drawing: the board's camera, the braid's axes, every state of every lane"""

    def __init__(self, S, speed=1.0):
        self.S = S
        Gt, T = S.Gt, S.T
        self.mp = Mapper(Gt)
        mp = self.mp
        # ---- the routing layers, each a colour: a lane's layer is its index here (on two, F.Cu 0 and B.Cu 1 -- the
        # solve's own bit), drawn bottom to top in `order`
        self.LN = list(getattr(S, 'layers', None) or ['F.Cu', 'B.Cu'])
        self._layers_drawn()
        self.li = {n: i for i, n in enumerate(T['launch'])}
        self.fi = {n: i for i, n in enumerate(T['final_order'])}
        if T.get('layn'):
            # (more layers than two: each lane's tooth's and berth's layer by name)
            self.tl = {n: self.lidx(mp.layer(L)) for n, L in T['tooth_layer'].items()}
            self.dl = {n: self.lidx(mp.layer(L)) for n, L in T['dest_layer'].items()}
        else:
            self.tl = {n: mp.bit(int(v)) for n, v in T['tl'].items()}     # (the layer bits of the board's own frame)
            self.dl = {n: mp.bit(int(v)) for n, v in T['dl'].items()}
        self.M = S.M
        self.N = len(self.M)
        self.rules = Gt['rules']
        # ---- plans: CP-SAT's, in the order it found them; the one the passing round was laid on. (Not the plane
        # tie's, on more layers than two: it chooses the found plan's layers again by the planes they split -- its
        # crossings and changes held -- and its plan is the solve's, its final)
        runs_ = [r for r in T['runs'] if r.get('who') != '_plane_tie']
        self.plans = [Plan(p['cross'], p['changes'], self, p.get('layers')) for r in runs_ for p in r['plans']]
        self.plan_meta = [dict(p, workers=r['workers']) for r in runs_ for p in r['plans']]
        # (a long search finds plans by the hundred: the braid and the board draw a dozen, the first and the last
        # among them; the search's chart has every one)
        n_ = len(self.plans)
        self.pshow = sorted({int(round(j * (n_ - 1) / 11)) for j in range(12)}) if n_ > 12 else list(range(n_))
        self.root = Plan(T['final']['cross'], T['final']['changes'], self, T['final'].get('layers'))
        self.final = Plan(S.solve_round['cross'], S.solve_round['changes'], self, S.solve_round.get('layers'))
        # (a negotiated bus: the coupled solve's plan for it, the one its loop starts from)
        cp_ = getattr(S, 'coupled', None)
        self.coupled = Plan(cp_['cross'], cp_['changes'], self, cp_.get('layers')) if cp_ else None
        # ---- columns (the LP's own), their route coordinate and board point / normal
        self.cols = {}
        for c in Gt['cols']:
            b = mp.pt(*c['b'])
            nv = mp.vec(*c['n'])
            self.cols[(c['f'], c['k'])] = (c['u'], b, nv)
        self.ends = {n: (mp.pt(*Gt['tooth'][n]), mp.pt(*Gt['start'][n]), mp.pt(*Gt['land'][n]), mp.pt(*Gt['berth'][n]))
                     for n in self.M}
        self._camera()
        self._states()
        self._copper()
        self.g = self._group() if S.G else None
        self.neg = getattr(S, 'N', None)
        if self.neg:
            neg_prepare(self)
        # the layers the run's lanes stand on (the legend's, on more than two): their plans' and their copper's
        used = {self.LN[b] for st in self.st.values() for _P, bits in st.values() for b in set(bits.tolist())}
        used |= {s[5] for s in self.seq_c[0]} & set(self.LN)
        self.used = stack_order(used)

    def _layers_drawn(self):
        global _LAYERS
        self.col = [layer_col(L) for L in self.LN]
        self.order = [self.LN.index(L) for L in draw_order(self.LN)]
        _LAYERS = draw_order(self.LN)

    def lidx(self, name):
        """a layer's index among the film's (one the run did not name is added, its colour its own)"""
        if name not in self.LN:
            self.LN.append(name)
            self._layers_drawn()
        return self.LN.index(name)

    # -------------------------------------------------------------- camera
    def _camera(self):
        S, Gt, mp = self.S, self.S.Gt, self.mp
        pts = []
        for ref in (S.src, S.dst):
            fp = S.fo.footprints[ref]
            pts += [(p.global_x, p.global_y) for p in fp.pads]
        for st in (S.snapd, S.geo, S.emb):
            for n, v in (st['lanes'] if st else {}).items():
                pts += [mp.pt(*p) for p in v['xy']]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        # the braid's u, from the first entry to the last end, on the trunk's line: the board shows it all
        T = S.T
        self.u0 = min(T['entry'].values()) - 0.6
        self.u1 = max(T['end'].values()) + 0.6
        sp = [mp.pt(*p) for p in Gt['spine']]
        d = np.subtract(sp[-1], sp[0])
        d = d / np.hypot(*d)
        self.sp0, self.spd = np.array(sp[0], float), d
        for u in (self.u0, self.u1):
            q = self.sp0 + d * u
            xs.append(q[0])
        m = 0.7
        self.view = (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)
        bw, bh = BOARD[2] - BOARD[0], BOARD[3] - BOARD[1]
        wx, wy = self.view[2] - self.view[0], self.view[3] - self.view[1]
        # (the board itself is drawn by route_render's BoardRenderer on a canvas the panel's size, which keeps a
        # margin of the canvas's short side round the view: the camera is the same fit -- base_image checks the two)
        sys.path.insert(0, PYR)
        from route_render import CANVAS_MARGIN_FRAC
        mg = CANVAS_MARGIN_FRAC * min(bw, bh)
        self.sc = min((bw - 2 * mg) / wx, (bh - 2 * mg) / wy)   # px per mm (1x)
        self.ox = BOARD[0] + mg + (bw - 2 * mg - wx * self.sc) / 2 - self.view[0] * self.sc
        self.oy = BOARD[1] + mg + (bh - 2 * mg - wy * self.sc) / 2 - self.view[1] * self.sc
        # the braid: x of u where the trunk's point at u lands on the board; y of a rank
        # the board's x of u (the trunk's point at u)
        self.board_x = lambda u: self.X(*(self.sp0 + self.spd * u))[0] / SS
        # ...and the braid's: piecewise, the TRUNK between the arrays magnified -- every crossing and change is
        # there, a few millimetres of it -- the run-in along the source's face and the rings compressed; the funnel
        # between the two panels shows which stretch is which
        T = S.T
        h1 = max(T['Hn'].values()) if T['Hn'] else S.Gt['H0']
        self.ku = [self.u0, T['S_FACE'], h1 + 0.4, self.u1]
        xl, xr = BRAID[0] + 120, BRAID[2] - 30
        fr = np.cumsum([0.0, 0.16, 0.54, 0.30])
        self.kx = [xl + (xr - xl) * f for f in fr]
        self.bx = lambda u: float(np.interp(u, self.ku, self.kx))
        self.mag = (self.kx[2] - self.kx[1]) / max(1e-9, self.board_x(self.ku[2]) - self.board_x(self.ku[1]))
        self.down = S.Gt.get('chi', 1) >= 0
        top, bot = BRAID[1] + 22, BRAID[3] - 14
        self.rank_y = lambda r: (top + r * (bot - top) / max(1, self.N - 1)) if self.down else (bot - r * (bot - top) / max(1, self.N - 1))
        self.rstep = (bot - top) / max(1, self.N - 1)

    def X(self, x, y):
        """a board point in supersampled canvas pixels"""
        return ((self.ox + x * self.sc) * SS, (self.oy + y * self.sc) * SS)

    # -------------------------------------------------------------- lane states
    def _column_lane(self, n, keys, o_of, plan):
        """lane n through its columns keys [(f, k)], at offsets o_of(f, k): (points, per-segment bits, route u per
        point) -- from its tooth (and a pair's start past its end run) to its landing and berth"""
        tooth, start, land, berth = self.ends[n]
        pts, us = [], []
        for (f, k) in keys:
            u, b, nv = self.cols[(f, k)]
            o = o_of(f, k)
            pts.append((b[0] + o * nv[0], b[1] + o * nv[1]))
            us.append(u)
        head = [tooth] + ([start] if start != tooth else [])
        tail = [land] + ([berth] if berth != land else [])
        P = head + pts + tail
        U = [us[0]] * len(head) + us + [us[-1]] * len(tail)
        bits = [plan.layer(n, (U[j] + U[j + 1]) / 2 if U[j + 1] > U[j] else (U[j] + (1e-6 if j < len(head) else -1e-6)))
                for j in range(len(P) - 1)]
        return P, bits, U

    def slots(self, plan):
        """a plan drawn in the references' slots: at every column the present lanes' reference offsets, sorted, handed
        out in the plan's order there -- the solve's order and layers, no geometry"""
        Gt = self.S.Gt
        ref = {n: {(f, k): o for f, k, o in Gt['refs'][n]} for n in Gt['refs']}
        keys = {n: [(f, k) for f, k, _o in Gt['passes'][0]['lanes'][n]] for n in self.M}
        at = collections.defaultdict(list)
        for n in self.M:
            for fk in keys[n]:
                at[fk].append(n)
        slot = {}
        for fk, ns in at.items():
            u = self.cols[fk][0]
            od = sorted(ns, key=functools.cmp_to_key(lambda a, b: -1 if plan.below(a, b, u) else 1))
            # (the LP's own convention, every frame: a lane earlier in the order stands at the smaller offset --
            # whole_geo's pitch rows hold o_a + bar <= o_b for a before b)
            offs = sorted(ref[n][fk] for n in ns)
            for n, o in zip(od, offs):
                slot[(n,) + fk] = o
        return {n: self._column_lane(n, keys[n], lambda f, k, n=n: slot[(n, f, k)], plan) for n in self.M}

    def pass_lanes(self, p, plan):
        L = self.S.Gt['passes'][p]['lanes']
        out = {}
        for n in self.M:
            o = {(f, k): v for f, k, v in L[n]}
            out[n] = self._column_lane(n, [(f, k) for f, k, _v in L[n]], lambda f, k, o=o: o[(f, k)], plan)
        return out

    def json_lanes(self, J):
        """a plan file's lanes (geometry, polish, snap): (points, bits, None)"""
        mp = self.mp
        out = {}
        for n in self.M:
            v = J['lanes'].get(n)
            if not v or not v['pieces']:
                continue
            pts, bits = [mp.pt(*v['pieces'][0][:2])], []
            for x0, y0, x1, y1, L in v['pieces']:
                if math.hypot(x0 - pts[-1][0], y0 - pts[-1][1]) > 1e-6:
                    pts.append(mp.pt(x0, y0))
                    bits.append(self.lidx(mp.layer(L)))
                pts.append(mp.pt(x1, y1))
                bits.append(self.lidx(mp.layer(L)))
            out[n] = (pts, bits, None)
        return out

    def json_vias(self, J):
        return [(n, *self.mp.pt(x, y)) for n, x, y in J.get('vias', [])]

    def col_vias(self, lanes, plan):
        out = []
        for n, (P, _b, U) in lanes.items():
            for c in plan.changes.get(n, []):
                out.append((n, *along(P, U, c)))
        return out

    def _states(self):
        S = self.S
        raw = {}
        for i in self.pshow:
            raw[f'plan{i}'] = self.slots(self.plans[i])
        raw['root'] = self.slots(self.root)
        if self.coupled:
            raw['coupled'] = self.slots(self.coupled)
        raw['slots'] = self.slots(self.final)
        raw['p1'] = self.pass_lanes(0, self.final)
        raw['p2'] = self.pass_lanes(len(S.Gt['passes']) - 1, self.final)
        vias = {k: self.col_vias(v, self.plans[int(k[4:])] if k.startswith('plan') else
                                 {'root': self.root, 'coupled': self.coupled}.get(k, self.final)) for k, v in raw.items()}
        for key, J in (('geo', S.geo), ('emb', S.emb), ('pol', S.pol), ('pairs', S.pairsd), ('held', S.held),
                       ('snap', S.snapd)):
            if J:
                raw[key] = self.json_lanes(J)
                vias[key] = self.json_vias(J)
        # the taut strings (each from its tooth to its berth), and each one's chord -- what its relaxation starts from
        mp = self.mp
        self.strings = {n: [mp.pt(*p) for p in S.Gt['strings'][n]] for n in self.M if n in S.Gt['strings']}
        raw['string'] = {n: (P, [self.tl[n]] * (len(P) - 1), None) for n, P in self.strings.items()}
        raw['chord'] = {n: ([P[0], P[-1]], [self.tl[n]], None) for n, P in self.strings.items()}
        # ...and the relaxation itself, where the trace has it: every string's line after each recorded round -- its
        # chord first, the string as the stages read it last -- each at one count of points along its length
        self.relax, self.relax_at = None, []
        if S.Gt.get('relax'):
            rs = lambda P: resample(P, [0] * (len(P) - 1), 240)[0]
            cur = {n: rs([P[0], P[-1]]) for n, P in self.strings.items()}
            self.relax = [dict(cur)]
            for step, k, d in S.Gt['relax']:
                cur.update({n: rs([mp.pt(*q) for q in pts]) for n, pts in d.items() if n in cur})
                self.relax.append(dict(cur))
                self.relax_at.append((step, k, len(d)))
            self.relax.append({n: rs(P) for n, P in self.strings.items()})
            # (the film's time for each: by how many strings still move in it, the last few a beat between them)
            w = [0.15 + n_ / max(1, len(cur)) for _s, _k, n_ in self.relax_at] + [0.5]
            self.relax_t = np.concatenate([[0.0], np.cumsum(w)]) / sum(w)
        if S.Et:
            self._embed(raw, vias)
        for r in S.failed:
            J = json.load(open(os.path.join(S.loop, f'p{r["i"]}.json')))
            raw[f'fail{r["i"]}'] = self.json_lanes(J)
            vias[f'fail{r["i"]}'] = self.json_vias(J)
        # every state of a lane resampled to ONE count, from its longest: a morph is a point-for-point blend
        self.nres = {}
        for n in self.M:
            Lmax = max(plen(st[n][0]) for st in raw.values() if n in st)
            self.nres[n] = int(min(1600, max(120, Lmax / 0.03)))
        self.st = {}
        for key, st in raw.items():
            self.st[key] = {n: resample(P, bits, self.nres[n]) for n, (P, bits, _U) in st.items()}
        self.vias = vias
        self.raw = raw

    def _embed(self, raw, vias):
        """the 2-D geometry's record made ready for drawing, in the board's frame: its cells, its fixed copper, and
        its STANDS in turn -- the layout it starts from, the layout after each round, and after each slide -- each
        also as lanes (a lane's runs joined from tooth to berth) for the board's view"""
        E, mp = self.S.Et, self.mp
        P = lambda p: mp.pt(p[0], p[1])
        self.ebit = [self.lidx(mp.layer(name)) for name in E['layers']]     # (each of its layers, the film's index)
        self.erun = {k: (v['L'], P(v['lead']) if v['lead'] else None, P(v['tail']) if v['tail'] else None)
                     for k, v in E['runs'].items()}
        self.emesh = [(np.array([P(p) for p in m['V']], float).reshape(-1, 2), np.array(m['T'], int).reshape(-1, 3))
                      for m in E['mesh'] + [E['vmesh']]]              # (each layer's cells, then the vias' own)
        self.efixed = []
        for x0, y0, x1, y1, ls, lab in E['fixed']:
            (a0, b0), (a1, b1) = P((x0, y0)), P((x1, y1))
            self.efixed.append((min(a0, a1), min(b0, b1), max(a0, a1), max(b0, b1), ls, lab))

        def board(st):
            return {'xy': {k: [P(p) for p in v] for k, v in st['xy'].items()}, 'kind': st['kind'],
                    'via': {k: P(p) for k, p in st['via'].items()}, 'cell': st['cell'],
                    'barrel': {k: [P(p) for p in v] for k, v in st['barrel'].items()}}

        def lanes_of(st):
            out = {}
            for n in self.M:
                pts, bits, k = [], [], 0
                while f'{n}|{k}' in st['xy']:
                    L, lead, tail = self.erun[f'{n}|{k}']
                    xy = list(st['xy'][f'{n}|{k}'])
                    if k == 0 and lead:
                        xy = [lead] + xy
                    if tail and f'{n}|{k + 1}' not in st['xy']:
                        xy = xy + [tail]
                    if pts:
                        xy = xy[1:]                  # (a later run starts at the via the one before ends at)
                        bits += [self.ebit[L]] * len(xy)
                    else:
                        bits += [self.ebit[L]] * (len(xy) - 1)
                    pts += xy
                    k += 1
                if len(pts) > 1:
                    out[n] = (pts, bits, None)
            return out
        # the stands in turn: (state key, 'start' | 'round' | 'slide', its record, the stand)
        slides = {s['after']: s for s in E['slides']}
        self.eseq = [('e0', 'start', None, board(E['start']))]
        for j, r in enumerate(E['rounds']):
            if j in slides:
                self.eseq.append((f'es{j}', 'slide', slides[j], board(slides[j]['now'])))
            self.eseq.append((f'e{j + 1}', 'round', r, board(r['after'])))
        if len(E['rounds']) in slides:
            j = len(E['rounds'])
            self.eseq.append((f'es{j}', 'slide', slides[j], board(slides[j]['now'])))
        for key, _kind, _rec, st in self.eseq:
            raw[key] = lanes_of(st)
            vias[key] = [(k.split('|')[0], x, y) for k, (x, y) in st['via'].items()]
        self.elast = self.eseq[-1][0]

    def _copper(self):
        S = self.S
        run = set(S.nets)
        self.base_c = copper(S.base, run)
        self.fo_c = copper(S.fo, run)
        self.seq_c = copper(S.seq, run)
        self.human_c = copper(S.human, run) if S.human is not None else None
        # the fanout's changes: stubs the ends model moved away (in the round's input, not in its fanout) and laid
        self.stub_gone = minus(self.base_c[0], self.fo_c[0], _skey)
        self.stub_new = minus(self.fo_c[0], self.base_c[0], _skey)
        vkey = lambda v: (round(v[0] * 1000), round(v[1] * 1000), v[3])
        self.via_gone = minus(self.base_c[1], self.fo_c[1], vkey)
        self.via_new = minus(self.fo_c[1], self.base_c[1], vkey)
        self.stub_kept = minus(self.fo_c[0], self.stub_new, _skey)
        self.via_kept = minus(self.fo_c[1], self.via_new, vkey)
        # the routed copper: what the route added to the fanout, per lane
        rs = minus(self.seq_c[0], self.fo_c[0], _skey)
        rv = minus(self.seq_c[1], self.fo_c[1], vkey)
        self.routed = {n: ([s for s in rs if S.lane_of.get(s[6]) == n], [v for v in rv if S.lane_of.get(v[3]) == n])
                       for n in self.M}
        others = set(n.name.split('/')[-1] for n in S.fo.nets.values()) - run
        # (every other net's copper -- less what lies wholly off the canvas, which no frame draws)
        x0_, y0_ = -self.ox / self.sc - 1, -self.oy / self.sc - 1
        x1_, y1_ = (W - self.ox) / self.sc + 1, (H - self.oy) / self.sc + 1
        os_, ov_ = copper(S.fo, others)
        self.other_c = ([s for s in os_ if not (max(s[0], s[2]) < x0_ or min(s[0], s[2]) > x1_ or max(s[1], s[3]) < y0_
                                                or min(s[1], s[3]) > y1_)],
                        [v for v in ov_ if x0_ <= v[0] <= x1_ and y0_ <= v[1] <= y1_])
        # the source-side and destination-side halves of the new stubs
        DB = S.Gt['DB']
        near_dst = lambda x, y: DB[0] - 1.5 <= x <= DB[2] + 1.5 and DB[1] - 1.5 <= y <= DB[3] + 1.5
        self.teeth_new = [s for s in self.stub_new if not near_dst(s[0], s[1])]
        self.berths_new = [s for s in self.stub_new if near_dst(s[0], s[1])]
        self.tvia_new = [v for v in self.via_new if not near_dst(v[0], v[1])]
        self.bvia_new = [v for v in self.via_new if near_dst(v[0], v[1])]
        moved = sorted({s[6] for s in self.stub_gone} | {s[6] for s in self.teeth_new})
        self.moved_teeth = sorted({S.lane_of.get(m, m) for m in moved})
        # the numbers of the result, counted as whole_route.py counts them
        self.vias_ours = len(self.seq_c[1])
        self.mm_ours = copper_len(self.seq_c[0])
        if self.human_c:
            self.vias_human = len(self.human_c[1])
            self.mm_human = copper_len(self.human_c[0])

    # -------------------------------------------------------------- the group's ends
    def _group(self):
        """the group's ends made ready for drawing, in the bench's frame: each array's balls by what serves them, every
        state the model's solves held (each lane's tooth and berth as the escape it took: its pieces and its via), the
        field the model laid (its copper over the board it measured, the parts the nudge moved, the ties) -- all as the
        run recorded them"""
        import ast
        S, G = self.S, self.S.G
        tb = G.tb
        g = Story()
        # the two arrays' views (the source's and this bus's destination's): each its pads' box and a margin
        g.views = []
        for ref in (S.src, S.dst):
            P = [(p.global_x, p.global_y) for p in S.fo.footprints[ref].pads]
            xs, ys = [p[0] for p in P], [p[1] for p in P]
            g.views.append((ref, (min(xs) - 1.4, min(ys) - 1.4, max(xs) + 1.4, max(ys) + 1.4)))
        # each array's balls, by what the model serves them with: a bus's lane, another signal's escape, a plane's drop
        # or strap (group.json)
        g.balls = {'bus': [], 'other': [], 'plane': []}
        for ref, A in (G.json.get('arrays') or {}).items():
            for nm, e in (A.get('exits') or {}).items():
                legs = e.get('legs') or []
                if legs:
                    g.balls['other' if e.get('bus') == 'other signals' else 'bus'].append(tb(legs[0][0]))
            for d in (A.get('drops') or []) + (A.get('straps') or []):
                g.balls['plane'].append(tb(d['at']))

        import flow_frame as ff

        def bus_frame(bn):
            """a bus's ends model's frame onto this bench's: its states hold each bus's ends in that bus's own frame
            (its whole route's flow frame), which group.json gives by two of the source's pads placed in it -- the
            quarter turn and the shift that put them where this bench has them; where they do not fit, the board's
            point turned as the bench is (tb)"""
            B_ = (G.json.get('buses') or {}).get(bn) or {}
            fr, fp_ = B_.get('frame') or {}, S.fo.footprints.get(B_.get('src') or S.src)
            here = {p.pad_number: (p.global_x, p.global_y) for p in fp_.pads} if fp_ is not None else {}
            two = [n_ for n_ in sorted(fr) if n_ in here]
            if len(two) == 2:
                (a0, a1), (b0, b1) = fr[two[0]], fr[two[1]]
                (A0, A1), (B0, B1) = here[two[0]], here[two[1]]
                for q in range(4):
                    d_ = ff._q(b0 - a0, b1 - a1, q)
                    if abs(d_[0] - (B0 - A0)) < 1e-3 and abs(d_[1] - (B1 - A1)) < 1e-3:
                        return lambda p, q=q, a=(a0, a1), A=(A0, A1): tuple(
                            v_ + w_ for v_, w_ in zip(A, ff._q(float(p[0]) - a[0], float(p[1]) - a[1], q)))
            return tb

        def end_geo(e, to):
            """an end as the state holds it (its signature: each leg's kind, its via's site, its pieces), its points
            put in the bench's frame by `to`"""
            segs, vias = [], []
            try:
                legs = ast.literal_eval(e.get('sig') or '()')
            except (ValueError, SyntaxError):
                legs = ()
            for leg in legs:
                site, pieces = leg[5], leg[6]
                for x0, y0, x1, y1, L in pieces:
                    a, b = to((x0, y0)), to((x1, y1))
                    segs.append((a[0], a[1], b[0], b[1], L))
                if site is not None:
                    vias.append(to(site))
            return segs, vias
        g.states = []
        maps = {}
        for M_ in G.models:
            st, moved = {}, set()
            for bn, rec in M_['states'].items():
                to = maps[bn] = maps.get(bn) or bus_frame(bn)
                for ln, v in (rec.get('lanes') or {}).items():
                    for end in ('tooth', 'berth'):
                        if v.get(end):
                            st[(bn, ln, end)] = end_geo(v[end], to)
                for ln, ends_ in (rec.get('moved') or {}).items():
                    for end in ends_:
                        moved.add((bn, ln, end))
            g.states.append({'ends': st, 'moved': moved})
        # the field as laid: every net's copper on the field over the board its menus were measured on
        g.field_segs, g.field_vias, g.moved_parts = [], [], []
        if G.field is not None and G.prep is not None:
            alln = lambda pcb: {n.name.split('/')[-1] for n in pcb.nets.values() if n.name}
            fs, fv = copper(G.field, alln(G.field))
            ps, pv = copper(G.prep, alln(G.prep))
            vkey = lambda v: (round(v[0] * 1000), round(v[1] * 1000), v[3])
            for s in minus(fs, ps, _skey):
                a, b = tb(s[:2]), tb(s[2:4])
                g.field_segs.append((a[0], a[1], b[0], b[1], s[4], s[5], s[6]))
            for v in minus(fv, pv, vkey):
                g.field_vias.append((*tb(v[:2]), v[2], v[3]))
            for ref, fp in G.field.footprints.items():
                q = G.prep.footprints.get(ref)
                if q is not None and math.hypot(fp.x - q.x, fp.y - q.y) > 1e-3:
                    g.moved_parts.append((ref, [(tb((p.global_x, p.global_y)), p.size_x, p.size_y) for p in q.pads],
                                          [(tb((p.global_x, p.global_y)), p.size_x, p.size_y) for p in fp.pads]))
        g.ties = []
        for t_ in (G.ties or {}).get('tracks') or []:
            a, b = tb(t_['start']), tb(t_['end'])
            g.ties.append((a[0], a[1], b[0], b[1], t_.get('width', 0.1), t_['layer'], 'tie'))
        g.plane_nets = {d['net'] for A in (G.json.get('arrays') or {}).values()
                        for d in (A.get('drops') or []) + (A.get('straps') or [])}
        return g


# =============================================================================== drawing
class Canvas:
    def __init__(self, film, base):
        from PIL import ImageDraw
        self.f = film
        self.img = base.copy()
        self.d = ImageDraw.Draw(self.img)
        self.clipped = False

    def clip(self):
        """the board's overlays end at its panel: everything outside it painted over, once a frame, before the
        panels and cards are drawn"""
        if self.clipped:
            return
        self.clipped = True
        for box in ((0, 0, W, BOARD[1]), (0, BOARD[3], W, H), (0, 0, BOARD[0], H), (BOARD[2], 0, W, H)):
            self.d.rectangle([v * SS for v in box], fill=BG)

    # ---------------------------------------------------------------- primitives (1x coordinates in, SS drawn)
    def text(self, x, y, s, px=20, col=TEXT, bold=False, mono=False, anchor='la'):
        self.d.text((x * SS, y * SS), s, font=font(px * SS, bold, mono), fill=col, anchor=anchor)

    def tlen(self, s, px=20, bold=False, mono=False):
        return font(px * SS, bold, mono).getlength(s) / SS

    def wrap(self, s, px, width, bold=False):
        out, cur = [], ''
        for w in s.split(' '):
            t = (cur + ' ' + w).strip()
            if self.tlen(t, px, bold) > width and cur:
                out.append(cur)
                cur = w
            else:
                cur = t
        return out + ([cur] if cur else [])

    def rect(self, box, fill=None, outline=None, width=1, r=0):
        b = [v * SS for v in box]
        if r:
            self.d.rounded_rectangle(b, radius=r * SS, fill=fill, outline=outline, width=int(width * SS))
        else:
            self.d.rectangle(b, fill=fill, outline=outline, width=int(width * SS))

    def card(self, box, alpha=0.86, border=(70, 78, 92)):
        from PIL import Image
        self.clip()
        b = tuple(int(v * SS) for v in box)
        reg = self.img.crop(b)
        self.img.paste(Image.blend(reg, Image.new('RGB', reg.size, PANEL), alpha), b[:2])
        self.rect(box, outline=border, width=1, r=8)

    def line(self, pts, col, w=1.0):
        self.d.line([(x * SS, y * SS) for x, y in pts], fill=col, width=max(1, int(round(w * SS))), joint='curve')

    # ---------------------------------------------------------------- board-space primitives
    def wline(self, pts, col, w_px, joint=True):
        f = self.f
        P = [f.X(x, y) for x, y in pts]
        if len(P) >= 2:
            self.d.line(P, fill=col, width=max(1, int(round(w_px * SS))), joint='curve' if joint else None)

    def wring(self, x, y, r_mm, col, w_px=1.5, fill=None):
        cx, cy = self.f.X(x, y)
        r = max(1.5 * SS, r_mm * self.f.sc * SS)
        self.d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=col, width=max(1, int(w_px * SS)), fill=fill)

    def wdot(self, x, y, r_px, col):
        cx, cy = self.f.X(x, y)
        r = r_px * SS
        self.d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)

    def wtext(self, x, y, s, px=12, col=TEXT, anchor='la', bold=False):
        cx, cy = self.f.X(x, y)
        self.d.text((cx, cy), s, font=font(px * SS, bold), fill=col, anchor=anchor)

    def lanes(self, st, alpha=1.0, w=2.6, only=None, dim=0.0, ribbon=False, hl=None):
        """lanes of a state {n: (points, bits)} -- B under F, pairs on a ribbon; `only` lanes at full strength,
        the rest at `dim`"""
        f = self.f
        prs = f.S.pairs
        order = [n for n in f.M if n in st]
        a_of = lambda n: alpha * (1.0 if only is None or n in only else dim)
        if ribbon:
            for n in order:
                if n in prs and a_of(n) > 0.02:
                    P = st[n][0]
                    wr = (f.rules['PP'] + f.rules['TW']) * f.sc
                    self.d.line([f.X(x, y) for x, y in P], fill=mix(BODY, (110, 92, 26), a_of(n)),
                                width=max(1, int(wr * SS)), joint='curve')
        for layer in f.order:
            for n in order:
                a = a_of(n)
                if a <= 0.02:
                    continue
                P, bits = st[n]
                col = mix(BODY, f.col[layer], a)
                ww = w * (1.6 if hl and n in hl else 1.0)
                j0 = None
                for j in range(len(bits) + 1):
                    on = j < len(bits) and bits[j] == layer
                    if on and j0 is None:
                        j0 = j
                    elif not on and j0 is not None:
                        self.wline(P[j0:j + 1], col, ww)
                        j0 = None

    def vias(self, vs, alpha=1.0, only=None, dim=0.0, r_mm=None):
        f = self.f
        r = (r_mm if r_mm is not None else f.rules['via'] / 2)
        for n, x, y in vs:
            a = alpha * (1.0 if only is None or n in only else dim)
            if a > 0.02:
                self.wring(x, y, r, mix(BODY, VIA_COL, a), 1.6)

    def segs(self, segs, alpha=1.0, true_width=True, w_px=2.0, col=None):
        f = self.f
        for layer in _LAYERS:
            c = col or layer_col(layer)
            c = mix(BODY, c, alpha)
            for s in segs:
                if s[5] != layer:
                    continue
                w = s[4] * f.sc if true_width else w_px
                self.d.line([f.X(s[0], s[1]), f.X(s[2], s[3])], fill=c, width=max(1, int(round(w * SS))))
                if true_width:
                    for (x, y) in ((s[0], s[1]), (s[2], s[3])):
                        cx, cy = f.X(x, y)
                        r = w * SS / 2
                        self.d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=c)

    def cvias(self, vias, alpha=1.0):
        for x, y, size, _n in vias:
            cx, cy = self.f.X(x, y)
            r = size / 2 * self.f.sc * SS
            self.d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=mix(BODY, (200, 200, 200), alpha))
            r2 = r * 0.55
            self.d.ellipse([cx - r2, cy - r2, cx + r2, cy + r2], fill=mix(BODY, (30, 30, 30), alpha))


def morph(A, B, t):
    """two resampled states, blended point for point (the layer of the nearer end)"""
    out = {}
    for n in A:
        if n not in B:
            continue
        (PA, bA), (PB, bB) = A[n], B[n]
        out[n] = (PA + (PB - PA) * t, bA if t < 0.5 else bB)
    return out


def morph_vias(VA, VB, t):
    """two via lists blended: each lane's vias matched in order along x; the unmatched fade (alpha returned)"""
    ga, gb = collections.defaultdict(list), collections.defaultdict(list)
    for n, x, y in VA:
        ga[n].append((x, y))
    for n, x, y in VB:
        gb[n].append((x, y))
    out = []
    for n in set(ga) | set(gb):
        a, b = sorted(ga[n]), sorted(gb[n])
        for j in range(max(len(a), len(b))):
            if j < len(a) and j < len(b):
                out.append((n, lerp(a[j][0], b[j][0], t), lerp(a[j][1], b[j][1], t), 1.0))
            elif j < len(a):
                out.append((n, a[j][0], a[j][1], 1.0 - t))
            else:
                out.append((n, b[j][0], b[j][1], t))
    return out


# =============================================================================== the braid panel
_STRANDS = {}


def braid_strands(film, plan, w=0.11):
    key = id(plan)
    if key in _STRANDS and _STRANDS[key][0] is plan:
        return _STRANDS[key][1]
    out = _braid_strands(film, plan, w)
    if plan in film.plans or plan is film.final or plan is film.root:
        _STRANDS[key] = (plan, out)
    return out


def _braid_strands(film, plan, w=0.11):
    """every lane's strand: u samples, y (rank) at each, layer bit at each -- a strand's rank is its launch rank
    plus a ramp of one rank at each of its crossings (down past a lane launched south of it, up past one launched
    north), so the strands are a consistent permutation everywhere"""
    T = film.S.T
    per = collections.defaultdict(list)
    li = film.li
    for k, u in plan.cross.items():
        a, b = tuple(k)
        per[a].append((u, 1 if li[a] < li[b] else -1))
        per[b].append((u, 1 if li[b] < li[a] else -1))
    out = {}
    for n in film.M:
        u0, u1 = T['entry'][n], T['end'][n]
        us = np.arange(u0, u1 + 1e-9, 0.025)
        if us[-1] < u1:
            us = np.append(us, u1)
        y = np.full(len(us), float(li[n]))
        for (ux, sg) in per[n]:
            y += sg * np.clip((us - ux) / (2 * w) + 0.5, 0.0, 1.0)
        ch = np.array(plan.changes.get(n, []), float)
        rl = plan.run_layers(n)
        if rl:
            # (more layers than two: the run's own layer, between the changes)
            bits = np.asarray(rl, int)[np.minimum(np.searchsorted(ch, us, side='left'), len(rl) - 1)]
        else:
            bits = film.tl[n] ^ (np.searchsorted(ch, us, side='left') & 1) if len(ch) else np.full(len(us), film.tl[n])
        out[n] = (us, y, bits)
    return out


def magnify_view(box, view, title):
    """the magnifier's world view: `view`'s centre and width, its height the box's own aspect"""
    bx0, by0, bx1, by1 = box[0] + 6, box[1] + (26 if title else 6), box[2] - 6, box[3] - 6
    cx, cy, hw = (view[0] + view[2]) / 2, (view[1] + view[3]) / 2, (view[2] - view[0]) / 2
    hh = hw * (by1 - by0) / (bx1 - bx0)
    return (cx - hw, cy - hh, cx + hw, cy + hh)


def magnify(C, film, box, view, draw_fn, title=None, frame=True):
    """a magnifier: a region of the board drawn into `box` on an image of its own (so it clips itself) -- its pads,
    then draw_fn(draw, Z, s): Z maps board mm to the magnifier's pixels, s its pixels per mm (both supersampled)"""
    from PIL import Image, ImageDraw
    C.card(box, alpha=0.95)
    view = magnify_view(box, view, title)
    bx0, by0, bx1, by1 = box[0] + 6, box[1] + (26 if title else 6), box[2] - 6, box[3] - 6
    wpx, hpx = int((bx1 - bx0) * SS), int((by1 - by0) * SS)
    s = wpx / (view[2] - view[0])
    img = Image.new('RGB', (wpx, hpx), BODY)
    d = ImageDraw.Draw(img)
    Z = lambda x, y: ((x - view[0]) * s, (y - view[1]) * s)
    for fp in film.S.fo.footprints.values():
        for p in fp.pads:
            if view[0] - 1 < p.global_x < view[2] + 1 and view[1] - 1 < p.global_y < view[3] + 1 and p.pad_type != 'np_thru_hole':
                (x0, y0), (x1, y1) = Z(p.global_x - p.size_x / 2, p.global_y - p.size_y / 2), Z(p.global_x + p.size_x / 2, p.global_y + p.size_y / 2)
                if (p.shape or '').lower() == 'circle':
                    d.ellipse([x0, y0, x1, y1], fill=(150, 131, 75))
                else:
                    d.rounded_rectangle([x0, y0, x1, y1], radius=min(x1 - x0, y1 - y0) * 0.25, fill=(150, 131, 75))
    draw_fn(d, Z, s)
    C.img.paste(img, (int(bx0 * SS), int(by0 * SS)))
    if title:
        C.text(box[0] + 12, box[1] + 6, title, 13, TEXT)
    if frame:
        (a0, b0), (a1, b1) = film.X(view[0], view[1]), film.X(view[2], view[3])
        if BOARD[0] * SS <= a0 and a1 <= BOARD[2] * SS:
            C.d.rectangle([a0, b0, a1, b1], outline=ACCENT, width=2 * SS)
    return view


def funnel(C, film, a=1.0):
    """the braid's u against the board's: the magnified trunk as a funnel from the board's panel to the braid's"""
    f = film
    y0, y1 = BOARD[3], BRAID[1]
    bx0, bx1 = f.board_x(f.ku[1]), f.board_x(f.ku[2])
    C.d.polygon([(bx0 * SS, y0 * SS), (bx1 * SS, y0 * SS), (f.kx[2] * SS, y1 * SS), (f.kx[1] * SS, y1 * SS)],
                fill=mix(BG, (40, 44, 34), a))
    for u, x in zip(f.ku, f.kx):
        C.line([(f.board_x(u), y0), (x, y1)], mix(BG, (110, 104, 70), a), 1)
    C.line([(bx0, y0 - 3), (bx0, y0), (bx1, y0), (bx1, y0 - 3)], mix(BG, ACCENT, a), 1.5)
    C.text((f.kx[1] + f.kx[2]) / 2, (y0 + y1) / 2, f'the trunk between the arrays, magnified x{f.mag:.1f}', 12,
           mix(BG, (200, 190, 130), a), anchor='mm')


def draw_braid(C, film, plan, alpha=1.0, cursor=None, cuts=None, labels=True, hl=None, marks=True, vias_alpha=1.0):
    f = film
    T = f.S.T
    C.card(BRAID, alpha=0.92)
    funnel(C, f)
    st = braid_strands(f, plan)
    X = f.bx
    # the face band: no crossing and no change there
    xb0, xb1 = X(T['S_FACE']), X(T['BAND'])
    C.rect((xb0, BRAID[1] + 4, xb1, BRAID[3] - 4), fill=(34, 38, 46))
    C.text((xb0 + xb1) / 2, BRAID[1] + 6, 'face band', 11, DIM, anchor='ma')
    xh = X(T['Hn'][next(iter(T['Hn']))]) if T['Hn'] else X(f.S.Gt['H0'])
    C.line([(xh, BRAID[1] + 16), (xh, BRAID[3] - 4)], (70, 76, 88), 1)
    C.text(xh + 4, BRAID[1] + 6, 'rings (handoff)', 11, DIM)
    # u ticks
    for u in range(int(math.ceil(f.u0)), int(f.u1) + 1):
        x = X(u)
        C.line([(x, BRAID[3] - 6), (x, BRAID[3] - 2)], (90, 96, 108), 1)
        if u % 5 == 0:
            C.text(x, BRAID[3] - 7, f'u = {u} mm', 10, DIM, anchor='mb')
    if cuts:
        for (n, ulo, uhi, kind) in cuts:
            if n not in st:
                continue
            y = f.rank_y(float(np.interp((ulo + uhi) / 2, st[n][0], st[n][1])))
            col = (255, 110, 60) if kind == 'island' else (255, 60, 200)
            C.rect((X(ulo), y - 4, X(uhi), y + 4), outline=col, width=1.5)
    for layer in f.order:
        for n in f.M:
            us, y, bits = st[n]
            a = alpha * (1.0 if not hl or n in hl else 0.25)
            col = mix(PANEL, f.col[layer], a)
            pts = [(X(u), f.rank_y(r)) for u, r in zip(us, y)]
            j0 = None
            for j in range(len(bits) + 1):
                on = j < len(bits) and bits[j] == layer
                if on and j0 is None:
                    j0 = j
                elif not on and j0 is not None:
                    C.line(pts[max(0, j0 - 1):j + 1] if j0 else pts[j0:j + 1], col, 1.8 if n not in f.S.pairs else 3.2)
                    j0 = None
    # ends, labels, vias
    for n in f.M:
        us, y, _b = st[n]
        x0, y0 = X(us[0]), f.rank_y(y[0])
        C.d.ellipse([(x0 - 2.5) * SS, (y0 - 2.5) * SS, (x0 + 2.5) * SS, (y0 + 2.5) * SS], fill=mix(PANEL, TEXT, alpha))
        x1, y1 = X(us[-1]), f.rank_y(y[-1])
        C.d.ellipse([(x1 - 2.5) * SS, (y1 - 2.5) * SS, (x1 + 2.5) * SS, (y1 + 2.5) * SS], fill=mix(PANEL, TEXT, alpha))
        if labels:
            C.text(x0 - 5, y0, n, 10, mix(PANEL, PAIR_COL if n in f.S.pairs else DIM, alpha), anchor='rm')
    if marks:
        for n in f.M:
            us, y, _b = st[n]
            for c in plan.changes.get(n, []):
                r = float(np.interp(c, us, y))
                x, yy = X(c), f.rank_y(r)
                R = 4.2
                C.d.ellipse([(x - R) * SS, (yy - R) * SS, (x + R) * SS, (yy + R) * SS],
                            outline=mix(PANEL, VIA_COL, alpha * vias_alpha), width=int(1.6 * SS))
    if cursor is not None:
        x = X(cursor)
        C.line([(x, BRAID[1] + 14), (x, BRAID[3] - 8)], ACCENT, 1.2)
    return st


# =============================================================================== the scenes
class Scene:
    def __init__(self, key, stage, secs, fn, captions, says=(), cues=()):
        self.key, self.stage, self.secs, self.fn, self.captions = key, stage, secs, fn, captions
        # (the sound, --sound: the scene's narration [(t, words[, t_end])], each line spoken from its moment t of the
        # scene and done by its t_end where what it speaks of leaves the screen there, and its cues [(t, effect[, x[,
        # t_end]])], each a thing the scene draws at t -- whole_movie_sound.py)
        self.says, self.cues = list(says), list(cues)
        self.knots = None           # (film seconds into the scene, t) where the narration's pace was fitted

    @property
    def span(self):
        """the scene's length on film: its own, or with the narration fitted (whole_movie_sound.fit) the longer"""
        return self.knots[-1][0] if self.knots else self.secs

    def at(self, s):
        """the scene's moment t (0 to 1) s seconds into it on film"""
        if not self.knots:
            return min(1.0, s / self.secs)
        return float(np.interp(s, [k[0] for k in self.knots], [k[1] for k in self.knots]))


def spoken(name):
    """a part's or a net's name as the narration says it: letter by letter, its numbers whole (AB12: A B 12)"""
    return ' '.join(re.findall(r'\d+|[A-Za-z]', name)) or name


# ------------------------------------------------------------------------------- the 2-D geometry's scenes
EVC = {'vhop': (90, 230, 255), 'bulge': (255, 90, 210), 'pull': (150, 245, 120)}      # an event's colour, by its kind
SHORT_C, TIGHT_C = (255, 40, 40), (255, 170, 60)


def embed_scenes(film, sc, draw_fanout, forces, P2):
    """the scenes of the round's 2-D geometry (whole_embed), appended to sc: the frame geometry's layout it starts
    from, its cells, the lanes' own points, the rounds, a closer look at one event of each kind the rounds had, and
    what stands at the end. Every number, place and layout drawn is the stage's own (_trace_embed); a move between two
    of its layouts is drawn as a blend of the two"""
    S, E, Gt = film.S, film.S.Et, film.S.Gt
    ER, rl, seq = E['rounds'], E['rules'], film.eseq
    NR = len(ER)
    sidx = {s[0]: i for i, s in enumerate(seq)}
    nlay = len(E['layers'])
    BP = lambda p: film.mp.pt(p[0], p[1])                  # a recorded point in the board's frame
    bname = lambda L: film.LN[film.ebit[L]]                # a layer of the record as the board names it
    lane_of = lambda key: key.split('|')[0]
    CELLC = [mix(BODY, film.col[b], 0.6) for b in film.ebit] + [(170, 170, 178)]
    VL = len(film.emesh) - 1                               # (the vias' own cells: the last mesh)
    prs = set(S.pairs)
    boxA = (BRAID[0], BRAID[1], BRAID[0] + 620, BRAID[3])
    boxB = (BRAID[0] + 640, BRAID[1], BRAID[0] + 1260, BRAID[3])
    boxT = (BRAID[0] + 1280, BRAID[1], BRAID[2], BRAID[3])
    asp = (boxA[3] - boxA[1] - 32) / (boxA[2] - boxA[0] - 12)          # a titled magnifier's height over its width
    win = lambda c, w: (c[0] - w / 2, c[1] - w * asp / 2, c[0] + w / 2, c[1] + w * asp / 2)
    mesh_px = [np.array([film.X(x, y) for x, y in V], float).reshape(-1, 2) for V, _T in film.emesh]

    def draw_cells(C, L, a, upto=1.0):
        """mesh L's cells on the board (each layer's, then the vias' own): the ones left of `upto` of the panel"""
        if a <= 0.02 or upto <= 0:
            return
        px, T_ = mesh_px[L], film.emesh[L][1]
        col = mix(BODY, CELLC[L], a)
        lim = (BOARD[0] + (BOARD[2] - BOARD[0]) * upto) * SS
        for i, j, k in T_:
            if (px[i][0] + px[j][0] + px[k][0]) / 3 <= lim:
                C.d.line([tuple(px[i]), tuple(px[j]), tuple(px[k]), tuple(px[i])], fill=col, width=SS)

    def facts(C, box, title, lines, a=1.0):
        C.card(box, alpha=0.95)
        C.text(box[0] + 14, box[1] + 10, title, 17, mix(PANEL, ACCENT, a), bold=True)
        y = box[1] + 40
        for ln, col in lines:
            for w_ in C.wrap(ln, 14, box[2] - box[0] - 28):
                if nlay > 2 and y > box[3] - 20:
                    return                  # (more layers than two, more lines: the card holds what fits)
                C.text(box[0] + 14, y, w_, 14, mix(PANEL, col, a))
                y += 19
            y += 4

    def busiest(pts, w, h):
        """the middle of the w x h window that holds the most of pts"""
        P = np.asarray(pts, float).reshape(-1, 2)
        if not len(P):
            return ((film.view[0] + film.view[2]) / 2, (film.view[1] + film.view[3]) / 2)
        keys, cnt = np.unique(np.round(P / 0.5).astype(int), axis=0, return_counts=True)
        inside = lambda c: (np.abs(P[:, 0] - c[0]) < w / 2) & (np.abs(P[:, 1] - c[1]) < h / 2)
        best = max((tuple(k * 0.5) for k in keys[np.argsort(-cnt)[:40]]), key=lambda c: int(inside(c).sum()))
        near = P[inside(best)]
        return (float(near[:, 0].mean()), float(near[:, 1].mean()))

    def close(C, box, view, st, title, L=None, vcells=False, hl=None, hlv=(), pts=True, marks=None, dim=0.6):
        """the 2-D geometry up close in `box`: mesh L's cells (the vias' own with `vcells`), every run of the stand
        `st` at its copper's width and the vias at theirs; the runs `hl` (None: every run) and the vias `hlv` lit, the
        runs of `hl` with their own points (white squares on cell edges, yellow dots free); then marks(d, Z, s, st)"""
        TW_, PP_ = rl['TW'], rl['PP']
        view_ = magnify_view(box, view, bool(title))

        def draw(d, Z, s):
            v = view_
            for Lc in ([L] if L is not None else []) + ([VL] if vcells else []):
                V, T_ = film.emesh[Lc]
                tri = V[T_]
                lo, hi = tri.min(axis=1), tri.max(axis=1)
                keep = ~((hi[:, 0] < v[0]) | (lo[:, 0] > v[2]) | (hi[:, 1] < v[1]) | (lo[:, 1] > v[3]))
                col = mix(BODY, CELLC[Lc], 0.95)
                for t3 in tri[keep]:
                    q = [Z(*p) for p in t3]
                    d.line(q + [q[0]], fill=col, width=int(1.5 * SS))
            for bit in film.order:
                for key, xy in st['xy'].items():
                    Lr, lead, tail = film.erun[key]
                    if film.ebit[Lr] != bit:
                        continue
                    A = np.asarray(xy, float)
                    if A[:, 0].max() < v[0] - 1 or A[:, 0].min() > v[2] + 1 or A[:, 1].max() < v[1] - 1 \
                            or A[:, 1].min() > v[3] + 1:
                        continue
                    n = lane_of(key)
                    line = ([lead] if lead and key.endswith('|0') else []) + list(xy) \
                        + ([tail] if tail and f'{n}|{int(key.split("|")[1]) + 1}' not in st['xy'] else [])
                    a_ = (1.0 if (hl is None or key in hl) else dim) * (0.62 if n in prs else 1.0)
                    d.line([Z(*p) for p in line], fill=mix(BODY, film.col[bit], a_),
                           width=max(1, int((PP_ + TW_ if n in prs else TW_) * s)), joint='curve')
            if pts:
                r = max(3.0 * SS, 0.022 * s)
                for key in hl or ():
                    for (x, y), k in zip(st['xy'].get(key, [])[1:-1], st['kind'].get(key, '')):
                        if not (v[0] < x < v[2] and v[1] < y < v[3]):
                            continue
                        cx, cy = Z(x, y)
                        if k == 'P':
                            d.rectangle([cx - r, cy - r, cx + r, cy + r], fill=(250, 250, 250), outline=(20, 20, 20))
                        elif k == 'F':
                            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(255, 226, 110), outline=(20, 20, 20))
                        elif k == 'D':                      # (the end of a pair's dive: a diamond)
                            d.polygon([(cx, cy - 1.4 * r), (cx + 1.4 * r, cy), (cx, cy + 1.4 * r), (cx - 1.4 * r, cy)],
                                      fill=PAIR_COL, outline=(20, 20, 20))
                        else:
                            d.line([cx - 2 * r, cy - 2 * r, cx + 2 * r, cy + 2 * r], fill=SHORT_C, width=2 * SS)
                            d.line([cx - 2 * r, cy + 2 * r, cx + 2 * r, cy - 2 * r], fill=SHORT_C, width=2 * SS)
            rv = E['via_size'] / 2 * s
            for key, (x, y) in st['via'].items():
                if v[0] - 0.5 < x < v[2] + 0.5 and v[1] - 0.5 < y < v[3] + 0.5:
                    on = key in hlv
                    # (a pair's via is its barrels, one a leg, either side of the pair's line; a crossover's two stand
                    # on one side of the pair, and the stage keeps room for them on either: both sides hollow)
                    bl = st['barrel'].get(key) or [(x, y)]
                    for bx, by in bl:
                        cx, cy = Z(bx, by)
                        d.ellipse([cx - rv, cy - rv, cx + rv, cy + rv], fill=None if len(bl) > 2 else (58, 58, 62),
                                  outline=ACCENT if on else (240, 240, 240), width=int((3 if on else 1.5) * SS))
            if marks:
                marks(d, Z, s, st)
        magnify(C, film, box, view, draw, title)

    def blend(A, B, t, view):
        """the stand between two, for the runs near `view`: each run's line blended point for point along its
        length, each via likewise (its points' kinds left out: they are one stand's or the other's)"""
        if t <= 0:
            return A
        if t >= 1:
            return B
        xy = {}
        for k, a in A['xy'].items():
            b = B['xy'].get(k, a)
            pa = np.asarray(a, float)
            if pa[:, 0].max() < view[0] - 2 or pa[:, 0].min() > view[2] + 2 or pa[:, 1].max() < view[1] - 2 \
                    or pa[:, 1].min() > view[3] + 2:
                continue
            n = 6 * max(len(a), len(b), 2)
            qa, _ = resample(a, [0] * (len(a) - 1), n)
            qb, _ = resample(b, [0] * (len(b) - 1), n)
            xy[k] = [tuple(q) for q in qa + (qb - qa) * t]
        via = {k: (lerp(a[0], B['via'][k][0], t), lerp(a[1], B['via'][k][1], t)) for k, a in A['via'].items()}
        # (a pair's barrels move with its via; where its heading turns between the two stands, they turn at the half)
        bar = {k: [(q[0] - A['via'][k][0] + via[k][0], q[1] - A['via'][k][1] + via[k][1]) for q in a] if t < 0.5 else
                  [(q[0] - B['via'][k][0] + via[k][0], q[1] - B['via'][k][1] + via[k][1]) for q in B['barrel'].get(k, a)]
               for k, a in A['barrel'].items()}
        return {'xy': xy, 'kind': {}, 'via': via, 'cell': A['cell'], 'barrel': bar}

    # ------------------------------------------------------------------ the start: the frame geometry's layout
    npaid = sum(P2.get('paid', {}).values())

    def start(C, t, T_):
        draw_fanout(C, 0.5)
        m = ease(span(t, 0.04, 0.42))
        st = morph(film.st['slots'], film.st['p2'], m)
        vs = morph_vias(film.vias['slots'], film.vias['p2'], m)
        C.lanes(st, w=2.2)
        C.vias([(n, x, y) for n, x, y, al in vs])
        a_f = ease(span(t, 0.52, 0.62))
        if a_f > 0:
            forces(C, [mk for mk in force_marks(film, P2['rows'], st, vs) if mk[0]['paid'] > 1e-4], a_f)
        draw_braid(C, film, film.final, alpha=0.9)
        box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 440, BOARD[1] + 190)
        C.card(box)
        C.text(box[0] + 16, box[1] + 12, 'the start: a layout in the frame', 19, ACCENT, bold=True)
        lines = [f'{P2["nvar"]} offsets: one for each lane at each', f'   column, every {Gt["G"]:.2f} mm along its route',
                 'each via at the place along the route', '   the solve gave it', f'one linear program, {P2["nrow"]} rows',
                 f'{pl(npaid, "rule")} it could not meet (red)' if npaid else 'every rule of its own met']
        for j, ln in enumerate(lines):
            C.text(box[0] + 16, box[1] + 44 + j * 21, ln, 14, (255, 110, 110) if (j == 5 and npaid) else TEXT)
    sc.append(Scene('start', 'geometry', 10, start, [
        (0, f'The geometry starts in the frame. ONE linear program gives every lane an offset at every column '
            f'({Gt["G"]:.2f} mm apart) along the trunk and its ring, in the solve\'s order, on its solved layers: the '
            f'order becomes millimetres.'),
        (0.48, 'But there a lane can only shift sideways at its columns, and every via stays at the place along the '
               'route the solve gave it.' + (f' Where that does not fit the board, the program pays ({pl(npaid, "rule")}, '
                                             f'red).' if npaid else '')
               + ' This layout is now only the START of the geometry.')],
        says=[(0, 'The geometry begins in the frame: one linear program turns the order into millimetres.'),
              (0.48, ('But lanes can only shift sideways here, so some rules go unmet, in red.' if npaid else
                      'But lanes can only shift sideways here, and every via stays put.') + ' It is only a start.')],
        cues=[(0.04, 'glide', 0.4, 0.42)] + ([(0.52, 'alert', 0.4)] if npaid else [])))   # (the layout; its paid rules)

    # ------------------------------------------------------------------ the cells
    e0 = seq[0][3]
    hots = [BP(q) for r in ER[1:] for q in r['hot']] or [BP(q) for r in ER for q in r['hot']] or list(e0['via'].values())
    cwin = win(busiest(hots, 2.8, 2.8 * asp), 2.8)
    # (...and a wider one for the cells: about the pad of another part the most of the start's lanes pass by)
    part = [((x0 + x1) / 2, (y0 + y1) / 2) for (x0, y0, x1, y1, _ls, lab) in film.efixed if ' ' not in lab]
    lane_xy = np.array([q for xy in e0['xy'].values() for q in xy], float).reshape(-1, 2)
    by = lambda c: int(((np.abs(lane_xy[:, 0] - c[0]) < 3.0) & (np.abs(lane_xy[:, 1] - c[1]) < 3.0 * asp)).sum())
    cwide = win(max(part, key=by) if part else busiest(hots, 6.0, 6.0 * asp), 6.0)
    ncell = [len(T_) for _V, T_ in film.emesh]
    nmesh = len(film.emesh)
    m_ = re.search(r'fixed copper: (\d+) piece', ' '.join(E['log'][:3]))
    nfix = int(m_.group(1)) if m_ else len(E['fixed'])
    slot = 0.86 / nmesh

    def cells(C, t, T_):
        draw_fanout(C, 0.5)
        C.lanes(film.st['e0'], alpha=0.45, w=1.6)
        for L in range(nmesh):
            t0 = 0.06 + L * slot
            later = ease(span(t, t0 + slot, t0 + slot + 0.05)) if L < nmesh - 1 else 0.0
            draw_cells(C, L, 0.95 - 0.55 * later, upto=span(t, t0, t0 + 0.6 * slot))
        # (the layers up close two at a time, as the sweep reaches them: the pair of the last layer it has begun)
        p_ = max([L for L in range(nlay) if t >= 0.06 + L * slot] or [0]) // 2
        for box, L in ((boxA, 2 * p_), (boxB, min(2 * p_ + 1, nlay - 1))):
            if t >= 0.06 + L * slot:
                close(C, box, cwide, e0, f'{bname(L)} up close: its free space, cut into cells', L=L, dim=0.3, pts=False,
                      hl=set())
            else:
                C.card(box, alpha=0.95)
        if t >= 0.06 + VL * slot:
            close(C, boxT, cwide, e0, 'the vias\' own free space, up close', vcells=True, dim=0.3, pts=False, hl=set())
        else:
            facts(C, boxT, 'cells', [(f'{nfix} pieces of fixed copper: pads, vias, tracks, the end of every stub', TEXT)]
                  + [(f'{bname(L)}: {ncell[L]} cells', film.col[film.ebit[L]]) for L in range(nlay)
                     if t >= 0.06 + L * slot + 0.6 * slot]
                  + ([('every corner of a cell is a corner of the copper\'s margin: no point stands where nothing is',
                       DIM)] if nlay <= 2 else []))
    # (each layer its share of the sweep: on more layers than two the scene is longer)
    sc.append(Scene('cells', 'geometry', 12 if nmesh <= 3 else 12 + 1.5 * (nmesh - 3), cells, [
        (0, 'The 2-D geometry works in board xy, not in the frame. A lane\'s centre keeps a margin off every piece '
            'of fixed copper: half a track, the clearance, half a grid step. What is left of a layer inside the board '
            'is its FREE SPACE.'),
        (0.06 + 0.3 * slot, 'Each layer\'s free space is cut into triangles whose corners are corners of that margin: '
                            'CELLS (' + ', '.join(f'{bname(L)} {ncell[L]}' for L in range(nlay)) + '). The cells a lane '
                            'passes say which side of each piece of copper it goes -- and nothing more.'),
        (0.06 + VL * slot, f'A via needs its room on every layer at once, so the vias have a free space of their own, '
                           f'cut the same way ({ncell[VL]} cells).')],
        says=[(0, 'Now the geometry works on the board itself. Each layer\'s free space is cut into triangles: cells.'),
              (0.06 + VL * slot, 'Vias need room on every layer at once, so they get cells of their own.')],
        cues=[(0.06 + L * slot, 'sweep', L / max(1, nmesh - 1), 0.06 + L * slot + 0.6 * slot) for L in range(nmesh)]))

    # ------------------------------------------------------------------ the lanes' own points
    kinds0 = ''.join(e0['kind'].values())
    nP, nF, nD = kinds0.count('P'), kinds0.count('F'), kinds0.count('D')
    KINDC = {'P': (250, 250, 250), 'F': (255, 226, 110), 'D': PAIR_COL}
    dots0 = [(k, x, y) for key, xy in e0['xy'].items() for (x, y), k in zip(xy[1:-1], e0['kind'][key])]
    on_layer = [{key for key in e0['xy'] if film.erun[key][0] == L} for L in range(nlay)]
    # (up close, the two layers that carry the most runs -- on two layers, the two)
    two_ = sorted(sorted(range(nlay), key=lambda L: (-len(on_layer[L]), L))[:2]) if nlay > 2 else [0, min(1, nlay - 1)]

    def points(C, t, T_):
        draw_fanout(C, 0.5)
        m = ease(span(t, 0.03, 0.22))
        for L in range(nlay):
            draw_cells(C, L, 0.3)
        C.lanes(morph(film.st['geo'], film.st['e0'], m), w=2.0)
        C.vias(film.vias['e0'] if m > 0.5 else film.vias['geo'])
        a_p = ease(span(t, 0.28, 0.4))
        if a_p > 0:
            for k, x, y in dots0:
                C.wdot(x, y, 1.3, mix(BODY, KINDC.get(k, SHORT_C), a_p))
        for box, L in ((boxA, two_[0]), (boxB, two_[-1])):
            close(C, box, cwin, e0, f'{bname(L)} up close: each run\'s own points', L=L, hl=on_layer[L], pts=a_p > 0.5)
        facts(C, boxT, 'the lanes\' own points', [
            (f'{len(e0["xy"])} runs of {len(film.M)} lanes, {len(e0["via"])} vias', TEXT),
            (f'{nP} points on cell edges (white): each slides along its edge', (250, 250, 250)),
            (f'{nF} free points (yellow): each moves across its lane, inside its cell', (255, 226, 110))]
            + ([(f'{nD} ends of a pair\'s dive (gold): each a fixed step from the pair\'s via, and moves with it', PAIR_COL)]
               if nD else [])
            + ([('a pair\'s via is two barrels, one a leg; where the pair crosses over, room is kept for them on either '
                 'side of it (hollow)', DIM)] if any(len(v) > 2 for v in e0['barrel'].values()) else []) + [
            (f'{E["nmouth"]} of {2 * len(film.M)} lane ends leave straight through their own stub', DIM)]
            + ([(f'{pl(len(E["breach"]), "breach", "breaches")}: the start crosses copper no free cells lead round', SHORT_C)]
               if E['breach'] else []))
    sc.append(Scene('points', 'geometry', 12, points, [
        (0, 'The start is read onto the cells. A RUN is a lane\'s stretch on one layer, between its ends and its vias. '
            'The chain of cells it crosses says which side of each piece of copper it passes; inside them it is free.'),
        (0.28, f'Each run carries its own points: one where it crosses each cell edge (white, {nP} of them), sliding '
               f'along the edge, and free points between (yellow, {nF}), moving across the lane inside their cell. A '
               f'via is a free point too, in a cell of the vias\' space.'),
        (0.7, 'Where the start stood inside the margin of some copper its line is read out to the free space, the '
              'runs kept in the order they had. And a lane leaves its tooth, and reaches its berth, straight along its '
              'stub.')],
        says=[(0, 'The starting layout is read onto the cells.'),
              (0.28, 'White points slide along cell edges; yellow points move freely inside their cells.'),
              (0.7, 'And each lane leaves and arrives straight along its stub.')],
        cues=[(0.03, 'glide', 0.3, 0.22), (0.28, 'shimmer', 0.7)]))          # (the start read on; its points)

    # ------------------------------------------------------------------ the rounds
    # the film's time for each stand: the first rounds slow (they move the most), a slide a beat of its own
    wts = [6.0 if kind == 'slide' else 4.0 if rec_['it'] <= 3 else 2.0 if rec_['it'] <= 10 else 1.0
           for (_k, kind, rec_, _st) in seq[1:]]
    cumw = np.concatenate([[0.0], np.cumsum(wts)])
    top = max([r['nlive'] for r in ER] + [1])
    slide_at = {s['after']: s for s in E['slides']}
    slid_txt = lambda n: f'{pl(n, "via")} slid along ' + ('its lane' if n == 1 else 'their lanes')
    tot = {k: sum(r[k] for r in ER) for k in ('hops', 'nexc', 'npull')}
    rwin = cwin
    boxR = (BRAID[0] + 1340, BRAID[1], BRAID[2], BRAID[3])
    px_of = lambda pts: [film.X(*BP(q)) for q in pts]
    lit_px = {r['it']: px_of(r['lit']) for r in ER}
    hot_px = {r['it']: px_of(r['hot']) for r in ER}

    def chart(C, done, x0, x1):
        """the rounds as bars, `done` of them run (a fraction: the round under way): the points each round's program
        held and the places that moved in it, and beneath them its events"""
        y0, y1 = BRAID[1] + 50, BRAID[3] - 62
        bw_ = (x1 - x0) / max(1, NR)
        C.rect((x0, BRAID[1] + 12, x0 + 12, BRAID[1] + 24), fill=(96, 104, 118))
        C.text(x0 + 18, BRAID[1] + 10, 'the points a round\'s program holds', 12, DIM)
        C.rect((x0 + 270, BRAID[1] + 12, x0 + 282, BRAID[1] + 24), fill=ACCENT)
        C.text(x0 + 288, BRAID[1] + 10, 'the places that moved in it', 12, DIM)
        C.text(x0 - 8, y0, str(top), 11, DIM, anchor='rm')
        C.text(x0 - 8, y1, '0', 11, DIM, anchor='rm')
        C.line([(x0, y0), (x0, y1), (x1, y1)], (70, 76, 88), 1)
        for row, (kind, lab) in enumerate((('vhop', 'a via changes cell'), ('bulge', 'a point into the next cell'),
                                           ('pull', 'a run pulled taut'))):
            C.text(x0 - 8, y1 + 10 + row * 14, lab, 11, EVC[kind], anchor='rm')
        for j, r in enumerate(ER):
            f = min(1.0, max(0.0, done - j))
            if f <= 0:
                break
            xa, xb = x0 + j * bw_ + 1, x0 + (j + 1) * bw_ - 1
            C.rect((xa, y1 - (y1 - y0) * r['nlive'] / top * f, xb, y1), fill=(96, 104, 118))
            h2 = (y1 - y0) * min(top, len(r['hot'])) / top * f
            if h2 >= 0.5:
                C.rect((xa + (xb - xa) * 0.25, y1 - h2, xb - (xb - xa) * 0.25, y1), fill=ACCENT)
            for row, (kind, cnt) in enumerate((('vhop', r['hops']), ('bulge', r['nexc']), ('pull', r['npull']))):
                if cnt and f >= 1:
                    hh = min(12.0, 2.0 + 2.4 * math.log2(1 + cnt))
                    yb = y1 + 16 + row * 14
                    C.rect((xa, yb - hh, xb, yb), fill=EVC[kind])
        for j in range(0, NR, 10 if NR > 30 else 5 if NR > 10 else 1):
            C.text(x0 + (j + 0.5) * bw_, BRAID[3] - 6, f'round {j + 1}', 10, DIM, anchor='mb')
        for after, s in slide_at.items():
            if done >= after:
                x = x0 + after * bw_
                C.line([(x, y0 - 6), (x, y1 + 46)], ACCENT, 1.5)
                C.text(x + 5 if after < NR * 0.6 else x - 5, y0 - 2, slid_txt(len(s['vias'])), 12,
                       ACCENT, anchor='lt' if after < NR * 0.6 else 'rt')

    def rounds(C, t, T_):
        draw_fanout(C, 0.5)
        u = span(t, 0.04, 0.92)
        if u < 1:
            x = u * cumw[-1]
            i = int(min(len(wts) - 1, np.searchsorted(cumw, x, side='right') - 1))
            f = (x - cumw[i]) / wts[i]
        else:
            i, f = len(wts) - 1, 1.0
        (ka, _ka, _ra, sa), (kb, kind, rec_, sb) = seq[i], seq[i + 1]
        m = ease(span(f, 0.2, 0.8))
        for L in range(nlay):
            draw_cells(C, L, 0.26)
        C.lanes(morph(film.st[ka], film.st[kb], m), w=2.0)
        C.vias([(n, x_, y_) for n, x_, y_, al in morph_vias(film.vias[ka], film.vias[kb], m)])
        done = sum(1 for s_ in seq[1:i + 1] if s_[1] == 'round')
        if kind == 'round':
            a_l = 1 - ease(span(f, 0.55, 1.0))
            c1 = mix(BODY, (235, 238, 245), a_l)
            for (x_, y_) in lit_px[rec_['it']]:
                C.d.rectangle([x_ - SS / 2, y_ - SS / 2, x_ + SS / 2, y_ + SS / 2], fill=c1)
            a_h = ease(span(f, 0.25, 0.5)) * (1 - ease(span(f, 0.85, 1.0)))
            c2 = mix(BODY, ACCENT, a_h)
            if a_h > 0.02:
                for (x_, y_) in hot_px[rec_['it']]:
                    C.d.ellipse([x_ - 1.6 * SS, y_ - 1.6 * SS, x_ + 1.6 * SS, y_ + 1.6 * SS], fill=c2)
                for h in rec_['vhop']:
                    C.wring(*BP(h['to']), 0.22, mix(BODY, EVC['vhop'], a_h), 1.6)
                for b in rec_['bulge']:
                    C.wring(*BP(b['xy']), 0.14, mix(BODY, EVC['bulge'], a_h), 1.4)
                for q in rec_['pulled']:
                    C.wring(*BP(q), 0.14, mix(BODY, EVC['pull'], a_h), 1.4)
            done += f
        else:
            a_s = math.sin(math.pi * span(f, 0.0, 1.0))
            for vk in rec_['vias']:
                p0, p1 = sa['via'][vk], sb['via'][vk]
                C.wline([p0, p1], mix(BODY, ACCENT, a_s), 2.0)
                C.wring(*p1, 0.4, mix(BODY, ACCENT, a_s), 2.5)
        C.card((BRAID[0], BRAID[1], boxR[0] - 10, BRAID[3]), alpha=0.95)
        chart(C, done if u < 1 else NR, BRAID[0] + 180, BRAID[0] + 960)
        x0 = BRAID[0] + 990
        if kind == 'round':
            sd = rec_['standing']
            C.text(x0, BRAID[1] + 12, f'round {rec_["it"]} of {NR}', 20, ACCENT, bold=True)
            lines = [f'{rec_["nlive"]} points and {rec_["nvia"]} vias in its program',
                     f'{rec_["nvar"]} variables, {rec_["nrow"]} rows',
                     f'the largest move: {rec_["move"]:.3f} mm',
                     f'{rec_["hops"]} via and {rec_["nexc"]} lane cell changes',
                     f'{pl(rec_["npull"], "run")} pulled taut',
                     f'short of a rule: {sd["pitch"][0]} pitch, {sd["via"][0] + sd["viavia"][0]} via',
                     f'the lanes\' length: {rec_["length"]:.0f} mm']
        else:
            C.text(x0, BRAID[1] + 12, 'between the rounds', 20, ACCENT, bold=True)
            lines = [f'{pl(len(rec_["vias"]), "via")} short of room', 'slid along their own lanes',
                     'to a more open place:', '   ' + ', '.join(lane_of(v) for v in rec_['vias'])[:40],
                     'then the rounds again from there']
        for j, ln in enumerate(lines):
            C.text(x0, BRAID[1] + 46 + j * 22, ln, 14, TEXT if j < 3 else DIM)
        close(C, boxR, rwin, blend(sa, sb, m, magnify_view(boxR, rwin, True)), 'up close', pts=False)
    cap_r = [(0, 'ONE linear program a round places every via and every lane\'s points together: the shortest and '
                 'straightest lanes, two lanes of a layer a pitch apart, every lane a via\'s room from every other '
                 'lane\'s via, the vias their own distance apart.'),
             (0.2, f'Each rule is held as a straight line read off the layout as it stands, so a point moves only so '
                   f'far a round ({rl["TRUST"]:.2f} mm) and the program is run again. A rule may fall short, at a '
                   f'price -- but nothing passes through anything: lanes, vias and copper keep their sides.'),
             (0.45, 'A round\'s program holds only what stands near where the round before moved: it shrinks as the '
                    'layout settles (white, the points it holds; yellow, the places that moved), and the rounds end '
                    'when nothing moves.'),
             (0.72, f'Over the {NR} rounds a via changed cell {tot["hops"]} times, a lane\'s point went on into the '
                    f'next cell {tot["nexc"]} times, {tot["npull"]} stretches were pulled taut'
                    + (f', and {pl(len(E["slid"]), "via")} slid along its lane' if len(E['slid']) == 1 else
                       f', and {len(E["slid"])} vias were slid along their lanes' if E['slid'] else '')
                    + '. A closer look at one of each follows.')]
    did = ([f'vias changed cell {tot["hops"]:,} times'] if tot['hops'] else []) \
        + ([f'{pl(tot["npull"], "stretch", "stretches")} {"was" if tot["npull"] == 1 else "were"} pulled taut']
           if tot['npull'] else []) \
        + ([f'{pl(len(E["slid"]), "via")} slid along ' + ('its lane' if len(E['slid']) == 1 else 'their lanes')]
           if E['slid'] else [])
    say_r = [(0, 'Then, round after round, one linear program moves every via and every point together.'),
             (0.2, 'Each round moves things only a little, and nothing passes through anything: lanes, vias and copper '
                   'keep their sides.'),
             (0.45, 'Each round holds only what is near the last round\'s moves, so the program shrinks as the layout '
                    'settles.'),
             (0.72, f'Over {NR:,} rounds, ' + (', '.join(did[:-1]) + ' and ' + did[-1] if len(did) > 1 else did[0])
              + '.' if did else f'{pl(NR, "round")} in all.')]
    # (each stand in turn, at its share of the scene: a round of the program -- the louder, the more points it holds
    # -- and its via cell changes lit; or a via slid along its lane)
    cue_r = []
    for i, (_k, kind, rec_, _st) in enumerate(seq[1:]):
        t_i = 0.04 + 0.88 * cumw[i] / cumw[-1]
        if kind == 'round':
            cue_r.append((t_i, 'round', rec_['nlive'] / top))
            if rec_['hops']:
                cue_r.append((0.04 + 0.88 * (cumw[i] + 0.25 * wts[i]) / cumw[-1], 'blip', min(1.0, rec_['hops'] / 12)))
        else:
            cue_r.append((t_i, 'slide', 0.5, 0.04 + 0.88 * cumw[i + 1] / cumw[-1]))
    sc.append(Scene('rounds', 'geometry', 24, rounds, cap_r, says=say_r, cues=cue_r))

    # ------------------------------------------------------------------ one event of each kind, up close
    def closeup(key, secs, board_key, view, sA, sB, ta, tb, kw_a, kw_b, title, lines, caps, sC=None, tc=None, says=(),
                cues=()):
        """a scene of one event: the board, and beneath it the stand before (left) and the move to the stand after
        (right, from t 0.38 to 0.62) -- then, with sC, on to a third (the layout the rounds that follow reach, from
        0.76 to 0.92)"""
        def fn(C, t, T_):
            draw_fanout(C, 0.5)
            for L in range(nlay):
                draw_cells(C, L, 0.2)
            C.lanes(film.st[board_key], w=2.0)
            C.vias(film.vias[board_key])
            vb = magnify_view(boxB, view, True)
            m = ease(span(t, 0.38, 0.62))
            m2 = ease(span(t, 0.76, 0.92)) if sC is not None else 0.0
            close(C, boxA, view, sA, ta, **kw_a)
            if m2 > 0:
                close(C, boxB, view, sC if m2 >= 1 else blend(sB, sC, m2, vb), tc if m2 >= 1 else tb, **dict(kw_b, pts=False))
            else:
                close(C, boxB, view, sB if m >= 1 else blend(sA, sB, m, vb), tb if m >= 1 else 'the move',
                      **(kw_b if m >= 1 else dict(kw_b, pts=False)))
            facts(C, boxT, title, lines)
        sc.append(Scene(key, 'geometry', secs, fn, caps, says=says, cues=cues))

    def tri_marks(Lm, was, now):
        """a cell left (dim) and the cell gone into (bright), outlined"""
        V, T_ = film.emesh[Lm]

        def f(d, Z, s, st, both=True):
            if both and now is not None:
                q = [Z(*p) for p in V[T_[now]]]
                d.line(q + [q[0]], fill=ACCENT, width=3 * SS)
            q = [Z(*p) for p in V[T_[was]]]
            d.line(q + [q[0]], fill=ACCENT if not both else (150, 130, 70), width=(3 if not both else 2) * SS)
        return f

    def fat(Lm, t):
        """a cell's width: twice its area over its longest side (a sliver's is next to nothing)"""
        a, b, c = film.emesh[Lm][0][film.emesh[Lm][1][t]]
        return abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) \
            / max(math.hypot(*(b - a)), math.hypot(*(c - b)), math.hypot(*(a - c)), 1e-9)

    def tri_view(Lm, ts, c, lo, hi):
        """a window about c that takes in the cells ts of mesh Lm, between lo and hi wide"""
        V, T_ = film.emesh[Lm]
        P = np.concatenate([V[T_[t]] for t in ts])
        ext = max(2 * np.abs(P[:, 0] - c[0]).max(), 2 * np.abs(P[:, 1] - c[1]).max() / asp)
        return win(c, min(hi, max(lo, 1.15 * ext)))
    before_of = lambda j: seq[sidx[f'e{j + 1}'] - 1]                    # the stand round j (from 0) starts from
    dist = lambda a, b: math.hypot(b[0] - a[0], b[1] - a[1])
    # (the event drawn of each kind is one the eye can follow: the largest move between two cells wide enough to see)
    plain = lambda Lm, t0, t1, d: min(fat(Lm, t0), fat(Lm, t1), 1.0) * d

    # ---- a via changes cell
    hops = [(plain(VL, h['was'][0], h['now'][0], dist(h['from'], h['to'])), j, h)
            for j, r in enumerate(ER) for h in r['vhop'] if h['was'][0] != h['now'][0] and not h['own']]
    if hops:
        _d, j, h = max(hops, key=lambda e: e[0])
        r = ER[j]
        sA, sB = before_of(j)[3], seq[sidx[f'e{j + 1}']][3]
        vk = h['via']
        pa, pb = sA['via'][vk], sB['via'][vk]
        tm = tri_marks(VL, h['was'][0], h['now'][0])
        hl = {vk, f'{lane_of(vk)}|{int(vk.split("|")[1]) + 1}'}
        closeup('hop', 8, f'e{j + 1}',
                tri_view(VL, (h['was'][0], h['now'][0]), ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2), 1.7, 4.5),
                sA, sB, f'before round {r["it"]}: {lane_of(vk)}\'s via in its cell (outlined)',
                f'after round {r["it"]}: in the next cell',
                dict(vcells=True, hl=hl, hlv={vk}, pts=False, marks=lambda d, Z, s, st, tm=tm: tm(d, Z, s, st, False)),
                dict(vcells=True, hl=hl, hlv={vk}, pts=False, marks=tm),
                'a via changes cell', [
                    (f'round {r["it"]}: {lane_of(vk)}\'s via', TEXT),
                    (f'moved {dist(pa, pb):.3f} mm, out of the cell it stood in and into its neighbour', TEXT),
                    (f'{pl(r["hops"], "such change")} in this round, {tot["hops"]} over all {NR}', DIM),
                    ('grey lines: the cells of the vias\' own free space', (170, 170, 178))],
                [(0, 'A via changes cell. A via is held inside its cell of the vias\' free space: that keeps it out of '
                     'the copper\'s margin with straight-line rules alone.'),
                 (0.38, 'When the program presses it on an edge of that cell, with free space beyond, it goes on into the '
                        'next cell. So a via is not tied to the cell it starts in: it moves on, cell by cell, wherever '
                        'there is room for it.')],
                says=[(0, 'A via changes cell. It is held inside its cell.'),
                      (0.38, 'Pressed against an edge, with free space beyond, it moves on into the next cell.')],
                cues=[(0.38, 'blip', 0.6)])

    # ---- a lane's point goes on into the next cell
    bul = [(plain(b['L'], b['from'], b['to'], dist(b['was'], b['xy'])), j, b) for j, r in enumerate(ER) for b in r['bulge']]
    if bul:
        _d, j, b = max(bul, key=lambda e: e[0])
        r = ER[j]
        sA, sB = before_of(j)[3], seq[sidx[f'e{j + 1}']][3]
        tm = tri_marks(b['L'], b['from'], b['to'])
        closeup('bulge', 8, f'e{j + 1}', tri_view(b['L'], (b['from'], b['to']), BP(b['xy']), 1.3, 4.0), sA, sB,
                f'before round {r["it"]}: {lane_of(b["run"])} on {bname(b["L"])}, and the cell one of its points is in',
                f'after round {r["it"]}: that point in the next cell',
                dict(L=b['L'], hl={b['run']}, marks=lambda d, Z, s, st, tm=tm: tm(d, Z, s, st, False)),
                dict(L=b['L'], hl={b['run']}, marks=tm),
                'a lane bulges into the next cell', [
                    (f'round {r["it"]}: a free point of {lane_of(b["run"])} on {bname(b["L"])}', TEXT),
                    (f'moved {dist(b["was"], b["xy"]):.3f} mm, pressed on an edge of its cell (outlined) with free space '
                     f'beyond', TEXT),
                    ('the run now crosses that edge and comes back, or its crossing of it moves past the point', TEXT),
                    (f'{r["nexc"]} such in this round, {tot["nexc"]} over all {NR}', DIM)],
                [(0, 'A lane bulges into the next cell. A free point moves only inside its cell: that is what keeps a '
                     'run in the free space. Pushed aside by a neighbour or a via, it comes to press on an edge of the '
                     'cell.'),
                 (0.38, 'With free space beyond that edge, the point goes on into the cell beyond, and the run with it. '
                        'The chain of cells a run crosses is not fixed: only the copper is.')],
                says=[(0, 'A lane bulges. Its free points are held inside their cells too.'),
                      (0.38, 'Pushed to an edge, a point crosses into the next cell, and the lane goes with it.')],
                cues=[(0.38, 'boing', 0.5)])

    # ---- a run pulled taut
    pulls = [(plen(p['before']) - plen(p['after']), j, p) for j, r in enumerate(ER) for p in r['pull']]
    if pulls:
        saved, j, p = max(pulls, key=lambda e: e[0])
        r = ER[j]
        sB = seq[sidx[f'e{j + 1}']][3]
        sA = dict(sB, xy=dict(sB['xy'], **{p['run']: [BP(q) for q in p['before']]}), kind=dict(sB['kind'], **{p['run']: ''}))
        aft = {tuple(q) for q in p['after']}
        gone = np.array([BP(q) for q in p['before'] if tuple(q) not in aft] or [BP(q) for q in p['before']], float)
        ext = max(np.ptp(gone[:, 0]), np.ptp(gone[:, 1]) / asp)
        closeup('pull', 8, f'e{j + 1}', win((float(gone[:, 0].mean()), float(gone[:, 1].mean())), max(1.3, 1.9 * ext)),
                sA, sB, f'round {r["it"]}, after its program: {lane_of(p["run"])} on {bname(p["L"])} doubles back',
                f'round {r["it"]}: pulled taut',
                dict(L=p['L'], hl={p['run']}, pts=False), dict(L=p['L'], hl={p['run']}),
                'a run pulled taut', [
                    (f'round {r["it"]}: {lane_of(p["run"])} on {bname(p["L"])}', TEXT),
                    (f'{saved:.2f} mm shorter', TEXT),
                    ('a stretch half as long again as the straight line between its ends, with nothing inside the loop '
                     '-- no copper, no other run of its layer, no via -- becomes that straight line', TEXT),
                    (f'{r["npull"]} pulled in this round, {tot["npull"]} over all {NR}', DIM)],
                [(0, 'A run pulled taut. A via that moves drags its run behind it along the way it came, and a lane that '
                     'bulged into a cell and came back keeps the bulge: loops with nothing in them, which small moves '
                     'never undo.'),
                 (0.38, 'After each round\'s program every run is looked at for such a stretch, and where nothing stands '
                        'inside the loop -- and the straight line comes no nearer anything than its rule -- it is pulled '
                        'straight.')],
                says=[(0, 'A run pulled taut. Moving vias and bulges leave empty loops behind.'),
                      (0.38, 'After each round, a loop with nothing inside it is pulled straight.')],
                cues=[(0.38, 'zip', 0.5)])

    # ---- a via slid along its lane
    if E['slides'] and E['slides'][0]['vias']:
        s = E['slides'][0]
        si = sidx[f'es{s["after"]}']
        sA, sB = seq[si - 1][3], seq[si][3]
        vk = s['vias'][0]
        n, kv = lane_of(vk), int(vk.split('|')[1])
        pa, pb = sA['via'][vk], sB['via'][vk]
        lg = next((re.search(r'slid (on|back) along its lane from .*?: ([\d.]+) -> ([\d.]+) off', ln) for ln in E['log']
                   if ln.strip().startswith(f"{n}'s via {kv} slid")), None)
        still_short = any(x['kind'] != 'pitch' and vk in x['what'] for x in E['short'])

        def room(d, Z, s_, st, vk=vk):
            x, y = st['via'][vk]
            cx, cy = Z(x, y)
            rr = rl['VIA_R'] * s_
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=EVC['vhop'], width=int(1.5 * SS))
        hl = {vk, f'{n}|{kv + 1}'}
        # (the circle is a single's via's room; a pair's via is two barrels, each with a room of its own: none drawn)
        own = bool(sA['barrel'].get(vk))
        mk = None if own else room
        closeup('slide', 12, seq[si][0], win(((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2), max(1.8, 2.6 * dist(pa, pb))),
                sA, sB, f'after round {s["after"]}: {n}\'s via, short of its room' + ('' if own else ' (the circle)'),
                'slid along its own lane',
                dict(hl=hl, hlv={vk}, pts=False, marks=mk), dict(hl=hl, hlv={vk}, pts=False, marks=mk),
                'a via slid along its lane', [
                    (f'{n}\'s via: {dist(pa, pb):.2f} mm along its own lane', TEXT)]
                + ([(f'off the nearest copper\'s margin: {float(lg.group(2)):.2f} mm before, {float(lg.group(3)):.2f} mm '
                     f'after', TEXT)] if lg else [])
                + [('the stretch between changes layer with it (its colour)', TEXT)]
                + ([('a pair\'s via: its two barrels, one a leg', PAIR_COL)] if own else
                   [('circle: the room a via needs from the middle of every other lane', EVC['vhop'])])
                + [('last, the layout after the rounds that follow' + ('' if still_short else ': its room clear'), TEXT),
                   (f'{pl(len(E["slid"]), "via")} slid in all: ' + ', '.join(lane_of(v) for v in E['slid']), DIM)],
                [(0, 'A via slid along its lane. In a corridor too narrow for a via and the lane that must pass it, no '
                     'small step gains anything: the rounds leave that via short of its room. So it is moved the one '
                     'way the program cannot move it.'),
                 (0.38, 'It slides along its own lane to the most open place within reach, the stretch between changing '
                        'layer with it -- allowed wherever that stretch lies in the other layer\'s free space and crosses '
                        'no lane there.'),
                 (0.74, 'Then the rounds again, from there: the lanes round the via move off it'
                        + ('.' if still_short else ', and it has its room.'))],
                sC=seq[-1][3], tc=f'after round {NR}',
                says=[(0, 'A via slides. In a corridor too narrow, small steps cannot give it its room.'),
                      (0.38, 'So it slides along its own lane, to the most open place within reach.'),
                      (0.74, 'Then the rounds run again, from there' + ('.' if still_short else ', and it has its room.'))],
                cues=[(0.38, 'slide', 0.5, 0.62), (0.76, 'glide', 0.6, 0.92)])

    # ---- a breach
    if E['breach']:
        b = E['breach'][0]
        pa, pb = BP(b['from']), BP(b['to'])

        def wall(d, Z, s_, st, pa=pa, pb=pb):
            d.line([Z(*pa), Z(*pb)], fill=SHORT_C, width=3 * SS)
        kw = dict(L=b['L'], hl={b['run']}, marks=wall)
        closeup('breach', 8, 'e0', win(((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2), max(1.6, 3 * dist(pa, pb))),
                e0, seq[-1][3], f'the start: {lane_of(b["run"])} on {bname(b["L"])} crosses copper', 'after the rounds',
                kw, kw, 'a breach', [
                    (f'{lane_of(b["run"])} on {bname(b["L"])}: across ' + (', '.join(b['copper'][:3]) or 'fixed copper'), TEXT),
                    ('no free cells of that layer lead round it: the run is read as it stands, two fixed points either '
                     'side (red)', TEXT),
                    (f'{pl(len(E["breach"]), "breach", "breaches")} in this plan, each a layer cut for the solve', DIM)],
                [(0, 'A breach. Here the plan asks a lane to pass copper on a layer whose free space does not lead '
                     'round it -- the two sides are two separate pieces of that layer.'),
                 (0.38, 'No geometry can lay that lane on that layer there. The run is kept as it stands and the place '
                        'goes back to the solve: that lane off that layer across that copper.')],
                says=[(0, 'A breach: no free space on this layer leads around the copper.'),
                      (0.38, 'No geometry can lay that lane there, so the place goes back to the solve.')],
                cues=[(0.05, 'alert', 0.6), (0.38, 'glide', 0.3, 0.62)])

    # ------------------------------------------------------------------ what stands at the end
    len0 = sum(plen(xy) for xy in e0['xy'].values())
    nsh = collections.Counter(x['kind'] for x in E['short'])
    vcuts, lcuts = S.emb.get('vcuts', []), S.emb.get('lcuts', [])
    shp = E['shape']
    short_txt = ', '.join(lab for k, lab in (('pitch', pl(nsh['pitch'], 'pair') + ' of lanes under their pitch'),
                                            ('via', pl(nsh['via'], 'via') + ' short of room'),
                                            ('viavia', pl(nsh['viavia'], 'pair') + ' of vias too near')) if nsh[k])
    cuts_txt = f'{pl(len(vcuts), "via cut")} and {pl(len(lcuts), "layer cut")}'
    ncross = sum(1 for r in ER if r['crossings'])            # (the rounds two same-layer lanes crossed after)
    folds = f'{E["faults"]} fold{"" if E["faults"] == 1 else "s"} or notch{"" if E["faults"] == 1 else "es"}'

    def settled(C, t, T_):
        draw_fanout(C, 0.5)
        m = ease(span(t, 0.5, 0.68))
        C.lanes(morph(film.st[film.elast], film.st['emb'], m), w=2.2)
        C.vias(film.vias['emb'] if m > 0.5 else film.vias[film.elast])
        a = ease(span(t, 0.08, 0.2))
        for x in E['tight']:
            C.wring(*BP(x['at']), 0.2, mix(BODY, TIGHT_C, a), 1.8)
        for x in E['short']:
            q = BP(x['at'])
            C.wring(*q, 0.32, mix(BODY, SHORT_C, a), 2.5)
            C.wtext(q[0] + 0.4, q[1], f'{x["mm"]:.3f} mm', 12, mix(BODY, SHORT_C, a), anchor='lm', bold=True)
        a_g = ease(span(t, 0.5, 0.6))
        for p in shp.get('gone', []):
            C.wring(*BP(p), 0.16, mix(BODY, EVC['pull'], a_g), 1.8)
        C.card((BRAID[0], BRAID[1], BRAID[0] + 1010, BRAID[3]), alpha=0.95)
        chart(C, NR, BRAID[0] + 180, BRAID[0] + 990)
        facts(C, (BRAID[0] + 1020, BRAID[1], BRAID[2], BRAID[3]), 'after the rounds', [
            (f'{NR} rounds' + (': nothing moves any more' if E['settled'] else ': the most it runs; a few places still move')
             + (f'; {slid_txt(len(E["slid"]))}' if E['slid'] else ''), TEXT),
            (f'the lanes\' length: {len0:.0f} mm at the start, {ER[-1]["length"]:.0f} mm now' if ER else '', TEXT),
            ((f'short of a rule (red): {short_txt}' if E['short'] else 'nothing stands short of a rule'),
             SHORT_C if E['short'] else EVC['pull']),
            (('no two lanes of a layer cross, after any round' if not ncross else
              f'two lanes of a layer cross after {pl(ncross, "round")}'), EVC['pull'] if not ncross else SHORT_C),
            (f'the comfortable margin given up at {pl(len(E["tight"]), "place")} (amber)', TIGHT_C),
            (f'the written lines: {folds}; {pl(shp.get("out", 0), "point")} left out (green), '
             f'{shp.get("left", 0)} stay', DIM),
            ((f'to the solve: {cuts_txt}' if vcuts or lcuts else 'no cut goes back to the solve'), TEXT)])
    sc.append(Scene('settled', 'geometry', 10, settled, [
        (0, (f'After {NR} rounds nothing moves any more. ' if E['settled'] else
             f'The rounds end after {NR}, the most the stage runs: a few places still move. ')
            + 'What stands short is measured on the lines themselves: '
            + (f'{short_txt} (red)' if E['short'] else 'nothing -- every lane has its pitch and every via its room')
            + f'. The comfortable margin beyond the rules was given up at {pl(len(E["tight"]), "place")} (amber).'),
        (0.5, f'The lines are then written for the stages after, cleared of the folds and notches no router lays '
              f'({folds} found, {pl(shp.get("out", 0), "point")} left out where nothing stands in the way). '
              + (f'A via still short of its room, and each breach, go back to the solve as cuts: {cuts_txt}.'
                 if vcuts or lcuts else 'Nothing goes back to the solve from here.'))],
        says=[(0, (f'After {NR:,} rounds, nothing moves.' if E['settled'] else
                   f'The rounds stop after {NR:,}, the most they run; a few places still move.')
               + (f' Short of a rule: {short_txt}, in red.' if E['short'] else
                  ' Every lane has its pitch, and every via its room.')),
              (0.5, 'The lines are cleaned of folds and notches' + (', and what is still short goes back to the solve.'
                                                                    if vcuts or lcuts else
                                                                    ', and nothing goes back to the solve.'))],
        # (what stands short, and the margin given up; the written lines)
        cues=([(0.08, 'alert' if E['short'] else 'shimmer', 0.4)] if E['short'] or E['tight'] else [])
        + [(0.5, 'glide', 0.5, 0.68)]))


# ------------------------------------------------------------------------------- the group's ends' scenes
GBALL = {'bus': ACCENT, 'other': (170, 178, 190), 'plane': (120, 220, 120)}
LESSON_C, FREED_C, MOVED_C = (255, 70, 70), (90, 230, 255), (255, 255, 255)


def fit_view(box, view, title=True):
    """a view holding all of `view` in a magnifier `box` (magnify_view keeps a view's width: here the width is widened
    where the box is wider than the view, so its height fits too)"""
    bx0, by0, bx1, by1 = box[0] + 6, box[1] + (26 if title else 6), box[2] - 6, box[3] - 6
    asp = (bx1 - bx0) / (by1 - by0)
    w, h = view[2] - view[0], view[3] - view[1]
    w = max(w, h * asp)
    cx = (view[0] + view[2]) / 2
    return (cx - w / 2, view[1], cx + w / 2, view[3])


def _mdraw(d, Z, s, segs=(), vias=(), alpha=1.0, col=None, w_px=None, via_d=0.25, via_col=None):
    """pieces of copper in a magnifier: segments (x0, y0, x1, y1, width, layer, ..) B under F, vias as rings"""
    for layer in _LAYERS:
        c = mix(BODY, col or layer_col(layer), alpha)
        for q in segs:
            if q[5] != layer:
                continue
            w = max(2, int(w_px * SS)) if w_px else max(2, int(q[4] * s))
            (a, b), (e, f_) = Z(q[0], q[1]), Z(q[2], q[3])
            d.line([(a, b), (e, f_)], fill=c, width=w)
            r = w / 2
            for (x, y) in ((a, b), (e, f_)):
                d.ellipse([x - r, y - r, x + r, y + r], fill=c)
    for (x, y) in vias:
        cx, cy = Z(x, y)
        r = max(3, via_d / 2 * s)
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=mix(BODY, via_col or (220, 220, 220), alpha))
        r2 = r * 0.5
        d.ellipse([cx - r2, cy - r2, cx + r2, cy + r2], fill=mix(BODY, (30, 30, 30), alpha))


def _ring(d, Z, s, x, y, r_mm, col, w=2):
    cx, cy = Z(x, y)
    r = r_mm * s
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=col, width=int(w * SS))


def group_scenes(film, sc):
    """THE GROUP'S ENDS (group_ends.py, negotiated_buses.route_grouped), appended to sc, before the bus's own run: the
    buses found and captured, the one model of the arrays' fanout and every bus's ends, its search, the checks by each
    bus's own whole solve and the lessons of the laid field, and the field itself. Drawn from what the step recorded --
    its events, the model's checks.jsonl, group.log, group.json and the boards it measured and laid -- never solved
    again here: the model's search is not re-run (its CP-SAT runs on many workers, which no re-run reproduces byte for
    byte), so nothing on these scenes is a re-run's, and nothing is drawn that the run did not write"""
    S, G, g = film.S, film.S.G, film.g
    TW = film.rules.get('TW', 0.1)
    VD = film.rules.get('via', 0.25)
    half = (BOARD[2] + BOARD[0]) / 2
    boxes = [(BOARD[0], BOARD[1], half - 5, BOARD[3]), (half + 5, BOARD[1], BOARD[2], BOARD[3])]
    st_ = lambda q: [(a, b, c, d_, TW, L, None) for a, b, c, d_, L in q]

    def arrays(C, draw_fn, sub=lambda ref: ''):
        """the two arrays, each in a magnifier of its own: draw_fn(d, Z, s, ref)"""
        C.rect(BOARD, fill=BG)
        for box, (ref, view) in zip(boxes, g.views):
            vv = fit_view(box, view)
            magnify(C, film, box, vv, lambda d, Z, s, ref=ref: draw_fn(d, Z, s, ref), f'{ref}  {sub(ref)}', frame=False)

    def balls(d, Z, s, a_bus, a_other, a_plane, r=0.36):
        for k, a in (('bus', a_bus), ('other', a_other), ('plane', a_plane)):
            if a > 0.02:
                for (x, y) in g.balls[k]:
                    _ring(d, Z, s, x, y, r, mix(BODY, GBALL[k], a), 2)

    def state_draw(d, Z, s, j, alpha=1.0, hl=None, hl_col=MOVED_C, dim=None):
        """state j's ends: each lane's escape as the model held it; `hl` ends ringed, `dim` the rest at that strength"""
        ends = g.states[j]['ends']
        for key, (segs, vias) in ends.items():
            a = alpha * (dim if dim is not None and (not hl or key not in hl) else 1.0)
            _mdraw(d, Z, s, st_(segs), vias, a, via_d=VD)
        if hl:
            for key in hl:
                if key in ends and ends[key][0]:
                    x, y = ends[key][0][-1][2:4]
                    _ring(d, Z, s, x, y, 0.3, mix(BODY, hl_col, alpha), 2)

    def card_lines(C, box, rows, px=14, a=1.0, title=None, mono=False):
        C.card(box, alpha=0.95)
        y = box[1] + 12
        if title:
            C.text(box[0] + 14, y, title, 16, mix(PANEL, ACCENT, a), bold=True)
            y += 26
        for txt, col in rows:
            for w_ in C.wrap(txt, px, box[2] - box[0] - 28):
                if y > box[3] - px - 6:
                    return
                C.text(box[0] + 14, y, w_, px, mix(PANEL, col, a), mono=mono)
                y += px + 5
            y += 3

    trim = lambda ln, n=400: ln if len(ln) <= n else ln[:n - 3] + '...'
    clk = lambda t_s: f'{t_s:.0f} s'

    # ------------------------------------------------------------------ the buses, the group, the capture
    caps_ = ', '.join(f'{e["bus"].split(":")[-1]} in {e.get("secs", "?")} s' for e in G.cap_end) or 'none recorded'
    srcs = sorted({b['src'] for b in G.plan})
    nb = {k: len(v) for k, v in g.balls.items()}

    def buses(C, t, T_):
        a1, a2, a3 = ease(span(t, 0.12, 0.3)), ease(span(t, 0.4, 0.55)), ease(span(t, 0.55, 0.7))
        arrays(C, lambda d, Z, s, ref: balls(d, Z, s, a1, a2, a3))
        rows = [(f'the bus step found {pl(len(G.plan), "bus", "buses")}:', TEXT)]
        rows += [(f'   {b["src"]} -> {b["dest"]}, {b["routed"]} nets'
                  + (f' ({b["nets"]} between the two parts alone)' if b['routed'] != b['nets'] else '')
                  + f' -- {b["why"]}', DIM) for b in G.plan]
        rows += [(f'one group for each source array: {", ".join(srcs)}', TEXT),
                 (f'the capture, each bus\'s ends model by a short run of its own: {caps_}', TEXT if t > 0.3 else DIM)]
        card_lines(C, (BRAID[0], BRAID[1], BRAID[0] + 930, BRAID[3]), rows, 16)
        rows2 = [(f'gold: {nb["bus"]} bus balls  ·  grey: {nb["other"]} other signals\' balls  ·  green: '
                  f'{nb["plane"]} plane balls', TEXT)] + [(trim(ln.split('; the pattern')[0], 120), DIM) for ln in G.arrays]
        card_lines(C, (BRAID[0] + 950, BRAID[1], BRAID[2], BRAID[3]), rows2, 16, a=ease(span(t, 0.35, 0.5)))
    sc.append(Scene('buses', 'capture', 10, buses, [
        (0, f'The bus step finds its buses, and the buses that leave one source array are ONE GROUP: '
            f'{", ".join(b["src"] + " -> " + b["dest"] for b in G.plan)}. Their ends are chosen together.'),
        (0.3, 'CAPTURE: each bus\'s ends model is captured by a short run of its own -- its frame, its cut, each array '
              'left bare -- stopped as soon as it is written.'),
        (0.55, 'The arrays\' balls: the bus\'s (gold), the other signals\' (grey), the plane balls (green). One model '
               'will serve every one of them.')]))

    # ------------------------------------------------------------------ the one model, and a state at all
    m0 = G.models[0] if G.models else None
    o0 = m0['model']['objective'] if m0 else {}
    rest0 = sorted((o0.get('rest_vias') or {}).items(), key=lambda kv: -kv[1])
    wl = (m0['model']['W_LEX'] / 1000) if m0 else 0
    s0 = next(iter(m0['states'].values()), {}) if m0 else {}

    def model(C, t, T_):
        a_s = ease(span(t, 0.62, 0.8))
        arrays(C, lambda d, Z, s, ref: (balls(d, Z, s, 0.55 * (1 - a_s) + 0.15, 0.55, 0.55),
                                        state_draw(d, Z, s, 0, a_s) if a_s > 0.02 else None))
        a_l = ease(span(t, 0.05, 0.2))
        rows = [(trim(ln, 220), DIM) for ln in G.tiers]
        rows += [(f'round {r}: {trim(q, 220)}', (255, 140, 120) if q.startswith('no state') else (150, 245, 120))
                 for j, (r, q) in enumerate(G.state_q) if t > 0.3 + 0.1 * j]
        card_lines(C, (BRAID[0], BRAID[1], BRAID[0] + 1000, BRAID[3]), rows, 14, a=a_l,
                   title='the menus, the pattern first -- and a state at all?')
        if t > 0.6 and m0:
            a = ease(span(t, 0.6, 0.7))
            rows = [(trim(ln, 200), DIM) for ln in G.start_lines]
            rows += [(f'the start state: {s0.get("crossings", "?")} crossings, {s0.get("changes", "?")} lanes changing '
                      f'layer, {s0.get("vias", "?")} fanout vias', TEXT),
                     (f'its objective: {o0.get("walls", 0)} walled, {o0.get("fans", 0)} plane balls fanned out, '
                      f'{o0.get("far_teeth", 0)} end(s) on a far face, {o0.get("changes", 0)} changes of layer at '
                      f'{wl:g} vias each, and the rest in vias:', TEXT),
                     ('   ' + ',  '.join(f'{k} {v:.0f}' for k, v in rest0 if v), DIM)]
            card_lines(C, (BRAID[0] + 1020, BRAID[1], BRAID[2], BRAID[3]), rows, 14, a=a, title='the start')
    sc.append(Scene('model', 'ends model', 16, model, [
        (0, 'THE GROUP\'S ENDS: one CP-SAT model chooses the arrays\' whole fanout -- every bus ball\'s tooth or berth, '
            'every other signal\'s escape, every plane ball\'s drop -- together with every bus\'s ends.'),
        (0.22, 'Each ball\'s moves come in tiers, the pattern a hand lays first; a ball is offered its whole menu only '
               'where a refusal names it. Every rule is hard: where no state holds them all, the rules that cannot hold '
               'together are named.'),
        (0.6, 'A state at last, and from it the start: each bus\'s ends in turn, the costly ends first. Drawn: the '
              'start\'s every lane end, its escape and its via, as the model held it.')]))

    # ------------------------------------------------------------------ the search
    nv = next((j for j, r_ in enumerate(G.recs) if r_['rec'] == 'verdict'), None)
    t_chk = G.recs[nv]['t'] if nv is not None else (G.done or {}).get('secs', 1.0)
    t_all = max([(G.done or {}).get('secs', 0.0)] + [r_['t'] for r_ in G.rounds] + [1.0])
    m_t = [M_['model']['t'] for M_ in G.models]
    line_v = []
    for M_ in G.models:
        v = M_['uncounted'] or M_['start_value']
        if v is None:              # (a repair: no search's line -- its own solve's best, as the run reported it)
            v = next((r_['best'] for r_ in reversed(G.rounds) if r_['t'] <= M_['model']['t'] + 0.5
                      and r_['label'] == 'the model'), None)
        line_v.append(v)
    # (the chart's range: the states' own values and the rounds' near them -- a round's first answers, found before
    # its solve has a state of the costly ends, stand thousands of times higher; those are counted off the top)
    lv_ = [v for v in line_v if v] or [1.0]
    vals = [r_['best'] for r_ in G.rounds if r_['best'] and r_['best'] > 0] + lv_
    vlo, vhi = max(1.0, min(vals) * 0.9), max(lv_) * 1.8
    n_top = sum(1 for r_ in G.rounds if r_['best'] and r_['best'] > vhi)
    taken = re.findall(r'(\d+) of (\d+) states asked taken', G.log)
    n_rounds = re.findall(r'the search: (\d+) rounds of (\d+) lanes', G.log)
    rcol = lambda lab: ((255, 170, 60) if 'costly' in lab else (90, 230, 255) if 'rest held' in lab else
                        (255, 90, 210) if lab == 'the model' else (215, 220, 228))

    def chart(C, box, t_now, a=1.0, marks=True):
        C.card(box, alpha=0.95)
        gx0, gy0, gx1, gy1 = box[0] + 70, box[1] + 34, box[2] - 20, box[3] - 44
        C.text(box[0] + 14, box[1] + 8, 'the model\'s search, on its own clock: each dot one round\'s best as the search '
               'reported it; each gold square the state a solve ended on (vias\' worth, log scale)', 13, mix(PANEL, TEXT, a))
        X = lambda ts: gx0 + ts / t_all * (gx1 - gx0)
        Y = lambda v: gy1 - (math.log(max(v, vlo)) - math.log(vlo)) / (math.log(vhi) - math.log(vlo)) * (gy1 - gy0)
        C.line([(gx0, gy0), (gx0, gy1), (gx1, gy1)], mix(PANEL, DIM, a), 1)
        for v in (100, 200, 300, 400, 500, 700, 1000, 2000, 3000, 5000):
            if vlo <= v <= vhi:
                C.line([(gx0 - 4, Y(v)), (gx1, Y(v))], mix(PANEL, (44, 48, 58), a), 1)
                C.text(gx0 - 8, Y(v), str(v), 11, mix(PANEL, DIM, a), anchor='rm')
        step_ = 60 if t_all > 300 else 30
        # (a long search: a label every few minutes, not a crowd of them)
        step_ = next((c_ for c_ in (30, 60, 120, 300, 600, 1200, 3000) if c_ >= step_ and t_all / c_ <= 16), 6000)
        for ts in range(0, int(t_all) + 1, step_):
            C.text(X(ts), gy1 + 6, f'{ts} s', 11, mix(PANEL, DIM, a), anchor='ma')
        if marks:
            for r_ in G.recs:
                if r_['rec'] in ('verdict', 'field') and r_['t'] <= t_now:
                    x = X(r_['t'])
                    ok = (r_['verdict'] or {}).get('plan') if r_['rec'] == 'verdict' else not r_.get('lessons')
                    col = (150, 245, 120) if ok else LESSON_C
                    C.line([(x, gy0), (x, gy1)], mix(PANEL, col, a * 0.7), 1)
                    lab = (f'check {r_["check"]}: {"a plan" if ok else "refused"}' if r_['rec'] == 'verdict' else
                           f'field: DRC {r_.get("drc_field")}, {pl(r_.get("lessons", 0), "lesson")}')
                    C.text(x - 3, gy0 + (0 if r_['rec'] == 'verdict' else 16), lab, 11, mix(PANEL, col, a), anchor='ra')
        top_ = 0
        for r_ in G.rounds:
            if r_['t'] <= t_now and r_['best'] and r_['best'] > 0:
                if r_['best'] > vhi:
                    top_ += 1
                    continue
                x, y = X(r_['t']), Y(r_['best'])
                C.d.ellipse([(x - 2.4) * SS, (y - 2.4) * SS, (x + 2.4) * SS, (y + 2.4) * SS], fill=mix(PANEL, rcol(r_['label']), a))
        if top_:
            C.text(gx1, gy0 - 2, f'{top_} round(s)\' first answers above {vhi:.0f}, off the top', 11, mix(PANEL, DIM, a), anchor='rb')
        # the state each solve ended on (the record keeps no state between: no line is drawn between them)
        pts = [(X(tt), Y(v)) for tt, v in zip(m_t, line_v) if v and tt <= t_now]
        for (x, y) in pts:
            C.d.rectangle([(x - 4) * SS, (y - 4) * SS, (x + 4) * SS, (y + 4) * SS], fill=mix(PANEL, ACCENT, a))
        x_ = X(min(t_now, t_all))
        C.line([(x_, gy0), (x_, gy1)], mix(PANEL, (120, 126, 140), a), 1)
        C.text(gx1, box[3] - 18, 'cyan: each bus\'s ends in turn   orange: a round about a costly ball   white: a round of '
               'the search   magenta: a repair\'s solve   gold: a solve\'s state', 11, mix(PANEL, DIM, a), anchor='ra')

    def which(t_now):
        """the last of the model's states standing at t_now on its clock"""
        return max([j for j, tt in enumerate(m_t) if tt <= t_now] or [0])

    def search(C, t, T_):
        t_now = ease(span(t, 0.04, 0.9)) * t_chk
        j = which(t_now)
        M_ = G.models[j]
        hl = g.states[j]['moved'] if j else None
        arrays(C, lambda d, Z, s, ref: state_draw(d, Z, s, j, 1.0, hl=hl))
        chart(C, (BRAID[0], BRAID[1], BRAID[2] - 470, BRAID[3]), t_now)
        sv = next(iter(M_['states'].values()), {})
        rows = [(M_['label'], ACCENT),
                (f'{sv.get("crossings", "?")} crossings, {sv.get("changes", "?")} lanes changing layer, '
                 f'{sv.get("vias", "?")} fanout vias', TEXT),
                (f'the model\'s own value {M_["value"]:.1f} vias\' worth' + (f' + {M_["costly"]} costly end(s)' if M_['costly'] else ''), TEXT),
                (f'{sv.get("n_moved", 0)} lane(s) moved an end since the state before (ringed)' if j else
                 'the start: each lane on its first tiers where a state allows', DIM)]
        if M_['search']:
            rows.append((M_['search'], DIM))
        card_lines(C, (BRAID[2] - 460, BRAID[1], BRAID[2], BRAID[3]), rows, 14,
                   title=f'state {j + 1} of {len(G.models)}, at {clk(m_t[j])}')
    nr_ = sum(int(a_) for a_, _b in n_rounds)
    tk_ = (sum(int(a_) for a_, _b in taken), sum(int(b_) for _a, b_ in taken))
    sc.append(Scene('search', 'search', 22, search, [
        (0, 'THE SEARCH: the lanes are cut into stretches; each round frees a few lanes of a stretch, '
            + (f'{n_rounds[0][1]}, ' if n_rounds else '') + 'and the balls near them, the rest held, and solves again. '
            'Every solve is bounded by deterministic batches, never a clock.'),
        (0.35, f'Each dot is one round\'s best ({nr_} rounds in all' + (f'; {tk_[0]} of the {tk_[1]} states asked were '
               'taken' if taken else '') + '). The objective comes down from the start to the state the search ends on.'),
        (0.7, 'On the board, the ends of the state the search holds; the ends that moved since the state before are '
              'ringed.')]))

    # ------------------------------------------------------------------ the checks, the lessons, the repair
    v1 = G.verdicts[0] if G.verdicts else None
    f1 = G.fields[0] if G.fields else None
    j_rep = next((j for j, M_ in enumerate(G.models) if M_['model'].get('how') == 'repair'), None)
    j_pre = (j_rep - 1) if j_rep else len(G.models) - 1
    free_ = G.models[j_rep]['model'].get('free') if j_rep is not None else {}
    freed = {(bn, ln) for bn, d_ in (free_ or {}).items() for ln in d_}
    why_n = collections.Counter(('named' if w_ == 'named' else 'crossed one named' if w_.startswith('crossed')
                                 else 'let go, could not stand with the rest' if w_.startswith('let go') else w_)
                                for d_ in (free_ or {}).values() for w_ in d_.values())
    items = (f1 or {}).get('items') or []
    lessons_at = [(tb_, it) for it in items for tb_ in [G.tb(it['closest_pt'])] if it.get('closest_pt')]
    stuck_refs = sorted({it['pad_ref'].split('.')[0] for it in items if it.get('pad_ref')})
    lesson_nets = {(it.get('net1') or '').split('/')[-1] for it in items}

    def lesson_draw(d, Z, s, a):
        for (x, y), it in lessons_at:
            cx, cy = Z(x, y)
            r = 0.22 * s
            d.line([(cx - r, cy - r), (cx + r, cy + r)], fill=mix(BODY, LESSON_C, a), width=3 * SS)
            d.line([(cx - r, cy + r), (cx + r, cy - r)], fill=mix(BODY, LESSON_C, a), width=3 * SS)
        # (the part where the lesson found it is not drawn: the field it was laid on is not kept, only the finding's
        # point -- the pads drawn are where the last field left them)

    def check(C, t, T_):
        ph = 0 if t < 0.25 else 1 if t < 0.5 else 2 if t < 0.75 else 3
        if ph <= 1 or j_rep is None:
            j = j_pre
            hl = {k for k in g.states[j]['ends'] if k[1] in lesson_nets} if ph == 1 else None
            arrays(C, lambda d, Z, s, ref: (state_draw(d, Z, s, j, 1.0, hl=hl, hl_col=LESSON_C, dim=0.45 if hl else None),
                                            lesson_draw(d, Z, s, ease(span(t, 0.27, 0.33))) if ph == 1 else None))
        elif ph == 2:
            hl = {k for k in g.states[j_rep]['ends'] if (k[0], k[1]) in freed}
            arrays(C, lambda d, Z, s, ref: state_draw(d, Z, s, j_rep, 1.0, hl=hl, hl_col=FREED_C, dim=0.4))
        else:
            j = len(G.models) - 1
            arrays(C, lambda d, Z, s, ref: state_draw(d, Z, s, j, 1.0, hl=g.states[j]['moved'] or None))
        wcol = lambda v: (150, 245, 120) if (v or {}).get('verdict', {}) and v['verdict'].get('plan') else LESSON_C
        third = (BRAID[2] - BRAID[0]) / 3
        b1 = (BRAID[0], BRAID[1], BRAID[0] + third - 8, BRAID[3])
        b2 = (BRAID[0] + third, BRAID[1], BRAID[0] + 2 * third - 8, BRAID[3])
        b3 = (BRAID[0] + 2 * third, BRAID[1], BRAID[2], BRAID[3])
        if v1:
            vd = v1.get('verdict') or {}
            rows = [(f'{v1["bus"]}\'s own whole solve, on the board as the model fanned it out: '
                     + ('A PLAN' if vd.get('plan') else 'NO PLAN') + f' [{vd.get("status", "?")}] in {v1["secs"]:g} s', wcol(v1))]
            if vd.get('lanes'):
                rows.append((f'lanes it named: {", ".join(vd["lanes"][:12])}', DIM))
            rows += [(ln, DIM) for ln in v1['console'][:6]]
            card_lines(C, b1, rows, 13, title='check 1: the whole solve')
        if f1 and t > 0.25:
            a = ease(span(t, 0.25, 0.32))
            rows = [(f'the field laid and measured: DRC {f1.get("drc_field")} (the bare board {f1.get("drc_bare")}); '
                     f'{pl(f1.get("lessons", 0), "lesson")}', LESSON_C if f1.get('lessons') else (150, 245, 120))]
            rows += [(ln, DIM) for ln in G.lesson_lines[:4]]
            if f1.get('moved'):
                rows.append((f'parts the nudge moved: {", ".join(f1["moved"])}', DIM))
            card_lines(C, b2, rows, 13, a=a, title='the field, measured')
        if j_rep is not None and t > 0.5:
            a = ease(span(t, 0.5, 0.57))
            sv = next(iter(G.models[j_rep]['states'].values()), {})
            rows = [(f'the repair: every lane held on its ends but {len(freed)}: '
                     + ', '.join(f'{n_} {k_}' for k_, n_ in why_n.most_common()), FREED_C),
                    (f'its state: {sv.get("crossings", "?")} crossings, {sv.get("changes", "?")} lanes changing layer, '
                     f'{G.models[j_rep]["value"]:.1f} vias\' worth', TEXT)]
            if G.dearer:
                rows.append((G.dearer, DIM))
            if t > 0.75:
                jl_ = len(G.models) - 1
                svl = next(iter(G.models[jl_]['states'].values()), {})
                rows.append((f'searched again: {svl.get("crossings", "?")} crossings, {svl.get("changes", "?")} lanes '
                             f'changing layer, {G.models[jl_]["value"]:.1f} vias\' worth', TEXT))
                for v_ in G.verdicts[1:]:
                    rows.append((f'check {v_["check"]}: ' + ('a plan' if (v_.get('verdict') or {}).get('plan') else 'no plan')
                                 + f' in {v_["secs"]:g} s', wcol(v_)))
                    rows += [(ln.strip(), DIM) for ln in v_['console'] if ln.strip().startswith(('check:', 'warm start'))]
                for f_ in G.fields[1:]:
                    rows.append((f'the field: DRC {f_.get("drc_field")}, {pl(f_.get("lessons", 0), "lesson")}',
                                 (150, 245, 120) if not f_.get('lessons') else LESSON_C))
            card_lines(C, b3, rows, 13, a=a, title='the repair, and the check again')
    if v1:
        sc.append(Scene('check', 'check', 22, check, [
            (0, 'CHECKED: each bus\'s ends go to the bus\'s own whole solve, under its strict rules, on the board as the '
                'model fanned it out. A refusal teaches the model, and it is solved again.'),
            (0.25, 'The field is laid and measured too. ' + (
                f'Here its DRC is {f1.get("drc_field")}, but {pl(len(items), "finding")}: '
                + (f'{", ".join(stuck_refs)} left standing in lanes\' ways out (red X: where the finding was) -- a '
                   'lesson for the model.'
                   if stuck_refs else 'each a lesson for the model.') if f1 and items else
                'Nothing in it is the model\'s to answer.')),
            (0.5, 'THE REPAIR: every lane held on the ends it had but those the lesson named and those that cross one '
                  '(cyan); a repair dearer than the state before has those lanes searched again.'),
            (0.75, 'Checked again by the whole solve, and the field laid again and measured: '
                   + ('clean.' if G.fields and not G.fields[-1].get('lessons') and not G.fields[-1].get('drc_field')
                      else 'not clean.'))]))

    # ------------------------------------------------------------------ the field
    tie_c = (255, 90, 210)
    fan_ = re.findall(r'the arrays fanned out as chosen: (\d+) tracks and (\d+) vias', G.log)
    fan_ = fan_[-1] if fan_ else None

    def field(C, t, T_):
        a = ease(span(t, 0.05, 0.4))
        a_m = ease(span(t, 0.45, 0.6))
        a_t = ease(span(t, 0.65, 0.75))

        def draw(d, Z, s, ref):
            _mdraw(d, Z, s, g.field_segs, [v[:2] for v in g.field_vias], a, via_d=VD)
            for (x, y) in g.balls['plane']:
                _ring(d, Z, s, x, y, 0.36, mix(BODY, GBALL['plane'], a * 0.8), 2)
            if a_m > 0.02:
                for ref_, old, new in g.moved_parts:
                    for ((x0, y0), sx, sy), ((x1, y1), _sx, _sy) in zip(old, new):
                        (p0, q0), (p1, q1) = Z(x0 - sx / 2, y0 - sy / 2), Z(x0 + sx / 2, y0 + sy / 2)
                        d.rectangle([p0, q0, p1, q1], outline=mix(BODY, (150, 150, 150), a_m), width=SS)
                        d.line([Z(x0, y0), Z(x1, y1)], fill=mix(BODY, MOVED_C, a_m), width=2 * SS)
                    (x, y), _sx, sy = new[0]
                    cx, cy = Z(x, y)
                    d.text((cx, cy - sy * s - 4 * SS), ref_, font=font(13 * SS, True), fill=mix(BODY, MOVED_C, a_m), anchor='mb')
            if a_t > 0.02 and g.ties:
                _mdraw(d, Z, s, g.ties, (), a_t, col=tie_c, w_px=4)
                for q in g.ties:
                    _ring(d, Z, s, q[0], q[1], 0.4, mix(BODY, tie_c, a_t), 2)
        arrays(C, draw)
        rows = [(ln.split(' -> ')[0], TEXT if ln.startswith(('the arrays', 'the field as laid')) else DIM)
                for ln in G.field_lines]
        if G.secs_line:
            rows.append((trim(G.secs_line.split(' -> ')[0], 120), DIM))
        card_lines(C, (BRAID[0], BRAID[1], BRAID[2], BRAID[3]), rows, 16, title='the shared field')
    if g.field_segs:
        sc.append(Scene('field', 'field', 14, field, [
            (0, 'THE FIELD: the arrays are fanned out exactly as the model chose' + (
                f' -- {fan_[0]} tracks and {fan_[1]} vias' if fan_ else '') + ': the bus lanes\' teeth and berths, the '
                'other signals\' escapes, the plane balls\' drops (green).'),
            (0.45, f'The cap nudge runs with each lane\'s way out held clear: '
                   f'{pl(len(g.moved_parts), "part")} stand elsewhere on the field than on the board the menus were '
                   f'measured on ({", ".join(r_ for r_, _o, _n in g.moved_parts)}; grey: where each stood).'),
            (0.65, 'The field\'s ties: a pad on a poured net beneath the group\'s parts is tied before any lane is '
                   'planned' + (' (magenta).' if g.ties else '.') + ' The field\'s DRC is measured again; every bus is '
                   'routed on this board.')]))
    group_sound(film, sc, locals())


def group_sound(film, sc, v):
    """the group's ends' scenes' narration and effects (--sound: whole_movie_sound.py), each line at the moment its
    scene shows what it speaks of, every number read from the run's record as the captions read it. `v` is
    group_scenes' own names (its counts, its states' times)"""
    S, G, g = film.S, film.S.G, film.g
    by = {s_.key: s_ for s_ in sc}
    num = lambda n: f'{n:,}'
    plw = lambda n, one, many=None: f'{num(n)} {one if n == 1 else (many or one + "s")}'
    # ---- the buses, the group, the capture
    if 'buses' in by:
        nb, srcs = v['nb'], v['srcs']
        bs = ', '.join(f'{spoken(b["src"])} to {spoken(b["dest"])}, {num(b["routed"])} nets' for b in G.plan)
        secs_ = [e.get('secs') for e in G.cap_end if e.get('secs') is not None]
        by['buses'].says = [
            (0, f'The bus step finds its buses: {plw(len(G.plan), "bus", "buses")}, {bs}. The buses that leave one '
                'source array are one group, and their ends are chosen together.', 0.3),
            (0.3, 'First, each bus\'s ends model is captured by a short run of its own'
                  + (f', here in {plw(max(secs_), "second")}.' if secs_ else '.'), 0.55),
            (0.55, f'On the arrays: {num(nb["bus"])} bus balls in gold, {num(nb["other"])} other signals\' balls in grey, '
                   f'and {num(nb["plane"])} plane balls in green. One model will serve every one of them.')]
        by['buses'].cues = [(0.12, 'shimmer', 0.7), (0.3, 'swell', 0.3), (0.4, 'shimmer', 0.45), (0.55, 'shimmer', 0.9)]
    # ---- the one model, a state at all, the start
    if 'model' in by and G.models:
        sq = G.state_q
        k_ok = next((j for j, (_r, q) in enumerate(sq) if not q.startswith('no state')), None)
        nno = len(sq) if k_ok is None else k_ok
        ss = [(0, 'Then one CP-SAT model chooses the arrays\' whole fanout: every bus ball\'s tooth or berth, every other '
                  'signal\'s escape, every plane ball\'s drop, together with every bus\'s ends.', 0.22),
              (0.22, 'Each ball\'s moves come in tiers, the pattern a designer would lay coming first. Every rule is '
                     'hard.', 0.3)]
        if nno:
            ss.append((0.3, ('In the first round, ' if nno == 1 else f'In the first {num(nno)} rounds, ')
                       + 'no state holds every rule. The rules that cannot hold together are named, and the balls of '
                         'their lanes are offered their whole menus.', 0.3 + 0.1 * nno if k_ok is not None else 0.6))
        if k_ok is not None:
            ss.append((0.3 + 0.1 * k_ok, f'In round {sq[k_ok][0]}, a state that holds every rule.', 0.6))
        s0 = next(iter(G.models[0]['states'].values()), {})
        ss.append((0.6, 'From it, the start: each bus\'s ends in turn, the costly ends first. Drawn, every lane\'s ends '
                        f'as the model holds them: {plw(s0.get("crossings", 0), "crossing")}, and '
                        f'{plw(s0.get("changes", 0), "lane")} changing layer.'))
        by['model'].says = ss
        by['model'].cues = [(0.05, 'swell', 0.3)] + [(0.3 + 0.1 * j, 'alert' if q.startswith('no state') else 'chord',
                                                      0.5 if q.startswith('no state') else 1.0)
                                                     for j, (_r, q) in enumerate(sq) if 0.3 + 0.1 * j < 0.6] \
            + [(0.62, 'glide', 0.5, 0.8)]
    # ---- the search
    if 'search' in by:
        t_chk, m_t, line_v, j_pre = v['t_chk'], v['m_t'], v['line_v'], v['j_pre']
        n_rounds, taken = v['n_rounds'], v['taken']
        # (the scene's moment of a time on the model's clock: the inverse of its cursor, ease(span(t, .04, .9)) t_chk)
        grid_ = np.linspace(0, 1, 2001)
        clock_ = np.array([ease(span(t_, 0.04, 0.9)) * t_chk for t_ in grid_])
        at_ = lambda tm: float(grid_[min(len(grid_) - 1, int(np.searchsorted(clock_, tm)))])
        nr_ = sum(int(a_) for a_, _b in n_rounds)
        tk_ = (sum(int(a_) for a_, _b in taken), sum(int(b_) for _a, b_ in taken))
        jl_ = max([j for j, tt in enumerate(m_t) if tt <= t_chk] or [0])
        # (the two values spoken are the search's own, its start's and its last stretch's before the check: the
        # chart's squares add what the model does not count, which the start's value has not)
        v0 = G.models[0]['start_value'] if G.models else None
        v1 = G.models[jl_]['end_value'] if G.models else None
        by['search'].says = [
            (0, 'The search. The lanes are cut into stretches, and each round frees a few lanes'
                + (f', {n_rounds[0][1]} of them,' if n_rounds else '') + ' and the balls near them, holds the rest, and '
                'solves again.', 0.35),
            (0.35, f'Each dot is one round\'s best: {num(nr_)} rounds in all'
                   + (f', and {num(tk_[0])} of the {num(tk_[1])} states they asked for were taken.' if taken else '.'), 0.7),
            (0.7, ('The state the search holds comes down from ' + f'{v0:,.0f} to {v1:,.0f} vias\' worth. '
                   if v0 and v1 else '') + 'On the board, the ends that moved since the state before are ringed.')]
        by['search'].cues = [(0.02, 'swell', 0.3)] \
            + [(at_(r_['t']), 'tick', 0.6) for r_ in G.rounds if r_['t'] <= t_chk] \
            + [(at_(tt), 'bell', min(1.0, j / max(1, jl_))) for j, tt in enumerate(m_t) if tt <= t_chk]
    # ---- the check, the field's lessons, the repair, the check again
    if 'check' in by and G.verdicts:
        v1_, f1 = G.verdicts[0], (G.fields[0] if G.fields else None)
        vd = v1_.get('verdict') or {}
        ss = [(0, f'Then the check: {spoken(v1_["bus"])}\'s own whole solve, under its strict rules, on the board as the '
                  'model fanned it out. ' + (f'It finds a plan, in {plw(round(v1_["secs"]), "second")}.' if vd.get('plan')
                                             else 'It finds none, and its refusal teaches the model.'), 0.25)]
        items, stuck = v['items'], v['stuck_refs']
        if f1 is not None:
            ss.append((0.25, f'The field is laid and measured: {plw(f1.get("drc_field", 0), "design rule violation")}'
                       + (f', but {plw(len(items), "finding")}: ' + ', '.join(spoken(r_) for r_ in stuck)
                          + ' is left standing in lanes\' ways out, at the red marks. Each finding is a lesson for the '
                            'model.' if items and stuck else
                          f', and {plw(len(items), "finding")}, each a lesson for the model.' if items else
                          ', and nothing for the model to answer.'), 0.5))
        if v['j_rep'] is not None:
            wn = v['why_n']
            parts_ = [f'{num(wn[k_])} {w_}' for k_, w_ in (('named', 'named'), ('crossed one named', 'crossing one'),
                                                          ('let go, could not stand with the rest', 'let go'))
                      if wn.get(k_)]
            ss.append((0.5, f'The repair holds every lane on the ends it had, but {num(len(v["freed"]))}: '
                       + ', '.join(parts_) + '.' + (' Its state is dearer than the one before, so those lanes are '
                                                    'searched again.' if G.dearer else ''), 0.75))
        later = [('a plan' if (x_.get('verdict') or {}).get('plan') else 'no plan') for x_ in G.verdicts[1:]]
        if later or G.fields[1:]:
            ff_ = G.fields[-1] if len(G.fields) > 1 else None
            ss.append((0.75, 'Checked again, the whole solve finds ' + (later[-1] if later else 'its answer')
                       + ('; and the field, laid again, ' + ('has no finding and no violation.' if not ff_.get('lessons')
                                                              and not ff_.get('drc_field') else 'is not clean.')
                          if ff_ else '.')))
        by['check'].says = ss
        by['check'].cues = [(0.03, 'swell', 0.3), (0.06, 'chord' if vd.get('plan') else 'alert', 1.0)] \
            + ([(0.28, 'alert', 0.5)] if items else []) + [(0.5, 'glide', 0.5, 0.72), (0.76, 'chord', 1.0)]
    # ---- the field
    if 'field' in by:
        fan_ = v['fan_']
        tie_ = re.findall(r'field ties: (\d+) of the (\d+) pad', G.log)
        tie_drc = re.findall(r'field ties: .*DRC (\d+)', G.log)
        by['field'].says = [
            (0, 'The field: the arrays fanned out exactly as the model chose'
                + (f', {plw(int(fan_[0]), "track")} and {plw(int(fan_[1]), "via")}' if fan_ else '')
                + ': the bus lanes\' teeth and berths, the other signals\' escapes, and the plane balls\' drops.', 0.45),
            (0.45, 'The cap nudge runs with every lane\'s way out held clear: '
                   f'{plw(len(g.moved_parts), "part")} stand elsewhere than on the board the menus were measured on.',
             0.65),
            (0.65, ('Then the field\'s ties: ' + (f'{num(int(tie_[-1][0]))} of the {plw(int(tie_[-1][1]), "pad")} '
                                                  'on a poured net beneath the parts tied before any lane is planned. '
                                                  if tie_ else 'the pads on a poured net beneath the parts. '))
             + (f'Measured again: {plw(int(tie_drc[-1]), "design rule violation")}. ' if tie_drc else '')
             + 'Every bus is routed on this board.')]
        by['field'].cues = [(0.05, 'glide', 0.4, 0.4), (0.45, 'slide', 0.5, 0.6), (0.66, 'mark', 0.5), (0.85, 'chord', 1.0)]


# ------------------------------------------------------------------------------- the negotiation's scenes
BUS_TINT = ((255, 210, 90), (110, 200, 200), (205, 130, 205), (160, 205, 110), (200, 170, 130), (150, 160, 230))
PHASE_C = {'fanout': (140, 148, 160), 'solve': (90, 230, 255), 'wait': (64, 70, 82), 'loop': (150, 245, 120),
           'lay': (255, 165, 70)}


def neg_prepare(film):
    """the negotiation made ready for drawing: each bus's tint (the filmed bus's the film's gold), the view that holds
    every bus's copper and arrays, and the other buses' copper, drawn faint where the focus is the filmed bus"""
    S, N = film.S, film.S.N
    N.tint = {b: BUS_TINT[0] if b == N.me else BUS_TINT[1 + j % (len(BUS_TINT) - 1)]
              for j, b in enumerate([b for b in N.buses if b != N.me])}
    N.tint[N.me] = BUS_TINT[0]
    pts = [(q[0], q[1]) for segs, _v in N.copper.values() for q in segs] + \
          [(q[2], q[3]) for segs, _v in N.copper.values() for q in segs]
    for ref in [S.src] + N.buses:
        fp = S.fo.footprints.get(ref)
        if fp is not None:
            pts += [(p.global_x, p.global_y) for p in fp.pads]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    N.view = fit_view(BOARD, (min(xs) - 1.5, min(ys) - 1.5, max(xs) + 1.5, max(ys) + 1.5))
    N.peer_segs = [q for b in N.buses if b != N.me for q in N.copper.get(b, ([], []))[0]]
    N.peer_vias = [q for b in N.buses if b != N.me for q in N.copper.get(b, ([], []))[1]]
    N.field = {b: [q for q in (film.g.field_segs if film.g else []) if q[6] in N.nets.get(b, ())] for b in N.buses}


def neg_view(C, film, draw, title, show=None, field=1.0):
    """the whole group, every bus where it runs, in the board's panel: its pads, each bus's field ends (`field`), each
    bus's copper as `show` {bus: alpha} has it -- its layers' colours over its tint -- then draw(d, Z, s); each bus named
    by its destination"""
    N = film.S.N

    def fn(d, Z, s):
        for b in N.buses:
            if field > 0.02:
                _mdraw(d, Z, s, N.field[b], (), field * (1.0 if b == N.me else 0.6))
        for b, a in (show or {}).items():
            segs, vias = N.copper.get(b, ([], []))
            if a <= 0.02:
                continue
            tw = mix(BODY, N.tint[b], 0.38 * a)
            for q in segs:
                (x0, y0), (x1, y1) = Z(q[0], q[1]), Z(q[2], q[3])
                d.line([(x0, y0), (x1, y1)], fill=tw, width=max(2, int((q[4] * s) + 5 * SS)))
            _mdraw(d, Z, s, segs, [v_[:2] for v_ in vias], a, via_d=film.rules['via'])
        for b in N.buses:
            fp = film.S.fo.footprints.get(b)
            if fp is not None:
                xs = [p.global_x for p in fp.pads]
                ys = [p.global_y for p in fp.pads]
                x, y = Z((min(xs) + max(xs)) / 2, min(ys))
                d.text((x, y - 6 * SS), b, font=font(15 * SS, True), fill=N.tint[b], anchor='mb')
        draw(d, Z, s)
    magnify(C, film, BOARD, N.view, fn, title, frame=False)


def neg_timeline(C, film, box, now, a=1.0):
    """the negotiation on its clock: a row for each bus in the standing order, its phases as the step's events have
    them -- its fanout, its first solve, its wait for the coupled solve, its loop (each re-solve a tick, each polish
    taken in step a dot), its lay, its end -- drawn up to `now` (seconds from the negotiated route's start)"""
    N = film.S.N
    C.card(box, alpha=0.95)
    gx0, gx1 = box[0] + 170, box[2] - 30
    top, bot = box[1] + 34, box[3] - 46
    rh = (bot - top) / max(1, len(N.buses))
    X = lambda ts: gx0 + min(ts, N.t_end) / N.t_end * (gx1 - gx0)          # noqa: E731
    for m in range(0, int(N.t_end // 600) + 1):
        x = X(m * 600)
        C.line([(x, top - 6), (x, bot + 4)], mix(PANEL, (44, 48, 58), a), 1)
        C.text(x, bot + 8, f'{m * 10} min', 11, mix(PANEL, DIM, a), anchor='ma')
    cs = N.search
    if cs:
        t1 = cs['t'] - N.t0
        x0, x1 = X(max(0.0, t1 - cs['secs'])), X(min(t1, now))
        if x1 > x0:
            C.rect((x0, top - 4, x1, bot), outline=mix(PANEL, (255, 90, 210), a), width=1.5)
            C.text(x0 + 4, top - 18, 'the coupled solve', 11, mix(PANEL, (255, 90, 210), a))
    for i, b in enumerate(N.buses):
        P = N.ph[b]
        y = top + (i + 0.5) * rh
        C.text(box[0] + 14, y, f'{b}', 15, mix(PANEL, ACCENT if b == N.me else TEXT, a), bold=b == N.me, anchor='lm')
        C.text(box[0] + 60, y, f'{(P["start"] or {}).get("nets", "?")} nets', 12, mix(PANEL, DIM, a), anchor='lm')
        for k in ('fanout', 'solve', 'wait', 'loop', 'lay'):
            t0_, t1_ = P[k]
            if t0_ is None or t0_ > now:
                continue
            t1_ = min(t1_ if t1_ is not None else now, now)
            C.rect((X(t0_), y - rh * 0.28, max(X(t0_) + 1, X(t1_)), y + rh * 0.28), fill=mix(PANEL, PHASE_C[k], a))
        for tr in P['resolves']:
            if tr is not None and tr <= now:
                C.line([(X(tr), y - rh * 0.4), (X(tr), y + rh * 0.4)], mix(PANEL, TEXT, a), 1.5)
        for ts in N.syncs.get(b, []):
            if ts <= now:
                C.d.ellipse([(X(ts) - 3) * SS, (y - rh * 0.4 - 3) * SS, (X(ts) + 3) * SS, (y - rh * 0.4 + 3) * SS],
                            fill=mix(PANEL, TEXT, a))
        if P['end'] is not None and P['end'] <= now:
            x = X(P['end'])
            C.d.ellipse([(x - 6) * SS, (y - 6) * SS, (x + 6) * SS, (y + 6) * SS], outline=mix(PANEL, (150, 245, 120), a),
                        width=2 * SS)
    x = X(min(now, N.t_end))
    C.line([(x, top - 8), (x, bot + 4)], mix(PANEL, ACCENT, a), 1.2)
    lx = box[0] + 14
    for k, lab in (('fanout', 'fanout'), ('solve', 'first solve, alone'), ('wait', 'waiting for the coupled solve'),
                   ('loop', 'loop (| a re-solve, · a polish taken in step)'), ('lay', 'lay')):
        C.rect((lx, box[3] - 22, lx + 12, box[3] - 12), fill=mix(PANEL, PHASE_C[k], a))
        C.text(lx + 18, box[3] - 24, lab, 11, mix(PANEL, DIM, a))
        lx += 30 + C.tlen(lab, 11)
    C.text(lx + 10, box[3] - 24, 'o its end', 11, mix(PANEL, (150, 245, 120), a))


def neg_mins(ts):
    return f'{ts / 60:.0f} min'


def neg_scene_start(film, sc):
    """THE NEGOTIATION, before the filmed bus's own run: every bus of the group routed by a whole route of its own, all
    at once, in a standing order; each one's first solve alone, and the wait for the coupled solve"""
    S, N = film.S, film.S.N
    me = N.ph[N.me]
    t_hi = me['wait'][0] or N.t_end * 0.3
    solved = [(b, N.ph[b]['solve_ev']) for b in N.buses if N.ph[b]['solve_ev']]

    def fn(C, t, T_):
        now = ease(span(t, 0.08, 0.9)) * t_hi
        neg_view(C, film, lambda d, Z, s: None, f'the group: {len(N.buses)} buses off {S.src}, each on its own ends of '
                                                f'the shared field', field=1.0)
        neg_timeline(C, film, BRAID, now)
    order_ = ', '.join(N.order)
    sv = '; '.join(f'{b} in {e["secs"]} s' + ('' if e.get('proved') else ', kept unproved') for b, e in solved)
    at_ = lambda ts: 0.08 + 0.82 * min(1.0, ts / t_hi)                    # noqa: E731
    sc.append(Scene('negotiate', 'negotiation', 18, fn, [
        (0, f'THE NEGOTIATION. The {len(N.buses)} buses off {S.src} are routed together, each by a whole route of its '
            f'own in a process of its own, all at once: ' + ', '.join(f'{b} ({(N.ph[b]["start"] or {}).get("nets", "?")} '
                                                                     f'nets)' for b in N.buses) + '.'),
        (0.3, f'They stand in an order, {order_}: a bus plans round the lanes the buses before it publish, and every bus '
              f'round the copper any bus has laid. {N.order[-1] if N.order else N.me} plans last, round all of them.'),
        (0.6, f'Each solves its first plan alone ({sv}), publishes it, and waits: the first plans are solved again '
              f'together, in one coupled solve.')],
        says=[(0, f'The {len(N.buses)} buses off this array are routed together, each by a whole route of its own, all '
                  f'at once.'),
              (0.3, f'They stand in an order: {", ".join(spoken(b) for b in N.order)}. A bus plans around the lanes '
                    f'of the buses before it, and every bus around the copper any of them has laid.'),
              (0.6, 'Each solves its first plan alone, publishes it, and waits for the others.')],
        cues=[(0.02, 'swell', 0.4)] + [(at_(N.ph[b]['solve'][1]), 'ping', 0.3 + 0.7 * j / max(1, len(solved) - 1))
                                       for j, (b, _e) in enumerate(solved) if N.ph[b]['solve'][1] is not None
                                       and N.ph[b]['solve'][1] <= t_hi]
        + [(at_(N.ph[b]['wait'][0]), 'mark', 0.4) for b in N.buses if N.ph[b]['wait'][0] is not None
           and N.ph[b]['wait'][0] <= t_hi]))


def neg_scene_couplings(film, sc):
    """THE COUPLED SOLVE, for the whole group: where two buses' plans cross or pass near (its couplings, at their places),
    the ones the buses' own plans broke side by side, and the ones the coupled plan still breaks"""
    S, N = film.S, film.S.N
    cs = N.search or {}
    t_lo = min([N.ph[b]['wait'][0] for b in N.buses if N.ph[b]['wait'][0] is not None] or [0.0])
    t_hi = max([N.ph[b]['wait'][1] for b in N.buses if N.ph[b]['wait'][1] is not None] or [N.t_end])
    settled = len(N.settled)
    waits = ', '.join(f'{b} {neg_mins(N.ph[b]["wait"][1] - N.ph[b]["wait"][0])}' for b in N.buses
                      if N.ph[b]['wait'][0] is not None and N.ph[b]['wait'][1] is not None)

    def fn(C, t, T_):
        a_all = ease(span(t, 0.08, 0.25))
        a_al = ease(span(t, 0.35, 0.45)) * (1 - ease(span(t, 0.62, 0.7)))
        a_cp = ease(span(t, 0.62, 0.72))

        def marks(d, Z, s):
            r = max(1.5 * SS, 0.06 * s)
            if a_all > 0.02:
                for x, y in N.couplings:
                    cx, cy = Z(x, y)
                    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=mix(BODY, (200, 204, 214), 0.55 * a_all))
            R = max(3 * SS, 0.16 * s)
            for (x, y) in (N.broken_alone if a_al > 0.02 else []):
                cx, cy = Z(x, y)
                d.line([(cx - R, cy - R), (cx + R, cy + R)], fill=mix(BODY, LESSON_C, a_al), width=2 * SS)
                d.line([(cx - R, cy + R), (cx + R, cy - R)], fill=mix(BODY, LESSON_C, a_al), width=2 * SS)
            if a_cp > 0.02:
                for (x, y) in N.settled:
                    cx, cy = Z(x, y)
                    d.ellipse([cx - R, cy - R, cx + R, cy + R], outline=mix(BODY, (150, 245, 120), a_cp), width=2 * SS)
                for (x, y) in N.broken:
                    cx, cy = Z(x, y)
                    d.line([(cx - R, cy - R), (cx + R, cy + R)], fill=mix(BODY, LESSON_C, a_cp), width=3 * SS)
                    d.line([(cx - R, cy + R), (cx + R, cy - R)], fill=mix(BODY, LESSON_C, a_cp), width=3 * SS)
        neg_view(C, film, marks, 'the coupled solve: where two buses\' lanes cross or pass near', field=0.6)
        neg_timeline(C, film, BRAID, t_lo + ease(span(t, 0.05, 0.9)) * (t_hi - t_lo))
        box = (BOARD[0] + 10, BOARD[1] + 34, BOARD[0] + 560, BOARD[1] + 250)
        C.card(box)
        rows = [(f'{len(N.couplings):,} couplings: a lane of one bus crossing or passing near a lane of another', TEXT),
                (f'the buses\' own plans side by side: {len(N.broken_alone):,} broken (red)', LESSON_C if t >= 0.35 else DIM),
                (f'the coupled search: {cs.get("secs", "?")} s, {len(N.broken):,} still broken; {settled:,} settled '
                 f'(green)', (150, 245, 120) if t >= 0.62 else DIM),
                ('each bus\'s plan the coupled one, and the couplings it holds through its loop: '
                 + ', '.join(f'{b} {N.held[b]}' for b in N.buses), DIM),
                (f'the waits for it: {waits}', DIM)]
        y = box[1] + 14
        for ln, col in rows:
            for w_ in C.wrap(ln, 14, box[2] - box[0] - 28):
                if y < box[3] - 18:
                    C.text(box[0] + 14, y, w_, 14, col)
                y += 19
            y += 5
    sc.append(Scene('couplings', 'negotiation', 16, fn, [
        (0, f'THE COUPLED SOLVE. Every bus\'s model is built again into one, and the buses are COUPLED where their lanes '
            f'cross or pass near: {len(N.couplings):,} couplings, each settled by putting the two lanes on different '
            f'layers there, or broken at a price.'),
        (0.35, f'Side by side, the buses\' own plans break {len(N.broken_alone):,} of them (red).'),
        (0.62, f'The coupled search ran {cs.get("secs", "?")} s: {len(N.broken):,} still broken, {settled:,} settled '
               f'(green). Each bus takes the coupled plan, and holds its couplings through its loop. The buses that '
               f'solved alone fastest waited the longest: {waits}.')],
        says=[(0, f'Then the coupled solve: every bus\'s model built into one, coupled wherever two buses\' lanes cross '
                  f'or pass near; {len(N.couplings):,} couplings.'),
              (0.35, f'Side by side, the buses\' own plans break {len(N.broken_alone):,} of them, in red.'),
              (0.62, f'After {cs.get("secs", "?")} seconds of coupled search, {len(N.broken):,} are still broken, and '
                     f'{settled:,} are settled, in green.')],
        cues=[(0.05, 'swell', 0.4), (0.1, 'shimmer', 0.6), (0.36, 'alert', 0.5), (0.63, 'chord', 1.0)]))


def neg_scene_others(film, sc):
    """THE OTHER BUSES FINISH, while the filmed bus's loop runs: each one's laid copper on the group's board as it ends,
    its rounds and its time; from there on the filmed bus plans round it"""
    S, N = film.S, film.S.N
    me = N.ph[N.me]
    t_lo = me['loop'][0] if me['loop'][0] is not None else 0.0
    t_hi = me['loop'][1] if me['loop'][1] is not None else N.t_end
    others = [b for b in N.buses if b != N.me]
    fin = {b: N.ph[b]['laid']['t'] - N.t0 if N.ph[b]['laid'] else None for b in others}

    def fn(C, t, T_):
        now = t_lo + ease(span(t, 0.05, 0.9)) * (t_hi - t_lo)
        show = {b: ease(min(1.0, max(0.0, (now - fin[b]) / max(1.0, 0.04 * (t_hi - t_lo))))) for b in others
                if fin[b] is not None}
        neg_view(C, film, lambda d, Z, s: None, 'the others lay their copper while this bus\'s loop runs', show=show,
                 field=0.8)
        neg_timeline(C, film, BRAID, now)

    def done(b):
        lv, P = N.ph[b]['laid'] or {}, N.ph[b]
        return (f'{b} at {neg_mins(fin[b])}: {lv.get("lanes", "?")} lanes, {lv.get("vias", "?")} vias, '
                f'{lv.get("copper_mm", "?")} mm, loop rounds {N.rounds.get(b, "?")}')
    nres = len(me['resolves'])
    at_ = lambda ts: 0.05 + 0.85 * min(1.0, max(0.0, (ts - t_lo) / max(1.0, t_hi - t_lo)))   # noqa: E731
    sc.append(Scene('others', 'negotiation', 14, fn, [
        (0, f'Meanwhile the others: each bus\'s loop runs on its own, the first polish of every loop taken in step (each '
            f'bus\'s polished plan published before any audits it), then each at its own pace.'),
        (0.3, 'They finish one after another, each publishing the copper it laid: ' + '; '.join(done(b) for b in others
                                                                                                if fin[b] is not None)
         + '.'),
        (0.7, f'{N.me}, last in the order, plans round all of it: its loop solved again {pl(nres, "time")}, on a board '
              f'with their copper on it -- faint from here on -- and ended at {neg_mins(t_hi)}.')],
        says=[(0, 'Meanwhile, each of the other buses runs its own loop.'),
              (0.3, 'They finish one after another: ' + ', '.join(f'{spoken(b)} at {fin[b] / 60:.0f} minutes'
                                                                  for b in others if fin[b] is not None)
               + '. Each publishes the copper it laid.'),
              (0.7, f'This bus, last in the order, plans around all of it: its loop solves again {pl(nres, "time")}, '
                    f'and ends at {t_hi / 60:.0f} minutes.')],
        cues=[(at_(fin[b]), 'chord', 0.0) for b in others if fin[b] is not None]
        + [(at_(tr), 'tick', 0.5) for tr in me['resolves'] if tr is not None]))


def neg_scene_together(film, sc):
    """ALL TOGETHER: every bus's copper on the merged board, each with its tint and its numbers"""
    S, N = film.S, film.S.N
    gg = N.group_grade or {}
    g_open = len(gg.get('open') or []) if isinstance(gg.get('open'), list) else gg.get('open')
    g_drc = (gg.get('drc') or {}).get('n') if isinstance(gg.get('drc'), dict) else gg.get('drc')

    def row(b):
        lv, gr, P = N.ph[b]['laid'] or {}, N.ph[b]['graded'] or {}, N.ph[b]
        return (b, f'{(P["start"] or {}).get("nets", "?")} nets', f'{gr.get("vias", lv.get("vias", "?"))} vias',
                f'{gr.get("copper_mm", lv.get("copper_mm", "?"))} mm', f'{lv.get("open", "?")} open',
                f'{gr.get("drc", lv.get("drc", "?"))} DRC', f'loop rounds {N.rounds.get(b, "?")}',
                neg_mins(P['end']) if P['end'] is not None else '?')

    def fn(C, t, T_):
        a = {b: ease(span(t, 0.05 + 0.08 * j, 0.2 + 0.08 * j)) for j, b in enumerate(N.buses)}
        neg_view(C, film, lambda d, Z, s: None, 'all together: every bus\'s copper on the merged board', show=a,
                 field=0.0)
        C.card(BRAID, alpha=0.95)
        x0, y0 = BRAID[0] + 20, BRAID[1] + 16
        cols = [0, 110, 210, 320, 430, 530, 630, 790]
        for j, b in enumerate(N.buses):
            aj = a[b]
            y = y0 + j * 30
            C.rect((x0, y + 4, x0 + 26, y + 16), fill=mix(PANEL, N.tint[b], aj))
            for k, cell in enumerate(row(b)):
                C.text(x0 + 40 + cols[k], y, cell, 16, mix(PANEL, ACCENT if (k == 0 and b == N.me) else TEXT, aj),
                       bold=k == 0)
        C.text(x0, BRAID[3] - 36, f'merged: {N.conflicts if N.conflicts is not None else "?"} conflicts between the '
               f'buses; the group\'s grade: {g_open} open, {g_drc} violations, '
               f'{len((gg.get("walled") or {}).get("walled") or [])} walled', 16,
               mix(PANEL, (150, 245, 120), ease(span(t, 0.5, 0.6))))
    tot_v = sum((N.ph[b]['graded'] or {}).get('vias', 0) for b in N.buses)
    sc.append(Scene('together', 'negotiation', 14, fn, [
        (0, f'ALL TOGETHER: the {len(N.buses)} buses\' copper merged onto one board, each in its own tint, the layers in '
            f'their colours. {N.me} is the one this film follows.'),
        (0.5, f'The merge found {N.conflicts if N.conflicts is not None else "no record of"} conflicts between them, '
              f'and the group\'s grade is {g_open} open, {g_drc} violations: {tot_v} vias over the {len(N.buses)} buses.')],
        says=[(0, f'All together: the {len(N.buses)} buses\' copper, merged onto one board.'),
              (0.5, f'{N.conflicts if N.conflicts is not None else "No"} conflicts between them; the group has '
                    f'{g_open} open nets and {g_drc} design rule violations.')],
        cues=[(0.05 + 0.08 * j, 'pluck', j / max(1, len(N.buses) - 1)) for j in range(len(N.buses))]
        + [(0.5, 'chord', 1.0)]))


def with_peers(film, fn):
    """a scene of the filmed bus with the other buses' copper under it, faint: what its stages planned round"""
    N = film.S.N

    def fn2(C, t, T_):
        C.segs(N.peer_segs, alpha=0.3)
        C.cvias(N.peer_vias, 0.3)
        fn(C, t, T_)
    return fn2


def build_scenes(film, speed=1.0):
    S, T, Gt = film.S, film.S.T, film.S.Gt
    nl, npairs = len(film.M), len(S.pairs)
    nn = len(S.nets)
    plans = film.plan_meta
    W_OVER, W_V = T['W_OVER'], T['W_V']
    xo = T['xo']
    n_cross = len(T['windows'])
    cls = Gt['cls']
    nN = sum(1 for n in film.M if cls.get(n) == 'N')
    nS = sum(1 for n in film.M if cls.get(n) == 'S')
    nW = nl - nN - nS
    P1, P2 = Gt['passes'][0], Gt['passes'][-1]
    sc = []

    # ------------------------------------------------------------------ title
    def title(C, t, T_):
        C.d.rectangle([0, 0, W * SS, H * SS], fill=BG)
        a = ease(span(t, 0, 0.3))
        C.text(W / 2, 400, film.title, 64, mix(BG, TEXT, a), bold=True, anchor='mm')
        C.text(W / 2, 480, f'{nl} lanes ({nn} nets: {nl - npairs} single-ended, {npairs} differential pairs) from '
               f'{S.src} to {S.dst}', 28, mix(BG, DIM, a), anchor='mm')
        C.text(W / 2, 530, 'every lane\'s whole path planned before anything is routed -- then routed all at once',
               24, mix(BG, DIM, a), anchor='mm')
        C.text(W / 2, 600, f'{os.path.basename(S.base_path)}  ·  round {S.rn} of the fanout  ·  loop round {S.lr}  ·  '
               f'{S.tag}', 18, mix(BG, (110, 116, 128), a), anchor='mm')
        if not S.identical:
            C.text(W / 2, 660, 'NOTE: a re-run did not reproduce the chain\'s files (filmed with --force)', 20,
                   (255, 90, 90), anchor='mm')
    sc.append(Scene('title', None, 5, title, [],
                    says=[(0, f'{film.title}. {nl:,} lanes, each one\'s whole path planned before anything is routed.')],
                    cues=[(0, 'swell', 0.3)]))

    # ------------------------------------------------------------------ the bench
    def bench(C, t, T_):
        a = ease(span(t, 0.1, 0.5))
        for ref in (S.src, S.dst):
            for p in S.fo.footprints[ref].pads:
                if (p.net_name or '').split('/')[-1] in S.nets:
                    C.wring(p.global_x, p.global_y, 0.22, mix(BODY, ACCENT, a), 1.6)
        C.segs(film.other_c[0], alpha=0.22)
        for ref, lab in ((S.src, 'source'), (S.dst, 'destination')):
            fp = S.fo.footprints[ref]
            xs = [p.global_x for p in fp.pads]
            ys = [p.global_y for p in fp.pads]
            C.wtext((min(xs) + max(xs)) / 2, min(ys) - 0.35, f'{ref} ({lab})', 18, mix(BODY, TEXT, a), anchor='mb', bold=True)
    sc.append(Scene('bench', None, 5, bench, [
        (0, f'The bench. {nl} lanes to route from {S.src} to {S.dst}: the lit balls ({nn} nets, {npairs} of the lanes are '
            f'differential pairs). Every other net and part on the board is an obstacle.')],
        says=[(0, f'{nl:,} lanes to route, from {spoken(S.src)} to {spoken(S.dst)}. Everything else is an obstacle.')],
        cues=[(0.1, 'shimmer', 0.5)]))                    # (the balls lit)

    # ------------------------------------------------------------------ the group's ends (a bus step's run)
    if film.g:
        group_scenes(film, sc)
    if film.neg:
        neg_scene_start(film, sc)

    # ------------------------------------------------------------------ the ends
    e0 = next((e for e in S.ends if e[0].startswith('on the teeth')), None)
    e1 = next((e for e in S.ends if e[0].startswith('teeth moved')), None)
    # the wiring diagram's crossings: where each pair of lanes whose two orders disagree cross, straight from launch
    # rank to final rank (the k-th drawn from t = 0.8 + 0.16 k / n_cross)
    xa_, xb_ = film.bx(T['S_FACE']), film.bx(max(T['end'].values()))
    rk = {n: ((xa_, film.rank_y(film.li[n])), (xb_, film.rank_y(film.fi[n]))) for n in film.M}
    xdots = []
    for key in T['windows']:
        a_, b_ = key.split('|')
        (p0, p1), (q0, q1) = rk[a_], rk[b_]
        d1, d2 = (p1[0] - p0[0], p1[1] - p0[1]), (q1[0] - q0[0], q1[1] - q0[1])
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-9:
            continue
        s_ = ((q0[0] - p0[0]) * d2[1] - (q0[1] - p0[1]) * d2[0]) / den
        xdots.append((p0[0] + d1[0] * s_, p0[1] + d1[1] * s_))

    def ends(C, t, T_):
        a_old = 1.0 - ease(span(t, 0.28, 0.45))
        a_new = ease(span(t, 0.30, 0.48))
        a_b = ease(span(t, 0.52, 0.66))
        C.segs(film.other_c[0], alpha=0.22)
        C.segs(film.stub_kept, alpha=0.95)
        C.cvias(film.via_kept)
        if a_old > 0:
            C.segs(film.stub_gone, alpha=a_old)
            C.cvias(film.via_gone, a_old)
        if a_new > 0:
            C.segs(film.teeth_new, alpha=a_new)
            C.cvias(film.tvia_new, a_new)
            hl = 1.0 - ease(span(t, 0.5, 0.7))
            for s in film.teeth_new:
                C.wring(s[0], s[1], 0.18, mix(BODY, ACCENT, hl * a_new), 1.4)
        if a_b > 0:
            C.segs(film.berths_new, alpha=a_b)
            C.cvias(film.bvia_new, a_b)
        # the two orders, numbered
        a_o = ease(span(t, 0.7, 0.82))
        if a_o > 0:
            for n in film.M:
                tooth, _s, _l, berth = film.ends[n]
                C.wdot(*tooth, 3.2, mix(BODY, TEXT, a_o))
                C.wdot(*berth, 3.2, mix(BODY, TEXT, a_o))
        # the wiring diagram: each lane from its launch rank to its final rank, straight -- every crossing is a pair
        # of lanes whose two orders disagree
        a_w = ease(span(t, 0.74, 0.88))
        if a_w > 0:
            C.card(BRAID, alpha=0.92)
            funnel(C, film, a_w)
            xa, xb = film.bx(T['S_FACE']), film.bx(max(T['end'].values()))
            C.text(xa - 6, BRAID[1] + 8, 'teeth (launch order)', 12, DIM, anchor='ra')
            C.text(min(xb + 6, BRAID[2] - 8 - C.tlen('berths (final order)', 12)), BRAID[1] + 8, 'berths (final order)',
                   12, DIM, anchor='la')
            for n in film.M:
                (_, ya), (_, yb) = rk[n]
                C.line([(xa, ya), (xb, yb)], mix(PANEL, PAIR_COL if n in S.pairs else (170, 178, 190), a_w), 1.4)
                C.text(xa - 6, ya, n, 10, mix(PANEL, DIM, a_w), anchor='rm')
                C.text(xb + 6, yb, n, 10, mix(PANEL, DIM, a_w), anchor='lm')
            for k, (x, y) in enumerate(xdots, 1):
                if k / n_cross <= span(t, 0.8, 0.96):
                    C.d.ellipse([(x - 2.6) * SS, (y - 2.6) * SS, (x + 2.6) * SS, (y + 2.6) * SS], fill=ACCENT)
            C.text((xa + xb) / 2, BRAID[3] - 8, f'{n_cross} pairs of lanes disagree between the two orders: '
                   f'{n_cross} crossings the route must make', 14, mix(PANEL, ACCENT, ease(span(t, 0.9, 1.0))), anchor='mb')
    cap_e = [(0, f'The ends. Each net leaves {S.src} through a short stub (its TOOTH) and arrives at {S.dst} through '
                 f'another (its BERTH). ' + ('Here are the teeth as this round started.' if film.stub_kept
                                             else 'Here is the board before any of them is laid.'))]
    if film.g:
        cap_e = [(0, f'ON THE FIELD: {S.dst}\'s own whole route. Its teeth at {S.src} and its berths at {S.dst} are the '
                     'group\'s, as the field laid them: this run fans nothing out and chooses no end again.')]
    elif S.grouped:
        cap_e.append((0.26, 'These ends were chosen before this step, by a model that chooses the ends of every bus on '
                            'these arrays together and lays them: this step fans nothing out and chooses no end again.'))
    if e0 and e1:
        cap_e.append((0.26, f'The ends model searches the fanout\'s escape menus, judged by the whole route\'s own '
                            f'estimate: with the teeth as they were, {e0[1]} vias and {e0[2]} crossings predicted; '
                            f'moving {e1[0].split("(")[1].rstrip(")")} teeth, {e1[1]} vias and {e1[2]} crossings.'))
    if not S.grouped:
        cap_e.append((0.5, f'...and every berth at {S.dst} with them. The fanout lays exactly what the model chose, and '
                           f'it is judged as laid.'))
    cap_e.append((0.72, 'The ends decide the braid: the teeth\'s order round the source and the berths\' order round '
                        'the destination fix every crossing -- two lanes whose orders disagree must cross once.'))
    if film.g:
        say_e = [(0, f'On the field, {spoken(S.dst)}\'s own whole route begins. Its teeth and berths are the group\'s, as the '
                     'field laid them; this run chooses no end again.')]
    else:
        say_e = [(0, 'Each lane leaves by a short stub, its tooth, and arrives by another, its berth.')]
        if e0 and e1:
            say_e.append((0.26, 'The ends model searches the fanout\'s escapes, judged by the whole route\'s own '
                                'estimate.'))
        elif S.grouped:
            say_e.append((0.26, 'These ends were chosen earlier, by one model for every bus on these arrays together. '
                                'This step keeps them.'))
        if not S.grouped:
            say_e.append((0.5, 'The berths are chosen with them, and the fanout lays exactly what the model chose.'))
    say_e.append((0.72, f'The two orders, round the source and round the destination, fix every crossing: {n_cross:,} '
                        'pairs of lanes must cross.'))
    # (the stubs moved away fade, the new teeth and berths come in, the orders are numbered, the wiring diagram, and
    # its crossings one by one)
    cue_e = ([(0.28, 'glide', 0.3)] if film.stub_gone or film.via_gone else []) \
        + ([(0.30, 'shimmer', 0.6)] if film.teeth_new or film.tvia_new else []) \
        + ([(0.52, 'shimmer', 0.75)] if film.berths_new or film.bvia_new else []) \
        + [(0.7, 'shimmer', 0.4), (0.74, 'swell', 0.5)] \
        + [(0.8 + 0.16 * k / n_cross, 'tick', k / len(xdots)) for k in range(1, len(xdots) + 1)]
    # (on the group's field nothing of the ends changes in this run: the scene is the orders and their crossings)
    sc.append(Scene('ends', 'ends', 11 if film.g else 16, ends, cap_e, says=say_e, cues=cue_e))

    def draw_fanout(C, alpha=0.9):
        C.segs(film.other_c[0], alpha=0.22)
        C.segs(film.fo_c[0], alpha=alpha)
        C.cvias(film.fo_c[1], alpha)

    # ------------------------------------------------------------------ the taut strings
    # each lane's string as every stage reads it, and its chord (the straight line its relaxation starts from).
    # Between the two: the relaxation's own rounds where the geometry's re-run ran it (film.relax: no memo of the
    # strings on disk), each string as it stood after them -- else a blend of the two, and the scene says so
    STRC = (236, 222, 130)
    rsteps = collections.OrderedDict()                       # the relaxation's rounds at each spacing
    for step_, k_, _n in film.relax_at:
        rsteps[step_] = max(rsteps.get(step_, 0), k_)
    slen = {n: plen(P) for n, P in film.strings.items()}
    schord = {n: math.hypot(P[-1][0] - P[0][0], P[-1][1] - P[0][1]) for n, P in film.strings.items()}
    bent = sorted((n for n in slen if slen[n] > schord[n] + 0.01), key=lambda n: schord[n] - slen[n])
    swin = None
    if bent:
        # up close: where the most bent string stands farthest off its chord
        P = np.asarray(film.strings[bent[0]], float)
        ab = P[-1] - P[0]
        off = np.abs((P[:, 0] - P[0, 0]) * ab[1] - (P[:, 1] - P[0, 1]) * ab[0]) / max(1e-9, math.hypot(*ab))
        cx, cy = P[int(np.argmax(off))]
        swin = (cx - 3.4, cy - 1.0, cx + 3.4, cy + 1.0)

    def strings(C, t, T_):
        draw_fanout(C, 0.7)
        a = ease(span(t, 0.03, 0.14))
        m = ease(span(t, 0.24, 0.62))
        at = None
        if film.relax:
            u_ = span(t, 0.24, 0.66)
            j_ = int(min(len(film.relax) - 2, max(0, np.searchsorted(film.relax_t, u_, side='right') - 1)))
            f_ = span(u_, film.relax_t[j_], film.relax_t[j_ + 1])
            st = {n: (A + (film.relax[j_ + 1][n] - A) * ease(f_), None) for n, A in film.relax[j_].items()}
            at = film.relax_at[min(j_, len(film.relax_at) - 1)] if t >= 0.24 else None
        else:
            st = morph(film.st['chord'], film.st['string'], m)
        for n, (P, _b) in st.items():
            C.wline([tuple(q) for q in P], mix(BODY, PAIR_COL if n in S.pairs else STRC, a), 1.3)
        for n in film.strings:
            tooth, _s, _l, berth = film.ends[n]
            C.wdot(*tooth, 2.6, mix(BODY, TEXT, a))
            C.wdot(*berth, 2.6, mix(BODY, TEXT, a))
        box = (BRAID[0], BRAID[1], BRAID[0] + 1260, BRAID[3])
        if swin:
            def draw(d, Z, s):
                for n, (P, _b) in st.items():
                    d.line([Z(*q) for q in simplify(P, 0.3 / s * SS)], fill=PAIR_COL if n in S.pairs else STRC,
                           width=max(1, int(1.4 * SS)), joint='curve')
            magnify(C, film, box, swin, draw,
                    f'up close, where {bent[0]}\'s string bends most: straight, then taut round the pads')
        else:
            C.card(box, alpha=0.95)
        box3 = (BRAID[0] + 1280, BRAID[1], BRAID[2], BRAID[3])
        C.card(box3, alpha=0.95)
        lines = [(f'{len(film.strings)} strings, one a lane: {len(bent)} bend round copper, '
                  f'{len(film.strings) - len(bent)} stay straight', TEXT)]
        if bent:
            lines += [(f'the most bent, {bent[0]}: {slen[bent[0]]:.1f} mm, its straight line {schord[bent[0]]:.1f} mm', DIM)]
        lines += [('a string has no width and no layer', ACCENT)]
        if film.relax:
            lines += [('pulled taut in ' + ', then '.join(f'up to {k_} rounds with a point every {st_:.2f} mm'
                                                          for st_, k_ in rsteps.items()), DIM),
                      ('now: straight, to begin with' if t < 0.24 else
                       f'now: round {at[1]}, a point every {at[0]:.2f} mm; {pl(at[2], "string")} still moving'
                       if at and t < 0.66 else 'now: taut', TEXT),
                      ('(each string drawn as it stood after rounds 1, 2, 3, 4, 8, 16, 25, 32, 50 ..; between them, a '
                       'blend)', (110, 116, 128))]
        else:
            lines += [('(drawn: the straight line and the taut string; the frames between are a blend)', (110, 116, 128))]
        y_ = box3[1] + 14
        for ln, col in lines:
            for w_ in C.wrap(ln, 15, box3[2] - box3[0] - 28):
                C.text(box3[0] + 14, y_, w_, 15, mix(PANEL, col, a))
                y_ += 21
            y_ += 6
    sc.append(Scene('strings', 'strings', 12, strings, [
        (0, 'The taut strings. Each lane is first a STRING from its tooth to its berth -- a straight line across the '
            'board, through whatever stands between.'),
        (0.22, ('Each string is then pulled TAUT, round after round: every point moves toward the middle of its two '
                'neighbours and is put back outside any copper it has entered, the two ends held.' if film.relax else
                'Each string is then pulled TAUT: it shortens wherever it can and is pushed off the other nets\' copper, '
                'its two ends held.')
               + ' What is left is the lane\'s way round the obstacles -- which side of each it passes.'),
        (0.66, f'{len(bent)} of the {len(film.strings)} strings bend round something. A string has no width and no '
               f'layer yet: the frame, the solve and the geometry are all built on these.')],
        says=[(0, 'Each lane starts as a straight string.'),
              (0.22, 'Pulled taut, round after round, it finds its way around the copper in its path.'),
              (0.66, f'{len(bent):,} of them bend. A string has no width, and no layer, yet.')],
        # (the strings drawn; each recorded round of their relaxation -- the higher, the fewer still moving -- or the
        # blend where there is none; taut)
        cues=[(0.03, 'shimmer', 0.3)]
        + ([(0.24 + 0.42 * film.relax_t[j], 'tick', 1 - n_ / max(1, len(film.strings)))
            for j, (_s, _k, n_) in enumerate(film.relax_at)] if film.relax else [(0.24, 'glide', 0.5, 0.62)])
        + [(0.66, 'pluck', 0.2)]))

    # ------------------------------------------------------------------ the frame

    def draw_frame(C, a, ticks=True, refs=0.0):
        mp = film.mp
        FRC = (210, 214, 226)
        nrm = np.array([-film.spd[1], film.spd[0]])
        # the face band: no crossing and no change there
        q = [film.sp0 + film.spd * u for u in (T['S_FACE'], T['BAND'])]
        band = [tuple(q[0] - nrm * 7), tuple(q[1] - nrm * 7), tuple(q[1] + nrm * 7), tuple(q[0] + nrm * 7)]
        C.d.polygon([film.X(*b_) for b_ in band], fill=mix(BODY, (60, 64, 76), a * 0.8))
        sp = [mp.pt(*p) for p in Gt['spine']]
        C.wline(sp, mix(BODY, FRC, a), 2.0)
        pm = film.sp0 + film.spd * ((T['BAND'] + Gt['H0']) / 2)
        C.wtext(*(pm + nrm * 0.3), 'trunk', 16, mix(BODY, FRC, a), anchor='mt', bold=True)
        for k, ring in Gt['rings'].items():
            R = [mp.pt(*p) for p in ring]
            C.wline(R, mix(BODY, FRC, a * 0.9), 2.0)
            far = max(R, key=lambda p_: p_[0])
            C.wtext(far[0] + 0.3, far[1], f'ring {k}', 16, mix(BODY, FRC, a), anchor='lm', bold=True)
        if ticks:
            for u in range(int(math.ceil(film.u0)), int(Gt['H0']) + 1):
                p = film.sp0 + film.spd * u
                nrm = np.array([-film.spd[1], film.spd[0]])
                q0, q1 = p - nrm * 0.12, p + nrm * 0.12
                C.wline([tuple(q0), tuple(q1)], mix(BODY, (190, 196, 210), a), 1.2)
                if u % 5 == 0:
                    C.wtext(*(p - nrm * 0.25), f'u={u}', 11, mix(BODY, (190, 196, 210), a), anchor='mb')
        if refs > 0:
            for n in film.M:
                pts = []
                for f_, k_, o_ in Gt['refs'][n]:
                    u_, b_, nv_ = film.cols[(f_, k_)]
                    pts.append((b_[0] + o_ * nv_[0], b_[1] + o_ * nv_[1]))
                C.wline(pts, mix(BODY, (200, 200, 120), refs * 0.6), 1.0)

    def frame(C, t, T_):
        draw_fanout(C)
        a = ease(span(t, 0.0, 0.3))
        a_s = 0.55 * (1 - ease(span(t, 0.45, 0.6)))             # the strings, until the frame's reading of them
        if a_s > 0.02:
            for n, (P, _b) in film.st['string'].items():
                C.wline([tuple(q) for q in P], mix(BODY, PAIR_COL if n in S.pairs else STRC, a_s), 1.0)
        draw_frame(C, a, refs=ease(span(t, 0.45, 0.65)) * (1 - ease(span(t, 0.9, 1.0))))
        C.card(BRAID, alpha=0.92)
        ab = ease(span(t, 0.2, 0.5))
        funnel(C, film, ab)
        X = film.bx
        for u in range(int(math.ceil(film.u0)), int(film.u1) + 1):
            x = X(u)
            C.line([(x, BRAID[1] + 16), (x, BRAID[3] - 10)], mix(PANEL, (40, 44, 52), ab), 1)
            if u % 5 == 0:
                C.text(x, BRAID[3] - 8, f'u = {u} mm', 10, mix(PANEL, DIM, ab), anchor='mb')
        xb0, xb1 = X(T['S_FACE']), X(T['BAND'])
        C.rect((xb0, BRAID[1] + 4, xb1, BRAID[3] - 18), fill=mix(PANEL, (34, 38, 46), ab))
        for n in film.M:
            x0, x1 = X(T['entry'][n]), X(T['end'][n])
            y0, y1 = film.rank_y(film.li[n]), film.rank_y(film.fi[n])
            C.line([(x0, y0), (x0 + 30, y0)], mix(PANEL, DIM, ab), 1.4)
            C.line([(x1 - 30, y1), (x1, y1)], mix(PANEL, DIM, ab), 1.4)
            C.text(x0 - 5, y0, n, 10, mix(PANEL, PAIR_COL if n in S.pairs else DIM, ab), anchor='rm')
        C.text(W / 2, BRAID[1] + 8, 'the solve\'s plane: u along the route (the trunk, then a ring) -- '
               'the lanes in ORDER, north to south', 13, mix(PANEL, TEXT, ab), anchor='ma')
    sc.append(Scene('frame', 'frame', 10, frame, [
        (0, f'The frame, read off the board alone: a straight TRUNK from the source\'s box through the destination\'s, '
            f'and a RING round the destination for each of its faces ({nN} lanes north, {nS} south, {nW} end on its '
            f'near face).'),
        (0.3, 'Every lane\'s route is ONE coordinate u: along the trunk, then round its ring. On the trunk u is '
              'distance along the board -- so the plane below shares the board\'s x axis.'),
        (0.45, 'The frame reads each lane\'s taut string in these coordinates: its REFERENCE (yellow), an offset from '
               'the trunk or ring at every u. Inside an array\'s pad box, where a string threads between balls no track '
               'fits, the reference takes the box\'s edge.')],
        says=[(0, 'The frame: a straight trunk from source to destination, and rings around the destination.'),
              (0.45, 'Every route is now one coordinate along it, and each string becomes a reference, in yellow.')],
        cues=[(0.2, 'swell', 0.4), (0.45, 'shimmer', 0.55)]))       # (the solve's plane; the references)

    # ------------------------------------------------------------------ the solve
    np_ = len(plans)
    rdr = lambda p: (f'{pl(p["over"], "via")} past two on a net, ' if p['over'] else '') + f'{p["vias"]} layer changes'
    log = [ln for r in T['runs'] for ln in r['log']]
    mdl = next((ln for ln in log if ln.startswith('#Model')), '')
    mv = re.search(r'var:(\d+)/(\d+)\s+constraints:(\d+)/(\d+)', mdl)
    # the search's events, in its log's order: each plan it found (#1, #2, ..) and each rise of its lower bound
    # (#Bound), with the best and the bound after it -- a plan with nothing left between best and bound is proved.
    # (A search of several runs -- the workers that raise the bound, then the ones that find plans: each run's clock
    # starts at zero, and the times drawn are since the first began; a later run starts from the bound so far)
    events, t_off, lb_so_far = [], 0.0, 0.0
    for r_ in T['runs']:
        t_run = 0.0
        for ln in r_['log']:
            m = re.match(r'#(Bound|\d+)\s+([\d.]+)s\s+best:(\S+)\s+next:\[([^\]]*)\]', ln)
            if not m:
                continue
            best = float(m.group(3))
            nx = m.group(4).split(',')
            lb_so_far = max(lb_so_far, float(nx[0]) if nx[0] else best)
            t_run = max(t_run, float(m.group(2)))
            events.append({'plan': m.group(1) != 'Bound', 't': t_off + float(m.group(2)), 'best': best, 'lb': lb_so_far})
        t_off += max([t_run, r_.get('secs', 0.0)] + [p_['t'] for p_ in r_['plans']])
    t_end = max(t_off, 0.05)
    last = plans[-1]
    proved = T['final'].get('proved', True)
    if proved and all(p_['over'] == last['over'] for p_ in plans):
        fl = lambda v: (v - last['over'] * W_OVER) / W_V      # objective in layer changes, the over-two count held
        ylab = 'layer changes (the vias past two on a net held)'
    else:
        fl = lambda v: v / W_V
        ylab = f'the objective: layer changes, + {W_OVER / W_V:.0f} for each via past two on a net'
    # the plans' slots: plan i holds from k_in + i seg_. Each event's moment in the film: a plan at its slot, a bound
    # raised just before the next plan
    k_in = 0.14
    seg_ = (0.62 - k_in) / max(1, np_)
    when, jp = [], 0
    for e in events:
        if e['plan']:
            when.append(k_in + jp * seg_)
            jp += 1
        else:
            when.append(k_in - 0.05 if jp == 0 else k_in + (jp - 1) * seg_ + 0.7 * seg_)

    def solve(C, t, T_):
        draw_fanout(C, 0.55)
        # which plan: plan i holds from its slot; the morphs between
        pi = min(np_ - 1, max(0, int((t - k_in) / seg_))) if t >= k_in else 0
        # the plan drawn: the last of the ones drawn (film.pshow) found by now, moved to from the one drawn before it
        ps = film.pshow
        d = max(j for j, q in enumerate(ps) if q <= pi)
        t_d = k_in + ps[d] * seg_
        t_n = k_in + ps[d + 1] * seg_ if d + 1 < len(ps) else 0.62
        mt = ease(span(t, t_d, t_d + 0.3 * (t_n - t_d))) if d > 0 else 1.0
        ia, ib = ps[max(d - 1, 0)], ps[d]
        cur = lerp_plan(film.plans[ia], film.plans[ib], mt)
        a_in = ease(span(t, 0.08, k_in + 0.02))
        st = morph(film.st[f'plan{ia}'], film.st[f'plan{ib}'], mt)
        C.lanes(st, alpha=a_in, w=2.2)
        C.vias([(n, x, y) for n, x, y, al in morph_vias(film.vias[f'plan{ia}'], film.vias[f'plan{ib}'], mt)
                if al > 0.5], alpha=a_in)
        # the rule, shown once proved: one crossing and one change, lit on the braid and on the board
        a_r = ease(span(t, 0.7, 0.76)) * (1 - ease(span(t, 0.97, 1.0)))
        draw_braid(C, film, cur, alpha=max(0.15, a_in))
        # the search card
        a_c = ease(span(t, 0.0, 0.08))
        box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 400, BOARD[1] + 300)
        C.card(box)
        x0, y0 = box[0] + 16, box[1] + 14
        C.text(x0, y0, 'CP-SAT', 20, ACCENT, bold=True)
        rows = [f'{n_cross} crossing positions t (u along the route)',
                f'{T["KMAX"]} layer-change slots c per lane',
                f'{T["triples"]} braid triples: every three lanes cross',
                '   in one consistent order',
                'crossing lanes on different layers',
                'each lane ends on its berth\'s layer',
                'crossings and changes keep their room',
                'no crossing or change in the face band']
        if mv:
            rows.append(f'{mv.group(2)} variables, {mv.group(4)} constraints')
        for j, r in enumerate(rows):
            C.text(x0, y0 + 32 + j * 19, r, 14, mix(PANEL, TEXT if j < 3 else DIM, a_c))
        C.text(x0, y0 + 32 + len(rows) * 19 + 6, 'minimise: vias past two on a net, then vias', 14, mix(PANEL, ACCENT, a_c))
        # the search chart, event by event (CP-SAT's plans came within hundredths of a second of each other)
        a_g = ease(span(t, k_in - 0.06, k_in))
        box2 = (BOARD[2] - 440, BOARD[1] + 6, BOARD[2] - 6, BOARD[1] + 310)
        C.card(box2)
        gx0, gy0, gx1, gy1 = box2[0] + 58, box2[1] + 68, box2[2] - 24, box2[3] - 64
        C.text(box2[0] + 16, box2[1] + 12, f'the search: {t_end:.2f} s of real time' if t_end < 10
               else f'the search: {t_end:.0f} s of real time', 16, mix(PANEL, TEXT, a_g), bold=True)
        C.text(box2[0] + 16, box2[1] + 32, ylab, 11, mix(PANEL, DIM, a_g))
        vals = [fl(e['best']) for e in events if e['best'] < math.inf] + [fl(e['lb']) for e in events]
        vmax, vmin = max(vals) + 1, max(0, min(vals) - 2)
        gy = lambda v: gy1 - (v - vmin) / (vmax - vmin) * (gy1 - gy0)
        E = max(1, len(events))
        gxx = lambda j: gx0 + (j + 0.5) / E * (gx1 - gx0)
        C.line([(gx0, gy0), (gx0, gy1), (gx1, gy1)], mix(PANEL, DIM, a_g), 1)
        for v in range(int(math.ceil(vmin)), int(vmax) + 1, max(1, int((vmax - vmin) / 5))):
            C.text(gx0 - 6, gy(v), str(v), 11, mix(PANEL, DIM, a_g), anchor='rm')
        shown = [j for j, w in enumerate(when) if t >= w]
        pb, pl = [], []
        for j in shown:
            e = events[j]
            if e['best'] < math.inf:
                if pb:
                    pb.append((gxx(j), pb[-1][1]))
                pb.append((gxx(j), gy(fl(e['best']))))
            if pl:
                pl.append((gxx(j), pl[-1][1]))
            pl.append((gxx(j), gy(fl(e['lb']))))
        if pl:
            C.line(pl + [(gxx(shown[-1]) + 0.4 * (gx1 - gx0) / E, pl[-1][1])], mix(PANEL, (90, 230, 255), a_g), 2)
        if pb:
            C.line(pb + [(gxx(shown[-1]) + 0.4 * (gx1 - gx0) / E, pb[-1][1])], mix(PANEL, TEXT, a_g), 2)
        k_ = 0
        every = max(1, int(math.ceil(E / 7)))                  # (a long search: a time and a label for some only)
        for j in shown:
            e = events[j]
            x = gxx(j)
            if j % every == 0 or j == E - 1:
                C.text(x, gy1 + 6, f'{e["t"]:.2f}s' if t_end < 10 else f'{e["t"]:.0f}s', 10, mix(PANEL, DIM, a_g), anchor='ma')
            if e['plan']:
                y = gy(fl(e['best']))
                r_ = 4 if np_ <= 8 else 2.5
                C.d.ellipse([(x - r_) * SS, (y - r_) * SS, (x + r_) * SS, (y + r_) * SS], fill=TEXT)
                if np_ <= 8 or k_ in (0, np_ - 1):
                    C.text(x + 7, y - 3, f'plan {k_ + 1}: {plans[k_]["vias"]}', 12, TEXT,
                           anchor='lb' if k_ < np_ - 1 or np_ <= 8 else 'rb')
                k_ += 1
        C.text(gx1, gy1 + 22, 'white: the best plan   cyan: the lower bound', 11, mix(PANEL, DIM, a_g), anchor='ra')
        if len(film.pshow) < np_:
            C.text(gx0, gy1 + 22, f'drawn: {len(film.pshow)} of its {np_} plans', 11, mix(PANEL, DIM, a_g))
        if t >= 0.62:
            ap = ease(span(t, 0.62, 0.66))
            C.text((box2[0] + box2[2]) / 2, box2[3] - 20, f'best = bound: {last["vias"]} changes, PROVED optimal' if proved
                   else f'the best found: {last["vias"]} changes -- not proved the fewest', 15,
                   mix(PANEL, ACCENT, ap), bold=True, anchor='mm')
        # the rules, lit on one crossing and one change
        if a_r > 0:
            fin = film.plans[-1]
            mid_u = (T['BAND'] + Gt['H0']) / 2
            trunk = [(k, u) for k, u in fin.cross.items() if not any(Gt['cls'].get(n) and u > T['Hn'].get(n, 1e9) for n in k)]
            if trunk:
                kx, ux = min(trunk, key=lambda e: abs(e[1] - mid_u))
                a_, b_ = sorted(kx)
                stb = braid_strands(film, fin)
                ys = [float(np.interp(ux, stb[n][0], stb[n][1])) for n in (a_, b_)]
                x, y = film.bx(ux), film.rank_y(sum(ys) / 2)
                R = 11
                C.d.ellipse([(x - R) * SS, (y - R) * SS, (x + R) * SS, (y + R) * SS], outline=mix(PANEL, ACCENT, a_r), width=2 * SS)
                la, lb = (film.LN[fin.layer(n_, ux)].split('.')[0] for n_ in (a_, b_))
                C.text(x + 14, y - 14, f'{a_} on {la} crosses {b_} on {lb}', 13, mix(PANEL, ACCENT, a_r), anchor='lb')
                # ...and on the board: where the slots swap
                Pa, _ba, Ua = film.raw[f'plan{np_ - 1}'][a_]
                Pb, _bb, Ub = film.raw[f'plan{np_ - 1}'][b_]
                qa, qb = along(Pa, Ua, ux), along(Pb, Ub, ux)
                C.wring((qa[0] + qb[0]) / 2, (qa[1] + qb[1]) / 2, 0.45, mix(BODY, ACCENT, a_r), 2)
            chs = [(n, c) for n in film.M for c in fin.changes.get(n, [])]
            if chs:
                n, c = min(chs, key=lambda e: abs(e[1] - mid_u - 0.8))
                stb = braid_strands(film, fin)
                x, y = film.bx(c), film.rank_y(float(np.interp(c, stb[n][0], stb[n][1])))
                R = 11
                C.d.ellipse([(x - R) * SS, (y - R) * SS, (x + R) * SS, (y + R) * SS], outline=mix(PANEL, (90, 230, 255), a_r), width=2 * SS)
                C.text(x + 14, y + 14, f'{n} changes layer: a via', 13, mix(PANEL, (90, 230, 255), a_r), anchor='lt')
                P, _b, U = film.raw[f'plan{np_ - 1}'][n]
                q = along(P, U, c)
                C.wring(q[0], q[1], 0.45, mix(BODY, (90, 230, 255), a_r), 2)
    cap_s = [(0, f'The solve (CP-SAT) decides every crossing and every layer change, all lanes at once. Its variables '
                 f'are positions along u: where each of the {n_cross} crossings happens, and where each lane changes '
                 f'layer.'),
             (0.14, ('Below, each plan it finds as a braid: red F, blue B, a white ring is a via.' if len(film.LN) <= 2
                     else f'Below, each plan it finds as a braid: its colour the layer ({len(film.used)} of the '
                          f'{len(film.LN)} routing layers carry lanes), a white ring a via.')
                    + ' Above, the same plan on the board, each lane in its references\' slot in that order -- the solve '
                      'knows ORDER and LAYERS, not millimetres.'),
             (0.3, 'The search: ' + (' -> '.join(rdr(p) for p in plans) if np_ <= 4 else
                                      f'{np_} plans, from {rdr(plans[0])} to {rdr(last)}')
                   + '. The lower bound rises as it goes.'),
             (0.62, f'Best meets bound: {last["vias"]} layer changes, proved optimal. Only a plan proved optimal in '
                    f'its vias goes on to the geometry.' if proved else
                    f'The search stops at a count of its own work, not at a clock, so it gives the same plan each time. '
                    f'Its best has {rdr(last)}: not proved the fewest, and kept all the same -- a round\'s first plan '
                    f'goes on to the geometry, and is laid.'),
             (0.72, 'The rules can be seen: two strands that cross are on different layers (a via lets a lane cross '
                    'lanes on its own layer), and each lane ends on its berth\'s layer.')]
    say_s = [(0, 'The solve places every crossing and every layer change, all lanes at once.'),
             (0.14, 'Below, each plan as a braid: red front, blue back, white rings for vias.' if len(film.LN) <= 2 else
                    f'Below, each plan as a braid: each colour one of its {len(film.used):,} layers, white rings for '
                    f'vias.'),
             (0.3, (f'It finds {np_:,} plans, from {plans[0]["vias"]:,} layer changes down to {last["vias"]:,}, while its '
                    f'lower bound rises.' if np_ > 1 and plans[0]['vias'] != last['vias'] else
                    f'It finds {pl(np_, "plan")}, with {last["vias"]:,} layer changes, while its lower bound rises.')),
             (0.62, 'Best meets bound: proved the fewest.' if proved else
                    'It stops after a fixed amount of work: its best is kept, though not proved the fewest.'),
             (0.72, 'Where two strands cross, they are on different layers, and each lane ends on its berth\'s layer.')]
    # (the plans' braid comes in; every plan found -- higher, the fewer its layer changes -- and every bound raised, at
    # its moment in the chart; each plan the board and the braid move to; best meets bound, or a best kept unproved;
    # the rules lit on one crossing and one change)
    o0, o1 = fl(plans[0]['obj']), fl(last['obj'])
    better = lambda v: min(1.0, max(0.0, (o0 - fl(v)) / (o0 - o1))) if o0 > o1 else 1.0
    cue_s = [(0.08, 'swell', 0.3)] \
        + [(w, 'ping', better(e['best'])) if e['plan'] else (w, 'thump', 0.5) for w, e in zip(when, events)] \
        + [(k_in + q * seg_, 'bell', better(plans[q]['obj'])) for q in film.pshow[1:]] \
        + [(0.62, 'chord', 1.0 if proved else 0.0), (0.7, 'mark', 0.5)]
    sc.append(Scene('solve', 'solve', 26, solve, cap_s, says=say_s, cues=cue_s))

    # ------------------------------------------------------------------ the coupled solve (a negotiated bus's round)
    if film.neg:
        neg_scene_couplings(film, sc)
    if film.coupled:
        jn = S.coupled['joint']
        v_own, v_cp = (sum(len(v) for v in P_.changes.values()) for P_ in (film.root, film.coupled))
        moved_ = sum(1 for k in film.root.cross if abs(film.root.cross[k] - film.coupled.cross.get(k, film.root.cross[k]))
                     > 1e-6)
        chm_ = sum(1 for n in film.M if film.root.changes.get(n, []) != film.coupled.changes.get(n, [])
                   or film.root.layers.get(n) != film.coupled.layers.get(n))
        nbr = len(jn.get('broken') or [])

        def coupled(C, t, T_):
            draw_fanout(C, 0.55)
            m = ease(span(t, 0.3, 0.7))
            C.lanes(morph(film.st['root'], film.st['coupled'], m), w=2.2)
            C.vias([(n, x, y) for n, x, y, al in morph_vias(film.vias['root'], film.vias['coupled'], m) if al > 0.5])
            draw_braid(C, film, lerp_plan(film.root, film.coupled, m))
            box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 600, BOARD[1] + 300)
            a = ease(span(t, 0.0, 0.08))
            C.card(box)
            C.text(box[0] + 16, box[1] + 14, 'the coupled solve', 20, mix(PANEL, ACCENT, a), bold=True)
            rows = [(f'every bus off this array, solved again in ONE model: {S.dst} with {", ".join(jn.get("with") or [])}',
                     TEXT)] + [(ln, DIM) for ln in S.joint_lines] + [
                (f'this bus: {v_own} layer changes alone, {v_cp} coupled; {moved_} crossings and {chm_} lanes\' changes '
                 f'moved', ACCENT if t >= 0.3 else TEXT),
                (f'{pl(nbr, "coupling")} of this bus\'s broken, each at its price', DIM)]
            y = box[1] + 46
            for ln, col in rows:
                for w_ in C.wrap(ln, 14, box[2] - box[0] - 32):
                    if y < box[3] - 18:
                        C.text(box[0] + 16, y, w_, 14, mix(PANEL, col, a))
                    y += 19
                y += 4
        # (where the film has shown the group's coupled solve just before -- a negotiation -- this scene is its plan for
        # this bus alone)
        sc.append(Scene('coupled', 'solve', 10, coupled, [
            (0, f'The coupled solve\'s plan for {S.dst}, on the board and in the braid, from the plan it solved alone.'
             if film.neg else
             f'THE COUPLED SOLVE. The buses off one array each solved their first plan alone; then every bus\'s model '
             f'is built again into ONE, coupled wherever two buses\' lanes cross or pass near: on different layers '
             f'there, or the coupling broken at a price.'),
            (0.3, f'Its plan for {S.dst}: {v_cp} layer changes, where alone it had {v_own}, {moved_} crossings moved. This '
                  f'plan, not its own, is the one its loop lays out.')],
            says=[(0, 'Here is the coupled plan for this bus, moving from the plan it solved alone.' if film.neg else
                      'Then the coupled solve: every bus off this array is solved again together, so that where their lanes '
                      'cross, they stand on different layers.'),
                  (0.3, f'For this bus, {v_cp:,} layer changes, where alone it had {v_own:,}. Its loop starts from this '
                        f'plan.')],
            cues=[(0.02, 'swell', 0.4), (0.3, 'glide', 0.5, 0.7), (0.72, 'chord', 1.0)]))

    # ------------------------------------------------------------------ the loop's failed rounds (when any)
    if S.failed:
        # each failed round: its smooth plan and what its audit found, what went back (side flips to the geometry;
        # island and via cuts and history to the solve), and the solve again -- the braid from the round's solve to
        # the next round's
        fails = []
        for r in S.failed:
            i = r['i']
            ld_ = lambda f: json.load(open(os.path.join(S.loop, f))) if os.path.isfile(os.path.join(S.loop, f)) else {}
            nxt = S.rounds.get(i + 1)
            gi, pi_ = ld_(f'g{i}.json'), ld_(f'p{i}.json')
            flips = sorted({tuple(x) for x in pi_.get('flips', [])} - {tuple(x) for x in gi.get('flips', [])})
            cuts, hot, slog = [], [], ''
            if nxt and re.match(r's\d+\.json$', os.path.basename(nxt['solve'])):
                j = re.match(r's(\d+)\.json', os.path.basename(nxt['solve'])).group(1)
                cj = ld_(f'cuts{j}.json')
                cuts = [(c['lane'], c['u_lo'], c['u_hi'], 'island') for c in cj.get('cuts', [])] + \
                       [(c['lane'], c['u'] - c['w'], c['u'] + c['w'], 'via') for c in cj.get('vcuts', [])]
                slog = _read(os.path.join(S.loop, f's{j}.log'))
            for f in (f'hp{i}.json', f'hq{i}.json', f'hs{i}.json'):
                hot += [film.mp.pt(h[0], h[1]) for h in ld_(f).get('hot', [])]
            finds = []
            for line in _read(os.path.join(S.loop, f'p{i}.audit')).splitlines():
                m = re.search(r'at \((-?\d+\.\d+),\s*(-?\d+\.\d+)\)', line) or re.search(r'^DIVE \S+\s+\(\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)\)', line)
                if m and re.match(r'^(PITCH|STATIC|DIVE|SHAPE) [A-Z]', line):
                    finds.append(film.mp.pt(float(m.group(1)), float(m.group(2))))
            A_ = json.load(open(r['solve']))
            B_ = json.load(open(nxt['solve'])) if nxt else A_
            pa, pb = (Plan(J_['cross'], J_['changes'], film, J_.get('layers')) for J_ in (A_, B_))
            moved = sum(1 for k in pa.cross if abs(pa.cross[k] - pb.cross.get(k, pa.cross[k])) > 1e-6)
            chm = sum(1 for n in film.M if pa.changes.get(n, []) != pb.changes.get(n, []))
            st_ = re.search(r'\[(OPTIMAL|FEASIBLE)\] (\d+)s(?: \([^)]*\))? vias (\d+)', slog)
            fails.append(dict(i=i, flips=flips, cuts=cuts, hot=hot, finds=finds, pa=pa, pb=pb, moved=moved, chm=chm,
                              solved=st_.groups() if st_ else None, nhist=len(hot),
                              va=sum(len(v) for v in pa.changes.values()), vb=sum(len(v) for v in pb.changes.values())))

        def loop_back(C, t, T_):
            draw_fanout(C, 0.5)
            nf = len(fails)
            j = min(nf - 1, int(t * nf))
            F_ = fails[j]
            loc = t * nf - j
            key = f'fail{F_["i"]}'
            flipped = {n for n, _isl in F_['flips']}
            C.lanes(film.st[key], alpha=ease(span(loc, 0, 0.1)), w=2.0, hl=flipped)
            C.vias(film.vias[key])
            a_f = ease(span(loc, 0.08, 0.16))
            for x, y in F_['finds']:
                C.wring(x, y, 0.32, mix(BODY, (255, 0, 255), a_f), 2.5)
            for isl in Gt['islands']:
                if any(isl['label'] == il for _n, il in F_['flips']):
                    x0, y0 = film.mp.pt(isl['box'][0], isl['box'][1])
                    x1, y1 = film.mp.pt(isl['box'][2], isl['box'][3])
                    C.d.rectangle([film.X(min(x0, x1) - 0.1, min(y0, y1) - 0.1), film.X(max(x0, x1) + 0.1, max(y0, y1) + 0.1)],
                                  outline=mix(BODY, (150, 245, 120), a_f), width=2 * SS)
            a_h = ease(span(loc, 0.35, 0.45))
            for x, y in F_['hot']:
                C.wring(x, y, 0.62, mix(BODY, (255, 150, 40), a_h), 2)
            m = ease(span(loc, 0.6, 0.85))
            cur = lerp_plan(F_['pa'], F_['pb'], m) if m > 0 else F_['pa']
            draw_braid(C, film, cur, alpha=1.0, cuts=F_['cuts'] if a_h > 0 else None, hl=flipped if loc < 0.35 else None)
            box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 470, BOARD[1] + 250)
            C.card(box)
            x0, y0 = box[0] + 16, box[1] + 14
            C.text(x0, y0, f'loop round {F_["i"]}: not yet', 20, ACCENT, bold=True)
            lines = [('the smooth plan\'s audit found', TEXT, 0.08), (f'   {len(F_["finds"])} place(s) short (magenta)', (255, 120, 255), 0.08),
                     ('the polish found no room on a side:', TEXT, 0.16),
                     (f'   {len(F_["flips"])} side flip(s) -> the geometry', (150, 245, 120), 0.16),
                     ('back to the solve:', TEXT, 0.35),
                     (f'   {sum(1 for c in F_["cuts"] if c[3] == "island")} island cut(s), '
                      f'{sum(1 for c in F_["cuts"] if c[3] == "via")} via cut(s) (boxes, below)', (255, 150, 60), 0.35),
                     (f'   {F_["nhist"]} history place(s) priced (orange)', (255, 150, 40), 0.4)]
            if F_['solved']:
                lines += [(f'the solve again ({F_["solved"][1]} s): {F_["moved"]} crossings', ACCENT, 0.6),
                          (f'   and {F_["chm"]} lanes\' changes moved, {F_["solved"][2]} vias', ACCENT, 0.6)]
            for k_, (l_, col, a0) in enumerate(lines):
                C.text(x0, y0 + 34 + k_ * 21, l_, 14, mix(PANEL, col, ease(span(loc, a0, a0 + 0.05))))
        f0 = fails[0]
        sc.append(Scene('loop', 'solve', 12 * len(fails), loop_back, [
            (0, f'Loop round {f0["i"]} did not pass. Its smooth plan\'s audit found {len(f0["finds"])} place(s) short '
                f'(magenta), and the polish found {len(f0["flips"])} lane(s) with no room on their side of an island (green).'),
            (0.35 / len(fails), 'What goes back: the side flips to the geometry; to the solve, the islands and vias the geometry '
                                'could not give room (cuts: no crossing or change of that lane in those spans) and the places '
                                'found short, priced (history).'),
            (0.6 / len(fails), 'The solve again, warm' + (', floored by the root\'s proof' if T['final'].get('proved', True) else '')
                               + (f': the same number of vias ({f0["vb"]})' if f0['va'] == f0['vb'] else
                                  f': {f0["vb"]} vias where it had {f0["va"]}')
                               + ', the crossings and changes moved out of the places that were short -- and the geometry '
                                 'again, with the flips.')],
            says=[(0, f'Loop round {f0["i"]} does not pass: its audit finds {pl(len(f0["finds"]), "place")} short.'),
                  (0.35 / len(fails), 'What it found goes back: side flips to the geometry, and cuts and history to the '
                                      'solve.'),
                  (0.6 / len(fails), 'The solve runs again, warm, and moves its crossings out of the places that were '
                                     'short.')],
            # (each failed round in turn: its card, the places short, the side flips, the cuts and the history, the
            # braid moving to the next round's solve)
            cues=[c for j, F_ in enumerate(fails) for c in
                  ([(j / len(fails), 'mark', 0.4)] if j else [])
                  + ([((j + 0.08) / len(fails), 'alert', 0.5)] if F_['finds'] else [])
                  + ([((j + 0.16) / len(fails), 'shimmer', 0.6)] if F_['flips'] else [])
                  + ([((j + 0.35) / len(fails), 'mark', 0.6)] if F_['cuts'] or F_['hot'] else [])
                  + [((j + 0.6) / len(fails), 'glide', 0.5, (j + 0.85) / len(fails))]]))

    # ------------------------------------------------------------------ the other buses finish (a negotiated group)
    n_peers = None
    if film.neg:
        neg_scene_others(film, sc)
        n_peers = len(sc)

    # ------------------------------------------------------------------ the geometry
    def forces(C, marks, a, wmax=5.0):
        for r, how, pts in sorted(marks, key=lambda m: m[0]['price']):
            col = mix(BODY, FORCE.get(r['kind'], (255, 255, 255)) if r['paid'] <= 1e-4 else (255, 40, 40), a)
            w = min(wmax, 1.2 + 1.3 * math.log10(1.0 + r['price']))
            if how == 'strut':
                C.wline(pts, col, w)
            elif how == 'wall':
                # the lane stands at a wall (a pad box, the board edge, an island's side): a stub toward it, the wall
                nv = np.array(film.mp.vec(*r['nv'])) * r['wall']
                p0 = np.array(pts[0])
                q = p0 + nv * 0.22
                t_ = np.array([-nv[1], nv[0]]) * 0.12
                C.wline([tuple(p0), tuple(q)], col, w)
                C.wline([tuple(q - t_), tuple(q + t_)], col, w)
            else:
                C.wring(*pts[0], 0.07 + 0.015 * w, col, 1.4)

    def cross_section(C, u, st_key, pass_i, a):
        """the LP column at u on the trunk: its variables (the present lanes' offsets), its priced rows"""
        k = int(round(u / Gt['G']))
        L = Gt['passes'][pass_i]['lanes']
        pres = [(n, v) for n in film.M for (f, kk, v) in L[n] if f == 'T' and kk == k]
        if not pres:
            return
        u_, b, nv = film.cols[('T', k)]
        box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 330, BOARD[3] - 6)
        C.card(box)
        C.text(box[0] + 14, box[1] + 12, f'one LP column: u = {u_:.2f} mm', 16, mix(PANEL, TEXT, a), bold=True)
        C.text(box[0] + 14, box[1] + 34, f'{len(pres)} variables: these lanes\' offsets', 13, mix(PANEL, DIM, a))
        os_ = [v for _n, v in pres]
        lo, hi = min(os_) - 0.3, max(os_) + 0.3
        y0, y1 = box[1] + 84, box[3] - 40
        sgn = -1 if nv[1] > 0 else 1                # north up
        oy = lambda v: y0 + (hi - v) / (hi - lo) * (y1 - y0) if sgn > 0 else y0 + (v - lo) / (hi - lo) * (y1 - y0)
        # the two layers side by side: the order binds a layer's OWN lanes only, so an F lane and a B lane may stand
        # at one offset (F over B, as a human stacks them); same-layer neighbours stand a bar apart
        lay = {n: film.final.layer(n, u_) for n, _v in pres}
        if len(film.LN) <= 2:
            xL = {0: box[0] + 130, 1: box[0] + 196}
        else:
            # (more layers than two: a column for each layer a lane stands on here, in the board's stack order)
            ks_ = [film.LN.index(L_) for L_ in stack_order([film.LN[v_] for v_ in set(lay.values())])]
            xL = {L_: box[0] + (165 if len(ks_) == 1 else 110 + j_ * 110 / (len(ks_) - 1)) for j_, L_ in enumerate(ks_)}
        first, last = min(xL, key=xL.get), max(xL, key=xL.get)
        for L_, x_ in xL.items():
            C.text(x_, box[1] + 58, film.LN[L_] if len(xL) <= 2 else film.LN[L_].split('.')[0], 13,
                   mix(PANEL, film.col[L_], a), bold=True, anchor='mm')
        for x_ in xL.values():
            C.line([(x_, y0 - 6), (x_, y1 + 6)], mix(PANEL, (70, 76, 88), a), 1)
        at = {n: v for n, v in pres}
        rows = [r for r in Gt['passes'][pass_i]['rows'] if r['f'] == 'T' and r['k'] == k]
        for r in rows:
            col = mix(PANEL, FORCE.get(r['kind'], TEXT), a)
            if r['kind'] in ('pitch', 'via', 'viavia') and len(r['lanes']) == 2 and all(n in at for n in r['lanes']):
                na, nb = r['lanes']
                w = min(5, 1.2 + 1.3 * math.log10(1 + r['price']))
                if r['kind'] == 'pitch':
                    xx = xL[lay[na]] + (8 if lay[na] == first else -8)
                    C.line([(xx, oy(at[na])), (xx, oy(at[nb]))], col, w)
                else:
                    C.line([(xL[lay[na]], oy(at[na])), (xL[lay[nb]], oy(at[nb]))], col, w)
        prs_ = set(S.pairs)
        for L_ in xL:
            ls = sorted([(oy(v), n) for n, v in pres if lay[n] == L_])
            # the labels a line apart at least, each led to its dot
            ys = [y for y, _n in ls]
            for j in range(1, len(ys)):
                ys[j] = max(ys[j], ys[j - 1] + 13)
            over = ys[-1] - (y1 + 10) if ys else 0
            if over > 0:
                ys = [y - over for y in ys]
                for j in range(len(ys) - 2, -1, -1):
                    ys[j] = min(ys[j], ys[j + 1] - 13)
            for (y, n), yl in zip(ls, ys):
                x_ = xL[L_]
                C.d.ellipse([(x_ - 5) * SS, (y - 5) * SS, (x_ + 5) * SS, (y + 5) * SS], fill=mix(PANEL, film.col[L_], a))
                if L_ not in (first, last):
                    continue                # (a middle column's lanes: their dots, the names beside the outer two)
                xt = x_ - 14 if L_ == first else x_ + 14
                C.line([(x_ + (-6 if L_ == first else 6), y), (xt, yl)], mix(PANEL, (90, 96, 108), a), 1)
                C.text(xt - (2 if L_ == first else -2), yl, n, 12, mix(PANEL, PAIR_COL if n in prs_ else TEXT, a),
                       anchor='rm' if L_ == first else 'lm')
        C.text(box[0] + 14, box[3] - 30, 'magenta: at the bar   cyan: a via\'s room', 11, mix(PANEL, DIM, a))
        # the column on the board
        ends_ = [(b[0] + (lo - 0.2) * nv[0], b[1] + (lo - 0.2) * nv[1]), (b[0] + (hi + 0.2) * nv[0], b[1] + (hi + 0.2) * nv[1])]
        C.wline(ends_, mix(BODY, ACCENT, a), 1.2)

    # up close: the window of the trunk where the most rules bind (both passes), in the force magnifier's aspect
    fbox = (BOARD[2] - 450, BOARD[1] + 6, BOARD[2] - 6, BOARD[3] - 6)
    fw = 2.6
    fh = fw * (fbox[3] - fbox[1] - 32) / (fbox[2] - fbox[0] - 12)
    fpts = [film.mp.pt(*r['pts'][0]) for P_ in (P1, P2) for r in P_['rows']
            if r['kind'] in ('pitch', 'via', 'viavia', 'static', 'bound')]
    fwin = None
    if fpts:
        c_ = max(fpts, key=lambda p: sum(1 for q in fpts if abs(q[0] - p[0]) < fw / 2 and abs(q[1] - p[1]) < fh / 2))
        near = [q for q in fpts if abs(q[0] - c_[0]) < fw / 2 and abs(q[1] - c_[1]) < fh / 2]
        mx, my = sum(q[0] for q in near) / len(near), sum(q[1] for q in near) / len(near)
        fwin = (mx - fw / 2, my - fh / 2, mx + fw / 2, my + fh / 2)

    def force_zoom(C, st, vs, marks, a):
        """the window up close: every lane at its copper's width, the vias at theirs, and the rules that bind"""
        TW_, PP_ = film.rules['TW'], film.rules['PP']
        prs = set(S.pairs)
        top = sorted(marks, key=lambda m: -m[0]['price'])

        def draw(d, Z, s):
            v0, v1 = (fwin[0] - 1, fwin[1] - 1), (fwin[2] + 1, fwin[3] + 1)
            for layer in film.order:
                for n, (P, bits) in st.items():
                    if not ((P[:, 0] > v0[0]) & (P[:, 0] < v1[0]) & (P[:, 1] > v0[1]) & (P[:, 1] < v1[1])).any():
                        continue
                    wd = (PP_ + TW_ if n in prs else TW_) * s
                    col = mix(BODY, film.col[layer], 0.55 if n in prs else 0.8)
                    j0 = None
                    for j in range(len(bits) + 1):
                        on = j < len(bits) and bits[j] == layer
                        if on and j0 is None:
                            j0 = j
                        elif not on and j0 is not None:
                            Q = simplify(P[j0:j + 1], 0.3 / s * SS)
                            d.line([Z(*q) for q in Q], fill=col, width=max(1, int(wd)), joint='curve')
                            j0 = None
            for n, x, y, al in vs:
                cx, cy = Z(x, y)
                r = film.rules['via'] / 2 * s
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(185, 185, 185), outline=(240, 240, 240), width=SS)
            lab = 0
            for r, how, pts in top:
                col = FORCE.get(r['kind'], TEXT) if r['paid'] <= 1e-4 else (255, 40, 40)
                w = SS * min(7.0, 1.5 + 1.8 * math.log10(1.0 + r['price']))
                if not any(fwin[0] - 0.3 < p[0] < fwin[2] + 0.3 and fwin[1] - 0.3 < p[1] < fwin[3] + 0.3 for p in pts):
                    continue
                if how == 'strut':
                    d.line([Z(*pts[0]), Z(*pts[1])], fill=col, width=int(w))
                    for q in pts:
                        cx, cy = Z(*q)
                        d.ellipse([cx - w, cy - w, cx + w, cy + w], fill=col)
                    at = ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
                elif how == 'wall':
                    nv = np.array(film.mp.vec(*r['nv'])) * r['wall']
                    p0 = np.array(pts[0])
                    q = p0 + nv * (TW_ / 2 + film.rules['CL'])
                    tt = np.array([-nv[1], nv[0]]) * 0.09
                    d.line([Z(*p0), Z(*q)], fill=col, width=int(w))
                    d.line([Z(*(q - tt)), Z(*(q + tt))], fill=col, width=int(w))
                    at = tuple(q)
                else:
                    cx, cy = Z(*pts[0])
                    R = 0.05 * s + w
                    d.ellipse([cx - R, cy - R, cx + R, cy + R], outline=col, width=int(w * 0.7))
                    at = pts[0]
                if lab < 6 and fwin[0] < at[0] < fwin[2] and fwin[1] < at[1] < fwin[3]:
                    cx, cy = Z(*at)
                    d.text((cx + 8, cy - 8), f'{r["price"]:.3g}', font=font(12 * SS, True), fill=col, anchor='lb')
                    lab += 1
        magnify(C, film, fbox, fwin, draw, 'up close: the rules that bind, and their prices')

    def geometry(C, t, T_):
        draw_fanout(C, 0.5)
        m1 = ease(span(t, 0.02, 0.16))
        m2 = ease(span(t, 0.72, 0.84))
        if t < 0.72:
            st = morph(film.st['slots'], film.st['p1'], m1)
            vs = morph_vias(film.vias['slots'], film.vias['p1'], m1)
            rows = P1['rows']
        else:
            st = morph(film.st['p1'], film.st['p2'], m2)
            vs = morph_vias(film.vias['p1'], film.vias['p2'], m2)
            rows = P2['rows']
        # the islands (another part's pads a lane must pass on one side) in pass 2
        a_i = ease(span(t, 0.68, 0.74))
        if a_i > 0:
            for isl in Gt['islands']:
                if isl['label'].startswith(('end ', 'tooth ', 'stub ', 'svia ')):
                    continue
                x0, y0 = film.mp.pt(isl['box'][0], isl['box'][1])
                x1, y1 = film.mp.pt(isl['box'][2], isl['box'][3])
                P = [film.X(min(x0, x1) - 0.08, min(y0, y1) - 0.08), film.X(max(x0, x1) + 0.08, max(y0, y1) + 0.08)]
                C.d.rectangle([P[0], P[1]], outline=mix(BODY, (255, 160, 40), a_i), width=int(1.5 * SS))
        C.lanes(st, w=2.2)
        C.vias([(n, x, y) for n, x, y, al in vs])
        a_f = ease(span(t, 0.2, 0.26)) * (1 - ease(span(t, 0.68, 0.72))) + ease(span(t, 0.86, 0.92))
        if a_f > 0:
            marks = force_marks(film, rows, st, vs)
            forces(C, marks, a_f)
            if a_f > 0.05 and fwin:
                force_zoom(C, st, vs, marks, a_f)
        # the cursor column
        cur = None
        if 0.42 <= t < 0.68:
            cur = T['BAND'] + (Gt['H0'] - T['BAND'] - 0.2) * ease(span(t, 0.43, 0.67))
            cross_section(C, cur, 'p1', 0, ease(span(t, 0.42, 0.45)) * (1 - ease(span(t, 0.66, 0.68))))
        draw_braid(C, film, film.final, alpha=0.9, cursor=cur)
        # the LP card
        if t < 0.42 or t >= 0.68:
            P_ = P1 if t < 0.72 else P2
            lp = [ln for ln in Gt['log'] if 'joint LP' in ln]
            ln = lp[0 if t < 0.72 else -1] if lp else ''
            m = re.search(r'build (\d+)s solve (\d+)s', ln)
            box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 400, BOARD[1] + 250)
            C.card(box)
            x0, y0 = box[0] + 16, box[1] + 14
            C.text(x0, y0, f'the LP, pass {1 if t < 0.72 else 2}', 20, ACCENT, bold=True)
            lines = [f'{P_["nvar"]} variables: an offset per lane', f'  per column (every {Gt["G"]:.2f} mm)',
                     f'{P_["nrow"]} rows (inequalities)', 'solved as its dual by HiGHS' + (f' in {m.group(2)} s' if m else ''),
                     f'{len(P_["rows"])} rules with a shadow price:']
            cnt = collections.Counter(r['kind'] for r in P_['rows'])
            name = {'pitch': 'neighbours at the bar', 'via': 'a via\'s room', 'viavia': 'via to via', 'bound': 'pad box / edge',
                    'static': 'an island\'s side', 'turn': 'turn limit', 'pturn': 'a pair\'s turn', 'pdive': 'a pair\'s dive run',
                    'slope': 'slope cap', 'approach': 'approach to its end'}
            for j, l_ in enumerate(lines):
                C.text(x0, y0 + 32 + j * 19, l_, 14, TEXT if j < 4 else DIM)
            yy = y0 + 32 + len(lines) * 19
            for kind, c_ in cnt.most_common(6):
                C.d.rectangle([(x0 + 6) * SS, (yy + 4) * SS, (x0 + 22) * SS, (yy + 12) * SS], fill=FORCE.get(kind, TEXT))
                C.text(x0 + 30, yy, f'{c_}  {name.get(kind, kind)}', 13, DIM)
                yy += 18
    paid2 = P2.get('paid', {})
    npaid = sum(paid2.values())
    cap_g = [(0, f'The geometry: ONE joint linear program turns the order into millimetres -- an offset for every '
                 f'lane at every column ({Gt["G"]:.2f} mm apart) along its frame, in the solve\'s order, on its '
                 f'solved layers.'),
             (0.16, f'Hard rules: same-layer neighbours a bar ({Gt["rules"]["P_MIN"]:.3f} mm) apart, a via\'s room '
                    f'round every change, off the pad boxes. Soft: the length a move adds, bends, and a comfortable '
                    f'pitch ({Gt["rules"]["P_COMF"]:.3f} mm) wherever there is room.'),
             (0.22, 'WHY each lane is where it is: the LP\'s dual gives every rule a shadow price. The lit rules are '
                    'the ones holding a lane in place -- thicker pushes harder. Every other rule has room to spare.'),
             (0.42, 'One column of the LP (the cursor): its variables are the offsets of the lanes there -- in the '
                    'braid\'s order at the same u below, now in millimetres, same-layer neighbours held a bar apart.'),
             (0.68, 'Pass 2 holds each lane to ONE side of every island (another part\'s pads, orange): one split per '
                    'island, in the lane order, the side with the room.'),
             (0.86, (f'What the LP had to pay (red) would go back to the solve as cuts: {npaid} rule(s) paid here'
                     + (', none of them a cut.' if not (S.geo.get('cuts') or S.geo.get('vcuts')) else '.'))
              if npaid else 'Nothing paid: every hard rule met, no cut for the solve.')]
    if not S.Et:
        sc.append(Scene('geometry', 'geometry', 30, geometry, cap_g, says=[
            (0, 'The geometry: one linear program turns the order into millimetres, an offset for every lane at every '
                'column.'),
            (0.22, 'Each rule gets a shadow price: the lit rules are the ones holding a lane in place, and the thicker '
                   'push harder.'),
            (0.42, 'The cursor is one column of the program: the lanes there, in millimetres, in the braid\'s order.'),
            (0.68, 'Pass two holds each lane to one side of every island of another part\'s pads.'),
            (0.86, 'What the program had to pay, in red, would go back to the solve.' if npaid else
                   'Nothing is paid: every hard rule is met.')],
            # (pass 1's layout; its priced rules; the cursor along the trunk; the islands; pass 2; its rules again)
            cues=[(0.02, 'glide', 0.4, 0.16), (0.2, 'shimmer', 0.5), (0.43, 'sweep', 0.5, 0.67), (0.68, 'mark', 0.5),
                  (0.72, 'glide', 0.6, 0.84), (0.86, 'alert' if npaid else 'shimmer', 0.5)]))
    else:
        embed_scenes(film, sc, draw_fanout, forces, P2)

    # ------------------------------------------------------------------ the polish
    # the polish's largest move: each polished vertex's distance from the geometry's line (its dense samples)
    dmax = 0.0
    gk = 'emb' if S.Et else 'geo'            # the round's geometry: the 2-D geometry's lines where its loop ran one
    if gk in film.st and S.pol:
        for n in film.M:
            if n in film.st[gk] and n in film.raw.get('pol', {}):
                G_ = film.st[gk][n][0]
                for q in film.raw['pol'][n][0]:
                    dmax = max(dmax, float(np.sqrt(((G_ - np.asarray(q)) ** 2).sum(axis=1).min())))

    def polish(C, t, T_):
        draw_fanout(C, 0.5)
        m = ease(span(t, 0.1, 0.6))
        st = film.st[gk] if S.Et else morph(film.st['p2'], film.st['geo'], ease(span(t, 0.0, 0.1)))
        st = morph(st, film.st['pol'], m) if m > 0 else st
        C.lanes(st, w=2.2)
        C.vias([(n, x, y) for n, x, y, al in morph_vias(film.vias[gk], film.vias['pol'], m)])
        draw_braid(C, film, film.final, alpha=0.6)
        if S.gate.get('smooth'):
            box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 700, BOARD[1] + 60)
            C.card(box)
            C.text(box[0] + 14, box[1] + 12, 'audit of the smooth plan', 13, DIM)
            C.text(box[0] + 14, box[1] + 30, S.gate['smooth'][:88], 14, (150, 245, 120), mono=True)
    sc.append(Scene('polish', 'polish', 7, polish, [
        (0, f'The polish: the audit\'s own measures (pitch, via rooms, clearance to pads as KiCad draws them, turns) '
            f'met in board xy by small vertex moves, one LP per round -- here the largest move is {dmax:.3f} mm.'),
        (0.5, 'A lane with no room on its side of an island would be FLIPPED to the other and the geometry run '
              'again. The smooth plan passes the audit.')],
        says=[(0, 'The polish meets the audit\'s own measures with small moves.'),
              (0.5, 'The smooth plan passes the audit.')],
        cues=[(0.1, 'glide', 0.5, 0.6)]))                  # (the polished lines)

    # ------------------------------------------------------------------ the snap
    pairs_ = set(S.pairs)
    singles = set(film.M) - pairs_
    grid = Gt['rules']['grid']

    def zoom(C, box, view, t_state, show, title):
        """the snap up close: the router's grid, and the lanes of the states `show` [(state, width, alpha)]"""
        def draw(d, Z, s):
            gx = math.ceil(view_[0][0] / grid) * grid
            while gx < view_[0][2]:
                gy = math.ceil(view_[0][1] / grid) * grid
                while gy < view_[0][3]:
                    x, y = Z(gx, gy)
                    d.rectangle([x - 1, y - 1, x + 1, y + 1], fill=(78, 86, 98))
                    gy += grid
                gx += grid
            TW_, PP_ = film.rules['TW'], film.rules['PP']
            for key, wid, al in show:
                st = film.raw[key]
                J = {'pairs': S.pairsd, 'held': S.held, 'snap': S.snapd}.get(key)
                if J and al > 0.5:
                    for n in S.pairs:
                        if n not in t_state or n not in st:
                            continue
                        P, _bits, _U = st[n]
                        d.line([Z(*q) for q in simplify(P, 0.3 / s * SS)], fill=mix(PANEL, (120, 100, 28), al),
                               width=max(1, int((PP_ + TW_) * s)), joint='curve')
                        v = J['lanes'].get(n, {})
                        legs = []
                        for e in v.get('ends', []) or []:
                            legs += [(leg, e['layer']) for leg in e.get('legs', [])]
                        c_ = v.get('cross')
                        if c_:
                            for side in ('P', 'N'):
                                legs += [(pts_, L_) for pts_, L_ in c_['legs'][side]]
                        for pts_, L_ in legs:
                            col = film.col[film.lidx(film.mp.layer(L_))]
                            d.line([Z(*film.mp.pt(*q)) for q in pts_], fill=mix(PANEL, col, al), width=max(1, int(TW_ * s)),
                                   joint='curve')
                        if c_:
                            for x_, y_, _side in c_['vias']:
                                cx, cy = Z(*film.mp.pt(x_, y_))
                                r = film.rules['via'] / 2 * s
                                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(185, 185, 185), outline=(240, 240, 240), width=SS)
                for layer in film.order:
                    for n, (P, bits, _U) in st.items():
                        if n not in t_state:
                            continue
                        for j, bt in enumerate(bits):
                            if bt == layer:
                                d.line([Z(*P[j]), Z(*P[j + 1])], fill=mix(PANEL, film.col[layer], al),
                                       width=max(1, int(wid * SS)))
                for n, x, y in film.vias[key]:
                    if n in t_state:
                        cx, cy = Z(x, y)
                        r = film.rules['via'] / 2 * s
                        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=mix(PANEL, VIA_COL, al), width=2 * SS)
        view_ = [magnify_view(box, view, True)]
        magnify(C, film, box, view, draw, title)

    # where to look closely: the crossover (an opposite-hands pair), else the first pair's dive, else a via
    focus = None
    for n in S.pairs:
        c = S.snapd['lanes'].get(n, {}).get('cross') if S.snapd else None
        if c:
            # the crossover's whole shape: its legs and its barrels
            q = [pt for side in ('P', 'N') for pts_, _L in c['legs'][side] for pt in pts_] + [v[:2] for v in c['vias']]
            xs, ys = [pt[0] for pt in q], [pt[1] for pt in q]
            focus = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, n)
            break
    if focus is None and S.snapd and S.snapd.get('vias'):
        n, x, y = S.snapd['vias'][0]
        focus = (x, y, n)
    if focus:
        focus = film.mp.pt(focus[0], focus[1]) + (focus[2],)

    def snap(C, t, T_):
        draw_fanout(C, 0.5)
        mp_ = ease(span(t, 0.05, 0.28))
        mq = ease(span(t, 0.4, 0.55))
        ms = ease(span(t, 0.62, 0.85))
        pr = morph(film.st['pol'], film.st['pairs'], mp_)
        si = morph(film.st['pol'], film.st['held'], mq)
        si = morph(si, film.st['snap'], ms) if ms > 0 else si
        pr = morph(pr, film.st['snap'], ms) if ms > 0 else pr
        st = {n: (pr[n] if n in pairs_ else si[n]) for n in film.M if n in pr and n in si}
        dim = 0.25 if t < 0.36 else 1.0
        C.lanes(st, w=2.2, only=pairs_ if t < 0.36 else None, dim=dim, hl=pairs_ if t < 0.36 else None, ribbon=t >= 0.12)
        vs = film.vias['snap'] if t >= 0.62 else film.vias['pairs'] if t >= 0.2 else film.vias['pol']
        C.vias(vs, only=pairs_ if t < 0.36 else None, dim=dim)
        if focus:
            v = (focus[0] - 1.25, focus[1] - 0.5, focus[0] + 1.25, focus[1] + 0.5)
            box = (BRAID[0], BRAID[1], BRAID[0] + 620, BRAID[3])
            if t < 0.36:
                zoom(C, box, v, pairs_, [('pol', 1.2, 0.35), ('pairs', 2.4, 1.0)],
                     f'{focus[2]}: laid as the pair router moves (the dots: its {grid} mm grid)')
            else:
                zoom(C, box, v, set(film.M), [('held', 1.2, 0.35), ('snap', 2.4, 1.0)] if t >= 0.62 else
                     [('pol', 1.2, 0.35), ('held', 2.2, 1.0)], 'the lanes: smooth (faint) and on the router\'s grid')
            # a second magnifier: a single's via
            sv = [(n, x, y) for n, x, y in film.vias['snap'] if n in singles]
            if sv:
                n, x, y = sv[len(sv) // 2]
                v2 = (x - 1.25, y - 0.5, x + 1.25, y + 0.5)
                box2 = (BRAID[0] + 640, BRAID[1], BRAID[0] + 1260, BRAID[3])
                zoom(C, box2, v2, set(film.M), [('pol', 1.2, 0.35), ('snap' if t >= 0.62 else 'held', 2.4, 1.0)],
                     f'{n}: its via, and its neighbours')
            box3 = (BRAID[0] + 1280, BRAID[1], BRAID[2], BRAID[3])
            C.card(box3, alpha=0.95)
            lines = [('pairs laid first', S.gate.get('pairs held', '')), ('snapped', S.gate.get('snapped', ''))]
            yy = box3[1] + 12
            for lab, g in lines:
                C.text(box3[0] + 12, yy, lab, 13, DIM)
                for w_ in C.wrap(g, 12, box3[2] - box3[0] - 24)[:3]:
                    yy += 18
                    C.text(box3[0] + 12, yy, w_, 12, (150, 245, 120))
                yy += 28
            if S.lint:
                C.text(box3[0] + 12, yy, S.lint[-1], 14, (150, 245, 120), bold=True)
    xo_txt = f' {", ".join(xo)} swap{"s" if len(xo) == 1 else ""} its legs at its dive (a crossover), as a designer does.' if xo else ''
    sc.append(Scene('snap', 'snap', 20, snap, [
        (0, 'The snap makes the smooth plan octilinear on the router\'s grid. The PAIRS go first and alone, moving as '
            'the pair router does: 45-degree turns, a turning radius of straight after each, a dive only on a '
            'straight run.' + xo_txt),
        (0.36, 'The pairs are then HELD while the polish fits the singles round them...'),
        (0.6, f'...and every single is laid on the router\'s {grid} mm grid: a grid search in a band round its smooth '
              f'line, starting and ending where the router will.')],
        says=([(0, 'The snap puts the plan on the router\'s grid. The differential pairs go first, with forty-five degree '
                   'turns.' + (f' {" and ".join(spoken(n) for n in xo)} '
                               f'{"swaps its legs" if len(xo) == 1 else "swap their legs"} at a crossover.' if xo else '')),
               (0.36, 'Then the pairs are held, while the singles are fitted round them.')] if pairs_ else
              [(0, 'The snap puts the plan on the router\'s grid, with forty-five degree turns.')])
        + [(0.6, 'Finally, every single lane is snapped onto the grid.')],
        cues=[(0.05, 'snap', 0.4), (0.4, 'snap', 0.6), (0.62, 'snap', 0.8)]))   # (the pairs; the singles held; snapped)

    # ------------------------------------------------------------------ the audit
    def audit(C, t, T_):
        draw_fanout(C, 0.5)
        C.lanes(film.st['snap'], w=2.2, ribbon=True)
        C.vias(film.vias['snap'])
        box = (W / 2 - 520, BOARD[1] + 150, W / 2 + 520, BOARD[1] + 450)
        a = ease(span(t, 0.05, 0.2))
        C.card(box, alpha=0.9 * a)
        C.text(W / 2, box[1] + 26, 'the audit, the gate and the lint', 24, mix(PANEL, ACCENT, a), bold=True, anchor='ma')
        rows = [('smooth plan', S.gate.get('smooth', '')), ('pairs held', S.gate.get('pairs held', '')),
                ('snapped plan', S.gate.get('snapped', '')), ('lint', S.lint[-1] if S.lint else '')]
        for j, (lab, g) in enumerate(rows):
            aj = ease(span(t, 0.15 + 0.12 * j, 0.25 + 0.12 * j))
            y = box[1] + 80 + j * 48
            C.text(box[0] + 30, y, lab, 18, mix(PANEL, DIM, aj))
            C.text(box[0] + 200, y, g[:84], 15, mix(PANEL, (150, 245, 120), aj), mono=True)
        C.text(W / 2, box[3] - 40, f'loop round {S.lr}: the plan passes -- every check, nothing waived', 18,
               mix(PANEL, TEXT, ease(span(t, 0.7, 0.8))), anchor='ma')
    sc.append(Scene('audit', 'audit', 7, audit, [
        (0, 'The audit installs the plan exactly as the router will get it and runs every check; the gate passes it '
            'only when complete and clean with nothing waived; the lint checks what the snap promised.'),
        (0.55, 'A failing round sends its findings back -- side flips to the geometry, cuts and history (the places '
               'it was short, priced) to the solve, among them a cut for a via the geometry moved across another lane '
               '-- and the loop runs again; ' + (
                   'findings that stand at the ENDS stop it, and go back to the group\'s model with the next round of '
                   'the group.' if film.g else 'trouble at the ends goes back to the fanout.')
               + (f' Here the loop passed in round {S.lr}: nothing went back.' if not S.failed else ''))],
        says=[(0, 'The audit checks the plan exactly as the router will get it, and the gate passes it only when it is '
                  'complete and clean.'),
              (0.55, 'A failing round sends its findings back: side flips to the geometry, cuts and history to the solve. '
                     + ('Findings that stand at the ends would go back to the group\'s model. ' if film.g else '')
                     + (f'Here the loop passed in round {S.lr}, with nothing waived.' if not S.failed else
                        f'This loop passed in round {S.lr}.'))],
        # (the card; its four rows in turn; the plan passes)
        cues=[(0.05, 'swell', 0.4)] + [(0.15 + 0.12 * j, 'tick', 0.5 + 0.15 * j) for j in range(4)]
        + [(0.7, 'chord', 1.0)]))

    # ------------------------------------------------------------------ the route
    order = [n for n in S.route_order if n in film.routed] + [n for n in film.M if n not in S.route_order]

    def route(C, t, T_):
        C.segs(film.other_c[0], alpha=0.22)
        C.segs(film.fo_c[0], alpha=0.9)
        C.cvias(film.fo_c[1])
        C.lanes(film.st['snap'], alpha=0.28, w=1.6, ribbon=False)
        k_ = t / 0.8 * len(order)
        for j, n in enumerate(order):
            a = min(1.0, max(0.0, k_ - j))
            if a <= 0:
                break
            segs, vias = film.routed[n]
            C.segs(segs, alpha=a)
            C.cvias(vias, a)
        C.card(BRAID, alpha=0.95)
        shown = [ln for j, ln in enumerate(S.route_lines) if j < k_]
        C.text(BRAID[0] + 12, BRAID[1] + 8, 'route_lanes.py --plan  (the production router, each lane in its band, '
               'all at once)', 13, DIM)
        cols_ = 3 if len(S.route_lines) <= 30 else 4
        per = int(math.ceil(len(S.route_lines) / cols_))
        dy_ = min(22.0, (BRAID[3] - BRAID[1] - 40) / max(1, per))
        for j, ln in enumerate(shown):
            c_, r_ = divmod(j, per)
            p = ln.split()
            txt = f'{p[0]:<7} {"pair" if p[1] == "pair" else "single":<6} in band  vias {p[p.index("vias") + 1]}  len {p[p.index("len") + 1]} mm' if 'vias' in p and 'len' in p else ln[:60]
            C.text(BRAID[0] + 12 + c_ * (1860 / cols_), BRAID[1] + 32 + r_ * dy_, txt, 13 if dy_ >= 18 else 11, (150, 245, 120),
                   mono=True)
        if t > 0.85:
            a = ease(span(t, 0.85, 0.92))
            box = (W / 2 - 430, BOARD[1] + 10, W / 2 + 430, BOARD[1] + 70)
            C.card(box, alpha=0.9 * a)
            C.text(W / 2, box[1] + 30, f'{S.route_summary.replace("SUMMARY seq: ", "")}   ·   check_connected: '
                   f'{"all connected" if S.connected else "OPEN NETS"}   ·   check_drc: {"clean" if S.drc_clean else "VIOLATIONS"}',
                   18, mix(PANEL, (150, 245, 120) if S.connected and S.drc_clean else (255, 90, 90), a), bold=True, anchor='mm')
    sc.append(Scene('route', 'lay', 16, route, [
        (0, 'The lay: the production router lays the plan\'s copper, each lane held to its band -- the pairs first (their '
            'end connectors and crossover laid as planned), then the singles in the berths\' order.'),
        (0.45, 'The wall guard: another net\'s ball of the arrays the lay walls in has its whole way out kept between the lanes, '
               'the lanes walling it lifted and routed again round it -- '
               + (f'here {pl(len(S.walled), "ball")} (the lay\'s log).' if S.walled else
                  'here the lay walled in none (its log names none).')),
        (0.85, 'A lane the router reports routed is not proof its net connects: the board is checked -- connectivity '
               'and DRC, at the routed clearance.')],
        says=[(0, 'The lay: the production router lays the copper, each lane held to its band. '
                  + ('The pairs go first, then the singles.' if pairs_ else 'The lanes go in the berths\' order.')),
              (0.45, 'Where the lay walls in another net\'s ball, the walling lanes are lifted and routed again round '
                     'its way out. ' + (f'Here, {pl(len(S.walled), "ball")}.' if S.walled else 'Here the lay walled in '
                                                                                                 'none.')),
              (0.85, 'Then the board is checked, for connectivity and the design rules.')],
        # (each lane's copper as it comes in, rising along the order they are laid; the checks)
        cues=[(j * 0.8 / len(order), 'pluck', j / max(1, len(order) - 1)) for j, n in enumerate(order)
              if film.routed[n][0] or film.routed[n][1]]
        + [(0.85, 'chord' if S.connected and S.drc_clean else 'alert', 1.0)]))

    if film.neg:
        neg_scene_together(film, sc)

    # ------------------------------------------------------------------ the result
    m_ = re.search(r'(\d+/\d+) in band', S.route_summary)
    band_ = m_.group(1) if m_ else '?'
    def result(C, t, T_):
        a_h = ease(span(t, 0.35, 0.45)) * (1 - ease(span(t, 0.75, 0.85))) if film.human_c else 0.0
        C.segs(film.other_c[0], alpha=0.22)
        if a_h < 1:
            C.segs(film.seq_c[0], alpha=1 - a_h)
            C.cvias(film.seq_c[1], 1 - a_h)
        if a_h > 0:
            C.segs(film.human_c[0], alpha=a_h)
            C.cvias(film.human_c[1], a_h)
        C.card(BRAID, alpha=0.95)
        x0 = BRAID[0] + 40
        C.text(x0, BRAID[1] + 20, 'on these nets, over the whole board (every via and mm of their copper)', 15, DIM)
        C.text(x0, BRAID[1] + 60, f'the whole route:  {film.vias_ours} vias,  {film.mm_ours:.0f} mm', 30,
               mix(PANEL, TEXT, 1 - 0.6 * a_h), bold=True)
        if film.human_c:
            C.text(x0, BRAID[1] + 110, f'the human:        {film.vias_human} vias,  {film.mm_human:.0f} mm', 30,
                   mix(PANEL, TEXT, 0.4 + 0.6 * a_h), bold=True)
        C.text(x0 + 1000, BRAID[1] + 60, 'showing: ' + ('the human\'s board' if a_h > 0.5 else 'the whole route'), 18,
               ACCENT)
    sc.append(Scene('result', 'lay', 10, result, [
        (0, f'{band_} lanes routed in their bands, all at once; check_connected: '
            f'{"every net connected" if S.connected else "NETS OPEN"}; check_drc: {"clean" if S.drc_clean else "VIOLATIONS"}.' + (
            f' The whole route: {film.vias_ours} vias and {film.mm_ours:.0f} mm; the human\'s board on the same nets: '
            f'{film.vias_human} vias and {film.mm_human:.0f} mm (the human\'s copper includes its length-matching meanders).'
            if film.human_c else ''))],
        says=[(0, ('The lanes are routed' if '/' not in band_ else f'All {band_.split("/")[1]} lanes routed'
                   if band_.split('/')[0] == band_.split('/')[1] else f'{band_.replace("/", " of ")} lanes routed')
               + (' and connected' if S.connected else ', some nets open')
               + (', no design rule errors: ' if S.drc_clean else ', with design rule errors: ')
               + f'{film.vias_ours:,} vias, {film.mm_ours:,.0f} millimetres.')]
        + ([(0.35, f'The human\'s board: {film.vias_human:,} vias and {film.mm_human:,.0f} millimetres, meanders '
                   f'included.', 0.75)] if film.human_c else []),      # (said while the human's board is shown)
        cues=[(0.35, 'whoosh', 0.4), (0.75, 'whoosh', 0.6)] if film.human_c else []))   # (to the human's board, back)

    # ------------------------------------------------------------------ the escalation (a bus step's group): what ran
    esc_txt, esc_clean, esc_ge, esc_count = '', False, {}, 0
    if film.g:
        ge_ = esc_ge = S.G.group_end or {}
        dr_ = (S.G.summary.get('design_rules') or {})
        esc_count = dr_.get('count') or 0
        esc_clean = ge_.get('open') == 0 and ge_.get('drc') == 0 and ge_.get('walled') == 0
        esc_txt = (f'this run: the group\'s grade {ge_.get("open", "?")} open, {ge_.get("drc", "?")} violations, '
                   f'{ge_.get("walled", "?")} walled'
                   + (f', {dr_.get("count", 0)} net(s) delivered below the sizes asked for' if dr_ else '')
                   + (' -- clean at the sizes asked for: no rung was needed' if esc_clean and not dr_.get('count') else ''))
        next(s_ for s_ in sc if s_.key == 'result').captions.append(
            (0.5, 'The group\'s escalation: a group left short of clean is routed again a rung down the fab ladder. '
                  + esc_txt[0].upper() + esc_txt[1:] + '.'))
    # ------------------------------------------------------------------ the loop, summed up
    def coda(C, t, T_):
        C.d.rectangle([0, 0, W * SS, H * SS], fill=BG)
        a = ease(span(t, 0, 0.2))
        boxes = [('ends', 'fanout'), ('strings', 'pulled taut'), ('frame', 'trunk, rings'), ('solve', 'CP-SAT'),
                 ('geometry', 'cells, LP rounds' if S.Et else 'LP'), ('polish', 'LP rounds'), ('snap', 'grid search'),
                 ('audit', 'gate, lint'), ('lay', 'router')]
        if film.g:
            # (a bus step's: the group's ends first -- each bus captured, one model, each bus's check, the field)
            boxes = [('capture', 'each bus'), ('ends model', 'one CP-SAT'), ('check', 'whole solve'),
                     ('field', 'laid, measured')] + boxes[1:]
        nb_ = len(boxes)
        gap = 22 if nb_ <= 9 else 14
        bw = min(176, (W - 2 * 40 - (nb_ - 1) * gap) / nb_)
        x0, y0 = (W - nb_ * bw - (nb_ - 1) * gap) / 2, 420
        px_ = 24 if nb_ <= 9 else 19
        for j, (k, sub) in enumerate(boxes):
            x = x0 + j * (bw + gap)
            C.rect((x, y0, x + bw, y0 + 90), fill=PANEL, outline=mix(BG, ACCENT, a), width=2, r=10)
            C.text(x + bw / 2, y0 + 30, k, px_, mix(BG, TEXT, a), bold=True, anchor='mm')
            C.text(x + bw / 2, y0 + 62, sub, 15, mix(BG, DIM, a), anchor='mm')
            if j:
                C.line([(x - gap + 4, y0 + 45), (x - 6, y0 + 45)], mix(BG, DIM, a), 2)
        ix = {k: j for j, (k, _s) in enumerate(boxes)}

        # the arrows back
        def back(j0, j1, lab, dy):
            xa, xb = x0 + j0 * (bw + gap) + bw / 2, x0 + j1 * (bw + gap) + bw / 2
            C.line([(xa, y0 + 90), (xa, y0 + 90 + dy), (xb, y0 + 90 + dy), (xb, y0 + 96)], mix(BG, (255, 110, 60), a), 2)
            C.text((xa + xb) / 2, y0 + 96 + dy, lab, 16, mix(BG, (255, 110, 60), a), anchor='ma')
        back(ix['audit'], ix['solve'], 'cuts + history: the solve again', 50)
        back(ix['polish'], ix['geometry'], 'side flips', 20)
        if film.g:
            back(ix['check'], ix['ends model'], 'a refusal: a lesson', 20)
            back(ix['field'], ix['ends model'], 'a finding of the field: a lesson', 50)
            back(ix['audit'], ix['ends model'], 'ends crowded: the group\'s model again, the next round of the group', 120)
        else:
            back(ix['audit'], ix['ends'], 'ends crowded: the fanout again', 120)
        C.text(W / 2, 250, 'the whole route', 48, mix(BG, TEXT, a), bold=True, anchor='mm')
        C.text(W / 2, 320, 'each stage fed a MEASUREMENT of the one before -- a plan is routed only once the router\'s '
               'own rules say it can be', 22, mix(BG, DIM, a), anchor='mm')
        if film.g:
            C.text(W / 2, 800, 'a group left short of clean is routed again a rung down the fab ladder -- the lanes\' track, '
                   'the escapes\' track and the via together, the buses that failed alone', 20, mix(BG, DIM, a), anchor='mm')
            C.text(W / 2, 836, esc_txt, 20, mix(BG, (150, 245, 120) if esc_clean else (255, 170, 60), a), anchor='mm')
    sc.append(Scene('coda', None, 6, coda, [],
                    says=[(0, 'Each stage measures the one before, and a plan is routed only when the router\'s own rules '
                              'allow it.')] + ([
                        (0.3, 'A refusal of the check, or a finding of the field, teaches the group\'s model; and a '
                              'round of the group that leaves the board short of clean sends what its buses found at '
                              'their ends back to it.'),
                        (0.6, 'And a group left short of clean is routed again a rung down the fab ladder: the lanes\' '
                              'track, the escapes\' track and the via together; and where only some of its buses failed, those '
                              'buses alone are routed again. '
                              + ('This group was clean at the sizes asked for, so no rung was needed.'
                                 if esc_clean and not esc_count else
                                 f'This group\'s grade: {esc_ge.get("open", "unknown")} open, '
                                 f'{esc_ge.get("drc", "unknown")} violations, {esc_ge.get("walled", "unknown")} walled.'))]
                        if film.g else []),
                    cues=[(0, 'swell', 0.6)] + ([(0.3, 'shimmer', 0.5), (0.6, 'chord' if esc_clean else 'alert', 1.0)]
                                                if film.g else [])))
    if n_peers is not None:
        # (the filmed bus's scenes after the others finished: their copper under it, faint)
        for s_ in sc[n_peers:]:
            if s_.key not in ('together', 'coda'):
                s_.fn = with_peers(film, s_.fn)
    for s in sc:
        s.secs *= speed
    return sc


# =============================================================================== the short (--short)
SHORT_BPM = 80             # the short's pulse: every cut on a beat, a clip's steps on its beats or half beats
SHORT_LAY = 0.3            # the lay's share of the short's beats (5 at least): the last clip
SHORT_MIN = 4              # a clip's beats at the least


class Clip(Scene):
    """a clip of the short: a scene with no words, `beats` of the music long, drawn at the film's one framing of the
    bus; its stage the film's own (the stage's bell at its cut)"""

    def __init__(self, key, stage, beats, fn, cues=()):
        super().__init__(key, stage, beats * 60.0 / SHORT_BPM, fn, [], cues=cues)
        self.beats = beats


def _quant(t, q):
    """t on the grid q (a step of the music, as a share of the clip)"""
    return round(t / q) * q


def short_clips(film, secs=15.0):
    """THE SHORT: about `secs` of the run's moments, each an excerpt of the film cut tighter -- no words, no captions,
    no cards, something moving from its first frame to its last. Every clip is drawn at ONE framing, the film's own of
    the bus (its source array to its destination), and as the film draws its stage: its colours, its line weights, its
    board; a cut changes what is drawn, never where the camera is. The moments are the run's own and so is their choice
    (the same run gives the same short): each stage the run recorded is a candidate --

      search   the group's ends model searching (a bus step's group), on the model's own clock: every state a solve
               ended on, its ends at both arrays, the ones it moved ringed, and the rounds between them as the film's
               chart of the search draws their bests -- its worth the states the search held, its share of the beats
               its rounds besides;
      strings  each lane a straight string pulled taut round the copper -- the strings that bend;
      solve    CP-SAT's plans one after another, each lane in its references' slot (a negotiated bus's last, the coupled
               solve's) -- the plans;
      rounds   the 2-D geometry's rounds settling the lanes among the cells -- its rounds and their events;
      snap     the smooth lanes laid on the router's grid, the pairs first -- a constant;
      lay      the router's copper coming in, lane after lane, in the order it was laid, a single-ended lane twice as
               fast as a pair's: always, last --

    and the candidates the beats allow, the most worth first, are played in the run's order, SHORT_MIN beats each and
    the rest of the beats shared by worth. The lay is paced as SHORT_LAY of the beats would bring every lane in at a
    pair's pace, and the beats its singles save go to the search first"""
    S, T = film.S, film.S.T
    beat = 60.0 / SHORT_BPM
    nb = max(SHORT_MIN + 5, int(round(secs / beat)))
    TW, VD = film.rules['TW'], film.rules['via']
    prs = set(S.pairs)
    STRC = (236, 222, 130)

    def fanout(C, a):
        """the film's own ground of a stage: every other net's copper, faint, and the fanout's at `a`"""
        C.segs(film.other_c[0], alpha=0.22)
        C.segs(film.fo_c[0], alpha=a)
        C.cvias(film.fo_c[1], a)

    def starts(beats, n, per=1):
        """the moments n steps of a clip of `beats` start at, each on the pulse (`per` steps to a beat at most)"""
        G = beats * per
        return [math.ceil(k * G / n - 1e-9) / G for k in range(n)] + [1.0]

    def stepper(t, t0s):
        """(step, the share of it gone) at t, the steps starting at t0s"""
        k = max(0, min(len(t0s) - 2, int(np.searchsorted(t0s, t, side='right')) - 1))
        return k, span(t, t0s[k], t0s[k + 1])

    def thin(seq, n):
        """seq cut to n at the most, evenly, its first and its last kept"""
        return seq if len(seq) <= n else [seq[i] for i in sorted({int(round(j * (len(seq) - 1) / (n - 1)))
                                                                   for j in range(n)})]

    cands = []
    # ------------------------------------------------------------------ the group's search
    if film.g and len(film.g.states) > 1:
        g, G = film.g, S.G
        st_ = lambda q: [(a, b, c, d_, TW, L, None) for a, b, c, d_, L in q]   # noqa: E731
        # (the search on the model's own clock: every state a solve ended on -- its ends at both arrays, the ones it
        # moved ringed -- and between them the rounds, each one's best as the search reported it, drawn as the film's
        # chart draws them: no ends are recorded for a round, so a round moves no end on the board)
        m_t = [M_['model']['t'] for M_ in G.models]
        r_t = [r_['t'] for r_ in G.rounds if r_['best'] and r_['best'] > 0]
        t_all = max(m_t + r_t + [1.0])
        line_v = [M_['uncounted'] or M_['start_value'] for M_ in G.models]
        lv_ = [v for v in line_v if v] or [1.0]
        vals = [r_['best'] for r_ in G.rounds if r_['best'] and r_['best'] > 0] + lv_
        vlo, vhi = max(1.0, min(vals) * 0.9), max(lv_) * 1.8
        rcol = lambda lab: ((255, 170, 60) if 'costly' in lab else (90, 230, 255) if 'rest held' in lab else  # noqa: E731
                            (255, 90, 210) if lab == 'the model' else (215, 220, 228))
        # (the chart below the bus where the frame has room for it, else above it)
        vy0, vy1 = film.X(0, film.view[1])[1] / SS, film.X(0, film.view[3])[1] / SS
        cbox = (60, H - 150, W - 60, H - 24) if H - vy1 >= 140 or vy0 < 140 else (60, 24, W - 60, 150)

        def c_search(beats):
            # (every state, and the rounds sampled evenly between them, a step each on the quarter beats)
            steps = 4 * beats
            ev_ = sorted((t_, 0, j_) for j_, t_ in enumerate(m_t))
            if len(ev_) > steps:
                ev_ = thin(ev_, steps)
            else:
                ev_ = sorted(ev_ + [(t_, 1, -1) for t_ in thin(sorted(r_t), steps - len(ev_))])
            t0s = starts(beats, len(ev_), per=4)
            jat = []
            for _t, kind_, j_ in ev_:
                jat.append(j_ if kind_ == 0 else (jat[-1] if jat else 0))

            def fn(C, t, T_):
                C.segs(film.other_c[0], alpha=0.22)
                k, f = stepper(t, t0s)
                j = jat[k]
                ends = g.states[j]['ends']
                for segs, vias in ends.values():
                    C.segs(st_(segs))
                    C.cvias([(x, y, VD, None) for x, y in vias])
                for key in (g.states[j]['moved'] if j else ()):
                    if key in ends and ends[key][0]:
                        C.wring(*ends[key][0][-1][2:4], 0.3, MOVED_C, 2)
                # the film's chart of the search, its clock run to this step's moment
                now = ev_[k][0] + (ev_[k + 1][0] - ev_[k][0]) * f if k + 1 < len(ev_) else ev_[k][0]
                C.card(cbox, alpha=0.92)
                gx0, gy0, gx1, gy1 = cbox[0] + 20, cbox[1] + 14, cbox[2] - 20, cbox[3] - 14
                Xc = lambda ts: gx0 + ts / t_all * (gx1 - gx0)                                        # noqa: E731
                Yc = lambda v: gy1 - (math.log(max(v, vlo)) - math.log(vlo)) / (math.log(vhi) - math.log(vlo)) \
                    * (gy1 - gy0)                                                                     # noqa: E731
                C.line([(gx0, gy0), (gx0, gy1), (gx1, gy1)], DIM, 1)
                for r_ in G.rounds:
                    if r_['t'] <= now and r_['best'] and vlo <= r_['best'] <= vhi:
                        x, y = Xc(r_['t']), Yc(r_['best'])
                        C.d.ellipse([(x - 2.4) * SS, (y - 2.4) * SS, (x + 2.4) * SS, (y + 2.4) * SS],
                                    fill=rcol(r_['label']))
                for tt, v in zip(m_t, line_v):
                    if v and tt <= now:
                        x, y = Xc(tt), Yc(v)
                        C.d.rectangle([(x - 4) * SS, (y - 4) * SS, (x + 4) * SS, (y + 4) * SS], fill=ACCENT)
                C.line([(Xc(min(now, t_all)), gy0), (Xc(min(now, t_all)), gy1)], (120, 126, 140), 1)
            # (a soft tick on each state the search ends a solve on)
            return Clip('search', 'search', beats, fn,
                        cues=[(t0s[k], 'tick', 0.4) for k in range(1, len(ev_)) if ev_[k][1] == 0])
        # (its worth the states the search held; its share of the beats, its rounds besides)
        cands.append((0, 'search', 1 + math.log2(len(g.states)), c_search, 1 + math.log2(len(g.states) + len(r_t))))

    # ------------------------------------------------------------------ the taut strings
    if film.strings:
        slen = {n: plen(P) for n, P in film.strings.items()}
        sch = {n: math.hypot(P[-1][0] - P[0][0], P[-1][1] - P[0][1]) for n, P in film.strings.items()}
        bent = [n for n in slen if slen[n] > sch[n] + 0.01]

        def c_strings(beats):
            def fn(C, t, T_):
                fanout(C, 0.7)
                u_ = span(t, 0.0, 0.9)
                if film.relax:
                    j_ = int(min(len(film.relax) - 2, max(0, np.searchsorted(film.relax_t, u_, side='right') - 1)))
                    f_ = span(u_, film.relax_t[j_], film.relax_t[j_ + 1])
                    st = {n: A + (film.relax[j_ + 1][n] - A) * ease(f_) for n, A in film.relax[j_].items()}
                else:
                    st = {n: P for n, (P, _b) in morph(film.st['chord'], film.st['string'], ease(u_)).items()}
                for n, P in st.items():
                    C.wline([tuple(q) for q in P], PAIR_COL if n in prs else STRC, 1.3)
                for n in film.strings:
                    tooth, _s, _l, berth = film.ends[n]
                    C.wdot(*tooth, 2.6, TEXT)
                    C.wdot(*berth, 2.6, TEXT)
            cues = [(0.0, 'shimmer', 0.3)] + ([(0.9 * film.relax_t[j], 'tick', 1 - n_ / max(1, len(film.strings)))
                                              for j, (_s, _k, n_) in enumerate(film.relax_at)] if film.relax else
                                             [(0.0, 'glide', 0.5, 0.9)]) + [(0.9, 'pluck', 0.2)]
            return Clip('strings', 'strings', beats, fn, cues=cues)
        cands.append((1, 'strings', 1 + math.log2(1 + len(bent)), c_strings, None))

    # ------------------------------------------------------------------ the solve's plans, in the references' slots
    if film.plans:
        o0, o1 = film.plan_meta[0]['obj'], film.plan_meta[-1]['obj']
        better = lambda v: min(1.0, max(0.0, (o0 - v) / (o0 - o1))) if o0 > o1 else 1.0     # noqa: E731
        proved = T['final'].get('proved', True)

        def c_solve(beats):
            ps = thin(list(film.pshow), 2 * beats - (1 if film.coupled else 0))
            keys = [f'plan{i}' for i in ps] + (['coupled'] if film.coupled else [])
            objs = [film.plan_meta[i]['obj'] for i in ps] + ([o1] if film.coupled else [])
            t0s = starts(beats, len(keys), per=2)

            def fn(C, t, T_):
                fanout(C, 0.55)
                k, f = stepper(t, t0s)
                ka, kb = keys[max(0, k - 1)], keys[k]
                mt = ease(span(f, 0.0, 0.3)) if k else 1.0
                C.lanes(morph(film.st[ka], film.st[kb], mt), w=2.2)
                C.vias([(n, x, y) for n, x, y, al in morph_vias(film.vias[ka], film.vias[kb], mt) if al > 0.5])
            cues = [(t0s[k], 'bell', better(o)) for k, o in enumerate(objs) if k] + \
                [(t0s[len(keys) - 1], 'chord', 1.0 if proved and not film.coupled else 0.0)]
            return Clip('solve', 'solve', beats, fn, cues=cues)
        cands.append((2, 'solve', 2 + math.log2(1 + len(film.plans)), c_solve, None))

    # ------------------------------------------------------------------ the 2-D geometry's rounds
    if S.Et and len(film.eseq) > 2:
        E = S.Et
        seq_e = film.eseq
        BP = lambda p: film.mp.pt(p[0], p[1])                       # noqa: E731
        nlay = len(E['layers'])
        CELLC = [mix(BODY, film.col[b], 0.6) for b in film.ebit]
        mesh_px = [np.array([film.X(x, y) for x, y in V], float).reshape(-1, 2) for V, _T in film.emesh[:nlay]]
        px_of = lambda pts: [film.X(*BP(q)) for q in pts]           # noqa: E731
        lit_px = {r['it']: px_of(r['lit']) for r in E['rounds']}
        hot_px = {r['it']: px_of(r['hot']) for r in E['rounds']}
        top = max([r['nlive'] for r in E['rounds']] + [1])

        def c_rounds(beats):
            n = len(seq_e)
            idx = sorted({int(round((j / (2 * beats - 1)) ** 1.7 * (n - 1))) for j in range(2 * beats)})
            t0s = starts(beats, len(idx), per=2)

            def fn(C, t, T_):
                fanout(C, 0.5)
                for L in range(nlay):
                    col = mix(BODY, CELLC[L], 0.26)
                    px, T_c = mesh_px[L], film.emesh[L][1]
                    for i_, j_, k_ in T_c:
                        C.d.line([tuple(px[i_]), tuple(px[j_]), tuple(px[k_]), tuple(px[i_])], fill=col, width=SS)
                k, f = stepper(t, t0s)
                (ka, _k, _r, sa), (kb, kind, rec_, sb) = seq_e[idx[max(0, k - 1)]], seq_e[idx[k]]
                m = ease(span(f, 0.2, 0.8)) if k else 1.0
                C.lanes(morph(film.st[ka], film.st[kb], m), w=2.0)
                C.vias([(n_, x, y) for n_, x, y, _al in morph_vias(film.vias[ka], film.vias[kb], m)])
                if kind == 'round':
                    c1 = mix(BODY, (235, 238, 245), 1 - ease(span(f, 0.55, 1.0)))
                    for (x_, y_) in lit_px[rec_['it']]:
                        C.d.rectangle([x_ - SS / 2, y_ - SS / 2, x_ + SS / 2, y_ + SS / 2], fill=c1)
                    a_h = ease(span(f, 0.25, 0.5)) * (1 - ease(span(f, 0.85, 1.0)))
                    if a_h > 0.02:
                        c2 = mix(BODY, ACCENT, a_h)
                        for (x_, y_) in hot_px[rec_['it']]:
                            C.d.ellipse([x_ - 1.6 * SS, y_ - 1.6 * SS, x_ + 1.6 * SS, y_ + 1.6 * SS], fill=c2)
                        for h in rec_['vhop']:
                            C.wring(*BP(h['to']), 0.22, mix(BODY, EVC['vhop'], a_h), 1.6)
                        for b in rec_['bulge']:
                            C.wring(*BP(b['xy']), 0.14, mix(BODY, EVC['bulge'], a_h), 1.4)
                        for q in rec_['pulled']:
                            C.wring(*BP(q), 0.14, mix(BODY, EVC['pull'], a_h), 1.4)
                elif kind == 'slide':
                    a_s = math.sin(math.pi * f)
                    for vk in rec_['vias']:
                        C.wline([sa['via'][vk], sb['via'][vk]], mix(BODY, ACCENT, a_s), 2.0)
                        C.wring(*sb['via'][vk], 0.4, mix(BODY, ACCENT, a_s), 2.5)
            cues = []
            for k in range(1, len(idx)):
                kind, rec_ = seq_e[idx[k]][1], seq_e[idx[k]][2]
                if kind == 'round':
                    cues.append((t0s[k], 'round', rec_['nlive'] / top))
                    if rec_['hops']:
                        cues.append((t0s[k] + 0.25 * (t0s[k + 1] - t0s[k]), 'blip', min(1.0, rec_['hops'] / 12)))
                elif kind == 'slide':
                    cues.append((t0s[k], 'slide', 0.5, t0s[k + 1]))
            return Clip('rounds', 'geometry', beats, fn, cues=cues)
        tot_ = sum(r['hops'] + r['nexc'] + r['npull'] for r in E['rounds'])
        cands.append((3, 'rounds', 1 + math.log2(1 + len(E['rounds'])) + 0.5 * math.log10(1 + tot_), c_rounds, None))

    # ------------------------------------------------------------------ the snap
    if all(k in film.st for k in ('pol', 'pairs', 'held', 'snap')):
        def c_snap(beats):
            def fn(C, t, T_):
                fanout(C, 0.5)
                mp_, mq, ms = ease(span(t, 0.05, 0.28)), ease(span(t, 0.4, 0.55)), ease(span(t, 0.62, 0.85))
                pr = morph(film.st['pol'], film.st['pairs'], mp_)
                si = morph(film.st['pol'], film.st['held'], mq)
                si = morph(si, film.st['snap'], ms) if ms > 0 else si
                pr = morph(pr, film.st['snap'], ms) if ms > 0 else pr
                st = {n: (pr[n] if n in prs else si[n]) for n in film.M if n in pr and n in si}
                dim = 0.25 if t < 0.36 else 1.0
                C.lanes(st, w=2.2, only=prs if t < 0.36 else None, dim=dim, hl=prs if t < 0.36 else None,
                        ribbon=t >= 0.12)
                vs = film.vias['snap'] if t >= 0.62 else film.vias['pairs'] if t >= 0.2 else film.vias['pol']
                C.vias(vs, only=prs if t < 0.36 else None, dim=dim)
            return Clip('snap', 'snap', beats, fn, cues=[(0.05, 'snap', 0.4), (0.4, 'snap', 0.6), (0.62, 'snap', 0.8)])
        cands.append((4, 'snap', 1.0, c_snap, None))

    # ------------------------------------------------------------------ the lay, last
    order = [n for n in S.route_order if n in film.routed] + [n for n in film.M if n not in S.route_order]
    order = [n for n in order if film.routed[n][0] or film.routed[n][1]]
    clean = S.connected and S.drc_clean

    # (the lay's pace: a pair's lane at the pace of SHORT_LAY of the beats for every lane, a single's twice as fast)
    wt = {n: (1.0 if n in prs else 0.5) for n in order}
    b_lay0 = max(5, int(round(nb * SHORT_LAY)))
    b_lay = max(3, int(round(b_lay0 * sum(wt.values()) / max(1, len(order)))))

    def c_lay(beats):
        q = 1.0 / (4 * beats)               # (lanes come in on the quarter beats, the last one's done at the end)
        cum = np.concatenate([[0.0], np.cumsum([wt[n] for n in order])])
        t_in = {n: _quant((1 - q) * cum[j] / max(1e-9, cum[-1] - wt[order[-1]]), q) if len(order) > 1 else 0.0
                for j, n in enumerate(order)}

        def fn(C, t, T_):
            C.segs(film.other_c[0], alpha=0.22)
            C.segs(film.fo_c[0], alpha=0.9)
            C.cvias(film.fo_c[1])
            if 'snap' in film.st:
                C.lanes(film.st['snap'], alpha=0.28, w=1.6, ribbon=False)
            for n in order:
                a = span(t, t_in[n], t_in[n] + q)
                if a <= 0:
                    continue
                segs, vias = film.routed[n]
                C.segs(segs, alpha=a)
                C.cvias(vias, a)
        cues = [(t_in[n], 'pluck', j / max(1, len(order) - 1)) for j, n in enumerate(order)] + \
            [(_quant(0.75, q), 'chord' if clean else 'alert', 1.0)]
        return Clip('lay', 'lay', beats, fn, cues=cues)

    # ---- the clips the beats allow, the most worth first, SHORT_MIN beats each, the rest shared by worth (the search's
    # by its rounds too); and the beats the lay's singles save, to the search first, else shared the same way
    rest = nb - b_lay0
    take = sorted(sorted(cands, key=lambda c: (-c[2], c[0]))[:max(0, rest // SHORT_MIN)], key=lambda c: c[0])
    give = {c[1]: SHORT_MIN for c in take}
    wgt = {c[1]: c[4] or c[2] for c in take}

    def share(extra):
        """`extra` beats shared among the clips taken by their weights; what none could take"""
        if not take or extra <= 0:
            return max(0, extra)
        w_ = sum(wgt.values())
        sh = {k: extra * wgt[k] / w_ for k in wgt}
        for k in sh:
            give[k] += int(sh[k])
        for k in sorted(sh, key=lambda k: (-(sh[k] - int(sh[k])), k))[:extra - sum(int(v) for v in sh.values())]:
            give[k] += 1
        return 0
    left = share(rest - SHORT_MIN * len(take))
    saved = b_lay0 - b_lay
    if 'search' in give:
        give['search'] += saved
    else:
        left += share(saved)
    clips = [c[3](give[c[1]]) for c in take] + [c_lay(b_lay + left)]
    if getattr(film, 'neg', None):
        # (a negotiated bus: its clips after the other buses finished drawn with their copper under them, faint -- as
        # the film draws those stages)
        for c_ in clips:
            if c_.key in ('rounds', 'snap', 'lay'):
                c_.fn = with_peers(film, c_.fn)
    return clips


# =============================================================================== the frames
_FILM = None


def chrome(C, scene, t, T_total, t_abs, scenes):
    """header, caption, the stage timeline"""
    C.text(HEAD[0], HEAD[1] + 4, 'the whole route', 26, TEXT, bold=True)
    S = C.f.S
    C.text(HEAD[0] + 230, HEAD[1] + 10, f'{os.path.basename(S.base_path)} → {S.src} to {S.dst}, {len(C.f.M)} lanes', 18, DIM)
    if scene.stage:
        C.text(HEAD[2], HEAD[1] + 4, scene.stage, 26, ACCENT, bold=True, anchor='ra')
    if len(C.f.LN) > 2 and scene.stage:
        # (more layers than two: the legend of the layers the lanes stand on, top to bottom of the board)
        x = HEAD[2] - C.tlen(scene.stage, 26, bold=True) - 40
        for L in reversed(C.f.used):
            x -= C.tlen(L, 15)
            C.text(x, HEAD[1] + 12, L, 15, DIM)
            x -= 30
            C.line([(x, HEAD[1] + 21), (x + 24, HEAD[1] + 21)], layer_col(L), 5)
            x -= 16
    cap = None
    for (t0, s) in scene.captions:
        if t >= t0:
            cap = (t0, s)
    if cap:
        a = ease(span(t, cap[0], cap[0] + 0.6 / max(1.0, scene.secs)))
        for j, ln in enumerate(C.wrap(cap[1], 21, CAP[2] - CAP[0])[:2]):
            C.text(CAP[0], CAP[1] + 4 + j * 27, ln, 21, mix(BG, TEXT, a))
    # the timeline
    # (the stages the film's scenes walk, in their order: a bus step's group's ends first, where the run has them)
    stages = list(dict.fromkeys(s_.stage for s_ in scenes if s_.stage)) or STAGES
    x0, x1 = FOOT[0], FOOT[2]
    wch = (x1 - x0) / len(stages)
    for j, st in enumerate(stages):
        on = scene.stage == st
        done = scene.stage in stages and stages.index(scene.stage) > j
        C.rect((x0 + j * wch + 3, FOOT[1] + 4, x0 + (j + 1) * wch - 3, FOOT[3] - 16),
               fill=(60, 52, 20) if on else PANEL, outline=ACCENT if on else (50, 56, 66), width=1, r=6)
        C.text(x0 + j * wch + wch / 2, FOOT[1] + 19, st, 16 if len(stages) <= 10 else 14, ACCENT if on else
               (TEXT if done else DIM), bold=on, anchor='mm')
    C.rect((x0, FOOT[3] - 8, x0 + (x1 - x0) * t_abs / T_total, FOOT[3] - 4), fill=(90, 96, 108))


_BASE = None


def base_image(film):
    """the static substrate: the board (outline, pads) in its panel, drawn once"""
    from PIL import Image
    sys.path.insert(0, PYR)
    from route_render import BoardRenderer
    S = film.S
    img = Image.new('RGB', (W * SS, H * SS), BG)
    # (the renderer's DARK theme, whatever its default is: its board body is the BODY every overlay here fades to)
    r = BoardRenderer(S.fo, size=1000, supersample=1, show_zones=False, margin_frac=0.0, theme='dark')
    bw, bh = (BOARD[2] - BOARD[0]) * SS, (BOARD[3] - BOARD[1]) * SS
    r.set_canvas(bw, bh)
    # the renderer's view, the same as the film's camera (its Transform fits and centres as ours does)
    v = film.view
    r.set_view(v)
    img.paste(r._base, (BOARD[0] * SS, BOARD[1] * SS))
    # (check the two transforms agree: the renderer's point for the view's corner and ours)
    px = r.tf.pt(v[0], v[1])
    ox = film.X(v[0], v[1])
    assert abs(px[0] + BOARD[0] * SS - ox[0]) < 2 and abs(px[1] + BOARD[1] * SS - ox[1]) < 2, (px, ox)
    return img


def render_frame(i):
    film, scenes, fps, T_total, base, short = _FILM
    t_abs = i / fps
    acc = 0.0
    for sc in scenes:
        if t_abs < acc + sc.span or sc is scenes[-1]:
            break
        acc += sc.span
    t = sc.at(t_abs - acc)
    C = Canvas(film, base)
    sc.fn(C, t, t_abs)
    if not short:
        chrome(C, sc, t, T_total, t_abs, scenes)       # (the short: no header, caption or timeline)
    return C.img.reduce(SS)


def frame_bytes(i):
    return render_frame(i).tobytes()


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--_trace':
        if sys.argv[2] == 'solve':
            _trace_solve(sys.argv[3], sys.argv[4])
        elif sys.argv[2] == 'embed':
            _trace_embed(sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])
        else:
            _trace_geo(sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])
        return
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('rundir')
    ap.add_argument('out', nargs='?')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument('--stills', help='write PNG stills into this directory (at --at seconds, or one per scene)')
    ap.add_argument('--at', help='comma list of times (s) for --stills')
    ap.add_argument('--human', help='the human\'s board, drawn at the end (default fb_t2q_human.kicad_pcb beside this script)')
    ap.add_argument('--speed', type=float, default=1.0, help='scale every scene\'s length')
    ap.add_argument('--title', default='The whole route', help='the title card\'s title')
    ap.add_argument('--force', action='store_true', help='film a run whose re-runs do not reproduce the chain\'s files')
    ap.add_argument('--check', action='store_true',
                    help='run the round\'s stages again, say whether each reproduces the chain\'s file, and stop (exit 1 '
                         'when one does not)')
    ap.add_argument('--scenes', action='store_true', help='list the scenes and their start times, and stop (with '
                                                           '--sound: and every line\'s and effect\'s time)')
    from whole_movie_sound import DEFAULT_VOICE
    ap.add_argument('--sound', action='store_true',
                    help='a sound track: every scene narrated, effects on what the scenes draw, and music, mixed by '
                         'ffmpeg (whole_movie_sound.py) -- the film\'s timeline fitted to the narration')
    ap.add_argument('--short', nargs='?', type=float, const=15.0, default=None, metavar='SECS',
                    help='a SHORT instead of the film: about SECS (default 15) of the run\'s moments, each an excerpt '
                         'of the film at one framing of the bus, cut on the music\'s pulse, with music and effects and no '
                         'narration, captions or cards (short_clips); --stills and --scenes work on it as on the film')
    ap.add_argument('--voice', default=DEFAULT_VOICE,
                    help=f'the narration\'s voice (default {DEFAULT_VOICE}): kokoro:NAME, a voice of the neural '
                         'Kokoro-82M, installed on first use into its own venv in the user\'s cache ($XDG_CACHE_HOME or '
                         '~/.cache, krt_tts) and its weights fetched there once; or a macOS `say` voice by name (one '
                         '`say -v ?` lists, e.g. Samantha)')
    a = ap.parse_args()
    if a.short is not None and a.sound:
        raise SystemExit('whole_movie: --short has a sound track of its own (music and effects, no narration) -- '
                         '--sound is the film\'s')
    t0 = time.time()
    S = load_story(a)
    if a.short is not None:
        # (the short's frame: the board over all of it, and the braid too -- no header, captions or timeline)
        global BOARD, BRAID
        BOARD, BRAID = (0, 0, W, H), (40, 80, W - 40, H - 80)
    film = Film(S)
    film.title = a.title
    scenes = build_scenes(film, a.speed) if a.short is None else short_clips(film, a.short)
    if a.sound:
        # (the narration spoken first, and the scenes fitted to it: the film's timeline is then the sound's)
        import atexit
        import shutil
        import tempfile
        import whole_movie_sound as wms
        work = tempfile.mkdtemp(prefix='whole_movie_sound.')        # (the system's temporary directory, $TMPDIR)
        atexit.register(shutil.rmtree, work, ignore_errors=True)
        wms.narrate(scenes, work, voice=a.voice)
        wms.fit(scenes)
    T_total = sum(s.span for s in scenes)
    acc = 0.0
    for s in scenes:
        print(f'  {acc:6.1f} s  {s.key:9s} {s.span:5.1f} s' + (f'  ({s.beats} beats)' if a.short is not None else ''))
        acc += s.span
    if a.sound:
        wms.report(scenes)
    if a.scenes:
        return
    global _FILM
    base = base_image(film)
    _FILM = (film, scenes, a.fps, T_total, base, a.short is not None)
    print(f'whole_movie: {T_total:.0f} s at {a.fps} fps, prepared in {time.time() - t0:.0f} s')
    if a.stills:
        os.makedirs(a.stills, exist_ok=True)
        if a.at:
            ts = [float(x) for x in a.at.split(',')]
        else:
            ts, acc = [], 0.0
            for s in scenes:
                ts.append(acc + s.span * 0.75)
                acc += s.span
        for tt in ts:
            img = render_frame(int(round(tt * a.fps)))
            p = os.path.join(a.stills, f'still_{tt:06.1f}.png')
            img.save(p)
            print('wrote', p)
        if not a.out:
            return
    if not a.out:
        raise SystemExit('whole_movie: name OUT.mp4 (or --stills DIR)')
    nfr = int(round(T_total * a.fps))
    import shutil
    ffmpeg = shutil.which('ffmpeg') or next((p for p in ('/opt/homebrew/bin/ffmpeg', '/usr/local/bin/ffmpeg')
                                             if os.path.isfile(p)), None)
    if ffmpeg is None:
        raise SystemExit('whole_movie: no ffmpeg found (on the PATH, or under /opt/homebrew or /usr/local) -- the mp4 '
                         'needs it; --stills DIR writes PNGs without it')
    vout = a.out
    if a.sound:
        # (the sound made before the frames, so a sound that cannot be made stops the film before they are drawn; the
        # frames then go to a silent mp4 the sound is mixed onto)
        stems = wms.score(scenes, work)
        vout = os.path.join(work, 'video.mp4')
    elif a.short is not None:
        # (the short's music and effects, on its beat)
        import atexit
        import tempfile
        import whole_movie_sound as wms
        work = tempfile.mkdtemp(prefix='whole_movie_short.')
        atexit.register(shutil.rmtree, work, ignore_errors=True)
        stems = wms.short_score(scenes, work, SHORT_BPM)
        vout = os.path.join(work, 'video.mp4')
    cmd = [ffmpeg, '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(a.fps),
           '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', vout]
    ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    import multiprocessing as mpr
    t1 = time.time()
    with mpr.get_context('fork').Pool(a.workers) as pool:
        for j, buf in enumerate(pool.imap(frame_bytes, range(nfr), chunksize=4)):
            ff.stdin.write(buf)
            if j % (a.fps * 10) == 0:
                print(f'  frame {j}/{nfr} ({time.time() - t1:.0f} s)', flush=True)
    ff.stdin.close()
    if ff.wait() != 0:
        raise SystemExit('whole_movie: ffmpeg failed')
    if a.sound or a.short is not None:
        wms.mux(ffmpeg, vout, stems, a.out)
    print(f'wrote {a.out}: {nfr} frames, {T_total:.0f} s, in {time.time() - t0:.0f} s'
          + (', with sound' if a.sound or a.short is not None else ''))


if __name__ == '__main__':
    main()
