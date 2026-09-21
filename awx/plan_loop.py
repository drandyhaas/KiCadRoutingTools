#!/usr/bin/env python3
"""plan_loop.py -- the PLAN-LEVEL LOOP: the routing inside the planning
iteration (#622, 2026-09-18).

Why. Every plan-time judge this campaign built is measured flat or
anti-correlated with the routed count at K51 (README: *K51's gap*; the
0918 floor ranking, 59 plans, |rho| < 0.2 on every feature), and the
per-net replan judges with the world FROZEN -- at K51 the schedule is
the cost, so a move that stands at the frozen probe re-braids worse
eight rounds running. The one judge that is right is the route itself,
and the one search that can use it is a re-solve judged by the route.

What a candidate IS. The first cut of this loop hinted the incumbent plan
and priced its classes, then let the solve run free: it moved 33 of 48
ends and routed 125 against 98, because the model's objective is the
thing measured anti-correlated with the route, and a hint is only a
starting point. So a candidate is a NAMED PERTURBATION of the incumbent:
every net not named is HELD at the incumbent's berth and its tooth as it
stands; the worst nets are freed, at one end or both, with their current
class banned or not; the wide arms free the crossers of those nets too.
The solver's freedom is exactly the hypothesis. (`radius` is the ablation:
no holds, at most r ends off the hinted plan, the solver choosing which.)

  round 0   the chain as it stands (or a recorded run, --from=STEM).
  round r   1. VERDICT off the incumbent R: per net its class in the
               braid's schedule, its real vias, refused; and off EVERY
               routed board seen so far the RESIDUAL per net -- real vias
               minus the plan's own prediction (the count judge, residual
               off) -- attributed half to the class of each end.
            2. BASE: the incumbent's SOURCE VIEW (its fanout board with the
               destination copper stripped), so its teeth are the standing
               teeth and a held source is literally unchanged copper.
            3. FEEDBACK (plan_feedback.py, env PLAN_LOOP_FEEDBACK): holds,
               bans, the residual prices, the incumbent plan as hint.
            4. CANDIDATES: one full chain each (plan, fan out both ways,
               braid both arms, grade) -- INDEPENDENT, so --jobs runs them
               side by side; that is where the parallelism goes.
            5. JUDGE: (open, drc, vias) of each candidate's shipped board
               against the incumbent's; the best replaces it only when
               strictly better (--margin raises the bar), so the kept board
               is monotone BY CONSTRUCTION. A candidate that lost retires
               its hypothesis (the next round's worst list rotates past it);
               every candidate's boards feed the residuals either way.

Nothing here reads a face, a ref or a board name: the classes come off
the boards, the residuals off the grade, the candidates off the verdict.

usage: plan_loop.py TAG K [--from=STEM] [--board=BASE] [--dest=DU1]
                    [--rounds=4] [--worst=3] [--cands=dst,src,both,free]
                    [--jobs=1] [--gain=1.0] [--margin=0] [--patience=2]
                    [--min-vias=3] [--crossers=4] [--radius=6]
                    [--base=incumbent|bench] [--resume=1]
  --cands   dst / src / both = free the worst nets at that end and BAN their
            current class there (the other end held); free = both ends free,
            no ban; a `+w` suffix (dst+w ...) also frees the crossers of
            those nets (no ban); radius = the ablation (no holds, the trust
            region); noprice / nohint = `both` with one channel off; none =
            holds only (reproduces the incumbent; a determinism control).
  writes  tmp/TAG/r<N>/<cand>_* per round, tmp/TAG/best_k<K>.kicad_pcb (+
          best_fo_k<K> and its sidecar, best_src_k<K> = the round base) when
          the incumbent changes, tmp/TAG/loop_k<K>.json (the ledger;
          --resume=1 continues it).
Run it under the chain's own env (PLAN_PAGES=1 PLAN_JUDGE=count ...): the
verdict's menus and judge must be the ones the candidates plan with.
"""
import concurrent.futures
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from collections import Counter, defaultdict

ARGV = [a for a in sys.argv[1:] if not a.startswith('--')]
OPTS = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--'))
# replan.py sets SRC_CLIMB=14 at import for its own menus; the loop's
# verdict must read the menus the CHAIN plans with, so the chain's value
# (unset = 0) is pinned before that import.
os.environ.setdefault('SRC_CLIMB', '0')

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import fanout_from_plan as fp  # noqa: E402
import source_realize as sr  # noqa: E402
import braid as te  # noqa: E402
import replan  # noqa: E402  Board, verdict, copy_board, source_view
from coherent_nets import coherent_nets  # noqa: E402
import dedupe_boards  # noqa: E402
import rules as _rules  # noqa: E402

ENV_SHOWN = ('PLAN_PAGES', 'PLAN_JUDGE', 'PLAN_JUDGE_LEN', 'SRC_CLIMB', 'DST_CLIMB',
             'DST_WALK', 'CHAIN_FANOUT_AB', 'CHAIN_BRAID_AB', 'PLAN_PAGES_DET')
BASE_KINDS = ('dst', 'src', 'both', 'free', 'radius', 'noprice', 'nohint', 'none')
FREE_ENDS = {'dst': ('dst',), 'src': ('src',), 'both': ('dst', 'src'), 'free': ('dst', 'src'),
             'noprice': ('dst', 'src'), 'nohint': ('dst', 'src'), 'radius': (), 'none': ()}
BAN_ENDS = {'dst': ('dst',), 'src': ('src',), 'both': ('dst', 'src'), 'noprice': ('dst', 'src'),
            'nohint': ('dst', 'src')}


def kind_of(cand):
    """`dst`, `dst+w`, `dst:r8`, `dst+w:r8:w1` -> (base kind, wide, options):
    `rN` = this arm's reach, `wN` = how many worst nets it frees."""
    head, *opts = cand.split(':')
    base = head[:-2] if head.endswith('+w') else head
    if base not in BASE_KINDS:
        sys.exit(f'--cands: {cand!r} is not one of {BASE_KINDS} (optionally +w, :rN, :wN)')
    o = {}
    for t in opts:
        if len(t) < 2 or t[0] not in 'rw' or not t[1:].isdigit():
            sys.exit(f'--cands: {cand!r}: option {t!r} is not rN or wN')
        o[t[0]] = int(t[1:])
    return base, head.endswith('+w'), o


def log(msg=''):
    print(msg, flush=True)


# ------------------------------------------------------------ grading
def grade(board, nets_csv):
    """(opens, drc, vias) by grade_k, or None when it did not grade."""
    r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'), board, nets_csv],
                       capture_output=True, text=True, cwd=HERE)
    line = next((l for l in (r.stdout + r.stderr).splitlines() if l.startswith('GRADE')), '')
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', line)
    if not m or 'BROKEN' in line:
        return None, line
    opens = sorted(line.split('open: ')[1].split(',')) if 'open: ' in line else []
    return [opens, int(m.group(2)), int(m.group(3))], line


def better(g, ref, margin=0):
    """Is grade g better than ref: fewer opens; no DRC; fewer vias by
    more than `margin` (opens equal)."""
    if g is None or ref is None:
        return g is not None and ref is None
    if g[1] != 0:
        return False
    if len(g[0]) != len(ref[0]):
        return len(g[0]) < len(ref[0])
    return g[2] < ref[2] - margin


def fmt_g(g):
    if g is None:
        return 'NO GRADE'
    return f'open {len(g[0])} drc {g[1]} vias {g[2]}' + (f' {g[0]}' if g[0] else '')


# ------------------------------------------------------ the boards seen
def arm_boards(stem, K):
    """The (fanout board, routed board) pairs a chain run left under
    `stem`: every portfolio arm STEM_fo_kK_J<j>_<A|B>, else the single-shot
    pair. Only pairs whose routed board exists."""
    pairs = []
    for j in (0, 1):
        fo = f'{stem}_fo_k{K}_J{j}.kicad_pcb'
        if not os.path.exists(fo):
            continue
        for arm in ('A', 'B'):
            rb = f'{stem}_fo_k{K}_J{j}_{arm}.kicad_pcb'
            if os.path.exists(rb):
                pairs.append((fo, rb))
    if not pairs:
        fo, rb = f'{stem}_fo_k{K}.kicad_pcb', f'{stem}_k{K}.kicad_pcb'
        if os.path.exists(fo) and os.path.exists(rb):
            pairs.append((fo, rb))
    return pairs


def winner_pair(stem, K, pairs):
    """Which arm pair the chain SHIPPED as STEM_kK: named in the chain's
    output when there is one, else the arm whose copper is the shipped
    board's (dedupe_boards' fingerprint), else the single pair."""
    shipped = f'{stem}_k{K}.kicad_pcb'
    out = stem + '.out'
    if os.path.exists(out):
        m = re.findall(r'braid A/B: keeping (\S+\.kicad_pcb)', open(out, encoding='utf-8').read())
        if m:
            for fo, rb in pairs:
                if os.path.basename(rb) == m[-1]:
                    return fo, rb
    if os.path.exists(shipped) and len(pairs) > 1:
        fpr = dedupe_boards.fingerprint(shipped)
        for fo, rb in pairs:
            if dedupe_boards.fingerprint(rb) == fpr:
                return fo, rb
    return pairs[0] if pairs else (None, None)


# ---------------------------------------------------------- the verdict
class Seen:
    """The plan states and model predictions per fanout board, computed
    once (the two braid arms of one fanout share its ends)."""

    def __init__(self, names, dref):
        self.names, self.dref = names, dref
        self.boards = {}

    def board(self, F):
        if F not in self.boards:
            B = replan.Board(F, self.names, self.dref)
            named = set(json.load(open(F[:-len('.kicad_pcb')] + '.plan.json')).get('ends', {}))
            pred, bp = B.model(named)
            self.boards[F] = (B, pred, bp)
        return self.boards[F]


def real_vias(R, B):
    pcb = parse_kicad_pcb(R)
    return {nm: sum(1 for v in pcb.vias if v.net_id == B.byname[nm][0]) for nm in B.names}


def refused_on(R):
    p = R[:-len('.kicad_pcb')] + '_refusals.json'
    return set(json.load(open(p))) if os.path.exists(p) else set()


def verdict_of(R):
    """replan.verdict on the routed board's stem, tolerant of a board
    without a braid log or pack sidecar beside it."""
    try:
        return replan.verdict(R[:-len('.kicad_pcb')])
    except (OSError, ValueError, KeyError):
        return {}


def observe(seen, F, R, obs, src_of):
    """Residuals off one routed board: per net real - predicted, half to
    the class of each end the fanout board carries. Refused nets carry no
    copper and say nothing."""
    B, pred, _bp = seen.board(F)
    real = real_vias(R, B)
    ref = refused_on(R)
    n = 0
    for nm in B.names:
        if nm not in pred or nm in ref:
            continue
        res = real[nm] - pred[nm]
        for end in ('dst', 'src'):
            c = B.cls(nm, end)
            if c:
                obs[(nm, end, c[0], c[1])].append(res / 2.0)
                src_of[(nm, end, c[0], c[1])].add(os.path.basename(R))
                n += 1
    return n


def worst_list(B, V, real, min_vias):
    """The incumbent's nets worst first: refused, then lane vias (the
    ends' own taken off) descending, swimmers before page lanes."""
    rows = []
    for nm in B.names:
        v = V.get(nm, {})
        ev = sum((B.ends[nm][e] or {}).get('vias', 0) for e in ('src', 'dst'))
        lane = max(0, real.get(nm, 0) - ev)
        rows.append((nm, bool(v.get('refused')), lane, v.get('cls', '?')))
    rows.sort(key=lambda t: (not t[1], -t[2], t[3] != 'swim', t[0]))
    return [(nm, ref, lane, cls) for nm, ref, lane, cls in rows if ref or lane >= min_vias]


def crossers_of(bp, nm, cap):
    """The nets whose planned lanes cross `nm`'s in the braid's own plan
    (same corridor, launch and target ranks inverted), nearest by launch
    rank first, at most `cap`."""
    me = bp.get(nm) or {}
    if me.get('launch_idx') is None or me.get('target_idx') is None:
        return []
    out = []
    for o, bo in bp.items():
        if o == nm or bo.get('corridor') != me.get('corridor'):
            continue
        if bo.get('launch_idx') is None or bo.get('target_idx') is None:
            continue
        if (me['launch_idx'] - bo['launch_idx']) * (me['target_idx'] - bo['target_idx']) < 0:
            out.append((abs(me['launch_idx'] - bo['launch_idx']), o))
    return [o for _d, o in sorted(out)[:cap]]


# ---------------------------------------------------------- the feedback
def prices_from(obs, gain):
    out = defaultdict(lambda: defaultdict(dict))
    for (nm, end, d, L), xs in obs.items():
        p = round(gain * sum(xs) / len(xs), 2)
        if abs(p) >= 0.05:
            out[nm][end][f'{d}/{L}'] = p
    return {nm: {e: dict(v) for e, v in d.items()} for nm, d in out.items()}


def hint_from(B, base_teeth):
    """The incumbent plan by signature: its berth per net, and its tooth
    where that differs from the tooth the round's base stands at (on the
    incumbent's own source view that is never)."""
    h = {}
    n_src = 0
    for nm in B.names:
        d = B.choice.get(nm)
        if d is None:
            continue
        ent = {'dst': sr.move_sig(d), 'src': None}
        s_end = B.ends[nm]['src']
        bt = base_teeth.get(nm)
        moved = (s_end and bt and (s_end['layer'] != bt[1]
                                   or math.hypot(s_end['tooth'][0] - bt[0][0],
                                                 s_end['tooth'][1] - bt[0][1]) > 0.05))
        if moved and B.cur_src.get(nm) is not None:
            ent['src'] = sr.move_sig(B.cur_src[nm])
            n_src += 1
        h[nm] = ent
    return h, n_src


def arm_spec(cand, B, bp, worst, W, tried, tried_kind, refused, crossers_cap, radius, reach_k=0):
    """One candidate's bans, holds and freed nets. Refused nets are freed
    and banned at both ends in every arm; then the W worst nets not yet
    tried under this kind are freed at the kind's ends (banned where the
    kind bans); the wide arms free their crossers too. Every other net is
    held at the incumbent's berth and its standing tooth."""
    base, wide, o = kind_of(cand)
    W = o.get('w', W)
    reach_k = o.get('r', reach_k)
    free_ends = defaultdict(set)
    bans = defaultdict(lambda: defaultdict(list))
    for nm in sorted(refused):
        for end in ('dst', 'src'):
            free_ends[nm].add(end)
            c = B.cls(nm, end)
            if c:
                bans[nm][end].append([c[0], c[1], None, None])
    picked = []
    if base not in ('radius', 'none'):
        for nm, ref, lane, cls in worst:
            if ref or len(picked) >= W:
                continue
            if (base, nm) in tried_kind:
                continue
            keys = [(nm, end) + B.cls(nm, end) for end in BAN_ENDS.get(base, ()) if B.cls(nm, end)]
            if keys and any(k in tried for k in keys):
                continue
            for end in FREE_ENDS[base]:
                free_ends[nm].add(end)
            for end in BAN_ENDS.get(base, ()):
                c = B.cls(nm, end)
                if c:
                    bans[nm][end].append([c[0], c[1], None, None])
            picked.append(nm)
    crossers = []
    if wide:
        for nm in picked:
            for o in crossers_of(bp, nm, crossers_cap):
                if o not in free_ends and o not in picked:
                    crossers.append(o)
                    free_ends[o] |= {'dst', 'src'}
    hold = {}
    if base != 'radius':
        for nm in B.names:
            h = {}
            if 'dst' not in free_ends.get(nm, ()):
                d = B.choice.get(nm)
                c = B.cls(nm, 'dst')
                if d is not None:
                    h['dst'] = sr.move_sig(d)
                if c:
                    h['dst_cls'] = [c[0], c[1]]
            if 'src' not in free_ends.get(nm, ()):
                h['src'] = None
            if h:
                hold[nm] = h
    # the REACH windows: a free end may land only between its held
    # neighbours k ranks away in the incumbent's plan order (per corridor;
    # `bp` is the braid's own plan phase on the incumbent's ends)
    reach = {}
    if reach_k and base != 'radius':
        for end, idx_key in (('dst', 'target_idx'), ('src', 'launch_idx')):
            held_end = {nm for nm, h in hold.items()
                        if (end == 'dst' and ('dst' in h or 'dst_cls' in h)) or (end == 'src' and 'src' in h)}
            for nm in B.names:
                # every net, held or not: a holder the solve un-holds to make
                # room (the unblock) is bounded too -- measured without this,
                # four freed holders took full menus and 25 other nets changed
                me = bp.get(nm) or {}
                if me.get(idx_key) is None:
                    continue
                order = sorted((bo[idx_key], o) for o, bo in bp.items()
                               if o in held_end and o != nm and bo.get('corridor') == me.get('corridor')
                               and bo.get(idx_key) is not None)
                below = [o for i, o in order if i < me[idx_key]]
                above = [o for i, o in order if i > me[idx_key]]
                lo = below[-reach_k] if len(below) >= reach_k else None
                hi = above[reach_k - 1] if len(above) >= reach_k else None
                if lo is not None or hi is not None:
                    reach.setdefault(nm, {})[end] = [lo, hi]
    return ({nm: dict(v) for nm, v in bans.items()}, hold, picked, crossers,
            radius if base == 'radius' else 0, reach, reach_k)


def ban_keys(bans):
    return {(nm, end, b[0], b[1]) for nm, d in bans.items() for end, bl in d.items() for b in bl}


# ------------------------------------------------------------- the chain
def run_chain(tagpath, K, feedback_path, env_extra):
    env = dict(os.environ)
    env.update(env_extra or {})
    if feedback_path:
        env['PLAN_LOOP_FEEDBACK'] = feedback_path
    else:
        env.pop('PLAN_LOOP_FEEDBACK', None)
    os.makedirs(os.path.dirname(os.path.join(HERE, tagpath)), exist_ok=True)
    t0 = time.time()
    with open(os.path.join(HERE, tagpath + '.out'), 'w', encoding='utf-8') as f:
        p = subprocess.run(['bash', 'chain_k.sh', tagpath, str(K)], cwd=HERE, env=env,
                           stdout=f, stderr=subprocess.STDOUT, text=True)
    return p.returncode, round(time.time() - t0)


def keep_incumbent(dst_stem, K, F, R):
    """The incumbent copied to tmp/TAG/best_* with everything the next
    consumer reads (sidecar, log, pack, refusals)."""
    replan.copy_board(R, f'{dst_stem}_k{K}.kicad_pcb', eco=True)
    for ext in ('.log', '.pack.json', '_refusals.json'):
        s = R[:-len('.kicad_pcb')] + ext
        if os.path.exists(s):
            shutil.copy(s, f'{dst_stem}_k{K}{ext}')
    replan.copy_board(F, f'{dst_stem}_fo_k{K}.kicad_pcb')
    shutil.copy(F[:-len('.kicad_pcb')] + '.plan.json', f'{dst_stem}_fo_k{K}.plan.json')


# ------------------------------------------------------------------ main
def main():
    if len(ARGV) < 2:
        sys.exit(__doc__)
    tag, K = ARGV[0], int(ARGV[1])
    bench = OPTS.get('board', os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb'))
    dest = OPTS.get('dest', 'DU1')
    ROUNDS = int(OPTS.get('rounds', 4))
    W = int(OPTS.get('worst', 3))
    CANDS = [c for c in OPTS.get('cands', 'dst,src,both,free').split(',') if c]
    JOBS = int(OPTS.get('jobs', 1))
    GAIN = float(OPTS.get('gain', 1.0))
    MARGIN = int(OPTS.get('margin', 0))
    PATIENCE = int(OPTS.get('patience', 2))
    MIN_VIAS = int(OPTS.get('min-vias', 3))
    CROSSERS = int(OPTS.get('crossers', 4))
    RADIUS = int(OPTS.get('radius', 6))
    REACH = int(OPTS.get('reach', 3))
    BASE_MODE = OPTS.get('base', 'incumbent')
    RESUME = OPTS.get('resume', '0') == '1'
    for c in CANDS:
        kind_of(c)
    if BASE_MODE not in ('incumbent', 'bench'):
        sys.exit('--base: incumbent | bench')
    _r = _rules.install_defaults()
    # GENERAL: the pair is the one named here, on every chain this loop runs
    # (chain_k.sh's own defaults are the bench's; a second article must not
    # silently plan the bench's destination)
    chain_env = {'BASE': os.path.abspath(bench), 'DEST': dest}
    root = os.path.join(HERE, 'tmp', tag)
    os.makedirs(root, exist_ok=True)
    ledger_path = os.path.join(root, f'loop_k{K}.json')
    t_all = time.time()
    log(f'plan_loop: K{K} tag {tag}; {ROUNDS} round(s), worst {W}, candidates {CANDS}, '
        f'jobs {JOBS}, gain {GAIN}, margin {MARGIN}, patience {PATIENCE}, crossers {CROSSERS}, '
        f'radius {RADIUS}, reach {REACH}, base {BASE_MODE}')
    log('  env: ' + ' '.join(f'{k}={os.environ.get(k, "")}' for k in ENV_SHOWN))
    nets_all = coherent_nets(K, bench)
    nets_csv = ','.join(nets_all)
    pcb0 = parse_kicad_pcb(bench)
    byname0 = {n.name.split('/')[-1]: (i, n) for i, n in pcb0.nets.items()}
    ends0 = te.endpoints(pcb0, nets_all, byname0)
    dref = Counter(ends0[nm][2] for nm in nets_all if nm in ends0).most_common(1)[0][0]
    names = [nm for nm in nets_all if nm in ends0 and ends0[nm][2] == dref]
    bench_teeth = None
    if BASE_MODE == 'bench':
        # the teeth the BENCH stands at: what a hint of "no source move" means
        st0 = fp.plan_state(pcb0, names)
        bench_teeth = {nm: (tuple(st0['launch'][nm]), st0['tooth0'].get(nm, 'F.Cu')) for nm in names
                       if nm in st0['launch']}
    log(f'  {len(names)} nets on {dref} ({len(nets_all) - len(names)} elsewhere)')

    seen = Seen(names, dref)
    obs = defaultdict(list)      # (net, end, direction, layer) -> residuals
    src_of = defaultdict(set)
    tried = set()                # (net, end, direction, layer) bans that lost
    tried_kind = set()           # (kind, net) hypotheses that lost
    ledger = {'tag': tag, 'K': K, 'rounds': [], 'incumbent': None}
    inc = None                   # {'F', 'R', 'grade', 'name'}

    def absorb(stem):
        """Every routed board of one chain run into the residuals; returns
        the shipped pair and its grade."""
        pairs = arm_boards(stem, K)
        n_obs = 0
        for fo, rb in pairs:
            try:
                n_obs += observe(seen, fo, rb, obs, src_of)
            except Exception as e:      # a board that cannot be read teaches nothing
                log(f'    {os.path.basename(rb)}: not observed ({type(e).__name__}: {str(e)[:80]})')
        F, R = winner_pair(stem, K, pairs)
        g, line = (grade(R, nets_csv) if R else (None, 'no routed board'))
        return F, R, g, line, n_obs, len(pairs)

    start = 1
    if RESUME and os.path.exists(ledger_path):
        ledger = json.load(open(ledger_path))
        for (nm, end, d, L, x) in ledger.get('obs', []):
            obs[(nm, end, d, L)].append(x)
        tried = {tuple(t) for t in ledger.get('tried', [])}
        tried_kind = {tuple(t) for t in ledger.get('tried_kind', [])}
        inc = ledger.get('incumbent')
        start = len(ledger['rounds']) + 1
        log(f'  resumed: {len(ledger["rounds"])} round(s) done, incumbent {fmt_g(inc["grade"])} '
            f'from {os.path.basename(inc["R"])}, {sum(len(v) for v in obs.values())} residual(s)')
    else:
        # ---- round 0
        t0 = time.time()
        if OPTS.get('from'):
            stem = OPTS['from']
            if '/' not in stem:
                stem = os.path.join('tmp', stem)
            stem = os.path.join(HERE, stem) if not os.path.isabs(stem) else stem
            log(f'\n=== round 0: the recorded run {stem}')
        else:
            stem = os.path.join(root, 'r0', 'c0')
            log(f'\n=== round 0: the chain, {os.path.relpath(stem, HERE)}')
            rc, secs = run_chain(os.path.relpath(stem, HERE), K, None, chain_env)
            log(f'  chain rc {rc} ({secs} s)')
        F, R, g, line, n_obs, n_pairs = absorb(stem)
        if g is None:
            sys.exit(f'round 0 did not grade: {line}')
        inc = {'F': F, 'R': R, 'grade': g, 'name': 'r0'}
        keep_incumbent(os.path.join(root, 'best'), K, F, R)
        log(f'  incumbent: {fmt_g(g)} <- {os.path.basename(R)} ({n_pairs} routed board(s) seen, '
            f'{n_obs} residual(s), {time.time() - t0:.0f} s)')
        ledger['rounds'].append({'round': 0, 'cands': [{'name': 'r0', 'grade': g, 'R': R, 'F': F}],
                                 'kept': 'r0'})
        ledger['incumbent'] = inc

    since = 0
    for rnd in range(start, ROUNDS + 1):
        t_r = time.time()
        log(f'\n=== round {rnd}: verdict off {os.path.basename(inc["R"])} ({fmt_g(inc["grade"])})')
        B, pred, bp = seen.board(inc['F'])
        real = real_vias(inc['R'], B)
        V = verdict_of(inc['R'])
        refused = refused_on(inc['R'])
        worst = worst_list(B, V, real, MIN_VIAS)
        nsw = sum(1 for nm in names if V.get(nm, {}).get('cls') == 'swim')
        log(f'  incumbent: plan predicted {sum(pred.values()):.0f} vias over {len(pred)} nets, '
            f'routed {sum(real.values())}, {nsw} swimmer(s), refused {sorted(refused) or "none"}; '
            f'residuals: {sum(len(v) for v in obs.values())} over {len(obs)} (net, end, class)')
        log('  worst: ' + '; '.join(f'{nm} {cls} lane {lane}' + (' REFUSED' if ref else '')
                                    + f' [{"/".join(B.cls(nm, "dst") or ("?",))} <- {"/".join(B.cls(nm, "src") or ("?",))}]'
                                    for nm, ref, lane, cls in worst[:max(W * 2, 6)]))
        # ---- the round's base: the incumbent's source view, or the bench
        env_extra = dict(chain_env)
        if BASE_MODE == 'incumbent':
            base_path = os.path.join(root, f'best_src_k{K}.kicad_pcb')
            replan.source_view(inc['F'], base_path, names, B.byname, dref)
            env_extra['BASE'] = base_path
            base_teeth = {nm: (tuple(B.ends[nm]['src']['tooth']), B.ends[nm]['src']['layer'])
                          for nm in names if B.ends[nm]['src']}
            log(f'  base: the incumbent\'s source view {os.path.basename(base_path)} '
                f'({len(base_teeth)} teeth standing as the incumbent laid them)')
        else:
            base_teeth = bench_teeth
        prices = prices_from(obs, GAIN)
        hint, n_src = hint_from(B, base_teeth)
        n_pr = sum(len(v) for d in prices.values() for v in d.values())
        big = sorted(((p, nm, e, c) for nm, d in prices.items() for e, v in d.items() for c, p in v.items()),
                     key=lambda t: -abs(t[0]))[:8]
        log(f'  prices: {n_pr} class(es) over {len(prices)} net(s); largest: '
            + ', '.join(f'{nm}.{e} {c} {p:+.1f}' for p, nm, e, c in big))
        log(f'  hint: {len(hint)} berth(s), {n_src} moved tooth/teeth')
        # ---- the candidates
        specs = []
        for cand in CANDS:
            base, wide, _o = kind_of(cand)
            bans, hold, picked, crossers, radius, reach, reach_k = arm_spec(cand, B, bp, worst, W, tried, tried_kind,
                                                                   refused, CROSSERS, RADIUS, REACH)
            if base not in ('radius', 'none') and not picked and not refused:
                log(f'  {cand}: nothing left to try (every worst net tried under this kind) -- skipped')
                continue
            fb = {'bans': bans, 'hold': hold, 'radius': radius, 'accept_laid': True, 'reach': reach,
                  'prices': {} if base == 'noprice' else prices,
                  'hint': {} if base == 'nohint' else hint}
            rdir = os.path.join(root, f'r{rnd}')
            os.makedirs(rdir, exist_ok=True)
            fpath = os.path.join(rdir, f'{cand}.feedback.json')
            with open(fpath, 'w', encoding='utf-8') as f:
                json.dump(fb, f, indent=1, sort_keys=True)
            specs.append({'name': cand, 'kind': base, 'stem': os.path.join(rdir, cand), 'feedback': fpath,
                          'bans': bans, 'picked': picked, 'crossers': crossers})
            log(f'  {cand}: free {picked}' + (f' + crossers {crossers}' if crossers else '')
                + '; bans ' + (', '.join(f'{nm}.{e} {"/".join(b[:2])}' for nm, d in bans.items()
                                         for e, bl in d.items() for b in bl) or 'none')
                + f'; held {len(hold)} net(s)' + (f'; radius {radius}' if radius else '')
                + (f'; reach {reach_k} on {len(reach)} net(s): '
                   + ', '.join(f'{nm}.{e} [{w[0]}..{w[1]}]' for nm, d in sorted(reach.items())
                               for e, w in d.items() if nm in picked or nm in crossers) if reach else ''))
        if not specs:
            log(f'  round {rnd}: no candidate -- stopping')
            break
        log(f'  running {len(specs)} candidate chain(s), {JOBS} at a time ...')

        def _run(sp):
            rc, secs = run_chain(os.path.relpath(sp['stem'], HERE), K, sp['feedback'], env_extra)
            return sp['name'], rc, secs
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, JOBS)) as ex:
            for name, rc, secs in ex.map(_run, specs):
                log(f'    {name}: chain rc {rc} ({secs} s)')
        # ---- grade, absorb, judge
        results = []
        for sp in specs:
            F, R, g, line, n_obs, n_pairs = absorb(sp['stem'])
            fo_log = sp['stem'] + f'_fo_k{K}.log'
            fbl = ''
            if os.path.exists(fo_log):
                mm = re.findall(r'feedback (?:hint|hold|trust region|reach): .*|plan feedback \[.*',
                                open(fo_log, encoding='utf-8').read())
                fbl = ' | '.join(sorted(set(m.strip() for m in mm)))[:400]
            log(f'  {sp["name"]:8s} {fmt_g(g)} <- {os.path.basename(R) if R else "-"} '
                f'({n_pairs} board(s), {n_obs} residual(s))' + (f'\n      planner: {fbl}' if fbl else ''))
            results.append((sp, F, R, g))
        graded = [t for t in results if t[3] is not None]
        graded.sort(key=lambda t: (len(t[3][0]), t[3][1] != 0, t[3][2]))
        kept = None
        if graded and better(graded[0][3], inc['grade'], MARGIN):
            sp, F, R, g = graded[0]
            inc = {'F': F, 'R': R, 'grade': g, 'name': f'r{rnd}/{sp["name"]}'}
            keep_incumbent(os.path.join(root, 'best'), K, F, R)
            kept = sp['name']
            since = 0
        else:
            since += 1
        for sp, F, R, g in results:
            if sp['name'] != kept:
                tried |= ban_keys(sp['bans'])
                tried_kind |= {(sp['kind'], nm) for nm in sp['picked']}
        log(f'  round {rnd}: ' + (f'KEPT {kept} -> incumbent {fmt_g(inc["grade"])}' if kept else
                                 f'rejected -- best candidate {fmt_g(graded[0][3]) if graded else "none"} '
                                 f'vs incumbent {fmt_g(inc["grade"])}')
            + f'; {len(tried)} retired ban(s), {len(tried_kind)} retired hypothesis(es) '
            f'({time.time() - t_r:.0f} s)')
        ledger['rounds'].append({'round': rnd, 'kept': kept,
                                 'cands': [{'name': sp['name'], 'grade': g, 'R': R, 'F': F,
                                            'bans': sp['bans'], 'picked': sp['picked'],
                                            'crossers': sp['crossers']} for sp, F, R, g in results]})
        ledger['incumbent'] = inc
        ledger['obs'] = [[nm, end, d, L, x] for (nm, end, d, L), xs in obs.items() for x in xs]
        ledger['tried'] = sorted(tried)
        ledger['tried_kind'] = sorted(tried_kind)
        with open(ledger_path, 'w', encoding='utf-8') as f:
            json.dump(ledger, f, indent=1)
        if since >= PATIENCE:
            log(f'  {since} round(s) without improvement -- stopping')
            break
    log(f'\nplan_loop: best {fmt_g(inc["grade"])} <- {inc["name"]} '
        f'({os.path.basename(inc["R"])}) -> {os.path.relpath(os.path.join(root, f"best_k{K}.kicad_pcb"), HERE)} '
        f'({time.time() - t_all:.0f} s)')
    for rd in ledger['rounds']:
        log('  round {}: '.format(rd['round']) + '; '.join(
            f'{c["name"]} {fmt_g(c["grade"])}' for c in rd['cands']) + (f'  <- kept {rd["kept"]}' if rd.get('kept') else ''))
    r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'),
                        os.path.join(root, f'best_k{K}.kicad_pcb'), nets_csv],
                       capture_output=True, text=True, cwd=HERE)
    log((r.stdout + r.stderr).strip().splitlines()[-1] if (r.stdout + r.stderr).strip() else 'no grade')
    return 0


if __name__ == '__main__':
    sys.exit(main())
