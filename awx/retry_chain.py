#!/usr/bin/env python3
"""Directed iteration on refusal, at the CHAIN level.

One fanout+braid pass is a single draw from a chaotic system: at
K32/K35 saturation every forced-net knob measured 0831 closed its
target and opened a different marginal pair (q6..q8: SDQ14's B tooth
left it open but closed SA4; its dest-B berth routed it and collapsed
K32 to 9 open). No single arm dominates, so the driver runs the plain
chain first and, for the nets the braid itself REFUSED, retries the
fanout with those nets forced into the two split mechanisms -- source
tooth to B (TP_SRC_B_NETS) and dest berth to B (TP_SPLIT_NETS) -- as
SEPARATE arms, keeping whichever board grades best:

    (open, drc, vias, segs), lexicographic. Completion first.

Rounds after the first extend the current winner's force list with its
own remaining refusals, so the loop is monotone: a board is only ever
replaced by one that grades strictly better, and a round that improves
nothing ends the loop. Within a round the arms CHAIN: `best` updates
as soon as an arm wins, so the next arm builds on it (dest-B on top of
the round's source-B winner). That chaining is load-bearing -- it is
exactly what completed K32 for the first time (rt1, 0831: r0 4 open ->
sB 3 -> sB2 2 -> dB2 on top of sB2 = 0 open / 0 drc / 96 vias).

Env knobs are scoped PER SUBPROCESS (the A/B leak lesson): the driver
never exports the force knobs into its own environment.

usage: retry_chain.py TAG K [--rounds N] [-- fanout extra options]
env (same as chain_k.sh): BASE, DEST, DIRS, PLAN_OPTS, TWO_PAGE,
TP_SCOPE -- the caller sets the arm; the driver adds only the
per-round force knobs.
"""
import ast
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def sh(argv, env_extra=None, log_path=None):
    env = os.environ.copy()
    if env_extra:
        env.update(env_extra)
    r = subprocess.run(argv, capture_output=True, text=True, env=env,
                       cwd=HERE)
    if log_path:
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write(r.stdout + r.stderr)
    return r


def main():
    args = [a for a in sys.argv[1:]]
    fo_extra = []
    if '--' in args:
        i = args.index('--')
        fo_extra = args[i + 1:]
        args = args[:i]
    rounds = 2
    if '--rounds' in args:
        i = args.index('--rounds')
        rounds = int(args[i + 1])
        del args[i:i + 2]
    tag, K = args[0], args[1]
    base = os.environ.get('BASE', 'fb_t2q_base.kicad_pcb')
    dest = os.environ.get('DEST', 'DU1')
    # DIRS is opt-in exactly as in chain_k.sh: the recorded arm runs
    # UNRESTRICTED (general costs steer the faces); a left,down
    # default here was a leftover board-specific restriction
    dirs = os.environ.get('DIRS', '')
    plan_opts = os.environ.get('PLAN_OPTS', '--two-page').split()
    nets = sh([sys.executable, 'coherent_nets.py', K]).stdout.strip()

    def run_arm(atag, force_env):
        """fanout + braid + grade under this arm's env. Returns
        (grade tuple or None, refused list, board path)."""
        fo = f'{atag}_fo_k{K}.kicad_pcb'
        sh([sys.executable, 'fanout_from_plan.py', fo, K,
            f'--board={base}',
            *([f'--dirs={dirs}'] if dirs else []), '--no-lines',
            *plan_opts, *fo_extra],
           env_extra=force_env, log_path=f'{atag}_fo_k{K}.log')
        if not os.path.exists(os.path.join(HERE, fo)):
            print(f'  {atag}: NO FANOUT BOARD')
            return None, [], None
        board = f'{atag}_k{K}.kicad_pcb'
        sh([sys.executable, '-u', 'braid.py', '--board', fo,
            '--dest', dest, '--nets', nets, '--out', f'{atag}_k{K}'],
           log_path=f'{atag}_k{K}.log')
        if not os.path.exists(os.path.join(HERE, board)):
            print(f'  {atag}: NO BRAID BOARD')
            return None, [], None
        g = sh([sys.executable, 'grade_k.py', board, nets])
        m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+) segs=(\d+)',
                      g.stdout)
        if not m:
            print(f'  {atag}: NO GRADE ({g.stdout.strip()[:80]})')
            return None, [], None
        grade = tuple(int(x) for x in m.groups())
        refused = []
        try:
            txt = open(os.path.join(HERE, f'{atag}_k{K}.log'),
                       encoding='utf-8').read()
            mm = re.search(r'REFUSED nets \(left open\): (\[[^\]]*\])',
                           txt)
            if mm:
                refused = ast.literal_eval(mm.group(1))
        except OSError:
            pass
        print(f'  {atag}: open={grade[0]} drc={grade[1]} '
              f'vias={grade[2]} segs={grade[3]}'
              + (f'  refused: {",".join(refused)}' if refused else ''))
        return grade, refused, board

    print(f'retry_chain K={K}: round 0 (plain arm)')
    g0, refused, b0 = run_arm(f'{tag}r0', {})
    if g0 is None:
        sys.exit(2)
    best = (g0, b0, {}, refused, f'{tag}r0')
    for rnd in range(1, rounds + 1):
        if best[0][0] == 0 and best[0][1] == 0:
            break                        # complete and clean: done
        if not best[3]:
            break                        # nothing named to force
        force = ','.join(sorted(best[3]))
        print(f'round {rnd}: forcing refused [{force}]')
        improved = False
        for kind, knob in (('sB', 'TP_SRC_B_NETS'),
                           ('dB', 'TP_SPLIT_NETS')):
            env = dict(best[2])
            env['TP_SRC_B'] = '1'
            prev = env.get(knob, '')
            env[knob] = ','.join(sorted(set(
                ([x for x in prev.split(',') if x]) + list(best[3]))))
            g, ref, b = run_arm(f'{tag}{kind}{rnd}', env)
            if g is not None and g < best[0]:
                best = (g, b, env, ref, f'{tag}{kind}{rnd}')
                improved = True
        if not improved:
            break
    g, b, env, refused, wtag = best
    out = f'{tag}_k{K}.kicad_pcb'
    if b != out:
        shutil.copy(os.path.join(HERE, b), os.path.join(HERE, out))
        pro = os.path.splitext(b)[0] + '.kicad_pro'
        if os.path.exists(os.path.join(HERE, pro)):
            shutil.copy(os.path.join(HERE, pro),
                        os.path.join(HERE, os.path.splitext(out)[0]
                                     + '.kicad_pro'))
    print(f'KEPT {wtag} -> {out}: open={g[0]} drc={g[1]} vias={g[2]} '
          f'segs={g[3]}'
          + (f'  (env {" ".join(f"{k}={v}" for k, v in env.items())})'
             if env else '  (plain)'))


if __name__ == '__main__':
    main()
