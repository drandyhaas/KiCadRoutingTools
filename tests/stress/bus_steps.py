#!/usr/bin/env python3
"""bus_steps.py -- the BUS STEPS check: each bus of the bus boards check routed ALONE, on the board the buses before it
left in a recorded run (tests/stress/bus_boards.py's OUT), so a change to the bus step is judged bus by bus, every bus
at once, and a later bus's result does not move with an earlier bus's.

  python3 tests/stress/bus_steps.py record RUN_DIR STEPS_DIR
  python3 tests/stress/bus_steps.py run STEPS_DIR [--out DIR] [--cases A,B] [-j N] [--rounds R] [--after RUN_OUT]
                                       [--set K=V ...]
  python3 tests/stress/bus_steps.py replay STEPS_DIR [--out DIR] [--cases A,B] [-j N] [--set K=V ...]

`record`: from a bus boards run (each board's input.kicad_pcb and its buses' boards, BUS.bus/SRC-DEST.kicad_pcb), each
bus a STEP: the board its turn started from (the case's input, else the board the bus before it handed on), and what
route_bus gave it there -- an escape bus on an array an earlier bus went through planned whole with that array
(ESCAPE_JOINT), its ends on the case's input (ESCAPE_JOINT_ENDS), the later buses' nets (ESCAPE_JOINT_LATER) -- copied
into STEPS_DIR with steps.json; each step's recorded work dir (BUS.bus/SRC-DEST.bus) named for `replay`.

`run`: every step routed by route_bus (in process, --rounds R, the case's sizes) on its own board, -j at a time, each
graded as bus_boards grades a board: every bus net's pads on the bus's parts one piece of copper on the step's output.
Exit 0 when no step leaves a bus net open. Use it for a change to the fanout, the ends or anything the whole step runs.

`replay`: every step's recorded ROUND again (awx/replay_round.py: its fanout as recorded -- and with --keep-solve its
first solve -- the rest by this tree), its round-1 audit and its laid open count beside the recorded run's own (WORSE
where more). Minutes where `run` is an hour -- a change to the solve, the loop, the geometry, the polish or the lay; a
change to the fanout or the ends is `run`'s.

Not collected by tests/run_all.py (routing). Steps run as one process each: keep -j within the machine's routing
budget."""
import argparse
import concurrent.futures
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
FIX = os.path.join(ROOT, 'tests', 'fixtures', 'bus_boards')
SIBS = ('.kicad_pcb', '.kicad_pro', '.kicad_dru')
SIZE_FLAGS = {'--clearance-ceiling': 'clearance_ceiling', '--track-width': 'track_width',
              '--fanout-track-width': 'fanout_track_width', '--via-size': 'via_size', '--via-drill': 'via_drill'}


def _copy_board(src, dst_stem):
    """`src` and its siblings copied to `dst_stem` + their extensions"""
    s_ = src[:-len('.kicad_pcb')]
    for ext in SIBS:
        if os.path.isfile(s_ + ext):
            shutil.copy2(s_ + ext, dst_stem + ext)
    return dst_stem + '.kicad_pcb'


def parse_args(args):
    """(buses [(src, dest, mode, layers)], sizes {route_bus keyword: value}) of a manifest case's arguments"""
    buses, sizes, i = [], {}, 0
    while i < len(args):
        if args[i] == '--bus':
            p = args[i + 1].split(':')
            buses.append((p[0], p[1], p[2] if len(p) > 2 else 'plain', p[3] if len(p) > 3 else None))
            i += 2
        elif args[i] in SIZE_FLAGS:
            sizes[SIZE_FLAGS[args[i]]] = float(args[i + 1])
            i += 2
        else:
            i += 1
    return buses, sizes


def record(run_dir, steps_dir):
    man = json.load(open(os.path.join(FIX, 'manifest.json'), encoding='utf-8'))['cases']
    os.makedirs(steps_dir, exist_ok=True)
    steps = []
    for case in sorted(man):
        d = os.path.join(run_dir, case)
        buses, sizes = parse_args(man[case]['args'])
        if not os.path.isfile(os.path.join(d, 'input.kicad_pcb')):
            if len(buses) != 1:
                print(f'bus_steps: {case} is not in {run_dir} -- its steps after the first need that run: left out')
                continue
            # (a board of one bus: its one step's board is the fixture's input itself, and no round is recorded)
            sys.path.insert(0, HERE)
            import bus_boards
            d, _b = bus_boards.stage(case, man[case], os.path.join(steps_dir, '_fixtures'))
        cur = os.path.join(d, 'input.kicad_pcb')
        through = {}                    # (a part: the first step that went through it)
        inputs = []
        for k, (s, dst, mode, layers) in enumerate(buses):
            name = f'{case}-{k + 1}-{s}-{dst}'
            sd = os.path.join(steps_dir, name)
            os.makedirs(sd, exist_ok=True)
            board = _copy_board(cur, os.path.join(sd, 'cur'))
            settings = {'ROUTE_LAYERS': layers} if layers else {}
            parts = [s] + dst.split('+')
            shared = [r for r in parts if r in through]
            if mode == 'escape' and shared:
                # (route_bus.route_buses' sequential step: the shared array planned whole with this bus, its ends on
                # the board before the first earlier bus through it, the later buses' nets steered to their parts)
                ends = _copy_board(inputs[min(through[r] for r in shared)], os.path.join(sd, 'ends'))
                settings.update(ESCAPE_JOINT=','.join(shared), ESCAPE_JOINT_ENDS=os.path.basename(ends))
                later = os.path.join(d, 'bus.bus', f'{s}-{dst}.later.json')
                if os.path.isfile(later):
                    shutil.copy2(later, os.path.join(sd, 'later.json'))
                    settings['ESCAPE_JOINT_LATER'] = 'later.json'
            inputs.append(cur)
            for r in parts:
                through.setdefault(r, k)
            out_ = os.path.join(d, 'bus.bus', f'{s}-{dst}.kicad_pcb')
            work = os.path.join(d, 'bus.bus', f'{s}-{dst}.bus')
            steps.append(dict(name=name, case=case, src=s, dest=dst, mode=mode, layers=layers, sizes=sizes,
                              settings=settings, board=os.path.basename(board),
                              work=work if os.path.isdir(os.path.join(work, 'run', 'r1')) else None))
            if os.path.isfile(out_):
                cur = out_
    with open(os.path.join(steps_dir, 'steps.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(run=os.path.abspath(run_dir), steps=steps), f, indent=1)
    print(f'bus_steps: {len(steps)} step(s) from {run_dir} into {steps_dir}: ' + ', '.join(s_['name'] for s_ in steps))


def one(step_dir, out_dir, rounds):
    """(the child) one step routed in this process"""
    st = json.load(open(os.path.join(step_dir, 'step.json'), encoding='utf-8'))
    sys.path.insert(0, os.path.join(ROOT, 'awx'))
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    os.chdir(os.path.join(ROOT, 'awx'))
    import route_bus as rb
    sets = {k: (os.path.join(step_dir, v) if k in ('ESCAPE_JOINT_ENDS', 'ESCAPE_JOINT_LATER') else v)
            for k, v in st['settings'].items()}
    sets.update(st.get('extra') or {})
    board = os.path.join(out_dir, 'cur.kicad_pcb')
    out = os.path.join(out_dir, 'bus.kicad_pcb')
    code, g = rb.route_bus(board, out, st['src'], st['dest'], None, rounds, True, print,
                           json_out=os.path.join(out_dir, 'bus.summary.json'),
                           whole_array=st['mode'] == 'whole-array', escape=st['mode'] == 'escape',
                           settings=sets or None, **st['sizes'])
    print('bus_steps: exit', code, g, flush=True)
    return 0


def _select(steps, cases):
    want = [c for c in (cases or '').split(',') if c]
    return [s_ for s_ in steps if not want or s_['case'] in want or s_['name'] in want]


def run_one(st, steps_dir, out, rounds, sets, timeout, after=None, prev=None):
    sd = os.path.join(steps_dir, st['name'])
    od = os.path.join(out, st['name'])
    os.makedirs(od, exist_ok=True)
    # (`after`: the board the step before left in that run, where it left one -- the step on this tree's earlier bus)
    prior = os.path.join(after, prev, 'bus.kicad_pcb') if after and prev else None
    _copy_board(prior if prior and os.path.isfile(prior) else os.path.join(sd, st['board']), os.path.join(od, 'cur'))
    with open(os.path.join(sd, 'step.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(st, extra=sets), f, indent=1)
    cmd = [sys.executable, '-u', '-X', 'utf8', os.path.abspath(__file__), '_one', sd, od, str(rounds)]
    t0 = time.time()
    with open(os.path.join(od, 'run.log'), 'w', encoding='utf-8') as log:
        try:
            rc = subprocess.run(cmd, env=dict(os.environ, **sets), stdout=log, stderr=subprocess.STDOUT,
                                timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            rc = 'timeout'
    sys.path.insert(0, HERE)
    import bus_boards
    g = bus_boards.grade_board(os.path.join(od, 'bus.kicad_pcb'), [(os.path.join(od, 'bus.summary.json'),
                                                                     dict(mode=st['mode']))])
    return dict(exit=rc, secs=round(time.time() - t0), **g)


AUDIT = re.compile(r'smooth: audit: dive (\d+), static (\d+), shape (\d+), swim (\d+), band [\d.]+ mm, pitch (\d+)')


def replay_one(st, out, sets, timeout, keep_solve=False):
    od = os.path.join(out, st['name'])
    if not st.get('work'):
        return dict(why='no recorded round')
    work = os.path.join(od, 'work')
    if os.path.isdir(work):
        shutil.rmtree(work)
    # (the recorded work dir copied: replay_round reads it, and the record stays as it was)
    shutil.copytree(st['work'], work, ignore=shutil.ignore_patterns('loop', 'loop_ends', 'r2', 'r3', 'r4'))
    summ = os.path.join(os.path.dirname(st['work']), os.path.basename(st['work'])[:-len('.bus')] + '.summary.json')
    k_ = rec_ = None
    try:
        g_ = (json.load(open(summ, encoding='utf-8')).get('grade') or '').split(' | ')[0]
        m_ = re.search(r'K=(\d+)', g_)
        k_ = int(m_.group(1)) if m_ else None
        m_ = re.search(r'open=(\d+)', g_)
        rec_ = int(m_.group(1)) if m_ else None        # (the recorded run's own count for the step)
    except (OSError, ValueError):
        pass
    if k_ is None:
        k_ = sum(1 for ln in open(os.path.join(work, 'run', 'r1', 'nets.lines')) if ln.strip())
    sz = st['sizes']
    sizes = ','.join(str(sz[k]) for k in ('clearance_ceiling', 'track_width', 'fanout_track_width', 'via_size',
                                           'via_drill'))
    r1 = os.path.join(work, 'run', 'r1')
    extra = []
    if st['mode'] == 'escape':
        extra += ['--set', 'FANOUT_ESCAPE=1']
        sd = os.path.dirname(os.path.abspath(st['_steps_json']))
        for k, v in st['settings'].items():
            if k.startswith('ESCAPE_JOINT'):
                extra += ['--set', f"{k}={os.path.join(sd, st['name'], v) if k != 'ESCAPE_JOINT' else v}"]
        for k, f_ in (('ESCAPE_JOINT_SPEC', 'escape_joint.json'), ('ESCAPE_ENDS_BASE', 'ends_frame.kicad_pcb')):
            if os.path.isfile(os.path.join(work, f_)):
                extra += ['--set', f'{k}={os.path.join(work, f_)}']
    for k, v in sets.items():
        extra += ['--set', f'{k}={v}']
    cmd = [sys.executable, '-u', '-X', 'utf8', os.path.join(ROOT, 'awx', 'replay_round.py'), work,
           os.path.join(od, 'replay'), '--sizes', sizes, '--k', str(k_), '--dest', st['dest'],
           '--fanout-from', r1] + (['--solve-from', r1] if keep_solve else []) \
        + (['--layers', st['layers']] if st['layers'] else []) + extra
    t0 = time.time()
    log_p = os.path.join(od, 'replay.log')
    with open(log_p, 'w', encoding='utf-8') as log:
        log.write(' '.join(cmd) + '\n')
        log.flush()
        try:
            rc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            rc = 'timeout'
    txt = open(log_p, encoding='utf-8', errors='replace').read()
    au = AUDIT.search(txt)
    op = re.findall(r'replay_round: chain exit \S+; WHOLE .*?open=(\d+)', txt)
    return dict(exit=rc, secs=round(time.time() - t0), audit=au.groups() if au else None,
                open=int(op[-1]) if op else None, passed='loop exit 0' in txt, recorded=rec_)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '_one':
        sys.exit(one(sys.argv[2], sys.argv[3], int(sys.argv[4])))
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r_ = sub.add_parser('record', help='each step of a bus boards run, recorded')
    r_.add_argument('run_dir')
    r_.add_argument('steps_dir')
    for name in ('run', 'replay'):
        p_ = sub.add_parser(name)
        p_.add_argument('steps_dir')
        p_.add_argument('--out', default=None, help="the runs' directory (default: a new temporary one, kept)")
        p_.add_argument('--cases', default=None, help='the boards or steps to take, comma separated (default: all)')
        p_.add_argument('-j', type=int, default=6, help='steps at once (default 6)')
        p_.add_argument('--set', action='append', default=[], metavar='K=V', help='a setting for every step')
        p_.add_argument('--timeout', type=float, default=3 * 3600, help='seconds a step may take (default 3 h)')
        if name == 'run':
            p_.add_argument('--rounds', type=int, default=1, help='route_bus --rounds (default 1)')
            p_.add_argument('--after', default=None, metavar='RUN_OUT',
                            help="each step on the board the step before it left in that `run` (where it left one), "
                                 "not its recorded one: a later bus on this tree's earlier bus")
        else:
            p_.add_argument('--keep-solve', action='store_true',
                            help="the round's first solve as recorded too (default: solved again on the recorded fanout "
                                 "-- a recorded solve meets this tree's frame, and a re-solve on a frame changed since "
                                 "can find no plan its warm start had)")
    a = ap.parse_args()
    if a.cmd == 'record':
        record(os.path.abspath(a.run_dir), os.path.abspath(a.steps_dir))
        return
    sj = os.path.join(os.path.abspath(a.steps_dir), 'steps.json')
    steps = _select(json.load(open(sj, encoding='utf-8'))['steps'], a.cases)
    for s_ in steps:
        s_['_steps_json'] = sj
    sets = dict(kv.partition('=')[::2] for kv in a.set)
    out = os.path.abspath(a.out) if a.out else tempfile.mkdtemp(prefix=f'bus_steps_{a.cmd}_')
    os.makedirs(out, exist_ok=True)
    print(f'bus_steps {a.cmd}: {len(steps)} step(s) into {out}, {a.j} at once'
          + (f', {" ".join(f"{k}={v}" for k, v in sorted(sets.items()))}' if sets else ''), flush=True)
    rows = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, a.j)) as ex:
        if a.cmd == 'run':
            allsteps = json.load(open(sj, encoding='utf-8'))['steps']
            prev_of = {s_['name']: p_['name'] for p_, s_ in zip(allsteps, allsteps[1:]) if p_['case'] == s_['case']}
            futs = {ex.submit(run_one, s_, os.path.abspath(a.steps_dir), out, a.rounds, sets, a.timeout,
                              os.path.abspath(a.after) if a.after else None, prev_of.get(s_['name'])): s_['name']
                    for s_ in steps}
        else:
            futs = {ex.submit(replay_one, s_, out, sets, a.timeout, a.keep_solve): s_['name'] for s_ in steps}
        for f in concurrent.futures.as_completed(futs):
            n = futs[f]
            try:
                rows[n] = f.result()
            except Exception as e:                      # (one step's trouble is its row, not the whole check's)
                rows[n] = dict(why=f'{type(e).__name__}: {e}')
            r = rows[n]
            if a.cmd == 'run':
                print(f'  {n}: exit {r.get("exit")}, {r.get("secs")} s -- {r.get("nets", 0)} bus net(s), '
                      f'{len(r.get("open") or [])} open' + (f' ({", ".join(r["open"][:8])})' if r.get('open') else '')
                      + (f' [{r["why"]}]' if r.get('why') else ''), flush=True)
            else:
                print(f'  {n}: exit {r.get("exit")}, {r.get("secs")} s -- round 1 audit dive/static/shape/swim/pitch '
                      f'{"/".join(r["audit"]) if r.get("audit") else "-"}, loop {"passed" if r.get("passed") else "not passed"}'
                      f', laid open {r.get("open")}' + (f' [{r["why"]}]' if r.get('why') else ''), flush=True)
    ok = True
    print()
    for s_ in steps:
        r = rows.get(s_['name']) or {}
        if a.cmd == 'run':
            fail = bool(r.get('open')) or bool(r.get('why')) or r.get('exit') not in (0, 1)
            print(f'{s_["name"]:28s} {r.get("nets", 0):5d} nets {len(r.get("open") or []):4d} open {r.get("secs", 0):6} s'
                  + ('  FAIL' if fail else ''))
        else:
            if r.get('why') == 'no recorded round':
                print(f'{s_["name"]:28s} (no recorded round: run it with `run`)')
                continue
            fail = r.get('open') not in (0,) or bool(r.get('why'))
            print(f'{s_["name"]:28s} audit {"/".join(r["audit"]) if r.get("audit") else "-":14s} '
                  f'{"passed" if r.get("passed") else "      "} open {r.get("open")} (recorded '
                  f'{r.get("recorded")}) {r.get("secs", 0):6} s'
                  + ('  WORSE' if (r.get('open') or 0) > (r.get('recorded') or 0) else '') + ('  FAIL' if fail else ''))
        ok &= not fail
    with open(os.path.join(out, 'report.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(cmd=a.cmd, sets=sets, steps=rows), f, indent=1)
    print(f'\nbus_steps {a.cmd}: {"every step clean" if ok else "OPEN nets or failed steps"} -- '
          f'{os.path.join(out, "report.json")}')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
