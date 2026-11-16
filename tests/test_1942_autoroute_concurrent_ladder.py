"""#1942: autoroute runs a fan-out ladder's rungs side by side, same selection.

`fan_ladder` ran a part's rungs one after another from the same board until
one escaped every ball with no graze. The rungs are independent runs, so they
are now started side by side and read in ladder order: the rung kept, the step
numbers, the files left and the lines printed are those of the one-at-a-time
ladder, which `KRT_AUTOROUTE_JOBS=1` still runs.

  1. scripted rungs (a stand-in tool with a set delay and outcome), every
     ladder shape, at 1, 2 and 3 jobs: the same rung kept, the same steps,
     the same lines in the same order, the same files as one at a time;
  2. the rungs overlap at 3 jobs and do not at 1 (the stand-in's own clock);
  3. a rung the ladder does not read is ended and reaped, leaves no file, and
     at 2 jobs a rung past a clean one is never started;
  4. a ladder of one rung goes through `Chain.run`, as before;
  5. the count: KRT_AUTOROUTE_JOBS, the cores, the memory; autoroute.py
     refuses a count that is not one;
  6. an in-repo board, the chain's own fan-out steps with the real tool at 3
     jobs against 1: the same rung, copper, lines and files where the first
     rung is clean (the second, started ahead, leaves nothing);
  7. and where the first rung fails and the second is needed and kept.
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
import autoroute as AP  # noqa: E402

FAILS = []
ENV = AP.LADDER_JOBS_ENV
#: An in-repo board with a part whose ladder has two rungs, cheap to fan out.
BOARD, PART = 'tigard.kicad_pcb', 'U3'


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


# A fan-out stand-in: waits, then ends as scripted. It writes when it started
# and ended, and its pid, outside the chain's folder.
STUB = r'''
import json, os, shutil, sys, time
src, out, notes, delay, escaped, failed, grazes, how = sys.argv[1:9]
name = os.path.basename(out)
with open(os.path.join(notes, name + '.start'), 'w') as fh:
    fh.write(f"{os.getpid()} {time.time()}")
time.sleep(float(delay))
if how == 'fail':
    print('scripted failure', file=sys.stderr)
    sys.exit(3)
shutil.copy(src, out)
with open(out[:-len('.kicad_pcb')] + '.kicad_pro', 'w') as fh:
    fh.write('{}')
if how != 'silent':
    print('JSON_SUMMARY: ' + json.dumps({
        'requested': int(escaped) + int(failed), 'escaped': int(escaped),
        'failed': int(failed), 'total_iterations': 7,
        'drc_grazes': {'total': int(grazes)}}))
with open(os.path.join(notes, name + '.end'), 'w') as fh:
    fh.write(str(time.time()))
'''

#: Ladder shapes: (delay s, escaped, dropped, grazes, how) per rung, and the
#: rung the one-at-a-time ladder keeps.
SHAPES = {
    'clean first, the rest slow': ([(0.1, 9, 0, 0, 'ok'), (30, 9, 0, 0, 'ok'),
                                    (30, 9, 0, 0, 'ok')], 0),
    'clean third, first still running': ([(0.9, 7, 2, 0, 'ok'), (0.5, 8, 1, 0, 'ok'),
                                          (0.1, 9, 0, 0, 'ok')], 2),
    'clean second ends first, third slow': ([(0.7, 7, 2, 0, 'ok'), (0.1, 9, 0, 0, 'ok'),
                                             (30, 9, 0, 0, 'ok')], 1),
    'none clean, equals end in reverse': ([(0.7, 8, 1, 2, 'ok'), (0.4, 8, 1, 2, 'ok'),
                                           (0.1, 8, 1, 2, 'ok')], 0),
    'none clean, fewest grazes last': ([(0.5, 8, 1, 3, 'ok'), (0.1, 8, 1, 5, 'ok'),
                                        (0.3, 8, 1, 1, 'ok')], 2),
    'more escapes beat fewer grazes': ([(0.1, 6, 3, 0, 'ok'), (0.5, 8, 1, 4, 'ok'),
                                        (0.3, 7, 2, 0, 'ok')], 1),
    'first rung fails, second clean': ([(0.5, 0, 0, 0, 'fail'), (0.1, 9, 0, 0, 'ok'),
                                        (30, 9, 0, 0, 'ok')], 1),
    'a rung prints no summary': ([(0.4, 0, 0, 0, 'silent'), (0.1, 8, 1, 0, 'ok'),
                                  (0.2, 8, 1, 0, 'ok')], 1),
    'every rung fails': ([(0.3, 0, 0, 0, 'fail'), (0.1, 0, 0, 0, 'fail')], None),
}


def alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def lines_of(text, root):
    """The lines a ladder printed, less what differs between any two runs:
    the clock, the seconds, and the folder."""
    out = []
    for ln in text.replace(root, '<work>').splitlines():
        m = re.match(r'^(AUTOROUTE_\w+): (\{.*\})$', ln)
        if m:
            d = json.loads(m.group(2))
            d.pop('t', None)
            d.pop('seconds', None)
            ln = m.group(1) + ': ' + json.dumps(d, sort_keys=True)
        out.append(ln)
    return out


def ladder(td, name, rows, jobs, **kw):
    """Run the scripted ladder ROWS at JOBS; everything a reader could see."""
    work = os.path.join(td, f'w{len(os.listdir(td))}')
    notes = work + '.notes'
    os.makedirs(work)
    os.makedirs(notes)
    src = os.path.join(work, 'input.kicad_pcb')
    with open(src, 'w') as fh:
        fh.write('(kicad_pcb)\n')
    ch = AP.Chain(src, work)
    ch.n = 4                            # a ladder in the middle of a chain
    fan = {'ref': 'U1', 'tool': 'bga',
           'ladder': [['{IN}', '{OUT}', notes] + [str(x) for x in row] for row in rows]}
    old = os.environ.get(ENV)
    os.environ[ENV] = str(jobs)
    buf = io.StringIO()
    t0 = time.monotonic()
    try:
        with contextlib.redirect_stdout(buf):
            best = AP.fan_ladder(ch, fan, **kw)
            ch.end_step()
    finally:
        if old is None:
            del os.environ[ENV]
        else:
            os.environ[ENV] = old
    wall = time.monotonic() - t0
    marks = {}
    for f in os.listdir(notes):
        stem, kind = f.rsplit('.', 1)
        val = open(os.path.join(notes, f)).read().split()
        marks.setdefault(stem, {})[kind] = [float(x) for x in val]
    seen = {'best': best and dict(best, out=os.path.basename(best['out'])),
            'n': ch.n, 'iterations': ch.last_iterations,
            'steps': [{k: v for k, v in s.items() if k != 'seconds'} for s in ch.steps],
            'lines': lines_of(buf.getvalue(), work), 'files': sorted(os.listdir(work)),
            'started': sorted(ch.started)}
    seen = json.loads(json.dumps(seen).replace(work, '<work>'))
    return seen, {'wall': wall, 'marks': marks, 'work': work}


def same(a, b):
    return [k for k in a if json.dumps(a[k], sort_keys=True, default=str)
            != json.dumps(b[k], sort_keys=True, default=str)]


def test_scripted(td):
    print('1. scripted ladders at 1, 2 and 3 jobs')
    for name, (rows, kept) in SHAPES.items():
        one, t1 = ladder(td, name, rows, 1)
        ok((one['best'] and one['best']['rung']) == kept,
           f"{name}: one at a time keeps rung {kept}: {one['best']}")
        for jobs in (2, 3):
            got, t = ladder(td, name, rows, jobs)
            diff = same(one, got)
            ok(not diff, f"{name}: {jobs} jobs is the one-at-a-time ladder"
               + (f" -- differs in {diff}: {[(one[k], got[k]) for k in diff]}"[:1500]
                  if diff else ''))
            pids = [int(m['start'][0]) for m in t['marks'].values()]
            ok(not any(alive(p) for p in pids), f"{name}: {jobs} jobs leaves no child running")
            if rows[-1][0] >= 30:
                ok(t['wall'] < 10, f"{name}: {jobs} jobs did not wait for the slow rung "
                   f"({t['wall']:.1f} s)")
    one, _ = ladder(td, 'lines', SHAPES['clean third, first still running'][0], 1)
    ev = [json.loads(ln.split(': ', 1)[1]) for ln in one['lines']
          if ln.startswith('AUTOROUTE_STEP: ')]
    ok([(e['tag'], e['state']) for e in ev] == [
        ('fanout_U1', 'running'), ('fanout_U1', 'done'),
        ('fanout_U1_r1', 'running'), ('fanout_U1_r1', 'done'),
        ('fanout_U1_r2', 'running'), ('fanout_U1_r2', 'done')] and one['n'] == 7
       and 'step07_fanout_U1_r2.kicad_pcb' in one['files'],
       'the compared lines are a ladder\'s events, in ladder order (non-vacuity): '
       f"{[(e['tag'], e['state']) for e in ev]}")
    held_one, _ = ladder(td, 'held', SHAPES['clean second ends first, third slow'][0], 1,
                         hold=('C1', 'C2'), tag='_held')
    held, _ = ladder(td, 'held', SHAPES['clean second ends first, third slow'][0], 3,
                     hold=('C1', 'C2'), tag='_held')
    ok(not same(held_one, held) and held['best']['out'] == 'step06_fanout_U1_r1_held.kicad_pcb'
       and held['steps'][0]['argv'][-3:] == ['--immovable-refs', 'C1', 'C2'],
       f"a re-fan's ladder holding parts is the same too: {held['best']}")


def test_overlap(td):
    print('2. the rungs overlap at 3 jobs, not at 1')
    rows = SHAPES['none clean, equals end in reverse'][0]
    for jobs, want in ((1, False), (3, True)):
        _, t = ladder(td, 'overlap', rows, jobs)
        m = t['marks']
        r0, r1 = m['step05_fanout_U1.kicad_pcb'], m['step06_fanout_U1_r1.kicad_pcb']
        ok((r1['start'][1] < r0['end'][0]) is want,
           f"{jobs} job(s): rung 1 started {'before' if want else 'after'} rung 0 ended")


def test_unread(td):
    print('3. a rung the ladder does not read')
    rows = SHAPES['clean second ends first, third slow'][0]
    got, t = ladder(td, 'unread', rows, 3)
    third = t['marks'].get('step07_fanout_U1_r2.kicad_pcb')
    ok(third is not None and 'end' not in third,
       'at 3 jobs the third rung was started and never finished (non-vacuity)')
    ok(third is not None and not alive(int(third['start'][0])), 'its process is gone')
    ok(not [f for f in got['files'] if f.startswith('step07_')] and got['n'] == 6,
       f"it left no file and its number is free: {got['files']}, n={got['n']}")
    ok('fanout_U1_r2' not in got['started']
       and not any('fanout_U1_r2' in ln for ln in got['lines']),
       'no line names it and it is not a started step')
    got, t = ladder(td, 'unread2', rows, 2)
    ok('step07_fanout_U1_r2.kicad_pcb' not in t['marks'],
       'at 2 jobs a rung past a clean one is never started')
    # a clean rung that ends while an earlier one runs stops the later ones at once
    rows = [(1.5, 7, 2, 0, 'ok'), (0.1, 9, 0, 0, 'ok'), (30, 9, 0, 0, 'ok')]
    work_before = set(os.listdir(td))
    old, os.environ[ENV] = os.environ.get(ENV), '3'
    try:
        work = os.path.join(td, 'early')
        notes = work + '.notes'
        os.makedirs(work)
        os.makedirs(notes)
        src = os.path.join(work, 'input.kicad_pcb')
        open(src, 'w').write('(kicad_pcb)\n')
        ch = AP.Chain(src, work)
        fan = {'ref': 'U1', 'tool': 'bga',
               'ladder': [['{IN}', '{OUT}', notes] + [str(x) for x in r] for r in rows]}
        calls = [(a, f'fanout_U1_r{i}', None) for i, a in enumerate(fan['ladder'])]
        with contextlib.redirect_stdout(io.StringIO()):
            runs = ch.run_each('bga', calls, jobs=3, done=AP._rung_clean)
            first = next(runs)          # rung 0 ends at 1.5 s; rung 1 was clean at 0.1 s
            pid = int(open(os.path.join(
                notes, 'step03_fanout_U1_r2.kicad_pcb.start')).read().split()[0])
            stopped_early = not alive(pid)
            runs.close()
        ok(first[0] is not None and stopped_early,
           'the third rung was already stopped when the first was read')
        ok(sorted(f for f in os.listdir(work) if f.startswith('step')) == [
            'step01_fanout_U1_r0.kicad_pcb', 'step01_fanout_U1_r0.kicad_pro',
            'step01_fanout_U1_r0.log'] and ch.n == 1,
           f"a caller that stops reading keeps only what it read: "
           f"{sorted(os.listdir(work))}, n={ch.n}")
    finally:
        if old is None:
            del os.environ[ENV]
        else:
            os.environ[ENV] = old
    assert work_before < set(os.listdir(td))


def test_single(td):
    print('4. a ladder of one rung')
    calls = []
    real = AP.Chain.run

    def spy(self, *a, **k):
        calls.append(a[2])
        return real(self, *a, **k)
    AP.Chain.run = spy
    try:
        one, _ = ladder(td, 'single', [(0.1, 8, 1, 2, 'ok')], 3)
        n_single = len(calls)
        ladder(td, 'single3', SHAPES['none clean, equals end in reverse'][0], 3, rungs=[1])
        n_named = len(calls) - n_single
        ladder(td, 'three', SHAPES['none clean, equals end in reverse'][0], 3)
        n_three = len(calls) - n_single - n_named
    finally:
        AP.Chain.run = real
    ok(n_single == 1 and one['best']['rung'] == 0, f'goes through Chain.run: {calls[:1]}')
    ok(n_named == 1, 'so does one named rung of a longer ladder (a re-fan\'s replay)')
    ok(n_three == 0, 'and a ladder of three at 3 jobs does not (non-vacuity)')


def test_count():
    print('5. the count')
    keep = os.environ.pop(ENV, None), AP.os.cpu_count, AP._physical_memory
    try:
        AP._physical_memory = lambda: 64 * 2 ** 30
        AP.os.cpu_count = lambda: 10
        ok(AP.ladder_jobs() == AP.LADDER_JOBS == 3, f'3 on a large machine: {AP.ladder_jobs()}')
        AP.os.cpu_count = lambda: 2
        ok(AP.ladder_jobs() == 2, f'the cores hold it: {AP.ladder_jobs()} on 2')
        AP.os.cpu_count = lambda: None
        ok(AP.ladder_jobs() == 1, 'one at a time where the cores cannot be counted')
        AP.os.cpu_count = lambda: 10
        AP._physical_memory = lambda: int(2.5 * AP.LADDER_CHILD_GB * 2 ** 30)
        ok(AP.ladder_jobs() == 2, f'the memory holds it: {AP.ladder_jobs()} at 2.5 rungs\' worth')
        AP._physical_memory = lambda: 2 ** 30
        ok(AP.ladder_jobs() == 1, 'never under one')
        AP._physical_memory = lambda: None
        ok(AP.ladder_jobs() == 3, 'memory that cannot be asked holds nothing')
        os.environ[ENV] = '1'
        ok(AP.ladder_jobs() == 1, f'{ENV}=1 is one at a time')
        os.environ[ENV] = '5'
        AP.os.cpu_count = lambda: 2
        ok(AP.ladder_jobs() == 5, 'a count that is asked for is the count')
    finally:
        os.environ.pop(ENV, None)
        if keep[0] is not None:
            os.environ[ENV] = keep[0]
        AP.os.cpu_count, AP._physical_memory = keep[1:]
    ok(isinstance(AP._physical_memory(), int) and AP._physical_memory() > 2 ** 28,
       f"this machine's memory is read: {AP._physical_memory()}")
    board = run_utils.evidence(os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb'))
    for bad in ('0', 'many', '2.5'):
        run_utils.check([sys.executable, os.path.join(REPO, 'py_router', 'autoroute.py'),
                         board, os.path.join(tempfile.gettempdir(), 'krt1942_unused.kicad_pcb'),
                         '--dry-run'], env=dict(os.environ, **{ENV: bad}),
                        refuse=f'{ENV} must be a whole number of at least 1', code=2)
    ok(True, f'autoroute.py refuses {ENV} = 0, many, 2.5, naming it')
    r = run_utils.check([sys.executable, os.path.join(REPO, 'py_router', 'autoroute.py'),
                         board, os.path.join(tempfile.gettempdir(), 'krt1942_unused.kicad_pcb'),
                         '--dry-run'], env=dict(os.environ, **{ENV: '1'}), accept=True)
    ok('AUTOROUTE_PLAN: ' in r.stdout, f'and takes {ENV}=1')


class _Stop(Exception):
    """The chain reached its first step past the fan-outs."""


def fan_steps(td, name, src, jobs):
    """Run the chain's fan-out steps on SRC at JOBS (the planes first, as the
    chain does): everything a reader could see, and when each child ran."""
    from kicad_parser import parse_kicad_pcb
    work = os.path.join(td, f'{name}{jobs}')
    os.makedirs(work)
    board = os.path.join(work, 'input.kicad_pcb')
    subprocess.run([sys.executable, os.path.join(REPO, 'py_router', 'copy_board.py'),
                    src, board], capture_output=True, cwd=REPO)
    run_utils.evidence(board)
    chains, spans = [], []
    real_run, real_popen = AP.Chain.run, subprocess.Popen

    def run(self, kind, args, tag, **kw):
        chains.append(self)
        if kind not in ('planes', 'bga', 'qfn'):
            self.end_step()
            raise _Stop()
        return real_run(self, kind, args, tag, **kw)

    def popen(argv, *a, **k):
        proc = real_popen(argv, *a, **k)
        argv = [str(x) for x in argv]
        row = {'out': (os.path.basename(argv[argv.index('--output') + 1])
                       if '--output' in argv else None),
               'start': time.monotonic(), 'proc': proc}
        spans.append(row)
        return proc
    old = os.environ.get(ENV)
    os.environ[ENV] = str(jobs)
    AP.Chain.run, AP.subprocess.Popen = run, popen
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(io.StringIO()):
            a = AP.analyze(board)
            t0 = time.monotonic()
            try:
                AP.run_plan(board, os.path.join(work, 'out.kicad_pcb'), a, work)
            except _Stop:
                pass
            wall = time.monotonic() - t0
    finally:
        AP.Chain.run, AP.subprocess.Popen = real_run, real_popen
        if old is None:
            del os.environ[ENV]
        else:
            os.environ[ENV] = old
    ch = chains[0]
    pcb = parse_kicad_pcb(ch.cur)
    copper = sorted(
        [('s', round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4),
          round(s.width, 4), s.layer, s.net_id) for s in pcb.segments]
        + [('v', round(v.x, 4), round(v.y, 4), round(v.size, 4), round(v.drill, 4),
            list(v.layers), v.net_id) for v in pcb.vias], key=repr)
    seen = {'kept': os.path.basename(ch.cur), 'n': ch.n, 'decisions': ch.decisions,
            'steps': [{k: v for k, v in s.items() if k != 'seconds'} for s in ch.steps],
            'lines': lines_of(buf.getvalue(), work), 'files': sorted(os.listdir(work)),
            'copper': copper}
    seen = json.loads(json.dumps(seen).replace(work, '<work>'))
    return seen, {'wall': wall, 'spans': spans,
                  'seconds': {s['tag']: s['seconds'] for s in ch.steps}}


def test_board(td):
    src = run_utils.evidence(os.path.join(REPO, 'kicad_files', BOARD))
    r1_board = f'_fanout_{PART}_r1.kicad_pcb'

    print(f'6. {BOARD}, {PART}: its first rung is clean')
    one, _ = fan_steps(td, 'clean', src, 1)
    got, t3 = fan_steps(td, 'clean', src, 3)
    tags = [s['tag'] for s in one['steps']]
    ok(f'fanout_{PART}' in tags and f'fanout_{PART}_r1' not in tags,
       f"one at a time the ladder ends at its first rung: {tags}")
    ok(len(one['copper']) > 20, f"the kept board carries the fan-out's copper: "
       f"{len(one['copper'])} tracks and vias")
    for k in one:
        ok(one[k] == got[k], f"3 jobs: the same {k}" + ('' if one[k] == got[k] else
           f": {str(one[k])[:400]} / {str(got[k])[:400]}"))
    ahead = [s for s in t3['spans'] if s['out'] and s['out'].endswith(r1_board)]
    ok(len(ahead) == 1, 'at 3 jobs the second rung was started too (non-vacuity)')
    ok(not [s for s in t3['spans'] if s['proc'].poll() is None], 'no child is left running')
    ok(not [f for f in got['files'] if '_r1' in f],
       f"and nothing the real tool wrote for it is left: {got['files']}")

    print(f'7. {BOARD}, {PART}: its first rung fails, so the second is needed')
    old = os.environ.get(AP.FAIL_STEPS_ENV)
    os.environ[AP.FAIL_STEPS_ENV] = f'fanout_{PART}'
    try:
        one, _ = fan_steps(td, 'needed', src, 1)
        got, t3 = fan_steps(td, 'needed', src, 3)
    finally:
        if old is None:
            del os.environ[AP.FAIL_STEPS_ENV]
        else:
            os.environ[AP.FAIL_STEPS_ENV] = old
    kept = [d for d in one['decisions'] if d['tag'] == f'fanout_{PART}']
    ok(kept and kept[0]['kept'] and 'kept rung 1' in kept[0]['why']
       and one['kept'].endswith(r1_board),
       f"one at a time the second rung is kept (non-vacuity): {kept and kept[0]['why']}")
    for k in one:
        ok(one[k] == got[k], f"3 jobs: the same {k}" + ('' if one[k] == got[k] else
           f": {str(one[k])[:400]} / {str(got[k])[:400]}"))
    ok(not [s for s in t3['spans'] if s['proc'].poll() is None], 'no child is left running')


def main():
    td = tempfile.mkdtemp(prefix='krt1942_')
    stub = os.path.join(td, 'stub_fanout.py')
    with open(stub, 'w') as fh:
        fh.write(STUB)
    tools = dict(AP.TOOLS)
    AP.TOOLS['bga'] = stub
    try:
        test_scripted(td)
        test_overlap(td)
        test_unread(td)
        test_single(td)
    finally:
        AP.TOOLS.update(tools)
    try:
        test_count()
        test_board(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
