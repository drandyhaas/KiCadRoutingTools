#!/usr/bin/env python3
"""The watcher that cried wolf 683 times.

Run 20 armed `run_watch cheats` over a 5h43m run. It emitted ~683 lines from a
bare case-sensitive substring scan; **6** were flags the run actually passed.
Everything else was the flag being MENTIONED: a driver's own refusal telling you
to pass `--accept-residue`, a tool disclosing "`--clearance` not given", an
argparse help dump, a ledger `lever` sentence narrating the lever it used. A
watcher wrong that often trains its reader to skim it, which is the same as not
having one.

The anchor is `py_router/cli_banner.py`: exactly one `CMD: <argv>` line per tool
run, built from `sys.orig_argv`, installed by 27 tools and pinned by
`tests/test_run4_instruments.py` and `tests/test_run12_tools.py`. A flag on that
line was passed; a flag anywhere else was typed about.

Three other findings from the same run are pinned here:

  * BLOCKING-UP compared cycle 2's FIRST scored lap against cycle 1's FINAL one
    and reported `blocking 10 -> 82` as a regression.
  * the JOURNAL -- the artifact the run is REQUIRED to write as it goes -- was
    invisible to the stall timer, so a stretch whose only output was the journal
    read as a dead run. It must count as LIVENESS but must not be SCANNED: it
    quotes tool output, and scanning it reported `NOT BUILDABLE` from a line
    reading "NOT BUILDABLE -> buildable", i.e. the record of the fix.
  * `run_watch.py` had no test file and no `--self-test` at all.
"""
import json
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tests', 'stress'))

import run_watch as RW  # noqa: E402

SKIP_EXIT = 77
passed = failed = 0


def check(label, cond, detail=''):
    global passed, failed
    if cond:
        passed += 1
        print(f'  OK   {label}')
    else:
        failed += 1
        print(f'  FAIL {label} -- {detail}')


def scan_cheats(workdir):
    """What `watch_cheats` would report on one pass over `workdir`."""
    seen, out = set(), []
    for path in RW._walk(workdir):
        rel = os.path.relpath(path, workdir).replace('\\', '/')
        try:
            lines = open(path, encoding='utf-8', errors='replace').readlines()
        except OSError:
            continue
        for line in lines:
            parsed = RW._parse_cmd(line)
            if parsed is None:
                continue
            tool, toks = parsed
            for label, flag, _why in RW._cheat_hits(tool, toks):
                key = RW._use_key(label, flag, tool, toks)
                if key in seen:
                    continue
                seen.add(key)
                out.append(f'{label} {flag} {tool}')
        if path.endswith('.jsonl'):
            for line in RW._scan_ledger_argv(path, seen, rel):
                out.append(line.split(' -- ')[0])
    return out


print('--- the precision fixture: shapes that fired 683 times ---')

d = tempfile.mkdtemp()
with open(os.path.join(d, 'run.log'), 'w', encoding='utf-8') as fh:
    fh.write(
        # a driver refusal that NAMES the flag it wants you to pass
        'If the overhang is BY DESIGN, declare it in the intent and re-run\n'
        'with --accept-residue oob_pad_count.\n'
        # a tool disclosing that it did NOT get the flag
        '  --clearance not given; honoring net classes with base = 0.15mm.\n'
        '  --board-edge-clearance not given; using 0.254mm [board constraint].\n'
        # an argparse help dump
        'usage: route.py [-h] [--nets NETS [NETS ...]] [--force-reroute]\n'
        '                [--rip-existing-nets PATTERN [PATTERN ...]]\n'
        # a router hint suggesting a flag
        "Hint: retry with --rip-existing-nets to authorize\n"
        # the driver naming the escape hatch it did NOT take
        'DELEGATING: this half goes to a TEAMMATE. --no-delegate runs it here.\n')
with open(os.path.join(d, 'ledger.jsonl'), 'w', encoding='utf-8') as fh:
    fh.write(json.dumps({
        'iteration': 0, 'kind': 'completion', 'accepted': True,
        # prose that narrates the flags -- exactly the disclosure the run is
        # SUPPOSED to make. Reporting it punishes the disclosure.
        'lever': 'rip lever AUTHORIZED AND DISCLOSED -- --rip-existing-nets '
                 'over 17 nets, at the board floors, no --clearance passed',
    }) + '\n')

hits = scan_cheats(d)
check('zero events from prose, help, hints, refusals and ledger levers',
      hits == [], f'{hits}')

with open(os.path.join(d, 'real.log'), 'w', encoding='utf-8') as fh:
    fh.write('CMD: python3.exe -X utf8 py_router/check_drc.py b.kicad_pcb '
             '--clearance 0.2\nEXIT=0\n')
hits = scan_cheats(d)
check('exactly one event once a real CMD: line appears',
      len(hits) == 1 and '--clearance' in hits[0], f'{hits}')

print('--- token-exactness ---')
check('--clearance does not match inside --clearance-margin',
      not RW._flag_in(['--clearance-margin', '0.1'], '--clearance'))
check('--nets does not match inside --ignore-nets',
      not RW._flag_in(['--ignore-nets', 'GND'], '--nets'))
check('the --flag=value form still matches',
      RW._flag_in(['--nets=BUSY'], '--nets'))

print('--- tool scoping ---')
check('route_diff.py --nets DP DN is silent (naming the pair IS the tool)',
      not RW._cheat_hits('route_diff.py', ['--nets', 'DP', 'DN']))
check('route_planes.py --nets GND +3V3 is silent (naming the pour IS the tool)',
      not RW._cheat_hits('route_planes.py', ['--nets', 'GND', '+3V3']))
check('route.py --nets "*" is silent (the whole board, not a narrowing)',
      not RW._cheat_hits('route.py', ['--nets', '*']))
check('route.py --nets <subset> is reported',
      RW._cheat_hits('route.py', ['--nets', 'BUSY', 'SCK']))
check('check_drc.py --clearance is reported (a grader floor override)',
      RW._cheat_hits('check_drc.py', ['--clearance', '0.2']))
check('place_optimize.py --ignore-nets is silent (airwire scoring, no verdict)',
      not RW._cheat_hits('place_optimize.py', ['--ignore-nets', 'GND']))

print('--- one lever recorded twice is one event ---')
log = ['python3.exe', '-X', 'utf8', 'C:/r/py_router/route.py',
       'C:/r/wk/a.kicad_pcb', '--nets', 'CS', '/40M_N']
led = ['python3', '-X', 'utf8', 'py_router/route.py', 'wk/a.kicad_pcb',
       '--nets', '/40M_N', 'CS']
check('log spelling and ledger spelling collapse to one key',
      RW._use_key('SCOPE', '--nets', 'route.py', log)
      == RW._use_key('SCOPE', '--nets', 'route.py', led))
check('a different scope stays a separate event',
      RW._use_key('SCOPE', '--nets', 'route.py', log)
      != RW._use_key('SCOPE', '--nets', 'route.py',
                     ['route.py', '--nets', 'BUSY']))

print('--- BLOCKING-UP: kept routing laps only ---')


def ledger_events(rows):
    p = os.path.join(tempfile.mkdtemp(), 'ledger.jsonl')
    with open(p, 'w', encoding='utf-8') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')
    return RW._scan_ledger(p, set(), 'ledger.jsonl')


ev = ledger_events([
    {'iteration': 1, 'kind': 'completion', 'accepted': True,
     'score': {'blocking': 10}},
    {'iteration': 2, 'kind': 'completion', 'accepted': False,
     'score': {'blocking': 82}},          # rejected: the loop already reverted it
    {'iteration': 3, 'kind': 'completion', 'accepted': True,
     'score': {'blocking': 12}},
])
# This is the DISCRIMINATING direction, and my first version of this test had it
# backwards. If the rejected 82 became the baseline, 12 < 82 and the watcher goes
# SILENT -- a real regression hidden by a lap the loop already threw away. With
# the rejected lap skipped, 12 is compared against 10 and fires. So the correct
# assertion is that it DOES fire, naming 10 -> 12.
up = [e for e in ev if e.startswith('BLOCKING-UP')]
check('a rejected lap is skipped, so the next kept lap compares against the '
      'last KEPT one (10 -> 12, not 82 -> 12 which would be silent)',
      len(up) == 1 and 'blocking 10 -> 12' in up[0],
      f'{[e[:70] for e in ev]}')
check('and it is still reported as a rejected lap',
      len([e for e in ev if e.startswith('REJECTED-LAP')]) == 1)

ev = ledger_events([
    {'iteration': 1, 'kind': 'completion', 'accepted': True,
     'score': {'blocking': 10}},
    {'iteration': 2, 'kind': 'placement', 'accepted': True},   # a half boundary
    {'iteration': 3, 'kind': 'completion', 'accepted': True,
     'score': {'blocking': 82}},          # a NEW board's first score
])
check('a placement lap resets the sequence (run 20: 10 -> 82 was cross-cycle)',
      not [e for e in ev if e.startswith('BLOCKING-UP')],
      f'{[e[:70] for e in ev]}')

ev = ledger_events([
    {'iteration': 1, 'kind': 'completion', 'accepted': True,
     'score': {'blocking': 10}},
    {'iteration': 2, 'kind': 'completion', 'accepted': True,
     'score': {'blocking': 15}},
])
check('a REAL regression between two kept routing laps is still reported',
      len([e for e in ev if e.startswith('BLOCKING-UP')]) == 1,
      f'{[e[:70] for e in ev]}')

ev = ledger_events([{'iteration': 1, 'kind': 'placement', 'accepted': True,
                     'score': {'blocking': None}}])
check('BLOCKING-NULL is reported on ANY lap, not only routing ones',
      len([e for e in ev if e.startswith('BLOCKING-NULL')]) == 1,
      f'{[e[:70] for e in ev]}')

print('--- the journal: liveness yes, scanning no ---')
jd = tempfile.mkdtemp()
with open(os.path.join(jd, 'JOURNAL.md'), 'w', encoding='utf-8') as fh:
    fh.write('## 9. the damage repaired\n'
             'assembly verdict NOT BUILDABLE -> buildable, and the ERROR the\n'
             'router printed is quoted here so the record carries it.\n')
check('the journal is NOT scanned for signatures',
      RW._scan_text(os.path.join(jd, 'JOURNAL.md'), set(), 'JOURNAL.md') == []
      or '.md' not in RW.SCAN_EXT,
      'a journal narrating "NOT BUILDABLE -> buildable" is not an incident')
check('and it is not in the scan walk at all',
      list(RW._walk(jd)) == [], str(list(RW._walk(jd))))
check('but it IS in the liveness walk',
      len(list(RW._walk(jd, RW.WATCHED_EXT))) == 1)

print('--- the real run-20 work dir, if it is still here ---')
wk = os.path.join(REPO, 'wk', 'run20')
if os.path.isdir(wk):
    raw = 0
    for p in RW._walk(wk):
        try:
            txt = open(p, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for _l, f, _w, _t in RW.CHEAT_FLAGS:
            raw += txt.count(f)
    hits = scan_cheats(wk)
    print(f'       raw substring hits {raw} -> reported {len(hits)}')
    check('the real run reports far fewer than the raw substring count',
          len(hits) < raw / 20, f'{len(hits)} of {raw}')
    led = os.path.join(wk, 'ledger.jsonl')
    ev = RW._scan_ledger(led, set(), 'ledger.jsonl')
    check('and its ledger produces no BLOCKING-UP at all',
          not [e for e in ev if e.startswith('BLOCKING-UP')],
          f'{[e[:80] for e in ev if e.startswith("BLOCKING-UP")]}')
else:
    print('       (wk/run20 absent -- skipping the real-corpus checks)')

print('--- a stall is a SYMPTOM, not an outcome ---')
# Run 20's timer exited on a 20-minute stall while the run was still working,
# and from that point the absence of events -- the timer's entire signal --
# meant nothing, because nothing was watching. A watcher that stops watching
# when it sees something worrying is the opposite of a watcher.
#
# Driven on a FAKE CLOCK: a real one would make this a 20-minute test, and a
# test nobody runs pins nothing.
import io                                                          # noqa: E402
import contextlib                                                  # noqa: E402

check('the escalation ladder is multiplicative and rises',
      RW._STALL_LADDER == (1.0, 2.0, 4.0)
      and list(RW._STALL_LADDER) == sorted(RW._STALL_LADDER),
      str(RW._STALL_LADDER) + ' -- repeating the same sentence four times does '
      'not make an 80-minute silence read differently from a 20-minute one')


def _fake_run(events, stall_min=20.0, max_min=0, stall_exit=False):
    """Drive watch_timer over a scripted timeline.

    `events` is [(minute, touch_or_not), ...]; the clock and the work-dir
    signature are both faked, so the whole run is instantaneous.
    """
    d = tempfile.mkdtemp()
    state = {'t': 0.0, 'i': 0, 'sig': 0}
    _t0 = 1_000_000.0

    def _time():
        return _t0 + state['t'] * 60.0

    def _walk(_wd, _exts=None):
        return []

    def _sleep(_s):
        if state['i'] >= len(events):
            raise SystemExit
        state['t'], touched = events[state['i']]
        state['i'] += 1
        if touched:
            state['sig'] += 1

    # The signature comes from _walk; fake it by returning a changing list.
    def _walk2(_wd, _exts=None):
        return [os.path.join(d, f'f{state["sig"]}.log')]

    buf = io.StringIO()
    old = (RW.time.time, RW.time.sleep, RW._walk, RW.os.path.getmtime,
           RW.os.path.getsize)
    RW.time.time, RW.time.sleep, RW._walk = _time, _sleep, _walk2
    RW.os.path.getmtime = lambda p: 0.0
    RW.os.path.getsize = lambda p: 0
    try:
        with contextlib.redirect_stdout(buf):
            try:
                RW.watch_timer(d, os.path.join(d, 'NEVER'), 1.0, stall_min,
                               max_min, stall_exit=stall_exit)
            except SystemExit:
                pass
    finally:
        (RW.time.time, RW.time.sleep, RW._walk, RW.os.path.getmtime,
         RW.os.path.getsize) = old
    return [ln for ln in buf.getvalue().splitlines() if ln.strip()]


# Quiet for 25 minutes: one notice, and the watch does NOT end.
out = _fake_run([(m, False) for m in range(1, 30)])
_st = [ln for ln in out if 'STALLED' in ln]
check('a quiet stretch emits ONE stall notice, not one per poll',
      len(_st) == 1, f'{len(_st)}: {_st[:3]}')
check('and the notice says it is not terminal',
      _st and 'NOT a terminal state' in _st[0], str(_st))
# "It continues" is only demonstrable by PAIRING: the same timeline with and
# without --stall-exit. A single run's output cannot distinguish "kept polling
# and had nothing more to say" from "returned".
_pair_on = _fake_run([(m, False) for m in range(1, 50)], stall_exit=False)
_pair_off = _fake_run([(m, False) for m in range(1, 50)], stall_exit=True)
check('the watch CONTINUES past a stall, where --stall-exit ends it',
      len([x for x in _pair_on if 'STALLED' in x]) == 2
      and len([x for x in _pair_off if 'STALLED' in x]) == 1,
      f'{len([x for x in _pair_on if "STALLED" in x])} vs '
      f'{len([x for x in _pair_off if "STALLED" in x])} on the same timeline '
      f'-- the second notice can only exist if the first did not return')

# Quiet, then activity, then quiet again: TWO episodes, with a RESUMED between.
out = _fake_run([(m, False) for m in range(1, 26)]
                + [(26, True)]
                + [(m, False) for m in range(27, 55)])
_st = [ln for ln in out if 'STALLED' in ln]
check('activity RE-ARMS the notice: a second episode is reported',
      len(_st) >= 2, f'{len(_st)}: {[x[:60] for x in _st]}')
check('and the resumption is announced, so the first notice is not left '
      'reading as the end',
      any('RESUMED' in ln for ln in out), str(out[:4]))

# Sustained silence: escalating thresholds, each named, then it stops repeating.
out = _fake_run([(m, False) for m in range(1, 130)])
_st = [ln for ln in out if 'STALLED' in ln]
check('a sustained silence escalates through the ladder',
      len(_st) == len(RW._STALL_LADDER),
      f'{len(_st)} notices for {len(RW._STALL_LADDER)} rungs: '
      f'{[x[:70] for x in _st]}')
check('each notice names the threshold it fired at',
      all('threshold' in ln for ln in _st), str(_st[:1]))
check('and the last one says it is the last',
      _st and 'last stall notice' in _st[-1], str(_st[-1:]))

out = _fake_run([(m, False) for m in range(1, 30)], stall_exit=True)
check('--stall-exit restores the old behaviour exactly',
      len([ln for ln in out if 'STALLED' in ln]) == 1
      and not [ln for ln in out if 'RESUMED' in ln], str(out))

out = _fake_run([(m, False) for m in range(1, 30)], stall_min=0)
check('--stall-min 0 emits nothing at all',
      not [ln for ln in out if 'STALLED' in ln], str(out))

out = _fake_run([(m, False) for m in range(1, 40)], max_min=30)
check('TIMEOUT is still terminal, and still reached through a stall',
      any('TIMEOUT' in ln for ln in out)
      and any('STALLED' in ln for ln in out), str(out))

check('the contract in the docstring says so too',
      'ZERO OR MORE STALL NOTICES' in (RW.__doc__ or ''),
      'a contract change that only lives in the code is one the next reader '
      'will undo')

print('--- run-22: a teed cmd_timing.jsonl is a source of truth ---')

# tee_cmd.py records argv for EVERY invocation a timed run makes. Run 22 wrote
# 203 such rows carrying five --waive uses, one --fab-overrides and a --nets
# narrowing, and this watcher reported ZERO of them: it only ever read
# `lever_argv`. The two skill drivers install no cli_banner either, so a teed
# row is the ONLY place their invocations are ever visible.
d22 = tempfile.mkdtemp()
with open(os.path.join(d22, 'cmd_timing.jsonl'), 'w', encoding='utf-8') as f:
    f.write(json.dumps({
        'label': 'place_driver_p3b',
        'argv': ['python3', '-X', 'utf8',
                 '.claude/skills/plan-pcb-placement/scripts/placement_driver.py',
                 '--stage', 'P3', '--waive', 'H1:mechanically pinned'],
        'cmdline': 'x', 'exit': 0}) + chr(10))
    f.write(json.dumps({
        'label': 'rt_route4_pinned',
        'argv': ['python3', 'py_router/route.py', 'in.kicad_pcb',
                 'out.kicad_pcb', '--fab-overrides', 'fab_floor.txt'],
        'cmdline': 'x', 'exit': 0}) + chr(10))
out22 = scan_cheats(d22)
check('--waive on a driver in a teed row is reported',
      any('--waive' in ln for ln in out22), str(out22))
check('--fab-overrides on a router in a teed row is reported',
      any('--fab-overrides' in ln for ln in out22), str(out22))

_flags = {f for _lab, f, _why, _tools in RW.CHEAT_FLAGS}
for _f in ('--waive', '--accept-boxin', '--fab-tier', '--fab-overrides',
           '--no-fix-drc-settings'):
    check(f'{_f} is a known cheat flag', _f in _flags)

# `lever` PROSE must still never be matched -- reporting a run's own
# disclosure is the mistake the scanner's docstring exists to prevent.
d23 = tempfile.mkdtemp()
with open(os.path.join(d23, 'ledger.jsonl'), 'w', encoding='utf-8') as f:
    f.write(json.dumps({
        'iteration': 1,
        'lever': 'I passed --waive H1:mechanical and said so in the journal',
    }) + chr(10))
check('a flag named in ledger PROSE is still not a finding',
      scan_cheats(d23) == [], str(scan_cheats(d23)))

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
