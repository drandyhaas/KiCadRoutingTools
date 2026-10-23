#!/usr/bin/env python3
"""#1177: measure.py measures the run's OWN transcript, or refuses.

It used to read the newest top-level `*.jsonl` under the slug of `--root`.
Claude Code files a session under the directory it STARTED in, and a
subagent under `<session>/subagents/agent-<id>.jsonl`, so a run working in a
worktree got `refuse: no transcript`, and a subagent run got the
orchestrator's numbers: fa10 board 02 reported 2 days 9:30 and 28 spawns for
a run its own transcript measures at 1:13. Nothing checked that the file
chosen was the run.

Now every transcript under the projects dir is a candidate (sessions and
subagents, every slug), and the filters say which one is the run:
`--self` (the transcript whose call to measure.py is in flight -- the
default), `--evidence PATH` (its OWN tool calls name the run's work dir or
board) and `--agent-id`. None, or more than one, survivor is a refusal
(exit 3) that lists them. The measured file, its sidechain flag and the
candidate count are printed and stored under `selection`.

Minimal input, synthetic (the issue's): session A spawned subagent B, and A
was written LAST. Pinned here:
  * the default measures B, the caller, though A is newer;
  * `--agent-id` and `--evidence` find B; A's delegation prompt and A's own
    call to measure.py naming B's work dir are not evidence of A;
  * a parent that also touched the work dir makes `--evidence` alone
    ambiguous: refused, both named; `--self --evidence` resolves it;
  * a Windows-slashed command still names the work dir, and `wk/runB10`
    does not name `wk/runB`;
  * no in-flight call, or no match, is a refusal with exit 3, never a guess.

Run: python3 -X utf8 tests/test_1177_measure_finds_own_transcript.py
"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from run_utils import check, evidence                          # noqa: E402

PY = [sys.executable, '-X', 'utf8']
MEASURE = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                       'scripts', 'measure.py')
PRIMARY = '/home/u/krt'
WORKTREE = '/home/u/krt-fa10'
SLUG = '-home-u-krt'
SID = '4f17074a-0000-4000-8000-000000000001'


def _use(ts, tid, name, inp, sid=SID, side=False, agent=None, cwd=PRIMARY):
    r = {'type': 'assistant', 'timestamp': ts, 'sessionId': sid,
         'isSidechain': side, 'cwd': cwd,
         'message': {'id': 'm' + tid, 'usage': {'output_tokens': 3},
                     'content': [{'type': 'tool_use', 'id': tid, 'name': name,
                                  'input': inp}]}}
    if agent:
        r['agentId'] = agent
    return r


def _result(ts, tid, sid=SID, side=False, agent=None, cwd=PRIMARY):
    r = {'type': 'user', 'timestamp': ts, 'sessionId': sid,
         'isSidechain': side, 'cwd': cwd,
         'message': {'content': [{'type': 'tool_result', 'tool_use_id': tid}]}}
    if agent:
        r['agentId'] = agent
    return r


def _bash(cmd):
    return {'command': cmd}


def _write(path, rows, mtime):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')
    os.utime(path, (mtime, mtime))
    return path


def _sub(agent, steps, in_flight=True):
    """A subagent transcript: its shell steps, then (optionally) the
    in-flight measure.py call it is making right now."""
    kw = dict(side=True, agent=agent, cwd=WORKTREE)
    rows = []
    for i, cmd in enumerate(steps):
        rows += [_use('2026-10-07T10:%02d:00Z' % i, '%s%d' % (agent, i), 'Bash',
                      _bash(cmd), **kw),
                 _result('2026-10-07T10:%02d:30Z' % i, '%s%d' % (agent, i), **kw)]
    call = _use('2026-10-07T11:13:00Z', agent + 'm', 'Bash', _bash(
        'python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/measure.py '
        '--self --evidence wk/runB'), **kw)
    rows.append(call)
    if not in_flight:
        rows.append(_result('2026-10-07T11:13:05Z', agent + 'm', **kw))
    return rows


def _session(touch_run_b=False):
    """Session A: does its own run (wk/runA), delegates B, measures B after
    the fact -- and optionally lists B's work dir itself."""
    rows = [
        _use('2026-10-05T09:00:00Z', 'a1', 'Bash', _bash(
            'python3 py_placer/converge.py record --ledger wk/runA/ledger.jsonl')),
        _result('2026-10-05T09:00:10Z', 'a1'),
        _use('2026-10-05T09:01:00Z', 'a2', 'Agent', {
            'prompt': 'Run /plan-pcb-placement full on wk/runB/pile.kicad_pcb; '
                      'work in wk/runB.'}),
        _result('2026-10-05T09:01:01Z', 'a2'),
        _use('2026-10-07T12:00:00Z', 'a3', 'Bash', _bash(
            'python3 measure.py --evidence wk/runB --json wk/runB/m.json')),
        _result('2026-10-07T12:00:02Z', 'a3'),
    ]
    if touch_run_b:
        rows += [_use('2026-10-07T12:01:00Z', 'a4', 'Read',
                      {'file_path': PRIMARY + '/wk/runB/REPORT.md'}),
                 _result('2026-10-07T12:01:01Z', 'a4')]
    # A keeps running after B: 2 days of wall clock, still being written
    rows += [_use('2026-10-07T19:30:00Z', 'a5', 'Bash', _bash('ls')),
             _result('2026-10-07T19:30:07Z', 'a5')]
    return rows


class MeasureFindsItsOwnRun(unittest.TestCase):

    def setUp(self):
        self.td = tempfile.mkdtemp(prefix='t1177_')
        self.base = os.path.join(self.td, 'projects')
        self.now = time.time()

    def tearDown(self):
        shutil.rmtree(self.td, ignore_errors=True)

    def _board(self, touch_run_b=False, b_steps=None, b_in_flight=True,
               others=()):
        b = os.path.join(self.base, SLUG, SID, 'subagents',
                         'agent-bbbb.jsonl')
        _write(b, _sub('bbbb', b_steps or [
            'python3 py_tools/check_floorplan.py wk/runB/pile.kicad_pcb',
            'python3 py_placer/converge.py record --ledger wk/runB/ledger.jsonl',
        ], in_flight=b_in_flight), self.now - 600)
        for name, steps, flight in others:
            _write(os.path.join(self.base, SLUG, SID, 'subagents',
                                'agent-%s.jsonl' % name),
                   _sub(name, steps, in_flight=flight), self.now - 300)
        a = os.path.join(self.base, SLUG, SID + '.jsonl')
        _write(a, _session(touch_run_b), self.now)      # A written LAST
        self.assertGreater(os.path.getmtime(a), os.path.getmtime(b))
        return a, b

    def _measure(self, *args):
        out = os.path.join(self.td, 'm.json')
        if os.path.exists(out):
            os.remove(out)
        r = check(PY + [MEASURE, '--projects-dir', self.base, '--json', out]
                  + list(args), accept=True)
        evidence(out, 'measure.py --json')
        with open(out, encoding='utf-8') as fh:
            return json.load(fh), r.stdout + r.stderr

    def _refused(self, reason, *args):
        return check(PY + [MEASURE, '--projects-dir', self.base] + list(args),
                     refuse=reason, code=3)

    def test_the_default_measures_the_caller_not_the_newest_session(self):
        _a, b = self._board()
        d, out = self._measure()
        self.assertEqual(d['session'], b, 'measured the newest file, not the run')
        self.assertTrue(d['sidechain'])
        self.assertEqual(d['agent_id'], 'bbbb')
        self.assertEqual(d['selection']['by'], ['self'])
        self.assertEqual(d['selection']['candidates'], 1)
        self.assertEqual(d['selection']['scanned'], 2)
        # B's own clock: 10:00 -> 11:13, not A's two days
        self.assertEqual(d['main']['wall_clock'], '1:13:00', d['main'])
        self.assertIn('measured: %s (sidechain, agent bbbb; selected by self, '
                      '1 candidate of 2 scanned)' % b, out)

    def test_agent_id_and_evidence_find_the_subagent(self):
        a, b = self._board()
        for args in (['--agent-id', 'bbbb'], ['--agent-id', 'agent-bbbb'],
                     ['--evidence', 'wk/runB'], ['--evidence', 'wk/runB/'],
                     ['--self', '--evidence', 'wk/runB']):
            d, _out = self._measure(*args)
            self.assertEqual(d['session'], b, args)
        # A's own run is A, a session (not a sidechain), with B as its subagent
        d, _out = self._measure('--evidence', 'wk/runA')
        self.assertEqual(d['session'], a)
        self.assertFalse(d['sidechain'])
        self.assertEqual(sorted(d['subagents']), ['agent-bbbb.jsonl'])

    def test_a_parent_that_touched_the_work_dir_is_refused_not_guessed(self):
        a, b = self._board(touch_run_b=True)
        r = self._refused('2 of 2 transcript(s)', '--evidence', 'wk/runB')
        out = r.stdout + r.stderr
        self.assertIn('candidate: ' + a, out)
        self.assertIn('candidate: ' + b, out)
        self.assertIn('narrow with --agent-id', out)
        d, _out = self._measure('--self', '--evidence', 'wk/runB')
        self.assertEqual(d['session'], b)

    def test_slashes_and_path_boundaries(self):
        _a, b = self._board(b_steps=[
            r'py -3 py_tools\check_floorplan.py wk\runB\pile.kicad_pcb'])
        d, _out = self._measure('--evidence', 'wk/runB')
        self.assertEqual(d['session'], b, 'a Windows-slashed command names it')
        self._refused('0 of 2 transcript(s)', '--evidence', 'wk/run')

    def test_two_runs_measuring_at_once_need_evidence(self):
        _a, b = self._board(others=[
            ('cccc', ['python3 py_router/route.py wk/runC/in.kicad_pcb '
                      'wk/runC/out.kicad_pcb'], True)])
        self._refused('2 of 3 transcript(s)')
        d, _out = self._measure('--self', '--evidence', 'wk/runB')
        self.assertEqual(d['session'], b)

    def test_nothing_in_flight_or_nothing_named_is_a_refusal(self):
        self._board(b_in_flight=False)
        r = self._refused('0 of 2 transcript(s)')
        self.assertIn('--self finds the agent RUNNING this command',
                      r.stdout + r.stderr)
        self._refused('0 of 2 transcript(s)', '--evidence', 'wk/nope')
        self._refused('0 of 2 transcript(s)', '--agent-id', 'zzzz')


if __name__ == '__main__':
    unittest.main()
