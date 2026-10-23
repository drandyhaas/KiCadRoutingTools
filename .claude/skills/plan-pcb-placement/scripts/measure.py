#!/usr/bin/env python3
"""Where a plan-pcb-placement session's wall clock went, from its transcript.

    python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/measure.py [SESSION.jsonl]
        [--self] [--evidence PATH ...] [--agent-id ID] [--projects-dir DIR]
        [--json OUT]

With no SESSION, it FINDS the run's own transcript among every transcript
Claude Code filed under --projects-dir (default ~/.claude/projects): each
session's `<slug>/<session>.jsonl` and each subagent's
`<slug>/<session>/subagents/agent-<id>.jsonl`, across every slug, because a
session is filed under the directory it STARTED in -- a run working in a
worktree is not under the worktree's slug (#1177). The filters compose
(a transcript must pass every one given):

- `--self`: the transcript whose shell call to measure.py is still waiting
  for its result, i.e. the agent running this command. The default when no
  other filter is given.
- `--evidence PATH`: its OWN tool calls name PATH (the run's work dir or
  board). A delegation prompt (Agent/Task) is not the run's own work, and a
  call to measure.py is the measuring, not the run, so neither counts.
- `--agent-id ID`: the subagent transcript `agent-<ID>.jsonl`.

Exactly one survivor is measured; none, or more than one, is a refusal
(exit 3) that lists what it saw. It never falls back to "the newest file":
that measured the orchestrator session -- 2 days, 28 spawns -- for a
subagent run that took 1:13. The measured file, whether it is a sidechain,
and how many candidates there were are printed (stderr) and stored under
`selection`. Subagent transcripts in <session>/subagents/ of a measured
top-level session are reported separately.

Exit codes: 0 measured; 3 refused.

Reports:
- wall clock;
- tool calls by tool;
- repo scripts run;
- how long the agent spent WAITING on its own jobs (polls, Monitor calls, and
  idling until a background job reported back) versus other tool time versus
  model time -- each gap between transcript rows goes to what ended it;
- tokens, de-duplicated by message id (a streamed message repeats its usage
  per content block);
- `converge.py` calls, COUNTED per verb, so a report can say how the ledger
  was used.

A line that does not parse is counted in `unparsed_lines`, never dropped.
"""
import argparse
import collections
import datetime
import glob
import json
import os
import re
import sys

KRT_TOOL = {'scope': ['combined'], 'kind': 'utility'}

SHELLS = ('Bash', 'PowerShell')
#: tools whose input hands work to someone else: a path in a prompt is not
#: evidence that THIS transcript did the run.
DELEGATION = ('Agent', 'Task')
MEASURE_RE = re.compile(r'measure\.py\b')
#: how far from a transcript's end its in-flight call can sit (a parallel
#: sibling's result may be written after it).
SELF_TAIL_BYTES = 4 << 20
REFUSED = 3
SCRIPT_RE = re.compile(r'([\w./\\-]*\w\.py)\b')
#: converge's own subcommands. Matching a closed set means an argument that
#: merely follows the script name (`grep x converge.py py_tools/y.py`) is not
#: read as a verb.
CONVERGE_VERBS = ('where', 'record', 'verdict')
CONVERGE_VERB_RE = re.compile(r'converge\.py["\']?\s+(%s)(?![\w./\\-])'
                              % '|'.join(re.escape(v) for v in CONVERGE_VERBS))
WAIT_RE = re.compile(r'(^|[;&|]\s*|\bdo\s+)(until|sleep)\b')


def projects_dir():
    return os.path.join(os.path.expanduser('~'), '.claude', 'projects')


def _ts(s):
    return datetime.datetime.fromisoformat(s.replace('Z', '+00:00'))


def read(path):
    rows, bad = [], 0
    with open(path, encoding='utf-8-sig', errors='replace') as fh:
        for line in fh:
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                bad += 1
    return rows, bad


def _split(rows, uses):
    """(waiting, tool, model) seconds: every gap between consecutive rows is
    attributed to WHAT ENDED IT.

    - an assistant row: the model was generating -> model;
    - a tool_result: a foreground command ran -> tool, unless that command
      was a poll (`until`/`sleep`, or Monitor) -> waiting;
    - anything else (a background job's <task-notification>, queue rows):
      the agent sat idle until its own job reported back -> waiting.

    The first cut counted only foreground polls as waiting, so an agent idle
    on background jobs read as "model time": run 33 reported 211 min of
    model time where the model spent 19 and waited 193.
    """
    polls = {k for k, (_t, name, inp) in uses.items()
             if name == 'Monitor'
             or (name in SHELLS and WAIT_RE.search(str(inp.get('command', ''))))}
    timed = sorted((r for r in rows if r.get('timestamp')),
                   key=lambda r: _ts(r['timestamp']))
    wait = work = model = 0.0
    for prev, cur in zip(timed, timed[1:]):
        gap = (_ts(cur['timestamp']) - _ts(prev['timestamp'])).total_seconds()
        content = (cur.get('message') or {}).get('content')
        ended_by = ({b.get('tool_use_id') for b in content
                     if isinstance(b, dict) and b.get('type') == 'tool_result'}
                    if isinstance(content, list) else set())
        if cur.get('type') == 'assistant':
            model += gap
        elif ended_by:
            if ended_by & polls:
                wait += gap
            else:
                work += gap
        else:
            wait += gap
    return wait, work, model


def tally(rows):
    tools, scripts, tokens = (collections.Counter() for _ in range(3))
    seen, uses, results = set(), {}, {}
    first = last = None
    turns = agents = 0
    verbs = collections.Counter()
    for r in rows:
        if r.get('timestamp'):
            t = _ts(r['timestamp'])
            first, last = min(first or t, t), max(last or t, t)
        m = r.get('message') or {}
        content = m.get('content')
        if r.get('type') == 'assistant':
            if m.get('id') and m['id'] not in seen:
                seen.add(m['id'])
                turns += 1
                for k, v in (m.get('usage') or {}).items():
                    if isinstance(v, int):
                        tokens[k] += v
        if not isinstance(content, list):
            continue
        for b in content:
            if b.get('type') == 'tool_result':
                results[b.get('tool_use_id')] = r.get('timestamp')
            if b.get('type') != 'tool_use':
                continue
            name, inp = b.get('name'), b.get('input') or {}
            tools[name] += 1
            uses[b.get('id')] = (r.get('timestamp'), name, inp)
            if name == 'Agent':
                agents += 1
            if name in SHELLS:
                cmd = str(inp.get('command', ''))
                hits = {os.path.basename(s.replace('\\', '/'))
                        for s in SCRIPT_RE.findall(cmd)}
                scripts.update(hits)
                if 'converge.py' in hits:
                    verbs.update(CONVERGE_VERB_RE.findall(cmd))
    wait, work, model = _split(rows, uses)
    wall = (last - first).total_seconds() if first else 0.0
    return {'wall_clock': str(last - first) if first else None,
            'wall_seconds': wall,
            'waiting_on_jobs_seconds': round(wait, 1),
            'other_tool_seconds': round(work, 1),
            'model_seconds': round(model, 1),
            'assistant_turns': turns, 'tool_calls': sum(tools.values()),
            'tools': dict(tools.most_common()),
            'repo_scripts': dict(scripts.most_common()),
            'agent_spawns': agents,
            'converge_record_calls': verbs.get('record', 0),
            'converge_verbs': dict(verbs),
            'tokens': dict(tokens)}


def identity(path, rows=None):
    """Who a transcript is, from its own rows (`isSidechain`, `agentId`,
    `sessionId`, `cwd`), falling back to where Claude Code filed it."""
    if rows is None:
        rows, _bad = read(path)
    first = lambda k: next((r[k] for r in rows if r.get(k) is not None), None)
    side = first('isSidechain')
    if side is None:
        side = os.path.basename(os.path.dirname(path)) == 'subagents'
    return {'path': path, 'sidechain': bool(side), 'agent_id': first('agentId'),
            'session_id': first('sessionId'), 'cwd': first('cwd')}


def measure(path):
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        raise SystemExit(f'refuse: {path} is not a real non-empty file')
    rows, bad = read(path)
    who = identity(path, rows)
    out = {'session': path, 'sidechain': who['sidechain'],
           'agent_id': who['agent_id'],
           'main': dict(tally(rows), unparsed_lines=bad), 'subagents': {}}
    for p in sorted(glob.glob(os.path.join(os.path.splitext(path)[0],
                                           'subagents', '*.jsonl'))):
        srows, sbad = read(p)
        out['subagents'][os.path.basename(p)] = dict(tally(srows),
                                                     unparsed_lines=sbad)
    return out


# ---------------------------------------------------------------- selection

def transcripts(base):
    """Every transcript filed under `base`, every slug: sessions, then
    their subagents."""
    top = glob.glob(os.path.join(base, '*', '*.jsonl'))
    sub = glob.glob(os.path.join(base, '*', '*', 'subagents', '**', '*.jsonl'),
                    recursive=True)
    return sorted(top) + sorted(sub)


def _uses(rows):
    """(tool_use blocks, answered tool_use ids) of a transcript's rows."""
    uses, answered = [], set()
    for r in rows:
        content = (r.get('message') or {}).get('content')
        if not isinstance(content, list):
            continue
        for b in content:
            if not isinstance(b, dict):
                continue
            if b.get('type') == 'tool_use':
                uses.append(b)
            elif b.get('type') == 'tool_result':
                answered.add(b.get('tool_use_id'))
    return uses, answered


def _is_measure_call(b):
    return (b.get('name') in SHELLS
            and bool(MEASURE_RE.search(str((b.get('input') or {}).get('command', '')))))


def in_flight_measure(path, tail=SELF_TAIL_BYTES):
    """True when the transcript has a shell call to measure.py with no
    result yet -- the agent that is running measure.py right now. Claude
    Code writes a tool call's row BEFORE it runs the tool, so the caller's
    own transcript always holds one; only its tail need be read, since the
    result of a call in the tail would come after it."""
    size = os.path.getsize(path)
    with open(path, 'rb') as fh:
        fh.seek(max(0, size - tail))
        raw = fh.read()
    if b'measure.py' not in raw:
        return False
    lines = raw.split(b'\n')[1 if size > tail else 0:]   # first may be torn
    rows = []
    for line in lines:
        try:
            rows.append(json.loads(line.decode('utf-8-sig', 'replace')))
        except ValueError:
            continue
    uses, answered = _uses(rows)
    return any(_is_measure_call(b) and b.get('id') not in answered for b in uses)


def _slashed(s):
    return s.replace('\\', '/').rstrip('/')


def evidence_patterns(evidence, cwd=None):
    """Regexes for one --evidence path: as given and absolute, either slash,
    bounded so `out/a` does not match `out/a2`."""
    forms = {_slashed(os.path.abspath(os.path.join(cwd or os.getcwd(), evidence)))}
    given = _slashed(os.path.normpath(evidence))
    if given not in ('', '.'):
        forms.add(given)
    return [re.compile(r'(?<![\w.-])%s(?![\w.-])' % re.escape(f), re.I)
            for f in sorted(forms)]


def _strings(obj):
    if isinstance(obj, str):
        yield obj
    elif isinstance(obj, dict):
        for v in obj.values():
            yield from _strings(v)
    elif isinstance(obj, list):
        for v in obj:
            yield from _strings(v)


def evidence_hits(rows, patterns):
    """How many of the transcript's OWN tool calls name the path: delegation
    prompts and calls to measure.py are not the run's work."""
    uses, _answered = _uses(rows)
    n = 0
    for b in uses:
        if b.get('name') in DELEGATION or _is_measure_call(b):
            continue
        text = _slashed('\n'.join(_strings(b.get('input') or {})))
        if any(p.search(text) for p in patterns):
            n += 1
    return n


def select(base, self_=False, evidence=(), agent_id=None, cwd=None):
    """Narrow every transcript under `base` by the filters given; returns
    (survivors as identity dicts, how many transcripts were scanned)."""
    paths = transcripts(base)
    scanned = len(paths)
    if agent_id:
        want = 'agent-%s.jsonl' % re.sub(r'^agent-', '', agent_id)
        paths = [p for p in paths if os.path.basename(p) == want]
    if self_:
        paths = [p for p in paths if in_flight_measure(p)]
    hits = {}
    if evidence:
        pats = [evidence_patterns(e, cwd) for e in evidence]
        # cheap byte prefilter on each path's last component, either slash
        keys = [_slashed(e).split('/')[-1].encode('utf-8').lower()
                for e in evidence]
        kept = []
        for p in paths:
            with open(p, 'rb') as fh:
                raw = fh.read().lower()
            if not all(k in raw for k in keys):
                continue
            rows, _bad = read(p)
            counts = [evidence_hits(rows, pp) for pp in pats]
            if all(counts):
                kept.append(p)
                hits[p] = counts
        paths = kept
    out = []
    for p in paths:
        who = identity(p)
        if p in hits:
            who['evidence_calls'] = hits[p]
        out.append(who)
    return out, scanned


def _describe(c):
    return ('%s  [%s%s, cwd %s%s]'
            % (c['path'], 'sidechain' if c['sidechain'] else 'session',
               ', agent ' + c['agent_id'] if c.get('agent_id') else '',
               c.get('cwd'),
               ', %s call(s) name the evidence' % c['evidence_calls']
               if 'evidence_calls' in c else ''))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('session', nargs='?',
                    help='measure this transcript (no search)')
    ap.add_argument('--self', dest='self_', action='store_true',
                    help='the transcript whose call to measure.py is in flight: '
                         'the agent running this (default with no other filter)')
    ap.add_argument('--evidence', action='append', default=[], metavar='PATH',
                    help="a path the run's own tool calls name: its work dir "
                         "(wk/<run>) or board. Repeatable; all must be named")
    ap.add_argument('--agent-id', default=None,
                    help='the subagent transcript agent-<ID>.jsonl')
    ap.add_argument('--projects-dir', default=None,
                    help='where Claude Code files transcripts '
                         '(default ~/.claude/projects)')
    ap.add_argument('--json', default=None)
    a = ap.parse_args(argv)
    filters = a.self_ or a.evidence or a.agent_id
    if a.session and filters:
        ap.error('pass a transcript OR --self/--evidence/--agent-id, not both')
    if a.session:
        path, sel = a.session, {'by': ['path'], 'candidates': 1}
    else:
        base = a.projects_dir or projects_dir()
        self_ = a.self_ or not filters
        by = (['self'] if self_ else []) + (['evidence'] if a.evidence else []) \
            + (['agent-id'] if a.agent_id else [])
        cands, scanned = select(base, self_=self_, evidence=a.evidence,
                                agent_id=a.agent_id)
        what = ' + '.join(by) + (' %s' % a.evidence if a.evidence else '') \
            + (' %s' % a.agent_id if a.agent_id else '')
        if len(cands) != 1:
            lines = [f'refuse: {len(cands)} of {scanned} transcript(s) under '
                     f'{base} match {what}; measuring needs exactly one']
            lines += ['  candidate: ' + _describe(c) for c in cands]
            if not cands and self_:
                lines.append('  (--self finds the agent RUNNING this command; '
                             'from elsewhere pass --agent-id or the transcript)')
            if len(cands) > 1:
                lines.append('  narrow with --agent-id, another --evidence, or '
                             'pass the transcript path')
            print('\n'.join(lines), file=sys.stderr)
            return REFUSED
        path = cands[0]['path']
        sel = {'by': by, 'evidence': a.evidence, 'candidates': 1,
               'scanned': scanned, 'projects_dir': base,
               **{k: cands[0][k] for k in ('cwd', 'session_id')}}
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        print(f'refuse: {path} is not a real non-empty file', file=sys.stderr)
        return REFUSED
    out = measure(path)
    out['selection'] = sel
    print(f"measured: {path} ({'sidechain' if out['sidechain'] else 'session'}"
          f"{', agent ' + out['agent_id'] if out['agent_id'] else ''}; "
          f"selected by {'+'.join(sel['by'])}, {sel['candidates']} candidate"
          f"{' of %d scanned' % sel['scanned'] if 'scanned' in sel else ''})",
          file=sys.stderr)
    print(json.dumps(out, indent=2))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
