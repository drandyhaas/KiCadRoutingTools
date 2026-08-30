#!/usr/bin/env python3
"""Two watchers for a long run: what went wrong, and how it could pass unearned.

Both are written for the Monitor contract -- ONE STDOUT LINE PER EVENT, exit
ends the watch -- so they can be armed once and left alone while the run
works.

WHY A SCRIPT AND NOT A GREP. A watcher that matches only the happy path is
silent through a crash, and silence is indistinguishable from "still
running". That is the failure this file exists to avoid, so `bugs` matches
the signatures you would ACT on -- tracebacks, refusals, leaks, rejected
laps, a `blocking` that went UP -- and `cheats` matches the ways a run could
report success without earning it: the answer key read, a scope narrowed to
the nets that were failing, a grader floor overridden.

Neither watcher BUDGETS on time. `cheats` ends when the run's DONE marker
appears -- a fact on disk, the same on every machine -- and `bugs` runs until
you stop it. Nothing here compares an elapsed time to a threshold and decides
something. The three `subprocess.run(..., timeout=)` values below are the
ordinary guard against a hung CHILD process, and each one is reported when it
fires rather than swallowed.

    # every new problem, as it appears (many events, until you stop it)
    python3 -X utf8 tests/stress/run_watch.py bugs --workdir wk/run20

    # ways the run could pass without earning it; ends at the DONE marker
    python3 -X utf8 tests/stress/run_watch.py cheats --workdir wk/run20 \
        --truthdir wk/run20_truth --done wk/run20/DONE

Exit codes: 0 normal, 2 usage, 1 from `--self-test` when a self-check fails.
`bugs` never exits on its own.
"""
import argparse
import hashlib
import json
import os
import shlex
import sys
import time

#: Substrings that mean something went wrong, in any text the run leaves
#: behind. Deliberately broad: a false positive costs one line, a missed
#: crash costs the whole run's credibility. Ordered roughly by severity so
#: the emitted label is the most specific one that matched.
SIGNATURES = (
    ('TRACEBACK', 'Traceback (most recent call last)'),
    ('LEAK', 'VERDICT: LEAK'),
    ('UNAIDED-VIOLATION', 'UnaidedViolation'),
    ('PROVENANCE', 'UNAIDED VIOLATION'),
    ('NOT-BUILDABLE', 'NOT BUILDABLE'),
    ('REFUSED', 'refusing'),
    ('REFUSED', 'REFUSED'),
    ('ERROR', 'Error:'),
    ('ERROR', 'ERROR'),
    ('FAILED', 'FAILED'),
    ('ASSERT', 'AssertionError'),
    ('DID-NOT-RUN', 'did not run'),
    ('UNAVAILABLE', 'unavailable'),
)

#: Files SCANNED for those signatures: tool output only. The run tees its own
#: output to `*.log`; the tools write `*.json` reports whose `skipped` blocks say
#: what could not be measured.
#:
#: `.md` is deliberately NOT here. The journal's job is to QUOTE tool output, so
#: every signature appears in it legitimately -- measured on run 20, scanning it
#: reported `NOT BUILDABLE` three times from a journal line reading
#: "NOT BUILDABLE -> buildable", i.e. the sentence recording that the defect was
#: FIXED. Narration is not an incident, and the incident it narrates was already
#: reported from the log it quotes.
SCAN_EXT = ('.log', '.txt', '.json', '.jsonl')

#: Ledger `kind`s whose scores form ONE comparable sequence. `converge record`
#: writes completion|placement|systemic (and classification, once added); a
#: routing score and a placement score grade different things, and a lap of any
#: other kind sits at the boundary between two routing sequences. 'routing' is
#: accepted defensively -- it is not a current choice, but the field is free text
#: on older ledgers.
ROUTING_KINDS = frozenset({'completion', 'routing'})

POLL_SEC = 5.0


def _self_output_ids():
    """(dev, ino) of whatever this watcher's own stdout/stderr point at.

    A watcher must not scan its own output, and this one did. Measured on run
    21, which armed `bugs` with `*> wk/run21/tigard/watch_bugs.log` -- inside
    --workdir, because that is where the run tees everything so the watchers
    can see it. Every line the watcher emitted then matched a signature on the
    next poll, so hit N quoted hit N-1 verbatim:

        NOT-BUILDABLE assembly0.json:1934: "verdict": "NOT BUILDABLE",
        NOT-BUILDABLE watch_bugs.log:2: NOT-BUILDABLE assembly0.json:1934: ...
        NOT-BUILDABLE watch_bugs.log:3: NOT-BUILDABLE watch_bugs.log:2: ...

    Two hits become four, four become eight, and the ONE real finding is buried
    under nesting copies of itself. This is the same class as the `.md`
    exclusion above -- narration is not an incident -- except self-amplifying,
    so it destroys the log rather than padding it.

    Identified by inode rather than by name because the redirect target is
    chosen by whoever arms the watcher, not by this file; a name list would
    only cover the convention this repo happens to use today. `_is_self_output`
    keeps that convention as the fallback for platforms reporting st_ino 0.
    """
    ids = set()
    for fd in (1, 2):
        try:
            st = os.fstat(fd)
        except OSError:
            continue
        if st.st_ino:
            ids.add((st.st_dev, st.st_ino))
    return ids


#: Computed once, at import: the redirect cannot change under a running watcher.
_SELF_OUTPUT_IDS = _self_output_ids()


def _is_self_output(path):
    """Is this file a WATCHER's output rather than a tool's?

    Own stdout/stderr by inode, and any sibling watcher's log by the
    `watch_*.log` convention. The sibling half matters as much as the own
    half: two watchers over one work dir would otherwise quote each other,
    and neither of them is reporting anything the RUN produced.
    """
    try:
        st = os.stat(path)
    except OSError:
        return False
    if st.st_ino and (st.st_dev, st.st_ino) in _SELF_OUTPUT_IDS:
        return True
    name = os.path.basename(path)
    return name.startswith('watch_') and name.endswith('.log')


def _walk(workdir):
    """Files under `workdir` with a SCAN_EXT extension, never a watcher's own.

    `.md` is excluded by SCAN_EXT and that is deliberate: the journal QUOTES
    tool output, so scanning it re-reports defects that were already reported
    from the log it quotes.
    """
    for root, dirs, files in os.walk(workdir):
        dirs[:] = [d for d in dirs if not d.startswith('.')]
        for name in sorted(files):
            if not name.endswith(SCAN_EXT):
                continue
            path = os.path.join(root, name)
            if _is_self_output(path):
                continue
            yield path


def _unchanged(path, stamps):
    """True when `path` has not changed since the last poll.

    Purely a cost guard: `seen` already stops re-emission, but every poll
    re-read every watched file end to end, and a long run's logs grow without
    bound.
    """
    try:
        st = os.stat(path)
        now = (st.st_mtime, st.st_size)
    except OSError:
        return False
    if stamps.get(path) == now:
        return True
    stamps[path] = now
    return False


def _scan_text(path, seen, rel):
    """Emit one line per NEW signature hit in this file."""
    out = []
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            lines = f.readlines()
    except OSError:
        return out
    for i, line in enumerate(lines):
        for label, needle in SIGNATURES:
            if needle in line:
                key = (rel, label, line.strip()[:160])
                if key in seen:
                    break
                seen.add(key)
                out.append(f'{label} {rel}:{i + 1}: {line.strip()[:200]}')
                break
    return out


def _scan_ledger(path, seen, rel):
    """A rejected lap, or a `blocking` that went UP, is a finding too.

    The ledger is the run's own record of what it decided, so a regression
    here is more trustworthy than any log line -- it is what the loop acted
    on, not what a tool printed.

    BLOCKING-UP compares CONSECUTIVE KEPT ROUTING LAPS, and both qualifiers were
    learned from a false positive. Run 20 emitted

        BLOCKING-UP ledger.jsonl: iteration 35 blocking 10 -> 82

    which compared cycle 2's FIRST scored lap against cycle 1's FINAL one. Two
    local rules fix it without teaching this watcher what a cycle is:

      * advance `prev` only on `accepted is True`. A rejected lap's score is the
        measurement that got it rejected; the loop already reverted it, and
        REJECTED-LAP reports it. Feeding it to `prev` reports the revert as a
        regression.
      * reset `prev` at any row whose `kind` is not a routing kind. A placement,
        systemic or classification lap sits BETWEEN two routing sequences and is
        the boundary -- scores either side of it grade different boards.

    Measured on run 20's ledger: zero BLOCKING-UP, against one false positive
    today. Deliberately NOT done by importing `loop_driver._cycle_index`: a
    cross-tree import from tests/stress into .claude/skills/.../scripts would tie
    a watcher to a driver that ships independently.
    """
    out, prev = [], None
    try:
        with open(path, encoding='utf-8') as f:
            rows = [json.loads(x) for x in f if x.strip()]
    except (OSError, ValueError):
        return out
    for r in rows:
        if not isinstance(r, dict):
            continue
        it = r.get('iteration')
        if r.get('accepted') is False:
            key = (rel, 'rejected', it)
            if key not in seen:
                seen.add(key)
                out.append(f'REJECTED-LAP {rel}: iteration {it} '
                           f'lever={r.get("lever")} was not accepted')
        sc = r.get('score') if isinstance(r.get('score'), dict) else {}
        blk = sc.get('blocking')

        # A lap that recorded `blocking: null` is a finding about the
        # MEASUREMENT, not about the sequence, so it is reported for every lap --
        # rejected and non-routing ones included. Keep it outside the
        # comparison rules below.
        if not isinstance(blk, (int, float)) and 'blocking' in sc:
            key = (rel, 'blocking-none', it)
            if key not in seen:
                seen.add(key)
                out.append(f'BLOCKING-NULL {rel}: iteration {it} reports '
                           f'blocking=None -- "0 violations" and "0 rules '
                           f'ran" are different answers')

        if (r.get('kind') or 'completion') not in ROUTING_KINDS:
            prev = None            # a half boundary: the next score grades a
            continue               # different board than the last one did
        if r.get('accepted') is not True:
            continue               # a reverted lap is not a step in the sequence
        if not isinstance(blk, (int, float)):
            continue               # unmeasured: evidence in neither direction
        if prev is not None and blk > prev:
            key = (rel, 'regress', it)
            if key not in seen:
                seen.add(key)
                out.append(f'BLOCKING-UP {rel}: iteration {it} '
                           f'blocking {prev} -> {blk}')
        prev = blk
    return out


def watch_bugs(workdir, poll):
    seen, stamps = set(), {}
    print(f'WATCHING {os.path.abspath(workdir)} for problems '
          f'({len(SIGNATURES)} signatures + the ledger), '
          f'scanning {" ".join(SCAN_EXT)}', flush=True)
    while True:
        if os.path.isdir(workdir):
            for path in _walk(workdir):
                if _unchanged(path, stamps):
                    continue
                rel = os.path.relpath(path, workdir).replace('\\', '/')
                if path.endswith('.jsonl'):
                    for line in _scan_ledger(path, seen, rel):
                        print(line, flush=True)
                else:
                    for line in _scan_text(path, seen, rel):
                        print(line, flush=True)
        time.sleep(poll)


#: Tool groups a flag is scoped to. `()` means "any tool".
GRADERS = ('check_drc.py', 'board_score.py', 'check_complete.py')
ROUTERS = ('route.py', 'route_diff.py', 'route_planes.py', 'repair_planes.py')
#: `route_diff.py --nets DP DN` and `route_planes.py --nets GND +3V3` are what
#: those tools ARE -- naming the pair or the pour is the invocation, not a
#: narrowing. Only the general router can narrow its own scope.
NET_SCOPED = ('route.py',)

#: Flags that make a run report success by lowering the bar rather than
#: clearing it. Each is legitimate SOMEWHERE -- that is why they exist -- so
#: this reports the observation and the reason it matters, never a verdict.
#: The run is answerable for them; the watcher only makes them visible.
#:
#: Fourth field: the tools on which the flag lowers a bar. Measured on run 20 --
#: 683 raw substring hits, 24 once anchored to a real command line, 6 once
#: scoped to the tools where the flag means what the `why` says.
#: The two skill drivers. They install no cli_banner, so their invocations are
#: only ever seen through a teed `cmd_timing.jsonl` row -- see _scan_ledger_argv.
DRIVERS = ('placement_driver.py', 'loop_driver.py')
#: Every CLI that calls `fab_tiers.add_fab_tier_args`, which is what actually
#: supplies `--fab-tier` / `--fab-overrides`. Scoped to GRADERS + ROUTERS
#: these were invisible on the fanouts -- and a fanout at `--fab-tier
#: advanced` is precisely the run-22 ratchet the entries exist for.
FAB_SCOPED = ROUTERS + GRADERS + (
    'bga_fanout.py', 'qfn_fanout.py', 'fix_kicad_drc_settings.py',
    'list_nets.py', 'check_reachability.py')


CHEAT_FLAGS = (
    ('WAIVER', '--accept-residue', 'accepts placement residue the close-out '
                                   'refused', ()),
    ('WAIVER', '--accept-unclosed', 'closes a half that did not close', ()),
    ('WAIVER', '--accept-congestion', 'accepts a congestion regression', ()),
    ('WAIVER', '--allow-unplaced', 'proceeds on a board the gate called '
                                   'unplaced', ()),
    ('WAIVER', '--allow-routed', 'places onto existing copper', ()),
    ('WAIVER', '--force', 'overrides a refusal', ()),
    # A GIT flag, not one of this repo's: `git commit --no-verify` skips the
    # hooks. Scoped to the empty tuple like the other tool-agnostic entries,
    # which is what lets it fire on a bare `git` line rather than needing a
    # KRT tool to declare it.
    ('WAIVER', '--no-verify', 'skips a git hook', ()),
    ('FLOOR', '--clearance', 'overrides the clearance the BOARD declares. A '
                             'looser value makes DRC pass by grading what was '
                             'not routed', GRADERS + ROUTERS),
    ('FLOOR', '--board-edge-clearance', 'overrides the board edge floor',
     GRADERS + ROUTERS),
    ('FLOOR', '--clearance-margin', 'filters DRC hits by distance', GRADERS),
    ('SCOPE', '--nets', 'routes a SUBSET -- "0 unrouted" over 6 of 84 nets '
                        'is not the same claim', NET_SCOPED),
    # `--ignore-nets` HAS NO ENTRY, deliberately. The flag exists only on
    # `place_optimize.py`, `place_seed.py`, `place_portfolio.py`,
    # `compare_seeds.py` and `render_placement.py`, where it scopes AIRWIRE
    # SCORING -- it is how you tell the panel not to draw 80 GND stubs -- and
    # reporting it there buried the real uses in run 20. No tool that
    # produces a VERDICT declares it, so an entry scoped to the graders or
    # the routers could never fire, which reads on the page as coverage this
    # watcher does not have. Named rather than silently omitted.
    ('SCOPE', '--rip-existing-nets', 'rips copper an earlier step placed',
     ROUTERS),
    ('DELEGATION', '--no-delegate', 'runs an inner half inline; an inline '
                                    'loop can silently do the outer loop\'s '
                                    'job and never report it (run 14)', ()),
    # --- run-22 additions ---------------------------------------------------
    # A gap in an already-covered family, not a category decision: the three
    # sibling `--accept-*` flags were listed and `--waive` was not. Measured:
    # run 22 used --waive FIVE times and this watcher emitted ZERO events.
    ('WAIVER', '--waive', 'waives a lock advisory the P3 gate raised, per ref',
     DRIVERS),
    # The fab floor is a CEILING on what the router may emit. Lowering it lets
    # copper reach 0.0889/0.25 AND grade clean there -- the run-22 ratchet: a
    # board reported unrouted 0 / broken 0 while carrying 39 objects below its
    # own declared floors.
    ('FLOOR', '--fab-tier', 'routes and grades at a LOOSER fab tier '
                            '(advanced reaches 0.0762 track, 0.25/0.15 via)',
     FAB_SCOPED),
    ('FLOOR', '--fab-overrides', 'pins the fab floor to a file, which may sit '
                                 'under the standard tier', FAB_SCOPED),
    # Without the writeback there is no `fab_floor_origin` in the output
    # project, so check_complete.fab_floor_integrity reports ran: False and
    # the ratchet check goes dark.
    ('WAIVER', '--no-fix-drc-settings', 'leaves the output with no '
                                        'fab_floor_origin, so the fab-floor '
                                        'integrity check cannot run at all',
     ROUTERS),
)

def _parse_cmd(line):
    """`CMD: python3 -X utf8 route.py b.kicad_pcb --nets '*'` -> ('route.py', [tokens]).

    The anchor is `py_router/cli_banner.py`, which prints exactly one
    `CMD: <argv>` line per tool run, built from `sys.orig_argv`, and one
    `EXIT=<rc>`. 27 tools install it and two tests pin that there is exactly one
    of each per log. Anchoring here is what separates a flag that was PASSED from
    a flag that was merely MENTIONED -- in help text, in a tool's own
    "--clearance not given" disclosure, in a driver's refusal telling you to pass
    it, or in a ledger `lever` sentence. Returns None for any other line.
    """
    if not line.startswith('CMD: '):
        return None
    try:
        toks = shlex.split(line[5:].strip(), posix=False)
    except ValueError:
        toks = line[5:].strip().split()
    tool = ''
    for t in toks:
        low = t.strip('"\'').lower()
        if low.endswith('.py'):
            tool = os.path.basename(low)
            break
    return tool, toks


def _flag_in(toks, flag):
    """Token-exact flag match, including the `--flag=value` form.

    Substring matching is what made `--clearance` fire on `--clearance-margin`
    and `--nets` fire on `--ignore-nets`.
    """
    for t in toks:
        if t == flag or t.startswith(flag + '='):
            return True
    return False


def _flag_value(toks, flag):
    """First value after `flag` (or after `flag=`), stripped of quotes."""
    for i, t in enumerate(toks):
        if t.startswith(flag + '='):
            return t.split('=', 1)[1].strip('"\'')
        if t == flag and i + 1 < len(toks):
            return toks[i + 1].strip('"\'')
    return ''


def _flag_values(toks, flag):
    """Every value belonging to `flag`, up to the next option. List-taking flags
    (`--nets A B C`) are the norm here, so one value is not enough."""
    vals, grabbing = [], False
    for t in toks:
        if t.startswith(flag + '='):
            vals.append(t.split('=', 1)[1].strip('"\''))
            continue
        if t == flag:
            grabbing = True
            continue
        if grabbing:
            if t.startswith('--'):
                grabbing = False
                continue
            vals.append(t.strip('"\''))
    return vals


def _use_key(label, flag, tool, toks):
    """Identity of a bar-lowering USE: which tool, which flag, applied to what.

    Deliberately NOT the whole argv. The same lever reaches this watcher spelled
    two ways -- a log's `CMD:` line carries absolute board paths and shell
    quoting, the ledger's `lever_argv` carries the relative form the run recorded
    -- and on run 20 those two differed in token count and quoting while naming
    the identical tool, flags and nets. Keying on the argv reported one lever
    twice; keying on what the flag was applied to reports it once, and still
    separates two genuinely different scopes of the same flag.
    """
    vals = _flag_values(toks, flag)
    payload = f'{tool}|{flag}|' + '\x1f'.join(sorted(vals))
    return (label, flag, tool,
            hashlib.sha1(payload.encode('utf-8', 'replace')).hexdigest()[:16])


#: JSONL keys that carry a REAL argv, in priority order. `lever_argv` is a
#: converge lever; `argv` / `cmdline` are what tests/stress/tee_cmd.py records
#: for EVERY invocation a timed run makes. Run 22 wrote 203 such rows and this
#: watcher extracted nothing from any of them, because it looked only for
#: `lever_argv` -- so every driver invocation was invisible (the drivers
#: install no cli_banner, so they print no CMD: line of their own either).
_ARGV_KEYS = ('lever_argv', 'argv', 'cmdline')


def _scan_ledger_argv(path, seen, rel):
    """Cheat flags in a ledger row's `lever_argv` -- the second source of truth.

    `lever_argv` is a real command the run executed (converge refuses one whose
    argv[0] could never replay). `lever` beside it is free prose and is NEVER
    matched: run 20's levers narrate the flags they used, which is exactly the
    disclosure the run is supposed to make, and reporting it as a finding
    punishes the disclosure.
    """
    out = []
    try:
        with open(path, encoding='utf-8') as f:
            rows = [json.loads(x) for x in f if x.strip()]
    except (OSError, ValueError):
        return out
    for r in rows:
        if not isinstance(r, dict):
            continue
        argv = src = None
        for _k in _ARGV_KEYS:
            if r.get(_k) is not None:
                argv, src = r[_k], _k
                break
        if isinstance(argv, str):
            try:
                argv = shlex.split(argv, posix=False)
            except ValueError:
                argv = argv.split()
        if not isinstance(argv, list) or not argv:
            continue
        toks = [str(t) for t in argv]
        tool = ''
        for t in toks:
            if t.lower().endswith('.py'):
                tool = os.path.basename(t.lower())
                break
        for label, flag, why in _cheat_hits(tool, toks):
            key = _use_key(label, flag, tool, toks)
            if key in seen:
                continue
            seen.add(key)
            where = (f'iteration {r.get("iteration")}'
                     if src == 'lever_argv' else f'label {r.get("label")!r}')
            out.append(f'{label} {flag} in {rel} {src} {where} '
                       f'({tool}) -- {why}')
    return out


def _cheat_hits(tool, toks):
    """The (label, flag, why) triples this command line genuinely earns."""
    out = []
    for label, flag, why, tools in CHEAT_FLAGS:
        if tools and tool not in tools:
            continue
        if not _flag_in(toks, flag):
            continue
        # `--nets '*'` is the whole board: the opposite of a narrowing, and the
        # form #562 requires so the plane finalize keeps the pours in scope.
        if flag == '--nets' and _flag_value(toks, flag) in ('*', '"*"', "'*'"):
            continue
        out.append((label, flag, why))
    return out

#: Directories whose tracked contents a RUN has no business changing. Editing
#: the toolchain to make a gate pass is the cheat that leaves the most
#: convincing evidence of success.
GUARDED_DIRS = ('py_router', 'py_placer', 'py_tools', 'tests', '.claude',
                'kicad_files')


def _sha(path):
    import hashlib
    h = hashlib.sha256()
    try:
        with open(path, 'rb') as f:
            for chunk in iter(lambda: f.read(1 << 20), b''):
                h.update(chunk)
    except OSError:
        return None
    return h.hexdigest()


def _repo_root():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


def _git_dirty(root):
    """Tracked files changed under the guarded dirs, via git."""
    import subprocess
    try:
        r = subprocess.run(['git', '-C', root, 'status', '--porcelain'],
                           capture_output=True, text=True, timeout=60)
    except Exception:                                  # noqa: BLE001
        return None                                    # cannot tell -- say so
    if r.returncode != 0:
        # NOT a clean tree. `git status` exits non-zero when `root` is not a
        # repository, when the index is locked, when git is missing -- and
        # stdout is then empty, so an unchecked read printed "git baseline
        # clean" precisely in the cases where it could not tell.
        return None
    out = []
    for line in (r.stdout or '').splitlines():
        path = line[3:].strip().strip('"')
        if path.split('/')[0] in GUARDED_DIRS:
            out.append(f'{line[:2].strip()} {path}')
    return out


def watch_cheats(workdir, truthdir, done_path, poll):
    """Ways this run could report success without earning it.

    NOT an accusation channel. Every flag below is legitimate somewhere, and
    a run may have a good reason; what it may not do is use one silently. So
    each line states what was seen and why it matters, and the run answers.

    What this CANNOT see, stated so nobody reads silence as proof: it cannot
    tell that a file was read. Blindness to the control is enforced after the
    fact by `fence_audit` -- which compares the DELIVERED BOARD's poses, so
    bypassing the tool does not bypass the check -- and that is run here when
    the run declares itself done.
    """
    root = _repo_root()
    seen = set()
    truth0 = {}
    if truthdir and os.path.isdir(truthdir):
        for r, _d, fs in os.walk(truthdir):
            for n in fs:
                p = os.path.join(r, n)
                truth0[os.path.relpath(p, truthdir)] = _sha(p)
    dirty0 = _git_dirty(root)
    print(f'WATCHING for shortcuts: {len(CHEAT_FLAGS)} flags, '
          f'{len(truth0)} truth file(s) fingerprinted, '
          + ('git baseline clean'
             if dirty0 == [] else
             f'git baseline has {len(dirty0)} pre-existing change(s)'
             if dirty0 else 'git unavailable -- toolchain edits INVISIBLE'),
          flush=True)

    while True:
        # 1. the truth dir must not move. It is the answer key.
        if truthdir and os.path.isdir(truthdir):
            for r, _d, fs in os.walk(truthdir):
                for n in fs:
                    p = os.path.join(r, n)
                    rel = os.path.relpath(p, truthdir)
                    now = _sha(p)
                    was = truth0.get(rel)
                    if was is None and rel not in truth0:
                        key = ('truth-new', rel)
                        if key not in seen:
                            seen.add(key)
                            print(f'TRUTH-CHANGED a file appeared in the '
                                  f'truth dir after staging: {rel}',
                                  flush=True)
                    elif now != was:
                        key = ('truth-mod', rel)
                        if key not in seen:
                            seen.add(key)
                            print(f'TRUTH-CHANGED {rel} was modified after '
                                  f'staging -- the control is the thing the '
                                  f'fence compares against', flush=True)

        # 2. the toolchain must not move either.
        dirty = _git_dirty(root)
        if dirty is not None and dirty0 is not None:
            for row in dirty:
                if row in dirty0:
                    continue
                key = ('git', row)
                if key not in seen:
                    seen.add(key)
                    print(f'TOOLCHAIN-EDIT {row} -- a gate that passes '
                          f'because the gate changed has measured nothing',
                          flush=True)

        # 3. flags that lower the bar, and repeat staging.
        #
        # Two sources of truth, and only two. A flag that appears anywhere else
        # was MENTIONED, not passed:
        #   * a `CMD:` line, written by cli_banner from sys.orig_argv (27 tools)
        #   * a ledger row's `lever_argv`, which IS an executed command --
        #     never its `lever`, which is prose. converge.py deliberately does
        #     not install cli_banner ("converge's stdout is a JSON API"), so its
        #     invocations reach us only this way.
        stages = 0
        unanchored = 0
        for path in _walk(workdir):
            if not path.endswith(('.log', '.txt', '.jsonl')):
                continue
            rel = os.path.relpath(path, workdir).replace('\\', '/')
            try:
                with open(path, encoding='utf-8', errors='replace') as f:
                    lines = f.readlines()
            except OSError:
                continue
            for i, line in enumerate(lines):
                parsed = _parse_cmd(line)
                if parsed is None:
                    # Count, do not report. Without this the redesign is
                    # unfalsifiable -- a reader cannot tell "precise" from
                    # "blind", and a tool that stops printing CMD: would go
                    # silent instead of announcing itself.
                    if any(f in line for _l, f, _w, _t in CHEAT_FLAGS):
                        unanchored += 1
                    continue
                tool, toks = parsed
                if any(t.endswith('stage_blind.py') for t in toks):
                    stages += 1
                for label, flag, why in _cheat_hits(tool, toks):
                    key = _use_key(label, flag, tool, toks)
                    if key in seen:
                        continue
                    seen.add(key)
                    print(f'{label} {flag} in {rel}:{i + 1} ({tool}) -- {why} '
                          f':: {line.strip()[:140]}', flush=True)
            if path.endswith('.jsonl'):
                for out in _scan_ledger_argv(path, seen, rel):
                    print(out, flush=True)
        # Key on the FACT, not the count. The running count was in the key,
        # so every increment was a fresh event: the line promised "counted,
        # not reported individually" and then reported every increment.
        if unanchored and ('unanchored',) not in seen:
            seen.add(('unanchored',))
            print(f'NOTE {unanchored} flag mention(s) outside a CMD: line '
                  f'(prose, help text, a tool disclosing "--clearance not '
                  f'given", a driver refusal naming the flag, a ledger lever) '
                  f'-- counted, not reported individually. If this number is '
                  f'0 while a run is clearly using tools, a tool has stopped '
                  f'printing its CMD: banner and this watcher has gone blind.',
                  flush=True)
        if stages > 1 and ('restage',) not in seen:
            seen.add(('restage',))
            print(f'RESTAGE the log shows {stages} staging invocations -- a '
                  f'second draw is a different (possibly easier) damage, and '
                  f'the run must say which one it reports', flush=True)

        # 4. done: run the two audits that check the board rather than the log.
        if os.path.exists(done_path):
            print('DONE declared -- running the audits that read the BOARD, '
                  'not the log', flush=True)
            import subprocess
            control = os.path.join(truthdir or '', 'control.kicad_pcb')
            if truthdir and os.path.isfile(control):
                try:
                    r = subprocess.run(
                        [sys.executable, '-X', 'utf8',
                         os.path.join(root, 'tests', 'stress',
                                      'fence_audit.py'),
                         '--control', control, '--workdir', workdir],
                        capture_output=True, text=True, timeout=900)
                    _said = False
                    for line in (r.stdout or '').splitlines():
                        if 'VERDICT' in line or line.startswith('  LEAK'):
                            print(f'FENCE {line.strip()}', flush=True)
                            _said = True
                    # 0 CLEAN and 4 LEAK are the audit's ANSWERS. Anything
                    # else is the audit failing to run, and printing nothing
                    # for it reads exactly like a clean fence -- the failure
                    # this whole file exists to avoid.
                    if r.returncode not in (0, 4) or not _said:
                        _tail = ((r.stderr or r.stdout or '').strip()
                                 .splitlines() or [''])[-1]
                        print(f'FENCE did NOT report a verdict '
                              f'(exit {r.returncode}): {_tail[:160]} -- that '
                              f'is not a pass', flush=True)
                except Exception as e:                 # noqa: BLE001
                    print(f'FENCE could not run ({type(e).__name__}: {e}) -- '
                          f'that is not a pass', flush=True)
            else:
                print('FENCE no control board found, so blindness was NOT '
                      'verified -- that is not a pass', flush=True)
            try:
                r = subprocess.run(
                    [sys.executable, '-X', 'utf8',
                     os.path.join(root, 'tests', 'stress',
                                  'provenance_audit.py'),
                     '--workdir', workdir],
                    capture_output=True, text=True, timeout=900)
                _said = False
                for line in (r.stdout or '').splitlines():
                    if line.startswith('VERDICT') or 'unclaimed' in line:
                        print(f'PROVENANCE {line.strip()}', flush=True)
                        _said = True
                # 0/4/5 are its verdicts (CLEAN / VIOLATION / UNPROVEN); 2 is
                # a usage error and anything else is a crash. Silence there
                # reads as CLEAN.
                if r.returncode not in (0, 4, 5) or not _said:
                    _tail = ((r.stderr or r.stdout or '').strip()
                             .splitlines() or [''])[-1]
                    print(f'PROVENANCE did NOT report a verdict '
                          f'(exit {r.returncode}): {_tail[:160]} -- that is '
                          f'not a pass', flush=True)
                if r.returncode == 4:
                    print('PROVENANCE exit 4 -- a pose in the delivered board '
                          'traces to no registered lever, i.e. something '
                          'moved parts that was not the engine', flush=True)
            except Exception as e:                     # noqa: BLE001
                print(f'PROVENANCE could not run ({type(e).__name__}: {e})',
                      flush=True)
            return 0
        time.sleep(poll)


def _self_test():
    """Checkable in the field, where the harness runs and pytest does not.

    Mirrors `loop_driver.py --self-test`. The full fixtures live in
    tests/test_run20_run_watch.py; this is the subset that needs no files.
    """
    bad = []

    def want(cond, label):
        if not cond:
            bad.append(label)

    # The anchor: only a real CMD: line is a use.
    want(_parse_cmd('CMD: python3 -X utf8 py_router/check_drc.py b.kicad_pcb '
                    '--clearance 0.2')[0] == 'check_drc.py',
         'CMD: line should yield its tool')
    for prose in ('  --clearance not given; honoring net classes',
                  'retry with --rip-existing-nets to authorize',
                  'usage: route.py [-h] [--nets NETS [NETS ...]]',
                  '   python3 $D --stage L2 --accept-residue oob_pad_count'):
        want(_parse_cmd(prose) is None, f'prose must not parse: {prose[:40]}')

    # Token-exact: the substrings that made 683 hits.
    want(not _flag_in(['--clearance-margin', '0.1'], '--clearance'),
         '--clearance must not match inside --clearance-margin')
    want(not _flag_in(['--ignore-nets', 'GND'], '--nets'),
         '--nets must not match inside --ignore-nets')
    want(_flag_in(['--nets=A'], '--nets'), '--flag=value form must match')

    # Tool scoping.
    want(not _cheat_hits('route_diff.py', ['--nets', 'DP', 'DN']),
         'naming the pair is what route_diff IS')
    want(_cheat_hits('route.py', ['--nets', 'BUSY', 'SCK']),
         'route.py narrowing its own scope is a real disclosure')
    want(not _cheat_hits('route.py', ['--nets', '*']),
         '--nets * is the whole board, the opposite of a narrowing')
    want(_cheat_hits('check_drc.py', ['--clearance', '0.2']),
         'a grader clearance is a floor override')

    # One lever recorded two ways is one event.
    log = ['python3.exe', '-X', 'utf8', 'C:/r/py_router/route.py',
           'C:/r/wk/a.kicad_pcb', '--nets', 'CS', '/40M_N']
    led = ['python3', '-X', 'utf8', 'py_router/route.py',
           'wk/a.kicad_pcb', '--nets', '/40M_N', 'CS']
    want(_use_key('SCOPE', '--nets', 'route.py', log)
         == _use_key('SCOPE', '--nets', 'route.py', led),
         'the same lever spelled two ways must dedupe to one event')
    want(_use_key('SCOPE', '--nets', 'route.py', log)
         != _use_key('SCOPE', '--nets', 'route.py',
                     ['route.py', '--nets', 'BUSY']),
         'a different scope must remain a separate event')

    # Prose files: narration is not an incident.
    want('.md' not in SCAN_EXT,
         'the journal QUOTES tool output; scanning it re-reports fixed defects')

    print('FAIL: ' + '; '.join(bad) if bad else 'OK')
    return 1 if bad else 0


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if '--self-test' in list(argv):
        return _self_test()
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='mode', required=True)

    b = sub.add_parser('bugs', help='one event per new problem, forever')
    b.add_argument('--workdir', required=True)
    b.add_argument('--poll', type=float, default=POLL_SEC)

    c = sub.add_parser('cheats', help='ways the run could pass without '
                                      'earning it; ends at DONE')
    c.add_argument('--workdir', required=True)
    c.add_argument('--truthdir', help='the answer key, fingerprinted at arm '
                                      'time and re-checked (default: '
                                      'WORKDIR + "_truth")')
    c.add_argument('--done', help='path whose existence triggers the final '
                                  'audits (default: WORKDIR/DONE)')
    c.add_argument('--poll', type=float, default=POLL_SEC)

    a = p.parse_args(argv)
    # An empty WORKDIR is what an unset shell variable looks like, and it used
    # to reach os.makedirs('') and traceback -- which in a watcher reads as
    # "the thing I am watching crashed", not "you passed nothing".
    if not (a.workdir or '').strip():
        p.error('--workdir is empty (an unset shell variable?), so there is '
                'nothing to watch')
    if a.mode == 'bugs':
        try:
            watch_bugs(a.workdir, a.poll)
        except KeyboardInterrupt:
            return 0
        return 0
    truth = a.truthdir or (a.workdir.rstrip('/\\') + '_truth')
    done = a.done or os.path.join(a.workdir, 'DONE')
    try:
        return watch_cheats(a.workdir, truth, done, a.poll)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
