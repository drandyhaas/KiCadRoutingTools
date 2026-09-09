#!/usr/bin/env python3
"""Surgical convergence: rank a move before paying for it, and step back cheaply.

A convergence run is expensive in exactly one way -- routing -- and a loop that
re-runs a full chain for every candidate spends its whole budget discovering
things a millisecond of arithmetic already knew. Measured on one board: eleven
full-chain iterations bought about eight useful moves, and the run stopped with
five nets carrying no copper.

So: a ladder, cheapest evidence first, stopping as soon as a tier discriminates.

    tier 1  legality            QuenchState.candidate_valid      ms
    tier 2  placement cost      QuenchState.total_cost           ms
    tier 3  scoped route        route.py --nets <affected>       seconds
    tier 4  full chain          the caller's own                 minutes

Tiers 1 and 2 live in pose_score.py. This module adds tier 3 -- routing only the
nets a move can affect -- and the bookkeeping that makes a step back a checkout
rather than a reconstruction.

VERBS

    converge.py poses BOARD --ref U3 [--route] [--affected NET ...]
        Rank the part's candidate poses. With --route, also run tier 3 on the
        top few and report what actually happened to the copper.

    converge.py where BOARD --nets NET ...
        What is unconnected, where the gap is, and which foreign copper is
        walling it in -- via net_forensics, which already answers this and which
        nothing in the usual chain calls.

    converge.py record --ledger L --board B --kind completion --argv ...
        Store a board by content and record what produced it.

    converge.py step-back --ledger L [--to SHA|--iteration N] --out BOARD
        Check out an earlier board. Exact, because it is addressed by content.

    converge.py replay --ledger L --iteration N
        Re-run that iteration's lever verbatim. An entry that recorded only
        prose refuses, loudly.

    converge.py status --ledger L
        Iterations spent, split completion vs systemic. A budget going to the
        instrument rather than the board is the failure this makes visible.
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import time

# ROOT is the REPO root (this script lives in py_placer/), because the
# subprocesses below run with cwd=ROOT and their paths are repo-relative.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROUTE_PY = os.path.join(ROOT, 'py_router', 'route.py')


# --------------------------------------------------------------------- tier 3

def scoped_route(board, nets, out=None, extra_args=()):
    """Route ONLY `nets`, and return the merged summary. Seconds, not minutes.

    This is the tier that actually discriminates: placement cost says a pose
    looks better, and only a route says the copper agrees. Scoping it to the
    affected nets is what makes it affordable enough to run per candidate --
    and that SCOPE is the bound. There is no `timeout`: it existed only to
    serve the two probe budgets #713 item 2 removed, no caller passes one now,
    and leaving the parameter would invite a clock straight back into a
    comparison. The temp dir is deliberately NOT cleaned up -- callers read
    `res['board']` after the call returns.
    """
    tmp = tempfile.mkdtemp(prefix='converge_t3_')
    out = out or os.path.join(tmp, 'routed.kicad_pcb')
    js = os.path.join(tmp, 'route.json')
    argv = [sys.executable, '-X', 'utf8', _ROUTE_PY, board, out,
            '--nets'] + list(nets) + ['--json-out', js] + list(extra_args)
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    summary = {}
    # A summary we cannot READ is a non-verdict, not an exception. This file
    # can exist and not parse -- a killed router, ENOSPC, a flush that fails at
    # close, a stale file at a reused path -- and route.py exits 0 on that path
    # having printed only a WARNING. Raising here does not spoil one candidate;
    # it empties the CALLER's loop, and none of the four call sites catch it
    # (cmd_poses has no try at all; place_portfolio._probe and compare_seeds
    # catch only subprocess.TimeoutExpired). compare_seeds collects its rows
    # and writes seeds.json only AFTER the loop, so a throw on seed 3 of 8
    # ships no document at all, at an exit code not in its own vocabulary.
    #
    # `summary` stays {} -- the no-verdict channel every caller already handles
    # -- and `summary_error` says WHICH absence this was, because the return
    # code is 0 either way and the router's own WARNING has usually fallen out
    # of the 1500-character stdout_tail by the time we return:
    #     summary truthy           -> a verdict
    #     {} + summary_error None  -> the router wrote nothing
    #     {} + summary_error set   -> the router wrote something unreadable
    # Same shape, and the same reasoning, as _load_defects below.
    #
    # (OSError, ValueError) rather than the narrower JSONDecodeError: json.load
    # decodes the text handle first, so a tail cut mid-UTF-8-sequence raises
    # UnicodeDecodeError -- a ValueError that is NOT a JSONDecodeError.
    summary_error = None
    if os.path.isfile(js):
        try:
            with open(js, encoding='utf-8') as f:
                summary = json.load(f)
        except (OSError, ValueError) as exc:                    # noqa: BLE001
            summary_error = f'{type(exc).__name__}: {exc}'
            # stderr, never stdout: `converge poses` pipes a JSON document.
            print(f"  WARNING: unreadable route summary {js} "
                  f"({summary_error}); rc={r.returncode} -- treating as "
                  f"no summary", file=sys.stderr)
    return {'argv': argv, 'returncode': r.returncode, 'board': out,
            'json': js, 'summary': summary, 'summary_error': summary_error,
            'stdout_tail': r.stdout[-1500:]}


#: A probe row's `status`, and the ONLY place the vocabulary is written down.
#: `ok` is the one value carrying a verdict; the rest are ways of not having
#: one, and before #713 item 2 they all shared a single `failures: None` that
#: no consumer inspected -- so a route.py crash, a "No nets matched" exit, a
#: deliberate ratsnest skip and a wall-clock timeout were the same row.
PROBE_STATUSES = ('ok', 'crashed', 'no_summary', 'screened')


def probe_route(board, nets, extra_args=(), out=None, note_prefix=''):
    """One probe route -> a JSON-safe verdict row, shaped IDENTICALLY whatever
    happens.

    ONE helper because there were two, in place_portfolio and compare_seeds,
    whose timeout rows dropped different subsets of the success row's keys
    (`vias`/`returncode` in one; `probe_kind`/`iterations`/`vias`/`returncode`
    in the other). tests/test_compare_seeds.py:96 reads `probe_kind`
    unconditionally, so it was a latent KeyError on any timed-out row rather
    than the assertion it looks like.

    NO TIMEOUT. Both callers used to wrap this in `subprocess.run(timeout=...)`
    -- 900 s in one, 1800 s in the other -- and record the expiry as
    `failures: None`, which both then DROPPED from their rankings. A candidate
    whose verdict a clock erased is not ranked worse; it stops being a
    contender. route.py has no self-budget by design (#621) and the repo's own
    main loop, place_route_loop._run_route_cmd, already calls it unbounded. The
    bound here is SCOPE: `nets`, which both callers already pass.
    """
    res = scoped_route(board, nets, out=out, extra_args=list(extra_args))
    summary = res['summary']
    if not summary:
        # Two different absences, kept apart: the router died, or it ran and
        # wrote nothing we can read. Both used to be 'no summary'.
        # `summary_error` (#830) is the third: the file existed and did not
        # parse. It shares the 'no_summary' status -- there IS no readable
        # summary, and the vocabulary stays closed -- but the note says which,
        # because rc is 0 either way. `.get()`: probe tests replace
        # scoped_route with lambdas returning hand-built dicts.
        status = 'crashed' if res['returncode'] != 0 else 'no_summary'
        _err = res.get('summary_error')
        _what = (f"unreadable JSON summary ({_err})" if _err
                 else "no JSON summary")
        return {'failures': None, 'status': status,
                'note': f"{note_prefix}rc={res['returncode']}, {_what}",
                'iterations': None, 'vias': None, 'nets': len(nets),
                'returncode': res['returncode']}
    n, note = route_verdict(summary)
    return {'failures': n, 'status': 'ok', 'note': note_prefix + note,
            'iterations': summary.get('total_iterations'),
            'vias': summary.get('total_vias'), 'nets': len(nets),
            'returncode': res['returncode']}


def screened_row(nets, note):
    """The row for a probe DELIBERATELY not run (the ratsnest screen). Same
    shape, distinct status -- it was previously another `failures: None`."""
    return {'failures': None, 'status': 'screened', 'note': note,
            'iterations': None, 'vias': None, 'nets': len(nets),
            'returncode': None}


def route_verdict(summary):
    """(failures, note) from a route summary -- the tier-3 comparison key."""
    if not summary:
        return None, 'no summary'
    # A summary that is not a MAPPING at all. `json.load` is happy to return a
    # str, a list or a number, and the whole premise here is documents route.py
    # did not write -- a stale file at a reused path, another tool's output.
    # Without this the guard below is worse than useless on a str: `k in
    # summary` becomes a SUBSTRING test, so a document merely mentioning
    # "failed_single" passes it and then dies on `.get` two lines down. On an
    # int it raises TypeError outright. Either way the crash lands back in the
    # caller's loop -- the exact failure scoped_route's guard just closed.
    if not isinstance(summary, dict):
        return None, 'unreadable summary'
    # Every read below is a `.get(..., default)`, so a dict that is truthy but
    # carries NONE of the verdict's own keys scores 0 failures and 'clean' --
    # the best possible result, for a document that never mentioned routing.
    # `{}` was caught above; `{'x': 1}` was not.
    #
    # PRESENCE, not truthiness: a genuinely clean route sets `failed_single:
    # []`, and ANY ONE key is enough to judge -- a summary carrying
    # failed_single but not open_single must still degrade to the old
    # arithmetic (tests/test_open_single_verdict.py). `protected_skipped` is
    # deliberately absent from the tuple: it only decorates the note, so a
    # document carrying it alone would still fabricate a 0.
    #
    # HARDENING, not a live bug: route.py's summary literal sets failed_single,
    # open_single AND multipoint_pads_total unconditionally (route.py:3606,
    # 3611, 3624), and there is one append site into _SUMMARY_SINK, so nothing
    # it writes today lands here. This is the schema-drift tripwire.
    #
    # The tuple is INLINE rather than a module constant on purpose: the gap
    # between scoped_route and route_verdict is where #713's probe helpers land,
    # and a constant parked there would collide with them for no benefit.
    if not any(k in summary for k in ('failed_single', 'open_single',
                                      'failed_multipoint',
                                      'multipoint_pads_total',
                                      'multipoint_pads_connected')):
        return None, 'unreadable summary'
    failed = list(summary.get('failed_single') or [])
    # Routed-but-OPEN nets (kept result, disconnected pads). Before this key a
    # non-multipoint open net weighed ZERO here -- probes read failures=0 on
    # boards shipping open copper. Multipoint nets are excluded from the key by
    # the emitter, so adding it to the pad deficit cannot double-count.
    opened = list(summary.get('open_single') or [])
    fm = [d.get('net_name') if isinstance(d, dict) else d
          for d in (summary.get('failed_multipoint') or [])]
    deficit = (summary.get('multipoint_pads_total', 0)
               - summary.get('multipoint_pads_connected', 0))
    n = len(failed) + len(opened) + max(0, deficit)
    parts = []
    if failed or fm:
        parts.append('failed: ' + ', '.join(sorted(set(failed + fm))[:6]))
    if deficit:
        parts.append(f'{deficit} pad(s) short')
    prot = summary.get('protected_skipped')
    if prot:
        # Surfaced because a caller following the router's own retry hint would
        # otherwise loop: 'locked' has no override, ever.
        flat = {k: v for ctx in prot.values() for k, v in ctx.items()}
        parts.append('refused rips: ' + ', '.join(
            f'{k}({v})' for k, v in sorted(flat.items())[:4]))
    return n, '; '.join(parts) or 'clean'


# ------------------------------------------------------------- rip invariants

def check_rip_invariants(nets, rip_set, power_nets=(), impedance_nets=()):
    """Complaints about a proposed rip. Empty list means it is safe to run.

    Four rules, each of which cost a wasted iteration to learn:

    1. A ripped net is re-routed at the CALLING command's parameters, not the
       ones it was originally routed with. Ripping a width-bearing net without
       carrying its width brings it back at the signal default and silently
       destroys a spec geometry.
    2. One net per call. Two together let the second rip the first, reported as
       "1/2 routed" twice running with a DIFFERENT net each time.
    3. A glob never substitutes for an exact name on a protected net: the glob
       is silently skipped while the router keeps asking for that exact rip.
    4. A rip set that names the net being routed is a no-op that needs
       --force-reroute instead.
    """
    out = []
    if len(nets) > 1:
        out.append(f"routing {len(nets)} nets in one call: the second can rip "
                   f"the first and the tally will not say so -- one net per call")
    widthy = set(power_nets) | set(impedance_nets)
    unguarded = sorted(widthy & set(rip_set))
    if unguarded:
        out.append(f"rip set contains width-bearing net(s) {', '.join(unguarded)} "
                   f"-- carry --power-nets/--impedance in the SAME call or they "
                   f"come back at the signal default")
    globs = sorted(p for p in rip_set if any(c in p for c in '*?['))
    if globs:
        out.append(f"rip pattern(s) {', '.join(globs)} are globs: a protected or "
                   f"locked net matching them is skipped silently -- name it "
                   f"exactly to override, and note 'locked' has no override")
    both = sorted(set(nets) & set(rip_set))
    if both:
        out.append(f"{', '.join(both)} is in BOTH --nets and the rip set: that is "
                   f"a no-op unless you also pass --force-reroute")
    return out


# ------------------------------------------- a lens verdict against the score

#: lens name -> the score components a PASS on it CONTRADICTS.
#:
#: `connectivity` asks "is every net actually joined", which is exactly
#: `unrouted` + `broken`; `drc` asks "does the copper break a rule", which is
#: `drc` + `undersized`; `spec` asks "does the board meet what was ASKED for"
#: (verifier-prompts.md lens 9: impedance, connector positions, length rules,
#: track and pair widths), which is `impedance` + `floorplan` + `length` +
#: `net_widths`.
#:
#: `spec` USED TO BE ABSENT, on the argument that its components are routinely
#: ungraded and an ungraded component contradicts nothing. The premise is true
#: and the conclusion does not follow: `score_component` already returns None
#: for an ungraded component, so a mapping over them can only ever fire on a
#: count something MEASURED. Leaving it out did not make the check
#: conservative, it made it absent -- run 25 wrote two rows carrying
#: `VERDICT=PASS:lens=spec` beside a score reporting `impedance 1`, and the
#: end-to-end verifier returned FAIL on that same clause two hours later.
#: Measured on that row: `[]` without the entry, `[('spec','impedance',1)]`
#: with it.
#:
#: `intent` is lens 1, the PLACEMENT half's floorplan lens ("Does this board
#: honour the declared floorplan? Report every violations[] entry"), which is
#: the same `check_floorplan` count `spec` reaches from the other side. TWO
#: lenses legitimately speak to one component, and a row carrying a PASS on
#: both while `floorplan` is non-zero is wrong twice; the table is
#: lens -> components, not a partition. Without this entry a
#: `VERDICT=PASS:lens=intent` beside a measured `floorplan 3` was exactly
#: #904's defect, one lens over.
#:
#: A consequence worth stating: these mappings bind ORDINARY LAPS too, not only
#: close-outs -- the contradiction check runs on any row carrying a lens and a
#: score. That is the intent (a lap that records a verdict its own numbers deny
#: is no better than a close-out that does), but it is a wider behaviour change
#: than the close-out this issue is about.
#:
#: `assembly` IS DELIBERATELY UNMAPPED, and this is the part to read before
#: "completing" the table. `blocking` sums NINE components; the routed-board
#: lenses are 7-9 and cover eight of them. `assembly` is graded at the
#: PLACEMENT boundaries, by the boundary verifier's check 5
#: (references/verifier-prompts.md, "Check 5 addendum"), which answers
#: `VERDICT=...:check=<1-5>` -- a different grammar that `_LENS_RE` refuses on
#: purpose. Mapping it onto one of these three would make a routed-board lens
#: answerable for a check nobody asked it to run.
#: `tests/test_904_lens_components_cover_blocking.py` re-derives the nine names
#: from board_score.py's own source and fails when a new one has no home here.
LENS_COMPONENTS = {
    'connectivity': ('unrouted', 'broken'),
    'drc': ('drc', 'undersized'),
    'spec': ('impedance', 'floorplan', 'length', 'net_widths'),
    'intent': ('floorplan',),
}

_LENS_RE = r'^VERDICT=(PASS|FAIL):lens=([A-Za-z0-9_-]+)'

#: Lenses whose verdict a `--final --kind completion` row may not carry as a
#: BARE `--lens`. A close-out is the run's terminal record and nothing reopens
#: it, so every verdict in it must have an artifact behind it: a path and a
#: sha256 a later reader can open, rather than a line somebody retyped from a
#: reply. references/verifier-prompts.md has required that durable copy since
#: run 23; this is what makes it load-bearing instead of advisory.
#:
#: All three, deliberately, and the counter-argument is worth keeping because
#: it is a good one: `spec` is the lens the arithmetic usually CANNOT refute --
#: impedance, floorplan, length and net_widths are ungraded on most boards, and
#: an ungraded component contradicts nothing -- while `connectivity` and `drc`
#: grade on essentially every board, so LENS_COMPONENTS is a live backstop for
#: them. That argues for listing `spec` alone. Against it: one rule is easier
#: to obey than two, a backstop is not a source, and a bare line for any of the
#: three is a claim about the run rather than about a file. Narrowing this is
#: one token if a measured row ever justifies it.
#:
#: NON-final rows are untouched. A lap's lenses are working notes.
LENS_MUST_BE_SOURCED = ('connectivity', 'drc', 'spec')

#: Stop conditions a `--final --kind completion` row may carry when a lens
#: FAILED. Two vocabularies, both of record: the routing half's NUMBERS
#: (convergence.md §3 -- 2 budget spent, 4 measured-unfixable) and the outer
#: loop's verdict NAMES as `verdict` prints them and L5 interpolates them.
#: DONE-EXHAUSTED is deliberately absent -- with a FAIL lens it is a
#: contradiction, refused above the membership check.
FAIL_COMPATIBLE_STOPS = ('2', '4', 'STUCK', 'BUDGET')

#: The WHOLE stop-condition vocabulary (#901), checked on every `record` that
#: carries one -- not only when a lens FAILED, which is what let ~500 characters
#: of prose into rows 29/30 of run 25 while the orchestrator's `4 (this half):
#: ...` was refused twice at close-out. One record, two rules, depending on a
#: lens. The numbers are convergence.md §3 (1 done, 2 budget spent, 3 plateau,
#: 4 measured-unfixable); the names are what `verdict` prints and L5
#: interpolates. FAIL_COMPATIBLE_STOPS is the subset legal beside a FAIL lens.
STOP_TOKENS = ('1', '2', '3', '4', 'DONE-EXHAUSTED', 'STUCK', 'BUDGET')

#: MSYS2's argv-rewrite signature. Git Bash rewrites any argument starting with
#: `/` into a Windows path unless MSYS2_ARG_CONV_EXCL is set, and EVERY KiCad
#: net name is `/`-prefixed -- so `/D_P` reaches the tool as
#: `C:/Program Files/Git/D_P`. Nothing warns, because a tool cannot tell a
#: mangled net name from a net that does not exist (CLAUDE.md). Row 31 of run 25
#: holds exactly this in its `lever_argv`; `replay` of it would grade impedance
#: on two nets that do not exist and return null, i.e. a vacuous pass.
#: `.search`, never `.match`, on BOTH argv tokens and lever prose. The rewrite
#: is at position 0 of a bare token but NOT of `--impedance-nets=/D_P`, which
#: MSYS rewrites in place; and prose cannot be split on whitespace to find it
#: because the rewritten path itself contains a space ("Program Files").
#:
#: WHAT THIS DOES NOT CATCH, so its silence is not read as a clean bill: the
#: MSYS root is the install directory, and this knows the three common ones.
#: A portable Git, an unusual install path, or a rewrite whose root is none of
#: these is NOT detected -- the guard is a detector for the shape that has
#: actually bitten this repo, not a proof of absence. The reliable defence
#: remains `export MSYS2_ARG_CONV_EXCL='*'` before any command carrying net
#: names (CLAUDE.md).
_MANGLED_RE = re.compile(
    r'[A-Za-z]:[/\\](?:Program Files(?: \(x86\))?[/\\]Git|msys64|msys32)[/\\]')

_MSYS_REMEDY = ("export MSYS2_ARG_CONV_EXCL='*' before the command, and pass "
                "Windows-style paths (C:/Users/...) in the same command since "
                "the variable also stops ~/ and /c/ paths being converted")


def split_stop_condition(value):
    """``"4 (this half): the pair is parity-fixed"`` -> ``('4', 'the pair ...')``.

    The TOKEN is the first whitespace-delimited chunk with a trailing ``:``
    stripped, so both shapes run 25 actually recorded are legal as printed --
    a bare ``DONE-EXHAUSTED`` and a token carrying an aside and a reason. The
    remainder is the REASON and goes in its own field rather than being
    validated as if it were a token. Returns ``(None, raw)`` when the token is
    not one of :data:`STOP_TOKENS`; the caller refuses.
    """
    raw = (value or '').strip()
    if not raw:
        return None, ''
    # `split(None, 1)` and not `partition(' ')`: a tab between the token and its
    # reason is not a parse error.
    parts = raw.split(None, 1)
    token = parts[0].rstrip(':')
    if token not in STOP_TOKENS:
        return None, raw
    # EVERYTHING after the token is the reason, verbatim. The first cut
    # partitioned it again on its own first ':' and kept only the tail, which
    # deleted exactly the informative half: `4 (this half): ...` lost the aside
    # that distinguishes the routing half's close from the orchestrator's,
    # `3: plateau: five laps` lost "plateau", and stop condition 4 -- defined as
    # "a finding about the requirement, with the measurement that proves it" --
    # lost the finding and kept the measurement. The ledger is the terminal
    # record; it does not get to silently drop half a sentence.
    return token, (parts[1].strip() if len(parts) > 1 else '')


def score_component(score, key):
    """The graded count for `key`, or None when NOTHING measured it.

    `blocking_by` first (board_score's own breakdown), then
    `components[key]['count']` and only when that component actually `ran`.
    None means UNGRADED, and that is the conservative half of every check
    built on this: a component nobody graded can never contradict a verdict.
    """
    if not isinstance(score, dict):
        return None
    by = score.get('blocking_by')
    if isinstance(by, dict) and key in by:
        v = by.get(key)
        return None if isinstance(v, bool) or not isinstance(v, (int, float)) \
            else v
    comp = (score.get('components') or {}).get(key)
    if isinstance(comp, dict) and comp.get('ran') is True:
        v = comp.get('count')
        if not isinstance(v, bool) and isinstance(v, (int, float)):
            return v
    return None


def _grades_another_board(board, score):
    """Does this score payload demonstrably grade a DIFFERENT board?

    True only when `board_sha` is present AND differs. Absent sha, unreadable
    board, missing board_store: False -- "I could not tell" must not switch a
    check off. Baseline rows legitimately attach a parent score to a rejected
    candidate (record already WARNS about that), and a check that compared a
    verdict about board A against board B's numbers would be the same class of
    mistake it exists to catch.
    """
    psha = (score or {}).get('board_sha') if isinstance(score, dict) else None
    if not psha or not board or not os.path.isfile(board):
        return False
    try:
        from board_store import sha256_file
        return sha256_file(board) != psha
    except Exception:                                           # noqa: BLE001
        return False


def read_lens_file(path):
    """(line, lineno) -- the FIRST line of `path` that begins `VERDICT=`.

    references/verifier-prompts.md has required every verifier to write its
    verdict to disk since run 23 ("a reply is a notification and notifications
    get lost"). Nothing read those files: the line was retyped into `--lens`
    from a reply, so the ledger recorded a CLAIM ABOUT THE RUN where it could
    have recorded a claim about a file. This is the reader.

    SELECTION IS DELIBERATELY DUMB, and validation is left where it already
    lives. Selecting "the first line matching `_LENS_RE`" instead would step
    silently past a MALFORMED verdict to a well-formed one further down -- the
    exact normalisation verifier-prompts.md forbids, and the one this file
    already refuses to do with `--lens` ("stored RAW, so a malformed line stays
    visible instead of being normalised into something that reads like a
    pass"). So the first `VERDICT=`-prefixed line wins whatever it says, and
    the grammar gate that already exists names it.

    No size cap and no line cap: a cap is a place to bury a FAIL.

    `utf-8-sig`, unlike `--score-file`'s plain utf-8. This is a line-PREFIX
    test, so a BOM on line 1 would make the file report "no VERDICT= line"
    about a file that visibly contains one -- a confusing message on a gate
    nobody should want to work around. A JSON parser has its own BOM handling;
    a startswith() does not.

    Raises OSError (unreadable) or ValueError (no such line); the caller turns
    both into a refusal that writes nothing.
    """
    n = 0
    with open(path, encoding='utf-8-sig') as fh:
        for n, line in enumerate(fh, 1):
            if line.strip().startswith('VERDICT='):
                return line.strip(), n
    raise ValueError(
        f"no line beginning 'VERDICT=' in {path} ({n} line(s) scanned). The "
        f"token is case-sensitive, and the verifier's own reply format is "
        f"`VERDICT=(PASS|FAIL):lens=<lens>` on a line of its own "
        f"(references/verifier-prompts.md). A boundary-verification verdict "
        f"spells `check=<1-5>` instead of `lens=<name>` and is not a lens: "
        f"cite it in the report, not here.")


def lens_name(raw):
    """The lens a VERDICT= line speaks about, lower-cased, or None.

    ONE definition, because there were three: this file's grammar check, its
    --final lens-set loop (which re-inlined the pattern as a literal beside an
    `__import__('re')`), and loop_driver._cross_check's own copy. Three regexes
    for one grammar is three places for a `lens=Connectivity` to be handled
    differently.

    CASE-FOLDED. `_LENS_RE` accepts [A-Za-z0-9_-]+ and every table keyed by a
    lens name here is lower-case, so `lens=Connectivity` used to pass the
    format check, miss the table, and be written -- the whole gate bypassed by
    a shift key. The verdict word (PASS/FAIL) is NOT returned: callers that
    need it match `_LENS_RE` themselves, and folding two questions into one
    return value is how the FAIL branch gets forgotten.
    """
    m = re.match(_LENS_RE, str(raw or '').strip())
    return m.group(2).lower() if m else None


def lens_contradictions(lenses, score):
    """[(lens, component, count)] where a PASS verdict contradicts the score.

    `record --lens` took the verdict string VERBATIM and validated only its
    FORMAT, so nothing ever compared it against the numbers sitting in the same
    row. Measured (run 17, ledger iteration 21): a `--final` row carrying
    `VERDICT=PASS:lens=connectivity` on a score reporting unrouted 32 and
    broken 47, while route.log -- 2.65 MB and 19 JSON_SUMMARY blocks -- held
    ZERO `VERDICT=` lines, because no verifier had ever run. That row passed L3,
    L4 and L5 untested and was corrected only because a human challenged it.

    The row already carries the score, so the contradiction is arithmetic.
    Conservative by construction: only a component with a DEFINITE positive
    count contradicts, a null never does, and a FAIL verdict is honest about
    any number at all.
    """
    out = []
    for raw in (lenses or []):
        m = re.match(_LENS_RE, str(raw or '').strip())
        if not m or m.group(1) != 'PASS':
            continue
        for key in LENS_COMPONENTS.get(lens_name(raw), ()):
            n = score_component(score, key)
            if isinstance(n, (int, float)) and n > 0:
                out.append((m.group(2), key, n))
    return out


# ------------------------------------------- two scores that measured the same

#: The flag that makes each component gradeable, when the score itself does not
#: say. board_score's own `components[k].reason` names it verbatim ("no
#: --impedance-nets given; impedance is ungraded"), so that is preferred and
#: this is only the fallback for a payload that carries `ungraded` alone.
_UNGRADED_FLAG = {'floorplan': '--intent', 'impedance': '--impedance-nets',
                  'length': '--length-groups', 'net_widths': '--net-min-widths'}


def ungraded_set(score):
    """Which components this score did NOT measure, or None if unknowable.

    Two sources, unioned, because either can be absent: board_score's own
    `ungraded` list, and every key `blocking_by` reports as null. A null in
    `blocking_by` IS the component saying it did not answer.
    """
    if not isinstance(score, dict):
        return None
    out, seen = set(), False
    ung = score.get('ungraded')
    if isinstance(ung, list):
        out |= {str(x) for x in ung}
        seen = True
    by = score.get('blocking_by')
    if isinstance(by, dict):
        out |= {str(k) for k, v in by.items() if v is None}
        seen = True
    return out if seen else None


def commensurability(old, new):
    """(differ, hint, false_improvement) -- or None when the two are comparable.

    Two `blocking` totals are only comparable if they are totals over the SAME
    components. Measured (run 17, cycles 2 and 3 of the same board): 78 against
    79, and a decision taken on the difference. Graded commensurably -- with
    --impedance-nets, which neither run passed and which the board warrants --
    both score 92 (32 unrouted / 46 broken, tied net for net), and the board
    called WORSE was one impedance crossing BETTER. The whole gap was which
    components each run happened to measure.

    `false_improvement` is the dangerous shape: `blocking` went DOWN while the
    graded set got strictly SMALLER. Routing accepts a lap on strict improvement
    of `blocking`, so a lap that quietly measures one fewer component looks like
    progress and enters the ledger as a false monotone trend.
    """
    a_ung, b_ung = ungraded_set(old), ungraded_set(new)
    if a_ung is None or b_ung is None:
        return None                     # nothing to compare against
    differ = sorted(a_ung ^ b_ung)
    if not differ:
        return None
    bits = []
    for k in differ:
        why = ''
        for side in (old, new):
            comp = ((side or {}).get('components') or {}).get(k) \
                if isinstance(side, dict) else None
            if isinstance(comp, dict) and comp.get('ran') is False \
                    and comp.get('reason'):
                why = str(comp['reason'])
                break
        bits.append(f'{k} ({why or _UNGRADED_FLAG.get(k, "no flag recorded")})')
    ob, nb = (old or {}).get('blocking'), (new or {}).get('blocking')
    false_improvement = (
        isinstance(ob, (int, float)) and isinstance(nb, (int, float))
        and not isinstance(ob, bool) and not isinstance(nb, bool)
        and nb < ob and b_ung > a_ung)   # graded strictly FEWER components
    return differ, '; '.join(bits), false_improvement


# ------------------------------------------------------------------ the verbs

class _StdoutToStderr:
    """`poses` emits JSON on stdout, so nothing else may.

    Parsing the board and building the placement state print diagnostics --
    quench warns about footprints with no courtyard, for instance -- straight to
    stdout, which lands in the middle of the document a caller is piping into
    `json.load`. The diagnostics are worth keeping; they just belong on stderr.
    """

    def __enter__(self):
        self._real = sys.stdout
        sys.stdout = sys.stderr
        return self

    def __exit__(self, *exc):
        sys.stdout = self._real
        return False


def _pose_knobs(board, clearance, board_edge_clearance):
    """Resolve unset pose knobs from the BOARD (run-7 S4).

    The old fixed argparse defaults (0.25/0.55) silently vetoed legal
    rotations on any board routed to a tighter floor (0.15/0.3): the grader
    was stricter than the board's own spec, and `poses` reported "no legal
    pose" for poses the router would happily route. Resolution order (shared
    with check_floorplan via list_nets.board_floor_knobs): explicit CLI
    value > the board's own Default netclass / board constraint > the fixed
    default.
    """
    from list_nets import board_floor_knobs
    return board_floor_knobs(board, clearance, board_edge_clearance)


def cmd_poses(a):
    from kicad_parser import parse_kicad_pcb
    import pose_score
    clearance, board_edge_clearance, knobs = _pose_knobs(
        a.board, a.clearance, a.board_edge_clearance)
    with _StdoutToStderr():
        pcb = parse_kicad_pcb(a.board)
        st = pose_score.make_state(pcb, a.board, clearance=clearance,
                                   board_edge_clearance=board_edge_clearance)
    diag = {}
    with _StdoutToStderr():
        poses = pose_score.rank_poses(pcb, a.board, a.ref, radius=a.radius,
                                      step=a.step, limit=a.limit, state=st,
                                      diagnostics=diag)
    if not poses:
        # The dropped-pose census is the difference between "this part has
        # nowhere to go" and "your knobs veto even staying put" (run-7 S4:
        # flip-in-place WAS enumerated, then silently dropped).
        _cut = bool(diag.get('stopped_early'))
        print(json.dumps({'ref': a.ref, 'poses': [], 'knobs': knobs,
                          'dropped_total': diag.get('dropped_total', 0),
                          'dropped_in_place': diag.get('dropped_in_place', []),
                          'stopped_early': _cut,
                          # "no legal pose" is a VERDICT about the part. A cut
                          # sweep has not earned it -- it diagnoses a part whose
                          # poses were never enumerated.
                          'note': ('the sweep stopped early -- this is NOT a '
                                   'verdict about the part' if _cut else
                                   'no legal pose, including staying put')},
                         indent=1))
        return 2 if _cut else 1

    if a.route:
        if not a.affected:
            print("--route needs --affected NET ... : only the caller knows "
                  "which nets a move can affect", file=sys.stderr)
            return 2
        from placement.writer import write_placed_output
        tmp = tempfile.mkdtemp(prefix='converge_poses_')
        with _StdoutToStderr():     # the writer and the router both narrate
            for p in poses[:a.route_top]:
                cand = os.path.join(
                    tmp, f"p_{p['x']}_{p['y']}_{int(p['rot'])}.kicad_pcb")
                write_placed_output(a.board, cand, [{'reference': a.ref,
                                                     'new_x': p['x'],
                                                     'new_y': p['y'],
                                                     'new_rotation': p['rot']}])
                res = scoped_route(cand, a.affected, extra_args=a.route_args or [])
                n, note = route_verdict(res['summary'])
                # `nets`/`returncode`/`summary_error` so a row that carries no
                # verdict still says what happened. `failures: None` alone
                # cannot distinguish "the router died", "it wrote nothing" and
                # "it wrote something unreadable". A `status` vocabulary
                # belongs with #713's probe helpers, which own it; this row
                # deliberately does not invent a second one.
                # .get() for BOTH of the new reads, not a subscript: #713's
                # probe tests replace scoped_route with a lambda returning a
                # hand-built dict, and a row that is defensive about one key
                # while subscripting the next is not defensive at all.
                p['route'] = {'failures': n, 'note': note,
                              'iterations': res['summary'].get('total_iterations'),
                              'vias': res['summary'].get('total_vias'),
                              'nets': len(a.affected),
                              'returncode': res.get('returncode'),
                              'summary_error': res.get('summary_error')}
    # A cut sweep returns a DIFFERENT best pose with a byte-identical document
    # shape -- measured, r=3/s=0.25: a full sweep chose rot 0 where a truncated
    # one chose rot 90, with no key marking it partial. A ranking nobody can
    # tell is partial is worse than a slow one, hence `stopped_early`.
    print(json.dumps({'ref': a.ref, 'stopped_early': bool(diag.get('stopped_early')),
                      'base_cost': poses[0]['cost'] - poses[0]['delta'],
                      'knobs': knobs,
                      'dropped_total': diag.get('dropped_total', 0),
                      'dropped_in_place': diag.get('dropped_in_place', []),
                      'poses': poses}, indent=1))
    return 0


def cmd_where(a):
    """net_forensics already answers 'where is the gap and what is walling it
    in', per layer, nearest-first. Nothing in the usual chain calls it.

    --oracle prepends KiCad's OWN join list (kicad-cli DRC after a real zone
    refill, parsed to endpoint pairs): each remaining join printed as an
    exact net + pad<->copper pair spec, THEN the per-net forensics. Run-5's
    endgame worked joins one at a time from --items prose; this makes the
    work list machine-shaped and scopes forensics to the nets that still
    need work."""
    nets = list(a.nets or [])
    if getattr(a, 'oracle', False):
        from kicad_unconnected import kicad_unconnected, parse_pairs
        n, items, err = kicad_unconnected(a.board)
        if n is None:
            print(f"ORACLE: ERR {err}")
            return 3
        pairs = parse_pairs(items)
        print(f"ORACLE: {len(pairs)} join(s) remain (kicad-cli DRC, zones refilled)")
        for p in pairs:
            pa, pb = p['a'], p['b']
            print(f"  {p['net']}: {pa['kind']}@({pa['x']:.3f},{pa['y']:.3f},"
                  f"{pa['layer'] or 'all'}) <-> {pb['kind']}@({pb['x']:.3f},"
                  f"{pb['y']:.3f},{pb['layer'] or 'all'})")
        oracle_nets = sorted({p['net'] for p in pairs})
        if not nets:
            nets = oracle_nets
        if not nets:
            return 0
    if not nets:
        print("where: no nets given (pass --nets, or --oracle to derive them)")
        return 2
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_tools', 'net_forensics.py'), a.board,
            '--nets'] + nets + ['--radius', str(a.radius)]
    return subprocess.run(argv, cwd=ROOT).returncode


def _load_defects(paths):
    """The defect documents behind `--defect-json`, inlined into the row.

    A path into a work dir is not a measurement: work dirs are deleted and
    the ledger outlives them. An unreadable path is kept as
    `{'path': ..., 'error': ...}` rather than dropped -- "the lap named a
    defect record we cannot read" is a different fact from "the lap named
    none", and silently collapsing the two is how a ledger stops being
    evidence.
    """
    if not paths:
        return None
    out = []
    for p in paths:
        try:
            with open(p, encoding='utf-8') as f:
                out.append(json.load(f))
        except (OSError, ValueError) as exc:                    # noqa: BLE001
            out.append({'path': p, 'error': f'{type(exc).__name__}: {exc}'})
    return out


def cmd_record(a):
    from board_store import BoardStore, Ledger
    # --score-file: the payload as a PATH, not as argv.
    #
    # `--score` is JSON text, and every caller passes `--score "$(cat ...)"`.
    # board_score payloads run 24-49 kB (they repeat every net name in both
    # `unrouted.nets` and `connectivity_nets`, and carry the whole assembly
    # `pairs` array), so past roughly 32 kB of total argv the SHELL fails with
    # `Argument list too long` and exits 126 -- before `record` ever execs.
    # Nothing is written, and because the 126 belongs to the shell rather than
    # to this tool, a caller who does not re-count the ledger rows sees no
    # error at all. Run 9 lost a lap that way and found it only by chance.
    #
    # `verdict --score` was already a path (it is open()ed), so this makes the
    # two subcommands agree rather than inventing a convention.
    if getattr(a, 'score_file', None):
        if a.score:
            print("record: pass --score OR --score-file, not both.",
                  file=sys.stderr)
            return 2
        try:
            with open(a.score_file, encoding='utf-8') as _sf:
                a.score = _sf.read()
        except OSError as _e:
            print(f"record: --score-file unreadable: {_e}", file=sys.stderr)
            return 2
    # --lens-file: the verdict as a PATH, beside --score-file and for the same
    # reason -- the payload is materialised into the attribute the rest of this
    # function already knows how to handle, so the grammar gate, the
    # contradiction check, the --final lens-set gate and entry['lenses'] are
    # all unchanged. What a path buys that a retyped line does not is
    # PROVENANCE: the row stores where the verdict came from and the sha256 of
    # the bytes it came from, so a reader can open the same file.
    #
    # Order: every bare --lens first, in the order given, then every
    # --lens-file, in the order given. argparse keeps two independent append
    # lists and loses their relative order, so rather than pretend otherwise
    # the rule is written down here and in both --help strings.
    #
    # NOT mutually exclusive with --lens, unlike --score/--score-file: those
    # name ONE payload, while a row carries one verdict per lens and a run may
    # legitimately have a file for one and a typed line for another.
    _lens_src = [None] * len(a.lens or [])
    for _p in (getattr(a, 'lens_file', None) or []):
        try:
            _line, _no = read_lens_file(_p)
        except UnicodeDecodeError as _e:
            # BEFORE the ValueError arm: UnicodeDecodeError SUBCLASSES
            # ValueError, so without its own branch a file that is not text at
            # all was reported as "no line beginning 'VERDICT='" -- an answer
            # about the content of a file nothing could read, with no path and
            # no remedy in it.
            print(f"record: --lens-file {_p} is not UTF-8 text ({_e}). A "
                  f"verdict file holds one VERDICT= line; if this is a log, "
                  f"pass the file the verifier wrote. Nothing was written.",
                  file=sys.stderr)
            return 2
        except OSError as _e:
            print(f"record: --lens-file unreadable: {_e}", file=sys.stderr)
            return 2
        except ValueError as _e:
            print(f"record: {_e} Nothing was written.", file=sys.stderr)
            return 2
        from board_store import sha256_file
        a.lens = (a.lens or []) + [_line]
        # The WHOLE FILE is hashed, not the selected line. A verdict line is
        # `finding=` and `evidence=` as much as it is PASS or FAIL, and hashing
        # the line alone would let everything around it be rewritten with the
        # row still verifying.
        #
        # TWO paths. `path` is as the caller spelled it, which is this repo's
        # idiom everywhere else -- and on its own it is unresolvable, because
        # the command L5 prints is relative to the work dir and a later reader
        # is somewhere else. `abspath` is what the file WAS at record time. The
        # sha256 remains the identity; the paths are where to look for it.
        _lens_src.append({'path': _p, 'abspath': os.path.abspath(_p),
                          'sha256': sha256_file(_p), 'line': _no})
    # Refuse an --argv that can never replay (run-7 F4: entries recorded with
    # placeholder script names made replay a reconstruction, which is exactly
    # what the ledger exists to prevent). Nothing is written on refusal.
    if a.argv:
        import shutil
        exe = a.argv[0]
        if not (os.path.isfile(exe) or shutil.which(exe)):
            print(f"record: --argv starts with '{exe}', which is neither an "
                  f"existing file nor an executable on PATH -- this entry "
                  f"could never replay. Record the REAL command (the one that "
                  f"produced the board), or omit --argv for a prose-only "
                  f"entry. Nothing was written.", file=sys.stderr)
            return 2
        # ...and EVERY OTHER TOKEN, for the one corruption a replay cannot
        # detect either (#901). The guard above only ever saw argv[0], which is
        # `python3` for every invocation the doctrine teaches, so a mangled net
        # name three tokens later sailed through -- and `replay` re-executes the
        # stored list verbatim, so the row grades nets that do not exist and
        # returns null: a vacuous pass nothing reports.
        _bad = [t for t in a.argv if _MANGLED_RE.search(str(t))]
        if _bad:
            print(f"record: --argv contains {len(_bad)} token(s) rewritten by "
                  f"MSYS2 -- {', '.join(repr(t) for t in _bad[:3])}"
                  f"{' ...' if len(_bad) > 3 else ''}. Git Bash converts any "
                  f"argument starting with '/' into a Windows path, and every "
                  f"KiCad net name is '/'-prefixed, so this row records nets "
                  f"that do not exist and would REPLAY as a vacuous pass. "
                  f"Re-run the command with {_MSYS_REMEDY}, then record it. "
                  f"Nothing was written.", file=sys.stderr)
            return 2
    # The same shape in --lever is a WARNING, not a refusal: the lever is prose
    # for a human, so a mangled name there misleads a reader without making the
    # row unreplayable.
    if a.lever and _MANGLED_RE.search(str(a.lever)):
        print(f"record: WARNING -- --lever contains an MSYS2-rewritten token "
              f"(a '/'-prefixed net name turned into a Windows path). The row "
              f"is still replayable; the prose is wrong. {_MSYS_REMEDY}.",
              file=sys.stderr)
    # Lens verdicts are stored RAW, so the grammar stays owned by
    # verifier-prompts.md and a malformed line stays visible instead of being
    # normalised into something that reads like a pass. Refuse the shape at
    # write time -- same posture as --argv above -- so the ledger never holds a
    # row that cannot be read back.
    if a.lens:
        _badl = [v for v in a.lens if not re.match(_LENS_RE, v.strip())]
        if _badl:
            print("record: --lens takes the verifier's VERDICT= line verbatim, "
                  "e.g. 'VERDICT=PASS:lens=connectivity' or "
                  "'VERDICT=FAIL:lens=drc;finding=...;evidence=...'. "
                  f"Not: {_badl[0]!r}. Nothing was written.", file=sys.stderr)
            return 2
    # THE VERDICT AGAINST THE NUMBERS IN THE SAME ROW. Format was the only
    # thing ever checked, so a PASS could be recorded beside the score that
    # refutes it -- see lens_contradictions for the measured row. Refuse before
    # anything is written, the same posture as --argv and the lens grammar.
    _score_doc = None
    if a.score:
        try:
            _score_doc = json.loads(a.score)
        except (OSError, ValueError) as exc:                    # noqa: BLE001
            # REFUSE, rather than carrying None forward. This used to swallow
            # the failure here and at the board_sha check below, then hit a
            # THIRD, unguarded `json.loads(a.score)` while building the ledger
            # entry -- so a truncated --score-file crashed with a traceback at
            # exit 1, AFTER store.put() had already written the board into the
            # content store. Same defect as the route-summary read this change
            # is about, in the same file: a document that exists and does not
            # parse. `--score-file` reads it off disk, so the "valid prefix of
            # a killed writer" case applies here too.
            print(f"record: --score is not readable JSON "
                  f"({type(exc).__name__}: {exc}). Nothing was written.",
                  file=sys.stderr)
            return 2
    if a.lens and isinstance(_score_doc, dict) and \
            _grades_another_board(a.board, _score_doc):
        # NEVER SILENT. The skip is correct -- a verdict about this board must
        # not be judged with another board's numbers -- but an unannounced
        # skip is a door: attach any other board's score and the whole gate
        # switches off. Say that it did not run, and why.
        print("record NOTE: the lens-vs-score check did NOT run -- the score "
              "payload's board_sha names a different board than --board, so "
              "its numbers are not about the board these verdicts describe. "
              "The lens verdicts below are recorded UNCHECKED.",
              file=sys.stderr)
    elif a.lens and isinstance(_score_doc, dict):
        _contra = lens_contradictions(a.lens, _score_doc)
        if _contra:
            _lines = '\n'.join(
                f'    VERDICT=PASS:lens={lens}   vs   score {comp} = {n:g}'
                for lens, comp, n in _contra)
            print(
                f"record: a lens verdict CONTRADICTS the score in this same "
                f"row.\n{_lines}\n\n"
                f"A PASS is a claim about the board, and the numbers being "
                f"recorded beside it say otherwise. Measured (run 17): a "
                f"--final row carried VERDICT=PASS:lens=connectivity on 32 "
                f"unrouted nets and 47 broken joins, and the route log held no "
                f"VERDICT= line at all -- no verifier had run. That row passed "
                f"L3, L4 and L5 untested.\n\n"
                f"Either fix the board and re-score it, or record what the "
                f"verifier actually found:\n"
                f"  --lens 'VERDICT=FAIL:lens={_contra[0][0]};finding=<what is "
                f"wrong>;evidence=<file#/path/to/the/number>'\n\n"
                f"An UNGRADED component (null) is never a contradiction, so "
                f"this only fires on a count that was measured. Nothing was "
                f"written.", file=sys.stderr)
            return 2
    # THE DECLARATION THE VERDICT TEXT ALREADY PROMISED. `verdict`'s own
    # too-few-laps sentence offers two levers -- "give it something further to
    # optimise (or to say on the record that there is nothing)" -- and only the
    # first had a mechanism. This is the parenthetical: a row that says a half
    # has nothing further, WITH A REASON, in the one place the loop reads.
    # A reason is mandatory because the declaration is the claim; without it
    # this would be --flat by another name.
    if a.exhausted and not (a.exhausted_reason or '').strip():
        print(f"record: --exhausted {a.exhausted} needs "
              f"--exhausted-reason \"<what was tried and why nothing is "
              f"left>\". The declaration IS the evidence -- an unreasoned one "
              f"is just a lower --flat with extra steps. Nothing was written.",
              file=sys.stderr)
        return 2
    if a.final and not a.stop_condition:
        print("record: --final requires --stop-condition (which of the run's "
              "stop conditions ended it). Nothing was written.",
              file=sys.stderr)
        return 2
    # ALWAYS, not only when a lens FAILED (#901). With every lens passing, any
    # string was accepted and stored -- so the same record had two rules
    # depending on a lens, and ~500 characters of prose went into the ledger as
    # a "stop condition" while an orchestrator's `4 (this half): <reason>` was
    # refused. The token is now checked wherever one is given, and the prose
    # after it keeps its own field instead of being validated as a token.
    _stop_token, _stop_reason = split_stop_condition(a.stop_condition)
    if a.stop_condition and _stop_token is None:
        print(f"record: --stop-condition {a.stop_condition!r} does not start "
              f"with a stop condition. It must be one of "
              f"{' | '.join(STOP_TOKENS)} -- the numbers are convergence.md "
              f"S3 (1 done, 2 budget spent, 3 plateau, 4 measured-unfixable) "
              f"and the names are what `verdict` prints. Prose about WHY goes "
              f"after it (\"3: five laps, no new copper\") or in "
              f"--stop-reason; both land in the row's stop_reason. Nothing "
              f"was written.", file=sys.stderr)
        return 2
    if a.stop_reason and _stop_reason and a.stop_reason.strip() != _stop_reason:
        print("record: a reason was given twice, in --stop-condition and in "
              "--stop-reason, and they differ. Give it once. Nothing was "
              "written.", file=sys.stderr)
        return 2
    _stop_reason = (a.stop_reason or '').strip() or _stop_reason
    # #901: these two are about --final, NOT about which half it closes.
    # They sat inside the `kind == 'completion'` gate below, so
    # `--kind systemic --final --stop-condition DONE-EXHAUSTED --lens
    # VERDICT=FAIL:...` was accepted -- a one-word bypass of the entire
    # run-closing contradiction check, in the same shape ("one record, two
    # rules, depending on something orthogonal") this issue is about. The
    # THREE-LENS requirement stays completion-only: that one really is about
    # the routed board.
    _failed = [v for v in (a.lens or []) if v.strip().startswith('VERDICT=FAIL')]
    # TWO STOP VOCABULARIES ARE OF RECORD, and both must be acceptable as
    # printed: the routing half closes on the NUMBERS of convergence.md §3,
    # and the outer loop's L5 interpolates the verdict NAMES this tool's
    # own `verdict` subcommand prints. L5's command was refused verbatim
    # here for exactly that gap -- a FAIL lens is the NORMAL case on the
    # STUCK/BUDGET paths. DONE-EXHAUSTED is the exception: done-and-
    # measured-done IS the all-lenses-pass claim.
    # The extracted TOKEN, so `4 (this half): <reason>` is judged as a 4.
    _sc = _stop_token or ''
    if a.final and _failed and _sc == 'DONE-EXHAUSTED':
        print(f"record: {len(_failed)} lens FAILED under --stop-condition "
              f"DONE-EXHAUSTED. Done-and-measured-done IS the every-lens-"
              f"passes claim, so a FAIL beside it is the contradiction "
              f"L5's cross-check exists to refuse. Record STUCK or BUDGET "
              f"(or fix the board and re-dispatch the lens), never a done "
              f"a lens denies. Nothing was written.", file=sys.stderr)
        return 2
    if a.final and _failed and _sc not in FAIL_COMPATIBLE_STOPS:
        print(f"record: {len(_failed)} lens FAILED, so this run did not "
              f"finish clean -- --stop-condition must be 2 (budget spent), "
              f"4 (measured-unfixable and said so), or the loop verdict "
              f"naming the same thing (STUCK, BUDGET), not "
              f"{a.stop_condition!r}. A FAIL means `blocking` was not "
              f"really zero. Nothing was written.", file=sys.stderr)
        return 2

    # A run-closing COMPLETION record must carry the routed-board lenses.
    # `blocking == 0` and "every lens passes" are two different claims and the
    # second had no mechanism at all -- verifier-prompts.md states the conjunct
    # and nothing computed it, so a close-out could be written with no lens ever
    # dispatched. This one IS completion-only: it is about the routed board.
    if a.final and a.kind == 'completion':
        _seen = {n for n in (lens_name(v) for v in (a.lens or [])) if n}
        _need = {'connectivity', 'drc', 'spec'}
        _miss = sorted(_need - _seen)
        if _miss:
            # The old text said "routing_driver --stage V5 fans them out".
            # There is no routing_driver.py in this repo and there never has
            # been; V1-V5 survive only as prose. This is a REFUSAL, i.e. the
            # one message whose entire job is to say what to do next, so it
            # names a file the reader can open instead of a tool they cannot
            # find.
            # The path is on ONE line on purpose. Split across two f-string
            # fragments it is still correct for a human and invisible to any
            # grep -- including the cited-path guard this repo runs, whose
            # citation pattern needs a '/' inside a single token.
            _ref = ('.claude/skills/plan-pcb-placement-and-routing/references/verifier-prompts.md')
            print(f"record: --final needs the routed-board lenses and is "
                  f"missing {', '.join(_miss)}. Dispatch them -- {_ref}, "
                  f"'The nine lenses' 7-9 -- and pass each VERDICT= line as "
                  f"--lens-file. `blocking == 0` is not `every lens passes`. "
                  f"Nothing was written.", file=sys.stderr)
            return 2
    # ...and each routed-board lens on ANY --final row must have a FILE behind
    # it. Placed after the missing-lens check (a lens you do not have is a more
    # basic complaint than a lens you cannot trace) and before the
    # stop-condition checks (those are about the board; this is about the
    # record).
    #
    # NOT nested in the `kind == 'completion'` block above, and that is the
    # point. The three-lens requirement is completion-only because it is about
    # the routed board; THIS one is about provenance, and a lens verdict is
    # exactly as unsourced on a `--kind systemic --final` row. That shape is
    # not hypothetical: `_cross_check`'s per-lens supersession takes the LATEST
    # final row that speaks to a lens, so a bare `--kind systemic --final
    # --lens 'VERDICT=PASS:lens=spec'` silently overrides a sourced FAIL --
    # which is #901's "one record, two rules depending on something orthogonal"
    # rebuilt inside the new gate. One rule: a --final row's lens has a file.
    #
    # PASS *and* FAIL. Treating only PASS as needing a source would leave the
    # mechanism unexercised on exactly the runs that print this command: a
    # STUCK or BUDGET close-out normally carries a FAIL, and
    # final_record_command would then have to print a conditional slot.
    if a.final:
        _unsourced = [v for v, s in zip(a.lens or [], _lens_src)
                      if s is None and lens_name(v) in LENS_MUST_BE_SOURCED]
        if _unsourced:
            _list = '\n'.join(f'    {v}' for v in _unsourced)
            print(f"record: this --final row carries "
                  f"{len(_unsourced)} lens verdict(s) with no file behind "
                  f"them:\n{_list}\n\n"
                  f"A close-out is this run's terminal record and nothing "
                  f"reopens a ledger, so every verdict in it needs an artifact "
                  f"a later reader can open -- not a line retyped from a "
                  f"reply. references/verifier-prompts.md already requires "
                  f"every verifier to write its VERDICT= line to disk; pass "
                  f"that file and the row stores its path and sha256:\n"
                  f"    --lens-file <the verifier's file for that lens>\n\n"
                  f"Measured (run 25): a close-out inherited "
                  f"VERDICT=PASS:lens=spec from an earlier step, the "
                  f"end-to-end verifier returned FAIL on the same clause two "
                  f"hours later, and the terminal row had to be re-recorded. "
                  f"Nothing was written.", file=sys.stderr)
            return 2

    store = BoardStore(a.store or os.path.join(os.path.dirname(a.ledger), 'boards'))
    sha = store.put(a.board)
    # Run-3 B4: three ledger entries shipped carrying a PRIOR board's score
    # because --score is free JSON with no binding to --board. board_score
    # now embeds board_sha; warn LOUDLY when it is absent or names a
    # different board than the one being recorded. A warning rather than a
    # refusal: baseline rows legitimately attach a parent score to a
    # rejected candidate -- but never silently.
    if a.score:
        try:
            _payload_sha = json.loads(a.score).get('board_sha')
        except Exception:
            _payload_sha = None
        if _payload_sha is None:
            print("record WARNING: score payload carries no board_sha "
                  "(pre-B4 board_score, or hand-built JSON) -- the ledger "
                  "cannot verify it grades THIS board.", file=sys.stderr)
        elif _payload_sha != sha:
            print(f"record WARNING: score payload grades a DIFFERENT board "
                  f"(payload board_sha {_payload_sha[:12]}... != recorded "
                  f"board {sha[:12]}...). Run-3 shipped three stale-payload "
                  f"entries exactly this way; if this attachment is "
                  f"deliberate (baseline row on a rejected candidate), say "
                  f"so in --lever.", file=sys.stderr)
    lg = Ledger(a.ledger)
    # Three run-7 ledger defects, each caught only by a human re-reading the
    # file afterwards. None is a refusal: every one has a legitimate shape, and
    # a ledger that refuses entries is a ledger people stop writing.
    #
    # (1) The accept flag and the prose disagreeing. Run-7 recorded a rung whose
    #     --lever text says REJECTED while the entry carried accepted=true, so a
    #     reader counting accepted iterations counted a rejection.
    _lever_txt = (a.lever or '').lower()
    _says_reject = any(w in _lever_txt for w in ('reject', 'rolled back',
                                                 'rolled-back', 'reverted'))
    if _says_reject and not a.rejected:
        print("record WARNING: --lever reads as a REJECTION but the entry is "
              "recorded accepted (no --rejected). A reader counting accepted "
              "iterations will count this one. Pass --rejected, or say in the "
              "lever why an entry describing a rejection is the accepted "
              "state.", file=sys.stderr)
    elif a.rejected and _lever_txt and not _says_reject:
        print("record WARNING: entry is --rejected but the lever text never "
              "says so -- a later reader has only the flag. Name the rejection "
              "and the gate that refused it in --lever.", file=sys.stderr)
    # (2) The lever naming a CHECKER instead of the transform. A checker does
    #     not change a board, so an entry whose argv is a checker records no
    #     lever at all -- the board moved for a reason the ledger did not keep.
    if a.argv:
        # Skip past the interpreter to the SCRIPT. This guard inspected
        # argv[0], which is `python3` for every invocation the doctrine
        # teaches (`python3 -X utf8 <script> ...`) -- so it had never fired
        # once, on any entry, in any run. The same blindness applies to the
        # replay guard above, which was validating that an interpreter exists
        # rather than that a command can replay.
        _toks = [str(t) for t in a.argv]
        _script = ''
        for _t in _toks:
            _base = os.path.basename(_t).lower()
            if _base.endswith('.py'):
                _script = _base
                break
            if _base.startswith('python') or _t.startswith('-') or \
                    _base in ('utf8', 'timeout', 'env', 'nice'):
                continue
            _script = _base
            break
        _exe = _script or os.path.basename(_toks[0]).lower()
        if _exe.endswith('.py'):
            _exe = _exe[:-3]
        _stem = _exe.split()[0]
        if _stem.startswith('check_') or _stem in ('board_score', 'list_nets',
                                                   'kicad_drc_compare'):
            print(f"record WARNING: --argv names '{_stem}', which measures a "
                  f"board rather than changing one. The ledger's lever is meant "
                  f"to be the TRANSFORM that produced this board; record that "
                  f"command and put the measurement in --score.",
                  file=sys.stderr)
    # (3) Back-fill. An entry timestamped before the one it follows means the
    #     record was written after the fact, so its ordering is a reconstruction.
    _prior = lg.entries()
    if _prior:
        _last_t = _prior[-1].get('t')
        if isinstance(_last_t, (int, float)) and time.time() < _last_t - 1.0:
            print(f"record WARNING: this entry's clock is BEHIND the previous "
                  f"entry's (t {time.time():.0f} < {_last_t:.0f}). The ledger's "
                  f"order is meant to be the order things happened; a "
                  f"back-filled entry cannot support that claim.",
                  file=sys.stderr)
    # (4) A score that measured a DIFFERENT SET OF COMPONENTS than the lap it
    #     will be compared against. `blocking` is a total, and two totals over
    #     different component sets are not larger and smaller versions of each
    #     other -- see commensurability() for the measured 78-vs-79 that was
    #     really 92-vs-92. Warned always; REFUSED only in the one shape that is
    #     definitely a false improvement, because that is the shape routing's
    #     accept rule turns into an accepted lap.
    _half = _HALF.get(a.kind)
    if _half and isinstance(_score_doc, dict):
        _pacc = None
        # _is_lap, not a bare _HALF match: the previous ACCEPTED LAP is
        # what a new lap is commensurable with. A close-out row is graded
        # over a different component set BY CONSTRUCTION -- that is the
        # whole reason a --final row is not a lap -- so comparing the
        # first ordinary lap after one against it produces exactly the
        # 'different set of components' finding this block exists to
        # report, and can reach the REFUSAL below: --accept-incommensurable
        # demanded for a lap that is fine.
        for _r in reversed(_prior):
            if _r.get('accepted') and _is_lap(_r, _half) \
                    and isinstance(_r.get('score'), dict):
                _pacc = _r
                break
        _cm = commensurability((_pacc or {}).get('score'), _score_doc) \
            if _pacc else None
        if _cm:
            _differ, _hint, _false = _cm
            if _false and not a.rejected and not a.accept_incommensurable:
                print(
                    f"record: this lap looks like an IMPROVEMENT only because "
                    f"it graded FEWER components.\n"
                    f"  blocking {(_pacc.get('score') or {}).get('blocking')} "
                    f"-> {_score_doc.get('blocking')}, but the components that "
                    f"differ are: {_hint}\n\n"
                    + (f"Routing accepts a lap on STRICT improvement of "
                       f"`blocking`, so " if _half == 'routing' else
                       f"This half's laps are ranked by `blocking` and the "
                       f"plateau test compares them, so ") +
                    f"a lap that quietly measures one component "
                    f"less enters the ledger as progress and the history "
                    f"acquires a false monotone trend. Measured (run 17): two "
                    f"cycles compared as 78 vs 79 were 92 vs 92 once both were "
                    f"graded with --impedance-nets, and the board called worse "
                    f"was one impedance crossing better.\n\n"
                    f"Re-score this board with the SAME flags as iteration "
                    f"{_pacc.get('iteration')}, or record it "
                    f"--rejected, or say why the comparison stands anyway:\n"
                    f"  --accept-incommensurable \"<the reason>\"\n"
                    f"Nothing was written.", file=sys.stderr)
                return 2
            print(f"record WARNING: INCOMMENSURABLE with iteration "
                  f"{_pacc.get('iteration')}, the last accepted {_half} lap: "
                  f"the two scores did not grade the same components "
                  f"({_hint}). `blocking` is a total, so the difference "
                  f"between these two rows is partly a difference in what was "
                  f"MEASURED, not in the board.", file=sys.stderr)
    prev = lg.last_accepted()
    entry = {'iteration': len(lg.entries()), 'kind': a.kind,
             'parent_sha': (prev or {}).get('result_sha'),
             'result_sha': sha, 'lever': a.lever,
             'lever_argv': list(a.argv) if a.argv else None,
             # _score_doc, not a THIRD json.loads: the payload was parsed and
             # validated once at the top of this function, which is the only
             # place that can still refuse before anything is written.
             'score': _score_doc,
             'renders': list(a.render_json) if a.render_json else None,
             # The MEASUREMENT the next lap is aimed at, not a paragraph
             # about it. Run 20 recorded "throat 0.409mm vs 0.450 needed,
             # blocked by U4.53/R7.2" as English inside `lever`, so the
             # re-entry could be read by a person and by nothing else.
             #
             # INLINED, unlike `renders`: a render is a large file a reader
             # opens, a defect record is a handful of numbers the ledger
             # itself should still hold once the work dir is gone.
             'defects': _load_defects(a.defect_json),
             # L4 requires a --shape and the ledger has never kept it, so
             # "which shape did we re-enter at, and did it work" was not a
             # question the record could answer.
             'shape': a.shape,
             'lenses': list(a.lens) if a.lens else None,
             # WHERE each lens verdict came from, positionally parallel to
             # `lenses` -- {path, sha256, line} for a --lens-file, null for a
             # line typed on the command line.
             #
             # A LIST, not a dict keyed by lens name: two lines may carry the
             # same lens (a corrected re-record, a verifier that emitted two),
             # and a dict drops one of them silently. `lenses` is already a
             # positional raw list, and a parallel list is the only shape that
             # cannot drift out of correspondence with it under append.
             #
             # Emitted whenever there are lenses at all, even when every entry
             # is null: `lens_source: null` ("written by a converge that
             # predates the field") and `[null, null, null]` ("every lens was
             # typed") are different facts, and collapsing them recreates the
             # silence-looks-like-compliance failure this file names elsewhere.
             'lens_source': _lens_src if a.lens else None,
             # Split on whitespace and commas so `--scope-refs "$(cat locks.txt)"`
             # records 45 refs rather than one 45-ref string.
             'scope_refs': ([t for chunk in a.scope_refs
                             for t in re.split(r'[\s,]+', chunk) if t]
                            or None) if a.scope_refs else None,
             'accepted': not a.rejected}
    # A placement lap moved parts. The skill mandates the move be LOOKED AT,
    # and run 9 skipped that for an entire campaign without anything noticing
    # -- including afterwards, because the record kept no trace either way. A
    # warning, not a refusal: a rejected lap or a no-move gate row legitimately
    # has nothing to show. But silence must stop being indistinguishable from
    # compliance.
    if a.kind == 'placement' and not a.render_json and not a.rejected:
        print("record NOTE: this placement lap records no --render-json. The "
              "read mandates are only auditable through the ledger; without "
              "one, a skipped read and an absent trigger look identical later. "
              "Attach the render you read, or say in --lever why there was "
              "no trigger.", file=sys.stderr)
    if a.final:
        entry['final'] = True
    # The TOKEN alone, so a reader (and `verdict`, and the film) can match it
    # against the vocabulary instead of parsing prose (#901). The reason keeps
    # its own key -- absent, not empty, when there is none. Recorded on ANY row
    # that carries one, not only a --final row: the first cut stored both keys
    # inside `if a.final:`, so `--stop-reason` on an ordinary lap was accepted,
    # validated, and silently dropped -- a flag that does nothing.
    if _stop_token:
        entry['stop_condition'] = _stop_token
    if _stop_reason:
        entry['stop_reason'] = _stop_reason
    if a.exhausted:
        entry['exhausted'] = {'half': a.exhausted,
                              'reason': a.exhausted_reason.strip()}
    if a.accept_incommensurable:
        # The disposition belongs in the row, not only in the console the
        # refusal was cleared from. Same shape as --exhausted: what is recorded
        # is that somebody decided, not that the numbers passed.
        entry['accepted_incommensurable'] = str(a.accept_incommensurable).strip()
    e = lg.append(entry)
    if a.exhausted:
        print(f"recorded: {a.exhausted} declared EXHAUSTED. `verdict` will "
              f"stop asking that half to plateau, and the reason is in the "
              f"ledger where the film and every re-entry read it. A later "
              f"recorded lap of that half SUPERSEDES this declaration.",
              file=sys.stderr)
    print(json.dumps(e, indent=1, sort_keys=True))
    # Failing-net NAMES belong in the record (run-7 S10/F5): a score that
    # carries only a count forces every later read to re-derive which nets,
    # and a truncated re-derivation shipped a wrong close-out.
    sc = e.get('score') or {}
    fails = sc.get('failures')
    names = sc.get('failed_nets') or sc.get('failed') or []
    if fails:
        if names:
            print("failing nets: " + ", ".join(str(n) for n in names[:12]),
                  file=sys.stderr)
        else:
            print(f"NOTE: score records failures={fails} but names no nets -- "
                  f"add 'failed_nets' to the score JSON so the ledger stays "
                  f"readable without re-deriving the open set.",
                  file=sys.stderr)
    return 0


def cmd_step_back(a):
    from board_store import BoardStore, Ledger
    lg = Ledger(a.ledger)
    store = BoardStore(a.store or os.path.join(os.path.dirname(a.ledger), 'boards'))
    if a.to:
        sha = a.to
    elif a.iteration is not None:
        m = [e for e in lg.entries() if e.get('iteration') == a.iteration]
        if not m:
            print(f"no iteration {a.iteration} in {a.ledger}", file=sys.stderr)
            return 2
        sha = m[-1]['result_sha']
    else:
        last = lg.last_accepted()
        if not last:
            print("no accepted iteration to step back to", file=sys.stderr)
            return 2
        sha = last['result_sha']
    store.get(sha, a.out)
    print(f"checked out {sha[:12]} -> {a.out}")
    return 0


def cmd_replay(a):
    from board_store import Ledger, replay_command
    lg = Ledger(a.ledger)
    m = [e for e in lg.entries() if e.get('iteration') == a.iteration]
    if not m:
        print(f"no iteration {a.iteration}", file=sys.stderr)
        return 2
    try:
        argv = replay_command(m[-1])
    except ValueError as e:
        # A message, not a traceback: "this iteration is not replayable" is an
        # ordinary answer about the ledger, not a crash.
        print(str(e), file=sys.stderr)
        print("Record lever_argv when you write an entry and this becomes a "
              "one-liner instead of a reconstruction.", file=sys.stderr)
        return 4
    print('replaying: ' + ' '.join(argv))
    return subprocess.run(argv, cwd=ROOT).returncode


CONTINUE, DONE, STUCK, BUDGET = 4, 0, 5, 6

#: Which half of the loop a ledger row belongs to. `systemic` and
#: `classification` are neither -- one changes how the chain measures itself,
#: the other decides where to re-enter -- and neither changes the board, so
#: neither can make a half look like it is still improving.
_HALF = {'placement': 'placement', 'completion': 'routing'}


def _is_lap(row, half):
    """Is this row a LAP of `half` -- a turn of the loop that could improve it?

    THREE recorded shapes are not laps, and every one of them moved a verdict
    it had no business moving:

      * `kind systemic` / `kind classification`, which _HALF already excludes.
      * a `--final` row. It is the RECORD OF a verdict, so a verdict computed
        from it is computed from its own output. Measured: re-running the same
        L5 command after its own close-out reads "routing improved within its
        last 5 laps" -- the final row carries a score graded over different
        components than the routing half's laps -- and answers "not done yet"
        about a run that already shipped STUCK. The verdict of record was
        reproducible only from the ledger state BEFORE the row recording it,
        which is the wrong way round for a record.
      * an `--exhausted` declaration. It changes no board and says so.
        The shape this clause actually saves is a CROSS-HALF declaration --
        `--kind placement --exhausted routing`, which is a row of the
        placement half by kind and says nothing about placement at all, yet
        used to retract a live placement declaration. (A SELF-declaring row,
        `--kind placement --exhausted placement`, never reaches here:
        `_declaration`'s first branch matches on `exhausted.half` and re-arms
        the declaration before the `elif` runs. Measured both ways.)

    ONE predicate, used by _declaration's supersession branch AND by
    _half_state's window AND by cmd_record's commensurability lookback,
    because the three disagreeing is what produced run 25's fourth
    declaration: the L2 freeze row superseded three live declarations while
    contributing an UNJUDGED lap to the window, so the half was neither
    declared nor answerable and L5 printed "still improving" at a half that
    had said three times it was finished.

    A row with no `kind` is in neither half. (Ledger.counts defaults a missing
    kind to `completion`; this does not, and that predates this function.)
    """
    if _HALF.get(row.get('kind')) != half:
        return False
    if row.get('final'):
        return False
    if isinstance(row.get('exhausted'), dict):
        return False
    return True


def _score_key(score):
    """(blocking, quality) as a comparable tuple, or None if not gradeable.

    Lexicographic, never a weighted sum: a weighted sum lets a router buy off a
    disconnected net with a lower via count. `blocking == None` is NOT zero --
    it means a component that was asked for could not answer -- so it sorts
    worse than any real number rather than reading as a perfect board.
    """
    if not isinstance(score, dict):
        return None
    b = score.get('blocking')
    q = score.get('quality') or {}
    # A quality tuple carrying None (board_score.quality returns {'error': ...}
    # when the board will not parse) makes min() raise TypeError the moment two
    # rows tie on `blocking`. Untested until now because the self-tests use a
    # uniform empty quality. Sort unknowns LAST rather than crashing.
    quality = tuple(v if isinstance(v, (int, float)) else float('inf')
                    for v in (q.get('vias'), q.get('copper_mm'),
                              q.get('segments')))
    if b is None:
        # NOT `inf`. A row whose score never measured `blocking` used to rank
        # as the worst possible board, and `inf >= inf` then made the plateau
        # test TRUE -- so an unmeasured lap read as a plateaued one, which is
        # the opposite of what it is. Returning None drops it from the window
        # entirely (the callers already filter None), so a half is judged on
        # laps that actually measured something.
        return None
    return (b, quality)


def _declaration(rows, half):
    """(reason, live) for the last `--exhausted <half>` row, else None.

    `live` is False when a lap of that half was recorded AFTER the declaration:
    the half went back to work, so the claim "there is nothing further" is
    stale. Superseding it needs no flag and no deletion -- running the half
    again is the retraction.
    """
    found, live = None, False
    for r in rows:
        dec = r.get('exhausted')
        if isinstance(dec, dict) and dec.get('half') == half:
            found, live = (str(dec.get('reason') or '').strip()
                           or 'no reason recorded'), True
        elif found is not None and _is_lap(r, half):
            # _is_lap, not a bare _HALF match, and this is the sharper half of
            # the fix: "the half went back to work" must mean a LAP was run.
            # A freeze row and a close-out row are both `kind placement` /
            # `kind completion` and neither turns the loop, so either one
            # silently retracted a declaration a person had written down.
            live = False
    return None if found is None else (found, live)


def placement_terms(score):
    """The `placement.terms` block of a score, or None. #894."""
    if not isinstance(score, dict):
        return None
    p = score.get('placement')
    if not isinstance(p, dict):
        return None
    t = p.get('terms')
    return t if isinstance(t, dict) else None


def _placement_movement(runs_pairs):
    """`(verdict, hint)` over a window's commensurable runs, or None.

    Compares each run's LAST lap against its FIRST -- the same baseline
    `min(r) < r[0]` uses one tier up, and for the same measured reason: "has
    this half improved across its own last `flat` laps", never "has it beaten
    the best lap ever seen", which one large early improvement pins forever.

    Returns the strongest movement found: `better` if any run improved by
    Pareto, else `mixed` if any run traded, else None. Delegates the
    comparison to `placement_score.compare_terms` -- there is no ordering rule
    here, and no weight anywhere.
    """
    try:
        import placement_score as ps
    except Exception as exc:                                 # noqa: BLE001
        # NOT silently None. `None` here means "the terms did not move", and
        # a half whose comparator could not even load would then report
        # `plateau` -- the exact defect this tier exists to fix, arriving
        # through the tier itself. `placement_score`'s own Vacuity rule is
        # that an unmeasurable thing reports a REASON.
        return 'unmeasured', (f'placement_score could not be imported, so the '
                              f'terms were not compared: '
                              f'{type(exc).__name__}: {exc}')
    best = None
    for run in runs_pairs:
        first = placement_terms(run[0][1])
        last = placement_terms(run[-1][1])
        if not first or not last:
            continue
        verdict, detail = ps.compare_terms(first, last)
        if verdict in ('better', 'mixed'):
            hint = ps.format_delta(detail)
            if verdict == 'better':
                return 'better', hint
            best = best or ('mixed', hint)
    return best


def parent_score(rows, row):
    """The score of the row this one was recorded against, or None.

    `cmd_record` has written `parent_sha` on every ledger row since the ledger
    existed. `tests/stress/harvest_predictor_rows.py` walks it as a lineage
    graph and `py_tools/make_film.py` names it, but nothing in converge itself
    ever resolved it back to the parent's SCORE. This is that read side.

    Returns None when there is no parent, when no row carries that
    `result_sha`, or when MORE THAN ONE does -- a re-recorded board is not a
    parent, and "I could not tell which" must not become an answer.
    """
    if not isinstance(row, dict):
        return None
    sha = row.get('parent_sha')
    if not sha:
        return None
    hits = [r for r in rows
            if isinstance(r, dict) and r.get('result_sha') == sha]
    if len(hits) != 1:
        return None
    return hits[0].get('score')


def _half_state(rows, half, flat):
    """Can this half still improve? -- with the evidence it was decided from.

    Returns a dict: flat, laps, accepted, rejected, why, and (when they apply)
    declared / declared_superseded / incommensurable / compared. `why` is one of
    declared-exhausted, too-few-laps, no-comparison, plateau, improving.

    THE COUNTER COUNTS REJECTED LAPS TOO, and that is the fix for a gate that
    could not be satisfied. It used to count accepted rows only, while L5's own
    closing paragraph instructs "Record the lap you are about to run, accepted
    or rejected. A rejected lap is data" -- so recording a rejected lap could
    not satisfy the gate it was offered for. Worse, routing accepts a lap only
    on STRICT improvement, so a half that is genuinely exhausted has no honest
    accepted lap left to produce: the gate demanded the one thing a finished
    half cannot make. A recorded rejection is exactly the evidence that a half
    tried and did not improve, which is what a plateau IS.

    An accepted lap whose score never measured `blocking` COUNTS AS A LAP but
    carries no key, so it can be compared with nothing. Dropping it entirely
    was worse than it looks: on the real run-17 ledger the placement half then
    read `plateau` from a window of two cycle-1 accepted laps plus three
    cycle-2 REJECTIONS, while five accepted cycle-2 laps were invisible for
    having `score: null`. A half that ran five laps must not read as one that
    stopped. A rejection needs no score, because the rejection is itself the
    measurement.

    Measured (neo6502, run 15): the placement half satisfied its OWN close-out
    in 4 accepted laps -- every gate clean, residue named -- against a --flat of
    5. It could never plateau, so L5 returned CONTINUE forever while reporting
    the half as "still improving", which it was not.
    """
    dec = _declaration(rows, half)
    # (iteration, accepted, key, score) in ledger order. The ITERATION is
    # carried so a `no-comparison` verdict can NAME the rows it could not
    # judge: "2 accepted laps recorded no blocking" tells a reader there is a
    # problem and not where it is, and the remedy -- re-score them, or record
    # them as the systemic rows they were -- needs the numbers.
    ev = []
    for r in rows:
        if not _is_lap(r, half):
            continue
        if not r.get('accepted'):
            ev.append((r.get('iteration'), False, None, r.get('score')))
            continue
        ev.append((r.get('iteration'), True, _score_key(r.get('score')),
                   r.get('score')))
    n_acc = sum(1 for e in ev if e[1])
    out = {'laps': len(ev), 'accepted': n_acc, 'rejected': len(ev) - n_acc,
           'flat': False, 'why': 'too-few-laps'}
    if dec and dec[1]:
        out.update(flat=True, why='declared-exhausted', declared=dec[0])
        return out
    if dec and not dec[1]:
        out['declared_superseded'] = dec[0]
    if len(ev) < flat:
        # NOT the same as "still improving" -- say which. The threshold is
        # `flat` and the test is `<`: a window of exactly `flat` laps is
        # answerable, and the guard used to be `<=`, which demanded flat+1
        # while the refusal text said flat. It read as self-contradictory
        # because it was.
        return out
    window = ev[-flat:]
    # The evidence the verdict is read off, set HERE -- before any branch can
    # return. It used to be attached at the bottom of the function, so the
    # all-rejected `plateau` below returned without it: the one verdict shape
    # where a reader most wants to know which laps were counted (it can drive
    # DONE or STUCK) was the one that did not say. The docstring has promised
    # "with the evidence it was decided from" since it was written.
    out['window_iterations'] = [i for i, _a, _k, _s in window]
    if not any(acc for _i, acc, _k, _s in window):
        # Every lap in the window was REJECTED. Nothing improved, by
        # construction -- that is a plateau stated by the half itself.
        out.update(flat=True, why='plateau')
        return out
    # Did this half improve ACROSS ITS OWN last `flat` laps?
    #
    # `best_before = min(keys[:-flat])` asked a different question: has the
    # window beaten the best lap EVER seen before it. One large early
    # improvement then pins the bar for the rest of the run. Measured on run 9:
    # the pour took the routing half to 297 on its first lap, and the series
    # [297, 371, 365, 340, 330, 320] -- a necessary rise at the fanout, then
    # five laps of strict improvement -- reported flat:true, because 320 never
    # beat 297. A half that is demonstrably still moving read as plateaued,
    # which is exactly the DONE-vs-STUCK confusion this function exists to
    # prevent.
    #
    # (Replacing it with `keys[-flat-1]` does NOT fix that case -- on a
    # 6-lap ledger that IS the 297. The baseline has to be the window's own
    # first lap.)
    #
    # ...AND IMPROVEMENT IS ONLY CREDITED WITHIN A COMPARABLE RUN. Two
    # `blocking` totals over different component sets are not larger and
    # smaller versions of each other, so a drop across such a pair is not
    # evidence that the half improved -- it may be evidence that it measured
    # less. The window is split into maximal runs of consecutive laps that
    # actually compare (both scored, same graded set), and the half is
    # `improving` if ANY run shows a lap beating that run's own first lap.
    #
    # It is deliberately NOT "judge the leading run and discard the rest": that
    # is what this did first, and one incommensurable pair at the FRONT then
    # collapsed the window to a single lap, where `min(keys[:1]) >= keys[0]` is
    # a tautology. Measured on the fix itself: the series 78 -> 70 -> 60 -> 50
    # -> 40, four consecutive comparable improvements of 10 each, reported
    # `plateau` and ended the run at STUCK because the FIRST pair did not
    # compare. Guarding against a false improvement must not manufacture a
    # false plateau; both are the same error.
    # Comparability is a property of TWO SCORES -- their graded component sets
    # -- and of nothing else. So a rejection or an unscored lap is simply not
    # in this sequence; it does not stand BETWEEN two scores and separate them.
    # Making it break the run was a second false plateau, measured over all
    # 7776 accept/reject windows of 5: 441 flipped `improving` -> `plateau`
    # (78, 78, 78, REJ, 40 ended the run at STUCK on a half whose last lap took
    # blocking 78 -> 40), and 2313 flipped `plateau` -> unanswerable, on the
    # alternating accept/reject pattern that IS normal routing.
    seq = [(k, s) for _i, acc, k, s in window if acc and k is not None]
    runs, cur, hints = [], [], []
    for k, s in seq:
        cm = commensurability(cur[-1][1], s) if cur else None
        if cm:
            hints.append(cm[1])
            runs.append(cur)
            cur = []
        cur.append((k, s))
    runs.append(cur)
    runs_pairs = [r for r in runs if len(r) >= 2]
    runs = [[k for k, _s in r] for r in runs_pairs]
    # An ACCEPTED lap that recorded no `blocking` is unjudged, and a plateau
    # asserted over unjudged laps is the "reported clean because unexamined"
    # error this toolchain names everywhere else. An improvement, by contrast,
    # is a DEFINITE finding and stands whatever else is in the window. So:
    # improvement wins; a plateau requires every accepted lap to have been
    # judged; anything else is not answerable and says so. A REJECTION is not
    # unjudged -- the rejection is itself the measurement.
    unjudged_its = [i for i, acc, k, _s in window if acc and k is None]
    unjudged = len(unjudged_its)
    _place = _placement_movement(runs_pairs) if half == 'placement' else None
    if any(min(r) < r[0] for r in runs):
        out.update(flat=False, why='improving')
    elif _place and _place[0] == 'better' and not unjudged:
        # THE PLACEMENT TIER (#894). Reachable only when `blocking` and
        # `quality` have ALREADY tied across the run -- which on a copper-free
        # board is every lap, because `quality` is (0, 0.0, 0) for every
        # placement of every board. So this turns `plateau` into `improving`
        # and NOTHING else: it cannot make an improving half plateau, cannot
        # reach the routing half (the `half ==` guard above), and `not
        # unjudged` is what keeps it out of `no-comparison`.
        #
        # That last condition is LOAD-BEARING and was missing in the first
        # version of this branch. Without it the tier also caught the window
        # shape "some laps compare, and at least one ACCEPTED lap carries no
        # `blocking`", which `no-comparison` owns: a review measured a
        # five-lap window flipping from `no-comparison` to `improving` with
        # `unjudged`, `blocked` and `unjudged_iterations` silently dropped --
        # the exact diagnostic those keys exist to carry, and the shape
        # test_904_not_a_lap.py pins. An unjudged lap is not evidence that a
        # half improved, whatever the laps around it did on their terms.
        #
        # PARETO, not a score. `placement_score.compare_terms` says `better`
        # only when no measured term regressed, so a lap that traded pair
        # length for balance is NOT credited -- it reports `plateau` with
        # `placement_traded` naming both sides. #694 is why there is no weight
        # here: a corridor term's measured sign reversed while an aggregate
        # verdict kept printing PASS, because a collapsed mark cannot say
        # which of its inputs moved.
        out.update(flat=False, why='improving', placement_improved=_place[1])
    elif runs and not unjudged:
        out.update(flat=True, why='plateau')
        if _place and _place[0] == 'mixed':
            out['placement_traded'] = _place[1]
        elif _place and _place[0] == 'unmeasured':
            # A plateau asserted while the placement comparator was broken is
            # a plateau over something nobody measured. Say so on the record.
            out['placement_unmeasured'] = _place[1]
    else:
        # Answerable again after one more comparable lap, after `flat`
        # rejections, or by declaring the half exhausted on the record. NAME
        # the cause: these three call for different actions, and a message
        # that guesses tells a still-improving half to declare itself finished.
        out.update(flat=False, why='no-comparison', unjudged=unjudged,
                   blocked=('unjudged' if unjudged else
                            'incommensurable' if hints else 'single-lap'))
        # WHICH rows could not be judged, by the ledger's own word for them.
        # "2 accepted laps recorded no blocking" tells a reader that there is a
        # problem and not where it is, and both remedies -- re-score those laps,
        # or record them as the systemic rows they always were -- need the
        # numbers. `iterations` because that is what `replay --iteration` and
        # `step-back --iteration` take.
        #
        # ONLY when there ARE unjudged rows, and UNFILTERED. Emitting `[]` on
        # the incommensurable and single-lap branches made "no unjudged rows"
        # and "unjudged rows I cannot name" the same value; and dropping the
        # un-numbered ones made `unjudged: 2` sit beside an empty list, which
        # is the count contradicting its own detail. A row with no `iteration`
        # appears as `null` -- "this row is unjudged AND unnumbered" is a
        # sharper finding than silence. (`cmd_record` always numbers a row, so
        # this is about hand-built and foreign ledgers.)
        if unjudged:
            out['unjudged_iterations'] = list(unjudged_its)
    if hints:
        out['incommensurable'] = '; '.join(sorted(set(hints)))
        out['compared'] = sum(len(r) for r in runs)
    return out


def _half_is_flat(rows, half, flat):
    """(is_flat, n_laps, reason) -- the tuple view of _half_state."""
    st = _half_state(rows, half, flat)
    return st['flat'], st['laps'], st['why']


def _halves_note(st):
    """One line naming HOW each half came to be blocked.

    A DONE reached because a half was DECLARED exhausted is a different claim
    from one reached by five measured laps, and the terminal artifact has to
    say which -- the declaration carries a human's reason, not a measurement.
    """
    bits = []
    for h in ('placement', 'routing'):
        s = st[h]
        if s.get('why') == 'declared-exhausted':
            bits.append(f'{h}: DECLARED exhausted -- {s.get("declared")}')
        else:
            bits.append(f'{h}: {s["laps"]} laps, {s["accepted"]} accepted + '
                        f'{s["rejected"]} rejected')
    return '; '.join(bits)


def cmd_verdict(a):
    """Continue, or stop -- and say WHICH kind of stop it was.

    Every stop RULE in this toolchain is written down (a 5-lap placement cap,
    four routing stop conditions, a budget of 100, "5 consecutive flat") and
    not one of them had a MECHANISM: no counter, no budget check, no read of
    the ledger that gated continuation. `final` and `stop_condition` were
    written and never read back. So a run that stopped because it was finished
    and a run that stopped because it was stuck produced the same artifact.

    Reaching `blocking == 0` is the FLOOR, not the finish. The score is
    lexicographic (blocking, quality) and quality orders the boards that
    already got there, so the loop keeps pulling levers on the second key and
    stops only when NEITHER half can improve EITHER key -- which is what
    "fully blocked in both placement and routing" means, measured.
    """
    from board_store import Ledger
    rows = Ledger(a.ledger).entries()
    score, err = None, None
    if a.score:
        try:
            with open(a.score, encoding='utf-8') as fh:
                score = json.load(fh)
        except Exception as exc:                            # noqa: BLE001
            err = f'{type(exc).__name__}: {exc}'
    if score is None:
        print(json.dumps({'verdict': 'NO-SCORE', 'reason': (
            err or '--score is required: the verdict is about a board, and '
            'without its score there is nothing to be blocked or done ABOUT.'
        )}, indent=1, sort_keys=True))
        return 2

    scored = [r for r in rows if _score_key(r.get('score')) is not None]
    key = _score_key(score)
    blocking = key[0]
    st = {h: _half_state(rows, h, a.flat) for h in ('placement', 'routing')}
    flat_p, flat_r = st['placement']['flat'], st['routing']['flat']
    laps_p, laps_r = st['placement']['laps'], st['routing']['laps']
    why_p, why_r = st['placement']['why'], st['routing']['why']

    doc = {'ledger_rows': len(rows), 'scored_rows': len(scored),
           'budget': a.budget, 'flat': a.flat,
           'blocking': None if blocking == float('inf') else blocking,
           'quality': score.get('quality'),
           'ungraded': sorted(score.get('ungraded') or []),
           'unknown': sorted(score.get('unknown') or []),
           # `accepted_laps` is kept and still means accepted laps only; `laps`
           # is what the plateau test counts, and it counts REJECTED laps too
           # -- see _half_state. They differ, so both are published rather than
           # one quietly changing meaning under a reader.
           'placement': dict(st['placement'],
                             accepted_laps=st['placement']['accepted']),
           'routing': dict(st['routing'],
                           accepted_laps=st['routing']['accepted'])}

    if len(rows) >= a.budget:
        doc.update(verdict='BUDGET', reason=(
            f'{len(rows)} ledger entries written, budget {a.budget}. Report '
            f'the best-scoring board AND every remaining blocker, itemised '
            f'with its measurement.'))
        code = BUDGET
    elif not (flat_p and flat_r):
        still = [h for h, f in (('placement', flat_p), ('routing', flat_r))
                 if not f]
        # Say WHICH of the two causes applies, per half. They call for opposite
        # actions and the old sentence offered both at once.
        _why = {'placement': why_p, 'routing': why_r}
        _parts = []
        for h in still:
            if _why[h] == 'too-few-laps':
                _parts.append(
                    f'{h} has run {st[h]["laps"]} recorded lap(s) '
                    f'({st[h]["accepted"]} accepted + {st[h]["rejected"]} '
                    f'rejected), fewer than the {a.flat} this test needs, so '
                    f'whether it plateaued is NOT YET ANSWERABLE -- which is '
                    f'not the same as "it is still improving". A REJECTED lap '
                    f'counts here: it is exactly the evidence that a half '
                    f'tried and did not improve, and routing accepts only on '
                    f'strict improvement, so an exhausted half has no honest '
                    f'accepted lap left to give. Either give it something '
                    f'further to optimise and record the lap (accepted or '
                    f'rejected), or say on the record that there is nothing:\n'
                    f'    python3 -X utf8 py_placer/converge.py record --ledger '
                    f'{a.ledger} \\\n'
                    f'        --board <the board> --kind systemic \\\n'
                    f'        --exhausted {h} --exhausted-reason "<what was '
                    f'tried and why nothing is left>" \\\n'
                    f'        --lever "declaration: {h} has nothing further"\n'
                    f'  Lowering --flat is not one of the options')
            elif _why[h] == 'no-comparison':
                _parts.append(
                    f'{h} has {st[h]["laps"]} recorded lap(s) but no two of the '
                    f'last {a.flat} can be COMPARED -- '
                    + {'unjudged': (
                        f'{st[h].get("unjudged")} accepted lap(s) in that '
                        f'window recorded no `blocking` -- iteration(s) '
                        + (', '.join(str(i) for i in
                                     (st[h].get('unjudged_iterations') or []))
                           or 'not numbered')
                        + ' -- and a lap that '
                        f'measured nothing is evidence in neither direction'),
                       'incommensurable': (
                        f'they were not graded over the same components '
                        f'({st[h].get("incommensurable")})'),
                       'single-lap': (
                        'only one of them carries a score, and one number '
                        'compared with nothing is not a trend')}[
                           st[h].get('blocked', 'single-lap')]
                    + f'. So whether it plateaued is NOT ANSWERABLE, which is '
                      f'not "it did not". Score those laps the same way as the '
                      f'rest and re-record them -- or, if they were never laps '
                      f'(a freeze, a disposition, a close-out), record them '
                      f'--kind systemic, which is what that kind is for: a row '
                      f'that changed no pose must not be able to make a half '
                      f'look unjudged. Or declare the half exhausted on the '
                      f'record:\n'
                      f'    python3 -X utf8 py_placer/converge.py record --ledger '
                      f'{a.ledger} \\\n'
                      f'        --board <the board> --kind systemic \\\n'
                      f'        --exhausted {h} --exhausted-reason "<what was '
                      f'tried and why nothing is left>" \\\n'
                      # The --lever the too-few-laps remedy above has always
                      # carried and this one never did. An --exhausted row has
                      # `lever: null` by construction otherwise, and every
                      # consumer that renders a lap -- the film's caption, the
                      # GUI's stage label, the watcher's rejected-lap line --
                      # then prints a blank or a literal "?" for the one row
                      # whose whole content is a human's reason.
                      f'        --lever "declaration: {h} has nothing further"')
            else:
                _parts.append(
                    f'{h} improved within its last {a.flat} laps, so it has '
                    f'more to give')
        # `improving` used to be every half that was not flat, which put a half
        # whose plateau is NOT ANSWERABLE into a key named for a half that is
        # getting better. The driver reads this key to write its headline, so a
        # half that had declared itself exhausted three times was reported as
        # "still improving". Two keys, two claims: `improving` is measured,
        # `unanswerable` is the absence of a measurement.
        _improving = [h for h in still if _why[h] == 'improving']
        _unanswerable = [h for h in still if _why[h] != 'improving']
        doc.update(verdict='CONTINUE', improving=_improving,
                   unanswerable=_unanswerable,
                   why={h: _why[h] for h in still}, reason=(
            '; '.join(_parts) + '. Reaching blocking == 0 is the floor, not '
            'the finish -- keep pulling levers on quality until neither half '
            'can improve either key.'))
        code = CONTINUE
    elif blocking == 0:
        doc.update(verdict='DONE-EXHAUSTED', reason=(
            'blocking == 0 and neither half improved in its last '
            f'{a.flat} recorded laps ({_halves_note(st)}). This is the best '
            f'board these levers found.'))
        code = DONE
    else:
        doc.update(verdict='STUCK', reason=(
            f'blocking == {doc["blocking"]} and neither half improved in its '
            f'last {a.flat} recorded laps ({_halves_note(st)}). Stopping here '
            f'is legitimate; calling the board finished is not. Itemise every '
            f'remaining blocker with the measurement that proves it.'))
        code = STUCK

    # LOUD, on every verdict, never only on the one it happened to change.
    # A plateau or an improvement read off two totals that graded different
    # components is a claim about the instrument, not about the board.
    for h in ('placement', 'routing'):
        _ic = st[h].get('incommensurable')
        if _ic:
            doc['reason'] += (
                f' NOT COMPARABLE: {h} laps in the window were graded over '
                f'different components ({_ic}), so improvement was judged only '
                f'within the {st[h].get("compared", 0)} lap(s) that do compare '
                f'with each other. Measured '
                f'(run 17): two cycles compared as 78 vs 79 were 92 vs 92 once '
                f'both were graded with --impedance-nets, and the board called '
                f'worse was one impedance crossing better. Re-score with the '
                f'same flags to make the comparison mean anything.')
    if doc['ungraded']:
        # Not fatal: a board with no spec files has nothing to grade those
        # components against, and making it fatal would put every corpus board
        # permanently in STUCK. But it is never silent -- a component nothing
        # examined is UNEXAMINED, and DONE must say so out loud.
        doc['reason'] += (' UNEXAMINED, and not passed: '
                          + ', '.join(doc['ungraded']) + '.')
    if doc['unknown']:
        doc['reason'] += (' A component RAN and could not answer: '
                          + ', '.join(doc['unknown'])
                          + ' -- fix the instrument before trusting any '
                            'verdict here.')
    print(json.dumps(doc, indent=1, sort_keys=True))
    return code


def row_label(row):
    """What this ledger row DID, in one line, for a human reading a list.

    A ladder, because the field that carries the answer depends on the kind of
    row: `lever` for a lap, `exhausted.reason` for a declaration (which has
    `lever: null` by construction -- the reason is the whole content of the
    row), `stop_condition` for a close-out. Falling back on the first one and
    stopping is why an `--exhausted` row rendered as a blank, or literally as
    `lap 31: systemic/?`, in every consumer that prints a lap.

    Never returns '' -- "this row says nothing" is itself a finding, and the
    caller should print it rather than an empty column.
    """
    lever = str(row.get('lever') or '').strip()
    if lever:
        return lever
    dec = row.get('exhausted')
    if isinstance(dec, dict) and str(dec.get('reason') or '').strip():
        return 'declared exhausted: ' + str(dec['reason']).strip()
    stop = str(row.get('stop_condition') or '').strip()
    if stop:
        return 'close-out: ' + stop
    return '(no lever recorded)'


def cmd_status(a):
    from board_store import Ledger
    lg = Ledger(a.ledger)
    rows = lg.entries()
    c = lg.counts()
    # ONE JSON DOCUMENT ON STDOUT, still: converge's stdout is an API that
    # callers json.loads() whole, which is why this file installs no
    # cli_banner. `unlevered` is additive; the per-row detail goes to stderr,
    # where the systemic NOTE already lives.
    _unlevered = [e for e in rows if not str(e.get('lever') or '').strip()]
    c['unlevered'] = len(_unlevered)
    # #894: the placement terms per lap, and each lap's movement against the
    # row it was recorded against. Additive, inside the one JSON document --
    # this stdout is an API that callers json.loads() whole, which is why
    # `unlevered` above is set the same way.
    _place = []
    for e in rows:
        t = placement_terms(e.get('score'))
        if not t:
            continue
        row = {'iteration': e.get('iteration'), 'kind': e.get('kind'),
               'accepted': bool(e.get('accepted')),
               'terms': {k: v.get('value') for k, v in t.items()}}
        pt = placement_terms(parent_score(rows, e))
        if pt:
            try:
                import placement_score as ps
                verdict, detail = ps.compare_terms(pt, t)
                row['vs_parent'] = verdict
                # The ACCEPT RULE #894 asks converge status to print: "the
                # named finding is gone AND no placement term regressed". The
                # second conjunct is what this can see, so it is what it says.
                row['no_term_regressed'] = verdict in ('better', 'same')
                row['delta'] = ps.format_delta(detail)
            except Exception:                                # noqa: BLE001
                pass
        _place.append(row)
    if _place:
        c['placement_terms'] = _place
    print(json.dumps(c, indent=1, sort_keys=True))
    for r in _place:
        if r.get('vs_parent') and not r.get('no_term_regressed'):
            print(f"  i{r['iteration']}  placement {r['vs_parent']}: "
                  f"{r.get('delta')}", file=sys.stderr)
    # ITEMISE THE UNLEVERED ROWS, unconditionally -- not only when the systemic
    # warning below fires. #904's inherited item is "an --exhausted row prints
    # blank, and its reason lives in exhausted.reason": that is true of an
    # ORDINARY ledger with one declaration in it, which is the common case and
    # the one where nothing else is shouting. Nesting this under the
    # systemic-share warning made it visible only on a ledger already in
    # trouble.
    for e in _unlevered:
        print(f"  i{e.get('iteration')}  {row_label(e)[:80]}", file=sys.stderr)
    if c['total'] and c['systemic'] * 2 >= c['total']:
        print("NOTE: at least half of this budget went to SYSTEMIC iterations -- "
              "changes to how the chain measures or grades itself, not to the "
              "copper. Check what is still unrouted before spending more.",
              file=sys.stderr)
    return 0


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='verb', required=True)

    q = sub.add_parser('poses', help='rank a part\'s candidate poses')
    q.add_argument('board')
    q.add_argument('--ref', required=True)
    q.add_argument('--radius', type=float, default=2.0)
    q.add_argument('--step', type=float, default=0.5)
    q.add_argument('--limit', type=int, default=12,
                   help='how many ranked poses to RETURN. It does not bound '
                        'the sweep: every candidate is still evaluated. Bound '
                        'the work with --radius / --step instead.')
    q.add_argument('--clearance', type=float, default=None,
                   help='pose-legality clearance (default: the board\'s own '
                        'Default netclass, else 0.25; run-7 S4 -- a fixed '
                        'default tighter than the board floor silently '
                        'vetoes legal poses)')
    q.add_argument('--board-edge-clearance', type=float, default=None,
                   help='pose-legality edge clearance (default: the board\'s '
                        'own min_copper_edge_clearance, else 0.55)')
    q.add_argument('--route', action='store_true',
                   help='also run tier 3 (a scoped route) on the top poses')
    q.add_argument('--route-top', type=int, default=2,
                   help='how many ranked poses to actually route (default 2)')
    q.add_argument('--affected', nargs='+', default=None,
                   help='nets a move of this part can affect; required by --route')
    q.add_argument('--route-args', nargs='+', default=None)
    q.set_defaults(fn=cmd_poses)

    w = sub.add_parser('where', help='islands, gaps and the copper walling them in')
    w.add_argument('board')
    w.add_argument('--nets', nargs='+', default=None,
                   help='nets to run forensics on (optional with --oracle, '
                        'which derives them from the remaining joins)')
    w.add_argument('--radius', type=float, default=1.0)
    w.add_argument('--oracle', action='store_true',
                   help="prepend KiCad's own join list (kicad-cli DRC after a "
                        "zone refill) as exact endpoint-pair specs, and scope "
                        "forensics to those nets when --nets is omitted")
    w.set_defaults(fn=cmd_where)

    r = sub.add_parser('record', help='store a board and record what produced it')
    r.add_argument('--ledger', required=True)
    r.add_argument('--board', required=True)
    r.add_argument('--store', default=None)
    r.add_argument('--kind',
                   choices=('completion', 'placement', 'systemic',
                            'classification'),
                   default='completion',
                   help="which half of the loop this lap belongs to. "
                        "`classification` is the L3 lap that DECIDES the shape "
                        "of the next re-entry; it changes no board, so like "
                        "`systemic` it belongs to neither half. It had to be "
                        "filed as `systemic` before, which made a decision "
                        "look like a tool change.")
    r.add_argument('--lever', default=None)
    r.add_argument('--score', default=None, help='JSON')
    r.add_argument('--score-file', default=None, metavar='PATH',
                   help='the score payload as a FILE. Prefer this: '
                        '--score "$(cat ...)" exceeds the OS argv limit at '
                        '~32kB and the shell then exits 126 BEFORE record '
                        'runs, so the lap is lost with no error. Mutually '
                        'exclusive with --score.')
    r.add_argument('--defect-json', action='append', default=None,
                   metavar='PATH',
                   help='defect-record document(s) this lap was aimed at; '
                        'repeatable. The CONTENTS are inlined into '
                        'entry["defects"] (not the path -- work dirs are '
                        'deleted and the ledger outlives them). A ledger that '
                        'keeps the MEASUREMENT can tell a later reader what '
                        'the lap was for; a ledger that keeps a paragraph '
                        'about it cannot.')
    r.add_argument('--shape', choices=('parameter', 'placement', 'floorplan'),
                   default=None,
                   help='the re-entry shape this lap acted on (the same word '
                        'L4 demands). Stored as entry["shape"].')
    r.add_argument('--render-json', action='append', default=None,
                   metavar='PATH',
                   help='render_placement --json-out document(s) that were '
                        'READ for this lap; repeatable. Stored as '
                        'entry["renders"]. The [read: ...] convention lived in '
                        'free-text --lever, so an audit could not tell a '
                        'skipped mandate from an absent trigger.')
    r.add_argument('--lens', action='append', default=None, metavar='VERDICT',
                   help='a verifier lens verdict, VERBATIM: '
                        '"VERDICT=PASS:lens=connectivity" or '
                        '"VERDICT=FAIL:lens=drc;finding=...;evidence=...". '
                        'Repeatable; stored raw as entry["lenses"]. Same '
                        'reason as --render-json: a verdict that lives in '
                        'free-text --lever cannot be told from a lens nobody '
                        'ran. --final requires the three routed-board lenses, '
                        'and requires each of them as --lens-file.')
    r.add_argument('--lens-file', action='append', default=None,
                   metavar='PATH',
                   help='the same verdict, read from the file the verifier '
                        'wrote it to: the FIRST line beginning VERDICT= is '
                        'taken, and the row records the path and the file\'s '
                        'sha256 in entry["lens_source"]. Repeatable, one per '
                        'lens. Prefer this: a retyped line is a claim about '
                        'the run, a file is a claim about a file, and '
                        'references/verifier-prompts.md already requires the '
                        'copy on disk. --final --kind completion REQUIRES it '
                        'for connectivity, drc and spec. Combines with --lens '
                        '(bare verdicts first, then files, each in the order '
                        'given).')
    # nargs='+' with 'extend' (#901): the help promised a list and argparse
    # took exactly one token per flag, so `--scope-refs R1 R2 R3` was an
    # argparse error and the writer had to repeat the flag. Both spellings now
    # work, and the whitespace/comma split below still reads a quoted lock file.
    r.add_argument('--scope-refs', action='extend', nargs='+', default=None,
                   metavar='REF',
                   help='the refs this lap was ALLOWED to move -- its search '
                        'scope. Takes a list, repeatable, and a '
                        'whitespace/comma-separated string is split, so a lock '
                        'file reads straight in. Stored as '
                        'entry["scope_refs"]. Same reason as --lens and '
                        '--render-json: a scope that lives in free-text '
                        '--lever cannot be told from a lap that scoped nothing '
                        'and swept the board. Run 16 moved 76 of 113 parts '
                        'across laps whose scopes existed only as loose '
                        'lock_*.txt files nothing reads.')
    r.add_argument('--rejected', action='store_true')
    r.add_argument('--exhausted', choices=('placement', 'routing'),
                   default=None,
                   help='declare ON THE RECORD that this half has nothing '
                        'further to optimise. `verdict` then stops asking it '
                        'to plateau. This is the lever its own too-few-laps '
                        'text has always named -- "or to say on the record '
                        'that there is nothing" -- and which no flag '
                        'implemented: routing accepts only on strict '
                        'improvement, so an exhausted half cannot produce the '
                        'accepted lap the counter wanted. Needs '
                        '--exhausted-reason. A later recorded lap of that half '
                        'SUPERSEDES it. --kind systemic is the natural kind: '
                        'the declaration changes no copper.')
    r.add_argument('--exhausted-reason', default=None, metavar='REASON',
                   help='what was tried and why nothing is left. Mandatory '
                        'with --exhausted: the declaration IS the evidence, '
                        'and an unreasoned one is just a lower --flat.')
    r.add_argument('--accept-incommensurable', default=None, metavar='REASON',
                   help='record a lap whose score graded FEWER components than '
                        'the last accepted lap of its half even though '
                        '`blocking` fell -- i.e. an improvement that may be an '
                        'artefact of measuring less. Needs a reason, and the '
                        'reason is what gets recorded: the numbers are not '
                        'being judged, a person is.')
    r.add_argument('--final', action='store_true',
                   help='mark the run-closing record; requires --stop-condition')
    r.add_argument('--stop-condition', default=None,
                   help='which stop condition ended the run (with --final). '
                        'A TOKEN -- ' + ' | '.join(STOP_TOKENS) + ' -- checked '
                        'on every record, not only when a lens failed. Prose '
                        'about WHY may follow it ("3: five laps, no new '
                        'copper"); it is split off into stop_reason.')
    r.add_argument('--stop-reason', default=None, metavar='TEXT',
                   help='why that stop condition fired, in words. The same '
                        'text may instead ride after the token in '
                        '--stop-condition; giving it twice, differently, is '
                        'refused. Stored as entry["stop_reason"].')
    r.add_argument('--argv', nargs=argparse.REMAINDER, default=None,
                   help='the command that produced it -- what makes replay '
                        'possible. Refused (exit 2) when its first token is '
                        'neither an existing file nor on PATH.')
    r.set_defaults(fn=cmd_record)

    s = sub.add_parser('step-back', help='check out an earlier board, exactly')
    s.add_argument('--ledger', required=True)
    s.add_argument('--store', default=None)
    s.add_argument('--to', default=None, help='a board sha')
    s.add_argument('--iteration', type=int, default=None)
    s.add_argument('--out', required=True)
    s.set_defaults(fn=cmd_step_back)

    y = sub.add_parser('replay', help="re-run an iteration's lever verbatim")
    y.add_argument('--ledger', required=True)
    y.add_argument('--iteration', type=int, required=True)
    y.set_defaults(fn=cmd_replay)

    t = sub.add_parser('status', help='budget spent, completion vs systemic')
    t.add_argument('--ledger', required=True)
    t.set_defaults(fn=cmd_status)

    v = sub.add_parser('verdict',
                       help='continue, or stop -- and which kind of stop')
    v.add_argument('--ledger', required=True)
    v.add_argument('--score', required=True,
                   help="the current board's score JSON (board_score --json)")
    v.add_argument('--budget', type=int, default=100,
                   help='ledger entries this run may write (default 100, the '
                        'figure convergence.md already states)')
    v.add_argument('--flat', type=int, default=5,
                   help='RECORDED laps -- accepted OR rejected -- a half may go '
                        'without improving before it counts as blocked '
                        '(default 5, ditto). A rejected lap counts because it '
                        'is the evidence that the half tried and did not '
                        'improve; routing accepts only on strict improvement, '
                        'so an exhausted half has no accepted lap left to give.')
    v.set_defaults(fn=cmd_verdict)
    return p


def main(argv=None):
    a = build_parser().parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    # Declare the lever for the WHOLE run, so every pose this CLI
    # writes carries its name. Nothing called declare_lever outside
    # tests, so the unaided instrument had no armed state at all:
    # unarmed it is silent, and armed by hand it refused the engine.
    from placement.provenance import declare_lever
    with declare_lever('converge.py', sys.argv):
        # NO cli_banner here (deliberate): converge's stdout is a JSON API --
        # `record` and `status` print documents that callers json.loads() whole
        # (tests/test_converge.py does). The other instruments' stdout is a log.
        sys.exit(main())
