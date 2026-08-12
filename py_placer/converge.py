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

def scoped_route(board, nets, out=None, extra_args=(), timeout=None):
    """Route ONLY `nets`, and return the merged summary. Seconds, not minutes.

    This is the tier that actually discriminates: placement cost says a pose
    looks better, and only a route says the copper agrees. Scoping it to the
    affected nets is what makes it affordable enough to run per candidate.
    """
    tmp = tempfile.mkdtemp(prefix='converge_t3_')
    out = out or os.path.join(tmp, 'routed.kicad_pcb')
    js = os.path.join(tmp, 'route.json')
    argv = [sys.executable, '-X', 'utf8', _ROUTE_PY, board, out,
            '--nets'] + list(nets) + ['--json-out', js] + list(extra_args)
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=timeout, cwd=ROOT)
    summary = {}
    if os.path.isfile(js):
        with open(js, encoding='utf-8') as f:
            summary = json.load(f)
    return {'argv': argv, 'returncode': r.returncode, 'board': out,
            'json': js, 'summary': summary, 'stdout_tail': r.stdout[-1500:]}


def route_verdict(summary):
    """(failures, note) from a route summary -- the tier-3 comparison key."""
    if not summary:
        return None, 'no summary'
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
#: Only lenses whose subject is unambiguous are listed. `connectivity` asks "is
#: every net actually joined", which is exactly `unrouted` + `broken`; `drc`
#: asks "does the copper break a rule", which is `drc` + `undersized`. `spec` is
#: deliberately ABSENT: it covers impedance, length and net widths, every one of
#: which is routinely ungraded, and an ungraded component contradicts nothing.
LENS_COMPONENTS = {
    'connectivity': ('unrouted', 'broken'),
    'drc': ('drc', 'undersized'),
}

_LENS_RE = r'^VERDICT=(PASS|FAIL):lens=([A-Za-z0-9_-]+)'

#: Stop conditions a `--final --kind completion` row may carry when a lens
#: FAILED. Two vocabularies, both of record: the routing half's NUMBERS
#: (convergence.md §3 -- 2 budget spent, 4 measured-unfixable) and the outer
#: loop's verdict NAMES as `verdict` prints them and L5 interpolates them.
#: DONE-EXHAUSTED is deliberately absent -- with a FAIL lens it is a
#: contradiction, refused above the membership check.
FAIL_COMPATIBLE_STOPS = ('2', '4', 'STUCK', 'BUDGET')


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
        # CASE-FOLDED. The grammar accepts [A-Za-z0-9_-]+ and the table is
        # lower-case, so `lens=Connectivity` matched the format check, missed
        # the table, and was written -- the whole gate bypassed by a shift key.
        for key in LENS_COMPONENTS.get(m.group(2).lower(), ()):
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
        # Armed INSIDE the guard: this verb's stdout is a JSON document, and
        # krt_deadline's DEADLINE:/PROGRESS: lines would land in the middle of
        # it. The guard sends them to stderr, where they belong.
        import krt_deadline
        _dl = krt_deadline.arm(a.deadline, tool='converge poses')
        poses = pose_score.rank_poses(pcb, a.board, a.ref, radius=a.radius,
                                      step=a.step, limit=a.limit, state=st,
                                      diagnostics=diag,
                                      cancel_check=(_dl.cancel_check('sweep')
                                                    if _dl else None))
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
                          # sweep has not earned it: with --deadline 0 the
                          # check fires before even the identity offset, so the
                          # old text diagnosed a part whose poses were never
                          # enumerated.
                          'note': ('the sweep was cut by --deadline before it '
                                   'finished -- this is NOT a verdict about '
                                   'the part' if _cut else
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
                p['route'] = {'failures': n, 'note': note,
                              'iterations': res['summary'].get('total_iterations'),
                              'vias': res['summary'].get('total_vias')}
    # A cut sweep returns a DIFFERENT best pose with a byte-identical document
    # shape -- measured, r=3/s=0.25: full ran 176s and chose rot 0; the same
    # call with --deadline 4 ran 7s and chose rot 90, with no key marking it
    # partial. A ranking nobody can tell is partial is worse than a slow one.
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
        except Exception:                                       # noqa: BLE001
            _score_doc = None
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
    # A run-closing record must carry the routed-board lenses. `blocking == 0`
    # and "every lens passes" are two different claims and the second had no
    # mechanism at all -- verifier-prompts.md states the conjunct and nothing
    # computed it, so a close-out could be written with no lens ever dispatched.
    if a.final and a.kind == 'completion':
        _seen = set()
        for v in (a.lens or []):
            _m = __import__('re').match(r'^VERDICT=(PASS|FAIL):lens=([A-Za-z0-9_-]+)',
                                        v.strip())
            if _m:
                _seen.add(_m.group(2))
        _need = {'connectivity', 'drc', 'spec'}
        _miss = sorted(_need - _seen)
        if _miss:
            print(f"record: --final needs the routed-board lenses and is "
                  f"missing {', '.join(_miss)}. Dispatch them "
                  f"(routing_driver --stage V5 fans them out) and pass each "
                  f"VERDICT= line as --lens. `blocking == 0` is not `every "
                  f"lens passes`. Nothing was written.", file=sys.stderr)
            return 2
        _failed = [v for v in (a.lens or []) if v.strip().startswith('VERDICT=FAIL')]
        _sc = (a.stop_condition or '').strip()
        # TWO STOP VOCABULARIES ARE OF RECORD, and both must be acceptable as
        # printed: the routing half closes on the NUMBERS of convergence.md §3,
        # and the outer loop's L5 interpolates the verdict NAMES this tool's
        # own `verdict` subcommand prints. L5's command was refused verbatim
        # here for exactly that gap -- a FAIL lens is the NORMAL case on the
        # STUCK/BUDGET paths. DONE-EXHAUSTED is the exception: done-and-
        # measured-done IS the all-lenses-pass claim.
        if _failed and _sc == 'DONE-EXHAUSTED':
            print(f"record: {len(_failed)} lens FAILED under --stop-condition "
                  f"DONE-EXHAUSTED. Done-and-measured-done IS the every-lens-"
                  f"passes claim, so a FAIL beside it is the contradiction "
                  f"L5's cross-check exists to refuse. Record STUCK or BUDGET "
                  f"(or fix the board and re-dispatch the lens), never a done "
                  f"a lens denies. Nothing was written.", file=sys.stderr)
            return 2
        if _failed and _sc not in FAIL_COMPATIBLE_STOPS:
            print(f"record: {len(_failed)} lens FAILED, so this run did not "
                  f"finish clean -- --stop-condition must be 2 (budget spent), "
                  f"4 (measured-unfixable and said so), or the loop verdict "
                  f"naming the same thing (STUCK, BUDGET), not "
                  f"{a.stop_condition!r}. A FAIL means `blocking` was not "
                  f"really zero. Nothing was written.", file=sys.stderr)
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
        for _r in reversed(_prior):
            if _r.get('accepted') and _HALF.get(_r.get('kind')) == _half \
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
             'score': json.loads(a.score) if a.score else None,
             'renders': list(a.render_json) if a.render_json else None,
             'lenses': list(a.lens) if a.lens else None,
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
        entry['stop_condition'] = a.stop_condition
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

#: Which half of the loop a ledger row belongs to. `systemic` is neither -- it
#: changes how the chain measures itself, not the board -- so it can never make
#: a half look like it is still improving.
_HALF = {'placement': 'placement', 'completion': 'routing'}


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
        elif found is not None and _HALF.get(r.get('kind')) == half:
            live = False
    return None if found is None else (found, live)


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
    ev = []                      # (accepted, key, score) in ledger order
    for r in rows:
        if _HALF.get(r.get('kind')) != half:
            continue
        if not r.get('accepted'):
            ev.append((False, None, r.get('score')))
            continue
        ev.append((True, _score_key(r.get('score')), r.get('score')))
    n_acc = sum(1 for e in ev if e[0])
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
    if not any(acc for acc, _k, _s in window):
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
    seq = [(k, s) for acc, k, s in window if acc and k is not None]
    runs, cur, hints = [], [], []
    for k, s in seq:
        cm = commensurability(cur[-1][1], s) if cur else None
        if cm:
            hints.append(cm[1])
            runs.append(cur)
            cur = []
        cur.append((k, s))
    runs.append(cur)
    runs = [[k for k, _s in r] for r in runs if len(r) >= 2]
    # An ACCEPTED lap that recorded no `blocking` is unjudged, and a plateau
    # asserted over unjudged laps is the "reported clean because unexamined"
    # error this toolchain names everywhere else. An improvement, by contrast,
    # is a DEFINITE finding and stands whatever else is in the window. So:
    # improvement wins; a plateau requires every accepted lap to have been
    # judged; anything else is not answerable and says so. A REJECTION is not
    # unjudged -- the rejection is itself the measurement.
    unjudged = sum(1 for acc, k, _s in window if acc and k is None)
    if any(min(r) < r[0] for r in runs):
        out.update(flat=False, why='improving')
    elif runs and not unjudged:
        out.update(flat=True, why='plateau')
    else:
        # Answerable again after one more comparable lap, after `flat`
        # rejections, or by declaring the half exhausted on the record. NAME
        # the cause: these three call for different actions, and a message
        # that guesses tells a still-improving half to declare itself finished.
        out.update(flat=False, why='no-comparison', unjudged=unjudged,
                   blocked=('unjudged' if unjudged else
                            'incommensurable' if hints else 'single-lap'))
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
                        f'window recorded no `blocking`, and a lap that '
                        f'measured nothing is evidence in neither direction'),
                       'incommensurable': (
                        f'they were not graded over the same components '
                        f'({st[h].get("incommensurable")})'),
                       'single-lap': (
                        'only one of them carries a score, and one number '
                        'compared with nothing is not a trend')}[
                           st[h].get('blocked', 'single-lap')]
                    + f'. So whether it plateaued is NOT ANSWERABLE, which is '
                      f'not "it did not". Re-score its laps the same way, or '
                      f'declare the half exhausted on the record:\n'
                      f'    python3 -X utf8 py_placer/converge.py record --ledger '
                      f'{a.ledger} \\\n'
                      f'        --board <the board> --kind systemic \\\n'
                      f'        --exhausted {h} --exhausted-reason "<what was '
                      f'tried and why nothing is left>"')
            else:
                _parts.append(
                    f'{h} improved within its last {a.flat} laps, so it has '
                    f'more to give')
        doc.update(verdict='CONTINUE', improving=still,
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


def cmd_status(a):
    from board_store import Ledger
    lg = Ledger(a.ledger)
    c = lg.counts()
    print(json.dumps(c, indent=1, sort_keys=True))
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
                        'the sweep: every candidate is still evaluated. Use '
                        '--deadline for that.')
    q.add_argument('--deadline', type=float, default=None, metavar='SECONDS',
                   help='wall-clock budget for the candidate sweep. The sweep '
                        'is radius/step rings x rotations and each candidate '
                        'pays a full cost evaluation; --limit only truncates '
                        'the result. Env: KRT_DEADLINE_S')
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
    r.add_argument('--kind', choices=('completion', 'placement', 'systemic'),
                   default='completion')
    r.add_argument('--lever', default=None)
    r.add_argument('--score', default=None, help='JSON')
    r.add_argument('--score-file', default=None, metavar='PATH',
                   help='the score payload as a FILE. Prefer this: '
                        '--score "$(cat ...)" exceeds the OS argv limit at '
                        '~32kB and the shell then exits 126 BEFORE record '
                        'runs, so the lap is lost with no error. Mutually '
                        'exclusive with --score.')
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
                        'ran. --final requires the three routed-board lenses.')
    r.add_argument('--scope-refs', action='append', default=None,
                   metavar='REF',
                   help='the refs this lap was ALLOWED to move -- its search '
                        'scope. Repeatable; a whitespace/comma-separated list '
                        'is split, so a lock file reads straight in. Stored as '
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
                   help='which stop condition ended the run (with --final)')
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
    # NO cli_banner here (deliberate): converge's stdout is a JSON API --
    # `record` and `status` print documents that callers json.loads() whole
    # (tests/test_converge.py does). The other instruments' stdout is a log.
    sys.exit(main())
