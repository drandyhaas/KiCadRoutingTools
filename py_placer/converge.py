#!/usr/bin/env python3
"""The run ledger: what each lap produced, where an open net is walled in,
and whether the run should continue.

VERBS

    converge.py where BOARD --nets NET ...
        What is unconnected, where the gap is, and which foreign copper is
        walling it in -- via net_forensics, which already answers this and which
        nothing in the usual chain calls.

    converge.py record --ledger L --board B --kind completion --lever ...
        Store a board by content and record what produced it.

    converge.py verdict --ledger L --score S
        Continue, or stop -- and which kind of stop: DONE-EXHAUSTED, STUCK or
        BUDGET, read off the recorded laps of both halves.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'routing', 'combined'], 'kind': 'actor'}

import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

# ROOT is the REPO root (this script lives in py_placer/), because the
# subprocess below runs with cwd=ROOT and its path is repo-relative.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ------------------------------------------- a lens verdict against the score

#: lens name -> the score components a PASS on it CONTRADICTS.
#:
#: `connectivity` asks "is every net actually joined", which is exactly
#: `unrouted` + `broken`; `drc` asks "does the copper break a rule", which is
#: `drc` + `undersized`; `spec` asks "does the board meet what was ASKED for"
#: (the verifier brief's `spec` lens, .claude/skills/plan-pcb-placement/
#: references/verifier.md: check_complete's impedance, length and width
#: clauses plus the declared mechanical facts), which is `impedance` +
#: `floorplan` + `length` + `net_widths`.
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
#: lenses cover eight of them. `assembly` is graded by the verifier brief's
#: every-mode `check_assembly` step, in its overall VERDICT line rather than a
#: lens file. Mapping it onto one of these three would make a routed-board lens
#: answerable for a check nobody asked it to run.
#: `tests/test_904_lens_components_cover_blocking.py` re-derives the nine names
#: from board_score.py's own source and fails when a new one has no home here.
LENS_COMPONENTS = {
    'connectivity': ('unrouted', 'broken'),
    # diff_pairs: the .kicad_dru pair-rule rows #1303 split out of `drc`
    'drc': ('drc', 'undersized', 'diff_pairs'),
    'spec': ('impedance', 'floorplan', 'length', 'net_widths'),
    'intent': ('floorplan',),
}

_LENS_RE = r'^VERDICT=(PASS|FAIL):lens=([A-Za-z0-9_-]+)'

#: Stop conditions a `--final --kind completion` row may carry when a lens
#: FAILED. Two vocabularies, both of record: the stop NUMBERS (2 budget
#: spent, 3 plateau, 4 measured-unfixable) and the verdict NAMES as `verdict`
#: prints them. '3' and STUCK
#: are the same stop (#1202: STUCK passed while its number was refused).
#: DONE-EXHAUSTED is deliberately absent -- with a FAIL lens it is a
#: contradiction, refused above the membership check.
FAIL_COMPATIBLE_STOPS = ('2', '3', '4', 'STUCK', 'BUDGET')

#: The WHOLE stop-condition vocabulary (#901), checked on every `record` that
#: carries one -- not only when a lens FAILED, which is what let ~500 characters
#: of prose into rows 29/30 of run 25 while the orchestrator's `4 (this half):
#: ...` was refused twice at close-out. One record, two rules, depending on a
#: lens. The numbers are the stop conditions (1 done, 2 budget spent, 3
#: plateau, 4 measured-unfixable; this tuple is their definition); the names
#: are what `verdict`
#: prints. FAIL_COMPATIBLE_STOPS is the subset legal beside a FAIL lens.
STOP_TOKENS = ('1', '2', '3', '4', 'DONE-EXHAUSTED', 'STUCK', 'BUDGET')

#: The three re-entry shapes, named once; `--shape` records which one a lap
#: acted on. Spelled as a constant since #963 required one on a classification
#: row: the refusal and the argparse choices must be the same list, or the
#: message can name a word the parser refuses.
SHAPES = ('parameter', 'placement', 'floorplan')

#: The stop token that claims the board is MEASURED-UNFIXABLE rather than
#: merely not finished -- stop condition 4. Run 29 recorded
#: exactly this, falsely, about an impedance clause whose own log's next two
#: lines read `SE fallback: 2/2 member net(s) routed`, with no classification
#: row anywhere in its ledger.
#:
#: `STUCK` is deliberately NOT here, and the distinction is the point: STUCK is
#: `verdict`'s own name for "neither half improved in its last N laps", which
#: is stop condition 3, a plateau. A plateau is a statement about a search;
#: "unfixable" is a statement about a board.
UNFIXABLE_STOPS = ('4',)

#: MSYS2's argv-rewrite signature. Git Bash rewrites any argument starting with
#: `/` into a Windows path unless MSYS2_ARG_CONV_EXCL is set, and EVERY KiCad
#: net name is `/`-prefixed -- so `/D_P` reaches the tool as
#: `C:/Program Files/Git/D_P`. Nothing warns, because a tool cannot tell a
#: mangled net name from a net that does not exist (CLAUDE.md). Row 31 of run 25
#: held exactly this in its recorded command line.
#: `.search`, never `.match`: the rewrite is at position 0 of a bare token but
#: NOT of `--impedance-nets=/D_P`, which MSYS rewrites in place; and prose
#: cannot be split on whitespace to find it because the rewritten path itself
#: contains a space ("Program Files").
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


# ONE rule for what a `blocking` is (#1088): `_score_key` (the ranking),
# `record` (the refusal), `check_complete` and `run_watch` all use the same
# two functions. They live in `py_router/ledger_score.py` so the router side
# can import them without importing a placement engine (`_placer_path`'s
# one-way rule); converge re-exports them.
from ledger_score import blocking_defect, blocking_value  # noqa: E402,F401


def score_board_binding(board, payload, board_sha=None):
    """(binding, payload_sha) -- does this score payload grade THIS board?

    ONE implementation of a question FIVE sites used to answer separately, and
    they answered it at different STRENGTHS: `record` warned, L3 and L5
    refused at exit 4, and the close-out compare refused about a different
    document. #963's contributor measured that asymmetry from the outside -- a
    stale score `record` accepted with `accepted: true` was refused by the very
    next stage -- and named it "different enforcement strengths". The strengths
    are a policy each caller may legitimately choose; the PREDICATE is not, and
    it is the predicate that had drifted into five copies.

    FOUR-VALUED RATHER THAN A BOOL, because `unbound` is a different operator
    action from `other`: a payload with no `board_sha` at all is a pre-B4
    board_score or a hand-built JSON, and `record` discloses that in its own
    sentence. Collapsing it into False deletes that disclosure silently, which
    is why `_grades_another_board` below is a WRAPPER and not the interface.

    WHAT THIS DOES NOT COVER, said here rather than implied: it is the only
    implementation in `converge.py`, the one file
    `tests/test_963_one_binding_predicate.py` scans. There is a sibling
    of the same shape in `py_tools/render_placement.py` (~:570, ~:641-651),
    hand-rolled on `hashlib` and degrading the same way, which #963 does not
    touch and no gate here can see.

    `unknown` is "I could not tell" -- no board, unreadable, board_store
    unimportable -- and it must NEVER read as a mismatch. A check that switched
    itself ON because it could not answer would be the same class of mistake it
    exists to catch.

    `board_sha` lets a caller that has ALREADY hashed the board hand the digest
    in (`cmd_record` has it from `store.put`), so the board is not read twice
    for one answer.
    """
    psha = payload.get('board_sha') if isinstance(payload, dict) else None
    if not psha:
        return ('unbound', None)
    if board_sha:
        return ('this' if board_sha == psha else 'other', psha)
    if not board or not os.path.isfile(board):
        return ('unknown', psha)
    try:
        from board_store import sha256_file
        return ('this' if sha256_file(board) == psha else 'other', psha)
    except Exception:                                           # noqa: BLE001
        return ('unknown', psha)


def _grades_another_board(board, score):
    """Does this score payload demonstrably grade a DIFFERENT board?

    True only when `board_sha` is present AND differs. Absent sha, unreadable
    board, missing board_store: False -- "I could not tell" must not switch a
    check off. Baseline rows legitimately attach a parent score to a rejected
    candidate (record already WARNS about that), and a check that compared a
    verdict about board A against board B's numbers would be the same class of
    mistake it exists to catch.

    Kept as a one-line wrapper over `score_board_binding` rather than deleted
    because the bool is the shape its one caller wants -- the lens-vs-score
    skip in `cmd_record`, which needs "is this foreign" and nothing finer.
    """
    return score_board_binding(board, score)[0] == 'other'


def read_lens_file(path):
    """(line, lineno) -- the FIRST line of `path` that begins `VERDICT=`.

    The verifier brief (.claude/skills/plan-pcb-placement/references/verifier.md)
    requires every verifier to write its verdict to disk ("a reply is a
    notification and notifications get lost"). Nothing read those files: the line was retyped into the
    ledger from a reply, so it recorded a CLAIM ABOUT THE RUN where it could
    have recorded a claim about a file. This is the reader.

    SELECTION IS DELIBERATELY DUMB, and validation is left where it already
    lives. Selecting "the first line matching `_LENS_RE`" instead would step
    silently past a MALFORMED verdict to a well-formed one further down -- the
    exact normalisation the verifier grammar forbids ("stored RAW, so a
    malformed line stays visible instead of being normalised into something
    that reads like a pass"). So the first `VERDICT=`-prefixed line wins
    whatever it says, and the grammar gate that already exists names it.

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
        f"(the verifier brief, .claude/skills/plan-pcb-placement/references/verifier.md). A boundary-verification verdict "
        f"spells `check=<1-5>` instead of `lens=<name>` and is not a lens: "
        f"cite it in the report, not here.")


def lens_name(raw):
    """The lens a VERDICT= line speaks about, lower-cased, or None.

    ONE definition, because this file's grammar check and its --final
    lens-set loop (which re-inlined the pattern as a literal beside an
    `__import__('re')`) each had one: two regexes for one grammar are two
    places for a `lens=Connectivity` to be handled differently.

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

    `record` took the verdict string VERBATIM and validated only its
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
                  'length': '--length-groups', 'net_widths': '--net-min-widths',
                  # #1183: check_assembly's moved-vs-baseline courtyard gate.
                  'assembly.courtyard_gating': '--baseline'}


def ungraded_set(score):
    """Which components this score did NOT measure, or None if unknowable.

    Three sources, unioned, because any can be absent: board_score's own
    `ungraded` list, every key `blocking_by` reports as null (a null in
    `blocking_by` IS the component saying it did not answer), and the
    assembly component's own `courtyard_gating_armed` (#1183) -- so a score
    written before board_score listed 'assembly.courtyard_gating' still
    compares with one written after, unarmed against unarmed.
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
    asm = (score.get('components') or {}).get('assembly') \
        if isinstance(score.get('components'), dict) else None
    if isinstance(asm, dict):
        # Only when the score SAYS something about assembly: a partial
        # document with no assembly component reads from the list alone.
        if asm.get('courtyard_gating_armed') is not True:
            out.add('assembly.courtyard_gating')
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


def _resolve_parent(a, store, lg):
    """(parent_sha, parent_source, error) for a `record` row (#1034).

    `--parent` (a board path, or a sha / unique sha prefix the store or the
    ledger knows) wins; else the last accepted row -- which is whichever
    lineage accepted last, so the caller prints a NOTE. A bad `--parent` is an
    error, and nothing may be written.
    """
    from board_store import sha256_file
    rows = lg.entries()
    if a.parent:
        p = a.parent
        if os.path.isfile(p):
            return sha256_file(p), 'parent', None
        key = p.strip().lower()
        known = {r.get('result_sha') for r in rows} | {
            d for d in (os.listdir(store.root) if os.path.isdir(store.root)
                        else ()) if not d.endswith('.partial')}
        known.discard(None)
        if re.fullmatch(r'[0-9a-f]{6,64}', key):
            hits = sorted(s for s in known if s.startswith(key))
            if len(hits) == 1:
                return hits[0], 'parent', None
            if len(hits) > 1:
                return None, None, (f"--parent {p!r} matches {len(hits)} "
                                    f"stored boards; give more of the sha")
        return None, None, (f"--parent {p!r} is neither an existing board "
                            f"file nor a sha this store or ledger knows")
    prev = lg.last_accepted()
    if prev and prev.get('result_sha'):
        return prev.get('result_sha'), 'last_accepted', None
    return None, None, None


def cmd_record(a):
    from board_store import BoardStore, Ledger
    # --score-file: the payload as a PATH, never as argv. board_score payloads
    # run 24-49 kB, and past roughly 32 kB of total argv the SHELL fails with
    # `Argument list too long` and exits 126 -- before `record` ever execs.
    # Nothing is written, and because the 126 belongs to the shell rather than
    # to this tool, a caller who does not re-count the ledger rows sees no
    # error at all. Run 9 lost a lap that way and found it only by chance.
    # `verdict --score` is a path too.
    _score_text = None
    if a.score_file:
        try:
            with open(a.score_file, encoding='utf-8') as _sf:
                _score_text = _sf.read()
        except OSError as _e:
            print(f"record: --score-file unreadable: {_e}", file=sys.stderr)
            return 2
    # --lens-file: the verdict as a PATH, beside --score-file and for the same
    # reason. What a path buys that a retyped line does not is PROVENANCE: the
    # row stores where the verdict came from and the sha256 of the bytes it
    # came from, so a reader can open the same file.
    _lenses, _lens_src = [], []
    for _p in (a.lens_file or []):
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
        _lenses.append(_line)
        # The WHOLE FILE is hashed, not the selected line. A verdict line is
        # `finding=` and `evidence=` as much as it is PASS or FAIL, and hashing
        # the line alone would let everything around it be rewritten with the
        # row still verifying.
        #
        # TWO paths. `path` is as the caller spelled it, which is this repo's
        # idiom everywhere else -- and on its own it is unresolvable, because
        # a printed command's path is relative to the work dir and a later
        # reader is somewhere else. `abspath` is what the file WAS at record
        # time. The sha256 remains the identity; the paths are where to look
        # for it.
        _lens_src.append({'path': _p, 'abspath': os.path.abspath(_p),
                          'sha256': sha256_file(_p), 'line': _no})
    # A mangled name in --lever is a WARNING, not a refusal: the lever is prose
    # for a human, so it misleads a reader without making the row wrong.
    if a.lever and _MANGLED_RE.search(str(a.lever)):
        print(f"record: WARNING -- --lever contains an MSYS2-rewritten token "
              f"(a '/'-prefixed net name turned into a Windows path), so the "
              f"prose names a net that does not exist. {_MSYS_REMEDY}.",
              file=sys.stderr)
    # Lens verdicts are stored RAW, so the grammar stays owned by the
    # verifier brief and a malformed line stays visible instead of being
    # normalised into something that reads like a pass. Refuse the shape at
    # write time, so the ledger never holds a row that cannot be read back.
    if _lenses:
        _badl = [v for v in _lenses if not re.match(_LENS_RE, v.strip())]
        if _badl:
            print("record: a --lens-file's VERDICT= line must be the "
                  "verifier's per-lens verdict, "
                  "e.g. 'VERDICT=PASS:lens=connectivity' or "
                  "'VERDICT=FAIL:lens=drc;finding=...;evidence=...'. "
                  f"Not: {_badl[0]!r}. Nothing was written.", file=sys.stderr)
            return 2
    # THE VERDICT AGAINST THE NUMBERS IN THE SAME ROW. Format was the only
    # thing ever checked, so a PASS could be recorded beside the score that
    # refutes it -- see lens_contradictions for the measured row. Refuse before
    # anything is written, the same posture as the lens grammar.
    _score_doc = None
    if _score_text is not None:
        try:
            _score_doc = json.loads(_score_text)
        except (OSError, ValueError) as exc:                    # noqa: BLE001
            # REFUSE, rather than carrying None forward. This used to swallow
            # the failure here and at the board_sha check below, then hit a
            # THIRD, unguarded `json.loads` while building the ledger
            # entry -- so a truncated --score-file crashed with a traceback at
            # exit 1, AFTER store.put() had already written the board into the
            # content store. Same defect as the route-summary read this change
            # is about, in the same file: a document that exists and does not
            # parse. `--score-file` reads it off disk, so the "valid prefix of
            # a killed writer" case applies here too.
            print(f"record: --score-file is not readable JSON "
                  f"({type(exc).__name__}: {exc}). Nothing was written.",
                  file=sys.stderr)
            return 2
    # A SCORE IS AN OBJECT (#1078). `[1]`, `"x"` or `5` parse, pass every
    # dict-guarded check below, get APPENDED -- and then the summary's
    # `sc.get('failures')` raised AttributeError with the row already in the
    # append-only ledger.
    if _score_doc is not None and not isinstance(_score_doc, dict):
        print(f"record: --score-file must hold a JSON object (board_score's "
              f"--json output); this is a JSON {type(_score_doc).__name__}. Nothing "
              f"was written.", file=sys.stderr)
        return 2
    # ...AND ITS `blocking` IS A COUNT (#1071, #1075). `verdict` ranks every lap
    # on it, and the ledger is append-only, so this is the one place to stop a
    # value that breaks or bends every later verdict on the ledger.
    _bad_blocking = blocking_defect(_score_doc.get('blocking')) \
        if isinstance(_score_doc, dict) else None
    if _bad_blocking:
        print(f"record: --score-file's `blocking` is {_bad_blocking}. `verdict` "
              f"ranks every lap on this number, so a value that is not a "
              f"non-negative number either breaks every later verdict on this "
              f"ledger (#1071: a per-term dict made the plateau test compare "
              f"two dicts) or ranks wrong (false == 0 reads as a finished "
              f"board). Record the total as a number -- board_score's own "
              f"`blocking` -- or leave `blocking` out (or null) if nothing "
              f"measured it. Nothing was written.", file=sys.stderr)
        return 2
    if _lenses and isinstance(_score_doc, dict) and \
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
    elif _lenses and isinstance(_score_doc, dict):
        _contra = lens_contradictions(_lenses, _score_doc)
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
                f"VERDICT= line at all -- no verifier had run.\n\n"
                f"Either fix the board and re-score it, or record what the "
                f"verifier actually found, a --lens-file whose line reads:\n"
                f"  VERDICT=FAIL:lens={_contra[0][0]};finding=<what is "
                f"wrong>;evidence=<file#/path/to/the/number>\n\n"
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
    # A DECLARATION IS A CLAIM ABOUT --board, SO IT IS REFUSED, NOT WARNED
    # (#963). An ordinary row keeps the warning below and must: a baseline row
    # legitimately attaches a parent score to a rejected candidate, which is
    # what tests/test_converge.py::test_record_warns_on_unbound_or_mismatched_
    # score pins. A declaration has no such case -- its whole content is "this
    # board's half has nothing left", so numbers taken on a different board are
    # not weak evidence for it, they are evidence about something else.
    # Checked HERE, before `store.put`, so the message can honestly say nothing
    # was written; the shared predicate hashes the board itself, which costs
    # one extra read on the rare `--exhausted` + `--score-file` path.
    if a.exhausted and isinstance(_score_doc, dict):
        _eb, _eps = score_board_binding(a.board, _score_doc)
        if _eb == 'other':
            print(f"record: --exhausted {a.exhausted} was given a score that "
                  f"grades a DIFFERENT board (score board_sha "
                  f"{str(_eps)[:12]}... is not --board "
                  f"{os.path.abspath(a.board)}).\n\nA declaration is a claim "
                  f"about the board it names, and it outlives the run: run 29 "
                  f"declared placement exhausted against a board that existed "
                  f"for eight minutes, and the claim survived three close-out "
                  f"calls. Score the board you are declaring about, or drop "
                  f"the score -- an unscored declaration is still a "
                  f"declaration. Nothing was written.", file=sys.stderr)
            return 2
    # A CLASSIFICATION WITH NO SHAPE IS A DECISION THAT RECORDED NO DECISION
    # (#963). `--shape` defaults to None and nothing required it, so the one
    # row the retry gate reads could be written by a command that names
    # nothing -- which would make that gate a formality one flagless call
    # clears. The three words are not interchangeable and the cost of guessing
    # is asymmetric: a wrong `parameter` spends iterations on a board no
    # parameter can fix, a wrong `placement` throws away a routed board.
    if a.kind == 'classification' and not a.shape:
        print(f"record: --kind classification needs --shape "
              f"{' | '.join(SHAPES)}. The shape IS the decision -- it is what "
              f"the next re-entry changes, and a classification row that "
              f"names none records that a decision was made without recording "
              f"which. Nothing was written.", file=sys.stderr)
        return 2
    # ...AND THE MEASUREMENT THAT NAMED IT. `--shape` alone left the row
    # writable by a command that records no evidence: a verifier measured run
    # 29's exact false close-out ACCEPTED after one lever-less `record --kind
    # classification --shape parameter`, which made the gate a formality
    # rather than a demand for a decision. Same `.strip()` test, and the same
    # reason, as `--exhausted-reason` thirty lines above: an unreasoned
    # declaration is just a lower --flat with extra steps.
    # ...AND IT MAY NOT JUST SAY THE SHAPE BACK. `--shape parameter --lever
    # parameter` cleared every gate in this item, which is `--lever ""` with a
    # word typed in it. This catches THAT SPELLING ONLY and nothing cleverer:
    # no gate reads a sentence and knows whether a measurement is behind it,
    # and a length or word-count rule would refuse an honest short lever while
    # still passing `--lever "congestion"`. The limit is disclosed rather than
    # papered over -- see #963's sub-issue A.
    if (a.kind == 'classification'
            and (a.lever or '').strip().strip('.:;,-').lower() in SHAPES):
        print(f"record: --lever {a.lever!r} only names the shape again. The "
              f"lever is the MEASUREMENT that made `{a.shape}` the answer -- "
              f"the congestion read, the escape-face census, the DRC cluster "
              f"-- so that the next reader can check the decision instead of "
              f"taking it. Nothing was written.", file=sys.stderr)
        return 2
    if a.kind == 'classification' and not (a.lever or '').strip():
        print("record: --kind classification needs --lever \"<the shape, and "
              "the measurement that names it>\". The row's job is to say WHY "
              "the next re-entry changes what it changes; a shape with no "
              "measurement behind it is the default answer of a classifier "
              "that could not see congestion. The three shapes cost very "
              "different things to get wrong -- a mistaken `parameter` "
              "spends iterations on a board no parameter can fix, a mistaken "
              "`placement` throws away a routed board. Nothing was written.",
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
              f"{' | '.join(STOP_TOKENS)} -- the numbers are the stop "
              f"conditions (1 done, 2 budget spent, 3 plateau, 4 measured-"
              f"unfixable) and the names are what `verdict` prints. Prose "
              f"about WHY goes after it (\"3: five laps, no new copper\") and "
              f"lands in the row's stop_reason. Nothing was written.",
              file=sys.stderr)
        return 2
    # A "MEASURED-UNFIXABLE" CLAIM NEEDS THE MEASUREMENT ON THE RECORD (#963).
    # Run 29 closed with stop condition 4 on an impedance clause whose own log
    # said, two lines later, `SE fallback: 2/2 member net(s) routed`. Its
    # ledger held no classification row at all -- L3 and L4 were never invoked
    # across 439 commands -- so the claim rested on 57 inline routing calls
    # nothing had classified. An outside verifier refuted it an hour later and
    # the re-route took BLOCKING 2 -> 0.
    #
    # BOUND TO THE ROW THAT MAKES THE CLAIM, not to the stage that prints it:
    # a gate in a driver is another thing to walk past.
    if a.final and _stop_token in UNFIXABLE_STOPS:
        _prior = Ledger(a.ledger).entries() if os.path.isfile(a.ledger) else []
        _cls = _classification_state(_prior)
        # EITHER HALF, unlike the retry gate. "This board cannot be fixed" is a
        # claim about the whole board, so a placement lap after the decision
        # makes it as stale as a routing lap does -- `classification(shape=
        # placement)` followed by six placement laps used to reach this claim
        # untouched. The RETRY gate counts routing only, and for the opposite
        # reason: there, a placement lap is the decision being acted on.
        _since = None if _cls is None else sum(_cls['laps_since'].values())
        _rej = _classification_rejected(_prior)
        if _cls is None or _since:
            _what = ((f'{_rej} classification row(s) were recorded and '
                      f'every one was --rejected, so none of them is a '
                      f'decision this close-out can rest on'
                      if _rej else 'no classification row was ever '
                                   'recorded')
                     if _cls is None else
                     f'{_since} lap(s) were recorded after the last '
                     f'classification (iteration {_cls["iteration"]}, shape '
                     f'{_cls["shape"]}): '
                     f'{_cls["laps_since"]["placement"]} placement, '
                     f'{_cls["laps_since"]["routing"]} routing')
            print(f"record: --stop-condition {_stop_token} says the board is "
                  f"MEASURED-UNFIXABLE, and in this ledger {_what}.\n\n"
                  f"That is the strongest claim a close-out makes and the "
                  f"one run 29 recorded falsely: an impedance clause declared "
                  f"geometrically unsatisfiable on a log whose next two lines "
                  f"read `SE fallback: 2/2 member net(s) routed`, over 57 "
                  f"routing calls nothing had classified. Write the decision "
                  f"the claim rests on, then record the close-out:\n\n"
                  f"  python3 -X utf8 py_placer/converge.py record --ledger "
                  f"{a.ledger} \\\n"
                  f"      --board {a.board} --kind classification "
                  f"--shape <{'|'.join(SHAPES)}> \\\n"
                  f"      --lever \"<the measurement that names the shape>\"\n\n"
                  f"A plateau is a different claim and needs none of this: "
                  f"--stop-condition 3, or the STUCK the verdict prints. "
                  f"Nothing was written.", file=sys.stderr)
            return 2
    # #901: these two are about --final, NOT about which half it closes.
    # They sat inside the `kind == 'completion'` gate below, so
    # `--kind systemic --final --stop-condition DONE-EXHAUSTED` beside a FAIL
    # lens was accepted -- a one-word bypass of the entire
    # run-closing contradiction check, in the same shape ("one record, two
    # rules, depending on something orthogonal") this issue is about. The
    # THREE-LENS requirement stays completion-only: that one really is about
    # the routed board.
    _failed = [v for v in _lenses if v.strip().startswith('VERDICT=FAIL')]
    # TWO STOP VOCABULARIES ARE OF RECORD, and both must be acceptable as
    # printed: a close-out may carry the stop NUMBERS (1-4) or the verdict
    # NAMES this tool's own `verdict` subcommand prints: a FAIL lens is the
    # NORMAL case on the STUCK/BUDGET paths.
    # DONE-EXHAUSTED is the exception: done-and-measured-done IS the
    # all-lenses-pass claim.
    # The extracted TOKEN, so `4 (this half): <reason>` is judged as a 4.
    _sc = _stop_token or ''
    if a.final and _failed and _sc == 'DONE-EXHAUSTED':
        print(f"record: {len(_failed)} lens FAILED under --stop-condition "
              f"DONE-EXHAUSTED. Done-and-measured-done IS the every-lens-"
              f"passes claim, so a FAIL beside it is the contradiction "
              f"this check exists to refuse. Record STUCK or BUDGET "
              f"(or fix the board and re-dispatch the lens), never a done "
              f"a lens denies. Nothing was written.", file=sys.stderr)
        return 2
    if a.final and _failed and _sc not in FAIL_COMPATIBLE_STOPS:
        print(f"record: {len(_failed)} lens FAILED, so this run did not "
              f"finish clean -- --stop-condition must be 2 (budget spent), "
              f"3 (plateau), 4 (measured-unfixable and said so), or the "
              f"loop verdict naming the same thing (BUDGET, STUCK), not "
              f"{a.stop_condition!r}. A FAIL means `blocking` was not "
              f"really zero. Nothing was written.", file=sys.stderr)
        return 2

    # A run-closing COMPLETION record must carry the routed-board lenses.
    # `blocking == 0` and "every lens passes" are two different claims and the
    # second had no mechanism at all -- the verifier prompts stated the
    # conjunct and nothing computed it, so a close-out could be written with
    # no lens ever dispatched. This one IS completion-only: it is about the
    # routed board.
    if a.final and a.kind == 'completion':
        _seen = {n for n in (lens_name(v) for v in _lenses) if n}
        _need = {'connectivity', 'drc', 'spec'}
        _miss = sorted(_need - _seen)
        if _miss:
            # The old text said "routing_driver --stage V5 fans them out".
            # There is no routing_driver.py on main, and V1-V5 survive here
            # only as prose. (It DID exist -- created at 255af97d, grown to
            # 19 stages, on 10 commits, none of them on main; the maintainer
            # removed it twice by name because it forked before #562 and went
            # on emitting two steps main had deleted. "Never has been" was
            # wrong, and a refusal is the worst place to be wrong about what
            # exists.) This is a REFUSAL, i.e. the
            # one message whose entire job is to say what to do next, so it
            # names a file the reader can open instead of a tool they cannot
            # find.
            # The path is on ONE line on purpose. Split across two f-string
            # fragments it is still correct for a human and invisible to any
            # grep -- including the cited-path guard this repo runs, whose
            # citation pattern needs a '/' inside a single token.
            _ref = ('.claude/skills/plan-pcb-placement/references/verifier.md')
            print(f"record: --final needs the routed-board lenses and is "
                  f"missing {', '.join(_miss)}. Have an independent verifier "
                  f"({_ref}) write one `VERDICT=PASS|FAIL:lens=<name>` line "
                  f"per lens to a file, and pass each file as --lens-file. "
                  f"`blocking == 0` is not `every lens passes`. "
                  f"Nothing was written.", file=sys.stderr)
            return 2
    store = BoardStore(os.path.join(os.path.dirname(a.ledger), 'boards'))
    # #1034: the parent is the board this lap was MADE FROM, resolved BEFORE
    # anything is stored so a bad --parent writes nothing. It used to be the
    # last ACCEPTED row unconditionally, so two lineages run side by side
    # chained across each other (run 32: A1, made from A0, parented on B0).
    _parent_sha, _parent_source, _parent_err = _resolve_parent(
        a, store, Ledger(a.ledger))
    if _parent_err:
        print(f"record: {_parent_err}. Pass the board this lap was made "
              f"from (its path, or a recorded result_sha). Nothing was "
              f"written.", file=sys.stderr)
        return 2
    if _parent_source == 'last_accepted':
        print(f"record NOTE: parent_sha is the last ACCEPTED row "
              f"({_parent_sha[:12]}), because no --parent was given. When "
              f"lineages run in parallel "
              f"that is whichever lineage accepted last -- pass --parent "
              f"<the board this lap was made from>.", file=sys.stderr)
    if _parent_source == 'parent' and a.parent and os.path.isfile(a.parent):
        store.put(a.parent)         # so the store holds the parent too
    sha = store.put(a.board)
    # Run-3 B4: three ledger entries shipped carrying a PRIOR board's score
    # because a score is free JSON with no binding to --board. board_score
    # now embeds board_sha; warn LOUDLY when it is absent or names a
    # different board than the one being recorded. A warning rather than a
    # refusal: baseline rows legitimately attach a parent score to a
    # rejected candidate -- but never silently.
    _binding, _payload_sha = 'unbound', None
    if _score_doc is not None:
        # Through the SHARED predicate, and with the digest `store.put` just
        # computed handed in: this used to re-parse the score behind a bare
        # `except Exception` even though `_score_doc` was already parsed and
        # validated above, so the mismatch was judged on a second, weaker read
        # of the same string. Passing `board_sha=sha` also spares the board a
        # second hash -- `store.put` computed exactly this digest one line up.
        _binding, _payload_sha = score_board_binding(a.board, _score_doc,
                                                     board_sha=sha)
        if _binding == 'unbound':
            print("record WARNING: score payload carries no board_sha "
                  "(pre-B4 board_score, or hand-built JSON) -- the ledger "
                  "cannot verify it grades THIS board.", file=sys.stderr)
        elif _binding == 'other':
            print(f"record WARNING: score payload grades a DIFFERENT board "
                  f"(payload board_sha {_payload_sha[:12]}... != recorded "
                  f"board {sha[:12]}...). Run-3 shipped three stale-payload "
                  f"entries exactly this way; if this attachment is "
                  f"deliberate (baseline row on a rejected candidate), say "
                  f"so in --lever.", file=sys.stderr)
    lg = Ledger(a.ledger)
    # Run-7 ledger defects, each caught only by a human re-reading the file
    # afterwards. The first two are warnings: each has a legitimate shape, and
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
    # (2) Back-fill. An entry timestamped before the one it follows means the
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
    # (3) A score that measured a DIFFERENT SET OF COMPONENTS than the lap it
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
        # report, and can reach the REFUSAL below for a lap that is fine.
        for _r in reversed(_prior):
            if _r.get('accepted') and _is_lap(_r, _half) \
                    and isinstance(_r.get('score'), dict):
                _pacc = _r
                break
        _cm = commensurability((_pacc or {}).get('score'), _score_doc) \
            if _pacc else None
        if _cm:
            _differ, _hint, _false = _cm
            if _false and not a.rejected:
                print(
                    f"record: this lap looks like an IMPROVEMENT only because "
                    f"it graded FEWER components.\n"
                    f"  blocking {(_pacc.get('score') or {}).get('blocking')} "
                    f"-> {_score_doc.get('blocking')}, but the components that "
                    f"differ are: {_hint}\n\n"
                    + ("Routing accepts a lap on STRICT improvement of "
                       "`blocking`, so " if _half == 'routing' else
                       "This half's laps are ranked by `blocking` and the "
                       "plateau test compares them, so ") +
                    f"a lap that quietly measures one component "
                    f"less enters the ledger as progress and the history "
                    f"acquires a false monotone trend. Measured (run 17): two "
                    f"cycles compared as 78 vs 79 were 92 vs 92 once both were "
                    f"graded with --impedance-nets, and the board called worse "
                    f"was one impedance crossing better.\n\n"
                    f"Re-score this board with the SAME flags as iteration "
                    f"{_pacc.get('iteration')}, or record it --rejected. "
                    f"Nothing was written.", file=sys.stderr)
                return 2
            print(f"record WARNING: INCOMMENSURABLE with iteration "
                  f"{_pacc.get('iteration')}, the last accepted {_half} lap: "
                  f"the two scores did not grade the same components "
                  f"({_hint}). `blocking` is a total, so the difference "
                  f"between these two rows is partly a difference in what was "
                  f"MEASURED, not in the board.", file=sys.stderr)
    entry = {'iteration': lg.next_iteration(), 'kind': a.kind,
             # #1034: resolved above -- --parent, else the last accepted row
             # (NOTE printed); the source says which.
             'parent_sha': _parent_sha,
             'parent_source': _parent_source,
             'result_sha': sha, 'lever': a.lever,
             # _score_doc, not a THIRD json.loads: the payload was parsed and
             # validated once at the top of this function, which is the only
             # place that can still refuse before anything is written.
             'score': _score_doc,
             # L4 requires a --shape and the ledger has never kept it, so
             # "which shape did we re-enter at, and did it work" was not a
             # question the record could answer.
             'shape': a.shape,
             'lenses': list(_lenses) if _lenses else None,
             # WHERE each lens verdict came from, positionally parallel to
             # `lenses`: {path, abspath, sha256, line} per --lens-file.
             #
             # A LIST, not a dict keyed by lens name: two lines may carry the
             # same lens (a corrected re-record, a verifier that emitted two),
             # and a dict drops one of them silently. `lenses` is already a
             # positional raw list, and a parallel list is the only shape that
             # cannot drift out of correspondence with it under append.
             'lens_source': _lens_src if _lenses else None,
             'accepted': not a.rejected}
    if a.final:
        entry['final'] = True
    # The TOKEN alone, so a reader (and `verdict`) can match it
    # against the vocabulary instead of parsing prose (#901). The reason keeps
    # its own key -- absent, not empty, when there is none. Recorded on ANY row
    # that carries one, not only a --final row.
    if _stop_token:
        entry['stop_condition'] = _stop_token
    if _stop_reason:
        entry['stop_reason'] = _stop_reason
    if a.exhausted:
        entry['exhausted'] = {'half': a.exhausted,
                              'reason': a.exhausted_reason.strip()}
    if _binding in ('other', 'unbound') and isinstance(_score_doc, dict):
        # THE WARNING, ON THE ROW (#963). It has always gone to stderr and
        # vanished, while the row kept a score `_score_key` will happily rank
        # -- so no later reader could tell a warned row from a clean one, which
        # is the contributor's "stale scores may be stored only as explicitly
        # labelled attachments". NOTHING READS THIS YET, deliberately: making
        # `_score_key` or `_half_state` skip such a row would move plateau
        # windows on every historical ledger, which is a large behaviour change
        # to hide inside a "just record it" line.
        # No `board_sha` key here: it would be `result_sha` on this same row,
        # byte for byte, and two numbers for one fact is the defect
        # `_declaration`'s docstring refuses a field for.
        entry['score_stale'] = {'binding': _binding,
                                'payload_sha': _payload_sha}
    e = lg.append(entry)
    if a.exhausted:
        print(f"recorded: {a.exhausted} declared EXHAUSTED. `verdict` will "
              f"stop asking that half to plateau, and the reason is in the "
              f"ledger where every re-entry reads it. A later "
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
    # `str(...)`, and a dict check, because this reads ROWS FROM A FILE. A
    # hand-built or truncated ledger carrying `"kind": []` raised
    # `TypeError: unhashable type` out of whichever stage was walking it --
    # a reader crashing on a bad row rather than reading it as "not this half".
    if not isinstance(row, dict):
        return False
    if _HALF.get(str(row.get('kind') or '')) != half:
        return False
    if row.get('final'):
        return False
    if isinstance(row.get('exhausted'), dict):
        return False
    return True


def _score_key(score):
    """(blocking, (joins_needed, *quality)) as a comparable tuple, or None if
    not gradeable.

    Lexicographic, never a weighted sum: a weighted sum lets a router buy off a
    disconnected net with a lower via count. `joins_needed` (board_score, #1340:
    the joins the open nets still need) ranks ahead of quality, because
    `blocking` counts an unrouted net as one whatever its size: a lap that
    strands a 17-pad rail has fewer vias BECAUSE the rail is missing, and read
    as an improvement on vias alone. A score without it is 0 at `blocking` 0
    (no open net needs a join) and unmeasured (sorts last) above it. `blocking == None` is NOT zero --
    it means a component that was asked for could not answer -- so it sorts
    worse than any real number rather than reading as a perfect board.

    A `blocking` that is not a count (`blocking_value`: a dict, a boolean, a
    string, NaN, a negative) is None here too (#1071): the row is still a lap,
    but an UNJUDGED one, never ranked and never read as plateaued.
    """
    if not isinstance(score, dict):
        return None
    b = blocking_value(score.get('blocking'))
    q = score.get('quality')
    # `quality` IS NOT NECESSARILY A DICT. `record` accepts any JSON for it, so
    # `{"quality": [1, 2]}` reaches here and `q.get` raised AttributeError --
    # the same crash class as #936 D1, in the same function, one line above the
    # guard added for it. Worse than a one-off: the raising value lands in the
    # LEDGER, and thereafter every `verdict` on that ledger tracebacks in the
    # row comprehension no matter how good its `--score` is, so L5 reports "the
    # score is not a measurement" about the wrong artifact. A quality that is
    # not a mapping is unmeasured quality, which is what `inf` below means.
    if not isinstance(q, dict):
        q = {}
    # A quality tuple carrying None (board_score.quality returns {'error': ...}
    # when the board will not parse) makes min() raise TypeError the moment two
    # rows tie on `blocking`. Untested until now because the self-tests use a
    # uniform empty quality. Sort unknowns LAST rather than crashing. A boolean
    # or a NaN is not a measurement either (#1075): NaN never compares, so it
    # would scramble every tie it touched. `isfinite` on floats only: it
    # converts an int to float, and a 400-digit one raised OverflowError.
    quality = tuple(v if isinstance(v, (int, float))
                    and not isinstance(v, bool)
                    and (isinstance(v, int) or math.isfinite(v))
                    else float('inf')
                    for v in (q.get('vias'), q.get('copper_mm'),
                              q.get('segments')))
    joins = score.get('joins_needed')
    if not (isinstance(joins, int) and not isinstance(joins, bool)
            and joins >= 0):
        joins = 0 if b == 0 else float('inf')
    if b is None:
        # NOT `inf`. A row whose score never measured `blocking` used to rank
        # as the worst possible board, and `inf >= inf` then made the plateau
        # test TRUE -- so an unmeasured lap read as a plateaued one, which is
        # the opposite of what it is. Returning None drops it from the window
        # entirely (the callers already filter None), so a half is judged on
        # laps that actually measured something.
        return None
    return (b, (joins,) + quality)


def _declaration(rows, half):
    """(reason, live, board_sha) for the last `--exhausted <half>` row, else None.

    `live` is False when a lap of that half was recorded AFTER the declaration:
    the half went back to work, so the claim "there is nothing further" is
    stale. Superseding it needs no flag and no deletion -- running the half
    again is the retraction.

    `board_sha` is the declaration row's OWN `result_sha` -- the board the claim
    was made about (#963). It is not a new field and deliberately not one:
    `cmd_record` already writes `result_sha` on every row from
    `store.put(a.board)`, `--board` is required, and `BoardStore.put` COPIES the
    file, so the binding is on disk on every ledger ever written -- including
    run 29's row 29, whose board `frozen.kicad_pcb` was overwritten three
    minutes later. A second key for one fact would be absent on every historical row,
    would need this same fallback anyway, and both paths would then live
    forever; two numbers for one fact is #941's defect in miniature.

    The sha is captured on the row that RE-ARMS the walk, not on an earlier
    declaration's: a self-declaring row restarts `found`, and the claim being
    judged is the last one made.
    """
    found, live, sha = None, False, None
    for r in rows:
        dec = r.get('exhausted')
        if isinstance(dec, dict) and dec.get('half') == half:
            found, live = (str(dec.get('reason') or '').strip()
                           or 'no reason recorded'), True
            sha = r.get('result_sha')
        elif found is not None and _is_lap(r, half):
            # _is_lap, not a bare _HALF match, and this is the sharper half of
            # the fix: "the half went back to work" must mean a LAP was run.
            # A freeze row and a close-out row are both `kind placement` /
            # `kind completion` and neither turns the loop, so either one
            # silently retracted a declaration a person had written down.
            live = False
    return None if found is None else (found, live, sha)


def _classification_rejected(rows):
    """How many `kind: classification` rows were recorded `--rejected`.

    `_classification_state` counts ACCEPTED rows only, and rightly -- a
    discarded decision is not one the next lap can act on. But the refusal
    it feeds then said "no classification row was ever recorded" to a
    ledger holding three of them, which is a false sentence about the
    reader's own file and the fastest way to lose their trust in the gate.
    Published beside it so the text can say which of the two it is.
    """
    return sum(1 for r in rows
               if isinstance(r, dict)
               and (r.get('kind') or '') == 'classification'
               and not r.get('accepted'))


def _classification_state(rows):
    """The last L3 DECISION on the record, and what has happened since (#963).

    `None` when the ledger holds no `kind: classification` row AT ALL, which is
    not the same as "no laps since one" and is the difference that decides
    whether this catches anything. Run 29 recorded ZERO classification rows
    across 439 commands, so a predicate phrased only as "laps
    since the last classification" is vacuously satisfied on exactly the run
    it was written for. A caller must
    handle `None` explicitly; `verdict` publishes each half's total `laps`
    beside this, which is the count that applies then.

    `laps_since` is published for BOTH halves, and a gate on it belongs to
    ROUTING only. A placement lap
    recorded after `shape=placement` is the classification being ACTED ON,
    not invalidated: gating on it would refuse the loop for doing what the
    decision said. That sentence is the whole reason both numbers are here
    rather than one, and deleting it is how the next reader adds the wrong
    conjunct.

    Counted with `_is_lap`, so a `--final` row, a declaration, a freeze and a
    `systemic` or `classification` row are none of them laps -- one predicate,
    the same one `_declaration` and `_half_state` use.
    """
    found, idx = None, -1
    for i, r in enumerate(rows):
        # A LEDGER LINE NEED NOT BE AN OBJECT. `_is_lap` was hardened for
        # exactly this and is unreachable from here: the walk below runs
        # first, so a bare string or list on any line tracebacked out of
        # `record --final --stop-condition 4` before the laps were counted.
        if not isinstance(r, dict):
            continue
        # ACCEPTED ONLY. `--rejected` on a classification says the decision
        # was thrown away, and a discarded decision is not one the next lap
        # can act on -- it satisfied the gate before this line existed.
        if (r.get('kind') or '') == 'classification' and r.get('accepted'):
            found, idx = r, i
    if found is None:
        return None
    after = rows[idx + 1:]
    return {'iteration': found.get('iteration'),
            'shape': found.get('shape'),
            'result_sha': found.get('result_sha'),
            'lever': found.get('lever'),
            'laps_since': {h: sum(1 for r in after if _is_lap(r, h))
                           for h in ('placement', 'routing')}}


def _half_state(rows, half, flat, board_sha=None):
    """Can this half still improve? -- with the evidence it was decided from.

    Returns a dict: flat, laps, accepted, rejected, why, and (when they apply)
    declared / declared_board / declared_stale_board / declared_superseded /
    incommensurable / compared. `why` is one of
    declared-exhausted, too-few-laps, no-comparison, plateau, improving.

    `board_sha` is the board being JUDGED, and it is keyword-with-a-default
    because this function is called positionally at some thirty-five sites
    across the test suite. It only ever adds `declared_stale_board`: `flat` and
    `why` do not change, on purpose (#963). "Any digest change invalidates a
    declaration" sounds safe and is inert-making -- every accepted lap of
    EITHER half writes a new sha, so a placement exhaustion would die on the
    next routing lap although routing copper says nothing about placement's
    remaining levers; and the L2 freeze row changes the sha BY CONSTRUCTION
    while `test_904_not_a_lap` pins that a freeze must not retract. A sha
    cannot tell "rewrote the file, same poses" from "replaced the placement".
    So this REPORTS, and nothing gates on it. A first cut refused the ship
    behind a stale declaration, and a verifier measured that refusal firing on
    the chain's own prescribed ordering -- including the L2 freeze, which
    changes the sha by construction and which `test_904_not_a_lap` pins as
    non-retracting. What refuses is `record --exhausted` given a score that
    grades another board, and a later lap of the half, which retracts with no
    flag at all.

    THE COUNTER COUNTS REJECTED LAPS TOO, and that is the fix for a gate that
    could not be satisfied. It used to count accepted rows only, while every
    lap is recorded, accepted or rejected ("a rejected lap is data") -- so
    recording a rejected lap could not satisfy the gate it was offered for.
    Worse, routing accepts a lap only on STRICT improvement, so a half that is
    genuinely exhausted has no honest accepted lap left to produce: the gate
    demanded the one thing a finished half cannot make. A recorded rejection
    is exactly the evidence that a half tried and did not improve, which is
    what a plateau IS.

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
        # LAP SUPERSESSION FIRST, ALWAYS -- and it already has, because this
        # branch is the live one. The ledger test is total and always
        # answerable; the board test depends on a sha the caller may not have
        # supplied, so reporting both about one declaration would be two
        # findings for one fact.
        if dec[2]:
            out['declared_board'] = dec[2]
            if board_sha and board_sha != dec[2]:
                out['declared_stale_board'] = dec[2]
        else:
            # A hand-built or pre-#963 row can carry no `result_sha`. FAILING
            # OPEN is right -- nothing can be judged -- but failing open in
            # silence is not, because a reader sees the same absent key as a
            # declaration that matched.
            out['declared_board_unknown'] = True
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
    runs = [[k for k, _s in r] for r in runs if len(r) >= 2]
    # An ACCEPTED lap that recorded no `blocking` -- or one that is not a count
    # (`blocking_value`, #1071) -- is unjudged, and a plateau
    # asserted over unjudged laps is the "reported clean because unexamined"
    # error this toolchain names everywhere else. An improvement, by contrast,
    # is a DEFINITE finding and stands whatever else is in the window. So:
    # improvement wins; a plateau requires every accepted lap to have been
    # judged; anything else is not answerable and says so. A REJECTION is not
    # unjudged -- the rejection is itself the measurement.
    unjudged_its = [i for i, acc, k, _s in window if acc and k is None]
    unjudged = len(unjudged_its)
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
        # WHICH rows could not be judged, by the ledger's own word for them.
        # "2 accepted laps recorded no blocking" tells a reader that there is a
        # problem and not where it is, and both remedies -- re-score those laps,
        # or record them as the systemic rows they always were -- need the
        # numbers. `iterations` because that is what a ledger row is numbered
        # by.
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
    score, err, given = None, None, False
    if a.score:
        given = True
        try:
            with open(a.score, encoding='utf-8') as fh:
                score = json.load(fh)
        except Exception as exc:                            # noqa: BLE001
            err = f'{type(exc).__name__}: {exc}'

    def _names(k):
        """`score[k]` as a sorted list of names, whatever the document holds.

        A malformed score is the whole subject of this branch, so reading THIS
        KEY must not be the thing that raises: `sorted(score.get(k) or [])` is
        a TypeError on `{"ungraded": 5}` -- the same crash class this guard
        exists to remove, which is why it is not spelled that way here. It says
        nothing about the rest of the document; `_score_key` guards `quality`
        separately, for the same reason and after the same kind of crash.
        """
        v = score.get(k) if isinstance(score, dict) else None
        if isinstance(v, (list, tuple, set)):
            return sorted(str(x) for x in v)
        return [] if v is None else [f'<not a list: {v!r}>']

    def _no_score(reason):
        # ONE shape for every NO-SCORE document, so a consumer reading
        # `doc['ungraded']` does not KeyError on half of them.
        print(json.dumps({'verdict': 'NO-SCORE', 'reason': reason,
                          'ungraded': _names('ungraded'),
                          'unknown': _names('unknown')},
                         indent=1, sort_keys=True))
        return 2

    if score is None:
        return _no_score(
            err or
            ('the file named by --score holds the JSON document `null`, which '
             'names no board and no measurement.' if given else
             '--score is required: the verdict is about a board, and without '
             'its score there is nothing to be blocked or done ABOUT.'))

    scored = [r for r in rows if _score_key(r.get('score')) is not None]
    key = _score_key(score)
    if key is None:
        # The `--score` caller has to filter None exactly like the ledger-row
        # caller on the line above -- and did not, so `key[0]` raised
        # `TypeError: 'NoneType' object is not subscriptable`.
        #
        # A null-GUARD alone would be worse than the crash: on a plateaued
        # ledger execution falls past `elif blocking == 0` into the terminal
        # branch and prints `STUCK: blocking == None and neither half
        # improved`, which reads as a measurement of a board nothing measured.
        # NO-SCORE says what actually happened, and a reader routes that
        # verdict back to re-scoring instead of the ship ceremony.
        #
        # _score_key returns None for THREE distinct documents and they are
        # not the same fact, so none of them borrows another's sentence -- and
        # the "a component could not answer" cause is only asserted when the
        # score itself names one in `unknown`. Reporting an unmeasured cause
        # is the defect this whole issue is about.
        if not isinstance(score, dict):
            why = (f'the score document is a {type(score).__name__}, not an '
                   f'object: a verdict is about a board, and this names none.')
        elif 'blocking' not in score:
            why = ('the score document has no `blocking` key at all, so there '
                   'is nothing to be blocked or done ABOUT. If this came from '
                   'board_score, it did not finish.')
        elif blocking_defect(score.get('blocking')):
            # #1075. Not null, so neither null sentence below is true of it.
            # Before this branch such a score fell through to the terminal
            # verdicts: `false == 0` read DONE-EXHAUSTED and `Infinity`
            # printed "STUCK: blocking == None".
            why = ('`blocking` is ' + blocking_defect(score.get('blocking'))
                   + '. A verdict ranks boards on `blocking`, so it must be a '
                   'non-negative number (board_score writes an integer, or '
                   'null when it could not measure); this is not a measurement '
                   'of any board. Fix whatever wrote the score, re-score, then '
                   'ask for a verdict.')
        elif isinstance(score.get('unknown'), (list, tuple, set)) \
                and _names('unknown'):
            # The LIST test, not just truthiness: `{"unknown": "impedance"}`
            # renders as `<not a list: 'impedance'>`, and asserting "a component
            # RAN and could not answer" over that is claiming a cause the score
            # never named -- which is the defect this whole issue is about.
            why = ('`blocking` is null because a component RAN and could not '
                   'answer (' + ', '.join(_names('unknown')) + '). Fix the '
                   'instrument and re-score, then ask for a verdict.')
        else:
            why = ('`blocking` is null -- nothing measured it, and null is '
                   'not zero. Re-score, then ask for a verdict.')
        return _no_score(why)
    blocking = key[0]
    # THE BOARD BEING CLOSED, for the exhaustion binding (#963). The score's
    # own `board_sha` is primary -- board_score writes it, and L5 has already
    # proven it equals `--board` before it shells out to here, so reading it
    # costs nothing. `--board` is the second channel, for a pre-B4 or
    # hand-built score that carries no sha. Absent on BOTH is unanswerable and
    # invalidates nothing: every hand-built score in tests/test_converge.py
    # lacks the key, and a question that cannot be answered must not decide.
    board_sha = score.get('board_sha') if isinstance(score, dict) else None
    board_sha_source = 'score' if board_sha else None
    if getattr(a, 'board', None):
        _bind, _psha = score_board_binding(
            a.board, score if isinstance(score, dict) else None)
        if _bind == 'other':
            print(f"verdict: --board and --score disagree about which board "
                  f"is being judged (score board_sha {str(_psha)[:12]}... is "
                  f"not {os.path.abspath(a.board)}). A stop verdict read off "
                  f"one board's ledger and another board's numbers is the "
                  f"defect this binding exists to catch. Re-score the board "
                  f"you are closing out. Nothing was judged.", file=sys.stderr)
            return 2
        if not board_sha:
            try:
                from board_store import sha256_file
                board_sha, board_sha_source = sha256_file(a.board), '--board'
            except Exception as exc:                        # noqa: BLE001
                # NEVER SILENT. `--board` pointing at a directory or a path
                # that is not there used to leave `board_sha_source: null` and
                # say nothing, so a caller who passed the flag precisely to get
                # the binding checked was told nothing had been checked only by
                # reading a null they had no reason to look at.
                print(f"verdict NOTE: --board {os.path.abspath(a.board)} "
                      f"could not be hashed ({type(exc).__name__}), so the "
                      f"exhaustion binding was NOT checked from it. The score "
                      f"carries no board_sha either.", file=sys.stderr)
    st = {h: _half_state(rows, h, a.flat, board_sha=board_sha)
          for h in ('placement', 'routing')}
    flat_p, flat_r = st['placement']['flat'], st['routing']['flat']
    why_p, why_r = st['placement']['why'], st['routing']['why']

    doc = {'ledger_rows': len(rows), 'scored_rows': len(scored),
           'budget': a.budget, 'flat': a.flat,
           # A finite count by construction (`blocking_value`), so there is no
           # inf to map: the old `None if inf` printed "STUCK: blocking ==
           # None" for an `Infinity` score (#1075).
           'blocking': blocking,
           'quality': score.get('quality'),
           # `_names`, not `sorted(score.get(k) or [])`: that raised on
           # `{"ungraded": 5}` and on a mixed list, next to a REAL blocking,
           # where the NO-SCORE branch's guard never ran (#1076).
           'ungraded': _names('ungraded'),
           'unknown': _names('unknown'),
           # The last L3 decision on the record, published on EVERY verdict
           # rather than only on the branch that reads it -- the posture the
           # incommensurable block already takes. `null` means no
           # classification row exists at all, which is run 29's case and the
           # one a reader most needs told.
           'classification': _classification_state(rows),
           # WHICH INPUT the board binding came from, published beside it: an
           # aggregate verdict cannot say which of its inputs moved (#694), and
           # `null` here is the honest answer for a score with no board_sha and
           # no --board -- not a passing one.
           'board_sha': board_sha, 'board_sha_source': board_sha_source,
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
                        f'window recorded no `blocking` a verdict can rank '
                        f'(null, absent, or not a non-negative number) -- '
                        f'iteration(s) '
                        + (', '.join(str(i) for i in
                                     (st[h].get('unjudged_iterations') or []))
                           or 'not numbered')
                        + ' -- and a lap that '
                        'measured nothing is evidence in neither direction'),
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
                      # consumer that renders a lap -- the GUI's stage label,
                      # the watcher's rejected-lap line --
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

    # A DECLARATION MADE ABOUT ANOTHER BOARD, named on every verdict (#963).
    # Run 29 declared placement exhausted against a board that existed for
    # eight minutes -- `os-promote-placed` created it at 12:51:40,
    # `ol-exh-place` declared against it at 12:56:04, and
    # `ov-restore-frozen` overwrote `frozen.kicad_pcb` at 12:59:45, three
    # minutes and 41 seconds after the row was written -- and it outlived its
    # board and survived three L5 calls, because nothing here read the sha the
    # row had carried all along. This REPORTS, and nothing gates on it:
    # `test_963_exhaustion_binding::test_the_ship_is_not_gated_on_the_board_
    # having_moved` pins that, because a first cut did gate and refused the
    # chain's own prescribed ordering.
    for h in ('placement', 'routing'):
        if st[h].get('declared_board_unknown'):
            # A READER for the key, without which it was one write site and
            # nothing else: a hand-built or pre-#963 declaration carries no
            # `result_sha`, so "no stale-board finding" there means "nobody
            # could look", which is not the same as "it matched".
            doc['reason'] += (
                f' The live {h} exhaustion carries NO board of its own '
                f'(a row written before the binding existed, or by hand), so '
                f'whether it still applies to this board could not be checked.')
        _sb = st[h].get('declared_stale_board')
        if _sb:
            doc['reason'] += (
                f' DECLARED ABOUT ANOTHER BOARD: the live {h} exhaustion was '
                f'recorded against board {str(_sb)[:12]}..., and the board '
                f'being judged is {str(board_sha)[:12]}... (from '
                f'{board_sha_source}). EXPECTED after any accepted lap -- '
                f'every lap of either half writes a new sha, and a routing '
                f'lap says nothing about placement levers -- so this is not '
                f'a finding against the run. What it says is that the claim '
                f'has not been re-checked against the board in hand. The '
                f'board it was made about is in this ledger\'s store, at '
                f'boards/{_sb}/board.kicad_pcb beside the ledger: compare '
                f'the two and see what changed.')

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
    if doc['ungraded'] and not isinstance(score.get('ungraded'),
                                          (list, tuple, set)):
        # Like `unknown` below: `"abc"` names no component, so it must not be
        # read as three components nobody examined (#1076).
        doc['reason'] += (' The score\'s `ungraded` is not a list ('
                          + ', '.join(doc['ungraded'])
                          + '), so it names no component -- re-score before '
                            'trusting any verdict here.')
    elif doc['ungraded']:
        # Not fatal: a board with no spec files has nothing to grade those
        # components against, and making it fatal would put every corpus board
        # permanently in STUCK. But it is never silent -- a component nothing
        # examined is UNEXAMINED, and DONE must say so out loud.
        doc['reason'] += (' UNEXAMINED, and not passed: '
                          + ', '.join(doc['ungraded']) + '.')
    if doc['unknown'] and isinstance(score.get('unknown'),
                                     (list, tuple, set)):
        doc['reason'] += (' A component RAN and could not answer: '
                          + ', '.join(doc['unknown'])
                          + ' -- fix the instrument before trusting any '
                            'verdict here.')
    elif doc['unknown']:
        # The NO-SCORE branch's rule: "a component RAN" is asserted only over
        # a LIST the score named (#1076) -- `false` or `"impedance"` names no
        # component, it is a malformed score.
        doc['reason'] += (' The score\'s `unknown` is not a list ('
                          + ', '.join(doc['unknown'])
                          + '), so it names no component -- re-score before '
                            'trusting any verdict here.')
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


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='verb', required=True)

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
    r.add_argument('--score-file', default=None, metavar='PATH',
                   help="the board's score payload (board_score --json), as "
                        'a FILE: a payload passed as argv exceeds the OS '
                        'limit at ~32kB and the shell then exits 126 BEFORE '
                        'record runs, so the lap is lost with no error.')
    r.add_argument('--shape', choices=SHAPES,
                   default=None,
                   help='the re-entry shape this lap acted on (the same word '
                        'L4 demands). Stored as entry["shape"].')
    r.add_argument('--lens-file', action='append', default=None,
                   metavar='PATH',
                   help='a verifier lens verdict, read from the file the '
                        'verifier wrote it to: the FIRST line beginning '
                        'VERDICT= is taken ("VERDICT=PASS:lens=connectivity" '
                        'or "VERDICT=FAIL:lens=drc;finding=...;evidence=..."), '
                        'stored raw in entry["lenses"], and the row records '
                        'the path and the file\'s sha256 in '
                        'entry["lens_source"]. Repeatable, one per lens. '
                        '--final --kind completion REQUIRES it for '
                        'connectivity, drc and spec.')
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
    r.add_argument('--final', action='store_true',
                   help='mark the run-closing record; requires --stop-condition')
    r.add_argument('--stop-condition', default=None,
                   help='which stop condition ended the run (with --final). '
                        'A TOKEN -- ' + ' | '.join(STOP_TOKENS) + ' -- checked '
                        'on every record, not only when a lens failed. Prose '
                        'about WHY may follow it ("3: five laps, no new '
                        'copper"); it is split off into stop_reason.')
    r.add_argument('--parent', default=None, metavar='BOARD_OR_SHA',
                   help='the board this lap was MADE FROM: a board path, or '
                        'a recorded result_sha (a unique prefix will do). '
                        'Stored as entry["parent_sha"], with '
                        'entry["parent_source"]="parent". Omitted: the last '
                        'ACCEPTED row, with a NOTE (source "last_accepted"). '
                        'Parallel lineages MUST pass it (#1034). A value that '
                        'is neither is refused (exit 2), nothing written.')
    r.set_defaults(fn=cmd_record)

    v = sub.add_parser('verdict',
                       help='continue, or stop -- and which kind of stop')
    v.add_argument('--ledger', required=True)
    v.add_argument('--score', required=True,
                   help="the current board's score JSON (board_score --json)")
    v.add_argument('--board',
                   help='the board being closed out. Optional: the score '
                        "already carries board_sha and that is read first. "
                        'Pass it when the score is pre-B4 or hand-built, so an '
                        'exhaustion declared about a board that no longer '
                        'exists can still be named. Given BOTH and they '
                        'disagree, nothing is judged (exit 2).')
    v.add_argument('--budget', type=int, default=100,
                   help='ledger entries this run may write (default 100)')
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
    # `record` and `verdict` print documents that callers json.loads()
    # whole (tests/test_converge.py does). The other instruments' stdout is
    # a log.
    sys.exit(main())
