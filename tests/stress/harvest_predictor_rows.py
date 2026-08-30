#!/usr/bin/env python3
"""Join every recorded run's PRE-route document to its POST-route score (#703).

The question #703 asks is whether the numbers this repo prints BEFORE routing
rank the `blocking` outcome it measures AFTER it. Nobody had ever joined the two
documents, though both have been written side by side in every recorded run for
months: `handoff.json` (`render_placement --json-out` on the frozen, placed,
copper-free board) and a `board_score` payload on the routed board.

This tool does the join, and its most important output is the REFUSAL LIST.

WHY REFUSING IS THE DELIVERABLE

A hand join of these documents was published in the #703 issue body: six rows,
rho(crossings, blocking) = +0.339. That join picked the score file NAMED
`score.json`, and a name is not a verdict. `wk/run14/castor` holds four score
files across two boards, and `score_final.json` (blocking 0) and
`score_final_declared.json` (blocking 62 -- drc 42, undersized 20) grade the SAME
final board at two different floors. Castor is the row whose removal moves the
published headline from +0.339 to +0.632, so the most influential sample in that
table is a board whose truth is 0 or 62 depending on which grade you reach for.
It is REFUSED here.

THE LEDGER IS THE AUTHORITY, AND THE FIRST VERSION OF THIS FILE GOT IT WRONG

That first version used `RUN_STATE.json`'s `board_sha`, reasoning that
`converge.write_run_state` is the one thing in a run that asserts where it
ended. It is not: that field is copied from the LAST ledger row's `result_sha`
whether or not the lap was ACCEPTED, while the sibling `final_recorded` flag is
a property of a different row. On `wk/run23/tigard` the file contradicts itself
-- `board_sha` names the output of row 42 (`accepted: false`, lever text ending
"Step back to routed.kicad_pcb") while its own `quality.vias` is 377, which is
the ACCEPTED board's value, and `score_lap` is 39.

So the rule reads the ledger: the terminal board is the LAST row carrying
`final: true`, and the terminal score is the single score file grading it.
RUN_STATE stays as a CROSS-CHECK and any disagreement is written onto the row,
because that disagreement is informative. An adversarial review found this, and
it changed three of five rows and un-refused two more:

  * tigard  -- terminal is `score.json` (b52f79c9, vias 377), NOT `score_r18`
    (5eff4294, vias 372). **The published 377 was right and my first
    correction of it was wrong.**
  * urchin  -- terminal is `score_c2.json`: blocking **0**, vias 177. The
    published 3 / 175 came from an earlier final board that happens to be the
    file named `routed.kicad_pcb`. The headline truth variable flips.
  * neo6502 -- terminal is `score2.json`: blocking **78**, vias 408, not
    79 / 427.
  * run10 and run11 smartknob each have exactly one score grading their final
    board and were previously refused for want of an authority that was in the
    ledger the whole time.

LINEAGE IS A REACHABILITY SEARCH, NOT A WALK

`handoff.json`'s `instrument` block carries a PATH and never a sha, so the
pre-route board is bound to the post-route score over `ledger.jsonl`'s
`parent_sha -> result_sha` edges. It has to be a BFS with a seen-set: an
accepted-only greedy walk on run23 returns to the frozen sha after 18 steps.
`tests/test_703_predictor_harvest.py` pins that shape, and a cyclic ledger that
must terminate.

AN ABSENT KEY IS null, NEVER 0

Ten of the twelve recorded handoff documents carry no courtyard keys at all --
they predate the run-23 rename. Coercing absent to 0 would record "zero
courtyard overlap" as a measurement on ten boards. Absent is `null` plus an
entry in the row's `schema_gaps`; a key found under its legacy name is
normalised and recorded in `schema_aliases_used`.

    python3 -X utf8 tests/stress/harvest_predictor_rows.py
    python3 -X utf8 tests/stress/harvest_predictor_rows.py \
        --runs-dir wk --out wk/703/harvest_rows.json
    python3 -X utf8 tests/stress/harvest_predictor_rows.py --verify

Nothing here routes anything, and nothing here computes a correlation: these
rows are ONE PLACEMENT PER BOARD, which is the pooled form #703 exists to
retire. `predictor_study.py` is what ranks within a board.
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

import rank_stats                                              # noqa: E402

SCHEMA = 1

#: The 19 keys `render_placement`'s `metrics` block has carried unchanged across
#: every recorded document (verified: 19/19 present on all 12). Pinned so a new
#: key is REPORTED rather than silently ignored, and a dropped one is reported
#: rather than silently becoming a hole.
METRIC_KEYS = (
    'align', 'corridor_cut', 'crossings', 'edge', 'halo', 'hole_shortfall',
    'hpwl', 'length', 'locked_contact_pairs', 'oob_amount', 'oob_area',
    'oob_count', 'orient', 'overlap_area', 'pad_conflict_pairs',
    'pad_intersection_pairs', 'pad_overlap_pairs', 'pad_shortfall', 'total',
)

#: Checklist-derived predictors: canonical name -> (path, legacy aliases).
#: `a_off_outline.pad_copper` leads because it is the ONLY pre-route number in
#: this repo that refuses anything (loop_driver's L2 gate).
CHECKLIST_COUNTS = (
    ('pad_copper', ('a_off_outline', 'pad_copper'), ()),
    ('courtyard_off_outline', ('a_off_outline', 'courtyard'), ()),
    ('body_overlap_pairs', ('b_body_overlap_pairs',), ()),
    ('pad_clearance_pairs', ('b_pad_clearance_pairs',), ()),
    # run-23 split one key into an advisory and a blocking half. Ten of twelve
    # recorded docs predate it and carry only the old name.
    ('courtyard_advisory_pairs', ('b_courtyard_advisory_pairs',),
     ('b_courtyard_overlap_pairs',)),
    ('courtyard_blocking_pairs', ('b_courtyard_blocking_pairs',), ()),
    ('cross_side_stacks', ('b_cross_side_stacks',), ()),
    ('hole_conflicts', ('c_hole_conflicts',), ()),
)

CHECKLIST_SCALARS = (
    ('courtyard_overlap_mm2', ('b_courtyard_overlap_mm2',), ()),
)

PREDICTOR_KEYS = tuple(METRIC_KEYS
                       + tuple(n for n, _p, _a in CHECKLIST_COUNTS)
                       + tuple(n for n, _p, _a in CHECKLIST_SCALARS))

TRUTH_BY_KEYS = ('unrouted', 'broken', 'drc', 'undersized', 'floorplan',
                 'assembly', 'impedance', 'length', 'net_widths')


class Refusal(Exception):
    """A run this tool will not turn into a row, with the reason as data."""

    def __init__(self, code, detail):
        super().__init__(f'{code}: {detail}')
        self.code = code
        self.detail = detail


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------

def sha256_file(path):
    """Streamed, so a 2 MB board costs nothing. Same digest board_score uses."""
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _rel(path, root):
    return os.path.relpath(path, root).replace('\\', '/')


def git_describe(root):
    try:
        p = subprocess.run(['git', 'describe', '--always', '--dirty'],
                           capture_output=True, text=True, cwd=root,
                           timeout=30)
        return (p.stdout or '').strip() or None
    except Exception:                                           # noqa: BLE001
        return None


# ---------------------------------------------------------------------------
# discovery
# ---------------------------------------------------------------------------

def discover_runs(runs_dir, max_depth=3):
    """Directories under `runs_dir` holding a `handoff.json`.

    Recursive and depth-bounded rather than the `wk/run*/<board>/` glob the
    hand join used: `wk/run20/handoff.json` sits ONE level down, not two, and
    that glob missed it entirely.
    """
    out = []
    base = os.path.abspath(runs_dir)
    for p in glob.glob(os.path.join(base, '**', 'handoff.json'),
                       recursive=True):
        d = os.path.dirname(os.path.abspath(p))
        depth = len(os.path.relpath(d, base).replace('\\', '/').split('/'))
        if os.path.relpath(d, base) == '.':
            depth = 0
        if depth <= max_depth and d not in out:
            out.append(d)
    return sorted(out)


# ---------------------------------------------------------------------------
# the terminal score
# ---------------------------------------------------------------------------

def score_files(run_dir):
    return sorted(glob.glob(os.path.join(run_dir, 'score*.json')))


def terminal_from_ledger(run_dir):
    """(sha, row_index, accepted) of the run's LAST `final: true` ledger row.

    THE LEDGER IS THE AUTHORITY, and the first version of this file got that
    wrong in a way worth writing down.

    It used `RUN_STATE.json`'s `board_sha`, on the reasoning that
    `converge.write_run_state` is the one thing in a run that asserts "this is
    where it ended". But that field is set from the LAST ledger row's
    `result_sha` whether or not the lap was ACCEPTED, while the sibling
    `final_recorded` flag is a property of a different row entirely. On
    `wk/run23/tigard` the two disagree and the file contradicts itself:

        board_sha      5eff4294   <- row 42, accepted: False, whose own lever
                                     text ends "Step back to routed.kicad_pcb"
        score_lap      39         <- the accepted final lap
        quality.vias   377        <- b52f79c9's vias, not 5eff4294's 372
        last_accepted  False

    Reading `board_sha` there selects the output of a REJECTED lap. The ledger
    has no such ambiguity: row 39 is the run's only `final: true` row, it is
    accepted, and it names b52f79c9 -- which is `routed.kicad_pcb`, which is
    what `score.json` grades, and it is what `RUN_STATE`'s own quality block
    agrees with.

    `RUN_STATE` is kept as a CROSS-CHECK and disclosed when it disagrees,
    because that disagreement is informative. It is not the authority.
    """
    led = os.path.join(run_dir, 'ledger.jsonl')
    if not os.path.isfile(led):
        raise Refusal(
            'no_ledger',
            'the run has no ledger.jsonl, so nothing in it says which board it '
            'ended on. A filename convention is not a verdict.')
    rows = []
    with open(led, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except Exception:                               # noqa: BLE001
                    continue
    finals = [(i, r) for i, r in enumerate(rows) if r.get('final')]
    if not finals:
        raise Refusal(
            'run_never_closed',
            f'ledger.jsonl has {len(rows)} row(s) and none carries '
            f'`final: true`. The run did not record a final board, so it has '
            f'no terminal score.')
    i, row = finals[-1]
    sha = row.get('result_sha')
    if not sha:
        raise Refusal('final_row_has_no_sha',
                      f'ledger row {i} is final but carries no result_sha')
    return sha, i, row.get('accepted')


def terminal_score(run_dir):
    """(score_path, rule, terminal_sha, notes) or raise Refusal.

    The terminal board comes from the ledger; the terminal SCORE is the single
    score file that grades it. Never from a filename: two runs on this corpus
    are decided wrongly by the name `score.json` and two more are refused by a
    harvester that cannot read the ledger.
    """
    sha, idx, accepted = terminal_from_ledger(run_dir)
    notes = []
    if accepted is False:
        notes.append(f'ledger row {idx} is final but records accepted=False; '
                     f'the run ended on a lap it did not accept')

    scored = []
    for p in score_files(run_dir):
        try:
            s = _load(p)
        except Exception as e:                                  # noqa: BLE001
            scored.append((p, None, f'unreadable: {e}'))
            continue
        scored.append((p, s.get('board_sha'), None))
    inventory = ', '.join(
        f'{os.path.basename(p)}={str(bs)[:10] if bs else "no board_sha"}'
        for p, bs, _e in scored) or '(none)'

    hits = [p for p, bs, _e in scored if bs == sha]
    if not hits:
        raise Refusal(
            'no_score_for_terminal_board',
            f'ledger row {idx} names the final board {str(sha)[:10]} and no '
            f'score file grades it. Scanned: {inventory}')
    if len(hits) > 1:
        verdicts = []
        for p in hits:
            s = _load(p)
            verdicts.append(f'{os.path.basename(p)} '
                            f'blocking={s.get("blocking")!r} '
                            f'label={(s.get("label") or "")!r}')
        raise Refusal(
            'ambiguous_terminal_score',
            f'{len(hits)} score files grade the final board {str(sha)[:10]}, '
            f'and they disagree about what it is worth: {"; ".join(verdicts)}. '
            f'Picking one would be choosing the answer.')

    # RUN_STATE as a CROSS-CHECK, never as the authority.
    rs_path = os.path.join(run_dir, 'RUN_STATE.json')
    if os.path.isfile(rs_path):
        try:
            rst = _load(rs_path)
        except Exception:                                       # noqa: BLE001
            rst = {}
        rs_sha = rst.get('board_sha')
        if rs_sha and rs_sha != sha:
            notes.append(
                f'RUN_STATE.board_sha is {str(rs_sha)[:10]} but the ledger\'s '
                f'last final row (#{idx}) names {str(sha)[:10]}. '
                f'write_run_state copies the LAST row\'s result_sha whether or '
                f'not it was accepted; the ledger is the authority here')
    return hits[0], 'ledger_final', sha, notes


# ---------------------------------------------------------------------------
# lineage
# ---------------------------------------------------------------------------

def ledger_edges(path):
    """`parent_sha -> {result_sha}` over a ledger.jsonl, malformed rows skipped."""
    edges = {}
    if not os.path.isfile(path):
        return edges
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except Exception:                                   # noqa: BLE001
                continue
            a, b = row.get('parent_sha'), row.get('result_sha')
            if a and b:
                edges.setdefault(a, set()).add(b)
    return edges


def reachable(edges, src, dst):
    """(found, hops, seen_count) -- BFS with a seen set, never a greedy walk.

    Measured on the recorded corpus: following "the last accepted row" from the
    frozen board on run23 terminates back at the frozen sha itself and reports
    the lineage unproven. The BFS finds the terminal board 19 hops out. A
    ledger with a cycle must terminate, which is what the seen set is for.
    """
    if src == dst:
        return True, 0, 1
    seen = {src}
    frontier = [src]
    hops = 0
    while frontier:
        hops += 1
        nxt = []
        for node in frontier:
            for n in edges.get(node, ()):
                if n in seen:
                    continue
                if n == dst:
                    return True, hops, len(seen) + 1
                seen.add(n)
                nxt.append(n)
        frontier = nxt
    return False, hops, len(seen)


# ---------------------------------------------------------------------------
# predictors
# ---------------------------------------------------------------------------

def _dig(doc, path):
    cur = doc
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return None, False
        cur = cur[k]
    return cur, True


def predictors_from_handoff(handoff):
    """(predictors, schema_gaps, aliases_used, metrics_unknown).

    Absent is null plus a gap entry. Never 0: ten of twelve recorded documents
    carry no courtyard keys, and a zero there would be a fabricated
    measurement of "no courtyard overlap" on every one of them.
    """
    pred, gaps, aliases = {}, [], {}
    metrics = handoff.get('metrics') or {}
    for k in METRIC_KEYS:
        if k in metrics:
            pred[k] = metrics[k]
        else:
            pred[k] = None
            gaps.append(f'metrics.{k}')
    unknown = sorted(set(metrics) - set(METRIC_KEYS))

    checklist = handoff.get('checklist') or {}
    for name, path, legacy in CHECKLIST_COUNTS:
        val, found = _dig(checklist, path)
        if not found:
            for alt in legacy:
                val, found = _dig(checklist, (alt,) + tuple(path[1:]))
                if found:
                    aliases[name] = alt
                    break
        if not found:
            pred[name] = None
            gaps.append('checklist.' + '.'.join(path))
        else:
            pred[name] = len(val) if isinstance(val, (list, tuple)) else val
    for name, path, legacy in CHECKLIST_SCALARS:
        val, found = _dig(checklist, path)
        if not found:
            for alt in legacy:
                val, found = _dig(checklist, (alt,))
                if found:
                    aliases[name] = alt
                    break
        if not found:
            pred[name] = None
            gaps.append('checklist.' + '.'.join(path))
        else:
            pred[name] = val
    return pred, gaps, aliases, unknown


# ---------------------------------------------------------------------------
# one run -> one row
# ---------------------------------------------------------------------------

def build_row(run_dir, root):
    handoff_path = os.path.join(run_dir, 'handoff.json')
    handoff = _load(handoff_path)

    score_path, rule, terminal_sha, tier_notes = terminal_score(run_dir)
    score = _load(score_path)

    blocking = score.get('blocking')
    if blocking is None:
        # board_score's own vacuity rule: None means a component that was asked
        # for could not run. Coercing it to 0 is the exact failure that file
        # exists to prevent.
        raise Refusal(
            'vacuous_blocking',
            f'{os.path.basename(score_path)} reports blocking=None (ungraded: '
            f'{score.get("ungraded")}). None is "a component could not run", '
            f'not zero.')

    inst = handoff.get('instrument') or {}
    recorded = inst.get('board')
    if not recorded:
        raise Refusal('no_pre_board',
                      'handoff.json carries no instrument.board')
    # Resolve INSIDE the run dir. The recorded path is absolute and
    # machine-local; on any other checkout it points outside the tree, and on
    # this one it could point at a board that has since been overwritten.
    pre_path = os.path.join(run_dir, os.path.basename(recorded))
    if not os.path.isfile(pre_path):
        raise Refusal(
            'pre_board_gone',
            f'the rendered board {os.path.basename(recorded)} no longer exists '
            f'in {_rel(run_dir, root)}, so the pre-route document cannot be '
            f'bound to a board')
    pre_sha = sha256_file(pre_path)

    edges = ledger_edges(os.path.join(run_dir, 'ledger.jsonl'))
    found, hops, seen = reachable(edges, pre_sha, terminal_sha)
    if not found:
        raise Refusal(
            'lineage_unproven',
            f'{os.path.basename(pre_path)} ({pre_sha[:10]}) does not reach the '
            f'terminal board {terminal_sha[:10]} over ledger.jsonl '
            f'({len(edges)} edge source(s), {seen} sha(s) reachable) -- the '
            f'pre-route document may describe a different board')

    pred, gaps, aliases, unknown = predictors_from_handoff(handoff)
    by = score.get('blocking_by') or {}
    q = score.get('quality') or {}
    notes = list(tier_notes)
    if unknown:
        notes.append(f'handoff metrics carries {len(unknown)} key(s) this tool '
                     f'does not record: {", ".join(unknown)}')

    board_key = os.path.basename(run_dir.rstrip('/\\'))
    # `wk/run20/` names the run, not the board; fall back to the board file.
    if board_key.lower().startswith('run'):
        board_key = os.path.splitext(os.path.basename(
            str(score.get('board') or 'unknown')))[0]

    return {
        'schema': SCHEMA,
        'kind': 'predictor-row',
        'row_id': f'harvest:{board_key}:{_rel(run_dir, root).replace("/", "-")}',
        'source': 'harvest',
        # Harvest rows read the gitignored wk/ tree. They cannot be re-derived
        # on another machine, so they may never carry a verdict.
        'reproducible': False,
        'board_key': board_key,
        'variant': 'recorded',
        'generator': None,
        # These predictors are READ from a recorded document, not re-derived
        # from the written board. Study rows say 'reparse' and the two are
        # never merged into one rho.
        'predictor_source': 'handoff.json',
        'provenance': {
            'run_dir': _rel(run_dir, root),
            'handoff': _rel(handoff_path, root),
            'pre_board': os.path.basename(pre_path),
            'pre_board_sha': pre_sha,
            'terminal_board_sha': terminal_sha,
            'terminal_score_file': os.path.basename(score_path),
            'terminal_rule': rule,
            'score_files_scanned': [os.path.basename(p)
                                    for p in score_files(run_dir)],
            'lineage': 'ledger_reachable',
            'lineage_hops': hops,
            'clearance': inst.get('clearance'),
            'clearance_requested': inst.get('clearance_requested'),
        },
        'route': {'argv': None, 'argv_sha': None, 'returncode': None,
                  'seconds': None},
        'predictors': pred,
        'truth': {
            'headline': blocking,
            'blocking': blocking,
            'blocking_by': {k: by.get(k) for k in TRUTH_BY_KEYS},
            'quality': {'vias': q.get('vias'), 'copper_mm': q.get('copper_mm'),
                        'segments': q.get('segments')},
            'ungraded': score.get('ungraded'),
        },
        'schema_gaps': gaps,
        'schema_aliases_used': aliases,
        'notes': notes,
    }


def harvest(runs_dir, root):
    rows, refusals = [], []
    for run_dir in discover_runs(runs_dir):
        try:
            rows.append(build_row(run_dir, root))
        except Refusal as r:
            refusals.append({'run_dir': _rel(run_dir, root),
                             'reason_code': r.code, 'detail': r.detail})
        except Exception as e:                                  # noqa: BLE001
            refusals.append({'run_dir': _rel(run_dir, root),
                             'reason_code': 'harvester_error',
                             'detail': f'{type(e).__name__}: {e}'})
    rows.sort(key=lambda r: r['row_id'])
    refusals.sort(key=lambda r: r['run_dir'])
    return rows, refusals


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------

def report(rows, refusals):
    print('=' * 78)
    print('HARVEST -- one recorded run, one row. THIS IS THE POOLED FORM.')
    print('=' * 78)
    if rows:
        print(f'{"board":16s} {"tier":22s} {"score file":22s} {"cross":>6} '
              f'{"hpwl":>9} {"blk":>5} {"vias":>6}')
        for r in rows:
            p, t, pr = r['predictors'], r['truth'], r['provenance']
            print(f'{r["board_key"]:16s} {pr["terminal_rule"]:22s} '
                  f'{pr["terminal_score_file"]:22s} '
                  f'{str(p.get("crossings")):>6} '
                  f'{(p.get("hpwl") or 0):>9.1f} '
                  f'{str(t["headline"]):>5} {str(t["quality"]["vias"]):>6}')
    else:
        print('  no run produced a row')

    print(f'\nREFUSED {len(refusals)} run(s) -- this list is the finding, not '
          f'the leftovers:')
    for r in refusals:
        print(f'  {r["run_dir"]:24s} {r["reason_code"]}')
        print(f'  {"":24s}   {r["detail"]}')

    tiers = {}
    for r in rows:
        tiers[r['provenance']['terminal_rule']] = tiers.get(
            r['provenance']['terminal_rule'], 0) + 1
    print(f'\n{len(rows)} row(s) by authority tier: '
          + (', '.join(f'{k}={v}' for k, v in sorted(tiers.items())) or 'none'))

    gaps = {}
    for r in rows:
        for g in r['schema_gaps']:
            gaps[g] = gaps.get(g, 0) + 1
    if gaps:
        print(f'schema gaps (absent -> null, NEVER 0) across {len(rows)} row(s):')
        for g, c in sorted(gaps.items()):
            print(f'  {g:44s} missing on {c}')

    print('\nWHAT THIS TABLE CANNOT SUPPORT')
    print('  One placement per board is the POOLED form: between-board variance')
    print('  dominates, so it measures board size. On this very corpus')
    print('  rho(crossings, vias) is +0.714 pooled and -0.400 WITHIN the one')
    print('  recorded slate of one board -- opposite signs. No correlation is')
    print('  computed here on purpose; predictor_study.py ranks within a board.')


# ---------------------------------------------------------------------------
# verify -- re-derive committed rows where the run still exists
# ---------------------------------------------------------------------------

def verify(committed_path, runs_dir, root):
    """Re-derive each committed row and classify the disagreements.

    The classes are `test_placement_ab.compare_baseline`'s, because the failure
    is the same one: a stored number that has quietly stopped describing what
    produced it. A row whose run is absent HERE prints UNVERIFIABLE and is
    never silently skipped -- a dropped unverifiable row is how a stale dataset
    reads as fresh.
    """
    doc = _load(committed_path)
    old = {r['row_id']: r for r in doc.get('rows') or []}
    old_ref = {r['run_dir'] + '|' + r['reason_code']
               for r in doc.get('refusals') or []}
    rows, refusals = harvest(runs_dir, root)
    new = {r['row_id']: r for r in rows}
    new_ref = {r['run_dir'] + '|' + r['reason_code'] for r in refusals}

    problems = 0
    for rid, o in sorted(old.items()):
        run_dir = os.path.join(root, o['provenance']['run_dir'])
        if not os.path.isdir(run_dir):
            print(f'  UNVERIFIABLE HERE  {rid} -- '
                  f'{o["provenance"]["run_dir"]} is not on this machine')
            continue
        n = new.get(rid)
        if n is None:
            print(f'  MISSING            {rid} -- the run is here and no longer '
                  f'produces a row')
            problems += 1
            continue
        for key in ('predictors', 'truth'):
            for k in sorted(set(o[key]) | set(n[key])):
                a, b = o[key].get(k), n[key].get(k)
                if a == b:
                    continue
                cls = 'DRIFT'
                if isinstance(a, (int, float)) and isinstance(b, (int, float)):
                    if (a > 0) != (b > 0) or (a < 0) != (b < 0):
                        cls = 'INVERTED'
                print(f'  {cls:18s} {rid} {key}.{k}: {a!r} -> {b!r}')
                problems += 1
    for rid in sorted(set(new) - set(old)):
        print(f'  NEW ROW            {rid}')
    for r in sorted(new_ref - old_ref):
        print(f'  NEW REFUSAL        {r}')
    for r in sorted(old_ref - new_ref):
        run = r.split('|')[0]
        if os.path.isdir(os.path.join(root, run)):
            print(f'  ORPHAN REFUSAL     {r} -- the run is here and no longer '
                  f'refuses. The rule was loosened, or the run changed.')
            problems += 1
    if not problems:
        print('  every committed row that can be checked here re-derives '
              'identically')
    return 1 if problems else 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs-dir', default=None,
                    help='tree of recorded runs (default: <root>/wk)')
    ap.add_argument('--root', default=ROOT, help='repo root')
    ap.add_argument('--out', default=None,
                    help='write {schema, rows[], refusals[]} here')
    ap.add_argument('--verify', nargs='?', const='', default=None,
                    metavar='ROWS_JSON',
                    help='re-derive the committed rows and report the '
                         'disagreements (default file: '
                         'wk/703/harvest_rows.json)')
    a = ap.parse_args(argv)

    # The kernel's arms, every invocation. It costs milliseconds and the live
    # path cannot reach any of them.
    rank_stats._self_test()

    runs_dir = a.runs_dir or os.path.join(a.root, 'wk')
    if not os.path.isdir(runs_dir):
        print(f'no such runs dir: {runs_dir}', file=sys.stderr)
        return 3

    if a.verify is not None:
        path = a.verify or os.path.join(a.root, 'wk', '703',
                                        'harvest_rows.json')
        if not os.path.isfile(path):
            print(f'no committed rows at {path}', file=sys.stderr)
            return 3
        print(f'VERIFY {_rel(path, a.root)} against {_rel(runs_dir, a.root)}')
        return verify(path, runs_dir, a.root)

    rows, refusals = harvest(runs_dir, a.root)
    report(rows, refusals)
    if a.out:
        doc = {'schema': SCHEMA, 'kind': 'predictor-rows',
               'generated_by': 'tests/stress/harvest_predictor_rows.py',
               'harvester_git': git_describe(a.root),
               'predictor_keys': list(PREDICTOR_KEYS),
               'rows': rows, 'refusals': refusals}
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
            f.write('\n')
        print(f'\nwrote {len(rows)} row(s) and {len(refusals)} refusal(s) to '
              f'{_rel(a.out, a.root)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
