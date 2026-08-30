#!/usr/bin/env python3
"""#789: does `rank_key`'s ORDER agree with the routed order, and does
`rule1_check`'s crossings clause ever bar a candidate that routed BETTER?

    python3 -X utf8 tests/stress/slate_study.py --out wk/789 -j 4
    python3 -X utf8 tests/stress/slate_study.py --from-rows wk/789/slate_rows.jsonl
    python3 -X utf8 tests/stress/slate_study.py --boards esp_prog --plan

THE CIRCULARITY POSITION, AND IT IS THE OPPOSITE OF #703's.
`predictor_study.py` says in source that it NEVER calls `rank_key` or
`select_best`, because selecting with a predictor and then correlating that
predictor is the circle #703 exists to break. This study calls both, because
here the SELECTOR IS THE OBJECT UNDER MEASUREMENT. The question #703 answered --
does `crossings` rank `blocking`? -- is already answered (5 right / 1 wrong,
fails the sign rule, `docs/placement-predictors.md`). Two different questions
are asked here, and both are about the selector's own output:

  Q-BAR    does the crossings clause of `rule1_check` ever bar a candidate
           that routed to strictly LOWER `blocking` than the baseline it is
           compared against? ONE such candidate discharges pre-registered
           rule 2. Rule 2 is an EXISTENCE claim: it can be discharged, never
           refuted, and the clause standing is a default rather than a finding.

  Q-ORDER  does the static order agree with the routed order over the same
           candidates? Kendall tau-b, pre-registered as rule 5. This does NOT
           discharge rule 2 -- rule 2 is about the BAR, rule 5 about the ORDER.

WHAT MUST NOT HAPPEN, and `tests/mutate_789.py` has a row for it: computing the
tau against `ranking_routed` AS PRINTED. Its final tiebreak is `static_pos`
(`place_portfolio.py`), so that would correlate the two orders by construction
and produce a number that means nothing. The routed order here is keyed on
`board_score`'s `blocking` VALUE, and its ties are left as ties.

WHY THIS BUYS ONE ROUTE PER BOARD INSTEAD OF ELEVEN. #703 already routed and
graded ten portfolio candidates per board under a frozen argv. The only thing
missing from a rule-1 slate is candidate 0 -- `portfolio.generate` refuses
`only=0`, so the study never made one. This grafts: candidate 0 is generated,
routed and scored here; candidates 1..10 come from the recorded rows. The graft
is only legitimate if the artifacts still reproduce, so it is GUARDED rather
than asserted -- see `--guards`. Any guard failure VOIDs the whole board; a
board is never dropped for its result.

WHAT THE GRAFT COSTS, said plainly: #703 generated its portfolio candidates
with `quench_kw=None`, the LIBRARY defaults (max_displacement 10.0,
crossing_penalty 10.0, length_weight 1.0, halo_coef 0.25), while
`place_portfolio.main` ships 3.0 / 30.0 / 0.3 / 0.15. So this measures
`rule1_check` and `rank_key` on `portfolio.generate`'s library-default slate,
not on the CLI's. Candidate 0 is generated with the same library defaults, so
the comparison inside a board is on equal terms and rule 2 is discharged on its
own terms -- but a reader is owed the difference, and the 4x lower crossing
penalty is on exactly the axis this question is about.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import rank_stats as rs                                          # noqa: E402
import predictor_study as PS                                     # noqa: E402

SCHEMA = 1

#: The board set, IMPORTED rather than re-declared. A second board list is a
#: second study, and reusing #703's is what makes the graft possible at all.
SLATE_BOARDS = PS.STUDY_BOARDS

#: Candidates 1..10, exactly the indices #703 recorded. Candidate 0 is
#: generated here; `portfolio.generate` has no stream for it under `only=`.
GRAFT_INDICES = PS.PORTFOLIO_INDICES

#: Where #703's artifacts live. Read-only to this study.
STUDY_DIR = os.path.join(ROOT, 'wk', '703', 'study')

#: Boards whose guards are reduced, DECLARED IN ADVANCE rather than chosen when
#: the clock ran long. kit-dev-coldfire's quench is ~1.75 h and its route
#: ~1.2 h, so a re-route control would cost more than the measurement it
#: guards. It gets ONE regeneration check and no re-route control, and every
#: report says so on the board's own line.
REDUCED_GUARDS = {'kit-dev-coldfire-xilinx_5213'}

#: How many candidates per board the regeneration control re-derives.
REGEN_CHECKS = 2


def _sha(path):
    return PS.sha256_file(path)


def study_rows(board_key):
    """#703's rows for one board, keyed by variant."""
    p = os.path.join(STUDY_DIR, 'rows.jsonl')
    if not os.path.isfile(p):
        raise SystemExit(
            f'REFUSED: {p} is not here. This study GRAFTS #703\'s routed\n'
            f'candidates; without its rows there is nothing to graft onto.\n'
            f'Rebuild with: python3 -X utf8 tests/stress/predictor_study.py '
            f'--out wk/703/study -j 4')
    out = {}
    for r in PS.load_rows(p):
        if r.get('board_key') == board_key:
            out[r.get('variant')] = r
    return out


def cand_board(board_key, i):
    """The on-disk board #703 wrote for candidate i, or None."""
    p = os.path.join(STUDY_DIR, board_key, f'portfolio-{i}', f'pf_{i}',
                     f'cand_{i:02d}.kicad_pcb')
    return p if os.path.isfile(p) else None


def make_candidate_zero(board_file, out_dir, seed=0):
    """Generate candidate 0 -- the plain quench -- exactly as the library does.

    `only=None` with `n_candidates=1` produces the baseline and NO perturbed
    candidates (`range(1, 1)` is empty), and `quench_kw=None` is what #703
    passed for candidates 1..10, so the slate is internally consistent.
    """
    from placement import portfolio as PF
    res = PF.generate(board_file, out_dir, seed=seed, n_candidates=1,
                      strategies=PS.PORTFOLIO_STRATEGIES, only=None)
    return res.get('baseline'), res.get('free') or []


def rebuild_candidate(board_file, out_dir, i, seed=0):
    """Regenerate candidate i byte-identically -- the graft's control."""
    from placement import portfolio as PF
    res = PF.generate(board_file, out_dir, seed=seed, n_candidates=i + 1,
                      strategies=PS.PORTFOLIO_STRATEGIES, only=i)
    cands = res.get('candidates') or []
    return cands[0] if cands and cands[0].board else None


def route_and_score(board_path, work, tag, timeout):
    """Route with #703's frozen argv, then grade with board_score.

    Returns (blocking, detail). `blocking` is `board_score`'s key and nothing
    else: `converge.route_verdict`'s `failures` is a DIFFERENT sum
    (failed_single + open_single + a pad deficit) and `check_assembly` answers
    a buildability question, not a routing one. Rule 2 says `blocking`.
    """
    os.makedirs(work, exist_ok=True)
    routed = os.path.join(work, f'{tag}.routed.kicad_pcb')
    rjson = os.path.join(work, f'{tag}.route.json')
    argv = PS.route_argv_for(board_path, routed, rjson)
    r = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True,
                       timeout=timeout)
    if r.returncode != 0 or not os.path.isfile(routed):
        return None, {'stage': 'route', 'returncode': r.returncode,
                      'tail': (r.stderr or r.stdout or '')[-400:]}
    sjson = os.path.join(work, f'{tag}.score.json')
    s = subprocess.run(
        [sys.executable, '-X', 'utf8', PS.BOARD_SCORE, routed, '--json', sjson,
         '--label', f'slate789:{tag}', '-q'],
        cwd=ROOT, capture_output=True, text=True, timeout=timeout)
    if not os.path.isfile(sjson):
        return None, {'stage': 'score', 'returncode': s.returncode,
                      'tail': (s.stderr or s.stdout or '')[-400:]}
    with open(sjson, encoding='utf-8') as fh:
        doc = json.load(fh)
    # None, never 0: board_score's own vacuity rule. A component that could not
    # run makes `blocking` unknown, and an unknown is not a clean board.
    return doc.get('blocking'), {'stage': 'ok', 'routed': routed,
                                 'score': sjson,
                                 'blocking_by': doc.get('blocking_by')}


# ---------------------------------------------------------------------------
# the slate: candidate 0 generated here, 1..10 grafted from #703
# ---------------------------------------------------------------------------

def build_slate(board_key, board_file, out_dir, seed=0, timeout=14400,
                reduced=False):
    """One board's slate, with every graft guard executed.

    Returns (rows, verdict, notes). `verdict` is 'ok' or 'VOID:<reason>'. A
    board is VOIDed only by a GUARD, never by its result -- selecting boards
    after seeing the answer is the failure a pre-registered board set exists to
    prevent, and a guard failure is a statement about the artifacts rather than
    about the question.
    """
    from placement import portfolio as PF
    notes = []
    work = os.path.join(out_dir, board_key)
    os.makedirs(work, exist_ok=True)

    rows_703 = study_rows(board_key)
    abs_board = os.path.join(ROOT, board_file)

    # GUARD 1 -- the argv. If #703 routed these candidates with a different
    # command from the one candidate 0 will get, `blocking_0` is incomparable
    # with every grafted `blocking_i`, and it is incomparable in the direction
    # that MANUFACTURES a fire. This is the guard that matters most.
    #
    # The REPO-RELATIVE path is what gets signed, because that is what #703
    # signed: `argv_signature` basenames only the `.py` entries, so the board
    # path itself is inside the hash and an absolute path hashes differently.
    # This guard caught that on its first run -- which is the guard working,
    # not a false alarm, since a signature that ignored the board would not be
    # a signature of the command.
    sig = PS.freeze_argv(board_key, board_file, out_dir)
    old_argv = os.path.join(STUDY_DIR, board_key, 'ARGV.json')
    if not os.path.isfile(old_argv):
        return [], f'VOID:no #703 ARGV.json for {board_key}', notes
    with open(old_argv, encoding='utf-8') as fh:
        if json.load(fh).get('argv_sha') != sig:
            return [], 'VOID:argv_sha differs from the #703 run', notes

    # GUARD 2 -- the input board is the one #703 measured.
    want_in = {(r.get('provenance') or {}).get('input_board_sha')
               for r in rows_703.values()}
    if len(want_in) != 1 or _sha(abs_board) not in want_in:
        return [], 'VOID:input board sha differs from the #703 rows', notes

    # Candidate 0: generated HERE, because portfolio.generate has no stream for
    # it under `only=` and #703 therefore never made one.
    zero, free = make_candidate_zero(abs_board, os.path.join(work, 'pf_0'),
                                     seed=seed)
    if zero is None or not zero.board:
        return [], 'VOID:candidate 0 could not be generated', notes

    # The gate baselines are candidate 0's own, exactly as place_portfolio
    # computes them before scoring anything.
    b_overlap = zero.metrics.get('overlap_area', 0.0)
    b_oob = zero.metrics.get('oob_count', 0)
    b_pad = zero.metrics.get('pad_conflict_pairs', 0)
    b_hole = zero.metrics.get('hole_shortfall', 0.0)

    # BOARD-FIRST, like `place_portfolio` does it, rather than a constant.
    # `viable` IS the tau's K and the disclosed cause of two boards reporting
    # K=4, so deciding it at a fixed 0.25 would be deciding the denominator at
    # a number the board never asked for -- `board_floor_knobs`' own docstring
    # warns a constant manufactures phantom violations. Measured: all six
    # boards resolve to exactly 0.25 / 0.55, so this changes no published
    # number; it stops a seventh board from silently diverging from the
    # selector this study is measuring.
    try:
        from list_nets import board_floor_knobs
        _clr, _edge = board_floor_knobs(abs_board, None, None)[:2]
    except Exception:                                       # noqa: BLE001
        _clr, _edge = 0.25, 0.55

    def _score(c):
        PF.score_candidate(
            c, free=free, baseline_overlap=b_overlap, baseline_oob=b_oob,
            baseline_pad_pairs=b_pad, baseline_hole_shortfall=b_hole,
            clearance=_clr, board_edge_clearance=_edge, grid_step=0.1,
            ignore_nets=None)

    _score(zero)

    # Candidates 1..10, reconstructed from #703's artifacts. `metrics` comes
    # from the recorded row (its `crossings` and `hpwl` ARE `_quench_metrics`'s
    # -- the same numbers rank_key and rule1_check compare); `inversions` and
    # the gates are recomputed here from the board on disk, because #703 never
    # ran score_candidate and PREDICTOR_KEYS does not carry inversions.
    cands = [zero]
    for i in GRAFT_INDICES:
        row = rows_703.get(f'portfolio-{i}')
        bp = cand_board(board_key, i)
        if row is None or bp is None:
            notes.append(f'candidate {i}: no recorded row or board, skipped')
            continue
        prov = row.get('provenance') or {}
        if prov.get('variant_board_sha') != _sha(bp):
            return [], f'VOID:candidate {i} board on disk differs from its row', notes
        c = PF.Candidate(
            index=i, strategy=(prov.get('recipe') or {}).get('strategy', 'jitter'),
            board=bp, metrics=dict(row.get('predictors') or {}),
            displacement_rms=float(
                (prov.get('recipe') or {}).get('displacement_rms') or 0.0))
        _score(c)
        c.route = {'blocking': (row.get('truth') or {}).get('blocking'),
                   'source': 'graft:#703', 'row_id': row.get('row_id')}
        cands.append(c)

    if len(cands) < 1 + rs.MIN_N:
        return [], f'VOID:only {len(cands)} candidates reconstructed', notes

    # GUARD 3 -- REGENERATION. The graft's whole licence is that
    # `portfolio.generate(..., only=i)` is byte-identical by contract. Prove it
    # on this build rather than citing the contract: an engine change since
    # #703's `measured_git` would otherwise make this a comparison between two
    # different quenches wearing one name.
    checks = 1 if reduced else REGEN_CHECKS
    ran_regen = 0
    for i in list(GRAFT_INDICES)[:checks]:
        row = rows_703.get(f'portfolio-{i}')
        if row is None:
            continue
        again = rebuild_candidate(abs_board, os.path.join(work, f'regen_{i}'),
                                  i, seed=seed)
        if again is None:
            return [], f'VOID:candidate {i} no longer regenerates', notes
        got = PS.poses_sha(again.board)
        want = (row.get('provenance') or {}).get('poses_sha256')
        if got != want:
            return [], (f'VOID:candidate {i} regenerates to a different '
                        f'placement ({got[:12]} vs {want[:12]})'), notes
        ran_regen += 1
        notes.append(f'regeneration control: candidate {i} reproduces')

    # A guard that checked NOTHING is not a guard. If the declared candidates
    # had no recorded row the loop above would `continue` past every one of
    # them, append no note, and let the board proceed with the graft unproven.
    # Found by an adversarial review; it was not live (all six boards record
    # portfolio-1..10) and it is now impossible rather than merely unobserved.
    if ran_regen < checks:
        return [], (f'VOID:regeneration control ran {ran_regen} of {checks} '
                    f'declared checks'), notes

    # Candidate 0 is routed and graded HERE -- the one route this study buys.
    blk0, detail0 = route_and_score(zero.board, os.path.join(work, 'route'),
                                    'cand0', timeout)
    zero.route = {'blocking': blk0, 'source': 'measured', 'detail': detail0}
    if blk0 is None:
        return [], f'VOID:candidate 0 did not produce a blocking ({detail0})', notes

    # GUARD 4 -- RE-ROUTE. Route ONE grafted candidate again, to a FRESH output
    # path (a re-used path reads back its own .kicad_pro floor and silently
    # changes the routing), and require the same `blocking` its score.json
    # recorded. Without this a router change since #703 turns the graft into a
    # comparison between two routers. The pick is the lowest recorded index --
    # fixed, so it cannot be chosen for its answer.
    if reduced:
        notes.append('re-route control SKIPPED (declared reduced guards)')
    else:
        pick = next((c for c in cands[1:] if c.route.get('blocking') is not None),
                    None)
        if pick is None:
            return [], 'VOID:no grafted candidate carries a blocking', notes
        again_blk, d = route_and_score(pick.board,
                                       os.path.join(work, 'reroute'),
                                       f'reroute_{pick.index}', timeout)
        if again_blk != pick.route['blocking']:
            return [], (f'VOID:re-routing candidate {pick.index} gives '
                        f'blocking {again_blk}, recorded '
                        f'{pick.route["blocking"]} ({d})'), notes
        notes.append(f're-route control: candidate {pick.index} reproduces '
                     f'blocking {again_blk}')

    rows = [_row(board_key, board_file, c, zero, sig, seed, reduced,
                 ran_regen)
            for c in cands]
    return rows, 'ok', notes


def _row(board_key, board_file, c, zero, argv_sha, seed, reduced,
         regen_checks=0):
    """One slate row. `rule1_would_bar` is recorded per CLAUSE, not as a bool:
    a candidate barred on hpwl as well stays barred after the crossings clause
    is withdrawn, so it is not a candidate the withdrawal changes."""
    from placement import portfolio as PF
    clauses = PF.rule1_check(c, zero) if c.index else []
    return {
        'schema': SCHEMA, 'kind': 'slate-row',
        'row_id': f'slate:{board_key}:{c.index}',
        'board_key': board_key, 'index': c.index,
        'strategy': c.strategy, 'viable': bool(c.viable),
        'predictors': {k: c.metrics.get(k) for k in
                       ('crossings', 'hpwl', 'inversions', 'plane_islands',
                        'health_penalty', 'plane_neck')},
        'displacement_rms': c.displacement_rms,
        'rank_key': list(PF.rank_key(c, 0)),
        'rule1_clauses': clauses,
        'rule1_would_bar_on_crossings': any(
            s.startswith('crossings ') for s in clauses),
        'rule1_would_bar_on_hpwl': any(s.startswith('hpwl ') for s in clauses),
        'truth': {'blocking': (c.route or {}).get('blocking'),
                  'source': (c.route or {}).get('source')},
        'provenance': {'input_board': board_file, 'argv_sha': argv_sha,
                       'seed': seed, 'measured_git': PS.git_describe(ROOT),
                       'reduced_guards': reduced,
                       'regen_checks_run': regen_checks,
                       'quench_kw': 'library defaults (quench_kw=None)'},
    }


# ---------------------------------------------------------------------------
# Q-BAR -- pre-registered rule 2
# ---------------------------------------------------------------------------

#: Why a board could not fire the criterion, decided BEFORE the run and
#: reported with its constant value rather than dropped (rule 4).
CANNOT_FIRE = ('baseline_clean', 'no_barred_candidate',
               'no_graded_barred_candidate')


def qbar(rows):
    """Does the crossings clause bar a candidate that routed BETTER?

    FIRE(i) := barred on crossings AND NOT barred on hpwl AND both `blocking`
    defined AND blocking_i < blocking_0.

    The `not barred on hpwl` conjunct is deliberate and pre-registered, because
    it is the clause most likely to be re-litigated afterwards: a candidate
    barred on BOTH stays barred once the crossings clause is withdrawn, so its
    treatment is not changed by the withdrawal and it is not evidence about
    the withdrawal. `fire_literal` records rule 2's literal wording beside it,
    so neither reading can be picked after seeing which one fired.
    """
    zero = next((r for r in rows if r['index'] == 0), None)
    if zero is None:
        return {'verdict': 'void', 'reason': 'no candidate 0 in these rows'}
    b0 = zero['truth'].get('blocking')
    if b0 is None:
        return {'verdict': 'void', 'reason': 'candidate 0 has no blocking'}
    # VIABLE only. `select_best` walks `rank_static`, which ranks viable
    # candidates alone, so a gate-rejected candidate was never excluded BY the
    # crossings bar and cannot be evidence about it. Counting them inflated
    # `n_barred_on_crossings` (watchy reported 2, one of them gate-rejected)
    # and could in principle have let one FIRE. Measured before the change:
    # every firing candidate on both discharging boards was viable, so the
    # discharge is unaffected -- but the count was wrong.
    others = [r for r in rows if r['index'] != 0 and r['viable']]
    barred_x = [r for r in others if r['rule1_would_bar_on_crossings']]
    graded = [r for r in barred_x if r['truth'].get('blocking') is not None]
    fire = [r for r in graded
            if not r['rule1_would_bar_on_hpwl']
            and r['truth']['blocking'] < b0]
    fire_lit = [r for r in graded if r['truth']['blocking'] < b0]
    out = {'baseline_blocking': b0, 'n_candidates': len(others),
           'n_barred_on_crossings': len(barred_x), 'n_graded_barred': len(graded),
           'fire': [r['row_id'] for r in fire],
           'fire_literal': [r['row_id'] for r in fire_lit],
           'blocking': {r['row_id']: r['truth'].get('blocking') for r in others}}
    if b0 == 0:
        out['verdict'] = 'cannot_fire'
        out['reason'] = 'baseline_clean'
    elif not barred_x:
        out['verdict'] = 'cannot_fire'
        out['reason'] = 'no_barred_candidate'
    elif not graded:
        out['verdict'] = 'cannot_fire'
        out['reason'] = 'no_graded_barred_candidate'
    else:
        out['verdict'] = 'fires' if fire else 'does_not_fire'
    return out


def qbar_null(rows, n=200, seed=7789, verdict=None):
    """P(at least one FIRE by chance), permuting `blocking` WITHIN the board.

    Pre-registered, and printed beside the verdict whatever it says. It is
    EXPECTED to be high: roughly half a slate is crossings-barred and
    `blocking` varies, so rule 2's criterion is easy to satisfy. Saying so
    after seeing an 85% rate would read as excuse-making; saying it before is
    the honest form. A discharge at a high null rate is still a discharge --
    the rule is an existence claim and it was pre-registered -- but it is
    evidence of very little, and the number says how little.
    """
    # A board that CANNOT fire has no null rate to report: the criterion is
    # inapplicable there, not hard. Printing 0% beside `cannot_fire` read as
    # "a demanding criterion on this board", which is the opposite of true.
    if verdict == 'cannot_fire':
        return None
    import random
    rng = random.Random(seed)
    zero = next((r for r in rows if r['index'] == 0), None)
    graded = [r for r in rows if r['index'] != 0
              and r['truth'].get('blocking') is not None]
    if zero is None or zero['truth'].get('blocking') is None or not graded:
        return None
    vals = [r['truth']['blocking'] for r in graded]
    hits = 0
    for _ in range(n):
        shuffled = vals[:]
        rng.shuffle(shuffled)
        b0 = zero['truth']['blocking']
        if any(r['rule1_would_bar_on_crossings']
               and not r['rule1_would_bar_on_hpwl'] and v < b0
               for r, v in zip(graded, shuffled)):
            hits += 1
    return hits / float(n)


# ---------------------------------------------------------------------------
# Q-ORDER -- newly pre-registered rule 5
# ---------------------------------------------------------------------------

def qorder(rows):
    """tau-b between the STATIC position and the routed `blocking` VALUE.

    Both sides are built here rather than read from `place_portfolio`'s
    `ranking_routed`, whose final tiebreak is the static position -- using it
    would correlate the two orders by construction.

    SIGN CONVENTION, frozen with a worked example: tau is computed on
    (static_position, blocking), position 0 being the best static rank and a
    lower `blocking` being the better routed outcome. So a POSITIVE tau means
    the key AGREES. Worked: three candidates whose static positions are 0,1,2
    and whose blocking is 1,2,3 give three concordant pairs and tau-b +1.0.

    Candidate 0's key is COMPUTED, not the synthesised position
    `place_portfolio` pins baseline-first by fiat -- the question is about the
    key, so the key has to be asked.
    """
    from placement import portfolio as PF
    viable = [r for r in rows if r['viable']
              and r['truth'].get('blocking') is not None]
    # WHY K IS OFTEN BELOW 11, disclosed rather than left for a reader to
    # notice. `rank_static` ranks only gate-passing candidates, so a candidate
    # `score_candidate` rejected is not one this order ever had to get right --
    # excluding it is the shipped behaviour, not a convenience. But the count
    # travels with the tau, because a K of 4 and a K of 11 are not the same
    # evidence and the coefficient alone does not say which it is.
    excluded = {'not_viable': len([r for r in rows if not r['viable']]),
                'ungraded': len([r for r in rows if r['viable']
                                 and r['truth'].get('blocking') is None])}
    if len(viable) < rs.MIN_N_FOR_TAU:
        return {'verdict': 'no_verdict',
                'reason': f'{len(viable)} viable graded candidates < '
                          f'{rs.MIN_N_FOR_TAU}',
                'n': len(viable), 'excluded': excluded}
    order = sorted(viable, key=lambda r: tuple(r['rank_key']))
    pos = {r['row_id']: i for i, r in enumerate(order)}
    xs = [pos[r['row_id']] for r in viable]
    ys = [r['truth']['blocking'] for r in viable]
    side = rs.constant_side(xs, ys)
    if side:
        return {'verdict': 'no_verdict', 'n': len(viable),
                'excluded': excluded,
                'reason': {'dependent': 'truth constant (saturated)',
                           'predictor': 'static order constant',
                           'both': 'both sides constant'}[side],
                'constant_value': ys[0] if side in ('dependent', 'both') else None}
    k = rs.tau_counts(xs, ys)
    lo, hi = rs.tau_loo_span(xs, ys)
    tau = rs.kendall_tau(xs, ys)
    return {'verdict': 'measured', 'n': len(viable), 'excluded': excluded,
            'tau': tau,
            'tau_a': rs.tau_a(xs, ys), 'concordant': k['concordant'],
            'discordant': k['discordant'], 'ties_a': k['ties_a'],
            'ties_b': k['ties_b'], 'loo_lo': lo, 'loo_hi': hi,
            'display': rs.fmt_tau(tau, lo, hi, len(viable), k['ties_a'],
                                  k['ties_b']),
            'order': [r['row_id'] for r in order],
            'blocking': ys}


def delta(rows):
    """What withdrawing the crossings clause would CHANGE, per board.

    Rule 2 asks whether the clause ever bars a candidate that routed better.
    That is the criterion. This is the consequence, and they are not the same
    question: a criterion can fire on a clause that decides nothing.

    `select_best` takes the first index in `ranking_primary + ranking_static`
    that is 0 or not a violator, so the answer depends on whether a PROBE
    ranking exists -- and `--route-top` defaults to 2, so in a shipped run one
    does. Both arms are computed:

      static-only   `--route-top 0`. Expect no change: `rank_key`'s slot 1 IS
                    crossings, so a candidate barred for having MORE crossings
                    than the baseline already sorts below every candidate with
                    fewer, and select_best reaches a non-violator first.
      oracle_probe  the probe ranked by each candidate's ACTUAL routed
                    `blocking`. It CANNOT show "worse", and that is a property
                    of the construction rather than a result: the withdrawn
                    violator set is a subset of the old one and the list is
                    sorted by the truth, so `select_best` can only move the
                    pick earlier. Reported because it bounds the BENEFIT, and
                    labelled because an earlier version of this study counted
                    its zero as a second independent arm of safety.
      worst_probe   a probe whose head is the crossings-barred candidate that
                    routed WORST. This is the adversarial case a real probe can
                    stumble into, and it is where the withdrawal can cost
                    something -- which is the honest other half of the claim.
    """
    from placement import portfolio as PF
    out = {}
    for b, rr in sorted(rs.per_board(rows).items()):
        R = sorted(rr, key=lambda r: r['index'])
        viable = [r for r in R if r['viable']
                  and r['truth'].get('blocking') is not None]
        if not viable:
            continue
        static = [r['index'] for r in sorted(viable,
                                             key=lambda r: tuple(r['rank_key']))]
        probe = [r['index'] for r in sorted(
            viable, key=lambda r: (r['truth']['blocking'], r['index']))]
        now = {r['index'] for r in R if r['rule1_clauses']}
        after = {r['index'] for r in R if r['rule1_would_bar_on_hpwl']}
        blk = {r['index']: r['truth'].get('blocking') for r in R}
        # The adversarial probe: head is the crossings-barred candidate that
        # routed WORST, which is the case a real probe can stumble into.
        barred_worst = [r['index'] for r in sorted(
            (r for r in R if r['rule1_would_bar_on_crossings']
             and not r['rule1_would_bar_on_hpwl']
             and r['truth'].get('blocking') is not None and r['viable']),
            key=lambda r: -r['truth']['blocking'])]
        worst = (barred_worst[:1]
                 + [i for i in static if i not in barred_worst[:1]])
        arms = {}
        for name, primary in (('static_only', []), ('oracle_probe', probe),
                              ('worst_probe', worst)):
            n = PF.select_best(primary, static, now)
            a = PF.select_best(primary, static, after)
            bn, ba = blk.get(n), blk.get(a)
            # `select_best` returns None when every ranked index is a violator
            # AND index 0 is absent from the ranking -- which happens when
            # candidate 0 itself is not viable (`score_candidate` fails a
            # candidate against its own metrics on a pile-degenerate board).
            # Because the withdrawn violator set is a SUBSET, the two arms can
            # disagree about that, and comparing an int with None was a
            # TypeError that took the whole report down, `--from-rows`
            # included. Found by an adversarial review, not by a run.
            if n == a:
                verdict = 'unchanged'
            elif bn is None or ba is None:
                verdict = ('no pick before' if bn is None
                           else 'no pick after')
            elif ba < bn:
                verdict = 'better'
            elif ba > bn:
                verdict = 'worse'
            else:
                verdict = 'same blocking'
            arms[name] = {'pick_now': n, 'pick_after': a,
                          'blocking_now': bn, 'blocking_after': ba,
                          'verdict': verdict}
        out[b] = arms
    return out


def aggregate(rows):
    """Per board, then across boards by SIGN -- never pooled, never averaged."""
    by_board = rs.per_board(rows)
    per = {}
    for b, rr in sorted(by_board.items()):
        _q = qbar(rr)
        per[b] = {'qbar': _q, 'qorder': qorder(rr),
                  'qbar_null': qbar_null(rr, verdict=_q.get('verdict'))}
    fired = [b for b, d in per.items() if d['qbar'].get('verdict') == 'fires']
    able = [b for b, d in per.items()
            if d['qbar'].get('verdict') in ('fires', 'does_not_fire')]
    # Rule 5's shape is rule 3's, and it introduces NO magnitude threshold: any
    # tau cut-off would be a number chosen after seeing tau's scale. The
    # threshold IS the sign rule.
    #
    # It is DELEGATED to `rank_stats.sign_test` rather than re-derived here.
    # An adversarial review caught the hand-rolled version dropping both of
    # that function's corrections: it divided by every board with a defined
    # tau (zeros included) instead of by the boards with a DIRECTION, and it
    # never clamped -- so a 2-positive / 2-negative split printed
    # `two-sided p=1.375`, which is not a probability. The shipped run landed
    # on exactly 1.000 by luck. `rank_stats` is this repo's only sanctioned
    # aggregation and its own docstring warns about a p-value labelled with
    # the wrong denominator; re-deriving it was the mistake.
    taus = {b: d['qorder'] for b, d in per.items()
            if d['qorder'].get('verdict') == 'measured'}
    st = rs.sign_test({b: rs.BoardRho(d['tau'], d['n'])
                       for b, d in taus.items()})
    pos, neg = st['positive'], st['negative']
    n = st['boards_defined']
    agrees = st['passes_sign_rule'] and not neg
    disagrees = (n >= rs.MIN_SIGN_BOARDS and len(neg) >= max(1, n - 1)
                 and not pos)
    return {
        'per_board': per,
        'delta': delta(rows),
        'rule2': {
            'discharged': bool(fired),
            'boards_that_fired': fired,
            'boards_able_to_fire': able,
            'verdict': ('DISCHARGED' if fired
                        else 'NO VERDICT' if len(able) < rs.MIN_SIGN_BOARDS
                        else 'STANDS (not discharged)'),
        },
        'rule5': {
            'n_boards_with_a_defined_tau': n,
            'zero': st['zero'],
            'positive': sorted(pos), 'negative': sorted(neg),
            'p_two_sided': st['p_two_sided'],
            'p_denominator': st['p_denominator'],
            'verdict': ('AGREES' if agrees
                        else 'DISAGREES (open a reorder issue, do not reorder)'
                        if disagrees
                        else 'NOT SHOWN TO AGREE'),
        },
    }


# ---------------------------------------------------------------------------
# reporting and CLI
# ---------------------------------------------------------------------------

def report(agg):
    out = []
    out.append('')
    out.append('Q-BAR -- pre-registered rule 2 (the crossings clause of '
               'rule1_check)')
    out.append(f'  {"board":30s} {"verdict":22s} {"base":>5s} '
               f'{"barred":>7s} {"fires":>6s}  null')
    for b, d in sorted(agg['per_board'].items()):
        q = d['qbar']
        nul = d.get('qbar_null')
        out.append(
            f'  {b:30s} {q.get("verdict", "?") + (":" + q["reason"] if q.get("reason") else ""):22s} '
            f'{str(q.get("baseline_blocking")):>5s} '
            f'{str(q.get("n_barred_on_crossings")):>7s} '
            f'{str(len(q.get("fire", []))):>6s}  '
            f'{"n/a" if nul is None else f"{nul:.0%}"}')
    r2 = agg['rule2']
    out.append(f'  => rule 2: {r2["verdict"]}'
               + (f'  (fired on {", ".join(r2["boards_that_fired"])})'
                  if r2['boards_that_fired'] else '')
               + f'  [{len(r2["boards_able_to_fire"])} board(s) ABLE to fire]')
    out.append('     Rule 2 is an EXISTENCE claim: it can be discharged, never')
    out.append('     refuted. The clause standing is a default, not a finding.')
    out.append('')
    out.append('WHAT THE WITHDRAWAL WOULD CHANGE (the consequence, not the '
               'criterion)')
    for b, arms in sorted((agg.get('delta') or {}).items()):
        def _v(a):
            return (a['verdict']
                    + ('' if a['verdict'] == 'unchanged'
                       else ' (%s->%s)' % (a['blocking_now'],
                                           a['blocking_after'])))
        out.append('  %-30s static: %-12s oracle: %-12s worst-probe: %s'
                   % (b, _v(arms['static_only']), _v(arms['oracle_probe']),
                      _v(arms['worst_probe'])))
    _ws = [b for b, a in (agg.get('delta') or {}).items()
           if a['static_only']['verdict'] == 'worse']
    _ww = [b for b, a in (agg.get('delta') or {}).items()
           if a['worst_probe']['verdict'] == 'worse']
    out.append('  => worse on %d board(s) on the STATIC order%s'
               % (len(_ws), (': ' + ', '.join(_ws)) if _ws else ''))
    out.append('     worse on %d board(s) under an adversarial probe%s'
               % (len(_ww), (': ' + ', '.join(_ww)) if _ww else ''))
    out.append('     The oracle arm cannot show "worse" -- the new violator '
               'set is a')
    out.append('     subset and the list is truth-sorted, so the pick can '
               'only move')
    out.append('     earlier. Its zero is construction, not evidence.')
    out.append('')
    out.append('Q-ORDER -- newly pre-registered rule 5 (rank_key slot 1\'s order)')
    for b, d in sorted(agg['per_board'].items()):
        q = d['qorder']
        ex = q.get('excluded') or {}
        exs = (f'  ({ex.get("not_viable", 0)} gate-rejected, '
               f'{ex.get("ungraded", 0)} ungraded)'
               if any(ex.values()) else '')
        if q.get('verdict') == 'measured':
            out.append(f'  {b:30s} {q["display"]}'
                       f'  [C={q["concordant"]} D={q["discordant"]}]{exs}')
        else:
            out.append(f'  {b:30s} tau=n/a [{q.get("reason")}'
                       + (f', constant {q["constant_value"]}'
                          if q.get('constant_value') is not None else '')
                       + f', K={q.get("n")}]{exs}')
    r5 = agg['rule5']
    out.append(f'  => rule 5: {r5["verdict"]}  '
               f'({len(r5["positive"])} positive, {len(r5["negative"])} '
               f'negative, N={r5["n_boards_with_a_defined_tau"]}'
               + (f', two-sided p={r5["p_two_sided"]:.3f}'
                  if r5['p_two_sided'] is not None else '') + ')')
    out.append('     A board with an undefined tau is reported with its')
    out.append('     constant value and excluded from the denominator, never')
    out.append('     dropped (rule 4).')
    return out


def main(argv=None):
    rs._self_test()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', default=os.path.join(ROOT, 'wk', '789'),
                    help='work dir (gitignored)')
    ap.add_argument('--boards', nargs='*', default=None,
                    help='NARROW the declared set. A narrowed run prints that '
                         'it is narrowed; a verdict needs the declared table.')
    ap.add_argument('-j', type=int, default=1)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--route-timeout', type=int, default=14400)
    ap.add_argument('--from-rows', metavar='PATH',
                    help='re-derive every statistic from rows already written')
    ap.add_argument('--plan', action='store_true',
                    help='print what would run, and its measured cost')
    ap.add_argument('--shuffle-control', type=int, default=200)
    a = ap.parse_args(argv)

    if a.from_rows:
        rows = PS.load_rows(a.from_rows)
        if not rows:
            print(f'no rows in {a.from_rows}')
            return 2
        for ln in report(aggregate(rows)):
            print(ln)
        return 0

    boards = [b for b in SLATE_BOARDS
              if not a.boards or b['key'] in a.boards]
    if a.boards:
        print('NOTE: this is a NARROWED run. A verdict over the declared '
              'table requires all 6 boards.')
    if a.plan:
        for b in boards:
            print(f'  {b["key"]:30s} guards='
                  f'{"reduced" if b["key"] in REDUCED_GUARDS else "full"}')
        return 0

    os.makedirs(a.out, exist_ok=True)
    jsonl = os.path.join(a.out, 'slate_rows.jsonl')
    done = set()
    if os.path.isfile(jsonl):
        for r in PS.read_jsonl(jsonl):
            done.add((r.get('board_key'), r.get('index')))

    def one(b):
        if (b['key'], 0) in done:
            return b['key'], [], 'skipped (already in slate_rows.jsonl)', []
        rows, verdict, notes = build_slate(
            b['key'], b['file'], a.out, seed=a.seed,
            timeout=a.route_timeout, reduced=b['key'] in REDUCED_GUARDS)
        return b['key'], rows, verdict, notes

    results = []
    with ThreadPoolExecutor(max_workers=max(1, a.j)) as ex:
        for key, rows, verdict, notes in ex.map(one, boards):
            results.append((key, verdict, notes))
            print(f'[{key}] {verdict}')
            for n in notes:
                print(f'    {n}')
            if rows:
                with open(jsonl, 'a', encoding='utf-8') as fh:
                    for r in rows:
                        fh.write(json.dumps(r, sort_keys=True) + '\n')

    voided = [(k, v) for k, v, _ in results if v.startswith('VOID')]
    if voided:
        print('\nVOIDED BOARDS (a guard failed -- never a result):')
        for k, v in voided:
            print(f'  {k}: {v}')
    if os.path.isfile(jsonl):
        for ln in report(aggregate(PS.load_rows(jsonl))):
            print(ln)
    return 0


if __name__ == '__main__':
    sys.exit(main())
