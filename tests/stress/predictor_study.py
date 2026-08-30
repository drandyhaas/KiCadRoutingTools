#!/usr/bin/env python3
"""Do pre-route placement predictors rank the routed outcome? (#703)

WHAT THIS ANSWERS, AND WHAT THE REPO HAD INSTEAD

Every correlation number quoted in this repo's skills and drivers is measured
against distance-to-the-correct-placement or against the gap a human left.
`r(crossings) = +0.780` is 29 candidates on ONE board against distance-to-truth;
the corridor law's `r = +0.41..+0.90` is against the human's gap. CLAUDE.md's
own "What a placement run is FOR" says the headline is `blocking`
(`unrouted + broken + ...`), and no predictor here had ever been correlated
with it. `harvest_predictor_rows.py` shows the recorded runs cannot supply the
answer -- one placement per board is n=1 per board, which is below the
arithmetic floor. So this generates the placements and pays for the routes.

THE ORDER IS THE EXPERIMENT, AND THE DRIVER ENFORCES IT

Four circularity traps, each of which this repo has already been bitten by:

  1. **Route argv is frozen per board BEFORE any variant exists.**
     `route_argv_for` derives it from the board alone, hashes it into
     `<out>/<board>/ARGV.json`, and REFUSES to generate or route anything if
     that file already exists at a different hash. Per-variant tuning is not
     representable in this tool, and the aggregator refuses a board whose rows
     disagree about `argv_sha`.

  2. **The sampler optimises none of the predictors.** The bad end comes from
     `placement/perturb.py` (the #411 damage rig) and the realistic end from
     `portfolio.generate`, whose quench DOES minimise crossings and hpwl -- so
     those rows carry `generator: "portfolio_quench"` and every statistic is
     reported twice, with and without them. If the two disagree in sign the
     report says so and calls neither the answer. Nothing here ever calls
     `rank_key` or `select_best`: selecting with a predictor and then
     correlating that predictor is the circle this issue exists to break.

  3. **Predictors are re-derived from the WRITTEN board.** Structurally, not by
     convention: `generate_variant` returns a PATH and `predictors_for` takes a
     path and nothing else, so the optimizer's live `QuenchState` is
     unreachable from the measurement.

  4. **A fresh output path per route.** `route.py` reads back a sibling
     `.kicad_pro` DRC floor, so re-running to the same path silently changes
     the routing -- it looks like non-determinism and is not (CLAUDE.md).

THE STATISTIC IS WITHIN A BOARD, NEVER POOLED

`rank_stats` will not let it be otherwise: `board_rho` refuses rows from more
than one board, and `sign_test` takes a mapping. Pooling measures board size --
on this repo's own corpus `rho(crossings, vias)` is +0.714 pooled across six
boards and -0.400 within one board's slate, opposite signs for the same two
quantities. Boards are aggregated by a SIGN TEST, the shape
`test_placement_ab.gate()` already uses: right direction on >= N-1 boards,
wrong direction on none, with the `1 in 2^N` coin-flip null printed beside it.

`blocking` and `vias` are reported as SEPARATE dependent variables. The recorded
evidence says crossings may predict cost and not completion, and collapsing the
two would hide exactly that.

SATURATION IS REPORTED, NEVER DROPPED

A board where every variant reaches `blocking 0` has no headroom and can rank
nothing; a board where every variant is equally broken is the same problem at
the other end. Both are printed with their constant value, counted in "boards
attempted", excluded from the sign test's denominator -- and BOTH numbers are
printed, because a p-value labelled with the planned board count when saturation
reduced the real one is a verdict resting on a criterion nobody printed.

    python3 -X utf8 tests/stress/predictor_study.py --calibrate --out wk/703
    python3 -X utf8 tests/stress/predictor_study.py --plan
    python3 -X utf8 tests/stress/predictor_study.py --out wk/703 -j 4
    python3 -X utf8 tests/stress/predictor_study.py --task esp_prog:authored --out wk/703
    python3 -X utf8 tests/stress/predictor_study.py --from-rows wk/703/rows.jsonl
    python3 -X utf8 tests/stress/predictor_study.py --from-rows ... --shuffle-control 200
    python3 -X utf8 tests/stress/predictor_study.py --verify-row esp_prog:authored

The run is hours and a session can die inside it, so rows are appended to a
JSONL as each task finishes and `--resume` skips any (board, variant) already
present. `--from-rows` re-derives every statistic with no routing at all, so the
answer never has to be bought twice.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import glob
import hashlib
import json
import os
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _HERE)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import rank_stats as rs                                        # noqa: E402
from harvest_predictor_rows import (METRIC_KEYS, PREDICTOR_KEYS,  # noqa: E402
                                    TRUTH_BY_KEYS, sha256_file, git_describe)

SCHEMA = 1
BOARD_SCORE = os.path.join(
    ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts',
    'board_score.py')
ROUTE_PY = os.path.join(ROOT, 'py_router', 'route.py')

#: THE BOARD SET, fixed in source before any number was seen.
#:
#: Every board is git-TRACKED, which is the property that makes this study
#: reproducible rather than a report from one machine: with the board, the seed
#: and the frozen argv, any reviewer can regenerate a variant byte-identically
#: (`portfolio.generate(..., only=i)` is deterministic by contract) and re-route
#: it. The research note that proposed this study named `neo6502`, which lives
#: only in an external corpus; it is dropped for exactly that reason.
#:
#: `route_seconds` is MEASURED by `--calibrate`, not estimated. Boards are
#: chosen to span part count and layer count within a budget that a laptop can
#: actually pay.
STUDY_BOARDS = [
    # key, board file, measured authored route seconds, authored blocking
    {'key': 'esp_prog', 'file': 'kicad_files/esp_prog.kicad_pcb'},
    {'key': 'splitflap_driver', 'file': 'kicad_files/splitflap_driver.kicad_pcb'},
    {'key': 'watchy', 'file': 'kicad_files/watchy.kicad_pcb'},
    {'key': 'tigard', 'file': 'kicad_files/tigard.kicad_pcb'},
    {'key': 'sonde_u', 'file': 'kicad_files/sonde_u.kicad_pcb'},
    {'key': 'kit-dev-coldfire-xilinx_5213',
     'file': 'kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb'},
]

CALIBRATION_CANDIDATES = STUDY_BOARDS + [
    {'key': 'ulx3s', 'file': 'kicad_files/ulx3s.kicad_pcb'},
    {'key': 'glasgow_revC', 'file': 'kicad_files/glasgow_revC.kicad_pcb'},
]

#: K = 20 per board. At K=20 a per-board Spearman resolves |rho| >= 0.44 from
#: zero; at K=12 only |rho| >= 0.6, which is why it is not 12. The split spans
#: the range on purpose -- an all-realistic slate has no headroom and an
#: all-damaged one has no ceiling.
#: Kinds whose damage SCALES with the requested dose, so three doses give three
#: placements.
DOSED_KINDS = ('translate', 'scatter')
#: Kinds that ignore the dose entirely, so they contribute exactly ONE variant.
#:
#: `swap` trades the two highest-ranked disjoint blocks -- a deterministic
#: choice with no dose and no rng in it. `pile` collapses every free part to one
#: coordinate. And `wrong_side` REFLECTS the block through the board centre:
#: `perturb.py`'s translate branch computes `t = (2*(bx-cx), 2*(by-cy))` for
#: that kind and never reads the dose at all.
#:
#: `wrong_side` was in DOSED_KINDS for one run, and the study measured it out:
#: three variants, `dose_mm_applied` 11.352973 on all three, one poses_sha256
#: between them. The duplicate guard caught it -- which is what the guard is
#: for -- but it cost two routes per board to learn something the source says
#: in one line. Asking any of these three for a dose produces identical boards.
UNDOSED_KINDS = ('wrong_side', 'swap', 'pile')

#: Doses as a fraction of the MAXIMUM FEASIBLE travel for this board and block,
#: not of the board diagonal.
#:
#: The diagonal version was wrong and the first study run proved it in nine
#: rows: on esp_prog every requested dose exceeded the feasible travel, so
#: `perturb` clipped all three to the same value and translate-d0/d1/d2 came
#: back with ONE poses_sha256 between them. Nine rows, four distinct
#: placements -- n-inflation dressed as a sample.
#:
#: So the dose is probed first (one `perturb` call with a huge cap, which
#: reports `max_feasible_dose_mm` even when it clips) and the three doses are
#: fractions of THAT. Every one is achievable by construction, and the fractions
#: are spread far enough apart that a grid snap cannot merge them.
DOSE_FRACTIONS = (0.25, 0.55, 0.90)
#: Ten portfolio candidates rather than four, because the three undosed kinds
#: contribute three variants instead of nine. K stays 20.
PORTFOLIO_INDICES = tuple(range(1, 11))
#: JITTER ONLY, and both exclusions are measured.
#:
#: `swap` is barren without resolved blocks, and `perturb.py`'s swap re-picks
#: the pair itself, so the damage end covers that shape properly.
#:
#: `poses` was in this tuple for one run and produced NO BOARD on any even
#: index: "poses: no rotation variant left at round 0", again at rounds 1-4.
#: It enumerates discrete rotation variants and a board whose parts have few
#: legal orientations exhausts them immediately -- so five of esp_prog's twenty
#: slots came back empty and the effective K was 15, not 20. Losing a quarter of
#: the sample to a strategy that cannot run on the board is not a null result,
#: it is a smaller study.
#:
#: `jitter` is continuous in its rng, so `only=i` gives a distinct candidate for
#: every i. Ten of them.
PORTFOLIO_STRATEGIES = ('jitter',)


def variant_names():
    """The K=20 variant list, in a fixed order, identical for every board."""
    out = ['authored']
    for k in DOSED_KINDS:
        for i, _f in enumerate(DOSE_FRACTIONS):
            out.append(f'perturb-{k}-d{i}')
    out += [f'perturb-{k}' for k in UNDOSED_KINDS]
    out += [f'portfolio-{i}' for i in PORTFOLIO_INDICES]
    return out


VARIANTS = variant_names()
K = len(VARIANTS)


# ---------------------------------------------------------------------------
# the frozen argv
# ---------------------------------------------------------------------------

def route_argv_for(board_file, out_board, json_out):
    """The route command for a board, derived from THE BOARD ALONE.

    No variant, no index, no measurement is in scope here, which is what makes
    "identical argv per board" a property of the code rather than a promise.
    Floors are deliberately NOT passed: with no `--clearance` every net routes
    at its own net-class clearance and the writeback preserves it, so all of a
    board's variants are graded on the same terms without this tool inventing
    a number (CLAUDE.md's `--clearance` ceiling rules).
    """
    return [sys.executable, '-X', 'utf8', ROUTE_PY, board_file,
            '--output', out_board, '--json-out', json_out]


def argv_signature(board_file):
    """The hash of the argv SHAPE, with the per-variant paths blanked out."""
    argv = route_argv_for(board_file, '<OUT>', '<JSON>')
    argv = [os.path.basename(a) if a.endswith('.py') else a for a in argv]
    argv[0] = 'python'
    return hashlib.sha256(json.dumps(argv).encode()).hexdigest(), argv


def freeze_argv(board_key, board_file, out_dir):
    """Write (or check) ARGV.json. Refuses a changed argv mid-study."""
    d = os.path.join(out_dir, board_key)
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, 'ARGV.json')
    sig, argv = argv_signature(board_file)
    if os.path.isfile(path):
        try:
            old = json.load(open(path, encoding='utf-8'))
        except ValueError:
            # A PARTIAL READ, not a corrupt file. Four tasks on one board
            # freeze the argv concurrently, and the first study run caught a
            # reader mid-write: the task died on a JSONDecodeError that read
            # like a study failure and was a file-system race. The write below
            # is atomic now; this arm covers a file left behind by the
            # non-atomic version and by any future writer that is not.
            old = {'argv_sha': sig, 'argv': argv}
        if old.get('argv_sha') != sig:
            raise SystemExit(
                f'REFUSING: {board_key} was frozen at argv_sha '
                f'{old.get("argv_sha")[:12]} and this run would use '
                f'{sig[:12]}.\n  frozen: {old.get("argv")}\n  now:    {argv}\n'
                f'Identical argv per board is the study\'s control. Delete '
                f'{path} and re-run the WHOLE board, or keep the argv.')
        return old['argv_sha']
    # ATOMIC: write beside it and rename, so a concurrent reader sees either
    # the whole file or no file, never half of one. os.replace is atomic on
    # both POSIX and Windows.
    tmp = f'{path}.{os.getpid()}.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump({'board': board_file, 'argv_sha': sig, 'argv': argv,
                   'frozen_at': 'before any variant was generated'}, f,
                  indent=1)
    try:
        os.replace(tmp, path)
    except OSError:
        # WINDOWS refuses os.replace while another process holds the
        # destination open -- `PermissionError: [WinError 5]`, which killed a
        # task on the very next run after the atomic write was introduced. A
        # sibling task freezing the same board simply got there first. That is
        # not a failure: the file it wrote carries the same argv, because
        # `argv_signature` is a function of the board alone. Verify and move
        # on; only a DISAGREEING file is an error, and the branch above
        # already raises on that.
        try:
            os.remove(tmp)
        except OSError:
            pass
        if not os.path.isfile(path):
            raise
        try:
            other = json.load(open(path, encoding='utf-8'))
        except ValueError:
            return sig
        if other.get('argv_sha') != sig:
            raise SystemExit(
                f'REFUSING: {board_key} was frozen concurrently at argv_sha '
                f'{str(other.get("argv_sha"))[:12]} and this task would use '
                f'{sig[:12]}')
    return sig


# ---------------------------------------------------------------------------
# variant generation -- returns a PATH and nothing else
# ---------------------------------------------------------------------------

def _board_diagonal(board_file):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board_file)
    b = getattr(pcb.board_info, 'board_bounds', None)
    if not b:
        return 100.0
    import math
    return math.hypot(b[2] - b[0], b[3] - b[1])


def generate_variant(board_file, variant, work_dir, seed=0):
    """Write the variant's board and return (path, recipe, note).

    Returns a PATH. It deliberately returns no state object: the predictor
    function takes a path and nothing else, so there is no way for the
    optimizer's live model to reach the measurement.
    """
    os.makedirs(work_dir, exist_ok=True)
    if variant == 'authored':
        return board_file, {'generator': 'authored'}, ''

    if variant.startswith('perturb-'):
        from placement import perturb as P
        parts = variant.split('-')
        kind = parts[1]
        di = int(parts[2][1:]) if len(parts) > 2 else None
        frac = DOSE_FRACTIONS[di] if di is not None else None
        if frac is None:
            # An undosed kind. Its damage is a function of the board alone, so
            # asking for a dose would only invite three identical boards.
            dose = _board_diagonal(board_file)
        else:
            # PROBE the feasible travel first. `perturb` reports
            # `max_feasible_dose_mm` even when it clips, so one throwaway call
            # at a huge cap says how far this block can actually go -- and the
            # three doses are fractions of THAT rather than of the diagonal.
            # Without this every dose on a small board clips to the same value
            # and the three variants are one variant, which is what the first
            # study run measured.
            probe_out = os.path.join(work_dir, '_probe.kicad_pcb')
            probe = P.perturb(board_file, probe_out, kind=kind,
                              dose_mm=10.0 * _board_diagonal(board_file),
                              seed=seed, write_record=False,
                              control_out=os.path.join(work_dir, '_probe.ctl'))
            feasible = probe.get('max_feasible_dose_mm') or 0.0
            if feasible <= 0:
                feasible = probe.get('dose_mm_applied') or 0.0
            dose = frac * feasible
        out = os.path.join(work_dir, f'{variant}.kicad_pcb')
        truth = os.path.join(work_dir, '_truth')
        os.makedirs(truth, exist_ok=True)
        rec = P.perturb(board_file, out, kind=kind, dose_mm=dose, seed=seed,
                        write_record=False,
                        control_out=os.path.join(truth, f'{variant}.ctl.kicad_pcb'))
        if rec.get('status') != 'ok':
            return None, {'generator': 'perturb', 'kind': kind,
                          'dose_fraction': frac,
                          'status': rec.get('status')}, rec.get('reason', '')
        recipe = {'generator': 'perturb', 'kind': kind, 'dose_fraction': frac,
                  'dose_basis': ('fraction of max feasible travel'
                                 if frac is not None else 'undosed kind'),
                  'dose_mm_requested': rec.get('dose_mm_requested'),
                  'dose_mm_applied': rec.get('dose_mm_applied'),
                  'clipped': rec.get('clipped'),
                  'max_feasible_dose_mm': rec.get('max_feasible_dose_mm'),
                  'seed': seed, 'block': (rec.get('block') or {}).get('name')}
        return out, recipe, ('dose CLIPPED to the feasible travel'
                             if rec.get('clipped') else '')

    if variant.startswith('portfolio-'):
        from placement import portfolio as PF
        i = int(variant.split('-')[1])
        sub = os.path.join(work_dir, f'pf_{i}')
        res = PF.generate(board_file, sub, seed=seed, n_candidates=i + 1,
                          strategies=PORTFOLIO_STRATEGIES, only=i)
        cands = res.get('candidates') or []
        if not cands or not cands[0].board:
            return None, {'generator': 'portfolio_quench', 'only': i,
                          'seed': seed}, (cands[0].note if cands else 'barren')
        c = cands[0]
        return c.board, {'generator': 'portfolio_quench', 'only': i,
                         'seed': seed, 'strategy': c.strategy,
                         'strategies': list(PORTFOLIO_STRATEGIES),
                         'displacement_rms': round(c.displacement_rms, 4)}, ''
    raise ValueError(f'unknown variant {variant!r}')


def poses_sha(board_file):
    """sha256 over the sorted poses, quantised to 1 nm.

    64 bytes that answer "is this the same placement?" without the board. Two
    variants sharing one is a sampler that produced a duplicate, which is
    n-inflation rather than a sample.
    """
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board_file)
    rows = sorted(
        f'{r}|{round(f.x, 6):.6f}|{round(f.y, 6):.6f}|'
        f'{round(float(f.rotation or 0.0) % 360, 6):.6f}|{f.layer}'
        for r, f in pcb.footprints.items())
    return hashlib.sha256('\n'.join(rows).encode()).hexdigest()


# ---------------------------------------------------------------------------
# predictors -- from the WRITTEN board, by a second parse
# ---------------------------------------------------------------------------

def predictors_for(board_path):
    """Every pre-route number, re-derived from the board on disk.

    Takes a PATH and nothing else. `PlacementModel(exact=True)` is used at the
    weights `render_placement._build_state` uses -- crossing_penalty 10.0,
    halo_base 0.5, halo_coef 0.25, length_weight 1.0 -- because
    `pose_score.make_state`'s 0.15/30.0/0.3 produce a halo that is NOT
    comparable to the one every recorded handoff carries. That knob trap is
    documented in `tests/stress/calibrate_congestion_ratio.py` and it is the
    reason this does not simply call the cheapest available state builder.

    A predictor that raises is recorded null with its exception. Never 0.
    """
    from kicad_parser import parse_kicad_pcb
    from render_placement import PlacementModel

    pred = {k: None for k in PREDICTOR_KEYS}
    gaps = []
    pcb = parse_kicad_pcb(board_path)
    model = PlacementModel(pcb, board_path, exact=True)
    m = dict(model.metrics or {})
    for k in METRIC_KEYS:
        if k in m:
            pred[k] = m[k]
        else:
            gaps.append(f'metrics.{k}')

    # The legality checklist, from THE SAME model, through the very function
    # `render_placement` uses to build its own `checklist` block -- so a study
    # row and a recorded handoff are the same quantity rather than two
    # implementations of one name.
    from render_placement import legality_findings
    try:
        fnd = legality_findings(model)
    except Exception as e:                                      # noqa: BLE001
        fnd = None
        gaps.append(f'legality:{type(e).__name__}: {e}')
    if isinstance(fnd, dict):
        def _n(key):
            v = fnd.get(key)
            if v is None:
                gaps.append(f'legality.{key}')
                return None
            return len(v) if isinstance(v, (list, tuple)) else v
        pred['pad_copper'] = _n('oob_refs_pad_copper')
        pred['courtyard_off_outline'] = _n('oob_refs_courtyard')
        pred['body_overlap_pairs'] = _n('body_overlap_pairs_refs')
        pred['pad_clearance_pairs'] = _n('pad_conflict_pairs_refs')
        # The run-23 rename: the recorded handoffs carry
        # `b_courtyard_overlap_pairs` and the current engine calls the same
        # channel `courtyard_overlap_pairs_refs`. The harvester normalises the
        # document side under the same canonical name.
        pred['courtyard_advisory_pairs'] = _n('courtyard_overlap_pairs_refs')
        pred['courtyard_blocking_pairs'] = _n('courtyard_blocking_pairs_refs')
        pred['cross_side_stacks'] = _n('cross_side_stacks')
        pred['hole_conflicts'] = _n('hole_conflict_pairs_refs')
        pred['courtyard_overlap_mm2'] = fnd.get('courtyard_overlap_mm2')
        if fnd.get('courtyard_census_error'):
            gaps.append(f'legality.courtyard_census_error='
                        f'{fnd["courtyard_census_error"]}')

    # The pocket census (#709). SEPARATE from the placement model on purpose:
    # it is the aggregate question -- how much of the board is EMPTY, and does
    # the part mass sit where the demand does -- which the per-part legality
    # checklist structurally does not ask.
    #
    # #703's comment set the bar and it is the right one: this family has to
    # earn its place against legality counts already at median rho +0.785 on
    # 6/6 boards. Adding the columns is what makes that answerable by the
    # per-board sign test instead of by argument. Note the CHOICE of scalars:
    # the hot ranking is a known constant predictor (rank_stats records an
    # empty ranked list on three of five real boards), and a constant
    # contributes nothing to a sign test; the cold ones exist on every board.
    #
    # A failure here is recorded as a gap with its exception and every key
    # stays None -- never 0, which would be a measurement.
    try:
        import check_pockets as _cp
        _doc, _hot = _cp.pocket_census(pcb, board_path)
        _arr = _doc.get('arrangement') or {}
        _side = (_arr.get('sides') or {}).get(_arr.get('headline_side')) or {}
        _off = (_side.get('offset_frac_span') or [None, None])[0]
        pred['cold_area_frac'] = _doc.get('cold_area_frac')
        pred['cold_regions'] = len(_doc.get('cold_regions') or [])
        _top = (_doc.get('cold_regions') or [{}])[0]
        pred['cold_top_area_mm2'] = _top.get('area_mm2')
        pred['centroid_offset_frac'] = _off
    except Exception as e:                                      # noqa: BLE001
        gaps.append(f'pockets:{type(e).__name__}: {e}')

    for k, v in list(pred.items()):
        if v is None and f'metrics.{k}' not in gaps:
            gaps.append(f'predictor.{k}')
    return pred, sorted(set(gaps))


# ---------------------------------------------------------------------------
# one task -> one row
# ---------------------------------------------------------------------------

def run_task(board_key, board_file, variant, out_dir, seed=0, timeout=3600):
    work = os.path.join(out_dir, board_key, variant)
    os.makedirs(work, exist_ok=True)
    argv_sha = freeze_argv(board_key, board_file, out_dir)

    row = {
        'schema': SCHEMA, 'kind': 'predictor-row',
        'row_id': f'study:{board_key}:{variant}',
        'source': 'study', 'reproducible': True,
        'board_key': board_key, 'variant': variant, 'generator': None,
        'predictor_source': 'reparse',
        'provenance': {
            'input_board': board_file,
            'input_board_sha': sha256_file(os.path.join(ROOT, board_file))
            if not os.path.isabs(board_file) else sha256_file(board_file),
            'seed': seed, 'k_declared': K,
            'measured_git': git_describe(ROOT),
        },
        'route': {'argv': None, 'argv_sha': argv_sha, 'returncode': None,
                  'seconds': None},
        'predictors': {k: None for k in PREDICTOR_KEYS},
        'truth': {'headline': None, 'blocking': None, 'blocking_by': {},
                  'quality': {}},
        'schema_gaps': [], 'schema_aliases_used': {}, 'notes': [],
    }

    t0 = time.time()
    try:
        path, recipe, note = generate_variant(board_file, variant, work, seed)
    except Exception as e:                                      # noqa: BLE001
        row['notes'].append(f'generate failed: {type(e).__name__}: {e}')
        return row
    row['generator'] = recipe.get('generator')
    row['provenance']['recipe'] = recipe
    if note:
        row['notes'].append(note)
    if not path:
        row['notes'].append('the sampler produced no board for this variant')
        return row
    row['provenance']['variant_board_sha'] = sha256_file(path)
    row['provenance']['poses_sha256'] = poses_sha(path)

    try:
        pred, gaps = predictors_for(path)
    except Exception as e:                                      # noqa: BLE001
        row['notes'].append(f'predictors failed: {type(e).__name__}: {e}')
        return row
    row['predictors'] = pred
    row['schema_gaps'] = gaps

    routed = os.path.join(work, 'routed.kicad_pcb')
    rjson = os.path.join(work, 'route.json')
    argv = route_argv_for(
        path if os.path.isabs(path) else os.path.join(ROOT, path),
        routed, rjson)
    row['route']['argv'] = [os.path.basename(a) if a.endswith('.py') else a
                            for a in argv]
    r0 = time.time()
    try:
        p = subprocess.run(argv, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', cwd=ROOT,
                           timeout=timeout)
        row['route']['returncode'] = p.returncode
    except subprocess.TimeoutExpired:
        row['route']['returncode'] = -9
        row['notes'].append(f'route TIMED OUT after {timeout}s')
        row['route']['seconds'] = round(time.time() - r0, 1)
        return row
    row['route']['seconds'] = round(time.time() - r0, 1)
    if os.path.isfile(rjson):
        try:
            js = json.load(open(rjson, encoding='utf-8'))
            row['route']['min_clearance_used'] = js.get('min_clearance_used')
            row['route']['failed_single'] = len(js.get('failed_single') or [])
            row['route']['open_single'] = len(js.get('open_single') or [])
        except Exception:                                       # noqa: BLE001
            pass
    if not os.path.isfile(routed):
        row['notes'].append('route wrote no board')
        return row

    sjson = os.path.join(work, 'score.json')
    sp = subprocess.run(
        [sys.executable, '-X', 'utf8', BOARD_SCORE, routed, '--json', sjson,
         '--label', row['row_id'], '-q'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT, timeout=timeout)
    if os.path.isfile(sjson):
        s = json.load(open(sjson, encoding='utf-8'))
        by = s.get('blocking_by') or {}
        q = s.get('quality') or {}
        row['truth'] = {
            'headline': s.get('blocking'), 'blocking': s.get('blocking'),
            'blocking_by': {k: by.get(k) for k in TRUTH_BY_KEYS},
            'quality': {'vias': q.get('vias'), 'copper_mm': q.get('copper_mm'),
                        'segments': q.get('segments')},
            'ungraded': s.get('ungraded'),
            'routed_board_sha': s.get('board_sha'),
        }
        if s.get('blocking') is None:
            # board_score's vacuity rule. None is "a component that was asked
            # for could not run", never zero.
            row['notes'].append(
                f'blocking is None (ungraded: {s.get("ungraded")}) -- this row '
                f'is excluded from every statistic, not counted as 0')
    else:
        row['notes'].append(f'board_score wrote no json (exit {sp.returncode})')
    row['provenance']['total_seconds'] = round(time.time() - t0, 1)
    return row


# ---------------------------------------------------------------------------
# aggregation
# ---------------------------------------------------------------------------

DEPENDENTS = (('blocking', 'headline'), ('vias', None))


def _truth_col(row, dep):
    if dep == 'headline':
        return row['truth'].get('headline')
    return (row['truth'].get('quality') or {}).get('vias')


def drop_duplicate_placements(rows):
    """(kept, dropped) -- two variants with one poses_sha256 are ONE sample.

    This is n-inflation, and it is not hypothetical: the first study run
    produced nine esp_prog rows carrying four distinct placements, because
    `perturb` clips a requested dose to the feasible travel and three doses
    clipped to the same value. Ranking those nine would have counted one
    placement three times and manufactured ties that no board has.

    The generator side is fixed (doses are fractions of the FEASIBLE travel
    now, and the dose-independent kinds contribute one variant each), but the
    guard stays: a sampler that silently produces a duplicate must not be able
    to inflate a correlation, whatever the reason. The FIRST variant in the
    declared order wins, so the choice does not depend on completion order.
    """
    order = {v: i for i, v in enumerate(VARIANTS)}
    kept, dropped, seen = [], [], {}
    for r in sorted(rows, key=lambda r: (r.get('board_key', ''),
                                         order.get(r.get('variant'), 999))):
        ps = (r.get('provenance') or {}).get('poses_sha256')
        key = (r.get('board_key'), ps)
        if ps and key in seen:
            dropped.append({'row_id': r['row_id'], 'same_as': seen[key],
                            'poses_sha256': ps})
            continue
        if ps:
            seen[key] = r['row_id']
        kept.append(r)
    return kept, dropped


def aggregate(rows, include_quench=True):
    """Per-board rho for every predictor, then a sign test across boards."""
    rows = [r for r in rows if r.get('source') == 'study']
    if not include_quench:
        rows = [r for r in rows if r.get('generator') != 'portfolio_quench']
    rows, duplicates = drop_duplicate_placements(rows)
    groups = rs.per_board(rows)

    # The frozen-argv control, checked on the DATA and not only at write time.
    argv_bad = {b: sorted({r['route'].get('argv_sha') for r in rr})
                for b, rr in groups.items()
                if len({r['route'].get('argv_sha') for r in rr}) > 1}

    out = {'boards': {}, 'predictors': {}, 'argv_disagreement': argv_bad,
           'include_quench': include_quench, 'duplicates': duplicates}
    for b, rr in sorted(groups.items()):
        out['boards'][b] = {
            'k': len(rr),
            'classification': rs.classify_board(rr, 'headline'),
            'blocking_values': sorted(
                {r['truth'].get('headline') for r in rr
                 if r['truth'].get('headline') is not None}),
            'excluded': [r['variant'] for r in rr
                         if r['truth'].get('headline') is None],
        }
    for name, dep_key in DEPENDENTS:
        per_pred = {}
        for pred in PREDICTOR_KEYS:
            by_board = {}
            for b, rr in sorted(groups.items()):
                if dep_key == 'headline':
                    br = rs.board_rho(rr, pred, 'headline')
                else:
                    sub = [{'board_key': b, 'predictors': r['predictors'],
                            'truth': {'v': _truth_col(r, 'vias')}} for r in rr]
                    br = rs.board_rho(sub, pred, 'v')
                by_board[b] = br
            per_pred[pred] = {
                'by_board': {b: br.as_dict() for b, br in by_board.items()},
                'sign_test': rs.sign_test(by_board),
            }
        out['predictors'][name] = per_pred
    return out


def report(agg, dep='blocking', top=None):
    lines = []
    lines.append('=' * 78)
    lines.append(f'PREDICTORS vs {dep.upper()} -- Spearman WITHIN each board, '
                 f'never pooled')
    lines.append('=' * 78)
    if agg['argv_disagreement']:
        lines.append('REFUSED -- these boards carry more than one argv_sha, so '
                     'their variants were not routed on the same terms:')
        for b, shas in agg['argv_disagreement'].items():
            lines.append(f'  {b}: {shas}')
        return lines
    lines.append(f"portfolio_quench rows "
                 f"{'INCLUDED' if agg['include_quench'] else 'EXCLUDED'}")
    if agg.get('duplicates'):
        lines.append(f"{len(agg['duplicates'])} DUPLICATE placement(s) dropped "
                     f"-- two variants that produced one board are one sample:")
        for d in agg['duplicates'][:8]:
            lines.append(f"    {d['row_id']} == {d['same_as']}")
    lines.append('')
    lines.append('boards:')
    for b, info in agg['boards'].items():
        lines.append(f"  {b:30s} K={info['k']:<3d} {info['classification']:11s} "
                     f"blocking values {info['blocking_values'][:6]}"
                     + (f"  ({len(info['excluded'])} row(s) excluded: "
                        f"{', '.join(info['excluded'][:4])})"
                        if info['excluded'] else ''))
    lines.append('')
    preds = agg['predictors'][dep]
    order = sorted(preds, key=lambda k: -abs(
        preds[k]['sign_test'].get('median_rho') or 0.0))
    if top:
        order = order[:top]
    for pred in order:
        st = preds[pred]['sign_test']
        lines.append(f'  {pred}')
        for b, d in preds[pred]['by_board'].items():
            lines.append(f'      {b:30s} {d["display"]}')
        lines += ['    ' + ln for ln in rs.format_sign_test('across boards', st)]
        lines.append('')
    return lines


def shuffle_control(rows, n=200, seed=12345):
    """How often does a predictor with NO signal pass our own acceptance rule?

    Truth is permuted WITHIN each board, so board size, K and the predictor
    columns are all preserved and only the pairing is destroyed. If a shuffled
    run passes the >= N-1 sign rule at any appreciable rate, the rule is not
    honest and no amount of prose fixes that. Free over --from-rows.
    """
    import random
    rows = [r for r in rows if r.get('source') == 'study']
    # DEDUPE FIRST, exactly as `aggregate` does. An adversarial review caught
    # this running on a different sample than the verdicts it is supposed to
    # calibrate: watchy entered the null at K=19 with six tied duplicates while
    # its real rho used K=13, and the resulting rates moved by up to 5.5 points
    # in both directions. A control measured on a sample the study did not use
    # is not a control.
    rows, _dropped = drop_duplicate_placements(rows)
    groups = rs.per_board(rows)
    rng = random.Random(seed)
    passes = {p: 0 for p in PREDICTOR_KEYS}
    trials = 0
    for _ in range(n):
        shuffled = {}
        for b, rr in groups.items():
            truths = [r['truth'].get('headline') for r in rr]
            rng.shuffle(truths)
            shuffled[b] = [
                {'board_key': b, 'predictors': r['predictors'],
                 'truth': {'headline': t}} for r, t in zip(rr, truths)]
        trials += 1
        for p in PREDICTOR_KEYS:
            by_board = {b: rs.board_rho(rr, p, 'headline')
                        for b, rr in shuffled.items()}
            if rs.sign_test(by_board)['passes_sign_rule']:
                passes[p] += 1
    return {'trials': trials,
            'rate': {p: round(c / trials, 4) for p, c in sorted(passes.items())}}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def compare_row(row, want):
    """Diff a freshly measured row against a recorded expectation.

    Compares only fields that are a property of the BOARD and the argv, never
    of the run: `routed_board_sha` changes every run because KiCad stamps fresh
    UUIDs into the output, which CLAUDE.md warns about explicitly ("outputs
    carry per-run random UUIDs ... never hash or whole-file-diff .kicad_pcb
    outputs to judge determinism").

    `poses_sha256` rather than the input board's bytes: it is computed from
    PARSED footprint poses, so it is identical on a CRLF and an LF checkout.
    Hashing the raw file is not -- all four board hashes in the withdrawn rows
    file were CRLF hashes, and the gate that checked them was green only on
    Windows.
    """
    bad = []
    for k in ('poses_sha256',):
        got = (row.get('provenance') or {}).get(k)
        if got != want.get(k):
            bad.append(f'provenance.{k}: {got} != {want.get(k)}')
    if (row.get('route') or {}).get('argv_sha') != want.get('argv_sha'):
        bad.append(f'route.argv_sha: {(row.get("route") or {}).get("argv_sha")}'
                   f' != {want.get("argv_sha")}')
    for k, v in (want.get('truth') or {}).items():
        got = (row.get('truth') or {}).get(k)
        if k == 'quality':
            got = {kk: (row['truth'].get('quality') or {}).get(kk)
                   for kk in v}
        if got != v:
            bad.append(f'truth.{k}: {got} != {v}')
    for k, v in (want.get('predictors') or {}).items():
        got = (row.get('predictors') or {}).get(k)
        if isinstance(v, float) or isinstance(got, float):
            if got is None or abs(float(got) - float(v)) > 1e-6:
                bad.append(f'predictors.{k}: {got} != {v}')
        elif got != v:
            bad.append(f'predictors.{k}: {got} != {v}')
    return bad


def load_rows(path):
    """Rows from EITHER a `{rows: [...]}` document or the run's own JSONL.

    The study writes `<out>/rows.jsonl`, one row per line, and both the usage
    text above and `docs/placement-predictors.md` tell the reader to feed that
    file straight back in with `--from-rows`. This function only read the
    document form, so following the documented command crashed with
    `JSONDecodeError: Extra data: line 2 column 1` -- a tool refusing the file
    it had just written. Found when the committed rows file was withdrawn and
    the JSONL became the only path anyone would use.

    A ONE-row JSONL is the trap here: it parses as a lone JSON object, so a
    `.get('rows') or []` reads it as a document with no rows and returns
    NOTHING -- and the caller then blames the file ("carries no study rows")
    and exits 0. Same defect as the crash, only silent. Only a document has
    the `rows` key, so anything else that parses as an object is one row.
    """
    with open(path, encoding='utf-8') as f:
        text = f.read()
    try:
        d = json.loads(text)
    except ValueError:
        pass            # not one JSON value -- JSONL below
    else:
        if isinstance(d, list):
            return d
        if isinstance(d, dict):
            if 'rows' in d:
                return d.get('rows') or []
            return [d] if d else []
    rows = []
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except ValueError as e:
            raise SystemExit(f'{path}:{n} is neither a rows document nor a '
                             f'JSONL row: {e}')
    return rows


def read_jsonl(path):
    out = []
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line:
                    out.append(json.loads(line))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(ROOT, 'wk', '703'),
                    help='work dir (gitignored)')
    ap.add_argument('--boards', nargs='+', default=None,
                    help='NARROW the study to these board keys (never widen)')
    ap.add_argument('--variants', nargs='+', default=None)
    ap.add_argument('-k', type=int, default=None,
                    help='first K variants only (a smoke run, not a study)')
    ap.add_argument('-j', type=int, default=1, help='parallel tasks')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--route-timeout', type=int, default=3600)
    ap.add_argument('--task', default=None, metavar='BOARD:VARIANT',
                    help='run ONE task and print its row as json')
    ap.add_argument('--plan', action='store_true')
    ap.add_argument('--calibrate', action='store_true')
    ap.add_argument('--from-rows', default=None, metavar='ROWS_JSON')
    ap.add_argument('--shuffle-control', type=int, default=0, metavar='N')
    ap.add_argument('--verify-row', default=None, metavar='BOARD:VARIANT',
                    help='regenerate ONE variant, re-measure it, and diff '
                         'against the expectation recorded in '
                         'tests/test_703_predictor_regen.py. Exits non-zero on '
                         'any disagreement')
    ap.add_argument('--append', default=None, metavar='ROWS_JSON',
                    help='merge the run\'s rows into this committed file')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args(argv)

    rs._self_test()

    if a.list:
        for v in VARIANTS:
            print(v)
        return 0

    if a.plan:
        print(f'{len(STUDY_BOARDS)} boards x K={K} = '
              f'{len(STUDY_BOARDS) * K} routes')
        print(f'variants (identical for every board): {", ".join(VARIANTS)}')
        cal = os.path.join(a.out, 'calibration.json')
        if os.path.isfile(cal):
            c = json.load(open(cal, encoding='utf-8'))
            tot = 0.0
            for b in STUDY_BOARDS:
                s = (c.get(b['key']) or {}).get('seconds')
                print(f"  {b['key']:30s} {s if s is not None else '?':>8} s "
                      f"authored -> {K} routes ~ "
                      f"{(s * K / 60.0) if s else float('nan'):.1f} min")
                tot += (s or 0) * K
            print(f'  TOTAL ~ {tot / 3600.0:.1f} h serial, '
                  f'~{tot / 3600.0 / 4:.1f} h at -j 4  (from MEASURED '
                  f'authored routes; damaged boards route slower)')
        else:
            print(f'  no calibration at {cal} -- run --calibrate first. This '
                  f'tool does not estimate a runtime it has not measured.')
        return 0

    if a.from_rows:
        rows = load_rows(a.from_rows)
        study = [r for r in rows if r.get('source') == 'study']
        if not study:
            print(f'{a.from_rows} carries no study rows (only '
                  f'{len(rows)} harvest row(s)). The harvest is one placement '
                  f'per board, which is n=1 per board: no correlation is '
                  f'computable from it, and that is the finding.')
            return 0
        for inc in (True, False):
            agg = aggregate(study, include_quench=inc)
            for dep, _ in DEPENDENTS:
                print('\n'.join(report(agg, dep)))
        if a.shuffle_control:
            sc = shuffle_control(study, a.shuffle_control)
            print(f'\nSHUFFLE CONTROL -- truth permuted WITHIN each board, '
                  f'{sc["trials"]} trials')
            print('  how often a predictor with NO signal passes our own '
                  'sign rule:')
            for p, r in sorted(sc['rate'].items(), key=lambda kv: -kv[1])[:10]:
                print(f'    {p:28s} {r:6.1%}')
        return 0

    if a.calibrate:
        os.makedirs(a.out, exist_ok=True)
        cal_path = os.path.join(a.out, 'calibration.json')
        cal = json.load(open(cal_path, encoding='utf-8')) if os.path.isfile(
            cal_path) else {}
        for b in CALIBRATION_CANDIDATES:
            if a.boards and b['key'] not in a.boards:
                continue
            row = run_task(b['key'], b['file'], 'authored',
                           os.path.join(a.out, 'cal'), a.seed, a.route_timeout)
            cal[b['key']] = {
                'seconds': row['route'].get('seconds'),
                'blocking': row['truth'].get('headline'),
                'vias': (row['truth'].get('quality') or {}).get('vias'),
                'returncode': row['route'].get('returncode'),
            }
            print(f"  {b['key']:30s} {cal[b['key']]}")
            with open(cal_path, 'w', encoding='utf-8') as f:
                json.dump(cal, f, indent=1, sort_keys=True)
        print(f'wrote {cal_path}')
        return 0

    if a.verify_row:
        # THE FLAG THAT DID NOTHING. It was declared, documented in the usage
        # text, and advertised in the pull request as the cheap single-row
        # check -- and `args.verify_row` was read ZERO times, so passing it fell
        # through to a full run and printed "111 task(s) to run". An advertised
        # cheap check that silently starts an 8.8-hour study is worse than a
        # flag that errors. Caught in review by drandyhaas.
        bk, _, variant = a.verify_row.partition(':')
        try:
            from test_703_predictor_regen import EXPECTED
        except ImportError:
            sys.path.insert(0, os.path.join(ROOT, 'tests'))
            from test_703_predictor_regen import EXPECTED
        want = EXPECTED.get(f'{bk}:{variant}')
        if want is None:
            print(f'no recorded expectation for {a.verify_row!r}. Known: '
                  f'{", ".join(sorted(EXPECTED))}', file=sys.stderr)
            return 2
        board = next((b for b in CALIBRATION_CANDIDATES if b['key'] == bk), None)
        if board is None:
            print(f'no such board: {bk}', file=sys.stderr)
            return 2
        row = run_task(bk, board['file'], variant, a.out, a.seed,
                       a.route_timeout)
        bad = compare_row(row, want)
        for line in bad:
            print(f'  DISAGREES  {line}')
        if not bad:
            print(f'  {a.verify_row}: regenerated and matched every recorded '
                  f'field ({row["route"].get("seconds")}s)')
        return 1 if bad else 0

    if a.task:
        bk, _, variant = a.task.partition(':')
        board = next((b for b in CALIBRATION_CANDIDATES if b['key'] == bk), None)
        if board is None:
            print(f'no such board: {bk}', file=sys.stderr)
            return 2
        if variant not in VARIANTS:
            print(f'no such variant: {variant}', file=sys.stderr)
            return 2
        row = run_task(bk, board['file'], variant, a.out, a.seed,
                       a.route_timeout)
        print('ROW_JSON=' + json.dumps(row))
        return 0

    # --- the study itself -------------------------------------------------
    boards = STUDY_BOARDS
    if a.boards:
        unknown = [b for b in a.boards
                   if b not in {x['key'] for x in CALIBRATION_CANDIDATES}]
        if unknown:
            print(f'no such board(s): {unknown}; try --plan', file=sys.stderr)
            return 2
        boards = [b for b in CALIBRATION_CANDIDATES if b['key'] in a.boards]
    variants = a.variants or VARIANTS
    if a.k:
        variants = variants[:a.k]

    os.makedirs(a.out, exist_ok=True)
    jsonl = os.path.join(a.out, 'rows.jsonl')
    # A row whose TASK CRASHED is not a completed measurement, and resuming
    # past it would bake a harness failure into the dataset as a permanent
    # hole. A row that legitimately has no truth (a route timeout, a barren
    # sampler) IS complete: it carries its reason and is excluded downstream.
    done = {r['row_id'] for r in read_jsonl(jsonl)
            if not any('task subprocess exit' in n
                       for n in (r.get('notes') or []))}
    tasks = [(b['key'], b['file'], v) for b in boards for v in variants
             if f'study:{b["key"]}:{v}' not in done]
    # FREEZE EVERY BOARD'S ARGV HERE, serially, before a single task starts.
    # It is the control the whole study rests on, so it should not be
    # established by whichever of four concurrent workers wins a race -- and
    # doing it once in the parent means the per-task freeze finds the file
    # already correct and never contends for it.
    for b in boards:
        freeze_argv(b['key'], b['file'], a.out)
    print(f'{len(tasks)} task(s) to run ({len(done)} already in {jsonl})')
    if a.k or a.boards or a.variants:
        print('NOTE: this is a NARROWED run. A verdict requires the whole '
              'declared table.')

    def _one(t):
        bk, bf, v = t
        p = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.abspath(__file__),
             '--task', f'{bk}:{v}', '--out', a.out, '--seed', str(a.seed),
             '--route-timeout', str(a.route_timeout)],
            capture_output=True, text=True, encoding='utf-8', errors='replace',
            cwd=ROOT)
        for line in (p.stdout or '').splitlines():
            if line.startswith('ROW_JSON='):
                return json.loads(line[len('ROW_JSON='):])
        return {'row_id': f'study:{bk}:{v}', 'source': 'study',
                'board_key': bk, 'variant': v,
                'notes': [f'task subprocess exit {p.returncode}: '
                          f'{(p.stderr or "")[-300:]}'],
                'predictors': {k: None for k in PREDICTOR_KEYS},
                'truth': {'headline': None}, 'route': {}}

    n_done = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, a.j)) as ex:
        for row in ex.map(_one, tasks):
            with open(jsonl, 'a', encoding='utf-8') as f:
                f.write(json.dumps(row) + '\n')
            n_done += 1
            print(f'  PROGRESS {n_done}/{len(tasks)}  {row["row_id"]}  '
                  f'blocking={row["truth"].get("headline")}  '
                  f'{row.get("route", {}).get("seconds")}s')

    rows = read_jsonl(jsonl)
    if a.append:
        doc = (json.load(open(a.append, encoding='utf-8'))
               if os.path.isfile(a.append)
               else {'schema': SCHEMA, 'kind': 'predictor-rows', 'rows': [],
                     'refusals': []})
        keep = [r for r in doc['rows'] if r.get('source') != 'study']
        doc['rows'] = sorted(keep + rows, key=lambda r: r['row_id'])
        doc['predictor_keys'] = list(PREDICTOR_KEYS)
        doc['study_boards'] = [b['key'] for b in STUDY_BOARDS]
        doc['study_variants'] = VARIANTS
        with open(a.append, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
            f.write('\n')
        print(f'merged {len(rows)} study row(s) into {a.append}')
    print('ALL DONE')
    return 0


if __name__ == '__main__':
    sys.exit(main())
