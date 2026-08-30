#!/usr/bin/env python3
"""#553: does the mover ranking recover a block somebody broke? MEASURED: NO.

THE RESULT FIRST, BECAUSE IT IS A NULL
---------------------------------------
This study was built to support `--target-select diagnosis` and it does not.
Run as committed, the evidence arm's median DELTA -- the lift on the damaged
board minus the lift the SAME ranking scores on the UNDAMAGED one -- is +0.135
over 8 cells, 4 up and 2 down. `wrong_side`, the second evidence arm, is
+0.000, 2 up and 3 down. That is not a finding. It is recorded rather than
tuned away, and the flag it was meant to support ships default-off with no
efficacy claim either way.

The number that read like a finding was the raw lift on the damaged board, and
two things were wrong with it:

  * NO UNDAMAGED CONTROL. `perturb.pick_block` ranks units by source and size,
    and so, largely, does this ranking -- so on tigard the "damaged" block is
    already ranked #1 by displacement on the PRISTINE board. "Above chance on
    the damaged board" could not be told apart from "always ranks that block
    first". The paired arm exists now and is the only evidence column.
  * SATURATION. Lift has a ceiling of `movable / n`, reached whenever the
    selection contains the whole truth, and most cells sit ON it -- 7 of 8
    `wrong_side` cells. There the lift reports how big the selection was and
    nothing else. `lift_ceiling` travels beside every lift now.

A third defect was in the exclusions rather than the numbers. The dose gate
read `max_feasible_dose_mm`, which is RIGID-TRANSLATE travel that only two of
the four arms clip against, so kit-dev-coldfire's `wrong_side` cell was
excluded as "1.404 mm landed" while its 21 members had moved 111 mm. The gate
now diffs the control against the damaged board and uses what the parts DID --
which is also how three `scatter` cells turned out to have moved nothing at all
(`portfolio.perturb_jitter` skips a part whose incumbent pose is not fully
legal, which on a dense board is every part), so the "negative control" board
was byte-identical to the control on those cells.

WHAT WOULD BE NEEDED, if anyone wants to finish this
-----------------------------------------------------
A null measured against a null that is itself wrong is worth little.
`base_rate` is the exact null for a uniform random pick of n PARTS; this
selector picks whole BLOCKS, and the truth is a block from the same derivation,
so the honest null is a random pick of the same number of candidate blocks.
Measured over 2000 permutations per cell that null runs 0.69 to 3.12 -- biased
in both directions depending on the board -- and under it no cell reached the
95th percentile. Anyone reviving this should build that null in, and should
size the selection well below the truth so the ceiling stops binding.

It does not measure whether the selected set ROUTES better either. Nothing here
routes; see `py_placer/placement/diagnosis.py`.

WHY THERE IS NO `pins` ARM, AND WHAT STANDS IN FOR IT
-----------------------------------------------------
`place_route_loop`'s pin selector is `nets_to_refs(pcb, failed + blockers, ...)`
-- it needs the names of the nets the ROUTER failed, so it cannot be computed
on a board nobody routed. Inventing failed nets would make the control a
function of my own choice of nets.

So the control is the pin filter's ESSENCE rather than its code: the N movable
parts with the FEWEST connected pins, N matched to what the diagnosis selected.
That is exactly the preference `--max-target-pins` encodes and exactly what
#553 says is wrong -- "the part that needs to move is never a passive". It is
named `low_pin` here, never `pins`, so nobody reads it as the shipped selector.

`low_pin` is a WEAK control and the weakness is measured: it reads only pin
counts, which are pose-invariant, so `low_pin(damaged) == low_pin(undamaged)`
on every cell -- it cannot respond to damage at all, where the real selector
starts from the nets the router failed. Most of its membership is decided by
its `(pins, ref)` alphabetical tie-break (88-92% on ulx3s). A damage-aware
pin-style proxy is NOT at chance: an oracle running the shipped `nets_to_refs`
over the truth block's own nets beats the diagnosis on 2 of 8 evidence cells.
Read `low_pin` as a floor, never as "a pin-based selector sits at chance".

THE ARMS, AND WHY ONE OF THEM IS NOT EVIDENCE
----------------------------------------------
  translate   INSTRUMENT CHECK, NOT EVIDENCE. `perturb.block_direction` calls
              `routability.block_displacements` itself, and its own docstring
              says a rigid translate of the unit moves the reported distance by
              exactly the dose. "The displacement signal ranks the translated
              block first" is therefore arithmetic. Run it to prove the wiring
              works; never count it.
  swap        THE EVIDENCE ARM. The direction comes from a partner centroid,
              not from the metric being scored. NOTE it is not size-matched to
              the others: `perturb` re-picks a PAIR of disjoint units for this
              arm, so its truth is bigger and its base rate roughly double
              (0.302 against 0.166 in the committed baseline). The lift
              normalises by each cell's own base rate, so the comparison holds
              -- but the arms are not the same experiment on the same block,
              and reading them as a matched set would be wrong.
  wrong_side  Weak evidence. A reflection through the board centre.
  scatter     Intended as a negative control for displacement -- per-part
              jitter should leave the block centroid roughly where it was.
              MEASURED, it mostly does not perturb at all:
              `portfolio.perturb_jitter` skips a part whose incumbent pose is
              not fully legal, which on a dense board is every part, so three
              of its cells produced a board byte-identical to the control and
              are excluded by name. NOTE also that `perturb.py` calls this arm
              a POSITIVE control for its own purpose (it is the arm a RECOVERY
              run must undo). Both readings are right for their own question;
              the collision is named so nobody has to guess.
  pile        Excluded. Every free part moves, so there is no "the block".

A board whose damaged block is the only candidate is EXCLUDED and named: a
ranking over one candidate made no choice.

THE CONFIGURATION GAP, DISCLOSED
--------------------------------
This study calls `diagnose()` with a `top_k` and NO BUDGET.
`place_route_loop` passes `budget=len(pins_targets)` on every round. That is
material and the module says so: unbudgeted, permuting `SIGNAL_ORDER` changes
only the order of `selected_keys`; under a budget it can change the selected
SET outright. So these numbers describe the ranking, not the ranking as the
loop invokes it. Budgeting the study would need a routed board to supply the
pin count, which is exactly what this script exists to avoid -- so the gap is
recorded rather than closed.

    python3 -X utf8 tests/stress/diagnosis_recall.py --boards esp_prog tigard --out wk/553
    python3 -X utf8 tests/stress/diagnosis_recall.py --from-rows wk/553/rows.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb            # noqa: E402
from placement import diagnosis as D                # noqa: E402
from placement.groups import derive_groups, parse_sources   # noqa: E402

#: `pile` is excluded on purpose -- see the module docstring.
ARMS = ('swap', 'wrong_side', 'scatter', 'translate')

#: Which arms are allowed to count toward a verdict. `translate` is an
#: instrument check and is measured, printed, and NEVER aggregated.
EVIDENCE_ARMS = ('swap', 'wrong_side')

DEFAULT_BOARDS = ('esp_prog', 'splitflap_driver', 'watchy', 'tigard',
                  'sonde_u', 'ulx3s', 'orangecrab_ext_pll',
                  'kit-dev-coldfire-xilinx_5213', 'rp2350_fpga_eensy_prePlane')

DEFAULT_DOSE = 20.0

#: A cell is discarded when NO member of the damaged block actually moved this
#: far, measured by diffing the control against the damaged board. Below a
#: millimetre there is nothing to recover and the recovery half is void.
#:
#: This replaced a gate on `max_feasible_dose_mm`, which was wrong for three of
#: the four arms: that number is the max feasible RIGID-TRANSLATE travel and
#: only `translate`/`wrong_side` clip against it. kit-dev-coldfire's wrong_side
#: cell was excluded as "1.404 mm landed" while its 21 members had in fact
#: moved 111 mm. `perturb`'s own `clipped` flag catches neither case.
MIN_DAMAGE_MM = 1.0

#: WHY THE HEADLINE IS LIFT AND NOT RECALL. The first version of this script
#: asked "does the selection contain a damaged part". It excluded 33 of its 36
#: cells and produced no evidence arm at all, for a structural reason:
#: `perturb.pick_block` picks BIG units -- 117 of ulx3s's 234 movable parts,
#: 20 of watchy's 84 -- so a size-matched random pick hits the truth 84-100% of
#: the time and a "hit" carries no information. The cells where the truth was
#: small were the cells where the dose could not land.
#:
#: LIFT does not saturate: what fraction of the SELECTION is damaged, over the
#: fraction of the board that is damaged. 1.0 is chance. It is defined
#: whatever the truth size, and its null is exact rather than assumed.


def _movable(pcb):
    """Every footprint the optimizer could move: not KiCad-locked."""
    return {r for r, fp in (pcb.footprints or {}).items()
            if not getattr(fp, 'locked', False)}


def _pin_counts(pcb):
    return {r: sum(1 for p in fp.pads if p.net_id > 0)
            for r, fp in (pcb.footprints or {}).items()}


def exact_chance(movable: int, truth: int, n: int) -> float:
    """P(a size-n random pick from `movable` parts hits any of `truth`).

    The hypergeometric complement, computed exactly rather than approximated by
    n/movable -- which is only right for a one-member truth. It matters: the
    perturber picks a 117-member block out of 234 movable parts on ulx3s, where
    almost any selection intersects it and a "hit" carries no information.
    """
    from math import comb
    if not movable or n <= 0 or truth <= 0:
        return 0.0
    n = min(n, movable)
    miss = movable - truth
    if n > miss:
        return 1.0
    return 1.0 - comb(miss, n) / comb(movable, n)


def _low_pin_arm(pcb, movable, n):
    """The pin filter's essence: the n movable parts with the fewest pins.

    Ties broken by reference, so this is a deterministic function of the board
    -- not of dict order.
    """
    pins = _pin_counts(pcb)
    order = sorted(movable, key=lambda r: (pins.get(r, 0), r))
    return set(order[:n])


def run_one(board_path, kind, *, dose, group_by, top_k, workroot):
    """One (board, arm) cell. Returns a row dict, or one with 'skipped'."""
    from placement import perturb as P

    name = os.path.splitext(os.path.basename(board_path))[0]
    work = tempfile.mkdtemp(prefix=f'{name}_{kind}_', dir=workroot)
    truth_dir = os.path.join(workroot, '_truth')
    os.makedirs(truth_dir, exist_ok=True)
    damaged = os.path.join(work, 'damaged.kicad_pcb')
    control = os.path.join(truth_dir, f'{name}_{kind}_control.kicad_pcb')

    row = {'board': name, 'kind': kind, 'dose_mm_requested': dose,
           'group_by': group_by, 'top_k': top_k}
    try:
        rec = P.perturb(board_path, damaged, kind=kind, dose_mm=dose,
                        group_by=group_by, write_record=False,
                        control_out=control)
    except Exception as e:                            # noqa: BLE001
        row['skipped'] = f'perturb failed: {type(e).__name__}: {e}'
        return row

    truth = sorted(rec['block']['members'])
    row.update({'truth_block': rec['block']['name'],
                'truth_source': rec['block']['source'],
                'truth_members': truth,
                'dose_mm_applied': rec.get('max_feasible_dose_mm'),
                'clipped': rec.get('clipped')})
    # THE DAMAGE, MEASURED. Not `max_feasible_dose_mm`: that is the max feasible
    # RIGID-TRANSLATE travel, computed before perturb's kind branch, and only
    # `translate` and `wrong_side` clip against it. `swap` and `scatter` never
    # read the dose at all (`predictor_study.UNDOSED_KINDS` says so in one
    # line), so gating them on it was wrong by up to 79x -- kit-dev-coldfire's
    # wrong_side cell was excluded as "1.404 mm" while its members had actually
    # moved 111 mm. So: diff the control against the damaged board and use what
    # the parts DID.
    ctrl_pcb = parse_kicad_pcb(control)
    pcb = parse_kicad_pcb(damaged)
    moved = {}
    for r in truth:
        a = (ctrl_pcb.footprints or {}).get(r)
        b = (pcb.footprints or {}).get(r)
        if a is not None and b is not None:
            moved[r] = math.hypot(b.x - a.x, b.y - a.y)
    max_move = max(moved.values()) if moved else 0.0
    n_moved = sum(1 for v in moved.values() if v > 1e-6)
    row.update({'max_member_move_mm': round(max_move, 4),
                'members_moved': n_moved,
                'members': len(truth)})
    if max_move < MIN_DAMAGE_MM:
        row['skipped'] = (
            f'the damaged block moved at most {max_move:.3f} mm '
            f'({n_moved} of {len(truth)} members moved at all) -- below '
            f'{MIN_DAMAGE_MM} mm there is no damage to recover, so the '
            f'recovery half is void')
        return row

    blocks = derive_groups(pcb, parse_sources(group_by))
    movable = _movable(pcb)
    if len(blocks) < 2:
        row['skipped'] = (f'{len(blocks)} candidate block(s): a ranking over '
                          f'one candidate made no choice')
        row['blocks'] = len(blocks)
        return row

    def _lift(board_pcb, board_path):
        st = D.make_state(board_pcb, board_path)
        leg = D.legality_defects(board_pcb, pcb_file=board_path)
        dg = D.diagnose(st, board_pcb, derive_groups(board_pcb,
                                                     parse_sources(group_by)),
                        blocker_report=None, legality=leg, top_k=top_k)
        return st, leg, dg

    state, legality, _ = _lift(pcb, damaged)
    # blocker_report is None ON PURPOSE: nothing routed, so the router
    # attributed nothing, and imputing an attribution would invent the
    # evidence. Two of the three signals run; the third reports why it did not.
    diag = D.diagnose(state, pcb, blocks, blocker_report=None,
                      legality=legality, top_k=top_k)
    # THE PAIRED ARM, and the one whose absence voided the first version of
    # this study: the SAME ranking on the UNDAMAGED board, scored against the
    # SAME truth. Without it "the ranking is above chance on the damaged board"
    # cannot be told apart from "the ranking always ranks that block first" --
    # and measured, tigard's truth block is already ranked #1 by displacement
    # on the pristine board. Only the DELTA is evidence.
    _, _, diag0 = _lift(ctrl_pcb, control)

    sel = set(diag.selected)
    n = len(sel)
    low = _low_pin_arm(pcb, movable, n)
    tset = set(truth) & movable
    chance = exact_chance(len(movable), len(tset), n)
    base = len(tset) / len(movable) if movable else 0.0
    p_diag = len(sel & tset) / n if n else 0.0
    p_low = len(low & tset) / len(low) if low else 0.0
    row.update({
        'blocks': len(blocks),
        'movable': len(movable),
        'truth_movable': len(tset),
        'selected': n,
        'selected_refs': sorted(sel),
        'signals': diag.signals_defined,
        'skipped_signals': dict(sorted(diag.skipped.items())),
        'clearance': legality.get('clearance'),
        'clearance_source': legality.get('clearance_source'),
        'hit_diagnosis': bool(sel & tset),
        'hit_low_pin': bool(low & tset),
        'chance': round(chance, 6),
        'base_rate': round(base, 6),
        'precision_diagnosis': round(p_diag, 6),
        'precision_low_pin': round(p_low, 6),
        'lift_diagnosis': round(p_diag / base, 4) if base else None,
        'lift_low_pin': round(p_low / base, 4) if base else None,
        # THE CEILING. Lift DOES saturate: a selection at least as big as the
        # truth can at best contain all of it, so max lift is movable/n. A cell
        # at 100% of ceiling says only "the selection is this fraction of the
        # board", which is why this column is reported beside every lift.
        'lift_ceiling': round(min(n, len(tset)) / (n * base), 4)
                        if (n and base) else None,
        # The undamaged pairing.
        'selected_undamaged': len(diag0.selected),
        'lift_undamaged': (round((len(set(diag0.selected) & tset)
                                  / len(diag0.selected)) / base, 4)
                           if diag0.selected and base else None),
        'selected_by': {c.key: list(c.selected_by) for c in diag.candidates
                        if c.selected_by},
        'displacement_ranked_truth': any(
            c.key == rec['block']['name']
            and any(r.signal == 'block_displacement' for r in c.rows)
            for c in diag.candidates),
    })
    shutil.rmtree(work, ignore_errors=True)
    if not tset:
        row['skipped'] = ('the damaged block holds no MOVABLE part, so no '
                          'selector could have picked one')
    return row


def _median(vals):
    v = sorted(vals)
    if not v:
        return None
    m = len(v) // 2
    return round(v[m] if len(v) % 2 else (v[m - 1] + v[m]) / 2.0, 4)


def summarise(rows):
    """Per-arm tallies, with the instrument check kept apart from evidence."""
    out = {'arms': {}, 'skipped': [], 'boards': sorted(
        {r['board'] for r in rows})}
    # Per-CELL numbers, not only the per-arm medians. Without these a
    # single-board re-run has nothing to diff against, and the change detector
    # can only check that the baseline is shaped like a baseline -- which is
    # what the first version of it did.
    out['cells'] = {f"{r['board']}/{r['kind']}": {
        k: r[k] for k in ('lift_diagnosis', 'lift_undamaged', 'lift_low_pin',
                          'lift_ceiling', 'base_rate', 'selected',
                          'selected_undamaged', 'movable', 'truth_movable',
                          'max_member_move_mm', 'members_moved',
                          'hit_diagnosis', 'hit_low_pin')}
        for r in rows if not r.get('skipped')}
    for r in rows:
        if r.get('skipped'):
            out['skipped'].append(f"{r['board']}/{r['kind']}: {r['skipped']}")
    for kind in ARMS:
        cells = [r for r in rows if r['kind'] == kind and not r.get('skipped')]
        if not cells:
            continue
        lift_d = [c['lift_diagnosis'] for c in cells
                  if c['lift_diagnosis'] is not None]
        lift_l = [c['lift_low_pin'] for c in cells
                  if c['lift_low_pin'] is not None]
        # THE ONLY EVIDENCE COLUMN. Raw lift on the damaged board is not it:
        # the ranking scores just as well on the pristine board wherever the
        # perturber's own unit_rank picks the block the ranking would have
        # picked anyway. The delta is what damage did.
        delta = [round(c['lift_diagnosis'] - c['lift_undamaged'], 4)
                 for c in cells
                 if c['lift_diagnosis'] is not None
                 and c['lift_undamaged'] is not None]
        ceil_frac = [c['lift_diagnosis'] / c['lift_ceiling']
                     for c in cells if c.get('lift_ceiling')]
        out['arms'][kind] = {
            'is_evidence': kind in EVIDENCE_ARMS,
            'boards': len(cells),
            'board_names': sorted(c['board'] for c in cells),
            # The sign test this repo uses elsewhere: direction per board,
            # never a pooled mean standing in for one.
            'diagnosis_above_chance': sum(1 for v in lift_d if v > 1.0),
            'diagnosis_below_chance': sum(1 for v in lift_d if v < 1.0),
            'delta_positive': sum(1 for v in delta if v > 0),
            'delta_negative': sum(1 for v in delta if v < 0),
            'delta_zero': sum(1 for v in delta if v == 0),
            'median_delta': _median(delta),
            'cells_at_ceiling': sum(1 for f in ceil_frac if f >= 0.999),
            'beats_low_pin': sum(1 for c in cells
                                 if (c['lift_diagnosis'] or 0)
                                 > (c['lift_low_pin'] or 0)),
            'loses_to_low_pin': sum(1 for c in cells
                                    if (c['lift_diagnosis'] or 0)
                                    < (c['lift_low_pin'] or 0)),
            'median_lift_diagnosis': _median(lift_d),
            'median_lift_low_pin': _median(lift_l),
            'mean_base_rate': round(
                sum(c['base_rate'] for c in cells) / len(cells), 4),
            'mean_selected': round(
                sum(c['selected'] for c in cells) / len(cells), 2),
            'displacement_ranked_truth': sum(
                1 for c in cells if c['displacement_ranked_truth']),
        }
    return out


def format_summary(s):
    L = ['', 'RECALL OF KNOWN DAMAGE -- NOT a routing result. Nothing routed.',
         f"boards: {', '.join(s['boards'])}", '']
    L.append('LIFT = (share of the SELECTION that is damaged) / (share of the '
             'BOARD that is damaged).')
    L.append('DELTA = that lift MINUS the same ranking on the UNDAMAGED board. '
             'Delta is the evidence;')
    L.append('raw lift is not: a ranking that always picks that block scores '
             'the same either way.')
    L.append('CEIL counts cells at the metric ceiling (movable/n), where the '
             'lift says only how')
    L.append('big the selection was. Direction is counted PER BOARD, never '
             'pooled into one mean.')
    L.append('')
    L.append(f"{'arm':<12}{'n':>3} {'d+':>3} {'d-':>3} {'med.delta':>10}"
             f" {'med.lift':>9} {'med.L':>7} {'ceil':>5} {'base':>6}"
             f" {'|sel|':>6}  role")
    for kind in ARMS:
        a = s['arms'].get(kind)
        if not a:
            continue
        role = ('evidence' if a['is_evidence'] else
                'INSTRUMENT CHECK, not evidence' if kind == 'translate' else
                'negative control for displacement')
        md = a['median_lift_diagnosis']
        ml = a['median_lift_low_pin']
        dl = a['median_delta']
        L.append(f"{kind:<12}{a['boards']:>3} {a['delta_positive']:>3}"
                 f" {a['delta_negative']:>3}"
                 f" {(f'{dl:+.3f}' if dl is not None else '-'):>10}"
                 f" {(f'{md:.2f}' if md is not None else '-'):>9}"
                 f" {(f'{ml:.2f}' if ml is not None else '-'):>7}"
                 f" {a['cells_at_ceiling']:>5}"
                 f" {a['mean_base_rate']:>6.3f} {a['mean_selected']:>6.1f}"
                 f"  {role}")
        L.append(f"{'':<12}    boards: {', '.join(a['board_names'])}")
    if 'scatter' in s['arms']:
        sc = s['arms']['scatter']
        L += ['', f"scatter kept {sc['boards']} of its cells: on the rest "
                  f"`portfolio.perturb_jitter` moved NOTHING, because it skips "
                  f"a part whose incumbent pose is not fully legal, which on a "
                  f"dense board is every part. A control that did not perturb "
                  f"is not a control, which is why those cells are excluded by "
                  f"name rather than counted."]
    for note in s['skipped']:
        L.append(f'  EXCLUDED {note}')
    ev = [s['arms'][k] for k in EVIDENCE_ARMS if k in s['arms']]
    strong = any((a.get('median_delta') or 0) > 0.5 for a in ev)
    L += ['',
          'VERDICT: ' + ('' if strong else 'NO EVIDENCE. ')
          + "the evidence arms' median deltas are "
          + ', '.join(f"{k} {s['arms'][k]['median_delta']:+.3f} "
                      f"({s['arms'][k]['delta_positive']}+/"
                      f"{s['arms'][k]['delta_negative']}-)"
                      for k in EVIDENCE_ARMS if k in s['arms']) + '.',
          'DELTA is lift on the damaged board minus lift on the UNDAMAGED one. '
          'A ranking that always',
          'picks that block scores the same on both, which is why the raw '
          'lift column is not evidence.',
          'translate is arithmetic: perturb.block_direction calls the very '
          'metric this arm scores.',
          'No paired routed A/B of pins vs diagnosis exists. Recall is not '
          'efficacy, and this recall is not a finding.']
    return '\n'.join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--boards', nargs='*', default=list(DEFAULT_BOARDS))
    ap.add_argument('--kinds', nargs='*', default=list(ARMS), choices=ARMS)
    ap.add_argument('--dose', type=float, default=DEFAULT_DOSE)
    ap.add_argument('--group-by', default='auto,netprefix,decap',
                    help="group sources for BOTH the perturbation's block pick "
                         "and the diagnosis (default: auto,netprefix,decap -- "
                         "'auto' alone derives nothing on most tracked boards)")
    ap.add_argument('--top-k', type=int, default=D.TOP_K)
    ap.add_argument('--out', default=None, metavar='DIR',
                    help='write rows.jsonl and summary.json here')
    ap.add_argument('--from-rows', default=None, metavar='JSONL',
                    help='re-summarise an existing rows.jsonl and exit')
    a = ap.parse_args()

    if a.from_rows:
        with open(a.from_rows, encoding='utf-8') as f:
            rows = [json.loads(ln) for ln in f if ln.strip()]
        print(format_summary(summarise(rows)))
        return 0

    workroot = tempfile.mkdtemp(prefix='diagnosis_recall_')
    rows = []
    try:
        for name in a.boards:
            path = os.path.join(ROOT, 'kicad_files', f'{name}.kicad_pcb')
            if not os.path.isfile(path):
                print(f'  MISSING {path}', flush=True)
                continue
            for kind in a.kinds:
                print(f'  {name}/{kind} ...', flush=True)
                row = run_one(path, kind, dose=a.dose, group_by=a.group_by,
                              top_k=a.top_k, workroot=workroot)
                rows.append(row)
                print(f'    {json.dumps({k: v for k, v in row.items() if k not in ("selected_refs", "truth_members", "selected_by", "skipped_signals")})}',
                      flush=True)
    finally:
        shutil.rmtree(workroot, ignore_errors=True)

    s = summarise(rows)
    print(format_summary(s))
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, 'rows.jsonl'), 'w', encoding='utf-8') as f:
            for r in rows:
                f.write(json.dumps(r, sort_keys=True) + '\n')
        with open(os.path.join(a.out, 'summary.json'), 'w',
                  encoding='utf-8') as f:
            json.dump(s, f, indent=1, sort_keys=True)
        print(f'\nWrote {a.out}/rows.jsonl and {a.out}/summary.json')
    return 0


if __name__ == '__main__':
    sys.exit(main())
