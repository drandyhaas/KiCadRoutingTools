#!/usr/bin/env python3
"""#553: does the mover ranking recover a block somebody broke? No routing.

THE CLAIM THIS CAN SUPPORT, AND THE ONE IT CANNOT
-------------------------------------------------
It measures RECALL of known damage: displace a block by a known dose with
`placement/perturb.py`, hand the damaged board to `placement.diagnosis`, and
ask whether the selected set contains a member of the block that was broken.

It does NOT measure whether the selected set ROUTES better. Nothing here
routes. `--target-select diagnosis` ships with no efficacy claim, and no number
this script prints is one; see `py_placer/placement/diagnosis.py`.

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

The second control is CHANCE: N / |movable|, the probability that a size-matched
random pick contains a damaged member. A selector that picks nearly everything
has near-perfect recall and no value, so recall is never reported without it.

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
  scatter     NEGATIVE CONTROL for displacement HERE. `perturb.py` calls the
              same arm a POSITIVE control, for its own purpose -- it is the
              arm a RECOVERY run must undo. Both readings are right for their
              own question and the collision is named so nobody has to guess:
              per-part jitter leaves the
              block centroid roughly where it was, so the displacement signal
              SHOULD stay quiet. If it fires here it fires on everything, and
              that is a finding to record, not to hide.
  pile        Excluded. Every free part moves, so there is no "the block".

A board whose damaged block is the only candidate is EXCLUDED and named: recall
1.0 out of one candidate is not a measurement.

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

#: A cell is discarded when the dose that actually landed is below this
#: fraction of the dose asked for. NOT a tuned constant: it is the line between
#: "a block was displaced" and "a block was nudged", and the recovery half of
#: an experiment whose damage never happened is void.
#:
#: `perturb`'s own `clipped` flag does NOT catch this -- measured on esp_prog,
#: a 20 mm request landed 0.316 mm and reported `clipped: false`. So the ratio
#: is computed here rather than trusted from the record.
MIN_DOSE_FRACTION = 0.25

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
    applied = rec.get('max_feasible_dose_mm') or 0.0
    if dose and applied < MIN_DOSE_FRACTION * dose:
        # NOT read off `clipped`: measured on esp_prog, a 20 mm request landed
        # 0.316 mm and the record still said `clipped: false`.
        row['skipped'] = (
            f'the dose that landed was {applied:.3f} mm of {dose:g} mm asked '
            f'for ({applied / dose:.1%}); the damage was never applied, so the '
            f'recovery half is void')
        return row

    pcb = parse_kicad_pcb(damaged)
    blocks = derive_groups(pcb, parse_sources(group_by))
    movable = _movable(pcb)
    if len(blocks) < 2:
        row['skipped'] = (f'{len(blocks)} candidate block(s): a ranking over '
                          f'one candidate made no choice')
        row['blocks'] = len(blocks)
        return row

    state = D.make_state(pcb, damaged)
    legality = D.legality_defects(pcb, pcb_file=damaged)
    # blocker_report is None ON PURPOSE: nothing routed, so the router
    # attributed nothing, and imputing an attribution would invent the
    # evidence. Two of the three signals run; the third reports why it did not.
    diag = D.diagnose(state, pcb, blocks, blocker_report=None,
                      legality=legality, top_k=top_k)

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
        k: r[k] for k in ('lift_diagnosis', 'lift_low_pin', 'base_rate',
                          'selected', 'movable', 'truth_movable',
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
        out['arms'][kind] = {
            'is_evidence': kind in EVIDENCE_ARMS,
            'boards': len(cells),
            'board_names': sorted(c['board'] for c in cells),
            # The sign test this repo uses elsewhere: direction per board,
            # never a pooled mean standing in for one.
            'diagnosis_above_chance': sum(1 for v in lift_d if v > 1.0),
            'diagnosis_below_chance': sum(1 for v in lift_d if v < 1.0),
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
    L.append('1.0 is chance. Direction is counted PER BOARD, never pooled '
             'into one mean.')
    L.append('')
    L.append(f"{'arm':<12}{'n':>3} {'above':>6} {'below':>6} {'med.D':>7}"
             f" {'med.L':>7} {'D>L':>4} {'D<L':>4} {'base':>6} {'|sel|':>6}"
             f"  role")
    for kind in ARMS:
        a = s['arms'].get(kind)
        if not a:
            continue
        role = ('evidence' if a['is_evidence'] else
                'INSTRUMENT CHECK, not evidence' if kind == 'translate' else
                'negative control for displacement')
        md = a['median_lift_diagnosis']
        ml = a['median_lift_low_pin']
        L.append(f"{kind:<12}{a['boards']:>3} {a['diagnosis_above_chance']:>6}"
                 f" {a['diagnosis_below_chance']:>6}"
                 f" {(f'{md:.2f}' if md is not None else '-'):>7}"
                 f" {(f'{ml:.2f}' if ml is not None else '-'):>7}"
                 f" {a['beats_low_pin']:>4} {a['loses_to_low_pin']:>4}"
                 f" {a['mean_base_rate']:>6.3f} {a['mean_selected']:>6.1f}"
                 f"  {role}")
        L.append(f"{'':<12}    boards: {', '.join(a['board_names'])}")
    if 'scatter' in s['arms']:
        sc = s['arms']['scatter']
        L += ['', f"scatter is the control that makes the first row mean "
                  f"something: per-part jitter leaves the block centroid where "
                  f"it was, so the selector SHOULD sit at chance here. Median "
                  f"lift {sc['median_lift_diagnosis']}, "
                  f"{sc['diagnosis_above_chance']} board(s) above and "
                  f"{sc['diagnosis_below_chance']} below. The damaged block "
                  f"still CARRIES a displacement row in "
                  f"{sc['displacement_ranked_truth']} of {sc['boards']} cells "
                  f"-- being ranked at all is not the same as leading, and "
                  f"only the lift says which."]
    for note in s['skipped']:
        L.append(f'  EXCLUDED {note}')
    L += ['', 'translate is arithmetic: perturb.block_direction calls the very '
              'metric this arm scores, so passing it proves the wiring, not '
              'the signal.',
          'No paired routed A/B of pins vs diagnosis exists. Recall is not '
          'efficacy.']
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
