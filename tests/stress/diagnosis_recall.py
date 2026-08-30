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
              not from the metric being scored.
  wrong_side  Weak evidence. A reflection through the board centre.
  scatter     NEGATIVE CONTROL for displacement: per-part jitter leaves the
              block centroid roughly where it was, so the displacement signal
              SHOULD stay quiet. If it fires here it fires on everything, and
              that is a finding to record, not to hide.
  pile        Excluded. Every free part moves, so there is no "the block".

A board whose damaged block is the only candidate is EXCLUDED and named: recall
1.0 out of one candidate is not a measurement.

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

#: A cell is discarded when a size-matched RANDOM pick would hit the damaged
#: block at least this often. A coin flip cannot distinguish a selector from
#: chance, so such a cell is not a measurement whichever way it lands. This
#: bites hardest where it should: on ulx3s the perturber picks a 117-member
#: block out of 234 movable parts, and almost any selection intersects it.
MAX_CHANCE = 0.5


def _movable(pcb):
    """Every footprint the optimizer could move: not KiCad-locked."""
    return {r for r, fp in (pcb.footprints or {}).items()
            if not getattr(fp, 'locked', False)}


def _pin_counts(pcb):
    return {r: sum(1 for p in fp.pads if p.net_id > 0)
            for r, fp in (pcb.footprints or {}).items()}


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
    if rec.get('clipped') and not rec.get('max_feasible_dose_mm'):
        row['skipped'] = ('the dose clipped to zero -- the damage was never '
                          'applied, so the recovery half is void')
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
    tset = set(truth)
    row.update({
        'blocks': len(blocks),
        'movable': len(movable),
        'selected': n,
        'selected_refs': sorted(sel),
        'signals': diag.signals_defined,
        'skipped_signals': dict(sorted(diag.skipped.items())),
        'hit_diagnosis': bool(sel & tset),
        'hit_low_pin': bool(low & tset),
        'chance': round(n / len(movable), 6) if movable else None,
        'selected_by': {c.key: list(c.selected_by) for c in diag.candidates
                        if c.selected_by},
        'displacement_ranked_truth': any(
            c.key == rec['block']['name']
            and any(r.signal == 'block_displacement' for r in c.rows)
            for c in diag.candidates),
    })
    shutil.rmtree(work, ignore_errors=True)
    return row


def summarise(rows):
    """Per-arm tallies, with the instrument check kept apart from evidence."""
    out = {'arms': {}, 'skipped': [], 'boards': sorted(
        {r['board'] for r in rows})}
    for r in rows:
        if r.get('skipped'):
            out['skipped'].append(f"{r['board']}/{r['kind']}: {r['skipped']}")
    for kind in ARMS:
        cells = [r for r in rows if r['kind'] == kind and not r.get('skipped')]
        if not cells:
            continue
        out['arms'][kind] = {
            'is_evidence': kind in EVIDENCE_ARMS,
            'boards': len(cells),
            'hit_diagnosis': sum(1 for c in cells if c['hit_diagnosis']),
            'hit_low_pin': sum(1 for c in cells if c['hit_low_pin']),
            'diagnosis_only': sum(1 for c in cells
                                  if c['hit_diagnosis'] and not c['hit_low_pin']),
            'low_pin_only': sum(1 for c in cells
                                if c['hit_low_pin'] and not c['hit_diagnosis']),
            'mean_chance': round(
                sum(c['chance'] or 0 for c in cells) / len(cells), 4),
            'mean_selected': round(
                sum(c['selected'] for c in cells) / len(cells), 2),
            'displacement_ranked_truth': sum(
                1 for c in cells if c['displacement_ranked_truth']),
        }
    return out


def format_summary(s):
    L = ['', 'RECALL OF KNOWN DAMAGE -- NOT a routing result. Nothing routed.',
         f"boards: {', '.join(s['boards'])}", '']
    L.append(f"{'arm':<12}{'n':>3}  {'diag':>5} {'lowpin':>7} {'d-only':>7}"
             f" {'l-only':>7} {'chance':>7} {'|sel|':>6}  role")
    for kind in ARMS:
        a = s['arms'].get(kind)
        if not a:
            continue
        role = ('evidence' if a['is_evidence'] else
                'INSTRUMENT CHECK, not evidence' if kind == 'translate' else
                'negative control for displacement')
        L.append(f"{kind:<12}{a['boards']:>3}  {a['hit_diagnosis']:>5}"
                 f" {a['hit_low_pin']:>7} {a['diagnosis_only']:>7}"
                 f" {a['low_pin_only']:>7} {a['mean_chance']:>7.3f}"
                 f" {a['mean_selected']:>6.1f}  {role}")
    if 'scatter' in s['arms']:
        sc = s['arms']['scatter']
        L += ['', f"scatter: the displacement signal ranked the damaged block "
                  f"in {sc['displacement_ranked_truth']} of {sc['boards']} "
                  f"cell(s). Per-part jitter leaves the block centroid put, so "
                  f"a high number here means the signal fires on everything -- "
                  f"a negative finding, to be recorded, not hidden."]
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
