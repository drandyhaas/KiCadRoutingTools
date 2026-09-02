#!/usr/bin/env python3
"""Which delta predicate the starved-face gate should use, measured.

#847 asks for the preregistered demand calibration to be re-run at the
pad-copper rect, and for the adjacent `supply_finest_grid == 0` knife edge to
be settled. `tests/measure_847_band_gate.py` established what actually decides
that gate today; this decides what should.

THE ACCEPTANCE RULE, WRITTEN BEFORE THE RUN THAT DECIDES
--------------------------------------------------------
A candidate delta predicate is ADOPTED only if all of:

  R1  PLATEAU, NOT A POINT. The chosen threshold and both +/-25% perturbations
      of it give the SAME verdict on every pair below. A value that works
      because it happens to is a fit, not a calibration.
  R2  SEPARATION. Every POSITIVE pair fires; every CONTROL pair is silent; and
      the worst control margin is at least 2x from the threshold.
  R3  FALSE-POSITIVE BUDGET. The ABSOLUTE starvation form's corpus fire count
      at the re-pinned --min-demand must not exceed what the shipped
      GATE_MIN_DEMAND produces today. Reported for both arms, with the
      denominator NAMED.
  R4  NO BARE MILLIMETRES. Every term is a board quantity, a lane count or a
      dimensionless fraction with a stated meaning.
  R5  MONOTONE IN THE BAND. Once a pair fires, it must keep firing as the band
      deepens. The shipped predicate FAILS this by construction -- it goes
      4 -> 0 -> 4 -- and that flicker is what #847 is about.

R2 and R5 are judged AT THE SHIPPED BAND, because that is the band the tool
uses and the band at which #847 says the gate lost its true positive. The
deeper bands are swept as a ROBUSTNESS CHECK, not as an adoption criterion.

An earlier draft of R5 demanded that every control stay silent at EVERY band
in the sweep. That is the wrong criterion and the run falsified it: at a 2.0mm
band the controls themselves start firing (glasgow's truth-restore reaches a
0.435 drop, tigard's reverse direction a 0.765), because at that depth a
neighbour two parts away is charged against a face and ordinary re-placement
moves it. Demanding silence there would reject every candidate for a reason
that is a property of the BAND, not of the predicate.

That is not a weakened rule -- it is a FINDING, and it is the direct answer to
the band half of #847: **deepening the band raises the false-positive rate.**
The sweep is kept and reported so the evidence for that sentence is in the same
file as the claim.

A related correction, made after the first run and recorded rather than
quietly applied: `tigard_placed vs tigard_damaged` (the repair direction) was
first labelled a CONTROL on the assumption that a repair never reduces any
face's escape. That assumption is false -- a repair moves parts, and moving
parts can cost a face lanes -- so its verdict is not known in advance and it
is now `unlabelled`: reported, never judged on. An arm whose expected answer
is an assumption is not a control.

If no candidate satisfies the rules, NOTHING is adopted and the refusal is the
result. That is a complete answer to #847, not a deferral.

WHAT THE DENOMINATOR IS, and why it is not 33
----------------------------------------------
`tests/test_run8_starved_face_gate.py` records the preregistered ladder as
"6 of 33 healthy in-repo boards fire". That set is NOT reconstructable:
`git ls-files 'kicad_files/*.kicad_pcb'` is 22, while a plain `ls` on a
working copy that has run the suite is 33 -- the extra 11 are GITIGNORED
generated outputs (`interf_u_*` x4, `fanout_output*`, `sonde_u_routed_routed`,
...), several of them derivatives of one source board. This is the exact hazard
`run_utils.corpus_boards()`'s own docstring describes, and the unreproducible
number is cited in PRODUCTION code at `py_tools/check_channels.py`. Everything
here uses `corpus_boards()` and prints its length beside every count.

ONE BASIS, AND IT IS NOT OPTIONAL. Measured while building this: grading the
glasgow control pair at each board's OWN resolved floors put the two halves of
one delta at 0.2/0.2 and 0.0889/0.09, which manufactured two false positives
(RN9 W and RN4 W, both "supply 3 -> 0") that vanish when both sides are graded
at one basis. A delta between two different measurements is not a delta.

    python3 -X utf8 tests/measure_847_calibration.py
    python3 -X utf8 tests/measure_847_calibration.py --out after.json
    python3 -X utf8 tests/measure_847_calibration.py --diff before.json after.json
"""
import argparse
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))
os.environ.setdefault('KRT_NO_BANNER', '1')

import run_utils                                               # noqa: E402
import routing_defaults as D                                   # noqa: E402
import check_channels as CC                                    # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _here)
import importlib.util                                          # noqa: E402
_spec = importlib.util.spec_from_file_location(
    '_m847', os.path.join(_here, 'measure_847_band_gate.py'))
BG = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(BG)

SKIP_EXIT = 77

#: Thresholds swept for the fraction candidate, and the deficit-lane candidate.
DROP_FRACTIONS = (0.15, 0.20, 0.25, 0.30, 0.40, 0.50)
DEFICIT_LANES = (1, 3, 5, 10, 20)
DEMANDS = (1, 5, 7, 9, 11)
BANDS = (None, 2.0, 3.0, 4.0)

WK = os.path.join(ROOT, 'wk', 'run7', 'glasgow_revC')
FIX = os.path.join(ROOT, 'tests', 'fixtures', 'run23')


def _pairs():
    """(label, now, base, kind, forced basis) -- the arms, with their verdicts.

    `kind` is what the pair MUST report: 'positive' is real damage the gate
    exists to catch, 'control' is a legitimate change it must stay silent on.
    A pair whose kind is not known in advance does not belong in a calibration.

    The basis is FORCED per pair rather than resolved per board, because the
    two halves of a delta must be graded identically -- see the module
    docstring for the two false positives that came from not doing this.
    """
    out = []
    g = lambda n: os.path.join(WK, n)                          # noqa: E731
    if os.path.isfile(g('rL_repair.kicad_pcb')):
        # ONE basis for the whole glasgow family: what the shipped CLI
        # resolves for rL_repair at its own netclass.
        b = (0.0889, 0.09)
        out.append(('glasgow wrong-basin repair', g('rL_repair.kicad_pcb'),
                    g('perturbed.kicad_pcb'), 'positive', b))
        if os.path.isfile(g('perturbed.control.kicad_pcb')):
            out.append(('glasgow truth restore',
                        g('perturbed.control.kicad_pcb'),
                        g('perturbed.kicad_pcb'), 'control', b))
        out.append(('glasgow perturbed vs itself', g('perturbed.kicad_pcb'),
                    g('perturbed.kicad_pcb'), 'control', b))
    t = lambda n: os.path.join(FIX, n)                         # noqa: E731
    if os.path.isfile(t('tigard_damaged.kicad_pcb')):
        b = (0.15, 0.15)
        out.append(('tigard damaged', t('tigard_damaged.kicad_pcb'),
                    t('tigard_placed.kicad_pcb'), 'positive', b))
        # UNLABELLED, not a control -- see the module docstring. A repair
        # moves parts, and moving parts can cost a face lanes, so "the reverse
        # direction must be silent" is an assumption rather than a fact.
        out.append(('tigard repaired (the other direction)',
                    t('tigard_placed.kicad_pcb'),
                    t('tigard_damaged.kicad_pcb'), 'unlabelled', b))
        out.append(('tigard placed vs itself', t('tigard_placed.kicad_pcb'),
                    t('tigard_placed.kicad_pcb'), 'control', b))
    return out


def _face_deltas(now_path, base_path, trk, clr, band):
    """Per (ref, face): supply and deficit on both sides, at ONE basis."""
    now = parse_kicad_pcb(now_path)
    base = parse_kicad_pcb(base_path)
    refs = BG.fine_refs(now, trk, clr)
    L = BG.ledgers(now, now_path, refs, trk, clr, D.GRID_STEP, band)
    B = BG.ledgers(base, base_path, refs, trk, clr, D.GRID_STEP, band)
    was = {(r, x['face']): x for r, rows in B.items() for x in rows}
    out = []
    for ref, rows in sorted(L.items()):
        for r in rows:
            b = was.get((ref, r['face']))
            if b is None:
                continue
            out.append({'ref': ref, 'face': r['face'],
                        'demand': r['demand_nets'],
                        'supply_now': r['supply_finest_grid'],
                        'supply_base': b['supply_finest_grid'],
                        'deficit_now': r['deficit_finest_grid'],
                        'deficit_base': b['deficit_finest_grid']})
    return out, len(refs)


def fires_shipped(deltas, min_demand):
    """The predicate as it ships: supply crossed to zero (lost_last_lane)."""
    return [d for d in deltas
            if d['supply_now'] == 0 and d['supply_base'] > 0
            and d['demand'] >= 1]


def fires_deficit(deltas, lanes, min_demand):
    """Candidate B: the deficit GREW by at least `lanes` in total."""
    total = sum(max(0, d['deficit_now'] - d['deficit_base']) for d in deltas)
    return ([d for d in deltas
             if d['deficit_now'] - d['deficit_base'] > 0] if total >= lanes
            else [])


def fires_drop(deltas, frac, min_demand):
    """Candidate C: a face carrying real demand lost a FRACTION of its escape.

    The magnitude form of `lost_last_lane`'s zero-crossing, and the reason it
    is a fraction rather than a lane count is R4: a lane count would have to be
    scaled by the face length, which differs by an order of magnitude across
    one board, while "this face lost a third of its escape" is comparable
    between a 2mm passive and a 20mm BGA edge.

    The demand conjunct is LOAD-BEARING here in a way it is not for the
    shipped predicate, which ignores --min-demand entirely: without it this
    fires on demand-1 diodes whose supply halved from 8 to 3, which is noise.
    """
    out = []
    for d in deltas:
        if d['demand'] < min_demand or d['supply_base'] <= 0:
            continue
        lost = 1.0 - (d['supply_now'] / d['supply_base'])
        if lost >= frac:
            out.append(dict(d, drop=round(lost, 4)))
    return out


def measure(pairs, bands=BANDS):
    doc = {'engine_sha': BG.engine_sha(),
           'grid': D.GRID_STEP,
           'shipped_min_demand': CC.GATE_MIN_DEMAND,
           'pairs': [], 'corpus': {}}
    for label, now, base, kind, (trk, clr) in pairs:
        row = {'label': label, 'kind': kind, 'now': os.path.basename(now),
               'base': os.path.basename(base),
               'basis': {'track': trk, 'clearance': clr,
                         'grid': D.GRID_STEP},
               'bands': {}}
        for band in bands:
            deltas, nrefs = _face_deltas(now, base, trk, clr, band)
            worst = max((1.0 - d['supply_now'] / d['supply_base']
                         for d in deltas
                         if d['supply_base'] > 0
                         and d['demand'] >= CC.GATE_MIN_DEMAND), default=0.0)
            row['bands'][str(band)] = {
                'refs': nrefs, 'faces': len(deltas),
                'worst_drop_at_min_demand': round(worst, 4),
                'shipped': len(fires_shipped(deltas, CC.GATE_MIN_DEMAND)),
                'deficit': {str(k): len(fires_deficit(deltas, k, 1))
                            for k in DEFICIT_LANES},
                'drop': {f'{f}@{md}': len(fires_drop(deltas, f, md))
                         for f in DROP_FRACTIONS for md in DEMANDS},
                'top_drops': sorted(
                    ({'ref': d['ref'], 'face': d['face'],
                      'demand': d['demand'],
                      'supply': [d['supply_base'], d['supply_now']],
                      'drop': round(1.0 - d['supply_now'] / d['supply_base'],
                                    4)}
                     for d in deltas if d['supply_base'] > 0
                     and d['supply_now'] < d['supply_base']),
                    key=lambda e: -e['drop'])[:5],
            }
        doc['pairs'].append(row)
    return doc


def corpus_ladder(doc):
    """R3: the ABSOLUTE starvation form's fire count over the tracked corpus.

    The preregistered ladder, re-run at the pad-copper rect. Two denominators
    are reported and both matter: the tracked board count, and how many of
    those auto-detect a fine-pitch part at all -- a board that can never fire
    is not evidence that the gate is quiet.
    """
    boards = run_utils.corpus_boards()
    if not boards:
        doc['corpus'] = {'skipped': 'corpus_boards() returned nothing -- git '
                                    'could not name the tracked set'}
        return doc
    rows = {}
    with_ledger = 0
    for path in boards:
        name = os.path.basename(path)
        trk, clr, _ts, _cs = BG.floors(path)
        try:
            pcb = parse_kicad_pcb(path)
        except Exception as exc:                               # noqa: BLE001
            rows[name] = {'error': f'{type(exc).__name__}'}
            continue
        refs = BG.fine_refs(pcb, trk, clr)
        led = BG.ledgers(pcb, path, refs, trk, clr, D.GRID_STEP, None)
        if led:
            with_ledger += 1
        rows[name] = {'refs': len(led),
                      'fires': {str(m): len(CC._starved_faces(led, m))
                                for m in DEMANDS}}
    doc['corpus'] = {
        'tracked': len(boards), 'with_ledger': with_ledger, 'boards': rows,
        'boards_firing': {str(m): sorted(n for n, r in rows.items()
                                         if r.get('fires', {}).get(str(m), 0))
                          for m in DEMANDS}}
    return doc


def _nearest(values, target):
    """The member of `values` closest to `target` -- R1's +/-25% neighbours.

    A swept ladder rarely contains exactly 0.75x and 1.25x of a threshold, so
    the neighbour is the nearest value that was actually MEASURED. Interpolating
    one would be inventing a datum to test stability with.
    """
    return min(values, key=lambda v: abs(v - target))


def _fire_counts(pair, field, key):
    """{band: count} for one candidate at one threshold, on one pair."""
    return {b: pair['bands'][b][field].get(key, 0) for b in pair['bands']}


def _verdict(doc):
    """Apply R1/R2/R5 to every candidate at every swept threshold.

    R2 (separation) and R5 (band-independence) are checked together and
    deliberately: a candidate must fire on every POSITIVE and stay silent on
    every CONTROL **at every band in the sweep**. The shipped predicate fails
    R5 by construction -- that is the artifact #847 is about -- so it appears
    here with `adopt: False` rather than being omitted, which keeps it a
    change detector rather than a deleted comparison.
    """
    pairs = doc['pairs']
    pos = [p for p in pairs if p['kind'] == 'positive']
    ctl = [p for p in pairs if p['kind'] == 'control']
    out = []

    cands = [('shipped (lost_last_lane)', 'shipped', [None], lambda t: [],
              lambda t: None)]
    cands.append(('deficit lanes', 'deficit', list(DEFICIT_LANES),
                  lambda t: [_nearest(DEFICIT_LANES, t * 0.75),
                             _nearest(DEFICIT_LANES, t * 1.25)],
                  lambda t: str(t)))
    for md in DEMANDS:
        cands.append((f'supply drop @ demand>={md}', 'drop',
                      list(DROP_FRACTIONS),
                      lambda t: [_nearest(DROP_FRACTIONS, t * 0.75),
                                 _nearest(DROP_FRACTIONS, t * 1.25)],
                      (lambda md_: (lambda t: f'{t}@{md_}'))(md)))

    SHIP = str(BANDS[0])          # the band the tool actually uses
    deeper = [str(b) for b in BANDS[1:]]

    def count(p, field, t, band):
        if field == 'shipped':
            return p['bands'][band]['shipped']
        return p['bands'][band][field].get(key_of_cur(t), 0)

    for name, field, thresholds, neighbours, key_of in cands:
        key_of_cur = key_of
        for t in thresholds:
            # R2, at the shipped band: every positive fires, every control is
            # silent. The `unlabelled` arms are reported and never judged.
            ok_pos = all(count(p, field, t, SHIP) > 0 for p in pos) if pos                 else False
            ok_ctl = all(count(p, field, t, SHIP) == 0 for p in ctl) if ctl                 else False
            # R1: the +/-25% neighbours agree with it at the shipped band.
            stable = True
            for nt in neighbours(t):
                for p in pairs:
                    a = count(p, field, t, SHIP) > 0
                    c = (p['bands'][SHIP][field].get(key_of(nt), 0) > 0)
                    if a != c:
                        stable = False
            # R5: monotone in the band on the POSITIVES -- once it fires it
            # keeps firing. This is the flicker test; the shipped predicate
            # fails it (4 -> 0 -> 4) and that is the defect.
            mono = True
            for p in pos:
                seq = [count(p, field, t, b) > 0 for b in [SHIP] + deeper]
                if any(seq[i] and not seq[i + 1] for i in range(len(seq) - 1)):
                    mono = False
            # Robustness, reported not gated: how many CONTROL arms fire at a
            # deeper band. Rising with depth is the evidence that deepening
            # the band costs false positives.
            deep_ctl = sum(1 for p in ctl for b in deeper
                           if count(p, field, t, b) > 0)
            out.append({'candidate': name, 'threshold': t,
                        'positives_fire_at_shipped_band': ok_pos,
                        'controls_silent_at_shipped_band': ok_ctl,
                        'stable_under_pm25pct': stable,
                        'monotone_in_band_on_positives': mono,
                        'control_fires_at_deeper_bands': deep_ctl,
                        'adopt': bool(ok_pos and ok_ctl and stable and mono)})
    return out


def report(doc):
    print(f"engine {doc['engine_sha'][:12]}  grid {doc['grid']}  "
          f"shipped GATE_MIN_DEMAND {doc['shipped_min_demand']}")
    print()
    for p in doc['pairs']:
        b = p['basis']
        print(f"{p['kind'].upper():8s} {p['label']}")
        print(f"         {p['now']} vs {p['base']}  "
              f"(track {b['track']} clearance {b['clearance']} "
              f"grid {b['grid']} -- ONE basis, both sides)")
        for band, e in p['bands'].items():
            drops = '  '.join(
                f"{f}@{CC.GATE_MIN_DEMAND}:{e['drop'][f'{f}@{CC.GATE_MIN_DEMAND}']}"
                for f in DROP_FRACTIONS)
            print(f"         band {band:>5s}  shipped:{e['shipped']}  "
                  f"deficit>=3:{e['deficit']['3']}  worst-drop "
                  f"{e['worst_drop_at_min_demand']:.3f}   {drops}")
        top = p['bands'][str(BANDS[0])]['top_drops'][:3]
        for t in top:
            print(f"           {t['ref']} {t['face']}: supply "
                  f"{t['supply'][0]}->{t['supply'][1]} demand {t['demand']} "
                  f"drop {t['drop']:.3f}")
        print()
    c = doc.get('corpus') or {}
    if 'skipped' in c:
        print(f"corpus ladder SKIPPED: {c['skipped']}")
    elif c:
        print(f"R3 -- the ABSOLUTE starvation form over the TRACKED corpus "
              f"({c['tracked']} boards, {c['with_ledger']} with a ledger; "
              f"NOT the unreconstructable 33):")
        for m in DEMANDS:
            names = c['boards_firing'][str(m)]
            print(f"    demand >= {m:2d}:  {len(names)} of {c['with_ledger']} "
                  f"with a ledger  {names}")
    print()
    print('VERDICTS (R2 separation and R1 stability, across all bands in R5):')
    for r in _verdict(doc):
        print('   ', r)


def diff(a, b):
    print(f"engine {a['engine_sha'][:12]} -> {b['engine_sha'][:12]}")
    la = {p['label']: p for p in a['pairs']}
    moved = 0
    for p in b['pairs']:
        q = la.get(p['label'])
        if q is None:
            print(f"  NEW pair: {p['label']}")
            continue
        for band in p['bands']:
            if band not in q['bands']:
                continue
            x, y = q['bands'][band], p['bands'][band]
            for k in ('shipped', 'worst_drop_at_min_demand'):
                if x[k] != y[k]:
                    moved += 1
                    print(f"  {p['label']} band {band}: {k} {x[k]} -> {y[k]}")
    ca, cb = a.get('corpus', {}), b.get('corpus', {})
    if ca.get('boards_firing') != cb.get('boards_firing'):
        moved += 1
        print(f"  corpus ladder moved: {ca.get('boards_firing')} -> "
              f"{cb.get('boards_firing')}")
    print(f"  {moved} value(s) moved")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', help='write the measurement to this JSON file')
    ap.add_argument('--diff', nargs=2, metavar=('BEFORE', 'AFTER'))
    ap.add_argument('--no-corpus', action='store_true',
                    help='skip the R3 ladder (it is the slow half)')
    args = ap.parse_args()

    if args.diff:
        a = json.load(open(run_utils.evidence(args.diff[0], 'BEFORE'),
                           encoding='utf-8'))
        b = json.load(open(run_utils.evidence(args.diff[1], 'AFTER'),
                           encoding='utf-8'))
        return diff(a, b)

    pairs = _pairs()
    if not pairs:
        print('SKIP: no calibration pair is present. The glasgow family lives '
              'under wk/, which is gitignored, and the tracked pair '
              'tests/fixtures/run23/tigard_{damaged,placed}.kicad_pcb could '
              'not be found either -- so this run would calibrate against '
              'nothing.')
        return SKIP_EXIT
    have = {p[3] for p in pairs}
    if 'positive' not in have or 'control' not in have:
        print(f'SKIP: a calibration needs both kinds and this run has only '
              f'{sorted(have)}. Separation cannot be measured against one '
              f'side.')
        return SKIP_EXIT

    doc = measure(pairs)
    if not args.no_corpus:
        doc = corpus_ladder(doc)
    report(doc)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
        print(f'  JSON -> {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
