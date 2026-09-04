#!/usr/bin/env python3
"""#831: does a predicate over the board's fill signature separate "the exact
fill will finish in 300 s" from "it will not"? Pins the MEASUREMENT.

`tests/831_fill_timing_census.json` is the corpus census produced by
`tests/stress/fill_timing_census.py` (see the file's `machine`/`date`). This
test re-runs the separation analysis on it and asserts the finding, so the
finding is a change detector rather than a sentence in an issue: if the
census is re-recorded on a machine where the population looks different,
the assertion here is what says so.

The analyser is checked against two SYNTHETIC populations first -- one with
a clean gap, one overlapping -- so a "does not separate" verdict on the real
data cannot be the analyser failing to separate anything at all.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

import fill_timing_census as ftc                    # noqa: E402

CENSUS = os.path.join(ROOT, 'tests', '831_fill_timing_census.json')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}"
          f"{(' -- ' + str(detail)) if detail else ''}")


def _row(zones, pads, area, secs, segs=1000, vias=100, status='ok',
         timeout=1800):
    return {'signature_kind': 'fill', 'status': status, 'timeout_s': timeout,
            'fill_elapsed_s': secs, 'n_zones': zones, 'n_pads': pads,
            'n_footprints': pads // 4, 'bbox_area_mm2': area,
            'zone_outline_area_mm2': area * zones * 0.9,
            'n_segments': segs, 'n_vias': vias, 'file_bytes': 1000}


print("--- analyser NEGATIVE CONTROLS on synthetic populations")
# Clean gap: every slow board has zones*area far above every fast one.
sep = ([_row(1, 200, 1000 + i, 1.0 + i) for i in range(30)]
       + [_row(8, 3000, 50000 + i, 900 + i) for i in range(5)]
       + [_row(9, 3000, 60000, 1800, status='timeout')])
a = ftc.analyse(sep)
p = a['predicates']['zones_x_bbox_area']
check("a clean gap is reported as SEPARATES with zero false refusals",
      p['separates'] and p['false_refusals'] == 0, p)
check("its margin ratio is < 1 (fast_max below the threshold)",
      p['margin_ratio'] is not None and p['margin_ratio'] < 1, p)
check("a timeout row counts as slow at its timeout",
      a['n_slow'] == 6 and a['n_fast'] == 30, (a['n_slow'], a['n_fast']))

# Overlap: the slow boards sit INSIDE the fast population's range.
ovl = ([_row(1 + i % 8, 200 + 100 * i, 1000 + 500 * i, 1.0 + i)
        for i in range(40)]
       + [_row(3, 800, 4000, 600), _row(2, 500, 2500, 1200)])
a = ftc.analyse(ovl)
worst = min(r['false_refusals'] for r in a['predicates'].values()
            if r.get('slow_min') is not None)
check("an overlapping population is reported as OVERLAPS by every predicate",
      all(not r['separates'] for r in a['predicates'].values()
          if r.get('slow_min') is not None) and worst > 0,
      f"best predicate still refuses {worst} fast board(s)")

a = ftc.analyse([_row(1, 200, 1000, 1.0)])
check("no slow boards: every predicate reports slow_min None, no verdict",
      all(r['slow_min'] is None and 'separates' not in r
          for r in a['predicates'].values()))

check("a ('path', realpath) signature row is excluded, not mis-scored",
      ftc.analyse([dict(_row(1, 1, 1, 1.0), signature_kind='path'),
                   _row(1, 200, 1000, 1.0)])['n'] == 1)

print("\n--- the recorded census (the finding)")
if not os.path.isfile(CENSUS):
    check("census file present", False, CENSUS)
else:
    with open(CENSUS) as f:
        census = json.load(f)
    rows = census['rows']
    check("population is the corpus, not a sample of it (>= 700 boards)",
          len(rows) >= 700, len(rows))
    a = ftc.analyse(rows, budget=census['production_timeout_s'])
    secs = sorted((r['timeout_s'] if r['status'] == 'timeout'
                   else r['fill_elapsed_s'])
                  for r in rows if r.get('status') in ('ok', 'timeout')
                  and r.get('signature_kind') == 'fill')
    print(f"  n={a['n']} slow(>= {a['budget_s']}s)={a['n_slow']} "
          f"fast={a['n_fast']} max={secs[-1]}s p99={secs[int(0.99 * (len(secs) - 1))]}s")
    # THE FINDING -- see the module docstring of fill_timing_census.py and
    # RUNBOOK "Exact-fill timing census (#831)" for the table behind it.
    FINDING = census.get('finding', {})
    check("the census records its finding", bool(FINDING), FINDING)
    if FINDING.get('separates') is False:
        check("no board in the corpus reached the 300 s budget "
              "(so there is nothing for a predicate to separate)",
              a['n_slow'] == 0, a['n_slow'])
        check("the slowest board is well under the budget (< 1/2 of it)",
              secs[-1] < a['budget_s'] / 2, secs[-1])
        # A predicate would have to be extrapolated from a rank correlation;
        # record how weak/strong the best one is so a later census can compare.
        best = max(a['predicates'].items(),
                   key=lambda kv: kv[1]['spearman'] or 0)
        print(f"  best rank predictor: {best[0]} spearman={best[1]['spearman']}")
        check("the recorded best predictor matches the recomputed one",
              FINDING.get('best_predictor') == best[0]
              and abs(FINDING.get('best_spearman', 0) - (best[1]['spearman'] or 0)) < 0.01,
              (FINDING.get('best_predictor'), best[0], best[1]['spearman']))
        # The signature's OWN components (the only inputs a predicate over
        # `_fill_cost_key` could use) rank fill time weakly. Pinned as a
        # change detector: a census on which this rises past 0.5 is one
        # where the predicate question deserves re-asking.
        sig_sp = max(a['predicates'][k]['spearman'] for k in (
            'n_zones', 'n_pads', 'bbox_area_mm2', 'zones_x_bbox_area',
            'zones_x_pads', 'zones_x_bbox_area_x_pads'))
        check("no signature component ranks fill time (Spearman <= 0.3)",
              sig_sp <= 0.3, sig_sp)
        # The decisive shape: the slowest board has SIGNATURE TWINS -- boards
        # with every signature component within 2x of its own -- that fill in
        # a small fraction of its time. A threshold on the signature that
        # refused the slowest board would refuse them too.
        slow = max(rows, key=lambda r: r['fill_elapsed_s'])

        def _near(x, y):
            return y / 2 <= x <= y * 2
        twins = [r for r in rows if r is not slow
                 and _near(r['n_zones'], slow['n_zones'])
                 and _near(r['n_pads'], slow['n_pads'])
                 and _near(r['n_footprints'], slow['n_footprints'])
                 and _near(r['bbox_area_mm2'], slow['bbox_area_mm2'])]
        tsecs = sorted(r['fill_elapsed_s'] for r in twins)
        check("the slowest board has >= 10 signature twins within 2x",
              len(twins) >= 10, len(twins))
        check("whose median fill is under 1/8 of the slowest board's",
              tsecs and tsecs[len(tsecs) // 2] < slow['fill_elapsed_s'] / 8,
              f"twins median {tsecs[len(tsecs) // 2] if tsecs else None}s "
              f"vs slowest {slow['fill_elapsed_s']}s")
        check("what separates them is copper the signature does not carry "
              "(segments+vias: slowest > 3x the twins' median)",
              slow['n_segments'] + slow['n_vias'] > 3 * sorted(
                  r['n_segments'] + r['n_vias'] for r in twins)[len(twins) // 2],
              slow['n_segments'] + slow['n_vias'])
    elif FINDING.get('separates') is True:
        name = FINDING['predicate']
        p = a['predicates'][name]
        check(f"predicate {name} separates on the recorded census",
              p['separates'], p)
        check("with the recorded threshold and margin",
              abs(p['threshold'] - FINDING['threshold']) < 1e-6
              and p['margin_ratio'] <= FINDING['margin_ratio'] + 1e-6, p)
    else:
        check("finding says separates True/False", False, FINDING)

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
