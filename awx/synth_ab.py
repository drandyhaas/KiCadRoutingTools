#!/usr/bin/env python3
"""synth_ab.py A B -- two runs of the generated cases, case by case: synth_handoff.py's (OUTDIR/handoff.tsv) or
synth_layers.py's (OUTDIR/layers.tsv) table, or several OUTDIRs of one run joined with commas (a run split by layer
count). A case is its tag, and its layer count in a synth_layers table.

Prints each case whose grade differs -- verdict, round, vias, connected, drc, open, and the handoff's gap, turn, hold
and static where the table has them, and the copper: a synth_layers row's own, else the best round's WHOLE line in the
case's run.log beside the table -- then the totals of both runs and the cases in one only. The seconds are summed, not
compared case by case: a cloud container's cores vary between runs (the same 4-layer cases ran 0.75x as long in one
container as in the other, the 3-layer ones 5.6x, both runs of one code).

    python3 synth_ab.py BASE_OUT NEW_OUT        # in awx/
"""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # a research tool (awx), catalogued, shown at no door

import argparse
import csv
import os
import re
import sys

GRADE = ('verdict', 'round', 'vias', 'connected', 'drc', 'open', 'gap', 'turn', 'hold', 'static')
PASSED = ('PASS', 'OPTIMAL', 'BETTER', 'ROUTED')
WHOLE = re.compile(r'WHOLE K=\d+ round=\d+ lanes=\S+ vias=\d+ copper=(\d+)mm')


def table(spec):
    """{(tag, layers): row} of a run: its table's rows, each with 'copper' (mm, as text) where it can be read"""
    rows = {}
    for d in spec.split(','):
        f = d if d.endswith('.tsv') else next((os.path.join(d, t) for t in ('handoff.tsv', 'layers.tsv')
                                               if os.path.isfile(os.path.join(d, t))), None)
        if f is None or not os.path.isfile(f):
            raise SystemExit(f'synth_ab: no handoff.tsv or layers.tsv in {d}')
        for r in csv.DictReader(open(f), delimiter='\t'):
            r = dict(r)
            if r.get('copper'):
                r['copper'] = r['copper'].rstrip('m')
            else:
                log = os.path.join(os.path.dirname(f), r['tag'], 'run.log')
                got = [m.group(1) for m in map(WHOLE.search, open(log, errors='replace')) if m] \
                    if os.path.isfile(log) else []
                r['copper'] = got[-1] if got else ''         # (the last WHOLE line is the best round's)
            rows[(r['tag'], r.get('layers', ''))] = r
    return rows


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('a', metavar='A', help='the first run: an OUTDIR (or its table), several joined with commas')
    ap.add_argument('b', metavar='B', help='the second run, the same')
    args = ap.parse_args(argv)
    a, b = table(args.a), table(args.b)
    both = sorted(set(a) & set(b))
    cols = [c for c in GRADE if any(c in a[k] for k in both)] + ['copper']
    differ = 0
    for k in both:
        d = [(c, a[k].get(c), b[k].get(c)) for c in cols if a[k].get(c) != b[k].get(c)]
        if d:
            differ += 1
            print(f'  {k[0]:28s}{" L" + k[1] if k[1] else ""}  ' + ', '.join(f'{c} {x} -> {y}' for c, x, y in d))
    for name, only in (('A', sorted(set(a) - set(b))), ('B', sorted(set(b) - set(a)))):
        if only:
            print(f'  only in {name}: {", ".join(t + (" L" + L if L else "") for t, L in only)}')

    def total(rows, c):
        return sum(num(rows[k].get(c)) or 0 for k in both)

    def failed(rows):
        # (synth_handoff's PASS; synth_layers' OPTIMAL, BETTER and ROUTED -- routed, graded or not)
        return sum(1 for k in both if rows[k].get('verdict') not in PASSED)
    print(f'{len(both)} cases in both, {differ} differ; vias {total(a, "vias"):.0f} -> {total(b, "vias"):.0f}, copper '
          f'{total(a, "copper"):.0f} -> {total(b, "copper"):.0f} mm'
          + (f', static {total(a, "static"):.0f} -> {total(b, "static"):.0f}' if 'static' in cols else '')
          + f', failed {failed(a)} -> {failed(b)}; seconds {total(a, "secs"):.0f} -> {total(b, "secs"):.0f}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
