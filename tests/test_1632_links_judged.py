#!/usr/bin/env python3
"""#1632: a run of several seeds names each intent link a seed leaves past its
`max_mm`, and judge.json carries every seed's links.

A link (#1477) is priced, not held, and check_floorplan grades it a WARN
unless the intent's `severity.link` promotes it, so the judge does not rank
by it. Before #1632 the only line naming a missed link was in the seed's own
log: place_simple's run output named none, judge.json had no key for it, and
a seed 10 mm past a 3 mm link was judged first, ahead of a seed that held it,
with nothing said.

Checks (synthetic seed summaries, as place_simple's JSON_SUMMARY writes them):
  1. `seed_findings` names a seed's link past its limit on the seed's line,
     and nothing for a link it holds;
  2. `_name_judged` writes `links` per seed to judge.json and WARNS once per
     link past its limit: the seeds past it with their gaps, the seeds
     holding it, that the first judged seed breaks it while another holds it,
     and that the judge does not rank by it; a link no seed breaks is not
     warned;
  3. with the intent's `severity.link` promoted, the warning says the judge
     ranks by it instead;
  4. a seed report written before `past` was recorded is read off its gap.
"""
import argparse
import contextlib
import io
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import place_simple                                           # noqa: E402


def report(cost, gaps, severity='warn', legacy=False):
    """A seed's summary as place_simple writes it, with links `gaps`
    ({name: (gap_mm, max_mm)})."""
    links = []
    for name, (gap, lim) in gaps.items():
        row = {'link': name, 'gap_mm': gap, 'max_mm': lim}
        if not legacy:
            row.update(past=gap > lim + 1e-9, severity=severity)
        links.append(row)
    return {'final': {'cost': cost}, 'illegal': [], 'refused': {},
            'floorplan': {'errors': 0, 'rules': []}, 'links': links}


# the issue's four seeds: the cheapest (s2) 10 mm past a 3 mm link that s0
# holds; a second link every seed holds
GAPS = {0: 2.649, 1: 13.981, 2: 13.126, 3: 3.516}
COST = {0: 3215.9, 1: 3619.6, 2: 3174.3, 3: 4454.6}


def run(summaries):
    with tempfile.TemporaryDirectory() as td:
        a = argparse.Namespace(judge=6, out=os.path.join(td, 's{seed}.kicad_pcb'))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            place_simple._name_judged(a, sorted(summaries), summaries)
        doc = json.load(open(os.path.join(td, 'judge.json'), encoding='utf-8'))
    return doc, buf.getvalue()


def test_seed_line():
    s2 = report(COST[2], {'u3_j7': (GAPS[2], 3.0), 'held': (0.5, 2.0)})
    line = place_simple.seed_findings(s2)
    assert 'link u3_j7 13.13 mm, past its 3 mm' in line, line
    assert 'held' not in line, line
    assert place_simple.seed_findings(report(COST[0], {'u3_j7': (GAPS[0], 3.0)})) == ''
    print(f'  PASS: the seed line names the link it breaks: {line}')


def test_judge_json_and_warning():
    seeds = {s: report(COST[s], {'u3_j7': (GAPS[s], 3.0), 'held': (0.5, 2.0)})
             for s in GAPS}
    doc, out = run(seeds)
    assert doc['judge'] == [2, 0, 1, 3], doc['judge']      # the judge is unchanged
    assert sorted(doc['links']) == ['0', '1', '2', '3'], doc['links']
    assert doc['links']['2'][0] == {'link': 'u3_j7', 'gap_mm': 13.126, 'max_mm': 3.0,
                                    'past': True, 'severity': 'warn'}, doc['links']['2']
    warned = [ln for ln in out.splitlines() if 'WARNING: link' in ln]
    assert len(warned) == 1, out
    w = warned[0]
    for want in ('link u3_j7 is past its 3 mm in seed(s) 1 (13.98 mm), 2 (13.13 mm), '
                 '3 (3.52 mm)', 'held in seed(s) 0',
                 'the first judged seed, 2, breaks it while another holds it',
                 'so the judge ranks a seed no lower for it'):
        assert want in w, (want, w)
    print(f'  PASS: judge.json carries each seed\'s links, and one WARNING: {w}')


def test_promoted_severity():
    seeds = {s: report(COST[s], {'u3_j7': (GAPS[s], 3.0)}, severity='error') for s in GAPS}
    _doc, out = run(seeds)
    w = next(ln for ln in out.splitlines() if 'WARNING: link' in ln)
    assert 'makes it a floorplan error, which the judge ranks by' in w, w
    assert 'no lower' not in w, w
    print('  PASS: a promoted link is said to be ranked by')


def test_legacy_rows():
    seeds = {s: report(COST[s], {'u3_j7': (GAPS[s], 3.0)}, legacy=True) for s in GAPS}
    _doc, out = run(seeds)
    w = next(ln for ln in out.splitlines() if 'WARNING: link' in ln)
    assert 'in seed(s) 1 (13.98 mm), 2 (13.13 mm), 3 (3.52 mm)' in w, w
    print('  PASS: a report without `past` is read off its gap')


def main():
    test_seed_line()
    test_judge_json_and_warning()
    test_promoted_severity()
    test_legacy_rows()
    print('test_1632_links_judged: all passed')


if __name__ == '__main__':
    main()
