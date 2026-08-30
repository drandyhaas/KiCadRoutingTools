#!/usr/bin/env python3
"""#788: how close are the two pre-route legality censuses, and does the
ungated half of the family earn a refusal at L2?

NOT a test, and deliberately not named `test_*` so `run_all.py` never collects
it: it needs the #703 study tree, which is gitignored (`wk/`), and it spends a
few minutes of `check_assembly` subprocesses. It regenerates every number
`docs/placement-predictors.md` states about the two censuses, and the numbers
`loop_driver.py`'s #788 comment states about what a graze/hole gate would
refuse. `tests/test_788_marginal_literals.py` is the committed, clean-clone
change detector for a declared subset of the same rows.

    python3 -X utf8 tests/measure_788_censuses.py
    python3 -X utf8 tests/measure_788_censuses.py --arm routed
    python3 -X utf8 tests/measure_788_censuses.py --boards esp_prog watchy

THE TWO ARMS, AND WHY BOTH ARE REPORTED. The study measured its predictors on
the PRE-ROUTE variant board; `check_assembly` is normally run on a placed
board. Those two boards carry the same poses -- copper is inert to
`grade_pad_legality`, which reads footprints, courtyards and the outline -- but
NOT the same clearance floor:

  * `routed`    grades `<variant>/routed.kicad_pcb`, which carries the routing
                writeback's sibling `.kicad_pro`, so each board grades at its
                own Default netclass (0.1 - 0.2 mm here).
  * `preroute`  grades the variant board the study actually measured. The
                study's variant writer produced no sibling `.kicad_pro` -- the
                #441 hazard, live in this dataset -- so every board falls back
                to `routing_defaults.CLEARANCE` 0.25.

Neither is the deployment condition (a real placed board does carry a
project), so a single agreement figure would be a claim about a floor the
study never pinned. Both are printed, and the doc quotes both.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
STUDY = os.path.join(ROOT, 'wk', '703', 'study')
ROWS = os.path.join(STUDY, 'rows.jsonl')
CHECK_ASSEMBLY = os.path.join(ROOT, 'py_tools', 'check_assembly.py')

#: study predictor name -> check_assembly JSON key. Every pair here is a claim
#: that the two are meant to be the same quantity; the run says how close they
#: get. `pad_conflict_pairs` is an alias of `pad_clearance_pairs` in the row
#: (metrics vs checklist), so it is not a seventh comparison.
PAIRS = (
    ('pad_copper', 'oob_pad_count'),
    ('pad_clearance_pairs', 'pad_conflicts'),
    ('hole_conflicts', 'hole_conflicts'),
    ('courtyard_blocking_pairs', 'courtyard_blocking'),
    ('pad_intersection_pairs', 'blocking'),
)


def load_rows():
    if not os.path.isfile(ROWS):
        sys.stderr.write(
            'REFUSED: %s is not here.\n\nThis measurement needs the #703 study\n'
            'tree, which is gitignored. Rebuild it with:\n'
            '  python3 -X utf8 tests/stress/predictor_study.py --out wk/703/study -j 4\n'
            '(60 h serial; see docs/placement-predictors.md for what that buys.)\n'
            % ROWS)
        raise SystemExit(2)
    with open(ROWS, encoding='utf-8') as fh:
        return [json.loads(ln) for ln in fh if ln.strip()]


def board_for(row, arm):
    """The board file this arm grades, or None when it is not on disk."""
    d = os.path.join(STUDY, row['board_key'], row['variant'])
    if arm == 'routed':
        p = os.path.join(d, 'routed.kicad_pcb')
        return p if os.path.isfile(p) else None
    v = row['variant']
    if v == 'authored':
        p = os.path.join(ROOT, (row.get('provenance') or {}).get(
            'input_board', ''))
    elif v.startswith('portfolio-'):
        n = v.split('-', 1)[1]
        p = os.path.join(d, 'pf_%s' % n, 'cand_%02d.kicad_pcb' % int(n))
    else:
        p = os.path.join(d, '%s.kicad_pcb' % v)
    return p if os.path.isfile(p) else None


def grade(board, tmpdir, tag):
    """check_assembly's JSON for one board, or None with the reason printed."""
    out = os.path.join(tmpdir, tag + '.json')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', CHECK_ASSEMBLY, board, '--json', out],
        cwd=ROOT, capture_output=True, text=True, timeout=900)
    # exit 4 is NOT BUILDABLE, which is a verdict about the board and not a
    # failure of the measurement. Anything else, we did not measure.
    if r.returncode not in (0, 4) or not os.path.isfile(out):
        sys.stderr.write('  UNGRADED %s (exit %s): %s\n'
                         % (tag, r.returncode, (r.stderr or '')[-200:]))
        return None
    with open(out, encoding='utf-8') as fh:
        return json.load(fh)


def measure(rows, arm, jobs):
    tmp = tempfile.mkdtemp(prefix='m788_')
    todo = []
    for r in rows:
        b = board_for(r, arm)
        if b is None:
            sys.stderr.write('  MISSING %s %s:%s\n'
                             % (arm, r['board_key'], r['variant']))
            continue
        todo.append((r, b))
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        graded = list(ex.map(
            lambda rb: (rb[0], grade(rb[1], tmp,
                                     '%s_%s_%s' % (arm, rb[0]['board_key'],
                                                   rb[0]['variant']))),
            todo))
    return [(r, g) for r, g in graded if g is not None]


def report(arm, graded):
    print('\n=== ARM: %s (n=%d graded) ===' % (arm, len(graded)))
    floors = sorted({(round(float(g.get('clearance') or 0), 4),
                      g.get('clearance_source')) for _, g in graded})
    print('  floors seen: %s' % ', '.join('%gmm [%s]' % f for f in floors))

    print('  %-46s %-14s %s' % ('study predictor  vs  check_assembly key',
                                'same sign', 'same integer'))
    for pred, key in PAIRS:
        sign = exact = n = 0
        diffs = []
        for r, g in graded:
            a, b = r['predictors'].get(pred), g.get(key)
            if a is None or b is None:
                continue
            n += 1
            if (a > 0) == (b > 0):
                sign += 1
            else:
                diffs.append((r['board_key'], r['variant'], b, a))
            if a == b:
                exact += 1
        print('  %-46s %8s      %8s'
              % ('%s vs %s' % (pred, key), '%d/%d' % (sign, n),
                 '%d/%d' % (exact, n)))
        for d in diffs:
            print('      SIGN DISAGREEMENT %s %s: check_assembly=%s study=%s'
                  % d)

    req = [(r['board_key'], r['variant'], g.get('pad_clearance_required'))
           for r, g in graded if g.get('pad_clearance_required')]
    print('  pad_clearance_required non-empty on %d/%d' % (len(req), len(graded)))
    for b, v, rows_ in req:
        srcs = sorted({str(x[3]) for x in rows_ if len(x) > 3})
        print('      %-28s %-22s %s' % (b, v, ', '.join(srcs)))

    # The #788 question, conditioned on the refusals L2 already runs.
    have = [(r, g) for r, g in graded if r['truth'].get('blocking') is not None]
    shipped = [(r, g) for r, g in have
               if (g.get('blocking') or 0) > 0 or (g.get('oob_pad_count') or 0) > 0]
    added = [(r, g) for r, g in have if (r, g) not in shipped
             and ((g.get('pad_conflicts') or 0) > 0
                  or (g.get('hole_conflicts') or 0) > 0)]
    passed = [r['truth']['blocking'] for r, g in have if (r, g) not in shipped]
    passed.sort()
    import statistics as st
    # WHICH CONJUNCT DOES THE WORK. The gate has two, and a PR that says "the
    # gate we already have is validated" has to say which one it validated.
    by_blocking = [x for x in shipped if (x[1].get('blocking') or 0) > 0]
    by_oob = [x for x in shipped if x not in by_blocking]
    print('  L2 as shipped (blocking>0 or oob_pad_count>0): refuses %d/%d'
          % (len(shipped), len(have)))
    print('     of those, blocking>0 carries %d and oob_pad_count adds %d'
          % (len(by_blocking), len(by_oob)))
    print('     the %d it passes routed to blocking min %d / median %g / max %d'
          % (len(passed), passed[0], st.median(passed), passed[-1]))
    print('  + (pad_conflicts>0 or hole_conflicts>0): refuses %d/%d, ADDING %d'
          % (len(shipped) + len(added), len(have), len(added)))
    for r, g in added:
        print('      %-28s %-22s routed blocking=%s  pad=%s hole=%s'
              % (r['board_key'], r['variant'], r['truth']['blocking'],
                 g.get('pad_conflicts'), g.get('hole_conflicts')))
    for key in ('pad_conflicts', 'hole_conflicts'):
        carry = [(r, g) for r, g in have if (g.get(key) or 0) > 0]
        already = [x for x in carry if x in shipped]
        print('  boards carrying %-16s %2d; already refused by L2: %d'
              % (key + ':', len(carry), len(already)))
    court = [(r, g) for r, g in have if (r, g) not in shipped
             and (g.get('courtyard_blocking') or 0) > 0]
    hist = {}
    for r, _ in court:
        hist[r['truth']['blocking']] = hist.get(r['truth']['blocking'], 0) + 1
    print('  courtyard_blocking as an ABSOLUTE census would add %d marginal '
          'refusals, routed blocking %s'
          % (len(court), {k: hist[k] for k in sorted(hist)}))


def inertness(rows, jobs, clearance=0.25):
    """Is copper inert to `grade_pad_legality`? MEASURED, not assumed.

    Both arms grade the same poses, so every difference between them should be
    the clearance floor and nothing else -- but that is a premise, and the
    first version of this work "verified" it on three esp_prog variants, two of
    which are all-zero on every key. Such a sample cannot tell inertness from
    cleanliness. This grades the pre-route board and the routed board at a
    PINNED clearance and requires all five keys to agree, choosing variants
    with a wide spread rather than the cheapest ones.
    """
    tmp = tempfile.mkdtemp(prefix='m788i_')
    pairs, bad = 0, []
    for r in rows:
        pre, post = board_for(r, 'preroute'), board_for(r, 'routed')
        if pre is None or post is None:
            continue
        got = []
        for which, b in (('pre', pre), ('post', post)):
            out = os.path.join(tmp, f'{r["board_key"]}_{r["variant"]}_{which}.json')
            p = subprocess.run(
                [sys.executable, '-X', 'utf8', CHECK_ASSEMBLY, b,
                 '--clearance', str(clearance), '--json', out],
                cwd=ROOT, capture_output=True, text=True, timeout=900)
            if p.returncode not in (0, 4) or not os.path.isfile(out):
                got = None
                break
            with open(out, encoding='utf-8') as fh:
                g = json.load(fh)
            got.append(tuple(g.get(k) for _, k in PAIRS))
        if not got:
            continue
        pairs += 1
        if got[0] != got[1]:
            bad.append((r['board_key'], r['variant'], got[0], got[1]))
    print('\n=== copper inertness, both boards graded at --clearance %g ==='
          % clearance)
    print('  %d variant(s) compared; %d disagree' % (pairs, len(bad)))
    for b, v, a, c in bad:
        print('      %s:%s  pre=%s  routed=%s' % (b, v, a, c))
    if pairs and not bad:
        print('  => the routed board grades the PLACEMENT, not the copper')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--arm', choices=('routed', 'preroute', 'both'),
                    default='both')
    ap.add_argument('--inertness', action='store_true',
                    help='also grade both boards at a pinned clearance and '
                         'require all five keys to agree')
    ap.add_argument('--boards', nargs='*', default=None)
    ap.add_argument('-j', type=int, default=8)
    ap.add_argument('--dump', metavar='PATH',
                    help='also write one JSON row per graded board, so a later '
                         'question about this run costs no subprocesses')
    a = ap.parse_args()
    rows = load_rows()
    if a.boards:
        rows = [r for r in rows if r['board_key'] in a.boards]
    print('#788 census measurement over %d rows (%d boards)'
          % (len(rows), len({r['board_key'] for r in rows})))
    if a.inertness:
        inertness(rows, a.j)
        return 0
    dump = open(a.dump, 'w', encoding='utf-8') if a.dump else None
    try:
        for arm in (('routed', 'preroute') if a.arm == 'both' else (a.arm,)):
            graded = measure(rows, arm, a.j)
            report(arm, graded)
            for r, g in graded:
                if dump:
                    dump.write(json.dumps({
                        'arm': arm, 'board_key': r['board_key'],
                        'variant': r['variant'],
                        'routed_blocking': r['truth'].get('blocking'),
                        'predictors': {p: r['predictors'].get(p)
                                       for p, _ in PAIRS},
                        'assembly': {k: g.get(k) for k in (
                            'blocking', 'oob_pad_count', 'pad_conflicts',
                            'hole_conflicts', 'courtyard_blocking',
                            'buildable', 'locked_contacts', 'clearance',
                            'clearance_source', 'pad_clearance_required')},
                    }, sort_keys=True) + '\n')
    finally:
        if dump:
            dump.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
