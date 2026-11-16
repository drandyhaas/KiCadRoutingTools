"""#1802: a pair whose polarity swap is denied still gets the no-swap coupled
attempt, and is reported `polarity-swap-denied` only when the swap is what is
missing.

`_detect_polarity` only SUSPECTS a swap (it compares P's side at the first
and last centreline segments, so a route that turns can trip it); the ladder
answers a suspicion by routing the natural, un-swapped geometry too (#142).
That candidate was gated on the swap policy, so a denied pair -- the default
-- never got it: a false positive sent it to the hybrid escape or
single-ended. And a denied pair whose every candidate failed on geometry was
reported `polarity-swap-denied`, pointing a user at the schematic, where
allowing the swap would not have coupled it either.

    1. a false-positive verdict on a denied pair that routes coupled (the
       detector forced to suspect a swap, in-process): the no-swap candidate
       routes it as a plain coupled route -- no failed ladder, no hybrid
       escape, not listed as denied. Before: the hybrid escape.
    2. a denied pair whose every candidate fails at the terminal, at a grid
       where the hybrid escape is rejected too: the ladder has its no-swap
       row and a probed pad swap that fails, the pair reads
       `polarity-mismatch` (not `polarity-swap-denied`), and its
       pair_reports row carries a `retry_hint` naming --grid-step.
    3. a denied pair whose other candidates fail and whose swap routes (the
       ladder's routes stubbed, in-process; no repository board shows it):
       the probe says so, and the pair is listed as denied.

    python3 -X utf8 tests/test_1802_polarity_denied_no_swap.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-X', 'utf8', '-B']
failures = []

#: route_diff with the polarity detector forced to suspect a swap on every
#: pair: the false positive the no-swap candidate exists for (#142), which
#: no board in the repository shows on its own
FORCED_DRIVER = r'''
import os, runpy, sys
sys.path.insert(0, os.path.join(sys.argv[1], 'py_router'))
import diff_pair_routing as d
_orig = d._detect_polarity
def _suspect(*a, **k):
    p_sign, _need, lc = _orig(*a, **k)
    return p_sign, True, lc
d._detect_polarity = _suspect
if sys.argv[2] == '--only-the-swap-routes':
    # every candidate the policy allows fails, and the denied swap's probe
    # routes: the case polarity-swap-denied is for. The probe is never
    # shipped, so a stub result is all it needs.
    del sys.argv[2]
    _route = d.route_diff_pair_with_obstacles
    def _ladder(*a, **k):
        if k.get('force_no_swap') or k.get('flip_source') or k.get('flip_target'):
            return {'failed': True, 'iterations': 0}
        if k.get('force_swap'):
            return {'iterations': 0, 'new_segments': [], 'new_vias': []}
        return _route(*a, **k)
    d.route_diff_pair_with_obstacles = _ladder
sys.argv = [os.path.join(sys.argv[1], 'py_router', 'route_diff.py')] + sys.argv[2:]
runpy.run_path(sys.argv[0], run_name='__main__')
'''


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def _copy(td, board):
    dst = os.path.join(td, os.path.basename(board))
    run_utils.check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                          os.path.join(REPO, 'kicad_files', board), dst], accept=True)
    run_utils.evidence(dst)
    return dst


def _report(js, pair):
    run_utils.evidence(js)
    with open(js, encoding='utf-8') as fh:
        doc = json.load(fh)
    return doc, next((p for p in doc.get('pair_reports', []) if p.get('pair') == pair), {})


def false_positive(td):
    print('1. a denied pair the detector wrongly suspects: the no-swap route couples it')
    board = _copy(td, 'lvds_converter_dualclk.kicad_pcb')
    drv = os.path.join(td, 'forced_polarity.py')
    with open(drv, 'w', encoding='utf-8') as fh:
        fh.write(FORCED_DRIVER)
    out, js = os.path.join(td, 'fp.kicad_pcb'), os.path.join(td, 'fp.json')
    r = run_utils.check(PY + [drv, REPO, board, out, '--nets', '/CLK*',
                              '--track-width', '0.127', '--clearance', '0.127',
                              '--diff-pair-gap', '0.127', '--json-out', js], accept=True)
    if 'Polarity mismatch - trying' not in r.stdout:
        raise AssertionError('BROKEN TEST: the forced detector did not reach the '
                             'polarity ladder, so this tests nothing\n' + r.stdout[-2000:])
    doc, rep = _report(js, '/CLK')
    check('the pair is denied (no --polarity-swap-nets)',
          'Polarity swaps disabled' in r.stdout)
    check('it ships coupled, as a plain coupled route (not the hybrid escape)',
          rep.get('outcome') == 'coupled' and 'escape' not in rep,
          f"{rep.get('outcome')}/{rep.get('escape')}")
    check('no ladder gave up on it',
          'Polarity mismatch not resolved' not in r.stdout
          and 'COUPLED ROUTE FAILED' not in r.stdout)
    check('nothing was wanted, so nothing is listed as denied',
          doc.get('polarity_swap_denied_pairs') == [],
          f"{doc.get('polarity_swap_denied_pairs')}")


def geometry_failure(td):
    print('2. a denied pair every candidate fails on geometry, the probed swap included')
    board = _copy(td, 'glasgow_revC.kicad_pcb')
    out, js = os.path.join(td, 'z0.kicad_pcb'), os.path.join(td, 'z0.json')
    r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'route_diff.py'), board, out,
                              '--nets', '/IO_Banks/Z0_*', '--no-bga-zones',
                              '--grid-step', '0.05', '--json-out', js], accept=True)
    lines = r.stdout.splitlines()
    try:
        start = next(i for i, ln in enumerate(lines)
                     if 'Polarity mismatch not resolved' in ln)
    except StopIteration:
        raise AssertionError('BROKEN TEST: the fixture no longer exhausts the polarity '
                             'ladder, so it tests nothing\n' + r.stdout[-2500:])
    rows = []
    for ln in lines[start + 1:]:
        m = re.match(r'^    ([a-z][a-z /-]*): (.+)$', ln)
        if not m:
            break
        rows.append((m.group(1), m.group(2)))
    why = dict(rows)
    check('the denied ladder has its no-swap row first',
          rows and rows[0][0] == 'no swap', f'{rows}')
    check('the pad swap row: not allowed, probed, fails too',
          why.get('pad swap', '').startswith('not allowed for this pair')
          and 'probed, the swap fails too' in why.get('pad swap', ''),
          f"{why.get('pad swap')}")
    doc, rep = _report(js, '/IO_Banks/Z0')
    if rep.get('outcome') != 'deferred':
        raise AssertionError(f"BROKEN TEST: the pair is no longer deferred at this grid "
                             f"({rep.get('outcome')}), so the reason and hint are untested")
    check('the reason is polarity-mismatch, not polarity-swap-denied',
          rep.get('failure_reason') == 'polarity-mismatch', f"{rep.get('failure_reason')}")
    check('not listed as denied', doc.get('polarity_swap_denied_pairs') == [],
          f"{doc.get('polarity_swap_denied_pairs')}")
    hint = rep.get('retry_hint') or ''
    check('its pair_reports row carries a retry_hint naming --grid-step',
          '--grid-step' in hint and '0.05' in hint, hint)
    check('...and the log prints it', any(ln.strip().startswith('Retry hint:') and
                                          '--grid-step' in ln for ln in lines))


def swap_is_missing(td):
    print('3. a denied pair only the swap would couple: polarity-swap-denied, said so')
    board = _copy(td, 'glasgow_revC.kicad_pcb')
    drv = os.path.join(td, 'forced_polarity.py')
    with open(drv, 'w', encoding='utf-8') as fh:
        fh.write(FORCED_DRIVER)
    out, js = os.path.join(td, 'sw.kicad_pcb'), os.path.join(td, 'sw.json')
    r = run_utils.check(PY + [drv, REPO, '--only-the-swap-routes', board, out,
                              '--nets', '/IO_Banks/Z3_*', '--no-bga-zones',
                              '--json-out', js], accept=True)
    lines = r.stdout.splitlines()
    rows = dict(m.groups() for m in (re.match(r'^    ([a-z][a-z /-]*): (.+)$', ln)
                                     for ln in lines) if m)
    if 'pad swap' not in rows:
        raise AssertionError('BROKEN TEST: no ladder verdict, so this tests nothing\n'
                             + r.stdout[-2500:])
    check('the pad swap row: not allowed, and the probe would route',
          rows['pad swap'] == 'not allowed for this pair (--polarity-swap-nets); '
                             'probed, the swap would route', rows['pad swap'])
    tail = next((ln for ln in lines if 'COUPLED ROUTE FAILED' in ln), '')
    check('the loop says the swap is what is missing',
          'not allowed for this pair (--polarity-swap-nets): the swap would route' in tail,
          tail)
    doc, _rep = _report(js, '/IO_Banks/Z3')
    check('listed in polarity_swap_denied_pairs',
          doc.get('polarity_swap_denied_pairs') == ['/IO_Banks/Z3'],
          f"{doc.get('polarity_swap_denied_pairs')}")


def main():
    td = tempfile.mkdtemp(prefix='t1802_')
    try:
        false_positive(td)
        geometry_failure(td)
        swap_is_missing(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'\nFAILED {len(failures)}: ' + '; '.join(failures))
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
