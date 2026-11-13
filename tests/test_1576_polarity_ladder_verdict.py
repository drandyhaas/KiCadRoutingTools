"""#1576: route_diff's polarity ladder says why it gave up, and what runs next.

When a pair's polarity mismatch survives every candidate of the ladder (#142:
no swap, the pad swap, the source / target connector flips), route_diff used
to print two verdict lines the log just above contradicted:

  * `pad swap failed, no opposite-side connector option` -- the second half
    a literal, printed right after both flips were tried;
  * `SKIPPED - polarity swap needed but not allowed for this pair` -- for every
    pair, without reading `polarity_swap_denied`, so a pair whose swap
    `--polarity-swap-nets '*'` allowed (JSON `polarity_swap_denied_pairs []`)
    read "not allowed"; and "SKIPPED" was not the outcome, since the hybrid
    escape routes the pair right after it.

Now the ladder lists each candidate with the reason it failed (the rejection
it printed), `no opposite-side option` only when neither end can flip, and the
loop's line is keyed on `polarity_swap_denied` and names what runs next.
Since #1802 the no-swap candidate runs under either policy and a denied swap
is probed; on this pair the probe fails as the allowed swap does, so neither
arm is `polarity_swap_denied`.

    1. `_ladder_failure_text` maps each failed route result to its cause
    2. glasgow_revC /IO_Banks/Z0 (both flips fail), swaps allowed and denied

    python3 tests/test_1576_polarity_ladder_verdict.py
"""
import json
import os
import re
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-X', 'utf8', '-B']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def reasons():
    print('1. a failed candidate result -> its cause')
    import diff_pair_routing
    t = getattr(diff_pair_routing, '_ladder_failure_text', None)
    check('diff_pair_routing has _ladder_failure_text', t is not None)
    if t is None:
        return
    pcb = SimpleNamespace(nets={7: SimpleNamespace(name='/GND')})
    rows = [
        (None, 'no usable endpoints'),
        ({'failed': True, 'intra_pair_short': True, 'intra_pair_gap': 0.0758},
         'intra-pair clearance (P and N 0.0758mm apart)'),
        ({'failed': True, 'pn_crossing': True, 'connector_graze': True, 'graze_net_id': 7},
         'P/N tracks cross'),
        ({'failed': True, 'connector_graze': True, 'graze_net_id': 7},
         'grazes foreign copper (net /GND)'),
        ({'failed': True, 'needs_fanout': True}, 'no escape for the pair (needs a coupled fanout)'),
        ({'probe_blocked': True}, 'no escape path'),
        ({'failed': True, 'iterations': 5000}, 'no route found'),
    ]
    for res, want in rows:
        got = t(res, pcb)
        check(f'{res and sorted(k for k in res if res[k] is True)} -> {want!r}', got == want,
              f'got {got!r}')


PAIR = '/IO_Banks/Z0'


def _route(td, arm, board):
    out = os.path.join(td, f'{arm}.kicad_pcb')
    argv = PY + [os.path.join('py_router', 'route_diff.py'), board, out,
                 '--nets', PAIR + '_*', '--no-bga-zones']
    if arm == 'allowed':
        argv += ['--polarity-swap-nets', '*']
    r = run_utils.check(argv, accept=True)
    lines = r.stdout.splitlines()
    js = next((ln for ln in lines if ln.startswith('JSON_SUMMARY:')), None)
    if js is None:
        raise AssertionError(f'BROKEN TEST: no JSON_SUMMARY ({arm})\n{r.stdout[-1500:]}')
    try:
        # the verdict's head ("not resolved by any coupled candidate"; it read
        # "cannot be resolved (...)" before #1576)
        start = next(i for i, ln in enumerate(lines)
                     if 'WARNING: Polarity mismatch' in ln)
    except StopIteration:
        raise AssertionError(f'BROKEN TEST: the fixture no longer exhausts the polarity '
                             f'ladder ({arm}), so it tests nothing\n{r.stdout[-2500:]}')
    verdict = []
    for ln in lines[start + 1:]:
        m = re.match(r'^    ([a-z][a-z /-]*): (.+)$', ln)
        if not m:
            break
        verdict.append((m.group(1), m.group(2)))
    return lines, start, verdict, json.loads(js[len('JSON_SUMMARY:'):])


def ladder(td):
    print('2. glasgow_revC /IO_Banks/Z0: the ladder fails, the hybrid routes it')
    board = os.path.join(td, 'a.kicad_pcb')
    run_utils.check(PY + [os.path.join('py_router', 'copy_board.py'),
                          os.path.join('kicad_files', 'glasgow_revC.kicad_pcb'), board],
                    accept=True)
    run_utils.evidence(board)
    for arm in ('allowed', 'denied'):
        lines, start, verdict, doc = _route(td, arm, board)
        text = '\n'.join(lines)
        names = [n for n, _ in verdict]
        denied = arm == 'denied'
        # #1802: the denied swap, probed, fails like the allowed one, so the
        # swap is not what is missing and the pair is not listed as denied
        check(f'{arm}: JSON polarity_swap_denied_pairs is []',
              doc.get('polarity_swap_denied_pairs') == [],
              f"{doc.get('polarity_swap_denied_pairs')}")
        # #1802: the no-swap candidate runs whatever the policy
        want = ['no swap', 'pad swap', 'flipped source', 'flipped target']
        check(f'{arm}: the verdict names every candidate: {want}', names == want,
              f'{verdict}')
        tried = [l for l in lines[:start] if 'trying' in l and 'opposite side' in l]
        check(f'{arm}: both flips ran, and "no opposite-side" is never claimed',
              len(tried) == 2 and 'no opposite-side' not in text, f'{tried}')
        why = dict(verdict)
        check(f'{arm}: the pad swap reads "not allowed" exactly when the policy '
              f'denies it', (why.get('pad swap', '').startswith('not allowed')) == denied,
              f"{why.get('pad swap')}")
        if denied:
            check('denied: the pad swap row says the probe failed too (#1802)',
                  'probed, the swap fails too: intra-pair clearance'
                  in why.get('pad swap', ''), f"{why.get('pad swap')}")
        # every intra-pair cause is the rejection the ladder printed above it
        warned = set(re.findall(r'intra-pair clearance: P and N would sit '
                                r'([0-9.]+)mm apart', '\n'.join(lines[:start])))
        cited = [m for _, w in verdict for m in
                 re.findall(r'intra-pair clearance \(P and N ([0-9.]+)mm apart\)', w)]
        check(f'{arm}: each intra-pair cause matches a rejection printed above',
              cited and set(cited) <= warned, f'cited {cited}, warned {sorted(warned)}')
        tail = next((l for l in lines[start:] if 'COUPLED ROUTE FAILED' in l), '')
        # keyed on whether the swap is what is missing (#1802): here it is
        # not, under either policy
        check(f'{arm}: the last line says the coupled route failed and the '
              f'hybrid is next, as a polarity mismatch',
              'trying the hybrid escape' in tail
              and 'polarity mismatch, and no candidate routed clean' in tail
              and not any('SKIPPED - polarity' in l for l in lines), tail)
        rep = next((p for p in doc.get('pair_reports', []) if p.get('pair') == PAIR), {})
        check(f'{arm}: ...and the hybrid does route it, coupled',
              rep.get('outcome') == 'coupled' and rep.get('escape') == 'hybrid',
              f"{rep.get('outcome')}/{rep.get('escape')}")


def main():
    reasons()
    td = tempfile.mkdtemp(prefix='t1576_')
    try:
        ladder(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'\nFAILED {len(failures)}: ' + '; '.join(failures))
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
