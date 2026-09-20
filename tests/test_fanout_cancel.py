#!/usr/bin/env python3
"""#621: the fanout engines take a cooperative `cancel_check`, and it is inert.

The engines gained the same zero-arg `cancel_check` that `batch_route` and
`create_plane` take, honoured at every escape-pass and per-ball loop head as a
`break` (never a raise -- an exception dies in these packages' `except
Exception` swallowers). Its ONLY caller is the GUI: the fanout tab's Cancel
button and the AI tab's Stop, both through `FanoutTab._cancel_requested`. The
CLI passes None and has no budget flag, deliberately -- no result these tools
produce may depend on a wall clock.

What is pinned here:

1.  INERTNESS, at the only level that counts: the copper. `cancel_check=None`
    and `cancel_check=lambda: False` must emit the same tracks and vias,
    coordinate for coordinate, on both engines. The #621 checks sit at loop
    heads and do nothing but `break`, so a predicate that never returns True
    cannot reach a single routing decision -- and this is the measurement that
    says so, which is why no corpus A/B is needed for the change.

    (A metric that scored the same with and without the change would be no
    evidence at all. This one is the opposite: it is defined to differ the
    instant a cancel check does anything other than break a loop.)

2.  The CLI ledger is unchanged and carries no cancel keys, and the tool still
    prints exactly ONE JSON_SUMMARY.

3.  `--deadline` is gone from both mains and stays gone. A removed flag exits
    argparse rc 2, which reads as an engine failure to any harness that still
    passes it, so the removal is pinned rather than assumed.

Deliberately NOT tested by driving a cancel from a call counter or a clock:
the only real cancel is a human clicking Cancel, and a synthetic one measures
the harness, not the tool.

    python3 -X utf8 tests/test_fanout_cancel.py
"""
import io
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

BOARD = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted_placed.kicad_pcb')
QFN_BOARD = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')

# The measured, uncancelled ledger for `bga_fanout.py <BOARD> --component U9`
# (~2s, every ball escapes -- fast, and with real work a cancel could abandon).
# Re-measured 2026-09-20 on bus622-take5: 75 -> 77. The fanout's nanometre
# tie-breaks (nearest edge / channel / gap on this 2.54 mm PGA, where many
# balls are EXACTLY equidistant) give the channel pass an assignment with no
# collisions, so every ball escapes there; main's channel pass hit 32
# collisions, dropped two balls and fell back to the under-pad engine, whose
# ledger counted 75. Same board, same command, both arms 0 failed.
BASE = {'requested': 77, 'escaped': 77, 'failed': 0, 'unescaped_nets': [],
        'skipped_nc': 24, 'component': 'U9'}
BASE_DRC_TOTAL = 0

BAD = []


def check(cond, label):
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        BAD.append(label)


def _copper(tracks, vias):
    """Coordinate-level identity of the emitted copper."""
    t = sorted((round(x['start'][0], 6), round(x['start'][1], 6),
                round(x['end'][0], 6), round(x['end'][1], 6),
                round(x['width'], 6), x['layer'], x['net_id'])
               for x in tracks)
    v = sorted((round(x['x'], 6), round(x['y'], 6), round(x['size'], 6),
                round(x['drill'], 6), tuple(x.get('layers') or ()), x['net_id'])
               for x in vias)
    return t, v


def _summaries(stdout):
    return [json.loads(l.split('JSON_SUMMARY: ', 1)[1])
            for l in stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]


def _quiet(fn):
    """Run fn with stdout captured -- these engines are very chatty."""
    buf, old = io.StringIO(), sys.stdout
    sys.stdout = buf
    try:
        return fn()
    finally:
        sys.stdout = old


def test_bga_engine_is_identical_with_a_cancel_that_never_fires():
    print('test_bga_engine_is_identical_with_a_cancel_that_never_fires')
    from kicad_parser import parse_kicad_pcb
    import bga_fanout

    def run(cc):
        pcb = parse_kicad_pcb(BOARD)          # fresh board each arm
        tr, va, _vr, failed = _quiet(
            lambda: bga_fanout.generate_bga_fanout(
                pcb.footprints['U9'], pcb, cancel_check=cc))
        return _copper(tr, va), sorted(failed)

    try:
        (t0, v0), f0 = run(None)
        (t1, v1), f1 = run(lambda: False)
    except TypeError as exc:
        # Reported as a FAILED CHECK, not a traceback: a crash here would abort
        # the remaining tests instead of accumulating them, which is how a red
        # run stops telling you what else is red.
        check(False, f'generate_bga_fanout takes cancel_check ({exc})')
        return
    check(len(t0) > 0, f'the control arm actually routed ({len(t0)} tracks)')
    check(t0 == t1,
          f'IDENTICAL track set ({len(t0)} vs {len(t1)} segments, coordinate '
          'for coordinate)')
    check(v0 == v1, f'IDENTICAL via set ({len(v0)} vs {len(v1)})')
    check(f0 == f1, 'IDENTICAL failed-net list')
    check(bga_fanout.LAST_CANCEL_SKIPPED == [],
          'a cancel that never fires leaves LAST_CANCEL_SKIPPED empty')


def test_qfn_engine_is_identical_with_a_cancel_that_never_fires():
    print('test_qfn_engine_is_identical_with_a_cancel_that_never_fires')
    if not os.path.isfile(QFN_BOARD):
        print('  SKIP  no QFN board')
        return
    from kicad_parser import parse_kicad_pcb
    import qfn_fanout

    def run(cc):
        pcb = parse_kicad_pcb(QFN_BOARD)
        tr, va, failed = _quiet(
            lambda: qfn_fanout.generate_qfn_fanout(
                pcb.footprints['U2'], pcb, cancel_check=cc))
        return _copper(tr, va), sorted(failed)

    try:
        (t0, v0), f0 = run(None)
        (t1, v1), f1 = run(lambda: False)
    except TypeError as exc:
        check(False, f'generate_qfn_fanout takes cancel_check ({exc})')
        return
    check(len(t0) > 0, f'the control arm actually routed ({len(t0)} tracks)')
    check(t0 == t1 and v0 == v1,
          f'IDENTICAL copper ({len(t0)} segments, {len(v0)} vias)')
    check(f0 == f1, 'IDENTICAL failed-net list')
    check(qfn_fanout.LAST_CANCEL_SKIPPED == [],
          'a cancel that never fires leaves LAST_CANCEL_SKIPPED empty')


def test_cli_ledger_is_field_for_field_the_baseline(tmpdir):
    print('test_cli_ledger_is_field_for_field_the_baseline')
    out = os.path.join(tmpdir, 'plain.kicad_pcb')
    p = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join('py_router', 'bga_fanout.py'),
         BOARD, '--component', 'U9', '--output', out],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT, timeout=900)
    check(p.returncode == 0, f'rc 0 (got {p.returncode})')
    docs = _summaries(p.stdout)
    check(len(docs) == 1, f'exactly one JSON_SUMMARY (got {len(docs)})')
    if not docs:
        return
    d = docs[0]
    for k, v in BASE.items():
        check(d.get(k) == v, f'{k} == {v!r} (got {d.get(k)!r})')
    check((d.get('drc_grazes') or {}).get('total') == BASE_DRC_TOTAL,
          f'drc_grazes.total == {BASE_DRC_TOTAL}')
    check(not any(k.startswith('deadline') or k.startswith('cancel')
                  for k in d),
          'no cancel/deadline keys on a CLI run -- the CLI cannot cancel')
    check(os.path.isfile(out), 'the output board IS written')


def test_the_deadline_flag_is_gone(tmpdir):
    """A removed flag exits argparse rc 2, which a harness reads as an engine
    failure -- so pin the removal instead of assuming it."""
    print('test_the_deadline_flag_is_gone')
    for tool, comp, board in (('bga_fanout.py', 'U9', BOARD),
                              ('qfn_fanout.py', 'U2', QFN_BOARD)):
        if not os.path.isfile(board):
            print(f'  SKIP  no board for {tool}')
            continue
        p = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join('py_router', tool),
             board, '--component', comp, '--deadline', '4',
             '--output', os.path.join(tmpdir, 'dl.kicad_pcb')],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=300)
        check('unrecognized arguments: --deadline' in (p.stdout + p.stderr),
              f'{tool}: --deadline is rejected by argparse')
        h = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join('py_router', tool),
             '--help'], capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=300)
        check('deadline' not in h.stdout.lower(),
              f'{tool}: --help does not advertise a budget')


def run():
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        test_bga_engine_is_identical_with_a_cancel_that_never_fires()
        test_qfn_engine_is_identical_with_a_cancel_that_never_fires()
        test_cli_ledger_is_field_for_field_the_baseline(tmp)
        test_the_deadline_flag_is_gone(tmp)
    print('\nFAIL: %d check(s)' % len(BAD) if BAD else '\nOK')
    for b in BAD:
        print('  - ' + b)
    return 1 if BAD else 0


if __name__ == '__main__':
    sys.exit(run())
