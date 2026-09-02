#!/usr/bin/env python3
"""#849: the lane ledger's whole-board geometry is resolved once per BOARD.

`face_lane_ledger` grades ONE ref out of five whole-board objects, and it
rebuilt all five per call -- including `graded_parts_from_file`, which re-reads
the `.kicad_pcb` and regex-walks every footprint's courtyards. `check_channels`
sweeps every fine-pitch ref, and a second time per ref with `--baseline`.

This is a HOIST. The output must not move, so most of what could go wrong
cannot be seen in a number -- which is why the arms below are mechanism arms,
not value arms:

  1. EQUIVALENCE, computed independently. Every ref on two boards, context
     path against no-context path, on a SEPARATELY PARSED board so the
     comparison is not the same objects agreeing with themselves.
  2. THE HOIST HAPPENED. A spy on the upstream producer
     (`placement.parser.extract_courtyard_sides`) counts 1 parse for an N-ref
     sweep with a context, and N without. Equivalence alone passes just as
     happily when the hoist silently did not happen; this is the arm that
     pins #849's actual claim, and it counts the PRODUCER rather than a
     derived object.
  3-5. THE GUARD REFUSES a context built for another board, another file, or
     another clearance -- naming both values. The BOARD arm is the one no
     committed test covered before: `check_channels` ledgers two boards in
     one run (the board and its `--baseline`), so a context hoisted to the
     wrong scope would grade the baseline's refs against the primary board's
     geometry and invent a `--gate` exit 4 on a board that did not regress.
  6. A LOAD-TOLERANT SPEEDUP FLOOR. Deliberately far below the measured
     ratio: pinning a multiple would make this a machine detector (the
     lesson `tests/test_outline_prefilter.py` records). What it catches is
     the regression where the context is accepted and then ignored, which
     lands at 1.0x.
  7. THE DEFAULT PATH IS UNTOUCHED -- no context, and `escape_band_mm` still
     reaches through the context path. Nine call sites in five files pass no
     context; if they had to change, the hoist would not be a hoist.
  8. `pair_channel_widths` agrees with itself either way. It runs once after
     the sweep and parsed the courtyards one more time.
  9. END TO END: the CLI's own `ledgers` equal a fresh library recomputation
     with no context at all. The byte-identity claim, executable.

MEASURED on this machine (min-of-5, both arms in one process, the context
built inside the timed region because one per run is what the caller does):

    board                       refs   sweep            courtyard parses
    tigard                         2   0.119s -> 0.052s   2 -> 1
    rp2350_fpga_eensy_prePlane     7   0.284s -> 0.045s   7 -> 1
    glasgow_revC                   9   1.767s -> 0.198s   9 -> 1

The parse counts are the durable half; the seconds are load-dependent and
this machine does not reproduce #849's own 12.6-15.8s on glasgow at all.

Those ref counts are `check_channels`' OWN auto-detection at that lane. The
arms below sweep `escape.fine_pitch_parts` instead -- 10 refs on both boards
-- so their numbers are not comparable to the table and are not meant to be:
the table measures the tool, the arms measure the ledger. `refs_of` says why
this file does not carry a second copy of the CLI's selector.

Run: python3 -X utf8 tests/test_849_lane_context.py
"""
import json
import os
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'tests')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                              # noqa: E402
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import escape as E                             # noqa: E402
from placement import parser as pparser                       # noqa: E402
from placement import routability as R                        # noqa: E402

SKIP_EXIT = 77

#: The lane the corpus is swept at elsewhere in this file's family.
LANE = {'clearance': 0.09, 'track_width': 0.127, 'grid_step': 0.05}

#: Two boards, because one board agreeing with itself is one board. tigard is
#: small enough to sweep twice in a test; rp2350 carries enough fine-pitch
#: parts that the per-ref rebuild is visible in the clock.
SMALL = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
DENSE = os.path.join(ROOT, 'kicad_files',
                     'rp2350_fpga_eensy_prePlane.kicad_pcb')

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def refs_of(pcb):
    """The parts worth a ledger.

    `escape.fine_pitch_parts` rather than a copy of `check_channels`' inline
    auto-detection: a second copy of a selector is how two instruments come to
    disagree, and this file does not need the SAME set -- only a real one.
    """
    return E.fine_pitch_parts(pcb)


def sweep(pcb, path, refs, ctx=None):
    kw = dict(LANE, pcb_file=path)
    if ctx is not None:
        kw['context'] = ctx
    return {r: R.face_lane_ledger(pcb, r, **kw) for r in refs}


def counted(fn):
    """Run `fn`, returning (result, times the board's courtyards were parsed).

    The spy goes on `placement.parser`, and it lands because
    `legality.part_local_bounds` does its `from placement.parser import ...`
    INSIDE the function body -- so the name is re-resolved on the module
    object at every call. A module-level binding elsewhere (quench, labels)
    would not be intercepted, and neither is on this path.
    """
    real = pparser.extract_courtyard_sides
    n = [0]

    def spy(p, _r=real, _n=n):
        _n[0] += 1
        return _r(p)

    pparser.extract_courtyard_sides = spy
    try:
        return fn(), n[0]
    finally:
        pparser.extract_courtyard_sides = real


def refuses(fn, *musts):
    """`fn` raises ValueError whose message names every one of `musts`."""
    try:
        fn()
    except ValueError as exc:
        msg = str(exc)
        missing = [m for m in musts if m not in msg]
        return (not missing), (f'raised, but did not name {missing}: {msg}'
                               if missing else msg)
    except Exception as exc:                       # noqa: BLE001
        return False, f'raised {type(exc).__name__}, not ValueError: {exc}'
    return False, 'did not raise at all -- the guard did not hold'


def main():
    for p in (SMALL, DENSE):
        if not os.path.isfile(p):
            print(f'SKIP: {p} is not in this checkout')
            return SKIP_EXIT
    run_utils.evidence(SMALL, 'the small board every arm below is fed')
    run_utils.evidence(DENSE, 'the dense board the timing arm is fed')

    print('1. the context path and the no-context path agree, per ref')
    for path in (SMALL, DENSE):
        name = os.path.basename(path)
        # TWO parses on purpose. Handing one `pcb` to both arms would let a
        # bug that mutates shared state pass by mutating both alike.
        pcb_a = parse_kicad_pcb(path)
        pcb_b = parse_kicad_pcb(path)
        refs = refs_of(pcb_a)
        check(f'{name} has fine-pitch parts to ledger', bool(refs),
              'no refs: this arm would assert nothing')
        plain = sweep(pcb_a, path, refs)
        ctx = R.board_lane_context(pcb_b, LANE['clearance'], pcb_file=path)
        hoisted = sweep(pcb_b, path, refs, ctx)
        check(f'{name}: {len(refs)} refs, identical rows either way',
              plain == hoisted,
              _first_difference(plain, hoisted))
        check(f'{name}: the sweep actually produced rows',
              any(plain.values()), f'{plain}')

    print('2. the board is parsed ONCE per sweep, not once per ref')
    pcb = parse_kicad_pcb(DENSE)
    refs = refs_of(pcb)
    _, n_plain = counted(lambda: sweep(pcb, DENSE, refs))
    ctx = R.board_lane_context(pcb, LANE['clearance'], pcb_file=DENSE)
    _, n_ctx = counted(lambda: sweep(pcb, DENSE, refs, ctx))
    check(f'without a context: one courtyard parse per ref ({len(refs)})',
          n_plain == len(refs), f'{n_plain} parses for {len(refs)} refs')
    check('with a context: exactly one, however many refs',
          n_ctx == 1, f'{n_ctx} parses for {len(refs)} refs')
    check('...and the spy saw anything at all', n_plain > 0,
          'zero parses means the spy missed the call and this arm is blind')

    print('3. a context built for ANOTHER BOARD is refused')
    other = parse_kicad_pcb(SMALL)
    other_ctx = R.board_lane_context(other, LANE['clearance'], pcb_file=SMALL)
    ok, why = refuses(
        lambda: R.face_lane_ledger(pcb, refs[0], **dict(LANE, pcb_file=DENSE,
                                                        context=other_ctx)),
        'DIFFERENT board', os.path.basename(SMALL))
    check('the wrong board is named, not silently graded', ok, why)

    print('4. a context built from ANOTHER FILE is refused')
    same_board_wrong_file = R.board_lane_context(pcb, LANE['clearance'],
                                                 pcb_file=SMALL)
    ok, why = refuses(
        lambda: R.face_lane_ledger(pcb, refs[0],
                                   **dict(LANE, pcb_file=DENSE,
                                          context=same_board_wrong_file)),
        os.path.basename(SMALL), os.path.basename(DENSE))
    check('both file names appear in the refusal', ok, why)

    print('5. a context built at ANOTHER CLEARANCE is refused')
    wrong_clr = R.board_lane_context(pcb, 0.2, pcb_file=DENSE)
    ok, why = refuses(
        lambda: R.face_lane_ledger(pcb, refs[0], **dict(LANE, pcb_file=DENSE,
                                                        context=wrong_clr)),
        '0.2', '0.09')
    check('both clearances appear in the refusal', ok, why)

    print('6. the sweep is faster, by a margin no machine can explain away')
    best_plain = min(_timed(sweep, pcb, DENSE, refs) for _ in range(3))
    best_ctx = min(_timed(_ctx_sweep, pcb, DENSE, refs) for _ in range(3))
    ratio = best_plain / max(best_ctx, 1e-9)
    # The floor is 1.5x against a measured 6.3x. A context that is accepted
    # and then ignored lands at 1.0x, which is what this catches; anything
    # tighter is a detector for how busy the machine is.
    check(f'{len(refs)}-ref sweep at least 1.5x faster ({ratio:.1f}x measured)',
          ratio >= 1.5,
          f'plain {best_plain:.3f}s, hoisted {best_ctx:.3f}s')

    print('7. the default path is untouched')
    rows = R.face_lane_ledger(pcb, refs[0], **dict(LANE, pcb_file=DENSE))
    check('no context at all still returns four faces', len(rows) == 4,
          f'{len(rows)} rows')
    check('...and still says taps are not modeled',
          all(r['taps_not_modeled'] for r in rows), f'{rows}')
    band_plain = R.face_lane_ledger(pcb, refs[0],
                                    **dict(LANE, pcb_file=DENSE,
                                           escape_band_mm=4.0))
    band_ctx = R.face_lane_ledger(pcb, refs[0],
                                  **dict(LANE, pcb_file=DENSE,
                                         escape_band_mm=4.0, context=ctx))
    check('escape_band_mm still reaches through the context path',
          band_plain == band_ctx, f'{band_plain}\n        {band_ctx}')
    check('...and a wider band is a DIFFERENT answer, so that arm bites',
          band_plain != rows,
          'band 4.0 gave the same rows as the default band; this arm would '
          'pass even if the keyword were dropped on the floor')

    print('8. the anchor-channel table agrees either way')
    pcw_plain = R.pair_channel_widths(pcb, clearance=LANE['clearance'],
                                      pcb_file=DENSE)
    pcw_ctx = R.pair_channel_widths(pcb, clearance=LANE['clearance'],
                                    pcb_file=DENSE, context=ctx)
    check('pair_channel_widths: identical rows', pcw_plain == pcw_ctx,
          f'{len(pcw_plain)} vs {len(pcw_ctx)} rows')
    check('...and it produced some', bool(pcw_plain), 'no rows: arm is blind')

    print('9. the CLI ledgers what the library ledgers, with no context')
    with tempfile.TemporaryDirectory() as td:
        jp = os.path.join(td, 'c.json')
        run_utils.check(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_tools', 'check_channels.py'), SMALL,
             '--clearance', str(LANE['clearance']),
             '--track-width', str(LANE['track_width']),
             '--grid-step', str(LANE['grid_step']), '--json', jp],
            accept=True)
        run_utils.evidence(jp, "check_channels' own json")
        doc = json.load(open(jp, encoding='utf-8'))
        fresh = parse_kicad_pcb(SMALL)
        mine = {r: R.face_lane_ledger(fresh, r, **dict(LANE, pcb_file=SMALL))
                for r in doc['ledgers']}
        # The CLI writes tuples as JSON arrays; round-trip mine the same way
        # so this compares the numbers rather than the container types.
        mine = json.loads(json.dumps(mine))
        check('every ref the CLI reports matches an un-hoisted recomputation',
              mine == doc['ledgers'],
              _first_difference(mine, doc['ledgers']))
        check('...and it reported some', bool(doc['ledgers']), 'no ledgers')

    print()
    if FAILURES:
        print(f'FAILED: {len(FAILURES)} -- ' + ', '.join(FAILURES))
        return 1
    print('all arms passed')
    return 0


def _ctx_sweep(pcb, path, refs):
    """One FRESH context per timed run, as `check_channels` builds one per run.

    Reusing a context across reps times a warm cache and reported 800x.
    """
    return sweep(pcb, path, refs,
                 R.board_lane_context(pcb, LANE['clearance'], pcb_file=path))


def _timed(fn, *a):
    t = time.perf_counter()
    fn(*a)
    return time.perf_counter() - t


def _first_difference(a, b):
    """Which ref and which face disagree -- a bare 'not equal' is unfixable."""
    if a == b:
        return ''
    ka, kb = set(a), set(b)
    if ka != kb:
        return f'different refs: only in A {sorted(ka - kb)}, B {sorted(kb - ka)}'
    for ref in sorted(ka):
        if a[ref] != b[ref]:
            for ra, rb in zip(a[ref], b[ref]):
                if ra != rb:
                    return f'{ref} face {ra.get("face")}: {ra} != {rb}'
            return f'{ref}: {a[ref]} != {b[ref]}'
    return 'equal by ==, unequal by ... nothing; report this'


if __name__ == '__main__':
    sys.exit(main())
