#!/usr/bin/env python3
"""Is this board DONE? The one command that answers it, and fails closed.

    python3 -X utf8 check_complete.py BOARD [--authored-from ORIGINAL] \
        [--intent J] [--impedance-nets G...] [--length-groups J] \
        [--net-min-widths J] [--min-track-width MM] [--min-via-diameter MM] \
        [--clearance MM] [--json PATH]

`board_score` is the authority on `blocking`, and it is honest about being
incomplete -- but read alone it says "done" too easily, three separate ways:

  * IT EXITS 0 WITH FOUR OF NINE COMPONENTS UNGRADED. floorplan, impedance,
    length and net_widths return `skipped(...)` when their flag is absent, and
    `ungraded` does not touch the exit code. A board nobody graded exits 0.
  * `undersized` IS SILENTLY PERMISSIVE without spec numbers. It runs at the
    FAB floor for the layer count, so `undersized == 0` is not "the sizes are
    right" -- one board had 141 of 141 vias violating a 0.6mm spec while
    clearing the 0.25mm two-layer floor.
  * IT HAS NO COMPONENT AT ALL for orphan stubs, weird copper, pad overlaps or
    channel starvation. Several of those are run by the routing chain and none
    of them reaches the scalar.

And underneath all of it, the DRC writeback rewrites the board's own
manufacturing floors down to whatever was produced and then everything grades
against the new value. That is disclosed in a printed line and enforced by
nothing.

So this aggregates, and the difference from board_score is the direction it
fails in. Three verdict classes, and only one of them is "done":

    DONE        every component that could be graded is graded and clean
    INCOMPLETE  a component RAN and could not answer, or an instrument this
                adds found something -- exit 4
    UNSOUND     the board grades clean against floors it rewrote -- exit 5

UNGRADED components do NOT block DONE -- a board with no spec files has nothing
to grade them against, and making that fatal would put every corpus board
permanently in the failing state -- but they are NAMED in the verdict line,
because a component nothing examined is UNEXAMINED and never clean.

Exit: 0 done, 2 usage, 3 board state, 4 incomplete, 5 unsound floors.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
# #522/py_placer layout: the engine modules (kicad_parser, cli_banner, ...)
# live in py_router/py_tools/py_placer, not at the repo root, so inserting
# only ROOT left `import cli_banner` failing when run as a script.
sys.path.insert(0, ROOT)
for _pkg in ('py_router', 'py_tools', 'py_placer'):
    _d = os.path.join(ROOT, _pkg)
    if os.path.isdir(_d):
        sys.path.insert(0, _d)
SKILL_SCRIPTS = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                             'scripts')
PY = [sys.executable, '-X', 'utf8']

DONE, USAGE, BOARD_STATE, INCOMPLETE, UNSOUND = 0, 2, 3, 4, 5


TIMED_OUT = 124                     # the shell's code, and deliberately so:
                                    # a caller reading 124 is reading "killed
                                    # by a clock", which is what happened.


def _run(args, timeout=3600):
    """Run a checker. A timeout is a RESULT, not an exception.

    This process is the thing a gate reads, so it must not die on the slowest
    board in the corpus: board_score alone fans out to check_drc, and on a
    6-layer board this runs six orphan-stub sweeps beside it. An uncaught
    TimeoutExpired propagates out of main and takes the whole document with it
    (see the finally in main) -- which loses the verdict on exactly the boards
    a close-out exists for.
    """
    try:
        p = subprocess.run(PY + args, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', cwd=ROOT,
                           timeout=timeout,
                           env=dict(os.environ, KRT_NO_BANNER='1'))
    except subprocess.TimeoutExpired:
        return TIMED_OUT, f'timed out after {timeout}s'
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def copper_layers(board):
    """-> (layers, error). The error is a STRING, never a raised exception.

    Returning a bare [] on a parse failure would make "this board has no copper
    layers" and "nobody could read this board" the same answer, which is the
    conflation this whole file exists to refuse. The caller records the error
    as a reason rather than sweeping a partial sweep in as a clean one.
    """
    try:
        from kicad_parser import parse_kicad_pcb
        return list(parse_kicad_pcb(board).board_info.copper_layers or []), None
    except Exception as exc:                                    # noqa: BLE001
        return [], f'{type(exc).__name__}: {exc}'


def fab_floor_integrity(board, authored_from):
    """Does this board's copper sit below the floors its project ONCE declared?

    The writeback only ever loosens: every constraint is set to min(current,
    target) and the target is the smallest object actually placed. So a board
    that routed at 0.0889 rewrites its own 0.2 minimum and then grades clean
    against it -- measured at 5629 of 5933 segments below the original floor,
    with check_drc, board_score and KiCad's own DRC all reading clean.

    Needs the ORIGINAL project to compare against, because a chain overwrites
    the floors in place. Without it, say the check did not run rather than
    implying it passed.
    """
    from fix_kicad_drc_settings import (FAB_FLOOR_KEYS, find_project,
                                        scan_board_minima)
    if not authored_from:
        return {'ran': False, 'reason': 'no --authored-from: the chain rewrites '
                                        'the floors in place, so there is '
                                        'nothing left to compare against'}
    pro = find_project(authored_from)
    if not os.path.isfile(pro):
        return {'ran': False, 'reason': f'no project beside {authored_from}'}
    try:
        with open(pro, encoding='utf-8') as fh:
            authored = ((json.load(fh).get('board') or {})
                        .get('design_settings') or {}).get('rules') or {}
    except Exception as exc:                                # noqa: BLE001
        return {'ran': False, 'reason': f'unreadable project: {exc}'}
    now = scan_board_minima(board) or {}
    relaxed = []
    unmeasured = []
    for key, label in FAB_FLOOR_KEYS:
        was, is_ = authored.get(key), now.get(key)
        if isinstance(was, (int, float)) and not isinstance(is_, (int, float)):
            # The board DECLARES this floor and nothing here can measure it --
            # scan_board_minima reads object sizes only, no pairwise geometry,
            # so copper-to-hole has no measured counterpart. Silently skipping
            # it made `relaxed: []` read as "every declared floor is honoured"
            # when one of them had never been looked at. Name it instead.
            unmeasured.append({'key': key, 'label': label, 'authored': was})
            continue
        if isinstance(was, (int, float)) and isinstance(is_, (int, float)) \
                and is_ < was - 1e-9:
            relaxed.append({'key': key, 'label': label, 'authored': was,
                            'on_board': is_})
    return {'ran': True, 'relaxed': relaxed, 'unmeasured': unmeasured,
            'authored_from': authored_from}


def _grade(a, doc):
    """Fill `doc` and return the exit code. Raises only on a genuine bug.

    Split out of main so main can own a try/finally: the document must be
    written even when this raises, because the boards that break an instrument
    are exactly the boards a close-out is being asked about.
    """
    # ---- 1. board_score, with every spec flag the caller has ---------------
    # The intermediate goes to a temp dir, not next to the board: a checker
    # that drops files into the directory it is inspecting turns a read-only
    # question into a write, and on a corpus directory that is somebody's
    # source tree.
    import tempfile
    scratch = tempfile.mkdtemp(prefix='check_complete.')
    try:
        score_path = os.path.join(scratch, 'score.json')
        args = [os.path.join(SKILL_SCRIPTS, 'board_score.py'), a.board,
                '--json', score_path, '--quiet']
        for flag, val in (('--clearance', a.clearance), ('--intent', a.intent),
                          ('--length-groups', a.length_groups),
                          ('--net-min-widths', a.net_min_widths),
                          ('--min-track-width', a.min_track_width),
                          ('--min-via-diameter', a.min_via_diameter),
                          ('--min-via-drill', a.min_via_drill)):
            if val is not None:
                args += [flag, str(val)]
        if a.impedance_nets:
            args += ['--impedance-nets'] + list(a.impedance_nets)
        code, out = _run(args, timeout=a.timeout)
        score = {}
        if os.path.isfile(score_path):
            with open(score_path, encoding='utf-8') as fh:
                score = json.load(fh)
    finally:
        shutil.rmtree(scratch, ignore_errors=True)

    doc['score'] = score
    # Hoisted so a gate can bind this document to a board by CONTENT rather
    # than by path, reusing the sha board_score already computed.
    doc['board_sha'] = score.get('board_sha')
    blocking = score.get('blocking')
    ungraded = sorted(score.get('ungraded') or [])
    unknown = sorted(score.get('unknown') or [])
    score_failed = (code == TIMED_OUT or not score)

    # ---- 1b. is there a board here at all? ---------------------------------
    # ALWAYS, including under --skip-slow. `parse_kicad_pcb` is lenient: a
    # truncated file yields an EMPTY board rather than an error, and an empty
    # board has nothing to violate, so every component grades 0 and the verdict
    # was DONE. "Nothing to grade" is not "graded clean" -- that is the whole
    # claim of this file, and it failed it on the cheapest possible input.
    # A real board always declares at least two copper layers.
    layers, lyr_err = copper_layers(a.board)
    doc['copper_layers'] = layers
    if lyr_err:
        doc['copper_layers_error'] = lyr_err

    # ---- 2. the instruments board_score has no component for ---------------
    extra = {}
    if not a.skip_slow:
        # check_orphan_stubs hardcodes four layers, so on a 6+ layer board the
        # inner ones are never scanned and it says nothing about them. Ask per
        # layer instead of accepting a partial sweep as a clean one.
        stub_hits, stub_ran, stub_exits = 0, [], {}
        for ly in layers:
            c, o = _run([os.path.join('py_tools', 'check_orphan_stubs.py'),
                         a.board, '--layer', ly],
                        timeout=a.timeout)
            stub_ran.append(ly)
            stub_exits[ly] = c
            if c == 1:
                stub_hits += 1
        extra['orphan_stubs'] = {'ran': not lyr_err, 'layers': stub_ran,
                                 'exits': stub_exits,
                                 'layers_with_orphans': stub_hits}
        if lyr_err:
            extra['orphan_stubs']['error'] = lyr_err
        # 1 is "orphans found"; 0 is clean. ANY OTHER CODE IS THE INSTRUMENT
        # FAILING, not the board passing -- a traceback also exits 1 and an
        # argparse error exits 2, and 2 used to be counted as clean because
        # only `c == 1` was tested. The siblings below use `clean = c == 0`,
        # which fails in the safe direction; this now matches them.
        bad = sorted(ly for ly, c in stub_exits.items() if c not in (0, 1))
        if bad:
            extra['orphan_stubs']['failed_layers'] = bad
        for name, cmd in (
                ('weird_copper', [os.path.join('py_router', 'check_weird.py'),
                                  a.board]),
                ('pad_overlaps', [os.path.join('py_router', 'check_pads.py'),
                                  a.board, '--cross-footprint'])):
            c, o = _run(cmd, timeout=a.timeout)
            extra[name] = {'ran': True, 'exit': c, 'clean': c == 0}
    doc['components'] = extra

    # ---- 3. the fab floors this board grades itself against ----------------
    try:
        floors = fab_floor_integrity(a.board, a.authored_from)
    except Exception as exc:                                    # noqa: BLE001
        floors = {'ran': False,
                  'reason': f'the floor check itself failed: '
                            f'{type(exc).__name__}: {exc}'}
    doc['fab_floors'] = floors

    # ---- the verdict -------------------------------------------------------
    reasons = []
    if lyr_err:
        reasons.append(f'the board did not parse: {lyr_err}')
    elif not layers:
        reasons.append('this file declares NO copper layers -- it did not '
                       'parse into a board. Every component then grades 0 '
                       'against nothing, which is not a clean board')
    if score_failed:
        reasons.append('board_score produced no score'
                       + (f' (exit {code})' if code == TIMED_OUT else ''))
    if unknown:
        reasons.append('a component RAN and could not answer: '
                       + ', '.join(unknown))
    if blocking is None and not score_failed:
        reasons.append('blocking is null -- something was not graded, and a '
                       'null is not a zero')
    elif blocking:
        reasons.append(f'blocking = {blocking}')
    for name, r in sorted(extra.items()):
        if r.get('error'):
            reasons.append(f'{name}: did not run ({r["error"]})')
        elif r.get('failed_layers'):
            reasons.append(f'{name}: the instrument failed on '
                           f'{", ".join(r["failed_layers"])} -- '
                           f'not examined, and not clean')
        elif r.get('layers_with_orphans'):
            reasons.append(f'{name}: {r["layers_with_orphans"]} layer(s) carry '
                           f'orphan stubs')
        elif r.get('clean') is False:
            reasons.append(f'{name}: not clean (exit {r.get("exit")})')

    # An UNMEASURED declared floor is not a pass. It does not make the board
    # UNSOUND on its own -- nothing measured a violation -- but it must not
    # vanish, because `fab_floors.relaxed == []` is exactly the sentence a
    # reader takes for "every declared floor is honoured". Append it to the
    # INCOMPLETE reasons so it travels with the verdict.
    if floors.get('unmeasured'):
        _um = ', '.join(f'{r["label"]} (declared {r["authored"]})'
                        for r in floors['unmeasured'])
        reasons.append(f'declared fab floor(s) NOT measured by this check, so '
                       f'unknown rather than honoured: {_um} -- grade with '
                       f'check_drc.py, which reads them')
    if floors.get('relaxed'):
        worst = ', '.join(f'{r["label"]} {r["authored"]} -> {r["on_board"]}'
                          for r in floors['relaxed'])
        verdict, code_out = 'UNSOUND', UNSOUND
        why = (f'this board carries copper below floors its project once '
               f'declared ({worst}). Every checker grades against the NEW '
               f'value, so a clean report here means the rule moved, not the '
               f'copper. Confirm the fab supports it before calling it done.')
    elif reasons:
        verdict, code_out = 'INCOMPLETE', INCOMPLETE
        why = '; '.join(reasons)
    else:
        verdict, code_out = 'DONE', DONE
        why = 'every component that could be graded is graded and clean'

    if ungraded:
        why += ('. UNEXAMINED, and not passed: ' + ', '.join(ungraded)
                + ' -- nothing was asked to grade them, so they are unknown '
                  'rather than clean')
    doc['verdict'] = verdict
    doc['reason'] = why
    doc['ungraded'] = ungraded
    return code_out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board')
    ap.add_argument('--authored-from', default=None,
                    help='the board the chain STARTED from -- its project '
                         'carries the floors this one should still respect')
    ap.add_argument('--clearance', type=float, default=None)
    ap.add_argument('--intent', default=None)
    ap.add_argument('--impedance-nets', nargs='+', default=None)
    ap.add_argument('--length-groups', default=None)
    ap.add_argument('--net-min-widths', default=None)
    ap.add_argument('--min-track-width', type=float, default=None)
    ap.add_argument('--min-via-diameter', type=float, default=None)
    ap.add_argument('--min-via-drill', type=float, default=None)
    ap.add_argument('--skip-slow', action='store_true',
                    help='the added instruments only; board_score always runs. '
                         'NOTE this empties `components`, so the document is '
                         'then weaker than the board_score it wraps -- a gate '
                         'reading it should say so')
    ap.add_argument('--timeout', type=float, default=3600, metavar='S',
                    help='per-instrument wall clock (default 3600). A timeout '
                         'is recorded as exit 124, never raised')
    ap.add_argument('--json', default=None, metavar='PATH')
    a = ap.parse_args(argv)

    if not os.path.isfile(a.board):
        print(f'no such board: {a.board}', file=sys.stderr)
        return BOARD_STATE

    # Absolute, because _run uses cwd=ROOT. A relative board path resolves
    # against ROOT inside the subprocess and against the caller's cwd out here,
    # so from any directory but the repo root board_score was handed a path
    # that does not exist -- and returned an empty score, which became a FALSE
    # `INCOMPLETE: blocking is null`. `doc['board']` was abspathed already, so
    # the document even looked correctly bound. The existing test does
    # os.chdir(ROOT), which is why this never surfaced.
    a.board = os.path.abspath(a.board)
    if a.authored_from:
        a.authored_from = os.path.abspath(a.authored_from)

    doc = {'schema': 1, 'kind': 'board-complete',
           'board': a.board, 'board_sha': None, 'components': {}, 'score': {},
           'fab_floors': {}, 'ungraded': [],
           'verdict': 'INCOMPLETE',
           'reason': 'the close-out itself did not finish'}

    code_out = INCOMPLETE
    try:
        code_out = _grade(a, doc)
    except Exception as exc:                                    # noqa: BLE001
        # An instrument that dies must not take the document with it. This
        # file's own doctrine -- "say the check did not run rather than
        # implying it passed" -- was written for fab_floor_integrity and never
        # applied to its own failure modes: the --json write used to be the
        # last statement, so anything above it that raised produced no
        # document at all, on precisely the broken boards a gate is for.
        doc['verdict'] = 'INCOMPLETE'
        doc['reason'] = (f'the close-out itself did not finish: '
                         f'{type(exc).__name__}: {exc}')
        code_out = INCOMPLETE
    finally:
        print(f"VERDICT: {doc['verdict']} -- {doc['reason']}")
        if a.json:
            try:
                with open(a.json, 'w', encoding='utf-8') as fh:
                    json.dump(doc, fh, indent=1, sort_keys=True, default=str)
                print(f'  JSON -> {a.json}')
            except OSError as exc:
                print(f'  could not write {a.json}: {exc}', file=sys.stderr)
    return code_out


if __name__ == '__main__':
    import cli_banner
    cli_banner.install()
    sys.exit(main())
