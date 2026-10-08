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
                (a floor the --authored-from board's own copper breaks at
                least as badly is named in the reason instead: the
                reference's, not this board's)

UNGRADED components do NOT block DONE -- a board with no spec files has nothing
to grade them against, and making that fatal would put every corpus board
permanently in the failing state -- but they are NAMED in the verdict line,
because a component nothing examined is UNEXAMINED and never clean.

Exit: 0 done, 2 usage, 3 board state, 4 incomplete, 5 unsound floors.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

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
# board_score is a repo instrument, in py_tools/ with the checkers it runs. It
# used to live inside a skill's scripts/ dir, and every time that skill moved
# this lookup broke with "board_score produced no score" -- an INCOMPLETE
# verdict caused by a missing file rather than by the board.
SKILL_SCRIPTS = os.path.join(ROOT, 'py_tools')
PY = [sys.executable, '-X', 'utf8']

DONE, USAGE, BOARD_STATE, INCOMPLETE, UNSOUND = 0, 2, 3, 4, 5


TIMED_OUT = 124                     # the shell's code, and deliberately so:
                                    # a caller reading 124 is reading "killed
                                    # by a clock", which is what happened.


def _score_names(score, key):
    """`score[key]` as sorted names, and why not when it is not a list.

    board_score writes a list. Anything else names no component -- the rule
    converge's verdict follows (#1076): `sorted("abc")` read as three
    components nobody examined, and `sorted(5)` raised.
    """
    v = score.get(key)
    if v is None:
        return [], None
    if isinstance(v, (list, tuple, set)):
        return sorted(str(x) for x in v), None
    return [], f'board_score wrote an `{key}` that is not a list ({v!r})'


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


#: The objects a measurable fab floor is a statement about (#1198).
_FLOOR_OBJECT = {
    'min_track_width': 'track',
    'min_via_diameter': 'via', 'min_via_annular_width': 'via',
    'min_via_drill': 'via',
    'min_through_hole_diameter': 'drilled hole',
}

#: Slack when one measured minimum is compared with another. KiCad reports an
#: actual distance to 4 decimals (``actual 0.1540 mm``).
_MIN_TOL = 1e-4


def kicad_hole_clearance(board, value, timeout=3600):
    """KiCad's own copper-to-hole grade of ``board`` at ``value`` mm (#1198).

    A staged copy (the board and its siblings) whose project declares
    ``min_hole_clearance = value`` at error severity, through kicad-cli with
    zones refilled -- KiCad refills before fab, and the stored fill was poured
    at whatever value the chain lowered the rule to. KiCad's hole_clearance
    covers every hole: via drills, PTH barrels and NPTH, which check_drc's hole
    arms (NPTH only) do not.

    -> ``{'count': N, 'min_actual': mm or None}``, or ``{'error': why}`` when
    kicad-cli is missing or produced no report. Never raises."""
    import re
    import tempfile
    from copy_board import copy_board
    try:
        from kicad_oracle import find_kicad_cli
        cli = find_kicad_cli(warn=False)
    except Exception as exc:                                    # noqa: BLE001
        return {'error': f'no kicad-cli lookup: {exc}'}
    if not cli:
        return {'error': 'kicad-cli not found (set KICAD_CLI)'}
    tmp = tempfile.mkdtemp(prefix='check_complete_hole.')
    try:
        dst = os.path.join(tmp, 'b.kicad_pcb')
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(board, dst)
        pro = os.path.join(tmp, 'b.kicad_pro')
        proj = {}
        if os.path.isfile(pro):
            with open(pro, encoding='utf-8') as fh:
                proj = json.load(fh)
        ds = proj.setdefault('board', {}).setdefault('design_settings', {})
        ds.setdefault('rules', {})['min_hole_clearance'] = float(value)
        ds.setdefault('rule_severities', {})['hole_clearance'] = 'error'
        with open(pro, 'w', encoding='utf-8') as fh:
            json.dump(proj, fh, indent=2)
        out = os.path.join(tmp, 'drc.json')
        try:
            r = subprocess.run([cli, 'pcb', 'drc', '--format', 'json',
                                '--units', 'mm', '--severity-all',
                                '--refill-zones', '--output', out, dst],
                               capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return {'error': f'kicad-cli timed out after {timeout:g} s'}
        if not os.path.isfile(out) or os.path.getsize(out) == 0:
            return {'error': f'kicad-cli produced no report (rc={r.returncode}: '
                             f'{r.stderr.strip()[:200]})'}
        with open(out, encoding='utf-8') as fh:
            report = json.load(fh)
    except Exception as exc:                                    # noqa: BLE001
        return {'error': f'{type(exc).__name__}: {exc}'}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    actual = []
    count = 0
    for v in report.get('violations', []):
        if v.get('type') != 'hole_clearance' or v.get('severity') != 'error':
            continue
        count += 1
        m = re.search(r'actual\s+(-?[0-9.]+)\s*mm', v.get('description') or '')
        if m:
            actual.append(float(m.group(1)))
    return {'count': count, 'min_actual': min(actual) if actual else None}


def fab_floor_integrity(board, authored_from, hole_grader=None, timeout=3600):
    """Does this board's copper sit below the floors its project ONCE declared?

    The writeback only ever loosens: every constraint is set to min(current,
    target) and the target is the smallest object actually placed. So a board
    that routed at 0.0889 rewrites its own 0.2 minimum and then grades clean
    against it -- measured at 5629 of 5933 segments below the original floor,
    with check_drc, board_score and KiCad's own DRC all reading clean.

    Needs the ORIGINAL project to compare against, because a chain overwrites
    the floors in place. Without it, say the check did not run rather than
    implying it passed.

    The original board is also the REFERENCE (#1198): a floor its own copper
    breaks, at least as badly as this board does, is ``reference_too`` --
    reported, not counted against this board. A board that breaks it worse,
    or a floor the reference holds, is ``relaxed``. Copper-to-hole has no
    measured counterpart in scan_board_minima, so when this board's project
    lowered it, ``hole_grader(board, mm)`` (default: KiCad's own grade,
    :func:`kicad_hole_clearance`) measures both boards at the authored value.
    """
    from fix_kicad_drc_settings import (FAB_FLOOR_KEYS,
                                        FAB_FLOOR_KEYS_MEASURABLE,
                                        find_project, scan_board_minima)
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
    try:
        with open(find_project(board), encoding='utf-8') as fh:
            declared_now = ((json.load(fh).get('board') or {})
                            .get('design_settings') or {}).get('rules') or {}
    except (OSError, ValueError):
        declared_now = {}
    try:
        ref_min = scan_board_minima(authored_from) or {}
    except Exception:                                           # noqa: BLE001
        ref_min = {}
    grade_holes = hole_grader or (
        lambda b, v: kicad_hole_clearance(b, v, timeout=timeout))
    relaxed = []
    unmeasured = []
    vacuous = []
    declared_kept = []
    measured_kept = []
    reference_too = []
    for key, label in FAB_FLOOR_KEYS:
        was, is_ = authored.get(key), now.get(key)
        if not isinstance(was, (int, float)) or isinstance(was, bool):
            continue
        if was <= 0:
            # #1198: a declared 0 is honoured by any board (ecc83 and
            # complex_hierarchy declare copper-to-hole 0.0).
            vacuous.append({'key': key, 'label': label, 'authored': was,
                            'why': 'declared 0: nothing can sit below it'})
            continue
        if key in FAB_FLOOR_KEYS_MEASURABLE:
            if not isinstance(is_, (int, float)):
                # scan_board_minima emits a key only when the board has that
                # kind of object, so a missing one is a floor held vacuously:
                # a 0-via board breaks no via floor (#1198).
                vacuous.append({'key': key, 'label': label, 'authored': was,
                                'why': f'no {_FLOOR_OBJECT.get(key, "object")} '
                                       f'on the board'})
            elif is_ < was - 1e-9:
                ref = ref_min.get(key)
                row = {'key': key, 'label': label, 'authored': was,
                       'on_board': is_}
                if isinstance(ref, (int, float)) and ref < was - 1e-9 \
                        and is_ >= ref - _MIN_TOL:
                    row['reference'] = ref
                    reference_too.append(row)
                else:
                    relaxed.append(row)
            continue
        # Copper-to-hole: scan_board_minima reads object sizes, no pairwise
        # geometry, so it has no measured counterpart (#1198). A project that
        # still declares the authored floor is graded at it by whatever grades
        # copper-to-hole (KiCad's DRC; check_drc's hole arms cover NPTH
        # only): declared vs declared, nothing to report. One that LOWERED it
        # is graded here at the authored value, by KiCad, on this board and
        # then on the reference. Unmeasurable (no kicad-cli) it stays
        # UNKNOWN, not honoured: silently skipping it made `relaxed: []` read
        # as "every declared floor is honoured" when it was never looked at,
        # so it is named and keeps the verdict INCOMPLETE.
        dec = declared_now.get(key)
        if isinstance(dec, (int, float)) and dec >= was - 1e-9:
            declared_kept.append({'key': key, 'label': label, 'authored': was,
                                  'declared': dec})
            continue
        row = {'key': key, 'label': label, 'authored': was, 'declared': dec}
        g = grade_holes(board, was) or {'error': 'no grade'}
        if g.get('error'):
            row['why'] = f'not measured: {g["error"]}'
            unmeasured.append(row)
            continue
        row.update(measured_by='kicad-cli', violations=g['count'])
        if not g['count']:
            measured_kept.append(row)
            continue
        row['on_board'] = g.get('min_actual')
        rg = grade_holes(authored_from, was) or {'error': 'no grade'}
        if rg.get('error'):
            row['reference_error'] = rg['error']
        else:
            row['reference_violations'] = rg['count']
            row['reference'] = rg.get('min_actual')
        if rg.get('count') and row['on_board'] is not None \
                and row['reference'] is not None \
                and row['on_board'] >= row['reference'] - _MIN_TOL:
            reference_too.append(row)
        else:
            relaxed.append(row)
    # #1160: the Default CLASS clearance, declared vs declared (no measured
    # counterpart: clearance is pairwise). Counted only when an automatic
    # descent lowered it -- the board's project carries the writeback's
    # class_clearance_relaxed record -- because a class lowered by the
    # chain's own --clearance / --clearance-ceiling is a decision, not a
    # relaxation (stock classes are aspirational).
    try:
        from fab_tiers import project_default_class_clearance
        from fix_kicad_drc_settings import CLASS_CLEARANCE_RELAXED_KEY
        with open(pro, encoding='utf-8') as fh:
            _cls_was = project_default_class_clearance(json.load(fh))
        _bpro = find_project(board)
        with open(_bpro, encoding='utf-8') as fh:
            _bproj = json.load(fh)
        _cls_now = project_default_class_clearance(_bproj)
        _rec = (_bproj.get('kicad_routing_tools') or {}).get(
            CLASS_CLEARANCE_RELAXED_KEY)
        if _rec and _cls_was is not None and _cls_now is not None \
                and _cls_now < _cls_was - 1e-9:
            relaxed.append({'key': 'net_class.Default.clearance',
                            'label': 'Default class clearance (lowered by an '
                                     'automatic descent on '
                                     + ', '.join(_rec.get('nets') or ['?']) + ')',
                            'authored': _cls_was, 'on_board': _cls_now})
    except (OSError, ValueError):
        pass
    return {'ran': True, 'relaxed': relaxed, 'unmeasured': unmeasured,
            'vacuous': vacuous, 'declared_kept': declared_kept,
            'measured_kept': measured_kept, 'reference_too': reference_too,
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
    ungraded, ungraded_bad = _score_names(score, 'ungraded')
    unknown, unknown_bad = _score_names(score, 'unknown')
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
        floors = fab_floor_integrity(a.board, a.authored_from,
                                     timeout=a.timeout)
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
    reasons += [r for r in (ungraded_bad, unknown_bad) if r]
    # ONE rule for what a `blocking` is (`converge.blocking_defect`, #1075):
    # `false` is falsy, so it used to read as a clean board and exit DONE.
    from converge import blocking_defect
    _bdef = blocking_defect(blocking)
    if _bdef:
        reasons.append(f'blocking is {_bdef}')
    elif blocking is None and not score_failed:
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
        _um = ', '.join(f'{r["label"]} (declared {r["authored"]}'
                        + (f', the project now declares {r["declared"]}'
                           if r.get('declared') is not None else '')
                        + (f'; {r["why"]}' if r.get('why') else '')
                        + ')'
                        for r in floors['unmeasured'])
        reasons.append(f'declared fab floor(s) lowered in the project and NOT '
                       f'measured by this check, so unknown rather than '
                       f'honoured: {_um} -- grade the copper with KiCad\'s DRC '
                       f'at the authored value (check_drc\'s hole arms cover '
                       f'NPTH only)')
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

    if floors.get('reference_too'):
        def _ref(r):
            n = (f' over {r["violations"]}' if 'violations' in r else '')
            rn = (f' over {r["reference_violations"]}'
                  if 'reference_violations' in r else '')
            return (f'{r["label"]} {r["authored"]} (this board {r["on_board"]}'
                    f'{n}, the reference {r["reference"]}{rn})')
        why += ('. DECLARED, AND BROKEN BY THE REFERENCE TOO, at least as '
                'badly (reported, not counted against this board): '
                + ', '.join(_ref(r) for r in floors['reference_too']))
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
