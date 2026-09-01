"""
check_floorplan.py: the exit-code contract and the JSON surface (issue #549).

Subprocess tests, because the things that matter here are what a CALLER sees --
the exit code it branches on, and whether two runs produce the same bytes. An
in-process test cannot check either honestly: `main()` returning 4 proves
nothing if `__main__` drops the value (place_optimize.py:146 does exactly that,
issue #551), and determinism across hash seeds needs separate interpreters.

The exit codes are a contract, not a detail:

    0  graded clean (or --exit-zero)     3  board state / no usable outline
    2  argparse, incl. a malformed intent  4  graded, violations found

4 rather than 1 because 3 is already taken and 1 is ambiguous with a crash. A
caller has to be able to tell "the floorplan is wrong" from "the grader is
broken" -- that distinction is the whole product.
"""

import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOL = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')

VIOLATIONS_EXIT = 4
STATE_EXIT = 3

_TMP = tempfile.mkdtemp(prefix='fp549_')
_EMITTED = os.path.join(_TMP, 'emitted.json')

ROUND_BOARD = (
    '(kicad_pcb (version 20221018) (generator pcbnew)\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
    '  (gr_circle (center 50 50) (end 70 50) (stroke (width 0.1)'
    ' (type default)) (layer "Edge.Cuts"))\n  (net 0 "")\n)\n')

# No Edge.Cuts geometry at all -> board_bounds is None and there is nothing to
# contain anything against. This is the case the exit-3 guard exists for.
NO_EDGE_BOARD = (
    '(kicad_pcb (version 20221018) (generator pcbnew)\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
    '  (net 0 "")\n)\n')


def _run(*argv, env=None):
    e = dict(os.environ)
    e.setdefault('PYTHONIOENCODING', 'utf-8')
    if env:
        e.update(env)
    r = subprocess.run([sys.executable, '-X', 'utf8', TOOL] + list(argv),
                       capture_output=True, text=True, cwd=ROOT, env=e)
    return r.returncode, r.stdout, r.stderr


def _summary(stdout):
    for line in stdout.splitlines():
        if line.startswith('JSON_SUMMARY: '):
            return json.loads(line[len('JSON_SUMMARY: '):])
    raise AssertionError('no JSON_SUMMARY line')


def _emit_once():
    if not os.path.exists(_EMITTED):
        code, out, err = _run(BOARD, '--emit-intent', _EMITTED)
        assert code == 0, (code, err)
    return _EMITTED


def test_emit_then_grade_is_clean_and_is_honest_about_what_it_graded():
    """BOARD is ulx3s, and its emitted intent WITHHOLDS `overlap_area`: the
    board carries 18 unwaived courtyard interpenetrations and an auto-budget
    would bless them. So this round trip produces zero errors AND an ungraded
    declared channel.

    Before #713 item 5 that combination printed `PASS: 5 rule(s) ran, no
    violations` twenty-six lines above `1 declared value(s) NOT DERIVABLE --
    not graded, not passed`, reported `pass: true`, and exited 0. This test
    asserted exactly that, so it was pinning the defect. It now asserts the
    two halves it always meant: the rules find nothing wrong, and the run says
    plainly that it did not grade everything.
    """
    code, out, err = _run(BOARD, '--emit-intent', _EMITTED)
    assert code == 0, (code, err)
    doc = json.load(open(_EMITTED, encoding='utf-8'))
    assert doc['kind'] == 'floorplan-intent'
    assert 'not editable by this toolchain' in out, out

    code, out, err = _run(BOARD, '--intent', _EMITTED)
    s = _summary(out)
    assert s['errors'] == 0 and s['violations'] == 0, s
    assert s['complete'] is False and s['not_graded'] == {
        'budget_abstained': 1}, s
    assert s['pass'] is False, "an ungraded declared channel is not a pass"
    assert code == VIOLATIONS_EXIT, (code, err)
    assert 'INCOMPLETE' in out and 'PASS:' not in out, out
    # --exit-zero suppresses the CODE without lying about the VERDICT, the
    # same contract it already has for violations.
    code, out, _ = _run(BOARD, '--intent', _EMITTED, '--exit-zero')
    assert code == 0 and _summary(out)['pass'] is False
    print(f"  PASS: emit -> grade is clean but INCOMPLETE, "
          f"{s['rules_run']} rules ran, exit {VIOLATIONS_EXIT}")


def test_violations_exit_4_and_exit_zero_overrides():
    raw = json.load(open(_emit_once(), encoding='utf-8'))
    raw['must_lock'] = ['R1']
    p = os.path.join(_TMP, 'unlocked.json')
    json.dump(raw, open(p, 'w', encoding='utf-8'))

    code, out, _ = _run(BOARD, '--intent', p, '-q')
    assert code == VIOLATIONS_EXIT, code
    s = _summary(out)
    assert s['pass'] is False and s['errors'] >= 1
    assert s['violations_by_rule'].get('must_lock') == 1, s['violations_by_rule']

    code, out, _ = _run(BOARD, '--intent', p, '-q', '--exit-zero')
    assert code == 0, code
    assert _summary(out)['pass'] is False, "the verdict changed with --exit-zero"

    # A board can be BOTH wrong and incompletely graded, and the second fact
    # must not be swallowed by the first. ulx3s is exactly that case here: a
    # must_lock violation AND a withheld overlap_area. The first draft of
    # #713 item 5 returned on `errors` before it reached the completeness
    # block, and printed the INCOMPLETE line only on the no-violation branch,
    # so this board reported the errors and said nothing about what it never
    # measured.
    code, out, err = _run(BOARD, '--intent', p)
    s2 = _summary(out)
    assert s2['errors'] >= 1 and s2['complete'] is False, s2
    assert 'ALSO NOT FULLY GRADED' in out, out[-600:]
    assert 'NOT FULLY GRADED' in err, err
    print("  PASS: 4 on violations; --exit-zero returns 0 without lying "
          "about `pass`; a wrong board still discloses what went ungraded")


def test_argparse_failures_all_exit_2():
    broken = os.path.join(_TMP, 'broken.json')
    open(broken, 'w', encoding='utf-8').write('{"schema": 1,')
    wrongkind = os.path.join(_TMP, 'sidecar.json')
    json.dump({'schema': 1, 'round': 2, 'accepted': True},
              open(wrongkind, 'w', encoding='utf-8'))
    cases = {
        'no --intent and no --emit-intent': (BOARD,),
        'truncated intent': (BOARD, '--intent', broken),
        'missing intent file': (BOARD, '--intent',
                                os.path.join(_TMP, 'nope.json')),
        'a round sidecar as the intent': (BOARD, '--intent', wrongkind),
        'unknown --group-by source': (BOARD, '--intent', _emit_once(),
                                      '--group-by', 'bogus'),
        'missing board': ('nosuch.kicad_pcb', '--intent', _emit_once()),
    }
    for why, argv in cases.items():
        code, _out, _err = _run(*argv)
        assert code == 2, f"{why}: exit {code}, expected 2"
    print(f"  PASS: {len(cases)} malformed invocations all exit 2")


def test_a_round_board_now_grades_normally():
    """#550, FIXED SIDE: a round board's gr_circle IS its outline.

    This test used to assert the opposite -- that a round board reports
    board_bounds None and must exit 3 rather than silently grade against a
    bounding box that does not exist. `extract_board_bounds` then learned to
    read gr_circle/gr_curve (#550, 8bc8ea8), so the ring now yields real bounds
    and the degrade-to-nothing hazard is gone for this shape. The guard itself
    still matters, so it moved to the genuinely-outline-less board below.
    """
    p = os.path.join(_TMP, 'round.kicad_pcb')
    open(p, 'w', encoding='utf-8').write(ROUND_BOARD)
    code, _out, err = _run(p, '--emit-intent', os.path.join(_TMP, 'x.json'))
    assert code == 0, (code, err)
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
    from kicad_parser import parse_kicad_pcb
    b = parse_kicad_pcb(p).board_info.board_bounds
    assert b == (30.0, 30.0, 70.0, 70.0), b
    print(f"  PASS: round board grades; gr_circle bounds {b} (#550)")


def test_a_board_with_no_usable_outline_exits_3_not_0():
    """A board with NO Edge.Cuts geometry has no bounding box at all. Every
    containment check would silently degrade to a box that does not exist, so a
    clean report would mean 'stopped checking' -- exit 3, never 0."""
    p = os.path.join(_TMP, 'no_edge.kicad_pcb')
    open(p, 'w', encoding='utf-8').write(NO_EDGE_BOARD)
    for argv in ((p, '--emit-intent', os.path.join(_TMP, 'x.json')),
                 (p, '--intent', _emit_once())):
        code, _out, err = _run(*argv)
        assert code == STATE_EXIT, (argv[1], code, err)
        assert 'Edge.Cuts' in err, err
    print("  PASS: emit and grade both exit 3 on a board with no outline")


def test_json_output_is_byte_identical_across_hash_seeds():
    """#457. Anything whose order can reach a report must be a property of the
    finding, not of dict iteration -- and only separate interpreters can show
    it."""
    outs = []
    for seed in ('0', '12345'):
        p = os.path.join(_TMP, f'out{seed}.json')
        # KRT_NO_BANNER: the B1 CMD echo would legitimately differ between
        # these two runs (their --json paths differ) -- this test compares
        # hash-seed determinism of the REPORT, not the banner.
        code, stdout, err = _run(BOARD, '--intent', _emit_once(), '-q',
                                 '--json', p, env={'PYTHONHASHSEED': seed,
                                                   'KRT_NO_BANNER': '1'})
        # 4, not 0: ulx3s's emitted intent withholds `overlap_area`, so this
        # round trip is clean but INCOMPLETE (#713 item 5). The determinism
        # claim is about the BYTES and is unaffected either way.
        assert code == VIOLATIONS_EXIT, (seed, err)
        outs.append((open(p, 'rb').read(), stdout))
    assert outs[0][0] == outs[1][0], "--json differs across PYTHONHASHSEED"
    assert outs[0][1] == outs[1][1], "JSON_SUMMARY differs across PYTHONHASHSEED"
    print(f"  PASS: {len(outs[0][0])} bytes identical at seeds 0 and 12345")


def test_the_summary_reports_what_did_not_run():
    """Anti-vacuity for machines: `0 violations` and `0 rules ran` must not look
    the same. A caller that only reads `pass` would treat a fully-skipped grade
    as a clean board."""
    minimal = os.path.join(_TMP, 'minimal.json')
    json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'},
              open(minimal, 'w', encoding='utf-8'))
    code, out, _ = _run(BOARD, '--intent', minimal, '-q')
    assert code == 0
    s = _summary(out)
    assert s['pass'] is True and s['violations'] == 0
    assert s['rules_run'] == 0, s['rules_run']
    # A hand-stated literal, deliberately: deriving it from `len(RULES)`
    # would make this arm agree with whatever the module currently says
    # and stop being a change detector. 9 before #794 added
    # `decap_ungraded` and #705 added `decap_pin_distance`.
    assert s['rules_skipped'] == 11, s['rules_skipped']
    print(f"  PASS: an empty intent passes with rules_run=0, "
          f"rules_skipped={s['rules_skipped']} -- visibly vacuous")


def test_the_summary_carries_the_keys_a_caller_branches_on():
    code, out, _ = _run(BOARD, '--intent', _emit_once(), '-q')
    s = _summary(out)
    for k in ('pass', 'violations', 'errors', 'warnings', 'violations_by_rule',
              'rules_run', 'rules_skipped', 'blocks', 'blocks_resolved',
              'parts_covered', 'parts_total', 'cutouts', 'edge_contours',
              'overlap_area', 'oob_count', 'oob_amount', 'hpwl',
              'state_unplaced', 'state_partially_unplaced',
              'state_duplicate_fraction', 'state_spread_ratio',
              'state_outside_fraction'):
        assert k in s, f"JSON_SUMMARY lost {k}"
    json.dumps(s)
    print(f"  PASS: {len(s)} keys, all JSON-round-trippable")


def test_grading_knobs_resolve_from_the_board_and_are_disclosed():
    """Run-7 S1: a fixed 0.25 default on a 0.15-floor board manufactured
    phantom oob findings. Unset knobs must resolve from the board and the
    summary must say what was used, with its source."""
    code, out, _ = _run(BOARD, '--intent', _emit_once(), '-q')
    s = _summary(out)
    for k in ('clearance_used', 'edge_clearance_used'):
        assert k in s, f"JSON_SUMMARY lost {k}"
        assert 'value' in s[k] and 'source' in s[k], s[k]
        assert s[k]['source'] in ('cli', 'board netclass', 'board constraint',
                                  'fixed default'), s[k]
    code, out, _ = _run(BOARD, '--intent', _emit_once(), '-q',
                        '--clearance', '0.11')
    s = _summary(out)
    assert s['clearance_used'] == {'value': 0.11, 'source': 'cli'}, (
        "an explicit --clearance must win and be labeled cli")
    print("  PASS: knobs resolve board-first and the summary discloses them")


def test_the_full_json_carries_the_measured_numbers():
    raw = json.load(open(_emit_once(), encoding='utf-8'))
    x0, y0, x1, y1 = raw['envelope']['rect']
    raw['envelope']['rect'] = [x0, y0, x1 + 7.5, y1]
    p = os.path.join(_TMP, 'wide.json')
    json.dump(raw, open(p, 'w', encoding='utf-8'))
    full = os.path.join(_TMP, 'full.json')
    code, _out, _ = _run(BOARD, '--intent', p, '-q', '--json', full)
    assert code == VIOLATIONS_EXIT, code
    doc = json.load(open(full, encoding='utf-8'))
    v = [x for x in doc['violations'] if x['rule'] == 'envelope']
    assert len(v) == 1, doc['violations']
    assert abs(v[0]['measured']['worst_corner_mm'] - 7.5) < 1e-6
    assert 'do not resize' in v[0]['message']
    # the outline block is reported whether or not anything was wrong with it
    assert doc['outline']['bounds'] and 'cutouts' in doc['outline']
    assert doc['state']['unplaced'] is False
    print(f"  PASS: measured 7.50mm, and the message refuses the resize")


def test_emit_writes_no_board_and_touches_nothing():
    """A report-only tool that mutates its input is the failure the whole
    design rests on not having."""
    before = (os.path.getmtime(BOARD), os.path.getsize(BOARD))
    listing = set(os.listdir(os.path.dirname(BOARD)))
    p = os.path.join(_TMP, 'emit2.json')
    code, _out, _ = _run(BOARD, '--emit-intent', p)
    assert code == 0
    assert (os.path.getmtime(BOARD), os.path.getsize(BOARD)) == before, \
        "the input board was modified"
    assert set(os.listdir(os.path.dirname(BOARD))) == listing, \
        "a stray file was written beside the board"
    print("  PASS: input board mtime/size unchanged, no stray siblings")


def test_a_placement_run_leaves_the_board_outline_untouched():
    """The invariant the whole outline rule rests on, asserted rather than
    assumed.

    It is true today by construction -- no writer in the routing chain emits an
    Edge.Cuts primitive, and `strip_zero_length_edge_cuts` removes only
    degenerate segments that bound nothing. But "true by construction" is
    exactly the kind of property that stops being true silently, so it gets a
    test rather than a comment.

    watchy is the right board for it: 2 real interior cutouts, and 81 of its 82
    parts seeded in courtyard violation, so the optimizer has every reason to
    move things around.
    """
    import shutil
    src = os.path.join(ROOT, 'kicad_files', 'watchy.kicad_pcb')
    if not os.path.exists(src):
        print("  SKIP: watchy absent")
        return
    work = os.path.join(_TMP, 'watchy.kicad_pcb')
    shutil.copy2(src, work)
    for ext in ('.kicad_pro', '.kicad_dru'):
        sib = src[:-len('.kicad_pcb')] + ext
        if os.path.exists(sib):
            shutil.copy2(sib, work[:-len('.kicad_pcb')] + ext)
    out = os.path.join(_TMP, 'watchy_placed.kicad_pcb')

    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'place_optimize.py'), work, out,
         '--max-displacement', '2', '--align-weight', '5',
         '--orient-weight', '1'],
        capture_output=True, text=True, cwd=ROOT)
    assert r.returncode == 0, r.stderr[-800:]

    sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
    from kicad_parser import parse_kicad_pcb
    a = parse_kicad_pcb(work).board_info
    b = parse_kicad_pcb(out).board_info
    assert a.board_bounds == b.board_bounds, \
        f"the board was RESIZED: {a.board_bounds} -> {b.board_bounds}"
    assert len(a.board_cutouts) == len(b.board_cutouts), \
        f"a cutout was lost: {len(a.board_cutouts)} -> {len(b.board_cutouts)}"
    assert a.board_cutouts == b.board_cutouts, "a cutout moved"
    assert a.board_outlines == b.board_outlines, "the outline moved"
    assert len(a.board_cutouts) >= 2, "the fixture stopped having cutouts"
    print(f"  PASS: bounds, {len(a.board_cutouts)} cutouts and "
          f"{len(a.board_outlines)} outline ring(s) all identical through a "
          f"placement run with both #548 terms on")


TESTS = [
    test_emit_then_grade_is_clean_and_is_honest_about_what_it_graded,
    test_a_placement_run_leaves_the_board_outline_untouched,
    test_violations_exit_4_and_exit_zero_overrides,
    test_argparse_failures_all_exit_2,
    test_a_board_with_no_usable_outline_exits_3_not_0,
    test_json_output_is_byte_identical_across_hash_seeds,
    test_the_summary_reports_what_did_not_run,
    test_the_summary_carries_the_keys_a_caller_branches_on,
    test_the_full_json_carries_the_measured_numbers,
    test_emit_writes_no_board_and_touches_nothing,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
