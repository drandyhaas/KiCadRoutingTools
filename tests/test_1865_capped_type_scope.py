"""#1865: a violation type kicad-cli cut short holds only the rules that
report under it.

`check_complete --kicad-rules` reads a rule with 0 KiCad rows as met unless
the report may have dropped a row naming it. It used to say so for EVERY
such rule as soon as ANY violation type reached 199 rows: 199 `track_width`
rows turned a `hole_clearance` rule to `unknown`, and the verdict stayed
INCOMPLETE on a rule KiCad enforced and found met. KiCad also lists up to 499
`clearance` rows, so 483 of them were read as a report cut at 199.

Now a cut type counts against a rule only when the rule's constraint reports
under it (`check_complete._KICAD_ROW_TYPES`), each type is held to its own
limit, and a constraint the table does not list keeps the old reading.

Part 1 injects the grader's answer, or KiCad's report, and needs no KiCad.
Part 2 asks kicad-cli: what type it reports a rule of each listed kind under,
where it stops listing a type, and the board of the issue. Without kicad-cli
it is skipped and the test exits 77 (after part 1 has passed).

    python3 tests/test_1865_capped_type_scope.py
"""
import collections
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
import run_utils  # noqa: E402
import check_complete  # noqa: E402
from synth import RoutingBoard  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIXTURE = os.path.join(REPO, 'tests', 'fixtures', '1643', 'edge_gap.kicad_pcb')
ORANGECRAB = os.path.join(REPO, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
#: a rule check_drc cannot model (a rule area) that binds nothing
FAR = ('(version 1)\n(rule "far_corner"\n  (constraint %s (min 0.1mm))\n'
       "  (condition \"A.insideArea('far_zone')\"))\n")
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def judged(kinds, capped, name='r'):
    grade = {'kicad_version': '10.9.9', 'errors_by_rule': {}, 'capped_types': capped}
    table = None if kinds is None else {name: kinds}
    try:
        return check_complete.judge_rules([name], grade, [name], table)[name]
    except TypeError as exc:        # no `kinds` argument: a failed row, not a crash
        return {'verdict': f'TypeError: {exc}', 'why': ''}


def part1_judge():
    print('1. judge_rules: which cut type holds which rule (no KiCad needed)')
    cut3 = ['clearance', 'track_dangling', 'track_width']
    r = judged(['hole_clearance'], cut3)
    check('a hole_clearance rule is met beside cut clearance, track_dangling and '
          'track_width rows', r == {'verdict': 'met', 'kicad_errors': 0,
                                    'why': '0 KiCad violations'}, str(r))
    r = judged(['hole_clearance'], ['hole_clearance', 'track_width'])
    check('... and unknown when its own type is cut, naming that type alone',
          r['verdict'] == 'unknown' and r['why'] ==
          "KiCad's report is cut at 199 rows of hole_clearance, so its 0 rows "
          "naming it may be a dropped row", str(r))
    r = judged(['clearance'], cut3)
    check('a clearance rule beside the same three: unknown, at clearance\'s own '
          '499 rows', r['verdict'] == 'unknown'
          and 'cut at 499 rows of clearance, so' in r['why']
          and 'track_width' not in r['why'], str(r))
    r = judged(['clearance', 'track_width'], ['track_width'])
    check('a rule with two constraints is held by a cut type of either',
          r['verdict'] == 'unknown' and 'cut at 199 rows of track_width, so' in r['why'],
          str(r))
    r = judged(['hole_size'], ['microvia_drill_out_of_range'])
    check('a kind that reports under two types is held by each',
          r['verdict'] == 'unknown', str(r))
    r = judged(['diff_pair_gap'], ['track_width'])
    check('a constraint the table does not list is held by every cut type, '
          'saying so', r['verdict'] == 'unknown'
          and 'which type a diff_pair_gap rule reports under is not known here'
          in r['why'] and '199 rows of track_width' in r['why'], str(r))
    r = judged(['hole_clearance', 'diff_pair_gap'], ['track_width'])
    check('... also beside a listed one', r['verdict'] == 'unknown'
          and 'a diff_pair_gap rule' in r['why'], str(r))
    for label, kinds in (('nobody read', None), ('are empty', [])):
        r = judged(kinds, ['track_width'])
        check(f'a rule whose constraint kinds {label} is held by every cut type',
              r['verdict'] == 'unknown'
              and 'its constraint kinds were not read' in r['why'], str(r))
    r = judged(['diff_pair_gap'], [])
    check('nothing cut: met, whatever the kind', r['verdict'] == 'met', str(r))


def part1_limits(td):
    print('1b. kicad_rule_grade: each type against its own limit (no KiCad needed)')
    d = os.path.join(td, 'limits')
    os.makedirs(d)
    board = os.path.join(d, 'b.kicad_pcb')
    shutil.copy(FIXTURE, board)
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(FAR % 'hole_clearance')
    run_utils.evidence(board)
    probe = {'type': 'assertion_failure', 'severity': 'warning',
             'description': "Assertion failure (rule 'check_complete load probe')"}
    real = check_complete._kicad_drc_report

    def rows(**counts):
        return [probe] + [{'type': t, 'severity': 'error', 'description': t}
                          for t, n in counts.items() for _ in range(n)]
    try:
        for label, counts, want in (
                ('483 clearance rows are a whole report', {'clearance': 483}, []),
                ('498 are', {'clearance': 498}, []),
                ('499 clearance rows are a cut one', {'clearance': 499}, ['clearance']),
                ('198 track_width rows are a whole report', {'track_width': 198}, []),
                ('199 are a cut one', {'track_width': 199}, ['track_width']),
                ('the issue\'s report: 483 clearance, 199 track_width, 199 '
                 'track_dangling', {'clearance': 483, 'track_width': 199,
                                    'track_dangling': 199},
                 ['track_dangling', 'track_width'])):
            check_complete._kicad_drc_report = (
                lambda *_a, _c=counts, **_k: {'kicad_version': '10.9.9',
                                              'violations': rows(**_c)})
            g = check_complete.kicad_rule_grade(board)
            check(f'{label}: capped_types {want}', g.get('capped_types') == want, str(g)[:200])
    finally:
        check_complete._kicad_drc_report = real
    return board


def graded(board, injected, *extra):
    """check_complete in-process with `kicad_rule_grade` answering `injected`
    (None: the real one). -> (exit, document)."""
    real = check_complete.kicad_rule_grade
    if injected is not None:
        check_complete.kicad_rule_grade = lambda b, timeout=3600: injected
    js = board + '.cc.json'
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            code = check_complete.main([board, '--skip-slow', '--json', js, *extra])
    finally:
        check_complete.kicad_rule_grade = real
    with open(run_utils.evidence(js), encoding='utf-8') as fh:
        return code, json.load(fh)


def part1_verdict(board):
    print('1c. the verdict, from an injected KiCad grade (no KiCad needed)')
    cut = {'kicad_version': '10.9.9', 'errors_by_rule': {},
           'capped_types': ['clearance', 'track_dangling', 'track_width']}
    dru = os.path.splitext(board)[0] + '.kicad_dru'
    code, doc = graded(board, cut, '--kicad-rules')
    kr = doc.get('kicad_rules') or {}
    check('a hole_clearance rule KiCad has 0 rows of, three other types cut: DONE',
          code == 0 and doc['verdict'] == 'DONE'
          and "judged by KiCad 10.9.9: 'far_corner' (hole_clearance): 0 KiCad violations"
          in doc['reason'], doc['reason'][:400])
    check('the document still lists every cut type',
          kr.get('capped_types') == cut['capped_types']
          and kr['rules']['far_corner']['verdict'] == 'met', json.dumps(kr)[:300])
    with open(dru, 'w', encoding='utf-8') as fh:
        fh.write(FAR % 'clearance')
    code, doc = graded(board, cut, '--kicad-rules')
    check('the same rule as a clearance rule: INCOMPLETE, naming clearance alone',
          code == 4 and "nothing grades: 'far_corner' (clearance): KiCad's report is "
          "cut at 499 rows of clearance, so its 0 rows naming it may be a dropped row"
          in doc['reason'], doc['reason'][:400])
    with open(dru, 'w', encoding='utf-8') as fh:
        fh.write(FAR % 'hole_clearance')


# ---- part 2: KiCad itself ---------------------------------------------------
#: One rule per listed kind that fires on `probe_board`; a kind with no row
#: here (connection_width needs a filled zone) is not in the table.
PROBES = (
    ('clearance', '(constraint clearance (min 0.6mm))'),
    ('physical_clearance', '(constraint physical_clearance (min 0.7mm))'),
    ('hole_clearance', '(constraint hole_clearance (min 1mm))'),
    ('physical_hole_clearance', '(constraint physical_hole_clearance (min 1.2mm))'),
    ('edge_clearance', '(constraint edge_clearance (min 2mm))'),
    ('hole_to_hole', '(constraint hole_to_hole (min 2mm))'),
    ('hole_size', '(constraint hole_size (min 2mm))'),
    ('track_width', '(constraint track_width (min 1mm))'),
    ('track_segment_length', '(constraint track_segment_length (min 100mm))'),
    ('via_diameter', '(constraint via_diameter (min 2mm))'),
    ('annular_width', '(constraint annular_width (min 1mm))'),
    ('courtyard_clearance', '(constraint courtyard_clearance (min 3mm))'),
    ('silk_clearance', '(constraint silk_clearance (min 5mm))'),
    ('text_height', '(constraint text_height (min 2mm))'),
    ('text_thickness', '(constraint text_thickness (min 1mm))'),
    ('length', '(constraint length (max 1mm)) (condition "A.NetName == \'N1\'")'),
    ('skew', '(constraint skew (max 0.01mm)) '
             '(condition "A.NetName == \'N1\' || A.NetName == \'N2\'")'),
    ('via_count', '(constraint via_count (max 0)) (condition "A.NetName == \'N1\'")'),
    ('disallow', '(constraint disallow via) (condition "A.NetName == \'N2\'")'),
    ('assertion', '(constraint assertion "A.NetName != \'N3\'")'),
)


def probe_board():
    """Two nets with a via each, a drilled pad beside a track, a track end
    near the outline and two courtyards 1 mm apart."""
    rb = RoutingBoard(w=60, h=40)
    rb.pad('A', 10, 10, 'N1')
    rb.pad('B', 10, 11.2, 'N2')
    rb.pad('H', 20, 11.0, 'N3', sx=1.4, sy=1.4, tht=True)
    rb.seg(10, 10, 30, 10, 'N1')
    rb.via(30, 10, 'N1')
    rb.seg(30, 10, 30, 6, 'N1', layer='B.Cu')
    rb.pad('A2', 30, 6, 'N1', sx=1.6, sy=1.6, tht=True)
    rb.via(30.9, 10, 'N2')
    rb.seg(30.9, 10, 30.9, 14, 'N2')
    rb.seg(10, 11.2, 0.5, 11.2, 'N2')
    rb.pad('B2', 30.9, 14, 'N2')
    for ref, x in (('K1', 45.0), ('K2', 49.0)):
        rb.items.append(
            '  (footprint "synth:C" (layer "F.Cu") (uuid "%s") (at %s 30)\n'
            '    (property "Reference" "%s" (at 0 -3 0) (layer "F.SilkS") (uuid "%s")\n'
            '      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
            '    (fp_rect (start -1.5 -1.5) (end 1.5 1.5) (stroke (width 0.05) '
            '(type solid)) (fill no) (layer "F.CrtYd") (uuid "%s"))\n'
            '    (pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu" "F.Mask") '
            '(net %d "N4") (uuid "%s"))\n  )'
            % (rb._uid(), x, ref, rb._uid(), rb._uid(), rb.net('N4'), rb._uid()))
    return rb.text().replace('(37 "F.SilkS" user)',
                             '(37 "F.SilkS" user) (49 "F.CrtYd" user)')


def kicad_rows(cli, board):
    out = board + '.drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--units', 'mm',
                    '--severity-all', '--output', out, board], capture_output=True)
    with open(run_utils.evidence(out), encoding='utf-8') as fh:
        return json.load(fh).get('violations') or []


def part2_table(cli, td):
    print('2. kicad-cli: the type it reports a rule of each listed kind under')
    listed = check_complete._KICAD_ROW_TYPES
    check('every listed kind has a probe', sorted(listed) == sorted(k for k, _ in PROBES),
          str(sorted(set(listed) ^ {k for k, _ in PROBES})))
    text = probe_board()
    for kind, body in PROBES:
        d = os.path.join(td, 'probe_' + kind)
        os.makedirs(d)
        board = os.path.join(d, 'b.kicad_pcb')
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(text)
        with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(f'(version 1)\n(rule "probe" {body})\n')
        got = collections.Counter(v['type'] for v in kicad_rows(cli, board)
                                  if "(rule 'probe'" in (v.get('description') or ''))
        check(f'{kind}: rows naming the rule, all of a listed type '
              f'({", ".join(listed.get(kind, ()))})',
              bool(got) and set(got) <= set(listed.get(kind, ())), str(dict(got)))


def part2_limits(cli, td):
    print('2b. kicad-cli: where it stops listing a type')
    # 1600 tracks 0.2 mm apart in four rows, two nets alternating. KiCad
    # stops at a track's first violation, so 1599 gaps give 800 rows or more.
    rb = RoutingBoard(w=210, h=50)
    for i in range(1600):
        x, y = 5 + 0.5 * (i % 400), 5 + 10 * (i // 400)
        rb.seg(x, y, x, y + 5, 'N1' if i % 2 else 'N2')
    d = os.path.join(td, 'limits2')
    os.makedirs(d)
    board = os.path.join(d, 'b.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n(rule "gap" (constraint clearance (min 0.4mm)))\n'
                 '(rule "wide" (constraint track_width (min 1mm)))\n')
    got = collections.Counter(v['type'] for v in kicad_rows(cli, board))
    # KiCad 10.0.7 stops exactly at a limit; an earlier one checks it without
    # a lock and can list a few rows more, so the rows are held to a band.
    check('1600 tracks under a width rule: the track_width rows stop at 199',
          check_complete._type_cap('track_width') == 199
          and 199 <= got.get('track_width', 0) < 260, str(dict(got)))
    check('their gaps under a clearance rule: the clearance rows stop at 499, '
          'not at 199 and not at the 800 or more there are',
          check_complete._type_cap('clearance') == 499
          and 499 <= got.get('clearance', 0) < 560, str(dict(got)))


def part2_board(td):
    print('2c. kicad-cli: the board of the issue')
    d = os.path.join(td, 'oc')
    os.makedirs(d)
    board = os.path.join(d, 'oc.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()):
        from copy_board import copy_board
        copy_board(run_utils.evidence(ORANGECRAB), board)
    with open(os.path.join(d, 'oc.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(FAR % 'hole_clearance')
    _code, doc = graded(board, None, '--kicad-rules')
    kr = doc.get('kicad_rules') or {}
    cut = kr.get('capped_types') or []
    check('some other type is cut on it (track_width)', 'track_width' in cut, str(cut))
    check('its 483 clearance rows are not read as cut', 'clearance' not in cut, str(cut))
    check('the hole_clearance rule KiCad has 0 rows of is met',
          (kr.get('rules') or {}).get('far_corner') ==
          {'verdict': 'met', 'kicad_errors': 0, 'why': '0 KiCad violations'},
          json.dumps(kr.get('rules'))[:300])
    check('... and no longer among the rules nothing grades',
          "nothing grades: 'far_corner'" not in doc['reason']
          and "'far_corner' (hole_clearance): 0 KiCad violations" in doc['reason'],
          doc['reason'][:500])


def main():
    td = tempfile.mkdtemp(prefix='krt1865_')
    skipped = False
    try:
        part1_judge()
        board = part1_limits(td)
        part1_verdict(board)
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            part2_table(cli, td)
            part2_limits(cli, td)
            part2_board(td)
        else:
            print('2. SKIPPED: no kicad-cli (set KICAD_CLI)')
            skipped = True
    except AssertionError as exc:
        check('a run_utils assertion held', False, str(exc)[:1500])
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    if skipped:
        print('PART 1 PASS')
        print('SKIP: part 2 needs kicad-cli (none found; set KICAD_CLI) -- the '
              'type KiCad reports each rule kind under, and where it stops '
              'listing a type, were not checked')
        return 77
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
