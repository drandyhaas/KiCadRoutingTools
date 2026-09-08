#!/usr/bin/env python3
"""#895: the boundary criteria, and the numbers the worked example quotes.

The mandate this ships beside was followed to the letter on run 25 and passed a
layout a human rejects at a glance. The criteria are the fix; this file is what
stops the WORKED EXAMPLE becoming the thing it warns about.

Two of the numbers in that reference started life as journal figures -- a seam
of 0.183mm and a pair of 9.3mm -- and re-measuring them with the shipped
instruments gives -0.133mm and 8.10mm. Neither journal figure was wrong when it
was written; neither could be reproduced from what was written down. So every
figure the reference quotes is re-derived here from a TRACKED fixture and
compared against the file's own text: a number that stops being true fails the
suite instead of ageing quietly.

The reference names no board and no work directory -- `test_run8_skills_generic`
bans both in every skill file, and rightly: a skill that names a corpus board
teaches the next reader to reach for it. That is exactly why the numbers need a
gate HERE, where the fixture may be named.

    python3 tests/test_895_boundary_criteria.py
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, ROOT)

RUN_ALL_TIMEOUT = 600

SKILL = os.path.join(ROOT, '.claude', 'skills',
                     'plan-pcb-placement-and-routing', 'SKILL.md')
REFERENCE = os.path.join(ROOT, '.claude', 'skills',
                         'plan-pcb-placement-and-routing', 'references',
                         'boundary-criteria.md')
#: The board the example is drawn from. Named HERE and never in the reference.
FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                       'esp_prog_lap5.kicad_pcb')
#: The declared clauses criterion 3 is worked from.
BRIEF = os.path.join(ROOT, 'tests', 'fixtures', '902',
                     'esp_prog_proximity.design-brief.json')

FAILURES = []


def check(name, cond, detail=''):
    if cond:
        print(f"  PASS: {name}")
    else:
        FAILURES.append(f"{name} -- {detail}")
        print(f"  FAIL: {name} -- {detail}")


def _text(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def test_the_seven_criteria_are_named_and_the_ordering_is_stated():
    """A criterion that is not written down is one nobody has to answer."""
    s = _text(SKILL)
    for want in ('Pair and bus length', 'Pin-order agreement',
                 'Cluster distance', 'Facing', 'Seams',
                 'Density and balance', 'The human question'):
        check(f'the boundary section names {want!r}', want in s)
    check('...and says an unanswered criterion blocks the close',
          'unanswered criterion blocks the close' in s)
    # The sharpened ordering rule: BEFORE ANY VERDICT, not merely before any
    # checklist key. The letter of the old rule was satisfied by a run that
    # read three checker verdicts 16 seconds before its sheet existed.
    check('the ordering rule is before any VERDICT, not just any key',
          'before any VERDICT' in s and '16 seconds' in s)
    check('the reference is linked from the section',
          'references/boundary-criteria.md' in s)


def test_the_reference_names_no_board_and_no_work_directory():
    """The prose gate bans both; this says WHY rather than leaving the next
    author to discover it from a failing test."""
    import re
    s = _text(REFERENCE)
    check('no corpus board name', 'esp_prog' not in s)
    check('no work directory', re.search(r'wk/run\d+', s) is None)
    check('the subject is described by part class instead',
          'USB debug adapter' in s)


def test_criterion_1_and_2_are_what_the_instrument_reports():
    """`span_mm` and `verdict`, re-derived from the tracked fixture."""
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_tools',
                                                     'board_context.py'),
                        FIXTURE, '--json'],
                       capture_output=True, text=True)
    check('board_context exits 0', r.returncode == 0, r.stderr[-300:])
    doc = json.loads(r.stdout)
    rows = {(x['a'], x['b'], x['scope']): x for x in doc['pin_order']['rows']}
    pair = rows.get(('U1', 'USB1', 'pair /D_P//D_N'))
    iface = rows.get(('U1', 'USB1', 'interface'))
    check('the pair row exists', pair is not None, sorted(rows))
    ref = _text(REFERENCE)
    if pair:
        check('the reference quotes the pair span',
              f"{pair['span_mm']:.2f}" in ref,
              f"measured {pair['span_mm']}")
        check('...and the pair reads CROSSED, as the reference says',
              pair['verdict'] == 'CROSSED' and 'CROSSED' in ref,
              pair['verdict'])
        check('...with the one inversion the reference cites',
              pair['inversions'] == 1 and 'one inversion' in ref,
              pair['inversions'])
    if iface:
        check('the reference quotes the interface span',
              f"{iface['span_mm']:.2f}" in ref, f"measured {iface['span_mm']}")


def test_criterion_5_is_what_the_instrument_reports():
    """The tightest body seam, signed, with its sources."""
    from kicad_parser import parse_kicad_pcb
    from placement import legality
    pcb = parse_kicad_pcb(FIXTURE)
    found = legality.grade_body_overlap(pcb, 0.15, (), FIXTURE)
    seam = found.get('body_seam') or {}
    ref = _text(REFERENCE)
    check('a seam was measured', bool(seam), found.keys())
    if seam:
        check('the reference quotes the seam',
              f"{seam['mm']}" in ref, f"measured {seam['mm']}")
        check('...and names both source rungs',
              seam['source_a'] in ref and seam['source_b'] in ref,
              f"{seam['source_a']}/{seam['source_b']}")
        check('the seam is an OVERLAP, which the reference says outright',
              seam['mm'] < 0 and 'Negative is an overlap' in ref, seam['mm'])


def test_criterion_6_is_what_the_instrument_reports():
    """And that the reference calls the centroid what it is."""
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_tools',
                                                     'check_pockets.py'),
                        FIXTURE, '--bin', '5'],
                       capture_output=True, text=True)
    line = [x for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY:')]
    check('check_pockets emitted a summary', bool(line), r.stderr[-200:])
    if not line:
        return
    s = json.loads(line[0].split('JSON_SUMMARY: ', 1)[1])
    ref = _text(REFERENCE)
    check('the reference quotes the emptiest region',
          f"{s['cold_top_area_mm2']}" in ref, s['cold_top_area_mm2'])
    check('the reference quotes the centroid offset',
          f"{round(s['centroid_offset_frac'] * 100, 1)}" in ref,
          s['centroid_offset_frac'])
    # The weight is COURTYARD AREA and the tool says so; a reference that
    # called it "the centroid" would be quoting a different number.
    check('the reference says the weight is courtyard area',
          'COURTYARD AREA' in ref)


def test_criterion_3_is_what_the_grader_reports():
    """Every distance criterion 3 quotes, re-derived through the real grader.

    This gate exists because the reference first shipped the crystal leg as
    3.14mm -- the right number for a DIFFERENT fixture of the same board, one
    lap earlier. The file claimed every figure was pinned by a test; criterion
    3's were not, so the one figure measured on the wrong board was the one
    nothing caught.

    The PASSING distances matter as much as the failing one, so the brief is
    re-read with every `max_mm` tightened to 0.001: that makes each declared
    row report its measured gap instead of only the row that exceeds its own
    limit. It is the same rule measuring either way -- a limit decides what is
    REPORTED, never what is measured.
    """
    import tempfile

    from kicad_parser import parse_kicad_pcb
    from placement import groups

    with open(BRIEF, encoding='utf-8') as fh:
        brief = json.load(fh)
    for row in brief['proximity']:
        row['max_mm'] = 0.001
    ref = _text(REFERENCE)

    with tempfile.TemporaryDirectory() as tmp:
        tight = os.path.join(tmp, 'tight.json')
        intent = os.path.join(tmp, 'intent.json')
        graded = os.path.join(tmp, 'graded.json')
        with open(tight, 'w', encoding='utf-8') as fh:
            json.dump(brief, fh)
        cf = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')
        emit = subprocess.run([sys.executable, '-X', 'utf8', cf, FIXTURE,
                               '--brief', tight, '--emit-intent', intent],
                              capture_output=True, text=True)
        check('check_floorplan emitted an intent', os.path.isfile(intent),
              emit.stderr[-300:])
        if not os.path.isfile(intent):
            return
        subprocess.run([sys.executable, '-X', 'utf8', cf, FIXTURE,
                        '--brief', tight, '--intent', intent,
                        '--json', graded], capture_output=True, text=True)
        check('check_floorplan wrote a graded document',
              os.path.isfile(graded))
        if not os.path.isfile(graded):
            return
        with open(graded, encoding='utf-8') as fh:
            doc = json.load(fh)

    # Key on the PAIR, not on the number. Two of the measured gaps round to
    # the same two decimals (the second bulk cap at 0.292 and the transistor
    # pair at 0.295), so a bare `f'{gap:.2f}' in ref` sweep passes for a row
    # the reference never quotes, satisfied by a different row's digits.
    gaps = {}
    for v in doc['violations']:
        m = v.get('measured') or {}
        if v.get('rule') == 'proximity' and 'gap_mm' in m:
            gaps[(v.get('ref'), m.get('near'), m.get('pad'))] = m['gap_mm']
    check('the rule measured every declared row', len(gaps) >= 5, sorted(gaps))
    #: The distances criterion 3 states, and the subject pad each belongs to.
    quoted = (('Y1', 'U1', '1'), ('Y1', 'U1', '2'),
              ('C1', 'U2', '1'), ('C3', 'U2', '1'))
    for key in quoted:
        gap = gaps.get(key)
        check(f'{key[0]} pad {key[2]} -> {key[1]} was measured', gap is not None,
              sorted(gaps))
        if gap is None:
            continue
        check(f'the reference quotes it as {gap:.2f}mm', f'{gap:.2f}' in ref,
              f'measured {gap}')
    # The transistor pair is measured on the body basis and deliberately NOT
    # quoted -- criterion 3's text is about the regulator and the crystal. It
    # is asserted absent-by-pair rather than by digits, which C3 also carries.
    check('the body-basis pair was measured too',
          ('Q1', 'Q2', None) in gaps, sorted(gaps))

    # The two tether distances the same section cites as the WRONG partners.
    # They are the argument for the rule existing, so they are pinned too.
    tethers = groups.decap_tethers(parse_kicad_pcb(FIXTURE))
    wrong = sorted(mm for rows in (tethers or {}).values() for _, mm in rows)
    check('the reference quotes the wrong-partner distances it cites',
          all(f'{mm:.2f}' in ref for mm in wrong[-2:]),
          [f'{mm:.2f}' for mm in wrong])


def test_criterion_1_quotes_the_bodies_it_compares_against():
    """The denominator, not only the span.

    Criterion 1 is a RATIO, and the reference used to state its two bodies as
    round numbers nobody could source -- one of them out by 2mm. A span with an
    invented denominator is a criterion measuring nothing.
    """
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_tools',
                                                     'board_context.py'),
                        FIXTURE, '--json'],
                       capture_output=True, text=True)
    check('board_context exits 0', r.returncode == 0, r.stderr[-300:])
    parts = {p['ref']: p for p in json.loads(r.stdout).get('parts', [])}
    ref = _text(REFERENCE)
    for who in ('U1', 'USB1'):
        body = (parts.get(who) or {}).get('body_mm')
        check(f'{who} reports a body', bool(body), sorted(parts))
        if not body:
            continue
        for mm in body:
            check(f'the reference quotes the {mm:.2f}mm body extent',
                  f'{mm:.2f}' in ref, f'measured {body}')
        src = parts[who].get('body_source')
        check(f'...and names the rung it came from ({src})', str(src) in ref)


def test_the_reference_says_which_journal_numbers_did_not_reproduce():
    """The example's own warning about prose.

    A worked example that quietly used the reproducible numbers, and said
    nothing about the two that did not reproduce, would be the more flattering
    document and the less useful one.
    """
    ref = _text(REFERENCE)
    check('it names the journal seam that did not reproduce', '0.183' in ref)
    check('it names the journal pair length that did not reproduce',
          '9.3' in ref)
    check('...and says why neither was wrong when written',
          'measured differently' in ref)


TESTS = [test_the_seven_criteria_are_named_and_the_ordering_is_stated,
         test_the_reference_names_no_board_and_no_work_directory,
         test_criterion_1_and_2_are_what_the_instrument_reports,
         test_criterion_1_quotes_the_bodies_it_compares_against,
         test_criterion_3_is_what_the_grader_reports,
         test_criterion_5_is_what_the_instrument_reports,
         test_criterion_6_is_what_the_instrument_reports,
         test_the_reference_says_which_journal_numbers_did_not_reproduce]


def main():
    for t in TESTS:
        print(f'--- {t.__name__}')
        try:
            t()
        except Exception as exc:                             # noqa: BLE001
            check(f'{t.__name__} ran to completion', False,
                  f'{type(exc).__name__}: {exc}')
    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #895 boundary criteria, "
          f"{len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f'  - {f}')
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
