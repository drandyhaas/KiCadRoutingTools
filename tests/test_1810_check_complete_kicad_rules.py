"""#1810: `check_complete --kicad-rules` lets KiCad's own DRC grade a
`.kicad_dru` rule check_drc cannot model -- and fails closed.

Without the flag such a rule keeps the verdict INCOMPLETE (#1490), whether
KiCad finds it met or broken. With it, one kicad-cli DRC of the board as
shipped counts the error rows naming each such rule: 0 reads it met
(disclosed, `judged by KiCad <version>`), N blocks with the count. Anything
that leaves KiCad's rows unattributable -- no kicad-cli, a rules file KiCad
does not load (#1690), a rule name declared twice, a type cut at 199 rows --
keeps today's INCOMPLETE and says why.

Whether KiCad loaded the rules file is asked of KiCad, not only of the model
of what it refuses: the staged copy's file gets one probe rule, and a report
with no row naming it is a file KiCad dropped. Twelve forms KiCad 10 drops a
whole file for, none of which the model names, read `judged by KiCad ... 0
KiCad violations` and DONE without it, on the board that breaks the rule.

Part 1 injects the grader's answer, or KiCad's report, and needs no KiCad.
Part 2 runs kicad-cli on two boards, one meeting the rule and one breaking
it, and on the forms KiCad drops the file for; without kicad-cli it is
skipped and the test exits 77 (after part 1 has passed).

    python3 tests/test_1810_check_complete_kicad_rules.py
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
import run_utils  # noqa: E402
import check_complete  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIXTURE = os.path.join(REPO, 'tests', 'fixtures', '1643', 'edge_gap.kicad_pcb')
RULE = ('(version 1)\n(rule "court gap" (constraint clearance (min 0.3mm))\n'
        "  (condition \"A.insideCourtyard('J1') && A.getField('x') == ''\"))\n")
# check_drc reads a courtyard term by position (#1832) but not getField(), so
# the rule stays one it cannot grade; kicad-cli 10.0.6 reads a pad's
# getField('x') as '' and grades the rule as the courtyard rule alone
# (met: 0 rows, broken: 1)
_OK = RULE.split('\n', 1)[1]


def _before(rule):
    return '(version 1)\n' + rule + '\n' + _OK


#: Rules files KiCad 10 drops whole, each around the valid `court gap` rule
#: (measured on kicad-cli 10.0.7: 0 rows naming `court gap` on the `broken`
#: board, where the valid file gives 1).
DROPPED = (
    ('(version 1) is not the first form', _OK + '(version 1)\n'),
    ('(version) has no value', '(version)\n' + _OK),
    ('one rule has two (layer ...) clauses', _before(
        '(rule "w" (layer "F.Cu") (layer "B.Cu") (constraint track_width (min 0.1mm)))')),
    ('one rule has a constraint kind twice', _before(
        '(rule "w" (constraint track_width (min 0.1mm)) (constraint track_width (max 2mm)))')),
    ('another rule names a layer that is none', _before(
        '(rule "w" (layer "Top") (constraint track_width (min 0.1mm)))')),
    ('the judged rule names a layer that is none',
     '(version 1)\n' + _OK.replace('(constraint', '(layer "Nope") (constraint')),
    ('a length has no unit', _before(
        '(rule "w" (constraint length (max 5)) (condition "A.NetName == \'SIG\'"))')),
    ('a via_count has a unit', _before('(rule "w" (constraint via_count (max 2mm)))')),
    ('a zone_connection names no style', _before(
        '(rule "w" (constraint zone_connection bogus))')),
    ('a token follows (min ...)', _before(
        '(rule "w" (constraint track_width (min 0.1mm) extra))')),
    ('(min) has no value', _before('(rule "w" (constraint track_width (min)))')),
    ('(min ...) has two values', _before(
        '(rule "w" (constraint track_width (min 1mm 2mm)))')),
    ('a constraint value is no (min/opt/max ...)', _before(
        '(rule "w" (constraint track_width (foo 1mm)))')),
)
NOT_LOADED = 'KiCad did not load the .kicad_dru: a probe rule'
PROBE = 'check_complete load probe'      # the rule name KiCad's row carries
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def boards(td):
    """`met`: J1 gets a 4 x 4 mm courtyard, nothing near its pad. `broken`:
    the same plus a SIG pad 0.25 mm from J1.1 -- legal at the board's 0.2 mm
    clearance, not at the rule's 0.3 mm."""
    t = open(FIXTURE, encoding='utf-8', newline='').read()
    pad, edge = '\t\t(pad "1" thru_hole circle', '\t(gr_line\n'
    assert t.count(pad) == 1 and t.count(edge) == 4, 'the fixture changed'
    crt = ('\t\t(fp_rect\n\t\t\t(start -2 -2)\n\t\t\t(end 2 2)\n\t\t\t(stroke\n'
           '\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n\t\t\t(fill no)\n'
           '\t\t\t(layer "F.CrtYd")\n\t\t\t(uuid "0c3f6a1e-5b7d-4e2a-9f10-1d2c3b4a5e6f")\n\t\t)\n')
    tp1 = ('\t(footprint ""\n\t\t(layer "F.Cu")\n\t\t(uuid "5a1b2c3d-4e5f-4a6b-8c7d-9e0f1a2b3c4d")\n'
           '\t\t(at 6 16.55)\n\t\t(property "Reference" "TP1"\n\t\t\t(at 0 2 0)\n'
           '\t\t\t(layer "F.SilkS")\n\t\t\t(uuid "6b2c3d4e-5f6a-4b7c-9d8e-0f1a2b3c4d5e")\n'
           '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
           '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n\t\t\t(layers "F.Cu" "F.Mask")\n'
           '\t\t\t(net "SIG")\n\t\t\t(uuid "7c3d4e5f-6a7b-4c8d-8e9f-1a2b3c4d5e6f")\n\t\t)\n\t)\n')
    met = t.replace(pad, crt + pad)
    out = {}
    for name, text in (('met', met), ('broken', met.replace(edge, tp1 + edge, 1))):
        d = os.path.join(td, name)
        os.makedirs(d)
        b = os.path.join(d, 'b.kicad_pcb')
        with open(b, 'w', encoding='utf-8', newline='') as fh:
            fh.write(text)
        with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(RULE)
        out[name] = run_utils.evidence(b) or b
    return out


def graded(board, injected, *extra):
    """check_complete in-process, its kicad_rule_grade answering `injected`
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


def part1(b):
    print('1. the verdict from an injected KiCad grade (no KiCad needed)')
    code, doc = graded(b['met'], None)
    check('without the flag: INCOMPLETE, as #1490 has it, pointing at the flag',
          code == 4 and "nothing grades: 'court gap' (clearance)" in doc['reason']
          and '--kicad-rules' in doc['reason'] and 'kicad_rules' not in doc,
          doc['reason'][:300])

    ok = {'kicad_version': '10.9.9', 'errors_by_rule': {}, 'capped_types': []}
    code, doc = graded(b['met'], ok, '--kicad-rules')
    kr = doc.get('kicad_rules') or {}
    check('KiCad finds 0 rows naming it: DONE, disclosed as judged by KiCad',
          code == 0 and doc['verdict'] == 'DONE'
          and "judged by KiCad 10.9.9: 'court gap' (clearance): 0 KiCad violations"
          in doc['reason'], doc['reason'][:400])
    check('the document records the count, the version and who measured',
          kr.get('measured_by') == 'kicad-cli' and kr.get('kicad_version') == '10.9.9'
          and kr['rules']['court gap'] == {'verdict': 'met', 'kicad_errors': 0,
                                           'why': '0 KiCad violations'}, json.dumps(kr))
    check('a rule set KiCad judged is no longer UNEXAMINED',
          'drc.dru_rules' not in doc['ungraded'], str(doc['ungraded']))

    code, doc = graded(b['met'], dict(ok, errors_by_rule={'court gap': 1}), '--kicad-rules')
    check('KiCad finds 1 row naming it: INCOMPLETE with the count',
          code == 4 and "finds BROKEN: 'court gap' (clearance): 1 KiCad violation"
          in doc['reason'], doc['reason'][:300])

    for why, inj in (
            ('kicad-cli missing', {'error': 'kicad-cli not found (set KICAD_CLI)'}),
            ('a type cut at 199 rows', dict(ok, capped_types=['clearance']))):
        code, doc = graded(b['met'], inj, '--kicad-rules')
        check(f'fails closed on {why}: INCOMPLETE, saying why',
              code == 4 and "nothing grades: 'court gap'" in doc['reason']
              and ('kicad-cli not found' in doc['reason'] or 'cut at 199' in doc['reason']),
              doc['reason'][:300])

    dru = os.path.join(os.path.dirname(b['met']), 'b.kicad_dru')
    try:
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE + '(rule "court gap" (constraint clearance (min 0.1mm))\n'
                            "  (condition \"A.insideCourtyard('J2') && A.getField('x') == ''\"))\n")
        code, doc = graded(b['met'], ok, '--kicad-rules')
        check('fails closed on a rule name declared twice',
              code == 4 and "names 2 rule(s) 'court gap'" in doc['reason'],
              doc['reason'][:300])
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE.replace('(version 1)\n', ''))
        # the real grader: KiCad's answer (when kicad-cli is here) or the
        # refusal model's, which names the reason either way
        code, doc = graded(b['met'], None, '--kicad-rules')
        check('fails closed on a rules file KiCad refuses whole (#1690)',
              code == 4 and 'KiCad enforces no rule of the .kicad_dru' in doc['reason'],
              doc['reason'][:300])
    finally:
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE)


def _row(rule, severity='error', kind='clearance'):
    return {'type': kind, 'severity': severity,
            'description': f"Violation (rule '{rule}' clearance 0.3000 mm; "
                           f"actual 0.2500 mm)"}


def part1_probe(b):
    print('1b. the probe rule, from an injected KiCad report (no KiCad needed)')
    board = b['broken']
    dru = os.path.join(os.path.dirname(board), 'b.kicad_dru')
    real = check_complete._kicad_drc_report
    seen = {}

    def answering(rows):
        def fake(_board, timeout=3600, project_edit=None, rules_edit=None):
            with open(dru, encoding='utf-8') as fh:
                own = fh.read()
            seen['own'], seen['staged'] = own, rules_edit(own) if rules_edit else None
            return {'kicad_version': '10.9.9', 'violations': list(rows)}
        return fake

    probe = PROBE
    import kicad_parser
    real_parse = kicad_parser.parse_kicad_pcb

    def no_parse(*_a, **_k):
        raise AssertionError('a full parse')
    kicad_parser.parse_kicad_pcb = no_parse
    try:
        name, rule = check_complete._load_probe(board)
    finally:
        kicad_parser.parse_kicad_pcb = real_parse
    check('the probe reads the references off the text, with no full parse of the board',
          name == probe and "A.Reference != 'J1'" in rule, f'{name}: {rule}')
    try:
        with open(dru, 'rb') as fh:
            before = fh.read()
        check_complete._kicad_drc_report = answering(
            [{'type': 'clearance', 'severity': 'error',
              'description': "Clearance violation (netclass 'Default' clearance "
                             "0.2000 mm; actual 0.1000 mm)"}])
        g = check_complete.kicad_rule_grade(board)
        staged = seen.get('staged') or ''
        added = staged[len(seen['own']):] if staged.startswith(seen['own']) else ''
        check('the staged rules file is the board\'s own plus ONE rule, the probe, '
              'at warning severity',
              added.count('(rule ') == 1 and f'(rule "{probe}" (severity warning)' in added
              and '(constraint assertion "A.Reference != \'J1\'")' in added, repr(staged))
        with open(dru, 'rb') as fh:
            check('the board\'s own rules file is not written', fh.read() == before)
        check('a report with no row naming the probe: the file was not loaded',
              NOT_LOADED in g.get('error', '') and 'errors_by_rule' not in g, str(g))
        code, doc = graded(board, None, '--kicad-rules')
        check('... and the verdict is INCOMPLETE for that reason, not DONE',
              code == 4 and "nothing grades: 'court gap' (clearance): KiCad could "
              "not grade it: " + NOT_LOADED in doc['reason'], doc['reason'][:400])

        check_complete._kicad_drc_report = answering(
            [_row(probe, 'warning', 'assertion_failure'), _row('court gap')])
        g = check_complete.kicad_rule_grade(board)
        check('a report with the probe\'s row: graded, and the probe is no finding',
              g == {'kicad_version': '10.9.9', 'errors_by_rule': {'court gap': 1},
                    'capped_types': []}, str(g))

        check_complete._kicad_drc_report = answering(
            [_row('court gap', kind='assertion_failure')] * 199)
        g = check_complete.kicad_rule_grade(board)
        check('199 rows naming a rule of the file and none the probe: loaded, '
              'and the type reads cut',
              g.get('errors_by_rule') == {'court gap': 199}
              and g.get('capped_types') == ['assertion_failure'], str(g)[:300])

        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE + f'(rule "{probe}" (constraint track_width (min 0.1mm)))\n')
        check_complete._kicad_drc_report = answering([_row(probe)])
        g = check_complete.kicad_rule_grade(board)
        check('a rule of the file with the probe\'s name keeps its rows: the '
              'probe takes another name',
              f'(rule "{probe} 2" (severity warning)' in (seen.get('staged') or '')
              and g.get('errors_by_rule') == {probe: 1}, str(g))
        # the refusal model only says why: a file it refuses (no version line)
        # that KiCad's report shows loaded is graded by KiCad, and one KiCad
        # did not load carries the model's reason beside KiCad's answer
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE.replace('(version 1)\n', ''))
        check_complete._kicad_drc_report = answering(
            [_row(probe, 'warning', 'assertion_failure'), _row('court gap')])
        g = check_complete.kicad_rule_grade(board)
        check('a file the model refuses but KiCad loaded: KiCad grades it',
              g.get('errors_by_rule') == {'court gap': 1}, str(g)[:300])
        check_complete._kicad_drc_report = answering([])
        g = check_complete.kicad_rule_grade(board)
        check('a file the model refuses and KiCad did not load: KiCad\'s answer '
              'with the model\'s reason',
              NOT_LOADED in g.get('error', '') and 'no (version ...) line' in g['error'],
              str(g)[:300])
        check_complete._kicad_drc_report = (
            lambda *_a, **_k: {'error': 'kicad-cli not found (set KICAD_CLI)'})
        g = check_complete.kicad_rule_grade(board)
        check('... and KiCad cannot be asked: the model\'s reason stands in',
              g.get('error', '').startswith('KiCad enforces no rule of the .kicad_dru')
              and 'no (version ...) line' in g['error'], str(g)[:300])
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE)

        bare = os.path.join(os.path.dirname(board), 'bare.kicad_pcb')
        with open(bare, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20241229) (generator "pcbnew")\n'
                     '  (layers (0 "F.Cu" signal) (2 "B.Cu" signal)))\n')
        shutil.copy(dru, os.path.join(os.path.dirname(board), 'bare.kicad_dru'))
        seen.clear()
        g = check_complete.kicad_rule_grade(run_utils.evidence(bare))
        check('a board with no footprint to probe with fails closed, KiCad not asked',
              'no footprint with a plain reference' in g.get('error', '')
              and not seen, str(g))
    finally:
        check_complete._kicad_drc_report = real
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE)


def part2_dropped(b):
    print('3. KiCad itself, on rules files it drops whole')
    board = b['broken']
    here = os.path.dirname(board)
    dru = os.path.join(here, 'b.kicad_dru')
    cc = PY + [os.path.join(REPO, 'check_complete.py'), board, '--skip-slow',
               '--kicad-rules']
    import design_rules
    real_model = design_rules.read_board_kicad_refusals
    real_report = check_complete._kicad_drc_report
    try:
        for i, (why, text) in enumerate(DROPPED):
            with open(dru, 'w', encoding='utf-8') as fh:
                fh.write(text)
            g = check_complete.kicad_rule_grade(board)
            check(f'{why}: not loaded, so nothing is read as met',
                  NOT_LOADED in g.get('error', ''), str(g)[:300])
            if i in (2, 6):
                try:
                    run_utils.check(cc, refuse="nothing grades: 'court gap' "
                                               "(clearance): KiCad could not grade "
                                               "it: " + NOT_LOADED, code=4)
                    said = ''
                except AssertionError as exc:
                    said = str(exc)
                check(f'{why}: check_complete exits 4 for that reason', not said,
                      said[:700])

        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(_OK)
        design_rules.read_board_kicad_refusals = lambda _board: []
        g = check_complete.kicad_rule_grade(board)
        design_rules.read_board_kicad_refusals = real_model
        check('a refusal the model does name (no version line), the model '
              'silenced: KiCad still says it did not load the file',
              NOT_LOADED in g.get('error', ''), str(g)[:300])

        # The probe neither hides a row nor adds one: beside an assertion of
        # the file's own on the footprint the probe uses, the report with the
        # probe is the report without it plus the probe's one warning row.
        mine = ('(rule "mine" (constraint assertion "A.Reference != \'J1\'")\n'
                "  (condition \"A.Type == 'Footprint'\"))\n")
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n' + mine + _OK)
        got = {}

        def spy(*a, **k):
            got['report'] = real_report(*a, **k)
            return got['report']
        check_complete._kicad_drc_report = spy
        g = check_complete.kicad_rule_grade(board)
        check_complete._kicad_drc_report = real_report
        plain = real_report(board)

        def rows(rep):
            return collections.Counter((v.get('type'), v.get('severity'),
                                        v.get('description'))
                                       for v in rep.get('violations') or ())
        extra, lost = rows(got['report']) - rows(plain), rows(plain) - rows(got['report'])
        check('the report with the probe is the plain report plus one warning '
              'row naming the probe',
              not lost and list(extra.items()) == [(
                  ('assertion_failure', 'warning',
                   f"Assertion failure (rule '{PROBE}')"), 1)],
              f'added {dict(extra)} lost {dict(lost)}')
        check('both rules of the file keep their one error row each',
              g.get('errors_by_rule') == {'court gap': 1, 'mine': 1}, str(g))

        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE)
        pro = os.path.join(here, 'b.kicad_pro')
        with open(pro, 'w', encoding='utf-8') as fh:
            json.dump({'board': {'design_settings': {'rule_severities': {
                'assertion_failure': 'ignore'}}}}, fh)
        g = check_complete.kicad_rule_grade(board)
        os.remove(pro)
        check('a project that ignores assertion_failure does not silence the probe',
              g.get('errors_by_rule') == {'court gap': 1}, str(g)[:300])
    finally:
        design_rules.read_board_kicad_refusals = real_model
        check_complete._kicad_drc_report = real_report
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE)


def part2(b):
    print('2. KiCad itself grades the two boards')
    run_utils.check(PY + [os.path.join(REPO, 'check_complete.py'), b['met'],
                          '--skip-slow', '--kicad-rules'], accept=True)
    code, doc = graded(b['met'], None, '--kicad-rules')
    check('met: DONE, judged by KiCad with its version',
          code == 0 and "judged by KiCad " in doc['reason']
          and "'court gap' (clearance): 0 KiCad violations" in doc['reason']
          and (doc.get('kicad_rules') or {}).get('kicad_version'), doc['reason'][:300])
    run_utils.check(PY + [os.path.join(REPO, 'check_complete.py'), b['broken'],
                          '--skip-slow', '--kicad-rules'],
                    refuse="finds BROKEN: 'court gap' (clearance): 1 KiCad violation",
                    code=4)
    check('broken: INCOMPLETE with the rule and its count', True)


def main():
    td = tempfile.mkdtemp(prefix='krt1810_')
    skipped = False
    try:
        b = boards(td)
        part1(b)
        part1_probe(b)
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            part2(b)
            part2_dropped(b)
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
        print('SKIP: part 2 needs kicad-cli (none found; set KICAD_CLI) -- '
              'KiCad\'s own grade of the two boards, and of the rules files it '
              'drops, was not checked')
        return 77
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
