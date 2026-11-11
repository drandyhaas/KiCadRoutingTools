"""#1810: `check_complete --kicad-rules` lets KiCad's own DRC grade a
`.kicad_dru` rule check_drc cannot model -- and fails closed.

Without the flag such a rule keeps the verdict INCOMPLETE (#1490), whether
KiCad finds it met or broken. With it, one kicad-cli DRC of the board as
shipped counts the error rows naming each such rule: 0 reads it met
(disclosed, `judged by KiCad <version>`), N blocks with the count. Anything
that leaves KiCad's rows unattributable -- no kicad-cli, a rules file KiCad
refuses whole (#1690), a rule name declared twice, a type cut at 199 rows --
keeps today's INCOMPLETE and says why.

Part 1 injects the grader's answer and needs no KiCad. Part 2 runs kicad-cli
on two boards, one meeting the rule and one breaking it; without kicad-cli it
is skipped and the test exits 77 (after part 1 has passed).

    python3 tests/test_1810_check_complete_kicad_rules.py
"""
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
        "  (condition \"A.insideCourtyard('J1')\"))\n")
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
                            "  (condition \"A.insideCourtyard('J2')\"))\n")
        code, doc = graded(b['met'], ok, '--kicad-rules')
        check('fails closed on a rule name declared twice',
              code == 4 and "names 2 rule(s) 'court gap'" in doc['reason'],
              doc['reason'][:300])
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write(RULE.replace('(version 1)\n', ''))
        # the real grader: it refuses before it would need kicad-cli
        code, doc = graded(b['met'], None, '--kicad-rules')
        check('fails closed on a rules file KiCad refuses whole (#1690)',
              code == 4 and 'KiCad enforces no rule of the .kicad_dru' in doc['reason'],
              doc['reason'][:300])
    finally:
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
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            part2(b)
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
              'KiCad\'s own grade of the two boards was not checked')
        return 77
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
