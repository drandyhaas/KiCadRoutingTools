#!/usr/bin/env python3
"""#1367: fill_for_delivery's KiCad 10 re-save must not turn accepted
vias-in-paste into violations.

pcbnew saves what it runs, so a pre-KiCad-10 routed board comes back as
`(version 20260206)` with `(capping no) (covering ...) (plugging ...)
(filling no)` on every via. check_drc had accepted the vias under solder as
"undeclarable in this file format"; on the re-saved copy they became
violations, and the Type VII stamp read the migration's explicit `no` as a
decision and stamped none of them.

  1. (no KiCad) strip_via_protection_in_content removes exactly the named
     direct-child tokens of the named vias, and nothing else.
  2. (no KiCad) migrated_via_protection_drops lists only tokens the input via
     did not carry and whose value changes nothing.
  3. (KiCad python) fill_for_delivery on a pre-KiCad-10 routed board: the
     format change is printed, no via carries a leftover `(capping no)`, every
     via in a paste opening is filled+capped, and check_drc against the
     copper-stripped baseline reports 0 via-in-paste violations -- as it did
     on the input.

    python3 tests/test_1367_fill_resave_via_protection.py
"""
import os
import re
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in (TESTS_DIR, ROOT, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools')):
    sys.path.insert(0, _d)

from run_utils import check, evidence, tool              # noqa: E402

SKIP_EXIT = 77
RUN_ALL_TIMEOUT = 1500
BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


VIA = """\t(via
\t\t(at {x} 77)
\t\t(size 0.6)
\t\t(drill 0.3)
\t\t(layers "F.Cu" "B.Cu")
\t\t(free yes)
\t\t(capping no)
\t\t(covering
\t\t\t(front no)
\t\t\t(back no)
\t\t)
\t\t(filling no)
\t\t(net "/N")
\t\t(uuid "{u}")
\t)
"""


def unit_checks():
    from kicad_writer import strip_via_protection_in_content
    from fab_notes import migrated_via_protection_drops

    print('1: strip_via_protection_in_content')
    text = '(kicad_pcb\n' + VIA.format(x=1, u='aaaa') + VIA.format(x=2, u='bbbb') + ')\n'
    out, n = strip_via_protection_in_content(
        text, {'aaaa': ['capping', 'covering', 'filling']})
    ok('one via edited', n == 1, str(n))
    first, second = out.split('(via', 2)[1:]
    ok('the named via loses all three tokens',
       not re.search(r'capping|covering|filling', first), first)
    ok('its other fields stay', '(free yes)' in first and '(uuid "aaaa")' in first)
    ok('the unnamed via is untouched',
       '(capping no)' in second and '(filling no)' in second and '(covering' in second, second)
    out2, n2 = strip_via_protection_in_content(text, {'bbbb': ['capping']})
    ok('only the named token goes',
       n2 == 1 and out2.count('(capping no)') == 1 and out2.count('(filling no)') == 2)

    print('2: migrated_via_protection_drops')
    setup_in = {'tenting': '(front yes) (back yes)'}
    setup_out = {'tenting': '(front yes) (back yes)', 'covering': '(front no) (back no)',
                 'plugging': '(front no) (back no)', 'capping': 'no', 'filling': 'no'}

    def pcb(vias, setup):
        return SimpleNamespace(vias=vias, board_info=SimpleNamespace(via_protection_setup=setup))

    def via(u, spec):
        return SimpleNamespace(uuid=u, tenting_attrs=spec)
    src = pcb([via('a', {}), via('b', {'capping': 'yes'}), via('c', {})], setup_in)
    dst = pcb([via('a', {'capping': 'no', 'filling': 'no'}),
               via('b', {'capping': 'yes'}),
               via('c', {'capping': 'yes'})], setup_out)
    drops = migrated_via_protection_drops(src, dst)
    ok('the migration default is dropped', drops.get('a') == ['capping', 'filling'], str(drops))
    ok("the input's own token is kept", 'b' not in drops, str(drops))
    ok('a value the input did not fabricate with is kept', 'c' not in drops, str(drops))


def end_to_end():
    from kicad_exact_fill import find_kicad_python
    if find_kicad_python() is None:
        print('  NOT RUN: 3. no KiCad python here -- this is not a pass')
        return False
    from copy_board import copy_board
    from kicad_parser import parse_kicad_pcb
    evidence(BOARD, 'the pre-KiCad-10 routed board')
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'in.kicad_pcb')
        base = os.path.join(tmp, 'base.kicad_pcb')
        out = os.path.join(tmp, 'f.kicad_pcb')
        copy_board(BOARD, src)
        check([sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py'),
               BOARD, base], accept=True)
        evidence(base, 'the copper-stripped baseline')
        r = check([sys.executable, '-X', 'utf8', tool('fill_for_delivery.py'),
                   src, '-o', out], accept=True, timeout=900)
        ok('the format change is printed', 'file format: 20241229 -> 2026' in r.stdout,
           r.stdout[-400:])
        text = open(out, encoding='utf-8').read()
        ok('no via carries the re-save\'s explicit no',
           len(re.findall(r'\(capping no\)', text)) <= 1, '')
        from check_drc import run_drc
        rows = run_drc(out, clearance=0.1, clearance_margin=0.1, quiet=True,
                       print_summary=False, baseline=base)
        viol = [x for x in rows if x.get('type') == 'via-in-paste' and not x.get('accepted')]
        prot = [x for x in rows if x.get('accepted') == 'protected-via-in-paste']
        ok('check_drc: no via-in-paste violation on the filled copy',
           not viol, f'{len(viol)} violations')
        ok('the vias in paste read filled+capped', len(prot) >= 1, str(len(prot)))
        pcb = parse_kicad_pcb(out)
        ok('the filled copy parses in KiCad 10 format', pcb.kicad_version >= 20250000)
    return True


if __name__ == '__main__':
    unit_checks()
    ran = end_to_end()
    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        sys.exit(1)
    print('\nAll checks passed' if ran else '\nUnit checks passed; end-to-end NOT RUN')
    if not ran:
        print('SKIP: the end-to-end re-save needs KiCad (pcbnew) to fill the board')
    sys.exit(0 if ran else SKIP_EXIT)
