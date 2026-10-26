#!/usr/bin/env python3
"""#1405: a multi-pad `unconnected-*` net with no copper was never routed and
every DONE grader passed it, while KiCad reports it unconnected.

check_connected skipped such nets (#479) and the wildcard router drops them, so
a board with a JST-SH's two `MP` tabs on one auto-named net read "ALL NETS
FULLY CONNECTED", board_score blocking 0 and check_complete DONE. Now:

  1. check_connected reports the net unrouted (default grading), board_score
     counts it in `unrouted`, and check_complete is not DONE.
  2. A footprint that declares its same-number pads jumpered
     (`duplicate_pad_numbers_are_jumpers yes`), or the two pads in a
     `jumper_pad_groups` group, has nothing left to route: skipped.
  3. A footprint declaring `duplicate_pad_numbers_are_jumpers no` (what the
     stock libraries write) is open.
  4. Pads of two DIFFERENT footprints are never joined by a jumper
     declaration.

    python3 tests/test_1405_multipad_noconnect_open.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import run_utils                                               # noqa: E402
from check_connected import run_connectivity_check             # noqa: E402

NC = 'unconnected-(J1-PadMP)'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board(jumper_clause='', second_ref='J1', copy=False):
    """Two MP pads on one auto-named net (in J1, or one each in J1 and J2 --
    or, with `copy`, in J1 and a second footprint ALSO referenced J1, as copies
    of a board panelised with one set of references are)."""
    second = ''
    if copy:
        second_ref = 'J1'
    pads2 = f'''
    (pad "MP" smd rect (at 8 0) (size 1.2 1.8) (layers "F.Cu" "F.Paste" "F.Mask")
      (net 9 "{NC}"))'''
    if second_ref != 'J1' or copy:
        second = f'''
  (footprint "test:J2" (layer "F.Cu") (at 150 100)
    (property "Reference" "{second_ref}" (at 0 0) (layer "F.SilkS"))
    (pad "MP" smd rect (at 0 0) (size 1.2 1.8) (layers "F.Cu" "F.Paste" "F.Mask")
      (net 9 "{NC}")))'''
        pads2 = ''
    return f"""(kicad_pcb (version 20241229) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (net 0 "") (net 1 "SIG") (net 9 "{NC}")
  (footprint "test:J1" (layer "F.Cu") (at 150 90)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
    {jumper_clause}
    (pad "MP" smd rect (at 0 0) (size 1.2 1.8) (layers "F.Cu" "F.Paste" "F.Mask")
      (net 9 "{NC}")){pads2}
    (pad "1" smd rect (at 2 4) (size 0.6 0.6) (layers "F.Cu") (net 1 "SIG"))
    (pad "2" smd rect (at 6 4) (size 0.6 0.6) (layers "F.Cu") (net 1 "SIG"))){second}
  (segment (start 152 94) (end 156 94) (width 0.25) (layer "F.Cu") (net 1) (uuid "00000000-0000-0000-0000-000000000001"))
)
"""


def unrouted(path, **kw):
    issues = run_connectivity_check(path, quiet=True, **kw)
    return [i['net_name'] for i in issues if i.get('unrouted')]


def write(tmp, name, text):
    p = os.path.join(tmp, name)
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return p


def main():
    tmp = tempfile.mkdtemp(prefix='t1405_')
    try:
        no = write(tmp, 'no.kicad_pcb',
                   board('(duplicate_pad_numbers_are_jumpers no)'))
        check('a footprint declaring jumpers "no": the net is OPEN',
              unrouted(no) == [NC], str(unrouted(no)))
        bare = write(tmp, 'bare.kicad_pcb', board())
        check('a footprint declaring nothing: OPEN', unrouted(bare) == [NC])

        yes = write(tmp, 'yes.kicad_pcb',
                    board('(duplicate_pad_numbers_are_jumpers yes)'))
        check('same-number pads declared jumpers: nothing to route',
              unrouted(yes) == [], str(unrouted(yes)))
        check('...also when the net is named (a declared jumper is joined)',
              unrouted(yes, net_patterns=[NC]) == [],
              str(unrouted(yes, net_patterns=[NC])))
        check('...and an undeclared one named is checked',
              unrouted(no, net_patterns=[NC]) == [NC])

        grp = write(tmp, 'grp.kicad_pcb',
                    board('(jumper_pad_groups ("MP" "X"))'))
        check('a jumper_pad_groups group holding the pad number: skipped',
              unrouted(grp) == [], str(unrouted(grp)))
        other = write(tmp, 'other.kicad_pcb',
                      board('(jumper_pad_groups ("1" "2"))'))
        check('a group that does not hold the net\'s pads joins nothing',
              unrouted(other) == [NC], str(unrouted(other)))

        two = write(tmp, 'two.kicad_pcb',
                    board('(duplicate_pad_numbers_are_jumpers yes)', 'J2'))
        check('one auto-name on two parts\' no-connect pins joins nothing: '
              'not open', unrouted(two) == [], str(unrouted(two)))
        cp = write(tmp, 'copy.kicad_pcb', board(copy=True))
        check('...nor on two copies of one part (a panel\'s one set of '
              'references)', unrouted(cp) == [], str(unrouted(cp)))
        check('...unless the net is named, which still checks it',
              unrouted(cp, net_patterns=[NC]) == [NC], str(unrouted(cp, net_patterns=[NC])))

        # The graders built on check_connected see it too.
        js = os.path.join(tmp, 'score.json')
        subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_tools', 'board_score.py'), no,
                        '--quiet', '--json', js], cwd=ROOT, capture_output=True,
                       text=True, encoding='utf-8', errors='replace')
        run_utils.evidence(js, 'board_score json')
        with open(js, encoding='utf-8') as fh:
            sc = json.load(fh)
        check('board_score counts it in `unrouted`',
              (sc['components']['unrouted'] or {}).get('count') == 1,
              str(sc['components']['unrouted'])[:200])
        check('...so blocking is not 0', sc.get('blocking') not in (0, None)
              and sc['blocking'] >= 1, str(sc.get('blocking')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
