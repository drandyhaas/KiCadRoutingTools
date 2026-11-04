#!/usr/bin/env python3
"""#1553: `board_score` reads a net name that contains a SPACE.

A hierarchical net carries its sheet path, and KiCad allows a space in a sheet
name. `score_connectivity` read check_connected's per-net lines with `\\S+`, so

- an unrouted net with a space lost its NAME (the count, read off the section
  header, stayed): `unrouted 1, unrouted_net_names []`;
- a broken net's header did not match at all, so its stranded pads were filed
  under the broken net printed BEFORE it.

This runs the REAL check_connected on a four-net fixture whose nets are renamed
in a temp copy: two unrouted nets (one spaced), and two broken nets (a stub from one
pad each, the other pad bare), the plain one printed first and the spaced one
after it -- exactly the order that misfiled the pad.

Run:  python3 tests/test_1553_board_score_spaced_net_names.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import board_score  # noqa: E402

FIXTURE = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')

UNROUTED = '/sheet one/DPA_P'      # net 1, no copper
UNROUTED_PLAIN = 'DPA_N'           # net 3, no copper
BROKEN_PLAIN = 'DPB_P'             # net 2, stub from C1.2, J2.1 bare
BROKEN_SPACED = '/sheet two/DPB_N'  # net 4, stub from C2.2, J2.2 bare

STUB = """\t(segment
\t\t(start {x0} {y})
\t\t(end {x1} {y})
\t\t(width 0.2)
\t\t(layer "F.Cu")
\t\t(net "{net}")
\t\t(uuid "{uuid}")
\t)
"""

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}"
          + (f' -- {detail}' if detail and not ok else ''))


def build(tmp):
    with open(FIXTURE, encoding='utf-8', newline='') as fh:
        txt = fh.read()
    assert txt.count('(net "DPA_P")') == 2 and txt.count('(net "DPB_N")') == 2, \
        'fixture changed: expected two pads on each renamed net'
    txt = txt.replace('(net "DPA_P")', f'(net "{UNROUTED}")')
    txt = txt.replace('(net "DPB_N")', f'(net "{BROKEN_SPACED}")')
    stubs = (STUB.format(x0=18.48, x1=20.0, y=13.0, net=BROKEN_PLAIN,
                         uuid='15530000-0000-4000-8000-000000000001')
             + STUB.format(x0=18.48, x1=20.0, y=17.0, net=BROKEN_SPACED,
                           uuid='15530000-0000-4000-8000-000000000002'))
    end = txt.rstrip().rfind(')')
    txt = txt[:end] + stubs + txt[end:]
    path = os.path.join(tmp, 'spaced.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(txt)
    return path


def main():
    with tempfile.TemporaryDirectory(prefix='t1553_') as tmp:
        board = build(tmp)
        _, out = board_score.run_tool(ROOT, 'check_connected.py', board)
        # Verify the INPUT first: check_connected itself must see the shape
        # this test is about, or the score below grades nothing.
        check('check_connected names the spaced unrouted net',
              f'    {UNROUTED} (2 pads)' in out, out[-600:])
        check('check_connected prints the plain broken net before the spaced one',
              0 <= out.find(f'  {BROKEN_PLAIN} (net ')
              < out.find(f'  {BROKEN_SPACED} (net '), out[-600:])

        r = board_score.score_connectivity(ROOT, board)

    check('the unrouted count is 2', r.get('unrouted') == 2, r.get('unrouted'))
    check('...and the unrouted NAME survives its space',
          r.get('unrouted_net_names') == sorted([UNROUTED, UNROUTED_PLAIN]),
          r.get('unrouted_net_names'))
    check('...with its pad count',
          r.get('unrouted_pads') == {UNROUTED: 2, UNROUTED_PLAIN: 2},
          r.get('unrouted_pads'))
    det = r.get('broken_detail', {})
    check('both broken nets are in broken_detail',
          sorted(det) == sorted([BROKEN_PLAIN, BROKEN_SPACED]), sorted(det))
    plain = [(p['x'], p['y'], p['ref'])
             for p in det.get(BROKEN_PLAIN, {}).get('stranded_pads', [])]
    spaced = [(p['x'], p['y'], p['ref'], p['layer'])
              for p in det.get(BROKEN_SPACED, {}).get('stranded_pads', [])]
    check('the plain net keeps only its OWN stranded pad (J2.1 at y=15)',
          plain == [(26.0, 15.0, 'J2')], plain)
    check('the spaced net gets its stranded pad (J2.2 at y=17.54)',
          [s[:3] for s in spaced] == [(26.0, 17.54, 'J2')], spaced)
    check('the spaced net reads 2 components', det.get(BROKEN_SPACED, {})
          .get('components') == 2, det.get(BROKEN_SPACED))
    check('every name is listed in nets',
          set(r.get('nets', [])) == {UNROUTED, UNROUTED_PLAIN, BROKEN_PLAIN,
                                     BROKEN_SPACED},
          r.get('nets'))

    print(f'\n{passed} passed, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
