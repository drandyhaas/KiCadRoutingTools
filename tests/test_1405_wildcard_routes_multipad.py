#!/usr/bin/env python3
"""#1405 (routing half): `--nets '*'` routes a multi-pad `unconnected-*` net.

Such a net is one footprint's same-number pads (shield or mounting tabs, a
reversible footprint's doubled pins). KiCad counts it unconnected until copper
joins the pads, so the wildcard no longer drops it, and routes it after the
named nets (a join no signal depends on). Two kinds stay out:
a single-pad no-connect (nothing to join) and a footprint that declares its
same-number pads jumpered (the part is the conductor).

    python3 tests/test_1405_wildcard_routes_multipad.py
"""
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from test_1405_multipad_noconnect_open import board, write, NC   # noqa: E402
from check_connected import run_connectivity_check               # noqa: E402
from kicad_parser import parse_kicad_pcb                         # noqa: E402
from net_queries import expand_net_patterns, multipad_unconnected_to_route  # noqa: E402
from net_ordering import order_pin_joins_last                    # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def wildcard(path):
    return expand_net_patterns(parse_kicad_pcb(path), ['*'], quiet=True)


def main():
    tmp = tempfile.mkdtemp(prefix='t1405w_')
    bare = write(tmp, 'bare.kicad_pcb', board())
    no = write(tmp, 'no.kicad_pcb', board('(duplicate_pad_numbers_are_jumpers no)'))
    yes = write(tmp, 'yes.kicad_pcb', board('(duplicate_pad_numbers_are_jumpers yes)'))
    grp = write(tmp, 'grp.kicad_pcb', board('(jumper_pad_groups ("MP" "X"))'))
    other = write(tmp, 'other.kicad_pcb', board('(jumper_pad_groups ("1" "2"))'))
    two = write(tmp, 'two.kicad_pcb', board(second_ref='J2'))

    check('undeclared footprint: the wildcard includes the net', NC in wildcard(bare))
    check('declared "no": included', NC in wildcard(no))
    check('declared jumpers "yes": left out', NC not in wildcard(yes))
    check('both pads in a jumper group: left out', NC not in wildcard(grp))
    check('a jumper group that does not hold them: included', NC in wildcard(other))
    check('pads of two footprints: left out (separate no-connect pins under '
          'one auto-name)', NC not in wildcard(two))
    cp = write(tmp, 'copy.kicad_pcb', board(copy=True))
    check('pads of two copies of one part (a panel): left out',
          NC not in wildcard(cp))
    check('helper names the net with its pad count',
          multipad_unconnected_to_route(parse_kicad_pcb(bare)) == {NC: 2})

    # A single-pad no-connect stays out.
    one = write(tmp, 'one.kicad_pcb', board().replace(
        '      (net 9 "%s"))\n    (pad "1"' % NC, '      (net 0 ""))\n    (pad "1"'))
    check('a single-pad no-connect is never included',
          NC not in wildcard(one) and multipad_unconnected_to_route(parse_kicad_pcb(one)) == {})

    # The joins route after every named net, each partition in its own order.
    order = [('unconnected-(U1-Pad14)', 5), ('ROW 2', 1), (NC, 9), ('COL 0', 2)]
    check('pin-join nets move behind the named nets',
          order_pin_joins_last(order) == [('ROW 2', 1), ('COL 0', 2),
                                          ('unconnected-(U1-Pad14)', 5), (NC, 9)])
    only = [(NC, 9), ('unconnected-(U1-Pad14)', 5)]
    check('a run of only pin-join nets keeps its order',
          order_pin_joins_last(only) == only)

    # End to end: route.py '*' joins the tabs, check_connected agrees.
    out = os.path.join(tmp, 'out.kicad_pcb')
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'route.py'),
                        bare, out, '--nets', '*'], capture_output=True, text=True)
    check('route.py --nets "*" exits 0', r.returncode == 0, r.stdout[-400:] + r.stderr[-400:])
    left = [i['net_name'] for i in run_connectivity_check(out, quiet=True)
            if i.get('unrouted')] if os.path.isfile(out) else ['<no output>']
    check('...and the tabs are joined (nothing unrouted)', left == [], str(left))

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
