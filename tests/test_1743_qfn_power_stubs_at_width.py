#!/usr/bin/env python3
"""#1743: a qfn_fanout run names each escape stub it drew at the track width
on a net that reads as power or ground.

A stub run via-drops the interior pads its --nets filter selects (#1717),
and that via-drop draws the pad-to-via stub at --width: --power-nets widens a
SURFACE stub where it fits (#1318), never a via-drop stub. Nothing said so,
so a ground or supply pin's whole connection to its via shipped at signal
width unremarked. Now JSON_SUMMARY `power_stubs_at_width` lists each such
stub (pad, net, width drawn, escape, why the net reads as power) with a
console NOTE, and the copper is unchanged.

On kicad_files/qfn_interior_pads.kicad_pcb (U1: interior GND pads 33/36/37,
only U1.33 gets a via-drop; perimeter two-pad net FOREIGN on 31/32), and on
a copy whose FOREIGN net is renamed to a supply-rail name:

  1. --nets '*': U1.33's via-drop stub is listed (ground by its name), with
     the console NOTE; a signal net is not.
  2. --power-nets GND at a wider width: U1.33 is still listed, as named, and
     its copper is identical to arm 1 (the via-drop takes no power width).
  3. --power-nets GND at the drawn width: nothing is listed.
  4. --nets '*' '!GND': nothing is listed.
  5. the renamed copy, --nets on the rail: its surface stubs are listed as
     'surface', with the --power-nets hint; with --power-nets naming it,
     they are not.
  6. the engine both fronts call publishes LAST_POWER_STUBS_AT_WIDTH.

    python3 tests/test_1743_qfn_power_stubs_at_width.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence             # noqa: E402

QFN = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
RAIL = 'VDD_IO'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def summary(out):
    m = re.search(r'^JSON_SUMMARY: (.*)$', out, re.M)
    return json.loads(m.group(1)) if m else None


def copper(path):
    """The board's segments and vias as sorted tuples (UUIDs aside)."""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(path))
    r = lambda v: round(v, 4)
    rows = [('s', s.net_id, s.layer, r(s.width), r(s.start_x), r(s.start_y),
             r(s.end_x), r(s.end_y)) for s in pcb.segments]
    rows += [('v', v.net_id, r(v.x), r(v.y), r(v.size), r(v.drill)) for v in pcb.vias]
    return sorted(rows, key=str)


def renamed(dst):
    """The tracked board with FOREIGN renamed to a supply-rail name."""
    with open(QFN, encoding='utf-8') as f:
        text = f.read()
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(text.replace('"FOREIGN"', f'"{RAIL}"'))
    return evidence(dst)


def cli(tmp):
    base = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'qfn_fanout.py')]
    b = os.path.join(tmp, 'b.kicad_pcb')
    shutil.copyfile(QFN, b)
    br = renamed(os.path.join(tmp, 'br.kicad_pcb'))

    def run(board, name, *extra):
        out = os.path.join(tmp, f'{name}.kicad_pcb')
        r = run_check(base + [board, '--component', 'U1', '--output', out] + list(extra),
                      accept=True)
        s = summary(r.stdout)
        if s is None:
            check(f"{name}: a JSON_SUMMARY", False, r.stdout[-400:])
            return {}, r.stdout, out
        return s, r.stdout, out

    print("1. --nets '*'")
    s, log, out1 = run(b, 'glob', '--nets', '*')
    rows = s.get('power_stubs_at_width')
    check("U1.33's via-drop stub is listed, ground by its name",
          rows == [{'pad': 'U1.33', 'net': 'GND', 'width': 0.127,
                    'escape': 'via-drop', 'why': 'its name reads as ground'}],
          repr(rows))
    check("the console says so",
          'power/ground escape stub(s) drawn at the track width' in log
          and 'only a wider --width widens it' in log)

    print("2. --power-nets GND, wider than drawn")
    s, _log, out2 = run(b, 'named', '--nets', '*', '--power-nets', 'GND',
                        '--power-nets-widths', '0.3')
    rows = s.get('power_stubs_at_width') or []
    check("U1.33 is still listed, as named",
          [(r['pad'], r['escape'], r['why']) for r in rows]
          == [('U1.33', 'via-drop', '--power-nets names it at 0.3 mm')], repr(rows))
    check("its copper is identical to arm 1", copper(out1) == copper(out2))

    print("3. --power-nets GND at the drawn width")
    s, _log, _out = run(b, 'named_eq', '--nets', '*', '--power-nets', 'GND',
                        '--power-nets-widths', '0.127')
    check("nothing is listed", s.get('power_stubs_at_width') == [],
          repr(s.get('power_stubs_at_width')))

    print("4. --nets '*' '!GND'")
    s, log, _out = run(b, 'signal', '--nets', '*', '!GND')
    check("nothing is listed, no NOTE",
          s.get('power_stubs_at_width') == [] and 'power/ground escape stub' not in log,
          repr(s.get('power_stubs_at_width')))

    print("5. a perimeter rail's surface stubs")
    s, log, _out = run(br, 'rail', '--nets', RAIL)
    rows = s.get('power_stubs_at_width') or []
    check("both are listed as surface stubs, a supply rail by name",
          sorted((r['pad'], r['escape'], r['why']) for r in rows)
          == [('U1.31', 'surface', 'its name reads as a supply rail'),
              ('U1.32', 'surface', 'its name reads as a supply rail')], repr(rows))
    check("the console names --power-nets for them",
          f'pass --power-nets {RAIL} with --power-nets-widths' in log)
    s, _log, _out = run(br, 'rail_named', '--nets', RAIL, '--power-nets', RAIL,
                        '--power-nets-widths', '0.2')
    check("named by --power-nets, they are not listed",
          s.get('power_stubs_at_width') == [], repr(s.get('power_stubs_at_width')))


def engine():
    print("6. the engine (the fanout tab's call)")
    import qfn_fanout
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(QFN)
        qfn_fanout.generate_qfn_fanout(
            pcb.footprints['U1'], pcb, net_filter=['*'], layer='F.Cu',
            track_width=0.1, clearance=0.1, grid_step=0.05, escape_method='stub',
            via_size=0.45, via_drill=0.25)
    got = [r['pad'] for r in qfn_fanout.LAST_POWER_STUBS_AT_WIDTH]
    check("LAST_POWER_STUBS_AT_WIDTH names U1.33", got == ['U1.33'], repr(got))


def main():
    tmp = tempfile.mkdtemp(prefix='t1743_')
    try:
        cli(tmp)
        engine()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
