"""repair_planes region joins under --escalation, end to end.

lvds_converter_dualclk_gnd's GND pour on B.Cu is cut in two by a locked /VCC
fence at x=100 (both layers), with one gap on F.Cu; GND asks for 0.6 mm.
The repair must join the two GND halves through that gap. On main, under
`--escalation off`, the join started at 0.2 mm and stopped at 0.4 mm in a
2.4 mm gap, and took a 0.2 mm strap through a 0.9 mm gap. Now:

* off, 2.4 mm gap: the join crosses at the net's 0.6 mm;
* off, 0.9 mm gap (0.6 mm does not fit): no GND copper below 0.6 mm crosses;
* off, two GND zone polygons, 3.0 mm gap: main widened the join to 0.8 mm;
  it is not stopped at the net's 0.6 mm (the first version of this fix
  widened from 0.6 to 1.2 and stopped at 0.6);
* fab: the same joins as before (0.4 mm, 0.2 mm);
* off, 0.9 mm gap, GND's 0.6 mm from a .kicad_dru rule rather than
  --power-nets: the guaranteed-join gate used to cross the gap at the run's
  --track-width after the joins had refused it, and the CLI's oracle width
  map carried no rule width. Now no GND copper below 0.6 mm crosses, and
  the oracle map holds GND at 0.6.

The KiCad-oracle recheck is off here (--no-kicad-recheck), so this runs
without KiCad.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARD = ROOT / 'kicad_files' / 'lvds_converter_dualclk_gnd.kicad_pcb'
FENCE_X = 100.0
GAP_Y = 67.0


def _seg(x0, y0, x1, y1, layer, n):
    return (f'\t(segment\n\t\t(start {x0} {y0})\n\t\t(end {x1} {y1})\n'
            f'\t\t(width 0.25)\n\t\t(locked yes)\n\t\t(layer "{layer}")\n'
            f'\t\t(net "/VCC")\n'
            f'\t\t(uuid "0f3e1c2a-0000-4000-8000-00000000000{n}")\n\t)\n')


def _two_zones(text):
    """The GND zone, as two polygons, one on each side of the fence."""
    s = text.index('\t(zone\n\t\t(net "/GND")')
    depth, i = 0, s
    while True:
        depth += {'(': 1, ')': -1}.get(text[i], 0)
        if depth == 0 and text[i] == ')':
            break
        i += 1
    block = text[s:i + 1]
    head = block[:block.index('\t\t(polygon')]
    uuid = re.search(r'\(uuid "([^"]+)"\)', head).group(1)

    def zone(n, x0, x1):
        return (head.replace(uuid, uuid[:-1] + str(n))
                + f'\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy {x0} 50.475) '
                f'(xy {x1} 50.475) (xy {x1} 84.525) (xy {x0} 84.525)\n'
                f'\t\t\t)\n\t\t)\n\t)')
    return (text[:s] + zone(1, 80.475, FENCE_X - 0.5) + '\n'
            + zone(2, FENCE_X + 0.5, 129.525) + text[i + 1:])


GND_DRU = """(version 1)
(rule "gnd"
  (constraint track_width (min 0.2mm) (opt 0.6mm))
  (condition "A.NetName == '/GND'"))
"""


def _board(td, gap, two_polygons=False, dru=None):
    half = gap / 2 + 0.125
    text = BOARD.read_text(encoding='utf-8').rstrip()
    fence = (_seg(FENCE_X, 50.2, FENCE_X, 84.8, 'B.Cu', 1)
             + _seg(FENCE_X, 50.2, FENCE_X, round(GAP_Y - half, 4), 'F.Cu', 2)
             + _seg(FENCE_X, round(GAP_Y + half, 4), FENCE_X, 84.8, 'F.Cu', 3))
    text = text[:-1] + fence + ')\n'
    if two_polygons:
        text = _two_zones(text)
    path = os.path.join(td, 'in.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    if dru:
        with open(os.path.join(td, 'in.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(dru)
    return path


# Runs the standalone CLI's main() in-process, then prints the oracle width
# map it built (repair_planes.LAST_NET_WIDTHS_BY_NAME) on a marked line.
_MAIN_AND_MAP = (
    "import json, sys\n"
    "sys.path.insert(0, sys.argv[1])\n"
    "sys.argv = ['repair_planes.py'] + sys.argv[2:]\n"
    "import repair_planes\n"
    "rc = repair_planes.main()\n"
    "print('ORACLE_WIDTH_MAP: ' + json.dumps("
    "repair_planes.LAST_NET_WIDTHS_BY_NAME))\n"
    "sys.exit(rc or 0)\n")


def _run(gap, policy, two_polygons=False, dru=None):
    """(widths of the new GND copper that crosses the fence on F.Cu, the
    CLI's oracle width map). Without ``dru`` GND asks 0.6 mm through
    --power-nets; with it, through the rule alone."""
    with tempfile.TemporaryDirectory() as td:
        board = _board(td, gap, two_polygons, dru)
        out = os.path.join(td, 'out.kicad_pcb')
        power = ([] if dru else
                 ['--power-nets', '/GND', '--power-nets-widths', '0.6'])
        r = subprocess.run(
            [sys.executable, '-c', _MAIN_AND_MAP, str(ROOT / 'py_router'),
             board, out, '--nets', '/GND', '--plane-layers', 'B.Cu',
             '--escalation', policy, '--no-kicad-recheck'] + power,
            capture_output=True, text=True, timeout=1200)
        assert r.returncode == 0, (r.stdout + r.stderr)[-3000:]
        line = [l for l in r.stdout.splitlines()
                if l.startswith('ORACLE_WIDTH_MAP: ')]
        assert line, "the run printed no oracle width map: " + r.stdout[-2000:]
        pcb = parse_kicad_pcb(out)
        gnd = next(n for n, net in pcb.nets.items() if net.name == '/GND')
        return (sorted({round(s.width, 3) for s in pcb.segments
                        if s.net_id == gnd and s.layer == 'F.Cu'
                        and min(s.start_x, s.end_x) < FENCE_X
                        < max(s.start_x, s.end_x)}),
                json.loads(line[-1][len('ORACLE_WIDTH_MAP: '):]))


def _repair(gap, policy, two_polygons=False):
    return _run(gap, policy, two_polygons)[0]


def test_off_crosses_at_the_nets_width():
    assert _repair(2.4, 'off') == [0.6]


def test_off_lays_nothing_narrower_through_a_narrow_gap():
    assert _repair(0.9, 'off') == []


def test_off_is_not_narrower_than_main_on_two_polygons():
    assert _repair(3.0, 'off', two_polygons=True) == [0.8]


def test_fab_is_unchanged():
    assert _repair(2.4, 'fab') == [0.4]
    assert _repair(0.9, 'fab') == [0.2]


def test_off_holds_a_rule_width_in_the_gate_and_the_oracle_map():
    widths, oracle_map = _run(0.9, 'off', dru=GND_DRU)
    under = [w for w in widths if w < 0.6 - 1e-6]
    assert not under, (f"GND copper below its .kicad_dru 0.6 mm crosses the "
                       f"fence under off: {widths}")
    got = (oracle_map.get('net_track_widths') or {}).get('/GND')
    assert got == 0.6, (f"the CLI's oracle width map does not hold GND at its "
                        f"rule's 0.6 mm: {oracle_map}")


TESTS = [test_off_crosses_at_the_nets_width,
         test_off_lays_nothing_narrower_through_a_narrow_gap,
         test_off_is_not_narrower_than_main_on_two_polygons,
         test_fab_is_unchanged,
         test_off_holds_a_rule_width_in_the_gate_and_the_oracle_map]

if __name__ == '__main__':
    for t in TESTS:
        t()
        print(f"ok  {t.__name__}")
    print(f"{len(TESTS)} passed")
