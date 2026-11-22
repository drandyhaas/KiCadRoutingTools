#!/usr/bin/env python3
"""bga_fanout.py escapes at the clearance a rule area sets over the ball field.

KiCad's last matching rule wins. A board's .kicad_dru spaced two net classes
0.5 mm apart (a class-pair rule per routing layer) and, LATER in the file,
set 0.1 mm for everything in a rule area around the BGA. Inside the area
KiCad grades the 0.1 mm rule; the fan-out priced every pair by its nets alone,
held the escape at 0.5 mm and dropped the balls that could not fit.

The fixture (tests/fixture_rule_area_bga.py) is a synthetic 8x8 BGA at 0.8 mm
pitch whose columns alternate between the two classes in pairs. kicad-cli
10.0.8 on the copper of the fan-out with no rules at all: 0 clearance
violations under the room rule, 181 under the class-pair rule alone -- so the
tight value is what KiCad grades in the area.

Checks (each a bga_fanout.py run; check_drc grades every output, and kicad-cli
too when it is installed, counted against the input board):
  1. no .kicad_dru rule: all 64 balls escape (the board escapes at 0.1 mm);
  2. the class-pair rule alone binds: fewer than 64 escape (was the result
     with the room rule too: 42);
  3. the class-pair rule then the room: all 64 escape, no new violation, and
     the run names the room;
  4. the room first, then the class-pair rule: the later rule binds what it
     binds inside the room, so the escape is the class-pair rule's;
  5. a room that ends at the ball field: the escape copper past it is graded
     at the class-pair rule and re-escaped (the outside grading does work:
     priced at the room's view it leaves 5 check_drc rows, kicad-cli 4), and
     the output is clean;
  6. a room whose condition is more than area predicates, and 7. a room that
     leaves balls out: the escape keeps the class-pair values (the wider
     side) and says why;
  8. `--escalation off` with the class-pair rule alone: a rule scoped with
     `(layer ...)` binds nothing asked without a layer, so the check that
     sends an escape to the grading never saw it, and every ball shipped with
     142 kicad-cli violations of it; now it is graded as in row 2;
  9. `PartAreaView.holds` reads an item's copper against the area.

    python3 tests/test_bga_fanout_rule_area_precedence.py
"""
import collections
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
import fixture_rule_area_bga as fx  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
ARGS = ['--component', 'U1', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
        '--track-width', '0.1', '--clearance', '0.1', '--via-size', '0.3',
        '--via-drill', '0.15', '--no-fix-drc-settings']
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def kicad_cli():
    for c in (os.environ.get('KICAD_CLI'), shutil.which('kicad-cli'),
              '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'):
        if c and os.path.isfile(c):
            return c
    return None


def fan(board, extra=()):
    out = board.replace('.kicad_pcb', '_fan.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'bga_fanout.py'), evidence(board),
                    '--output', out] + ARGS + list(extra), accept=True, timeout=900)
    evidence(out, 'the fan-out output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout, re.M)
    if not m:
        raise AssertionError(f"BROKEN TEST: no JSON_SUMMARY\n{r.stdout[-1500:]}")
    return out, json.loads(m.group(1)), r.stdout


def drc_rows(board):
    r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), evidence(board),
                    '--no-size-checks', '--max-print', '0'], accept=True)
    out = r.stdout
    if 'NO DRC VIOLATIONS' not in out and 'violations (' not in out:
        raise AssertionError(f"BROKEN TEST: no verdict from check_drc\n{out[-1500:]}")
    return {k: int(n) for k, n in re.findall(r'^(\S+) violations \((\d+)\)', out, re.M)}


def kicad_new(cli, inp, out):
    """kicad-cli's error-severity violations on `out` that `inp` lacks, by type."""
    def grade(b):
        js = b + '.drc.json'
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-error',
                        '-o', js, b], capture_output=True, timeout=600)
        with open(evidence(js, 'the kicad-cli report'), encoding='utf-8') as fh:
            return collections.Counter(v['type'] for v in json.load(fh)['violations'])
    a, b = grade(inp), grade(out)
    return {k: b[k] - a[k] for k in b if b[k] > a[k]}


def clean(cli, inp, out, what):
    rows = drc_rows(out)
    ok(not rows, f"{what}: check_drc finds no violation ({rows})")
    if cli:
        new = kicad_new(cli, inp, out)
        ok(not new, f"{what}: kicad-cli finds no violation the input lacks ({new})")


def holds_row(td):
    print("9. PartAreaView.holds reads copper against the area")
    from kicad_parser import parse_kicad_pcb, Segment, Via
    from bga_fanout.area_view import part_view
    board = fx.write(os.path.join(td, 'h'), 'room', fx.WIDE + fx.ROOM, margin=0.0)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
        view = part_view(pcb, pcb.footprints['U1'])
    ok(view is not None and abs(view.value - 0.1) < 1e-9,
       f"the room is read over the ball field ({view and view.describe()})")
    if view is None:
        return
    # the area's edge is 3.2 mm from the part's centre (15, 15)
    seg = lambda x0, x1, w=0.1: Segment(start_x=x0, start_y=15.0, end_x=x1, end_y=15.0,
                                        width=w, layer='In1.Cu', net_id=1)
    ok(view.holds('seg', seg(17.0, 19.0)), "a track crossing the edge is in the area")
    ok(view.holds('seg', seg(18.22, 19.0)),
       "a track whose copper just reaches over the edge is in it")
    ok(not view.holds('seg', seg(18.26, 19.0)), "a track stopping short of it is not")
    ok(view.holds('via', Via(x=18.3, y=15.0, size=0.3, drill=0.15,
                             layers=['F.Cu', 'B.Cu'], net_id=1)),
       "a via whose disc meets the area is in it")
    ok(view.holds('pad', pcb.footprints['U1'].pads[0]), "a ball is in it")
    ok(not view.holds('pad', pcb.footprints['J1'].pads[0]), "a pin of J1 is not")


def main():
    cli = kicad_cli()
    if not cli:
        print("  NOTE: kicad-cli not found; check_drc alone grades the outputs")
    with tempfile.TemporaryDirectory() as td:
        print("1. no rule: the board escapes at 0.1 mm")
        _o, js, _log = fan(fx.write(td, 'none', ''))
        ok(js['escaped'] == 64 and js['failed'] == 0, f"64 balls escape: {js['escaped']}")

        print("2. the class-pair rule alone")
        inp = fx.write(td, 'wide', fx.WIDE)
        out, js, _log = fan(inp)
        wide = js['escaped']
        ok(wide < 64, f"the rule binds the escape: {wide} of 64 escape")
        clean(cli, inp, out, "class-pair rule")

        print("3. the class-pair rule, then the room")
        inp = fx.write(td, 'room', fx.WIDE + fx.ROOM)
        out, js, log = fan(inp)
        ok(js['escaped'] == 64 and js['failed'] == 0,
           f"every ball escapes (was {wide}): {js['escaped']}")
        ok("Rule area: U1's ball field lies in rule 'BGA Tight Clearance'" in log,
           "the run names the room it escapes in")
        clean(cli, inp, out, "room after the rule")

        print("4. the room, then the class-pair rule")
        inp = fx.write(td, 'room_first', fx.ROOM + fx.WIDE)
        out, js, _log = fan(inp)
        ok(js['escaped'] == wide, f"the later rule still binds: {js['escaped']} (wide {wide})")
        clean(cli, inp, out, "room before the rule")

        print("5. a room that ends at the ball field")
        inp = fx.write(os.path.join(td, 'small'), 'room', fx.WIDE + fx.ROOM, margin=0.0)
        out, js, log = fan(inp)
        ok(js['escaped'] == 64, f"every ball escapes: {js['escaped']}")
        ok(re.search(r'\(#1529, #1565\): \d+ pair\(s\) of this escape fell short', log)
           is not None, "the copper past the room is graded at the class-pair rule")
        clean(cli, inp, out, "small room")

        print("6. a room rule the area reader does not read")
        inp = fx.write(os.path.join(td, 'unread'), 'room', fx.WIDE + fx.ROOM.replace(
            "A.intersectsArea('BGA_ROOM')", "A.intersectsArea('BGA_ROOM') && A.Type == 'track'"))
        out, js, log = fan(inp)
        ok(js['escaped'] == wide, f"the escape keeps the class-pair values: {js['escaped']}")
        ok("is not read for U1's ball field (its condition is more than rule-area "
           "predicates)" in log, "and says why")

        print("7. a room that leaves balls out")
        inp = fx.write(os.path.join(td, 'partial'), 'room', fx.WIDE + fx.ROOM, margin=-0.9)
        out, js, log = fan(inp)
        ok(js['escaped'] == wide, f"the escape keeps the class-pair values: {js['escaped']}")
        ok("is not applied to U1's ball field: its area does not enclose ball" in log,
           "and says why")

        print("8. the class-pair rule alone, no rescue below the run clearance")
        inp = fx.write(os.path.join(td, 'eoff'), 'wide', fx.WIDE)
        out, js, _log = fan(inp, ['--escalation', 'off'])
        ok(js['escaped'] == wide, f"the layer-scoped rule is graded: {js['escaped']} escape "
                                  f"(was all 64, 142 kicad-cli violations)")
        clean(cli, inp, out, "--escalation off")

        holds_row(td)
    print()
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
