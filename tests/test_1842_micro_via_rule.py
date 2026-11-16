#!/usr/bin/env python3
"""#1842: a step that lays micro vias holds them by a `.kicad_dru` rule of
their own, and the board's minimums stay the through vias'.

KiCad grades a micro via's ring at min_via_annular_width and its drill at
min_hole_clearance, as it grades a through via's. A micro via laid at the
copper clearance (0.25/0.1 mm at 0.1 mm) sits under both on an ordinary
board (0.1 / 0.2 mm), and the writeback used to lower the two board-wide
minimums for it, which graded every through via looser too. Now the
writeback keeps `fix_kicad_drc_settings.MICRO_VIA_RULE`, conditioned on
`A.Via_Type == 'Micro'`, in the board's `.kicad_dru`.

The fixture is tests/fixtures/1842/bga_under (see
tests/test_1842_via_span_fanout.py), fanned out with `--via-span F.Cu
In1.Cu`: 12 micro vias in the balls.

Checks:
  1. the fan-out leaves the project's annular and copper-to-hole minimums at
     the board's own and writes the rule: ring 0.075 mm, hole clearance the
     micro vias' smallest, under the board's 0.2;
  2. the rules file is merged, never overwritten: a board's own rule stays
     as it was (CRLF kept), the micro via rule is added once, a re-run
     writes nothing, a lower value replaces it in place, a higher one does
     not raise it;
  3. design_rules reads it back: a micro via resolves the rule's ring and a
     through via the board's minimum, and a via a step sizes itself is not
     raised by it (`via_floors`, #1676); check_drc grades the board clean;
  4. kicad-cli (when installed) accepts the micro vias at the rule and still
     grades a through via under the board's ring and hole clearance (one
     added beside the part) at the board's minimums.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import uuid

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
import fix_kicad_drc_settings as fx  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1842', 'bga_under')
KICAD_CLI = shutil.which('kicad-cli') or next(
    (p for p in ('/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli',
                 '/usr/bin/kicad-cli') if os.path.isfile(p)), None)
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def fanned(td):
    board = os.path.join(td, 'in.kicad_pcb')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), FIX + '.kicad_pcb', board],
          accept=True)
    out = os.path.join(td, 'span.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'bga_fanout.py'), board, '-o', out,
                    '--component', 'U1', '--clearance', '0.1', '--track-width', '0.1',
                    '--via-span', 'F.Cu', 'In1.Cu'], accept=True, timeout=1800)
    return evidence(out, 'the fanned board'), (r.stdout or '') + (r.stderr or '')


def rule_text(path):
    with open(path, encoding='utf-8', newline='') as fh:
        return fh.read()


def main():
    with open(FIX + '.kicad_pro', encoding='utf-8') as fh:
        own = json.load(fh)['board']['design_settings']['rules']
    with tempfile.TemporaryDirectory() as td:
        print("1. a micro via span's fan-out")
        out, log = fanned(td)
        with open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8') as fh:
            rules = json.load(fh)['board']['design_settings']['rules']
        keys = ('min_via_annular_width', 'min_hole_clearance')
        ok(all(rules.get(k) == own.get(k) for k in keys),
           f"the board's annular and hole minimums are kept: {[rules.get(k) for k in keys]}")
        dru = os.path.splitext(out)[0] + '.kicad_dru'
        doc = rule_text(evidence(dru, 'the rules file'))
        ok(doc.count(f'(rule "{fx.MICRO_VIA_RULE}"') == 1
           and f'(condition "{fx.MICRO_VIA_CONDITION}")' in doc
           and '(constraint annular_width (min 0.075mm))' in doc
           and '(constraint hole_clearance (min 0.1' in doc,
           f"and the rule holds the micro vias:\n{doc}")
        ok("Micro via rule (#1842)" in log, "the run says so")

        print("2. the rules file is merged")
        tmp = os.path.join(td, 'merge.kicad_pcb')
        own_rule = '(version 1)\r\n(rule "user rule"\r\n\t(condition "A.NetClass == \'X\'")\r\n' \
                   '\t(constraint clearance (min 0.3mm)))\r\n'
        with open(os.path.splitext(tmp)[0] + '.kicad_dru', 'w', encoding='utf-8',
                  newline='') as fh:
            fh.write(own_rule)
        w1 = fx.merge_micro_via_rule(tmp, {'annular_width': 0.075, 'hole_clearance': 0.17951})
        t1 = rule_text(os.path.splitext(tmp)[0] + '.kicad_dru')
        ok(w1 == {'annular_width': 0.075, 'hole_clearance': 0.1795}
           and t1.startswith(own_rule) and t1.count('(rule ') == 2
           and '\r\n\t(constraint hole_clearance (min 0.1795mm)))' in t1,
           f"the board's rule is kept as it was and ours is added, rounded down: {w1}")
        ok(fx.merge_micro_via_rule(tmp, {'annular_width': 0.075, 'hole_clearance': 0.1799})
           is None and rule_text(os.path.splitext(tmp)[0] + '.kicad_dru') == t1,
           "a re-run (a higher value) writes nothing")
        w3 = fx.merge_micro_via_rule(tmp, {'hole_clearance': 0.15})
        t3 = rule_text(os.path.splitext(tmp)[0] + '.kicad_dru')
        ok(w3 == {'annular_width': 0.075, 'hole_clearance': 0.15} and t3.count('(rule ') == 2
           and t3.startswith(own_rule) and '(min 0.15mm)' in t3,
           f"a lower value replaces it in place, once: {w3}")
        fresh = os.path.join(td, 'fresh.kicad_pcb')
        fx.merge_micro_via_rule(fresh, {'annular_width': 0.075})
        ok(rule_text(os.path.splitext(fresh)[0] + '.kicad_dru').startswith('(version 1)\n'),
           "a board with no rules file gets one")

        print("3. design_rules reads it back")
        from design_rules import DesignRules
        pcb = parse_kicad_pcb(out)
        dr = DesignRules.from_project(pcb, out)
        micro = next(v for v in pcb.vias if v.via_type == 'micro')
        through = replace_via(micro, 'through', ['F.Cu', 'B.Cu'])
        ring_m = dr.resolve_for_copper('annular_width', dr.via_item(micro), None, None)
        ring_t = dr.resolve_for_copper('annular_width', dr.via_item(through), None, None)
        ok(abs((ring_m.min or 0) - 0.075) < 1e-9 and abs((ring_t.min or 0) - 0.1) < 1e-9,
           f"a micro via's ring is the rule's, a through via's the board's: "
           f"{ring_m.min} / {ring_t.min}")
        ok(dr.via_floors(micro.net_id) == {}, "and a via a step sizes itself is not raised")
        d = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                        '--clearance-margin-mm', '0.0005'], accept=True)
        ok('NO DRC VIOLATIONS FOUND' in (d.stdout or ''), "check_drc grades the board clean")

        print("4. kicad-cli")
        if not KICAD_CLI:
            print("  (kicad-cli not installed: not run)")
        else:
            kicad_rows(td, out)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


def replace_via(v, kind, layers):
    from dataclasses import replace
    return replace(v, via_type=kind, layers=list(layers))


def kicad_rows(td, out):
    """The fanned board plus a through via 0.45/0.3 (ring 0.075, under the
    board's 0.1) whose drill stands 0.18 mm from another net's track (under
    the board's 0.2; the copper gap is 0.105), clear of the part."""
    probe = os.path.join(td, 'probe.kicad_pcb')
    shutil.copy(os.path.splitext(out)[0] + '.kicad_pro', os.path.splitext(probe)[0] + '.kicad_pro')
    shutil.copy(os.path.splitext(out)[0] + '.kicad_dru', os.path.splitext(probe)[0] + '.kicad_dru')
    text = rule_text(out)
    extra = (f'\t(via (at 27 3) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net "S01") '
             f'(uuid "{uuid.uuid4()}"))\n'
             f'\t(segment (start 26 3.38) (end 28 3.38) (width 0.1) (layer "F.Cu") '
             f'(net "S02") (uuid "{uuid.uuid4()}"))\n')
    i = text.rindex(')')
    with open(probe, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text[:i] + extra + text[i:])
    rep = os.path.join(td, 'kc.json')
    r = subprocess.run([KICAD_CLI, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '-o', rep, probe], capture_output=True, text=True)
    if r.returncode not in (0, 5) or not os.path.isfile(rep):
        ok(False, f"kicad-cli ran: exit {r.returncode} {r.stderr[-400:]}")
        return
    with open(rep, encoding='utf-8') as fh:
        found = json.load(fh)['violations']
    rows = [(v['type'], 'micro' if any('Micro via' in it['description'] for it in v['items'])
             else 'other') for v in found
            if v['type'] in ('annular_width', 'hole_clearance', 'clearance', 'via_diameter',
                             'drill_out_of_range', 'microvia_drill_out_of_range')]
    ok(sorted(rows) == [('annular_width', 'other'), ('hole_clearance', 'other')],
       f"KiCad flags the through via's ring and hole clearance and no micro via: {rows}")


if __name__ == '__main__':
    sys.exit(main())
