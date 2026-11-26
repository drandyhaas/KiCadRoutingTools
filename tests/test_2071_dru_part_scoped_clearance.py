#!/usr/bin/env python3
"""#2071: a `.kicad_dru` clearance rule that keeps a net away from ONE part is
routed to and graded, under both of KiCad's spellings.

    (rule "phase_c_off_C12" (constraint clearance (min 3mm))
      (condition "A.NetName == '/OUT_E_PHASE_C' && B.Reference == 'C12'"))

or `B.memberOfFootprint('C12')`. route.py used to print `NOT honoured` and
lay /OUT_E_PHASE_C 0.31 mm from C12's pad 2 (kicad-cli: 3 violations of the
rule, the first at 1.857 mm to pad 1), and check_drc printed `NOT GRADED
(clearance)` and `NO DRC VIOLATIONS FOUND!`. `B.Reference` did not parse at
all; `memberOfFootprint()` parsed and no channel priced one part.

  1. the parse: `B.Reference == 'C12'` is read (no `unsupported`), and both
     spellings derive one keep-out per C12 pad, at the rule's 3 mm, bound to
     /OUT_E_PHASE_C alone; a `B.Reference != 'C12'` rule (every OTHER part)
     derives none and stays NOT honoured;
  2. route /OUT_E_PHASE_C under each spelling: the route step says the rule
     is honoured, the net routes, and every track and via of the net is at
     least 3.0 mm edge to edge from C12's pads, MEASURED ON THE BOARD;
  3. grade: the same net routed with no rule file runs closer than 3 mm to
     C12, and check_drc with the rule beside it exits 1 naming a
     `phase_c_off_C12` violation (both spellings), not NOT GRADED;
  4. control: a `B.NetName == 'GND'` rule (no part term) takes the scoped
     channel as before -- no part keep-out, no part-scoped grade line, and it
     routes;
  5. control: a rule naming a part the net never comes near (C1) lays
     exactly the copper the rule-less route lays;
  6. (kicad-cli present) KiCad reports 0 violations of the rule on each
     routed board and >= 1 on the rule-less one. Without kicad-cli the file
     exits 77 after 1-5.

    python3 tests/test_2071_dru_part_scoped_clearance.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from shapely.geometry import LineString, Point                # noqa: E402

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from kicad_oracle import find_kicad_cli                       # noqa: E402
import run_utils                                              # noqa: E402

FIXTURE = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
NET = '/OUT_E_PHASE_C'
PART = 'C12'
FAR_PART = 'C1'
RULE = 'phase_c_off_C12'
VALUE = 3.0
SPELLINGS = {
    'reference': f"A.NetName == '{NET}' && B.Reference == '{PART}'",
    'member': f"A.NetName == '{NET}' && B.memberOfFootprint('{PART}')",
}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}"
          f"{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def dru_text(condition, name=RULE, value=VALUE):
    return (f'(version 1)\n(rule "{name}"\n\t(constraint clearance (min {value:g}mm))\n'
            f'\t(condition "{condition}"))\n')


def stage(tmp, name, condition=None):
    """A fresh copy of the fixture (with its .kicad_pro) and, given a
    condition, a .kicad_dru beside it."""
    board = os.path.join(tmp, f'{name}.kicad_pcb')
    run_utils.run_step([run_utils.tool('copy_board.py'), FIXTURE, board], outputs=[board])
    if condition is not None:
        with open(os.path.splitext(board)[0] + '.kicad_dru', 'w', encoding='utf-8') as fh:
            fh.write(dru_text(condition))
    return board


def route(board):
    out = os.path.splitext(board)[0] + '.out.kicad_pcb'
    _rc, log = run_utils.run_step([run_utils.tool('route.py'), board, out, '--nets', NET],
                                  outputs=[out])
    return out, log


def summary(log):
    for line in log.splitlines():
        if line.startswith('JSON_SUMMARY: '):
            return json.loads(line[len('JSON_SUMMARY: '):])
    return {}


def copper_gap(board, part=PART):
    """The smallest copper-edge-to-pad-edge gap between NET's tracks and vias
    and `part`'s pads on a shared copper layer, measured on the board file."""
    from design_rules import _pad_copper_geometry
    from net_queries import expand_pad_layers
    pcb = parse_kicad_pcb(run_utils.evidence(board, 'routed board'))
    nid = next(i for i, n in pcb.nets.items() if n.name == NET)
    copper = list(pcb.board_info.copper_layers)
    pads = [(p, set(expand_pad_layers(list(p.layers), copper)), _pad_copper_geometry(p))
            for p in pcb.footprints[part].pads]
    best, n = float('inf'), 0
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        n += 1
        line = LineString([(s.start_x, s.start_y), (s.end_x, s.end_y)])
        for _p, lays, (g, r) in pads:
            if s.layer in lays:
                best = min(best, g.distance(line) - r - s.width / 2)
    for v in pcb.vias:
        if v.net_id != nid:
            continue
        n += 1
        for _p, _lays, (g, r) in pads:
            best = min(best, g.distance(Point(v.x, v.y)) - r - v.size / 2)
    return best, n


def net_copper(board):
    pcb = parse_kicad_pcb(run_utils.evidence(board, 'routed board'))
    nid = next(i for i, n in pcb.nets.items() if n.name == NET)
    segs = sorted((round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
                   round(s.end_y, 4), s.layer, round(s.width, 4))
                  for s in pcb.segments if s.net_id == nid)
    vias = sorted((round(v.x, 4), round(v.y, 4), round(v.size, 4))
                  for v in pcb.vias if v.net_id == nid)
    return segs, vias


def derive(board):
    """(rules, part keep-outs) the route step derives for `board`."""
    from design_rules import DesignRules
    from net_keepouts import derive_part_clearance_keepouts
    pcb = parse_kicad_pcb(board)
    with contextlib.redirect_stdout(io.StringIO()):
        dr = DesignRules.from_project(pcb, board)
    return dr, derive_part_clearance_keepouts(dr, pcb)


def kicad_rule_violations(cli, board, rule=RULE):
    out = board + '.kc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                    '-o', out, board], capture_output=True, text=True, timeout=300)
    with open(run_utils.evidence(out, 'kicad-cli DRC report'), encoding='utf-8') as fh:
        return [v for v in json.load(fh).get('violations', [])
                if f"rule '{rule}'" in v.get('description', '')]


def main():
    tmp = tempfile.mkdtemp(prefix='t2071_')
    routed = {}
    try:
        print("1. the parse and the derivation")
        for form, cond in SPELLINGS.items():
            board = stage(tmp, f'parse_{form}', cond)
            dr, kos = derive(board)
            rule = next(r for r in dr.rules if r.name == RULE)
            check(f"{form}: the rule is read", rule.unsupported is None, str(rule.unsupported))
            pads = sorted(k['name'] for k in kos)
            check(f"{form}: one keep-out per {PART} pad", pads == [
                f"{RULE} ({PART}.1)", f"{RULE} ({PART}.2)"], str(pads))
            check(f"{form}: each at the rule's {VALUE:g} mm, bound to {NET} alone",
                  all(abs(k['clearance'] - VALUE) < 1e-9 and k['nets'] == frozenset({NET})
                      and not k['tracks_allowed'] and not k['vias_allowed'] for k in kos),
                  str([(k['clearance'], sorted(k['nets'])) for k in kos]))
            check(f"{form}: reported honoured", bool(rule.honoured)
                  and rule.router_unhonoured is None,
                  f"{rule.honoured!r} / {rule.router_unhonoured!r}")
        board = stage(tmp, 'parse_ne', f"A.NetName == '{NET}' && B.Reference != '{PART}'")
        dr, kos = derive(board)
        rule = next(r for r in dr.rules if r.name == RULE)
        check("Reference != (every other part): no keep-out, still NOT honoured",
              not kos and bool(rule.router_unhonoured) and not rule.honoured,
              f"{len(kos)} keep-out(s), {rule.router_unhonoured!r}")

        print("2. route under each spelling")
        for form, cond in SPELLINGS.items():
            out, log = route(stage(tmp, form, cond))
            routed[form] = out
            check(f"{form}: the route step honours the rule",
                  f".kicad_dru rule '{RULE}': honoured as a part keep-out" in log
                  and f".kicad_dru rule '{RULE}': NOT honoured" not in log,
                  next((l for l in log.splitlines() if RULE in l), '(no line)'))
            js = summary(log)
            check(f"{form}: {NET} routes", js.get('routed_single') == [NET]
                  and not js.get('failed_single') and not js.get('open_single'),
                  str({k: js.get(k) for k in ('routed_single', 'failed_single', 'open_single')}))
            gap, n = copper_gap(out)
            check(f"{form}: every item of {NET} is >= {VALUE:g} mm from {PART}'s pads",
                  n > 0 and gap >= VALUE - 1e-6, f"{n} item(s), closest {gap:.4f} mm")

        print("3. grade a board laid closer than the rule")
        plain_out, _log = route(stage(tmp, 'plain'))
        gap, n = copper_gap(plain_out)
        check(f"the rule-less route runs closer than {VALUE:g} mm to {PART} "
              f"(the defect's copper)", n > 0 and gap < VALUE - 0.5, f"closest {gap:.4f} mm")
        dru = os.path.splitext(plain_out)[0] + '.kicad_dru'
        for form, cond in SPELLINGS.items():
            with open(dru, 'w', encoding='utf-8') as fh:
                fh.write(dru_text(cond))
            r = run_utils.check([sys.executable, '-X', 'utf8', run_utils.tool('check_drc.py'),
                                 plain_out], refuse=f"rule '{RULE}'", code=1)
            out = r.stdout + r.stderr
            check(f"{form}: check_drc grades the rule (no NOT GRADED line)",
                  f"NOT GRADED (clearance): rule '{RULE}'" not in out
                  and 'NO DRC VIOLATIONS FOUND' not in out)
            check(f"{form}: the violation is a pad-segment row against {PART}",
                  f"({PART}." in out and f"Required clearance: {VALUE:.4f}mm (rule '{RULE}'"
                  in out)

        print("4. control: a net-to-net rule takes the scoped channel as before")
        gnd = stage(tmp, 'gnd', f"A.NetName == '{NET}' && B.NetName == 'GND'")
        _dr, kos = derive(gnd)
        check("no part keep-out derived", not kos, str(len(kos)))
        gnd_out, log = route(gnd)
        check("no part keep-out reported", 'part keep-out' not in log)
        js = summary(log)
        check(f"{NET} routes", js.get('routed_single') == [NET], str(js.get('routed_single')))
        r = subprocess.run([sys.executable, '-X', 'utf8', run_utils.tool('check_drc.py'),
                            gnd_out], capture_output=True, text=True, cwd=ROOT,
                           encoding='utf-8', errors='replace', timeout=900)
        out = r.stdout + r.stderr
        check("check_drc grades it by the scoped channel, not the part grade",
              r.returncode in (0, 1) and 'Traceback' not in out
              and f"'{RULE}'" in next((l for l in out.splitlines()
                                       if l.startswith('Class/type-scoped clearance rules')), '')
              and 'Part-scoped clearance rules' not in out, out[-800:])

        print(f"5. control: a rule naming {FAR_PART}, which {NET} never comes near")
        far_gap, _n = copper_gap(plain_out, FAR_PART)
        check(f"the rule-less route stays far from {FAR_PART}", far_gap > VALUE + 5.0,
              f"{far_gap:.3f} mm")
        far_out, log = route(stage(tmp, 'far', f"A.NetName == '{NET}' && "
                                                f"B.Reference == '{FAR_PART}'"))
        check(f"the rule naming {FAR_PART} is honoured", 'part keep-out' in log)
        check("it lays exactly the rule-less copper",
              net_copper(far_out) == net_copper(plain_out),
              f"{net_copper(far_out)} vs {net_copper(plain_out)}")

        cli = find_kicad_cli()
        if not cli:
            print("\nSKIP: 6. needs kicad-cli, not found (checks 1-5 ran and passed)")
            if failures:
                print(f"\nFAILED: {len(failures)} check(s): {failures}")
                return 1
            return 77
        print("6. KiCad's DRC")
        for form, out in routed.items():
            v = kicad_rule_violations(cli, out)
            check(f"{form}: kicad-cli reports 0 violations of the rule", not v,
                  '; '.join(x['description'] for x in v))
        v = kicad_rule_violations(cli, plain_out)
        check("the rule-less board violates it in kicad-cli (the grade's reference)",
              len(v) >= 1, str(len(v)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAILED: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
