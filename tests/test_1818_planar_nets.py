#!/usr/bin/env python3
"""#1818: check_floorplan reads a `.kicad_dru` that keeps nets on one layer
with no via, and errors on a pad such a net cannot reach.

The board is the issue's: cap_chain (J1, C1 on the P leg, C2 on the N leg,
J2) with a rules file keeping the four pair nets' tracks on one layer and
forbidding their vias. Placed as `same` (both headers at the edges), the
rule naming F.Cu routes 4 of 4; the same placement with the rule naming B.Cu
(`wronglayer`) routes 0 of 4, because every capacitor pad is on F.Cu -- and
both graded `PASS` before.

Checks:
  1. `net_keepouts.planar_nets` reads the four nets and their layer, and no
     net when vias are allowed; `via_count (max 0)` counts as no via; a rule
     scoped by a `(layer "F.Cu")` clause reads the same whether the via ban
     is a second rule or the same constraint (`disallow track via`);
  2. `wronglayer` grades ERROR (`planar_nets`, exit 4) naming C1 and C2, the
     pads, the nets, the layer and the rules; `same` still grades as before
     (`PASS`, no planar_nets line); the one-rule form grades ERROR too, and
     check_drc refuses a through via on one of its nets under the same file;
  3. `--plan-only` on `wronglayer`: a WARN for a capacitor with no declared
     face (placement may still turn it over) and an ERROR once the intent
     declares its face;
  4. control: with the rules file removed, `wronglayer` grades as `same`.
"""
import io
import contextlib
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
from kicad_parser import parse_kicad_pcb  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'cap_chain.kicad_pcb')
NETS = ('DPA_P', 'DPA_N', 'DPB_P', 'DPB_N')
_ANY = ' || '.join(f"A.NetName == '{n}'" for n in NETS)
BAN = (f'(rule "pair_one_layer"\n  (condition "({_ANY}) && A.Type == \'Track\' && '
       "A.Layer != '{layer}'\")\n  (constraint disallow track))\n")
NO_VIA = f'(rule "pair_no_vias"\n  (condition "{_ANY}")\n  (constraint disallow via))\n'
NO_VIA_COUNT = f'(rule "pair_zero_vias"\n  (condition "{_ANY}")\n  (constraint via_count (max 0)))\n'
# the same declaration by a `(layer ...)` clause: tracks kept off F.Cu, and
# the via banned either by NO_VIA beside it or in the one constraint
OFF_FRONT = f'(rule "pair_off_front"\n  (layer "F.Cu")\n  (condition "{_ANY}")\n  (constraint disallow track))\n'
ONE_RULE = OFF_FRONT.replace('disallow track', 'disallow track via')

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def dru(layer, *more):
    return '(version 1)\n' + BAN.format(layer=layer) + ''.join(more)


def write(path, text):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)


def planar(path):
    from net_keepouts import planar_nets
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return {v['name']: (v['layer'], v['rules']) for v in planar_nets(pcb, path).values()}


def floorplan(board, intent, *extra, refuse=None, code=None, accept=False):
    argv = PY + [os.path.join(REPO, 'py_tools', 'check_floorplan.py'), evidence(board, 'the board'),
                 '--intent', evidence(intent, 'the intent'), *extra]
    if accept:
        return check(argv, accept=True)
    return check(argv, refuse=refuse, code=code)


def planar_lines(out):
    return [l.strip() for l in out.splitlines() if 'planar_nets:' in l]


def main():
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, 'in.kicad_pcb')
        check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, src], accept=True)
        write(os.path.join(td, 'in.kicad_dru'), dru('F.Cu', NO_VIA))
        same = os.path.join(td, 'same.kicad_pcb')
        check(PY + [os.path.join(REPO, 'py_placer', 'place_pose.py'), src, same,
                    'set', 'J1', '6.15', '15', 'set', 'J2', '29.85', '15'], accept=True)
        evidence(os.path.splitext(same)[0] + '.kicad_dru', "same's rules file")
        wrong = os.path.join(td, 'wronglayer.kicad_pcb')
        check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), same, wrong], accept=True)
        write(os.path.join(td, 'wronglayer.kicad_dru'), dru('B.Cu', NO_VIA))

        print("1. planar_nets")
        got = planar(same)
        ok(sorted(got) == sorted(NETS) and {g[0] for g in got.values()} == {'F.Cu'}
           and all(g[1] == ['pair_no_vias', 'pair_one_layer'] for g in got.values()),
           f"four nets on F.Cu, both rules named: {got}")
        ok({g[0] for g in planar(wrong).values()} == {'B.Cu'}, "the B.Cu rule reads B.Cu")
        probe = os.path.join(td, 'probe.kicad_pcb')
        check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), same, probe], accept=True)
        write(os.path.join(td, 'probe.kicad_dru'), dru('F.Cu'))
        ok(planar(probe) == {}, "vias allowed: no net is planar")
        write(os.path.join(td, 'probe.kicad_dru'), dru('F.Cu', NO_VIA_COUNT))
        ok(sorted(planar(probe)) == sorted(NETS), "via_count (max 0) counts as no via")
        write(os.path.join(td, 'probe.kicad_dru'), '(version 1)\n' + OFF_FRONT + NO_VIA)
        two = planar(probe)
        ok(sorted(two) == sorted(NETS)
           and all(g == ('B.Cu', ['pair_no_vias', 'pair_off_front']) for g in two.values()),
           f"a (layer \"F.Cu\") track ban and a via ban beside it: four nets on B.Cu: {two}")
        write(os.path.join(td, 'probe.kicad_dru'), '(version 1)\n' + ONE_RULE)
        one = planar(probe)
        ok(sorted(one) == sorted(NETS)
           and all(g == ('B.Cu', ['pair_off_front']) for g in one.values()),
           f"both bans in the one constraint of that rule read the same nets and layer: {one}")
        write(os.path.join(td, 'probe.kicad_dru'), '(version 1)\n' + OFF_FRONT)
        ok(planar(probe) == {}, "that rule with the track ban alone: vias allowed, no net is planar")

        print("2. the grade")
        intent = os.path.join(td, 'intent.json')
        check(PY + [os.path.join(REPO, 'py_tools', 'check_floorplan.py'), same,
                    '--emit-intent', intent], accept=True)
        r = floorplan(wrong, intent, refuse='[ERROR] planar_nets', code=4)
        lines = planar_lines(r.stdout)
        ok(len(lines) == 2 and all(l.startswith('[ERROR]') for l in lines), f"two ERRORs: {lines}")
        c1 = next((l for l in lines if 'planar_nets: C1:' in l), '')
        ok(all(s in c1 for s in ('pad 1 (DPA_P) on F.Cu', 'pad 2 (DPB_P) on F.Cu', 'on B.Cu',
                                 'pair_no_vias', 'pair_one_layer')),
           f"C1's finding names its pads, nets, the layer and the rules: {c1}")
        r = floorplan(same, intent, accept=True)
        ok('PASS:' in r.stdout and not planar_lines(r.stdout), "control: `same` still passes")
        onerule = os.path.join(td, 'onerule.kicad_pcb')
        check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), same, onerule], accept=True)
        write(os.path.join(td, 'onerule.kicad_dru'), '(version 1)\n' + ONE_RULE)
        r = floorplan(onerule, intent, refuse='[ERROR] planar_nets', code=4)
        lines = planar_lines(r.stdout)
        ok(len(lines) == 2 and all(l.startswith('[ERROR]') and 'on B.Cu' in l
                                   and 'pair_off_front' in l for l in lines),
           f"the one-rule form: two ERRORs naming B.Cu and the rule: {lines}")
        # the grader's reading of that file: a through via on one of the nets
        with open(onerule, encoding='utf-8') as fh:
            text = fh.read().rstrip()
        with contextlib.redirect_stdout(io.StringIO()):
            pad = next(p for p in parse_kicad_pcb(onerule).footprints['C1'].pads
                       if p.net_name == 'DPA_P')
        m = re.search(r'\(net (\d+) "DPA_P"\)', text)
        net = f'(net {m.group(1)})' if m else '(net "DPA_P")'
        viaed = os.path.join(td, 'onerule_via.kicad_pcb')
        write(viaed, text[:-1] + f'  (via (at {pad.global_x - 1.2:.3f} {pad.global_y:.3f})'
              f' (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") {net})\n)\n')
        shutil.copy(os.path.join(td, 'onerule.kicad_dru'), os.path.join(td, 'onerule_via.kicad_dru'))
        check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), evidence(viaed, 'the board with a via')],
              refuse="Via:DPA_P not allowed by .kicad_dru rule 'pair_off_front'")
        print("  [ok] check_drc refuses a DPA_P via under the same file, by that rule")

        print("3. --plan-only")
        r = floorplan(wrong, intent, '--plan-only', accept=True)
        lines = planar_lines(r.stdout)
        ok(len(lines) == 2 and all(l.startswith('[warn ]') and 'may still turn it over' in l
                                   for l in lines),
           f"no declared face: a WARN per capacitor: {lines}")
        with open(intent, encoding='utf-8') as fh:
            doc = json.load(fh)
        doc.setdefault('blocks', []).append({'name': 'caps', 'refs': ['C1', 'C2'], 'side': 'F'})
        sided = os.path.join(td, 'intent_sided.json')
        write(sided, json.dumps(doc))
        r = floorplan(wrong, sided, '--plan-only', refuse='[ERROR] planar_nets')
        lines = planar_lines(r.stdout)
        ok(len(lines) == 2 and all(l.startswith('[ERROR]') for l in lines),
           f"a declared face: ERROR: {lines}")
        doc['blocks'][-1]['side'] = 'B'
        write(sided, json.dumps(doc))
        r = floorplan(wrong, sided, '--plan-only', accept=True)
        ok(not planar_lines(r.stdout), "declared on B.Cu, where the rule keeps the nets: clean")

        print("4. control: no rules file")
        os.remove(os.path.join(td, 'wronglayer.kicad_dru'))
        r = floorplan(wrong, intent, accept=True)
        ok('PASS:' in r.stdout and not planar_lines(r.stdout),
           "without the rules file the board grades as before")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
