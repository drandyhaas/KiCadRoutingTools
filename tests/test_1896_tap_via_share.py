#!/usr/bin/env python3
"""#1896: the plane repair counts the pads on each tap via, and
`--tap-via-share N` limits them.

The repair lets a pad reach its plane through a neighbouring pad's tap via,
joined by a surface track, when that is cheaper than a via of its own. The
per-pad log said `reused via at ...`; nothing counted how many pads sit on
one via, and nothing could ask for a via per pad.

The fixture (tests/fixtures/1896/taps) is synthetic: an 8-layer board with a
`VDD` pour on In1.Cu and seven `VDD` pads, three of them connector pads a
through via can reach (the four balls under the part have no legal through
via site and stay open on this board, with or without the option). Without
a limit J1.22 reaches the plane through J1.21's via.

Rows:
  1. the summary record (`tap_via_summary`): vias, shared vias, the most pads
     on one, the pads on shared vias, the limit and the pads past it;
  2. no limit: 2 tap vias, one serving 2 pads, said on the console and in
     JSON_SUMMARY.tap_vias;
  3. `--tap-via-share 1`: 3 tap vias, none shared, the same pads connected,
     check_drc clean on the taps;
  4. `--tap-via-share 2`: the copper of row 2 (two pads on one via is within
     the limit);
  5. route.py and repair_planes.py both take the flag, every plane-finalize
     call of route.py hands it to the repair, the GUI's route call passes it,
     and a recorded manifest carrying it converts to a plan step with
     `tap_via_share`.
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence, tool  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1896')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def repair(td, tag, *extra):
    sub = os.path.join(td, tag)
    os.makedirs(sub)
    for ext in ('.kicad_pcb', '.kicad_pro'):
        shutil.copy(evidence(os.path.join(FIX, 'taps' + ext), 'the fixture'),
                    os.path.join(sub, 'taps' + ext))
    out = os.path.join(sub, 'out.kicad_pcb')
    r = check(PY + [tool('repair_planes.py'), os.path.join(sub, 'taps.kicad_pcb'), out,
                    '--nets', 'VDD', '--plane-layers', 'In1.Cu', '--clearance', '0.1']
              + list(extra), accept=True, timeout=1800)
    log = (r.stdout or '') + (r.stderr or '')
    doc = {}
    for l in log.splitlines():
        at = l.find('JSON_SUMMARY: {')
        if at >= 0 and '"tap_vias"' in l:
            doc = json.loads(l[at + len('JSON_SUMMARY: '):])
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(evidence(out, 'the repaired board'))
    vdd = next(n for n in pcb.nets.values() if n.name == 'VDD').net_id
    vias = sorted((round(v.x, 3), round(v.y, 3)) for v in pcb.vias if v.net_id == vdd)
    return out, log, doc.get('tap_vias'), vias


def main():
    print("1. the summary record")
    from repair_planes import tap_via_summary
    got = tap_via_summary({(1, 1): ['A.1'], (2, 2): ['A.2', 'A.3', 'A.4']}, 2, ['A.4 (N)'])
    ok(got == {'vias': 2, 'shared_vias': 1, 'max_pads_per_via': 3,
               'pads_on_shared_vias': 3, 'limit': 2, 'over_limit': ['A.4 (N)']}, f"{got}")
    ok(tap_via_summary({}, None, [])['max_pads_per_via'] == 0, "an empty repair reads zeros")

    with tempfile.TemporaryDirectory() as td:
        print("2. no limit")
        _o, log, tv, vias0 = repair(td, 'none')
        ok(tv == {'vias': 2, 'shared_vias': 1, 'max_pads_per_via': 2,
                  'pads_on_shared_vias': 2, 'limit': None, 'over_limit': []},
           f"JSON_SUMMARY.tap_vias: {tv}")
        ok('Tap vias: 2 tap via(s), 1 serving more than one pad' in log
           and 'reused via at' in log, "and the console says a via is shared")

        print("3. --tap-via-share 1")
        out, log, tv, vias1 = repair(td, 'one', '--tap-via-share', '1')
        ok(tv and tv['vias'] == 3 and tv['shared_vias'] == 0 and tv['limit'] == 1
           and tv['over_limit'] == [], f"three vias, none shared: {tv}")
        ok(len(vias1) == len(vias0) + 1 and set(vias0) <= set(vias1),
           f"one more via on the board, the first two where they were: {vias1}")
        ok('reused via at' not in log, "no pad reuses a via")
        drc = check(PY + [tool('check_drc.py'), out, '--nets', 'VDD',
                          '--clearance-margin-mm', '0.0005'], accept=True)
        ok('NO DRC VIOLATIONS FOUND' in (drc.stdout or ''), "check_drc is clean on VDD")

        print("4. --tap-via-share 2")
        _o, _log, tv, vias2 = repair(td, 'two', '--tap-via-share', '2')
        ok(vias2 == vias0 and tv and tv['limit'] == 2 and tv['shared_vias'] == 1,
           f"the copper of row 2: {tv}")

    print("5. the flag on both CLIs and in a plan")
    for t in ('route.py', 'repair_planes.py'):
        h = check(PY + [tool(t), '-h'], accept=True)
        ok('--tap-via-share' in (h.stdout or ''), f"{t} takes --tap-via-share")
    import ast
    with open(tool('route.py'), encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    fin = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
           and getattr(n.func, 'id', None) == '_rdp_engine']
    ok(len(fin) >= 2 and all(any(k.arg == 'tap_via_share' for k in n.keywords) for n in fin),
       f"every plane-finalize call of route.py ({len(fin)}) hands the limit to the repair")
    with open(os.path.join(REPO, 'kicad_routing_plugin', 'swig_gui.py'), encoding='utf-8') as fh:
        gui = ast.parse(fh.read())
    ok(any(isinstance(n, ast.Call) and any(k.arg == 'tap_via_share' for k in n.keywords)
           for n in ast.walk(gui)), "and the GUI's route call passes it")
    sys.path.insert(0, os.path.join(TESTS, 'stress'))
    import manifest_to_plan as m2p
    ok(m2p.FLAG_PARAMS.get('--tap-via-share') == 'tap_via_share',
       "manifest_to_plan carries it as tap_via_share")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
