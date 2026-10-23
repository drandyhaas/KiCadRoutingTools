#!/usr/bin/env python3
"""#1328: qfn_fanout's omitted --clearance is the board's Default net class, and
a pad that cannot escape at it escalates on its own.

qfn_fanout defaulted --clearance to routing_defaults.QFN_CLEARANCE (0.1)
whatever the board declared, and a fanout's writeback clamps every net class
to the clearance it escaped at. So the chain's own `--clearance-ceiling 0.15`
on steppenprobe (Default and Power classes 0.15) printed "no cap applied --
--clearance 0.1 is already at or below it", wrote Default 0.15 -> 0.1 and
Power 0.15 -> 0.1, and the next route.py routed the whole board at 0.1 --
while the same fanout escaped 31/31 pads at 0.15.

Checks:
  1. resolve_qfn_clearance (the one resolver the CLI and the GUI QFN panel
     share): an explicit value wins; omitted -> the Default class; no class
     (None, or a 0 the project leaves unset) -> QFN_CLEARANCE; a ceiling caps,
     tighten-only.
  2. qfn_clearance_ladder: nothing under --escalation off; under fab, rungs
     strictly below the run clearance, descending, ending at the tier floor;
     under board, never below the board's declared floor.
  3. glasgow_revC U1 (Default class 0.2), `--clearance-ceiling 0.2` and no
     --clearance: the run escapes at the class, escalates nothing, and the
     output project's Default class is still 0.2 (main wrote 0.1).
  4. The same board with its Default class raised to 0.35, no flags: the
     straight escapes cannot clear at 0.35, so they escalate (default policy)
     instead of being dropped -- every pad escapes, each escalated stub is a
     `clearance` row (site 'qfn escape') in design_rules and an entry in
     JSON_SUMMARY.escalated, and the project floor written back is the
     lowest rung a stub actually cleared at, not the fab floor, recorded as a
     Default class lowered by a descent (#1160).
  5. The same board under --escalation off: nothing escalates, the pads are
     dropped, and the Default class is left at 0.35.

    python3 tests/test_1328_qfn_default_clearance.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import fab_tiers                                                   # noqa: E402
import routing_defaults as defaults                                # noqa: E402
from copy_board import copy_board                                  # noqa: E402
from qfn_fanout import resolve_qfn_clearance, qfn_clearance_ladder  # noqa: E402

GLASGOW = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def fan(inp, out, *extra):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py',
                        inp, '--component', 'U1', '--output', out] + list(extra),
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
    log = r.stdout + r.stderr
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', log, re.M)
    return r.returncode, log, (json.loads(m.group(1)) if m else None)


def project(board):
    with open(os.path.splitext(board)[0] + '.kicad_pro', encoding='utf-8') as f:
        return json.load(f)


def default_class(proj):
    return fab_tiers.project_default_class_clearance(proj)


def stage(tmp, name, default_clearance=None):
    dst = os.path.join(tmp, name)
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(GLASGOW, dst)
    if default_clearance is not None:
        pro = os.path.splitext(dst)[0] + '.kicad_pro'
        proj = project(dst)
        for c in proj['net_settings']['classes']:
            if c.get('name') == 'Default':
                c['clearance'] = default_clearance
        with open(pro, 'w', encoding='utf-8') as f:
            json.dump(proj, f, indent=2)
    return dst


def main():
    print("1. resolve_qfn_clearance")
    v, _ = resolve_qfn_clearance(0.12, 0.2)
    check("an explicit --clearance wins over the class", abs(v - 0.12) < 1e-12, f"{v}")
    v, lines = resolve_qfn_clearance(None, 0.15)
    check("omitted -> the board's Default class", abs(v - 0.15) < 1e-12, f"{v}")
    check("...and says so", any('Default net-class clearance 0.15' in l for l in lines),
          f"{lines}")
    for dc in (None, 0.0):
        v, _ = resolve_qfn_clearance(None, dc)
        check(f"no class ({dc!r}) -> QFN_CLEARANCE", abs(v - defaults.QFN_CLEARANCE) < 1e-12,
              f"{v}")
    v, _ = resolve_qfn_clearance(None, 0.2, ceiling=0.15)
    check("a ceiling below caps it", abs(v - 0.15) < 1e-12, f"{v}")
    v, _ = resolve_qfn_clearance(None, 0.15, ceiling=0.2)
    check("a ceiling above never raises it", abs(v - 0.15) < 1e-12, f"{v}")

    print("2. qfn_clearance_ladder")
    fab_tiers.set_default_fab_tier('auto')
    fab_tiers.set_escalation_policy('off', {})
    check("--escalation off: no rungs", qfn_clearance_ladder(0.2, 2) == [])
    fab_tiers.set_escalation_policy('fab', {})
    lad = qfn_clearance_ladder(0.2, 2)
    floor = min(r['clearance'] for r in fab_tiers.escalation_rungs(2))
    check("fab: strictly below the run clearance, descending",
          bool(lad) and lad[0] < 0.2 and lad == sorted(lad, reverse=True), f"{lad}")
    check("...ending at the tier floor", bool(lad) and abs(lad[-1] - floor) < 1e-9,
          f"{lad[-1] if lad else None} vs {floor}")
    check("...with the tier's own rungs on it",
          all(any(abs(x - r['clearance']) < 1e-9 for x in lad)
              for r in fab_tiers.escalation_rungs(2) if r['clearance'] < 0.2))
    fab_tiers.set_escalation_policy('board', {'clearance': 0.15})
    lad = qfn_clearance_ladder(0.2, 2)
    check("board: never below the board's floor",
          bool(lad) and min(lad) >= 0.15 - 1e-9, f"{lad}")
    check("a run already at the floor has no rungs", qfn_clearance_ladder(0.15, 2) == [])
    fab_tiers.set_escalation_policy('fab', {})

    if not os.path.isfile(GLASGOW):
        print("  SKIP rows 3-5: kicad_files/glasgow_revC.kicad_pcb missing")
    else:
        tmp = tempfile.mkdtemp(prefix='t1328_')
        try:
            print("3. --clearance-ceiling alone keeps the board's class")
            g = stage(tmp, 'g20.kicad_pcb')
            out = os.path.join(tmp, 'g20_fan.kicad_pcb')
            rc, log, js = fan(g, out, '--clearance-ceiling', '0.2')
            check("ran", rc == 0 and js is not None, f"rc={rc}")
            js = js or {}
            check("escaped at the Default class 0.2, not 0.1",
                  abs(js.get('clearance', 0) - 0.2) < 1e-9, f"{js.get('clearance')}")
            check("every pad escaped, nothing escalated",
                  js.get('failed') == 0 and js.get('escalated') == [],
                  f"failed={js.get('failed')} escalated={js.get('escalated')}")
            dc = default_class(project(out)) if os.path.isfile(out) else None
            check("the output project's Default class is still 0.2",
                  dc is not None and abs(dc - 0.2) < 1e-9, f"{dc}")

            print("4. a class the part cannot escape at: escalation")
            g = stage(tmp, 'g35.kicad_pcb', default_clearance=0.35)
            out = os.path.join(tmp, 'g35_fab.kicad_pcb')
            rc, log, js = fan(g, out)
            check("ran", rc == 0 and js is not None, f"rc={rc}")
            js = js or {}
            esc = js.get('escalated') or []
            check("escaped at the class 0.35", abs(js.get('clearance', 0) - 0.35) < 1e-9,
                  f"{js.get('clearance')}")
            check("stubs escalated instead of being dropped",
                  bool(esc) and js.get('failed') == 0,
                  f"escalated={len(esc)} failed={js.get('failed')}")
            rungs = [e['clearance'] for e in esc]
            lo = min(rungs) if rungs else None
            fab_floor = min(r['clearance'] for r in fab_tiers.escalation_rungs(4))
            check("every rung is below 0.35 and above the fab floor",
                  bool(rungs) and all(fab_floor - 1e-9 <= r < 0.35 for r in rungs),
                  f"{sorted(set(rungs))} floor {fab_floor}")
            rows = [r for r in (js.get('design_rules') or {}).get('narrowed', [])
                    if r.get('kind') == 'clearance' and r.get('site') == 'qfn escape']
            check("one design_rules clearance row per escalated stub",
                  len(rows) == len(esc) and all(abs(r['requested'] - 0.35) < 1e-9 for r in rows),
                  f"{len(rows)} rows for {len(esc)} stubs")
            check("the end-of-run Design rules line names it",
                  'Design rules [' in log and 'smallest clearance' in log)
            check("min_clearance_used is the lowest rung used",
                  lo is not None and abs(js.get('min_clearance_used', 0) - lo) < 1e-9,
                  f"{js.get('min_clearance_used')} vs {lo}")
            proj = project(out) if os.path.isfile(out) else {}
            dc = default_class(proj)
            check("the Default class written back is that rung, not the fab floor",
                  dc is not None and lo is not None and abs(dc - lo) < 1e-9
                  and dc > fab_floor + 1e-9, f"{dc} (lowest rung {lo}, fab floor {fab_floor})")
            rec = (proj.get('kicad_routing_tools') or {}).get('class_clearance_relaxed') or {}
            check("...and recorded as a class lowered by a descent (#1160)",
                  abs((rec.get('from') or 0) - 0.35) < 1e-9
                  and abs((rec.get('to') or 0) - (lo or -1)) < 1e-9, f"{rec}")

            print("5. --escalation off")
            out = os.path.join(tmp, 'g35_off.kicad_pcb')
            rc, log, js = fan(g, out, '--escalation', 'off')
            js = js or {}
            check("nothing escalates", js.get('escalated') == [], f"{js.get('escalated')}")
            check("...so the pads that cannot clear 0.35 are dropped",
                  (js.get('failed') or 0) > 0, f"failed={js.get('failed')}")
            dc = default_class(project(out)) if os.path.isfile(out) else None
            check("the Default class is left at 0.35", dc is not None and abs(dc - 0.35) < 1e-9,
                  f"{dc}")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {failures}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
