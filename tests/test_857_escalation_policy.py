#!/usr/bin/env python3
"""#857 / #842: the escalation policy bounds every descent, and every descent
is disclosed where a harness can read it.

Three arms on the splitflap fixture with a 5.0 mm track request (which cannot
leave the pads, so the primary route fails and the rescue ladder fires):

  default (board, no project)  -> delivered at the fab floor 0.127, COUNTED in
                                  JSON_SUMMARY design_rules and JSON_SUMMARY_MIN
                                  escalations, and named in the end-of-run line
  board + min_track_width 0.3  -> delivered no narrower than the board's own 0.3
  off                          -> nothing narrowed (count 0); the net fails and
                                  says so instead of shipping thinner copper
  --strict-sizes               -> exit 3 when anything was delivered below request
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
NET = '/OUT_A_PHASE_A'


def _run(td, extra, project=None):
    staged = os.path.join(td, 'in.kicad_pcb')
    shutil.copyfile(BOARD, staged)
    pro = os.path.join(td, 'in.kicad_pro')
    if project is not None:
        json.dump(project, open(pro, 'w'))
    elif os.path.exists(pro):
        os.remove(pro)
    js = os.path.join(td, 'out.json')
    out = os.path.join(td, 'out.kicad_pcb')
    for p in (out, os.path.splitext(out)[0] + '.kicad_pro'):
        if os.path.exists(p):
            os.remove(p)
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'route.py'), staged, out,
                        '--nets', NET, '--track-width', '5.0', '--json-out', js] + extra,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    data = json.load(open(js, encoding='utf-8')) if os.path.exists(js) else {}
    return r, data


def main():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return 0
    fails = []
    with tempfile.TemporaryDirectory() as td:
        # --- default: fab policy on the auto tier (completion first, Andy
        # 2026-09-03), project-less board -> fab floor, DISCLOSED
        r, data = _run(td, [])
        if r.returncode != 0:
            print(r.stdout[-3000:])
            return 1
        dr = data.get('design_rules') or {}
        rescue = data.get('rescue') or {}
        if not rescue.get('attempted'):
            print("  SKIP: the fixture routed 5.0mm without the rescue; policy untestable here")
            return 0
        w = next(iter((rescue.get('widths') or {}).values()), {})
        if dr.get('escalation_policy') != 'fab' or dr.get('fab_tier') != 'auto':
            fails.append(f"default policy/tier reported as {dr.get('escalation_policy')}/{dr.get('fab_tier')}, expected fab/auto")
        if not dr.get('count'):
            fails.append(f"a thinner delivery ({w}) was not COUNTED in design_rules: {dr}")
        if not any(row.get('kind') == 'track_width' for row in dr.get('narrowed', [])):
            fails.append("the width narrowing is not in design_rules.narrowed")
        m = re.search(r'JSON_SUMMARY_MIN: (\{.*\})', r.stdout)
        if not m or not json.loads(m.group(1)).get('escalations'):
            fails.append("JSON_SUMMARY_MIN carries no escalations count")
        if 'Design rules [--escalation fab, --fab-tier auto]' not in r.stdout:
            fails.append("no end-of-run design-rules line was printed for the default policy")

        # --- the HARD tier + board policy, opted into: same disclosure, and no
        # fab-tier escalation may fire
        r, data = _run(td, ['--escalation', 'board', '--fab-tier', 'standard'])
        if r.returncode != 0:
            print(r.stdout[-3000:])
            return 1
        dr = data.get('design_rules') or {}
        if dr.get('escalation_policy') != 'board' or dr.get('fab_tier') != 'standard':
            fails.append(f"explicit policy/tier reported as {dr.get('escalation_policy')}/{dr.get('fab_tier')}")
        if 'Design rules [--escalation board, --fab-tier standard]' not in r.stdout:
            fails.append("no end-of-run design-rules line was printed for the explicit hard tier")
        if dr.get('fab_tier_escalations'):
            fails.append("a fab-tier escalation fired under the hard standard tier")

        # --- board policy with the board declaring min_track_width 0.3
        proj = {'board': {'design_settings': {'rules': {'min_track_width': 0.3,
                                                        'min_clearance': 0.1}}},
                'net_settings': {'classes': [{'name': 'Default', 'clearance': 0.2,
                                              'track_width': 0.3, 'via_diameter': 0.5,
                                              'via_drill': 0.3,
                                              'priority': 2147483647}],
                                 'meta': {'version': 0}}, 'meta': {'version': 1}}
        r, data = _run(td, ['--escalation', 'board'], project=proj)
        if r.returncode != 0:
            print(r.stdout[-3000:])
            return 1
        dr = data.get('design_rules') or {}
        if abs((dr.get('board_floors') or {}).get('track_width', 0) - 0.3) > 1e-9:
            fails.append(f"board floors not read from the project: {dr.get('board_floors')}")
        below = [row for row in dr.get('narrowed', [])
                 if row.get('kind') == 'track_width' and row.get('delivered', 1) < 0.3 - 1e-9]
        if below:
            fails.append(f"board policy delivered below the board's own 0.3 minimum: {below}")
        widths = (data.get('rescue') or {}).get('widths') or {}
        for net, ww in widths.items():
            if ww.get('delivered_mm', 1) < 0.3 - 1e-9:
                fails.append(f"rescue delivered {net} at {ww.get('delivered_mm')} under a 0.3 board minimum")

        # --- off: nothing narrowed; the net fails honestly
        r, data = _run(td, ['--escalation', 'off'])
        if r.returncode != 0:
            print(r.stdout[-3000:])
            return 1
        dr = data.get('design_rules') or {}
        if dr.get('count'):
            fails.append(f"--escalation off still narrowed something: {dr.get('narrowed')}")
        if not (data.get('failed') or data.get('failed_single') or data.get('open_single')):
            fails.append("--escalation off completed a 5.0mm net that cannot leave its pads")

        # --- strict sizes: the default arm again, now a non-zero exit
        r, data = _run(td, ['--strict-sizes'])
        if r.returncode != 3:
            fails.append(f"--strict-sizes exited {r.returncode}, expected 3 after a narrowed delivery")
        if '--strict-sizes:' not in r.stdout:
            fails.append("--strict-sizes gave no reason on stdout")

    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: the default (fab policy, auto tier) narrows to the fab floor and discloses it; "
          "the opted-in board policy on the hard standard tier narrows only to the board's declared floor and "
          "discloses it; off narrows nothing and fails honestly; --strict-sizes exits 3")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
