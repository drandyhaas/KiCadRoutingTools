#!/usr/bin/env python3
"""The rescue must report the width it delivered, not just that it 'recovered'.

The defect this guards (test-board run 5, journal [6]): --track-width is a
request. When the primary route fails, the per-net rescue re-routes at the fab
floor and reports the net recovered with `failed_single` EMPTY -- a 0.8mm USB
request silently shipped as 0.15mm copper, visible only by measuring the board.
The rescue summary now carries widths = {net: {requested_mm, delivered_mm}} in
the merged JSON, so the delta is machine-readable at the source.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def test_rescue_reports_requested_vs_delivered():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        staged = os.path.join(td, 'in.kicad_pcb')
        shutil.copyfile(BOARD, staged)
        js = os.path.join(td, 'out.json')
        # A 5.0mm track cannot leave this board's pads: the primary fails and
        # the rescue delivers at a thinner rung.
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'route.py'), staged,
                            os.path.join(td, 'out.kicad_pcb'),
                            '--nets', '/OUT_A_PHASE_A',
                            '--track-width', '5.0', '--json-out', js],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=ROOT)
        assert r.returncode == 0, r.stdout[-2500:]
        data = json.load(open(js, encoding='utf-8'))
        rescue = data.get('rescue') or {}
        if not rescue.get('attempted'):
            print("  SKIP: board routed 5.0mm without needing the rescue "
                  "(fixture geometry changed); provenance untestable here")
            return
        widths = rescue.get('widths') or {}
        assert widths, f"rescue ran but reported no widths map: {rescue}"
        net, w = next(iter(widths.items()))
        assert w['requested_mm'] > w['delivered_mm'], \
            f"expected a thinner delivery to be recorded: {net} {w}"
        print(f"  PASS: {net} requested {w['requested_mm']} delivered "
              f"{w['delivered_mm']}")


if __name__ == '__main__':
    test_rescue_reports_requested_vs_delivered()
