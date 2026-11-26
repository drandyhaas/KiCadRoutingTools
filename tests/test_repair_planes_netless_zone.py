"""repair_planes zone auto-detect skips a pour that has no net.

KiCad writes no ``(net ...)`` for a copper zone with no net. Zone auto-detect
listed it as net '' and the standalone run stopped at once with
"Error: Net '' not found in PCB". lvds_converter_dualclk_gnd
carries such a pour on B.Cu next to its /GND plane, so it shows the bug as it
is: the pour must be skipped and /GND still found.
"""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'py_router'))

from repair_planes import auto_detect_zones  # noqa: E402

BOARD = ROOT / 'kicad_files' / 'lvds_converter_dualclk_gnd.kicad_pcb'


def test_netless_zone_is_skipped():
    from plane_io import extract_zones
    # the fixture still carries the net-less pour this test is about
    assert any(not z.net_name for z in extract_zones(str(BOARD)))
    assert auto_detect_zones(str(BOARD)) == [('/GND', 'B.Cu')]


def test_cli_runs_past_zone_auto_detect():
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'in.kicad_pcb')
        subprocess.run([sys.executable, str(ROOT / 'py_router' / 'copy_board.py'),
                        str(BOARD), board], check=True, capture_output=True)
        r = subprocess.run(
            [sys.executable, str(ROOT / 'py_router' / 'repair_planes.py'),
             board, os.path.join(td, 'out.kicad_pcb')],
            capture_output=True, text=True, timeout=600)
        out = r.stdout + r.stderr
        assert "Net '' not found" not in out, out[-2000:]
        assert "Found 1 zone(s) to process:" in out, out[-2000:]
        assert r.returncode == 0, out[-2000:]


if __name__ == '__main__':
    test_netless_zone_is_skipped()
    test_cli_runs_past_zone_auto_detect()
    print("2 passed")
