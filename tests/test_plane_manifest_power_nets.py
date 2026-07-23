#!/usr/bin/env python3
"""Regression test for issue #479 (ch32v203_ev +5V orphaned by the manifest).

route_disconnected_planes.py used to record its --power-nets (a track-WIDTH
hint) into the plane-net manifest sidecar as if those nets had plane copper.
A later `route.py --nets '*'` step then excluded them from wildcard selection
("already plane-handled"), so a real rail could end the chain with zero
copper (ch32v203_ev's 9-pad +5V), or be locked out of every retry step
(steppenprobe +1V8, eth_tap VDDA, openairscope +3V3P_MCU, ...).

The manifest must record only nets whose ZONES the repair step actually
processed -- here GND -- never the power-net width hints.

    python3 tests/test_plane_manifest_power_nets.py
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (net 0 "") (net 1 "GND") (net 2 "FOO")
  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
  (footprint "test:C1" (layer "F.Cu") (at 5 5)
    (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)
      (layers "*.Cu" "*.Mask") (net 1 "GND")))
  (footprint "test:C2" (layer "F.Cu") (at 15 15)
    (property "Reference" "C2" (at 0 0) (layer "F.SilkS"))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)
      (layers "*.Cu" "*.Mask") (net 1 "GND"))
    (pad "2" smd rect (at 0 3) (size 0.8 0.8) (layers "F.Cu") (net 2 "FOO")))
  (zone (net 1) (net_name "GND") (layer "B.Cu") (tstamp 0) (hatch edge 0.5)
    (connect_pads (clearance 0.2))
    (min_thickness 0.25)
    (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.4))
    (polygon (pts (xy 1 1) (xy 19 1) (xy 19 19) (xy 1 19))))
)
"""


def main():
    tmpdir = tempfile.mkdtemp(prefix='manifest_power_nets_')
    board_in = os.path.join(tmpdir, 'in.kicad_pcb')
    board_out = os.path.join(tmpdir, 'out.kicad_pcb')
    with open(board_in, 'w', encoding='utf-8') as f:
        f.write(BOARD)

    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(REPO, 'route_disconnected_planes.py'),
         board_in, board_out,
         '--power-nets', 'FOO', '--power-nets-widths', '1.0',
         '--no-kicad-recheck'],
        capture_output=True, text=True, timeout=300)

    results = []
    results.append(("route_disconnected_planes exits 0", r.returncode == 0))

    sidecar = os.path.join(tmpdir, 'out_planes.json')
    manifest = {}
    if os.path.isfile(sidecar):
        with open(sidecar, encoding='utf-8') as f:
            manifest = json.load(f)
    plane_nets = manifest.get('plane_nets', [])
    results.append(("manifest records the processed zone net",
                    'GND' in plane_nets))
    results.append(("manifest does NOT record --power-nets width hints",
                    'FOO' not in plane_nets))

    if r.returncode != 0 or 'FOO' in plane_nets or 'GND' not in plane_nets:
        print(r.stdout[-2000:])
        print(r.stderr[-2000:])

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} plane-manifest power-nets tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
