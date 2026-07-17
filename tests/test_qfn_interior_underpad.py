#!/usr/bin/env python3
"""Issue #410: qfn_fanout --escape-method underpad escapes INTERIOR pads.

Board: kicad_files/qfn_interior_pads.kicad_pcb -- an HVQFN-32 whose EP was
shrunk to 1.6x1.6 (GND) with four interior pads added under the body at local
(+-0.45, 1.4) INT1/INT2 and (+-1.35, 1.4) GND guards. All five classify as
side='center' (min bbox-edge distance >= edge_tolerance) and, since #410,
carry a real long-axis escape direction instead of the old null vector.

Scenarios:
  1. Input board is DRC-clean (guard).
  2. analyze_pad unit-asserts: interior pads escape (0,+1) with extents along
     the axis; the EP at the exact package center keeps the positive long-axis
     fallback (1,0) -- dot-product 0 means no flip, deterministically.
  3. UNSCOPED underpad run: the --nets safety guard holds -- no interior or
     GND copper is ever emitted, the EP is not via-dropped.
  4. Scoped STUB run: the surface fan still skips all center pads.
  5. Scoped underpad WITHOUT --allow-via-in-pad: the off-pad offset ladder is
     blocked by the bottom perimeter row, so both INT pads drop cleanly.
  6. Scoped underpad WITH --allow-via-in-pad: both INT pads escape via a via
     centred on the pad (vias-only output, no stub), written board DRC-clean,
     EP and guards untouched.
  7. CLI subprocess run: JSON_SUMMARY grades requested=2 escaped=2 failed=0
     and the output board carries both vias -- regression guard for the two
     main() fixes (vias-only write path + via-only escape counting).

    python3 tests/test_qfn_interior_underpad.py
"""
import io
import json
import os
import re
import subprocess
import sys
import tempfile
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from kicad_writer import add_tracks_and_vias_to_pcb
from qfn_fanout import generate_qfn_fanout
from qfn_fanout.layout import analyze_qfn_layout, analyze_pad
from bga_fanout.constants import POSITION_TOLERANCE

BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
VIA_SIZE, VIA_DRILL, CLEARANCE, GRID = 0.45, 0.2, 0.15, 0.05

CHECKS = []


def check(name, ok):
    CHECKS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")


def _fanout(pcb, nets, via_in_pad):
    fp = pcb.footprints['U1']
    buf = io.StringIO()
    with redirect_stdout(buf):
        tracks, vias, dropped = generate_qfn_fanout(
            fp, pcb, net_filter=nets, escape_method='underpad',
            clearance=CLEARANCE, via_size=VIA_SIZE, via_drill=VIA_DRILL,
            grid_step=GRID, allow_via_in_pad=via_in_pad)
    return tracks, vias, dropped


def _drc_clean(path):
    from check_drc import run_drc
    buf = io.StringIO()
    with redirect_stdout(buf):
        viols = run_drc(path, clearance=CLEARANCE, check_sizes=False)
    return not viols


def main():
    print("=" * 60)
    print("Issue #410: interior-pad underpad escapes")
    print("=" * 60)
    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints['U1']
    layout = analyze_qfn_layout(fp)
    net_name = {p.pad_number: p.net_name for p in fp.pads}
    guarded_ids = {p.net_id for p in fp.pads
                   if p.net_name in ('INT1', 'INT2', 'GND')}

    # 1. Input board clean.
    check("input board DRC-clean", _drc_clean(BOARD))

    # 2. Classification + escape-axis unit asserts.
    pads = {p.pad_number: p for p in fp.pads}
    pi35 = analyze_pad(pads['35'], layout)
    check("pad 35 (INT2) classifies center", pi35.side == 'center')
    check("pad 35 escape axis is (0,+1)",
          abs(pi35.escape_direction[0]) < 1e-9
          and abs(pi35.escape_direction[1] - 1.0) < 1e-9)
    check("pad 35 extents follow the axis (w=0.8 along, l=0.5 across)",
          abs(pi35.pad_width - 0.8) < 1e-6 and abs(pi35.pad_length - 0.5) < 1e-6)
    pi33 = analyze_pad(pads['33'], layout)
    check("EP pad 33 at exact center: deterministic (+1,0) fallback",
          pi33.side == 'center'
          and abs(pi33.escape_direction[0] - 1.0) < 1e-9
          and abs(pi33.escape_direction[1]) < 1e-9)

    # 3. Unscoped underpad: the --nets safety guard holds.
    tracks, vias, dropped = _fanout(pcb, None, True)
    check("unscoped: no interior/GND via emitted",
          not [v for v in vias if v['net_id'] in guarded_ids])
    check("unscoped: no interior/GND track emitted",
          not [t for t in tracks if t.get('net_id') in guarded_ids])
    check("unscoped: no interior/GND net dropped either",
          not set(dropped) & {'INT1', 'INT2', 'GND'})

    # 4. Scoped stub: surface fan still skips center pads.
    fp2 = parse_kicad_pcb(BOARD).footprints['U1']
    buf = io.StringIO()
    with redirect_stdout(buf):
        stub = generate_qfn_fanout(fp2, parse_kicad_pcb(BOARD),
                                   net_filter=['INT*'], escape_method='stub',
                                   clearance=CLEARANCE)
    check("scoped stub run returns empty", stub == ([], [], []))

    # 5. Scoped underpad, no via-in-pad: boxed-in pads drop cleanly.
    tracks, vias, dropped = _fanout(parse_kicad_pcb(BOARD), ['INT*'], False)
    check("no-via-in-pad: both INT pads dropped, no vias",
          sorted(dropped) == ['INT1', 'INT2'] and vias == [] and tracks == [])

    # 6. Scoped underpad + via-in-pad: the headline.
    pcb6 = parse_kicad_pcb(BOARD)
    tracks, vias, dropped = _fanout(pcb6, ['INT*'], True)
    check("via-in-pad: exactly 2 vias, nothing dropped, no tracks",
          len(vias) == 2 and dropped == [] and tracks == [])
    by_net = {v['net_id']: v for v in vias}
    int_pads = {p.net_id: p for p in pcb6.footprints['U1'].pads
                if p.net_name in ('INT1', 'INT2')}
    check("via nets are exactly INT1+INT2",
          set(by_net) == set(int_pads))
    centred = all(abs(by_net[nid]['x'] - p.global_x) <= POSITION_TOLERANCE
                  and abs(by_net[nid]['y'] - p.global_y) <= POSITION_TOLERANCE
                  for nid, p in int_pads.items()) if set(by_net) == set(int_pads) else False
    check("each via centred on its own pad", centred)
    check("vias are unclamped 0.45 through-vias F.Cu->B.Cu",
          all(abs(v['size'] - VIA_SIZE) < 1e-9
              and v['layers'] == ['F.Cu', 'B.Cu'] for v in vias))
    out6 = os.path.join(tempfile.mkdtemp(), 'interior_vip.kicad_pcb')
    net_id_to_name = {nid: n.name for nid, n in pcb6.nets.items()}
    add_tracks_and_vias_to_pcb(BOARD, out6, tracks, vias,
                               net_id_to_name=net_id_to_name)
    check("written board DRC-clean", _drc_clean(out6))
    out_pcb = parse_kicad_pcb(out6)
    gnd_ids = {p.net_id for p in out_pcb.footprints['U1'].pads
               if p.net_name == 'GND'}
    check("EP/guards untouched (no GND via written)",
          not [v for v in out_pcb.vias if v.net_id in gnd_ids])

    # 7. CLI end-to-end (regression guard for the vias-only main() fixes).
    out7 = os.path.join(tempfile.mkdtemp(), 'interior_cli.kicad_pcb')
    r = subprocess.run(
        [sys.executable, 'qfn_fanout.py', BOARD, '--component', 'U1',
         '--nets', 'INT*', '--escape-method', 'underpad', '--allow-via-in-pad',
         '--via-size', str(VIA_SIZE), '--via-drill', str(VIA_DRILL),
         '--clearance', str(CLEARANCE), '--grid-step', str(GRID),
         '-o', out7],
        cwd=ROOT, capture_output=True, text=True)
    out = r.stdout + r.stderr
    m = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    check("CLI run emits JSON_SUMMARY", bool(m))
    summary = json.loads(m[-1]) if m else {}
    check("CLI summary grades requested=2 escaped=2 failed=0",
          summary.get('requested') == 2 and summary.get('escaped') == 2
          and summary.get('failed') == 0)
    cli_pcb = parse_kicad_pcb(out7) if os.path.exists(out7) else None
    cli_vias = [v for v in (cli_pcb.vias if cli_pcb else [])]
    check("CLI output board carries both vias (vias-only write path)",
          len(cli_vias) >= 2)

    failed = [n for n, okk in CHECKS if not okk]
    print("-" * 60)
    print(f"{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
