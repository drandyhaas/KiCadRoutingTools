#!/usr/bin/env python3
"""Issues #326 + #338: custom clearances honored end-to-end.

#326: KiCad honors per-pad `(clearance ...)` overrides, FOOTPRINT-level
overrides (inherited by pads without their own), and per-netclass clearances;
the router and check_drc modelled one global clearance, so boards graded 0 by
check_drc carried dozens of kicad-cli clearance items (hackrf 52, ghoul 47,
lily58 9, ulx3s 30...). #338: KiCad grades copper-to-Edge.Cuts from the
board's min_copper_edge_clearance; the chain neither routed to it nor graded
at it, and fix_project_for_output clobbered the rule to 0.0.

Covers:
  * parser: footprint-level clearance parse + inheritance into
    pad.local_clearance (pad's own override wins; explicit 0 inherits;
    negatives clamp; custom-pad "(clearance outline)" not confused),
  * check_drc: pad-segment graded at the pad override with required_mm
    attribution; netclass pair clearance via net_clearances,
  * per-net obstacle cache: pad stamped at its override (B6),
  * validator kernels: base_clearance subtracts the pad's excess (B6),
  * config: get_net_clearance (B5) and compute_targets edge-clobber guard +
    effective_board_edge_clearance (#338).

    python3 tests/test_local_clearance_overrides_326.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kicad_parser import parse_kicad_pcb
from check_drc import run_drc

FAILS = []


def check(name, cond):
    if not cond:
        FAILS.append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")


def _write(text):
    f = tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False)
    f.write(text)
    f.close()
    return f.name


def _board(fp_clearance='', pad1_clearance='', seg_y=10.75, seg_net=2,
           extra=''):
    """One footprint at (10,10) with a 0.6mm pad (net /A) and a horizontal
    0.2mm-wide segment of net /B at y=seg_y. Pad edge is at y=10.3; segment
    edge at seg_y-0.1. seg_y=10.75 -> edge gap 0.35mm: clear at 0.1 global,
    inside a 0.4 override."""
    return f'''(kicad_pcb
 (version 20221018)
 (layers
  (0 "F.Cu" signal)
  (2 "B.Cu" signal)
 )
 (net 0 "")
 (net 1 "/A")
 (net 2 "/B")
 (footprint "L:A" (layer "F.Cu") (at 10 10)
   (property "Reference" "U1")
   {fp_clearance}
   (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A") {pad1_clearance})
   {extra}
 )
 (segment (start 5 {seg_y}) (end 15 {seg_y}) (width 0.2) (layer "F.Cu") (net {seg_net}) (uuid "00000000-0000-0000-0000-000000000001"))
)'''


def test_parser_inheritance():
    print("\n-- parser: footprint-level clearance inheritance --")
    path = _write(_board(
        fp_clearance='(clearance 0.4)',
        extra='(pad "2" smd rect (at 2 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "/B") (clearance 0.15))\n'
              '   (pad "3" smd rect (at 4 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "/B") (clearance 0))\n'
              '   (pad "4" smd rect (at 6 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "/B") (clearance -0.05))'))
    try:
        pcb = parse_kicad_pcb(path)
        u1 = pcb.footprints['U1']
        by_num = {p.pad_number: p for p in u1.pads}
        check("footprint.clearance parsed (0.4)", abs(u1.clearance - 0.4) < 1e-9)
        check("pad w/o own override inherits 0.4",
              abs(by_num['1'].local_clearance - 0.4) < 1e-9)
        check("pad's own 0.15 override wins over footprint 0.4",
              abs(by_num['2'].local_clearance - 0.15) < 1e-9)
        check("explicit (clearance 0) inherits the footprint value",
              abs(by_num['3'].local_clearance - 0.4) < 1e-9)
        check("negative pad override clamps to inherit",
              abs(by_num['4'].local_clearance - 0.4) < 1e-9)
    finally:
        os.unlink(path)

    # No footprint-level token: pads keep 0 (and custom-pad option text
    # "(clearance outline)" must not read as an override).
    path = _write(_board(extra='(pad "5" custom (at 8 0) (size 0.3 0.3) (layers "F.Cu") (net 2 "/B")\n'
                               '     (options (clearance outline) (anchor rect)))'))
    try:
        pcb = parse_kicad_pcb(path)
        u1 = pcb.footprints['U1']
        check("no footprint token -> footprint.clearance 0", u1.clearance == 0.0)
        check("custom-pad (clearance outline) not parsed as override",
              all(p.local_clearance == 0.0 for p in u1.pads))
    finally:
        os.unlink(path)


def test_check_drc_pad_override():
    print("\n-- check_drc: pad/footprint override grading --")
    # Edge gap 0.35: clean at global 0.1...
    path = _write(_board())
    try:
        v = run_drc(path, clearance=0.1, quiet=True, print_summary=False)
        check("no override: 0.35mm gap clean at 0.1 global",
              not [x for x in v if x['type'] == 'pad-segment'])
    finally:
        os.unlink(path)
    # ...flagged when the pad carries a 0.5 override...
    path = _write(_board(pad1_clearance='(clearance 0.5)'))
    try:
        v = [x for x in run_drc(path, clearance=0.1, quiet=True,
                                print_summary=False)
             if x['type'] == 'pad-segment']
        check("pad-level 0.5 override flags the 0.35mm gap", len(v) == 1)
        check("violation attributed (required_mm 0.5)",
              v and abs(v[0].get('required_mm', 0) - 0.5) < 1e-9)
    finally:
        os.unlink(path)
    # ...and when the override comes from the FOOTPRINT level.
    path = _write(_board(fp_clearance='(clearance 0.5)'))
    try:
        v = [x for x in run_drc(path, clearance=0.1, quiet=True,
                                print_summary=False)
             if x['type'] == 'pad-segment']
        check("footprint-level 0.5 override flags via inheritance", len(v) == 1)
    finally:
        os.unlink(path)


def test_check_drc_netclass():
    print("\n-- check_drc: per-netclass pair clearance --")
    text = '''(kicad_pcb
 (version 20221018)
 (layers (0 "F.Cu" signal))
 (net 0 "")
 (net 1 "/A")
 (net 2 "/B")
 (segment (start 5 10) (end 15 10) (width 0.2) (layer "F.Cu") (net 1) (uuid "00000000-0000-0000-0000-000000000002"))
 (segment (start 5 10.5) (end 15 10.5) (width 0.2) (layer "F.Cu") (net 2) (uuid "00000000-0000-0000-0000-000000000003"))
)'''
    path = _write(text)
    try:
        v = run_drc(path, clearance=0.1, quiet=True, print_summary=False)
        check("0.3mm seg gap clean at 0.1 global",
              not [x for x in v if x['type'] == 'segment-segment'])
        v = run_drc(path, clearance=0.1, quiet=True, print_summary=False,
                    net_clearances={'/A': 0.4})
        ss = [x for x in v if x['type'] == 'segment-segment']
        check("netclass 0.4 on /A flags the 0.3mm pair gap", len(ss) == 1)
        check("netclass violation attributed (required_mm 0.4)",
              ss and abs(ss[0].get('required_mm', 0) - 0.4) < 1e-9)
    finally:
        os.unlink(path)


def test_obstacle_cache_override():
    print("\n-- per-net obstacle cache honors pad override (B6) --")
    from synth import make_pad, make_net, make_pcb
    from routing_config import GridRouteConfig
    from obstacle_cache import precompute_net_obstacles

    def cache_cells(lc):
        pad = make_pad(net_id=1, x=10.0, y=10.0, size_x=0.6, size_y=0.6,
                       net_name='/A', local_clearance=lc)
        pcb = make_pcb(nets={1: make_net(1, '/A')},
                       pads_by_net={1: [pad]})
        cfg = GridRouteConfig(track_width=0.1, clearance=0.1, grid_step=0.05,
                              layers=['F.Cu', 'B.Cu'])
        data = precompute_net_obstacles(pcb, 1, cfg)
        return len(data.blocked_cells)

    plain = cache_cells(0.0)
    wide = cache_cells(0.5)
    check(f"0.5 override widens the cached pad halo ({plain} -> {wide} cells)",
          wide > plain)


def test_validator_kernel_base_clearance():
    print("\n-- validator kernels: base_clearance subtracts override excess --")
    from synth import make_pad, make_net, make_pcb
    from single_ended_routing import _pt_foreign_pad_dist
    pad = make_pad(net_id=2, x=10.0, y=10.0, size_x=0.6, size_y=0.6,
                   net_name='/B', local_clearance=0.5)
    pcb = make_pcb(nets={2: make_net(2, '/B')}, pads_by_net={2: [pad]})
    # Query point 0.5mm from the pad edge (pad edge x=10.3 -> x=10.8).
    d_raw = _pt_foreign_pad_dist(pcb, 1, 10.8, 10.0, 'F.Cu')
    d_eff = _pt_foreign_pad_dist(pcb, 1, 10.8, 10.0, 'F.Cu', base_clearance=0.1)
    check("raw distance unchanged without base_clearance",
          abs(d_raw - 0.5) < 1e-6)
    check("effective distance drops by the 0.4 excess (0.5 - 0.1)",
          abs(d_eff - 0.1) < 1e-6)


def test_edge_clearance_338():
    print("\n-- #338: edge rule reading + clobber guard --")
    from fix_kicad_drc_settings import (compute_targets,
                                        effective_board_edge_clearance)
    t = compute_targets(clearance=0.1, edge_clearance=0.0)
    check("compute_targets skips edge 0.0 (no clobber to zero)",
          'min_copper_edge_clearance' not in t)
    t = compute_targets(clearance=0.1, edge_clearance=0.3)
    check("compute_targets keeps a real edge value",
          t.get('min_copper_edge_clearance') == 0.3)

    d = tempfile.mkdtemp()
    pcb_path = os.path.join(d, 'b.kicad_pcb')
    pro_path = os.path.join(d, 'b.kicad_pro')
    open(pcb_path, 'w').write('(kicad_pcb)')
    import json
    json.dump({'board': {'design_settings': {'rules':
               {'min_copper_edge_clearance': 0.5}}}}, open(pro_path, 'w'))
    try:
        check("effective edge = board rule when flag unset",
              abs(effective_board_edge_clearance(pcb_path, 0.0) - 0.5) < 1e-9)
        check("explicit larger flag wins",
              abs(effective_board_edge_clearance(pcb_path, 0.7) - 0.7) < 1e-9)
        check("smaller flag loses to the board rule",
              abs(effective_board_edge_clearance(pcb_path, 0.2) - 0.5) < 1e-9)
    finally:
        import shutil
        shutil.rmtree(d, ignore_errors=True)


def main():
    print("=" * 60)
    print("Issues #326/#338 custom-clearance regression test")
    print("=" * 60)
    test_parser_inheritance()
    test_check_drc_pad_override()
    test_check_drc_netclass()
    test_obstacle_cache_override()
    test_validator_kernel_base_clearance()
    test_edge_clearance_338()
    print("=" * 60)
    if FAILS:
        print(f"\n{len(FAILS)} failure(s): {FAILS}")
        return 1
    print("  ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
