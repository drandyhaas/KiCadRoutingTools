#!/usr/bin/env python3
"""--escalation off routes to the rules KiCad grades the shipped board at.

Under --escalation off the run writes no project, so KiCad grades the shipped
board at the rules the project declares, and at KiCad's factory value for a
rule the project leaves out (or when there is no project). The router must
route to those rules:

1. copper-to-edge: effective_board_edge_clearance takes the larger of the
   run's value and the graded rule (the project's, else the factory 0.5 mm).
   End to end: a net that can squeeze between a pad and the board edge at
   the fab floor (0.2 mm) does so under fab, and goes round under off.
2. via hole to pad copper: config.via_hole_clearance keeps a via's DRILL that
   floor off a pad, even where the pad's clearance override is smaller.
3. route._graded_via_hole_clearance resolves that floor: the declared rule,
   else the factory 0.25 mm when no project is written.
4. close_soft_joints refuses a bridge inside another net's FILLED footprint
   copper (an SOT tab drawn as a polygon), and keeps a bridge of the net the
   tab's own pad gives it, as the obstacle map does.

Controls: under fab (the writeback on) items 1 and 3 do not change.

    python3 tests/test_escalation_off_graded_rules.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

from synth import RoutingBoard, make_pad                        # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def _write(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


def _project(path, rules):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({"board": {"design_settings": {"rules": rules,
                                                 "rule_severities": {}}},
                   "meta": {"filename": os.path.basename(path), "version": 1}},
                  f, indent=2)


def _policy(policy):
    """Install `policy` as set_policy_from_args does: off implies no
    writeback."""
    import fab_tiers
    fab_tiers.set_escalation_policy(
        policy, writeback=fab_tiers.project_writeback_allowed(policy))


# ---------------------------------------------------------------- 1. edge
def t_edge_resolution(tmp):
    print("1a. copper-to-edge resolution")
    from fix_kicad_drc_settings import effective_board_edge_clearance as eff
    bare = _write(os.path.join(tmp, 'bare.kicad_pcb'),
                  RoutingBoard(w=20, h=20).text())
    cases = {}
    for name, rules in (('norule', {"min_clearance": 0.2}),
                        ('rule03', {"min_copper_edge_clearance": 0.3}),
                        ('rule0', {"min_copper_edge_clearance": 0.0})):
        pcb = _write(os.path.join(tmp, f'{name}.kicad_pcb'),
                     RoutingBoard(w=20, h=20).text())
        _project(os.path.splitext(pcb)[0] + '.kicad_pro', rules)
        cases[name] = pcb

    _policy('off')
    got = eff(bare, 0.0)
    check("off, no project: the factory 0.5 mm", abs(got - 0.5) < 1e-9, f'{got}')
    got = eff(cases['norule'], 0.0)
    check("off, project leaves the rule out: the factory 0.5 mm",
          abs(got - 0.5) < 1e-9, f'{got}')
    got = eff(cases['rule03'], 0.0)
    check("off, project declares 0.3: 0.3", abs(got - 0.3) < 1e-9, f'{got}')
    got = eff(cases['rule03'], 0.25)
    check("off, --board-edge-clearance 0.25 under a declared 0.3: 0.3",
          abs(got - 0.3) < 1e-9, f'{got}')
    got = eff(cases['rule03'], 0.4)
    check("off, --board-edge-clearance 0.4 over a declared 0.3: 0.4",
          abs(got - 0.4) < 1e-9, f'{got}')
    got = eff(cases['rule0'], 0.0)
    check("off, project declares 0 (no check): not raised to the factory value",
          got < 0.5 - 1e-9, f'{got}')

    _policy('fab')
    got = eff(bare, 0.0)
    check("control: fab, no project: not raised to 0.5", got < 0.5 - 1e-9, f'{got}')
    got = eff(cases['norule'], 0.0)
    check("control: fab, project leaves the rule out: not raised to 0.5",
          got < 0.5 - 1e-9, f'{got}')


def _min_edge_gap(pcb_path, net):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(pcb_path)
    x0, y0, x1, y1 = 0.0, 0.0, 20.0, 20.0
    segs = [s for s in pcb.segments if pcb.nets[s.net_id].name == net]
    if not segs:
        return None, 0
    gap = min(min(min(s.start_x, s.end_x) - x0, x1 - max(s.start_x, s.end_x),
                  min(s.start_y, s.end_y) - y0, y1 - max(s.start_y, s.end_y))
              - s.width / 2.0 for s in segs)
    return gap, len(segs)


def t_edge_route(tmp):
    """CLK runs from (1, 5) to (1, 15) past a BLK pad whose copper starts at
    x = 0.85. Under fab the edge floor is 0.2 mm and the track fits between
    the edge and the pad; under off KiCad grades the project-less board at
    0.5 mm, which closes that gap, so the track goes round the pad."""
    print("1b. end to end: route.py on a project-less board")
    d = os.path.join(tmp, 'route')
    os.makedirs(d)
    b = RoutingBoard(w=20, h=20, layers=('F.Cu',))
    b.pad('A1', 1.0, 5, 'CLK')
    b.pad('A2', 1.0, 15, 'CLK')
    b.pad('U1', 3.0, 10, 'BLK', sx=4.3, sy=2.0)
    inp = _write(os.path.join(d, 'in.kicad_pcb'), b.text())
    env = dict(os.environ, KICAD_PLANE_FINALIZE='0')
    gaps = {}
    for esc in ('fab', 'off'):
        out = os.path.join(d, f'out_{esc}.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'route.py'), inp, out,
                            '--nets', 'CLK', '--layers', 'F.Cu',
                            '--escalation', esc],
                           capture_output=True, text=True, env=env)
        if r.returncode != 0 or not os.path.isfile(out):
            check(f"{esc}: route.py ran", False,
                  f"exit {r.returncode}\n{(r.stdout or '')[-1500:]}"
                  f"{(r.stderr or '')[-1500:]}")
            return
        gaps[esc], n = _min_edge_gap(out, 'CLK')
        check(f"{esc}: CLK routed", n > 0, f'{n} segment(s)')
        if esc == 'off':
            check("off: no project written",
                  not os.path.isfile(os.path.splitext(out)[0] + '.kicad_pro'))
    if None in gaps.values() or len(gaps) < 2:
        return
    check("control: fab routes inside the factory 0.5 mm band (the gap is "
          "open at the fab floor)", gaps['fab'] < 0.5 - 1e-6, f"{gaps['fab']:.3f} mm")
    check("off: every CLK track keeps 0.5 mm off the edge",
          gaps['off'] >= 0.5 - 1e-3, f"{gaps['off']:.3f} mm")


# ------------------------------------------------------ 2. via hole to pad
def t_via_hole_keepout():
    """Pad 1.0 x 1.0 at (10, 10) with a 0.05 mm clearance override, via
    0.45/0.2 (ring 0.125), clearance 0.1. Without the floor the first legal
    via centre leaves its drill 0.20 mm off the pad; KiCad grades 0.25."""
    print("2. via drill keeps the copper-to-hole floor off a pad")
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import _pad_via_keepout_cells, _via_hole_copper_floor
    pad = make_pad(1, 10.0, 10.0, size_x=1.0, size_y=1.0)
    pad.local_clearance = 0.05
    step = 0.05
    coord = GridCoord(step)
    first_free = {}
    for hc in (0.0, 0.25):
        cfg = GridRouteConfig(clearance=0.1, via_size=0.45, via_drill=0.2,
                              grid_step=step, layers=['F.Cu', 'B.Cu'])
        cfg.via_hole_clearance = hc
        cells = {tuple(int(v) for v in c)
                 for c in _pad_via_keepout_cells(pad, coord, cfg)}
        gy = round(10.0 / step)
        xmax = max(c[0] for c in cells if c[1] == gy)
        # the first via centre right of the pad, on its row, that is legal
        first_free[hc] = (xmax + 1) * step - (10.0 + 0.5)
        if hc:
            check("floor = hole clearance less the annular ring",
                  abs(_via_hole_copper_floor(cfg) - 0.125) < 1e-9,
                  f'{_via_hole_copper_floor(cfg)}')
    drill_r = 0.1
    without = first_free[0.0] - drill_r
    with_ = first_free[0.25] - drill_r
    check("control: without the floor the drill can sit under 0.25 mm",
          without < 0.25 - 1e-6, f'{without:.3f} mm')
    check("with via_hole_clearance 0.25 the drill keeps 0.25 mm off the pad",
          with_ >= 0.25 - 1e-6, f'{with_:.3f} mm')

    cfg = GridRouteConfig(clearance=0.1, via_size=0.6, via_drill=0.3,
                          grid_step=step, layers=['F.Cu', 'B.Cu'])
    cfg.via_hole_clearance = 0.25
    check("a wider ring asks less (0.6/0.3: 0.25 - 0.15 = 0.1, the clearance)",
          abs(_via_hole_copper_floor(cfg) - 0.1) < 1e-9,
          f'{_via_hole_copper_floor(cfg)}')
    cfg.via_hole_clearance = 0.0
    check("unset: no floor", _via_hole_copper_floor(cfg) == 0.0)


def t_graded_via_hole(tmp):
    print("3. route._graded_via_hole_clearance")
    from kicad_parser import parse_kicad_pcb
    from route import _graded_via_hole_clearance
    bare = _write(os.path.join(tmp, 'vh_bare.kicad_pcb'),
                  RoutingBoard(w=20, h=20).text())
    decl = _write(os.path.join(tmp, 'vh_decl.kicad_pcb'),
                  RoutingBoard(w=20, h=20).text())
    _project(os.path.splitext(decl)[0] + '.kicad_pro', {"min_hole_clearance": 0.3})
    pb, pd = parse_kicad_pcb(bare), parse_kicad_pcb(decl)
    _policy('off')
    got = _graded_via_hole_clearance(pb, bare)
    check("off, no project: KiCad's factory 0.25", abs(got - 0.25) < 1e-9, f'{got}')
    got = _graded_via_hole_clearance(pd, decl)
    check("off, project declares 0.3: 0.3", abs(got - 0.3) < 1e-9, f'{got}')
    _policy('fab')
    got = _graded_via_hole_clearance(pb, bare)
    check("control: fab, no project: unset (the writeback records the floor)",
          got == 0.0, f'{got}')


# -------------------------------------------------- 4. filled footprint copper
TAB = '''  (footprint "synth:SOT89" (layer "F.Cu") (uuid "fp-tab") (at 10 10)
    (property "Reference" "Q1" (at 0 -3 0) (layer "F.SilkS") (uuid "ref-q1")
      (effects (font (size 0.5 0.5) (thickness 0.1))))
    (fp_poly (pts (xy -2 -2) (xy 2 -2) (xy 2 2) (xy -2 2))
      (stroke (width 0) (type solid)) (fill solid) (layer "F.Cu") (uuid "tab"))
    (pad "2" smd rect (at 1.6 1.6) (size 1.0 1.0) (layers "F.Cu" "F.Mask")
      (net %d "GND") (uuid "q1-2"))
  )'''


def _soft_joint(b, net, x, y):
    """Two `net` tracks whose free ends overlap 0.1 mm apart at (x, y): a soft
    joint whose bridge does not lie inside either flank."""
    b.seg(x, y - 3.0, x, y, net, w=0.2)
    b.seg(x - 3.0, y + 0.08, x + 0.06, y + 0.08, net, w=0.2)


def t_filled_bridge(tmp):
    """A filled 4 x 4 mm tab (8..12) owned by Q1, whose one pad (1 mm, over
    a corner of the tab, so the tab is its copper) is GND. A soft joint at
    (9, 9) is 3 mm off that pad and 1 mm off the tab's outline, so only the
    fill can refuse its bridge."""
    print("4. close_soft_joints and filled footprint copper")
    from kicad_parser import parse_kicad_pcb
    from pcb_modification import close_soft_joints
    from routing_config import GridRouteConfig
    bridged = {}
    for net in ('SIG', 'GND'):
        b = RoutingBoard(w=20, h=20)
        gnd = b.net('GND')
        b.items.append(TAB % gnd)
        _soft_joint(b, net, 9.0, 9.0)
        pcb = parse_kicad_pcb(_write(os.path.join(tmp, f'fill_{net}.kicad_pcb'),
                                     b.text()))
        nid = next(i for i, n in pcb.nets.items() if n.name == net)
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.05,
                              clearance=0.2, track_width=0.2)
        added = []
        with contextlib.redirect_stdout(io.StringIO()):
            close_soft_joints(added, pcb, {nid}, cfg)
        bridged[net] = [s for r in added for s in (r.get('new_segments') or [])]
    check("a SIG bridge inside the GND-owned fill is refused",
          not bridged['SIG'], f"{len(bridged['SIG'])} bridge(s)")
    check("a GND bridge inside the fill its own GND pad lifts is laid",
          len(bridged['GND']) == 1, f"{len(bridged['GND'])} bridge(s)")


def main():
    import fab_tiers
    prev = fab_tiers.get_escalation_policy()
    try:
        with tempfile.TemporaryDirectory(prefix='esc_off_graded_') as tmp:
            t_edge_resolution(tmp)
            t_edge_route(tmp)
            t_via_hole_keepout()
            t_graded_via_hole(tmp)
            t_filled_bridge(tmp)
    finally:
        fab_tiers.set_escalation_policy(*prev)
    if FAILS:
        print(f"\nFAILED {len(FAILS)}: " + "; ".join(FAILS))
        sys.exit(1)
    print("\nALL PASS")


if __name__ == '__main__':
    main()
