#!/usr/bin/env python3
"""#1667: a GND return via is placed only where it joins two layers of its net.

A via joining fewer than two layers connects nothing, and the closing route.py
removes it (#1166). `add_gnd_vias_to_existing_board` used to place them anyway:
on a net poured on one layer every via it added went again, so the step cost a
re-pour and a route and shipped nothing. It now judges a site by the sweep's
own test, on the zones as the board will hold them, on both fronts:

  A. a net poured on one layer, or on none: nothing placed, a warning that says
     why, also collected for a front-end that does not show the console; the
     same when the net's own track runs beside the site, because the obstacle
     map stamps the net's own copper, so a site never touches it;
  B. two layers: the pour may be an earlier step's, this call's or both
     (the CLI reads them from the written file, the GUI hands over `poured_now`,
     where `replace_existing` takes the old pour's place);
  C. two layers that do not both cover a site: that site is passed over for one
     that lies in both, and a signal via with no such site gets no via and says
     so;
  D. every via it places survives the sweep (`sweep_dangling_via_branches`), and
     the negative control: the same via on a one-layer net does not;
  E. route_planes.py on a board file carries all of it: one layer warns and
     writes none, an earlier pour plus this call's writes one;
  F. a signal via already covered is not a reason to warn;
  G. the skills: one rule for the tail, stated once, and the plan's line to the
     user follows it.

    python3 -X utf8 tests/test_1667_return_via_two_layers.py
"""
import contextlib
import io
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import BoardInfo, Net, PCBData, Pad, Segment, Via, Zone, parse_kicad_pcb  # noqa: E402
from routing_config import GridCoord, GridRouteConfig  # noqa: E402
from obstacle_map import build_base_obstacle_map  # noqa: E402  (puts rust_router on sys.path)
from add_gnd_vias import add_gnd_vias_to_existing_board  # noqa: E402
from pcb_modification import sweep_dangling_via_branches  # noqa: E402
from check_weird import via_support_parts  # noqa: E402
from run_utils import evidence  # noqa: E402

SIG, GND = 1, 2
WHOLE = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
LAYERS = ['F.Cu', 'B.Cu']
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def zone(layer, polygon=WHOLE):
    return Zone(net_id=GND, net_name='GND', layer=layer, polygon=list(polygon))


def board(zones=(), segments=(), sig_at=(10.0, 10.0)):
    """A signal via, and a far GND through-hole pad that leaves the net whole
    (the sweep leaves an unfinished net alone) and outside the coverage radius."""
    pad = Pad(pad_number='1', net_id=GND, net_name='GND', global_x=2.0, global_y=2.0,
              size_x=1.6, size_y=1.6, shape='circle', layers=['*.Cu'], drill=0.8,
              component_ref='J1', local_x=0.0, local_y=0.0)
    return PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=list(LAYERS),
                             board_bounds=(0.0, 0.0, 20.0, 20.0)),
        nets={0: Net(net_id=0, name='', pads=[]), SIG: Net(net_id=SIG, name='SIG', pads=[]),
              GND: Net(net_id=GND, name='GND', pads=[pad])},
        footprints={}, segments=list(segments),
        vias=[Via(x=sig_at[0], y=sig_at[1], size=0.6, drill=0.3, layers=list(LAYERS), net_id=SIG)],
        pads_by_net={0: [], SIG: [], GND: [pad]}, zones=list(zones))


def place(pcb, **kw):
    """(vias, what the engine printed), through the real obstacle map."""
    cfg = GridRouteConfig(via_size=0.6, via_drill=0.3, track_width=0.2, clearance=0.2,
                          grid_step=0.1, layers=list(LAYERS), hole_to_hole_clearance=0.25)
    obstacles = build_base_obstacle_map(pcb, cfg, [])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        vias = add_gnd_vias_to_existing_board(pcb, 'GND', 3.0, cfg, obstacles,
                                              GridCoord(cfg.grid_step), **kw)
    return vias, buf.getvalue()


def support(pcb, v):
    span, fixed, by_seg = via_support_parts(
        v, [s for s in pcb.segments if s.net_id == v.net_id],
        pcb.pads_by_net.get(v.net_id, []),
        [z for z in pcb.zones if z.net_id == v.net_id], list(LAYERS))
    return set(fixed) | set(by_seg.values())


def too_few_layers():
    print('A. fewer than two layers')
    for name, zones, what in (('one layer', [zone('B.Cu')], 'poured only on B.Cu'),
                              ('no pour', [], 'not poured on any layer')):
        notes = []
        vias, log = place(board(zones), warnings=notes)
        check(f'{name}: no via', vias == [], f'{len(vias)}')
        check(f'{name}: the warning says why', what in log and 'second layer' in log
              and 'route.py removes' in log, log.strip().splitlines()[-1][:120])
        check(f'{name}: the warning is collected once', len(notes) == 1 and what in notes[0], f'{notes}')
    # The net's own track beside the site on the other layer: a via touching it
    # would join two layers, and the sweep would keep it, but the map stamps the
    # net's own copper, so no site reaches it. Nothing else could be placed.
    track = Segment(start_x=9.0, start_y=10.5, end_x=11.0, end_y=10.5, width=0.2,
                    layer='F.Cu', net_id=GND)
    vias, _log = place(board([zone('B.Cu')], [track]))
    check('one layer plus a same-net track: no via', vias == [], f'{len(vias)}')
    two = board([zone('F.Cu'), zone('B.Cu')], [track])
    vias, _log = place(two)
    check('the map stamps the net\'s own track: a via on two pours stays off it',
          bool(vias) and not any(
              ((v.x - 10.0) ** 2 + (v.y - 10.5) ** 2) ** 0.5 < 0.3 + 0.1 for v in vias),
          f'{[(v.x, v.y) for v in vias]}')


def two_layers():
    print('B. two layers, wherever the pours came from')
    vias, _log = place(board([zone('F.Cu'), zone('B.Cu')]))
    check('both already on the board', len(vias) == 1, f'{len(vias)}')
    rec = dict(net_id=GND, net_name='GND', layer='B.Cu', polygon_points=list(WHOLE))
    vias, _log = place(board([zone('F.Cu')]), poured_now=[rec])
    check('one on the board, one in this call', len(vias) == 1, f'{len(vias)}')
    vias, _log = place(board(), poured_now=[rec, dict(rec, layer='F.Cu')])
    check('both in this call', len(vias) == 1, f'{len(vias)}')
    vias, log = place(board([zone('B.Cu')]), poured_now=[dict(rec, replace_existing=True)])
    check('a pour this call replaces is one layer, not two', vias == [] and 'poured only on B.Cu' in log)
    far = [(15.0, 15.0), (20.0, 15.0), (20.0, 20.0), (15.0, 20.0)]
    vias, _log = place(board([zone('B.Cu', far), zone('F.Cu')]),
                       poured_now=[dict(rec, replace_existing=True)])
    check('the replacing pour is the outline that counts', len(vias) == 1, f'{len(vias)}')
    vias, log = place(board([zone('B.Cu')]),
                      poured_now=[dict(net_name='', keepout=True, layers=['F.Cu'],
                                       polygon_points=list(WHOLE))])
    check('a keep-out record is not a pour', vias == [] and 'poured only on B.Cu' in log)


def partial_coverage():
    print('C. two layers that do not both cover the site')
    # F.Cu's pour stops at x=9.5: the search's first sites, to the right of the
    # signal via at x=10, lie in B.Cu's pour only.
    left = [(0.0, 0.0), (9.5, 0.0), (9.5, 20.0), (0.0, 20.0)]
    pcb = board([zone('F.Cu', left), zone('B.Cu')])
    vias, _log = place(pcb)
    check('a via is placed', len(vias) == 1, f'{len(vias)}')
    check('on a site inside both pours', all(support(pcb, v) == set(LAYERS) for v in vias),
          f'{[(v.x, v.y) for v in vias]}')
    far = [(0.0, 0.0), (1.0, 0.0), (1.0, 20.0), (0.0, 20.0)]
    vias, log = place(board([zone('F.Cu', far), zone('B.Cu')]))
    check('no such site in reach: no via, and it says so',
          vias == [] and 'no clear site' in log and "pours on two layers" in log,
          log.strip().splitlines()[-1][:100])


def survives_the_sweep():
    print('D. what the sweep does with them')
    pcb = board([zone('F.Cu'), zone('B.Cu')])
    vias, _log = place(pcb)
    pcb.vias.extend(vias)
    _segs, gone = sweep_dangling_via_branches(pcb, {GND})
    check('a placed via survives', len(vias) == 1 and not gone, f'{len(vias)} placed, {len(gone)} removed')
    # Negative control: the same via with one layer poured, as the base code placed it.
    pcb = board([zone('B.Cu')])
    pcb.vias.extend(vias)
    _segs, gone = sweep_dangling_via_branches(pcb, {GND})
    check('control: on a one-layer net the sweep removes it', len(gone) == len(vias) == 1,
          f'{len(gone)} removed')


BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "GND")
  (net 2 "SIG")
  (footprint "t:TH" (layer "F.Cu") (at 18 18)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)
      (layers "*.Cu") (net 1 "GND")))
  (via (at 10 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2))
  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default))
    (fill none) (layer "Edge.Cuts"))
)
"""


def _planes(d, src, tag, nets, layers):
    out = os.path.join(d, f'{tag}.kicad_pcb')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route_planes.py'), src, out,
         '--nets', *nets, '--plane-layers', *layers, '--via-size', '0.6', '--via-drill', '0.3',
         '--track-width', '0.2', '--clearance', '0.2', '--grid-step', '0.1',
         '--add-gnd-vias', '--gnd-via-distance', '3.0'],
        capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=ROOT)
    if r.returncode != 0:
        raise AssertionError(f'BROKEN TEST: route_planes exited {r.returncode}: '
                             f'{(r.stdout + r.stderr)[-1500:]}')
    pcb = parse_kicad_pcb(evidence(out))
    gnd = {nid for nid, n in pcb.nets.items() if n.name == 'GND'}
    return [v for v in pcb.vias if v.net_id in gnd], r.stdout, out


def cli():
    print('E. route_planes.py')
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, 'b.kicad_pcb')
        with open(src, 'w', encoding='utf-8') as f:
            f.write(BOARD)
        vias, log, first = _planes(d, src, 'one', ['GND'], ['B.Cu'])
        check('one layer: no via written', vias == [], f'{len(vias)}')
        check('one layer: the log says why', 'Warning: No return vias on GND' in log
              and 'poured only on B.Cu' in log)
        vias, log, _out = _planes(d, first, 'second', ['GND'], ['F.Cu'])
        check("an earlier step's pour plus this call's: one via written", len(vias) == 1,
              f'{len(vias)}; {log[-200:] if not vias else ""}')
        vias, log, _out = _planes(d, src, 'two', ['GND', 'GND'], ['F.Cu', 'B.Cu'])
        check('both in this call: one via written', len(vias) == 1, f'{len(vias)}')


def already_covered():
    print('F. nothing to do is not a warning')
    near = Via(x=10.5, y=11.0, size=0.6, drill=0.3, layers=list(LAYERS), net_id=GND)
    pcb = board([zone('B.Cu')])
    pcb.vias.append(near)
    notes = []
    vias, log = place(pcb, warnings=notes)
    check('a signal via already covered: no via and no warning',
          vias == [] and not notes and 'Warning' not in log, log.strip().splitlines()[-1][:100])


def _skill(name):
    with open(evidence(os.path.join(ROOT, '.claude', 'skills', name, 'SKILL.md')), encoding='utf-8') as f:
        return f.read()


def skills():
    print('G. the skills')
    plan = _skill('plan-pcb-routing')
    for gone in ('cheap and harmless', 'Skip this step entirely',
                 'drop `--add-gnd-vias`', 'The step is\n  included'):
        check(f'plan-pcb-routing no longer says "{gone}"', gone not in plan)
    rule = re.search(r'\n10\. \*\*Stitch/GND-via tail.*?(?=\n\S|\Z)', plan, re.S)
    check('rule 10 is there', rule is not None)
    text = re.sub(r'\s+', ' ', rule.group(0)) if rule else ''
    check('rule 10 names both tiers below it', 'Low-speed and medium-tier boards end at the Step 2 route' in text, text[:200])
    check('rule 10 says the medium row grades', 'grades' in text and 'does not schedule' in text, text[:300])
    step3 = plan.split('### Step 3 of the example chain', 1)[1].split('\n### ', 1)[0]
    flat = re.sub(r'\s+', ' ', step3)
    check('Step 3 cites rule 10', 'Step 10 rule 10' in flat)
    check('Step 3 says what a return via needs', 'needs GND on a second layer' in flat)
    check('Step 3\'s example asks for GND on two layers',
          '--plane-layers <the Step 1 layers, GND on two or more>' in flat)
    tell = plan.split('Report to user when presenting the plan, by', 1)[1].split('`/find-high-speed-nets` ALSO', 1)[0]
    flat = re.sub(r'\s+', ' ', tell)
    check('the line to the user cites rule 10', 'Step 10 rule 10' in flat)
    check('"included" is said only with GND on two layers', 'GND poured on two or more layers' in flat)
    check('the medium-tier line says the plan has none', 'no return vias' in flat)
    review = re.sub(r'\s+', ' ', _skill('review-routed-board'))
    check('review-routed-board: the medium row is a grade', 'grades coverage' in review)
    high = _skill('find-high-speed-nets')
    row = next((ln for ln in high.splitlines() if ln.startswith('| Medium |')), '')
    check('find-high-speed-nets: the medium row is a grade', 'grade' in row and 'no via step' in row, row[:120])


if __name__ == '__main__':
    too_few_layers()
    two_layers()
    partial_coverage()
    survives_the_sweep()
    cli()
    already_covered()
    skills()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print(f'  - {f}')
        sys.exit(1)
    print('ALL PASS')
