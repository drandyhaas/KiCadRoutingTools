"""#1602: the nested sub-runs carry the class/type-scoped .kicad_dru rules.

A `.kicad_dru` rule scoped by net name / item type (`A.NetName == 'HV' &&
A.Type == 'Pad'`, 0.5 mm) closes the only path of net SIG: a gap of 0.9 mm
between two HV pads, which a 0.25 mm track fits at the 0.25 mm class value
but not at the rule's. route.py's main pass held the rule (the scoped
channel, #1491) and failed SIG; the final reconciliation then routed SIG
straight through the gap and check_drc flagged both pads. The sub-run is
handed the parent's resolved track map, and `install_track_clearances`'
explicit-map path switched the scoped channel OFF -- the same for the in-run
plane finalize (repair_planes is handed the map too) on both fronts.

The scoped channel is now installed on both paths, and a parent's resolved
rules ride into every nested sub-run (`scoped_clearance_rules`, re-bound by
net NAME when a re-parse numbers the nets differently).

    python3 tests/test_1602_scoped_rules_nested_subruns.py
"""
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-X', 'utf8', '-B']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


DRU = """(version 1)
(rule "HV pads"
  (constraint clearance (min 0.5mm))
  (condition "A.NetName == 'HV' && A.Type == 'Pad'"))
"""


def _fp(ref, x, y, pads, n):
    body = ''.join(
        f'\t\t(pad "{num}" smd rect (at {px} {py}) (size {sx} {sy}) '
        f'(layers "F.Cu" "F.Mask" "F.Paste") (net "{net}") '
        f'(uuid "00000000-0000-0000-0000-0000000{n}{i:04d}"))\n'
        for i, (num, px, py, sx, sy, net) in enumerate(pads))
    return (f'\t(footprint "" (layer "F.Cu") '
            f'(uuid "00000000-0000-0000-0000-00000000000{n}") (at {x} {y})\n'
            f'\t\t(property "Reference" "{ref}" (at 0 0 0) (layer "F.SilkS") '
            f'(uuid "00000000-0000-0000-0000-0000000000a{n}") '
            f'(effects (font (size 1.27 1.27))))\n'
            f'{body}\t)\n')


def board_text():
    """The issue's board: SIG from J1 to J2 has one path, between U1's two HV
    pads (0.9 mm apart: room for a 0.25 track at 0.25, none at 0.5)."""
    edge = ''.join(
        f'\t(gr_line (start {a} {b}) (end {c} {d}) (stroke (width 0.05) '
        f'(type default)) (layer "Edge.Cuts") '
        f'(uuid "00000000-0000-0000-0000-0000000000e{i}"))\n'
        for i, (a, b, c, d) in enumerate([(100, 100, 120, 100), (120, 100, 120, 110),
                                          (120, 110, 100, 110), (100, 110, 100, 100)]))
    return ('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
            '\t(generator_version "10.0")\n\t(general (thickness 1.6))\n'
            '\t(paper "A4")\n\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
            '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n\t\t(13 "F.Paste" user)\n'
            '\t\t(5 "F.SilkS" user "F.Silkscreen")\n\t\t(25 "Edge.Cuts" user)\n\t)\n'
            '\t(setup (pad_to_mask_clearance 0))\n'
            + _fp('J1', 103, 105, [('1', 0, 0, 1, 1, 'SIG')], 1)
            + _fp('U1', 110, 105, [('1', 0, -2.725, 1, 4.55, 'HV'),
                                   ('2', 0, 2.725, 1, 4.55, 'HV')], 2)
            + _fp('J2', 117, 105, [('1', 0, 0, 1, 1, 'SIG')], 3)
            + edge + ')\n')


def _zone(n, x0, x1):
    return (f'\t(zone (net "GND") (layer "B.Cu") '
            f'(uuid "00000000-0000-0000-0000-0000000000f{n}") (hatch edge 0.5)\n'
            f'\t\t(connect_pads (clearance 0.25)) (min_thickness 0.2) '
            f'(filled_areas_thickness no)\n'
            f'\t\t(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
            f'\t\t(polygon (pts (xy {x0} 100) (xy {x1} 100) (xy {x1} 110) '
            f'(xy {x0} 110)))\n\t)\n')


def zone_board_text():
    """The same board plus two B.Cu GND pours, one GND pad in each: two
    islands for the plane repair engine to join."""
    t = board_text()
    gnd = (_fp('H1', 102, 102, [], 9).replace(
        '\t)\n',
        '\t\t(pad "1" thru_hole circle (at 0 0) (size 1.2 1.2) (drill 0.6) '
        '(layers "*.Cu" "*.Mask") (net "GND") '
        '(uuid "00000000-0000-0000-0000-000000090001"))\n'
        '\t\t(pad "2" thru_hole circle (at 16 6) (size 1.2 1.2) (drill 0.6) '
        '(layers "*.Cu" "*.Mask") (net "GND") '
        '(uuid "00000000-0000-0000-0000-000000090002"))\n\t)\n')
        + _zone(1, 100, 108) + _zone(2, 112, 120))
    i = t.rfind(')')
    return t[:i] + gnd + t[i:]


def write_board(td, name='in', text=None):
    b = os.path.join(td, f'{name}.kicad_pcb')
    with open(b, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text or board_text())
    with open(os.path.join(td, f'{name}.kicad_dru'), 'w', encoding='utf-8',
              newline='\n') as f:
        f.write(DRU)
    run_utils.evidence(b, 'test board')
    return b


def drc_clean(board):
    """check_drc's verdict on `board`: (clean, output). A violation exits 1,
    which is an answer here, not an accident -- but a traceback is not."""
    run_utils.evidence(board, 'board to grade')
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), board],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=300)
    out = (r.stdout or '') + (r.stderr or '')
    if 'Traceback' in out or r.returncode not in (0, 1):
        raise AssertionError(f'BROKEN TEST: check_drc died (exit {r.returncode}): '
                             f'{out[-1200:]}')
    return r.returncode == 0 and 'NO DRC VIOLATIONS' in out, out


def control(td):
    print('0. control: the board is clean, and the gap admits SIG at the class value')
    b = write_board(td, 'ctl')
    ok, out = drc_clean(b)
    check('the input is DRC-clean', ok, out[-400:])
    # Without the rule the router does route SIG through the gap -- so the
    # rule, and nothing else, is what keeps it out below.
    os.unlink(os.path.join(td, 'ctl.kicad_dru'))
    out = os.path.join(td, 'ctl_out.kicad_pcb')
    run_utils.check(PY + [os.path.join(REPO, 'py_router', 'route.py'), b, out,
                          '--nets', 'SIG', '--layers', 'F.Cu'], accept=True)
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(out)
    sig = next(i for i, n in pcb.nets.items() if n.name == 'SIG')
    check('without the rule SIG routes through the gap',
          any(s.net_id == sig for s in pcb.segments))


def cli(td):
    print('1. CLI: the reconciliation lap keeps SIG out of the ruled gap')
    b = write_board(td)
    out = os.path.join(td, 'out.kicad_pcb')
    r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'route.py'), b, out,
                              '--nets', 'SIG', '--layers', 'F.Cu'], accept=True)
    log = r.stdout + r.stderr
    # The leg under test must actually run, or this checks nothing.
    check('the final reconciliation retried SIG',
          'Final reconciliation: retrying 1' in log and 'SIG' in log, log[-600:])
    from kicad_parser import parse_kicad_pcb
    run_utils.evidence(out, 'routed board')
    pcb = parse_kicad_pcb(out)
    sig = next(i for i, n in pcb.nets.items() if n.name == 'SIG')
    check('SIG ships no copper (its only path breaks the rule)',
          not any(s.net_id == sig for s in pcb.segments), 'SIG copper shipped')
    ok, txt = drc_clean(out)
    check('check_drc grades the output clean', ok, txt[-600:])


def gui(td):
    print('2. GUI engine leg (return_results, in-memory reconciliation)')
    b = write_board(td, 'gui')
    from kicad_parser import parse_kicad_pcb
    from route import batch_route
    pcb = parse_kicad_pcb(b)
    sig = next(i for i, n in pcb.nets.items() if n.name == 'SIG')
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        _ok, _fail, _t, data = batch_route(b, "", ['SIG'], layers=['F.Cu'],
                                           return_results=True, pcb_data=pcb)
    log = buf.getvalue()
    check('the in-memory reconciliation retried SIG',
          'Final reconciliation: retrying 1' in log, log[-600:])
    segs = [s for r in data.get('results', []) for s in (r.get('new_segments') or [])
            if s.net_id == sig]
    check('no SIG copper comes back to the GUI applier', not segs,
          f'{len(segs)} segment(s)')


def forwarding(td):
    print('3. the channel rides an explicit track map, and re-binds by name')
    from kicad_parser import parse_kicad_pcb
    from kicad_dru import (ScopedClearanceRules, install_track_clearances,
                           board_has_scoped_clearance_rules)
    from routing_config import GridRouteConfig
    b = write_board(td, 'fw')
    pcb = parse_kicad_pcb(b)
    sig = next(i for i, n in pcb.nets.items() if n.name == 'SIG')
    cfg = GridRouteConfig()
    install_track_clearances(cfg, {}, b, pcb, routed_net_ids=[sig])
    check('an explicit track map no longer switches the scoped channel off',
          cfg.scoped_clearance is not None)
    sc = ScopedClearanceRules.for_board(pcb, b)
    # A sub-run on a board whose .kicad_dru is not beside it (the output a
    # parent is still writing): the parent's rules win.
    bare = os.path.join(td, 'bare.kicad_pcb')
    shutil.copyfile(b, bare)
    pcb2 = parse_kicad_pcb(bare)
    cfg2 = GridRouteConfig()
    install_track_clearances(cfg2, {}, bare, pcb2, routed_net_ids=[sig],
                             scoped_rules=sc)
    check("a parent's rules are installed where no .kicad_dru is beside the board",
          cfg2.scoped_clearance is not None
          and cfg2.scoped_clearance.sc.names == ['HV pads'])
    hv = next(i for i, n in pcb2.nets.items() if n.name == 'HV')
    v = cfg2.scoped_clearance.rule_value(hv, 'smd')
    check('...and price an HV pad at the rule', v is not None and abs(v - 0.5) < 1e-9,
          f'{v}')
    # A re-parse that numbers the nets differently: rebuilt by name.
    pcb3 = parse_kicad_pcb(b)
    remap = {sig: 99}
    for old, new in remap.items():
        n = pcb3.nets.pop(old)
        n.net_id = new
        pcb3.nets[new] = n
    re = sc.rebind(pcb3)
    hv3 = next(i for i, n in pcb3.nets.items() if n.name == 'HV')
    check('rebind on renumbered nets rebuilds by name', re is not sc
          and re.pair_value(99, 'track', hv3, 'pad') is not None
          and re.pair_value(99, 'track', hv3, 'pad')[0] == 0.5)
    check('rebind on the same numbering is the same rules', sc.rebind(pcb) is sc)
    check('the oracle sees a scoped-only .kicad_dru',
          board_has_scoped_clearance_rules(b)
          and not board_has_scoped_clearance_rules(bare))


def plane_repair(td):
    print('4. the plane repair engine (the in-run finalize) takes the rules it is handed')
    import kicad_dru
    from kicad_parser import parse_kicad_pcb
    from repair_planes import repair_planes
    b = write_board(td, 'zn', zone_board_text())
    sc = kicad_dru.ScopedClearanceRules.for_board(parse_kicad_pcb(b), b)
    # The finalize repairs the file the run is still writing: no .kicad_dru
    # of its own beside it, an explicit track map, and the parent's rules.
    bare = os.path.join(td, 'zn_out.kicad_pcb')
    shutil.copyfile(b, bare)
    seen = []
    orig = kicad_dru.install_track_clearances

    def spy(config, *a, **kw):
        r = orig(config, *a, **kw)
        seen.append(config.scoped_clearance)
        return r
    kicad_dru.install_track_clearances = spy
    try:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            repair_planes(bare, '', ['GND'], ['B.Cu'], track_clearances={},
                          scoped_clearance_rules=sc, pcb_data=parse_kicad_pcb(bare),
                          return_results=True)
    finally:
        kicad_dru.install_track_clearances = orig
    check('the repair engine installed the scoped channel',
          bool(seen) and all(s is not None and s.sc.names == ['HV pads'] for s in seen),
          f'{seen}')


def gnd_vias(td):
    print('5. route_planes --add-gnd-vias prices its return vias with the scoped rules')
    import add_gnd_vias
    import route_planes
    b = write_board(td, 'gv', zone_board_text())
    out = os.path.join(td, 'gv_out.kicad_pcb')
    seen = []
    orig = add_gnd_vias.add_gnd_vias_to_existing_board

    def spy(pcb_data, net, dist, config, *a, **kw):
        seen.append(config.scoped_clearance)
        return orig(pcb_data, net, dist, config, *a, **kw)
    add_gnd_vias.add_gnd_vias_to_existing_board = spy
    argv = sys.argv
    sys.argv = ['route_planes.py', b, out, '--nets', 'GND', '--plane-layers', 'B.Cu',
                '--add-gnd-vias', '--gnd-via-net', 'GND']
    try:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            try:
                route_planes.main()
            except SystemExit as e:
                if e.code not in (0, None):
                    raise AssertionError(f'BROKEN TEST: route_planes exited {e.code}: '
                                         f'{buf.getvalue()[-1200:]}')
    finally:
        sys.argv = argv
        add_gnd_vias.add_gnd_vias_to_existing_board = orig
    check('the GND-via config carries the scoped channel',
          len(seen) == 1 and seen[0] is not None and seen[0].sc.names == ['HV pads'],
          f'{seen}')


def main():
    td = tempfile.mkdtemp(prefix='t1602_')
    try:
        control(td)
        cli(td)
        gui(td)
        forwarding(td)
        plane_repair(td)
        gnd_vias(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f"\nFAILED: {len(failures)} check(s): {failures}")
        sys.exit(1)
    print("\nPASS: test_1602_scoped_rules_nested_subruns")


if __name__ == '__main__':
    main()
