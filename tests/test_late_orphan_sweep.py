#!/usr/bin/env python3
"""
Gate for the #659 debris fixes: BARE-VIA orphan islands, and the LATE sweep
that runs after the finalize/reconciliation.

Two independent defects, gated separately because they fail independently:

(b) `remove_orphan_islands` built its components from `pcb_data.segments`
    only, so a component consisting of VIAS AND NOTHING ELSE was never even
    visited. A failed reroute strips a net's tracks and leaves the barrels --
    measured on spartan6_4layer step 6, /GPIOS/GPIO-P27 went from 28 segments
    + 6 vias to 0 segments + 6 bare vias, and NO pass in the codebase could
    see any of them. KiCad's DRC reports the net unconnected forever while our
    pads-only grading calls it connected.

(a) `run_post_route_cleanup` is the only whole-scope cleanup and it runs long
    before the plane finalize and the final reconciliation, both of which lay
    copper WITH RIP AUTHORITY. Their debris therefore lands after the last
    pass that could remove it. Measured on daisho: /ddr2/DM6 kept a 7.35 mm
    pad-less fragment -- born at step 1, a BGA fanout, which runs no cleanup
    pipeline at all -- through eleven chain steps, and step 9's log printed no
    "Orphan islands" line at all.

The (a) gate discriminates against the main cleanup by planting the debris on
a net OUTSIDE the route's --nets scope: the main pipeline is scoped to the
routed nets and leaves it, the late sweep (whole-board minus unfinished nets,
matching the oracle debris pass's own policy) removes it. The negative control
-- KICAD_LATE_ORPHAN_SWEEP=0 -- must leave it on the board, or the test is
passing for some other reason.

Run:
    python3 tests/test_late_orphan_sweep.py
"""

import os
import shutil
import subprocess
import sys
import tempfile

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS_DIR)
for _p in (_ROOT, os.path.join(_ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb
from check_connected import check_net_connectivity
from pcb_modification import remove_orphan_islands
from synth import make_pad, make_seg, make_via, make_pcb, make_net

BOARD = os.path.join(_ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
# Far from every pad (the board is 5,5..31,25 and all pads sit at x 10..26,
# y 13..17.6), so no route can incidentally adopt it.
ORPHAN_VIA_XY = (7.5, 22.5)

FAILURES = []


def check(cond, label):
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        FAILURES.append(label)


# --------------------------------------------------------------- (b) unit ---

def _net_with_bare_via_debris():
    """A net whose pads are joined by a track, plus a via touching nothing."""
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    stray = make_via(5.0, 8.0, net_id=1)          # nowhere near the track
    pcb = make_pcb(segments=list(route), vias=[stray],
                   pads_by_net={1: pads}, nets={1: make_net(1, 'SIG')})
    return pcb, route, stray, pads


def test_bare_via_island_removed():
    pcb, route, stray, pads = _net_with_bare_via_debris()
    before = check_net_connectivity(1, list(pcb.segments), list(pcb.vias),
                                    pads, [], pcb_data=pcb)
    n, nsegs, strip, vstrip, nvias = remove_orphan_islands([], pcb, None)
    check(n == 1, f"bare-via island is found (islands={n}, expected 1)")
    check(nvias == 1, f"the via is counted as removed (vias_removed={nvias})")
    check(nsegs == 0, "no segment is removed with it (there is none)")
    check([id(v) for v in vstrip] == [id(stray)],
          "the stray via is returned for the writer's strip list")
    check([id(v) for v in pcb.vias] == [],
          "the stray via is gone from pcb_data")
    check([id(s) for s in pcb.segments] == [id(route[0])],
          "the real route is untouched")
    after = check_net_connectivity(1, list(pcb.segments), list(pcb.vias),
                                   pads, [], pcb_data=pcb)
    check(before['connected'] and after['connected'],
          "pads stay connected across the removal")


def test_via_carrying_the_route_is_kept():
    """Negative control for (b): a via that actually joins the net's copper
    is NOT debris. Without this, 'removes bare vias' could mean 'removes
    vias'."""
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1', layers=('F.Cu',)),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1', layers=('B.Cu',))]
    segs = [make_seg(0.0, 0.0, 5.0, 0.0, net_id=1, layer='F.Cu'),
            make_seg(5.0, 0.0, 10.0, 0.0, net_id=1, layer='B.Cu')]
    joiner = make_via(5.0, 0.0, net_id=1)
    pcb = make_pcb(segments=list(segs), vias=[joiner],
                   pads_by_net={1: pads}, nets={1: make_net(1, 'SIG')})
    n, _nsegs, _strip, vstrip, nvias = remove_orphan_islands([], pcb, None)
    check(n == 0 and nvias == 0,
          f"a layer-changing via is kept (islands={n}, vias_removed={nvias})")
    check(len(pcb.vias) == 1, "the joining via is still on the board")


def test_zone_anchored_via_is_kept():
    """Negative control for (b): a via whose only tie is the net's POUR
    reaches the net through the fill and must not be swept."""
    from kicad_parser import Zone
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    stitch = make_via(5.0, 8.0, net_id=1)
    try:
        zone = Zone(net_id=1, net_name='SIG', layer='F.Cu',
                    polygon=[(-2.0, -2.0), (14.0, -2.0), (14.0, 12.0),
                             (-2.0, 12.0)])
    except TypeError:
        print("  SKIP  zone-anchored control (Zone signature differs here)")
        return
    pcb = make_pcb(segments=list(route), vias=[stitch],
                   pads_by_net={1: pads}, nets={1: make_net(1, 'SIG')})
    pcb.zones = [zone]
    n, _ns, _st, _vs, nvias = remove_orphan_islands([], pcb, None)
    check(n == 0 and nvias == 0,
          f"a via inside the net's own pour is kept (islands={n}, "
          f"vias_removed={nvias})")


# -------------------------------------------------------- (a) integration ---

def _plant_orphan_via(src, dst, net_name):
    """Copy `src` to `dst` and add a bare via on `net_name` at ORPHAN_VIA_XY."""
    content = open(src, encoding='utf-8').read()
    from kicad_parser import is_kicad_10
    net_tok = f'(net "{net_name}")' if is_kicad_10(content) else None
    if net_tok is None:
        pcb = parse_kicad_pcb(src)
        nid = next(i for i, n in pcb.nets.items() if n.name == net_name)
        net_tok = f'(net {nid})'
    via = (f'\t(via\n\t\t(at {ORPHAN_VIA_XY[0]} {ORPHAN_VIA_XY[1]})\n'
           f'\t\t(size 0.6)\n\t\t(drill 0.3)\n'
           f'\t\t(layers "F.Cu" "B.Cu")\n\t\t{net_tok}\n'
           f'\t\t(uuid "beefbeef-0000-4000-8000-00000000dead")\n\t)\n')
    idx = content.rfind(')')
    open(dst, 'w', encoding='utf-8').write(content[:idx] + via + content[idx:])
    for ext in ('.kicad_pro', '.kicad_prl'):
        sib = os.path.splitext(src)[0] + ext
        if os.path.exists(sib):
            shutil.copy2(sib, os.path.splitext(dst)[0] + ext)


def _has_orphan_via(board):
    pcb = parse_kicad_pcb(board)
    return any(abs(v.x - ORPHAN_VIA_XY[0]) < 1e-6
               and abs(v.y - ORPHAN_VIA_XY[1]) < 1e-6 for v in pcb.vias)


def _route(inp, out, sweep_on):
    env = dict(os.environ)
    env['KICAD_LATE_ORPHAN_SWEEP'] = '1' if sweep_on else '0'
    cmd = [sys.executable, '-X', 'utf8',
           os.path.join(_ROOT, 'py_router', 'route.py'), inp, out,
           # DPA_P only: the planted via sits on DPB_P, OUTSIDE this scope,
           # so the main (scope-limited) cleanup cannot be what removes it.
           '--nets', 'DPA_P',
           '--layers', 'F.Cu', 'B.Cu', '--track-width', '0.2',
           '--clearance', '0.2', '--via-size', '0.6', '--via-drill', '0.3']
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600,
                       env=env, cwd=_ROOT)
    return r


def test_late_sweep_removes_out_of_scope_debris():
    if not os.path.exists(BOARD):
        print(f"  SKIP  integration: {BOARD} missing")
        return
    with tempfile.TemporaryDirectory() as td:
        planted = os.path.join(td, 'planted.kicad_pcb')
        _plant_orphan_via(BOARD, planted, 'DPB_P')
        if not _has_orphan_via(planted):
            check(False, "BROKEN TEST: the planted orphan via did not parse "
                         "back out of the fixture")
            return
        check(True, "fixture: a bare orphan via is planted on DPB_P")

        out_on = os.path.join(td, 'sweep_on.kicad_pcb')
        r_on = _route(planted, out_on, sweep_on=True)
        if not os.path.exists(out_on):
            check(False, f"BROKEN TEST: route.py wrote no output "
                         f"(rc={r_on.returncode}): {r_on.stdout[-600:]}")
            return
        check('Late orphan sweep (#659)' in r_on.stdout,
              "the late sweep reports having run")
        check(not _has_orphan_via(out_on),
              "sweep ON: the out-of-scope orphan via is gone")

        # NEGATIVE CONTROL: with the sweep disabled the via must SURVIVE.
        # If it vanishes here too, something else is removing it and the
        # assertion above proves nothing about this pass.
        out_off = os.path.join(td, 'sweep_off.kicad_pcb')
        r_off = _route(planted, out_off, sweep_on=False)
        if not os.path.exists(out_off):
            check(False, f"BROKEN TEST: control run wrote no output "
                         f"(rc={r_off.returncode}): {r_off.stdout[-600:]}")
            return
        check(_has_orphan_via(out_off),
              "sweep OFF (control): the orphan via survives, so the sweep "
              "is what removed it")

        # The routed net must be unharmed in both arms.
        for tag, path in (('on', out_on), ('off', out_off)):
            pcb = parse_kicad_pcb(path)
            nid = next((i for i, n in pcb.nets.items() if n.name == 'DPA_P'),
                       None)
            if nid is None:
                check(False, f"sweep {tag}: DPA_P missing from the output")
                continue
            res = check_net_connectivity(
                nid, [s for s in pcb.segments if s.net_id == nid],
                [v for v in pcb.vias if v.net_id == nid],
                pcb.pads_by_net.get(nid, []), [], pcb_data=pcb)
            check(res['connected'], f"sweep {tag}: DPA_P still fully connected")


def _route_all(inp, out):
    """Route every net, so a SECOND run over the result takes the
    'already fully connected' path."""
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(_ROOT, 'py_router', 'route.py'), inp, out,
         '--nets', '*', '--layers', 'F.Cu', 'B.Cu', '--track-width', '0.2',
         '--clearance', '0.2', '--via-size', '0.6', '--via-drill', '0.3'],
        capture_output=True, text=True, timeout=600, cwd=_ROOT)


def test_sweep_runs_on_the_nothing_to_route_early_exit():
    """The path that shipped the debris in the first place.

    `batch_route` returns EARLY when every net in scope is already connected
    -- it writes an unchanged pass-through copy and skips the rest of the
    function. The #659 fragment gate diverts a pad-less-only net to the late
    sweep instead of the router, so a step whose WHOLE scope is diverted lands
    on exactly that early return: measured on spartan6_4layer, three diverted
    nets took the run from 448 s to 6 s and shipped every one of their bare
    vias, while the gate's own message promised the sweep would remove them.

    A gate whose fixture routes something can never see this -- that is why
    this case exists separately from the one above.
    """
    if not os.path.exists(BOARD):
        print(f"  SKIP  integration: {BOARD} missing")
        return
    with tempfile.TemporaryDirectory() as td:
        routed = os.path.join(td, 'routed.kicad_pcb')
        r0 = _route_all(BOARD, routed)
        if not os.path.exists(routed):
            check(False, f"BROKEN TEST: could not route the fixture "
                         f"(rc={r0.returncode}): {r0.stdout[-500:]}")
            return
        planted = os.path.join(td, 'planted.kicad_pcb')
        _plant_orphan_via(routed, planted, 'DPB_P')
        if not _has_orphan_via(planted):
            check(False, "BROKEN TEST: planted orphan via did not parse back")
            return

        out_on = os.path.join(td, 'early_on.kicad_pcb')
        r_on = _route(planted, out_on, sweep_on=True)
        if not os.path.exists(out_on):
            check(False, f"BROKEN TEST: no output (rc={r_on.returncode}): "
                         f"{r_on.stdout[-500:]}")
            return
        # The test is only meaningful if the run really took the early exit.
        check('All nets are already fully connected' in r_on.stdout,
              "the run took the 'nothing to route' early exit (if this fails "
              "the case below proves nothing about that path)")
        check('Late orphan sweep (#659)' in r_on.stdout,
              "the late sweep runs on the early-exit path too")
        check(not _has_orphan_via(out_on),
              "early exit: the orphan via is removed, not shipped")

        out_off = os.path.join(td, 'early_off.kicad_pcb')
        r_off = _route(planted, out_off, sweep_on=False)
        if os.path.exists(out_off):
            check(_has_orphan_via(out_off),
                  "early exit, sweep OFF (control): the via survives")
        else:
            check(False, f"BROKEN TEST: control produced no output "
                         f"(rc={r_off.returncode})")


def test_via_on_same_net_graphic_is_kept():
    """The regression the sets1-5 corpus caught, in miniature.

    Net-tagged copper GRAPHICS do not conduct in our connectivity graph
    (#513), so a via whose only same-net neighbour is art looks pad-less --
    but KiCad DOES credit the graphic, so that barrel is the bridge between
    the net's copper and its art. remove_orphan_islands has guarded segment
    islands against this since #337 via `_touches_graphic`, but that test
    iterates the cluster's SEGMENTS, which is vacuous for a via-only cluster
    -- exactly the cluster kind #659 taught it to see.

    Measured on openstint /A- (stress set 4): the shipped board went from 0
    kicad-cli unconnected items to 2, both with a `graphic` endpoint.
    """
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    art = make_seg(4.0, 8.0, 6.0, 8.0, net_id=1)
    art.graphic = True
    on_art = make_via(5.0, 8.0, net_id=1)
    pcb = make_pcb(segments=[make_seg(0.0, 0.0, 10.0, 0.0, net_id=1), art],
                   vias=[on_art], pads_by_net={1: pads},
                   nets={1: make_net(1, 'SIG')})
    n, _ns, _st, _vs, nv = remove_orphan_islands([], pcb, None)
    check(n == 0 and nv == 0,
          f"a via sitting ON same-net art is KEPT (islands={n}, "
          f"vias_removed={nv})")

    # NEGATIVE CONTROL: the guard must not become "never sweep a via on a
    # board that happens to contain art".
    far = make_via(50.0, 50.0, net_id=1)
    pcb2 = make_pcb(segments=[make_seg(0.0, 0.0, 10.0, 0.0, net_id=1), art],
                    vias=[far], pads_by_net={1: pads},
                    nets={1: make_net(1, 'SIG')})
    n2, _ns2, _st2, _vs2, nv2 = remove_orphan_islands([], pcb2, None)
    check(n2 == 1 and nv2 == 1,
          f"a via far from the art is still swept (islands={n2}, "
          f"vias_removed={nv2}) -- the guard is adjacency, not blanket amnesty")

    # net_dead_copper must agree; the divert consults it, so a disagreement
    # would route one decision off each rule.
    from check_connected import net_dead_copper
    ds, dv = net_dead_copper(pcb, 1, list(pcb.segments), list(pcb.vias),
                             pads, [])
    check(not ds and not dv,
          f"net_dead_copper agrees the art-bridging via is not dead "
          f"({len(ds)} seg, {len(dv)} via)")
    # FRESH board: remove_orphan_islands mutates pcb_data in place, so
    # reusing pcb2 here would ask net_dead_copper about a via that has
    # already been deleted -- and get a vacuous "nothing dead".
    far2 = make_via(50.0, 50.0, net_id=1)
    art2 = make_seg(4.0, 8.0, 6.0, 8.0, net_id=1)
    art2.graphic = True
    pcb3 = make_pcb(segments=[make_seg(0.0, 0.0, 10.0, 0.0, net_id=1), art2],
                    vias=[far2], pads_by_net={1: pads},
                    nets={1: make_net(1, 'SIG')})
    ds2, dv2 = net_dead_copper(pcb3, 1, list(pcb3.segments), list(pcb3.vias),
                               pads, [])
    check(len(dv2) == 1,
          f"...and still calls the far via dead ({len(dv2)} via)")


def main():
    print(__doc__.strip().splitlines()[0])
    for fn in (test_bare_via_island_removed,
               test_via_carrying_the_route_is_kept,
               test_zone_anchored_via_is_kept,
               test_late_sweep_removes_out_of_scope_debris,
               test_via_on_same_net_graphic_is_kept,
               test_sweep_runs_on_the_nothing_to_route_early_exit):
        print(f"\n{fn.__name__}:")
        fn()
    print()
    if FAILURES:
        print(f"FAILED ({len(FAILURES)}): " + "; ".join(FAILURES))
        return 1
    print("Late orphan sweep + bare-via island invariants hold.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
