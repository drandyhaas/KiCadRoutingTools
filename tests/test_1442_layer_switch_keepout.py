#!/usr/bin/env python3
"""A layer switch keeps out of a keep-out rule area (#1442 follow-up).

The pre-routing layer switches lay copper of their own -- a through via on a
source pad, a stub moved onto another layer -- before the router's obstacle
map (which stamps rule areas) is ever asked. Their fit checks looked at
foreign copper only, so a "solo source switch" dropped a pad via inside a
vias-not-allowed area and check_drc graded it. The three checks those passes
go through now ask check_drc's own keep-out measurement (`keepout_refusal`):

  1. via_barrel_clear_of_foreign_copper refuses a pad via inside the area,
     and passes the same via with the area elsewhere (non-vacuity);
  2. stub_clear_of_foreign_pads refuses a stub moved onto the area's layer,
     and passes it moved onto a layer the area does not cover;
  3. _bare_pad_pair_vias_fit refuses a swap via inside the area;
  4. the router's terminal bridge from a through via of the net (on the board,
     or one the same path drills) to the path's first grid cell, both inside
     the via's copper, is dropped when it would
     reach into an area forbidding tracks on that (routable) layer -- the via
     already joins both ends -- and kept where no area forbids it;
  5. the diff-pair GND return-via ring search, which checks its candidates on
     a WINDOW of the board, runs and settles no via inside the area.

    python3 tests/test_1442_layer_switch_keepout.py
"""
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb, Segment, Via          # noqa: E402
from routing_config import GridRouteConfig                     # noqa: E402
from stub_layer_switching import (via_barrel_clear_of_foreign_copper,  # noqa: E402
                                  stub_clear_of_foreign_pads)
from layer_swap_optimization import _bare_pad_pair_vias_fit    # noqa: E402
from single_ended_routing import _bridge_inside_own_through_via  # noqa: E402
from diff_pair_routing import _gnd_via_ring_site               # noqa: E402
import run_utils                                               # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
# open board space: no pad or track within 3 mm
X, Y = 127.0, 100.0
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def with_area(tmp, name, rect, layers):
    out = os.path.join(tmp, f'{name}.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/add_rule_area.py',
                        BOARD, out, '--name', 'KO', '--layers', *layers,
                        '--rect', *[str(v) for v in rect]],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    if r.returncode != 0:
        raise SystemExit(f'add_rule_area failed: {(r.stdout + r.stderr)[-400:]}')
    run_utils.evidence(out, f'the board with area {name}')
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(out)


def main():
    tmp = tempfile.mkdtemp(prefix='t1442ls_')
    try:
        here = with_area(tmp, 'here', (X - 1, Y - 1, X + 1, Y + 1), ['B.Cu'])
        away = with_area(tmp, 'away', (X + 9, Y - 1, X + 11, Y + 1), ['B.Cu'])
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
        net = next(i for i, n in here.nets.items() if i and n.name)

        ok, why = via_barrel_clear_of_foreign_copper(X, Y, net, away, cfg, set())
        check('pad via with the area elsewhere: clear (non-vacuity)', ok, why)
        ok, why = via_barrel_clear_of_foreign_copper(X, Y, net, here, cfg, set())
        check('pad via inside a vias-not-allowed area: refused',
              not ok and 'keep-out rule area KO' in why, why)

        stub = Segment(start_x=X - 0.5, start_y=Y, end_x=X + 0.5, end_y=Y,
                       width=0.2, layer='F.Cu', net_id=net)
        ok, why = stub_clear_of_foreign_pads([stub], 'F.Cu', net, here, cfg, set())
        check('stub onto a layer the area does not cover: clear', ok, why)
        ok, why = stub_clear_of_foreign_pads([stub], 'B.Cu', net, here, cfg, set())
        check('stub onto the area\'s layer: refused',
              not ok and 'keep-out rule area KO' in why, why)
        check('the candidate stub itself is not moved', stub.layer == 'F.Cu')

        via = Via(x=X, y=Y, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=net)
        ok, why = _bare_pad_pair_vias_fit(away, [via], cfg)
        check('swap via with the area elsewhere: fits (non-vacuity)', ok, why)
        ok, why = _bare_pad_pair_vias_fit(here, [via], cfg)
        check('swap via inside the area: refused',
              not ok and 'keep-out rule area KO' in why, why)

        # 4. the terminal bridge inside the net's own through via -- first
        # one the same path drills (not on the board yet), then one on it
        own = Via(x=X, y=Y, size=0.45, drill=0.2, layers=['F.Cu', 'B.Cu'],
                  net_id=net)
        a, b = (X, Y), (X + 0.05, Y)
        check('bridge into the area, no via on the board: kept',
              _bridge_inside_own_through_via(here, net, cfg, 'B.Cu', a, b) is False)
        check('...the via this path drills there: dropped',
              _bridge_inside_own_through_via(here, net, cfg, 'B.Cu', a, b,
                                             path_vias=[own]) is True)
        for pcb in (here, away):
            pcb.vias.append(own)
        a, b = (X, Y), (X + 0.05, Y)
        check('bridge into the area on a routable layer: dropped',
              _bridge_inside_own_through_via(here, net, cfg, 'B.Cu', a, b) is True)
        check('the same bridge with the area elsewhere: kept (non-vacuity)',
              _bridge_inside_own_through_via(away, net, cfg, 'B.Cu', a, b) is False)
        check('a bridge leaving the via\'s copper: kept',
              _bridge_inside_own_through_via(here, net, cfg, 'B.Cu', a,
                                             (X + 0.5, Y)) is False)

        # 5. the ring search on a board window (it used to raise
        # AttributeError: the window carried no board_info)
        gnd = next(i for i, n in here.nets.items() if i and i != net and n.name)
        for pcb in (here, away):
            pcb.vias.remove(own)
        site = _gnd_via_ring_site([(X, Y)], (X, Y), cfg, gnd, here, (net,),
                                  [], [], [])
        check('GND via ring search on a board with the area: runs, and no '
              'site inside it', site is None or not (
                  X - 1 < site[0] < X + 1 and Y - 1 < site[1] < Y + 1), str(site))
        site_away = _gnd_via_ring_site([(X, Y)], (X, Y), cfg, gnd, away, (net,),
                                       [], [], [])
        check('...and with the area elsewhere, the site at the anchor '
              '(non-vacuity)', site_away is not None
              and abs(site_away[0] - X) < 1 and abs(site_away[1] - Y) < 1,
              str(site_away))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
