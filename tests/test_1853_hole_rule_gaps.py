#!/usr/bin/env python3
"""#1853 review: the gaps a hole_clearance rule left after the route step
first read it (test_1853_route_hole_clearance holds the first cut).

The fixtures are tests/fixtures/1853's (Default class clearance 0.0889 mm,
track 0.1 mm, via 0.6/0.3, rule `hole_clearance (min 0.254mm)`, routed at a
0.01 mm grid), edited here where a case needs a second pad or another rule.

Checks:
  1. a pad's hole is held when the pad's OWN net is routed in the same step:
     `pth_tight` / `pth_fits` with a second pad of the hole's net, routed
     `--nets SIG OTHER`. The base map leaves a routed net's pads to that
     net's own keep-outs, which held the pad's copper and not its hole, so
     SIG ran 0.45 mm from the hole's centre (0.250 mm from the hole, 0.254
     needed) and check_drc failed the board on the rule. Now SIG is refused
     on the tight board and routed clear on the other, check_drc clean; and
     the per-net cache holds a new via's copper off such a hole too.
"""
import json
import math
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1853')
FAILS = []

# a second pad of the hole's net, away from SIG's row
SECOND_PAD = '''		(pad "2" smd rect
			(at 2 -5)
			(size 0.3 0.3)
			(layers "F.Cu" "F.Mask")
			(net "OTHER")
			(uuid "72cb077a-5ebd-58ae-bf6c-43ff023feac7")
		)
'''


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru=None, edit=None):
    """Copy a fixture into ``td``; ``edit`` rewrites its board text."""
    os.makedirs(td, exist_ok=True)
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        shutil.copy(evidence(os.path.join(FIX, name + ext), 'the fixture'),
                    os.path.join(td, name + ext))
    board = os.path.join(td, name + '.kicad_pcb')
    if edit is not None:
        with open(board, encoding='utf-8') as fh:
            text = fh.read()
        new = edit(text)
        if new == text:
            raise RuntimeError(f"the edit of {name} changed nothing")
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(new)
    if dru is not None:
        with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return board


def add_second_pad(text):
    """Give the hole's footprint a second pad of its net, after the first."""
    i = text.index('(net "OTHER")')
    j = text.index('\t\t)\n', i) + len('\t\t)\n')
    return text[:j] + SECOND_PAD + text[j:]


def route(td, board, tag, nets, extra=()):
    out = os.path.join(td, tag + '.kicad_pcb')
    summary = os.path.join(td, tag + '.json')
    argv = (PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out, '--nets']
            + list(nets) + ['--grid-step', '0.01', '--escalation', 'off',
                            '--json-out', summary] + list(extra))
    r = check(argv, accept=True, timeout=900)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return (out, sorted(doc.get('routed_single') or []),
            sorted(doc.get('failed_single') or []), (r.stdout or '') + (r.stderr or ''))


def drc_clean(board):
    """check_drc passes the board; a failure is reported, a crash raised."""
    argv = PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                 evidence(board, 'the routed board'), '--clearance-margin-mm', '0.0005']
    try:
        r = check(argv, accept=True)
    except AssertionError as e:
        if 'BROKEN TEST' in str(e) or 'DRC VIOLATIONS' not in str(e):
            raise
        print(str(e)[-700:])
        return False
    return 'NO DRC VIOLATIONS FOUND' in (r.stdout or '')


def load(board):
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(board)


def rules_of(board, pcb):
    from routing_config import GridRouteConfig
    from design_rules import install_design_rules
    import contextlib
    import io
    cfg = GridRouteConfig(grid_step=0.01, clearance=0.0889, track_width=0.1,
                          via_size=0.6, via_drill=0.3,
                          layers=list(pcb.board_info.copper_layers))
    with contextlib.redirect_stdout(io.StringIO()):
        install_design_rules(cfg, board, pcb)
    return cfg


def net_id(pcb, name):
    return next(n for n in pcb.nets.values() if n.name == name).net_id


def sig_reach(pcb, x, y):
    from geometry_utils import point_to_segment_distance
    sig = net_id(pcb, 'SIG')
    return min(point_to_segment_distance(x, y, s.start_x, s.start_y, s.end_x, s.end_y)
               for s in pcb.segments if s.net_id == sig)


def routed_pad_hole(td):
    print("1. a pad's hole, its own net routed in the same step")
    sub = os.path.join(td, 'tight')
    out, routed, failed, _log = route(sub, stage(sub, 'pth_tight', edit=add_second_pad),
                                      'out', ('SIG', 'OTHER'), ('--layers', 'F.Cu'))
    ok('SIG' in failed and 'SIG' not in routed,
       f"pth_tight: SIG is refused the row 0.45 mm from the hole: {routed} / {failed}")
    ok(drc_clean(out), "and check_drc is clean")

    sub = os.path.join(td, 'fits')
    out, routed, failed, _log = route(sub, stage(sub, 'pth_fits', edit=add_second_pad),
                                      'out', ('SIG', 'OTHER'), ('--layers', 'F.Cu'))
    ok(routed == ['OTHER', 'SIG'] and failed == [], f"pth_fits: both route: {routed} / {failed}")
    d = sig_reach(load(out), 15.0, 6.0)
    ok(d >= 0.15 + 0.254 + 0.05 - 1e-6,
       f"SIG stays {d:.3f} mm from the hole's centre (needs {0.15 + 0.254 + 0.05:.3f})")
    ok(drc_clean(out), "and check_drc is clean")

    # the per-net cache: the hole's net is routed, so its pads come from there
    print("   the per-net cache of the hole's net")
    from obstacle_cache import precompute_net_obstacles
    small_ring = (lambda t: add_second_pad(t).replace(
        '(size 0.6 0.6)\n\t\t\t(drill 0.3)', '(size 0.34 0.34)\n\t\t\t(drill 0.3)'))
    got = {}
    for tag, dru in (('rule', None), ('none', '(version 1)\n')):
        b = stage(os.path.join(td, 'cache_' + tag), 'pth_fits', dru=dru, edit=small_ring)
        pcb = load(b)
        cfg = rules_of(b, pcb)
        data = precompute_net_obstacles(pcb, net_id(pcb, 'OTHER'), cfg)
        got[tag] = (cfg, {tuple(c) for c in data.blocked_cells.tolist()},
                    {tuple(c) for c in data.blocked_vias.tolist()})
    cfg, cells, vias = got['rule']
    _c0, cells0, vias0 = got['none']
    # a track centre 0.45 mm from the hole: past the 0.17 mm pad's copper
    # keep-out (0.17 + 0.0889 + 0.05), inside the rule's (0.15 + 0.254 + 0.05)
    ok((1500, 645, 0) in cells and (1500, 645, 0) not in cells0,
       "a track cell 0.45 mm from the hole is held with the rule, not without")
    # a new via's centre 0.68 mm from the hole: its copper stands 0.23 mm
    # from the hole (hole_r + rule + via_size/2 = 0.704 needed), past the
    # thin pad's via keep-out and the hole-to-hole stamp
    h2h_reach = 0.15 + 0.15 + cfg.hole_to_hole_clearance
    pad_reach = 0.17 + 0.3 + 0.0889 + 0.005 + cfg.via_hole_extra(0.0889)
    r = 0.68
    ok(max(h2h_reach, pad_reach) < r < 0.704,
       f"the probe at {r} mm lies past the other keep-outs ({max(h2h_reach, pad_reach):.3f})")
    probe = (1500, 600 + int(round(r / 0.01)))
    ok(probe in vias and probe not in vias0,
       "a new via 0.68 mm from the hole is held with the rule, not without")


def main():
    with tempfile.TemporaryDirectory() as td:
        routed_pad_hole(td)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
