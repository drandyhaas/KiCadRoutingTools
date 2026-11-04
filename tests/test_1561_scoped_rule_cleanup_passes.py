"""#1561: a class/type-scoped .kicad_dru clearance rule (#1491) holds through
the passes that MOVE copper after routing.

The router stamps obstacles at a scoped rule (`A.NetClass == 'HV' &&
B.NetClass == 'HV'`, 0.6 mm over an HV class of 0.1), but the post-route
passes that lay new copper -- the octolinear smoothing, the graze re-bend,
micro-shift and via nudge, and the terminal checks -- measured foreign copper
through the `_seg_foreign_*_dist` helpers, which priced the class value and
never the scoped rule. The smoothing then collapsed an HV route to 0.2 mm
from another HV net, and the run reported nothing. The helpers now take the
call's config and fold the scoped rule wherever it binds, by item type.

    python3 tests/test_1561_scoped_rule_cleanup_passes.py
"""
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(TESTS, 'oracle'))

from constraint_agreement import write_board  # noqa: E402

failures = []

HV_RULE = ('(rule "HV to HV" (constraint clearance (min 0.6mm)) '
           "(condition \"A.NetClass == 'HV' && B.NetClass == 'HV'\"))")
HV_CLASS = {'name': 'HV', 'clearance': 0.1, 'track_width': 0.2, 'priority': 0}


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def _board(td, name, b_net, segments, vias=(), footprints=()):
    """Nets A (1) and, when `b_net` is 2, B in class HV at 0.1 mm under a
    0.6 mm HV-to-HV rule; net C (3) stays Default."""
    b = os.path.join(td, name)
    write_board(b, classes=[HV_CLASS], patterns=[('A', 'HV'), ('B', 'HV')],
                rules={'min_clearance': 0.05}, dru=HV_RULE,
                segments=segments, vias=vias, footprints=footprints)
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(b)


def _config(pcb, routed=(1,)):
    from routing_config import GridRouteConfig
    from kicad_dru import install_track_clearances
    import contextlib
    import io
    cfg = GridRouteConfig()
    cfg.clearance = 0.1
    cfg.layers = ['F.Cu', 'B.Cu']
    cfg.set_net_clearances({1: 0.1, 2: 0.1}, list(routed))
    with contextlib.redirect_stdout(io.StringIO()):
        install_track_clearances(cfg, None, pcb.source_path, pcb,
                                 routed_net_ids=list(routed))
    return cfg


def helpers(td):
    print('1. the foreign-copper distance helpers fold the scoped rule')
    from single_ended_routing import (_seg_foreign_seg_dist, _seg_foreign_via_dist,
                                      _seg_foreign_pad_dist, _pt_foreign_pad_dist)
    for b_net, label, want in ((2, 'HV net B', 0.5), (3, 'Default net C', 0.1)):
        pcb = _board(td, f'h{b_net}.kicad_pcb', b_net,
                     segments=[(5, 10.5, 15, 10.5, 0.2, 'F.Cu', b_net)],
                     vias=[(10, 14, 0.6, 0.3, b_net)],
                     footprints=[{'ref': 'U1', 'x': 10, 'y': 18, 'net_id': b_net,
                                  'net_name': {2: 'B', 3: 'C'}[b_net], 'size': 1.0}])
        cfg = _config(pcb)
        nc = cfg.net_clearances
        # net C sits in Default (0.2): its class excess over the HV base
        # 0.1 is 0.1 whether or not the config is passed
        args = (pcb, 1, 5, 10, 15, 10, 'F.Cu')
        raw = _seg_foreign_seg_dist(*args, net_clearances=nc, base_clearance=0.1)
        got = _seg_foreign_seg_dist(*args, net_clearances=nc, base_clearance=0.1,
                                    config=cfg)
        check(f'track vs {label}: the scoped rule takes {want if b_net == 2 else 0} '
              f'mm more', abs((raw - got) - (want if b_net == 2 else 0.0)) < 1e-6,
              f'{raw} -> {got}')
        args = (pcb, 1, 5, 13, 15, 13, 'F.Cu')
        raw = _seg_foreign_via_dist(*args, net_clearances=nc, base_clearance=0.1)
        got = _seg_foreign_via_dist(*args, net_clearances=nc, base_clearance=0.1,
                                    config=cfg)
        check(f'track vs a via of {label}', abs((raw - got) - (want if b_net == 2 else 0.0))
              < 1e-6, f'{raw} -> {got}')
        args = (pcb, 1, 5, 17, 15, 17, 'F.Cu')
        raw = _seg_foreign_pad_dist(*args, net_clearances=nc, base_clearance=0.1)
        got = _seg_foreign_pad_dist(*args, net_clearances=nc, base_clearance=0.1,
                                    config=cfg)
        check(f'track vs a pad of {label}', abs((raw - got) - (want if b_net == 2 else 0.0))
              < 1e-6, f'{raw} -> {got}')
        raw = _pt_foreign_pad_dist(pcb, 1, 10, 17, 'F.Cu', base_clearance=0.1,
                                   net_clearances=nc)
        got = _pt_foreign_pad_dist(pcb, 1, 10, 17, 'F.Cu', base_clearance=0.1,
                                   net_clearances=nc, config=cfg, own_type='via')
        check(f'a via site vs a pad of {label}',
              abs((raw - got) - (want if b_net == 2 else 0.0)) < 1e-6, f'{raw} -> {got}')


def _u_route(b_net):
    # net A: a U detour between its two pads, which the smoothing would
    # collapse to the straight run at y=10 -- 0.3 mm (edge) from a short
    # track of `b_net` at y=9.5
    segs = [(5, 10, 5, 12, 0.2, 'F.Cu', 1), (5, 12, 15, 12, 0.2, 'F.Cu', 1),
            (15, 12, 15, 10, 0.2, 'F.Cu', 1), (8, 9.5, 12, 9.5, 0.2, 'F.Cu', b_net)]
    fps = [{'ref': 'U1', 'x': 5, 'y': 10, 'net_id': 1, 'net_name': 'A', 'size': 0.4},
           {'ref': 'U2', 'x': 15, 'y': 10, 'net_id': 1, 'net_name': 'A', 'size': 0.4}]
    return segs, fps


def smoothing(td):
    print('2. the octolinear smoothing does not lay copper inside the rule')
    from pcb_modification import smooth_octolinear_chains
    import contextlib
    import io
    from geometry_utils import segment_to_segment_closest_points
    for b_net, label, need in ((3, 'a Default track (class 0.2)', 0.2),
                               (2, 'an HV track (rule 0.6)', 0.6)):
        segs, fps = _u_route(b_net)
        pcb = _board(td, f's{b_net}.kicad_pcb', b_net, segments=segs, footprints=fps)
        cfg = _config(pcb)
        with contextlib.redirect_stdout(io.StringIO()):
            out = smooth_octolinear_chains([], pcb, {1}, clearance=0.1,
                                           net_clearances=cfg.net_clearances,
                                           config=cfg)
        other = next(s for s in pcb.segments if s.net_id == b_net)
        gap = min(segment_to_segment_closest_points(s, other)[0]
                  - (s.width + other.width) / 2.0
            for s in pcb.segments if s.net_id == 1)
        check(f'the U beside {label} is shortened', out[4].get('spans', 0) > 0,
              f'stats={out[4]}')
        # (the straight run the class value admits is 0.3 mm from the track)
        check(f'...and keeps {need} mm from it', gap >= need - 1e-4, f'gap={gap:.4f}')


def main():
    td = tempfile.mkdtemp(prefix='krt1561_')
    try:
        for t in (helpers, smoothing):
            t(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
