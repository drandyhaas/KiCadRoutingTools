"""#1530: a part that draws no courtyard grades clean at its own declared pose.

A `fixed_poses[]` entry (or a mechanical.json pose) compiles into a grade-only
anchor zone, `fixed:<ref>` (`mech:<ref>`), built by `reconcile.anchor_blocks`
at the declared pose. `rule_zone_containment` grades a part on the ZONE
basis (`floorplan.zone_parts`), which since #1466 is the ESTIMATED courtyard
for a part that draws none. The anchor was built on the state's pad box, so
the zone was smaller than the part it held: whenever the estimate's centre
lay off the pad box's, the part failed its own anchor at exactly its
declared pose, at a fixed ERROR no severity map could demote.

Pinned here, on a tracked fixture whose pad-bearing parts draw no courtyard:
  1. every such part, declared at the pose it already has, grades clean
     against its own `fixed:` anchor, and against a `mech:` anchor;
  2. the anchor zone covers the estimated courtyard the rule grades;
  3. the anchor still bites: the part moved off its pose is a finding.

    python3 tests/test_1530_anchor_zone_basis.py
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
import pose_score                                             # noqa: E402
from placement import floorplan, reconcile                    # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _quiet(fn, *a, **kw):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = fn(*a, **kw)
    return out


def _pose(fp, dx=0.0):
    return {'x': fp.x + dx, 'y': fp.y, 'rot': (fp.rotation or 0.0) % 360.0,
            'reason': 'the pose it already has'}


def main():
    pcb = parse_kicad_pcb(BOARD)
    state = _quiet(pose_score.make_state, pcb, BOARD)
    estimated = sorted(r for r in floorplan.zone_parts(state, pcb, BOARD)
                       if floorplan.zone_parts(state, pcb, BOARD)[r]
                       is not state.parts.get(r))
    check('the fixture has courtyard-less parts with an estimate',
          len(estimated) >= 3, estimated)
    # the parts whose estimate is not centred on their pad box: the shape
    # the old anchor failed
    off_centre = []
    for ref in estimated:
        est = floorplan.zone_parts(state, pcb, BOARD)[ref].rect()
        pad = state.parts[ref].rect()
        if (abs((est[0] + est[2]) - (pad[0] + pad[2])) > 0.1
                or abs((est[1] + est[3]) - (pad[1] + pad[3])) > 0.1):
            off_centre.append(ref)
    check('...some with the estimate off the pad box\'s centre',
          bool(off_centre), estimated)

    for prefix, basis in (('fixed:', 'fixed_pose'), ('mech:', 'mechanical')):
        found = {}
        for ref in estimated:
            fp = pcb.footprints[ref]
            doc = {'poses': {ref: _pose(fp)}, 'path': 'test'}
            vs = _quiet(floorplan.mechanical_anchor_violations, pcb, BOARD,
                        doc, state=state, prefix=prefix, basis=basis)
            if vs:
                found[ref] = [v.message for v in vs]
        check(f'every courtyard-less part at its own pose grades clean '
              f'against its {prefix}<ref> anchor', not found, found)

    zp = floorplan.zone_parts(state, pcb, BOARD)
    bad = {}
    for ref in off_centre:
        fp = pcb.footprints[ref]
        blocks, _ = _quiet(reconcile.anchor_blocks, pcb, BOARD,
                           {'poses': {ref: _pose(fp)}}, state=state,
                           prefix='fixed:', basis='fixed_pose')
        zone, est = blocks[0]['zone'], zp[ref].rect()
        if not (zone[0] <= est[0] + 1e-6 and zone[1] <= est[1] + 1e-6
                and zone[2] >= est[2] - 1e-6 and zone[3] >= est[3] - 1e-6):
            bad[ref] = (zone, [round(v, 4) for v in est])
    check('the anchor zone covers the estimated courtyard the rule grades',
          not bad, bad)

    ref = off_centre[0] if off_centre else estimated[0]
    fp = pcb.footprints[ref]
    moved = {'poses': {ref: _pose(fp, dx=3.0)}, 'path': 'test'}
    vs = _quiet(floorplan.mechanical_anchor_violations, pcb, BOARD, moved,
                state=state, prefix='fixed:', basis='fixed_pose')
    check('a part 3 mm off its declared pose still fails its anchor',
          any(v.rule == 'zone_containment' and v.ref == ref for v in vs),
          [v.message for v in vs])

    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
