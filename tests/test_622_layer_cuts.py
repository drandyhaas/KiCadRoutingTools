#!/usr/bin/env python3
"""The whole route's LAYER cuts from the loop's own evidence (whole_route.layer_cut, audit_layer_cuts, snap_layer_cuts).

  python3 tests/test_622_layer_cuts.py

A lane the audit finds against an island on one layer, or the snap cannot lay with its search stuck beside one, is held
on the other layer across that island (whole_solve reads the cut files' `lcuts`) -- a wall the lanes cannot go round.
On a geometry's islands of its own -- WALL on F alone, BARREL on both layers, FAR on B -- this pins:

1. a cut only for an island on one layer, to the other layer, carrying the island's box; none for one on both layers
   (no change answers it) and none where the finding's layer is not the island's;
2. an audit's STATIC findings against a pad or a hole of an island give its cut; a finding against other copper, or a
   pad of no island, gives none; each lane and island once;
3. a snap's failure stuck within SNAP_ISLAND_REACH of an island on its layer gives a cut under the NEAREST such
   island; one stuck farther, or beside an island on the other layer only, gives none;
4. on MORE routing layers than two, a finding against another net's TRACK (copper) gives a cut round that place too, as
   one against an array's own pad does: the lane held off every layer the bench's other tracks cross it on (a bus an
   earlier step routed on several layers -- sdi_mipi_conv's DDR3 bus along U11's south face, which a lane moved off
   one layer a round found again on the next), the finding's own among them; none where they cross it on every
   layer, which no change answers;
5. a KEEP-OUT rule area whose tracks are not allowed on some layers (the router stamps it; the plan did not see it):
   plan_audit.keepout_areas resolves its layers as the router does ('*.Cu' every one), keepout_dist is negative
   inside it and positive outside, its hole outside; the audit's finding against it holds the lane off its layers,
   the cut carrying the area's shape read off the bench, and own_spans holds it off them where its laid path meets
   the AREA (an L), not the area's box.
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.argv = sys.argv[:1]
import whole_route as wr  # noqa: E402

GEO = {'islands': {'C1.1': 'WALL', 'C1.2': 'WALL', 'H1.': 'WALL', 'TP1.1': 'BARREL', 'C9.1': 'FAR'},
       'island_boxes': {'WALL': [5.0, -3.0, 6.0, 3.0, [0]], 'BARREL': [5.0, 4.0, 6.0, 5.0, [0, 1]],
                        'FAR': [5.0, 8.0, 6.0, 9.0, [1]]}}

AUDIT = """STATIC SYN05   F -0.257/0.181 pad C1.1 0 at (4.91,2.74)
STATIC SYN05   F -0.079/0.181 pad C1.2 0 at (4.69,2.20)
STATIC SYN06   F +0.143/0.181 hole H1.  at (4.50,1.12)
STATIC SYN07   F -0.050/0.181 pad TP1.1 GND at (4.50,4.50)
STATIC SYN08   F -0.050/0.181 copper SYN09 at (3.00,0.00)
STATIC SYN09   F -0.050/0.181 pad U7.3 0 at (3.00,0.00)
STATIC 6 lane/object pair(s) short of their bar (track/2 + clearance 0.168, + half a grid step off the grid): {}
"""

SNAP = """snap: 9/12 lanes laid, 0 vias, 26 s
    SYN00   FAILED: no path in its band (54080 states searched); the farthest it got: 8.39 of 21.85 mm along it, at (4.20, -4.10) on F.Cu
    SYN11   FAILED: no path in its band (61033 states searched); the farthest it got: 8.28 of 19.95 mm along it, at (5.50, 12.50) on F.Cu
    SYN03   FAILED: no path in its band (1000 states searched); the farthest it got: 2.00 of 19.95 mm along it, at (5.50, 7.50) on F.Cu
    SYN04   FAILED: no path in its band (1000 states searched); the farthest it got: 2.00 of 19.95 mm along it, at (5.50, 7.50) on B.Cu
SNAP FAILED: 4 lane(s) could not be laid: SYN00, SYN11, SYN03, SYN04
"""


def main():
    print('=' * 60)
    print('the whole route\'s layer cuts from the audit and the snap')
    print('=' * 60)
    fails = []
    bx = GEO['island_boxes']
    # 1
    c = wr.layer_cut('SYN05', 'WALL', bx['WALL'], 0)
    if c != {'lane': 'SYN05', 'island': 'WALL', 'layer': 1, 'box': [5.0, -3.0, 6.0, 3.0]}:
        fails.append(f'one layer: {c}')
    for what, args in (('both layers', ('SYN05', 'BARREL', bx['BARREL'], 0)),
                       ('another layer', ('SYN05', 'WALL', bx['WALL'], 1)), ('no box', ('SYN05', 'X', None, 0))):
        if wr.layer_cut(*args) is not None:
            fails.append(f'{what}: a cut')
    with tempfile.TemporaryDirectory() as td:
        # 2
        pa = os.path.join(td, 'p.audit')
        open(pa, 'w').write(AUDIT)
        got = sorted((c['lane'], c['island'], c['layer']) for c in wr.audit_layer_cuts(pa, GEO))
        if got != [('SYN05', 'WALL', 1), ('SYN06', 'WALL', 1)]:
            fails.append(f'audit: {got}, want SYN05 and SYN06 under WALL once each (a hole too; not the barrel, '
                         f'other copper, nor a pad of no island)')
        # 3
        ps = os.path.join(td, 'snap.log')
        open(ps, 'w').write(SNAP)
        got = sorted((c['lane'], c['island'], c['layer']) for c in wr.snap_layer_cuts(ps, GEO))
        # SYN00 stuck on F 1.4 mm off WALL (F alone): under it, to B; SYN04 stuck on B 0.5 mm off FAR (B alone):
        # under it, to F; SYN11 stuck 3.5 mm off FAR, past the reach; SYN03 stuck on F beside FAR, on B alone, with
        # WALL 4.5 mm off
        if got != [('SYN00', 'WALL', 1), ('SYN04', 'FAR', 0)]:
            fails.append(f'snap: {got}, want SYN00 under WALL (to B) and SYN04 under FAR (to F)')
        if not 1.4 < wr.SNAP_ISLAND_REACH < 3.5:
            raise SystemExit(f'BROKEN TEST: the reach {wr.SNAP_ISLAND_REACH} is outside the distances it was drawn for')
        # 4
        os.environ['ROUTE_LAYERS'] = 'F.Cu,B.Cu,In1.Cu,In2.Cu'
        try:
            def bench(layers):
                segs = '\n'.join(f'  (segment (start 2.0 {-1.0 + 0.1 * i:.2f}) (end 4.0 {1.0 - 0.1 * i:.2f}) (width 0.15) '
                                 f'(layer "{L}") (net 2))' for i, L in enumerate(layers))
                pb = os.path.join(td, f'bench{len(layers)}.kicad_pcb')
                open(pb, 'w').write('(kicad_pcb (version 20240108) (generator "test")\n  (general (thickness 1.6))\n'
                                    '  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) '
                                    '(31 "B.Cu" signal) (44 "Edge.Cuts" user))\n  (net 0 "")\n  (net 1 "SYN08")\n'
                                    f'  (net 2 "SYN09")\n{segs}\n)\n')
                return pb
            cut8 = lambda cs: [c for c in cs if c['lane'] == 'SYN08']
            got = cut8(wr.audit_layer_cuts(pa, GEO))
            if not (len(got) == 1 and got[0]['blocked'] == [0] and got[0]['box'][0] < 3.0 < got[0]['box'][2]):
                fails.append(f'more layers, no bench: SYN08 against the track held off F round it: {got}')
            got = cut8(wr.audit_layer_cuts(pa, GEO, bench(['F.Cu', 'In1.Cu'])))
            if not (len(got) == 1 and got[0]['blocked'] == [0, 2]):
                fails.append(f'more layers, the track on F and In1: SYN08 held off both: {got}')
            got = cut8(wr.audit_layer_cuts(pa, GEO, bench(['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])))
            if got:
                fails.append(f'more layers, the track on every layer: no cut: {got}')
            if not any(c['lane'] == 'SYN09' for c in wr.audit_layer_cuts(pa, GEO)):
                fails.append('more layers: the array-pad finding\'s cut (as before)')
            # 5
            import numpy as np
            import plan_audit as pa_
            import types
            lays4 = ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']
            sq = [(10.0, 10.0), (20.0, 10.0), (20.0, 20.0), (10.0, 20.0)]
            hole = [(14.0, 14.0), (16.0, 14.0), (16.0, 16.0), (14.0, 16.0)]
            bi = types.SimpleNamespace(keepouts=[
                {'polygon': sq, 'holes': [hole], 'layers': {'In1.Cu', 'In2.Cu'}, 'tracks_allowed': False},
                {'polygon': sq, 'holes': [], 'layers': {'*.Cu'}, 'tracks_allowed': False},
                {'polygon': sq, 'holes': [], 'layers': {'In1.Cu'}, 'tracks_allowed': True}])
            ka = pa_.keepout_areas(types.SimpleNamespace(board_info=bi), lays4)
            if [sorted(k_[2]) for k_ in ka] != [['In1.Cu', 'In2.Cu'], sorted(lays4)]:
                fails.append(f'keepout_areas: {[sorted(k_[2]) for k_ in ka]}, want its two layers, then every one '
                             f'(the tracks-allowed area left out)')
            d_ = pa_.keepout_dist(np.array([[12.0, 12.0], [25.0, 12.0], [15.0, 15.0]]), ka[0][0], ka[0][1])
            if not (d_[0] < -1.9 and abs(d_[1] - 5.0) < 1e-6 and abs(d_[2] - 1.0) < 1e-6):
                fails.append(f'keepout_dist: inside, outside, in its hole: {d_.tolist()}')
            ko = ('(zone (net 0) (net_name "") (layers "In1.Cu" "In2.Cu") (hatch edge 0.5) (connect_pads (clearance 0))'
                  ' (min_thickness 0.25) (keepout (tracks not_allowed) (vias allowed) (pads allowed) '
                  '(copperpour allowed) (footprints allowed)) (fill (thermal_gap 0.5) (thermal_bridge_width 0.5)) '
                  '(polygon (pts (xy 10 10) (xy 20 10) (xy 20 20) (xy 10 20))))')
            pk = os.path.join(td, 'ko.kicad_pcb')
            open(pk, 'w').write('(kicad_pcb (version 20240108) (generator "test")\n  (general (thickness 1.6))\n'
                                '  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) '
                                '(31 "B.Cu" signal) (44 "Edge.Cuts" user))\n  (net 0 "")\n  (net 1 "SYN08")\n'
                                f'  {ko}\n)\n')
            pk_a = os.path.join(td, 'ko.audit')
            open(pk_a, 'w').write('STATIC SYN08   In2.Cu -1.412/0.193 keepout K0 In1.Cu+In2.Cu at (15.00,12.00)\n')
            # (the audit's own finding: a lane piece across the area on In2.Cu, another on F.Cu, which it keeps no
            # track off)
            import contextlib
            import io
            ctx_ = types.SimpleNamespace(cfg=types.SimpleNamespace(track_width=0.15, clearance=0.1, grid_step=0.025,
                                                                   layers=lays4),
                                         pcb=types.SimpleNamespace(footprints={}, nets={}, board_info=types.SimpleNamespace(
                                             keepouts=bi.keepouts[:1])), pairs={},
                                         base_segments=[], base_vias=[], byname={'SYN08': (1, None)})
            cor_ = types.SimpleNamespace(members=['SYN08'], virtual_of=lambda names: [
                ((12.0, 12.0), (25.0, 12.0), 'In2.Cu'), ((12.0, 18.0), (25.0, 18.0), 'F.Cu')])
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rows_ = pa_.check_static(ctx_, [cor_])
            fl_ = [ln for ln in buf.getvalue().splitlines() if ln.startswith('STATIC SYN08')]
            if not (len(fl_) == 1 and 'In2.Cu' in fl_[0] and 'keepout K0 In1.Cu+In2.Cu' in fl_[0]):
                fails.append(f'the audit: one finding, on In2.Cu against the area: {fl_}')
            got = wr.audit_layer_cuts(pk_a, GEO, pk)
            if got != [{'lane': 'SYN08', 'island': 'keepout K0', 'layer': 0, 'box': [10.0, 10.0, 20.0, 20.0],
                        'blocked': [2, 3], 'poly': [[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]],
                        'holes': []}]:
                fails.append(f'keepout cut: {got}, want SYN08 held off In1 and In2 across the area, its shape with it')
            # ...held off them where its laid path meets the AREA, not its box: an L whose box the path crosses for
            # 20 mm and whose shape for 2 (sdi_mipi_conv's area round U15, whose box reaches U15's berths)
            pl = os.path.join(td, 'ko_l.kicad_pcb')
            open(pl, 'w').write(open(pk).read().replace(
                '(xy 10 10) (xy 20 10) (xy 20 20) (xy 10 20)',
                '(xy 10 10) (xy 30 10) (xy 30 12) (xy 12 12) (xy 12 20) (xy 10 20)'))
            cl = wr.audit_layer_cuts(pk_a, GEO, pl)
            geo_ = {'uxy': {'SYN08': [[float(x_), float(x_), 15.0] for x_ in range(0, 41)]},
                    'lanes': {'SYN08': {'xy': [[0.0, 15.0], [40.0, 15.0]]}}}
            sp_ = wr.own_spans({'lcuts': cl}, geo_)['lcuts']
            span = sp_[0].get('span') if len(sp_) == 1 else None
            if not (span and 9.5 < span[0] < 10.0 and 12.0 < span[1] < 12.5):
                fails.append(f'keepout span: {span}, want the 2 mm the path runs in the L (u 10..12, a clearance '
                             f'either side), not the 20 of its box')
        finally:
            os.environ.pop('ROUTE_LAYERS', None)
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: a cut only under an island on one layer, to the other; the audit\'s pad and hole findings against an '
          'island, each lane once; a snap failure under the nearest one-layer island on its layer within reach; on '
          'more layers, a track\'s finding held off the layers the bench\'s tracks cross it on')
    return 0


if __name__ == '__main__':
    sys.exit(main())
