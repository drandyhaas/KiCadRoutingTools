#!/usr/bin/env python3
"""#1033 part 3: every branch of power_widen.ExactWideCheck, one synthetic
case each, so removing any branch fails a named check.

The widen check decides whether OPTIONAL extra width fits; each term is a
reason to keep the narrower width. Every case below puts ONE obstacle of one
kind 0.2..0.35 mm off a piece's centreline and asserts:

    clears at 0.3 mm  -> False  (the obstacle is inside the wide copper's reach)
    clears at 0.127   -> True   (control: it is THAT obstacle, not a blanket no)

with 0.1 mm routing clearance unless the case says otherwise. Plus the
behaviours the verifier found untested: F&B.Cu pads reach the SAMPLED term
(not only the exact confirm), the exact confirm is the strict word at the
boundary, footprint graphic copper is foreign even where #908 lifts it for the
router, keep-outs on F&B.Cu, fail-closed on a raising check, a loud warning on
a raising constructor, and the post-route pass (widen_power_copper):
one board rewrite per net, the run's clearance, and nets judged in turn.

    python3 tests/test_1033_exact_wide_check.py
"""
import os
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from kicad_parser import BoardInfo, Via  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from synth import make_net, make_pad, make_pcb, make_seg  # noqa: E402

import power_widen  # noqa: E402
from power_widen import ExactWideCheck, widen_power_copper  # noqa: E402

fails = []
NET, FOREIGN = 1, 2


def check(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name
          + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def cfg(clearance=0.1):
    c = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.127,
                        clearance=clearance, grid_step=0.05)
    return c


def board(pads=None, segs=None, vias=None, bounds=(-3.0, -3.0, 5.0, 3.0),
          keepouts=None, footprints=None):
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'], board_bounds=bounds)
    if keepouts:
        bi.keepouts = keepouts
    pbn = {NET: [], FOREIGN: [], 0: []}
    for p in (pads or []):
        pbn.setdefault(p.net_id, []).append(p)
    return make_pcb(nets={NET: make_net(NET, '+3V3'),
                          FOREIGN: make_net(FOREIGN, 'SIG')},
                    pads_by_net=pbn, segments=list(segs or []),
                    vias=list(vias or []), board_info=bi,
                    footprints=footprints or {})


PIECE = (0.0, 0.0, 2.0, 0.0, 'F.Cu')


def wide_vs_narrow(name, pcb, c=None):
    chk = ExactWideCheck(pcb, c or cfg(), NET)
    wide = chk.clears(*PIECE, 0.3)
    narrow = chk.clears(*PIECE, 0.127)
    check(f'{name}: refuses 0.3', wide is False)
    check(f'{name}: control, still clears 0.127', narrow is True)
    return chk


def main():
    # baseline: nothing near -> clears at 0.3
    check('empty board: clears 0.3',
          ExactWideCheck(board(), cfg(), NET).clears(*PIECE, 0.3) is True)

    # 1. board edge (top edge 0.22 from the centreline)
    wide_vs_narrow('edge', board(bounds=(-3.0, -3.0, 5.0, 0.22)))

    # 2. keep-out on F&B.Cu (#369-style composite token), edge 0.2 away
    ko = [{'tracks_allowed': False, 'layers': {'F&B.Cu'},
           'polygon': [(0.0, 0.2), (2.0, 0.2), (2.0, 1.5), (0.0, 1.5)]}]
    wide_vs_narrow('keep-out on F&B.Cu', board(keepouts=ko))

    # 3. NPTH hole (mask-only, no copper) -- hole floor 0.2, wall 0.3 away:
    #    0.3 needs 0.35, 0.127 needs 0.2635
    npth = make_pad(0, 1.0, 0.3 + 0.5, ref='H1', num='', size_x=1.0,
                    size_y=1.0, shape='circle', layers=('F.Mask', 'B.Mask'),
                    drill=1.0, pad_type='np_thru_hole')
    wide_vs_narrow('NPTH hole floor', board(pads=[npth]))

    # 4. foreign via, copper edge 0.2 away
    v = Via(x=1.0, y=0.2 + 0.25, size=0.5, drill=0.3,
            layers=['F.Cu', 'B.Cu'], net_id=FOREIGN)
    wide_vs_narrow('foreign via', board(vias=[v]))

    # 5. foreign track, edge 0.2 away
    wide_vs_narrow('foreign track', board(
        segs=[make_seg(0.0, 0.3, 2.0, 0.3, width=0.2, net_id=FOREIGN)]))

    # 6. net class on a foreign PAD (sampled term only): class 0.25, edge 0.35
    #    away: 0.3 needs 0.4, 0.127 needs 0.3135. The exact confirm prices the
    #    pad at the pair/override clearance only, so ONLY the sampled term
    #    (which folds the class excess) refuses -- the sampled-pad and
    #    net_clearances mutants both die here.
    fp = make_pad(FOREIGN, 1.0, 0.35 + 0.2, ref='R1', num='1', size_x=0.4,
                  size_y=0.4, net_name='SIG')
    c6 = cfg()
    c6.net_clearances = {FOREIGN: 0.25}
    wide_vs_narrow('foreign pad net class (sampled term)', board(pads=[fp]), c6)

    # 7. .kicad_dru layer rule on F.Cu: 0.25, foreign track edge 0.35 away
    c7 = cfg()
    c7.layer_clearances = {'F.Cu': 0.25}
    wide_vs_narrow('.kicad_dru layer rule', board(
        segs=[make_seg(0.0, 0.45, 2.0, 0.45, width=0.2, net_id=FOREIGN)]), c7)

    # 8. F&B.Cu through-hole pad: the SAMPLED term must see it too
    tht = make_pad(FOREIGN, 1.0, 0.2 + 0.4, ref='J1', num='1', size_x=0.8,
                   size_y=0.8, shape='circle', layers=('F&B.Cu', '*.Mask'),
                   drill=0.4, pad_type='thru_hole', net_name='SIG')
    chk = wide_vs_narrow('F&B.Cu THT pad', board(pads=[tht]))
    from single_ended_routing import _seg_foreign_pad_dist
    d_view = _seg_foreign_pad_dist(chk.pad_view, NET, *PIECE, base_clearance=0.1)
    check('F&B.Cu THT pad: the SAMPLED term sees it (expanded layer view)',
          d_view < 0.1 + 0.15 - 1e-4, round(d_view, 4))
    # ...and through clears() itself: an F&B.Cu pad whose CLASS clearance
    # (0.25) is what refuses -- the exact confirm prices pair/override only,
    # so only a sampled term that SEES F&B.Cu pads can refuse this one
    tht2 = make_pad(FOREIGN, 1.0, 0.35 + 0.4, ref='J2', num='1', size_x=0.8,
                    size_y=0.8, shape='circle', layers=('F&B.Cu', '*.Mask'),
                    drill=0.4, pad_type='thru_hole', net_name='SIG')
    c8 = cfg()
    c8.net_clearances = {FOREIGN: 0.25}
    wide_vs_narrow('F&B.Cu THT pad net class (sampled term, expanded view)',
                   board(pads=[tht2]), c8)

    # 9. exact confirm is the strict word at the boundary: pad edge exactly
    #    5e-5 inside the 0.3 requirement -- the sampled term's 1e-4 slack
    #    passes it, check_drc's geometry does not
    need = 0.1 + 0.15
    bp = make_pad(FOREIGN, 1.0, (need - 5e-5) + 0.2, ref='R2', num='1',
                  size_x=0.4, size_y=0.4, net_name='SIG')
    chk9 = ExactWideCheck(board(pads=[bp]), cfg(), NET)
    check('boundary pad: refused at 0.3 (exact confirm)',
          chk9.clears(*PIECE, 0.3) is False)

    # 10. footprint GRAPHIC copper touching an OWN pad (the #908 lift makes it
    #     non-foreign to the router); for widening it stays foreign
    own = make_pad(NET, -0.6, 0.6, ref='AE1', num='1', size_x=0.4,
                   size_y=0.4, net_name='+3V3')
    g = make_seg(-0.6, 0.6, 2.0, 0.3, width=0.2, net_id=0, graphic=True,
                 owner_ref='AE1')
    g.graphic = True
    fps = {'AE1': SimpleNamespace(pads=[own], reference='AE1')}
    pcb10 = board(pads=[own], segs=[g], footprints=fps)
    from check_drc import graphic_own_pad_nets
    lifted = NET in (graphic_own_pad_nets(pcb10).get(id(g)) or ())
    check('graphic: fixture really is #908-lifted for the net', lifted)
    wide_vs_narrow('own-pad graphic copper (no lift for widening)', pcb10)

    # 11. fail CLOSED on a raising check, counted
    chk11 = ExactWideCheck(board(), cfg(), NET)
    before = power_widen.ERRORS['check_errors']

    def boom(*a, **k):
        raise RuntimeError('probe')
    chk11._clears = boom
    check('a raising check REFUSES the piece',
          chk11.clears(*PIECE, 0.3) is False)
    check('...and is counted', power_widen.ERRORS['check_errors'] == before + 1)

    # 12. a raising CONSTRUCTOR is loud, counted, and leaves the copper as
    #     routed (the post-route pass skips that net)
    import io
    import contextlib
    c12 = cfg()
    c12.power_net_widths = {NET: 0.3}
    bad = SimpleNamespace(segments=[])   # no pads_by_net / board_info: raises
    s12 = make_seg(0.0, 0.0, 6.0, 0.0, width=0.127, net_id=NET)
    r12 = [{'new_segments': [s12]}]
    ce0 = power_widen.ERRORS['ctor_errors']
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        st12 = widen_power_copper(r12, bad, c12)
    check('a raising constructor prints a WARNING naming the exception',
          'WARNING: power-width widen check could not be built' in buf.getvalue()
          and 'AttributeError' in buf.getvalue(), buf.getvalue()[-300:])
    check('...and is counted, and the copper stays as routed',
          power_widen.ERRORS['ctor_errors'] == ce0 + 1
          and st12['nets'] == 0 and r12[0]['new_segments'] == [s12])

    # 13. the post-route pass rewrites the board ONCE per net, after deciding
    #     every piece: several narrow segments across two
    #     results, a foreign track to keep the check busy -- every one widens,
    #     both result lists and the board carry the pieces, nothing stale
    c13 = cfg()
    c13.power_net_widths = {NET: 0.3}
    rs = [make_seg(0.0, -1.0, 2.0, -1.0, width=0.127, net_id=NET),
          make_seg(2.0, -1.0, 4.0, -1.0, width=0.127, net_id=NET),
          make_seg(4.0, -1.0, 4.0, 1.0, width=0.127, net_id=NET)]
    foreign = make_seg(-2.0, 2.0, 4.5, 2.0, width=0.2, net_id=FOREIGN)
    pcb13 = board(segs=[foreign] + rs)
    res13 = [{'new_segments': rs[:2]}, {'new_segments': rs[2:]}]
    st13 = widen_power_copper(res13, pcb13, c13)
    allnew = [x for r in res13 for x in r['new_segments']]
    check('post-pass: every narrow segment widened across results',
          abs(st13['widened_mm'] - 6.0) < 1e-6
          and all(x.width == 0.3 for x in allnew), st13)
    check('post-pass: the board holds no None and no stale originals',
          None not in pcb13.segments
          and not any(x in pcb13.segments for x in rs)
          and all(x in pcb13.segments for x in allnew))

    # 14. the run's clearance governs: foreign track edge 0.3 away -- 0.3
    #     clears at clearance 0.1 (needs 0.25); at 0.2 it does not (needs
    #     0.35) and the ladder's 0.15 does (needs 0.275)
    for clr, want in ((0.1, 0.3), (0.2, 0.15)):
        c14 = cfg(clearance=clr)
        c14.power_net_widths = {NET: 0.3}
        s14 = make_seg(0.0, 0.0, 2.0, 0.0, width=0.127, net_id=NET)
        pcb14 = board(segs=[make_seg(0.0, 0.4, 2.0, 0.4, width=0.2,
                                     net_id=FOREIGN), s14])
        r14 = [{'new_segments': [s14]}]
        widen_power_copper(r14, pcb14, c14)
        check(f'post-pass at clearance {clr}: width {want}',
              all(abs(x.width - want) < 1e-9 for x in r14[0]['new_segments']),
              [x.width for x in r14[0]['new_segments']])

    # 15. two power nets side by side: the second net's check must see the
    #     FIRST net's widened copper (the board is rewritten between nets).
    #     Centrelines 0.38 apart: net 1 widens to 0.3 (0.3165 from net 3's
    #     edge); net 3 then sees net 1 at 0.3 (0.23 away) -> only 0.15 fits.
    c15 = cfg()
    c15.power_net_widths = {NET: 0.3, 3: 0.3}
    a15 = make_seg(0.0, 0.0, 2.0, 0.0, width=0.127, net_id=NET)
    b15 = make_seg(0.0, 0.38, 2.0, 0.38, width=0.127, net_id=3)
    pcb15 = board(segs=[a15, b15])
    pcb15.nets[3] = make_net(3, '+5V')
    r15 = [{'new_segments': [a15]}, {'new_segments': [b15]}]
    widen_power_copper(r15, pcb15, c15)
    wa = max(x.width for x in r15[0]['new_segments'])
    wb = max(x.width for x in r15[1]['new_segments'])
    gap = 0.38 - wa / 2 - wb / 2
    check('two nets: the second is judged against the first as WIDENED '
          '(edge gap stays >= the 0.1 clearance)', gap >= 0.1 - 1e-6,
          (wa, wb, round(gap, 4)))

    # 16. #27 user keep-outs (--keepout): pcb_data.keepout_zones, active only
    #     with config.keepout_enabled (the smoother's rule)
    kz = SimpleNamespace(points=[(0.0, 0.2), (2.0, 0.2), (2.0, 1.5), (0.0, 1.5)])
    p16 = board()
    p16.keepout_zones = [kz]
    c16 = cfg()
    c16.keepout_enabled = True
    wide_vs_narrow('user --keepout zone', p16, c16)
    c16b = cfg()
    c16b.keepout_enabled = False
    check('user --keepout zone: inert when keepout_enabled is off',
          ExactWideCheck(p16, c16b, NET).clears(*PIECE, 0.3) is True)

    # 17. nets the smoother skips (protected / matched / impedance) are not
    #     widened either
    import cleanup_pipeline
    c17 = cfg()
    c17.power_net_widths = {NET: 0.3}
    s17 = make_seg(0.0, -1.0, 2.0, -1.0, width=0.127, net_id=NET)
    p17 = board(segs=[s17])
    orig_skip = cleanup_pipeline._smooth_skip_net_ids
    cleanup_pipeline._smooth_skip_net_ids = lambda _pcb: {NET}
    try:
        st17 = widen_power_copper([{'new_segments': [s17]}], p17, c17)
    finally:
        cleanup_pipeline._smooth_skip_net_ids = orig_skip
    check('a protected / impedance net is not widened', st17['nets'] == 0
          and s17.width == 0.127, st17)
    st17b = widen_power_copper([{'new_segments': [s17]}], p17, c17)
    check('control: the same net unprotected is widened', st17b['nets'] == 1,
          st17b)

    # 18. the failure counters reset at the start of each (outermost) run --
    #     a GUI session must not inherit the last run's
    import route as _route
    power_widen.ERRORS['check_errors'] = 5
    power_widen.ERRORS['ctor_errors'] = 2
    brd = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()):
        _route.batch_route(brd, '', ['/CLK'], return_results=True,
                           track_width=0.2, clearance=0.2, grid_step=0.1)
    check('ERRORS reset by batch_route (return_results, in-process)',
          power_widen.ERRORS['check_errors'] == 0
          and power_widen.ERRORS['ctor_errors'] == 0, dict(power_widen.ERRORS))

    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
