"""A diff pair whose own members short must not be written (#318 follow-up).

`_neck_pair_partner_grazes` (#318) necks a polarity's segment that sits
sub-clearance to its partner's just-created copper. It floors every width at
the fab minimum, so when the required width goes NEGATIVE it quietly shipped
copper that is still inside clearance -- necking was attempted, reported as
done, and the short went out.

THE CASE NECKING CANNOT REACH AT ALL is two COLLINEAR members: their gap is
measured END TO END, so narrowing the tracks barely moves it. Measured on
icepi_zero's /USB/D1 at x=143.100, where the coupled trunk hands off to the
terminal legs:

    trunk    D1+ y=110.444  D1- y=110.256   c-c 0.188   edge 0.0991  legal
    handoff  D1+ y=110.400  D1- y=110.300   c-c 0.100   edge 0.0111  SHORT

Clearing 0.09mm at 0.100mm centre-to-centre needs a track of about 0.01mm. The
geometry has to change, and the only honest answer at emission time is to
refuse the pair.

TWO SITES, because a pair reaches copper two ways and only one of them used to
be able to refuse:
  * the coupled emission point returns a failed result, so the ladder takes
    another option (the single-ended terminal SHORT gate, #157, does this);
  * the HYBRID escape assembles candidates per layer combination and already
    RANKED them by intra-pair overlaps -- but when every candidate for a
    combination grazed, the winner was still written. It now rejects that
    combination the way any other unusable one is rejected, and tries the next.

Measured on icepi_zero's real route_diff step: D1-pair violations 4 -> 0, total
DRC 26 -> 18, and the unconnected-net set IDENTICAL before and after (the pair
was already open; it was shipping a short AS WELL). Refusing cost no
connectivity here -- which is the trade this gate exists to keep honest, so it
checks both halves.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []


def _neck(p, n, cfg, pcb):
    """Call the helper and normalise its return.

    Before this fix it returned a bare `necked` count with no way to say
    "necking was not enough". Unpacking that raises TypeError, which would fail
    this file as a BROKEN TEST rather than as a detected regression -- so the
    old shape is recognised and reported as what it is.
    """
    from diff_pair_routing import _neck_pair_partner_grazes
    out = _neck_pair_partner_grazes(p, n, cfg, pcb)
    if isinstance(out, tuple):
        return out
    return out, None      # None == "this build cannot report hard violations"


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


class _Cfg:
    clearance = 0.09
    track_width = 0.0889


def _seg(x1, y1, x2, y2, net_id, w=0.0889, layer='F.Cu'):
    from kicad_parser import Segment
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=w, layer=layer, net_id=net_id)


def _pcb():
    from kicad_parser import PCBData, BoardInfo
    return PCBData(board_info=BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu'],
                                        board_bounds=(0.0, 0.0, 200.0, 200.0),
                                        stackup=[]),
                   nets={}, footprints={}, vias=[], segments=[], pads_by_net={})


def t_collinear_members_are_reported_hard():
    """icepi_zero's exact geometry: the handoff jogs, collinear at x=143.100."""
    p = [_seg(143.100, 110.444, 143.100, 110.400, 1)]
    n = [_seg(143.100, 110.256, 143.100, 110.300, 2)]
    necked, hard = _neck(p, n, _Cfg(), _pcb())
    if hard is None:
        check('t_collinear_members_are_reported_hard', False,
              'the helper returns no hard list at all -- an unfixable '
              'intra-pair graze is necked to the floor and WRITTEN')
        return
    check('t_collinear_members_are_reported_hard',
          bool(hard),
          f'{len(hard)} hard violation(s), gap '
          f'{min(h[2] for h in hard):.4f}mm against {_Cfg.clearance}mm'
          if hard else 'nothing reported -- the short would be written')
    # ...and necking really cannot save it, which is WHY it must be hard: even
    # at the fab floor the collinear gap stays inside clearance.
    if hard:
        check('t_necking_could_not_have_saved_it',
              min(h[2] for h in hard) < _Cfg.clearance,
              'still inside clearance after necking ran')


def t_a_properly_spaced_pair_is_not_refused():
    """The control: the coupled trunk itself must pass, or this gate refuses
    every pair and 'no shorts' becomes 'no diff pairs'."""
    p = [_seg(149.294, 110.444, 143.100, 110.444, 1)]
    n = [_seg(149.294, 110.256, 143.100, 110.256, 2)]
    necked, hard = _neck(p, n, _Cfg(), _pcb())
    gap = 0.188 - 0.0889
    check('t_a_properly_spaced_pair_is_not_refused',
          not hard,
          f'the real trunk (edge gap {gap:.4f}mm >= {_Cfg.clearance}mm) passes')


def t_the_helper_still_necks_what_it_can():
    """#318's original job must survive: a PERPENDICULAR graze still necks."""
    # Parallel and WIDE (0.3mm), 0.28mm apart: too close at that width, but
    # narrowing genuinely fixes it -- unlike the collinear case, where the gap
    # is end-to-end and width barely moves it. Chosen so the result clears by a
    # margin at any plausible fab floor, rather than sitting on the boundary.
    p = [_seg(140.0, 100.000, 145.0, 100.000, 1, w=0.3)]
    n = [_seg(140.0, 100.280, 145.0, 100.280, 2, w=0.3)]
    necked, hard = _neck(p, n, _Cfg(), _pcb())
    check('t_the_helper_still_necks_what_it_can',
          necked > 0,
          f'{necked} segment(s) necked on a parallel graze (#318 behaviour kept)')


def _real_cfg(clearance, pair_class=None):
    """The REAL GridRouteConfig, not a stub: the point of these rows is the
    production `obstacle_clearance` semantics, which a stub would restate."""
    from routing_config import GridRouteConfig
    c = GridRouteConfig(clearance=clearance, track_width=0.0889)
    if pair_class is not None:
        c.net_clearances = {1: pair_class, 2: pair_class}
    return c


def t_a_wider_pair_class_is_honoured():
    """P and N are DIFFERENT NETS, so the gap between them is KiCad's pairwise
    max(classP, classN) -- not the run's global floor.

    This used to test the bare `config.clearance`. Net classes only ever WIDEN
    (`get_net_clearance`), so a pair whose class asks for more than the run's
    floor was measured too leniently and could ship copper KiCad then flags.
    Geometry here: centres 0.320mm apart at 0.2mm width -> 0.120mm of copper
    gap, which clears a 0.10mm floor and does NOT clear a 0.20mm class.
    """
    p = [_seg(1.0, 1.0, 5.0, 1.0, 1, w=0.2)]
    n = [_seg(1.0, 1.32, 5.0, 1.32, 2, w=0.2)]
    _, hard = _neck(p, n, _real_cfg(0.10, pair_class=0.20), _pcb())
    check('t_a_wider_pair_class_is_honoured',
          bool(hard),
          f'0.120mm of copper gap is refused against a 0.20mm pair class '
          f'({len(hard)} hard violation(s))')


def t_it_is_inert_without_netclasses():
    """The other half, and the reason this is a safe tightening: on a board
    that declares no class, `obstacle_clearance` is documented as byte-identical
    to `config.clearance`, so the SAME geometry must still pass. A fix that
    also rejected this would be a regression dressed as a correction."""
    p = [_seg(1.0, 1.0, 5.0, 1.0, 1, w=0.2)]
    n = [_seg(1.0, 1.32, 5.0, 1.32, 2, w=0.2)]
    _, hard = _neck(p, n, _real_cfg(0.10), _pcb())
    check('t_it_is_inert_without_netclasses',
          not hard,
          '0.120mm of copper gap still passes a 0.10mm floor with no classes')


def main():
    t_collinear_members_are_reported_hard()
    t_a_properly_spaced_pair_is_not_refused()
    t_the_helper_still_necks_what_it_can()
    t_a_wider_pair_class_is_honoured()
    t_it_is_inert_without_netclasses()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
