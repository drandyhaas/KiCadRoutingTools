"""Half a pair is not a protected pair -- but a PARTIAL pair is (#521/#906).

#521 protects coupled-pair copper because a later chain step cannot reproduce
it -- P/N geometry, gap, polarity. That reasoning needs BOTH members to have
laid pair copper: if one failed or self-grazed, the survivor's copper is
ordinary single-ended routing the next step can redo, and freezing it only takes
a rip candidate away from whatever still has to get through.

`protection_candidates` decided per NET, off each member's own result dict, so a
survivor was protected on its own and the `is_diff_pair` path never looked at
the partner at all. The decision is now per PAIR: both members must be admitted.

IT DOES NOT REQUIRE THE PAIR TO END CONNECTED, and that is the point of this
file. An earlier cut added `_member_connected` on both members. It could not
tell a pair that FAILED from one that handed a leg off BY DESIGN, and it cost
cparti_fpga its coupled USB copper: /USB/USB_D+ /USB/USB_D- is a 3-terminal
multi-point pair whose route_diff step lays a coupled middle on F.Cu and then
reports "electrically short (< 3.0mm coupled) - deferring leg to single-ended".
The pair is not terminal-to-terminal, so both members lost protection and the
chain's later rip-up passes were free to tear the coupled middle out -- while
the manifest's very next step routes those two nets single-ended, which is the
handoff the engine intended. Coupled copper that exists on the board is exactly
what a later step cannot reproduce, whether or not the pair finished.

SCOPE: what remains is a strict narrowing versus the per-net rule, and only for
a pair with a FAILED or result-less partner. On picodvi -- the board that raised
the question, where protecting /uC_DVI_CK boxes /uC_DVI_D1+ out entirely -- all
three protected pairs are fully connected, so the protected set is IDENTICAL
either way and the board still grades 1/37 incomplete. That board's loss is a
consequence of protecting a pair that DID land, which is #906 working as
designed; it is not this bug.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []


def _candidates(row, results, pcb, pairs):
    """Call with the pair list, or fail the row with the reason.

    A build without the pair argument raises TypeError, which is a BROKEN TEST,
    not a detected regression -- they exit the same way and only one is
    evidence. Absence of the parameter IS the finding.
    """
    from route_diff import protection_candidates
    try:
        return protection_candidates(results, pcb, pairs=pairs)
    except TypeError:
        check(row, False,
              'protection_candidates takes no `pairs` argument -- protection is '
              'still decided per NET, so half a pair can be protected on the '
              'survivor\'s own result')
        return None


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


class _Pair:
    def __init__(self, p, n):
        self.p_net_id, self.n_net_id = p, n


def _pcb(connected_nets):
    """Two nets, each with two pads; `connected_nets` get a segment joining
    theirs, the others do not -- so _member_connected can tell them apart."""
    from kicad_parser import PCBData, BoardInfo, Net, Pad, Segment
    pads, segs = {}, []
    for nid, (x0, name) in enumerate([(0.0, '/D+'), (5.0, '/D-')], start=1):
        a = Pad(pad_number='1', net_id=nid, net_name=name, global_x=x0 + 1.0,
                global_y=1.0, local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5,
                shape='circle', layers=['F.Cu'], drill=0, pad_type='smd',
                component_ref=f'U{nid}')
        b = Pad(pad_number='2', net_id=nid, net_name=name, global_x=x0 + 3.0,
                global_y=1.0, local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5,
                shape='circle', layers=['F.Cu'], drill=0, pad_type='smd',
                component_ref=f'U{nid}')
        pads[nid] = [a, b]
        if nid in connected_nets:
            segs.append(Segment(start_x=a.global_x, start_y=a.global_y,
                                end_x=b.global_x, end_y=b.global_y,
                                width=0.2, layer='F.Cu', net_id=nid))
    return PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu'],
                             board_bounds=(0.0, 0.0, 20.0, 10.0), stackup=[]),
        nets={1: Net(net_id=1, name='/D+', pads=pads[1]),
              2: Net(net_id=2, name='/D-', pads=pads[2])},
        footprints={}, vias=[], segments=segs, pads_by_net=pads)


COUPLED = {'is_diff_pair': True, 'new_segments': [1], 'new_vias': []}


def t_a_landed_pair_is_protected():
    """Non-vacuity: the rule must still protect what #521 is FOR."""
    got = _candidates('t_a_landed_pair_is_protected',
                      {1: COUPLED, 2: COUPLED}, _pcb({1, 2}),
                      [('/D', _Pair(1, 2))])
    if got is None:
        return
    check('t_a_landed_pair_is_protected',
          got == {'/D+': 'diff-pair', '/D-': 'diff-pair'},
          f'both members protected: {sorted(got)}')


def t_a_partially_routed_pair_is_still_protected():
    """A pair that laid coupled copper keeps protection even if it is not yet
    connected terminal to terminal.

    An earlier cut of the per-pair rule required BOTH members to end CONNECTED,
    and that could not tell a pair that failed from one that handed a leg off BY
    DESIGN. cparti_fpga's /USB/USB_D+ /USB/USB_D- is a 3-terminal multi-point
    pair whose route_diff step reports:

        DIRECT HYBRID: coupled middle on F.Cu + 14 leg seg(s)
        Leg 1 via hybrid (coupled middle + single-ended escapes)
          electrically short (< 3.0mm coupled) - deferring leg to single-ended

    The coupled middle is on the board; only the short leg was deferred, and the
    manifest's next step routes those two nets single-ended. Under the
    connectivity requirement both members lost protection and the chain's later
    rip-up passes were free to tear the coupled middle out. Coupled copper is
    exactly what a later step cannot reproduce, so it stays protected.
    """
    got = _candidates('t_a_partially_routed_pair_is_still_protected',
                      {1: COUPLED, 2: COUPLED}, _pcb({1}),
                      [('/D', _Pair(1, 2))])
    if got is None:
        return
    check('t_a_partially_routed_pair_is_still_protected',
          sorted(got) == ['/D+', '/D-'],
          f'both members protected though /D- is not fully connected '
          f'(got {sorted(got)})')


def t_a_pair_whose_partner_failed_is_not_protected():
    """The other half: the partner's result says failed."""
    got = _candidates('t_a_pair_whose_partner_failed_is_not_protected',
                      {1: COUPLED, 2: dict(COUPLED, failed=True)},
                      _pcb({1, 2}), [('/D', _Pair(1, 2))])
    if got is None:
        return
    check('t_a_pair_whose_partner_failed_is_not_protected',
          got == {}, f'nothing protected (got {sorted(got)})')
    # ...and a partner with no result at all.
    got2 = _candidates('t_a_pair_whose_partner_has_no_result_is_not_protected',
                       {1: COUPLED}, _pcb({1, 2}), [('/D', _Pair(1, 2))])
    if got2 is None:
        return
    check('t_a_pair_whose_partner_has_no_result_is_not_protected',
          got2 == {}, f'nothing protected (got {sorted(got2)})')


def t_the_legacy_per_net_path_is_unchanged():
    """The compatibility contract for a caller with no pair list (#906's gate).

    It is also the change detector: on a FAILED partner the per-net rule
    protects the survivor alone and the per-pair rule protects neither, which is
    the whole of what the pair check now narrows.
    """
    from route_diff import protection_candidates
    legacy = protection_candidates({1: COUPLED, 2: COUPLED}, _pcb({1}))
    check('t_the_legacy_per_net_path_is_unchanged',
          legacy == {'/D+': 'diff-pair', '/D-': 'diff-pair'},
          'per-net decision protects both members')
    survivor = protection_candidates({1: COUPLED, 2: dict(COUPLED, failed=True)},
                                     _pcb({1, 2}))
    check('t_the_per_net_path_protects_a_failed_partners_survivor',
          survivor == {'/D+': 'diff-pair'},
          f'per-net protects the survivor alone (got {sorted(survivor)}) -- the '
          f'per-pair rule protects neither, which is the narrowing')


def t_the_caller_passes_the_pair_list():
    """Wiring: route_diff must actually hand the pairs in, or this is inert."""
    import ast
    src = open(os.path.join(os.path.dirname(__file__), '..',
                            'py_router', 'route_diff.py')).read()
    tree = ast.parse(src)
    ok = False
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'protection_candidates'):
            if any(kw.arg == 'pairs' for kw in node.keywords):
                ok = True
    check('t_the_caller_passes_the_pair_list', ok,
          'batch_route_diff_pairs calls protection_candidates(pairs=...)')


def t_it_does_not_require_terminal_connectivity():
    """Asked of the CALL GRAPH, not the text: `_member_connected` still appears
    in a comment explaining why it is not used, so a substring test would read
    that comment as the defect it warns about."""
    import ast
    src = open(os.path.join(os.path.dirname(__file__), '..',
                            'py_router', 'route_diff.py')).read()
    called = set()
    for fn in ast.walk(ast.parse(src)):
        if isinstance(fn, ast.FunctionDef) and fn.name == 'protection_candidates':
            for node in ast.walk(fn):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                    called.add(node.func.id)
                if isinstance(node, ast.ImportFrom):
                    called.update(a.name for a in node.names)
    check('t_it_does_not_require_terminal_connectivity',
          '_member_connected' not in called,
          'protection_candidates does not gate on _member_connected -- a '
          'partially routed pair still carries coupled copper')


def main():
    t_a_landed_pair_is_protected()
    t_a_partially_routed_pair_is_still_protected()
    t_a_pair_whose_partner_failed_is_not_protected()
    t_the_legacy_per_net_path_is_unchanged()
    t_the_caller_passes_the_pair_list()
    t_it_does_not_require_terminal_connectivity()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
