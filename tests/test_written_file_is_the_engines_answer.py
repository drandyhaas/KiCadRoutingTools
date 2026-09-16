"""The shipped file must carry exactly the copper pcb_data ends with (#220/#84).

Both front-ends assemble their output the same way: start from something that
ALREADY holds the board's original copper, then apply the run's additions and a
list of REMOVALS -- the CLI copies the input file verbatim and strips
`segments_to_remove` (#220), the GUI applies the same list to the live pcbnew
board (#84). So input copper only leaves if some pass REMEMBERED to put it in
that list. That is a coverage argument, and coverage arguments fail silently.

Measured on cparti_fpga's retry step: the dead-end sweep removed SRAM_D0's B.Cu
diagonal from pcb_data, octolinear smoothing then routed SRAM_A4 through the
vacated corridor -- CORRECTLY, its clearance check saw an empty corridor, 0
violations across 602 emitted segments -- and the diagonal shipped anyway,
crossing it. 8 of 333 nets diverged, SRAM_D0 by 19 segments, and every net in
that board's remaining DRC was in the diverged set. Two structural reasons the
list misses copper, both live on that board:

  * IDENTITY -- input copper is matched by id(), and a rip -> restore ->
    cleanup cycle can leave a DIFFERENT object in pcb_data;
  * NESTING -- each batch_route owns its own strip list, and route.py's plane
    finalize runs a NESTED batch_route whose removals need not reach the outer
    writer.

`file_only_copper` compares the ARTIFACT against pcb_data instead, which is
representation-independent: it does not care which channel missed the removal,
whether identity survived, or how deep the nesting went. pcb_data is the
engine's own final answer -- every pass maintains it, and the DRC and
connectivity route.py reports are computed from it -- so copper on the target
that pcb_data lacks was never part of that answer.

THE ROW THAT MATTERS MOST HERE IS THE INERT ONE. A comparison that finds
"leaks" on an agreeing file would have the writer deleting real copper on every
board, which is far worse than the bug. Hence t_an_agreeing_file_reports_nothing
and t_stacked_duplicates_are_not_a_leak -- the multiset case, where a net
legitimately carries two identical segments and a set-based comparison reads
one of them as a leak.

Run with:  python3 tests/test_written_file_is_the_engines_answer.py
"""
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

from kicad_parser import (BoardInfo, Net, PCBData, Segment, Via,  # noqa: E402
                          parse_kicad_pcb)

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


HDR = ('(kicad_pcb (version 20241229) (generator "pcbnew")\n'
       '\t(net 0 "")\n\t(net 7 "/SRAM_D0")\n\t(net 8 "/SRAM_A4")\n')


def _seg(x0, y0, x1, y1, net_id, layer='B.Cu'):
    return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=0.2,
                   layer=layer, net_id=net_id)


def _via(x, y, net_id):
    return Via(x=x, y=y, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'],
               net_id=net_id, uuid='')


def _write(path, segs, vias):
    """A board file holding exactly these objects, in the numeric dialect."""
    body = []
    for s in segs:
        body.append(f'\t(segment (start {s.start_x} {s.start_y}) '
                    f'(end {s.end_x} {s.end_y}) (width {s.width}) '
                    f'(layer "{s.layer}") (net {s.net_id}))')
    for v in vias:
        body.append(f'\t(via (at {v.x} {v.y}) (size {v.size}) '
                    f'(drill {v.drill}) (layers "F.Cu" "B.Cu") '
                    f'(net {v.net_id}))')
    open(path, 'w').write(HDR + '\n'.join(body) + '\n)\n')


def _pcb(segs, vias):
    return PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                             copper_layers=['F.Cu', 'B.Cu'],
                             board_bounds=(0.0, 0.0, 50.0, 50.0), stackup=[]),
        nets={7: Net(net_id=7, name='/SRAM_D0'),
              8: Net(net_id=8, name='/SRAM_A4')},
        footprints={}, vias=list(vias), segments=list(segs), pads_by_net={})


DIAGONAL = _seg(10.0, 10.0, 13.0, 13.0, 7)      # SRAM_D0's B.Cu diagonal
KEPT = _seg(20.0, 20.0, 24.0, 20.0, 7)
CROSSER = _seg(10.0, 13.0, 13.0, 10.0, 8)       # SRAM_A4 through the corridor
STRAY_VIA = _via(11.0, 11.0, 7)


def _run(file_segs, file_vias, board_segs, board_vias, scope=(7, 8)):
    from cleanup_pipeline import file_only_copper
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, 'out.kicad_pcb')
        _write(p, file_segs, file_vias)
        # Verify the fixture before trusting the result: a comparison whose
        # input did not parse tests nothing.
        parsed = parse_kicad_pcb(p)
        if len(parsed.segments) != len(file_segs) or len(parsed.vias) != len(file_vias):
            return None, (f'BROKEN FIXTURE: wrote {len(file_segs)} seg / '
                          f'{len(file_vias)} via, parsed back '
                          f'{len(parsed.segments)} / {len(parsed.vias)}')
        return file_only_copper(p, _pcb(board_segs, board_vias), scope), ''


def t_an_agreeing_file_reports_nothing():
    """The inert row, and the one that matters most: on a file that matches
    pcb_data the audit must find NOTHING, or the writer deletes real copper on
    every board."""
    got, err = _run([DIAGONAL, KEPT, CROSSER], [STRAY_VIA],
                    [DIAGONAL, KEPT, CROSSER], [STRAY_VIA])
    if got is None:
        return check('t_an_agreeing_file_reports_nothing', False, err)
    segs, vias = got
    check('t_an_agreeing_file_reports_nothing', segs == [] and vias == [],
          f'{len(segs)} segment(s), {len(vias)} via(s) reported')


def t_the_unreported_removal_is_found():
    """The bug: the sweep removed the diagonal from pcb_data but no pass
    recorded a strip, so the input file's copy shipped."""
    got, err = _run([DIAGONAL, KEPT, CROSSER], [],
                    [KEPT, CROSSER], [])          # pcb_data no longer has it
    if got is None:
        return check('t_the_unreported_removal_is_found', False, err)
    segs, vias = got
    check('t_the_unreported_removal_is_found',
          len(segs) == 1 and abs(segs[0].start_x - 10.0) < 1e-6
          and abs(segs[0].end_x - 13.0) < 1e-6 and not vias,
          f'the diagonal alone is reported ({len(segs)} seg, {len(vias)} via)')


def t_a_via_removal_is_found_too():
    got, err = _run([KEPT], [STRAY_VIA], [KEPT], [])
    if got is None:
        return check('t_a_via_removal_is_found_too', False, err)
    segs, vias = got
    check('t_a_via_removal_is_found_too', not segs and len(vias) == 1,
          f'{len(segs)} seg, {len(vias)} via reported')


def t_stacked_duplicates_are_not_a_leak():
    """MULTISET, not set. A net legitimately carrying two identical segments
    must not have one read as a leak because the other matched."""
    twin = _seg(20.0, 20.0, 24.0, 20.0, 7)        # same geometry as KEPT
    got, err = _run([KEPT, twin], [], [KEPT, twin], [])
    if got is None:
        return check('t_stacked_duplicates_are_not_a_leak', False, err)
    segs, vias = got
    check('t_stacked_duplicates_are_not_a_leak', not segs and not vias,
          f'{len(segs)} segment(s) reported for two identical same-net segments')
    # ...and one of a stacked pair genuinely going missing IS still a leak.
    got2, err2 = _run([KEPT, twin], [], [KEPT], [])
    if got2 is None:
        return check('t_one_of_a_stacked_pair_is_still_a_leak', False, err2)
    check('t_one_of_a_stacked_pair_is_still_a_leak', len(got2[0]) == 1,
          f'{len(got2[0])} of the two reported')


def t_identity_is_not_used():
    """The whole point: the comparison is by GEOMETRY, so copper that survived
    a rip/restore as a DIFFERENT object still matches. Rebuilt objects, equal
    geometry, must report nothing."""
    rebuilt = [_seg(s.start_x, s.start_y, s.end_x, s.end_y, s.net_id, s.layer)
               for s in (DIAGONAL, KEPT, CROSSER)]
    got, err = _run([DIAGONAL, KEPT, CROSSER], [], rebuilt, [])
    if got is None:
        return check('t_identity_is_not_used', False, err)
    segs, vias = got
    check('t_identity_is_not_used', not segs and not vias,
          f'{len(segs)} reported though every object is a distinct instance')


def t_out_of_scope_nets_are_untouched():
    """Scope discipline: a net this run never touched must never be stripped,
    however much the file and pcb_data disagree about it."""
    # /SRAM_D0 (net 7) diverges by the whole diagonal; /SRAM_A4 (net 8) agrees.
    # With only net 8 in scope, nothing may be reported.
    got, err = _run([DIAGONAL, KEPT, CROSSER], [], [KEPT, CROSSER], [],
                    scope=(8,))
    if got is None:
        return check('t_out_of_scope_nets_are_untouched', False, err)
    segs, vias = got
    check('t_out_of_scope_nets_are_untouched', not segs and not vias,
          f'{len(segs)} reported for /SRAM_D0 while only /SRAM_A4 is in scope')


def t_an_unreadable_target_is_not_a_strip_list():
    """An audit must never break a run: a file that cannot be parsed reports
    nothing, rather than reporting everything as file-only and deleting it."""
    from cleanup_pipeline import file_only_copper
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, 'nope.kicad_pcb')
        segs, vias = file_only_copper(p, _pcb([KEPT], []), (7, 8))
    check('t_an_unreadable_target_is_not_a_strip_list',
          segs == [] and vias == [],
          f'missing file reported {len(segs)} seg / {len(vias)} via')


def t_route_py_acts_on_it():
    """Wiring. The audit printed 'FAILED: 8/333 net(s) differ' on cparti while
    the run shipped the file anyway; reporting is not fixing."""
    import ast
    src = open(os.path.join(ROOT_DIR, 'py_router', 'route.py')).read()
    called = any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == 'file_only_copper'
                 for n in ast.walk(ast.parse(src)))
    check('t_route_py_acts_on_it', called,
          'route.py calls file_only_copper after writing')


def main():
    t_an_agreeing_file_reports_nothing()
    t_the_unreported_removal_is_found()
    t_a_via_removal_is_found_too()
    t_stacked_duplicates_are_not_a_leak()
    t_identity_is_not_used()
    t_out_of_scope_nets_are_untouched()
    t_an_unreadable_target_is_not_a_strip_list()
    t_route_py_acts_on_it()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
