#!/usr/bin/env python3
"""The stub-debris trim must not leave a GHOST via (#805).

MISLABEL, corrected here: this was committed as "#796" (176f9328, already
pushed, so its message cannot be rewritten). 796 is a MERGED PR -- "#702: the
quench refuses a move that breaks a declared floorplan claim" -- and has nothing
to do with this. The number was inferred from the gap in the open-ISSUE list
without checking pull requests, which share the numbering. The real issue is
#805 (obstacle bookkeeping: copper placed or freed without a matching map
update).

`f30f44d5` frees unused stub branches and dangling vias the moment a route
commits, so the cells are routable by the very next net. The freeing was only
half-accounted. The output writer emits vias from TWO sources -- each result's
`new_vias`, AND `state.all_swap_vias` (stub layer-swap vias) -- and a swap via
also lives in `pcb_data.vias`. Dropping it from `pcb_data` alone released its
obstacle cells while the writer kept emitting it, so the via became a GHOST:
absent from the board model, present in the written file. A later net routed
straight through the vacated row and the via reappeared beside the new track.

Measured on tinytapeout_qfn's `step5_route`, from an input both engines produce
identically: 10 of 34 swap vias were freed and every one was still written --
12 Via<->Seg clearance violations (one 0.431 mm, between foreign nets) on a
board that graded 0 before this pass existed. `KICAD_STUB_DEBRIS_TRIM=0` cleared
all 12. The fix keeps the trim ON (14 trims still fire, 229 vias against the
trim-off 203) and takes the violations to 0.

The `vias_to_remove` writer channel cannot cover this: it strips vias out of the
verbatim copy of the INPUT file, so it never reaches copper this run appended.

WHY IT IS SHAPED THIS WAY. The original shipped behind three green gates
(`test_734_reconcile_scope`, `test_cleanup_pipeline`, `test_keep_input_copper`,
13/13) and not one DRC-grades real copper -- the single check a pass that
changes WHICH CELLS ARE FREE most needs. A DRC assertion would need a 4-minute
route, so this asserts the INVARIANT those violations were a symptom of, in
milliseconds: after the trim, no via may be absent from `pcb_data.vias` while a
writer list still holds it. It is BEHAVIOURAL -- it runs the real trim on a real
board and inspects the real lists -- and it carries its own negative control:
the same scenario with the swap list NOT handed over must produce the ghost, or
the test proves nothing.

    python3 tests/test_805_trim_ghost_vias.py
"""
import os
import sys

RUN_ALL_TIMEOUT = 120
#: milliseconds; runs no chain and starts no child process.
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

FAILURES = []


def check(name, ok, detail=''):
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{('  -- ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(name)


class _Cfg:
    clearance = 0.2
    track_width = 0.2
    via_size = 0.5
    via_drill = 0.25
    verbose = False
    debug_lines = None
    _keep_input_copper = False

    def obstacle_clearance(self, net_id):
        return 0.2


def _scenario():
    """A net whose route leaves an unused stub ending in a DANGLING via, with
    that via in both the board model and the writer's swap list."""
    from kicad_parser import parse_kicad_pcb, Segment, Via
    board = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    pcb = parse_kicad_pcb(board)
    nid = next(n for n, p in pcb.pads_by_net.items() if n and len(p) >= 2)
    pads = pcb.pads_by_net[nid]
    pcb.segments = [s for s in pcb.segments if s.net_id != nid]
    pcb.vias = [v for v in pcb.vias if v.net_id != nid]
    ax, ay = pads[0].global_x, pads[0].global_y
    bx, by = pads[1].global_x, pads[1].global_y
    mk = lambda x1, y1, x2, y2: Segment(  # noqa: E731
        start_x=x1, start_y=y1, end_x=x2, end_y=y2,
        width=0.2, layer='F.Cu', net_id=nid)
    main, stub = mk(ax, ay, bx, by), mk(bx, by, bx + 0.6, by + 0.6)
    via = Via(x=bx + 0.6, y=by + 0.6, size=0.5, drill=0.25,
              layers=['F.Cu', 'B.Cu'], net_id=nid)
    pcb.segments += [main, stub]
    pcb.vias.append(via)
    swap = [via]                      # the writer's OTHER via source
    result = {'new_segments': [main, stub], 'new_vias': [via]}
    return pcb, nid, result, swap, via


def test_negative_control_the_ghost_is_reproducible():
    """Without the swap list handed over, the ghost MUST appear -- otherwise the
    check below passes for a reason unrelated to the fix."""
    from pcb_modification import trim_net_stub_debris
    pcb, nid, result, swap, via = _scenario()
    _s, v_rm = trim_net_stub_debris(pcb, nid, result, _Cfg())   # swap_vias omitted
    gone_from_model = not any(v is via for v in pcb.vias)
    still_writable = any(v is via for v in swap)
    check('the trim removes the dangling via at all', v_rm >= 1, f'{v_rm} via(s)')
    check('CONTROL: omitting the swap list reproduces the ghost',
          gone_from_model and still_writable,
          f'gone_from_model={gone_from_model} still_writable={still_writable}')


def test_the_swap_list_is_stripped():
    """The fix: a freed via leaves the board model and every writer list."""
    from pcb_modification import trim_net_stub_debris
    pcb, nid, result, swap, via = _scenario()
    _s, v_rm = trim_net_stub_debris(pcb, nid, result, _Cfg(), swap_vias=swap)
    gone_from_model = not any(v is via for v in pcb.vias)
    gone_from_swap = not any(v is via for v in swap)
    check('the via leaves pcb_data.vias', gone_from_model)
    check('the via leaves the swap list', gone_from_swap)
    check('NO GHOST (freed and written can no longer disagree)',
          not (gone_from_model and not gone_from_swap))


def test_the_swap_list_is_filtered_in_place():
    """The writer holds the SAME list object, so rebinding would strip nothing."""
    from pcb_modification import trim_net_stub_debris
    pcb, nid, result, swap, via = _scenario()
    held_by_writer = swap                       # the writer's reference
    trim_net_stub_debris(pcb, nid, result, _Cfg(), swap_vias=swap)
    check('the caller\'s own list object was mutated',
          not any(v is via for v in held_by_writer),
          'a rebind inside the trim would leave this list untouched')


def main():
    print('--- #805 the stub-debris trim must not leave ghost vias')
    test_negative_control_the_ghost_is_reproducible()
    test_the_swap_list_is_stripped()
    test_the_swap_list_is_filtered_in_place()
    if FAILURES:
        print(f"\n{len(FAILURES)} failure(s): {', '.join(FAILURES)}")
        return 1
    print('\ntest_805_trim_ghost_vias: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
