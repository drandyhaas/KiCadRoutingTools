"""A stub layer-swap via is SHIPPED copper, so the sync must keep it (#874).

`sync_pcb_data_segments` reconciles pcb_data with the results so the obstacle
map and the written file say the same thing about the board. Its via half
(#874) removes a via that is "neither an input-file original nor carried by any
current result" -- the shape a meander rebuild leaves behind.

That predicate is too wide by exactly one channel. A stub layer-swap pad via is
appended straight to `pcb_data.vias` by `stub_layer_switching`, rides the run in
`all_swap_vias`, and is WRITTEN by `output_writer`'s own `all_swap_vias`
channel -- it appears in no result's `new_vias` at any point, so it was never
superseded by anything. Dropping it deletes real copper from the board model
while the file keeps it.

MEASURED on ecp5_mini's route_diff step, which is why this file exists:

    Sync pcb_data vias: 124 -> 116 (kept originals)      8 dropped
    Diff-pair Dead-end sweep: trimmed 68 dead-end segment(s)
    MEMBER AUDIT MISMATCH: /PH15 reported 'coupled' ... -> 'incomplete'
    MEMBER AUDIT MISMATCH: /PA26 reported 'coupled' ... -> 'incomplete'

Four of the eight were the layer-switch vias of /PH15+/- and /PA26+/-. With
their barrels gone from pcb_data every leg hanging off one looked unsupported,
so the dead-end sweep running immediately after trimmed the legs away: /PH15+
went 22 segments -> 1. Both pairs routed IDENTICALLY (same iteration count,
same "DIRECT HYBRID" line) in both arms -- nothing about the routing changed,
only the bookkeeping that decides what survives it. End to end the board went
from 0 unconnected / 0 DRC to 3 unconnected / 4 DRC.

route.py already takes this exact union for `run_post_route_cleanup`'s orphan
sweep, with the same reasoning recorded there; the sync point is the site that
did not get it.

The gate is written as a THREE-WAY population, not as "the swap via survives",
because keeping everything passes that row: #874's own removal must still fire
on the same call that keeps the swap via.
"""
import ast
import contextlib
import io
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _via(x, net_id):
    from kicad_parser import Via
    return Via(x=x, y=10.0, size=0.5, drill=0.25,
               layers=['F.Cu', 'B.Cu'], net_id=net_id)


def _pcb(vias):
    from kicad_parser import PCBData, BoardInfo
    return PCBData(board_info=BoardInfo(layers={0: 'F.Cu'},
                                        copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(0.0, 0.0, 50.0, 50.0),
                                        stackup=[]),
                   nets={}, footprints={}, vias=list(vias), segments=[],
                   pads_by_net={})


def t_three_populations_on_one_call():
    """Original kept, swap via kept, superseded dropped -- in ONE sync.

    All three vias are on the SAME routed net, so nothing here is decided by
    net membership: the only thing that separates them is which keep-set they
    are in, which is exactly what the fix changes.
    """
    from routing_common import sync_pcb_data_segments
    original = _via(10.0, 7)      # came from the input file
    swap = _via(11.0, 7)          # all_swap_vias -- written, in no result
    superseded = _via(12.0, 7)    # a rebuilt result's discarded barrel
    carried = _via(13.0, 7)       # the rebuilt result's CURRENT barrel
    pcb = _pcb([original, swap, superseded, carried])
    results = {7: {'new_segments': [], 'new_vias': [carried]}}
    # What both callers now pass: the originals UNION the run's swap vias.
    keep = {id(original), id(swap)}
    with contextlib.redirect_stdout(io.StringIO()):
        sync_pcb_data_segments(pcb, results, set(), None, None,
                               original_via_ids=keep)
    kept = {id(v) for v in pcb.vias}
    check('t_the_swap_via_survives_the_sync',
          id(swap) in kept,
          'a via written through the all_swap_vias channel is still on the board')
    check('t_the_input_original_survives',
          id(original) in kept, 'input-file barrel kept')
    check('t_the_result_barrel_survives',
          id(carried) in kept, 'the current result via kept')
    # NON-VACUITY: #874's removal must still fire on this very call, or the row
    # above is passing because the sync kept everything it was handed.
    check('t_874_still_removes_a_superseded_via',
          id(superseded) not in kept,
          'the discarded barrel is gone (#874 intent intact)')


def t_without_the_union_the_swap_via_is_lost():
    """The change detector: the OLD argument really did drop it.

    Same call, same fixture, only the union removed. If this row starts
    passing, the predicate has stopped distinguishing the two and the gate
    above has become decorative.
    """
    from routing_common import sync_pcb_data_segments
    original = _via(10.0, 7)
    swap = _via(11.0, 7)
    pcb = _pcb([original, swap])
    results = {7: {'new_segments': [], 'new_vias': []}}
    with contextlib.redirect_stdout(io.StringIO()):
        sync_pcb_data_segments(pcb, results, set(), None, None,
                               original_via_ids={id(original)})
    kept = {id(v) for v in pcb.vias}
    check('t_without_the_union_the_swap_via_is_lost',
          id(swap) not in kept and id(original) in kept,
          'omitting all_swap_vias drops it -- so the union is load-bearing')


def _sync_calls(path):
    """Every `sync_pcb_data_segments(...)` call node in a source file."""
    with open(path) as f:
        tree = ast.parse(f.read())
    out = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == 'sync_pcb_data_segments'):
            out.append(node)
    return out


def t_both_callers_pass_the_union():
    """Wiring, read off the AST rather than off a grep of the whole file.

    A `all_swap_vias` mentioned ANYWHERE in route.py proves nothing -- it is
    mentioned a dozen times. What matters is that the name reaches THIS call's
    `original_via_ids` argument.
    """
    root = os.path.join(os.path.dirname(__file__), '..', 'py_router')
    for fname in ('route.py', 'route_diff.py'):
        path = os.path.join(root, fname)
        calls = _sync_calls(path)
        check(f't_{fname.replace(".", "_")}_calls_the_sync',
              len(calls) == 1,
              f'{len(calls)} sync_pcb_data_segments call(s) found')
        ok = False
        for call in calls:
            for kw in call.keywords:
                if kw.arg != 'original_via_ids':
                    continue
                names = {n.id for n in ast.walk(kw.value)
                         if isinstance(n, ast.Name)}
                if 'all_swap_vias' in names:
                    ok = True
        check(f't_{fname.replace(".", "_")}_unions_all_swap_vias',
              ok,
              'all_swap_vias reaches original_via_ids at the sync call site')


def t_the_writer_really_ships_that_channel():
    """Non-vacuity for the whole premise: output_writer emits all_swap_vias.

    If it did not, keeping those vias in pcb_data would be the bug rather than
    the fix -- the board model would then carry copper the file does not.
    """
    root = os.path.join(os.path.dirname(__file__), '..', 'py_router')
    with open(os.path.join(root, 'output_writer.py')) as f:
        src = f.read()
    tree = ast.parse(src)
    gen = next((n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef)
                and n.name == '_generate_routing_text'), None)
    args = [a.arg for a in gen.args.args] if gen else []
    check('t_the_writer_really_ships_that_channel',
          gen is not None and 'all_swap_vias' in args,
          '_generate_routing_text takes all_swap_vias'
          + ('' if gen else ' -- function not found'))


def main():
    t_three_populations_on_one_call()
    t_without_the_union_the_swap_via_is_lost()
    t_both_callers_pass_the_union()
    t_the_writer_really_ships_that_channel()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
