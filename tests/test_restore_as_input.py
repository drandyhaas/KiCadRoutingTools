#!/usr/bin/env python3
"""An earlier step's net, ripped and put back, stands as the input had it.

A route run asked for a few nets may rip a pre-existing net that blocks one
of them (custody-backed: the original copper goes back where the reroute
fails). Where that net was INCOMPLETE in the input -- a fan-out's escape, a
ball's stub and its run to a free end, left for a later step to finish --
two things went wrong after the restore:

  1. `add_route_to_pcb_data` ran its per-commit dead-end prune on the restore
     as on a fresh route. An escape is a dead end by design, so "original
     copper restored" put back the ball's stub and dropped its run.
  2. The restored net is "not connected", as it came. The run read that as a
     casualty of its own and enrolled the net for the final reconciliation:
     a net outside --nets routed with full router power, whose laps ripped
     its neighbours in turn, and whose cleanup -- every ripped net is in its
     scope -- trimmed the escapes put back as dead ends. A run asked for
     three nets left three array balls with no copper at all.

Checks:
  1. a restore of an `is_existing_route` payload puts back every segment of
     an escape; the same copper committed as a fresh route still loses its
     dead-end run (the prune is unchanged for this run's own routes), and so
     does a restore of a route this run laid;
  2. `route.restored_as_input` names a ripped net as the input had it only
     when it is outside the requested nets, its original copper stands, no
     rescue copper was joined to it, and the board's copper of the net is
     exactly the input's;
  3. batch_route asks it, and takes those nets out of the retry list, the
     end-of-run audit's disturbed set and the cleanup's scope.

    python3 tests/test_restore_as_input.py
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools')):
    if p not in sys.path:
        sys.path.insert(0, p)

from kicad_parser import PCBData, BoardInfo, Net, Pad, Segment, Via   # noqa: E402
from pcb_modification import add_route_to_pcb_data                    # noqa: E402
from route import restored_as_input                                   # noqa: E402

FAILS = []
ESC, ASKED = 7, 8


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        FAILS.append(label)


def pad(ref, x, y, net, name):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y, local_x=0.0, local_y=0.0, size_x=0.25,
               size_y=0.25, shape='circle', layers=['F.Cu'], net_id=net, net_name=name, pad_type='smd')


def board():
    """one net with a ball and a far pad nothing joins yet"""
    pcb = PCBData(board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=(0.0, 0.0, 20.0, 20.0)),
                  nets={ESC: Net(net_id=ESC, name='ESC'), ASKED: Net(net_id=ASKED, name='ASKED')},
                  footprints={}, vias=[], segments=[], pads_by_net={}, zones=[])
    pcb.pads_by_net = {ESC: [pad('U1', 5.0, 5.0, ESC, 'ESC'), pad('R1', 15.0, 15.0, ESC, 'ESC')]}
    return pcb


def seg(a, b, net=ESC, layer='F.Cu'):
    return Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=0.1, layer=layer, net_id=net)


def escape():
    """a ball's stub to the gap beside it and a run to a free end"""
    return [seg((5.0, 5.0), (5.25, 5.25)), seg((5.25, 5.25), (5.25, 9.0))]


def commit(kind):
    pcb = board()
    res = {'net_id': ESC, 'new_segments': escape(), 'new_vias': []}
    if kind == 'input restored':
        res['is_existing_route'] = True
    add_route_to_pcb_data(pcb, res, trace_event='route' if kind == 'fresh route' else 'restore')
    return len(pcb.segments), len(res['new_segments'])


def check_restore():
    print('1. the restore of an earlier step\'s escape is whole')
    on, kept = commit('input restored')
    check('both segments of the escape are back on the board', on == 2, f'{on} of 2')
    check('...and in the result the writer reads', kept == 2, f'{kept} of 2')
    on, _ = commit('fresh route')
    check('the same copper as a fresh route loses its dead-end run (the prune stands)', on == 1, f'{on} on the board')
    on, _ = commit('own route restored')
    check('...and so does a restore of a route this run laid', on == 1, f'{on} on the board')


def check_rule():
    print('2. restored_as_input: the input\'s state, measured')
    orig = escape()
    entry = {'net_id': ESC, 'new_segments': orig, 'new_vias': [], 'is_existing_route': True}

    def ask(segments=None, requested=(), results=(), result=entry, vias=(), orig_vias=()):
        pcb = board()
        pcb.segments = list(orig if segments is None else segments)
        pcb.vias = list(vias)
        return restored_as_input(pcb, {ESC: 'ESC'}, {} if result is None else {ESC: result}, list(results),
                                 set(requested), {ESC: orig}, {ESC: list(orig_vias)})

    check('its original copper back, outside the requested nets: as the input had it', ask() == {ESC})
    check('a net the run was asked to route is its own', ask(requested={ESC}) == set())
    check('one segment short of the input: not as it came', ask(segments=orig[:1]) == set())
    check('a segment more than the input: not as it came',
          ask(segments=orig + [seg((5.25, 9.0), (6.0, 9.0))]) == set())
    via = Via(x=5.25, y=9.0, size=0.45, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=ESC)
    check('a via the input did not have: not as it came', ask(vias=[via]) == set())
    check('...and one it had: as it came', ask(vias=[via], orig_vias=[via]) == {ESC})
    check('rescue copper joined to the restored result: not as it came',
          ask(results=[{'net_id': ESC, 'new_segments': [], 'extends': entry}]) == set())
    check('a reroute standing in its place: not as it came',
          ask(result={'net_id': ESC, 'new_segments': orig, 'new_vias': []}) == set())
    check('no result at all (left ripped): not as it came', ask(result=None) == set())


def check_wiring():
    print('3. batch_route acts on it')
    src = open(os.path.join(ROOT, 'py_router', 'route.py'), encoding='utf-8').read()
    tree = ast.parse(src)
    br = next(f for f in tree.body if isinstance(f, ast.FunctionDef) and f.name == 'batch_route')
    calls = [n for n in ast.walk(br) if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'restored_as_input']
    check('batch_route calls restored_as_input', len(calls) == 1, f'{len(calls)} call(s)')
    target = next((n.targets[0].id for n in ast.walk(br) if isinstance(n, ast.Assign) and n.value in calls
                   and isinstance(n.targets[0], ast.Name)), None)
    check('...and keeps its answer', target is not None)
    names = lambda node: {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}   # noqa: E731
    swept = [n for n in ast.walk(br) if isinstance(n, ast.AugAssign) and isinstance(n.op, ast.Sub)
             and getattr(n.target, 'id', '') == 'sweep_scope_ids' and target in names(n.value)]
    check('those nets leave the cleanup\'s scope', len(swept) == 1)
    audit = [n for n in ast.walk(br) if isinstance(n, ast.Assign)
             and getattr(n.targets[0], 'id', '') == '_pe_disturbed_audit']
    minus = [b for a_ in audit for b in ast.walk(a_.value) if isinstance(b, ast.BinOp) and isinstance(b.op, ast.Sub)
             and target in names(b.right)]
    check('...and the audit\'s disturbed set', len(audit) == 1 and len(minus) == 1)
    # (the retry list: the loop that fills it passes over such a net before it appends)
    loops = [n for n in ast.walk(br) if isinstance(n, ast.For) and any(
        isinstance(c, ast.Call) and getattr(c.func, 'attr', '') == 'append'
        and getattr(c.func.value, 'id', '') == '_victim_retry_names' for c in ast.walk(n))]
    skips = [i for lp in loops for i in ast.walk(lp) if isinstance(i, ast.If) and target in names(i.test)
             and any(isinstance(s, ast.Continue) for s in i.body)]
    check('...and the retry list', len(loops) == 1 and len(skips) == 1)


def main():
    check_restore()
    check_rule()
    check_wiring()
    if FAILS:
        print(f'\n{len(FAILS)} FAILED: ' + '; '.join(FAILS))
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
