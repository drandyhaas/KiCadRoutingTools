#!/usr/bin/env python3
"""Terminal escalation keeps a net's per-layer widths (#2036).

_size_escalation_ladder dropped the net's net_layer_widths entry and marched
ONE width taken from layers[0]. A net whose layer-scoped width rule fixes it
at 0.18 mm outer / 0.102 mm inner (an impedance class, #1983) was then
escalated at 0.18 mm on every layer: its recovered copper broke the inner
rule's max. The ladder now marches each layer from the width the net draws
there, so no rung is ever wider than that layer's own width.

Fixture (2 layers): net P (class via 0.8/0.4) from an F.Cu pad to a B.Cu pad,
per-layer widths F.Cu 0.18 / B.Cu 0.102. Ladder rows check every rung; the
functional row routes the first rung and reads the B.Cu copper widths.
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))

FAILS = []
OUTER, INNER = 0.18, 0.102
P = 1


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _board(td):
    from constraint_agreement import write_board
    path = os.path.join(td, 'w.kicad_pcb')
    write_board(path, segments=[], footprints=[
        {'ref': 'U1', 'x': 5, 'y': 20, 'net_id': P, 'net_name': 'A', 'layer': 'F.Cu'},
        {'ref': 'U2', 'x': 35, 'y': 20, 'net_id': P, 'net_name': 'A', 'layer': 'B.Cu'}])
    return path


def _cfg():
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=OUTER, clearance=0.15,
                           grid_step=0.1, via_size=0.45, via_drill=0.2,
                           net_via_sizes={P: (0.8, 0.4)},
                           net_layer_widths={P: {'F.Cu': OUTER, 'B.Cu': INNER}},
                           max_iterations=200000)


def _route(pcb, cfg):
    from obstacle_map import build_base_obstacle_map
    from single_ended_routing import route_net_with_obstacles
    obs = build_base_obstacle_map(pcb, cfg, [P])
    with contextlib.redirect_stdout(io.StringIO()):
        res = route_net_with_obstacles(pcb, P, cfg, obs)
    return None if (not res or res.get('failed')) else res


def main():
    try:
        import grid_router  # noqa: F401
    except ImportError:
        print('SKIP: grid_router not importable')
        return 0
    from kicad_parser import parse_kicad_pcb
    from fab_tiers import may_narrow
    import net_rescue
    if not may_narrow():
        print('SKIP: escalation is off in this environment (no rungs exist)')
        return 0
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(_board(td))
        cfg = _cfg()
        with contextlib.redirect_stdout(io.StringIO()):
            rungs = net_rescue._size_escalation_ladder(cfg, pcb, P)
        print('-- the terminal escalation size ladder')
        for r in rungs:
            print(f'     rung: F.Cu {r.get_net_track_width(P, "F.Cu"):.4g} '
                  f'B.Cu {r.get_net_track_width(P, "B.Cu"):.4g} via {r.net_via(P)}')
        check('rungs exist', bool(rungs), f'{len(rungs)} rungs')
        b = [round(r.get_net_track_width(P, 'B.Cu'), 4) for r in rungs]
        f = [round(r.get_net_track_width(P, 'F.Cu'), 4) for r in rungs]
        check('no rung is wider than the net\'s B.Cu width',
              all(w <= INNER + 1e-9 for w in b), str(b))
        check('no rung is wider than the net\'s F.Cu width',
              all(w <= OUTER + 1e-9 for w in f), str(f))
        check('the marched width applies per layer (B.Cu below F.Cu on every rung)',
              all(wb < wf - 1e-9 for wb, wf in zip(b, f)), f'{b} vs {f}')
        check('every rung keeps the net in net_layer_widths',
              all(r.net_layer_widths.get(P) is not None for r in rungs))
        _label = getattr(net_rescue, '_rung_width_label', None)
        lab = _label(rungs[0], P) if (_label and rungs) else ''
        check('the rung log names both widths', '/' in lab, lab)

        print('-- routing the first rung')
        if not rungs:
            return _finish()
        res = _route(pcb, rungs[0])
        segs = (res or {}).get('new_segments', [])
        bw = sorted({round(s.width, 4) for s in segs if s.layer == 'B.Cu'})
        check('first rung routes, crossing to B.Cu',
              bool(res) and bool(bw), f'B.Cu widths {bw}')
        check('B.Cu copper is no wider than the B.Cu width',
              all(w <= INNER + 1e-9 for w in bw), str(bw))
    return _finish()


def _finish():
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
