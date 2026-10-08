#!/usr/bin/env python3
"""#590 history-based negotiated congestion (PathFinder) -- unit gates.

The lever: every conflict event bumps the contested CELLS' cost permanently
for the rest of the routing call, so repeatedly-fought-over ground prices
itself out instead of being arbitrated. This asserts the properties that make
it that lever rather than another transient proximity field:

  1. OFF by default -- no rows, no state, nothing recorded (the knob is the
     only way in; there is no CLI flag or GUI control yet).
  2. CUMULATIVE per cell -- ripping the same corridor twice costs twice as
     much, which is the whole point (the ripped-route ghosts are per-NET and
     the C1 filter DELETES them when the victim reroutes).
  3. PERSISTENT -- the field is not a function of who is routed now; nothing
     in the read path can retire it mid-call.
  4. CAP clamps the accumulation (guards the productive-churn regime: a
     fine-pitch BGA escape field must not wall itself off).
  5. The frontier event is weighted lower than a rip, and ONE failure
     analyzed twice in a row is charged once.
  6. Rows compose through the real merge_track_proximity_costs pass into the
     real Rust layer-proximity map, unique per (layer, cell) so sum mode
     counts history exactly once.
  7. The rip hook is really wired into rip_up_reroute.rip_up_net, and the v2
     contest hook into blocking_analysis.analyze_frontier_blocking.
  8. v2 (contest-targeted): at the v2 defaults the v1 whole-footprint rip
     stamp and raw-frontier charge are OFF; the frontier∩blocker contest
     event charges at FULL weight and repeat contests DOUBLE (escalate 1.0).
  9. The growth guard EVICTS lowest-weight cells instead of refusing new
     events (chronological refusal unpriced the endgame conflicts).

    python3 tests/test_590_history_congestion.py
"""
import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(REPO, 'rust_router'))

import numpy as np

import env_knobs
import history_congestion as hc
from obstacle_costs import merge_track_proximity_costs
from routing_config import GridCoord, GridRouteConfig
from grid_router import GridObstacleMap

LAYER_MAP = {'F.Cu': 0, 'B.Cu': 1}


def _cfg():
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'])


def _seg(x1, y1, x2, y2, layer='F.Cu', width=0.2):
    return types.SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                                 layer=layer, width=width)


def _via(x, y, size=0.6):
    return types.SimpleNamespace(x=x, y=y, size=size)


def _result(segs=(), vias=()):
    return {'new_segments': list(segs), 'new_vias': list(vias)}


_ALL_KNOBS = ('KICAD_HISTORY_COST', 'KICAD_HISTORY_CAP', 'KICAD_HISTORY_RADIUS',
              'KICAD_HISTORY_BLOCKED_WEIGHT', 'KICAD_HISTORY_MAX_CELLS',
              'KICAD_HISTORY_RIP_WEIGHT', 'KICAD_HISTORY_ESCALATE')


def _arm(**over):
    """Set the knobs and re-read them (env_knobs caches at import).

    The base values pin the v1 semantics the older sections assert (flat
    accumulation, full-weight rip stamps, 0.25 raw-frontier weight); the v2
    sections override toward the v2 defaults explicitly.
    """
    for k in _ALL_KNOBS:
        os.environ.pop(k, None)
    vals = {'KICAD_HISTORY_COST': '0.5', 'KICAD_HISTORY_CAP': '0',
            'KICAD_HISTORY_RADIUS': '0.0',
            'KICAD_HISTORY_BLOCKED_WEIGHT': '0.25',
            'KICAD_HISTORY_RIP_WEIGHT': '1.0',
            'KICAD_HISTORY_ESCALATE': '0'}
    vals.update({k: str(v) for k, v in over.items()})
    os.environ.update(vals)
    env_knobs.refresh()


def _disarm():
    for k in _ALL_KNOBS:
        os.environ.pop(k, None)
    env_knobs.refresh()


def _cost_at(rows, layer, gx, gy):
    if rows is None:
        return 0
    m = (rows[:, 0] == layer) & (rows[:, 1] == gx) & (rows[:, 2] == gy)
    return int(rows[m][0][3]) if m.any() else 0


def main():
    failures = []
    coord = GridCoord(_cfg().grid_step)

    # ---- 1a. the SHIPPED default is the promoted v1flat_01 arm -------------
    # #590 no longer ships OFF: clearing the env now yields the flat diffuse
    # field. Pin the exact promoted values, so a later edit to env_knobs cannot
    # quietly change what every user routes with.
    _disarm()                       # clears every KICAD_HISTORY_* + refresh
    _shipped = dict(env_knobs.HISTORY)
    for _k, _v in (('cost', 0.1), ('cap', 0.5), ('rip_weight', 1.0),
                   ('blocked_weight', 0.25), ('escalate', 0.0)):
        if _shipped.get(_k) != _v:
            failures.append(f'shipped default {_k}: expected {_v}, got {_shipped.get(_k)}')
    if not hc.history_enabled():
        failures.append('shipped default: history_enabled() must be True')

    # ---- 1b. KICAD_HISTORY_COST=0 still fully disables ---------------------
    # The escape hatch is the whole A/B story now that the mechanism ships on.
    # #625 shipped a knob whose 0 did NOT disable it and had to be reverted --
    # this asserts #590's does.
    os.environ['KICAD_HISTORY_COST'] = '0'
    env_knobs.refresh()
    cfg = _cfg()
    hc.reset_history(cfg)
    hc.record_rip(cfg, _result([_seg(1.0, 1.0, 3.0, 1.0)]), LAYER_MAP)
    hc.record_blocked_frontier(cfg, [(10, 10, 0)])
    hc.record_contested(cfg, [(10, 10, 0)])
    if hc.history_enabled():
        failures.append('COST=0: history_enabled() must be False')
    if hc.history_rows(cfg) is not None:
        failures.append('COST=0: history_rows must be None')
    if getattr(cfg, '_history_cong', 'unset') is not None:
        failures.append('COST=0: reset_history must clear the field')
    _disarm()

    # ---- 2/3. cumulative per cell, and not a function of routed state ------
    _arm()
    cfg = _cfg()
    hc.reset_history(cfg)
    route = _result([_seg(1.0, 1.0, 3.0, 1.0)])
    hc.record_rip(cfg, route, LAYER_MAP)
    rows1 = hc.history_rows(cfg)
    gx, gy = coord.to_grid(2.0, 1.0)
    c1 = _cost_at(rows1, 0, gx, gy)
    if c1 != cfg.cell_cost(0.5):
        failures.append(f'one rip: cell cost {c1} != cell_cost(0.5)={cfg.cell_cost(0.5)}')
    if _cost_at(rows1, 1, gx, gy) != 0:
        failures.append('an F.Cu rip must not charge B.Cu')

    hc.record_rip(cfg, route, LAYER_MAP)
    c2 = _cost_at(hc.history_rows(cfg), 0, gx, gy)
    if c2 != 2 * c1:
        failures.append(f'second rip of the same corridor: {c2} != 2 x {c1} '
                        '(history must ACCUMULATE, unlike the rra ghosts)')
    # persistence: rows carry no notion of "the victim rerouted"
    if hc.history_rows(cfg) is not hc.history_rows(cfg):
        failures.append('rows must be memoized between events')
    if _cost_at(hc.history_rows(cfg), 0, gx, gy) != c2:
        failures.append('history must survive re-reads unchanged')

    # ---- 4. cap ------------------------------------------------------------
    _arm(KICAD_HISTORY_CAP='0.6')
    cfg = _cfg()
    hc.reset_history(cfg)
    for _ in range(5):
        hc.record_rip(cfg, route, LAYER_MAP)
    capped = _cost_at(hc.history_rows(cfg), 0, gx, gy)
    if capped != cfg.cell_cost(0.6):
        failures.append(f'cap: {capped} != cell_cost(0.6)={cfg.cell_cost(0.6)} '
                        'after 5 rips of 0.5mm')

    # ---- 5. frontier weight + consecutive-duplicate suppression ------------
    _arm()
    cfg = _cfg()
    hc.reset_history(cfg)
    frontier = [(50, 50, 0), (51, 50, 0), (50, 51, 1)]
    hc.record_blocked_frontier(cfg, frontier)
    fc = _cost_at(hc.history_rows(cfg), 0, 50, 50)
    if fc != cfg.cell_cost(0.5 * 0.25):
        failures.append(f'frontier cell {fc} != weighted cell_cost(0.125)='
                        f'{cfg.cell_cost(0.125)}')
    if _cost_at(hc.history_rows(cfg), 1, 50, 51) != fc:
        failures.append('frontier must charge the cell on ITS layer')
    hc.record_blocked_frontier(cfg, frontier)          # same failure, re-analyzed
    if _cost_at(hc.history_rows(cfg), 0, 50, 50) != fc:
        failures.append('one failure analyzed twice in a row must charge once')
    hc.record_rip(cfg, route, LAYER_MAP)               # something else happens
    hc.record_blocked_frontier(cfg, frontier)          # genuine recurrence
    if _cost_at(hc.history_rows(cfg), 0, 50, 50) != 2 * fc:
        failures.append('a frontier recurring later must accrue')
    field = cfg._history_cong
    if (field.rips, field.frontiers) != (1, 2):
        failures.append(f'event counters wrong: {field.rips} rips / '
                        f'{field.frontiers} frontiers, expected 1 / 2')

    # ---- 5b. radius widens the stamp beyond the copper ---------------------
    _arm(KICAD_HISTORY_RADIUS='0.3')
    cfg = _cfg()
    hc.reset_history(cfg)
    hc.record_rip(cfg, _result([_seg(1.0, 1.0, 3.0, 1.0)],
                               [_via(5.0, 5.0)]), LAYER_MAP)
    rows = hc.history_rows(cfg)
    # 0.2mm track: half-width 0.1 + 0.3 radius = 4 cells at the 0.1mm grid
    if _cost_at(rows, 0, gx, gy + 4) <= 0:
        failures.append('radius: cell 4 cells off the centerline must be charged')
    if _cost_at(rows, 0, gx, gy + 8) != 0:
        failures.append('radius: the stamp must stop at half-width + radius')
    vgx, vgy = coord.to_grid(5.0, 5.0)
    if _cost_at(rows, 0, vgx, vgy) <= 0 or _cost_at(rows, 1, vgx, vgy) <= 0:
        failures.append('a ripped via must charge EVERY layer (barrel)')

    # ---- 5c. the sorted merge-insert accumulator == a plain dict -----------
    # accumulate() keeps a sorted (keys, weights) pair by merge-insert instead
    # of re-sorting the field on every event (it runs once per conflict on a
    # field of up to ~1e6 cells). Cross-check it against the obvious
    # implementation over random events.
    _arm()
    field = hc.HistoryField()
    ref = {}
    rng = np.random.default_rng(7)
    for _ in range(200):
        cells = rng.integers(0, 3000, size=int(rng.integers(1, 80))).astype(np.int64)
        inc = float(rng.integers(1, 4))
        field.accumulate(cells, inc)
        for c in set(cells.tolist()):
            ref[c] = ref.get(c, 0) + inc
    ref_keys = sorted(ref)
    if list(field.keys) != ref_keys or any(
            abs(field.weights[i] - ref[k]) > 1e-9 for i, k in enumerate(ref_keys)):
        failures.append('merge-insert accumulator disagrees with a dict reference')

    # ---- 6. composition into the real map ---------------------------------
    _arm()
    cfg = _cfg()
    hc.reset_history(cfg)
    hc.record_rip(cfg, route, LAYER_MAP)
    rows = hc.history_rows(cfg)
    keys = (rows[:, 0].astype(np.int64) << 48) | \
           ((rows[:, 1].astype(np.int64) + (1 << 23)) << 24) | \
           (rows[:, 2].astype(np.int64) + (1 << 23))
    if len(np.unique(keys)) != len(keys):
        failures.append('rows must be unique per (layer, cell) -- sum mode '
                        'would otherwise count history more than once')
    obs = GridObstacleMap(len(cfg.layers))
    merge_track_proximity_costs(obs, {}, ghost_costs={('history',): rows},
                                config=cfg)
    got = obs.get_layer_proximity_cost(gx, gy, 0)
    if got != cfg.cell_cost(0.5):
        failures.append(f'composed map cost {got} != {cfg.cell_cost(0.5)}')
    if obs.get_layer_proximity_cost(gx, gy + 30, 0) != 0:
        failures.append('history must not leak far from the ripped corridor')

    # ---- 8. v2 defaults: contest event is primary, v1 stamps are off -------
    _arm(KICAD_HISTORY_COST='0.1', KICAD_HISTORY_RIP_WEIGHT='0',
         KICAD_HISTORY_BLOCKED_WEIGHT='0', KICAD_HISTORY_ESCALATE='1.0')
    cfg = _cfg()
    hc.reset_history(cfg)
    hc.record_rip(cfg, route, LAYER_MAP)              # v1 stamp off ...
    hc.record_blocked_frontier(cfg, [(9, 9, 0)])      # ... and raw frontier off
    if hc.history_rows(cfg) is not None:
        failures.append('v2 defaults: rip footprint / raw frontier must not charge')
    contested = [(50, 50, 0), (51, 50, 0), (50, 51, 1)]
    hc.record_contested(cfg, contested)
    cc = _cost_at(hc.history_rows(cfg), 0, 50, 50)
    if cc != cfg.cell_cost(0.1):
        failures.append(f'contest cell {cc} != full cell_cost(0.1)='
                        f'{cfg.cell_cost(0.1)}')
    if _cost_at(hc.history_rows(cfg), 1, 50, 51) != cc:
        failures.append('contest must charge the cell on ITS layer')
    hc.record_contested(cfg, contested)               # same failure re-analyzed
    if _cost_at(hc.history_rows(cfg), 0, 50, 50) != cc:
        failures.append('one contest analyzed twice in a row must charge once')
    hc.record_rip(cfg, route, LAYER_MAP)              # board changed
    hc.record_contested(cfg, contested)               # repeat -> doubles
    if _cost_at(hc.history_rows(cfg), 0, 50, 50) != 2 * cc:
        failures.append('2nd contest of a cell must DOUBLE it (escalate 1.0)')
    hc.record_rip(cfg, route, LAYER_MAP)
    hc.record_contested(cfg, contested)               # third -> doubles again
    if _cost_at(hc.history_rows(cfg), 0, 50, 50) != 4 * cc:
        failures.append('3rd contest must double again (0.1 -> 0.2 -> 0.4)')
    field = cfg._history_cong
    if field.contests != 3:
        failures.append(f'contest counter {field.contests} != 3')

    # ---- 9. growth guard EVICTS lowest-weight, never refuses ---------------
    _arm(KICAD_HISTORY_MAX_CELLS='5')                 # flat v1 accumulation
    field = hc.HistoryField()
    field.accumulate(np.array([1, 2, 3], dtype=np.int64), 0.5)
    field.accumulate(np.array([1, 2, 3], dtype=np.int64), 0.5)   # w = 1.0
    field.accumulate(np.array([4, 5, 6, 7, 8], dtype=np.int64), 0.5)
    if list(field.keys) != [1, 2, 3, 7, 8]:
        failures.append(f'eviction must drop the lowest-weight cells '
                        f'(key tie-break), got {list(field.keys)}')
    if field.evicted != 3:
        failures.append(f'evicted counter {field.evicted} != 3')
    if [float(w) for w in field.weights] != [1.0, 1.0, 1.0, 0.5, 0.5]:
        failures.append('eviction must keep the heavy cells\' weights intact')

    # ---- 7. the hook is really in rip_up_net ------------------------------
    import inspect
    import rip_up_reroute
    src = inspect.getsource(rip_up_reroute.rip_up_net)
    if src.count('record_rip') < 2:
        failures.append('rip_up_net must record BOTH the whole-net and the '
                        '#510 partial-leg rip')
    import routing_context
    ctx_src = inspect.getsource(routing_context)
    if ctx_src.count('add_history_source') < 8:   # import + call per builder
        failures.append('every obstacle builder (single-ended, diff pair, '
                        'incremental, in-place prepare) must price history')
    import layer_swap_fallback
    lsf_src = inspect.getsource(layer_swap_fallback)
    # Its maps come from the main loop's builder, which prices history (the
    # routing_context count above); a hand-built map would not.
    if ('build_diff_pair_obstacles' not in lsf_src
            or lsf_src.count('_pair_map(') < 4 or 'clone_fresh()' in lsf_src):
        failures.append('the diff-pair layer-swap fallback maps (retry / rip / '
                        'reroute) must be built by build_diff_pair_obstacles, '
                        'so they price history too')
    if 'record_rip(config' not in lsf_src:
        failures.append('the layer-swap fallback rips blockers INLINE (not via '
                        'rip_up_net) -- that rip must be recorded too')
    import blocking_analysis
    ba_src = inspect.getsource(blocking_analysis.analyze_frontier_blocking)
    if 'record_blocked_frontier' not in ba_src:
        failures.append('failed-search frontier hook missing')
    if 'record_contested' not in ba_src:
        failures.append('v2 contest hook (frontier∩blocker) missing from '
                        'analyze_frontier_blocking')

    _disarm()
    if failures:
        for f in failures:
            print(f'FAIL: {f}')
        return 1
    print('PASS: history congestion ships ARMED (v1flat_01) with a working '
          'COST=0 disable, accumulates per cell '
          'across rips, caps, weights frontiers lower, composes through the '
          'real merge pass, and is wired into every rip / builder')
    return 0


if __name__ == '__main__':
    sys.exit(main())
