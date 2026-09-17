#!/usr/bin/env python3
"""#977: the #908 own-pad lift now has EXACTLY ONE lifter per obstacle map, and
this is the census that keeps that list honest.

The base obstacle map used to lift the rows itself whenever it was built for a
single net, and record which net in a table keyed by `id(map)` so the routing
paths would not lift them a second time. That record could not be trusted -- an
address names an object only while the object is alive, and a freed map's
address goes straight to the next map -- so both the lift and the record were
deleted, and each routing path now lifts on the map it routes on.

That trade is the reason for this file. The old bake was a CATCH-ALL: any map
built for one net arrived lifted, whoever had built it. Lifting at the point of
use is correct and unambiguous, but it is only as complete as the list of
places that do it -- and a NEW map-producing path added later would route on a
map whose own-pad rows were never taken off, with no failure anywhere until a
board ships with a pad sealed by its own footprint's copper.

So: every function in `py_router` that PRODUCES an obstacle map (a
`build_base_obstacle_map` call, or a `clone_fresh()` of one) is registered
below with what it does about the lift. A new producer fails this gate until
someone writes down which it is.

WHAT THIS GATE CAN AND CANNOT SEE. It reads the AST, so it can see every
producer, and it can check a `LIFTS` claim -- that function's own source must
reference the lift rows, so the label cannot outlive the code. It CANNOT see
whether a `CONSUMER_LIFTS`, `NOT_ROUTED` or `NO_LIFT` verdict is true: those
are a reading of what the caller does with the map, recorded here so the next
reader starts from one, not so the machine can confirm it.

`NO_LIFT` rows are PRE-EXISTING #908 coverage gaps, not damage from #977: those
maps are built for a whole batch of nets, so the old bake never fired for them
either. They are written down because an unrecorded gap is indistinguishable
from a covered path.

Run:
    python3 tests/test_977_lift_site_registry.py
"""

import ast
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)

RUN_ALL_TIMEOUT = 120

#: A call to one of these produces an obstacle map a net could be routed on.
PRODUCERS = ('build_base_obstacle_map', 'clone_fresh')

#: What a producer's own function does about the #908 own-pad lift.
LIFTS = 'lifts'                    # applies it to the map it produces
CONSUMER_LIFTS = 'consumer-lifts'  # hands the map to something that applies it
NOT_ROUTED = 'not-routed'          # no copper is laid from this map
NO_LIFT = 'no-lift'                # routes without it: a recorded #908 gap

#: The rows the lift is made of. A function claiming LIFTS must name them.
LIFT_ROWS = ('_graphic_own_pad_lift', '_graphic_own_pad_via_lift')

REGISTRY = {
    # --- applies the lift itself ------------------------------------------
    ('py_router/routing_context.py', 'build_single_ended_obstacles'): (
        LIFTS, 'clones a base for ONE net and lifts on the clone; no restore, '
               'because the clone is discarded with the route'),
    ('py_router/net_rescue.py', '_pristine_rescue_map'): (
        LIFTS, 'the rescue reaches neither prepare nor build_single_ended_'
               'obstacles, so the clone it hands out arrives lifted (#977; '
               'the base build used to do this for it)'),

    # --- hands the map to a lifter ----------------------------------------
    ('py_router/route.py', 'batch_route'): (
        CONSUMER_LIFTS, "the run's base map: never routed on directly -- the "
                        'working map clones it and prepare_obstacles_inplace '
                        'lifts per net, and the fallback path clones it '
                        'through build_single_ended_obstacles'),
    ('py_router/obstacle_cache.py', 'build_working_obstacle_map'): (
        CONSUMER_LIFTS, 'the shared working map, lifted per net by '
                        'prepare_obstacles_inplace and put back by '
                        'restore_obstacles_inplace'),

    # --- no copper is laid from this map ----------------------------------
    ('py_router/obstacle_map.py', 'build_base_obstacle_map'): (
        NOT_ROUTED, 'the per-net via-rung sub-build: only its blocked-via '
                    'cells are read back, the map itself is dropped'),
    ('py_router/obstacle_cache.py', 'run_obstacle_audit'): (
        NOT_ROUTED, 'audit clone, measured not routed'),
    ('py_router/obstacle_cache.py', 'run_obstacle_content_audit'): (
        NOT_ROUTED, 'audit clone, measured not routed'),
    ('py_router/blocking_analysis.py', 'mincut_probe_order'): (
        NOT_ROUTED, 'mincut feasibility probe; orders rip candidates, lays '
                    'nothing'),
    ('py_router/bus_corridor.py', 'plan_bus_corridors'): (
        NOT_ROUTED, 'corridor probe: it routes representatives to SCORE rungs '
                    'and the copper is discarded (an unlifted probe can only '
                    'mis-score a group, never ship copper)'),
    ('py_router/global_plan.py', 'plan_global_routes'): (
        NOT_ROUTED, 'rough global plan probe; produces a net order and '
                    'reservation stamps, no copper'),

    # --- routes without the lift: recorded #908 gaps ----------------------
    ('py_router/routing_context.py', 'build_incremental_obstacles'): (
        NO_LIFT, 'phase-3 fast clone of the working map. Its callers that '
                 'route MULTIPOINT taps go through route_multipoint_taps, '
                 'which lifts via ensure_own_pad_lift; the phase-3 victim '
                 'reroute and the stranded-net probe route on it directly and '
                 'unlifted. Pre-existing: the working map is a batch map, so '
                 'the old bake never covered it either'),
    ('py_router/routing_context.py', 'build_diff_pair_obstacles'): (
        NO_LIFT, 'the diff-pair engine has never carried the own-pad lift '
                 '(#908 wired it into the single-ended paths only)'),
    ('py_router/route_diff.py', 'batch_route_diff_pairs'): (
        NO_LIFT, 'diff-pair base maps; see build_diff_pair_obstacles'),
    ('py_router/layer_swap_fallback.py', 'try_fallback_layer_swap'): (
        NO_LIFT, 'clones the diff-pair base; see build_diff_pair_obstacles'),
    ('py_router/bga_fanout/__init__.py', '_generate_bga_fanout_core'): (
        NO_LIFT, 'BGA fanout escapes; built for every fanned net at once, so '
                 'the old bake never fired here either'),
    ('py_router/qfn_fanout/__init__.py', 'generate_qfn_fanout'): (
        NO_LIFT, 'QFN fanout; as for BGA fanout'),
    ('py_router/qfn_fanout/__init__.py', '_underpad_via_escape'): (
        NO_LIFT, 'QFN under-pad escape; as for BGA fanout'),
    ('py_router/route_planes.py', 'main'): (
        NO_LIFT, 'plane base map (the standalone plane CLI)'),
    ('py_router/route_planes.py', 'route_plane_connection'): (
        NO_LIFT, 'plane region join'),
    ('py_router/plane_region_connector.py', 'route_plane_connection_wide'): (
        NO_LIFT, 'wide plane region join'),
}


def _producer_sites():
    """{(relpath, function): [(producer, lineno)]} for every producing call."""
    found = {}
    pkg = os.path.join(ROOT_DIR, 'py_router')
    for dirpath, _dirs, files in os.walk(pkg):
        for fn in sorted(files):
            if not fn.endswith('.py'):
                continue
            path = os.path.join(dirpath, fn)
            rel = os.path.relpath(path, ROOT_DIR).replace(os.sep, '/')
            with open(path, encoding='utf-8') as fh:
                text = fh.read()
            tree = ast.parse(text, path)
            stack = []

            def visit(node):
                is_fn = isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                if is_fn:
                    stack.append(node)
                if isinstance(node, ast.Call):
                    f = node.func
                    name = (f.id if isinstance(f, ast.Name)
                            else f.attr if isinstance(f, ast.Attribute) else None)
                    if name in PRODUCERS and stack:
                        found.setdefault((rel, stack[-1].name), []).append(
                            (name, node.lineno))
                for child in ast.iter_child_nodes(node):
                    visit(child)
                if is_fn:
                    stack.pop()

            visit(tree)
            _SOURCES[rel] = text.splitlines()
            for key, node in _fn_nodes(tree, rel):
                _FN_NODES[key] = node
    return found


_SOURCES = {}
_FN_NODES = {}


def _fn_nodes(tree, rel):
    out = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            out.append(((rel, node.name), node))
    return out


def _fn_source(key):
    node = _FN_NODES.get(key)
    if node is None:
        return ''
    lines = _SOURCES.get(key[0], [])
    return '\n'.join(lines[node.lineno - 1:getattr(node, 'end_lineno',
                                                   node.lineno)])


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"\n        {detail}" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    sites = _producer_sites()

    check('the census found the producers at all (else it gates nothing)',
          len(sites) >= 15, f'{len(sites)} producing function(s)')

    unregistered = sorted(set(sites) - set(REGISTRY))
    check('every obstacle-map producer is registered',
          not unregistered,
          'new map-producing function(s) -- say what each does about the #908\n'
          '        own-pad lift and add it to REGISTRY (LIFTS / CONSUMER_LIFTS /\n'
          '        NOT_ROUTED / NO_LIFT):\n        '
          + '\n        '.join(f'{r}:{f} @ line(s) '
                              + ','.join(str(n) for _p, n in sites[(r, f)])
                              for r, f in unregistered))

    gone = sorted(set(REGISTRY) - set(sites))
    check('no registry row names a producer that no longer exists',
          not gone,
          'REGISTRY row(s) with no matching call -- delete them:\n        '
          + '\n        '.join(f'{r}:{f}' for r, f in gone))

    # A LIFTS claim is the one the machine can check: the function must name
    # the rows it says it lifts.
    for key in sorted(k for k in REGISTRY if k in sites):
        verdict, _why = REGISTRY[key]
        src = _fn_source(key)
        names = any(tok in src for tok in LIFT_ROWS)
        if verdict == LIFTS:
            check(f'{key[0]}:{key[1]} really does lift', names,
                  'registered LIFTS but its body never names '
                  + ' or '.join(LIFT_ROWS))
        elif verdict == NO_LIFT:
            check(f'{key[0]}:{key[1]} is still the gap it is registered as',
                  not names,
                  'registered NO_LIFT but its body now names the lift rows -- '
                  'if the gap was closed, re-register it as LIFTS')

    counts = {}
    for key in sites:
        counts[REGISTRY.get(key, ('?',))[0]] = counts.get(
            REGISTRY.get(key, ('?',))[0], 0) + 1
    print('\n  producers by verdict: '
          + ', '.join(f'{k}={v}' for k, v in sorted(counts.items())))
    print('  (NO_LIFT rows are pre-existing #908 gaps, recorded so they stay '
          'visible;\n   this gate cannot verify a non-LIFTS verdict -- see the '
          'module docstring.)')

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
