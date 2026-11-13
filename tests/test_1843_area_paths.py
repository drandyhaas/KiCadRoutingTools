#!/usr/bin/env python3
"""#1843: every path that prices copper clearance reads the rule areas.

A clearance rule scoped by a rule area is honoured by COMPOSING each
keep-out stamp over the areas (`area_clearance.composed` /
`composed_builder`) and by folding the area into every pair the post-route
passes price (`single_ended_routing._area_excess` and the per-pass hooks).
A stamp path or a pass that bypasses those reads the class value inside the
area without a word -- the cleanup pipeline and the plane builders did, and
shipped copper inside a tightened room. This test makes such a path LOUD:

  1. every function in py_router/ and kicad_routing_plugin/ that stamps an
     obstacle map directly is in REGISTRY, as composed (it carries a
     composition decorator, or composes inside), as stamping only within a
     composed caller (checked on the call graph), or with the reason it
     prices no copper-to-copper clearance; a new one fails here until it is
     classified;
  2. every distance-helper call in the post-route passes passes `config`
     (the area fold rides on it), and the cleanup pipeline hands `config`
     to every pass whose signature takes it;
  3. the fold itself: a track and a via measured against foreign copper in
     a tight area, and in a relaxing one, read the area's value, and a
     track that only crosses the area's split zone is split there;
  4. a room's track width: every function that builds a `min_track_width`
     from tracks reads the room's rule (KiCad grades a track it binds at the
     rule, not at Board Setup's minimum), and every census of shipped track
     widths leaves a room neck out of its shortfall (the neck is the board's
     own rule, disclosed as `area_necks`) -- or is in WIDTH_REGISTRY with the
     reason it needs neither.
"""
import ast
import glob
import os
import re
import sys
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'rust_router'))

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


# module:qualname -> ('composed' | 'inside' | 'exempt', why)
REGISTRY = {
    # composed by a decorator or inside the function
    'obstacle_map.py:build_base_obstacle_map': ('composed', 'one sub-build per area (_area_pass)'),
    'obstacle_map.py:add_net_stubs_as_obstacles': ('composed', '@_area_composed'),
    'obstacle_map.py:add_net_vias_as_obstacles': ('composed', '@_area_composed'),
    'obstacle_map.py:add_vias_list_as_obstacles': ('composed', '@_area_composed'),
    'obstacle_map.py:add_segments_list_as_obstacles': ('composed', '@_area_composed'),
    'obstacle_map.py:remove_segments_list_from_obstacles': ('composed', '@_area_composed'),
    'obstacle_map.py:remove_vias_list_from_obstacles': ('composed', '@_area_composed'),
    'obstacle_map.py:add_pads_via_keepout': ('composed', '@_area_composed'),
    'obstacle_map.py:remove_pads_via_keepout': ('composed', '@_area_composed'),
    'plane_obstacle_builder.py:build_via_obstacle_map': ('composed', '@_area_composed_builder'),
    'plane_obstacle_builder.py:build_routing_obstacle_map': ('composed', '@_area_composed_builder'),
    'plane_region_connector.py:build_base_obstacles': ('composed', '@_area_composed_builder'),
    # stamp only for a composed caller (the call graph is checked)
    'obstacle_map.py:add_rule_area_keepout_obstacles': ('exempt', 'a keep-out area, not a clearance pair'),
    'obstacle_map.py:add_pour_barrier_obstacles': ('inside', 'build_base_obstacle_map'),
    'obstacle_map.py:_add_segment_obstacle_with_exclusion': ('inside', 'net stubs'),
    'obstacle_map.py:_stamp_rungs_at_own_via': ('inside', 'net stubs / pads / vias'),
    'obstacle_map.py:_mirror_rungs_add': ('inside', 'list stamps'),
    'obstacle_map.py:_mirror_rungs_remove': ('inside', 'list stamps'),
    'obstacle_map.py:_batch_cells_one_layer': ('inside', 'builders, the composed route stamp'),
    'obstacle_map.py:_flush_cell_sink': ('inside', 'builders'),
    'obstacle_map.py:_batch_vias': ('inside', 'builders, the composed route stamp'),
    'obstacle_map.py:_flush_via_sink': ('inside', 'builders'),
    'plane_obstacle_builder.py:_batch_block_circles_via': ('inside', 'builders'),
    'plane_obstacle_builder.py:_batch_block_circles_cell': ('inside', 'builders'),
    'plane_obstacle_builder.py:block_circle': ('inside', 'builders'),
    'plane_obstacle_builder.py:_block_custom_pad_polys': ('inside', 'builders'),
    'plane_obstacle_builder.py:_add_pad_via_obstacle': ('inside', 'build_via_obstacle_map'),
    'sense_branch.py:stamp_base_obstacles': ('inside', 'build_base_obstacle_map'),
    'sense_branch.py:stamp_plane_obstacles': ('inside', 'plane builders'),
    # no copper-to-copper clearance priced, or rows composed where computed
    'obstacle_cache.py:add_net_obstacles_from_cache': ('exempt', 'replays rows precompute_net_obstacles composed'),
    'obstacle_cache.py:remove_net_obstacles_from_cache': ('exempt', 'replays rows precompute_net_obstacles composed'),
    'plane_pad_tap.py:SharedViaMaps.note_pass_copper': ('exempt', 'rows from precompute_via_placement_obstacles, composed'),
    'plane_pad_tap.py:SharedViaMaps.note_net_ripped': ('exempt', 'rows from precompute_via_placement_obstacles, composed'),
    'plane_pad_tap.py:SharedViaMaps.note_net_restored': ('exempt', 'rows from precompute_via_placement_obstacles, composed'),
    'obstacle_map.py:_StaticStampProxy.add_blocked_cell': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_StaticStampProxy.add_blocked_via': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_StaticStampProxy.add_blocked_cells_batch': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_StaticStampProxy.add_blocked_vias_batch': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_StaticStampProxy.add_blocked_cell_spans_batch': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_StaticStampProxy.add_blocked_via_spans_batch': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_RungPassStampProxy.add_static_blocked_via': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_RungPassStampProxy.add_static_blocked_vias_batch': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_RungPassStampProxy.add_static_blocked_via_spans_batch': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_RecordingObstacles.add_blocked_cells_batch': ('exempt', 'proxy plumbing'),
    'obstacle_map.py:_RecordingObstacles.add_blocked_cell': ('exempt', 'proxy plumbing'),
    'sense_branch.py:_Collector.add_blocked_cell': ('exempt', 'proxy plumbing'),
    'sense_branch.py:_Collector.add_blocked_via': ('exempt', 'proxy plumbing'),
    'sense_branch.py:sense_obstacle_rows': ('exempt', 'a row collector; its stamps are the two above'),
    'obstacle_map.py:add_user_keepout_obstacles': ('exempt', 'a keep-out area, not a clearance pair'),
    'obstacle_map.py:_add_cutout_obstacles': ('exempt', 'board edge rule'),
    'obstacle_map.py:_add_edge_contour_obstacles': ('exempt', 'board edge rule'),
    'obstacle_map.py:_add_rectangular_edge_obstacles._sweep': ('exempt', 'board edge rule'),
    'obstacle_map.py:_add_polygon_edge_obstacles': ('exempt', 'board edge rule'),
    'plane_obstacle_builder.py:_add_board_edge_via_obstacles': ('exempt', 'board edge rule'),
    'plane_obstacle_builder.py:_add_board_edge_track_obstacles': ('exempt', 'board edge rule'),
    'obstacle_map.py:block_via_cells_near_drills': ('exempt', 'hole-to-hole, not copper clearance'),
    'obstacle_map.py:block_track_cells_near_drills': ('exempt', 'copper-to-hole, not copper clearance'),
    'obstacle_map.py:block_track_cells_near_override_pad_holes': ('exempt', 'copper-to-hole'),
    'obstacle_map.py:add_same_net_via_clearance': ('exempt', 'same net: no clearance rule binds it'),
    'obstacle_map.py:add_same_net_pad_drill_via_clearance': ('exempt', 'same net'),
    'net_rescue.py:_attempt_edge': ('exempt', 'same-net pad via keep-out (#581)'),
    'net_rescue.py:_pristine_rescue_map': ('exempt', 'lifts own-pad rows; the map is build_base_obstacle_map'),
    'routing_context.py:_add_free_via_positions': ('exempt', 'same-net free vias'),
    'routing_context.py:build_diff_pair_leg_obstacles': ('exempt', "other pairs' reserved launch regions"),
    'routing_context.py:build_single_ended_obstacles': ('exempt', 'own-pad lift of composed rows'),
    'routing_context.py:prepare_obstacles_inplace': ('exempt', 'own-pad lift, same-net via rings'),
    'routing_context.py:restore_obstacles_inplace': ('exempt', 'undoes prepare_obstacles_inplace'),
    'routing_context.py:ensure_own_pad_lift': ('exempt', 'own-pad lift of composed rows'),
    'routing_context.py:release_own_pad_lift': ('exempt', 'own-pad lift of composed rows'),
    'single_ended_routing.py:_register_unblock_via': ('exempt', 'same-net hole-to-hole ring'),
    'single_ended_routing.py:_release_unblock_via_rings': ('exempt', 'same-net hole-to-hole ring'),
    'single_ended_routing.py:route_multipoint_taps': ('exempt', 'same-net hole-to-hole ring'),
    'single_ended_routing.py:_route_multipoint_taps_impl._register_inprogress_via': ('exempt', 'same-net ring'),
    'obstacle_map.py:add_connector_region_via_blocking': ('exempt', "a pair's own breakout corridor"),
    'diff_pair_routing.py:_route_hybrid_leg': ('exempt', "the pair's own partner (the pair gap)"),
    'net_keepouts.py:_EdgeBandSink._cells': ('exempt', 'edge band'),
    'net_keepouts.py:_EdgeBandSink._vias': ('exempt', 'edge band'),
    'net_keepouts.py:block': ('exempt', 'net-filtered keep-out area'),
    'net_keepouts.py:block.remove': ('exempt', 'net-filtered keep-out area'),
    'route_planes.py:_block_route_as_obstacle': ('exempt', "a split plane's spine: KiCad's fill keeps the clearance"),
    'route_planes.py:build_plane_base_obstacles': ('exempt', "a split plane's spine: KiCad's fill keeps the clearance"),
    'routing_diagnostics.py:same_net_pad_seal_hint': ('exempt', 'a diagnostic on a scratch map'),
    'area_clearance.py:composed_builder.deco.build': ('exempt', "the composition itself: its own stamp is a tight area's no-via edge band"),
}

STAMP = re.compile(r'^(add|remove)_(static_)?blocked_')
COMPOSERS = ('_area_composed', '_area_composed_builder', 'composed', 'composed_builder')


def _functions():
    """{module:qualname: (FunctionDef, path)} of every function that calls a
    map stamp method itself (not in a nested function)."""
    out = {}
    for path in sorted(glob.glob(os.path.join(REPO, 'py_router', '*.py'))
                       + glob.glob(os.path.join(REPO, 'kicad_routing_plugin', '*.py'))):
        tree = ast.parse(open(path, encoding='utf-8').read())
        base = os.path.basename(path)

        def walk(node, prefix):
            for ch in ast.iter_child_nodes(node):
                if isinstance(ch, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    q = prefix + ch.name
                    nested = set()
                    for sub in ast.walk(ch):
                        if sub is not ch and isinstance(sub, (ast.FunctionDef, ast.AsyncFunctionDef)):
                            nested.update(id(n) for n in ast.walk(sub))
                    if any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                           and STAMP.match(n.func.attr) and id(n) not in nested
                           for n in ast.walk(ch)):
                        out[f"{base}:{q}"] = (ch, path)
                    walk(ch, q + '.')
                elif isinstance(ch, ast.ClassDef):
                    walk(ch, prefix + ch.name + '.')
        walk(tree, '')
    return out


def _graph():
    """(callers, decorated): {callee name: {module:qualname of top-level
    py_router / plugin functions calling it}} and the set of those functions
    carrying a composition decorator."""
    callers, decorated = {}, set()
    for path in (glob.glob(os.path.join(REPO, 'py_router', '*.py'))
                 + glob.glob(os.path.join(REPO, 'kicad_routing_plugin', '*.py'))):
        tree = ast.parse(open(path, encoding='utf-8').read())
        for node in tree.body:
            if not isinstance(node, ast.FunctionDef):
                continue
            key = f"{os.path.basename(path)}:{node.name}"
            if any(ast.unparse(d).split('(')[0] in COMPOSERS for d in node.decorator_list):
                decorated.add(key)
            for n in ast.walk(node):
                if isinstance(n, ast.Call):
                    f = n.func
                    fn = f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', None)
                    if fn:
                        callers.setdefault(fn, set()).add(key)
    return callers, decorated


# functions that stamp only through registered helpers and compose inside:
# {key: the composition call their body must make}
COMPOSED_INSIDE = {
    'plane_region_connector.py:add_route_to_obstacles': '_area_compose_route_stamp',
}

# functions that stamp only through a registered helper, with why they need
# not compose (they are not stamp paths of their own)
INTERMEDIATE = {
    'plane_obstacle_builder.py:block_via_position': 'same-net stitching via spacing',
}


def stamp_paths():
    print("1. every stamp path is composed, stamps inside a composed one, or says why not")
    found = _functions()
    new = sorted(set(found) - set(REGISTRY))
    ok(not new, "no unclassified stamp path"
       + (": " + ", ".join(new) + " -- compose it over the area rules "
          "(area_clearance.composed / composed_builder) or register why it prices "
          "no copper clearance" if new else ""))
    gone = sorted(set(REGISTRY) - set(found))
    ok(not gone, "every registered path still exists" + (f": {gone}" if gone else ""))
    for key, (kind, why) in sorted(REGISTRY.items()):
        if key not in found or kind != 'composed':
            continue
        node, path = found[key]
        decos = [ast.unparse(d) for d in node.decorator_list]
        src = ast.get_source_segment(open(path, encoding='utf-8').read(), node) or ''
        ok(any(d.split('(')[0] in COMPOSERS for d in decos) or 'area_clearance' in src,
           f"{key} composes ({why})")
    callers, decorated = _graph()
    composed = decorated | {k for k, v in REGISTRY.items() if v[0] == 'composed'}
    for key, token in COMPOSED_INSIDE.items():
        mod, name = key.split(':')
        src = open(os.path.join(REPO, 'py_router', mod), encoding='utf-8').read()
        node = next((n for n in ast.parse(src).body
                     if isinstance(n, ast.FunctionDef) and n.name == name), None)
        body = ast.get_source_segment(src, node) if node is not None else ''
        ok(token in (body or ''), f"{key} composes ({token})")
        if token in (body or ''):
            composed.add(key)

    def caller_ok(c, seen):
        # a caller is fine when it composes, is a registered path of its own,
        # or is itself called only from such (followed up the call graph)
        if c in composed or c in INTERMEDIATE:
            return True
        if c in REGISTRY and REGISTRY[c][0] != 'inside':
            return True
        if c in seen:
            return True
        seen = seen | {c}
        name = c.split(':')[1]
        up = callers.get(name, set())
        if not up:
            return name.startswith('_')     # a private function nothing calls
        return all(caller_ok(u, seen) for u in up)
    for key, (kind, why) in sorted(REGISTRY.items()):
        if kind != 'inside':
            continue
        name = key.split(':')[1].split('.')[-1]
        bad = sorted(c for c in callers.get(name, set()) if not caller_ok(c, {key}))
        ok(not bad, f"{key} is called only from composed stamps"
           + (f" (also from {bad})" if bad else ""))


HELPERS = ('_seg_foreign_seg_dist', '_seg_foreign_via_dist', '_seg_foreign_pad_dist',
           '_pt_foreign_pad_dist')


def pass_calls():
    print("2. the post-route passes price every pair with `config`")
    path = os.path.join(REPO, 'py_router', 'pcb_modification.py')
    tree = ast.parse(open(path, encoding='utf-8').read())
    missing = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            fn = f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', None)
            if fn in HELPERS and not any(k.arg == 'config' for k in n.keywords):
                missing.append(f"{fn}@{n.lineno}")
    ok(not missing, "every distance-helper call passes config"
       + (f": {missing}" if missing else ""))
    import inspect
    import pcb_modification
    src = open(os.path.join(REPO, 'py_router', 'cleanup_pipeline.py'), encoding='utf-8').read()
    ptree = ast.parse(src)
    fn_names = {}
    for n in ast.walk(ptree):
        if isinstance(n, ast.Call):
            for a in [n.func] + list(n.args):
                nm = getattr(a, 'id', None)
                if nm and hasattr(pcb_modification, nm) and callable(getattr(pcb_modification, nm)):
                    fn_names.setdefault(nm, []).append(n)
    lacking = []
    for nm, calls in sorted(fn_names.items()):
        try:
            params = inspect.signature(getattr(pcb_modification, nm)).parameters
        except (TypeError, ValueError):
            continue
        if 'config' not in params:
            continue
        for c in calls:
            pos = list(params).index('config')
            direct = c.func is not None and getattr(c.func, 'id', None) == nm
            has = any(k.arg == 'config' for k in c.keywords)
            if direct and len(c.args) > pos:
                has = True
            if not has:
                lacking.append(f"{nm}@{c.lineno}")
    ok(not lacking, "the cleanup pipeline hands config to every pass that takes it"
       + (f": {lacking}" if lacking else ""))


def fold():
    print("3. the fold reads the area, and a track is split at a tight area's zone")
    from shapely.geometry import box
    from area_clearance import AreaClearance, Region, split_run_copper
    from kicad_parser import Segment
    import single_ended_routing as ser
    tight = AreaClearance([Region('r', 'ROOM', 0.4, box(0.0, 0.0, 10.0, 10.0),
                                  frozenset(['F.Cu', 'B.Cu']), 1.0)], ['F.Cu', 'B.Cu'],
                          split_reach=0.2)
    relax = AreaClearance([Region('r', 'ROOM', 0.1, box(0.0, 0.0, 10.0, 10.0),
                                  frozenset(['F.Cu', 'B.Cu']), 0.0)], ['F.Cu', 'B.Cu'],
                          split_reach=0.2)

    def board(segs):
        return SimpleNamespace(segments=segs, vias=[], pads_by_net={}, footprints={})

    def cfg(area):
        return SimpleNamespace(area_clearance=area, scoped_clearance=None,
                               coupled_pair_clearances=None, layer_clearances={})
    # foreign track inside the area, own track outside 0.3 mm edge-to-edge
    foreign = Segment(start_x=5.0, start_y=5.0, end_x=9.0, end_y=5.0, width=0.2,
                      layer='F.Cu', net_id=2)
    pcb = board([foreign])
    base = 0.25
    hw = 0.1
    y = 5.0 + 0.1 + 0.3 + hw              # 0.3 mm gap
    d_plain = ser._seg_foreign_seg_dist(pcb, 1, 5.0, y, 9.0, y, 'F.Cu',
                                        base_clearance=base)
    d_tight = ser._seg_foreign_seg_dist(pcb, 1, 5.0, y, 9.0, y, 'F.Cu',
                                        base_clearance=base, config=cfg(tight))
    ok(d_plain - hw >= base - 1e-9 and d_tight - hw < base,
       f"a 0.3 mm gap clears the 0.25 base ({d_plain - hw:.3f}) but not a 0.4 room "
       f"({d_tight - hw:.3f} effective)")
    y2 = 5.0 + 0.1 + 0.15 + hw            # 0.15 mm gap
    d_relax = ser._seg_foreign_seg_dist(pcb, 1, 5.0, y2, 9.0, y2, 'F.Cu',
                                        base_clearance=base, config=cfg(relax))
    ok(d_relax - hw >= base - 1e-9, f"a 0.15 mm gap clears a 0.1 room ({d_relax - hw:.3f})")
    pcb.segments = []
    pcb.vias = [SimpleNamespace(x=5.0, y=5.0, size=0.6, net_id=2)]
    pcb._foreign_via_arr_cache = None
    yv = 5.0 + 0.3 + 0.3 + hw
    dv = ser._seg_foreign_via_dist(pcb, 1, 5.0, yv, 9.0, yv, 'F.Cu',
                                   base_clearance=base, config=cfg(tight))
    ok(dv - hw < base, f"a via in the room is held at its value ({dv - hw:.3f} effective)")
    # a track running out of the room splits at the zone, once
    s = Segment(start_x=5.0, start_y=3.0, end_x=20.0, end_y=3.0, width=0.2,
                layer='F.Cu', net_id=1)
    pairs = split_run_copper(tight, [s])
    pieces = pairs[0][1] if pairs else []
    ok(len(pieces) == 2 and abs(pieces[0].end_x - 10.2) < 1e-9
       and (pieces[0].end_x, pieces[0].end_y) == (pieces[1].start_x, pieces[1].start_y),
       f"split at x = {pieces[0].end_x if pieces else None} (zone 0.2 past the edge), "
       f"pieces meet exactly")
    ok(not split_run_copper(tight, pieces), "and a split piece is not split again")
    ok(not split_run_copper(relax, [s]), "a relaxing area splits nothing")


# module:qualname -> why it needs no room reading
WIDTH_REGISTRY = {
    'fix_kicad_drc_settings.py:copper_minima':
        "the copper a placement step laid or moved (#1578); placement lays no track",
    'repair_planes.py:main':
        "the standalone CLI's power census counts a neck as a shortfall: it says "
        "more than it must, never less",
}
#: a call to one of these reads the room's track_width rule
ROOM_READERS = ('_board_track_widths', 'room_width_test', 'live_room_width_test',
                'track_item', 'is_room_neck')
CENSUSES = ('new_copper_width_shortfall', 'power_width_report')


def width_paths():
    print("4. every track-width scan and census reads a room's width")
    files = (glob.glob(os.path.join(REPO, 'py_router', '*.py'))
             + glob.glob(os.path.join(REPO, 'kicad_routing_plugin', '*.py')))
    found, blind = set(), []
    for path in files:
        tree = ast.parse(open(path, encoding='utf-8').read())
        mod = os.path.basename(path)
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            # the function's own body (nested defs are visited on their own)
            nodes, todo = [], list(fn.body)
            while todo:
                n = todo.pop()
                nodes.append(n)
                for c in ast.iter_child_nodes(n):
                    if not isinstance(c, (ast.FunctionDef, ast.AsyncFunctionDef)):
                        todo.append(c)
            called, alias = set(), {}
            for n in nodes:
                if isinstance(n, ast.Call):
                    f = n.func
                    called.add(f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', None))
                elif isinstance(n, ast.ImportFrom):
                    for al in n.names:
                        if al.asname:
                            alias[al.asname] = al.name
            called |= {alias[c] for c in called if c in alias}
            # it reads tracks: a board's segments, or the live board's tracks
            tracks = any((isinstance(n, ast.Attribute) and n.attr == 'segments')
                         or (isinstance(n, ast.Name) and n.id == 'segments')
                         or (isinstance(n, ast.Call) and getattr(n.func, 'attr', None)
                             == 'GetTracks') for n in nodes)
            scans = tracks and any(
                (isinstance(n, ast.Dict) and any(isinstance(k, ast.Constant)
                                                 and k.value == 'min_track_width'
                                                 for k in n.keys))
                or (isinstance(n, ast.Subscript) and isinstance(n.ctx, ast.Store)
                    and isinstance(n.slice, ast.Constant)
                    and n.slice.value == 'min_track_width')
                for n in nodes)
            census = bool(called & set(CENSUSES))
            if not (scans or census) or fn.name in ROOM_READERS:
                continue
            key = f"{mod}:{fn.name}"
            found.add(key)
            if key in WIDTH_REGISTRY or called & set(ROOM_READERS):
                continue
            blind.append(key)
    ok(not blind, "every function that scans a track minimum or censuses track "
                  "widths reads the room" + (f": {sorted(blind)}" if blind else ""))
    stale = sorted(set(WIDTH_REGISTRY) - found)
    ok(not stale, "and WIDTH_REGISTRY names only live sites"
       + (f": {stale}" if stale else ""))
    ok({'fix_kicad_drc_settings.py:board_size_values',
        'fix_kicad_drc_settings.py:size_values_below_authored',
        'gui_utils.py:board_minima_from_live',
        'gui_utils.py:update_live_drc_floors',
        'diff_pair_custody.py:disclose_pair_widths'} <= found,
       "the scan finds the writeback, the authored grade, the GUI's live scans "
       "and the pair-width census")


def main():
    stamp_paths()
    pass_calls()
    fold()
    width_paths()
    print()
    if FAILS:
        print(f"FAILED: {len(FAILS)}")
        sys.exit(1)
    print("ALL PASS")


if __name__ == '__main__':
    main()
