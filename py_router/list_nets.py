#!/usr/bin/env python3
"""
List and analyze nets in a KiCad PCB file.

Examples:
    # List nets on a component
    python list_nets.py board.kicad_pcb --component U1

    # Show pad-to-net assignments
    python list_nets.py board.kicad_pcb --component U1 --pads

    # Detect differential pairs in the board
    python list_nets.py board.kicad_pcb --diff-pairs

    # Show power/ground nets with pad counts
    python list_nets.py board.kicad_pcb --power

    # Full board analysis
    python list_nets.py board.kicad_pcb --diff-pairs --power
"""

import argparse
import json
import math
import os
import re
from collections import defaultdict
from fnmatch import fnmatch, fnmatchcase
from kicad_parser import parse_kicad_pcb, find_components_by_type


# KiCad netclass field -> the routing-CLI flag that consumes it.
_NETCLASS_FIELDS = [
    ('clearance', 'clearance', '--clearance'),
    ('track_width', 'track_width', '--track-width'),
    ('via_diameter', 'via_size', '--via-size'),
    ('via_drill', 'via_drill', '--via-drill'),
    ('diff_pair_gap', 'diff_pair_gap', '--diff-pair-gap (route_diff.py)'),
    ('diff_pair_width', 'diff_pair_width', '--track-width (route_diff.py)'),
]

# Board Constraints (KiCad: Board Setup -> Constraints) live in the .kicad_pro
# under board.design_settings.rules. Unlike the net-class *defaults*, THESE are
# what DRC actually enforces for the geometric rules: a net-class track_width /
# via_diameter is only the size new objects are drawn at, never a DRC minimum --
# only these min_* values are. So the router may emit a via/track SMALLER than
# the net-class nominal (down to these) and the board still passes DRC. (#111/#115)
_CONSTRAINT_FIELDS = ('min_clearance', 'min_track_width', 'min_via_diameter',
                      'min_via_annular_width', 'min_hole_to_hole',
                      'min_through_hole_diameter', 'min_hole_clearance',
                      'min_copper_edge_clearance')

# JLCPCB manufacturing floors are modelled as selectable cost TIERS (issue #237)
# in fab_tiers.py. Re-exported here so existing `from list_nets import fab_floors`
# call sites keep working. `fab_floors(n)` is the NOMINAL (preferred) floor of the
# active tier; `fab_floor_min(n)` is the DEEPEST floor it can reach (the advanced
# rung that 'standard' escalates to, or the hard floor of 'advanced'/overrides) --
# used to grade DRC so legitimately-escalated fine geometry isn't false-flagged.
# The retired 'fine_via_*' keys are now just the advanced tier's via (fab_floor_min).
from fab_tiers import (  # noqa: E402  (re-export)
    fab_floors, fab_floor_min, fab_floor_ladder,
    set_default_fab_tier, get_default_fab_tier,
    add_fab_tier_args, fab_tier_from_args, warn_fab_escalation,
)


def _count_copper_layers(content):
    """Count copper layers from a .kicad_pcb's (layers ...) definitions.

    Matches layer-def lines like `(0 "F.Cu" signal)` / `(4 "In1(GND).Cu" ...)`
    but not pad `(layers "F.Cu" ...)` lists (those have no leading layer index).
    """
    return len(re.findall(r'\(\d+\s+"[^"]*\.Cu"', content)) or 0


def effective_floors(constraints, copper_layers):
    """Combine the board's own Constraints with the JLC fab floor.

    Returns the values the tools should actually use:
      - working_via_diameter / working_via_drill: the SMALL escape/working via to
        emit for fanout & routing, instead of the (larger) net-class nominal (#115).
        These are GEOMETRY constraints (the board's min via, reliably the size the
        human's vias actually are), floored at the fab minimum.
      - drc_clearance / drc_hole_to_hole: the manufacturability floor to grade DRC
        and connectivity against, instead of the inflated net-class clearance (#111).
        These are the fab SPACING floor directly: fine-pitch escapes are routed down
        to it, so DRC must check at it to pass them; and the board's own
        `min_clearance` is an unreliable edit-floor (often 0, sometimes a stale large
        value that would re-flood the DRC), so it is deliberately NOT used here.
    """
    fab = fab_floors(copper_layers)        # nominal (preferred) floor of the active tier
    fmin = fab_floor_min(copper_layers)    # deepest reachable floor (escalation/hard)

    def floor(constraint_val, fab_val):
        return max(constraint_val or 0.0, fab_val)

    return {
        'working_via_diameter': floor(constraints.get('min_via_diameter'), fab['via_diameter']),
        'working_via_drill':    floor(constraints.get('min_through_hole_diameter'), fab['via_drill']),
        # Advanced small via for fine-pitch escape only. The fab's smallest reachable
        # via (the rung 'standard' escalates to), not floored by the board's nominal.
        'fine_via_diameter':    fmin['via_diameter'],
        'fine_via_drill':       fmin['via_drill'],
        # DRC-enforced track floor: the board's own min_track_width if set, else the
        # fab minimum. min_track_width IS a DRC rule, so this is what DRC checks.
        'min_track_width':      floor(constraints.get('min_track_width'), fab['track_width']),
        # Fab PHYSICAL track minimum, NOT clamped to the board constraint. Dense/
        # congested signals route down to this (below the board's min_track_width,
        # like the human originals); it's the routing-capability floor, distinct
        # from the DRC floor above. The deepest reachable, so DRC grades pass it.
        'fab_track_width':      fmin['track_width'],
        # min_clearance is deliberately NOT pinned up here (see the docstring):
        # nothing downstream enforces it either -- check_drc does not raise its
        # clearance from the constraint -- so the fab floor IS the honest value.
        'drc_clearance':        fmin['clearance'],
        # #603: hole-to-hole and board-edge ARE pinned up by the board's own
        # DRC-enforced constraint everywhere else in the toolchain -- check_drc
        # raises both from the project (#439) and route.py adopts the constraint
        # when the flag is omitted -- so printing the bare fab minimum advertised
        # a floor nothing will honour. muzy_zynq2 declares min_hole_to_hole 0.25
        # and this line printed 0.2; the RUNBOOK tells the worker to take its
        # flags from here, so --hole-to-hole-clearance 0.2 went into every
        # command and was then silently graded at 0.25 anyway.
        'drc_hole_to_hole':     floor(constraints.get('min_hole_to_hole'), fmin['hole_to_hole']),
        # Pad-drill/pad-drill: the same board rule applies to any two holes, so
        # it floors at the constraint too (usually the JLC pad minimum is larger).
        'pad_hole_to_hole':     floor(constraints.get('min_hole_to_hole'), fmin['pad_hole_to_hole']),
        'drc_edge_clearance':   floor(constraints.get('min_copper_edge_clearance'), fmin['board_edge']),
        'fab': fab,
    }


def read_design_rules(pcb_path):
    """Read net-class design rules so a skill can pass them to the routing CLIs.

    Net classes live in the sibling .kicad_pro (KiCad 8+, JSON under
    net_settings.classes) or, for KiCad 6/7, in (net_class ...) blocks inside
    the .kicad_pcb. The router itself does not read these - it defaults to
    routing_defaults.CLEARANCE etc. - so the analysis step should read them
    here and pass --clearance/--track-width/--via-size/--via-drill (and the
    diff-pair gap) explicitly, instead of over-/under-clearancing the board.

    Returns {'classes': {name: {clearance, track_width, via_diameter,
    via_drill, diff_pair_gap, diff_pair_width}}, 'assignments': {net: class},
    'constraints': {min_clearance, min_track_width, min_via_diameter, ...},
    'copper_layers': int, 'effective': {...}, 'source': str}. Missing values
    are omitted per class. 'constraints' are the DRC-enforced Board Setup minima;
    'effective' combines them with the JLC fab floor (see effective_floors()).
    """
    classes = {}
    assignments = {}
    patterns = []       # list of (glob_pattern, class_name) from netclass_patterns
    constraints = {}
    source = None

    pro_path = os.path.splitext(pcb_path)[0] + '.kicad_pro'
    if os.path.exists(pro_path):
        try:
            with open(pro_path, encoding='utf-8') as f:
                pro = json.load(f)
            ns = pro.get('net_settings', {}) or {}
            for c in ns.get('classes', []) or []:
                name = c.get('name', 'Default')
                classes[name] = {k: c[k] for k in
                                 ('clearance', 'track_width', 'via_diameter',
                                  'via_drill', 'diff_pair_gap', 'diff_pair_width')
                                 if k in c}
            # net -> class assignment (dict in newer files; list of pairs in some).
            # KiCad 9 values can be a LIST of class names (a net may belong to
            # several classes); normalize to a single class name (prefer the first
            # non-Default) so downstream consumers see {net: classname}.
            na = ns.get('netclass_assignments') or {}
            if isinstance(na, dict):
                # KiCad 9 values can be a LIST of class names (a net may belong to
                # several classes); normalize to a single class name (prefer the
                # first non-Default) so downstream consumers see {net: classname}.
                for _net, _cls in na.items():
                    if isinstance(_cls, list):
                        _nd = [c for c in _cls if c and c != 'Default']
                        assignments[_net] = _nd[0] if _nd else (_cls[0] if _cls else 'Default')
                    elif _cls:
                        assignments[_net] = _cls
            # Wildcard assignments / net-name -> class glob patterns (KiCad 8+
            # net_settings.netclass_patterns): the primary membership mechanism on
            # many boards (explicit assignments are often empty). Consumers fnmatch
            # net names against it; explicit assignment wins. Kept as (pattern,
            # class) pairs, in file order.
            for pe in ns.get('netclass_patterns') or []:
                try:
                    if pe.get('netclass'):
                        patterns.append((pe.get('pattern', ''), pe['netclass']))
                except (KeyError, TypeError, AttributeError):
                    continue
            # DRC-enforced Board Constraints (what humans actually route against).
            rules = ((pro.get('board', {}) or {}).get('design_settings', {}) or {}).get('rules', {}) or {}
            constraints = {k: rules[k] for k in _CONSTRAINT_FIELDS if k in rules}
            if classes or constraints:
                source = pro_path
        except (json.JSONDecodeError, OSError):
            pass

    if not classes:
        # KiCad 6/7 fallback: (net_class "Name" (clearance X) (trace_width Y) ...)
        try:
            with open(pcb_path, encoding='utf-8') as f:
                content = f.read()
        except OSError:
            content = ''
        # KiCad 6/7: (net_class <name|"name"> "description" (clearance ..)
        # (trace_width ..) (via_dia ..) (via_drill ..) (add_net <name>) ...)
        for m in re.finditer(r'\(net_class\s+(?:"([^"]+)"|([^\s"]+))(.*?)\n\s*\)',
                             content, re.DOTALL):
            name = m.group(1) or m.group(2)
            body = m.group(3)
            cls = {}
            for src_key, key in (('clearance', 'clearance'), ('trace_width', 'track_width'),
                                 ('via_dia', 'via_diameter'), ('via_drill', 'via_drill'),
                                 ('diff_pair_gap', 'diff_pair_gap'),
                                 ('diff_pair_width', 'diff_pair_width')):
                vm = re.search(r'\(' + src_key + r'\s+([\d.]+)\)', body)
                if vm:
                    cls[key] = float(vm.group(1))
            classes[name] = cls
            for am in re.finditer(r'\(add_net\s+(?:"([^"]+)"|([^\s")]+))\)', body):
                assignments[am.group(1) or am.group(2)] = name
            source = pcb_path

    # Copper-layer count (for the fab-floor backstop) from the .kicad_pcb.
    copper_layers = 0
    try:
        with open(pcb_path, encoding='utf-8') as f:
            copper_layers = _count_copper_layers(f.read())
    except OSError:
        pass

    return {'classes': classes, 'assignments': assignments, 'patterns': patterns,
            'constraints': constraints, 'copper_layers': copper_layers,
            'effective': effective_floors(constraints, copper_layers),
            'source': source}


def board_default_netclass_param(pcb_path, key, design_rules=None):
    """Return the board's **Default** net-class value for ``key`` -- one of
    ``clearance``, ``track_width``, ``via_diameter``, ``via_drill`` -- from the
    sibling .kicad_pro (mm), or ``None`` when there is no project / no Default
    class / that value is unset. The routing CLIs and GUI use these as the default
    clearance/track/via when the corresponding flag/override is omitted (#439
    follow-up), so a board routes to its OWN Default class instead of the generic
    ``routing_defaults`` constants. Pass a cached ``read_design_rules`` result as
    ``design_rules`` to avoid re-reading."""
    try:
        dr = design_rules if design_rules is not None else read_design_rules(pcb_path)
    except Exception:
        return None
    v = ((dr.get('classes') or {}).get('Default') or {}).get(key)
    try:
        return float(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def board_default_netclass_clearance(pcb_path, design_rules=None):
    """The board's Default net-class **clearance** (mm), or ``None``. Thin wrapper
    over :func:`board_default_netclass_param` kept for existing callers."""
    return board_default_netclass_param(pcb_path, 'clearance', design_rules)


def board_constraint(pcb_path, key, design_rules=None):
    """Return a DRC-enforced Board Constraint (mm) from the sibling .kicad_pro's
    ``design_settings.rules`` -- e.g. ``min_hole_to_hole``,
    ``min_copper_edge_clearance``, ``min_clearance`` -- or ``None`` when there is
    no project / that constraint is unset. The routing CLIs use this so an omitted
    ``--hole-to-hole-clearance`` / ``--board-edge-clearance`` defaults to the
    board's OWN minimum (#439 follow-up) instead of a generic constant. Pass a
    cached ``read_design_rules`` result as ``design_rules`` to avoid re-reading."""
    try:
        dr = design_rules if design_rules is not None else read_design_rules(pcb_path)
    except Exception:
        return None
    v = (dr.get('constraints') or {}).get(key)
    try:
        return float(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def board_floor_declaration(pcb_path, design_rules=None):
    """What this board DECLARES about its own DRC floors -- and whether that is
    nothing at all.

    Returns ``{'classes': int, 'constraints': int, 'source': str|None,
    'declares_nothing': bool}``. ``declares_nothing`` is True when the board
    carries NO net class and NO board constraint from any source -- no sibling
    ``.kicad_pro``, and no KiCad 6/7 ``(net_class ...)`` block in the board
    file either.

    Why the graders need this rather than just a value (run-12 Tier 1.3): every
    floor accessor here answers ``None`` on such a board, and each grader then
    quietly substitutes its own constant (``routing_defaults.CLEARANCE`` 0.25,
    check_drc's 0.2). Measured on tigard, which ships no project: a whole
    placement baseline was graded against a fallback with nothing in the
    transcript recording that the number was a fallback rather than the board's
    own. That is the "grade at a floor the board was not routed to" failure
    CLAUDE.md warns about, reached from the other direction -- and unlike a
    board that declares 0.25, there is no value to compare against to notice it.

    Report-only. Nothing here changes a floor or an exit code; it exists so a
    grader can NAME its fallback.

    A board whose rules could not be READ answers ``declares_nothing: False``
    plus an ``error``, never True: "I could not look" is not "there is nothing
    there", and a disclosure that fires on a read failure is the kind of line
    readers learn to skip.
    """
    try:
        dr = design_rules if design_rules is not None else read_design_rules(pcb_path)
    except Exception as exc:                                   # noqa: BLE001
        return {'classes': 0, 'constraints': 0, 'source': None,
                'declares_nothing': False,
                'error': f'{type(exc).__name__}: {exc}'}
    classes = dr.get('classes') or {}
    constraints = dr.get('constraints') or {}
    return {'classes': len(classes), 'constraints': len(constraints),
            'source': dr.get('source'),
            'declares_nothing': not classes and not constraints}


def board_floor_knobs(pcb_path, clearance=None, board_edge_clearance=None,
                      clearance_default=0.25, edge_default=0.55,
                      design_rules=None):
    """Resolve grading/legality knobs BOARD-first (run-7 S1/S4).

    An explicit value wins; an unset one resolves from the board's own
    Default netclass clearance / min_copper_edge_clearance constraint; only
    a project-less board falls back to the fixed default. Grading a board at
    a fixed constant tighter than its own floor manufactures phantom
    violations (S1: check_floorplan oob), and one LOOSER lets real ones
    pass; the same mistake vetoed legal poses in converge (S4).

    Returns ``(clearance, board_edge_clearance, knobs)`` where ``knobs``
    records each value's source (``'cli'`` | ``'board netclass'`` |
    ``'board constraint'`` | ``'fixed default'``) for JSON disclosure.
    """
    # A non-positive declared value is UNSET, not a floor of zero -- KiCad
    # writes 0 into these fields for "not configured", and reading it as a real
    # floor collapses every consumer to no clearance at all. `board_floor`
    # below encodes the same rule; this helper predates it and did NOT, so a
    # project declaring `min_copper_edge_clearance: 0.0` silently gave
    # render_placement a 0.0 edge floor (every edge-halo and oob term
    # vanishing) where it had previously used 0.55.
    knobs = {}
    if clearance is None:
        v = board_default_netclass_clearance(pcb_path, design_rules)
        clearance, src = ((v, 'board netclass') if v is not None and v > 0
                          else (clearance_default, 'fixed default'))
    else:
        src = 'cli'
    knobs['clearance'] = {'value': clearance, 'source': src}
    if board_edge_clearance is None:
        v = board_constraint(pcb_path, 'min_copper_edge_clearance',
                             design_rules)
        board_edge_clearance, src = ((v, 'board constraint')
                                     if v is not None and v > 0
                                     else (edge_default, 'fixed default'))
    else:
        src = 'cli'
    knobs['board_edge_clearance'] = {'value': board_edge_clearance,
                                     'source': src}
    return clearance, board_edge_clearance, knobs


# Where each floor legitimately comes from: (Default-netclass key, board
# constraint key). None means "this floor is not expressed there".
#
# `clearance` deliberately has NO constraint fallback. `min_clearance` is an
# unreliable edit-floor -- it is 0.0 on the measured board and stale-large on
# others (see effective_floors' note at :86) -- so falling back to it would
# resolve a board declaring a 0.2 netclass to 0.0 and relax every consumer to
# nothing. The netclass is the honest source for clearance; the constraints are
# the honest source for the hole/edge floors, which no netclass expresses.
_FLOOR_SOURCES = {
    'clearance':            ('clearance',    None),
    'track_width':          ('track_width',  'min_track_width'),
    'via_diameter':         ('via_diameter', 'min_via_diameter'),
    'via_drill':            ('via_drill',    None),
    'board_edge_clearance': (None,           'min_copper_edge_clearance'),
    'hole_clearance':       (None,           'min_hole_clearance'),
    'hole_to_hole':         (None,           'min_hole_to_hole'),
}


def board_floor(pcb_path, name, explicit=None, fallback=None,
                design_rules=None):
    """ONE floor, resolved BOARD-FIRST. Returns ``(value, source)``.

    Precedence, the same one `board_floor_knobs` uses and with the same source
    vocabulary (``'cli'`` | ``'board netclass'`` | ``'board constraint'`` |
    ``'fixed default'``): an explicit value wins, then the board's own Default
    netclass, then its board constraint, then the caller's fallback.

    This exists because instruments kept substituting their own constant for a
    value the board declares, each in its own way, and each wrong in a
    different direction. Measured here: `check_channels` at its old 0.25/0.3
    constants reported 334 escape lanes where the board's own 0.2/0.254 floor
    gives 399 -- 65 lanes understated. (The originating report measured 304 vs
    378 on the same board at 0.2/0.2; same direction and scale, different track
    width.) It also invented a deficit on a face that had none -- a phantom
    that would have steered a placement search. `obstacle_map` priced NPTH at
    a hardcoded max(clearance, 0.20) while the board declared
    ``min_hole_clearance`` 0.25, and a route came within 0.2263 mm of an NPTH:
    a real 0.0237 mm violation, routing-introduced.

    A non-positive constraint is treated as UNSET rather than as a floor of
    zero. KiCad writes 0 for "not configured" in these fields, and reading it
    as a genuine 0 relaxes the consumer to nothing -- the same trap that keeps
    `min_clearance` out of the table above.

    IT IS NOT RAISE-ONLY, and nothing here pretends otherwise. Once a declared
    value is positive it is returned as-is, with NO max() against `fallback`,
    so a board can resolve a floor DOWNWARDS. That is the point for a tool
    that GRADES EXISTING COPPER: `check_assembly` must measure at the board's
    own clearance even when it sits below the packaged default, or it
    manufactures phantom violations on copper placed correctly (CLAUDE.md,
    "Grade DRC at the clearance the board was actually routed to").

    THE DISTINCTION IS GRADE-vs-PREDICT, not tool-by-tool. `check_channels`
    was listed here as another such consumer and that was wrong: it does not
    grade copper, it PREDICTS routability, so a declared lane pitch finer than
    the fab can etch makes it promise capacity nobody can build. Measured on
    tigard --refs U3 (fab floors 0.09 / 0.0762): a declared 0.05/0.05 took its
    deficit faces from 3 to 1 and U3's supply from 29 to 120, hiding two real
    deficits. It now wraps at the fab floor (check_channels.py, `_fab`). A
    predictive consumer needs the wrap; a grading one must not have it.

    A consumer for which downward is a FAB question must therefore wrap this
    in its own max(), exactly as `resolve_hole_clearance`'s consumers do
    (obstacle_map.py:1580, plane_obstacle_builder.py:1208) -- that helper is
    called "raise-only" only because of those wraps, never on its own.
    Measured: a project declaring ``min_hole_to_hole: 0.10`` resolves here to
    ``(0.1, 'board constraint')``, and the qfn underpad escape spaced this
    run's drills at 0.10 -- below the 0.20 JLC fab floor -- until it grew the
    wrap (qfn_fanout/__init__.py, ``_h2h_fab``). The routing CLIs get the same
    protection from `enforce_fab_floors`, which runs on args right after
    `resolve_cli_floor`; an engine-internal read like the qfn one does not,
    because that pins a value it never reads.
    """
    if name not in _FLOOR_SOURCES:
        raise KeyError(f"unknown floor {name!r}; "
                       f"known: {', '.join(sorted(_FLOOR_SOURCES))}")
    if explicit is not None:
        return float(explicit), 'cli'
    cls_key, con_key = _FLOOR_SOURCES[name]
    # The guard wraps the ACCESSORS too, not just read_design_rules: a caller
    # may hand in its own `design_rules`, and a malformed one raises inside
    # board_default_netclass_param / board_constraint rather than here.
    try:
        dr = (design_rules if design_rules is not None
              else read_design_rules(pcb_path))
        if cls_key:
            v = board_default_netclass_param(pcb_path, cls_key, dr)
            if v is not None and v > 0:
                return float(v), 'board netclass'
        if con_key:
            v = board_constraint(pcb_path, con_key, dr)
            if v is not None and v > 0:
                return float(v), 'board constraint'
    except Exception:                                          # noqa: BLE001
        # "I could not look" is NOT "there is nothing there" -- the same
        # distinction board_floor_declaration exists to preserve. The VALUE is
        # the fallback either way, but the source must not claim the board was
        # read and found silent, or a corrupt .kicad_pro reads in a table
        # exactly like a board that genuinely declares nothing.
        return _num(fallback), 'unreadable project'
    return _num(fallback), 'fixed default'


def _num(v):
    """Fallbacks are returned in the same type the board paths return."""
    return None if v is None else float(v)


def resolve_cli_floor(pcb_path, name, explicit, fallback, flag):
    """One routing-CLI geometry floor, resolved board-first and ANNOUNCED with
    its real source. Returns the value.

    THE SPLIT THIS CLOSES. `board_floor` / `board_floor_knobs` /
    `resolve_hole_clearance` -- and the GUI's own
    `_effective_plane_edge_clearance` -- all treat a declared 0 as UNSET,
    because that is what KiCad writes into these fields for "not configured".
    The four routing mains did not: each carried its own copy of
    ``board_constraint(...) if ... is not None else <default>``, an
    ``is not None`` test with no positivity guard, eight sites in all
    (route.py 3725/3731, route_diff.py 1947/1953, route_planes.py 4940/4946,
    route_disconnected_planes.py 3410/3416).

    So on a board declaring ``min_copper_edge_clearance: 0.0`` the two halves
    of the place/route loop read one declared floor two ways: the placement
    half (render_placement, check_floorplan, converge -- all via
    board_floor_knobs) resolved 0.55 ``[fixed default]``, while the routing
    half took a REAL floor of 0.0 and printed *"using the board
    min_copper_edge_clearance 0.0mm"* -- claiming the board had declared what
    it had in fact left unset. The plane engines were worse than misleading:
    a declared 0.0 dropped their zone inset from PLANE_EDGE_CLEARANCE 0.5 to
    0.0, while the GUI's plane tab held 0.5 because it already had the guard.

    The FALLBACKS legitimately differ between callers and are NOT unified
    here: the signal mains fall back to BOARD_EDGE_CLEARANCE 0.0, which is a
    deliberate sentinel meaning "no edge rule declared, use the copper-copper
    clearance instead" (route.py:896, obstacle_map.py:797), while the plane
    mains fall back to PLANE_EDGE_CLEARANCE 0.5 and the placement model to
    0.55. What must agree -- and now does -- is whether the board declared
    anything at all, and what the tool SAYS about where its number came from.

    The source tag is printed in the same ``[board constraint]`` /
    ``[fixed default]`` / ``[unreadable project]`` vocabulary the placement
    instruments use, because "fixed default" means the board declared nothing,
    not that it agreed.
    """
    value, src = board_floor(pcb_path, name, explicit, fallback)
    if src != 'cli':
        print(f"{flag} not given; using {value}mm [{src}].")
    return value


def net_clearance_map_by_id(pcb_path, nets, design_rules=None):
    """Resolve each net to its net-class clearance (mm) from the sibling
    .kicad_pro netclasses, for the routing CLIs' cross-class clearance map.

    KiCad's required spacing between two nets of different classes is
    max(classA, classB). The router carries a {net_id: class_clearance} map and
    prices every foreign/in-run obstacle at max(routing-side floor, that net's
    class clearance). This helper builds that map so route.py / route_diff.py can
    auto-honor the board's netclasses without a hand-written --net-clearances JSON.

    `nets` is {net_id: net_name} (e.g. {nid: n.name for nid, n in pcb.nets.items()}).
    A net's classes come from BOTH the explicit ``netclass_assignments`` and every
    matching ``netclass_patterns`` glob (KiCad merges memberships); its effective
    clearance is the MAX over those classes' clearances (KiCad enforces the strictest).

    A net is returned iff it belongs to at least one NON-Default net class -- with
    that class's clearance, EVEN WHEN it equals or is below the Default class
    clearance (#439). This is the fix for the old "eff > Default" gate, which
    dropped a net whose class equalled Default: on an all-0.2-classes board (zynq
    -- Default/ADDRES/DQ* all 0.2) the map came back EMPTY, so those nets routed
    at the smaller ``--clearance`` instead of their 0.2 class and shipped
    DRC-violating. Nets that resolve ONLY to Default are still omitted (they use
    the router's config.clearance = ``--clearance``, which IS the Default/routing
    clearance). The map value is a FLOOR the caller max()es with config.clearance,
    so a non-Default class BELOW ``--clearance`` is inert -- correct: ``--clearance``
    is the routing floor for those too. Returns {net_id: clearance_mm}.

    Pattern matching is glob-style (KiCad ``*``/``?`` wildcards). Exotic patterns that
    do not translate to a glob simply fail to match -> that net falls back to
    config.clearance (the safe/inert direction), never over-blocked.
    """
    dr = design_rules if design_rules is not None else read_design_rules(pcb_path)
    classes = dr.get('classes') or {}

    def _class_clr(cname):
        c = classes.get(cname) or {}
        v = c.get('clearance')
        return float(v) if isinstance(v, (int, float)) else None

    out = {}
    for nid, cand in net_class_memberships(pcb_path, nets,
                                           design_rules=dr).items():
        # Only NON-Default class memberships matter: a Default-only net routes at
        # config.clearance. Take the strictest (max) NON-Default class clearance.
        clrs = [c for c in (_class_clr(cn) for cn in cand if cn != 'Default')
                if c is not None]
        if not clrs:
            continue
        out[nid] = max(clrs)
    return out


def net_class_memberships(pcb_path, nets, design_rules=None):
    """{net_id: set of net-class names} from the sibling .kicad_pro: the
    explicit ``netclass_assignments`` entry UNION every matching
    ``netclass_patterns`` glob (KiCad merges memberships). The SHARED membership
    resolver -- net_clearance_map_by_id and the .kicad_dru track-clearance
    channel both resolve through this, so router and grader cannot drift on who
    is in a class. Nets with no membership are omitted (not mapped to set()).

    Pattern matching is glob-style; an exotic pattern that does not translate
    simply fails to match (safe/inert direction)."""
    import fnmatch
    dr = design_rules if design_rules is not None else read_design_rules(pcb_path)
    assignments = dr.get('assignments') or {}
    patterns = dr.get('patterns') or []
    out = {}
    for nid, name in nets.items():
        if not name:
            continue
        cand = set()
        a = assignments.get(name)
        if a:
            cand.add(a)
        for pat, cname in patterns:
            if not pat:
                continue  # empty pattern = the Default catch-all
            try:
                if fnmatch.fnmatchcase(name, pat):
                    cand.add(cname)
            except Exception:
                pass
        if cand:
            out[nid] = cand
    return out


def net_track_width_map_by_id(pcb_path, nets, design_rules=None):
    """Resolve each net to its net-class TRACK WIDTH (mm) -- the single-ended
    companion to net_clearance_map_by_id, so route.py can route each net at its OWN
    class width when --track-width is omitted (a controlled-impedance signal class or
    a power class, each with a different width). Unlike clearance (KiCad enforces the
    strictest max), a net's width is the width of its ASSIGNED class, so this uses
    resolve_net_class (explicit assignment, else the first matching pattern).

    `nets` is {net_id: net_name}. Returns {net_id: width_mm} for nets whose resolved
    class is NON-Default AND defines a track_width (Default-only nets use
    config.track_width = the board Default class width already). Empty when the board
    has no netclass widths. The caller floors each value at the fab minimum."""
    dr = design_rules if design_rules is not None else read_design_rules(pcb_path)
    classes = dr.get('classes') or {}
    if not classes:
        return {}
    out = {}
    for nid, name in nets.items():
        if not name:
            continue
        cname = resolve_net_class(name, dr)
        if cname == 'Default':
            continue
        w = (classes.get(cname) or {}).get('track_width')
        if isinstance(w, (int, float)):
            out[nid] = float(w)
    return out


def resolve_net_class(net_name, rules):
    """Class name for a net under `rules` (a read_design_rules() result):
    explicit assignment wins, else the first matching wildcard pattern
    (KiCad 8+ netclass_patterns), else 'Default'."""
    cls = rules.get('assignments', {}).get(net_name)
    if cls:
        return cls
    for pattern, pcls in rules.get('patterns', []):
        if fnmatchcase(net_name, pattern):
            return pcls
    return 'Default'


def net_clearance_map(pcb_path, net_names, rules=None):
    """Per-net netclass clearance for #326 grading/routing parity.

    Returns {net_name: clearance_mm} for every net in `net_names` whose
    resolved class defines a clearance -- including Default, so the caller
    can max() it with its own global value. Empty dict when the board has
    no netclass data. Pass a read_design_rules() result via `rules` to
    avoid re-reading."""
    if rules is None:
        rules = read_design_rules(pcb_path)
    classes = rules.get('classes', {})
    if not classes:
        return {}
    out = {}
    for name in net_names:
        if not name:
            continue
        cl = classes.get(resolve_net_class(name, rules), {}).get('clearance')
        if cl:
            out[name] = float(cl)
    return out


def print_design_rules(pcb_path):
    """Human + skill-friendly dump of the board's net-class design rules."""
    dr = read_design_rules(pcb_path)
    eff = dr['effective']
    if not dr['classes'] and not dr['constraints']:
        print("No net classes or Board Constraints found (no sibling .kicad_pro "
              "and no (net_class) in the PCB).")
        print(f"Falling back to JLCPCB fab floors for {dr['copper_layers'] or '?'}-"
              "layer board — route/DRC at these:")
        print(f"  working via {eff['working_via_diameter']}/{eff['working_via_drill']}  "
              f"clearance {eff['drc_clearance']}  hole-to-hole {eff['drc_hole_to_hole']}")
        return
    src = os.path.basename(dr['source']) if dr['source'] else '?'
    print(f"\nNet-class design rules (from {src}):")
    for name, c in sorted(dr['classes'].items()):
        fields = "  ".join(f"{k}={c[k]}" for k in
                           ('clearance', 'track_width', 'via_diameter', 'via_drill',
                            'diff_pair_gap', 'diff_pair_width') if k in c)
        print(f"  [{name}] {fields}")
    non_default = {n: c for n, c in dr['assignments'].items() if c != 'Default'}
    if non_default:
        print(f"  ({len(non_default)} net(s) assigned to non-default classes — "
              f"route those separately with that class's flags)")

    # The DRC-enforced Board Constraints. Net-class track_width/via_diameter are
    # only drawing DEFAULTS; ONLY these min_* are DRC floors (#111/#115).
    if dr['constraints']:
        cons = "  ".join(f"{k}={v}" for k, v in dr['constraints'].items())
        print(f"\nBoard Constraints (DRC-enforced minima, {dr['copper_layers']}-layer): {cons}")
    _tier, _ovr = get_default_fab_tier()
    _tier_desc = f"{_tier}" + (f" + overrides({','.join(sorted(_ovr))})" if _ovr else "")
    print(f"Manufacturing floor (active fab tier: {_tier_desc}; Constraint or JLC fab "
          f"min, whichever is larger): "
          f"via {eff['working_via_diameter']}/{eff['working_via_drill']}  "
          f"clearance {eff['drc_clearance']}  hole-to-hole {eff['drc_hole_to_hole']}  "
          f"edge {eff['drc_edge_clearance']}  "
          f"track {eff['min_track_width']} (DRC track floor = board min_track_width)")
    # #603: name the constraint when it -- not the fab -- set the hole-to-hole
    # floor. The RUNBOOK tells the worker to route and grade from this line, and
    # a board minimum ABOVE the JLC figure is exactly the case where the two
    # disagreed: the printed 0.2 went into every command and check_drc graded at
    # the board's 0.25 anyway.
    _h2h_con = float((dr['constraints'] or {}).get('min_hole_to_hole') or 0.0)
    if _h2h_con > eff['fab']['hole_to_hole']:
        print(f"  - hole-to-hole {eff['drc_hole_to_hole']} comes from the BOARD's "
              f"min_hole_to_hole ({_h2h_con}), above the JLC fab min "
              f"{eff['fab']['hole_to_hole']}: it is DRC-enforced, so route AND grade "
              "at it (check_drc raises a lower --hole-to-hole-clearance to it anyway).")
    # Copper clearance is the one floor NOT pinned up by the board's constraint
    # -- min_clearance is an unreliable edit-floor (often 0, sometimes stale-large)
    # and nothing downstream enforces it, so the fab minimum is the honest value.
    _clr_con = float((dr['constraints'] or {}).get('min_clearance') or 0.0)
    if _clr_con > eff['drc_clearance']:
        print(f"  - clearance {eff['drc_clearance']} is the FAB floor; the board's "
              f"min_clearance ({_clr_con}) is deliberately NOT applied here (it is an "
              "aspirational edit-floor, and grading above what was routed manufactures "
              "phantom violations -- #439). Fine-pitch escapes route down to the fab floor.")
    # The router must honour these as DISTINCT rules (issue #125):
    print(f"  - via hole-to-hole {eff['drc_hole_to_hole']} = drill-to-drill minimum, "
          "net-INDEPENDENT (via/via and via/pad-drill, all nets incl. same-net); "
          f"pad-drill/pad-drill is the larger {eff['pad_hole_to_hole']} (JLC pad "
          "hole-to-hole, fixed by placement).")
    print(f"  - copper clearance {eff['drc_clearance']} = via/pad and via/via copper, "
          "between DIFFERENT nets. Same-net via-pad copper clearance is 0 "
          "(via-in-pad) where the fab allows it; hole-to-hole still applies.")
    # Advanced small via for fine-pitch escape (4+ layer). Standard via can't
    # dog-bone / via-in-pad sub-0.5mm BGA/QFN balls (issues #99/#122).
    if eff['fine_via_diameter'] < eff['fab']['via_diameter']:
        print(f"  - fine-pitch escape via {eff['fine_via_diameter']}/{eff['fine_via_drill']} "
              "(JLC advanced, small extra cost): use ONLY for sub-~0.5mm-pitch "
              "BGA/QFN escape (bga_fanout/qfn_fanout/route_diff for those parts); "
              "keep the standard via for general routing.")
    # Fab PHYSICAL track floor, distinct from the DRC track floor above. Surfaced
    # only when the fab can go thinner than the board's min_track_width — that gap
    # is where dense-board completion is won (the human originals route there).
    if eff['fab_track_width'] < eff['min_track_width']:
        print(f"  - fine-pitch signal track floor {eff['fab_track_width']} "
              f"({dr['copper_layers']}-layer fab min; the board's min_track_width is "
              f"{eff['min_track_width']}): on DENSE/congested boards route ordinary "
              "signals at this width (route.py --track-width), BELOW the board's own "
              "rule, like the human originals — thinner is more complete AND faster. "
              "Keep power nets on --power-nets and impedance nets at net-class width.")

    # Routing flags: track_width is a per-class MINIMUM (keep, for current/
    # impedance); clearance is the per-class default. But the VIA uses the small
    # working floor, NOT the net-class via_diameter (which is just a max-like
    # default) -- emitting the nominal everywhere is #115.
    d = dr['classes'].get('Default') or (next(iter(dr['classes'].values())) if dr['classes'] else {})
    flags = []
    if 'clearance' in d:   flags.append(f"--clearance {d['clearance']}")
    if 'track_width' in d: flags.append(f"--track-width {d['track_width']}")
    flags.append(f"--via-size {eff['working_via_diameter']}")
    flags.append(f"--via-drill {eff['working_via_drill']}")
    print("\nSUGGESTED route.py/qfn_fanout/bga_fanout/route_planes flags "
          "(Default class; small working via):\n  " + " ".join(flags))
    print("  Fine-pitch escape may drop --clearance toward the manufacturing floor "
          f"{eff['drc_clearance']} (never below); route non-Default-class nets separately.")
    if eff['fab_track_width'] < eff['min_track_width']:
        print(f"  On dense/congested boards, drop --track-width to the fab floor "
              f"{eff['fab_track_width']} for ordinary signals (keep --power-nets and "
              "impedance nets wide); thinner completes more nets and routes faster.")

    if 'diff_pair_gap' in d or 'diff_pair_width' in d:
        dp = []
        if 'diff_pair_width' in d: dp.append(f"--track-width {d['diff_pair_width']}")
        if 'diff_pair_gap' in d:   dp.append(f"--diff-pair-gap {d['diff_pair_gap']}")
        print("SUGGESTED route_diff.py flags (Default class):\n  " + " ".join(dp))

    # Verification: DRC/connectivity at the manufacturability floor -- the rule
    # the human original passes -- NOT the inflated net-class clearance (#111).
    print(f"\nSUGGESTED check_drc.py flags (grade at the manufacturing floor):\n  "
          f"--clearance {eff['drc_clearance']} "
          f"--hole-to-hole-clearance {eff['drc_hole_to_hole']} "
          f"--board-edge-clearance {eff['drc_edge_clearance']}")


def net_islands(pcb, net_id, tol=0.05):
    """The net's copper as ISLANDS: union-find over coincident points.

    Returns a list of components, largest first; each component is a list of
    `(kind, obj, points)` where kind is 'seg' | 'via' | 'pad'. Two items join
    when a point of one lies within `max(tol, radius_a, radius_b)` of a point
    of the other -- a pad's radius is half its larger dimension, a via's half
    its diameter, a track endpoint's a 60 um nub.

    Lives HERE, not in a py_tools CLI, because two front-ends need it:
    `net_forensics` (which is where it was written) and `check_reachability`'s
    auto-widen, which has to find the nearest OTHER island of a net without
    rasterising the board. The reachability tool used to reach across and
    import `net_forensics._components` -- a sibling CLI's private name, from a
    directory `_path` does not put on `sys.path`, so the ImportError would have
    escaped as exit **1**, which in that tool is the CAGED geometry verdict.

    The pair scan is bucketed by a uniform grid at the largest join radius, so
    a net with thousands of endpoints does not pay the O(P^2) all-pairs walk
    the first version did. The join rule is unchanged.
    """
    items = []
    for s in pcb.segments:
        if s.net_id == net_id:
            items.append(('seg', s, [(s.start_x, s.start_y),
                                     (s.end_x, s.end_y)]))
    for v in pcb.vias:
        if v.net_id == net_id:
            items.append(('via', v, [(v.x, v.y)]))
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if p.net_id == net_id:
                items.append(('pad', p, [(p.global_x, p.global_y)]))
    parent = list(range(len(items)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    pts = []
    for idx, (_, obj, ps) in enumerate(items):
        r = (max(obj.size_x, obj.size_y) / 2 if hasattr(obj, 'size_x')
             else getattr(obj, 'size', 0) / 2 or 0.06)
        for (x, y) in ps:
            pts.append((x, y, idx, r))
    # Bucket at the largest join radius: two points can only join if they are
    # within it, so only the 3x3 neighbourhood of a point's cell can hold a
    # partner. Same answer as the all-pairs scan, without the P^2.
    # `or tol` was not a guard: with tol=0 and every radius 0 (a caller
    # asking for exact coincidence only) the fallback is 0 too, and the
    # bucket index divides by it. Any positive cell is correct here -- the
    # 3x3 neighbourhood still contains every point within `cell`.
    cell = max([tol] + [p[3] for p in pts]) or 1e-6
    buckets = defaultdict(list)
    for i, (x, y, _a, _r) in enumerate(pts):
        buckets[(int(math.floor(x / cell)), int(math.floor(y / cell)))].append(i)
    for i, (x1, y1, a, r1) in enumerate(pts):
        cx, cy = int(math.floor(x1 / cell)), int(math.floor(y1 / cell))
        for gx in (cx - 1, cx, cx + 1):
            for gy in (cy - 1, cy, cy + 1):
                for j in buckets.get((gx, gy), ()):
                    if j <= i:
                        continue
                    x2, y2, b, r2 = pts[j]
                    if a != b and math.hypot(x1 - x2, y1 - y2) <= max(tol, r1, r2):
                        union(a, b)
    comps = defaultdict(list)
    for idx in range(len(items)):
        comps[find(idx)].append(items[idx])
    return sorted(comps.values(), key=len, reverse=True)


def island_gap(comp_a, comp_b):
    """Closest approach between two islands: (mm, (x, y), (x, y)).

    The unclosed MST edge between them, which is the number a rip-set decision
    needs. `(None, None, None)` when either island has no points. One
    implementation because there were two, in the same file, one printing and
    one returning a dict, and they could drift.
    """
    best = (None, None, None)
    for _k, _o, ps in comp_a:
        for p1 in ps:
            for _k2, _o2, ps2 in comp_b:
                for p2 in ps2:
                    d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                    if best[0] is None or d < best[0]:
                        best = (d, p1, p2)
    return best


def find_differential_pairs(pcb_data):
    """Find differential pairs based on common naming conventions.

    Delegates to net_queries.extract_diff_pair_base so this report and the
    route_diff/fanout engines recognize the exact same conventions (issue #91:
    DDR _t/_c case + no-separator channels, USB DP/DM and DPLUS/DMINUS).
    """
    from net_queries import extract_diff_pair_base

    net_names = [n.name for n in pcb_data.nets.values() if n.name]
    name2net = {n.name: n for n in pcb_data.nets.values() if n.name}

    # Key by (base, style) so a net only pairs within its own convention.
    halves = {}  # (base, style) -> {True: pos_name, False: neg_name}
    for name in net_names:
        result = extract_diff_pair_base(name)
        if result is None:
            continue
        base, is_pos, style = result
        halves.setdefault((base, style), {})[is_pos] = name

    found_pairs = []
    for (base, style), sides in halves.items():
        if True in sides and False in sides:
            # Issue #145: a P/N name on a 2-terminal crystal/oscillator (e.g.
            # watchy /32K_P+/32K_N on Y1) is not a differential signal - skip it.
            if _is_two_terminal_resonator_pair(pcb_data, name2net, sides[True], sides[False]):
                continue
            found_pairs.append((sides[True], sides[False]))

    found_pairs.sort()
    return found_pairs


def _is_two_terminal_resonator_pair(pcb_data, name2net, pos_name, neg_name):
    """True if both nets land on a single crystal/oscillator footprint - a
    2-terminal resonator, not a real differential signal (issue #145).

    Identified as a footprint that BOTH nets connect to and that is crystal-like
    either by name (crystal/xtal/resonator/oscillator) or by a Y*/X* reference,
    with a small pad count (<=4, allowing a case-ground crystal). A real diff
    pair never shares a single small crystal-like part."""
    import re

    def refs_of(net_name):
        net = name2net.get(net_name)
        return {p.component_ref for p in net.pads if p.component_ref} if net else set()

    for ref in refs_of(pos_name) & refs_of(neg_name):
        fp = pcb_data.footprints.get(ref)
        if not fp or len(fp.pads) > 4:
            continue
        fp_name = (fp.footprint_name or '').lower()
        name_xtal = any(k in fp_name for k in ('crystal', 'xtal', 'resonator', 'oscillator'))
        ref_xtal = bool(re.match(r'^[XY]\d', ref))
        if name_xtal or ref_xtal:
            return True
    return False


def net_outer_pad_profile(pcb_data, net_name):
    """Per-outer-layer SMD pad counts + plated-TH count for a net (#562).

    An outer-layer flood of this net connects its same-layer SMD pads by
    fill contact (pour-direct, NO via each), and its plated-TH barrels on
    every layer. Planners should weigh this when choosing which net floods
    F.Cu/B.Cu: the net with the most SMD pads on a layer saves the most
    vias when poured there. Returns ({'F.Cu': n, 'B.Cu': n}, th_count).
    """
    from kicad_parser import pad_is_plated_through
    prof = {}
    th = 0
    net = next((n for n in pcb_data.nets.values() if n.name == net_name), None)
    for p in (net.pads if net else []):
        if pad_is_plated_through(p):
            th += 1
        elif not p.drill:
            for lay in ('F.Cu', 'B.Cu'):
                if lay in p.layers:
                    prof[lay] = prof.get(lay, 0) + 1
    return prof, th


def _fmt_outer_profile(pcb_data, net_name):
    prof, th = net_outer_pad_profile(pcb_data, net_name)
    parts = [f"{lay} {n} SMD" for lay, n in sorted(prof.items()) if n]
    if th:
        parts.append(f"TH {th}")
    return f"  ({', '.join(parts)})" if parts else ""


def find_power_nets(pcb_data):
    """Find power and ground nets by name patterns and connection count.

    Returns (gnd_nets, vcc_nets, candidate_nets) - candidates are unmatched
    nets whose pad count rivals the detected power rails (issue #91: rails
    like '-12V', '/5V', '/3V3', and bare 'VDC' defeat prefix patterns; pad
    count catches them).
    """
    import re
    gnd_patterns = ['GND', 'VSS', 'AGND', 'DGND', 'PGND', 'GNDA', 'GNDD']
    vcc_patterns = ['VCC', 'VDD', '+3.3', '+5', '+12', '+1.8', '+2.5', 'VBUS', 'VBAT', 'VIN']
    # Rail-shaped names, checked against the LAST hierarchical path component:
    # +5V, -12V, 5V, 3V3, 1V8, 12V0, 3.3V ...
    rail_re = re.compile(r'^[+-]?(\d+(\.\d+)?V\d*|\d+V\d+)$')

    gnd_nets = []
    vcc_nets = []
    matched = set()

    for net in pcb_data.nets.values():
        if not net.name:
            continue
        name_upper = net.name.upper()
        leaf = name_upper.rsplit('/', 1)[-1]
        pad_count = len(net.pads)

        if any(g in name_upper for g in gnd_patterns):
            gnd_nets.append((net.name, pad_count))
            matched.add(net.name)
        elif (any(v in name_upper for v in vcc_patterns)
              or (net.name[0] in '+-' and any(c.isdigit() for c in net.name))
              or rail_re.match(leaf)):
            vcc_nets.append((net.name, pad_count))
            matched.add(net.name)

    # Sort by pad count descending
    gnd_nets.sort(key=lambda x: -x[1])
    vcc_nets.sort(key=lambda x: -x[1])

    # High pad-count nets the patterns missed: anything rivaling the known
    # POWER rails is probably a power net with an unconventional name (e.g.
    # VDC). Threshold from the largest vcc rail, not GND - GND's pad count
    # dwarfs everything and would hide real rails.
    vcc_counts = [c for _, c in vcc_nets]
    threshold = max(10, max(vcc_counts) // 2) if vcc_counts else 10
    candidates = sorted(
        ((net.name, len(net.pads)) for net in pcb_data.nets.values()
         if net.name and net.name not in matched
         and not net.name.lower().startswith('unconnected')
         and len(net.pads) >= threshold),
        key=lambda x: -x[1])[:8]

    return gnd_nets, vcc_nets, candidates


def find_high_connection_nets(pcb_data, top_n=10):
    """Find nets with the most connections (often power/ground)."""
    nets_by_count = []
    for net in pcb_data.nets.values():
        if net.name and not net.name.startswith('unconnected'):
            nets_by_count.append((net.name, len(net.pads)))

    nets_by_count.sort(key=lambda x: -x[1])
    return nets_by_count[:top_n]


def main():
    parser = argparse.ArgumentParser(
        description='List and analyze nets in a KiCad PCB file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--component', '-c', help='Component reference (e.g., U1)')
    parser.add_argument('--pads', action='store_true', help='Show pad-to-net assignments')
    parser.add_argument('--diff-pairs', '-d', action='store_true', help='Detect differential pairs')
    parser.add_argument('--power', '-p', action='store_true', help='Show power/ground nets')
    parser.add_argument('--design-rules', '-r', action='store_true',
                        help="Show net-class design rules (clearance/track/via/diff-pair) "
                             "from the .kicad_pro (or .kicad_pcb net_class) and the CLI "
                             "flags to pass them to the routing tools")
    parser.add_argument('--top', '-t', type=int, default=10, help='Show top N most-connected nets (default: 10)')
    parser.add_argument('--pattern', help='Filter nets by glob pattern')

    add_fab_tier_args(parser)
    args = parser.parse_args()
    set_default_fab_tier(*fab_tier_from_args(args))

    # Design rules read from project metadata, not the parsed PCB object.
    if args.design_rules:
        print_design_rules(args.pcb)
        if not (args.component or args.diff_pairs or args.power):
            return

    pcb_data = parse_kicad_pcb(args.pcb)

    # If no specific analysis requested and no component, show help
    if not args.component and not args.diff_pairs and not args.power:
        # Default: show board summary and top connected nets
        print(f"\nBoard: {args.pcb}")
        print(f"Total nets: {len(pcb_data.nets)}")
        print(f"Total components: {len(pcb_data.footprints)}")
        print(f"\nTop {args.top} most-connected nets:")
        for name, count in find_high_connection_nets(pcb_data, args.top):
            print(f"  {name}: {count} pads")
        print(f"\nUse --component/-c to list nets on a component")
        print(f"Use --diff-pairs/-d to detect differential pairs")
        print(f"Use --power/-p to show power/ground nets")
        return 0

    # Differential pair detection
    if args.diff_pairs:
        pairs = find_differential_pairs(pcb_data)
        print(f"\nDifferential Pairs ({len(pairs)} found):")
        if pairs:
            for p, n in pairs:
                print(f"  {p}  /  {n}")
        else:
            print("  None detected")
        print()

    # Power net detection
    if args.power:
        gnd_nets, vcc_nets, candidates = find_power_nets(pcb_data)
        print(f"\nGround Nets ({len(gnd_nets)} found):")
        for name, count in gnd_nets:
            print(f"  {name}: {count} pads{_fmt_outer_profile(pcb_data, name)}")
        print(f"\nPower Nets ({len(vcc_nets)} found):")
        for name, count in vcc_nets:
            print(f"  {name}: {count} pads{_fmt_outer_profile(pcb_data, name)}")
        if candidates:
            print(f"\nHigh pad-count nets NOT matched by power patterns "
                  f"(possible power rails - verify):")
            for name, count in candidates:
                print(f"  {name}: {count} pads{_fmt_outer_profile(pcb_data, name)}")
        print("\nOuter-layer flood hint: SMD counts above are pads an F.Cu/B.Cu "
              "pour of that net would reach by fill contact (no via each); "
              "TH barrels connect on every layer. When picking the net to "
              "flood an outer layer, prefer the one with the most SMD pads "
              "on that layer.")
        print()

    # Component-specific listing
    if args.component:
        # Auto-detect BGA component if 'auto' specified
        if args.component.lower() == 'auto':
            bga_components = find_components_by_type(pcb_data, 'BGA')
            if bga_components:
                args.component = bga_components[0].reference
                print(f"Auto-detected BGA component: {args.component}")
            else:
                print("Error: No BGA components found. Please specify --component")
                print(f"Available: {list(pcb_data.footprints.keys())[:20]}...")
                return 1

        if args.component not in pcb_data.footprints:
            print(f"Error: Component {args.component} not found")
            print(f"Available: {sorted(pcb_data.footprints.keys())}")
            return 1

        footprint = pcb_data.footprints[args.component]

        if args.pads:
            # Show pad-to-net assignments
            print(f"\nPads on {args.component} ({len(footprint.pads)} pads):\n")
            pads_sorted = sorted(footprint.pads, key=lambda p: (
                # Sort by pad number (handle alphanumeric like A1, B2)
                ''.join(c for c in p.pad_number if c.isalpha()),
                int(''.join(c for c in p.pad_number if c.isdigit()) or '0')
            ))
            for pad in pads_sorted:
                net_name = pad.net_name if pad.net_name else "(no net)"
                if args.pattern and not fnmatch(net_name, args.pattern):
                    continue
                print(f"  {pad.pad_number}: {net_name}")
        else:
            # Collect unique net names
            nets = set()
            for pad in footprint.pads:
                if pad.net_name and pad.net_id > 0:
                    if args.pattern and not fnmatch(pad.net_name, args.pattern):
                        continue
                    nets.add(pad.net_name)

            # Print sorted
            print(f"\nNets on {args.component} ({len(nets)} total):\n")
            for net in sorted(nets):
                print(f"  {net}")

    return 0


if __name__ == '__main__':
    from console_encoding import enable_utf8_console
    enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
    exit(main())
