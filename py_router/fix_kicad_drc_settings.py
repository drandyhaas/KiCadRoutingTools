#!/usr/bin/env python3
"""Make a routed board's KiCad DRC settings consistent with the clearances and
sizes it was actually routed to, so an interactive DRC in KiCad shows only the
relevant (routing) errors instead of stock-default noise (issue #160).

KiCad stores DRC *design rules* and *violation severities* in the PROJECT file
(``.kicad_pro``), NOT in the board (``.kicad_pcb``). A freshly written board gets
a project with KiCad's stock defaults, which produce noise in two ways:

  1. Constraint floors stricter than the board was routed to -- e.g. the stock
     ``min_clearance`` 0.2 mm, ``min_via_diameter`` 0.45 mm, ``min_track_width``
     0.2 mm or ``min_hole_clearance`` 0.25 mm -- fire on every track/via/drill
     the router placed below them (hundreds of spurious markers). They are not
     real problems at the manufacturing floor the board was routed to;
     ``check_drc.py`` never reports them.
  2. Placement / fabrication categories (courtyard overlaps, solder-mask bridges,
     footprint-library annular/mismatch) fire even though the router neither
     creates nor fixes them.

This script rewrites the sibling ``.kicad_pro`` so KiCad's enforced
**Board Setup -> Constraints / Net Classes** match the per-object minima the
board actually uses:

  * copper **clearance** -- TWO quantities, not one (#900). ``min_clearance``
    is an absolute floor and is capped at the smallest copper-pad clearance
    override (#530), because KiCad floors an override there; the routing
    steps' entry point ``fix_project_for_output`` (and its live-board twin)
    caps it at the smallest ``.kicad_dru`` layer rule as well (#498), which
    this module's own ``main()`` has never done. The Default net-class
    **clearance** carries the requirement the board was routed to and is
    capped by neither.
  * **hole-to-hole** clearance (``min_hole_to_hole``)
  * **hole/copper** clearance (``min_hole_clearance``)
  * **copper-to-edge** clearance (``min_copper_edge_clearance``)
  * **min track width / via diameter / via drill / annular ring** -- lowered to
    the smallest such object actually placed on the board
  * Default net-class **clearance** only. The class ``track_width`` /
    ``via_diameter`` / ``via_drill`` / ``diff_pair_*`` are DRAW DEFAULTS (KiCad
    loads them with SetOpt, never SetMin) and are NEVER written: lowering them
    to the board's smallest object was the #842 ratchet -- one 0.127 mm neck
    made the Default class 0.127 and every later run routed at it.
  * non-routing severities (courtyard shapes, solder-mask, footprint/library
    -> ignore; ``starved_thermal`` and ``courtyards_overlap`` -> warning)
    **only with ``--relax-severities``** (#856). A routing step never changes
    what the project counts as a violation unless asked; when it does, the
    previous values are kept under ``kicad_routing_tools.saved_severities``.

**Only loosen, never tighten.** Every constraint is set to ``min(current, target)``
-- it is only *lowered* toward the real fab floor, never raised. So this can
never introduce a NEW violation or silently strengthen a rule; it only stops
KiCad flagging copper the router legitimately placed. A constraint the user
already set looser than the routed floor is left as-is.

Targets come from the routing parameters when you pass them (``--clearance``,
``--hole-to-hole``, ``--edge-clearance``, ``--track-width``, ``--via-size``,
``--via-drill`` -- match what you gave ``route.py``); track/via/drill/annular also
fall back to the smallest object found on the board, and clearance falls back to
the project's Default net-class clearance.

With ``--enable-used-layers`` (OFF by default), it also (``enable_used_layers``)
adds any layer the board actually *uses* -- a footprint, graphic, pad, track, via
or zone draws on it -- but that is missing from the board's ``(layers)`` table,
back into that table in the ``.kicad_pcb``, so KiCad shows the layer as selectable
and stops flagging ``item_on_disabled_layer``. This is the one place the module
edits the ``.kicad_pcb`` (a format-preserving text insert), which is why it is
opt-in; everything else edits only the ``.kicad_pro``.

IMPORTANT: close the board in KiCad before running this. KiCad keeps the project
in memory and will overwrite an externally-edited ``.kicad_pro`` on save/close.

Usage:
    python3 fix_kicad_drc_settings.py board.kicad_pcb [options]
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'actor'}

import argparse
import json
import os
import re
import sys

# Severity categories treated as non-routing noise by default.
COURTYARD_CATS = ["courtyards_overlap", "malformed_courtyard",
                  "npth_inside_courtyard", "pth_inside_courtyard"]
MASK_CATS = ["solder_mask_bridge"]
# Footprint / library-geometry issues inherited from the source board's
# footprints (annular rings, pad/footprint library mismatches). The router does
# not create or fix these, so they are pure noise when reviewing a routed board
# -- on the stress boards they dominate the report (e.g. 199 annular_width + 149
# lib_footprint markers on orangecrab). Ignored by default; --keep-footprint
# restores them.
FOOTPRINT_CATS = ["annular_width", "lib_footprint_issues", "lib_footprint_mismatch"]
# Thermal-relief spoke shortfalls (a zone connects a pad with fewer spokes than
# the zone's min). It is a real-but-minor fab detail, not a routing short, so
# demote it from error to a WARNING (still visible, not blocking) rather than
# hiding it. --keep-thermal leaves it an error.
WARNING_CATS = ["starved_thermal"]

# Severity rank for "only loosen" comparisons (higher = stricter).
_SEV_RANK = {"error": 2, "warning": 1, "ignore": 0}

# The change strings the most recent fix_project_for_output wrote. Kept for a
# run summary to publish; nothing reads it today (no JSON_SUMMARY carries a
# project_writes key).
LAST_PROJECT_WRITES = []

# Net-class fields a writeback may lower, for EVERY class including Default.
# KiCad enforces exactly ONE net-class field as a DRC minimum: ``clearance``
# (drc_engine.cpp loads it with SetMin). ``track_width`` / ``via_diameter`` /
# ``via_drill`` / ``diff_pair_width`` / ``diff_pair_gap`` are loaded with SetOpt:
# they are the size KiCad DRAWS a new object at, the designer's intent, never a
# floor. Lowering them prevents no violation and destroys the spec. Measured
# twice: a QFN fanout laying 0.15mm escape stubs rewrote USB_FS_DIFF's
# track_width from 0.8 to 0.15 (HW-TB-PCB13); and #842 -- one terminal segment
# necked to the 0.127 fab floor lowered the DEFAULT class's track_width to
# 0.127, the next run read the Default class back as "the board's own width",
# and every track on the board came out at 0.127 from then on. Nothing ever
# raised it again. The Default class used to be exempt from this set on the
# theory that it "is the writeback's own floor record"; the floor record is
# ``rules.min_*``, and a draw default is not a floor.
_NETCLASS_WRITABLE_FIELDS = frozenset({"clearance"})
_NONDEFAULT_CLAMP_FIELDS = _NETCLASS_WRITABLE_FIELDS  # historical name, same set

# The KiCad rule names `compute_targets` may emit (#900). The project writeback
# loops over `targets` and used to write EVERY key straight into
# `board.design_settings.rules`, so the moment the dict carries anything that is
# not a rule -- `class_clearance` -- it lands in the shipped project as a rule
# KiCad has never heard of, invisibly: `list_nets.read_design_rules` reads a
# fixed field tuple, `FAB_FLOOR_KEYS` is another, and no test enumerates the
# rules key set. An ALLOW-list rather than a skip-list, because that is what
# makes the next non-rule key inert BY CONSTRUCTION instead of by someone
# remembering -- and it is the shape `apply_targets_to_board` already has by
# accident (it looks each key up in its rule->attribute map and skips a miss).
# The cost is the other direction: a newly added rule must be registered here
# or it is silently not written. `tests/test_900_class_clearance_not_capped.py`
# re-derives this set from `compute_targets` -- BY SOURCE, walking every
# `targets["..."] =` assignment, not only the keys one call happens to emit. An
# earlier draft derived it from a single CALL, which is blind to any key gated
# on a `minima` entry that call does not supply -- the shape
# `min_via_annular_width` already has. (That draft's call did pass that
# particular entry; the point is that the next key of the same shape would have
# depended on somebody remembering to.)
_RULE_KEYS = frozenset({
    "min_clearance", "min_hole_clearance", "min_hole_to_hole",
    "min_copper_edge_clearance", "min_track_width", "min_connection",
    "min_via_diameter", "min_through_hole_diameter", "min_via_drill",
    "min_via_annular_width",
})


def _class_clearance(targets):
    """The NET-CLASS clearance from a ``compute_targets`` result (mm or None).

    The routed value, never the capped rule floor. ``rules.min_clearance`` is
    capped at the smallest pad clearance override (#530) and at the smallest
    ``.kicad_dru`` layer rule (#498) because KiCad floors those there; the net
    classes carry the requirement the board was actually routed to and must not
    inherit either cap (#900).

    Falls back to ``min_clearance`` so a hand-built target dict still works --
    ``gui_utils.update_live_drc_floors`` and the GUI fanout tab both call
    ``clamp_nondefault_netclasses_on_board`` with ``{'min_clearance': ceiling}``,
    which is already the uncapped value they mean.
    """
    t = targets or {}
    return t.get("class_clearance", t.get("min_clearance"))

# A complete KiCad "Default" net class. KiCad only honours a net class it
# considers well-formed; a sparse {name, clearance, ...} stub is silently
# dropped and the board falls back to the stock 0.2 mm default (issue #160
# v9 demo). Used only when the project has NO Default class (a bare/stub
# project); a real KiCad-written project already has a complete one we just edit.
_DEFAULT_NETCLASS = {
    "bus_width": 12, "clearance": 0.2, "diff_pair_gap": 0.25,
    "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2, "line_style": 0,
    "microvia_diameter": 0.3, "microvia_drill": 0.2, "name": "Default",
    "pcb_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647,
    "schematic_color": "rgba(0, 0, 0, 0.000)", "track_width": 0.2,
    "via_diameter": 0.6, "via_drill": 0.3, "wire_width": 6,
}


def find_project(path: str) -> str:
    """Return the .kicad_pro path for a .kicad_pcb / .kicad_pro / base path."""
    base, ext = os.path.splitext(path)
    pro = path if ext == ".kicad_pro" else base + ".kicad_pro"
    return pro


# Canonical KiCad file-format layer ids (stable across KiCad 6-9). The id and
# type are needed to write a well-formed (layers) table entry. Copper layers are
# 'signal'; technical/user layers are 'user'. Returns None for a name we don't
# have a canonical id for (User.10+, exotic layers) so we skip rather than guess.
_TECH_LAYER_IDS = {
    'F.Mask': 1, 'B.Mask': 3, 'F.SilkS': 5, 'B.SilkS': 7,
    'F.Adhes': 9, 'B.Adhes': 11, 'F.Paste': 13, 'B.Paste': 15,
    'Dwgs.User': 17, 'Cmts.User': 19, 'Eco1.User': 21, 'Eco2.User': 23,
    'Edge.Cuts': 25, 'Margin': 27, 'B.CrtYd': 29, 'F.CrtYd': 31,
    'B.Fab': 33, 'F.Fab': 35,
}


def _canonical_layer(name: str):
    """(id, type) for a canonical KiCad layer name, or None if unknown."""
    if name == 'F.Cu':
        return (0, 'signal')
    if name == 'B.Cu':
        return (2, 'signal')
    m = re.fullmatch(r'In(\d+)\.Cu', name)        # In1.Cu=4, In2.Cu=6, ... In30.Cu=62
    if m:
        n = int(m.group(1))
        return (2 + 2 * n, 'signal') if 1 <= n <= 30 else None
    if name in _TECH_LAYER_IDS:
        return (_TECH_LAYER_IDS[name], 'user')
    m = re.fullmatch(r'User\.(\d+)', name)         # User.1=39, User.2=41, ... User.9=55
    if m:
        n = int(m.group(1))
        return (37 + 2 * n, 'user') if 1 <= n <= 9 else None
    return None


def enable_used_layers(pcb_path: str, verbose: bool = True):
    """Add any layer the board actually *uses* (a footprint, graphic, pad, track,
    via or zone draws on it) but that is missing from its ``(layers)`` table, so
    KiCad shows the layer as selectable and stops flagging ``item_on_disabled_layer``.

    This makes the layer **enabled / selectable** -- the layer SET in the
    ``.kicad_pcb`` ``(layers)`` table (KiCad's ``board.GetEnabledLayers()``). It
    does NOT touch layer **visibility** (which enabled layers are shown in the
    canvas) -- that is appearance state in the sibling ``.kicad_prl`` local-settings
    file, a separate concept this function deliberately leaves alone.

    CLI-ONLY AND OPT-IN. ``fix_project_for_output`` calls this only when its
    ``enable_layers`` flag is set -- the CLI ``--enable-used-layers`` option, which
    is OFF by default -- so by default it never runs and the ``.kicad_pcb`` is left
    untouched. It is opt-in because it mutates board structure (the layer table),
    not just DRC settings. The GUI plugin does NOT offer it at all: the GUI applies
    its DRC settings to a *live* pcbnew board the user is editing (via
    ``apply_targets_to_board``), and silently restructuring that board's
    enabled-layer set as a side effect of routing would be intrusive (the user
    manages layers in Board Setup). So the CLI/GUI asymmetry exists only when a CLI
    user explicitly opts in, and is intentional -- not a parity gap to close (see
    CLAUDE.md's parity rule).

    Format-preserving text edit of the ``.kicad_pcb`` (unlike the rest of this
    module, which only touches the ``.kicad_pro``). Returns the list of layer
    names added (empty if none / on any problem). Best-effort and conservative:
    a layer whose canonical id we don't know, or whose id would collide with an
    existing entry, is left alone."""
    try:
        with open(pcb_path, encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return []

    tbl = re.search(r'\(layers\b', text)
    if not tbl:
        return []
    start = tbl.start()
    depth, end = 0, start
    for i in range(start, len(text)):
        if text[i] == '(':
            depth += 1
        elif text[i] == ')':
            depth -= 1
            if depth == 0:
                end = i
                break
    block = text[start:end + 1]

    enabled = set(re.findall(r'\(\d+\s+"([^"]+)"', block))
    used_ids = {int(x) for x in re.findall(r'\((\d+)\s+"', block)}

    # Layer names referenced anywhere EXCEPT inside the (layers) table itself:
    # (layer "X") on graphics/text/zones, and (layers "A" "B" ...) on pads/vias.
    # Also drop the (setup (stackup ...)) block, whose (layer "dielectric N"...)
    # entries describe the physical stack, not the logical layer set.
    outside = text[:start] + text[end + 1:]
    stk = re.search(r'\(stackup\b', outside)
    if stk:
        d, k = 0, stk.start()
        for k in range(stk.start(), len(outside)):
            if outside[k] == '(':
                d += 1
            elif outside[k] == ')':
                d -= 1
                if d == 0:
                    break
        outside = outside[:stk.start()] + outside[k + 1:]
    from kicad_parser import layer_list_tokens  # quoted, or KiCad 6's bare pad lists
    refs = set(re.findall(r'\(layer\s+"([^"]+)"', outside))
    for grp in re.findall(r'\(layers\s+([^()]*)\)', outside):
        refs.update(layer_list_tokens(grp))

    indent_m = re.search(r'\n([ \t]+)\(\d+\s+"', block)
    indent = indent_m.group(1) if indent_m else '\t\t'

    additions = []
    for name in sorted(refs):
        if name in enabled or '*' in name or '&' in name or name == '':
            continue  # already enabled, or a wildcard/layer-set token (e.g. *.Cu, F&B.Cu)
        canon = _canonical_layer(name)
        if canon is None:
            if verbose:
                print(f"  layer enable: skipping {name!r} (no canonical id)")
            continue
        lid, ltype = canon
        if lid in used_ids:
            if verbose:
                print(f"  layer enable: skipping {name!r} (id {lid} already in use)")
            continue
        used_ids.add(lid)
        additions.append((lid, name, ltype))

    if not additions:
        return []

    additions.sort()
    new_entries = ''.join(f'\n{indent}({lid} "{name}" {ltype})'
                          for lid, name, ltype in additions)
    # Insert right after the last existing entry (before the block's closing paren).
    j = end - 1
    while j > start and text[j] in ' \t\r\n':
        j -= 1
    new_text = text[:j + 1] + new_entries + text[j + 1:]
    with open(pcb_path, 'w', encoding='utf-8') as f:
        f.write(new_text)
    if verbose:
        print(f"  layer enable: added {len(additions)} used layer(s) to {pcb_path}: "
              + ", ".join(n for _, n, _ in additions))
    return [n for _, n, _ in additions]


def project_copper_clearance(proj: dict):
    """The board's copper clearance: the Default netclass clearance, else
    rules.min_clearance. Returns None if neither is set (>0)."""
    for cls in proj.get("net_settings", {}).get("classes", []):
        if cls.get("name") == "Default" and cls.get("clearance"):
            return cls["clearance"]
    classes = proj.get("net_settings", {}).get("classes", [])
    if classes and classes[0].get("clearance"):
        return classes[0]["clearance"]
    mc = proj.get("board", {}).get("design_settings", {}).get("rules", {}).get("min_clearance")
    return mc if mc else None


def project_grading_clearance(proj: dict):
    """``(clearance, source)`` KiCad's DRC grades a Default-class pair at
    (#1210): the Default class, floored at Board Setup ``rules.min_clearance``
    -- KiCad's constraint resolution takes the larger, as
    ``design_rules.resolve`` does (measured against KiCad 10.0.0). A 0 is
    unset. ``source`` is 'Default net class' or 'board minimum clearance';
    ``(None, None)`` when neither is set.

    The GRADING reading only. :func:`project_copper_clearance` stays the class
    value: it is what an omitted ``--clearance`` routes at and what the
    writeback targets, and the writeback lowers ``min_clearance`` to the
    routed clearance, so a routed output grades the same either way.
    multichannel_mixer (class 0.2, min_clearance 0.3): kicad-cli reports 12
    clearance errors on the designer's own board; graded at the class alone,
    check_drc and board_score read clean."""
    mc = ((proj.get("board") or {}).get("design_settings") or {}) \
        .get("rules", {}).get("min_clearance")
    return grading_clearance(project_copper_clearance(proj), mc)


def grading_clearance(class_clearance, min_clearance):
    """``(clearance, source)``: the Default class clearance floored at Board
    Setup ``min_clearance``, KiCad's grading rule (#1210), from the two
    values however they were read. A missing or non-positive value is unset;
    ``(None, None)`` when both are. One rule for
    :func:`project_grading_clearance` and the placement graders'
    ``list_nets.board_grading_clearance`` (#1338)."""
    def _mm(v):
        return float(v) if isinstance(v, (int, float)) and v > 0 else None
    cls, mc = _mm(class_clearance), _mm(min_clearance)
    if mc is not None and (cls is None or mc > cls + 1e-9):
        return mc, "board minimum clearance"
    if cls is None:
        return None, None
    return cls, "Default net class"


def board_min_clearance_above(input_pcb, run_clearance):
    """``(declared, default_class)`` when the input board declares a Board
    Setup ``min_clearance`` above ``run_clearance`` (#1210), else None.

    KiCad grades the board at the declared value (see
    :func:`project_grading_clearance`); a run below it relaxes that rule, and
    the writeback then lowers ``min_clearance`` to what was routed. Read off
    the INPUT project, so in a chain only the step that relaxes it reports
    it. ``default_class`` (None when unset) lets the caller tell the board's
    own class from a lower clearance the run was asked for."""
    if not input_pcb or run_clearance is None:
        return None
    try:
        pro = find_project(input_pcb)
        with open(pro, encoding="utf-8") as f:
            proj = json.load(f)
    except (OSError, ValueError):
        return None
    mc = ((proj.get("board") or {}).get("design_settings") or {}) \
        .get("rules", {}).get("min_clearance")
    if not isinstance(mc, (int, float)) or mc <= 0 or run_clearance >= mc - 1e-9:
        return None
    from fab_tiers import project_default_class_clearance
    return float(mc), project_default_class_clearance(proj)


def project_edge_clearance(proj: dict):
    """The board's copper-to-Edge.Cuts constraint (Board Setup ->
    min_copper_edge_clearance). KiCad grades copper_edge_clearance from this
    value; the router and check_drc must honor it too (issue #338). Returns
    None if unset or 0."""
    ec = (proj.get("board", {}).get("design_settings", {}) or {}) \
        .get("rules", {}).get("min_copper_edge_clearance")
    return ec if ec else None


def read_project_edge_clearance(pcb_path: str):
    """min_copper_edge_clearance from a board's sibling .kicad_pro, or 0.0
    (missing project / unset value). Convenience wrapper used by the routing
    CLIs and check_drc so both route and grade at the same effective edge
    clearance (issue #338)."""
    try:
        pro = find_project(pcb_path)
        if os.path.isfile(pro):
            with open(pro) as f:
                return project_edge_clearance(json.load(f)) or 0.0
    except Exception:
        pass
    return 0.0


def web_min_connection(cfg: dict):
    """The min-copper-web width (mm) a board should be graded at (#406), from
    its .kicad_pro: an author-set `min_connection` is a real design rule and
    wins; otherwise the project's `min_track_width` (the post-route ledger
    floors it at the smallest object on the board, so the graded condition is
    "a copper web narrower than the narrowest intentional track"). None when
    neither is recorded -- connection_width is then NOT graded (KiCad's
    default min_connection is 0 = checker off), and the caller reports None
    rather than a fake clean 0. kicad_drc_compare stages KiCad's grade from
    this; check_weird and the repair passes read it through
    connection_width_floor (#1187)."""
    try:
        rules = cfg.get("board", {}).get("design_settings", {}).get("rules", {})
        for key in ("min_connection", "min_track_width"):
            try:
                v = float(rules.get(key))
            except (TypeError, ValueError):
                continue
            if v > 0:
                return v
    except AttributeError:
        pass
    return None


def connection_width_floor(pcb_data, track_width=None, *,
                           shipped=False) -> float:
    """THE narrow-pad-joint floor (#1187), for detection (check_weird) and
    repair (the terminal-web firming, the strict removal model) alike.

    The board's declared rules decide it, through :func:`web_min_connection`
    -- the same call kicad_drc_compare stages KiCad's connection_width grade
    from -- so copper on other nets never moves the grade of a finished board.
    It used to be the thinnest track now on the board: re-routing
    complex_hierarchy's one 0.2032 mm rescue at 0.4 mm flipped two unrelated
    pad joints to narrow-pad-joint while the project, and KiCad's grade, still
    said 0.2032. The declared rules are the sibling .kicad_pro, or on the GUI
    ``pcb_data.live_rules_provider`` (the live board's design settings, read
    now): mid-plan the file beside the live board is still the original, and
    the floors the plan lowered live in pcbnew's memory.

    ``shipped=True`` is a REPAIR pass's question, asked mid-run: the floor the
    board will be graded at once the writeback has run. The writeback lowers
    min_track_width (and an enabled min_connection) to the run's track width
    and to the thinnest track on the board, because KiCad grades connection
    width at ONE board-wide floor and that floor must admit every track. So
    the shipped floor is the declared one lowered to ``track_width`` and to
    the thinnest track. It only ever lowers: a thin track re-routed wider
    cannot raise it above the declared floor. A repair priced at the declared
    floor alone would skip this run's own thinner tracks (no floor-width web
    exists through them) and ship their sub-floor joints, which the lowered
    grade flags. Detection (``shipped=False``) reads the declared rules as
    they stand.

    With no project recording either rule (a project-less board, a synthetic
    PCBData) the floor is the thinnest track on the board, as #416 read it,
    then ``track_width``; 0.0 when there is nothing to read."""
    floor = None
    path = getattr(pcb_data, 'source_path', '') or ''
    provider = getattr(pcb_data, 'live_rules_provider', None)
    live = provider() if callable(provider) else None
    if live is not None:
        floor = web_min_connection(
            {'board': {'design_settings': {'rules': dict(live)}}})
    elif path:
        try:
            pro = find_project(path)
            if os.path.isfile(pro):
                with open(pro) as f:
                    floor = web_min_connection(json.load(f))
        except (OSError, ValueError):
            floor = None
    tw = float(track_width) if track_width and track_width > 0 else None
    widths = [s.width for s in getattr(pcb_data, 'segments', ())
              if not getattr(s, 'graphic', False) and s.width and s.width > 0]
    thinnest = min(widths) if widths else None
    if floor is not None and not shipped:
        return min(floor, tw) if tw else floor
    vals = [v for v in (floor, thinnest, tw) if v is not None]
    return min(vals) if vals else 0.0


def fab_edge_floor(pcb_path=None) -> float:
    """The fab-process copper-to-Edge.Cuts minimum (JLC routed-outline 0.20 mm)
    for the active fab tier -- the hard lower bound below which routed copper
    runs into the milled board edge. Independent of what the board declares: a
    board whose min_copper_edge_clearance is below this (or 0) is pinned UP to it
    for both routing and grading (#441). Copper-to-edge is a hard fab defect, so
    -- unlike the aspirational copper netclasses (#439), which clamp DOWN -- the
    edge floor is only ever raised, never relaxed below the fab minimum. Returns
    0.0 only if the active fab tier explicitly sets board_edge to 0 (a custom
    tier that genuinely allows edge copper -- via ``--fab-overrides board_edge=0``,
    the way to disable the pin for a board with intentional edge copper)."""
    try:
        from fab_tiers import fab_floor_min
        ncu = 2
        if pcb_path:
            try:
                from list_nets import _count_copper_layers
                with open(pcb_path, encoding='utf-8') as f:
                    ncu = _count_copper_layers(f.read()) or 2
            except Exception:
                pass
        return float(fab_floor_min(ncu).get('board_edge') or 0.0)
    except Exception:
        # fab_tiers is the single source of the copper-to-edge floor; if it cannot
        # be imported (broken install) degrade to no pin rather than duplicate the
        # magic value here.
        return 0.0


def effective_board_edge_clearance(pcb_path: str, cli_value: float,
                                   fab_floor: bool = True) -> float:
    """The copper-to-Edge.Cuts clearance a route/grade step must honor.

    An EXPLICIT ``--board-edge-clearance`` (cli_value > 0) OVERRIDES the board's
    own min_copper_edge_clearance -- a CLI value should be able to relax an
    aspirational board rule the way every other routing param does (a 0.5-declaring
    board routed at --board-edge-clearance 0.2, parity with the GUI's override +
    Obey-DRC-off path), floored ONLY at the fab copper-to-edge minimum. When
    OMITTED, the board's own rule is used (issue #338, KiCad enforces it). Either
    way it is pinned UP to the fab floor (#441) unless ``fab_floor`` is False, so a
    board declaring a sub-fab (or 0) edge rule is never routed/graded with copper
    against the milled edge (80/184 corpus boards declare < 0.20 mm)."""
    base = cli_value if (cli_value and cli_value > 0) else read_project_edge_clearance(pcb_path)
    return max(base, fab_edge_floor(pcb_path)) if fab_floor else base


# Rules that describe what the FAB can make, as opposed to how the board is
# graded. Lowering a clearance is a grading decision with a measured rationale
# (stock netclass clearances are aspirational, and keeping them manufactures
# phantom violations on correctly-routed copper). Lowering one of these is a
# different claim: that the board can be manufactured at the new number.
_FLOOR_EPS = 1e-9

FAB_FLOOR_KEYS = (
    ("min_track_width", "track width"),
    ("min_via_diameter", "via diameter"),
    ("min_via_annular_width", "via annular ring"),
    ("min_via_drill", "via drill"),
    ("min_through_hole_diameter", "hole diameter"),
    # Copper-to-hole. It belongs here and not with the aspirational netclass
    # clearances: it is a drill-REGISTRATION constraint, the same family as
    # annular ring above, and relaxing it is a claim about what the fab can
    # make. Measured on neo6502: a chain lowered it 0.25 -> 0.20 and the
    # disclosure said nothing, because this tuple did not list it, so
    # `relaxed: []` was a blind pass over three NPTH holes carrying copper at
    # 0.2126/0.2263/0.2263 mm.
    #
    # NOTE it is DECLARATION-only: scan_board_minima measures object sizes and
    # no pairwise geometry, so no measured counterpart exists for this key.
    # `_fab_floor_disclosure` (declared-vs-declared) covers it; consumers that
    # compare against a measured board minimum cannot, and must say so rather
    # than skip it in silence -- see check_complete's `unmeasured` list.
    ("min_hole_clearance", "copper-to-hole clearance"),
)

#: Subset of :data:`FAB_FLOOR_KEYS` that :func:`scan_board_minima` can actually
#: measure off the copper. Anything outside this set can be compared between two
#: DECLARATIONS but never against the board itself.
FAB_FLOOR_KEYS_MEASURABLE = frozenset({
    "min_track_width", "min_via_diameter", "min_via_annular_width",
    "min_via_drill", "min_through_hole_diameter",
})

#: Rules this repo writes into ``design_settings.rules`` that are NOT KiCad
#: rules. KiCad's own project save keeps an unknown key only when it was in the
#: file when the board was loaded (probed, KiCad 10.0.0), so one written after
#: load -- by a GUI step, or by the plan executor's writeback -- is gone after
#: the user's save. Each is mirrored into the records (``kicad_routing_tools.
#: repo_rules``, the copy that survives, tool_records) and read through
#: :func:`_effective_rules`.
REPO_RULE_KEYS = ("min_via_drill",)
REPO_RULES_KEY = "repo_rules"


def _effective_rules(proj: dict) -> dict:
    """``design_settings.rules`` with each repo-private rule
    (:data:`REPO_RULE_KEYS`) taken from the records when the rules lack it --
    the lower of the two when both carry it, since every writer of these
    floors only lowers them."""
    rules = dict((((proj or {}).get("board") or {}).get("design_settings")
                  or {}).get("rules") or {})
    rec = (((proj or {}).get("kicad_routing_tools") or {})
           .get(REPO_RULES_KEY) or {})
    for k in REPO_RULE_KEYS:
        v = rec.get(k)
        if isinstance(v, (int, float)) and v > 0:
            cur = rules.get(k)
            rules[k] = (min(float(cur), float(v))
                        if isinstance(cur, (int, float)) else float(v))
    return rules


def declared_project_rule(pcb_path: str, key: str):
    """The rule ``key`` the board at ``pcb_path`` declares (mm), read the way
    the writeback reads it: the project's rules, with the records' copy of a
    repo-private rule (:func:`_effective_rules`). None when absent."""
    try:
        import tool_records
        v = _effective_rules(tool_records.load_project(find_project(pcb_path))
                             ).get(key)
    except Exception:                                           # noqa: BLE001
        return None
    return float(v) if isinstance(v, (int, float)) else None


def seed_fab_floor_origin(proj: dict, rules_before: dict):
    """``(origin, seeded_now)`` for ``proj``'s ``fab_floor_origin`` record.

    The floors the board declared BEFORE this chain touched anything, seeded
    from ``rules_before`` when the project carries no origin yet and returned
    unchanged when it does. The caller stores ``origin`` under
    ``kicad_routing_tools.fab_floor_origin`` when ``seeded_now``.

    EVERY writer that can lower a FAB_FLOOR_KEYS rule must call this BEFORE it
    lowers anything -- not just :func:`fix_project_for_output`. It used to be
    inline there and nowhere else, so :func:`apply_routed_floors` (the #650
    mid-run copper writer, which runs FIRST) lowered ``min_hole_clearance``
    with no origin recorded, and the writeback then seeded the origin from the
    ALREADY-LOWERED value and compared it against itself. Measured on a 6-layer
    board declaring 0.25: the mid-run pass took it straight to 0.127, the
    origin recorded 0.127, and ``FAB FLOOR RELAXED`` said NOTHING about a real
    0.25 -> 0.127 relaxation. On a second board the same path understated it as
    "0.2 -> 0.127" because the mid-run pass had stopped at 0.2. A silent
    disclosure is exactly the failure this record exists to prevent (see
    :func:`_fab_floor_disclosure`'s run-14 note), so the seeding lives in one
    function that both writers call.
    """
    origin = dict((proj.get("kicad_routing_tools") or {})
                  .get("fab_floor_origin") or {})
    if origin:
        return origin, False
    origin = {k: float(v) for k, v in (rules_before or {}).items()
              if k in {key for key, _ in FAB_FLOOR_KEYS}
              and isinstance(v, (int, float))}
    return origin, bool(origin)


# --- The LIVE-board twin (the GUI) -------------------------------------------
#
# The GUI lowers the same floors on the live pcbnew board -- apply_targets_to_
# board, then gui_utils.update_live_drc_floors, every step -- and recorded no
# origin, so a manual GUI run that relaxed a fab floor said nothing, and a later
# CLI step baselined on the already-lowered value (the ad7f24de defect, on the
# other front). These are the file writers' rules applied to a live board.

#: FAB_FLOOR_KEYS rule -> BOARD_DESIGN_SETTINGS attribute. `min_via_drill` has
#: no attribute of its own: KiCad grades via drills against m_MinThroughDrill,
#: which is `min_through_hole_diameter`.
FAB_FLOOR_LIVE_ATTR = {
    "min_track_width": "m_TrackMinWidth",
    "min_via_diameter": "m_ViasMinSize",
    "min_via_annular_width": "m_ViasMinAnnularWidth",
    "min_through_hole_diameter": "m_MinThroughDrill",
    "min_hole_clearance": "m_HoleClearance",
}

#: Every design-settings rule a live reader takes from pcbnew -> the
#: BOARD_DESIGN_SETTINGS attribute it lives in. The live twin of the
#: ``design_settings.rules`` keys the file readers take from the .kicad_pro.
LIVE_RULE_ATTR = {
    "min_clearance": "m_MinClearance",
    "min_track_width": "m_TrackMinWidth",
    "min_connection": "m_MinConn",
    "min_via_diameter": "m_ViasMinSize",
    "min_through_hole_diameter": "m_MinThroughDrill",
    "min_via_annular_width": "m_ViasMinAnnularWidth",
    "min_hole_clearance": "m_HoleClearance",
    "min_hole_to_hole": "m_HoleToHoleMin",
    "min_copper_edge_clearance": "m_CopperEdgeClearance",
}

#: rule key -> the pcbnew constant BOARD_DESIGN_SETTINGS initialises it from
#: (mm). The annular ring has no constant of its own; KiCad derives it from
#: the via and drill ones, (0.5 - 0.3) / 2 = 0.1 on KiCad 10.
_PCBNEW_DEFAULT_CONSTANT = {
    "min_clearance": "DEFAULT_MINCLEARANCE",
    "min_track_width": "DEFAULT_TRACKMINWIDTH",
    "min_connection": "DEFAULT_MINCONNECTION",
    "min_via_diameter": "DEFAULT_VIASMINSIZE",
    "min_through_hole_diameter": "DEFAULT_MINTHROUGHDRILL",
    "min_hole_clearance": "DEFAULT_HOLECLEARANCE",
    "min_hole_to_hole": "DEFAULT_HOLETOHOLEMIN",
    "min_copper_edge_clearance": "DEFAULT_COPPEREDGECLEARANCE",
}
_PCBNEW_FACTORY = None


def pcbnew_rule_defaults() -> dict:
    """pcbnew's factory value for each :data:`LIVE_RULE_ATTR` rule, mm: what a
    board whose project does not declare the rule -- every project-less board
    -- holds in memory. ``{}`` without pcbnew.

    Read from the constants BOARD_DESIGN_SETTINGS initialises from, NEVER off a
    throwaway ``pcbnew.BOARD()``: constructing and dropping a board in the GUI
    process made the planes tab's next ZONE_FILLER.Fill segfault
    (test_gui_engine_parity, 3 runs of 3)."""
    global _PCBNEW_FACTORY
    if _PCBNEW_FACTORY is None:
        out = {}
        try:
            import pcbnew
            for key, name in _PCBNEW_DEFAULT_CONSTANT.items():
                v = getattr(pcbnew, name, None)
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    out[key] = float(v)
            if "min_via_diameter" in out and "min_through_hole_diameter" in out:
                out["min_via_annular_width"] = round(
                    (out["min_via_diameter"] - out["min_through_hole_diameter"])
                    / 2.0, 9)
        except Exception:                                       # noqa: BLE001
            out = {}
        _PCBNEW_FACTORY = out
    return dict(_PCBNEW_FACTORY)


def file_stamp(path: str):
    """``(mtime_ns, size, inode)`` of ``path``, or None when it does not exist:
    what identifies a file's CONTENT without reading it. The inode is in it
    because every writer here replaces a project atomically (tmp +
    ``os.replace``), which gives it a new inode even inside one mtime tick."""
    try:
        st = os.stat(path)
    except (OSError, ValueError, TypeError):
        return None
    return (st.st_mtime_ns, st.st_size, st.st_ino)


_PROJECT_DOC_CACHE: dict = {}


def _project_doc(pcb_path: str) -> dict:
    """``pcb_path``'s sibling .kicad_pro as parsed JSON (``{}`` when there is
    none), re-read whenever the file changes. The live readers below ask it on
    every obstacle-map build, so it is cached by stamp. Callers must not
    mutate what it returns."""
    pro = find_project(pcb_path) if pcb_path else ""
    if not pro:
        return {}
    stamp = file_stamp(pro)
    hit = _PROJECT_DOC_CACHE.get(pro)
    if hit is not None and hit[0] == stamp:
        return hit[1]
    doc = {}
    if stamp is not None:
        try:
            with open(pro, "r", encoding="utf-8") as f:
                doc = json.load(f)
        except Exception:                                       # noqa: BLE001
            doc = {}
    _PROJECT_DOC_CACHE[pro] = (stamp, doc if isinstance(doc, dict) else {})
    return _PROJECT_DOC_CACHE[pro][1]


def project_rules(pcb_path: str) -> dict:
    """``design_settings.rules`` of ``pcb_path``'s sibling .kicad_pro (``{}``
    when there is none), re-read whenever the file changes."""
    return dict((((_project_doc(pcb_path).get("board") or {})
                  .get("design_settings") or {}).get("rules") or {}))


#: The session-declaration key for the Default net class's clearance, which a
#: project-less board holds at pcbnew's stock class: the file writer CREATES
#: the class at the routed clearance when the project has none.
DEFAULT_CLASS_CLEARANCE_KEY = "net_settings.Default.clearance"


def live_board_key(board) -> str:
    """The key a live board's session records are kept under: its file, or
    the object itself when it has never been saved (#1238)."""
    try:
        path = board.GetFileName() or ""
    except Exception:                                           # noqa: BLE001
        path = ""
    return os.path.normcase(os.path.abspath(path)) if path else f"id:{id(board)}"


#: board key -> the LIVE_RULE_ATTR rules a live writer of this session set or
#: targeted (apply_targets_to_board, gui_utils.update_live_drc_floors). The CLI
#: writeback WRITES every rule it targets into the project, declared before or
#: not, so on a project-less board those rules are declared from step 2 on --
#: and a GUI writer that targets one at exactly pcbnew's factory value leaves
#: nothing in memory to tell that apart from "never configured". This is it.
_LIVE_DECLARED: dict = {}


def note_live_declared(board, keys) -> None:
    """Record that a live writer of this session set or targeted ``keys``."""
    try:
        _LIVE_DECLARED.setdefault(live_board_key(board), set()).update(
            k for k in keys
            if k in LIVE_RULE_ATTR or k == DEFAULT_CLASS_CLEARANCE_KEY
            or str(k).startswith("severity:"))
    except Exception:                                           # noqa: BLE001
        pass


def live_declares_default_class(board) -> bool:
    """Does the live board DECLARE its Default net class -- its project has
    one, or a live writer of this session set its clearance? A project-less
    board holds pcbnew's stock class, which no file declares; the file writer
    creates the class at the routed clearance there rather than lowering a
    stock one."""
    try:
        path = board.GetFileName() or ""
    except Exception:                                           # noqa: BLE001
        path = ""
    if DEFAULT_CLASS_CLEARANCE_KEY in _LIVE_DECLARED.get(live_board_key(board), ()):
        return True
    classes = ((_project_doc(path).get("net_settings") or {}).get("classes")
               or []) if path else []
    return any(isinstance(c, dict) and c.get("name") == "Default" for c in classes)


def live_declared_rules(board, keys=None):
    """``{rule: mm}``: the design-settings rules the LIVE board DECLARES, or
    None when it has no design settings to read.

    pcbnew holds every rule in memory whether or not anything declared it: a
    project that does not declare a rule, and every project-less board, loads
    it at the factory value (:func:`pcbnew_rule_defaults`). The file readers
    see no rule there. So a live value counts as declared when the board's
    project declares the key, or a live writer of this session set or targeted
    it (:func:`note_live_declared`), or it differs from the factory value --
    which someone set (Board Setup, or a GUI step's writeback). Everything else
    is pcbnew's stand-in for "not configured" and is left out, so a board
    pcbnew just loaded declares exactly what its file declares, and both
    fronts read the same floors off it. A rule whose factory value cannot be
    read is declared only by the project or the session.
    """
    try:
        bds = board.GetDesignSettings()
    except Exception:                                           # noqa: BLE001
        return None
    if bds is None:
        return None
    try:
        path = board.GetFileName() or ""
    except Exception:                                           # noqa: BLE001
        path = ""
    in_project = project_rules(path) if path else {}
    session = _LIVE_DECLARED.get(live_board_key(board), set())
    factory = pcbnew_rule_defaults()
    out = {}
    for key in (keys if keys is not None else LIVE_RULE_ATTR):
        attr = LIVE_RULE_ATTR.get(key)
        v = getattr(bds, attr, None) if attr else None
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            continue
        mm = v / 1e6                              # nm -> mm; divide (#493)
        if key in in_project or key in session \
                or (key in factory and abs(mm - factory[key]) > 1e-9):
            out[key] = mm
    return out


def live_fab_floor_rules(bds) -> dict:
    """The FAB_FLOOR_KEYS a live board's design settings HOLD now, mm, every
    one of them -- including pcbnew's factory stand-ins. What the board
    DECLARES is :func:`live_declared_fab_floor_rules`."""
    out = {}
    for key, attr in FAB_FLOOR_LIVE_ATTR.items():
        v = getattr(bds, attr, None)
        if isinstance(v, (int, float)) and v > 0:
            out[key] = v / 1e6                    # nm -> mm; divide (#493)
    return out


#: board key -> {rule: mm} for the FAB_FLOOR_KEYS a live writer of this session
#: set that pcbnew has no attribute for: `min_via_drill`, the repo's own via-
#: drill floor, which the CLI's writeback writes into the project beside the
#: KiCad rules (compute_targets) and its disclosure reports. Only-lower, like
#: the file writer.
_LIVE_SESSION_RULES: dict = {}


def note_live_session_rule(board, key: str, mm) -> None:
    """Lower the session-held ``key`` floor to ``mm`` (see
    :data:`_LIVE_SESSION_RULES`); a None ``mm`` is ignored. A repo-private
    rule (:data:`REPO_RULE_KEYS`) also goes into the board's records, the
    copy that survives KiCad's save (:func:`_effective_rules`)."""
    if mm is None:
        return
    try:
        v = round(float(mm), 6)
        rules = _LIVE_SESSION_RULES.setdefault(live_board_key(board), {})
        cur = rules.get(key)
        rules[key] = v if cur is None else min(cur, v)
        if key in REPO_RULE_KEYS:
            def _record(ns):
                rec = ns.setdefault(REPO_RULES_KEY, {})
                old = rec.get(key)
                if isinstance(old, (int, float)) and old <= v + _FLOOR_EPS:
                    return False
                rec[key] = v
                return True
            import tool_records
            tool_records.update_live_namespace(board, _record)
    except Exception:                                           # noqa: BLE001
        pass


def live_declared_fab_floor_rules(board) -> dict:
    """The FAB_FLOOR_KEYS the live board DECLARES (:func:`live_declared_rules`,
    plus the floors pcbnew has no attribute for: held for the session, or
    carried in the board's records from an earlier one), positive values
    only, mm -- what the origin records and the disclosure compares, so a
    project-less board records and says what the CLI's writeback does on the
    same file: nothing at step 1."""
    out = {k: v for k, v in (live_declared_rules(board, FAB_FLOOR_LIVE_ATTR)
                             or {}).items() if v > 0}
    extra = {}
    try:
        import tool_records
        extra = {k: v for k, v in (tool_records.live_namespace(board)
                                   .get(REPO_RULES_KEY) or {}).items()
                 if k in REPO_RULE_KEYS and isinstance(v, (int, float))}
    except Exception:                                           # noqa: BLE001
        extra = {}
    for k, v in (_LIVE_SESSION_RULES.get(live_board_key(board)) or {}).items():
        if v and v > 0:
            extra[k] = min(v, extra[k]) if k in extra else v
    for k, v in extra.items():
        if v and v > 0:
            out[k] = v
    return out


#: Board keys whose GUI step is OPEN: its first live writer
#: (apply_targets_to_board) has run and its last (gui_utils.
#: update_live_drc_floors) has not. A CLI step has ONE writer, which seeds the
#: origin from the project as the step found it; a GUI step has two, and the
#: second must not seed from what the first just lowered.
_LIVE_STEP_OPEN: set = set()

#: board key -> the floors the board declared when its current GUI step
#: STARTED -- the CLI writeback's ``rules_before``, which its disclosure falls
#: back to for a floor the origin does not carry.
_LIVE_STEP_BEFORE: dict = {}


def live_step_rules_before(board) -> dict:
    """The declared floors at the start of the board's current GUI step."""
    return dict(_LIVE_STEP_BEFORE.get(live_board_key(board)) or {})


#: board key -> the Default class clearance the board DECLARED when its current
#: GUI step started (None: a class it does not declare) -- the file
#: writeback's ``_class_before`` (#1160).
_LIVE_STEP_CLASS_BEFORE: dict = {}


def _live_default_class_clearance(board):
    """The live Default class clearance (mm) when the board declares the
    class, else None -- what ``project_default_class_clearance`` reads off a
    project."""
    try:
        if not live_declares_default_class(board):
            return None
        nc = board.GetDesignSettings().m_NetSettings.GetDefaultNetclass()
        v = nc.GetClearance() / 1e6
        return v if v > 0 else None
    except Exception:                                           # noqa: BLE001
        return None


def record_live_class_clearance_descent(board) -> list:
    """At a GUI step's end: :func:`record_class_clearance_descent` for the live
    board -- the step's automatic descents lowered its Default class below what
    it asked for -- recorded in the board's records (tool_records) as the file
    writeback records it in the project. Returns the disclosure lines, as the
    writeback prints them."""
    key = live_board_key(board)
    if key not in _LIVE_STEP_CLASS_BEFORE:
        return []
    before = _LIVE_STEP_CLASS_BEFORE.pop(key)
    try:
        import tool_records
        ns = tool_records.live_namespace(board)
        after = _live_default_class_clearance(board)
        view = {"kicad_routing_tools": ns,
                "net_settings": {"classes": ([{"name": "Default",
                                               "clearance": after}]
                                             if after else [])}}
        rec, here = record_class_clearance_descent(view, before)
        if here:
            def _record(n):
                n[CLASS_CLEARANCE_RELAXED_KEY] = dict(rec)
                return True
            tool_records.update_live_namespace(board, _record)
        return class_clearance_disclosure(rec, here, after)
    except Exception:                                           # noqa: BLE001
        return []


def open_live_step(board) -> None:
    """A GUI step's first live writer has seeded the origin (see
    :data:`_LIVE_STEP_OPEN`)."""
    _LIVE_STEP_OPEN.add(live_board_key(board))


def close_live_step(board) -> None:
    """A GUI step's last live writer is done: the next seed belongs to the
    next step."""
    _LIVE_STEP_OPEN.discard(live_board_key(board))


def seed_live_fab_floor_origin(board) -> dict:
    """:func:`seed_fab_floor_origin` for a live pcbnew board. Returns the origin.

    Call BEFORE lowering anything, from every live writer, exactly as the file
    writers do. The first call of a session records the floors the board
    DECLARES (:func:`live_declared_fab_floor_rules`) under
    ``kicad_routing_tools.fab_floor_origin`` in the board's records
    (tool_records: its .kicad_pro when it has one, and the board itself, the
    copy KiCad's own save keeps) when none is recorded yet. An origin already
    recorded wins, so a GUI step after a CLI chain (or an earlier session)
    keeps the chain's original. A board that declares none (a
    project-less one at pcbnew's factory values) records none, as the CLI's
    first writeback records none on it; the next step then seeds from what the
    first one wrote, as the CLI's second does -- the next STEP, not the second
    writer of the same one (:data:`_LIVE_STEP_OPEN`). Best-effort: never
    raises.
    """
    try:
        before = live_declared_fab_floor_rules(board)
        import tool_records
        ns = tool_records.live_namespace(board)
    except Exception:                                           # noqa: BLE001
        return {}
    key = live_board_key(board)
    if key not in _LIVE_STEP_OPEN:
        _LIVE_STEP_BEFORE[key] = dict(before)    # this step's rules_before
        _LIVE_STEP_CLASS_BEFORE[key] = _live_default_class_clearance(board)
    # The records are the board's (tool_records): its project's copy and the
    # copy on the board itself, which KiCad's save keeps when it drops what
    # was written into the project after load.
    origin, seeded = seed_fab_floor_origin({"kicad_routing_tools": ns}, before)
    if origin and not seeded:
        return origin             # already recorded (a CLI chain, a session)
    if key in _LIVE_STEP_OPEN:
        # This step's first writer found nothing declared. What the board
        # declares now is what that writer lowered, which the CLI's
        # one-writer step records only at the NEXT step.
        return {}
    if seeded:
        def _record(n):
            n["fab_floor_origin"] = dict(origin)
            return True
        try:
            tool_records.update_live_namespace(board, _record)
        except Exception:                                       # noqa: BLE001
            pass
    return origin


def live_fab_floor_disclosure(origin: dict, rules_after: dict,
                              objects: dict = None,
                              rules_before: dict = None) -> list:
    """The FAB FLOOR RELAXED lines for a live board: :func:`_fab_floor_disclosure`
    against the ORIGIN, with ``objects`` (``{rule key: [object sizes mm]}``
    measured off the live board) as the census -- the live copper is not on
    disk to count. ``rules_before`` is the step's input
    (:func:`live_step_rules_before`), which the file writer falls back to for a
    floor its origin does not carry; omitted, the origin stands in for it."""
    return _fab_floor_disclosure(
        "", origin if rules_before is None else rules_before,
        {"board": {"design_settings": {"rules": rules_after}}},
        origin, objects=objects)


def declared_fab_floor(pcb_path: str, key: str):
    """The floor the board declared for ``key`` BEFORE this chain touched it, mm.

    Reads ``kicad_routing_tools.fab_floor_origin`` from the sibling
    ``.kicad_pro`` -- seeded on the FIRST writeback and carried down the chain
    with the project, the same record :func:`_fab_floor_disclosure` compares
    against. Returns ``None`` when there is no project, no origin (nothing has
    written back yet, so ``design_settings.rules`` IS the original), or the key
    is absent.

    Why an ENGINE needs this and not just the disclosure: the writeback clamps
    ``rules`` DOWN to what the step routed, so by step 2 the rules value is the
    routed clearance and the author's declaration is gone from the place every
    reader looks. Measured on tigard, a pour + route chain from a project
    declaring ``min_hole_clearance`` 0.25::

        step   rules.min_hole_clearance   fab_floor_origin.min_hole_clearance
        in     0.25                       (none yet)
        pour   0.15                       0.25
        out    0.1375                     0.25

    A consumer reading ``rules`` alone stops honouring 0.25 after step 1 --
    silently, because 0.15 is a perfectly plausible value. The origin is the
    durable record; prefer the LARGER of the two (see
    :func:`obstacle_map.resolve_hole_clearance`).

    The record is read through tool_records: the newer of the project's copy
    and the board file's, which is where a GUI session's origin survives the
    user's KiCad save.
    """
    try:
        import tool_records
        v = (tool_records.effective_namespace(pcb_path)
             .get("fab_floor_origin") or {}).get(key)
        return float(v) if isinstance(v, (int, float)) and v > 0 else None
    except Exception:                                            # noqa: BLE001
        return None


def _fab_floor_disclosure(output_pcb: str, rules_before: dict, proj: dict,
                          origin: dict = None, objects: dict = None):
    """Say out loud when the writeback relaxed a MANUFACTURING floor.

    ``origin`` is the floor the board declared BEFORE this chain touched it
    (``kicad_routing_tools.fab_floor_origin``, seeded on the first writeback
    and carried down with the project). Without it this function compares
    against its immediate input, which goes silent the moment a chain has more
    than one step: run 14's R1 announced ``via diameter 0.5 -> 0.25`` once, and
    then R4 added 7 more sub-0.5 vias and R5 added 10 with NO banner at all,
    because by then 0.25 was the input and nothing had "moved". The board that
    shipped had 10 vias under its own declared 0.5 and every instrument called
    it clean. So the comparison is against the ORIGIN, and a step that inherits
    an already-relaxed floor still says so.

    Run-7 finding: two routed boards shipped with their project's
    min_track_width rewritten 0.2 -> 0.0889 and min_via_diameter 0.5 -> 0.25,
    and on one of them 5629 of 5933 segments and 371 of 712 vias sat below the
    board's ORIGINAL declared floors -- while check_drc, board_score and
    KiCad's own DRC all read clean, because every one of them grades against
    the rewritten project.

    The relaxation is not simply a bug: both boards carried KiCad's STOCK
    defaults (0.2 / 0.5 / 0.1), which is exactly the aspirational case the
    clamp exists for, and the routed copper sat at the repo's fab track floor
    rather than below it. The defect is that the tool cannot tell a stock
    default from a deliberate fab constraint -- and when it is the latter, a
    silent rewrite ships a board the fab cannot make, with every instrument
    green. A third board whose author had set 0.127 was never ratcheted,
    because its routing stayed above it; that is luck, not protection.

    So: keep the relaxation (removing it re-manufactures phantom DRC), and
    make it impossible to miss. Counts, not adjectives.
    """
    # A repo-private rule KiCad's save dropped is read from the records.
    rules_after = _effective_rules(proj)
    origin = origin or {}
    relaxed = []
    for key, label in FAB_FLOOR_KEYS:
        was, now = rules_before.get(key), rules_after.get(key)
        # Baseline is the board's ORIGINAL declaration where we know it, and
        # the immediate input otherwise (first step of a chain, or a project
        # that predates the origin key). Once an origin is recorded, a key it
        # lacks is one the board never declared: the input's value is then
        # one an earlier step wrote, and reporting it as "the ORIGINAL" is
        # how a bare pour's invented min_via_drill 0.6 became route.py's
        # "via drill: 0.6 -> 0.5 ... ORIGINAL 0.6mm" (#1319).
        base = origin.get(key) if origin else was
        if not isinstance(now, (int, float)) or not isinstance(base, (int, float)):
            continue
        if now < base - _FLOOR_EPS:
            # `moved_here` separates "this step lowered it" from "this step
            # inherited it and is still under the original".
            moved_here = (isinstance(was, (int, float))
                          and now < was - _FLOOR_EPS)
            relaxed.append((key, label, float(base), float(now), moved_here))
    if not relaxed:
        return []

    census = {}
    try:
        if objects is None:
            # `objects` is {rule key: [object sizes, mm]}. The live-board twin
            # passes it, measured off pcbnew's own objects; the file writers
            # measure the board they just wrote.
            from kicad_parser import parse_kicad_pcb
            pcb = parse_kicad_pcb(output_pcb)
            objects = {
                # Graphic copper is a shape, not a track (#908, #337) -- the
                # same exclusion `scan_board_minima` makes below. Counting a
                # filled fp_poly's stroke here told the reader that N tracks
                # sit under the original floor when none of them is a track.
                "min_track_width": [s.width for s in pcb.segments
                                    if s.width and not getattr(s, 'graphic', False)],
                "min_via_diameter": [v.size for v in pcb.vias if v.size],
                "min_via_drill": [v.drill for v in pcb.vias if v.drill],
                "min_via_annular_width": [(v.size - v.drill) / 2.0 for v in pcb.vias
                                          if v.size and v.drill and v.size > v.drill],
            }
        for key, _label, was, _now, _mv in relaxed:
            objs = list(objects.get(key) or [])
            if objs:
                census[key] = (sum(1 for o in objs if o < was - _FLOOR_EPS), len(objs))
    except Exception:                                   # disclosure is best-effort
        pass

    lines = ["  FAB FLOOR RELAXED -- the output project declares a smaller "
             "minimum than the board originally did:"]
    for key, label, was, now, moved_here in relaxed:
        under, total = census.get(key, (None, None))
        tail = (f"; {under} of {total} object(s) on this board are below the "
                f"ORIGINAL {was:g}mm" if under is not None else "")
        if key == "min_hole_clearance":
            # #1217: no router setting holds copper off a via drill or a PTH
            # barrel at this floor (the router applies it at NPTH walls), so
            # "re-route at that floor" cannot restore it there.
            tail += ("; the router holds it at NPTH walls only -- plated holes "
                     "and vias sit at copper clearance, which re-routing does "
                     "not change")
        if moved_here:
            lines.append(f"    {label}: {was:g} -> {now:g} mm{tail}")
        else:
            # Inherited from an earlier step in the chain. Saying nothing here
            # is what let run 14 add 17 sub-floor vias in silence.
            lines.append(f"    {label}: {now:g} mm, unchanged by this step but "
                         f"still below the board's ORIGINAL {was:g}mm{tail}")
    lines.append("    Every checker (check_drc, board_score, KiCad's own DRC) "
                 "grades against the NEW value, so this copper will read clean. "
                 "Confirm your fab supports it; if the original number was a "
                 "real process limit, re-route at that floor rather than "
                 "shipping this project.")
    return lines


def scan_board_minima(pcb_path: str):
    """Smallest track width / via diameter / via drill / via annular ring / hole
    diameter actually present on the board. These are floors KiCad's min-size
    rules must sit at or below, or it flags the board's own copper. Returns a
    dict of floats (missing keys absent). Best-effort -- returns {} if the board
    can't be parsed."""
    if not os.path.isfile(pcb_path):
        return {}
    try:
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(pcb_path)
    except Exception as e:  # pragma: no cover - parser is robust, but stay safe
        print(f"warning: could not scan board minima ({e})", file=sys.stderr)
        return {}

    out = {}
    # Footprint / board GRAPHIC copper (#908, #337) is a shape, not a track:
    # a filled fp_poly's stroke width is an outline width and KiCad's
    # min_track_width rule never grades it. Counting it here wrote
    # `rules.min_track_width 0.15 -> 0.1` on every step of a chain whose
    # only 0.1 mm "track" was a SOT-89 tab outline, and made check_complete
    # read that board as UNSOUND (run 26, esp_prog).
    widths = [s.width for s in pcb.segments
              if s.width and s.width > 0 and not getattr(s, 'graphic', False)]
    if widths:
        out["min_track_width"] = min(widths)
    via_drills = [v.drill for v in pcb.vias if v.drill]
    if pcb.vias:
        sizes = [v.size for v in pcb.vias if v.size]
        if sizes:
            out["min_via_diameter"] = min(sizes)
        if via_drills:
            out["min_via_drill"] = min(via_drills)
        annular = [(v.size - v.drill) / 2.0 for v in pcb.vias
                   if v.size and v.drill and v.size > v.drill]
        if annular:
            out["min_via_annular_width"] = min(annular)
    # Through-hole pad / via drills set the smallest hole diameter on the board.
    hole = list(via_drills)
    for fp in pcb.footprints.values():
        for pad in fp.pads:
            if getattr(pad, "drill", 0):
                hole.append(pad.drill)
    if hole:
        out["min_through_hole_diameter"] = min(hole)
    # #530: the smallest pad / footprint clearance OVERRIDE on copper pads.
    # KiCad floors an override at rules.min_clearance (measured, KiCad 10), so
    # a project whose min_clearance sits ABOVE an override the router honoured
    # flags the copper routed at it. The writeback caps min_clearance here.
    ovr = [pad.local_clearance for fp in pcb.footprints.values() for pad in fp.pads
           if (getattr(pad, "local_clearance", 0) or 0) > 0
           and getattr(pad, "pad_type", "") != "np_thru_hole"
           and any(str(l).endswith(".Cu") for l in (getattr(pad, "layers", None) or []))]
    if ovr:
        out["min_pad_clearance_override"] = min(ovr)
    return out


# --- Shared logic (front-end-agnostic) ------------------------------------
# Both the CLI file-edit path and the GUI pcbnew-API path call these, so the two
# front-ends compute the same DRC floors and differ only in how they apply them
# (issue #160 CLI/GUI-parity rule).

def compute_targets(clearance=None, hole_clearance=None, hole_to_hole=None,
                    edge_clearance=None, track_width=None, via_diameter=None,
                    via_drill=None, minima=None, fab_edge=None, copper_holes=True):
    """Map KiCad rule keys -> target floor (mm) from the routing parameters.
    Each value, when given, becomes a floor; sizes fall back to the board's
    smallest such object (``minima`` from :func:`scan_board_minima`) when the
    param is None. Keys absent from the result => leave that rule alone.

    ``copper_holes`` False: the step laid no track or via (a bare pour,
    :func:`plane_fix_kwargs`), so no copper of its own sits at copper
    clearance from a hole and the copper-to-hole floor does NOT follow
    ``clearance`` -- only an explicit ``hole_clearance`` is written (#1319).

    One key is NOT a rule: ``class_clearance`` (#900), the net-class clearance,
    which is the routed value and never carries the ``min_clearance`` caps.
    ``_RULE_KEYS`` is what keeps it out of the project's rule map; read it with
    :func:`_class_clearance`."""
    minima = minima or {}
    targets = {}
    if clearance is not None:
        targets["min_clearance"] = clearance
        # The NET-CLASS clearance, deliberately a SEPARATE key from the rule
        # floor and never capped (#900). They are different quantities: the
        # rule is an absolute floor KiCad applies underneath everything, the
        # class is the clearance requirement the board was routed to. Read it
        # through _class_clearance(), never by reaching for "min_clearance".
        targets["class_clearance"] = clearance
        # #530: never above the smallest pad clearance override the router
        # honoured -- KiCad floors an override at min_clearance (measured), so
        # a higher floor would flag copper routed correctly at the override.
        # rules.min_clearance is only a floor; the class clearances carry the
        # real requirement, so this costs nothing -- TRUE ONLY BECAUSE the cap
        # stops here. Until #900 it reached the Default class as well, so one
        # part carrying a 2 mil library override turned a requested 0.15 into
        # a 0.0508 mm board (run 25, esp_prog: C1/C3/Q1/Q2/U2 from OLIMEX).
        _ovr = minima.get("min_pad_clearance_override")
        if _ovr is not None and _ovr > 0 and clearance > _ovr:
            targets["min_clearance"] = round(float(_ovr), 6)
    # Hole/copper clearance: explicit value, else the copper-clearance floor
    # when the step drew copper that sits at it (copper_holes).
    hole_clr = (hole_clearance if hole_clearance is not None
                else clearance if copper_holes else None)
    if hole_clr is not None:
        targets["min_hole_clearance"] = hole_clr
    if hole_to_hole is not None:
        targets["min_hole_to_hole"] = hole_to_hole
    # Edge: 0.0 means "no edge clearance was enforced" (the CLI default), NOT
    # "lower the rule to zero" -- writing 0.0 erased the board's own
    # min_copper_edge_clearance so neither KiCad nor check_drc could grade the
    # rule the design demands (issue #338, core1106_cam's 0.5 -> 0.0 clobber).
    # The routers route to max(--board-edge-clearance, the board rule), so a real
    # enforced value is always >= the rule. Unlike the other (aspirational) floors
    # this one is PINNED to the fab copper-to-edge minimum (#441): copper closer
    # to the milled edge than the fab can make is a hard defect, so the recorded
    # floor is max(the routed edge clearance, fab_edge) and apply_targets is
    # allowed to RAISE it above a sub-fab board rule (never lower it below fab).
    edge_target = max(edge_clearance or 0.0, fab_edge or 0.0)
    if edge_target > 0:
        targets["min_copper_edge_clearance"] = round(edge_target, 6)

    # Size minima: take the SMALLER of the routing param and the smallest such
    # object already on the board. A multi-step chain leaves thinner tracks /
    # smaller vias from earlier steps than the current step's param (e.g. a 0.25
    # VREF-repair pass over a board that already has 0.127 USB tracks), so a floor
    # set to just this step's param flags that earlier copper. The DRC floor must
    # sit at or below the smallest object physically present.
    def _floor(param, scanned):
        vals = [v for v in (param, scanned) if v is not None]
        return min(vals) if vals else None
    tw = _floor(track_width, minima.get("min_track_width"))
    if tw is not None:
        targets["min_track_width"] = tw
        # Min copper WEB (#505). KiCad's connection_width rule grades the
        # narrowest copper joining two areas, so a web floor left ABOVE the
        # track floor condemns copper we deliberately routed: a track IS the
        # web wherever it is the sole connection. Left unclamped this was the
        # single largest DRC number in the corpus and none of it was real --
        # ulx5m_gatemate's stock 0.127 web rule against its 0.0889 routed floor
        # flagged 4749 items (13495 of its 14515 tracks sit under 0.127);
        # clamped to the routed floor the same board grades 2, with every other
        # DRC count unchanged. Same principle as the clearance ceiling (#439):
        # grade what was actually built, not the stock aspiration.
        # ONLY-LOWER, like every floor here (apply_* enforces it), and never
        # TURNS THE CHECKER ON: a board with no min_connection (or 0) has
        # KiCad's connection_width check disabled, and enabling it would invent
        # a constraint the author never asked for. See the guard in
        # apply_targets_to_project.
        targets["min_connection"] = tw
    vd = _floor(via_diameter, minima.get("min_via_diameter"))
    if vd is not None:
        targets["min_via_diameter"] = vd
    dr = _floor(via_drill, minima.get("min_through_hole_diameter"))
    if dr is not None:
        targets["min_through_hole_diameter"] = dr
    # VIA-only drill floor, kept SEPARATE from min_through_hole_diameter above.
    # That one is KiCad's board constraint and correctly spans every hole on the
    # board, PADS INCLUDED. The net-class ``via_drill`` field is a different
    # quantity -- the size KiCad draws a NEW VIA at -- and a through-hole pad's
    # drill says nothing about it. Sourcing the class field from the pad-inclusive
    # minimum let one 0.25mm pad drill rewrite every class's via_drill from 0.3 to
    # 0.25, i.e. BELOW the board's own HARD via spec, on a board whose smallest
    # placed via drill was 0.3 (test-board, HW-TB-PCB08).
    vdr = _floor(via_drill, minima.get("min_via_drill"))
    if vdr is not None:
        targets["min_via_drill"] = vdr
    if "min_via_annular_width" in minima:
        targets["min_via_annular_width"] = minima["min_via_annular_width"]
    return targets


def severity_plan(keep_courtyards=False, keep_mask=False, keep_footprint=False,
                  keep_thermal=False, extra_ignore=()):
    """Desired severity per DRC category: {category -> 'ignore' | 'warning'}.
    Applied with only-loosen semantics by the apply_* functions.

    Run-6: `courtyards_overlap` demotes to WARNING, never ignore. The old
    ignore GAGGED KiCad on the one check that catches a stacked part (run 5
    shipped C14-on-R14 with a project file whose severities silenced
    kicad-cli's own courtyards_overlap error). Warning keeps a routed
    board's exit green (the routing checks stay the gate) while the pair
    remains VISIBLE to any reader of the report; check_assembly is the
    blocking arbiter with its class waivers. The other courtyard-shape
    categories (malformed etc.) stay ignore -- library noise, not
    placement facts."""
    plan = {}
    for cat in extra_ignore:
        plan[cat] = "ignore"
    if not keep_courtyards:
        for cat in COURTYARD_CATS:
            plan[cat] = "ignore"
        plan["courtyards_overlap"] = "warning"
    if not keep_mask:
        for cat in MASK_CATS:
            plan[cat] = "ignore"
    if not keep_footprint:
        for cat in FOOTPRINT_CATS:
            plan[cat] = "ignore"
    if not keep_thermal:
        for cat in WARNING_CATS:
            plan[cat] = "warning"
    return plan


def apply_targets_to_project(proj: dict, targets: dict, sev_plan: dict,
                             ignore_current_warnings=False,
                             diff_pair_gap=None, diff_pair_width=None,
                             clamp_nondefault_netclasses=False):
    """Apply the floors + severity plan to a parsed ``.kicad_pro`` dict, only
    ever loosening (lowering a constraint / lowering a severity rank), never
    tightening. Returns a list of human-readable change strings.

    ``diff_pair_gap`` / ``diff_pair_width`` are accepted for signature
    compatibility and IGNORED (#842): they are net-class draw defaults, and a
    routing step never writes those -- lowering them was the #842 ratchet.

    ``clamp_nondefault_netclasses`` clamps the NON-Default net classes'
    clearance/track/via floors DOWN to the routed values. The real entry points
    (``fix_project_for_output`` and the CLI) default it ON (#439) because stock net
    classes are largely ASPIRATIONAL: corpus and real boards route below them (even
    the human-routed references violate their own class -- zynq has 499 clearance
    violations at its 0.2 class, routed ~0.1), so keeping the stock class in the
    output manufactures phantom sub-class DRC on copper that was routed correctly at
    the fab floor. The router honors min(class, --clearance-ceiling) in-run and the writeback
    records that same floor, so KiCad grades exactly what was routed. Passing False
    (the caller routed without --clearance-ceiling, i.e. it HONORED the classes)
    PRESERVES the original class spec. The Default-class /
    rules.min_clearance write below is UNRELATED to this flag and stays: it records
    the actual ROUTING clearance (the router's config.clearance), only-lowering so a
    board routed tighter than its stock Default class does not storm."""
    EPS = 1e-9
    ds = proj.setdefault("board", {}).setdefault("design_settings", {})
    rules = ds.setdefault("rules", {})
    sev = ds.setdefault("rule_severities", {})
    changes = []

    for key, target in targets.items():
        if key not in _RULE_KEYS:
            continue          # #900: class_clearance is not a KiCad rule name
        if target is None:
            continue
        target = round(float(target), 6)
        cur = rules.get(key)
        if key in REPO_RULE_KEYS:
            # A repo-private rule KiCad's save may have dropped: decide on the
            # records' copy too, and keep that copy in step with the rule.
            cur = _effective_rules(proj).get(key)
            if cur is None or cur > target + EPS:
                changes.append(f"rules.{key}: {cur} -> {target} mm")
                rules[key] = target
                proj.setdefault("kicad_routing_tools", {}).setdefault(
                    REPO_RULES_KEY, {})[key] = target
            continue
        # min_copper_edge_clearance is the one floor that may RAISE (#441): it is
        # pinned to max(board rule, fab copper-to-edge minimum) because sub-fab
        # edge copper is a hard defect, so a board declaring a tiny/zero edge rule
        # is lifted to the fab floor rather than kept. `target` already carries
        # max(routed edge, fab_edge); take max with cur so a board rule ABOVE the
        # fab floor (e.g. 0.5) is preserved.
        # Min copper WEB (#505): only ever LOWER an existing, ENABLED rule.
        # KiCad's connection_width checker is off at 0 (and absent means 0), so
        # writing a value where the board had none would switch on a check the
        # author never enabled -- the opposite of this function's only-loosen
        # guarantee, even though the number itself is a loosening.
        if key == "min_connection" and not cur:
            continue
        if key == "min_copper_edge_clearance":
            new = max(cur or 0.0, target)
            if cur is None or abs(new - cur) > EPS:
                changes.append(f"rules.{key}: {cur} -> {new} mm (fab-edge pin)")
                rules[key] = new
            continue
        if cur is None or cur > target + EPS:        # lower only; never raise
            changes.append(f"rules.{key}: {cur} -> {target} mm")
            rules[key] = target

    # KiCad enforces copper clearance PER NET CLASS (rules.min_clearance alone
    # does not relax it), so keep the Default class at the floor too -- creating
    # a COMPLETE one if the project has none (a sparse class is ignored by KiCad,
    # which then falls back to the stock 0.2 mm default).
    # ONLY clearance. track_width / via_diameter / via_drill / diff_pair_* are
    # draw defaults (see _NETCLASS_WRITABLE_FIELDS) and are never written by a
    # routing step: lowering them was the #842 ratchet. ``diff_pair_gap`` /
    # ``diff_pair_width`` are still accepted for signature compatibility and
    # ignored.
    # The ROUTED clearance, not the capped rule floor (#900): see
    # _class_clearance. Reading "min_clearance" here is what turned a requested
    # 0.15 into a 0.0508 board on any part carrying a 2 mil pad override.
    nc_map = {"clearance": _class_clearance(targets)}
    net_settings = proj.setdefault("net_settings", {})
    net_settings.setdefault("meta", {"version": 0})  # KiCad needs this to read classes
    classes = net_settings.setdefault("classes", [])
    default_cls = next((c for c in classes if c.get("name") == "Default"), None)
    if default_cls is None and any(v is not None for v in nc_map.values()):
        default_cls = dict(_DEFAULT_NETCLASS)
        # Born at the ROUTED clearance, not at the template's stock 0.2 then
        # lowered (#900). Nothing is being loosened away here -- the project
        # had no class at all -- and the only-lower loop below cannot RAISE the
        # template value, so a board routed at 0.3 used to be handed a 0.2
        # class it violates everywhere.
        if nc_map.get("clearance") is not None:
            default_cls["clearance"] = round(float(nc_map["clearance"]), 6)
        classes.insert(0, default_cls)
        changes.append(f"net_class[Default]: created (project had none), "
                       f"clearance {default_cls['clearance']} mm")
    if default_cls is not None:
        for field, target in nc_map.items():
            if target is None:
                continue
            target = round(float(target), 6)
            cur = default_cls.get(field)
            # A declared 0 is UNSET, not a floor of zero: KiCad writes 0 for
            # "not configured" and then grades the class at rules.min_clearance.
            # This branch read it as "already below the target" and left it,
            # so a project whose Default class was never configured (the
            # human-routed originals the bench boards descend from) shipped
            # every chain step with clearance 0.0 -- which route.py pins up to
            # the fab floor and the GUI replaces with its Min Clearance
            # control's default, 0.25 on copper routed at 0.1. The same rule
            # every reader in this module applies (project_copper_clearance:
            # "None if unset or 0").
            if cur is not None and cur <= EPS:
                cur = None
            if cur is None or cur > target + EPS:
                changes.append(f"net_class[Default].{field}: {cur} -> {target} mm")
                default_cls[field] = target
    # NON-Default classes (#295 follow-up; ON by default since #439): stock net
    # classes are largely aspirational, so copper routed at the real fab floor
    # (min(class, --clearance-ceiling)) would storm KiCad's per-net-class DRC if the
    # output kept the stock class. Clamp each non-Default class DOWN to the routed
    # values so grading matches the copper. clamp=False (the caller routed WITHOUT
    # --clearance-ceiling, honoring the classes) preserves the original class rules.
    if clamp_nondefault_netclasses:
        for cls in classes:
            if cls is default_cls or not isinstance(cls, dict):
                continue
            cname = cls.get("name", "?")
            for field, target in nc_map.items():
                if target is None or field not in _NONDEFAULT_CLAMP_FIELDS:
                    continue
                target = round(float(target), 6)
                cur = cls.get(field)
                if cur is not None and cur > target + EPS:
                    changes.append(f"net_class[{cname}].{field}: {cur} -> {target} mm")
                    cls[field] = target

    def loosen_severity(cat, level):
        cur = sev.get(cat, "error")  # KiCad's default severity is "error"
        if _SEV_RANK.get(level, 2) < _SEV_RANK.get(cur, 2):
            changes.append(f"severity[{cat}]: {sev.get(cat)} -> {level}")
            # #856: a severity change is reversible only if the previous value
            # survives. Record it once (the FIRST writer's value, so a chain of
            # steps keeps the author's setting, not an intermediate one).
            saved = proj.setdefault("kicad_routing_tools", {}) \
                        .setdefault("saved_severities", {})
            if cat not in saved:
                saved[cat] = sev.get(cat, "error")
            sev[cat] = level

    if ignore_current_warnings:
        for cat, s in list(sev.items()):
            if s == "warning":
                loosen_severity(cat, "ignore")
    for cat, level in sev_plan.items():
        loosen_severity(cat, level)
    return changes


def add_drc_fix_args(parser, *, include_no_fix=True):
    """Add the post-route DRC-settings-fix CLI options shared by the routing
    front-ends (``route.py`` / ``route_diff.py`` / ``route_planes.py`` /
    ``repair_planes.py`` / ``bga_fanout`` / ``qfn_fanout``, and this script's
    own CLI). Wiring a new shared DRC-fix flag in here
    adds it to all of them at once; pair with :func:`drc_fix_kwargs` to forward the
    parsed values into :func:`fix_project_for_output`.

    ``include_no_fix=False`` omits ``--no-fix-drc-settings`` (the standalone
    ``fix_kicad_drc_settings`` script always fixes, so the flag is meaningless there)."""
    g = parser.add_argument_group("DRC settings (post-route, issue #160)")
    if include_no_fix:
        g.add_argument("--no-fix-drc-settings", action="store_true",
                       help="Do not adjust the output's .kicad_pro DRC constraints to match the "
                            "routed clearances/sizes afterwards. By default the written project's "
                            "Board Setup floors are loosened to the routed values so KiCad's DRC "
                            "only flags genuine problems.")
    g.add_argument("--relax-drc-severities", action="store_true",
                   help="ALSO lower the project's DRC severities for the non-routing "
                        "categories (courtyard shapes, solder-mask bridges, footprint/"
                        "library issues incl. annular_width -> ignore; starved_thermal -> "
                        "warning; courtyards_overlap -> warning). OFF by default (#856): a "
                        "routing step never changes what the project counts as a "
                        "violation unless asked. Each change is logged and the previous "
                        "value is kept under kicad_routing_tools.saved_severities.")
    g.add_argument("--keep-thermal", action="store_true",
                   help="Only with --relax-drc-severities (the standalone script's "
                        "--relax-severities): leave starved_thermal at its current severity "
                        "instead of lowering it to warning. Without it a routing step does "
                        "not touch DRC severities, so this does nothing.")
    g.add_argument("--enable-used-layers", action="store_true",
                   help="Add any layer the board uses but that is missing from its (layers) table "
                        "back into the .kicad_pcb, so KiCad shows it as selectable and stops "
                        "flagging item_on_disabled_layer. OFF by default (it edits the board, not "
                        "just DRC settings).")
    return parser


def drc_fix_kwargs(args):
    """Map args parsed via :func:`add_drc_fix_args` to :func:`fix_project_for_output`
    keyword arguments (the shared DRC-fix flags only -- per-script routing floors
    like clearance/track/via are passed separately by each caller)."""
    # #439/#530: clamp NON-Default classes to the routed clearance whenever the
    # caller routed with --clearance-ceiling (args._clamp_netclasses, set by the
    # route / route_diff / plane mains); without it the classes were honored in
    # full, so the writeback preserves them. Callers that do not set the
    # attribute (fanout, standalone runs) clamp by default -- the safe choice, since
    # clamping only ever lowers the output class to the copper actually routed.
    clamp = getattr(args, "_clamp_netclasses", True)
    return dict(keep_thermal=args.keep_thermal, enable_layers=args.enable_used_layers,
                relax_severities=getattr(args, "relax_drc_severities", False),
                clamp_nondefault_netclasses=clamp)


#: The project record of a Default class clearance an AUTOMATIC descent
#: lowered (#1160), under ``kicad_routing_tools``: ``{from, to, nets}``.
CLASS_CLEARANCE_RELAXED_KEY = "class_clearance_relaxed"


def record_class_clearance_descent(proj: dict, class_before):
    """``(record, lowered_here)``: did this step's AUTOMATIC clearance descent
    lower the project's Default class below what the step asked for?

    The writeback stores the run's smallest clearance as the Default CLASS
    (the ratchet #489 keeps), so one net a rescue reconnected at 0.2567 turns
    a 0.3 class into 0.2567, and every later step that reads the project
    routes EVERY Default net there -- complex_hierarchy's step 2 then shipped
    16 pad-segment grazes at the authored 0.3, while every checker graded the
    new class and read clean. A class lowered by the step's own request
    (``--clearance``, ``--clearance-ceiling``) is a decision, not this: the
    test is against the clearance the descent rows say the step ASKED for
    (``design_rules`` ``clearance`` rows, fab_tiers.note_narrowing), never
    against the class the project had.

    Records ``{from, to, nets}`` under ``kicad_routing_tools`` and carries an
    earlier step's record forward (its ``from`` and nets are kept), so a later
    step and ``check_complete --authored-from`` still see it."""
    from fab_tiers import escalation_summary, project_default_class_clearance
    kr = proj.get("kicad_routing_tools") or {}
    rec = dict(kr.get(CLASS_CLEARANCE_RELAXED_KEY) or {})
    after = project_default_class_clearance(proj)
    try:
        rows = [r for r in escalation_summary().get("narrowed") or ()
                if r.get("kind") == "clearance"]
    except Exception:                                        # noqa: BLE001
        rows = []
    if not rows or after is None:
        return (rec or None), False
    asked = max(float(r["requested"]) for r in rows)
    # Only a class THIS step lowered, and only below what it asked for.
    lowered = class_before is None or after < float(class_before) - _FLOOR_EPS
    if not lowered or after >= asked - _FLOOR_EPS:
        return (rec or None), False
    names = {r.get("net_name") or f"net {r.get('net')}" for r in rows
             if float(r["delivered"]) < asked - _FLOOR_EPS}
    # `from` is what the class would read without the descent: the request,
    # or the class itself when the request sat above it.
    base = asked if class_before is None else min(asked, float(class_before))
    rec = {"from": round(max(float(rec.get("from") or 0.0), base), 6),
           "to": round(after, 6),
           "nets": sorted(set(rec.get("nets") or ()) | names)}
    proj.setdefault("kicad_routing_tools", {})[CLASS_CLEARANCE_RELAXED_KEY] = rec
    return rec, True


def class_clearance_disclosure(rec, lowered_here, class_now=None):
    """The lines that say a Default class clearance was lowered by a descent."""
    if not rec:
        return []
    nets = rec.get("nets") or []
    who = ", ".join(nets[:6]) + (f" (+{len(nets) - 6} more)" if len(nets) > 6 else "")
    frm, to = rec.get("from"), rec.get("to")
    if class_now is not None and class_now >= (frm or 0) - _FLOOR_EPS:
        return []          # the class was raised back since: nothing to say
    if lowered_here:
        head = (f"  DEFAULT CLASS CLEARANCE LOWERED BY A DESCENT -- {frm:g} -> "
                f"{to:g} mm: an automatic clearance descent on {who} routed at "
                f"{to:g} mm, and the writeback stores the run's smallest clearance "
                f"as the Default class.")
    else:
        head = (f"  Default class clearance {class_now if class_now is not None else to:g} "
                f"mm, unchanged by this step but below the {frm:g} mm an earlier "
                f"step asked for: an automatic descent on {who} lowered it.")
    return [head,
            f"    Every later step that reads this project routes EVERY Default "
            f"net at the lower value, and every checker grades against it, so "
            f"the copper reads clean. Pass --clearance {frm:g} to route the "
            f"other nets at {frm:g} again."]


def warn_if_class_clearance_relaxed(input_pcb) -> bool:
    """At the start of a routing step: say so when the input project's Default
    class carries a clearance an earlier step's descent lowered (#1160), since
    an omitted --clearance is about to route every Default net at it."""
    if not input_pcb:
        return False
    try:
        import tool_records
        proj = tool_records.load_project(os.path.splitext(input_pcb)[0] + ".kicad_pro")
        from fab_tiers import project_default_class_clearance
        rec = (proj.get("kicad_routing_tools") or {}).get(CLASS_CLEARANCE_RELAXED_KEY)
        lines = class_clearance_disclosure(rec, False,
                                           project_default_class_clearance(proj))
    except Exception:                                        # noqa: BLE001
        return False
    for line in lines:
        print(line)
    return bool(lines)


def warn_if_missing_project_floor(input_pcb) -> bool:
    """Complain LOUDLY when an input board arrives WITHOUT its sibling ``.kicad_pro`` (#441).

    The ``.kicad_pro`` carries the DRC floor -- the Default-netclass clearance/width the
    board was actually routed to. A bare ``cp board.kicad_pcb copy.kicad_pcb`` that omits
    the sibling ``.kicad_pro`` strands that floor, and the damage comes in two forms:

    1. Phantom DRC. The next routing step reads NO project, resolves its floor from the
       STOCK (looser) netclass, and its writeback stamps that looser floor over copper
       routed tighter -- so KiCad grades correct sub-floor copper as a clearance
       violation (icepi_zero: a dropped 0.09 floor became 0.10 -> 160 phantom grazes).
    2. Front divergence. With no project, the CLI seeds a minimal one pinned to the FAB
       floors while a live pcbnew board carries KiCad's stock defaults -- so the CLI and
       the GUI legitimately route DIFFERENT copper from the same board (the copper-parity
       gate measured 12/8 divergent GND segments from exactly this before its harness
       started staging a project).

    The old single-line text version of this warning scrolled away unread through an
    entire debugging session, hence the banner. It stays a WARNING, not an error: a
    pristine never-routed board legitimately has no project yet, so aborting would
    break first-step runs -- the banner is for the mid-chain copy that lost its floor.

    Returns True iff the project is missing (so callers may also record it in a summary)."""
    if not input_pcb:
        return False
    proj = os.path.splitext(input_pcb)[0] + ".kicad_pro"
    if os.path.isfile(proj):
        return False
    try:
        from terminal_colors import RED, RESET
    except Exception:
        RED = RESET = ""
    bar = "!" * 74
    print(f"{RED}{bar}\n"
          f"!! NO .kicad_pro NEXT TO '{os.path.basename(input_pcb)}' (#441)\n"
          f"!!\n"
          f"!! The sibling project carries the board's DRC floor. Without it this\n"
          f"!! step resolves clearances from the STOCK netclass, which can be LOOSER\n"
          f"!! than the copper already on the board:\n"
          f"!!   - KiCad will then report phantom sub-clearance DRC on correct copper\n"
          f"!!   - CLI and GUI runs will route DIFFERENT copper from this same board\n"
          f"!!\n"
          f"!! If this board was copied or renamed, bring its .kicad_pro along:\n"
          f"!!   python3 py_router/copy_board.py src.kicad_pcb dst.kicad_pcb\n"
          f"!! (Ignore only if this is a pristine board that has never been routed.)\n"
          f"{bar}{RESET}")
    return True


def apply_routed_floors(board_pcb: str, clearance=None, hole_clearance=None,
                        clamp_nondefault_netclasses=False, verbose=False):
    """Lower ``board_pcb``'s sibling ``.kicad_pro`` to the COPPER floors this run
    routed to, MID-RUN, so anything that grades the board before ``main()``'s
    authoritative writeback sees the floors the board will actually SHIP with.

    Why (#650). The plane finalize's oracle audits ``output_file`` while its
    sibling project is still the one :func:`seed_project_for_output` copied from
    the INPUT -- so the audit fills the pours at the input's *declared* floors
    while the shipped board is graded at the *clamped* ones. Measured on
    orangecrab (``runs_set3``, identical copper, only the staged project
    varied): 57 unconnected links under the input's project vs 46 under the
    written-back one -- GND 19 vs 11 -- and ``rules.min_hole_clearance``
    (0.25 -> 0.0889) accounted for ALL of it, because the zone filler pulls
    copper back from every drill by that rule. The audit's extra links are
    phantom, and kicad-cli is the DEMAND gate of the #648 source union, so they
    become junk welds in the longest leg of the chain.

    COPPER floors only -- clearance / hole-to-copper, and the net classes that
    follow them, which is what changes zone fill. Track / via / annular floors
    are left to the final writeback: measured inert for the fill (the same board
    graded 57 either way with only those clamped) and they want the board's
    scanned minima, which mid-run copper has not settled.

    How big the gap is depends on CHAIN POSITION, so do not read 57/46 as a
    per-step cost: the declared floors survive only until the first writeback
    (down that same chain ``min_hole_clearance`` goes 0.25 -> 0.09 at step 1 and
    stays there, leaving later steps stale by ~1 um -- inert). This earns its
    keep on the first step over a board still carrying its declared floors,
    which includes the ordinary case of a board that arrives with pours already
    drawn, and on the aspirational stock netclass (0.2 declared, routed 0.1).

    ``clamp_nondefault_netclasses`` defaults OFF, unlike the writeback's ON
    (#439): whether the non-Default classes get clamped depends on the caller
    having passed a ``--clearance`` ceiling, which is a ``main()`` fact and not
    an engine one, and lowering them here could ship a tightened class on a run
    that meant to honor them. The Default class and ``rules.min_clearance`` are
    written regardless -- that is :func:`apply_targets_to_project`'s documented
    behaviour and matches what the writeback will do either way.

    Only-loosen, via the same :func:`apply_targets_to_project` the writeback
    uses, so :func:`fix_project_for_output` stays authoritative and can never
    conflict with what this wrote. No-op when the board has no sibling project
    or no floor was given. Returns the change strings (empty = nothing done)."""
    if not board_pcb or (clearance is None and hole_clearance is None):
        return []
    pro = find_project(board_pcb)
    if not os.path.isfile(pro):
        return []
    try:
        import tool_records
        proj = tool_records.load_project(pro)
    except (OSError, ValueError):
        return []
    # The floors the project declares BEFORE this pass lowers them. Captured
    # here, not after, because this pass runs BEFORE the authoritative
    # writeback and is therefore the first thing in the chain that can move a
    # FAB_FLOOR_KEYS rule -- see seed_fab_floor_origin for what went silent
    # while this was the writeback's private business.
    _rules_before = _effective_rules(proj)
    targets = compute_targets(clearance=clearance, hole_clearance=hole_clearance)
    changes = apply_targets_to_project(
        proj, targets, {},
        clamp_nondefault_netclasses=clamp_nondefault_netclasses)
    if not changes:
        return []
    _origin, _origin_seeded = seed_fab_floor_origin(proj, _rules_before)
    if _origin_seeded:
        proj.setdefault("kicad_routing_tools", {})["fab_floor_origin"] = _origin
        changes = list(changes) + ["kicad_routing_tools.fab_floor_origin: recorded"]
    try:
        # Atomic replace, same discipline as the writeback (#513 item 12).
        tool_records.stamp_revision(proj)
        tmp_pro = pro + ".tmp"
        with open(tmp_pro, "w") as f:
            json.dump(proj, f, indent=2)
            f.write("\n")
        os.replace(tmp_pro, pro)
    except OSError:
        return []
    if verbose:
        print(f"  In-run DRC floors (#650): lowered {len(changes)} value(s) in "
              f"{os.path.basename(pro)} to the routed floors so the in-run "
              f"audit grades what ships")
        for c in changes:
            print(f"      {c}")
    return changes


def carry_input_siblings(output_pcb: str, input_pcb=None, skip=()) -> list:
    """Copy each of the input board's siblings (``copy_board.SIBLING_EXTS``,
    less ``skip``) beside the output board, unless the output already has
    one. An existing output sibling is never overwritten. Each copy is atomic
    (tmp + ``os.replace``). Returns the paths written."""
    import shutil
    if not (output_pcb and input_pcb):
        return []
    from copy_board import SIBLING_EXTS
    in_base = os.path.splitext(input_pcb)[0]
    out_base = os.path.splitext(output_pcb)[0]
    written = []
    for ext in SIBLING_EXTS:
        if ext in skip:
            continue
        src, dst = in_base + ext, out_base + ext
        if os.path.isfile(src) and not os.path.isfile(dst) \
                and os.path.abspath(src) != os.path.abspath(dst):
            tmp = dst + '.tmp'
            shutil.copyfile(src, tmp)
            os.replace(tmp, dst)
            written.append(dst)
    return written


def seed_project_for_output(output_pcb: str, input_pcb=None):
    """Carry the input board's siblings over to the output path BEFORE the
    board file is written (#513 item 12, #1286). The full floor writeback
    (``fix_project_for_output``) must run after the board exists because it scans
    the written copper -- so a mid-run kill in that window used to leave a
    valid board with NO sibling project, and the next step silently fell back
    to stock netclass floors (the #441/#338 class; peaksat_obc_adcs, twice).
    Seeding the input's project first means the window leaves the INPUT's
    floors -- slightly stale at worst, never the stock fallback.

    Every sibling is seeded here, not only the ``.kicad_pro``, because this is
    the one carry every routing writer makes. The writeback is skipped under
    ``--no-fix-drc-settings``, and when the ``.kicad_dru`` was carried only
    there, such an output reached kicad-cli without the board's custom rules:
    a pad pair legal under its 0.16 mm rule was graded a clearance violation
    on 12 of 13 single-net re-routes (#1286). Idempotent: an existing output
    sibling is never touched. Returns the paths seeded.

    An existing output ``.kicad_pro`` whose rules differ from the input's is
    still kept, but said so on the first write of that output in this process
    (``warn_leftover_output_project``)."""
    warn_leftover_output_project(output_pcb, input_pcb)
    return carry_input_siblings(output_pcb, input_pcb)


#: Outputs this process has already seeded: the leftover warning is about the
#: project found BEFORE the step's first write, not the one its own writeback
#: lowered since.
_SEEDED_OUTPUTS = set()


def _project_rules(path):
    """A project's rule floors and net classes, or None when unreadable."""
    try:
        with open(path, encoding='utf-8') as f:
            proj = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(proj, dict):
        return None
    rules = ((proj.get('board') or {}).get('design_settings') or {}).get('rules')
    return rules, (proj.get('net_settings') or {}).get('classes')


def warn_leftover_output_project(output_pcb, input_pcb=None):
    """Warn when the output already has a ``.kicad_pro`` whose rules differ
    from the input's: one left by an earlier run to the same path. It is kept
    (an existing output sibling is never overwritten), and the floor writeback
    then lowers from ITS floors, so a rerun silently inherits an earlier
    attempt's relaxations ("unchanged by this step but still below the board's
    ORIGINAL"). A PCB-chat routing plan run twice in one folder on 2026-10-22
    kept a 0.127 mm track floor from its first attempt that way. Once per
    output per process. Returns True when it warned."""
    if not (output_pcb and input_pcb):
        return False
    key = os.path.abspath(output_pcb)
    if key == os.path.abspath(input_pcb) or key in _SEEDED_OUTPUTS:
        return False
    _SEEDED_OUTPUTS.add(key)
    in_pro, out_pro = find_project(input_pcb), find_project(output_pcb)
    if not (os.path.isfile(in_pro) and os.path.isfile(out_pro)):
        return False
    if _project_rules(in_pro) == _project_rules(out_pro):
        return False
    print(f"  WARNING: {out_pro} is left from an earlier run and is kept (an "
          f"existing output sibling is never overwritten). Its design rules "
          f"differ from the input's, and this step's floor writeback lowers "
          f"from them, not from {os.path.basename(in_pro)}. Route to a fresh "
          f"output path, or delete it first, to start from the input's rules.")
    return True


def fanout_written_floors(kind, tracks, vias):
    """Which floors a fanout step's writeback may lower: (any, vias) (#1195).

    A QFN/QFP fanout writes only the floors of copper it drew: nothing when the
    run changed no copper, and the via and hole floors only when it placed a
    via (stub mode never does). bga_fanout writes them on every run. Both
    fronts read this -- qfn_fanout's main for the file, the GUI fanout tab for
    the live board -- because the tab kept lowering the via floors on every
    stub run after the CLI stopped.
    """
    if kind != 'qfn':
        return True, True
    return bool(tracks or vias), bool(vias)


def fanout_fix_kwargs(kind, tracks, vias, *, clearance, track_width,
                      via_size, via_drill):
    """THE fanout writeback: the :func:`fix_project_for_output` keywords a
    fanout step's floors are written with, or None when it writes none.

    One decision for both fronts and both fanouts. bga_fanout's and
    qfn_fanout's mains pass these to :func:`fix_project_for_output` for the
    file, and the GUI fanout tab passes the same to :func:`apply_fix_to_board`
    for the live board. What they write, from :func:`fanout_written_floors`
    (#1195): the routed clearance (which also sets the copper-to-hole floor),
    the track width, the via size and drill only when the step may write via
    floors, and every non-Default class clamped to the clearance (#439 -- a
    fanout routes its escapes at that one clearance whatever the class). NO
    hole-to-hole and no edge clearance: the CLI fanouts pass neither, so the
    edge floor is the fab copper-to-edge pin alone. The tab used to write its
    hole-to-hole and edge settings, never the copper-to-hole floor, and
    clamped classes only under the Min-Clearance ceiling."""
    floors, via_floors = fanout_written_floors(kind, tracks, vias)
    if not floors:
        return None
    return {"clearance": clearance, "track_width": track_width,
            "via_diameter": via_size if via_floors else None,
            "via_drill": via_drill if via_floors else None,
            "clamp_nondefault_netclasses": True}


def plane_fix_kwargs(tracks, vias, *, clearance, hole_to_hole, track_width,
                     via_size, via_drill):
    """THE plane-step writeback: the floors a pour step writes, from the
    copper it drew (#1319, #1195's rule for a fanout applied to a pour).

    route_planes' main passes these to :func:`fix_project_for_output` for the
    file and the Planes tab to ``compute_targets`` / ``update_live_drc_floors``
    for the live board, so the two fronts write the same floors. ``tracks`` and
    ``vias`` count what the step ADDED (:func:`drawn_copper` on the CLI, the
    tab's applied lists in the GUI):

    * the via floors (diameter, drill) and hole-to-hole only when it placed a
      via, and the track floor only when it laid a track. A bare pour passed
      the run's sizes anyway, so interf_u's GND pour wrote min_via_drill
      None -> 0.6 for vias that do not exist, and multichannel_mixer's took
      min_track_width 0.3 -> 0.2 and min_via_diameter 1.5 -> 0.6;
    * the copper-to-hole floor only when it laid either (``copper_holes``): a
      pour keeps the board's hole clearance itself (its fill and KiCad's refill
      knock out at it), so daplink's pour lowering it 0.25 -> 0.2 bought nothing;
    * the copper clearance always: a KiCad refill regrows the pour at
      max(zone clearance, net class), so a pour laid below its class keeps its
      corridors only if the floor drops with it (route_planes'
      ``_resolve_zone_clearance``).

    A size the step leaves out still falls back to the board's own smallest
    object in ``compute_targets``, as every writeback does."""
    drew = bool(tracks or vias)
    return {"clearance": clearance,
            "hole_to_hole": hole_to_hole if vias else None,
            "track_width": track_width if tracks else None,
            "via_diameter": via_size if vias else None,
            "via_drill": via_drill if vias else None,
            "copper_holes": drew}


def drawn_copper(input_pcb, output_pcb):
    """``(tracks, vias)``: how many track segments and vias ``output_pcb``
    carries that ``input_pcb`` does not, matched by position, layer and net
    (graphic copper is not a track). What a file-writing step drew, for
    :func:`plane_fix_kwargs`. ``(None, None)`` when either board cannot be
    read -- the caller then keeps the sizes it would have written."""
    try:
        from kicad_parser import parse_kicad_pcb
        a, b = parse_kicad_pcb(input_pcb), parse_kicad_pcb(output_pcb)
    except Exception:                                           # noqa: BLE001
        return None, None

    def _seg(s):
        p, q = ((round(s.start_x, 4), round(s.start_y, 4)),
                (round(s.end_x, 4), round(s.end_y, 4)))
        return (min(p, q), max(p, q), s.layer, s.net_id, round(s.width, 4))

    def _via(v):
        return (round(v.x, 4), round(v.y, 4), v.net_id, round(v.size, 4),
                round(v.drill, 4))

    old_s = {_seg(s) for s in a.segments if not getattr(s, 'graphic', False)}
    old_v = {_via(v) for v in a.vias}
    tracks = sum(1 for s in b.segments
                 if not getattr(s, 'graphic', False) and _seg(s) not in old_s)
    vias = sum(1 for v in b.vias if _via(v) not in old_v)
    return tracks, vias


def apply_fix_to_board(board, *, minima, clearance=None, hole_clearance=None,
                       hole_to_hole=None, edge_clearance=None, track_width=None,
                       via_diameter=None, via_drill=None, diff_pair_gap=None,
                       diff_pair_width=None, clamp_nondefault_netclasses=True):
    """:func:`fix_project_for_output`'s FLOOR writeback, applied to a LIVE
    board -- the same :func:`compute_targets` call over the same keywords,
    through :func:`apply_targets_to_board`. ``minima`` is the board's own
    smallest objects (gui_utils.board_minima_from_live, the live twin of
    :func:`scan_board_minima`). Returns the change strings.

    Severities, the origin record's disclosure and the project file are not
    written here: the caller's step ends with gui_utils.update_live_drc_floors,
    which discloses, and the live board is what KiCad saves."""
    path = ""
    try:
        path = board.GetFileName() or ""
    except Exception:                                           # noqa: BLE001
        path = ""
    targets = compute_targets(clearance=clearance, hole_clearance=hole_clearance,
                              hole_to_hole=hole_to_hole,
                              edge_clearance=edge_clearance,
                              track_width=track_width, via_diameter=via_diameter,
                              via_drill=via_drill, minima=minima or {},
                              fab_edge=fab_edge_floor(path or None))
    return apply_targets_to_board(board, targets, {},
                                  diff_pair_gap=diff_pair_gap,
                                  diff_pair_width=diff_pair_width,
                                  clamp_nondefault_netclasses=clamp_nondefault_netclasses)


def fix_project_for_output(output_pcb: str, input_pcb=None, *, clearance=None,
                           hole_clearance=None, hole_to_hole=None, edge_clearance=None,
                           track_width=None, via_diameter=None, via_drill=None,
                           diff_pair_gap=None, diff_pair_width=None,
                           keep_courtyards=False, keep_mask=False, keep_footprint=False,
                           keep_thermal=False, enable_layers=False,
                           clamp_nondefault_netclasses=True,  # #439: clamp by default
                           extra_ignore=(), verbose=True, minima=None,
                           relax_severities=False, copper_holes=True):
    """Make the DRC settings of a freshly written board consistent with the
    routing floors (issue #160 auto-invoke). Ensures ``output_pcb`` has a sibling
    ``.kicad_pro`` -- copying the input board's project if the output is a new
    file, or seeding a minimal complete one if the input has none -- then applies
    the floors and severity plan. Edits the ``.kicad_pro`` (DRC settings); the
    board's DRC-rule format is preserved.

    When ``enable_layers`` is True (the CLI ``--enable-used-layers`` flag, OFF by
    default), it also adds any layer the board uses but that is missing from its
    ``(layers)`` table back into that table in the ``.kicad_pcb``
    (``enable_used_layers``), so KiCad shows it as selectable and stops flagging
    ``item_on_disabled_layer`` -- a format-preserving text edit. It is opt-in
    because that mutates board structure (not just DRC settings); the default
    leaves the ``.kicad_pcb`` untouched. Returns the ``.kicad_pro`` path, or None
    if nothing was done."""
    import shutil
    if enable_layers:
        enable_used_layers(output_pcb, verbose=verbose)
    out_pro = find_project(output_pcb)
    if not os.path.isfile(out_pro):
        in_pro = find_project(input_pcb) if input_pcb else None
        if in_pro and os.path.isfile(in_pro) and os.path.abspath(in_pro) != os.path.abspath(out_pro):
            shutil.copyfile(in_pro, out_pro)              # carry the user's real project over
        else:
            with open(out_pro, "w") as f:                 # seed a minimal valid project
                json.dump({"board": {"design_settings": {"rules": {}, "rule_severities": {}}},
                           "meta": {"filename": os.path.basename(out_pro), "version": 1},
                           "net_settings": {"classes": [], "meta": {"version": 0}}},
                          f, indent=2)
    # Carry the input's other siblings (copy_board.SIBLING_EXTS) the same way:
    # the .kicad_dru (#498: the router routed to its per-layer clearances, and
    # every grader resolves them from the OUTPUT board's sibling), the
    # .design-brief.json (#711: the declared intent, without which the next
    # grade infers every edge from the current pose, #1190) and the .kicad_prl.
    # Never overwrite an existing output sibling. A routing writer has already
    # seeded them (seed_project_for_output); this covers a caller that did not
    # write the board through one.
    carry_input_siblings(output_pcb, input_pcb, skip=(".kicad_pro",))

    # The records (origin, protected nets, ...) as the board holds them: the
    # newer of the project's copy and the board file's, which is where a GUI
    # session's records survive the user's KiCad save (tool_records).
    import tool_records
    proj = tool_records.load_project(out_pro)
    # The board's DECLARED manufacturing floors, before this function lowers
    # them to whatever was routed. Kept so the relaxation can be disclosed
    # (see _fab_floor_disclosure): a clearance clamp is a grading decision, a
    # track/via floor is a statement about what the fab can make.
    _rules_before = _effective_rules(proj)
    # The floors the board declared BEFORE this chain touched anything. Seeded
    # on the first writeback and carried down with the project (same mechanism
    # as protected_nets / net_impedance, #521), so step N still knows what step
    # 0 started from. Without it the disclosure below compares against its
    # immediate input and goes silent for every step after the first -- which
    # is exactly how run 14 shipped 10 vias under its declared 0.5 mm with one
    # banner at R1 and none at R4 or R5.
    _origin, _origin_seeded = seed_fab_floor_origin(proj, _rules_before)
    from fab_tiers import project_default_class_clearance as _pdcc
    _class_before = _pdcc(proj)     # #1160
    # `minima` lets a caller that ALREADY has the board in memory supply these
    # instead of us re-parsing the file. The GUI does: scan_board_minima ->
    # parse_kicad_pcb allocates thousands of GC-tracked objects, and the GUI
    # calls this from inside a wx timer dispatch where that allocation burst
    # triggers a mid-dispatch collection that segfaults (see
    # gui_utils.board_minima_from_live). CLI callers pass nothing and scan as
    # before, so file-to-file behaviour is unchanged.
    if minima is None:
        minima = scan_board_minima(output_pcb)
    clr = clearance if clearance is not None else project_copper_clearance(proj)
    targets = compute_targets(clearance=clr, hole_clearance=hole_clearance,
                              hole_to_hole=hole_to_hole, edge_clearance=edge_clearance,
                              track_width=track_width, via_diameter=via_diameter,
                              via_drill=via_drill, minima=minima,
                              fab_edge=fab_edge_floor(output_pcb),
                              copper_holes=copper_holes)
    # #498: a .kicad_dru rule may legally RELAX clearance on its layer, and
    # KiCad's rules.min_clearance is an ABSOLUTE floor that outranks custom
    # rules -- cap the recorded floor at the smallest rule value, or the ruled
    # layers re-manufacture phantom violations on copper routed at the rule.
    # RULE-ONLY, deliberately: `class_clearance` is left alone (#900). A dru
    # rule REPLACES the pair clearance on ITS layer and already outranks the
    # class, so lowering the class to a B.Cu rule's value would weaken grading
    # on every layer the rule does not cover, for no DRC benefit.
    try:
        from kicad_dru import min_rule_clearance
        _dru_min = min_rule_clearance(output_pcb)
        if _dru_min is not None and targets.get("min_clearance") \
                and _dru_min < targets["min_clearance"]:
            targets["min_clearance"] = _dru_min
    except Exception:
        pass
    # #856: severities are the project author's statement of what counts as a
    # violation. A routing step relaxes them ONLY when asked
    # (--relax-drc-severities / the GUI checkbox); the numeric floors above are
    # a different act and stay.
    if relax_severities or extra_ignore:
        plan = severity_plan(keep_courtyards=keep_courtyards, keep_mask=keep_mask,
                             keep_footprint=keep_footprint, keep_thermal=keep_thermal,
                             extra_ignore=extra_ignore)
        if not relax_severities:
            plan = {cat: "ignore" for cat in extra_ignore}
    else:
        plan = {}
    changes = apply_targets_to_project(proj, targets, plan,
                                       diff_pair_gap=diff_pair_gap,
                                       diff_pair_width=diff_pair_width,
                                       clamp_nondefault_netclasses=clamp_nondefault_netclasses)
    # Machine-readable record of what this call wrote (LAST_PROJECT_WRITES:
    # nothing publishes it yet). Replaced per call, never appended.
    LAST_PROJECT_WRITES[:] = list(changes)
    if _origin_seeded:
        # Custody: the board's ORIGINAL floors are recorded on the first
        # writeback even when nothing else moved. (#856 made a no-change run
        # common -- severities used to guarantee a write -- and the origin
        # must not depend on some other key having changed.)
        proj.setdefault("kicad_routing_tools", {})["fab_floor_origin"] = _origin
        changes = list(changes) + ["kicad_routing_tools.fab_floor_origin: recorded"]
        LAST_PROJECT_WRITES[:] = list(changes)
    # #1160: a Default class lowered by an automatic descent, not a request.
    _ccr, _ccr_here = record_class_clearance_descent(proj, _class_before)
    if _ccr_here:
        changes = list(changes) + [
            f"kicad_routing_tools.{CLASS_CLEARANCE_RELAXED_KEY}: recorded"]
        LAST_PROJECT_WRITES[:] = list(changes)
    _ccr_lines = class_clearance_disclosure(_ccr, _ccr_here, _pdcc(proj))
    if not changes:
        if verbose:
            print(f"  DRC settings already consistent ({out_pro})")
            # The floors did not move, but they may ALREADY be under the
            # board's original declaration from an earlier step -- and that is
            # still true of the board being shipped. Say so.
            for line in _fab_floor_disclosure(output_pcb, _rules_before, proj,
                                              _origin) + _ccr_lines:
                print(line)
        return out_pro
    # Atomic replace (#513 item 12): a kill mid-dump must not leave a
    # truncated/unparseable project stranding the DRC floor.
    tool_records.stamp_revision(proj)
    _tmp_pro = out_pro + ".tmp"
    with open(_tmp_pro, "w") as f:
        json.dump(proj, f, indent=2)
        f.write("\n")
    os.replace(_tmp_pro, out_pro)
    if verbose:
        print(f"  DRC settings: wrote {len(changes)} value(s) to {out_pro} "
              f"to match the routed floors (close+reopen in KiCad if it "
              f"is open). WRITES, not changes: a key the project never "
              f"declared counts here, and one logical value is written once "
              f"per net class:")
        # LIST them. Pointing at a summary that names three of seventeen is how
        # run 14's netclass via_diameter went 0.6 -> 0.25 and its
        # min_hole_clearance 0.25 -> 0.175 with nothing said about either.
        for c in changes:
            print(f"      {c}")
        # One machine-readable line per writeback (#856/#857): a harness that
        # grades the project must be able to see what the routing step changed
        # in it without grepping prose.
        print("PROJECT_WRITES_JSON: " + json.dumps(
            {"project": out_pro, "writes": list(changes)}, sort_keys=True))
    for line in _fab_floor_disclosure(output_pcb, _rules_before, proj,
                                      _origin) + _ccr_lines:
        print(line)
    return out_pro


def clamp_nondefault_netclasses_on_board(board, targets, *, diff_pair_gap=None,
                                         diff_pair_width=None, default_nc=None):
    """THE live-board NON-Default net-class clamp: lower each non-Default class's
    DRC-enforced floors to the values this run actually used. Returns a list of
    change strings (empty when nothing moved, or when the board's pcbnew shape
    is one this cannot read).

    Extracted from `apply_targets_to_board` at #782 so both GUI fronts share one
    implementation. The fanout tab is the reason: it prices its decoupling-cap
    pass at the #768 `--clearance` CEILING when the Min-Clearance override is
    ticked, but finished with `gui_utils.update_live_drc_floors`, which touches
    `m_MinClearance` and the DEFAULT class only. So it ran the pricing half of
    #768's GIVEN branch and not the writeback half -- a Wide-class pair priced at
    min(0.4, 0.2) and then graded by KiCad at the still-0.4 class.

    ``targets`` is the same dict `compute_targets` returns; only the CLASS
    clearance is read -- ``class_clearance``, falling back to ``min_clearance``
    for the hand-built ``{'min_clearance': ceiling}`` dicts `gui_utils` and the
    fanout tab pass (see :func:`_class_clearance`). It must NOT be the capped
    rule floor: a class clamped to one part's 2 mil pad override declares every
    pair in that class legal at 0.05 mm (#900). WHY ONLY CLEARANCE: parity with
    `apply_targets_to_project`'s `_NONDEFAULT_CLAMP_FIELDS` -- clearance is the
    one field KiCad enforces PER CLASS, so it is the only one whose stale value
    manufactures violations. SetTrackWidth / SetViaDiameter / SetViaDrill are
    deliberately ABSENT: they are draw defaults, and lowering them overwrote a
    board's declared per-class geometry with a local escape's stub width. Keep
    this list and the CLI one in step.

    ``default_nc`` is the caller's already-resolved Default class when it has one
    (`apply_targets_to_board` resolves it for its own half); resolved here
    otherwise, so a caller that only wants this clamp -- the fanout tab -- does
    not have to. The Default class is skipped BY IDENTITY and by name, and which
    of those does the work DEPENDS ON THE BUILD: probed on KiCad 10.0.0,
    `m_NetSettings.GetNetclasses()` returns the NON-Default classes only, so
    neither guard fires there -- the enumeration simply never offers it. They
    are for the older shape, `bds.GetNetClasses()`, which DOES include it. Stated
    as measured rather than as "the enumeration returns it too", which was my
    own first wording and is false on the build this was developed against.

    Best-effort across KiCad versions (the non-Default enumeration API varies),
    guarded so an unknown shape simply no-ops rather than raising into a step
    that has already placed its copper.
    """
    MM = 1e6  # mm -> internal nm
    EPS = 1.0  # nm
    changes = []
    try:
        bds = board.GetDesignSettings()
    except Exception:                                          # noqa: BLE001
        return changes
    if default_nc is None:
        try:
            default_nc = _default_netclass_of(bds)
        except Exception:                                      # noqa: BLE001
            default_nc = None
    # Clearance ONLY (parity with _NETCLASS_WRITABLE_FIELDS). The diff-pair
    # gap/width kwargs are accepted for signature compatibility and ignored:
    # they are draw defaults, and lowering them was the #842 ratchet.
    nd_map = {"SetClearance": _class_clearance(targets)}    # routed, not capped (#900)
    if not any(v is not None for v in nd_map.values()):
        return changes
    other = {}
    ns2 = getattr(bds, "m_NetSettings", None)
    for getter in ("GetNetclasses", "GetNetClasses"):
        src = (ns2 if ns2 is not None and hasattr(ns2, getter)
               else (bds if hasattr(bds, getter) else None))
        if src is None:
            continue
        try:
            m = getattr(src, getter)()
            if hasattr(m, "items"):
                other = dict(m.items())
            elif hasattr(m, "keys"):
                other = {k: m[k] for k in m.keys()}
            if other:
                break
        except Exception:
            pass
    for cname, nc in (other or {}).items():
        if nc is None or (default_nc is not None and nc is default_nc) \
                or cname == "Default":
            continue
        for setter, target in nd_map.items():
            if target is None or not hasattr(nc, setter):
                continue
            getter = "Get" + setter[3:]
            if hasattr(nc, getter):
                try:
                    cur = getattr(nc, getter)()
                    if cur is not None and cur <= round(float(target) * MM) + EPS:
                        continue  # only loosen
                except Exception:
                    pass
            try:
                getattr(nc, setter)(round(float(target) * MM))
                changes.append(f"net_class[{cname}].{setter} -> {target:.4g} mm")
            except Exception:
                pass
    return changes


def _default_netclass_of(bds):
    """The board's Default net class from a BOARD_DESIGN_SETTINGS, or None.

    The same probe order `apply_targets_to_board` uses inline for its own half:
    KiCad 8+ exposes it on NET_SETTINGS (`m_NetSettings.GetDefaultNetclass`),
    older builds on the settings object's net-class map.
    """
    ns = getattr(bds, "m_NetSettings", None)
    for getter in ("GetDefaultNetclass",):
        src = (ns if ns is not None and hasattr(ns, getter)
               else (bds if hasattr(bds, getter) else None))
        if src is not None:
            try:
                nc = getattr(src, getter)()
                if nc is not None:
                    return nc
            except Exception:
                pass
    if hasattr(bds, "GetNetClasses"):
        try:
            return bds.GetNetClasses().GetDefault()
        except Exception:
            pass
    return None


def apply_targets_to_board(board, targets: dict, sev_plan: dict,
                           diff_pair_gap=None, diff_pair_width=None,
                           clamp_nondefault_netclasses=False):
    """GUI path: apply the same floors + severity plan to a live pcbnew BOARD
    via BOARD_DESIGN_SETTINGS (issue #160). Best-effort and defensive -- the
    pcbnew API field/severity names vary across KiCad versions, so each step is
    guarded. Returns a list of change strings. Caller should mark the board
    modified so the user's next save persists the change.

    ``clamp_nondefault_netclasses`` (parity with apply_targets_to_project) clamps
    the NON-Default net classes' floors down to the routed values. Driven by whether
    routing used --clearance-ceiling (#439/#530: stock classes are aspirational; keeping
    them manufactures phantom sub-class DRC). Pass False (routing HONORED the classes
    -- no --clearance-ceiling / GUI Clearance ceiling box unchecked) to preserve the original
    class spec, for a genuine impedance board whose classes are met."""
    import pcbnew
    MM = 1e6  # mm -> internal nm
    EPS = 1.0  # nm
    bds = board.GetDesignSettings()
    changes = []
    # BEFORE anything is lowered: this runs first in a GUI step, ahead of
    # gui_utils.update_live_drc_floors, and a writer that lowers without seeding
    # leaves the next one to record the already-lowered value (ad7f24de).
    seed_live_fab_floor_origin(board)
    open_live_step(board)
    # The repo's own via-drill floor, which pcbnew has no attribute for: the
    # file writer writes it into the project's rules, so it is held for the
    # session here (only-lower) and counted with the declared floors.
    note_live_session_rule(board, "min_via_drill", targets.get("min_via_drill"))

    # #498 parity with fix_project_for_output: cap min_clearance at the
    # smallest .kicad_dru layer rule (an absolute board floor above a relaxing
    # rule re-manufactures phantom violations on that rule's layer). RULE-ONLY:
    # `class_clearance` is untouched here too (#900).
    try:
        from kicad_dru import min_rule_clearance
        _dru_min = min_rule_clearance(board.GetFileName() or "")
        if _dru_min is not None and targets.get("min_clearance") \
                and _dru_min < targets["min_clearance"]:
            targets = dict(targets)
            targets["min_clearance"] = _dru_min
    except Exception:
        pass

    # rule key -> BOARD_DESIGN_SETTINGS attribute (units: nm). Only loosen.
    attr = {
        "min_clearance": "m_MinClearance",
        "min_track_width": "m_TrackMinWidth",
        "min_via_diameter": "m_ViasMinSize",
        "min_through_hole_diameter": "m_MinThroughDrill",
        "min_hole_to_hole": "m_HoleToHoleMin",
        "min_copper_edge_clearance": "m_CopperEdgeClearance",
        "min_hole_clearance": "m_HoleClearance",
        # #439 parity with apply_targets_to_project (annular is ignore-severity by
        # default, so usually moot, but the CLI lowers it -- keep the two paths
        # writing the same rule set). Guarded by hasattr below for older KiCad.
        "min_via_annular_width": "m_ViasMinAnnularWidth",
        # #505 min copper web. Name varies by KiCad version; the hasattr guard
        # below skips it where absent, and the `not cur` guard keeps a disabled
        # checker disabled (parity with apply_targets_to_project).
        "min_connection": "m_MinConn",
    }
    # What the board DECLARES before this writer touches it. A rule it does not
    # declare is what the file writer sees as a MISSING key (`cur is None`), and
    # it writes the target there outright -- not only-lower against pcbnew's
    # factory stand-in. On a project-less board that is the difference between
    # copper-to-edge 0.2 (the CLI's project) and 0.5 (pcbnew's factory value).
    _declared = live_declared_rules(board) or {}
    _class_declared = live_declares_default_class(board)
    for key, target in targets.items():
        a = attr.get(key)
        if a is None or target is None or not hasattr(bds, a):
            continue
        if key == "min_connection" and not getattr(bds, a, 0):
            continue    # checker was OFF -- do not switch it on
        # The file writer writes every rule it targets, so the rule is
        # declared from here on even where only-lower leaves it unmoved.
        note_live_declared(board, (key,))
        tgt_nm = round(float(target) * MM)
        cur = getattr(bds, a)
        cur_declared = cur if key in _declared else None
        # Edge clearance may RAISE to the fab copper-to-edge floor (#441); every
        # other rule is only-lower. `target` already carries max(routed, fab_edge);
        # max with cur preserves a board rule above the fab floor.
        if key == "min_copper_edge_clearance":
            new_nm = max(cur_declared or 0, tgt_nm)
            if cur is None or abs(new_nm - cur) > EPS:
                try:
                    setattr(bds, a, new_nm)
                    changes.append(f"{a}: {(cur or 0)/MM:.4g} -> {new_nm/MM:.4g} mm (fab-edge pin)")
                except Exception:
                    pass
            continue
        if cur is None or (cur_declared is None and abs(cur - tgt_nm) > EPS) \
                or cur > tgt_nm + EPS:
            try:
                setattr(bds, a, tgt_nm)
                changes.append(f"{a}: {(cur or 0)/MM:.4g} -> {target:.4g} mm")
            except Exception:
                pass

    # Default net class CLEARANCE (the one class field KiCad's DRC enforces).
    # Track/via/drill/diff-pair class values are draw defaults and are never
    # lowered (parity with apply_targets_to_project; the #842 ratchet).
    nc_map = {"SetClearance": _class_clearance(targets)}    # routed, not capped (#900)
    default_nc = None
    for getter in ("GetDefaultNetclass",):           # KiCad 8+: NET_SETTINGS
        ns = getattr(bds, "m_NetSettings", None)
        if ns is not None and hasattr(ns, getter):
            try:
                default_nc = getattr(ns, getter)()
            except Exception:
                default_nc = None
            break
    if default_nc is None and hasattr(bds, "GetNetClasses"):  # older KiCad
        try:
            default_nc = bds.GetNetClasses().GetDefault()
        except Exception:
            default_nc = None
    if default_nc is not None:
        for setter, target in nc_map.items():
            if target is None or not hasattr(default_nc, setter):
                continue
            getter = "Get" + setter[3:]
            # A class the board does not declare (pcbnew's stock one on a
            # project-less board) is set to the routed value outright: the file
            # writer CREATES the class at it there, it does not lower a stock 0.2.
            if hasattr(default_nc, getter):
                try:
                    cur = getattr(default_nc, getter)()
                    tgt_nm = round(float(target) * MM)
                    if cur is not None and _class_declared and cur <= tgt_nm + EPS:
                        continue  # only loosen (lower); never raise
                    if cur is not None and not _class_declared \
                            and abs(cur - tgt_nm) <= EPS:
                        note_live_declared(board, (DEFAULT_CLASS_CLEARANCE_KEY,))
                        continue  # already there; now declared
                except Exception:
                    pass
            try:
                getattr(default_nc, setter)(round(float(target) * MM))
                note_live_declared(board, (DEFAULT_CLASS_CLEARANCE_KEY,))
                changes.append(f"net_class[Default].{setter} -> {target:.4g} mm")
            except Exception:
                pass

    # NON-Default net classes (#295 parity with the CLI apply_targets_to_project).
    # ONE SPELLING (#782): the body lives in
    # `clamp_nondefault_netclasses_on_board` above, because the GUI's fanout tab
    # needs exactly this clamp and reached it through neither this function nor
    # the CLI writeback -- it finishes with gui_utils.update_live_drc_floors,
    # which writes the DEFAULT class only. A second copy over there is the
    # bug class #736/#747/#775 each fixed once in the placement engine.
    if clamp_nondefault_netclasses:
        changes.extend(clamp_nondefault_netclasses_on_board(
            board, targets, diff_pair_gap=diff_pair_gap,
            diff_pair_width=diff_pair_width, default_nc=default_nc))

    # Severities. Map our category strings to pcbnew DRCE_* codes (best-effort).
    sev_const = {"ignore": getattr(pcbnew, "RPT_SEVERITY_IGNORE", 0),
                 "warning": getattr(pcbnew, "RPT_SEVERITY_WARNING", 2),
                 "error": getattr(pcbnew, "RPT_SEVERITY_ERROR", 3)}
    rank = {sev_const["ignore"]: 0, sev_const["warning"]: 1, sev_const["error"]: 2}
    code = {
        "courtyards_overlap": "DRCE_OVERLAPPING_FOOTPRINTS",
        "malformed_courtyard": "DRCE_MALFORMED_COURTYARD",
        "npth_inside_courtyard": "DRCE_NPTH_IN_COURTYARD",
        "pth_inside_courtyard": "DRCE_PTH_IN_COURTYARD",
        "solder_mask_bridge": "DRCE_SOLDERMASK_BRIDGE",
        "annular_width": "DRCE_ANNULAR_WIDTH",
        "lib_footprint_issues": "DRCE_LIBRARY_FOOTPRINT_ISSUES",
        "lib_footprint_mismatch": "DRCE_LIBRARY_FOOTPRINT_MISMATCH",
        "starved_thermal": "DRCE_STARVED_THERMAL",
    }
    _sev_api = (hasattr(bds, "SetSeverity") and hasattr(bds, "GetSeverity")
                and all(hasattr(pcbnew, c) for c in code.values()))
    if sev_plan and not _sev_api:
        # KiCad 10.0.0's SWIG has no SetSeverity and no DRCE_* / RPT_SEVERITY_*
        # names, and GetSeverity returns an opaque pointer (probed): there is
        # no way to change a severity from here. Say so -- the CLI writes the
        # relaxation into its project, and silence here read as "done". Nothing
        # is changed, so nothing is recorded under saved_severities either.
        print("  (DRC severities NOT relaxed: this pcbnew's scripting API "
              "cannot change a rule severity -- set them in Board Setup > "
              "Violation Severity if you want them relaxed)")
    if _sev_api:
        name_of = {v: k for k, v in sev_const.items()}
        # What the board DECLARES for a category: its project's
        # rule_severities, or this session's own setting. One it does not
        # declare is "error" to the file writer (`sev.get(cat, "error")`), so it
        # is here too -- for the decision and for the record below.
        try:
            _path = board.GetFileName() or ""
        except Exception:
            _path = ""
        _proj_sev = ((((_project_doc(_path).get("board") or {})
                       .get("design_settings") or {}).get("rule_severities")
                      or {}) if _path else {})
        _sess = _LIVE_DECLARED.get(live_board_key(board), set())
        saved = {}
        for cat, level in sev_plan.items():
            cname = code.get(cat)
            drce = getattr(pcbnew, cname, None) if cname else None
            if drce is None:
                continue
            tgt = sev_const[level]
            try:
                cur = bds.GetSeverity(drce)
                cur_name = (name_of.get(cur, "error")
                            if cat in _proj_sev or f"severity:{cat}" in _sess
                            else "error")
                if _SEV_RANK.get(level, 2) < _SEV_RANK.get(cur_name, 2):
                    bds.SetSeverity(drce, tgt)               # only loosen
                    note_live_declared(board, (f"severity:{cat}",))
                    saved.setdefault(cat, cur_name)
                    changes.append(f"severity[{cat}] -> {level}")
            except Exception:
                pass
        if saved:
            # #856: the value a severity had before a tool lowered it, the
            # FIRST writer's, as apply_targets_to_project records it -- into
            # the board's records (tool_records), so it survives KiCad's save.
            def _record(ns):
                rec = ns.setdefault("saved_severities", {})
                added = {c: v for c, v in saved.items() if c not in rec}
                rec.update(added)
                return bool(added)
            try:
                import tool_records
                tool_records.update_live_namespace(board, _record)
            except Exception:
                pass
    return changes


def main():
    ap = argparse.ArgumentParser(
        description="Make a routed board's KiCad DRC settings consistent with the routed floors.")
    ap.add_argument("board", help="Path to the .kicad_pcb (or .kicad_pro) file")
    # Routing parameters (match what you passed route.py); each defaults to the
    # board's own minimum / the project clearance when omitted.
    ap.add_argument("--clearance", type=float, default=None,
                    help="Copper clearance floor in mm (default: project Default net-class clearance)")
    ap.add_argument("--hole-clearance", type=float, default=None,
                    help="Hole/copper clearance floor in mm (default: copper clearance)")
    ap.add_argument("--hole-to-hole", type=float, default=None,
                    help="Hole-to-hole clearance floor in mm (routing --hole-to-hole-clearance)")
    ap.add_argument("--edge-clearance", type=float, default=None,
                    help="Copper-to-edge clearance floor in mm (routing --board-edge-clearance)")
    ap.add_argument("--track-width", type=float, default=None,
                    help="Min track width in mm (default: smallest track on the board)")
    ap.add_argument("--via-size", type=float, default=None,
                    help="Min via diameter in mm (default: smallest via on the board)")
    ap.add_argument("--via-drill", type=float, default=None,
                    help="Min hole/drill diameter in mm (default: smallest drill on the board)")
    ap.add_argument("--diff-pair-gap", type=float, default=None,
                    help="Accepted for compatibility and ignored (#842): the net class's "
                         "diff-pair gap is a draw default, never written.")
    ap.add_argument("--diff-pair-width", type=float, default=None,
                    help="Accepted for compatibility and ignored (#842): the net class's "
                         "diff-pair width is a draw default, never written.")
    ap.add_argument("--relax-severities", action="store_true",
                    help="Lower the non-routing DRC severities (courtyard shapes, solder-mask "
                         "bridges, footprint/library issues -> ignore; starved_thermal and "
                         "courtyards_overlap -> warning). OFF by default (#856); the previous "
                         "values are recorded under kicad_routing_tools.saved_severities.")
    ap.add_argument("--keep-courtyards", action="store_true", help="Do not ignore courtyard categories")
    ap.add_argument("--keep-mask", action="store_true", help="Do not ignore solder-mask bridge")
    ap.add_argument("--keep-footprint", action="store_true",
                    help="Do not ignore footprint/library categories (annular_width, lib_footprint_*)")
    # Shared DRC-fix flags (--keep-thermal, --enable-used-layers; also
    # --relax-drc-severities, which this script does not read -- its own
    # --relax-severities above is the switch); the standalone script always
    # fixes, so --no-fix-drc-settings is omitted.
    add_drc_fix_args(ap, include_no_fix=False)
    ap.add_argument("--ignore", nargs="+", default=[], metavar="CAT",
                    help="Extra severity categories to set to ignore")
    ap.add_argument("--ignore-warnings", action="store_true",
                    help="Set every category currently at 'warning' severity to 'ignore' "
                         "(hides all warning markers; errors are untouched)")
    ap.add_argument("--dry-run", action="store_true", help="Show changes without writing")
    from fab_tiers import (add_fab_tier_args, fab_tier_from_args,
                           set_default_fab_tier, fab_floor_min)
    add_fab_tier_args(ap)
    args = ap.parse_args()
    set_default_fab_tier(*fab_tier_from_args(args))

    pro = find_project(args.board)
    if not os.path.isfile(pro):
        # Seed a minimal valid project rather than refusing (mirrors
        # fix_project_for_output): a board WITHOUT a sibling .kicad_pro is the
        # worst case this tool exists for -- KiCad auto-creates one with
        # DEFAULT constraints and a fine-pitch board storms with hundreds of
        # annular/track/hole "violations" (#295, zynq_ad9364's cp'd final).
        if args.dry_run:
            sys.exit(f"error: no project file at {pro} (would seed one; re-run without --dry-run)")
        print(f"  No project file at {pro} - seeding a minimal one")
        with open(pro, "w") as f:
            json.dump({"board": {"design_settings": {"rules": {}, "rule_severities": {}}},
                       "meta": {"filename": os.path.basename(pro), "version": 1},
                       "net_settings": {"classes": [], "meta": {"version": 0}}},
                      f, indent=2)

    import tool_records
    proj = tool_records.load_project(pro)

    # Compute the floors (routing params; sizes fall back to the board minima,
    # clearance to the project's Default net-class clearance) and the severity
    # plan, then apply with only-loosen semantics via the shared logic.
    pcb_path = args.board if args.board.endswith(".kicad_pcb") else os.path.splitext(args.board)[0] + ".kicad_pcb"
    if args.enable_used_layers and not args.dry_run:
        enable_used_layers(pcb_path)
    minima = scan_board_minima(pcb_path)
    clearance = args.clearance if args.clearance is not None else project_copper_clearance(proj)
    # When a size / hole-to-hole floor isn't given explicitly, fall back to the
    # selected fab tier's deepest floor (issue #237) so the written DRC documents
    # the chosen fab capability; compute_targets still takes the SMALLER of this and
    # any tighter object already on the board.
    try:
        from list_nets import _count_copper_layers
        with open(pcb_path, encoding='utf-8') as _f:
            _ncu = _count_copper_layers(_f.read())
    except (OSError, ImportError):
        _ncu = 2
    _fab = fab_floor_min(_ncu)
    targets = compute_targets(
        clearance=clearance, hole_clearance=args.hole_clearance,
        hole_to_hole=args.hole_to_hole if args.hole_to_hole is not None else _fab['hole_to_hole'],
        edge_clearance=args.edge_clearance,
        track_width=args.track_width if args.track_width is not None else _fab['track_width'],
        via_diameter=args.via_size if args.via_size is not None else _fab['via_diameter'],
        via_drill=args.via_drill if args.via_drill is not None else _fab['via_drill'],
        minima=minima, fab_edge=fab_edge_floor(pcb_path))
    # #856: the category plan is opt-in (--relax-severities). An explicit
    # --ignore CAT / --ignore-warnings is its own request and works without it.
    if args.relax_severities:
        plan = severity_plan(keep_courtyards=args.keep_courtyards, keep_mask=args.keep_mask,
                             keep_footprint=args.keep_footprint, keep_thermal=args.keep_thermal,
                             extra_ignore=args.ignore)
    else:
        plan = {cat: "ignore" for cat in args.ignore}
    changes = apply_targets_to_project(proj, targets, plan,
                                       ignore_current_warnings=args.ignore_warnings,
                                       diff_pair_gap=args.diff_pair_gap,
                                       diff_pair_width=args.diff_pair_width,
                                       clamp_nondefault_netclasses=True)  # #439: clamp to the routed floor

    if not changes:
        print(f"{pro}: already consistent, nothing to change.")
        return

    print(f"{pro}:")
    for c in changes:
        print(f"  {c}")

    if args.dry_run:
        print("(dry run -- not written)")
        return

    tool_records.stamp_revision(proj)
    with open(pro, "w") as f:
        json.dump(proj, f, indent=2)
        f.write("\n")
    print(f"\nWrote {pro}. Constraints only loosened toward the routed floor; "
          f"shorts / unconnected are unchanged.")
    print("NOTE: if the board is open in KiCad, close it first and reopen -- "
          "KiCad overwrites the project file on save.")


if __name__ == "__main__":
    main()
