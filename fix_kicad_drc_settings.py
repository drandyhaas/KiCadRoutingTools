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

  * copper **clearance** (``min_clearance`` + Default net-class clearance)
  * **hole-to-hole** clearance (``min_hole_to_hole``)
  * **hole/copper** clearance (``min_hole_clearance``)
  * **copper-to-edge** clearance (``min_copper_edge_clearance``)
  * **min track width / via diameter / via drill / annular ring** -- lowered to
    the smallest such object actually placed on the board
  * Default net-class **differential-pair gap / width** (``--diff-pair-gap`` /
    ``--diff-pair-width``) -- lowered to the routed values so the net class stops
    advertising the stock-wide 0.25 mm gap a planner would read back and re-use
  * non-routing severities (courtyard, solder-mask, footprint/library) -> ignore

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
    refs = set(re.findall(r'\(layer\s+"([^"]+)"', outside))
    for grp in re.findall(r'\(layers\s+((?:"[^"]+"\s*)+)\)', outside):
        refs.update(re.findall(r'"([^"]+)"', grp))

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
    widths = [s.width for s in pcb.segments if s.width and s.width > 0]
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
    return out


# --- Shared logic (front-end-agnostic) ------------------------------------
# Both the CLI file-edit path and the GUI pcbnew-API path call these, so the two
# front-ends compute the same DRC floors and differ only in how they apply them
# (issue #160 CLI/GUI-parity rule).

def compute_targets(clearance=None, hole_clearance=None, hole_to_hole=None,
                    edge_clearance=None, track_width=None, via_diameter=None,
                    via_drill=None, minima=None):
    """Map KiCad rule keys -> target floor (mm) from the routing parameters.
    Each value, when given, becomes a floor; sizes fall back to the board's
    smallest such object (``minima`` from :func:`scan_board_minima`) when the
    param is None. Keys absent from the result => leave that rule alone."""
    minima = minima or {}
    targets = {}
    if clearance is not None:
        targets["min_clearance"] = clearance
    # Hole/copper clearance: explicit value, else the copper-clearance floor.
    hole_clr = hole_clearance if hole_clearance is not None else clearance
    if hole_clr is not None:
        targets["min_hole_clearance"] = hole_clr
    if hole_to_hole is not None:
        targets["min_hole_to_hole"] = hole_to_hole
    if edge_clearance is not None:
        targets["min_copper_edge_clearance"] = edge_clearance

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
    vd = _floor(via_diameter, minima.get("min_via_diameter"))
    if vd is not None:
        targets["min_via_diameter"] = vd
    dr = _floor(via_drill, minima.get("min_through_hole_diameter"))
    if dr is not None:
        targets["min_through_hole_diameter"] = dr
    if "min_via_annular_width" in minima:
        targets["min_via_annular_width"] = minima["min_via_annular_width"]
    return targets


def severity_plan(keep_courtyards=False, keep_mask=False, keep_footprint=False,
                  keep_thermal=False, extra_ignore=()):
    """Desired severity per DRC category: {category -> 'ignore' | 'warning'}.
    Applied with only-loosen semantics by the apply_* functions."""
    plan = {}
    for cat in extra_ignore:
        plan[cat] = "ignore"
    if not keep_courtyards:
        for cat in COURTYARD_CATS:
            plan[cat] = "ignore"
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
                             diff_pair_gap=None, diff_pair_width=None):
    """Apply the floors + severity plan to a parsed ``.kicad_pro`` dict, only
    ever loosening (lowering a constraint / lowering a severity rank), never
    tightening. Returns a list of human-readable change strings.

    ``diff_pair_gap`` / ``diff_pair_width`` (mm), when given, lower the Default
    net class's differential-pair geometry to the values the board was actually
    routed to (issue: a stock 0.25 mm net-class gap is far wider than the
    fab-floor ~0.1 mm coupled pairs route_diff places, and a planner reading the
    net class back would recommend the wide value). Neither is a DRC-enforced
    minimum -- they are draw defaults -- so lowering them cannot create a new
    violation, consistent with the only-loosen guarantee."""
    EPS = 1e-9
    ds = proj.setdefault("board", {}).setdefault("design_settings", {})
    rules = ds.setdefault("rules", {})
    sev = ds.setdefault("rule_severities", {})
    changes = []

    for key, target in targets.items():
        if target is None:
            continue
        target = round(float(target), 6)
        cur = rules.get(key)
        if cur is None or cur > target + EPS:        # lower only; never raise
            changes.append(f"rules.{key}: {cur} -> {target} mm")
            rules[key] = target

    # KiCad enforces copper clearance PER NET CLASS (rules.min_clearance alone
    # does not relax it), so keep the Default class at the floor too -- creating
    # a COMPLETE one if the project has none (a sparse class is ignored by KiCad,
    # which then falls back to the stock 0.2 mm default).
    nc_map = {"clearance": targets.get("min_clearance"),
              "track_width": targets.get("min_track_width"),
              "via_diameter": targets.get("min_via_diameter"),
              "via_drill": targets.get("min_through_hole_diameter"),
              "diff_pair_gap": diff_pair_gap,
              "diff_pair_via_gap": diff_pair_gap,
              "diff_pair_width": diff_pair_width}
    net_settings = proj.setdefault("net_settings", {})
    net_settings.setdefault("meta", {"version": 0})  # KiCad needs this to read classes
    classes = net_settings.setdefault("classes", [])
    default_cls = next((c for c in classes if c.get("name") == "Default"), None)
    if default_cls is None and any(v is not None for v in nc_map.values()):
        default_cls = dict(_DEFAULT_NETCLASS)
        classes.insert(0, default_cls)
        changes.append("net_class[Default]: created (project had none)")
    if default_cls is not None:
        for field, target in nc_map.items():
            if target is None:
                continue
            target = round(float(target), 6)
            cur = default_cls.get(field)
            if cur is None or cur > target + EPS:
                changes.append(f"net_class[Default].{field}: {cur} -> {target} mm")
                default_cls[field] = target
    # NON-Default classes too (#295 follow-up): the original design's classes
    # (e.g. kuchen's HDMI/USB at 0.125mm) survive into the routed board's
    # project, and KiCad enforces THEIR clearance on their nets -- copper we
    # legitimately routed at the (smaller) run clearance then storms with
    # netclass-clearance violations the moment the user opens the board. The
    # router does not read per-class clearances from the project, so after
    # routing the class floors must be clamped down to what was actually
    # routed (only-lower, same guarantee as everything else here). Via sizes /
    # diff-pair geometry are draw defaults, clamped for planner consistency.
    for cls in classes:
        if cls is default_cls or not isinstance(cls, dict):
            continue
        cname = cls.get("name", "?")
        for field, target in nc_map.items():
            if target is None:
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
    ``route_disconnected_planes.py``). Wiring a new shared DRC-fix flag in here
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
    g.add_argument("--keep-thermal", action="store_true",
                   help="When fixing DRC settings, leave thermal-relief severity (starved_thermal) "
                        "untouched instead of demoting it to a warning.")
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
    return dict(keep_thermal=args.keep_thermal, enable_layers=args.enable_used_layers)


def fix_project_for_output(output_pcb: str, input_pcb=None, *, clearance=None,
                           hole_clearance=None, hole_to_hole=None, edge_clearance=None,
                           track_width=None, via_diameter=None, via_drill=None,
                           diff_pair_gap=None, diff_pair_width=None,
                           keep_courtyards=False, keep_mask=False, keep_footprint=False,
                           keep_thermal=False, enable_layers=False,
                           extra_ignore=(), verbose=True):
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

    with open(out_pro) as f:
        proj = json.load(f)
    minima = scan_board_minima(output_pcb)
    clr = clearance if clearance is not None else project_copper_clearance(proj)
    targets = compute_targets(clearance=clr, hole_clearance=hole_clearance,
                              hole_to_hole=hole_to_hole, edge_clearance=edge_clearance,
                              track_width=track_width, via_diameter=via_diameter,
                              via_drill=via_drill, minima=minima)
    plan = severity_plan(keep_courtyards=keep_courtyards, keep_mask=keep_mask,
                         keep_footprint=keep_footprint, keep_thermal=keep_thermal,
                         extra_ignore=extra_ignore)
    changes = apply_targets_to_project(proj, targets, plan,
                                       diff_pair_gap=diff_pair_gap,
                                       diff_pair_width=diff_pair_width)
    if not changes:
        if verbose:
            print(f"  DRC settings already consistent ({out_pro})")
        return out_pro
    with open(out_pro, "w") as f:
        json.dump(proj, f, indent=2)
        f.write("\n")
    if verbose:
        print(f"  DRC settings: updated {len(changes)} value(s) in {out_pro} "
              f"to match the routed floors (close+reopen in KiCad if it is open)")
    return out_pro


def apply_targets_to_board(board, targets: dict, sev_plan: dict,
                           diff_pair_gap=None, diff_pair_width=None):
    """GUI path: apply the same floors + severity plan to a live pcbnew BOARD
    via BOARD_DESIGN_SETTINGS (issue #160). Best-effort and defensive -- the
    pcbnew API field/severity names vary across KiCad versions, so each step is
    guarded. Returns a list of change strings. Caller should mark the board
    modified so the user's next save persists the change."""
    import pcbnew
    MM = 1e6  # mm -> internal nm
    EPS = 1.0  # nm
    bds = board.GetDesignSettings()
    changes = []

    # rule key -> BOARD_DESIGN_SETTINGS attribute (units: nm). Only loosen.
    attr = {
        "min_clearance": "m_MinClearance",
        "min_track_width": "m_TrackMinWidth",
        "min_via_diameter": "m_ViasMinSize",
        "min_through_hole_diameter": "m_MinThroughDrill",
        "min_hole_to_hole": "m_HoleToHoleMin",
        "min_copper_edge_clearance": "m_CopperEdgeClearance",
        "min_hole_clearance": "m_HoleClearance",
    }
    for key, target in targets.items():
        a = attr.get(key)
        if a is None or target is None or not hasattr(bds, a):
            continue
        tgt_nm = round(float(target) * MM)
        cur = getattr(bds, a)
        if cur is None or cur > tgt_nm + EPS:
            try:
                setattr(bds, a, tgt_nm)
                changes.append(f"{a}: {cur/MM:.4g} -> {target:.4g} mm")
            except Exception:
                pass

    # Default net class clearance/track/via/drill (governs the clearance check).
    nc_map = {"SetClearance": targets.get("min_clearance"),
              "SetTrackWidth": targets.get("min_track_width"),
              "SetViaDiameter": targets.get("min_via_diameter"),
              "SetViaDrill": targets.get("min_through_hole_diameter")}
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
    # Differential-pair geometry (gap/width) -- draw defaults, not DRC floors;
    # lowered only, same as the CLI path. Best-effort across KiCad versions.
    nc_map.update({"SetDiffPairGap": diff_pair_gap,
                   "SetDiffPairViaGap": diff_pair_gap,
                   "SetDiffPairWidth": diff_pair_width})
    if default_nc is not None:
        for setter, target in nc_map.items():
            if target is None or not hasattr(default_nc, setter):
                continue
            getter = "Get" + setter[3:]
            if hasattr(default_nc, getter):
                try:
                    cur = getattr(default_nc, getter)()
                    if cur is not None and cur <= round(float(target) * MM) + EPS:
                        continue  # only loosen (lower); never raise
                except Exception:
                    pass
            try:
                getattr(default_nc, setter)(round(float(target) * MM))
                changes.append(f"net_class[Default].{setter} -> {target:.4g} mm")
            except Exception:
                pass

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
    if hasattr(bds, "SetSeverity") and hasattr(bds, "GetSeverity"):
        for cat, level in sev_plan.items():
            cname = code.get(cat)
            drce = getattr(pcbnew, cname, None) if cname else None
            if drce is None:
                continue
            tgt = sev_const[level]
            try:
                cur = bds.GetSeverity(drce)
                if rank.get(tgt, 2) < rank.get(cur, 2):   # only loosen
                    bds.SetSeverity(drce, tgt)
                    changes.append(f"severity[{cat}] -> {level}")
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
                    help="Default net-class differential-pair gap in mm (match route_diff "
                         "--diff-pair-gap; lowered only). Use the fab floor (~0.1) for "
                         "impedance-controlled pairs so the net class stops recommending the "
                         "stock-wide 0.25 mm gap.")
    ap.add_argument("--diff-pair-width", type=float, default=None,
                    help="Default net-class differential-pair trace width in mm (the diff-pair "
                         "track width; lowered only).")
    ap.add_argument("--keep-courtyards", action="store_true", help="Do not ignore courtyard categories")
    ap.add_argument("--keep-mask", action="store_true", help="Do not ignore solder-mask bridge")
    ap.add_argument("--keep-footprint", action="store_true",
                    help="Do not ignore footprint/library categories (annular_width, lib_footprint_*)")
    # Shared DRC-fix flags (--keep-thermal, --enable-used-layers); the standalone
    # script always fixes, so --no-fix-drc-settings is omitted.
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

    with open(pro) as f:
        proj = json.load(f)

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
        minima=minima)
    plan = severity_plan(keep_courtyards=args.keep_courtyards, keep_mask=args.keep_mask,
                         keep_footprint=args.keep_footprint, keep_thermal=args.keep_thermal,
                         extra_ignore=args.ignore)
    changes = apply_targets_to_project(proj, targets, plan,
                                       ignore_current_warnings=args.ignore_warnings,
                                       diff_pair_gap=args.diff_pair_gap,
                                       diff_pair_width=args.diff_pair_width)

    if not changes:
        print(f"{pro}: already consistent, nothing to change.")
        return

    print(f"{pro}:")
    for c in changes:
        print(f"  {c}")

    if args.dry_run:
        print("(dry run -- not written)")
        return

    with open(pro, "w") as f:
        json.dump(proj, f, indent=2)
        f.write("\n")
    print(f"\nWrote {pro}. Constraints only loosened toward the routed floor; "
          f"shorts / unconnected are unchanged.")
    print("NOTE: if the board is open in KiCad, close it first and reopen -- "
          "KiCad overwrites the project file on save.")


if __name__ == "__main__":
    main()
