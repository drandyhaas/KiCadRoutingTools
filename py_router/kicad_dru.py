"""Per-layer clearance rules from a board's custom design rules (#498).

KiCad stores layer-scoped clearance requirements in ``<project>.kicad_dru``
(Board Setup -> Custom Rules) -- netclasses cannot express them (a net spans
layers but carries one clearance). This module reads the PARSABLE SUBSET of
those rules into a ``{layer_name: clearance_mm}`` map the router and checkers
consume with REPLACEMENT semantics: on a ruled layer the rule value replaces
the net/class-resolved pair clearance (KiCad's own precedence -- custom rules
outrank netclasses and may tighten OR relax), while unruled layers keep the
existing resolution. The map is the board's own spec, so it outranks
``--clearance`` too; only the fab tier's physical floor pins it up.

Subset honored (anything else is skipped with a note -- mis-modeling KiCad's
expression engine would silently manufacture wrong clearances):
  - ``(constraint clearance (min <val>))`` rules whose scope is pure layers:
      * a ``(layer <name>|outer|inner)`` clause, and/or
      * a ``(condition "A.onLayer('X') || B.onLayer('Y') ...")`` expression
        containing ONLY onLayer terms joined by ``||``
  - a rule with NO scope applies to every copper layer (KiCad semantics)
  - later rules override earlier ones per layer (KiCad evaluates last-to-
    first, first match wins)
  - ``(severity ignore)`` rules are skipped

THE SECOND CHANNEL, and it is not the same shape. TRACK-SCOPED rules
(``A.Type=='track' && B.Type=='track'`` plus one net-class term) cannot be a
layer map at all -- they are a predicate on a PAIR of nets -- so they parse out
of the same single pass as a ``TrackRule`` list and are **raise-only** over an
already-resolved pair value, never a replacement. Two resolvers consume them,
and the difference matters when reading a number back:

  * ``track_pair_clearance`` is PAIR-EXACT and is what a grader or a
    geometry gate that knows both nets must use (check_drc's seg-seg site,
    the fanout-clearance connector gate).
  * ``effective_track_clearances`` OVER-APPROXIMATES per obstacle net,
    because the router's obstacle map is shared across the routed set and
    has no room for a per-pair value. Router output therefore always grades
    clean, never the reverse.

A track rule binds tracks and nothing else: no pad, no via, no zone. That is
KiCad's own ``Type=='track'`` and it is why the pad channels above are
untouched by this half of the module.

HISTORICAL NOTE -- the `#549` label, and why this module no longer carries it.

Commits dated 2026-08-02 label four unrelated topics `#549 A-1`, `#549 A-2`,
`#549 B-1`, `#549 B-2`; comments added `#549 C3` and `#549 D`. That label was a
session work plan, NOT GitHub #549 -- which was CLOSED on 2026-08-01 by PR #555
("Floorplan intent graded") and is about the placement skill. Its sub-letters
collide with each other too: `A-1` marks both this parser and the
strict-fragment view, `A-2` marks both this channel going live and the planner
riding that view, `B-1` marks both the corridor seeds and the oracle summary
check. The map, for `git log`:

  fa685741  #549 A-1  -> .kicad_dru TRACK channel, parse (inert)    -> #735
  8856c0e4  #549 A-2  -> .kicad_dru TRACK channel, live             -> #735
  972131a4  #549      -> TRACK channel, octolinear smoothing        -> #735
  56dea0cf  #549 A-1  -> strict-fragment view (check_connected)
  a29d3f88  #549 A-2  -> planner rides the strict view              -> #578
  b0dfaeb2  #549 B-1  -> corridor seeds (inert)      REMOVED f93bd946 (#575)
  ab650b93  #549 B-2  -> --corridor-nets live        REMOVED f93bd946 (#575)
  b9a527fd  #549 B-1  -> end-of-run oracle summary check

The floorplan-intent, routability and skill families KEEP `#549`: those really
are GitHub #549 (adc17a39, 0095aee9, dfdfe7c8, 7bef8b90, 4af27f51, all in
PR #555). Retagging them would have removed the one accurate pointer.
"""

import os
import re
from typing import Dict, List, NamedTuple, Optional, Tuple

# value with unit suffix; KiCad writes e.g. 0.3mm / 12mil / 0.012in
_VAL = re.compile(r"^(-?[0-9.]+)\s*(mm|mil|in|um|nm)?$")
_UNIT_MM = {"mm": 1.0, "mil": 0.0254, "in": 25.4, "um": 0.001, "nm": 1e-6, None: 1.0}
_ONLAYER = re.compile(r"[AB]\s*\.\s*onLayer\s*\(\s*['\"]([^'\"]+)['\"]\s*\)")
# The TRACK-scoped clearance subset (#735): A.Type=='track' && B.Type=='track' plus
# exactly one X.NetClass=='C' term (optionally the mirror Y.NetClass!='C').
_TRACK_TYPE = re.compile(r"([AB])\s*\.\s*Type\s*==\s*['\"]track['\"]", re.IGNORECASE)
_NC_TERM = re.compile(r"([AB])\s*\.\s*NetClass\s*([!=]=)\s*['\"]([^'\"]+)['\"]")


class TrackRule(NamedTuple):
    """A track-to-track clearance rule scoped to one net class (#735).

    ``other_only`` is True for the ``A.NetClass=='C' && B.NetClass!='C'``
    shape: the requirement binds only member-vs-NON-member pairs, so a
    member's own siblings (the coupled half of a diff pair) are exempt.
    """
    name: str
    cls: str
    other_only: bool
    clearance_mm: float


def _parse_track_condition(expr: str) -> Optional[Tuple[str, bool]]:
    """(class, other_only) when ``expr`` is exactly the honored track-pair
    subset; None otherwise (the rule falls back to the skip note)."""
    types = {t.upper() for t in _TRACK_TYPE.findall(expr)}
    ncs = _NC_TERM.findall(expr)
    residue = _NC_TERM.sub("", _TRACK_TYPE.sub("", expr))
    residue = residue.replace("&&", "").strip()
    if residue or types != {'A', 'B'} or not ncs:
        return None
    eq = [(s.upper(), c) for s, op, c in ncs if op == '==']
    ne = [(s.upper(), c) for s, op, c in ncs if op == '!=']
    if len(eq) != 1 or len(ne) > 1 or len(ncs) != len(eq) + len(ne):
        return None
    side, cls = eq[0]
    if ne:
        oside, ocls = ne[0]
        if oside == side or ocls != cls:
            return None
        return cls, True
    return cls, False


def _to_mm(tok: str) -> Optional[float]:
    m = _VAL.match(tok.strip())
    if not m:
        return None
    try:
        return float(m.group(1)) * _UNIT_MM[m.group(2)]
    except (ValueError, KeyError):
        return None


def _tokenize(text: str):
    """S-expression tokens: '(', ')', atoms, quoted strings (quotes kept off)."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c in "()":
            out.append(c)
            i += 1
        elif c in " \t\r\n":
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while j < n and text[j] != '"':
                if text[j] == "\\" and j + 1 < n:
                    buf.append(text[j + 1])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            out.append("".join(buf))
            i = j + 1
        elif c == '#':  # comment to end of line
            while i < n and text[i] != "\n":
                i += 1
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            out.append(text[i:j])
            i = j
    return out


def _parse_nodes(tokens):
    """Nested lists from the token stream."""
    stack, cur = [], []
    for t in tokens:
        if t == "(":
            stack.append(cur)
            cur = []
        elif t == ")":
            done = cur
            cur = stack.pop() if stack else []
            cur.append(done)
        else:
            cur.append(t)
    return cur


def _rule_layers(rule, copper_layers, notes) -> Optional[List[str]]:
    """Resolve a rule's layer scope to concrete copper layers, or None when the
    scope uses constructs outside the honored subset (rule must be skipped)."""
    outer = [l for l in copper_layers if l in ("F.Cu", "B.Cu")]
    inner = [l for l in copper_layers if l not in ("F.Cu", "B.Cu")]
    clause_set = cond_set = None
    for item in rule:
        if not isinstance(item, list) or not item:
            continue
        if item[0] == "layer" and len(item) >= 2:
            arg = item[1]
            if arg == "outer":
                clause_set = list(outer)
            elif arg == "inner":
                clause_set = list(inner)
            elif arg in copper_layers:
                clause_set = [arg]
            elif isinstance(arg, str) and arg.endswith(".Cu"):
                clause_set = []  # a copper layer this board doesn't have
            else:
                return None  # non-copper / wildcard layer clause: out of subset
        elif item[0] == "condition" and len(item) >= 2:
            expr = item[1]
            # Only onLayer terms joined by ||: strip every onLayer term and the
            # join operators; anything left means the expression carries other
            # predicates (netclass, area, ...) we must not guess at.
            found = _ONLAYER.findall(expr)
            residue = _ONLAYER.sub("", expr).replace("||", "").strip()
            if residue or not found:
                return None
            cond_set = [l for l in found if l in copper_layers]
    if clause_set is not None and cond_set is not None:
        return [l for l in clause_set if l in cond_set]
    if clause_set is not None:
        return clause_set
    if cond_set is not None:
        return cond_set
    return list(copper_layers)  # unscoped rule: applies everywhere


def _parse_dru(text: str, copper_layers: List[str]
               ) -> Tuple[Dict[str, float], List[TrackRule], List[str]]:
    """Single-pass parse of .kicad_dru text ->
    ({layer: clearance_mm}, [TrackRule], notes).

    Later rules override earlier ones per layer. A rule whose condition is the
    honored track-pair subset becomes a TrackRule (and never a layer entry); any
    other out-of-subset scope is skipped with a note.
    """
    result: Dict[str, float] = {}
    track_rules: List[TrackRule] = []
    notes: List[str] = []
    for node in _parse_nodes(_tokenize(text)):
        if not isinstance(node, list) or not node or node[0] != "rule":
            continue
        name = node[1] if len(node) > 1 and not isinstance(node[1], list) else "?"
        clearance_mm = None
        severity_ignore = False
        condition = None
        has_layer_clause = False
        for item in node:
            if not isinstance(item, list) or not item:
                continue
            if item[0] == "severity" and len(item) >= 2 and item[1] == "ignore":
                severity_ignore = True
            if item[0] == "condition" and len(item) >= 2:
                condition = item[1]
            if item[0] == "layer":
                has_layer_clause = True
            if item[0] == "constraint" and len(item) >= 2 and item[1] == "clearance":
                for sub in item[2:]:
                    if isinstance(sub, list) and sub and sub[0] == "min" and len(sub) >= 2:
                        clearance_mm = _to_mm(sub[1])
        if clearance_mm is None:
            continue  # not a clearance rule (or no min): not ours to model
        if severity_ignore:
            notes.append(f"rule '{name}': severity ignore, skipped")
            continue
        track = (_parse_track_condition(condition)
                 if condition and not has_layer_clause else None)
        if track is not None:
            cls, other_only = track
            track_rules.append(TrackRule(str(name), cls, other_only, clearance_mm))
            notes.append(f"rule '{name}': track-to-track clearance "
                         f"{clearance_mm}mm for class '{cls}'"
                         f"{' vs other classes only' if other_only else ''}"
                         f" -- handled by the track channel")
            continue
        layers = _rule_layers(node, copper_layers, notes)
        if layers is None:
            notes.append(f"rule '{name}': clearance {clearance_mm}mm has a "
                         f"non-layer scope (netclass/area/... condition) -- "
                         f"skipped, KiCad will still enforce it")
            continue
        for l in layers:
            result[l] = clearance_mm
    return result, track_rules, notes


def parse_dru_layer_clearances(text: str, copper_layers: List[str]
                               ) -> Tuple[Dict[str, float], List[str]]:
    """Parse .kicad_dru text -> ({layer: clearance_mm}, notes).

    Later rules override earlier ones per layer. Notes describe every skipped
    clearance rule (out-of-subset scope) so the caller can print them.
    """
    result, _track, notes = _parse_dru(text, copper_layers)
    return result, notes


def parse_dru_track_clearances(text: str) -> Tuple[List[TrackRule], List[str]]:
    """Parse .kicad_dru text -> ([TrackRule], notes) for the track
    channel. Copper layers are irrelevant to track rules; a synthetic list
    keeps the shared single-pass parser happy."""
    copper = ["F.Cu", "B.Cu"] + [f"In{i}.Cu" for i in range(1, 31)]
    _layers, track_rules, notes = _parse_dru(text, copper)
    return track_rules, notes


def read_board_layer_clearances(board_path: str, copper_layers: List[str],
                                fab_clearance_floor: Optional[float] = None,
                                ) -> Tuple[Dict[str, float], List[str]]:
    """Auto-read the sibling ``<board>.kicad_dru`` layer-clearance map.

    Returns ({} , []) when there is no dru file -- the feature is a strict
    no-op for boards without custom rules. ``fab_clearance_floor`` pins each
    value UP to the fab tier's physical minimum (the one thing that outranks
    the board's own rules), with a note.
    """
    dru = os.path.splitext(board_path)[0] + ".kicad_dru" if board_path else ""
    if not dru or not os.path.isfile(dru):
        return {}, []
    try:
        text = open(dru, encoding="utf-8").read()
    except OSError as e:
        return {}, [f"could not read {dru}: {e}"]
    result, notes = parse_dru_layer_clearances(text, copper_layers)
    if fab_clearance_floor:
        for l, v in list(result.items()):
            if v < fab_clearance_floor:
                notes.append(f"layer {l}: rule clearance {v}mm below the fab "
                             f"floor {fab_clearance_floor}mm -- pinned up")
                result[l] = fab_clearance_floor
    return result, notes


def read_board_track_clearances(board_path: str) -> Tuple[List[TrackRule], List[str]]:
    """Auto-read the sibling ``<board>.kicad_dru`` track rules (#735).

    Returns ([], []) when there is no dru file. Unlike the layer map there is
    deliberately NO fab pinning: the track channel is raise-only over the
    already-resolved pair clearance, so a rule below the resolved value is
    simply inert -- it can never drag copper under the fab floor."""
    dru = os.path.splitext(board_path)[0] + ".kicad_dru" if board_path else ""
    if not dru or not os.path.isfile(dru):
        return [], []
    try:
        text = open(dru, encoding="utf-8").read()
    except OSError as e:
        return [], [f"could not read {dru}: {e}"]
    return parse_dru_track_clearances(text)


def track_pair_clearance(track_rules: List[TrackRule], a_cls, b_cls,
                         resolved: float) -> Tuple[float, Optional[TrackRule]]:
    """THE pair-exact track-rule resolver: (effective mm, the rule that RAISED
    it or None).

    ``resolved`` is the caller's already-resolved pair value -- class pairwise
    max with the #498 layer replacement applied -- and this is **raise-only**
    over it, so a rule at or below it is inert and the fab floor can never be
    dragged down. ``a_cls`` / ``b_cls`` are the two nets' class-membership sets
    from ``list_nets.net_class_memberships``.

    One site, deliberately. ``check_drc`` graded the seg-seg pair through a
    closure of this body and ``fanout_clearance``'s connector gate had no
    equivalent at all (#735), which is how the cap-repair pass could draw
    copper closer than the grader accepts. Both now call this.

    The rule identity comes back because check_drc's violation record uses it
    to tell a structural, rule-governed pair from a physical graze; a caller
    that only needs the number takes ``[0]``.
    """
    eff, rule = resolved, None
    if not track_rules:
        return eff, rule
    for r in track_rules:
        a_in, b_in = r.cls in a_cls, r.cls in b_cls
        binds = ((a_in != b_in) or (a_in and b_in and not r.other_only))
        if binds and r.clearance_mm > eff:
            eff = r.clearance_mm
            rule = r
    return eff, rule


def board_track_rules(pcb_data, board_path: str = None):
    """([TrackRule], {net_id: frozenset of class names}) for a PCBData.

    The QUIET, pcb_data-shaped reader -- the track-channel twin of
    ``board_layer_clearance_map``, for engines that resolve pairs outside a
    GridRouteConfig (the placement passes). Path discovery is the #498 rule:
    the caller's ``board_path`` when it has one, else ``PCBData.source_path``.

    ``([], {})`` for no path, no sibling .kicad_dru, no rule, and for a
    membership resolution that comes back EMPTY -- a rule whose class has no
    members can never bind a pair, and returning it would leave a caller
    paying for a live channel that is arithmetically dead. That last case
    covers the ordinary "project declares no netclass_patterns" board, which
    `net_class_memberships` answers with ``{}`` rather than an exception.

    It does not raise for any input `net_class_memberships` can produce; the
    two expressions outside the guards are the ``source_path`` read and the
    final comprehension, and neither is reachable through that resolver. The
    point is that a board declaring nothing must cost its caller nothing, and
    a failed read must not become a crash in a pass that worked before the
    rules file appeared.
    """
    path = board_path or getattr(pcb_data, 'source_path', "") or ""
    if not path:
        return [], {}
    try:
        rules, _notes = read_board_track_clearances(path)
    except Exception:                                       # noqa: BLE001
        return [], {}
    if not rules:
        return [], {}
    try:
        from list_nets import net_class_memberships
        nets = {nid: n.name for nid, n in (pcb_data.nets or {}).items()
                if getattr(n, 'name', None)}
        raw = net_class_memberships(path, nets)
    except Exception:                                       # noqa: BLE001
        # The rules parsed but nobody can be said to be IN a class, so no pair
        # can bind. Drop the rules with them rather than keep a channel that
        # would silently grade every pair as a non-member. This mirrors
        # check_drc, which clears its own rule list on the same failure.
        return [], {}
    if not raw:
        return [], {}
    return rules, {nid: frozenset(cls) for nid, cls in raw.items()}


def effective_track_clearances(track_rules: List[TrackRule],
                               memberships: Dict[int, set],
                               all_net_ids, routed_net_ids
                               ) -> Dict[int, float]:
    """The per-obstacle-net {net_id: mm} map for THIS call's routed set.

    The obstacle map is SHARED across the routed nets, so a per-pair rule must
    be over-approximated per obstacle net: eff[nid] is the max clearance any
    ROUTED net requires against nid under the rules. For a rule on class C:

      * nid in C: applies when the routed set has NON-members (the pair
        member-vs-nonmember binds), or -- unless ``other_only`` -- when it has
        members (C-vs-C pairs bind too).
      * nid not in C: applies when the routed set has members of C.

    An XTAL-only run therefore does NOT inflate XTAL-vs-XTAL sibling stamps
    under an ``other_only`` rule -- the route-the-pair-tight-first workflow
    survives. Monolithic runs mixing classes over-block non-member pairs, the
    same over-approximation shape PR392 accepted; route a ruled class in its
    own invocation for exact pricing."""
    if not track_rules:
        return {}
    routed = set(routed_net_ids or ())
    eff: Dict[int, float] = {}
    for rule in track_rules:
        members = {nid for nid, cls in memberships.items() if rule.cls in cls}
        routed_has_member = bool(routed & members)
        routed_has_nonmember = bool(routed - members)
        for nid in all_net_ids:
            in_c = nid in members
            applies = ((in_c and (routed_has_nonmember
                                  or (not rule.other_only and routed_has_member)))
                       or (not in_c and routed_has_member))
            if applies and rule.clearance_mm > eff.get(nid, 0.0):
                eff[nid] = rule.clearance_mm
    return eff


_TRACK_ANNOUNCED = set()  # (board path, rules) already printed this process


def install_track_clearances(config, track_clearances, input_file,
                             pcb_data=None, routed_net_ids=None):
    """Resolve and install the track-to-track clearance map on ``config``,
    engine-side so BOTH fronts inherit it (like install_layer_clearances: no
    flag, no GUI control -- the .kicad_dru is the single source of truth). An
    explicit {net_id: mm} dict (tests) wins and stops the auto-read; None ->
    read the sibling .kicad_dru, resolve class memberships through
    list_nets.net_class_memberships (the SAME resolver the netclass clearance
    map uses), and compute the effective per-obstacle map for this call's
    routed set. Announces what is honored, once per (board, rules)."""
    if track_clearances is not None:
        config.track_clearances = dict(track_clearances)
        return
    if not input_file:
        input_file = getattr(pcb_data, 'source_path', "") or ""
    if not input_file or pcb_data is None:
        config.track_clearances = {}
        return
    rules, notes = read_board_track_clearances(input_file)
    if not rules:
        config.track_clearances = {}
        return
    from list_nets import net_class_memberships
    nets = {nid: n.name for nid, n in pcb_data.nets.items() if n.name}
    memberships = net_class_memberships(input_file, nets)
    eff = effective_track_clearances(rules, memberships, nets.keys(),
                                     routed_net_ids or nets.keys())
    config.track_clearances = eff
    _key = (os.path.abspath(input_file),
            tuple(sorted((r.name, r.cls, r.other_only, r.clearance_mm)
                         for r in rules)))
    if _key not in _TRACK_ANNOUNCED:
        _TRACK_ANNOUNCED.add(_key)
        for r in rules:
            print(f"Track-to-track clearance rule from the board's .kicad_dru "
                  f"(raise-only on seg-seg pairs): '{r.name}' class "
                  f"'{r.cls}' {r.clearance_mm:g}mm"
                  f"{' (vs other classes only)' if r.other_only else ''}"
                  f" -- {len(eff)} net(s) priced")


_ANNOUNCED = set()  # (board path, map items) already printed this process


def board_layer_clearance_map(pcb_data) -> Dict[str, float]:
    """Fab-pinned #498 layer-clearance map for a PCBData, discovered via its
    ``source_path`` sibling .kicad_dru. Quiet ({} when there is none) -- for
    engines that consume the map outside a GridRouteConfig (fanout scalars)."""
    path = getattr(pcb_data, 'source_path', "") or ""
    if not path:
        return {}
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    if not copper:
        return {}
    try:
        from fab_tiers import fab_floors
        floor = fab_floors(len(copper)).get('clearance')
    except Exception:
        floor = None
    lmap, _ = read_board_layer_clearances(path, copper, fab_clearance_floor=floor)
    return lmap


def min_rule_clearance(board_path: str) -> Optional[float]:
    """Smallest honored .kicad_dru layer-rule clearance for ``board_path``'s
    project, or None (no dru / no layer clearance rules). The DRC-settings
    writeback caps ``rules.min_clearance`` at this: KiCad's board minimum is an
    ABSOLUTE floor that outranks custom rules, so leaving it above a RELAXING
    rule re-manufactures phantom violations on copper legally routed at the
    rule value. Uses a generous synthetic copper list -- for the minimum it
    only matters that every plausibly-referenced layer resolves."""
    copper = ["F.Cu", "B.Cu"] + [f"In{i}.Cu" for i in range(1, 31)]
    lmap, _ = read_board_layer_clearances(board_path, copper)
    return min(lmap.values()) if lmap else None


def install_layer_clearances(config, layer_clearances, input_file, pcb_data=None):
    """Resolve and install the #498 per-layer map on ``config``, engine-side so
    BOTH fronts inherit it (the CLI passes nothing; the GUI passes nothing --
    there is deliberately no flag or control, the .kicad_dru is the single
    source of truth). An explicit dict (tests) wins and stops the auto-read,
    mirroring the net_clearances pattern; None -> read the sibling .kicad_dru
    of ``input_file``, expanded over the BOARD's full copper list (rules may
    target layers outside the routed subset -- via keep-outs still need them)
    and pinned up to the fab tier's clearance floor. Prints what is honored."""
    if layer_clearances is not None:
        config.layer_clearances = dict(layer_clearances)
        return
    if not input_file:
        # Engines whose signatures carry no input path (planes, fanout, oracle
        # sub-configs) discover the board file via PCBData.source_path.
        input_file = getattr(pcb_data, 'source_path', "") or ""
    copper = None
    if pcb_data is not None and getattr(pcb_data, 'board_info', None) is not None:
        copper = list(pcb_data.board_info.copper_layers or [])
    if not copper:
        copper = list(config.layers)
    try:
        from fab_tiers import fab_floors
        floor = fab_floors(len(copper)).get('clearance')
    except Exception:
        floor = None
    lmap, notes = read_board_layer_clearances(input_file or "", copper,
                                              fab_clearance_floor=floor)
    # One announcement per (board, map) per process -- plane/fanout runs build
    # several configs for the same board and would repeat it.
    _key = (os.path.abspath(input_file) if input_file else "",
            tuple(sorted(lmap.items())))
    if _key not in _ANNOUNCED:
        _ANNOUNCED.add(_key)
        for note in notes:
            print(f"  .kicad_dru: {note}")
        if lmap:
            vals = ", ".join(f"{l}:{v:g}" for l, v in sorted(lmap.items()))
            print(f"Per-layer clearance rules from the board's .kicad_dru (#498, "
                  f"replace net/class values on their layers): {vals}")
    config.layer_clearances = lmap
