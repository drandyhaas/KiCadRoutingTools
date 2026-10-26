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
  - later rules override earlier ones per layer (KiCad's engine walks the
    rules forward in file order; the last matching rule wins each field)
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
# KiCad's "keep pairs apart" term (#1295): the rule binds every pair except a
# diff pair's P against its own N.
_NOT_COUPLED = re.compile(r"!\s*AB\s*\.\s*isCoupledDiffPair\s*\(\s*\)")


class TrackRule(NamedTuple):
    """A track-to-track clearance rule scoped to one net class (#735).

    ``other_only`` is True for the ``A.NetClass=='C' && B.NetClass!='C'``
    shape: the requirement binds only member-vs-NON-member pairs, so a
    member's own siblings (the coupled half of a diff pair) are exempt.

    ``except_coupled`` is True when the condition also ANDs in
    ``!AB.isCoupledDiffPair()`` (#1295): a pair whose two nets KiCad's
    MatchDpSuffix couples (``nets_coupled``) is exempt. ``layer_clause`` is
    the rule's ``(layer ...)`` token (a layer name, ``outer``, ``inner``), or
    None for every layer.
    """
    name: str
    cls: str
    other_only: bool
    clearance_mm: float
    except_coupled: bool = False
    layer_clause: Optional[str] = None


def _parse_track_condition(expr: str) -> Optional[Tuple[str, bool, bool]]:
    """(class, other_only, except_coupled) when ``expr`` is exactly the
    honored track-pair subset; None otherwise (the rule falls back to the
    skip note)."""
    except_coupled = len(_NOT_COUPLED.findall(expr))
    if except_coupled > 1:
        return None
    expr = _NOT_COUPLED.sub("", expr)
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
        return cls, True, bool(except_coupled)
    return cls, False, bool(except_coupled)


def track_rule_on_layer(rule: TrackRule, layer: Optional[str]) -> bool:
    """Whether ``rule`` binds on ``layer``. A rule with no layer clause binds
    on every layer; ``layer`` None (the caller cannot say) binds too, which
    over-requires rather than under-requires."""
    lc = rule.layer_clause
    if lc is None or layer is None or lc in ('*.Cu', '*'):
        return True
    if lc == 'outer':
        return layer in ('F.Cu', 'B.Cu')
    if lc == 'inner':
        return layer.endswith('.Cu') and layer not in ('F.Cu', 'B.Cu')
    return layer == lc


def nets_coupled(name_a: Optional[str], name_b: Optional[str]) -> bool:
    """KiCad's AB.isCoupledDiffPair() for two tracks: B's net is the
    MatchDpSuffix complement of A's (#1293/#1295). Symmetric, and decided on
    the NAMES: a pair our detector couples under another convention (DP/DM)
    is not coupled to KiCad, so a "keep pairs apart" rule binds it."""
    if not name_a or not name_b:
        return False
    from design_rules import match_dp_suffix
    rv, comp, _base = match_dp_suffix(name_a)
    return bool(rv) and comp == name_b


def coupled_net_pairs(net_names: Dict[int, str]) -> List[Tuple[int, int]]:
    """Every (P, N) pair of net ids KiCad's MatchDpSuffix couples, each once
    (P is the positive side)."""
    by_name = {nm: nid for nid, nm in net_names.items() if nm}
    out = []
    from design_rules import match_dp_suffix
    for nid, nm in net_names.items():
        if not nm:
            continue
        rv, comp, _base = match_dp_suffix(nm)
        if rv > 0 and comp in by_name:
            out.append((nid, by_name[comp]))
    return out


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
        layer_clause = None
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
                layer_clause = item[1] if len(item) >= 2 else None
            if item[0] == "constraint" and len(item) >= 2 and item[1] == "clearance":
                for sub in item[2:]:
                    if isinstance(sub, list) and sub and sub[0] == "min" and len(sub) >= 2:
                        clearance_mm = _to_mm(sub[1])
        if clearance_mm is None:
            continue  # not a clearance rule (or no min): not ours to model
        if severity_ignore:
            notes.append(f"rule '{name}': severity ignore, skipped")
            continue
        # A track rule may carry a copper (layer ...) clause (#1295); any other
        # clause keeps the rule out of the channel.
        copper_clause = (not has_layer_clause or (
            isinstance(layer_clause, str)
            and (layer_clause in ('outer', 'inner', '*.Cu', '*')
                 or layer_clause.endswith('.Cu'))))
        track = (_parse_track_condition(condition)
                 if condition and copper_clause else None)
        if track is not None:
            cls, other_only, except_coupled = track
            track_rules.append(TrackRule(str(name), cls, other_only, clearance_mm,
                                         except_coupled, layer_clause))
            notes.append(f"rule '{name}': track-to-track clearance "
                         f"{clearance_mm}mm for class '{cls}'"
                         f"{' vs other classes only' if other_only else ''}"
                         f"{', its own coupled partner exempt' if except_coupled else ''}"
                         f"{f' on {layer_clause}' if layer_clause else ''}"
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


# ---------------------------------------------------------------------------
# THE THIRD CHANNEL: length windows (#1313)
# ---------------------------------------------------------------------------
# `(constraint length (min ..) (opt ..) (max ..))` bounds each matching net's
# routed length; KiCad's DRC reports `length_out_of_range` outside it. No
# router step routes to it (design_rules lists the kind as unconsumed), so this
# channel is for the GRADER: check_drc reads it and measures each net the way
# KiCad's length DRC does (net_queries.net_match_length, mode 'total'). Before
# it, check_drc and board_score read a board clean that KiCad failed 8 times.
#
# Honoured conditions: none (every net), or `||`-joined terms each of which is
# `A.NetClass == 'X'`, `A.NetName == 'X'`, `A.hasNetclass('X')` or
# `A.inDiffPair('X')`. KiCad's string `==` matches `*` / `?` wildcards
# case-insensitively (libeval VALUE::EqualTo -> WildCompareString), so these
# do too. NetClass is read as membership, as the track channel reads it.
# inDiffPair is KiCad's inDiffPairFunc (#1303): the net has a MatchDpSuffix
# complement that exists on the board, and its base name -- or the base
# without its trailing '_' -- matches X with wxString::Matches, which is
# case-SENSITIVE. Anything else -- another predicate, `&&`, a (layer ..)
# clause, a time-domain bound -- is kept as an UNSUPPORTED rule with its
# reason, never dropped: a grader must say what it did not grade.
#
# The same channel carries the diff-pair constraints (#1303), selected with
# `kinds`: `skew` (KiCad's skew_out_of_range, with its `within_diff_pairs`
# option) and `diff_pair_uncoupled` (diff_pair_uncoupled_length_too_long).
# check_drc grades both; each kind is its own rule list, because KiCad picks
# the binding rule per constraint type.
_LEN_TERM = re.compile(
    r"^A\s*\.\s*(?:(NetClass|NetName)\s*==\s*['\"]([^'\"]*)['\"]"
    r"|(hasNetclass|inDiffPair)\s*\(\s*['\"]([^'\"]*)['\"]\s*\))$")
_TIME_VAL = re.compile(r"^-?[0-9.]+\s*(fs|ps|ns|us|s)$")
#: The constraint kinds this channel reads.
LENGTH_KINDS = ('length', 'skew', 'diff_pair_uncoupled')


class LengthRule(NamedTuple):
    """One `length` (or, by `kind`, `skew` / `diff_pair_uncoupled`)
    constraint. `terms` is a tuple of ('netclass'|'netname'|'indiffpair',
    pattern) alternatives -- () for an unconditioned rule (every net) -- or
    None when the scope is outside the honoured subset, with the reason in
    `unsupported`. Bounds are mm; a missing bound is None.
    `within_diff_pairs` is the skew option: grade each pair's P against its
    own N instead of every bound net against the longest."""
    name: str
    min_mm: Optional[float]
    opt_mm: Optional[float]
    max_mm: Optional[float]
    terms: Optional[Tuple[Tuple[str, str], ...]]
    unsupported: str = ''
    severity: str = 'error'
    kind: str = 'length'
    within_diff_pairs: bool = False


def _parse_length_condition(expr: Optional[str]):
    """(terms, reason): terms as LengthRule.terms, reason '' when honoured."""
    if expr is None or not expr.strip():
        return (), ''
    if '&&' in expr or '!' in expr:
        return None, f"condition {expr!r} is not an ||-list of net terms"
    terms = []
    for raw in expr.split('||'):
        t = raw.strip()
        while t.startswith('(') and t.endswith(')'):
            t = t[1:-1].strip()
        m = _LEN_TERM.match(t)
        if not m:
            return None, f"condition term {raw.strip()!r} is not modelled"
        if m.group(1) == 'NetName':
            terms.append(('netname', m.group(2)))
        elif m.group(3) == 'inDiffPair':
            terms.append(('indiffpair', m.group(4)))
        else:
            terms.append(('netclass', m.group(2) if m.group(1) else m.group(4)))
    return tuple(terms), ''


_KIND_NOUN = {'length': 'length window', 'skew': 'skew window',
              'diff_pair_uncoupled': 'max uncoupled length'}


def parse_dru_length_rules(text: str, kinds=('length',)
                           ) -> Tuple[List[LengthRule], List[str]]:
    """Every constraint of `kinds` (default `length` alone, #1313) in
    .kicad_dru text, in FILE order, and notes. A rule carrying several of the
    kinds gives one entry per kind. A `(severity ignore)` rule is dropped with
    a note (KiCad grades nothing there); every other rule is returned,
    honoured or not."""
    rules: List[LengthRule] = []
    notes: List[str] = []
    for node in _parse_nodes(_tokenize(text)):
        if not isinstance(node, list) or not node or node[0] != "rule":
            continue
        name = node[1] if len(node) > 1 and not isinstance(node[1], list) else "?"
        found = []                  # (kind, bounds, within_diff_pairs, time-valued)
        condition = None
        layer_clause = None
        severity = 'error'
        for item in node:
            if not isinstance(item, list) or not item:
                continue
            if item[0] == "severity" and len(item) >= 2:
                severity = str(item[1])
            elif item[0] == "condition" and len(item) >= 2:
                condition = item[1]
            elif item[0] == "layer":
                layer_clause = item[1] if len(item) >= 2 else ''
            elif item[0] == "constraint" and len(item) >= 2 and item[1] in kinds:
                bounds, within, timed = {}, False, False
                for sub in item[2:]:
                    if isinstance(sub, list) and len(sub) >= 2 \
                            and sub[0] in ('min', 'opt', 'max'):
                        bounds[sub[0]] = _to_mm(sub[1])
                        timed = timed or bool(_TIME_VAL.match(str(sub[1]).strip()))
                    elif isinstance(sub, list) and sub and sub[0] == 'within_diff_pairs':
                        within = True
                found.append((item[1], bounds, within, timed))
        if not found:
            continue
        if severity == 'ignore':
            for kind, _b, _w, _t in found:
                notes.append(f"rule '{name}': {_KIND_NOUN.get(kind, kind)}, "
                             f"severity ignore -- skipped")
            continue
        terms, why = _parse_length_condition(condition)
        if layer_clause is not None and terms is not None:
            terms, why = None, f"a (layer {layer_clause}) clause is not modelled"
        for kind, bounds, within, timed in found:
            t, w = terms, why
            if timed and t is not None:
                t, w = None, "a time-domain bound (ps/ns) is not modelled"
            rules.append(LengthRule(str(name), bounds.get('min'), bounds.get('opt'),
                                    bounds.get('max'), t, w, severity, kind, within))
    return rules, notes


def read_board_length_rules(board_path: str, kinds=('length',)
                            ) -> Tuple[List[LengthRule], List[str]]:
    """Auto-read the sibling ``<board>.kicad_dru`` length windows (#1313) --
    or, by `kinds`, its skew / diff_pair_uncoupled rules (#1303).
    ([], []) when there is no dru file."""
    dru = os.path.splitext(board_path)[0] + ".kicad_dru" if board_path else ""
    if not dru or not os.path.isfile(dru):
        return [], []
    try:
        text = open(dru, encoding="utf-8").read()
    except OSError as e:
        return [], [f"could not read {dru}: {e}"]
    return parse_dru_length_rules(text, kinds)


def length_rule_matches(rule: LengthRule, net_name: str, classes,
                        net_names=None) -> Optional[bool]:
    """Does `rule` bind the net? None when its scope is not modelled (the
    caller cannot know). `classes` is the net's class memberships; a net with
    none is in Default. `net_names` (every net name on the board) lets an
    inDiffPair term require the complement net to exist, as KiCad does;
    without it the complement is taken as present."""
    if rule.terms is None:
        return None
    if not rule.terms:
        return True
    cls = list(classes or ('Default',))
    for kind, pat in rule.terms:
        if kind == 'netname':
            if _kicad_wild_eq(pat, net_name or ''):
                return True
        elif kind == 'indiffpair':
            if in_diff_pair(net_name or '', pat, net_names):
                return True
        elif any(_kicad_wild_eq(pat, c) for c in cls):
            return True
    return False


def in_diff_pair(net_name: str, pattern: str, net_names=None) -> bool:
    """KiCad's `A.inDiffPair('pattern')` (pcbexpr_functions.cpp
    inDiffPairFunc, 10.0.0): the net has a MatchDpSuffix complement that
    exists on the board, and its base name -- or, when the base ends in '_',
    the base without it -- matches `pattern` with wxString::Matches (`*`/`?`,
    case-sensitive)."""
    from design_rules import match_dp_suffix
    rv, comp, base = match_dp_suffix(net_name)
    if not rv or (net_names is not None and comp not in net_names):
        return False
    if _wx_matches(pattern, base):
        return True
    return base.endswith('_') and _wx_matches(pattern, base[:-1])


def _wx_matches(pattern: str, text: str) -> bool:
    """wxString::Matches: `*` and `?` wildcards, case-sensitive."""
    rx = ''.join('.*' if ch == '*' else '.' if ch == '?' else re.escape(ch)
                 for ch in pattern)
    return re.fullmatch(rx, text, re.DOTALL) is not None


def _kicad_wild_eq(pattern: str, text: str) -> bool:
    """KiCad's string `==`: case-insensitive, `*` and `?` wildcards and
    nothing else special (WildCompareString) -- unlike fnmatch, a `[` in a
    net name is literal."""
    rx = ''.join('.*' if ch == '*' else '.' if ch == '?' else re.escape(ch)
                 for ch in pattern)
    return re.fullmatch(rx, text, re.IGNORECASE | re.DOTALL) is not None


def binding_length_rule(rules: List[LengthRule], net_name: str, classes,
                        net_names=None):
    """The rule KiCad applies to the net: rules are evaluated LAST-to-first and
    the first match wins (as the layer channel reads them). Returns (rule,
    None), (None, None) when no rule binds, or (None, rule) when the walk
    reaches an unsupported rule first -- whether IT binds is unknown, so the
    net cannot be graded. Pass the rules of ONE kind: KiCad picks per kind."""
    for r in reversed(rules):
        m = length_rule_matches(r, net_name, classes, net_names)
        if m is None:
            return None, r
        if m:
            return r, None
    return None, None


def unbound_length_rules(rules: List[LengthRule], net_names: Dict[int, str],
                         memberships) -> List[LengthRule]:
    """Rules of ONE kind that bind NO net on the board (#1388).

    A rule is bound when :func:`binding_length_rule` returns it for at least
    one net. One whose condition names a class or net the board does not have
    (or whose every candidate net is taken by a later rule) grades nothing,
    and a report that counts it among "N rule(s)" reads as graded and clean.
    A rule whose scope is not modelled is never listed: whether it binds is
    unknown, and the callers already name it NOT GRADED. ``memberships`` is
    ``list_nets.net_class_memberships`` for ``net_names``."""
    board_names = set(n for n in net_names.values() if n)
    bound = set()
    for nid, name in net_names.items():
        if not name:
            continue
        rule, blocker = binding_length_rule(rules, name, memberships.get(nid),
                                            board_names)
        if rule is not None:
            bound.add(id(rule))
        if blocker is not None:
            bound.add(id(blocker))
    return [r for r in rules if r.terms is not None and id(r) not in bound]


def track_pair_clearance(track_rules: List[TrackRule], a_cls, b_cls,
                         resolved: float, *, coupled: bool = False,
                         layer: Optional[str] = None
                         ) -> Tuple[float, Optional[TrackRule]]:
    """THE pair-exact track-rule resolver: (effective mm, the rule that RAISED
    it or None).

    ``resolved`` is the caller's already-resolved pair value -- class pairwise
    max with the #498 layer replacement applied -- and this is **raise-only**
    over it, so a rule at or below it is inert and the fab floor can never be
    dragged down. ``a_cls`` / ``b_cls`` are the two nets' class-membership sets
    from ``list_nets.net_class_memberships``.

    #1295: ``coupled`` says the two nets are a diff pair's P and N to KiCad
    (``nets_coupled``), which exempts them from an ``except_coupled`` rule;
    ``layer`` is where the two tracks meet, which a rule's layer clause must
    take in (None binds it everywhere, the over-requiring answer). A caller
    that passes neither gets the #735 answer for every rule shape.

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
        if r.except_coupled and coupled:
            continue
        if r.layer_clause is not None and not track_rule_on_layer(r, layer):
            continue
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
    own invocation for exact pricing.

    #1295: a "keep pairs apart" rule (``except_coupled``) prices like a rule
    without the term -- a member's partner is a member too, so its stamp is
    raised for the partner as well; the P<->N pricing sites read the exact
    value from ``config.track_partner_clearances`` instead. A rule's layer
    clause is not read here either: one value per net prices it on every
    layer. Both over-block, neither under-blocks."""
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
    routed set. Announces what is honored, once per (board, rules).

    #1295: with a "keep pairs apart" rule, ``config.track_partner_clearances``
    ({frozenset({P, N}): mm}) carries the pair-exact track-rule value of each
    diff pair KiCad couples by name, for ``pair_clearance``'s P<->N sites --
    the per-net map above prices the partner too. Empty otherwise, and on the
    explicit-dict path, whose map a parent resolved (a nested sub-run then
    prices P<->N at that map, over-requiring)."""
    config.track_partner_clearances = {}
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
    if any(r.except_coupled for r in rules):
        config.track_partner_clearances = {
            frozenset((p, n)): track_pair_clearance(
                rules, memberships.get(p, ()), memberships.get(n, ()), 0.0,
                coupled=True)[0]
            for p, n in coupled_net_pairs(nets) if p in eff or n in eff}
    _key = (os.path.abspath(input_file),
            tuple(sorted((r.name, r.cls, r.other_only, r.clearance_mm,
                          r.except_coupled, r.layer_clause or '')
                         for r in rules)))
    if _key not in _TRACK_ANNOUNCED:
        _TRACK_ANNOUNCED.add(_key)
        for r in rules:
            print(f"Track-to-track clearance rule from the board's .kicad_dru "
                  f"(raise-only on seg-seg pairs): '{r.name}' class "
                  f"'{r.cls}' {r.clearance_mm:g}mm"
                  f"{' (vs other classes only)' if r.other_only else ''}"
                  f"{' (its own coupled partner exempt)' if r.except_coupled else ''}"
                  f"{f' (on {r.layer_clause}; the router prices it on every layer)' if r.layer_clause else ''}"
                  f" -- {len(eff)} net(s) priced")


_ANNOUNCED = set()  # (board path, map items) already printed this process


def _install_rules_quietly(config, input_file, pcb_data, net_keepouts=False):
    """#530: attach the full DesignRules table as ``config.rules`` at the same
    chokepoint every engine already passes through for the layer map. Report-
    only in this phase (the resolver announces unsupported rules once per
    board); the legacy channels below keep deciding the copper until each
    consumer is migrated. Never raises into an engine."""
    if pcb_data is None:
        return
    try:
        from design_rules import install_design_rules
        from fab_tiers import fab_floors
        copper = list(getattr(getattr(pcb_data, 'board_info', None),
                              'copper_layers', None) or getattr(config, 'layers', []) or [])
        fab = None
        try:
            fab = fab_floors(len(copper) or 2)
        except Exception:                                       # noqa: BLE001
            fab = None
        install_design_rules(config, input_file, pcb_data, fab_floor=fab,
                             net_keepouts=net_keepouts)
    except Exception as e:                                      # noqa: BLE001
        print(f"  WARNING: design rules not installed ({e})")


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
    only matters that every plausibly-referenced layer resolves.

    A coupled-pair rule (``AB.isCoupledDiffPair()``, #1293) counts too: KiCad
    grades a pair's gap at max(this minimum, its class diff_pair_gap), so a
    board minimum above the rule fails every pair built at the rule's value
    (measured on KiCad 10: 6 diff_pair_gap_out_of_range at 0.25, 0 at 0.15)."""
    copper = ["F.Cu", "B.Cu"] + [f"In{i}.Cu" for i in range(1, 31)]
    lmap, _ = read_board_layer_clearances(board_path, copper)
    vals = list(lmap.values())
    dru = os.path.splitext(board_path)[0] + ".kicad_dru" if board_path else ""
    if dru and os.path.isfile(dru):
        try:
            from design_rules import parse_dru
            with open(dru, encoding="utf-8") as fh:
                rules, _ = parse_dru(fh.read())
            vals += [r.constraints["clearance"]["min"] for r in rules
                     if r.coupled and not r.unsupported
                     and "min" in r.constraints.get("clearance", {})]
        except Exception:                                       # noqa: BLE001
            pass
    return min(vals) if vals else None


def _record_board_copper(config, pcb_data) -> None:
    """Keep the board's copper list on `config` for
    `GridRouteConfig.pad_pair_clearance`; untouched without a board."""
    copper = list(getattr(getattr(pcb_data, 'board_info', None),
                          'copper_layers', None) or [])
    if copper:
        config.board_copper_layers = copper


def _read_layer_map(input_file, pcb_data, fallback_layers):
    """({layer: mm}, notes, copper) for the board: its sibling .kicad_dru
    expanded over the board's copper list (``fallback_layers`` without one)
    and pinned up to the fab tier's clearance floor."""
    copper = None
    if pcb_data is not None and getattr(pcb_data, 'board_info', None) is not None:
        copper = list(pcb_data.board_info.copper_layers or [])
    if not copper:
        copper = list(fallback_layers or [])
    try:
        from fab_tiers import fab_floors
        floor = fab_floors(len(copper)).get('clearance')
    except Exception:
        floor = None
    lmap, notes = read_board_layer_clearances(input_file or "", copper,
                                              fab_clearance_floor=floor)
    return lmap, notes, copper


def resolve_layer_clearances(layer_clearances, input_file, pcb_data=None,
                             fallback_layers=()) -> Dict[str, float]:
    """The #498 map `install_layer_clearances` installs, for a caller that
    needs it before it has a config (route_diff's coupling gap, #1145).
    Quiet; the same precedence: an explicit dict wins, else the sibling
    .kicad_dru of ``input_file`` (or ``PCBData.source_path``)."""
    if layer_clearances is not None:
        return dict(layer_clearances)
    if not input_file:
        input_file = getattr(pcb_data, 'source_path', "") or ""
    return _read_layer_map(input_file, pcb_data, fallback_layers)[0]


def pair_gap_rule_floor(layer_map: Dict[str, float], layers,
                        track_rules: List[TrackRule] = (),
                        p_cls=frozenset(), n_cls=frozenset(),
                        coupled: bool = False
                        ) -> Tuple[float, Optional[str]]:
    """The largest .kicad_dru clearance that binds a diff pair's P against
    its own N on a layer in ``layers``, and what set it: (mm, why), or
    (0.0, None) when no rule binds (#1145).

    KiCad grades P against N like any two nets, so a layer rule (#498) on a
    layer the pair routes on and a track rule (#735) whose class takes in
    the pair (``track_pair_clearance``, pair-exact: an ``other_only`` rule
    exempts a member's own partner, and so does a "keep pairs apart" rule
    when ``coupled`` -- the two names are a pair to KiCad, #1295) both bind
    the coupled run. The caller raises the coupling gap to this and never
    lowers it, so a rule that RELAXES its layer below the class changes
    nothing. One value over every layer: a per-layer gap would need
    per-layer geometry."""
    best, why = 0.0, None
    for layer in layers or ():
        v = layer_map.get(layer) if layer_map else None
        if v is not None and v > best:
            best, why = v, f"the .kicad_dru clearance rule on {layer}"
    if track_rules:
        for layer in list(layers or ()) or [None]:
            v, rule = track_pair_clearance(track_rules, p_cls, n_cls, best,
                                           coupled=coupled, layer=layer)
            if rule is not None:
                best, why = v, (f"the .kicad_dru track rule '{rule.name}' "
                                f"(class '{rule.cls}')")
    return best, why


def install_layer_clearances(config, layer_clearances, input_file, pcb_data=None,
                             net_keepouts=False):
    """Resolve and install the #498 per-layer map on ``config``, engine-side so
    BOTH fronts inherit it (the CLI passes nothing; the GUI passes nothing --
    there is deliberately no flag or control, the .kicad_dru is the single
    source of truth). An explicit dict (tests) wins and stops the auto-read,
    mirroring the net_clearances pattern; None -> read the sibling .kicad_dru
    of ``input_file``, expanded over the BOARD's full copper list (rules may
    target layers outside the routed subset -- via keep-outs still need them)
    and pinned up to the fab tier's clearance floor. Prints what is honored.
    ``net_keepouts``: the caller stamps net-filtered keep-outs (route.py /
    route_diff.py), so the design rules derive them (net_keepouts.py)."""
    if layer_clearances is not None:
        config.layer_clearances = dict(layer_clearances)
        _install_rules_quietly(config, input_file, pcb_data, net_keepouts)
        _record_board_copper(config, pcb_data)
        return
    if not input_file:
        # Engines whose signatures carry no input path (planes, fanout, oracle
        # sub-configs) discover the board file via PCBData.source_path.
        input_file = getattr(pcb_data, 'source_path', "") or ""
    _install_rules_quietly(config, input_file, pcb_data, net_keepouts)
    lmap, notes, copper = _read_layer_map(input_file, pcb_data, config.layers)
    # The copper list the map is expanded over, for pad_pair_clearance (a
    # `*.Cu` pad's shared layers are the BOARD's, not the routed subset's).
    config.board_copper_layers = list(copper)
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
