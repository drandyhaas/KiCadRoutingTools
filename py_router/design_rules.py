"""KiCad design rules, resolved in KiCad's own order (#530).

ONE resolver for every constraint the router, the graders, the placer and both
front-ends care about: copper clearance, hole/edge/hole-to-hole clearance,
track width, via diameter, hole size, annular width, diff-pair gap/width,
``disallow`` and ``via_count``. Its inputs are exactly what KiCad's DRC engine
reads and its evaluation is the order ``pcbnew/drc/drc_engine.cpp`` uses,
verified against the 9.0 and master sources on 2026-09-03:

    1  pad / footprint clearance OVERRIDE on a or b (clearance kinds only):
           max(overrides), floored at rules.min_clearance -> RETURN
           (KiCad returns before it looks at a single net class or rule)
    2  the constraint vector, walked FORWARD, min/opt/max applied PER FIELD,
       the last matching rule winning each field:
           [Board Setup minimum]                 rules.min_*   (SetMin)
         + [net-class implicit rules]            clearance = max(class a, class b)
                                                 (SetMin); track_width / via /
                                                 drill / diff_pair_* are SetOpt
                                                 -- the size KiCad DRAWS, never
                                                 a floor -- except that a class
                                                 diff_pair_gap below the class
                                                 clearance is ALSO the clearance
                                                 between a pair's P and N
                                                 ("(diff pair)" rule, #1542;
                                                 pair_gap_clearance)
         + [.kicad_dru rules, file order]        each matching rule overwrites
                                                 the fields it sets
    3  zone local clearance: max(result, zone clearance)
    4  kind in {clearance, diff_pair_gap}: max(result, rules.min_clearance)
       -- the ONLY kinds with a post-loop board floor, and MEASURED on the
       installed KiCad 10.0.0 it floors a net-class value and a pad override
       but NOT an explicit custom rule (tests/oracle/constraint_agreement.py).
       A later rule may take track_width / via_diameter / hole_size /
       hole_to_hole / hole_clearance / edge_clearance BELOW the Board Setup
       minimum too, and KiCad grades the rule.
    5  fab profile floor (THIS TOOL ONLY, raise-only, disclosed when it binds)

Net classes: a net in several classes gets an AGGREGATE class that takes each
property from the highest-priority (lowest ``priority`` number) member class
that sets it; Default has the lowest priority. Rules have no priority of their
own -- file order is the order.

Two loaders build the same table: ``DesignRules.from_project`` (the sibling
``.kicad_pro`` / ``.kicad_dru`` files, the CLI path) and
``DesignRules.from_pcbnew`` (the live board, the GUI path). A parity gate
diffs ``table()`` between them. A rule this module cannot model is kept in
the table marked UNSUPPORTED with the reason, reported once per board, and
never silently dropped -- the #770 finding was that the previous reader had
never bound on a real board and said nothing about it.

Vocabulary (``kind`` strings) is KiCad's ``.kicad_dru`` constraint token,
with two additions the net-class channel needs: ``diff_pair_width`` (a class
field, not a dru token) and ``via_drill`` as an alias of ``hole_size``.
"""
from __future__ import annotations

import fnmatch
import json
import os
import re
from dataclasses import dataclass, field, replace
from typing import Dict, FrozenSet, Iterable, List, Optional, Sequence, Set, Tuple

__all__ = [
    'Constraint', 'RuleItem', 'Rule', 'DesignRules', 'Unsupported',
    'parse_dru', 'parse_condition', 'CLEARANCE_KINDS', 'SIZE_KINDS',
    'BOARD_MIN_KEYS', 'NETCLASS_OPT_KEYS', 'FAB_FLOOR_KEYS',
    'install_design_rules', 'KIND_ALIASES', 'project_class_membership',
]

# --------------------------------------------------------------------------
# Vocabulary
# --------------------------------------------------------------------------

# Every constraint token the KiCad 9/10 parser accepts, so an unknown token is
# a parse note rather than a silent drop. Only a subset is CONSUMED (see
# CONSUMED_KINDS); the rest ride in the table for disclosure.
DRU_KINDS = frozenset({
    'assertion', 'clearance', 'creepage', 'hole_clearance', 'edge_clearance',
    'hole_size', 'hole_to_hole', 'courtyard_clearance', 'silk_clearance',
    'text_height', 'text_thickness', 'track_width', 'track_angle',
    'track_segment_length', 'connection_width', 'annular_width',
    'via_diameter', 'zone_connection', 'thermal_relief_gap',
    'thermal_spoke_width', 'min_resolved_spokes', 'disallow', 'length', 'skew',
    'via_count', 'diff_pair_gap', 'diff_pair_uncoupled', 'physical_clearance',
    'physical_hole_clearance',
    # master / KiCad 10 additions
    'via_dangling', 'microvia_stack_depth', 'microvia_aspect_ratio',
    'solder_mask_expansion', 'solder_mask_sliver', 'solder_paste_abs_margin',
    'solder_paste_rel_margin', 'stub_length', 'return_path',
    'net_chain_length', 'bridged_mask',
})

# Kinds resolved as a (pairwise) SPACING: the DRC-limit family.
CLEARANCE_KINDS = frozenset({
    'clearance', 'hole_clearance', 'edge_clearance', 'hole_to_hole',
    'physical_clearance', 'physical_hole_clearance', 'courtyard_clearance',
    'silk_clearance', 'diff_pair_gap',
})
# Kinds resolved as a SIZE of one item: the draw-default family. ``opt`` is
# what we draw when free to choose; ``min``/``max`` bound it.
SIZE_KINDS = frozenset({
    'track_width', 'via_diameter', 'hole_size', 'annular_width',
    'diff_pair_width', 'connection_width',
})
#: The kinds a via is sized by (`relaxed_via_nets`, `CourtyardIndex`, #1805)
VIA_SIZE_KINDS = ('annular_width', 'via_diameter', 'hole_size')
#: The kinds check_drc grades by where each item stands when a courtyard
#: term binds them: a via's size (#1805), a track's width and the clearance
#: of tracks, vias and pads (#1832). The router reads none of them.
COURTYARD_GRADED_KINDS = VIA_SIZE_KINDS + ('track_width', 'clearance')
# The kinds a rule may carry without being reported as "parsed, not
# consumed". WIDER than what is read today: the resolver's callers ask only
# for track_width / via_diameter / hole_size (draw_size, floor), copper
# clearance reaches the router only through kicad_dru's layer- and
# track-scoped subsets, and `disallow` through net_keepouts (a rule area's
# keep-out, or a track ban's forbidden layers -- whatever of a `disallow`
# rule no step honours is named by `Rule.router_unhonoured`, #1527). A rule
# built only on the other kinds listed here is reported neither "NOT
# honoured" nor "not consumed", except a ROUTER_UNHONOURED_KINDS kind
# (`annular_width`, `via_count`, `hole_clearance`), which `unsupported()`
# names NOT honoured.
CONSUMED_KINDS = CLEARANCE_KINDS | SIZE_KINDS | {'disallow', 'via_count'}
# Kinds no routing step honours but a grader does: check_drc grades every
# `length` window (kicad_dru.read_board_length_rules, #1313) and every
# diff-pair `skew` / `diff_pair_uncoupled` rule through the same channel
# (#1303).
GRADED_ONLY_KINDS = frozenset({'length', 'skew', 'diff_pair_uncoupled'})
# Kinds the resolver models that a routing step does not hold every via to
# (#1490): a via a step sizes itself meets an annular_width rule (#1676,
# `DesignRules.via_meeting_rules`), an explicit via size and a last-resort
# descent need not, so a rule declaring one is reported NOT honoured -- in
# route.py's `design_rules.unsupported_rules` -- rather than passing as consumed.
# `via_count` (#1707) likewise: no step counts a net's vias against it -- a
# net is routed with the vias it needs, completion first -- and check_drc
# grades it as KiCad's too_many_vias does. `hole_clearance` too: no step
# reads a rule of it (`obstacle_map.resolve_hole_clearance` holds the board's
# min_hole_clearance or the fab floor), and check_drc grades it
# (`check_drc.hole_rule_grade`).
ROUTER_UNHONOURED_KINDS = frozenset({'annular_width', 'via_count', 'hole_clearance'})
#: why each ROUTER_UNHONOURED_KINDS kind is not honoured, as `unsupported()` says it
_ROUTER_UNHONOURED_WHY = {
    'annular_width': ("a via a step sizes itself holds it, an explicit via size and a "
                      "last-resort descent need not, so this line is printed whatever "
                      "the vias are; route.py and route_diff.py grade their own vias "
                      "against it when they end, check_drc the board"),
    'via_count': ("no routing step counts a net's vias against it (a net is routed "
                  "with the vias it needs, completion first); check_drc grades the "
                  "board's vias per net against it, as KiCad's too_many_vias does"),
    'hole_clearance': ("no routing step reads this rule: copper is kept off another "
                       "net's NPTH holes at the board's min_hole_clearance (or the fab "
                       "floor) and off plated holes at the copper clearance, not at this "
                       "rule's value; check_drc grades the board against it"),
}
# `disallow` item tokens (#1527). The router lays tracks and THROUGH vias
# only, so a rule forbidding only blind, buried or micro vias asks nothing of
# it. A track ban is honoured by the steps that derive it (route.py and
# route_diff.py, `net_keepouts.derive_layer_bans`: the bound nets' forbidden
# layers); every other token is not honoured by any routing step and is
# reported NOT honoured -- check_drc grades the track and via ones.
DISALLOW_NOT_LAID = frozenset({'micro_via', 'blind_via', 'buried_via'})
DISALLOW_GRADED = frozenset({'track', 'via', 'through_via'}) | DISALLOW_NOT_LAID
#: The via `disallow` tokens and the via types each forbids (None: every via),
#: as KiCad's items_not_allowed reads them (#1527, `DesignRules.via_disallow_rule`)
DISALLOW_VIA_TYPES = {'via': None, 'through_via': {'through'},
                      'micro_via': {'micro'},
                      'blind_via': {'blind', 'blind_buried'},
                      'buried_via': {'buried', 'blind_buried'}}


def disallow_router_reason(tokens, honoured_tokens=frozenset()) -> Optional[str]:
    """Why a routing step does not honour a scope-evaluable `disallow` rule
    forbidding ``tokens`` (None when it asks nothing more of the router than
    ``honoured_tokens`` covers)."""
    left = sorted(set(tokens) - DISALLOW_NOT_LAID - set(honoured_tokens))
    if not left:
        return None
    graded = [t for t in left if t in DISALLOW_GRADED]
    return (f"disallow {', '.join(left)}: this step does not keep the bound "
            f"nets' {', '.join(left)} off"
            + ("; route.py and route_diff.py keep a net's tracks off the "
               "layers the rule forbids" if 'track' in left else '')
            + (f"; check_drc grades {', '.join(graded)}" if graded else ''))

# Spellings a caller may use for a kind.
KIND_ALIASES = {
    'via_drill': 'hole_size', 'via_size': 'via_diameter',
    'board_edge_clearance': 'edge_clearance', 'board_edge': 'edge_clearance',
    'hole_to_hole_clearance': 'hole_to_hole', 'min_track_width': 'track_width',
}

# kind -> the Board Setup key (.kicad_pro board.design_settings.rules).
BOARD_MIN_KEYS = {
    'clearance': 'min_clearance',
    'track_width': 'min_track_width',
    'via_diameter': 'min_via_diameter',
    'hole_size': 'min_through_hole_diameter',
    'annular_width': 'min_via_annular_width',
    'hole_to_hole': 'min_hole_to_hole',
    'hole_clearance': 'min_hole_clearance',
    'edge_clearance': 'min_copper_edge_clearance',
    'connection_width': 'min_connection',
}
# kind -> the net-class field KiCad loads as SetOpt (a draw default).
NETCLASS_OPT_KEYS = {
    'track_width': 'track_width',
    'via_diameter': 'via_diameter',
    'hole_size': 'via_drill',
    'diff_pair_width': 'diff_pair_width',
    'diff_pair_gap': 'diff_pair_gap',
}
# kind -> fab_tiers FLOOR_KEYS entry (the tool's own raise-only floor).
FAB_FLOOR_KEYS = {
    'clearance': 'clearance',
    'track_width': 'track_width',
    'via_diameter': 'via_diameter',
    'hole_size': 'via_drill',
    'hole_to_hole': 'hole_to_hole',
    'edge_clearance': 'board_edge',
    'annular_width': 'annular',
    'diff_pair_gap': 'clearance',
    'diff_pair_width': 'track_width',
}

# KiCad's Default net class carries priority INT_MAX (lowest precedence).
_DEFAULT_PRIORITY = 2147483647
# The net-class fields whose absence brings Default into a net's effective class
# (`DesignRules.class_constituents`), each measured alone on kicad-cli 10.0.6.
# Dropping diff_pair_via_gap, schematic_color, pcb_color or line_style does not.
_CLASS_COMPLETE_FIELDS = ('clearance', 'track_width', 'via_diameter', 'via_drill',
                          'diff_pair_gap', 'diff_pair_width', 'microvia_diameter',
                          'microvia_drill', 'wire_width', 'bus_width')


def canonical_kind(kind: str) -> str:
    return KIND_ALIASES.get(kind, kind)


class Unsupported(Exception):
    """A rule construct this module deliberately does not model."""


# --------------------------------------------------------------------------
# Data
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Constraint:
    kind: str
    min: Optional[float] = None
    opt: Optional[float] = None
    max: Optional[float] = None
    source: str = 'none'
    # for kind == 'disallow': the item types the matching rules forbid
    disallow: FrozenSet[str] = frozenset()
    # True when the fab profile floor raised ``min`` above the KiCad answer.
    fab_bound: bool = False
    # ``min`` before the fab floor, so a caller can say what KiCad would accept.
    kicad_min: Optional[float] = None
    # the custom rule that set ``min`` (None: a class, an override, the board
    # minimum), and whether that rule is a coupled-pair rule (#1293)
    min_rule: Optional[str] = None
    min_rule_coupled: bool = False


@dataclass(frozen=True)
class RuleItem:
    """The A / B of a KiCad rule condition. Hashable so resolutions cache."""
    type: str = 'track'                      # track | via | pad | zone | hole | footprint | graphic | text
    net_id: int = 0
    net_name: str = ''
    # KiCad's effective class (`DesignRules.class_constituents`): the classes it
    # is built from, and its name, those classes joined by commas
    netclasses: FrozenSet[str] = frozenset()
    effective_class: str = 'Default'
    layers: FrozenSet[str] = frozenset()
    # a pad's other layers (F.Mask, ...), which existsOnLayer() reads too
    other_layers: FrozenSet[str] = frozenset()
    footprint_ref: Optional[str] = None      # as the file spells it
    footprint_lib: Optional[str] = None      # the part's library id, 'lib:name'
    pad_type: Optional[str] = None           # smd | thru_hole | np_thru_hole | connect
    pad_number: Optional[str] = None         # as the file spells it; '' is a pad with no number
    via_type: Optional[str] = None           # through | micro | blind_buried
    groups: FrozenSet[str] = frozenset()
    diff_pair: Optional[str] = None
    xy: Optional[Tuple[float, float]] = None
    # pad/footprint (clearance ...) override -> KiCad's early-return path
    clearance_override: Optional[float] = None
    # zone (connect_pads (clearance ...)) -> KiCad's post-loop max path
    local_clearance: Optional[float] = None
    # the rule areas the item lies in, for an areas=True condition only
    areas: FrozenSet[str] = frozenset()
    # #1805: the courtyards the item's copper overlaps, as (reference, library
    # id, 'front' | 'back' of that part), for a rule's `courtyard_condition`
    # (`CourtyardIndex`). None = not measured: a courtyard term then reads as
    # unknown and the rule stays unread, as it always was.
    courtyards: Optional[FrozenSet[Tuple[str, Optional[str], str]]] = None


@dataclass
class Rule:
    name: str
    order: int
    constraints: Dict[str, dict] = field(default_factory=dict)  # kind -> {min,opt,max} | {'disallow': set}
    layer_clause: Optional[str] = None       # raw token: a layer name, 'outer', 'inner', '*.Cu'
    condition_src: Optional[str] = None
    condition: object = None                  # AST, or None for unconditioned
    severity: Optional[str] = None
    unsupported: Optional[str] = None         # reason this rule is NOT evaluated
    unconsumed_kinds: List[str] = field(default_factory=list)
    # A `disallow` rule's condition parsed WITH its rule-area predicates
    # (parse_condition(areas=True)); `condition` stays None and `unsupported`
    # stays set, so the resolver never evaluates it. net_keepouts reads it.
    area_condition: object = None
    # #1805: the condition parsed WITH its courtyard predicates
    # (parse_condition(courtyards=True)) when they are all it does not read.
    # `unsupported` stays set: only an item whose `courtyards` were measured
    # (check_drc's vias, and #1832 its tracks and pads) is resolved against
    # it, by position. No routing step measures one, so none reads the rule.
    courtyard_condition: object = None
    # set by net_keepouts.derive_area_keepouts when the router honours the
    # rule as a net-filtered keep-out: what it honours, for the report
    honoured: Optional[str] = None
    # #1527: a `disallow` rule the resolver EVALUATES (its condition parses,
    # no rule area) but the routing step does not honour, and why --
    # `unsupported()` lists it while `unsupported` stays None, so check_drc's
    # resolve('disallow') still grades it. net_keepouts.derive_layer_bans
    # rewrites it for the track bans route.py / route_diff.py honour.
    router_unhonoured: Optional[str] = None
    # why KiCad cannot load this rule, and so enforces no rule of the file
    # (#1690, `kicad_condition_refusal`). It says nothing of whether this tool
    # reads the rule: `unsupported` does.
    kicad_refused: Optional[str] = None
    # the condition ANDs in AB.isCoupledDiffPair() (#1293): the rule binds a
    # diff pair's P against its own N
    coupled: bool = False
    # the condition ANDs in !AB.isCoupledDiffPair() (#1295): KiCad's "keep
    # pairs apart" rule, which binds everything EXCEPT a pair's own partner
    keep_apart: bool = False

    def layers_for(self, copper_layers: Sequence[str]) -> Optional[List[str]]:
        """Concrete copper layers this rule's (layer ...) clause names, or None
        for "every layer" (no clause)."""
        lc = self.layer_clause
        if lc is None:
            return None
        outer = [l for l in copper_layers if l in ('F.Cu', 'B.Cu')]
        if lc == 'outer':
            return outer
        if lc == 'inner':
            return [l for l in copper_layers if l not in ('F.Cu', 'B.Cu')]
        if lc in ('*.Cu', '*'):
            return list(copper_layers)
        return [lc] if lc in copper_layers else []


# --------------------------------------------------------------------------
# .kicad_dru parsing (s-expression) -- reuses kicad_dru's tokenizer
# --------------------------------------------------------------------------

_VAL = re.compile(r"^(-?[0-9.]+)\s*(mm|mil|in|um|nm|deg)?$")
_UNIT_MM = {"mm": 1.0, "mil": 0.0254, "in": 25.4, "um": 0.001, "nm": 1e-6,
            "deg": 1.0, None: 1.0}


def _to_num(tok) -> Optional[float]:
    if isinstance(tok, list):
        return None
    m = _VAL.match(str(tok).strip())
    if not m:
        return None
    try:
        return float(m.group(1)) * _UNIT_MM[m.group(2)]
    except (ValueError, KeyError):
        return None


def validate_dru_structure(text: str) -> None:
    """Reject malformed rule containers before a consumer certifies coverage.

    The legacy list parser is permissive and can drop an unterminated rule.
    Quotes, escapes and line comments must not count as structural parentheses.
    This validates structure only, not KiCad's full rule grammar.
    """
    depth = 0
    quoted = escaped = comment = False
    for char in text:
        if comment:
            comment = char != '\n'
        elif escaped:
            escaped = False
        elif quoted:
            if char == '\\':
                escaped = True
            elif char == '"':
                quoted = False
        elif char == '#':
            comment = True
        elif char == '"':
            quoted = True
        elif char == '(':
            depth += 1
        elif char == ')':
            depth -= 1
            if depth < 0:
                raise ValueError('unmatched closing parenthesis in custom rules')
    if quoted:
        raise ValueError('unterminated quoted string in custom rules')
    if depth:
        raise ValueError('unclosed parenthesis in custom rules')
    from kicad_dru import _tokenize, _parse_nodes
    for node in _parse_nodes(_tokenize(text)):
        if (not isinstance(node, list) or not node
                or not isinstance(node[0], str) or node[0] not in ('version', 'rule')):
            raise ValueError('unsupported top-level form in custom rules')
        if len(node) < 2 or isinstance(node[1], list):
            raise ValueError('missing version or rule name in custom rules')
        if node[0] == 'version':
            if len(node) != 2:
                raise ValueError('malformed version in custom rules')
            continue
        for clause in node[2:]:
            if (not isinstance(clause, list) or len(clause) < 2
                    or not isinstance(clause[0], str)
                    or clause[0] not in ('severity', 'layer', 'condition', 'constraint')
                    or isinstance(clause[1], list)):
                raise ValueError('unsupported rule clause in custom rules')


def parse_dru(text: str) -> Tuple[List[Rule], List[str]]:
    """Every rule in a .kicad_dru, in file order, with every constraint and
    every min/opt/max. Returns (rules, notes). A rule whose condition or layer
    clause is outside the modelled subset is returned with ``unsupported`` set
    (never dropped)."""
    from kicad_dru import _tokenize, _parse_nodes
    rules: List[Rule] = []
    notes: List[str] = []
    for order, node in enumerate(_parse_nodes(_tokenize(text))):
        if not isinstance(node, list) or not node or node[0] != 'rule':
            continue
        name = node[1] if len(node) > 1 and not isinstance(node[1], list) else '?'
        rule = Rule(name=str(name), order=order)
        for item in node[2:]:
            if not isinstance(item, list) or not item:
                continue
            head = item[0]
            if head == 'severity' and len(item) >= 2:
                rule.severity = str(item[1])
            elif head == 'layer' and len(item) >= 2:
                rule.layer_clause = str(item[1])
            elif head == 'condition' and len(item) >= 2:
                rule.condition_src = str(item[1])
            elif head == 'constraint' and len(item) >= 2:
                kind = canonical_kind(str(item[1]))
                if kind not in DRU_KINDS:
                    notes.append(f"rule '{rule.name}': unknown constraint "
                                 f"'{item[1]}' -- kept in the table, not evaluated")
                if kind == 'disallow':
                    rule.constraints['disallow'] = {
                        'disallow': {str(t).lower() for t in item[2:]
                                     if not isinstance(t, list)}}
                    continue
                spec = rule.constraints.setdefault(kind, {})
                for sub in item[2:]:
                    if isinstance(sub, list) and sub and sub[0] in ('min', 'opt', 'max') \
                            and len(sub) >= 2:
                        v = _to_num(sub[1])
                        if v is None:
                            rule.unsupported = (f"constraint {kind} {sub[0]} value "
                                                f"{sub[1]!r} is not a number")
                        else:
                            spec[sub[0]] = v
                    elif not isinstance(sub, list):
                        # bare token value (zone_connection solid, etc.)
                        spec.setdefault('tokens', []).append(str(sub))
        if rule.severity in ('ignore', 'exclusion'):
            rule.unsupported = f"severity {rule.severity}"
        rule.kicad_refused = _rule_refusal(node, rule)
        if rule.condition_src is not None and rule.unsupported is None:
            try:
                rule.condition = parse_condition(rule.condition_src)
                rule.coupled, rule.keep_apart = _coupled_terms(rule.condition)
            except Unsupported as e:
                rule.unsupported = f"condition: {e}"
                try:
                    rule.courtyard_condition = parse_condition(rule.condition_src,
                                                               courtyards=True)
                except Unsupported:
                    pass
                # A rule that only disallows, on a rule area: the routing
                # steps read it as a net-filtered keep-out (net_keepouts);
                # the resolver and every other step still do not.
                if set(rule.constraints) == {'disallow'}:
                    try:
                        rule.area_condition = parse_condition(rule.condition_src,
                                                              areas=True)
                    except Unsupported:
                        pass
                    else:
                        rule.unsupported = (f"condition: {e} here; route.py, "
                                            f"route_diff.py and repair_planes.py "
                                            f"read it as a net-filtered keep-out")
        if rule.layer_clause is not None and (rule.unsupported is None
                                              or rule.courtyard_condition is not None):
            lc = rule.layer_clause
            if not (lc in ('outer', 'inner', '*.Cu', '*') or lc.endswith('.Cu')):
                rule.unsupported = rule.unsupported or f"layer clause {lc!r} (non-copper)"
                rule.courtyard_condition = None
        rule.unconsumed_kinds = sorted(k for k in rule.constraints
                                       if k not in CONSUMED_KINDS)
        if 'clearance' in rule.constraints and rule.unsupported is None:
            # evaluable, and check_drc reads it for the violations it may make
            # legal, but no obstacle stamp prices one pad or one part
            term = _reads_one_part(rule.condition)
            if term:
                rule.router_unhonoured = (f"clearance scoped by {term}: routing "
                                          f"prices a net and a pad type, never one "
                                          f"pad or one part")
        if 'disallow' in rule.constraints and rule.unsupported is None:
            # #1527: evaluable, so nothing of it is "consumed" until a step
            # honours it -- it used to pass silently as consumed
            rule.router_unhonoured = disallow_router_reason(
                rule.constraints['disallow'].get('disallow', ()))
        rules.append(rule)
    return rules, notes


# KiCad's error key for what each constraint kind reports (`severity_of`)
_SEVERITY_KEY = {'edge_clearance': 'copper_edge_clearance', 'hole_size': 'drill_out_of_range'}
# (KiCad's report type, check_drc's row) of a via below a rule (`via_size_grade`)
_VIA_SIZE_FLAGS = {'via_diameter': ('via_diameter', 'VIA-SIZE'),
                   'hole_size': ('drill_out_of_range', 'VIA-DRILL-SIZE')}


def _via_attr(v, key):
    """A via's `size` / `drill` / `net_id`, from a Via or the dict a fanout
    engine returns, 0 when absent."""
    val = v.get(key) if isinstance(v, dict) else getattr(v, key, 0)
    return val or 0


# --------------------------------------------------------------------------
# Condition expressions
# --------------------------------------------------------------------------
# Grammar (the KiCad rule language subset we evaluate):
#   expr   := and
#   and    := or ('&&' or)*
#   or     := not ('||' not)*
#   not    := '!' not | atom
# '||' binds TIGHTER than '&&', as in KiCad (#1797): its grammar declares
#   `%left G_BOOL_AND.` before `%left G_BOOL_OR.`, and kicad-cli 10.0.6 reads
#   `X && Y || Z` as `X && (Y || Z)` and `X || Y && Z` as `(X || Y) && Z`.
#   atom   := '(' expr ')' | cmp
#   cmp    := term (('=='|'!=') term)?
#   term   := ITEM '.' NAME '(' args ')' | ITEM '.' NAME | STRING | NUMBER | 'L'
# Supported properties: NetClass, NetName, Net, Type, Layer, Pad_Type, Via_Type,
#   Pad_Number (#1660: a string compared as a NetName is, so '1' is not '01' and
#   a plain number is not a pad number -- `kicad_condition_refusal`)
# Supported functions: hasNetclass, hasExactNetclass, existsOnLayer (a wildcard is
#   a layer pattern), isMicroVia,
#   isBlindBuriedVia, isPlated, memberOfFootprint, memberOfGroup,
#   inDiffPair('pattern') (#1295), and AB.isCoupledDiffPair() as a term of an
#   AND only, positive (#1293: the pair's own P<->N rule) or negated (#1295:
#   KiCad's "keep pairs apart" rule); OR'ed or compared it stays Unsupported
# Recognised but NOT modelled (-> Unsupported): intersectsArea, enclosedByArea,
#   insideArea (each modelled ONLY for a `disallow` rule's net-filtered keep-out:
#   parse_condition(areas=True), net_keepouts.derive_area_keepouts),
#   intersectsCourtyard, intersectsFrontCourtyard, intersectsBackCourtyard,
#   isCoupledDiffPair in any other position, fromTo, memberOfSheet,
#   memberOfSheetOrChildren, hasComponentClass, getField, and any numeric
#   comparison (<, >, arithmetic).
# Read, though KiCad refuses it (`kicad_condition_refusal`, which is what says
#   so): onLayer, the spelling existsOnLayer replaced. Whether the tools should
#   read a rule KiCad will not load is a question they have not been asked to
#   change, so it stays read and the run names the file.

_TOKEN = re.compile(r"""
    \s*(?:
        (?P<str>'[^']*(?:'|$)|"[^"]*") |
        (?P<op>==|!=|&&|\|\||<=|>=|[<>!()+\-*/]) |
        (?P<num>-?\d+(?:\.\d+)?\s*(?:mm|mil|in|um|nm)?) |
        (?P<name>[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)?) |
        (?P<comma>,)
    )""", re.VERBOSE)

_SUPPORTED_PROPS = {'netclass', 'netname', 'net', 'type', 'layer', 'pad_type',
                    'via_type', 'pad_number'}
_SUPPORTED_FUNCS = {'hasnetclass', 'hasexactnetclass', 'onlayer', 'existsonlayer',
                    'ismicrovia', 'isblindburiedvia', 'isplated',
                    'memberoffootprint', 'memberofgroup', 'indiffpair'}
#: Rule-area predicates. KiCad's insideArea() is the older spelling of
#: intersectsArea() and KiCad 10 still grades it.
_AREA_FUNCS = {'insidearea', 'intersectsarea', 'enclosedbyarea'}
#: Courtyard predicates (#1805) -> which of the part's courtyards they read:
#: None either, 'front' the one on the part's own side, 'back' the other.
#: Measured on kicad-cli 10.0.6 with a via: insideCourtyard() and
#: intersectsCourtyard() read alike, and hold when the via's disc OVERLAPS
#: the courtyard (centre outside by less than its radius: yes; tangent: no);
#: insideFrontCourtyard() holds on a back part's B.CrtYd and
#: insideBackCourtyard() on neither part's own courtyard.
_COURTYARD_FUNCS = {'insidecourtyard': None, 'intersectscourtyard': None,
                    'insidefrontcourtyard': 'front', 'intersectsfrontcourtyard': 'front',
                    'insidebackcourtyard': 'back', 'intersectsbackcourtyard': 'back'}
_KNOWN_UNSUPPORTED_FUNCS = {
    'insidearea', 'intersectsarea', 'enclosedbyarea', 'intersectscourtyard',
    'intersectsfrontcourtyard', 'intersectsbackcourtyard',
    'iscoupleddiffpair', 'fromto', 'memberofsheet', 'memberofsheetorchildren',
    'hascomponentclass', 'getfield', 'isbetween',
}


def _tokens(src: str):
    pos = 0
    out = []
    while pos < len(src):
        m = _TOKEN.match(src, pos)
        if not m or m.end() == pos:
            if src[pos:].strip() == '':
                break
            raise Unsupported(f"cannot tokenize {src[pos:pos + 20]!r}")
        pos = m.end()
        kind = m.lastgroup
        if kind is None:
            continue
        out.append((kind, m.group(kind).strip()))
    return out


_NUMERIC_OPS = ('<', '>', '<=', '>=', '+', '-', '*', '/')

#: How a condition KiCad refuses is named: `report_lines` drops its "KiCad will
#: still enforce it" for one, since KiCad enforces nothing of that file.
_KICAD_REFUSES = 'KiCad refuses'


def _reads_one_part(node) -> Optional[str]:
    """The first term of a parsed condition that scopes it to one pad or one
    part (A.Pad_Number, memberOfFootprint()), or None."""
    if not isinstance(node, tuple) or not node:
        return None
    if node[0] == 'prop':
        return f'{node[1]}.Pad_Number' if node[2] == 'pad_number' else None
    if node[0] == 'call':
        return f'{node[1]}.memberOfFootprint()' if node[2] == 'memberoffootprint' else None
    if node[0] in ('cmp', 'and', 'or', 'not'):
        return next(filter(None, map(_reads_one_part, node[1:])), None)
    return None


#: Every function KiCad's condition compiler knows, with the arguments it will
#: not compile without. Measured on kicad-cli 10.0.6.
_KICAD_FUNCTIONS = {
    'existsonlayer': 1, 'isplated': 0, 'insidecourtyard': 1, 'insidefrontcourtyard': 1,
    'insidebackcourtyard': 1, 'intersectscourtyard': 1, 'intersectsfrontcourtyard': 1,
    'intersectsbackcourtyard': 1, 'insidearea': 1, 'intersectsarea': 1, 'enclosedbyarea': 1,
    'ismicrovia': 0, 'isblindburiedvia': 0, 'isblindvia': 0, 'isburiedvia': 0,
    'memberof': 1, 'memberofgroup': 1, 'memberoffootprint': 1, 'memberofsheet': 1,
    'memberofsheetorchildren': 1, 'fromto': 2, 'iscoupleddiffpair': 0, 'indiffpair': 1,
    'getfield': 1, 'hasnetclass': 1, 'hasexactnetclass': 1, 'hascomponentclass': 1,
}
#: Every property it knows, lower case with spaces as '_'. Measured on kicad-cli
#: 10.0.6: each name here compiles as `A.<name>`. The names tried were the Title
#: Case strings the KiCad binary holds, so a later KiCad may know one this does not.
#: `A.Net_Name` does not compile and `A.NetName` does.
_KICAD_PROPERTIES = frozenset({
    'arrow_direction', 'associated_layer', 'auto_thickness', 'back_covering',
    'back_plugging', 'back_tenting', 'backdrill_mode', 'barcode_type',
    'best_length_ratio', 'best_width_ratio', 'bold', 'border', 'border_style',
    'border_width', 'bottom_backdrill_size', 'bottom_counterbore_depth',
    'bottom_countersink_angle', 'capping', 'cell_separators', 'center_x', 'center_y',
    'clearance', 'clearance_override', 'column_width', 'component_class',
    'componentclass', 'copper_layers', 'corner_radius', 'corner_radius_ratio',
    'corner_radius_size', 'crossbar_height', 'curved_teardrops', 'diameter',
    'do_not_populate', 'enable', 'enable_teardrops', 'end_x', 'end_y',
    'error_correction', 'exclude_from_bill_of_materials',
    'exclude_from_position_files', 'exempt_from_courtyard_requirement',
    'extension_line_overshoot', 'external_border', 'fabrication_property', 'fill',
    'fill_mode', 'filling', 'font', 'front_covering', 'front_plugging',
    'front_tenting', 'hatch_gap', 'hatch_minimum_hole_ratio', 'hatch_width',
    'header_border', 'height', 'hole', 'hole_shape', 'hole_size_x', 'hole_size_y',
    'horizontal_justification', 'hyperlink', 'initial_side', 'italic',
    'keep_aligned_with_dimension', 'keep_out_footprints', 'keep_out_pads',
    'keep_out_tracks', 'keep_out_vias', 'keep_out_zone_fills', 'keep_upright',
    'keywords', 'knockout', 'layer', 'layer_bottom', 'layer_top', 'leader_length',
    'library_description', 'library_link', 'line_style', 'line_width', 'locked',
    'margin_bottom', 'margin_left', 'margin_right', 'margin_top', 'margin_x',
    'margin_y', 'max_amplitude', 'max_length', 'max_width', 'max_width_ratio',
    'min_amplitude', 'min_spacing', 'minimum_width', 'mirrored', 'name', 'net',
    'net_class', 'netclass', 'netname', 'not_in_schematic', 'number_box',
    'orientation', 'origin_x', 'origin_y', 'override_custom_rules', 'override_text',
    'pad_connections', 'pad_number', 'pad_shape', 'pad_to_die_delay',
    'pad_to_die_length', 'pad_type', 'parent', 'pin_name', 'pin_type', 'position_x',
    'position_y', 'precision', 'prefer_zone_connections', 'prefix', 'radius',
    'reference', 'remove_islands', 'rounded', 'row_height', 'row_separators', 'scale',
    'separators_style', 'separators_width', 'shape', 'show_text', 'size', 'size_x',
    'size_y', 'smoothing_amount', 'smoothing_effort', 'soldermask',
    'soldermask_margin_override', 'solderpaste_margin_override',
    'solderpaste_margin_ratio_override', 'source_name', 'source_type', 'start_x',
    'start_y', 'suffix', 'suppress_trailing_zeroes', 'target_delay', 'target_length',
    'target_skew', 'target_skew_delay', 'text', 'text_frame', 'text_size',
    'thermal_relief_gap', 'thermal_relief_spoke_angle', 'thermal_relief_spoke_width',
    'thermal_spoke_template', 'thickness', 'top_backdrill_size',
    'top_counterbore_depth', 'top_countersink_angle', 'transform_offset_x',
    'transform_offset_y', 'tuning_mode', 'type', 'units', 'units_format',
    'vertical_justification', 'via_type', 'visible', 'width', 'zone_connection_style'})
#: The units a number may carry (`1 mm` too). `1um`, `1nm`, `1cm`, `1MM` do not compile.
_KICAD_UNITS = frozenset({'mm', 'mil', 'in', 'deg', 'ps', 'fs'})

_KC_TOKEN = re.compile(r"""\s*(?:
    (?P<str>'[^']*(?:'|$)) |
    (?P<num>\d+(?:\.\d+)?(?![\d.])) |
    (?P<name>[A-Za-z_][A-Za-z0-9_]*) |
    (?P<op>==|!=|&&|\|\||<=|>=|[<>!()+\-*/.,])
    )""", re.VERBOSE)
_KC_COMPARE = ('==', '!=', '<', '>', '<=', '>=')


class _KiCadCompile:
    """KiCad's condition compiler as `kicad_condition_refusal` needs it: raises
    Unsupported naming the first thing it will not compile, and answers nothing
    about what a condition means. Its forms are the table in
    tests/test_1690_rule_readings.py, each measured on kicad-cli."""
    PLAIN, UNIT, OTHER = 'plain number', 'number with a unit', 'other'

    def __init__(self, src: str):
        self.toks, pos = [], 0
        while src[pos:].strip():
            m = _KC_TOKEN.match(src, pos)
            if not m:
                raise Unsupported(f"it cannot read {src[pos:].strip()[:20]!r}")
            self.toks.append((m.lastgroup, m.group(m.lastgroup)))
            pos = m.end()
        self.i = 0

    def _peek(self):
        return self.toks[self.i] if self.i < len(self.toks) else (None, None)

    def _next(self):
        t = self._peek()
        self.i += 1
        return t

    def _is_op(self, *ops) -> bool:
        k, v = self._peek()
        return k == 'op' and v in ops

    def run(self):
        if not self.toks:
            return
        kind = self._and()
        if self.i != len(self.toks):
            raise Unsupported(f"it does not expect {self._peek()[1]!r} there")
        if kind == self.PLAIN:
            raise Unsupported("a plain number is not a condition")

    def _pair(self, a, b, op):
        """A plain number compiles against another number, never anything else."""
        if self.PLAIN in (a, b) and not {a, b} <= {self.PLAIN, self.UNIT}:
            raise Unsupported(f"a plain number with no unit beside {op}: a string or a "
                              f"property is never compared with one (quote it, or give "
                              f"it a unit)")
        return self.OTHER

    def _chain(self, ops, down):
        a = down()
        while self._is_op(*ops):
            op = self._next()[1]
            a = self._pair(a, down(), op)
        return a

    def _and(self):
        return self._chain(('&&',), self._or)

    def _or(self):
        return self._chain(('||',), self._cmp)

    def _cmp(self):
        a = self._add()
        if self._is_op(*_KC_COMPARE):
            op = self._next()[1]
            a = self._pair(a, self._add(), op)
            if self._is_op(*_KC_COMPARE):
                raise Unsupported(f"{op} does not chain")
        return a

    def _add(self):
        return self._chain(('+', '-'), self._mul)

    def _mul(self):
        return self._chain(('*', '/'), self._unary)

    def _unary(self):
        if self._is_op('!'):
            self._next()
            if self._unary() == self.PLAIN:
                raise Unsupported("! on a plain number")
            return self.OTHER
        if self._is_op('-'):
            self._next()
            self._unary()
            return self.OTHER
        return self._atom()

    def _atom(self):
        k, v = self._next()
        if k == 'str':
            return self.OTHER
        if k == 'num':
            nk, nv = self._peek()
            if nk != 'name':
                return self.PLAIN
            self._next()
            if nv not in _KICAD_UNITS:
                raise Unsupported(f"{v}{nv}: {nv!r} is not a unit it knows")
            return self.UNIT
        if k == 'op' and v == '(':
            kind = self._and()
            if not self._is_op(')'):
                raise Unsupported("an unclosed parenthesis")
            self._next()
            return kind
        if k == 'name':
            return self._name(v)
        raise Unsupported("a term is missing" if k is None else f"it does not expect {v!r} there")

    def _name(self, item):
        if not self._is_op('.'):
            if item in ('A', 'B', 'AB', 'L'):
                return self.OTHER
            raise Unsupported(f"{item!r} is not a name it knows")
        self._next()
        k, name = self._next()
        if k != 'name':
            raise Unsupported(f"{item}. names nothing")
        if item not in ('A', 'B', 'AB'):
            raise Unsupported(f"{item}.{name}: it knows the items A, B and AB, spelled so")
        if self._is_op('('):
            self._next()
            return self._call(item, name)
        if item == 'AB':
            raise Unsupported(f"AB.{name}: AB has no properties")
        if name.lower() not in _KICAD_PROPERTIES:
            raise Unsupported(f"{item}.{name} is not a property it knows")
        return self.OTHER

    def _call(self, item, name):
        need = _KICAD_FUNCTIONS.get(name.lower())
        if need is None:
            raise Unsupported(f"{item}.{name}() is not a function it knows")
        strings = others = props = 0
        while not self._is_op(')'):
            k, v = self._peek()
            if k is None:
                raise Unsupported(f"{item}.{name}( is not closed")
            if k == 'op' and v == ',':
                self._next()
                continue
            if k == 'str':
                self._next()
                strings += 1
                continue
            props += k == 'name'
            others += 1
            if self._and() == self.PLAIN:
                raise Unsupported(f"{item}.{name}() with a plain number")
        self._next()
        if need and props and not strings:
            raise Unsupported(f"{item}.{name}() takes a quoted string")
        if strings + others < need:
            raise Unsupported(f"{item}.{name}() needs {need} argument(s)")
        return self.OTHER


def kicad_condition_refusal(src: str) -> Optional[str]:
    """Why KiCad cannot compile a `.kicad_dru` condition, or None. Measured on
    kicad-cli 10.0.6: a file holding such a condition, in any position, enforces
    NONE of its rules, with exit 0 and nothing printed."""
    try:
        _KiCadCompile(src).run()
    except Unsupported as e:
        return (f"{_KICAD_REFUSES} the condition {src!r} ({e}) and then enforces no "
                f"rule of this file")
    return None


#: What a rule's clauses may be, beside the condition. A file with any other
#: enforces none of its rules, the same way (kicad-cli 10.0.6, #1690).
_KICAD_CLAUSES = frozenset({'condition', 'constraint', 'layer', 'severity'})
_KICAD_SEVERITIES = frozenset({'error', 'warning', 'ignore', 'exclusion'})
_KICAD_VALUE = re.compile(r"-?\d*\.?\d+(?:mm|mil|in|deg|ps|fs)?$")
_KICAD_DISALLOW = DISALLOW_GRADED | {'pad', 'zone', 'text', 'graphic', 'hole', 'footprint'}
#: A layer clause spelled as copper that is no copper layer KiCad has
_KICAD_COPPER_LAYER = re.compile(r"(?:F|B|In\d+|\*)\.Cu$")


def _rule_refusal(node: list, rule: 'Rule') -> Optional[str]:
    """Why KiCad cannot load this rule of a parsed `.kicad_dru`, or None."""
    def refused(what):
        return f"{_KICAD_REFUSES} {what} and then enforces no rule of this file"
    for item in node[2:]:
        head = item[0] if isinstance(item, list) and item else item
        if head not in _KICAD_CLAUSES:
            return refused(f"the clause {head!r}")
        if head == 'severity' and len(item) > 1 and item[1] not in _KICAD_SEVERITIES:
            return refused(f"the severity {item[1]!r}")
        if head == 'layer' and len(item) > 1 and str(item[1]).endswith('.Cu') \
                and not _KICAD_COPPER_LAYER.match(str(item[1])):
            return refused(f"the layer {item[1]!r}")
        if head != 'constraint' or len(item) < 2:
            continue
        kind = canonical_kind(str(item[1]))
        if kind not in DRU_KINDS:
            return refused(f"the constraint {item[1]!r}")
        for sub in item[2:]:
            if kind == 'disallow' and not isinstance(sub, list) \
                    and str(sub).lower() not in _KICAD_DISALLOW:
                return refused(f"the disallow item {sub!r}")
            if not (isinstance(sub, list) and len(sub) == 2 and sub[0] in ('min', 'opt', 'max')):
                continue
            val = str(sub[1])
            # a count (via_count, min_resolved_spokes) takes a plain number
            if not _KICAD_VALUE.match(val) or (
                    kind in CLEARANCE_KINDS | SIZE_KINDS and val[-1].isdigit()):
                return refused(f"the value {val!r} of {item[1]} {sub[0]} (a length has "
                               f"a unit)")
    if rule.condition_src is not None:
        return kicad_condition_refusal(rule.condition_src)
    return None


def dru_refusals(text: str, rules: Sequence['Rule']) -> List[dict]:
    """[{'rule', 'reason'}] for each thing in a `.kicad_dru` that makes KiCad
    enforce none of its rules (#1690), [] when it loads the file. `rule` is ''
    for a fault of the file itself."""
    from kicad_dru import _tokenize, _parse_nodes
    out = []
    try:
        nodes = _parse_nodes(_tokenize(text))
    except Exception:                                          # noqa: BLE001
        nodes = []
    if not any(isinstance(n, list) and n and n[0] == 'version' for n in nodes):
        out.append({'rule': '', 'reason': f"{_KICAD_REFUSES} a file with no (version ...) "
                                          f"line and then enforces no rule of it"})
    for n in nodes:
        head = n[0] if isinstance(n, list) and n else n
        if head not in ('version', 'rule'):
            out.append({'rule': '', 'reason': f"{_KICAD_REFUSES} the top-level form {head!r} "
                                              f"and then enforces no rule of this file"})
        elif head == 'rule' and (len(n) < 2 or isinstance(n[1], list)):
            out.append({'rule': '?', 'reason': f"{_KICAD_REFUSES} a rule with no name and "
                                               f"then enforces no rule of this file"})
    out.extend({'rule': r.name, 'reason': r.kicad_refused} for r in rules if r.kicad_refused)
    return out


class _Parser:
    def __init__(self, src: str, areas: bool = False, unknown: bool = False,
                 courtyards: bool = False):
        self.toks = _tokens(src)
        self.i = 0
        self.areas = areas
        self.courtyards = courtyards
        # parse_condition_maybe: a construct outside the modelled subset
        # becomes an ('unknown',) leaf instead of raising
        self.unknown = unknown

    def _unmodelled(self, why: str):
        if self.unknown:
            return ('unknown',)
        raise Unsupported(why)

    def _skip_args(self):
        """Consume a call's argument list up to its closing ')' (the '(' is
        already taken)."""
        depth = 1
        while depth:
            k, v = self.peek()
            if k is None:
                raise Unsupported("unclosed argument list")
            self.i += 1
            if (k, v) == ('op', '('):
                depth += 1
            elif (k, v) == ('op', ')'):
                depth -= 1

    def peek(self):
        return self.toks[self.i] if self.i < len(self.toks) else (None, None)

    def take(self, kind=None, val=None):
        k, v = self.peek()
        if k is None or (kind and k != kind) or (val is not None and v != val):
            raise Unsupported(f"unexpected {v!r} (wanted {val or kind})")
        self.i += 1
        return v

    def parse(self):
        node = self.and_()
        if self.i != len(self.toks):
            raise Unsupported(f"trailing {self.peek()[1]!r}")
        return node

    def and_(self):
        node = self.or_()
        while self.peek() == ('op', '&&'):
            self.take()
            node = ('and', node, self.or_())
        return node

    def or_(self):
        node = self.not_()
        while self.peek() == ('op', '||'):
            self.take()
            node = ('or', node, self.not_())
        return node

    def not_(self):
        if self.peek() == ('op', '!'):
            self.take()
            return ('not', self.not_())
        return self.atom()

    def atom(self):
        if self.peek() == ('op', '('):
            self.take()
            node = self.and_()
            self.take('op', ')')
            return node
        return self.cmp()

    def cmp(self):
        lhs = self.term()
        k, v = self.peek()
        if k == 'op' and v in ('==', '!='):
            self.take()
            rhs = self.term()
            if 'unknown' in (lhs[0], rhs[0]):
                return ('unknown',)
            return ('cmp', v, lhs, rhs)
        if k == 'op' and v in _NUMERIC_OPS:
            if not self.unknown:
                raise Unsupported(f"numeric comparison/arithmetic ({v})")
            while self.peek()[0] == 'op' and self.peek()[1] in _NUMERIC_OPS + ('==', '!='):
                self.take()
                self.term()
            return ('unknown',)
        return lhs

    def term(self):
        k, v = self.peek()
        if k == 'str':
            self.take()
            # KiCad's lexer ends a literal at the end of the condition when it
            # has no closing quote, measured on kicad-cli 10.0.6: `A.Type == 'Via`
            # compiles and binds vias
            return ('lit', v[1:-1] if len(v) > 1 and v[-1] == v[0] else v[1:])
        if k == 'num':
            self.take()
            return self._unmodelled("numeric literal outside a supported comparison")
        if k == 'name':
            self.take()
            if '.' not in v:
                if v == 'L':
                    return ('L',)
                return self._unmodelled(f"bare identifier {v!r}")
            item, name = v.split('.', 1)
            item_u = item.upper()
            if item_u not in ('A', 'B', 'AB'):
                if self.unknown and self.peek() == ('op', '('):
                    self.take()
                    self._skip_args()
                return self._unmodelled(f"item {item!r}")
            lname = name.lower()
            if self.peek() == ('op', '('):
                self.take()
                args = []
                while self.peek() != ('op', ')'):
                    ak, av = self.peek()
                    if ak == 'str':
                        args.append(av[1:-1])
                        self.take()
                    elif ak == 'comma':
                        self.take()
                    elif self.unknown:
                        self._skip_args()
                        return ('unknown',)
                    else:
                        raise Unsupported(f"argument {av!r} to {name}()")
                self.take('op', ')')
                if lname == 'iscoupleddiffpair' and not args:
                    # KiCad's isCoupledDiffPairFunc reads the PAIR whichever
                    # item it is called on: A., B. and AB. are one predicate
                    # (measured on kicad-cli 10.0.8)
                    return ('coupled',)
                if self.areas and lname in _AREA_FUNCS:
                    if len(args) != 1:
                        raise Unsupported(f"{name}() with {len(args)} arguments")
                    return ('area', item_u, lname, args[0])
                if self.courtyards and lname in _COURTYARD_FUNCS and item_u != 'AB':
                    if len(args) != 1:
                        raise Unsupported(f"{name}() with {len(args)} arguments")
                    return ('courtyard', item_u, _COURTYARD_FUNCS[lname], args[0])
                if lname in _KNOWN_UNSUPPORTED_FUNCS or lname not in _SUPPORTED_FUNCS:
                    return self._unmodelled(f"predicate {name}() not modelled")
                if lname == 'indiffpair' and (item_u == 'AB' or len(args) != 1
                                              or not args[0]):
                    # KiCad reports "Missing diff-pair name argument" and the
                    # term is false; AB has no single net to test.
                    return self._unmodelled(f"{item}.{name}() needs one diff-pair "
                                            f"name pattern on A or B")
                return ('call', item_u, lname, tuple(args))
            if lname not in _SUPPORTED_PROPS:
                return self._unmodelled(f"property {name} not modelled")
            return ('prop', item_u, lname)
        raise Unsupported(f"unexpected token {v!r}")


def parse_condition(src: str, areas: bool = False, courtyards: bool = False):
    """AST for a rule condition, or raise Unsupported. With ``areas`` the
    rule-area predicates parse too, as ('area', item, fn, name) nodes that
    `_eval` answers from RuleItem.areas. Only net_keepouts asks for that, on
    a `disallow` rule: everywhere else an area predicate stays Unsupported,
    so a `!A.intersectsArea()` clearance rule is not read as matching
    every item on the board.

    With ``courtyards`` (#1805) the courtyard predicates parse too, as
    ('courtyard', item, side, pattern) nodes that `_eval` answers from
    RuleItem.courtyards, and raises Unsupported on an item whose courtyards
    were not measured (`Rule.courtyard_condition`).

    AB.isCoupledDiffPair() parses only as a term of an AND, positive (#1293,
    the form that gives a pair its own P<->N clearance) or negated (#1295,
    KiCad's "keep pairs apart" rule, which binds everything but the pair's
    own partner). OR'ed or compared it raises Unsupported, so such a rule
    stays unread rather than half-read."""
    try:
        node = _Parser(src, areas=areas, courtyards=courtyards).parse()
    except Unsupported as e:
        # a condition this reader does not read that KiCad refuses as well: the
        # second is the one a user has to act on (and "KiCad will still enforce
        # it", which report_lines says of the first, would not be so)
        raise Unsupported(kicad_condition_refusal(src) or str(e)) from None
    if _contains_coupled(node) and _coupled_terms(node) is None:
        raise Unsupported("AB.isCoupledDiffPair() is modelled only as a "
                          "term ANDed into the condition, or its negation")
    return node


def parse_condition_maybe(src: Optional[str]):
    """AST for ANY rule condition, for a three-valued reading (`_eval3`):
    the constructs `parse_condition` refuses (insideCourtyard(), getField(),
    numeric comparisons, ...) become ('unknown',) leaves, and a condition
    that does not even tokenize is ('unknown',) as a whole. None for an
    unconditioned rule."""
    if src is None:
        return None
    try:
        node = _Parser(src, unknown=True).parse()
    except Unsupported:
        return ('unknown',)
    # a refused condition's terms do not hold or fail one by one
    return ('unknown',) if _has_unknown(node) and kicad_condition_refusal(src) else node


def _eval3(node, a: RuleItem, b: Optional[RuleItem], layer: Optional[str],
           ctx=None) -> Optional[bool]:
    """Kleene's three-valued `_eval`: True, False, or None when an unknown
    leaf decides the answer (it may hold for some items and not others)."""
    op = node[0]
    if op == 'unknown':
        return None
    if op in ('and', 'or'):
        l, r = (_eval3(node[1], a, b, layer, ctx), _eval3(node[2], a, b, layer, ctx))
        if op == 'and':
            return False if False in (l, r) else (True if l and r else None)
        return True if True in (l, r) else (False if l is False and r is False else None)
    if op == 'not':
        v = _eval3(node[1], a, b, layer, ctx)
        return None if v is None else not v
    try:
        return bool(_eval(node, a, b, layer, ctx))
    except Unsupported:
        return None


def rule_may_match(rule: 'Rule', ast, a: RuleItem, b: Optional[RuleItem],
                   layer: Optional[str], copper_layers: Sequence[str],
                   ctx=None, layer_clause: bool = True) -> Optional[bool]:
    """`_rule_matches` in three values over a `parse_condition_maybe` AST:
    True when the rule binds the pair, False when it cannot, None when an
    unmodelled term decides it (or the pair's layer is not known and the rule
    names layers). Commutative, as KiCad's EvaluateFor is. ``layer_clause``
    False: the rule's `(layer ...)` is not read (`PadEdgeRules`); its condition
    still is, at ``layer``."""
    lay = rule.layers_for(copper_layers) if layer_clause else None
    unsure = False
    if lay is not None:
        if layer is None:
            if not lay:
                return False
            unsure = True
        elif layer not in lay:
            return False
    if ast is None:
        return None if unsure else True
    best = False
    for x, y in ((a, b), (b, a)) if b is not None else ((a, b),):
        v = _eval3(ast, x, y, layer, ctx)
        if v is True:
            return None if unsure else True
        if v is None:
            best = None
    return best


def _contains_coupled(node) -> bool:
    if not isinstance(node, tuple) or not node:
        return False
    if node[0] == 'coupled':
        return True
    return any(_contains_coupled(c) for c in node[1:] if isinstance(c, tuple))


def _coupled_terms(node):
    """(positive, negated): whether the condition ANDs in AB.isCoupledDiffPair()
    and whether it ANDs in its negation, or None when a coupled node sits
    anywhere else (under an OR, a comparison, a double negation)."""
    if node is None or not _contains_coupled(node):
        return False, False
    if node == ('coupled',):
        return True, False
    if node == ('not', ('coupled',)):
        return False, True
    if node[0] == 'and':
        pos = neg = False
        for c in node[1:]:
            t = _coupled_terms(c)
            if t is None:
                return None
            pos, neg = pos or t[0], neg or t[1]
        return pos, neg
    return None


def match_dp_suffix(name: str) -> Tuple[int, str, str]:
    """KiCad's DRC_ENGINE::MatchDpSuffix, ported exactly (#1293; identical in
    9.0 and master): (polarity, complement net name, base name), or
    (0, '', '') for a name with no pair suffix.

    Scanning from the end, digits and '_' are skipped and the first other
    character decides: '+'/'P' is the positive side, '-'/'N' the negative
    one, case-sensitive. The base drops the polarity character and every
    digit or '_' after it; the complement puts them back around the opposite
    character. 'DATA_P_1' -> (1, 'DATA_N_1', 'DATA_'); 'D+1' -> (1, 'D-1',
    'D'). No separator is required, so 'SPIN' is the N side of 'SPI'."""
    rv, count, comp = 0, 0, ''
    for ch in reversed(name):
        if '0' <= ch <= '9' or ch == '_':
            count += 1
            continue
        if ch == '+':
            comp, rv = '-', 1
        elif ch == '-':
            comp, rv = '+', -1
        elif ch == 'N':
            comp, rv = 'P', -1
        elif ch == 'P':
            comp, rv = 'N', 1
        break
    if rv == 0:
        return 0, '', ''
    count += 1
    base = name[:len(name) - count]
    return rv, base + comp + name[len(name) - count + 1:], base


#: Constraint kinds KiCad evaluates "only in the context of a diffpair" and so
#: for which AB.isCoupledDiffPair() holds with no B when the complement net
#: exists (isCoupledDiffPairFunc; master adds net_chain_length).
_DP_CONTEXT_KINDS = frozenset({'diff_pair_gap', 'length', 'skew',
                               'net_chain_length'})


def _is_coupled(a: Optional['RuleItem'], b: Optional['RuleItem'], ctx) -> bool:
    """KiCad's isCoupledDiffPairFunc: B's net is the complement of A's."""
    if a is None or not a.net_name:
        return False
    rv, comp, _base = match_dp_suffix(a.net_name)
    if not rv:
        return False
    kind, nets = ctx if ctx is not None else (None, frozenset())
    if kind in _DP_CONTEXT_KINDS and comp in nets:
        return True
    return b is not None and b.net_name == comp


def _wx_matches(pattern: str, text: str) -> bool:
    """wxString::Matches: '*' and '?' are the only wildcards, case-sensitive
    (fnmatch would also read '[...]' as a character class)."""
    rx = ''.join('.*' if ch == '*' else '.' if ch == '?' else re.escape(ch)
                 for ch in pattern)
    return re.fullmatch(rx, text, re.DOTALL) is not None


def _in_diff_pair(it: 'RuleItem', pattern: str, ctx) -> bool:
    """KiCad's inDiffPairFunc (#1295; identical in 9.0 and master): the
    item's net has a pair suffix (MatchDpSuffix), its complement net exists on
    the board, and the pair's base name matches ``pattern`` -- or, when the
    base ends in '_', the base before that last '_' does."""
    if not it.net_name:
        return False
    rv, comp, base = match_dp_suffix(it.net_name)
    nets = ctx[1] if ctx is not None else frozenset()
    if not rv or comp not in nets:
        return False
    if _wx_matches(pattern, base):
        return True
    return base.endswith('_') and _wx_matches(pattern, base[:base.rfind('_')])


#: The Edge.Cuts shape an edge clearance is measured to, as `_rule_matches`'
#: swapped A. Measured on kicad-cli 10.0.6 (tests/test_1660_pad_number_rule.py):
#: A.Type reads 'Graphic', A.Layer 'Edge.Cuts', A.NetName '', A.NetClass
#: 'Default', hasNetclass() and isPlated() false, the pad and via properties
#: undefined. So `!(A.Pad_Number == '1')` binds every item on the board, pad 1
#: included, and `A.Layer == 'Edge.Cuts'` does too.
_EDGE_ITEM = RuleItem(type='graphic', layers=frozenset({'Edge.Cuts'}))


class _Undefined:
    """A property the item does not have (KiCad's VT_UNDEFINED value)."""

    def __repr__(self):
        return 'UNDEFINED'


_UNDEFINED = _Undefined()


#: What `A.Pad_Type` and `A.Via_Type` read in KiCad, measured on kicad-cli 10.0.6
#: (#1690), for the board file's tokens. A via this module knows only as
#: 'blind_buried' reads as 'Blind'.
_PAD_TYPE_NAMES = {'smd': 'SMD', 'thru_hole': 'Through-hole', 'connect': 'Edge connector',
                   'np_thru_hole': 'NPTH, mechanical'}
_VIA_TYPE_NAMES = {'through': 'Through', 'micro': 'Micro', 'blind': 'Blind', 'buried': 'Buried',
                   'blind_buried': 'Blind'}


def _eval(node, a: RuleItem, b: Optional[RuleItem], layer: Optional[str],
          ctx=None):
    """``ctx`` is (constraint kind, the board's net names) -- what
    AB.isCoupledDiffPair() reads beyond A and B."""
    op = node[0]
    if op == 'or':
        return _eval(node[1], a, b, layer, ctx) or _eval(node[2], a, b, layer, ctx)
    if op == 'and':
        return _eval(node[1], a, b, layer, ctx) and _eval(node[2], a, b, layer, ctx)
    if op == 'not':
        return not _eval(node[1], a, b, layer, ctx)
    if op == 'coupled':
        return _is_coupled(a, b, ctx)
    if op == 'cmp':
        _, cop, lhs, rhs = node
        lv = _value(lhs, a, b, layer, ctx)
        rv = _value(rhs, a, b, layer, ctx)
        if lv is _UNDEFINED or rv is _UNDEFINED:
            # KiCad's VALUE::EqualTo and NotEqualTo are BOTH false when either
            # side is undefined (a property the item does not have)
            return False
        if lhs[0] == 'prop' and rhs[0] == 'prop' and lhs[2] == 'net' and rhs[2] == 'net':
            eq = (lv == rv)
        elif lhs[0] == 'prop' and lhs[2] == 'netclass' and rhs[0] == 'lit' \
                and isinstance(lv, str):
            # #1690, measured on kicad-cli 10.0.6: true when the literal matches
            # one class the net's effective class is built from, or the whole
            # name ('HS,LS'), as a NetName compare does (case-insensitive, '*'
            # and '?' wildcards)
            it = _item(lhs[1], a, b)
            eq = any(netname_matches(n, rv) for n in it.netclasses | {it.effective_class})
        elif isinstance(lv, str) and isinstance(rv, str):
            eq = _cmp_strings(lhs, lv, rv, rhs)
        else:
            eq = (lv == rv)
        return eq if cop == '==' else not eq
    if op == 'call':
        return bool(_call(node, a, b, layer, ctx))
    if op == 'area':
        it = _item(node[1], a, b)
        return it is not None and node[3] in it.areas
    if op == 'courtyard':
        it = _item(node[1], a, b)
        if it is None:
            return False
        if it.courtyards is None:
            raise Unsupported("courtyard not measured for this item")
        return any((node[2] is None or node[2] == side) and _footprint_named(node[3], ref, lib)
                   for ref, lib, side in it.courtyards)
    if op == 'prop':
        v = _value(node, a, b, layer)
        # a string alone is false in KiCad, whatever it holds: measured on
        # kicad-cli 10.0.6, a rule on bare A.Pad_Number, A.NetName, A.NetClass,
        # A.Type, A.Layer, A.Pad_Type or A.Via_Type binds nothing
        return v is not _UNDEFINED and not isinstance(v, str) and bool(v)
    if op == 'lit':
        return bool(node[1])
    if op == 'L':
        return bool(layer)
    raise Unsupported(f"node {op}")


def _cmp_strings(lhs_node, lv: str, rv: str, rhs_node=None) -> bool:
    prop = lhs_node[2] if lhs_node[0] == 'prop' else None
    if 'pad_number' in (prop, rhs_node[2] if rhs_node is not None
                        and rhs_node[0] == 'prop' else None):
        # #1660: the NetName compare, measured on kicad-cli 10.0.6. 'a1' binds
        # pad A1, 'A*' and '?1' bind it, '[A]1' does not, and only a literal
        # on the RIGHT is a wildcard ('A*' == A.Pad_Number does not bind A1).
        return netname_matches(lv, rv, wildcard=rhs_node is not None
                               and rhs_node[0] == 'lit')
    if prop in ('type', 'pad_type', 'via_type', 'netname') or (
            rhs_node is not None and rhs_node[0] == 'prop'
            and rhs_node[2] in ('type', 'pad_type', 'via_type')):
        # KiCad's string compare, as for a net name: case-insensitive, and a
        # literal on the right a wildcard ('T*' binds a through-hole pad)
        return netname_matches(lv, rv, wildcard=rhs_node is not None
                               and rhs_node[0] == 'lit')
    if 'netclass' in (prop, rhs_node[2] if rhs_node is not None
                      and rhs_node[0] == 'prop' else None):
        # #1690: not `A.NetClass == 'X'`, which `_eval` answers. A literal on
        # the LEFT is compared with the whole name, case-insensitive and no
        # wildcard, measured on kicad-cli 10.0.6: 'HS' == A.NetClass is false on
        # a net in HS and LS, 'H*' == A.NetClass false on a net in HS.
        return netname_matches(lv, rv, wildcard=False)
    if prop == 'layer':
        # KiCad accepts wildcards in layer names ('*.Cu').
        return fnmatch.fnmatchcase(lv, rv) if any(ch in rv for ch in '*?[') else lv == rv
    return lv == rv


def netname_matches(name: str, pattern: str, wildcard: bool = True) -> bool:
    """KiCad's string ``==`` for ``A.NetName == 'pattern'``: case-insensitive,
    and a literal carrying '*' or '?' is a wildcard (VALUE::EqualTo:
    WildCompareString / IsSameAs(.., false)). Measured on kicad-cli 10.0.6:
    ``A.NetName == '/clk'`` binds /CLK, ``A.NetName == '/CLK*'`` binds
    /CLK-, /CLK_N and /CLK+."""
    if wildcard and any(ch in pattern for ch in '*?'):
        return _wx_matches(pattern.lower(), name.lower())
    return name.lower() == pattern.lower()


def _item(which: str, a: RuleItem, b: Optional[RuleItem]) -> Optional[RuleItem]:
    if which == 'A':
        return a
    if which == 'B':
        return b
    return None


def _value(node, a, b, layer, ctx=None):
    op = node[0]
    if op == 'lit':
        return node[1]
    if op == 'L':
        return layer or ''
    if op == 'prop':
        it = _item(node[1], a, b)
        if it is None:
            # a term on a B there is not: `A.Net != B.Net` is false on one item
            # (kicad-cli 10.0.6, #1690), as `==` is
            return _UNDEFINED
        name = node[2]
        if name == 'netclass':
            return it.effective_class
        if name == 'netname':
            return it.net_name
        if name == 'net':
            return ('net', it.net_id)
        if name == 'type':
            return it.type
        if name == 'layer':
            if it.type == 'via':
                # PCB_VIA masks the Layer property (it has Layer Top/Bottom
                # instead), so KiCad reads A.Layer of a via as undefined:
                # measured on kicad-cli 10.0.6, a `disallow via` rule on
                # `A.Layer != 'F.Cu'` and one on `A.Layer == 'F.Cu'` both
                # flag no via
                return _UNDEFINED
            if it is _EDGE_ITEM:
                return 'Edge.Cuts'
            if layer:
                return layer
            return next(iter(it.layers)) if len(it.layers) == 1 else ''
        # A property only a pad or a via has is undefined on anything else,
        # measured on kicad-cli 10.0.6: `A.Pad_Type != 'SMD'` binds no track
        # or via, and `A.Pad_Number != '1'` none of them either.
        if name == 'pad_type':
            return (_PAD_TYPE_NAMES.get(it.pad_type, it.pad_type or '')
                    if it.type == 'pad' else _UNDEFINED)
        if name == 'via_type':
            return (_VIA_TYPE_NAMES.get(it.via_type, it.via_type or '')
                    if it.type == 'via' else _UNDEFINED)
        if name == 'pad_number':
            return (it.pad_number if it.type == 'pad' and it.pad_number is not None
                    else _UNDEFINED)
    if op == 'call':
        return _call(node, a, b, layer, ctx)
    raise Unsupported(f"value of {op}")


def _call(node, a, b, layer, ctx=None):
    """``ctx`` is _eval's (constraint kind, the board's net names), which
    inDiffPair() reads to find the complement net."""
    _, which, fn, args = node
    if which == 'AB':
        raise Unsupported(f"AB.{fn}")
    it = _item(which, a, b)
    if it is None:
        return False
    if it is _EDGE_ITEM and fn in ('hasnetclass', 'hasexactnetclass'):
        return False
    if fn == 'hasnetclass':
        # one class of the effective class, by wxString::Matches: 'H*' binds HS,
        # 'hs' does not, 'HS,LS' does not (kicad-cli 10.0.6, #1690)
        return bool(args) and any(_wx_matches(args[0], n)
                                  for n in it.netclasses or {it.effective_class})
    if fn == 'hasexactnetclass':
        # the whole name, spelled exactly: 'HS,LS', never 'HS' on a net in both
        return bool(args) and args[0] == it.effective_class
    if fn == 'existsonlayer':
        # by wxString::Matches, measured on kicad-cli 10.0.6 (#1690): 'In*.Cu'
        # binds an inner layer, 'B.cu' nothing, and a second argument makes it
        # false whatever the first
        on = it.layers | it.other_layers or ({layer} if layer else ())
        return len(args) == 1 and any(_wx_matches(args[0], n) for n in on)
    if fn == 'onlayer':     # KiCad refuses it: read as it always was
        return bool(args) and (args[0] in it.layers or
                               (layer is not None and args[0] == layer and not it.layers))
    if fn == 'ismicrovia':
        return it.type == 'via' and it.via_type == 'micro'
    if fn == 'isblindburiedvia':
        return it.type == 'via' and it.via_type in ('blind_buried', 'blind', 'buried')
    if fn == 'isplated':
        # #1690, measured on kicad-cli 10.0.6: true on a via of any type and a
        # through-hole pad (a castellated one too), false on an SMD or a
        # connector pad, an NPTH pad, a track, a zone and a graphic
        return it.type == 'via' or (it.type == 'pad' and it.pad_type == 'thru_hole')
    if fn == 'memberoffootprint':
        # the reference by wxString::Matches and, for a name with a ':' in it,
        # the library id too, measured on kicad-cli 10.0.6 (#1690): 'Device:*'
        # binds that library's parts, '*0402*' does not, 'a:b' binds a part
        # whose reference is spelled so. A second argument makes it false.
        return (len(args) == 1 and it.footprint_ref is not None
                and _footprint_named(args[0], it.footprint_ref, it.footprint_lib))
    if fn == 'memberofgroup':
        return bool(args) and args[0] in it.groups
    if fn == 'indiffpair':
        return bool(args) and _in_diff_pair(it, args[0], ctx)
    raise Unsupported(f"predicate {fn}()")


def _footprint_named(pattern: str, ref: Optional[str], lib: Optional[str]) -> bool:
    """Does a memberOfFootprint() / courtyard argument name this part: its
    reference by wxString::Matches and, for a name with a ':' in it, its
    library id too (#1690)."""
    return ref is not None and (_wx_matches(pattern, ref) or (
        ':' in pattern and lib is not None and _wx_matches(pattern, lib)))


#: #1805: the first kicad-cli release that tests a `via_diameter` rule's
#: (layer ...) clause for a via (against F.Cu, as every release measured tests
#: an `annular_width` rule's). 10.0.6 does not test it, as no release measured
#: tests a `hole_size` rule's. Measured on 10.0.6 and 10.0.7 only.
_VIA_DIAMETER_CLAUSE_FROM = '10.0.7'


#: Set only inside `DesignRules.resolve_for_copper`: the via_diameter
#: reading to resolve under, in place of the installed release's.
_via_diameter_reading: Optional[bool] = None


def _via_clause_tested(kind: str) -> bool:
    """Whether KiCad tests the (layer ...) clause of a ``kind`` rule
    (`VIA_SIZE_KINDS`) for a via, as the release the tool reproduces
    (`net_queries.length_measure_release`: the installed kicad-cli's, the
    newest modelled without one) grades it. That is the GRADERS' reading;
    copper reads every release's at once (`DesignRules.resolve_for_copper`)."""
    if kind == 'annular_width':
        return True
    if kind != 'via_diameter':
        return False
    if _via_diameter_reading is not None:
        return _via_diameter_reading
    from net_queries import length_measure_release, _release_key
    return _release_key(length_measure_release()) >= _release_key(_VIA_DIAMETER_CLAUSE_FROM)


def _rule_matches(rule: Rule, a: RuleItem, b: Optional[RuleItem],
                  layer: Optional[str], copper_layers: Sequence[str],
                  ctx=None, condition=None) -> bool:
    """``condition``: the AST to read instead of ``rule.condition`` (a
    `courtyard_condition`).

    A rule with a (layer ...) clause binds on the layers it names, so not
    without ``layer`` -- except a via-size rule on a via (#1805), read as
    kicad-cli grades a via, through, blind, buried and micro alike: where the
    clause is tested (`_via_clause_tested`) it is tested against F.Cu, whatever
    layers the via spans and whichever layer is asked about. Every reader of a
    via's size -- check_drc's grade, `via_floors`, the step grades,
    `draw_size`, `rule_floors` -- so binds the vias KiCad binds."""
    lay = rule.layers_for(copper_layers)
    if lay is not None:
        kind = ctx[0] if ctx else None
        if a.type == 'via' and kind in VIA_SIZE_KINDS:
            if 'F.Cu' not in lay and _via_clause_tested(kind):
                return False
        elif layer is None or layer not in lay:
            return False
    condition = rule.condition if condition is None else condition
    if condition is None:
        return True
    # KiCad's DRC_RULE_CONDITION::EvaluateFor: "Conditions are commutative" --
    # a two-item condition that fails as (A, B) is tried again as (B, A).
    for x, y in ((a, b), (b, a)) if b is not None else ((a, b),):
        try:
            if _eval(condition, x, y, layer, ctx):
                return True
        except Unsupported:
            pass
    return False


# --------------------------------------------------------------------------
# Net-class membership as the .kicad_pro declares it
# --------------------------------------------------------------------------

# The newest net_settings schema whose ``classes[].nets`` lists are
# membership. KiCad 6 kept a class's members inside the class; KiCad 7's
# schema 2 -> 3 migration turns each name into a ``netclass_patterns`` entry,
# and it runs on every load of a file still at schema 0-2. Probed on pcbnew
# 10.0.0 with steppenprobe's KiCad-6 project (#1304): meta.version 0, 1 or 2
# resolves the listed nets to their class, 3 and 4 leave them Default (the key
# is no longer read), and a file with no ``meta`` loads no net_settings at all.
KICAD6_MEMBERS_MAX_SCHEMA = 2


def project_class_membership(net_settings) -> Tuple[Dict[str, List[str]],
                                                      List[Tuple[str, str]]]:
    """Net-class MEMBERSHIP from a .kicad_pro ``net_settings`` dict, in every
    form KiCad reads -- the one parser both project readers
    (``list_nets.read_design_rules`` and ``DesignRules.from_project``, so the
    router, the graders, the placer and the GUI's design-rule table) go
    through, so none of them can miss a form the others honour.

    Returns ``(assignments, patterns)``:

    * ``assignments`` -- {net name: [class, ...]} in file order: the explicit
      ``netclass_assignments`` (KiCad 7+; a value is one name or, KiCad 9+, a
      list), then the KiCad-6 ``classes[].nets`` lists when the schema is at
      most ``KICAD6_MEMBERS_MAX_SCHEMA`` (#1304). Those are exact net names:
      KiCad's migration makes each a pattern, and a pattern spelled as a whole
      net name matches that net.
    * ``patterns`` -- [(glob, class)] from ``netclass_patterns``, file order.
    """
    ns = net_settings if isinstance(net_settings, dict) else {}
    assignments: Dict[str, List[str]] = {}

    def _add(net, cls):
        if not net or not cls:
            return
        lst = assignments.setdefault(str(net), [])
        if cls not in lst:
            lst.append(str(cls))

    na = ns.get('netclass_assignments') or {}
    if isinstance(na, dict):
        for net, cl in na.items():
            for c in (cl if isinstance(cl, list) else [cl]):
                _add(net, c)
    meta = ns.get('meta')
    version = meta.get('version') if isinstance(meta, dict) else None
    if (isinstance(version, int) and not isinstance(version, bool)
            and version <= KICAD6_MEMBERS_MAX_SCHEMA):
        for c in ns.get('classes') or []:
            if not isinstance(c, dict):
                continue
            members = c.get('nets')
            if isinstance(members, list):
                for net in members:
                    if isinstance(net, str):
                        _add(net, c.get('name') or 'Default')
    patterns: List[Tuple[str, str]] = []
    for pe in ns.get('netclass_patterns') or []:
        try:
            if pe.get('netclass'):
                patterns.append((pe.get('pattern', ''), pe['netclass']))
        except (AttributeError, TypeError):
            continue
    return assignments, patterns


# --------------------------------------------------------------------------
# The resolver
# --------------------------------------------------------------------------

@dataclass
class DesignRules:
    board_min: Dict[str, float] = field(default_factory=dict)      # rules.min_* (only >0)
    classes: Dict[str, dict] = field(default_factory=dict)        # name -> {clearance, track_width, ..., priority}
    memberships: Dict[int, FrozenSet[str]] = field(default_factory=dict)  # net_id -> classes
    # footprint key -> (its reference as the file spells it, its library id)
    footprint_ids: Dict[str, Tuple[str, str]] = field(default_factory=dict)
    # class name -> the fields its project entry leaves unset (`class_constituents`)
    unset_fields: Dict[str, FrozenSet[str]] = field(default_factory=dict)
    net_names: Dict[int, str] = field(default_factory=dict)
    rules: List[Rule] = field(default_factory=list)
    copper_layers: List[str] = field(default_factory=list)
    fab_floor: Dict[str, float] = field(default_factory=dict)      # fab_tiers FLOOR_KEYS dict, or {}
    cli: Dict[str, float] = field(default_factory=dict)           # explicit CLI sizes by kind
    groups: Dict[str, FrozenSet[str]] = field(default_factory=dict)  # footprint ref -> group names
    source: str = ''
    dru_path: str = ''
    notes: List[str] = field(default_factory=list)
    # `dru_refusals`: why KiCad enforces no rule of this board's .kicad_dru
    kicad_refused: List[dict] = field(default_factory=list)
    # net_keepouts.derive_area_keepouts' regions (install_design_rules)
    net_keepouts: List[dict] = field(default_factory=list)
    # the project's rule_severities: {KiCad error key: severity}
    rule_severities: Dict[str, str] = field(default_factory=dict)
    _cache: dict = field(default_factory=dict, repr=False)
    _effective: Dict[int, str] = field(default_factory=dict, repr=False)

    # ---------------- construction ----------------

    @classmethod
    def from_project(cls, pcb_data, board_path: Optional[str] = None, *,
                     fab_floor: Optional[dict] = None, cli: Optional[dict] = None,
                     copper_layers: Optional[Sequence[str]] = None) -> "DesignRules":
        """Build from the sibling .kicad_pro / .kicad_dru of ``board_path`` (or
        ``pcb_data.source_path``). A board with neither still gets a table
        (empty minimums, a Default class from routing_defaults is NOT invented
        -- absent means absent)."""
        path = board_path or getattr(pcb_data, 'source_path', '') or ''
        dr = cls()
        dr.source = path
        dr.copper_layers = list(copper_layers or
                                getattr(getattr(pcb_data, 'board_info', None),
                                        'copper_layers', None) or [])
        dr.fab_floor = dict(fab_floor or {})
        dr.cli = {canonical_kind(k): float(v) for k, v in (cli or {}).items()
                  if v is not None}
        dr.net_names = {nid: n.name for nid, n in (getattr(pcb_data, 'nets', None) or {}).items()
                        if getattr(n, 'name', None)}
        dr.groups = _groups_by_ref(pcb_data)
        dr.footprint_ids = _footprint_ids(pcb_data)
        if path:
            dr._load_project_file(os.path.splitext(path)[0] + '.kicad_pro', path)
            dru = os.path.splitext(path)[0] + '.kicad_dru'
            if os.path.isfile(dru):
                dr.dru_path = dru
                try:
                    text = open(dru, encoding='utf-8').read()
                except OSError as e:
                    dr.notes.append(f"could not read {dru}: {e}")
                else:
                    dr.rules, notes = parse_dru(text)
                    dr.notes.extend(notes)
                    dr.kicad_refused = dru_refusals(text, dr.rules)
        dr._finish()
        return dr

    @classmethod
    def from_pcbnew(cls, board, pcb_data, *, fab_floor: Optional[dict] = None,
                    cli: Optional[dict] = None) -> "DesignRules":
        """Build from a live pcbnew BOARD (GUI). Minimums and classes come from
        BOARD_DESIGN_SETTINGS so unsaved edits count; pattern/label assignments
        and the .kicad_dru come from the files beside board.GetFileName()
        (the SWIG API exposes neither the pattern text nor the rules)."""
        path = ''
        try:
            path = board.GetFileName() or ''
        except Exception:                                      # noqa: BLE001
            path = ''
        dr = cls.from_project(pcb_data, path or None, fab_floor=fab_floor, cli=cli)
        try:
            bds = board.GetDesignSettings()
        except Exception:                                      # noqa: BLE001
            dr.notes.append('pcbnew: no design settings; file values used')
            return dr
        live_min = {}
        for key, attr in (('min_clearance', 'm_MinClearance'),
                          ('min_track_width', 'm_TrackMinWidth'),
                          ('min_via_diameter', 'm_ViasMinSize'),
                          ('min_through_hole_diameter', 'm_MinThroughDrill'),
                          ('min_via_annular_width', 'm_ViasMinAnnularWidth'),
                          ('min_hole_to_hole', 'm_HoleToHoleMin'),
                          ('min_hole_clearance', 'm_HoleClearance'),
                          ('min_copper_edge_clearance', 'm_CopperEdgeClearance'),
                          ('min_connection', 'm_MinConn')):
            v = getattr(bds, attr, None)
            if isinstance(v, (int, float)) and v > 0:
                live_min[key] = v / 1e6           # divide, never multiply (#493)
        if live_min:
            dr.board_min = live_min
        # Net classes live: default + non-default (KiCad 10's GetNetclasses
        # omits Default, #782).
        try:
            ns = bds.m_NetSettings
            live_classes = {}
            dflt = ns.GetDefaultNetclass()
            if dflt is not None:
                live_classes['Default'] = _netclass_dict(dflt)
            others = ns.GetNetclasses()
            for name in others.keys():
                nc = others[name]
                if nc is not None and name != 'Default':
                    live_classes[str(name)] = _netclass_dict(nc)
            if live_classes:
                # keep file-only fields (priority when the API lacks it)
                for name, d in live_classes.items():
                    old = dr.classes.get(name, {})
                    merged = dict(old)
                    merged.update({k: v for k, v in d.items() if v is not None})
                    dr.classes[name] = merged
        except Exception as e:                                 # noqa: BLE001
            dr.notes.append(f'pcbnew: net classes from file ({e})')
        dr._cache.clear()
        dr._effective.clear()
        dr._finish()
        return dr

    def _load_project_file(self, pro_path: str, pcb_path: str) -> None:
        proj = None
        if os.path.isfile(pro_path):
            try:
                with open(pro_path, encoding='utf-8') as f:
                    proj = json.load(f)
            except (OSError, json.JSONDecodeError) as e:
                self.notes.append(f"could not read {pro_path}: {e}")
        assignments: Dict[str, List[str]] = {}
        patterns: List[Tuple[str, str]] = []
        if proj is not None:
            ns = proj.get('net_settings', {}) or {}
            for c in ns.get('classes', []) or []:
                name = c.get('name', 'Default')
                d = {}
                for k in ('clearance', 'track_width', 'via_diameter', 'via_drill',
                          'diff_pair_gap', 'diff_pair_width', 'diff_pair_via_gap',
                          'microvia_diameter', 'microvia_drill'):
                    if isinstance(c.get(k), (int, float)):
                        d[k] = float(c[k])
                d['priority'] = int(c.get('priority', _DEFAULT_PRIORITY
                                          if name == 'Default' else 0))
                self.classes[name] = d
                self.unset_fields[name] = frozenset(
                    k for k in _CLASS_COMPLETE_FIELDS
                    if not isinstance(c.get(k), (int, float)))
            # every membership form, KiCad 6's classes[].nets included (#1304)
            assignments, patterns = project_class_membership(ns)
            ds = (proj.get('board', {}) or {}).get('design_settings', {}) or {}
            rules = ds.get('rules', {}) or {}
            for k, v in rules.items():
                if k.startswith('min_') and isinstance(v, (int, float)) and v > 0:
                    self.board_min[k] = float(v)
            self.rule_severities = {str(k): str(v) for k, v in
                                    (ds.get('rule_severities') or {}).items()}
        if not self.classes:
            # KiCad 6/7: (net_class ...) blocks in the board file.
            try:
                from list_nets import read_design_rules
                legacy = read_design_rules(pcb_path)
                for name, d in (legacy.get('classes') or {}).items():
                    dd = {k: float(v) for k, v in d.items() if isinstance(v, (int, float))}
                    dd['priority'] = _DEFAULT_PRIORITY if name == 'Default' else 0
                    self.classes[name] = dd
                for net, cl in (legacy.get('assignments') or {}).items():
                    assignments[net] = [cl]
            except Exception:                                  # noqa: BLE001
                pass
        # memberships: explicit assignment UNION every matching pattern (KiCad
        # merges), the same rule list_nets.net_class_memberships applies.
        for nid, name in self.net_names.items():
            cand = set(assignments.get(name, []))
            for pat, cname in patterns:
                if not pat:
                    continue
                try:
                    if fnmatch.fnmatchcase(name, pat):
                        cand.add(cname)
                except Exception:                              # noqa: BLE001
                    pass
            cand = {c for c in cand if c in self.classes or c == 'Default'}
            if cand:
                self.memberships[nid] = frozenset(cand)

    def _finish(self) -> None:
        if 'Default' not in self.classes:
            self.classes['Default'] = {'priority': _DEFAULT_PRIORITY}
        self.classes['Default'].setdefault('priority', _DEFAULT_PRIORITY)
        self._cache.clear()
        self._effective.clear()

    # ---------------- net-class aggregate ----------------

    def effective_class(self, net_id: int) -> str:
        """The aggregate class NAME for a net: the highest-priority member
        class (Default when it belongs to none)."""
        e = self._effective.get(net_id)
        if e is None:
            members = [c for c in self.memberships.get(net_id, ()) if c in self.classes]
            members.sort(key=lambda c: (self.classes[c].get('priority', 0), c))
            e = members[0] if members else 'Default'
            self._effective[net_id] = e
        return e

    def class_constituents(self, net_id: int) -> Tuple[str, ...]:
        """The classes KiCad builds a net's effective class from, in the order
        its name joins them, which is what a rule condition reads (#1690):
        priority, then name, then Default when the net's classes leave one of
        `_CLASS_COMPLETE_FIELDS` unset between them, since Default then
        supplies it. So on a net whose one class sets no bus width,
        `A.NetClass == 'Default'` and `A.hasNetclass('Default')` are true,
        `A.hasExactNetclass('HS')` false and `A.hasExactNetclass('HS,Default')`
        true. Measured on kicad-cli 10.0.6, the same at every project schema
        0 to 5. A net assigned to a class the project does not declare is
        not modelled: `memberships` drops that class."""
        members = [c for c in self.memberships.get(net_id, ()) if c in self.classes]
        if not members:
            return ('Default',)
        members.sort(key=lambda c: (self.classes[c].get('priority', 0), c))
        unset = frozenset.intersection(*(self.unset_fields.get(c, frozenset())
                                         for c in members))
        if unset and 'Default' not in members:
            members.append('Default')
        return tuple(members)

    def class_value(self, net_id: int, prop: str) -> Optional[float]:
        """``prop`` from the aggregate class: the highest-priority member class
        that SETS it, else Default's, else None."""
        members = [c for c in self.memberships.get(net_id, ()) if c in self.classes]
        members.sort(key=lambda c: (self.classes[c].get('priority', 0), c))
        for c in members + ['Default']:
            v = self.classes.get(c, {}).get(prop)
            if isinstance(v, (int, float)) and v > 0:
                return float(v)
        return None

    def item_for_net(self, net_id: int, type: str = 'track',
                     layer: Optional[str] = None, **facts) -> RuleItem:
        """A RuleItem for a net with the class facts filled in."""
        layers = facts.pop('layers', None)
        if layers is None and type == 'via' and facts.get('via_type') in (None, 'through'):
            # for existsOnLayer(): a through via is on every copper layer of the
            # board, whichever one is asked about
            facts['other_layers'] = frozenset(self.copper_layers)
        if layers is None:
            layers = frozenset([layer]) if layer else frozenset()
        names = self.class_constituents(net_id)
        return RuleItem(type=type, net_id=net_id,
                        net_name=self.net_names.get(net_id, ''),
                        netclasses=frozenset(names),
                        effective_class=','.join(names),
                        layers=frozenset(layers), **facts)

    def item_for_pad(self, pad, layer: Optional[str] = None) -> RuleItem:
        ref = getattr(pad, 'component_ref', None)
        names = self.class_constituents(getattr(pad, 'net_id', 0))
        fp_ref, fp_lib = self.footprint_ids.get(ref, (ref, None))
        layers = _pad_rule_layers(getattr(pad, 'layers', None))
        return RuleItem(
            type='pad', net_id=int(getattr(pad, 'net_id', 0) or 0),
            net_name=getattr(pad, 'net_name', '') or self.net_names.get(getattr(pad, 'net_id', 0), ''),
            netclasses=frozenset(names),
            effective_class=','.join(names),
            layers=frozenset(l for l in layers if l.endswith('.Cu')),
            other_layers=frozenset(l for l in layers if not l.endswith('.Cu')),
            footprint_ref=fp_ref, footprint_lib=fp_lib, pad_type=getattr(pad, 'pad_type', None),
            pad_number=getattr(pad, 'pad_number', None),
            groups=self.groups.get(ref, frozenset()),
            xy=(getattr(pad, 'global_x', None), getattr(pad, 'global_y', None))
            if getattr(pad, 'global_x', None) is not None else None,
            clearance_override=(getattr(pad, 'local_clearance', 0) or None))

    # ---------------- resolution ----------------

    def resolve(self, kind: str, a: RuleItem, b: Optional[RuleItem] = None,
                layer: Optional[str] = None) -> Constraint:
        kind = canonical_kind(kind)
        key = (kind, a, b, layer, _via_diameter_reading if kind == 'via_diameter' else None)
        c = self._cache.get(key)
        if c is None:
            c = self._resolve(kind, a, b, layer)
            self._cache[key] = c
        return c

    def resolve_for_copper(self, kind: str, a: RuleItem, b: Optional[RuleItem] = None,
                           layer: Optional[str] = None) -> Constraint:
        """`resolve` for what decides COPPER, which must not depend on the
        installed kicad-cli: a via's `via_diameter` is resolved under every
        modelled release's reading of a (layer ...) clause (`_via_clause_tested`)
        and the strictest minimum kept, so a via sized by it passes on each
        release, and a relaxation one release does not grant is not taken.
        Every other kind and item reads the same on every release."""
        global _via_diameter_reading
        kind = canonical_kind(kind)
        if kind != 'via_diameter' or a.type != 'via':
            return self.resolve(kind, a, b, layer)
        found = []
        try:
            for tested in (False, True):
                _via_diameter_reading = tested
                found.append(self.resolve(kind, a, b, layer))
        finally:
            _via_diameter_reading = None
        return max(found, key=lambda c: (c.min or 0.0,
                                         (c.kicad_min if c.fab_bound else c.min) or 0.0,
                                         c.min_rule is not None))

    def _resolve(self, kind: str, a: RuleItem, b: Optional[RuleItem],
                 layer: Optional[str]) -> Constraint:
        cur: Dict[str, Optional[float]] = {'min': None, 'opt': None, 'max': None}
        source = 'none'
        board_min_clr = self.board_min.get('min_clearance')

        # 1. pad / footprint override: KiCad returns before any rule.
        if kind in ('clearance', 'hole_clearance'):
            ovs = [x.clearance_override for x in (a, b)
                   if x is not None and x.clearance_override is not None]
            if ovs:
                v = max(ovs)
                floor_key = 'min_clearance' if kind == 'clearance' else 'min_hole_clearance'
                bm = self.board_min.get(floor_key)
                src = 'pad override'
                if bm is not None and v < bm:
                    v, src = bm, 'board minimum'
                return self._fab(kind, Constraint(kind, min=v, source=src, kicad_min=v))

        # 2a. board minimum -- the first rule in KiCad's vector.
        bkey = BOARD_MIN_KEYS.get(kind)
        if bkey and self.board_min.get(bkey):
            cur['min'] = self.board_min[bkey]
            source = 'board minimum'

        # 2b. net-class implicit rules.
        implicit_rule = None
        if kind == 'clearance':
            vals = [self.class_value(x.net_id, 'clearance') for x in (a, b) if x is not None]
            vals = [v for v in vals if v is not None]
            if vals:
                v = max(vals)
                cur['min'] = v
                names = sorted({self.effective_class(x.net_id) for x in (a, b) if x is not None})
                source = 'netclass ' + '/'.join(names)
            dp = self.pair_gap_clearance(a, b)
            if dp is not None:
                cur['min'], implicit_rule = dp
                source = implicit_rule
        elif kind in NETCLASS_OPT_KEYS:
            v = self.class_value(a.net_id, NETCLASS_OPT_KEYS[kind])
            # KiCad's "netclass (diff pair)" implicit rule, conditioned on
            # A.inDiffPair('*'): a coupled-pair member's track is drawn at the
            # class diff_pair_width, and custom rules below still override it.
            if kind == 'track_width' and a.diff_pair:
                dv = self.class_value(a.net_id, 'diff_pair_width')
                if dv is not None:
                    v = dv
            if v is not None:
                cur['opt'] = v
                source = f'netclass {self.effective_class(a.net_id)}'

        # 2c. custom rules, file order, per field, last wins.
        disallow: set = set()
        min_from_rule = False
        min_rule = None
        ctx = (kind, self._net_name_set())
        for rule in self.rules:
            if kind not in rule.constraints:
                continue
            cond = None
            if rule.unsupported:
                # #1805: a courtyard rule binds only an item measured by position
                cond = rule.courtyard_condition
                if cond is None or a.courtyards is None or (
                        b is not None and b.courtyards is None):
                    continue
            if not _rule_matches(rule, a, b, layer, self.copper_layers, ctx, cond):
                continue
            spec = rule.constraints[kind]
            if kind == 'disallow':
                disallow |= set(spec.get('disallow', ()))
                source = f'rule "{rule.name}"'
                continue
            for f in ('min', 'opt', 'max'):
                if f in spec:
                    cur[f] = spec[f]
                    if f == 'min':
                        min_from_rule = True
                        min_rule = rule
            source = f'rule "{rule.name}"'

        if kind == 'disallow':
            return Constraint(kind, source=source, disallow=frozenset(disallow))

        # 3. zone local clearance: max with the rule result.
        if kind == 'clearance':
            for x in (a, b):
                if x is not None and x.local_clearance is not None:
                    if cur['min'] is None or x.local_clearance > cur['min']:
                        cur['min'] = x.local_clearance
                        source = 'zone clearance'
                        min_from_rule = False
                        min_rule = None
                        implicit_rule = None

        # 4. the ONLY post-loop board floors KiCad applies -- and MEASURED on
        # KiCad 10.0.0 (tests/oracle/constraint_agreement.py rows
        # board_min_clearance_floors_class / _vs_rule / pad_override_below_
        # board_min): rules.min_clearance floors a net-class value and a pad
        # override, but an EXPLICIT custom rule below it wins. (The 9.0
        # source reads as unconditional; the installed engine is the spec.)
        if kind in ('clearance', 'diff_pair_gap') and board_min_clr and not min_from_rule:
            if cur['min'] is None or cur['min'] < board_min_clr:
                cur['min'] = board_min_clr
                source = 'board minimum'

        if min_rule is not None:
            rule_name, rule_coupled = min_rule.name, bool(min_rule.coupled)
        elif implicit_rule is not None:
            # the class "(diff pair)" rule still sets the value (floored at
            # the board minimum, as KiCad floors it): no custom rule or zone
            # clearance replaced it
            rule_name, rule_coupled = implicit_rule, True
        else:
            rule_name, rule_coupled = None, False
        c = Constraint(kind, min=cur['min'], opt=cur['opt'], max=cur['max'],
                       source=source, kicad_min=cur['min'],
                       min_rule=rule_name, min_rule_coupled=rule_coupled)
        return self._fab(kind, c)

    def pair_gap_clearance(self, a: Optional[RuleItem], b: Optional[RuleItem]
                           ) -> Optional[Tuple[float, str]]:
        """KiCad's implicit "netclass '<class>' (diff pair)" clearance rule
        (#1542): (the class diff_pair_gap, the rule's name) when ``a`` and
        ``b`` are the two nets of one pair (MatchDpSuffix on the names) and
        the class of either has ``0 < diff_pair_gap < clearance``, else None.

        KiCad adds it per net class as ``A.NetClass == '<class>' &&
        AB.isCoupledDiffPair()`` with the gap as the clearance minimum, after
        the class clearance rules and before every custom rule, for every item
        kind. Measured on kicad-cli 10.0.6: a pair class at clearance 0.25 with
        gap 0.15 grades 0.15 copper clean, at gap 0.20 it reports
        "netclass 'HS' (diff pair) clearance 0.2000 mm", a class that sets no
        gap of its own takes Default's (``class_value``), and the board
        minimum clearance still floors it (step 4). Where both classes carry
        one, the wider is taken."""
        if a is None or b is None or not a.net_name or not b.net_name:
            return None
        if a.net_id == b.net_id or not _is_coupled(a, b, None):
            return None
        best = None
        for x in (a, b):
            gap = self.class_value(x.net_id, 'diff_pair_gap')
            clr = self.class_value(x.net_id, 'clearance')
            if gap is None or clr is None or gap >= clr - 1e-9:
                continue
            if best is None or gap > best[0]:
                best = (gap, f"netclass '{self.effective_class(x.net_id)}' (diff pair)")
        return best

    def has_pair_gap_classes(self) -> bool:
        """A net class (aggregate, as ``class_value`` reads it) whose
        diff_pair_gap is below its clearance: KiCad's implicit "(diff pair)"
        rule (#1542) can then bind some pair's P against its N."""
        h = self._cache.get('_has_pair_gap_classes')
        if h is None:
            nids = set(self.memberships) | {0}
            h = False
            for nid in nids:
                gap = self.class_value(nid, 'diff_pair_gap')
                clr = self.class_value(nid, 'clearance')
                if gap is not None and clr is not None and gap < clr - 1e-9:
                    h = True
                    break
            self._cache['_has_pair_gap_classes'] = h
        return h

    def _fab(self, kind: str, c: Constraint) -> Constraint:
        """5. this tool's fab-profile floor, raise-only and disclosed."""
        fk = FAB_FLOOR_KEYS.get(kind)
        if not fk or not self.fab_floor:
            return c
        fv = self.fab_floor.get(fk)
        if fv is None:
            return c
        if c.min is None or c.min < fv - 1e-9:
            return replace(c, min=fv, source=f'fab floor (over {c.source})',
                           fab_bound=True, kicad_min=c.min)
        return c

    def _net_name_set(self) -> FrozenSet[str]:
        s = self._cache.get('_net_name_set')
        if s is None:
            s = self._cache['_net_name_set'] = frozenset(self.net_names.values())
        return s

    def has_coupled_rules(self) -> bool:
        """A supported rule ANDs in AB.isCoupledDiffPair() (#1293)."""
        return any(r.coupled and not r.unsupported for r in self.rules)

    def coupled_pair_clearance(self, p_net_id: int, n_net_id: int,
                               layer: Optional[str],
                               types: Tuple[str, str] = ('track', 'track')
                               ) -> Optional[Constraint]:
        """The clearance KiCad grades a diff pair's P against its own N at on
        ``layer``, when the rule that sets it is a coupled-pair rule
        (``AB.isCoupledDiffPair()``, #1293); None when a class, an override,
        the board minimum or any other rule sets it -- the caller then keeps
        its own value. ``types`` are the two items' KiCad types (a rule may
        scope itself with ``A.Type == 'track'``). ``min`` carries the fab
        floor, ``kicad_min`` what KiCad accepts."""
        if not (self.has_coupled_rules() or self.has_pair_gap_classes()):
            return None
        a = self.item_for_net(p_net_id, types[0], layer)
        b = self.item_for_net(n_net_id, types[1], layer)
        c = self.resolve('clearance', a, b, layer)
        return c if c.min_rule_coupled and c.min is not None else None

    def resolve_stack(self, kind: str, a: RuleItem, b: Optional[RuleItem],
                      layers: Iterable[str]) -> Constraint:
        """A stack-spanning pair (via barrel, TH drill) binds on every layer
        both coppers exist on: the strictest layer wins."""
        best = None
        for l in layers:
            c = self.resolve(kind, a, b, l)
            if best is None or (c.min or 0) > (best.min or 0):
                best = c
        return best if best is not None else self.resolve(kind, a, b, None)

    def floor(self, kind: str, net_id: Optional[int] = None,
              layer: Optional[str] = None, type: str = 'track',
              b: Optional[RuleItem] = None) -> Optional[float]:
        """The smallest value DRC accepts for ``kind`` on this net/layer (with
        the fab floor applied). None when nothing declares one."""
        a = self.item_for_net(net_id or 0, type, layer)
        return self.resolve_for_copper(kind, a, b, layer).min

    def draw_size(self, kind: str, net_id: int, layer: Optional[str] = None,
                  default: Optional[float] = None, type: Optional[str] = None,
                  diff_pair: Optional[str] = None) -> Optional[float]:
        """The size to DRAW: explicit CLI value (PNS 'custom size'), else the
        resolved ``opt`` (rule opt over the aggregate class value, PNS 'use
        netclass values'), else ``default``; then clamped into [min, max].
        ``diff_pair`` (the pair's name) marks the net a coupled-pair member,
        whose class track width is the class ``diff_pair_width``."""
        kind = canonical_kind(kind)
        if type is None:
            type = 'via' if kind in ('via_diameter', 'hole_size') else 'track'
        a = self.item_for_net(net_id, type, layer, diff_pair=diff_pair)
        c = self.resolve_for_copper(kind, a, None, layer)
        v = self.cli.get(kind)
        if v is None:
            v = c.opt
        if v is None:
            v = default
        if v is None:
            return None
        if c.min is not None and v < c.min:
            v = c.min
        if c.max is not None and v > c.max:
            v = c.max
        return float(v)

    # ---------------- disclosure ----------------

    def unsupported(self) -> List[Tuple[str, str]]:
        out = [(r.name, r.unsupported) for r in self.rules
               if r.unsupported and not r.honoured]
        named = {n for n, _w in out}
        for r in self.rules:
            if r.router_unhonoured and not r.unsupported:
                out.append((r.name, r.router_unhonoured))
                named.add(r.name)
        for r in self.rules:
            kinds = sorted(k for k in r.constraints if k in ROUTER_UNHONOURED_KINDS)
            if kinds and r.name not in named and not r.honoured:
                out.append((r.name, '; '.join(f"{k}: {_ROUTER_UNHONOURED_WHY[k]}"
                                              for k in kinds)))
        return out

    def severity_of(self, kind: str) -> str:
        """KiCad's severity for the error ``kind`` reports under (the project's
        ``rule_severities``, 'error' when it says nothing). A rule of a kind
        the project ignores is one KiCad flags nothing for."""
        return self.rule_severities.get(_SEVERITY_KEY.get(kind, kind), 'error')

    def courtyard_index(self, pcb_data, board_path: Optional[str] = None,
                        kinds: Iterable[str] = VIA_SIZE_KINDS) -> Optional['CourtyardIndex']:
        """#1805: a `CourtyardIndex` when a rule of one of ``kinds`` binds by
        courtyard (`Rule.courtyard_condition`) and the outlines were read,
        else None: then nothing is resolved by position."""
        kinds = set(kinds)
        if not any(r.courtyard_condition is not None and kinds & set(r.constraints)
                   for r in self.rules):
            return None
        idx = CourtyardIndex(self, pcb_data, board_path)
        return idx if idx.available else None

    def via_item(self, via, courtyards: Optional['CourtyardIndex'] = None) -> RuleItem:
        """The RuleItem of one via of the board; with ``courtyards``, the
        courtyards its disc overlaps measured (#1805), so a rule bound by a
        courtyard term resolves for it."""
        item = self.item_for_net(int(getattr(via, 'net_id', 0) or 0), 'via')
        if courtyards is None:
            return item
        return replace(item, courtyards=courtyards.of_disc(
            float(via.x), float(via.y), float(via.size) / 2.0))

    def track_item(self, seg, courtyards: Optional['CourtyardIndex'] = None) -> RuleItem:
        """#1832: `via_item`'s twin for a track segment on its own layer
        (a footprint's graphic copper is typed 'graphic'); with
        ``courtyards``, the courtyards its capsule overlaps measured."""
        item = self.item_for_net(int(getattr(seg, 'net_id', 0) or 0),
                                 'graphic' if getattr(seg, 'graphic', False) else 'track',
                                 getattr(seg, 'layer', None))
        if courtyards is None:
            return item
        return replace(item, courtyards=courtyards.of_capsule(
            float(seg.start_x), float(seg.start_y), float(seg.end_x), float(seg.end_y),
            float(seg.width or 0.0) / 2.0))

    def pad_item(self, pad, courtyards: Optional['CourtyardIndex'] = None) -> RuleItem:
        """#1832: `item_for_pad` with, given ``courtyards``, the courtyards
        the pad's copper overlaps measured."""
        item = self.item_for_pad(pad)
        if courtyards is None:
            return item
        return replace(item, courtyards=courtyards.of_pad(pad))

    def via_disallow_rule(self, net_id: int, via_type: str = 'through',
                          layers: Optional[Sequence[str]] = None) -> Optional['Rule']:
        """The `disallow` rule KiCad's items_not_allowed names for a via of
        ``net_id`` and ``via_type`` with copper on ``layers`` (default every
        copper layer), or None: the LAST rule in file order whose tokens
        forbid that via type (`DISALLOW_VIA_TYPES`) and that binds the via on
        one of the layers it has copper on (`_rule_matches`), so a rule a
        `(layer ...)` clause scopes to one of them forbids it. Rules the
        resolver does not evaluate (a rule area, an unread condition) are not
        read here. check_drc's grade and `net_keepouts.planar_nets` both ask
        this, so a no-via net is the one check_drc flags a via on."""
        on = tuple(layers if layers is not None else self.copper_layers)
        key = ('_via_disallow', net_id, via_type, on)
        if key in self._cache:
            return self._cache[key]
        item = self.item_for_net(
            net_id, 'via', None, layers=frozenset(on),
            via_type=via_type if via_type in ('through', 'micro', 'blind', 'buried')
            else 'blind_buried')
        ctx = ('disallow', self._net_name_set())
        hit = None
        for r in reversed(self.rules):
            if r.unsupported or r.area_condition is not None or 'disallow' not in r.constraints:
                continue
            tokens = r.constraints['disallow'].get('disallow', ())
            if not any(t in DISALLOW_VIA_TYPES and (DISALLOW_VIA_TYPES[t] is None
                                                    or via_type in DISALLOW_VIA_TYPES[t])
                       for t in tokens):
                continue
            if any(_rule_matches(r, item, None, l, self.copper_layers, ctx) for l in on):
                hit = r
                break
        self._cache[key] = hit
        return hit

    def via_floors(self, net_id: int = 0) -> Dict[str, float]:
        """#1676: the minimums `.kicad_dru` rules set on a via of ``net_id``, in
        fab_tiers FLOOR_KEYS vocabulary (`via_diameter`, `via_drill`,
        `annular`), at KiCad's value. A kind no rule sets is absent (a Board
        Setup minimum is `--escalation board`'s, not this), and so is one the
        project's severities ignore. {} on a board without such a rule, so a
        via is then exactly what it was."""
        if not self.dru_path:
            return {}
        key = ('_via_floors', net_id)
        hit = self._cache.get(key)
        if hit is not None:
            return hit
        out: Dict[str, float] = {}
        item = self.item_for_net(net_id, 'via')
        for kind, floor_key in (('via_diameter', 'via_diameter'), ('hole_size', 'via_drill'),
                                ('annular_width', 'annular')):
            if self.severity_of(kind) == 'ignore':
                continue
            c = self.resolve_for_copper(kind, item, None, None)
            v = c.kicad_min if c.fab_bound else c.min
            if c.min_rule is not None and v is not None and v > 0:
                out[floor_key] = float(v)
        self._cache[key] = out
        return out

    def via_meeting_rules(self, size: float, drill: float, net_id: int = 0) -> Tuple[float, float]:
        """(size, drill) raised as far as the rules `via_floors` names require:
        the drill to a `hole_size` minimum, the diameter to a `via_diameter`
        minimum and to the drill plus the `annular_width` ring each side
        (`fab_tiers._via_holding_ring`, as #1579 does for the Board Setup
        ring). Raise-only: a via that meets them is returned as it came."""
        floors = self.via_floors(net_id)
        if not floors:
            return size, drill
        from fab_tiers import _via_holding_ring
        d = max(drill, floors.get('via_drill', 0.0))
        s = max(size, floors.get('via_diameter', 0.0))
        ring = floors.get('annular')
        if ring:
            s = _via_holding_ring(s, d, ring)
        return s, d

    def via_size_grade(self, vias, board_floors=None) -> List[dict]:
        """#1676: `annular_grade` for the two other kinds a rule sizes a via by:
        one row per `via_diameter` / `hole_size` rule that sets the minimum of
        at least one of ``vias``: ``{rule, kind, min, vias, below, smallest}``,
        ``min`` KiCad's value and ``smallest`` the smallest size it bound.
        ``board_floors`` as for `annular_grade` (#1775): a via no rule binds is
        graded against its `via_diameter` / `via_drill` under the rule name
        `Board Setup <key>`, the row marked ``board_setup``."""
        rows: Dict[Tuple[str, str], dict] = {}
        bf = board_floors or {}
        for v in vias or ():
            size, drill = _via_attr(v, 'size'), _via_attr(v, 'drill')
            if not size or not drill:
                continue
            item = self.item_for_net(int(_via_attr(v, 'net_id')), 'via')
            for kind, have in (('via_diameter', size), ('hole_size', drill)):
                c = self.resolve_for_copper(kind, item, None, None)
                need = c.kicad_min if c.fab_bound else c.min
                rule, setup = c.min_rule, False
                if rule is None:
                    need = bf.get('via_diameter' if kind == 'via_diameter' else 'via_drill')
                    rule, setup = f"Board Setup {BOARD_MIN_KEYS[kind]}", True
                if not need:
                    continue
                row = rows.setdefault((rule, kind), {
                    'rule': rule, 'kind': kind, 'min': need, 'vias': 0, 'below': 0,
                    'smallest': have, **({'board_setup': True} if setup else {})})
                row['vias'] += 1
                row['smallest'] = min(row['smallest'], have)
                if have < need - 1e-6:
                    row['below'] += 1
        for row in rows.values():
            row['smallest'] = round(row['smallest'], 4)
        return [rows[k] for k in sorted(rows)]

    def annular_grade(self, vias, board_floors=None) -> List[dict]:
        """#1649: the vias of one step graded against each `annular_width`
        rule (`ROUTER_UNHONOURED_KINDS`) -- the
        answer the "NOT honoured" line, printed before the step, cannot give.
        One row per rule that sets the ring of at least one of ``vias``:
        ``{rule, min, vias, below, smallest_ring}``, ``min`` KiCad's value
        (before this tool's fab-profile raise, as check_drc grades it) and
        ``below`` the vias whose ring, (size - drill) / 2, falls short of it.
        A via the rule does not bind (another rule, a class, the board
        minimum set its ring) is not counted. [] when no such rule binds.

        ``board_floors`` (#1775), a FLOOR_KEYS dict of the Board Setup
        minimums the run ships graded against (`board_setup_via_floors`):
        a via no rule binds is graded against its `annular` instead, under
        the rule name `Board Setup min_via_annular_width`, the row marked
        ``board_setup``. None (the default) grades rules only."""
        rows: Dict[str, dict] = {}
        board_ring = (board_floors or {}).get('annular')
        for v in vias or ():
            size, drill = _via_attr(v, 'size'), _via_attr(v, 'drill')
            if not size or not drill:
                continue
            c = self.resolve('annular_width',
                             self.item_for_net(int(_via_attr(v, 'net_id')), 'via'),
                             None, None)
            rule, setup = c.min_rule, False
            if rule is None:                # a class or the board minimum set it
                if not board_ring:
                    continue
                rule, setup = f"Board Setup {BOARD_MIN_KEYS['annular_width']}", True
                need = board_ring
            else:
                need = c.kicad_min if c.fab_bound else c.min
            if need is None:
                continue
            ring = (size - drill) / 2.0
            row = rows.setdefault(rule, {'rule': rule, 'min': need, 'vias': 0,
                                         'below': 0, 'smallest_ring': ring,
                                         **({'board_setup': True} if setup else {})})
            row['vias'] += 1
            row['smallest_ring'] = min(row['smallest_ring'], ring)
            if ring < need - 1e-6:
                row['below'] += 1
        for row in rows.values():
            row['smallest_ring'] = round(row['smallest_ring'], 4)
        return [rows[k] for k in sorted(rows)]

    def honoured_rules(self) -> List[Tuple[str, str]]:
        """Rules the resolver does not evaluate but another consumer honours
        (net_keepouts: a `disallow` rule on a named rule area)."""
        return [(r.name, r.honoured) for r in self.rules if r.honoured]

    def unconsumed(self) -> List[Tuple[str, List[str]]]:
        return [(r.name, r.unconsumed_kinds) for r in self.rules
                if not r.unsupported and r.unconsumed_kinds]

    def report_lines(self) -> List[str]:
        out = kicad_refusal_lines(self.kicad_refused)
        for name, what in self.honoured_rules():
            out.append(f".kicad_dru rule '{name}': {what}")
        for name, why in self.unsupported():
            out.append(f".kicad_dru rule '{name}': NOT honoured ({why})"
                       + ('' if _KICAD_REFUSES in why
                          else '; KiCad will still enforce it'))
        for name, kinds in self.unconsumed():
            # #1313: a length window is graded (check_drc) but not routed to.
            graded = [k for k in kinds if k in GRADED_ONLY_KINDS]
            rest = [k for k in kinds if k not in GRADED_ONLY_KINDS]
            if rest:
                out.append(f".kicad_dru rule '{name}': parsed, but {', '.join(rest)} "
                           f"has no consumer in this tool yet")
            if graded:
                out.append(f".kicad_dru rule '{name}': {', '.join(graded)} is not "
                           f"routed to (a --length-match-group aims its target "
                           f"into a length window); check_drc grades it")
        out.extend(self.notes)
        return out

    def table(self) -> dict:
        """The whole rule table as plain data (loader parity, JSON)."""
        return {
            'board_min': dict(sorted(self.board_min.items())),
            'classes': {k: dict(sorted(v.items())) for k, v in sorted(self.classes.items())},
            'memberships': {int(k): sorted(v) for k, v in sorted(self.memberships.items())},
            'rules': [{
                'name': r.name, 'order': r.order,
                'constraints': {k: {kk: (sorted(vv) if isinstance(vv, set) else vv)
                                    for kk, vv in v.items()}
                                for k, v in sorted(r.constraints.items())},
                'layer': r.layer_clause, 'condition': r.condition_src,
                'severity': r.severity, 'unsupported': r.unsupported,
                'honoured': r.honoured, 'router_unhonoured': r.router_unhonoured,
                'unconsumed': list(r.unconsumed_kinds),
            } for r in self.rules],
            'kicad_refused': list(self.kicad_refused),
            'fab_floor': dict(sorted(self.fab_floor.items())),
            'cli': dict(sorted(self.cli.items())),
        }


def kicad_refusal_lines(refused: Sequence[dict]) -> List[str]:
    """The finding `dru_refusals` is, for a console: about the file, and first,
    because KiCad itself says nothing (exit 0, no message). One line per cause."""
    return [".kicad_dru: KiCad ENFORCES NO RULE OF THIS FILE"
            + (f": rule '{r['rule']}': " if r['rule'] else ': ') + r['reason']
            + ". This tool still reads the file's other rules, so it routes and "
              "grades by rules KiCad's own DRC is not applying" for r in refused]


def read_board_kicad_refusals(board_path: Optional[str]) -> List[dict]:
    """`dru_refusals` of the `.kicad_dru` beside a board, [] without one, for a
    tool with no DesignRules in hand."""
    dru = os.path.splitext(board_path)[0] + '.kicad_dru' if board_path else ''
    if not dru or not os.path.isfile(dru):
        return []
    try:
        text = open(dru, encoding='utf-8').read()
    except OSError:
        return []
    return dru_refusals(text, parse_dru(text)[0])


def relaxing_rules(rules, board_min) -> List[dict]:
    """#1649: the `.kicad_dru` rules that set a minimum BELOW the board-wide
    one Board Setup declares for the same kind (``board_min``: the
    ``min_*`` keys of `BOARD_MIN_KEYS`), each ``{rule, kind, min, board,
    board_key, condition, layer, courtyard, unresolved}``. Such a rule licenses a smaller size only on the
    items its condition binds, so a request below the board minimum belongs
    on a step scoped to them. In file order.

    #1805: a rule whose condition is not fully read is listed too.
    ``courtyard`` names the parts its courtyard terms name, when they are all
    it does not read: it binds by POSITION, inside those courtyards only.
    ``unresolved`` is why it could not be read otherwise (it may relax). A
    rule its courtyard terms do not CONFINE to the parts (`_courtyard_confines`:
    a negated term, `!A.insideCourtyard('U1')`, or one ORed with another,
    `A.insideCourtyard('U1') || A.NetName == 'X'`) binds outside them, so it
    names no courtyard and is ``unresolved``.
    A rule unread for anything but its condition (an ignored severity, a
    value, a non-copper layer clause) relaxes nothing and is left out."""
    out = []
    for r in rules or ():
        if r.unsupported and not r.unsupported.startswith('condition'):
            continue
        cy = sorted(_courtyard_patterns(r.courtyard_condition))
        unresolved = r.unsupported if r.unsupported and not cy else None
        if cy and not _courtyard_confines(r.courtyard_condition):
            cy, unresolved = [], (
                'a negated courtyard term: it binds by position, outside the part'
                if _courtyard_negated(r.courtyard_condition) else
                'a courtyard term another term stands in for (||): it binds '
                'outside the part too')
        for kind, spec in sorted(r.constraints.items()):
            key = BOARD_MIN_KEYS.get(kind)
            v, b = spec.get('min'), (board_min or {}).get(key)
            if key and isinstance(v, (int, float)) and isinstance(b, (int, float)) \
                    and v < b - 1e-9:
                out.append({'rule': r.name, 'kind': kind, 'min': v, 'board': b,
                            'board_key': key, 'condition': r.condition_src,
                            'layer': r.layer_clause, 'courtyard': cy,
                            'unresolved': unresolved})
    return out


def relaxed_via_nets(dr: 'DesignRules', pcb_data, rows) -> Dict[str, Dict[str, object]]:
    """#1805: for each `relaxing_rules` row of a via kind (`VIA_SIZE_KINDS`),
    the nets whose via the rule binds, ``{rule: {'nets': [names], 'how': ...}}``
    keyed by rule name. ``how`` is 'condition' (read exactly), 'courtyard'
    (an UPPER BOUND: a courtyard term read as "the net has a pad on that
    part"; KiCad binds the via by where it stands, so the same net's via
    outside the courtyard is held to the board minimum), 'may' (nets the
    unread condition does not rule out), or 'every' (no condition).

    A `(layer ...)` clause is read as KiCad reads it for a via
    (`_rule_matches`): it does not empty the list where KiCad binds the via
    through it (outer, "F.Cu"), and does where KiCad binds none."""
    out: Dict[str, Dict[str, object]] = {}
    by_name = {r.name: r for r in dr.rules}
    pads_on: Dict[int, Set[str]] = {}
    for key, fp in (getattr(pcb_data, 'footprints', None) or {}).items():
        for pad in getattr(fp, 'pads', ()) or ():
            if getattr(pad, 'net_id', 0):
                pads_on.setdefault(int(pad.net_id), set()).add(key)
    ctx_names = dr._net_name_set()
    copper = list(dr.copper_layers or ())
    for row in rows:
        r = by_name.get(row['rule'])
        if r is None or row['kind'] not in VIA_SIZE_KINDS or r.name in out:
            continue
        ctx = (row['kind'], ctx_names)
        if not _rule_matches(replace(r, condition=None), dr.item_for_net(0, 'via'),
                             None, None, copper, ctx):
            # its layer clause binds no via, whatever the condition names
            out[r.name] = {'nets': [], 'how': 'condition'}
            continue
        if r.condition_src is None:
            out[r.name] = {'nets': [], 'how': 'every'}
            continue
        nets = []
        maybe = parse_condition_maybe(r.condition_src) if row.get('unresolved') else None
        for nid, name in sorted(dr.net_names.items(), key=lambda kv: kv[1]):
            if not nid or not name:
                continue
            item = dr.item_for_net(nid, 'via')
            if row.get('courtyard'):
                cy = frozenset((dr.footprint_ids.get(k, (k, None)) + (s,))
                               for k in pads_on.get(nid, ()) for s in ('front', 'back'))
                hit = _rule_matches(r, replace(item, courtyards=cy), None, None,
                                    copper, ctx, r.courtyard_condition)
            elif maybe is not None:
                hit = _eval3(maybe, item, None, None, ctx) is not False
            else:
                hit = _rule_matches(r, item, None, None, copper, ctx)
            if hit:
                nets.append(name)
        out[r.name] = {'nets': nets, 'how': ('courtyard' if row.get('courtyard') else
                                             'may' if maybe is not None else 'condition')}
    return out


def _grade_rule_label(row) -> str:
    """What a via-grade row names: a `.kicad_dru` rule, or (#1775) the
    Board Setup minimum a via no rule binds is graded against."""
    if row.get('board_setup'):
        return row['rule']
    return f".kicad_dru rule '{row['rule']}'"


def annular_grade_lines(rows) -> List[str]:
    """One end-of-step line per `DesignRules.annular_grade` row (#1649):
    whether this step's vias on the nets the rule binds meet its ring."""
    out = []
    for r in rows:
        head = (f"{_grade_rule_label(r)} (annular_width min {r['min']:g} mm): "
                f"this step's {r['vias']} via(s) it binds")
        if r['below']:
            out.append(f"{head}: {r['below']} BELOW it (smallest ring "
                       f"{r['smallest_ring']:g} mm); KiCad flags them (annular_width), "
                       f"check_drc VIA-ANNULAR")
        else:
            out.append(f"{head} meet it (smallest ring {r['smallest_ring']:g} mm)")
    return out


def run_via_for_rules(board_path: Optional[str], pcb_data, via_size: float,
                      via_drill: float, board_setup: bool = True) -> Tuple[float, float]:
    """#1676: the (size, drill) a step that was given no explicit via starts
    from: its own, raised to the `.kicad_dru` rules that bind a via of no
    particular net, then (``board_setup``, #1775) to the Board Setup via
    minimums under ``--escalation board`` (`via_for_board_setup`, which reads
    the run's escalation policy, so it must run after the policy is set).
    Without a rule file the rule step is skipped, unread."""
    path = board_path or getattr(pcb_data, 'source_path', '') or ''
    out = (via_size, via_drill)
    if path and os.path.isfile(os.path.splitext(path)[0] + '.kicad_dru'):
        try:
            out = DesignRules.from_project(pcb_data, path).via_meeting_rules(via_size,
                                                                             via_drill)
        except (OSError, ValueError) as e:
            print(f"  (via rules not read: {e})")
            out = (via_size, via_drill)
        if out != (via_size, via_drill):
            print(f"Via {via_size:g}/{via_drill:g} raised to {out[0]:g}/{out[1]:g} to meet "
                  f"the board's .kicad_dru via rules (#1676).")
    if board_setup:
        out = via_for_board_setup(*out)
    return out


#: FLOOR_KEYS of the Board Setup via minimums (`min_via_diameter`,
#: `min_through_hole_diameter`, `min_via_annular_width`).
BOARD_SETUP_VIA_KEYS = ('via_diameter', 'via_drill', 'annular')


def board_setup_via_floors() -> Dict[str, float]:
    """#1775: the Board Setup via minimums of this run, as its escalation
    policy resolved them (`fab_tiers.set_policy_from_args` on the CLI,
    `set_fab_tier_from_config` in the GUI); {} when the board declares none."""
    from fab_tiers import get_escalation_policy
    floors = get_escalation_policy()[1]
    return {k: floors[k] for k in BOARD_SETUP_VIA_KEYS if floors.get(k)}


def via_for_board_setup(via_size: float, via_drill: float) -> Tuple[float, float]:
    """#1775: a via a step sizes itself, raised to the board's Board Setup
    minimums ONLY under ``--escalation board``: the drill to
    `min_through_hole_diameter`, the diameter to `min_via_diameter` and to
    the drill plus `min_via_annular_width` each side (`_via_holding_ring`, as
    #1579 raises a descent rung). Raise-only. Any other policy keeps the via;
    a shortfall is then disclosed after the step (`disclose_board_setup_vias`)."""
    from fab_tiers import get_escalation_policy, _via_holding_ring
    if get_escalation_policy()[0] != 'board':
        return via_size, via_drill
    f = board_setup_via_floors()
    if not f:
        return via_size, via_drill
    d = max(via_drill, f.get('via_drill', 0.0))
    s = max(via_size, f.get('via_diameter', 0.0))
    if f.get('annular'):
        s = _via_holding_ring(s, d, f['annular'])
    if (s, d) != (via_size, via_drill):
        print(f"Via {via_size:g}/{via_drill:g} raised to {s:g}/{d:g} to meet the board's "
              f"Board Setup via minimums (--escalation board, #1775).")
    return s, d


def board_setup_via_grade(pcb_data, vias, board_path: Optional[str] = None,
                          rules=None) -> List[dict]:
    """#1775: the rows of `annular_grade` / `via_size_grade` that grade
    ``vias`` (Via objects or a fanout's via dicts) against the Board Setup
    via minimums no `.kicad_dru` rule replaces, and only those with a via
    below. [] when the run's project writeback will lower those minimums to
    the copper (`fab_tiers.project_writeback`; it says so itself, FAB FLOOR
    RELAXED), when the board declares none, or when no via falls short.
    ``rules`` is the step's DesignRules when it has one (built from the
    board's project otherwise)."""
    from fab_tiers import project_writeback
    floors = board_setup_via_floors()
    if not vias or not floors or project_writeback():
        return []
    if rules is None:
        try:
            rules = DesignRules.from_project(pcb_data, board_path)
        except (OSError, ValueError):
            rules = DesignRules()
    rows = rules.annular_grade(vias, floors) + rules.via_size_grade(vias, floors)
    return [r for r in rows if r.get('board_setup') and r['below']]


def disclose_board_setup_vias(pcb_data, vias, board_path: Optional[str] = None,
                              rules=None) -> List[dict]:
    """`board_setup_via_grade`, printed one line per row: the end-of-step
    disclosure of vias laid below the board's own Board Setup minimum, which
    a self-sized via is under any policy but ``board`` (#1775). Returns the
    rows (the JSON_SUMMARY ``board_setup_vias`` key)."""
    rows = board_setup_via_grade(pcb_data, vias, board_path, rules)
    ring = [r for r in rows if 'smallest_ring' in r]
    size = [r for r in rows if 'smallest_ring' not in r]
    for line in annular_grade_lines(ring) + via_size_grade_lines(size):
        print(f"  WARNING: {line}; the project writeback is off, so the minimum "
              f"ships as declared (--escalation board sizes a self-sized via to it)")
    return rows


def install_board_via_rule_floors(pcb_data, board_path: Optional[str] = None) -> None:
    """`fab_tiers.set_rule_via_floors` for a step that reads its via rungs
    before it has built a config (the fanouts)."""
    from fab_tiers import set_rule_via_floors
    path = board_path or getattr(pcb_data, 'source_path', '') or ''
    if not path or not os.path.isfile(os.path.splitext(path)[0] + '.kicad_dru'):
        set_rule_via_floors({})
        return
    try:
        set_rule_via_floors(DesignRules.from_project(pcb_data, path).via_floors(0))
    except (OSError, ValueError):
        set_rule_via_floors({})


def via_size_grade_lines(rows) -> List[str]:
    """One end-of-step line per `DesignRules.via_size_grade` row (#1676)."""
    out = []
    for r in rows:
        kicad, ours = _VIA_SIZE_FLAGS[r['kind']]
        head = (f"{_grade_rule_label(r)} ({r['kind']} min {r['min']:g} mm): "
                f"this step's {r['vias']} via(s) it binds")
        if r['below']:
            out.append(f"{head}: {r['below']} BELOW it (smallest {r['smallest']:g} mm); "
                       f"KiCad flags them ({kicad}), check_drc {ours}")
        else:
            out.append(f"{head} meet it (smallest {r['smallest']:g} mm)")
    return out


def edge_rule_grade(pcb_data, segments, vias, board_path: Optional[str] = None) -> List[dict]:
    """#1678: the tracks and vias of one step graded against each `.kicad_dru`
    `edge_clearance` rule that sets their floor: one row per rule, ``{rule,
    min, tracks, vias, closer, closest}``, ``min`` KiCad's value and ``closest``
    the smallest copper-to-Edge.Cuts gap it bound. It reads only geometry, so
    it sees copper from every path, and it counts a track that lands on a pad
    already inside the rule, which KiCad flags too. [] when no rule binds."""
    if not segments and not vias:
        return []
    edge = PadEdgeRules(pcb_data, board_path)
    if not edge.copper_rules:
        return []
    from check_drc import board_edge_geometry, _point_to_rings_distance, _segment_to_rings_distance
    rings, _outer, _cutouts = board_edge_geometry(pcb_data.board_info, rect_ring=True)
    if not rings:
        return []
    values = {r.name: v for r, _a, v in edge.copper_rules}
    rows: Dict[str, dict] = {}

    def note(name, gap, kind):
        row = rows.setdefault(name, {'rule': name, 'min': values[name], 'tracks': 0, 'vias': 0,
                                     'closer': 0, 'closest': gap})
        row['tracks' if kind == 'track' else 'vias'] += 1
        row['closest'] = min(row['closest'], gap)
        if gap < values[name] - 1e-6:
            row['closer'] += 1

    for s in segments or ():
        name = edge.for_copper(int(s.net_id or 0), 'track', 0.0, s.layer)[2]
        if name:
            note(name, _segment_to_rings_distance(s.start_x, s.start_y, s.end_x, s.end_y, rings)
                 - s.width / 2.0, 'track')
    for v in vias or ():
        name = edge.for_copper(int(v.net_id or 0), 'via', 0.0)[2]
        if name:
            note(name, _point_to_rings_distance(v.x, v.y, rings) - v.size / 2.0, 'via')
    for row in rows.values():
        row['closest'] = round(row['closest'], 4)
    return [rows[k] for k in sorted(rows)]


def edge_rule_grade_lines(rows) -> List[str]:
    """One end-of-step line per `edge_rule_grade` row (#1678)."""
    out = []
    for r in rows:
        head = (f".kicad_dru rule '{r['rule']}' (edge_clearance min {r['min']:g} mm): "
                f"this step's {r['tracks']} track(s), {r['vias']} via(s) it binds")
        if r['closer']:
            out.append(f"{head}: {r['closer']} closer to Edge.Cuts (closest {r['closest']:g} mm); "
                       f"KiCad flags them (copper_edge_clearance), check_drc "
                       f"segment-board-edge / via-board-edge, or accepted where the "
                       f"copper lands on a pad that is itself inside the rule")
        else:
            out.append(f"{head} keep it (closest {r['closest']:g} mm)")
    return out


def disclose_step_rules(rules, pcb_data, results, board_path: Optional[str] = None) -> dict:
    """The rule grades of one routing step's own copper, printed per rule and
    returned as `JSON_SUMMARY.design_rules` keys: `annular_rules` (#1649),
    `via_rules` (#1676), `edge_rules` (#1678) and `board_setup_vias` (#1775,
    the vias below a Board Setup minimum the writeback will not lower). A rule broken by a descent is
    named here whatever the policy that allowed the descent. A disclosure only:
    it changes no copper. Both routing engines call it, so both fronts get it."""
    vias = [v for r in results for v in (r.get('new_vias') or [])]
    segs = [s for r in results for s in (r.get('new_segments') or [])]
    out = {}
    ag = rules.annular_grade(vias)
    if ag:
        out['annular_rules'] = ag
    vg = rules.via_size_grade(vias)
    if vg:
        out['via_rules'] = vg
    eg = edge_rule_grade(pcb_data, segs, vias, board_path or rules.source or None)
    if eg:
        out['edge_rules'] = eg
    for line in annular_grade_lines(ag) + via_size_grade_lines(vg) + edge_rule_grade_lines(eg):
        print(f"  {line}")
    bs = disclose_board_setup_vias(pcb_data, vias, board_path or rules.source or None, rules)
    if bs:
        out['board_setup_vias'] = bs
    return out


def override_clearance(base, board_min_clearance, *pads):
    """KiCad's pad/footprint clearance OVERRIDE semantics for a pair whose
    resolved (class / rule) clearance is ``base``: when any pad in ``pads``
    carries a positive ``local_clearance`` (its own or its footprint's
    ``(clearance ...)``), the pair clearance IS max(overrides), floored at
    rules.min_clearance -- the engine returns before it looks at a class or a
    rule, so an override BELOW the class wins (drc_engine.cpp, measured on
    KiCad 10.0.0 by tests/oracle/constraint_agreement.py rows
    pad_override_below_class / pad_override_beats_rule /
    pad_override_below_board_min). With no override, ``base``.

    Before this the whole tree priced a pad override as max(base, lc), which
    is right for a ZONE's local clearance and wrong for a pad's: 2932 pads on
    48 corpus boards declare an override below their class (fine-pitch
    BGA/QFN footprints), so those escapes were priced wider than KiCad
    requires and check_drc flagged copper KiCad accepts."""
    lc = 0.0
    for p in pads:
        if p is None:
            continue
        v = getattr(p, 'local_clearance', 0.0) or 0.0
        if v > lc:
            lc = v
    if lc <= 0.0:
        return base
    bm = board_min_clearance or 0.0
    return lc if lc >= bm else bm


def board_min_clearance_for(pcb_data, board_path=None):
    """rules.min_clearance of the board beside ``board_path`` /
    ``pcb_data.source_path`` (0.0 when undeclared or unreadable) -- the floor
    ``override_clearance`` applies. A stdlib JSON read, no parser."""
    path = board_path or getattr(pcb_data, 'source_path', '') or ''
    if not path:
        return 0.0
    pro = os.path.splitext(path)[0] + '.kicad_pro'
    try:
        with open(pro, encoding='utf-8') as f:
            proj = json.load(f)
        v = ((proj.get('board') or {}).get('design_settings') or {}).get('rules', {}).get('min_clearance')
        return float(v) if isinstance(v, (int, float)) and v > 0 else 0.0
    except (OSError, ValueError, AttributeError):
        return 0.0


def board_min_clearance_cached(pcb_data) -> float:
    """``board_min_clearance_for`` memoised on the PCBData object, for hot
    per-connector callers that have no GridRouteConfig in scope."""
    v = getattr(pcb_data, '_krt_board_min_clearance', None)
    if v is None:
        v = board_min_clearance_for(pcb_data)
        try:
            pcb_data._krt_board_min_clearance = v
        except Exception:                                      # noqa: BLE001
            pass
    return v


def _edge_item_unknown(node):
    """`node` with every term that reads B as ('unknown',), B.Pad_Number apart.
    An edge clearance is evaluated with B the Edge.Cuts shape it is measured
    to, and `_EDGE_ITEM` answers only what was measured of that shape as A."""
    if not isinstance(node, tuple) or not node:
        return node
    op = node[0]
    if op == 'prop' and node[1:] == ('B', 'pad_number'):
        return node                # the shape has none (`_EDGE_ITEM`), the pad does
    if op in ('prop', 'call', 'area'):
        return ('unknown',) if node[1] in ('B', 'AB') else node
    if op == 'coupled':
        return ('unknown',)
    if op == 'cmp':
        lhs, rhs = _edge_item_unknown(node[2]), _edge_item_unknown(node[3])
        return ('unknown',) if 'unknown' in (lhs[0], rhs[0]) else node
    if op in ('and', 'or', 'not'):
        return (op,) + tuple(_edge_item_unknown(c) for c in node[1:])
    return node


#: A.insideCourtyard('X') / A.intersectsCourtyard('X') on an edge rule
_COURTYARD_TERM = re.compile(
    r"\bA\.(?:insideCourtyard|intersectsCourtyard)\(\s*('[^']*'|\"[^\"]*\")\s*\)",
    re.IGNORECASE)


def _has_unknown(node) -> bool:
    if not isinstance(node, tuple) or not node:
        return False
    return node[0] == 'unknown' or any(_has_unknown(c) for c in node[1:])


def _courtyard_patterns(node) -> Set[str]:
    """Every footprint pattern the courtyard terms of an AST name."""
    if not isinstance(node, tuple) or not node:
        return set()
    if node[0] == 'courtyard':
        return {node[3]}
    out: Set[str] = set()
    for c in node[1:]:
        out |= _courtyard_patterns(c)
    return out


def _courtyard_confines(node) -> bool:
    """The condition AST is false wherever every courtyard term is false: the
    rule binds an item only inside a courtyard it names. A term under a `!`
    (`!A.insideCourtyard('U1')`) or beside an `||` that another term can
    satisfy does not confine it."""
    def false_without(n):           # false when every courtyard term is false
        if not isinstance(n, tuple) or not n:
            return False
        op = n[0]
        if op == 'courtyard':
            return True
        if op == 'and':
            return any(false_without(c) for c in n[1:])
        if op == 'or':
            return all(false_without(c) for c in n[1:])
        if op == 'not':
            return true_without(n[1])
        return False

    def true_without(n):            # true when every courtyard term is false
        if not isinstance(n, tuple) or not n:
            return False
        op = n[0]
        if op == 'and':
            return all(true_without(c) for c in n[1:])
        if op == 'or':
            return any(true_without(c) for c in n[1:])
        if op == 'not':
            return false_without(n[1])
        return False
    return false_without(node)


def _courtyard_negated(node, odd: bool = False) -> bool:
    """A courtyard term of an AST stands under an odd number of negations: it
    holds for an item OUTSIDE the part it names."""
    if not isinstance(node, tuple) or not node:
        return False
    if node[0] == 'courtyard':
        return odd
    if node[0] == 'not':
        odd = not odd
    return any(_courtyard_negated(c, odd) for c in node[1:])


class CourtyardIndex:
    """#1805: the part courtyards a via overlaps (#1832: a track or a pad,
    `of_capsule` / `of_pad`), as KiCad's courtyard
    predicates read them (`_COURTYARD_FUNCS`): its disc against each drawn
    courtyard of the parts the board's courtyard rules name, tagged 'front'
    for the courtyard on the part's own side and 'back' for the other. The
    outlines are placement's (`placement.parser.extract_courtyard_shapes`,
    nested contours even-odd, as KiCad assembles a courtyard).

    `available` is False when the outlines could not be read: an item then
    gets no `courtyards` and its courtyard rules stay unread, as before."""

    def __init__(self, rules: 'DesignRules', pcb_data, board_path: Optional[str]):
        self.parts: List[tuple] = []
        self.available = False
        patterns: Set[str] = set()
        for r in rules.rules:
            patterns |= _courtyard_patterns(r.courtyard_condition)
        path = board_path or getattr(pcb_data, 'source_path', '') or ''
        if not patterns or not path:
            return
        try:
            import _placer_path  # noqa: F401  (placement lives in py_placer/)
            from placement.parser import extract_courtyard_shapes
            from placement.legality import footprint_side
            from shapely import affinity
            shapes = extract_courtyard_shapes(path)
        except Exception:                                      # noqa: BLE001
            return
        self.available = True
        for key, fp in (getattr(pcb_data, 'footprints', None) or {}).items():
            ref, lib = rules.footprint_ids.get(key, (key, None))
            if not any(_footprint_named(p, ref, lib) for p in patterns):
                continue
            own = footprint_side(fp)
            rot = (getattr(fp, 'rotation', 0.0) or 0.0) % 360
            for side, (geom, _how) in (shapes.get(key) or {}).items():
                g = geom if abs(rot) < 0.01 else affinity.rotate(geom, -rot, origin=(0, 0))
                g = affinity.translate(g, fp.x, fp.y)
                self.parts.append((ref, lib, 'front' if side == own else 'back', g, g.bounds))

    def of_disc(self, x: float, y: float, r: float) -> FrozenSet[Tuple[str, Optional[str], str]]:
        """The courtyards a disc of radius ``r`` at (x, y) overlaps (a
        tangent disc does not, as in KiCad)."""
        from shapely.geometry import Point
        out = set()
        p = None
        for ref, lib, side, g, (x0, y0, x1, y1) in self.parts:
            if x < x0 - r or x > x1 + r or y < y0 - r or y > y1 + r:
                continue
            p = p or Point(x, y)
            if g.distance(p) < r - 1e-9:
                out.add((ref, lib, side))
        return frozenset(out)

    def of_capsule(self, x0: float, y0: float, x1: float, y1: float,
                   r: float) -> FrozenSet[Tuple[str, Optional[str], str]]:
        """#1832: the courtyards a track of half-width ``r`` overlaps, its
        capsule against each outline as `of_disc` reads a via's disc. Measured
        on kicad-cli 10.0.6: a track partly inside binds, one whose capsule
        only reaches the courtyard's edge does not, and the track's own layer
        does not matter (a B.Cu track under a front part's F.CrtYd binds
        insideFrontCourtyard())."""
        from shapely.geometry import LineString, Point
        geom = (Point(x0, y0) if (x0, y0) == (x1, y1)
                else LineString([(x0, y0), (x1, y1)]))
        return self._of(geom, r)

    def of_pad(self, pad) -> FrozenSet[Tuple[str, Optional[str], str]]:
        """#1832: the courtyards a pad's copper overlaps -- its shape as
        check_drc measures it (`_pad_copper_core`: custom polygons, round
        corners, tilt). Measured on kicad-cli 10.0.6: a pad straddling the
        courtyard's edge binds."""
        from shapely.geometry import LineString, Point, Polygon
        from shapely.ops import unary_union
        from check_drc import _pad_copper_core
        polys, r = _pad_copper_core(pad)
        parts = []
        for poly in polys:
            pts = list(dict.fromkeys((float(x), float(y)) for x, y in poly))
            if len(pts) == 1:
                parts.append(Point(pts[0]))
            elif len(pts) == 2 or Polygon(pts).area <= 1e-12:
                parts.append(LineString(pts))
            else:
                parts.append(Polygon(pts))
        if not parts:
            return frozenset()
        return self._of(parts[0] if len(parts) == 1 else unary_union(parts), r)

    def _of(self, geom, r: float) -> FrozenSet[Tuple[str, Optional[str], str]]:
        """The courtyards ``geom`` grown by ``r`` overlaps: within less than
        ``r`` of the outline, or, at ``r`` 0, sharing area with it (a shape
        that only touches the outline does not, as in KiCad)."""
        bx0, by0, bx1, by1 = geom.bounds
        out = set()
        for ref, lib, side, g, (x0, y0, x1, y1) in self.parts:
            if bx1 < x0 - r or bx0 > x1 + r or by1 < y0 - r or by0 > y1 + r:
                continue
            if r > 0:
                hit = g.distance(geom) < r - 1e-9
            else:
                hit = g.intersects(geom) and g.intersection(geom).area > 1e-12
            if hit:
                out.add((ref, lib, side))
        return frozenset(out)


class PadEdgeRules:
    """The board-edge clearance each pad is graded at (#1615), each track and
    via (`for_copper`, #1678), and each zone's fill kept at (`for_zone`,
    #1643): the floor the caller passes, replaced by every `.kicad_dru`
    `edge_clearance` rule that binds the item, in file order, the last
    winning. That is KiCad's order, and an explicit rule binds below the
    Board Setup minimum too. Edge-launch connectors rely on that: their land
    runs to the edge.

    A rule's `(layer ...)` clause scopes only the ZONE gap. KiCad's edge check
    evaluates pads, tracks and vias at no layer, and a rule's layer clause
    then excludes none of them. Measured on kicad-cli 10.0.6: `(layer inner)`
    and `(layer B.Cu)` each flag an F.Cu track, an F.Cu pad and a through via,
    and a relaxing `(layer F.Cu)` rule passes B.Cu copper. A term of the
    condition does read the item's layer: `A.Layer == 'In1.Cu'` binds only a
    track on In1.Cu, no via and no other layer's track.

    A pad's `A.insideCourtyard('X')` / `A.intersectsCourtyard('X')` is read
    as `A.memberOfFootprint('X')`, and `notes` says so: placement keeps
    courtyards apart, so the only pads in X's courtyard are X's own. (An
    edge-connector rule is often written that way.)

    A rule whose condition this resolver cannot evaluate for a pad (an
    unmodelled predicate, or a term on B, the edge item) MAY bind it, so
    `for_pad` returns a RANGE: `(lo, hi, rule)`, lo == hi when every rule
    decided. A grader fails a pad only short of `lo` and passes it at
    `hi`, and between the two says it could not tell. `rule` names the
    rule that set the decided value, '' for the caller's floor.

    `active` is False on a board with no edge rule, where every pad reads
    `(base, base, '')`.

    KiCad tries a failed condition again with the two items swapped, and here
    the other item is the Edge.Cuts shape, so `for_pad` asks both ways: a rule
    the shape meets as A binds every pad (`_EDGE_ITEM`)."""

    def __init__(self, pcb_data, board_path: Optional[str] = None):
        path = board_path or getattr(pcb_data, 'source_path', '') or ''
        self.rules: List[tuple] = []
        #: the same rows read for a track or via, whose courtyard term is a
        #: place on the board and not a part: it stays undecided
        self.copper_rules: List[tuple] = []
        #: [{'rule', 'reason'}] for each rule whose condition is not modelled
        self.unmodelled: List[dict] = []
        #: {rule name: how its condition was read}, for a courtyard term
        self.notes: Dict[str, str] = {}
        self.dr = None
        self._cache: Dict[tuple, tuple] = {}
        dru = os.path.splitext(path)[0] + '.kicad_dru' if path else ''
        if not dru or not os.path.isfile(dru):
            return
        dr = DesignRules.from_project(pcb_data, path)
        for rule in dr.rules:
            spec = rule.constraints.get('edge_clearance') or {}
            if spec.get('min') is None:
                continue
            if rule.unsupported and not rule.unsupported.startswith('condition'):
                # a severity, a value or a layer clause the resolver does not
                # read: not evaluated, and said so
                self.unmodelled.append({'rule': rule.name, 'reason': rule.unsupported})
                continue
            src = rule.condition_src
            if src is not None and _COURTYARD_TERM.search(src):
                src = _COURTYARD_TERM.sub(r'A.memberOfFootprint(\1)', src)
                self.notes[rule.name] = ('a courtyard term read as the pads of '
                                         'that footprint')
            ast = _edge_item_unknown(parse_condition_maybe(src))
            self.rules.append((rule, ast, float(spec['min'])))
            self.copper_rules.append(
                (rule, _edge_item_unknown(parse_condition_maybe(rule.condition_src)),
                 float(spec['min'])))
            if _has_unknown(ast):
                self.unmodelled.append({
                    'rule': rule.name,
                    'reason': ('condition not modelled for an edge clearance: '
                               + (rule.unsupported or 'it reads B, the edge item'))})
        if self.rules:
            self.dr = dr

    @property
    def active(self) -> bool:
        return bool(self.rules)

    def for_pad(self, pad, base: float) -> Tuple[float, float, str]:
        if not self.rules:
            return base, base, ''
        key = (getattr(pad, 'component_ref', None), getattr(pad, 'pad_number', None),
               getattr(pad, 'net_id', None), getattr(pad, 'pad_type', None),
               tuple(getattr(pad, 'layers', None) or ()), base)
        hit = self._cache.get(key)
        if hit is not None:
            return hit
        from check_drc import pad_copper_layers
        dr = self.dr
        item = dr.item_for_pad(pad)
        ctx = ('edge_clearance', dr._net_name_set())
        lo = hi = None
        decided = ''
        for layer in sorted(pad_copper_layers(pad, dr.copper_layers)) or [None]:
            # the values this layer may resolve to, walked in file order
            values, rule_name = {base}, ''
            for rule, ast, value in self.rules:
                m = rule_may_match(rule, ast, item, _EDGE_ITEM, layer, dr.copper_layers, ctx,
                                   layer_clause=False)
                if m is True:
                    values, rule_name = {value}, rule.name
                elif m is None:
                    values = values | {value}
            # a pad is clear only when every layer's copper is
            if lo is None or min(values) > lo:
                lo = min(values)
            if hi is None or max(values) > hi:
                hi = max(values)
                decided = rule_name
        out = (lo, hi, decided if lo == hi else '')
        self._cache[key] = out
        return out

    def for_copper(self, net_id: int, kind: str, base: float,
                   layer: Optional[str] = None) -> Tuple[float, float, str]:
        """The same range for a track (``kind`` 'track', on ``layer``) or a via
        ('via') of `net_id`: what KiCad grades, so a rule below the board's
        minimum binds and no fab floor applies (`EdgeFloors` applies it to what
        is routed)."""
        if not self.copper_rules:
            return base, base, ''
        key = ('copper', net_id, kind, layer, base)
        hit = self._cache.get(key)
        if hit is not None:
            return hit
        dr = self.dr
        item = dr.item_for_net(net_id, type=kind, layer=layer)
        ctx = ('edge_clearance', dr._net_name_set())
        values, rule_name = {base}, ''
        for rule, ast, value in self.copper_rules:
            m = rule_may_match(rule, ast, item, _EDGE_ITEM, layer, dr.copper_layers, ctx,
                               layer_clause=False)
            if m is True:
                values, rule_name = {value}, rule.name
            elif m is None:
                values = values | {value}
        lo, hi = min(values), max(values)
        out = (lo, hi, rule_name if lo == hi else '')
        self._cache[key] = out
        return out

    def layer_dependent(self) -> bool:
        """A rule's CONDITION reads the copper layer, so a track's floor may
        differ layer by layer."""
        from net_keepouts import _layer_dependent
        return any(_layer_dependent(ast) for _r, ast, _v in self.copper_rules)

    def for_zone(self, net_id: int, layer: str, base: float) -> Tuple[float, float, str]:
        """The same range for a zone of `net_id` filling `layer`: the gap
        KiCad's filler keeps from Edge.Cuts there. Measured on kicad-cli
        10.0.6 (tests/test_1643_zone_edge_gap.py): that gap is the edge
        clearance alone -- the board's rule, replaced by a binding rule here,
        a lower one too -- whatever the zone's or its class's clearance. The
        filler asks both ways as the DRC does (#1690): `!(A.Type == 'Zone')`
        binds, on the Edge.Cuts shape."""
        if not self.rules:
            return base, base, ''
        key = ('zone', net_id, layer, base)
        hit = self._cache.get(key)
        if hit is not None:
            return hit
        dr = self.dr
        item = dr.item_for_net(net_id, type='zone', layer=layer)
        ctx = ('edge_clearance', dr._net_name_set())
        values, rule_name = {base}, ''
        for rule, ast, value in self.rules:
            m = rule_may_match(rule, ast, item, _EDGE_ITEM, layer, dr.copper_layers, ctx)
            if m is True:
                values, rule_name = {value}, rule.name
            elif m is None:
                values = values | {value}
        lo, hi = min(values), max(values)
        out = (lo, hi, rule_name if lo == hi else '')
        self._cache[key] = out
        return out


class EdgeFloors:
    """What one routing step holds its tracks and vias to from Edge.Cuts when
    a `.kicad_dru` `edge_clearance` rule binds them (#1678): per net, the
    largest value `PadEdgeRules.for_copper` says KiCad may grade it at, pinned
    up to the fab floor, as a layer rule is (a rule may tighten or relax, and
    only the fab floor stops it).

    `track_band` / `via_band` are the smallest floors over the board's nets, so
    the board's static band over-blocks no net; `strictest` is the largest, for
    every scalar reader of ``config.board_edge_clearance``, through which no
    net can then break a rule. A net above a band gets the rest while it routes
    (`net_keepouts.derive_edge_bands`).

    `for_config` answers None on a board with no edge rule on copper, and
    `changed` says whether a rule moved any net's floor: a board where none does
    takes the path it always took."""

    def __init__(self, track, via, base, rule_names, fab):
        #: {net id: {layer: floor}}, {net id: floor}
        self.track, self.via = track, via
        self.base = base
        self.rule_names = rule_names
        self.fab = fab
        vals = [v for d in track.values() for v in d.values()]
        self.track_band = min(vals)
        self.via_band = min(via.values())
        self.strictest = max(max(vals), max(via.values()))
        #: does any net's floor differ from the base the run resolved?
        self.changed = (abs(self.strictest - base) > 1e-9 or abs(self.track_band - base) > 1e-9
                        or abs(self.via_band - base) > 1e-9)

    @classmethod
    def for_config(cls, rules: 'DesignRules', pcb_data, config,
                   base: float) -> Optional['EdgeFloors']:
        if rules is None or not rules.dru_path or rules.severity_of('edge_clearance') == 'ignore':
            return None
        edge = PadEdgeRules(pcb_data, rules.source or None)
        if not edge.copper_rules:
            return None
        from fix_kicad_drc_settings import fab_edge_floor
        fab = fab_edge_floor(rules.source or None)
        nets = [nid for nid, n in (getattr(pcb_data, 'nets', None) or {}).items()
                if nid and getattr(n, 'name', None)] or [0]
        layers = [l for l in (getattr(config, 'layers', None) or rules.copper_layers or ())
                  if str(l).endswith('.Cu')] or [None]
        per_layer = edge.layer_dependent()
        track, via, named = {}, {}, set()

        def floor(row):
            lo, hi, name = row
            if name:
                named.add(name)
            # A value no rule set is the base the run already resolved; a
            # rule's own, or the larger of what an undecided rule may set,
            # floored at the fab's.
            return base if lo == hi == base and not name else max(hi, fab)
        for nid in nets:
            if per_layer:
                track[nid] = {l: floor(edge.for_copper(nid, 'track', base, l)) for l in layers}
            else:
                v = floor(edge.for_copper(nid, 'track', base, layers[0]))
                track[nid] = {l: v for l in layers}
            via[nid] = floor(edge.for_copper(nid, 'via', base))
        return cls(track, via, base, named, fab)


def strictest_edge_floor(pcb_data, base: float, board_path: Optional[str] = None) -> float:
    """`EdgeFloors.strictest` for a step that keeps one edge number and has no
    config in hand; ``base`` on a board with no rule that moves it."""
    path = board_path or getattr(pcb_data, 'source_path', '') or ''
    if not path or not os.path.isfile(os.path.splitext(path)[0] + '.kicad_dru'):
        return base
    try:
        rules = DesignRules.from_project(pcb_data, path)
        floors = EdgeFloors.for_config(rules, pcb_data, object(), base)
    except (OSError, ValueError):
        return base
    return floors.strictest if floors is not None and floors.changed else base


def _netclass_dict(nc) -> dict:
    """A live NETCLASS -> the same dict shape the file loader builds."""
    def g(name, has=None):
        try:
            if has and hasattr(nc, has) and not getattr(nc, has)():
                return None
            fn = getattr(nc, name, None)
            v = fn() if fn else None
            return v / 1e6 if isinstance(v, (int, float)) and v > 0 else None
        except Exception:                                      # noqa: BLE001
            return None
    d = {'clearance': g('GetClearance', 'HasClearance'),
         'track_width': g('GetTrackWidth', 'HasTrackWidth'),
         'via_diameter': g('GetViaDiameter', 'HasViaDiameter'),
         'via_drill': g('GetViaDrill', 'HasViaDrill'),
         'diff_pair_width': g('GetDiffPairWidth', 'HasDiffPairWidth'),
         'diff_pair_gap': g('GetDiffPairGap', 'HasDiffPairGap')}
    try:
        d['priority'] = int(nc.GetPriority())
    except Exception:                                          # noqa: BLE001
        pass
    return d


def _footprint_ids(pcb_data) -> Dict[str, Tuple[str, str]]:
    """{footprint key: (reference, library id)}. A key is not always the
    reference: the second block claiming one is keyed `R7~2` and a block with
    none `#<uuid>` (kicad_parser.disambiguate_references), and KiCad reads the
    reference itself."""
    dup = getattr(pcb_data, 'duplicate_references', None) or {}
    out = {}
    for key, fp in (getattr(pcb_data, 'footprints', None) or {}).items():
        ref = str(key)
        head, sep, tail = ref.rpartition('~')
        if sep and tail.isdigit() and head in dup:
            ref = head
        elif ref.startswith('#'):
            ref = ''
        out[key] = (ref, getattr(fp, 'footprint_name', None))
    return out


#: A pad's `*.Cu` is every copper layer KiCad has, whatever the board's stack:
#: `A.existsOnLayer('In9.Cu')` holds on a through-hole pad of a 4-layer board.
_ALL_COPPER = ('F.Cu', 'B.Cu') + tuple(f'In{i}.Cu' for i in range(1, 31))
_PAD_LAYER_SETS = {'*.Cu': _ALL_COPPER, 'F&B.Cu': ('F.Cu', 'B.Cu'),
                   '*.Mask': ('F.Mask', 'B.Mask'), '*.Paste': ('F.Paste', 'B.Paste'),
                   '*.SilkS': ('F.SilkS', 'B.SilkS')}


def _pad_rule_layers(layers) -> Tuple[str, ...]:
    return tuple(n for l in layers or () for n in _PAD_LAYER_SETS.get(str(l), (str(l),)))


def _groups_by_ref(pcb_data) -> Dict[str, FrozenSet[str]]:
    out: Dict[str, set] = {}
    for gname, refs in (getattr(pcb_data, 'groups', None) or {}).items():
        for ref in refs or ():
            out.setdefault(ref, set()).add(gname)
    return {k: frozenset(v) for k, v in out.items()}


# --------------------------------------------------------------------------
# Engine-side install (both fronts, no flag)
# --------------------------------------------------------------------------

_ANNOUNCED = set()


def _install_rule_floors(config, rules, pcb_data) -> None:
    """The floors the board's `.kicad_dru` sets on what a routing step lays,
    installed at the chokepoint every engine passes through, so both fronts get
    them with no flag: the via rungs' first choice (#1676) and each net's edge
    clearance (#1678). Never raises into the engine."""
    try:
        from fab_tiers import set_rule_via_floors
        set_rule_via_floors(rules.via_floors(0))
    except Exception as e:                                     # noqa: BLE001
        rules.notes.append(f"via rule floors not installed ({e})")
    if pcb_data is None:
        return
    try:
        declared = getattr(config, 'board_edge_declared', None)
        scalar = declared if declared is not None else config.board_edge_clearance
        base = scalar if scalar and scalar > 0 else config.clearance
        floors = EdgeFloors.for_config(rules, pcb_data, config, base)
    except Exception as e:                                     # noqa: BLE001
        rules.notes.append(f"edge_clearance rules not resolved for routing ({e})")
        return
    if floors is None:
        return
    for rule, _ast, value in PadEdgeRules(pcb_data, rules.source or None).copper_rules:
        for own in rules.rules:
            if own.name == rule.name and rule.name in floors.rule_names and not own.honoured:
                own.honoured = (f"edge_clearance min {value:g} mm: the tracks and vias it "
                                f"binds are held to {max(value, floors.fab):g} mm from Edge.Cuts"
                                + (' (the fab floor)' if value < floors.fab - 1e-9 else ''))
    if floors.changed:
        config.board_edge_declared = scalar
        config.board_edge_clearance = floors.strictest
        config.edge_floors = floors


def install_design_rules(config, input_file, pcb_data=None, *, fab_floor=None,
                         cli=None, board=None, net_keepouts=False):
    """Build the board's DesignRules and attach it as ``config.rules``. The
    .kicad_dru / .kicad_pro are discovered beside ``input_file`` (else
    ``pcb_data.source_path``), the same way install_layer_clearances does.
    Prints the unsupported-rule report once per (board, rules). Never raises
    into the engine: a failure leaves ``config.rules`` at None with a note."""
    try:
        path = input_file or getattr(pcb_data, 'source_path', '') or ''
        if board is not None:
            rules = DesignRules.from_pcbnew(board, pcb_data, fab_floor=fab_floor, cli=cli)
        else:
            rules = DesignRules.from_project(pcb_data, path or None, fab_floor=fab_floor,
                                             cli=cli, copper_layers=getattr(config, 'layers', None))
    except Exception as e:                                     # noqa: BLE001
        print(f"  WARNING: design rules not resolved ({e}); legacy channels only")
        try:
            config.rules = None
        except Exception:                                      # noqa: BLE001
            pass
        return None
    # Net-filtered keep-outs (``net_keepouts``: the routing engines, which
    # stamp them): the `disallow` rules on a named rule area, derived against
    # this board before the report, which says which are honoured. Never
    # raises into the engine either.
    if net_keepouts and pcb_data is not None:
        try:
            from net_keepouts import derive_area_keepouts, derive_layer_bans
            # #1527: and the track bans of the rules scoped without an area
            rules.net_keepouts = (derive_area_keepouts(rules, pcb_data)
                                  + derive_layer_bans(rules, pcb_data))
        except Exception as e:                                 # noqa: BLE001
            rules.net_keepouts = []
            rules.notes.append(f"net-filtered keep-outs not derived ({e})")
    try:
        config.rules = rules
    except Exception:                                          # noqa: BLE001
        pass
    _install_rule_floors(config, rules, pcb_data)
    if net_keepouts and getattr(config, 'edge_floors', None) is not None:
        try:
            from net_keepouts import derive_edge_bands
            rules.net_keepouts = list(rules.net_keepouts) + derive_edge_bands(
                config.edge_floors, pcb_data)
        except Exception as e:                                 # noqa: BLE001
            rules.notes.append(f"per-net edge bands not derived ({e})")
    if net_keepouts and pcb_data is not None:
        # #1116: no via in an edge connector's contact finger
        try:
            from net_keepouts import derive_contact_fingers
            rules.net_keepouts = list(rules.net_keepouts) + derive_contact_fingers(pcb_data)
        except Exception as e:                                 # noqa: BLE001
            rules.notes.append(f"contact-finger via keep-outs not derived ({e})")
    key = (os.path.abspath(rules.source) if rules.source else '',
           tuple((r.name, r.unsupported, r.honoured, r.router_unhonoured)
                 for r in rules.rules))
    if key not in _ANNOUNCED:
        _ANNOUNCED.add(key)
        for line in rules.report_lines():
            print(f"  {line}")
    return rules
