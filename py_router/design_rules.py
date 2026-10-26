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
                                                 a floor
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
from typing import Dict, FrozenSet, Iterable, List, Optional, Sequence, Tuple

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
# The kinds a rule may carry without being reported as "parsed, not
# consumed". WIDER than what is read today: the resolver's callers ask only
# for track_width / via_diameter / hole_size (draw_size, floor), copper
# clearance reaches the router only through kicad_dru's layer- and
# track-scoped subsets, and `disallow` through net_keepouts. A rule built only on the other kinds listed here is
# reported neither "NOT honoured" nor "not consumed".
CONSUMED_KINDS = CLEARANCE_KINDS | SIZE_KINDS | {'disallow', 'via_count'}
# Kinds no routing step honours but a grader does: check_drc grades every
# `length` window (kicad_dru.read_board_length_rules, #1313) and every
# diff-pair `skew` / `diff_pair_uncoupled` rule through the same channel
# (#1303).
GRADED_ONLY_KINDS = frozenset({'length', 'skew', 'diff_pair_uncoupled'})

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
    netclasses: FrozenSet[str] = frozenset()  # every class the net is a member of
    effective_class: str = 'Default'         # the aggregate (priority-resolved) class name
    layers: FrozenSet[str] = frozenset()
    footprint_ref: Optional[str] = None
    pad_type: Optional[str] = None           # smd | thru_hole | np_thru_hole | connect
    via_type: Optional[str] = None           # through | micro | blind_buried
    groups: FrozenSet[str] = frozenset()
    diff_pair: Optional[str] = None
    xy: Optional[Tuple[float, float]] = None
    # pad/footprint (clearance ...) override -> KiCad's early-return path
    clearance_override: Optional[float] = None
    # zone (connect_pads (clearance ...)) -> KiCad's post-loop max path
    local_clearance: Optional[float] = None
    plated: bool = True
    # the rule areas the item lies in, for an areas=True condition only
    areas: FrozenSet[str] = frozenset()


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
    # set by net_keepouts.derive_area_keepouts when the router honours the
    # rule as a net-filtered keep-out: what it honours, for the report
    honoured: Optional[str] = None
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
        if rule.condition_src is not None and rule.unsupported is None:
            try:
                rule.condition = parse_condition(rule.condition_src)
                rule.coupled, rule.keep_apart = _coupled_terms(rule.condition)
            except Unsupported as e:
                rule.unsupported = f"condition: {e}"
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
                        rule.unsupported = (f"condition: {e} here; route.py and "
                                            f"route_diff.py read it as a "
                                            f"net-filtered keep-out")
        if rule.layer_clause is not None and rule.unsupported is None:
            lc = rule.layer_clause
            if not (lc in ('outer', 'inner', '*.Cu', '*') or lc.endswith('.Cu')):
                rule.unsupported = f"layer clause {lc!r} (non-copper)"
        rule.unconsumed_kinds = sorted(k for k in rule.constraints
                                       if k not in CONSUMED_KINDS)
        rules.append(rule)
    return rules, notes


# --------------------------------------------------------------------------
# Condition expressions
# --------------------------------------------------------------------------
# Grammar (the KiCad rule language subset we evaluate):
#   expr   := or
#   or     := and ('||' and)*
#   and    := not ('&&' not)*
#   not    := '!' not | atom
#   atom   := '(' expr ')' | cmp
#   cmp    := term (('=='|'!=') term)?
#   term   := ITEM '.' NAME '(' args ')' | ITEM '.' NAME | STRING | NUMBER | 'L'
# Supported properties: NetClass, NetName, Net, Type, Layer, Pad_Type, Via_Type
# Supported functions: hasNetclass, hasExactNetclass, onLayer, existsOnLayer,
#   isMicroVia, isBlindBuriedVia, isPlated, memberOfFootprint, memberOfGroup,
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

_TOKEN = re.compile(r"""
    \s*(?:
        (?P<str>'[^']*'|"[^"]*") |
        (?P<op>==|!=|&&|\|\||<=|>=|[<>!()+\-*/]) |
        (?P<num>-?\d+(?:\.\d+)?\s*(?:mm|mil|in|um|nm)?) |
        (?P<name>[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)?) |
        (?P<comma>,)
    )""", re.VERBOSE)

_SUPPORTED_PROPS = {'netclass', 'netname', 'net', 'type', 'layer', 'pad_type',
                    'via_type'}
_SUPPORTED_FUNCS = {'hasnetclass', 'hasexactnetclass', 'onlayer', 'existsonlayer',
                    'ismicrovia', 'isblindburiedvia', 'isplated',
                    'memberoffootprint', 'memberofgroup', 'indiffpair'}
#: Rule-area predicates. KiCad's insideArea() is the older spelling of
#: intersectsArea() and KiCad 10 still grades it.
_AREA_FUNCS = {'insidearea', 'intersectsarea', 'enclosedbyarea'}
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


class _Parser:
    def __init__(self, src: str, areas: bool = False):
        self.toks = _tokens(src)
        self.i = 0
        self.areas = areas

    def peek(self):
        return self.toks[self.i] if self.i < len(self.toks) else (None, None)

    def take(self, kind=None, val=None):
        k, v = self.peek()
        if k is None or (kind and k != kind) or (val is not None and v != val):
            raise Unsupported(f"unexpected {v!r} (wanted {val or kind})")
        self.i += 1
        return v

    def parse(self):
        node = self.or_()
        if self.i != len(self.toks):
            raise Unsupported(f"trailing {self.peek()[1]!r}")
        return node

    def or_(self):
        node = self.and_()
        while self.peek() == ('op', '||'):
            self.take()
            node = ('or', node, self.and_())
        return node

    def and_(self):
        node = self.not_()
        while self.peek() == ('op', '&&'):
            self.take()
            node = ('and', node, self.not_())
        return node

    def not_(self):
        if self.peek() == ('op', '!'):
            self.take()
            return ('not', self.not_())
        return self.atom()

    def atom(self):
        if self.peek() == ('op', '('):
            self.take()
            node = self.or_()
            self.take('op', ')')
            return node
        return self.cmp()

    def cmp(self):
        lhs = self.term()
        k, v = self.peek()
        if k == 'op' and v in ('==', '!='):
            self.take()
            rhs = self.term()
            return ('cmp', v, lhs, rhs)
        if k == 'op' and v in ('<', '>', '<=', '>=', '+', '-', '*', '/'):
            raise Unsupported(f"numeric comparison/arithmetic ({v})")
        return lhs

    def term(self):
        k, v = self.peek()
        if k == 'str':
            self.take()
            return ('lit', v[1:-1])
        if k == 'num':
            self.take()
            raise Unsupported("numeric literal outside a supported comparison")
        if k == 'name':
            self.take()
            if '.' not in v:
                if v == 'L':
                    return ('L',)
                raise Unsupported(f"bare identifier {v!r}")
            item, name = v.split('.', 1)
            item_u = item.upper()
            if item_u not in ('A', 'B', 'AB'):
                raise Unsupported(f"item {item!r}")
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
                    else:
                        raise Unsupported(f"argument {av!r} to {name}()")
                self.take('op', ')')
                if item_u == 'AB' and lname == 'iscoupleddiffpair' and not args:
                    return ('coupled',)
                if self.areas and lname in _AREA_FUNCS:
                    if len(args) != 1:
                        raise Unsupported(f"{name}() with {len(args)} arguments")
                    return ('area', item_u, lname, args[0])
                if lname in _KNOWN_UNSUPPORTED_FUNCS or lname not in _SUPPORTED_FUNCS:
                    raise Unsupported(f"predicate {name}() not modelled")
                if lname == 'indiffpair' and (item_u == 'AB' or len(args) != 1
                                              or not args[0]):
                    # KiCad reports "Missing diff-pair name argument" and the
                    # term is false; AB has no single net to test.
                    raise Unsupported(f"{item}.{name}() needs one diff-pair "
                                      f"name pattern on A or B")
                return ('call', item_u, lname, tuple(args))
            if lname not in _SUPPORTED_PROPS:
                raise Unsupported(f"property {name} not modelled")
            return ('prop', item_u, lname)
        raise Unsupported(f"unexpected token {v!r}")


def parse_condition(src: str, areas: bool = False):
    """AST for a rule condition, or raise Unsupported. With ``areas`` the
    rule-area predicates parse too, as ('area', item, fn, name) nodes that
    `_eval` answers from RuleItem.areas. Only net_keepouts asks for that, on
    a `disallow` rule: everywhere else an area predicate stays Unsupported,
    so a `!A.intersectsArea()` clearance rule is not read as matching
    every item on the board.

    AB.isCoupledDiffPair() parses only as a term of an AND, positive (#1293,
    the form that gives a pair its own P<->N clearance) or negated (#1295,
    KiCad's "keep pairs apart" rule, which binds everything but the pair's
    own partner). OR'ed or compared it raises Unsupported, so such a rule
    stays unread rather than half-read."""
    node = _Parser(src, areas=areas).parse()
    if _contains_coupled(node) and _coupled_terms(node) is None:
        raise Unsupported("AB.isCoupledDiffPair() is modelled only as a "
                          "term ANDed into the condition, or its negation")
    return node


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


def _norm_type(t: Optional[str]) -> str:
    return (t or '').lower().replace('-', '_').replace(' ', '_')


_PAD_TYPE_NAMES = {'smd': 'smd', 'through_hole': 'thru_hole', 'thru_hole': 'thru_hole',
                   'connector': 'connect', 'connect': 'connect',
                   'npth,_mechanical': 'np_thru_hole', 'npth': 'np_thru_hole',
                   'np_thru_hole': 'np_thru_hole'}
_VIA_TYPE_NAMES = {'through': 'through', 'micro': 'micro', 'blind/buried': 'blind_buried',
                   'blind_buried': 'blind_buried'}


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
        if lhs[0] == 'prop' and rhs[0] == 'prop' and lhs[2] == 'net' and rhs[2] == 'net':
            eq = (lv == rv)
        elif isinstance(lv, str) and isinstance(rv, str):
            eq = _cmp_strings(lhs, lv, rv)
        else:
            eq = (lv == rv)
        return eq if cop == '==' else not eq
    if op == 'call':
        return bool(_call(node, a, b, layer, ctx))
    if op == 'area':
        it = _item(node[1], a, b)
        return it is not None and node[3] in it.areas
    if op == 'prop':
        v = _value(node, a, b, layer)
        return bool(v)
    if op == 'lit':
        return bool(node[1])
    if op == 'L':
        return bool(layer)
    raise Unsupported(f"node {op}")


def _cmp_strings(lhs_node, lv: str, rv: str) -> bool:
    prop = lhs_node[2] if lhs_node[0] == 'prop' else None
    if prop in ('type', 'pad_type', 'via_type'):
        return _norm_type(lv) == _norm_type(rv)
    if prop == 'netclass':
        # NetClass == 'X' is true for any class the net is a member of (its
        # aggregate class included); this is the lenient reading and agrees
        # with KiCad on every single-membership board.
        return lv == rv or rv in lv.split(',')
    if prop == 'layer':
        # KiCad accepts wildcards in layer names ('*.Cu').
        return fnmatch.fnmatchcase(lv, rv) if any(ch in rv for ch in '*?[') else lv == rv
    return lv == rv


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
            return None
        name = node[2]
        if name == 'netclass':
            # the aggregate name plus every membership, joined so the lenient
            # compare above can see both
            names = [it.effective_class] + sorted(it.netclasses - {it.effective_class})
            return ','.join(n for n in names if n)
        if name == 'netname':
            return it.net_name
        if name == 'net':
            return ('net', it.net_id)
        if name == 'type':
            return it.type
        if name == 'layer':
            if layer:
                return layer
            return next(iter(it.layers)) if len(it.layers) == 1 else ''
        if name == 'pad_type':
            return it.pad_type or ''
        if name == 'via_type':
            return it.via_type or ''
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
    if fn == 'hasnetclass':
        return bool(args) and (args[0] in it.netclasses or args[0] == it.effective_class)
    if fn == 'hasexactnetclass':
        return bool(args) and args[0] == it.effective_class
    if fn in ('onlayer', 'existsonlayer'):
        return bool(args) and (args[0] in it.layers or
                               (layer is not None and args[0] == layer and not it.layers))
    if fn == 'ismicrovia':
        return it.type == 'via' and it.via_type == 'micro'
    if fn == 'isblindburiedvia':
        return it.type == 'via' and it.via_type == 'blind_buried'
    if fn == 'isplated':
        return it.plated
    if fn == 'memberoffootprint':
        return bool(args) and it.footprint_ref is not None and \
            fnmatch.fnmatchcase(it.footprint_ref, args[0])
    if fn == 'memberofgroup':
        return bool(args) and args[0] in it.groups
    if fn == 'indiffpair':
        return bool(args) and _in_diff_pair(it, args[0], ctx)
    raise Unsupported(f"predicate {fn}()")


def _rule_matches(rule: Rule, a: RuleItem, b: Optional[RuleItem],
                  layer: Optional[str], copper_layers: Sequence[str],
                  ctx=None) -> bool:
    lay = rule.layers_for(copper_layers)
    if lay is not None:
        if layer is None:
            return False
        if layer not in lay:
            return False
    if rule.condition is None:
        return True
    # KiCad's DRC_RULE_CONDITION::EvaluateFor: "Conditions are commutative" --
    # a two-item condition that fails as (A, B) is tried again as (B, A).
    for x, y in ((a, b), (b, a)) if b is not None else ((a, b),):
        try:
            if _eval(rule.condition, x, y, layer, ctx):
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
    net_names: Dict[int, str] = field(default_factory=dict)
    rules: List[Rule] = field(default_factory=list)
    copper_layers: List[str] = field(default_factory=list)
    fab_floor: Dict[str, float] = field(default_factory=dict)      # fab_tiers FLOOR_KEYS dict, or {}
    cli: Dict[str, float] = field(default_factory=dict)           # explicit CLI sizes by kind
    groups: Dict[str, FrozenSet[str]] = field(default_factory=dict)  # footprint ref -> group names
    source: str = ''
    dru_path: str = ''
    notes: List[str] = field(default_factory=list)
    # net_keepouts.derive_area_keepouts' regions (install_design_rules)
    net_keepouts: List[dict] = field(default_factory=list)
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
            # every membership form, KiCad 6's classes[].nets included (#1304)
            assignments, patterns = project_class_membership(ns)
            rules =((proj.get('board', {}) or {}).get('design_settings', {}) or {}).get('rules', {}) or {}
            for k, v in rules.items():
                if k.startswith('min_') and isinstance(v, (int, float)) and v > 0:
                    self.board_min[k] = float(v)
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
        if layers is None:
            layers = frozenset([layer]) if layer else frozenset()
        return RuleItem(type=type, net_id=net_id,
                        net_name=self.net_names.get(net_id, ''),
                        netclasses=self.memberships.get(net_id, frozenset()),
                        effective_class=self.effective_class(net_id),
                        layers=frozenset(layers), **facts)

    def item_for_pad(self, pad, layer: Optional[str] = None) -> RuleItem:
        ref = getattr(pad, 'component_ref', None)
        return RuleItem(
            type='pad', net_id=int(getattr(pad, 'net_id', 0) or 0),
            net_name=getattr(pad, 'net_name', '') or self.net_names.get(getattr(pad, 'net_id', 0), ''),
            netclasses=self.memberships.get(getattr(pad, 'net_id', 0), frozenset()),
            effective_class=self.effective_class(getattr(pad, 'net_id', 0)),
            layers=frozenset(l for l in (getattr(pad, 'layers', None) or []) if str(l).endswith('.Cu')),
            footprint_ref=ref, pad_type=getattr(pad, 'pad_type', None),
            groups=self.groups.get(ref, frozenset()),
            xy=(getattr(pad, 'global_x', None), getattr(pad, 'global_y', None))
            if getattr(pad, 'global_x', None) is not None else None,
            clearance_override=(getattr(pad, 'local_clearance', 0) or None),
            plated=(getattr(pad, 'pad_type', '') != 'np_thru_hole'))

    # ---------------- resolution ----------------

    def resolve(self, kind: str, a: RuleItem, b: Optional[RuleItem] = None,
                layer: Optional[str] = None) -> Constraint:
        kind = canonical_kind(kind)
        key = (kind, a, b, layer)
        c = self._cache.get(key)
        if c is None:
            c = self._resolve(kind, a, b, layer)
            self._cache[key] = c
        return c

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
        if kind == 'clearance':
            vals = [self.class_value(x.net_id, 'clearance') for x in (a, b) if x is not None]
            vals = [v for v in vals if v is not None]
            if vals:
                v = max(vals)
                cur['min'] = v
                names = sorted({self.effective_class(x.net_id) for x in (a, b) if x is not None})
                source = 'netclass ' + '/'.join(names)
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
            if rule.unsupported or kind not in rule.constraints:
                continue
            if not _rule_matches(rule, a, b, layer, self.copper_layers, ctx):
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

        c = Constraint(kind, min=cur['min'], opt=cur['opt'], max=cur['max'],
                       source=source, kicad_min=cur['min'],
                       min_rule=min_rule.name if min_rule is not None else None,
                       min_rule_coupled=bool(min_rule is not None and min_rule.coupled))
        return self._fab(kind, c)

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
        if not self.has_coupled_rules():
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
        return self.resolve(kind, a, b, layer).min

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
        c = self.resolve(kind, a, None, layer)
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
        return [(r.name, r.unsupported) for r in self.rules
                if r.unsupported and not r.honoured]

    def honoured_rules(self) -> List[Tuple[str, str]]:
        """Rules the resolver does not evaluate but another consumer honours
        (net_keepouts: a `disallow` rule on a named rule area)."""
        return [(r.name, r.honoured) for r in self.rules if r.honoured]

    def unconsumed(self) -> List[Tuple[str, List[str]]]:
        return [(r.name, r.unconsumed_kinds) for r in self.rules
                if not r.unsupported and r.unconsumed_kinds]

    def report_lines(self) -> List[str]:
        out = []
        for name, what in self.honoured_rules():
            out.append(f".kicad_dru rule '{name}': {what}")
        for name, why in self.unsupported():
            out.append(f".kicad_dru rule '{name}': NOT honoured ({why}); "
                       f"KiCad will still enforce it")
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
                'honoured': r.honoured,
                'unconsumed': list(r.unconsumed_kinds),
            } for r in self.rules],
            'fab_floor': dict(sorted(self.fab_floor.items())),
            'cli': dict(sorted(self.cli.items())),
        }


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
            from net_keepouts import derive_area_keepouts
            rules.net_keepouts = derive_area_keepouts(rules, pcb_data)
        except Exception as e:                                 # noqa: BLE001
            rules.net_keepouts = []
            rules.notes.append(f"net-filtered keep-outs not derived ({e})")
    try:
        config.rules = rules
    except Exception:                                          # noqa: BLE001
        pass
    key = (os.path.abspath(rules.source) if rules.source else '',
           tuple((r.name, r.unsupported, r.honoured) for r in rules.rules))
    if key not in _ANNOUNCED:
        _ANNOUNCED.add(key)
        for line in rules.report_lines():
            print(f"  {line}")
    return rules
