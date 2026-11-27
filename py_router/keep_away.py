"""Pairwise keep-away between net groups (#1146).

A rule ``AGGRESSOR:VICTIM:GAP`` asks copper of the nets on one side to stay
GAP mm (edge to edge, same layer) from copper of the nets on the other side.
The clearance is still the hard floor; the keep-away is a SOFT cost above it.
While a net on either side is routed, every grid cell where its track would
sit closer than GAP to the other side's copper (pads, tracks, vias, and its
POURS, #2070) costs ``keep_away_cost`` per cell on the per-layer proximity
map -- the same mm-equivalent unit as every other proximity knob. Nets on the
same side route against each other at the normal clearance.

A pour gives way to a track, so its band is drawn from the zone OUTLINE:
every cell inside it, or within GAP of it, on the zone's layer. The report
measures a pour on KiCad's refill of the shipped copper when the run already
made one (``pour_measured_on: 'kicad_refill'``), else on the outline, which
the fill lies inside (``'outline'``); it never refills for its own sake.

An optional fourth field, ``AGGRESSOR:VICTIM:GAP:adjacent`` (#2070), also
prices the other side's copper on the copper layers NEXT to the routed
layer in the board's stack, in plan view (coupling to the next layer). A rule
without it sees the routed layer only, and the report says so
(``not_measured``), so "0 mm inside a band" is not read as "clear on every
layer".

Within ``keep_away_free`` mm of the routed net's OWN pads the band is not
priced, so a pin can leave a package whose neighbouring pins belong to the
other group (a relay's coil pin beside its contact pins, a converter's clock
pin beside its analog pins).

Because the cost is soft, a band the router cannot avoid is still routed
through. ``keep_away_report`` measures the result on the copper itself -- per
net, the length that ended up inside a band beyond the free radius -- and the
route step carries it as ``JSON_SUMMARY['keep_away']``, so a run can be graded.

Rule syntax: each side is a comma-separated list of net patterns, resolved by
``expand_net_patterns`` exactly like ``--nets`` (``*``/``?`` wildcards,
``!`` exclusions), and/or net-class terms ``class=NAME`` (NAME may be a glob;
``!class=NAME`` takes a class out), so ``class=Digital:class=Audio:0.5`` is
KiCad's ``A.NetClass == 'Digital' && B.NetClass == 'Audio'`` as a soft rule.
Classes come from the board's project through the resolver the router and
check_drc share; a net no class claims is in ``Default``. Several rules may
share one string, separated by whitespace or ``;`` (the GUI's one-line
field). Net patterns and class names may therefore not contain ``:``, ``,``,
``;`` or whitespace; use ``?`` for such a character (``class=High?Speed``).

Rules may also be DECLARED once, in the board's design brief
(``<board>.design-brief.json``, key ``keep_away``, #1253). The routing
engines read that sibling the way they read ``.kicad_dru`` and add its rules
to the caller's (``resolve_keep_away``), so the CLI and the GUI both honour
it with no flag; check_floorplan grades the same rules on the PLACED board,
pad edge to pad edge, before any track exists (``floorplan.rule_keep_away``).
The planes step's via stitching reads them too (#2066) and, having no cost
search, holds them as a site gate (``ViaBandGate``).
"""

from __future__ import annotations

import contextlib
import io
import json
import math
import os
import re
from collections import OrderedDict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from routing_config import GridCoord
from routing_utils import _capsule_mask, pad_blocked_cells_array

# (layer, gx, gy) packed into one int64, as obstacle_costs packs its SUM rows:
# grid coordinates are well inside +/-2^23.
_OFF = 1 << 23
_XY_MASK = (1 << 48) - 1
_COORD_MASK = (1 << 24) - 1

# A side term naming a net CLASS rather than nets: `class=Audio`, or
# `!class=Default` to take a class out. (`:` already separates the rule.)
CLASS_PREFIX = 'class='

# Report tolerance (mm). A track whose cells all lie outside the priced band
# can still come a few microns closer than GAP between two cell centres (a
# chord across a round pad's band), so spacing within this much of GAP is not
# reported as inside the band.
REPORT_TOL = 0.01
# Sample pitch (mm) along each track for the report.
_REPORT_STEP = 0.05
# The widest GAP a rule may ask for (mm). A band is rasterised around every
# copper item of the other side, so its cost grows with GAP squared: a typo
# such as 50 for 0.5 would allocate gigabytes.
MAX_GAP = 10.0
# Memory the cached bands may hold (bytes): per-net fields, and composites.
_FIELD_BYTES_MAX = 256 * 2 ** 20
_COMPOSITE_BYTES_MAX = 256 * 2 ** 20


#: The optional fourth field of a rule (#2070): also price the other side's
#: copper on the copper layers next to the routed one, in plan view.
ADJACENT = 'adjacent'


@dataclass(frozen=True)
class KeepAwayRule:
    aggressor: Tuple[str, ...]
    victim: Tuple[str, ...]
    gap: float
    adjacent: bool = False

    @property
    def sides(self) -> str:
        return f"{','.join(self.aggressor)}:{','.join(self.victim)}"

    @property
    def spec(self) -> str:
        return (f"{self.sides}:{self.gap:.12g}"
                + (f":{ADJACENT}" if self.adjacent else ''))


def _is_number(text: str) -> bool:
    try:
        float(text)
    except ValueError:
        return False
    return True


def split_keep_away_specs(specs) -> List[str]:
    """One token per rule. A CLI value or the GUI's text may hold several
    rules separated by whitespace or ';'."""
    if not specs:
        return []
    if isinstance(specs, str):
        specs = [specs]
    out = []
    for s in specs:
        out.extend(t for t in re.split(r'[;\s]+', s or '') if t)
    return out


def parse_keep_away_rules(specs) -> Tuple[KeepAwayRule, ...]:
    """Parse ``AGGRESSOR:VICTIM:GAP[:adjacent]`` rules; raises ValueError
    naming the bad rule."""
    rules = []
    raw = [specs] if isinstance(specs, str) else list(specs or ())
    # Rules are split on whitespace, so a name with a space in it arrives
    # here in pieces; say so rather than blame the piece.
    hint = (" -- rules are separated by spaces, so write a space inside a net "
            "or class name as '?'" if any(re.search(r'\s', s or '') for s in raw)
            else "")
    for tok in split_keep_away_specs(specs):
        # The optional fourth field first (#2070): GAP is then the third.
        body, adjacent = tok, False
        lead, sep0, opt = tok.rpartition(':')
        if sep0 and opt.lower() == ADJACENT:
            body, adjacent = lead, True
        head, sep, gap_s = body.rpartition(':')
        agg_s, sep2, vic_s = head.partition(':')
        if (sep and sep2 and ':' in vic_s and not _is_number(gap_s)
                and _is_number(head.rpartition(':')[2])):
            raise ValueError(
                f"keep-away rule {tok!r}: unknown option {gap_s!r} after GAP "
                f"(the one option is ':{ADJACENT}'){hint}")
        if not sep or not sep2 or ':' in vic_s:
            raise ValueError(
                f"keep-away rule {tok!r}: expected AGGRESSOR:VICTIM:GAP"
                f"[:{ADJACENT}] (net patterns may not contain ':'){hint}")
        try:
            gap = float(gap_s)
        except ValueError:
            raise ValueError(f"keep-away rule {tok!r}: GAP {gap_s!r} is not "
                             f"a number of mm{hint}") from None
        if not (math.isfinite(gap) and 0 < gap <= MAX_GAP):
            raise ValueError(f"keep-away rule {tok!r}: GAP must be > 0 and "
                             f"<= {MAX_GAP:g} mm")
        agg = tuple(p for p in agg_s.split(',') if p)
        vic = tuple(p for p in vic_s.split(',') if p)
        if not agg or not vic:
            raise ValueError(f"keep-away rule {tok!r}: both net groups need "
                             f"at least one pattern")
        if any(p.lstrip('!') == CLASS_PREFIX for p in agg + vic):
            raise ValueError(f"keep-away rule {tok!r}: '{CLASS_PREFIX}' needs a "
                             f"net-class name")
        rules.append(KeepAwayRule(agg, vic, gap, adjacent))
    return tuple(rules)


def normalize_keep_away_specs(specs) -> Tuple[str, ...]:
    """Validated, canonical rule strings for GridRouteConfig.keep_away."""
    return tuple(r.spec for r in parse_keep_away_rules(specs))


# --------------------------------------------------------------------------
# The design brief's `keep_away` key (#1253): ONE declaration for both the
# placement grade (check_floorplan, through the compiled intent) and the
# router (batch_route / batch_route_diff_pairs, through the sibling file).

#: The board's sibling design brief (#711). The same string as
#: `placement.design_brief.BRIEF_EXT`, which this package cannot import;
#: tests/test_1253_keep_away_placement.py pins the two equal.
BRIEF_EXT = '.design-brief.json'
#: The brief key, and the keys an object row may carry: `rule` plus the
#: brief's usual prose slots, which the compiler moves into `context`.
BRIEF_KEY = 'keep_away'
BRIEF_ROW_KEYS = ('rule', 'why', 'requirement', 'note', 'context')
#: "I do not know" (#711): the author looked and cannot say yet which nets
#: are noisy or sensitive. Compiles to no rule and is reported as unknown.
BRIEF_UNKNOWN = 'unknown'


def brief_rule_rows(value, where: str = BRIEF_KEY) -> List[dict]:
    """Validate a design brief's ``keep_away`` value; one row per rule.

    The value is a list whose entries are each ONE rule in route.py's own
    syntax -- a string, or ``{"rule": "...", "why": ...}`` -- or the string
    "unknown" (no rows). Returns ``[{'row': i, 'rule': canonical spec,
    <prose keys>}]``. The ONLY reader of the key: `design_brief` validates
    the brief through it and the router reads the brief through it, so the
    two cannot disagree about what a row says. Raises ValueError naming the
    row and carrying the rule parser's own message."""
    if value is None or value == BRIEF_UNKNOWN:
        return []
    if not isinstance(value, list):
        raise ValueError(f"{where}: expected a list of rules "
                         f"('AGGRESSOR:VICTIM:GAP[:adjacent]') or "
                         f"{BRIEF_UNKNOWN!r}, got "
                         f"{type(value).__name__}")
    rows: List[dict] = []
    seen: Dict[str, int] = {}
    for i, entry in enumerate(value):
        at = f"{where}[{i}]"
        if isinstance(entry, dict):
            bad = sorted(set(entry) - set(BRIEF_ROW_KEYS))
            if bad:
                raise ValueError(f"{at}: unknown key(s) {', '.join(bad)} "
                                 f"(a row carries {', '.join(BRIEF_ROW_KEYS)})")
            text = entry.get('rule')
            if entry.get('context') is not None \
                    and not isinstance(entry['context'], dict):
                raise ValueError(f"{at}.context: expected an object")
        else:
            text = entry
        if not isinstance(text, str) or not text.strip():
            raise ValueError(f"{at}: expected a rule "
                             f"'AGGRESSOR:VICTIM:GAP[:adjacent]' "
                             f"(or an object with one in `rule`), got "
                             f"{text!r}")
        try:
            parsed = parse_keep_away_rules(text)
        except ValueError as exc:
            raise ValueError(f"{at}: {exc}") from None
        if len(parsed) != 1:
            raise ValueError(f"{at}: {text!r} holds {len(parsed)} rules; write "
                             f"one rule per entry, so each is graded and "
                             f"reported under its own row")
        spec = parsed[0].spec
        if spec in seen:
            raise ValueError(f"{at}: {spec!r} repeats {where}[{seen[spec]}] "
                             f"-- one claim, declared twice")
        seen[spec] = i
        row = {'row': i, 'rule': spec}
        if isinstance(entry, dict):
            row.update({k: entry[k] for k in BRIEF_ROW_KEYS[1:]
                        if entry.get(k) is not None})
        rows.append(row)
    return rows


def brief_path_for(board_path: str) -> str:
    """``<board>.design-brief.json``, or '' for an empty path."""
    return os.path.splitext(board_path)[0] + BRIEF_EXT if board_path else ''


def read_brief_keep_away(board_path: str = '', pcb_data=None
                         ) -> Tuple[Tuple[str, ...], str]:
    """(canonical rules, brief path) the board's sibling design brief
    declares under ``keep_away``; ((), '') when it has no brief or declares
    none. Discovered the way `.kicad_dru` is (#498): ``board_path`` when the
    caller has one, else ``PCBData.source_path``.

    Raises ValueError for a brief that cannot be read, or whose
    ``keep_away`` is malformed: a declared rule the router silently skipped
    is a constraint the author believes is honoured and nothing prices. Only
    the ``keep_away`` key is read here; check_floorplan validates the rest."""
    path = brief_path_for(board_path
                          or getattr(pcb_data, 'source_path', '') or '')
    if not path or not os.path.isfile(path):
        return (), ''
    try:
        with open(path, encoding='utf-8') as fh:
            raw = json.load(fh)
    except (OSError, ValueError) as exc:
        raise ValueError(f"cannot read design brief {path}: {exc}") from None
    if not isinstance(raw, dict):
        raise ValueError(f"design brief {path}: expected a JSON object")
    rows = brief_rule_rows(raw.get(BRIEF_KEY),
                           where=f"design brief {os.path.basename(path)} "
                                 f"{BRIEF_KEY}")
    return tuple(r['rule'] for r in rows), (path if rows else '')


#: (brief path, rules) already announced in this process: a GUI session and
#: a run's nested sub-runs resolve the same brief more than once.
_BRIEF_ANNOUNCED: set = set()


def resolve_keep_away(keep_away, input_file: str = '', pcb_data=None,
                      announce: bool = True) -> Optional[List[str]]:
    """The keep-away rules a routing run prices: the caller's (``--keep-away``
    / the GUI field) first, then each rule the board's design brief declares
    that is not already among them (#1253). None when there are none, so a
    board with no declaration and no flag routes exactly as before.

    Engine-side, so both fronts get it with no flag and no control: the CLI
    passes ``input_file``, the GUI its board file, and an engine without one
    is found through ``PCBData.source_path``."""
    explicit = list(normalize_keep_away_specs(keep_away)) if keep_away else []
    declared, path = read_brief_keep_away(input_file, pcb_data)
    added = [s for s in declared if s not in explicit]
    if announce and declared:
        key = (os.path.abspath(path), declared, tuple(explicit))
        if key not in _BRIEF_ANNOUNCED:
            _BRIEF_ANNOUNCED.add(key)
            if added:
                print(f"Keep-away: {len(added)} rule(s) read from the design "
                      f"brief {os.path.basename(path)} (#1253): "
                      + ' '.join(repr(s) for s in added))
            else:
                print(f"Keep-away: the design brief {os.path.basename(path)} "
                      f"declares {len(declared)} rule(s), all already given "
                      f"(#1253)")
    merged = explicit + added
    return merged or None


def unpriced_rules(rules, board_path: str = '', pcb_data=None) -> Dict:
    """Which of ``rules`` the routing engines will NOT price on this board.

    route.py, route_diff.py and the GUI read keep-away rules from exactly two
    places: the run's own ``--keep-away`` (or the GUI field), and the design
    brief beside the board (``resolve_keep_away``). A rule a floorplan
    INTENT carries without the brief is graded on the placement by
    check_floorplan and priced by no router, so a grade that passes says
    nothing about the tracks. Answered from the same reader the engines use
    (``read_brief_keep_away``), compared in canonical spelling.

    Returns ``{'rules': [canonical rules not priced], 'brief': the brief the
    engines read, or '' when there is none, 'brief_declares': its rules,
    'brief_error': why that brief cannot be read (the engines then refuse
    to route), or None}``."""
    try:
        declared, path = read_brief_keep_away(board_path, pcb_data)
        err = None
    except ValueError as exc:
        declared, err = (), str(exc)
        path = brief_path_for(board_path
                              or getattr(pcb_data, 'source_path', '') or '')
    if not path:
        cand = brief_path_for(board_path
                              or getattr(pcb_data, 'source_path', '') or '')
        path = cand if cand and os.path.isfile(cand) else ''
    out: List[str] = []
    for r in rules or ():
        for spec in normalize_keep_away_specs(r):
            if spec not in declared and spec not in out:
                out.append(spec)
    return {'rules': out, 'brief': path, 'brief_declares': list(declared),
            'brief_error': err}


def resolve_rule_sides(pcb_data, rules) -> Tuple[
        List[Tuple[frozenset, frozenset, float]], List[str], List[str]]:
    """``([(aggressor ids, victim ids, gap)] per rule, notes, unmatched
    specs)`` -- each rule's two sides resolved against ``pcb_data`` exactly
    as the router resolves them (net patterns as ``--nets``, ``class=``
    terms through the board's project). For a grader that measures the
    rules without routing (check_floorplan's ``keep_away``, #1253)."""
    st = _KeepAwayState(pcb_data, tuple(rules))
    return list(st.groups), list(st.notes), list(st.unmatched)


def keep_away_knob_error(free: float, cost: float) -> Optional[str]:
    """Why --keep-away-free / --keep-away-cost is refused, or None. The range
    is the one the GUI's spin controls offer, so a CLI value a plan carries
    over is never clamped silently there."""
    import routing_defaults as defaults
    for flag, name, v in (('--keep-away-free', 'keep_away_free', free),
                          ('--keep-away-cost', 'keep_away_cost', cost)):
        r = defaults.PARAM_RANGES[name]
        if not (math.isfinite(v) and r['min'] <= v <= r['max']):
            return f"{flag} {v:g} is outside {r['min']:g}..{r['max']:g}"
    return None


def _pack(layer_idx: int, gx: np.ndarray, gy: np.ndarray) -> np.ndarray:
    return ((np.int64(layer_idx) << 48)
            | ((gx.astype(np.int64) + _OFF) << 24)
            | (gy.astype(np.int64) + _OFF))


def _pack_xy(cells: np.ndarray) -> np.ndarray:
    return (((cells[:, 0].astype(np.int64) + _OFF) << 24)
            | (cells[:, 1].astype(np.int64) + _OFF))


def _pad_corner_radius(pad) -> float:
    hw, hh = pad.size_x / 2, pad.size_y / 2
    if pad.shape in ('circle', 'oval'):
        return min(hw, hh)
    if pad.shape == 'roundrect':
        return getattr(pad, 'roundrect_rratio', 0.25) * min(pad.size_x, pad.size_y)
    return 0.0


def _pad_cells(pad, margin: float, coord: GridCoord) -> np.ndarray:
    """(N, 2) cells whose centre lies within `margin` of the pad's copper."""
    step = coord.grid_step
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    return pad_blocked_cells_array(
        gx, gy, pad.size_x / 2, pad.size_y / 2, margin, step,
        _pad_corner_radius(pad),
        off_x=pad.global_x - gx * step, off_y=pad.global_y - gy * step,
        rotation_deg=getattr(pad, 'rect_rotation', 0.0) or 0.0)


def _pad_layers(pad, routing_layers) -> List[str]:
    from net_queries import expand_pad_layers
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return []          # no copper, only a hole
    return [L for L in expand_pad_layers(pad.layers, routing_layers)
            if L in routing_layers]


def _pads_sig(pcb_data, net_id: int) -> tuple:
    """A net's pads as geometry, order-free: what its band and its free
    radius are drawn from besides its copper."""
    return tuple(sorted(
        (p.global_x, p.global_y, p.size_x, p.size_y, str(p.shape),
         float(getattr(p, 'rect_rotation', 0.0) or 0.0),
         tuple(p.layers or ()), str(getattr(p, 'pad_type', '')))
        for p in pcb_data.pads_by_net.get(net_id, [])))


def _layer_key(name: str):
    """F.Cu, In1.Cu, In2.Cu, ..., B.Cu: KiCad's copper order by name."""
    if name == 'F.Cu':
        return (0, 0)
    if name == 'B.Cu':
        return (2, 0)
    m = re.match(r'In(\d+)\.Cu$', name)
    return (1, int(m.group(1))) if m else (1, 10 ** 6)


def copper_stack(pcb_data, routing_layers) -> List[str]:
    """The board's copper layers top to bottom (#2070): the stackup's copper
    order when it names every layer in play, else the KiCad names' order.
    What `AGGRESSOR:VICTIM:GAP:adjacent` reads "next to" in."""
    bi = getattr(pcb_data, 'board_info', None)
    names = set(getattr(bi, 'copper_layers', None) or ()) | set(routing_layers)
    stack = [getattr(sl, 'name', '') for sl in (getattr(bi, 'stackup', None) or ())
             if getattr(sl, 'layer_type', '') == 'copper']
    if stack and names <= set(stack):
        return [L for L in stack if L in names] if names else stack
    return sorted(names, key=_layer_key)


def adjacent_layers(stack: List[str], layer: str) -> List[str]:
    """The copper layers directly above and below `layer` in `stack`."""
    if layer not in stack:
        return []
    i = stack.index(layer)
    return [stack[j] for j in (i - 1, i + 1) if 0 <= j < len(stack)]


def _zone_cells(zone, margin: float, coord: GridCoord, clip=None) -> np.ndarray:
    """(N, 2) cells whose centre lies inside `zone`'s outline or within
    `margin` of it, less the cells deeper than `margin` inside one of its
    cut-outs (#2070). A pour gives way to a track, so the outline -- not a
    fill -- is what a track must keep its distance from."""
    from obstacle_map import _box_masked_cells, _rasterize_polygon_box
    gx_lo, gy_lo, nx, _ny, inside, edge = _rasterize_polygon_box(
        zone.polygon, coord, margin, clip_bounds=clip)
    if inside is None:
        return np.empty((0, 2), dtype=np.int32)
    gx, gy = _box_masked_cells(gx_lo, gy_lo, nx, inside | (edge <= margin))
    cells = np.column_stack([gx, gy])
    for hole in getattr(zone, 'holes', None) or ():
        h = _rasterize_polygon_box(hole, coord, margin, clip_bounds=clip)
        if h[4] is None or not len(cells):
            continue
        hx, hy = _box_masked_cells(h[0], h[1], h[2], h[4] & ~(h[5] <= margin))
        if len(hx):
            cut = _pack_xy(np.column_stack([hx, hy]))
            cells = cells[~np.isin(_pack_xy(cells), cut)]
    return cells


def _routed_half_width(config, net_ids, single_track=False) -> float:
    """Half the copper the routed object puts either side of the searched
    centreline: one track, or a diff pair's P + gap + N (a hybrid pair's
    single-ended legs are one track each)."""
    w = max(config.get_net_track_width(n, L)
            for n in net_ids for L in config.layers)
    if len(net_ids) == 2 and not single_track:
        return w + config.diff_pair_gap / 2
    return w / 2


class _KeepAwayState:
    """Rules resolved against one board, plus the caches the per-net field is
    assembled from. Lives on the PCBData (``_keep_away_state``) so it follows
    the board the run is routing, keyed by the rules and the grid."""

    def __init__(self, pcb_data, rules: Tuple[KeepAwayRule, ...]):
        self.rules = rules
        self.groups: List[Tuple[frozenset, frozenset, float]] = []
        self.unmatched: List[str] = []
        self.notes: List[str] = []
        self._classes = None
        # Each side resolves quietly (a name with a space written '?' is a
        # wildcard, and expand_net_patterns counts every wildcard's matches):
        # what a pattern got wrong lands in `notes`, once however many sides
        # list it, and `announce` gives each rule one line of side counts.
        for r in rules:
            sides = [self._resolve_side(pcb_data, pats)
                     for pats in (r.aggressor, r.victim)]
            if not sides[0] or not sides[1]:
                self.unmatched.append(r.spec)
            self.groups.append((sides[0], sides[1], r.gap))
        # Parallel to `groups` (kept a 3-tuple for resolve_rule_sides' readers):
        # which rules also price the layers next to the routed one (#2070).
        self.adjacent: List[bool] = [r.adjacent for r in rules]
        self.members = frozenset().union(*(a | v for a, v, _g in self.groups))
        self.copper: Dict[int, tuple] = {}
        self._fields: "OrderedDict[tuple, np.ndarray]" = OrderedDict()
        self._field_bytes = 0
        self._exempt: Dict[tuple, np.ndarray] = {}
        self._composites: "OrderedDict[tuple, Optional[np.ndarray]]" = OrderedDict()

    def _note(self, text: str) -> None:
        if text and text not in self.notes:
            self.notes.append(text)

    def announce(self) -> None:
        """One line per rule with the nets each side resolved to, then what
        did not resolve."""
        for i, (r, (a, v, _g)) in enumerate(zip(self.rules, self.groups), 1):
            sides = r.sides
            if len(sides) > 48:
                sides = sides[:45] + '...'
            tail = r.spec[len(r.sides):]
            print(f"Keep-away rule {i} '{sides}{tail}': {len(a)} net(s) "
                  f"vs {len(v)} net(s)")
        for note in self.notes:
            print(f"WARNING: keep-away: {note}")
        for spec in self.unmatched:
            print(f"WARNING: keep-away rule '{spec}' has a side that matches "
                  f"no net on this board; it does nothing.")

    def _class_members(self, pcb_data) -> Dict[int, frozenset]:
        """{net_id: its net-class names} from the board's project, through
        the resolver the router and check_drc share; a net no class claims
        is in 'Default'."""
        if self._classes is None:
            from list_nets import net_class_memberships
            nets = {nid: n.name for nid, n in pcb_data.nets.items()
                    if nid and n.name}
            path = getattr(pcb_data, 'source_path', '') or ''
            try:
                raw = net_class_memberships(path, nets) if path else {}
            except Exception:                               # noqa: BLE001
                raw = {}
            self._classes = {nid: frozenset(raw.get(nid) or ('Default',))
                             for nid in nets}
        return self._classes

    def _resolve_side(self, pcb_data, pats) -> frozenset:
        """Net ids one side names. Without a class term the side is exactly
        what expand_net_patterns makes of it (as --nets). With one, the side
        is the union of its included net patterns and `class=NAME` terms
        (NAME may be a glob) minus its `!` terms of either kind -- the name
        terms read exactly as --nets reads them (a `!NAME` that names a real
        active-low net is that net, #177), and, as --nets does, a class never
        brings in an `unconnected-*` net."""
        from fnmatch import fnmatchcase
        from net_queries import expand_net_patterns
        name_to_id: Dict[str, int] = {}
        for nid, net in pcb_data.nets.items():
            if nid and net.name:
                name_to_id.setdefault(net.name, nid)

        def by_name(patterns, report=True):
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                names = expand_net_patterns(pcb_data, list(patterns), quiet=True)
            if report:
                for line in buf.getvalue().splitlines():
                    self._note(re.sub(r'^\s*warning:\s*', '', line, flags=re.I))
            return {name_to_id[n] for n in names if n in name_to_id}
        if not any(p.lstrip('!').startswith(CLASS_PREFIX) for p in pats):
            return frozenset(by_name(pats))
        classes = self._class_members(pcb_data)

        def by_class(cls_glob):
            ids = {nid for nid, cs in classes.items()
                   if any(fnmatchcase(c, cls_glob) for c in cs)
                   and not pcb_data.nets[nid].name.lower().startswith('unconnected-')}
            if not ids:
                known = sorted({c for cs in classes.values() for c in cs})
                self._note(f"class '{cls_glob}' has no nets on this board "
                           f"(its classes: {', '.join(known) or 'none'})")
            return ids
        incl, excl, incl_cls, excl_cls = [], [], [], []
        for p in pats:
            neg = p.startswith('!')
            body = p[1:] if neg else p
            if body.startswith(CLASS_PREFIX):
                (excl_cls if neg else incl_cls).append(body[len(CLASS_PREFIX):])
            elif neg and p not in name_to_id:
                excl.append(p)
            else:
                incl.append(p)
        everything = by_name(['*'], report=False)   # a reference set, not a request
        ids = by_name(incl) if incl else set()
        for c in incl_cls:
            ids |= by_class(c)
        if not incl and not incl_cls:  # exclusions only: everything else
            ids = set(everything)
        if excl:
            ids -= everything - by_name(['*'] + excl)
        for c in excl_cls:
            ids -= by_class(c)
        return frozenset(ids)

    def opposite(self, net_ids: Iterable[int],
                 adjacent: bool = False) -> Dict[int, float]:
        """{other net: GAP} the routed net(s) must keep away from; the largest
        GAP wins where rules overlap. `adjacent` asks only the rules that also
        price the layers next to the routed one (`:adjacent`, #2070)."""
        own = set(net_ids)
        out: Dict[int, float] = {}
        for (a, v, gap), adj in zip(self.groups, self.adjacent):
            if adjacent and not adj:
                continue
            for n in own:
                others = (v if n in a else frozenset()) | (a if n in v else frozenset())
                for o in others:
                    if o not in own and out.get(o, 0.0) < gap:
                        out[o] = gap
        return out

    def refresh(self, pcb_data) -> None:
        """Index the member nets' tracks, vias and pours as they are NOW.

        Rebuilt on every call rather than once per `_copper_epoch`: rip-up
        and restore bump the epoch, but over a hundred other sites edit
        pcb_data.segments / .vias in place (a segment swapped for another, a
        coordinate nudged) without bumping it or changing a length, and a
        band built from copper that has since moved prices the wrong cells.
        A net's signature is its geometry, order-free, so a ripped and
        restored net gets its cached band back. Its PADS are part of it: a
        polarity swap moves a net's pads without touching its copper. So are
        its POURS (#2070), KiCad's generated teardrops left out
        (`pour_zones`): a teardrop is routing output, not a pour."""
        from kicad_parser import pour_zones
        segs: Dict[int, list] = {n: [] for n in self.members}
        vias: Dict[int, list] = {n: [] for n in self.members}
        zones: Dict[int, list] = {n: [] for n in self.members}
        for z in pour_zones(getattr(pcb_data, 'zones', None)):
            if z.net_id in zones and len(z.polygon or ()) >= 3:
                zones[z.net_id].append(z)
        for s in pcb_data.segments:
            if s.net_id in segs and not getattr(s, 'graphic', False):
                segs[s.net_id].append(s)
        for v in pcb_data.vias:
            if v.net_id in vias:
                vias[v.net_id].append(v)
        self.copper = {}
        for n in self.members:
            sig = hash((tuple(sorted((s.start_x, s.start_y, s.end_x, s.end_y,
                                      s.width, s.layer) for s in segs[n])),
                        tuple(sorted((v.x, v.y, v.size, tuple(v.layers or ()))
                                     for v in vias[n])),
                        _pads_sig(pcb_data, n),
                        tuple(sorted((z.layer, tuple(map(tuple, z.polygon)),
                                      tuple(tuple(map(tuple, h))
                                            for h in (z.holes or ())))
                                     for z in zones[n]))))
            self.copper[n] = (sig, segs[n], vias[n], zones[n])

    def field(self, pcb_data, config, net_id: int, margin: float,
              window=None, adjacent: bool = False) -> np.ndarray:
        """Packed cells within `margin` of `net_id`'s copper on the same layer
        (a band's GAP plus the routed track's half-width: the cells where
        that track would sit closer than GAP). A pour's band is drawn from its
        outline (`_zone_cells`). With `adjacent` (#2070) the band is the one
        the same copper casts, in plan view, onto the copper layers next to
        its own in the board's stack -- copper on a layer the run does not
        route included -- and not its own layer's. With `window` (x0, y0, x1,
        y1 in mm) only the copper whose band reaches the window is
        rasterised, and nothing is cached: a rescue rung routes inside a
        small window, at a grid fine enough that the whole board's band is ~1
        minute and ~0.5 GB per net."""
        from connectivity import via_copper_layers
        sig, segs, vias, zones = self.copper[net_id]
        margin = round(margin, 9)
        key = (net_id, margin, sig, adjacent)
        if window is None:
            hit = self._fields.get(key)
            if hit is not None:
                self._fields.move_to_end(key)
                return hit
        pads = pcb_data.pads_by_net.get(net_id, [])
        clip = None
        if window is not None:
            x0, y0, x1, y1 = window

            def meets(lx, ly, hx, hy, r):
                return lx - r <= x1 and hx + r >= x0 and ly - r <= y1 and hy + r >= y0
            segs = [s for s in segs if meets(
                min(s.start_x, s.end_x), min(s.start_y, s.end_y),
                max(s.start_x, s.end_x), max(s.start_y, s.end_y),
                (s.width or 0.0) / 2 + margin)]
            vias = [v for v in vias if meets(v.x, v.y, v.x, v.y,
                                             (v.size or 0.0) / 2 + margin)]
            pads = [p for p in pads if meets(
                p.global_x, p.global_y, p.global_x, p.global_y,
                math.hypot(p.size_x, p.size_y) / 2 + margin)]
            clip = (x0 - config.grid_step, y0 - config.grid_step,
                    x1 + config.grid_step, y1 + config.grid_step)
        elif zones:
            bb = getattr(getattr(pcb_data, 'board_info', None), 'board_bounds',
                         None)
            if bb:
                clip = (bb[0] - margin, bb[1] - margin,
                        bb[2] + margin, bb[3] + margin)
        coord = GridCoord(config.grid_step)
        step = config.grid_step
        lidx = {L: i for i, L in enumerate(config.layers)}
        if adjacent:
            stack = copper_stack(pcb_data, config.layers)
            span = stack

            def onto(layers):
                return sorted({lidx[N] for L in layers
                               for N in adjacent_layers(stack, L) if N in lidx})
        else:
            span = config.layers

            def onto(layers):
                return sorted({lidx[L] for L in layers if L in lidx})
        parts = []
        for s in segs:
            targets = onto((s.layer,))
            if not targets:
                continue
            _xs, _ys, gxg, gyg, mask = _capsule_mask(
                s.start_x, s.start_y, s.end_x, s.end_y,
                (s.width or 0.0) / 2 + margin, step)
            for li in targets:
                parts.append(_pack(li, gxg[mask], gyg[mask]))
        for v in vias:
            targets = onto(via_copper_layers(v, span))
            if not targets:
                continue
            _xs, _ys, gxg, gyg, mask = _capsule_mask(
                v.x, v.y, v.x, v.y, (v.size or 0.0) / 2 + margin, step)
            gx, gy = gxg[mask], gyg[mask]
            for li in targets:
                parts.append(_pack(li, gx, gy))
        for pad in pads:
            targets = onto(_pad_layers(pad, span))
            if not targets:
                continue
            cells = _pad_cells(pad, margin, coord)
            for li in targets:
                parts.append(_pack(li, cells[:, 0], cells[:, 1]))
        for z in zones:
            targets = onto((z.layer,))
            if not targets:
                continue
            cells = _zone_cells(z, margin, coord, clip)
            if len(cells):
                for li in targets:
                    parts.append(_pack(li, cells[:, 0], cells[:, 1]))
        out = (np.unique(np.concatenate(parts)) if parts
               else np.empty(0, dtype=np.int64))
        if window is None:
            self._fields[key] = out
            self._field_bytes += out.nbytes
            while self._field_bytes > _FIELD_BYTES_MAX and len(self._fields) > 1:
                self._field_bytes -= self._fields.popitem(last=False)[1].nbytes
        return out

    def exempt(self, pcb_data, config, net_ids: Tuple[int, ...],
               radius: float) -> np.ndarray:
        """Packed (gx, gy) cells within `radius` of the routed net's own pads."""
        key = (net_ids, radius, tuple(_pads_sig(pcb_data, n) for n in net_ids))
        hit = self._exempt.get(key)
        if hit is not None:
            return hit
        coord = GridCoord(config.grid_step)
        parts = [_pack_xy(_pad_cells(pad, radius, coord))
                 for n in net_ids for pad in pcb_data.pads_by_net.get(n, [])
                 if _pad_layers(pad, config.layers)]
        out = (np.unique(np.concatenate(parts)) if parts
               else np.empty(0, dtype=np.int64))
        if len(self._exempt) >= 256:
            self._exempt.clear()
        self._exempt[key] = out
        return out

    def rows(self, pcb_data, config, net_ids: Tuple[int, ...],
             single_track: bool = False, window=None) -> Optional[np.ndarray]:
        opp = self.opposite(net_ids)
        if not opp:
            return None
        opp_adj = self.opposite(net_ids, adjacent=True)
        self.refresh(pcb_data)
        cost = config.cell_cost(config.keep_away_cost)
        free = max(0.0, float(config.keep_away_free or 0.0))
        half_w = _routed_half_width(config, net_ids, single_track)
        parts_key = tuple(sorted((o, g, self.copper[o][0], False)
                                 for o, g in opp.items() if o in self.copper))
        parts_key += tuple(sorted((o, g, self.copper[o][0], True)
                                  for o, g in opp_adj.items() if o in self.copper))
        ckey = (net_ids, half_w, free, cost, parts_key)
        if window is None and ckey in self._composites:
            self._composites.move_to_end(ckey)
            return self._composites[ckey]
        fields = [f for f in (self.field(pcb_data, config, o, g + half_w, window,
                                         adjacent=adj)
                              for o, g, _s, adj in parts_key) if len(f)]
        rows = None
        if fields:
            ex = (self.exempt(pcb_data, config, net_ids, free) if free > 0
                  else np.empty(0, dtype=np.int64))
            # A NEW array per composition: merge_track_proximity_costs
            # memoizes on the ids of the arrays it is handed.
            rows = _compose(fields, ex, cost)
        if window is None:
            self._composites[ckey] = rows
            while len(self._composites) > 8 or (
                    len(self._composites) > 1
                    and sum(r.nbytes for r in self._composites.values()
                            if r is not None) > _COMPOSITE_BYTES_MAX):
                self._composites.popitem(last=False)
        return rows


# Largest bounding box (cells x layers) _compose unions on a dense bitmap;
# beyond it, it sorts instead.
_BITMAP_MAX_CELLS = 64_000_000


def _compose(fields: List[np.ndarray], exempt_xy: np.ndarray,
             cost: int) -> Optional[np.ndarray]:
    """Union of packed field cells minus the exempt (gx, gy) cells, as
    unique [layer, gx, gy, cost] rows in sorted order.

    Every cell carries the same cost, so the union only has to be unique (sum
    mode in merge_track_proximity_costs would count a duplicate twice). A
    dense bitmap over the cells' bounding box does that in linear time; the
    sort it replaces cost ~0.1 s per routed net on a 2 mm band."""
    cat = np.concatenate(fields) if len(fields) > 1 else fields[0]
    lay = cat >> 48
    gx = (cat >> 24) & _COORD_MASK
    gy = cat & _COORD_MASK
    x0, y0 = int(gx.min()), int(gy.min())
    n_l = int(lay.max()) + 1
    w, h = int(gx.max()) - x0 + 1, int(gy.max()) - y0 + 1
    if n_l * w * h <= _BITMAP_MAX_CELLS:
        bm = np.zeros(n_l * w * h, dtype=bool)
        bm[(lay * w + (gx - x0)) * h + (gy - y0)] = True
        if len(exempt_xy):
            ex = (exempt_xy >> 24) - x0, (exempt_xy & _COORD_MASK) - y0
            inside = (ex[0] >= 0) & (ex[0] < w) & (ex[1] >= 0) & (ex[1] < h)
            flat = ex[0][inside] * h + ex[1][inside]
            for li in range(n_l):
                bm[li * w * h + flat] = False
        nz = np.flatnonzero(bm)
        if not len(nz):
            return None
        rem = nz % (w * h)
        lay, gx, gy = nz // (w * h), rem // h + x0, rem % h + y0
    else:
        keys = np.unique(cat)
        if len(exempt_xy):
            keys = keys[~np.isin(keys & _XY_MASK, exempt_xy)]
        if not len(keys):
            return None
        lay, gx, gy = keys >> 48, (keys >> 24) & _COORD_MASK, keys & _COORD_MASK
    rows = np.empty((len(lay), 4), dtype=np.int32)
    rows[:, 0] = lay
    rows[:, 1] = gx - _OFF
    rows[:, 2] = gy - _OFF
    rows[:, 3] = cost
    return rows


def _state(config, pcb_data, announce: bool = True) -> Optional[_KeepAwayState]:
    specs = tuple(getattr(config, 'keep_away', None) or ())
    if not specs or pcb_data is None:
        return None
    store = getattr(pcb_data, '_keep_away_state', None)
    if store is None:
        store = pcb_data._keep_away_state = {}
    key = (specs, tuple(config.layers), config.grid_step)
    st = store.get(key)
    if st is None:
        # The rules resolve alike on every grid of one board: say so once.
        announce = announce and not any(k[0] == specs for k in store)
        st = store[key] = _KeepAwayState(pcb_data, parse_keep_away_rules(specs))
        if announce:
            st.announce()
    return st


def keep_away_rows(config, pcb_data, net_ids, single_track=False,
                   window=None) -> Optional[np.ndarray]:
    """[layer, gx, gy, cost] rows pricing the keep-away band for the routed
    net (an int) or diff pair (a tuple of its P and N ids); `single_track`
    sizes a pair's band for one track (a hybrid pair's legs); `window`
    (x0, y0, x1, y1 mm) prices only the band that reaches it, uncached. None
    when no rule concerns it or the cost is 0."""
    if getattr(config, 'keep_away_cost', 0) <= 0:
        return None
    st = _state(config, pcb_data)
    if st is None:
        return None
    ids = (net_ids,) if isinstance(net_ids, int) else tuple(sorted(net_ids))
    return st.rows(pcb_data, config, ids, single_track, window)


def stamp_keep_away(obstacles, config, pcb_data, net_ids,
                    single_track=False, window=None) -> bool:
    """Price the keep-away band on a map no proximity builder prepared: the
    per-attempt clones net_rescue's gap rescue and terminal escalation route
    on. A rescue rung that routes inside a window passes it as `window`
    (x0, y0, x1, y1 mm). Returns True when it stamped anything; on a map
    that outlives the route the caller clears it again with
    clear_layer_proximity(). Nothing is stamped without a rule, so a run
    without one routes exactly as before."""
    rows = keep_away_rows(config, pcb_data, net_ids, single_track, window)
    if rows is None:
        return False
    obstacles.set_layer_proximity_batch(rows)
    return True


def add_keepaway_source(ghosts, config, pcb_data, net_ids, single_track=False):
    """Fold the keep-away rows into a merge_track_proximity_costs ghost dict
    (the add_plan_source / add_history_source pattern; the key is the
    ('keepaway',) tuple). Returns the input unchanged when nothing applies."""
    rows = keep_away_rows(config, pcb_data, net_ids, single_track)
    if rows is None:
        return ghosts
    merged = dict(ghosts) if ghosts else {}
    merged[('keepaway',)] = rows
    return merged


# --------------------------------------------------------------------------
# Report: how much copper ended up inside a band, measured on the geometry.

def _report_layers(pcb_data, config) -> List[str]:
    """The layer list the report indexes copper by: the routed layers first
    (so their indices are config.layers'), then every other copper layer of
    the board, which only an `:adjacent` rule compares with (#2070)."""
    layers = list(config.layers)
    return layers + [L for L in copper_stack(pcb_data, layers) if L not in layers]


def _pour_geometry(zone, islands):
    """A pour's copper for the report, as a prepared shapely geometry, or
    None: KiCad's refilled islands of its (net, layer) when `islands` has the
    run's refill (missing = KiCad poured nothing there), else its outline
    less its cut-outs (the fill lies inside it)."""
    import shapely
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    if islands is not None:
        polys = [Polygon(p).buffer(0)
                 for p in islands.get((zone.net_name, zone.layer)) or ()
                 if len(p) >= 3]
        geom = unary_union(polys) if polys else None
    else:
        geom = Polygon(zone.polygon,
                       [h for h in (zone.holes or ()) if len(h) >= 3]).buffer(0)
    if geom is None or geom.is_empty:
        return None
    shapely.prepare(geom)
    return geom


def _net_geometry(st: _KeepAwayState, pcb_data, config, net_id: int,
                  layers=None, islands=None) -> dict:
    """Float arrays of one net's copper, one row per (item, layer):
    segs [layer, x1, y1, x2, y2, half_width], vias [layer, x, y, radius],
    pads [layer, cx, cy, half_w, half_h, corner_r, cos, sin]; and its pours
    as [(layer, (minx, miny, maxx, maxy), geometry)] (#2070). `layers` is
    the index the rows use (`_report_layers`; config.layers by default)."""
    from connectivity import via_copper_layers
    layers = list(layers or config.layers)
    lidx = {L: i for i, L in enumerate(layers)}
    _sig, segs, vias, zones = st.copper[net_id]
    s_rows = [(lidx[s.layer], s.start_x, s.start_y, s.end_x, s.end_y,
               (s.width or 0.0) / 2) for s in segs if s.layer in lidx]
    v_rows = [(lidx[L], v.x, v.y, (v.size or 0.0) / 2)
              for v in vias for L in via_copper_layers(v, layers)
              if L in lidx]
    p_rows = []
    for pad in pcb_data.pads_by_net.get(net_id, []):
        rot = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
        for L in _pad_layers(pad, layers):
            p_rows.append((lidx[L], pad.global_x, pad.global_y, pad.size_x / 2,
                           pad.size_y / 2, _pad_corner_radius(pad),
                           math.cos(rot), math.sin(rot)))
    pours = []
    for z in zones:
        if z.layer in lidx:
            geom = _pour_geometry(z, islands)
            if geom is not None:
                pours.append((lidx[z.layer], geom.bounds, geom))
    return {'segs': np.array(s_rows, dtype=np.float64).reshape(-1, 6),
            'vias': np.array(v_rows, dtype=np.float64).reshape(-1, 4),
            'pads': np.array(p_rows, dtype=np.float64).reshape(-1, 8),
            'pours': pours}


def _pad_sdf(px, py, pads):
    """(k, m) signed distance from points to rounded-rect pads (rows as in
    _net_geometry), negative inside."""
    dx = px[:, None] - pads[None, :, 1]
    dy = py[:, None] - pads[None, :, 2]
    c, s = pads[None, :, 6], pads[None, :, 7]
    lx = np.abs(dx * c + dy * s)
    ly = np.abs(-dx * s + dy * c)
    cr = pads[None, :, 5]
    qx = lx - (pads[None, :, 3] - cr)
    qy = ly - (pads[None, :, 4] - cr)
    outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
    inside = np.minimum(np.maximum(qx, qy), 0.0)
    return outside + inside - cr


def _seg_dist(px, py, segs):
    """(k, m) distance from points to segment centrelines."""
    x1, y1 = segs[None, :, 1], segs[None, :, 2]
    dx, dy = segs[None, :, 3] - x1, segs[None, :, 4] - y1
    l2 = dx * dx + dy * dy
    t = np.where(l2 > 0, ((px[:, None] - x1) * dx + (py[:, None] - y1) * dy)
                 / np.where(l2 > 0, l2, 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(px[:, None] - (x1 + t * dx), py[:, None] - (y1 + t * dy))


def _opposite_rows(geo, opp: Dict[int, float], kind: str, width: int) -> np.ndarray:
    arrs = [np.column_stack([geo[o][kind], np.full(len(geo[o][kind]), opp[o]),
                             np.full(len(geo[o][kind]), o)])
            for o in sorted(opp) if o in geo and len(geo[o][kind])]
    return np.vstack(arrs) if arrs else np.empty((0, width + 2), dtype=np.float64)


def _opposite_copper(geo, opp: Dict[int, float]):
    """The other side's (segs, vias, pads, pours), each row with its GAP and
    owning net appended (a pour as (layer, bounds, geometry, GAP, net)), and
    how far from an item's reference point its band can reach: a track's
    bounding box grows by this before filtering items (a pour is filtered by
    its own bounds)."""
    o_segs, o_vias, o_pads = (_opposite_rows(geo, opp, kind, width)
                              for kind, width in (('segs', 6), ('vias', 4),
                                                  ('pads', 8)))
    o_pours = [(li, bb, g, opp[o], o) for o in sorted(opp) if o in geo
               for li, bb, g in geo[o]['pours']]
    reach = max(float(o_segs[:, 5].max()) if len(o_segs) else 0.0,
                float(o_vias[:, 3].max()) if len(o_vias) else 0.0,
                float(np.hypot(o_pads[:, 3], o_pads[:, 4]).max())
                if len(o_pads) else 0.0) + max(opp.values())
    return (o_segs, o_vias, o_pads, o_pours), reach


def _pour_spacing(px, py, geom) -> np.ndarray:
    """Signed distance (mm) from points to a pour's copper, negative inside."""
    import shapely
    pts = shapely.points(px, py)
    d = shapely.distance(geom, pts)
    ins = shapely.contains_xy(geom, px, py)
    if ins.any():
        d = np.where(ins, -shapely.distance(geom.boundary, pts), d)
    return d


def _track_samples(x1, y1, x2, y2, li, hw, own_pads, free, views):
    """Sample one track of a keep-away net against the other side's copper.

    `views` is [(opposite copper, its reach, the layer indices it is compared
    on)]: the routed layer for every rule, and the layers next to it for the
    `:adjacent` rules (#2070). Returns (keep, hit, spacing, owners), or None
    when every sample lies within `free` of the net's own pads: `keep` marks
    the samples the band prices, `hit` those of them inside a band, and per
    (sample, item) the edge-to-edge spacing and the item's owning net (-1
    for none)."""
    seg_len = math.hypot(x2 - x1, y2 - y1)
    k = max(1, int(math.ceil(seg_len / _REPORT_STEP)))
    t = (np.arange(k) + 0.5) / k
    px, py = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
    keep = np.ones(k, dtype=bool)
    if free > 0 and len(own_pads):
        keep &= ~(_pad_sdf(px, py, own_pads) <= free).any(axis=1)
    if not keep.any():
        return None
    # Edge-to-edge spacing per (sample, item), and each item's GAP.
    spacing = [np.full((k, 1), math.inf)]
    gaps = [np.zeros(1)]
    owners = [np.full(1, -1.0)]
    for (o_segs, o_vias, o_pads, o_pours), reach, on in views:
        lo_x, hi_x = min(x1, x2) - hw - reach, max(x1, x2) + hw + reach
        lo_y, hi_y = min(y1, y2) - hw - reach, max(y1, y2) + hw + reach
        on_arr = np.asarray(on, dtype=np.float64)
        sel = o_segs[np.isin(o_segs[:, 0], on_arr)
                     & (np.maximum(o_segs[:, 1], o_segs[:, 3]) >= lo_x)
                     & (np.minimum(o_segs[:, 1], o_segs[:, 3]) <= hi_x)
                     & (np.maximum(o_segs[:, 2], o_segs[:, 4]) >= lo_y)
                     & (np.minimum(o_segs[:, 2], o_segs[:, 4]) <= hi_y)]
        if len(sel):
            spacing.append(_seg_dist(px, py, sel) - sel[None, :, 5] - hw)
            gaps.append(sel[:, 6])
            owners.append(sel[:, 7])
        for arr, dist in ((o_vias, lambda a: np.hypot(
                               px[:, None] - a[None, :, 1],
                               py[:, None] - a[None, :, 2]) - a[None, :, 3]),
                          (o_pads, lambda a: _pad_sdf(px, py, a))):
            sel = arr[np.isin(arr[:, 0], on_arr)
                      & (arr[:, 1] >= lo_x) & (arr[:, 1] <= hi_x)
                      & (arr[:, 2] >= lo_y) & (arr[:, 2] <= hi_y)]
            if len(sel):
                spacing.append(dist(sel) - hw)
                gaps.append(sel[:, -2])
                owners.append(sel[:, -1])
        for pli, (bx0, by0, bx1, by1), geom, gap, owner in o_pours:
            r = hw + gap
            if (pli not in on or bx1 < min(x1, x2) - r or bx0 > max(x1, x2) + r
                    or by1 < min(y1, y2) - r or by0 > max(y1, y2) + r):
                continue
            spacing.append((_pour_spacing(px, py, geom) - hw)[:, None])
            gaps.append(np.array([gap]))
            owners.append(np.array([float(owner)]))
    sp = np.hstack(spacing)
    gap_of = np.concatenate(gaps)
    # In band where spacing < GAP for ANY item (each its own GAP).
    dmin = (sp - gap_of[None, :]).min(axis=1)
    hit = keep & (dmin < -REPORT_TOL)
    return keep, hit, sp, np.concatenate(owners)


def _report_views(st, geo, net_id: int, li: int, layers, stack, cache):
    """The `views` `_track_samples` compares a track of `net_id` on layer
    index `li` with: the other side's copper on that layer, and, for an
    `:adjacent` rule, the other side's copper on the layers next to it."""
    key = (net_id, li)
    if key in cache:
        return cache[key]
    views = []
    opp = st.opposite((net_id,))
    if opp:
        bundle = cache.get(('same', net_id))
        if bundle is None:
            bundle = cache[('same', net_id)] = _opposite_copper(geo, opp)
        views.append((bundle[0], bundle[1], (li,)))
    opp_adj = st.opposite((net_id,), adjacent=True)
    if opp_adj:
        lidx = {L: i for i, L in enumerate(layers)}
        on = tuple(lidx[N] for N in adjacent_layers(stack, layers[li])
                   if N in lidx)
        if on:
            bundle = cache.get(('adj', net_id))
            if bundle is None:
                bundle = cache[('adj', net_id)] = _opposite_copper(geo, opp_adj)
            views.append((bundle[0], bundle[1], on))
    cache[key] = views
    return views


def band_meter(pcb_data, config, net_id: int):
    """In-band length of a candidate track of `net_id`, measured as
    keep_away_report measures the routed copper (#2006): a function
    (layer, x1, y1, x2, y2, width) -> mm. A post-route pass that moves copper
    asks it so it does not hand back a band the router kept clear. None when
    the band is not priced (no rule sets the net against another, or
    keep_away_cost is 0). Reads the other side's copper as it is now; build a
    new meter once that copper moves."""
    if getattr(config, 'keep_away_cost', 0) <= 0:
        return None
    st = _state(config, pcb_data, announce=False)
    if st is None or net_id not in st.members:
        return None
    opp = st.opposite((net_id,))
    if not opp:
        return None
    st.refresh(pcb_data)
    free = max(0.0, float(getattr(config, 'keep_away_free', 0.0) or 0.0))
    layers = _report_layers(pcb_data, config)
    stack = copper_stack(pcb_data, config.layers)
    geo = {n: _net_geometry(st, pcb_data, config, n, layers)
           for n in (net_id, *opp)}
    own_pads = _own_rows(geo[net_id]['pads'], len(config.layers))
    lidx = {L: i for i, L in enumerate(config.layers)}
    cache: dict = {}

    def in_band(layer, x1, y1, x2, y2, width) -> float:
        li = lidx.get(layer)
        if li is None:
            return 0.0
        samples = _track_samples(
            x1, y1, x2, y2, li, width / 2, own_pads, free,
            _report_views(st, geo, net_id, li, layers, stack, cache))
        if samples is None:
            return 0.0
        keep, hit, _sp, _owners = samples
        return math.hypot(x2 - x1, y2 - y1) * float(hit.sum()) / len(keep)

    return in_band


class ViaBandGate:
    """The keep-away bands as a SITE gate for vias a pass places without a
    cost search (route_planes' stitch lattice and edge fence, #2066).

    The route steps price a band softly per grid cell; a stitch pass has no
    search to price it in, and a lattice is dense enough that dropping a site
    costs little, so here the band is a gate: ``for_net(net_id)`` answers
    ``in_band(x, y, radius)`` -- True when a via disk of ``radius`` at (x, y)
    comes closer than a rule's GAP (edge to edge) to copper of a net the rules
    set ``net_id`` against (tracks, vias, pads, pours) on any of ``layers`` (the
    via's span), outside ``free`` mm of ``net_id``'s own pads, the same free
    radius the route steps leave unpriced. Measured as keep_away_report
    measures routed copper. The other side's copper is read when
    ``for_net`` is called, so build the net's predicate after the vias an
    earlier net placed are on ``pcb_data``."""

    def __init__(self, pcb_data, rules, layers, free: float):
        from types import SimpleNamespace
        self._pcb = pcb_data
        self._st = _KeepAwayState(pcb_data, parse_keep_away_rules(rules))
        self._cfg = SimpleNamespace(layers=list(layers))
        self._free = max(0.0, float(free or 0.0))
        self._st.announce()

    def for_net(self, net_id: int):
        """``in_band(x, y, radius) -> bool`` for vias of ``net_id``, or None
        when no rule sets it against another net on this board."""
        st = self._st
        if net_id not in st.members:
            return None
        opp = st.opposite((net_id,))
        if not opp:
            return None
        st.refresh(self._pcb)
        geo = {n: _net_geometry(st, self._pcb, self._cfg, n)
               for n in (net_id, *opp)}
        (o_segs, o_vias, o_pads, o_pours), reach = _opposite_copper(geo, opp)
        own_pads = geo[net_id]['pads']
        free = self._free

        def in_band(x: float, y: float, radius: float) -> bool:
            px, py = np.array([x]), np.array([y])
            if free > 0 and len(own_pads) \
                    and (_pad_sdf(px, py, own_pads) <= free).any():
                return False
            r = reach + radius
            sel = o_segs[(np.maximum(o_segs[:, 1], o_segs[:, 3]) >= x - r)
                         & (np.minimum(o_segs[:, 1], o_segs[:, 3]) <= x + r)
                         & (np.maximum(o_segs[:, 2], o_segs[:, 4]) >= y - r)
                         & (np.minimum(o_segs[:, 2], o_segs[:, 4]) <= y + r)]
            if len(sel) and ((_seg_dist(px, py, sel)[0] - sel[:, 5] - radius)
                             < sel[:, 6] - 1e-9).any():
                return True
            for arr, dist in ((o_vias, lambda a: np.hypot(
                                   x - a[:, 1], y - a[:, 2]) - a[:, 3]),
                              (o_pads, lambda a: _pad_sdf(px, py, a)[0])):
                sel = arr[(np.abs(arr[:, 1] - x) <= r)
                          & (np.abs(arr[:, 2] - y) <= r)]
                if len(sel) and ((dist(sel) - radius)
                                 < sel[:, -2] - 1e-9).any():
                    return True
            for _li, (bx0, by0, bx1, by1), geom, gap, _o in o_pours:
                g = gap + radius
                if bx0 - g <= x <= bx1 + g and by0 - g <= y <= by1 + g \
                        and (_pour_spacing(px, py, geom)[0] - radius) < gap - 1e-9:
                    return True
            return False

        return in_band


#: What a rule without `:adjacent` does not see (#2070), said with every
#: report so that "0 mm inside a band" is not read as "clear on every layer".
NOT_MEASURED_OTHER_LAYERS = ("copper on other layers (a rule without "
                             "':adjacent' compares the routed layer only)")
#: ... and what an `:adjacent` rule does not see.
NOT_MEASURED_FAR_LAYERS = ("copper two or more layers away (':adjacent' "
                           "compares the layers next to the routed one)")


def _own_rows(arr: np.ndarray, n_route: int) -> np.ndarray:
    """The rows of a net's own copper on the routed layers (the report's
    layer index puts those first)."""
    return arr[arr[:, 0] < n_route] if len(arr) else arr


def keep_away_report(pcb_data, config, announce: bool = True,
                     pour_islands=None) -> Optional[dict]:
    """Measure every keep-away net's tracks against the other side's copper.

    Per net: the track length whose edge comes closer than GAP to copper of a
    net the rules set it against (same layer; and the layers next to it in
    plan view for an `:adjacent` rule), excluding the stretch within the free
    radius of its own pads, where the band is not priced either. The other
    side's POURS count (#2070): measured on `pour_islands` -- a KiCad refill
    of exactly this copper the run already made,
    `{(net name, layer): [polygon, ...]}` -- when given, else on the zone
    outlines; `pour_measured_on` says which. `not_measured` lists what the
    rules do not look at. Board-scoped: it reads all copper on the board, not
    only this run's. None when no rule is set. `announce` False keeps the
    rules' resolution off the log (a re-read of a board the run already
    announced them on)."""
    st = _state(config, pcb_data, announce)
    if st is None:
        return None
    st.refresh(pcb_data)
    free = max(0.0, float(getattr(config, 'keep_away_free', 0.0) or 0.0))
    layers = _report_layers(pcb_data, config)
    stack = copper_stack(pcb_data, config.layers)
    n_route = len(config.layers)
    geo = {n: _net_geometry(st, pcb_data, config, n, layers, pour_islands)
           for n in st.members}
    names = {n: (pcb_data.nets[n].name if n in pcb_data.nets else str(n))
             for n in st.members}
    per_net = {}
    checked = 0
    total_in = 0.0
    cache: dict = {}
    for n in sorted(st.members, key=lambda k: names[k]):
        own_segs = _own_rows(geo[n]['segs'], n_route)
        if not len(own_segs):
            continue
        opp = st.opposite((n,))
        if not opp:
            continue
        gap_of = dict(st.opposite((n,), adjacent=True))
        gap_of.update({o: max(g, gap_of.get(o, 0.0)) for o, g in opp.items()})
        own_pads = _own_rows(geo[n]['pads'], n_route)
        checked += 1
        length = in_band = 0.0
        best = (math.inf, None, 0.0)      # (closest spacing, net, its GAP)
        for li, x1, y1, x2, y2, hw in own_segs:
            seg_len = math.hypot(x2 - x1, y2 - y1)
            length += seg_len
            samples = _track_samples(
                x1, y1, x2, y2, int(li), hw, own_pads, free,
                _report_views(st, geo, n, int(li), layers, stack, cache))
            if samples is None:
                continue
            keep, hit, sp, own_of = samples
            if hit.any():
                in_band += seg_len * float(hit.sum()) / len(keep)
            # The closest copper of the other side, over the samples the band
            # prices -- the spacing a reader compares with the GAP.
            near = sp.argmin(axis=1)
            spmin = sp[np.arange(len(keep)), near]
            kept = np.flatnonzero(keep)
            j = int(kept[spmin[kept].argmin()])
            if spmin[j] < best[0]:
                o = int(own_of[near[j]])
                best = (float(spmin[j]), o, gap_of.get(o, 0.0))
        if in_band > 0:
            total_in += in_band
            per_net[names[n]] = {
                'in_band_mm': round(in_band, 2),
                'length_mm': round(length, 2),
                'min_spacing_mm': round(best[0], 3),
                'gap_mm': best[2],
                'closest_net': names.get(best[1], str(best[1])),
            }
    rep = {
        'rules': [r.spec for r in st.rules],
        'free_radius_mm': free,
        'cost': float(getattr(config, 'keep_away_cost', 0.0)),
        'nets_checked': checked,
        'nets_in_band': len(per_net),
        'in_band_mm': round(total_in, 2),
        'nets': dict(sorted(per_net.items(),
                            key=lambda kv: -kv[1]['in_band_mm'])),
    }
    if any(st.copper[n][3] for n in st.members):
        rep['pour_measured_on'] = ('kicad_refill' if pour_islands is not None
                                   else 'outline')
    not_measured = []
    if not all(st.adjacent):
        not_measured.append(NOT_MEASURED_OTHER_LAYERS)
    if any(st.adjacent) and len(stack) > 2:
        not_measured.append(NOT_MEASURED_FAR_LAYERS)
    rep['not_measured'] = not_measured
    return rep


def disclose_keep_away(board, config, title: str = 'Keep-away',
                       quiet: bool = False, pour_islands=None) -> Optional[dict]:
    """keep_away_report on `board`, printed unless `quiet` (which also keeps
    the rules' resolution off the log). `pour_islands`: a KiCad refill of
    exactly this copper the run already made, else None (the pours are then
    measured on their outlines; nothing is refilled for this). None without a
    rule; {'error': ...} when the measurement raised, so a summary says the
    reading is missing rather than dropping the key."""
    if not getattr(config, 'keep_away', None):
        return None
    try:
        rep = keep_away_report(board, config, announce=not quiet,
                               pour_islands=pour_islands)
    except Exception as e:                                     # noqa: BLE001
        return {'error': str(e)}
    if rep is not None and not quiet:
        print_keep_away_report(rep, title=title)
    return rep


def keep_away_entries(report: Optional[dict]) -> List[dict]:
    """The report's in-band nets as a list, {'net': name, ...} each -- the
    shape results_data carries (every field there is a list)."""
    return [{'net': n, **e} for n, e in ((report or {}).get('nets') or {}).items()]


def print_keep_away_report(report: Optional[dict], limit: int = 20,
                           title: str = 'Keep-away') -> None:
    if not report or 'rules' not in report:
        return
    n_in = report['nets_in_band']
    print(f"\n{title} ({len(report['rules'])} rule(s), free radius "
          f"{report['free_radius_mm']:g} mm): {n_in} of "
          f"{report['nets_checked']} net(s) run {report['in_band_mm']:g} mm "
          f"inside a band")
    for name, e in list(report['nets'].items())[:limit]:
        print(f"  {name}: {e['in_band_mm']:g} of {e['length_mm']:g} mm, "
              f"closest {e['min_spacing_mm']:g} mm (gap {e['gap_mm']:g}) "
              f"to {e['closest_net']}")
    if n_in > limit:
        print(f"  ... and {n_in - limit} more (JSON_SUMMARY keep_away)")
    on = report.get('pour_measured_on')
    if on:
        print("  pours measured on: " + (
            "KiCad's refill of this copper" if on == 'kicad_refill' else
            "their zone outlines (no KiCad refill of this copper in the run; "
            "the fill lies inside them)"))
    for what in report.get('not_measured') or ():
        print(f"  not measured: {what}")
