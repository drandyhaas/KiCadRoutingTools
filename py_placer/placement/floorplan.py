"""Floorplan intent: declare where things are supposed to go, then grade it (#549).

Every placement judgement in this toolchain collapses to "did `crossings` go
down". That number is indifferent between a sensible layout and a scattered one
with the same wirelength -- and a render only moves the judgement from a number
to a vibe. Nothing is ever *declared* about where parts belong, so nothing can
check whether they went there.

This module is the declaration and the check. An intent file says what the
floorplan is meant to be; `grade()` measures the board against it and returns
violations carrying the measured number next to the limit it broke. That makes
intent falsifiable: "the render looks fine" stops being available as a verdict.

THE BOARD OUTLINE IS NOT OURS TO CHANGE. `envelope` is READ from the board, not
authored -- `emit_intent` fills it from `board_bounds` and nothing here ever
writes Edge.Cuts. A part outside the envelope is a finding about the PART. Board
size, cutouts and slots are mechanical decisions (enclosure fit, panel rails,
connector apertures) that belong to the user; the honest response to a board
that is genuinely too small is to say so with the measured number and stop.

Design notes worth knowing before extending this:

* `refs` is the primitive for block membership, not `group`. Sheet group keys
  are opaque uuid paths (KiCad's Sheetname is absent from every corpus board),
  so nothing can author `"group": "sheet:1a2b3c4d"` without first listing the
  board. `group` is accepted, and matched against the raw key AND its
  `short_name` form, but globs over references are what a human or a model can
  actually write.

* A block that resolves to ZERO refs is an error, never a silent pass. A typo'd
  block grading clean is the exact failure this file exists to prevent.

* `rules_run` / `rules_skipped` are reported alongside the violation count. "0
  violations" and "0 rules ran" must not look the same to a machine.

* Every rule reuses the geometry the optimizer itself gates on -- `legality`'s
  `BoardOutlineGate` and `GradedPart`, `groups.decap_tethers`, `QuenchState`'s
  own `legality_metrics`. A grader with its own idea of "legal" grades the
  reimplementation, not the board.
"""
from __future__ import annotations

import fnmatch
import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

from . import legality
from . import groups as groups_mod

SCHEMA_VERSION = 1
KIND = 'floorplan-intent'

# Severity drives the exit code; 'warn' is reported and does not fail the run.
ERROR = 'error'
WARN = 'warn'

DEFAULT_ZONE_TOLERANCE_MM = 0.5
DEFAULT_ENVELOPE_TOLERANCE_MM = 0.5

#: Floor for how large an OBSERVED overhang may be and still be emitted as a
#: declared edge band. An observation entry converts what the board DOES into
#: what the spec ALLOWS, and above this there is no plausible reading under
#: which it is a spec: a part hanging 160 mm off an 81 mm board is not an edge
#: connector with a 160 mm band, it is a part that was dragged off the board.
#:
#: The real cap is `max(this, the part's own largest dimension)` -- an overhang
#: cannot exceed the part's own size without the part being entirely off the
#: outline -- and this floor keeps a small part's legitimate band (a 0603 test
#: point half off the edge, a switch actuator) from being called damage.
#:
#: Measured, run 10: `check_floorplan --emit-intent --declare-classes` on a
#: damaged board declared all ELEVEN off-board parts edge connectors with bands
#: equal to their damage displacement (R7 east 34.792 ... TP4 north 160.062).
#: Every downstream consumer then went blind: seeder's off-board census skips
#: declared edge refs, so `--repair` found 5 violators and none of the 11; and
#: reconstruct's gate read `oob = 4.348` on a board with parts 158 mm out
#: (929.671 without the bands). This generalises the mount_hole refusal below,
#: which already knows this failure mode for one part class.
EDGE_BAND_SANITY_MM = 5.0

#: What this build can ACT on, as distinct from what it can parse (#710).
#: `schema` is the format number and is matched exactly; bumping it invalidates
#: every existing intent file at once, which is far too blunt for "this build
#: learned a new field". `READER_VERSION` is the field-vocabulary number, and an
#: intent whose claim would be MEANINGLESS to an older build says so by setting
#: `min_reader` to the version that introduced the field.
#:
#: Bump this whenever the loader learns a declarable field that changes a
#: verdict, and say in docs/floorplan-intent.md which field arrived. Do NOT
#: bump it for a field nothing grades.
READER_VERSION = 1

_TOP_LEVEL_KEYS = {
    'schema', 'kind', 'board', 'units', 'envelope', 'defaults', 'blocks',
    'keepouts', 'edge_connectors', 'decaps', 'must_lock', 'legality_budget',
    'health', 'severity', 'context', 'overlap_waivers', 'min_reader',
}
_BLOCK_KEYS = {'name', 'group', 'refs', 'zone', 'side', 'exclusive',
               'tolerance_mm', 'note', 'context'}

# The principle `_BLOCK_KEYS` already encodes, applied one level down (#710).
# Before this the loader was strict at exactly two levels -- top-level and
# `blocks[]` -- and permissive everywhere else, so `{"ref": "J1",
# "max_setback": 2.0}` and `{"severity": {"decap_distanc": "warn"}}` loaded
# clean and did nothing. That is the failure `block_unresolved` exists to
# prevent, one level down: a constraint the author thinks they set and the
# grader never checks.
#
# The compatibility half matters as much. Because unknown nested keys were
# IGNORED rather than refused, every field added below the top level was
# automatically backward compatible AND silently inert on an older build --
# for a constraint the worst possible pair, because the older reader answers
# "clean" instead of "I do not understand this". Refusing makes it say so;
# `min_reader` is the explicit, author-set form of the same guarantee.
#
# Every name below is either read by code or written by `emit_intent`. The
# sets were enumerated from both directions, and the emitter's own output is
# pinned against them by tests/test_549_floorplan_grade.py -- so a new emitted
# key fails there, rather than as an artifact that stops loading months later.
_ENVELOPE_KEYS = {'rect', 'tolerance_mm'}
_KEEPOUT_KEYS = {'name', 'rect', 'circle', 'sides', 'allow', 'note',
                 'context'}
#: `source`, `suspect`, `suspect_reason`, `overhang_capped` and
#: `observed_overhang_mm` are emitter-written and read by nothing today; they
#: are accepted because `emit_intent` writes them and the round trip must
#: survive, not because anything acts on them.
_EDGE_CONNECTOR_KEYS = {'ref', 'edge', 'overhang_mm', 'max_setback_mm',
                        'class', 'source', 'note', 'suspect', 'suspect_reason',
                        'overhang_capped', 'observed_overhang_mm', 'context'}
_OVERHANG_KEYS = {'min', 'max'}
_DECAP_KEYS = {'max_distance_mm', 'exempt', 'search_radius_mm'}
_DEFAULTS_KEYS = {'zone_tolerance_mm'}
#: `zoned_blocks` is setdefault-injected into this same dict by `grade` after
#: load, and `affinity_exempt_net_ids` is derived there from
#: `affinity_exempt_nets` -- but only when that key is present, so an author
#: may also set the ids directly. Both are accepted for that reason.
_HEALTH_KEYS = {'bus_corridors', 'classes', 'zoned_blocks',
                'affinity_exempt_nets', 'affinity_exempt_net_ids',
                'ignore_net_ids', 'max_fanout', 'block_displacement_mm'}
_CORRIDOR_KEYS = {'name', 'nets', 'width_mm'}
_BUDGET_KEYS = {'overlap_area', 'oob_count', 'oob_amount'}
_WAIVER_KEYS = {'pair', 'reason', 'context'}
# `context` deliberately has NO key set of its own, at the top level or on an
# entry: it is the read-only slot where a run records provenance no rule will
# ever grade. The four objects a human AUTHORS entry-by-entry accept one --
# _BLOCK_KEYS, _KEEPOUT_KEYS, _EDGE_CONNECTOR_KEYS, _WAIVER_KEYS -- because the
# alternative is worse; the settings objects (envelope, defaults, decaps,
# health, legality_budget, overhang_mm) do not, since prose about a setting
# belongs on the claim that uses it or at the top level.
# Refusing prose outright pushes it into a key that IS graded -- the recorded
# runs show exactly that drift, an `edge_connectors[]` entry that grew
# `band_basis`, `why`, `why_not_repaired` and `rejected_alternative` because
# there was nowhere else for the reasoning to go. Folding it into `note`
# instead would be worse still: `note` is load-bearing, grepped for the
# substring SUSPECT by emit_intent and place_reconstruct.
_EDGES = ('north', 'south', 'east', 'west')


class IntentError(ValueError):
    """A malformed intent file. Distinct from a violation: this is the intent
    being unreadable, not the board being wrong."""


# --------------------------------------------------------------------------
# records
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Violation:
    rule: str
    severity: str
    message: str
    ref: Optional[str] = None
    block: Optional[str] = None
    measured: Dict[str, object] = field(default_factory=dict)
    expected: Dict[str, object] = field(default_factory=dict)

    def sort_key(self):
        """Order is a property of the finding, never of dict iteration (#457)."""
        return (self.rule, self.ref or '', self.block or '', self.message)

    def to_dict(self):
        d = {'rule': self.rule, 'severity': self.severity,
             'message': self.message}
        if self.ref:
            d['ref'] = self.ref
        if self.block:
            d['block'] = self.block
        if self.measured:
            d['measured'] = self.measured
        if self.expected:
            d['expected'] = self.expected
        return d


@dataclass(frozen=True)
class Zone:
    name: str
    rect: Optional[Tuple[float, float, float, float]] = None
    side: Optional[str] = None
    group: Optional[str] = None
    refs: Tuple[str, ...] = ()
    exclusive: bool = False
    tolerance_mm: Optional[float] = None
    note: str = ''
    #: Free-form provenance, read by nothing (see `_BLOCK_KEYS`). Carried on
    #: the Zone rather than dropped: `keepouts`/`edge_connectors`/
    #: `overlap_waivers` keep their raw dict and so keep theirs, and a slot
    #: that silently vanishes for ONE of the four is the #710 defect itself.
    #: (Zone is frozen, so this makes it unhashable -- nothing hashes a Zone,
    #: only iterates them.)
    context: Dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class Intent:
    schema: int
    kind: str
    board: str
    units: str
    envelope: Dict[str, object]
    defaults: Dict[str, object]
    blocks: Tuple[Zone, ...]
    keepouts: Tuple[Dict[str, object], ...]
    edge_connectors: Tuple[Dict[str, object], ...]
    decaps: Dict[str, object]
    must_lock: Tuple[str, ...]
    legality_budget: Dict[str, object]
    health: Dict[str, object]
    severity: Dict[str, str]
    source_path: str = ''
    # Run-6: authored courtyard-overlap waivers, [{'pair': [a, b], 'reason'}].
    # NEVER auto-emitted (a waiver derived from the board under repair would
    # be the budget self-bless bug again); consumed by grade_body_overlap.
    overlap_waivers: Tuple[Dict[str, object], ...] = ()
    # Run-23: budget keys `emit_intent` deliberately did NOT bake, and why
    # ({key: reason}). Emitted into `context.budget_withheld`; carried here so
    # the GRADE can say "not derivable" instead of grading nothing and
    # printing 0 errors. Hand-written intents leave it empty, which is the
    # honest answer for a budget a human simply chose not to declare.
    budget_withheld: Dict[str, str] = field(default_factory=dict)

    def edge_claims(self) -> Tuple[Dict[str, object], ...]:
        """The `edge_connectors` entries that actually CLAIM AN EDGE.

        The wire key holds two populations. An `edge_receptacle` /
        `edge_actuator` entry says "this part's mating face belongs at the
        boundary", and the placement engines act on that: place_seed LOCKS it
        during the polish quench, place_reconstruct grants it a banded
        out-of-outline allowance and excludes it from the exchange stage, and
        reconstruct.classify forces it into the anchor tier. A
        `connector_affinity` entry (run-23) says only "this is a
        connector-family part with NO edge claim" -- it exists so a mid-board
        header stops being invisible to `rule_edge_connector`, which flags an
        interior pose at WARN.

        Handing the second population to the first's consumers would silently
        change placement: on tigard_placed that is 6 extra refs locked in the
        seed quench, given a 2.0mm off-outline allowance each and pinned as
        anchors -- for parts nobody said anything about. So the engines read
        THIS, and the rule reads `edge_connectors`.

        The split lives here, in one place, rather than as a filter repeated
        at every consumer: a filter that must be remembered is a filter that
        will be forgotten at the next call site.
        """
        return tuple(c for c in self.edge_connectors
                     if c.get('class') != 'connector_affinity')

    def waiver_pairs(self) -> Tuple[Tuple[str, str], ...]:
        out = []
        for w in self.overlap_waivers:
            pair = w.get('pair') or ()
            if len(pair) == 2:
                out.append((str(pair[0]), str(pair[1])))
        return tuple(out)

    def severity_of(self, rule: str, default: str = ERROR) -> str:
        return self.severity.get(rule, default)

    def zone_tolerance(self, zone: Zone) -> float:
        if zone.tolerance_mm is not None:
            return float(zone.tolerance_mm)
        return float(self.defaults.get('zone_tolerance_mm',
                                       DEFAULT_ZONE_TOLERANCE_MM))


def empty_intent(board: str = '') -> Intent:
    """An intent that declares nothing -- every construct empty.

    For engine paths that are intent-DRIVEN but must still run when the caller
    has none (`place_reconstruct` without `--intent`, `place_seed --reseat` on a
    board with no floorplan file). Declaring nothing is not the same as having
    no intent object: the seeding stages read zones, edge bands and must_lock
    off it, and each of those reads is a no-op here rather than a branch at
    every site."""
    return Intent(schema=SCHEMA_VERSION, kind=KIND, board=board, units='mm',
                  envelope={}, defaults={}, blocks=(), keepouts=(),
                  edge_connectors=(), decaps={}, must_lock=(),
                  legality_budget={}, health={}, severity={})


# --------------------------------------------------------------------------
# loading and board-independent validation
# --------------------------------------------------------------------------

def _rect(value, where: str) -> Tuple[float, float, float, float]:
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       for v in value)):
        raise IntentError(f"{where}: expected a rect [x0, y0, x1, y1] of four "
                          f"numbers, got {value!r}")
    x0, y0, x1, y1 = (float(v) for v in value)
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def _reject_unknown(obj, allowed, where: str) -> None:
    """Refuse an unknown key rather than dropping it (#710).

    One message shape for every level, so `blocks[3]`, `severity` and
    `health.bus_corridors[0]` all read alike. The `Known:` list is what turns
    a typo into a fix -- `keepout` only looks wrong next to `keepouts`.
    """
    # str() BEFORE sorting, not at join time: a dict handed to
    # `intent_from_dict` directly (it is public, and the place_* mains catch
    # only ValueError) can carry a non-string key, and both `sorted` over
    # mixed types and `join` over non-strings raise TypeError -- which would
    # traceback past the callers instead of becoming their exit 2.
    bad = sorted(str(k) for k in set(obj) - set(allowed))
    if bad:
        raise IntentError(f"{where}: unknown key(s) {', '.join(bad)}. "
                          f"Known: {', '.join(sorted(map(str, allowed)))}")


def _entry_context(entry, where: str) -> None:
    """An entry's `context` is free-form, but it is still an OBJECT.

    Type-checked and otherwise untouched: a list here means the author meant
    something else, while an unknown key inside means nothing at all.
    """
    if 'context' in entry:
        _obj(entry['context'], f"{where}.context")


def _obj(value, where: str) -> Dict:
    """An intent object, or `{}` when absent.

    `raw.get(k) or {}` handed a list or a string straight on, so a schema
    error surfaced as an AttributeError inside a rule three call frames later
    -- or, for a key nothing reads yet, not at all.
    """
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise IntentError(f"{where}: expected an object, got {value!r}")
    return value


def _str_tuple(value, where: str) -> Tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        raise IntentError(f"{where}: expected a list of strings, got a bare "
                          f"string {value!r} (wrap it in a list)")
    if not isinstance(value, (list, tuple)) or not all(
            isinstance(v, str) for v in value):
        raise IntentError(f"{where}: expected a list of strings, got {value!r}")
    return tuple(value)


def load_intent(path: str) -> Intent:
    """Read and structurally validate an intent file.

    Raises IntentError on anything unreadable. Board-relative checks (does this
    block resolve, is this zone inside the envelope) belong to `grade`.
    """
    try:
        with open(path, encoding='utf-8') as fh:
            raw = json.load(fh)
    except (OSError, ValueError) as exc:
        raise IntentError(f"{path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise IntentError(f"{path}: expected a JSON object at the top level")
    return intent_from_dict(raw, source_path=path)


def intent_from_dict(raw: Dict, source_path: str = '') -> Intent:
    # FIRST, ahead of the unknown-key and schema checks. A build that declares
    # a new field almost always declares a new TOP-LEVEL one, so checking the
    # key set first means the actionable "this build is too old, upgrade"
    # message is exactly the one an author never sees -- they get "unknown
    # key(s) zones" and go looking for a typo that is not there.
    #
    # Grading such a file halfway is the same wrong answer as grading it
    # fully, so nothing else is read until this passes.
    min_reader = raw.get('min_reader')
    if min_reader is not None:
        if not isinstance(min_reader, int) or isinstance(min_reader, bool):
            raise IntentError(
                f"min_reader {min_reader!r}: expected an integer")
        if min_reader > READER_VERSION:
            raise IntentError(
                f"min_reader {min_reader}: this build is reader "
                f"{READER_VERSION}. The intent declares a claim this build "
                f"would not act on, and grading it would report clean on a "
                f"constraint that was never checked -- upgrade instead")

    _reject_unknown(raw, _TOP_LEVEL_KEYS, 'top level')

    schema = raw.get('schema')
    if schema != SCHEMA_VERSION:
        raise IntentError(
            f"schema {schema!r}: this build reads schema {SCHEMA_VERSION}")

    kind = raw.get('kind')
    if kind != KIND:
        # A round sidecar or a lock-advisor dump handed in by mistake reads as
        # JSON and would otherwise grade as an empty, clean intent.
        raise IntentError(
            f"kind {kind!r}: expected {KIND!r}. This does not look like a "
            f"floorplan intent file")
    units = raw.get('units', 'mm')
    if units != 'mm':
        raise IntentError(f"units {units!r}: only 'mm' is supported")

    envelope = _obj(raw.get('envelope'), 'envelope')
    _reject_unknown(envelope, _ENVELOPE_KEYS, 'envelope')
    if 'rect' in envelope and envelope['rect'] is not None:
        envelope = dict(envelope)
        envelope['rect'] = _rect(envelope['rect'], 'envelope.rect')

    blocks: List[Zone] = []
    seen_names = set()
    for i, b in enumerate(raw.get('blocks') or []):
        if not isinstance(b, dict):
            raise IntentError(f"blocks[{i}]: expected an object")
        _reject_unknown(b, _BLOCK_KEYS, f"blocks[{i}]")
        _entry_context(b, f"blocks[{i}]")
        name = b.get('name') or f"block{i}"
        if name in seen_names:
            raise IntentError(f"blocks[{i}]: duplicate block name {name!r}")
        seen_names.add(name)
        side = b.get('side')
        if side is not None and side not in ('F', 'B'):
            raise IntentError(
                f"blocks[{i}] ({name}): side {side!r}, expected 'F' or 'B'")
        zone_rect = b.get('zone')
        blocks.append(Zone(
            name=name,
            rect=_rect(zone_rect, f"blocks[{i}].zone") if zone_rect else None,
            side=side,
            group=b.get('group'),
            refs=_str_tuple(b.get('refs'), f"blocks[{i}].refs"),
            exclusive=bool(b.get('exclusive', False)),
            tolerance_mm=b.get('tolerance_mm'),
            note=b.get('note', '') or '',
            context=b.get('context') or {},
        ))
        if not blocks[-1].refs and not blocks[-1].group:
            raise IntentError(
                f"blocks[{i}] ({name}): needs `refs` (reference globs) or "
                f"`group` (a --group-by block name). Prefer `refs`: sheet group "
                f"keys are uuid paths you cannot author without listing them")

    keepouts = []
    for i, k in enumerate(raw.get('keepouts') or []):
        if not isinstance(k, dict):
            raise IntentError(f"keepouts[{i}]: expected an object")
        _reject_unknown(k, _KEEPOUT_KEYS, f"keepouts[{i}]")
        _entry_context(k, f"keepouts[{i}]")
        k = dict(k)
        k.setdefault('name', f"keepout{i}")
        if 'rect' in k and k['rect'] is not None:
            k['rect'] = _rect(k['rect'], f"keepouts[{i}].rect")
        elif 'circle' in k and k['circle'] is not None:
            c = k['circle']
            if (not isinstance(c, (list, tuple)) or len(c) != 3
                    or not all(isinstance(v, (int, float)) for v in c)):
                raise IntentError(
                    f"keepouts[{i}].circle: expected [x, y, radius]")
            k['circle'] = tuple(float(v) for v in c)
        else:
            raise IntentError(f"keepouts[{i}] ({k['name']}): needs `rect` or "
                              f"`circle`")
        k['sides'] = tuple(k.get('sides') or ('F', 'B'))
        k['allow'] = _str_tuple(k.get('allow'), f"keepouts[{i}].allow")
        keepouts.append(k)

    conns = []
    for i, c in enumerate(raw.get('edge_connectors') or []):
        if not isinstance(c, dict):
            raise IntentError(f"edge_connectors[{i}]: expected an object with "
                              f"a `ref`")
        # Unknown keys BEFORE the `ref` check: a typo'd `reff` should be told
        # it is unknown, not reported as a missing `ref` while the author
        # stares at the key they did write.
        _reject_unknown(c, _EDGE_CONNECTOR_KEYS, f"edge_connectors[{i}]")
        _entry_context(c, f"edge_connectors[{i}]")
        if not c.get('ref'):
            raise IntentError(f"edge_connectors[{i}]: expected an object with "
                              f"a `ref`")
        c = dict(c)
        edge = c.get('edge')
        if edge is not None and edge not in _EDGES:
            raise IntentError(
                f"edge_connectors[{i}] ({c['ref']}): edge {edge!r}, expected "
                f"one of {', '.join(_EDGES)}")
        oh = c.get('overhang_mm')
        if oh is not None:
            if not isinstance(oh, dict):
                raise IntentError(f"edge_connectors[{i}] ({c['ref']}): "
                                  f"overhang_mm expects "
                                  f"{{'min': .., 'max': ..}}")
            _reject_unknown(oh, _OVERHANG_KEYS,
                            f"edge_connectors[{i}] ({c['ref']}).overhang_mm")
        conns.append(c)

    severity = _obj(raw.get('severity'), 'severity')
    if any(v not in (ERROR, WARN) for v in severity.values()):
        raise IntentError(
            f"severity: expected {{rule: 'error'|'warn'}}, got {severity!r}")
    # Keys too, not only values (#710). `{"decap_distanc": "warn"}` used to
    # load clean and leave the rule at its default -- a demotion the author
    # believes they made and the exit code never reflects.
    _reject_unknown(severity, _SEVERITY_KEYS, 'severity')

    budget = _obj(raw.get('legality_budget'), 'legality_budget')
    if 'oob_area' in budget:
        # Refused loudly rather than ignored, because it is the ONE legality
        # number that lies about cutouts. `out_of_board_area` measures against
        # the rectangular usable inset only -- its own docstring calls itself "a
        # lower bound on a notched one" -- so a part sitting ENTIRELY inside a
        # milled slot scores oob_count=1, oob_amount>0 and oob_area=0.0. A
        # budget on it would grade a part in a hole as clean.
        raise IntentError(
            "legality_budget.oob_area: not gateable. out_of_board_area is "
            "measured against the bounding-box inset, so a part sitting inside "
            "a CUTOUT scores 0.0 area and would grade clean. Use oob_count or "
            "oob_amount, which both see the real Edge.Cuts rings")
    _reject_unknown(budget, _BUDGET_KEYS, 'legality_budget')

    defaults = _obj(raw.get('defaults'), 'defaults')
    _reject_unknown(defaults, _DEFAULTS_KEYS, 'defaults')

    decaps = _obj(raw.get('decaps'), 'decaps')
    _reject_unknown(decaps, _DECAP_KEYS, 'decaps')

    health = _obj(raw.get('health'), 'health')
    _reject_unknown(health, _HEALTH_KEYS, 'health')
    for i, spec in enumerate(health.get('bus_corridors') or []):
        if not isinstance(spec, dict):
            raise IntentError(f"health.bus_corridors[{i}]: expected an object")
        _reject_unknown(spec, _CORRIDOR_KEYS, f"health.bus_corridors[{i}]")

    # `context` is deliberately OPEN -- the documented read-only slot, where a
    # run records provenance no rule will ever grade (`emit_intent` writes
    # `cutouts`, `file_locked`, `budget_withheld`; run artifacts add their own
    # prose). Type-checked so a list cannot reach `.get('budget_withheld')`,
    # but its KEYS are the author's business: refusing them would only push
    # provenance into a key that IS graded, which is the worse failure.
    context = _obj(raw.get('context'), 'context')

    waivers = raw.get('overlap_waivers') or []
    if not isinstance(waivers, list):
        raise IntentError("overlap_waivers: expected a list of "
                          "{'pair': [refA, refB], 'reason': ...} objects")
    for i, w in enumerate(waivers):
        if (not isinstance(w, dict) or not isinstance(w.get('pair'), list)
                or len(w['pair']) != 2):
            raise IntentError(
                "overlap_waivers: each entry needs 'pair': [refA, refB] "
                "(and should carry a 'reason')")
        _reject_unknown(w, _WAIVER_KEYS, f"overlap_waivers[{i}]")
        _entry_context(w, f"overlap_waivers[{i}]")

    return Intent(
        schema=schema, kind=kind, board=raw.get('board', '') or '',
        units=units, envelope=envelope,
        defaults=defaults,
        blocks=tuple(blocks), keepouts=tuple(keepouts),
        edge_connectors=tuple(conns),
        decaps=decaps,
        must_lock=_str_tuple(raw.get('must_lock'), 'must_lock'),
        legality_budget=budget,
        health=health,
        severity={str(k): str(v) for k, v in severity.items()},
        source_path=source_path,
        overlap_waivers=tuple(waivers),
        budget_withheld={
            str(k): str(v) for k, v in
            _obj(context.get('budget_withheld'),
                 'context.budget_withheld').items()},
    )


def validate_intent(intent: Intent) -> List[Violation]:
    """Checks that need no board: does the intent contradict itself.

    Kept separate from `grade` so an intent can be reviewed before a board
    exists, and so a self-contradictory intent is reported as such rather than
    as a pile of board violations.
    """
    out: List[Violation] = []
    env = intent.envelope.get('rect')

    for z in intent.blocks:
        if z.rect is None:
            continue
        if env is not None and not _rect_contains(env, z.rect):
            out.append(Violation(
                rule='intent_zone_outside_envelope',
                severity=intent.severity_of('intent_zone_outside_envelope'),
                block=z.name,
                message=(f"zone {z.name!r} {_fmt_rect(z.rect)} is not inside "
                         f"the envelope {_fmt_rect(env)}"),
                measured={'zone': list(z.rect)},
                expected={'envelope': list(env)}))

    # Two zones overlapping on a shared side is an intent that cannot be
    # satisfied, whatever the board does.
    for i, a in enumerate(intent.blocks):
        for b in intent.blocks[i + 1:]:
            if a.rect is None or b.rect is None:
                continue
            if a.side and b.side and a.side != b.side:
                continue
            area = legality.rect_overlap_area(a.rect, b.rect)
            if area > legality.EPS:
                out.append(Violation(
                    rule='intent_zone_overlap',
                    severity=intent.severity_of('intent_zone_overlap'),
                    block=a.name,
                    message=(f"zones {a.name!r} and {b.name!r} overlap by "
                             f"{area:.2f}mm2 on the same side; no placement can "
                             f"satisfy both"),
                    measured={'overlap_area_mm2': round(area, 4),
                              'other_block': b.name},
                    expected={'overlap_area_mm2': 0.0}))
    return out


# --------------------------------------------------------------------------
# geometry helpers
# --------------------------------------------------------------------------

def _rect_contains(outer, inner, tol: float = 0.0) -> bool:
    return (inner[0] >= outer[0] - tol and inner[1] >= outer[1] - tol
            and inner[2] <= outer[2] + tol and inner[3] <= outer[3] + tol)


def _rect_escape(outer, inner) -> Tuple[float, str]:
    """How far `inner` sticks out of `outer`, and on which side. 0.0 when in."""
    worst, axis = 0.0, ''
    for amount, name in ((outer[0] - inner[0], 'west'),
                         (outer[1] - inner[1], 'north'),
                         (inner[2] - outer[2], 'east'),
                         (inner[3] - outer[3], 'south')):
        if amount > worst:
            worst, axis = amount, name
    return worst, axis


def _fmt_rect(r) -> str:
    return f"[{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}, {r[3]:.2f}]"


def _ceil4(v: float) -> float:
    """Round UP to 4 decimals. See the legality_budget note in emit_intent."""
    return math.ceil(v * 1e4 - 1e-9) / 1e4


def _rects_touch(a, b) -> bool:
    return legality.rect_overlap_area(a, b) > legality.EPS


def _circle_hits_rect(cx, cy, radius, rect) -> bool:
    nx = min(max(cx, rect[0]), rect[2])
    ny = min(max(cy, rect[1]), rect[3])
    return math.hypot(nx - cx, ny - cy) < radius


# --------------------------------------------------------------------------
# keep-outs: ONE resolver and ONE hit test, shared by the grader (rule_keepout,
# below) and the seat predicate (seeder.pose_ok / seeder.edge_seat_ok).
#
# They live here, together, because the alternative measured badly elsewhere in
# this module: docs/floorplan-intent.md says of the rules that "every one of
# them measures with the geometry the OPTIMIZER ITSELF gates on", and until
# #701 the keepout row was the one place that was false -- the optimizer gated
# on nothing at all. A seat the search accepts that the grade then flags is an
# exit 4 on a board the seeder produced correctly, so the two must not be two
# implementations.
# --------------------------------------------------------------------------

def keepouts_for_ref(keepouts, ref: str, sides) -> Tuple[Dict, ...]:
    """The keep-out entries that BIND `ref`: not exempted by `allow`, and
    sharing at least one face with it.

    Pose-INVARIANT by construction -- an fnmatch against a reference and the
    set of faces a part occupies are both unchanged by moving it -- which is
    what lets a seat search resolve this ONCE per part instead of once per
    candidate pose. `_try_place` evaluates thousands of poses per part.

    Both filters live here rather than at each caller, for the reason
    `Intent.edge_claims` gives about its own split: a filter that must be
    remembered is a filter that will be forgotten at the next call site. If
    the seat honoured an `allow` glob the grade ignored, a mounting-hole
    keep-out would strand its own mounting hole.
    """
    out = []
    for k in keepouts:
        if any(allow_pattern_matches(pat, ref) for pat in (k.get('allow') or ())):
            continue
        if not (set(sides) & set(k.get('sides') or ('F', 'B'))):
            continue
        out.append(k)
    return tuple(out)


def allow_pattern_matches(pattern: str, ref: str) -> bool:
    """Does ONE `allow` glob exempt ONE reference? (#793)

    THE match, split out of `keepouts_for_ref` so the audit that asks "did this
    pattern exempt ANYTHING" cannot answer it with a different matcher than the
    exemption itself uses. A warning that disagrees with the resolver is worse
    than no warning: it would send an author to fix a pattern that works, or
    stay quiet about one that does not.

    `fnmatch.fnmatch`, deliberately, not `fnmatchcase`: it applies
    `os.path.normcase`, so matching is case-insensitive on Windows and
    case-sensitive elsewhere. That platform split is PRE-EXISTING and is not
    fixed here -- the point of this function is that both callers inherit
    exactly the same behaviour, whatever it is.
    """
    return fnmatch.fnmatch(ref, pattern)


def unresolved_keepout_allows(intent, pcb_data) -> List['Violation']:
    """`allow` globs that match no reference on this board (#793).

    The same failure class as `block_unresolved`, one construct over: a pattern
    the author believes grants an exemption, that `keepouts_for_ref` never
    matches. The consequence is worse than a no-op, because since #701 the seat
    search consults the same resolver -- so a typo does not merely fail to
    exempt the part, it STRANDS the part the keep-out was drawn around.

    What the author saw before this, measured on splitflap_driver with a
    keep-out over H1 carrying `allow: ["H01"]`:

      * with H1 inside it, `rule_keepout` fires -- but its message is
        "H1 (F) is inside keep-out 'mount-NW'", which describes the part and
        never the exemption. The failing pattern reaches the JSON in
        `expected.allow` and reaches the TEXT nowhere, and nothing anywhere
        says it matched nothing;
      * with the keep-out over empty space -- the state a board is in BEFORE
        placement, which is when an intent is authored -- `violations 0,
        errors 0, pass true`, exit 0. Silent, exactly as #793 says.

    WARN by default, and settable. Not the forced-WARN of
    `rule_edge_connector`'s `connector_affinity` branch: an author who wants a
    stale glob to fail CI writes `{"keepout_allow_unresolved": "error"}`, and
    `severity_of` gives that for free. Note `severity_of` DEFAULTS TO ERROR, so
    the `default=WARN` here is load-bearing rather than decorative.

    PER PATTERN, not per entry. `allow: ["H1", "H01"]` must still report the
    typo -- an `any()` over the tuple sees one match and says nothing, which is
    the bug this function exists to be.

    Resolved means "matches SOME reference on the board", deliberately not
    "the exemption changes an outcome". A pattern naming a real part that the
    keep-out would not have bound anyway (wrong side) is not a typo, and
    reporting it would put a finding on a correct spec.
    """
    refs = sorted((pcb_data.footprints or {}))
    out: List[Violation] = []
    for k in intent.keepouts:
        dead = [p for p in (k.get('allow') or ())
                if not any(allow_pattern_matches(p, r) for r in refs)]
        if not dead:
            continue
        name = str(k.get('name') or '<unnamed>')
        shown = ', '.join(refs[:6]) + (', ...' if len(refs) > 6 else '')
        out.append(Violation(
            rule='keepout_allow_unresolved',
            severity=intent.severity_of('keepout_allow_unresolved',
                                        default=WARN),
            message=(f"keep-out {name!r}: allow pattern(s) "
                     f"{', '.join(repr(p) for p in dead)} match no footprint "
                     f"on this board ({shown}). They exempt NOTHING, and since "
                     f"#701 the seat search refuses that part's pose too -- so "
                     f"a stale pattern strands the very part the keep-out was "
                     f"drawn around, rather than merely failing to excuse it"),
            measured={'keepout': name, 'unmatched': list(dead),
                      'matched': [p for p in (k.get('allow') or ())
                                  if p not in dead],
                      'available': refs[:12]},
            expected={'allow': list(k.get('allow') or ())}))
    return out


def keepout_hit(entry, rects) -> float:
    """How far into keep-out `entry` any of `rects` reaches; 0.0 when clear.

    THE hit test. The `legality.EPS` thresholding is INSIDE this function, not
    at the callers: two `> EPS` comparisons at two call sites are two chances
    to drift, and a pose the seeder accepts at the boundary that the grade
    then flags is exactly the round trip this exists to keep closed.

    `rects` may contain None -- `quench._Part.rects()` returns
    `(courtyard, None)` for a part with no drilled pads -- so both callers can
    pass their own natural shape without a branch.

    A rect entry returns the overlap AREA in mm2. A circle entry returns 1.0:
    a MARKER, not a measurement. Nothing in this tree computes circle/rect
    intersection area, and returning a fabricated one would be a figure a
    reader could quote.
    """
    hit = 0.0
    for r in rects:
        if r is None:
            continue
        if entry.get('rect') is not None:
            hit = max(hit, legality.rect_overlap_area(r, entry['rect']))
        else:
            cx, cy, radius = entry['circle']
            if _circle_hits_rect(cx, cy, radius, r):
                hit = max(hit, 1.0)
    return hit if hit > legality.EPS else 0.0


# --------------------------------------------------------------------------
# the board's own outline, checked before anything is graded against it
# --------------------------------------------------------------------------

def _is_simple_rectangle(segments) -> bool:
    """The parser's own rectangle short-circuit, reproduced.

    `extract_board_contours` returns ([], []) for THREE different reasons:
    fewer than 3 segments, a simple axis-aligned 4-segment rectangle (where the
    bounding box IS the outline, exactly), and a chaining failure. Only the
    third is a defect, and `board_outlines` alone cannot tell them apart -- so
    the same test is applied here rather than guessed at.
    """
    if len(segments) != 4:
        return False
    vertices = set()
    for seg in segments:
        vertices.add((round(seg[0][0], 3), round(seg[0][1], 3)))
        vertices.add((round(seg[1][0], 3), round(seg[1][1], 3)))
    if len(vertices) != 4:
        return False
    return all(abs(s[0][0] - s[1][0]) < 0.001 or abs(s[0][1] - s[1][1]) < 0.001
               for s in segments)


def outline_state(pcb_data, pcb_file: str = '') -> Dict[str, object]:
    """What the parser made of Edge.Cuts, and whether it can be trusted.

    A broken outline degrades SILENTLY today: unclosable segment groups are
    dropped, `extract_board_contours` returns ([], []), `BoardOutlineGate.active`
    goes False, and every containment test quietly falls back to the bounding
    box. No exception, no warning. A grader that inherits that fallback reports
    a clean board because it stopped checking -- the single worst thing this
    file could do.

    So the envelope is checked before anything is graded against it:

      * Edge.Cuts geometry exists but `board_bounds` is None. That is #550:
        `extract_board_bounds` reads neither board-level `gr_circle` nor
        `gr_curve`, so a round board reads as having no outline at all on the
        text path while the pcbnew path sees it fine.
      * A parsed ring reaches OUTSIDE `board_bounds` -- the same bug, curve
        flavour, where the bbox scan misses the bulge.
      * Segments exist, they are not a simple rectangle, and no ring chained.

    None of these are graded around; the run refuses and says which.
    """
    bi = pcb_data.board_info
    outlines = list(getattr(bi, 'board_outlines', None) or [])
    if not outlines and getattr(bi, 'board_outline', None):
        outlines = [bi.board_outline]
    cutouts = list(getattr(bi, 'board_cutouts', None) or [])
    contours = list(getattr(bi, 'board_edge_contours', None) or [])
    bounds = bi.board_bounds

    segments = []
    if pcb_file and os.path.exists(pcb_file):
        try:
            from kicad_parser import _collect_edge_cuts_segments
            with open(pcb_file, encoding='utf-8') as fh:
                segments = _collect_edge_cuts_segments(fh.read())
        except (OSError, ImportError, ValueError):
            segments = []
    rectangle = _is_simple_rectangle(segments)

    problems: List[str] = []
    ring_pts = [p for ring in outlines for p in ring]
    if bounds is None:
        if ring_pts or segments:
            problems.append(
                f"the board has {len(segments)} Edge.Cuts segment(s) but "
                f"board_bounds is None (#550: extract_board_bounds reads "
                f"neither gr_circle nor gr_curve, so a round or curve-cornered "
                f"board reads as having no outline)")
        else:
            problems.append("the board has no Edge.Cuts outline")
    elif ring_pts:
        out = max(max(bounds[0] - x, x - bounds[2], bounds[1] - y,
                      y - bounds[3]) for x, y in ring_pts)
        if out > 1e-3:
            problems.append(
                f"a parsed outline ring reaches {out:.3f}mm outside "
                f"board_bounds (#550: the bbox scan missed a curve or circle)")
    if not outlines and len(segments) >= 3 and not rectangle:
        problems.append(
            f"{len(segments)} Edge.Cuts segments chained into no closed ring, "
            f"and they are not a simple rectangle; containment would silently "
            f"fall back to the bounding box")

    return {'bounds': tuple(round(v, 6) for v in bounds) if bounds else None,
            'outlines': len(outlines), 'cutouts': len(cutouts),
            'edge_contours': len(contours),
            'edge_segments': len(segments),
            'simple_rectangle': rectangle,
            'problems': problems,
            'trustworthy': not problems}


# --------------------------------------------------------------------------
# block resolution
# --------------------------------------------------------------------------

def resolve_blocks(intent: Intent, pcb_data, group_sources: Sequence[str] = ()
                   ) -> Tuple[Dict[str, List[str]], List[Violation]]:
    """{block name: sorted refs} plus a violation per block that resolved empty.

    A block is `refs` globs, `group` membership, or both unioned. `group` is
    matched against the raw derive_groups key AND its `short_name` form, because
    `short_name` is what `--list-groups` prints and therefore what anyone would
    copy into an intent file.
    """
    refs_all = sorted(pcb_data.footprints)
    derived: Dict[str, List[str]] = {}
    if group_sources and any(z.group for z in intent.blocks):
        derived = groups_mod.derive_groups(pcb_data, tuple(group_sources))
    by_short = {}
    for key, members in derived.items():
        by_short.setdefault(groups_mod.short_name(key), []).extend(members)

    out: Dict[str, List[str]] = {}
    problems: List[Violation] = []
    for z in intent.blocks:
        members = set()
        for pattern in z.refs:
            members.update(fnmatch.filter(refs_all, pattern))
        if z.group:
            found = derived.get(z.group)
            if found is None:
                found = by_short.get(z.group)
            if found is None:
                problems.append(Violation(
                    rule='block_unresolved', block=z.name,
                    severity=intent.severity_of('block_unresolved'),
                    message=(f"block {z.name!r}: group {z.group!r} does not "
                             f"exist on this board. Available: "
                             f"{', '.join(sorted(by_short)[:6]) or 'none'}"
                             f"{' ...' if len(by_short) > 6 else ''}. Derive "
                             f"them with --group-by and --list-groups first"),
                    measured={'group': z.group,
                              'available': sorted(by_short)[:12]}))
            else:
                members.update(found)
        out[z.name] = sorted(members)
        if not members:
            problems.append(Violation(
                rule='block_unresolved', block=z.name,
                severity=intent.severity_of('block_unresolved'),
                message=(f"block {z.name!r} matched no footprint on this board "
                         f"(refs {list(z.refs)!r}). A block that resolves to "
                         f"nothing grades clean, so this is an error rather "
                         f"than an empty block"),
                measured={'refs': list(z.refs), 'matched': 0}))
    return out, problems


def _swallows(entry, rect) -> bool:
    """Does this keep-out cover `rect` entirely?

    Rect: containment. Circle: all four corners inside, which is exactly the
    condition for a convex disc to contain a rectangle -- four `hypot` calls,
    not the fabricated area `keepout_hit` declines to invent.
    """
    r = entry.get('rect')
    if r is not None:
        return (r[0] <= rect[0] and r[1] <= rect[1]
                and r[2] >= rect[2] and r[3] >= rect[3])
    cx, cy, rad = entry['circle']
    return all(math.hypot(px - cx, py - cy) <= rad
               for px in (rect[0], rect[2]) for py in (rect[1], rect[3]))


def _inflate(rect, by: float):
    return (rect[0] - by, rect[1] - by, rect[2] + by, rect[3] + by)


def zone_covered_by_keepout(zone, keepouts, member_sides=None,
                            tolerance: float = 0.0) -> Optional[str]:
    """The name of a keep-out that swallows `zone` whole for a member that it
    actually BINDS, or None (#702).

    An intent whose keep-out covers the region its zone demands is a
    CONTRADICTION, and `validate_intent` cannot see it: it checks
    zone-inside-envelope and zone-vs-zone overlap, and nothing compares a zone
    against a keep-out.

    That was harmless while the intent was only graded -- the part took two
    findings and the optimizer moved it anyway. Under the #702 gate it is not:
    the rule is termwise-monotone, so `keepout` falls only by leaving the
    keep-out, leaving raises `zone_containment`, and no candidate can lower
    both. The part is CONFINED TO ITS ZONE for the run, where the pre-#702
    quench would have walked it out.

    Confined, not frozen -- the distinction was measured, and the message says
    the weaker true thing rather than the stronger false one. Every pose inside
    a fully-swallowed zone yields an identical term vector (the intrusion is
    the constant full-courtyard area, the escape is 0), so the monotone rule
    admits all of them: 6 of 8 probed alternative poses were accepted. What the
    member can never do is get OUT.

    `member_sides` is `{ref: sides}` for the block's resolved members, and it
    is what makes this test the same question the GRADE asks. Without it the
    check reads raw `intent.keepouts` and ignores both filters
    `keepouts_for_ref` exists to centralize -- so a `sides: ["B"]` keep-out
    over an F-side block, or the mounting-hole `allow: ["MH1"]` pattern over
    MH1's own zone, would each be reported as a contradiction at ERROR while
    the grade raises no `keepout` finding at all. Measured: both did.

    Only TOTAL coverage. A partial overlap is a legitimate intent -- a zone
    with a corner bitten out still has room -- and deciding whether what is
    left can actually hold the part is a different (harder) question than this
    one.
    """
    if zone.rect is None:
        return None
    # The zone a member must satisfy is its rect PLUS its tolerance -- that is
    # what `rule_zone_containment` grades against -- so a keep-out that covers
    # the bare rect but not the tolerance band leaves real poses. Measured on a
    # 4x4 zone at tolerance 2.0 with the keep-out equal to the rect: 5 probed
    # poses satisfy both rules, and without this the intent was refused at
    # ERROR anyway.
    reach = _inflate(zone.rect, tolerance)
    for k in keepouts:
        if not _swallows(k, reach):
            continue
        if member_sides is None:
            return str(k.get('name') or '<unnamed>')
        # Binding is per member, through the SAME resolver the seat predicate
        # and the grade use, so `allow` globs and `sides` are honoured here
        # exactly as they are there.
        for ref, sides in member_sides.items():
            if keepouts_for_ref((k,), ref, sides):
                return str(k.get('name') or '<unnamed>')
    return None


def resolve_intent_gate(intent: Intent, pcb_data,
                        group_sources: Sequence[str] = ()
                        ) -> Tuple[Dict[str, object], List[Violation]]:
    """The pose-INVARIANT join a per-move gate needs, plus the problems (#702).

    `resolve_blocks` returns refs and NO geometry, so every consumer has had to
    join `block name -> Zone.rect` itself; `place_seed.py` was the first copy
    and four more quench call sites were about to add theirs. One resolver
    instead, for the reason `Intent.edge_claims` gives about its own split: a
    filter that must be remembered is a filter that will be forgotten at the
    next call site.

    Returns PLAIN DATA -- dicts and tuples, no `Intent`, no `Zone`. The quench
    must not have to import this schema to run, and `grade` builds a
    QuenchState of its own that must keep measuring INDEPENDENTLY of whatever
    the optimizer was gated on (tests/test_701_keepout_predicate.py:395).

    `lock_refs` carries the two rules that are enforced by FREEZING rather than
    by a pose term, because neither is a property of a pose:

      * `must_lock` is a claim about the FILE. No pose satisfies or violates
        it. Freezing does not launder the grade -- the file is still unstamped,
        so `rule_must_lock` still fires and still tells the author to stamp it.
      * `edge_claims()`, NOT `edge_connectors`: a `connector_affinity` entry
        makes no seat claim and must not be locked out of the search. That
        split is measured in `Intent.edge_claims`' own docstring (6 extra refs
        on tigard_placed).
    """
    blocks, problems = resolve_blocks(intent, pcb_data, group_sources)
    zones = tuple(
        {'name': z.name,
         'rect': tuple(z.rect),
         'tolerance_mm': intent.zone_tolerance(z),
         'refs': tuple(blocks.get(z.name, ())),
         'side': z.side,
         'exclusive': bool(z.exclusive)}
        for z in intent.blocks if z.rect is not None)

    # Total coverage AND the per-member "leaves it no pose" case (#799), from
    # the one function `grade` also calls -- so the same intent on the same
    # board cannot get a different verdict depending on which entry point asked.
    problems.extend(intent_zone_keepout_problems(intent, blocks, pcb_data))

    # #793, raised HERE as well as in `grade`: the gate is what the four
    # quenching CLIs run, and it is where a stale `allow` is actively stranding
    # the part right now. One raiser, two reach points -- `block_unresolved`'s
    # own shape, and for the same reason.
    problems.extend(unresolved_keepout_allows(intent, pcb_data))

    lock: set = set()
    for pat in intent.must_lock:
        lock.update(fnmatch.filter(sorted(pcb_data.footprints), pat))
    lock.update(str(c['ref']) for c in intent.edge_claims()
                if c.get('ref') in pcb_data.footprints)
    return ({'zones': zones,
             'keepouts': tuple(intent.keepouts),
             'lock_refs': tuple(sorted(lock))}, problems)


# --------------------------------------------------------------------------
# rules
# --------------------------------------------------------------------------

class _Ctx:
    """What every rule is handed. Built once; rules never re-derive geometry."""

    def __init__(self, intent, pcb_data, pcb_file, state, blocks, locked,
                 outline):
        self.intent = intent
        self.pcb = pcb_data
        self.pcb_file = pcb_file
        self.state = state
        self.blocks = blocks
        self.locked = locked
        self.outline = outline
        self.outline_bounds = outline.get('bounds')
        self.parts = {p.ref: p for p in state.graded_parts()}
        self.gate = state.edge_gate
        self.legality = state.legality_metrics()
        self.envelope = intent.envelope.get('rect')
        self.owner: Dict[str, str] = {}
        for name, refs in sorted(blocks.items()):
            for r in refs:
                self.owner.setdefault(r, name)

    def sev(self, rule: str) -> str:
        return self.intent.severity_of(rule)


def _nearest_edge(rect, bounds) -> str:
    d = {'west': rect[0] - bounds[0], 'north': rect[1] - bounds[1],
         'east': bounds[2] - rect[2], 'south': bounds[3] - rect[3]}
    return min(d, key=lambda k: d[k])


def rule_envelope(ctx) -> Iterator[Violation]:
    """The declared envelope must BE the board outline, not a wish about it.

    A mismatch is never a licence to resize. It means the intent was written
    against a different board or revision, and grading parts against the wrong
    rectangle reports violations that are really the author bookkeeping.
    """
    env = ctx.envelope
    bounds = ctx.outline_bounds
    if env is None or bounds is None:
        return
    tol = float(ctx.intent.envelope.get('tolerance_mm',
                                        DEFAULT_ENVELOPE_TOLERANCE_MM))
    delta = max(abs(a - b) for a, b in zip(env, bounds))
    if delta > tol:
        yield Violation(
            rule='envelope', severity=ctx.sev('envelope'),
            message=(f"the intent envelope {_fmt_rect(env)} is not this board "
                     f"outline {_fmt_rect(bounds)} (worst corner {delta:.2f}mm "
                     f"> {tol}mm). The outline is fixed: correct the intent, "
                     f"do not resize the board"),
            measured={'board_bounds': [round(v, 4) for v in bounds],
                      'worst_corner_mm': round(delta, 4)},
            expected={'envelope': list(env), 'tolerance_mm': tol})


def zone_fits_courtyard(zone_rect, part_rect, tol: float) -> bool:
    """Can this zone geometrically contain this part's courtyard at ANY
    90-degree rotation? False means the zone is a spec-COORDINATE (a tight
    rect around where the part belongs), and containment must be graded on
    the part's anchor point instead -- a courtyard-containment demand against
    a zone smaller than the courtyard is unsatisfiable by construction (the
    run-2 R3 failure: place_seed could never seat H1/H3 in their 0.4mm
    zones)."""
    zw = zone_rect[2] - zone_rect[0] + 2 * tol
    zh = zone_rect[3] - zone_rect[1] + 2 * tol
    w = part_rect[2] - part_rect[0]
    h = part_rect[3] - part_rect[1]
    return (w <= zw + 1e-9 and h <= zh + 1e-9) or \
           (h <= zw + 1e-9 and w <= zh + 1e-9)


def zone_escape(zone_rect, part_rect, anchor: bool) -> Tuple[float, str]:
    """How far `part_rect` is outside `zone_rect`, in mm, and on which side.
    Exactly 0.0 when contained.

    THE zone measurement, shared by `rule_zone_containment` and the quench's
    intent gate (#702), for the reason `keepout_hit`'s docstring gives about
    the keep-out one: a pose the optimizer accepts that the grade then flags is
    an exit 4 on a board this tool placed itself, and two implementations of
    "outside its zone" is how that happens.

    `anchor` selects the spec-COORDINATE branch -- grade the courtyard CENTRE,
    because a zone smaller than the courtyard cannot contain it at any
    rotation. The CALLER passes the decision `zone_fits_courtyard` makes,
    because it is pose-INVARIANT (it reads only w/h, and tests both orders) and
    a per-pose gate must resolve it once per part rather than once per
    candidate pose.

    Note what this is NOT: `seeder.zone_gate`'s anchor branch tests the
    footprint ORIGIN (x, y), not the courtyard centre. The two differ by
    (b[0]+b[2])/2, and `_feasible_centre_box` records how much that is -- 17 of
    65 parts on splitflap_driver and 6 of 89 on tigard have an offset centre,
    up to 10.15mm on tigard J3. A gate built on the seeder's predicate would
    admit, by up to 10mm, poses this rule flags.
    """
    if anchor:
        cx = (part_rect[0] + part_rect[2]) / 2.0
        cy = (part_rect[1] + part_rect[3]) / 2.0
        return _rect_escape(zone_rect, (cx, cy, cx, cy))
    return _rect_escape(zone_rect, part_rect)


def zone_is_anchor(zone_rect, part, tol: float) -> bool:
    """Is this zone a spec-COORDINATE rather than a region? (#799)

    ONE definition, for the three consumers that must agree: `seeder.zone_gate`
    picks its containment branch with it, `rule_zone_containment` grades on it,
    and the intent contradiction check asks it before deciding where a member
    may sit. It used to live inline in `zone_gate`, which meant a load-time
    check asking the same question had to re-derive it -- and a re-derivation
    that disagreed would move the anchor boundary for one consumer only.

    Pose-INVARIANT: `zone_fits_courtyard` reads only w/h and tests both orders,
    so `part.rot` and `part.rot + 90` settle it for the whole 90-degree
    lattice.
    """
    return not any(
        zone_fits_courtyard(zone_rect, part.rect(0.0, 0.0, r), tol)
        for r in (part.rot % 360, (part.rot + 90) % 360))


def zone_origin_box(zone_rect, bounds, tol: float):
    """Where a part with LOCAL box `bounds` may put its ORIGIN, at ONE rotation.

    `zone_escape(zone_rect, part_rect, anchor=False) <= tol`, solved for the
    origin. Containment at this rotation is `zone[0]-tol <= x + b[0]` and
    `x + b[2] <= zone[2]+tol`, so `x` in `[zone[0]-tol-b[0], zone[2]+tol-b[2]]`.

    Returns a possibly-INVERTED box: `hi < lo` on an axis means this rotation
    admits nothing. That is `seeder._feasible_centre_box`'s own convention
    (`zone_census_offsets` detects the inversion), and it is kept rather than
    returning None so the two callers branch the same way.

    THE COURTYARD IS NOT CENTRED ON THE FOOTPRINT ORIGIN, so this is the
    algebra and never a half-extent -- `_feasible_centre_box`'s docstring
    records two earlier forms that were wrong here in opposite directions, and
    the offset reaches 10.15mm on tigard J3.

    Lives here, beside `zone_escape` whose inverse it is, because the intent
    contradiction check (#799) needs it PER ROTATION while the seeder needs
    the union over rotations. One algebra, two shapes.
    """
    x0, y0, x1, y1 = (float(v) for v in zone_rect)
    b0x, b0y, b2x, b2y = bounds
    return (x0 - tol - b0x, y0 - tol - b0y, x1 + tol - b2x, y1 + tol - b2y)


def _anchor_origin_box(zone_rect, bounds, tol: float):
    """`zone_origin_box`'s spec-COORDINATE twin: the zone holds the part's
    courtyard CENTRE rather than its whole courtyard.

    `zone_escape(..., anchor=True)` measures `((b0+b2)/2, (b1+b3)/2)` offset
    from the origin, so the admissible origin box is the zone shifted by minus
    that offset. THE GRADE'S convention, deliberately -- `seeder.zone_gate`
    constrains the footprint ORIGIN instead, and `zone_escape`'s docstring
    records that the two differ by up to 10.15mm on tigard J3.

    An earlier draft of #799 intersected the two conventions, on the theory
    that a refusal should be true under both. Measured, that is a false-ERROR
    machine: 234 of 1316 corpus parts have an off-centre courtyard (worst
    36.825mm, kit-dev MCU_PORT201), and where the offset exceeds the zone extent
    the two boxes are DISJOINT -- so the intersection is empty and ANY keep-out
    anywhere on the board refuses the intent. This finding is a contradiction
    between two GRADED claims, `zone_containment` and `keepout`, and both are
    measured on the grade's convention. That is the one to use.
    """
    x0, y0, x1, y1 = (float(v) for v in zone_rect)
    b0x, b0y, b2x, b2y = bounds
    cx, cy = (b0x + b2x) / 2.0, (b0y + b2y) / 2.0
    return (x0 - tol - cx, y0 - tol - cy, x1 + tol - cx, y1 + tol - cy)


def _forbidden_origin_rect(entry, bounds):
    """Origins at which a part with LOCAL box `bounds` would HIT rect `entry`.

    `keepout_hit` fires when `rect_overlap_area > EPS`, and that area is
    positive exactly when the placed box and the keep-out overlap on BOTH axes,
    so the forbidden origin set is the OPEN rect

        (k0 - b2x,  k1 - b2y,  k2 - b0x,  k3 - b0y)

    Open, because touching is legal. Never a half-extent: the courtyard is not
    centred on the footprint origin, and a symmetric deflation SHIFTS the box.

    `None` for a circle (no disc/rect kernel exists here) and for a DEGENERATE
    rect. Degenerate matters: `_rect` NORMALISES an inverted rect rather than
    refusing it, so a typo'd `[15,20,15,10]` loads as a zero-width keep-out --
    which `rect_overlap_area` can never report a positive area for, so it can
    never hit anything, while this rect would still forbid a 2*courtyard-wide
    band of origins. Returning None keeps it out of the candidate geometry;
    `keepout_hit` still gets the final say on every candidate.
    """
    k = entry.get('rect')
    if k is None:
        return None
    if not (k[2] - k[0] > 0.0 and k[3] - k[1] > 0.0):
        return None
    b0x, b0y, b2x, b2y = bounds
    return (k[0] - b2x, k[1] - b2y, k[2] - b0x, k[3] - b0y)


def _axis_candidates(lo: float, hi: float, cuts) -> List[float]:
    """Coordinates worth testing on one axis: the ends, every cut clamped into
    `[lo, hi]`, and the midpoint of each consecutive pair.

    The cut coordinates themselves must be in the set, not only the midpoints.
    A gap EXACTLY as wide as the courtyard leaves a measure-zero line of legal
    origins, and that line is a keep-out edge -- a midpoints-only sample set
    reports such an intent unsatisfiable, which it is not.

    Cuts are CLAMPED into the box; the rects they came from are NOT clipped.
    Clipping the holes and then testing strictly is a real and tempting bug:
    on #799's own counterexample the hole clips to exactly the box, its corner
    stops being strictly interior, and the check reports the contradiction
    feasible -- passing every test written from the issue.
    """
    xs = {lo, hi}
    for c in cuts:
        if c < lo:
            c = lo
        elif c > hi:
            c = hi
        xs.add(c)
    out = sorted(xs)
    return out + [(a + b) / 2.0 for a, b in zip(out, out[1:])]


#: Bound keep-outs past which the feasibility search abstains rather than
#: paying O(k^2) candidates x O(k) hit tests. Abstaining reports FEASIBLE, so
#: the cap can only cost a missed contradiction, never invent one.
_JOINT_KEEPOUT_CAP = 16


def zone_pose_feasibility(zone_rect, tolerance: float, part,
                          keepouts) -> Dict[str, object]:
    """Does `zone_rect` MINUS the keep-outs still hold `part` at some rotation?

    #702 refuses an intent whose keep-out swallows a zone entirely. That is the
    total-coverage case; this is the question underneath it, and they coincide
    only there. Measured: zone `[10,10,20,20]` at tolerance 0 against a keep-out
    `[10,10,19.9,20]` -- 99% of the zone -- raises nothing today while the
    member has ZERO satisfying poses. Two keep-outs covering half each are
    missed the same way, and neither triggers a per-entry test.

    Why it must be refused where it is authored rather than discovered later:
    the #702 quench gate is termwise-monotone, so for such a member `keepout`
    falls only by leaving the zone, leaving raises `zone_containment`, and no
    candidate lowers both. The member is CONFINED TO ITS ZONE for the whole run.
    Confined, not frozen -- every pose inside the zone yields an identical term
    vector, so the rule admits all of them; what the member can never do is get
    OUT.

    THE INVARIANT: compute a SUPERSET of the poses satisfying (zone AND binding
    keep-outs) and refuse only when that superset is empty -- so a refusal is
    sound, up to the one bounded exception recorded below.

    THE ALGEBRA ONLY PROPOSES. Candidate origins come from coordinate
    compression over the admissible box and the keep-outs' forbidden rects, and
    every candidate is then judged by `zone_escape` and `keepout_hit` -- the
    same two functions `rule_zone_containment` and `rule_keepout` grade with.
    (Not `seeder.pose_ok`: it applies no zone predicate at all, and its
    caller's `zone_gate` uses `_rect_inside` and a bare origin-in-zone test
    rather than `zone_escape`. The kernel matches the GRADE, which is whose
    contradiction this is.) So the verdict cannot drift from the grade,
    and two float-scale traps disappear on their own: a keep-out a candidate
    overlaps by less than `EPS` of AREA is not a hit and the candidate stands,
    and a degenerate keep-out forbids nothing because it can hit nothing.

    ONE KNOWN EXCEPTION TO "A FALSE ERROR IS IMPOSSIBLE", stated because it is
    real. `keepout_hit` fires on overlap AREA above `EPS`, so the true
    satisfying set is slightly LARGER than the open-rect complement the
    candidates are drawn from -- and a free window can therefore fall strictly
    between two sampled coordinates. Derived analytically and then run: zone
    (0,0,10,2) tol 0, a 2x2 courtyard, keep-outs (-99,0.5,4,1) and
    (5.9999978,-10,99,10) are refused, while (4.99999815, 1.0, 0) has
    `zone_escape` 0.0 and `keepout_hit` 0.0 against both (raw areas 9.25e-7 and
    7.0e-7, under EPS). For any two binding keep-outs with part-overlap lengths
    L1 <= L2 on the free axis the window exists when the gap d satisfies
    max(EPS/L1, 2*EPS/L2) < d <= EPS/L1 + EPS/L2.

    It is bounded rather than open-ended: a missed pose must graze EVERY
    binding keep-out by under one square micrometre, i.e. far below the 0.05mm
    floor of any lattice the seat search sweeps, so no authored intent reaches
    it. It is NOT modelled, because widening the candidate set to cover the
    slack would invent tolerance the grade does not have -- and the grade would
    then flag the pose this function admitted. `tests/test_799_*` carries the
    counterexample as a recorded limitation so it is a change detector rather
    than a surprise.

    The zone side is slack by `EPS` and the keep-out side is not, and that
    asymmetry is measured rather than chosen. `(z0 - tol - b0) + b0 < z0 - tol`
    on a few percent of random triples (6.3% and 7.4% in two independent
    probes; the sampling distribution is not pinned, so treat it as the order
    of magnitude rather than a figure), so at `tolerance_mm: 0` a candidate sitting on a
    zone-derived edge is rejected by its own construction; on the keep-out side
    the worst boundary overlap over 200000 trials was 8.5e-12 mm2, six orders
    below `EPS`, so no slack is needed and adding it would invent tolerance the
    grade does not have.

    Returns plain data with EVERY key present, so a consumer never needs a
    defaulting `.get` to tell "nothing binds it" from "it was not considered":

        feasible   bool          -- False is the only value that refuses
        reason     str           -- see below
        witness    (x, y, rot)|None
        bound      (names,...)   -- keep-outs that BIND this ref
        keepouts_freeing (names,...)  -- each alone leaves it a pose
        keepouts_joint   (names,...)  -- none alone does; together they refuse
        undecided_circles (names,...) -- why an abstention abstained
        rotations  (floats,...)

    `reason` is one of `no_keepout_binds`, `seated`, `keepout_alone`,
    `keepout_any_of`, `keepout_joint`, `circle_undecided`, `too_many_keepouts`,
    `zone_too_small`. The three refusing ones say how the blame divides:
    ONE entry is the whole cause (`keepout_alone`), SEVERAL are each
    individually necessary so lifting any one would free it (`keepout_any_of`),
    or no single lift frees anything and they refuse jointly (`keepout_joint`).

    CIRCLES ABSTAIN, and the abstention is SCOPED. No disc/rect free-area
    kernel exists in this tree (`keepout_hit` returns a marker for a disc and
    says why), so a disc contributes no candidate geometry and the candidate set
    stops being provably sufficient -- a refusal would be unsound. But
    abstaining whenever a circle merely appears in the bound list would let one
    decorative disc anywhere on the board switch the check off for every part.
    So: when nothing verifies, the search is re-run over the RECTS ALONE. If
    the rects alone still refuse, the refusal is sound and is reported; only if
    dropping the discs would have found a pose is the answer undecided.
    `zone_covered_by_keepout` still decides total disc coverage exactly, which
    is what #702 shipped, so nothing regresses there.

    NOT MODELLED, deliberately: the board outline, clearance, and neighbours.
    A FEASIBLE verdict says the zone and the intent's own keep-outs leave room,
    never that the part can be seated -- `seeder.pose_ok` demands all three, and
    "no legal pose" already has a better-informed owner in the `keepout_blocks`
    verdict, which counts poses with the seat predicate and names the blocker.
    Modelling them here would shrink the feasible set, i.e. move toward refusing
    an intent that is fine.
    """
    out: Dict[str, object] = {
        'feasible': True, 'reason': 'no_keepout_binds', 'witness': None,
        'bound': (), 'keepouts_freeing': (), 'keepouts_joint': (),
        'undecided_circles': (), 'rotations': ()}
    if zone_rect is None:
        return out
    rot0 = float(part.rot) % 360
    rots = tuple((rot0 + d) % 360 for d in (0.0, 90.0, 180.0, 270.0))
    out['rotations'] = rots
    bound = tuple(keepouts)
    out['bound'] = tuple(str(k.get('name') or '<unnamed>') for k in bound)
    if not bound:
        return out
    if len(bound) > _JOINT_KEEPOUT_CAP:
        out['reason'] = 'too_many_keepouts'
        return out

    anchor = zone_is_anchor(zone_rect, part, tolerance)

    def _search(entries):
        """First (x, y, rot) satisfying zone AND every entry, or None."""
        for rot in rots:
            b = part.rect(0.0, 0.0, rot)
            t = part.tht_rect(0.0, 0.0, rot)
            box = (_anchor_origin_box(zone_rect, b, tolerance) if anchor
                   else zone_origin_box(zone_rect, b, tolerance))
            if box[2] < box[0] or box[3] < box[1]:
                continue
            holes = []
            for k in entries:
                for lb in ((b, t) if t is not None else (b,)):
                    f = _forbidden_origin_rect(k, lb)
                    if f is not None:
                        holes.append(f)
            cx = _axis_candidates(box[0], box[2],
                                  [v for h in holes for v in (h[0], h[2])])
            cy = _axis_candidates(box[1], box[3],
                                  [v for h in holes for v in (h[1], h[3])])
            for x in cx:
                for y in cy:
                    r = part.rect(x, y, rot)
                    th = part.tht_rect(x, y, rot)
                    if zone_escape(zone_rect, r, anchor)[0] > tolerance + legality.EPS:
                        continue
                    if any(keepout_hit(k, (r, th)) for k in entries):
                        continue
                    return (round(x, 6), round(y, 6), rot)
        return None

    seat = _search(bound)
    if seat is not None:
        out['reason'] = 'seated'
        out['witness'] = seat
        return out

    rects = tuple(k for k in bound if k.get('rect') is not None)
    discs = tuple(k for k in bound if k.get('rect') is None)
    if discs and (not rects or _search(rects) is not None):
        # Dropping the discs would have found a pose, so THEY are what refused
        # it -- and that is the one question this cannot answer exactly.
        out['reason'] = 'circle_undecided'
        out['undecided_circles'] = tuple(
            str(k.get('name') or '<unnamed>') for k in discs)
        return out

    # The rects alone refuse. Sound, so attribute it -- the same single-lift
    # then joint-lift escalation `seed_from_intent` does with the pose census,
    # in the currency this function has (names, from an exact search) rather
    # than the census's pose COUNTS. Inventing a count here would be a figure a
    # reader could quote.
    if _search(()) is None:
        # Nothing binds and it STILL does not fit: the zone is too small for
        # the part, which is `zone_containment`'s finding and not this one.
        # A named marker rather than a wrong message.
        out['feasible'] = True
        out['reason'] = 'zone_too_small'
        return out
    # Over ALL bound entries, not just the rects. Computing it over `rects`
    # made "lifting any one of them would give it a pose" FALSE whenever a disc
    # also bound the member: lifting the named rect left the disc still
    # refusing. Measured -- A=[-50,-50,4,50], B=[5.999,-50,50,50],
    # D=circle(7,5,4): the message named A and B, and lifting B alone left the
    # part with no pose at all.
    freeing = tuple(str(k.get('name') or '<unnamed>') for k in bound
                    if _search(tuple(e for e in bound if e is not k)) is not None)
    out['feasible'] = False
    if freeing:
        # `keepouts_freeing` carries the seeder's meaning: lifting this entry
        # frees a pose. ONE such entry is the sole cause; SEVERAL means each is
        # individually necessary and they refuse together, which is a different
        # sentence -- "alone leaves it none" would be false of every one of
        # them. Measured on two keep-outs covering half a zone each: both lifts
        # free a pose, so the honest report is "lifting any one would".
        out['reason'] = 'keepout_alone' if len(freeing) == 1 else 'keepout_any_of'
        # NOTE the exact claim `freeing` supports: "lifting this entry leaves a
        # pose". It does NOT support "this entry is the sole cause" -- two
        # OVERLAPPING keep-outs each mask the other, so neither appears here
        # while together they are the reason. Measured: two identical keep-outs
        # plus a third made the message name only the third, which alone left
        # 8mm2 free. The wording below says the thing that was computed.
        out['keepouts_freeing'] = freeing
    else:
        out['reason'] = 'keepout_joint'
        out['keepouts_joint'] = tuple(
            str(k.get('name') or '<unnamed>') for k in rects)
    return out


class _LocalPart:
    """`quench._Part`'s geometry interface over `legality.LocalBounds`.

    Both entry points build this, rather than `grade` using its QuenchState
    parts and the gate building something else: the same intent on the same
    board must get the same verdict whichever asked, and two geometry sources
    is how that stops being true.

    Rotation is exact for any angle (`rotate_local_bounds` rotates the corners
    and re-takes the bbox), so a part at a non-multiple of 90 gets its OWN
    lattice -- which is the lattice `_candidate_rotations` offers it. Scoring
    such a part on (0, 90, 180, 270) would credit poses the optimizer can never
    generate.
    """
    __slots__ = ('rot', '_b', '_t')

    def __init__(self, rot: float, local, tht_local=None):
        self.rot = float(rot) % 360
        self._b = tuple(local)
        self._t = tuple(tht_local) if tht_local is not None else None

    def rect(self, x: float, y: float, rot: float):
        b = legality.rotate_local_bounds(*self._b, rot)
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def tht_rect(self, x: float, y: float, rot: float):
        if self._t is None:
            return None
        t = legality.rotate_local_bounds(*self._t, rot)
        return (x + t[0], y + t[1], x + t[2], y + t[3])


def intent_zone_keepout_problems(intent, blocks, pcb_data,
                                 pcb_file: str = '') -> List['Violation']:
    """`intent_zone_in_keepout`, both the #702 case and the #799 one.

    TOTAL COVERAGE RUNS FIRST, unchanged, and its Violation is byte-identical to
    the one #702 shipped. Only when it says nothing does the per-member search
    run. Three things that buys: #702's behaviour is preserved exactly, a disc
    keeps the exact total-coverage answer `_swallows` can give it, and the
    compatibility argument becomes auditable -- the widened check can only ADD
    findings, and only where `_swallows` already said no.

    PER MEMBER, because the answer genuinely differs per member: a block's parts
    have different courtyards, different rotations and different `allow`
    standing. A per-block verdict computed on one member, or on a synthetic
    block bbox, is wrong for the others -- measured, a 2x2 part and an 8x8 part
    in the same zone under the same keep-out disagree.

    Members whose geometry is the +/-0.5mm FICTION (no courtyard AND no pads)
    are skipped: it is not geometry anyone drew, so it must never gate. Note
    `QuenchState.graded_parts()` never sets that flag -- only
    `legality.part_local_bounds` does -- so this reads it from there rather than
    from a `GradedPart`, which would silently always be False.
    """
    out: List[Violation] = []
    zoned = [z for z in intent.blocks
             if z.rect is not None and blocks.get(z.name)]
    if not zoned or not intent.keepouts:
        return out

    member_sides = {}
    for ref in sorted(pcb_data.footprints or {}):
        fp = pcb_data.footprints[ref]
        member_sides[ref] = legality.sides_occupied(
            legality.footprint_side(fp), legality.footprint_has_through_pads(fp))

    for z in zoned:
        tol = intent.zone_tolerance(z)
        mine = {r: member_sides[r] for r in blocks.get(z.name, ())
                if r in member_sides}
        hit = zone_covered_by_keepout(z, intent.keepouts, mine, tol)
        if hit is not None:
            out.append(Violation(
                rule='intent_zone_in_keepout', block=z.name,
                severity=intent.severity_of('intent_zone_in_keepout'),
                message=(f"block {z.name!r} declares a zone that keep-out "
                         f"{hit!r} covers entirely: its members are required "
                         f"to be somewhere they are forbidden to be. No pose "
                         f"satisfies both, so such a member can never LEAVE "
                         f"its zone under the optimizer's monotone intent "
                         f"gate, and cannot clear the keep-out without "
                         f"leaving it"),
                measured={'zone': list(z.rect), 'keepout': hit},
                expected={'overlap': 'partial or none'}))

    # Geometry only once the cheap tests are past, and only if some zoned
    # member is actually bound by a keep-out. On every board that declares no
    # keep-out -- which is every board `emit_intent` can write -- this function
    # has already returned, so it costs no file read at all.
    needs = [z for z in zoned
             if any(keepouts_for_ref(intent.keepouts, r, member_sides[r])
                    for r in blocks.get(z.name, ()) if r in member_sides)]
    if not needs:
        return out
    locals_ = legality.part_local_bounds(pcb_data, pcb_file or None)
    named = {v.block for v in out}

    for z in needs:
        if z.name in named:          # already reported as total coverage
            continue
        tol = intent.zone_tolerance(z)
        for ref in blocks.get(z.name, ()):
            lb = locals_.get(ref)
            if lb is None or lb.synthetic:
                continue
            bound = keepouts_for_ref(intent.keepouts, ref, member_sides[ref])
            if not bound:
                continue
            fp = pcb_data.footprints[ref]
            part = _LocalPart(fp.rotation or 0.0, lb.local, lb.tht_local)
            v = zone_pose_feasibility(z.rect, tol, part, bound)
            if v['feasible']:
                continue
            names = list(v['keepouts_freeing'] or v['keepouts_joint'])
            joint = v['reason'] == 'keepout_joint'
            which = ', '.join(repr(n) for n in names)
            out.append(Violation(
                rule='intent_zone_in_keepout', block=z.name, ref=ref,
                severity=intent.severity_of('intent_zone_in_keepout'),
                message=(
                    f"block {z.name!r} member {ref} has no pose inside its "
                    f"zone, at any of {len(v['rotations'])} rotations, that "
                    f"clears keep-out(s) {which} -- "
                    + ("which refuse it JOINTLY: no single one of them "
                       "leaves it a pose, and dropping all of them does"
                       if joint else
                       "which is what refuses it: lifting it would leave a "
                       "pose"
                       if len(names) == 1 else
                       "which together leave it none -- lifting any one of "
                       "them would give it a pose") +
                    f". No pose satisfies both, so {ref} can never LEAVE its "
                    f"zone under the optimizer's monotone intent gate, and "
                    f"cannot clear the keep-out without leaving it"),
                measured={'zone': list(z.rect), 'tolerance_mm': tol,
                          'ref': ref, 'reason': v['reason'],
                          'keepouts_freeing': list(v['keepouts_freeing']),
                          'keepouts_joint': list(v['keepouts_joint']),
                          'rotations': [round(r, 3) for r in v['rotations']],
                          'anchor_graded': zone_is_anchor(z.rect, part, tol),
                          'from_courtyard': lb.from_courtyard},
                expected={'legal_poses': '>= 1'}))
    return out


def rule_zone_containment(ctx) -> Iterator[Violation]:
    for z in ctx.intent.blocks:
        if z.rect is None:
            continue
        tol = ctx.intent.zone_tolerance(z)
        for ref in ctx.blocks.get(z.name, ()):
            part = ctx.parts.get(ref)
            if part is None:
                continue
            if not zone_fits_courtyard(z.rect, part.rect, tol):
                # Spec-coordinate zone: grade the part's CENTER against it.
                out, axis = zone_escape(z.rect, part.rect, True)
                if out > tol:
                    yield Violation(
                        rule='zone_containment',
                        severity=ctx.sev('zone_containment'), ref=ref,
                        block=z.name,
                        message=(f"{ref} sits {out:.2f}mm past the {axis} "
                                 f"edge of block {z.name!r} (zone smaller "
                                 f"than the courtyard: graded on the part "
                                 f"center)"),
                        measured={'rect': [round(v, 4) for v in part.rect],
                                  'outside_mm': round(out, 4), 'axis': axis,
                                  'anchor_graded': True},
                        expected={'zone': list(z.rect), 'tolerance_mm': tol})
                continue
            out, axis = zone_escape(z.rect, part.rect, False)
            if out > tol:
                yield Violation(
                    rule='zone_containment',
                    severity=ctx.sev('zone_containment'), ref=ref, block=z.name,
                    message=(f"{ref} courtyard extends {out:.2f}mm past the "
                             f"{axis} edge of block {z.name!r}"),
                    measured={'rect': [round(v, 4) for v in part.rect],
                              'outside_mm': round(out, 4), 'axis': axis},
                    expected={'zone': list(z.rect), 'tolerance_mm': tol})


def rule_zone_side(ctx) -> Iterator[Violation]:
    for z in ctx.intent.blocks:
        if not z.side:
            continue
        for ref in ctx.blocks.get(z.name, ()):
            part = ctx.parts.get(ref)
            if part is not None and part.side != z.side:
                yield Violation(
                    rule='zone_side', severity=ctx.sev('zone_side'),
                    ref=ref, block=z.name,
                    message=(f"{ref} is on side {part.side} but block "
                             f"{z.name!r} declares side {z.side}"),
                    measured={'side': part.side}, expected={'side': z.side})


def rule_zone_exclusive(ctx) -> Iterator[Violation]:
    """An exclusive zone is real estate reserved for its block, so a stranger
    inside it is the finding. This is what makes "keep this area clear for the
    RF section" checkable rather than aspirational."""
    for z in ctx.intent.blocks:
        if z.rect is None or not z.exclusive:
            continue
        members = set(ctx.blocks.get(z.name, ()))
        for ref, part in sorted(ctx.parts.items()):
            if ref in members:
                continue
            if z.side and part.side != z.side:
                continue
            area = legality.rect_overlap_area(part.rect, z.rect)
            if area > legality.EPS:
                yield Violation(
                    rule='zone_exclusive', severity=ctx.sev('zone_exclusive'),
                    ref=ref, block=z.name,
                    message=(f"{ref} (not a member of {z.name!r}) intrudes "
                             f"{area:.2f}mm2 into its exclusive zone"),
                    measured={'overlap_area_mm2': round(area, 4),
                              'owner': ctx.owner.get(ref)},
                    expected={'zone': list(z.rect), 'overlap_area_mm2': 0.0})


def rule_keepout(ctx) -> Iterator[Violation]:
    # PART-outer, so `keepouts_for_ref` is called once per part over the whole
    # list -- which is the resolution its own docstring describes, and the
    # same shape `QuenchState` uses to build `keepouts_for`. Keep-out-outer
    # with a 1-tuple worked, but rebuilt two sets per (keep-out, part) pair.
    # Violation order is not affected: `grade` sorts on `Violation.sort_key`.
    for ref, part in sorted(ctx.parts.items()):
        # `allow` and the side filter, from the SHARED resolver: the grader
        # and the seat predicate must agree on WHICH keep-outs bind a ref, not
        # merely on the geometry once they do. A through-hole part occupies
        # BOTH faces -- its leads pass through the keep-out even when its body
        # sits on the other side -- which is why `sides` is `part.sides` and
        # the hit test is given both rects.
        for k in keepouts_for_ref(ctx.intent.keepouts, ref, part.sides):
            if keepout_hit(k, (part.rect, part.tht_rect)):
                name = k['name']
                shape = (_fmt_rect(k['rect']) if k.get('rect') is not None
                         else f"circle {k['circle']}")
                yield Violation(
                    rule='keepout', severity=ctx.sev('keepout'), ref=ref,
                    message=(f"{ref} ({part.side}) is inside keep-out "
                             f"{name!r} {shape}"),
                    measured={'keepout': name, 'side': part.side,
                              'sides_occupied': sorted(part.sides)},
                    expected={'allow': list(k.get('allow') or ())})


def rule_edge_connector(ctx) -> Iterator[Violation]:
    """A connector that must reach the board edge: a card edge, a USB shell, a
    HAT header. This is the one class of part whose courtyard leaving the
    outline is CORRECT, so declaring it is also what stops `oob_count` from
    reporting it as a defect forever."""
    for c in ctx.intent.edge_connectors:
        ref = c['ref']
        part = ctx.parts.get(ref)
        if part is None:
            yield Violation(
                rule='edge_connector', severity=ctx.sev('edge_connector'),
                ref=ref, message=f"edge connector {ref} is not on this board",
                measured={'found': False})
            continue
        amount = ctx.gate.rect_outside_amount(part.rect)
        lim = c.get('overhang_mm') or {}
        lo = float(lim.get('min', 0.0))
        hi = lim.get('max')
        if amount < lo - legality.EPS:
            yield Violation(
                rule='edge_connector', severity=ctx.sev('edge_connector'),
                ref=ref, message=(f"{ref} overhangs the outline by "
                                  f"{amount:.2f}mm, under the declared minimum "
                                  f"{lo:.2f}mm"),
                measured={'overhang_mm': round(amount, 4)},
                expected={'min': lo, 'max': hi})
        elif hi is not None and amount > float(hi) + legality.EPS:
            yield Violation(
                rule='edge_connector', severity=ctx.sev('edge_connector'),
                ref=ref, message=(f"{ref} overhangs the outline by "
                                  f"{amount:.2f}mm, past the declared maximum "
                                  f"{float(hi):.2f}mm"),
                measured={'overhang_mm': round(amount, 4)},
                expected={'min': lo, 'max': float(hi)})
        edge = c.get('edge')
        if edge and ctx.outline_bounds:
            actual = _nearest_edge(part.rect, ctx.outline_bounds)
            if actual != edge:
                yield Violation(
                    rule='edge_connector', severity=ctx.sev('edge_connector'),
                    ref=ref, message=(f"{ref} sits nearest the {actual} edge "
                                      f"but is declared on the {edge} edge"),
                    measured={'edge': actual}, expected={'edge': edge})
        # Run-4 A: the missing PROXIMITY conjunct. The rule used to grade only
        # the overhang band and the nearest-edge identity, so a receptacle
        # 15.8 mm INTERIOR passed ("nearest west, declared west" is satisfied
        # anywhere on the board). For entries carrying the edge_receptacle
        # class (or an explicit max_setback_mm), no overhang AND off-seat is
        # a violation -- which is also what makes a misplaced edge part
        # CHARGEABLE by place_seed --repair.
        setback = c.get('max_setback_mm')
        _sev = ctx.sev('edge_connector')
        if setback is None and c.get('class') == 'edge_receptacle':
            from .part_class import SEAT_TOL_MM
            setback = SEAT_TOL_MM
        if setback is None and c.get('class') == 'connector_affinity':
            # run-23: the weak class. An INTERIOR generic connector is a flag
            # for the boundary review, never an error -- legitimately-interior
            # connectors exist (tigard J7), so this fires at WARN whatever the
            # rule's configured severity. An author upgrades by writing
            # max_setback_mm (then the configured severity applies) or edge.
            from .part_class import INTERIOR_AFFINITY_MM
            setback = INTERIOR_AFFINITY_MM
            _sev = WARN
        if setback is not None and amount <= legality.EPS:
            clr = ctx.gate.edge_clearance(part.rect)
            if clr > float(setback) + legality.EPS:
                yield Violation(
                    rule='edge_connector', severity=_sev,
                    ref=ref,
                    message=(f"{ref} is an edge part seated {clr:.2f}mm from "
                             f"the nearest edge with no overhang (seat "
                             f"tolerance {float(setback):.2f}mm) -- "
                             + ("a plug may not reach it; disposition in the "
                                "boundary review or declare max_setback_mm"
                                if _sev == WARN else
                                "the mating face cannot reach the edge")),
                    measured={'edge_clearance_mm': round(clr, 4)},
                    expected={'max_setback_mm': float(setback)})


def rule_decap_distance(ctx) -> Iterator[Violation]:
    """Decoupling caps within reach of the IC they decouple.

    The tether is `groups.decap_tethers` -- the same nearest-IC-sharing-a-net
    rule the placement grouper uses, with the distance it already measures. A
    second implementation here would grade the reimplementation.
    """
    spec = ctx.intent.decaps or {}
    limit = spec.get('max_distance_mm')
    if limit is None:
        return
    limit = float(limit)
    exempt = tuple(spec.get('exempt') or ())
    radius = float(spec.get('search_radius_mm', groups_mod.DECAP_RADIUS_MM))
    tethers = groups_mod.decap_tethers(ctx.pcb, radius=radius)
    for ic in sorted(tethers):
        for cap, dist in tethers[ic]:
            if any(fnmatch.fnmatch(cap, pat) for pat in exempt):
                continue
            if dist > limit + legality.EPS:
                yield Violation(
                    rule='decap_distance', severity=ctx.sev('decap_distance'),
                    ref=cap, block=ctx.owner.get(cap),
                    message=(f"{cap} is {dist:.2f}mm from {ic}, the IC it "
                             f"decouples (limit {limit:.2f}mm)"),
                    measured={'distance_mm': round(dist, 4), 'ic': ic},
                    expected={'max_distance_mm': limit})


def rule_must_lock(ctx) -> Iterator[Violation]:
    """Parts the intent says must not move, that the FILE does not pin.

    Unlocked is not a violation of physics, it is a violation of the plan: a
    mounting hole with no net has no airwire at all, so nothing but the halo
    term decides where the optimizer slides it.
    """
    refs = sorted(ctx.pcb.footprints)
    for pattern in ctx.intent.must_lock:
        matched = fnmatch.filter(refs, pattern)
        if not matched:
            yield Violation(
                rule='must_lock', severity=ctx.sev('must_lock'),
                message=f"must_lock pattern {pattern!r} matched no footprint",
                measured={'pattern': pattern, 'matched': 0})
            continue
        for ref in matched:
            if ref not in ctx.locked:
                yield Violation(
                    rule='must_lock', severity=ctx.sev('must_lock'), ref=ref,
                    message=(f"{ref} is declared must_lock but is not locked "
                             f"in the board file"),
                    measured={'locked': False, 'pattern': pattern},
                    expected={'locked': True})


def rule_legality(ctx) -> Iterator[Violation]:
    """Courtyard overlap and off-board parts, against a declared budget.

    `oob_area` is deliberately NOT gateable. `out_of_board_area` measures
    against the rectangular usable inset only -- its own docstring calls it "a
    lower bound on a notched one" -- so a part sitting ENTIRELY inside a cutout
    scores count=1, amount>0, area=0.0. Gating on area would grade a part in a
    slot as clean. Count and amount both see the real rings.
    """
    budget = ctx.intent.legality_budget or {}
    for key, label in (('overlap_area', 'courtyard overlap area (mm2)'),
                       ('oob_count', 'parts leaving the board outline'),
                       ('oob_amount', 'total off-board overhang (mm)')):
        if key not in budget:
            # Not graded. When the emitter WITHHELD it (a blocking body pair
            # or an unwaived courtyard interpenetration on the board it was
            # emitted from), the run must not look like a pass on this
            # channel: `budget_abstained` on the result carries it, and both
            # the report and the summary print it. See grade().
            continue
        got = ctx.legality.get(key)
        if got is None:
            continue
        lim = float(budget[key])
        if got > lim + legality.EPS:
            yield Violation(
                rule='legality', severity=ctx.sev('legality'),
                message=(f"{label}: {got:.3f} exceeds the declared budget "
                         f"{lim:.3f}"),
                measured={key: round(float(got), 4)}, expected={key: lim})


RULES = (
    ('envelope', rule_envelope),
    ('zone_containment', rule_zone_containment),
    ('zone_side', rule_zone_side),
    ('zone_exclusive', rule_zone_exclusive),
    ('keepout', rule_keepout),
    ('edge_connector', rule_edge_connector),
    ('decap_distance', rule_decap_distance),
    ('must_lock', rule_must_lock),
    ('legality', rule_legality),
)

#: Rules whose violations are raised OUTSIDE the `RULES` loop, and so have no
#: rule function to be enumerated from: the two self-contradiction findings in
#: `validate_intent`, the unresolved-block finding in `resolve_blocks`, and the
#: zone-inside-a-keep-out contradiction `resolve_intent_gate` raises (#702).
#: Kept as a named set rather than folded into `_SEVERITY_KEYS` by hand, so the
#: next reader can see which names are the exception and why.
_NON_RULE_SEVERITIES = frozenset({
    'intent_zone_outside_envelope', 'intent_zone_overlap', 'block_unresolved',
    'intent_zone_in_keepout', 'keepout_allow_unresolved'})

#: Every rule name an intent may set a severity for. Derived from `RULES`, so a
#: new rule is settable the moment it is registered -- a hand-listed set would
#: silently refuse the newest rule's own name.
_SEVERITY_KEYS = frozenset(name for name, _ in RULES) | _NON_RULE_SEVERITIES

# Why a rule did not run. Reported so that "0 violations" and "0 rules ran"
# cannot look the same to a reader or to a machine.
_SKIP_REASON = {
    'envelope': 'the intent declares no envelope.rect',
    'zone_containment': 'no block declares a zone',
    'zone_side': 'no block declares a side',
    'zone_exclusive': 'no block is marked exclusive',
    'keepout': 'the intent declares no keepouts',
    'edge_connector': 'the intent declares no edge_connectors',
    'decap_distance': 'the intent declares no decaps.max_distance_mm',
    'must_lock': 'the intent declares no must_lock patterns',
    'legality': 'the intent declares no legality_budget',
}


#: A key `emit_intent` may WITHHOLD -> (the rule that key disarms, a predicate
#: for "the author declared it by hand anyway"). One table, because the old
#: code hard-wired both halves to `legality`: it computed the abstention as
#: "in budget_withheld and not in legality_budget", and attached the note only
#: when `name == 'legality'`. Neither generalises, and #704 adds a withheld key
#: that is not a budget at all.
#:
#: A key that is NOT in this table still abstains -- it is reported as not
#: derivable and blamed on no rule -- rather than being silently dropped. A
#: typo'd withholding note is then visible, which is the whole point of the
#: channel.
_WITHHELD_RULE = {
    'overlap_area': ('legality',
                     lambda i: 'overlap_area' in (i.legality_budget or {})),
    'oob_count': ('legality',
                  lambda i: 'oob_count' in (i.legality_budget or {})),
    'oob_amount': ('legality',
                   lambda i: 'oob_amount' in (i.legality_budget or {})),
    'decaps.max_distance_mm': (
        'decap_distance',
        lambda i: (i.decaps or {}).get('max_distance_mm') is not None),
}


def _declared_by_hand(intent: Intent, key: str) -> bool:
    """Did the author declare a withheld key anyway? Then it is graded, and
    the withholding note is only history."""
    entry = _WITHHELD_RULE.get(key)
    return bool(entry and entry[1](intent))


def _wants(intent: Intent, rule: str) -> bool:
    if rule == 'envelope':
        return intent.envelope.get('rect') is not None
    if rule == 'zone_containment':
        return any(z.rect is not None for z in intent.blocks)
    if rule == 'zone_side':
        return any(z.side for z in intent.blocks)
    if rule == 'zone_exclusive':
        return any(z.exclusive and z.rect is not None for z in intent.blocks)
    if rule == 'keepout':
        return bool(intent.keepouts)
    if rule == 'edge_connector':
        return bool(intent.edge_connectors)
    if rule == 'decap_distance':
        return (intent.decaps or {}).get('max_distance_mm') is not None
    if rule == 'must_lock':
        return bool(intent.must_lock)
    if rule == 'legality':
        return bool(intent.legality_budget)
    return True


# --------------------------------------------------------------------------
# grade
# --------------------------------------------------------------------------

@dataclass
class GradeResult:
    intent: Intent
    board: str
    violations: List[Violation]
    blocks: Dict[str, List[str]]
    legality: Dict[str, object]
    outline: Dict[str, object]
    state: Dict[str, object]
    health: Dict[str, object]
    rules_run: Tuple[str, ...]
    rules_skipped: Dict[str, str]
    n_footprints: int
    # Run-23: budget keys the intent could NOT derive, {key: reason}. An
    # abstention, not a pass -- the channel was never graded. Before this the
    # withheld key was a bare `continue` in rule_legality and the run printed
    # "PASS: N rules ran, no violations" with overlap unmeasured.
    budget_abstained: Dict[str, str] = field(default_factory=dict)

    @property
    def errors(self) -> List[Violation]:
        return [v for v in self.violations if v.severity == ERROR]

    @property
    def warnings(self) -> List[Violation]:
        return [v for v in self.violations if v.severity != ERROR]

    @property
    def passed(self) -> bool:
        return not self.errors


class UntrustworthyOutline(ValueError):
    """The board outline did not parse into something a grader may rely on.

    Raised rather than graded around, because the fallback is INVISIBLE: with no
    rings, `BoardOutlineGate.active` is False and every containment test quietly
    degrades to the bounding box. A grader that inherits that reports a clean
    board because it stopped checking.
    """

    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__('; '.join(self.problems))


def grade(intent: Intent, pcb_data, pcb_file: str, *,
          group_sources: Sequence[str] = (), clearance: Optional[float] = None,
          board_edge_clearance: Optional[float] = None,
          with_health: bool = False) -> GradeResult:
    """Measure a board against its declared floorplan intent."""
    from .quench import QuenchState
    from . import placement_state
    import routing_defaults as defaults

    outline = outline_state(pcb_data, pcb_file)
    if not outline['trustworthy']:
        raise UntrustworthyOutline(outline['problems'])

    qargs = dict(clearance=clearance if clearance is not None
                 else defaults.CLEARANCE,
                 board_edge_clearance=(board_edge_clearance
                                       if board_edge_clearance is not None
                                       else 0.55),
                 crossing_penalty=10.0, halo_base=0.5, halo_coef=0.25,
                 halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
                 grid_step=defaults.GRID_STEP, length_weight=1.0)
    state = QuenchState(pcb_data, pcb_file, **qargs)

    try:
        from .parser import extract_locked_refs
        locked = extract_locked_refs(pcb_file) if pcb_file else set()
    except (OSError, ValueError):
        locked = set()

    blocks, block_problems = resolve_blocks(intent, pcb_data, group_sources)
    ctx = _Ctx(intent, pcb_data, pcb_file, state, blocks, locked, outline)

    violations = (list(validate_intent(intent)) + list(block_problems)
                  + list(unresolved_keepout_allows(intent, pcb_data))
                  + list(intent_zone_keepout_problems(
                      intent, blocks, pcb_data, pcb_file)))
    ran: List[str] = []
    skipped: Dict[str, str] = {}
    # Budget keys the emitter withheld and that are therefore NOT graded.
    # A key present in the budget was declared (by hand, deliberately) and
    # overrides its withholding note.
    abstained = {str(k): str(v)
                 for k, v in (intent.budget_withheld or {}).items()
                 if not _declared_by_hand(intent, str(k))}
    for name, fn in RULES:
        if not _wants(intent, name):
            reason = _SKIP_REASON.get(name, 'not requested')
            # "declares no X" is true but reads as "nobody wanted one". Say
            # that the emitter refused to DERIVE it, on whichever rule the
            # withheld key disarms -- not only on `legality` (#704).
            mine = {k: v for k, v in abstained.items()
                    if (_WITHHELD_RULE.get(k) or (None,))[0] == name}
            if mine:
                reason += ('; the emitter WITHHELD ' + ', '.join(
                    f'{k} ({v})' for k, v in sorted(mine.items())))
            skipped[name] = reason
            continue
        ran.append(name)
        violations.extend(fn(ctx))
    violations.sort(key=lambda v: v.sort_key())

    health_out: Dict[str, object] = {}
    if with_health:
        from . import routability
        # Which blocks carry a declared seat. net_affinity blames a part for
        # sitting where its BLOCK put it, so a block with no zone has nothing
        # to answer for and its members must not be reported.
        spec = dict(intent.health or {})
        spec.setdefault('zoned_blocks',
                        [z.name for z in intent.blocks if z.rect])
        exempt = spec.get('affinity_exempt_nets')
        if exempt:
            from net_queries import matches_net_filter
            spec['affinity_exempt_net_ids'] = [
                nid for nid, n in pcb_data.nets.items()
                if nid > 0 and n.name
                and matches_net_filter(n.name, list(exempt))]
        health_out = routability.health(state, pcb_data, blocks, spec)

    st = placement_state.assess_placement(pcb_data, pcb_file)
    return GradeResult(
        intent=intent, board=pcb_file, violations=violations, blocks=blocks,
        legality={k: (round(float(v), 4) if isinstance(v, float) else v)
                  for k, v in ctx.legality.items()},
        outline=outline,
        state={'unplaced': st.unplaced,
               'partially_unplaced': st.partially_unplaced,
               'has_copper': st.has_copper,
               'n_footprints': st.n_footprints,
               'distinct_positions': st.distinct_positions,
               'duplicate_fraction': round(st.duplicate_fraction, 4),
               'spread_ratio': (None if st.spread_ratio is None
                                else round(st.spread_ratio, 4)),
               'outside_fraction': (None if st.outside_fraction is None
                                    else round(st.outside_fraction, 4)),
               'stacked_refs': list(st.stacked_refs),
               # The subset `partially_unplaced` is actually decided on, and
               # the one place_seed scopes its seed from. Without it a reader
               # sees `partially_unplaced: false` beside a non-empty
               # `stacked_refs` and has no way to tell "suppressed as markers /
               # opposite sides" from a bug in the check.
               'stacked_suspect_refs': list(st.stacked_suspect_refs),
               'segments': st.segments, 'vias': st.vias},
        health=health_out,
        rules_run=tuple(ran), rules_skipped=skipped,
        budget_abstained=abstained,
        n_footprints=len(pcb_data.footprints))


# --------------------------------------------------------------------------
# emit: a starter intent, derived from the board
# --------------------------------------------------------------------------

#: Below this many tethers a `max` is a coordinate, not a limit. Chosen, not
#: tuned: with one or two samples there is no body-versus-tail to speak of, so
#: a derived limit describes one cap's position rather than a design rule.
#: Corpus effect, measured: withholds on interf_u_unrouted (1 tether) alone.
DECAP_MIN_SAMPLE = 3

#: The share of rail-sharing caps that may lie BEYOND the tether search radius
#: before a derived limit stops meaning anything. `groups.decap_tethers` drops
#: any cap further than `DECAP_RADIUS_MM` from a chip carrying its rail, so the
#: observed distribution is CENSORED by construction and a limit read off the
#: survivors can bless a board whose caps have left decoupling range entirely.
#: Measured over the tracked boards: healthy 0.04-0.12 (tigard 0.04,
#: splitflap_driver 0.08, watchy 0.08, ulx3s 0.12, glasgow_revC 0.05),
#: degenerate or mid-repair 0.42-1.00 (both run-23 tigard fixtures 0.42,
#: interf_u_unrouted 0.80, sonde_u 1.00). 0.25 sits in that gap with ~2x
#: margin on the healthy side and ~1.7x on the other.
DECAP_MAX_CENSORED = 0.25


def decap_census(pcb_data, radius: float = None) -> Dict:
    """What the board's decoupling tethers look like, and what they HIDE.

    Two passes over `groups.decap_tethers`: one at the ordinary radius, which
    is the population `rule_decap_distance` will grade, and one unbounded,
    which is the population that actually exists. The difference is the point
    of this function -- the rule cannot see a cap that has left the search
    radius, so a limit derived from the survivors is a limit that says nothing
    about the board's worst decap. Measured on splitflap_driver: max 3.4617mm
    within the radius, and a rail-sharing cap 19.30mm away that neither the
    census nor the rule counts.

    Emitted as `context.decap_census` on EVERY intent, flag or no flag, so a
    reader of the document can tell "no cap is far from its IC" from "nobody
    measured". That distinction is this module's own stated principle one
    level down from `rules_run` / `rules_skipped`.
    """
    r = float(groups_mod.DECAP_RADIUS_MM if radius is None else radius)
    near = groups_mod.decap_tethers(pcb_data, radius=r)
    dists = sorted(d for caps in near.values() for _c, d in caps)
    # The same query with the radius prune removed. NOT equivalent to
    # filtering the near result: without the prune, "nearest chip carrying the
    # rail" can pick a DIFFERENT chip, so this is a second measurement rather
    # than a superset.
    far = groups_mod.decap_tethers(pcb_data, radius=float('inf'))
    beyond = sorted((c, d) for caps in far.values() for c, d in caps
                    if d > r)
    n = len(dists)
    out = {
        'source': 'auto-tethers',
        'metric': ('cap footprint centroid to IC bounding box, clamped to 0 '
                   'inside (placement.groups.decap_tethers)'),
        'search_radius_mm': round(r, 4),
        'tethers': n,
        'ics': len(near),
        'beyond_radius': len(beyond),
        'beyond_radius_refs': [c for c, _d in beyond],   # already sorted
        'worst_beyond_mm': round(beyond[-1][1], 4) if beyond else None,
    }
    if n:
        # NOT rounded, unlike every other number here. This one is
        # LOAD-BEARING: `_decap_derivation` ceils it, and `round(v, 4)` can
        # land BELOW the true max -- measured on 3 of the 9 tracked boards
        # (splitflap_driver 3.4617228369700497 -> 3.4617, ulx3s
        # 4.714904558949195 -> 4.7149, glasgow_revC 4.786912496589008 ->
        # 4.7869). Ceiling a display value that is already low reproduces the
        # exact defect `_ceil4` exists to prevent -- a budget that fails
        # against the board it was written from -- and it does so INVISIBLY,
        # because the rounded number looks right. Rounded for the reader in
        # `max_mm_display`.
        out['max_mm'] = dists[-1]
        out['max_mm_display'] = round(dists[-1], 4)
        out['median_mm'] = round(dists[n // 2], 4)
    return out


def _decap_derivation(census: Dict) -> Tuple[Optional[float], Optional[str]]:
    """`(max_distance_mm, withheld_reason)` -- exactly one of them is None.

    The limit is `_ceil4(max)`, and the argument for `max` is a FIXED-POINT
    argument rather than a statistical one. An emitted intent is a baseline to
    tighten: emit, grade clean, re-emit, same limit. Only the max has that
    property.

      * the median is refuted by measurement -- glasgow_revC's tether median
        is 0.0000, because 58 of its 87 caps sit INSIDE their IC's bounding
        box and clamp to zero. A median limit flags 29 caps on a healthy
        human-routed board at the first emission.
      * a high percentile has no fixed point at all: it flags ~5% by
        construction, forever. Fix those, re-emit, and it flags another 5%.
        Every violation would be manufactured by the emitter rather than found
        on the board, which is what `check_floorplan`'s own --clearance help
        text warns about one channel over.

    `_ceil4` and not `round`: `round` can land BELOW the measured max, so the
    document would fail against the very board it was written from. Measured,
    that bites on 3 of the 7 tracked boards (splitflap_driver 3.46172 ->
    3.4617, ulx3s 4.71490 -> 4.7149, glasgow_revC 4.78691 -> 4.7869).

    WITHHOLDING. The issue proposing this asked for the `overlap_area` guard:
    withhold when the emitting board already violates the number. That guard
    is UNREACHABLE here -- `_ceil4(max(observed)) >= every observed` by
    construction, so it is a branch that can never execute, which is worse
    than absent because it reads like a guard. The reachable analogue is the
    CENSORING above: a board whose caps have left the search radius has a
    survivors' max that blesses it.

    An alternative was built and measured and does NOT work, recorded here so
    it is not re-proposed: withhold when `max > K * p75` (the max is a tail
    outlier over its own body). Healthy splitflap_driver scores 2.46 and
    mid-repair tigard_placed 2.22 -- the two populations overlap, so no
    threshold separates them.
    """
    n = int(census.get('tethers', 0))
    if not n:
        return None, (f"no cap is within {census['search_radius_mm']}mm of a "
                      f"chip carrying its rail, so nothing was measured"
                      + (f" ({census['beyond_radius']} rail-sharing cap(s) lie "
                         f"beyond it, worst {census['worst_beyond_mm']}mm)"
                         if census.get('beyond_radius') else ""))
    if n < DECAP_MIN_SAMPLE:
        return None, (f"only {n} tether(s) on the emitting board -- a max "
                      f"over {n} sample(s) is a coordinate, not a limit "
                      f"(DECAP_MIN_SAMPLE={DECAP_MIN_SAMPLE})")
    beyond = int(census.get('beyond_radius', 0))
    total = n + beyond
    if total and beyond / total > DECAP_MAX_CENSORED:
        return None, (
            f"{beyond} of {total} rail-sharing cap(s) lie beyond the "
            f"{census['search_radius_mm']}mm search radius (worst "
            f"{census['worst_beyond_mm']}mm): a limit derived from the "
            f"survivors would bless a board whose caps have left decoupling "
            f"range")
    return _ceil4(float(census['max_mm'])), None


def emit_intent(pcb_data, pcb_file: str, *,
                group_sources: Sequence[str] = ('kicad', 'sheet'),
                zone_pad_mm: float = 1.0,
                declare_classes: bool = False,
                derive_decaps: bool = False) -> Dict:
    """A starter intent READ OFF the board, for a human or a model to edit.

    Everything here describes what the board already is. The envelope is
    `board_bounds` verbatim -- never a rounded or convenient rectangle -- and
    the cutouts are emitted as read-only `context` so an editor can see what the
    parts have to avoid without being able to mistake it for something to
    change. Nothing in this module writes Edge.Cuts.

    The emitted intent grades CLEAN by construction. That is the point: it is a
    baseline to tighten, and the round trip (emit then grade) is what proves the
    rules are wired to real geometry rather than silently skipping.
    """
    from .quench import QuenchState
    import routing_defaults as defaults

    outline = outline_state(pcb_data, pcb_file)
    if not outline['trustworthy']:
        raise UntrustworthyOutline(outline['problems'])

    state = QuenchState(pcb_data, pcb_file, clearance=defaults.CLEARANCE,
                        board_edge_clearance=0.55, crossing_penalty=10.0,
                        halo_base=0.5, halo_coef=0.25, halo_weight=2.0,
                        edge_halo=2.0, edge_weight=2.0,
                        grid_step=defaults.GRID_STEP, length_weight=1.0)
    parts = {p.ref: p for p in state.graded_parts()}
    bounds = outline['bounds']

    derived = (groups_mod.derive_groups(pcb_data, tuple(group_sources))
               if group_sources else {})

    # Candidate zones: each block's member bbox, padded, then CLAMPED to the
    # envelope. Without the clamp a block touching the board edge gets a zone
    # that leaves the outline, which the emitted intent would then flag against
    # itself.
    cand = {}
    for key in sorted(derived):
        members = [r for r in derived[key] if r in parts]
        if len(members) < 2:
            continue
        rects = [parts[r].rect for r in members]
        cand[key] = (members, (
            max(bounds[0], min(r[0] for r in rects) - zone_pad_mm),
            max(bounds[1], min(r[1] for r in rects) - zone_pad_mm),
            min(bounds[2], max(r[2] for r in rects) + zone_pad_mm),
            min(bounds[3], max(r[3] for r in rects) + zone_pad_mm)))

    # A ZONE is a spatial claim, and most derived blocks cannot make one. A
    # schematic sheet is a FUNCTIONAL grouping: its members are scattered across
    # the board, so its bounding box swallows most of the others -- on ulx3s all
    # 10 sheet bboxes mutually overlap, up to 4508mm2. Emitting those as zones
    # produces an intent no placement could satisfy, and it would be the
    # emitter, not the board, that was wrong.
    #
    # So a zone is emitted only where it is DISJOINT from every other kept zone,
    # tightest first. Membership is still emitted for the rest -- it is what
    # zone_side, must_lock and the reader all want -- with the omission stated
    # rather than left as a silent absence.
    kept: List[Tuple[float, float, float, float]] = []
    zoned = set()
    for key in sorted(cand, key=lambda k: (legality.rect_area(cand[k][1]), k)):
        rect = cand[key][1]
        if any(legality.rect_overlap_area(rect, other) > legality.EPS
               for other in kept):
            continue
        kept.append(rect)
        zoned.add(key)

    blocks = []
    for key in sorted(cand):
        members, rect = cand[key]
        sides = {parts[r].side for r in members}
        entry = {
            'name': groups_mod.short_name(key),
            'group': groups_mod.short_name(key),
            'refs': sorted(members),
        }
        if key in zoned:
            entry['zone'] = [round(v, 3) for v in rect]
            entry['note'] = 'derived from the board; tighten or delete'
        else:
            entry['note'] = ('members are spread across the board and their '
                             'bounding box overlaps another block, so no zone '
                             'is claimed. Add one only if this really is a '
                             'contiguous area')
        if len(sides) == 1:
            entry['side'] = sides.pop()
        blocks.append(entry)

    # Parts already overhanging the outline are edge connectors by observation.
    # Recording them is what stops oob_count reporting them forever.
    #
    # Run-5 SUSPECT-AND-DERIVE: an observation entry can bless a DAMAGED pose
    # -- tigard SW1 was a displaced part whose wrong-place overhang two runs
    # rationalized as by-design because the INPUT had it overhanging. Before
    # blessing, compute a suspect bit from two board-only signals:
    #   S1  the part participates in a pad-legality conflict pair;
    #   S2  rigid pattern vectors exist on this board (fit survivors
    #       over-determine them -- healthy boards yield none, which is the
    #       no-op guarantee) AND some +/-v pose of this part sits fully
    #       on-board (its overhang has a displacement explanation).
    # Suspect -> class-only entry (class-default band, NO edge, note): the
    # derivation/exchange rungs decide, nothing is blessed. Healthy-board
    # entries are byte-identical to before. mount_hole-class parts get no
    # observation entry at all (their overhang is either damage or a
    # courtyard artifact; the pattern fit owns their positions).
    suspect_pairs: set = set()
    if state.legality_ctx is not None:
        from . import legality as _leg
        try:
            g = _leg.grade_pad_legality(pcb_data, state.clearance, worst_n=0)
            for (ra, rb, _mm) in g.get('worst', ()):
                suspect_pairs.add(ra)
                suspect_pairs.add(rb)
        except Exception:
            pass
    pattern_vectors = []
    try:
        from . import reconstruct as _rec
        _tiers = _rec.classify(state, None, 'auto')
        _props = _rec.fit_corner_insets(state, _tiers)
        pattern_vectors = _rec.rigid_vectors(state, _props)
    except Exception:
        pattern_vectors = []

    def _suspect(ref) -> Optional[str]:
        if ref in suspect_pairs:
            return 'participates in a pad-legality conflict'
        if pattern_vectors and state.legality_ctx is not None:
            pp = state.legality_ctx.parts.get(ref)
            p = state.parts.get(ref)
            if pp is not None and p is not None:
                for (vx, vy) in pattern_vectors:
                    for sx, sy in ((vx, vy), (-vx, -vy)):
                        ext = pp.extent(p.x + sx, p.y + sy, p.rot)
                        if ext is None:
                            continue
                        b = state.board
                        oob = (max(0.0, b[0] - ext[0]) + max(0.0, ext[2] - b[2])
                               + max(0.0, b[1] - ext[1]) + max(0.0, ext[3] - b[3]))
                        if oob <= legality.EPS:
                            return (f'rigid pattern vectors exist and the '
                                    f'+/-v pose ({p.x + sx:.2f},{p.y + sy:.2f}) '
                                    f'sits fully on-board')
        return None

    def _band_cap(ref: str) -> float:
        """The largest overhang that can still be READ as a declared band.

        An overhang wider than the part itself puts the part entirely off the
        outline, which no spec expresses; below that, `EDGE_BAND_SANITY_MM`
        keeps a small part's legitimate band from being called damage."""
        rect = parts[ref].rect
        try:
            extent = max(abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))
        except Exception:                                       # noqa: BLE001
            extent = 0.0
        return max(EDGE_BAND_SANITY_MM, extent)

    conns = []
    declared = set()
    for ref in sorted(parts):
        amt = state.edge_gate.rect_outside_amount(parts[ref].rect)
        if amt > legality.EPS:
            # An OBSERVED overhang above the sanity cap is not a band. Emitting
            # it as one launders the damage into the spec that is supposed to
            # gate the damage's repair -- see EDGE_BAND_SANITY_MM.
            over_cap = amt > _band_cap(ref)
            fp = (pcb_data.footprints or {}).get(ref)
            pc = None
            if fp is not None:
                from .part_class import classify_part, default_band
                pc = classify_part(fp, ref)
            if pc is not None and pc.name == 'mount_hole':
                # Run-5: never bless a mounting hole's overhang -- the
                # pattern fit owns hole positions, and an observation entry
                # here records damage as an allowance (run-4's H1 west 4.75).
                declared.add(ref)
                continue
            why = _suspect(ref)
            if why is not None:
                entry = {'ref': ref,
                         # Run-8 A3: machine-readable, not only prose. A
                         # consumer that has to grep a note for the word
                         # SUSPECT will eventually not, and an emitted intent
                         # is read by tools as often as by people. The note
                         # stays for the human.
                         'suspect': True,
                         'suspect_reason': why,
                         'note': (f'overhang observed but SUSPECT ({why}): '
                                  f'edge withheld -- the derivation/exchange '
                                  f'rungs decide (run-5 suspect-and-derive)')}
                if pc is not None and pc.name in ('edge_receptacle',
                                                  'edge_actuator'):
                    entry['class'] = pc.name
                    entry['source'] = 'auto-class'
                    entry['overhang_mm'] = default_band(pc.name, fp)
                elif over_cap:
                    entry['overhang_mm'] = {'min': 0.0,
                                            'max': round(_band_cap(ref), 3)}
                    entry['overhang_capped'] = True
                    entry['observed_overhang_mm'] = round(amt, 3)
                    entry['note'] += (
                        f'; observed overhang {amt:.3f}mm exceeds any '
                        f'plausible band ({_band_cap(ref):.3f}mm) and is '
                        f'treated as DAMAGE, not an allowance')
                else:
                    entry['overhang_mm'] = {'min': 0.0,
                                            'max': round(amt + 0.5, 3)}
                conns.append(entry)
                declared.add(ref)
                continue
            if over_cap:
                # NO `edge` key: the schema supports edge-less entries and both
                # the seeder's stage 1 (seeder.py:409-417) and its repair path
                # already handle them by declining to guess. The band is capped
                # rather than emitted at the observed amount, so a consumer
                # that reads only `overhang_mm` cannot be blinded either.
                entry = {'ref': ref,
                         'overhang_mm': {'min': 0.0,
                                         'max': round(_band_cap(ref), 3)},
                         'overhang_capped': True,
                         'observed_overhang_mm': round(amt, 3),
                         'note': (f'overhang observed at {amt:.3f}mm, which '
                                  f'exceeds any plausible band '
                                  f'({_band_cap(ref):.3f}mm) -- treated as '
                                  f'DAMAGE, not an allowance: no edge is '
                                  f'declared and the excess is charged. An '
                                  f'observation entry that blesses this is '
                                  f'how a 160mm displacement became a 160mm '
                                  f'spec allowance (run 10)')}
            else:
                entry = {'ref': ref, 'edge': _nearest_edge(parts[ref].rect,
                                                           bounds),
                         'overhang_mm': {'min': 0.0,
                                         'max': round(amt + 0.5, 3)}}
            if declare_classes and pc is not None \
                    and pc.name in ('edge_receptacle', 'edge_actuator'):
                entry['class'] = pc.name
                entry['source'] = 'auto-class'
            conns.append(entry)
            declared.add(ref)

    if declare_classes:
        # Run-4 A: observation-only emission reproduces the failure it exists
        # to prevent -- a MISPLACED edge part is exactly the one not
        # overhanging, so it never got declared (run 3's J1). Classify
        # pose-independently and declare edge-class parts too. `edge` is
        # written ONLY when the current pose is plausible (overhanging or
        # seated); an implausibly-posed receptacle gets a class-default band
        # and NO edge -- naming one would be an invention, and the seeder
        # deliberately skips edge-less entries.
        from .part_class import classify_part, default_band, pose_plausible
        for ref in sorted(parts):
            if ref in declared:
                continue
            fp = (pcb_data.footprints or {}).get(ref)
            if fp is None:
                continue
            pc = classify_part(fp, ref)
            if pc.name == 'connector_affinity':
                # run-23: generic connectors (headers, JST, terminal blocks)
                # had NO class, so J2/J5/J6/J7 seated mid-board and no
                # instrument could say so. Declared WITHOUT an edge (the
                # run-4 rule stands: naming one would be an invention) and
                # with no band ceiling; the grade flags an INTERIOR pose at
                # ADVISORY severity only. A human upgrades by adding `edge`
                # or `max_setback_mm` to the entry.
                clr = state.edge_gate.edge_clearance(parts[ref].rect)
                conns.append({
                    'ref': ref, 'class': pc.name, 'source': 'auto-class',
                    'overhang_mm': {'min': 0.0},
                    'note': (f'connector-family part, no edge claim; '
                             f'measured {clr:.2f}mm from the nearest edge')})
                continue
            if pc.name != 'edge_receptacle':
                # actuators make no claim unless they actually overhang
                # (handled above); nothing else is an edge class.
                continue
            clr = state.edge_gate.edge_clearance(parts[ref].rect)
            plaus = pose_plausible(pc.name, 0.0, clr)
            entry = {'ref': ref, 'class': pc.name, 'source': 'auto-class',
                     'overhang_mm': default_band(pc.name, fp)}
            if plaus:
                entry['edge'] = _nearest_edge(parts[ref].rect, bounds)
            else:
                entry['note'] = (
                    f'edge-receptacle class in an implausible pose '
                    f'({clr:.2f} mm from the nearest edge, no overhang): '
                    f'no edge declared -- reconstruct/repair must derive it')
            conns.append(entry)

    locked = sorted(extract_locked_refs_safe(pcb_file))
    leg = state.legality_metrics()
    # Run-6: a board carrying a blocking BODY pair (two footprints' pad
    # copper in the same space) must not bake its own overlap_area as the
    # budget -- that is the exact self-bless cycle run 5 shipped through
    # (the emitted 6.112 budget graded the C14-on-R14 board clean). The
    # repaired board re-emits the honest number; meanwhile board_score's
    # `assembly` component grades independently of any budget.
    # Run-23 extends the same withholding to unwaived COURTYARD interpene-
    # trations past the blocking floors: run 23's intent was emitted from a
    # mid-repair board carrying J4 0.90mm inside U6, baked overlap_area
    # 30.1085, and the final board's 26.302 then graded PASS -- the budget
    # blessed the board it was emitted from. A board with such pairs gets no
    # auto overlap budget; declare one by hand (visibly) if the overlap is
    # by design. Cost, measured: 5 of 34 corpus boards carry by-design
    # censuses and lose the auto-budget too -- the legality rule then
    # ABSTAINS (not-derivable) on them, which is honest degradation; their
    # independent coverage is check_assembly's moved-vs-baseline gate.
    try:
        from placement.legality import grade_body_overlap
        _g_overlap = grade_body_overlap(
            pcb_data, state.clearance, pcb_file=pcb_file)
        _body_blocking = _g_overlap['blocking']
        _courtyard_blocking = _g_overlap.get('courtyard_blocking', 0)
    except Exception:
        _body_blocking = 0
        _courtyard_blocking = 0
    _suspects = any('SUSPECT' in (c.get('note') or '') for c in conns)
    _budget = {}
    _withheld = {}
    if _body_blocking:
        _withheld['overlap_area'] = (f'{_body_blocking} blocking body '
                                     f'pair(s) on the emitting board (run-6)')
    elif _courtyard_blocking:
        _withheld['overlap_area'] = (
            f'{_courtyard_blocking} unwaived courtyard interpenetration(s) '
            f'past the blocking floors on the emitting board (run-23): an '
            f'auto-budget would bless them')
    else:
        _budget['overlap_area'] = _ceil4(float(leg['overlap_area']))
    if not _suspects:
        _budget['oob_count'] = int(leg['oob_count'])

    # #704. The census runs on EVERY emission: a reader of the document must
    # be able to tell "no cap is far from its IC" from "nobody measured", and
    # `decaps: {}` alone says only the second. The LIMIT is opt-in, because
    # declaring it is not a grading-only change -- see the seeder note below.
    _census = decap_census(pcb_data)
    _decaps: Dict[str, object] = {}
    if derive_decaps:
        _limit, _why = _decap_derivation(_census)
        if _limit is None:
            _withheld['decaps.max_distance_mm'] = _why
        else:
            _decaps['max_distance_mm'] = _limit
            # Repeated inside the census DELIBERATELY: a hand edit of
            # `decaps.max_distance_mm` that leaves the census behind is then
            # detectable rather than a silent lie about where the number came
            # from.
            _census['emitted_max_distance_mm'] = _limit
    # What declaring this key COSTS, measured, next to the number itself.
    # `seeder.seed_from_intent` uses its presence to pull every 2-net-bearing
    # -pad `C*` out of radial zone packing into stage 2.5, which seats one cap
    # per supply pin -- a different population from the one graded here (the
    # grouper tests "exactly two distinct net ids", the seeder "exactly two
    # net-bearing pads") and a different metric. On ulx3s that is 70 caps
    # moved to stage 2.5 against 53 graded, 17 of them never graded at all.
    _scope = {r for r, fp in (pcb_data.footprints or {}).items()
              if r[:1].upper() == 'C'
              and len([p for p in fp.pads if p.net_id > 0]) == 2}
    _census['seeder_scope'] = len(_scope)
    _census['seeder_scope_ungraded'] = len(
        _scope - {c for caps in
                  groups_mod.decap_tethers(pcb_data).values()
                  for c, _d in caps})
    return {
        'schema': SCHEMA_VERSION,
        'kind': KIND,
        'board': os.path.basename(pcb_file),
        'units': 'mm',
        'envelope': {'rect': [round(v, 4) for v in bounds],
                     'tolerance_mm': DEFAULT_ENVELOPE_TOLERANCE_MM},
        'defaults': {'zone_tolerance_mm': DEFAULT_ZONE_TOLERANCE_MM},
        'blocks': blocks,
        # A keep-out is a MECHANICAL fact -- an enclosure rib, a standoff, a
        # battery, a display window, an antenna clearance -- and none of those
        # can be read off a board, so this stays empty and the emitter says so
        # in `context.keepouts_note` rather than leaving the reader unable to
        # tell "none declared" from "not considered" (#704). Since #701 a
        # declared keep-out is ENFORCED by the seat search, not merely graded.
        'keepouts': [],
        'edge_connectors': conns,
        'decaps': _decaps,
        # must_lock is a REQUIREMENT ("these refs must end up locked"), and an
        # emitted intent describes a board rather than making demands of it.
        # Filling it with the board's own locked set (as this did) closed a
        # loop with the tools that read must_lock: place_seed --repair treated
        # a must_lock ref as seeder-owned and lifted its lock, so
        # "--emit-intent then --repair" resolved to "unlock exactly the parts
        # the user locked, and move them" -- measured on two run-7 boards.
        # The observation is still worth recording; it belongs in `context`,
        # which nothing acts on.
        'must_lock': [],
        # Budget values are rounded UP, not to nearest: round() can land up
        # to 5e-5 BELOW the measured value, 50x legality.EPS, so a budget
        # written from a board would fail against that same board (watchy:
        # overlap 9.09724 was written as 9.0972 and instantly violated).
        # Withholding rules (see _budget above): SUSPECT overhangs freeze
        # the oob census (run-5); a blocking body pair freezes overlap_area
        # (run-6). Healthy boards bake both, as before.
        'legality_budget': _budget,
        'context': {
            'note': ('read-only, describing the board as it is. The outline is '
                     'not editable by this toolchain: size, cutouts and slots '
                     'are mechanical decisions the user owns'),
            # Budget keys deliberately NOT baked, and why -- so a reader of
            # the intent can tell "withheld" from "forgot" (empty when
            # nothing was withheld).
            'budget_withheld': _withheld,
            # #704: what the tethers look like, and what the RULE cannot see.
            # Written on every emission, with or without the limit.
            'decap_census': _census,
            'keepouts_note': (
                'empty because a keep-out is a mechanical fact -- an '
                'enclosure rib, a standoff, a battery, a display window, an '
                'antenna clearance -- and none of those can be read off a '
                'board. "[]" here means NONE DECLARED, never "not '
                'considered". Declare them by hand; since #701 the seat '
                'search honours them, not only the grade.'),
            'cutouts': [[[round(x, 3), round(y, 3)] for x, y in ring]
                        for ring in (pcb_data.board_info.board_cutouts or [])],
            'edge_contours': len(
                getattr(pcb_data.board_info, 'board_edge_contours', None) or []),
            # What the board already declares locked. An OBSERVATION: the
            # placement tools read the file's own (locked yes) stamps and will
            # not move these regardless of what any intent says. Promote a ref
            # to `must_lock` by hand if you want the lock GRADED as a
            # requirement.
            'file_locked': locked,
        },
    }


def extract_locked_refs_safe(pcb_file: str):
    try:
        from .parser import extract_locked_refs
        return extract_locked_refs(pcb_file) if pcb_file else set()
    except (OSError, ValueError):
        return set()


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------

def format_text(r: GradeResult) -> str:
    lines = []
    board = os.path.basename(r.board) or '<board>'
    lines.append(f"Floorplan: {board} vs {os.path.basename(r.intent.source_path) or 'intent'}")
    lines.append(f"  {r.n_footprints} footprints, {len(r.blocks)} block(s), "
                 f"{sum(len(v) for v in r.blocks.values())} part(s) covered")
    ol = r.outline
    lines.append(f"  outline: {ol['outlines']} ring(s), {ol['cutouts']} cutout(s), "
                 f"{ol['edge_contours']} milled contour(s)")
    if not r.violations:
        lines.append(f"  PASS: {len(r.rules_run)} rule(s) ran, no violations")
    else:
        lines.append(f"  {len(r.errors)} error(s), {len(r.warnings)} warning(s) "
                     f"from {len(r.rules_run)} rule(s)")
        for v in r.violations:
            tag = 'ERROR' if v.severity == ERROR else 'warn '
            lines.append(f"    [{tag}] {v.rule}: {v.message}")
    if r.budget_abstained:
        # "legality budget key(s)" since #704 would be a lie for a withheld
        # `decaps.max_distance_mm`, which is not a budget. The WIRE key keeps
        # its name (`budget_abstained` in to_json and summary) because
        # consumers and tests pin it and renaming it buys nothing measurable.
        lines.append(f"  {len(r.budget_abstained)} declared value(s) NOT "
                     f"DERIVABLE -- not graded, not passed:")
        for key in sorted(r.budget_abstained):
            lines.append(f"    - {key}: {r.budget_abstained[key]}")
    if r.rules_skipped:
        lines.append(f"  {len(r.rules_skipped)} rule(s) did not run:")
        for name in sorted(r.rules_skipped):
            lines.append(f"    - {name}: {r.rules_skipped[name]}")
    if r.health:
        lines.append("  routability (advisory -- this says the floorplan will "
                     "fight the router, not that it breaks the intent):")
        disp = r.health.get('block_displacement') or []
        if disp:
            lines.append(f"    block displacement, worst first "
                         f"(max {r.health.get('block_displacement_max_mm')}mm):")
            for d in disp[:5]:
                lines.append(f"      {d['block']}  {d['distance_mm']:.2f}mm  "
                             f"({d['members']} parts, {d['foreign_pads']} "
                             f"foreign pads on {d['nets']} nets)")
        esc = r.health.get('escape_lanes') or []
        short = [p for p in esc if p.get('worst_deficit')]
        if short:
            lines.append(
                f"    escape lanes ({r.health.get('escape_deficit_parts')} of "
                f"{r.health.get('escape_parts')} fine-pitch part(s) have a "
                f"face that cannot pass its own nets):")
            for p in short[:3]:
                w = next((f for f in p['faces']
                          if f['face'] == p['worst_face']), None)
                if not w:
                    continue
                lines.append(
                    f"      {p['ref']} {w['face']}: supply {w['supply']} < "
                    f"demand {w['demand']} (short {w['deficit']} lane(s) at "
                    f"{w['lane_pitch_mm']}mm pitch)")
                if w['blockers']:
                    lines.append(
                        f"        {w['blocked_mm']}mm of that face is taken by "
                        f"{', '.join(w['blockers'][:3])} -- move those, not "
                        f"the nets: ordering only chooses WHICH nets strand")
                if p.get('interior_pads'):
                    lines.append(
                        f"        {p['interior_pads']} interior pad(s) escape "
                        f"through no face at all -- a fanout question")

        phantom = set(r.health.get('bus_corridors_phantom') or ())
        for row in (r.health.get('bus_corridors') or []):
            # cut_mm before the count: a shallow diagonal does several times
            # the damage of a square crossing and both score 1.
            lines.append(f"    corridor {row['name']}: "
                         f"{row.get('cut_mm', 0.0)}mm cut by "
                         f"{row['foreign_crossings']} foreign crossing(s)"
                         + (f", worst {', '.join(row['worst_nets'][:3])}"
                            if row['worst_nets'] else ''))
            if row['name'] in phantom:
                lines.append(
                    f"      NOT A CORRIDOR: only {row.get('cover', 0.0):.0%} "
                    f"of this bus's pads sit at its ends, so the rectangle is "
                    f"an average of clusters that are not there. Every number "
                    f"above is measuring a fiction. Declare the sub-buses "
                    f"separately (ADDR and DATA leave a part on different "
                    f"faces), or drop the corridor.")
        for row in (r.health.get('convergence') or []):
            lines.append(f"    convergence in {row['corridor']}: "
                         f"{', '.join(row['classes'])}")
        aff = r.health.get('net_affinity') or []
        if aff:
            total = r.health.get('net_affinity_rows', len(aff))
            lines.append(f"    net affinity ({total} part/net pair(s); a part "
                         f"seated by its BLOCK while the net's mass is "
                         f"elsewhere):")
            for a in aff:
                pierced = (f", pierces {', '.join(a['pierced'])}"
                           if a['pierced'] else '')
                lock = ' [LOCKED]' if a['locked'] else ''
                lines.append(
                    f"      {a['ref']}{lock} in '{a['block']}' carries "
                    f"{100 * a['share']:.0f}% of {a['net']} "
                    f"({a['length_mm']:.2f}mm){pierced}; moving it onto that "
                    f"net's centroid frees {a['recoverable_mm']:.2f}mm")
                lines.append(
                    f"        try: converge.py poses {os.path.basename(r.board)}"
                    f" --ref {a['ref']} --route")
        for name in sorted(r.health.get('skipped') or {}):
            lines.append(f"    - {name} not measured: "
                         f"{r.health['skipped'][name]}")
    return '\n'.join(lines)


def to_json(r: GradeResult) -> Dict:
    return {
        'schema': SCHEMA_VERSION,
        'board': r.board,
        'intent': r.intent.source_path,
        'pass': r.passed,
        'violations': [v.to_dict() for v in r.violations],
        'blocks': {k: list(v) for k, v in sorted(r.blocks.items())},
        'legality': r.legality,
        'outline': r.outline,
        'state': r.state,
        'health': r.health,
        'rules_run': list(r.rules_run),
        'rules_skipped': r.rules_skipped,
        'budget_abstained': r.budget_abstained,
        'n_footprints': r.n_footprints,
    }


def summary(r: GradeResult) -> Dict:
    """The flat JSON_SUMMARY dict, shaped like place_optimize's."""
    by_rule: Dict[str, int] = {}
    for v in r.violations:
        by_rule[v.rule] = by_rule.get(v.rule, 0) + 1
    out = {
        'board': os.path.basename(r.board),
        'intent': os.path.basename(r.intent.source_path),
        'pass': r.passed,
        'violations': len(r.violations),
        'errors': len(r.errors),
        'warnings': len(r.warnings),
        'violations_by_rule': by_rule,
        'rules_run': len(r.rules_run),
        'rules_skipped': len(r.rules_skipped),
        # Not a violation count and not a pass: channels nothing graded.
        'budget_abstained': len(r.budget_abstained),
        'budget_abstained_keys': sorted(r.budget_abstained),
        'blocks': len(r.blocks),
        'blocks_resolved': sum(1 for v in r.blocks.values() if v),
        'parts_covered': len({ref for v in r.blocks.values() for ref in v}),
        'parts_total': r.n_footprints,
        'cutouts': r.outline['cutouts'],
        'edge_contours': r.outline['edge_contours'],
    }
    out.update(r.legality)
    for k in ('unplaced', 'partially_unplaced', 'has_copper',
              'duplicate_fraction', 'spread_ratio', 'outside_fraction'):
        out[f"state_{k}"] = r.state[k]
    if r.health:
        # Advisory, and namespaced so a caller cannot mistake one of these for
        # a violation count.
        for key, out_key in (('block_displacement_max_mm',
                              'health_block_displacement_max_mm'),
                             ('blocks_displaced', 'health_blocks_displaced'),
                             ('bus_foreign_crossings',
                              'health_bus_foreign_crossings'),
                             ('net_affinity_offenders',
                              'health_net_affinity_offenders'),
                             ('net_affinity_worst_norm',
                              'health_net_affinity_worst_norm'),
                             ('bus_cut_mm', 'health_bus_cut_mm'),
                             ('escape_deficit_parts',
                              'health_escape_deficit_parts'),
                             ('escape_worst_deficit',
                              'health_escape_worst_deficit')):
            if key in r.health:
                out[out_key] = r.health[key]
        out['health_signals_skipped'] = len(r.health.get('skipped') or {})
    return out
