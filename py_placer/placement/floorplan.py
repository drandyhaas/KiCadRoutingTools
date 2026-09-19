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
import dataclasses
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
#:
#: 2 (#712): `edge_connectors[].center_on_edge` and `.along_edge_band` -- WHERE
#: ALONG the edge a connector sits. Both are declarable and both change a
#: verdict, so the rule above mandates the bump. The unknown-key refusal
#: already protects an older build (it raises `unknown key(s) center_on_edge`
#: and refuses the file), so the bump is not needed for SAFETY; it is needed
#: because READER_VERSION is the number an author copies into `min_reader`,
#: and at 1 a brief-compiled intent would claim a reader-1 build acts on a
#: claim it has never heard of. That is a false statement in the one field
#: whose only job is to be true.
#:
#: 3 (#837): `assembly.sides` -- which faces the fab will populate. Declarable,
#: and it changes two verdicts rather than one: `assembly_side` violations, and
#: `options.grow_board`'s utilisation, which credits the back face's area to
#: every board and had no way to be told the board is built on one side. The
#: same reasoning as 2 applies -- the unknown-key refusal already protects an
#: older build, and the bump is about `min_reader` being able to name the
#: version that can act on the claim.
#:
#: 4 (#902): `proximity[]` -- "these two named parts, this far apart". The one
#: class of constraint the netlist IMPLIES and nothing here could read: a 3mm
#: crystal loop and a 30mm one have identical connectivity, so no instrument
#: that reads the board can tell them apart. Declarable, and it changes the
#: exit code, so the rule above mandates the bump. The same reasoning as 2 and
#: 3 -- the unknown-key refusal already protects an older build (it raises
#: `unknown key(s) proximity` and refuses the file), so the bump is not for
#: SAFETY. It is because `READER_VERSION` is the number `compile_brief` copies
#: into `min_reader`, and at 3 a brief-compiled intent would claim a reader-3
#: build acts on a claim it has never heard of.
#: 5 (#893): `blocks[].rotation` and `blocks[].rotation_candidates` -- the
#: ANGLE a part is to be placed at, and the set a search may choose from. The
#: one placement decision the board file cannot hold as a decision: a
#: footprint's `(at x y rot)` records the angle it HAS, and nothing
#: distinguishes an angle somebody chose from a generator default, so
#: `seeder._try_place` has always been free to turn a part whose rotation was
#: load-bearing. Its docstring said so and told the author to lock the part
#: instead, which freezes its POSITION too. Declarable, and it changes a
#: verdict: the seeder honours it and REFUSES rather than falling back to the
#: 90-degree lattice, and the quench's gate pins the part to it instead of
#: offering the lattice -- so the rule above mandates the bump.
#: There is deliberately NO `rule_rotation` in `RULES`: a declared rotation is
#: ENFORCED (the seat search is given a one-angle ladder), so a grade rule
#: would be checking an invariant the search cannot violate. An earlier draft
#: of this comment claimed such a rule existed; it never did, and a
#: justification naming a grader nobody wrote is worse than a shorter one. Contrast `blocks[].side`, which is declarable and
#: whose rule docs/floorplan-intent.md calls "vacuous, not conservative"
#: because no search move carries a side: rotation is carried by every nudge.
#: 6 (#959, #1000): `edge_connectors[].side` -- the face a connector is on,
#: compiled from the brief's `user_top_side` with a user-facing or
#: perpendicular-cable connector. Declarable and graded (`edge_connector_side`,
#: an advisory WARN), so the rule above mandates the bump -- and an older
#: build refuses the key by name, which is the safety the bump does not add.
READER_VERSION = 6

_TOP_LEVEL_KEYS = {
    'schema', 'kind', 'board', 'units', 'envelope', 'defaults', 'blocks',
    'keepouts', 'edge_connectors', 'decaps', 'must_lock', 'legality_budget',
    'health', 'severity', 'context', 'overlap_waivers', 'min_reader',
    'assembly', 'proximity', 'dispositions',
}
#: #959 (#997). WRITTEN answers to "why is this not graded", one map per kind
#: of question: a rule the plan leaves dark, a budget key the emitter withheld,
#: a pad-less block the seeder cannot place, a contradiction between two
#: declared sources. Every value is a non-empty `why`. No READER_VERSION bump:
#: the rule above says not to bump for a field nothing grades, and a
#: disposition changes no verdict -- it answers P1's refusal, which reads it.
#: An older build still refuses the file (unknown top-level key), which is the
#: safe direction.
_DISPOSITION_KEYS = {'rules', 'withheld', 'refs', 'contradictions'}
_BLOCK_KEYS = {'name', 'group', 'refs', 'zone', 'side', 'exclusive',
               'tolerance_mm', 'note', 'context',
               # #893. `rotation` is a DECISION (honoured exactly, and the
               # seeder refuses rather than silently turning the part);
               # `rotation_candidates` is a SET a search may choose from.
               # Declaring both on one block is refused -- see `_rotation`.
               'rotation', 'rotation_candidates'}
#: #837. The board-level assembly policy: which faces the fab will populate.
#: `blocks[].side` is a claim about ONE subsystem; this is a claim about the
#: whole board, and it is the thing `options.grow_board` needed and could not
#: be told -- it credits the back face's area to every board, so a placement
#: can be reported as fitting on area the fab will never populate.
_ASSEMBLY_KEYS = {'sides', 'why', 'context'}
#: Named faces, not "single". `"sides": "single"` is under-specified, and the
#: corpus says so: ulx3s is BACK-dominant (163 of its 226 pad-bearing parts),
#: so a rule reading "single implies F.Cu" would flag most of a shipping
#: board. 'both' is the observed default and grades nothing.
_ASSEMBLY_SIDES = ('F', 'B', 'both')

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
#: `center_on_edge` / `along_edge_band` are #712: the along-edge half of an
#: edge claim. Absent by default and NEVER written by `emit_intent`, so every
#: intent that existed before them grades identically.
_EDGE_CONNECTOR_KEYS = {'ref', 'edge', 'overhang_mm', 'max_setback_mm',
                        'class', 'source', 'note', 'suspect', 'suspect_reason',
                        'overhang_capped', 'observed_overhang_mm', 'context',
                        'center_on_edge', 'along_edge_band', 'side'}
#: #959 (#1000): mount modes where a connector stands OFF a face, so its mating
#: face points away from the board and reaching the edge is not what makes it
#: usable. The edge-receptacle seat does not apply to one: measured on the
#: as-built boards, it false-failed 8 vertical headers (esp_prog CON2 0.69 mm,
#: tigard J2-J6 2.4-3.0 mm, glasgow J2/J3/J5).
VERTICAL_MOUNTS = ('top_mount', 'bottom_mount')
_OVERHANG_KEYS = {'min', 'max'}
#: #712. `tolerance_mm` is REQUIRED and has no default: "centred within what?"
#: is the whole claim, and a defaulted tolerance is a threshold this tool
#: chose. Measured on the tracked corpus, tigard's three connectors sit at
#: +16.1 / -25.4 / -28.7 percent off their edge centres, so any default would
#: fail a good human board 3 times out of 3.
_CENTER_ON_EDGE_KEYS = {'tolerance_mm'}
#: Fractions of the edge's own span, so a declaration survives an outline
#: resize. `from < to`, both in [0, 1].
_ALONG_EDGE_BAND_KEYS = {'from', 'to'}
_DECAP_KEYS = {'max_distance_mm', 'exempt', 'search_radius_mm',
               'max_pin_distance_mm', 'pin_functions', 'same_side'}
#: #902. One declared claim: these two named parts, no further apart than
#: `max_mm`. `ref` is always a SINGLE ref here -- the brief's list form is
#: sugar that `compile_brief` expands, so the intent carries one row per claim
#: and `Violation.ref` always has a ref to name.
#:
#: No `why` / `requirement`: prose lives in `context`, which is this schema's
#: own rule, and the brief compiler moves them there exactly as it does for
#: `interfaces[]`.
_PROXIMITY_KEYS = {'ref', 'near', 'max_mm', 'basis', 'pads', 'note', 'source',
                   'context'}
#: Which geometry the gap is measured between.
#:
#: `pad_edge` is pad copper to pad copper -- the currency `decap_pin_distance`
#: already uses, so a reader meets one definition of "how far apart" rather
#: than two. `body` measures #896's drawn bodies, and it exists because two
#: parts a brief wants "together" may share no net at all: an auto-reset
#: transistor pair has no pad pair to measure.
#:
#: `courtyard` is deliberately NOT a spelling, and `design_brief` refuses it by
#: name with the measurement that decided it -- `placement.body` is a ladder,
#: and on the board this rule was written for 0 of 21 footprints draw a
#: courtyard, so the word would name geometry the board does not have.
_PROXIMITY_BASES = ('pad_edge', 'body')
_PROXIMITY_DEFAULT_BASIS = 'pad_edge'
_DEFAULTS_KEYS = {'zone_tolerance_mm'}
#: `zoned_blocks` is setdefault-injected into this same dict by `grade` after
#: load, and `affinity_exempt_net_ids` is derived there from
#: `affinity_exempt_nets` -- but only when that key is present, so an author
#: may also set the ids directly. Both are accepted for that reason.
_HEALTH_KEYS = {'bus_corridors', 'classes', 'zoned_blocks',
                'affinity_exempt_nets', 'affinity_exempt_net_ids',
                'ignore_net_ids', 'max_fanout', 'block_displacement_mm',
                'plane_layers'}
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
    #: #893. The angle this block's members are to be placed at (a DECISION,
    #: honoured exactly), and the set a search may choose from. Never both.
    rotation: Optional[float] = None
    rotation_candidates: Optional[Tuple[float, ...]] = None
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
    # #837: the board-level assembly policy, `{'sides': 'F'|'B'|'both', ...}`.
    # Empty when the intent declares none, which disarms `assembly_side` with
    # the honest "nobody asked" reason rather than grading a board against a
    # policy nothing states.
    assembly: Dict[str, object] = field(default_factory=dict)
    # #902. Declared proximity claims, one row per (ref, near) pair. NEVER
    # emitted: a relation between two parts is not readable off a board -- the
    # board supplies the distance and never the claim -- so an emitter writing
    # one would be blessing the current pose as the spec, which is the
    # emit-then-grade round trip this file guards against everywhere else.
    # Defaulted, so every existing `Intent(...)` construction site is
    # untouched and an intent declaring none behaves exactly as before.
    proximity: Tuple[Dict[str, object], ...] = ()
    #: #959 (#997). `{kind: {key: why}}` for kind in `_DISPOSITION_KEYS`: the
    #: plan's written answers to the questions P1 refuses on. Read by
    #: `rule_roster` and the driver, never by a rule -- a disposition is not a
    #: verdict. Defaulted, so every existing construction site is untouched.
    dispositions: Dict[str, Dict[str, str]] = field(default_factory=dict)
    #: #959 (#1002). `context.basis`: `{intent path: basis}` -- what each
    #: number IS (`declared`, `observed_baseline`, `derived_default`,
    #: `mechanical`). Carried like `budget_withheld` so a finding can say an
    #: observed baseline is not a requirement. Read for messages only.
    basis: Dict[str, str] = field(default_factory=dict)

    def assembly_sides(self) -> str:
        """The declared policy, or 'both' -- which constrains nothing.

        'both' is the resolved default rather than None so every consumer
        (the rule, `options.grow_board`) reads one vocabulary. A board nobody
        declared is a board that may use both faces, which is exactly what the
        arithmetic did before this key existed.
        """
        return str((self.assembly or {}).get('sides') or 'both')

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


def _rotation(raw, where: str) -> Optional[float]:
    """One declared angle, normalised to [0, 360). Refuses anything else.

    Normalised because KiCad writes -90 where this tool writes 270, and an
    author copying an angle out of a board file must not get a claim that can
    never be met. `bool` is refused explicitly: it is an int subclass, so
    `rotation: true` would otherwise load as 1.0 degrees.
    """
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise IntentError(
            f"{where}: rotation {raw!r}, expected a number of degrees")
    return float(raw) % 360.0


def _rotation_candidates(raw, where: str) -> Tuple[float, ...]:
    """The declared candidate set, order PRESERVED.

    Order is load-bearing, not cosmetic: the seeder keeps the FIRST pose that
    fits, so a reordered list changes which rotation wins a tie. That is why
    this returns a tuple in the author's order and never a set -- the same
    warning `quench._candidate_rotations`' docstring carries.

    An EMPTY list is refused rather than treated as "no constraint": an author
    who writes `[]` has said something, and the something they said admits no
    pose at all.
    """
    if isinstance(raw, (str, bytes)) or not isinstance(raw, (list, tuple)):
        raise IntentError(
            f"{where}: rotation_candidates {raw!r}, expected a list of "
            f"degrees")
    if not raw:
        raise IntentError(
            f"{where}: rotation_candidates is empty. An empty candidate set "
            f"admits no pose at all; omit the key to leave the rotation free")
    out = []
    for i, v in enumerate(raw):
        out.append(_rotation(v, f"{where}[{i}]"))
    # Duplicates are refused rather than de-duplicated, for the same reason the
    # empty list is: it is almost always a typo, and silently collapsing it
    # would make the declared set differ from the graded one.
    if len(set(out)) != len(out):
        raise IntentError(
            f"{where}: rotation_candidates has repeated angles {out!r}")
    return tuple(out)


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


def _number(value, where: str, lo=None, hi=None) -> float:
    """A real number, `bool` refused. `True` is an `int` in Python, and a
    tolerance of `True` is 1.0mm nobody typed."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise IntentError(f"{where}: expected a number, got {value!r}")
    v = float(value)
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        rng = (f"[{lo}, {hi}]" if lo is not None and hi is not None
               else (f">= {lo}" if lo is not None else f"<= {hi}"))
        raise IntentError(f"{where}: expected {rng}, got {v!r}")
    return v


def _proximity_claims(raw: Dict) -> List[Dict]:
    """Validate `proximity[]` (#902), AT LOAD.

    At load and not in `validate_intent`, the deviation `_along_edge_claim`
    already documents and for the same reason: `validate_intent` has exactly
    one caller -- `grade()` -- while `place_seed` and `place_reconstruct` load
    an intent and never call it. A refusal that lived there would leave those
    holding a row whose `pads` is a number.

    Pad numbers are refused unless they are STRINGS. `Pad.pad_number` is a
    string on both parse paths, so a JSON `1` matches no pad, resolves an empty
    set, and grades CLEAN -- a refusal that reads as a pass, in the one
    direction nobody checks. `design_brief` refuses the same shape at its own
    level; this is the second reader of the same document, and a document that
    can reach the grade by a path that skips the brief (a hand-written intent)
    must not be graded on a claim that cannot resolve.
    """
    got = raw.get('proximity')
    if got is not None and not isinstance(got, list):
        raise IntentError(f"proximity: expected a list of claims, got "
                          f"{type(got).__name__}")
    out: List[Dict] = []
    seen: Dict[Tuple[str, str], int] = {}
    for i, p in enumerate(got or []):
        where = f"proximity[{i}]"
        if not isinstance(p, dict):
            raise IntentError(f"{where}: expected an object with `ref`, "
                              f"`near` and `max_mm`")
        _reject_unknown(p, _PROXIMITY_KEYS, where)
        _entry_context(p, where)
        ref, near = p.get('ref'), p.get('near')
        for name, val in (('ref', ref), ('near', near)):
            if not val or not isinstance(val, str):
                raise IntentError(
                    f"{where}: `{name}` must be a single reference. The "
                    f"brief's list form is sugar that `compile_brief` expands, "
                    f"so an intent row names exactly one part on each side")
        if ref == near:
            raise IntentError(f"{where} ({ref}): the subject and the partner "
                              f"are the same part, which is 0mm from itself")
        if (ref, near) in seen:
            raise IntentError(
                f"{where} ({ref} near {near}): duplicate claim, already "
                f"declared at proximity[{seen[(ref, near)]}] -- two claims "
                f"about one relation, with no rule for which wins, and the "
                f"grade would charge BOTH")
        seen[(ref, near)] = i
        # The REVERSED pair, when neither row names a SUBJECT pad list -- the
        # same refusal `design_brief` makes, and it has to be made here too:
        # this loader exists for the HAND-WRITTEN intent, which never passes
        # through the brief compiler. Without it the two documents disagreed
        # about the same rows, and the grade charged one symmetric measurement
        # twice, reporting an identical number under two claims.
        #
        # A subject pad list is what turns the existential minimum into "for
        # EACH of my pads", so with one on either side the two rows really are
        # different claims and both are kept.
        if not (p.get('pads') or {}).get(ref) and (near, ref) in seen:
            j = seen[(near, ref)]
            if not ((got[j].get('pads') or {}).get(near)):
                raise IntentError(
                    f"{where} ({ref} near {near}): the reverse of "
                    f"proximity[{j}], and neither row names a `pads` list for "
                    f"its own subject -- so the two are the same symmetric "
                    f"measurement declared twice, with no rule for which "
                    f"claim wins. Keep one, or name the pads that make them "
                    f"different claims")
        if 'max_mm' not in p:
            raise IntentError(f"{where} ({ref}): needs `max_mm`, the distance "
                              f"in mm these parts may be apart")
        limit = _number(p['max_mm'], f"{where} ({ref}).max_mm")
        if not math.isfinite(limit) or limit <= 0.0:
            raise IntentError(
                f"{where} ({ref}).max_mm: {p['max_mm']!r}. A limit must be a "
                f"positive finite distance -- infinity can never be exceeded "
                f"and NaN fails every comparison, so either would be a "
                f"declared claim nothing can ever violate")
        basis = p.get('basis', _PROXIMITY_DEFAULT_BASIS)
        if basis not in _PROXIMITY_BASES:
            raise IntentError(
                f"{where} ({ref}).basis: {basis!r}, expected one of "
                f"{', '.join(map(repr, _PROXIMITY_BASES))}")
        pads = p.get('pads')
        if pads is not None:
            if not isinstance(pads, dict) or not pads:
                raise IntentError(
                    f"{where} ({ref}).pads: expected a non-empty "
                    f"{{'REF': ['1', '2']}}")
            for who, nums in pads.items():
                if who not in (ref, near):
                    raise IntentError(
                        f"{where} ({ref}).pads: a pad list for {who!r}, which "
                        f"this claim does not name")
                if not isinstance(nums, (list, tuple)) or not nums:
                    raise IntentError(f"{where} ({ref}).pads.{who}: expected a "
                                      f"non-empty list of pad numbers")
                for n in nums:
                    if not isinstance(n, str):
                        raise IntentError(
                            f"{where} ({ref}).pads.{who}: pad {n!r} has type "
                            f"{type(n).__name__}; pad numbers are strings. "
                            f"`Pad.pad_number` is a string, so {n!r} would "
                            f"match no pad, measure nothing, and grade clean")
        out.append(dict(p))
    return out


def _along_edge_claim(c: Dict, i: int) -> None:
    """Validate `center_on_edge` / `along_edge_band` on one entry (#712).

    WHERE the part sits along its edge. The rule already grades the overhang
    band, the nearest-edge identity and a setback, so a seat well off the
    centreline of its edge is satisfied exactly as well as a centred one.

    Refused HERE, at load, rather than in `validate_intent`, and that is a
    deliberate deviation from the issue's own text. `validate_intent` has
    exactly one caller -- `grade()` -- while `place_seed` and
    `place_reconstruct` load an intent and never call it. A refusal that lived
    there would leave the SEEDER, which acts on these fields, holding an entry
    that declares two conflicting bands with no policy for which wins. The
    precedent is `legality_budget.oob_area`, refused at load for the same
    reason: a contradiction the reader can never satisfy belongs in the
    message, not in a verdict.
    """
    ref = c['ref']
    where = f"edge_connectors[{i}] ({ref})"
    centre = c.get('center_on_edge')
    band = c.get('along_edge_band')
    if centre is not None and band is not None:
        raise IntentError(
            f"{where}: declares BOTH center_on_edge and along_edge_band. "
            f"center_on_edge is sugar for a symmetric along_edge_band, so "
            f"this is two bands on one entry -- keep the one you mean")
    if centre is not None:
        if not isinstance(centre, dict):
            raise IntentError(f"{where}: center_on_edge expects "
                              f"{{'tolerance_mm': ..}}, got {centre!r}")
        _reject_unknown(centre, _CENTER_ON_EDGE_KEYS,
                        f"{where}.center_on_edge")
        if 'tolerance_mm' not in centre:
            raise IntentError(
                f"{where}.center_on_edge: needs `tolerance_mm`. There is no "
                f"default: measured on this repo's own tracked boards, "
                f"tigard's connectors sit 16-29% off their edge centres, so "
                f"any threshold this tool picked would fail a good board")
        _number(centre['tolerance_mm'], f"{where}.center_on_edge.tolerance_mm",
                lo=0.0)
    if band is not None:
        if not isinstance(band, dict):
            raise IntentError(f"{where}: along_edge_band expects "
                              f"{{'from': .., 'to': ..}}, got {band!r}")
        _reject_unknown(band, _ALONG_EDGE_BAND_KEYS, f"{where}.along_edge_band")
        missing = sorted(_ALONG_EDGE_BAND_KEYS - set(band))
        if missing:
            raise IntentError(f"{where}.along_edge_band: needs "
                              f"{', '.join('`%s`' % m for m in missing)} "
                              f"(fractions of the edge span, 0 to 1)")
        f0 = _number(band['from'], f"{where}.along_edge_band.from", 0.0, 1.0)
        f1 = _number(band['to'], f"{where}.along_edge_band.to", 0.0, 1.0)
        if not f0 < f1:
            raise IntentError(
                f"{where}.along_edge_band: from {f0} is not less than to "
                f"{f1}, so the band is empty or inverted and no pose can "
                f"satisfy it")


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
        if str(name).startswith(MECHANICAL_ANCHOR_PREFIX):
            raise IntentError(
                f"blocks[{i}]: the name {name!r} is reserved -- "
                f"`{MECHANICAL_ANCHOR_PREFIX}` blocks are compiled from "
                f"mechanical.json by the grade itself (#959). Remove it; the "
                f"grade reads the declaration directly")
        if name in seen_names:
            raise IntentError(f"blocks[{i}]: duplicate block name {name!r}")
        seen_names.add(name)
        side = b.get('side')
        if side is not None and side not in ('F', 'B'):
            raise IntentError(
                f"blocks[{i}] ({name}): side {side!r}, expected 'F' or 'B'")
        # #893. A decision and a search set are contradictory: one says
        # "this angle", the other "any of these". Refused rather than given a
        # precedence rule nobody would remember.
        if b.get('rotation') is not None and b.get('rotation_candidates') is not None:
            raise IntentError(
                f"blocks[{i}] ({name}): declares BOTH rotation and "
                f"rotation_candidates. `rotation` is a decision the seeder "
                f"honours exactly; `rotation_candidates` is a set it may "
                f"choose from. Declare one")
        rot = (None if b.get('rotation') is None
               else _rotation(b['rotation'], f"blocks[{i}].rotation"))
        rot_cands = (None if b.get('rotation_candidates') is None
                     else _rotation_candidates(
                         b['rotation_candidates'],
                         f"blocks[{i}].rotation_candidates"))
        zone_rect = b.get('zone')
        blocks.append(Zone(
            name=name,
            rect=_rect(zone_rect, f"blocks[{i}].zone") if zone_rect else None,
            side=side,
            group=b.get('group'),
            refs=_str_tuple(b.get('refs'), f"blocks[{i}].refs"),
            exclusive=bool(b.get('exclusive', False)),
            tolerance_mm=b.get('tolerance_mm'),
            rotation=rot,
            rotation_candidates=rot_cands,
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
        _along_edge_claim(c, i)
        if c.get('side') is not None and c['side'] not in ('F', 'B'):
            raise IntentError(
                f"edge_connectors[{i}] ({c['ref']}): side {c['side']!r}, "
                f"expected 'F' or 'B'")
        conns.append(c)

    proximity = _proximity_claims(raw)

    severity = _obj(raw.get('severity'), 'severity')
    if any(v not in (ERROR, WARN) for v in severity.values()):
        raise IntentError(
            f"severity: expected {{rule: 'error'|'warn'}}, got {severity!r}")
    # Keys too, not only values (#710). `{"decap_distanc": "warn"}` used to
    # load clean and leave the rule at its default -- a demotion the author
    # believes they made and the exit code never reflects.
    _reject_unknown(severity, _SEVERITY_KEYS, 'severity')
    if severity.get('edge_connector_side') == ERROR:
        raise IntentError(
            "severity: edge_connector_side is advisory by design and cannot "
            "be raised to error -- nothing in the placement stack moves a "
            "part between faces (#836), so an error would be a red mark no "
            "run could clear")

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

    assembly = _obj(raw.get('assembly'), 'assembly')
    _reject_unknown(assembly, _ASSEMBLY_KEYS, 'assembly')
    if 'sides' in assembly:
        # Refused BY REASON rather than as a bare bad enum: 'single' is the
        # spelling an author reaches for first, and the reason it is not
        # accepted -- that it does not say WHICH face -- is the whole content
        # of the correction.
        v = assembly['sides']
        if v not in _ASSEMBLY_SIDES:
            extra = (" -- name the face: a single-sided board can be built on "
                     "either one, and the corpus has back-dominant boards"
                     if isinstance(v, str) and v.lower() in
                     ('single', 'one', 'single-sided', 'one-sided') else '')
            raise IntentError(
                f"assembly.sides must be one of {list(_ASSEMBLY_SIDES)}, got "
                f"{v!r}{extra}")
    elif assembly:
        raise IntentError(
            "assembly: declares no `sides`, so it constrains nothing. Give it "
            f"one of {list(_ASSEMBLY_SIDES)} or drop the key -- an assembly "
            "block that grades nothing is the failure `block_unresolved` "
            "exists to prevent, one level down")

    decaps = _obj(raw.get('decaps'), 'decaps')
    _reject_unknown(decaps, _DECAP_KEYS, 'decaps')
    # #705's three keys get REAL checks, unlike `max_distance_mm`,
    # which is float()'d at rule time and blows up there on a string.
    # Not retrofitted onto the old key in this change: that would be a
    # separate behaviour change to an intent an author may already ship.
    if 'max_pin_distance_mm' in decaps:
        v = decaps['max_pin_distance_mm']
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise IntentError(
                f"decaps.max_pin_distance_mm must be a number, got "
                f"{type(v).__name__}")
        if v <= 0:
            # A zero limit flags every pin with a positive gap, which
            # is `decap_distance`'s own recorded vacuity trap running
            # the other way -- a rule that fires on everything reports
            # nothing.
            raise IntentError(
                f"decaps.max_pin_distance_mm must be positive, got {v}")
    if 'pin_functions' in decaps:
        decaps = dict(decaps)
        decaps['pin_functions'] = list(
            _str_tuple(decaps['pin_functions'], 'decaps.pin_functions'))
        if not decaps['pin_functions']:
            raise IntentError(
                "decaps.pin_functions is empty: it REPLACES the default "
                "keyword table, so an empty list silently disables the "
                "pinfunction channel. Omit the key to keep the default")
    if 'same_side' in decaps and not isinstance(decaps['same_side'],
                                                bool):
        raise IntentError(
            f"decaps.same_side must be true or false, got "
            f"{type(decaps['same_side']).__name__}")
    if 'search_radius_mm' in decaps:
        # Retrofitted after all, because the review found it accepts a NEGATIVE
        # radius: `{"max_distance_mm": 2.5, "search_radius_mm": -5.0}` loads,
        # and every tether on glasgow_revC then reports as "beyond the -5.00mm
        # tether search radius" -- 92 warnings whose text is nonsense. The
        # neighbouring keys got real checks and this one did not, which is the
        # kind of gap a reviewer finds and an author meets.
        v = decaps['search_radius_mm']
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise IntentError(
                f"decaps.search_radius_mm must be a number, got "
                f"{type(v).__name__}")
        if v <= 0:
            raise IntentError(
                f"decaps.search_radius_mm must be positive, got {v}")

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

    dispositions = _dispositions(raw.get('dispositions'))

    intent = Intent(
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
        assembly=dict(assembly),
        proximity=tuple(proximity),
        dispositions=dispositions,
        basis={str(k): str(v) for k, v in
               _obj(context.get('basis'), 'context.basis').items()},
    )
    # A disposition for a rule the intent ARMS says "this is not graded" about
    # a rule that is -- the two statements cannot both be true, and a reader
    # trusting the disposition would skip a live finding. Checked here, where
    # only the intent is needed. Refs need the board: `stale_dispositions`
    # reports them when handed it (`grade` and `check_floorplan --plan-only`
    # do), and the P1 driver refuses on them in its own words. Contradiction
    # ids need the brief and mechanical.json: `stale_dispositions` names them
    # when handed the reconciliation (both of those callers do), and P1
    # refuses them.
    armed = sorted(r for r in dispositions.get('rules', {})
                   if _wants(intent, r))
    if armed:
        raise IntentError(
            f"dispositions.rules: {', '.join(armed)} "
            f"{'is' if len(armed) == 1 else 'are'} ARMED by this intent, so a "
            f"disposition saying why it is not graded contradicts the intent "
            f"itself. Drop the disposition, or drop the key that arms the rule")
    return intent


def _dispositions(raw) -> Dict[str, Dict[str, str]]:
    """Parse `dispositions` (#959, #997): `{kind: {key: why}}`.

    Refused, not ignored, on anything that would make a disposition answer a
    question nobody can see: an unknown kind, a non-string or empty `why`, a
    rule name `RULES` does not have, a withheld key the emitter cannot
    withhold. An empty `why` is the one that matters most -- a disposition
    with no reason is a flag that makes the refusal go away rather than an
    answer to it, which is exactly what the driver's `--waive x:` refuses.
    """
    if raw is None:
        return {}
    d = _obj(raw, 'dispositions')
    _reject_unknown(d, _DISPOSITION_KEYS, 'dispositions')
    known = {
        'rules': {name for name, _ in RULES},
        'withheld': set(_WITHHELD_RULE),
    }
    out: Dict[str, Dict[str, str]] = {}
    for kind in sorted(d):
        m = _obj(d[kind], f'dispositions.{kind}')
        entries: Dict[str, str] = {}
        for key, why in sorted(m.items()):
            if not isinstance(why, str) or not why.strip():
                raise IntentError(
                    f"dispositions.{kind}.{key}: needs a non-empty `why`. A "
                    f"disposition is a written reason, and one with no reason "
                    f"makes a refusal go away rather than answering it")
            if kind in known and key not in known[kind]:
                raise IntentError(
                    f"dispositions.{kind}.{key}: not a "
                    f"{'rule' if kind == 'rules' else 'withholdable key'}"
                    f" -- expected one of {', '.join(sorted(known[kind]))}")
            entries[str(key)] = why.strip()
        if entries:
            out[kind] = entries
    return out


def mechanical_drift(intent: Intent, pcb_data, mechanical: Dict, *,
                     skip: Sequence[str] = ()) -> List[Violation]:
    """Parts that moved off, or turned away from, their declared mechanical
    pose (#959, #1001).

    Run 29 moved `Ref*` from its declared (141.2, 95.9) to (123.65, 93.2) and
    nothing objected at any gate: the file declaring the pose was read by
    nothing. `skip` holds refs whose mechanical value LOST a contradiction
    (the brief says otherwise, and the brief wins by default) -- grading them
    against the losing value would report the winning placement as drift.
    Pad-less refs ARE graded here: a fiducial or a logo that moved is still a
    moved mechanical fact, even though no anchor block is compiled for it.
    """
    from .reconcile import POSE_TOL_MM, ROT_TOL_DEG
    out: List[Violation] = []
    for ref, p in sorted((mechanical.get('poses') or {}).items()):
        fp = (pcb_data.footprints or {}).get(ref)
        if fp is None or ref in skip:
            continue
        dx, dy = fp.x - p['x'], fp.y - p['y']
        dist = math.hypot(dx, dy)
        drot = None
        if p.get('rot') is not None:
            d = abs((fp.rotation or 0.0) % 360.0 - p['rot']) % 360.0
            drot = min(d, 360.0 - d)
        moved = dist > POSE_TOL_MM
        turned = drot is not None and drot > ROT_TOL_DEG
        if not (moved or turned):
            continue
        what = ' and '.join(x for x in (
            f"{dist:.3f}mm from" if moved else '',
            f"turned {drot:.1f} deg from" if turned else '') if x)
        # WARN where the anchor already reports the move as an ERROR; an
        # ERROR where nothing else can see it -- a TURN (a symmetric body
        # sits inside its anchor turned 180: 68 of 97 anchored corpus refs)
        # and ANY drift of a pad-less ref, which is never anchored.
        default = ERROR if (turned or not fp.pads) else WARN
        out.append(Violation(
            rule='mechanical_drift',
            severity=intent.severity_of('mechanical_drift', default),
            ref=ref,
            message=(f"{ref} is {what} its declared mechanical pose "
                     f"({p['x']:.3f}, {p['y']:.3f}"
                     + (f", {p['rot']:.1f}" if p.get('rot') is not None
                        else '') + ") -- "
                     + (p.get('reason') or 'mechanical.json')),
            measured={'x': round(fp.x, 4), 'y': round(fp.y, 4),
                      'rotation': round((fp.rotation or 0.0) % 360.0, 4),
                      'distance_mm': round(dist, 4),
                      'rotation_off_deg': (None if drot is None
                                           else round(drot, 4))},
            expected={'x': p['x'], 'y': p['y'], 'rotation': p.get('rot')}))
    return out


#: Block names the grade compiles from `mechanical.json` (#959, #1001) and
#: a plan may therefore not use. An anchor taken from the PLAN could be left
#: out (run 29's plans had none, so its moved fiducial graded a WARN), and a
#: plan block that could call itself an anchor could exempt itself from the
#: envelope and overlap checks (the Phase-3 verifier did exactly that with
#: `context.basis: mechanical`). So the grade builds them from the file, and
#: this prefix is refused in an intent.
MECHANICAL_ANCHOR_PREFIX = 'mech:'


def mechanical_anchor_violations(pcb_data, pcb_file: str, mechanical: Dict,
                                 *, skip: Sequence[str] = (), state=None,
                                 locked=(), outline=None) -> List['Violation']:
    """`zone_containment` for every anchored mechanical ref, against an
    anchor compiled from the FILE at grade time (#959, #1001).

    The anchor is the grader's own rect at the declared pose
    (`reconcile.anchor_blocks`); `skip` holds refs whose mechanical value lost
    a contradiction. ERROR, fixed: the pose is a recorded fact, and a plan's
    `severity` map cannot demote it."""
    from . import reconcile as _rc
    anchors, _skipped = _rc.anchor_blocks(pcb_data, pcb_file, mechanical,
                                          lost=skip, state=state)
    if not anchors:
        return []
    it = intent_from_dict({'schema': 1, 'kind': 'floorplan-intent',
                           'units': 'mm', 'severity': {
                               'zone_containment': 'error'}}, '')
    it = dataclasses.replace(it, blocks=[_anchor_zone(b) for b in anchors])
    blocks = {z.name: [z.name[len(MECHANICAL_ANCHOR_PREFIX):]]
              for z in it.blocks}
    if state is None:
        import pose_score
        state = pose_score.make_state(pcb_data, pcb_file)
    if outline is None:
        outline = outline_state(pcb_data, pcb_file)
    ctx = _Ctx(it, pcb_data, pcb_file, state, blocks, set(locked), outline)
    return [dataclasses.replace(
        v, message=(f"{v.message} -- the anchor is {v.ref}'s declared pose "
                    f"in {mechanical.get('path')}"))
        for v in rule_zone_containment(ctx)]


def _anchor_zone(b: Dict) -> 'Zone':
    """One compiled anchor dict as a `Zone`, bypassing the reserved-name
    refusal that keeps plans from declaring one."""
    doc = dict(b)
    name = doc.pop('name')
    z = intent_from_dict({'schema': 1, 'kind': 'floorplan-intent',
                          'units': 'mm',
                          'blocks': [dict(doc, name='anchor')]}, '').blocks[0]
    return dataclasses.replace(z, name=name)


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

    # Two zones overlapping on a shared side. NOT unsatisfiable, whatever
    # the old message said (#959): a member of either zone may sit in the
    # shared area, and measured, run 29's own lap-5 board satisfied both of its
    # "no placement can satisfy both" pairs (zone_containment clean). What IS
    # unsatisfiable -- a member that cannot fit in its own zone clear of a
    # stranger's EXCLUSIVE zone -- needs the board, and is
    # `plan_zone_exclusive_unsatisfiable`. So this reports the overlap, at WARN
    # by default, and says how to make it an exclusion if one was meant.
    for i, a in enumerate(intent.blocks):
        for b in intent.blocks[i + 1:]:
            if a.rect is None or b.rect is None:
                continue
            if a.side and b.side and a.side != b.side:
                continue
            area = legality.rect_overlap_area(a.rect, b.rect)
            if area > legality.EPS:
                ex = [z.name for z in (a, b) if z.exclusive]
                out.append(Violation(
                    rule='intent_zone_overlap',
                    severity=intent.severity_of('intent_zone_overlap', WARN),
                    block=a.name,
                    message=(f"zones {a.name!r} and {b.name!r} overlap by "
                             f"{area:.2f}mm2 on the same side; "
                             + (f"the shared area belongs to the EXCLUSIVE "
                                f"zone {ex[0]!r}, so no member of the other "
                                f"may sit in it -- "
                                f"`plan_zone_exclusive_unsatisfiable` says "
                                f"whether each still has room"
                                if len(ex) == 1 else
                                "both are EXCLUSIVE, so the shared area is "
                                "closed to the members of each"
                                if ex else
                                "a member of either may be placed there -- "
                                "declare `exclusive` on one of them if the "
                                "area is not meant to be shared")),
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


def rotations_for_ref(intent: Intent, blocks: Dict[str, List[str]]
                      ) -> Dict[str, Tuple[Optional[float],
                                           Optional[Tuple[float, ...]]]]:
    """{ref: (declared rotation, declared candidates)} over ALREADY-RESOLVED blocks.

    Iterates `intent.blocks`, not `zone_entries`: that one yields only blocks
    carrying a `rect`, and a block may declare a rotation with no zone at all
    ("U1 faces the USB socket" is not a coordinate claim).

    A ref in two blocks that declare DIFFERENT angles is a contradiction the
    author has to resolve, so it raises rather than picking one. Two blocks
    declaring the SAME angle is not a contradiction and is allowed -- globs
    overlap legitimately (`U*` and `U1`).
    """
    out: Dict[str, Tuple[Optional[float], Optional[Tuple[float, ...]]]] = {}
    owner: Dict[str, str] = {}
    for z in intent.blocks:
        if z.rotation is None and z.rotation_candidates is None:
            continue
        claim = (z.rotation, z.rotation_candidates)
        for ref in blocks.get(z.name, ()):
            prev = out.get(ref)
            if prev is not None and prev != claim:
                raise IntentError(
                    f"{ref} is claimed by blocks {owner[ref]!r} and {z.name!r} "
                    f"with different rotations ({prev} vs {claim}). A part has "
                    f"one angle; resolve the overlap")
            out[ref] = claim
            owner[ref] = z.name
    return out


def zone_entries(intent: Intent, blocks: Dict[str, List[str]]) -> Tuple[Dict, ...]:
    """The plain-dict zone rows, given ALREADY-RESOLVED blocks.

    Split out of `resolve_intent_gate` (#797) because the SEAT search needs the
    same rows and must not pay for the rest of that function: it has resolved
    its blocks already, and re-running the gate resolver there would report the
    `intent_zone_in_keepout` problems a second time.

    One construction, several callers, for the reason `resolve_intent_gate`'s
    own docstring gives about the join it replaced: a filter that must be
    remembered is a filter that will be forgotten at the next call site.
    """
    return tuple(
        {'name': z.name,
         'rect': tuple(z.rect),
         'tolerance_mm': intent.zone_tolerance(z),
         'refs': tuple(blocks.get(z.name, ())),
         'side': z.side,
         'exclusive': bool(z.exclusive)}
        for z in intent.blocks if z.rect is not None)


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
    zones = zone_entries(intent, blocks)

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
    # #893. The declared ROTATION travels with the gate, so the quench can
    # enforce what the seeder honoured. Without it the feature was strictly
    # WEAKER than the advice it replaced: locking a part DID protect its angle
    # from the quench (one boolean covers position and rotation), so
    # `place_seed` -> `place_optimize` would have turned a declared part
    # straight back -- against the very U3 case the seeder docstring cites.
    return ({'rotations': rotations_for_ref(intent, blocks),
             'zones': zones,
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
        #: #712. A DECLARED claim this board's geometry cannot support a
        #: verdict on. Neither a violation nor a pass -- it joins
        #: `budget_withheld`'s existing channel, which already exists so a
        #: reader can tell "checked and clean" from "never checked".
        self.abstained: Dict[str, str] = {}
        #: #712. The along-edge measurement, taken on EVERY entry that names
        #: an edge, declared or not. A number nobody has to opt into is the
        #: half of this feature that can catch a defect nobody suspected.
        self.edge_seating: List[Dict[str, object]] = []
        #: #894. Every gap `rule_proximity` MEASURES, whether it passed or
        #: violated -- the same "a measurement, never a verdict" channel as
        #: `edge_seating` above, and for a sharper reason: `rule_proximity` is
        #: SILENT on a passing clause, because a rule yields violations. So
        #: the number behind a clause that holds -- exactly what a score wants
        #: to report and to compare against the previous lap -- was computed
        #: and thrown away, and any consumer wanting it had to re-derive the
        #: pad-resolution ladder (declared pads vs part adjacency, the net
        #: narrowing, the body basis). That second copy is the drift this
        #: repo has a dedicated test class against, so the rule RECORDS what
        #: it measures and the consumer reads it.
        self.proximity_measured: List[Dict[str, object]] = []
        self._decap_pops: Dict[float, tuple] = {}
        self._supply_pins = None
        self._assembly_census = None
        self._bodies = None
        self._bodies_error = ''
        self._oob_exempt = None
        #: #961. One row per declared edge connector on the board, passing
        #: ones included: the number its `overhang_mm` band was graded on,
        #: the currency of that number, the drawn-body measurements and the
        #: part's pad-copper edge clearance. A measurement, never a verdict.
        self.edge_connector_evidence: List[Dict[str, object]] = []
        #: The caller's own (clearance, board_edge_clearance), None = unset,
        #: so the copper evidence resolves its floor exactly as
        #: `grade_pad_legality` does. `grade` sets it.
        self.requested_floors = (None, None)
        self._connector_copper = None
        self._zero_gate = None

    def oob_exempt(self) -> Dict[str, float]:
        """`{ref: overhang_mm}` for every declared edge connector the census
        counts as leaving the outline and whose overhang lies INSIDE its own
        `overhang_mm` band -- the parts `rule_edge_connector` says are
        correct off the board, and that `rule_legality` used to count
        against `legality_budget.oob_count` anyway.

        #961: TWO readings, on purpose. "Counted" is still the census number
        described below; "inside the band" is the band's own currency -- the
        drawn body where it can be measured (what `rule_edge_connector`
        grades), else that census number. Where no body is measured the "one
        number" below holds as it did before, including its old caveat: the
        census skips the part's own milled rings and the rule does not, so
        the two differ for a part that owns milled rings.

        Measured, run 26: an intent declaring `CON1 east overhang 0..0.5` and
        `legality_budget {oob_count: 0}` (the emitter bakes the pile's own 0)
        refused all ten of its seeds on CON1's 0.25 mm overhang, and the
        placement was finished by hand.

        The amount is `rect_outside_amount` with the part's OWN milled rings
        skipped -- the exact call `QuenchState.legality_metrics` makes -- so
        "counted" and "exempt" are decided on one number, never on two
        readings of the outline. Only `edge_claims()` entries qualify (a
        `connector_affinity` row claims no edge), only entries with a `max`
        (an unbounded band would exempt any overhang whatever, which is the
        run-10 160 mm case), and only when the part is actually off the board
        (an interior connector is not "exempt" from anything). A part outside
        its band is NOT exempt: it stays in the count AND `rule_edge_connector`
        names it, two rules reporting one fact.
        """
        if self._oob_exempt is None:
            out: Dict[str, float] = {}
            for c in self.intent.edge_claims():
                ref = c['ref']
                part = self.parts.get(ref)
                lim = c.get('overhang_mm') or {}
                if part is None or lim.get('max') is None:
                    continue
                lo = float(lim.get('min', 0.0))
                hi = float(lim['max'])
                amt = self.gate.rect_outside_amount(
                    part.rect, skip_rings=self.state._owned_rings(ref))  # noqa: SLF001
                # #961: "exempt" is decided on the band's OWN currency, the
                # one `rule_edge_connector` grades -- the drawn body where it
                # can be measured, this very reading where it cannot. `amt`
                # still decides whether the census counted the part at all.
                band, _basis, _body = _band_amount(self, ref, c.get('edge'),
                                                   amt)
                # The same body path that must still see copper off the
                # outline (`rule_edge_connector`): a part with pads past the
                # edge stays counted, whatever its body reads.
                copper_ok = (not _body.get('body_measured')
                             or (_copper_outside_mm(self, ref) <= legality.EPS
                                 and not _unmodellable_pads(self, ref)))
                if (amt > legality.EPS and copper_ok
                        and lo - legality.EPS <= band <= hi + legality.EPS):
                    out[ref] = float(band)
            self._oob_exempt = out
        return self._oob_exempt

    def zero_gate(self):
        """The outline at ZERO margin -- "is this copper ON the board", which
        `self.gate` cannot answer because it carries the placement margin."""
        if self._zero_gate is None:
            self._zero_gate = legality.BoardOutlineGate(
                self.pcb.board_info, 0.0)
        return self._zero_gate

    def connector_copper(self) -> Dict[str, object]:
        """#961: pad-copper edge clearance of the DECLARED edge connectors,
        graded once per grade by `legality.grade_pad_edge_clearance` at the
        floor `grade_pad_legality` resolves -- the channel `check_drc
        --check-pad-edge` grades, reported beside the body overhang so the
        two can be read side by side. The CLEARANCE it measures is evidence
        only -- no violation, no abstention -- but `_copper_outside_mm` reads
        the same findings for copper past the OUTLINE, which the rule does
        grade on the body path. Memoised; only declared parts' pads walked."""
        if self._connector_copper is None:
            from copy import copy
            from list_nets import board_floor_knobs
            clearance, edge_margin = self.requested_floors
            _, required, knobs = board_floor_knobs(
                self.pcb_file or getattr(self.pcb, 'source_path', None),
                clearance, edge_margin)
            subset = copy(self.pcb)
            subset.footprints = {
                c['ref']: self.pcb.footprints[c['ref']]
                for c in self.intent.edge_connectors
                if c['ref'] in (self.pcb.footprints or {})}
            graded = legality.grade_pad_edge_clearance(
                subset, required, self.pcb_file)
            graded['requirement_source'] = (
                knobs['board_edge_clearance']['source'])
            self._connector_copper = graded
        return self._connector_copper

    def assembly_census(self) -> Dict[str, object]:
        """#837's per-side census, memoised. The SAME function
        `check_assembly` prints and `emit_intent` observes, so the three
        cannot come to disagree about how many parts are on a face.

        A dict walk over `fp.layer` / `fp.pads`, no geometry, so it does not
        break `_Ctx`'s contract that rules never re-derive geometry.
        """
        if self._assembly_census is None:
            from .legality import assembly_census as _ac
            self._assembly_census = _ac(self.pcb)
        return self._assembly_census

    def body_rect(self, ref: str):
        """`(rect_in_board_coords, source)` for one part's DRAWN body (#902).

        LAZY and memoised, the `_supply_pins` precedent: `body.board_bodies`
        reads the board FILE three times (courtyard, fab and silk graphics
        reach neither parse path), and no board without a proximity claim may
        pay for that.

        THE DRAWN body, `drawn_local`, and not `body_local`. #896 stores two
        ladders on purpose and says why: a courtyard "is not a body -- it is a
        body plus an assembly margin plus any shell overhang", and
        `body_local` is courtyard-FIRST. Reading it would have made
        `basis: "body"` silently measure courtyards on any board that draws
        them, under-stating every gap by the assembly margin and passing
        claims that should fail -- while `basis: "courtyard"` is refused BY
        NAME on the grounds that boards do not draw them. The fixture this
        rule was written against draws 0 courtyards of 21, so no test here
        could ever have seen it.

        `drawn_local` is `fab`, else `silk` unioned with the pads. When the
        library drew NEITHER it is None, and the fall-back is the pad bbox
        from `body_local` -- reported as `pad_bbox`, so a reader always knows
        the number rests on copper rather than on a drawn outline. That
        fall-back is what lets a transistor pair sharing no net be measured at
        all, which is the case the second basis exists for.

        `occupancy_local` is deliberately never used: it is the body unioned
        with the pads, which answers "may something be seated here" and would
        make `body` a superset of `pad_edge` rather than a second currency.
        """
        if self._bodies is None:
            from .body import board_bodies
            try:
                self._bodies = board_bodies(self.pcb, self.pcb_file)
            except Exception as exc:                         # noqa: BLE001
                # Remembered, and reported as ITSELF. Falling through to the
                # "this part draws no body" reason would tell an author a
                # false fact about their board and send them to fix a
                # footprint that is fine.
                self._bodies = {}
                self._bodies_error = f"{type(exc).__name__}: {exc}"
        return drawn_body_rect(self._bodies.get(ref),
                               self.pcb.footprints.get(ref))

    def sev(self, rule: str) -> str:
        return self.intent.severity_of(rule)

    def abstain(self, key: str, why: str) -> None:
        self.abstained[key] = why
    def supply_pins(self):
        """The #705 ladder, memoised. `_arm_decap_pins` and the rule read
        the SAME record, so the abstention reason and the findings can
        never describe two different pin sets."""
        if self._supply_pins is None:
            kw = (self.intent.decaps or {}).get('pin_functions')
            self._supply_pins = supply_pins(self.pcb, pin_functions=kw)
        return self._supply_pins

    def decap_populations(self, radius: float):
        """(near, beyond, orphans), memoised per radius (#794).

        `decap_distance` and `decap_ungraded` divide ONE population between
        them, so they must read one election -- otherwise "these two rules
        partition the caps" is a claim about two calls agreeing rather than a
        property of the code. Memoised because this class's own contract is
        "built once; rules never re-derive geometry", and the two rules run
        back to back over the same board.
        """
        key = round(float(radius), 6)
        hit = self._decap_pops.get(key)
        if hit is None:
            hit = groups_mod.decap_populations(self.pcb, radius=radius)
            self._decap_pops[key] = hit
        return hit

    def superseded(self) -> Dict[str, str]:
        """`{cap: claim}` -- the caps a DECLARED proximity relation
        supersedes (#959), read against the brief the grade was handed.
        Empty with no brief."""
        hit = getattr(self, '_superseded', None)
        if hit is None:
            hit = superseded_caps(self.pcb, self.intent.proximity,
                                  getattr(self, 'brief_fragment', None))
            self._superseded = hit
        return hit


def _nearest_edge(rect, bounds) -> str:
    d = {'west': rect[0] - bounds[0], 'north': rect[1] - bounds[1],
         'east': bounds[2] - rect[2], 'south': bounds[3] - rect[3]}
    return min(d, key=lambda k: d[k])


def edge_seat_rect(entry: Dict, part_rect, body_rect):
    """`(rect, basis)` on which `rule_edge_connector` asks where a declared
    edge part's MATING FACE is: its nearest-edge and setback conjuncts.

    The drawn body for an `edge_receptacle`, or an entry whose context says
    `mount_mode: edge_mount`, when the library drew one (fab or silk);
    otherwise the part's courtyard rect. `body_rect` is a zero-argument
    callable returning `(rect, source)`, called only when the entry asks for
    a body, so a board with no receptacle never reads its bodies.

    Lifted out of the rule (#975) so an edge seat that picks a pose the rule
    has not seen can ask the rule's own question of it, rather than a copy.
    """
    ctxd = entry.get('context') or {}
    if (entry.get('class') == 'edge_receptacle'
            or ctxd.get('mount_mode') == 'edge_mount'):
        brect, src = body_rect()
        if brect is not None and src in ('fab', 'silk'):
            return brect, f'body:{src}'
    return part_rect, 'courtyard'


#: The axis a part slides along when it moves ALONG the named edge: x for a
#: horizontal edge, y for a vertical one.
_EDGE_AXIS = {'north': 0, 'south': 0, 'east': 1, 'west': 1}

#: How close a ring segment must lie to the bounding-box side to count as
#: being ON that side. Board coordinates are mm at 4-6 decimal places.
_EDGE_SPAN_TOL_MM = 0.01


def edge_span(gate, bounds, edge: str, outline: Dict, *,
              tol: float = _EDGE_SPAN_TOL_MM):
    """(lo, hi, basis) along `edge`, or (None, None, reason-why-not) (#712).

    An along-edge FRACTION is only as good as the span it is a fraction of,
    and `_nearest_edge` / `edge_clearance` both answer off the bounding box.
    On a notched board those disagree, measured: `interf_u_unrouted_placed`'s
    bbox south side spans 115.570mm where the board's real south edge spans
    81.280mm, and `BUS1` -- which sits EXACTLY on the real centre -- reads
    5.715mm off against the bbox. A silently wrong centring number is worse
    than none, so this abstains rather than guess.

    Three branches, in order, and the order is the whole design:

      1. The outline RINGS resolve this edge to ONE contiguous run -> use it.
      2. There are no rings at all AND the outline is a simple rectangle with
         no cutout -> the bounding box IS the outline, exactly. Use it. This
         is not a fallback: `extract_board_contours` short-circuits a plain
         rectangle and publishes no ring, so branch 1 cannot fire on most of
         the corpus and the bbox is the only correct answer there.
      3. Otherwise -> abstain, naming what was found.

    Deliberately PER EDGE, not per board. The obvious rule ("abstain unless
    `simple_rectangle`") is refuted by measurement: `watchy` is not a simple
    rectangle (21 segments, 2 cutouts) yet its east and west ring spans are
    IDENTICAL to the bbox on all 13 of its declared entries, so a per-board
    abstention would discard 13 correct measurements to avoid one wrong one.
    """
    ax = _EDGE_AXIS[edge]
    perp = 1 - ax
    rings = list(getattr(gate, 'rings', None) or [])
    if rings:
        target = {'north': bounds[1], 'south': bounds[3],
                  'west': bounds[0], 'east': bounds[2]}[edge]
        runs = []
        for seg in gate.edges():
            a, b = (seg[0], seg[1]), (seg[2], seg[3])
            if abs(a[perp] - target) > tol or abs(b[perp] - target) > tol:
                continue
            lo, hi = sorted((a[ax], b[ax]))
            if hi - lo > 1e-9:
                runs.append([lo, hi])
        if not runs:
            return (None, None,
                    f"no Edge.Cuts segment lies on the {edge} side of this "
                    f"outline ({len(rings)} ring(s), "
                    f"{outline.get('cutouts', 0)} cutout(s)), so that edge has "
                    f"no span to take a fraction of")
        runs.sort()
        merged = [runs[0]]
        for lo, hi in runs[1:]:
            if lo <= merged[-1][1] + tol:
                merged[-1][1] = max(merged[-1][1], hi)
            else:
                merged.append([lo, hi])
        if len(merged) != 1:
            pieces = ', '.join(f"{m[1] - m[0]:.2f}mm" for m in merged)
            return (None, None,
                    f"the {edge} edge of this outline is {len(merged)} "
                    f"separate runs ({pieces}), not one span, so 'along the "
                    f"edge' has no single meaning here")
        return (merged[0][0], merged[0][1], 'ring')
    if outline.get('simple_rectangle') and not outline.get('cutouts'):
        return ((bounds[0], bounds[2], 'bbox') if ax == 0
                else (bounds[1], bounds[3], 'bbox'))
    return (None, None,
            f"this board's Edge.Cuts parsed no ring and is not a simple "
            f"rectangle ({outline.get('edge_segments', 0)} segment(s), "
            f"{outline.get('cutouts', 0)} cutout(s)), so the {edge} edge has "
            f"no span this tool can defend a fraction against")


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


def zone_containment_of(intent: 'Intent', pcb_data, pcb_file: str, refs,
                        *, group_sources: Sequence[str] = (),
                        clearance: Optional[float] = None,
                        board_edge_clearance: Optional[float] = None
                        ) -> Dict[Tuple[str, str], 'Violation']:
    """`rule_zone_containment`'s findings for `refs` alone, keyed
    `(ref, block)` (#959, #998). What `place_pose --intent` compares on its
    input and its candidate: the GRADE's own rule on the grade's own context,
    so the write-time check and the later grade cannot disagree about where
    a zone ends. Raises `UntrustworthyOutline` as `grade` does."""
    ctx = _grade_ctx(intent, pcb_data, pcb_file, group_sources=group_sources,
                     clearance=clearance,
                     board_edge_clearance=board_edge_clearance)[0]
    want = set(refs)
    return {(v.ref, v.block): v for v in rule_zone_containment(ctx)
            if v.ref in want}


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


def rule_assembly_side(ctx) -> Iterator[Violation]:
    """#837. Parts on a face the declared assembly policy does not populate.

    WARN by default, and that is the design rather than timidity. Nothing in
    the engine can move a part between faces -- `_Part.side` is set once at
    construction and no move carries it (#836) -- so an ERROR here would be a
    red mark no run could clear, which is the defect `zone_side` already
    carries (`docs/floorplan-intent.md`: "vacuous, not conservative"). A
    second instance of it would be a step backwards. An author who wants the
    hard gate sets `severity: {"assembly_side": "error"}` and means it.

    `severity_of(..., default=WARN)` rather than `ctx.sev(...)`, which
    hard-defaults to ERROR -- the same trap `rule_decap_ungraded` records.

    Graded on the BODY face, never `part.sides`. A through-hole part on the
    front does not demand a reflow pass on the back; it demands wave or hand
    soldering, and `legality.sides_occupied` -- which answers the obstruction
    question -- would flag 24 parts on splitflap_driver, a board with nothing
    on its back at all.
    """
    want = ctx.intent.assembly_sides()
    if want != 'F' and want != 'B':
        # 'both' (declared or defaulted): every face is populated, so no part
        # can be on an undeclared one. The rule RAN and found nothing, which
        # is a different report from "nobody asked" -- `_wants` keeps that
        # distinction, and the docstring above says why the choice went this
        # way.
        return
    sev = ctx.intent.severity_of('assembly_side', default=WARN)
    other = 'B' if want == 'F' else 'F'
    # The census's own ref list, NOT `ctx.parts`. The two agree on every
    # tracked board and are not the same set by construction: `QuenchState`
    # admits a zero-pad footprint that draws a courtyard as a locked obstacle
    # and the pad-bearing census excludes it. One such board and
    # `check_assembly` would print a count `check_floorplan` contradicts --
    # two populations for one number, which is the defect #837 is about.
    for ref in ctx.assembly_census()['pad_bearing_refs'][other]:
        yield Violation(
            rule='assembly_side', severity=sev, ref=ref,
            message=(f"{ref} is on side {other} but the board declares "
                     f"assembly.sides {want}"),
            measured={'side': other}, expected={'side': want})


def rule_zone_exclusive(ctx) -> Iterator[Violation]:
    """An exclusive zone is real estate reserved for its block, so a stranger
    inside it is the finding. This is what makes "keep this area clear for the
    RF section" checkable rather than aspirational."""
    for z in ctx.intent.blocks:
        if z.rect is None or not z.exclusive:
            continue
        members = set(ctx.blocks.get(z.name, ()))
        # #797: a zone with no member VISIBLE HERE cannot tell a member from a
        # stranger, so it grades nobody. Without this the rule flags every
        # part on the board -- including the ones the block was drawn around --
        # which is the rule inverted rather than applied.
        #
        # It is not enough to leave this to `block_unresolved`, and that was
        # measured rather than assumed: `block_unresolved` is a SETTABLE
        # severity, so an intent that downgrades it to `warn` produced a run
        # whose ONLY error was `zone_exclusive`, against a part the seat gate
        # had deliberately declined to bind and therefore could not repair.
        # The seat gate (`quench.exclusive_spec`) skips the same zones, so the
        # two agree at every severity.
        if not (members & set(ctx.parts)):
            continue
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
    reporting it as a defect forever -- kept by `rule_legality` through
    `_Ctx.oob_exempt()`, for a declared part inside its own overhang band
    (it was a promise with no implementation until run 26 measured it)."""
    for c in ctx.intent.edge_connectors:
        ref = c['ref']
        part = ctx.parts.get(ref)
        if part is None:
            yield Violation(
                rule='edge_connector', severity=ctx.sev('edge_connector'),
                ref=ref, message=f"edge connector {ref} is not on this board",
                measured={'found': False})
            continue
        # #959 (#1000): the face the brief's viewing side puts it on. A
        # fixed WARN, never the configured severity: nothing moves a part
        # between faces (#836), and the seat search and quench do not read
        # it, so it can steer nothing.
        if c.get('side') and part.side != c['side']:
            yield Violation(
                rule='edge_connector_side', severity=WARN, ref=ref,
                message=(f"{ref} is on {part.side}.Cu, but the declared "
                         f"viewing face puts it on {c['side']}.Cu -- advisory:"
                         f" nothing in the placement stack moves a part "
                         f"between faces"),
                measured={'side': part.side}, expected={'side': c['side']})
        amount = ctx.gate.rect_outside_amount(part.rect)
        lim = c.get('overhang_mm') or {}
        lo = float(lim.get('min', 0.0))
        hi = lim.get('max')
        # #961: the band is graded on the DRAWN BODY's overhang past the
        # outline -- summed over the sides it crosses, the form of the reading
        # it replaces -- at zero margin, wherever that body can be measured.
        # `amount` is the occupancy reading at the gate's margin -- `margin -
        # gap` inside the board, `overhang + margin` outside -- and graded
        # alone it let esp_prog's USB1, 0.15 mm INSIDE its edge, read 0.10 and
        # satisfy a band `check_drc --check-pad-edge` failed. Where no body
        # can be read `band` IS `amount`, and `overhang_basis` says which it
        # was. `amount` keeps its other job: the setback conjunct's
        # no-overhang gate below, which #961 does not change.
        band, overhang_basis, body = _band_amount(ctx, ref, c.get('edge'),
                                                  amount)
        if band < lo - legality.EPS:
            yield Violation(
                rule='edge_connector', severity=ctx.sev('edge_connector'),
                ref=ref, message=(f"{ref} overhangs the outline by "
                                  f"{band:.2f}mm, under the declared minimum "
                                  f"{lo:.2f}mm ({overhang_basis})"),
                measured={'overhang_mm': round(band, 4),
                          'overhang_basis': overhang_basis},
                expected={'min': lo, 'max': hi})
        elif hi is not None and band > float(hi) + legality.EPS:
            yield Violation(
                rule='edge_connector', severity=ctx.sev('edge_connector'),
                ref=ref, message=(f"{ref} overhangs the outline by "
                                  f"{band:.2f}mm, past the declared maximum "
                                  f"{float(hi):.2f}mm ({overhang_basis})"),
                measured={'overhang_mm': round(band, 4),
                          'overhang_basis': overhang_basis},
                expected={'min': lo, 'max': float(hi)})
        # A band licenses the BODY, never copper. The occupancy reading this
        # replaced often carried pad copper in front of the body -- it
        # measures the courtyard, which is the PAD BOX itself on a part that
        # draws none -- and the body reading never does, so on the body path
        # copper past the outline is named here. Without it a part with its
        # pads off the board grades clean where it used to fail twice
        # (measured: tigard J7 flush with its edge, 0.2 mm of copper off).
        # BODY PATH ONLY, and not because the legacy reading is equivalent:
        # a courtyard that does not enclose its pads misses the same copper,
        # on this branch and on main alike (the suite pins that with a
        # courtyard-only fixture carrying 0.75 mm of unnamed copper). It is
        # scoped so the path this change cannot measure grades exactly as it
        # did before #961.
        copper_out = (_copper_outside_mm(ctx, ref)
                      if body.get('body_measured') else 0.0)
        if copper_out > legality.EPS:
            yield Violation(
                rule='edge_connector', severity=ctx.sev('edge_connector'),
                ref=ref, message=(f"{ref}'s pad copper leaves the outline by "
                                  f"{copper_out:.2f}mm; its band is graded on "
                                  f"the drawn body ({overhang_basis}) and "
                                  f"licenses no copper"),
                measured={'pad_copper_outside_mm': round(copper_out, 4),
                          # `_charge` in the seeder's repair census reads its
                          # magnitude from `outside_mm`.
                          'outside_mm': round(copper_out, 4),
                          'overhang_basis': overhang_basis},
                expected={'pad_copper_outside_mm': 0.0})
        evidence = _connector_evidence(ctx, c, ref, band, overhang_basis, body,
                                       lo, hi)
        # The SEAT BASIS, decided once for the two conjuncts that ask where
        # the part's MATING FACE is (nearest edge, seat). A receptacle's
        # pads sit well inboard of its opening by construction: a micro-USB
        # shell's SMD pads are 1.6-2.1 mm behind it. For an
        # `edge_receptacle` (or a brief row carrying `mount_mode:
        # edge_mount`) both conjuncts are therefore measured on the DRAWN
        # body when the library drew one; everything else keeps the
        # courtyard. (The OVERHANG conjunct above no longer does: since #961
        # it reads the drawn body wherever one can be measured, because the
        # edge-margin graze read a flush body as a 0.55 mm overhang.)
        seat_rect, basis = edge_seat_rect(c, part.rect,
                                          lambda: ctx.body_rect(ref))
        edge = c.get('edge')
        if edge and ctx.outline_bounds:
            # Run 27's replay measured the courtyard reading on the same
            # USB1: its pad box is 1.6 mm from the west edge and 1.3 mm from
            # the south, so a socket flush with the west edge read "nearest
            # the south edge but declared on the west" -- on every one of
            # ten seeds, since the socket is a fixed part.
            actual = _nearest_edge(seat_rect, ctx.outline_bounds)
            if actual != edge:
                yield Violation(
                    rule='edge_connector', severity=ctx.sev('edge_connector'),
                    ref=ref, message=(f"{ref} sits nearest the {actual} edge "
                                      f"but is declared on the {edge} edge"),
                    measured={'edge': actual, 'basis': basis},
                    expected={'edge': edge})
        # Run-4 A: the missing PROXIMITY conjunct. The rule used to grade only
        # the overhang band and the nearest-edge identity, so a receptacle
        # 15.8 mm INTERIOR passed ("nearest west, declared west" is satisfied
        # anywhere on the board). For entries carrying the edge_receptacle
        # class (or an explicit max_setback_mm), no overhang AND off-seat is
        # a violation -- which is also what makes a misplaced edge part
        # CHARGEABLE by place_seed --repair.
        setback = c.get('max_setback_mm')
        _sev = ctx.sev('edge_connector')
        # #959 (#1000): a connector standing off a face is not held to the
        # receptacle seat -- its declared edge and overhang still grade it.
        vertical = ((c.get('context') or {}).get('mount_mode')
                    in VERTICAL_MOUNTS)
        if setback is None and c.get('class') == 'edge_receptacle' \
                and not vertical:
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
            # The SEAT is a question about the part's BODY -- does its
            # mating face reach the edge -- on `seat_rect`, the basis
            # decided above. Measured on run 26's board: USB1's drawn fab
            # body sits at 0.00 mm from the west edge where its pad box
            # reads 2.1 mm, so the courtyard (here the pad-bbox fallback)
            # reported "seated 1.30mm ... no overhang" on a socket that was
            # flush, and the brief's four USB1 clauses had to be waived.
            clr = ctx.gate.edge_clearance(seat_rect)
            if clr > float(setback) + legality.EPS:
                yield Violation(
                    rule='edge_connector', severity=_sev,
                    ref=ref,
                    message=(f"{ref} is an edge part seated {clr:.2f}mm from "
                             f"the nearest edge with no overhang ({basis}; "
                             f"seat tolerance {float(setback):.2f}mm) -- "
                             + ("a plug may not reach it; disposition in the "
                                "boundary review or declare max_setback_mm"
                                if _sev == WARN else
                                "the mating face cannot reach the edge")),
                    measured={'edge_clearance_mm': round(clr, 4),
                              'basis': basis,
                              'courtyard_clearance_mm': round(
                                  ctx.gate.edge_clearance(part.rect), 4)},
                    expected={'max_setback_mm': float(setback)})

        # #712: WHERE ALONG the edge. The three conjuncts above are all
        # satisfied anywhere along it, so a receptacle well off the centre of
        # its edge grades exactly as well as a centred one. Measured on the
        # tracked corpus: esp_prog's USB1 sits 1.75mm off the centre of its
        # 14.50mm east edge -- 12.07% -- and no conjunct could say so.
        # (#712's own report cites 2.35mm on a 14.5mm edge, from a board
        # revision that is not in this repo; the number measurable HERE is
        # 1.75mm, and that is the one this code is checked against.)
        #
        # MEASURED ALWAYS, GRADED ONLY WHEN DECLARED. The offset reaches
        # `edge_seating` on every entry naming an edge, because a number
        # nobody opted into is what catches a defect nobody suspected. A
        # VIOLATION needs an explicit `center_on_edge` / `along_edge_band`,
        # because there is no defensible default: measured on the tracked
        # corpus, tigard's three connectors sit at +16.1 / -25.4 / -28.7
        # percent off their edge centres, so any threshold this tool chose
        # would fail a good human board 3 times out of 3. The author writes
        # the number or there is no claim.
        # `ctx.sev(...)`, NOT `_sev`. `_sev` is forced to WARN for an entry
        # the EMITTER classed `connector_affinity`, and that forcing is about
        # the interior-proximity conjunct above: a generic connector sitting
        # mid-board is a flag for the boundary review, never an error. Letting
        # it reach here would let an INFERRED class silently downgrade an
        # author's EXPLICIT along-edge claim -- "declared outranks inferred"
        # inverting. Measured on esp_prog: with the class set, the same
        # violation went [ERROR] -> [warn ], errors 6 -> 4, and had it been
        # the only finding the exit would have flipped 4 -> 0.
        rows_before = len(ctx.edge_seating)
        yield from _grade_along_edge(ctx, c, ref, part,
                                     ctx.sev('edge_connector'))
        # #961: "the edge_seating row carries the number its clause was graded
        # on". Onto the row the along-edge measurement just appended, when it
        # appended one -- never a NEW row, because `summary` counts rows.
        # `setdefault` rather than assignment so a key the row already owns
        # keeps its own value; today the two dicts share only `ref` and
        # `edge`, with equal values, so it is a guard rather than a fix.
        # Every entry, row or not, lands in `edge_connector_evidence`, so a
        # passing clause is reported too.
        if len(ctx.edge_seating) > rows_before:
            for key, value in evidence.items():
                ctx.edge_seating[-1].setdefault(key, value)
        ctx.edge_connector_evidence.append(evidence)


def _band_amount(ctx, ref, edge, legacy_amount):
    """`connector_geometry.band_amount` for a grade context: one geometry per
    context, the gate's own margin named in the legacy basis."""
    from .connector_geometry import band_amount, geometry_for
    return band_amount(geometry_for(ctx, ctx.pcb, ctx.pcb_file), ref, edge,
                       legacy_amount, ctx.gate.margin)


def _unmodellable_pads(ctx, ref):
    """Indices of `ref`'s pads `grade_pad_edge_clearance` could not model.

    It records them as unmeasured and produces no finding, so nothing exact
    is known about where their copper reaches. A part carrying one is never
    CERTIFIED clean of copper past the outline: it keeps its `oob_count`
    charge and its evidence row says `certified: false`."""
    fp = ctx.pcb.footprints.get(ref)
    return {index for index, pad in enumerate(getattr(fp, 'pads', None) or ())
            if not legality._pad_has_no_copper(pad)            # noqa: SLF001
            and not legality.pad_shape_is_modelled(pad)}


def _copper_outside_mm(ctx, ref):
    """How far a declared connector's pad copper reaches past the outline, at
    zero margin: the largest `-gap` among its own edge-clearance findings.

    #961's round-2 review: a band graded on the drawn body stops seeing pad
    copper that sits in front of that body, which the occupancy reading (a
    courtyard, often the pad box itself) always carried. A band licenses the
    body, never copper, so the body path must still see it. Castellated pads
    straddle the outline by design and are skipped. 0.0 when nothing leaves
    the board; findings from a sampled (non-rectangular) outline carry no gap
    and are not counted, which is harmless because the body path only runs on
    rectangular outlines."""
    fp = ctx.pcb.footprints.get(ref)
    prefix = ref + '.'
    worst = 0.0
    for f in ctx.connector_copper()['findings']:
        if not str(f['pad_ref']).startswith(prefix) or f.get('gap_mm') is None:
            continue
        pads = getattr(fp, 'pads', None) or ()
        index = f.get('pad_index')
        if (isinstance(index, int) and index < len(pads)
                and getattr(pads[index], 'castellated', False)):
            continue
        worst = max(worst, -float(f['gap_mm']))
    # A pad the edge grader cannot model -- a trapezoid, a custom pad with no
    # parsed primitives -- yields NO finding: it is recorded as unmeasured
    # and skipped, so the loop above reads 0.0 for a pad that may be entirely
    # off the board. Upstream needed no conjunct there, because its band read
    # the pad box itself. For those pads only, fall back to the rotated
    # pad-box reading the seat predicate uses. That box is a BEST EFFORT, not
    # a bound: a trapezoid's copper lies up to its `rect_delta` outside it
    # (pcbnew measures 1.10 mm where the box says 1.00), and the parser does
    # not expose `rect_delta`, so nothing here can recover the true extent.
    # A positive reading therefore still names copper off the board, and a
    # zero one certifies nothing -- which is why `_unmodellable_pads` also
    # withholds the part's exemption and marks its row uncertified. The exact
    # extrema above still decide every pad the grader could model.
    unsupported = _unmodellable_pads(ctx, ref)
    if unsupported and fp is not None:
        from .connector_geometry import geometry_for, pad_copper_outside
        worst = max(worst, pad_copper_outside(
            geometry_for(ctx, ctx.pcb, ctx.pcb_file), ctx.zero_gate(), ref,
            (fp.x, fp.y, fp.rotation or 0.0), only=unsupported))
    return worst


def _connector_evidence(ctx, c, ref, band, basis, body, lo, hi):
    """One `edge_connector_evidence` row (#961): the band's number and
    currency, the body measurements behind it, and the part's pad-copper edge
    clearance from `_Ctx.connector_copper` -- units, limits and a disposition
    on every channel, passing ones included."""
    def r4(v):
        return None if v is None else round(float(v), 4)

    copper = ctx.connector_copper()
    prefix = ref + '.'
    findings = [f for f in copper['findings']
                if str(f['pad_ref']).startswith(prefix)]
    unmeasured = [u for u in copper['unmeasured']
                  if str(u['pad_ref']).startswith(prefix)]
    gap = (copper.get('minimum_gap_by_ref_mm') or {}).get(ref)
    if findings:
        copper_disposition = 'fail'
    elif unmeasured or copper['rules_unmeasured']:
        copper_disposition = 'unmeasured'
    elif gap is None:
        copper_disposition = 'no copper pads measured'
    else:
        copper_disposition = 'pass'
    over = (band < lo - legality.EPS
            or (hi is not None and band > float(hi) + legality.EPS))
    others = body.get('other_body_edge_overhang_mm')
    return {
        'ref': ref, 'edge': c.get('edge'), 'units': 'mm',
        'overhang_mm': round(band, 4),
        'overhang_basis': basis,
        'overhang_limit_mm': {'min': lo,
                              'max': None if hi is None else float(hi)},
        'overhang_disposition': 'fail' if over else 'pass',
        'effective_margin_mm': ctx.gate.margin,
        'body_measured': body['body_measured'],
        'body_layer': body.get('body_layer'),
        # The declared edge alone, beside the summed number the band read.
        'body_overhang_mm': r4(body.get('body_overhang_mm')),
        'body_signed_position_mm': r4(body.get('body_signed_position_mm')),
        'body_setback_mm': r4(body.get('body_setback_mm')),
        'other_body_edge_overhang_mm': (
            None if others is None else {e: r4(v) for e, v in others.items()}),
        'body_unmeasured_reason': body.get('body_unmeasured_reason'),
        'pad_copper_edge': {
            'required_mm': copper['required_mm'],
            'requirement_source': copper.get('requirement_source'),
            'minimum_gap_mm': r4(gap),
            'shortfall_mm': round(max((f['shortfall_mm'] for f in findings),
                                      default=0.0), 4),
            # Past the outline itself, castellated pads excepted: the number
            # the body path's copper conjunct grades (0.0 = on the board).
            # None when a finding came from the sampled (non-rectangular)
            # path, which carries no gap -- the amount is unknown, not zero.
            'outside_mm': (None if any(
                str(f['pad_ref']).startswith(prefix) and f.get('gap_mm') is None
                for f in copper['findings'])
                else round(_copper_outside_mm(ctx, ref), 4)),
            # False when a pad's shape defeated the edge grader: `outside_mm`
            # is then a best-effort box reading, so a zero says "not shown to
            # be off the board", never "on the board". Such a part is not
            # exempted from the occupancy census either.
            'certified': not _unmodellable_pads(ctx, ref),
            # The pads this grade walked: every DECLARED connector's, never
            # the whole board's. One number for the grade, repeated on each
            # row -- not this part's own count.
            'measured_pads': copper['measured_pads'],
            'findings': findings, 'unmeasured': unmeasured,
            'rules_unmeasured': copper['rules_unmeasured'],
            'disposition': copper_disposition,
            'basis': copper['basis'], 'units': 'mm'},
    }


def _grade_along_edge(ctx, c, ref, part, sev) -> Iterator[Violation]:
    """The along-edge conjunct of `rule_edge_connector` (#712).

    A separate function only for length; it is deliberately NOT a rule of its
    own, so `RULES`, `_wants`, `_SKIP_REASON` and `_SEVERITY_KEYS` need no
    entry and `rules_run` bookkeeping is untouched -- which is what the issue
    asks for, and is safe because every conjunct in this rule yields
    independently and none of them `continue`s.
    """
    centre_claim = c.get('center_on_edge')
    band_claim = c.get('along_edge_band')
    declared = centre_claim is not None or band_claim is not None
    key = (f"edge_connectors[{ref}]."
           + ('center_on_edge' if centre_claim is not None
              else 'along_edge_band' if band_claim is not None else 'position'))

    edge = c.get('edge')
    if not edge:
        if declared:
            ctx.abstain(key, "the entry declares no `edge`, so there is no "
                             "span to take a fraction along -- name the edge, "
                             "or drop the along-edge claim")
        return
    if not ctx.outline_bounds:
        if declared:
            ctx.abstain(key, "this board has no usable bounds")
        return

    lo, hi, basis = edge_span(ctx.gate, ctx.outline_bounds, edge, ctx.outline)
    if lo is None:
        ctx.edge_seating.append({'ref': ref, 'edge': edge, 'declared': declared,
                                 'abstained': basis})
        if declared:
            ctx.abstain(key, basis)
        return

    ax = _EDGE_AXIS[edge]
    span = hi - lo
    if span <= 1e-9:
        return
    centre = (part.rect[ax] + part.rect[ax + 2]) / 2.0
    offset = centre - (lo + span / 2.0)
    frac = (centre - lo) / span
    row = {'ref': ref, 'edge': edge, 'declared': declared, 'basis': basis,
           'span_mm': round(span, 4),
           'along_edge_offset_mm': round(offset, 4),
           'along_edge_offset_pct': round(100.0 * offset / span, 2),
           'along_edge_fraction': round(frac, 4)}
    ctx.edge_seating.append(row)
    if not declared:
        return

    # `toward` reuses `_rect_escape`'s compass vocabulary rather than
    # inventing a second one for the same four directions.
    toward = ({0: ('west', 'east')}.get(ax) or ('north', 'south'))[offset > 0]
    if centre_claim is not None:
        tol = float(centre_claim['tolerance_mm'])
        if abs(offset) > tol + legality.EPS:
            yield Violation(
                rule='edge_connector', severity=sev, ref=ref,
                message=(f"{ref} sits {abs(offset):.2f}mm "
                         f"({abs(100.0 * offset / span):.1f}% of the "
                         f"{span:.2f}mm {edge} edge) {toward} of that edge's "
                         f"centre, past the declared tolerance {tol:.2f}mm"),
                measured={'along_edge_offset_mm': round(offset, 4),
                          'along_edge_offset_pct': round(100.0 * offset / span, 2),
                          'along_edge_fraction': round(frac, 4),
                          # `_charge` in the seeder's repair census reads a
                          # magnitude off `measured` and falls back to 1.0mm,
                          # which would sort a 52mm centring error below every
                          # pad graze. Publish the magnitude it can use.
                          'outside_mm': round(abs(offset) - tol, 4)},
                expected={'center_on_edge_mm': tol, 'edge_span_mm': round(span, 4)})
        return

    f0 = float(band_claim['from'])
    f1 = float(band_claim['to'])
    # Compared in MILLIMETRES, so there is one epsilon in one currency: a
    # fraction epsilon would mean something different on a 14mm edge and a
    # 200mm one.
    past = max(f0 * span - (centre - lo), (centre - lo) - f1 * span)
    if past > legality.EPS:
        yield Violation(
            rule='edge_connector', severity=sev, ref=ref,
            message=(f"{ref} sits at {100.0 * frac:.0f}% along the "
                     f"{span:.2f}mm {edge} edge, outside the declared band "
                     f"{100.0 * f0:.0f}%-{100.0 * f1:.0f}% "
                     f"({past:.2f}mm past its nearer end)"),
            measured={'along_edge_fraction': round(frac, 4),
                      'along_edge_offset_mm': round(offset, 4),
                      'along_edge_offset_pct': round(100.0 * offset / span, 2),
                      'outside_mm': round(past, 4)},
            expected={'along_edge_band': {'from': f0, 'to': f1},
                      'edge_span_mm': round(span, 4)})


def superseded_caps(pcb_data, proximity, brief_fragment) -> Dict[str, str]:
    """`{cap: claim}` -- decoupling caps whose INFERRED tether a DECLARED
    proximity relation replaces (#959 comment 3.2).

    A relation supersedes only when it is the BRIEF's own -- present in the
    compiled brief with the same partner, limit, basis and pads, so a row a
    zone plan wrote (a hypothesis) cannot launder a cap out of the decap
    rules -- and when it names the cap's pads on the cap's RAIL: a pad whose
    net the partner also carries and that is not ground. Computed at grade
    time, never written into `decaps.exempt`: exempting a cap would turn a
    declared `max_pin_distance_mm` on its rail into `decap_pin_uncovered`,
    weakening a declared value to excuse an inferred one."""
    if not brief_fragment:
        return {}
    brief_rows = {(str(p.get('ref')), str(p.get('near'))): p
                  for p in brief_fragment.get('proximity') or ()}
    fps = pcb_data.footprints or {}
    out: Dict[str, str] = {}
    for p in proximity or ():
        key = (str(p.get('ref')), str(p.get('near')))
        b = brief_rows.get(key)
        if b is None:
            continue
        if any(b.get(k) != p.get(k) for k in ('max_mm', 'basis', 'pads')):
            continue            # drifted from the brief: not a declaration
        pads = p.get('pads') or {}
        for cap, partner in (key, key[::-1]):
            fp_c, fp_p = fps.get(cap), fps.get(partner)
            if fp_c is None or fp_p is None \
                    or not groups_mod.is_decoupling_cap(fp_c, cap):
                continue
            named = set(pads.get(cap) or ())
            partner_nets = {q.net_id for q in fp_p.pads if q.net_id > 0}
            if any(q.pad_number in named and q.net_id in partner_nets
                   and _gradeable_supply_net(pcb_data, q.net_id) is not None
                   for q in fp_c.pads):
                out[cap] = f"proximity {key[0]} near {key[1]}"
    return out


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
    # Read through `_Ctx` so this rule and `decap_ungraded` divide ONE
    # election. `near` is byte-identical to `decap_tethers(radius)` and
    # `tests/test_792_decap_predicate.py` asserts that on every tracked board.
    tethers, _beyond, _orphans = ctx.decap_populations(radius)
    sup = ctx.superseded()
    observed = (ctx.intent.basis or {}).get(
        'decaps.max_distance_mm') == 'observed_baseline'
    for ic in sorted(tethers):
        for cap, dist in tethers[ic]:
            if any(fnmatch.fnmatch(cap, pat) for pat in exempt):
                continue
            if cap in sup:
                continue        # graded by the declared relation instead
            if dist > limit + legality.EPS:
                yield Violation(
                    rule='decap_distance', severity=ctx.sev('decap_distance'),
                    ref=cap, block=ctx.owner.get(cap),
                    message=(f"{cap} is {dist:.2f}mm from {ic}, the IC it "
                             f"decouples (limit {limit:.2f}mm"
                             + (" -- an observed regression baseline read "
                                "off a board, not an electrical "
                                "requirement" if observed else '')
                             + ")"),
                    measured={'distance_mm': round(dist, 4), 'ic': ic},
                    expected={'max_distance_mm': limit})


def _decap_caps_by_net(pcb_data) -> Dict[int, List]:
    """{net_id: [decap footprint]} -- every cap that carries the net.

    DERIVED, deliberately not `groups.decap_tethers`. The tether map is a
    cap->one-IC assignment truncated at 5mm and it DISCARDS the net that
    matched, so it cannot answer a per-rail question at all. Measured over 271
    typed supply pins on 8 boards, a tether-derived cap set reports a LARGER
    distance on 11 and a spurious "uncovered" on 56 (interf_u_placed: 20 of
    24), because the metric here is a minimum and the tether set is a strict
    subset. Three causes: the truncation (#794), nearest-IC exclusivity (a cap
    serving two chips is invisible to one of them), and the discarded net.

    The CAP predicate is still single-sourced -- `groups.is_decoupling_cap` --
    so the two rules disagree about assignment, which is the point, and agree
    about what a decap is, which is #792.
    """
    out: Dict[int, List] = {}
    for ref, fp_ in sorted((pcb_data.footprints or {}).items()):
        if not groups_mod.is_decoupling_cap(fp_, ref):
            continue
        for p in fp_.pads:
            if p.net_id > 0:
                out.setdefault(p.net_id, []).append(fp_)
    return out


def _gradeable_supply_net(pcb_data, net_id: int) -> Optional[str]:
    """The net name, when a pin on it is a candidate for the pin rule at all.

    Rejects, each for its own reason:

    * `net_id <= 0` -- no net;
    * `unconnected-*` -- KiCad's name for a pad the designer left open. A
      `power_in+no_connect` pad lands here too, so this is the second of two
      independent refusals for `glasgow_revC` U30 pad B10 (`VPP_FAST`), and
      the redundancy is deliberate: a no-connect power pin that someone HAS
      wired must still not be graded;
    * GROUND. Measured on `lvds_converter_dualclk`, the healthiest fixture in
      the corpus: its three VCC pins sit 2.100 / 2.100 / 1.925 mm from their
      rail's nearest cap, while the three GND pins on the SAME ICs sit 8.043 /
      7.425 / 6.941 mm. Nothing is wrong with that board -- pins 7 and 14 are
      opposite corners of a SOIC-14, so what a GND arm measures is the
      PACKAGE, not the placement. And half of every `power_in` population is
      ground: 580 of 1163 corpus pins, 48 of glasgow_revC U30's 113. Grading
      them would make the pass rate a statement about pin numbering.
    """
    net = (pcb_data.nets.get(net_id).name if net_id in (pcb_data.nets or {})
           else '') or ''
    if net_id <= 0 or net.startswith('unconnected-'):
        return None
    from net_queries import is_ground_net_name
    return None if is_ground_net_name(net) else net


def supply_pins(pcb_data, *, pin_functions=None) -> Dict[str, Dict]:
    """{chip ref: record} -- which pads are supply pins, and on what evidence.

    Three channels, tried in order, **per chip**, and keyed on YIELD rather
    than on the field being present:

      1. `pintype` is `power_in`/`power_out` (token-split, `no_connect` out);
      2. `pinfunction` matches the keyword table (`decaps.pin_functions`
         replaces it);
      3. the NET NAME looks like a rail AND already carries a decoupling cap.

    PER CHIP, not per pin: a per-pin ladder mixes channels inside one chip and
    yields a pin set no channel asserts. Measured, on the 85 corpus chips where
    channels 1 and 2 both yield, they disagree about the SET on 31 (36%), so
    the order materially decides the answer. Not per board either: glasgow_revC
    wins 33 chips on `pintype` and 1 on the rail-net fallback, watchy 5 and 5.

    YIELD-KEYED, and `lvds_converter_dualclk` is why. It carries `pintype` on
    76 of 76 pads and ZERO of them are `power_in`/`power_out` -- IC2/IC3/IC4's
    supply pins are typed `passive` and named `VCC_14`, `GND_7`, `VCC_16`. A
    ladder that asks "does this board carry pintype?" stops at channel 1 with
    an empty set, grades nothing, and records that channel 1 fired: a vacuous
    pass no naturally-written test catches, because the field IS there.

    Channel 3's second conjunct is what bounds it. An INFERRED pin can never
    produce the rule's strongest claim ("nothing decouples this at all"); it
    costs only false negatives, which is the safe direction.

    "Channel N fired" is NOT "channel N is right" -- a board whose pins are all
    typed `bidirectional` yields nothing at channel 1 and falls through, and
    the tool cannot tell a mis-typed board from a correct one. That is why
    every channel's count is recorded for every chip, fired or not, and why
    channel 3's findings carry their own rule name at warn.
    """
    from net_queries import is_supply_pintype, is_supply_pinfunction, \
        is_power_net_name
    chips = groups_mod.chip_refs(pcb_data)
    by_net = _decap_caps_by_net(pcb_data)
    out: Dict[str, Dict] = {}
    for ref in sorted(chips):
        fp_ = (pcb_data.footprints or {}).get(ref)
        if fp_ is None:
            continue
        cands = []
        for p in fp_.pads:
            net = _gradeable_supply_net(pcb_data, p.net_id)
            if net is not None:
                cands.append((p, net))
        chan = {
            'pintype': [(p, n) for p, n in cands
                        if is_supply_pintype(getattr(p, 'pintype', '') or '')],
            'pinfunction': [
                (p, n) for p, n in cands
                if is_supply_pinfunction(getattr(p, 'pinfunction', '') or '',
                                         pin_functions)],
            'rail_net': [(p, n) for p, n in cands
                         if is_power_net_name(n) and by_net.get(p.net_id)],
        }
        order = ('pintype', 'pinfunction', 'rail_net')
        won = next((c for c in order if chan[c]), None)
        pins = chan[won] if won else []
        # Whether the ladder's ORDER decided the answer, per chip -- the
        # per-board form of the 36% disagreement above. A reader can see when a
        # finding rests on which channel happened to be tried first.
        nxt = next((c for c in order
                    if c != won and chan[c]), None) if won else None
        out[ref] = {
            'ref': ref,
            'channel': won,
            'counts': {c: len(chan[c]) for c in order},
            'agrees_with_next': (None if nxt is None else
                                 ({id(p) for p, _n in chan[won]}
                                  == {id(p) for p, _n in chan[nxt]})),
            'pins': pins,
        }
    return out


def rule_decap_ungraded(ctx) -> Iterator[Violation]:
    """Caps your declared limit never looked at, because they left the horizon.

    `groups.DECAP_RADIUS_MM` prunes the tether list at 5mm, and BOTH the #704
    emitter and `decap_distance` call it at that same truncation. So a cap that
    has left the radius is invisible to both: it does not set the limit, and it
    is never graded against it. Measured on splitflap_driver -- the emitted
    limit is 3.4618mm, `rules_run` goes up, the verdict is clean, and C3 sits
    19.30mm from the IC it shares a rail with.

    THE CLAIM IS COVERAGE, NOT COMPLIANCE, and the rule's name says so. It does
    NOT say the cap violates the limit: at 19mm it is far likelier a bulk or
    filter cap than a failed decoupler, and asserting a violation would
    manufacture exactly the kind of finding `_decap_derivation` refuses a
    percentile limit for. It says: your limit was derived from the caps INSIDE
    the radius, so it says nothing about this one.

    It names the IC with no hedge. That is safe because the election is
    radius-free (`groups._elect_tethers`): `beyond` carries the same owner and
    the same distance `decap_distance` would have reported at a wider radius.
    The comment that used to sit in `decap_census` claimed otherwise; it was
    wrong, and `test_792_decap_predicate` now pins the true relation.

    WARN by default, and `severity_of(..., default=WARN)` rather than
    `ctx.sev`, which hard-defaults to ERROR -- the same load-bearing bypass
    `unresolved_keepout_allows` needed. Ten tracked boards newly emit this, and
    on kit-dev it is 12 findings against a limit the emitter itself derived and
    blessed; at ERROR, `--declare-decaps` would become a flag that fails its
    own emitting board.

    Armed by `decaps.max_distance_mm`, exactly as `decap_distance` is. A board
    whose limit the emitter WITHHELD is not silently dropped: `_WITHHELD_RULE`
    routes the withholding into BOTH rules' skip reasons, and that note already
    carries the beyond count and the worst distance -- it says more than this
    rule could, because it says the board cannot support a limit at all.
    """
    spec = ctx.intent.decaps or {}
    limit = spec.get('max_distance_mm')
    if limit is None:
        return
    limit = float(limit)
    exempt = tuple(spec.get('exempt') or ())
    radius = float(spec.get('search_radius_mm', groups_mod.DECAP_RADIUS_MM))
    _near, beyond, _orphans = ctx.decap_populations(radius)
    sev = ctx.intent.severity_of('decap_ungraded', default=WARN)
    sup = ctx.superseded()
    for cap, ic, dist in beyond:
        # An author who waived a cap from the distance claim has already
        # decided about it; telling them it is also ungraded is noise about
        # their own decision. A cap a declared relation supersedes IS
        # graded -- by that relation.
        if any(fnmatch.fnmatch(cap, pat) for pat in exempt) or cap in sup:
            continue
        yield Violation(
            rule='decap_ungraded', severity=sev,
            ref=cap, block=ctx.owner.get(cap),
            message=(f"{cap} is {dist:.2f}mm from {ic}, the IC it decouples "
                     f"-- beyond the {radius:.2f}mm tether search radius, so "
                     f"decap_distance never measured it against the "
                     f"{limit:.2f}mm limit. That limit was derived from the "
                     f"caps INSIDE the radius, so it says nothing about this "
                     f"one"),
            measured={'distance_mm': round(dist, 4), 'ic': ic,
                      'search_radius_mm': round(radius, 4)},
            expected={'max_distance_mm': limit})


def _pin_gap(pin_pad, cap_fp, net_id: int) -> Optional[float]:
    """Smallest pad-edge gap from a supply pin to a cap's pad ON THAT NET.

    The cap's RAIL leg, never its nearest pad: a 0402's ground pad can sit half
    a millimetre nearer and shave the number, and the rail leg is what the rule
    is about.

    SIGNED (`legality.rect_gap` goes negative on overlap), not clamped. Clamping
    erases the overlap magnitude, and 58 of glasgow_revC's 87 tethers already
    clamp to zero under the existing centroid-to-inflated-bbox metric -- #704's
    median argument is that scar.
    """
    best = None
    for q in cap_fp.pads:
        if q.net_id != net_id:
            continue
        g = legality.rect_gap(legality.pad_rect(pin_pad), legality.pad_rect(q))
        if best is None or g < best:
            best = g
    return best


def drawn_body_rect(geom, fp_obj):
    """`(rect_in_board_coords, source)` for one part's DRAWN body.

    Lifted out of `_Ctx.body_rect` (#894) so a caller that is not grading an
    intent can have the same rect without building a `_Ctx`. `geom` is a
    `placement.body.BodyGeometry` (or None); `fp_obj` a parsed footprint.

    THE DRAWN body, `drawn_local`, and not `body_local`. #896 stores two
    ladders on purpose and says why: a courtyard "is not a body -- it is a
    body plus an assembly margin plus any shell overhang", and `body_local` is
    courtyard-FIRST, so reading it would under-state every gap by the assembly
    margin. `occupancy_local` is deliberately never used either: it is the
    body unioned with the pads, which answers "may something be seated here".

    The fall-back when the library drew NEITHER fab nor silk is the pad bbox,
    reported as source `pad_bbox`, so a reader always knows the number rests
    on copper rather than on a drawn outline.
    """
    rect_local = None if geom is None else (geom.drawn_local
                                            or geom.body_local)
    source = 'none' if geom is None else (geom.drawn_source
                                          if geom.drawn_local is not None
                                          else geom.source)
    if rect_local is None or fp_obj is None:
        return None, source
    x0, y0, x1, y1 = legality.rotate_local_bounds(
        *rect_local, fp_obj.rotation or 0.0)
    return ((fp_obj.x + x0, fp_obj.y + y0, fp_obj.x + x1, fp_obj.y + y1),
            source)


def _pads_named(fp_obj, names) -> List:
    """Every pad carrying one of `names`. A pad NUMBER is not unique.

    `Y1` on the board this rule was written for has two pads both numbered
    `3` (a crystal's two ground tabs), and `USB1` has six numbered `0` plus two
    with an empty name. A first-hit lookup would pick whichever the parser saw
    first, so the answer would depend on file order; taking every match and
    minimising over them cannot.
    """
    want = set(names)
    return [p for p in (fp_obj.pads or ()) if p.pad_number in want]


def _proximity_reach(pad, partners, net_match: bool):
    """`(gap, partner_pad, how)` for ONE subject pad -- the MINIMUM over its
    partner set, or None when that set is empty.

    SIGNED, never clamped: `rect_gap` goes negative on overlap, and erasing
    that magnitude is the scar `_pin_gap` records at the top of this file.

    `net_match` narrows the partner set to the pads on this pad's OWN net when
    any partner carries it. That is `_pin_gap`'s rule generalised, and its
    docstring is the argument verbatim: "the cap's RAIL leg, never its nearest
    pad -- a 0402's ground pad can sit half a millimetre nearer and shave the
    number". It is the one NARROWING in this rule, so it is the one place a
    finding could be manufactured; `how` goes on the wire for exactly that
    reason, so every finding is falsifiable from its own payload.
    """
    same = ([q for q in partners if q.net_id == pad.net_id and pad.net_id > 0]
            if net_match else [])
    use, how = (same, 'net') if same else (partners, 'any')
    best = None
    for q in use:
        g = legality.rect_gap(legality.pad_rect(pad), legality.pad_rect(q))
        if best is None or g < best[0]:
            best = (g, q, how)
    return best


def _arm_decap_pins(ctx) -> Optional[str]:
    """Why this BOARD cannot answer the pin question. None = it can.

    `_wants` sees only the intent. A rule the intent asked for and the board
    cannot answer would otherwise land in `rules_run`, print "N rule(s) ran, no
    violations", and make `--require-rules` EASIER to satisfy -- the exact
    vacuous pass that flag exists to catch, arriving through the mechanism that
    implements it.
    """
    caps = _decap_caps_by_net(ctx.pcb)
    if not caps:
        return ("the board carries no decoupling capacitor (no C* footprint "
                "with exactly two nets), so no supply pin can be graded "
                "against one")
    recs = ctx.supply_pins()
    if not any(r['pins'] for r in recs.values()):
        tot = {c: sum(r['counts'][c] for r in recs.values())
               for c in ('pintype', 'pinfunction', 'rail_net')}
        return (f"no supply pin on any of {len(recs)} candidate IC(s): "
                f"pintype yielded {tot['pintype']}, pinfunction "
                f"{tot['pinfunction']}, rail-net {tot['rail_net']}. The rule "
                f"is not graded and is not passed")
    return None


def rule_decap_pin_distance(ctx) -> Iterator[Violation]:
    """Decoupling caps within reach of the PIN, not of the package (#705).

    `decap_distance` measures cap CENTROID to the IC's pad bbox inflated 0.5mm,
    clamped to 0 inside. The requirement it stands in for is about the pin: a
    100nF 1.7mm from a QFN and 9mm from the IOVDD pin it decouples satisfies
    that rule and fails the spec. A downstream project hit exactly this and
    hand-wrote its own checker.

    THE INVARIANT. Compute a SUPERSET of the caps that could serve a pin, over
    a pin set whose evidence is recorded per chip, and violate only when the
    MINIMUM over that superset exceeds the limit -- abstaining, never passing,
    when either set is empty. Because the metric is a minimum, every cap added
    can only lower it, so the only way to manufacture a violation is to have
    MISSED a cap. That reduces "is this finding real?" to one auditable
    question, and the answer is forced: every two-net `C*` carrying the pin's
    exact net, either side, any distance, whoever else it also serves.

    THREE NAMES, because severity is settable per NAME and these are three
    different claims:

      decap_pin_distance           error  a DECLARED supply pin (channel 1-2)
                                          is further than the limit
      decap_pin_distance_inferred  warn   same measurement, but the pin was
                                          inferred from a net NAME
      decap_pin_uncovered          warn   a declared supply pin's rail carries
                                          no decoupling cap at all

    The split is not cosmetic. At a 3mm limit corpus-wide, channels 1-2 produce
    44 findings and channel 3 produces 70 -- one name would make ulx3s, which
    has no `pintype` at all, fail on inference. And `_uncovered` is per (IC,
    rail) rather than per pin because `haasoscope_pro_max_test` is an
    8-footprint fixture with 98 typed supply pins and ZERO capacitors; per-pin
    it would emit 98 findings that all say one thing. (In fact the whole-board
    abstention catches that board first, so it emits one skip reason instead.)

    Opt-in and with NO default: at 3mm the corpus produces 114 findings with
    worsts of 30.77 / 33.92 / 37.47mm. A shipped default would flood every
    board on day one, which is why `max_distance_mm` is opt-in too.
    """
    spec = ctx.intent.decaps or {}
    limit = spec.get('max_pin_distance_mm')
    if limit is None:
        return
    limit = float(limit)
    exempt = tuple(spec.get('exempt') or ())
    same_side = bool(spec.get('same_side'))
    by_net = _decap_caps_by_net(ctx.pcb)
    sev_inf = ctx.intent.severity_of('decap_pin_distance_inferred',
                                     default=WARN)
    sev_unc = ctx.intent.severity_of('decap_pin_uncovered', default=WARN)
    for ref, rec in sorted(ctx.supply_pins().items()):
        if not rec['pins']:
            continue
        inferred = rec['channel'] == 'rail_net'
        ic_fp = ctx.pcb.footprints.get(ref)
        ic_side = legality.footprint_side(ic_fp) if ic_fp is not None else None
        uncovered_rails = {}
        for pad, net in rec['pins']:
            # THREE states, not two, and conflating them made the rule LIE.
            # `on_rail` is the ground truth: every decoupling cap carrying this
            # net. `caps` is what the author's constraints leave usable.
            on_rail = [c for c in by_net.get(pad.net_id, ())
                       if c.reference != ref]
            caps = [c for c in on_rail
                    if not any(fnmatch.fnmatch(c.reference, pat)
                               for pat in exempt)]
            if same_side:
                # A MANUFACTURING claim, never an electrical one: the author is
                # asserting the back side is not available -- single-sided
                # assembly, a can or heatsink over it, an enclosure wall. See
                # the docs for what it costs.
                caps = [c for c in caps
                        if legality.footprint_side(c) == ic_side
                        or legality.footprint_has_through_pads(c)]
            if not caps:
                if not on_rail:
                    # Genuinely uncovered: no decoupling cap on this net
                    # anywhere. A design fact, not a placement failure (#705's
                    # own item 4), reported once per (IC, rail). Unreachable
                    # for an INFERRED pin, whose channel requires the net to
                    # carry a cap already.
                    if not inferred:
                        uncovered_rails.setdefault(net, (pad.pad_number, 0, ()))
                    continue
                # The rail HAS caps and the author's own constraints removed
                # every one. Silence here was a real defect, in two directions:
                # it printed "has NO decoupling capacitor on it anywhere on the
                # board" for a rail carrying two (glasgow /VCCPLL0 under
                # `same_side`), and for an INFERRED pin it printed nothing at
                # all -- so turning a STRICTER constraint on took ulx3s from 39
                # findings to 4 while the evidence line did not move.
                #
                # The invariant that must hold is about COVERAGE, not counts:
                # no (IC, rail) pair reported without the filter may go
                # unreported with it. Raw counts legitimately fall, because a
                # per-pin distance finding collapses into ONE per-rail
                # exclusion -- ulx3s 39 findings over 14 rails becomes 15 over
                # 15 rails. Asserting the count would have been asserting the
                # granularity.
                excluded = tuple(sorted(c.reference for c in on_rail))
                uncovered_rails.setdefault(
                    net, (pad.pad_number, len(on_rail), excluded))
                continue
            gaps = [(g, c.reference) for g, c in
                    ((_pin_gap(pad, c, pad.net_id), c) for c in caps)
                    if g is not None]
            if not gaps:
                continue
            gap, who = min(gaps)
            if gap <= limit + legality.EPS:
                continue
            cap_fp = ctx.pcb.footprints.get(who)
            # Constructed in two branches rather than with a conditional
            # `rule=`, because `test_every_violation_rule_name_is_settable`
            # scans this file for LITERAL `rule='...'` and asserts exact
            # set equality with `_SEVERITY_KEYS` in both directions. A
            # name hidden behind an expression is a settable name the
            # gate cannot see -- and it caught this one.
            payload = dict(
                severity=(sev_inf if inferred
                          else ctx.sev('decap_pin_distance')),
                ref=ref, block=ctx.owner.get(ref),
                message=(f"{ref} pin {pad.pad_number} ({net}) is "
                         f"{gap:.2f}mm from {who}, the nearest decoupling cap "
                         f"on that rail (limit {limit:.2f}mm)"
                         + (" -- pin inferred from the net NAME, not from a "
                            "pintype or pinfunction" if inferred else "")),
                measured={'gap_mm': round(gap, 4), 'pad': pad.pad_number,
                          'net': net, 'cap': who, 'channel': rec['channel'],
                          'caps_on_rail': len(caps),
                          'pins_on_rail': sum(
                              1 for _p, n in rec['pins'] if n == net),
                          'ic_side': ic_side,
                          'cap_side': (legality.footprint_side(cap_fp)
                                       if cap_fp is not None else None)},
                expected={'max_pin_distance_mm': limit})
            if inferred:
                yield Violation(rule='decap_pin_distance_inferred',
                                **payload)
            else:
                yield Violation(rule='decap_pin_distance', **payload)
        for net, (pad_no, n_on_rail, excluded) in sorted(
                uncovered_rails.items()):
            # The MESSAGE distinguishes the two causes, because they call for
            # opposite actions: add a cap, versus relax a constraint you set.
            # `measured` carries the excluded refs so the finding is falsifiable
            # from its own payload -- it was not, and that is how the false
            # "anywhere on the board" survived.
            if n_on_rail:
                msg = (f"{ref} rail {net} (e.g. pin {pad_no}) carries "
                       f"{n_on_rail} decoupling cap(s) "
                       f"({', '.join(excluded)}), and this intent's own "
                       f"constraints exclude every one of them")
            else:
                msg = (f"{ref} rail {net} (e.g. pin {pad_no}) has NO "
                       f"decoupling capacitor on it anywhere on the board")
            yield Violation(
                rule='decap_pin_uncovered', severity=sev_unc,
                ref=ref, block=ctx.owner.get(ref),
                message=msg,
                measured={'net': net, 'pad': pad_no,
                          'channel': rec['channel'],
                          'caps_on_rail': n_on_rail,
                          'caps_usable': 0,
                          'excluded': list(excluded),
                          'pins_on_rail': sum(
                              1 for _p, n in rec['pins'] if n == net)},
                expected={'caps_on_rail': '>= 1'} if not n_on_rail
                else {'caps_usable': '>= 1'})


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
    # Declared edge connectors inside their own overhang band are off the
    # board CORRECTLY (the promise in `rule_edge_connector`'s docstring, now
    # kept here): they leave `oob_count` before it meets the budget. The raw
    # count stays in `ctx.legality['oob_count']` -- it is the optimizer's own
    # number and `test_legality_numbers_are_the_optimizers_own` pins it --
    # and the exemption is published beside it as `oob_count_exempt`.
    # `oob_amount` is NOT reduced: an author who budgets millimetres of
    # overhang is budgeting the connectors too, and no run has asked for
    # the split; say so here rather than guess.
    exempt = ctx.oob_exempt()
    ctx.legality['oob_count_exempt'] = len(exempt)
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
        measured = {key: round(float(got), 4)}
        note = ''
        if key == 'oob_count' and exempt:
            raw = got
            got = raw - len(exempt)
            measured = {key: int(got), 'oob_count_raw': int(raw),
                        'exempt': sorted(exempt)}
            note = (f" ({len(exempt)} declared edge connector(s) within "
                    f"their overhang band exempt: {', '.join(sorted(exempt))})")
        if got > lim + legality.EPS:
            yield Violation(
                rule='legality', severity=ctx.sev('legality'),
                message=(f"{label}: {got:.3f} exceeds the declared budget "
                         f"{lim:.3f}{note}"),
                measured=measured, expected={key: lim})


def rule_proximity(ctx) -> Iterator[Violation]:
    """These two named parts, no further apart than `max_mm` (#902).

    The one class of constraint the NETLIST implies and nothing here could
    read: a 3mm crystal loop and a 30mm one have identical connectivity, so no
    instrument that reads the board can tell them apart. `decap_distance`
    cannot stand in for it -- a decap is syntactic (`ref` starts with `C`,
    bridges exactly two nets) and its partner must carry >= 4 copper pads, so a
    3-pad SOT89 regulator can never be a tether target at ANY radius. Measured
    on the board this was written for, the election gives `C1 -> USB1 2.03mm`
    and `C3 -> U1 1.83mm`: the regulator's own bulk caps, graded against a USB
    socket and a bridge IC.

    THE INVARIANT. For each SUBJECT pad the claim declares, take the MINIMUM
    over a partner set that is a SUPERSET of the pads which could satisfy it,
    and violate only when that minimum exceeds `max_mm`. Because the metric is
    a minimum, every partner added can only lower it, so the only way to
    manufacture a violation is to have MISSED a partner. The quantifiers are
    `for all subject pads, there exists a partner pad` -- the same shape as
    `decap_pin_distance` and for the same reason: a crystal whose XTAL_IN leg
    is 30mm away must not be blessed because XTAL_OUT is 1mm away.

    TWO ARITIES, because they are two claims:

      `pads[ref]` declared   per-PIN reach. Each declared subject pad against
                             the partner's pads, narrowed to its own net when
                             any partner carries it (`_proximity_reach`).
      `pads[ref]` omitted    part-to-part ADJACENCY. Every pad against every
                             pad, no net matching, one finding per claim.

    All pads rather than shared-net pads in the second arity, and that is the
    invariant again: all-pads is the SUPERSET, so the minimum can only fall.
    Shared-net is a subset and can RAISE the number, which is the one direction
    that manufactures findings. And with no `pads` the author stated a GEOMETRY
    claim; net-matching it would silently make it an electrical one.

    TOTAL OVER ITS CLAIMS, which is why there is no `_ARM` entry. Every claim
    yields exactly one of: a measured pass (silence), a `proximity` violation,
    a `proximity_unresolved` violation, or an abstention on `ctx.abstained`. A
    branch that `continue`d without one of those would put this rule in
    `rules_run` having graded a claim it never measured -- the vacuous pass
    `--require-rules` exists to catch, arriving through the mechanism that
    implements it.
    """
    sev_unres = ctx.intent.severity_of('proximity_unresolved', default=ERROR)
    for i, claim in enumerate(ctx.intent.proximity):
        ref, near = str(claim['ref']), str(claim['near'])
        limit = float(claim['max_mm'])
        basis = claim.get('basis', _PROXIMITY_DEFAULT_BASIS)
        spec = claim.get('pads') or {}
        where = f"proximity[{i}]"
        # The abstention key carries the INTENT ROW INDEX as well as the two
        # refs, so a consumer reporting per-clause coverage can attribute an
        # abstention to the claim that caused it. The index is not decoration:
        # a reference may legally contain `~` (`disambiguate_references`
        # produces `TP4~2`, and this very board parses `Ref*~2`), so a bare
        # `ref~near` makes a row `A~B` near `C` and a row `A` near `B~C` share
        # one string -- and `ctx.abstained` is a DICT, so one of the two
        # abstentions would silently disappear. The index is unique by
        # construction, and a consumer resolves it against the intent's own
        # row to recover the pair exactly.
        akey = f"proximity[{i}:{ref}~{near}]"

        a_fp = ctx.pcb.footprints.get(ref)
        b_fp = ctx.pcb.footprints.get(near)
        missing = [r for r, f in ((ref, a_fp), (near, b_fp)) if f is None]
        if missing:
            # A VIOLATION, not a skip: `rule_edge_connector` reports an
            # unmatched brief ref at error severity and `compile_brief` KEEPS
            # such a ref precisely so it does. Dropping it would make a typo
            # grade clean, which is `block_unresolved`'s failure one level
            # over -- and an ARM skip would lose the ref's name entirely.
            yield Violation(
                rule='proximity_unresolved', severity=sev_unres,
                ref=ref, block=ctx.owner.get(ref),
                message=(f"{where}: {' and '.join(missing)} "
                         f"{'are' if len(missing) > 1 else 'is'} not on this "
                         f"board, so {ref} near {near} cannot be measured"),
                measured={'missing': missing, 'near': near},
                expected={'max_mm': limit})
            continue

        if basis == 'body':
            a_rect, a_src = ctx.body_rect(ref)
            b_rect, b_src = ctx.body_rect(near)
            if a_rect is None or b_rect is None:
                nogeom = [r for r, rect in ((ref, a_rect), (near, b_rect))
                          if rect is None]
                # The READER's own failure is reported as itself. Falling
                # through to "draws no body" would state a false fact about
                # the author's board and send them to fix a footprint that is
                # fine -- on the fixture here both parts draw a body and carry
                # 4 and 20 pads.
                why = (f"placement.body could not be read for this board "
                       f"({ctx._bodies_error}), so no body claim can be "
                       f"measured -- this is a failure of the geometry "
                       f"reader, not of {' or '.join(nogeom)}"
                       if ctx._bodies_error else
                       f"{' and '.join(nogeom)} draws no body and has no "
                       f"pads, so placement.body answers source 'none' -- "
                       f"there is no geometry to measure. Name pads and use "
                       f"basis pad_edge, or drop the claim")
                ctx.abstain(f"{akey}.basis", why)
                continue
            gap = legality.rect_gap(a_rect, b_rect)
            # Recorded before the branch, as in the pad_edge arity below (#894).
            ctx.proximity_measured.append({
                'claim': akey, 'index': i, 'ref': ref, 'near': near,
                'gap_mm': round(gap, 4), 'limit_mm': limit,
                'passes': gap <= limit + legality.EPS,
                'pad': None, 'near_pad': None, 'net': None, 'paired_by': None,
                'pads_basis': 'body', 'basis': 'body',
                'basis_source': a_src, 'near_basis_source': b_src})
            if gap <= limit + legality.EPS:
                continue
            yield Violation(
                rule='proximity', severity=ctx.sev('proximity'),
                ref=ref, block=ctx.owner.get(ref),
                message=(f"{ref} is {gap:.2f}mm from {near}, body to body, "
                         f"past the declared {limit:.2f}mm "
                         f"(bodies from {a_src}/{b_src})"),
                measured={'gap_mm': round(gap, 4), 'near': near,
                          'basis': 'body', 'basis_source': a_src,
                          'near_basis_source': b_src,
                          'pads_basis': 'body'},
                expected={'max_mm': limit})
            continue

        declared = bool(spec.get(ref))
        subject = (_pads_named(a_fp, spec[ref]) if declared
                   else list(a_fp.pads or ()))
        partners = (_pads_named(b_fp, spec[near]) if spec.get(near)
                    else list(b_fp.pads or ()))
        # A NAME that matches nothing is unresolved, not clean -- and it is
        # reported PER NAME, not only when every name misses. The first
        # version fired on `not got`, so `pads: {'Y1': ['2', '7']}` graded
        # CLEAN on the survivor while `'7'` vanished: the claim then measured
        # a strictly smaller subject set than it declared, which is the very
        # failure the loader cites when it refuses an integer pad number
        # ("would match no pad, measure nothing, and grade clean"). It also
        # falsified this rule's own invariant, which is quantified over the
        # pads the claim DECLARES and not over the ones that happened to
        # resolve.
        #
        # The loader can refuse a pad number of the wrong TYPE; only here,
        # holding a board, can "pad 7 of a 4-pad part" be seen at all.
        blind = False
        for who, names, got in ((ref, spec.get(ref), subject),
                                (near, spec.get(near), partners)):
            if not names:
                continue
            have = {p.pad_number for p in got}
            missing = [n for n in names if n not in have]
            if not missing:
                continue
            blind = True
            yield Violation(
                rule='proximity_unresolved', severity=sev_unres,
                ref=ref, block=ctx.owner.get(ref),
                message=(f"{where}: {who} has no pad numbered "
                         f"{', '.join(repr(n) for n in missing)}"
                         + (f" (it has {', '.join(sorted(repr(p.pad_number) for p in (b_fp if who == near else a_fp).pads or ()))})"
                            if len(missing) == len(names) else
                            f" -- {len(names) - len(missing)} of "
                            f"{len(names)} named pad(s) resolved, so the "
                            f"claim would measure less than it declares")),
                measured={'unresolved_ref': who, 'pads': list(missing),
                          'declared_pads': list(names),
                          'resolved_pads': len(names) - len(missing),
                          'near': near},
                expected={'max_mm': limit})
        if blind:
            # Not measured at all: a distance taken over a subject set smaller
            # than the claim declares is a number about a different claim.
            continue
        if not subject or not partners:
            ctx.abstain(
                f"{akey}.pads",
                f"{ref if not subject else near} has no pads at all, so there "
                f"is nothing to measure pad edge to pad edge. Use basis body, "
                f"or drop the claim")
            continue

        # PER SUBJECT PAD when the claim declares them; ONCE for the pair when
        # it does not -- because those are the two claims the two arities
        # make. `decap_pin_distance` reports per PIN for the same reason: "the
        # crystal is too far" is one fact, but "XTAL_IN is 4.74mm away AND
        # XTAL_OUT is 2.52mm away" is two, and collapsing them to the worst
        # hides a leg the author still has to move. Measured on the unplaced
        # board: reporting only the worst turned three failing pads into two
        # findings. With no declared pads there is no pin to name, so the
        # part-adjacency arity reports once.
        reaches = []
        for pad in subject:
            got = _proximity_reach(pad, partners, net_match=declared)
            if got is not None:
                reaches.append((got[0], pad, got[1], got[2]))
        # No `if not reaches` arm: `partners` is proven non-empty by the guard
        # above and `_proximity_reach` falls back to it, so it never returns
        # None here. An arm that cannot run is not a safety net -- it is a
        # claim about the code that no test can check, and this one survived
        # being replaced by `raise AssertionError`.
        if not declared:
            reaches = [min(reaches, key=lambda t: t[0])]
        for gap, pad, partner, how in reaches:
            net = pad.net_name or ''
            # RECORDED BEFORE the pass/fail branch, so a clause that HOLDS
            # publishes its number too (#894). The `continue` below is what
            # makes this rule silent on a pass; without this line the gap a
            # score wants to compare lap-to-lap is computed and discarded.
            ctx.proximity_measured.append({
                'claim': akey, 'index': i, 'ref': ref, 'near': near,
                'gap_mm': round(gap, 4), 'limit_mm': limit,
                'passes': gap <= limit + legality.EPS,
                'pad': pad.pad_number, 'near_pad': partner.pad_number,
                'net': net or None, 'paired_by': how,
                'pads_basis': 'declared' if declared else 'part',
                'basis': 'pad_edge',
                'subject_pads': len(subject), 'near_pads': len(partners)})
            if gap <= limit + legality.EPS:
                continue
            yield Violation(
                rule='proximity', severity=ctx.sev('proximity'),
                ref=ref, block=ctx.owner.get(ref),
                message=(f"{ref} pad {pad.pad_number} is {gap:.2f}mm from "
                         f"{near} pad {partner.pad_number}"
                         + (f" on {net}" if how == 'net' and net else '')
                         + f", past the declared {limit:.2f}mm proximity "
                           f"limit"),
                measured={'gap_mm': round(gap, 4), 'near': near,
                          'pad': pad.pad_number,
                          'near_pad': partner.pad_number,
                          'net': net or None, 'paired_by': how,
                          'pads_basis': 'declared' if declared else 'part',
                          'basis': 'pad_edge',
                          'subject_pads': len(subject),
                          'near_pads': len(partners)},
                expected={'max_mm': limit})


def rule_pins_to_edge(ctx) -> Iterator[Violation]:
    """A part whose pad row faces the board outline with nothing beyond it.

    The facing criterion of the boundary review, as a number (run 26: a SOT-89
    regulator seated with all three pins 0.40 mm from the north edge, every
    net forced under its own body, and a review that wrote PASS because
    nothing measured it). The geometry is `placement.edge_facing`, shared
    with `placement_score.edge_facing` and the seeder, so the rule and the
    term cannot disagree about a pad.

    ALWAYS WARN, whatever the configured severity, like the
    `connector_affinity` arm of `rule_edge_connector`: this is a judgement
    for criterion 4 of the boundary review, and a legitimately edge-facing
    row exists (a test-point header, a part whose only partner IS the edge
    strip), so the reviewer disposes and the gate does not. The exclusion is
    the intent's `edge_claims()` -- a declared connector's mating row SHOULD
    face the edge -- and that is also why `_wants` arms the rule only on an
    intent that declares `edge_connectors`: without the declaration the rule
    would name every connector on the board.
    """
    from .edge_facing import EDGE_MM, count_pads_to_edge, part_inputs
    excluded = {c['ref'] for c in ctx.intent.edge_claims()}
    bounds = ctx.outline_bounds
    if not bounds:
        return
    for ref in sorted(ctx.pcb.footprints or {}):
        if ref in excluded:
            continue
        inputs = part_inputs(ctx.pcb, ref)
        if inputs is None:
            continue
        pads, rect, partners, centre, pitch = inputs
        if len(pads) < PINS_TO_EDGE_MIN_PADS:
            continue
        r = count_pads_to_edge(pads, rect, bounds, partners, centre,
                               pitch=pitch)
        if r['to_edge'] <= 0:
            continue
        faces = sorted(r['faces'])
        gap = min(r['gaps'][f] for f in faces)
        yield Violation(
            rule='pins_to_edge', severity=WARN, ref=ref,
            message=(f"{ref}: {r['to_edge']} of {r['pads']} connected pads sit "
                     f"on a row facing the {'/'.join(faces)} edge "
                     f"({gap:.2f}mm) with no partner beyond -- their nets can "
                     f"only leave along the edge strip or under the part"),
            measured={'pads_to_edge': r['to_edge'], 'pads': r['pads'],
                      'faces': faces, 'gap_mm': round(gap, 4),
                      'edge_mm': EDGE_MM},
            expected={'pads_to_edge': 0})


#: `rule_pins_to_edge` grades parts with at least this many CONNECTED pads
#: -- the same floor `placement_score.EDGE_FACING_MIN_PADS` applies, read
#: from the geometry core both share (the rule module does not import the
#: score module).
from .edge_facing import MIN_PADS as PINS_TO_EDGE_MIN_PADS  # noqa: E402


RULES = (
    ('envelope', rule_envelope),
    ('zone_containment', rule_zone_containment),
    ('zone_side', rule_zone_side),
    ('assembly_side', rule_assembly_side),
    ('zone_exclusive', rule_zone_exclusive),
    ('keepout', rule_keepout),
    ('edge_connector', rule_edge_connector),
    ('decap_distance', rule_decap_distance),
    ('decap_ungraded', rule_decap_ungraded),
    ('decap_pin_distance', rule_decap_pin_distance),
    ('proximity', rule_proximity),
    ('must_lock', rule_must_lock),
    ('legality', rule_legality),
    ('pins_to_edge', rule_pins_to_edge),
)

#: Rules whose violations are raised OUTSIDE the `RULES` loop, and so have no
#: rule function to be enumerated from: the two self-contradiction findings in
#: `validate_intent`, the unresolved-block finding in `resolve_blocks`, and the
#: zone-inside-a-keep-out contradiction `resolve_intent_gate` raises (#702).
#: Kept as a named set rather than folded into `_SEVERITY_KEYS` by hand, so the
#: next reader can see which names are the exception and why.
_NON_RULE_SEVERITIES = frozenset({
    'intent_zone_outside_envelope', 'intent_zone_overlap', 'block_unresolved',
    'intent_zone_in_keepout', 'keepout_allow_unresolved',
    # #705's two siblings. They are raised by `rule_decap_pin_distance`
    # rather than outside the loop, so this set is now "settable names that
    # are not RULES entries" -- a rule may raise more than one FINDING, and
    # severity is settable per finding because a channel-3 inference and a
    # declared pin are different claims about the same measurement.
    'decap_pin_distance_inferred', 'decap_pin_uncovered',
    # #902. Raised BESIDE `proximity`'s own name rather than outside the
    # loop: a named ref or pad number the board does not have is a
    # different CLAIM from a distance, and severity is settable per name,
    # so a DNP-variant board legitimately missing a part can demote the
    # resolution finding without demoting the distance one.
    'proximity_unresolved',
    # #959 (#1001). Raised by `grade` itself when `mechanical.json` is read:
    # a part that moved off (or turned away from) its declared mechanical
    # pose. WARN for a move the grade-time anchor already reports at ERROR
    # (zone_containment on `mech:<ref>`); ERROR for a TURN, which no anchor
    # sees, and for any drift of a pad-less ref, which has no anchor.
    'mechanical_drift',
    # #959 (#1000): raised BESIDE `edge_connector` -- the face a declared
    # viewing side puts a connector on. Settable only DOWN (an intent may
    # say `warn`; `error` is refused at load), because it is advisory by
    # design.
    'edge_connector_side',
    # #959 (#998). Raised by `plan_check`, the zone plan checked against
    # itself and the board BEFORE the first pose write, and (the first) by
    # `grade` too. Every ERROR among them is sound: the plan cannot be seeded
    # or satisfied however the parts are arranged. The `*_crowded` ones are
    # the WARN halves -- the same quantities with a margin added.
    'plan_zone_exclusive_unsatisfiable', 'block_glob_literal',
    'plan_fixed_outside_zone', 'plan_zone_overfull', 'plan_zone_crowded',
    'plan_edge_overfull', 'plan_edge_crowded', 'plan_board_overfull',
    'plan_board_crowded',
    # Row 8: two FILE-locked parts overlapping. A WARN per pair; an ERROR
    # only when their overlap alone exceeds a DECLARED overlap budget.
    'plan_fixed_overlap', 'plan_fixed_overlap_budget'})

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
    'assembly_side': 'the intent declares no assembly.sides',
    'zone_exclusive': 'no block is marked exclusive',
    'keepout': 'the intent declares no keepouts',
    'edge_connector': 'the intent declares no edge_connectors',
    'decap_distance': 'the intent declares no decaps.max_distance_mm',
    'decap_ungraded': 'the intent declares no decaps.max_distance_mm',
    'decap_pin_distance': 'the intent declares no decaps.max_pin_distance_mm',
    'proximity': 'the intent declares no proximity claims',
    'must_lock': 'the intent declares no must_lock patterns',
    'legality': 'the intent declares no legality_budget',
    # The rule's exclusion list IS the declaration: without it every
    # connector on the board would be named for facing the edge it mates at.
    'pins_to_edge': 'the intent declares no edge_connectors',
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
#: The first element is a TUPLE of rule names since #794, because
#: `decaps.max_distance_mm` now disarms two rules and a one-rule mapping
#: would have attached the withholding note to only one of them -- leaving
#: the other reading a bare skip reason that does not say the emitter tried.
#: A second table entry under an invented key name would have been worse:
#: `test_an_unmapped_withheld_key_still_abstains_and_blames_no_rule` exists
#: to catch exactly that, and a made-up key would land in `budget_abstained`.
_WITHHELD_RULE = {
    'overlap_area': (('legality',),
                     lambda i: 'overlap_area' in (i.legality_budget or {})),
    'oob_count': (('legality',),
                  lambda i: 'oob_count' in (i.legality_budget or {})),
    'oob_amount': (('legality',),
                   lambda i: 'oob_amount' in (i.legality_budget or {})),
    'decaps.max_distance_mm': (
        ('decap_distance', 'decap_ungraded'),
        lambda i: (i.decaps or {}).get('max_distance_mm') is not None),
}


#: A rule the intent ASKED for that this BOARD cannot answer -> the reason.
#: `_wants` sees only the intent; this sees the board (#705).
#:
#: Needed because `grade()` decides skip-vs-run BEFORE the rule runs, so a rule
#: that would run and then measure nothing lands in `rules_run` and prints
#: "N rule(s) ran, no violations". That makes `--require-rules` EASIER to
#: satisfy -- the vacuous pass it exists to catch, arriving through the
#: mechanism that implements it. `orangecrab_ext_pll` is the live case: 28
#: candidate ICs, zero supply pins on any channel.
#:
#: Deliberately NOT routed through `budget_abstained`, which means "the EMITTER
#: could not derive this key" -- a property of the intent, computed with no
#: board. This is the opposite, and overloading that key would make
#: `budget_abstained_keys` mean two things.
def _arm_decap_superseded(ctx) -> Optional[str]:
    """The decap distance rules have nothing of their own to grade when a
    declared relation supersedes EVERY cap with an IC on its rail (#959):
    armed, they would run and measure nothing, so they abstain and say why."""
    spec = ctx.intent.decaps or {}
    r = float(spec.get('search_radius_mm', groups_mod.DECAP_RADIUS_MM))
    near, beyond, _orph = ctx.decap_populations(r)
    caps = ({c for cs in near.values() for c, _d in cs}
            | {c for c, _ic, _d in beyond})
    sup = ctx.superseded()
    if caps and caps <= set(sup):
        return (f"every cap with an IC on its rail ({len(caps)}) is graded "
                f"by a declared proximity relation instead")
    return None


_ARM = {'decap_pin_distance': _arm_decap_pins,
        'decap_distance': _arm_decap_superseded,
        'decap_ungraded': _arm_decap_superseded}


#: The marker `grade()` appends to a `_SKIP_REASON` when a WITHHELD key
#: disarmed the rule. A skip carrying it is an abstention, not a "nobody
#: asked".
_WITHHELD_MARK = 'the emitter WITHHELD'


def _is_not_asked(rule: str, reason: str) -> bool:
    """Is this `rules_skipped` entry the honest kind -- nobody declared it?

    ONE home for the distinction, because `rules_skipped` carries two opposite
    things and reading it as one number is how the #713 census first reported
    21 of 22 boards instead of 5. A `_SKIP_REASON` value means the intent
    declared nothing, which must not make a verdict incomplete; anything else
    is an `_ARM` reason ("the intent asked, this board cannot answer") or a
    withholding note, both of which must.

    Keyed on the RULE, not on the reason string alone. Matching against
    `_SKIP_REASON.values()` meant an `_ARM` reason that ever happened to equal
    some OTHER rule's skip reason would read as "nobody asked" -- a real
    abstention silently downgraded to a pass, which is the dangerous
    direction. The strings are distinct today; this makes them not have to be.

    The `_WITHHELD_MARK` conjunct is belt over brace: `grade()` builds a
    withheld reason by APPENDING to `_SKIP_REASON[name]`, so the decorated
    string can never equal the bare one and the first test already fails. It
    is kept because if it ever does fire, it fires toward "abstention", which
    is the safe direction.
    """
    return reason == _SKIP_REASON.get(rule) and _WITHHELD_MARK not in reason


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
    if rule == 'assembly_side':
        # ANY declared value arms it, 'both' included -- measured, and the
        # measurement reversed the obvious choice. Skipping on 'both' looks
        # like the honest "this cannot fire, so do not claim to have graded
        # it", but the intent DECLARES the key, so a skip lands in the
        # abstention channel as "declared and ungraded". Measured with
        # `_wants` hand-edited to skip: `713_abstention_census` reports
        # `rules_skipped_arm 0 -> 7` and `boards_clean_but_ungraded 0 -> 2`
        # (kit-dev-coldfire-xilinx_5213 and sonde_u) -- an emitted intent
        # declaring a key its own grade then abstains on.
        #
        # (An earlier version of this comment claimed the skip took
        # glasgow_revC and orangecrab_ext_pll from `pass: true` to `false`.
        # That was read off a re-record of a baseline that had gone stale for
        # unrelated reasons four days earlier; a review re-ran the census at
        # the base commit with none of this code present and found those
        # boards already failing. The decision stands, the mechanism above is
        # the measured one.)
        #
        # And running it is not vacuous. 'both' is a real policy -- the fab
        # will populate both faces -- and "every part is on a declared face"
        # is a true answer to a real question. It simply cannot fail, the way
        # `zone_containment` cannot fail on a board whose parts are all inside
        # their zones.
        return (intent.assembly or {}).get('sides') in _ASSEMBLY_SIDES
    if rule == 'zone_exclusive':
        return any(z.exclusive and z.rect is not None for z in intent.blocks)
    if rule == 'keepout':
        return bool(intent.keepouts)
    if rule == 'edge_connector':
        return bool(intent.edge_connectors)
    if rule == 'decap_pin_distance':
        return (intent.decaps or {}).get('max_pin_distance_mm') is not None
    if rule in ('decap_distance', 'decap_ungraded'):
        # ONE arm for both: they divide one population, so a board that
        # arms one and not the other would report a partition with a
        # missing half and no way to see that it was missing.
        return (intent.decaps or {}).get('max_distance_mm') is not None
    if rule == 'proximity':
        return bool(intent.proximity)
    if rule == 'must_lock':
        return bool(intent.must_lock)
    if rule == 'legality':
        return bool(intent.legality_budget)
    if rule == 'pins_to_edge':
        # Armed by the edge-connector DECLARATION, which is the rule's
        # exclusion list: without it the rule would name every connector on
        # the board for facing the edge it mates at.
        return bool(intent.edge_connectors)
    return True


# --------------------------------------------------------------------------
# the rule roster (#959, #997): which rules a plan leaves dark, and whether
# anything answers for that
# --------------------------------------------------------------------------

#: The key an author declares to ARM each rule -- the thing a refusal names.
#: Pinned against `_wants` rule by rule (tests/test_959_rule_roster.py), so a
#: change to what arms a rule cannot leave this table naming the old key.
_ARMING_KEY = {
    'envelope': 'envelope.rect',
    'zone_containment': 'blocks[].zone',
    'zone_side': 'blocks[].side',
    'assembly_side': 'assembly.sides',
    'zone_exclusive': 'blocks[].exclusive (on a zoned block)',
    'keepout': 'keepouts[]',
    'edge_connector': 'edge_connectors[]',
    'decap_distance': 'decaps.max_distance_mm',
    'decap_ungraded': 'decaps.max_distance_mm',
    'decap_pin_distance': 'decaps.max_pin_distance_mm',
    'proximity': 'proximity[]',
    'must_lock': 'must_lock[]',
    'legality': 'legality_budget',
    'pins_to_edge': 'edge_connectors[]',
}

#: What each rule reports at when the intent's `severity` map says nothing.
#: One table, because "is this rule advisory" is what the roster refuses on
#: and the answer lived in three places: `severity_of`'s ERROR default, two
#: rules passing `default=WARN`, and one hard-wired WARN. Pinned against the
#: severity each rule actually EMITS by a test that fires every rule, so the
#: table cannot drift from the rules without failing.
_RULE_DEFAULT_SEVERITY = dict({name: ERROR for name, _ in RULES},
                              assembly_side=WARN, decap_ungraded=WARN,
                              pins_to_edge=WARN)

#: A rule whose severity the intent's map cannot change, because the rule
#: ignores the map (`rule_pins_to_edge` always yields WARN).
_FORCED_SEVERITY = {'pins_to_edge': WARN}

#: Rules no board fact can arm or excuse, so they are REPORTED dark and never
#: refused. Measured before this was built (#959 Phase 0, P4): `emit_intent`
#: writes neither `exclusive` nor `proximity`, so both were dark on 22 of 22
#: corpus boards and on both of run 29's zone plans. Refusing them would have
#: opened every plan with the same two boilerplate dispositions, and a refusal
#: everybody answers the same way carries no signal.
_POLICY_RULES = {
    'proximity': ('a design relation between two named parts; nothing on the '
                  'board, and nothing in a design brief, says which parts '
                  'must be near which'),
    'zone_exclusive': ('a policy about who may enter a zone; nothing on the '
                       'board says that a zone is reserved'),
}


def _brief_claims(rule: str, brief_fragment) -> List[str]:
    """What the design BRIEF declares that `rule` grades, by name (#959).

    A brief-declared requirement is not the roster's to excuse: a plan that
    leaves it out has DROPPED a declaration, and P1's brief-clause check
    refuses that by clause id -- answered by carrying the clause, or by
    `--waive brief-clause:<id>:<why>`, the same spelling P-close takes. So
    the roster reports such a rule `uncovered` rather than asking for a
    second, rule-level answer to the same question. (The Phase-1 verifier
    measured the gap this closes: a plan dropping all four of fixture 902's
    brief proximity claims, or splitflap's 17 brief edge claims, passed P1.)"""
    frag = brief_fragment or {}
    if rule == 'proximity':
        return [f"{p.get('ref')}~{p.get('near')}"
                for p in frag.get('proximity') or ()]
    if rule == 'keepout':
        return [str(k.get('name')) for k in frag.get('keepouts') or ()]
    if rule in ('edge_connector', 'pins_to_edge'):
        return [str(c.get('ref')) for c in frag.get('edge_connectors') or ()
                if c.get('edge')]
    return []

#: Part classes whose presence makes the edge rules applicable. Strict on
#: purpose: `connector_affinity` (a header, a JST) makes no edge claim, and
#: counting it would make the edge rules applicable on nearly every board.
_EDGE_CLAIM_CLASSES = ('edge_receptacle', 'edge_actuator')


def _applicability(rule: str, intent: Intent, pcb_data, ctx, census,
                   brief_fragment) -> Tuple[bool, str]:
    """Could this rule apply to THIS board? `(applicable, reason)`.

    Conservative: applicable unless a board fact proves otherwise, because the
    direction a wrong answer errs in matters -- a false "not applicable"
    excuses a rule silently, a false "applicable" costs one written sentence.
    Reads board facts, never the intent's own claims about the board: an
    intent that says `assembly.sides: "F"` is a claim, and letting it exempt
    `zone_side` would let the plan excuse itself.
    """
    if rule == 'zone_side':
        faces = sorted({legality.footprint_side(fp)
                        for fp in (pcb_data.footprints or {}).values()
                        if fp.pads})
        if len(faces) < 2:
            return False, (f"every part with pads is on "
                           f"{faces[0] if faces else 'no'} face, so no block "
                           f"can be on the wrong one")
        return True, "the board carries parts on both faces"
    if rule == 'keepout':
        return False, ("nothing declares a keep-out: the brief (if any) names "
                       "none, and the board file has none this intent can "
                       "grade")
    if rule in ('edge_connector', 'pins_to_edge'):
        from .part_class import classify_part
        refs = sorted(r for r, fp in (pcb_data.footprints or {}).items()
                      if classify_part(fp, r).name in _EDGE_CLAIM_CLASSES)
        if not refs:
            return False, ("no part classifies as an edge receptacle or "
                           "actuator, so nothing has an edge to claim")
        return True, (f"{len(refs)} part(s) classify as edge receptacles or "
                      f"actuators ({', '.join(refs[:6])}"
                      + (', ...' if len(refs) > 6 else '') + ")")
    if rule in ('decap_distance', 'decap_ungraded'):
        # tethered + beyond-radius is every cap with an IC on its rail. The
        # SPLIT between the two moves with the poses; the SUM does not, so
        # this reads the same on a pile as on the placed board (measured on
        # 24 rows in #959 Phase 0).
        scope = int(census.get('tethers', 0)) + int(
            census.get('beyond_radius', 0))
        if not scope:
            return False, ("no decoupling cap shares a rail with any IC, so "
                           "no cap has anything to be near")
        if ctx is not None and _arm_decap_superseded(ctx) is not None:
            return False, _arm_decap_superseded(ctx)
        return True, (f"{scope} decoupling cap(s) share a rail with an IC")
    if rule == 'decap_pin_distance':
        why = _arm_decap_pins(ctx) if ctx is not None else None
        if why is not None:
            return False, why
        return True, "the board has supply pins and caps on their rails"
    if rule == 'must_lock':
        return False, ("must_lock is a plan's choice, not a requirement no "
                       "board fact can excuse: the design brief never "
                       "compiles one (filling it made place_seed --repair "
                       "lift the user's own locks, docs/design-brief.md), so "
                       "leaving it dark owes nothing")
    if rule == 'envelope':
        return True, "the board has an outline, and it bounds every placement"
    if rule == 'zone_containment':
        return True, "every movable block can be given a zone"
    if rule == 'assembly_side':
        return True, "every board has faces for the fab to populate"
    if rule == 'legality':
        return True, "every placement has a legality to measure"
    return True, "no board fact excuses it"


def _gating(rule: str, intent: Intent, ctx) -> bool:
    """Does a violation of this rule fail the grade on THIS board?

    The table default decides, plus one direction of the intent's map: a
    PROMOTION to error makes a rule gating, a demotion never makes it
    advisory. A dark rule never runs, so demoting it would change nothing but
    this answer -- a way to make P1's refusal go away by editing a severity
    no finding will ever carry (#959 plan review, round 3).
    """
    if rule in _FORCED_SEVERITY:
        return _FORCED_SEVERITY[rule] == ERROR
    if rule == 'decap_pin_distance':
        # Per board. A pin found only through the rail-net channel is filed
        # under `decap_pin_distance_inferred`, WARN by default, so on a board
        # whose every supply pin came that way the rule cannot fail the grade
        # (esp_prog, splitflap_driver, tigard, ulx3s -- measured, #959 P4).
        recs = ctx.supply_pins() if ctx is not None else {}
        chans = {r['channel'] for r in recs.values() if r['pins']}
        if chans & {'pintype', 'pinfunction'}:
            return True
        if 'rail_net' in chans:
            return intent.severity.get('decap_pin_distance_inferred') == ERROR
        return False
    if intent.severity.get(rule) == ERROR:
        return True
    return _RULE_DEFAULT_SEVERITY.get(rule, ERROR) == ERROR


def _roster(intent: Intent, pcb_data, ctx, *, census=None,
            brief_fragment=None) -> List[Dict[str, object]]:
    """One row per rule in `RULES`: is it armed, and if not, does anything
    answer for that? The `needs_disposition` rows are what P1 refuses on.

    A rule is REFUSED only when all of these hold: it is dark (or abstained,
    or armed with a withheld key), it is not a policy rule, a board fact says
    it applies, it is gating, and no written disposition answers for it.
    Withheld budgets are refused like dark rules, for decaps and legality
    alike: in both cases the emitter declined to derive a number, and in both
    cases the honest answers are "declare it from a requirement" or "say why
    not" (#959 Phase 0 checkpoint).
    """
    census = census if census is not None else decap_census(pcb_data)
    disp = intent.dispositions or {}
    rule_disp = disp.get('rules', {})
    held_disp = disp.get('withheld', {})
    abstained = {str(k): str(v)
                 for k, v in (intent.budget_withheld or {}).items()
                 if not _declared_by_hand(intent, str(k))}
    # Read off the BUDGET the plan declares, not only its own note about what
    # was withheld: `context.budget_withheld` is prose the plan carries, and
    # deleting it used to delete the debt (Phase-1 verifier, run 29's r1).
    # The two keys the emitter grades by default are owed whenever the budget
    # arms legality without them.
    if _wants(intent, 'legality'):
        for k in ('overlap_area', 'oob_count'):
            if k not in (intent.legality_budget or {}):
                abstained.setdefault(
                    k, f"legality_budget declares no `{k}`, so it is not "
                       f"graded -- whatever the plan's notes say")
    rows: List[Dict[str, object]] = []
    for name, _fn in RULES:
        wants = _wants(intent, name)
        arm_why = None
        if wants and ctx is not None and name in _ARM:
            arm_why = _ARM[name](ctx)
        state = ('armed' if wants and arm_why is None
                 else 'abstained' if wants else 'dark')
        withheld = {k: v for k, v in sorted(abstained.items())
                    if name in (_WITHHELD_RULE.get(k) or ((),))[0]}
        brief = _brief_claims(name, brief_fragment)
        policy = name in _POLICY_RULES and not brief
        if state == 'armed':
            applicable, why_app = True, (
                f"armed by the plan's `{_ARMING_KEY.get(name, name)}`")
        elif brief:
            applicable, why_app = True, (
                f"the design brief declares {len(brief)} "
                f"({', '.join(brief[:6])}{', ...' if len(brief) > 6 else ''}"
                f"); a plan that drops one is refused by P1's brief-clause "
                f"check, answered by carrying it or by `--waive "
                f"brief-clause:<id>:<why>`")
        elif policy:
            applicable, why_app = True, _POLICY_RULES[name]
        else:
            applicable, why_app = _applicability(
                name, intent, pcb_data, ctx, census, brief_fragment)
        gating = _gating(name, intent, ctx)
        disposition = rule_disp.get(name, '')
        held_answered = {k: held_disp[k] for k in withheld if k in held_disp}
        open_held = [k for k in withheld if k not in held_disp]
        needs = False
        if not policy and applicable and gating:
            if state != 'armed':
                # A brief-declared rule is answered at the clause gate, not
                # here: one question, one answer.
                needs = (not brief and not disposition
                         and not (withheld and not open_held))
            else:
                needs = bool(open_held)
        skip = ('' if state == 'armed'
                else arm_why if state == 'abstained'
                else _SKIP_REASON.get(name, 'not requested'))
        rows.append({
            'rule': name, 'state': state,
            'arming_key': _ARMING_KEY.get(name, ''),
            'policy': policy, 'applicable': applicable,
            'applicability_reason': why_app, 'gating': gating,
            'default_severity': _RULE_DEFAULT_SEVERITY.get(name, ERROR),
            'skip_reason': skip, 'withheld': withheld,
            'disposition': disposition,
            'withheld_dispositions': held_answered,
            'needs_disposition': needs,
            'brief_claims': brief,
        })
    return rows


def stale_dispositions(intent: Intent, rows, pcb_data=None,
                       reconciliation=None) -> List[str]:
    """Written answers to questions this plan does not ask, by name, so the
    author removes them -- a stale disposition reads as though something were
    excused when nothing is.

    A withheld-key disposition for a key the emitter did not withhold; and,
    with the board in hand, a `refs` disposition for a block the board does
    not have, one that carries pads (the seeder's to place), or one already
    FILE-locked (the lock is the answer). The P1 driver refuses on the refs
    cases with its own wording; this is the same judgement for
    `check_floorplan`, so the two never disagree about one file.
    """
    if reconciliation is not None:
        ids = {r['id'] for r in reconciliation
               if r.get('kind') == 'contradiction'}
        stale_c = [f"dispositions.contradictions.{k}: no such contradiction "
                   f"on this board and these inputs"
                   for k in (intent.dispositions or {}).get(
                       'contradictions', {}) if k not in ids]
    else:
        stale_c = []
    held = set()
    for r in rows:
        held.update(r['withheld'])
    out = [f"dispositions.withheld.{k}: nothing is withheld under that key"
           for k in (intent.dispositions or {}).get('withheld', {})
           if k not in held]
    if pcb_data is not None:
        fps = pcb_data.footprints or {}
        for k in (intent.dispositions or {}).get('refs', {}):
            fp_ = fps.get(k)
            if fp_ is None:
                why = 'no such block on this board (keys are exact)'
            elif fp_.pads:
                why = ('the block carries pads -- the seeder places it, '
                       'and `refs` answers pad-less blocks only')
            elif getattr(fp_, 'locked', False):
                why = 'the block is already locked; the lock is the answer'
            else:
                continue
            out.append(f"dispositions.refs.{k}: {why}")
    return sorted(out + stale_c)


def roster_refusal_lines(rows) -> List[str]:
    """One line per row P1 refuses on, naming the key that arms the rule and
    the disposition spelling that answers it. Never steers toward inventing a
    limit: "declare it from a requirement" and "write why it does not apply"
    are offered side by side."""
    out = []
    for r in rows:
        if not r['needs_disposition']:
            continue
        name = r['rule']
        if r['state'] == 'armed':
            for k in r['withheld']:
                if k in r['withheld_dispositions']:
                    continue
                out.append(
                    f"{name}: armed, but the emitter WITHHELD `{k}` "
                    f"({r['withheld'][k]}), so that half is never graded. "
                    f"Declare `{k}` from a requirement, or write why it "
                    f"stays ungraded: dispositions.withheld.{k}")
            continue
        out.append(
            f"{name}: {r['state']} -- {r['skip_reason']}. It applies here "
            f"({r['applicability_reason']}). Arm it with `{r['arming_key']}` "
            f"if the design has that requirement, or write why it does not: "
            f"dispositions.rules.{name}")
    return out


def rule_roster(intent: Intent, pcb_data, pcb_file: str, *,
                group_sources: Sequence[str] = (),
                clearance: Optional[float] = None,
                board_edge_clearance: Optional[float] = None,
                brief_fragment=None) -> List[Dict[str, object]]:
    """The roster for a PLAN, before anything is graded (P1's question).

    Builds the same `_Ctx` `grade` builds, so the board facts the roster reads
    (supply pins, the decap census) are the ones the grade would read."""
    ctx = _grade_ctx(intent, pcb_data, pcb_file, group_sources=group_sources,
                     clearance=clearance,
                     board_edge_clearance=board_edge_clearance)[0]
    ctx.brief_fragment = brief_fragment
    return _roster(intent, pcb_data, ctx, brief_fragment=brief_fragment)


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
    #: #712: the along-edge position of every declared edge connector, taken
    #: whether or not anyone declared a claim about it. Advisory, like the
    #: `health` block -- a measurement, never a verdict.
    edge_seating: List[Dict[str, object]] = field(default_factory=list)
    #: #894: every gap `rule_proximity` measured, PASSING CLAUSES INCLUDED.
    #: A rule yields violations, so a clause that holds is silent and its
    #: number -- the one a placement score wants to report and compare against
    #: the previous lap -- was discarded. Same channel and same standing as
    #: `edge_seating`: a measurement, never a verdict. Empty when the intent
    #: declares no proximity claim, which is DIFFERENT from every claim
    #: passing, and the consumer must not confuse the two.
    proximity_measured: List[Dict[str, object]] = field(default_factory=list)
    #: #961: one row per declared edge connector found on the board -- the
    #: number its `overhang_mm` band was graded on and the CURRENCY of it
    #: (`overhang_basis`: the drawn body, or the legacy occupancy reading when
    #: no body can be measured), the body measurements, and the part's
    #: pad-copper edge clearance. A measurement, never a verdict; empty when
    #: the rule did not run.
    edge_connector_evidence: List[Dict[str, object]] = field(
        default_factory=list)
    #: #705: HOW the pin rule reached its answer, whether or not it found
    #: anything. Without it a board graded entirely on channel-3 inference
    #: and a board graded on declared pintype print the same clean pass --
    #: `rules_run` cannot tell them apart, and the difference is the whole
    #: reason the inferred findings carry their own name. Empty when the
    #: rule did not run.
    decap_pin_evidence: Dict[str, object] = field(default_factory=dict)
    #: #959 (#997): the rule roster, when `grade(with_roster=True)` built it;
    #: None when nobody asked, which a consumer must not read as "no rule is
    #: dark".
    roster: Optional[List[Dict[str, object]]] = None
    #: #959: dispositions this plan writes that answer nothing (see
    #: `stale_dispositions`). Empty unless the roster was built.
    stale_dispositions: List[str] = field(default_factory=list)

    @property
    def dark_undispositioned(self) -> List[str]:
        """Rules P1 would refuse on, by name. Empty when no roster was built,
        so read it together with `roster is not None`."""
        return [r['rule'] for r in (self.roster or ())
                if r['needs_disposition']]

    @property
    def errors(self) -> List[Violation]:
        return [v for v in self.violations if v.severity == ERROR]

    @property
    def warnings(self) -> List[Violation]:
        return [v for v in self.violations if v.severity != ERROR]

    @property
    def not_graded(self) -> Dict[str, int]:
        """The channels a DECLARED thing went ungraded through, by name.

        By NAME, and never one aggregate count, because the three are
        different claims and `grade()` deliberately keeps them apart
        (see the note at `_ARM`). #694's lesson applies directly: an
        aggregate verdict cannot say which of its inputs moved.

        `rules_skipped` is NOT wholly in here, and that is the load-bearing
        distinction. It carries two opposite things: `_SKIP_REASON` entries,
        every one of which reads "the intent declares no X" -- NOBODY ASKED,
        which is honest and fires on 22 of 22 tracked boards -- and `_ARM`
        entries, "the intent asked, this BOARD cannot answer", which is an
        abstention. Counting the first would make every board with a minimal
        intent permanently incomplete and collapse the signal to "every board
        that passes" (measured: 21 of 22 rather than 5).
        """
        out = {}
        if self.budget_abstained:
            out['budget_abstained'] = len(self.budget_abstained)
        _armed = sum(1 for k, r in self.rules_skipped.items()
                     if not _is_not_asked(k, r))
        if _armed:
            out['rules_skipped_armed'] = _armed
        _edge = sum(1 for e in self.edge_seating
                    if e.get('declared') and e.get('abstained'))
        if _edge:
            out['edge_seating_abstained'] = _edge
        return out

    @property
    def complete(self) -> bool:
        """Did every channel the intent ASKED FOR actually get graded?

        Separate from `passed` on purpose. `passed` answers "were there
        errors"; this answers "was anything left unmeasured". A run can be
        both clean and incomplete, and until #713 that combination printed
        `PASS: N rule(s) ran, no violations` twenty-six lines above
        `N declared value(s) NOT DERIVABLE -- not graded, not passed`, with
        `pass: true` and exit 0. Measured on ulx3s, tigard, watchy,
        glasgow_revC and orangecrab_ext_pll -- 5 of the 22 tracked boards --
        in the DEFAULT emit-intent-then-grade round trip.
        """
        return not self.not_graded

    @property
    def passed(self) -> bool:
        """No errors AND nothing declared left ungraded.

        The second conjunct is #713 item 5. A verdict about a channel nobody
        measured is not a pass; it is an absence of a verdict, and the
        machine-readable `pass` key is where consumers read it -- every
        production consumer reads `errors` or the violations list, and none
        read this property, so landing the fix here alone would have been
        invisible. It lands on the wire and the exit code too.
        """
        return not self.errors and self.complete


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


def _run_rules(ctx, abstained=None):
    """`(violations, ran, skipped)`: every rule in RULES over one `_Ctx`.

    The one loop both `grade` and `PoseGrader` run, so a grade asked about a
    pose nothing has written yet cannot drift from the grade of the board.
    `abstained` is the emitter's withheld-budget map, used only to word a
    skip reason."""
    found: List[Violation] = []
    ran: List[str] = []
    skipped: Dict[str, str] = {}
    for name, fn in RULES:
        if not _wants(ctx.intent, name):
            reason = _SKIP_REASON.get(name, 'not requested')
            # "declares no X" is true but reads as "nobody wanted one". Say
            # that the emitter refused to DERIVE it, on whichever rule the
            # withheld key disarms -- not only on `legality` (#704).
            mine = {k: v for k, v in (abstained or {}).items()
                    if name in (_WITHHELD_RULE.get(k) or ((),))[0]}
            if mine:
                reason += ('; the emitter WITHHELD ' + ', '.join(
                    f'{k} ({v})' for k, v in sorted(mine.items())))
            skipped[name] = reason
            continue
        arm = _ARM.get(name)
        why = arm(ctx) if arm is not None else None
        if why is not None:
            skipped[name] = why
            continue
        ran.append(name)
        found.extend(fn(ctx))
    return found, ran, skipped


class _PosedState:
    """What `_Ctx` reads off a `QuenchState`, answered for the board a seat
    search is ASKING about rather than one it has written: the search state's
    own parts at its current poses, `poses` overriding some, `exclude` (the
    pile) left out. Each answer calls the state's own code on that part set."""

    def __init__(self, state, exclude=(), poses=None):
        self._s = state
        self._exclude = frozenset(exclude)
        self._poses = dict(poses or {})
        self.edge_gate = state.edge_gate

    def pose(self, ref):
        if ref in self._poses:
            return self._poses[ref]
        part = self._s.parts[ref]
        return (part.x, part.y, part.rot)

    def graded_parts(self):
        out = []
        for ref, part in self._s.parts.items():
            if ref in self._exclude:
                continue
            x, y, rot = self.pose(ref)
            out.append(legality.GradedPart(ref=ref, side=part.side,
                                           rect=part.rect(x, y, rot),
                                           tht_rect=part.tht_rect(x, y, rot),
                                           has_tht=part.has_tht))
        return out

    def _owned_rings(self, ref):
        """Ring ownership at the pose this board would be WRITTEN at: a grade
        of the written board takes its seed pose from the file."""
        part = self._s.parts[ref]
        x, y, rot = self.pose(ref)
        pts = [(gx, gy) for (gx, gy, _net) in part.pad_globals(x, y, rot)]
        return self.edge_gate.rings_enclosing(pts) if pts else frozenset()

    def hpwl(self):          # no rule reads it
        return 0.0

    def pad_legality_metrics(self):   # no rule reads it; the gate grades pads itself
        return {}

    def legality_metrics(self):
        from .quench import QuenchState
        return QuenchState.legality_metrics(self)

    def board(self):
        from copy import copy
        view = copy(self._s.pcb_data)
        view.footprints = {
            ref: (legality.footprint_at_pose(fp, self.pose(ref))
                  if ref in self._s.parts else fp)
            for ref, fp in self._s.pcb_data.footprints.items()
            if ref not in self._exclude}
        return view


class PoseGrader:
    """The intent grade of a seat search's board at poses it has not written
    (#975's grade delta). Constructing one reads nothing; the pose-invariant
    inputs (outline, locked refs, drawn bodies) are read on first use and kept.

    Every violation comes from the same RULES loop `grade` runs (`_run_rules`)
    over a `_Ctx` built on `_PosedState`. The violations `grade` adds outside
    that loop (intent validation, block resolution, keep-out allows) read only
    the intent and the footprint SET, so they cannot differ between two poses
    of one part and are left out of a delta.

    ONE cached input is NOT pose-invariant, which is why `interior_split`
    exists: the state's `edge_gate` carries the PARSER's split of interior
    Edge.Cuts contours into cutouts (holes, which put anything inside them off
    the board) and milled rings (edges copper holds clearance from). A contour
    enclosing >= 2 pad CENTRES is reclassified from the first to the second
    (`kicad_parser.drop_pad_containing_cutouts`), so a pose that carries pads
    into or out of one changes the classification -- and a grade of the board
    that would be WRITTEN, which re-parses, then reads different off-board
    numbers than this grader does. Measured on a synthetic board: a
    sub-millimetre move of a declared connector (0.5 mm on each axis) took
    `board_cutouts` 1 -> 0 and the written board's `oob_count` 2 -> 0 while
    this grader's reading held at 2. A caller comparing two poses asks `interior_split` for each and
    does not compare grades across a difference."""

    def __init__(self, intent, state, *, blocks, clearance=None,
                 board_edge_clearance=None):
        self.intent, self.state, self.blocks = intent, state, blocks
        self.floors = (clearance, board_edge_clearance)
        self._outline = None
        self._locked = None
        self._bodies = None
        self._rings = None
        self._ring_base = None
        self._ring_at = None

    def interior_split(self, poses=None):
        """The cutout / milled verdict for each interior contour at `poses`.

        A tuple of bools, one per interior contour in a fixed order: True when
        at least two pad centres fall inside it, which is the parser's own
        threshold for calling it a milled ring rather than a hole. `()` on a
        board with no interior contour, which is most of them and costs
        nothing after the first call.

        Counted over the pads of every part the search knows -- the pile at its
        input coordinates included, because the parser counts every footprint in
        the file it reads back, and both poses of a comparison include it
        identically. Only the refs in `poses`, and those the SEARCH has moved
        since the last call, are re-measured; the rest are kept.

        That last clause is load-bearing: caching every other part's count once
        and never re-reading it let an `apply_move` of a DIFFERENT part mask a
        real crossing of the threshold, which is the failure this method exists
        to catch. Stage 1 shares one grader across every edge connector and
        moves parts between them, so the case is not hypothetical.
        """
        gate = getattr(self.state, 'edge_gate', None)
        if self._rings is None:
            rings = [r for r in (getattr(gate, 'cutouts', None) or ())
                     if len(r) >= 3]
            rings += [r for r in (getattr(gate, 'milled', None) or ())
                      if len(r) >= 3 and r not in rings]
            self._rings = rings
        if not self._rings:
            return ()
        if self._ring_base is None:
            self._ring_base, self._ring_at = {}, {}
        counts = [0] * len(self._rings)
        for ref, part in self.state.parts.items():
            if poses and ref in poses:
                per = self._ring_counts(ref, poses[ref])
            else:
                here = (part.x, part.y, part.rot)
                if self._ring_at.get(ref) != here:
                    self._ring_base[ref] = self._ring_counts(ref, here)
                    self._ring_at[ref] = here
                per = self._ring_base[ref]
            for i, n in enumerate(per):
                counts[i] += n
        return tuple(n >= 2 for n in counts)

    def legality_at(self, *, exclude=(), poses=None):
        """The placement's own legality numbers at `poses` -- overlap and
        off-board -- whatever the intent declares.

        `violations` cannot stand in for these. `_run_rules` skips `legality`
        when the intent carries no `legality_budget`, and `emit_intent`
        WITHHOLDS `overlap_area` exactly on a board that already has blocking
        body pairs or unwaived courtyard interpenetration -- so on the boards
        where courtyard overlap is the live risk, the rule that would catch it
        is not armed. A caller comparing two poses therefore compares these as
        well, or it is blind to a move that buys interpenetration: measured on
        a fixture, 0.18 mm2 of new overlap with a LOCKED part, no pad or hole
        predicate able to see it and no grade error raised.
        """
        return _PosedState(self.state, exclude, poses).legality_metrics()

    def _ring_counts(self, ref, pose):
        """How many of `ref`'s pad centres fall inside each interior contour.

        `pose` None means the pose the search holds for it right now, which is
        the file's for anything the seeder has not moved.
        """
        from kicad_parser import _pt_in_ring
        fp = (self.state.pcb_data.footprints or {}).get(ref)
        if fp is None:
            return [0] * len(self._rings)
        if pose is None:
            part = self.state.parts[ref]
            pose = (part.x, part.y, part.rot)
        pads = [(p.global_x, p.global_y)
                for p in legality.pads_at_pose(fp, pose)]
        return [sum(1 for (px, py) in pads if _pt_in_ring(px, py, r))
                for r in self._rings]

    def violations(self, *, exclude=(), poses=None) -> List[Violation]:
        state = self.state
        if getattr(state, 'body_model', False):
            raise ValueError('a body_model search state grades occupancy rects, '
                             'not the courtyards the grade reads')
        if self._outline is None:
            self._outline = outline_state(state.pcb_data, state.pcb_file)
        if not self._outline['trustworthy']:
            raise UntrustworthyOutline(self._outline['problems'])
        if self._locked is None:
            try:
                from .parser import extract_locked_refs
                self._locked = (extract_locked_refs(state.pcb_file)
                                if state.pcb_file else set())
            except (OSError, ValueError):
                self._locked = set()
        if self._bodies is None:
            from .body import board_bodies
            self._bodies = board_bodies(state.pcb_data, state.pcb_file)
        view = _PosedState(state, exclude, poses)
        ctx = _Ctx(self.intent, view.board(), state.pcb_file, view, self.blocks,
                   self._locked, self._outline)
        ctx.requested_floors = self.floors
        ctx._bodies = self._bodies
        found, _ran, _skipped = _run_rules(ctx)
        return found


def grade_delta(before: Sequence[Violation],
                after: Sequence[Violation]) -> List[Dict[str, object]]:
    """What `after` adds to `before`, in the exit gate's currency: ERRORS only.

    A claim is `(rule, ref, block, expected keys)`, compared as a multiset, so
    an error the first pose does not have is added whatever its message says.
    A board-level budget has no ref and stays ONE error however far it is
    over, so for those the measured value must not grow either -- with the
    pile left out of both grades, only the moved part can have grown it.

    What a claim deliberately cannot see, since the currency is the exit gate's
    and the gate counts errors: an error SWAPPED for another error of the same
    rule, ref, block and expected keys (one keep-out for another) reads as no
    change, and a ref-carrying error that merely gets worse (0.10mm -> 9.90mm)
    is still one error. The value check above is for the ref-less budgets only,
    where one error is all there ever is."""
    from collections import Counter

    def claim(v):
        return (v.rule, v.ref or '', v.block or '',
                tuple(sorted((v.expected or {}).keys())))
    was = [v for v in before if v.severity == ERROR]
    now = [v for v in after if v.severity == ERROR]
    out: List[Dict[str, object]] = [
        {'rule': rule, 'ref': ref or None, 'block': block or None, 'added': n}
        for (rule, ref, block, _keys), n
        in sorted((Counter(map(claim, now)) - Counter(map(claim, was))).items())]
    budgets = {claim(v): v for v in was if not v.ref}
    for v in now:
        if v.ref or claim(v) not in budgets:
            continue
        for key in sorted(v.expected or {}):
            a = budgets[claim(v)].measured.get(key)
            b = (v.measured or {}).get(key)
            if (isinstance(a, (int, float)) and isinstance(b, (int, float))
                    and b > a + legality.EPS):
                out.append({'rule': v.rule, 'budget': key, 'before': a, 'after': b})
    return out


def _plan_severity(intent: Intent, name: str, needs) -> str:
    """A plan-check ERROR stands for "the grade will fail", so it is only
    an ERROR while every rule it stands for is one: a plan that demoted
    `zone_containment` to warn is not refused for a zone its members may
    leave (round-2 verifier). An explicit setting for the finding wins."""
    if name in intent.severity:
        return intent.severity[name]
    return (ERROR if all(intent.severity_of(r) == ERROR for r in needs)
            else WARN)


def exclusive_unsatisfiable(intent: Intent, blocks, pcb_data,
                            pcb_file: str = '', *, state=None
                            ) -> List[Violation]:
    """Members that cannot fit in their own zone clear of a stranger's
    EXCLUSIVE zone (#959, #998) -- the one overlap no placement can satisfy.

    `rule_zone_exclusive` forbids a stranger's courtyard inside an exclusive
    zone. So a member m of zone B, not a member of exclusive zone A, on A's
    side, must fit inside B (within tolerance) without entering A. That is
    exactly `zone_pose_feasibility` with A as a keep-out -- the same kernel
    `intent_zone_in_keepout` uses, judged by the same `zone_escape` /
    `keepout_hit` the grade uses, so the verdict cannot drift from the grade.
    Anything else about two overlapping zones is satisfiable.

    THE GRADE'S GEOMETRY, from the grade's own placement state: the rect
    `rule_zone_containment` and `rule_zone_exclusive` test (courtyard, else
    pad bbox), never `legality.part_local_bounds`' occupancy, which adds the
    drawn .Fab body -- measured differing by more than 0.05 mm on 29 corpus
    parts (ulx3s U9 by 7 mm), so a satisfied board failed its own grade
    (Phase-4 verifier). A part the state does not carry is graded by neither
    rule and is skipped here too. And two rotation lattices: the part's own
    and the 0-degree one,
    since an author can turn a part the seeder would not. That is not every
    angle, and it need not be for soundness: the part's current pose is on
    its own lattice, so a board that grades clean always has a candidate.
    """
    out: List[Violation] = []
    excl = [z for z in intent.blocks if z.exclusive and z.rect is not None]
    if not excl:
        return out
    if state is None:
        import pose_score
        state = pose_score.make_state(pcb_data, pcb_file)
    for zb in intent.blocks:
        if zb.rect is None:
            continue
        tol = intent.zone_tolerance(zb)
        for za in excl:
            if za.name == zb.name:
                continue
            if legality.rect_overlap_area(_inflate(zb.rect, tol),
                                          za.rect) <= legality.EPS:
                continue
            owners = set(blocks.get(za.name, ()))
            # #797's guard, exactly as `rule_zone_exclusive` applies it: an
            # exclusive zone with no member the GRADE can see grades nobody,
            # so it can make nothing unsatisfiable either (round-2 verifier:
            # a zone exclusive to a courtyard-less logo refused a plan the
            # grade passed).
            if not (owners & set(state.parts)):
                continue
            for ref in blocks.get(zb.name, ()):
                if ref in owners:
                    continue
                fp = (pcb_data.footprints or {}).get(ref)
                sp = state.parts.get(ref)
                if fp is None or sp is None:
                    continue
                if za.side and sp.side != za.side:
                    continue
                local0 = sp.bounds_by_rot[0.0]
                tried = []
                feasible = False
                for base in sorted({(fp.rotation or 0.0) % 360.0, 0.0}):
                    v = zone_pose_feasibility(
                        zb.rect, tol, _LocalPart(base, local0, None),
                        [{'name': za.name, 'rect': za.rect,
                          'sides': ('F', 'B'), 'allow': ()}])
                    tried += list(v['rotations'])
                    if v['feasible']:
                        feasible = True
                        break
                if feasible:
                    continue
                v = dict(v, rotations=sorted(set(tried)))
                out.append(Violation(
                    rule='plan_zone_exclusive_unsatisfiable',
                    severity=_plan_severity(
                        intent, 'plan_zone_exclusive_unsatisfiable',
                        ('zone_containment', 'zone_exclusive')),
                    block=zb.name, ref=ref,
                    message=(f"{ref} must sit in zone {zb.name!r} but has no "
                             f"pose there, at any of "
                             f"{len(v['rotations'])} rotations, that stays "
                             f"out of {za.name!r}, which is EXCLUSIVE to "
                             f"its own members -- no placement satisfies "
                             f"both"),
                    measured={'zone': list(zb.rect), 'exclusive_zone':
                              list(za.rect), 'tolerance_mm': tol,
                              'rotations': [round(r, 3)
                                            for r in v['rotations']]},
                    expected={'legal_poses': '>= 1'}))
    return out


def _glob_literal_findings(intent: Intent, blocks, pcb_data
                           ) -> List[Violation]:
    """A pattern that is ALSO a real reference and matches other blocks.

    `Ref*` names the footprint `Ref*` and, as a glob, matches `Ref*~2` too:
    the trap #726 created by keying a second block `~2`. It is an ERROR only
    when the over-match lands a block in a ZONE that another block also
    claims -- then the plan says two things about one part. An over-match
    the list names anyway (fixture 975's `must_lock ['USB1', 'Ref*',
    'Ref*~2']`) is intended, and a stray one elsewhere is a lint: WARN.
    """
    out: List[Violation] = []
    refs = set(pcb_data.footprints or {})
    zmap = {z.name: z for z in intent.blocks}

    def _disjoint(names):
        # Two zones a part must sit in at once are a CONTRADICTION only when
        # they share no area (with tolerance). Nested or overlapping ones --
        # `Ref*` board-wide beside a tight `Ref[*]~2` -- are satisfiable.
        rs = [_inflate(zmap[n].rect, intent.zone_tolerance(zmap[n]))
              for n in names if zmap.get(n) is not None
              and zmap[n].rect is not None]
        return any(legality.rect_overlap_area(a_, b_) <= legality.EPS
                   for i_, a_ in enumerate(rs) for b_ in rs[i_ + 1:])
    zoned_owner: Dict[str, List[str]] = {}
    for z in intent.blocks:
        if z.rect is not None:
            for r in blocks.get(z.name, ()):
                zoned_owner.setdefault(r, []).append(z.name)
    lists = [(f"blocks[{z.name}].refs", list(z.refs), z)
             for z in intent.blocks]
    lists.append(('must_lock', list(intent.must_lock), None))
    for i, k in enumerate(intent.keepouts):
        lists.append((f"keepouts[{k.get('name', i)}].allow",
                      list(k.get('allow') or ()), None))
    lists.append(('decaps.exempt',
                  list((intent.decaps or {}).get('exempt') or ()), None))
    for where, pats, zone in lists:
        # A block the same list NAMES -- literally, or by its escaped form
        # (`Ref[*]~2`) -- was meant, whatever else also matches it.
        esc = set(pats)
        named = esc | {r for r in refs if glob_escape(r) in esc}
        for pat in pats:
            if pat not in refs or not any(ch in pat for ch in '*?['):
                continue
            extra = sorted(r for r in refs
                           if r != pat and fnmatch.fnmatchcase(r, pat))
            if not extra:
                continue
            unnamed = [r for r in extra if r not in named]
            if not unnamed:
                continue
            conflict = [r for r in unnamed
                        if zone is not None and zone.rect is not None
                        and len(zoned_owner.get(r, ())) > 1
                        and _disjoint(zoned_owner[r])]
            out.append(Violation(
                rule='block_glob_literal',
                severity=(intent.severity_of('block_glob_literal')
                          if conflict else WARN),
                block=(zone.name if zone is not None else None),
                message=(f"{where}: {pat!r} is a real reference AND, as a "
                         f"glob, also matches {', '.join(unnamed)}"
                         + (f" -- which another zoned block also claims, "
                            f"so the plan puts {', '.join(conflict)} in two "
                            f"zones that share no area" if conflict else '')
                         + f". Write {glob_escape(pat)!r} to mean the one "
                         f"block"),
                measured={'pattern': pat, 'matches': [pat] + extra},
                expected={'pattern': glob_escape(pat)}))
    return out


def glob_escape(ref: str) -> str:
    """The pattern that matches `ref` and nothing else (`Ref*` -> `Ref[*]`)."""
    import glob as _glob
    return _glob.escape(ref)


#: The `plan_check` findings `place_seed` REFUSES on (#959): the area
#: bounds and the literal glob -- plans the seeder has no per-member answer
#: for. The older plan contradictions keep their pre-#959 contract there:
#: a zone a keep-out covers (#701/#799) and a zone a stranger's EXCLUSIVE
#: zone swallows (#797) are seeded, and the seeder names the member it could
#: not seat with a measured verdict (`keepout_blocks`,
#: `zone_exclusive_blocks`, the per-zone and joint censuses), which
#: `test_701_keepout_seating` and `test_797_zone_exclusive_seating` pin and
#: which a plan-level refusal would replace with a coarser answer. Those,
#: `intent_zone_outside_envelope` and `block_unresolved` are PRINTED by
#: place_seed and REFUSED by P1, which gates every plan ERROR.
PLAN_SEED_REFUSES = frozenset({
    'plan_zone_overfull', 'plan_edge_overfull', 'plan_board_overfull',
    'block_glob_literal'})


def plan_check(intent: Intent, pcb_data, pcb_file: str, *,
               group_sources: Sequence[str] = (),
               clearance: Optional[float] = None,
               board_edge_clearance: Optional[float] = None):
    """The zone plan checked against itself and the board, BEFORE the first
    pose write (#959, #998). Returns `(violations, measured)`.

    Run 29 found its zone plan's ERRORs at lap 5 because the only caller of
    `validate_intent` was `grade`, which place_seed runs AFTER writing the
    seed. This runs the checks that need no pose of a movable part, so P1 and
    place_seed can refuse a plan before anything is placed.

    EVERY ERROR IS SOUND -- the plan cannot be seeded or satisfied however
    the movable parts are arranged -- and the evidence behind each bound is
    in its message. The WARN halves are the same quantities with a margin.

      1. `intent_zone_overlap` (WARN; the old "no placement can satisfy both"
         was false) and `plan_zone_exclusive_unsatisfiable` (ERROR).
      2. `intent_zone_in_keepout`, as `grade` raises it.
      3. `block_glob_literal`: a real reference used as a glob.
      4. `plan_fixed_outside_zone`: a FILE-locked member already outside its
         zone. A lock is the only thing the seeder does not move (edge claims
         and must_lock are seated), so this is a fact about the plan.
      8. `plan_fixed_overlap` (WARN, per pair) / `plan_fixed_overlap_budget`
         (ERROR): two FILE-locked parts whose courtyards overlap on a shared
         face overlap in every placement. The grade counts courtyard overlap
         only against a declared `legality_budget.overlap_area` -- run 29's
         shipped board carried a 1.0 mm2 fiducial-in-connector overlap -- so
         the ERROR fires only when the locked pairs ALONE exceed that budget,
         which no arrangement of the other parts can undo.
      5. `plan_zone_overfull` / `_crowded`: per FACE, the members' areas,
         locked ones included (courtyard, else pad bbox; a through-hole
         member's drilled footprint, clipped to its courtyard, charged to the
         far face) against the zone's area.
         Fitting area A into a zone of area Z forces at least A - Z of
         pairwise courtyard overlap, which the grade counts against a
         DECLARED `legality_budget.overlap_area`: past it, ERROR. With no
         budget declared nothing bounds the overlap, and it is the WARN (the
         seeder never overlaps courtyards, so it may leave members unseated).
         Anchor-graded members and zones holding a waived pair are not
         charged; a locked member is, because the grade counts its overlap
         too.
      6. `plan_edge_overfull` / `_crowded`: one edge-claimed part whose PAD
         extent in its own frame, at its best 90-degree rotation, is longer
         than the edge (ERROR); the claimed parts' summed extents against the
         edge's span (WARN -- flanges legitimately overhang corners).
      7. `plan_board_overfull` / `_crowded`: `options.grow_board` at
         clearance 0 on the per-face basis. The same argument as row 5: an
         ERROR when the overlap it forces exceeds a declared budget; a WARN
         past `options.CROWDED_UTILISATION` or on the one-face basis a
         declared `assembly.sides` implies.
    """
    from . import options as _opts
    ctx, outline, state, blocks, block_problems = _grade_ctx(
        intent, pcb_data, pcb_file, group_sources=group_sources,
        clearance=clearance, board_edge_clearance=board_edge_clearance)
    out: List[Violation] = (list(validate_intent(intent))
                            + list(block_problems)
                            + list(intent_zone_keepout_problems(
                                intent, blocks, pcb_data, pcb_file))
                            + exclusive_unsatisfiable(intent, blocks,
                                                      pcb_data, pcb_file,
                                                      state=state)
                            + _glob_literal_findings(intent, blocks,
                                                     pcb_data))
    measured: Dict[str, object] = {}

    # 4. file-locked members outside their zone -- the grade's own rule,
    #    restricted to the members the seeder cannot move.
    for v in rule_zone_containment(ctx):
        if v.ref in ctx.locked:
            out.append(Violation(
                rule='plan_fixed_outside_zone',
                # The zone's OWN severity unless this finding is set apart:
                # a plan that demoted zone_containment to warn must not be
                # refused by its plan check for the same fact.
                severity=intent.severity.get('plan_fixed_outside_zone',
                                             v.severity),
                ref=v.ref, block=v.block,
                message=(f"{v.ref} is LOCKED in the board and already "
                         f"outside its zone: {v.message}. Nothing the seeder "
                         f"does will move it -- fix the zone or the pose"),
                measured=v.measured, expected=v.expected))

    # 8. two FILE-locked parts overlapping -- in every placement there is.
    fixed_gp = {p.ref: p for p in state.graded_parts() if p.ref in ctx.locked}
    fixed_refs = sorted(fixed_gp)
    fixed_pairs = []
    for i_, a_ in enumerate(fixed_refs):
        for b_ in fixed_refs[i_ + 1:]:
            area = legality.placement_overlap_area([fixed_gp[a_],
                                                    fixed_gp[b_]])
            if area > legality.EPS:
                fixed_pairs.append((a_, b_, area))
                out.append(Violation(
                    rule='plan_fixed_overlap',
                    severity=intent.severity_of('plan_fixed_overlap', WARN),
                    ref=a_,
                    message=(f"{a_} and {b_} are both LOCKED in the board and "
                             f"their courtyards overlap by {area:.3f}mm2 -- "
                             f"in every placement, since the seeder moves "
                             f"neither"),
                    measured={'pair': [a_, b_],
                              'overlap_area_mm2': round(area, 4)},
                    expected={'overlap_area_mm2': 0.0}))
    budget = (intent.legality_budget or {}).get('overlap_area')
    fixed_total = sum(x[2] for x in fixed_pairs)
    if budget is not None and fixed_total > float(budget) + legality.EPS:
        out.append(Violation(
            rule='plan_fixed_overlap_budget',
            severity=intent.severity_of('plan_fixed_overlap_budget'),
            message=(f"the LOCKED parts alone overlap by {fixed_total:.3f}"
                     f"mm2, over the declared legality_budget.overlap_area "
                     f"{float(budget):g} -- no arrangement of the other parts "
                     f"can bring the total under it. Unlock one of each "
                     f"pair, move it, or raise the budget: "
                     + ', '.join(f"{a_}/{b_} {ar:.3f}"
                                 for a_, b_, ar in fixed_pairs)),
            measured={'fixed_overlap_area_mm2': round(fixed_total, 4),
                      'pairs': [[a_, b_, round(ar, 4)]
                                for a_, b_, ar in fixed_pairs]},
            expected={'overlap_area': float(budget)}))
    measured['fixed_overlap'] = {
        'pairs': [[a_, b_, round(ar, 4)] for a_, b_, ar in fixed_pairs],
        'total_mm2': round(fixed_total, 4)}

    # 5. per-zone area, per face
    import routing_defaults as _rd
    clr_used = float(clearance if clearance is not None else _rd.CLEARANCE)
    waived = {frozenset(p_) for p_ in intent.waiver_pairs()}
    zone_rows = []
    # Sound for the GRADE, not only for the seeder: courtyards may overlap
    # there up to a declared `legality_budget.overlap_area`, so members
    # whose areas exceed the zone by A must overlap by at least A in total
    # (Bonferroni: union >= sum - pairwise), and the grade's overlap is that
    # pairwise sum over the whole board. ERROR only past a DECLARED budget;
    # with none, nothing bounds the overlap and it is a WARN about the seeder.
    # EVERY member counts, locked ones included: the grade counts a locked
    # pair's overlap too, so against the budget the sum is sound -- and
    # dropping them lost a zone a locked part already fills (round-2
    # verifier). A locked member outside its zone is `plan_fixed_outside_zone`.
    overlap_budget = (intent.legality_budget or {}).get('overlap_area')
    for z in intent.blocks:
        if z.rect is None:
            continue
        members = [r for r in blocks.get(z.name, ()) if r in state.parts]
        if any(frozenset((a_, b_)) in waived
               for a_ in members for b_ in members if a_ < b_):
            continue
        tol = intent.zone_tolerance(z)
        zr = _inflate(z.rect, tol)
        zarea = max(0.0, zr[2] - zr[0]) * max(0.0, zr[3] - zr[1])
        per_face = {'F': 0.0, 'B': 0.0}
        # The WARN half: the same sum with the grading clearance around each
        # member, which is what the seeder actually needs between them.
        per_face_c = {'F': 0.0, 'B': 0.0}
        counted = []
        for r in members:
            part = state.parts[r]
            b0 = part.bounds_by_rot[0.0]
            w, h = b0[2] - b0[0], b0[3] - b0[1]
            if not zone_fits_courtyard(z.rect, (0.0, 0.0, w, h), tol) and                     not zone_fits_courtyard(z.rect, (0.0, 0.0, h, w), tol):
                continue            # anchor-graded: the zone cannot hold it
            per_face[part.side] = per_face.get(part.side, 0.0) + w * h
            per_face_c[part.side] = per_face_c.get(part.side, 0.0) + (
                (w + clr_used) * (h + clr_used))
            t0 = (part.tht_by_rot or {}).get(0.0)
            if t0 is not None:
                # The drilled-pad rect's part INSIDE the courtyard: only the
                # courtyard is confined to the zone, so a lead field reaching
                # past it may sit outside the zone (round-2 verifier: a 12x1
                # drill rect on a 10x4 courtyard was charged 2 mm2 the grade
                # never counts). Pose-invariant, since both turn together.
                far = 'B' if part.side == 'F' else 'F'
                per_face[far] = per_face.get(far, 0.0) + (
                    legality.rect_overlap_area(t0, b0))
            counted.append(r)
        worst = max(per_face, key=lambda f_: per_face[f_])
        need = per_face[worst]
        zone_rows.append({'block': z.name, 'face': worst,
                          'members_area_mm2': round(need, 3),
                          'zone_area_mm2': round(zarea, 3),
                          'overlap_budget_mm2': overlap_budget,
                          'members': counted})
        excess = need - zarea
        if (overlap_budget is not None
                and excess > float(overlap_budget) + legality.EPS):
            out.append(Violation(
                rule='plan_zone_overfull',
                severity=_plan_severity(intent, 'plan_zone_overfull',
                                        ('zone_containment', 'legality')),
                block=z.name,
                message=(f"zone {z.name!r} is {zarea:.2f}mm2 (with its "
                         f"{tol}mm tolerance) and its {len(counted)} "
                         f"member(s) on {worst}.Cu need {need:.2f}mm2 by "
                         f"courtyard alone: fitting them forces at least "
                         f"{excess:.2f}mm2 of courtyard overlap, over the "
                         f"declared legality_budget.overlap_area "
                         f"{float(overlap_budget):g}"),
                measured={'members_area_mm2': round(need, 4),
                          'face': worst, 'members': counted,
                          'forced_overlap_mm2': round(excess, 4)},
                expected={'zone_area_mm2': round(zarea, 4),
                          'overlap_area': float(overlap_budget)}))
        elif excess > legality.EPS:
            out.append(Violation(
                rule='plan_zone_crowded',
                severity=intent.severity_of('plan_zone_crowded', WARN),
                block=z.name,
                message=(f"zone {z.name!r} is {zarea:.2f}mm2 and its "
                         f"{len(counted)} member(s) on {worst}.Cu "
                         f"need {need:.2f}mm2 by courtyard: they fit only by "
                         f"overlapping ({excess:.2f}mm2 at least), which the "
                         f"seeder never does -- it may leave some unseated"),
                measured={'members_area_mm2': round(need, 4),
                          'face': worst, 'forced_overlap_mm2':
                          round(excess, 4)},
                expected={'zone_area_mm2': round(zarea, 4)}))
        elif max(per_face_c.values()) > zarea + legality.EPS:
            face_c = max(per_face_c, key=lambda f_: per_face_c[f_])
            out.append(Violation(
                rule='plan_zone_crowded',
                severity=intent.severity_of('plan_zone_crowded', WARN),
                block=z.name,
                message=(f"zone {z.name!r} holds its members by courtyard "
                         f"but not with {clr_used}mm clearance around each "
                         f"({per_face_c[face_c]:.2f}mm2 needed on "
                         f"{face_c}.Cu, {zarea:.2f}mm2 available) -- the "
                         f"seeder may leave some unseated"),
                measured={'members_area_with_clearance_mm2':
                          round(per_face_c[face_c], 4), 'face': face_c},
                expected={'zone_area_mm2': round(zarea, 4)}))
    measured['zones'] = zone_rows

    # 6. edge length -- a single part's PAD extent vs the edge (ERROR); the
    #    summed extents vs the span (WARN)
    bounds = ctx.outline_bounds
    edge_rows = {}
    if bounds is not None:
        blen = {'north': bounds[2] - bounds[0], 'south': bounds[2] - bounds[0],
                'east': bounds[3] - bounds[1], 'west': bounds[3] - bounds[1]}
        for c in intent.edge_claims():
            ref, edge = str(c.get('ref')), c.get('edge')
            fp = (pcb_data.footprints or {}).get(ref)
            if fp is None or edge not in blen or not fp.pads:
                continue
            # In the footprint's OWN frame, so the answer is the part's and
            # not its current pose's: board-frame extents of a part at 45
            # degrees read a 9x9 array as 11.91 mm (Phase-4 verifier).
            from .utility import compute_footprint_bbox_local
            lb = compute_footprint_bbox_local(fp)
            ext = min(lb[2] - lb[0], lb[3] - lb[1])
            row = edge_rows.setdefault(edge, {'edge': edge,
                                              'bbox_length_mm':
                                              round(blen[edge], 3),
                                              'claimed_mm': 0.0, 'parts': []})
            row['claimed_mm'] = round(row['claimed_mm'] + ext, 3)
            row['parts'].append(ref)
            if ext > blen[edge] + legality.EPS:
                out.append(Violation(
                    rule='plan_edge_overfull',
                    severity=intent.severity_of('plan_edge_overfull'),
                    ref=ref,
                    message=(f"{ref}'s pads span {ext:.2f}mm at its best "
                             f"90-degree rotation, and the {edge} edge is "
                             f"{blen[edge]:.2f}mm long: it cannot sit on "
                             f"that edge at all"),
                    measured={'pad_extent_mm': round(ext, 4), 'edge': edge},
                    expected={'edge_length_mm': round(blen[edge], 4)}))
        for edge, row in sorted(edge_rows.items()):
            lo, hi, basis = edge_span(ctx.gate, bounds, edge, outline)
            span = (hi - lo) if lo is not None else blen[edge]
            row['span_mm'] = round(span, 3)
            row['span_basis'] = basis
            if row['claimed_mm'] > span + legality.EPS:
                out.append(Violation(
                    rule='plan_edge_crowded',
                    severity=intent.severity_of('plan_edge_crowded', WARN),
                    message=(f"the {edge} edge's claimed parts "
                             f"({', '.join(row['parts'])}) span "
                             f"{row['claimed_mm']:.2f}mm by pads against a "
                             f"{span:.2f}mm edge ({basis}) -- a flange may "
                             f"overhang a corner, so this is a warning, not "
                             f"a verdict"),
                    measured={'claimed_mm': row['claimed_mm'], 'edge': edge},
                    expected={'span_mm': round(span, 4)}))
    measured['edges'] = sorted(edge_rows.values(), key=lambda r: r['edge'])

    # 7. interior utilisation
    try:
        g0 = _opts.grow_board(pcb_data, pcb_file, clearance=0.0,
                              board_edge_clearance=0.0, assembly_sides=None)
    except Exception as exc:                                # noqa: BLE001
        g0 = {'error': f"{type(exc).__name__}: {exc}"}
    m0 = g0.get('measured') or {}
    util0 = m0.get('utilisation')
    measured['utilisation_per_face_clearance0'] = util0
    # The same soundness argument as the zone bound: parts may overlap in
    # the grade up to a declared budget, so "does not fit by area" is an
    # ERROR only when the forced overlap exceeds it.
    excess0 = ((m0.get('charged_area_mm2') or 0.0)
               - (m0.get('usable_area_mm2') or 0.0))
    # ...and only while the plan also keeps every part ON the board: the
    # grade confines parts to the outline through an `oob_count` budget of
    # 0, and a part allowed off it takes its area with it (round-2
    # verifier: two 8x8 parts on a 10x10 board graded clean with one off).
    on_board = (intent.legality_budget or {}).get('oob_count') == 0
    if g0.get('fits_by_area') is False and overlap_budget is not None \
            and on_board \
            and excess0 > float(overlap_budget) + legality.EPS:
        out.append(Violation(
            rule='plan_board_overfull',
            severity=_plan_severity(intent, 'plan_board_overfull',
                                    ('legality',)),
            message=(f"the parts do not fit on the board by AREA alone, "
                     f"even at zero clearance on the busiest face "
                     f"(utilisation {util0}): they force at least "
                     f"{excess0:.2f}mm2 of courtyard overlap, over the "
                     f"declared legality_budget.overlap_area "
                     f"{float(overlap_budget):g}"),
            measured={'utilisation': util0,
                      'forced_overlap_mm2': round(excess0, 4)},
            expected={'utilisation': '<= 1.0',
                      'overlap_area': float(overlap_budget)}))
    elif util0 is not None and util0 >= _opts.CROWDED_UTILISATION:
        out.append(Violation(
            rule='plan_board_crowded',
            severity=intent.severity_of('plan_board_crowded', WARN),
            message=(f"the busiest face is {util0:.0%} covered by courtyard "
                     f"at zero clearance"
                     + ("" if g0.get('fits_by_area') is not False else
                        ", MORE than its usable area")
                     + f" -- past the {_opts.CROWDED_UTILISATION:.0%} this "
                       f"toolchain calls comfortable to route. Several "
                       f"shipped boards run hotter, so it is a warning: "
                       f"expect to route between parts with little room"),
            measured={'utilisation': util0},
            expected={'utilisation': f'< {_opts.CROWDED_UTILISATION}'}))
    sides = intent.assembly_sides()
    if sides in ('F', 'B'):
        try:
            g1 = _opts.grow_board(pcb_data, pcb_file, clearance=0.0,
                                  board_edge_clearance=0.0,
                                  assembly_sides=sides)
        except Exception:                                   # noqa: BLE001
            g1 = {}
        util1 = (g1.get('measured') or {}).get('utilisation')
        measured['utilisation_one_face_clearance0'] = util1
        if g1.get('fits_by_area') is False:
            out.append(Violation(
                rule='plan_board_crowded',
                severity=intent.severity_of('plan_board_crowded', WARN),
                message=(f"the declared assembly policy puts every part on "
                         f"{sides}.Cu, and on that one face they do not fit "
                         f"by area (utilisation {util1}). Nothing moves a "
                         f"part between faces (#836), so this is the "
                         f"policy's problem, not the plan's"),
                measured={'utilisation': util1},
                expected={'utilisation': '<= 1.0'}))
    out.sort(key=lambda v: v.sort_key())
    return out, measured


def _grade_ctx(intent: Intent, pcb_data, pcb_file: str, *,
               group_sources: Sequence[str] = (),
               clearance: Optional[float] = None,
               board_edge_clearance: Optional[float] = None):
    """`(ctx, outline, state, blocks, block_problems)`: the one setup `grade`
    and `rule_roster` share, so a plan's roster reads the board facts its
    grade will read."""
    from .quench import QuenchState
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
    ctx.requested_floors = (clearance, board_edge_clearance)
    return ctx, outline, state, blocks, block_problems


def grade(intent: Intent, pcb_data, pcb_file: str, *,
          group_sources: Sequence[str] = (), clearance: Optional[float] = None,
          board_edge_clearance: Optional[float] = None,
          with_health: bool = False, with_roster: bool = False,
          brief_fragment=None, mechanical=None,
          mechanical_skip: Sequence[str] = (),
          reconciliation=None) -> GradeResult:
    """Measure a board against its declared floorplan intent.

    `with_roster` (#959) also builds the rule roster -- which rules the intent
    leaves dark and whether a disposition answers for each. Opt-in, because
    it costs a decap census and most callers (the seeder's self-grade, the
    A/B harness) never read it."""
    from . import placement_state

    ctx, outline, state, blocks, block_problems = _grade_ctx(
        intent, pcb_data, pcb_file, group_sources=group_sources,
        clearance=clearance, board_edge_clearance=board_edge_clearance)
    ctx.brief_fragment = brief_fragment

    violations = (list(validate_intent(intent)) + list(block_problems)
                  + list(unresolved_keepout_allows(intent, pcb_data))
                  + list(intent_zone_keepout_problems(
                      intent, blocks, pcb_data, pcb_file)))
    # Budget keys the emitter withheld and that are therefore NOT graded.
    # A key present in the budget was declared (by hand, deliberately) and
    # overrides its withholding note.
    abstained = {str(k): str(v)
                 for k, v in (intent.budget_withheld or {}).items()
                 if not _declared_by_hand(intent, str(k))}
    found, ran, skipped = _run_rules(ctx, abstained)
    violations.extend(found)
    violations.extend(exclusive_unsatisfiable(intent, blocks, pcb_data,
                                              pcb_file, state=state))
    if mechanical:
        violations.extend(mechanical_drift(intent, pcb_data, mechanical,
                                           skip=mechanical_skip))
        violations.extend(mechanical_anchor_violations(
            pcb_data, pcb_file, mechanical, skip=mechanical_skip,
            state=state, locked=ctx.locked, outline=outline))
    # #712: a DECLARED along-edge claim this outline cannot support a verdict
    # on joins the same not-derivable channel the withheld budgets use. It is
    # neither a violation nor a pass, and `pass: true` beside a non-zero
    # abstention count is the thing a caller must be able to see.
    abstained.update(ctx.abstained)
    violations.sort(key=lambda v: v.sort_key())

    pin_evidence: Dict[str, object] = {}
    if 'decap_pin_distance' in ran:
        recs = ctx.supply_pins()
        graded = [r for r in recs.values() if r['pins']]
        pin_evidence = {
            'chips': len(recs),
            'chips_graded': len(graded),
            'pins': sum(len(r['pins']) for r in graded),
            'by_channel': {c: sum(1 for r in graded
                                  if r['channel'] == c)
                           for c in ('pintype', 'pinfunction',
                                     'rail_net')},
            'pins_by_channel': {
                c: sum(len(r['pins']) for r in graded
                       if r['channel'] == c)
                for c in ('pintype', 'pinfunction', 'rail_net')},
            # Chips where a LOWER-precedence channel would have named a
            # different pin set -- i.e. where the ladder's ORDER decided
            # the answer. 31 of 85 dual-yield corpus chips disagree, so
            # this is not a rare footnote.
            'order_decided': sorted(r['ref'] for r in graded
                                    if r['agrees_with_next'] is False),
        }

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

    roster = stale = None
    if with_roster:
        roster = _roster(intent, pcb_data, ctx, brief_fragment=brief_fragment)
        stale = stale_dispositions(intent, roster, pcb_data,
                                   reconciliation=reconciliation)

    st = placement_state.assess_placement(pcb_data, pcb_file)
    return GradeResult(
        roster=roster, stale_dispositions=list(stale or ()),
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
        edge_seating=list(ctx.edge_seating),
        proximity_measured=list(ctx.proximity_measured),
        edge_connector_evidence=list(ctx.edge_connector_evidence),
        decap_pin_evidence=pin_evidence,
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


def decap_census(pcb_data, radius: float = None,
                 exclude: Sequence[str] = ()) -> Dict:
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
    # ONE election, partitioned -- not two queries whose relationship has
    # to be argued. The comment that used to stand here said the unbounded
    # pass was "a second measurement rather than a superset" because
    # "nearest chip carrying the rail" could elect a DIFFERENT chip without
    # the prune. That was never true of this code: `radius` only ever
    # appeared AFTER the argmin, and the chip list is built before any cap
    # is considered. Measured, 0 of 357 tethers re-elect. Since #794 the
    # radius does not reach the election at all, so it is a property of the
    # code shape rather than a fact about a corpus -- pinned by
    # `tests/test_792_decap_predicate.py`.
    near, beyond, orphans = groups_mod.decap_populations(pcb_data, radius=r)
    # #959: caps a DECLARED proximity relation supersedes are graded by that
    # relation, so a limit derived for the rest must not be set by them.
    skip = set(exclude or ())
    n_sup = 0
    if skip:
        before = sum(len(cs) for cs in near.values()) + len(beyond)
        near = {ic: [(c, d) for c, d in caps if c not in skip]
                for ic, caps in near.items()}
        near = {ic: caps for ic, caps in near.items() if caps}
        beyond = [row for row in beyond if row[0] not in skip]
        n_sup = before - sum(len(cs) for cs in near.values()) - len(beyond)
    dists = sorted(d for caps in near.values() for _c, d in caps)
    n = len(dists)
    # `beyond_radius_refs` is cap-sorted for determinism (#457);
    # `worst_beyond_mm` is the MAX. Those are two different orderings and
    # conflating them was a real defect: this key used to read
    # `beyond[-1][1]` off the ref-sorted list, so it reported the
    # alphabetically LAST beyond cap rather than the farthest one.
    # Measured, it understated on 7 of the 14 boards that have any --
    # kit-dev 10.80 where the truth is 22.89, interf_u 8.39 where it is
    # 19.82, flat_hierarchy 6.44 where it is 18.42. The number is quoted in
    # `--declare-decaps` stdout, in the withholding reason, and in
    # docs/floorplan-intent.md's censoring table, so all three understated
    # the thing the census exists to disclose.
    # Counted off the predicate rather than off the election's output --
    # but NOT independently of it, and the first draft of this comment
    # claimed otherwise. `_elect_tethers(movable=None)` walks the same
    # footprint dict through the same `is_decoupling_cap` call, so
    # `unaccounted` is 0 by CONSTRUCTION, not by measurement: it catches
    # a future election that drops a cap for some new reason, and it
    # cannot catch today's. Its battery row is recorded as an expected
    # survivor for exactly that reason, paired with one that breaks the
    # election so the tripwire has something to catch.
    scope = sum(1 for _r, _fp in (pcb_data.footprints or {}).items()
                if groups_mod.is_decoupling_cap(_fp, _r))
    out = {
        'source': 'auto-tethers',
        'metric': ('cap footprint centroid to IC bounding box, clamped to 0 '
                   'inside (placement.groups.decap_tethers)'),
        'search_radius_mm': round(r, 4),
        'tethers': n,
        'ics': len(near),
        'beyond_radius': len(beyond),
        'beyond_radius_refs': sorted(c for c, _ic, _d in beyond),
        'worst_beyond_mm': (round(max(d for _c, _ic, d in beyond), 4)
                            if beyond else None),
        # The third cause, which no issue named and which the doc used to
        # blame on a predicate mismatch that does not exist: a cap whose
        # rail NO chip carries. It has no IC to be far from, so the grader
        # is right to ignore it -- ten of ulx3s's are bulk and filter caps
        # upstream of an LC network. The SEEDER is not right to evict it.
        'no_rail_chip': len(orphans),
        'no_rail_chip_refs': sorted(orphans),
        'population': scope,
        # Whose only correct value is 0, emitted on EVERY board. It is what
        # turns "the three arms are the whole scope" from a claim into a
        # number a reader can check, so a future divergence shows up in
        # every emitted document instead of as a silent set difference.
        'unaccounted': scope - n - len(beyond) - len(orphans) - n_sup,
    }
    if n_sup:
        # Left out because a declared relation grades them (#959); counted,
        # so `unaccounted` stays the closure check it is.
        out['superseded_excluded'] = n_sup
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
        # The beyond-population clause is appended HERE too, not only on
        # the zero-tether arm (#794). Without it `interf_u_unrouted` --
        # withheld for having one sample, with FOUR caps beyond the radius
        # and a 19.82mm worst -- abstains with a note that is silent about
        # them. This channel is how a withholding board learns it has a
        # horizon problem, since `decap_ungraded` cannot arm without a
        # limit; a channel that covers one withholding reason and not the
        # others only looks complete.
        return None, (f"only {n} tether(s) on the emitting board -- a max "
                      f"over {n} sample(s) is a coordinate, not a limit "
                      f"(DECAP_MIN_SAMPLE={DECAP_MIN_SAMPLE})"
                      + (f" ({census['beyond_radius']} rail-sharing cap(s) "
                         f"lie beyond the {census['search_radius_mm']}mm "
                         f"search radius, worst "
                         f"{census['worst_beyond_mm']}mm)"
                         if census.get('beyond_radius') else ""))
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


def _decap_mode(v) -> str:
    """`derive_decaps` as one of 'off' | 'strict' | 'auto'. A bool is the
    old spelling (True was the strict flag). Compared by EQUALITY everywhere:
    'off' is a truthy string, and a truthiness test would derive under it."""
    if v is True:
        return 'strict'
    if v is False or v is None:
        return 'off'
    if v in ('off', 'strict', 'auto'):
        return v
    raise ValueError(f"derive_decaps {v!r}: expected 'off', 'strict' or "
                     f"'auto'")


def _emitted_basis(decaps, budget, conns, blocks) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if 'max_distance_mm' in (decaps or {}):
        out['decaps.max_distance_mm'] = 'observed_baseline'
    for k in sorted(budget or {}):
        out[f'legality_budget.{k}'] = 'observed_baseline'
    for c in conns or ():
        for k in ('edge', 'overhang_mm'):
            if k in c:
                out[f"edge_connectors[{c['ref']}].{k}"] = 'observed_baseline'
    for b in blocks or ():
        if 'side' in b:
            out[f"blocks[{b['name']}].side"] = 'observed_baseline'
    return out


def emit_intent(pcb_data, pcb_file: str, *,
                group_sources: Sequence[str] = ('kicad', 'sheet'),
                zone_pad_mm: float = 1.0,
                declare_classes: bool = False,
                derive_decaps='off', brief_fragment=None) -> Dict:
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
            g = _leg.grade_pad_legality(pcb_data, state.clearance, worst_n=0,
                                        edge_margin=state.edge_gate.margin,
                                        pcb_file=pcb_file)
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
    body_geometry = None     # #961: built only if an edged entry is emitted
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
                # #961: an edged entry's band is GRADED on the drawn body
                # when one can be measured, so a band observed only on the
                # occupancy reading can come out narrower than the body it
                # blesses -- a pad-box courtyard 1.6 mm inboard of a flush
                # body is esp_prog's USB1. Widen to the body in that case
                # alone, and say so: where the body reads no more than `amt`
                # the emitted band is unchanged. Either way `max` is at least
                # the number the rule grades (`body_outside_mm`, or `amt`
                # itself when no body is measured) plus 0.5, so an edged
                # entry still grades its band clean by construction. Measured
                # on the 22 tracked boards: the widening never fires.
                if body_geometry is None:
                    from .connector_geometry import ConnectorGeometry
                    body_geometry = ConnectorGeometry(pcb_data, pcb_file)
                body = body_geometry.measure(ref, entry['edge'])
                if (body['body_measured']
                        and body['body_outside_mm'] > amt + legality.EPS):
                    entry['overhang_mm']['max'] = round(
                        body['body_outside_mm'] + 0.5, 3)
                    entry['note'] = (
                        f"band max from the drawn body's "
                        f"{body['body_outside_mm']:.3f}mm overhang "
                        f"({body['body_layer']}), wider than the occupancy "
                        f"reading {amt:.3f}mm it would otherwise use")
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
    else:
        _withheld['oob_count'] = (
            'an edge connector on the emitting board sits in a SUSPECT pose '
            '(see its edge_connectors note), so its overhang is not a '
            'baseline to bless')

    # #704. The census runs on EVERY emission: a reader of the document must
    # be able to tell "no cap is far from its IC" from "nobody measured", and
    # `decaps: {}` alone says only the second. The LIMIT is opt-in, because
    # declaring it is not a grading-only change -- see the seeder note below.
    # #959: caps the BRIEF's own proximity relations supersede are graded by
    # those relations, so a derived limit is read off the rest.
    _sup = (superseded_caps(pcb_data, (brief_fragment or {}).get('proximity'),
                            brief_fragment) if brief_fragment else {})
    _census = decap_census(pcb_data, exclude=sorted(_sup))
    if _sup:
        _census['superseded'] = dict(sorted(_sup.items()))
    _decaps: Dict[str, object] = {}
    _mode = _decap_mode(derive_decaps)
    _derive = _mode == 'strict'
    if _mode == 'auto':
        # #959 (#1002): derive only off a PLACED board. Run 29's pile read
        # `unplaced: false` and `partially_unplaced: true` (duplicate
        # fraction 0.833), and a strict derivation there wrote a limit of 0.0.
        from .placement_state import assess_placement
        _st = assess_placement(pcb_data, pcb_file)
        if _st.unplaced or _st.partially_unplaced:
            _census['auto_withheld'] = (
                'the board is not placed ('
                + '; '.join(_st.reasons[:2])
                + '): a limit read off it would bless a pile -- run 29\'s '
                  'read 0.0')
        else:
            _derive = True
    if _derive:
        _limit, _why = _decap_derivation(_census)
        if _limit is None:
            if _mode == 'auto':
                # Recorded where the roster reads it, NOT in budget_withheld:
                # `auto` promises the default emit changes no exit code.
                _census['auto_withheld'] = _why
            else:
                _withheld['decaps.max_distance_mm'] = _why
        else:
            _decaps['max_distance_mm'] = _limit
            # Repeated inside the census DELIBERATELY: a hand edit of
            # `decaps.max_distance_mm` that leaves the census behind is then
            # detectable rather than a silent lie about where the number came
            # from.
            _census['emitted_max_distance_mm'] = _limit
            _census['decaps_basis'] = 'observed_baseline'
    # What declaring this key COSTS, measured, next to the number itself.
    # `seeder.seed_from_intent` uses its presence to pull caps out of radial
    # zone packing into stage 2.5, which seats one cap per supply pin -- a
    # different question from the one graded here, and since #792 a
    # different SET: the seeder takes the caps that elect a tether at any
    # distance, because a cap no chip's rail touches has no pin to be
    # seated at, ever.
    #
    # This used to compute a THIRD spelling of "is this a decap" inline --
    # case-blind like the grouper, pad-counting like the seeder -- and
    # report the gap as one number, `seeder_scope_ungraded`. Both are gone.
    # The number conflated three causes and the doc explained it with a
    # FOURTH that does not exist: measured over every tracked board, the
    # three predicates name identical sets and the residue attributable to
    # them is 0. ulx3s's famous 17 is 7 beyond the radius plus 10 whose
    # rail no chip carries. The census now reports those separately, and
    # `unaccounted` stays 0 to say the three arms are the whole scope.
    _census['seeder_pin_scope'] = (_census['tethers']
                                   + _census['beyond_radius'])

    # #837. An OBSERVATION, so `why` records how it was reached rather than a
    # reason nobody gave. An emitted intent describes a board; it does not make
    # demands of it, which is the same rule `must_lock` is emitted empty for.
    from placement.legality import assembly_census as _ac
    _cen = _ac(pcb_data)
    # `sides` is None when NO face carries a pad-bearing part. The key is
    # omitted entirely then, so the grade reports "the intent declares no
    # assembly.sides" -- a board with nothing on it has no policy to observe,
    # and 'both' would read identically to a real two-sided board.
    _assembly = {} if _cen['sides'] is None else {
        'sides': _cen['sides'],
        'why': (f"observed: {_cen['pad_bearing']['F']} pad-bearing part(s) on "
                f"F and {_cen['pad_bearing']['B']} on B, "
                f"{_cen['reflow_passes']} reflow pass(es). "
                f"{_cen['basis']}"),
    }
    return {
        'schema': SCHEMA_VERSION,
        'kind': KIND,
        'board': os.path.basename(pcb_file),
        'units': 'mm',
        'envelope': {'rect': [round(v, 4) for v in bounds],
                     'tolerance_mm': DEFAULT_ENVELOPE_TOLERANCE_MM},
        'defaults': {'zone_tolerance_mm': DEFAULT_ZONE_TOLERANCE_MM},
        # #837. The OBSERVED policy, never an assumed one: a board with parts
        # on both faces emits 'both', which arms nothing, so every existing
        # board still grades clean by construction and the round-trip gates
        # stay honest. `single` appears only where a human typed it -- which is
        # the case where the violations are the point.
        #
        # Read off the PAD-BEARING census, so esp_prog -- whose three back-side
        # blocks are zero-pad OLIMEX logos -- emits 'F', which is what the fab
        # builds. `_assembly_observed` says which rule produced it.
        'assembly': _assembly,
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
            # #959 comment 3.2: every number this emitter chose, labelled as
            # what it is -- a baseline OBSERVED on this board, not a
            # requirement anyone declared. Keyed by intent path; a brief
            # merged over it re-labels what it declares.
            'basis': _emitted_basis(_decaps, _budget, conns, blocks),
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
    if r.violations and not r.complete:
        # A board can be BOTH wrong and incompletely graded, and the second
        # fact does not stop mattering because the first is true. The first
        # draft printed the incompleteness only on the no-violation branch,
        # so a board with a single warning lost the disclosure entirely.
        lines.append(
            "  ALSO NOT FULLY GRADED: " + ', '.join(
                f'{n} {k}' for k, n in sorted(r.not_graded.items()))
            + " -- the violations below are what WAS measured, not all of it")
    if not r.violations and not r.complete:
        # NOT "PASS". A clean sweep of the channels that ran says nothing
        # about the ones that did not, and this line used to print 26 lines
        # above "N declared value(s) NOT DERIVABLE -- not graded, not passed"
        # in the same report (#713 item 5).
        lines.append(
            f"  INCOMPLETE: {len(r.rules_run)} rule(s) ran with no "
            f"violations, but " + ', '.join(
                f'{n} {k}' for k, n in sorted(r.not_graded.items()))
            + " -- this board was not fully graded")
    elif not r.violations:
        lines.append(f"  PASS: {len(r.rules_run)} rule(s) ran, no violations")
    else:
        lines.append(f"  {len(r.errors)} error(s), {len(r.warnings)} warning(s) "
                     f"from {len(r.rules_run)} rule(s)")
        for v in r.violations:
            tag = 'ERROR' if v.severity == ERROR else 'warn '
            lines.append(f"    [{tag}] {v.rule}: {v.message}")
    rows = [e for e in r.edge_seating
            if e.get('along_edge_offset_mm') is not None]
    if rows:
        rows.sort(key=lambda e: -abs(float(e['along_edge_offset_mm'])))
        lines.append(f"  along-edge seating (measured, not a verdict -- a "
                     f"violation needs a declared center_on_edge or "
                     f"along_edge_band):")
        for e in rows[:5]:
            mark = ' DECLARED' if e.get('declared') else ''
            lines.append(f"    {e['ref']} {e['edge']}: "
                         f"{e['along_edge_offset_mm']:+.2f}mm "
                         f"({e['along_edge_offset_pct']:+.1f}% of the "
                         f"{e['span_mm']:.2f}mm edge, {e['basis']}){mark}")
        if len(rows) > 5:
            lines.append(f"    ... {len(rows) - 5} more")
    if r.edge_connector_evidence:
        # #961: the number each band was graded on and its CURRENCY, printed
        # on a passing clause too -- a body reading and a legacy occupancy
        # reading are different claims about the same connector.
        lines.append("  edge connector overhang (measured; the band's own "
                     "currency is named):")
        for e in r.edge_connector_evidence:
            cu = e.get('pad_copper_edge') or {}
            gap = cu.get('minimum_gap_mm')
            lines.append(
                f"    {e['ref']} {e.get('edge') or '(no edge)'}: overhang "
                f"{e['overhang_mm']:.4f}mm [{e['overhang_basis']}] "
                f"{e['overhang_disposition']}; pad copper "
                + (f"{gap:.4f}mm" if gap is not None else 'unmeasured')
                + f" vs {cu.get('required_mm')}mm {cu.get('disposition')}")
    ev = r.decap_pin_evidence or {}
    if ev:
        # WHAT the pin rule graded, printed whether or not it found anything.
        # A clean pass over 39 inferred pins and a clean pass over 39 declared
        # ones are different results, and `rules_run` cannot tell them apart.
        by = ev.get('pins_by_channel') or {}
        lines.append(
            f"  decap pins: {ev.get('pins', 0)} on "
            f"{ev.get('chips_graded', 0)} of {ev.get('chips', 0)} candidate "
            f"IC(s) -- " + ', '.join(f"{k} {v}" for k, v in sorted(by.items())
                                     if v))
        if ev.get('order_decided'):
            lines.append(
                f"    {len(ev['order_decided'])} chip(s) where a lower channel "
                f"would have named a DIFFERENT pin set "
                f"({', '.join(ev['order_decided'][:6])}"
                + (", ..." if len(ev['order_decided']) > 6 else "") + ")")
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
    if r.roster is not None:
        lines.extend(format_roster(r.roster, r.stale_dispositions))
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
                # #700: printed ONLY when the face is short even after every
                # other signal layer is counted. `deficit_floor == 0` proves
                # nothing -- it is a lower bound -- so emitting it there would
                # read as an all-clear on the line under a real deficit.
                if w.get('deficit_floor'):
                    lines.append(
                        f"        still short {w['deficit_floor']} with "
                        f"{p.get('signal_layers')} signal layer(s) "
                        f"({p.get('signal_layers_source')}), bounded by "
                        f"{w.get('supply_bound')}"
                        + (f" -- {w['via_slots']} via slot(s) along the face"
                           if w.get('supply_bound') == 'via_slots' else ''))
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


def format_roster(rows, stale=()) -> List[str]:
    """The roster as report lines: armed / dark-with-reason / dispositioned,
    and what a plan still owes. Shared by the grade report and the emit path,
    so both print the same thing."""
    armed = [r['rule'] for r in rows if r['state'] == 'armed']
    owed = roster_refusal_lines(rows)
    n_owed = sum(1 for r in rows if r['needs_disposition'])
    lines = [f"  rule roster: {len(armed)} armed, {len(rows) - len(armed)} "
             f"not; {n_owed} rule(s) owe a written answer"]
    for r in rows:
        if r['state'] == 'armed' and not r['withheld']:
            continue
        if r['needs_disposition']:
            tag = 'OWED'
        elif r['disposition'] or r['withheld_dispositions']:
            tag = 'dispositioned'
        elif r.get('brief_claims') and r['state'] != 'armed':
            tag = 'brief: not carried'
        elif r['policy']:
            tag = 'policy'
        elif not r['applicable']:
            tag = 'not applicable'
        elif not r['gating']:
            tag = 'advisory'
        else:
            tag = 'armed'
        why = (r['disposition']
               or '; '.join(r['withheld_dispositions'].values())
               or (r['applicability_reason'] if tag in (
                   'policy', 'not applicable', 'brief: not carried')
                   else r['skip_reason'])
               or '; '.join(f"`{k}` withheld: {v}"
                            for k, v in r['withheld'].items()))
        lines.append(f"    [{tag}] {r['rule']}: {why}")
    for line in owed:
        lines.append(f"    OWED: {line}")
    for s in stale:
        lines.append(f"    STALE: {s}; remove it")
    return lines


#: #959 (#997): every status a declaration-ledger row can carry. `pending`
#: is an armed row with no grade yet (the before-placement view); `dark` is
#: an applicable gating rule nothing answers for; `carried` is a declared
#: fact no rule measures.
LEDGER_STATUSES = ('pending', 'graded_pass', 'graded_fail', 'graded_warn',
                   'carried',
                   'unmeasured', 'unknown', 'uncovered', 'inapplicable',
                   'abstained', 'dispositioned', 'dark', 'policy',
                   'advisory')


def declaration_ledger(intent: Intent, rows, *, result=None,
                       coverage=None, brief_source=None,
                       reconciliation=None,
                       consequences=None) -> List[Dict[str, object]]:
    """One row per REQUIREMENT, whoever declared it (#959 comment §3.1).

    Joins the rule roster (what the intent arms and leaves dark) with the
    brief's clause coverage (what the brief declares and which rule, if any,
    measures it). Each row says who declared it, what it compiled to, which
    rule grades it and with what outcome -- so "complete" can never again
    mean "every graded clause passed" while eight carried facts sit unread
    beside it.

    `result` is the grade, when there is one; without it an armed row is
    `pending`, which is the plan's view before any pose exists.

    `graded_fail` on a brief clause is attributed per (rule, ref): the rule
    that grades the clause reported an ERROR for that ref. A connector whose
    EDGE is wrong therefore marks its along-edge clause failed too -- the
    grader's findings name a rule and a ref, not a clause key, and inventing
    a finer attribution than the grader reports would be a claim nothing
    measured. `attribution` says so on every such row.
    """
    src = intent.source_path or None
    by_rule_err: Dict[str, set] = {}
    # A keep-out finding names the INTRUDER, never the keep-out's owner, so
    # a clause about a keep-out is judged by the keep-out's NAME; and the
    # side finding is a fixed WARN, so it has its own set -- read as an
    # ERROR it would never fire, and ignored it reads a WARN as a pass
    # (Phase-5 verifier S1).
    ko_err: set = set()
    side_warn: set = set()
    if result is not None:
        for v in result.violations:
            if v.severity == ERROR:
                by_rule_err.setdefault(v.rule, set()).add(v.ref or '')
                if v.rule == 'keepout':
                    ko_err.add(str((v.measured or {}).get('keepout') or ''))
            if v.rule == 'edge_connector_side':
                side_warn.add(v.ref or '')

    def _verdict(grader, ref, keepout=None):
        if result is None:
            return 'pending'
        if grader == 'keepout':
            return 'graded_fail' if keepout in ko_err else 'graded_pass'
        if grader == 'edge_connector_side':
            hit = side_warn if ref is None else side_warn & {ref}
            return 'graded_warn' if hit else 'graded_pass'
        return ('graded_fail' if (ref or '') in by_rule_err.get(grader, set())
                else 'graded_pass')
    out: List[Dict[str, object]] = []
    for r in rows or ():
        name = r['rule']
        # OWED first, armed or not: a rule armed with a budget key nobody
        # answers is not a pass on that key (Phase-1 verifier: legality read
        # `graded_pass` while the roster printed it OWED).
        if r['needs_disposition']:
            status = 'dark'
        elif r['state'] == 'armed':
            if result is None:
                status = 'pending'
            elif name in by_rule_err:
                status = 'graded_fail'
            else:
                status = 'graded_pass'
        elif r['disposition'] or (r['withheld'] and not [
                k for k in r['withheld']
                if k not in r['withheld_dispositions']]):
            status = 'dispositioned'
        elif r.get('brief_claims'):
            status = 'uncovered'
        elif r['state'] == 'abstained':
            status = 'abstained'
        elif r['policy']:
            status = 'policy'
        elif not r['applicable']:
            status = 'inapplicable'
        elif not r['gating']:
            status = 'advisory'
        else:
            status = 'inapplicable'
        out.append({
            'id': f"rule:{name}", 'kind': 'rule', 'source': src,
            'source_reason': (None if src else
                              'an in-memory intent has no file'),
            'authority': 'hypothesis', 'consequence': (
                r['arming_key'] if r['state'] == 'armed' else None),
            'grader': name, 'status': status,
            'basis': 'declared',
            'why': (r['disposition']
                    or ('; '.join(f"`{k}` withheld: {v}"
                                  for k, v in r['withheld'].items()
                                  if k not in r['withheld_dispositions'])
                        if status == 'dark' and r['state'] == 'armed'
                        else '')
                    or r['skip_reason'] or r['applicability_reason']),
            'disposition': r['disposition'] or None,
        })
    state_map = {'carried': 'carried', 'not_claimed': 'unknown',
                 'uncovered': 'uncovered', 'abstained': 'abstained'}
    for c in (coverage or {}).get('clauses') or ():
        st = c.get('state')
        if st == 'graded':
            grader = c.get('grader') or c.get('rule') or ''
            status = _verdict(
                grader, c.get('ref'),
                keepout=(c.get('keepout') or (
                    c.get('ref') if c.get('kind') == 'keepouts' else None)))
        elif c.get('unmeasured'):
            status = 'unmeasured'
        else:
            status = state_map.get(st, st)
        out.append({
            'id': c['id'], 'kind': 'brief_clause',
            'source': brief_source,
            'authority': 'declared',
            'consequence': c.get('rule') if st == 'graded' else None,
            'grader': c.get('rule'), 'status': status, 'basis': 'declared',
            'attribution': 'rule+ref',
            'why': c.get('why') or '', 'drifted': bool(c.get('drifted')),
            'disposition': None,
        })
    # #959 (#1000): what the brief's connector declarations COMPILED to,
    # with the basis of each number -- a `derived_default` is listed apart
    # from a declared value so it is never read as a validated claim.
    for r in consequences or ():
        grader = r.get('grader')
        cstat = r.get('status')
        if cstat == 'unmeasured':
            status = 'unmeasured'
        elif cstat == 'carried':
            status = 'carried'
        elif cstat != 'compiled':
            status = 'abstained'
        else:
            to = str(r.get('compiled_to') or '')
            status = _verdict(grader, r.get('ref'),
                              keepout=(to[len('keepouts['):-1]
                                       if to.startswith('keepouts[')
                                       else None))
        out.append({
            'id': f"derived:{r['id']}", 'kind': 'derived_clause',
            'source': brief_source,
            # A default dimension is nobody's declaration: the DECLARATION
            # (`mount_mode`) is the brief's, the number is this code's
            # (verifier N5) -- the same authority a staging default has.
            'authority': ('assumption'
                          if r.get('basis') == 'derived_default'
                          else 'declared'),
            'consequence': r.get('compiled_to'), 'grader': grader,
            'status': status, 'basis': r.get('basis') or 'declared',
            'attribution': 'rule+ref', 'why': r.get('why') or '',
            'value': r.get('value'), 'disposition': None,
        })
    # #959 (#1001): every ref two channels speak to. A contradiction is
    # failed until the plan acknowledges it; drift the BOARD alone loses is
    # pending before a grade (a pile is expected to disagree with where a
    # part will go) and failed after one; drift a plan loses is failed --
    # P1 refuses it; a floors row is reported, not graded.
    answered = (intent.dispositions or {}).get('contradictions', {})
    for r in reconciliation or ():
        kind = r.get('kind')
        if kind == 'report':
            # A floor staging ASSUMED, reported beside the one graded: not a
            # declared fact, so not a ledger row that `carried_facts` would
            # count as one. It stays in `context.reconciliation`.
            continue
        winner = r.get('winner')
        wv = (r.get('values') or {}).get(winner) or {}
        losers = {v.get('authority') for ch, v in (r.get('values')
                                                   or {}).items()
                  if ch != winner and v.get('value') is not None}
        if kind == 'contradiction':
            status = ('dispositioned' if r['id'] in answered
                      else 'graded_fail')
        elif kind == 'drift':
            status = ('pending' if result is None and losers <= {'inferred'}
                      else 'graded_fail')
        elif kind == 'agree':
            status = 'graded_pass'
        else:
            status = 'carried'
        out.append({
            'id': f"reconcile:{r['id']}", 'kind': 'reconciliation',
            'source': wv.get('source'),
            'authority': wv.get('authority'),
            'consequence': None, 'grader': 'reconciliation',
            'status': status,
            'basis': 'mechanical' if winner == 'mechanical' else 'declared',
            'why': r.get('why') or '',
            'disposition': answered.get(r['id']),
            'values': {ch: v.get('value') for ch, v in (r.get('values')
                                                        or {}).items()},
            'winner': winner,
        })
    return out


def ledger_summary(ledger) -> Dict[str, object]:
    """The JSON_SUMMARY view of the ledger: counts by status, and the facts a
    reader must not mistake for checked ones, by id."""
    counts = {s: 0 for s in LEDGER_STATUSES}
    for row in ledger:
        counts[row['status']] = counts.get(row['status'], 0) + 1
    return {
        'ledger_status': {k: v for k, v in counts.items() if v},
        # A `derived:` row that is carried (an exemption, a restatement)
        # always stands beside the DECLARED clause it came from, which is
        # listed already -- the fact is counted once, by its own id.
        'carried_facts': sorted(r['id'] for r in ledger
                                if r['status'] == 'carried'
                                and r.get('kind') != 'derived_clause'),
        'unmeasured_facts': sorted(r['id'] for r in ledger
                                   if r['status'] == 'unmeasured'),
        'derived_default_clauses': sorted(
            r['id'] for r in ledger if r.get('kind') == 'derived_clause'
            and r.get('basis') == 'derived_default'),
    }


def to_json(r: GradeResult) -> Dict:
    return {
        'rule_roster': r.roster,
        'stale_dispositions': list(r.stale_dispositions),
        'schema': SCHEMA_VERSION,
        'board': r.board,
        'intent': r.intent.source_path,
        'pass': r.passed,
        # #713 item 5. `complete` is the repo's existing name for "this run
        # did not measure everything" (route_summary.py, refused on by
        # place_route_loop), reused rather than a second channel invented.
        # `not_graded` names WHICH channel, because an aggregate cannot say
        # which of its inputs moved (#694).
        'complete': r.complete,
        'not_graded': r.not_graded,
        'violations': [v.to_dict() for v in r.violations],
        'blocks': {k: list(v) for k, v in sorted(r.blocks.items())},
        'legality': r.legality,
        'outline': r.outline,
        'state': r.state,
        'health': r.health,
        'rules_run': list(r.rules_run),
        'rules_skipped': r.rules_skipped,
        'budget_abstained': r.budget_abstained,
        'edge_seating': r.edge_seating,
        'edge_connector_evidence': r.edge_connector_evidence,
        'decap_pin_evidence': r.decap_pin_evidence,
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
        'complete': r.complete,
        'not_graded': r.not_graded,
        'violations': len(r.violations),
        'errors': len(r.errors),
        'warnings': len(r.warnings),
        'violations_by_rule': by_rule,
        'rules_run': len(r.rules_run),
        'rules_skipped': len(r.rules_skipped),
        # #959: None (not []) when no roster was built, so "nothing owed" and
        # "nobody asked" never read the same.
        'rules_dark_undispositioned': (None if r.roster is None
                                       else r.dark_undispositioned),
        # Not a violation count and not a pass: channels nothing graded.
        'budget_abstained': len(r.budget_abstained),
        'edge_seating_rows': len(r.edge_seating),
        'edge_seating_declared': sum(1 for e in r.edge_seating
                                     if e.get('declared')),
        'edge_seating_abstained': sum(1 for e in r.edge_seating
                                      if e.get('abstained')),
        'edge_seating_worst_offset_pct': (
            max((abs(float(e['along_edge_offset_pct']))
                 for e in r.edge_seating
                 if e.get('along_edge_offset_pct') is not None),
                default=None)),
        'decap_pins_graded': (r.decap_pin_evidence or {}).get('pins', 0),
        'decap_pins_by_channel': (r.decap_pin_evidence
                                  or {}).get('pins_by_channel', {}),
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


# --------------------------------------------------------------------------
# #974: the declared connector requirements, reported -- never a gate
# --------------------------------------------------------------------------

#: What an `overhang_evidence` row keeps of the grade's evidence row: the
#: band's number, currency, limit and verdict, and the copper conjunct's
#: verdict and amounts -- with the grade's lists as COUNTS. JSON_SUMMARY is
#: one stdout line, and the whole rows (body position, grade-wide basis
#: strings repeated per row, per-pad lists) measured 26.7 KB for 28 declared
#: connectors on kit-dev-coldfire's emitted intent. They stay in
#: `check_floorplan --json`'s `edge_connector_evidence`.
_EVIDENCE_ROW_KEYS = ('ref', 'edge', 'overhang_mm', 'overhang_basis',
                      'overhang_limit_mm', 'overhang_disposition',
                      'body_measured')
_EVIDENCE_COPPER_KEYS = ('disposition', 'outside_mm', 'certified',
                         'minimum_gap_mm', 'required_mm')
_EVIDENCE_COUNTED_LISTS = ('findings', 'unmeasured', 'rules_unmeasured')

#: `unmeasured[].reason` for a declared ref the grade left no evidence row
#: for, when no "not on this board" finding explains it.
NO_EVIDENCE_ROW = 'the grade produced no evidence row for it'

#: `unmeasured[].reason` for a declared along-edge claim that was neither
#: measured nor abstained -- `_grade_along_edge` returns silently on a
#: zero-length edge span.
NO_ALONG_EDGE_MEASUREMENT = 'no along-edge measurement recorded'

#: `bands_dropped[].reason`.
BAND_DROPPED_REASON = ("--reseat set this edge declaration aside because its "
                       "ref is in the re-seat scope, so none of its conjuncts "
                       "was graded, whether or not the ref moved")


def connector_requirements_ungraded(reason: str) -> Dict[str, object]:
    """`connector_requirements` for a run that graded nothing (#974)."""
    return {'complete': False, 'reason': str(reason)}


def connector_requirements(graded: GradeResult, own: Sequence[Violation],
                           pinned: Sequence[Violation], *,
                           bands_dropped=None) -> Dict[str, object]:
    """What a written board's grade said about its DECLARED edge connectors,
    as JSON_SUMMARY data (#974): abstain and report, never withhold.

    `own` / `pinned` are the caller's own split of `graded.errors` -- the
    SAME lists that decide its exit code -- so `errors_own` is non-empty
    exactly when a connector error counted against the run. The split is
    never recomputed here. `bands_dropped` is `seeder.reseat_scope`'s
    `edge_bands_dropped` ({ref: band max}): entries the grade never saw.

    `complete` means every declared requirement was MEASURED; it says nothing
    about whether they passed, and it is not `graded.complete`, which also
    counts channels that have nothing to do with connectors.

    Report-only, so it never raises: a report that crashed after the board
    was written would turn the caller's exit code into 1.
    """
    try:
        return _json_plain(_connector_requirements(graded, own, pinned,
                                                   bands_dropped))
    except Exception as exc:  # noqa: BLE001 -- a report must not fail the run
        try:
            detail = str(exc)
        except Exception:  # noqa: BLE001 -- nor may the message of one
            detail = '<unprintable>'
        return connector_requirements_ungraded(
            f"connector_requirements failed: {type(exc).__name__}: {detail}")


def _connector_requirements(graded, own, pinned, bands_dropped):
    declared = list(graded.intent.edge_connectors)
    declared_refs = sorted({str(c['ref']) for c in declared})
    evidence = list(graded.edge_connector_evidence)
    evidence_refs = {str(e['ref']) for e in evidence}
    not_found = {str(v.ref) for v in graded.violations
                 if v.rule == 'edge_connector'
                 and (v.measured or {}).get('found') is False}

    unmeasured = []
    for c in declared:
        ref = str(c['ref'])
        if ref not in evidence_refs:
            unmeasured.append({
                'ref': ref, 'requirement': 'presence',
                'reason': ('not on this board' if ref in not_found
                           else NO_EVIDENCE_ROW)})
            continue
        centre, band = c.get('center_on_edge'), c.get('along_edge_band')
        if centre is None and band is None:
            continue
        claim = 'center_on_edge' if centre is not None else 'along_edge_band'
        # A recorded measurement of THIS entry settles it: the grade abstains
        # only when it appends no measuring row, so an abstention beside one
        # is a hand-written `context.budget_withheld` key. Matched on the
        # entry, not the ref: a ref declared twice gets a measuring row from
        # any entry that names an edge, claim or not. Every row names its
        # edge, so an edgeless entry matches none.
        if any(str(row.get('ref')) == ref and row.get('declared')
               and row.get('edge') == c.get('edge')
               and 'along_edge_offset_mm' in row
               for row in graded.edge_seating):
            continue
        why = graded.budget_abstained.get(f"edge_connectors[{ref}].{claim}")
        unmeasured.append({'ref': ref, 'requirement': claim,
                           'reason': NO_ALONG_EDGE_MEASUREMENT
                           if why is None else why})

    for row in evidence:
        ref = str(row['ref'])
        lim = row.get('overhang_limit_mm') or {}
        # A band no reading can fail -- no max and a zero min -- needs no
        # body to grade it. Emitted `connector_affinity` entries are exactly
        # that, and counting them would mark every emitted intent incomplete.
        vacuous = (lim.get('max') is None
                   and float(lim.get('min') or 0.0) <= legality.EPS)
        if not row.get('body_measured') and not vacuous:
            unmeasured.append({
                'ref': ref, 'requirement': 'overhang_body',
                'reason': row.get('body_unmeasured_reason'),
                'graded_on': row.get('overhang_basis')})
        copper = row.get('pad_copper_edge') or {}
        # The copper-past-the-outline conjunct is graded on the BODY path
        # only. Its CLEARANCE half (and board-wide `.kicad_dru` rules) is
        # evidence, not a requirement, so it never reaches `unmeasured`.
        if row.get('body_measured'):
            if (copper.get('certified') is False
                    or ('outside_mm' in copper
                        and copper['outside_mm'] is None)):
                pads = sorted({f"{u.get('pad_ref')}: {u.get('reason')}"
                               for u in copper.get('unmeasured') or ()})
                unmeasured.append({
                    'ref': ref, 'requirement': 'pad_copper_outside',
                    'reason': ('; '.join(pads) if pads else
                               'the edge grader cannot model a pad shape, so '
                               'its outside_mm is not a certified reading')})
        elif row.get('edge') is not None and (
                copper.get('minimum_gap_mm') is not None
                or copper.get('findings') or copper.get('unmeasured')):
            # An entry that CLAIMS an edge, on a part with copper, whose body
            # could not be read: the conjunct was skipped, whatever its band
            # says. A vacuous band exempts `overhang_body` above, never this.
            unmeasured.append({
                'ref': ref, 'requirement': 'pad_copper_outside',
                'reason': ('pad copper is graded against the outline on the '
                           'drawn-body path only, and this body was not '
                           'measured: '
                           + str(row.get('body_unmeasured_reason')))})

    # Sorted, and only EXACT duplicates removed: a ref declared twice yields
    # the same entries twice, while two requirements on one ref are two
    # entries.
    seen, deduped = set(), []
    for u in sorted(unmeasured, key=lambda u: (u['ref'], u['requirement'],
                                               str(u['reason']),
                                               str(u.get('graded_on')))):
        key = tuple(sorted((k, str(v)) for k, v in u.items()))
        if key not in seen:
            seen.add(key)
            deduped.append(u)

    dropped = [{'ref': str(ref), 'band_max_mm': mm, 'graded': False,
                'reason': BAND_DROPPED_REASON}
               for ref, mm in sorted((bands_dropped or {}).items(),
                                     key=lambda kv: str(kv[0]))]

    projected = []
    for row in evidence:
        copper = row.get('pad_copper_edge') or {}
        slim = {key: row.get(key) for key in _EVIDENCE_ROW_KEYS}
        slim_copper = {key: copper.get(key) for key in _EVIDENCE_COPPER_KEYS}
        for name in _EVIDENCE_COUNTED_LISTS:
            slim_copper['n_' + name] = len(copper.get(name) or ())
        slim['pad_copper_edge'] = slim_copper
        projected.append(slim)

    return {
        'complete': not deduped and not dropped,
        'declared_refs': declared_refs,
        'errors_own': [v.to_dict() for v in own
                       if v.rule == 'edge_connector'],
        'errors_pinned': [v.to_dict() for v in pinned
                          if v.rule == 'edge_connector'],
        'warnings': [v.to_dict() for v in graded.warnings
                     if v.rule == 'edge_connector'],
        'overhang_evidence': projected,
        'unmeasured': deduped,
        'bands_dropped': dropped,
    }


def _json_plain(value):
    """`value` as data a STRICT JSON parser accepts, sharing nothing with its
    source: string keys, lists for tuples and sets, `None` for a non-finite
    float (`json.dumps` would write the invalid token `NaN`), `str()` for
    anything else."""
    import numbers
    if isinstance(value, dict):
        return {str(k): _json_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_plain(v) for v in value]
    if isinstance(value, (set, frozenset)):
        return [_json_plain(v) for v in sorted(value, key=str)]
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, numbers.Integral):
        return int(value)
    if isinstance(value, numbers.Real):
        return float(value) if math.isfinite(value) else None
    return str(value)
