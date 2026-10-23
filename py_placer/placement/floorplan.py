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

from kicad_parser import non_aperture_pads
from . import legality
from . import groups as groups_mod
from .edge_seat import SEATED_CLASSES

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
#: Every downstream consumer then went blind: an off-board census that skips
#: declared edge refs found 5 violators and none of the 11, and a gate read
#: `oob = 4.348` on a board with parts 158 mm out (929.671 without the
#: bands). This generalises the mount_hole refusal below,
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
#: distinguishes an angle somebody chose from a generator default. Declarable,
#: and it changes a verdict, so the rule above mandates the bump.
#: There is NO `rule_rotation` in `RULES`. `place_fanout_clearance --intent`
#: holds a declaration (a cap turns only within its claim, in the comparison
#: run without the decap gate too); `place_pose`'s set/rotate/face take the
#: caller's angle; `arrays[].rotation` is not a block claim and so not in
#: `rotations_for_ref`; and `place_simple` does not read block rotations.
#: Contrast `blocks[].side`, which is declarable and which no search move
#: carries (#836) -- `place_simple` applies a declared face before it
#: anneals -- while rotation is carried by every nudge.
#: 6 (#959, #1000): `edge_connectors[].side` -- the face a connector is on,
#: compiled from the brief's `user_top_side` with a user-facing or
#: perpendicular-cable connector. Declarable and graded (`edge_connector_side`,
#: an advisory WARN), so the rule above mandates the bump -- and an older
#: build refuses the key by name, which is the safety the bump does not add.
#: 7 (#1051, #1052, #1054): `arrays[]` -- parts that form ONE row (members,
#: served part, order, rotation, pitch, axis) -- graded by `array_formation`;
#: `fixed_poses[]` -- an exact pose a part is to be SEATED at, graded by an
#: anchor the grade compiles from each entry; and `blocks[].rigid`, which
#: declared a block one piece. One bump for the three: they
#: arrived together, and each changes a verdict or a placement, so the rule
#: above mandates it. An older build refuses each key by name. `rigid` is
#: refused by name since its only reader, the quench's group phase, was
#: retired (5dccf2fa): a key nothing reads is a constraint its author thinks
#: holds (`_RETIRED_BLOCK_KEYS`).
#: 8 (#1142): `decaps.within_radius_refs` and `decaps.within_radius_mm` -- the
#: caps a `--decaps-from` reference keeps within the tether search radius of
#: their chip, and the radius it was read at. A held cap the graded board
#: leaves beyond the radius is an ERROR (`decap_ungraded`, per cap), where
#: #1102 promoted the rule board-wide and only when the reference kept EVERY
#: cap inside -- which none of the seven #1105 references did. It changes the
#: exit code, so the rule above mandates the bump; an older build refuses
#: both keys by name.
#: 9 (#1255): `assembly.body_edge_mm` -- the least distance a component body
#: may sit from the board edge, an assembler's rule. Declared, it turns
#: check_assembly's `body_edge_band` census from report-only into a
#: `not_buildable` conjunct, so it changes the exit code and the rule above
#: mandates the bump; an older build refuses the key by name.
#: 10 (#1253): `keep_away[]` -- two NET GROUPS no closer than GAP mm, pad edge
#: to pad edge, in route.py's own rule syntax. WARN by default, so it changes
#: the exit code only when the `severity` map promotes it; that it CAN is
#: what the rule above asks about, so the bump is mandated. An older build
#: refuses the key by name.
#: 11 (#1211): `replicate[]` -- sibling instances of one hierarchical sheet
#: laid out as translated copies of a SOURCE instance. `place_pose
#: replicate --intent` acts on it and the grade measures each target's
#: distance from the source arrangement, WARN by default; that a tool moves
#: parts for it and the grade can be promoted is what the rule above asks
#: about. An older
#: build refuses the key by name.
#: 12: a `proximity[]` row whose `context.source` is `placement.crystals`
#: (`DERIVED_CLAIM_SOURCE`) -- a crystal claim `emit_intent` derives --
#: is graded at WARN and not held by the placement gate. An older build
#: reads the same row as a declared claim at the intent's `proximity`
#: severity, ERROR by default, so the verdict differs and the rule above
#: mandates the bump; an intent carrying such rows stamps `min_reader` 12.
#: 13: `templates[]` -- a chip's support parts held at the poses its
#: datasheet layout draws (`placement.templates`): `place_simple` seats and
#: carries them, and the grade's `template` rule (WARN by default) measures
#: each member's drift. A placement tool acts on it and the grade can be
#: promoted, so the rule above mandates the bump; an older build refuses the
#: key by name, and `fit_template.py` stamps `min_reader` 13.
#: 14: a `templates[]` row's `kind` and `ground_return_mm`. A `switching
#: stage` has each member cap's ground return graded
#: (`template_ground_return`, WARN by default), which the `severity` map can
#: promote, so the rule above mandates the bump; an older build refuses both
#: keys by name, and `fit_template.py` stamps `min_reader` 14 on a row that
#: carries either.
READER_VERSION = 14
#: The reader version that first reads `templates[]`.
TEMPLATES_READER = 13
#: The reader version that first reads a template row's `kind` and
#: `ground_return_mm`.
TEMPLATE_KIND_READER = 14

#: #1357. A row's FREE TEXT -- the author's reason for it, which no rule
#: grades -- has ONE meaning and three spellings. `why` is canonical;
#: `note` and `reason` are aliases, accepted on EVERY row type below. The
#: spelling used to depend on the row (fixed_poses and assembly took `why`;
#: proximity, blocks, edge_connectors, keepouts and keep_away `note`;
#: arrays and replicate both; overlap_waivers `reason`), and the #710 gate
#: refused the others, so `why` on a proximity row cost a run its intent.
#: Readers take the text through `free_text`, never one key. The emitter
#: keeps writing `note` for its own remarks, which is an alias, not a
#: second slot. No READER_VERSION bump: free text grades nothing.
FREE_TEXT_KEY = 'why'
FREE_TEXT_ALIASES = ('note', 'reason')
_FREE_TEXT_KEYS = frozenset((FREE_TEXT_KEY,) + FREE_TEXT_ALIASES)


def free_text(row) -> str:
    """A row's free text, whichever spelling carries it (#1357): the values
    of `why`, `note` and `reason`, in that order, joined with '; ' when a
    row carries more than one (an emitted array row may carry a `why` and a
    `note`). '' when there is none."""
    if not isinstance(row, dict):
        return ''
    parts = []
    for k in (FREE_TEXT_KEY,) + FREE_TEXT_ALIASES:
        v = row.get(k)
        if v is not None and str(v).strip() and str(v) not in parts:
            parts.append(str(v))
    return '; '.join(parts)


def _free_text_strings(row, where: str) -> None:
    """Every free-text spelling a row carries is a string (#1357)."""
    for k in (FREE_TEXT_KEY,) + FREE_TEXT_ALIASES:
        if row.get(k) is not None and not isinstance(row[k], str):
            raise IntentError(f"{where}.{k}: expected a string")


_TOP_LEVEL_KEYS = {
    'schema', 'kind', 'board', 'units', 'envelope', 'defaults', 'blocks',
    'keepouts', 'edge_connectors', 'decaps', 'must_lock', 'legality_budget',
    'health', 'severity', 'context', 'overlap_waivers', 'min_reader',
    'assembly', 'proximity', 'dispositions', 'arrays', 'fixed_poses',
    'keep_away', 'replicate', 'templates',
}
#: #959 (#997). WRITTEN answers to "why is this not graded", one map per kind
#: of question: a rule the plan leaves dark, a budget key the emitter withheld,
#: a pad-less block the placer cannot place, a contradiction between two
#: declared sources. Every value is a non-empty `why`. No READER_VERSION bump:
#: the rule above says not to bump for a field nothing grades, and a
#: disposition changes no verdict -- it answers what `--plan-only` reports.
#: An older build still refuses the file (unknown top-level key), which is the
#: safe direction.
_DISPOSITION_KEYS = {'rules', 'withheld', 'refs', 'contradictions'}
_BLOCK_KEYS = {'name', 'group', 'refs', 'zone', 'side', 'exclusive',
               'tolerance_mm', 'context',
               # #1357: the free text, under any of its three spellings.
               *_FREE_TEXT_KEYS,
               # #893. `rotation` is a DECISION (this angle);
               # `rotation_candidates` is a SET a search may choose from.
               # Declaring both on one block is refused -- see `_rotation`.
               'rotation', 'rotation_candidates'}
#: Block keys a retired reader honoured, refused by name with what replaced
#: them rather than as unknown: an author who wrote one meant it.
_RETIRED_BLOCK_KEYS = {
    'rigid': ("`rigid` (#1052) was read only by the retired quench engine, "
              "and nothing holds a block together now. place_simple moves "
              "each group of the groups file (check_groups.py) together; "
              "remove the key"),
}
#: #1051. One declared row of identical parts. `members` is ORDERED and
#: literal (no globs): `order: "declared"` reads it as the order, and a glob
#: has none. `serves` is the part the row serves (whose pin order
#: `order: "pin"` follows); `"unknown"` for either enum is first-class and
#: distinct from an absent key.
_ARRAY_KEYS = {'name', 'members', 'serves', 'order', 'rotation', 'pitch_mm',
               'axis', 'allow_mixed', 'source', 'context'} | _FREE_TEXT_KEYS
#: #1054. An exact pose a part is to be seated at. `basis` says whose fact it
#: is: `declared` (the design brief), `mechanical` (mechanical.json), or
#: `derived_default` -- nobody's: a pose a planner chose because nothing
#: declared one (mounting holes at the board corners, say; no tool in this
#: tree writes one now). `rot`/`side` may be `"unknown"`; absent means
#: nobody said.
#: `accept_courtyard_overlap` (#1060): the refs whose COURTYARD this pose may
#: overlap -- a named waiver, carrying its `why`, of courtyard overlap only.
_FIXED_POSE_KEYS = {'ref', 'x', 'y', 'rot', 'side', 'basis', 'context',
                    'accept_courtyard_overlap'} | _FREE_TEXT_KEYS
_FIXED_POSE_BASES = ('declared', 'mechanical', 'derived_default')
#: #837. The board-level assembly policy: which faces the fab will populate.
#: `blocks[].side` is a claim about ONE subsystem; this is a claim about the
#: whole board, and it is the thing `options.grow_board` needed and could not
#: be told -- it credits the back face's area to every board, so a placement
#: can be reported as fitting on area the fab will never populate.
#: #1255: `body_edge_mm` is the least distance (mm) a component body may sit
#: from the board edge -- an assembler's rule the board file cannot carry.
#: Declared, it arms check_assembly's `body_edge_band` gate; undeclared, that
#: census reports against JLCPCB's 2.5 mm, labelled `derived_default`, and
#: never gates. Spelled `body_edge_mm`, not "...band", so it cannot be read as
#: `edge_connectors[].along_edge_band`, which is a position ALONG an edge.
_ASSEMBLY_KEYS = {'sides', 'context', 'body_edge_mm'} | _FREE_TEXT_KEYS
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
_KEEPOUT_KEYS = {'name', 'rect', 'circle', 'sides', 'allow',
                 'context'} | _FREE_TEXT_KEYS
#: `source`, `suspect`, `suspect_reason`, `overhang_capped` and
#: `observed_overhang_mm` are emitter-written and read by nothing today; they
#: are accepted because `emit_intent` writes them and the round trip must
#: survive, not because anything acts on them.
#: `center_on_edge` / `along_edge_band` are #712: the along-edge half of an
#: edge claim. Absent by default and NEVER written by `emit_intent`, so every
#: intent that existed before them grades identically.
_EDGE_CONNECTOR_KEYS = {'ref', 'edge', 'overhang_mm', 'max_setback_mm',
                        'class', 'source', 'suspect', 'suspect_reason',
                        'overhang_capped', 'observed_overhang_mm', 'context',
                        'center_on_edge', 'along_edge_band', 'side',
                        'mating_face'} | _FREE_TEXT_KEYS
#: `edge_connectors[].mating_face`: the side of the footprint, in its OWN
#: unrotated frame, its mating face opens on -- declared for a part whose body
#: names none (`mating.mating_face_local` abstains: a side-entry JST drawn
#: symmetric about its pads). Never emitted.
MATING_FACES = ('+x', '-x', '+y', '-y')
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
               'max_pin_distance_mm', 'pin_functions', 'same_side',
               'within_radius_refs', 'within_radius_mm'}
#: #902. One declared claim: these two named parts, no further apart than
#: `max_mm`. `ref` is always a SINGLE ref here -- the brief's list form is
#: sugar that `compile_brief` expands, so the intent carries one row per claim
#: and `Violation.ref` always has a ref to name.
#:
#: The free text is `why` (or its alias `note` / `reason`, #1357). The
#: brief's own `requirement` and `why` still compile into `context`, as for
#: `interfaces[]`; an intent author writes the reason on the row.
_PROXIMITY_KEYS = {'ref', 'near', 'max_mm', 'basis', 'pads', 'source',
                   'context'} | _FREE_TEXT_KEYS
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
#: The `context.source` of the crystal claims `crystals.crystal_claims`
#: derives and `emit_intent` writes. Their limits are the human corpus's
#: 75th percentile, which a quarter of human boards exceed, so such a row is
#: graded at WARN whatever the intent's `proximity` severity, and the
#: placement gate (`tether_gate_spec`) does not hold it. An author who declares
#: a crystal claim of their own writes it without this source.
DERIVED_CLAIM_SOURCE = 'placement.crystals'


def derived_claim(claim) -> bool:
    """A proximity row the toolchain derived, not one an author declared."""
    return (claim.get('context') or {}).get('source') == DERIVED_CLAIM_SOURCE
#: #1253. One declared keep-away: `rule` is ONE rule in route.py's own
#: syntax (`AGGRESSOR:VICTIM:GAP`, `class=` terms), parsed by the router's
#: `keep_away.parse_keep_away_rules`, so the grade and the router read the
#: same declaration the same way. NOT a `min_mm` on `proximity[]`, which the
#: brief refuses by name: this is a relation between two NET groups at a
#: functional distance, not a copper clearance between two parts.
_KEEP_AWAY_KEYS = {'rule', 'source', 'context'} | _FREE_TEXT_KEYS
#: #1211. One declared channel copy: `from` and `to` name sheet INSTANCES
#: (a sheet name or a path, as `place_pose replicate` takes them); the rest
#: are that verb's own flags, so a declaration and a command line read
#: alike. `tolerance_mm` is the grade's: how far a target part may sit from
#: the source arrangement before it is reported.
_REPLICATE_KEYS = {'from', 'to', 'anchor', 'allow_net_mismatch', 'only',
                   'skip', 'search_mm', 'tolerance_mm', 'source',
                   'context'} | _FREE_TEXT_KEYS
#: The default `replicate[].tolerance_mm`: the writer's coordinates are
#: exact to 1e-6 mm, so a copy the tools made reads 0; this is room for a
#: designer's hand copy snapped to a 0.05 mm grid.
REPLICATE_TOLERANCE_MM = 0.05
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
_WAIVER_KEYS = {'pair', 'context'} | _FREE_TEXT_KEYS
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
# substring SUSPECT by emit_intent.
_EDGES = ('north', 'south', 'east', 'west')
# A groups file spells its edge hints N/S/E/W (grouping.EDGES), and an intent
# written beside one is edited the same way: three of four live PCB-chat runs
# on 2026-10-22 wrote `edge: 'E'` into edge_connectors and were refused. The
# letter is unambiguous, so it is read as the edge it names.
_EDGE_LETTERS = {'N': 'north', 'S': 'south', 'E': 'east', 'W': 'west'}


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
    #: plan's written answers to the questions `check_floorplan --plan-only`
    #: reports. Read by `rule_roster`, never by a rule -- a disposition is not a
    #: verdict. Defaulted, so every existing construction site is untouched.
    dispositions: Dict[str, Dict[str, str]] = field(default_factory=dict)
    #: #959 (#1002). `context.basis`: `{intent path: basis}` -- what each
    #: number IS (`declared`, `observed_baseline`, `derived_default`,
    #: `mechanical`). Carried like `budget_withheld` so a finding can say an
    #: observed baseline is not a requirement. Read for messages only.
    basis: Dict[str, str] = field(default_factory=dict)
    #: #1051. Declared rows of identical parts, raw validated dicts (the
    #: absent-vs-"unknown" difference is kept; `arrays.spec_of` resolves).
    #: NEVER emitted by default: which parts form a row is a design decision.
    arrays: Tuple[Dict[str, object], ...] = ()
    #: #1054. Exact poses to seat, `[{ref, x, y, rot?, side?, basis, why?}]`.
    #: Graded by an anchor `grade` compiles from each entry.
    fixed_poses: Tuple[Dict[str, object], ...] = ()
    #: #1253. Declared keep-aways, `[{rule, source?, note?, context?}]`, one
    #: rule each. NEVER emitted: which nets are noisy and which sensitive is
    #: a design fact the board does not hold. Graded by `keep_away`; the
    #: placer does not read it.
    keep_away: Tuple[Dict[str, object], ...] = ()
    #: #1211. Declared channel copies, `[{from, to[], anchor?, ...}]`. NEVER
    #: emitted: which instance is the template is a design decision. Copied
    #: by `place_pose replicate --intent`; graded by `replicate`, which the
    #: placer does not read.
    replicate: Tuple[Dict[str, object], ...] = ()
    #: Datasheet layout templates, `[{anchor, members[{ref, at, rot}],
    #: tolerance_mm, ...}]` (`placement.templates`). NEVER emitted: where a
    #: chip's datasheet puts its support parts is read from the datasheet,
    #: not the board. Seated and carried by `place_simple`; graded by
    #: `template`.
    templates: Tuple[Dict[str, object], ...] = ()

    def assembly_sides(self) -> str:
        """The declared policy, or 'both' -- which constrains nothing.

        'both' is the resolved default rather than None so every consumer
        (the rule, `options.grow_board`) reads one vocabulary. A board nobody
        declared is a board that may use both faces, which is exactly what the
        arithmetic did before this key existed.
        """
        return str((self.assembly or {}).get('sides') or 'both')

    def body_edge_mm(self) -> Optional[float]:
        """#1255: the declared least body-to-board-edge distance, or None.

        None, never a default: the default (JLCPCB's 2.5 mm) belongs to the
        census that reports it as `derived_default`, and an undeclared band
        must not read as a declared one -- the declaration is what gates."""
        v = (self.assembly or {}).get('body_edge_mm')
        return None if v is None else float(v)

    def edge_claim_refs(self) -> Tuple[str, ...]:
        """Refs whose `edge_claims()` entry names an edge (#1255): parts the
        intent says belong AT the edge, which the body-to-edge census lists
        apart rather than counting. An entry whose edge was declared
        "unknown" carries no `edge` and claims nothing."""
        return tuple(str(c['ref']) for c in self.edge_claims()
                     if c.get('edge'))

    def edge_claims(self) -> Tuple[Dict[str, object], ...]:
        """The `edge_connectors` entries that actually CLAIM AN EDGE.

        The wire key holds two populations. An `edge_receptacle` /
        `edge_actuator` entry says "this part's mating face belongs at the
        boundary", and its readers act on that: the grade's edge rules read
        its edge, and reconstruct.classify forces it into the anchor tier. A
        `connector_affinity` entry (run-23) says only "this is a
        connector-family part with NO edge claim" -- it exists so a mid-board
        header stops being invisible to `rule_edge_connector`, which flags an
        interior pose at WARN.

        Handing the second population to the first's consumers would
        silently claim an edge for parts nobody said anything about (6 extra
        refs on tigard_placed). So those consumers read THIS, and the rule
        reads `edge_connectors`.

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

    def courtyard_waiver_pairs(self) -> Tuple[Tuple[str, str], ...]:
        """The pairs whose COURTYARD overlap a declared pose may carry
        (#1060): `overlap_waivers[]` plus each `fixed_poses[].
        accept_courtyard_overlap` ref paired with its entry's ref. Read by
        the plan check of fixed poses only. Deliberately NOT folded into
        `waiver_pairs`, whose
        consumers (`grade_body_overlap`) exempt a pair from the drawn-BODY
        containment gate too -- a courtyard waiver must not license one part
        sitting wholly inside another's body (phase-4 verifier)."""
        out = list(self.waiver_pairs())
        for f in self.fixed_poses:
            for other in f.get('accept_courtyard_overlap') or ():
                out.append((str(f['ref']), str(other)))
        return tuple(out)

    def fixed_pose_courtyard_waivers(self) -> Tuple[Tuple[str, str], ...]:
        """#1298: the `fixed_poses[].accept_courtyard_overlap` half of
        `courtyard_waiver_pairs` -- each accepted ref with its entry's ref --
        which check_assembly and the renderer hand to `grade_body_overlap`'s
        courtyard-only channel (`courtyard_waivers`), so a declared pair is
        waived there and a containment still gates."""
        return tuple((str(f['ref']), str(other)) for f in self.fixed_poses
                     for other in f.get('accept_courtyard_overlap') or ())

    def severity_of(self, rule: str, default: str = ERROR) -> str:
        return self.severity.get(rule, default)

    def zone_tolerance(self, zone: Zone) -> float:
        if zone.tolerance_mm is not None:
            return float(zone.tolerance_mm)
        return float(self.defaults.get('zone_tolerance_mm',
                                       DEFAULT_ZONE_TOLERANCE_MM))


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
    if not math.isfinite(float(raw)):
        # NaN fails every comparison and inf % 360 is NaN, so either would
        # load as an angle no check can ever find wrong (Phase-1 verifier).
        raise IntentError(
            f"{where}: rotation {raw!r} is not a finite angle")
    return float(raw) % 360.0


def _rotation_candidates(raw, where: str) -> Tuple[float, ...]:
    """The declared candidate set, order PRESERVED.

    Order is load-bearing, not cosmetic: a search that keeps the FIRST pose
    that fits changes which rotation wins a tie when the list is reordered.
    That is why this returns a tuple in the author's order and never a set.

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
    # #1357: a row that takes free text takes it as a string, under any of
    # its spellings -- checked here so no row type can forget.
    if _FREE_TEXT_KEYS <= set(allowed):
        _free_text_strings(obj, where)


def _entry_context(entry, where: str) -> None:
    """An entry's `context` is free-form, but it is still an OBJECT.

    Type-checked and otherwise untouched: a list here means the author meant
    something else, while an unknown key inside means nothing at all. On an
    `edge_connectors[]` entry a few keys ARE read: `context.mount_mode`
    (`edge_mount` makes the seat read the drawn body, #961; a vertical mount
    exempts the part from the receptacle seat, #959 -- the plan choosing its
    own clause, as dropping `class` would be, and with a brief a disagreement
    about a vertical mount drifts), and `context.compiled_from`, by drift
    attribution.
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
    one caller -- `grade()` -- while `place_simple`, `place_pose` and
    `place_fanout_clearance` load an intent and never call it. A refusal that
    lived there would leave those holding a row whose `pads` is a number.

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


def _keep_away_claims(raw: Dict) -> List[Dict]:
    """Validate `keep_away[]` (#1253), AT LOAD, for `_proximity_claims`'s
    reason: the placement tools load an intent and never call
    `validate_intent`.

    Each `rule` is parsed by the ROUTER's own `parse_keep_away_rules`, so a
    rule this loader accepts is one route.py prices, and a malformed one is
    refused with the router's message. One rule per row, and no rule twice:
    a row is one claim, reported under its own index."""
    got = raw.get('keep_away')
    if got is None:
        return []
    if not isinstance(got, list):
        raise IntentError(f"keep_away: expected a list of rows, got "
                          f"{type(got).__name__}")
    from keep_away import parse_keep_away_rules
    out: List[Dict] = []
    seen: Dict[str, int] = {}
    for i, k in enumerate(got):
        where = f"keep_away[{i}]"
        if not isinstance(k, dict):
            raise IntentError(f"{where}: expected an object with a `rule` "
                              f"('AGGRESSOR:VICTIM:GAP')")
        _reject_unknown(k, _KEEP_AWAY_KEYS, where)
        _entry_context(k, where)
        text = k.get('rule')
        if not isinstance(text, str) or not text.strip():
            raise IntentError(f"{where}: `rule` must be a rule "
                              f"'AGGRESSOR:VICTIM:GAP', got {text!r}")
        try:
            rules = parse_keep_away_rules(text)
        except ValueError as exc:
            raise IntentError(f"{where}: {exc}") from None
        if len(rules) != 1:
            raise IntentError(f"{where}: {text!r} holds {len(rules)} rules; "
                              f"one rule per row")
        spec = rules[0].spec
        if spec in seen:
            raise IntentError(f"{where}: {spec!r} repeats keep_away"
                              f"[{seen[spec]}]")
        seen[spec] = i
        out.append(dict(k))
    return out


def _replicate_claims(raw: Dict) -> List[Dict]:
    """Validate `replicate[]` (#1211), AT LOAD: `place_pose replicate` acts
    on it and never calls `validate_intent`.

    What needs no board is checked here -- the shapes, `from` not among its
    own `to`, a target named twice (in one row or across rows: two copies
    onto one instance would each undo the other), `only` with `skip`, a
    waiver without a pad. Whether the sheets EXIST, and are wired alike, is
    the board's question: the grade answers it per row
    (`replicate_unresolved`), with `place_pose replicate`'s own reason."""
    got = raw.get('replicate')
    if got is None:
        return []
    if not isinstance(got, list):
        raise IntentError(f"replicate: expected a list of rows, got "
                          f"{type(got).__name__}")
    from .replicate import norm_name

    def _strs(v, where, *, empty_ok=False):
        if (not isinstance(v, list) or (not v and not empty_ok)
                or not all(isinstance(x, str) and x.strip() for x in v)):
            raise IntentError(f"{where}: expected a list of non-empty "
                              f"strings, got {v!r}")
        return [x.strip() for x in v]

    out: List[Dict] = []
    targets: Dict[str, str] = {}
    for i, r in enumerate(got):
        where = f"replicate[{i}]"
        if not isinstance(r, dict):
            raise IntentError(f"{where}: expected an object with `from` and "
                              f"`to` (sheet names or paths)")
        _reject_unknown(r, _REPLICATE_KEYS, where)
        _entry_context(r, where)
        src = r.get('from')
        if not isinstance(src, str) or not norm_name(src):
            raise IntentError(f"{where}.from: expected a sheet name or path, "
                              f"got {src!r}")
        to = _strs(r.get('to'), f"{where}.to")
        for t in to:
            key = norm_name(t).casefold()
            if key == norm_name(src).casefold():
                raise IntentError(f"{where}.to: {t!r} is the `from` sheet "
                                  f"itself")
            if key in targets:
                raise IntentError(
                    f"{where}.to: {t!r} is already a target of "
                    f"{targets[key]}; two copies onto one instance would "
                    f"each undo the other")
            targets[key] = where
        if 'anchor' in r and (not isinstance(r['anchor'], str)
                              or not r['anchor'].strip()):
            raise IntentError(f"{where}.anchor: expected a reference, got "
                              f"{r['anchor']!r}")
        for tok in _strs(r.get('allow_net_mismatch', []),
                         f"{where}.allow_net_mismatch", empty_ok=True):
            ref, dot, pad = tok.partition('.')
            if not (ref and dot and pad):
                raise IntentError(f"{where}.allow_net_mismatch: {tok!r} is "
                                  f"not REF.PAD (one pad per entry)")
        for k in ('only', 'skip'):
            if k in r:
                _strs(r[k], f"{where}.{k}")
        if 'only' in r and 'skip' in r:
            raise IntentError(f"{where}: `only` and `skip` together have two "
                              f"readings; declare one")
        for k, low in (('search_mm', False), ('tolerance_mm', True)):
            if k in r:
                v = r[k]
                if (isinstance(v, bool) or not isinstance(v, (int, float))
                        or not math.isfinite(v) or v < 0
                        or (v == 0 and not low)):
                    raise IntentError(f"{where}.{k}: expected a "
                                      f"{'non-negative' if low else 'positive'}"
                                      f" number of mm, got {v!r}")
        out.append(dict(r, to=to))
    return out


def _template_claims(raw: Dict) -> List[Dict]:
    """Validate `templates[]` AT LOAD (`placement.templates.validate`):
    `place_simple` acts on it and never calls `validate_intent`."""
    from .templates import TemplateError, validate
    try:
        return validate(raw.get('templates'))
    except TemplateError as ex:
        raise IntentError(str(ex)) from ex


def _along_edge_claim(c: Dict, i: int) -> None:
    """Validate `center_on_edge` / `along_edge_band` on one entry (#712).

    WHERE the part sits along its edge. The rule already grades the overhang
    band, the nearest-edge identity and a setback, so a seat well off the
    centreline of its edge is satisfied exactly as well as a centred one.

    Refused HERE, at load, rather than in `validate_intent`, and that is a
    deliberate deviation from the issue's own text. `validate_intent` has
    exactly one caller -- `grade()` -- while the placement tools load an
    intent and never call it. A refusal that lived there would leave a reader
    of these fields holding an entry that declares two conflicting bands with
    no policy for which wins. The
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


def _must_lock_hit(ref: str, must_lock) -> Optional[str]:
    """The `must_lock` pattern that names `ref`, or None."""
    for pat in must_lock or ():
        if fnmatch.fnmatchcase(ref, pat) or pat == ref:
            return pat
    return None


def _fixed_pose_entries(raw: Dict, conns, must_lock) -> List[Dict]:
    """Validate `fixed_poses[]` (#1054), AT LOAD -- `place_simple` acts on it
    and never calls `validate_intent`, the reason `_proximity_claims` gives.

    A fixed pose collides with two other claims on the same part, and each is
    refused BY NAME rather than given a precedence rule: an `edge_connectors`
    entry (a claim on an edge band, not on one pose) and a `must_lock`
    pattern (a claim about the FILE; a part the plan says to seat is not one
    the file already pins).
    """
    got = raw.get('fixed_poses')
    if got is not None and not isinstance(got, list):
        raise IntentError(f"fixed_poses: expected a list, got "
                          f"{type(got).__name__}")
    edge_refs = {str(c.get('ref')) for c in conns}
    out: List[Dict] = []
    seen: Dict[str, int] = {}
    for i, f in enumerate(got or []):
        where = f"fixed_poses[{i}]"
        if not isinstance(f, dict):
            raise IntentError(f"{where}: expected an object with `ref`, `x`, "
                              f"`y` and `basis`")
        _reject_unknown(f, _FIXED_POSE_KEYS, where)
        _entry_context(f, where)
        ref = f.get('ref')
        if not ref or not isinstance(ref, str):
            raise IntentError(f"{where}: `ref` must be a single reference")
        where = f"{where} ({ref})"
        if ref in seen:
            raise IntentError(f"{where}: duplicate fixed pose, already "
                              f"declared at fixed_poses[{seen[ref]}] -- two "
                              f"poses for one part, with no rule for which "
                              f"wins")
        seen[ref] = i
        for k in ('x', 'y'):
            if k not in f:
                raise IntentError(f"{where}: needs `{k}`")
            v = _number(f[k], f"{where}.{k}")
            if not math.isfinite(v):
                raise IntentError(f"{where}.{k}: {f[k]!r} is not a finite "
                                  f"coordinate")
        rot = f.get('rot')
        if rot is not None and rot != 'unknown':
            _rotation(rot, f"{where}.rot")
        side = f.get('side')
        if side is not None and side not in ('F', 'B', 'unknown'):
            raise IntentError(f"{where}.side: {side!r}, expected 'F', 'B' or "
                              f"'unknown'")
        basis = f.get('basis')
        if basis not in _FIXED_POSE_BASES:
            raise IntentError(
                f"{where}.basis: {basis!r}, expected one of "
                f"{', '.join(map(repr, _FIXED_POSE_BASES))} -- whose fact the "
                f"pose is decides what a contradiction means")
        # #1060: a NAMED courtyard waiver. Literal refs (a glob would waive
        # parts nobody looked at), never the entry's own ref, no repeats, and
        # a `why` -- a waiver is a claim the author must be able to defend.
        # Whether each ref is ON the board is a board question, answered by
        # the plan check and the grade (`fixed_pose_unresolved`).
        acc = f.get('accept_courtyard_overlap')
        if acc is not None:
            if not isinstance(acc, list) or not all(
                    isinstance(r, str) and r for r in acc):
                raise IntentError(
                    f"{where}.accept_courtyard_overlap: expected a list of "
                    f"references")
            bad = [r for r in acc if any(c in r for c in '*?[')]
            if bad:
                raise IntentError(
                    f"{where}.accept_courtyard_overlap: {bad} -- literal "
                    f"references only; a pattern would waive parts nobody "
                    f"looked at")
            if ref in acc:
                raise IntentError(
                    f"{where}.accept_courtyard_overlap names {ref} itself")
            if len(set(acc)) != len(acc):
                raise IntentError(
                    f"{where}.accept_courtyard_overlap repeats a reference")
            if acc and not free_text(f).strip():
                raise IntentError(
                    f"{where}: accept_courtyard_overlap needs a `why` (or its "
                    f"alias `note` / `reason`) -- it accepts a finding "
                    f"KiCad's DRC reports, so it must say why the overlap "
                    f"is the design")
        if ref in edge_refs:
            raise IntentError(
                f"{where}: {ref} is also an `edge_connectors` entry. An edge "
                f"connector entry claims its edge band, and a fixed pose "
                f"claims one exact pose -- two placements for one part. Keep "
                f"one")
        pat = _must_lock_hit(ref, must_lock)
        if pat is not None:
            raise IntentError(
                f"{where}: {ref} is also named by must_lock {pat!r}. must_lock "
                f"is a claim that the FILE already pins the part; a fixed "
                f"pose says the placer is to put it there. Keep one")
        out.append(dict(f))
    return out


def _array_entries(raw: Dict, conns, must_lock, fixed_refs) -> List[Dict]:
    """Validate `arrays[]` (#1051), AT LOAD, the board-free half.

    The board-aware half -- members and `serves` on the board, a FILE-locked
    member, mixed footprints, a member's block rotation or zone -- is
    `array_problems`, which `grade` and `resolve_intent_gate` both call, the
    way both report `block_unresolved`.

    Every refusal names the member it is about: an array is a list, and "an
    array is wrong" sends the reader hunting through it.
    """
    from . import arrays as arr
    got = raw.get('arrays')
    if got is not None and not isinstance(got, list):
        raise IntentError(f"arrays: expected a list, got "
                          f"{type(got).__name__}")
    edge_refs = {str(c.get('ref')) for c in conns}
    owner: Dict[str, str] = {}
    names = set()
    out: List[Dict] = []
    for i, a in enumerate(got or []):
        where = f"arrays[{i}]"
        if not isinstance(a, dict):
            raise IntentError(f"{where}: expected an object with `name` and "
                              f"`members`")
        _reject_unknown(a, _ARRAY_KEYS, where)
        _entry_context(a, where)
        name = a.get('name')
        if not name or not isinstance(name, str):
            raise IntentError(f"{where}: needs a `name`")
        where = f"{where} ({name})"
        if name in names:
            raise IntentError(f"{where}: duplicate array name {name!r}")
        names.add(name)
        members = a.get('members')
        if (not isinstance(members, list)
                or not all(isinstance(m, str) and m for m in members)):
            raise IntentError(f"{where}.members: expected a list of "
                              f"references, got {members!r}")
        if len(members) < 2:
            raise IntentError(
                f"{where}.members: {members!r} -- a row needs at least two "
                f"members; one part is not an array")
        dup = sorted({m for m in members if members.count(m) > 1})
        if dup:
            raise IntentError(f"{where}.members: {', '.join(dup)} listed "
                              f"twice")
        for m in members:
            if m in owner:
                raise IntentError(
                    f"{where}: member {m} is also in array {owner[m]!r}. A "
                    f"part is in one row; the two would pull it apart")
            owner[m] = name
            if m in fixed_refs:
                raise IntentError(
                    f"{where}: member {m} has a `fixed_poses` entry. A fixed "
                    f"part cannot move with its row; take it out of one")
            pat = _must_lock_hit(m, must_lock)
            if pat is not None:
                raise IntentError(
                    f"{where}: member {m} is named by must_lock {pat!r}. A "
                    f"locked part cannot move with its row; take it out of "
                    f"one")
            if m in edge_refs:
                raise IntentError(
                    f"{where}: member {m} is also an `edge_connectors` entry, "
                    f"which claims its own seat at its edge; it cannot move "
                    f"with its row. Take it out of one")
        serves = a.get('serves')
        if serves is not None and (not isinstance(serves, str) or not serves):
            raise IntentError(f"{where}.serves: expected a reference or "
                              f"'unknown', got {serves!r}")
        if serves in members:
            raise IntentError(f"{where}.serves: {serves} is a member of the "
                              f"row it serves")
        order = a.get('order')
        if order is not None and order not in arr.ORDERS:
            raise IntentError(f"{where}.order: {order!r}, expected one of "
                              f"{', '.join(map(repr, arr.ORDERS))}")
        if order == 'pin' and (serves is None or serves == 'unknown'):
            raise IntentError(
                f"{where}: order 'pin' follows the pin numbering of the part "
                f"the row SERVES, and `serves` is "
                f"{'absent' if serves is None else 'unknown'}. Name it, or "
                f"declare the order 'declared' or 'unknown'")
        rot = a.get('rotation')
        if rot is not None and rot not in arr.ROTATION_WORDS:
            _rotation(rot, f"{where}.rotation")
        pitch = a.get('pitch_mm')
        if pitch is not None and pitch not in arr.PITCH_WORDS:
            v = _number(pitch, f"{where}.pitch_mm")
            if not math.isfinite(v) or v <= 0:
                raise IntentError(f"{where}.pitch_mm: {pitch!r}, expected "
                                  f"'auto' or a positive distance")
        axis = a.get('axis')
        if axis is not None and axis not in arr.AXES:
            raise IntentError(f"{where}.axis: {axis!r}, expected one of "
                              f"{', '.join(map(repr, arr.AXES))}")
        if 'allow_mixed' in a and not isinstance(a['allow_mixed'], bool):
            raise IntentError(f"{where}.allow_mixed: expected true or false")
        if a.get('source') is not None and not isinstance(a['source'], str):
            raise IntentError(f"{where}.source: expected a string")
        out.append(dict(a))
    return out


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


def _checked_basis(context: Dict, decaps: Dict) -> Dict[str, str]:
    """`context.basis`, with the decap label dropped once the number no
    longer IS the observation. The census repeats the emitted limit
    (`emitted_max_distance_mm`) precisely so a hand edit is detectable; a
    limit edited away from it is somebody's choice, and a message calling it
    "an observed regression baseline read off a board" would be false
    (Phase-6 verifier)."""
    out = {str(k): str(v) for k, v in
           _obj(context.get('basis'), 'context.basis').items()}
    key = 'decaps.max_distance_mm'
    if out.get(key) == 'observed_baseline':
        emitted = (context.get('decap_census') or {}).get(
            'emitted_max_distance_mm') if isinstance(
                context.get('decap_census'), dict) else None
        if emitted is None or (decaps or {}).get('max_distance_mm') != emitted:
            out.pop(key)
    return out


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
        name = b.get('name') or f"block{i}"
        for key, why in _RETIRED_BLOCK_KEYS.items():
            if key in b:
                raise IntentError(f"blocks[{i}] ({name}): {why}")
        _reject_unknown(b, _BLOCK_KEYS, f"blocks[{i}]")
        _entry_context(b, f"blocks[{i}]")
        if str(name).startswith(MECHANICAL_ANCHOR_PREFIX):
            raise IntentError(
                f"blocks[{i}]: the name {name!r} is reserved -- "
                f"`{MECHANICAL_ANCHOR_PREFIX}` blocks are compiled from "
                f"mechanical.json by the grade itself (#959). Remove it; the "
                f"grade reads the declaration directly")
        if str(name).startswith(FIXED_POSE_ANCHOR_PREFIX):
            raise IntentError(
                f"blocks[{i}]: the name {name!r} is reserved -- "
                f"`{FIXED_POSE_ANCHOR_PREFIX}` blocks are compiled from "
                f"`fixed_poses[]` by the grade itself (#1054). Declare the "
                f"pose there instead")
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
                f"rotation_candidates. `rotation` is a decision (this "
                f"angle); `rotation_candidates` is a set to choose from. "
                f"Declare one")
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
            note=free_text(b),
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
        if isinstance(edge, str) and edge in _EDGE_LETTERS:
            edge = c['edge'] = _EDGE_LETTERS[edge]
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
        if c.get('mating_face') is not None \
                and c['mating_face'] not in MATING_FACES:
            raise IntentError(
                f"edge_connectors[{i}] ({c['ref']}): mating_face "
                f"{c['mating_face']!r}, expected one of "
                f"{', '.join(MATING_FACES)} (the side, in the footprint's own "
                f"frame, its mating face opens on)")
        conns.append(c)

    proximity = _proximity_claims(raw)

    must_lock = _str_tuple(raw.get('must_lock'), 'must_lock')
    fixed_poses = _fixed_pose_entries(raw, conns, must_lock)
    arrays = _array_entries(raw, conns, must_lock,
                            {str(f['ref']) for f in fixed_poses})

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
    if severity.get('mechanical_drift') == WARN:
        # #959: the pose is a recorded fact, so a turn (or a pad-less ref's
        # drift) stays an ERROR whatever the plan says, and a move is a WARN
        # already. `warn` would change nothing while reading like a
        # demotion -- refused, like the impossible direction above.
        raise IntentError(
            "severity: mechanical_drift cannot be demoted -- the declared "
            "pose is a recorded fact a plan cannot overrule. Only `error` "
            "is accepted, and it promotes the move-only WARN")

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
    elif assembly and 'body_edge_mm' not in assembly:
        raise IntentError(
            "assembly: declares neither `sides` nor `body_edge_mm`, so it "
            f"constrains nothing. Give it `sides` (one of "
            f"{list(_ASSEMBLY_SIDES)}) or `body_edge_mm`, or drop the key -- "
            "an assembly block that grades nothing is the failure "
            "`block_unresolved` exists to prevent, one level down")
    if 'body_edge_mm' in assembly:
        # #1255. Greater than zero: a 0 mm band measures nothing, and a
        # negative one reads as "past the edge is fine". Graded by
        # check_assembly's `body_edge_band` (and render's checklist), which is
        # the grader this declaration arms -- not by a floorplan rule.
        if _number(assembly['body_edge_mm'], 'assembly.body_edge_mm') <= 0:
            raise IntentError(
                f"assembly.body_edge_mm: expected a distance > 0 mm, got "
                f"{assembly['body_edge_mm']!r}")

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
    if 'within_radius_refs' in decaps or 'within_radius_mm' in decaps:
        # #1142: the caps a --decaps-from reference keeps within the radius,
        # each held to it per cap. Refused rather than half-read: a list with
        # no limit arms nothing (`rule_decap_ungraded` returns without a
        # `max_distance_mm`), and a list read at one radius cannot be applied
        # at another -- a hand-shrunk `search_radius_mm` would hold a cap the
        # reference keeps at 4 mm to a 3 mm horizon, and the reference would
        # then fail its own intent.
        decaps = dict(decaps)
        if 'within_radius_refs' not in decaps \
                or 'within_radius_mm' not in decaps:
            raise IntentError(
                "decaps.within_radius_refs and decaps.within_radius_mm come "
                "together: the list is the caps the reference keeps within "
                "THAT radius. Re-emit the intent with --decaps-from")
        if decaps['within_radius_refs'] is None:
            # `_str_tuple(None)` is `()`, which would disarm the per-cap
            # rule without a word (Phase-2 verifier).
            raise IntentError(
                "decaps.within_radius_refs is null: give the list (empty if "
                "the reference holds no cap), or drop both keys")
        decaps['within_radius_refs'] = sorted(_str_tuple(
            decaps['within_radius_refs'], 'decaps.within_radius_refs'))
        r = decaps['within_radius_mm']
        if isinstance(r, bool) or not isinstance(r, (int, float)) or r <= 0:
            raise IntentError(
                f"decaps.within_radius_mm must be a positive number, got "
                f"{r!r}")
        if 'max_distance_mm' not in decaps:
            raise IntentError(
                "decaps.within_radius_refs without decaps.max_distance_mm: "
                "the held caps are graded by decap_ungraded, which only "
                "runs under a tether limit, so the list would grade nothing")
        search = decaps.get('search_radius_mm', groups_mod.DECAP_RADIUS_MM)
        if abs(float(search) - float(r)) > 1e-9:
            raise IntentError(
                f"decaps.within_radius_refs was read at "
                f"{float(r):g} mm but decaps.search_radius_mm is "
                f"{float(search):g} mm: a cap held within one radius cannot "
                f"be graded at another. The emitter reads the list at "
                f"{groups_mod.DECAP_RADIUS_MM:g} mm, so either drop "
                f"search_radius_mm or drop both within_radius_* keys")

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
        must_lock=must_lock,
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
        basis=_checked_basis(context, decaps),
        arrays=tuple(arrays),
        fixed_poses=tuple(fixed_poses),
        keep_away=tuple(_keep_away_claims(raw)),
        replicate=tuple(_replicate_claims(raw)),
        templates=tuple(_template_claims(raw)),
    )
    # A disposition for a rule the intent ARMS says "this is not graded" about
    # a rule that is -- the two statements cannot both be true, and a reader
    # trusting the disposition would skip a live finding. Checked here, where
    # only the intent is needed. Refs need the board: `stale_dispositions`
    # reports them when handed it (`grade` and `check_floorplan --plan-only`
    # do). Contradiction ids need the brief and mechanical.json:
    # `stale_dispositions` names them when handed the reconciliation (both of
    # those callers do).
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
    with no reason is a flag that makes the report go away rather than an
    answer to it.
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
        # `fp.pads` on purpose (#1143): a pad-less ref is never anchored,
        # and an aperture-only part is placed like any other
        # (reconcile.anchor_blocks reads the same predicate).
        default = ERROR if (turned or not fp.pads) else WARN
        # The plan may PROMOTE the move-only WARN, never demote the ERROR:
        # the pose is a recorded fact, and a plan's severity map overruling
        # it at grade is exactly what the anchor's fixed ERROR forbids (a
        # 90-degree turn must not read `warn` under
        # `severity: {mechanical_drift: warn}`).
        sev = (ERROR if default == ERROR
               else intent.severity_of('mechanical_drift', default))
        out.append(Violation(
            rule='mechanical_drift',
            severity=sev,
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
#: #1054. The same, for anchors the grade compiles from `fixed_poses[]`.
FIXED_POSE_ANCHOR_PREFIX = 'fixed:'


def mechanical_anchor_violations(pcb_data, pcb_file: str, mechanical: Dict,
                                 *, skip: Sequence[str] = (), state=None,
                                 locked=(), outline=None,
                                 prefix: str = MECHANICAL_ANCHOR_PREFIX,
                                 basis: str = 'mechanical'
                                 ) -> List['Violation']:
    """`zone_containment` for every anchored mechanical ref, against an
    anchor compiled from the FILE at grade time (#959, #1001).

    The anchor is the grader's own rect at the declared pose
    (`reconcile.anchor_blocks`); `skip` holds refs whose mechanical value lost
    a contradiction. ERROR, fixed: the pose is a recorded fact, and a plan's
    `severity` map cannot demote it."""
    from . import reconcile as _rc
    anchors, _skipped = _rc.anchor_blocks(pcb_data, pcb_file, mechanical,
                                          lost=skip, state=state,
                                          prefix=prefix, basis=basis)
    if not anchors:
        return []
    it = intent_from_dict({'schema': 1, 'kind': 'floorplan-intent',
                           'units': 'mm', 'severity': {
                               'zone_containment': 'error'}}, '')
    it = dataclasses.replace(it, blocks=[_anchor_zone(b) for b in anchors])
    blocks = {z.name: [z.name[len(prefix):]] for z in it.blocks}
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


def fixed_pose_violations(intent: Intent, pcb_data, pcb_file: str, *,
                          mechanical=None, mechanical_skip=(), state=None,
                          locked=(), outline=None) -> List['Violation']:
    """Grade every `fixed_poses[]` entry, whatever its source (#1054).

    Through `mechanical_anchor_violations` -- the anchor `reconcile.
    anchor_blocks` builds, named `fixed:<ref>` -- so a pose the brief
    declared is graded by exactly the geometry a mechanical.json pose is,
    and never by a second copy of it.

    An entry at the SAME pose the mechanical file declares for its ref is
    skipped here: the file's own anchor grades that pose, and grading it
    twice would report one moved part as two findings. An entry at a
    DIFFERENT pose is graded too -- skipping it on the ref alone let a brief
    pose 13 mm from the file's go ungraded; the pose row of `reconcile`
    reports the disagreement as a contradiction.
    And an entry carrying the file's pose for a ref whose mechanical value
    LOST a contradiction (`mechanical_skip`) is an ERROR: the placer would
    seat the losing value, and neither anchor grades it.

    A ref the board does not have is an ERROR (`fixed_pose_unresolved`): a
    typo'd pose would otherwise grade clean. A ref the anchor declines for
    another reason (pad-less: it is never anchored) is a WARN with that
    reason, so the pose is never silently ungraded.
    """
    from . import reconcile as _rc
    file_poses = (mechanical or {}).get('poses') or {}
    lost = set(mechanical_skip or ())
    poses: Dict[str, Dict] = {}
    # #1060: the waivers first, and whatever happens below -- an entry whose
    # pose the mechanical file's anchor grades is skipped further down, and
    # its waiver must not be skipped with it (phase-4 verifier).
    out: List[Violation] = list(_fixed_pose_waiver_findings(
        intent, pcb_data, pcb_file, state=state))
    for f in intent.fixed_poses:
        ref = str(f['ref'])
        fp_ = file_poses.get(ref)
        if fp_ is not None and _rc.same_pose(
                _rc.fixed_pose_tuple(f), (fp_['x'], fp_['y'], fp_.get('rot'))):
            if ref not in lost:
                # The file's own anchor grades this very pose.
                continue
            if f.get('basis') == 'mechanical':
                out.append(Violation(
                    rule='fixed_pose_unresolved',
                    severity=intent.severity_of('fixed_pose_unresolved'),
                    ref=ref,
                    message=(f"fixed_poses[{ref}] carries the mechanical.json "
                             f"pose, and that value LOST a contradiction -- "
                             f"the placer would seat the losing value and no "
                             f"anchor grades it. Drop the entry, or answer "
                             f"the contradiction"),
                    measured={'found': True, 'lost_contradiction': True}))
                continue
            # A DECLARED entry that happens to sit at the losing file pose
            # is the plan's own claim: no file anchor grades it (the file
            # value lost), so it is graded here like any other entry.
        rot = f.get('rot')
        poses[ref] = {'x': float(f['x']), 'y': float(f['y']),
                      'rot': (None if rot is None or rot == 'unknown'
                              else float(rot) % 360.0),
                      'reason': free_text(f)
                      or f"a {f.get('basis')} fixed pose"}
    if not poses:
        return out
    for ref in sorted(poses):
        if ref not in (pcb_data.footprints or {}):
            out.append(Violation(
                rule='fixed_pose_unresolved',
                severity=intent.severity_of('fixed_pose_unresolved'),
                ref=ref,
                message=(f"fixed_poses names {ref}, which is not on this "
                         f"board -- a pose for a part that does not exist "
                         f"grades clean, so this is an error"),
                measured={'found': False}))
    src = intent.source_path or 'the intent'
    pseudo = {'poses': poses, 'path': f"{src} fixed_poses[]"}
    _anchors, skipped = _rc.anchor_blocks(pcb_data, pcb_file, pseudo,
                                          state=state,
                                          prefix=FIXED_POSE_ANCHOR_PREFIX,
                                          basis='fixed_pose')
    for ref, why in sorted(skipped.items()):
        if ref not in (pcb_data.footprints or {}):
            continue
        out.append(Violation(
            rule='fixed_pose_unresolved',
            severity=intent.severity_of('fixed_pose_unresolved', WARN),
            ref=ref,
            message=(f"fixed_poses[{ref}] is not graded: {why}"),
            measured={'found': True, 'skipped': why}))
    out.extend(mechanical_anchor_violations(
        pcb_data, pcb_file, pseudo, state=state, locked=locked,
        outline=outline, prefix=FIXED_POSE_ANCHOR_PREFIX, basis='fixed_pose'))
    return out


def _fixed_pose_waiver_findings(intent: Intent, pcb_data, pcb_file: str, *,
                                state=None) -> List['Violation']:
    """#1060: every `fixed_poses[].accept_courtyard_overlap` ref, graded.

    A ref the board does not have is an ERROR (`fixed_pose_unresolved`), as a
    pose for a missing part is: a waiver of nothing reads as a clean grade. A
    waived pair whose courtyards DO overlap on this board is a WARN
    (`fixed_pose_overlap_waived`), with the measured overlap -- the board
    carries exactly the finding KiCad's DRC reports, disclosed rather than
    silently accepted. A waiver that measures no overlap is said too (WARN
    with `overlap_mm2: 0`), since a stale waiver is how a real overlap is
    later waved through."""
    out: List[Violation] = []
    todo = [(str(f['ref']), str(o), free_text(f))
            for f in intent.fixed_poses
            for o in (f.get('accept_courtyard_overlap') or ())]
    if not todo:
        return out
    fps = pcb_data.footprints or {}
    live = []
    for ref, other, why in todo:
        missing = [r for r in (ref, other) if r not in fps]
        if missing:
            if ref in fps:
                out.append(Violation(
                    rule='fixed_pose_unresolved',
                    severity=intent.severity_of('fixed_pose_unresolved'),
                    ref=ref,
                    message=(f"fixed_poses[{ref}].accept_courtyard_overlap "
                             f"names {other}, which is not on this board -- "
                             f"a waiver of a part that does not exist grades "
                             f"clean, so this is an error"),
                    measured={'found': False, 'waiver': other}))
            continue
        live.append((ref, other, why))
    if not live:
        return out
    if state is None:
        import pose_score
        state = pose_score.make_state(pcb_data, pcb_file)
    for ref, other, why in live:
        pa, pb = state.parts.get(ref), state.parts.get(other)
        if pa is None or pb is None:
            continue
        area = legality.pair_overlap_area(
            pa.sides, pa.side, pa.grade_rect(), pa.tht_rect(),
            pb.sides, pb.side, pb.grade_rect(), pb.tht_rect())
        out.append(Violation(
            rule='fixed_pose_overlap_waived',
            severity=intent.severity_of('fixed_pose_overlap_waived', WARN),
            ref=ref,
            message=((f"{ref}'s courtyard overlaps {other}'s by "
                      f"{area:.3f}mm2, waived by its fixed pose ({why})")
                     if area > 1e-6 else
                     (f"{ref}'s fixed pose waives courtyard overlap with "
                      f"{other}, and none is measured -- a stale waiver")),
            measured={'waives': other, 'overlap_mm2': round(area, 4)}))
    return out


def declared_pose_overlaps(fixed_poses, pcb_data, pcb_file: str, *,
                           clearance: float, board_edge_clearance: float,
                           waived=()) -> Tuple[List[Dict], List[Dict]]:
    """#1298: `(courtyard, hard)` -- the pairs of declared `fixed_poses[]`
    that conflict with each other AT their declared poses.

    Measured by the plan check's own predicate (`seeder.
    fixed_pose_pair_conflicts`, stage 0's `_fixed_pose_check`), once as the
    intent stands and once with every conflicting pair waived, so the split
    is exactly what a courtyard waiver changes there: `courtyard` are the
    pairs whose conflict the waiver changes (their courtyards overlap, or a
    pin lies under a courtyard), each with its unwaived `reason`; `hard` are
    the pairs that still conflict waived -- pads or holes, which no waiver
    covers -- each with what is left. A pair can be in both. `waived` are
    the pairs the intent already waives, left out. Records are the
    predicate's own (`pair`, `reason`, `poses`), sorted by pair."""
    from .seeder import fixed_pose_pair_conflicts
    import pose_score
    if len(fixed_poses or ()) < 2:
        return [], []
    state = pose_score.make_state(pcb_data, pcb_file, clearance=clearance,
                                  board_edge_clearance=board_edge_clearance)
    done = frozenset(frozenset(p) for p in waived if len(p) == 2)
    found = {frozenset(c['pair']): c for c in fixed_pose_pair_conflicts(
        state, fixed_poses, waived=done)}
    if not found:
        return [], []
    left = {frozenset(c['pair']): c for c in fixed_pose_pair_conflicts(
        state, fixed_poses, waived=done | set(found))}
    order = sorted(found, key=lambda k: found[k]['pair'])
    return ([found[k] for k in order
             if k not in left or left[k]['reason'] != found[k]['reason']],
            [left[k] for k in order if k in left])


def accept_declared_overlaps(doc: Dict, courtyard_only: Sequence[Dict]
                             ) -> List[Tuple[str, str]]:
    """#1298: write each `declared_pose_overlaps` courtyard pair into the
    emitted intent `doc` as `fixed_poses[].accept_courtyard_overlap` on the
    pair's first entry (the waiver is unordered), with a `note` saying who
    wrote it and why -- explicit, in the file, where the author can delete
    it to refuse the overlap instead. Returns the pairs written."""
    by_ref = {str(f.get('ref')): f for f in doc.get('fixed_poses') or ()}
    out: List[Tuple[str, str]] = []
    for c in courtyard_only:
        a, b = (str(r) for r in c['pair'])
        f = by_ref.get(a)
        if f is None:
            continue
        acc = list(f.get('accept_courtyard_overlap') or ())
        if b in acc:
            continue
        f['accept_courtyard_overlap'] = acc + [b]
        why = (f"check_floorplan --emit-intent (#1298): at their declared "
               f"poses {a} and {b} overlap by courtyard ({c['reason']}); "
               f"both poses are declared, so the overlap is the declared "
               f"design -- delete {b} from accept_courtyard_overlap to refuse "
               f"it instead")
        f['note'] = (f"{f['note']}; {why}" if f.get('note') else why)
        out.append((a, b))
    return out


def array_problems(intent: Intent, pcb_data, blocks: Dict[str, List[str]],
                   *, locked=None) -> List['Violation']:
    """The board-aware half of `arrays[]` validation (#1051).

    Raised by BOTH `grade` and `resolve_intent_gate` -- `block_unresolved`'s
    shape, one raiser and two reach points -- because the gate is what the
    placement CLIs run and it must not seat or hold a row the grade calls
    impossible. Each finding names the member it is about.

    `array_unresolved` (a name the board does not have): a member or the
    served part. `array_conflict` (the row cannot be formed as declared):
    a FILE-locked member (`fp.locked`, and `locked` when the caller read the
    file), mixed footprints without `allow_mixed`, a member whose block
    declares a rotation the row's numeric (or shared) rotation contradicts,
    and members split across zoned blocks -- a row is seated as one piece, so
    it cannot be in two zones, nor half in one.
    """
    from . import arrays as arr
    if not intent.arrays:
        return []
    fps = pcb_data.footprints or {}
    locked = set(locked or ())
    rots = rotations_for_ref(intent, blocks)
    zoned = {z.name for z in intent.blocks if z.rect is not None}
    out: List[Violation] = []

    def _v(a, ref, msg, *, rule, **measured):
        out.append(Violation(
            rule=rule, severity=intent.severity_of(rule), ref=ref,
            block=str(a.get('name')), message=msg, measured=measured))

    for a in intent.arrays:
        spec = arr.spec_of(a)
        name = spec['name']
        serves = spec['serves']
        if serves not in (None, 'unknown') and serves not in fps:
            _v(a, None,
               f"array {name!r} serves {serves}, which is not on this board",
               rule='array_unresolved', serves=serves, found=False)
        present = [m for m in spec['members'] if m in fps]
        for m in spec['members']:
            if m not in fps:
                _v(a, m,
                   f"array {name!r}: member {m} is not on this board",
                   rule='array_unresolved', found=False)
        for m in present:
            if getattr(fps[m], 'locked', False) or m in locked:
                _v(a, m,
                   f"array {name!r}: member {m} is locked in the board file, "
                   f"so it cannot move with its row -- unlock it or take it "
                   f"out of the array", rule='array_conflict', locked=True)
        if not spec['allow_mixed'] and present:
            names = {m: str(fps[m].footprint_name or '') for m in present}
            first = names[present[0]]
            for m in present[1:]:
                if names[m] != first:
                    _v(a, m,
                       f"array {name!r}: member {m} is {names[m]!r} while "
                       f"{present[0]} is {first!r} -- a row of different "
                       f"footprints has no one pitch or rotation; set "
                       f"`allow_mixed` if it is meant",
                       rule='array_conflict', footprint=names[m],
                       first=first)
        want = spec['rotation']
        declared = {m: rots[m] for m in present if m in rots}
        # Rotation equivalence is the GRADER's (`arrays.formation`): a part
        # with <= 2 copper pads is the same turned 180, so its allowed set is
        # widened by `arrays.allowed_angles`, and the two checks agree.
        pads = {m: arr._copper_pad_count(fps[m]) for m in present}
        if arr.is_number(want):
            for m, (r, cands) in sorted(declared.items()):
                ok_set = arr.allowed_angles(
                    [r] if r is not None else cands, pads[m])
                if not any(arr._ang_diff(x, want) < 1e-9 for x in ok_set):
                    _v(a, m,
                       f"array {name!r}: member {m}'s block declares "
                       f"{'rotation ' + format(r, 'g') if r is not None else 'rotation_candidates ' + str(list(cands))}"
                       f", and the array declares rotation {want:g}. A part "
                       f"has one angle; resolve the two claims",
                       rule='array_conflict', block_rotation=r,
                       block_candidates=(None if cands is None
                                         else list(cands)),
                       array_rotation=want)
        elif want == 'shared' and declared:
            # ONE angle must be allowed to every member: intersect each
            # member's allowed set (a decided `rotation` is a one-angle set,
            # `rotation_candidates` its list). Comparing only the decided
            # angles missed R6 {0, 90} against R7 {180} (Phase-1 verifier).
            #
            # Modulo 180 only when EVERY member is symmetric: formation
            # compares a pair at the stricter of its two periods, so one
            # multi-pad member makes the whole row exact.
            sym = all(arr.rotation_period(pads[m]) == 180.0 for m in present)
            allowed = {m: (arr.allowed_angles([r] if r is not None
                                              else cands,
                                              pads[m] if sym else None))
                       for m, (r, cands) in declared.items()}
            common = set.intersection(*allowed.values())
            if not common:
                sets = '; '.join(
                    f"{m} {sorted(format(x, 'g') for x in allowed[m])}"
                    for m in sorted(allowed))
                for m in sorted(allowed):
                    _v(a, m,
                       f"array {name!r} declares ONE shared rotation, and no "
                       f"angle is allowed to every member's block ({sets})",
                       rule='array_conflict',
                       allowed=sorted(allowed[m]))
        in_zone = {m: sorted(b for b in zoned if m in blocks.get(b, ()))
                   for m in present}
        zones_used = sorted({b for bs in in_zone.values() for b in bs})
        if zones_used:
            for m in present:
                if in_zone[m] != zones_used or len(zones_used) > 1:
                    _v(a, m,
                       f"array {name!r}: member {m} is in zoned block(s) "
                       f"{in_zone[m] or 'none'}, while the row's members are "
                       f"in {zones_used}. A row is seated as one piece, so "
                       f"all of it is in one zone or none of it is",
                       rule='array_conflict', zones=in_zone[m],
                       row_zones=zones_used)
    return out


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


def emitted_zone_rect(rect, bounds):
    """A zone as `emit_intent` writes it (#1174): 3 decimals, then clamped
    into the envelope AS WRITTEN (`round(v, 4)` of the board bounds).

    The zone used to be rounded on its own: CM5_MINIMA_3's x-min 75.0504
    became 75.05, 0.4 um outside the 4-decimal envelope, and
    `validate_intent` (tol 0) refused the emitted intent on its own board
    with a message printing the same number on both sides. Clamping after
    rounding can only move a side INWARD, so zone <= envelope holds exactly."""
    z = [round(v, 3) for v in rect]
    if not bounds:
        return z
    env = [round(v, 4) for v in bounds]
    return [max(z[0], env[0]), max(z[1], env[1]),
            min(z[2], env[2]), min(z[3], env[3])]


def _ceil4(v: float) -> float:
    """Round UP to 4 decimals. See the legality_budget note in emit_intent."""
    return math.ceil(v * 1e4 - 1e-9) / 1e4


def _circle_hits_rect(cx, cy, radius, rect) -> bool:
    nx = min(max(cx, rect[0]), rect[2])
    ny = min(max(cy, rect[1]), rect[3])
    return math.hypot(nx - cx, ny - cy) < radius


# --------------------------------------------------------------------------
# keep-outs: ONE resolver and ONE hit test, shared by the grader (rule_keepout,
# below) and the seat predicate (seeder.pose_ok).
#
# They live here, together, because the alternative measured badly elsewhere in
# this module: docs/floorplan-intent.md says of the rules that "every one of
# them measures with the geometry the OPTIMIZER ITSELF gates on", and until
# #701 the keepout row was the one place that was false -- the optimizer gated
# on nothing at all. A seat the search accepts that the grade then flags is an
# exit 4 on a board the placer produced correctly, so the two must not be two
# implementations.
# --------------------------------------------------------------------------

def keepouts_for_ref(keepouts, ref: str, sides) -> Tuple[Dict, ...]:
    """The keep-out entries that BIND `ref`: not exempted by `allow`, and
    sharing at least one face with it.

    Pose-INVARIANT by construction -- an fnmatch against a reference and the
    set of faces a part occupies are both unchanged by moving it -- which is
    what lets a seat search resolve this ONCE per part instead of once per
    candidate pose: a search evaluates thousands of poses per part.

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

    `fnmatchcase`, as every reference glob is (#1208): plain `fnmatch.fnmatch`
    applies `os.path.normcase`, which folds case on Windows only, so one intent
    exempted different parts per host.
    """
    return fnmatch.fnmatchcase(ref, pattern)


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
    to drift, and a pose the seat predicate accepts at the boundary that the
    grade then flags is exactly the round trip this exists to keep closed.

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
            # #1206: a far side of several clusters is its clusters.
            hit = max(hit, legality.far_overlap_area(r, entry['rect']))
        else:
            cx, cy, radius = entry['circle']
            if any(_circle_hits_rect(cx, cy, radius, b)
                   for b in legality.far_boxes(r)):
                hit = max(hit, 1.0)
    return hit if hit > legality.EPS else 0.0


# --- #1098: a PCB-edge plug's mating region, kept clear on BOTH faces --------
# Run 36 placed 8 back-side parts on the tongue of StickHub's USB-A plug --
# the part of the board that slides into a socket -- and every instrument
# passed it. The plug (`USB_A_PCB_traces_small`) is board copper: SMD finger
# pads, `exclude_from_pos_files`, no 3D model, a courtyard on F.CrtYd only
# and no keep-out. Courtyards are per side, so a B-side part never pairs with
# it. The region is derived from the footprint and handed to every consumer
# of the #701 keep-out channel, which already enforces a rect on both faces
# with an `allow` exemption (seeder.pose_ok, the placement state, grade),
# plus the two
# graders that run without an intent (grade_pad_legality -> place_pose, and
# check_assembly).

#: Attrs that say the footprint is not placed by the assembly house.
MATING_ATTRS = ('board_only', 'exclude_from_pos_files')
#: The plug's netted finger pads must reach this close to the outline (mm).
#: StickHub J1's fingers stop 0.6 mm short of the tongue's tip, as a USB-A
#: plug's do; a solder jumper's pads do not reach an edge at all.
MATING_FINGER_EDGE_MM = 1.0
#: A plug carries at least this many netted finger pads: a USB-A PCB plug,
#: the smallest board-copper plug, has four (VBUS, D-, D+, GND), and a card
#: edge has more. KiCad's own Jumper library draws its 2- and 3-pad solder
#: jumpers `exclude_from_pos_files` with no model, most without a net-tie
#: group, and a DNP passive can carry the same attrs; at 2 fingers any of
#: them sitting at an edge read as a plug (#1098 review: tigard's JP1 moved
#: to its edge turned the board NOT BUILDABLE and was locked there).
MATING_MIN_FINGERS = 4
#: The courtyard is inset by this before it becomes the keep-out. StickHub's
#: J2 and J6 courtyards reach 0.15 mm past the tongue's root; the inset keeps
#: a neighbour's courtyard margin that grazes the root from reading as a part
#: on the plug, and still catches all 8 run-36 parts.
MATING_INSET_MM = 0.25
MATING_PREFIX = 'mating:'


#: `--decaps-from`: at least this share of EACH board's pad-bearing parts
#: must be on the other under the same reference AND footprint, or the
#: reference is another design and its limit is withheld (#1099). Both
#: directions, because generic passives match: a 20-part H3/DDR board has 18
#: of its parts on tigard (C1-C12, R1-R6, all 0402), but tigard has 18 of
#: its 89 on it.
DECAPS_FROM_MIN_MATCH = 0.9


def _copper_pads(fp):
    """The pads that carry copper: NPTH holes and aperture-only pads (paste or
    mask windows, #1143) skipped."""
    from paste_apertures import pad_has_copper
    return [p for p in (fp.pads or ()) if pad_has_copper(p)]


def _plug_seat_rect(ref, fp, crt, gate):
    """The board rect of `ref`'s courtyard while the part is SEATED at the
    outline the way a plug is, else None (#1098).

    Seated means: no pad copper past the outline (`pad_copper_overrun_mm`,
    the measure #1096's gate reports -- a plug hanging across an edge is
    misplaced, and a lock would keep it there), >= 2 netted pads within
    `MATING_FINGER_EDGE_MM` of the outline, and a courtyard that is on the
    board and reaches the outline. Geometry only: whether the part IS a plug
    is `derived_mating_keepouts`' question, or the author's when a keep-out
    names it."""
    from .parser import courtyard_for_side
    from .part_class import SEAT_TOL_MM
    pads = _copper_pads(fp)
    if legality.pad_copper_overrun_mm(pads, gate) > legality.EPS:
        return None
    near = 0
    for p in pads:
        if not p.net_id:
            continue
        pr = legality.pad_rect(p)
        if gate.edge_clearance(pr) <= MATING_FINGER_EDGE_MM:
            near += 1
    if near < 2:
        return None
    loc = courtyard_for_side((crt or {}).get(ref), legality.footprint_side(fp))
    if loc is None:
        return None
    x0, y0, x1, y1 = legality.rotate_local_bounds(*loc, fp.rotation or 0.0)
    rect = (fp.x + x0, fp.y + y0, fp.x + x1, fp.y + y1)
    if gate.out_of_board_area(rect) >= legality.rect_area(rect) - 1e-6:
        return None             # wholly off the board: not at an edge
    if not (gate.rect_outside_amount(rect) > legality.EPS
            or gate.edge_clearance(rect) <= SEAT_TOL_MM):
        return None
    return rect


def seated_plugs(keepouts, pcb_data, pcb_file: Optional[str] = None
                 ) -> set:
    """The refs a mating keep-out LOCKS: the part a `mating:<ref>` entry
    (declared or derived) names, only while `_plug_seat_rect` finds it
    seated (#1098). A derived entry's plug is seated by construction; a
    declared one's may be in the staging pile, and a lock would leave it
    there unseated and unreported."""
    fps = getattr(pcb_data, 'footprints', None) or {}
    named = sorted({str(k.get('name', ''))[len(MATING_PREFIX):]
                    for k in keepouts or ()
                    if str(k.get('name', '')).startswith(MATING_PREFIX)}
                   & set(fps))
    bi = getattr(pcb_data, 'board_info', None)
    if not named or bi is None:
        return set()
    from .parser import extract_courtyard_sides
    path = pcb_file or getattr(pcb_data, 'source_path', None)
    crt = extract_courtyard_sides(path) if path else {}
    gate = legality.BoardOutlineGate(bi, 0.0)
    return {r for r in named
            if _plug_seat_rect(r, fps[r], crt, gate) is not None}


def is_board_copper_plug(fp) -> bool:
    """True when a footprint is a PCB-edge plug made of board copper (#1098):
    not assembled (an attr in `MATING_ATTRS`, no 3D model), not a net-tie (a
    solder jumper shorts pad groups; a plug does not), no drilled copper, and
    at least `MATING_MIN_FINGERS` netted finger pads.

    The pose-INDEPENDENT half of `derived_mating_keepouts`'s test, which then
    asks `_plug_seat_rect` whether the plug is seated. Lifted (#1155) so
    `mating.mating_face_local` can abstain on a plug: the board is a plug's
    mating body, so its courtyard points INTO the board by construction
    (StickHub J1) and says nothing about which way it opens."""
    if not (set(getattr(fp, 'attrs', ()) or ()) & set(MATING_ATTRS)):
        return False
    if getattr(fp, 'has_model', False):
        return False
    pads = _copper_pads(fp)
    if any((p.drill or 0) > 0 for p in pads):
        return False
    if getattr(fp, 'net_tie_groups', None):
        return False
    return sum(1 for p in pads if p.net_id) >= MATING_MIN_FINGERS


def derived_mating_keepouts(pcb_data, pcb_file: Optional[str] = None
                            ) -> Tuple[Dict, ...]:
    """The keep-outs a board's PCB-edge plugs imply, one per plug (#1098).

    A footprint is a plug when it is not assembled (an attr in
    `MATING_ATTRS` and no 3D model), is not a net-tie (a solder jumper
    shorts pad groups; a plug does not), carries >= `MATING_MIN_FINGERS`
    netted pads and no drilled one (finger copper, not a connector body),
    and is SEATED (`_plug_seat_rect`: no pad copper past the outline, >= 2
    fingers within `MATING_FINGER_EDGE_MM` of it, a courtyard on the board
    that reaches it). The keep-out is the courtyard's board rect inset by
    `MATING_INSET_MM`, on both faces, allowing the plug itself and every
    part with no copper pad (a slot like StickHub's H1, a logo).

    Declared, not guessed, wins: an intent or brief keep-out named
    `mating:<ref>` replaces the derived one (see `with_derived_keepouts`).
    """
    import glob as _glob
    path = pcb_file or getattr(pcb_data, 'source_path', None)
    fps = getattr(pcb_data, 'footprints', None) or {}
    bi = getattr(pcb_data, 'board_info', None)
    if not path or not fps or bi is None:
        return ()
    cands = [r for r, fp in fps.items() if is_board_copper_plug(fp)]
    if not cands:
        return ()
    from .parser import extract_courtyard_sides
    try:
        crt = extract_courtyard_sides(path)
    except Exception:                                        # noqa: BLE001
        return ()
    gate = legality.BoardOutlineGate(bi, 0.0)
    free = tuple(_glob.escape(r) for r, fp in sorted(fps.items())
                 if not _copper_pads(fp))
    out = []
    for ref in sorted(cands):
        fp = fps[ref]
        rect = _plug_seat_rect(ref, fp, crt, gate)
        if rect is None:
            continue
        ins = (rect[0] + MATING_INSET_MM, rect[1] + MATING_INSET_MM,
               rect[2] - MATING_INSET_MM, rect[3] - MATING_INSET_MM)
        if ins[2] <= ins[0] or ins[3] <= ins[1]:
            continue
        out.append({'name': MATING_PREFIX + ref,
                    'rect': tuple(round(v, 4) for v in ins),
                    'sides': ('F', 'B'),
                    'allow': (_glob.escape(ref),) + free,
                    'context': {'source': 'derived',
                                'derived_from': f"footprint {ref}: "
                                f"{'/'.join(sorted(set(fp.attrs) & set(MATING_ATTRS)))}"
                                f", no 3D model, courtyard at the outline",
                                'inset_mm': MATING_INSET_MM}})
    return tuple(out)


def with_derived_keepouts(keepouts, pcb_data, pcb_file: Optional[str] = None
                          ) -> Tuple[Dict, ...]:
    """`keepouts` plus the derived mating keep-outs no entry already names
    (#1098). A declared `mating:<ref>` replaces the derived one."""
    declared = tuple(keepouts or ())
    names = {k.get('name') for k in declared}
    return declared + tuple(k for k in derived_mating_keepouts(pcb_data,
                                                               pcb_file)
                            if k['name'] not in names)


def mating_keepouts(pcb_data, pcb_file: Optional[str] = None,
                    declared=()) -> Tuple[Dict, ...]:
    """The board's mating regions as the seat predicate and the grade
    resolve them (#1098): `with_derived_keepouts(declared, ...)`, keeping
    only the
    `mating:` entries. A declared `mating:<ref>` therefore replaces the
    derived one here exactly as it does there; the intent's other
    keep-outs are the floorplan grade's rules, not an assembly fact."""
    return tuple(k for k in with_derived_keepouts(declared, pcb_data,
                                                  pcb_file)
                 if str(k.get('name', '')).startswith(MATING_PREFIX))


def mating_keepout_findings(pcb_data, pcb_file: Optional[str] = None,
                            keepouts=None, declared=()) -> List[Dict]:
    """`[{ref, keepout, side, area_mm2}]`: parts inside a mating keep-out at
    the file's poses (#1098) -- `keepouts` when given, else
    `mating_keepouts(pcb_data, pcb_file, declared)`, so a checker handed the
    intent's keep-outs resolves the region the placer enforced.
    `keepout_hit` over each part's (rect, drilled-pad rect), resolved by
    `keepouts_for_ref` -- the same two functions the seat predicate and the
    grade call.

    The rect is the placement state's (`quench._Part`): the courtyard, else
    the pad bbox. A checker measuring courtyard-plus-pads here graded a part
    the placer had just seated (its pad poking past its own courtyard into the
    tongue) NOT BUILDABLE -- the generator more permissive than the checker,
    which is the one direction that may never happen. So both read one
    rect."""
    ks = (mating_keepouts(pcb_data, pcb_file, declared)
          if keepouts is None else tuple(keepouts))
    if not ks:
        return []
    from .parser import courtyard_for_side, extract_courtyard_sides
    from .utility import compute_footprint_bbox_local
    path = pcb_file or getattr(pcb_data, 'source_path', None)
    try:
        crt = extract_courtyard_sides(path) if path else {}
    except Exception:                                        # noqa: BLE001
        crt = {}
    out = []
    for ref, fp in sorted((pcb_data.footprints or {}).items()):
        side = legality.footprint_side(fp)
        has_tht = legality.footprint_has_through_pads(fp)
        loc = courtyard_for_side(crt.get(ref), side)
        if loc is None:
            try:
                loc = compute_footprint_bbox_local(fp)
            except Exception:                                # noqa: BLE001
                continue
        rot = fp.rotation or 0.0
        x0, y0, x1, y1 = legality.rotate_local_bounds(*loc, rot)
        rect = (fp.x + x0, fp.y + y0, fp.x + x1, fp.y + y1)
        tht = None
        if has_tht:
            tl = legality.far_side_local(fp, legality.far_courtyard_of(
                crt.get(ref), side))
            if tl is not None:
                tht = legality.offset_far(legality.rotate_far(tl, rot),
                                          fp.x, fp.y)
        sides = legality.sides_occupied(side, has_tht)
        for k in keepouts_for_ref(ks, ref, sides):
            a = keepout_hit(k, (rect, tht))
            if a:
                out.append({'ref': ref, 'keepout': k['name'],
                            'side': side, 'area_mm2': round(a, 4)})
    return sorted(out, key=lambda f: (f['keepout'], f['ref']))


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
            members.update([r for r in refs_all if fnmatch.fnmatchcase(r, pattern)])
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
    both. The part is CONFINED TO ITS ZONE for the run, where an ungated
    search would have walked it out.

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


def rotations_for_ref(intent: Intent, blocks: Dict[str, List[str]],
                      conflicts: Optional[Dict[str, List[tuple]]] = None
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

    `conflicts`, when given, collects every claim on a contradicted ref
    (`{ref: [claim, ...]}`) instead of raising -- for a caller that must
    keep going (the plan check bounds such a ref on every claimed angle).
    """
    out: Dict[str, Tuple[Optional[float], Optional[Tuple[float, ...]]]] = {}
    owner: Dict[str, str] = {}
    for z in intent.blocks:
        if z.rotation is None and z.rotation_candidates is None:
            continue
        claim = (z.rotation, z.rotation_candidates)
        for ref in blocks.get(z.name, ()):
            prev = out.get(ref)
            if prev is not None and prev != claim and conflicts is not None:
                seen = conflicts.setdefault(ref, [prev])
                if claim not in seen:
                    seen.append(claim)
                continue
            if prev is not None and prev != claim:
                raise IntentError(
                    f"{ref} is claimed by blocks {owner[ref]!r} and {z.name!r} "
                    f"with different rotations ({prev} vs {claim}). A part has "
                    f"one angle; resolve the overlap")
            out[ref] = claim
            owner[ref] = z.name
    return out


def declared_ladder(claim) -> Optional[List[float]]:
    """The seat ladder one `rotations_for_ref` claim gives, or None (#1117).

    `[rot]` for a declared `rotation`, the author's `rotation_candidates` in
    the author's order (a search keeps the FIRST angle that fits), and None
    for no claim -- which tells the caller to use its own lattice. The
    angles were normalised at load.

    The ONE mapping, so a caller cannot copy it and drift (#1117):
    `place_fanout_clearance`'s cap turns (#1122) read it.
    """
    if claim is None:
        return None
    rot, cands = claim
    return [rot] if rot is not None else list(cands)


def zone_entries(intent: Intent, blocks: Dict[str, List[str]],
                 bounds=None) -> Tuple[Dict, ...]:
    """The plain-dict zone rows, given ALREADY-RESOLVED blocks.

    Split out of `resolve_intent_gate` (#797) because the SEAT search needs the
    same rows and must not pay for the rest of that function: it has resolved
    its blocks already, and re-running the gate resolver there would report the
    `intent_zone_in_keepout` problems a second time.

    One construction, several callers, for the reason `resolve_intent_gate`'s
    own docstring gives about the join it replaced: a filter that must be
    remembered is a filter that will be forgotten at the next call site.

    `open_sides` (#1174) is `{ref: sides}` for the members whose declared
    edge the zone may not bound (`zone_open_sides`); it needs the board
    `bounds`, and is empty without them.
    """
    claimed = claimed_edges(intent) if bounds else {}
    return tuple(
        {'name': z.name,
         'rect': tuple(z.rect),
         'tolerance_mm': intent.zone_tolerance(z),
         'refs': tuple(blocks.get(z.name, ())),
         'open_sides': {r: s for r in blocks.get(z.name, ())
                        for s in (zone_open_sides(z.rect, bounds,
                                                  claimed.get(r)),) if s},
         'side': z.side,
         'exclusive': bool(z.exclusive)}
        for z in intent.blocks if z.rect is not None)


def tether_gate_spec(intent: Intent) -> Dict[str, object]:
    """The declared tether limits a per-move gate may hold (#1043). Plain data.

    A rule is armed only when the intent declares its limit AND grades it at
    ERROR -- the currency `grade_delta` counts.
    A rule the author demoted to warn is advice; gating the search on it would
    enforce something the author chose not to. Keys:

      decap_distance      `decaps.max_distance_mm`
      decap_pin_distance  `decaps.max_pin_distance_mm`
      proximity           a non-empty `proximity[]`
    """
    out: Dict[str, object] = {}
    d = dict(intent.decaps or {})
    if (d.get('max_distance_mm') is not None
            and intent.severity_of('decap_distance') == ERROR):
        # #1150 (c): the caps whose `decap_ungraded` is an ERROR, read as
        # `rule_decap_ungraded` reads it -- a cap a --decaps-from reference
        # holds (#1142) unless `severity.decap_ungraded` is explicit, or
        # every cap when that severity is error board-wide (#1102).
        explicit = 'decap_ungraded' in (intent.severity or {})
        out['decap_distance'] = {
            'limit': float(d['max_distance_mm']),
            'radius': float(d.get('search_radius_mm',
                                  groups_mod.DECAP_RADIUS_MM)),
            'exempt': tuple(d.get('exempt') or ()),
            'held': (() if explicit else
                     tuple(sorted(d.get('within_radius_refs') or ()))),
            'ungraded_error': intent.severity_of(
                'decap_ungraded', default=WARN) == ERROR}
    if (d.get('max_pin_distance_mm') is not None
            and intent.severity_of('decap_pin_distance') == ERROR):
        out['decap_pin_distance'] = {
            'limit': float(d['max_pin_distance_mm']), 'decaps': d}
    _held = [c for c in intent.proximity if not derived_claim(c)]
    if _held and intent.severity_of('proximity') == ERROR:
        out['proximity'] = tuple(dict(c) for c in _held)
    return out


def tether_pairings(tethers: Dict[str, object], pcb_data
                    ) -> List[Dict[str, object]]:
    """Every tether the gate holds, ELECTED ONCE on `pcb_data` as it stands.

    Each pairing is the grader's own: `decap_distance` from
    `groups.decap_populations` at the rule's radius (the election
    `rule_decap_distance` reads through `_Ctx.decap_populations`) -- with
    the cap's owners (`groups.rail_chips`), because the grade
    RE-ELECTS on the board it is handed and the gate must hold the pair it
    will elect, not the one elected here -- the pins
    and caps of `decap_pin_distance` from `supply_pins` and `decap_pin_caps`,
    and a proximity claim's refs as `rule_proximity` resolves them. Frozen
    here, except `decap_distance`'s: its term re-runs the election per pose
    (`groups.elect_live` over `rail`), since a frozen cap->IC pair is NOT
    conservative for a pair elected beyond the radius -- measured on run 32,
    C26 elected to U15 at 7.20mm (ungraded) walked to 3.07mm from U36 and
    the grade charged a NEW error the frozen term read as 0.

    What is left out, each graded elsewhere or not pose-dependent:
      * exempt caps (`decaps.exempt`), as the rules skip them;
      * caps that are no decap (`groups.decap_population`), which the rule
        never grades;
      * INFERRED supply pins (`channel == 'rail_net'`), graded at WARN under
        their own name `decap_pin_distance_inferred`;
      * a proximity claim with a missing ref or an unresolved pad name, which
        is `proximity_unresolved` whatever the poses.
      * caps a brief's proximity relation SUPERSEDES: the gate has no brief,
        so it holds them to `decap_distance` as well -- stricter, never looser.

    `decap_distance` pairs outside the radius at election (`graded: False`)
    are carried too: the grade calls them `decap_ungraded` (warn; per cap an
    error for a cap a --decaps-from reference holds, #1142), and one
    that walks INTO the radius past the limit becomes an error, which the gate
    must see. One whose `decap_ungraded` IS an error there is
    `stranded_error: True` (#1150 c), and the gate measures it wherever it
    stands, so a stranded cap may come home but not drift further out.
    """
    out: List[Dict[str, object]] = []
    fps = pcb_data.footprints or {}
    dd = tethers.get('decap_distance')
    if dd:
        near, beyond, _orph = groups_mod.decap_populations(
            pcb_data, radius=dd['radius'])
        rows = [(cap, ic, True) for ic in sorted(near)
                for cap, _d in near[ic]]
        rows += [(cap, ic, False) for cap, ic, _d in beyond]
        held = frozenset(dd.get('held') or ())
        for cap, ic, graded in sorted(rows):
            if any(fnmatch.fnmatchcase(cap, p) for p in dd['exempt']):
                continue
            rail = tuple(groups_mod.rail_chips(pcb_data, cap))
            out.append({'rule': 'decap_distance', 'name': f"{cap}->{ic}",
                        'refs': (cap,) + rail, 'cap': cap, 'ic': ic,
                        'rail': rail, 'graded': graded,
                        # #1150 (c): beyond the radius at election, but an
                        # ERROR there (`decap_ungraded` for a held cap).
                        'stranded_error': (not graded and (
                            cap in held or bool(dd.get('ungraded_error')))),
                        'limit': dd['limit'], 'radius': dd['radius']})
    dp = tethers.get('decap_pin_distance')
    if dp:
        spec = dp['decaps']
        by_net = _decap_caps_by_net(pcb_data)
        recs = _decap_supply_pins(pcb_data, pin_functions=spec.get('pin_functions'))
        for ref, rec in sorted(recs.items()):
            if not rec['pins'] or rec['channel'] == 'rail_net':
                continue
            ic_fp = fps.get(ref)
            if ic_fp is None:
                continue
            ic_side = legality.footprint_side(ic_fp)
            index = {id(p): i for i, p in enumerate(ic_fp.pads)}
            for pad, net in rec['pins']:
                _on, caps = decap_pin_caps(spec, ref, ic_side, pad, by_net)
                if not caps or id(pad) not in index:
                    continue        # uncovered: a design fact, not a pose
                cap_refs = tuple(c.reference for c in caps)
                out.append({'rule': 'decap_pin_distance',
                            'name': f"{ref}.{pad.pad_number}",
                            'refs': (ref,) + cap_refs, 'ic': ref,
                            'pad_index': index[id(pad)], 'net': net,
                            'caps': cap_refs, 'limit': dp['limit']})
    for i, claim in enumerate(tethers.get('proximity') or ()):
        ref, near = str(claim['ref']), str(claim['near'])
        a_fp, b_fp = fps.get(ref), fps.get(near)
        if a_fp is None or b_fp is None:
            continue
        basis = claim.get('basis', _PROXIMITY_DEFAULT_BASIS)
        if basis != 'body':
            subject, partners, _dec = proximity_pads(claim, a_fp, b_fp)
            spec = claim.get('pads') or {}
            if not subject or not partners or any(
                    set(spec.get(w) or ()) - {p.pad_number for p in got}
                    for w, got in ((ref, subject), (near, partners))):
                continue
        out.append({'rule': 'proximity', 'name': f"proximity[{i}]",
                    'refs': (ref, near), 'claim': dict(claim),
                    'basis': basis, 'limit': float(claim['max_mm'])})
    return out


# --------------------------------------------------------------------------
# rules
# --------------------------------------------------------------------------

class _Ctx:
    """What every rule is handed. Built once; rules never re-derive geometry."""

    def __init__(self, intent, pcb_data, pcb_file, state, blocks, locked,
                 outline):
        # #1098: plus a PCB-edge plug's derived mating keep-out, so the
        # grade, its roster and the pose grader all see what the seat
        # predicate enforces. The same board without a plug keeps the very
        # intent it was handed.
        _ks = with_derived_keepouts(intent.keepouts, pcb_data, pcb_file)
        if len(_ks) != len(intent.keepouts or ()):
            intent = dataclasses.replace(intent, keepouts=_ks)
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
        #: #1162: check_assembly's courtyard channel, for `_ctx_census` and
        #: `_ctx_overlap_exact`; a PoseGrader hands its own over, `grade`
        #: builds one on demand.
        self._census = None
        self._overlap_exact = None
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
        #: #1253. One row per declared keep-away rule, passing ones too: the
        #: closest pad pair, both pads' sides, and what the rule cannot see.
        self.keep_away_measured: List[Dict[str, object]] = []
        #: #1211. One row per declared copy target, passing ones too: the
        #: best-fit translation and each part's distance from the source's
        #: arrangement.
        self.replicate_measured: List[Dict[str, object]] = []
        #: One row per declared template: each member's drift from its
        #: template pose, passing ones too.
        self.template_measured: List[Dict[str, object]] = []
        #: #1051. One row per declared array, formed or not: the verdict,
        #: the order it could place and the members it could not.
        self.array_measured: List[Dict[str, object]] = []
        self._decap_pops: Dict[float, tuple] = {}
        self._supply_pins = None
        self._assembly_census = None
        self._bodies = None
        self._bodies_error = ''
        self._oob_exempt = None
        #: `{ref: census amount}` of every exempt part, in the census's own
        #: currency, so `rule_legality` can take them out of `oob_amount`.
        self._oob_exempt_census: Dict[str, float] = {}
        #: `_graphic_copper_off_board`'s answer, computed on first use.
        self._graphic_off = None
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
        grades), else the same courtyard reading at ZERO margin (#1301: the
        band is a mechanical measure, while the census counts a margin
        graze). Both skip the part's own milled rings here, and the rule
        does not, so they differ for a part that owns milled rings.

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
            from .connector_geometry import courtyard_overhang, geometry_for
            out: Dict[str, float] = {}
            for c in self.intent.edge_claims():
                ref = c['ref']
                part = self.parts.get(ref)
                lim = c.get('overhang_mm') or {}
                if part is None or lim.get('max') is None:
                    continue
                lo = float(lim.get('min', 0.0))
                hi = float(lim['max'])
                own = self.state._owned_rings(ref)  # noqa: SLF001
                amt = self.gate.rect_outside_amount(part.rect,
                                                    skip_rings=own)
                # #961: "exempt" is decided on the band's OWN currency, the
                # one `rule_edge_connector` grades -- the drawn body where it
                # can be measured, the courtyard at zero margin where it
                # cannot (#1301). `amt` still decides whether the census
                # counted the part at all.
                band, _basis, _body = _band_amount(
                    self, ref, c.get('edge'), courtyard_overhang(
                        geometry_for(self, self.pcb, self.pcb_file),
                        part.rect, skip_rings=own))
                # The same body path that must still see copper off the
                # outline (`rule_edge_connector`): a part with pads past the
                # edge stays counted, whatever its body reads.
                copper_ok = (not _body.get('body_measured')
                             or (_copper_outside_mm(self, ref) <= legality.EPS
                                 and not _unmodellable_pads(self, ref)))
                if (amt > legality.EPS and copper_ok
                        and lo - legality.EPS <= band <= hi + legality.EPS):
                    out[ref] = float(band)
                    self._oob_exempt_census[ref] = float(amt)
            out.update(self._mechanical_pose_exempt())
            self._oob_exempt = out
        return self._oob_exempt

    def _mechanical_pose_exempt(self) -> Dict[str, float]:
        """`{ref: census amount}` for `fixed_poses[]` entries of basis
        `mechanical` that the census counts off the outline while every pad
        of theirs is on the board at ZERO margin.

        A mechanical datum (a module's mating connector) cannot move, so its
        courtyard entering the edge-clearance band is a fact of the mating
        part, not a placement a budget can refuse -- counted, it made
        `oob_count: 0` unsatisfiable by any seed. Only at the declared pose:
        a part moved off its datum is a placement again and stays counted
        (the fixed-pose anchor names the move), and so does one on the other
        face than the entry declares, which is not the part the datum
        describes (the fixed-pose check refuses it there). Copper past the
        outline is never licensed: pads (their extent box, at zero margin)
        and, since a drawn tab is copper too (#962), the footprint's graphic
        copper, read off the census `grade_pad_legality` publishes."""
        from . import reconcile as _rc
        from .legality import footprint_side
        out: Dict[str, float] = {}
        for f in self.intent.fixed_poses:
            ref = str(f['ref'])
            part = self.parts.get(ref)
            fp = (self.pcb.footprints or {}).get(ref)
            if f.get('basis') != 'mechanical' or part is None or fp is None:
                continue
            if not _rc.same_pose(_rc.fixed_pose_tuple(f),
                                 (fp.x, fp.y, (fp.rotation or 0.0) % 360.0)):
                continue
            if (f.get('side') in ('F', 'B')
                    and footprint_side(fp) != f['side']):
                continue
            amt = self.gate.rect_outside_amount(
                part.rect, skip_rings=self.state._owned_rings(ref))  # noqa: SLF001
            if amt <= legality.EPS:
                continue
            pads = legality.build_part_pads({ref: fp}, 0.0, None).get(ref)
            ext = (pads.extent(fp.x, fp.y, fp.rotation or 0.0)
                   if pads is not None else None)
            if ext is None or self.zero_gate().rect_outside_amount(
                    ext) > legality.EPS:
                continue
            if ref in self._graphic_copper_off_board():
                continue
            out[ref] = float(amt)
            self._oob_exempt_census[ref] = float(amt)
        return out

    def _graphic_copper_off_board(self) -> frozenset:
        """Refs whose footprint GRAPHIC copper reaches past the outline, or
        whose graphic copper the census could not measure -- a zero reading
        certifies nothing for them. #962's one measurement
        (`legality._graphic_copper_channel`, check_drc's census), so this
        and `grade_pad_legality`'s `oob_graphic_copper_refs` cannot disagree.
        Computed once per grade, and only when a datum reaches the question."""
        if self._graphic_off is None:
            ch = legality._graphic_copper_channel(self.pcb, 0.0)  # noqa: SLF001
            self._graphic_off = frozenset(
                [str(r) for r, _amt in ch['oob_graphic_copper_refs']]
                + [str(u[0]) for u in ch['oob_graphic_copper_unmeasured']
                   if u and u[0]])
        return self._graphic_off

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
        return drawn_body_rect(self.body_geom(ref),
                               self.pcb.footprints.get(ref))

    def body_geom(self, ref: str):
        """`body.BodyGeometry` for one part, or None -- the lazy, memoised
        read `body_rect` uses (#1155: the facing conjunct reads it too)."""
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
        return self._bodies.get(ref)

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
            self._supply_pins = _decap_supply_pins(self.pcb, pin_functions=kw)
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


def edge_reach_mm(state) -> float:
    """The `margin` every reader passes `_nearest_edge`: the board's EDGE
    clearance as placement resolves it (`state.board_edge_clearance`, the
    floor routing writes back, #1301), never the gate's margin
    `max(clearance, edge clearance)`.

    The clearance half of that max is a netclass value a routing step's DRC
    writeback lowers (`--clearance-ceiling`), so on a board whose class
    clearance is above its edge floor (sonde_xilinx: 0.254 over 0.20) one
    pose was read for the sides it reaches at 0.254 placed and 0.20 routed,
    while #1301 had already made the band read the same on both. The edge
    clearance is the value placement and routing agree on, and the distance
    a seated part's pad copper stands inside its edge. Wherever the class
    clearance is at or under it, it is the margin read before."""
    edge = getattr(state, 'board_edge_clearance', None)
    return float(edge if edge is not None else state.edge_gate.margin)


def _nearest_edge(rect, bounds, margin: float = 0.0, facing=None) -> str:
    """The board edge `rect` sits on: the nearest side of `bounds`. When the
    rect REACHES two or more sides -- comes within `margin` of them or past
    them, the occupancy reading's "outside" at that margin -- it is, of
    those: the one the part's mating face points at (`facing`: an edge name,
    or a zero-argument callable returning one or None, asked only here); else
    the one side it sticks out past by more than the margin; else the one it
    runs along the farthest, the nearest of those on a tie. Every reader
    passes `edge_reach_mm` (#1301), so a placed board and its routed copy
    read one edge.

    A header laid corner to corner along a long side reaches the short sides
    with its ends too, and which of them is nearest is an accident of a tenth
    of a millimetre. Measured: ch32v003_usb's J2/J3 (1x10, 25.5 mm along a
    26.1 mm side) read `south` at 0.13 mm where the west/east sides they run
    along were 0.22 mm away, and fusb302_pd_dev's J101/J102 (1x24, 62 mm
    along a 66.75 mm side) read `west`; the emitted intent then declared an
    edge no seat can hold such a part on, and the grade failed the designer's
    own pose against the edge it was moved to. Over the corpus's 478 human
    boards, 3123 parts read an edge, 311 of them reach two sides, and 37 read
    differently from the bare nearest side -- headers and card edges along a
    side that read at an end (also rp2040_dev's and rp2350_dev's 1x20s, three
    DDR testbeds' edge connectors), and two DSUBs whose strong face now
    decides; none reads against its part's mating face. The length alone,
    without the face and overhang rungs, misread 16 more: jacks and modules
    in a corner, their nose out past one side and a flank grazing the other
    (14 with a mating face, an SFP cage 5.3 mm out)."""
    d = {'west': rect[0] - bounds[0], 'north': rect[1] - bounds[1],
         'east': bounds[2] - rect[2], 'south': bounds[3] - rect[3]}
    reached = [k for k in d if d[k] <= margin + legality.EPS]
    if len(reached) > 1:
        face = facing() if callable(facing) else facing
        if face in reached:
            return face
        out = [k for k in reached if d[k] < -margin - legality.EPS]
        if len(out) == 1:
            return out[0]
        along = {'west': rect[3] - rect[1], 'east': rect[3] - rect[1],
                 'north': rect[2] - rect[0], 'south': rect[2] - rect[0]}
        far = max(along[k] for k in reached)
        return min((k for k in reached if along[k] >= far - legality.EPS),
                   key=lambda k: d[k])
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


#: #1155: the mount modes whose mating face must point at the declared edge.
FACING_MOUNTS = ('edge_mount', 'through_edge')


def edge_facing_policy(entry: Dict, fp) -> str:
    """Whether `rule_edge_connector`'s facing conjunct grades this entry:
    `'graded'`, or `'exempt: <why>'` (#1155).

    In order: a declared `mating_face` is graded, whatever the mount -- the
    edge seat turns the part by it, so the grade checks the same face (a
    vertical header's names the side its cable leaves); a vertical
    `mount_mode` is exempt (it mates off a face, not the edge --
    `VERTICAL_MOUNTS`), and so are card-edge fingers
    (`part_class.is_card_fingers`: they ARE the edge, with no opening to
    face; their 0.64 mm pad field read as a weak face on interf_u BUS1);
    `edge_mount` / `through_edge` is graded; a declared
    `class` grades only `edge_receptacle`; with none declared, the part's own
    class (`part_class.classify_part`) decides the same way.

    `connector_affinity` and the actuators are exempt, measured: on the
    designer's StickHub, J5 (a JST-SH horizontal the emitter declares on the
    north edge) opens EAST, sitting in a corner -- a connector the board
    merely keeps near an edge is not one whose opening must face it. An
    actuator's plastic says nothing about which way it is pressed (glasgow
    SW1: courtyard north, fab south)."""
    ctxd = entry.get('context') or {}
    mm = ctxd.get('mount_mode')
    if entry.get('mating_face'):
        return 'graded'
    if mm in VERTICAL_MOUNTS:
        return 'exempt: a %s mates off a face, not the edge' % mm
    from .part_class import is_card_fingers
    if fp is not None and is_card_fingers(fp):
        return 'exempt: card-edge fingers are the edge, with no opening to face'
    if mm in FACING_MOUNTS:
        return 'graded'
    cls = entry.get('class')
    how = 'declared'
    if not cls:
        from .part_class import classify_part
        cls = classify_part(fp, getattr(fp, 'reference', '') or '').name
        how = 'part_class'
    if cls == 'edge_receptacle':
        return 'graded'
    return 'exempt: %s class %s' % (how, cls or 'none')


def edge_facing(entry: Dict, fp, geom, rotation: Optional[float] = None
                ) -> Dict:
    """Which board edge a declared edge part's mating face points at, and
    whether `rule_edge_connector` fails it (#1155).

    `geom` is the part's `body.BodyGeometry` (None reads as no body);
    `rotation` defaults to the footprint's own, and is what a seat trying a
    turn passes. Where the body names no face, the entry's declared
    `mating_face` is the face (basis `declared`, strong), and the entry is
    graded whatever its class. Returns `{policy, facing, local_face, basis, offset_mm,
    strong, reason, fails}`. `fails` is True only when the policy grades the
    entry, the face is decisive at that rotation, and it is not the declared
    edge. `strong` (the face's offset reaches `mating.FACE_STRONG_OFFSET_MM`)
    decides the severity: a strong failure is an ERROR, a weak one a WARN --
    the tiering chosen after the phase verifier found stock receptacles whose
    weak face is wrong (`placement.mating`'s docstring).

    THE predicate the rule calls, so every reader asks one answer for which
    way a part faces. A wrong WEAK face (`placement.mating`) reads as a
    pass. A footprint's own "PCB Edge" marker
    (#1245) confirms a face (strong) or vetoes it (an abstention) where it
    draws one, through `mating_face_local`."""
    from .mating import LocalFace, facing_edge, mating_face_local
    policy = edge_facing_policy(entry, fp)
    lf = (mating_face_local(fp, geom) if geom is not None
          else LocalFace(None, 0.0, '', 'no body geometry'))
    if lf.face is None and entry.get('mating_face') in MATING_FACES:
        # The body names no face; the intent declares one. A declaration is
        # no weaker than a strong body reading: a wrong turn fails at ERROR.
        lf = LocalFace(entry['mating_face'], 0.0, 'declared', '', lf.votes,
                       strong=True, marker=lf.marker)
    rot = (fp.rotation or 0.0) if rotation is None else rotation
    facing = facing_edge(lf.face, rot)
    edge = entry.get('edge')
    reason = lf.reason or ('' if facing or not lf.face else
                           'the pose is more than 30 degrees off a quarter '
                           'turn, so the face names no edge')
    return {'policy': policy, 'facing': facing, 'local_face': lf.face,
            'basis': lf.basis, 'offset_mm': lf.offset_mm,
            'strong': bool(lf.strong and facing), 'reason': reason,
            'fails': bool(policy == 'graded' and edge and facing
                          and facing != edge)}


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
    a zone smaller than the courtyard is unsatisfiable by construction (run 2:
    H1/H3 could never be seated in their 0.4mm zones)."""
    zw = zone_rect[2] - zone_rect[0] + 2 * tol
    zh = zone_rect[3] - zone_rect[1] + 2 * tol
    w = part_rect[2] - part_rect[0]
    h = part_rect[3] - part_rect[1]
    return (w <= zw + 1e-9 and h <= zh + 1e-9) or \
           (h <= zw + 1e-9 and w <= zh + 1e-9)


def zone_escape(zone_rect, part_rect, anchor: bool,
                open_sides: Sequence[str] = ()) -> Tuple[float, str]:
    """How far `part_rect` is outside `zone_rect`, in mm, and on which side.
    Exactly 0.0 when contained.

    `open_sides` (#1174) are sides the zone does not bound: an edge-claim
    member's declared edge, where the zone side lies on the outline
    (`zone_open_sides`). The edge rule grades that side, so the zone must not
    -- an edge receptacle's mating face is SUPPOSED to cross it.

    THE zone measurement, shared by `rule_zone_containment` and the
    placement state's intent gate (#702), for the reason `keepout_hit`'s
    docstring gives about
    the keep-out one: a pose the optimizer accepts that the grade then flags is
    an exit 4 on a board this tool placed itself, and two implementations of
    "outside its zone" is how that happens.

    `anchor` selects the spec-COORDINATE branch -- grade the courtyard CENTRE,
    because a zone smaller than the courtyard cannot contain it at any
    rotation. The CALLER passes the decision `zone_fits_courtyard` makes,
    because it is pose-INVARIANT (it reads only w/h, and tests both orders) and
    a per-pose gate must resolve it once per part rather than once per
    candidate pose.

    Note it tests the courtyard CENTRE, not the footprint ORIGIN (x, y). The
    two differ by (b[0]+b[2])/2 -- 17 of 65 parts on splitflap_driver and 6
    of 89 on tigard have an offset centre, up to 10.15mm on tigard J3 -- so
    a gate built on the origin would admit, by up to 10mm, poses this rule
    flags.
    """
    if open_sides:
        zone_rect = open_zone_rect(zone_rect, open_sides)
    if anchor:
        cx = (part_rect[0] + part_rect[2]) / 2.0
        cy = (part_rect[1] + part_rect[3]) / 2.0
        return _rect_escape(zone_rect, (cx, cy, cx, cy))
    return _rect_escape(zone_rect, part_rect)


def claimed_edges(intent: 'Intent') -> Dict[str, Tuple[str, ...]]:
    """`{ref: (edge, ...)}` for every `edge_claims()` entry naming an edge
    AND bounding its overhang (`overhang_mm.max`).

    The claims, not `edge_connectors`: a `connector_affinity` entry makes no
    seat claim, so it must not open a zone side either (#1174). And only a
    bounded one: the open side hands that side to the edge rule, which bounds
    how far past the edge a part may go only through the band's max -- a
    claim without one (what the design brief compiles for a user-facing
    receptacle on a pile) would let a part sit wholly off the board and pass
    both rules (phase-1 verifier: 12.5 mm off, 0 errors). The population is
    every edge claim, not only receptacles: the emitter also claims the
    edge of a class-less part it sees overhanging (a fiducial, a cap), each
    with the band it measured."""
    out: Dict[str, set] = {}
    for c in intent.edge_claims():
        e = c.get('edge')
        if (e in _EDGE_AXIS
                and (c.get('overhang_mm') or {}).get('max') is not None):
            out.setdefault(str(c.get('ref')), set()).add(e)
    return {r: tuple(sorted(s)) for r, s in out.items()}


def zone_open_sides(zone_rect, bounds, edges) -> Tuple[str, ...]:
    """The sides of `zone_rect` an edge-claim member's courtyard may cross
    (#1174): each of `edges` (the member's declared edges) on which the zone
    side lies on the outline's bounding box (within `_EDGE_SPAN_TOL_MM`) or
    beyond it.

    Why: an edge receptacle's body is meant to reach -- often to cross -- the
    board edge, and a block zone that ends at that edge (which is where the
    emitter clamps it, `emit_intent`) then failed the very connector the edge
    rule passed: rp2350's J1 at 1.93 mm. The edge rule owns that side. A zone
    side INSIDE the board stays closed: the author drew it there."""
    if zone_rect is None or not bounds or not edges:
        return ()
    tol = _EDGE_SPAN_TOL_MM
    on = {'west': zone_rect[0] <= bounds[0] + tol,
          'north': zone_rect[1] <= bounds[1] + tol,
          'east': zone_rect[2] >= bounds[2] - tol,
          'south': zone_rect[3] >= bounds[3] - tol}
    return tuple(e for e in ('west', 'north', 'east', 'south')
                 if e in edges and on[e])


#: How far `zone_pose_feasibility` pushes an open zone side (#1174): far past
#: any board, so no pose the grade admits is left out of the superset.
_OPEN_SIDE_REACH_MM = 1000.0


def _reach_open_zone(zone_rect, open_sides):
    """`open_zone_rect` with a finite reach, for the candidate algebra."""
    r = _OPEN_SIDE_REACH_MM
    z = list(zone_rect)
    for side in open_sides or ():
        if side == 'west':
            z[0] -= r
        elif side == 'north':
            z[1] -= r
        elif side == 'east':
            z[2] += r
        elif side == 'south':
            z[3] += r
    return tuple(z)


def open_zone_rect(zone_rect, open_sides):
    """`zone_rect` with each of `open_sides` pushed to infinity (#1174)."""
    z = list(zone_rect)
    for side in open_sides or ():
        if side == 'west':
            z[0] = -math.inf
        elif side == 'north':
            z[1] = -math.inf
        elif side == 'east':
            z[2] = math.inf
        elif side == 'south':
            z[3] = math.inf
    return tuple(z)


def zone_is_anchor(zone_rect, part, tol: float) -> bool:
    """Is this zone a spec-COORDINATE rather than a region? (#799)

    ONE definition, for the consumers that must agree: `rule_zone_containment`
    grades on it, and the intent contradiction check asks it before deciding
    where a member may sit. A re-derivation that disagreed would move the
    anchor boundary for one consumer only.

    Pose-INVARIANT: `zone_fits_courtyard` reads only w/h and tests both orders,
    so `part.rot` and `part.rot + 90` settle it for the whole 90-degree
    lattice.
    """
    return not any(
        zone_fits_courtyard(zone_rect, part.grade_rect(0.0, 0.0, r), tol)
        for r in (part.rot % 360, (part.rot + 90) % 360))


def _anchor_reachable(zone, part, tol: float, claim=None) -> bool:
    """Could the grade read `zone` as an ANCHOR for `part` (`zone_is_anchor`)
    at any rotation the seed may give it? A plan bound charging such a
    member is not sound: seated where it does not fit, the grade does not
    hold it to the zone at all.

    The rotations: `claim` is the part's entry in `rotations_for_ref` (every
    block's declaration, not only the zone's own block -- a block may declare
    an angle with no zone), its decision or else its candidate set; a LOCKED
    part, or one with no claim, keeps its own 90-degree lattice (a locked part
    is never turned, whatever a block declares -- b6a63ad8 verifier).
    (`zone_fits_courtyard` tests both orders, so `r` stands for `r + 90`.)"""
    rot, cands = claim if claim else (None, None)
    if getattr(part, 'locked', False) or (rot is None and not cands):
        rots = [part.rot]
    elif rot is not None:
        rots = [rot]
    else:
        rots = list(cands)
    from .legality import rotate_local_bounds
    # Turned from the 0-degree box, not read from the part's rotation
    # cache: a declared angle off its lattice has no entry, and the cache
    # answers a miss with the 0-degree box.
    return any(
        not zone_fits_courtyard(
            zone.rect, rotate_local_bounds(*part.grade_by_rot[0.0], r % 360),
            tol)
        for r in rots)


def zone_origin_box(zone_rect, bounds, tol: float):
    """Where a part with LOCAL box `bounds` may put its ORIGIN, at ONE rotation.

    `zone_escape(zone_rect, part_rect, anchor=False) <= tol`, solved for the
    origin. Containment at this rotation is `zone[0]-tol <= x + b[0]` and
    `x + b[2] <= zone[2]+tol`, so `x` in `[zone[0]-tol-b[0], zone[2]+tol-b[2]]`.

    Returns a possibly-INVERTED box: `hi < lo` on an axis means this rotation
    admits nothing (`zone_census_offsets` detects the inversion), kept rather
    than returning None so every caller branches the same way.

    THE COURTYARD IS NOT CENTRED ON THE FOOTPRINT ORIGIN, so this is the
    algebra and never a half-extent -- the offset reaches 10.15mm on tigard
    J3.

    Lives here, beside `zone_escape` whose inverse it is: the intent
    contradiction check (#799) needs it PER ROTATION.
    """
    x0, y0, x1, y1 = (float(v) for v in zone_rect)
    b0x, b0y, b2x, b2y = bounds
    return (x0 - tol - b0x, y0 - tol - b0y, x1 + tol - b2x, y1 + tol - b2y)


def _anchor_origin_box(zone_rect, bounds, tol: float):
    """`zone_origin_box`'s spec-COORDINATE twin: the zone holds the part's
    courtyard CENTRE rather than its whole courtyard.

    `zone_escape(..., anchor=True)` measures `((b0+b2)/2, (b1+b3)/2)` offset
    from the origin, so the admissible origin box is the zone shifted by minus
    that offset. THE GRADE'S convention, deliberately -- a gate on the
    footprint ORIGIN would differ by up to 10.15mm on tigard J3
    (`zone_escape`'s docstring).

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
                          keepouts, open_sides: Sequence[str] = ()
                          ) -> Dict[str, object]:
    """Does `zone_rect` MINUS the keep-outs still hold `part` at some rotation?

    #702 refuses an intent whose keep-out swallows a zone entirely. That is the
    total-coverage case; this is the question underneath it, and they coincide
    only there. Measured: zone `[10,10,20,20]` at tolerance 0 against a keep-out
    `[10,10,19.9,20]` -- 99% of the zone -- raises nothing today while the
    member has ZERO satisfying poses. Two keep-outs covering half each are
    missed the same way, and neither triggers a per-entry test.

    Why it must be refused where it is authored rather than discovered later:
    the #702 intent gate is termwise-monotone, so for such a member `keepout`
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
    (Not `seeder.pose_ok`, which applies no zone predicate at all. The kernel
    matches the GRADE, which is whose contradiction this is.) So the verdict cannot drift from the grade,
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
    if open_sides:
        # #1174: the grade does not bound an edge member's open side, so the
        # superset must not either. Pushed out by a finite reach rather than
        # to infinity, because the candidate algebra samples box ENDS.
        zone_rect = _reach_open_zone(zone_rect, open_sides)

    def _search(entries):
        """First (x, y, rot) satisfying zone AND every entry, or None."""
        for rot in rots:
            b = part.grade_rect(0.0, 0.0, rot)
            t = part.tht_rect(0.0, 0.0, rot)
            box = (_anchor_origin_box(zone_rect, b, tolerance) if anchor
                   else zone_origin_box(zone_rect, b, tolerance))
            if box[2] < box[0] or box[3] < box[1]:
                continue
            holes = []
            for k in entries:
                for lb in (b,) + tuple(legality.far_boxes(t)):
                    f = _forbidden_origin_rect(k, lb)
                    if f is not None:
                        holes.append(f)
            cx = _axis_candidates(box[0], box[2],
                                  [v for h in holes for v in (h[0], h[2])])
            cy = _axis_candidates(box[1], box[3],
                                  [v for h in holes for v in (h[1], h[3])])
            for x in cx:
                for y in cy:
                    r = part.grade_rect(x, y, rot)
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

    # The rects alone refuse. Sound, so attribute it -- single-lift then
    # joint-lift escalation, in the currency this function has (names, from an
    # exact search) rather than pose COUNTS. Inventing a count here would be a
    # figure a reader could quote.
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
        # `keepouts_freeing` means: lifting this entry
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
        # #1206: a FarSide keeps its clusters; anything else is one box.
        self._t = (tht_local if isinstance(tht_local, legality.FarSide)
                   else tuple(tht_local) if tht_local is not None else None)

    def rect(self, x: float, y: float, rot: float):
        b = legality.rotate_local_bounds(*self._b, rot)
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def tht_rect(self, x: float, y: float, rot: float):
        if self._t is None:
            return None
        return legality.offset_far(legality.rotate_far(self._t, rot), x, y)

    # A local part has one ladder: the one it was built from (#1182).
    grade_rect = rect


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

    # #1174: the grade's open sides, so neither half refuses an edge member
    # the grade lets sit past its zone's open side.
    _claimed = claimed_edges(intent)
    _bounds = getattr(getattr(pcb_data, 'board_info', None), 'board_bounds',
                      None)
    for z in zoned:
        tol = intent.zone_tolerance(z)
        mine = {r: member_sides[r] for r in blocks.get(z.name, ())
                if r in member_sides
                and not zone_open_sides(z.rect, _bounds, _claimed.get(r))}
        if not mine:
            continue
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
            v = zone_pose_feasibility(
                z.rect, tol, part, bound,
                open_sides=zone_open_sides(z.rect, _bounds, _claimed.get(ref)))
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


def open_side_pad_escape(zone_rect, pad_rect, open_sides, bounds=None
                         ) -> Tuple[float, Optional[str]]:
    """`(mm, side)`: how far a member's PAD bbox reaches past one of its
    zone's open sides (#1174) -- past the BOARD's side there, when `bounds`
    (the outline bbox) is given -- or `(0.0, None)`.

    An open side lets an edge member's body overhang the edge, which the edge
    rule's band then bounds -- but only through the band's `max`, measured on
    the body's far side. A part wholly past the edge (pads included) passed
    both rules whenever that max covered its depth (second verifier: a 4 mm
    part 2.1 mm off a 40 mm board, `max 5`, 0 errors). Its pads are soldered
    to the board, so past an open side the pads are what stays graded, and
    against the outline: a side opens when it lies ON or BEYOND the outline,
    so a zone drawn 5 mm past the board measured at its own side let pads
    2.4 mm off the board grade clean (round-2 verifier)."""
    if pad_rect is None or not open_sides:
        return 0.0, None
    zx0, zy0, zx1, zy1 = zone_rect
    if bounds:
        zx0, zy0 = max(zx0, bounds[0]), max(zy0, bounds[1])
        zx1, zy1 = min(zx1, bounds[2]), min(zy1, bounds[3])
    x0, y0, x1, y1 = pad_rect
    past = {'north': zy0 - y0, 'south': y1 - zy1,
            'west': zx0 - x0, 'east': x1 - zx1}
    best = (0.0, None)
    for side in open_sides:
        if past.get(side, 0.0) > best[0]:
            best = (past[side], side)
    return best


def _member_pad_rect(ctx, ref):
    """The board-frame bbox of `ref`'s pads at the pose the context grades,
    or None. A `_PosedState` (the seat search's view) answers the pose it is
    asking about; a QuenchState its parts' current pose.

    Castellated pads are left out: they straddle the outline by design, and
    the edge rule's copper conjunct (`_copper_outside_mm`) skips them too.
    `PartPads` keeps one rect per copper pad in `fp.pads` order, so the
    rects pair with the footprint's copper pads one to one."""
    st = ctx.state
    lc = getattr(st, 'legality_ctx', None)
    pp = (getattr(lc, 'parts', None) or {}).get(ref) if lc is not None else None
    if pp is None:
        return None
    try:
        if callable(getattr(st, 'pose', None)):
            x, y, rot = st.pose(ref)
        else:
            p = st.parts[ref]
            x, y, rot = p.x, p.y, p.rot
        rects = pp.pad_rects(x, y, rot) or ()
    except Exception:                                        # noqa: BLE001
        return None
    fp = (getattr(ctx.pcb, 'footprints', None) or {}).get(ref)
    if fp is not None:
        cu = [p for p in (fp.pads or ()) if legality._pad_carries_copper(p)]
        if len(cu) == len(rects):
            rects = [r for r, p in zip(rects, cu)
                     if not getattr(p, 'castellated', False)]
    if not rects:
        return None
    return (min(r[0] for r in rects), min(r[1] for r in rects),
            max(r[2] for r in rects), max(r[3] for r in rects))


def rule_zone_containment(ctx) -> Iterator[Violation]:
    claimed = claimed_edges(ctx.intent)
    for z in ctx.intent.blocks:
        if z.rect is None:
            continue
        tol = ctx.intent.zone_tolerance(z)
        for ref in ctx.blocks.get(z.name, ()):
            part = ctx.parts.get(ref)
            if part is None:
                continue
            # #1174: an edge-claim member's declared edge, where the zone
            # ends at the outline, is the edge rule's to grade.
            opened = zone_open_sides(z.rect, ctx.outline_bounds,
                                     claimed.get(ref))
            if opened:
                pout, pside = open_side_pad_escape(
                    z.rect, _member_pad_rect(ctx, ref), opened,
                    ctx.outline_bounds)
                # At zero, not at the zone's tolerance: that is slack on
                # where a part sits in its zone, and a soldered pad past the
                # board's edge is off the board by any amount.
                if pout > legality.EPS:
                    yield Violation(
                        rule='zone_containment',
                        severity=ctx.sev('zone_containment'), ref=ref,
                        block=z.name,
                        message=(f"{ref}'s pads reach {pout:.2f}mm past the "
                                 f"board's {pside} edge, the open side of "
                                 f"block {z.name!r} -- that side is open to "
                                 f"its overhang, not to its pads leaving the "
                                 f"board"),
                        measured={'rect': [round(v, 4) for v in part.rect],
                                  'outside_mm': round(pout, 4), 'axis': pside,
                                  'pads_graded': True,
                                  'open_sides': list(opened)},
                        expected={'zone': list(z.rect), 'tolerance_mm': 0.0})
                    continue
            if not zone_fits_courtyard(z.rect, part.rect, tol):
                # Spec-coordinate zone: grade the part's CENTER against it.
                out, axis = zone_escape(z.rect, part.rect, True, opened)
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
                                  'anchor_graded': True,
                                  'open_sides': list(opened)},
                        expected={'zone': list(z.rect), 'tolerance_mm': tol})
                continue
            out, axis = zone_escape(z.rect, part.rect, False, opened)
            if out > tol:
                yield Violation(
                    rule='zone_containment',
                    severity=ctx.sev('zone_containment'), ref=ref, block=z.name,
                    message=(f"{ref} courtyard extends {out:.2f}mm past the "
                             f"{axis} edge of block {z.name!r}"),
                    measured={'rect': [round(v, 4) for v in part.rect],
                              'outside_mm': round(out, 4), 'axis': axis,
                              'open_sides': list(opened)},
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

    WARN by default, and that is the design rather than timidity. No search
    move carries a side -- `_Part.side` is set once at construction (#836) --
    and `place_simple` puts a part on a face only where a declaration names
    one for it (`declared_side_claims`: `blocks[].side`,
    `edge_connectors[].side`, `fixed_poses[].side`), which `assembly.sides`
    is not. So an ERROR here would be a red mark no run could clear. An
    author who wants the hard gate sets `severity: {"assembly_side":
    "error"}` and means it.

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
    from .connector_geometry import courtyard_overhang, geometry_for
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
        # fixed WARN, never the configured severity (the loader refuses
        # `error` for it): no search move carries a side (#836), and
        # place_simple puts the connector on this face before it places, as
        # the message says.
        if c.get('side') and part.side != c['side']:
            yield Violation(
                rule='edge_connector_side', severity=WARN, ref=ref,
                message=(f"{ref} is on {part.side}.Cu, but the declared "
                         f"viewing face puts it on {c['side']}.Cu -- advisory:"
                         f" no placement search moves a part between faces; "
                         f"place_simple.py applies the declared face "
                         f"before placing"),
                measured={'side': part.side}, expected={'side': c['side']})
        # #1301: the COURTYARD's overhang at ZERO margin. It was the occupancy
        # reading at the gate's margin -- `margin - gap` inside the board,
        # `overhang + margin` outside -- and the margin is max(clearance, edge
        # clearance), which every routing step's DRC writeback rewrites, so
        # one pose graded differently placed and routed (daplink: 3 errors
        # after routing, nothing moved).
        amount = courtyard_overhang(geometry_for(ctx, ctx.pcb, ctx.pcb_file),
                                    part.rect)
        lim = c.get('overhang_mm') or {}
        lo = float(lim.get('min', 0.0))
        hi = lim.get('max')
        # #961: the band is graded on the DRAWN BODY's overhang past the
        # outline -- summed over the sides it crosses, the form of the reading
        # it replaces -- at zero margin, wherever that body can be measured.
        # Graded at a margin, the courtyard let esp_prog's USB1, 0.15 mm
        # INSIDE its edge, read 0.10 and satisfy a band `check_drc
        # --check-pad-edge` failed. Where no body can be read `band` IS
        # `amount`, and `overhang_basis` says which it was. `amount` keeps its
        # other job: the setback conjunct's no-overhang gate below, which
        # reads the courtyard on either basis.
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
                          # the magnitude, for a reader that ranks by it.
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
            actual = _nearest_edge(
                seat_rect, ctx.outline_bounds, edge_reach_mm(ctx.state),
                facing=lambda: edge_facing(c, ctx.pcb.footprints.get(ref),
                                           ctx.body_geom(ref))['facing'])
            if actual != edge:
                yield Violation(
                    rule='edge_connector', severity=ctx.sev('edge_connector'),
                    ref=ref, message=(f"{ref} sits nearest the {actual} edge "
                                      f"but is declared on the {edge} edge"),
                    measured={'edge': actual, 'basis': basis},
                    expected={'edge': edge})
        # #1155: the FACING conjunct -- which way the body leaves its pad row
        # (`edge_facing`). The
        # nearest-edge conjunct above reads where the part SITS, and a DSUB
        # seated backwards sits nearest its declared edge whenever the back
        # of its shell grazes it: sonde_xilinx J2, east, shell over the
        # board, passed every grader. A STRONG face (offset >= 2 mm) failing
        # is an ERROR; a weak one a WARN -- three stock receptacles read a
        # confident wrong face below 2 mm (`placement.mating`).
        _ef = edge_facing(c, ctx.pcb.footprints.get(ref), ctx.body_geom(ref))
        evidence['facing'] = {
            'edge': edge, 'facing': _ef['facing'],
            'local_face': _ef['local_face'], 'basis': _ef['basis'],
            'offset_mm': _ef['offset_mm'], 'strong': _ef['strong'],
            'policy': _ef['policy'], 'reason': _ef['reason'],
            'disposition': ('exempt' if _ef['policy'] != 'graded'
                            else 'no_edge' if not edge
                            else 'abstained' if not _ef['facing']
                            else 'fail' if _ef['fails'] else 'pass')}
        if _ef['fails']:
            yield Violation(
                rule='edge_connector',
                severity=(ctx.sev('edge_connector') if _ef['strong']
                          else WARN),
                ref=ref,
                message=(f"{ref}'s body leaves its pad row toward the "
                         f"{_ef['facing']} ({_ef['basis']}, "
                         f"{_ef['offset_mm']:.2f}mm), away from its declared "
                         f"{edge} edge -- its mating face points "
                         f"{_ef['facing']}, not out of the {edge} edge"
                         + ('' if _ef['strong'] else
                            ' (a weak face, under 2 mm: a WARN)')),
                # No 'edge' key: the nearest-edge violation owns that one.
                measured={'facing': _ef['facing'],
                          'facing_basis': _ef['basis'],
                          'facing_offset_mm': _ef['offset_mm'],
                          'facing_strong': _ef['strong'],
                          'local_face': _ef['local_face']},
                expected={'facing': edge})
        # Run-4 A: the missing PROXIMITY conjunct. The rule used to grade only
        # the overhang band and the nearest-edge identity, so a receptacle
        # 15.8 mm INTERIOR passed ("nearest west, declared west" is satisfied
        # anywhere on the board). For entries carrying the edge_receptacle
        # class (or an explicit max_setback_mm), no overhang AND off-seat is
        # a violation.
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
            # max_setback_mm (then the configured severity applies); an
            # `edge` adds the edge conjuncts but leaves this flag a WARN.
            from .part_class import INTERIOR_AFFINITY_MM
            setback = INTERIOR_AFFINITY_MM
            _sev = WARN
        if setback is not None and amount <= legality.EPS:
            # "No overhang" is the courtyard at ZERO margin (#1301): at the
            # gate's margin a receptacle 0.52 mm inside read as overhanging
            # at 0.55 and was never held to its seat, then failed it at 0.2.
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


def _facing_tail(f) -> str:
    """The CLI's `; facing ...` clause for one evidence row (#1155), or ''."""
    if not f or f.get('disposition') in (None, 'exempt'):
        return ''
    if f['disposition'] == 'abstained':
        return f"; facing abstained ({f.get('reason') or 'no face'})"
    if f['disposition'] == 'no_edge':
        return (f"; facing {f['facing']} (no edge declared: not graded)"
                if f.get('facing') else '')
    return (f"; facing {f['facing']} [{f['basis']}, "
            f"{'strong' if f['strong'] else 'weak'}] "
            f"{f['disposition'].upper() if f['disposition'] == 'fail' else 'pass'}")


def _band_amount(ctx, ref, edge, courtyard_amount):
    """`connector_geometry.band_amount` for a grade context: one geometry per
    context, `courtyard_amount` the courtyard's reading at ZERO margin
    (`connector_geometry.courtyard_overhang`, #1301)."""
    from .connector_geometry import band_amount, geometry_for
    return band_amount(geometry_for(ctx, ctx.pcb, ctx.pcb_file), ref, edge,
                       courtyard_amount, 0.0)


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
                          # A magnitude a ranking reader can use: without
                          # it a 52mm centring error would sort below every
                          # pad graze.
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
        # Compared at the EFFECTIVE basis: a plan row spelling out the
        # default `pad_edge` the brief left implicit is the same claim.
        if b.get('max_mm') != p.get('max_mm') or b.get('pads') != p.get(
                'pads') or (b.get('basis') or _PROXIMITY_DEFAULT_BASIS) != (
                p.get('basis') or _PROXIMITY_DEFAULT_BASIS):
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
    # election. `near` is byte-identical to `decap_tethers(radius)`.
    tethers, _beyond, _orphans = ctx.decap_populations(radius)
    sup = ctx.superseded()
    observed = (ctx.intent.basis or {}).get(
        'decaps.max_distance_mm') == 'observed_baseline'
    for ic in sorted(tethers):
        for cap, dist in tethers[ic]:
            if any(fnmatch.fnmatchcase(cap, pat) for pat in exempt):
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
    """{rail net id: [decap footprint]} -- the decaps (`groups.decap_population`,
    the one rule place_simple prices), each under the rail it decouples (a
    stacked cap under both of its two). A per-rail question, not the tether
    map: a cap serving two chips counts for both, and nothing is truncated at
    the search radius."""
    out: Dict[int, List] = {}
    fps = pcb_data.footprints or {}
    for ref, (rails, _owners) in sorted(groups_mod.decap_population(pcb_data).items()):
        for rail in rails:
            out.setdefault(rail, []).append(fps[ref])
    return out


def _caps_carrying_net(pcb_data) -> Dict[int, List]:
    """{net_id: [cap footprint]} -- every two-net cap (`groups.is_decoupling_cap`)
    carrying the net: `supply_pins`' evidence that a net NAMED like a rail is
    one (its rail-net channel). Not the decap population, which is built on
    `supply_pins` (`groups.decap_population`) and so cannot be its input."""
    out: Dict[int, List] = {}
    for ref, fp_ in sorted((pcb_data.footprints or {}).items()):
        if not groups_mod.is_decoupling_cap(fp_, ref):
            continue
        for p in fp_.pads:
            if p.net_id > 0:
                out.setdefault(p.net_id, []).append(fp_)
    return out


def _decap_supply_pins(pcb_data, pin_functions=None) -> Dict[str, Dict]:
    """`supply_pins` less the chips whose supply pins own no decap -- a
    connector (`grouping.owns_decaps`, the decap rule's one connector test,
    which the placer reads too)."""
    from .grouping import owns_decaps
    return {r: rec for r, rec in supply_pins(pcb_data, pin_functions=pin_functions).items()
            if owns_decaps(pcb_data, r)}


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


#: `supply_pins`' channels in precedence order (#705, #1249). `role` yields
#: only for a regulator among the supply owners (`groups.regulator_owners`),
#: so for every other chip the ladder is pintype -> pinfunction -> rail_net.
SUPPLY_PIN_CHANNELS = ('role', 'pintype', 'pinfunction', 'rail_net')


def supply_pins(pcb_data, *, pin_functions=None,
                chips=None) -> Dict[str, Dict]:
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

    `chips` replaces `groups.chip_refs` as the set of parts asked about. The
    array detector (#1051) passes a POSE-FREE chip set: `chip_refs`' row
    test reads global pad coordinates, which a non-right-angle rotation
    changes, and the detector must answer the same on any pose.

    A REGULATOR among the supply owners (#1249: `groups.regulator_owners`,
    a part that owns caps without being an IC by shape and that a
    `power_out` pad or the role table calls a regulator) is read by its
    ROLE first: channel 0, `role`, takes every non-ground pin of it. A
    regulator's VIN and VOUT are supply pins because of what the part is,
    not because of a pintype or a rail-like net name: esp_prog U2 carries
    no pintype, and its VIN net is `Net-(C1-Pad1)`, which channel 3 never
    reads as a rail -- so without this channel C1 is graded against
    nothing, and complex_hierarchy U101's VI pin (typed `input`) would lose
    to channel 1's VO. An owner the three-pad shape alone admits (kit-dev
    LV101, a reset supervisor) is not a regulator by any evidence, so it
    keeps the ladder: its +3.3V pin, not its RST. The channel is empty for
    every IC by shape, so their ladder is the one above, unchanged.
    """
    from net_queries import is_supply_pintype, is_supply_pinfunction, \
        is_power_net_name
    if chips is None:
        chips = groups_mod.chip_refs(pcb_data)
    by_net = _caps_carrying_net(pcb_data)
    owners = groups_mod.regulator_owners(pcb_data)
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
            'role': list(cands) if ref in owners else [],
            'pintype': [(p, n) for p, n in cands
                        if is_supply_pintype(getattr(p, 'pintype', '') or '')],
            'pinfunction': [
                (p, n) for p, n in cands
                if is_supply_pinfunction(getattr(p, 'pinfunction', '') or '',
                                         pin_functions)],
            'rail_net': [(p, n) for p, n in cands
                         if is_power_net_name(n) and by_net.get(p.net_id)],
        }
        order = SUPPLY_PIN_CHANNELS
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
    wrong.

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
    # #1142, PER CAP: a cap a --decaps-from reference keeps within the radius
    # (`within_radius_refs`) is HELD to it, so leaving it beyond is an ERROR;
    # a cap the reference itself keeps beyond -- likelier a bulk or filter cap
    # than a failed decoupler, the reasoning above -- stays a WARN. An explicit
    # `severity.decap_ungraded` still wins in both directions (an author can
    # turn the rule down, and an intent #1102 promoted board-wide grades as it
    # always did).
    explicit = 'decap_ungraded' in (ctx.intent.severity or {})
    held = frozenset(spec.get('within_radius_refs') or ())
    for cap, ic, dist in beyond:
        # An author who waived a cap from the distance claim has already
        # decided about it; telling them it is also ungraded is noise about
        # their own decision. A cap a declared relation supersedes IS
        # graded -- by that relation.
        if any(fnmatch.fnmatchcase(cap, pat) for pat in exempt) or cap in sup:
            continue
        is_held = cap in held
        # `sev` stays the answer for every cap the list does not raise, so
        # the rule's WARN default is still the one line that decides it.
        cap_sev = ERROR if (is_held and not explicit) else sev
        yield Violation(
            rule='decap_ungraded', severity=cap_sev,
            ref=cap, block=ctx.owner.get(cap),
            message=(f"{cap} is {dist:.2f}mm from {ic}, the IC it decouples "
                     f"-- beyond the {radius:.2f}mm tether search radius, so "
                     f"decap_distance never measured it against the "
                     f"{limit:.2f}mm limit. "
                     + ("The reference board keeps it WITHIN that radius, so "
                        "here it is a stranded decoupler, not a bulk cap"
                        if is_held else
                        "That limit was derived from the caps INSIDE the "
                        "radius, so it says nothing about this one")),
            measured={'distance_mm': round(dist, 4), 'ic': ic,
                      'search_radius_mm': round(radius, 4),
                      'held_by_reference': is_held},
            expected={'max_distance_mm': limit})


def _pin_gap(pin_pad, cap_fp, net_id: int, pin_rect=None) -> Optional[float]:
    """Smallest pad-edge gap from a supply pin to a cap's pad ON THAT NET.

    The cap's RAIL leg, never its nearest pad: a 0402's ground pad can sit half
    a millimetre nearer and shave the number, and the rail leg is what the rule
    is about.

    SIGNED (`legality.rect_gap` goes negative on overlap), not clamped. Clamping
    erases the overlap magnitude, and 58 of glasgow_revC's 87 tethers already
    clamp to zero under the existing centroid-to-inflated-bbox metric -- #704's
    median argument is that scar.

    `pin_rect`: `legality.pad_rect(pin_pad)`, for a caller measuring one pin
    against many caps.
    """
    best = None
    pr = pin_rect if pin_rect is not None else legality.pad_rect(pin_pad)
    for q in cap_fp.pads:
        if q.net_id != net_id:
            continue
        g = legality.rect_gap(pr, legality.pad_rect(q))
        if best is None or g < best:
            best = g
    return best


def decap_pin_caps(spec: Dict, ref: str, ic_side, pad, by_net
                   ) -> Tuple[List, List]:
    """`(on_rail, caps)` for one supply pin of `ref`: every decoupling cap on
    the pin's net, and the ones the intent's `exempt` / `same_side` leave
    usable. Lifted out of `rule_decap_pin_distance` (#1043) so the quench's
    tether gate elects the SAME cap set the rule grades; `by_net` is
    `_decap_caps_by_net(pcb)`."""
    exempt = tuple(spec.get('exempt') or ())
    on_rail = [c for c in by_net.get(pad.net_id, ())
               if c.reference != ref]
    caps = [c for c in on_rail
            if not any(fnmatch.fnmatchcase(c.reference, pat) for pat in exempt)]
    if spec.get('same_side'):
        # A MANUFACTURING claim, never an electrical one: the author is
        # asserting the back side is not available -- single-sided assembly, a
        # can or heatsink over it, an enclosure wall. See the docs for what it
        # costs.
        caps = [c for c in caps
                if legality.footprint_side(c) == ic_side
                or legality.footprint_has_through_pads(c)]
    return on_rail, caps


def nearest_rail_cap(pin_pad, caps, stop_at=None) -> Optional[Tuple[float, str]]:
    """`(gap, cap ref)` -- the minimum `_pin_gap` from a supply pin over
    `caps`, or None when none carries its net. The number
    `rule_decap_pin_distance` grades, and the one the quench gate reads
    (#1043), with the caps at whatever poses their footprints hold.

    `stop_at`: the first cap within it is returned instead of the nearest --
    within `stop_at` exactly when the minimum is, and the minimum itself
    otherwise -- for a caller asking only "is some cap within the limit"."""
    pr = legality.pad_rect(pin_pad)
    best = None
    for c in caps:
        g = _pin_gap(pin_pad, c, pin_pad.net_id, pr)
        if g is None:
            continue
        got = (g, c.reference)
        if best is None or got < best:
            best = got
        if stop_at is not None and g <= stop_at:
            return got
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
    if source == 'fab_marking':
        # #1201: no silk, a rejected .Fab marking -- the rect is the pads.
        source = 'pad_bbox'
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


def proximity_pads(claim: Dict, a_fp, b_fp) -> Tuple[List, List, bool]:
    """`(subject pads, partner pads, declared)` for one `proximity[]` claim
    over the two footprints at whatever poses they hold. Lifted out of
    `rule_proximity` (#1043) so the quench's tether gate measures the pad
    sets the rule grades."""
    spec = claim.get('pads') or {}
    ref, near = str(claim['ref']), str(claim['near'])
    declared = bool(spec.get(ref))
    # Undeclared: every pad but the paste/mask apertures (#1143). Read as
    # pads, tigard J1's paste windows put C25 2.11 mm from J1, where its
    # copper is 2.61 mm away, so a 2.5 mm claim passed (Phase-1 verifier).
    subject = (_pads_named(a_fp, spec[ref]) if declared
               else non_aperture_pads(a_fp))
    partners = (_pads_named(b_fp, spec[near]) if spec.get(near)
                else non_aperture_pads(b_fp))
    return subject, partners, declared


def proximity_reaches(subject, partners, declared: bool) -> List[Tuple]:
    """`[(gap, subject pad, partner pad, how)]` -- one per declared subject
    pad, or ONE for the pair (the minimum) when no pads were declared: the
    two arities `rule_proximity` grades. Shared with the quench gate (#1043)."""
    reaches = []
    for pad in subject:
        got = _proximity_reach(pad, partners, net_match=declared)
        if got is not None:
            reaches.append((got[0], pad, got[1], got[2]))
    if not declared and reaches:
        reaches = [min(reaches, key=lambda t: t[0])]
    return reaches


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
        return ("the board carries no decoupling capacitor (no cap from "
                "ground to a rail some chip other than a connector has a "
                "supply pin on), so no supply pin can be graded against one")
    recs = ctx.supply_pins()
    if not any(r['pins'] for r in recs.values()):
        tot = {c: sum(r['counts'][c] for r in recs.values())
               for c in SUPPLY_PIN_CHANNELS}
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
    question, and the answer is forced: every decap (`groups.decap_population`) on the pin's
    exact net, either side, any distance, whoever else it also serves.

    THREE NAMES, because severity is settable per NAME and these are three
    different claims:

      decap_pin_distance           error  a DECLARED supply pin (channel 1-2,
                                          or a regulator owner's role,
                                          channel 0, #1249) is further than
                                          the limit
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
    sev_inf = ctx.intent.severity_of('decap_pin_distance_inferred',
                                     default=WARN)
    sev_unc = ctx.intent.severity_of('decap_pin_uncovered', default=WARN)
    by_net = _decap_caps_by_net(ctx.pcb)
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
            on_rail, caps = decap_pin_caps(spec, ref, ic_side, pad, by_net)
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
            got = nearest_rail_cap(pad, caps)
            if got is None:
                continue
            gap, who = got
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
        matched = [r for r in refs if fnmatch.fnmatchcase(r, pattern)]
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


#: How many overlap pairs `rule_legality` writes into `measured`, worst
#: first; `pairs_total` carries the full count.
MEASURED_PAIRS_CAP = 50


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
    # Mechanical fixed poses at their datum join them (`oob_exempt`).
    # `oob_amount` is reduced by the same parts' census amounts, the raw
    # total published as `oob_amount_raw`: each exempt overhang is already
    # bounded by its own band or datum, and left in, the six edge
    # connectors of one run made `oob_amount: 0` unsatisfiable.
    exempt = ctx.oob_exempt()
    ctx.legality['oob_count_exempt'] = len(exempt)
    census = {r: ctx._oob_exempt_census[r] for r in exempt}  # noqa: SLF001
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
        if key == 'overlap_area':
            # #1162: graded on check_assembly's drawn outlines, not the
            # search's rects. glasgow g4 read 4.909 mm2 here -- 3.93 of it
            # MK1-4's r=5 circle courtyards graded as 10 x 10 squares -- while
            # check_assembly said buildable; and the error named no pair, so
            # a caller rebuilt them from a different universe and threw a
            # legal layout away. Both readings and every pair are measured.
            ex, rc, pairs = _ctx_overlap_exact(ctx)
            ctx.legality['overlap_area_exact'] = round(ex, 4)
            got = ex
            # Worst first, capped: a pile lists thousands, and the JSON
            # doubled (phase-5 verifier). `pairs_total` keeps the count.
            # `excluded` names the parts the budget does not see -- a silk-
            # only part (#896) is invisible to it, and says so.
            _gp, _excl = legality.courtyard_budget_universe(
                _ctx_census(ctx), list(ctx.parts))
            ctx.legality['overlap_area_excluded'] = sum(
                len(v) for v in _excl.values())
            measured = {key: round(float(ex), 4),
                        'overlap_area_rect': round(float(rc), 4),
                        'pairs': pairs[:MEASURED_PAIRS_CAP],
                        'pairs_total': len(pairs),
                        'excluded': {k: v for k, v in _excl.items() if v}}
            if pairs:
                note = (' -- worst: ' + ', '.join(
                    f"{a}/{b} {e:.3f} (rect {r:.3f})"
                    for a, b, r, e in pairs[:5])
                        + (f", +{len(pairs) - 5} more" if len(pairs) > 5
                           else ''))
        if key == 'oob_count' and exempt:
            raw = got
            got = raw - len(exempt)
            measured = {key: int(got), 'oob_count_raw': int(raw),
                        'exempt': sorted(exempt)}
            note = (f" ({len(exempt)} declared edge connector(s) within "
                    f"their overhang band or mechanical datum(s) at their "
                    f"pose exempt: {', '.join(sorted(exempt))})")
        if key == 'oob_amount' and census:
            raw = float(got)
            got = max(0.0, raw - math.fsum(census.values()))
            measured = {key: round(got, 4), 'oob_amount_raw': round(raw, 4),
                        'exempt': sorted(census)}
            note = f" ({len(census)} exempt part(s) excluded)"
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
    bridges exactly two nets), so an inductor or an ESD array is never its
    subject, and it measures to the nearest owner's pad BOX, not a named pin.
    It was written for a sharper failure: a cap's partner had to carry >= 4
    copper pads, so a 3-pad SOT89 regulator was never a tether target, and on
    the board this was written for the election gave `C1 -> USB1 2.03mm` and
    `C3 -> U1 1.83mm` -- the regulator's own bulk caps, graded against a USB
    socket and a bridge IC. Since #1249 a supply owner
    (`groups.supply_owners`) is elected, so both caps now tether to U2 at
    0.95mm; a declared row still says WHICH pin and how far.

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
        sev = WARN if derived_claim(claim) else ctx.sev('proximity')

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
                rule='proximity', severity=sev,
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

        subject, partners, declared = proximity_pads(claim, a_fp, b_fp)
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
        # No `if not reaches` arm: `partners` is proven non-empty by the guard
        # above and `_proximity_reach` falls back to it, so it never returns
        # None here. An arm that cannot run is not a safety net -- it is a
        # claim about the code that no test can check, and this one survived
        # being replaced by `raise AssertionError`.
        for gap, pad, partner, how in proximity_reaches(subject, partners,
                                                         declared):
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
                rule='proximity', severity=sev,
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


#: #1253. What `keep_away` cannot see, stated with every reading of it.
KEEP_AWAY_NOT_MEASURED = (
    'tracks: route.py --keep-away prices and reports those once routed',
    'coupling through planes or between faces: the distance is plan view, '
    "and each pad's side is named so a reader can judge",
    'which nets are noisy or sensitive: the rule declares that, nothing on '
    'the board says it',
)


#: Aggressor pads measured against every victim pad at once.
_KEEP_AWAY_CHUNK = 256


def _keep_away_face(pad) -> Optional[Tuple[str, ...]]:
    """The copper layers a pad's copper is on -- `('*',)` for a pad through
    every layer -- or None for a pad with no copper (an NPTH hole, an
    aperture), which no keep-away is about."""
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return None
    layers = [str(L) for L in (pad.layers or ())]
    if '*.Cu' in layers or getattr(pad, 'pad_type', '') == 'thru_hole':
        return ('*',)
    cu = tuple(sorted(L for L in layers if L.endswith('.Cu')))
    return cu or None


def _keep_away_side(face) -> str:
    return 'through-hole' if face == ('*',) else '+'.join(face)


def _keep_away_cross(fa, fb) -> bool:
    """The two pads share no copper layer: plan view is all this measures."""
    return '*' not in fa and '*' not in fb and not (set(fa) & set(fb))


def _keep_away_gaps(a, b):
    """`legality.rect_gap` of every row of `a` against every row of `b`
    ((n, 4) and (m, 4) rects) as an (n, m) array -- the same formula, so the
    pair it picks is the pair `rect_gap` picks."""
    import numpy as np
    dx = np.maximum(a[:, None, 0] - b[None, :, 2], b[None, :, 0] - a[:, None, 2])
    dy = np.maximum(a[:, None, 1] - b[None, :, 3], b[None, :, 1] - a[:, None, 3])
    both_neg = (dx < 0) & (dy < 0)
    both_pos = (dx > 0) & (dy > 0)
    return np.where(both_neg, np.maximum(dx, dy),
                    np.where(both_pos, np.hypot(dx, dy), np.maximum(dx, dy)))


def rule_keep_away(ctx) -> Iterator[Violation]:
    """Two declared net groups no closer than GAP mm, pad edge to pad edge
    (#1253) -- the placement half of route.py's `--keep-away`.

    The routing keep-away (#1146) measures TRACKS: a net with none is
    skipped, so on a placed, unrouted board it checks nothing, and by the
    time it can, the parts are fixed. This measures the PADS the placement
    put down, in `proximity`'s currency (`legality.pad_rect` + `rect_gap`),
    from the same declaration -- a rule in the router's syntax, its sides
    resolved by the router's own resolver (net patterns as `--nets`,
    `class=` terms through the board's project).

    Per rule, every aggressor pad against every victim pad, except two kinds
    of pair placement cannot change: two pads of ONE net (a net on both
    sides), and two pads of ONE part, whose footprint fixes their gap (the
    router's free radius exempts the same pins). One finding per PART PAIR
    inside the band, at that pair's closest pads, naming both pads' sides.
    WARN by default; ERROR only when the intent's `severity` map says so.
    Pads are read off `ctx.pcb.footprints`, never `pads_by_net`, so a
    `PoseGrader` view's moved parts would be measured where they stand.
    Every rule is recorded on `ctx.keep_away_measured`, passing ones too,
    with what it cannot see (`KEEP_AWAY_NOT_MEASURED`).

    TOTAL OVER ITS RULES, like `proximity`: each yields a measured pass, a
    `keep_away` finding, a `keep_away_unresolved` finding (a side matching
    no net), or an abstention (nothing between two parts to measure).
    """
    import numpy as np
    from keep_away import parse_keep_away_rules, resolve_rule_sides
    sev = ctx.intent.severity_of('keep_away', default=WARN)
    sev_unres = ctx.intent.severity_of('keep_away_unresolved', default=WARN)
    owned = []                     # (ref, pad, face) over the board AS POSED
    for ref, fp_obj in sorted((ctx.pcb.footprints or {}).items()):
        for pad in fp_obj.pads or ():
            if pad.net_id and pad.net_id > 0:
                face = _keep_away_face(pad)
                if face is not None:
                    owned.append((ref, pad, face))
    names = {nid: n.name for nid, n in (ctx.pcb.nets or {}).items()}
    for i, entry in enumerate(ctx.intent.keep_away):
        rule = parse_keep_away_rules(entry['rule'])[0]
        spec, gap_mm = rule.spec, float(rule.gap)
        akey = f"keep_away[{i}]"
        rec: Dict[str, object] = {
            'index': i, 'rule': spec, 'gap_mm': gap_mm,
            'source': entry.get('source'),
            'not_measured': list(KEEP_AWAY_NOT_MEASURED)}
        ctx.keep_away_measured.append(rec)
        groups, notes, _unmatched = resolve_rule_sides(ctx.pcb, (rule,))
        agg, vic, _gap = groups[0]
        rec.update({'aggressor_nets': len(agg), 'victim_nets': len(vic)})
        if notes:
            # A pattern that matched nothing, kept even when its side still
            # resolved: the side is then smaller than the rule says.
            rec['notes'] = list(notes)
        if not agg or not vic:
            empty = [side for side, ids in (('aggressor', agg),
                                            ('victim', vic)) if not ids]
            rec.update({'state': 'unresolved', 'passes': None})
            yield Violation(
                rule='keep_away_unresolved', severity=sev_unres,
                message=(f"{akey} '{spec}': the {' and '.join(empty)} "
                         f"side{'s match' if len(empty) > 1 else ' matches'} "
                         f"no net on this board, so nothing is measured"
                         + (f" ({'; '.join(notes)})" if notes else '')),
                measured={'rule': spec, 'index': i, 'empty_sides': empty},
                expected={'gap_mm': gap_mm})
            continue
        A = [t for t in owned if t[1].net_id in agg]
        V = [t for t in owned if t[1].net_id in vic]
        rec.update({'aggressor_pads': len(A), 'victim_pads': len(V)})
        if not A or not V:
            rec.update({'state': 'abstained', 'passes': None})
            ctx.abstain(akey, f"the {'aggressor' if not A else 'victim'} "
                              f"nets of '{spec}' have no pad copper, so there "
                              f"is nothing to measure pad edge to pad edge -- "
                              f"route.py --keep-away still prices their tracks")
            continue
        ra = np.array([legality.pad_rect(p) for _r, p, _f in A], dtype=float)
        rv = np.array([legality.pad_rect(p) for _r, p, _f in V], dtype=float)
        a_net = np.array([p.net_id for _r, p, _f in A])
        v_net = np.array([p.net_id for _r, p, _f in V])
        a_ref = np.array([r for r, _p, _f in A], dtype=object)
        v_ref = np.array([r for r, _p, _f in V], dtype=object)
        measured = same_part = 0
        best = None                            # (gap, ai, vi)
        inside: Dict[Tuple[str, str], Tuple[float, int, int]] = {}
        for lo in range(0, len(A), _KEEP_AWAY_CHUNK):
            hi = min(len(A), lo + _KEEP_AWAY_CHUNK)
            g = _keep_away_gaps(ra[lo:hi], rv)
            same_net = a_net[lo:hi, None] == v_net[None, :]
            one_part = a_ref[lo:hi, None] == v_ref[None, :]
            same_part += int((one_part & ~same_net).sum())
            ok = ~(same_net | one_part)
            measured += int(ok.sum())
            if not ok.any():
                continue
            g = np.where(ok, g, np.inf)
            k = int(np.argmin(g))
            if best is None or float(g.flat[k]) < best[0]:
                best = (float(g.flat[k]), lo + k // g.shape[1],
                        k % g.shape[1])
            for ia, iv in zip(*np.nonzero(g < gap_mm - legality.EPS)):
                ai, vi = lo + int(ia), int(iv)
                key = (A[ai][0], V[vi][0])
                val = float(g[ia, iv])
                if key not in inside or val < inside[key][0]:
                    inside[key] = (val, ai, vi)
        rec.update({'pairs_measured': measured, 'same_part_pairs': same_part})
        if best is None:
            rec.update({'state': 'abstained', 'passes': None})
            ctx.abstain(akey, f"every pad pair of '{spec}' lies on one part, "
                              f"whose footprint fixes the gap -- there is "
                              f"nothing placement can move")
            continue

        def _end(t):
            ref, pad, face = t
            return {'ref': ref, 'pad': pad.pad_number,
                    'net': names.get(pad.net_id) or None,
                    'side': _keep_away_side(face)}

        def _gap(ai, vi):
            # The reported number is `rect_gap`'s own, on the chosen pair.
            return legality.rect_gap(legality.pad_rect(A[ai][1]),
                                     legality.pad_rect(V[vi][1]))
        _g0, ai, vi = best
        rec.update({
            'state': 'inside' if inside else 'pass',
            'passes': not inside,
            'min_gap_mm': round(_gap(ai, vi), 4),
            'closest': {'victim': _end(V[vi]), 'aggressor': _end(A[ai]),
                        'cross_face': _keep_away_cross(V[vi][2], A[ai][2])},
            'part_pairs_inside': len(inside)})
        for (aref, vref), (_v, ai, vi) in sorted(
                inside.items(), key=lambda kv: (kv[1][0], kv[0])):
            gv = _gap(ai, vi)
            va, aa = _end(V[vi]), _end(A[ai])
            cross = _keep_away_cross(V[vi][2], A[ai][2])
            yield Violation(
                rule='keep_away', severity=sev, ref=vref,
                block=ctx.owner.get(vref),
                message=(f"{vref} pad {va['pad']} ({va['side']}) on "
                         f"{va['net']} is {gv:.2f}mm from {aref} pad "
                         f"{aa['pad']} ({aa['side']}) on {aa['net']}, inside "
                         f"the {gap_mm:.2f}mm keep-away"
                         + ("; the pads are on opposite faces, so this is "
                            "the plan-view distance and the board between "
                            "them is not measured" if cross else '')
                         + f" ({akey} '{spec}')"),
                measured={'gap_mm': round(gv, 4), 'rule': spec, 'index': i,
                          'victim': va, 'aggressor': aa,
                          'cross_face': cross, 'basis': 'pad_edge'},
                expected={'gap_mm': gap_mm})


def rule_replicate(ctx) -> Iterator[Violation]:
    """Each declared copy target laid out as a translated copy of its
    source instance (#1211) -- how far it is from that, never how it got
    there.

    Per declaration, the parts are matched exactly as `place_pose
    replicate` matches them (`replicate.measure_declared`, with the
    declaration's waivers and `only` / `skip`); a declaration the verb would
    refuse -- an unknown sheet, instances not wired alike -- is a
    `replicate_unresolved` finding carrying the verb's own reason. Per
    target, the parts' offsets are fitted by one translation (their median)
    and each part's distance from it is its residual. One finding per target
    whose worst residual exceeds `tolerance_mm`
    (`REPLICATE_TOLERANCE_MM`), or with a part rotated or on the other side
    from its source. WARN by default (both names); the `severity` map may
    promote either. Every target is recorded on `ctx.replicate_measured`,
    passing ones too. Grade only: `place_pose replicate --intent` is what ACTS
    on the declaration."""
    from .pose_ops import PoseRefusal
    from .replicate import measure_declared
    sev = ctx.intent.severity_of('replicate', default=WARN)
    sev_unres = ctx.intent.severity_of('replicate_unresolved', default=WARN)
    for i, decl in enumerate(ctx.intent.replicate):
        key = f"replicate[{i}]"
        head = f"{key} {decl['from']} -> {', '.join(decl['to'])}"
        try:
            rows = measure_declared(ctx.pcb, decl)
        except PoseRefusal as exc:
            ctx.replicate_measured.append({
                'index': i, 'from': decl['from'], 'to': list(decl['to']),
                'state': 'unresolved', 'passes': None, 'reason': exc.reason})
            yield Violation(
                rule='replicate_unresolved', severity=sev_unres,
                message=f"{head}: not measured -- {exc.reason}",
                measured={'index': i, 'code': exc.code},
                expected={'from': decl['from'], 'to': list(decl['to'])})
            continue
        tol = float(decl.get('tolerance_mm', REPLICATE_TOLERANCE_MM))
        for row in rows:
            off = [p for p in row['parts']
                   if p['residual_mm'] > tol + legality.EPS
                   or p['rot_differs'] or p['side_differs']]
            row.update(index=i, tolerance_mm=tol, passes=not off,
                       off=len(off), state='measured')
            ctx.replicate_measured.append(row)
            if not off:
                continue
            worst = off[0]
            yield Violation(
                rule='replicate', severity=sev, ref=worst['ref'],
                block=ctx.owner.get(worst['ref']),
                message=(
                    f"{key}: {row['to']} has {len(off)} of {row['pairs']} "
                    f"part(s) off {row['from']}'s arrangement -- up to "
                    f"{row['max_residual_mm']:.2f}mm (rms "
                    f"{row['rms_residual_mm']:.2f}) from the best "
                    f"translation ({row['fit_offset'][0]:+.2f}, "
                    f"{row['fit_offset'][1]:+.2f})"
                    + (f"; {row['rotated']} rotated differently"
                       if row['rotated'] else '')
                    + (f"; {row['flipped']} on the other side"
                       if row['flipped'] else '')
                    + f"; worst {worst['ref']} (as {worst['source']}, "
                      f"{worst['residual_mm']:.2f}mm). `place_pose "
                      f"replicate --intent` copies it"),
                measured={'index': i, 'to': row['to'], 'off': len(off),
                          'pairs': row['pairs'],
                          'max_residual_mm': row['max_residual_mm'],
                          'rotated': row['rotated'],
                          'flipped': row['flipped'],
                          'worst': [p['ref'] for p in off[:NAMED_WORST]]},
                expected={'tolerance_mm': tol})


#: How many off-arrangement parts a `replicate` finding names.
NAMED_WORST = 6


def rule_template(ctx) -> Iterator[Violation]:
    """Each declared datasheet template's members at the poses it gives
    them in the anchor's frame (`placement.templates.measure`): one finding
    per template whose worst member pad sits more than `tolerance_mm` from
    its template position, or with a member on the other face. A template
    naming a ref the board does not have is `template_unresolved`. A row
    whose `kind` is `switching stage` also has each member cap's ground
    return measured (`templates.ground_returns`): one
    `template_ground_return` finding per template with a cap past its
    `ground_return_mm` (`GROUND_RETURN_MM` by default) -- a template can
    hold its members exactly where it was fitted and still be wrong, as an
    output cap given only its VOUT partner is seated past the inductor, its
    ground far from the chip's. WARN by default (all three names); the
    `severity` map may promote any. Every template is recorded on
    `ctx.template_measured`, passing ones too."""
    from .templates import GROUND_RETURN_MM, ground_returns, measure
    sev = ctx.intent.severity_of('template', default=WARN)
    sev_unres = ctx.intent.severity_of('template_unresolved', default=WARN)
    sev_ground = ctx.intent.severity_of('template_ground_return', default=WARN)
    for i, decl in enumerate(ctx.intent.templates):
        key = f"templates[{i}] ({decl['anchor']})"
        row = dict(measure(ctx.pcb, decl), index=i)
        if row['state'] == 'unresolved':
            row['passes'] = None
            ctx.template_measured.append(row)
            yield Violation(rule='template_unresolved', severity=sev_unres,
                            message=f"{key}: not measured -- {row['reason']}",
                            measured={'index': i}, expected={'anchor': decl['anchor']})
            continue
        tol = decl['tolerance_mm']
        off = [m for m in row['members']
               if m['worst_mm'] > tol + legality.EPS or m['side_differs']]
        row.update(passes=not off, off=len(off))
        ctx.template_measured.append(row)
        if decl.get('kind') == 'switching stage':
            limit = float(decl.get('ground_return_mm') or GROUND_RETURN_MM)
            caps = ground_returns(ctx.pcb, decl)
            far = sorted((c for c in caps if c['return_mm'] > limit + legality.EPS),
                         key=lambda c: -c['return_mm'])
            row['ground_return'] = {'limit_mm': limit, 'caps': caps, 'over': len(far)}
            if far:
                w = far[0]
                yield Violation(
                    rule='template_ground_return', severity=sev_ground, ref=w['ref'],
                    block=ctx.owner.get(w['ref']),
                    message=(f"{key}: {len(far)} of {len(caps)} cap(s) return their ground "
                             f"more than {limit:g}mm from the chip's -- worst {w['ref']}, "
                             f"{w['return_mm']:.2f}mm of pad-edge gaps from its {w['net']} pad "
                             f"to {w['to']}"
                             + (f" (via {', '.join(w['via'])})" if w['via'] else '')),
                    measured={'index': i, 'over': len(far), 'max_mm': w['return_mm'],
                              'worst': [c['ref'] for c in far[:NAMED_WORST]]},
                    expected={'ground_return_mm': limit})
        if not off:
            continue
        worst = off[0]
        yield Violation(
            rule='template', severity=sev, ref=worst['ref'],
            block=ctx.owner.get(worst['ref']),
            message=(f"{key}: {len(off)} of {len(row['members'])} member(s) off the "
                     f"template -- worst {worst['ref']}, a pad "
                     f"{worst['worst_mm']:.2f}mm from its template position"
                     + (" (on the other face)" if worst['side_differs'] else '')),
            measured={'index': i, 'off': len(off), 'max_mm': worst['worst_mm'],
                      'worst': [m['ref'] for m in off[:NAMED_WORST]]},
            expected={'tolerance_mm': tol})


def rule_pins_to_edge(ctx) -> Iterator[Violation]:
    """A part whose pad row faces the board outline with nothing beyond it.

    The facing criterion of the boundary review, as a number (run 26: a SOT-89
    regulator seated with all three pins 0.40 mm from the north edge, every
    net forced under its own body, and a review that wrote PASS because
    nothing measured it). The geometry is `placement.edge_facing`, so every
    reader of it agrees with the rule about a pad.

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


#: `rule_pins_to_edge` grades parts with at least this many CONNECTED pads,
#: read from the geometry core.
from .edge_facing import MIN_PADS as PINS_TO_EDGE_MIN_PADS  # noqa: E402


def rule_array_formation(ctx) -> Iterator[Violation]:
    """Each declared array (#1051) is ONE formed row: a common axis, the
    expected order along it, the declared rotation, an even (or declared)
    pitch.

    The verdict is `arrays.formation`'s -- the one predicate any row seat is
    to call too -- over each member's
    courtyard centre (the grader's own `rect`, so identical parts compare
    like for like) and its board rotation. The expected order is
    `arrays.expected_order`: the served part's PIN order for `order: "pin"`,
    read off pads and nets and never off poses.

    An array with a member the board does not have is left to
    `array_problems`, which reports it at ERROR; grading the survivors would
    call a row formed that is missing a part. A member with no geometry
    abstains, by name, rather than passing.
    """
    from . import arrays as arr
    fps = ctx.pcb.footprints or {}
    for a in ctx.intent.arrays:
        spec = arr.spec_of(a)
        name = spec['name']
        absent = [m for m in spec['members'] if m not in fps]
        if absent:
            # `array_problems` reports each at ERROR; grading the survivors
            # would call a row formed that is missing a part.
            ctx.array_measured.append({
                'name': name, 'formed': None,
                'skipped': f"not on this board: {', '.join(absent)}"})
            continue
        missing = [m for m in spec['members'] if m not in ctx.parts]
        if missing:
            why = (f"no placement geometry for {', '.join(missing)}, so the "
                   f"row cannot be measured")
            ctx.abstained[f"arrays[{name}]"] = why
            ctx.array_measured.append({'name': name, 'formed': None,
                                       'skipped': why})
            continue
        poses = []
        for m in spec['members']:
            r = ctx.parts[m].rect
            poses.append({'ref': m, 'x': (r[0] + r[2]) / 2.0,
                          'y': (r[1] + r[3]) / 2.0,
                          'rot': float(fps[m].rotation or 0.0) % 360.0,
                          'pads': arr._copper_pad_count(fps[m])})
        order_refs, unresolved = arr.expected_order(ctx.pcb, spec)
        v = arr.formation(poses, order_key=order_refs,
                          rotation_spec=spec['rotation'],
                          pitch_spec=spec['pitch_mm'],
                          axis_spec=spec['axis'])
        # Disclosed on EVERY grade, formed or not: which members the order
        # could place, and which it could not.
        ctx.array_measured.append({
            'name': name, 'formed': v['formed'], 'failed': list(v['failed']),
            'unchecked': list(v['unchecked']), 'axis': v['axis'],
            'order': list(v['order']), 'order_expected': order_refs,
            'order_unresolved': dict(unresolved)})
        if order_refs is not None and 'order' in v['unchecked']:
            # A DECLARED order that fewer than two members resolve to is not
            # a pass: the verdict is incomplete, and says which refs.
            ctx.abstained[f"arrays[{name}].order"] = (
                f"order {spec['order']!r} places only "
                f"{len((v['checks'].get('order') or {}).get('expected') or [])}"
                f" member(s); unresolved: "
                + (', '.join(f"{k} ({w})" for k, w in sorted(
                    unresolved.items())) or 'none'))
        if v['formed']:
            continue
        detail = []
        for check in v['failed']:
            c = v['checks'].get(check) or {}
            if check == 'axis':
                detail.append(f"off the {c.get('axis')} axis by up to "
                              f"{c.get('max_offset_mm')}mm")
            elif check == 'order':
                detail.append(f"order {c.get('observed')} is not "
                              f"{c.get('expected')} (either direction)")
            elif check == 'rotation':
                detail.append(f"rotations {c.get('rotations')}"
                              + (f", {', '.join(c['off'])} off "
                                 f"{c.get('declared'):g}"
                                 if c.get('off') else ''))
            elif check == 'pitch':
                gaps = c.get('gaps_mm') or []
                if c.get('stacked'):
                    # Stacked members, AND the gaps between the rest: the
                    # stack is one defect, uneven spacing may be another.
                    rest = [g for g in gaps
                            if g > arr.DEFAULT_TOLERANCES['pitch_spread_mm']]
                    detail.append(f"members stacked: "
                                  f"{', '.join(c['stacked'])} share one spot "
                                  f"along the row"
                                  + (f"; gaps between the rest {rest}mm"
                                     if rest else ''))
                else:
                    detail.append(f"gaps {gaps}mm")
        yield Violation(
            rule='array_formation', severity=ctx.sev('array_formation'),
            block=name,
            message=(f"array {name!r} is not a formed row: "
                     + '; '.join(detail)),
            measured={'failed': list(v['failed']),
                      'unchecked': list(v['unchecked']),
                      'axis': v['axis'], 'order': list(v['order']),
                      'checks': v['checks'],
                      'order_unresolved': dict(unresolved)},
            expected={'order': order_refs, 'rotation': spec['rotation'],
                      'pitch_mm': spec['pitch_mm'], 'axis': spec['axis']})


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
    ('keep_away', rule_keep_away),
    ('must_lock', rule_must_lock),
    ('legality', rule_legality),
    ('pins_to_edge', rule_pins_to_edge),
    ('array_formation', rule_array_formation),
    ('replicate', rule_replicate),
    ('template', rule_template),
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
    # #1253, beside `keep_away` for proximity's reason: a side that matches
    # no net on THIS board is a different claim from a distance. WARN by
    # default, like the rule it stands in for.
    'keep_away_unresolved',
    # #1211, beside `replicate`: a declaration the board cannot resolve (an
    # unknown sheet, instances not wired alike) is a different claim from a
    # distance. WARN by default, like the rule it stands in for.
    'replicate_unresolved',
    # Beside `template`: a template naming a ref the board does not have is
    # a different claim from a drift. WARN by default, like the rule.
    'template_unresolved',
    # Beside `template` too: a switching stage's cap returning its ground
    # far from the chip's is a claim about the template itself, which a
    # member exactly at its pose can still break. WARN by default.
    'template_ground_return',
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
    # Beside `edge_connector` too, by `grade(edge_seats=True)` and
    # `plan_check`: an edge whose declared parts cannot all seat together
    # (`edge_seat.oversubscribed`, the records place_simple makes). WARN by
    # default, like `plan_edge_crowded`; an intent may promote it.
    'edge_oversubscribed',
    # #959 (#998). Raised by `plan_check`, the zone plan checked against
    # itself and the board BEFORE the first pose write, and (the first) by
    # `grade` too. Every ERROR among them is sound: the plan cannot be seeded
    # or satisfied however the parts are arranged. The `*_crowded` ones are
    # the WARN halves -- the same quantities with a margin added.
    'plan_zone_exclusive_unsatisfiable', 'block_glob_literal',
    'plan_fixed_outside_zone', 'plan_zone_overfull', 'plan_zone_crowded',
    'plan_edge_overfull', 'plan_edge_crowded', 'plan_board_overfull',
    'plan_board_crowded',
    # #1054: two exact declarations that the fixed-pose check's own
    # courtyard/pad/hole geometry says cannot both be seated.
    'plan_fixed_pose_conflict',
    # Row 8: two FILE-locked parts overlapping. A WARN per pair; an ERROR
    # only when their overlap alone exceeds a DECLARED overlap budget.
    'plan_fixed_overlap', 'plan_fixed_overlap_budget',
    # #1051. Raised by `array_problems` (grade AND the intent gate): a member
    # or served part the board does not have, and a row that cannot be
    # formed as declared (a locked member, mixed footprints, a block
    # rotation or zone contradicting the row). Each names its member.
    'array_unresolved', 'array_conflict',
    # #1054. Raised by `fixed_pose_violations`: a fixed pose for a ref the
    # board does not have (ERROR), or one the anchor cannot grade (WARN).
    'fixed_pose_unresolved',
    # #1060: a declared courtyard waiver, disclosed (WARN by default).
    'fixed_pose_overlap_waived'})

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
    'keep_away': 'the intent declares no keep_away rules',
    'must_lock': 'the intent declares no must_lock patterns',
    'legality': 'the intent declares no legality_budget',
    # The rule's exclusion list IS the declaration: without it every
    # connector on the board would be named for facing the edge it mates at.
    'pins_to_edge': 'the intent declares no edge_connectors',
    'array_formation': 'the intent declares no arrays',
    'replicate': 'the intent declares no replicate copies',
    'template': 'the intent declares no templates',
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


def _arm_decap_superseded(ctx) -> Optional[str]:
    """The decap distance rules have nothing of their own to grade when a
    declared relation supersedes EVERY cap with an IC on its rail (#959).
    Read by the roster's APPLICABILITY only: a dark decap rule then owes
    nothing. It is deliberately not in `_ARM` -- an armed rule that abstains
    makes the grade incomplete, and declaring MORE (a relation for every
    cap) turned exit 0 into exit 4 (Phase-6 verifier). Armed, the rules run
    and skip each superseded cap, which is the truth: every cap IS graded,
    by its relation. The trade, taken deliberately: they then count in
    `rules_run` while `proximity` does their measuring (see `_ARM` below).
    """
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
_ARM = {'decap_pin_distance': _arm_decap_pins}


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
    if rule == 'keep_away':
        return bool(intent.keep_away)
    if rule == 'must_lock':
        return bool(intent.must_lock)
    if rule == 'legality':
        return bool(intent.legality_budget)
    if rule == 'pins_to_edge':
        # Armed by the edge-connector DECLARATION, which is the rule's
        # exclusion list: without it the rule would name every connector on
        # the board for facing the edge it mates at.
        return bool(intent.edge_connectors)
    if rule == 'array_formation':
        return bool(intent.arrays)
    if rule == 'replicate':
        return bool(intent.replicate)
    if rule == 'template':
        return bool(intent.templates)
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
    'keep_away': 'keep_away[]',
    'must_lock': 'must_lock[]',
    'legality': 'legality_budget',
    'pins_to_edge': 'edge_connectors[]',
    'array_formation': 'arrays[]',
    'replicate': 'replicate[]',
    'template': 'templates[]',
}

#: What each rule reports at when the intent's `severity` map says nothing.
#: One table, because "is this rule advisory" is what the roster refuses on
#: and the answer lived in three places: `severity_of`'s ERROR default, two
#: rules passing `default=WARN`, and one hard-wired WARN. Pinned against the
#: severity each rule actually EMITS by a test that fires every rule, so the
#: table cannot drift from the rules without failing.
_RULE_DEFAULT_SEVERITY = dict({name: ERROR for name, _ in RULES},
                              assembly_side=WARN, decap_ungraded=WARN,
                              pins_to_edge=WARN, keep_away=WARN,
                              replicate=WARN, template=WARN)

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
    # #1051. Which identical parts form ONE row is a design decision the
    # board file does not record: a detector may SUGGEST a candidate from
    # the pin numbering, and only a declaration arms the rule.
    'array_formation': ('a design decision about which parts form one row; '
                        'nothing on the board says which identical parts '
                        'belong together'),
    # #1253. Which nets are noisy and which sensitive is a design fact.
    'keep_away': ('a design relation between two net groups; nothing on the '
                  'board says which nets are noisy and which are sensitive'),
    # #1211. The board says which instances are siblings; that one of them
    # is the TEMPLATE the others copy is a design decision.
    'replicate': ('a design decision about which sheet instance is the '
                  'template; the board says which instances are siblings, '
                  'not that they must be laid out alike'),
    # Where a chip's support parts sit is read from its datasheet's layout.
    'template': ("an arrangement read from a part's datasheet; the board says "
                 'which parts are wired to the chip, not where its datasheet '
                 'puts them'),
}


def _brief_claims(rule: str, brief_fragment) -> List[str]:
    """What the design BRIEF declares that `rule` grades, by name (#959).

    A brief-declared requirement is not the roster's to excuse: a plan that
    leaves it out has DROPPED a declaration, which leaves that clause without
    a verdict (`check_floorplan --require-brief-coverage` exits 4 on it). So
    the roster reports such a rule `uncovered` rather than asking for a
    second, rule-level answer to the same question."""
    frag = brief_fragment or {}
    if rule == 'proximity':
        return [f"{p.get('ref')}~{p.get('near')}"
                for p in frag.get('proximity') or ()]
    if rule == 'keepout':
        return [str(k.get('name')) for k in frag.get('keepouts') or ()]
    if rule in ('edge_connector', 'pins_to_edge'):
        return [str(c.get('ref')) for c in frag.get('edge_connectors') or ()
                if c.get('edge')]
    if rule == 'array_formation':
        return [str(a.get('name')) for a in frag.get('arrays') or ()]
    if rule == 'keep_away':
        return [str(k.get('rule')) for k in frag.get('keep_away') or ()]
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
                        if non_aperture_pads(fp)})
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
                       "compiles one, so leaving it dark owes nothing")
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
    this answer -- a way to make the report go away by editing a severity
    no finding will ever carry.

    The one exception is `decap_ungraded` under a `--decaps-from` intent
    (#1142): its held list makes it gating per cap, and an explicit `warn`
    then makes it advisory again, because the rule obeys that warn for every
    cap -- so no finding of it can be an error.
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
    if (rule == 'decap_ungraded' and rule not in intent.severity
            and (intent.decaps or {}).get('within_radius_refs')):
        # #1142: a held cap left beyond the radius is an ERROR, per cap. An
        # explicit severity is read below and wins, as it does in the rule.
        return True
    if intent.severity.get(rule) == ERROR:
        return True
    return _RULE_DEFAULT_SEVERITY.get(rule, ERROR) == ERROR


def _roster(intent: Intent, pcb_data, ctx, *, census=None,
            brief_fragment=None) -> List[Dict[str, object]]:
    """One row per rule in `RULES`: is it armed, and if not, does anything
    answer for that? The `needs_disposition` rows are what `--plan-only`
    reports as `rules_dark_undispositioned`.

    A rule is REFUSED only when all of these hold: it is dark (or abstained,
    or armed with a withheld key), it is not a policy rule, a board fact says
    it applies, it is gating, and no written disposition answers for it.
    Withheld budgets are refused like dark rules, for decaps and legality
    alike: in both cases the emitter declined to derive a number, and in both
    cases the honest answers are "declare it from a requirement" or "say why
    not" (#959 Phase 0 checkpoint).
    """
    # The intent the grade ran (#1098: plus derived mating keep-outs), so the
    # roster never calls a rule inapplicable that the grade just fired.
    intent = getattr(ctx, 'intent', None) or intent
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
                f"); a plan that drops one leaves it without a verdict, "
                f"which --require-brief-coverage refuses")
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
    not have, one that carries pads (the placer's to place), or one already
    FILE-locked (the lock is the answer).
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
                why = ('the block carries pads -- the placer places it, '
                       'and `refs` answers pad-less blocks only')
            elif getattr(fp_, 'locked', False):
                why = 'the block is already locked; the lock is the answer'
            else:
                continue
            out.append(f"dispositions.refs.{k}: {why}")
    return sorted(out + stale_c)


def roster_refusal_lines(rows) -> List[str]:
    """One line per `needs_disposition` row, naming the key that arms the rule and
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
    """The roster for a PLAN, before anything is graded (`--plan-only`).

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
    #: #1253: every declared keep-away rule's reading, passing rules
    #: included -- the closest pad pair and both pads' sides. Empty when the
    #: intent declares none.
    keep_away_measured: List[Dict[str, object]] = field(default_factory=list)
    #: #1211: every declared copy target's distance from its source's
    #: arrangement, passing ones included (plus one row per declaration the
    #: board cannot resolve). Empty when the intent declares none.
    replicate_measured: List[Dict[str, object]] = field(default_factory=list)
    #: Every declared template's members' drift from their template poses,
    #: passing ones included. Empty when the intent declares none.
    template_measured: List[Dict[str, object]] = field(default_factory=list)
    #: #1051: every declared array's formation measurement, formed or not,
    #: with the members its pin order could not place. Empty when no array
    #: is declared.
    array_measured: List[Dict[str, object]] = field(default_factory=list)
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
    #: #1064: `legality.pad_stack_census` -- check_assembly's exact
    #: pad_intersection pairs (two parts' pad copper overlapping, ANY net),
    #: PRINTED and never a rule. None when `grade(with_pad_stacks=True)` was
    #: not asked, which a consumer must not read as "no stacks". The
    #: `legality['pad_intersection_pairs']` key beside it is the optimizer's
    #: bounding-box census and keeps that meaning.
    pad_stacks: Optional[Dict[str, object]] = None

    @property
    def dark_undispositioned(self) -> List[str]:
        """Dark rules no disposition answers, by name. Empty when no roster was built,
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


def _run_rules(ctx, abstained=None, leave_out=()):
    """`(violations, ran, skipped)`: every rule in RULES over one `_Ctx`.

    The one loop both `grade` and `PoseGrader` run, so a grade asked about a
    pose nothing has written yet cannot drift from the grade of the board.
    `abstained` is the emitter's withheld-budget map, used only to word a
    skip reason. `leave_out` names rules not run at all."""
    found: List[Violation] = []
    ran: List[str] = []
    skipped: Dict[str, str] = {}
    for name, fn in RULES:
        if name in leave_out:
            continue
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


def _ctx_census(ctx):
    """The ctx's `CourtyardCensus`, built once."""
    if ctx._census is None:
        ctx._census = legality.CourtyardCensus(ctx.pcb, ctx.pcb_file)
    return ctx._census


def _budget_outline_areas(ctx, refs):
    """`{ref: (side, mm2)}`: what each part of `refs` the overlap budget
    grades COVERS, on the budget's own geometry (#1162 verifier): its graded
    outline, clipped to the rect a zone or the board confines (the grade
    ladder's), at rotation 0 -- both turn together, so it is pose-free.
    A bound on forced overlap is sound only in the currency the budget is
    measured in: summing rects (an r=5 circle courtyard as a 10 x 10
    square) called a layout the grade passes at 61.4 mm2 a forced 100."""
    from shapely.geometry import box
    census = _ctx_census(ctx)
    st = ctx.state
    eligible, _x = legality.courtyard_budget_universe(
        census, [r for r in refs if r in st.parts],
        {r: (0.0, 0.0, 0.0) for r in refs})
    out = {}
    for r, g in eligible.items():
        b0 = st.parts[r].grade_by_rot[0.0]
        shape = g.poly if g.poly is not None else box(*g.rect)
        out[r] = (st.parts[r].side, shape.intersection(box(*b0)).area)
    return out


def _ctx_overlap_exact(ctx):
    """`(exact_total, rect_total, pairs)` for the parts `ctx` grades, at
    the poses it grades them (#1162): `legality.courtyard_overlap_pairs` over
    `ctx.parts` -- the grade's own universe, the container exemption the
    quench's metric applies -- with the grade's rects as the RECT reading.
    Cached on the ctx; zero when the project waives courtyard overlap, as the
    quench's `overlap_area` is."""
    if ctx._overlap_exact is not None:
        return ctx._overlap_exact
    st = ctx.state
    if getattr(st, 'courtyards_ignored', False):
        ctx._overlap_exact = (0.0, 0.0, [])
        return ctx._overlap_exact
    census = _ctx_census(ctx)
    cont = set(getattr(st, 'container_refs', ()) or ())
    refs = [r for r in ctx.parts if r not in cont]
    if hasattr(st, 'pose'):
        poses = {r: tuple(st.pose(r)) for r in refs}
    else:
        poses = {r: (st.parts[r].x, st.parts[r].y, st.parts[r].rot)
                 for r in refs if r in st.parts}
    gps = ctx.parts

    def rect_of(r):
        g = gps[r]
        return (g.sides, g.side, g.rect, g.tht_rect)
    ctx._overlap_exact = legality.courtyard_overlap_pairs(
        census, refs, poses, rect_of=rect_of)
    return ctx._overlap_exact


def _ctx_overlap_pairs_among(ctx, among):
    """`_ctx_overlap_exact(ctx)[2]` restricted to pairs whose two parts are
    both in `among`, measured over those parts alone: a pair's overlap does not
    depend on any other part. The full census is read when it is already
    cached. plan_check asks this of the LOCKED parts, where the full census
    measured every pair of a pile (glasgow_revC: 34,716 pairs, 4.7 s)."""
    among = set(among)
    if ctx._overlap_exact is not None:
        return [p_ for p_ in ctx._overlap_exact[2]
                if p_[0] in among and p_[1] in among]
    st = ctx.state
    if getattr(st, 'courtyards_ignored', False):
        return []
    census = _ctx_census(ctx)
    cont = set(getattr(st, 'container_refs', ()) or ())
    refs = [r for r in ctx.parts if r not in cont and r in among]
    if hasattr(st, 'pose'):
        poses = {r: tuple(st.pose(r)) for r in refs}
    else:
        poses = {r: (st.parts[r].x, st.parts[r].y, st.parts[r].rot)
                 for r in refs if r in st.parts}
    gps = ctx.parts

    def rect_of(r):
        g = gps[r]
        return (g.sides, g.side, g.rect, g.tht_rect)
    return legality.courtyard_overlap_pairs(
        census, refs, poses, rect_of=rect_of)[2]


def violation_claim(v: Violation) -> Tuple:
    """The identity `grade_delta` counts a violation by: `(rule, ref, block,
    expected keys)`. The message and the measured numbers are left out, so
    one finding moved by a millimetre is still the same claim. Module-level
    so a per-ref caller can ask the same question (#1066): is the violation
    a part was charged for still on the board?"""
    return (v.rule, v.ref or '', v.block or '',
            tuple(sorted((v.expected or {}).keys())))


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

    claim = violation_claim
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
    since an author can turn a part a search would not. That is not every
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
                local0 = sp.grade_by_rot[0.0]
                tried = []
                feasible = False
                for base in sorted({(fp.rotation or 0.0) % 360.0, 0.0}):
                    v = zone_pose_feasibility(
                        zb.rect, tol, _LocalPart(base, local0, None),
                        [{'name': za.name, 'rect': za.rect,
                          'sides': ('F', 'B'), 'allow': ()}],
                        open_sides=zone_open_sides(
                            zb.rect, getattr(pcb_data.board_info,
                                             'board_bounds', None),
                            claimed_edges(intent).get(ref)))
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


def declared_side_claims(intent: Intent, blocks: Dict[str, List[str]]
                         ) -> Dict[str, List[Tuple[str, str]]]:
    """Every face the intent declares for a part: `{ref: [(side, source)]}`.

    The three places a side can be declared -- `blocks[].side` (for the
    block's RESOLVED members), `edge_connectors[].side` and
    `fixed_poses[].side` -- read once, so `plan_check`'s per-face pricing
    and `place_simple`, which applies the faces before annealing, cannot
    disagree about where a part belongs. `None` and `"unknown"`
    declare nothing. Refs the board lacks are kept: whether to drop them is
    the caller's question.
    """
    claims: Dict[str, List[Tuple[str, str]]] = {}

    def claim(ref, side, source):
        if side in ('F', 'B') and ref:
            claims.setdefault(str(ref), []).append((side, source))

    for zone in intent.blocks:
        for ref in blocks.get(zone.name, ()):
            claim(ref, zone.side, f"blocks[{zone.name}]")
    for c in intent.edge_connectors:
        claim(c.get('ref'), c.get('side'), f"edge_connectors[{c.get('ref')}]")
    for f in intent.fixed_poses:
        claim(f.get('ref'), f.get('side'), f"fixed_poses[{f.get('ref')}]")
    return claims


def declared_sides(claims: Dict[str, List[Tuple[str, str]]]
                   ) -> Tuple[Dict[str, str], Dict[str, List[Tuple[str, str]]]]:
    """`(sides, conflicts)` from `declared_side_claims`: the one face every
    claim on a ref agrees on, and the refs whose claims name both faces.

    A conflict is not resolved here: choosing either face would invent a
    precedence rule. `plan_check` leaves such a part on its board face (the
    side grade names the contradiction) and `place_simple` refuses."""
    sides: Dict[str, str] = {}
    conflicts: Dict[str, List[Tuple[str, str]]] = {}
    for ref, rows in claims.items():
        faces = {side for side, _src in rows}
        if len(faces) == 1:
            sides[ref] = faces.pop()
        else:
            conflicts[ref] = list(rows)
    return sides, conflicts


def declared_moves(intent: Intent, sides: Dict[str, str], pcb_data
                   ) -> Tuple[Dict[str, Dict], List[Dict]]:
    """`(fixed, moves)`: the intent's `fixed_poses[]` entries for refs the
    board has, `{ref: entry}`, and the writer's moves (`write_placed_output`
    rows) that put each part at its declared pose and on its declared face
    (`sides`, from `declared_sides`) where it is not there yet -- what
    `place_simple` applies before it seats or anneals anything, and what an
    edge-seat measure made before placing (`edge_seat.oversubscribed`) must
    apply too."""
    from .legality import footprint_side
    fixed = {str(f['ref']): f for f in intent.fixed_poses
             if str(f['ref']) in pcb_data.footprints}
    pre: Dict[str, Dict] = {}
    for r, f in sorted(fixed.items()):
        foot = pcb_data.footprints[r]
        rot = (float(f['rot']) if f.get('rot') not in (None, 'unknown')
               else (foot.rotation or 0.0))
        if (abs(float(f['x']) - foot.x) > 1e-6 or abs(float(f['y']) - foot.y) > 1e-6
                or abs((rot - (foot.rotation or 0.0) + 180.0) % 360.0 - 180.0) > 1e-6):
            pre[r] = {'reference': r, 'new_x': float(f['x']), 'new_y': float(f['y']),
                      'new_rotation': rot}
    for r, side in sorted(sides.items()):
        foot = pcb_data.footprints[r]
        if footprint_side(foot) != side:
            pre.setdefault(r, {'reference': r, 'new_x': foot.x, 'new_y': foot.y,
                               'new_rotation': foot.rotation or 0.0})['new_side'] = side
    return fixed, [pre[r] for r in sorted(pre)]


def _planned_sides(intent: Intent,
                   blocks: Dict[str, List[str]]) -> Dict[str, str]:
    """The face each part is PRICED on by `plan_check`: the one its
    declarations agree on (`declared_side_claims`), else none -- the caller
    falls back to the board face.

    `plan_check` runs before the planned placement exists. The board can be
    a generator pile with every movable footprint on one face, so its current
    layers are not evidence for the plan's per-face capacity; the faces the
    plan declares are, and `place_simple` puts the parts on them before it
    anneals.
    """
    return declared_sides(declared_side_claims(intent, blocks))[0]


def edge_oversubscribed_findings(intent: Intent, pcb_data, pcb_file: str, *,
                                 clearance: Optional[float] = None,
                                 board_edge_clearance: Optional[float] = None,
                                 group_sources: Sequence[str] = ()
                                 ) -> List[Violation]:
    """One `edge_oversubscribed` finding per edge and face whose declared
    parts cannot all seat there together: the records `place_simple` makes
    (`edge_seat.oversubscribed`, the seat lines `_crowding` packs), worded as
    place_simple words them (`edge_seat.crowding_text`). WARN unless the
    intent's severity map promotes it. A floor left None is the board's own
    (`resolved_floors`), as place_simple reads it.

    place_simple reports such an edge once per run and places the parts that
    do not fit as any other part; before this, a check of the plan, or of
    the pile it is placed from, said nothing about it."""
    from .edge_seat import crowding_text, oversubscribed
    if not any(c.get('edge') for c in intent.edge_connectors):
        return []
    clearance, board_edge_clearance = resolved_floors(
        pcb_file, clearance, board_edge_clearance)
    out = []
    for rec in oversubscribed(pcb_data, pcb_file, intent, clearance,
                              board_edge_clearance, group_sources):
        need = sum(min(t['length_mm'] for t in p['turns'])
                   for p in rec['parts'])
        out.append(Violation(
            rule='edge_oversubscribed',
            severity=intent.severity_of('edge_oversubscribed', WARN),
            message=crowding_text(rec),
            measured={'edge': rec['edge'], 'side': rec['side'],
                      'parts': [p['ref'] for p in rec['parts']],
                      'parts_mm': round(need, 4),
                      'free': rec['free'], 'fit': rec['fit'],
                      'seated': rec['seated'], 'unseated': rec['unseated']},
            expected={'fit': len(rec['parts'])}))
    return out


def plan_check(intent: Intent, pcb_data, pcb_file: str, *,
               group_sources: Sequence[str] = (),
               clearance: Optional[float] = None,
               board_edge_clearance: Optional[float] = None):
    """The zone plan checked against itself and the board, BEFORE the first
    pose write (#959, #998). Returns `(violations, measured)`.

    Run 29 found its zone plan's ERRORs at lap 5 because the only caller of
    `validate_intent` was `grade`, which runs AFTER a placement is written.
    This runs the checks that need no pose of a movable part, so a plan can
    be refused before anything is placed.

    EVERY ERROR IS SOUND -- the plan cannot be seeded or satisfied however
    the movable parts are arranged -- and the evidence behind each bound is
    in its message. The WARN halves are the same quantities with a margin.

      1. `intent_zone_overlap` (WARN; the old "no placement can satisfy both"
         was false) and `plan_zone_exclusive_unsatisfiable` (ERROR).
      2. `intent_zone_in_keepout`, as `grade` raises it.
      3. `block_glob_literal`: a real reference used as a glob.
      4. `plan_fixed_outside_zone`: a FILE-locked member already outside its
         zone. The placer does not move a locked part, so this is a fact
         about the plan.
         Also (#1054, WARN unless set) a `fixed_poses[]` entry whose DECLARED
         pose is outside its own block's zone: the placer seats it exactly.
      4c. `plan_fixed_pose_conflict` (ERROR): two declared exact poses whose
          courtyards, pads or holes conflict under the fixed-pose check's own
          geometry.
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
         DECLARED `legality_budget.overlap_area`: past it, ERROR. The ERROR
         sums the members' graded OUTLINES and far-side clusters -- the
         geometry the budget is measured on (#1162) -- the WARN their rects.
         With no budget declared, or a project that waives courtyard overlap
         (the grade then prices none), nothing bounds the overlap, and it is
         the WARN (the placer's legality gate never admits an overlap, so it
         may leave members illegal). An anchor-graded member
         (`zone_is_anchor`) is not
         charged.
         Anchor-graded members and zones holding a waived pair are not
         charged; a locked member is, because the grade counts its overlap
         too.
      6. `plan_edge_overfull` / `_crowded`: one edge-claimed part whose PAD
         extent in its own frame, at its best 90-degree rotation, is longer
         than the edge (ERROR); the claimed parts' summed extents against the
         edge's span (WARN -- flanges legitimately overhang corners).
      7. `plan_board_overfull` / `_crowded`: `options.grow_board` at
         clearance 0 on the per-face basis. The same argument as row 5: an
         ERROR when the overlap the graded outlines force (per face, against
         `grow_board`'s usable area) exceeds a declared budget; a WARN past
         `options.CROWDED_UTILISATION` or on the one-face basis a declared
         `assembly.sides` implies.
      Rows 5 and 7 charge each part to the face the intent DECLARES for it
      (`blocks[].side`, `edge_connectors[].side`, `fixed_poses[].side`, when
      they agree -- `declared_side_claims`), else to its board face: a pile
      has every part on F.Cu, and the plan is priced as it will be placed
      once `place_simple` has applied those faces.
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
    #    restricted to the members the placer cannot move.
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
                         f"outside its zone: {v.message}. Nothing the placer "
                         f"does will move it -- fix the zone or the pose"),
                measured=v.measured, expected=v.expected))

    # 4b. #1054: a FIXED POSE outside its own block's zone, at the declared
    #     pose. Stage 0 seats the pose exactly and never moves it, so the zone
    #     is failed from the first write -- the same contradiction row 4
    #     names for a file lock, raised before anything is seeded. A WARN
    #     unless the plan sets it: the author may mean the pose to win.
    #     No #1174 open side here: only an edge claim opens one, and the
    #     loader refuses a fixed pose on any `edge_connectors` ref
    #     (`_fixed_pose_entries`), so no member graded here has one.
    _fx = {str(f['ref']): f for f in intent.fixed_poses}
    for z in intent.blocks:
        if z.rect is None:
            continue
        tol = intent.zone_tolerance(z)
        for ref in blocks.get(z.name, ()):
            f = _fx.get(ref)
            part = state.parts.get(ref)
            if f is None or part is None:
                continue
            rot = f.get('rot')
            rot = (part.rot if rot is None or rot == 'unknown'
                   else float(rot) % 360.0)
            fx_, fy_ = float(f['x']), float(f['y'])
            side = f.get('side')
            if side in ('F', 'B') and side != getattr(part, 'side', side):
                # The DECLARED face: the part's courtyard mirrored the way
                # the placement writer flips a footprint (local y -> -y, the
                # caller's angle), then turned to the declared rotation --
                # not its outline on the face it is on now.
                from .legality import rotate_local_bounds
                b0 = part.grade_by_rot[0.0]
                e = rotate_local_bounds(b0[0], -b0[3], b0[2], -b0[1], rot)
                r = (fx_ + e[0], fy_ + e[1], fx_ + e[2], fy_ + e[3])
            else:
                r = part.grade_rect(fx_, fy_, rot)
            if zone_fits_courtyard(z.rect, r, tol):
                esc, _axis = _rect_escape(z.rect, r)
            else:
                cx, cy = (r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0
                esc, _axis = _rect_escape(z.rect, (cx, cy, cx, cy))
            if esc > tol + 1e-9:
                out.append(Violation(
                    rule='plan_fixed_outside_zone',
                    severity=intent.severity.get('plan_fixed_outside_zone',
                                                 WARN),
                    ref=ref, block=z.name,
                    message=(f"{ref}'s fixed pose ({float(f['x']):g}, "
                             f"{float(f['y']):g}) lies {esc:.2f}mm outside "
                             f"its block {z.name!r}'s zone (tolerance "
                             f"{tol:g}mm). The placer seats the pose exactly, "
                             f"so the zone fails from the first write -- "
                             f"move the pose or the zone"),
                    measured={'escape_mm': round(esc, 4),
                              'pose': [float(f['x']), float(f['y']), rot]},
                    expected={'zone': list(z.rect), 'tolerance_mm': tol}))

    # 4c. Exact declarations are one simultaneous batch. Compare them
    # pairwise now with the fixed-pose check's own side-aware
    # courtyard/pad/hole predicate, so an impossible pair is refused before a
    # board is written. This is deliberately not folded into row 8: these
    # refs need not be locked in the input file, and no overlap budget can
    # license a pair the exact-pose check itself refuses.
    from .seeder import fixed_pose_pair_conflicts
    fixed_pose_conflicts = fixed_pose_pair_conflicts(
        state, intent.fixed_poses,
        waived=frozenset(frozenset(p_)
                         for p_ in intent.courtyard_waiver_pairs()))
    for conflict in fixed_pose_conflicts:
        a_, b_ = conflict['pair']
        out.append(Violation(
            rule='plan_fixed_pose_conflict',
            severity=intent.severity_of('plan_fixed_pose_conflict'),
            ref=a_,
            message=(f"fixed poses {a_} and {b_} cannot both be seated: "
                     f"{conflict['reason']}. Stage 0 judges exact anchors as "
                     f"one batch and would refuse both; move one pose"),
            measured=conflict,
            expected={'conflicts': []}))
    measured['fixed_pose_conflicts'] = fixed_pose_conflicts

    # 8. two FILE-locked parts overlapping -- in every placement there is.
    # #1162: on the reading the budget is graded in (check_assembly's drawn
    # outlines), from the grade's own exact pairs restricted to locked ones.
    fixed_refs = sorted(r for r in ctx.parts if r in ctx.locked)
    _fixed = set(fixed_refs)
    fixed_pairs = []
    for a_, b_, _rc, area in sorted(_ctx_overlap_pairs_among(ctx, _fixed),
                                    key=lambda p_: (p_[0], p_[1])):
        if a_ in _fixed and b_ in _fixed:
            if area > legality.EPS:
                fixed_pairs.append((a_, b_, area))
                out.append(Violation(
                    rule='plan_fixed_overlap',
                    severity=intent.severity_of('plan_fixed_overlap', WARN),
                    ref=a_,
                    message=(f"{a_} and {b_} are both LOCKED in the board and "
                             f"their courtyards overlap by {area:.3f}mm2 -- "
                             f"in every placement, since the placer moves "
                             f"neither"),
                    measured={'pair': [a_, b_],
                              'overlap_area_mm2': round(area, 4)},
                    expected={'overlap_area_mm2': 0.0}))
    budget = (intent.legality_budget or {}).get('overlap_area')
    fixed_total = sum(x[2] for x in fixed_pairs)
    if budget is not None and fixed_total > float(budget) + legality.EPS:
        out.append(Violation(
            rule='plan_fixed_overlap_budget',
            # As strong as the rule it stands for: with `legality` demoted
            # the grade passes this overlap, so the plan check must not
            # call it an error either.
            severity=_plan_severity(intent, 'plan_fixed_overlap_budget',
                                    ('legality',)),
            message=(f"the LOCKED parts alone overlap by {fixed_total:.3f}"
                     f"mm2, over the declared legality_budget.overlap_area "
                     f"{float(budget):g} -- no arrangement of the other parts "
                     f"can bring the total under it. Raise the budget, or "
                     f"unlock one of each pair and move it -- unless "
                     f"mechanical.json fixes that part's pose, which a plan "
                     f"cannot move: "
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
    # Sound for the GRADE, not only for the placer: courtyards may overlap
    # there up to a declared `legality_budget.overlap_area`, so members
    # whose areas exceed the zone by A must overlap by at least A in total
    # (Bonferroni: union >= sum - pairwise), and the grade's overlap is that
    # pairwise sum over the whole board. ERROR only past a DECLARED budget;
    # with none, nothing bounds the overlap and it is a WARN about the placer.
    # EVERY member counts, locked ones included: the grade counts a locked
    # pair's overlap too, so against the budget the sum is sound -- and
    # dropping them lost a zone a locked part already fills (round-2
    # verifier). A locked member outside its zone is `plan_fixed_outside_zone`.
    overlap_budget = (intent.legality_budget or {}).get('overlap_area')
    # The rotation claims, per ref, from every block. A part two blocks
    # declare at different angles is a contradiction (`rotations_for_ref`
    # raises, naming both blocks); here it is bounded on EVERY angle a block
    # claims for it, never on its own angle: a bound read on its own angle
    # reads 45-degree bars declared at 0 and at 90 as overfull at 45.
    _conflicts: Dict[str, List[tuple]] = {}
    _claims = rotations_for_ref(intent, blocks, conflicts=_conflicts)
    for _r, _seen in _conflicts.items():
        _claims[_r] = (None, tuple(sorted({
            float(a) for rot, cands in _seen
            for a in ((rot,) if rot is not None else (cands or ()))})))
    if getattr(state, 'courtyards_ignored', False):
        # The project waives KiCad's courtyard rule, so the grade prices no
        # courtyard overlap (#1104): no forced overlap can exceed the budget,
        # and these bounds stand down to their WARNs (8492b3c1 verifier:
        # three circles ERRORed at 35.5 where the grade read 0).
        overlap_budget = None
    # #1174: an edge member whose declared edge the zone leaves open may
    # sit past that side, so the zone need hold none of it.
    _claimed5 = claimed_edges(intent)
    planned_sides = _planned_sides(intent, blocks)
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
        # member, which is what the legality gate actually needs between
        # them.
        per_face_c = {'F': 0.0, 'B': 0.0}
        # The ERROR half, on the budget's own geometry (#1162 verifier): the
        # members' graded OUTLINES, not their rects.
        per_face_x = {'F': 0.0, 'B': 0.0}
        _outline = _budget_outline_areas(ctx, members)
        counted = []
        for r in members:
            part = state.parts[r]
            b0 = part.grade_by_rot[0.0]
            w, h = b0[2] - b0[0], b0[3] - b0[1]
            if _anchor_reachable(z, part, tol, _claims.get(r)):
                # Anchor-graded at some rotation the seed may give it, as the
                # grade decides it (`zone_is_anchor`): on the part's own
                # 90-degree lattice -- two 10 x 4 parts at 45 degrees were
                # charged as if seated at 0 (8492b3c1 verifier) -- or on any
                # the block DECLARES (`rotation` / `rotation_candidates`),
                # where a fit at one angle does not forbid the anchor at
                # another (b6a63ad8 verifier).
                continue
            if zone_open_sides(z.rect, ctx.outline_bounds,
                               _claimed5.get(r)):
                # #1174: not charged, like an anchor-graded member -- it can
                # sit wholly past the open side, so charging its outline
                # ERRORed an intent the grade passes (phase-1 verifier).
                continue
            planned_side = planned_sides.get(r, part.side)
            per_face[planned_side] = per_face.get(planned_side, 0.0) + w * h
            per_face_c[planned_side] = per_face_c.get(planned_side, 0.0) + (
                (w + clr_used) * (h + clr_used))
            if r in _outline:
                per_face_x[planned_side] = (
                    per_face_x.get(planned_side, 0.0) + _outline[r][1])
            t0 = (part.tht_by_rot or {}).get(0.0)
            if t0 is not None:
                # The drilled-pad rect's part INSIDE the courtyard: only the
                # courtyard is confined to the zone, so a lead field reaching
                # past it may sit outside the zone (round-2 verifier: a 12x1
                # drill rect on a 10x4 courtyard was charged 2 mm2 the grade
                # never counts). Pose-invariant, since both turn together.
                # Per CLUSTER of drilled pads (#1206), as the grade measures
                # the far side: the union box charged ulx3s GPDI1's two shell
                # posts and the gap between them, and called its own shipped
                # arrangement (0.0 mm2 under the grade) a forced overlap.
                far = 'B' if planned_side == 'F' else 'F'
                per_face[far] = per_face.get(far, 0.0) + (
                    legality.far_overlap_area(t0, b0))
                # The budget measures the far side on these same cluster
                # boxes (`legality._pair_exact`), so they are in its
                # currency too.
                if r in _outline:
                    per_face_x[far] = per_face_x.get(far, 0.0) + (
                        legality.far_overlap_area(t0, b0))
            counted.append(r)
        worst = max(per_face, key=lambda f_: per_face[f_])
        need = per_face[worst]
        worst_x = max(per_face_x, key=lambda f_: per_face_x[f_])
        need_x = per_face_x[worst_x]
        zone_rows.append({'block': z.name, 'face': worst,
                          'members_area_mm2': round(need, 3),
                          'members_outline_area_mm2': round(need_x, 3),
                          'zone_area_mm2': round(zarea, 3),
                          'overlap_budget_mm2': overlap_budget,
                          'members': counted})
        excess = need - zarea
        # The forced overlap the BUDGET sees (Bonferroni on the outlines and
        # far-side clusters it measures).
        excess_x = need_x - zarea
        if (overlap_budget is not None
                and excess_x > float(overlap_budget) + legality.EPS):
            out.append(Violation(
                rule='plan_zone_overfull',
                severity=_plan_severity(intent, 'plan_zone_overfull',
                                        ('zone_containment', 'legality')),
                block=z.name,
                message=(f"zone {z.name!r} is {zarea:.2f}mm2 (with its "
                         f"{tol}mm tolerance) and its {len(counted)} "
                         f"member(s) on {worst_x}.Cu cover {need_x:.2f}mm2 "
                         f"by their graded outlines alone: fitting them "
                         f"forces at least {excess_x:.2f}mm2 of courtyard "
                         f"overlap, over the declared "
                         f"legality_budget.overlap_area "
                         f"{float(overlap_budget):g}"),
                measured={'members_outline_area_mm2': round(need_x, 4),
                          'members_area_mm2': round(need, 4),
                          'face': worst_x, 'members': counted,
                          'forced_overlap_mm2': round(excess_x, 4)},
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
                         f"placer's legality gate never admits -- it may "
                         f"leave some illegal"),
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
                         f"placer may leave some illegal"),
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
            if fp is None or edge not in blen or not non_aperture_pads(fp):
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
                    # As strong as the rule it stands for: with
                    # `edge_connector` demoted, the grade passes a part that
                    # overruns its edge, so the plan check must not refuse
                    # the plan for it (PR fact-check).
                    severity=_plan_severity(intent, 'plan_edge_overfull',
                                            ('edge_connector',)),
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
    # 6b. the edges whose declared parts cannot all seat together, on the
    #     seat lines place_simple packs (`edge_oversubscribed_findings`)
    crowded = edge_oversubscribed_findings(
        intent, pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, group_sources=group_sources)
    out.extend(crowded)
    measured['edge_oversubscribed'] = [v.measured for v in crowded]

    # 7. interior utilisation
    try:
        g0 = _opts.grow_board(pcb_data, pcb_file, clearance=0.0,
                              board_edge_clearance=0.0, assembly_sides=None,
                              side_overrides=planned_sides)
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
    # The ERROR on the budget's own geometry (#1162 verifier), as row 5:
    # every graded outline the budget sees, per face, against the usable
    # area. Never more than the rect sum, so a weaker bound -- never an
    # unsound one.
    # On the declared faces, as `g0` charges them: read off the pile, every
    # outline lands on F.Cu and the forced overlap is overstated -- an ERROR
    # for a plan whose declared busiest face is within the budget.
    _faces_x: Dict[str, float] = {'F': 0.0, 'B': 0.0}
    for _r, (_side, _a) in _budget_outline_areas(
            ctx, list(state.parts)).items():
        _side = planned_sides.get(_r, _side)
        _faces_x[_side] = _faces_x.get(_side, 0.0) + _a
    excess0_x = (max(_faces_x.values())
                 - (m0.get('usable_area_mm2') or 0.0))
    measured['outline_area_per_face_mm2'] = {
        k: round(v, 3) for k, v in sorted(_faces_x.items())}
    if g0.get('fits_by_area') is False and overlap_budget is not None \
            and on_board \
            and excess0_x > float(overlap_budget) + legality.EPS:
        out.append(Violation(
            rule='plan_board_overfull',
            severity=_plan_severity(intent, 'plan_board_overfull',
                                    ('legality',)),
            message=(f"the parts do not fit on the board by AREA alone, "
                     f"even at zero clearance on the busiest face "
                     f"(utilisation {util0}): their graded outlines force "
                     f"at least {excess0_x:.2f}mm2 of courtyard overlap, "
                     f"over the declared legality_budget.overlap_area "
                     f"{float(overlap_budget):g}"),
            measured={'utilisation': util0,
                      'forced_overlap_mm2': round(excess0_x, 4),
                      'forced_overlap_rect_mm2': round(excess0, 4)},
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
                                  assembly_sides=sides,
                                  side_overrides=planned_sides)
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
                         f"by area (utilisation {util1}). No placement "
                         f"search moves a part between faces (#836), so "
                         f"this is the policy's problem, not the plan's"),
                measured={'utilisation': util1},
                expected={'utilisation': '<= 1.0'}))
    out.sort(key=lambda v: v.sort_key())
    return out, measured


def resolved_floors(pcb_file: Optional[str], clearance: Optional[float] = None,
                    board_edge_clearance: Optional[float] = None
                    ) -> Tuple[float, float]:
    """`(clearance, board_edge_clearance)`: each value a caller passes, as
    passed, and each it leaves None as placement reads the board's own floors
    (`list_nets.board_floor_knobs`: the Default class floored at Board Setup's
    minimum, and an unset or sub-fab edge clearance read as the fab floor
    routing writes back, #1301). With no board file, `routing_defaults
    .CLEARANCE` and that unset floor. One resolver for every path that grades
    or reads edges with no floors handed to it (`_grade_ctx`, `emit_intent`),
    so none of them keeps the 0.55 edge clearance #1301 retired."""
    if clearance is not None and board_edge_clearance is not None:
        return float(clearance), float(board_edge_clearance)
    import routing_defaults as defaults
    from list_nets import board_floor_knobs, unset_board_edge_clearance
    if pcb_file and os.path.isfile(pcb_file):
        c0, e0, _knobs = board_floor_knobs(pcb_file)
    else:
        c0, e0 = defaults.CLEARANCE, unset_board_edge_clearance(None)[0]
    return (float(c0 if clearance is None else clearance),
            float(e0 if board_edge_clearance is None else board_edge_clearance))


def _grade_ctx(intent: Intent, pcb_data, pcb_file: str, *,
               group_sources: Sequence[str] = (),
               clearance: Optional[float] = None,
               board_edge_clearance: Optional[float] = None):
    """`(ctx, outline, state, blocks, block_problems)`: the one setup `grade`
    and `rule_roster` share, so a plan's roster reads the board facts its
    grade will read."""
    from .quench import QuenchState

    outline = outline_state(pcb_data, pcb_file)
    if not outline['trustworthy']:
        raise UntrustworthyOutline(outline['problems'])

    # a floor left None is the board's own, as placement reads it (#1301),
    # never a fixed 0.55 edge clearance
    clr, edge = resolved_floors(pcb_file, clearance, board_edge_clearance)
    state = QuenchState(pcb_data, pcb_file, clearance=clr,
                        board_edge_clearance=edge)

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
          reconciliation=None,
          with_pad_stacks: bool = False,
          edge_seats: bool = False) -> GradeResult:
    """Measure a board against its declared floorplan intent.

    `with_roster` (#959) also builds the rule roster -- which rules the intent
    leaves dark and whether a disposition answers for each. Opt-in, because
    it costs a decap census and most callers never read it.

    `with_pad_stacks` (#1064) also measures check_assembly's exact pad
    stacks into `GradeResult.pad_stacks`, for check_floorplan to PRINT. It
    is not a rule and changes no verdict; opt-in for the same reason.

    `edge_seats` also reports each edge whose declared parts cannot all seat
    together (`edge_oversubscribed_findings`, place_simple's own seat
    measure). Opt-in, because it seats every declared edge part on a
    scratch state; check_floorplan asks for it."""
    from . import placement_state

    ctx, outline, state, blocks, block_problems = _grade_ctx(
        intent, pcb_data, pcb_file, group_sources=group_sources,
        clearance=clearance, board_edge_clearance=board_edge_clearance)
    ctx.brief_fragment = brief_fragment

    violations = (list(validate_intent(intent)) + list(block_problems)
                  + list(unresolved_keepout_allows(intent, pcb_data))
                  + list(intent_zone_keepout_problems(
                      intent, blocks, pcb_data, pcb_file)))
    # #1051: the board-aware half of `arrays[]`, raised here and by
    # `resolve_intent_gate`, the way `block_unresolved` is.
    violations.extend(array_problems(intent, pcb_data, blocks,
                                     locked=ctx.locked))
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
    if edge_seats:
        violations.extend(edge_oversubscribed_findings(
            intent, pcb_data, pcb_file, clearance=clearance,
            board_edge_clearance=board_edge_clearance,
            group_sources=group_sources))
    if mechanical:
        violations.extend(mechanical_drift(intent, pcb_data, mechanical,
                                           skip=mechanical_skip))
        violations.extend(mechanical_anchor_violations(
            pcb_data, pcb_file, mechanical, skip=mechanical_skip,
            state=state, locked=ctx.locked, outline=outline))
        # NOT amending `skipped['zone_containment']` when the plan's zones
        # are dark: `_is_not_asked` matches skip reasons by exact text, and
        # an amended reason read as an armed abstention -- a clean board
        # then exited 4 "NOT FULLY GRADED" (narrow re-review). The anchors'
        # findings carry their own `mech:<ref>` block names instead.
    # #1054: every `fixed_poses[]` entry, whatever its source, through the
    # same anchor -- a ref the mechanical file declares is left to the file.
    violations.extend(fixed_pose_violations(
        intent, pcb_data, pcb_file, mechanical=mechanical,
        mechanical_skip=mechanical_skip, state=state,
        locked=ctx.locked, outline=outline))
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
                           for c in SUPPLY_PIN_CHANNELS},
            'pins_by_channel': {
                c: sum(len(r['pins']) for r in graded
                       if r['channel'] == c)
                for c in SUPPLY_PIN_CHANNELS},
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

    pad_stacks = None
    if with_pad_stacks:
        from .legality import pad_stack_census
        import routing_defaults as _defaults1064
        pad_stacks = pad_stack_census(
            pcb_data, clearance if clearance is not None
            else _defaults1064.CLEARANCE)

    st = placement_state.assess_placement(pcb_data, pcb_file)
    return GradeResult(
        roster=roster, stale_dispositions=list(stale or ()),
        pad_stacks=pad_stacks,
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
               # The subset `partially_unplaced` is actually decided on.
               # Without it a reader
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
        keep_away_measured=list(ctx.keep_away_measured),
        replicate_measured=list(ctx.replicate_measured),
        template_measured=list(ctx.template_measured),
        array_measured=list(ctx.array_measured),
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
#: any decap further than `DECAP_RADIUS_MM` from its nearest owner, so the
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
    # ONE election, partitioned -- not two queries whose relationship has
    # to be argued. The comment that used to stand here said the unbounded
    # pass was "a second measurement rather than a superset" because
    # "nearest chip carrying the rail" could elect a DIFFERENT chip without
    # the prune. That was never true of this code: `radius` only ever
    # appeared AFTER the argmin, and the chip list is built before any cap
    # is considered. Measured, 0 of 357 tethers re-elect. Since #794 the
    # radius does not reach the election at all, so it is a property of the
    # code shape rather than a fact about a corpus.
    near, beyond, orphans = groups_mod.decap_populations(pcb_data, radius=r)
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
    # Counted off the population rather than off the election's output --
    # but NOT independently of it, and the first draft of this comment
    # claimed otherwise. `_elect_tethers(movable=None)` walks the same
    # `decap_population`, so
    # `unaccounted` is 0 by CONSTRUCTION, not by measurement: it catches
    # a future election that drops a cap for some new reason, and it
    # cannot catch today's. Its battery row is recorded as an expected
    # survivor for exactly that reason, paired with one that breaks the
    # election so the tripwire has something to catch.
    scope = len(groups_mod.decap_population(pcb_data))
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
        'unaccounted': scope - n - len(beyond) - len(orphans),
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


def _pin_census_of(pcb_data) -> Dict[str, object]:
    """The supply-pin census `decap_pin_distance` grades, on one board
    (#1102): every graded pin's gap to the nearest decoupling cap on its net,
    with the rule's own functions -- `supply_pins`, `decap_pin_caps`,
    `nearest_rail_cap` -- and no exemptions."""
    pins = _decap_supply_pins(pcb_data)
    by_net = _decap_caps_by_net(pcb_data)
    gaps, uncovered, total = [], 0, 0
    for ref, rec in sorted(pins.items()):
        fp_ = (pcb_data.footprints or {}).get(ref)
        if fp_ is None:
            continue
        side = legality.footprint_side(fp_)
        for pad, _net in rec['pins']:
            total += 1
            _on, caps = decap_pin_caps({}, ref, side, pad, by_net)
            hit = nearest_rail_cap(pad, caps)
            if hit is None:
                uncovered += 1
            else:
                gaps.append(hit[0])
    # `max_raw` unrounded: `_ceil4` must see the true max, or rounding first
    # can land the limit BELOW it and fail the reference against itself.
    return {'pins': total, 'covered': len(gaps), 'uncovered': uncovered,
            'max_mm': round(max(gaps), 4) if gaps else None,
            'max_raw': max(gaps) if gaps else None}


def _decap_pin_derivation(pcb_data) -> Tuple[Optional[float], Optional[str]]:
    """`(max_pin_distance_mm, withheld_reason)` off a REFERENCE (#1102):
    `_ceil4` of the largest covered pin gap, the fixed point
    `_decap_derivation` argues for. Withheld when fewer than
    `DECAP_MIN_SAMPLE` pins are covered, or when more than
    `DECAP_MAX_CENSORED` of the graded pins have no cap on their net at all
    -- an uncovered pin contributes no distance, so a max over the rest would
    be censored (docs/floorplan-intent.md, "The emitter derives no pin
    limit")."""
    c = _pin_census_of(pcb_data)
    if c['covered'] < DECAP_MIN_SAMPLE:
        return None, (f"{c['covered']} supply pin(s) have a cap on their "
                      f"net (at least {DECAP_MIN_SAMPLE} needed)")
    if c['pins'] and c['uncovered'] / c['pins'] > DECAP_MAX_CENSORED:
        return None, (f"{c['uncovered']} of {c['pins']} supply pins have no "
                      f"cap on their net, so a max over the rest is censored")
    return _ceil4(c['max_raw']), None


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
        return None, (f"no decap is within {census['search_radius_mm']}mm of "
                      f"its owner, so nothing was measured"
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


def _emitted_basis(decaps, budget, conns, blocks,
                   assembly=None, band_default=(),
                   decaps_basis: Optional[str] = None,
                   pin_basis: Optional[str] = None) -> Dict[str, str]:
    """`{intent path: basis}` for every number the emitter chose: what it
    READ off this board is `observed_baseline`; a constant of this module is
    `derived_default`. The envelope RECT is not here -- it is the outline,
    a recorded fact, not a choice."""
    out: Dict[str, str] = {
        'envelope.tolerance_mm': 'derived_default',
        'defaults.zone_tolerance_mm': 'derived_default',
    }
    if 'max_distance_mm' in (decaps or {}):
        # #1099: read off a REFERENCE board, not this one, says so.
        out['decaps.max_distance_mm'] = decaps_basis or 'observed_baseline'
    if 'max_pin_distance_mm' in (decaps or {}) and pin_basis:
        out['decaps.max_pin_distance_mm'] = pin_basis
    if 'within_radius_refs' in (decaps or {}):
        # #1142: read off the same reference as the limit.
        out['decaps.within_radius_refs'] = decaps_basis or 'observed_baseline'
        out['decaps.within_radius_mm'] = 'derived_default'
    for k in sorted(budget or {}):
        out[f'legality_budget.{k}'] = 'observed_baseline'
    for c in conns or ():
        for k in ('edge', 'overhang_mm'):
            if k in c:
                out[f"edge_connectors[{c['ref']}].{k}"] = (
                    'derived_default' if k == 'overhang_mm'
                    and c['ref'] in band_default else 'observed_baseline')
    for b in blocks or ():
        for k in ('side', 'zone'):
            if k in b:
                out[f"blocks[{b['name']}].{k}"] = 'observed_baseline'
    if (assembly or {}).get('sides'):
        out['assembly.sides'] = 'observed_baseline'
    return out


#: #1155: the edge across the board from each edge.
_OPPOSITE_EDGE = {'north': 'south', 'south': 'north',
                  'east': 'west', 'west': 'east'}


def _emit_bodies_of(pcb_data, pcb_file: str, cache: Dict[str, object]):
    if 'bodies' not in cache:
        try:
            from .body import board_bodies
            cache['bodies'] = board_bodies(pcb_data, pcb_file)
        except Exception:                                    # noqa: BLE001
            cache['bodies'] = {}
    return cache['bodies']


def _part_facing(pcb_data, pcb_file: str, ref: str,
                 cache: Dict[str, object]) -> Optional[str]:
    """The edge `ref`'s mating face points at in its own pose
    (`edge_facing`), or None."""
    fp = (pcb_data.footprints or {}).get(ref)
    if fp is None:
        return None
    return edge_facing({}, fp, _emit_bodies_of(pcb_data, pcb_file,
                                               cache).get(ref))['facing']


def _withhold_reversed_edge(entry: Dict, pcb_data, pcb_file: str,
                            cache: Dict[str, object]) -> None:
    """#1155: drop an emitted entry's edge when the part's STRONG mating face
    points at another edge -- the opposite one (a reversed pose) or one at
    right angles (the part opens along the edge it sits nearest). "The
    emitted intent grades CLEAN by construction" (`emit_intent`) and must not
    bless the very pose the facing conjunct ERRORs, so no edge is declared
    and the entry says which of the two it measured.
    Measured on the 22 tracked boards and the fa10 designer boards: it never
    fires."""
    edge = entry.get('edge')
    fp = (pcb_data.footprints or {}).get(entry.get('ref'))
    if not edge or fp is None:
        return
    ef = edge_facing(entry, fp, _emit_bodies_of(pcb_data, pcb_file,
                                                cache).get(entry['ref']))
    if ef['fails'] and ef['strong']:
        # Run-5's suspect-and-derive: the EDGE is as uncertain as the pose
        # -- a part opening west that sits nearest east is either reversed
        # on the east edge or a west-edge part displaced -- so neither is
        # declared, and run-8 A3's machine-readable flag says why.
        if _OPPOSITE_EDGE.get(edge) == ef['facing']:
            how = (f"away from the {edge} edge it sits nearest: a reversed "
                   f"pose")
        else:
            how = (f"along the {edge} edge it sits nearest rather than out "
                   f"of it")
        why = (f"its mating face points {ef['facing']} ({ef['basis']}, "
               f"{ef['offset_mm']:.2f}mm), {how} (#1155)")
        entry.pop('edge')
        entry['suspect'] = True
        entry['suspect_reason'] = why
        note = f"edge withheld, SUSPECT: {why}"
        entry['note'] = (f"{entry['note']}; {note}" if entry.get('note')
                         else note)


def emit_intent(pcb_data, pcb_file: str, *,
                group_sources: Sequence[str] = ('kicad', 'sheet'),
                zone_pad_mm: float = 1.0,
                declare_classes: bool = False,
                derive_decaps='off', brief_fragment=None,
                decaps_from: Optional[str] = None,
                derive_arrays='off', clearance: Optional[float] = None,
                board_edge_clearance: Optional[float] = None) -> Dict:
    """A starter intent READ OFF the board, for a human or a model to edit.

    Everything here describes what the board already is. The envelope is
    `board_bounds` verbatim -- never a rounded or convenient rectangle -- and
    the cutouts are emitted as read-only `context` so an editor can see what the
    parts have to avoid without being able to mistake it for something to
    change. Nothing in this module writes Edge.Cuts.

    The emitted intent grades CLEAN by construction. That is the point: it is a
    baseline to tighten, and the round trip (emit then grade) is what proves the
    rules are wired to real geometry rather than silently skipping. The
    crystal claims it writes (`proximity[]`, `context.source`
    `placement.crystals`) keep it so by grading at WARN: their limits are the
    human corpus's 75th percentile, not this board's.

    It writes no `arrays` and no `fixed_poses` by default (#1051, #1054):
    which parts form a row and where a part must sit are design decisions,
    not observations.
    `derive_arrays='auto'` writes the row detector's suggestions
    (`arrays.suggest_arrays`, pose-blind) as `arrays` -- SUGGESTIONS, so
    unlike everything else here they need not grade clean. Its DROP step is
    not pose-blind (`_derived_arrays`): which members survive reads the
    emitter's pose-inferred zones and edge entries, so the no-human-layout-
    leak guarantee is the detector's, and the written rows carry a little
    of the board's current arrangement -- disclosed member by member in
    `context.arrays_dropped` / `arrays_edge_released`.

    It reads the board at the floors its grade will (`resolved_floors`:
    `clearance` / `board_edge_clearance` as given, else the board's own),
    and in the grade's currencies (#1301): a part is AT the edge when it
    comes within `part_class.SEAT_TOL_MM` of the outline or past it, the seat
    tolerance the grade gives a receptacle; its band is read at zero margin
    (`connector_geometry.courtyard_overhang`), `max` that overhang plus 0.5;
    and its edge at `edge_reach_mm`. It used to read all three off one
    occupancy reading at a fixed 0.55 edge clearance, so every emitted band
    sat 0.55 above the zero-margin grade, and the edge it read was not the
    one the grade read.
    """
    from .quench import QuenchState

    if derive_arrays not in ('off', 'auto'):
        raise ValueError(f"derive_arrays {derive_arrays!r}: expected 'off' "
                         f"or 'auto'")

    outline = outline_state(pcb_data, pcb_file)
    if not outline['trustworthy']:
        raise UntrustworthyOutline(outline['problems'])

    clearance, board_edge_clearance = resolved_floors(
        pcb_file, clearance, board_edge_clearance)
    state = QuenchState(pcb_data, pcb_file, clearance=clearance,
                        board_edge_clearance=board_edge_clearance)
    parts = {p.ref: p for p in state.graded_parts()}
    # AT the edge: within the grade's receptacle seat tolerance of the
    # outline, or past it -- a mechanical distance, so no DRC floor decides
    # which parts are observed there (watchy's side switches sit 0.21 mm
    # inside, ulx3s's B4-B6 0.41: at the edge clearance they were not).
    from .connector_geometry import courtyard_overhang, geometry_for
    from .part_class import SEAT_TOL_MM
    _at_edge_gate = legality.BoardOutlineGate(pcb_data.board_info, SEAT_TOL_MM)
    _overhang_geo = geometry_for(state, pcb_data, pcb_file)
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
    # #1103: on a PILE the poses are staging, not decisions -- run 37's
    # emit read 87 "edge connectors" off StickHub's staging ring (every cap,
    # U1, Y1), each one nearest SOME edge. Nothing read off an unlocked
    # part's pose is written then: no observed edge claim, no zone, no
    # oob_count baseline. A KiCad-locked part's pose IS a decision and keeps
    # its claims; a brief or mechanical.json still merge after this.
    from .placement_state import assess_placement as _assess_pile
    _pile_st = _assess_pile(pcb_data, pcb_file)
    # `s3_outside` too: a staging RING (run 36/37's pile) is spread, not
    # stacked, so it reads neither unplaced nor partially unplaced, yet
    # 93% of its parts sit off the board.
    # NOT any `partially_unplaced`: two stacked parts on a placed board set
    # it, and that board's other claims are real; a heap is at least half
    # stacked (run 29's pile: 83%).
    # The heap test counts `stacked_suspect_refs`, which already excuses the
    # far-side and marker co-locations a placed board has on purpose; the
    # raw `duplicate_fraction` does not (verifier: a placed orangecrab with
    # 60 parts under front-side parts read as a pile). One predicate, shared
    # with board_brief (#1109).
    _pile = _pile_st.pile
    _pile_locked = set(extract_locked_refs_safe(pcb_file)) if _pile else set()
    _pose_withheld: List[str] = []

    for key in sorted(cand):
        members, rect = cand[key]
        sides = {parts[r].side for r in members}
        entry = {
            'name': groups_mod.short_name(key),
            'group': groups_mod.short_name(key),
            'refs': sorted(members),
        }
        if key in zoned and _pile and not set(members) <= _pile_locked:
            entry['note'] = ('the board is a pile, so its members\' bounding '
                             'box is staging, not a zone (#1103)')
        elif key in zoned:
            entry['zone'] = emitted_zone_rect(rect, bounds)
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
    # the bodies, read once if a part reaching two sides asks which way it
    # faces (`_nearest_edge`), and again by #1155's pass below
    _emit_bodies: Dict[str, object] = {}
    #: #959: refs whose `overhang_mm` is a CONSTANT of this module (a class
    #: default band, the sanity cap, the affinity floor) rather than a band
    #: read off the part -- labelled `derived_default`, not an observation.
    band_default = set()
    for ref in sorted(parts):
        near = _at_edge_gate.rect_outside_amount(parts[ref].rect)
        # the overhang the grade reads its band on, at zero margin (#1301)
        amt = courtyard_overhang(_overhang_geo, parts[ref].rect)
        if _pile and ref not in _pile_locked:
            if near > legality.EPS:
                _pose_withheld.append(ref)
            continue
        if near > legality.EPS:
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
                    # the class band, widened to the observed overhang when
                    # that is plausible (under the sanity cap): a suspect
                    # pose's edge is withheld, but a card's fingers 1.7 mm
                    # past the edge are not damage (pcie_backplane J2)
                    band = default_band(pc.name, fp, 0.0 if over_cap else amt)
                    entry['overhang_mm'] = band
                    if band == default_band(pc.name, fp):
                        band_default.add(ref)
                elif over_cap:
                    entry['overhang_mm'] = {'min': 0.0,
                                            'max': round(_band_cap(ref), 3)}
                    entry['overhang_capped'] = True
                    entry['observed_overhang_mm'] = round(amt, 3)
                    band_default.add(ref)
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
                # NO `edge` key: the schema supports edge-less entries, and
                # their readers handle them by declining to guess. The band is
                # capped
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
                band_default.add(ref)
            elif pc is None or pc.name not in SEATED_CLASSES:
                # Only an edge part claims an edge: the placer seats only
                # those (`edge_seat.SEATED_CLASSES`), so a cap or a solder
                # jumper the designer put by the edge, declared on it, failed
                # its own claim as soon as it was placed freely (noahfc C6, C8).
                # The band stays, so the observed pose still grades clean.
                entry = {'ref': ref,
                         'overhang_mm': {'min': 0.0,
                                         'max': round(amt + 0.5, 3)},
                         'note': (f'overhang observed at {amt:.3f}mm; not an '
                                  f'edge part '
                                  f'({(pc and pc.name) or "no part class"}),'
                                  f' so no edge is claimed')}
            else:
                entry = {'ref': ref, 'edge': _nearest_edge(
                             parts[ref].rect, bounds, edge_reach_mm(state),
                             facing=lambda: _part_facing(
                                 pcb_data, pcb_file, ref, _emit_bodies)),
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
                body = _overhang_geo.measure(ref, entry['edge'])
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
        # and NO edge -- naming one would be an invention, and edge-less
        # entries claim no edge.
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
                band_default.add(ref)
                conns.append({
                    'ref': ref, 'class': pc.name, 'source': 'auto-class',
                    'overhang_mm': {'min': 0.0},
                    'note': ('connector-family part, no edge claim'
                             + ('' if (_pile and ref not in _pile_locked)
                                else f'; measured {clr:.2f}mm from the '
                                     f'nearest edge'))})
                continue
            if pc.name != 'edge_receptacle':
                # actuators make no claim unless they actually overhang
                # (handled above); nothing else is an edge class.
                continue
            clr = state.edge_gate.edge_clearance(parts[ref].rect)
            plaus = pose_plausible(pc.name, 0.0, clr)
            entry = {'ref': ref, 'class': pc.name, 'source': 'auto-class',
                     'overhang_mm': default_band(pc.name, fp)}
            band_default.add(ref)
            if _pile and ref not in _pile_locked:
                # #1103: a class is pose-free and stays; an edge read off a
                # staging pose would be an invention.
                entry['note'] = ('edge-receptacle class on a pile: no edge '
                                 'declared -- its staging pose says nothing')
            elif plaus:
                entry['edge'] = _nearest_edge(
                    parts[ref].rect, bounds, edge_reach_mm(state),
                    facing=lambda: _part_facing(pcb_data, pcb_file, ref,
                                                _emit_bodies))
            else:
                entry['note'] = (
                    f'edge-receptacle class in an implausible pose '
                    f'({clr:.2f} mm from the nearest edge, no overhang): '
                    f'no edge declared -- declare one, or seat it with place_pose')
            conns.append(entry)
    # #1155: ONE pass over every edge this emitter observed (both branches
    # above read it off `_nearest_edge`), before a brief is compiled in -- an
    # author's declared edge is never second-guessed here.
    for c in conns:
        _withhold_reversed_edge(c, pcb_data, pcb_file, _emit_bodies)

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
    _suspects = any('SUSPECT' in (c.get('note') or '') for c in conns)
    _budget = {}
    _withheld = {}
    try:
        from .legality import courtyard_severity_of as _cso
        _cy_ignored = _cso(pcb_file)[0] == 'ignore'
    except Exception:                                        # noqa: BLE001
        _cy_ignored = False
    # A pile's overlap is staging, as its off-board count is (#1103), so no
    # budget is read off it; and its body grade, which would only withhold the
    # budget again, is not paid for (glasgow_revC: 9 of a 14 s emit).
    _pile_overlap = _pile and not _cy_ignored
    _body_blocking = 0
    _courtyard_blocking = 0
    if not _pile_overlap:
        try:
            from placement.legality import grade_body_overlap
            _g_overlap = grade_body_overlap(
                pcb_data, state.clearance, pcb_file=pcb_file)
            _body_blocking = _g_overlap['blocking']
            _courtyard_blocking = _g_overlap.get('courtyard_blocking', 0)
        except Exception:
            _body_blocking = 0
            _courtyard_blocking = 0
    if _pile_overlap:
        _withheld['overlap_area'] = (
            'the board is a pile: its overlap is staging, not a baseline '
            '(#1103)')
    elif _cy_ignored:
        # #1101 review: the budget is a courtyard OVERLAP AREA, and the
        # project waives courtyard overlap -- courtyards may pack there, so
        # a budget read off this board would fail its own placement.
        _withheld['overlap_area'] = (
            "the project sets courtyards_overlap to ignore, so a courtyard "
            "overlap budget grades nothing it asks for")
    elif _body_blocking:
        _withheld['overlap_area'] = (f'{_body_blocking} blocking body '
                                     f'pair(s) on the emitting board (run-6)')
    elif _courtyard_blocking:
        _withheld['overlap_area'] = (
            f'{_courtyard_blocking} unwaived courtyard interpenetration(s) '
            f'past the blocking floors on the emitting board (run-23): an '
            f'auto-budget would bless them')
    else:
        # #1162: the reading `rule_legality` grades -- check_assembly's
        # outlines, in the budget's universe -- or the emitted budget is a
        # number in another currency: too tight where the outlines read
        # more than the rects, too loose where they read less (glasgow g4:
        # 4.909 on rects, 0.595 on outlines).
        try:
            from types import SimpleNamespace as _SN
            _ex = _ctx_overlap_exact(_SN(
                state=state, pcb=pcb_data, pcb_file=pcb_file,
                parts={p.ref: p for p in state.graded_parts()},
                _census=None, _overlap_exact=None))[0]
        except Exception:                                    # noqa: BLE001
            _ex = float(leg['overlap_area'])
        _budget['overlap_area'] = _ceil4(float(_ex))
    if _pile and int(leg['oob_count']):
        _withheld['oob_count'] = (
            'the board is a pile: its off-board count is staging, not a '
            'baseline (#1103)')
    elif not _suspects:
        _budget['oob_count'] = int(leg['oob_count'])
    else:
        _withheld['oob_count'] = (
            'an edge connector on the emitting board sits in a SUSPECT pose '
            '(see its edge_connectors note), so its overhang is not a '
            'baseline to bless')

    # #704. The census runs on EVERY emission: a reader of the document must
    # be able to tell "no cap is far from its IC" from "nobody measured", and
    # `decaps: {}` alone says only the second. The LIMIT is opt-in, because
    # declaring it is not a grading-only change: a tool holding the limit
    # (place_fanout_clearance --intent) moves caps by it.
    # #959: caps the BRIEF's own proximity relations supersede are graded by
    # those relations where the brief is read -- and DISCLOSED here. The
    # limit is still read off every cap: a tool that grades without the
    # brief would otherwise fail a limit tightened by leaving the superseded
    # caps out, on the board it was emitted from (Phase-6 verifier).
    _sup = (superseded_caps(pcb_data, (brief_fragment or {}).get('proximity'),
                            brief_fragment) if brief_fragment else {})
    _census = decap_census(pcb_data)
    if _sup:
        _census['superseded'] = dict(sorted(_sup.items()))
    _decaps: Dict[str, object] = {}
    _mode = _decap_mode(derive_decaps)
    _derive = _mode == 'strict'
    if _mode == 'auto':
        # #959 (#1002): derive only off a PLACED board. Run 29's pile read
        # `unplaced: false` and `partially_unplaced: true` (duplicate
        # fraction 0.833), and a strict derivation there wrote a limit of 0.0.
        # `.pile` (#1109) catches it -- its heap arm is that 83% -- and,
        # unlike any `partially_unplaced`, does not discard a placed board
        # over two parts sharing an origin by design (#1178).
        from .placement_state import assess_placement
        _st = assess_placement(pcb_data, pcb_file)
        if _st.pile:
            _census['auto_withheld'] = (
                'the board is not placed ('
                + '; '.join(_st.reasons[:2])
                + '): a limit read off it would bless a pile -- run 29\'s '
                  'read 0.0')
        else:
            _derive = True
    if decaps_from:
        # #1099: the limit read off a PLACED REFERENCE -- a human layout or
        # an earlier one of this design -- instead of the board being
        # emitted, which on a pile has nothing to read (auto withholds;
        # strict wrote 0.0 on run 29). Same derivation, same withholding;
        # the basis names the file, and it replaces `derive_decaps`.
        from kicad_parser import parse_kicad_pcb as _parse_ref
        from .placement_state import assess_placement as _assess
        _ref_pcb = _parse_ref(decaps_from)
        _ref_census = decap_census(_ref_pcb)
        _limit, _why = _decap_derivation(_ref_census)
        # #1175: the reference is NAMED, as the basis already names it, and
        # identified by its bytes -- never pointed at. The reference is
        # typically a human layout, and an intent carrying its path hands a
        # run that grades against it the one file it must not read.
        import hashlib
        _ref_name = os.path.basename(decaps_from)
        _census['reference_board'] = _ref_name
        with open(decaps_from, 'rb') as _rf:
            _census['reference_board_sha256'] = hashlib.sha256(_rf.read()).hexdigest()
        _census['reference_tethers'] = _ref_census.get('tethers')
        # The reference REPLACES the auto/strict arms: an auto withholding
        # about THIS board is not a statement about the number written.
        _census.pop('auto_withheld', None)
        # The reference must BE a placement of THIS design: its pad-bearing
        # parts on the board with the same footprint (#1099 verifier: an
        # esp_prog intent took glasgow's 4.787 mm without a word), and
        # placed (a pile blesses nothing -- auto refuses the same board).
        # In BOTH directions: the lower of the two shares is the match, so a
        # small board of generic passives is not a placement of a large one.
        def _parts(pcb):
            return {(r, f.footprint_name) for r, f in
                    (pcb.footprints or {}).items() if non_aperture_pads(f)}
        _theirs, _ours = _parts(_ref_pcb), _parts(pcb_data)
        _common = len(_theirs & _ours)
        _match = (min(_common / len(_theirs), _common / len(_ours))
                  if _theirs and _ours else 0.0)
        _census['reference_part_match'] = round(_match, 3)
        _ref_state = _assess(_ref_pcb, decaps_from)
        if _match < DECAPS_FROM_MIN_MATCH:
            _limit, _why = None, (
                f"it is not a placement of this design: they share "
                f"{_common} part(s) under the same reference and footprint, "
                f"{_common}/{len(_theirs)} of its parts and "
                f"{_common}/{len(_ours)} of this board's (at least "
                f"{DECAPS_FROM_MIN_MATCH:.0%} of each needed)")
        elif _ref_state.pile:
            # #1178: the ONE pile test (#1109), as the emitter's own pile
            # gate reads it. Not any `partially_unplaced`: two parts sharing
            # an origin by design set it on a placed reference -- a
            # Tag-Connect over a SOT-666 on the far face (rp2350), the two
            # halves of a CM5 module footprint -- and discarded the whole
            # reference over them.
            _limit, _why = None, (
                "it is not placed (" + '; '.join(_ref_state.reasons[:2])
                + "): a limit read off it would bless a pile")
        _ref_ok = (_match >= DECAPS_FROM_MIN_MATCH
                   and not _ref_state.pile)
        if _ref_ok and _ref_state.stacked_suspect_refs:
            # Read as placed, and SAID: the limit is the reference's own
            # fixed point (`_decap_derivation`), so it is read off every
            # part -- leaving these out would leave caps the reference
            # holds out of the max the reference must then pass.
            _census['reference_stacked_refs'] = list(
                _ref_state.stacked_suspect_refs)
        if _limit is None:
            _withheld['decaps.max_distance_mm'] = (
                f"the reference board {_ref_name}: {_why}")
        else:
            _decaps['max_distance_mm'] = _limit
            _census['emitted_max_distance_mm'] = _limit
            _census['decaps_basis'] = ('reference:'
                                       + _ref_name)
            # #1102: the tether currency is the cap to the chip's inflated
            # PAD BBOX, and it stops at the 5 mm search radius -- a cap
            # left 10 mm away is only `decap_ungraded`, a WARN. A cap the
            # reference keeps INSIDE the radius and this board leaves beyond
            # it is a regression, not a horizon: run 37 stranded C3, C7, C12
            # at 7.8-10.2 mm with no error.
            # #1142: PER CAP. #1102 promoted the rule board-wide, and only
            # when the reference kept EVERY rail cap inside -- one bulk cap
            # beyond 5 mm switched it off for the whole board, and none of
            # the seven #1105 references passed. The list names the caps the
            # reference keeps within the radius (same ref, same footprint on
            # this board); a cap it keeps beyond stays a WARN.
            _r = groups_mod.DECAP_RADIUS_MM
            _near_ref = groups_mod.decap_populations(_ref_pcb, radius=_r)[0]
            _ours_fp = {r: f.footprint_name
                        for r, f in (pcb_data.footprints or {}).items()}
            _held = sorted(
                c for caps in _near_ref.values() for c, _d in caps
                if c in _ours_fp
                and _ours_fp[c] == _ref_pcb.footprints[c].footprint_name)
            if _held:
                _decaps['within_radius_refs'] = _held
                _decaps['within_radius_mm'] = _r
                _ref_beyond = sorted(
                    _ref_census.get('beyond_radius_refs') or ())
                _census['reference_beyond_radius_refs'] = _ref_beyond
                _census['decap_ungraded_promoted'] = (
                    f"per cap, for the {len(_held)} cap(s) the reference "
                    f"keeps within {_r:g} mm of their chip"
                    + (f"; the {len(_ref_beyond)} it keeps beyond "
                       f"({', '.join(_ref_beyond)}) stay warn"
                       if _ref_beyond else '')
                    + "; a cap the reference lacks, or carries under another "
                      "footprint, stays warn")
        # #1102, the PIN currency: every supply pin to the nearest cap on its
        # net, pad edge to pad edge (`decap_pin_distance`, #705). A cap 1.6
        # mm from a QFP's box can be 5 mm from the pin it decouples; the
        # tether limit cannot see that, the pin limit can. Derived only from
        # a REFERENCE: off the board being graded it would be vacuous (the
        # board grades clean against its own max by construction).
        _pin_limit, _pin_why = (None, "the reference is not usable (above)"
                                ) if not _ref_ok else \
            _decap_pin_derivation(_ref_pcb)
        _census['reference_pin_census'] = _pin_census_of(_ref_pcb) \
            if _ref_ok else None
        if _pin_limit is None:
            # A census NOTE, not `budget_withheld`: nothing asked for the pin
            # rule, so withholding it must not leave a clean board "not fully
            # graded" (#1102 verifier: esp_prog graded against its own
            # --decaps-from intent went pass/exit 0 -> exit 4).
            _census['pin_limit_withheld'] = (
                f"the reference board {_ref_name}: {_pin_why}")
        else:
            _decaps['max_pin_distance_mm'] = _pin_limit
            _census['emitted_max_pin_distance_mm'] = _pin_limit
        _derive = False
    if _derive:
        _limit, _why = _decap_derivation(_census)
        if _limit is None:
            if _mode == 'auto':
                # Recorded in the census, which the emit prints, NOT in
                # budget_withheld: `auto` promises an emit that changes no
                # exit code. The dark decap rule is still owed a
                # disposition; the census is the reason, not the answer.
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
    # The census reports the caps no limit grades by CAUSE: beyond the
    # search radius, or on a rail no chip carries. Measured over every
    # tracked board, the "is this a decap" predicates name identical sets, so
    # nothing is attributable to them: ulx3s's 17 ungraded caps are 7 beyond
    # the radius plus 10 whose rail no chip carries, and `unaccounted` stays
    # 0 to say the three arms are the whole scope.

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
    doc = {
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
        # No top-level `severity`: #1102 wrote `decap_ungraded: error` here;
        # #1142 holds caps PER CAP through `decaps.within_radius_refs`
        # instead, and an explicit severity would override that list.
        # must_lock is a REQUIREMENT ("these refs must end up locked"), and an
        # emitted intent describes a board rather than making demands of it.
        # Filling it with the board's own locked set would turn an
        # observation into a demand; the observation is still worth
        # recording, and it belongs in `context`, which nothing acts on.
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
            # #1103: what a PILE withheld, and why -- kept out of
            # budget_withheld so an emit changes no exit code.
            **({'pose_claims_withheld': {
                'reason': ('the board is a pile ('
                           + '; '.join(_pile_st.reasons[:2])
                           + '): staging poses are not claims'),
                'withheld': ['edge_connectors[].edge read off a pose',
                             'blocks[].zone',
                             'legality_budget.oob_count'],
                'refs_off_board': sorted(_pose_withheld),
                'kept': ('KiCad-locked parts; a brief / mechanical.json '
                         'merged after this')}}
               if _pile else {}),
            # #959 comment 3.2: every number this emitter chose, labelled as
            # what it is -- a baseline OBSERVED on this board, not a
            # requirement anyone declared. Keyed by intent path; a brief
            # merged over it re-labels what it declares.
            'basis': _emitted_basis(
                _decaps, _budget, conns, blocks, _assembly, band_default,
                decaps_basis=(_census.get('decaps_basis')
                              if str(_census.get('decaps_basis') or '')
                              .startswith('reference:') else None),
                pin_basis=('reference:' + os.path.basename(decaps_from)
                           if decaps_from else None)),
        },
    }
    if derive_arrays == 'auto':
        doc['arrays'], dropped, released = _derived_arrays(
            pcb_data, pcb_file, conns, blocks)
        if released:
            # The array wins over an emitter-INFERRED edge claim on a part
            # the classifier calls no connector (see `_derived_arrays`); the
            # claim is removed with its basis keys and disclosed, never
            # silently.
            gone = {r['ref'] for r in released}
            conns[:] = [c for c in conns if str(c.get('ref')) not in gone]
            basis = doc['context'].get('basis') or {}
            for k in [k for k in basis
                      if any(k.startswith(f"edge_connectors[{g}].")
                             for g in gone)]:
                del basis[k]
            doc['context']['arrays_edge_released'] = released
        doc['context']['arrays_note'] = (
            'SUGGESTED by the pose-blind array detector '
            '(`check_floorplan --suggest-arrays` shows the evidence), not '
            'read off the board: accept, edit or delete each. A suggested '
            'row need not be one the board already forms, so unlike the '
            'rest of this file it may not grade clean')
        if dropped:
            doc['context']['arrays_dropped'] = dropped
        doc['min_reader'] = max(int(doc.get('min_reader') or 0), 7)
    # #1142: a reader-7 build would refuse the held list by name anyway; the
    # stamp says which build can act on it.
    if 'within_radius_refs' in (doc.get('decaps') or {}):
        doc['min_reader'] = max(int(doc.get('min_reader') or 0), 8)
    # The crystal claims place_simple would derive on its own
    # (`crystals.crystal_claims`: each crystal at its IC's oscillator pins,
    # through a series part where there is one, and each load cap at its
    # crystal), written down so the grade and the intent's reader see them. They
    # read no pose. A pair the brief already relates, either way round,
    # keeps the brief's row; graded at WARN (`derived_claim`).
    from .crystals import crystal_claims
    taken = set()
    for c in (brief_fragment or {}).get('proximity') or ():
        taken |= {(str(c.get('ref')), str(c.get('near'))),
                  (str(c.get('near')), str(c.get('ref')))}
    xrows = []
    for r in crystal_claims(pcb_data)[0]:
        if (r['ref'], r['near']) in taken:
            continue
        taken.add((r['ref'], r['near']))
        xrows.append(r)
    if xrows:
        doc['proximity'] = list(doc.get('proximity') or []) + xrows
        doc['min_reader'] = max(int(doc.get('min_reader') or 0), 12)
    return doc


def _derived_arrays(pcb_data, pcb_file: str, conns, blocks
                    ) -> Tuple[List[Dict], List[Dict], List[Dict]]:
    """`(arrays, dropped, edge_released)`: the suggestions as intent entries.

    Only what the loader and `array_problems` would accept is written, so
    `derive_arrays='auto'` never emits an intent that refuses itself. A
    member is dropped -- and said so in `dropped`, never silently -- when it
    is locked in the file or outside the zoned block most of its row sits
    in (a row is seated as one piece and cannot straddle zones). A row left
    with fewer than `arrays.MIN_MEMBERS` is dropped whole. `order: "pin"` is
    re-derived over the survivors, because `pin_order` reads which nets are
    a member's OWN among the members given.

    A member the emitter ALSO wrote as an `edge_connectors` entry keeps its
    row, and the edge entry goes (`edge_released`). Every such entry is
    INFERRED from the part's current overhang -- the brief's declared edges
    are merged later, in the CLI -- and every detector member is a part
    `part_class` calls no connector, switch or test point
    (`arrays.not_a_member`), so the edge claim is a resistor or a cap
    overhanging the outline: watchy R4/R8/R11 (100K, 0.87-1.39mm over a
    curved edge) were dropped from `U4:100K` for it. Seating a resistor at
    the edge as a connector is the false claim; the overhang itself is then
    graded as what it is.

    THIS STEP READS POSES, where the detector does not: the locked set is a
    file fact, but the edge entries are inferred from the current overhang
    and the zones from the current bounding boxes. So the rows written can
    differ between a board and a re-posed copy of it even though
    `suggest_arrays` returns the same candidates; `context.arrays_dropped`
    and `arrays_edge_released` say exactly where.
    """
    from . import arrays as arr
    fps = pcb_data.footprints or {}
    locked = set(extract_locked_refs_safe(pcb_file)) | {
        r for r, fp_ in fps.items() if getattr(fp_, 'locked', False)}
    edge = {str(c.get('ref')): c for c in conns or ()}
    zone_of: Dict[str, str] = {}
    for b in blocks:
        if b.get('zone') is not None:
            for r in b.get('refs') or ():
                zone_of.setdefault(r, b['name'])
    out: List[Dict] = []
    dropped: List[Dict] = []
    released: List[Dict] = []
    for c in arr.suggest_arrays(pcb_data):
        members = list(c['members'])
        why_out: Dict[str, str] = {}
        for m in members:
            if m in locked:
                why_out[m] = 'locked in the board file'
        keep = [m for m in members if m not in why_out]
        zones: Dict[Optional[str], List[str]] = {}
        for m in keep:
            zones.setdefault(zone_of.get(m), []).append(m)
        if len(zones) > 1:
            best = min(zones, key=lambda z: (-len(zones[z]),
                                             arr.natural_key(zones[z][0]),
                                             zones[z][0]))
            for z, refs in zones.items():
                if z != best:
                    for m in refs:
                        why_out[m] = (f"in zoned block {z!r}" if z else
                                      "outside the row's zoned block")
            keep = zones[best]
        if c['order'] == 'pin' and why_out:
            keep, unres = arr.pin_order(pcb_data, str(c['serves']), keep)
            for m, w in unres.items():
                why_out[m] = w
        if why_out:
            dropped.append({'name': c['name'],
                            'members': dict(sorted(why_out.items())),
                            'row_dropped': len(keep) < arr.MIN_MEMBERS})
        if len(keep) < arr.MIN_MEMBERS:
            continue
        for m in keep:
            if m in edge:
                e = edge[m]
                released.append({
                    'ref': m, 'array': c['name'], 'edge': e.get('edge'),
                    'overhang_mm': e.get('overhang_mm'),
                    'why': ('an edge claim the emitter inferred from the '
                            'overhang of a part no classifier calls a '
                            'connector; the array keeps it')})
        entry = {'name': c['name'], 'members': keep}
        if c['serves']:
            entry['serves'] = c['serves']
        entry.update({'order': c['order'], 'rotation': c['rotation'],
                      'pitch_mm': c['pitch_mm'], 'axis': c['axis'],
                      'why': arr.suggestion_why(c)})
        out.append(entry)
    return out, dropped, released


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
    # #1064: printed, never a rule -- a floorplan PASS says nothing about
    # whether two parts' pads sit on each other, and check_assembly calls
    # that NOT BUILDABLE whatever the nets.
    if r.pad_stacks is not None:
        _ps1064 = r.pad_stacks
        lines.append(
            f"  pad stacks: {_ps1064['pad_stack_count']} (two parts' pad "
            f"copper overlapping, any net -- check_assembly's "
            f"pad_intersection, exact; printed, not a floorplan rule)"
            + (" -- check_assembly grades these NOT BUILDABLE"
               if _ps1064['pad_stack_count'] else ''))
        for a, b, area, side in _ps1064['pad_stack_pairs'][:5]:
            lines.append(f"    {a} <-> {b}  {area:.4f}mm2  side {side or '-'}")
        if _ps1064['pad_stack_count'] > 5:
            lines.append(f"    ... {_ps1064['pad_stack_count'] - 5} more")
    # #1051: every declared array, formed or not, with what its order could
    # not place -- a partly resolved pin order is disclosed on a pass too.
    for a in r.array_measured:
        state = ('skipped' if a.get('formed') is None
                 else 'formed' if a['formed'] else 'NOT formed')
        un = a.get('order_unresolved') or {}
        lines.append(
            f"  array {a['name']}: {state}"
            + (f" ({a['skipped']})" if a.get('skipped') else '')
            + (f"; unchecked: {', '.join(a['unchecked'])}"
               if a.get('unchecked') else '')
            + (f"; order unresolved for {', '.join(sorted(un))}"
               if un else ''))
    # #1253: every declared keep-away, passing ones included, with the
    # closest pad pair and both sides -- and once, what none of them sees.
    for k in r.keep_away_measured:
        head = f"  keep-away {k['rule']}: "
        c = k.get('closest')
        if c:
            v, a = c['victim'], c['aggressor']
            lines.append(
                head + f"closest {k['min_gap_mm']:.2f}mm, {v['ref']} pad "
                f"{v['pad']} ({v['side']}) to {a['ref']} pad {a['pad']} "
                f"({a['side']}), "
                + (f"{k['part_pairs_inside']} part pair(s) INSIDE the "
                   f"{k['gap_mm']:.2f}mm band" if k.get('part_pairs_inside')
                   else f"outside the {k['gap_mm']:.2f}mm band")
                + (" -- opposite faces, plan view" if c['cross_face'] else ''))
        else:
            lines.append(head + f"not measured ({k.get('state')})")
        for note in k.get('notes') or ():
            lines.append(f"    note: {note}")
    if r.keep_away_measured:
        lines.append("  keep-away is graded on pad copper in plan view; it "
                     "cannot see " + '; '.join(KEEP_AWAY_NOT_MEASURED))
    # #1211: every declared copy target, passing ones included.
    for m in r.replicate_measured:
        if m.get('state') == 'unresolved':
            lines.append(f"  replicate[{m['index']}] {m['from']} -> "
                         f"{', '.join(m['to'])}: not measured")
            continue
        lines.append(
            f"  replicate[{m['index']}] {m['from']} -> {m['to']}: "
            + ('a copy' if m['passes'] else
               f"{m['off']} of {m['pairs']} part(s) off")
            + f", max {m['max_residual_mm']:.2f}mm from the best translation "
              f"({m['fit_offset'][0]:+.2f}, {m['fit_offset'][1]:+.2f})"
            + (f", {len(m['left_in_place'])} left out by the declaration"
               if m.get('left_in_place') else ''))
    for m in r.template_measured:
        if m.get('state') == 'unresolved':
            lines.append(f"  templates[{m['index']}] {m['anchor']}: not measured")
            continue
        w = m['members'][0] if m['members'] else None
        lines.append(f"  templates[{m['index']}] {m['anchor']}: "
                     + ('as drawn' if m['passes'] else
                        f"{m['off']} of {len(m['members'])} member(s) off")
                     + (f", worst {w['ref']} {w['worst_mm']:.2f}mm" if w else ''))
        g = m.get('ground_return')
        if g and g['caps']:
            c = max(g['caps'], key=lambda c: c['return_mm'])
            lines.append(f"    ground return: {len(g['caps']) - g['over']} of {len(g['caps'])} "
                         f"cap(s) within {g['limit_mm']:g}mm of the chip's ground, longest "
                         f"{c['ref']} {c['return_mm']:.2f}mm")
    rows = [e for e in r.edge_seating
            if e.get('along_edge_offset_mm') is not None]
    if rows:
        rows.sort(key=lambda e: -abs(float(e['along_edge_offset_mm'])))
        lines.append("  along-edge seating (measured, not a verdict -- a "
                     "violation needs a declared center_on_edge or "
                     "along_edge_band):")
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
                + f" vs {cu.get('required_mm')}mm {cu.get('disposition')}"
                + _facing_tail(e.get('facing')))
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
                    f"        try: place_pose.py {os.path.basename(r.board)}"
                    f" OUT set {a['ref']} --near X Y, X Y at the net's "
                    "centroid")
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
    absent: set = set()
    # #1051: a formation finding names the ARRAY (its block), not a ref, so a
    # brief `arrays[NAME]` clause is judged by the array's name -- the
    # keep-out case above, one rule over.
    arr_err: set = set()
    # #1054: a fixed pose's anchor finding is a `zone_containment` on a
    # COMPILED anchor block (`fixed:<ref>`, or `mech:<ref>` when the file's
    # anchor grades it) -- not any zone the ref happens to fail.
    anchor_err: set = set()
    # #1253: a keep-away finding names a PART PAIR, never the clause, so a
    # brief `keep_away[N]` clause is judged by its RULE text, which every
    # finding carries; WARN is its default, so a warn is not a pass.
    ka_err: set = set()
    ka_warn: set = set()
    if result is not None:
        for v in result.violations:
            if v.rule in ('keep_away', 'keep_away_unresolved'):
                (ka_err if v.severity == ERROR else ka_warn).add(
                    str((v.measured or {}).get('rule') or ''))
            if v.severity == ERROR:
                by_rule_err.setdefault(v.rule, set()).add(v.ref or '')
                if v.rule == 'keepout':
                    ko_err.add(str((v.measured or {}).get('keepout') or ''))
                if v.rule in ('array_formation', 'array_unresolved',
                              'array_conflict'):
                    arr_err.add(v.block or '')
                if v.rule == 'zone_containment' and str(
                        v.block or '').startswith((FIXED_POSE_ANCHOR_PREFIX,
                                                   MECHANICAL_ANCHOR_PREFIX)):
                    anchor_err.add(v.ref or '')
            if v.rule == 'edge_connector_side':
                side_warn.add(v.ref or '')
            if v.rule == 'edge_connector' and (v.measured or {}).get(
                    'found') is False:
                absent.add(v.ref or '')

    def _verdict(grader, ref, keepout=None):
        if result is None:
            return 'pending'
        if ref and ref in absent:
            # A connector not on the board fails every clause about it --
            # the side finding, which only WARNs, would otherwise read a
            # missing part as a pass (round-2 verifier).
            return 'graded_fail'
        if grader == 'keepout':
            return 'graded_fail' if keepout in ko_err else 'graded_pass'
        if grader == 'array_formation':
            return 'graded_fail' if ref in arr_err else 'graded_pass'
        if grader == 'fixed_pose':
            # #1054: the anchor's finding is a `zone_containment` on the
            # ref's ANCHOR block (a plain zone the ref fails is its block's
            # clause, not this one); a pose for a ref the board lacks is
            # `fixed_pose_unresolved`.
            hit = ((ref or '') in anchor_err
                   or (ref or '') in by_rule_err.get('fixed_pose_unresolved',
                                                     set()))
            return 'graded_fail' if hit else 'graded_pass'
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
        if st == 'graded' and c.get('kind') == 'keep_away':
            spec = str(c.get('keep_away_rule') or '')
            status = ('pending' if result is None
                      else 'graded_fail' if spec in ka_err
                      else 'graded_warn' if spec in ka_warn
                      else 'graded_pass')
        elif st == 'graded':
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
    # part will go) and failed after one; drift a plan loses is failed;
    # a floors row is reported, not graded.
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
        # A loser is a channel whose value DIFFERS from the winner's: one
        # that agrees with it lost nothing (a brief pose corroborating the
        # mechanical one read `graded_fail` under --plan-only, Phase-1
        # re-verifier). Poses compare by `reconcile.same_pose`.
        from .reconcile import same_pose as _same_pose

        def _differs(v, w=wv.get('value'), fld=r.get('field')):
            if w is None:
                return True
            if fld == 'pose':
                return not _same_pose(v, w)
            return v != w
        losers = {v.get('authority') for ch, v in (r.get('values')
                                                   or {}).items()
                  if ch != winner and v.get('value') is not None
                  and _differs(v.get('value'))}
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
        # did not measure everything" (route_summary.py), reused rather than
        # a second channel invented.
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
        **({'array_formation': r.array_measured} if r.array_measured
           else {}),
        **({'keep_away': r.keep_away_measured} if r.keep_away_measured
           else {}),
        **({'replicate': r.replicate_measured} if r.replicate_measured
           else {}),
        **({'templates': r.template_measured} if r.template_measured
           else {}),
        **({'pad_stacks': r.pad_stacks} if r.pad_stacks is not None
           else {}),
    }


def summary(r: GradeResult) -> Dict:
    """The flat JSON_SUMMARY dict."""
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
    # #1064: the EXACT stack count, beside (not instead of) the bounding-box
    # `pad_intersection_pairs` r.legality just contributed.
    if r.pad_stacks is not None:
        out['pad_stack_count'] = r.pad_stacks['pad_stack_count']
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


