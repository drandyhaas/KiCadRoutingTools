"""The declared design brief: what a board is SUPPOSED to be (#711).

Placement infers everything about a board from the board. `emit_intent` says so
in its own docstring -- "a starter intent READ OFF the board ... grades CLEAN by
construction" -- and every connector's edge is guessed from its current pose by
`_nearest_edge`, which is the only source of an edge anywhere in this toolchain.
There is no channel through which a human or a model can state what the board is
FOR, and the one prose channel that exists, `board_brief.py --requirements`, is
carried verbatim and read by nothing.

This module is that channel. A `<board>.design-brief.json` sibling holds the
facts a board file cannot contain -- which connectors are user-facing, which
edge each belongs on and where along it, what the enclosure forbids -- and
`compile_brief` turns them into the EXISTING floorplan intent schema.

Three design rules, each a decision rather than an omission:

* IT IS A COMPILER, NOT A SECOND CONSTRAINT SYSTEM. Everything a brief declares
  becomes an intent entry the existing rules already grade and the existing
  seat search already honours -- ordinarily an `edge_connectors[]` or a
  `keepouts[]` one. Provenance goes in `source` and `context`, which the schema
  already accepts. A brief that could express something the intent cannot would
  be a constraint nothing checks.

  #712 added two intent FIELDS and no key. #902 adds one KEY, `proximity[]`,
  and that is the rule applied rather than an exception to it: the principle is
  not "never add a key", it is "never declare what nothing grades". So this
  module is deliberately only HALF of #902 -- the other half is the rule that
  grades the compiled rows, in `floorplan`, and a build carrying this key
  without that rule declares something nothing measures. `tests/
  test_902_proximity.py` holds a row that reports exactly which of the two
  states the tree is in, rather than passing either way.

  It needed a key of its own because no existing entry means *these two named
  parts, this far apart*. `decaps.max_distance_mm` is one board-wide
  budget over a DERIVED population whose partner election cannot reach a 3-pad
  regulator at any radius (#902's own measurement), and a keep-out is an
  exclusion, not an attraction -- compiling into either would have graded a
  different claim from the one written.

* "I DO NOT KNOW" IS A VALUE, and it is distinct from "nobody said". The
  failure this is designed against is a brief nobody writes, so `"unknown"`
  (the author looked and the answer does not exist yet) and an absent key
  (nobody looked) are reported apart, everywhere. That is `rules_run` vs
  "0 violations" one level up, and `context.keepouts_note` one level down.

* THE BOARD IS NOT THE AUTHORITY. Where a brief and the emitter disagree the
  brief wins, and the disagreement is REPORTED rather than resolved silently.
  That is the whole point: an intent derived from a board can only ever
  re-state that board, including its damage.

Two things the brief deliberately CANNOT say, refused by name at load rather
than ignored, because an author who wrote one believes it is being honoured:

* an envelope or outline -- `floorplan`'s module docstring is the rule, the
  board outline is not ours to change;
* a height limit -- nothing in the placement stack measures z (no height in
  `legality.GradedPart`, none in the parser), so a declared limit would grade
  nothing at all. A constraint the author believes they set and the grader
  never checks is the exact failure `_reject_unknown` exists to prevent.
"""
from __future__ import annotations

import fnmatch
import glob
import json
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from . import floorplan as fp

SCHEMA_VERSION = 1
KIND = 'design-brief'

#: What this build can ACT on -- the same two-number policy `floorplan` uses:
#: `schema` is the format and is matched exactly, `min_reader` is the field
#: vocabulary. Copied rather than reinvented so one rule covers both artifacts.
MIN_READER = 1

#: Sibling of the board. NOT a `.kicad_*` extension -- those belong to KiCad --
#: and NOT `<board>.brief.json`, which would sit one filename away from
#: `board_brief.py --json`'s output. That collision is also refused by `kind`
#: below, because a filename convention is not a guard.
BRIEF_EXT = '.design-brief.json'

#: The author looked and the answer does not exist yet. Distinct from an absent
#: key, which means nobody looked.
UNKNOWN = 'unknown'


class BriefError(fp.IntentError):
    """A document that cannot be read as a design brief.

    Subclasses `IntentError` (itself a `ValueError`) so every CLI's existing
    `except (OSError, ValueError)` already turns a bad brief into exit 2, with
    no new error plumbing at six call sites.
    """


_TOP_LEVEL_KEYS = {'schema', 'kind', 'board', 'units', 'min_reader',
                   'product', 'interfaces', 'keepouts', 'fixed', 'unknown',
                   'proximity', 'context'}

#: Keys refused BY NAME, with the reason, rather than as merely unknown.
_REFUSED_TOP_LEVEL = {
    'envelope': 'the board outline is READ from the board, never authored -- '
                'a part outside it is a finding about the PART',
    'outline': 'the board outline is READ from the board, never authored',
    'board_size': 'board size is a mechanical decision this toolchain does '
                  'not make; nothing here writes Edge.Cuts',
    'height': 'nothing in the placement stack measures height -- there is no '
              'z in legality.GradedPart and none in the parser -- so a '
              'declared limit would grade nothing, which is worse than not '
              'declaring it',
}

_PRODUCT_KEYS = {'form_factor', 'primary_axis', 'held_by', 'user_top_side'}
_INTERFACE_KEYS = {'ref', 'role', 'user_facing', 'edge', 'along_edge',
                   'along_edge_tolerance_mm', 'overhang_mm', 'mount_mode',
                   'cable_entry', 'cable_envelope_mm', 'requirement', 'why',
                   'note', 'context'}
#: #959 (#1000): the space a cable needs, declared. `depth` is how far an
#: in-plane plug reaches in from the edge; `clear` is the margin around a
#: perpendicular one. There are NO defaults: measured on five as-built boards
#: (Phase-0 P3), no clearance passed every control and the in-plane band was
#: vacuous, so an undeclared envelope is reported unmeasured, never guessed.
_ENVELOPE_KEYS = {'depth', 'clear'}
#: The intent's own keep-out shape, plus two slots for reasoning that has
#: nowhere else to go. Both land in the compiled entry's `context`.
_KEEPOUT_KEYS = {'name', 'rect', 'circle', 'sides', 'allow', 'kind', 'why',
                 'note', 'context'}
_FIXED_KEYS = {'ref', 'why', 'requirement', 'context'}
_BAND_KEYS = {'from', 'to'}
_OVERHANG_KEYS = {'min', 'max'}

#: #902. "These two named parts, this far apart" -- the one class of claim the
#: netlist IMPLIES and no instrument here can read: a 3mm crystal loop and a
#: 30mm one have identical connectivity.
_PROXIMITY_KEYS = {'ref', 'near', 'max_mm', 'basis', 'pads', 'requirement',
                   'why', 'note', 'context'}

#: Which geometry the gap is measured between. Spelled `body` and NOT
#: `courtyard` -- the refusal below carries the measurement that decided it.
#:
#: This tuple and `floorplan`'s must agree, or the brief accepts a spelling the
#: intent loader then refuses -- an author would see their own compiled
#: document rejected. `tests/test_902_proximity.py` compares the two, and while
#: `floorplan` has no such tuple that row prints PASS (VACUOUS) and says so --
#: it arms itself when the intent half lands, rather than passing silently in
#: both states.
_PROXIMITY_BASES = ('pad_edge', 'body')
_PROXIMITY_DEFAULT_BASIS = 'pad_edge'

#: Row keys refused BY NAME with the reason, the way `_REFUSED_TOP_LEVEL` does
#: it one level up. Each is a spelling an author reaches for first, so the
#: message has to carry the correction rather than just the refusal.
_PROXIMITY_REFUSED_ROW = {
    'min_mm': 'a MINIMUM separation is the clearance channel\'s claim '
              '(legality, and check_drc pad-to-pad), measured in a different '
              'currency and enforced by a different gate. A second minimum '
              'here would let one board pass one and fail the other with no '
              'rule for which wins. This key is `max_mm`: how far apart these '
              'parts may be, not how close',
    'max_distance_mm': 'that is the `decaps` spelling, and it means cap '
                       'CENTROID to the IC pad-bbox inflated 0.5mm, clamped '
                       'to 0 inside. This key is `max_mm`, and it is pad edge '
                       'to pad edge (or body to body). Two spellings for two '
                       'currencies, so a reader cannot mistake one number for '
                       'the other',
}

_PRIMARY_AXIS = ('east-west', 'north-south', UNKNOWN)
_SIDES = ('F', 'B', UNKNOWN)
_MOUNT_MODES = ('edge_mount', 'top_mount', 'bottom_mount', 'through_edge',
                UNKNOWN)
_CABLE_ENTRY = ('in_plane', 'perpendicular_top', 'perpendicular_bottom',
                'none', UNKNOWN)

#: #959 (#1000): what an `edge_mount` part's drawn body must sit within of its
#: edge. Measured, not mapped: the literal mapping would reuse the receptacle
#: seat tolerance (0.5 mm), and tigard's J7 -- an edge-mount header on a
#: shipping board -- sits 0.60 mm in (Phase-0 P3, five as-built boards).
EDGE_MOUNT_SETBACK_MM = 0.75

#: The tier-0 questions. Named so a report can say which were answered, which
#: were answered "I do not know", and which nobody touched -- the three states
#: this module exists to keep apart.
_TIER0 = ('product.form_factor', 'product.primary_axis', 'interfaces')


@dataclass(frozen=True)
class Brief:
    schema: int
    kind: str
    board: str
    units: str
    product: Dict[str, object]
    interfaces: Tuple[Dict[str, object], ...]
    keepouts: Tuple[Dict[str, object], ...]
    fixed: Tuple[Dict[str, object], ...]
    unknown: Tuple[str, ...]
    #: #902. Defaulted, so every existing construction site keeps working and a
    #: brief declaring none behaves exactly as it did before this key existed.
    proximity: Tuple[Dict[str, object], ...] = ()
    context: Dict[str, object] = field(default_factory=dict)
    source_path: str = ''


def empty_brief(board: str = '') -> Brief:
    return Brief(schema=SCHEMA_VERSION, kind=KIND, board=board, units='mm',
                 product={}, interfaces=(), keepouts=(), fixed=(),
                 unknown=(), proximity=(), context={})


# --------------------------------------------------------------------------
# discovery and loading
# --------------------------------------------------------------------------

def brief_path_for(board_path: str) -> str:
    """`<board>.design-brief.json`, or '' for an empty path."""
    if not board_path:
        return ''
    return os.path.splitext(board_path)[0] + BRIEF_EXT


def discover_brief(board_path: str = '', pcb_data=None) -> str:
    """The sibling brief's path, or ''. NEVER raises.

    The `.kicad_dru` rule copied exactly (`kicad_dru.read_board_*`): the
    caller's path when it has one, else `PCBData.source_path`; absence is a
    strict no-op rather than an exception. A board that declares nothing must
    cost its caller nothing.
    """
    path = board_path or getattr(pcb_data, 'source_path', '') or ''
    cand = brief_path_for(path)
    try:
        return cand if cand and os.path.isfile(cand) else ''
    except (OSError, ValueError):
        return ''


def load_brief(path: str) -> Brief:
    try:
        with open(path, encoding='utf-8') as fh:
            raw = json.load(fh)
    except (OSError, ValueError) as exc:
        raise BriefError(f"cannot read design brief {path}: {exc}") from None
    return brief_from_dict(raw, path)


def _known(value) -> bool:
    """Declared, and not declared-unknown."""
    return value is not None and value != UNKNOWN


def _enum(value, allowed, where: str):
    if value is None:
        return None
    if value not in allowed:
        raise BriefError(f"{where}: {value!r}, expected one of "
                         f"{', '.join(map(repr, allowed))}")
    return value


def _band(value, where: str):
    """`"center"`, `"unknown"`, or `{from, to}` fractions. Returns as given."""
    if value is None or value == UNKNOWN or value == 'center':
        return value
    if not isinstance(value, dict):
        raise BriefError(f"{where}: expected \"center\", \"{UNKNOWN}\", or "
                         f"{{'from': .., 'to': ..}}, got {value!r}")
    fp._reject_unknown(value, _BAND_KEYS, where)
    missing = sorted(_BAND_KEYS - set(value))
    if missing:
        raise BriefError(f"{where}: needs "
                         f"{', '.join('`%s`' % m for m in missing)} "
                         f"(fractions of the edge span, 0 to 1)")
    f0 = fp._number(value['from'], f"{where}.from", 0.0, 1.0)
    f1 = fp._number(value['to'], f"{where}.to", 0.0, 1.0)
    if not f0 < f1:
        raise BriefError(f"{where}: from {f0} is not less than to {f1}, so "
                         f"the band is empty or inverted and no pose can "
                         f"satisfy it")
    return {'from': f0, 'to': f1}


def proximity_claim_id(row: int, ref: str, near: str) -> str:
    """The name one proximity claim is reported under, everywhere (#902).

    PUBLIC and shared because this string is meant to be a contract between
    more than one reader -- `compile_brief`'s `declared` / `unknown` lists
    today, and any consumer that later names a claim (a coverage report, a
    waiver an author types). Hand-written f-strings at each site would drift,
    and a waiver that no longer matches its clause turns a working gate into
    one nobody can clear. Only the compiler calls it so far; it is a function
    rather than a literal so the second caller cannot spell it differently.

    It carries the ROW INDEX and not just the two refs, and that is not
    decoration. A reference may legally contain `~`: `disambiguate_references`
    produces `TP4~2` for a duplicated refdes, and `esp_prog` itself parses
    `Ref*` and `Ref*~2`. Without the index, a row `A~B` near `C` and a row `A`
    near `B~C` both spell `proximity[A~B~C]` -- and since `unknown` is a SET
    union, one of two DECLARED "I do not know"s silently disappeared. An index
    is unique by construction, and a list `ref` cannot repeat a member (the
    loader refuses that), so `(row, ref)` names exactly one claim.
    """
    return f"proximity[{row}:{ref}~{near}]"


def _proximity_pads(value, where: str, allowed):
    """`{ref: [pad numbers]}`, or `"unknown"`, or absent. Returns as given.

    Pad numbers are refused unless they are STRINGS, and that is not pedantry:
    `Pad.pad_number` is a string on both parse paths, so a JSON `1` would match
    nothing, resolve zero pads, and grade CLEAN -- a refusal that reads as a
    pass, in the one direction nobody checks. #710's rule ("a typo'd key that
    loads clean is a constraint the author believes they set") applied one
    level down, to a value.

    It catches ONE SPELLING of that failure and not the failure: `"4"` on a
    2-pad part is a well-typed name matching no pad, and no load-time check can
    know that without the board. Catching it is the GRADE's job, and the split
    is deliberate: the loader refuses what is wrong about the DOCUMENT, and
    only a rule holding a board can report a name that resolves to nothing on
    it. Until the grading half lands, a pad name matching nothing is silently
    unmeasured -- which is why it is named here rather than left for a reader
    to discover.
    """
    if value is None or value == UNKNOWN:
        return value
    if not isinstance(value, dict):
        raise BriefError(f"{where}: expected {{'REF': ['1', '2']}} or "
                         f"\"{UNKNOWN}\", got {value!r}")
    if not value:
        # An empty spec is not "no pads declared", it is a pads key that says
        # nothing -- and letting it through made it a THIRD spelling of
        # "unpadded" that the reversed-pair guard below did not recognise, so
        # two contradictory limits on one symmetric measurement loaded clean.
        raise BriefError(
            f"{where}: an empty pad spec declares nothing. Omit `pads` to "
            f"measure part to part, or write \"{UNKNOWN}\" to say the pins "
            f"are not known -- both are reported; an empty object is not")
    for ref, nums in value.items():
        if ref not in allowed:
            raise BriefError(
                f"{where}: a pad list for {ref!r}, which this row says nothing "
                f"about -- it names {' and '.join(repr(a) for a in allowed)}. "
                f"A pad list nothing reads is a claim nothing checks")
        if not isinstance(nums, (list, tuple)) or not nums:
            raise BriefError(f"{where}.{ref}: expected a non-empty list of pad "
                             f"numbers, got {nums!r}")
        for n in nums:
            if not isinstance(n, str):
                raise BriefError(
                    f"{where}.{ref}: pad {n!r} has type "
                    f"{type(n).__name__}; pad numbers are a list of strings. "
                    f"`Pad.pad_number` is a string, so {n!r} would match no "
                    f"pad, measure nothing, and grade clean")
    return {str(k): [str(n) for n in v] for k, v in value.items()}


def _proximity_rows(raw: Dict) -> List[Dict[str, object]]:
    """Validate `proximity[]` (#902). One row = one claim = one limit.

    `ref` may be a LIST -- "C1/C3 are U2's bulk caps" -- and it is pure sugar:
    `compile_brief` expands it to one intent row per member, so the intent's
    own `proximity[].ref` is always a string. The alternative reading, "every
    pair within the list", was rejected because a 3-list would then be three
    claims sharing one number and `Violation.ref` is a single ref with nowhere
    to say which pair the limit was about. "Q1 and Q2 together" is already
    exactly `{"ref": "Q1", "near": "Q2"}`, so nothing is lost.
    """
    rows: List[Dict[str, object]] = []
    seen: Dict[Tuple[str, str], int] = {}
    unpadded: Dict[Tuple[str, str], int] = {}
    got = raw.get('proximity')
    if got is not None and not isinstance(got, list):
        # Checked before the loop, because a STRING is iterable: `"unknown"`
        # was refused, but as `proximity[0]: expected an object` about the
        # character `u`. A refusal that names the wrong thing sends the author
        # to the wrong line.
        raise BriefError(
            f"proximity: expected a list of rows, got "
            f"{type(got).__name__}. There is no whole-key \"{UNKNOWN}\": the "
            f"three states are per CLAIM, so an unknown limit is written "
            f"`\"max_mm\": \"{UNKNOWN}\"` on the row it belongs to")
    for i, r in enumerate(got or []):
        where = f"proximity[{i}]"
        if not isinstance(r, dict):
            raise BriefError(f"{where}: expected an object with `ref`, `near` "
                             f"and `max_mm`")
        for key, why in _PROXIMITY_REFUSED_ROW.items():
            if key in r:
                raise BriefError(f"{where}: `{key}` is not a proximity key -- "
                                 f"{why}")
        fp._reject_unknown(r, _PROXIMITY_KEYS, where)
        fp._entry_context(r, where)

        ref = r.get('ref')
        refs = list(ref) if isinstance(ref, (list, tuple)) else [ref]
        if not refs or any(not isinstance(x, str) or not x for x in refs):
            raise BriefError(f"{where}: `ref` must be a reference or a list of "
                             f"references, got {ref!r}")
        if UNKNOWN in refs:
            raise BriefError(
                f"{where}: `ref` cannot be \"{UNKNOWN}\". A row whose subject "
                f"is unknown is not a row -- there is nothing to compile and "
                f"nothing to name in the report. Omit it, or name the part")
        if len(set(refs)) != len(refs):
            raise BriefError(f"{where}: `ref` lists {ref!r}, which names a "
                             f"part twice")
        near = r.get('near')
        if not near or not isinstance(near, str):
            raise BriefError(f"{where}: needs `near` -- the part the "
                             f"subject(s) must stay close to")
        if near == UNKNOWN:
            raise BriefError(
                f"{where}: `near` cannot be \"{UNKNOWN}\", for the same reason "
                f"`ref` cannot: a claim with no partner measures nothing")
        if near in refs:
            raise BriefError(f"{where}: {near!r} is both the subject and the "
                             f"partner; a part is 0mm from itself, so this "
                             f"row grades nothing")

        if 'max_mm' not in r:
            raise BriefError(
                f"{where}: needs `max_mm` -- how far apart these parts may be, "
                f"in mm. There is no default: the whole point of the key is "
                f"that a number nobody stated cannot be graded. If the spec "
                f"says \"as short as possible\" and names no number, write "
                f"\"{UNKNOWN}\": it is carried and REPORTED rather than "
                f"guessed")
        limit = r['max_mm']
        if limit != UNKNOWN:
            # `_number` for the TYPE (it refuses a string, a bool and anything
            # non-numeric), and the magnitude checked here rather than through
            # its `lo=`. With `lo=0.0` an author writing 0 and an author
            # writing -1 got two different messages for one mistake -- "expected
            # a positive distance" and "expected >= 0.0" -- and the bound was
            # also unreachable-by-test, since the guard below already caught
            # everything it would have.
            fp._number(limit, f"{where}.max_mm")
            # FINITE first, and it has to be checked explicitly: `inf` and
            # `nan` both pass a `lo=0.0` bound and a `<= 0.0` guard, because
            # `inf < 0` and `nan <= 0` are both False. `json.load` accepts the
            # literals `Infinity` and `NaN`, so this is reachable from a real
            # sibling file. An `inf` limit grades clean forever and a `nan`
            # limit makes every comparison False -- both are a declared
            # constraint that can never fail, which is this channel's own
            # definition of the bug, and both would also make the emitted
            # intent non-RFC JSON that a strict reader rejects.
            if not math.isfinite(float(limit)):
                raise BriefError(
                    f"{where}.max_mm: {limit!r} is not a finite distance. A "
                    f"limit of infinity can never be exceeded and a NaN limit "
                    f"fails every comparison, so either would be a declared "
                    f"claim nothing can ever violate. If the spec names no "
                    f"number, write \"{UNKNOWN}\" -- it is carried and "
                    f"REPORTED rather than silently satisfied")
            if float(limit) <= 0.0:
                raise BriefError(f"{where}.max_mm: {limit!r}, expected a "
                                 f"positive distance in mm")
        basis = r.get('basis')
        if basis == UNKNOWN:
            raise BriefError(
                f"{where}.basis: \"{UNKNOWN}\" is not allowed here, unlike "
                f"every other enum in this file, because `basis` HAS a default "
                f"({_PROXIMITY_DEFAULT_BASIS!r}). Declaring it unknown would "
                f"compile to that default while the report says nobody knows "
                f"-- two different documents. Omit the key to take the "
                f"default, or name the one you mean")
        if basis == 'courtyard':
            raise BriefError(
                f"{where}.basis: \"courtyard\" is not a basis. Since #896 "
                f"`placement.body` is a LADDER -- courtyard, fab, silk union "
                f"pads, pad bbox -- and it answers whichever rung the library "
                f"drew. On the board this rule was written for, 0 of 21 "
                f"footprints draw a courtyard -- 10 answer from fab, 4 from "
                f"silk, 4 from the pad bbox and 3 draw nothing at all -- so a "
                f"claim spelled \"courtyard\" would grade nothing there. Write "
                f"\"body\": the gap is then measured between the drawn bodies, "
                f"and the rung each one came from is reported beside the "
                f"number so it is never read without knowing what it rests on")
        _enum(basis, _PROXIMITY_BASES, f"{where}.basis")
        pads = _proximity_pads(r.get('pads'), f"{where}.pads",
                              tuple(refs) + (near,))

        for ref_one in refs:
            key = (ref_one, near)
            if key in seen:
                raise BriefError(
                    f"{where}: duplicate proximity claim for {ref_one} near "
                    f"{near} (already declared at proximity[{seen[key]}]) -- "
                    f"two claims about one relation, with no rule for which "
                    f"wins, and the grade would charge BOTH")
            seen[key] = i
        # The REVERSED pair, and only when NEITHER row carries an EFFECTIVE pad
        # spec. `rect_gap` is symmetric and both sides are then existential, so
        # the two rows are provably the same number -- one fact charged twice.
        # With real `pads` on either row the claim is genuinely asymmetric (for
        # each of MY declared pads, some pad of yours is close enough), so both
        # are kept.
        #
        # "Effective" and not `pads is None`, because there are THREE spellings
        # of unpadded and keying on one of them let the other two through: an
        # absent key, `"unknown"` (declared, but still measured part to part),
        # and -- until it was refused above -- an empty object. A brief
        # declaring 5mm one way and 9mm the other loaded clean.
        #
        # And the asymmetry that matters is a SUBJECT pad list specifically:
        # `pads[ref]` is what turns the existential min into "for EACH of my
        # pads", which is the quantifier the rule's own invariant rests on. A
        # list naming only the partner leaves both rows existential, so it is
        # still the same measurement.
        for ref_one in refs:
            if isinstance(pads, dict) and pads.get(ref_one):
                continue
            rev = (near, ref_one)
            if rev in unpadded:
                raise BriefError(
                    f"{where}: {ref_one} near {near} is the reverse of "
                    f"proximity[{unpadded[rev]}], and neither row names a "
                    f"`pads` list for its own subject -- so the two are the "
                    f"same symmetric measurement declared twice, with no "
                    f"rule for which claim wins. Keep one, or name the pads "
                    f"that make them different claims")
            unpadded[(ref_one, near)] = i

        row = {'ref': refs if isinstance(ref, (list, tuple)) else refs[0],
               'near': near, 'max_mm': limit}
        for key in ('basis', 'requirement', 'why', 'note', 'context'):
            if r.get(key) is not None:
                row[key] = r[key]
        if pads is not None:
            row['pads'] = pads
        rows.append(row)
    return rows


def brief_from_dict(raw: Dict, source_path: str = '') -> Brief:
    """Parse and validate a design brief. Strict at every level.

    The same discipline as `intent_from_dict`, for the same reason (#710): a
    typo'd key that is silently ignored is a constraint the author believes
    they set and nothing ever checks.

    Every refusal leaves here as a `BriefError`, including the ones raised by
    the shared `floorplan` validators this reuses. Those raise `IntentError`,
    and a caller that cannot tell "your BRIEF is malformed" from "your INTENT
    is malformed" is sent to the wrong file -- the two are different documents
    with different authors. `BriefError` subclasses `IntentError`, so a caller
    that only cares that something was malformed still catches it.
    """
    try:
        return _brief_from_dict(raw, source_path)
    except BriefError:
        raise
    except fp.IntentError as exc:
        raise BriefError(str(exc)) from None


def _brief_from_dict(raw: Dict, source_path: str = '') -> Brief:
    if not isinstance(raw, dict):
        raise BriefError(f"design brief: expected a JSON object, got "
                         f"{type(raw).__name__}")

    # `kind` FIRST, before unknown keys, so the near-miss gets its own answer.
    # `board_brief.py --json` writes a document that is also JSON and also
    # calls itself a brief; read as a design brief it would be a pile of
    # unknown keys, and the message would send the reader to fix spellings in
    # a file that was never meant to be one.
    kind = raw.get('kind')
    if kind != KIND:
        if kind == 'board-brief':
            raise BriefError(
                f"{source_path or 'design brief'}: kind 'board-brief'. That "
                f"is board_brief.py's OUTPUT -- an assembled measurement OF "
                f"the board. A design brief is the INPUT you hand it: what "
                f"the board file cannot know. Expected kind {KIND!r}")
        raise BriefError(
            f"{source_path or 'design brief'}: kind {kind!r}, expected "
            f"{KIND!r}. This does not look like a design brief")

    # `min_reader` before the key sets, as `intent_from_dict` does: grading a
    # file halfway is the same wrong answer as grading it fully.
    mr = raw.get('min_reader')
    if mr is not None:
        if isinstance(mr, bool) or not isinstance(mr, int):
            raise BriefError(f"min_reader: expected an integer, got {mr!r}")
        if mr > MIN_READER:
            raise BriefError(
                f"min_reader {mr}: this build is reader {MIN_READER}. The "
                f"brief declares a claim this build would not act on, and "
                f"compiling it without that claim would produce an intent "
                f"the author never wrote -- upgrade instead")

    for bad, why in sorted(_REFUSED_TOP_LEVEL.items()):
        if bad in raw:
            raise BriefError(f"design brief: `{bad}` is not declarable -- "
                             f"{why}")
    fp._reject_unknown(raw, _TOP_LEVEL_KEYS, 'design brief')

    if raw.get('schema') != SCHEMA_VERSION:
        raise BriefError(f"design brief: schema {raw.get('schema')!r}, this "
                         f"build reads schema {SCHEMA_VERSION}")
    units = raw.get('units', 'mm')
    if units != 'mm':
        raise BriefError(f"design brief: units {units!r}, expected 'mm'")

    product = fp._obj(raw.get('product'), 'product')
    fp._reject_unknown(product, _PRODUCT_KEYS, 'product')
    _enum(product.get('primary_axis'), _PRIMARY_AXIS, 'product.primary_axis')
    _enum(product.get('user_top_side'), _SIDES, 'product.user_top_side')

    interfaces: List[Dict[str, object]] = []
    seen_refs: Dict[str, int] = {}
    for i, c in enumerate(raw.get('interfaces') or []):
        where = f"interfaces[{i}]"
        if not isinstance(c, dict):
            raise BriefError(f"{where}: expected an object with a `ref`")
        fp._reject_unknown(c, _INTERFACE_KEYS, where)
        fp._entry_context(c, where)
        ref = c.get('ref')
        if not ref or not isinstance(ref, str):
            raise BriefError(f"{where}: expected an object with a `ref`")
        where = f"{where} ({ref})"
        # A duplicate row is two claims about one part with no rule for which
        # wins -- and the three consumers downstream disagree about that: the
        # grade charges BOTH, the repair dispatch keeps the LAST, and the
        # seeder's even distribution shifts every other connector on the edge.
        # `blocks` already refuses a duplicate name for the same reason.
        if ref in seen_refs:
            raise BriefError(f"{where}: duplicate interface for {ref!r} "
                             f"(already declared at interfaces"
                             f"[{seen_refs[ref]}]) -- two claims about one "
                             f"part, with no rule for which wins")
        seen_refs[ref] = i
        _enum(c.get('edge'), fp._EDGES + (UNKNOWN,), f"{where}.edge")
        _enum(c.get('mount_mode'), _MOUNT_MODES, f"{where}.mount_mode")
        _enum(c.get('cable_entry'), _CABLE_ENTRY, f"{where}.cable_entry")
        env = c.get('cable_envelope_mm')
        if env is not None and env != UNKNOWN:
            if not isinstance(env, dict) or not env:
                raise BriefError(
                    f"{where}.cable_envelope_mm: expected {{'depth': mm, "
                    f"'clear': mm}} or {UNKNOWN!r}, got {env!r}")
            fp._reject_unknown(env, _ENVELOPE_KEYS,
                               f"{where}.cable_envelope_mm")
            for k_ in sorted(env):
                v_ = fp._number(env[k_], f"{where}.cable_envelope_mm.{k_}")
                if v_ <= 0:
                    raise BriefError(
                        f"{where}.cable_envelope_mm.{k_}: {v_:g} is not a "
                        f"dimension -- write {UNKNOWN!r} if it is not known")
        if env is not None and c.get('cable_entry') in (None, 'none',
                                                         UNKNOWN):
            raise BriefError(
                f"{where}.cable_envelope_mm describes a cable, and this "
                f"interface declares no cable_entry it could apply to")
        uf = c.get('user_facing')
        if uf is not None and uf is not True and uf is not False \
                and uf != UNKNOWN:
            raise BriefError(f"{where}.user_facing: expected true, false or "
                             f"{UNKNOWN!r}, got {uf!r}")
        c = dict(c)
        c['along_edge'] = _band(c.get('along_edge'), f"{where}.along_edge")
        if c['along_edge'] is None:
            c.pop('along_edge')
        if c.get('along_edge_tolerance_mm') is not None:
            fp._number(c['along_edge_tolerance_mm'],
                       f"{where}.along_edge_tolerance_mm", lo=0.0)
        oh = c.get('overhang_mm')
        if oh is not None:
            if not isinstance(oh, dict):
                raise BriefError(f"{where}.overhang_mm: expected "
                                 f"{{'min': .., 'max': ..}}, got {oh!r}")
            fp._reject_unknown(oh, _OVERHANG_KEYS, f"{where}.overhang_mm")
        if c.get('along_edge') == 'center' \
                and c.get('along_edge_tolerance_mm') is None:
            raise BriefError(
                f"{where}: along_edge \"center\" needs "
                f"`along_edge_tolerance_mm`. There is no default: measured "
                f"on this repo's own tracked boards, tigard's connectors sit "
                f"16-29% off their edge centres, so any threshold this tool "
                f"picked would fail a good board")
        interfaces.append(c)

    keepouts: List[Dict[str, object]] = []
    for i, k in enumerate(raw.get('keepouts') or []):
        where = f"keepouts[{i}]"
        if not isinstance(k, dict):
            raise BriefError(f"{where}: expected an object")
        fp._reject_unknown(k, _KEEPOUT_KEYS, where)
        fp._entry_context(k, where)
        k = dict(k)
        k.setdefault('name', f"brief-keepout{i}")
        where = f"{where} ({k['name']})"
        has_rect, has_circle = k.get('rect') is not None, k.get('circle') is not None
        if has_rect == has_circle:
            raise BriefError(f"{where}: needs exactly one of `rect` or "
                             f"`circle`")
        if has_rect:
            k['rect'] = list(fp._rect(k['rect'], f"{where}.rect"))
        else:
            circ = k['circle']
            if not isinstance(circ, (list, tuple)) or len(circ) != 3:
                raise BriefError(f"{where}.circle: expected [x, y, radius]")
            k['circle'] = [fp._number(v, f"{where}.circle[{j}]")
                           for j, v in enumerate(circ)]
        for s in (k.get('sides') or ()):
            if s not in ('F', 'B'):
                raise BriefError(f"{where}.sides: {s!r}, expected 'F' or 'B'")
        fp._str_tuple(k.get('allow'), f"{where}.allow")
        keepouts.append(k)

    proximity = _proximity_rows(raw)

    fixed: List[Dict[str, object]] = []
    for i, f in enumerate(raw.get('fixed') or []):
        where = f"fixed[{i}]"
        if not isinstance(f, dict):
            raise BriefError(f"{where}: expected an object with a `ref`")
        fp._reject_unknown(f, _FIXED_KEYS, where)
        fp._entry_context(f, where)
        if not f.get('ref'):
            raise BriefError(f"{where}: expected an object with a `ref`")
        fixed.append(dict(f))

    unknown = fp._str_tuple(raw.get('unknown'), 'unknown')
    return Brief(schema=SCHEMA_VERSION, kind=KIND,
                 board=str(raw.get('board') or ''), units=units,
                 product=product, interfaces=tuple(interfaces),
                 keepouts=tuple(keepouts), fixed=tuple(fixed),
                 unknown=unknown, proximity=tuple(proximity),
                 context=fp._obj(raw.get('context'), 'context'),
                 source_path=source_path)


# --------------------------------------------------------------------------
# the compiler
# --------------------------------------------------------------------------

def compile_brief(brief: Brief, *, board_refs: Sequence[str] = (),
                  refs_known: bool = True) -> Tuple[Dict, Dict]:
    """Brief -> a floorplan-intent FRAGMENT, plus a report. PURE.

    No file IO and no board parsing, so it is testable board-independently --
    the same split `validate_intent` has from `grade`, and for the same reason.

    The fragment carries only `edge_connectors` and `keepouts`; `merge_into_
    intent` decides how it meets an emitted document. The report is the
    honesty channel: which claims were declared, which were declared UNKNOWN,
    which tier-0 fields nobody touched, what was carried but is not graded,
    and which refs the board does not have.

    `board_refs` empty with `refs_known=False` means "no board was available",
    and the unmatched check is SKIPPED and reported as skipped rather than
    guessed -- an empty ref list must not read as "every ref is wrong".
    """
    conns: List[Dict[str, object]] = []
    declared: List[str] = []
    unknown: List[str] = []
    not_graded: List[str] = []
    unmatched: List[str] = []
    wrote_along_edge = False

    for key in ('form_factor', 'primary_axis', 'held_by', 'user_top_side'):
        v = brief.product.get(key)
        if v is None:
            continue
        (unknown if v == UNKNOWN else declared).append(f"product.{key}")
    # Carried so the reader knows what the board is, graded by nothing. Said
    # so out loud rather than left to be discovered.
    if _known(brief.product.get('form_factor')):
        not_graded.append('product.form_factor')
    if _known(brief.product.get('user_top_side')):
        not_graded.append('product.user_top_side')

    refset = set(board_refs or ())
    for i, c in enumerate(brief.interfaces):
        ref = str(c['ref'])
        entry: Dict[str, object] = {'ref': ref, 'source': 'brief'}
        ctx: Dict[str, object] = dict(c.get('context') or {})
        ctx['brief_row'] = i
        if c.get('why'):
            ctx['why'] = c['why']
        if c.get('requirement'):
            ctx['requirement'] = c['requirement']
        if c.get('role'):
            ctx['role'] = c['role']

        edge = c.get('edge')
        if _known(edge):
            entry['edge'] = edge
            declared.append(f"interfaces[{ref}].edge")
        elif edge == UNKNOWN:
            # NO `edge` key. The schema supports edge-less entries and every
            # consumer already declines to guess: the seeder's stage 1 skips
            # them, the repair path refuses honestly, and `emit_intent`
            # refuses to name one for an implausible pose. "I do not know"
            # reaches machinery that knows what to do with it.
            unknown.append(f"interfaces[{ref}].edge")
            # DROP an inferred edge rather than leaving it. "I do not know"
            # is an answer, and an entry that keeps the emitter's
            # `_nearest_edge` guess is graded against a pose-derived edge the
            # author explicitly declined to name -- the inversion this whole
            # channel exists to prevent. `merge_into_intent` reads this key.
            ctx['edge_declared_unknown'] = True
            entry['note'] = ('the brief declares this connector with edge '
                             '"unknown" -- no edge is claimed, and any edge '
                             'the emitter inferred is dropped')

        if _known(c.get('user_facing')) and c.get('user_facing') is True:
            # An edge class is what the placement ENGINES read (via
            # `Intent.edge_claims`), so a user-facing connector the brief
            # names gets the same treatment a classified receptacle does.
            entry['class'] = 'edge_receptacle'
            declared.append(f"interfaces[{ref}].user_facing")
        elif c.get('user_facing') == UNKNOWN:
            unknown.append(f"interfaces[{ref}].user_facing")

        if c.get('overhang_mm') is not None:
            entry['overhang_mm'] = dict(c['overhang_mm'])
            declared.append(f"interfaces[{ref}].overhang_mm")

        band = c.get('along_edge')
        if band == 'center':
            entry['center_on_edge'] = {
                'tolerance_mm': float(c['along_edge_tolerance_mm'])}
            declared.append(f"interfaces[{ref}].along_edge")
            wrote_along_edge = True
        elif isinstance(band, dict):
            entry['along_edge_band'] = dict(band)
            declared.append(f"interfaces[{ref}].along_edge")
            wrote_along_edge = True
        elif band == UNKNOWN:
            unknown.append(f"interfaces[{ref}].along_edge")

        for key in ('mount_mode', 'cable_entry'):
            v = c.get(key)
            if v is None:
                continue
            if v == UNKNOWN:
                unknown.append(f"interfaces[{ref}].{key}")
                continue
            # Carried into the documented ungraded slot, and named as ungraded
            # in the same breath -- an author is told it was recorded AND that
            # nothing checks it, rather than discovering the second part later.
            ctx[key] = v
            declared.append(f"interfaces[{ref}].{key}")
            not_graded.append(f"interfaces[{ref}].{key}")
        env = c.get('cable_envelope_mm')
        if env == UNKNOWN:
            # Kept in context so the consequence step can tell "the author
            # looked and does not know" from "nobody said".
            ctx['cable_envelope_mm'] = UNKNOWN
            unknown.append(f"interfaces[{ref}].cable_envelope_mm")
        elif env is not None:
            ctx['cable_envelope_mm'] = dict(env)
            declared.append(f"interfaces[{ref}].cable_envelope_mm")
            not_graded.append(f"interfaces[{ref}].cable_envelope_mm")

        if c.get('note'):
            entry['note'] = ((entry.get('note', '') + '; ') if entry.get('note')
                             else '') + str(c['note'])
        entry['context'] = ctx
        if refs_known and refset and ref not in refset:
            # KEPT, never dropped. A brief ref the board does not have is a
            # typo that would otherwise grade clean -- `block_unresolved`'s
            # exact failure one level over. `rule_edge_connector` already
            # fires "edge connector {ref} is not on this board" at ERROR, so
            # keeping it means the grade says so with a rule name attached.
            unmatched.append(ref)
        conns.append(entry)

    keeps: List[Dict[str, object]] = []
    for k in brief.keepouts:
        out = {key: v for key, v in k.items()
               if key in ('name', 'rect', 'circle', 'sides', 'allow', 'note')}
        ctx = dict(k.get('context') or {})
        ctx['source'] = 'brief'
        for key in ('kind', 'why'):
            if k.get(key):
                ctx[key] = k[key]
        out['context'] = ctx
        keeps.append(out)
        declared.append(f"keepouts[{out['name']}]")

    # #902. Compiled 1:1 into the intent's own `proximity[]`, with a list
    # `ref` EXPANDED here so the intent never carries the sugar -- one row is
    # one claim is one limit, and `Violation.ref` has room for exactly one ref.
    prox: List[Dict[str, object]] = []
    expanded = dropped = 0
    for i, r in enumerate(brief.proximity):
        near = str(r['near'])
        raw_ref = r['ref']
        members = ([str(x) for x in raw_ref]
                   if isinstance(raw_ref, (list, tuple)) else [str(raw_ref)])
        if len(members) > 1:
            expanded += 1
        pads = r.get('pads')
        for ref in members:
            claim = proximity_claim_id(i, ref, near)
            # Reported BEFORE the unknown-limit branch below, not inside the
            # compiled arm: an author who wrote two "I do not know"s must see
            # two. Reporting it after the `continue` recorded only the limit,
            # and a declared unknown that vanishes is the failure this module's
            # second design rule is written against.
            if pads == UNKNOWN:
                # Declared-unknown at a different arity from the limit: with a
                # limit the row still grades, part to part, and the reader is
                # told the pin-level claim was not stated rather than left to
                # infer that from an absent key.
                unknown.append(f"{claim}.pads")
            # BEFORE the unknown-limit branch, for the same reason the pads
            # report is: a ref this board does not have is a TYPO, and a typo
            # on an "as short as possible" row is exactly as wrong as one on a
            # row with a number. The first fix moved the pads report above the
            # `continue` and left this below it, so a misspelled partner on an
            # unknown-limit row was reported by nothing at all -- the same
            # defect, one line further down. `interfaces[]` reports an
            # unmatched ref whatever else the row declares unknown.
            for who in (ref, near):
                if refs_known and refset and who not in refset:
                    unmatched.append(who)
            if r['max_mm'] == UNKNOWN:
                # DECLARED, and declared UNKNOWN. "Y1 must be beside U1, I do
                # not know how close" is the issue's own phrase ("as short as
                # possible"), and it compiles to NO ROW: there is no number to
                # grade against, and inventing one would be the guess this
                # whole channel exists to refuse. It is reported, not dropped,
                # so it cannot read as a key nobody wrote.
                unknown.append(f"{claim}.max_mm")
                dropped += 1
                continue
            entry: Dict[str, object] = {
                'ref': ref, 'near': near, 'max_mm': float(r['max_mm']),
                'source': 'brief'}
            ctx: Dict[str, object] = dict(r.get('context') or {})
            # The SOURCE row, so an expanded member can be traced back to the
            # line the author wrote -- `interfaces[]` carries the same key for
            # the same reason, and without it a list `ref` compiles to rows
            # whose origin is unrecoverable.
            ctx['brief_row'] = i
            for key in ('why', 'requirement'):
                if r.get(key):
                    ctx[key] = r[key]
            entry['context'] = ctx
            if r.get('basis'):
                entry['basis'] = r['basis']
            if isinstance(pads, dict):
                mine = {k: list(v) for k, v in pads.items()
                        if k in (ref, near)}
                if mine:
                    entry['pads'] = mine
            if r.get('note'):
                entry['note'] = str(r['note'])
            declared.append(f"{claim}.max_mm")
            prox.append(entry)

    fragment: Dict[str, object] = {}
    if conns:
        fragment['edge_connectors'] = conns
    if keeps:
        fragment['keepouts'] = keeps
    if prox:
        fragment['proximity'] = prox
    # A RUNNING MAX, not a literal, since #902: a brief carrying both an
    # along-edge claim and a proximity row needs the HIGHER of the two readers,
    # and writing whichever branch ran last would understate it. `min_reader`
    # is the one field whose only job is to be true.
    need_reader = 0
    if wrote_along_edge:
        # The first real use of the mechanism `min_reader` was built for: a
        # reader that predates #712 must refuse this document rather than
        # grade it without the along-edge claim.
        need_reader = max(need_reader, 2)
    if prox:
        # Same argument one version on: a reader that predates #902 must refuse
        # a document carrying `proximity[]` rather than grade it without.
        need_reader = max(need_reader, 4)
    if need_reader:
        fragment['min_reader'] = need_reader

    absent = [k for k in _TIER0
              if (k == 'interfaces' and not brief.interfaces)
              or (k.startswith('product.')
                  and brief.product.get(k.split('.', 1)[1]) is None)]
    counts: Dict[str, object] = {'interfaces': len(brief.interfaces),
                                 'keepouts': len(brief.keepouts),
                                 'fixed': len(brief.fixed)}
    if brief.proximity:
        # Added ONLY when the brief declares any, so a brief that declares none
        # produces the same `counts` dict it produced before this key existed
        # -- and `counts` is emitted verbatim as `design_brief.counts` by
        # board_brief, so an unconditional key would change a machine-readable
        # document for every board on the corpus to say "zero of a thing you
        # never mentioned".
        #
        # `proximity` is the count of ROWS AS WRITTEN, so it matches what the
        # author sees in their own file. `proximity_claims` is the count after
        # a list `ref` expands and an unknown limit drops out -- what the
        # intent carries and what the grade will report. `proximity_expanded`
        # and `proximity_dropped` are kept because the two counts can CANCEL
        # (one row expanding by one and another dropping) and then a bare
        # comparison discloses nothing.
        counts.update({'proximity': len(brief.proximity),
                       'proximity_claims': len(prox),
                       'proximity_expanded': expanded,
                       'proximity_dropped': dropped})
    report = {
        'path': brief.source_path,
        'declared': sorted(declared),
        'unknown': sorted(set(unknown) | set(brief.unknown)),
        'absent': absent,
        'not_graded': sorted(not_graded),
        # DEDUPED since #902. A proximity row names two refs and a list `ref`
        # names `near` once per member, so an absent partner used to be
        # reported once per member -- check_floorplan printed its "brief names
        # ZZ9, which is not on this board" line twice for one missing part.
        # Impossible before, because a duplicate `interfaces[].ref` is refused.
        'unmatched': sorted(set(unmatched)),
        'unmatched_checked': bool(refs_known and (board_refs or ())),
        'contradictions': [],
        'counts': counts,
        # #711 asks for `place_fixed` ops. There is no plan-op implementation
        # in this tree -- `place_fixed` is named only in comments -- so a
        # fixed pose is CARRIED and reported, never asserted, and never turned
        # into `must_lock`: filling must_lock made `place_seed --repair` treat
        # those refs as seeder-owned and LIFT the user's locks (measured on
        # two run-7 boards, see emit_intent's own comment). It buys nothing
        # either, since `resolve_intent_gate` already freezes every edge claim
        # inside the quench.
        'fixed': [dict(f) for f in brief.fixed],
        'product': dict(brief.product),
    }
    return fragment, report


def connector_consequences(fragment: Dict, report: Dict, pcb=None,
                           board_path: str = ''):
    """The connector declarations `compile_brief` only CARRIES, compiled
    into clauses a rule grades (#959, #1000). `(fragment, report)`, both NEW
    -- `compile_brief` stays the pure compile -- with `report['consequences']`
    listing one row per consequence:

      `{id, ref, status: compiled|unmeasured|withheld, compiled_to, grader,
        basis: declared|derived_default|None, value, why}`

    Measured on five as-built boards before it was built (Phase-0 P3), and
    therefore NOT the issue's literal mapping:

      * `mount_mode: edge_mount` -> `max_setback_mm` 0.75 on the drawn body
        (`derived_default`; tigard J7 sits 0.60 in);
      * `mount_mode: through_edge` -> the same setback: the body reaches
        the edge (how far PAST it is `overhang_mm`, declared or emitted,
        never derived);
      * `mount_mode: top_mount` / `bottom_mount` -> an EXEMPTION, carried:
        the edge-receptacle seat does not apply to a part standing off a
        face (`floorplan.VERTICAL_MOUNTS`; the default seat false-failed 8
        vertical headers on the as-built boards);
      * `cable_entry: perpendicular_*` with a declared
        `product.user_top_side` -> the face the part is on, graded as
        `edge_connector_side` at a fixed WARN that steers no search.
        `user_facing` compiles NO face: reaching a part says nothing about
        which face it sits on;
      * `cable_entry: in_plane` -> carried: the declared edge is already a
        clause, and unmeasured without one;
      * a cable keep-out ONLY from a declared `cable_envelope_mm`, and only
        for a FILE-locked part (a keep-out off an unlocked part moves with
        every seed) that reaches its declared edge when the band is
        in-plane. No default dimension: none passed the controls.

    Row statuses: `compiled` (a rule grades it), `carried` (an exemption or
    a restatement -- nothing of its own is graded), `unmeasured` (a
    dimension nobody declared), `withheld` (declared, but not derivable on
    this board). A value the brief declares itself always wins over a
    derived one, a declared `cable:<ref>` keep-out included. z-height and
    insertion travel are never measured -- a keep-out is a 2D projection --
    and the rows say so.
    """
    import copy
    frag = copy.deepcopy(fragment or {})
    rep = copy.deepcopy(report or {})
    rows: List[Dict[str, object]] = []
    uts = (rep.get('product') or {}).get('user_top_side')
    uts = uts if uts in ('F', 'B') else None
    other = {'F': 'B', 'B': 'F'}
    locked = ({k for k, f in (pcb.footprints or {}).items()
               if getattr(f, 'locked', False)} if pcb is not None else set())
    keeps = list(frag.get('keepouts') or [])
    #: Keep-out names the BRIEF declares itself. A declared value wins over
    #: a derived one, so an envelope never replaces one of these.
    declared_ko = {k.get('name') for k in keeps}
    need_side = False
    used_uts = False
    geo = None

    def _row(ref, key, status, why, *, compiled_to=None, grader=None,
             basis=None, value=None):
        rows.append({'id': f"interfaces[{ref}].{key}", 'ref': ref,
                     'status': status, 'compiled_to': compiled_to,
                     'grader': grader, 'basis': basis, 'value': value,
                     'why': why})

    for e in frag.get('edge_connectors') or []:
        ref = str(e['ref'])
        ctx = e.setdefault('context', {})
        basis = ctx.setdefault('basis', {})
        src = ctx.setdefault('compiled_from', {})
        mm, ce = ctx.get('mount_mode'), ctx.get('cable_entry')
        if mm == 'edge_mount':
            if 'max_setback_mm' not in e:
                e['max_setback_mm'] = EDGE_MOUNT_SETBACK_MM
                basis['max_setback_mm'] = 'derived_default'
            src['max_setback_mm'] = 'mount_mode'
            _row(ref, 'mount_mode', 'compiled',
                 'the drawn body must sit within this of its edge (tigard '
                 'J7, an edge-mount header on a shipping board, sits 0.60 '
                 'mm in)',
                 compiled_to=f"edge_connectors[{ref}].max_setback_mm",
                 grader='edge_connector',
                 basis=basis.get('max_setback_mm', 'declared'),
                 value=e['max_setback_mm'])
        elif mm == 'through_edge':
            # "Reaches the edge": past it, or within the edge-mount setback
            # of it -- the same clause `edge_mount` compiles to. An overhang
            # floor of 0 would add nothing (a body inside the board reads 0
            # overhang and passes it), and writing one REPLACED the emitted
            # `overhang_mm` wholesale on merge, dropping its `max` -- and
            # with it the part's off-outline exemption, so a through-edge
            # connector hanging correctly past the edge graded as an
            # off-board part (Phase-5 verifier B1). What would tell the two
            # apart -- how FAR past the edge -- is a dimension nobody
            # declared, so the row says so rather than inventing one.
            if 'max_setback_mm' not in e:
                e['max_setback_mm'] = EDGE_MOUNT_SETBACK_MM
                basis['max_setback_mm'] = 'derived_default'
            src['max_setback_mm'] = 'mount_mode'
            _row(ref, 'mount_mode', 'compiled',
                 'the body reaches the edge: past it, or within the '
                 'edge-mount setback of it -- graded exactly as edge_mount; '
                 'how far past the edge it may reach is declared by '
                 'overhang_mm, never derived',
                 compiled_to=f"edge_connectors[{ref}].max_setback_mm",
                 grader='edge_connector',
                 basis=basis.get('max_setback_mm', 'declared'),
                 value=e['max_setback_mm'])
        elif mm in fp.VERTICAL_MOUNTS:
            # An EXEMPTION, not a clause: nothing measures that a part
            # stands off its face, so the row is `carried` -- it relaxes
            # the receptacle seat and grades nothing of its own.
            _row(ref, 'mount_mode', 'carried',
                 'the part stands off a face, so the edge-receptacle seat '
                 '(the mating face reaching the edge) does not apply to '
                 'it; nothing measures the mount itself, and its declared '
                 'edge and overhang are graded as before',
                 compiled_to=f"edge_connectors[{ref}].context.mount_mode",
                 basis='declared', value=mm)

        # The face, from a PERPENDICULAR cable only: the cable leaves the
        # face it plugs into. `user_facing` says the user reaches the part,
        # not which face it sits on -- it compiled a face once and put three
        # shipping B-side connectors (a DSUB, a JST-SH, a microSD) on the
        # wrong one (Phase-5 verifier S3).
        perp = ce in ('perpendicular_top', 'perpendicular_bottom')
        if perp:
            if uts is None:
                _row(ref, 'cable_entry', 'unmeasured',
                     'which face is the top is not declared '
                     '(product.user_top_side), so no face follows from it')
            else:
                side = uts if ce == 'perpendicular_top' else other[uts]
                if 'side' not in e:
                    e['side'] = side
                    basis['side'] = 'declared'
                    need_side = True
                src['side'] = 'cable_entry'
                used_uts = True
                _row(ref, 'cable_entry', 'compiled',
                     f"a {ce} cable leaves by the face it plugs into, and "
                     f"product.user_top_side {uts} is the top; the finding "
                     f"is advisory (WARN) and steers no search",
                     compiled_to=f"edge_connectors[{ref}].side",
                     grader='edge_connector_side', basis='declared',
                     value=e['side'])
        if ce == 'in_plane':
            if not e.get('edge'):
                _row(ref, 'cable_entry', 'unmeasured',
                     'an in-plane cable leaves by an edge, and this '
                     'interface declares none (edge "unknown")')
            else:
                # The declared edge is ALREADY a clause; in_plane restates
                # it and adds none, so it is not counted as a second one.
                _row(ref, 'cable_entry', 'carried',
                     f"the cable leaves in the board plane, by the declared "
                     f"edge -- graded only through interfaces[{ref}].edge, "
                     f"which in_plane adds nothing to",
                     compiled_to=f"edge_connectors[{ref}].edge",
                     basis='declared', value=e['edge'])

        # The cable keep-out, only from a declared envelope.
        if ce in ('in_plane', 'perpendicular_top', 'perpendicular_bottom'):
            env = ctx.get('cable_envelope_mm')
            dim = 'depth' if ce == 'in_plane' else 'clear'
            tail = ('; z-height and insertion travel are not measured either '
                    '-- a keep-out is a 2D projection')
            if env is None or env == UNKNOWN:
                _row(ref, 'cable_envelope_mm', 'unmeasured',
                     ('declared "unknown"' if env == UNKNOWN else
                      'no cable_envelope_mm is declared, and no default is '
                      'used: none passed the as-built controls (#959 P3)')
                     + f" -- the cable's {dim} is not measured" + tail)
                continue
            if dim not in env:
                _row(ref, 'cable_envelope_mm', 'unmeasured',
                     f"cable_envelope_mm declares no `{dim}`, which a {ce} "
                     f"cable needs" + tail)
                continue
            face = None
            if ce != 'in_plane':
                face = (uts if ce == 'perpendicular_top'
                        else other[uts]) if uts else None
                if face is None:
                    _row(ref, 'cable_envelope_mm', 'unmeasured',
                         'which face the cable leaves from needs '
                         'product.user_top_side' + tail)
                    continue
            if ce == 'in_plane' and not e.get('edge'):
                _row(ref, 'cable_envelope_mm', 'unmeasured',
                     'an in-plane band runs in from a declared edge, and '
                     'none is declared' + tail)
                continue
            name = f"cable:{ref}"
            if name in declared_ko:
                # The brief states this keep-out ITSELF: declared wins, and
                # the envelope is graded through it rather than replacing it
                # (Phase-5 verifier S4: replacing it dropped 3 hits to 0).
                _row(ref, 'cable_envelope_mm', 'compiled',
                     f"the brief declares keepouts[{name}] itself, and a "
                     f"declared keep-out wins over the one this envelope "
                     f"would derive" + tail,
                     compiled_to=f"keepouts[{name}]", grader='keepout',
                     basis='declared',
                     value=next(k.get('rect') or k.get('circle')
                                for k in keeps if k.get('name') == name))
                continue
            if pcb is None or ref not in (pcb.footprints or {}):
                _row(ref, 'cable_envelope_mm', 'withheld',
                     'no board to place the keep-out against' + tail)
                continue
            if ref not in locked:
                _row(ref, 'cable_envelope_mm', 'withheld',
                     f"lock {ref} to derive its cable keep-out: a keep-out "
                     f"off an unlocked part would move with every seed"
                     + tail)
                continue
            if geo is None:
                from .reconcile import _Geometry
                geo = _Geometry(pcb, board_path)
            body = geo.body_rect(ref)
            bounds = pcb.board_info.board_bounds if pcb.board_info else None
            if body is None or (ce == 'in_plane' and bounds is None):
                _row(ref, 'cable_envelope_mm', 'withheld',
                     'the part or the outline has no measurable geometry'
                     + tail)
                continue
            d = float(env[dim])
            if ce == 'in_plane':
                edge = e['edge']
                # The band guards the path a cable takes OUT of the board
                # from the part. A locked part that does not reach its
                # declared edge (the edge clause fails it) has no such path
                # there, and a band on that edge would only flag bystanders
                # (fixture 711's east band hit Q1, Q2, R3, R4; verifier N8).
                gap = {'west': body[0] - bounds[0],
                       'east': bounds[2] - body[2],
                       'north': body[1] - bounds[1],
                       'south': bounds[3] - body[3]}[edge]
                if gap > d:
                    _row(ref, 'cable_envelope_mm', 'withheld',
                         f"{ref}'s body sits {gap:.2f} mm from its declared "
                         f"{edge} edge, beyond the {d:g} mm band, so the "
                         f"band would guard no path of its cable; the edge "
                         f"clause reports the part" + tail)
                    continue
                rect = {'west': [bounds[0], body[1], bounds[0] + d, body[3]],
                        'east': [bounds[2] - d, body[1], bounds[2], body[3]],
                        'north': [body[0], bounds[1], body[2], bounds[1] + d],
                        'south': [body[0], bounds[3] - d, body[2],
                                  bounds[3]]}[edge]
                sides = ['F', 'B']
            else:
                rect = [body[0] - d, body[1] - d, body[2] + d, body[3] + d]
                sides = [face]
            rect = [round(v, 4) for v in rect]
            keeps.append({'name': name, 'rect': rect, 'sides': sides,
                          'allow': [glob.escape(ref)],
                          'context': {'source': 'brief',
                                      'derived_from':
                                      f"interfaces[{ref}].cable_envelope_mm",
                                      'basis': 'declared',
                                      'dimension': {dim: d}}})
            _row(ref, 'cable_envelope_mm', 'compiled',
                 f"the {ce} cable's declared {dim} of {d:g} mm around "
                 f"{ref}'s drawn body" + tail,
                 compiled_to=f"keepouts[{name}]", grader='keepout',
                 basis='declared', value=rect)
    if keeps:
        frag['keepouts'] = keeps
    if need_side:
        frag['min_reader'] = max(int(frag.get('min_reader') or 0), 6)
    compiled = {r['id'] for r in rows if r['status'] == 'compiled'}
    if used_uts:
        # The viewing face now decides a graded `side` (verifier S8).
        compiled.add('product.user_top_side')
    rep['not_graded'] = [x for x in (rep.get('not_graded') or ())
                         if x not in compiled]
    rep['consequences'] = rows
    return frag, rep


def compile_with_consequences(brief: 'Brief', pcb=None,
                              board_path: str = ''):
    """`compile_brief` and then `connector_consequences`: what every CLI
    that reads a brief against a board calls, so the emit path, the grade
    path, `--plan-only`, `board_brief` and P1 derive the SAME clauses -- a
    consequence only the emit path derived would never reach drift."""
    frag, rep = compile_brief(
        brief, board_refs=sorted((pcb.footprints or {}) if pcb else ()),
        refs_known=pcb is not None)
    return connector_consequences(frag, rep, pcb, board_path)


def merge_into_intent(emitted: Dict, fragment: Dict, report: Dict) -> Dict:
    """Merge a compiled fragment over an EMITTED intent. Declared wins.

    Per entry and per key: start from the emitted entry so its EVIDENCE
    survives (`observed_overhang_mm`, `suspect`, `suspect_reason`,
    `overhang_capped`), then overwrite each key the brief stated.

    `suspect` is deliberately never dropped. A brief naming an edge for a part
    the emitter marked SUSPECT keeps both: the suspect bit is evidence about
    the BOARD, the brief is a claim about the SPEC, and both are true at once.
    Dropping the first is how a damaged pose gets laundered into the spec that
    is supposed to gate its repair.
    """
    out = dict(emitted)
    by_ref = {c['ref']: dict(c) for c in (emitted.get('edge_connectors') or [])}
    order = [c['ref'] for c in (emitted.get('edge_connectors') or [])]
    for c in (fragment.get('edge_connectors') or []):
        ref = c['ref']
        base = by_ref.get(ref)
        if base is None:
            by_ref[ref] = dict(c)
            order.append(ref)
            continue
        # A contradiction is REPORTED and resolved in the brief's favour --
        # that is the whole point of the channel. The board is the inference.
        if base.get('edge') and c.get('edge') and base['edge'] != c['edge']:
            report['contradictions'].append(
                f"{ref}: the brief declares the {c['edge']} edge, the board "
                f"observes {base['edge']} -- the brief wins, and the grade "
                f"will flag the part")
        if base.get('source') and base['source'] != 'brief':
            base.setdefault('context', {})
            base['context'] = dict(base['context'])
            base['context']['was_source'] = base['source']
        merged_ctx = dict(base.get('context') or {})
        merged_ctx.update(c.get('context') or {})
        base.update({k: v for k, v in c.items() if k != 'context'})
        # The flag is a statement about THIS brief row, so a row that DOES
        # declare an edge must clear it. Without that, re-merging a document
        # that once carried `edge: "unknown"` drops the newly declared edge
        # while the promotion below still fires -- leaving an
        # `edge_receptacle` entry with no edge in `edge_claims()`, which the
        # engines read as a claim they cannot act on.
        if c.get('edge'):
            merged_ctx.pop('edge_declared_unknown', None)
        if merged_ctx.get('edge_declared_unknown'):
            base.pop('edge', None)
        base['context'] = merged_ctx
        # An entry that gains a declared edge must not keep an inferred
        # `class` of connector_affinity: that class is exactly "no edge claim",
        # and `Intent.edge_claims()` drops it, so the engines would ignore the
        # very declaration the author wrote.
        # `c.get('edge')` -- the BRIEF's edge, not `base`'s. Reading the
        # merged entry would let an emitter-INFERRED edge promote a row into
        # `Intent.edge_claims()`, which locks it in the quench and grants it
        # the 2.0mm off-outline allowance. Unreachable today (the emitter
        # never writes `edge` on a `connector_affinity` entry) and one
        # emitter change away from being a silent placement change.
        if c.get('edge') and base.get('class') == 'connector_affinity':
            base['class'] = 'edge_receptacle'
            merged_ctx['was_class'] = 'connector_affinity'
        by_ref[ref] = base
    if by_ref:
        out['edge_connectors'] = [by_ref[r] for r in sorted(set(order),
                                                            key=order.index)]
    if fragment.get('keepouts'):
        out['keepouts'] = list(emitted.get('keepouts') or []) \
            + list(fragment['keepouts'])
    # #902. APPENDED, like keepouts, and deliberately not the per-ref merge
    # `edge_connectors` gets: there is no "declared outranks inferred" question
    # to answer, because there is no inference. `emit_intent` writes no
    # `proximity` -- a relation between two parts is not readable off a board,
    # only the distance is -- so the emitted side is always empty and a
    # reconciliation policy would be a rule for a case that cannot arise.
    if fragment.get('proximity'):
        out['proximity'] = list(emitted.get('proximity') or []) \
            + list(fragment['proximity'])
    if fragment.get('min_reader'):
        # A MAX here too, and for the same reason it is one inside the
        # fragment: an emitted document that already declares a reader must not
        # be talked DOWN by a brief that needs a lower one. Unreachable today
        # (`emit_intent` writes no `min_reader`) and one emitter change away
        # from silently understating the reader a document needs -- which is a
        # false statement in the one field whose only job is to be true.
        try:
            have = int(out.get('min_reader') or 0)
        except (TypeError, ValueError):
            have = 0
        out['min_reader'] = max(have, int(fragment['min_reader']))

    ctx = dict(out.get('context') or {})
    ctx['brief'] = {k: report[k] for k in
                    ('path', 'declared', 'unknown', 'absent', 'not_graded',
                     'unmatched', 'unmatched_checked', 'contradictions',
                     'counts', 'fixed', 'product')}
    if report.get('consequences') is not None:
        ctx['brief']['consequences'] = report['consequences']
    # #959 comment 3.2: a key the brief states is no longer an observation.
    # The emitter labelled every number it chose `observed_baseline`; each
    # one the brief overwrote is re-labelled with the brief's own basis --
    # `declared`, or `derived_default` for a consequence's default.
    bmap = dict(ctx.get('basis') or {})
    for c in (fragment.get('edge_connectors') or []):
        cb = (c.get('context') or {}).get('basis') or {}
        for key in ('edge', 'overhang_mm', 'center_on_edge',
                    'along_edge_band', 'max_setback_mm', 'side'):
            if key in c:
                own = cb.get(key) or (cb.get('overhang_mm.min')
                                      if key == 'overhang_mm' else None)
                bmap[f"edge_connectors[{c['ref']}].{key}"] = own or 'declared'
    if bmap:
        ctx['basis'] = bmap
    if fragment.get('keepouts'):
        ctx['keepouts_note'] = (
            f"{len(fragment['keepouts'])} keep-out(s) DECLARED by the design "
            f"brief {os.path.basename(report.get('path') or '')}. A keep-out "
            f"is a mechanical fact and cannot be read off a board, so the "
            f"emitter writes none; these came from the one channel that can "
            f"state one. Since #701 the seat search honours them, not only "
            f"the grade.")
    if fragment.get('proximity'):
        ctx['proximity_note'] = (
            f"{len(fragment['proximity'])} proximity claim(s) DECLARED by the "
            f"design brief {os.path.basename(report.get('path') or '')}. "
            f"\"These two named parts, this far apart\" is a SPEC fact: the "
            f"board supplies the distance but never the claim, so the emitter "
            f"writes none and these came from the one channel that can state "
            f"one. They are graded from the intent, not by any seat search, "
            f"so a violation is a finding to act on rather than a pose the "
            f"engine will refuse.")
    out['context'] = ctx
    return out


def _drift_clause_id(kind, ref, key, *, near=None, fragment=None):
    """The clause id a drift line belongs to, or '' when there is none.

    Proximity ids carry the BRIEF ROW, which a compiled fragment entry records
    in `context.brief_row` -- so the id is recovered from the fragment rather
    than re-invented, and it matches the one `compile_brief` reported.
    """
    if kind == 'interfaces':
        # `center_on_edge` and `along_edge_band` are two spellings of the one
        # claim the brief calls `along_edge`, and the report names that.
        if key in ('center_on_edge', 'along_edge_band'):
            key = 'along_edge'
        # #959 (#1000): a key the consequence step COMPILED belongs to the
        # declaration it came from -- the setback to `mount_mode`, the face
        # to `cable_entry` or `user_facing`, a derived overhang floor to
        # `mount_mode`.
        entry = next((c for c in ((fragment or {}).get('edge_connectors')
                                  or ()) if c.get('ref') == ref), None)
        src = ((entry or {}).get('context') or {}).get('compiled_from') or {}
        key = src.get(key, key)
        return f"interfaces[{ref}].{key}"
    if kind == 'proximity' and fragment is not None:
        for p in (fragment.get('proximity') or ()):
            if str(p.get('ref')) == ref and str(p.get('near')) == near:
                row = (p.get('context') or {}).get('brief_row')
                if row is not None:
                    return f"{proximity_claim_id(row, ref, near)}.{key}"
    return ''


def drift_pairs(intent_doc: Dict, fragment: Dict) -> List[Tuple[str, str]]:
    """Brief claims an existing intent does NOT carry.

    `(clause_id, line)` for each divergence -- `drift()` is the line-only
    view of exactly this list, so the human sentence and the machine-readable
    clause id can never disagree about what drifted.

    For the `--intent` path, where merging would be wrong: the graded document
    must be the file the user pointed at, or every violation cites a claim its
    reader cannot find. So the brief is compiled, diffed, and REPORTED.

    A clause id is EMPTY when the divergence is not about one declared clause
    -- a keep-out the intent has never heard of, say. Those still drift and
    still block; they simply have no per-clause row to hang a flag on.
    """
    have = {c['ref']: c for c in (intent_doc.get('edge_connectors') or [])}
    out: List[str] = []
    for c in (fragment.get('edge_connectors') or []):
        ref = c['ref']
        cur = have.get(ref)
        if cur is None:
            out.append(('', f"{ref}: the brief declares this connector; "
                            f"the intent has no entry for it"))
            continue
        for key in ('edge', 'center_on_edge', 'along_edge_band',
                    'overhang_mm', 'max_setback_mm', 'side'):
            if key in c and cur.get(key) != c[key]:
                out.append((
                    _drift_clause_id('interfaces', ref, key,
                                     fragment=fragment),
                    f"{ref}.{key}: brief says {c[key]!r}, the intent "
                    f"{'says ' + repr(cur[key]) if key in cur else 'does not declare it'}"))
        # #959 (#1000): a compiled key the brief no longer produces, or a
        # vertical mount the two read differently, is drift too -- the
        # intent would grade a declaration the brief stopped making.
        csrc = ((cur.get('context') or {}).get('compiled_from') or {})
        for key in ('max_setback_mm', 'side'):
            if key in cur and key not in c and key in csrc:
                out.append((f"interfaces[{ref}].{csrc[key]}",
                            f"{ref}.{key}: the intent carries "
                            f"{cur[key]!r}, which the brief no longer "
                            f"compiles"))
        bm = (c.get('context') or {}).get('mount_mode')
        im = (cur.get('context') or {}).get('mount_mode')
        if bm != im and (bm in fp.VERTICAL_MOUNTS
                         or im in fp.VERTICAL_MOUNTS):
            out.append((f"interfaces[{ref}].mount_mode",
                        f"{ref}.mount_mode: brief says {bm!r}, the intent "
                        f"{im!r}"))
    have_k = {k.get('name'): k for k in (intent_doc.get('keepouts') or [])}
    for k in (fragment.get('keepouts') or []):
        kctx = k.get('context') or {}
        # A keep-out the consequence step derived belongs to the interface
        # clause it came from; a declared one to its own id.
        kid = (kctx.get('derived_from') or f"keepouts[{k.get('name')}]")
        cur = have_k.get(k.get('name'))
        if cur is None:
            out.append((kid if kctx.get('derived_from') else '',
                        f"keepout {k.get('name')!r}: declared by the "
                        f"brief, absent from the intent"))
            continue
        # By SHAPE, not only by name: a face change keeps the name (#959).
        for field_name in ('rect', 'circle', 'sides', 'allow'):
            if field_name in k and cur.get(field_name) != k[field_name]:
                out.append((kid, f"keepout {k.get('name')!r}.{field_name}: "
                                 f"brief says {k[field_name]!r}, the intent "
                                 f"{cur.get(field_name)!r}"))
    # A keep-out the intent carries because an envelope DERIVED it, which the
    # brief no longer derives (envelope removed, set "unknown", or its cable
    # changed): the seat search and the quench still enforce it, so it is a
    # stale consequence and drifts (Phase-5 verifier S5).
    frag_k = {k.get('name') for k in (fragment.get('keepouts') or [])}
    for name, cur in sorted(have_k.items(), key=lambda kv: str(kv[0])):
        src = (cur.get('context') or {}).get('derived_from')
        if src and name not in frag_k:
            out.append((src, f"keepout {name!r}: the intent carries it, "
                             f"derived from {src}, which the brief no "
                             f"longer derives it from"))
    # #902. Keyed on the ORDERED (ref, near) pair, which is the row's identity
    # in the brief too, so the two halves cannot disagree about what "the same
    # claim" means. The list `ref` was expanded by `compile_brief`, so both
    # sides here are already one-ref rows.
    have_prox = {(str(p.get('ref')), str(p.get('near'))): p
                 for p in (intent_doc.get('proximity') or [])}
    for p in (fragment.get('proximity') or []):
        key = (str(p.get('ref')), str(p.get('near')))
        cur = have_prox.get(key)
        if cur is None:
            out.append((
                _drift_clause_id('proximity', key[0], 'max_mm', near=key[1],
                                 fragment=fragment),
                f"{key[0]} near {key[1]}: the brief declares this proximity "
                f"claim; the intent has no row for it"))
            continue
        # Compared at the EFFECTIVE value, not `if field_name in p`. The brief
        # writes `basis` only when it is non-default, so a brief meaning the
        # documented default read as "declares nothing" and an intent row
        # saying `basis: "body"` drifted silently -- the grade would then have
        # measured bodies against a clause written about pads, and reported no
        # drift. A key absent on BOTH sides is not drift; a key absent on one
        # and declared on the other is.
        _defaults = {'basis': _PROXIMITY_DEFAULT_BASIS}
        for field_name in ('max_mm', 'basis', 'pads'):
            mine = p.get(field_name, _defaults.get(field_name))
            theirs = cur.get(field_name, _defaults.get(field_name))
            if mine is None and theirs is None:
                continue
            if mine != theirs:
                out.append((
                    _drift_clause_id('proximity', key[0], 'max_mm',
                                     near=key[1], fragment=fragment),
                    f"{key[0]}~{key[1]}.{field_name}: brief "
                    + (f"says {mine!r}" if mine is not None
                       else 'declares none')
                    + ', the intent '
                    + (f"says {theirs!r}" if theirs is not None
                       else 'does not declare it')))
    return out


def drift(intent_doc: Dict, fragment: Dict) -> List[str]:
    """The lines `drift_pairs` produces. The long-standing shape, unchanged."""
    return [line for _cid, line in drift_pairs(intent_doc, fragment)]


def drifted_clause_ids(intent_doc: Dict, fragment: Dict) -> List[str]:
    """The clause ids `drift_pairs` could attribute. Deduped, sorted."""
    return sorted({cid for cid, _line in drift_pairs(intent_doc, fragment)
                   if cid})




# --------------------------------------------------------------------------
# clause coverage (#902)
# --------------------------------------------------------------------------

#: Which rule grades a clause of each kind. `product` is graded by nothing and
#: says so; `fixed` never reaches a rule at all (it is carried into `context`).
_CLAUSE_RULE = {'interfaces': 'edge_connector',
                'keepouts': 'keepout',
                'proximity': 'proximity',
                'product': None}

#: Interface keys that are CARRIED and graded by nothing, by design. They are
#: already in `report['not_graded']`; naming them here too keeps the coverage
#: verdict from depending on a list that exists for a different purpose.
_CLAUSE_CARRIED = {'mount_mode', 'cable_entry', 'cable_envelope_mm'}

_CLAUSE_RE = re.compile(
    r'^(?P<kind>interfaces|keepouts|proximity|product)'
    r'(?:\[(?P<inner>.*)\])?'
    r'(?:\.(?P<key>[a-z_]+))?$')


def parse_clause_id(cid: str) -> Optional[Dict[str, object]]:
    """`proximity[0:Y1~U1].max_mm` -> its parts, or None if it is not one.

    ONE parser for a format built in seventeen places. That is a real risk --
    a second implementation of a string format is how two halves drift -- so
    `tests/test_902_proximity.py` round-trips EVERY id `compile_brief`
    produces through this function and fails on one it cannot read. An id that
    stops parsing is then a test failure rather than a clause that silently
    vanishes from the coverage report.

    Returns `{kind, ref, near, key, row}`; `ref` is the keep-out NAME for a
    keep-out, and `near`/`row` are set only for a proximity claim.
    """
    m = _CLAUSE_RE.match(cid)
    if m is None:
        return None
    kind, inner, key = m.group('kind'), m.group('inner'), m.group('key')
    out: Dict[str, object] = {'kind': kind, 'ref': inner, 'near': None,
                              'key': key, 'row': None}
    if kind == 'product':
        out['ref'] = None
        return out
    if inner is None:
        return None
    if kind == 'proximity':
        # `<row>:<ref>~<near>`. The row index is what makes the id unique when
        # a reference itself contains `~` (`TP4~2` is a refdes this toolchain
        # PRODUCES), so it is split off first and the rest is split on the
        # LAST `~`: a ref may contain one, a `near` that contains one still
        # leaves the final separator as the boundary only if we split from the
        # left -- so both are recovered from the fragment instead, and this
        # parse is used for the row and kind alone when they disagree.
        row, _, rest = inner.partition(':')
        if not row.isdigit() or '~' not in rest:
            return None
        ref, _, near = rest.partition('~')
        out.update({'row': int(row), 'ref': ref, 'near': near})
    return out


def _intent_has(intent_doc, path: str) -> bool:
    """Does the intent carry `edge_connectors[REF].key[.sub]` or
    `keepouts[NAME]` -- the path a consequence row compiled to."""
    m = re.match(r'^(edge_connectors|keepouts)\[(.*?)\](?:\.(.*))?$',
                 path or '')
    if not m:
        return False
    kind, name, rest = m.groups()
    if kind == 'keepouts':
        return any(k.get('name') == name
                   for k in (intent_doc.get('keepouts') or ()))
    node = next((c for c in (intent_doc.get('edge_connectors') or ())
                 if c.get('ref') == name), None)
    for part in (rest or '').split('.'):
        if not part:
            continue
        if not isinstance(node, dict) or part not in node:
            return False
        node = node[part]
    return node is not None


def _clause_state(rec, intent_doc, rules_run, abstained, cons=None):
    """The verdict for ONE declared clause. Five states, kept apart.

    `graded` is the only one that means a rule reached a verdict. Collapsing
    the other four into "not graded" would rebuild the failure this whole
    channel exists against one key over: run 25 passed with `rules_run: 6` and
    every declared clause unmeasured, because a COUNT cannot say WHICH.
    """
    kind, ref, near, key = rec['kind'], rec['ref'], rec['near'], rec['key']
    rule = _CLAUSE_RULE.get(kind)
    if kind == 'interfaces' and key in _CLAUSE_CARRIED:
        # #959 (#1000): graded when `connector_consequences` compiled it
        # and the intent carries what it compiled to; otherwise carried,
        # and an UNMEASURED one says which dimension nobody declared.
        row = (cons or {}).get(f"interfaces[{ref}].{key}")
        if row is None:
            return 'carried', '', rule
        if row['status'] == 'withheld':
            # Declared, and derivable once the board allows it (a lock, an
            # outline): an ABSTENTION, which keeps coverage incomplete --
            # not a fact carried by design (Phase-5 verifier S6).
            return ('abstained', f"withheld: {row['why']}",
                    row.get('grader') or 'keepout')
        if row['status'] != 'compiled':
            return 'carried', f"{row['status']}: {row['why']}", None
        if not _intent_has(intent_doc, row['compiled_to']):
            return ('uncovered', f"the intent does not carry "
                                 f"{row['compiled_to']}, which the brief's "
                                 f"{key} compiles to", rule)
        rule = 'edge_connector' if row['grader'] in (
            'edge_connector', 'edge_connector_side') else row['grader']
        if rule not in rules_run:
            return ('uncovered', f"`{rule}`, which grades what {key} "
                                 f"compiles to, did not run", rule)
        return 'graded', '', rule
    if kind == 'product' and key == 'user_top_side':
        # Graded once a perpendicular cable turns it into a `side` (S8).
        sides = [r for r in (cons or {}).values()
                 if r.get('status') == 'compiled'
                 and r.get('grader') == 'edge_connector_side'
                 and _intent_has(intent_doc, r.get('compiled_to'))]
        if sides and 'edge_connector' in rules_run:
            return 'graded', '', 'edge_connector'
    if rule is None:
        return 'carried', '', rule
    if kind == 'interfaces' and key == 'user_facing':
        entry = next((c for c in (intent_doc.get('edge_connectors') or [])
                      if c.get('ref') == ref), None)
        mm = ((entry or {}).get('context') or {}).get('mount_mode')
        if mm in fp.VERTICAL_MOUNTS:
            # What grades `user_facing` is the receptacle seat, and a
            # vertical mount is exempt from it (Phase-5 verifier S2).
            return ('carried', f"a {mm} part is exempt from the "
                               f"edge-receptacle seat, which is what grades "
                               f"user_facing", None)
    if kind == 'interfaces':
        entry = next((c for c in (intent_doc.get('edge_connectors') or [])
                      if c.get('ref') == ref), None)
        if entry is None:
            return ('uncovered', f"the intent has no edge_connectors entry for "
                                 f"{ref}", rule)
        carried = {
            'edge': 'edge' in entry,
            'overhang_mm': 'overhang_mm' in entry,
            'user_facing': entry.get('class') == 'edge_receptacle',
            'along_edge': ('center_on_edge' in entry
                           or 'along_edge_band' in entry),
        }.get(key)
        if carried is False:
            return ('uncovered', f"the intent's {ref} entry does not carry "
                                 f"{key}", rule)
    elif kind == 'keepouts':
        if not any(k.get('name') == ref
                   for k in (intent_doc.get('keepouts') or [])):
            return ('uncovered', f"the intent carries no keepout named "
                                 f"{ref!r}", rule)
    elif kind == 'proximity':
        if not any(p.get('ref') == ref and p.get('near') == near
                   for p in (intent_doc.get('proximity') or [])):
            return ('uncovered', f"the intent carries no proximity claim for "
                                 f"{ref} near {near}", rule)
    if rule not in rules_run:
        return ('uncovered', f"`{rule}` did not run on this grade", rule)
    for akey, why in sorted((abstained or {}).items()):
        if _abstention_is_about(akey, kind, ref, near, intent_doc):
            return 'abstained', why, rule
    return 'graded', '', rule


_ABSTAIN_PROX_RE = re.compile(r'^proximity\[(?P<row>\d+):')


def _abstention_is_about(akey, kind, ref, near, intent_doc) -> bool:
    """Does this abstention key name THIS clause?

    Resolved through the intent's own ROW for a proximity claim, never by
    matching `ref~near` as a string: a reference may contain `~`
    (`disambiguate_references` produces `TP4~2`, and esp_prog parses
    `Ref*~2`), so a row `A~B` near `C` and a row `A` near `B~C` spell the same
    thing. The rule writes the row index into the key precisely so this lookup
    can be exact rather than a prefix guess.
    """
    if kind == 'interfaces':
        return akey.startswith(f"edge_connectors[{ref}].")
    if kind != 'proximity':
        return False
    m = _ABSTAIN_PROX_RE.match(akey)
    if m is None:
        return False
    rows = intent_doc.get('proximity') or []
    row = int(m.group('row'))
    if row >= len(rows):
        return False
    # BOTH halves must agree, and the negative control is what forced this:
    # trusting the index alone attributed a key spelled `[0:Q1~Q2]` to
    # whatever claim happened to sit at row 0. The index disambiguates the
    # refs (a reference may contain `~`) and the text confirms the index, so a
    # stale or hand-edited key attributes to nothing rather than to the wrong
    # clause -- an abstention charged to an innocent claim is worse than one
    # nobody attributes, because it reads as a finding about that claim.
    if not akey.startswith(f"proximity[{row}:{ref}~{near}]"):
        return False
    return (str(rows[row].get('ref')) == ref
            and str(rows[row].get('near')) == near)


def clause_coverage(report: Dict, intent_doc: Dict, *,
                    rules_run: Sequence[str] = (),
                    abstained: Optional[Dict[str, str]] = None,
                    drifted_ids: Sequence[str] = ()) -> Dict:
    """Did a rule reach a verdict on every clause the brief DECLARED? (#902)

    `--require-rules` counts RULES. Run 25 ran six of them, passed, and graded
    not one clause its brief declared -- the count was satisfied by rules
    nobody had declared anything for. This counts CLAUSES, which is the thing
    the author actually wrote.

    PURE: no file IO and no board. The same split `compile_brief` has, and for
    the same reason -- it is testable against a hand-built intent document.
    """
    drift_set = set(drifted_ids or ())
    cons = {r['id']: r for r in (report.get('consequences') or ())}
    clauses = []
    counts = {'graded': 0, 'abstained': 0, 'uncovered': 0,
              'not_claimed': 0, 'carried': 0, 'drifted': 0}
    for cid in list(report.get('declared') or ()):
        rec = parse_clause_id(cid)
        if rec is None:
            continue
        state, why, rule = _clause_state(rec, intent_doc, tuple(rules_run),
                                         abstained, cons)
        row = {'id': cid, 'kind': rec['kind'], 'ref': rec['ref'],
               'rule': rule, 'state': state, 'why': why,
               'drifted': cid in drift_set}
        # The FINDING that grades it, where it differs from the rule that
        # runs it (the side finding runs inside `edge_connector`), and the
        # keep-out a cable envelope compiled to -- so the ledger attributes
        # a verdict to this clause, not to whatever else the rule flagged.
        crow = cons.get(cid) or {}
        if state == 'graded' and crow.get('grader'):
            row['grader'] = crow['grader']
            if crow['grader'] == 'keepout':
                row['keepout'] = str(crow.get('compiled_to') or '')[
                    len('keepouts['):-1]
        if rec['kind'] == 'product' and state == 'graded':
            row['grader'] = 'edge_connector_side'
        if why.startswith('unmeasured: '):
            row['unmeasured'] = why.split(': ', 1)[1]
        elif why.startswith('withheld: '):
            row['withheld'] = why.split(': ', 1)[1]
        clauses.append(row)
        counts[state] += 1
        if row['drifted']:
            counts['drifted'] += 1
    for cid in list(report.get('unknown') or ()):
        rec = parse_clause_id(cid)
        if rec is None:
            # A free-text entry from the brief's own `unknown[]` list, which
            # names a QUESTION rather than a clause ("mounting_datum"). It is
            # reported by `brief_unknown_keys` already; it is not a clause and
            # must not be counted as one.
            continue
        clauses.append({'id': cid, 'kind': rec['kind'], 'ref': rec['ref'],
                        'rule': _CLAUSE_RULE.get(rec['kind']),
                        'state': 'not_claimed',
                        'why': 'the brief declares this "unknown"',
                        'drifted': False})
        counts['not_claimed'] += 1
    for cid in list(report.get('not_graded') or ()):
        rec = parse_clause_id(cid)
        if rec is None or any(c['id'] == cid for c in clauses):
            continue
        clauses.append({'id': cid, 'kind': rec['kind'], 'ref': rec['ref'],
                        'rule': None, 'state': 'carried',
                        'why': 'carried into context; nothing grades it',
                        'drifted': False})
        counts['carried'] += 1
    clauses.sort(key=lambda c: c['id'])
    out = {'schema': 1, 'brief': os.path.basename(report.get('path') or '')
           or None, 'clauses': clauses}
    out.update(counts)
    # COMPLETE means every clause that could reach a verdict did. `carried`
    # and `not_claimed` never block: one is ungraded by design and the other
    # is an author saying "I do not know", and punishing an honest unknown is
    # how a channel teaches people to stop declaring.
    out['complete'] = (counts['uncovered'] == 0 and counts['abstained'] == 0
                       and counts['drifted'] == 0)
    return out


def format_report(report: Dict, *, path: str = '') -> str:
    """The one line every run prints, found or not.

    A silent absence is the failure this module exists to fix, so the
    not-found branch says what is filling the gap instead.
    """
    if not path and not report:
        return ''
    c = report.get('counts') or {}
    bits = [f"{c.get('interfaces', 0)} interface(s)",
            f"{c.get('keepouts', 0)} keep-out(s)"]
    if c.get('proximity'):
        # Printed whenever any row was written, even if every one of them
        # declared `max_mm: "unknown"` and compiled to nothing -- a declared
        # claim nobody prints is invisible, which is this module's own
        # argument.
        #
        # The `-> N claim(s)` suffix keys on the EVENTS, not on whether the two
        # counts differ. They can cancel exactly: one row expanding by one
        # while another drops out leaves 3 rows and 3 claims, and the first
        # version of this line then printed nothing at all -- on the very brief
        # this feature was built for.
        claims = c.get('proximity_claims', c['proximity'])
        moved = c.get('proximity_expanded') or c.get('proximity_dropped')
        bits.append(f"{c['proximity']} proximity row(s)"
                    + (f" -> {claims} claim(s)" if moved else ''))
    if c.get('fixed'):
        bits.append(f"{c['fixed']} fixed pose(s), carried not asserted")
    if report.get('unknown'):
        bits.append(f"{len(report['unknown'])} declared UNKNOWN")
    if report.get('absent'):
        bits.append(f"{len(report['absent'])} tier-0 field(s) not declared")
    return f"design brief {os.path.basename(path)}: " + ', '.join(bits)


def format_absent_note(board_path: str) -> str:
    return (f"design brief: none beside this board (looked for "
            f"{os.path.basename(brief_path_for(board_path))}). Every `edge` "
            f"below is INFERRED from the part's current pose by "
            f"_nearest_edge; nothing here declares design intent.")
