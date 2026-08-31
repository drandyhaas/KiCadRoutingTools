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
  becomes an ordinary `edge_connectors[]` or `keepouts[]` entry that the
  existing rules already grade and the existing seat search already honours.
  The only intent keys this work adds are #712's two; provenance goes in
  `source` and `context`, which the schema already accepts. A brief that could
  express something the intent cannot would be a constraint nothing checks.

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
import json
import os
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
                   'context'}

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
                   'cable_entry', 'requirement', 'why', 'note', 'context'}
#: The intent's own keep-out shape, plus two slots for reasoning that has
#: nowhere else to go. Both land in the compiled entry's `context`.
_KEEPOUT_KEYS = {'name', 'rect', 'circle', 'sides', 'allow', 'kind', 'why',
                 'note', 'context'}
_FIXED_KEYS = {'ref', 'why', 'requirement', 'context'}
_BAND_KEYS = {'from', 'to'}
_OVERHANG_KEYS = {'min', 'max'}

_PRIMARY_AXIS = ('east-west', 'north-south', UNKNOWN)
_SIDES = ('F', 'B', UNKNOWN)
_MOUNT_MODES = ('edge_mount', 'top_mount', 'bottom_mount', 'through_edge',
                UNKNOWN)
_CABLE_ENTRY = ('in_plane', 'perpendicular_top', 'perpendicular_bottom',
                'none', UNKNOWN)

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
    context: Dict[str, object] = field(default_factory=dict)
    source_path: str = ''


def empty_brief(board: str = '') -> Brief:
    return Brief(schema=SCHEMA_VERSION, kind=KIND, board=board, units='mm',
                 product={}, interfaces=(), keepouts=(), fixed=(),
                 unknown=(), context={})


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
                 unknown=unknown,
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
            entry['note'] = ('the brief declares this connector with edge '
                             '"unknown" -- no edge is claimed, and none is '
                             'inferred here')

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

    fragment: Dict[str, object] = {}
    if conns:
        fragment['edge_connectors'] = conns
    if keeps:
        fragment['keepouts'] = keeps
    if wrote_along_edge:
        # The first real use of the mechanism `min_reader` was built for: a
        # reader that predates #712 must refuse this document rather than
        # grade it without the along-edge claim.
        fragment['min_reader'] = 2

    absent = [k for k in _TIER0
              if (k == 'interfaces' and not brief.interfaces)
              or (k.startswith('product.')
                  and brief.product.get(k.split('.', 1)[1]) is None)]
    report = {
        'path': brief.source_path,
        'declared': sorted(declared),
        'unknown': sorted(set(unknown) | set(brief.unknown)),
        'absent': absent,
        'not_graded': sorted(not_graded),
        'unmatched': sorted(unmatched),
        'unmatched_checked': bool(refs_known and (board_refs or ())),
        'contradictions': [],
        'counts': {'interfaces': len(brief.interfaces),
                   'keepouts': len(brief.keepouts),
                   'fixed': len(brief.fixed)},
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
        base['context'] = merged_ctx
        # An entry that gains a declared edge must not keep an inferred
        # `class` of connector_affinity: that class is exactly "no edge claim",
        # and `Intent.edge_claims()` drops it, so the engines would ignore the
        # very declaration the author wrote.
        if base.get('edge') and base.get('class') == 'connector_affinity':
            base['class'] = 'edge_receptacle'
            merged_ctx['was_class'] = 'connector_affinity'
        by_ref[ref] = base
    if by_ref:
        out['edge_connectors'] = [by_ref[r] for r in sorted(set(order),
                                                            key=order.index)]
    if fragment.get('keepouts'):
        out['keepouts'] = list(emitted.get('keepouts') or []) \
            + list(fragment['keepouts'])
    if fragment.get('min_reader'):
        out['min_reader'] = fragment['min_reader']

    ctx = dict(out.get('context') or {})
    ctx['brief'] = {k: report[k] for k in
                    ('path', 'declared', 'unknown', 'absent', 'not_graded',
                     'unmatched', 'unmatched_checked', 'contradictions',
                     'counts', 'fixed', 'product')}
    if fragment.get('keepouts'):
        ctx['keepouts_note'] = (
            f"{len(fragment['keepouts'])} keep-out(s) DECLARED by the design "
            f"brief {os.path.basename(report.get('path') or '')}. A keep-out "
            f"is a mechanical fact and cannot be read off a board, so the "
            f"emitter writes none; these came from the one channel that can "
            f"state one. Since #701 the seat search honours them, not only "
            f"the grade.")
    out['context'] = ctx
    return out


def drift(intent_doc: Dict, fragment: Dict) -> List[str]:
    """Brief claims an existing intent does NOT carry.

    For the `--intent` path, where merging would be wrong: the graded document
    must be the file the user pointed at, or every violation cites a claim its
    reader cannot find. So the brief is compiled, diffed, and REPORTED.
    """
    have = {c['ref']: c for c in (intent_doc.get('edge_connectors') or [])}
    out: List[str] = []
    for c in (fragment.get('edge_connectors') or []):
        ref = c['ref']
        cur = have.get(ref)
        if cur is None:
            out.append(f"{ref}: the brief declares this connector; the intent "
                       f"has no entry for it")
            continue
        for key in ('edge', 'center_on_edge', 'along_edge_band', 'overhang_mm'):
            if key in c and cur.get(key) != c[key]:
                out.append(f"{ref}.{key}: brief says {c[key]!r}, the intent "
                           f"{'says ' + repr(cur[key]) if key in cur else 'does not declare it'}")
    names = {k.get('name') for k in (intent_doc.get('keepouts') or [])}
    for k in (fragment.get('keepouts') or []):
        if k.get('name') not in names:
            out.append(f"keepout {k.get('name')!r}: declared by the brief, "
                       f"absent from the intent")
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
