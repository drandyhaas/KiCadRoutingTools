"""
Write placed PCB output by modifying footprint positions in the .kicad_pcb file.

Uses text-based manipulation (same approach as output_writer.py / kicad_writer.py)
to update the (at X Y [rotation]) of each footprint block.
"""
from __future__ import annotations

import os
import re
import sys
from typing import List, Dict

from kicad_parser import (find_matching_paren, flip_layer_token,
                          iter_footprint_blocks)
from kicad_writer import move_copper_text_to_silkscreen


class OutlineOwnerMove(Exception):
    """A placement list asked to move a footprint that draws the board outline.

    #829. Raised rather than skipped -- see the call site for why a skip is
    worse than a refusal here.
    """


# Cheap pre-filter: only a footprint block that mentions Edge.Cuts can possibly
# own outline geometry, and on an ordinary board no block does, so the real
# (parsing) check below never runs.
_OWNS_OUTLINE_RE = re.compile(r'"Edge\.Cuts"')

_OUTLINE_OWNER_CACHE: Dict[tuple, Dict[str, bool]] = {}

# A pose the caller asked for that differs from the current one by less
# than this is not a move. 1 nm, KiCad's own quantum.
_POSE_EPS = 1e-6


def _outline_owner_map(input_file: str) -> Dict[str, bool]:
    """`{ref: draws_the_board_outline}` for `input_file`, memoised.

    Memoised because `write_placed_output` is called once per board, not once
    per ref, and the classifier re-reads the file.
    """
    try:
        st = os.stat(input_file)
        key = (os.path.abspath(input_file), st.st_mtime_ns, st.st_size)
    except OSError:
        return {}
    # Keyed by (path, mtime, size), NOT by path alone. `place_seed` writes a
    # temp board and `os.replace`s it over the output, and `route.py` writes in
    # place, so a path-keyed memo answers for a file that no longer exists --
    # measured: after the file changed on disk the writer still reported the
    # old owners. In a long-lived plugin process a path-keyed dict also grows
    # without bound.
    if key not in _OUTLINE_OWNER_CACHE:
        try:
            from kicad_parser import footprint_outline_owners
            with open(input_file, encoding='utf-8') as fh:
                _OUTLINE_OWNER_CACHE[key] = footprint_outline_owners(fh.read())
        except Exception:                                     # noqa: BLE001
            # Cannot decide -> do not invent a refusal. The movable-set gates
            # are the primary defence; a backstop that failed closed on an
            # unreadable file would break every ordinary run.
            #
            # Deliberately BROADER than (OSError, ValueError), which is what
            # this caught first: the classifier walks paren structure, so an
            # IndexError out of `find_matching_paren` on a malformed board is
            # reachable, and letting it escape would make a guard documented
            # as failing OPEN take the whole run down instead.
            _OUTLINE_OWNER_CACHE[key] = {}
    return _OUTLINE_OWNER_CACHE[key]


def _draws_board_outline(input_file: str, ref: str) -> bool:
    """Does `ref` draw geometry OUTSIDE the board-level outline (#829)?

    Answered by the parser's own classifier, never by a second predicate here:
    `owns_edge_cuts` is not the question -- a relief parented to a part travels
    with it and must keep moving (crkbd's 184 per-LED windows, #628).
    """
    return bool(_outline_owner_map(input_file).get(ref))


def _fmt_mm(v: float) -> str:
    """KiCad-style minimal decimal formatting (nanometre-faithful)."""
    s = f"{v:.6f}".rstrip('0').rstrip('.')
    return '0' if s in ('', '-0') else s


def _edit_reference_node(node: str, res) -> str:
    """Apply one LabelResult to a Reference text sub-node (#481).

    Edits ONLY: the first ``(at ...)``, the font ``(size H W)`` /
    ``(thickness T)`` (whole effects/font blocks inserted when absent), and
    the ``(justify ...)`` token -- alignment tokens are dropped (the engine
    places CENTERED boxes) but ``mirror`` is PRESERVED (glasgow's 94 B-side
    refs carry it; dropping it would mirror-flip every one). layer / uuid /
    tstamp / hide pass through untouched.
    """
    at_m = re.search(r'\(at\s+[\d.-]+\s+[\d.-]+(?:\s+[\d.-]+)?\)', node)
    if not at_m:
        return node  # no (at ...): not a label the parser modelled; leave it
    rot = res.file_rotation % 360
    new_at = f"(at {_fmt_mm(res.at_x)} {_fmt_mm(res.at_y)}" \
        + (f" {rot:.6g})" if rot else ")")
    node = node[:at_m.start()] + new_at + node[at_m.end():]

    size_sexpr = f"(size {_fmt_mm(res.size)} {_fmt_mm(res.size)})"
    thick_sexpr = f"(thickness {_fmt_mm(res.thickness)})"

    eff_m = re.search(r'\(effects\b', node)
    if not eff_m:
        # No effects block at all (parser defaulted 1.0/0.15): insert a full
        # one before the node's closing paren.
        close = node.rindex(')')
        return (node[:close].rstrip()
                + f" (effects (font {size_sexpr} {thick_sexpr})))"
                + node[close + 1:])

    eff_end = find_matching_paren(node, eff_m.start())
    eff = node[eff_m.start():eff_end]

    font_m = re.search(r'\(font\b', eff)
    if not font_m:
        eff = (eff[:len('(effects')]
               + f" (font {size_sexpr} {thick_sexpr})"
               + eff[len('(effects'):])
    else:
        font_end = find_matching_paren(eff, font_m.start())
        font = eff[font_m.start():font_end]
        font, n_size = re.subn(r'\(size\s+[\d.-]+\s+[\d.-]+\)', size_sexpr,
                               font, count=1)
        font, n_th = re.subn(r'\(thickness\s+[\d.-]+\)', thick_sexpr,
                             font, count=1)
        insert = ('' if n_size else f" {size_sexpr}") \
            + ('' if n_th else f" {thick_sexpr}")
        if insert:
            font = font[:len('(font')] + insert + font[len('(font'):]
        eff = eff[:font_m.start()] + font + eff[font_end:]

    jm = re.search(r'\(justify\s+([^)]*)\)', eff)
    if jm:
        keep = ' (justify mirror)' if 'mirror' in jm.group(1).split() else ''
        eff = (eff[:jm.start()].rstrip() + keep + eff[jm.end():]) \
            if keep else re.sub(r'\s*\(justify\s+[^)]*\)', '', eff, count=1)

    return node[:eff_m.start()] + eff + node[eff_end:]


def _report_unapplied(requested, matched, unaddressed, what):
    """Say which requested names reached no block, instead of dropping them.

    Before #726 an entry that matched nothing was silently discarded: a typo in
    `--ref`, a reference that only exists on a different board, or a name the
    caller built from a stale parse all produced a successful-looking write
    that did nothing. The writer is the last place that can still tell.

    `unaddressed` is the narrower case -- a block the name DID resolve to, but
    which the writer could not act on: no `(at ...)` to rewrite, or no
    Reference node to edit.

    ON STDERR, for the same reason the parser's duplicate-reference warning is
    (`kicad_parser.py`): shipped tools emit bare JSON on stdout, where a
    WARNING line in front is a JSONDecodeError at char 0. `converge.py` happens
    to wrap its write in `_StdoutToStderr()`, which is luck rather than a
    contract.
    """
    missing = sorted(set(requested) - set(matched))
    if missing:
        print(f"WARNING: {len(missing)} {what}(s) matched no footprint block: "
              + ', '.join(repr(m) for m in missing[:12])
              + (' ...' if len(missing) > 12 else ''), file=sys.stderr)
    if unaddressed:
        print(f"WARNING: {len(unaddressed)} {what}(s) resolved to a block with "
              f"no `(at ...)` or no Reference node, so nothing was written: "
              + ', '.join(repr(u) for u in sorted(unaddressed)[:12]),
              file=sys.stderr)


def write_label_output(input_file: str, output_file: str,
                       results: List) -> bool:
    """Write a beautified-labels PCB file (#481).

    Text-surgical, the `write_placed_output` discipline: every edit is
    strictly INSIDE a Reference text sub-node (modern
    ``(property "Reference" ...)`` or KiCad 6/7 ``(fp_text reference ...)``
    -- alternate header, same inner grammar), so masking every Reference
    sub-node from input and output yields byte-identical files (the
    acceptance property tests/test_beautify_labels.py pins). Copper, zones,
    footprint poses, and every untouched label pass through byte-for-byte.
    Deliberately does NOT call `move_copper_text_to_silkscreen`: this writer
    must touch nothing but Reference nodes.

    `results` are `placement.labels.LabelResult`s; only entries whose
    `.changed` is true are applied.
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    by_ref = {r.reference: r for r in results
              if getattr(r, 'changed', False)}
    modified_count = 0
    matched = set()
    unaddressed = []

    # Blocks are NAMED by the parser's own resolver (#726), so a label result
    # for `TP4` edits the one block the parser calls `TP4` -- not every block
    # that happens to carry that string. Reverse order so string indices stay
    # valid after replacements; the spans come from the string-aware block scan
    # (the same #113 hazard as above).
    for start, end, fp_text, _raw_ref, key in reversed(
            list(iter_footprint_blocks(content))):
        res = by_ref.get(key)
        if res is None:
            continue
        # MATCHED, not yet applied: a result whose node already reads the way
        # we would write it is a no-op, not an unresolved name.
        matched.add(key)

        ref_m = (re.search(r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
                 or re.search(r'\(fp_text\s+reference\s+"([^"]+)"', fp_text))
        if not ref_m:
            # A reference-LESS block has no Reference node to edit. The parser
            # keys it `#uuid` and `placement/labels.py` already skips those, so
            # a result for one should not exist -- and if one ever does, it
            # is named rather than dropped in silence.
            unaddressed.append(key)
            continue

        node_end = find_matching_paren(fp_text, ref_m.start())
        node = fp_text[ref_m.start():node_end]
        new_node = _edit_reference_node(node, res)
        if new_node == node:
            continue
        fp_text = fp_text[:ref_m.start()] + new_node + fp_text[node_end:]
        content = content[:start] + fp_text + content[end:]
        modified_count += 1

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"Modified {modified_count} reference label(s)")
    _report_unapplied(by_ref, matched, unaddressed, 'label result')
    print(f"Successfully wrote {output_file}")
    return True


def _rotate_pad_angles(fp_text: str, delta_rot: float) -> str:
    """Add delta_rot to the (at x y [angle]) of every pad in a footprint block."""

    def fix_pad(m):
        head, x, y, angle = m.group(1), m.group(2), m.group(3), m.group(4)
        new_angle = ((float(angle) if angle else 0.0) + delta_rot) % 360
        if new_angle != 0:
            # :.6g, not :.4g -- 4 significant digits rounds a 137.25deg pad to
            # 137.2. x/y pass through as text here, so only the angle is at risk.
            return f'{head}(at {x} {y} {new_angle:.6g})'
        return f'{head}(at {x} {y})'

    return re.sub(
        r'(\(pad\s+"[^"]*"\s+\S+\s+\S+\s*\n?\s*)'
        r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)',
        fix_pad, fp_text)


class SideFlipUnsupported(Exception):
    """A flip was asked for on a footprint carrying a construct we will not guess at.

    #714. RAISES rather than passing the construct through untouched, and the
    argument is the same one `OutlineOwnerMove` above already makes:
    `write_placed_output` returns True unconditionally, so a construct the
    transform quietly skipped is indistinguishable from success at every call
    site -- and a half-mirrored footprint is the SILENT failure this whole
    feature is written around. `legality.footprint_side` reads `fp.layer[0]`
    and nothing cross-checks the pads, so the board would grade plausibly and
    wrongly forever.

    Refusing is cheap, measured over the 22 tracked boards (1349 footprint
    blocks): the refusal set touches at most 7 of them -- `primitives` 6,
    `zone` 1 -- while `chamfer`, `rect_delta`, `fp_text_box` and `group` have
    NO witness at all. A construct the corpus does not contain is a construct
    no gate here can validate, so any rule for it would be an unfalsifiable
    guess.
    """


# A coordinate literal, as this corpus actually spells them: measured 148417
# tokens across every `(at|start|end|mid|center|xy|offset ...)` on every tracked
# board, with ZERO outside this shape -- no leading `+`, no exponent, no `-0`.
# That is what makes the sign TOGGLE below an exact involution, and it is why a
# literal outside the shape is a refusal rather than a `float()` round trip.
_COORD_LITERAL = re.compile(r'^-?\d+(\.\d+)?$')

# Top-level children of a `(footprint ...)` block, by what the flip does with
# them. Anything else REFUSES -- a whitelist, because you cannot test for a
# node kind you have not thought of.
#
# TWO DIFFERENT COUNTS, and an earlier version of this comment read them as one
# measured claim: the tracked corpus has 27 distinct top-level child kinds, and
# this set also happens to hold 27 names, but they are not the same 27. Fifteen
# of these have a corpus witness; TWELVE DO NOT -- `autoplace_cost90`,
# `autoplace_cost180`, `clearance`, `generator`, `generator_version`,
# `jumper_pad_groups`, `net_tie_pad_groups`, `thermal_gap`, `thermal_width`,
# `tstamp`, `version`, `zone_connect`.
#
# Which raises the obvious objection: `chamfer` and `rect_delta` are refused
# for having no witness, so why are these passed through? Because the doctrine
# is about GEOMETRY, not about familiarity. Every name here is a scalar, an
# identifier or a flag that a reflection cannot act on -- a cost weight, a
# uuid, a margin, a pad-number group, a version string. `chamfer` and
# `rect_delta` describe SHAPE, and their behaviour under a mirror is a real
# question with a real answer that this corpus cannot supply. Passing through a
# number that has no orientation is not a guess; guessing how a corner name
# reflects is.
_FLIP_PASSTHROUGH = frozenset({
    'uuid', 'descr', 'tags', 'path', 'attr', 'embedded_fonts', 'sheetfile',
    'sheetname', 'units', 'locked', 'variant', 'solder_mask_margin',
    'solder_paste_margin', 'duplicate_pad_numbers_are_jumpers',
    'net_tie_pad_groups', 'jumper_pad_groups', 'tstamp', 'autoplace_cost90',
    'autoplace_cost180', 'clearance', 'zone_connect', 'thermal_width',
    'thermal_gap', 'generator', 'generator_version', 'version',
    # The 3D model is left COMPLETELY alone: the viewer applies the flip at
    # render time, so mirroring its offset double-applies it. Probed on
    # tigard J1 and orangecrab J4 -- pcbnew leaves both byte-identical.
    'model',
})
_FLIP_GRAPHICS = frozenset({'fp_line', 'fp_rect', 'fp_circle', 'fp_poly',
                            'fp_curve', 'fp_arc'})
_FLIP_TEXTS = frozenset({'property', 'fp_text'})
# `(layer ...)` and `(at ...)` are the footprint's OWN; `pad` and
# `private_layers` have their own handlers.
_FLIP_HANDLED = (_FLIP_GRAPHICS | _FLIP_TEXTS
                 | {'layer', 'at', 'pad', 'private_layers'})

# Constructs we refuse BY NAME, so the message can say which and why, rather
# than falling through the generic "unknown node kind" arm.
_FLIP_NAMED_REFUSALS = {
    'zone': ("a footprint-internal (zone ...) stores its (pts (xy ...)) in "
             "ABSOLUTE board coordinates, so mirroring it needs a flip centre "
             "that this call does not have -- the pose may change in the same "
             "write. One witness on the tracked corpus"),
    'fp_text_box': "no tracked board carries one, so no gate here can validate a rule for it",
    'group': "no tracked board carries one, so no gate here can validate a rule for it",
}
# ...and inside a pad.
_PAD_REFUSALS = {
    'primitives': ("a custom pad's sub-geometry frame is unconfirmed against "
                   "pcbnew; 6 witnesses on the tracked corpus, none of them "
                   "enough to validate a rule"),
    'chamfer': ("corner NAMES under a mirror are unmeasured -- no tracked "
                "board carries a chamfered pad"),
    'rect_delta': ("trapezoid pad deltas under a mirror are unmeasured -- no "
                   "tracked board carries one"),
}

# Pad children this mirror knows what to do with. `at`, `layers` and `drill`
# are TRANSFORMED; the rest are mirror-invariant -- sizes, margins, net and pin
# metadata, thermal and teardrop settings, which a reflection does not touch.
# Measured: these 22 heads are every pad child kind on the 22 tracked boards
# (9491 pads), with `primitives` the only one refused. Anything outside the set
# refuses, so a KiCad that adds a pad child -- `padstack` is the live example
# in 9/10 -- stops the flip instead of being emitted unmirrored.
_PAD_HANDLED = frozenset({
    'at', 'size', 'layers', 'uuid', 'net', 'pintype', 'pinfunction',
    'roundrect_rratio', 'drill', 'remove_unused_layers', 'solder_paste_margin',
    'solder_mask_margin', 'teardrops', 'zone_connect', 'property', 'options',
    'clearance', 'thermal_bridge_angle', 'solder_paste_margin_ratio',
    'thermal_bridge_width', 'thermal_gap', 'thermal_width', 'die_length',
    'zone_layer_connections', 'tstamp', 'locked', 'chamfer_ratio',
})

_SIDED_LAYER = re.compile(r'^[FB]\.')
_INNER_LAYER = re.compile(r'^In\d+\.Cu$', re.IGNORECASE)


def _negate_coord(tok: str, ref: str, where: str) -> str:
    """Toggle a coordinate literal's sign, as TEXT.

    Not `_fmt_mm(-float(tok))`: that rewrites `0.500` as `0.5`, and the
    flip-and-flip-back byte-identity gate would then be pinning the formatter
    instead of the transform. A sign toggle over the measured literal grammar
    is an exact involution by construction.
    """
    if not _COORD_LITERAL.match(tok):
        raise SideFlipUnsupported(
            f"{ref}: the coordinate {tok!r} in {where} is outside the literal "
            f"grammar -?\\d+(\\.\\d+)? that makes a sign toggle an exact "
            f"involution (measured: 148417 tokens on the tracked corpus, 0 "
            f"outside it). Refusing rather than round-tripping it through "
            f"float and silently reformatting the board.")
    if float(tok) == 0.0:
        return tok                      # '0' and '-0.000' both stay put
    return tok[1:] if tok.startswith('-') else '-' + tok


def _iter_sexpr_children(text: str, open_idx: int = 0):
    """(head, start, end) for each direct child of the node starting at open_idx.

    Built on the shipped string-aware `find_matching_paren`, so a lone paren
    inside a quoted property value (an MPN like "TCR2EF115,LM(CT", #113) cannot
    run the walk out of one node into the next.
    """
    end_all = find_matching_paren(text, open_idx)
    i = open_idx + 1
    while i < end_all - 1:
        c = text[i]
        if c == '"':
            i += 1
            while i < end_all and text[i] != '"':
                i += 2 if text[i] == '\\' else 1
            i += 1
            continue
        if c == '(':
            end = find_matching_paren(text, i)
            m = re.match(r'\(\s*([A-Za-z_][A-Za-z_0-9]*)', text[i:end])
            yield (m.group(1) if m else '?'), i, end
            i = end
            continue
        i += 1


def _mirror_xy_pairs(node: str, ref: str, heads=('start', 'end', 'center',
                                                 'mid', 'xy')) -> str:
    """Negate the Y of every `(head x y)` in `node`. X is never touched."""
    # `[^\s()]+`, NOT `\S+`: `\S` matches `)`, so on the compact
    # `(pts (xy -1 1))` spelling the last group backtracks to `1)` and the
    # match runs to the `pts` closer. It failed CLOSED -- the literal `'1)'`
    # is outside the coordinate grammar, so `_negate_coord` refused -- but with
    # a message blaming the board for a regex bug, and it made every
    # compactly-serialised `fp_poly` / `fp_curve` unflippable.
    pat = re.compile(r'\((' + '|'.join(heads)
                     + r')(\s+)([^\s()]+)(\s+)([^\s()]+)\)')

    def fix(m):
        return (f"({m.group(1)}{m.group(2)}{m.group(3)}{m.group(4)}"
                f"{_negate_coord(m.group(5), ref, m.group(1))})")

    return pat.sub(fix, node)


def _flip_layer_nodes(node: str) -> str:
    """Toggle every `(layer "X")` token inside `node`."""
    def fix(m):
        return f'(layer "{flip_layer_token(m.group(1))}")'
    return re.sub(r'\(layer\s+"([^"]+)"\)', fix, node)


def _flip_at_angle(node: str, ref: str, new_angle_of) -> str:
    """Mirror the FIRST `(at x y [a])` in `node`: y negated, angle remapped.

    The angle token's PRESENCE is preserved, which is what makes the round trip
    exact. Measured: all 1110 `fp_text` and all 6565 `property` nodes on the
    tracked corpus carry an explicit three-token `(at x y a)`, so a text never
    gains or loses the token; 2884 pads carry the two-token form, and for those
    the angle is 0 and stays 0 whenever the footprint's own rotation is 0 or
    180. A pad that would gain a non-zero angle gains the token.
    """
    m = re.search(r'\(at\s+([^\s()]+)\s+([^\s()]+)(?:\s+([^\s()]+))?\)', node)
    if not m:
        if not re.search(r'\(at\b', node):
            return node          # genuinely has no `(at ...)`; nothing to move
        # There IS one and it did not parse: REFUSE, never return the node
        # untouched. The rest of the footprint mirrors around it, so a silently
        # skipped `(at ...)` leaves exactly the half-mirrored block this whole
        # feature exists to prevent. The reachable spelling is KiCad 6/7's
        # four-token `(at 0 -3.98 0 unlocked)` -- no tracked board writes it
        # (they all use the modern `(unlocked yes)` node), which is why nothing
        # caught this.
        raise SideFlipUnsupported(
            f"{ref}: a node has an `(at ...)` this mirror cannot parse -- "
            f"expected two or three plain numbers. Refusing rather than "
            f"leaving it unmirrored inside a footprint whose every other node "
            f"moved: {node[:120]!r}")
    x, y, a = m.group(1), m.group(2), m.group(3)
    ny = _negate_coord(y, ref, 'at')
    na = new_angle_of(float(a) if a is not None else 0.0)
    na = round(na % 360, 6)
    if a is not None or na != 0:
        rep = f"(at {x} {ny} {na:.6g})"
    else:
        rep = f"(at {x} {ny})"
    return node[:m.start()] + rep + node[m.end():]


def _flip_justify(node: str, ref: str) -> str:
    """Toggle the `mirror` token of the first `(justify ...)` in `node`.

    Three cases, and two of them add or delete a whole node. Measured INSIDE
    footprint blocks on the 22 tracked boards -- which is the only place this
    function ever runs -- there is exactly ONE form, `(justify mirror)`, 1356
    of them, and ZERO instances of any alignment token. (Whole-file counts are
    1367 and 42, but the 42 `(justify left bottom)` are board-level `gr_text`
    that this never sees.) So the alignment-preserving branch below has no
    corpus witness at all; it is there because pcbnew emits alignment tokens on
    a footprint text the moment a user sets one, and dropping them would be a
    silent edit to somebody's silkscreen. Said plainly because an earlier
    version of this docstring quoted 1391/58 -- numbers that match no board set
    -- and justified the branch with a form that cannot occur here.

      absent            -> insert `(justify mirror)` as the last child of
                           `(effects ...)`, at the `(font ...)` indentation,
                           which is where KiCad writes it
      `left bottom`     -> `left bottom mirror`
      `mirror` alone    -> DELETE the node and its leading newline+indent
                           (dropping only the token would leave `(justify)`)

    The delete case is the one a token-replacement implementation gets wrong,
    and it is the whole of the B->F direction.
    """
    m = re.search(r'(\s*)\(justify\s+([^()]*?)\)', node)
    if m:
        toks = m.group(2).split()
        if 'mirror' in toks:
            toks = [t for t in toks if t != 'mirror']
            if not toks:
                # the node's ENTIRE text, leading whitespace included, so
                # insert and delete are exact inverses
                return node[:m.start()] + node[m.end():]
            rep = m.group(1) + '(justify ' + ' '.join(toks) + ')'
        else:
            rep = m.group(1) + '(justify ' + ' '.join(toks + ['mirror']) + ')'
        return node[:m.start()] + rep + node[m.end():]

    # INSERT. The separator is taken from however `(font ...)` is actually
    # written, so the compact one-line spelling works too. An earlier version
    # required `(font` to START a line, which meant that on a compactly
    # serialised board (KiCad 6/7 output, and anything a third-party tool
    # writes) the DELETE path fired and the INSERT path silently did not:
    # B->F removed the flag, F->B never added it, and the two stopped being
    # inverses. No tracked board writes a one-line `(effects ...)`, so no gate
    # could see it.
    fm = re.search(r'(\s*)\(font\b', node)
    if fm is None:
        # No font block to hang it on. REFUSE rather than return the node
        # untouched: silently declining to mirror one text inside a footprint
        # whose every other node moved is the half-flip this feature exists to
        # prevent, and `write_placed_output` returns True either way.
        raise SideFlipUnsupported(
            f"{ref}: a text node has no `(effects (font ...))` to carry "
            f"`(justify mirror)`, so its mirrored state cannot be written: "
            f"{node[:120]!r}")
    sep = fm.group(1) if '\n' in fm.group(1) else ' '
    fend = find_matching_paren(node, fm.end() - len('(font'))
    return node[:fend] + sep + '(justify mirror)' + node[fend:]


def _flip_pad(node: str, ref: str, old_rot: float, new_rot: float) -> str:
    """Mirror one `(pad ...)` node.

    The pad ANGLE is the subtle one and is derived, not copied. KiCad stores a
    pad's angle ABSOLUTE (`a = R + p`, p the pad-local angle); a flip sends
    `p -> -p`, so

        a' = R_new - (a - R_old)

    `_rotate_pad_angles` computes `a + (R_new - R_old) = R_new + p`, which
    agrees only when p is 0 or 180. This path therefore owns the pad `(at ...)`
    entirely and never calls it -- one code path per mode, so the ordinary
    non-flip write cannot regress.
    """
    # A WHITELIST, like the footprint-level dispatch, and for the same reason.
    # This was a blacklist -- refuse the three named heads, pass everything
    # else through -- which meant the module's own argument stopped applying
    # one level down: the very head that refuses at footprint level was emitted
    # verbatim from inside a pad. The realistic instance is KiCad 9/10's
    # `(padstack ...)` (per-layer pad geometry with its own nested
    # `(primitives ...)` and per-layer `(layer ...)` sub-nodes): not in the
    # refusal table, so its primitives never reached the check and its layers
    # were never toggled. No tracked board carries one, which by this file's
    # own doctrine is the argument for refusing rather than for passing
    # through.
    for head, s, e in _iter_sexpr_children(node, 0):
        if head in _PAD_REFUSALS:
            raise SideFlipUnsupported(
                f"{ref}: a pad carries ({head} ...) -- {_PAD_REFUSALS[head]}. "
                f"Refusing to guess at its mirror image.")
        if head not in _PAD_HANDLED:
            raise SideFlipUnsupported(
                f"{ref}: a pad carries a ({head} ...) node, which this mirror "
                f"has no rule for. The pad dispatch is a WHITELIST for the "
                f"same reason the footprint one is: a node passed through "
                f"untouched leaves a pad whose geometry contradicts the face "
                f"it claims, and nothing downstream raises.")

    lm = re.search(r'\(layers\b', node)
    if lm:
        lend = find_matching_paren(node, lm.start())
        block = node[lm.start():lend]
        toks = re.findall(r'"([^"]+)"', block)
        for t in toks:
            if '&' in t:
                raise SideFlipUnsupported(
                    f"{ref}: a pad names the layer set {t!r}. It is plausibly "
                    f"its own mirror, and no tracked board carries one, so "
                    f"nothing here can check that. Refusing rather than "
                    f"guessing.")
            if _INNER_LAYER.match(t):
                raise SideFlipUnsupported(
                    f"{ref}: a pad names the inner copper layer {t!r}. Its "
                    f"mirror image depends on the board's inner-layer count "
                    f"and on remove_unused_layers padstack semantics, and no "
                    f"tracked board carries one in a footprint pad. Refusing.")
        new_block = re.sub(r'"([^"]+)"',
                           lambda m: '"' + flip_layer_token(m.group(1)) + '"',
                           block)
        node = node[:lm.start()] + new_block + node[lend:]

    node = _flip_at_angle(node, ref, lambda a: new_rot - (a - old_rot))

    # `(drill ... (offset x y))` -- the offset is the HOLE's displacement from
    # the copper centre and mirrors with everything else. The only witness on
    # the tracked corpus is rp2350 U8.
    dm = re.search(r'\(drill\b', node)
    if dm:
        dend = find_matching_paren(node, dm.start())
        node = (node[:dm.start()]
                + _mirror_xy_pairs(node[dm.start():dend], ref, heads=('offset',))
                + node[dend:])
    return node


def _flip_graphic(node: str, head: str, ref: str) -> str:
    """Mirror one `fp_*` graphic node."""
    node = _mirror_xy_pairs(node, ref)
    node = _flip_layer_nodes(node)
    if head == 'fp_arc':
        # An arc's SWEEP is start->mid->end. Mirroring the three points
        # reverses the sweep, so KiCad exchanges start and end to restore it.
        # Measured on sonde_u U2 and rp2350 SW1. This is invisible to every
        # geometric check -- the same curve passes through the same three
        # points either way -- which is why the acceptance gate compares nodes.
        sm = re.search(r'\(start\s+(\S+)\s+(\S+)\)', node)
        em = re.search(r'\(end\s+(\S+)\s+(\S+)\)', node)
        if sm and em:
            s_body, e_body = f"{sm.group(1)} {sm.group(2)}", f"{em.group(1)} {em.group(2)}"
            lo, hi = sorted((sm, em), key=lambda m: m.start())
            lo_new = f"(start {e_body})" if lo is sm else f"(end {s_body})"
            hi_new = f"(end {s_body})" if hi is em else f"(start {e_body})"
            node = (node[:lo.start()] + lo_new + node[lo.end():hi.start()]
                    + hi_new + node[hi.end():])
    return node


def _block_side(fp_text: str) -> str:
    """'F' or 'B' from the block's own `(layer ...)`.

    The same first-character rule `legality.footprint_side` applies to the
    parsed object, read here off the text so the writer and the side model
    cannot disagree about what a block says it is.
    """
    m = re.search(r'\(layer\s+"([^"]+)"\)', fp_text)
    return 'B' if (m and m.group(1).startswith('B')) else 'F'


def _flip_footprint_block(fp_text: str, ref: str, old_rot: float,
                          new_rot: float) -> str:
    """Mirror a whole footprint block to the other face (#714).

    A paren-balanced walk over the block's top-level children with a per-kind
    dispatch, NOT a set of global regexes: a blanket `y -> -y` over the block
    would also hit `(size 1 1)`, `(thickness 0.15)` and `(stroke (width 0.12))`,
    and a blanket layer toggle would hit nodes that must not move.

    The block's own `(at ...)` is deliberately skipped -- `write_placed_output`
    has already written it from `new_x/new_y/new_rotation`, and the caller owns
    the final rotation. Note that means this differs from `pcbnew`'s `Flip`,
    which negates the orientation itself; a caller that wants pcbnew's result
    passes the negated angle, which is what the parity gate does.
    """
    # The block MUST declare a side of its own, and this is checked rather
    # than assumed: `_block_side` defaults to 'F' when there is no top-level
    # `(layer ...)`, so a block without one would have all its geometry
    # mirrored and no layer written -- #714's exact silent failure,
    # reintroduced in the degenerate case. KiCad always writes the node, so
    # this is unreachable from KiCad; it is checked because a whitelist that
    # refuses far less likely things should not wave this one through.
    if not re.search(r'\(layer\s+"[^"]+"\)', fp_text):
        raise SideFlipUnsupported(
            f"{ref}: the footprint block declares no top-level `(layer ...)`, "
            f"so there is nothing to flip and no way to say which face the "
            f"mirrored geometry ended up on.")

    pieces = []
    prev = 0
    saw_layer = False
    for head, s, e in _iter_sexpr_children(fp_text, 0):
        node = fp_text[s:e]
        if head in _FLIP_NAMED_REFUSALS:
            raise SideFlipUnsupported(
                f"{ref}: the footprint carries a ({head} ...) node -- "
                f"{_FLIP_NAMED_REFUSALS[head]}. Refusing to guess at its "
                f"mirror image.")
        if head in _FLIP_PASSTHROUGH:
            continue
        if head not in _FLIP_HANDLED:
            raise SideFlipUnsupported(
                f"{ref}: the footprint carries a ({head} ...) node, which this "
                f"mirror has no rule for. The dispatch is a WHITELIST on "
                f"purpose: a node passed through untouched leaves a footprint "
                f"whose geometry contradicts the face it claims, and nothing "
                f"downstream raises. Add a measured rule for ({head} ...) or "
                f"leave the flip refused.")

        if head == 'at':
            continue                     # already written by the caller
        elif head == 'layer':
            saw_layer = True
            new = _flip_layer_nodes(node)
        elif head == 'private_layers':
            new = re.sub(r'"([^"]+)"',
                         lambda m: '"' + flip_layer_token(m.group(1)) + '"',
                         node)
        elif head == 'pad':
            new = _flip_pad(node, ref, old_rot, new_rot)
        elif head in _FLIP_TEXTS:
            # The mirror FLAG is conditional on the text living on a sided
            # layer, and only the flag is. Measured against pcbnew 10.0.0: a
            # text on `Cmts.User` / `Eco1.User` / `Dwgs.User` has its y and its
            # angle mirrored like any other, but pcbnew does NOT add
            # `(justify mirror)` to it -- there is no face for it to be seen
            # from the wrong side of. Toggling it anyway is a real divergence
            # and it is invisible on an ordinary part: 28 flip-eligible texts
            # on the tracked corpus sit on a User layer, and not one of the
            # thirteen parity fixtures carried one until orangecrab U3 and J1
            # were added for exactly this.
            sided = bool(re.search(r'\(layer\s+"[FB]\.', node))
            # A TEXT's angle composes with the pose change exactly as a pad's
            # does; `180 - a` alone is the special case `new_rot == -old_rot`,
            # i.e. pcbnew's own bare Flip, and it is wrong by the rotation
            # delta for every other request. That is not hypothetical: the
            # `layer_flip` perturbation HOLDS the pose, so its delta is 2R, and
            # 9 of tigard's 33 flipped parts shipped a reference designator
            # rotated 180 degrees from where KiCad puts it -- relative to their
            # own pads, which had rotated correctly. Silent, and invisible to
            # the parity gate because that gate only ever asked for the pure
            # flip. See the COMPOSED pass in
            # `tests/gui_parity/test_714_mirror_pcbnew_parity.py`.
            new = _flip_at_angle(node, ref,
                                 lambda a: new_rot + 180.0 - (a - old_rot))
            new = _flip_layer_nodes(new)
            if sided:
                new = _flip_justify(new, ref)
        else:
            new = _flip_graphic(node, head, ref)

        if new != node:
            pieces.append((s, e, new))

    if not saw_layer:
        # Belt and braces: the regex above found a `(layer ...)` somewhere in
        # the block, the WALK must have found it as a top-level child. If the
        # two disagree, the walk is wrong and the block would ship half
        # mirrored.
        raise SideFlipUnsupported(
            f"{ref}: the block's `(layer ...)` is not a top-level child, so "
            f"the mirrored geometry would carry no face declaration.")

    if not pieces:
        return fp_text
    out = []
    for s, e, new in pieces:
        out.append(fp_text[prev:s])
        out.append(new)
        prev = e
    out.append(fp_text[prev:])
    return ''.join(out)


def write_placed_output(input_file: str, output_file: str,
                        placements: List[Dict],
                        via_moves: List = None,
                        new_segments: List = None,
                        pcb_data=None) -> bool:
    """
    Write a placed PCB file by modifying footprint positions.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        placements: List of dicts with keys:
            - reference: str (e.g. "U1")
            - new_x: float (mm)
            - new_y: float (mm)
            - new_rotation: float (degrees)
            - new_side: 'F' or 'B', OPTIONAL (#714). Absent or None leaves the
              footprint's `(layer ...)` alone and the write is byte-identical
              to one built before this existed -- which is what keeps the ~20
              producers of these dicts unchanged. Present and DIFFERENT from
              the block's current side, the whole footprint is mirrored to the
              other face: pad layers, local y, angles, graphics, justify. Any
              value other than 'F'/'B' raises at the boundary; `'B.Cu'` is the
              mistake a caller actually makes. A construct the mirror cannot
              express raises `SideFlipUnsupported` rather than being skipped,
              because this function returns True unconditionally and a
              half-mirrored footprint is indistinguishable from success at
              every call site.
              `new_rotation` still means the FINAL orientation; pcbnew's
              `Flip` negates it and this does not, so the same call cannot
              mean different things depending on the input side.

    Returns:
        True if output was written successfully
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)

    placement_by_ref = {p['reference']: p for p in placements}
    modified_count = 0
    matched = set()
    unapplied_blocks = []

    # Blocks are NAMED by the parser's own resolver (#726). Before that, this
    # loop read each block's `(property "Reference" ...)` itself and looked the
    # STRING up, so one placement rewrote EVERY block carrying it: measured, a
    # single placement for esp_prog's `Ref*` moved both fiducials and left them
    # stacked 23.44 mm from where the second one belonged, and `perturb.
    # _all_at_current` -- the identity write that builds the stress harness's
    # ground-truth CONTROL board -- relocated 3 blocks on 2 corpus boards, up
    # to 23.44 mm. (12 blocks across 5 boards, to 70.66 mm, is the ceiling for
    # a caller that places every PARSED footprint; `state.parts` excludes the
    # zero-pad blocks that make up the difference.)
    #
    # Resolving here means the writer and the parser can no longer disagree
    # about what a name means -- which is the property that matters, not merely
    # that duplicates are handled: producers build their placement dicts from
    # `pcb.footprints` keys, so those keys have to address these blocks.
    #
    # Process in reverse order so string indices remain valid after
    # replacements; `iter_footprint_blocks` computes every span up front on the
    # original content. The scan is string-aware, so a lone paren inside a
    # property value (an MPN like "TCR2EF115,LM(CT") cannot run past a block and
    # swallow the next footprint -- issue #113. A naive depth counter here would
    # place the WRONG footprint, or silently place none.
    for start, end, fp_text, _raw_ref, key in reversed(
            list(iter_footprint_blocks(content))):
        placement = placement_by_ref.get(key)
        if placement is None:
            continue

        # Find the footprint's (at X Y [rotation]) - it's the first (at ...) in the block
        at_match = re.search(r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)',
                             fp_text)
        if not at_match:
            # Resolved, but there is nothing to rewrite. `matched` is added
            # BELOW this guard on purpose: it means "the writer acted", so a
            # block with no `(at ...)` is reported rather than falling into
            # the silence this reporter exists to end.
            unapplied_blocks.append(key)
            continue
        matched.add(key)

        # Build replacement (at ...) string
        new_x = placement['new_x']
        new_y = placement['new_y']
        new_rot = placement['new_rotation']
        old_rot = float(at_match.group(3)) if at_match.group(3) else 0.0

        # #714. An OPTIONAL key: absent or None means the layer is left alone
        # and this write is byte-identical to one built before the flip
        # existed, which is what keeps the ~20 producers of these dicts
        # (quench, seeder, portfolio, relocate, perturb, fanout_clearance,
        # place_reconstruct, place_seed, converge, net_rescue) unchanged.
        #
        # Spelled 'F'/'B', matching `legality.footprint_side` exactly, and
        # anything else RAISES at the boundary rather than 200 lines
        # downstream -- 'B.Cu' is the mistake a caller actually makes.
        new_side = placement.get('new_side')
        if new_side is not None:
            if new_side not in ('F', 'B'):
                raise ValueError(
                    f"{key}: new_side must be 'F' or 'B' (the spelling "
                    f"`legality.footprint_side` uses), not {new_side!r}. A "
                    f"layer NAME is not a side.")
            side_change = new_side != _block_side(fp_text)
        else:
            side_change = False

        # #829 backstop. RAISES, and does not skip: this function returns True
        # unconditionally, so a skip would be indistinguishable from success --
        # and `route.py`'s #666 cap move gates its IN-MEMORY mirror on that
        # return, advancing pcb_data's footprint and pad coordinates while the
        # file kept the old pose, after which oracle_reconnect welds copper to
        # pad coordinates that exist nowhere on the board.
        #
        # ON AN ACTUAL POSE CHANGE ONLY, and that qualifier is not a nicety.
        # `perturb._all_at_current` deliberately hands this writer EVERY part,
        # at its current pose, so the moved set cannot be read off the
        # six-decimal `(at)` formatting -- and its dose-0 CONTROL board is
        # exactly that call with no moves at all. Gating on presence in
        # `placements` therefore refused to write the control board of every
        # perturbation run on a board with an outline owner, with a message
        # blaming a mover that had not moved anything. Measured before this
        # qualifier existed.
        if (_OWNS_OUTLINE_RE.search(fp_text)
                and (abs(new_x - float(at_match.group(1))) > _POSE_EPS
                     or abs(new_y - float(at_match.group(2))) > _POSE_EPS
                     or abs((new_rot - old_rot + 180) % 360 - 180) > _POSE_EPS
                     # #714: a flip with identical x/y/rot is still a change,
                     # and on an outline owner it MIRRORS the owned Edge.Cuts
                     # geometry -- the board resize this guard exists to
                     # refuse. Without this term the flip walks straight past
                     # it. `perturb._all_at_current` never sets `new_side`, so
                     # the dose-0 control board is unaffected, which is the
                     # reason the rest of this predicate is written the way it
                     # is.
                     or side_change)
                # `key`, not the raw reference: #726 keys a duplicated
                # reference's second block `TP4~2`, and the owner map is keyed
                # the same way (kicad_parser._footprint_blocks_by_key), so a
                # raw-name lookup would ask about the wrong block.
                and _draws_board_outline(input_file, key)):
            raise OutlineOwnerMove(
                f"{key} draws the board outline (Edge.Cuts geometry outside "
                f"the board-level outline), so moving it would resize the "
                f"board. Refusing to write {output_file}. This is a bug in "
                f"whatever produced the placement list -- the movable-set "
                f"gates should never have offered {key} a NEW pose (#829).")

        # :.6f, NOT :.4g. `%g` counts SIGNIFICANT digits, so a coordinate past
        # 100mm -- most of a real board -- was quantised to 0.1mm: a cap moved to
        # x=139.96 was written as "140", a 40um placement error, and past 1000mm
        # the step becomes 1mm. That silently coarsens the very clearance repair
        # this writer exists to apply (#130 cap moves, #313 via nudge), and it
        # can push a part back into the graze it was moved out of. :.6f is the
        # nanometre-faithful format kicad_writer already uses for track geometry,
        # and matches KiCad's own footprint `at` precision (6 decimals). Rotation
        # keeps a significant-digit format (angles are small and exact-ish) but
        # widened so e.g. 137.25deg survives.
        if new_rot != 0:
            new_at = f"(at {new_x:.6f} {new_y:.6f} {new_rot:.6g})"
        else:
            new_at = f"(at {new_x:.6f} {new_y:.6f})"

        new_fp_text = fp_text[:at_match.start()] + new_at + fp_text[at_match.end():]

        # KiCad stores pad angles as footprint rotation + pad-local rotation,
        # so a footprint rotation change must be added to every pad angle.
        if side_change:
            # The flip owns every pad angle itself: a mirror sends the
            # pad-local angle p -> -p, so a' = new_rot - (a - old_rot), which
            # `_rotate_pad_angles`' a + delta_rot equals only when p is 0 or
            # 180. One code path per mode, so the ordinary write below cannot
            # regress.
            new_fp_text = _flip_footprint_block(new_fp_text, key,
                                                old_rot, new_rot)
        else:
            delta_rot = (new_rot - old_rot) % 360
            if delta_rot != 0:
                new_fp_text = _rotate_pad_angles(new_fp_text, delta_rot)

        content = content[:start] + new_fp_text + content[end:]
        modified_count += 1

    # Via-nudge rewrites (#313): remove each moved via from the input text and
    # append it at its new position, plus the new connector segments back to
    # the fanout stub start. Attached segments are untouched by design.
    if via_moves or new_segments:
        from plane_io import _remove_vias_at_positions
        from kicad_writer import generate_via_sexpr, generate_segment_sexpr
        n2n = getattr(pcb_data, 'net_id_to_name', None) if pcb_data is not None else None

        def _net_name(net_id):
            """This board's name for a net id, or None if it has none.

            net 0 is spelled out because it is a hole in the map rather than an
            unknown net: `extract_nets` records `name_to_id[""] = 0` but never
            creates a Net for id 0, so a legitimate no-net `(net "")` via --
            which the parser models and this writer emits -- misses the lookup
            on EVERY board. Measured: orangecrab_ext_pll resolves 169 nets and
            has no key 0.

            Getting this wrong is not cosmetic. Falling back to the numeric
            dialect while also emitting a protection spec produces a via
            `kicad_parser.extract_vias` CANNOT read back -- its KiCad-9 pattern
            has strict field ordering and no gap for the protection tokens --
            so the barrel disappears from the model entirely. Invisible copper
            the router then plans tracks through, which is the hazard PR #534
            was written against and strictly worse than the wrong-tenting bug
            #741 set out to fix. The parser asymmetry itself is #748.
            """
            if not n2n:
                return None
            nm = n2n.get(net_id)
            return '' if nm is None and net_id == 0 else nm

        elements = []
        if via_moves:
            content, _ = _remove_vias_at_positions(
                content, [(m[0], m[1]) for m in via_moves],
                net_ids=[m[2]['net_id'] for m in via_moves],
                # through the same resolver as the emit: passing None here for
                # net 0 dropped the removal to POSITIONAL matching for exactly
                # the via this fix is about.
                net_names=[_net_name(m[2]['net_id']) for m in via_moves])
            for m in via_moves:
                v = m[2]
                nm = _net_name(v['net_id'])
                # Every via in via_moves came OFF this board a few lines up,
                # so its OWN spec is the whole answer. That much is
                # plane_io.py:862-866's form -- the rip-up restore, which is
                # structurally the same act -- and deliberately NOT
                # plane_io.py:557-563's `or
                # prevailing_via_protection_in_text(...)`, the right default
                # for vias this tool ADDS but a THIRD answer for one that
                # already existed: it would stamp the majority spec of OTHER
                # vias onto this one.
                #
                # `inherit_when_unspecified=True` is where this call goes
                # BEYOND plane_io's form, and it is the second half of #741. An
                # absent or empty spec is not silence from the CALLER, it is the
                # board saying nothing about this via -- so emit nothing and let
                # it keep inheriting `(setup (tenting ...))`. Without the flag,
                # None/{} reaches kicad_writer's front+back default and the via
                # GAINS a fab attribute it never had. The GUI twin has never
                # done that: gui_utils.apply_via_protection returns early on an
                # empty spec, because pcbnew's *_MODE_FROM_BOARD already means
                # inherit. plane_io's restore carries the spec but not this
                # flag, so it still has the residue; #749 rather than changed
                # from here.
                #
                # Stating the rule HERE rather than relying on the engine key
                # means the two halves of #741 fail independently: a move dict
                # that lost the key inherits rather than re-tents.
                #
                # `.get` for this one key, and the asymmetry with the
                # hard-indexed geometry below is deliberate rather than sloppy:
                # a dict with no 'x'/'net_id' is not a via and SHOULD raise,
                # while a dict with no spec is a via nobody recorded one for --
                # a meaning this line has an answer for.
                spec = v.get('tenting_attrs')
                if nm is None and spec:
                    # Belt for the residue of the same hazard: a numeric-net
                    # emission that still carries a spec is the unreadable
                    # pairing described in _net_name. Losing the spec is the
                    # #741 bug; losing the VIA is worse, so this trades back
                    # deliberately and only where the two conflict.
                    #
                    # Reachable, though not from a real board: no KiCad-9 board
                    # carries per-via protection (that is a KiCad-10 feature and
                    # those boards are name-net), but `write_placed_output`
                    # takes an arbitrary pcb_data, and a synth.make_pcb() one
                    # has an EMPTY net_id_to_name -- which sends every via
                    # numeric.
                    spec = None
                elements.append(generate_via_sexpr(
                    v['x'], v['y'], v['size'], v['drill'], v['layers'],
                    v['net_id'], net_name=nm, tenting_attrs=spec,
                    # The DECISION, carried separately from the value: `{}` is
                    # what Via.tenting_attrs holds for a via that inherits, and
                    # handing it back without this would re-stamp the via with
                    # front+back tenting it never had (#741). A sentinel VALUE
                    # cannot do this job -- the repo's own idiom for carrying a
                    # spec is `dict(...)`, which would turn any dict-shaped
                    # sentinel back into a plain {}.
                    inherit_when_unspecified=True))
        for nsd in (new_segments or []):
            nm = n2n.get(nsd['net_id']) if n2n else None
            elements.append(generate_segment_sexpr(
                nsd['start'], nsd['end'], nsd['width'], nsd['layer'],
                nsd['net_id'], net_name=nm))
        if elements:
            close = content.rindex(')')
            content = content[:close] + '\n'.join(elements) + '\n)' + content[close+1:]

    # THE SINGLE POSE FUNNEL, and it must run BEFORE the write. Every
    # footprint `(at ...)` rewrite in py_placer comes through here, so this is
    # the one place that can answer "was every pose in this board produced by
    # a registered engine lever?". Outside an unaided regime it is a no-op and
    # the behaviour is byte-identical; inside one, an undeclared write raises.
    #
    # It used to raise AFTER `f.write(content)`, which made the refusal
    # decorative: the poses were already on disk, so the "gate" reported a
    # violation about a file it had just helped produce. Refusing means not
    # writing.
    #
    # NOTE the qualifier -- "in py_placer". The GUI writes poses through
    # pcbnew (`ai_plan._run_place_plan`, `fanout_gui`, `placement_gui`) and
    # does NOT come through here, so a GUI-driven placement leaves no row.
    # See provenance.record_write for what that means for a verdict.
    # #829 tripwire: would this write change the board OUTLINE? The per-ref
    # gate above answers "may this footprint move"; this answers the question
    # the placement stack's contract actually makes -- "is the outline the same
    # board it was" -- and it catches a route no gate anticipated.
    #
    # BEFORE THE WRITE, comparing the ORIGINAL text against the in-memory
    # `content`. Both halves of that matter and the first draft got both wrong:
    #
    #  * Raising after `f.write` left the resized board sitting at
    #    `output_file` with `commit_write` skipped -- a board with a changed
    #    outline on disk, no ledger row for it, and an orphaned _PENDING entry.
    #    `provenance.py` documents that exact shape as the defect its
    #    pending/commit split was introduced to fix; doing it here reinstated
    #    it.
    #  * Re-reading `output_file` made the check STRUCTURALLY INERT when
    #    `input_file is output_file` -- which is `route.py:4074`, the #666
    #    scoped cap move, the very call site this backstop cites as its reason
    #    to exist. Both paths read the same bytes, so it could never fire.
    #
    # It compares a FINGERPRINT, not `board_bounds`. Bounds are blind to an
    # interior cutout moving and to a rotated circular window (fp_circle bounds
    # are rotation-invariant about the moved centre), and rings are blind to an
    # open stub that never chains -- the #829 repro moved bounds while
    # board_outlines stayed identical, so either alone would have missed it.
    #
    # Armed off the BOARD, not off which refs the per-ref gate happened to
    # check: using the gate's cache made the tripwire depend on a side effect,
    # so it disarmed whenever the moved footprints were not the ones carrying
    # Edge.Cuts. Only a board with footprint-embedded Edge.Cuts arms it, which
    # is 0 of the 27 tracked boards.
    if _outline_owner_map(input_file):
        from kicad_parser import structural_outline_fingerprint
        with open(input_file, encoding='utf-8') as _fh:
            _before = structural_outline_fingerprint(_fh.read())
        if _before != structural_outline_fingerprint(content):
            raise OutlineOwnerMove(
                f"this write would have CHANGED THE BOARD OUTLINE of "
                f"{output_file}. The placement stack does not resize boards: "
                f"size, cutouts and slots are mechanical decisions the user "
                f"owns. Nothing was written. This is a bug in whatever "
                f"produced the placement list (#829).")

    from placement import provenance
    provenance.record_write(input_file, output_file, placements,
                            pending=True)

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)

    provenance.commit_write(output_file)

    print(f"Modified {modified_count} footprint positions")
    _report_unapplied(placement_by_ref, matched, unapplied_blocks,
                      'placement')
    print(f"Successfully wrote {output_file}")
    return True
