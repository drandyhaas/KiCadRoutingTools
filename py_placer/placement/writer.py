"""
Write placed PCB output by modifying footprint positions in the .kicad_pcb file.

Uses text-based manipulation (same approach as output_writer.py / kicad_writer.py)
to update the (at X Y [rotation]) of each footprint block.
"""
from __future__ import annotations

import os
import math
import re
import sys
from typing import List, Dict, Optional

from kicad_parser import (find_matching_paren, flip_layer_token,
                          iter_footprint_blocks, footprint_at_match,
                          AT_NUM, layer_list_tokens, map_layer_list_tokens)
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
    # Keyed by (path, mtime, size), NOT by path alone. A tool that writes a
    # temp board and `os.replace`s it over the output, and `route.py` writes in
    # place, so a path-keyed memo answers for a file that no longer exists --
    # measured: after the file changed on disk the writer still reported the
    # old owners. In a long-lived plugin process a path-keyed dict also grows
    # without bound.
    if key not in _OUTLINE_OWNER_CACHE:
        try:
            from kicad_parser import footprint_outline_owners, read_board_text
            _OUTLINE_OWNER_CACHE[key] = footprint_outline_owners(
                read_board_text(input_file, quiet=True))
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
    at_m = re.search(r'\(at\s+' + AT_NUM + r'\s+' + AT_NUM + r'(?:\s+' + AT_NUM + r')?\)', node)
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
    WARNING line in front is a JSONDecodeError at char 0.
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


#: A pad's own `(at x y [angle])`: the pad number quoted or bare, then any run
#: of bare tokens before it -- type and shape, and the `locked` KiCad 6 writes
#: on every pad, which a fixed two-token pattern missed, leaving every pad of a
#: rotated KiCad 6 footprint turned against its body (#1305, steppenprobe).
_PAD_AT = re.compile(
    r'(\(pad\s+(?:"[^"]*"|[^\s()"]+)(?:\s+[^\s()"]+)*\s*)'
    r'\(at\s+(' + AT_NUM + r')\s+(' + AT_NUM + r')(?:\s+(' + AT_NUM + r'))?\)')


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

    return _PAD_AT.sub(fix_pad, fp_text)


#: A text node's own `(at x y [angle] [unlocked])`. KiCad 6/7 write a text
#: that does not keep upright as a fourth token, `(at 0 -1.85 90 unlocked)`
#: (KiCad 8+ writes an `(unlocked yes)` node instead); the angle is a number,
#: so the word is never read as one, and a rewrite puts it back.
_TEXT_AT = re.compile(r'\(at\s+([^\s()]+)\s+([^\s()]+)(?:\s+(' + AT_NUM
                      + r'))?(\s+unlocked)?\)')


def _rotate_text_angles(fp_text: str, delta_rot: float, ref: str) -> str:
    """Add delta_rot to the FIRST `(at x y [a])` of every top-level text node
    (`property`, `fp_text`) in a footprint block.

    KiCad stores a footprint text's angle as an ABSOLUTE board angle (probed
    on pcbnew 10.0.0: `SetOrientationDegrees(old + d)` moves every text angle
    by exactly d), so a rotation composes additively -- the same rule
    `_rotate_pad_angles` applies to pads -- under the token rule
    `_flip_at_angle` measured on the corpus: a present angle token is kept
    (a text at 315 rotated by 45 keeps `(at x y 0)`), an absent one is added
    only when the result is non-zero.

    Until this existed the rotation path rotated pads only, so every rotated
    part shipped its texts at the pre-rotation angle: 11 of run 26's 15 rotated
    parts carried a Reference designator that no longer matched the pads it
    labelled. The flip path has always composed its texts (#714) and is
    untouched here -- one code path per mode, as `write_placed_output` says.
    """
    def rotated(x, y, a):
        na = round(((float(a) if a is not None else 0.0) + delta_rot) % 360, 6)
        if a is not None or na != 0:
            return f"(at {x} {y} {na:.6g})"
        return f"(at {x} {y})"
    return _rewrite_text_ats(fp_text, ref, rotated, 'this rotation')


def _set_text_angles(fp_text: str, angle: float, ref: str) -> str:
    """Set EVERY top-level text node's absolute angle to `angle` (#1163).

    A pile must carry nothing about the poses it was made from. Composing a
    text with the rotation change (`_rotate_text_angles`) is right for a real
    rotation, but a pile sets every part to one rotation, so each text keeps
    its text-to-part angle `T - R`; humans keep labels upright, and that
    hands back the human rotation `R`. Writing one angle for every text
    removes it, and the token is always written (KiCad 10's own spelling for
    a text), so its presence carries nothing either.
    """
    def fixed(x, y, _a):
        return f"(at {x} {y} {round(angle % 360, 6):.6g})"
    return _rewrite_text_ats(fp_text, ref, fixed, 'this text-angle reset')


def text_node_angles(fp_text: str) -> List[Optional[float]]:
    """The angle token of each top-level text node's `(at ...)`, in order.

    None where the token is absent (KiCad reads that as 0). For a check that
    has to prove what a writer did to a block's texts (make_unplaced, #1163).
    """
    out = []
    for head, s, e in _iter_sexpr_children(fp_text, 0):
        if head in _FLIP_TEXTS:
            m = _TEXT_AT.search(fp_text[s:e])
            if m:
                out.append(float(m.group(3)) if m.group(3) is not None else None)
    return out


def _rewrite_text_ats(fp_text: str, ref: str, new_at, what: str) -> str:
    """Replace the FIRST `(at x y [a])` of every top-level text node
    (`property`, `fp_text`) with `new_at(x, y, a)`; `a` is None when absent."""
    pieces = []
    for head, s, e in _iter_sexpr_children(fp_text, 0):
        if head not in _FLIP_TEXTS:
            continue
        node = fp_text[s:e]
        m = _TEXT_AT.search(node)
        if not m:
            if re.search(r'\(at\b', node):
                # Same doctrine as `_flip_at_angle`: an `(at ...)` that is
                # there and does not parse must not be skipped silently, or
                # the block ships with its pads turned and its texts not.
                raise SideFlipUnsupported(
                    f"{ref}: a text node has an `(at ...)` {what} "
                    f"cannot parse -- expected two or three plain numbers "
                    f"and KiCad 6/7's optional `unlocked`. "
                    f"Refusing rather than rotating the pads and leaving the "
                    f"text where it was: {node[:120]!r}")
            continue
        rep = new_at(m.group(1), m.group(2), m.group(3))
        if m.group(4):
            rep = rep[:-1] + ' unlocked)'
        if rep != node[m.start():m.end()]:
            pieces.append((s + m.start(), s + m.end(), rep))
    if not pieces:
        return fp_text
    out = []
    prev = 0
    for s, e, rep in pieces:
        out.append(fp_text[prev:s])
        out.append(rep)
        prev = e
    out.append(fp_text[prev:])
    return ''.join(out)


def _move_zone_pts(fp_text: str, ref: str, old_x: float, old_y: float,
                   delta_rot: float, dx: float, dy: float) -> str:
    """Translate and rotate a footprint's own keep-out zone `(pts ...)`.

    A footprint-internal `(zone ...)` stores its `(polygon (pts (xy ...)))` in
    ABSOLUTE board coordinates -- the same fact `_FLIP_NAMED_REFUSALS['zone']`
    records for the mirror. The move/rotate path of `write_placed_output`
    rewrites the footprint's `(at ...)` and rotates pad and text angles, but
    until this existed left the zone's `(pts ...)` untouched, so moving a
    footprint left its keep-out zone behind at the old absolute position
    (#1214).

    Each point is carried by the same rigid transform as the footprint: rotate
    about the footprint's OLD origin by `delta_rot`, then translate by
    `(dx, dy)`. The rotation is KiCad's, which with y pointing down means the
    standard matrix at the NEGATED angle -- the one
    `kicad_parser.local_to_global` places the pads with, so the zone turns
    with its pads. At +delta_rot the zone turns the other way, which agrees
    with the pads only at 0 and 180 degrees.

    A footprint `(point ...)` is absolute too and is carried the same way. A
    mirror is not a rigid motion, so the flip mirrors zones and points first
    (`_flip_zone`, `_flip_point`) and then calls this for the rigid rest of
    the pose change.

    A zone whose `(pts ...)` does not parse is a refusal, not a skip: the
    writer returns True unconditionally, so a silently-untouched zone is
    indistinguishable from success at every call site, and a keep-out zone
    left behind is exactly the failure this fix exists to remove.
    """
    if dx == 0 and dy == 0 and delta_rot % 360 == 0:
        # An unchanged pose must write the zone byte for byte, or the
        # identity write (test_714_identity_write_unchanged) re-spells it.
        return fp_text
    rad = math.radians(-delta_rot)  # KiCad's sense, as local_to_global
    c, s_ = math.cos(rad), math.sin(rad)

    def carried(fx, fy):
        rx = old_x + (fx - old_x) * c - (fy - old_y) * s_
        ry = old_y + (fx - old_x) * s_ + (fy - old_y) * c
        # Minimal decimals, as KiCad writes them: a flip composed with a
        # rotation and then undone must give the zone back byte for byte.
        return f"{_fmt_mm(rx + dx)} {_fmt_mm(ry + dy)}"

    pieces = []
    for head, s, e in _iter_sexpr_children(fp_text, 0):
        node = fp_text[s:e]
        if head == 'point':
            # A footprint `(point ...)` is absolute too (pcbnew 10.0 probe),
            # so it is carried by the same motion as the zone.
            m = re.search(r'\(at\s+([^\s()]+)\s+([^\s()]+)\)', node)
            if not m:
                raise SideFlipUnsupported(
                    f"{ref}: a (point ...) has no two-number (at x y) this "
                    f"move can carry: {node[:120]!r}")
            rep = (node[:m.start()]
                   + f"(at {carried(float(m.group(1)), float(m.group(2)))})"
                   + node[m.end():])
            pieces.append((s, e, rep))
            continue
        if head != 'zone':
            continue
        if re.search(r'\(filled_polygon\b', node):
            # A fill is also absolute, and a moved outline with a stale fill
            # is a second way to ship copper where the part used to be.
            raise SideFlipUnsupported(
                f"{ref}: a footprint-internal (zone ...) carries a "
                f"(filled_polygon ...), which this move does not carry. "
                f"Refusing rather than leaving its fill behind: {node[:120]!r}")
        polygons = list(re.finditer(
            r'\(polygon\s+\(pts\s+((?:\(xy\s+[^\s()]+\s+[^\s()]+\)\s*)+)\)\s*\)', node))
        if len(polygons) != len(re.findall(r'\(polygon\b', node)):
            raise SideFlipUnsupported(
                f"{ref}: a footprint-internal (zone ...) has a (polygon ...) "
                f"this move cannot parse -- expected a (pts (xy x y) ...) "
                f"list. Refusing rather than moving the footprint and "
                f"leaving its keep-out zone behind: {node[:120]!r}")
        if not polygons:
            continue
        rep_parts = []
        prev_end = 0
        for m in polygons:
            # Each point rewritten where it stands, so the polygon keeps the
            # board's own layout and a move undone gives back the same bytes.
            rep_parts.append(node[prev_end:m.start()])
            rep_parts.append(re.sub(
                r'\(xy\s+([^\s()]+)\s+([^\s()]+)\)',
                lambda p: f"(xy {carried(float(p.group(1)), float(p.group(2)))})",
                m.group(0)))
            prev_end = m.end()
        rep_parts.append(node[prev_end:])
        rep = ''.join(rep_parts)
        if rep != node:
            pieces.append((s, e, rep))
    if not pieces:
        return fp_text
    out = []
    prev = 0
    for s, e, rep in pieces:
        out.append(fp_text[prev:s])
        out.append(rep)
        prev = e
    out.append(fp_text[prev:])
    return ''.join(out)


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

    Refusing is cheap, measured over the stress corpus's 71503 footprint
    blocks: about 200 refuse, most of them per-layer padstacks, the rest a few
    footprints each (`rect_delta`, `dimension`, KiCad 6's `tedit`). Every
    construct the corpus carries in quantity has a rule, compared against
    pcbnew's own flip by `tests/gui_parity/test_714_mirror_pcbnew_parity.py`
    on corpus parts (`tests/fixtures/714/flip_witnesses.kicad_pcb`).
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
# Which raises the obvious objection: `rect_delta` is refused for want of a
# measured rule, so why are these passed through? Because the doctrine is
# about GEOMETRY, not about familiarity. Every name here is a scalar, an
# identifier or a flag that a reflection cannot act on -- a cost weight, a
# uuid, a margin, a pad-number group, a version string. `rect_delta` describes
# SHAPE, and its behaviour under a mirror is a real question; `chamfer` was
# one too, until pcbnew's own flip answered it (`_flip_chamfer`).
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
    # Data a reflection cannot act on, which pcbnew 10.0's flip leaves
    # unchanged: an embedded file's name and checksum, the placed flag,
    # component classes, a footprint paste ratio, and a footprint-internal
    # group (its members are uuids, which the flip keeps). Pinned by the
    # corpus parts in `tests/fixtures/714/flip_witnesses.kicad_pcb`.
    'embedded_files', 'placed', 'component_classes',
    'solder_paste_margin_ratio', 'group',
})
_FLIP_GRAPHICS = frozenset({'fp_line', 'fp_rect', 'fp_circle', 'fp_poly',
                            'fp_curve', 'fp_arc'})
_FLIP_TEXTS = frozenset({'property', 'fp_text'})
# `(layer ...)` and `(at ...)` are the footprint's OWN; `pad` and
# `private_layers` have their own handlers.
_FLIP_HANDLED = (_FLIP_GRAPHICS | _FLIP_TEXTS
                 | {'layer', 'at', 'pad', 'private_layers', 'zone', 'point'})

# Constructs we refuse BY NAME, so the message can say which and why, rather
# than falling through the generic "unknown node kind" arm.
_FLIP_NAMED_REFUSALS = {
    'fp_text_box': "no tracked board carries one, so no gate here can validate a rule for it",
}
# ...and inside a pad.
_PAD_REFUSALS = {
    'rect_delta': ("trapezoid pad deltas under a mirror are unmeasured -- 4 "
                   "corpus footprints carry one"),
    'padstack': ("a per-layer padstack swaps its front definition (the pad's "
                 "own shape, size and options) with its B.Cu entry field by "
                 "field under pcbnew's flip, and this mirror has no rule for "
                 "that swap"),
}

# Pad children this mirror knows what to do with. `at`, `layers` and `drill`
# are TRANSFORMED; the rest are mirror-invariant -- sizes, margins, net and pin
# metadata, thermal and teardrop settings, which a reflection does not touch.
# Measured: these heads are every pad child kind on the 22 tracked boards
# (9491 pads), plus the ones the stress corpus carries in quantity. Anything
# outside the set refuses, so a KiCad that adds a pad child stops the flip
# instead of being emitted unmirrored.
_PAD_HANDLED = frozenset({
    'at', 'size', 'layers', 'uuid', 'net', 'pintype', 'pinfunction',
    'roundrect_rratio', 'drill', 'remove_unused_layers', 'solder_paste_margin',
    'solder_mask_margin', 'teardrops', 'zone_connect', 'property', 'options',
    'clearance', 'thermal_bridge_angle', 'solder_paste_margin_ratio',
    'thermal_bridge_width', 'thermal_gap', 'thermal_width', 'die_length',
    'zone_layer_connections', 'tstamp', 'locked', 'chamfer_ratio',
    # A flag, left alone by pcbnew's flip.
    'keep_end_layers',
    # TRANSFORMED in `_flip_pad`: a custom pad's sub-shapes and the chamfered
    # corner names.
    'primitives', 'chamfer',
    # #1248: KiCad 10's pad-to-die DELAY (ps) is a per-pad scalar like
    # `die_length` and mirror-invariant the same way: pcbnew's FOOTPRINT::Flip
    # leaves both values unchanged (probed on 10.0.0). No tracked board
    # carries either token, so the parity gate and the round trip pin it on
    # an injected tigard J7; without it a part carrying one refused to flip.
    'die_delay',
})

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


def _flip_layer_nodes(node: str, flip=flip_layer_token) -> str:
    """Toggle every `(layer "X")` token inside `node`."""
    def fix(m):
        return f'(layer "{flip(m.group(1))}")'
    return re.sub(r'\(layer\s+"([^"]+)"\)', fix, node)


def board_inner_copper_count(board_text: str) -> int:
    """The number of inner copper layers the board's `(layers ...)` table
    declares: its copper layers less the two faces."""
    from kicad_parser import _layer_table
    return max(0, len(_layer_table(board_text)[1]) - 2)


def _block_layer_flip(ref: str, inner_count: Optional[int]):
    """The layer-token flip for one footprint block: the board's inner copper
    mirrored through the stack (`flip_layer_token`), and an inner layer
    REFUSED when the caller did not say how many there are -- its mirror
    depends on the count, and passing it through unchanged is what pcbnew
    10.0.0 did and 10.0.6 no longer does."""
    def flip(name: str) -> str:
        if _INNER_LAYER.match(name) and inner_count is None:
            raise SideFlipUnsupported(
                f"{ref}: an item on the inner copper layer {name!r} mirrors "
                f"through the board's stack, and this flip was not given the "
                f"board's inner-layer count. Refusing.")
        return flip_layer_token(name, inner_count)
    return flip


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

    # A custom pad's sub-shapes and a chamfered pad's corner names live in the
    # pad's OWN frame, which the flip mirrors in y like the footprint's.
    # Rewritten first, on the spans just walked, so nothing above shifts them.
    subs = []
    for head, s, e in _iter_sexpr_children(node, 0):
        if head == 'primitives':
            subs.append((s, e, _flip_primitives(node[s:e], ref)))
        elif head == 'chamfer':
            subs.append((s, e, _flip_chamfer(node[s:e], ref)))
    for s, e, new in reversed(subs):
        node = node[:s] + new + node[e:]

    lm = re.search(r'\(layers\b', node)
    if lm:
        lend = find_matching_paren(node, lm.start())
        # The names AFTER the keyword, quoted or bare: KiCad 6 writes a pad's
        # list bare, and a quoted-only rewrite left such a pad on the face the
        # part just left.
        body = node[lm.end():lend]
        toks = layer_list_tokens(body)
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
        node = node[:lm.end()] + map_layer_list_tokens(body, flip_layer_token) + node[lend:]

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


def _swap_arc_ends(node: str) -> str:
    """Exchange an arc's `(start ...)` and `(end ...)` points.

    An arc's SWEEP is start->mid->end. Mirroring the three points reverses the
    sweep, so KiCad exchanges start and end to restore it. Measured on sonde_u
    U2 and rp2350 SW1, and on a custom pad's `gr_arc` primitives. This is
    invisible to every geometric check -- the same curve passes through the
    same three points either way -- which is why the acceptance gate compares
    nodes.
    """
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


def _flip_graphic(node: str, head: str, ref: str, flip=flip_layer_token) -> str:
    """Mirror one `fp_*` graphic node."""
    node = _mirror_xy_pairs(node, ref)
    node = _flip_layer_nodes(node, flip)
    if head == 'fp_arc':
        node = _swap_arc_ends(node)
    return node


# The sub-shapes a custom pad's `(primitives ...)` carries on the corpus
# (4586 gr_arc, 3715 gr_poly, 1313 gr_line, 742 gr_circle, 188 gr_rect).
_PRIMITIVE_KINDS = frozenset({'gr_poly', 'gr_line', 'gr_arc', 'gr_circle',
                              'gr_rect'})


def _flip_primitives(node: str, ref: str) -> str:
    """Mirror a custom pad's `(primitives ...)` in the pad's own frame.

    pcbnew 10.0 negates every sub-shape point's y, exactly as for a footprint
    graphic, and exchanges a `gr_arc`'s start and end for the same reason an
    `fp_arc`'s are. A kind outside the measured five refuses, and so does an
    `(arc ...)` inside a polygon's point list: no flipped part carrying one has
    been compared against pcbnew.
    """
    pieces = []
    for head, s, e in _iter_sexpr_children(node, 0):
        child = node[s:e]
        if head not in _PRIMITIVE_KINDS:
            raise SideFlipUnsupported(
                f"{ref}: a custom pad's (primitives ...) carries a ({head} ...), "
                f"which this mirror has no rule for. Refusing to guess at its "
                f"mirror image.")
        if head == 'gr_poly' and re.search(r'\(arc\b', child):
            raise SideFlipUnsupported(
                f"{ref}: a custom pad's (gr_poly ...) carries an (arc ...) in "
                f"its point list, whose mirror image is unmeasured against "
                f"pcbnew. Refusing to guess at it.")
        new = _mirror_xy_pairs(child, ref)
        if head == 'gr_arc':
            new = _swap_arc_ends(new)
        pieces.append((s, e, new))
    for s, e, new in reversed(pieces):
        node = node[:s] + new + node[e:]
    return node


# KiCad's own order for a chamfer's corners, as it writes them.
_CHAMFER_ORDER = ('top_left', 'top_right', 'bottom_left', 'bottom_right')
_CHAMFER_MIRROR = {'top_left': 'bottom_left', 'bottom_left': 'top_left',
                   'top_right': 'bottom_right', 'bottom_right': 'top_right'}


def _flip_chamfer(node: str, ref: str) -> str:
    """Mirror a pad's `(chamfer <corners>)`: top and bottom exchange.

    Corners are named in the pad's own frame, which the flip mirrors in y, so
    pcbnew 10.0 writes `top_left` as `bottom_left` and `top_right` as
    `bottom_right` (and back); left and right stay. The result is written in
    KiCad's own corner order, which is what pcbnew writes.
    """
    m = re.match(r'\(chamfer((?:\s+[a-z_]+)*)\s*\)$', node)
    toks = m.group(1).split() if m else None
    if toks == []:
        return node                      # `(chamfer)`: no corner is cut
    if toks is None or any(t not in _CHAMFER_MIRROR for t in toks):
        raise SideFlipUnsupported(
            f"{ref}: a pad's {node!r} names a corner outside "
            f"{', '.join(_CHAMFER_ORDER)}. Refusing to guess at its mirror.")
    flipped = {_CHAMFER_MIRROR[t] for t in toks}
    return '(chamfer ' + ' '.join(c for c in _CHAMFER_ORDER if c in flipped) + ')'


def _mirror_about(tok: str, centre: str, ref: str, where: str) -> str:
    """`2 * centre - tok`, exactly, as a plain decimal literal.

    For a coordinate stored ABSOLUTE (a footprint zone's points, a `point`),
    the flip mirrors about the footprint's position rather than about zero.
    Decimal arithmetic on the two literals, so the flip and the flip back are
    an exact involution, the same property the sign toggle gives the local
    coordinates.
    """
    from decimal import Decimal
    for t in (tok, centre):
        if not _COORD_LITERAL.match(t):
            raise SideFlipUnsupported(
                f"{ref}: the coordinate {t!r} in {where} is outside the literal "
                f"grammar -?\\d+(\\.\\d+)?. Refusing rather than round-tripping "
                f"it through float.")
    v = 2 * Decimal(centre) - Decimal(tok)
    out = format(v, 'f')
    if '.' in out:
        out = out.rstrip('0').rstrip('.')
    return '0' if out in ('-0', '') else out


def _flip_layer_list(node: str, flip=flip_layer_token) -> str:
    """Toggle the names in `node`'s `(layers ...)`, keeping a list that names
    both faces as written: it is its own mirror, and pcbnew writes it in the
    same order."""
    lm = re.search(r'\(layers\b', node)
    if not lm:
        return node
    lend = find_matching_paren(node, lm.start())
    body = node[lm.end():lend]
    toks = layer_list_tokens(body)
    flipped = [flip(t) for t in toks]
    if set(flipped) == set(toks):
        return node
    if all(_copper_rank(t) is not None for t in flipped):
        # an all-copper list (a zone's) is written in layer-id order -- F.Cu,
        # B.Cu, In1.Cu, ... -- which an inner layer's mirror reverses
        ranked = iter(sorted(flipped, key=_copper_rank))
        return node[:lm.end()] + map_layer_list_tokens(body, lambda t: next(ranked)) \
            + node[lend:]
    return node[:lm.end()] + map_layer_list_tokens(body, flip) + node[lend:]


def _copper_rank(name: str) -> Optional[int]:
    """A copper layer's place in KiCad's layer-id order (F.Cu, B.Cu, In1.Cu,
    In2.Cu, ...), or None for any other layer."""
    if name == 'F.Cu':
        return 0
    if name == 'B.Cu':
        return 1
    m = re.match(r'In(\d+)\.Cu$', name)
    return 1 + int(m.group(1)) if m else None


def _flip_zone(node: str, ref: str, y0: str, flip=flip_layer_token) -> str:
    """Mirror a footprint-internal `(zone ...)` about the footprint's y.

    Its outline is stored in ABSOLUTE board coordinates, so pcbnew 10.0's flip
    sends each point to `y' = 2*y0 - y`, in the same order, and toggles the
    zone's layers. A fill is absolute too, and refuses here as it does on a
    move (`_move_zone_pts`).
    """
    if re.search(r'\(filled_polygon\b', node):
        raise SideFlipUnsupported(
            f"{ref}: a footprint-internal (zone ...) carries a "
            f"(filled_polygon ...), which this flip does not carry. Refusing "
            f"rather than leaving its fill behind: {node[:120]!r}")

    def fix(m):
        return f"(xy {m.group(1)} {_mirror_about(m.group(2), y0, ref, 'zone')})"

    node = re.sub(r'\(xy\s+([^\s()]+)\s+([^\s()]+)\)', fix, node)
    node = _flip_layer_nodes(node, flip)
    return _flip_layer_list(node, flip)


def _flip_point(node: str, ref: str, y0: str, flip=flip_layer_token) -> str:
    """Mirror a footprint `(point ...)`: an ABSOLUTE `(at x y)` sent to
    `y' = 2*y0 - y` by pcbnew 10.0's flip, and its layer toggled."""
    m = re.search(r'\(at\s+([^\s()]+)\s+([^\s()]+)\)', node)
    if not m:
        raise SideFlipUnsupported(
            f"{ref}: a (point ...) has no two-number (at x y) this mirror can "
            f"read: {node[:120]!r}")
    rep = f"(at {m.group(1)} {_mirror_about(m.group(2), y0, ref, 'point')})"
    node = node[:m.start()] + rep + node[m.end():]
    return _flip_layer_nodes(node, flip)


# #878: the rule itself lives in `legality`; this module reads it off
# raw text rather than off a parsed object, which is why it keeps its
# own function and only the collapse is shared. `legality`'s module
# scope is `math` + `typing`, so this edge introduces no cycle.
def _block_side(fp_text: str) -> str:
    """'F' or 'B' from the block's own `(layer ...)`.

    The same first-character rule `legality.footprint_side` applies to the
    parsed object, read here off the text so the writer and the side model
    cannot disagree about what a block says it is. Since #878 it CALLS that
    rule (`legality.side_of_layer`) rather than spelling it again, so the
    sentence above is structural instead of a promise two copies must keep.
    """
    # Below the docstring, deliberately: an import placed above it makes the
    # string a discarded expression and `__doc__` None, which is how this
    # function silently lost its documentation once already.
    from placement.legality import side_of_layer
    m = re.search(r'\(layer\s+"([^"]+)"\)', fp_text)
    return side_of_layer(m.group(1) if m else '')


def _flip_footprint_block(fp_text: str, ref: str, old_rot: float,
                          new_rot: float, old_y: Optional[str] = None,
                          inner_count: Optional[int] = None) -> str:
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

    `old_y` is the footprint's y BEFORE this write, as its literal: a
    footprint zone and a `point` are stored in absolute coordinates and mirror
    about it. The caller then carries them through the rest of the pose change
    (`_move_zone_pts`), exactly as it does on a move.

    `inner_count` is the board's number of inner copper layers: an item on
    `In<k>.Cu` mirrors to `In<n+1-k>.Cu`, as pcbnew does since 10.0.6, and
    without the count such an item is refused (`_block_layer_flip`).
    """
    flip = _block_layer_flip(ref, inner_count)
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
            new = _flip_layer_nodes(node, flip)
        elif head == 'private_layers':
            new = re.sub(r'"([^"]+)"', lambda m: '"' + flip(m.group(1)) + '"',
                         node)
        elif head == 'pad':
            new = _flip_pad(node, ref, old_rot, new_rot)
        elif head in ('zone', 'point'):
            if old_y is None:
                raise SideFlipUnsupported(
                    f"{ref}: a ({head} ...) is stored in absolute coordinates "
                    f"and mirrors about the footprint's own y, which this call "
                    f"was not given.")
            new = (_flip_zone if head == 'zone' else _flip_point)(node, ref, old_y, flip)
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
            # Inner copper counts as sided here: pcbnew 10.0 adds the flag to
            # a footprint text on `In1.Cu` / `In2.Cu` exactly as on a face.
            sided = bool(re.search(r'\(layer\s+"(?:[FB]\.|In\d+\.Cu")', node))
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
            new = _flip_layer_nodes(new, flip)
            if sided:
                new = _flip_justify(new, ref)
        else:
            new = _flip_graphic(node, head, ref, flip)

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
            - new_text_angle: float, OPTIONAL (#1163). Absent or None, every
              text composes with the rotation change as pcbnew does (the
              write is unchanged). Present, every top-level text node
              (`property`, `fp_text`) is written at this ABSOLUTE angle
              instead -- the pile builder's way to keep a text-to-part
              angle from handing back the rotation it replaced.

    Returns:
        True if output was written successfully
    """
    # Read the way KiCad reads it (#1149): a footprint block cut short by an
    # unbracketed teardrop token would rotate or flip only its first pad.
    from kicad_parser import read_board_text
    content = read_board_text(input_file, quiet=True)
    # the board's inner copper count: a flipped part's inner-layer items
    # mirror through the stack (`flip_layer_token`)
    inner_count = board_inner_copper_count(content)

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)

    placement_by_ref = {p['reference']: p for p in placements}
    modified_count = 0
    # #1322: a placement at the block's own pose rewrites the block and moves
    # nothing; counted apart, so `rotate R2 180` on a part already at 180 no
    # longer reports "Modified 1 footprint positions" beside place_pose's
    # "unmoved".
    unchanged_count = 0
    matched = set()
    unapplied_blocks = []

    # Blocks are NAMED by the parser's own resolver (#726). Before that, this
    # loop read each block's `(property "Reference" ...)` itself and looked the
    # STRING up, so one placement rewrote EVERY block carrying it: measured, a
    # single placement for esp_prog's `Ref*` moved both fiducials and left them
    # stacked 23.44 mm from where the second one belonged, and `perturb.
    # _all_at_current` (since retired) -- the identity write that built the
    # stress harness's ground-truth CONTROL board -- relocated 3 blocks on 2
    # corpus boards, up to 23.44 mm. (12 blocks across 5 boards, to 70.66 mm,
    # is the ceiling for a caller that places every PARSED footprint;
    # `state.parts` excludes the zero-pad blocks that make up the difference.)
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
    #
    # The rewritten board is assembled ONCE, from pieces collected back to
    # front: splicing each block into the whole string copied the board once
    # per moved part, which on a 54 MB board (ddr4_rdimm_tester) was 3 s of a
    # 300-part write and grows as parts x file size.
    pieces: List[str] = []
    tail = len(content)
    for start, end, fp_text, _raw_ref, key in reversed(
            list(iter_footprint_blocks(content))):
        placement = placement_by_ref.get(key)
        if placement is None:
            continue

        # Find the footprint's (at X Y [rotation]) - it's the first (at ...) in the block
        at_match = footprint_at_match(fp_text)
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
        # existed, which is what keeps every producer of these dicts
        # (place_simple, pose_ops, fanout_clearance, converge, net_rescue,
        # ...) unchanged.
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
        # An identity write hands this writer EVERY part at its current
        # pose, so the moved set cannot be read off the six-decimal `(at)`
        # formatting -- `perturb._all_at_current` (since retired) did, and
        # its dose-0 CONTROL board was exactly that call with no moves at all.
        # Gating on presence in `placements` therefore refused to write the
        # control board of every perturbation run on a board with an outline
        # owner, with a message blaming a mover that had not moved anything.
        # Measured before this qualifier existed.
        if (_OWNS_OUTLINE_RE.search(fp_text)
                and (abs(new_x - float(at_match.group(1))) > _POSE_EPS
                     or abs(new_y - float(at_match.group(2))) > _POSE_EPS
                     or abs((new_rot - old_rot + 180) % 360 - 180) > _POSE_EPS
                     # #714: a flip with identical x/y/rot is still a change,
                     # and on an outline owner it MIRRORS the owned Edge.Cuts
                     # geometry -- the board resize this guard exists to
                     # refuse. Without this term the flip walks straight past
                     # it. An identity write never sets `new_side`, so it is
                     # unaffected, which is the reason the rest of this
                     # predicate is written the way it is.
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
                                                old_rot, new_rot,
                                                old_y=at_match.group(2),
                                                inner_count=inner_count)
            # Zones and points mirrored about the OLD position, which leaves
            # them where pcbnew's bare flip puts them: the part at its old x/y
            # with its orientation negated. From there to the requested pose
            # is a rigid motion, the move path's own.
            new_fp_text = _move_zone_pts(
                new_fp_text, key,
                float(at_match.group(1)), float(at_match.group(2)),
                (new_rot + old_rot) % 360, new_x - float(at_match.group(1)),
                new_y - float(at_match.group(2)))
        else:
            delta_rot = (new_rot - old_rot) % 360
            if delta_rot != 0:
                new_fp_text = _rotate_pad_angles(new_fp_text, delta_rot)
                # The texts' angles are absolute too (pcbnew 10 probe), so
                # they compose the same way; leaving them was how a rotated
                # part shipped its Reference at the old angle.
                new_fp_text = _rotate_text_angles(new_fp_text, delta_rot, key)
            # A footprint-internal (zone ...) stores its (pts ...) in
            # ABSOLUTE board coordinates, so a move or rotation must carry
            # the zone's points with the footprint (#1214). Runs for a
            # pure move too, not just a rotation; _move_zone_pts is a
            # no-op when the pose is unchanged.
            new_fp_text = _move_zone_pts(
                new_fp_text, key,
                float(at_match.group(1)), float(at_match.group(2)),
                delta_rot, new_x - float(at_match.group(1)),
                new_y - float(at_match.group(2)))

        # #1163: an absolute text angle, after either path has composed its
        # texts, so it holds whatever the rotation or the side did.
        new_text_angle = placement.get('new_text_angle')
        if new_text_angle is not None:
            if isinstance(new_text_angle, bool) or \
                    not isinstance(new_text_angle, (int, float)):
                raise ValueError(f"{key}: new_text_angle must be a number of "
                                 f"degrees, not {new_text_angle!r}")
            new_fp_text = _set_text_angles(new_fp_text, float(new_text_angle),
                                           key)

        pieces.append(content[end:tail])
        pieces.append(new_fp_text)
        tail = start
        if (abs(new_x - float(at_match.group(1))) > _POSE_EPS
                or abs(new_y - float(at_match.group(2))) > _POSE_EPS
                or abs((new_rot - old_rot + 180) % 360 - 180) > _POSE_EPS
                or side_change):
            modified_count += 1
        else:
            unchanged_count += 1
    if pieces:
        pieces.append(content[:tail])
        content = ''.join(reversed(pieces))

    # Via-nudge rewrites (#313): remove each moved via from the input text and
    # append it at its new position, plus the new connector segments back to
    # the fanout stub start. Attached segments are untouched by design.
    if via_moves or new_segments:
        from plane_io import _remove_vias_at_positions
        from kicad_writer import generate_via_sexpr, generate_segment_sexpr
        from kicad_parser import is_immutable_copper
        n2n = getattr(pcb_data, 'net_id_to_name', None) if pcb_data is not None else None
        # A KiCad-locked via (#521) or one a `sense:` group holds (#1279) is
        # never re-emitted: deleting it and appending it again gives it a new
        # uuid, which takes it out of its group, and moves pinned copper. The
        # nudge never proposes one, so a move naming one is a caller's bug --
        # refused rather than written. The nudge moves its Via in place, so
        # the via is looked for at the move's old AND new position.
        for m in (via_moves or ()) if pcb_data is not None else ():
            nid = m[2]['net_id']
            for v in getattr(pcb_data, 'vias', ()) or ():
                if v.net_id == nid and is_immutable_copper(v) and any(
                        abs(v.x - x) < 1e-6 and abs(v.y - y) < 1e-6
                        for x, y in ((m[0], m[1]), (m[2]['x'], m[2]['y']))):
                    raise ValueError(
                        f"via move ({m[0]}, {m[1]}) -> ({m[2]['x']}, "
                        f"{m[2]['y']}) names a KiCad-locked or sense-held via; "
                        f"such a via is never re-emitted (#1279)")

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
                # structurally the same act. Falling back to the majority
                # spec of OTHER vias would be a THIRD answer for one that
                # already existed: it would stamp their spec onto this one.
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
                    inherit_when_unspecified=True,
                    # A nudged blind/micro via keeps its type word.
                    via_type=v.get('via_type')))
        for nsd in (new_segments or []):
            nm = n2n.get(nsd['net_id']) if n2n else None
            elements.append(generate_segment_sexpr(
                nsd['start'], nsd['end'], nsd['width'], nsd['layer'],
                nsd['net_id'], net_name=nm))
        if elements:
            close = content.rindex(')')
            content = content[:close] + '\n'.join(elements) + '\n)' + content[close+1:]

    # Outline tripwire (#829): would this write change the board OUTLINE? It
    # runs BEFORE the write and compares the ORIGINAL text against the
    # in-memory `content`, so nothing is written when it fires, and it still
    # works when `input_file is output_file` (route.py's scoped cap move).
    # It compares a structural FINGERPRINT, not `board_bounds`: bounds are
    # blind to an interior cutout moving and to a rotated circular window,
    # and rings are blind to an open stub that never chains. Only a board
    # with footprint-embedded Edge.Cuts arms it.
    _owners = _outline_owner_map(input_file)
    if _owners:
        from kicad_parser import structural_outline_fingerprint, read_board_text
        # Carried versus structural is decided ONCE, on the input (#1409): it
        # is judged at each part's current pose, so a part carrying a milled
        # window would be "carried" before a move off the board and
        # "structural" after it, and its own legal move read as the outline
        # changing. The same set masks both texts.
        _carried = {ref for ref, structural in _owners.items()
                    if not structural}
        # The same read as `content` (#1149), or a repaired footprint block
        # would read as an outline change.
        _before = structural_outline_fingerprint(
            read_board_text(input_file, quiet=True), carried=_carried)
        if _before != structural_outline_fingerprint(content,
                                                     carried=_carried):
            raise OutlineOwnerMove(
                f"this write would have CHANGED THE BOARD OUTLINE of "
                f"{output_file}. The placement stack does not resize boards: "
                f"size, cutouts and slots are mechanical decisions the user "
                f"owns. Nothing was written. This is a bug in whatever "
                f"produced the placement list (#829).")

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"Modified {modified_count} footprint positions"
          + (f" ({unchanged_count} placement(s) already at that pose, "
             f"unchanged)" if unchanged_count else ""))
    _report_unapplied(placement_by_ref, matched, unapplied_blocks,
                      'placement')
    print(f"Successfully wrote {output_file}")
    return True
