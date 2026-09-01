"""
Write placed PCB output by modifying footprint positions in the .kicad_pcb file.

Uses text-based manipulation (same approach as output_writer.py / kicad_writer.py)
to update the (at X Y [rotation]) of each footprint block.
"""
from __future__ import annotations

import os
import re
from typing import List, Dict

from kicad_parser import find_matching_paren
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

    footprint_starts = [m.start()
                        for m in re.finditer(r'\(footprint\s+"', content)]
    # Reverse order so string indices stay valid after replacements
    # (string-aware block ends -- the same #113 hazard as above).
    for start in reversed(footprint_starts):
        end = find_matching_paren(content, start)
        fp_text = content[start:end]

        ref_m = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
        if not ref_m:
            ref_m = re.search(r'\(fp_text\s+reference\s+"([^"]+)"', fp_text)
        if not ref_m:
            continue
        res = by_ref.get(ref_m.group(1))
        if res is None:
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

    Returns:
        True if output was written successfully
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)

    placement_by_ref = {p['reference']: p for p in placements}
    modified_count = 0

    # Find all footprint blocks and modify their (at ...) lines
    footprint_starts = [m.start() for m in re.finditer(r'\(footprint\s+"', content)]

    # Process in reverse order so string indices remain valid after replacements
    for start in reversed(footprint_starts):
        # String-aware, so a lone paren inside a property value (an MPN like
        # "TCR2EF115,LM(CT") cannot run the scan past this block and swallow the
        # next footprint -- issue #113, the same hazard placement/parser.py's
        # readers carry. A naive depth counter here would place the WRONG
        # footprint, or silently place none.
        end = find_matching_paren(content, start)
        fp_text = content[start:end]

        # Extract reference from (property "Reference" "XX" ...)
        ref_match = re.search(
            r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
        if not ref_match:
            continue
        ref = ref_match.group(1)

        if ref not in placement_by_ref:
            continue

        placement = placement_by_ref[ref]

        # Find the footprint's (at X Y [rotation]) - it's the first (at ...) in the block
        at_match = re.search(r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)',
                             fp_text)
        if not at_match:
            continue

        # Build replacement (at ...) string
        new_x = placement['new_x']
        new_y = placement['new_y']
        new_rot = placement['new_rotation']
        old_rot = float(at_match.group(3)) if at_match.group(3) else 0.0

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
                     or abs((new_rot - old_rot + 180) % 360 - 180) > _POSE_EPS)
                and _draws_board_outline(input_file, ref)):
            raise OutlineOwnerMove(
                f"{ref} draws the board outline (Edge.Cuts geometry outside "
                f"the board-level outline), so moving it would resize the "
                f"board. Refusing to write {output_file}. This is a bug in "
                f"whatever produced the placement list -- the movable-set "
                f"gates should never have offered {ref} a NEW pose (#829).")

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
    print(f"Successfully wrote {output_file}")
    return True
