"""
Write placed PCB output by modifying footprint positions in the .kicad_pcb file.

Uses text-based manipulation (same approach as output_writer.py / kicad_writer.py)
to update the (at X Y [rotation]) of each footprint block.
"""
from __future__ import annotations

import re
from typing import List, Dict

from kicad_parser import find_matching_paren, iter_footprint_blocks
from kicad_writer import move_copper_text_to_silkscreen


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
    which carries no Reference node to edit.
    """
    missing = sorted(set(requested) - set(matched))
    if missing:
        print(f"WARNING: {len(missing)} {what}(s) matched no footprint block: "
              + ', '.join(repr(m) for m in missing[:12])
              + (' ...' if len(missing) > 12 else ''))
    if unaddressed:
        print(f"WARNING: {len(unaddressed)} {what}(s) name a reference-less "
              f"block, which has no Reference node: "
              + ', '.join(repr(u) for u in sorted(unaddressed)[:12]))


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
            # a result for one cannot be produced -- but say nothing silently.
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
    matched = set()

    # Blocks are NAMED by the parser's own resolver (#726). Before that, this
    # loop read each block's `(property "Reference" ...)` itself and looked the
    # STRING up, so one placement rewrote EVERY block carrying it: measured, a
    # single placement for esp_prog's `Ref*` moved both fiducials and left them
    # stacked 23.44 mm from where the second one belonged, and handing the
    # writer every part at the pose it already had relocated 12 blocks across
    # 5 corpus boards, up to 70.66 mm.
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
        matched.add(key)

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
    from placement import provenance
    provenance.record_write(input_file, output_file, placements,
                            pending=True)

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)

    provenance.commit_write(output_file)

    print(f"Modified {modified_count} footprint positions")
    _report_unapplied(placement_by_ref, matched, (), 'placement')
    print(f"Successfully wrote {output_file}")
    return True
