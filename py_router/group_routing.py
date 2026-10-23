"""Route, preview and unroute one placement BLOCK (#459 follow-on).

`placement/groups.py` answers "which footprints belong together". This answers
the three things you then want to do with that block:

    route    scope a routing run to just the block's nets
    preview  route it and report what WOULD change, writing nothing
    undo     remove the block's routed copper, putting it back to unrouted

Scope has two honest readings, and both are useful because which one you want
depends on the block:

    internal   nets whose every pad is inside the block. A schematic sheet is
               60-70% internal (glasgow's two 68-part sheets: 52 internal vs 23
               boundary), so "route this sheet" is a real, self-contained job.
    touching   nets with ANY pad inside the block -- the block's interface to the
               rest of the board included. This matches what `--component`
               already means, and it is the ONLY useful reading for a decap
               block: those are 0% internal by construction, since a decoupling
               cap bridges VCC to GND and both span the board.

Neither is right in general, so the caller picks.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Set, Tuple

SCOPES = ('internal', 'touching')


class GroupRoutingError(ValueError):
    """Bad block name or scope."""


def resolved_name(pcb_data, group_name: str, sources) -> Optional[str]:
    """The full block key `group_name` resolves to, or None.

    Separate from block_refs so a caller can ECHO what a short or fuzzy name
    matched: `--group U3 --undo` can resolve to `decap:U3` and start deleting
    copper without ever naming the block it picked.
    """
    import _placer_path  # noqa: F401  (placement lives in py_placer/)
    from placement.groups import derive_groups, short_name
    blocks = derive_groups(pcb_data, sources)
    if group_name in blocks:
        return group_name
    want = group_name.split(':', 1)[-1]
    hits = sorted(n for n in blocks
                  if group_name == short_name(n)
                  or want in (short_name(n).split(':', 1)[-1],
                              n.split(':', 1)[-1]))
    return hits[0] if len(hits) == 1 else None


def block_refs(pcb_data, group_name: str, sources) -> List[str]:
    """The footprints of `group_name`, or a helpful error naming what exists."""
    import _placer_path  # noqa: F401  (placement lives in py_placer/)
    from placement.groups import derive_groups, short_name
    blocks = derive_groups(pcb_data, sources)
    if group_name in blocks:
        return blocks[group_name]

    # A sheet block is named after a 36-character uuid, and the banner prints it
    # abbreviated -- `short_name` keeps the uuid's LAST 8 characters, because that is
    # the half that actually differs between sibling sheets. Accept what the
    # banner showed, with or without its `source:` prefix, and accept the bare
    # uuid. Only when it picks out exactly ONE block, though: silently taking the
    # first of several matches would route a different block than the user named.
    want = group_name.split(':', 1)[-1]
    hits = sorted(n for n in blocks
                  if group_name == short_name(n)
                  or want in (short_name(n).split(':', 1)[-1], n.split(':', 1)[-1]))
    if len(hits) == 1:
        return blocks[hits[0]]
    if len(hits) > 1:
        raise GroupRoutingError(
            f"{group_name!r} matches {len(hits)} blocks "
            f"({', '.join(short_name(h) for h in hits)}); name one exactly")

    avail = ', '.join(sorted(short_name(n) for n in blocks)[:12]) or '(none)'
    raise GroupRoutingError(
        f"no placement block named {group_name!r}. Available with these "
        f"sources: {avail}")


def block_net_names(pcb_data, refs, scope: str = 'touching') -> List[str]:
    """Net names in scope for a block, sorted.

    Nets with no name, and net 0, are skipped -- they are not routable targets.
    """
    if scope not in SCOPES:
        raise GroupRoutingError(
            f"unknown scope {scope!r}; choose from {', '.join(SCOPES)}")
    rs = set(refs)
    out = []
    for net_id, net in (pcb_data.nets or {}).items():
        if net_id <= 0 or not net.name:
            continue
        owners = {p.component_ref for p in net.pads if p.component_ref}
        if not owners or not (owners & rs):
            continue
        if scope == 'internal' and not owners <= rs:
            continue
        out.append(net.name)
    return sorted(out)


def describe_scope(pcb_data, refs, scope: str, final: Optional[int] = None) -> str:
    """One line describing what the block covers and what will be acted on.

    `final` is the count left AFTER intersecting with net patterns; pass it
    whenever patterns were given, so the line reports the nets actually in play
    rather than the block's full tally.
    """
    rs = set(refs)
    internal = touching = 0
    for net_id, net in (pcb_data.nets or {}).items():
        if net_id <= 0 or not net.name:
            continue
        owners = {p.component_ref for p in net.pads if p.component_ref}
        if not owners or not (owners & rs):
            continue
        touching += 1
        if owners <= rs:
            internal += 1
    chosen = internal if scope == 'internal' else touching
    note = ''
    if scope == 'internal' and internal == 0:
        note = ("  -- nothing to do: every net of this block crosses its "
                "boundary. Decoupling blocks are 0% internal by construction; "
                "try --group-scope touching.")
    elif final is not None and final < chosen:
        note = f"  ({chosen - final} dropped by the net patterns)"
    act = chosen if final is None else final
    return (f"  block: {len(refs)} part(s), {touching} net(s) touching, "
            f"{internal} internal -> acting on {act}{note}")


# --- undo -------------------------------------------------------------------

def _remove_arc_tracks(content: str, net_ids, net_names) -> Tuple[str, int]:
    """Delete `(arc ...)` TRACK blocks belonging to the given nets.

    The router never emits arcs, but KiCad routes rounded corners as them and
    hand-routed boards are full of them. The parser linearizes each arc into
    Segments (kicad_parser ~:2829), so they show up in `pcb_data.segments` and an
    undo counts them -- but the text writer only deletes `(segment ...)` blocks,
    so without this an "unrouted" net keeps its arcs and the tally silently
    disagrees with the file.

    Matched with the parser's OWN field pattern, so this can only ever delete
    something the parser classified as an arc track: a polygon's `(pts (arc
    (start)(mid)(end)))` entry carries no width/layer/net and cannot match.
    LOCKED arcs are left alone -- #521 gives locked copper no override.
    """
    import re

    from kicad_parser import find_matching_paren

    ids = {str(i) for i in net_ids}
    names = {n for n in net_names if n}
    fields = (r'\(arc\s+\(start\s+[\d.-]+\s+[\d.-]+\)\s+'
              r'\(mid\s+[\d.-]+\s+[\d.-]+\)\s+'
              r'\(end\s+[\d.-]+\s+[\d.-]+\)\s+'
              r'\(width\s+[\d.-]+\)\s+(?:\(locked\s+yes\)\s+)?'
              r'\(layer\s+"[^"]+"\)\s+(?:\(locked\s+yes\)\s+)?'
              r'\(net\s+(\d+|"[^"]*")\)')
    cuts, uuids = [], set()
    for m in re.finditer(fields, content, re.DOTALL):
        tok = m.group(1)
        hit = (tok in ids) if tok.isdigit() else (tok.strip('"') in names)
        if not hit:
            continue
        end = find_matching_paren(content, m.start())
        if end <= m.start():
            continue
        block = content[m.start():end]
        if '(locked yes)' in block:
            continue
        cuts.append((m.start(), end))
        u = re.search(r'\(uuid\s+"([^"]+)"\)', block)
        if u:
            # The linearized Segments inherit the arc's uuid, so the caller can
            # tell "left behind" from "removed as an arc" exactly.
            uuids.add(u.group(1))
    for a, b in reversed(cuts):        # right-to-left keeps earlier offsets valid
        content = content[:a] + content[b:]
    return content, len(cuts), uuids

def unroute_nets(content: str, pcb_data, net_names, input_file=None
                 ) -> Tuple[str, Dict[str, object]]:
    """Remove every segment and via of `net_names` from a board's TEXT.

    Returns (new content, report). Text surgery on the same helpers the
    dead-end sweep and the plane cleanup use, so KiCad 9/10 net spellings and
    quoted net names are handled the same way everywhere.

    Refuses KiCad-LOCKED copper. Locked means the user pinned it, and #521 gives
    that no override anywhere else in the toolchain -- an undo is not the place
    to invent one. A net whose copper a `sense:` group holds (#1279) is refused
    the same way: the undo strips a net whole, and the declared sense copper
    must survive it. Nets protected for a REASON (length-matched, time-matched,
    diff-pair) are removed: naming a net exactly is deliberate targeting, which
    is precisely the override those reasons are written to allow.

    What this does NOT undo, and cannot:
      * pad/target SWAPS. route.py can exchange which pad a net lands on, which
        mutates nets OUTSIDE the block, and no inverse exists in the codebase.
      * copper the router removed as a dead end or stale via -- that is gone
        from the input by the time you see the output.
      * ZONE pours (reported as `zones_left`). A filled plane IS copper on the
        net, so a net with a surviving pour is not back to "no copper". Deleting
        a pour is a plane decision, not an undo, so this reports and leaves it.
      * `(arc ...)` tracks and copper GRAPHICS (reported as `unmatched`). Both
        land in `pcb_data.segments` -- the parser linearizes arcs -- but the text
        writer only matches `(segment ...)` blocks; kicad_parser's own field doc
        says of graphics "writers cannot strip it (there is no (segment) block to
        match)". Counted and surfaced rather than silently left behind.
    A board that had swaps applied is not returned to its prior state by this;
    it is returned to "these nets carry no copper", minus the caveats above.

    The sibling project's `protected_nets` / `net_impedance` records are carried
    across UNCHANGED, so a net can stay marked length-matched or impedance-spec'd
    after its copper is gone. That is deliberate: the declaration describes what
    the net REQUIRES, not what is currently on the board, and re-routing it
    should honor the same spec. It does mean the record briefly describes copper
    that is not there.
    """
    from kicad_writer import remove_segments_from_content
    from plane_io import _remove_vias_at_positions
    from protected_nets import protection_map, is_sense_reason, never_lifts

    wanted = set(net_names)
    prot = protection_map(pcb_data, input_file)
    locked = sorted(n for n in wanted if prot.get(n) == 'locked')
    held = sorted(n for n in wanted if is_sense_reason(prot.get(n)))
    targets = sorted(wanted - set(locked) - set(held))

    ids = {nid for nid, net in pcb_data.nets.items() if net.name in targets}
    segs = [s for s in pcb_data.segments if s.net_id in ids]
    vias = [v for v in pcb_data.vias if v.net_id in ids]

    n2n = {nid: net.name for nid, net in pcb_data.nets.items()}
    removed_segs = removed_vias = 0
    unmatched: List = []
    if segs:
        # unmatched_out is the escape hatch every other caller of this helper
        # uses. Without it an (arc) track or a copper graphic -- both present in
        # pcb_data.segments, neither expressible as a (segment) block -- is left
        # in the file while the undo reports success.
        content, removed_segs = remove_segments_from_content(
            content, segs, n2n, unmatched_out=unmatched)
    # Arc tracks have no (segment) block, so the call above cannot touch them.
    content, removed_arcs, arc_uuids = _remove_arc_tracks(content, ids, targets)
    # Anything unmatched that came from an arc we just deleted is NOT left
    # behind; what remains is genuinely still in the file.
    unmatched = [s for s in unmatched if getattr(s, 'uuid', None) not in arc_uuids]
    if vias:
        content, removed_vias = _remove_vias_at_positions(
            content, [(v.x, v.y) for v in vias],
            net_ids=[v.net_id for v in vias],
            net_names=[n2n.get(v.net_id) for v in vias])
    # A filled pour is copper on the net; a net that still has one is not
    # "unrouted", whatever the segment tally says.
    tset = set(targets)
    zones_left = sorted({z.net_name for z in (getattr(pcb_data, 'zones', None) or [])
                         if z.net_name in tset})
    return content, {
        'nets': targets,
        'segments_removed': removed_segs,
        'segments_found': len(segs),
        'vias_removed': removed_vias,
        'vias_found': len(vias),
        'locked_skipped': locked,
        'held_skipped': held,
        'protected_overridden': sorted(
            n for n in targets if prot.get(n) and not never_lifts(prot[n])),
        'arcs_removed': removed_arcs,
        # Copper still in the file that no (segment) deletion could express.
        # Arcs are handled above, so what remains here is net-tagged copper
        # GRAPHICS -- user artwork, which an "undo routing" has no business
        # deleting, so it is reported instead.
        'unmatched': len(unmatched),
        'unmatched_nets': sorted({n2n.get(s.net_id) for s in unmatched
                                  if n2n.get(s.net_id)}),
        'zones_left': zones_left,
    }
