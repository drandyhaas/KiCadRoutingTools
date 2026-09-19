"""Fabrication notes for geometry this tool creates that the fab must know about.

Right now: via-in-pad (#489 §8). The tool places it from three separate paths
(qfn_fanout `allow_via_in_pad`, `route_planes --same-net-pad-clearance -1`,
bga_fanout/underpad) and said nothing about the requirement it had just created.
Via-in-pad needs IPC-4761 Type VII -- filled, capped and plated -- or solder
wicks out of the joint into the barrel during reflow. A site is counted when
the via BARREL overlaps the pad copper, not merely when its centre is inside
(#695) -- an off-centre tap wicks just the same. The tool cannot enforce
that (it is a fab process, not board geometry), so the least it can do is COUNT
the sites and say so in the run output.

Leaf module: no routing imports, so any front (CLI main, shared engine, GUI tab)
can call it.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

# IPC-4761 Type VII is the protection class via-in-pad requires.
VIA_IN_PAD_FAB_NOTE = (
    "via-in-pad requires IPC-4761 Type VII (filled + capped + plated); "
    "state it on the fab drawing or solder wicks into the barrel")


#: #962: the per-via spec that DECLARES Type VII, in KiCad 10's whole-via
#: tokens (`(capping yes) (filling yes)`). This is what a tool-added via in a pad
#: or a paste opening is stamped with. Tenting, covering and plugging are not
#: stamped: they keep inheriting the board, and an override on them would be
#: the redundant stamp #741 removed.
TYPE_VII_STAMP = {'capping': 'yes', 'filling': 'yes'}

#: KiCad's factory policy for a token neither the via nor the board declares.
#: Duplicated from kicad_parser.VIA_PROTECTION_SETUP_DEFAULTS on purpose, to
#: keep this module a leaf; test_962_setup_via_protection pins the two equal.
_FACTORY_VIA_PROTECTION = {
    'tenting': '(front yes) (back yes)',
    'covering': '(front no) (back no)',
    'plugging': '(front no) (back no)',
    'capping': 'no',
    'filling': 'no',
}


def effective_via_protection(via_spec: Optional[Dict[str, str]],
                             setup: Optional[Dict[str, str]]) -> Dict[str, str]:
    """The protection a via will actually be FABRICATED with, token by token.

    A via's own spec overrides the board's `(setup ...)` one token at a time.
    KiCad writes a token only for an explicit override (probed, pcbnew
    10.0.0), so a via carrying only `(tenting ...)` still inherits capping and
    filling from the board. A token neither declares takes KiCad's factory
    value.
    """
    out = dict(_FACTORY_VIA_PROTECTION)
    for src in (setup or {}, via_spec or {}):
        for tok, inner in src.items():
            if tok in out:
                out[tok] = ' '.join(str(inner).split())
    return out


def is_filled_and_capped(effective: Dict[str, str]) -> bool:
    """Does an effective spec declare IPC-4761 Type VII (filled AND capped)?"""
    return (effective.get('capping', '').strip() == 'yes'
            and effective.get('filling', '').strip() == 'yes')


def via_paste_sites(vias, pcb_data, tol: float = 1e-6):
    """[(via, aperture, penetration_mm)] for every via whose BARREL overlaps a
    solder-paste opening that concerns its net (#962).

    The openings are `paste_apertures.apertures_for_net`. A foreign-net via
    inside an opening is already a short, which check_drc reports as such, so
    it is not also reported here. Uses the same barrel rule as
    `via_in_pad_sites` (#695): an off-centre via wicks solder just the same.
    Accepts vias as dicts or objects.

    A via is only under an F.Paste opening when its barrel reaches F.Cu (B side
    likewise): a buried In1-In2 via, or a blind F.Cu-In1 via under B.Paste,
    has no barrel open to that paste. A via with no layer span is a through
    via.
    """
    import paste_apertures as _pa
    out = []
    for via in vias or ():
        nid = _get(via, 'net_id')
        vx, vy = _get(via, 'x'), _get(via, 'y')
        if nid is None or vx is None or vy is None:
            continue
        vsz = _get(via, 'size', 0.6) or 0.6
        span = _get(via, 'layers', None) or ['F.Cu', 'B.Cu']
        best = None
        for ap in _pa.apertures_for_net(pcb_data, nid):
            if ('F.Cu' if ap.layer.startswith('F.') else 'B.Cu') not in span:
                continue
            b = ap.bounds
            r = vsz / 2.0
            if vx + r < b[0] or vx - r > b[2] or vy + r < b[1] or vy - r > b[3]:
                continue
            pen = _pa.via_paste_penetration(vx, vy, vsz, ap)
            if pen > tol and (best is None or pen > best[1]):
                best = (ap, pen)
        if best is not None:
            out.append((via, best[0], best[1]))
    return out


#: The first file format that can carry per-via `(capping ...)` /
#: `(filling ...)`. KiCad 10 added IPC-4761 via protection; KiCad 9.0's parser
#: has no case for either token and stops on an unknown one, so a stamped
#: 20241229 board would not open in KiCad 9. Duplicated from
#: kicad_parser.KICAD_10_MIN_VERSION to keep this module a leaf;
#: test_962_type_vii_stamp pins the two equal.
PER_VIA_PROTECTION_MIN_VERSION = 20250000


def via_snapshot(vias, pcb_data=None) -> List[tuple]:
    """`(net, x, y, size, spec, in_site)` of every via, taken BEFORE a run
    changes the board.

    It is what `via_protection_stamps` uses to tell a via this run ADDED from
    one the board already had. `spec` is the via's own protection spec, so a
    via stripped and re-laid on the same spot can get it back. `in_site` is
    whether the via was already in a same-net pad or paste opening, and is
    only known when `pcb_data` (the board BEFORE the run) is given, else
    None. A run that MOVES PARTS passes it: a cap pulled onto a same-net via
    turns a via nobody needed to protect into one under solder, and the run
    that did that owes it the Type VII declaration.
    """
    sites = None
    if pcb_data is not None:
        sites = {id(v) for v, _p in via_in_pad_sites(vias, pcb_data.pads_by_net)}
        sites |= {id(v) for v, _a, _pen in via_paste_sites(vias, pcb_data)}
    out = []
    for v in vias or ():
        nid = _get(v, 'net_id')
        x, y = _get(v, 'x'), _get(v, 'y')
        if nid is None or x is None or y is None:
            continue
        out.append((int(nid), float(x), float(y), float(_get(v, 'size', 0.6) or 0.6),
                    dict(_get(v, 'tenting_attrs', None) or {}),
                    None if sites is None else (id(v) in sites)))
    return out


def _input_match(via, snap_by_net):
    """The input via at `via`'s spot, as its snapshot entry `(x, y, size[,
    spec])`, or None.

    Same net, and within half the smaller diameter. The sub-grid nudge moves a
    via by at most size/4, so a nudged input via still reads as itself. A via
    the tool stripped and laid again on the same spot matches too: the file
    cannot tell the two apart, which is why the caller hands the input via's
    spec back rather than calling the new via the tool's own.
    """
    nid = _get(via, 'net_id')
    x, y = _get(via, 'x'), _get(via, 'y')
    sz = _get(via, 'size', 0.6) or 0.6
    for ent in snap_by_net.get(nid, ()):
        ix, iy, isz = ent[0], ent[1], ent[2]
        if math.hypot(x - ix, y - iy) <= max(1e-3, min(sz, isz) / 2.0):
            return ent
    return None


def _preexisting(via, snap_by_net) -> bool:
    """Is `via` at the spot of one the input board already had?"""
    return _input_match(via, snap_by_net) is not None


def _format_can_declare(pcb_data) -> bool:
    """Can this board's file format carry a per-via capping/filling token?

    A board parsed from text knows its version. One built from a live pcbnew
    board has none (0); there the running pcbnew decides, and
    `gui_utils.apply_via_protection` discloses a setter it lacks.
    """
    ver = getattr(pcb_data, 'kicad_version', 0) or 0
    return not (0 < ver < PER_VIA_PROTECTION_MIN_VERSION)


def via_protection_stamps(vias, input_snapshot, pcb_data):
    """Which shipped vias need IPC-4761 Type VII DECLARED on them (#962).

    A via is stamped with `TYPE_VII_STAMP` when ALL of these hold:
    - its barrel overlaps a same-net SMD pad (`via_in_pad_sites`) or a paste
      opening that concerns its net (`via_paste_sites`);
    - this run ADDED it: no input via sits at its spot (see `_input_match`).
      A via the board already had keeps whatever it had (#741). When the
      input via at that spot carried a spec and the shipped via does not, the
      via was stripped and laid again (`--force-reroute`, a rip-up), and the
      input's spec is handed BACK rather than lost (`restored`);
    - its spec (its own, or a restored one) declares neither capping nor
      filling. A spec that does is a decision, and is never overridden; one
      that only tents or covers is kept and Type VII is MERGED into it;
    - the board's own setup does not already make it filled AND capped;
    - the board's FILE FORMAT can carry the tokens (KiCad 10 and later, see
      `PER_VIA_PROTECTION_MIN_VERSION`). On an older format the via is
      counted `unstampable` and listed, so the requirement is disclosed for
      the fab drawing instead of written into a file KiCad 9 cannot open.

    One more way a via the input HAD is stamped: when the snapshot says it was
    NOT in a pad or paste opening before (`via_snapshot(..., pcb_data)`), and it
    is now. A part this run moved put solder on it (place_fanout_clearance
    pulls cap pads onto same-net vias by design), so this run created the site
    and declares it (`site_created`, counted inside `stamped`).

    Returns `(stamps, record)`:
    - `stamps` is a list of `(via, spec)`;
    - `record` is the machine-readable note: {count, sites, stamped,
      restored, protected, unstampable, unprotected, note}, where
      `unprotected` names each via-in-pad or via-in-paste that ships without
      Type VII, and why, so it is disclosed, not silent.
    """
    setup = getattr(getattr(pcb_data, 'board_info', None), 'via_protection_setup', None) or {}
    can_declare = _format_can_declare(pcb_data)
    snap_by_net: Dict[int, list] = {}
    for ent in input_snapshot or ():
        snap_by_net.setdefault(ent[0], []).append(tuple(ent[1:]))
    pad_sites = {id(v): p for v, p in via_in_pad_sites(vias, pcb_data.pads_by_net)}
    paste_sites = {id(v): ap for v, ap, _pen in via_paste_sites(vias, pcb_data)}
    stamps, sites, unprotected = [], [], []
    n_protected = n_restored = n_unstampable = n_site_created = 0
    for v in vias or ():
        vid = id(v)
        if vid not in pad_sites and vid not in paste_sites:
            continue
        if vid in paste_sites:
            where = paste_sites[vid].label()
        else:
            p = pad_sites[vid]
            where = '%s.%s' % (_get(p, 'component_ref', '?'), _get(p, 'pad_number', '?'))
        sites.append(where)
        own = _get(v, 'tenting_attrs', None) or {}
        if is_filled_and_capped(effective_via_protection(own, setup)):
            n_protected += 1
            continue

        def _unprot(why):
            unprotected.append({'site': where, 'x': round(_get(v, 'x'), 4),
                                'y': round(_get(v, 'y'), 4), 'why': why})
        # The spec the via ships with before any decision here: its own, or,
        # for one stripped and laid again on an input via's spot, the spec
        # that input via had (handed BACK, whatever it says).
        base = dict(own)
        restored = False
        match = _input_match(v, snap_by_net)
        if match is not None:
            in_spec = match[3] if len(match) > 3 else {}
            was_site = match[4] if len(match) > 4 else None
            if not own and in_spec:
                base, restored = dict(in_spec), True
            if was_site is not False:
                # The input had it here, already under solder (or nobody
                # knows): keep it as the input had it (#741).
                if restored:
                    stamps.append((v, base))
                    n_restored += 1
                    if not is_filled_and_capped(effective_via_protection(base, setup)):
                        _unprot("the input via's own spec, restored")
                else:
                    _unprot('own spec kept' if own
                            else 'at the spot of an input via, kept as the input had it')
                continue
            # The via was the input's, but it was NOT under solder there: a
            # part this run moved put a pad or paste opening on it. The site
            # is this run's, so the declaration is too.
            n_site_created += 1
        if 'capping' in base or 'filling' in base:
            # Someone DECIDED capping or filling for this via; that decision is
            # kept, never overridden.
            if restored:
                stamps.append((v, base))
                n_restored += 1
            _unprot('own spec kept')
            continue
        if not can_declare:
            n_unstampable += 1
            if restored:
                stamps.append((v, base))
                n_restored += 1
            _unprot('file format %s predates per-via capping/filling (KiCad 10)'
                    % (getattr(pcb_data, 'kicad_version', 0) or '?'))
            continue
        # Type VII MERGED into whatever the spec already says: a tenting-only
        # spec (every via on orangecrab and rp2350 carries one) says nothing
        # about capping or filling, so it does not block the declaration.
        stamps.append((v, dict(base, **TYPE_VII_STAMP)))
    record = {
        'count': len(sites), 'sites': sorted(set(sites)),
        'stamped': len(stamps) - n_restored, 'restored': n_restored,
        'protected': n_protected, 'unstampable': n_unstampable,
        'site_created': n_site_created,
        'unprotected': unprotected,
        'note': VIA_IN_PAD_FAB_NOTE,
    }
    return stamps, record


def apply_stamps_in_memory(stamps) -> int:
    """Set each stamped via's `tenting_attrs` (object or dict). The GUI path
    applies them to its pcbnew vias through `gui_utils.apply_via_protection`;
    the CLI writers emit them through `generate_via_sexpr`."""
    n = 0
    for v, spec in stamps:
        if isinstance(v, dict):
            v['tenting_attrs'] = dict(spec)
        else:
            v.tenting_attrs = dict(spec)
        n += 1
    return n


def ship_via_protection_file(output_file: str, input_snapshot, context: str = '',
                             quiet: bool = False):
    """Stamp Type VII onto the vias of a WRITTEN board that need it, and return
    the record (#962).

    For the CLI fronts, whose passes write through to the file: the board on
    disk is the final state. It re-parses it, decides with
    `via_protection_stamps`, and inserts the tokens into exactly those `(via
    ...)` blocks, matched by uuid. Nothing else in the file changes. Returns
    None when the file cannot be read.
    """
    import os
    if not output_file or not os.path.exists(output_file):
        return None
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import stamp_via_protection_in_content
    pcb = parse_kicad_pcb(output_file)
    stamps, record = via_protection_stamps(pcb.vias, input_snapshot, pcb)
    if stamps:
        with open(output_file, 'r', encoding='utf-8') as fh:
            content = fh.read()
        content, n = stamp_via_protection_in_content(
            content, {v.uuid: spec for v, spec in stamps if getattr(v, 'uuid', '')})
        with open(output_file, 'w', encoding='utf-8') as fh:
            fh.write(content)
        if n != len(stamps):
            # a via with no uuid, or a block the stamper would not touch
            record['unstampable'] = record.get('unstampable', 0) + (len(stamps) - n)
            record['written'] = n
    print_via_protection_record(record, context, quiet=quiet)
    return record


def print_via_protection_record(record, context: str = '', quiet: bool = False):
    """One FAB NOTE line for the run, when it shipped any via-in-pad/paste."""
    if quiet or not record or not record.get('count'):
        return
    where = f" ({context})" if context else ""
    unp = record.get('unprotected') or []
    print(f"\n  FAB NOTE{where}: {record['count']} via(s) in a pad or paste "
          f"opening [{', '.join(record['sites'][:8])}"
          f"{', +%d more' % (len(record['sites']) - 8) if len(record['sites']) > 8 else ''}]"
          f" -- {record['stamped']} stamped (capping yes) (filling yes), "
          f"{record.get('protected', 0)} already filled+capped"
          + (f", {record['restored']} given back the input via's own spec"
             if record.get('restored') else "")
          + (f", {record['unstampable']} NOT stampable (the file format cannot "
             f"declare it per via: put it on the fab drawing)"
             if record.get('unstampable') else "")
          + (f", {len(unp)} shipped WITHOUT Type VII" if unp else "")
          + f". {VIA_IN_PAD_FAB_NOTE}.")


def _get(obj, name: str, default=None):
    """Read `name` from a dict or an attribute-style object."""
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _pad_holds(pad, x: float, y: float, margin: float = 0.0) -> bool:
    """Is (x, y) within `margin` of the pad's COPPER?

    Exact when check_drc is importable (custom-pad polygons, roundrect
    corners, circle/oval, and rect_rotation), which is the same function --
    and the same function-local-import trick -- check_connected._point_in_pad
    uses. The import is inside the call so this module still IMPORTS as a
    leaf: any front can `import fab_notes` without dragging a checker in at
    module load, and only a point that survives the cheap reject pays for it.

    The reject is the DIAGONAL one, copied from check_connected, not an
    axis-aligned box. A box in board space clips a rotated pad's copper,
    because size_x/size_y stay in the PAD's frame and the tilt rides in
    rect_rotation -- a 1.0x0.2 pad at 45 degrees holds (0.35, 0.35) inside its
    copper, and a box gate rejects it, losing a real Type VII site.

    The box is not a safe credit either, in the other direction (#695):
    inflating one by the barrel radius over-credits by up to a factor of
    sqrt(2) along the diagonal, which is exactly where bga_fanout puts a
    dog-bone via (the half-pitch diagonal). Measured on ulx3s's BGA-381: a box
    named all 379 dog-bone vias as via-in-pad and the exact test named none,
    the nearest copper being 0.37mm from a 0.225mm barrel. So the fallback
    used when check_drc cannot be imported is the pad's circumscribed reach,
    which kills that corner artifact -- still not exact, and deliberately
    erring toward naming a site rather than missing one.
    """
    hx = _get(pad, 'size_x', 0.0) / 2
    hy = _get(pad, 'size_y', 0.0) / 2
    dx = x - _get(pad, 'global_x', 0.0)
    dy = y - _get(pad, 'global_y', 0.0)
    reach = max(hx, hy) + margin
    if dx * dx + dy * dy > reach * reach * 2:   # bbox-DIAGONAL prefilter
        return False
    try:
        from check_drc import point_to_pad_distance
    except ImportError:                          # no checker on this path
        # A point outside the pad's own circumscribed reach cannot be within
        # `margin` of copper that lives inside it. Only an ImportError is
        # caught: a geometry error must surface, not silently downgrade.
        return math.hypot(dx, dy) <= math.hypot(hx, hy) + margin
    return point_to_pad_distance(x, y, pad) <= margin


def via_overlaps_pad(pad, via_x: float, via_y: float, via_size: float,
                     margin: float = 0.0) -> bool:
    """Does a via's BARREL overlap this pad's copper? (#846)

    The public form of ``_pad_holds``, with the barrel radius applied for the
    caller. This is the question "is this via in that pad" actually means, and
    it is asked in two places that used to disagree: here, for the IPC-4761 fab
    note, and in ``qfn_fanout``'s commit loop, which decides whether to clamp
    the via to its pad edge (#202). The commit loop used to ask a 0.001mm
    CENTRE-coincidence question instead, so a via staggered onto its own pad
    was reported by this module as needing Type VII while shipping unclamped.

    ``via_size`` of 0 or None claims the 0.6 default's radius, as every other
    consumer of a size-less via in this repo does. ``margin`` is a FLOOR on the
    credit, not a replacement for it -- see ``via_in_pad_sites``.
    """
    vr = (via_size if via_size else 0.6) / 2.0
    return _pad_holds(pad, via_x, via_y, max(vr - 1e-6, margin))


def via_in_pad_sites(vias, pads_by_net: Dict[int, list],
                     margin: float = 0.0) -> List[Tuple[object, object]]:
    """[(via, pad)] for every via whose BARREL overlaps a SAME-NET pad.

    Overlap, not centre-containment (#695): an off-centre via-in-pad can have
    its centre just outside the pad outline while the barrel still overlaps
    the copper -- and solder wicks through copper continuity, not through the
    via's centre point, so that joint needs Type VII exactly as much. The
    router places such vias on purpose (QFN allow_via_in_pad, plane taps
    clamped to the pad edge, BGA underpad drops). Measured: the centre test
    missed 6 real sites on rp2350_fpga_eensy_prePlane alone, every one a true
    overlap against the exact pad shape.

    The credit must be EXACT, not a box inflated by the radius -- see
    _pad_holds. Over-counting here is not cheap: this note is a fab process
    requirement (filled, capped, plated), so a false site costs money, and
    bga_fanout hands this function its dog-bone vias, which sit on the
    half-pitch diagonal where a box over-credits worst.

    `margin` is a FLOOR on that credit, not a replacement for it: the barrel
    radius applies even when the caller passes 0. That is a change of meaning
    (#695) -- `margin` could once be used to ask for a credit TIGHTER than the
    barrel, and can no longer. No in-repo caller passes it.

    Same-net only: a via inside a FOREIGN pad is a short, not via-in-pad, and is
    the DRC checkers' business. Accepts vias as dicts or objects.
    """
    sites = []
    for via in vias or ():
        net_id = _get(via, 'net_id')
        if net_id is None:
            continue
        vx, vy = _get(via, 'x'), _get(via, 'y')
        if vx is None or vy is None:
            continue
        # A via with no declared size claims the 0.6 default's radius, as
        # every other consumer of a size-less via in this repo does.
        vsz = _get(via, 'size', 0.6)
        for pad in pads_by_net.get(net_id, ()) or ():
            if _get(pad, 'drill', 0.0) or 0.0:
                continue  # a plated TH pad's own barrel is not via-in-pad
            if via_overlaps_pad(pad, vx, vy, vsz, margin):
                sites.append((via, pad))
                break
    return sites


def via_in_pad_summary(vias, pads_by_net: Dict[int, list],
                       margin: float = 0.0) -> Optional[Dict]:
    """Machine-readable via-in-pad record, or None when there is none.

    Sites are barrel-overlap, not centre-in-pad -- see via_in_pad_sites.

    {'count', 'pads' (["U1.A1", ...]), 'note'} -- returned so a caller can put it
    in a run summary instead of only printing it.
    """
    sites = via_in_pad_sites(vias, pads_by_net, margin)
    if not sites:
        return None
    refs = []
    for _via, pad in sites:
        ref = f"{_get(pad, 'component_ref', '?')}.{_get(pad, 'pad_number', '?')}"
        if ref not in refs:
            refs.append(ref)
    return {'count': len(sites), 'pads': refs, 'note': VIA_IN_PAD_FAB_NOTE}


def print_via_in_pad_note(vias, pads_by_net: Dict[int, list],
                          context: str = "", margin: float = 0.0,
                          max_refs: int = 8) -> Optional[Dict]:
    """Print one fab note when this run put vias in pads. Returns the record.

    `margin` floors the barrel-overlap credit; see via_in_pad_sites.
    """
    record = via_in_pad_summary(vias, pads_by_net, margin)
    if not record:
        return None
    where = f" ({context})" if context else ""
    shown = record['pads'][:max_refs]
    more = len(record['pads']) - len(shown)
    print(f"\n  FAB NOTE{where}: {record['count']} via(s) placed in pad(s) "
          f"[{', '.join(shown)}{f', +{more} more' if more > 0 else ''}] -- "
          f"{VIA_IN_PAD_FAB_NOTE}.")
    return record
