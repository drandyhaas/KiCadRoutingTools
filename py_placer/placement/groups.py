"""Placement blocks: which footprints should move as one rigid body (#459).

The quench fixes what it can reach — individual parts within a few mm. The
failures that sink a board are block-scale: a magnetics block 80mm from its
endpoints, unroutable because of the floorplan rather than the routing. Before
anything can *move* a block, something has to know what a block IS, and nothing
in this repo did.

Four sources, in precedence order, first match wins per footprint. Each is
reported with its source so a user can see what was inferred before trusting it.

    kicad      KiCad (group ...) blocks -- the designer's own statement that
               these parts belong together, so it outranks everything inferred.
    sheet      Schematic sheet path. A sheet IS the functional block a floorplan
               thinks in, and it is declarative rather than guessed.
    netprefix  Net-name prefix (AUDIO_*, GPDI_*), gated on spatial coherence.
    decap      Two-pad capacitors tethered to the IC they decouple.

CORPUS EVIDENCE (measured across kicad_files/, not assumed):

  kicad      0 of 27 boards carry a single (group ...) block. Exact when present,
             but no in-repo board exercises it end to end.
  sheet      12 of 22 boards with (path ...) have more than one sheet. ulx3s: 11
             sheets sized 83/34/23/20/20/12 footprints. glasgow_revC: 4, nested.
             This is the workhorse.
  decap      Median distance from a 2-pad cap to the nearest IC bbox is 0.0-2.6mm
             across 8 boards, and the cap shares a net with that IC in 93-100% of
             cases (e.g. glasgow 91/92, orangecrab 75/75, ulx3s 57/70). The
             net-sharing check is what makes this safe: it is what separates "a
             cap that decouples this IC" from "a cap that happens to sit nearby",
             and on ulx3s it correctly rejects 13 of 70.
  netprefix  The WEAKEST source, and it is worth saying so. Raw prefixes are
             dominated by KiCad's auto-generated Net-(U1-Pad) names -- 21 to 92
             refs per board under a bogus "NET" prefix, spread over 16-75mm.
             With auto-generated and power names excluded and a 20mm coherence
             gate, what remains is small but genuine: ulx3s 9 blocks / 63 refs
             (AUDIO, GPDI, JTAG, LED, FPGA), orangecrab 13/80, glasgow 3/20,
             rp2350 1 block of 3, tigard 0. Enable it expecting little.

All output is sorted. Anything whose order can reach the optimizer must be
deterministic, or quench output stops being reproducible across processes (#457).
"""
from __future__ import annotations

import math
import re
from typing import Dict, List, Optional, Set, Tuple

# --- decap tethering -------------------------------------------------------
# 5mm from the IC's bounding box. The corpus median is 0.0-2.6mm and <5mm
# captures essentially every real decoupling cap; kit-dev (a large, loosely
# packed board) is the only one needing the full 5mm for 41 of its 53.
DECAP_RADIUS_MM = 5.0
DECAP_MIN_IC_PADS = 4          # what chip_boundary already calls "not a passive"

# --- netprefix -------------------------------------------------------------
# KiCad's auto-generated names carry no functional meaning: Net-(U1-Pad3),
# unconnected-(U2-Pad7). Bucketing on them produces one enormous "NET" pseudo
# block per board (up to 92 refs spread over 75mm), which is exactly the kind of
# wrong group that is worse than no group at all -- it would rigidly drag half
# the board.
_AUTO_NET = re.compile(r'^(net-\(|unconnected-\(|net\d)', re.I)
# Power and ground reach everywhere by design, so they say nothing about blocks.
# The pattern MOVED to `net_queries.is_power_net_name` (#705): the pin rule's
# rail-net fallback asks the same question, and a second copy of a predicate
# this tree already spells nine ways is exactly what it keeps paying for.
# Imported in the function body, matching this module's own idiom for a
# `py_router` sibling (see `decap_tethers`) -- a module-level import would need
# `py_router` on `sys.path` before `placement` is imported, which several test
# files do not arrange.
#
# It does EXTEND that requirement to `_from_netprefix`, which had no py_router
# dependency before: with only `py_placer` on the path it now raises
# ModuleNotFoundError where it used to work. Every in-tree caller is safe --
# `kicad_parser` also lives in `py_router`, so holding a real `PCBData` already
# implies it, and `swig_gui` inserts `py_router` first -- but the constraint is
# now wider than the sentence above, and saying so is cheaper than a reader
# rediscovering it.
_PREFIX = re.compile(r'^([A-Za-z][A-Za-z0-9]+)[_\-]')
NETPREFIX_MIN_REFS = 3
# Max distance from the cluster centroid to any member. A prefix scattered wider
# than this is a naming convention, not a physical block.
NETPREFIX_MAX_SPREAD_MM = 20.0

MIN_GROUP_SIZE = 2             # a block of one is not a block

SOURCES = ('kicad', 'sheet', 'netprefix', 'decap')
AUTO_SOURCES = ('kicad', 'sheet')   # the declarative ones


class GroupError(ValueError):
    """An unknown --group-by source."""


def parse_sources(spec: Optional[str]) -> Tuple[str, ...]:
    """'none' | 'auto' | comma list -> the ordered source tuple."""
    if not spec or spec.strip().lower() in ('', 'none'):
        return ()
    out: List[str] = []
    for tok in spec.split(','):
        tok = tok.strip().lower()
        if not tok:
            continue
        if tok == 'auto':
            out.extend(s for s in AUTO_SOURCES if s not in out)
            continue
        if tok not in SOURCES:
            raise GroupError(
                f"unknown group source {tok!r}; choose from "
                f"{', '.join(SOURCES)}, or 'auto' ({'+'.join(AUTO_SOURCES)}), "
                f"or 'none'")
        if tok not in out:
            out.append(tok)
    return tuple(out)


def _centroid(fp) -> Tuple[float, float]:
    pts = [(p.global_x, p.global_y) for p in fp.pads]
    if not pts:
        return (fp.x, fp.y)
    return (sum(p[0] for p in pts) / len(pts),
            sum(p[1] for p in pts) / len(pts))


def _sheet_of(fp) -> str:
    """The sheet prefix of a footprint's (path ...): everything but the LAST
    uuid, which is the symbol itself. A bare single-uuid path is top level and
    yields '' -- those must not all collapse into one bogus block."""
    parts = [q for q in (fp.sheet_path or '').split('/') if q]
    return '/'.join(parts[:-1])


def _from_kicad(pcb_data, movable) -> Dict[str, List[str]]:
    return {f"kicad:{name}": [r for r in refs if r in movable]
            for name, refs in (pcb_data.groups or {}).items()}


def _from_sheet(pcb_data, movable) -> Dict[str, List[str]]:
    out: Dict[str, List[str]] = {}
    for ref in movable:
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        sheet = _sheet_of(fp)
        if not sheet:          # top level: not a block
            continue
        out.setdefault(f"sheet:{sheet}", []).append(ref)
    return out


def _from_netprefix(pcb_data, movable) -> Dict[str, List[str]]:
    from net_queries import is_power_net_name
    buckets: Dict[str, Set[str]] = {}
    for net_id, net in (pcb_data.nets or {}).items():
        if net_id <= 0:
            continue
        leaf = (net.name or '').split('/')[-1]
        if not leaf or _AUTO_NET.match(leaf) or is_power_net_name(leaf):
            continue
        m = _PREFIX.match(leaf)
        if not m:
            continue
        for pad in net.pads:
            if pad.component_ref in movable:
                buckets.setdefault(m.group(1).upper(), set()).add(pad.component_ref)

    out: Dict[str, List[str]] = {}
    for prefix, refs in buckets.items():
        if len(refs) < NETPREFIX_MIN_REFS:
            continue
        pts = [_centroid(pcb_data.footprints[r]) for r in refs
               if r in pcb_data.footprints]
        if len(pts) < NETPREFIX_MIN_REFS:
            continue
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        # Spatial coherence: a prefix whose parts are scattered across the board
        # is a naming convention, not something that can move as one body.
        if max(math.hypot(p[0] - cx, p[1] - cy) for p in pts) > NETPREFIX_MAX_SPREAD_MM:
            continue
        out[f"net:{prefix}"] = sorted(refs)
    return out


def _net_name(pcb_data, net_id: int) -> str:
    """A net's name, or '' -- pcb_data.nets is keyed by id and may not have it."""
    n = pcb_data.nets.get(net_id)
    return getattr(n, 'name', '') or ''


def _pads_are_collinear(fp, eps: float = 1e-6) -> bool:
    """True when every pad of `fp` sits on one line -- a 1xN header, a pin
    strip, a castellated edge row.

    `build_chip_list` qualifies a footprint as a "chip" on PAD COUNT ALONE, so a
    1x10 castellated breakout row (10 pads) is an IC as far as it is concerned.
    That row spans a whole board edge and carries a rail, so it is nearer to
    half the decoupling caps than their real IC is and it passes the shared-net
    test -- it captures them. Measured on one board it captured three, and the grader
    then reported "C12 is 3.30mm from CN2, the IC it decouples".

    The false positive is only noise. The FALSE NEGATIVE is the reason this
    filter exists: a cap 2mm from the edge row and 8mm from the part it actually
    decouples grades CLEAN, because the tether measured to the row. That is a
    HARD spec limit (100nF within 3mm of every VDD pin) silently passing.

    Collinearity is the discriminator that needs no new parser field: at >= 4
    pads a real IC always has two-dimensional pad extent (SOIC, SOT-223, QFN,
    a 4-pad crystal), and a single row never does. `attr exclude_from_bom` would
    be the more direct signal, but the parser does not carry footprint attrs and
    adding one means touching both parse paths.
    """
    if fp is None or not fp.pads:
        return False
    xs = {round(p.global_x, 4) for p in fp.pads}
    ys = {round(p.global_y, 4) for p in fp.pads}
    return len(xs) <= 1 or len(ys) <= 1


def _copper_pads(fp) -> int:
    """How many of a footprint's pads carry copper.

    `chip_boundary.build_chip_list` gates on RAW pad count, and a KiCad 0201
    footprint carries two solder-paste apertures beside its two copper pads --
    so a two-terminal passive reaches four and is called a chip. The
    collinearity guard does not save it: at a non-orthogonal angle its two
    copper pads share neither an x nor a y, so the row test says "not a row".
    Measured, exactly two parts corpus-wide reach the chip list this way, both
    on rp2350: `C28` (Capacitor_SMD:C_0201_0603Metric at -45 degrees) and `R9`
    (the resistor of the same body at the same angle). `orangecrab_ext_pll C1`
    is the same four-pad shape and escapes only because it sits at 90 degrees,
    where the x coordinates collapse and the row test catches it -- so the old
    behaviour was rotation-dependent, which is not a property anyone chose.

    Found by review, and it was not theoretical: the pin rule graded C28 as an
    IC needing decoupling and reported `C28 pin 1 (/RP2354A/1V1) is 6.11mm from
    C30`, i.e. a capacitor flagged for being far from another capacitor.
    """
    if fp is None or not fp.pads:
        return 0
    return sum(1 for p in fp.pads
               if any(str(l).endswith('.Cu') for l in (p.layers or ())))


def _chip_list(pcb_data):
    """The ChipBoundary objects this module calls ICs. ONE answer, one place."""
    from chip_boundary import build_chip_list
    return [c for c in build_chip_list(pcb_data, min_pads=DECAP_MIN_IC_PADS)
            if not _pads_are_collinear(pcb_data.footprints.get(c.reference))
            and _copper_pads(pcb_data.footprints.get(c.reference))
            >= DECAP_MIN_IC_PADS]


def chip_refs(pcb_data) -> Set[str]:
    """The refs this module calls an IC: >= 4 COPPER pads, and not a row.

    Public since #792, because the seeder was answering the same question with
    `owner[0] == 'U'` -- a second answer to the problem `_pads_are_collinear`
    was written for, and the one without the measurement behind it. One
    question, one answer, and this is the one that carries its scar.
    """
    return {c.reference for c in _chip_list(pcb_data)}


def is_decoupling_cap(fp, ref: str) -> bool:
    """The syntactic half of "is this a decap": a `C*` bridging exactly two nets.

    TWO DISTINCT NETS, not two net-bearing PADS, and case-INSENSITIVE. Both
    differences were spelled three ways across this tree before #792 (here, in
    `seeder.decap_scope`, and in `emit_intent`'s census) and all three produce
    identical sets on every tracked board -- so the choice is argued, not
    measured:

    * a 2-pad part with both pads on ONE net is a zero-ohm link, not a decap
      (pad-count says yes, distinct-nets says no);
    * a 3-pad cap with a tied thermal pad IS a decap (pad-count says no).

    Both differences favour distinct-nets. `ref[:1]` rather than `ref[0]` also
    survives an empty reference instead of raising.
    """
    if fp is None or ref[:1].upper() != 'C':
        return False
    return len({p.net_id for p in fp.pads if p.net_id > 0}) == 2


def _elect_tethers(pcb_data, movable=None):
    """[(cap, ic|None, distance|None)] in sorted cap order -- the ONE election.

    **The radius never reaches here, and that is the entire point.** It used to
    live at the end of `decap_tethers` as a prune on the winner, which meant
    that asking the same question at two radii ran the election twice and made
    "the wider answer contains the narrower one" a per-board coincidence that a
    test had to re-verify. `decap_census`'s own comment asserted the opposite --
    that the unbounded pass "can pick a DIFFERENT chip, so this is a second
    measurement rather than a superset" -- and that was never true of this code:
    `radius` appeared only after the argmin, and `chips` and `ic_nets` are built
    before any cap is considered. Measured over the tracked corpus, 0 of 357
    tethers re-elect. With one election it is a property of the code shape
    instead of a fact about a corpus.

    `ic is None` means NO chip carries this cap's rail -- a bulk or filter cap
    upstream of an LC network, not a decoupler. Ten of ulx3s's caps are that,
    and they are why "in the seeder's scope" and "graded against an IC" are two
    different questions rather than two spellings of one (#792).
    """
    from net_queries import is_ground_net_name
    # THE SAME chip list `chip_refs` publishes, so the seeder's owner
    # test and the grader's tether election cannot answer "what is an
    # IC" differently -- which is the whole of #792.
    chips = _chip_list(pcb_data)
    if movable is None:
        movable = set(pcb_data.footprints)
    ic_nets = {}
    for c in chips:
        fp = pcb_data.footprints.get(c.reference)
        if fp is not None:
            ic_nets[c.reference] = {p.net_id for p in fp.pads if p.net_id > 0}

    out: List[Tuple[str, Optional[str], Optional[float]]] = []
    for ref in sorted(movable):
        fp = pcb_data.footprints.get(ref)
        if not is_decoupling_cap(fp, ref):
            continue
        nets = {p.net_id for p in fp.pads if p.net_id > 0}
        # Match on the POWER net, not on ground. A decoupling cap bridges a rail
        # and GND, and GND is shared with nearly every part on the board -- so
        # matching on "shares a net" lets the nearest 4-pad part win on ground
        # alone. Measured, that tethered a rail decap to the CRYSTAL and the
        # flash's own C2 to the USB connector, and the reported distance was then
        # to the wrong part in both directions.
        power = {n for n in nets
                 if not is_ground_net_name(_net_name(pcb_data, n))} or nets
        cx, cy = _centroid(fp)
        best, best_d = None, None
        for c in chips:
            if c.reference == ref:
                continue
            # Nearest chip THAT CARRIES THE RAIL, not nearest-then-reject.
            # Rejecting after the fact dropped the cap entirely whenever some
            # unrelated part happened to be closer, so a genuinely distant decap
            # went ungraded instead of flagged.
            if not (power & ic_nets.get(c.reference, set())):
                continue
            x0, y0, x1, y1 = c.bounds
            d = math.hypot(max(x0 - cx, cx - x1, 0.0), max(y0 - cy, cy - y1, 0.0))
            if best_d is None or d < best_d:
                best, best_d = c.reference, d
        # A second `nets & ic_nets[best]` recheck used to stand here, commented
        # "near, but not electrically its cap". It was UNREACHABLE: `power` is a
        # subset of `nets`, and `best` is only ever assigned inside the branch
        # that already required `power & ic_nets[c]` to be non-empty. Deleted
        # rather than kept, because a guard that cannot execute is worse than an
        # absent one -- it reads like protection, and its comment described an
        # ordering (proximity first, net-sharing second) that is the opposite of
        # what the loop above does.
        out.append((ref, best, best_d))
    return out


def decap_tethers(pcb_data, movable=None,
                  radius: float = DECAP_RADIUS_MM
                  ) -> Dict[str, List[Tuple[str, float]]]:
    """{IC ref: [(cap ref, bbox distance mm)]} -- the decoupling tethers.

    Nearest IC by bounding-box distance, but ONLY when the cap shares a net with
    it. Proximity alone is not enough -- on ulx3s 13 of 70 caps sit near an IC
    they have no electrical relationship with, and dragging those along would be
    a wrong group.

    Public, and returning the DISTANCE, because two consumers need the same
    tether and must not each decide what "near its IC" means: `_from_decap`
    wants the membership, and the floorplan grader's `decap_distance` rule
    (#549) wants the number. Grading the quench's own decap rule against a
    second implementation of it would measure the reimplementation, not the
    board.

    `movable` restricts which caps are considered (None = every footprint), as
    in `derive_groups`. Iteration is over a SORTED ref list: `movable` reaches
    us as a set, and cap order within an IC's list would otherwise vary with
    PYTHONHASHSEED (#457).

    Since #794 this is a thin filter over `_elect_tethers`. **Signature and
    result are unchanged, deliberately**: THREE callers depend on both --
    `_from_decap`, `reseat.clusters_from_tethers` and `board_brief` -- and
    `_from_decap` is what `--group-by decap` resolves through on BOTH the CLI
    and the GUI, so a change here is a change to grouping on two front ends.
    (`rule_decap_distance` and `decap_census` used to be a fourth and fifth;
    they now reach the election through `decap_populations`. An earlier draft
    of this line said six, which was the count before that refactor.)
    """
    out: Dict[str, List[Tuple[str, float]]] = {}
    for cap, ic, d in _elect_tethers(pcb_data, movable):
        if ic is not None and d <= radius:
            out.setdefault(ic, []).append((cap, d))
    return out


def decap_populations(pcb_data, movable=None,
                      radius: float = DECAP_RADIUS_MM):
    """(near, beyond, orphans) -- the in-scope caps, PARTITIONED, from one pass.

    * `near`    {ic: [(cap, d)]} -- byte-identical to `decap_tethers(radius)`;
                the population `rule_decap_distance` grades.
    * `beyond`  [(cap, ic, d)] cap-sorted -- elected, then pruned by the radius.
                Invisible to the grader before #794; `rule_decap_ungraded`
                reports them.
    * `orphans` [cap] -- in scope, but NO chip carries their rail. There is no
                IC for them to be far from, so the grader is right to ignore
                them; the SEEDER is not, which is #792.

    The three sets partition the in-scope population, and `decap_census` emits
    an `unaccounted` key whose only correct value is 0 so that stays a measured
    fact rather than a claim.
    """
    near: Dict[str, List[Tuple[str, float]]] = {}
    beyond: List[Tuple[str, str, float]] = []
    orphans: List[str] = []
    for cap, ic, d in _elect_tethers(pcb_data, movable):
        if ic is None:
            orphans.append(cap)
        elif d <= radius:
            near.setdefault(ic, []).append((cap, d))
        else:
            beyond.append((cap, ic, d))
    return near, beyond, orphans


def _from_decap(pcb_data, movable) -> Dict[str, List[str]]:
    """Two-pad caps tethered to the IC they decouple -- membership only.

    The tether itself (and its distance) is `decap_tethers`; this is the
    grouping view of it.
    """
    out: Dict[str, List[str]] = {}
    for ic, caps in decap_tethers(pcb_data, movable).items():
        refs = [cap for cap, _d in caps]
        # The IC itself anchors the tether, so it belongs to the block.
        if ic in movable:
            refs.append(ic)
        out[f"decap:{ic}"] = refs
    return out


_BUILDERS = {'kicad': _from_kicad, 'sheet': _from_sheet,
             'netprefix': _from_netprefix, 'decap': _from_decap}


def derive_groups(pcb_data, sources, movable=None) -> Dict[str, List[str]]:
    """{group name: sorted [reference]} for the requested sources.

    `movable` restricts membership to refs the caller is willing to move (locked
    parts, or the loop's target set); None means every footprint.

    Precedence is the order of `sources`, and a ref lands in at most ONE group:
    an explicit KiCad group must not be torn apart by a later sheet or prefix
    rule. Groups smaller than MIN_GROUP_SIZE after that are dropped.
    """
    if not sources:
        return {}
    if movable is None:
        movable = set(pcb_data.footprints)
    else:
        movable = set(movable)

    claimed: Set[str] = set()
    out: Dict[str, List[str]] = {}
    for src in sources:
        builder = _BUILDERS.get(src)
        if builder is None:
            raise GroupError(f"unknown group source {src!r}")
        for name, refs in sorted(builder(pcb_data, movable).items()):
            members = sorted({r for r in refs if r not in claimed})
            if len(members) < MIN_GROUP_SIZE:
                continue
            out[name] = members
            claimed.update(members)
    return out


def short_name(name: str) -> str:
    """Readable form of a group key. Sheet keys are uuid paths -- KiCad's
    Sheetname property is absent from every corpus board, so the uuid is all
    there is; show its TAIL rather than 36 meaningless characters.

    Public because it is what the user SEES: `--list-groups` prints this form,
    so `route.py --group` has to accept it back, and both need the same rule.
    """
    src, _, rest = name.partition(':')
    if src != 'sheet':
        return name
    return 'sheet:' + '/'.join(p[-8:] for p in rest.split('/') if p)


def describe(groups: Dict[str, List[str]], limit: int = 8) -> str:
    """One-line-per-block summary, for the run banner."""
    if not groups:
        return "  placement groups: none"
    lines = [f"  placement groups: {len(groups)} block(s), "
             f"{sum(len(v) for v in groups.values())} part(s)"]
    for name, refs in sorted(groups.items(), key=lambda kv: (-len(kv[1]), kv[0]))[:limit]:
        shown = ', '.join(refs[:6]) + (', ...' if len(refs) > 6 else '')
        lines.append(f"    {short_name(name)}  ({len(refs)}): {shown}")
    if len(groups) > limit:
        lines.append(f"    ... and {len(groups) - limit} more")
    return '\n'.join(lines)
