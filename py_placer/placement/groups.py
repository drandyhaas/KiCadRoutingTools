"""Placement blocks, and the decoupling facts several graders share.

A BLOCK is a set of footprints that belong together. Four sources derive
blocks (`derive_groups`), in precedence order, first match wins per
footprint; each block carries its source, so a user can see what was
inferred before trusting it:

    kicad      KiCad (group ...) blocks -- the designer's own statement that
               these parts belong together, so it outranks everything inferred.
    sheet      Schematic sheet path: a sheet is a functional block, declared
               rather than guessed.
    netprefix  Net-name prefix (AUDIO_*, GPDI_*), kept only when its parts lie
               within NETPREFIX_MAX_SPREAD_MM; auto-generated and power names
               are ignored. The weakest source.
    decap      Two-pad capacitors tethered to the IC they decouple
               (`decap_tethers`: the nearest chip within DECAP_RADIUS_MM that
               shares the cap's power net).

The default (`AUTO_SOURCES`) is the two declarative ones. Blocks are how an
intent names parts (`floorplan.resolve_blocks`), how `render_placement` frames
and colours a board (`--group-by`), and a routing scope (`route.py --group`).
Nothing moves a block as a body: `place_simple` keeps the functional groups
of the groups file (`grouping.py`) together through its cost.

The module also holds what the graders share about decoupling: which parts
are chips, which own a supply pin (`supply_owners`), and which cap decouples
which chip.

All output is sorted, so the same board gives the same blocks in every
process.
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
# block per board (up to 92 refs spread over 75mm), a wrong group worse than
# none.
_AUTO_NET = re.compile(r'^(net-\(|unconnected-\(|net\d)', re.I)
# Power and ground reach everywhere by design, so they say nothing about blocks.
# The pattern is `net_queries.is_power_net_name`, which the pin rule's
# rail-net fallback also reads.
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


def _pads(fp) -> list:
    """`fp`'s pads without its aperture-only ones (#1143): a paste or mask
    window is not a pin, so it moves neither a centroid, a row test nor a
    chip's bounds (tigard J1's 8 paste windows put C25 2.43 mm from it where
    its copper is 2.93 mm away)."""
    from kicad_parser import non_aperture_pads
    return non_aperture_pads(fp)


def _pads_view(pcb_data):
    """`pcb_data` as `chip_boundary.build_chip_list` reads it -- each
    footprint's `.pads` only -- with the aperture-only pads dropped. The
    router's own callers of `build_chip_list` are untouched."""
    from types import SimpleNamespace
    return SimpleNamespace(footprints={
        r: SimpleNamespace(pads=_pads(f))
        for r, f in (pcb_data.footprints or {}).items()})


def _centroid(fp) -> Tuple[float, float]:
    pts = [(p.global_x, p.global_y) for p in _pads(fp)]
    if not pts:
        return (fp.x, fp.y)
    return (sum(p[0] for p in pts) / len(pts),
            sum(p[1] for p in pts) / len(pts))


def sheet_and_symbol(fp) -> Tuple[str, str]:
    """`(sheet prefix, symbol uuid)` of a footprint's (path ...).

    The prefix is every uuid but the last (`_sheet_of`); the LAST is the
    schematic symbol. Two instances of one hierarchical sheet carry the SAME
    symbol uuid under different prefixes -- multichannel_mixer's R4, R15, R26
    and R37 all end in one uuid -- which is the correspondence KiCad's own
    "Repeat Layout" uses and `replicate` maps by (#1211). A footprint with no
    path yields ('', '')."""
    parts = [q for q in (fp.sheet_path or '').split('/') if q]
    return '/'.join(parts[:-1]), (parts[-1] if parts else '')


def _sheet_of(fp) -> str:
    """The sheet prefix of a footprint's (path ...): everything but the LAST
    uuid, which is the symbol itself. A bare single-uuid path is top level and
    yields '' -- those must not all collapse into one bogus block."""
    return sheet_and_symbol(fp)[0]


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
    pads = _pads(fp) if fp is not None else ()
    if not pads:
        return False
    xs = {round(p.global_x, 4) for p in pads}
    ys = {round(p.global_y, 4) for p in pads}
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


#: The roles `roles.infer_role` gives a part that has an IC's pad count and
#: is still never the IC a cap decouples (#1351): a resistor (a four-terminal
#: Kelvin shunt, an `R`-referenced array), a diode (an ESD array, a bridge
#: rectifier, an RGB LED filed under `D`) and an LED.
_PASSIVE_ROLES = ('resistor', 'diode', 'led')

#: A resistor NETWORK, which `infer_role` leaves `unknown` when its
#: reference is `RN` rather than `R`: by that reference (every `RN` part of
#: the stress corpus is one -- 238 owners, `R_Array_*`, `R_Cat16-*`, an
#: `MSOP-8` matched network), or by a footprint item name KiCad's library
#: and its look-alikes spell one with. Not `RP`, which is also an RP2040.
_RESISTOR_NETWORK_REF = re.compile(r'^RN\d', re.I)
_RESISTOR_NETWORK_FP = re.compile(
    r'(^|[^a-z0-9])(r_?array|resistor_?array|r_?network|r_?pack|r_cat\d)', re.I)

#: An addressable LED by family name, read off the footprint name and the
#: value with separators dropped (`APA-104-2020` -> `apa1042020`): an LED
#: with a controller inside and a VDD pin its datasheet puts a cap at. The
#: name is the evidence when the board carries no pin types or names
#: (piunora's `WS2812B3535`, nudac's `APA-104-2020-5mA`).
_ADDRESSABLE_LED = re.compile(r'ws28\d|sk68|sk98|apa10\d|neopixel|dotstar')


def _draws_supply(pcb_data, fp) -> bool:
    """Whether a copper pad of `fp` on a net other than ground is a SUPPLY
    pin by the symbol: typed `power_in`/`power_out`, or named as one
    (`net_queries.is_supply_pinfunction`). Ground is left out because an
    ESD array's GND pin is typed `power_in` too, and grounding is not being
    decoupled."""
    from net_queries import is_ground_net_name, is_supply_pinfunction, \
        is_supply_pintype
    for p in fp.pads or ():
        if p.net_id <= 0 or not any(str(l).endswith('.Cu') for l in (p.layers or ())):
            continue
        if is_ground_net_name(_net_name(pcb_data, p.net_id)):
            continue
        if (is_supply_pintype(str(getattr(p, 'pintype', '') or ''))
                or is_supply_pinfunction(str(getattr(p, 'pinfunction', '') or ''))):
            return True
    return False


def passive_by_role(pcb_data, ref: str) -> Optional[str]:
    """Why `ref` -- a part with an IC's pad count -- is no decap OWNER, or
    None (#1351).

    `_ic_list` admits a part on its shape alone, so a resistor array of
    pull-ups on +3V3 or an RGB LED whose common anode sits on +3V3 was "the
    IC it decouples": on rp2350 C25 graded 2.54 mm from RN1 and C22 clean at
    0.97 mm from it, while the nearest real IC on the rail (U2) is 4.85 mm
    from C22. A part that `infer_role` calls a resistor, a diode or an LED,
    or that is a resistor network (`_RESISTOR_NETWORK_*`), is not one --
    UNLESS a pin of it is a supply pin by its symbol (`_draws_supply`) or
    its name says it is an addressable LED (`_ADDRESSABLE_LED`). That
    exception is what keeps an addressable LED an owner: an SK6812 or
    WS2812 is a `D`/`LED` part with a VDD pin its datasheet puts a cap at.

    Measured over the stress corpus (~470 boards), the shape test admits
    1155 such parts on 140 boards. 757 are dropped: RGB and bicolour LEDs,
    resistor arrays (`R_Array_*`, `R_Cat16-*`), Kelvin shunts, ESD arrays
    (USBLC6, SP0503, PESD), bridge rectifiers; 171 of them on 67 boards
    carry a net some decoupling cap is on. The 398 kept are the
    addressable LEDs (SK68xx, WS28xx, APA102/104, one on an RGB 5050
    footprint) and apple1_mainboard's DIP ICs, whose grid references read
    as `D`. The one known miss is an IC filed under `D` with no pin
    metadata (fsae_custom_bms's SM91502 LED drivers). `capacitor` is
    deliberately not a role here: apple1_mainboard references its other
    ICs `C1`, `C10`, ..."""
    fp = (pcb_data.footprints or {}).get(ref)
    if fp is None:
        return None
    from .roles import infer_role, _norm
    role = infer_role(fp, ref, None)['role']
    name = getattr(fp, 'footprint_name', '') or ''
    item = name.split(':')[-1]
    if role not in _PASSIVE_ROLES:
        if not (_RESISTOR_NETWORK_REF.match(ref)
                or _RESISTOR_NETWORK_FP.search(item)):
            return None
        role = 'resistor network'
    if _draws_supply(pcb_data, fp):
        return None
    if _ADDRESSABLE_LED.search(_norm(name) + ' ' + _norm(getattr(fp, 'value', '') or '')):
        return None
    return f"role {role} with no supply pin"


def _ic_list(pcb_data):
    """The ChipBoundary objects of the parts that are ICs by SHAPE: >= 4
    copper pads, not a row, and not a passive by role (`passive_by_role`).
    `_chip_list` adds the supply owners to them."""
    from chip_boundary import build_chip_list
    return [c for c in build_chip_list(_pads_view(pcb_data),
                                       min_pads=DECAP_MIN_IC_PADS)
            if not _pads_are_collinear(pcb_data.footprints.get(c.reference))
            and _copper_pads(pcb_data.footprints.get(c.reference))
            >= DECAP_MIN_IC_PADS
            and passive_by_role(pcb_data, c.reference) is None]


#: A supply owner that is not an IC by shape has at least this many copper
#: pads: a regulator has an input, an output and a return (#1249). A
#: two-pad part typed `power_out` (a battery holder in some libraries) is a
#: source, not something a cap is placed to decouple.
SUPPLY_OWNER_MIN_PADS = 3

#: The roles `infer_role` may give a part the three-pad rule admits: an IC,
#: a regulator, or nothing the table knows. Any other role -- a transistor,
#: a feed-through capacitor, a three-pin header -- is a part whose two capped
#: nets do not make it the thing the caps decouple.
_THREE_PAD_ROLES = ('integrated circuit', 'regulator', 'unknown')


def _capped_nets(pcb_data) -> Set[int]:
    """Every net a decoupling cap (`is_decoupling_cap`) carries."""
    out: Set[int] = set()
    for ref, fp in (pcb_data.footprints or {}).items():
        if is_decoupling_cap(fp, ref):
            out.update(p.net_id for p in fp.pads if p.net_id > 0)
    return out


def supply_owner_evidence(pcb_data, ref: str,
                          capped: Optional[Set[int]] = None
                          ) -> Optional[str]:
    """Why `ref` owns decoupling caps although it is not an IC by shape, or
    None (#1249).

    `DECAP_MIN_IC_PADS` is four, so a three-pad regulator (SOT-89, TO-92)
    was never a chip: its input and output caps were elected to whatever
    other part carried the rail, and the grade called that part "the IC it
    decouples" (esp_prog C3 -> U1 at 2.125 mm, its own VOUT pin 1.12 mm
    away; complex_hierarchy C102 -> U102 at 35.08 mm). A part with at least
    `SUPPLY_OWNER_MIN_PADS` copper pads and no `part_class` (not a
    connector, switch, hole or marker) is an owner when ANY of:

    * a pad is typed `power_out` -- the symbol says it sources a rail
      (complex_hierarchy U101, a 78L05);
    * `roles.infer_role` calls it a `regulator` -- the one table
      `board_context` prints from (esp_prog U2, an ME6210 in SOT-89, which
      carries no pintype at all);
    * it has exactly three copper pads on three nets, one of them ground,
      and each of the other two carries a decoupling cap, and its role is
      an IC, a regulator or unknown (`_THREE_PAD_ROLES`). esp_prog U2 and
      complex_hierarchy U101 both meet it.

    No single signal covers both of those boards, which is why there are
    three. THE KNOWN CASE THE THIRD ADMITS THAT IS NOT A REGULATOR: kit-dev
    LV101, a DS1818 reset supervisor in TO-92 whose VCC (`+3.3V`) and RST
    nets each carry a cap. Admitting it changes 0 of that board's 53
    elections (every cap on those nets is nearer another chip). The shape
    says the part OWNS caps, not which of its pins are supply pins, so
    `floorplan.supply_pins` reads its pins by role only for the first two
    signals (`regulator_owners`); LV101 keeps the ordinary pin ladder, which
    takes its +3.3V pin and not its RST.
    """
    fp = (pcb_data.footprints or {}).get(ref)
    if fp is None:
        return None
    from .roles import _copper_pads as copper, infer_role, power_out_pads
    pads = copper(fp)
    if len(pads) < SUPPLY_OWNER_MIN_PADS:
        return None
    from .part_class import classify_part
    cls = classify_part(fp, ref).name
    if cls is not None:
        return None
    po = power_out_pads(fp)
    if po:
        return f"pad {po[0].pad_number} is typed power_out"
    role = infer_role(fp, ref, cls)
    if role['role'] == 'regulator':
        return 'role regulator (' + '; '.join(role['evidence']) + ')'
    if len(pads) != 3 or role['role'] not in _THREE_PAD_ROLES:
        return None
    nets = [p.net_id for p in pads]
    if min(nets) <= 0 or len(set(nets)) != 3:
        return None
    from net_queries import is_ground_net_name
    gnd = [n for n in nets if is_ground_net_name(_net_name(pcb_data, n))]
    if len(gnd) != 1:
        return None
    if capped is None:
        capped = _capped_nets(pcb_data)
    if not all(n in capped for n in nets if n not in gnd):
        return None
    return THREE_PAD_EVIDENCE


#: `supply_owner_evidence`'s answer for an owner the three-pad shape alone
#: admits -- the one kind `regulator_owners` leaves out.
THREE_PAD_EVIDENCE = ('3 copper pads: one ground net and two nets that each '
                      'carry a decoupling cap')


def supply_owners(pcb_data, ics=None) -> Dict[str, str]:
    """`{ref: evidence}` -- the cap owners that are not ICs by shape
    (`supply_owner_evidence`), ref-sorted. `ics` is the set to skip
    (default: `_ic_list`'s)."""
    if ics is None:
        ics = {c.reference for c in _ic_list(pcb_data)}
    capped = None
    out: Dict[str, str] = {}
    for ref, fp in sorted((pcb_data.footprints or {}).items()):
        # The pad count first: it is the cheap test, and it rejects every
        # two-pad passive before the classifier is asked.
        if ref in ics or _copper_pads(fp) < SUPPLY_OWNER_MIN_PADS:
            continue
        if capped is None:
            capped = _capped_nets(pcb_data)
        ev = supply_owner_evidence(pcb_data, ref, capped)
        if ev is not None:
            out[ref] = ev
    return out


def regulator_owners(pcb_data, ics=None) -> Dict[str, str]:
    """The supply owners a pin or the role table calls a regulator -- a
    `power_out` pad or the `regulator` role -- with their evidence; the
    three-pad shape alone does not say which pins are supply pins
    (`supply_owner_evidence`'s LV101). `floorplan.supply_pins` reads these
    by role."""
    return {r: ev for r, ev in supply_owners(pcb_data, ics=ics).items()
            if ev != THREE_PAD_EVIDENCE}


def _chip_list(pcb_data):
    """The ChipBoundary objects this module calls a cap's OWNER -- an IC by
    shape (`_ic_list`) or a supply owner (`supply_owners`, #1249). ONE
    answer, one place: `chip_refs`, `_elect_tethers`, `rail_chips` and
    through them the grade, the tether gate and `floorplan.supply_pins`
    all read it. In the footprints'
    own order, so the election's first-strict-minimum tie-break is the
    one it always was."""
    ics = _ic_list(pcb_data)
    have = {c.reference for c in ics}
    owners = supply_owners(pcb_data, ics=have)
    if not owners:
        return ics
    from chip_boundary import build_chip_list
    keep = have | set(owners)
    return [c for c in build_chip_list(_pads_view(pcb_data), min_pads=1)
            if c.reference in keep]


def chip_refs(pcb_data) -> Set[str]:
    """The refs this module calls a cap's owner: an IC (>= 4 COPPER pads,
    and not a row) or a supply owner (`supply_owners`). Public so every
    reader asks "is this an IC" the same way.
    """
    return {c.reference for c in _chip_list(pcb_data)}


def is_decoupling_cap(fp, ref: str) -> bool:
    """The syntactic half of "is this a decap": a `C*` bridging exactly two nets.

    TWO DISTINCT NETS, not two net-bearing PADS, and case-INSENSITIVE:

    * a 2-pad part with both pads on ONE net is a zero-ohm link, not a decap
      (pad-count says yes, distinct-nets says no);
    * a 3-pad cap with a tied thermal pad IS a decap (pad-count says no).

    Both differences favour distinct-nets. `ref[:1]` rather than `ref[0]` also
    survives an empty reference instead of raising.
    """
    if fp is None or ref[:1].upper() != 'C':
        return False
    return len({p.net_id for p in fp.pads if p.net_id > 0}) == 2


def decap_population(pcb_data) -> Dict[str, Tuple[Tuple[int, ...], List[str]]]:
    """{cap: ((rail net id, ...), [owner chips])} -- the decaps by the ONE
    rule place_simple prices (`grouping.BoardView.decaps`): a cap
    (`is_decoupling_cap`) from ground to a rail some chip other than a
    connector has a supply pin on, or between two rails that each carry a
    supply pin of one chip (a stacked domain: two rails). Its owners are
    those chips, in the footprints' order. An RC cap on a reset pin, a
    crystal's load cap, a cap on a connector's power pin is no decap here,
    so its distance sets no limit and is graded by none."""
    from placement.grouping import decap_view
    view = decap_view(pcb_data)
    rails, owners = view.decap_rails(), view.decap_owners()
    order = {r: i for i, r in enumerate(pcb_data.footprints or {})}
    return {cap: (rails[cap], sorted(owners[cap], key=lambda r: order.get(r, len(order))))
            for cap in view.decaps()}


def _elect_tethers(pcb_data, movable=None):
    """[(cap, ic|None, distance|None)] in sorted cap order -- the ONE election.

    Every decap (`decap_population`) against its owners, nearest wins: the
    cap's pad centroid to each owner's pad bbox (`chip_bounds_of`).

    **The radius never reaches here.** Callers prune the winner by distance
    afterwards, so asking at two radii runs ONE election, and the wider
    answer always contains the narrower one.

    `ic is None` only when no owner has copper pads to measure.
    """
    pop = decap_population(pcb_data)
    if movable is None:
        movable = set(pcb_data.footprints)
    fps = pcb_data.footprints or {}
    bounds: Dict[str, object] = {}
    out: List[Tuple[str, Optional[str], Optional[float]]] = []
    for ref in sorted(movable):
        hit = pop.get(ref)
        if hit is None:
            continue
        cands = []
        for ic in hit[1]:
            if ic not in bounds:
                bounds[ic] = chip_bounds_of(fps[ic])
            if bounds[ic] is not None:
                cands.append((ic, bounds[ic]))
        best, best_d = nearest_chip(_centroid(fps[ref]), cands)
        out.append((ref, best, best_d))
    return out


def _cap_power_nets(pcb_data, fp) -> Set[int]:
    """The nets a cap is tethered ON. Match on the POWER net, not on ground. A
    decoupling cap bridges a rail and GND, and GND is shared with nearly every
    part on the board -- so matching on "shares a net" lets the nearest 4-pad
    part win on ground alone. Measured, that tethered a rail decap to the
    CRYSTAL and the flash's own C2 to the USB connector, and the reported
    distance was then to the wrong part in both directions."""
    from net_queries import is_ground_net_name
    nets = {p.net_id for p in fp.pads if p.net_id > 0}
    return {n for n in nets
            if not is_ground_net_name(_net_name(pcb_data, n))} or nets


def nearest_chip(pt, candidates) -> Tuple[Optional[str], Optional[float]]:
    """`(ref, distance)` -- the election's argmin over `(ref, bounds)`
    candidates in the order given (the first strict minimum wins), or
    `(None, None)`. `_elect_tethers` calls it over the chips carrying a cap's
    rail; the decap tether gate (`quench.TetherGateView`, which
    `place_fanout_clearance --intent` runs) calls it over the same chips at
    LIVE poses, so the pair it holds is the pair the grade will elect."""
    best, best_d = None, None
    for ref, bounds in candidates:
        d = _point_to_bounds(pt, bounds)
        if best_d is None or d < best_d:
            best, best_d = ref, d
    return best, best_d


def rail_chips(pcb_data, cap: str) -> List[str]:
    """The chips `_elect_tethers` considers for `cap`, in its order: its
    owners (`decap_population`); none for a cap that is no decap.
    Pose-invariant: the rule reads nets, pin types and roles, never a pose."""
    hit = decap_population(pcb_data).get(cap)
    return list(hit[1]) if hit is not None else []


def _point_to_bounds(pt, bounds) -> float:
    """The election's distance: a point to a chip's pad bbox (0 inside)."""
    cx, cy = pt
    x0, y0, x1, y1 = bounds
    return math.hypot(max(x0 - cx, cx - x1, 0.0), max(y0 - cy, cy - y1, 0.0))


def chip_bounds_of(fp):
    """The bounds `_chip_list` gives this ONE footprint at the pose it holds,
    or None when it has no pads. Through `build_chip_list` itself, so a
    caller measuring a footprint the election has not seen (the tether gate,
    at a pose it is only considering) reads the election's margin."""
    from types import SimpleNamespace
    from chip_boundary import build_chip_list
    got = build_chip_list(SimpleNamespace(footprints={
        '_': SimpleNamespace(pads=_pads(fp))}), min_pads=1)
    return got[0].bounds if got else None


def elect_live(cap_fp, candidates) -> Tuple[Optional[str], Optional[float]]:
    """`_elect_tethers`' election for ONE cap with the footprints at whatever
    poses they hold: the cap's pad centroid against `(ref, bounds)` of the
    chips on its rail (`rail_chips`' order, bounds from `chip_bounds_of`),
    nearest wins. The tether gate calls this per candidate pose, so
    it holds the pair the GRADE will elect -- a cap that walks toward a
    different chip on its rail is measured against that chip."""
    return nearest_chip(_centroid(cap_fp), candidates)


def decap_tethers(pcb_data, movable=None,
                  radius: float = DECAP_RADIUS_MM
                  ) -> Dict[str, List[Tuple[str, float]]]:
    """{IC ref: [(cap ref, bbox distance mm)]} -- the decoupling tethers.

    Nearest owner by bounding-box distance: a decap (`decap_population`) and
    the chips with a supply pin on its rail. Proximity alone is not enough -- on ulx3s 13 of 70 caps sit near an IC
    they have no electrical relationship with, and dragging those along would be
    a wrong group.

    A filter over `_elect_tethers`, the one election the decap grade
    (`decap_populations`) also reads, so the block and the grade never
    disagree about which IC a cap belongs to. `_from_decap` builds the
    `decap` blocks from it (`--group-by decap`, on the CLI and the GUI).

    `movable` restricts which caps are considered (None = every footprint), as
    in `derive_groups`. Iteration is over a SORTED ref list: `movable` reaches
    us as a set, and cap order within an IC's list would otherwise vary with
    PYTHONHASHSEED.
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
    * `beyond`  [(cap, ic, d)] cap-sorted -- elected, then pruned by the radius;
                `rule_decap_ungraded` reports them.
    * `orphans` [cap] -- in scope, but no owner has pads to measure. There is no
                IC for them to be far from, so the grader ignores them.

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
