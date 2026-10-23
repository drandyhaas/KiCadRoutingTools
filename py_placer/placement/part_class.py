"""What determines this part's position? A pose-INDEPENDENT class.

The class question is pose-independent -- "what determines where this part
belongs" is a property of what the part IS (footprint, pin functions, ref
convention), never of where it currently sits. Pose enters only through
`pose_plausible`, a separate verdict.

Two edge classes with deliberately different strength:

- ``edge_receptacle`` -- mates THROUGH the board edge (USB, HDMI, RJ45,
  D-sub, card slots, jacks). An interior pose is mechanically implausible:
  the plug could not reach it.
- ``edge_actuator`` -- a switch/button whose actuator MAY overhang the
  edge by design (tigard SW1). Many switches are legitimately interior
  (tactile buttons), so this class makes NO implausibility claim; its value
  is that an OBSERVED overhang is by-design, not a defect.

Generic connector keywords (jst, molex, pinheader, header, socket,
terminal) deliberately do NOT map to an edge class: a JST wire-to-board
connector (tigard J7) is legitimately interior, and classifying it edge
would misplace it.

Thresholds derive from the board or the part, never from any one board:
``seat_tol`` defaults to 0.5 mm and callers should pass
``max(0.5, 2 * grid_step)``; overhang bands come from the part's own
extents.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# ---------------------------------------------------------------------------
# The tables (single source of truth -- lock_advisor imports these).
# ---------------------------------------------------------------------------

# Lexical: substrings of footprint_name.lower(). Stock KiCad library naming;
# house libraries miss, which is why classification also reads pin functions.
CONNECTOR_FP = ('connector', 'usb', 'hdmi', 'rj45', 'jack', 'pinheader',
                'pinsocket', 'terminal', 'dsub', 'molex', 'jst', 'hirose',
                'microsd', 'sim', 'card', 'banana', 'header', 'socket')
#: Marker parts by footprint-name SUBSTRING. Mounting holes are NOT here:
#: a substring also matched KiCad's horizontal D-subs
#: (`..._MountingHolesOffset...`, #1176), so a hole is decided by
#: `is_mount_hole_name` (the item name) and nowhere else.
MOUNT_FP = ('fiducial', 'testpoint')

#: `MountingHole` anywhere in the item, except as the plural `MountingHoles`
#: -- the word a part that merely HAS holes uses (a D-sub's
#: `MountingHolesOffset`, a module's `WithMountingHoles`).
_MOUNT_HOLE_MID = re.compile(r'mountinghole(?!s)')


def is_mount_hole_name(name) -> bool:
    """True when a footprint's ITEM name says it is a mounting hole (#1176).

    The item name STARTS with `MountingHole`, in any library
    (`MountingHole:`, `Mounting_Holes:`, `Glasgow:`, a house library), or
    carries the singular `MountingHole` later on (`M3_MountingHole_Pad`). The
    plural does not count: KiCad's horizontal D-subs carry the hole offset in
    their name (`DSUB-9_Female_Horizontal_..._Housed_
    MountingHolesOffset9.12mm`), its modules and encoders say
    `WithMountingHoles` / `CircularMountingHoles`, and a substring test read
    them all as holes, so a D-sub's edge claim was never emitted. A hole
    named otherwise (`JLCPCB:AssemblyToolingHole`, `Plain_Hole_3mm`) is
    NPTH-only and classifies `mount_hole` through `classify_part`'s
    structural rule. The item, not `lib:item`, because the text parser keeps
    the library prefix and the pcbnew path does not -- a rule on the whole
    string would answer differently on the two fronts. So a house library
    NAMED MountingHole whose item is not (`MountingHole:2.5mm`) is not a hole
    by name; an NPTH-only one is still caught by the structural rule."""
    item = str(name or '').rsplit(':', 1)[-1].lower()
    return (item.startswith('mountinghole')
            or _MOUNT_HOLE_MID.search(item) is not None)

# The edge-mating subset of CONNECTOR_FP, plus receptacle-only names. A part
# matching these mates through the board edge / an enclosure aperture.
EDGE_RECEPTACLE_FP = ('usb', 'hdmi', 'rj45', 'rj11', 'dsub', 'd-sub',
                      'microsd', 'sd_card', 'sdcard', 'sim', 'card',
                      'jack', 'displayport', 'cardedge', 'card_edge')
# Switches/buttons whose actuator may overhang the edge by design.
EDGE_ACTUATOR_FP = ('switch', 'slide', 'sw_', 'button', 'tact', 'pushbutton')
#: Stems too short to test as a bare substring, matched only with no letter on
#: either side (#1331). A DC power-input jack named for what it is
#: (`DC-IN-SMD_DC-045A-13A`, `DC_IN`) is a receptacle, and the `dc_in` inside
#: `Converter_DCDC_Intel_QFN-24` is not. The hyphen spelling of the `sw_` stem
#: (`SW-TH_TK-6580A8-1`, `SW-SMD_TS-1010A-B`) is an actuator, and Samtec's
#: `TSW-114` pin header is not: both bare substrings hit those on the corpus.
EDGE_RECEPTACLE_WORDS = re.compile(r'(?<![a-z])dc[-_]?in(?![a-z])')
EDGE_ACTUATOR_WORDS = re.compile(r'(?<![a-z])sw-')

# Reference-designator prefixes. IEEE-315 convention, NOT a property of the
# file -- prefix evidence alone never makes a receptacle.
CONN_PREFIX_MED = ('J', 'P', 'X', 'CN')
CONN_PREFIX_LOW = ('SW', 'TP', 'MH', 'H', 'FID')
#: Read on the leading letters AND on the designator after a qualifier
#: (`_designator`): `PWR_SW1` is a switch (#1331).
ACTUATOR_PREFIX = ('SW',)

# USB/edge pin functions KiCad writes only when the symbol carried one.
# CC1/CC2/SBU/SHIELD are RECEPTACLE pins specifically (a plug's mechanical
# side); D+/D-/VBUS also live on SILICON -- the corpus census measured a
# QFN-76 USB PHY, a Cypress QFN-56, two VSSOP-10 muxes and a SOT-666 ESD
# array all classifying as receptacles through them -- so classification
# uses only the mechanical set. (The lock advisor's broader IFACE_PINS
# advisory keeps D+/D-/VBUS: near-a-connector is still lock-relevant.)
IFACE_PINS = {'VBUS', 'D+', 'D-', 'CC1', 'CC2', 'SBU1', 'SBU2', 'SHIELD'}
RECEPTACLE_PINS = {'CC1', 'CC2', 'SBU1', 'SBU2', 'SHIELD'}
# IC package families: a footprint whose name says chip package is never an
# edge receptacle, whatever its pin functions say.
IC_PACKAGE_FP = ('qfn', 'dfn', 'sot', 'soic', 'ssop', 'tssop', 'vssop',
                 'msop', 'qfp', 'bga', 'csp', 'wlp', 'lga', 'to-', 'sod',
                 'dip-', 'pga')

# Connector families the substring tables above miss, read as WORDS of the
# footprint ITEM (`lib:item` -> item). The item, because the pcbnew front
# has no library name: a token that matched `Connector_Coaxial:` classed
# `SMA_..._EdgeMount` a connector on the CLI and nothing in the GUI. Over
# the 478 corpus human boards these were the real edge connectors that
# claimed no edge (BNC, FAKRA, edge-mount SMA, card-edge fingers, SFP cages,
# FFC, WAGO). Two strengths, as for the tables above:
#
# Mates THROUGH the edge -> `edge_receptacle`: an SFP cage (its host
# connector sits at the cage's far end, 3-16 mm inboard on the corpus, so it
# is only a connector); the `PJ-` jack series (barrel and 3.5 mm); TRS/TRRS;
# modular RJ jacks.
EDGE_RECEPTACLE_RE = re.compile(
    r'(?<![a-z])[qo]?sfp.*cage|(?<![a-z])pj-?\d|(?<![a-z])trr?s(?![a-z])'
    r'|(?<![a-z])rj-?\d')
# Coaxial RF. `sma` is also the DO-214AC diode package (`D_SMA`,
# `DIODE-SMA`, LCSC `SMA_L4.3-W2.6-LS5.1`), so it needs connector evidence
# or an edge-mount word and never reads a diode package; the other families
# name nothing else. Edge-mount, end-launch, horizontal or right-angle ->
# `edge_receptacle`; vertical and board-level (U.FL) -> `connector_affinity`.
RF_COAX_RE = re.compile(r'(?<![a-z0-9])(?:rp-?)?(?:bnc|fakra|mmcx|mcx'
                        r'|u[.-]?fl|w\.fl|ipex|mhf\d*[a-z]?)(?![a-z])')
RF_SMA_RE = re.compile(r'(?<![a-z0-9])(?:rp-?)?sma(?![a-z])')
RF_EDGE_MOUNT_RE = re.compile(r'edge|launch|horizontal|right.?angle'
                              r'|(?<![a-z])(?:ang|ra)(?![a-z])')
DIODE_PACKAGE_RE = re.compile(r'^d[_-]|diode|do-?214')
CONNECTOR_WORD_RE = re.compile(r'(?<![a-z])conn(?![a-z])|connector|card'
                               r'|cartridge')
# An item that says EDGE (`PCB-Edge_PCIexpress_x1`, `cartridge_edge_72`) or
# carries castellated edge pins, beside CONNECTOR evidence (a CONN_PREFIX_MED
# reference or a connector word) -> `connector_affinity`, not a receptacle:
# the word names a card's fingers and the slot that takes them alike (the
# fingers are read off their pads, `is_card_fingers`), and without the
# evidence it names a spacer (`m2_hex_spacer_passthrough_edge`, H) and
# passives with castellated terminals (`LED_0603_1608Metric_Castellated`).
EDGE_NAMED_RE = re.compile(r'edge|castellat')
# Wire-to-board and board-to-board -> `connector_affinity` (no edge claim by
# itself): `Conn_*` / `CONN-*`, FFC/FPC, WAGO / Phoenix / terminal blocks,
# JST part numbers (`SM04B-SRSS-TB`, `B2B-XH-A`), XT30/60/90, Tag-Connect,
# IDC, PMOD, DIMM and M.2 sockets, PCIe slots, SFP host connectors,
# shrouded headers, solder wires.
CONNECTOR_ITEM_RE = re.compile(
    r'(?<![a-z])conn(?![a-z])|(?<![a-z])(?:ffc|fpc)(?![a-z])'
    r'|(?<![a-z])wago|termblock|phoenix'
    r'|(?<![a-z0-9])(?:s|b|sm|bm)\d{1,2}b-[a-z]'
    r'|(?<![a-z])xt(?:30|60|90)|tag-?connect|(?<![a-z])tc20[357]0'
    r'|(?<![a-z])idc(?![a-z])|(?<![a-z])pmod|dimm(?![a-z])'
    r'|(?<![a-z])pci-?e|pciexpress|(?<![a-z0-9])m\.2(?![0-9]).*(?:key|socket)'
    r'|(?<![a-z])[qo]?sfp|shroud|(?<![a-z])(?:solder)?wire(?![a-z])')


def _item_connector_class(item: str, pref: str) -> Optional[str]:
    """`edge_receptacle`, `connector_affinity` or None from the item-word
    families above; `item` is the lower-case footprint item."""
    # IC packages never; `sod` (SOD-123) also spells SO-DIMM, whose sockets
    # and card edges are connectors.
    if any(k in item.replace('sodimm', '') for k in IC_PACKAGE_FP):
        return None
    conn = pref in CONN_PREFIX_MED or bool(CONNECTOR_WORD_RE.search(item))
    edge_mount = bool(RF_EDGE_MOUNT_RE.search(item))
    rf = (not DIODE_PACKAGE_RE.search(item)
          and (RF_COAX_RE.search(item) is not None
               or (RF_SMA_RE.search(item) is not None
                   and (conn or edge_mount))))
    if EDGE_RECEPTACLE_RE.search(item) or (rf and edge_mount):
        return 'edge_receptacle'
    if (rf or CONNECTOR_ITEM_RE.search(item)
            or (conn and EDGE_NAMED_RE.search(item))):
        return 'connector_affinity'
    return None

_PREFIX_RE = re.compile(r'^([A-Za-z]+)')

SEAT_TOL_MM = 0.5      # default; callers pass max(0.5, 2*grid_step)

#: Past this distance from every edge, a `connector_affinity` part is flagged
#: INTERIOR -- by the intent grade (advisory) and the review sheet, the SAME
#: constant so the two never disagree. 3.0mm is a cable-clearance scale, not
#: a board-derived number: within it a plug can usually still reach over the
#: edge parts.
INTERIOR_AFFINITY_MM = 3.0


@dataclass(frozen=True)
class PartClass:
    name: Optional[str]            # edge_receptacle | edge_actuator |
    #                                mount_hole | fiducial | testpoint | None
    confidence: Optional[str]      # high | medium | low | None
    evidence: Tuple[str, ...] = ()


def _designator(ref: str) -> str:
    """The letters before a reference's number once a qualifier is set
    aside: `SW` in `PWR_SW1` (#1331), where `_prefix` reads `PWR`. Read for
    the actuator prefix only: `TP_GND1` stays a test point by its leading
    `TP`, which this would read as `GND`."""
    return _prefix(re.split(r'[_\-.]', ref or '')[-1])


def _prefix(ref: str) -> str:
    m = _PREFIX_RE.match(ref or '')
    return (m.group(1) if m else '').upper()


def is_card_fingers(fp) -> bool:
    """Whether a part is card-edge fingers: made only of `connect` pads,
    KiCad's EDGE CONNECTOR pad type, four or more -- the fingers of a card
    that plugs in at this board's edge, whatever it is called (`pcie_x1`,
    `Zorro3`, `A2000_CPUSLOT_MALE`). Four, because a fiducial is one or
    three."""
    from kicad_parser import non_aperture_pads
    pads = non_aperture_pads(fp)
    return len(pads) >= 4 and all(
        getattr(p, 'pad_type', '') == 'connect' for p in pads)


def classify_part(fp, ref: str) -> PartClass:
    """Pose-independent classification from footprint name, ref prefix and
    pin functions. Never reads a coordinate."""
    name = (getattr(fp, 'footprint_name', '') or '').lower()
    pref = _prefix(ref)
    # Aperture-only pads (paste/mask windows) are not pads (#1143): a
    # mounting hole with a paste ring is still NPTH-only.
    from kicad_parser import non_aperture_pads
    pads = non_aperture_pads(fp)

    # Structural: NPTH-only part = mounting hole (the strongest rule in the
    # advisor, restated here as a class).
    npth = [p for p in pads if getattr(p, 'pad_type', '') == 'np_thru_hole']
    plated_or_smd = [p for p in pads
                     if getattr(p, 'pad_type', '') != 'np_thru_hole']
    if npth and not plated_or_smd:
        return PartClass('mount_hole', 'high', ('npth-only pad set',))
    if is_mount_hole_name(name):
        return PartClass('mount_hole', 'medium', ('footprint name',))
    if 'fiducial' in name or pref == 'FID':
        return PartClass('fiducial', 'medium', ('footprint name/prefix',))
    if 'testpoint' in name or pref == 'TP':
        return PartClass('testpoint', 'medium', ('footprint name/prefix',))

    pins = {str(getattr(p, 'pinfunction', '') or '').upper() for p in pads}
    recep_pins = pins & RECEPTACLE_PINS
    is_ic_pkg = any(k in name for k in IC_PACKAGE_FP)
    item_cls = ('edge_receptacle' if is_card_fingers(fp) else
                _item_connector_class(name.rsplit(':', 1)[-1], pref))
    fp_recep = (((any(k in name for k in EDGE_RECEPTACLE_FP)
                  or EDGE_RECEPTACLE_WORDS.search(name) is not None)
                 and not is_ic_pkg)
                or item_cls == 'edge_receptacle')
    fp_act = (any(k in name for k in EDGE_ACTUATOR_FP)
              or EDGE_ACTUATOR_WORDS.search(name) is not None)
    desig = _designator(ref)

    if ((fp_recep or recep_pins) and not is_ic_pkg) \
            or item_cls == 'edge_receptacle':
        ev = []
        if fp_recep:
            ev.append('footprint name matches the edge-receptacle table')
        if recep_pins:
            ev.append('receptacle pin functions '
                      f"({', '.join(sorted(recep_pins))[:40]})")
        # Two independent channels (or the receptacle-specific CC/SBU pins)
        # = high; a name alone = medium. Prefix alone NEVER makes a
        # receptacle, and neither do D+/D-/VBUS (silicon carries them too).
        strong = (fp_recep and (recep_pins or pref in CONN_PREFIX_MED)) or \
                 bool(recep_pins & {'CC1', 'CC2', 'SBU1', 'SBU2'})
        return PartClass('edge_receptacle', 'high' if strong else 'medium',
                         tuple(ev))
    if fp_act or pref in ACTUATOR_PREFIX or desig in ACTUATOR_PREFIX:
        ev = ['footprint name matches the edge-actuator table' if fp_act
              else f"reference prefix {pref!r} (convention only)"
              if pref in ACTUATOR_PREFIX
              else f"reference designator {desig!r} in {ref!r} "
                   f"(convention only)"]
        return PartClass('edge_actuator', 'medium' if fp_act else 'low',
                         tuple(ev))
    # The WEAK connector class. Generic connectors (headers, JST, molex,
    # terminal blocks) make NO edge claim -- a JST wire-to-board part is
    # legitimately interior (the guard above stands) -- but they are still
    # where a human plugs something in, and a part with no class is invisible
    # to every intent rule. `connector_affinity` exists to be DECLARED and
    # graded at ADVISORY severity: a mid-board pose is a flag for the
    # boundary review, never an error, and pose_plausible makes no claim
    # for it.
    if (any(k in name for k in CONNECTOR_FP) and not is_ic_pkg) \
            or item_cls == 'connector_affinity':
        return PartClass('connector_affinity', 'low',
                         ('connector-family footprint name; no edge claim',))
    return PartClass(None, None)


# Part classes whose POSITION is a mechanical fact rather than a placement
# decision. `testpoint` is deliberately absent: a test point is placed to suit
# the layout, not the enclosure.
MECHANICAL_CLASSES = ('mount_hole', 'fiducial', 'edge_receptacle',
                      'edge_actuator')


def mechanical_parts(pcb_data) -> Dict[str, Dict]:
    """The parts whose position a real new board would already know.

    Lives here, beside `classify_part`, because it is the ONLY pose-derived
    answer in the toolchain that survives an unplaced board: `classify_part`
    reads footprint name, reference prefix and pin function and never a
    coordinate, so a pile classifies exactly as the placed board does. Every
    other channel that could name the mechanically-fixed parts is geometric
    and collapses -- `lock_advisor`'s high-confidence list measured 7 refs on
    a placed watchy and 1 on the same board piled, and 10 -> 0 on ulx3s,
    because its two-channel promotion needs a geometric rule to fire.

    Callers project from this: the brief wants the class and its evidence.

    `at` is REPORTED, never asserted. On a pile it is the pile coordinate,
    which is the honest answer -- the caller decides what that is worth.
    """
    out: Dict[str, Dict] = {}
    for ref, fp in sorted((getattr(pcb_data, 'footprints', None) or {}).items()):
        cls = classify_part(fp, ref)
        name = getattr(cls, 'name', None) if cls else None
        locked = bool(getattr(fp, 'locked', False))
        if name in MECHANICAL_CLASSES:
            why = (f"{name} (confidence {getattr(cls, 'confidence', '?')}): "
                   f"its position is a mechanical fact a real new board would "
                   f"already know")
        elif locked:
            why = ("(locked yes) in the source: the board's author pinned it, "
                   "and a run that may not move it must know where it is")
        else:
            continue
        out[ref] = {
            'class': name,
            'confidence': getattr(cls, 'confidence', None) if cls else None,
            'evidence': list(getattr(cls, 'evidence', ()) or ()),
            'locked_in_source': locked,
            'reason': why,
            'at': [round(fp.x, 6), round(fp.y, 6),
                   round(fp.rotation or 0.0, 6)],
        }
    return out


def pose_plausible(cls: Optional[str], outside_amount: Optional[float],
                   edge_clearance: Optional[float],
                   seat_tol: float = SEAT_TOL_MM) -> Optional[bool]:
    """Is the part's CURRENT pose consistent with its class?

    Only ``edge_receptacle`` makes a claim: it must overhang the outline or
    sit within ``seat_tol`` of an edge (the mating face has to reach the
    edge). Every other class returns True; unknown geometry returns None.

    Run-3 calibration for the test suite, not for the rule: displaced J1
    measured outside_amount 0.0 / edge_clearance 2.45 -- implausible; SW1
    measured outside_amount 1.6 -- plausible. A bare "far from every edge"
    threshold would MISS J1 (a swap can park an edge part near a different
    edge), which is why the conjunct is class-conditional.
    """
    if cls != 'edge_receptacle':
        return True
    if outside_amount is None or edge_clearance is None:
        return None
    return outside_amount > 1e-6 or edge_clearance <= seat_tol


def default_band(cls: str, fp, observed_overhang: float = 0.0
                 ) -> Dict[str, float]:
    """Class-default overhang band {min, max} in mm, from the part's own
    geometry -- never from board history."""
    if cls == 'edge_actuator':
        mx = (observed_overhang + 0.5) if observed_overhang > 1e-6 else 3.0
        return {'min': 0.0, 'max': round(mx, 3)}
    # receptacle: the shell face may protrude a little past the pad field;
    # bound by half the part's SMALLER extent (the depth axis) capped at 2.
    # Aperture-only pads are not part of the pad field (#1143).
    from kicad_parser import non_aperture_pads
    pads = non_aperture_pads(fp)
    if pads:
        xs = [getattr(p, 'global_x', 0.0) for p in pads]
        ys = [getattr(p, 'global_y', 0.0) for p in pads]
        half_min_extent = 0.5 * min(
            (max(xs) - min(xs)) if len(xs) > 1 else 2.0,
            (max(ys) - min(ys)) if len(ys) > 1 else 2.0)
        mx = min(2.0, max(0.5, half_min_extent))
    else:
        mx = 2.0
    return {'min': 0.0, 'max': round(max(mx, observed_overhang + 0.25), 3)}
