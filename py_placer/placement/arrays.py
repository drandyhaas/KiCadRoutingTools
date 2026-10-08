"""Arrays of identical parts: the formation predicate and the pin order (#1051).

A declared `arrays[]` entry says "these parts are one row": resistor banks along
a bus in the served IC's pin order, a decap row under a chip, one rotation for
the lot. The seeder has no term that builds such a row and the quench pulls
each member toward its own nets, so identical parts scatter (run 32:
3750 crossings against the human layout's 1352).

This module holds the two PURE pieces every consumer shares:

* `formation` -- ONE predicate for "is this set of poses a formed row": a
  common axis, the expected order along it, one shared rotation, an even
  pitch. The grade rule `array_formation` calls it, and the seeder's row seat
  and the quench's block move are to call it too (#1051 phases 3-4) rather
  than each carrying a copy, which is how a grader and an engine come to
  disagree about what "formed" means.
* `pin_order` -- the order a row must follow for `order: "pin"`: the served
  IC's pad numbering, read off pads and nets only. POSE-BLIND on purpose: an
  order read off where the parts sit would bless whatever arrangement the
  board already has, the emit-then-grade round trip this toolchain refuses
  everywhere else.

* `suggest_arrays` -- the DETECTOR (#1051 phase 2). It suggests only, with
  evidence, and reaches an intent through `emit_intent(derive_arrays='auto')`
  or a reader accepting a row; it reads no pose.
"""
from __future__ import annotations

import math
import re
from typing import Dict, List, Optional, Sequence, Tuple

#: The row's tolerances. Constants rather than intent keys: the schema
#: declares WHAT a row is (axis, order, rotation, pitch) and these say how
#: close to exact the measurement must be, which is a property of the
#: measurement (grid and float noise), not of the design.
#:
#: `axis_mm`: how far a member's centre may sit off the row's line.
#: `rotation_deg`: how far a member may be turned from the row's rotation.
#: `pitch_spread_mm`: max minus min of the consecutive gaps, `pitch_mm: auto`.
#: `pitch_mm`: how far one gap may be from a DECLARED pitch.
#:
#: Checked against the HUMAN boards in phase 2 and left as they were: over
#: the detector's rows on glasgow_revC, ulx3s and splitflap (courtyard
#: centres, as `array_formation` grades), the largest axis offset of a row
#: the human formed is 0.236mm (ulx3s R40/R54) and the smallest of a row
#: that fails on its axis is 0.5715mm (ulx3s R22/R23; glasgow's is 0.65mm,
#: R52/R53), so 0.25 sits in the gap between them. The one near-miss pitch,
#: ulx3s C38-C45 at 0.381mm spread, is a deliberate 1.778mm gap in a
#: 1.397mm row, not float noise. Rotation failures on those boards are 90
#: or 180 degree splits, never a fraction of a degree
#: (tests/test_1051_suggest_arrays.py).
#:
#: Since the phase-1 re-verification a <= 2 pad part compares its rotation
#: modulo 180 (`SYMMETRIC_MAX_PADS`): the 180 degree splits were human rows
#: that route. Re-measured with it on the same instrument: glasgow 16 of its
#: 27 detected rows form (15 before; the remaining rotation failures, J1:5k1,
#: U1:9p, U1:100k, U30:2k2, U30:47R, U30:+1V2:u1 and bank:100R, are all 90
#: or 45 degree splits and every one also fails its axis), ulx3s 15 of 32,
#: splitflap 5 of 10. After phase-2 fix round 1 (connectors, jumpers and
#: peer-IC chains are no longer members; supply rails are no own net): glasgow
#: 14 of 24 (the two lost formed rows were socket pairs J6+J7 and J8+J9),
#: ulx3s 14 of 30, splitflap 0 of 4 (all five formed rows were chip chains
#: or headers), coldfire 9 of 14, watchy 1 of 5. After fix round 2 (a
#: supply-pin net is a rail only at RAIL_MIN_PARTS; digit-led non-voltage
#: names are no rail): coldfire 6 of 14 (the MAX202 cap rows grew from 2 to
#: 4 caps and no longer form), watchy 2 of 6 (U4:18pF, the crystal pair).
DEFAULT_TOLERANCES = {'axis_mm': 0.25, 'rotation_deg': 0.5,
                      'pitch_spread_mm': 0.25, 'pitch_mm': 0.25}

#: The enums, shared with `floorplan`'s loader so the schema and the
#: predicate cannot disagree about a spelling.
ORDERS = ('pin', 'declared', 'unknown')
ROTATION_WORDS = ('shared', 'unknown')
AXES = ('auto', 'x', 'y')
PITCH_WORDS = ('auto',)


def natural_key(pad_number) -> Tuple:
    """Sort key for pad numbers: '2' < '10', 'A2' < 'A10' < 'B1'.

    Digit runs compare as integers and letter runs as text, each tagged with
    its kind so a digit run never compares against a letter run.
    """
    out = []
    for i, chunk in enumerate(re.split(r'(\d+)', str(pad_number))):
        if not chunk:
            continue
        out.append((0, int(chunk), '') if i % 2 else (1, 0, chunk))
    return tuple(out)


def pin_order(pcb, serves: str, members: Sequence[str]
              ) -> Tuple[List[str], Dict[str, str]]:
    """`(ordered_refs, unresolved)`: members in the served part's PIN order.

    Pose-blind: reads pad numbering and nets, never a footprint position.

    A member's key is the lowest (natural-sorted) pad of `serves` on a net
    that member reaches and no OTHER member reaches. A net several members
    share is a rail (+3V3 on every pull-up of a bank) and says nothing about
    order. Among a member's own nets, one landing on exactly ONE pad of the
    served part is preferred to one landing on several: a GND pull-down's
    ground lands on every ground pin, while its signal lands on one.

    `unresolved` is `{ref: why}` for a member with no such net, or one the
    board does not have; it is ordered by nothing, and a caller must say so
    rather than drop it.
    """
    fps = pcb.footprints or {}
    unresolved: Dict[str, str] = {}
    host = fps.get(serves)
    if host is None:
        return [], {m: f"{serves} is not on this board" for m in members}
    host_pads: Dict[int, List[str]] = {}
    for pd in host.pads:
        if pd.net_id:
            host_pads.setdefault(pd.net_id, []).append(str(pd.pad_number))
    nets_of: Dict[str, set] = {}
    for m in members:
        fp = fps.get(m)
        if fp is None:
            unresolved[m] = 'not on this board'
            continue
        nets_of[m] = {pd.net_id for pd in fp.pads if pd.net_id}
    reach: Dict[int, int] = {}
    for nets in nets_of.values():
        for n in nets:
            reach[n] = reach.get(n, 0) + 1
    keyed = []
    for m, nets in nets_of.items():
        own = [n for n in nets if reach.get(n) == 1 and n in host_pads]
        single = [n for n in own if len(host_pads[n]) == 1]
        use = single or own
        if not use:
            unresolved[m] = (f"no net of its own lands on a pad of {serves}"
                             if any(n in host_pads for n in nets)
                             else f"shares no net with {serves}")
            continue
        pad = min((p for n in use for p in host_pads[n]), key=natural_key)
        keyed.append((natural_key(pad), m))
    keyed.sort()
    return [m for _k, m in keyed], unresolved


def _ang_diff(a: float, b: float, period: float = 360.0) -> float:
    d = abs(float(a) - float(b)) % period
    return min(d, period - d)


#: A part with at most this many copper pads looks the same turned 180
#: degrees as far as routing is concerned -- a resistor, a capacitor, a
#: two-pin diode -- so its row rotation is compared MODULO 180. Humans do
#: this routinely and it routes: glasgow R18/R19, ulx3s C59/C62, D20/D21,
#: D51/D52, C63-C72 (Phase 2 measurement). A part with more pads keeps the
#: exact modulo-360 comparison, because its pin order matters.
SYMMETRIC_MAX_PADS = 2


def rotation_period(pads) -> float:
    """180 for a part with <= `SYMMETRIC_MAX_PADS` copper pads, else 360.
    `pads` None (the caller does not know) is the strict 360."""
    return (180.0 if pads is not None and int(pads) <= SYMMETRIC_MAX_PADS
            else 360.0)


def allowed_angles(angles, pads) -> set:
    """An allowed angle set widened by the part's own symmetry: a two-pad
    part allowed 0 is also allowed 180. The load-time conflict check and
    the grade compare through this and `rotation_period`, so they agree."""
    out = set()
    for a in angles:
        a = float(a) % 360.0
        out.add(a)
        if rotation_period(pads) == 180.0:
            out.add((a + 180.0) % 360.0)
    return out


def formation(members_poses: Sequence[Dict[str, object]], *,
              order_key: Optional[Sequence[str]] = None,
              rotation_spec=None, pitch_spec='auto', axis_spec='auto',
              tolerances: Optional[Dict[str, float]] = None
              ) -> Dict[str, object]:
    """Is this set of member poses ONE formed row? The shared predicate.

    `members_poses`: `[{'ref', 'x', 'y', 'rot', 'pads'?}]`, `x`/`y` the
    member's centre in whatever frame the caller measures (the grade uses
    the courtyard centre, so identical parts compare like for like).
    `pads` is the member's copper pad count: a part with at most
    `SYMMETRIC_MAX_PADS` compares its rotation modulo 180, not 360.

    `order_key`: the expected order of refs along the row, or None when the
    order is not declared (`order: "unknown"`). A row may run either way, so
    the reversed order is formed too. Members absent from `order_key` are
    ordered by nothing and do not count against it.

    `rotation_spec`: a number (every member at that angle), `'shared'` (every
    member at ONE angle, whichever), or `'unknown'`/None (not checked).
    `pitch_spec`: `'auto'` (even gaps) or a number (every gap at it).
    `axis_spec`: `'x'` (a row running along x, so the centres share a y),
    `'y'`, or `'auto'` (the longer extent of the centres).

    Returns `{'formed', 'failed': [check names], 'unchecked': [...],
    'axis', 'order': [refs as they lie], 'checks': {name: detail}}`. A
    check that is not declared is `unchecked`, never passed silently -- the
    difference between "held" and "nobody asked" is the one this toolchain
    keeps everywhere.
    """
    tol = dict(DEFAULT_TOLERANCES)
    tol.update(tolerances or {})
    poses = [dict(p) for p in members_poses]
    checks: Dict[str, Dict[str, object]] = {}
    failed: List[str] = []
    unchecked: List[str] = []
    if len(poses) < 2:
        return {'formed': False, 'failed': ['members'], 'unchecked': [],
                'axis': None, 'order': [p.get('ref') for p in poses],
                'checks': {'members': {'ok': False, 'count': len(poses),
                                       'why': 'a row needs two members'}}}

    xs = [float(p['x']) for p in poses]
    ys = [float(p['y']) for p in poses]
    if axis_spec in ('x', 'y'):
        axis = axis_spec
    else:
        axis = 'x' if (max(xs) - min(xs)) >= (max(ys) - min(ys)) else 'y'
    along = xs if axis == 'x' else ys
    across = ys if axis == 'x' else xs
    mid = sorted(across)[len(across) // 2]
    dev = max(abs(a - mid) for a in across)
    ok = dev <= tol['axis_mm'] + 1e-9
    checks['axis'] = {'ok': ok, 'axis': axis, 'declared': axis_spec,
                      'max_offset_mm': round(dev, 4),
                      'tolerance_mm': tol['axis_mm']}
    if not ok:
        failed.append('axis')

    lie = sorted(range(len(poses)), key=lambda i: (along[i], poses[i]['ref']))
    observed = [poses[i]['ref'] for i in lie]
    want = ([] if order_key is None
            else [r for r in order_key if r in set(observed)])
    if order_key is None:
        unchecked.append('order')
    elif len(want) < 2:
        # Fewer than two members HAVE an expected position: an order over
        # one ref (or none) is satisfied by every arrangement, so it is
        # UNCHECKED, never a pass (Phase-1 verifier: `serves: J3` on
        # splitflap resolved nobody and graded clean).
        unchecked.append('order')
        checks['order'] = {'ok': None, 'expected': want,
                           'why': (f"only {len(want)} member(s) have an "
                                   f"expected position")}
    else:
        seen = [r for r in observed if r in set(want)]
        ok = seen == want or seen == list(reversed(want))
        checks['order'] = {'ok': ok, 'expected': want, 'observed': seen}
        if not ok:
            failed.append('order')

    rots = [float(p.get('rot') or 0.0) % 360.0 for p in poses]
    # Per member: 180 for a symmetric (<= 2 copper pad) part, else 360. A
    # pose that does not say how many pads it has is compared strictly.
    per = [rotation_period(p.get('pads')) for p in poses]
    if isinstance(rotation_spec, (int, float)) and not isinstance(
            rotation_spec, bool):
        want_rot = float(rotation_spec) % 360.0
        off = sorted(p['ref'] for p, r, t in zip(poses, rots, per)
                     if _ang_diff(r, want_rot, t) > tol['rotation_deg'])
        checks['rotation'] = {'ok': not off, 'declared': want_rot,
                              'off': off,
                              'rotations': sorted({round(r, 3)
                                                   for r in rots})}
        if off:
            failed.append('rotation')
    elif rotation_spec == 'shared':
        # Every PAIR, each compared at the stricter of its two periods: two
        # resistors 180 apart share a rotation, a resistor and a 3-pad part
        # 180 apart do not.
        spread = max(_ang_diff(rots[i], rots[j], max(per[i], per[j]))
                     for i in range(len(rots)) for j in range(i + 1,
                                                               len(rots)))
        ok = spread <= tol['rotation_deg']
        checks['rotation'] = {'ok': ok, 'declared': 'shared',
                              'rotations': sorted({round(r, 3)
                                                   for r in rots})}
        if not ok:
            failed.append('rotation')
    else:
        unchecked.append('rotation')

    pos = sorted(along)
    gaps = [b - a for a, b in zip(pos, pos[1:])]
    if isinstance(pitch_spec, (int, float)) and not isinstance(
            pitch_spec, bool):
        bad = [round(g, 4) for g in gaps
               if abs(g - float(pitch_spec)) > tol['pitch_mm'] + 1e-9]
        ok = not bad
        checks['pitch'] = {'ok': ok, 'declared': float(pitch_spec),
                           'gaps_mm': [round(g, 4) for g in gaps],
                           'off_mm': bad, 'tolerance_mm': tol['pitch_mm']}
    else:
        spread = (max(gaps) - min(gaps)) if gaps else 0.0
        # A zero gap is two members on one spot: even, and not a row.
        ok = (spread <= tol['pitch_spread_mm'] + 1e-9
              and min(gaps) > tol['pitch_spread_mm'])
        checks['pitch'] = {'ok': ok, 'declared': 'auto',
                           'gaps_mm': [round(g, 4) for g in gaps],
                           'spread_mm': round(spread, 4),
                           'tolerance_mm': tol['pitch_spread_mm']}
    # Members closer along the axis than the tolerance share one spot.
    stacked = set()
    for i in range(len(lie) - 1):
        a, b = lie[i], lie[i + 1]
        if along[b] - along[a] <= tol['pitch_spread_mm']:
            stacked.update((poses[a]['ref'], poses[b]['ref']))
    checks['pitch']['stacked'] = sorted(stacked)
    if not ok:
        failed.append('pitch')

    return {'formed': not failed, 'failed': failed, 'unchecked': unchecked,
            'axis': axis, 'order': observed, 'checks': checks}


def formation_at_state(state, pcb, spec: Dict[str, object],
                       members: Sequence[str]) -> Dict[str, object]:
    """`formation` over `members` at their CURRENT poses in a placement
    state (`pose_score.make_state` / `QuenchState`), in the grade's own
    measurement: each member's courtyard centre on the grade ladder
    (`part.grade_rect()`, the rect `rule_array_formation` reads -- under
    `body_model` `rect()` is the occupancy, whose centre moved by up to
    3.5 mm on One-Air-Max), its board rotation, its copper pad count.
    `spec` is a resolved array (`floorplan.resolved_arrays`: `order_refs`,
    `rotation`, `pitch_mm`, `axis`). The seeder's row self-check and the
    quench's "is this row formed, so hold it rigid" both call this."""
    poses = []
    for m in members:
        p = state.parts[m]
        r = p.grade_rect() if hasattr(p, 'grade_rect') else p.rect()
        poses.append({'ref': m, 'x': (r[0] + r[2]) / 2.0,
                      'y': (r[1] + r[3]) / 2.0, 'rot': p.rot % 360.0,
                      'pads': _copper_pad_count(pcb.footprints[m])})
    return formation(poses, order_key=spec.get('order_refs'),
                     rotation_spec=spec.get('rotation'),
                     pitch_spec=spec.get('pitch_mm', 'auto'),
                     axis_spec=spec.get('axis', 'auto'))


def spec_of(entry: Dict[str, object]) -> Dict[str, object]:
    """One intent `arrays[]` entry with its absent keys RESOLVED.

    Absent is not "unknown": an absent `order` is reported as absent by the
    brief and never checked; `"unknown"` is the author saying so. Both leave
    the order unchecked, so this collapses them for the PREDICATE only --
    the raw entry keeps the difference for every report.
    """
    rot = entry.get('rotation')
    return {
        'name': str(entry.get('name')),
        'members': [str(m) for m in (entry.get('members') or ())],
        'serves': entry.get('serves'),
        'order': entry.get('order') or 'unknown',
        'rotation': (float(rot) % 360.0
                     if isinstance(rot, (int, float))
                     and not isinstance(rot, bool) else (rot or 'unknown')),
        'pitch_mm': entry.get('pitch_mm', 'auto'),
        'axis': entry.get('axis', 'auto'),
        'allow_mixed': bool(entry.get('allow_mixed', False)),
    }


def expected_order(pcb, spec: Dict[str, object]
                   ) -> Tuple[Optional[List[str]], Dict[str, str]]:
    """`(order_key, unresolved)` for a resolved spec: the pin order for
    `order: "pin"`, the declared list for `"declared"`, None otherwise."""
    if spec['order'] == 'pin':
        return pin_order(pcb, str(spec['serves']), spec['members'])
    if spec['order'] == 'declared':
        return list(spec['members']), {}
    return None, {}


def is_number(v) -> bool:
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(float(v)))


# ---------------------------------------------------------------------------
# The detector: SUGGEST arrays, never declare them (#1051 phase 2)
# ---------------------------------------------------------------------------

#: The smallest candidate the detector reports. TWO, not three, and measured:
#: glasgow_revC's 33R resistor arrays -- the issue's own lead example -- are
#: two `R_Array_Convex_4x0402` per hierarchical channel serving U30 (RN7+RN8
#: on IO_Buffer_A, RN1+RN2 on IO_Buffer_B), and the human board lays each pair
#: out as its own row at a 2.5mm pitch with a 10mm gap between the channels.
#: At three the detector finds none of them. The schema's own floor is two
#: (`_array_entries`: "one part is not an array"), and a suggestion costs the
#: reader one accept-or-decline, so the lower threshold's price is paid in
#: review, not in geometry.
MIN_MEMBERS = 2

#: A served part (`pin_run`'s host) has at least this many COPPER pads: the
#: number `groups.DECAP_MIN_IC_PADS` / `chip_boundary` already call "not a
#: passive". Without it a 2-pad part links two members of a series chain
#: (R24 -> R48 <- R25 on glasgow) and reads as a two-member "run".
HOST_MIN_PADS = 4

#: A host must have at least this many times the copper pads of the row's
#: members: a row is small parts gathered at a BIGGER one, not two peer ICs
#: joined by one net. Measured on the phase-2 boards (host pads / member
#: pads, members above two pads): every chip-to-chip "row" sits at exactly
#: 1.0 -- splitflap's TPL7407L/74HC595 shift-register chain (16 on 16, four
#: rows each resting on one chain net) and glasgow's 8-pad R-arrays hosting
#: 8-pad sockets -- while the smallest real one is 1.43, glasgow's SP3012
#: ESD arrays (14 pads) on connector J3/J2 (20), a row the human formed.
#: 1.25 sits between them; the SN74LVC1T45 buffers (6 on U30's 121) and the
#: R-arrays (8 on 20 or 121) clear it by far.
MIN_HOST_TO_MEMBER_PADS = 1.25

#: Footprints whose name says JUMPER are links, not parts to row up
#: (ulx3s D51/D52 `D_SMA_Jumper_NC`). `part_class.classify_part` has no
#: jumper class -- a 2x4 header used as a jumper field is a
#: `connector_affinity` there already -- so this is the one name test the
#: detector adds, on KiCad's own `Jumper` library naming.
JUMPER_FP = ('jumper',)

#: The criteria in PRECEDENCE order. A part is in at most one candidate; when
#: two criteria would claim it, the earlier one wins, because its evidence is
#: more specific: a named pin of a named part (`pin_run`), then a rail pair
#: on one chip's supply pins (`decap_row`), then a connectivity SHAPE
#: (`sheet_bank`), which names no part at all.
CRITERIA = ('pin_run', 'decap_row', 'sheet_bank')


def _ref_key(ref: str) -> Tuple:
    return natural_key(ref)


def _net_label(pcb, net_id: int) -> str:
    n = (pcb.nets or {}).get(net_id)
    return (getattr(n, 'name', '') or '') if n is not None else ''


#: A leaf that starts with a digit is a rail to `net_queries.
#: is_power_net_name` (its `\d` alternative, meant for `3V3`, `5V`, `1V8`).
#: That also takes `32K_P`, `12MHZ`, `25M_XO`: watchy's 32.768 kHz crystal
#: nets `/32K_P` / `/32K_N` read as rails, so its crystal load caps C17/C18
#: had no own net and their `bank:18pF` vanished; read as signals, they are
#: the pin_run `U4:18pF` on U4's two crystal pins. The shared predicate has
#: five other callers and is NOT changed here; the detector alone reads a
#: digit-led leaf as a rail only when it is VOLTAGE-shaped.
_VOLTAGE_LEAF = re.compile(r'^\d+(?:[.,]\d+)?V\d*[A-Z]*(?:$|[_.\-])', re.I)


def _rail_by_name(name: str) -> bool:
    from net_queries import is_ground_net_name, is_power_net_name
    if not name:
        return False
    if is_ground_net_name(name):
        return True
    if not is_power_net_name(name):
        return False
    leaf = name.split('/')[-1]
    return not leaf[:1].isdigit() or bool(_VOLTAGE_LEAF.match(leaf))


def _is_rail(name: str, net_id: int = 0, rails=frozenset()) -> bool:
    """A ground or rail: by NAME (`_rail_by_name`, over `net_queries`), or
    -- `rails`, `supply_rail_nets` -- by what it FEEDS: a net on a chip's
    supply pin that reaches at least `RAIL_MIN_PARTS` parts. The second
    catches rails with no rail-shaped name.

    NOT traced: a rail that reaches a chip only THROUGH a 0-ohm jumper.
    ulx3s's `/power/P1V1`, `P2V5`, `P3V3` pass no name test and reach U1's
    supply pins only via jumpers RP1-RP3, so neither path sees them, and
    their output caps C22/C23/C24 still form `bank:2.2uF`."""
    return (bool(net_id) and net_id in rails) or _rail_by_name(name)


#: A supply-pin net is a RAIL only when it reaches at least this many parts.
#: A net joining one chip pin to one cap is that chip's LOCAL node -- a
#: charge-pump reservoir, an internal regulator's VCAP -- not a rail other
#: parts share. Measured: coldfire's MAX202s type V+ and V- `power_out`, each
#: net reaching the chip and one cap (C204/C207, C209/C212), so counting
#: them as rails cut the `U202:100nF` / `U203:100nF` rows from 4 caps to
#: the 2 flying caps. Every real rail on the phase-2 boards reaches more.
RAIL_MIN_PARTS = 3


def supply_rail_nets(supply, pcb=None) -> frozenset:
    """Every net on a supply pin of any chip in `supply_pins`' answer that
    reaches at least `RAIL_MIN_PARTS` parts (all of them without `pcb`)."""
    out = set()
    for rec in supply.values():
        for p, _n in rec['pins']:
            if not p.net_id:
                continue
            if pcb is not None:
                net = (pcb.nets or {}).get(p.net_id)
                parts = {q.component_ref
                         for q in (getattr(net, 'pads', None) or ())}
                if len(parts) < RAIL_MIN_PARTS:
                    continue
            out.add(p.net_id)
    return frozenset(out)


def _copper_pad_count(fp) -> int:
    from . import groups as groups_mod
    return groups_mod._copper_pads(fp)


def pose_free_chip_refs(pcb) -> set:
    """`groups.chip_refs`, asked of each part in its OWN frame.

    `chip_refs` rejects a single row of pads (a header, a castellated edge)
    through `_pads_are_collinear`, which reads GLOBAL pad coordinates -- and
    a row turned to a non-right angle shares neither an x nor a y, so the
    answer moves with the pose. The detector must answer the same on every
    pose, so the same predicate is asked of the pads' LOCAL coordinates: the
    footprint as drawn, before the board placed it.
    """
    from types import SimpleNamespace
    from . import groups as groups_mod
    out = set()
    from kicad_parser import non_aperture_pads
    for ref, fp in (pcb.footprints or {}).items():
        # the pins, not the paste/mask apertures (#1143): the stand-ins
        # below carry no layers, so `_pads_are_collinear` cannot tell
        pads = non_aperture_pads(fp)
        if len(pads) < groups_mod.DECAP_MIN_IC_PADS:
            continue
        if groups_mod._copper_pads(fp) < groups_mod.DECAP_MIN_IC_PADS:
            continue
        local = SimpleNamespace(pads=[
            SimpleNamespace(global_x=float(p.local_x or 0.0),
                            global_y=float(p.local_y or 0.0)) for p in pads])
        if groups_mod._pads_are_collinear(local):
            continue
        out.add(ref)
    return out


def not_a_member(fp, ref: str) -> Optional[str]:
    """Why a part can never be an array MEMBER, or None.

    `part_class.classify_part` -- the repo's pose-independent classifier --
    decides: a connector (`connector_affinity`, `edge_receptacle`), a
    switch or button (`edge_actuator`), a test point, a mounting hole or a
    fiducial is placed where a cable, a finger, a probe or a screw needs
    it, not in a row by its electrical neighbours. Measured before this:
    ulx3s U1:CONN_02X20 and the PTS645 buttons, coldfire's single-pad
    CONN_1 test points, CONN_4X2 jumper fields and DB9s, splitflap's motor
    and sensor headers. Such a part may still be a HOST (glasgow's J2/J3
    serve their R-array rows). Plus `JUMPER_FP`.
    """
    from .part_class import classify_part
    cls = classify_part(fp, ref).name
    if cls is not None:
        return f"part_class classifies it {cls}"
    name = str(getattr(fp, 'footprint_name', '') or '').lower()
    if any(k in name for k in JUMPER_FP):
        return 'a jumper footprint'
    return None


def _partitions(pcb, excluded=None) -> Dict[Tuple[str, str, str], List[str]]:
    """{(footprint, value, sheet): [refs]} -- the parts that could be ONE row.

    Same footprint AND same value (the issue's "identical parts"), and the
    same schematic sheet. The sheet is part of the key because a repeated
    hierarchical sheet is a CHANNEL, and a channel's parts are laid out
    together: glasgow's 33R arrays split IO_Buffer_A / IO_Buffer_B exactly
    where the human rows split, its 17 SN74LVC1T45 split 8 / 8 / 1 where the
    human grids do (the 1 is U32, a different function on the top sheet),
    and ulx3s's 549R resistors split out the eight LED resistors of the
    `blinkey` sheet, the one row the human drew. Parts with no net-bearing
    pad are not candidates, nor is any part `not_a_member` names; those are
    collected into `excluded` ({partition key: (why, [refs])}) when given.
    """
    from . import groups as groups_mod
    out: Dict[Tuple[str, str, str], List[str]] = {}
    for ref, fp in (pcb.footprints or {}).items():
        if ref.startswith('#') or not any(p.net_id for p in fp.pads or ()):
            continue
        key = (str(fp.footprint_name or ''), str(getattr(fp, 'value', '')
                                                  or ''),
               groups_mod._sheet_of(fp))
        why = not_a_member(fp, ref)
        if why is not None:
            if excluded is not None:
                excluded.setdefault(key, (why, []))[1].append(ref)
            continue
        out.setdefault(key, []).append(ref)
    for refs in out.values():
        refs.sort(key=_ref_key)
    if excluded is not None:
        for _why, refs in excluded.values():
            refs.sort(key=_ref_key)
    return out


def _own_signal_links(pcb, members: Sequence[str], rails=frozenset()
                      ) -> Dict[str, Dict[str, List[Dict[str, str]]]]:
    """{member: {host: [link]}} over each member's OWN signal nets.

    A net is the member's own when no other member reaches it (a rail every
    pull-up shares says nothing about which pin a member serves), and a
    SIGNAL when it is neither ground nor rail-named: ulx3s R36's only net
    not shared with another 549R is GND, which lands on every ground ball of
    U1 and would otherwise make it a "member" of U1's LED bank.
    """
    fps = pcb.footprints or {}
    mset = set(members)
    reach: Dict[int, int] = {}
    for m in members:
        for n in {p.net_id for p in fps[m].pads if p.net_id}:
            reach[n] = reach.get(n, 0) + 1
    out: Dict[str, Dict[str, List[Dict[str, str]]]] = {}
    for m in members:
        links: Dict[str, List[Dict[str, str]]] = {}
        for p in sorted(fps[m].pads, key=lambda q: natural_key(q.pad_number)):
            if not p.net_id or reach.get(p.net_id) != 1:
                continue
            name = _net_label(pcb, p.net_id)
            if _is_rail(name, p.net_id, rails):
                continue
            net = (pcb.nets or {}).get(p.net_id)
            for q in (getattr(net, 'pads', None) or ()):
                h = q.component_ref
                if h == m or h in mset or h not in fps:
                    continue
                links.setdefault(h, []).append({
                    'net': name, 'pad': str(q.pad_number),
                    'member_pad': str(p.pad_number)})
        out[m] = links
    return out


#: Why a grid-named host's run is suggested with `order: "unknown"`.
GRID_ORDER_NOTE = (
    "the host's pads are grid names (A1, B2 ...), whose sort order is not a "
    "position along any row; members are listed in that order, but no order "
    "is suggested")

_GRID_PAD = re.compile(r'^[A-Za-z]+[0-9]+$')


def _grid_named(fp) -> bool:
    """True when most of a part's pads are ball-grid names (`A1`, `AB12`).

    Measured, and why `pin_run` does not suggest `order: "pin"` on such a
    host: ulx3s R41-R48, eight LED resistors each on a ball of U1, are a
    human row that passes every `array_formation` check EXCEPT order --
    `natural_key` sorts U1's balls B2, C1, C2, D1, D2, E1, E2, H3 while the
    row follows LED0..LED7 -- and ulx3s's GPDI caps C38-C45 fail the same
    way. A grid name is a coordinate on the package's underside, not a
    count along one side of it, so its sort order says nothing about which
    member sits next to which.
    """
    pads = [str(p.pad_number) for p in (fp.pads or ())]
    return bool(pads) and 2 * sum(bool(_GRID_PAD.match(n))
                                  for n in pads) > len(pads)


def _bridge_partner(fps, links, host: str, members: Sequence[str],
                    floor: float) -> Optional[str]:
    """The second big part a pin run BRIDGES to, or None.

    A bridge is a run whose EVERY member also reaches, by an own signal net
    (`links`), some part other than `host` with at least `floor` copper
    pads -- the same floor a host must clear. Returned is the part the most
    members reach (ties: more pads, then the reference), for the reason
    text. One member without such a partner means the run is not a bridge:
    a pull-up bank's far side is a rail and has none.

    Only ACTIVE members (more than `SYMMETRIC_MAX_PADS` copper pads) bridge.
    A two-pad passive in series between a chip and a connector is the
    classic row a human DOES lay as a row -- ulx3s's GPDI coupling caps
    C38-C45 are one (the tolerance note above measures it) -- and the
    literal rule declined it, with glasgow's RN5+RN6 and ulx3s's audio
    dividers."""
    if any(_copper_pad_count(fps[m]) <= SYMMETRIC_MAX_PADS for m in members):
        return None
    count: Dict[str, int] = {}
    for m in members:
        ml = links.get(m, {})
        # On a DIFFERENT net from the member's link to the host: a crystal
        # on the same MCU pin as a load cap is that pin's net, not a far
        # side (glasgow U1:9p would otherwise "bridge U1 and Y1").
        host_nets = {lk['net'] for lk in ml.get(host, ())}
        far = {h for h, ls in ml.items()
               if h != host and _copper_pad_count(fps[h]) >= floor
               and any(lk['net'] not in host_nets for lk in ls)}
        if not far:
            return None
        for h in far:
            count[h] = count.get(h, 0) + 1
    # `h` last: `_ref_key` ties 'U1' and 'U01', and `count` is filled from
    # the SET `far`, so the key must be total or the winner is hash order.
    return min(count, key=lambda h: (-count[h], -_copper_pad_count(fps[h]),
                                     _ref_key(h), h))


def _pin_runs(pcb, refs: List[str], rails=frozenset(),
              declined: Optional[List[Dict]] = None
              ) -> Tuple[List[Dict], List[str]]:
    """Criterion (a): members each on their own pin of ONE common part.

    Repeatedly elects the host that the most remaining members reach by an
    own signal net (ties: more single-pad links, more copper pads, then the
    reference), orders them with `pin_order` -- the grader's own order, so a
    suggestion and its grade cannot disagree about it -- and keeps what that
    order resolves. The rest try the next host (glasgow IO_Buffer_A: RN7+RN8
    serve U30, then RN9+RN10 serve connector J2).

    A host under the pad-ratio floor (`MIN_HOST_TO_MEMBER_PADS`) that
    WOULD HAVE WON an election -- two or more members, and ahead of the
    host that was elected, or with none elected -- is recorded in
    `declined`, not dropped silently (splitflap: U4 over the shift
    registers, U1/U6/U7/U8 over each other). A floored host that would
    have lost anyway is not a decision the floor made, and is not listed.
    """
    fps = pcb.footprints or {}
    remaining = list(refs)
    floored: Dict[str, List[str]] = {}
    mp = _copper_pad_count(fps[refs[0]]) if refs else 0
    found: List[Dict] = []
    # One partition is one footprint, so one member pad count.
    floor = max(HOST_MIN_PADS,
                MIN_HOST_TO_MEMBER_PADS * _copper_pad_count(fps[refs[0]])
                if refs else 0)
    while len(remaining) >= MIN_MEMBERS:
        links = _own_signal_links(pcb, remaining, rails)
        score: Dict[str, List[int]] = {}
        for m in remaining:
            for h, ls in links[m].items():
                if _copper_pad_count(fps[h]) < HOST_MIN_PADS:
                    continue
                s = score.setdefault(h, [0, 0])
                s[0] += 1
                s[1] += len(ls)

        def elect(pool):
            return min(pool, key=lambda h: (-score[h][0], -score[h][1],
                                            -_copper_pad_count(fps[h]),
                                            _ref_key(h), h)) if pool else None
        host = elect([h for h in score
                      if _copper_pad_count(fps[h]) >= floor])
        best = elect(list(score))
        if (best is not None and best != host
                and score[best][0] >= MIN_MEMBERS and best not in floored):
            floored[best] = [m for m in remaining if best in links[m]]
        if host is None or score[host][0] < MIN_MEMBERS:
            break
        linked = [m for m in remaining if host in links[m]]
        ordered, _unres = pin_order(pcb, host, linked)
        if len(ordered) < MIN_MEMBERS:
            break
        claimed = set(ordered)
        # A BRIDGE: every member also reaches, by an own signal net, a
        # second part at or above the host floor. That is a buffer/driver
        # bank between two big parts (glasgow's 17 SN74LVC1T45 between U30
        # and the IO side), which humans lay as multi-row blocks near the
        # far part -- a shape `arrays[]` cannot express (one row per
        # entry; multi-row blocks are not supported). Seeding it as one
        # row at the host measured WORSE on glasgow (phase-3 verifier:
        # crossings and hpwl both rose), so it is declined with the
        # reason, and a reader may still declare it by hand.
        other = _bridge_partner(fps, links, host, ordered, floor)
        if other is not None:
            if declined is not None:
                declined.append({
                    'members': list(ordered), 'serves': host,
                    'criterion': 'pin_run',
                    'why': f"bridges {host} and {other}"})
            remaining = [m for m in remaining if m not in claimed]
            continue
        grid = _grid_named(fps[host])
        found.append({
            'serves': host, 'members': ordered,
            'order': 'unknown' if grid else 'pin',
            'evidence': {
                'host_footprint': str(fps[host].footprint_name or ''),
                'host_pads': 'grid' if grid else 'numbered',
                'order_basis': (GRID_ORDER_NOTE if grid else
                                'the lowest pad of the host each member '
                                'reaches by its own signal net'),
                'links': {m: links[m][host] for m in ordered},
            }})
        claimed = set(ordered)
        remaining = [m for m in remaining if m not in claimed]
    if declined is not None:
        for h in sorted(floored, key=_ref_key):
            if len(floored[h]) >= MIN_MEMBERS:
                hp = _copper_pad_count(fps[h])
                declined.append({
                    'members': list(floored[h]), 'serves': h,
                    'criterion': 'pin_run',
                    'why': (f"host {h} has {hp} copper pads, under "
                            f"{MIN_HOST_TO_MEMBER_PADS}x the members' {mp}: "
                            f"peer parts on one net, not small parts "
                            f"gathered at a bigger one")})
    return found, remaining


def _decap_rows(pcb, refs: List[str], supply) -> Tuple[List[Dict], List[str]]:
    """Criterion (b): caps bridging ONE rail pair that lands on the supply
    pins of exactly ONE chip.

    Pose-blind by construction, which is why it is not `groups.decap_tethers`:
    the tether election picks the NEAREST chip carrying the rail, so its
    answer is a fact about where the parts sit. Here a rail that feeds two
    chips' supply pins is ambiguous and produces no row -- the detector does
    not guess which chip a shared +3V3 cap is for. `supply` is
    `floorplan.supply_pins` over the pose-free chip set.
    """
    from net_queries import is_ground_net_name
    from . import groups as groups_mod
    fps = pcb.footprints or {}
    rail_chips: Dict[int, List[str]] = {}
    for chip, rec in supply.items():
        for p, _name in rec['pins']:
            # A chip whose only pins on the rail are `power_out` SOURCES it
            # (the regulator); it is not what the rail's caps decouple, and
            # counting it made every regulated rail "shared by two chips".
            toks = (getattr(p, 'pintype', '') or '').split('+')
            if 'power_out' in toks and 'power_in' not in toks:
                continue
            rail_chips.setdefault(p.net_id, [])
            if chip not in rail_chips[p.net_id]:
                rail_chips[p.net_id].append(chip)
    by_pair: Dict[Tuple[int, int], List[str]] = {}
    for m in refs:
        if not groups_mod.is_decoupling_cap(fps[m], m):
            continue
        nets = sorted({p.net_id for p in fps[m].pads if p.net_id > 0})
        gnd = [n for n in nets if is_ground_net_name(_net_label(pcb, n))]
        rail = [n for n in nets if n not in gnd]
        if len(gnd) != 1 or len(rail) != 1:
            continue
        by_pair.setdefault((rail[0], gnd[0]), []).append(m)
    found: List[Dict] = []
    claimed = set()
    for (rail, gnd), caps in sorted(by_pair.items(),
                                    key=lambda kv: _ref_key(kv[1][0])):
        chips = sorted(rail_chips.get(rail, ()), key=_ref_key)
        if len(caps) < MIN_MEMBERS or len(chips) != 1:
            continue
        ic = chips[0]
        rec = supply[ic]
        found.append({
            'serves': ic, 'members': list(caps), 'order': 'unknown',
            'evidence': {
                'rail': _net_label(pcb, rail), 'ground': _net_label(pcb, gnd),
                'supply_pads': sorted((str(p.pad_number)
                                       for p, _n in rec['pins']
                                       if p.net_id == rail), key=natural_key),
                'supply_channel': rec['channel'],
            }})
        claimed.update(caps)
    return found, [m for m in refs if m not in claimed]


def _sheet_banks(pcb, refs: List[str], rails=frozenset()) -> List[Dict]:
    """Criterion (c): identical parts on one sheet with PARALLEL connectivity.

    Each member's SHAPE is, pad by pad: a net every member shares (by name),
    or a net of its own (by what it reaches: the footprint, value and pad of
    the parts on it). A neighbour EVERY member reaches is recorded without
    its pad, so eight LEDs each on its own pin of one driver still match.
    Members with one shape and at least one own net are a bank: an LED+
    resistor channel repeated, not parallel caps on one rail (no own net).

    A RAIL (`_is_rail`) is never an own net, the way `pin_run` filters
    rails: three caps each on their own rail plus GND (`+1V8`, `+3V3`,
    `+5V`) share a shape only because each rail is "its own", and that is
    three regulators' output caps, not a channel repeated. A rail is
    recorded by its NAME, so such members differ and form nothing. A rail
    `_is_rail` cannot see still counts as own: ulx3s C22/C23/C24 on
    P1V1/P2V5/P3V3 (reaching U1 only through 0-ohm jumpers) DO form
    `bank:2.2uF`.
    """
    fps = pcb.footprints or {}
    reach: Dict[int, int] = {}
    for m in refs:
        for n in {p.net_id for p in fps[m].pads if p.net_id}:
            reach[n] = reach.get(n, 0) + 1

    def _own(net_id) -> bool:
        return (bool(net_id) and reach.get(net_id) == 1
                and not _is_rail(_net_label(pcb, net_id), net_id, rails))

    neigh: Dict[str, set] = {}
    for m in refs:
        s = set()
        for p in fps[m].pads:
            if _own(p.net_id):
                net = (pcb.nets or {}).get(p.net_id)
                s.update(q.component_ref for q in
                         (getattr(net, 'pads', None) or ())
                         if q.component_ref != m)
        neigh[m] = s
    common = set.intersection(*neigh.values()) if neigh else set()
    shapes: Dict[Tuple, List[str]] = {}
    for m in refs:
        shape = []
        own = 0
        for p in sorted(fps[m].pads, key=lambda q: natural_key(q.pad_number)):
            if not p.net_id:
                shape.append((str(p.pad_number), 'none', ()))
                continue
            if _own(p.net_id):
                own += 1
                net = (pcb.nets or {}).get(p.net_id)
                sig = []
                for q in (getattr(net, 'pads', None) or ()):
                    h = q.component_ref
                    if h == m or h not in fps:
                        continue
                    sig.append((str(fps[h].footprint_name or ''),
                                str(getattr(fps[h], 'value', '') or ''),
                                '*' if h in common else str(q.pad_number)))
                shape.append((str(p.pad_number), 'own', tuple(sorted(sig))))
            else:
                shape.append((str(p.pad_number), 'shared',
                              (_net_label(pcb, p.net_id),)))
        if own:
            shapes.setdefault(tuple(shape), []).append(m)
    found = []
    for shape, members in sorted(shapes.items(),
                                 key=lambda kv: _ref_key(kv[1][0])):
        if len(members) < MIN_MEMBERS:
            continue
        found.append({
            'serves': None, 'members': list(members), 'order': 'unknown',
            'evidence': {
                'shape': [{'pad': pd, 'kind': kind,
                           'reaches' if kind == 'own' else 'net':
                           ([list(x) for x in what] if kind == 'own'
                            else (what[0] if what else None))}
                          for pd, kind, what in shape],
                'own_nets': {m: sorted(_net_label(pcb, p.net_id)
                                       for p in fps[m].pads
                                       if _own(p.net_id))
                             for m in members},
            }})
    return found


def _one_role_per_part(pcb, cands: List[Dict],
                       declined: Optional[List[Dict]]) -> List[Dict]:
    """The candidates that survive "no part is both a host and a member".

    GREEDY over a fixed priority, and only a SURVIVOR refuses: a candidate
    is accepted unless its host is a member of an accepted one, or one of
    its members hosts an accepted one. The earlier draft refused every
    pair in one pass, so a candidate refused only by an already-refused
    one stayed refused: at a pad ratio of 1.0 splitflap's real `U5:220R`
    and `U9:220R` rows went, because U5/U9 were members of chip "rows"
    that were themselves refused (fix-round-1 re-verification).

    Priority: `CRITERIA` precedence (coldfire's `bank:MAX202` loses to the
    cap rows its members host), then the BIGGER host first -- a row is
    small parts gathered at a large one, so glasgow-style R-arrays on an
    FPGA outrank anything the R-array itself might "host" -- then the
    smaller members, then the larger row, then the first member's name.
    """
    fps = pcb.footprints or {}

    def key(c):
        host = (_copper_pad_count(fps[c['serves']])
                if c['serves'] in fps else 0)
        return (CRITERIA.index(c['criterion']), -host,
                _copper_pad_count(fps[c['members'][0]]),
                -len(c['members']), _ref_key(c['members'][0]))

    accepted: List[Dict] = []
    member_of: Dict[str, Dict] = {}
    host_of: Dict[str, Dict] = {}
    for c in sorted(cands, key=key):
        why = None
        if c['serves'] and c['serves'] in member_of:
            o = member_of[c['serves']]
            why = (f"its host {c['serves']} is a member of the kept "
                   f"{o['criterion']} ({', '.join(o['members'])})")
        else:
            for m in c['members']:
                if m in host_of:
                    o = host_of[m]
                    why = (f"member {m} is the host of the kept "
                           f"{o['criterion']} ({', '.join(o['members'])})")
                    break
        if why is not None:
            if declined is not None:
                declined.append({'members': list(c['members']),
                                 'serves': c['serves'],
                                 'criterion': c['criterion'],
                                 'why': why + '; a part is a host or a '
                                              'member, not both'})
            continue
        accepted.append(c)
        for m in c['members']:
            member_of[m] = c
        if c['serves']:
            host_of[c['serves']] = c
    return accepted


def suggest_arrays(pcb, *, pin_functions=None,
                   declined: Optional[List[Dict]] = None
                   ) -> List[Dict[str, object]]:
    """Candidate `arrays[]` entries for a reader to ACCEPT or DECLINE (#1051).

    Each candidate is `{name, members (ordered), serves, order,
    rotation: 'shared', pitch_mm: 'auto', axis: 'auto', criterion, evidence,
    row}` -- the intent's own shape plus WHY; `row` is the paste-ready
    `arrays[]` entry (`suggestion_row`), with no `criterion`/`evidence`. It suggests only: nothing reaches an
    intent except through `emit_intent(derive_arrays='auto')` or a reader
    copying it, and an absent row is not a claim that there is none.

    POSE-BLIND, and a test holds it to that: it reads footprints, values,
    sheets, pad numbering and nets, never a position or a rotation (the
    chip set is asked in each part's own frame, `pose_free_chip_refs`). A
    detector that read poses would suggest whatever arrangement the board
    already has, and an intent emitted from the HUMAN board would then leak
    the human's layout into the arm it is meant to be judged against. The
    guarantee is THIS function's: `emit_intent(derive_arrays='auto')` then
    drops members through the emitter's pose-inferred zones and releases
    its pose-inferred edge claims (`floorplan._derived_arrays`), so the rows
    it writes carry a little of the board's current arrangement, disclosed
    by member.

    Criteria, in precedence order (`CRITERIA`), within each (footprint,
    value, sheet) partition (`_partitions`): `pin_run` (order `'pin'`, or
    `'unknown'` on a ball-grid host -- `_grid_named`),
    `decap_row` (order `'unknown'`: caps on one rail pair are
    interchangeable, so no order is theirs to follow) and `sheet_bank`
    (order `'unknown'`). Output order: criterion, then larger first, then
    name -- deterministic, independent of dict or hash order.

    No part is both a HOST and a MEMBER (`_one_role_per_part`). `declined`,
    when a list is given, receives one record per refused candidate, per
    host the pad-ratio floor rejected (`MIN_HOST_TO_MEMBER_PADS`), and per
    partition of excluded parts (`not_a_member`) that could otherwise have
    formed a row.
    """
    from .floorplan import supply_pins
    supply = supply_pins(pcb, pin_functions=pin_functions,
                         chips=pose_free_chip_refs(pcb))
    rails = supply_rail_nets(supply, pcb)
    raw: List[Dict] = []
    excluded: Dict[Tuple[str, str, str], Tuple[str, List[str]]] = {}
    parts = _partitions(pcb, excluded)
    if declined is not None:
        for key, (why, refs) in sorted(excluded.items()):
            if len(refs) >= MIN_MEMBERS:
                declined.append({'members': list(refs), 'value': key[1],
                                 'why': f"{why}: not an array member"})
    for (fpname, value, sheet), refs in sorted(parts.items()):
        if len(refs) < MIN_MEMBERS:
            continue
        runs, rest = _pin_runs(pcb, refs, rails, declined)
        rows, rest = _decap_rows(pcb, rest, supply)
        banks = (_sheet_banks(pcb, rest, rails)
                 if len(rest) >= MIN_MEMBERS else [])
        for crit, cands in (('pin_run', runs), ('decap_row', rows),
                            ('sheet_bank', banks)):
            for c in cands:
                c['criterion'] = crit
                c['evidence'].update({'footprint': fpname, 'value': value,
                                      'sheet': sheet or '/'})
                raw.append(c)
    raw = _one_role_per_part(pcb, raw, declined)
    out: List[Dict[str, object]] = []
    used: Dict[str, int] = {}
    raw.sort(key=lambda c: (CRITERIA.index(c['criterion']),
                            -len(c['members']), _ref_key(c['members'][0])))
    for c in raw:
        ev = c['evidence']
        if c['criterion'] == 'decap_row':
            base = f"{c['serves']}:{ev['rail'].split('/')[-1]}:{ev['value']}"
        elif c['serves']:
            base = f"{c['serves']}:{ev['value'] or ev['footprint']}"
        else:
            base = f"bank:{ev['value'] or ev['footprint']}"
        used[base] = used.get(base, 0) + 1
        name = base if used[base] == 1 else f"{base}~{used[base]}"
        sug = {
            'name': name, 'members': list(c['members']),
            'serves': c['serves'], 'order': c['order'],
            'rotation': 'shared', 'pitch_mm': 'auto', 'axis': 'auto',
            'criterion': c['criterion'], 'evidence': ev}
        sug['row'] = suggestion_row(sug)
        out.append(sug)
    return out


def suggestion_row(c: Dict[str, object]) -> Dict[str, object]:
    """The paste-ready `arrays[]` entry for one suggestion: exactly the
    intent's keys, with `why` pre-filled from the criterion. `criterion`
    and `evidence` stay beside it on the suggestion -- the intent loader
    refuses both as unknown keys, so a reader who copied the whole
    suggestion got an exit 2 (Phase-5 fact-check). A reader accepting it
    copies this and writes their own reason into `why`."""
    row: Dict[str, object] = {'name': c['name'],
                              'members': list(c['members'])}
    if c.get('serves'):
        row['serves'] = c['serves']
    row.update({'order': c['order'], 'rotation': c['rotation'],
                'pitch_mm': c['pitch_mm'], 'axis': c['axis'],
                'why': suggestion_why(c)})
    return row


def suggestion_why(c: Dict[str, object]) -> str:
    """The one-line reason an emitted `arrays[]` entry carries (`why`),
    naming the criterion; the evidence bulk stays with the detector."""
    ev = c.get('evidence') or {}
    crit = c.get('criterion')
    if crit == 'pin_run':
        return (f"detector pin_run: {len(c['members'])}x {ev.get('value')} "
                f"each on its own pin of {c['serves']}"
                + (", in its pin order" if c.get('order') == 'pin' else
                   "; its pads are grid names, so no order is suggested"))
    if crit == 'decap_row':
        return (f"detector decap_row: {len(c['members'])}x "
                f"{ev.get('value')} on {ev.get('rail')}/{ev.get('ground')}, "
                f"a rail only {c['serves']}'s supply pins carry")
    return (f"detector sheet_bank: {len(c['members'])}x {ev.get('value')} "
            f"on sheet {ev.get('sheet')} with one connectivity shape")
