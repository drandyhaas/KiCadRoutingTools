"""Grade a recovered placement against the original it was perturbed from (#411).

The corpus has no bad seeds: the stress harness strips tracks, vias and pours
but keeps the placement, so every board arrives with a floorplan a human
iterated until it routed and shipped. #411's proposal is to manufacture bad
seeds by perturbing that placement, and to grade recovery against the original
-- free ground truth, exactly the way the original routing already is.

This module is the grader. One rule governs it:

    ONE SCORING FUNCTION, THREE BOARDS. `score_board()` is called identically
    on the control, the perturbed and the recovered board, each freshly parsed
    from a file. Nothing is scored from a mutated in-memory state.

A metric bug then becomes common-mode and cancels in the deltas; a metric that
can only be computed on one arm is not admitted; and the perturber cannot drift
from the grader, because the perturber calls this to fill its own record.

Two design decisions worth stating, both of which are easy to get wrong.

**Displacement is measured in PAD space, not between footprint origins.** Under
a pure translation every pad moves by |t|, so a 20 mm dose reads 20.0 mm --
dimensionally honest. Under a rotation in place it reads
2*RMS(pad radius)*|sin(theta/2)|, a real distance rather than an invented
penalty. Measured by THIS function on interf_u_unrouted, rotated in place, with
the footprint-origin distance 0.00 mm in every row:

    ref    pads   centre   pad@180   pad@90
    U5       32     0.00     49.64    35.10
    BUS1     62     0.00     45.44    32.13
    U9      120     0.00     30.55    21.60
    C2        2     0.00      3.54     2.50

That is this repo's own "the position diff showed 0.00 mm while a decap
requirement went 2.04 -> 9.57 mm", quantified by the metric instead of narrated
around it. The C2 row is the index-matching property: a symmetric two-pad
passive flipped 180 degrees reads 2*pad_offset, not 0.

The pad set is **every** pad on the footprint, including padstacks with no net.
This is a rigid-body geometry question -- how far did this part's copper move --
so it must not depend on which pads happen to carry a net. (Restricting it to
net-bearing pads shifts U5 to 49.6 -> 50.4 and U9 to 30.6; the ranking and every
conclusion are unchanged, but the number would then move whenever a netlist did.)

`portfolio.pose_distance_mm` must NOT be reused here, and the reason is not
style: it adds a flat unitless 1.0 into a sum of mm^2 when rotations differ, so
it is footprint-size-blind and would score U5's 50 mm reality as +1.0. It is
right for its own job -- "are these two candidates the same placement?" -- and
wrong for this one.

**Recovery is a RATIO of the applied dose, and it can go negative.** A 5 mm
dose recovered to 4 mm is 20% recovered, not "4 mm good". Unbounded below on
purpose: a grader that can only report improvement is not a grader.
"""
from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

from kicad_parser import local_to_global, parse_kicad_pcb

Pose = Tuple[float, float, float]

# A part is "home" when it is within this of its original pose, in the same
# pad-space units as everything else here. Not a tuned constant: it is the
# tolerance below which a placement difference cannot matter to a router at any
# grid this toolchain uses (the finest documented --grid-step is 0.025 mm).
HOME_TOLERANCE_MM = 2.0


# --------------------------------------------------------------------- poses

def board_poses(pcb_data) -> Dict[str, Pose]:
    """{ref: (x, y, rotation)} for every footprint that has pads.

    Padless footprints (logos, fiducial art) are excluded deliberately: they
    carry no net, so no metric here can see them, and including them would let
    a board's silkscreen dilute its displacement RMS.
    """
    return {ref: (fp.x, fp.y, (fp.rotation or 0.0) % 360.0)
            for ref, fp in (pcb_data.footprints or {}).items() if fp.pads}


def part_displacement(fp, pose_a: Pose, pose_b: Pose) -> float:
    """RMS distance moved by `fp`'s pads between two poses, in mm.

    Pads are matched **by index** through `fp.pads`, which is what makes a 180
    degree flip of a symmetric two-pad passive read 2*pad_offset rather than 0.
    That is correct rather than pedantic: the two nets have swapped ends, and
    the airwires with them.
    """
    pads = getattr(fp, 'pads', None) or ()
    if not pads:
        return 0.0
    ax, ay, arot = pose_a
    bx, by, brot = pose_b
    acc = 0.0
    for p in pads:
        gx1, gy1 = local_to_global(ax, ay, arot, p.local_x, p.local_y)
        gx2, gy2 = local_to_global(bx, by, brot, p.local_x, p.local_y)
        acc += (gx1 - gx2) ** 2 + (gy1 - gy2) ** 2
    return math.sqrt(acc / len(pads))


def _rms(values: Sequence[float]) -> float:
    vals = list(values)
    if not vals:
        return 0.0
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def displacement_to_original(poses_now: Dict[str, Pose],
                             poses_orig: Dict[str, Pose],
                             footprints,
                             subset: Optional[Sequence[str]] = None) -> Dict:
    """How far this placement sits from the original, in pad space.

    `subset` is the FROZEN perturbed member list. Aggregating over every part
    instead would dilute a 20 mm move of 21 parts on a 160-part board to ~7 mm,
    which makes the metric's sensitivity a function of board size and so
    uncomparable across boards. The three companions are reported beside it:

      perturbed_pad_rms   the headline -- over `subset`
      perturbed_pad_max   one member left behind
      collateral_pad_rms  recovery that shoved the REST of the board aside
      all_pad_rms         every part, for comparability with other tooling
      perturbed_centre_rms  footprint origins only, printed BESIDE the pad
                            number so the 0.00-vs-50.44 gap is visible in the
                            output and not only in this docstring
      rot_mismatch        members whose rotation differs from the original
    """
    members = set(subset or ())
    per_part: Dict[str, float] = {}
    centre: Dict[str, float] = {}
    rot_mismatch: List[str] = []
    for ref, pose in sorted(poses_now.items()):
        orig = poses_orig.get(ref)
        fp = (footprints or {}).get(ref)
        if orig is None or fp is None:
            continue
        per_part[ref] = part_displacement(fp, pose, orig)
        centre[ref] = math.hypot(pose[0] - orig[0], pose[1] - orig[1])
        if abs((pose[2] - orig[2]) % 360.0) > 1e-3:
            rot_mismatch.append(ref)

    in_set = [v for r, v in per_part.items() if r in members]
    out_set = [v for r, v in per_part.items() if r not in members]
    home = [r for r in members
            if per_part.get(r, float('inf')) <= HOME_TOLERANCE_MM]
    # A SINGLE threshold saturates and then carries no signal. Run 9 measured
    # `parts_home_frac` = 0.0 for the damaged board AND for the recovered one,
    # while `recovery` moved +0.045 -- so the headline metric could not
    # distinguish "moved nothing" from "moved 80 parts most of the way", which
    # is the distinction the experiment exists to make. The curve is pure
    # derivation from `per_part`, which is already computed above.
    _in = sorted(in_set)
    def _frac(t):
        return round(sum(1 for v in _in if v <= t) / len(_in), 6) if _in else None
    # The ladder runs well past HOME_TOLERANCE_MM on purpose. Measured on run 9:
    # a ladder stopping at 20mm was ALSO all-zero on both boards, because the
    # median displacement was 28.6mm -- i.e. a curve can saturate exactly like
    # the single threshold it replaces, just more verbosely. `per_part_median`
    # below is the scalar that cannot saturate, and it is the one that showed
    # the movement there (28.580 -> 27.277mm). Keep the ladder FIXED rather than
    # deriving it per board: an adaptive ladder is not comparable between two
    # boards, which is the whole point of reporting it.
    home_curve = {t: _frac(t) for t in (0.5, 1.0, HOME_TOLERANCE_MM,
                                        5.0, 10.0, 20.0, 50.0, 100.0)}
    if _in:
        _m = len(_in) // 2
        median = round(_in[_m] if len(_in) % 2 else (_in[_m - 1] + _in[_m]) / 2.0, 6)
    else:
        median = None
    return {
        'home_curve': home_curve,
        'per_part_median': median,
        'perturbed_pad_rms': round(_rms(in_set), 6),
        'perturbed_pad_max': round(max(in_set), 6) if in_set else 0.0,
        'perturbed_centre_rms': round(_rms([centre[r] for r in members
                                            if r in centre]), 6),
        'collateral_pad_rms': round(_rms(out_set), 6),
        'all_pad_rms': round(_rms(list(per_part.values())), 6),
        'parts_home_frac': round(len(home) / len(members), 6) if members else None,
        'rot_mismatch': sorted(r for r in rot_mismatch if r in members),
        'rot_mismatch_total': len(rot_mismatch),
        'members_scored': len(in_set),
        'per_part': {r: round(v, 6) for r, v in per_part.items() if r in members},
    }


def _gyration(poses: Dict[str, Pose], subset: Sequence[str]) -> Optional[float]:
    """RMS distance of the unit's members from their own centroid, in mm."""
    pts = [poses[r][:2] for r in subset if r in poses]
    if len(pts) < 2:
        return None
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return math.sqrt(sum((p[0] - cx) ** 2 + (p[1] - cy) ** 2
                         for p in pts) / len(pts))


def unit_spread_ratio(poses_now: Dict[str, Pose], poses_orig: Dict[str, Pose],
                      subset: Sequence[str]) -> Optional[float]:
    """Radius of gyration of the unit now, over what it was originally.

    The tear detector. With `--group-by none` a router-in-the-loop can drag a
    block's passives home while its IC stays put, shredding the block; a
    centroid-only recovery ratio reads that as partial success. Above ~1.5 the
    unit did not move, it came apart.

    **Its sensitivity depends on the unit being spatially coherent to begin
    with, and callers must be able to see that.** Measured while writing the
    test: displacing 3 of 6 members by 40 mm moved this ratio only 1.00 -> 1.04
    when the six were scattered across the whole board, because their original
    gyration was already ~50 mm and a tear cannot make a scattered set much
    more scattered. On a real block -- which is compact, that being what makes
    it a block -- the same tear reads far above 1.5. So `score_board` reports
    `unit_gyration_orig_mm` beside the ratio: a ratio near 1.0 against a LARGE
    original gyration means "this metric cannot tell", not "not torn".
    """
    a, b = _gyration(poses_now, subset), _gyration(poses_orig, subset)
    if a is None or b is None or b < 1e-9:
        return None
    return round(a / b, 6)


# ------------------------------------------------------------------ recovery

def recovery_fraction(d0: float, d1: float) -> Optional[float]:
    """1.0 perfect, 0.0 inert, NEGATIVE worse than the perturbed board.

    `None` when the dose was zero: scoring a perfect recovery for a no-op
    experiment is exactly how a dose-0 row poisons an aggregate mean. The
    caller reports that as `status: degenerate_dose`, never as 1.0 and never
    as 0.0.
    """
    if d0 is None or d1 is None or abs(d0) < 1e-9:
        return None
    return round((d0 - d1) / d0, 6)


def recovery_radius(rows: Sequence[Dict], threshold: float = 0.5
                    ) -> Optional[float]:
    """Largest applied dose D such that EVERY dose <= D reached `threshold`.

    Monotone from below on purpose: one lucky cell at 40 mm must not be able to
    claim a 40 mm radius when 20 mm failed. `None` when even the smallest dose
    fails -- which is a result, and the one #411 predicts.
    """
    graded = sorted(
        ((float(r['dose_mm_applied']), r.get('recovery_fraction')) for r in rows
         if r.get('dose_mm_applied') and r.get('recovery_fraction') is not None),
        key=lambda t: t[0])
    radius = None
    for dose, frac in graded:
        if frac < threshold:
            break
        radius = dose
    return radius


# ------------------------------------------------------- routed-half tallies

def frozen_net_ids(pcb_data, ignore_net_ids: Sequence[int] = ()) -> List[int]:
    """Every net with >= 2 pads on the board this is called with.

    Called ONCE on the original and stored in the record, because this is the
    denominator. Pads and nets are topological, so the list is arm-invariant --
    but computing it per arm would still be wrong the day a tool drops a net.
    """
    ignored = set(ignore_net_ids or ())
    out = []
    for nid, pads in (getattr(pcb_data, 'pads_by_net', None) or {}).items():
        if nid and nid not in ignored and len(pads) >= 2:
            out.append(nid)
    return sorted(out)


def dirty_net_ids(pcb_routed, drc_json_path: str) -> List[int]:
    """Net ids carrying a DRC violation, from `check_drc.py --json`.

    The missing half of `connectivity_tally`'s contract. `dirty_nets` has never
    once been supplied by any caller, so `PRR`/`NRR` -- the DRC-CLEAN forms,
    which are the ones OmniRouting actually defines -- have always come back
    `None`, and only the `_connected` forms carried a number. This is the whole
    producer: check_drc writes `items[].net1` / `net2` as net NAMES, and the
    parsed board maps names to ids.

    Names, not ids, is deliberate on check_drc's side and is why this cannot be
    a one-liner at the call site: an id is meaningless without the net table of
    the exact board it came from, and a report outlives the board it graded.
    An unresolvable name is DROPPED rather than guessed -- silently widening
    the dirty set would depress PRR on nets that are clean.
    """
    import json as _json
    try:
        with open(drc_json_path, encoding='utf-8') as fh:
            doc = _json.load(fh)
    except Exception:                                          # noqa: BLE001
        return []
    by_name = {n.name: nid for nid, n in (pcb_routed.nets or {}).items()
               if n.name}
    out = set()
    for item in (doc.get('items') or ()):
        for key in ('net1', 'net2'):
            nid = by_name.get(item.get(key))
            if nid is not None:
                out.add(nid)
    return sorted(out)


def connectivity_tally(pcb_routed, net_ids: Sequence[int],
                       prr_denominator: Optional[int] = None,
                       rr_denominator: Optional[int] = None,
                       dirty_nets: Optional[Sequence[int]] = None,
                       tolerance: float = 0.02) -> Dict:
    """PRR / NRR over a FROZEN denominator, computed here rather than read.

    Two things this deliberately does not do.

    It does not read `route.py`'s `pad_pairs_total`. That key is a per-RUN
    denominator: it is filled only from the nets the router actually scoped,
    emitted only when nonzero, and wrapped in a bare except. A net the router
    never touched is simply absent from it -- so OmniRouting's "missing outputs
    remain in both routability denominators" cannot be satisfied by reading it.

    It does not call `check_connected.run_connectivity_check`, whose net
    selection skips nets with no copper at all -- exactly the nets that must
    stay in the denominator.

    `dirty_nets` are the net ids carrying a DRC violation. OmniRouting's PRR is
    *"the percentage of required independent DRC-clean pad-to-pad connections"*
    and NRR *"the percentage of required nets that are connected and free of
    published design-rule violations"* -- connectivity ALONE would score a
    disconnected-but-clean board well, which is the confusion CLAUDE.md warns
    about ("connectivity is orthogonal to DRC"). When `dirty_nets` is None the
    DRC half did not run, and the clean columns say so rather than reporting a
    number nothing checked.
    """
    from check_connected import (check_net_connectivity,
                                 net_break_within_outlines,
                                 net_pad_pairs_within_outlines)
    segs = list(getattr(pcb_routed, 'segments', None) or [])
    vias = list(getattr(pcb_routed, 'vias', None) or [])
    zones = list(getattr(pcb_routed, 'zones', None) or [])
    by_net = getattr(pcb_routed, 'pads_by_net', None) or {}

    dirty = set(dirty_nets) if dirty_nets is not None else None
    pairs_total = pairs_conn = 0
    pairs_conn_clean = 0
    nets_ok = nets_ok_clean = 0
    nets_seen = 0
    open_nets: List[int] = []

    for nid in net_ids:
        pads = list(by_net.get(nid, ()))
        if len(pads) < 2:
            continue
        nets_seen += 1
        res = check_net_connectivity(nid, segs, vias, pads, zones,
                                     tolerance=tolerance, pcb_data=pcb_routed)
        pt, pc = net_pad_pairs_within_outlines(pcb_routed, res, pads)
        broken, _ = net_break_within_outlines(pcb_routed, res)
        pairs_total += pt
        pairs_conn += pc
        if not broken:
            nets_ok += 1
        else:
            open_nets.append(nid)
        if dirty is not None and nid not in dirty:
            pairs_conn_clean += pc
            if not broken:
                nets_ok_clean += 1

    pt_den = prr_denominator if prr_denominator is not None else pairs_total
    rr_den = rr_denominator if rr_denominator is not None else nets_seen

    def _pct(num, den):
        return round(100.0 * num / den, 4) if den else None

    out = {
        'ran': True,
        'prr_denominator': pt_den,
        'rr_denominator': rr_den,
        'pairs_connected': pairs_conn,
        'nets_connected': nets_ok,
        'nets_open': len(open_nets),
        # Connectivity-only. Kept because the gap between this and the
        # DRC-clean number is itself informative.
        'PRR_connected': _pct(pairs_conn, pt_den),
        'RR_connected': _pct(nets_ok, rr_den),
        'Open': _pct(len(open_nets), rr_den),
    }
    if dirty is None:
        out.update({'PRR': None, 'NRR': None, 'drc_clean_ran': False,
                    'drc_clean_reason': 'no dirty_nets supplied; the DRC half '
                                        'did not run'})
    else:
        out.update({'PRR': _pct(pairs_conn_clean, pt_den),
                    'NRR': _pct(nets_ok_clean, rr_den),
                    'drc_clean_ran': True,
                    'dirty_nets': len(dirty)})
    return out


# ------------------------------------------------------------- static half

def _knobs(record: Optional[Dict]) -> Dict:
    """The FROZEN scoring knobs, read back from the record.

    Not re-derived from this module's own defaults, and that is load-bearing:
    if the perturber cut high-fanout nets at 20 and the scorer at 25, "stated
    and scored in the same units" is a fiction and the identity that makes the
    dose axis exact (distance_after == distance_before + dose) silently breaks.
    """
    sc = dict((record or {}).get('scoring') or {})
    sc.setdefault('clearance', 0.25)
    sc.setdefault('board_edge_clearance', 0.55)
    sc.setdefault('grid_step', 0.1)
    sc.setdefault('max_fanout', 20)
    sc.setdefault('ignore_net_ids', [])
    sc.setdefault('group_by', 'kicad,sheet')
    return sc


def static_metrics(pcb_data, board_path: str, record: Optional[Dict] = None
                   ) -> Dict:
    """HPWL / NC / OO / OoB / block displacement / board state, one pass.

    Built through `pose_score.make_state`, which is the canonical guidance-knob
    QuenchState builder, and **`build_neighbor_lists` is never called** -- so
    the travel budget stays infinite, `BoardOutlineGate.edges_near`'s per-ref
    cache is sized conservatively, and the exact ring test is never skipped.
    That is why the pruning caveat in `quench._group_offsets` does not apply to
    anything in this module.
    """
    from pose_score import make_state
    from placement import routability
    from placement.placement_state import assess_placement

    sc = _knobs(record)
    st = make_state(pcb_data, board_path,
                    clearance=sc['clearance'],
                    board_edge_clearance=sc['board_edge_clearance'],
                    grid_step=sc['grid_step'],
                    ignore_net_ids=sc['ignore_net_ids'] or None)
    cost = st.total_cost()
    legality = st.legality_metrics()

    # FROZEN membership: never re-derived from this board. `_from_netprefix`
    # gates on a 20 mm spatial radius and `decap_tethers` on 5 mm, so a 40 mm
    # perturbation CHANGES what the block is -- re-deriving would score a
    # different set of parts at every dose and the curve would mean nothing.
    blocks = {}
    frozen = ((record or {}).get('block') or {}).get('members')
    name = ((record or {}).get('block') or {}).get('name') or 'unit'
    if frozen:
        blocks = {name: [r for r in frozen if r in st.parts]}
    bd = routability.block_displacements(
        st, blocks, ignore_net_ids=sc['ignore_net_ids'] or None,
        max_fanout=sc['max_fanout']) if blocks else []

    state = assess_placement(pcb_data, board_path)
    return {
        'HPWL': round(float(cost.get('hpwl') or 0.0), 6),
        'NC': int(cost.get('crossings') or 0),
        'OO': round(float(legality.get('overlap_area') or 0.0), 6),
        'OoB': {'count': int(legality.get('oob_count') or 0),
                'amount': round(float(legality.get('oob_amount') or 0.0), 6),
                'area': round(float(legality.get('oob_area') or 0.0), 6)},
        'block_displacement_mm': (round(bd[0].distance_mm, 6) if bd else None),
        'block_displacement_ran': bool(bd),
        'state': {'unplaced': state.unplaced,
                  'partially_unplaced': state.partially_unplaced,
                  # The refs the flag above is actually decided on. Without it
                  # a scorecard shows `partially_unplaced: false` beside a
                  # non-zero `signals.duplicate_fraction` and a reader cannot
                  # tell "suppressed as markers / far side" from a broken
                  # check. Same reason floorplan's `state` block carries it.
                  'stacked_suspect_refs': list(state.stacked_suspect_refs),
                  'has_copper': state.has_copper,
                  'signals': state.signals},
    }


def score_board(board_path: str, record: Optional[Dict] = None, *,
                routed_path: Optional[str] = None,
                dirty_nets: Optional[Sequence[int]] = None) -> Dict:
    """The full scorecard for ONE board, freshly parsed from disk.

    Called identically on the control, the perturbed and the recovered board.
    `routed_path` adds the routed half; without it those columns report
    `ran: False` rather than a number nothing measured.
    """
    pcb = parse_kicad_pcb(board_path)
    out = {'board': os.path.basename(board_path)}
    out.update(static_metrics(pcb, board_path, record))

    poses = board_poses(pcb)
    orig = {r: tuple(v) for r, v in
            ((record or {}).get('original_poses') or {}).items()}
    members = ((record or {}).get('block') or {}).get('members') or []
    if orig:
        out['displacement'] = displacement_to_original(
            poses, orig, pcb.footprints, members)
        out['unit_spread_ratio'] = unit_spread_ratio(poses, orig, members)
        # Without this the ratio is unreadable: 1.03 against a 50 mm original
        # gyration means "cannot tell", not "not torn". See unit_spread_ratio.
        g = _gyration(orig, members)
        out['unit_gyration_orig_mm'] = round(g, 6) if g is not None else None
    else:
        out['displacement'] = {'ran': False,
                               'reason': 'record carries no original_poses'}
        out['unit_spread_ratio'] = None
        out['unit_gyration_orig_mm'] = None

    if routed_path and os.path.exists(routed_path):
        routed = parse_kicad_pcb(routed_path)
        sc = _knobs(record)
        nets = ((record or {}).get('scoring') or {}).get('net_ids')
        if not nets:
            nets = frozen_net_ids(routed, sc['ignore_net_ids'])
        out['route'] = connectivity_tally(
            routed, nets,
            prr_denominator=sc.get('prr_denominator'),
            rr_denominator=sc.get('rr_denominator'),
            dirty_nets=dirty_nets)
        out['route']['board'] = os.path.basename(routed_path)
    else:
        out['route'] = {'ran': False, 'reason': 'no routed board supplied'}
    return out
