"""K diverse placement candidates from one seed placement.

The quench is a zero-temperature greedy descent, and the placement stack is
deterministic by DESIGN (#457): same input, same knobs, same board, byte for
byte. That is the right property for reproducibility and exactly the wrong one
for exploring -- every run walks into the same local minimum, so "more
placement options" cannot come from re-running.

Diversity is therefore INJECTED at the seed, never un-suppressed: each
candidate is a legal PERTURBATION of the input placement (seeded jitter /
rotation variants / block-interior swaps), quenched with the ordinary engine,
then scored WITHOUT routing and pruned to a diverse, ranked slate. The
determinism contract survives intact:

  * every candidate draws from its own ``random.Random(f"{seed}:{i}:{strat}")``
    stream, so candidate i is byte-reproducible independently of the others --
    which is what makes ``--only N`` an exact replay primitive for a ledger;
  * everything else iterates sorted; there is no unseeded randomness, no
    clock, and no set-of-strings order anywhere in this module;
  * the quench engine is consumed through its public surface and never
    modified -- a plain place_optimize.py run is bit-identical with this
    module in the tree.

Perturbations are generated ON a shared oracle QuenchState and validated with
``candidate_valid`` as they are proposed (later parts see earlier ones already
moved), then written to a seed board with ``write_placed_output`` and quenched
from THAT file. Writing the quench result against the seed file -- exactly what
place_optimize does for its own input -- is what neutralizes quench's return
filter (a part that lands back on its perturbed seed is omitted from the
return, which is only safe when the output is written over the same seed).
"""
from __future__ import annotations

import fnmatch
import math
import os
import random
import shutil
from dataclasses import dataclass, field
from typing import Dict, List, NamedTuple, Optional, Sequence, Set, Tuple

DEFAULT_STRATEGIES: Tuple[str, ...] = ('jitter', 'poses', 'swap')
# How many of the highest-pin-count free parts the `poses` strategy enumerates
# rotation variants for. Rotations of multi-pin parts are where pin order lives
# (the U3 rot-180 case: 9 forced inversions -> 0); a 2-pad passive's rotation
# rarely changes anything the jitter cannot.
POSES_TOP_PARTS = 6
# Rejection-resampling budget per part for the seeded strategies. Bounded so a
# dense board terminates; a part that finds no legal sample simply stays put,
# which is a smaller perturbation, not an error.
SAMPLE_ATTEMPTS = 20
EPS = 1e-6


@dataclass
class Candidate:
    """One portfolio row: where it came from, what it became, how it measures."""
    index: int
    strategy: str
    seed_board: str = ''            # the perturbed (pre-quench) board, or ''
    board: str = ''                 # the quenched result, or '' when barren
    poses: List[Dict] = field(default_factory=list)   # the perturbation
    final_poses: Dict[str, Tuple[float, float, float]] = field(
        default_factory=dict)       # ref -> (x, y, rot) after the quench
    moved_refs: List[str] = field(default_factory=list)
    displacement_rms: float = 0.0
    metrics: Dict = field(default_factory=dict)
    gates: Dict = field(default_factory=dict)
    intent: Optional[Dict] = None
    health: Optional[Dict] = None
    route: Optional[Dict] = None
    route_full: Optional[Dict] = None   # --full-probe verdict (whole board)
    note: str = ''

    @property
    def viable(self) -> bool:
        return bool(self.board) and bool(self.gates.get('passed'))

    def to_dict(self) -> Dict:
        d = {'index': self.index, 'strategy': self.strategy,
             'seed_board': self.seed_board, 'board': self.board,
             'poses': self.poses,
             'final_poses': {r: [round(v, 4) for v in p]
                             for r, p in sorted(self.final_poses.items())},
             'moved_refs': self.moved_refs,
             'displacement_rms': round(self.displacement_rms, 3),
             'metrics': self.metrics, 'gates': self.gates,
             'note': self.note}
        if self.intent is not None:
            d['intent'] = self.intent
        if self.health is not None:
            d['health'] = self.health
        if self.route is not None:
            d['route'] = self.route
        if self.route_full is not None:
            d['route_full'] = self.route_full
        return d


# --------------------------------------------------------------------------
# board / state plumbing
# --------------------------------------------------------------------------

def free_refs(pcb_data, pcb_file: str,
              lock_globs: Optional[Sequence[str]] = None) -> List[str]:
    """Sorted refs the portfolio may perturb: pad-bearing, not `(locked yes)`
    on the board, not matching a --lock glob. The same two lock sources the
    quench itself honors, resolved once so every consumer agrees."""
    from placement.parser import extract_locked_refs
    locked = set(extract_locked_refs(pcb_file))
    out = []
    for ref, fp in pcb_data.footprints.items():
        if not fp.pads:
            continue
        if ref in locked:
            continue
        if lock_globs and any(fnmatch.fnmatch(ref, p) for p in lock_globs):
            continue
        out.append(ref)
    return sorted(out)


def ignore_net_ids(pcb_data, patterns: Optional[Sequence[str]]) -> Set[int]:
    """Net ids matching the --ignore-nets patterns (plane-routed rails)."""
    ids: Set[int] = set()
    if patterns:
        for net_id, net in pcb_data.nets.items():
            if any(fnmatch.fnmatch(net.name, p) for p in patterns):
                ids.add(net_id)
    return ids


def make_oracle(pcb_data, pcb_file: str, *, free: Sequence[str],
                clearance: float, board_edge_clearance: float,
                grid_step: float, ignore_ids: Set[int]):
    """The shared scoring/legality state. Guidance weights (pose_score), the
    run's geometry knobs, and `move_refs=free` so `.locked` on each part
    matches what the portfolio is allowed to touch."""
    import pose_score
    return pose_score.make_state(
        pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        ignore_net_ids=ignore_ids, move_refs=set(free))


def _snapshot(state, refs: Sequence[str]) -> Dict[str, Tuple[float, float, float]]:
    return {r: (state.parts[r].x, state.parts[r].y, state.parts[r].rot)
            for r in refs if r in state.parts}


def _restore(state, snap: Dict[str, Tuple[float, float, float]]) -> None:
    for ref in sorted(snap):
        x, y, rot = snap[ref]
        p = state.parts[ref]
        if (p.x, p.y, p.rot) != (x, y, rot):
            state.apply_move(ref, x, y, rot)


def copy_siblings(src_board: str, dst_board: str) -> None:
    """Carry the project/rules siblings (#441): a board without its .kicad_pro
    grades at the stock netclass, so every written candidate gets the input's
    siblings -- the probe routes and any later adoption read them."""
    src_base = os.path.splitext(src_board)[0]
    dst_base = os.path.splitext(dst_board)[0]
    for ext in ('.kicad_pro', '.kicad_dru', '.kicad_prl'):
        s = src_base + ext
        if os.path.isfile(s):
            shutil.copyfile(s, dst_base + ext)


# --------------------------------------------------------------------------
# perturbation strategies
# --------------------------------------------------------------------------
# Each strategy PROPOSES poses on the oracle state, validating with
# candidate_valid and applying accepted moves as it goes -- so a later part is
# tested against its already-perturbed peers, and the emitted set is legal as a
# WHOLE, not merely pairwise against the original board. The caller owns the
# snapshot/restore bracket.

def perturb_jitter(state, refs: Sequence[str], rng: random.Random,
                   radius: float, attempts: int = SAMPLE_ATTEMPTS) -> List[Dict]:
    """Uniform-disc offset per free ref, rotation unchanged. The direct
    basin-escape for a zero-temperature descent: the quench cannot leave a
    local minimum, so the seed does it instead, bounded by `radius`.

    A part whose INCUMBENT pose is not fully legal is skipped, not jittered.
    Two real populations live there: parts off the board by design (edge
    connectors, castellated rows -- candidate_valid's toward-the-board branch
    would happily walk one inboard and silently break its overhang spec) and
    dense hand-packed parts already under the courtyard clearance (any
    accepted sample would be a lateral trade the quench's own gate refuses).
    Skipping draws nothing from the rng, so the stream stays stable."""
    poses: List[Dict] = []
    for ref in refs:
        part = state.parts.get(ref)
        if part is None:
            continue
        if not state.candidate_valid(ref, part.x, part.y, part.rot):
            continue
        for _ in range(attempts):
            r = radius * math.sqrt(rng.random())
            th = 2.0 * math.pi * rng.random()
            x = round(part.x + r * math.cos(th), 4)
            y = round(part.y + r * math.sin(th), 4)
            if state.candidate_valid(ref, x, y, part.rot):
                state.apply_move(ref, x, y, part.rot)
                poses.append({'reference': ref, 'new_x': x, 'new_y': y,
                              'new_rotation': part.rot})
                break
    return poses


def perturb_poses(state, refs: Sequence[str],
                  variant_index: int) -> Optional[List[Dict]]:
    """Deterministic rotation variant: one of the top multi-pin free parts
    turned to a legal rotation that does not RAISE its forced-crossing floor.

    The inversion prune is the point -- ``pair_order.ref_inversions`` is a
    lower bound no router can remove, so a rotation that increases it is
    provably worse before any quench is paid (the U3 rot-180 story, run 5:
    the cost was indifferent, the inversion count was not). Returns None when
    `variant_index` runs past the legal variants: that strategy round is
    barren, which the caller reports rather than papers over."""
    from placement.pair_order import ref_inversions
    ranked = sorted((r for r in refs
                     if r in state.parts and state.parts[r].pin_count >= 2),
                    key=lambda r: (-state.parts[r].pin_count, r))[:POSES_TOP_PARTS]
    variants: List[Tuple[str, float]] = []
    for ref in ranked:
        part = state.parts[ref]
        # Same incumbent guard as jitter: a part off the board or already
        # under clearance must not be offered rotations through
        # candidate_valid's improvement branch.
        if not state.candidate_valid(ref, part.x, part.y, part.rot):
            continue
        base_inv = ref_inversions(state, ref)
        for delta in (90.0, 180.0, 270.0):
            rot = (part.rot + delta) % 360
            if not state.candidate_valid(ref, part.x, part.y, rot):
                continue
            if ref_inversions(state, ref, part.x, part.y, rot) > base_inv:
                continue
            variants.append((ref, rot))
    if variant_index >= len(variants):
        return None
    ref, rot = variants[variant_index]
    part = state.parts[ref]
    state.apply_move(ref, part.x, part.y, rot)
    return [{'reference': ref, 'new_x': part.x, 'new_y': part.y,
             'new_rotation': rot}]


def perturb_swaps(state, blocks: Dict[str, Sequence[str]], rng: random.Random,
                  n_swaps: int, attempts: int = SAMPLE_ATTEMPTS) -> List[Dict]:
    """Exchange the positions of two free members of one block, n_swaps times.

    This is the rearrangement the quench's own swap phase cannot reach: its
    swaps are capped by max_displacement and same-footprint only, so two
    different parts across a zone never trade places. Each exchange is tested
    part-vs-world with the partner excluded (it vacates), plus an exact
    pair test of the two NEW poses against each other.

    Members are filtered to FREE parts here, not left to geometry:
    candidate_valid answers "is this pose legal", never "may this part move",
    so without the filter a swap could relocate a locked connector."""
    def _free_members(refs):
        # Free AND legally seated: the same incumbent guard jitter applies --
        # an off-board or sub-clearance part must not be traded through
        # candidate_valid's improvement branch.
        return sorted(r for r in refs
                      if r in state.parts and not state.parts[r].locked
                      and state.parts[r].pin_count > 0
                      and state.candidate_valid(
                          r, state.parts[r].x, state.parts[r].y,
                          state.parts[r].rot))

    eligible = sorted(name for name, refs in blocks.items()
                      if len(_free_members(refs)) >= 2)
    if not eligible:
        return []
    poses: Dict[str, Dict] = {}
    made = 0
    for _ in range(attempts):
        if made >= n_swaps:
            break
        name = eligible[rng.randrange(len(eligible))]
        members = _free_members(blocks[name])
        a, b = rng.sample(members, 2)
        pa, pb = state.parts[a], state.parts[b]
        ax, ay, bx, by = pa.x, pa.y, pb.x, pb.y
        if not state.candidate_valid(a, bx, by, pa.rot, exclude={b}):
            continue
        if not state.candidate_valid(b, ax, ay, pb.rot, exclude={a}):
            continue
        # candidate_valid excluded the partner, so the two NEW poses were
        # never tested against each other; do it exactly.
        gap = pa.gap_to(pb, pa.rects(bx, by, pa.rot), pb.rects(ax, ay, pb.rot))
        if gap is not None and gap < state.clearance:
            continue
        state.apply_move(a, bx, by, pa.rot)
        state.apply_move(b, ax, ay, pb.rot)
        poses[a] = {'reference': a, 'new_x': bx, 'new_y': by,
                    'new_rotation': pa.rot}
        poses[b] = {'reference': b, 'new_x': ax, 'new_y': ay,
                    'new_rotation': pb.rot}
        made += 1
    return [poses[r] for r in sorted(poses)]


# --------------------------------------------------------------------------
# generation
# --------------------------------------------------------------------------

def _final_poses(pcb_data, placements: List[Dict]) -> Dict[str, Tuple[float, float, float]]:
    """Seed poses overlaid with the quench's returned moves. The quench omits
    parts that ended exactly on their seed, so the overlay IS the final state."""
    out = {ref: (fp.x, fp.y, fp.rotation % 360)
           for ref, fp in pcb_data.footprints.items() if fp.pads}
    for p in placements:
        out[p['reference']] = (p['new_x'], p['new_y'], p['new_rotation'])
    return out


def _displacement(final: Dict[str, Tuple[float, float, float]],
                  origin: Dict[str, Tuple[float, float, float]],
                  free: Sequence[str]) -> Tuple[float, List[str]]:
    """(rms displacement over free refs, refs that moved) vs the input board."""
    moved = []
    acc = 0.0
    n = 0
    for ref in free:
        if ref not in final or ref not in origin:
            continue
        fx, fy, frot = final[ref]
        ox, oy, orot = origin[ref]
        d = math.hypot(fx - ox, fy - oy)
        rot_changed = abs((frot - orot) % 360) > 1e-3
        if d > 1e-4 or rot_changed:
            moved.append(ref)
        acc += d * d + (1.0 if rot_changed else 0.0)
        n += 1
    return (math.sqrt(acc / n) if n else 0.0), moved


def pose_distance_mm(final_a: Dict[str, Tuple[float, float, float]],
                     final_b: Dict[str, Tuple[float, float, float]],
                     free: Sequence[str]) -> float:
    """RMS pose distance over the free refs; a rotation difference counts a
    flat 1.0. Pose distance, not hpwl distance, deliberately: hpwl reads only
    per-net extremes, so a mirrored or permuted arrangement can tie a clone --
    this measures "a genuinely different placement" directly."""
    acc = 0.0
    n = 0
    for ref in free:
        if ref not in final_a or ref not in final_b:
            continue
        ax, ay, arot = final_a[ref]
        bx, by, brot = final_b[ref]
        acc += (ax - bx) ** 2 + (ay - by) ** 2
        if abs((arot - brot) % 360) > 1e-3:
            acc += 1.0
        n += 1
    return math.sqrt(acc / n) if n else 0.0


def _quench_to(src_board: str, dst_board: str, quench_kw: Dict,
               groups: Optional[Dict], cancel_check=None,
               progress_callback=None) -> Tuple[Dict, Dict]:
    """Parse src, quench in-process, write the result AGAINST src (the seed
    file), carry siblings. Returns (metrics, final_poses)."""
    from kicad_parser import parse_kicad_pcb
    from placement.quench import quench
    from placement.writer import write_placed_output
    pcb_local = parse_kicad_pcb(src_board)
    m: Dict = {}
    placements = quench(pcb_local, pcb_file=src_board, metrics_out=m,
                        groups=groups, cancel_check=cancel_check,
                        progress_callback=progress_callback, **quench_kw)
    write_placed_output(src_board, dst_board, placements)
    copy_siblings(src_board, dst_board)
    return m, _final_poses(pcb_local, placements)


def generate(input_file: str, out_dir: str, *, seed: int = 0,
             n_candidates: int = 12,
             strategies: Sequence[str] = DEFAULT_STRATEGIES,
             radius: float = 4.0,
             lock_globs: Optional[Sequence[str]] = None,
             ignore_nets: Optional[Sequence[str]] = None,
             swap_blocks: Optional[Dict[str, Sequence[str]]] = None,
             quench_kw: Optional[Dict] = None,
             groups: Optional[Dict] = None,
             only: Optional[int] = None,
             cancel_check=None, progress_callback=None) -> Dict:
    """Generate the portfolio: baseline quench (candidate 0) plus perturbed
    candidates 1..n_candidates-1, all boards written under `out_dir`.

    Deterministic by contract: candidate i's stream is
    ``random.Random(f"{seed}:{i}:{strategy}")`` and its `poses` variant index
    is the round number ``(i-1) // len(strategies)`` -- both functions of i
    alone, so ``only=i`` regenerates that candidate byte-identically without
    running the others.

    Returns {'baseline': Candidate|None, 'candidates': [Candidate], 'free': [...]}.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output

    os.makedirs(out_dir, exist_ok=True)
    qkw = dict(quench_kw or {})
    strategies = tuple(strategies)
    if not strategies:
        raise ValueError("no strategies enabled")

    pcb = parse_kicad_pcb(input_file)
    free = free_refs(pcb, input_file, lock_globs)
    ids = ignore_net_ids(pcb, ignore_nets)
    oracle = make_oracle(
        pcb, input_file, free=free,
        clearance=qkw.get('clearance', 0.25),
        board_edge_clearance=qkw.get('board_edge_clearance', 0.55),
        grid_step=qkw.get('grid_step', 0.1), ignore_ids=ids)
    origin = {ref: (p.x, p.y, p.rot) for ref, p in oracle.parts.items()}

    baseline: Optional[Candidate] = None
    if only is None:
        print(f"[portfolio] candidate 0 (baseline): plain quench")
        board0 = os.path.join(out_dir, 'baseline_quenched.kicad_pcb')
        m, final = _quench_to(input_file, board0, qkw, groups, cancel_check=cancel_check,
                   progress_callback=progress_callback)
        rms, moved = _displacement(final, origin, free)
        baseline = Candidate(
            index=0, strategy='baseline', seed_board=input_file, board=board0,
            final_poses=final, moved_refs=moved, displacement_rms=rms,
            metrics=_quench_metrics(m))

    indices = [only] if only is not None else list(range(1, n_candidates))
    candidates: List[Candidate] = []
    for i in indices:
        # BETWEEN CANDIDATES, which is this loop's real unit: each one is a
        # full quench over the whole board. Checking inside the quench too
        # (it has its own hook) bounds the tail; this bounds the count.
        if cancel_check is not None and cancel_check():
            print(f"  portfolio: stopping before candidate {i} (budget)")
            break
        if i < 1:
            raise ValueError(f"candidate index {i} out of range (baseline is "
                             f"not regenerable via only=; it has no stream)")
        strategy = strategies[(i - 1) % len(strategies)]
        rng = random.Random(f"{seed}:{i}:{strategy}")
        snap = _snapshot(oracle, free)
        note = ''
        if strategy == 'jitter':
            poses = perturb_jitter(oracle, free, rng, radius)
        elif strategy == 'poses':
            poses = perturb_poses(oracle, free, (i - 1) // len(strategies))
            if poses is None:
                poses = []
                note = ('poses: no rotation variant left at round '
                        f'{(i - 1) // len(strategies)}')
        elif strategy == 'swap':
            n_swaps = rng.randint(1, 3)
            poses = perturb_swaps(oracle, swap_blocks or {}, rng, n_swaps)
            if not poses and not swap_blocks:
                note = 'swap: no blocks resolved (pass --intent or --group-by)'
        else:
            raise ValueError(f"unknown strategy {strategy!r}")
        _restore(oracle, snap)

        cand = Candidate(index=i, strategy=strategy, poses=poses, note=note)
        if not poses:
            cand.note = cand.note or f'{strategy}: produced no perturbation'
            cand.gates = {'passed': False, 'reasons': [cand.note]}
            candidates.append(cand)
            print(f"[portfolio] candidate {i} ({strategy}): barren -- {cand.note}")
            continue

        print(f"[portfolio] candidate {i} ({strategy}): "
              f"{len(poses)} part(s) perturbed")
        seed_board = os.path.join(out_dir, f'cand_{i:02d}.seed.kicad_pcb')
        write_placed_output(input_file, seed_board, poses)
        copy_siblings(input_file, seed_board)
        board = os.path.join(out_dir, f'cand_{i:02d}.kicad_pcb')
        m, final = _quench_to(seed_board, board, qkw, groups, cancel_check=cancel_check,
                   progress_callback=progress_callback)
        rms, moved = _displacement(final, origin, free)
        cand.seed_board = seed_board
        cand.board = board
        cand.final_poses = final
        cand.moved_refs = moved
        cand.displacement_rms = rms
        cand.metrics = _quench_metrics(m)
        candidates.append(cand)

    return {'baseline': baseline, 'candidates': candidates, 'free': free}


def _quench_metrics(m: Dict) -> Dict:
    """The unweighted, cross-run-comparable slice of a quench's metrics_out."""
    after = m.get('after', {})
    leg = m.get('legality', {})
    return {'crossings': after.get('crossings'),
            'hpwl': round(after.get('hpwl', 0.0), 3),
            'length': round(after.get('length', 0.0), 3),
            'overlap_area': round(leg.get('overlap_area', 0.0), 4),
            'oob_count': leg.get('oob_count', 0),
            'oob_amount': round(leg.get('oob_amount', 0.0), 4),
            # Pad+drill layer tallies (AABB gate currency; absent -> 0 when
            # the layer is off, keeping old-metric candidates comparable).
            'pad_conflict_pairs': leg.get('pad_conflict_pairs', 0),
            'pad_shortfall': leg.get('pad_shortfall', 0.0),
            'hole_shortfall': leg.get('hole_shortfall', 0.0)}


# --------------------------------------------------------------------------
# scoring, ranking, diversity
# --------------------------------------------------------------------------

def score_candidate(cand: Candidate, *, free: Sequence[str],
                    baseline_overlap: float, baseline_oob: int = 0,
                    baseline_pad_pairs: int = 0,
                    baseline_hole_shortfall: float = 0.0,
                    clearance: float, board_edge_clearance: float,
                    grid_step: float, ignore_nets: Optional[Sequence[str]],
                    intent=None, group_sources: Sequence[str] = ()) -> None:
    """Fill gates / inversions / intent / health for one quenched candidate.

    Hard gates (fail => not ranked, reason kept): legality (no MORE courtyard
    overlap and no MORE out-of-board parts than the baseline -- both measured
    against the baseline rather than zero, because a legitimate board can
    already carry both: edge connectors and castellated rows overhang the
    outline by design, and dense hand placements sit under the courtyard
    clearance) and, when an intent is given, an error-free
    ``floorplan.grade``. Health signals are ADVISORY (they join the rank key,
    not the gate) -- routability.py states why: they say the floorplan will
    fight the router, not that it is wrong.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.pair_order import pair_inversions

    if not cand.board:
        return
    pcb = parse_kicad_pcb(cand.board)
    ids = ignore_net_ids(pcb, ignore_nets)
    state = make_oracle(pcb, cand.board, free=free, clearance=clearance,
                        board_edge_clearance=board_edge_clearance,
                        grid_step=grid_step, ignore_ids=ids)
    inv_total = sum(m['inversions'] for m in pair_inversions(state).values())
    cand.metrics['inversions'] = inv_total

    reasons: List[str] = []
    overlap = cand.metrics.get('overlap_area', 0.0)
    if overlap > baseline_overlap + EPS:
        reasons.append(f"courtyard overlap {overlap:.4f}mm2 exceeds the "
                       f"baseline's {baseline_overlap:.4f}mm2")
    oob = cand.metrics.get('oob_count', 0)
    if oob > baseline_oob:
        reasons.append(f"{oob} part(s) out of board vs the baseline's "
                       f"{baseline_oob}")
    # Pad+drill layer gates, same baseline-relative shape as the two above.
    pad_pairs = cand.metrics.get('pad_conflict_pairs', 0) or 0
    # run-19: a PILE seed's baseline carries hundreds of pad pairs, and "no
    # worse than the baseline" then licenses ANY smaller intersection count.
    # Above 50 the baseline is degenerate -- compare against 0, and say so.
    if baseline_pad_pairs > 50:
        if pad_pairs > 0:
            reasons.append(
                f"{pad_pairs} pad-clearance conflict pair(s) vs 0: the seed "
                f"baseline's {baseline_pad_pairs} is pile-degenerate (> 50) "
                f"and licenses nothing")
    elif pad_pairs > baseline_pad_pairs:
        reasons.append(f"{pad_pairs} pad-clearance conflict pair(s) vs the "
                       f"baseline's {baseline_pad_pairs}")
    hole_sf = cand.metrics.get('hole_shortfall', 0.0) or 0.0
    if hole_sf > baseline_hole_shortfall + EPS:
        reasons.append(f"pad-copper-in-hole-keepout {hole_sf:.4f}mm vs the "
                       f"baseline's {baseline_hole_shortfall:.4f}mm")

    health_penalty = 0
    if intent is not None:
        from placement import floorplan
        result = floorplan.grade(intent, pcb, cand.board,
                                 group_sources=group_sources,
                                 clearance=clearance,
                                 board_edge_clearance=board_edge_clearance,
                                 with_health=True)
        errors = [v.to_dict() for v in result.errors]
        cand.intent = {'errors': len(errors), 'warnings': len(result.warnings),
                       'violations': errors[:10]}
        if errors:
            reasons.append(f"{len(errors)} intent violation(s): "
                           + '; '.join(v['message'] for v in errors[:3]))
        h = result.health or {}
        rows = h.get('bus_corridors') or []
        # `intrusions` in the row is TRUNCATED for display (routability.health
        # keeps the five deepest). Summing len() of it silently saturated the
        # intrusion component at 5 per corridor, so a channel with twelve parts
        # parked in it scored the same as one with five. Read the untruncated
        # count; fall back to the list for health JSON written before it.
        intrusions = sum(int(r['intrusions_total']) if 'intrusions_total' in r
                         else len(r.get('intrusions') or ())
                         for r in rows)
        health_penalty = (int(h.get('bus_foreign_crossings') or 0)
                          + intrusions + int(h.get('blocks_displaced') or 0))
        cand.health = {
            'bus_foreign_crossings': h.get('bus_foreign_crossings'),
            'intrusions': intrusions,
            'blocks_displaced': h.get('blocks_displaced'),
            'block_displacement_max_mm': h.get('block_displacement_max_mm'),
            'penalty': health_penalty}
    cand.metrics['health_penalty'] = health_penalty
    cand.gates = {'passed': not reasons, 'reasons': reasons}


class RankKey(NamedTuple):
    """The static order, as a NamedTuple so the slots have names.

    It IS a tuple, so every ``rank_key(a) < rank_key(b)`` comparison and every
    positional index keeps working; the names exist so a future reorder fails
    loudly in tests that mean a specific term instead of passing by accident.
    """
    crossings_band: int
    health_banded: int
    inversions: int
    plane_islands: int
    hpwl: float
    plane_neck: int
    health_legacy: int
    displacement: float
    index: int


def rank_key(cand: Candidate, q: int = 0) -> RankKey:
    """The static order: lexicographic over numbers the repo already trusts,
    most significant first, no new magic weights.

      crossings      the first-class pre-route judge (raw count)
      inversions     the forced-crossing floor -- equal-crossings ties break
                     toward the more REMOVABLE set (pair_order is a lower
                     bound a router cannot beat)
      plane_islands  --plane-score only (0 otherwise): how many pieces the
                     poured plane lands in. Before hpwl deliberately -- a
                     split pour is a MEASURED plane-step loss the repair
                     step must stitch, and hpwl is a proxy (#118: "routing
                     is as much about planes as traces")
      hpwl           order-invariant geometry
      plane_neck     --plane-score only: the #424 fragility field summed
                     over the fill (mm-equivalents, whole-mm rounded) --
                     a neck tiebreak below hpwl
      health         advisory floorplan-fight signals (0 without an intent)
      displacement   least disturbance wins what is left
      index          stability

    ``q`` is the crossings BAND width, and it is what lets the advisory health
    signal ever speak. Measured: crossings is near-injective over a real slate,
    so slot 1 decides and nothing below it runs -- health at slot 6 never
    spoke, and health at slot 2 would speak just as rarely. Banding the first
    slot is the lever, not reordering it.

      q = 0 (default)  legacy order, bit-identical: health stays in its old
                       slot 6 and crossings are compared raw.
      q >= 1           crossings quantised to ceil(c/q) and health promoted to
                       slot 2, so inside one band the term that says "this
                       floorplan will FIGHT the router" decides. inversions,
                       plane_* and hpwl are all the same family as crossings
                       ("how hard is this to route"); health is the only one
                       that is about the floorplan fighting back.

    Quantisation, never a tolerance compare: ``abs(a-b) <= tol`` is not
    transitive, and ``sorted()`` on a non-transitive key is undefined. Integer
    ceil-div is a function into a totally ordered set, so it is well defined.
    """
    m = cand.metrics
    crossings = m.get('crossings', 1 << 30)
    health = m.get('health_penalty', 0)
    banded = q >= 1
    return RankKey(
        crossings_band=-(-crossings // q) if banded else crossings,
        health_banded=health if banded else 0,
        inversions=m.get('inversions', 1 << 30),
        plane_islands=m.get('plane_islands', 0),
        hpwl=round(m.get('hpwl', float('inf')), 3),
        plane_neck=round(m.get('plane_neck', 0.0)),
        health_legacy=0 if banded else health,
        displacement=round(cand.displacement_rms, 2),
        index=cand.index)


def band_width(cands: Sequence[Candidate], band_frac: float,
               baseline_crossings: Optional[int] = None) -> int:
    """The band width q for `band_frac`, or 0 when banding is off/inert.

    Two ways to end up at 0, both deliberate:
      * band_frac <= 0 -- the default, legacy order.
      * every ranked candidate scores health_penalty 0 (no --intent, or an
        intent with no health block). Banding then coarsens the strongest
        signal to buy nothing, so it is refused rather than applied blind.
    """
    if band_frac <= 0:
        return 0
    viable = [c for c in cands if c.viable]
    if not any(c.metrics.get('health_penalty', 0) for c in viable):
        return 0
    if baseline_crossings is None:
        baseline_crossings = min((c.metrics.get('crossings', 0)
                                  for c in viable), default=0)
    return max(1, round(band_frac * baseline_crossings))


def rank_static(cands: Sequence[Candidate], q: int = 0) -> List[int]:
    """Indices of the gate-passing candidates, best first."""
    return [c.index for c in sorted((c for c in cands if c.viable),
                                    key=lambda c: rank_key(c, q))]


def rule1_check(cand: Candidate, baseline: Candidate) -> List[str]:
    """Rule 1 of the Step-0c acceptance conjunction, applied K-way (run-7
    S6): crossings and hpwl must be NO WORSE than the baseline row.

    The hard gates in score_candidate are legality + intent (rule 2). This
    metric clause was DOCUMENTED as pre-applied but never was, so a
    candidate measuring worse on both proxies could top the slate and ship
    through JSON_SUMMARY['best'] unremarked. Pure: returns the violation
    strings; empty means it passes. Violators are still ranked, probed and
    recorded -- they are only barred from being the silent headline pick
    (select_best); a probe verdict that argues for one anyway is a decision
    the operator makes deliberately, reading portfolio.json.
    """
    out: List[str] = []
    bm, cm = baseline.metrics, cand.metrics
    bx, cx = bm.get('crossings'), cm.get('crossings')
    if bx is not None and cx is not None and cx > bx:
        out.append(f'crossings {cx} > baseline {bx}')
    bh, ch = bm.get('hpwl'), cm.get('hpwl')
    if bh is not None and ch is not None and ch > bh + EPS:
        out.append(f'hpwl {ch:.1f} > baseline {bh:.1f}')
    return out


def select_best(ranking_primary: Sequence[int], ranking_static: Sequence[int],
                rule1_violators: Set[int]) -> Optional[int]:
    """The slate's headline pick: the first ranked index that passes rule 1.

    `ranking_primary` (a probe ranking -- full-board when --full-probe ran,
    else the shared-window one) outranks the static ranking. The baseline
    (index 0) trivially passes rule 1 against itself, so "keep what you
    have" stays a first-class outcome. Returns None when nothing ranked
    passes -- the caller falls back to the baseline, never to a violator.
    """
    seen = set()
    for idx in list(ranking_primary) + list(ranking_static):
        if idx in seen:
            continue
        seen.add(idx)
        if idx == 0 or idx not in rule1_violators:
            return idx
    return None


def select_diverse(cands: Sequence[Candidate], order: Sequence[int],
                   keep: int, min_dist_mm: float,
                   free: Sequence[str],
                   baseline: Optional[Candidate]) -> Tuple[List[int], List[int]]:
    """Walk the ranked order, keeping a candidate only when it sits at least
    `min_dist_mm` (pose distance) from everything already kept -- the baseline
    included, so a candidate that quenched back into the incumbent's basin is
    recognized as "no new option" rather than presented twice. When fewer than
    `keep` survive, backfill by rank and SAY so (second return value): a slate
    padded with near-clones must not read as a diverse one.
    """
    by_index = {c.index: c for c in cands}
    kept: List[int] = []
    kept_poses = [baseline.final_poses] if baseline is not None else []
    for idx in order:
        if len(kept) >= keep:
            break
        c = by_index[idx]
        if all(pose_distance_mm(c.final_poses, p, free) >= min_dist_mm
               for p in kept_poses):
            kept.append(idx)
            kept_poses.append(c.final_poses)
    backfilled: List[int] = []
    if len(kept) < keep:
        for idx in order:
            if len(kept) >= keep:
                break
            if idx not in kept:
                kept.append(idx)
                backfilled.append(idx)
    return kept, backfilled
