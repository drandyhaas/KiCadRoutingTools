"""Which parts to offer the quench, from routing EVIDENCE rather than pin count (#553).

`place_route_loop` has always picked its movers with `nets_to_refs`: the pad
owners of the failed and blocker nets, minus anything with more pins than
`--max-target-pins`. #553's complaint is that the pin cap is a proxy that picks
wrong exactly when it matters, "because the part that needs to move is never a
passive" -- the cap EXCLUDES the IC whose position is the problem and OFFERS the
passives that are not.

This module ranks candidates on three signals and hands the union to the loop.
It changes WHICH parts are offered, never how the quench scores them.

WHAT BACKS EACH SIGNAL, which is not the same for the three
-----------------------------------------------------------
`block_displacement`  MECHANISM ONLY. `routability.block_displacements` is the
    #407 case: a block sitting far from the centroid of everything it connects
    to, which no 3 mm nudge can fix. It has never been measured against a
    routed outcome, and this module does not pretend otherwise.

`blocker_cells`  NOT A PREDICTOR, and that is the point. #703 measured
    PRE-ROUTE quantities PREDICTING a LATER routed outcome. This is the
    opposite direction: the router already failed and already said which
    copper blocked its frontier, so this is a post-hoc attribution of an
    observed fact. There is nothing to correlate.

`legality_pairs`  THE ONLY MEASURED ONE. `docs/placement-predictors.md` ranked
    pre-route predictors against routed `blocking` over 120 placements on 6
    boards, and the legality counts are what passed: pad and body pairs at
    rho = +0.785 on 6 of 6 boards, courtyard blocking at rho = +0.684 on 6 of 6,
    where `crossings` and `hpwl` failed.
    **The extrapolation is disclosed and is the weakest link here:** #703
    measured those counts as BOARD-LEVEL scalars ranking a BOARD-LEVEL outcome
    across many placements. Using them as PER-CANDIDATE counts ranking
    candidates WITHIN one board is a different question, and nobody has
    measured it.

WHAT IS NOT HERE, AND WHY
-------------------------
`foreign_crossings` -- the third signal #553 names -- is deliberately absent.
Three independent reasons, and the third is stated carefully because getting it
wrong in either direction becomes folklore:

  1. There is no input. It needs a declared `routability.Corridor`, built by
     `corridors_from_intent` from an intent's `bus_corridors`.
     `place_route_loop` has no corridor plumbing at all.
  2. Auto-deriving corridors to supply that input would manufacture the number.
     `CORRIDOR_MIN_COVER` exists precisely because a cluster fit returns a
     confident-looking rectangle for a bus that is not there (splitflap's
     `OUT_*`: 24 nets, cover 0.0).
  3. Family-level doubt, NOT a measurement of this function. #703 measured
     `crossings` -- ratsnest airwire crossings -- which failed its sign test on
     one of six boards and whose verdict flips depending on whether quench
     output is in the sample. `foreign_crossings` is a corridor-pierce count, a
     DIFFERENT quantity, and it has never been measured against anything. Do not
     record this omission as "#703 measured it and it failed".

Off-outline `pad_copper` (rho = +0.524 against routed `blocking`, 6/6 boards in
`docs/placement-predictors.md`) is also absent
from v1: the authoritative measure is `render_placement.legality_findings`,
which needs a whole `PlacementModel` per round for the weakest of the passing
predictors. Named here so it is a known gap rather than an oversight.

NO EFFICACY CLAIM
-----------------
No paired routed A/B of `pins` against `diagnosis` exists. Nothing in this
module or its callers may state or imply that diagnosis routes better. The
selector ships default-off, and the run verdict carries that sentence so a
machine consumer cannot read the result without it.

WHY THERE IS NO COMBINED SCORE
------------------------------
There is no measured exchange rate between millimetres, blocked cells and
defect pairs. Any scalar mixing them is a magic weight -- including a rank sum,
which is the hidden assertion that the three are equally informative, and that
is measured-false (one has 6/6 board-level backing, one has none, one is an
observation rather than a prediction). So: three independent rankings, unioned
ROUND ROBIN over a declared `SIGNAL_ORDER` so no signal owns the head. A
`rank_sum` is computed and reported as ADVISORY, never used to select, so a
future measurement has a candidate to test.

Every degeneracy gets a NAMED REASON rather than a number. A signal that
collapsed is `skipped` with why; it never contributes a silent 0.0.
"""

import fnmatch
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

if __package__ in (None, ''):                       # pragma: no cover
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from placement.routability import (  # noqa: E402
    DISPLACEMENT_MAX_FANOUT, block_displacements)

SCHEMA = 1

#: Ranking precision. Values are compared at this many decimals so floating
#: point noise cannot reorder two candidates differently in two processes
#: (the determinism rule). A REPORTING precision, not a threshold on a metric:
#: `BlockDisplacement.to_dict` already rounds `distance_mm` to the same 4.
RANK_DECIMALS = 4

#: How many candidates each signal offers per round-robin sweep. A REPORT SIZE,
#: in the same class as `grade_pad_legality`'s `worst_n=10` and the router's
#: `FRONTIER_BLOCKING_RECORD_CAP=10` -- not a calibrated threshold on any
#: arrangement metric.
TOP_K = 3

#: Fixed, declared, and never chosen per run: it breaks the intra-sweep tie
#: only. Changing it changes which signal is asked first in each sweep.
SIGNAL_ORDER = ('block_displacement', 'blocker_cells', 'legality_pairs')

SIGNAL_UNITS = {'block_displacement': 'mm',
                'blocker_cells': 'cells',
                'legality_pairs': 'pairs'}

#: The sentence that must travel with any diagnosis-selected run.
NO_EFFICACY_CLAIM = (
    'NOT MEASURED: no paired routed A/B of pins vs diagnosis exists. '
    'See py_placer/placement/diagnosis.py and docs/placement-predictors.md.')

_NO_BLOCKS = 'no blocks resolved; derive them with --group-by'


# --------------------------------------------------------------------------
# The router's frontier evidence
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class BlockerEvidence:
    """What the router attributed its blocked frontier to, per blocker NET.

    THE DENOMINATOR IS TRUNCATED AND THE NAME SAYS SO. `BlockingReport` counts
    whole-frontier cells in-router, but `blocking_info_to_dict` does not
    serialise them; what reaches a consumer is a per-failed-net list capped at
    10 entries with `more` counting the tail. So `total_cells` is the sum over
    what was REPORTED, and `cells_dropped` names what is missing. Any share
    computed from this is a share of the reported evidence -- never call it
    "share of frontier-blocked cells".
    """
    cells: Dict[str, int] = field(default_factory=dict)
    unique: Dict[str, int] = field(default_factory=dict)
    total_cells: int = 0
    countless: Tuple[str, ...] = ()
    truncated_nets: int = 0
    cells_dropped: int = 0
    entries: int = 0

    def to_dict(self) -> Dict[str, object]:
        return {'nets': len(self.cells),
                'total_cells': self.total_cells,
                'countless': list(self.countless),
                'truncated_nets': self.truncated_nets,
                'cells_dropped': self.cells_dropped,
                'entries': self.entries,
                'basis': 'sum over REPORTED blockers (10/net cap)'}


def blocker_evidence(report: Optional[Sequence[Dict]]) -> BlockerEvidence:
    """Fold `summary['blockers']` into per-net cell counts. Pure.

    Tolerates every shape route.py emits: the ordinary entry, the
    `stage='preexisting'` variant whose `blocked_by` items carry a net name and
    nothing else, an entry with no `blocked_by` key at all, and the `more`
    tail. A missing `blocked_count` is NEVER defaulted to 1 -- the net is
    recorded in `countless` and contributes no cells, because inventing the
    count invents the evidence this module ranks on.
    """
    cells: Dict[str, int] = {}
    unique: Dict[str, int] = {}
    countless: set = set()
    truncated = 0
    dropped = 0
    entries = 0
    for e in (report or ()):
        if not isinstance(e, dict):
            continue
        more = e.get('more')
        if isinstance(more, int) and more > 0:
            truncated += 1
            dropped += more
        for b in (e.get('blocked_by') or ()):
            if not isinstance(b, dict):
                continue
            net = b.get('net')
            if net is None:
                continue
            entries += 1
            if 'blocked_count' not in b:
                countless.add(net)
                continue
            cells[net] = cells.get(net, 0) + int(b.get('blocked_count') or 0)
            unique[net] = unique.get(net, 0) + int(b.get('unique_cells') or 0)
    # A net that appears both counted and count-less is counted: the count is
    # real evidence and the other entry merely added no more of it.
    countless -= set(cells)
    return BlockerEvidence(
        cells=dict(sorted(cells.items())), unique=dict(sorted(unique.items())),
        total_cells=sum(cells.values()), countless=tuple(sorted(countless)),
        truncated_nets=truncated, cells_dropped=dropped, entries=entries)


# --------------------------------------------------------------------------
# Legality defects, per ref pair
# --------------------------------------------------------------------------

def resolve_clearance(pcb_file: Optional[str] = None,
                      clearance: Optional[float] = None) -> float:
    """The board's OWN Default-netclass clearance, never a guessed round number.

    Same resolution `lock_advisor.advise_locks` uses for its pad-conflict
    demotion: the board's Default class, else `routing_defaults.CLEARANCE`.
    Grading at a value the board does not use manufactures phantom pairs on one
    side and misses real ones on the other.
    """
    if clearance:
        return float(clearance)
    val = None
    if pcb_file:
        try:
            from list_nets import board_default_netclass_clearance
            val = board_default_netclass_clearance(pcb_file)
        except Exception:
            val = None
    if not val:
        try:
            from routing_defaults import CLEARANCE as _DEF
            val = _DEF
        except Exception:
            val = 0.25
    return float(val)


def legality_defects(pcb_data, clearance: Optional[float] = None,
                     pcb_file: Optional[str] = None) -> Dict[str, object]:
    """`{pairs: [(a, b, kind)], clearance: mm, notes: [...]}`. Reads the board.

    The one function here that touches the file, kept apart from `diagnose` so
    the ranking stays pure. Kinds, all per-REF-PAIR and therefore attributable:

      pad_clearance        `grade_pad_legality(..., worst_n=0)['worst']`
      body_overlap         `grade_body_overlap(...)['blocking_pairs']`
      courtyard_blocking   ...`['courtyard_blocking_pairs']`

    A grader that RAISES is recorded in `notes` and contributes no pairs. A
    grader that returns an error string (the courtyard census) has that string
    propagated verbatim: "blocking: none" on a census that did not run is the
    silence this repo keeps filing bugs about.
    """
    from placement import legality as _leg
    clr = resolve_clearance(pcb_file, clearance)
    pairs: List[Tuple[str, str, str]] = []
    notes: List[str] = []

    try:
        rep = _leg.grade_pad_legality(pcb_data, clr, worst_n=0,
                                      pcb_file=pcb_file)
        for (a, b, _mm) in (rep.get('worst') or ()):
            pairs.append((a, b, 'pad_clearance'))
        for note in (rep.get('clearance_notes') or ()):
            notes.append(f'pad_clearance: {note}')
    except Exception as e:                            # noqa: BLE001
        notes.append(f'pad_clearance grader failed: {type(e).__name__}: {e}')

    try:
        bo = _leg.grade_body_overlap(pcb_data, clr, pcb_file=pcb_file)
        for p in (bo.get('blocking_pairs') or ()):
            pairs.append((p.a, p.b, 'body_overlap'))
        for p in (bo.get('courtyard_blocking_pairs') or ()):
            pairs.append((p.a, p.b, 'courtyard_blocking'))
        err = bo.get('courtyard_census_error')
        if err:
            notes.append(f'courtyard census: {err}')
    except Exception as e:                            # noqa: BLE001
        notes.append(f'body_overlap grader failed: {type(e).__name__}: {e}')

    # Normalised and de-duplicated: the same pair may reach two graders.
    seen = set()
    out: List[Tuple[str, str, str]] = []
    for a, b, kind in pairs:
        key = (min(a, b), max(a, b), kind)
        if key in seen:
            continue
        seen.add(key)
        out.append(key)
    out.sort()
    return {'pairs': out, 'clearance': clr, 'notes': notes}


def ignore_net_ids(pcb_data, patterns: Optional[Sequence[str]]) -> set:
    """Net ids matching `--ignore-nets` globs (the plane-routed rails).

    Same rule as `portfolio.ignore_net_ids`, restated here only so this module
    imports nothing from the portfolio engine.
    """
    ids = set()
    if patterns:
        for nid, net in (pcb_data.nets or {}).items():
            if any(fnmatch.fnmatch(net.name, p) for p in patterns):
                ids.add(nid)
    return ids


def make_state(pcb_data, pcb_file: str, **kw):
    """A `QuenchState` for ranking only: `pad_legality` OFF.

    The ranking reads `state.parts`, `state.net_refs` and `Part.pad_globals()`
    and nothing else. The pad-legality layer is a different question, answered
    by `legality_defects` against the file's own poses, so paying for it here
    would build the same answer twice.
    """
    from placement.quench import QuenchState
    args = dict(clearance=0.25, board_edge_clearance=0.55, grid_step=0.1,
                crossing_penalty=30.0, length_weight=0.3, halo_base=0.5,
                halo_coef=0.15, halo_weight=2.0, edge_halo=2.0,
                edge_weight=2.0)
    args.update(kw)
    return QuenchState(pcb_data=pcb_data, pcb_file=pcb_file,
                       pad_legality=False, **args)


# --------------------------------------------------------------------------
# The ranking
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class SignalRow:
    """One candidate's standing on one signal."""
    signal: str
    value: float
    rank: int
    unit: str
    detail: Dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        return {'signal': self.signal, 'value': round(self.value, RANK_DECIMALS),
                'rank': self.rank, 'unit': self.unit,
                'detail': dict(sorted(self.detail.items()))}


@dataclass(frozen=True)
class Candidate:
    """A block, or a single part that belongs to no block."""
    key: str
    kind: str                       # 'block' | 'part'
    members: Tuple[str, ...]
    rows: Tuple[SignalRow, ...] = ()
    selected_by: Tuple[str, ...] = ()

    @property
    def rank_sum(self) -> int:
        """ADVISORY ONLY -- never used to select. See the module docstring."""
        return sum(r.rank for r in self.rows)

    def to_dict(self) -> Dict[str, object]:
        return {'key': self.key, 'kind': self.kind,
                'members': list(self.members),
                'rows': [r.to_dict() for r in self.rows],
                'selected_by': list(self.selected_by),
                'rank_sum_advisory': self.rank_sum}


@dataclass
class Diagnosis:
    schema: int = SCHEMA
    candidates: List[Candidate] = field(default_factory=list)
    selected: List[str] = field(default_factory=list)      # refs, sorted
    selected_keys: List[str] = field(default_factory=list)  # candidate keys
    skipped: Dict[str, str] = field(default_factory=dict)
    disclosures: List[str] = field(default_factory=list)
    evidence: Optional[BlockerEvidence] = None
    budget: Optional[int] = None
    overshoot: int = 0
    top_k: int = TOP_K
    efficacy: str = NO_EFFICACY_CLAIM

    @property
    def signals_defined(self) -> List[str]:
        return [s for s in SIGNAL_ORDER if s not in self.skipped]

    @property
    def degenerate(self) -> bool:
        """True when no signal could be defined -- the caller must fall back."""
        return not self.signals_defined or not self.selected

    def fallback_reason(self) -> str:
        if self.signals_defined and self.selected:
            return ''
        if not self.signals_defined:
            return ('no signal could be defined: '
                    + '; '.join(f'{s}: {self.skipped[s]}' for s in SIGNAL_ORDER
                                if s in self.skipped))
        return 'every defined signal ranked nothing above zero'

    def tally(self) -> Dict[str, object]:
        return {'signals': self.signals_defined,
                'candidates': len(self.candidates),
                'selected_keys': len(self.selected_keys),
                'selected_parts': len(self.selected),
                'budget': self.budget,
                'overshoot': self.overshoot,
                'degenerate': self.degenerate}

    def to_dict(self) -> Dict[str, object]:
        return {'schema': self.schema,
                'signals': self.signals_defined,
                'skipped': dict(sorted(self.skipped.items())),
                'disclosures': list(self.disclosures),
                'candidates': [c.to_dict() for c in self.candidates],
                'selected': list(self.selected),
                'selected_keys': list(self.selected_keys),
                'evidence': self.evidence.to_dict() if self.evidence else None,
                'budget': self.budget,
                'overshoot': self.overshoot,
                'top_k': self.top_k,
                'efficacy': self.efficacy,
                'fallback_reason': self.fallback_reason()}


def _rank(values: Dict[str, float]) -> List[Tuple[str, float]]:
    """Descending by ROUNDED value, then key ascending. Deterministic."""
    return sorted(((k, v) for k, v in values.items() if v > 0),
                  key=lambda kv: (-round(kv[1], RANK_DECIMALS), kv[0]))


def diagnose(state, pcb_data, blocks: Optional[Dict[str, Sequence[str]]] = None,
             *,
             blocker_report: Optional[Sequence[Dict]] = None,
             legality: Optional[Dict[str, object]] = None,
             ignore_net_ids: Optional[Sequence[int]] = None,
             max_fanout: int = DISPLACEMENT_MAX_FANOUT,
             budget: Optional[int] = None,
             top_k: int = TOP_K) -> Diagnosis:
    """Rank blocks and unblocked parts on routing evidence. Pure: no IO.

    `blocker_report` is `metrics_from_summary(..., keep_blocker_cells=True)`'s
    `blocker_report` -- the raw list, or None when no structured evidence
    exists. `legality` is `legality_defects(...)`'s dict, or None to skip that
    signal. `budget` caps the selected PART count; a block is always added
    WHOLE, so the cap can be overshot, and the overshoot is reported rather
    than the block half-moved.
    """
    blocks = {k: list(v) for k, v in sorted((blocks or {}).items())}
    d = Diagnosis(budget=budget, top_k=top_k)

    parts = getattr(state, 'parts', {}) or {}
    net_refs = getattr(state, 'net_refs', {}) or {}

    ignored = set(ignore_net_ids or ())
    if max_fanout:
        ignored.update(nid for nid, refs in net_refs.items()
                       if len(refs) > max_fanout)

    # ---- candidate universe: every block, plus every part in no block.
    block_of: Dict[str, str] = {}
    for name in sorted(blocks):
        for ref in sorted(blocks[name]):
            if ref in parts and ref not in block_of:
                block_of[ref] = name
    members: Dict[str, Tuple[str, ...]] = {}
    kinds: Dict[str, str] = {}
    for name in sorted(blocks):
        mem = tuple(r for r in sorted(blocks[name]) if r in parts)
        if not mem:
            continue
        members[name] = mem
        kinds[name] = 'block'
    for ref in sorted(parts):
        if ref in block_of:
            continue
        members[ref] = (ref,)
        kinds[ref] = 'part'

    def owner(ref: str) -> Optional[str]:
        key = block_of.get(ref, ref)
        return key if key in members else None

    values: Dict[str, Dict[str, float]] = {s: {} for s in SIGNAL_ORDER}
    details: Dict[str, Dict[str, Dict]] = {s: {} for s in SIGNAL_ORDER}

    # ---- signal 1: connectivity-centroid displacement (blocks only)
    if not blocks:
        d.skipped['block_displacement'] = _NO_BLOCKS
    else:
        bd = block_displacements(state, blocks, sorted(ignored),
                                 max_fanout=max_fanout)
        if not bd:
            d.skipped['block_displacement'] = (
                'every block was omitted: no own pads after the ignore/fanout '
                'cut, or no pads outside the block on its own nets')
        else:
            for row in bd:
                values['block_displacement'][row.block] = row.distance_mm
                details['block_displacement'][row.block] = {
                    'foreign_pads': row.foreign_pads, 'nets': row.nets,
                    'nets_ignored': row.nets_ignored}
            omitted = sorted((set(members) & set(blocks))
                             - {r.block for r in bd})
            if omitted:
                d.disclosures.append(
                    f'block_displacement: {len(omitted)} block(s) omitted '
                    f'rather than scored 0.0 ({", ".join(omitted[:4])}'
                    f'{"..." if len(omitted) > 4 else ""})')

    # ---- signal 2: blocked cells owned, from the router's own attribution
    if blocker_report is None:
        d.skipped['blocker_cells'] = (
            'no structured blocker evidence: the pre-#409 log fallback yields '
            'net names without counts')
    else:
        ev = blocker_evidence(blocker_report)
        d.evidence = ev
        if ev.countless:
            d.disclosures.append(
                f'blocker_cells: {len(ev.countless)} blocker net(s) carried no '
                f'cell counts (route.py stage=preexisting) and were counted, '
                f'never imputed')
        if ev.cells_dropped:
            d.disclosures.append(
                f'blocker_cells: the report is truncated -- {ev.truncated_nets} '
                f'net(s) dropped {ev.cells_dropped} further attribution(s); a '
                f'share here is a share of what was REPORTED')
        if not ev.cells:
            d.skipped['blocker_cells'] = (
                'no blocker entry carried a cell count'
                + (' (every entry is stage=preexisting)' if ev.countless
                   else ' (the router attributed nothing)'))
        else:
            name_to_id = {n.name: nid for nid, n in (pcb_data.nets or {}).items()}
            unassigned = 0
            skipped_power = 0
            for net, cells in sorted(ev.cells.items()):
                nid = name_to_id.get(net)
                if nid is None or nid in ignored:
                    skipped_power += cells
                    continue
                tally: Dict[str, int] = {}
                for ref in sorted(net_refs.get(nid, ())):
                    key = owner(ref)
                    if key is None:
                        continue
                    tally[key] = tally.get(key, 0) + 1
                if not tally:
                    unassigned += cells
                    continue
                total = sum(tally.values())
                win = sorted(tally.items(), key=lambda kv: (-kv[1], kv[0]))[0]
                values['blocker_cells'][win[0]] = (
                    values['blocker_cells'].get(win[0], 0) + cells)
                det = details['blocker_cells'].setdefault(
                    win[0], {'nets': [], 'plurality': 0})
                det['nets'] = sorted(det['nets'] + [net])[:8]
                if win[1] * 2 < total:
                    det['plurality'] += 1
            if skipped_power:
                d.disclosures.append(
                    f'blocker_cells: {skipped_power} cell(s) on ignored or '
                    f'high-fanout nets were dropped -- a rail that reaches '
                    f'everywhere ranks nothing')
            if unassigned:
                d.disclosures.append(
                    f'blocker_cells: {unassigned} cell(s) belong to nets no '
                    f'candidate owns a pad of; never selected')
            if not values['blocker_cells']:
                d.skipped['blocker_cells'] = (
                    'every counted blocker net was an ignored/high-fanout rail '
                    'or owned by no candidate')

    # ---- signal 3: legality defect pairs incident on the candidate
    if legality is None:
        d.skipped['legality_pairs'] = 'not computed for this run'
    else:
        for note in (legality.get('notes') or ()):
            d.disclosures.append(f'legality_pairs: {note}')
        pairs = legality.get('pairs') or ()
        if not pairs:
            d.skipped['legality_pairs'] = (
                f'no legality findings at the board\'s own clearance '
                f'({legality.get("clearance")} mm)')
        else:
            seen_kinds: Dict[str, set] = {}
            for a, b, kind in pairs:
                ka, kb = owner(a), owner(b)
                for key in {k for k in (ka, kb) if k is not None}:
                    values['legality_pairs'][key] = (
                        values['legality_pairs'].get(key, 0) + 1)
                    seen_kinds.setdefault(key, set()).add(kind)
                    if ka is not None and ka == kb:
                        det = details['legality_pairs'].setdefault(
                            key, {'kinds': [], 'internal': 0})
                        det['internal'] = det.get('internal', 0) + 1
            for key, ks in seen_kinds.items():
                det = details['legality_pairs'].setdefault(
                    key, {'kinds': [], 'internal': 0})
                det['kinds'] = sorted(ks)
            if not values['legality_pairs']:
                d.skipped['legality_pairs'] = (
                    'every legality pair names a part outside the candidate set')

    # ---- rank each defined signal, drop the ones with nothing to say
    ranked: Dict[str, List[Tuple[str, float]]] = {}
    for sig in SIGNAL_ORDER:
        if sig in d.skipped:
            continue
        order = _rank(values[sig])
        if not order:
            d.skipped[sig] = 'no candidate scored above zero'
            continue
        vals = {round(v, RANK_DECIMALS) for _, v in order}
        if len(order) > 1 and len(vals) == 1:
            d.skipped[sig] = (
                f'no spread at {RANK_DECIMALS} dp: all {len(order)} candidates '
                f'scored {order[0][1]:.4g} -- a ranking here would be arbitrary')
            continue
        if len(order) == 1:
            d.disclosures.append(
                f'{sig}: exactly one candidate is defined on this signal, so '
                f'the ranking made no choice')
        ranked[sig] = order

    rows: Dict[str, List[SignalRow]] = {}
    for sig, order in ranked.items():
        for pos, (key, val) in enumerate(order, start=1):
            rows.setdefault(key, []).append(SignalRow(
                signal=sig, value=float(val), rank=pos, unit=SIGNAL_UNITS[sig],
                detail=details[sig].get(key, {})))

    # ---- round-robin union of the per-signal top-k
    pools = {sig: [k for k, _ in ranked[sig][:max(0, top_k)]] for sig in ranked}
    chosen: List[str] = []
    chosen_set: set = set()
    selected_by: Dict[str, List[str]] = {}
    n_parts = 0
    exhausted = False
    while not exhausted and any(pools.get(s) for s in SIGNAL_ORDER):
        for sig in SIGNAL_ORDER:
            pool = pools.get(sig)
            if not pool:
                continue
            if budget is not None and n_parts >= budget:
                exhausted = True
                break
            key = pool.pop(0)
            # `selected_by` is recorded only for a key that IS selected: a
            # signal cannot claim credit for a candidate the budget refused.
            selected_by.setdefault(key, []).append(sig)
            if key in chosen_set:
                continue
            chosen.append(key)
            chosen_set.add(key)
            n_parts += len(members[key])

    if budget is not None and n_parts > budget:
        d.overshoot = n_parts - budget
        d.disclosures.append(
            f'budget {budget} part(s) overshot by {d.overshoot}: a block is '
            f'added whole, never half-moved')

    d.candidates = sorted(
        (Candidate(key=k, kind=kinds[k], members=members[k],
                   rows=tuple(sorted(rows[k], key=lambda r: r.signal)),
                   selected_by=tuple(selected_by.get(k, ())))
         for k in rows),
        key=lambda c: (-len(c.selected_by), c.rank_sum, -len(c.rows), c.key))
    d.selected_keys = list(chosen)
    d.selected = sorted({r for k in chosen for r in members[k]})
    return d


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

def to_json(d: Diagnosis) -> Dict[str, object]:
    return d.to_dict()


def format_text(d: Diagnosis, limit: int = 3) -> str:
    """The operator's per-signal table. Never an aggregate score.

    An aggregate verdict cannot say which of its inputs moved, which is how a
    reversed signal once sat behind a PASS for weeks. Every signal prints its
    own row, its own spread and its own top few.
    """
    out = [f'  target-select: diagnosis  (selected {len(d.selected)} part(s) '
           f'in {len(d.selected_keys)} candidate(s)'
           + (f'; budget {d.budget}' if d.budget is not None else '') + ')']
    if d.evidence is not None:
        out.append(f'    blocker evidence: {d.evidence.total_cells} reported '
                   f'cell(s) over {len(d.evidence.cells)} net(s)')
    by_sig: Dict[str, List[Tuple[str, float]]] = {}
    for c in d.candidates:
        for r in c.rows:
            by_sig.setdefault(r.signal, []).append((c.key, r.value))
    for sig in SIGNAL_ORDER:
        if sig in d.skipped:
            out.append(f'    {sig:<20} SKIPPED  {d.skipped[sig]}')
            continue
        rowset = sorted(by_sig.get(sig, ()), key=lambda kv: (-kv[1], kv[0]))
        head = '  '.join(f'{k}({v:.4g})' for k, v in rowset[:limit])
        out.append(f'    {sig:<20} {len(rowset):>3} ranked  '
                   f'{rowset[0][1]:.4g} .. {rowset[-1][1]:.4g}  {head}')
    for note in d.disclosures:
        out.append(f'    NOTE {note}')
    out.append('    NOTE three separate rankings, unioned round-robin. There '
               'is no measured')
    out.append('         exchange rate between mm, cells and pairs; rank_sum '
               'is advisory.')
    out.append(f'    NOTE efficacy {d.efficacy}')
    return '\n'.join(out)


__all__ = ['SCHEMA', 'RANK_DECIMALS', 'TOP_K', 'SIGNAL_ORDER',
           'NO_EFFICACY_CLAIM', 'BlockerEvidence', 'blocker_evidence',
           'legality_defects', 'resolve_clearance', 'ignore_net_ids',
           'make_state', 'SignalRow', 'Candidate', 'Diagnosis', 'diagnose',
           'format_text', 'to_json']
