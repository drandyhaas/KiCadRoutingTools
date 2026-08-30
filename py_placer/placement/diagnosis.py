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
    pre-route predictors against routed `blocking` over 6 boards -- 120
    placements planned, 119 of which produced a routed result, and every N in
    that document is over the rows that produced a defined rho rather than over
    the planned count. The legality counts are what passed: pad and body pairs at
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

#: Fixed and declared, never chosen per run.
#:
#: HOW MUCH IT DECIDES, stated exactly, because the comfortable version of this
#: sentence is false. UNBUDGETED, permuting it changes only the ORDER of
#: `selected_keys`; the selected part SET is invariant. UNDER A BUDGET it can
#: change the set outright -- measured on the unit fixture, `budget=1` selects
#: `sheet:mag` (parts A1, A2) under the declared order and `C1` under an order
#: led by `legality_pairs`: disjoint. And `place_route_loop` budgets EVERY
#: round, so on a board where the pin budget bites, the head signal wins the
#: last slot. That is the price of refusing to combine the signals into one
#: score, and it is disclosed rather than hidden behind "it only breaks ties".
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
                'unique_cells': sum(self.unique.values()),
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
                      clearance: Optional[float] = None) -> Tuple[float, str]:
    """`(mm, source)` -- the board's own Default-netclass clearance if it has one.

    Returns the SOURCE as well as the number, and every caller must carry it,
    because the honest version of "never a guessed round number" is narrower
    than it sounds: MOST tracked boards have no sibling `.kicad_pro` at all --
    including all six that `docs/placement-predictors.md` measured on -- so the
    value is `routing_defaults.CLEARANCE`, a constant, on exactly the boards the
    only measured signal was measured on. It is load-bearing (glasgow_revC
    grades 23 defect pairs at 0.15 mm and 29 at 0.30), so a consumer that
    cannot see where it came from cannot judge the ranking it produced.

    Sources: `netclass` (the board said so), `default` (nothing did), or
    `caller`. `lock_advisor.advise_locks` resolves the same chain for its
    pad-conflict demotion; the one deliberate difference is that an EXPLICIT
    0.0 is honoured here rather than silently replaced -- a caller asking to
    grade at zero gets zero, and finds nothing, which is at least the thing
    they asked for.
    """
    if clearance is not None:
        return float(clearance), 'caller'
    if pcb_file:
        try:
            from list_nets import board_default_netclass_clearance
            val = board_default_netclass_clearance(pcb_file)
            if val:
                return float(val), 'netclass'
        except Exception:
            pass
    try:
        from routing_defaults import CLEARANCE as _DEF
        return float(_DEF), 'default'
    except Exception:
        return 0.25, 'default'


def legality_defects(pcb_data, clearance: Optional[float] = None,
                     pcb_file: Optional[str] = None) -> Dict[str, object]:
    """`{pairs: [(a, b, kind)], clearance: mm, notes: [...]}`. Reads the board.

    The one function here that touches the file, kept apart from `diagnose` so
    the ranking stays pure. Kinds, all per-REF-PAIR and therefore attributable:

      pad_clearance        `grade_pad_legality(..., worst_n=0)['worst']`
      body_overlap         `grade_body_overlap(...)['blocking_pairs']`
      courtyard_blocking   ...`['courtyard_blocking_pairs']`

    A grader that RAISES is recorded in `notes` and contributes no pairs, and
    so is a grader that could not JUDGE part of the board: `fab_unjudged_refs`
    (no .Fab outline to compare) and `courtyard_synthetic_refs` (a courtyard
    invented from the pad box) are parts whose absence from the pair list means
    "not looked at", not "clean". Reporting "no findings" for a census that
    could not run on half the board is the silence this repo keeps filing bugs
    about.
    """
    from placement import legality as _leg
    clr, clr_source = resolve_clearance(pcb_file, clearance)
    pairs: List[Tuple[str, str, str]] = []
    notes: List[str] = []
    if clr_source == 'default':
        notes.append(
            f'graded at {clr} mm from routing_defaults, NOT from the board: '
            f'it declares no Default netclass clearance (no sibling '
            f'.kicad_pro). The pair count moves with this number')

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
        # What the census could NOT judge. These are the real "did not run"
        # channels `grade_body_overlap` exposes -- it has no error key, and a
        # branch reading one would be dead code pretending to be a guarantee.
        unjudged = list(bo.get('fab_unjudged_refs') or ())
        if unjudged:
            notes.append(
                f'{len(unjudged)} part(s) had no .Fab outline to judge and are '
                f'absent from the pair list because they were NOT LOOKED AT, '
                f'not because they are clean: '
                f'{", ".join(sorted(unjudged)[:6])}'
                + ('...' if len(unjudged) > 6 else ''))
        synthetic = list(bo.get('courtyard_synthetic_refs') or ())
        if synthetic:
            notes.append(
                f'{len(synthetic)} part(s) have a SYNTHETIC courtyard (the pad '
                f'box, carrying no courtyard margin), so their blocking pairs '
                f'are excluded from the census by legality.py')
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
    return {'pairs': out, 'clearance': clr, 'clearance_source': clr_source,
            'notes': notes}


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
    #: Set when the CALLER's own limits admitted nothing, which is a different
    #: fact from "no signal could be defined" and must not borrow its words.
    refusal: str = ''
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
        if self.refusal:
            return self.refusal
        if not self.signals_defined:
            return ('no signal could be defined: '
                    + '; '.join(f'{s}: {self.skipped[s]}' for s in SIGNAL_ORDER
                                if s in self.skipped))
        if self.candidates:
            return ('every defined signal ranked candidates, but none reached '
                    'the selection')
        return 'every defined signal ranked nothing above zero'

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
                'refusal': self.refusal,
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

    # ---- candidate universe: every block, plus every MOVABLE part in no block.
    #
    # A KiCad-locked part is not a candidate. `nets_to_refs` does offer them
    # (quench filters them later, at no cost, because the pin selector has no
    # budget), but a ranking that spends a top-k slot AND a budget unit on a
    # part nothing can move has spent them on nothing. `QuenchState` already
    # knows: `Part.locked`.
    locked_parts = sorted(r for r, p in parts.items()
                          if getattr(p, 'locked', False))
    parts = {r: p for r, p in parts.items() if r not in set(locked_parts)}
    if locked_parts:
        d.disclosures.append(
            f'{len(locked_parts)} KiCad-locked part(s) are not candidates: '
            f'{", ".join(locked_parts[:6])}'
            f'{"..." if len(locked_parts) > 6 else ""}')

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
        if ref in members:
            # A block NAMED like a part. `derive_groups` cannot produce one
            # (its keys are all `kicad:`/`sheet:`/`net:`/`decap:`-prefixed), but
            # this is a public function and a caller may pass any dict. Silently
            # overwriting the block would delete its members from the universe
            # while `owner()` still credited their evidence to the surviving
            # single-part candidate -- so moving the selection could not address
            # the evidence that selected it.
            d.disclosures.append(
                f'candidate name collision: a block and a part are both named '
                f'{ref!r}; the BLOCK is kept and the loose part is not a '
                f'candidate of its own')
            continue
        members[ref] = (ref,)
        kinds[ref] = 'part'

    def owner(ref: str) -> Optional[str]:
        key = block_of.get(ref, ref)
        return key if key in members else None

    values: Dict[str, Dict[str, float]] = {s: {} for s in SIGNAL_ORDER}
    details: Dict[str, Dict[str, Dict]] = {s: {} for s in SIGNAL_ORDER}

    # ---- signal 1: connectivity-centroid displacement (blocks only)
    #
    # The blocks handed to `block_displacements` are the CANDIDATE ones, not
    # the raw argument. A block whose every member is KiCad-locked has no
    # candidate entry (locked parts were cut above), and passing it anyway
    # returns a row for a key nothing else knows about -- which reached the
    # candidate sort as a KeyError. Measured on glasgow_revC, whose
    # `decap:U3` is two locked parts, under the very `--group-by auto,decap`
    # the README prints as its example.
    ranked_blocks = {n: list(members[n]) for n in blocks
                     if kinds.get(n) == 'block'}
    if not blocks:
        d.skipped['block_displacement'] = _NO_BLOCKS
    elif not ranked_blocks:
        d.skipped['block_displacement'] = (
            'every derived block is wholly KiCad-locked, so none of them is a '
            'candidate this ranking could offer')
    else:
        bd = block_displacements(state, ranked_blocks, sorted(ignored),
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
            unknown_cells = 0
            unknown_nets = []
            for net, cells in sorted(ev.cells.items()):
                nid = name_to_id.get(net)
                if nid is None:
                    # NOT a rail, and not ignored: a name this board does not
                    # carry. Real input -- route.py and the plane repair rename
                    # nets between chain steps -- and filing it under the rail
                    # cut would report a reason that is simply false.
                    unknown_cells += cells
                    unknown_nets.append(net)
                    continue
                if nid in ignored:
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
            if unknown_cells:
                d.disclosures.append(
                    f'blocker_cells: {unknown_cells} cell(s) name '
                    f'{len(unknown_nets)} net(s) this board does not carry '
                    f'({", ".join(sorted(unknown_nets)[:4])}'
                    f'{"..." if len(unknown_nets) > 4 else ""}) -- the router '
                    f'and the board disagree about the net list, which a '
                    f'rename between chain steps will do')
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
    # A caller can ask for nothing. Say so by name rather than letting the
    # empty selection be reported as "no signal ranked anything above zero",
    # which is a different fact and was measured false in exactly these cases.
    if top_k is not None and top_k < 1:
        d.refusal = f'top_k={top_k} admits no candidate from any signal'
    elif budget is not None and budget < 1:
        d.refusal = f'budget={budget} part(s) admits no candidate'

    pools = ({} if d.refusal else
             {sig: [k for k, _ in ranked[sig][:top_k]] for sig in ranked})
    chosen: List[str] = []
    chosen_set: set = set()
    selected_by: Dict[str, List[str]] = {}
    picked: set = set()          # deduplicated PARTS, which is what a budget caps
    exhausted = False
    while not exhausted and any(pools.get(s) for s in SIGNAL_ORDER):
        for sig in SIGNAL_ORDER:
            pool = pools.get(sig)
            if not pool:
                continue
            if budget is not None and len(picked) >= budget:
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
            # Deduplicated, not summed: two blocks may name the same ref (this
            # function takes any dict), and counting a shared member twice
            # reports an overshoot that did not happen.
            picked.update(members[key])

    if budget is not None and chosen and len(picked) > budget:
        d.overshoot = len(picked) - budget
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
    if d.refusal:
        out.append(f'    REFUSED  {d.refusal}')
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
        if not rowset:
            out.append(f'    {sig:<20} defined, but nothing reached the report')
            continue
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
