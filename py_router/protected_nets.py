"""Protected nets: routed invariants that later chain steps must not rip (#521).

Length matching and coupled diff-pair routing produce copper whose VALUE is not
just connectivity -- meander trains and coupled P/N geometry embody an
invariant a later generic step cannot reproduce. The allwinner_h3_ddr3 chain
showed the failure: a retry step ran ``--rip-existing-nets '/DDR3 16x1/*'``
with no ``--length-match-group``, ripped the whole matched group, rerouted a
subset at natural length, and left the "matched" board 40/41 nets unmatched
(and one net stranded entirely).

The protection list lives in the sibling ``.kicad_pro`` under a tool-namespaced
key, exactly like the DRC-floor writeback: ``fix_project_for_output`` copies the
input project to each step's output, so protection flows down the chain for
free, and ``copy_board.py`` carries it as a sibling.

    {"kicad_routing_tools": {"protected_nets": {"/DDR3 16x1/SA7": "length-matched", ...}}}

**Policy** (no CLI flag, no GUI control -- deliberate, like the #498 .kicad_dru
auto-read): protection guards against COLLATERAL damage. A net named EXACTLY
(no glob metacharacters) in ``--nets`` or ``--rip-existing-nets`` is being
deliberately targeted and stays rippable; glob matches over protected nets are
filtered with a printed exclusion. Consumers:

  * route.py's ``--rip-existing-nets`` expansion (collateral rip of non-target
    nets) skips protected nets unless exactly named.
  * repair_planes' ``--rip-blocker-nets`` tap rip never selects a
    protected net as a blocker.

**Writers** (engine-side, so the GUI/AI-plan path inherits them): length/time
match groups mark their members 'length-matched'/'time-matched';
batch_route_diff_pairs marks routed coupled pair members 'diff-pair'. Writers
call ``note_protection_candidates``; the per-step persistence happens next to
the DRC-floor writeback (CLI mains, ai_plan executor) via
``consume_protection_candidates`` + ``persist_protected_nets``.
"""
import json
import os
from typing import Dict, Iterable, List, Optional, Set

PRO_NAMESPACE = "kicad_routing_tools"
PRO_KEY = "protected_nets"
IMPEDANCE_KEY = "net_impedance"
SNPC_KEY = "same_net_pad_clearance"

_GLOB_CHARS = set('*?[')

# Process-local accumulators: engines note candidates while routing; the
# writeback site (which knows the output project path) consumes them. One
# routing step runs at a time in both fronts (CLI process / GUI plan step).
_notes: Dict[str, Dict[str, object]] = {}


def _note(key: str, mapping: Dict[str, object]) -> None:
    _notes.setdefault(key, {}).update({k: v for k, v in mapping.items() if k})


def _consume(key: str) -> Dict[str, object]:
    return _notes.pop(key, {})


def note_protection_candidates(mapping: Dict[str, str]) -> None:
    """Record nets this step made protection-worthy (net name -> reason)."""
    _note(PRO_KEY, mapping)


def consume_protection_candidates() -> Dict[str, str]:
    """Return and clear the accumulated candidates (call once per step)."""
    return _consume(PRO_KEY)


def note_impedance_specs(mapping: Dict[str, dict]) -> None:
    """Record per-net impedance declarations from a step routed with
    --impedance: {net: {'ohms': 100, 'differential': True, 'pair_gap': 0.1,
    'coplanar_gap': 0.2}} -- so a later redo (rip/reroute) can recompute the
    SAME widths from the stackup instead of silently rerouting at the step's
    default width. Deliberately the DECLARATION, not the widths: widths go
    stale if the stackup changes, and copper measurement cannot distinguish
    impedance intent from necking nor recover the coplanar declaration at all
    (#486: the pour it assumes may not exist yet)."""
    _note(IMPEDANCE_KEY, mapping)


def consume_impedance_specs() -> Dict[str, dict]:
    return _consume(IMPEDANCE_KEY)


def pro_path_for_board(board_path: str) -> str:
    return os.path.splitext(board_path)[0] + '.kicad_pro'


def read_protected_nets(pro_path: str) -> Dict[str, str]:
    """Protection map from a .kicad_pro ({} when absent/unreadable)."""
    try:
        if not pro_path or not os.path.isfile(pro_path):
            return {}
        with open(pro_path, 'r', encoding='utf-8') as f:
            proj = json.load(f)
        m = (proj.get(PRO_NAMESPACE) or {}).get(PRO_KEY) or {}
        return {str(k): str(v) for k, v in m.items()} if isinstance(m, dict) else {}
    except Exception:
        return {}


def read_for_pcb_data(pcb_data, input_file: Optional[str] = None) -> Dict[str, str]:
    """Protection map for the board an engine is working on. ``input_file``
    when the caller has one; engines without it (GUI builds PCBData from the
    live board) discover the board file via PCBData.source_path (#498)."""
    path = input_file or getattr(pcb_data, 'source_path', "") or ""
    if not path:
        return {}
    return read_protected_nets(pro_path_for_board(path))


def _persist_map(pro_path: str, key: str, mapping: Dict[str, object],
                 label: str, verbose: bool = True) -> bool:
    """Merge ``mapping`` into the project's ``key`` map. Preserves every
    other key (plain json round-trip, same style as fix_kicad_drc_settings).
    No-op when the project file does not exist (the DRC writeback creates it
    first; without a project there is nothing for later steps to read)."""
    if not mapping or not pro_path or not os.path.isfile(pro_path):
        return False
    try:
        with open(pro_path, 'r', encoding='utf-8') as f:
            proj = json.load(f)
        ns = proj.setdefault(PRO_NAMESPACE, {})
        current = ns.get(key) or {}
        merged = dict(current)
        merged.update(mapping)
        if merged == current:
            return False
        ns[key] = merged
        with open(pro_path, 'w', encoding='utf-8') as f:
            json.dump(proj, f, indent=2)
        if verbose:
            added = len(merged) - len(current)
            print(f"  {label}: {len(merged)} recorded in {os.path.basename(pro_path)}"
                  f" (+{added} this step)")
        return True
    except Exception as e:
        if verbose:
            print(f"  (skipped {label} record: {e})")
        return False


def persist_protected_nets(pro_path: str, mapping: Dict[str, str],
                           verbose: bool = True) -> bool:
    return _persist_map(pro_path, PRO_KEY, mapping,
                        "Protected nets (not ripped by later steps)", verbose)


def persist_impedance_specs(pro_path: str, mapping: Dict[str, dict],
                            verbose: bool = True) -> bool:
    return _persist_map(pro_path, IMPEDANCE_KEY, mapping,
                        "Net impedance specs (reapplied on redo)", verbose)


def persist_same_net_pad_clearance(pro_path: str, value: float,
                                   verbose: bool = True) -> bool:
    """Record an ACTIVE (> 0) same-net pad via clearance (#581) so every later
    chain step keeps its vias off same-net pads too. Scalar, not a map: one
    board-wide assembly constraint. No-op for <= 0 (-1 = via-in-pad allowed,
    the default; 0 keeps its legacy stitching-only meaning and, per the
    compat contract, must not change any other step's behavior) or when the
    project file is absent."""
    if value is None or value <= 0 or not pro_path or not os.path.isfile(pro_path):
        return False
    try:
        with open(pro_path, 'r', encoding='utf-8') as f:
            proj = json.load(f)
        ns = proj.setdefault(PRO_NAMESPACE, {})
        if ns.get(SNPC_KEY) == value:
            return False
        ns[SNPC_KEY] = value
        with open(pro_path, 'w', encoding='utf-8') as f:
            json.dump(proj, f, indent=2)
        if verbose:
            print(f"  Same-net pad via clearance {value:g}mm recorded in "
                  f"{os.path.basename(pro_path)} (later steps keep vias off "
                  f"same-net pads)")
        return True
    except Exception as e:
        if verbose:
            print(f"  (skipped same-net pad clearance record: {e})")
        return False


def read_same_net_pad_clearance(pro_path: str) -> float:
    """The persisted #581 clearance, or -1.0 when absent/unreadable."""
    try:
        if not pro_path or not os.path.isfile(pro_path):
            return -1.0
        with open(pro_path, 'r', encoding='utf-8') as f:
            proj = json.load(f)
        v = (proj.get(PRO_NAMESPACE) or {}).get(SNPC_KEY)
        return float(v) if isinstance(v, (int, float)) and float(v) > 0 else -1.0
    except Exception:
        return -1.0


def read_snpc_for_pcb_data(pcb_data, input_file: Optional[str] = None) -> float:
    """#581 clearance for the board an engine is working on (same discovery
    rule as read_for_pcb_data: input_file, else PCBData.source_path)."""
    path = input_file or getattr(pcb_data, 'source_path', "") or ""
    if not path:
        return -1.0
    return read_same_net_pad_clearance(pro_path_for_board(path))


def read_impedance_specs(pro_path: str) -> Dict[str, dict]:
    """Per-net impedance declarations from a .kicad_pro ({} when absent)."""
    try:
        if not pro_path or not os.path.isfile(pro_path):
            return {}
        with open(pro_path, 'r', encoding='utf-8') as f:
            proj = json.load(f)
        m = (proj.get(PRO_NAMESPACE) or {}).get(IMPEDANCE_KEY) or {}
        return {str(k): dict(v) for k, v in m.items()
                if isinstance(v, dict)} if isinstance(m, dict) else {}
    except Exception:
        return {}


def read_impedance_for_pcb_data(pcb_data, input_file: Optional[str] = None) -> Dict[str, dict]:
    path = input_file or getattr(pcb_data, 'source_path', "") or ""
    if not path:
        return {}
    return read_impedance_specs(pro_path_for_board(path))


# --- #678: pour-served balls are a COMMITMENT, carried down the chain -------
#
# The BGA fanout's pour-direct serves a plane-net ball by fill contact instead
# of a drop via (measured win: 38 rail balls, 18 -> 0 gap vias). That is a
# PROMISE about a pour that a later route step can carve up: the ball's fill
# island gets cut off the sourced region and the refill ships it split. The
# fanout used to record only a per-net COUNT ('pour': n), in a process-local
# report, so no later step could tell WHICH balls were promised. The list now
# lives here, keyed by "REF.PAD" so it survives re-parses and both parse paths:
#
#     {"kicad_routing_tools": {"pour_served_pads":
#         {"U3.E5": {"net": "GND", "layer": "F.Cu", "how": "pour"}, ...}}}
#
# `how` is 'pour' (fill contact at the ball) or 'pour_track' (#652: a short
# same-layer track from the ball into the fill -- the same promise, one track
# longer). The route step's in-run plane finalize reads it back and audits
# every promised ball against the exact fill after routing (pour_promise.py).
# There is deliberately no CLI flag and no GUI control: a promise is a
# commitment, and defending it is default behaviour.
POUR_SERVED_KEY = "pour_served_pads"


def pour_served_key(component_ref: str, pad_number: str) -> str:
    return f"{component_ref}.{pad_number}"


def note_pour_served_pads(mapping: Dict[str, dict]) -> None:
    """Record balls this fanout step promised to serve by fill contact
    ({"REF.PAD": {"net": name, "layer": layer, "how": 'pour'|'pour_track'}})."""
    _note(POUR_SERVED_KEY, mapping)


def consume_pour_served_pads() -> Dict[str, dict]:
    return _consume(POUR_SERVED_KEY)


def persist_pour_served_pads(pro_path: str, mapping: Dict[str, dict],
                             verbose: bool = True) -> bool:
    return _persist_map(pro_path, POUR_SERVED_KEY, mapping,
                        "Pour-served balls (#678, defended by later route steps)",
                        verbose)


def read_pour_served_pads(pro_path: str) -> Dict[str, dict]:
    """The promise list from a .kicad_pro ({} when absent/unreadable)."""
    try:
        if not pro_path or not os.path.isfile(pro_path):
            return {}
        with open(pro_path, 'r', encoding='utf-8') as f:
            proj = json.load(f)
        m = (proj.get(PRO_NAMESPACE) or {}).get(POUR_SERVED_KEY) or {}
        return {str(k): dict(v) for k, v in m.items()
                if isinstance(v, dict)} if isinstance(m, dict) else {}
    except Exception:
        return {}


def read_pour_served_for_pcb_data(pcb_data, input_file: Optional[str] = None
                                  ) -> Dict[str, dict]:
    """Promise list for the board an engine is working on (same discovery
    rule as read_for_pcb_data: input_file, else PCBData.source_path)."""
    path = input_file or getattr(pcb_data, 'source_path', "") or ""
    if not path:
        return {}
    return read_pour_served_pads(pro_path_for_board(path))


def locked_net_names(pcb_data) -> Set[str]:
    """Nets with any KiCad-locked segment or via. The user pinned that copper;
    rip machinery must never strip the net (a partial rip would strand the
    locked fragments). Read straight from the board -- no .kicad_pro entry."""
    ids = {s.net_id for s in pcb_data.segments if getattr(s, 'locked', False)}
    ids |= {v.net_id for v in pcb_data.vias if getattr(v, 'locked', False)}
    ids.discard(0)
    return {pcb_data.nets[i].name for i in ids
            if i in pcb_data.nets and pcb_data.nets[i].name}


def cached_protection_map(pcb_data, input_file: Optional[str] = None) -> Dict[str, str]:
    """protection_map(), memoized per pcb_data for the in-run rip ladders.

    The phase-3 tap ladder and the blocking analyser consult protection on
    every candidate, and protection_map re-reads the sibling .kicad_pro each
    time. The map cannot change mid-run (the .pro is read-only to the engine
    and locked copper does not move), so one resolve per board is correct.
    """
    m = getattr(pcb_data, '_protection_map_memo', None)
    if m is None:
        m = protection_map(pcb_data, input_file)
        try:
            pcb_data._protection_map_memo = m
        except Exception:
            pass
    return m


def protection_map(pcb_data, input_file: Optional[str] = None) -> Dict[str, str]:
    """Full protection map for a board: the .kicad_pro list plus nets with
    KiCad-locked copper. 'locked' wins where both apply -- unlike the .pro
    reasons it has NO exact-name override (locked means never)."""
    m = read_for_pcb_data(pcb_data, input_file)
    m.update({n: 'locked' for n in locked_net_names(pcb_data)})
    return m






def exact_names(patterns: Optional[Iterable[str]]) -> Set[str]:
    """The non-glob entries of a pattern list: naming a net exactly is the
    deliberate-override signal that lifts its protection for this step."""
    if not patterns:
        return set()
    return {p for p in patterns if p and not (_GLOB_CHARS & set(p))}


def stash_rip_overrides(pcb_data, patterns: Optional[Iterable[str]]) -> Set[str]:
    """Record the exact-name rip overrides on pcb_data so the IN-RUN ladders
    can honor them (run-6 z2 fix). The pre-run filters (--rip-existing-nets /
    --force-reroute) already lift 'user' protection for exactly-named nets,
    but the in-run ladders re-consult cached_protection_map, which still
    lists them -- so the phase-3 tap cascade refused a net the operator had
    explicitly named ('protected_skipped {"phase3 tap cascade":
    {USB_DM_R: user}}' while --rip-existing-nets named it). 'locked' is
    never overridable, here or anywhere."""
    names = exact_names(patterns)
    if names:
        pcb_data._rip_override_names = set(
            getattr(pcb_data, '_rip_override_names', None) or set()) | names
    return getattr(pcb_data, '_rip_override_names', None) or set()


def rip_override_names(pcb_data) -> Set[str]:
    """The exact-name rip overrides stashed for this run (empty set if none)."""
    return getattr(pcb_data, '_rip_override_names', None) or set()




# What the last run's rip filters refused, and why: {context: {net: reason}}.
# The print below is for a human reading a log; a PROGRAM driving the router
# cannot see it, and the router's own failure hint tells that program to retry
# with --rip-existing-nets naming exactly the net that was just refused. A
# caller following that advice loops forever. route.py drains this into
# JSON_SUMMARY['protected_skipped'] so the refusal is machine-readable, and so a
# caller can tell "name it exactly to override" from "locked, no override ever".
PROTECTED_SKIPPED: Dict[str, Dict[str, str]] = {}


def clear_skipped() -> None:
    """Reset the record. route.py calls this once per run."""
    PROTECTED_SKIPPED.clear()


def filter_rippable_names(names: List[str], protected: Dict[str, str],
                          override_patterns: Optional[Iterable[str]] = None,
                          context: str = "rip-up") -> List[str]:
    """Drop protected names (minus exact-name overrides), printing what was
    excluded and why. Returns the surviving names in input order. A net whose
    reason is 'locked' (KiCad-locked copper) has NO override -- ever."""
    if not protected:
        return list(names)
    overrides = exact_names(override_patterns)
    kept, blocked = [], []
    for n in names:
        if n in protected and (protected[n] == 'locked' or n not in overrides):
            blocked.append(n)
        else:
            kept.append(n)
    if blocked:
        PROTECTED_SKIPPED.setdefault(context, {}).update(
            {n: protected[n] for n in blocked})
        by_reason: Dict[str, List[str]] = {}
        for n in blocked:
            by_reason.setdefault(protected[n], []).append(n)
        det = '; '.join(f"{r}: {', '.join(ns[:4])}{'...' if len(ns) > 4 else ''}"
                        for r, ns in sorted(by_reason.items()))
        print(f"  {len(blocked)} PROTECTED net(s) excluded from {context} ({det})"
              f" -- name a net exactly (no glob) to override")
    return kept
