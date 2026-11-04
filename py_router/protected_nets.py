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
free, and ``copy_board.py`` carries it as a sibling. The GUI keeps the same
namespace on the board itself as well (``tool_records``), because KiCad's own
project save drops what was written into the ``.kicad_pro`` after load; every
reader here takes the newer copy.

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

import tool_records

PRO_NAMESPACE = "kicad_routing_tools"
PRO_KEY = "protected_nets"
IMPEDANCE_KEY = "net_impedance"
SNPC_KEY = "same_net_pad_clearance"
POWER_WIDTHS_KEY = "power_widths"
#: #1503: the pairs a route_diff step length-matched P to N, keyed by the P
#: net: {'n': N net, 'tolerance_mm': the skew it matched to, 'source': 'cli'
#: | 'dru' | 'default'} -- what the graders hold a pair to when no
#: .kicad_dru skew rule binds it (`kicad_dru.recorded_intra_pair_rules`).
INTRA_KEY = "intra_pair_tolerance"

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


def note_power_widths(mapping: Dict[str, float]) -> None:
    """Record the track widths this step was ASKED to give power nets
    (--power-nets / --power-nets-widths, resolved to net names): {net: mm}.
    board_score reads it back and measures the board against it without being
    handed a width file -- disclosed (`net_widths.requested_under`), not
    counted: the router narrows a power route that does not fit and says so."""
    _note(POWER_WIDTHS_KEY, mapping)


def consume_power_widths() -> Dict[str, float]:
    return _consume(POWER_WIDTHS_KEY)


def note_intra_pair_tolerances(mapping: Dict[str, dict]) -> None:
    """Record the pairs this step length-matched and the tolerance each was
    matched to (#1503)."""
    _note(INTRA_KEY, mapping)


def consume_intra_pair_tolerances() -> Dict[str, dict]:
    return _consume(INTRA_KEY)


def persist_intra_pair_tolerances(pro_path: str, mapping: Dict[str, dict],
                                  verbose: bool = True, board=None) -> bool:
    return _persist_map(pro_path, INTRA_KEY, mapping,
                        "Intra-pair tolerances (graded where no .kicad_dru skew "
                        "rule binds the pair)", verbose, board=board)


def read_intra_pair_tolerances(pro_path: str) -> Dict[str, dict]:
    """The recorded intra-pair tolerances for the board beside a .kicad_pro
    ({} when absent)."""
    return _dict_map_from(_namespace(pro_path), INTRA_KEY)


def pro_path_for_board(board_path: str) -> str:
    return os.path.splitext(board_path)[0] + '.kicad_pro'


def _namespace(pro_path: str) -> dict:
    """The records for the board beside ``pro_path``: the newer of the
    project's copy and the board file's (tool_records)."""
    try:
        return tool_records.effective_namespace(pro_path) if pro_path else {}
    except Exception:
        return {}


def _pcb_data_namespace(pcb_data, input_file: Optional[str] = None) -> dict:
    """The records for the board an engine is working on: the LIVE board's
    when the PCBData came from pcbnew (``live_records_provider`` -- what the
    session wrote, saved or not), else the file's: ``input_file`` when the
    caller has one, else PCBData.source_path (#498)."""
    provider = getattr(pcb_data, 'live_records_provider', None)
    if callable(provider):
        try:
            ns = provider()
            if isinstance(ns, dict):
                return ns
        except Exception:
            pass
    path = input_file or getattr(pcb_data, 'source_path', "") or ""
    return _namespace(pro_path_for_board(path)) if path else {}


def _protected_from(ns: dict) -> Dict[str, str]:
    m = (ns or {}).get(PRO_KEY) or {}
    return {str(k): str(v) for k, v in m.items()} if isinstance(m, dict) else {}


def read_protected_nets(pro_path: str) -> Dict[str, str]:
    """Protection map for the board beside a .kicad_pro ({} when absent)."""
    return _protected_from(_namespace(pro_path))


def read_for_pcb_data(pcb_data, input_file: Optional[str] = None) -> Dict[str, str]:
    """Protection map for the board an engine is working on. ``input_file``
    when the caller has one; engines without it (GUI builds PCBData from the
    live board) discover the board file via PCBData.source_path (#498)."""
    return _protected_from(_pcb_data_namespace(pcb_data, input_file))


def _merge_map(ns: dict, key: str, mapping: Dict[str, object]):
    """Merge ``mapping`` into ``ns[key]``; ``(changed, before, after)`` sizes."""
    current = ns.get(key) or {}
    merged = dict(current)
    merged.update(mapping)
    if merged == current:
        return False, len(current), len(merged)
    ns[key] = merged
    return True, len(current), len(merged)


def _persist_map(pro_path: str, key: str, mapping: Dict[str, object],
                 label: str, verbose: bool = True, board=None) -> bool:
    """Merge ``mapping`` into the board's ``key`` record.

    FILE (``board`` None, every CLI): into the project's map, starting from
    the newer of its copy and the board file's (tool_records.load_project) and
    one revision past it. Preserves every other key (plain json round-trip,
    same style as fix_kicad_drc_settings). No-op when the project file does
    not exist (the DRC writeback creates it first; without a project there is
    nothing for later steps to read).

    LIVE (``board`` a pcbnew board, the GUI): into the live board's records,
    which go to its project when it has one and onto the board itself always
    -- the copy KiCad's own save keeps (tool_records). A project-less or
    unsaved board records too: the board copy is where it can."""
    if not mapping:
        return False
    try:
        if board is not None:
            sizes = {}

            def _apply(ns):
                changed, before, after = _merge_map(ns, key, mapping)
                sizes['n'] = (before, after)
                return changed
            changed = tool_records.update_live_namespace(board, _apply)
            if changed and verbose:
                before, after = sizes['n']
                print(f"  {label}: {after} recorded on the board"
                      f" (+{after - before} this step)")
            return changed
        if not pro_path or not os.path.isfile(pro_path):
            return False
        proj = tool_records.load_project(pro_path)
        ns = proj.setdefault(PRO_NAMESPACE, {})
        changed, before, after = _merge_map(ns, key, mapping)
        if not changed:
            return False
        tool_records.stamp_revision(proj)
        with open(pro_path, 'w', encoding='utf-8') as f:
            json.dump(proj, f, indent=2)
        if verbose:
            print(f"  {label}: {after} recorded in {os.path.basename(pro_path)}"
                  f" (+{after - before} this step)")
        return True
    except Exception as e:
        if verbose:
            print(f"  (skipped {label} record: {e})")
        return False


def persist_protected_nets(pro_path: str, mapping: Dict[str, str],
                           verbose: bool = True, board=None) -> bool:
    return _persist_map(pro_path, PRO_KEY, mapping,
                        "Protected nets (not ripped by later steps)", verbose,
                        board=board)


def persist_impedance_specs(pro_path: str, mapping: Dict[str, dict],
                            verbose: bool = True, board=None) -> bool:
    return _persist_map(pro_path, IMPEDANCE_KEY, mapping,
                        "Net impedance specs (reapplied on redo)", verbose,
                        board=board)


def persist_power_widths(pro_path: str, mapping: Dict[str, float],
                         verbose: bool = True, board=None) -> bool:
    return _persist_map(pro_path, POWER_WIDTHS_KEY,
                        {k: round(float(v), 4) for k, v in mapping.items()},
                        "Power-net widths (measured by board_score)", verbose,
                        board=board)


def persist_same_net_pad_clearance(pro_path: str, value: float,
                                   verbose: bool = True, board=None) -> bool:
    """Record an ACTIVE (> 0) same-net pad via clearance (#581) so every later
    chain step keeps its vias off same-net pads too. Scalar, not a map: one
    board-wide assembly constraint. No-op for <= 0 (-1 = via-in-pad allowed,
    the default; 0 keeps its legacy stitching-only meaning and, per the
    compat contract, must not change any other step's behavior) or when the
    project file is absent.

    ``board`` (the GUI): into the live board's records instead -- its project
    when it has one and the board itself always, the copy KiCad's own save
    keeps (tool_records), as _persist_map's live branch."""
    if value is None or value <= 0:
        return False
    if board is not None:
        try:
            def _apply(ns):
                if ns.get(SNPC_KEY) == value:
                    return False
                ns[SNPC_KEY] = value
                return True
            changed = tool_records.update_live_namespace(board, _apply)
            if changed and verbose:
                print(f"  Same-net pad via clearance {value:g}mm recorded on "
                      f"the board (later steps keep vias off same-net pads)")
            return changed
        except Exception as e:
            if verbose:
                print(f"  (skipped same-net pad clearance record: {e})")
            return False
    if not pro_path or not os.path.isfile(pro_path):
        return False
    try:
        proj = tool_records.load_project(pro_path)
        ns = proj.setdefault(PRO_NAMESPACE, {})
        if ns.get(SNPC_KEY) == value:
            return False
        ns[SNPC_KEY] = value
        tool_records.stamp_revision(proj)
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


def _snpc_from(ns: dict) -> float:
    v = (ns or {}).get(SNPC_KEY)
    return float(v) if isinstance(v, (int, float)) and float(v) > 0 else -1.0


def read_same_net_pad_clearance(pro_path: str) -> float:
    """The persisted #581 clearance, or -1.0 when absent/unreadable."""
    return _snpc_from(_namespace(pro_path))


def read_snpc_for_pcb_data(pcb_data, input_file: Optional[str] = None) -> float:
    """#581 clearance for the board an engine is working on (same discovery
    rule as read_for_pcb_data: input_file, else PCBData.source_path)."""
    return _snpc_from(_pcb_data_namespace(pcb_data, input_file))


def _dict_map_from(ns: dict, key: str) -> Dict[str, dict]:
    m = (ns or {}).get(key) or {}
    return {str(k): dict(v) for k, v in m.items()
            if isinstance(v, dict)} if isinstance(m, dict) else {}


def read_impedance_specs(pro_path: str) -> Dict[str, dict]:
    """Per-net impedance declarations for the board beside a .kicad_pro
    ({} when absent)."""
    return _dict_map_from(_namespace(pro_path), IMPEDANCE_KEY)


def read_impedance_for_pcb_data(pcb_data, input_file: Optional[str] = None) -> Dict[str, dict]:
    return _dict_map_from(_pcb_data_namespace(pcb_data, input_file), IMPEDANCE_KEY)


def _widths_from(ns: dict) -> Dict[str, float]:
    m = (ns or {}).get(POWER_WIDTHS_KEY) or {}
    out: Dict[str, float] = {}
    if isinstance(m, dict):
        for k, v in m.items():
            try:
                out[str(k)] = float(v)
            except (TypeError, ValueError):
                pass
    return out


def read_power_widths(pro_path: str) -> Dict[str, float]:
    """Power-net widths the board's routing steps were asked for
    ({} when none were recorded)."""
    return _widths_from(_namespace(pro_path))


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
                             verbose: bool = True, board=None) -> bool:
    return _persist_map(pro_path, POUR_SERVED_KEY, mapping,
                        "Pour-served balls (#678, defended by later route steps)",
                        verbose, board=board)


def read_pour_served_pads(pro_path: str) -> Dict[str, dict]:
    """The promise list for the board beside a .kicad_pro ({} when absent)."""
    return _dict_map_from(_namespace(pro_path), POUR_SERVED_KEY)


def read_pour_served_for_pcb_data(pcb_data, input_file: Optional[str] = None
                                  ) -> Dict[str, dict]:
    """Promise list for the board an engine is working on (same discovery
    rule as read_for_pcb_data: input_file, else PCBData.source_path)."""
    return _dict_map_from(_pcb_data_namespace(pcb_data, input_file),
                          POUR_SERVED_KEY)


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


def held_net_reasons(pcb_data) -> Dict[str, str]:
    """{net name: `sense:<REF>.<PAD>`} for nets carrying copper a `sense:`
    group holds (#1279, Segment.held_by). Read straight from the board."""
    out: Dict[str, str] = {}
    for it in list(pcb_data.segments) + list(pcb_data.vias):
        held = getattr(it, 'held_by', '')
        net = pcb_data.nets.get(it.net_id) if held and it.net_id else None
        if net is not None and net.name:
            out.setdefault(net.name, held)
    return out


def is_sense_reason(reason: Optional[str]) -> bool:
    """A protection reason naming the `sense:` group that holds the net's
    copper (#1279)."""
    return bool(reason) and reason.startswith('sense:')


def never_lifts(reason: Optional[str]) -> bool:
    """A reason no exact-name override lifts: KiCad-locked copper (#1261) and
    copper a `sense:` group holds (#1279) -- naming the net would strip it."""
    return reason == 'locked' or is_sense_reason(reason)


#: The hint for a net whose pre-existing copper a `sense:` group holds.
SENSE_HINT = ("copper held by a `sense:` group (#1279) is never stripped and "
              "has no override; ungroup or rename the group to release it")


def protection_map(pcb_data, input_file: Optional[str] = None) -> Dict[str, str]:
    """Full protection map for a board: the .kicad_pro list, nets whose
    copper a `sense:` group holds (reason: the group's name), and nets with
    KiCad-locked copper. 'locked' wins where several apply. Neither 'locked'
    nor a `sense:` reason has an exact-name override (`never_lifts`). A
    `sense:` reason bars whole-net strips of the net's PRE-EXISTING copper
    only: the in-run ladders rip a routed result, never held copper
    (rip_refusal)."""
    m = read_for_pcb_data(pcb_data, input_file)
    m.update(held_net_reasons(pcb_data))
    m.update({n: 'locked' for n in locked_net_names(pcb_data)})
    return m






def effective_protection_map(pcb_data, input_file: Optional[str] = None
                             ) -> Dict[str, str]:
    """protection_map() minus the nets this run's operator named EXACTLY
    (the stashed rip overrides, `stash_rip_overrides`): the ONE protected set
    a run reads once its main pass has lifted the overrides (#1395). The
    plane finalize's blocker rip-up, its nested sub-runs and the failure hints
    all see the net the main pass made rippable as rippable, instead of
    refusing it as "protected, cannot rip". 'locked' and `sense:` reasons
    have no override (`never_lifts`) and stay in the map."""
    m = protection_map(pcb_data, input_file)
    lifted = rip_override_names(pcb_data)
    if not lifted:
        return m
    return {n: r for n, r in m.items() if never_lifts(r) or n not in lifted}


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


def rip_refusal(pcb_data, net_id: int, context: str = "in-run rip ladder",
                diff_pair_by_net_id: Optional[Dict] = None) -> Optional[str]:
    """Why the IN-RUN rip machinery must leave `net_id` alone, or None.

    The one predicate for every place a run picks a net to rip as somebody
    else's blocker (#1261): filter_rippable_blockers, and every pick that does
    not go through it -- the via-in-pad decline blame, the N>1 re-analysis
    picks, the stub-swap rescue's swap-blocker rip, the diff-pair graze, probe
    and fallback rips, the victim-priority squatter rip. Same rule as the
    pre-run filters: 'locked' (KiCad-locked copper) never lifts; any other
    reason lifts for a net the operator named exactly (stash_rip_overrides).
    A `sense:` reason (#1279) does not refuse here: these ladders rip a
    routed RESULT, whose removal skips held copper
    (remove_route_from_pcb_data), so the net's routes of this run stay
    rippable -- unless the .kicad_pro also protects the net, whose rule then
    applies as usual.
    A diff pair rips as a unit, so its partner's protection refuses it too.
    A refusal is recorded in PROTECTED_SKIPPED under `context` (JSON_SUMMARY).
    """
    if pcb_data is None:
        return None
    protected = cached_protection_map(pcb_data)
    if not protected:
        return None
    ids = [net_id]
    if diff_pair_by_net_id and net_id in diff_pair_by_net_id:
        _pair = diff_pair_by_net_id[net_id][1]
        ids = [i for i in (getattr(_pair, 'p_net_id', None),
                           getattr(_pair, 'n_net_id', None))
               if i is not None] or ids
    for nid in ids:
        net = pcb_data.nets.get(nid)
        reason = in_run_rip_reason(pcb_data, net.name if net else None)
        if reason:
            PROTECTED_SKIPPED.setdefault(context, {})[net.name] = reason
            return reason
    return None


def in_run_rip_reason(pcb_data, name: Optional[str]) -> Optional[str]:
    """rip_refusal's rule for one net name, without recording it: the reason
    an in-run ladder must not rip the net's routed result, or None."""
    if not name:
        return None
    reason = cached_protection_map(pcb_data).get(name)
    if is_sense_reason(reason):
        reason = _cached_project_reasons(pcb_data).get(name)
    if not reason:
        return None
    if reason != 'locked' and name in rip_override_names(pcb_data):
        return None
    return reason


def _cached_project_reasons(pcb_data) -> Dict[str, str]:
    """The .kicad_pro protection list alone, memoised like
    cached_protection_map (a held net's own project reason, #1279)."""
    m = getattr(pcb_data, '_project_protection_memo', None)
    if m is None:
        m = read_for_pcb_data(pcb_data)
        try:
            pcb_data._project_protection_memo = m
        except Exception:
            pass
    return m




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
    reason is 'locked' (KiCad-locked copper) or a `sense:` group (#1279) has
    NO override -- ever (`never_lifts`)."""
    if not protected:
        return list(names)
    overrides = exact_names(override_patterns)
    kept, blocked = [], []
    for n in names:
        if n in protected and (never_lifts(protected[n]) or n not in overrides):
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
        # The override hint only for the reasons that HAVE one (#1192): a
        # 'locked' net was refused even if the caller named it exactly.
        hints = []
        if any(not never_lifts(r) for r in by_reason):
            hints.append("name a net exactly (no glob) to override"
                         + (" all but 'locked'" if 'locked' in by_reason else "")
                         + (" and `sense:`" if any(map(is_sense_reason, by_reason))
                            else ""))
        if 'locked' in by_reason:
            hints.append("KiCad-locked copper has no override; unlock it in "
                         "the board")
        if any(map(is_sense_reason, by_reason)):
            hints.append(SENSE_HINT)
        print(f"  {len(blocked)} PROTECTED net(s) excluded from {context} ({det})"
              f" -- {'; '.join(hints)}")
    return kept
