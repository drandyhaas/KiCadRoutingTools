"""Shared post-route cleanup pipeline (issue #319 restructure).

One ordered pipeline used by BOTH routing fronts (route.py single-ended,
route_diff.py diff-pair), so the pass list, the ordering constraints, and the
strip-list plumbing exist exactly once instead of being hand-mirrored.

The contract -- uniform, with NO exceptions:

  * every pass MUTATES ``pcb_data`` in place, so at any point in (and after)
    the pipeline ``pcb_data`` IS the board that will be written, modulo the
    swap vias / stub layer modifications the writer applies to the file text
    separately;
  * removed copper that this run routed (present in a ``results`` entry) is
    dropped from ``results`` -- the write-list -- in place;
  * removed copper that came from the INPUT FILE is collected in
    ``CleanupOutcome.input_strip_segments`` for the writer to delete from its
    verbatim copy of the input;
  * subtractive passes must not manufacture soft joints (they run with the
    ``_restore_soft_joint_bridges`` guard), and ``close_soft_joints`` runs
    LAST so no later pass can re-delete the bridges it adds. The
    ``no_more_copper_removal`` flag makes that ordering an enforced invariant
    instead of a comment.

``KICAD_BOARD_LEDGER=1`` audits the contract at the end of a run (see
``verify_board_file_parity``): per in-scope net, the copper in ``pcb_data``
and the copper the writer will produce (original input copper - strips +
emitted results) must be identical.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from pcb_modification import (
    snap_stub_gaps,
    drop_phantom_copper,
    prune_grazing_segments,
    nudge_grazing_octolinear,
    nudge_grazing_microshift,
    nudge_grazing_vias,
    prune_redundant_cycles,
    collapse_strict_redundant,
    remove_orphan_islands,
    sweep_dead_ends,
    trim_dangles_past_body_anchor,
    neck_wide_segments_grazing_pads,
    close_soft_joints,
)

RED = '\033[91m'
RESET = '\033[0m'


@dataclass
class CleanupOutcome:
    """What the pipeline did, for the caller's writer/summary plumbing."""
    # Original INPUT-FILE segments removed by any pass: the writer must strip
    # these blocks from its verbatim copy of the input file.
    input_strip_segments: List = field(default_factory=list)
    # Original INPUT-FILE vias removed (orphan islands take their barrels).
    input_strip_vias: List = field(default_factory=list)
    counts: dict = field(default_factory=dict)


def run_post_route_cleanup(results, pcb_data, scope_net_ids, config, *,
                           label: str = '',
                           snap: bool = True,
                           phantom: bool = True,
                           graze: bool = True,
                           octolinear: bool = True,
                           via_nudge: bool = True,
                           cycles: bool = True,
                           neck: bool = True,
                           microshift_max_shift: Optional[float] = None,
                           freeze_hook: Optional[Callable[[], None]] = None,
                           original_segment_ids=None,
                           original_via_ids=None,
                           keep_input_copper: bool = False,
                           ) -> CleanupOutcome:
    """Run the post-route cleanup passes in their one canonical order.

    Order and rationale (each pass's own docstring has the full story):
      1. snap_stub_gaps        -- additive; extend a stub that stopped a
                                  fraction short of same-net copper (#84).
      2. drop_phantom_copper   -- reconcile the write-list against the board so
                                  ripped-but-unrestored copper never ships (#133).
         -> ``freeze_hook`` fires HERE: the board is now exactly what routing
            committed, before any cleanup pass trims it. route.py freezes its
            stale-input-strip reference snapshot at this point (#220/#284),
            which is what makes the strip immune to the passes below.
      3. prune_grazing_segments -- drop a redundant foreign-grazing segment
                                  (#224); before the cycle/dead-end passes so
                                  the stub it leaves gets collapsed.
      4. nudge_grazing_octolinear / 5. nudge_grazing_microshift /
      6. nudge_grazing_vias    -- re-bend / micro-shift load-bearing grazes the
                                  prune must keep (#224/#276/#280).
      7. prune_redundant_cycles -- per-net tree invariant (RAM_A9 loops).
      8. sweep_dead_ends       -- trim dead-end spurs and unsupported vias.
      9. neck_wide_segments_grazing_pads -- width-only fix for wide power
                                  trunks overlapping a fine-pitch foreign pad.
     10. close_soft_joints     -- LAST copper step (#319 ordering): bridge any
                                  remaining same-net soft joint. The
                                  subtractive passes above run with the
                                  _restore_soft_joint_bridges guard, so every
                                  joint close sees is router-born, never one a
                                  cleanup pass manufactured.

    ``label`` prefixes the progress prints (e.g. "Diff-pair "). The pass
    switches exist for front parity, not taste:
      * route_diff.py historically runs without snap/phantom/neck; flipping
        them on there is a deliberate behavior change, not a side effect of
        this refactor (octolinear WAS flipped on for #318: the re-bend keeps
        anchors coincident and clears() includes the partner net);
      * the plane fronts' file-round-trip (clean_plane_copper) must keep
        ``via_nudge`` OFF -- it re-parses the written board, so every via is
        an "input" via whose in-place geometry move the segment-level
        write-back cannot express (#280/#281: input-via moves silently revert
        in writers, stranding the dragged segment endpoints);
      * ``microshift_max_shift`` defaults to grid_step/2 (sub-grid
        quantization); the plane repair path passes a full grid_step (#308 --
        its tracks are on-grid against off-grid holes).

    Returns a CleanupOutcome; input-file removals from every pass are merged
    into ``input_strip_segments`` in pass order.

    ``keep_input_copper`` makes INPUT-FILE copper read-only for every
    subtractive/rewriting pass: it still anchors connectivity, degree and
    grazing models, but is never removed, split, or re-bent — for chained
    flows whose earlier stages author copper (escape stubs, hand-authored
    routes) that a later stage must still see verbatim. This-run copper is
    cleaned normally, and the strip lists then stay empty by construction.
    Do NOT set it on the plane fronts' file-round-trip (results=[] makes
    every segment read as input and would freeze all plane copper).
    """
    out = CleanupOutcome()
    counts = out.counts
    strip = out.input_strip_segments

    # KICAD_LEDGER_TRACE="x,y;x,y;..." -- ledger-leak forensics: after every
    # pass, print how many pcb_data / write-list segments have an endpoint
    # within 0.05mm of each watched point. Pinpoints WHICH pass moved one
    # representation without the other (pair with KICAD_BOARD_LEDGER).
    _trace_pts = []
    _t = os.environ.get('KICAD_LEDGER_TRACE')
    if _t:
        for tok in _t.split(';'):
            try:
                x, y = tok.split(',')
                _trace_pts.append((float(x), float(y)))
            except ValueError:
                pass

    def _trace(stage):
        if not _trace_pts:
            return
        for (tx, ty) in _trace_pts:
            def near(s):
                return ((abs(s.start_x - tx) < 0.05 and abs(s.start_y - ty) < 0.05)
                        or (abs(s.end_x - tx) < 0.05 and abs(s.end_y - ty) < 0.05))
            nb = sum(1 for s in pcb_data.segments if near(s))
            nr = sum(1 for r in results for s in (r.get('new_segments') or []) if near(s))
            ns = sum(1 for s in strip if near(s))
            print(f"[LEDGER_TRACE]{label} after {stage}: ({tx},{ty}) "
                  f"board={nb} writelist={nr} strip={ns}")

    _trace('start')
    if snap:
        _snapped = snap_stub_gaps(results, pcb_data, scope_net_ids, config)
        counts['stub_gaps_snapped'] = _snapped
        _trace('snap')
        if _snapped:
            print(f"{label}Closed {_snapped} stub gap(s) to same-net copper")

    if phantom:
        # Two-way reconciliation: write-list phantoms dropped from results, and
        # -- when the caller identifies its input copper -- orphan routed
        # copper (rip/reroute slivers no result references) dropped from
        # pcb_data, so board == write model from here on.
        _ph_segs, _ph_vias = drop_phantom_copper(
            results, pcb_data,
            original_segment_ids=original_segment_ids,
            original_via_ids=original_via_ids)
        counts['phantom_segments'] = _ph_segs
        counts['phantom_vias'] = _ph_vias
        _trace('phantom')
        if _ph_segs or _ph_vias:
            print(f"{label}Dropped {_ph_segs} phantom segment(s) and {_ph_vias} "
                  f"phantom via(s) not on the board from the write-list")

    # The board now holds exactly what routing committed (rips applied, write
    # list reconciled). A caller that needs a stable "what routing produced"
    # reference -- route.py's #220/#284 stale-input strip -- snapshots it here,
    # BEFORE the subtractive passes below trim it.
    if freeze_hook is not None:
        freeze_hook()

    if graze:
        _gz_segs, _gz_nets, _gz_strip = prune_grazing_segments(
            results, pcb_data, scope_net_ids, clearance=config.clearance,
            check_foreign_segments=True, keep_input_copper=keep_input_copper)
        counts['graze_pruned'] = _gz_segs
        _trace('graze')
        strip.extend(_gz_strip)
        if _gz_segs:
            print(f"{label}Graze prune: removed {_gz_segs} grazing segment(s) "
                  f"across {_gz_nets} net(s)")

    if octolinear:
        _nz_segs, _nz_nets, _nz_strip, _ = nudge_grazing_octolinear(
            results, pcb_data, scope_net_ids, clearance=config.clearance,
            keep_input_copper=keep_input_copper)
        counts['octolinear_nudged'] = _nz_segs
        _trace('octolinear')
        strip.extend(_nz_strip)
        if _nz_segs:
            print(f"{label}Graze nudge: re-bent grazing octolinear jog(s) "
                  f"on {_nz_nets} net(s)")

    _ms_segs, _ms_nets, _ms_strip, _ = nudge_grazing_microshift(
        results, pcb_data, scope_net_ids, clearance=config.clearance,
        max_shift=(microshift_max_shift if microshift_max_shift is not None
                   else config.grid_step / 2),
        keep_input_copper=keep_input_copper,
        # #436: cross-class-aware graze fix — measure shortfall against each
        # net's own netclass floor and each foreign net's class, not the global
        # clearance (daisho's 456 same-class grid grazes, cparti's SW1-vs-SMA).
        net_clearances=getattr(config, 'net_clearances', None) or None)
    counts['microshifted'] = _ms_segs
    _trace('microshift')
    strip.extend(_ms_strip)
    if _ms_segs:
        print(f"{label}Graze micro-shift: moved copper by its clearance "
              f"shortfall on {_ms_nets} net(s)")

    if via_nudge:
        _vn_moved, _vn_nets, _ = nudge_grazing_vias(
            results, pcb_data, scope_net_ids, clearance=config.clearance,
            hole_to_hole=config.hole_to_hole_clearance,
            # #339 rework: one grid cell (was grid_step/2), capped per-via at
            # via_size/4 inside the fn. grid_step/2 (~0.025 at fine grid) was too
            # short to clear the ~40um grazes the looser UNBLOCK_REFIT_MARGIN_MM now
            # leaves; a full cell reaches them while a via never moves more than
            # one cell. Moves the via (still connected), never shrinks it.
            max_shift=config.grid_step)
        counts['vias_nudged'] = _vn_moved
        _trace('via_nudge')
        if _vn_moved:
            print(f"{label}Via nudge: moved {_vn_moved} grazing via(s) on "
                  f"{_vn_nets} net(s) by their sub-grid clearance shortfall (#280)")

    if cycles:
        _cy_segs, _cy_nets, _cy_strip = prune_redundant_cycles(
            results, pcb_data, scope_net_ids, clearance=config.clearance,
            keep_input_copper=keep_input_copper)
        counts['cycles_pruned'] = _cy_segs
        _trace('cycles')
        strip.extend(_cy_strip)
        if _cy_segs:
            print(f"{label}Cycle prune: removed {_cy_segs} redundant loop "
                  f"segment(s) across {_cy_nets} net(s)")

    # Strict-redundant collapse (#217 classes 1-2): superseded parallel
    # chains and pad/via-buried tails that are redundant under the strict
    # width-clamped graph. Before the sweep so freed this-run vias drop.
    _sc_n, _sc_strip = collapse_strict_redundant(results, pcb_data, scope_net_ids,
                                                 keep_input_copper=keep_input_copper)
    counts['strict_collapsed'] = _sc_n
    _trace('strict_collapse')
    strip.extend(_sc_strip)
    if _sc_n:
        print(f"{label}Strict collapse: removed {_sc_n} redundant segment(s) "
              f"(superseded parallel/buried copper)")

    # Orphan islands (#217): track-copper components reaching NO pad of
    # their net -- rip/reroute leftovers connected to nothing. Runs before
    # the dead-end sweep so the sweep's unsupported-via pass drops the
    # islands' freed this-run vias.
    _oi_n, _oi_segs, _oi_strip, _oi_via_strip = remove_orphan_islands(
        results, pcb_data, scope_net_ids, keep_input_copper=keep_input_copper)
    out.input_strip_vias.extend(_oi_via_strip)
    counts['orphan_islands'] = _oi_n
    _trace('orphan_islands')
    strip.extend(_oi_strip)
    if _oi_n:
        print(f"{label}Orphan islands: removed {_oi_n} pad-less copper "
              f"island(s) ({_oi_segs} segment(s))")

    _de_segs, _de_vias, _de_strip = sweep_dead_ends(results, pcb_data, scope_net_ids,
                                                    keep_input_copper=keep_input_copper)
    counts['dead_ends_swept'] = _de_segs
    counts['dead_end_vias'] = _de_vias
    _trace('sweep')
    strip.extend(_de_strip)
    if _de_segs or _de_vias:
        print(f"{label}Dead-end sweep: trimmed {_de_segs} dead-end segment(s) "
              f"and {_de_vias} unsupported via(s)")

    # Half-segment dangles the sweep cannot touch (#347 core1106 CLK1P tail):
    # a dead-end segment T-anchored mid-BODY (a via ON the trace, or a tee) is
    # load-bearing through the anchor, so the whole-segment prune keeps it and
    # the copper past the anchor ships as an antenna. Split-trim to the anchor.
    _dt_n, _dt_strip = trim_dangles_past_body_anchor(results, pcb_data, scope_net_ids,
                                                     keep_input_copper=keep_input_copper)
    counts['dangles_trimmed'] = _dt_n
    _trace('dangle_trim')
    strip.extend(_dt_strip)
    if _dt_n:
        print(f"{label}Dangle trim: cut {_dt_n} dead-end tail(s) back to their "
              f"mid-body anchor")

    if neck:
        _necked = neck_wide_segments_grazing_pads(results, pcb_data, config)
        counts['width_necked'] = _necked
        _trace('neck')
        if _necked:
            print(f"{label}Width neck: narrowed {_necked} wide segment(s) "
                  f"grazing a foreign pad")

    # FINAL copper step (#319 ordering): nothing below may remove copper.
    # KICAD_NO_SOFT_JOINT_BRIDGE=1 is an A/B ablation knob (like
    # PRUNE_CONN_VERIFY / KICAD_BOARD_LEDGER): it isolates close's contribution
    # when validating pipeline changes across a replay corpus -- the shipped
    # joints then surface as check_drc segment-endpoint-gap warnings instead.
    if os.environ.get('KICAD_NO_SOFT_JOINT_BRIDGE'):
        print(f"{label}soft-joint bridging DISABLED (KICAD_NO_SOFT_JOINT_BRIDGE)")
        counts['soft_joints_bridged'] = 0
    else:
        _bridged = close_soft_joints(results, pcb_data, scope_net_ids, config)
        counts['soft_joints_bridged'] = _bridged
        if _bridged:
            print(f"{label}Bridged {_bridged} same-net soft joint(s) with a tiny "
                  f"connector")

    return out


def _q(x):
    """Ledger coordinate quantizer: FIRST round at the writer's emission
    precision (kicad_writer emits coordinates with :.6f), THEN bucket at 1um.
    Without the pre-round, an in-memory float a half-ULP below a 0.5um
    boundary (50.12249999...) and its 6dp file text (50.122500) land in
    different 1um buckets and the post-write audit reports a phantom
    board/file pair (castor_pollux VUSB etc. -- same copper, sub-0.5um)."""
    return round(round(x, 6), 3)


def _seg_ledger_sig(s):
    a = (_q(s.start_x), _q(s.start_y))
    b = (_q(s.end_x), _q(s.end_y))
    return (min(a, b), max(a, b), s.layer, _q(s.width))


def _via_ledger_sig(v):
    return (_q(v.x), _q(v.y),
            _q(getattr(v, 'size', 0) or 0),
            _q(getattr(v, 'drill', 0) or 0))


def verify_written_file_parity(output_file, pcb_data, scope_net_ids,
                               label: str = '') -> bool:
    """KICAD_BOARD_LEDGER=1 post-write audit: re-parse the WRITTEN file and
    compare each in-scope net's copper (segments AND vias) against pcb_data.

    This is the deep end of the ledger: the in-memory check
    (verify_board_file_parity) audits the pipeline's own bookkeeping, but the
    writer applies a further set of TEXT transformations to the input copy --
    layer modifications from stub swaps, polarity/target-swap net relabels,
    strip-list block removal -- each matching file text by pattern (the #264
    bug class: a layer-swap mod relabelled the WRONG net's identical-geometry
    segment). Those mechanisms all mutate pcb_data at decision time, so after
    a correct write the file must equal the board for every in-scope net; any
    divergence is a writer-application bug. Nets are matched by NAME (KiCad 10
    files reference nets by name; ids can renumber across a parse).

    No-op unless KICAD_BOARD_LEDGER is set. Returns True when clean.
    """
    if not os.environ.get('KICAD_BOARD_LEDGER'):
        return True
    from collections import Counter
    from kicad_parser import parse_kicad_pcb

    try:
        written = parse_kicad_pcb(output_file)
    except Exception as e:  # audit must never break the run
        print(f"{RED}[FILE_LEDGER]{label} could not re-parse {output_file}: {e}{RESET}")
        return False

    def by_name(pcb):
        segs, vias = {}, {}
        for s in pcb.segments:
            n = pcb.nets[s.net_id].name if s.net_id in pcb.nets else s.net_id
            segs.setdefault(n, []).append(s)
        for v in pcb.vias:
            n = pcb.nets[v.net_id].name if v.net_id in pcb.nets else v.net_id
            vias.setdefault(n, []).append(v)
        return segs, vias

    board_segs, board_vias = by_name(pcb_data)
    file_segs, file_vias = by_name(written)
    scope_names = sorted(pcb_data.nets[nid].name for nid in (scope_net_ids or [])
                         if nid in getattr(pcb_data, 'nets', {}))
    bad = 0
    for name in scope_names:
        bs = Counter(_seg_ledger_sig(s) for s in board_segs.get(name, []))
        fs = Counter(_seg_ledger_sig(s) for s in file_segs.get(name, []))
        bv = Counter(_via_ledger_sig(v) for v in board_vias.get(name, []))
        fv = Counter(_via_ledger_sig(v) for v in file_vias.get(name, []))
        if bs != fs or bv != fv:
            bad += 1
            print(f"{RED}[FILE_LEDGER]{label} {name} diverged: "
                  f"segs board-only={sum((bs - fs).values())} "
                  f"file-only={sum((fs - bs).values())}, "
                  f"vias board-only={sum((bv - fv).values())} "
                  f"file-only={sum((fv - bv).values())}{RESET}")
            for sig, n in list((bs - fs).items())[:3]:
                print(f"    seg board-only x{n}: {sig}")
            for sig, n in list((fs - bs).items())[:3]:
                print(f"    seg file-only  x{n}: {sig}")
    if bad:
        print(f"{RED}[FILE_LEDGER]{label} FAILED: {bad}/{len(scope_names)} "
              f"net(s) differ between pcb_data and the written file{RESET}")
    else:
        print(f"[FILE_LEDGER]{label} OK: written file matches pcb_data for all "
              f"{len(scope_names)} in-scope net(s)")
    return bad == 0


def verify_board_file_parity(pcb_data, scope_net_ids, orig_seg_by_net, results,
                             strip_segments, label: str = '') -> bool:
    """KICAD_BOARD_LEDGER=1 audit of the pipeline contract: for every in-scope
    net, the board (``pcb_data.segments``) must equal the WRITE MODEL -- the
    original input copper minus every stripped segment, plus every segment the
    write-list will emit. A divergence means some pass changed one
    representation without the other (the exact class of bug behind the #319
    B1 regression) and would ship a file that does not match what the cleanup
    passes and connectivity gates reasoned about.

    Segments are compared as multisets of (endpoints, layer, width) signatures
    (object identity intentionally ignored: in-place edits shared between
    board and write-list are fine -- geometry divergence is what matters).
    Prints a per-net report and returns True when clean. No-op unless
    KICAD_BOARD_LEDGER is set.
    """
    if not os.environ.get('KICAD_BOARD_LEDGER'):
        return True
    from collections import Counter

    strip_ids = {id(s) for s in (strip_segments or [])}
    emitted_by_net = {}
    for r in results:
        for s in (r.get('new_segments') or []):
            emitted_by_net.setdefault(s.net_id, []).append(s)
    board_by_net = {}
    _seen_board_ids = set()
    for s in pcb_data.segments:
        # A duplicate OBJECT entry in pcb_data.segments is always a bug (#195:
        # duplicate graph nodes defeat every connectivity gate; here it also
        # desyncs the board from the write model, which holds the object once).
        if id(s) in _seen_board_ids:
            print(f"{RED}[BOARD_LEDGER]{label} DUPLICATE board entry: net "
                  f"{s.net_id} ({s.start_x},{s.start_y})-({s.end_x},{s.end_y}) "
                  f"{s.layer} appears more than once in pcb_data.segments{RESET}")
        _seen_board_ids.add(id(s))
        board_by_net.setdefault(s.net_id, []).append(s)
    _seen_via_ids = set()
    for v in pcb_data.vias:
        # Via twin of the duplicate check (the swap-via double-append shipped
        # every pad swap via onto the board twice).
        if id(v) in _seen_via_ids:
            print(f"{RED}[BOARD_LEDGER]{label} DUPLICATE board via: net "
                  f"{v.net_id} at ({v.x},{v.y}) appears more than once in "
                  f"pcb_data.vias{RESET}")
        _seen_via_ids.add(id(v))

    scope = sorted(scope_net_ids or [])
    bad = 0
    for nid in scope:
        model = [s for s in orig_seg_by_net.get(nid, []) if id(s) not in strip_ids]
        model += emitted_by_net.get(nid, [])
        model_sig = Counter(_seg_ledger_sig(s) for s in model)
        board_sig = Counter(_seg_ledger_sig(s) for s in board_by_net.get(nid, []))
        if model_sig != board_sig:
            bad += 1
            board_only = board_sig - model_sig
            file_only = model_sig - board_sig
            name = (pcb_data.nets[nid].name
                    if nid in getattr(pcb_data, 'nets', {}) else f"net {nid}")
            print(f"{RED}[BOARD_LEDGER]{label} {name} (id={nid}) diverged: "
                  f"{sum(board_only.values())} board-only, "
                  f"{sum(file_only.values())} file-only segment(s){RESET}")
            for sig, n in list(board_only.items())[:5]:
                print(f"    board-only x{n}: {sig}")
            for sig, n in list(file_only.items())[:5]:
                print(f"    file-only  x{n}: {sig}")
    if bad:
        print(f"{RED}[BOARD_LEDGER]{label} FAILED: {bad}/{len(scope)} net(s) "
              f"diverged between pcb_data and the write model{RESET}")
    else:
        print(f"[BOARD_LEDGER]{label} OK: pcb_data == write model for all "
              f"{len(scope)} in-scope net(s)")
    return bad == 0
