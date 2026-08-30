"""Read-once cache of the KICAD_* environment knobs.

The engine's experimental/diagnostic knobs were read with ``os.environ.get``
at their point of use -- some per net, per leg, or per congestion sample --
which both hammers the environ dict from hot paths and scatters the knob
inventory across a dozen modules. Every knob is read ONCE here, at import,
and stored as a plain module attribute; call sites read the attribute.

Semantics: a knob is its value at process start (first import). Harnesses that
pass knobs to CHILD processes via the environment are unaffected -- the child
re-reads at its own import (see memory: env vars must be EXPORTED for
subprocesses, an inline prefix misses children). Code that mutates
``os.environ`` mid-process and needs THIS process to see it calls
``refresh()`` afterwards.

Each attribute preserves the exact parse semantics its call site had (truthy
vs '1' vs default-on off-switch vs opt-in set) -- flipping one to a stricter
parse silently disables someone's recorded workflow. Argparse ``default=os.
environ.get(...)`` sites (route.py --ripup-* ) deliberately stay in place:
they are startup-once and self-document the override in --help.
"""
from __future__ import annotations

import os

_OFF = ('0', 'off', 'false')
_ON = ('1', 'true', 'on')


def _truthy(name: str) -> bool:
    return bool(os.environ.get(name))


def _on_default(name: str, default: str = '1') -> bool:
    return os.environ.get(name, default) not in _OFF


def _opt_in(name: str) -> bool:
    return os.environ.get(name, '') in _ON


def _f(name: str, dflt: float) -> float:
    try:
        return float(os.environ.get(name, str(dflt)) or dflt)
    except ValueError:
        return dflt


def _i(name: str, dflt: int) -> int:
    try:
        return int(os.environ.get(name, str(dflt)) or dflt)
    except ValueError:
        return dflt


def _s(name: str, dflt: str = '') -> str:
    return os.environ.get(name, dflt)


def refresh() -> None:
    """(Re)read every knob from os.environ (tests / in-process mutation)."""
    g = globals()

    # --- routing behavior switches (default ON, env turns OFF) -------------
    g['SEAM_REASK'] = _on_default('KICAD_SEAM_REASK', '')          # '' not in _OFF -> on
    g['ISLAND_LAUNCH'] = _on_default('KICAD_ISLAND_LAUNCH')
    g['MULTIPOINT_STUB_SWITCH'] = _on_default('KICAD_MULTIPOINT_STUB_SWITCH')
    g['HYBRID_COUPLE'] = _on_default('KICAD_HYBRID_COUPLE', '')
    g['BUS_MULTIPOINT_SPAN'] = _on_default('KICAD_BUS_MULTIPOINT_SPAN')
    g['BARE_BALL_ZONE_EXEMPT'] = _on_default('KICAD_BARE_BALL_ZONE_EXEMPT')
    g['DIRECT_FIRST'] = _on_default('KICAD_DIRECT_FIRST')
    # In-loop stub-debris trim (default ON, KICAD_STUB_DEBRIS_TRIM=0
    # reverts): when a route commits, immediately prune the unused
    # branches of its own pre-existing stub tree AND any via they leave
    # dangling, so the freed cells are routable by the very next net --
    # sweep_dead_ends does the same board-wide but only at cleanup,
    # after every net has already routed around the debris (and it never
    # touches input vias at all).
    g['STUB_DEBRIS_TRIM'] = _on_default('KICAD_STUB_DEBRIS_TRIM')
    g['IMPEDANCE_NECKDOWN'] = (_s('KICAD_IMPEDANCE_NECKDOWN', '1').strip().lower()
                               not in ('0', 'false', 'no', 'off'))
    # #648 oracle source union: kicad-cli DRC gates the demand set, the
    # exact-fill source carries the geometry. =0 restores the pure exact
    # source for A/B.
    g['ORACLE_UNION'] = _s('KICAD_ORACLE_UNION', '1') != '0'
    # #650 in-run DRC floor sync: before anything grades the board mid-run,
    # lower the output's sibling .kicad_pro to the COPPER floors this run
    # routed to, so the in-run audit fills the pours the way the SHIPPED
    # board will. Until this, that sibling was the input's (seeded by
    # seed_project_for_output) and its looser declared floors pulled the
    # pours back further than the shipped board's -- measured on orangecrab:
    # 57 reported opens vs the 46 that ship (GND 19 vs 11), all of it
    # rules.min_hole_clearance 0.25 -> 0.0889. =0 restores the old staleness
    # for A/B.
    g['INRUN_FLOOR_SYNC'] = _s('KICAD_INRUN_FLOOR_SYNC', '1') != '0'
    # #667 net-tie band pricing (mm-equivalent cost per band cell,
    # DIFFERENTIAL: the own-pad approach stays free, off-pad corridor
    # cells are priced). Measured INERT on cynthion at 0.5 and 5.0 --
    # the residual violations are the no-legal-landing footprint class
    # (a tab flush inside the partner outline admits NO flag-free tip
    # position, bisected to 5um), which no cost can steer. Default OFF;
    # kept for corpus A/B on boards with mid-route plough-throughs.
    try:
        g['TIE_BAND_COST'] = float(_s('KICAD_TIE_BAND_COST', '0') or 0)
    except ValueError:
        g['TIE_BAND_COST'] = 0.0
    g['NET_RESCUE'] = _s('KICAD_NET_RESCUE', '1') != '0'
    # #658 in-run river packing: pack each routed bus member's runs against
    # already-committed sibling runs at the copper choke point, BEFORE the
    # route becomes an obstacle -- the vacated lane is then free for every
    # later net in the SAME pass (the pack_river post-pass frees it only
    # for the next chain step). Experimental; default off pending the
    # multi-board screen.
    g['PACK_INLINE'] = _s('KICAD_PACK_INLINE', '0') != '0'
    # #666 bare-ball escape rung inside the net rescue: a stripped or
    # never-fanned SMD ball (zero own copper, 'no rippable blockers found')
    # gets a dogbone escape (offset via + pad->via trace, via-in-pad clamp)
    # before gap routing / terminal escalation. =0 disables for A/B.
    g['BARE_PAD_ESCAPE'] = _s('KICAD_BARE_PAD_ESCAPE', '1') != '0'
    # #666/IO_9: when the strict bare-ball escape fails, allow a MOVABLE-cap
    # conflict IF the cap has a verified relocation -- the scoped post-write
    # cap move + oracle re-weld then clears the conflict. =0 disables.
    g['RESCUE_CAP_MOVE'] = _s('KICAD_RESCUE_CAP_MOVE', '1') != '0'
    # Terminal geometry escalation ("better than shipping opens", 2026-08-05):
    # post-rescue whole-net retry with track width + via size marching down
    # together toward the fab floor. =0 disables for A/B debugging.
    g['TERMINAL_ESCALATION'] = _s('KICAD_TERMINAL_ESCALATION', '1') != '0'
    # PR #535's idea as a #189 escalation rung: when the in-pad unblock via
    # fails (or #581 forbids via-in-pad), search this radius (mm) around the
    # boxed pad for the first spot where a through-via is legal on every
    # layer and inject a pad->via escape stub (the plane-tap machinery does
    # the search + trace). =0 disables the off-pad rung.
    g['ESCAPE_STUB_RADIUS'] = _f('KICAD_ESCAPE_STUB_RADIUS', 1.0)
    # #529 dynamic iterations, DEFAULT ON (=0 reverts to static caps): full
    # searches run at min(base, CLAMP) and earn +1x base tranches while the
    # heuristic keeps approaching, up to a flat 1e7 ceiling. CLAMP defaults
    # to 1e7 = no clamping (corpus: -29 incomplete nets over 150 boards);
    # set CLAMP=200000 as the deliberate speed-over-completion dial.
    g['DYNAMIC_ITERATIONS'] = _s('KICAD_DYNAMIC_ITERATIONS', '1') != '0'
    g['DYNAMIC_ITERATIONS_CLAMP'] = _i('KICAD_DYNAMIC_ITERATIONS_CLAMP', 10_000_000)

    # --- opt-in experiments (env turns ON) ----------------------------------
    g['COLLINEAR_VIAS'] = _opt_in('KICAD_COLLINEAR_VIAS')  # #487: on-axis vias
    # Proximity composition: SUM across sources (per-net / per-category,
    # deduped max WITHIN each source) instead of a flat per-cell max. Adds a
    # density gradient (10 stubs price higher than 1) at the cost of steeper
    # fields in dense escape regions. Values: '1'/'sum' = sum everywhere;
    # 'zoned' = sum in open field, MAX inside BGA escape fields (zone +
    # bga_proximity_radius) -- the sum win comes from open-field density,
    # while inside the escape collar the stacked foreign-stub fields price a
    # net's MANDATORY approach (lpddr4 A/B: deficit 1->2, 6x search).
    # 'softcap' (#584) = the global blend max + ALPHA*(sum - max) per cell,
    # everywhere: keeps the max-mode floor plus a fraction of the crowd
    # pressure, so density still repels but stacked fields near mandatory
    # approaches can't explode. ALPHA=0 reproduces max, ALPHA=1 reproduces
    # sum (both exactly, up to integer-cost rounding).
    # Off = historical max semantics everywhere.
    # Default = historical MAX. Briefly 'zoned' (s2 rescan -30 on the curated
    # set) -- REVERTED with MAX_RIPUP=5: the sets-11-15 holdout showed the two
    # together erasing the other flips' gains on ordinary boards (v4 -2% vs
    # lean -45% vs old defaults) at +37% CPU. zoned/sum/softcap remain opt-in
    # (retry-tier on knob-sensitive boards).
    _ps = _s('KICAD_PROXIMITY_SUM', '').strip().lower()
    g['PROXIMITY_SUM_MODE'] = ('zoned' if _ps == 'zoned'
                               else 'softcap' if _ps == 'softcap'
                               else 'sum' if _ps in _ON else '')
    g['PROXIMITY_SUM'] = bool(g['PROXIMITY_SUM_MODE'])
    g['PROXIMITY_SOFTCAP_ALPHA'] = _f('KICAD_PROXIMITY_SOFTCAP_ALPHA', 0.3)
    # #585 item 1: layer-aware stub proximity. The stub's OWN layer keeps the
    # full cost; other layers pay cost/M (M = n_layers/2 by default, so a
    # 2-layer board is unchanged and a 6-layer board discounts 3x) -- a via
    # escape near the stub still needs track room on other layers, just less
    # of it. KICAD_STUB_LAYER_M overrides M (>= 1).
    # #585 item 8's two pseudo-stub gates, split so each can be A/B'd alone.
    # Both ON = f785a7e's shipped behaviour, restored. They were briefly
    # defaulted OFF on a two-board 2x2 whose spread turned out comparable to its
    # own noise; two corpus A/Bs then measured ON BETTER (by 3 on 148 boards, by
    # 2 on 129 with smoothing restored -- 7 boards better vs 4 worse). Setting
    # =0 gives the pre-f785a7e behaviour: any footprint with >= min_pads pads
    # emits, and the proxy never retires. Kept as separate knobs because the two
    # gates are wildly asymmetric -- on spartan6's final board the retire gate
    # removes 95%% of emitters (667 -> 33 pads), the package gate 33%%.
    g['FINE_PITCH_PSEUDO_STUBS'] = _on_default('KICAD_FINE_PITCH_PSEUDO_STUBS')
    g['PSEUDO_STUB_RETIRE'] = _on_default('KICAD_PSEUDO_STUB_RETIRE')
    g['STUB_LAYER_AWARE'] = _opt_in('KICAD_STUB_LAYER_AWARE')
    g['STUB_LAYER_M'] = _f('KICAD_STUB_LAYER_M', 0.0)  # 0 = auto n_layers/2
    # (#585 item 2 -- excluding plane-destined nets' stubs -- was DECIDED
    # AS-IS, no knob: a pour needs reserved access space exactly like tracks.)
    # #585 item 4: per-package proximity fields (BGA/QFN/QFP, radius scaled
    # sqrt(n_pads/1000) x bga_proximity_radius clamped [2, cap]). Screened
    # NEGATIVE as a default (glasgow 7->10, oc midchain 23->26 -- the sqrt
    # scaling SHRINKS large-BGA rings, and tiny DFN/SOT packages classified
    # QFN add clutter); opt-in pending a scale-up-only / pad-floor variant.
    g['PACKAGE_PROXIMITY'] = _opt_in('KICAD_PACKAGE_PROXIMITY')
    # #585 item 3a: pre-existing (input-board) copper of rippable registered
    # nets emits track-proximity ghosts like copper routed this run (only
    # matters when track_proximity_cost > 0).
    g['TRACK_PROX_PREEXISTING'] = _opt_in('KICAD_TRACK_PROX_PREEXISTING')
    # per-tranche quantum = max(CELLS grid cells, PCT% of tranche-start best_h)
    g['DYNAMIC_ITERATIONS_QUANTUM_CELLS'] = _f('KICAD_DYNAMIC_ITERATIONS_QUANTUM_CELLS', 2.0)
    g['DYNAMIC_ITERATIONS_QUANTUM_PCT'] = _f('KICAD_DYNAMIC_ITERATIONS_QUANTUM_PCT', 2.0)
    # plateau grace: consecutive quantum-failing tranches tolerated before
    # denial (progress judged cumulatively over the plateau; emulates a wider
    # window without a bigger base)
    g['DYNAMIC_ITERATIONS_GRACE'] = _i('KICAD_DYNAMIC_ITERATIONS_GRACE', 0)
    g['MULTIPOINT_DENSE_FIRST'] = _opt_in('KICAD_MULTIPOINT_DENSE_FIRST')
    g['FANOUT_DIRECT'] = _opt_in('KICAD_FANOUT_DIRECT')
    g['FANOUT_TOWARD_TARGETS'] = _opt_in('KICAD_FANOUT_TOWARD_TARGETS')
    # '' = follow the --plane-drop param (default auto); '0'/'off' forces the
    # plane-ball drop pass OFF, '1'/'auto' forces it ON -- the manifest-replay
    # A/B switch for #424 D2 (recorded chains carry no flag either way).
    g['FANOUT_PLANE_DROP'] = _s('KICAD_FANOUT_PLANE_DROP', '')
    # '0' reverts the surface-pour direct connect (plane-drop balls whose own-
    # layer same-net pour provably fills to the pad get NO via). Default ON.
    g['FANOUT_POUR_DIRECT'] = _s('KICAD_FANOUT_POUR_DIRECT', '1') != '0'
    # Near-miss companion to pour-direct (#652): a plane-drop ball whose own-
    # layer pour's MAIN component stops just short of the pad (within
    # KICAD_FANOUT_POUR_TRACK_R mm, default 2.0) gets a short surface track
    # into the fill instead of a via -- the other half of the human dog-bone
    # economy. '0' reverts. Default ON.
    g['FANOUT_POUR_TRACK'] = _s('KICAD_FANOUT_POUR_TRACK', '1') != '0'
    g['FANOUT_POUR_TRACK_R'] = float(_s('KICAD_FANOUT_POUR_TRACK_R', '2.0') or 2.0)
    # #669: '1' makes escape-method 'auto' retry channel drops with DOGBONE
    # first, then underpad only if dogbone still dropped balls. Default OFF:
    # the sets1-5 corpus A/B REJECTED it as a default (autodb0/autodb1 arms
    # at 21b1c0e5: +10 incomplete nets and +59 kicad DRC vs the underpad-only
    # retry -- dogbone gap vias claim inter-ball streets that recorded chains'
    # later steps collide with). Explicit --escape-method dogbone remains the
    # populated-array doctrine; this knob is the opt-in ladder for chains
    # whose params are chosen for dogbone.
    g['FANOUT_AUTO_DOGBONE'] = _s('KICAD_FANOUT_AUTO_DOGBONE', '0') == '1'
    # #652 directive 2: rip-swap rescue for terminally dropped fanout balls
    # (evict the nearest committed neighbour escape, re-escape both as a pair
    # at the fab floor; strict-win only). '0' reverts. Default ON.
    g['FANOUT_RIP_RESCUE'] = _s('KICAD_FANOUT_RIP_RESCUE', '1') != '0'
    # #652 directive 1: surface (channel-engine) rescue for balls the via
    # floor cannot serve at the array pitch. '0' reverts. Default ON.
    g['FANOUT_SURFACE_RESCUE'] = _s('KICAD_FANOUT_SURFACE_RESCUE', '1') != '0'
    # #652 human-survey escapes: lane-walked dog-bones (gap sites are a POOL
    # reached by riding the inter-row lane; k = max gaps walked) and pair
    # dive-first (diff pairs drop coupled vias at the first gap row instead
    # of running surface legs that wall the single-ended corridors). '0'
    # reverts each. Defaults ON.
    g['FANOUT_LANE_WALK'] = _s('KICAD_FANOUT_LANE_WALK', '1') != '0'
    g['FANOUT_LANE_WALK_MAX'] = _i('KICAD_FANOUT_LANE_WALK_MAX', 3)
    g['FANOUT_PAIR_DIVE'] = _s('KICAD_FANOUT_PAIR_DIVE', '1') != '0'
    # #622 lane-run surface escape (ported from bus622-take2): a ball the
    # surface A* cannot serve on its planned side -- the raster's inflated
    # pad stamps close a fine-pitch lane the exact check proves legal --
    # tries pad, 45-degree jog, straight down the gap lane to the boundary,
    # on that side first. OPT-IN: take2 shipped it default ON in dog-bone
    # mode; here it is measured on the #622 chain before any default.
    g['FANOUT_LANE_ESCAPE'] = _opt_in('KICAD_FANOUT_LANE_ESCAPE')
    g['STOP_CLEANUP'] = _opt_in('KICAD_STOP_CLEANUP')
    g['TAP_RELOCATION'] = _opt_in('KICAD_TAP_RELOCATION')  # phase-3 tap pocket moves
    # #536 octolinear route smoothing (cleanup_pipeline pass 9b): collapse
    # staircase micro-jogs into diagonal+axis shortcuts, DRC-checked, output
    # strictly octolinear. Tri-state like KICAD_FANOUT_PLANE_DROP: '' (unset,
    # default) = follow the FRONT's default -- ON for the single-ended route
    # step, OFF for diff pairs and the plane file-round-trip; '1' forces on,
    # '0' forces off. Protected copper (diff pairs, length/time-matched,
    # impedance-declared, locked) is never touched regardless. Pour cuts the
    # shortcut may inflict are the plane finalize / kicad-oracle recheck's
    # job, which runs after cleanup on both fronts.
    g['SMOOTH_ROUTE'] = _s('KICAD_SMOOTH_ROUTE', '')
    # #600 improvement gate: refuse to ship a board this run made WORSE (more
    # previously-connected nets broken than newly connected). '0' disables --
    # for A/B, and for the rare case where an operator genuinely wants the
    # regressed board on disk to inspect it.
    g['IMPROVEMENT_GATE'] = _s('KICAD_IMPROVEMENT_GATE', '1') != '0'
    g['PLANE_PARTIAL_RESTORE'] = _s('KICAD_PLANE_PARTIAL_RESTORE') == '1'
    g['DUMP_BATCH_KWARGS_CONTINUE'] = _s('KICAD_DUMP_BATCH_KWARGS_CONTINUE') == '1'
    # #431: the placement-movie camera. 'off' (default) keeps every existing
    # movie bit-for-bit; one variable turns it on for the GUI recorder,
    # run_plan.py --movie and the stress renderer at once, which is the
    # CLI/GUI parity story for a feature with no GUI control of its own.
    g['MOVIE_CAMERA'] = _s('KICAD_MOVIE_CAMERA', 'off')

    # --- truthy diagnostics / overrides -------------------------------------
    g['UNBLOCK_DEBUG'] = _truthy('KICAD_UNBLOCK_DEBUG')
    g['TAP_CROSS_SCAN'] = _truthy('KICAD_TAP_CROSS_SCAN')
    g['OBSTACLE_AUDIT'] = _truthy('KICAD_OBSTACLE_AUDIT')
    g['PLANE_MAP_PARITY'] = _truthy('KICAD_PLANE_MAP_PARITY')
    g['SETTLE_DEBUG'] = _truthy('KICAD_SETTLE_DEBUG')
    g['LEGACY_GATE_ORACLE'] = _truthy('KICAD_LEGACY_GATE_ORACLE')
    # route.py's end-of-run oracle summary check (one staged
    # kicad-cli DRC per run). Now OPT-IN (KICAD_ORACLE_SUMMARY=1), reverted
    # from default-on: it was billed as "strictly additive -- only ADDS
    # failure disclosure", but it also APPENDS to failed_multipoint, which
    # the reconcile gate reads, so oracle-open nets were retried and the
    # board's copper changed. That made routing depend on whether kicad-cli
    # is INSTALLED: the cloud image ships no KiCad, so the retry never fired
    # there and the same commit produced different copper locally vs in the
    # container -- the lora_v3 failure mode (7 DRC without KiCad, 0 with),
    # and it silently breaks A/B comparability. Disclosure is fine to make
    # environment-dependent; copper is not. See issue #675.
    g['ORACLE_SUMMARY'] = _opt_in('KICAD_ORACLE_SUMMARY')
    # #648's third link-source branch (exact-fill -> kicad-cli -> raster).
    # OPT-IN on purpose: a silent fallback would make the oracle run off a
    # different source depending on whether KiCad is installed, i.e. make
    # COPPER environment-dependent -- the exact failure #675 reverted the B-1
    # check for. Opt in and you accept a weaker source knowingly.
    g['RASTER_ORACLE'] = _opt_in('KICAD_RASTER_ORACLE')
    # HOT knobs (measured, not guessed): on splitflap_driver one route reads
    # KICAD_VIA_RUNG 902 times and KICAD_POUR_LAUNCH 316 -- they sit inside
    # per-net / per-endpoint loops, which is exactly the "hammers the environ
    # dict from hot paths" this module exists for. Every other bare-read knob
    # in the tree measured 1-2 reads per run and is left at its call site.
    # Parse semantics preserved: VIA_RUNG is a =='2' string test with default
    # '2' (so '1' or anything else disarms), the POUR_LAUNCH pair are =='1'
    # tests with default '1' (default ON, any other value disables), and
    # POUR_LAUNCH_FRAG keeps its `or 0` empty-value rule -- see below for the
    # one deliberate deviation.
    g['VIA_RUNG_2'] = os.environ.get('KICAD_VIA_RUNG', '2') == '2'
    g['POUR_LAUNCH'] = os.environ.get('KICAD_POUR_LAUNCH', '1') == '1'
    g['POUR_LAUNCH_COPPER'] = os.environ.get('KICAD_POUR_LAUNCH_COPPER', '1') == '1'
    # NOT _f(): the call site was `float(get(name, '1.0') or 0)`, so an
    # EMPTY value means 0.0 (threshold disabled) where _f would hand back the
    # 1.0 default -- silently re-enabling a threshold someone turned off,
    # which is precisely the "stricter parse disables a recorded workflow"
    # this module's docstring warns about. Preserved exactly. The one
    # deliberate difference: garbage used to raise ValueError mid-route, and
    # now falls back to the default; a crash is nobody's workflow.
    _plf = os.environ.get('KICAD_POUR_LAUNCH_FRAG', '1.0')
    try:
        g['POUR_LAUNCH_FRAG'] = float(_plf or 0)
    except ValueError:
        g['POUR_LAUNCH_FRAG'] = 1.0
    g['NO_GATE_ORACLE'] = _truthy('KICAD_NO_GATE_ORACLE')
    g['GATE_DEBUG'] = _truthy('KICAD_GATE_DEBUG')
    g['NO_SWEEP_PLATED'] = _truthy('KICAD_NO_SWEEP_PLATED')
    g['NO_SOFT_JOINT_BRIDGE'] = _truthy('KICAD_NO_SOFT_JOINT_BRIDGE')
    g['BOARD_LEDGER'] = _truthy('KICAD_BOARD_LEDGER')
    # "x,y" via position string ('' = off; consumers test truthiness + parse)
    g['RESCUE_DEBUG_VIA'] = _s('KICAD_RESCUE_DEBUG_VIA')
    g['MEM_PROBE'] = _truthy('KICAD_MEM_PROBE')
    g['NO_STATIC_BASE'] = _truthy('KICAD_NO_STATIC_BASE')
    g['ALLOW_STAGGERED_BGA'] = _truthy('KICAD_ALLOW_STAGGERED_BGA')
    g['QFN_UNDERPAD_NO_ALT_STAGGER'] = _truthy('QFN_UNDERPAD_NO_ALT_STAGGER')
    g['LEGACY_ORACLE'] = _truthy('KICAD_LEGACY_ORACLE')
    g['NO_EXACT_FILL'] = _truthy('KICAD_NO_EXACT_FILL')
    g['NO_FILL_NETCLASS'] = _truthy('KICAD_NO_FILL_NETCLASS')  # fill-model A/B

    # --- numeric knobs -------------------------------------------------------
    # Rasterization memo budget (MB, shared convention across the capsule/
    # polygon caches in routing_utils/obstacle_map). Profiling: 64MB cycled
    # ~40x on orangecrab (69% hit rate where ~99% is available). Parallel
    # stress workers multiply per-process caches -- export a smaller value
    # there.
    g['RASTER_CACHE_MB'] = _f('KICAD_RASTER_CACHE_MB', 256.0)
    g['BUS_XLAYER_PCT'] = _i('KICAD_BUS_XLAYER_PCT', 35)
    g['BUS_OFFLANE_MULT'] = _f('KICAD_BUS_OFFLANE_MULT', 1.0)   # off-lane surcharge (1.0 = off)
    g['BUS_CORRIDOR_PROBE_VIA_MULT'] = _f('KICAD_BUS_CORRIDOR_PROBE_VIA_MULT', 20.0)
    g['BUS_MAX_CORRIDOR_LAYER_CHANGES'] = _i('KICAD_BUS_MAX_CORRIDOR_LAYER_CHANGES', 1)
    g['TAP_RELOCATION_MAX'] = _i('KICAD_TAP_RELOCATION_MAX', 2)
    g['SEAM_SE_RATIO'] = _f('KICAD_SEAM_SE_RATIO', 1.3)
    g['SEAM_SE_MAX'] = _i('KICAD_SEAM_SE_MAX', 16)
    g['HYBRID_COUPLE_RADIUS'] = _f('KICAD_HYBRID_COUPLE_RADIUS', 1.0)
    g['BUS_RIP_RESISTANCE'] = _s('KICAD_BUS_RIP_RESISTANCE', '')  # float-or-empty; consumer parses
    g['CONGESTION_COST'] = _f('KICAD_CONGESTION_COST', 0.0)
    g['CONGESTION_BIN'] = _f('KICAD_CONGESTION_BIN', 1.0)
    g['CONGESTION_THRESHOLD'] = _f('KICAD_CONGESTION_THRESHOLD', 0.30)
    g['CONGESTION2'] = {
        'cost': _f('KICAD_CONGESTION2_COST', 0.0),
        'thresh': _f('KICAD_CONGESTION2_THRESHOLD', 0.5),
        'bin': _f('KICAD_CONGESTION2_BIN', 1.0),
        'exempt_r': _f('KICAD_CONGESTION2_EXEMPT_R', 1.0),
        'ramp_top': _f('KICAD_CONGESTION2_RAMP_TOP', 2.0),
    }
    # History-based negotiated congestion (#590, PathFinder): permanent
    # per-cell bump at each conflict event. See history_congestion.py.
    # v2 (contest-targeted): the primary event charges the frontier∩blocker
    # INTERSECTION (the cells one net holds and another stalled against) at
    # the full increment; the v1 whole-footprint rip stamp and raw-frontier
    # charge were measured negative/inert and now default OFF behind weights.
    # #590 SHIPPED DEFAULT = the "v1flat_01" arm: the flat diffuse field at dose
    # 0.1 / cap 0.5, whole-footprint rip stamps at full weight, raw frontier at
    # 0.25, and NO escalation. Best of every arm tested on three corpora; see
    # the promotion note in history_congestion.py for the evidence AND for the
    # caveat (the win is concentrated on congested boards; a pre-registered
    # sets 21-27 test did not clear its own bar). KICAD_HISTORY_COST=0 restores
    # the pre-promotion OFF state for A/B.
    g['HISTORY'] = {
        'cost': _f('KICAD_HISTORY_COST', 0.1),            # 0 = disabled
        'cap': _f('KICAD_HISTORY_CAP', 0.5),              # 0 = uncapped
        'radius': _f('KICAD_HISTORY_RADIUS', 0.25),       # mm beyond half-width (rip stamp)
        'blocked_weight': _f('KICAD_HISTORY_BLOCKED_WEIGHT', 0.25),  # raw frontier residue
        'rip_weight': _f('KICAD_HISTORY_RIP_WEIGHT', 1.0),  # v1 whole-footprint rip stamp
        'escalate': _f('KICAD_HISTORY_ESCALATE', 0.0),   # repeat-contest multiplier (0 = flat)
        'max_cells': _i('KICAD_HISTORY_MAX_CELLS', 500_000),
    }
    # Global planning pass (#589): rough-route every net before detailed
    # routing; predicted paths become soft corridor reservations and inform
    # net ordering. See global_plan.py. Opt-in experiment; 'cost' and 'order'
    # are independent so the A/B arms (reservations-only / order-only / both)
    # are all expressible under the one master gate.
    g['GLOBAL_PLAN'] = {
        'enable': _opt_in('KICAD_GLOBAL_PLAN'),
        'cost': _f('KICAD_GLOBAL_PLAN_COST', 0.0),      # mm-equiv reservation cost; 0 = no reservations (glasgow: 0.15 HURT, 0.05 mildly helped)
        'radius': _f('KICAD_GLOBAL_PLAN_RADIUS', 1.0),  # falloff radius (mm) beyond half-width
        'hweight': _f('KICAD_GLOBAL_PLAN_HWEIGHT', 2.3),  # probe A* weight (glasgow: 2.3 == 4.0 verdict+cost; 1.0 worse AND 72x probes)
        'iters': _i('KICAD_GLOBAL_PLAN_ITERS', 5000),   # static probe cap; <=10k disarms #529 extension
        'order': _s('KICAD_GLOBAL_PLAN_ORDER', 'share'),  # ''=keep | planar (crossings) | share (share-peel, glasgow winner) | share_rev (reversed peel: cliques first) | contended
        'c2': _opt_in('KICAD_GLOBAL_PLAN_C2'),          # seed congestion-v2 demand with rough paths
        # Reservation cost multiplier INSIDE the BGA escape collars (zone +
        # bga_proximity_radius): corridors converge there by NECESSITY, so
        # stacked reservations price the mandatory approach (the #584 zoned
        # lesson, predicted by the issue). 1.0 = off, 0 = no reservations
        # inside collars.
        'zone_scale': _f('KICAD_GLOBAL_PLAN_ZONE_SCALE', 1.0),
        # Clique-aware layer levers (#589 options 1+2): 'layer'=pref arms the
        # SOFT per-net step-cost discount on the plan-assigned layer;
        # 'swaps'=1 MOVES stub copper onto the assigned layer up front via
        # the validated stub-switch path. Either arms the assignment itself
        # (share-graph cliques round-robined across their corridor's viable
        # layers).
        'layer': _s('KICAD_GLOBAL_PLAN_LAYER', ''),     # '' off | 'pref'
        'layer_discount': _f('KICAD_GLOBAL_PLAN_LAYER_DISCOUNT', 0.7),
        # #658: emit each assigned net's ATTRACTION lane on its home layer
        # (the #656 potential does the pulling the soft discount cannot).
        'attract_home': _opt_in('KICAD_GLOBAL_PLAN_ATTRACT_HOME'),
        # #658: multiply NON-home cheap layers' step costs for assigned
        # nets (defection tax; 1.0 = off). The discount alone is proven
        # too weak to pack buses.
        'layer_tax': _f('KICAD_GLOBAL_PLAN_LAYER_TAX', 1.0),
        # #658 river mode: buses (endpoint-pair groups >= 4) route
        # consecutively in lateral order; each member's attraction lane is
        # its predecessor's REALIZED copper (follow-the-leader packing).
        'river': _opt_in('KICAD_GLOBAL_PLAN_RIVER'),
        'river_debug': _opt_in('KICAD_GLOBAL_PLAN_RIVER_DEBUG'),
        # #658 power discipline: per-layer cost MULTIPLIERS for power nets
        # (space list aligned with --layers; -1 forbids; '' = off).
        'power_layer_costs': _s('KICAD_POWER_LAYER_COSTS', ''),
        'swaps': _opt_in('KICAD_GLOBAL_PLAN_SWAPS'),
        # Layer-assignment source: 'clique' = round-robin across each
        # share-clique's viable layers (v1; built for BLIND probes that all
        # pile onto one layer); 'probe' = each net's OWN rough-path
        # majority layer -- only meaningful with SEQ probes, whose paths
        # already spread like the human board's. 'probe' + SWAPS pays the
        # net's first dive mechanically (stub moved before any map build),
        # bypassing the via-economics veto that made the soft discount and
        # attraction inert (adherence 37% attracted vs 38% not).
        'layer_mode': _s('KICAD_GLOBAL_PLAN_LAYER_MODE', 'clique'),
        'debug': _opt_in('KICAD_GLOBAL_PLAN_DEBUG'),    # ordering forensics dump
        # Sequential-aware probing (#589 v2-lite, negotiated-congestion in
        # ONE pass): each successful probe's corridor is stamped into the
        # shared probe map as a soft ghost (compute_ripped_route_costs rows
        # at _SEQ_COST/_SEQ_RADIUS), so later probes SEE earlier probes and
        # spread laterally/across layers instead of all piling onto the
        # cheapest layer (the human-anchor finding: probes 98.7% F.Cu vs
        # human 50% on glasgow -- mutually-blind probes fake the demand map
        # the ordering/layer levers consume).
        'seq': _opt_in('KICAD_GLOBAL_PLAN_SEQ'),
        'seq_cost': _f('KICAD_GLOBAL_PLAN_SEQ_COST', 0.15),
        'seq_radius': _f('KICAD_GLOBAL_PLAN_SEQ_RADIUS', 1.0),
        # Probe-only via cost override (grid steps; 0 = keep config's).
        # config.via_cost 75 =~ 3.75mm-equiv means a blind/soft-ghosted
        # probe will trade many mm of lateral squeeze before paying a via
        # pair -- the human solution pays ~1 via/net to spread layers.
        'probe_via_cost': _i('KICAD_GLOBAL_PLAN_VIA_COST', 0),
        # Owner attraction (#589 v2 handoff): each net's detailed route is
        # ATTRACTED to its own rough corridor via the bus attraction
        # machinery (set_attraction_path; bonus/radius are the config's
        # bus_attraction_* knobs, armed by default). v1 reservations are
        # repulsion-only -- everyone is pushed off everyone's lane but
        # nobody is guided INTO their own, so a near-disjoint negotiated
        # plan had no consumer. Bus corridors take precedence for bus
        # members.
        'attract': _opt_in('KICAD_GLOBAL_PLAN_ATTRACT'),
        # #656 potential-based attraction strength (cost units of the
        # potential's PEAK; 0 = legacy discount-only). Loop-proof by
        # construction (see rust GridRouter.attraction_potential). Sized
        # against via_cost: ~60-70 lets a planned dive outbid a 75 via.
        'attract_potential': _i('KICAD_ATTRACT_POTENTIAL', 0),
        # Front-load list (#589 escape-risk ordering experiment): a file of
        # net names (one per line); listed nets move to the FRONT of their
        # #472 partition block in file order, others keep relative order.
        # Solo forensics: walled nets route human-style (dive early) when
        # the board is open -- they fail only when routed late, after
        # in-run copper consumes their via sites. Oracle test: front-load
        # the known failure set and see if completion follows.
        'order_file': _s('KICAD_GLOBAL_PLAN_ORDER_FILE', ''),
        # #658: probe power/plane nets FIRST so (with SEQ ghosts) every
        # signal probe sees their approximate corridors -- the measured
        # origin of the first plan divergences is the unmodeled power
        # trees' early copper.
        'power_first': _opt_in('KICAD_GLOBAL_PLAN_POWER_FIRST'),
        # Plan-directed escape fanout (#589): for a plan-assigned end still
        # on a non-assigned layer with no swappable stub and no fitting
        # pad-center via, place a DOGBONE (offset via + pad->via trace,
        # tap_pad_with_escalation) BEFORE any obstacle map exists -- the
        # human's escape-first move (0.4-1.8mm surface stub + one via +
        # inner-layer run on exactly the nets our chains fail; wave26: 53
        # of 66 swap declines were 'no via fits at the pad CENTER').
        'escape': _opt_in('KICAD_GLOBAL_PLAN_ESCAPE'),
        'escape_radius': _f('KICAD_GLOBAL_PLAN_ESCAPE_RADIUS', 1.5),
        # Offline-analysis dump (#589 plan-quality scoring): write the full
        # plan state (rough paths, both conflict graphs, layer prefs, the
        # order actually used + the front partition) as JSON to this path
        # right after apply_plan_order; '_EXIT' stops the run there so a
        # dump costs seconds, not a route step.
        'dump': _s('KICAD_GLOBAL_PLAN_DUMP', ''),
        'dump_exit': _opt_in('KICAD_GLOBAL_PLAN_DUMP_EXIT'),
    }

    # --- strings -------------------------------------------------------------
    g['HYBRID_LEG_DEBUG'] = _s('KICAD_HYBRID_LEG_DEBUG')   # truthy; '2' = verbose
    g['LEDGER_TRACE'] = _s('KICAD_LEDGER_TRACE')
    g['STOP_AFTER'] = _s('KICAD_STOP_AFTER')
    g['STOP_FILE'] = _s('KICAD_STOP_FILE')
    g['DUMP_BATCH_KWARGS'] = _s('KICAD_DUMP_BATCH_KWARGS')  # dump file path
    # raw override for underpad outer-ring count ('' = keep the caller's
    # default, which differs by escape method; consumer parses float/inf)
    g['UNDERPAD_OUTER_RINGS'] = _s('KICAD_UNDERPAD_OUTER_RINGS')


refresh()


# --- #653: what was actually set, for the log and the JSON summary ----------
#
# Knobs are read once at import and never echoed, so a wave launched from a
# shell that exported KICAD_* inherited them SILENTLY and its logs were
# indistinguishable from a clean run. Measured cost: a whole A/B baseline
# (orangecrab, "33 issues") was env-contaminated -- same commit, flags, .so and
# python graded 37 clean -- and the cause could only be INFERRED, because
# nothing in the log recorded what was set. These two helpers make a dirty
# baseline detectable post-hoc instead of by re-running.
#
# Reported as the RAW environment, not the parsed attributes above: the point
# is to record what the operator's shell actually handed this process,
# including a knob this build does not know about (a knob from a branch, or
# one deleted since) -- exactly the case a parsed inventory would hide.

# KiCad's own installation variables are NOT knobs: the versioned dirs
# (KICAD9_FOOTPRINT_DIR, KICAD8_3DMODEL_DIR, ...) and the stock data paths say
# nothing about routing behaviour and would bury the real knobs in noise
# inside the GUI, where KiCad sets a dozen of them.
_INSTALL_PREFIX = __import__('re').compile(r'^KICAD\d+_')
_INSTALL_NAMES = frozenset({
    'KICAD_USER_TEMPLATE_DIR', 'KICAD_TEMPLATE_DIR', 'KICAD_STOCK_DATA_HOME',
    'KICAD_DOCUMENTS_HOME', 'KICAD_CONFIG_HOME', 'KICAD_DATA', 'KICAD_PATH',
    'KICAD_RUN_FROM_BUILD_DIR', 'KICAD_ALIAS_SUPPORT',
})


def active_env_knobs() -> dict:
    """{name: value} for every KICAD_*/KRT_* variable set in the environment.

    Installation paths (see _INSTALL_NAMES / KICAD<n>_*) are excluded. Read
    from os.environ LIVE rather than from the import-time snapshot, so a
    harness that sets a knob and calls refresh() is reported accurately.
    """
    out = {}
    for k, v in os.environ.items():
        if not (k.startswith('KICAD') or k.startswith('KRT_')):
            continue
        if _INSTALL_PREFIX.match(k) or k in _INSTALL_NAMES:
            continue
        out[k] = v if len(v) <= 200 else v[:197] + '...'
    return dict(sorted(out.items()))


def env_knobs_line() -> str:
    """One-line, log-greppable inventory of the active knobs."""
    knobs = active_env_knobs()
    if not knobs:
        return "ENV KNOBS: none"
    return "ENV KNOBS: " + ' '.join(f"{k}={v}" for k, v in knobs.items())
