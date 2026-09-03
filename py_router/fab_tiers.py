"""JLCPCB fab-capability floors, modelled as selectable cost tiers (issue #237),
and the ESCALATION POLICY that says how far below a requested size a routing
step may go (#857 / #842, docs/design-rules-proposal.md).

A *fab tier* is the manufacturing floor the router may shrink tracks/vias/
clearances DOWN toward when a route needs it and the policy allows it:

  - ``standard``  (default, no extra fab cost): a HARD floor. Nothing on the
                  board goes below it.
  - ``advanced``  (JLC "more costly" tier): the tight floor (0.25/0.15 via,
                  0.09 mm track/clearance, ...). A hard floor too.
  - ``auto``      the old default: the standard floor first, escalating to
                  ``advanced`` (with a warning, counted in the run summary)
                  when a fine-pitch fan-out or a last-resort via genuinely
                  cannot fit at the standard floor. Opt-in since #857: the
                  default used to escalate silently, 149 times on one board.

An optional ``--fab-overrides FILE`` (a plain ``key = value`` text file) overlays
the selected tier, replacing **only** the floor values listed in the file (the rest
come from the base tier). Supplying overrides pins the floor to a single **hard**
rung -- escalation is disabled, because the file states the user's exact fab limits.

The **escalation policy** (``--escalation``, one shared control in the GUI) is
orthogonal to the tier and bounds every descent site in the engine:

  - ``off``    sizes and clearances are exact. A net that cannot complete at
               them fails and is reported; nothing is narrowed or shrunk.
  - ``board``  (default) a failing net may be retried narrower / with a smaller
               via / tighter clearance, down to the BOARD'S OWN declared floors
               (Board Setup ``rules.min_*``), i.e. what KiCad's DRC accepts. A
               key the board leaves unset falls back to the fab tier floor for
               that key. The output is DRC-clean against the input project by
               construction.
  - ``fab``    additionally may go below the board's floors down to the fab
               tier floor (the pre-#857 reach), disclosed.

Every descent is recorded in a per-run LEDGER (``note_narrowing`` /
``escalation_summary``) that the routing mains put in JSON_SUMMARY and print
once at the end of the run; ``--strict-sizes`` turns a non-empty ledger into a
non-zero exit.

Values are sourced from jlcpcb.com/capabilities (saved at ~/Downloads/pcb_specs.mhtml,
2026-06): standard via 0.20 drill / 0.45 dia, advanced 0.15 / 0.25 ("more costly");
min drill 0.20 vs 0.15; track/clearance 0.10/0.10 (1-2 layer) vs 0.09/0.09 multilayer
(3 mil OK in BGA fan-outs); PTH annular recommended 0.25 (2L) / 0.20 (ML) vs absolute
minimum 0.18 / 0.15; Via hole-to-hole 0.20, Pad hole-to-hole 0.45.

This module is intentionally **stdlib-only** so the lightweight DRC-settings script can
import it without pulling in the PCB parser. ``list_nets`` re-exports the public names,
so ``from list_nets import fab_floors`` keeps working.
"""

import collections
import os
import re

# Flat floor keys every tier dict carries. Also the set an override file may set.
FLOOR_KEYS = ('clearance', 'track_width', 'via_diameter', 'via_drill',
              'hole_to_hole', 'pad_hole_to_hole', 'annular', 'board_edge')

# Which layer BUCKET a floor came from -- see fab_floor_bucket. Deliberately a
# separate object from the floor dict: a floor and its provenance must not be
# confusable, and FLOOR_KEYS is a written-out public contract.
FabBucket = collections.namedtuple(
    'FabBucket',
    ('bucket', 'requested', 'bucketed', 'saturated', 'buckets',
     'fine_via_rung'))

# _FAB_FLOORS[layer_count][tier] -> flat floor dict. Layer count is bucketed to
# 2 (1-2 layer) vs 4 (multilayer). 'standard' preserves the historical floors so a
# bare default run is unchanged; 'advanced' is the JLC tighter/more-costly tier and
# is also the rung 'auto' escalates to. 'pad_hole_to_hole' and the annular
# recommended/abs-min split are per the JLC spec; 'annular' is informational (the
# fan-out via clamp derives its own min annular from the via dia/drill).
_FAB_FLOORS = {
    2: {
        'standard': {'clearance': 0.127, 'track_width': 0.127,
                     'via_diameter': 0.45, 'via_drill': 0.20,
                     'hole_to_hole': 0.20, 'pad_hole_to_hole': 0.45,
                     'annular': 0.25, 'board_edge': 0.20},
        'advanced': {'clearance': 0.10, 'track_width': 0.10,
                     'via_diameter': 0.25, 'via_drill': 0.15,
                     'hole_to_hole': 0.20, 'pad_hole_to_hole': 0.45,
                     'annular': 0.18, 'board_edge': 0.20},
    },
    4: {
        'standard': {'clearance': 0.10, 'track_width': 0.0889,
                     'via_diameter': 0.45, 'via_drill': 0.20,
                     'hole_to_hole': 0.20, 'pad_hole_to_hole': 0.45,
                     'annular': 0.20, 'board_edge': 0.20},
        'advanced': {'clearance': 0.09, 'track_width': 0.0762,
                     'via_diameter': 0.25, 'via_drill': 0.15,
                     'hole_to_hole': 0.20, 'pad_hole_to_hole': 0.45,
                     'annular': 0.15, 'board_edge': 0.20},
    },
}

TIERS = ('standard', 'advanced', 'auto')
ESCALATION_POLICIES = ('off', 'board', 'fab')
DEFAULT_ESCALATION = 'board'

# .kicad_pro board.design_settings.rules key -> FLOOR_KEYS entry, for the
# 'board' policy. min_clearance is included: KiCad enforces it as the one
# absolute clearance floor, so a descent may not cross it either.
BOARD_RULE_TO_FLOOR_KEY = {
    'min_clearance': 'clearance',
    'min_track_width': 'track_width',
    'min_via_diameter': 'via_diameter',
    'min_through_hole_diameter': 'via_drill',
    'min_hole_to_hole': 'hole_to_hole',
    'min_via_annular_width': 'annular',
    'min_copper_edge_clearance': 'board_edge',
}


# --- Process-wide selected tier + policy ------------------------------------
# The active tier is a module-global so the dozens of deep ``fab_floors(n)`` call
# sites need not each thread a tier param. It is set once per run, near the start:
# each CLI calls ``set_default_fab_tier`` after arg parsing, and each in-process GUI
# routing action calls ``set_fab_tier_from_config`` at entry. There is no
# save/restore around an engine body -- the global simply holds the last value set,
# so a caller that routes without first setting the tier inherits the previous run's
# value. The in-process GUI avoids one tab's custom file leaking into the next by
# re-setting the tier from its own config at the start of every routing action.
# NOTE: process-wide, not thread-local -- engines run serially.
_DEFAULT_TIER = 'standard'
_DEFAULT_OVERRIDES = {}
_escalation_warned = set()

# The escalation policy and the board's own declared floors (FLOOR_KEYS
# vocabulary, only the keys the board declares > 0), set per run the same way.
_ESCALATION = DEFAULT_ESCALATION
_BOARD_FLOORS = {}

# The per-run LEDGER: what went below a requested size, where, and by how much.
_LEDGER = {'narrowed': [], 'fab_tier': []}


def set_default_fab_tier(tier, overrides=None):
    """Set the process-wide active fab tier (and custom overrides). Resets the
    per-run escalation-warning dedupe so a new run warns afresh."""
    global _DEFAULT_TIER, _DEFAULT_OVERRIDES
    _DEFAULT_TIER = tier or 'standard'
    _DEFAULT_OVERRIDES = dict(overrides or {})
    _escalation_warned.clear()


def get_default_fab_tier():
    """Return the active ``(tier, overrides_dict)`` -- pass back to
    ``set_default_fab_tier(*prev)`` to restore."""
    return (_DEFAULT_TIER, dict(_DEFAULT_OVERRIDES))


def set_escalation_policy(policy, board_floors=None):
    """Set the process-wide escalation policy and the board's declared floors
    (a FLOOR_KEYS dict, e.g. from ``design_rules.board_floor_dict``). Resets
    the per-run ledger: this marks the start of a run."""
    global _ESCALATION, _BOARD_FLOORS
    policy = policy or DEFAULT_ESCALATION
    if policy not in ESCALATION_POLICIES:
        raise ValueError(f"unknown escalation policy {policy!r} "
                         f"(expected one of {ESCALATION_POLICIES})")
    _ESCALATION = policy
    _BOARD_FLOORS = {k: float(v) for k, v in (board_floors or {}).items()
                     if k in FLOOR_KEYS and v is not None and float(v) > 0}
    reset_ledger()


def get_escalation_policy():
    """``(policy, board_floors_dict)`` -- pass back to
    ``set_escalation_policy(*prev)`` to restore."""
    return (_ESCALATION, dict(_BOARD_FLOORS))


def may_narrow():
    """False under ``--escalation off``: no descent site may shrink anything."""
    return _ESCALATION != 'off'


def board_floors_from_rules(rules):
    """Translate a ``rules.min_*`` dict (.kicad_pro) into FLOOR_KEYS vocabulary.
    Zero / absent keys are UNSET (KiCad writes 0 for 'not configured')."""
    out = {}
    for rk, fk in BOARD_RULE_TO_FLOOR_KEY.items():
        v = (rules or {}).get(rk)
        if isinstance(v, (int, float)) and v > 0:
            out[fk] = float(v)
    return out


# Routing request name -> FLOOR_KEYS entry, for drop_stale_board_floors.
_REQUEST_FLOOR_KEY = {
    'track_width': 'track_width', 'clearance': 'clearance',
    'via_size': 'via_diameter', 'via_diameter': 'via_diameter',
    'via_drill': 'via_drill', 'hole_to_hole_clearance': 'hole_to_hole',
    'board_edge_clearance': 'board_edge',
}


def drop_stale_board_floors(floors, requested, announce=True):
    """A board minimum bounds automatic DESCENTS only when the run's own
    REQUEST respects it. A request already below the declared minimum (the
    stock 0.5 mm via on a project nobody edited, routed with --via-size 0.3)
    is the operator saying that minimum is stale: keeping it would let the
    board forbid descents below a size the run is already using everywhere,
    while pinning the request up would silently reroute the board at the
    stock minimum. Such keys are dropped for the run, and said so."""
    kept = dict(floors or {})
    for name, val in (requested or {}).items():
        key = _REQUEST_FLOOR_KEY.get(name)
        if key is None or val is None or key not in kept:
            continue
        try:
            if float(val) < kept[key] - 1e-9:
                if announce:
                    print(f"  --escalation board: the board's declared minimum "
                          f"{key} {kept[key]:g} mm is above the requested "
                          f"{name.replace('_', ' ')} {float(val):g} mm; treating "
                          f"that minimum as stale for this run (descents bound "
                          f"at the fab floor for it)")
                kept.pop(key)
        except (TypeError, ValueError):
            continue
    return kept


def _layer_bucket(copper_layer_count):
    """The ``_FAB_FLOORS`` key a copper-layer count resolves to.

    THE ONE statement of the bucketing rule, so the bucket
    ``fab_floor_bucket`` reports can never disagree with the floor
    ``_layer_floors`` returns. Derived from the table's own keys rather than
    restating 2/4, so adding a rung to ``_FAB_FLOORS`` re-buckets by itself
    instead of needing a second edit somebody has to remember.

    Byte-identical to the ``(n or 2) <= 2`` rule it replaces for the keys the
    table actually has: 1->2, 2->2, 3->4, 4->4, 6->4, 8->4. Pinned by
    ``tests/test_768_cap_clearance_ceiling.py:580-591``, which must keep
    passing unmodified.
    """
    n = copper_layer_count or 2
    keys = sorted(_FAB_FLOORS)
    return next((k for k in keys if n <= k), keys[-1])


def _layer_floors(copper_layer_count):
    return _FAB_FLOORS[_layer_bucket(copper_layer_count)]


def fab_floor_bucket(copper_layer_count):
    """WHICH layer bucket a count lands in -- the provenance the floor dict
    deliberately does not carry.

    ``fab_floor_min(4)``, ``(6)`` and ``(8)`` return byte-identical dicts,
    because ``_FAB_FLOORS`` has exactly two rungs. That is a MODELLING LIMIT,
    not a fab fact: JLC publishes one multilayer capability column (see the
    module docstring's source line) and this module does not invent the rest.

    A consumer that COMPARES two layer counts cannot otherwise tell "the floor
    genuinely does not move" from "this table cannot see the difference", and
    `placement.options.add_layers` -- the only such consumer -- printed the
    first while meaning the second on every board above 2 copper layers. On a
    6-layer board 121 escape lanes short it said "more layers buy NO extra
    lanes on a face" (issue #700).

    A SEPARATE FUNCTION, not a key on the floor dict: the floor dicts contain
    exactly ``FLOOR_KEYS`` and are written out as-is --
    ``list_nets.effective_floors`` returns one under ``'fab'`` and
    ``read_design_rules`` embeds that whole structure as a documented public
    return -- so a key added there silently joins that contract. Several
    consumers also index the dict directly.

    TAKES ONE ARGUMENT ON PURPOSE. No ``tier``, no ``overrides``: an override
    file must not be able to forge provenance, and giving it no channel is the
    only way to guarantee that rather than promise it. (``fab_floor_ladder``
    can today only REPLACE keys already present, but "cannot today" and
    "cannot by construction" are different guarantees.)

    Returns a ``FabBucket`` -- a NamedTuple rather than a dict so a mistyped
    field raises instead of returning ``None``, which is the silent-default
    failure this whole function exists to end (see ``options.deficit_totals``
    for the one that cost "38 faces are short" -> "0").

        bucket        int    the ``_FAB_FLOORS`` key used (2 or 4 today)
        requested     int    the count asked for, after the ``or 2`` default
        bucketed      bool   requested != bucket: the floor is a PROXY
        saturated     bool   bucket is the table's LAST, so every higher layer
                             count returns this identical floor
        buckets       tuple  every bucket the table models, ascending
        fine_via_rung bool   whether the AUTO ladder carries its 0.30/0.15
                             intermediate via rung at this count -- the one
                             other layer-count branch in this module. Derived
                             by asking ``fab_floor_ladder`` rather than
                             restating its condition, and pinned to the
                             auto tier with no overrides, which is what the
                             one-argument signature buys. Read
                             ``len(fab_floor_ladder(n))`` for the ACTIVE ladder.
    """
    n = copper_layer_count or 2
    keys = tuple(sorted(_FAB_FLOORS))
    bucket = _layer_bucket(n)
    return FabBucket(bucket=bucket, requested=n, bucketed=(n != bucket),
                     saturated=(bucket == keys[-1]), buckets=keys,
                     fine_via_rung=len(fab_floor_ladder(n, 'auto', {})) > 2)


def _tier_rungs(copper_layer_count, tier, overrides):
    """The tier's rungs BEFORE the escalation policy is applied."""
    base = _layer_floors(copper_layer_count)
    if tier not in TIERS:
        raise ValueError(f"unknown fab tier {tier!r} (expected one of {TIERS})")
    if overrides:
        floor = dict(base[tier if tier != 'auto' else 'standard'])
        floor.update({k: v for k, v in overrides.items() if k in floor})
        return [floor]
    if tier == 'standard':
        return [dict(base['standard'])]
    if tier == 'advanced':
        return [dict(base['advanced'])]
    std = dict(base['standard'])
    adv = dict(base['advanced'])
    rungs = [std]
    # Intermediate fan-out / via-in-pad escape rung: the 0.30/0.15 "fine" via
    # the router used pre-#237. Multilayer boards try it before escalating to
    # the advanced 0.25/0.15, so fine-pitch escapes that fit a 0.30 via keep
    # their previous result instead of jumping straight to the smaller (more
    # costly) advanced via. (2-layer had no smaller-than-standard fine via, and
    # the rung is only added when 0.30 sits strictly between the two floors.)
    if (copper_layer_count or 2) > 2 and adv['via_diameter'] < 0.30 < std['via_diameter']:
        fine = dict(std)
        fine.update({'via_diameter': 0.30, 'via_drill': 0.15,
                     'annular': round((0.30 - 0.15) / 2.0, 4)})
        rungs.append(fine)
    rungs.append(adv)
    return rungs


def _apply_board_floors(rungs):
    """'board' policy: no rung may sit below a floor the board declares.
    Raising a rung can make two rungs identical; collapse those."""
    if not _BOARD_FLOORS:
        return rungs
    out = []
    for r in rungs:
        rr = dict(r)
        for k, v in _BOARD_FLOORS.items():
            if k in rr and rr[k] < v:
                rr[k] = v
        if not out or rr != out[-1]:
            out.append(rr)
    return out


def fab_floor_ladder(copper_layer_count, tier=None, overrides=None):
    """Ordered list of floor dicts for the tier under the active escalation
    policy: the nominal (preferred) floor first, then any escalation rungs
    (smaller). Routing tries them in order.

      standard           -> [standard]                     (hard)
      advanced           -> [advanced]                     (hard)
      auto               -> [standard, (fine), advanced]   (escalates, warned + counted)
      <tier> + overrides -> [<tier> with the file's keys overlaid]  (hard)

    Under ``--escalation board`` every rung is raised to the board's own
    declared floors (a rung the board forbids is dropped by collapsing into
    the rung above it). Under ``off`` and ``fab`` the rungs are the tier's.
    ``tier=None`` uses the process-wide default set by ``set_default_fab_tier``.
    """
    if tier is None:
        tier = _DEFAULT_TIER
        if overrides is None:
            overrides = _DEFAULT_OVERRIDES
    rungs = _tier_rungs(copper_layer_count, tier, overrides)
    if _ESCALATION == 'board':
        rungs = _apply_board_floors(rungs)
    return rungs


def escalation_rungs(copper_layer_count, tier=None, overrides=None):
    """The rungs a DESCENT site may step down through: ``fab_floor_ladder``
    under ``board`` / ``fab``, and NOTHING under ``off`` (an empty list --
    every site that walks it then keeps the requested geometry and reports).
    Every site that shrinks a via or narrows a track walks this, never the
    ladder directly, so the policy is one decision in one place."""
    if not may_narrow():
        return []
    return fab_floor_ladder(copper_layer_count, tier, overrides)


def fab_floors(copper_layer_count, tier=None, overrides=None):
    """The NOMINAL (preferred) fab floor for the tier -- what routing targets and
    necks down to first. Flat dict of FLOOR_KEYS (raised to the board's own
    floors under ``--escalation board``)."""
    return fab_floor_ladder(copper_layer_count, tier, overrides)[0]


def fab_floor_min(copper_layer_count, tier=None, overrides=None):
    """The DEEPEST fab floor the tier can reach (the escalation rung for
    ``auto``; the hard floor for ``standard``/``advanced``/overrides). Use this
    to grade DRC, so legitimately-escalated fine geometry isn't false-flagged."""
    return fab_floor_ladder(copper_layer_count, tier, overrides)[-1]


# Routing-parameter name (CLI flag / GUI control) -> fab floor dict key. Used to
# stop a track/clearance/via/drill/hole param being set below the fab can make.
_PARAM_FLOOR_KEY = {
    'track_width': 'track_width',
    # The plane-repair strap NECK floor (--min-track-width) is still copper the
    # fab must etch, so it floors at the same track minimum (#513 item 9:
    # allwinner_h3_ddr3 shipped 217 GND straps necked to 0.0889 on a 2-layer
    # board because only --track-width was enforced).
    'min_track_width': 'track_width',
    'clearance': 'clearance',
    'via_size': 'via_diameter',
    'via_diameter': 'via_diameter',
    'via_drill': 'via_drill',
    'hole_to_hole_clearance': 'hole_to_hole',
    'hole_to_hole': 'hole_to_hole',
    # Copper-to-board-edge: JLC routed-outline min 0.2 mm. A board declaring a
    # smaller (or 0.0) min_copper_edge_clearance is pinned up so routed copper does
    # not run to the milled edge (#439 follow-up).
    'board_edge_clearance': 'board_edge',
    'board_edge': 'board_edge',
    # The diff-pair P/N gap is copper-to-copper spacing, so it floors at the same
    # copper-clearance minimum (parity with the GUI, which floors it at 'clearance').
    'diff_pair_gap': 'clearance',
}


def fab_floor_for_param(param_name, copper_layer_count, tier=None, overrides=None):
    """The FAB floor (deepest reachable value) for a routing parameter, or None
    if the parameter has no fab floor: the smallest value the fab can actually
    make for the active tier. Deliberately the raw tier floor, NOT the
    board-raised one: a board's declared minimum bounds automatic DESCENTS
    (see set_policy_from_args), it does not pin an explicit request up."""
    key = _PARAM_FLOOR_KEY.get(param_name)
    if key is None:
        return None
    if tier is None:
        tier = _DEFAULT_TIER
        if overrides is None:
            overrides = _DEFAULT_OVERRIDES
    return _tier_rungs(copper_layer_count, tier, overrides)[-1].get(key)


def count_copper_layers_in_file(pcb_path):
    """Count copper layers in a .kicad_pcb (matches `(0 "F.Cu" signal)`-style layer
    defs, not pad layer lists). Returns 0 on any read error. Stdlib-only."""
    # `None` is the in-memory case, and it is the one this counter exists to
    # hand over to `count_copper_layers_in_data` -- a live GUI board has
    # `source_path is None`, not `''`. Without this it raised TypeError out of
    # `open`, which `placement.options.capacity_options` reports as INTERNAL
    # ERROR, a channel reserved for genuine crashes. Worse than the skip it
    # replaced.
    if not pcb_path:
        return 0
    try:
        with open(pcb_path, encoding='utf-8') as f:
            return len(re.findall(r'\(\d+\s+"[^"]*\.Cu"', f.read())) or 0
    except (OSError, TypeError, ValueError):
        return 0


def count_copper_layers_in_data(pcb_data):
    """Copper layers on an IN-MEMORY board -- the twin of
    ``count_copper_layers_in_file``.

    The file twin returns 0 for a live GUI board whose ``board_info.copper_layers``
    is perfectly good, which is why ``placement.options.add_layers`` skipped
    entirely on one ("could not count this board's copper layers") and why
    ``fanout_clearance.resolve_drill_floors`` says so in its own docstring.

    Returns 0 when the list is missing or empty -- the SAME "I could not look"
    value its file twin returns, and deliberately NOT a 2-or-4 guess. The
    ~25 sites that open-code this expression each spell their own fallback and
    they DISAGREE (``or 2`` in check_drc and fanout_clearance, ``or 4`` and
    ``else 2`` in the fanouts), so folding them into one helper would silently
    re-bucket boards through ``_layer_bucket`` and change routed output. The
    fallback stays at the call site, where it is visible.

    Filters on ``.Cu``, matching ``fanout_clearance``. ``check_drc`` does not
    filter; the two can only disagree on a layer list mixing copper and
    non-copper names, which neither parse path produces (the text one requires
    '.Cu' in the name, the pcbnew one maps copper ids through a canonical
    table).
    """
    cu = getattr(getattr(pcb_data, 'board_info', None), 'copper_layers', None)
    return len([l for l in (cu or ()) if str(l).endswith('.Cu')])


def min_via_center_distance(via_diameter, clearance, via_drill,
                            hole_to_hole=0.0):
    """Minimum centre-to-centre distance between two via drills (#491).

    Two INDEPENDENT rules bind and using only the first ships legal copper with
    unmanufacturable holes:

      copper: via_diameter + clearance
      drill : drill/2 + drill/2 + hole_to_hole  ==  via_drill + hole_to_hole

    On lvds_rx1_10 (0.3 via / 0.2 drill / 0.09 clearance, board
    min_hole_to_hole 0.3) the copper rule asks 0.39mm and the drill rule 0.5mm,
    so a pair placed to the copper rule sits 0.088mm inside the fab's drill
    spacing. The router READ the board constraint correctly and then never
    applied it to via placement.

    Lifted here from ``diff_pair_routing._min_via_center_distance`` so a
    stdlib-only, parser-free consumer can reach it. The placement escape ledger
    needs exactly this arithmetic, and importing ``diff_pair_routing`` from
    ``placement/escape.py`` costs a measured **+126 modules** -- including the
    Rust ``grid_router``, numpy, ``kicad_parser`` and ``obstacle_map`` -- into
    a module whose own docstring says "numpy only; no networkx in the placement
    stack" and which ``board_brief`` and ``routability.health`` import on every
    run. (Roughly half a second, but the module count is the
    machine-independent half, so that is the number quoted.) From here it is
    +1 module / ~4ms from a bare ``placement.escape`` import, and +0 once
    ``list_nets`` is loaded -- which the ledger's own ``lane_pitch`` does.

    VALUES IN, not a config object: ``py_placer`` has no ``GridRouteConfig`` to
    hand it, and a values-in signature is what lets both fronts share one rule
    rather than one of them re-deriving it.

    ``hole_to_hole`` of ``None`` means NO DRILL RULE DECLARED and yields the
    copper rule alone -- inherited verbatim from the adapter's long-standing
    ``getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0``. Worth naming
    because it is the permissive direction on the constraint this function
    exists to enforce: every caller in the tree resolves the value through
    ``list_nets.resolve_cli_floor`` or the GUI's floored spin control, and
    ``GridRouteConfig.hole_to_hole_clearance`` defaults to 0.20, so no reachable
    path passes None today. A future caller that can must pass a real floor.
    """
    return max(float(via_diameter) + float(clearance),
               float(via_drill) + float(hole_to_hole or 0.0))


def enforce_fab_floors(copper_layer_count, tier=None, overrides=None, **params):
    """CLI guard: pin any routing parameter that is below the floor UP to the
    floor, warn, and keep running (issue #237). The fab physically can't make
    sub-floor geometry (and under ``--escalation board`` the board's own
    minimum is a floor too), so rather than abort the run we clamp each
    out-of-range value to the floor and print a warning.

    Each CLI passes the params it accepts (track_width, clearance, via_size,
    via_drill, hole_to_hole_clearance); None values skip. Returns a
    ``{param_name: pinned_floor}`` dict of the clamps applied (empty if all in
    range) so the caller can write the pinned values back onto its parsed args."""
    viols = check_param_floors(copper_layer_count, tier, overrides, **params)
    pinned = {}
    for name, val, floor in viols:
        print(f"  WARNING: --{name.replace('_', '-')} {val} is below the floor {floor} "
              f"(the selected --fab-tier, or the board's own minimum under "
              f"--escalation board); pinning up to {floor}. Pass --fab-overrides "
              f"to declare a smaller fab capability, or raise the value to silence this.")
        pinned[name] = floor
    return pinned


def check_param_floors(copper_layer_count, tier=None, overrides=None, **params):
    """Return a list of (name, value, floor) for any param below its fab floor.
    Params whose value is None or which have no fab floor are skipped."""
    viols = []
    for name, val in params.items():
        if val is None:
            continue
        floor = fab_floor_for_param(name, copper_layer_count, tier, overrides)
        if floor is not None and val < floor - 1e-9:
            viols.append((name, val, floor))
    return viols


# --- The ledger --------------------------------------------------------------

def reset_ledger():
    _LEDGER['narrowed'] = []
    _LEDGER['fab_tier'] = []
    _escalation_warned.clear()
    _escalation_lever_said.clear()


def note_narrowing(net_id, kind, requested, delivered, site, count=1, net_name=None):
    """Record that ``site`` delivered ``kind`` (track_width / via_diameter /
    hole_size / clearance) at ``delivered`` where ``requested`` was asked, on
    net ``net_id``. No-op when nothing actually shrank."""
    try:
        if requested is None or delivered is None or delivered >= requested - 1e-9:
            return
    except TypeError:
        return
    _LEDGER['narrowed'].append({
        'net': int(net_id) if net_id is not None else None,
        'net_name': net_name,
        'kind': kind, 'requested': round(float(requested), 4),
        'delivered': round(float(delivered), 4), 'site': site,
        'count': int(count),
    })


def escalation_summary():
    """The per-run ledger as JSON-ready data (the JSON_SUMMARY ``design_rules``
    block). ``count`` is the number of recorded narrowings; ``fab_tier_escalations``
    the number of standard->advanced descents (only possible under tier auto)."""
    rows = list(_LEDGER['narrowed'])
    by_kind = {}
    for r in rows:
        k = r['kind']
        v = r['delivered']
        by_kind[k] = v if k not in by_kind else min(by_kind[k], v)
    return {
        'escalation_policy': _ESCALATION,
        'fab_tier': _DEFAULT_TIER,
        'board_floors': dict(_BOARD_FLOORS),
        'count': sum(r['count'] for r in rows),
        'nets': sorted({r['net'] for r in rows if r['net'] is not None}),
        'narrowed': rows,
        'min_delivered': by_kind,
        'fab_tier_escalations': len(_LEDGER['fab_tier']),
        'fab_tier_contexts': list(_LEDGER['fab_tier']),
    }


def escalation_report_line():
    """The ONE end-of-run line a reader can act on, or '' when nothing moved."""
    s = escalation_summary()
    if not s['count'] and not s['fab_tier_escalations']:
        return ''
    parts = []
    if s['count']:
        kinds = ', '.join(f"smallest {k.replace('_', ' ')} {v:g} mm"
                          for k, v in sorted(s['min_delivered'].items()))
        parts.append(f"{s['count']} feature(s) on {len(s['nets'])} net(s) delivered below "
                     f"the requested size ({kinds})")
    if s['fab_tier_escalations']:
        parts.append(f"{s['fab_tier_escalations']} fab-tier escalation(s) to advanced")
    return (f"Design rules [--escalation {s['escalation_policy']}, --fab-tier "
            f"{s['fab_tier']}]: " + '; '.join(parts) +
            ". Details: JSON_SUMMARY design_rules.")


_escalation_lever_said = []


def warn_fab_escalation(context):
    """Emit a one-line warning (deduped per run, per context) that a routing step
    dropped below the standard floor to the more-costly advanced floor, and
    count it in the ledger (every call counts, deduped or not)."""
    if not context:
        return
    _LEDGER['fab_tier'].append(context)
    if context in _escalation_warned:
        return
    _escalation_warned.add(context)
    print(f"  WARNING: {context}: escalated standard->advanced fab floor "
          f"(0.25/0.15 via etc., more costly to fab); --fab-tier auto permitted "
          f"it. Pass --fab-tier standard (the default) for a hard floor, or "
          f"--fab-overrides to pin your own floor")
    if not _escalation_lever_said:
        _escalation_lever_said.append(True)
        print("           NOTE: --via-size cannot prevent this. The via is "
              "re-derived from min(pad.size_x, pad.size_y) floored at the fab "
              "ladder, so RAISING --via-size makes the clamp fire more "
              "readily, never less. A hard tier or --fab-overrides is what "
              "forbids the escalation.")


# --- Override file + argparse helpers ----------------------------------------

def parse_fab_overrides(path):
    """Parse a human-editable fab-floor override file into a {key: float} dict.

    Lines are ``key = value`` / ``key: value`` / ``key value``; ``#`` starts a
    comment; blank lines ignored. Unknown keys, non-numeric or non-positive values
    are warned and skipped.
    """
    overrides = {}
    with open(path, encoding='utf-8') as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue
            parts = re.split(r'\s*[=:]\s*|\s+', line, maxsplit=1)
            if len(parts) != 2:
                print(f"WARNING: fab-overrides {path}:{lineno}: cannot parse "
                      f"{raw.strip()!r}")
                continue
            key, val = parts[0], parts[1]
            if key not in FLOOR_KEYS:
                print(f"WARNING: fab-overrides {path}:{lineno}: unknown key {key!r} "
                      f"(known: {', '.join(FLOOR_KEYS)})")
                continue
            try:
                fval = float(val)
            except ValueError:
                print(f"WARNING: fab-overrides {path}:{lineno}: {val!r} is not a number")
                continue
            if fval <= 0:
                print(f"WARNING: fab-overrides {path}:{lineno}: {key} must be > 0 "
                      f"(got {fval})")
                continue
            overrides[key] = fval
    return overrides


def add_fab_tier_args(parser, *, include_escalation=True):
    """Add ``--fab-tier`` / ``--fab-overrides`` (and, for the routing CLIs,
    ``--escalation`` / ``--strict-sizes``) to an argparse parser. Shared by
    every routing/DRC CLI so the flags are identical everywhere."""
    parser.add_argument(
        '--fab-tier', choices=list(TIERS), default='standard',
        help="JLC fab capability floor (default standard). 'standard' = the cheap "
             "no-extra-cost floor, HARD; 'advanced' = tight 0.25/0.15 via etc. (more "
             "costly), hard; 'auto' = standard, escalating to advanced (warned and "
             "counted in the run summary) when a fine-pitch fan-out or last-resort "
             "via cannot fit at the standard floor -- the pre-#857 behaviour, now opt-in.")
    parser.add_argument(
        '--fab-overrides', metavar='FILE', default=None,
        help="Path to a fab-floor override file (key=value lines, e.g. "
             "'via_drill = 0.15') overlaying the selected --fab-tier; only the "
             "listed values are replaced. Supplying it disables escalation (the "
             "floor becomes exactly the file + base tier). See the template "
             "fab_overrides.example.txt for the format and every key.")
    if include_escalation:
        parser.add_argument(
            '--escalation', choices=list(ESCALATION_POLICIES), default=DEFAULT_ESCALATION,
            help="How far below a REQUESTED size a failing net may be retried "
                 "(default board). 'off': never -- sizes and clearances are exact, "
                 "a net that cannot complete at them fails and is reported. 'board': "
                 "down to the board's own declared floors (Board Setup rules.min_*; "
                 "an unset key falls back to the fab tier floor), i.e. what KiCad's "
                 "DRC accepts. 'fab': down to the fab tier floor, below the board's "
                 "own minimums. Every descent is counted in JSON_SUMMARY design_rules.")
        parser.add_argument(
            '--strict-sizes', action='store_true',
            help="Exit non-zero (3) when any feature was delivered below its requested "
                 "size or a fab-tier escalation fired, so a harness needs no grep.")
    return parser


def set_fab_tier_from_config(config):
    """GUI helper: set the process-wide fab tier, custom overrides AND the
    escalation policy from a config / shared-params dict carrying 'fab_tier',
    'fab_overrides_path', 'escalation' and 'board_floors' (a FLOOR_KEYS dict
    the GUI reads off the live board). Tolerates a missing or unreadable
    override file (warns, falls back to the bare tier)."""
    tier = (config.get('fab_tier') or 'standard')
    path = (config.get('fab_overrides_path') or '').strip()
    overrides = {}
    if path:
        try:
            overrides = parse_fab_overrides(path)
        except OSError as exc:
            print(f"WARNING: could not read fab overrides {path}: {exc}")
    set_default_fab_tier(tier, overrides)
    floors = drop_stale_board_floors(
        config.get('board_floors') or {},
        {k: config.get(k) for k in ('track_width', 'clearance', 'via_size',
                                    'via_drill', 'hole_to_hole_clearance',
                                    'board_edge_clearance')})
    set_escalation_policy(config.get('escalation') or DEFAULT_ESCALATION, floors)


def fab_tier_from_args(args):
    """Resolve ``(tier, overrides_dict)`` from parsed args; load the override file
    once. The override file (if any) overlays whichever tier was selected."""
    tier = getattr(args, 'fab_tier', 'standard') or 'standard'
    path = getattr(args, 'fab_overrides', None)
    if path and not os.path.isfile(path):
        raise SystemExit(f"error: --fab-overrides file not found: {path}")
    overrides = parse_fab_overrides(path) if path else {}
    return tier, overrides


def set_policy_from_args(args, pcb_path=None):
    """CLI helper: ``set_escalation_policy`` from ``--escalation`` plus the
    board's own ``rules.min_*`` read from the sibling .kicad_pro of
    ``pcb_path`` (stdlib JSON read; no parser)."""
    import json
    floors = {}
    if pcb_path:
        pro = os.path.splitext(pcb_path)[0] + '.kicad_pro'
        try:
            with open(pro, encoding='utf-8') as f:
                proj = json.load(f)
            rules = ((proj.get('board') or {}).get('design_settings') or {}).get('rules') or {}
            floors = board_floors_from_rules(rules)
        except (OSError, ValueError, AttributeError):
            floors = {}
    policy = getattr(args, 'escalation', None) or DEFAULT_ESCALATION
    if policy == 'board':
        floors = drop_stale_board_floors(
            floors, {k: getattr(args, k, None) for k in _REQUEST_FLOOR_KEY})
    set_escalation_policy(policy, floors)
    return floors
