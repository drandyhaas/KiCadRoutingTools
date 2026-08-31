"""JLCPCB fab-capability floors, modelled as selectable cost tiers (issue #237).

A *fab tier* is the manufacturing floor the router shrinks tracks/vias/clearances
DOWN toward when a route needs it. The selected tier is a **floor ladder**:

  - ``standard``  (default, no extra fab cost): the cheap floor. Routing prefers it
                  but **auto-escalates to ``advanced`` (with a warning)** when a
                  fine-pitch fan-out genuinely cannot escape at the standard floor.
  - ``advanced``  (JLC "more costly" tier): the tight floor (0.25/0.15 via, 0.09 mm
                  track/clearance, ...). A **hard** floor — no escalation.

An optional ``--fab-overrides FILE`` (a plain ``key = value`` text file) overlays
the selected tier, replacing **only** the floor values listed in the file (the rest
come from the base tier). Supplying overrides pins the floor to a single **hard**
rung — escalation is disabled, because the file states the user's exact fab limits.

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
# is also the rung 'standard' escalates to. 'pad_hole_to_hole' and the annular
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

TIERS = ('standard', 'advanced')


# --- Process-wide selected tier ----------------------------------------------
# The active tier is a module-global so the dozens of deep ``fab_floors(n)`` call
# sites need not each thread a tier param. It is set once per run, near the start:
# each CLI calls ``set_default_fab_tier`` after arg parsing, and each in-process GUI
# routing action calls ``set_fab_tier_from_config`` at entry. There is no
# save/restore around an engine body — the global simply holds the last value set,
# so a caller that routes without first setting the tier inherits the previous run's
# value. The in-process GUI avoids one tab's custom file leaking into the next by
# re-setting the tier from its own config at the start of every routing action.
# NOTE: process-wide, not thread-local — engines run serially.
_DEFAULT_TIER = 'standard'
_DEFAULT_OVERRIDES = {}
_escalation_warned = set()


def set_default_fab_tier(tier, overrides=None):
    """Set the process-wide active fab tier (and custom overrides). Resets the
    per-run escalation-warning dedupe so a new run warns afresh."""
    global _DEFAULT_TIER, _DEFAULT_OVERRIDES
    _DEFAULT_TIER = tier or 'standard'
    _DEFAULT_OVERRIDES = dict(overrides or {})
    _escalation_warned.clear()


def get_default_fab_tier():
    """Return the active ``(tier, overrides_dict)`` — pass back to
    ``set_default_fab_tier(*prev)`` to restore."""
    return (_DEFAULT_TIER, dict(_DEFAULT_OVERRIDES))


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
        fine_via_rung bool   whether the STANDARD ladder carries its 0.30/0.15
                             intermediate via rung at this count -- the one
                             other layer-count branch in this module. Derived
                             by asking ``fab_floor_ladder`` rather than
                             restating its condition, and pinned to the
                             standard tier with no overrides, which is what the
                             one-argument signature buys. Read
                             ``len(fab_floor_ladder(n))`` for the ACTIVE ladder.
    """
    n = copper_layer_count or 2
    keys = tuple(sorted(_FAB_FLOORS))
    bucket = _layer_bucket(n)
    return FabBucket(bucket=bucket, requested=n, bucketed=(n != bucket),
                     saturated=(bucket == keys[-1]), buckets=keys,
                     fine_via_rung=len(fab_floor_ladder(n, 'standard')) > 2)


def fab_floor_ladder(copper_layer_count, tier=None, overrides=None):
    """Ordered list of floor dicts for the tier: the nominal (preferred) floor
    first, then any escalation rungs (smaller). Routing tries them in order.

      standard           -> [standard, advanced]   (escalates with a warning)
      advanced           -> [advanced]             (hard floor)
      <tier> + overrides -> [<tier> with the file's keys overlaid]  (hard, no escalation)

    Supplying an override file collapses the ladder to one hard rung built from the
    selected base tier, since the file states the user's exact fab limits.
    ``tier=None`` uses the process-wide default set by ``set_default_fab_tier``.
    """
    if tier is None:
        tier = _DEFAULT_TIER
        if overrides is None:
            overrides = _DEFAULT_OVERRIDES
    base = _layer_floors(copper_layer_count)
    if tier not in base:
        raise ValueError(f"unknown fab tier {tier!r} (expected one of {TIERS})")
    if overrides:
        floor = dict(base[tier])
        floor.update({k: v for k, v in overrides.items() if k in floor})
        return [floor]
    if tier == 'standard':
        std = dict(base['standard'])
        adv = dict(base['advanced'])
        rungs = [std]
        # Intermediate fan-out / via-in-pad escape rung: the 0.30/0.15 "fine" via
        # the router used pre-#237. Multilayer boards try it before escalating to
        # the advanced 0.25/0.15, so fine-pitch escapes that fit a 0.30 via keep
        # their previous result instead of jumping straight to the smaller (more
        # costly) advanced via. (2-layer had no smaller-than-standard fine via, and
        # the rung is only added when 0.30 sits strictly between the two floors.)
        # Only the via dia/drill differ from standard; this is NOT the ladder's
        # first rung (fab_floors -> nominal) nor its last (fab_floor_min -> DRC).
        if (copper_layer_count or 2) > 2 and adv['via_diameter'] < 0.30 < std['via_diameter']:
            fine = dict(std)
            fine.update({'via_diameter': 0.30, 'via_drill': 0.15,
                         'annular': round((0.30 - 0.15) / 2.0, 4)})
            rungs.append(fine)
        rungs.append(adv)
        return rungs
    return [dict(base['advanced'])]


def fab_floors(copper_layer_count, tier=None, overrides=None):
    """The NOMINAL (preferred) fab floor for the tier — what routing targets and
    necks down to first. Flat dict of FLOOR_KEYS."""
    return fab_floor_ladder(copper_layer_count, tier, overrides)[0]


def fab_floor_min(copper_layer_count, tier=None, overrides=None):
    """The DEEPEST fab floor the tier can reach (the escalation rung for
    ``standard``; the hard floor for ``advanced``/``custom``). Use this to grade
    DRC, so legitimately-escalated fine geometry isn't false-flagged."""
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
    """The fab floor (deepest reachable value) for a routing parameter, or None if
    the parameter has no fab floor. Uses fab_floor_min so it's the smallest value
    the fab can actually make for the active tier."""
    key = _PARAM_FLOOR_KEY.get(param_name)
    if key is None:
        return None
    return fab_floor_min(copper_layer_count, tier, overrides).get(key)


def count_copper_layers_in_file(pcb_path):
    """Count copper layers in a .kicad_pcb (matches `(0 "F.Cu" signal)`-style layer
    defs, not pad layer lists). Returns 0 on any read error. Stdlib-only."""
    try:
        with open(pcb_path, encoding='utf-8') as f:
            return len(re.findall(r'\(\d+\s+"[^"]*\.Cu"', f.read())) or 0
    except OSError:
        return 0


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
    """CLI guard: pin any routing parameter that is below the fab floor UP to the
    floor, warn, and keep running (issue #237). The fab physically can't make
    sub-floor geometry, so rather than abort the run we clamp each out-of-range
    value to the smallest the selected --fab-tier can make and print a warning.

    Each CLI passes the params it accepts (track_width, clearance, via_size,
    via_drill, hole_to_hole_clearance); None values skip. Returns a
    ``{param_name: pinned_floor}`` dict of the clamps applied (empty if all in
    range) so the caller can write the pinned values back onto its parsed args."""
    viols = check_param_floors(copper_layer_count, tier, overrides, **params)
    pinned = {}
    for name, val, floor in viols:
        print(f"  WARNING: --{name.replace('_', '-')} {val} is below the fab floor {floor} "
              f"for the selected --fab-tier; pinning up to {floor} (the fab can't "
              f"make it smaller). Pass --fab-overrides to declare a smaller fab "
              f"capability, or raise the value to silence this.")
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


_escalation_lever_said = []


def warn_fab_escalation(context):
    """Emit a one-line warning (deduped per run, per context) that a routing step
    dropped below the standard floor to the more-costly advanced floor."""
    if not context or context in _escalation_warned:
        return
    _escalation_warned.add(context)
    print(f"  WARNING: {context}: escalated standard->advanced fab floor "
          f"(0.25/0.15 via etc., more costly to fab); pass --fab-tier advanced to "
          f"silence, or --fab-overrides to pin your own floor (forbids escalation)")
    if not _escalation_lever_said:
        _escalation_lever_said.append(True)
        print("           NOTE: --via-size cannot prevent this. The via is "
              "re-derived from min(pad.size_x, pad.size_y) floored at the fab "
              "ladder, so RAISING --via-size makes the clamp fire more "
              "readily, never less. --fab-overrides collapses the ladder to a "
              "single rung and is the only flag that forbids the escalation.")


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


def add_fab_tier_args(parser):
    """Add ``--fab-tier`` / ``--fab-overrides`` to an argparse parser. Shared by
    every routing/DRC CLI so the flag is identical everywhere."""
    parser.add_argument(
        '--fab-tier', choices=list(TIERS), default='standard',
        help="JLC fab capability floor (default standard). 'standard' = cheap "
             "no-extra-cost floor that auto-escalates to advanced (with a warning) "
             "when a fine-pitch fan-out needs it; 'advanced' = tight 0.25/0.15 via "
             "etc. (more costly), a hard floor.")
    parser.add_argument(
        '--fab-overrides', metavar='FILE', default=None,
        help="Path to a fab-floor override file (key=value lines, e.g. "
             "'via_drill = 0.15') overlaying the selected --fab-tier; only the "
             "listed values are replaced. Supplying it disables escalation (the "
             "floor becomes exactly the file + base tier). See the template "
             "fab_overrides.example.txt for the format and every key.")
    return parser


def set_fab_tier_from_config(config):
    """GUI helper: set the process-wide fab tier from a config / shared-params dict
    carrying 'fab_tier' and (optionally) 'fab_overrides_path'. Tolerates a missing
    or unreadable override file (warns, falls back to the bare tier)."""
    tier = (config.get('fab_tier') or 'standard')
    path = (config.get('fab_overrides_path') or '').strip()
    overrides = {}
    if path:
        try:
            overrides = parse_fab_overrides(path)
        except OSError as exc:
            print(f"WARNING: could not read fab overrides {path}: {exc}")
    set_default_fab_tier(tier, overrides)


def fab_tier_from_args(args):
    """Resolve ``(tier, overrides_dict)`` from parsed args; load the override file
    once. The override file (if any) overlays whichever tier was selected."""
    tier = getattr(args, 'fab_tier', 'standard') or 'standard'
    path = getattr(args, 'fab_overrides', None)
    if path and not os.path.isfile(path):
        raise SystemExit(f"error: --fab-overrides file not found: {path}")
    overrides = parse_fab_overrides(path) if path else {}
    return tier, overrides
