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

import os
import re

# Flat floor keys every tier dict carries. Also the set an override file may set.
FLOOR_KEYS = ('clearance', 'track_width', 'via_diameter', 'via_drill',
              'hole_to_hole', 'pad_hole_to_hole', 'annular')

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
                     'annular': 0.25},
        'advanced': {'clearance': 0.10, 'track_width': 0.10,
                     'via_diameter': 0.25, 'via_drill': 0.15,
                     'hole_to_hole': 0.20, 'pad_hole_to_hole': 0.45,
                     'annular': 0.18},
    },
    4: {
        'standard': {'clearance': 0.10, 'track_width': 0.0889,
                     'via_diameter': 0.45, 'via_drill': 0.20,
                     'hole_to_hole': 0.20, 'pad_hole_to_hole': 0.45,
                     'annular': 0.20},
        'advanced': {'clearance': 0.09, 'track_width': 0.0762,
                     'via_diameter': 0.25, 'via_drill': 0.15,
                     'hole_to_hole': 0.20, 'pad_hole_to_hole': 0.45,
                     'annular': 0.15},
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


def _layer_floors(copper_layer_count):
    return _FAB_FLOORS[2] if (copper_layer_count or 2) <= 2 else _FAB_FLOORS[4]


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
        return [dict(base['standard']), dict(base['advanced'])]
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
    'clearance': 'clearance',
    'via_size': 'via_diameter',
    'via_diameter': 'via_diameter',
    'via_drill': 'via_drill',
    'hole_to_hole_clearance': 'hole_to_hole',
    'hole_to_hole': 'hole_to_hole',
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


def enforce_fab_floors(copper_layer_count, tier=None, overrides=None, **params):
    """CLI guard: raise SystemExit if any routing parameter is below the fab floor
    for the active tier. Each CLI passes the params it accepts (track_width,
    clearance, via_size, via_drill, hole_to_hole_clearance); None values skip."""
    viols = check_param_floors(copper_layer_count, tier, overrides, **params)
    if viols:
        details = "; ".join(f"--{n.replace('_', '-')} {v} is below the fab floor {f}"
                            for n, v, f in viols)
        raise SystemExit(
            f"error: {details}. The fab cannot make these for the selected --fab-tier; "
            f"raise the value(s), or pass a --fab-overrides file declaring a smaller fab "
            f"capability.")


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


def warn_fab_escalation(context):
    """Emit a one-line warning (deduped per run, per context) that a routing step
    dropped below the standard floor to the more-costly advanced floor."""
    if not context or context in _escalation_warned:
        return
    _escalation_warned.add(context)
    print(f"  ⚠ {context}: escalated standard→advanced fab floor "
          f"(0.25/0.15 via etc., more costly to fab); pass --fab-tier advanced to "
          f"silence, or --fab-overrides to pin your own floor (forbids escalation)")


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
