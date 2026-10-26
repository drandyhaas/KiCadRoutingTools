"""JLCPCB fab-capability floors, modelled as selectable cost tiers (issue #237),
and the ESCALATION POLICY that says how far below a requested size a routing
step may go (#857 / #842, docs/design-rules-proposal.md).

A *fab tier* is the manufacturing floor the router may shrink tracks/vias/
clearances DOWN toward when a route needs it and the policy allows it:

  - ``standard``  (no extra fab cost): a HARD floor. Nothing on the board
                  goes below it. Opt-in.
  - ``advanced``  (JLC "more costly" tier): the tight floor (0.25/0.15 via,
                  0.09 mm track/clearance, ...). A hard floor too. Opt-in.
  - ``auto``      (default, ``routing_defaults.FAB_TIER``): the standard floor
                  first, escalating to ``advanced`` (with a warning, counted
                  in the run summary) when a fine-pitch fan-out, a plane tap
                  or a last-resort via genuinely cannot fit at the standard
                  floor. Completion first (#857/#530): every escalation is
                  disclosed -- it used to happen silently, 149 times on one
                  board.

An optional ``--fab-overrides FILE`` (a plain ``key = value`` text file) overlays
the selected tier, replacing **only** the floor values listed in the file (the rest
come from the base tier). Supplying overrides pins the floor to a single **hard**
rung -- escalation is disabled, because the file states the user's exact fab limits.

The **escalation policy** (``--escalation``, one shared control in the GUI) is
orthogonal to the tier and bounds every descent site in the engine:

  - ``off``    sizes and clearances are exact. A net that cannot complete at
               them fails and is reported; nothing is narrowed or shrunk,
               with one bounded exception: a wide (power / impedance) net may
               neck below its width for at most
               ``routing_defaults.PAD_ENTRY_NECK_MM`` (2 mm) of routed path
               at a pad entry -- the stretch a wide rail cannot keep into a
               narrow pin: a whole edge that short, the last stretch into the
               pin of a longer edge whose rest keeps the net's width (#1452),
               or one via-to-pad tap. A longer neck fails the edge, and the
               failure names the allowance and the edge's span. A power net's under-width copper is in the ledger
               (route.py's ``power copper shipped under width`` row), so
               ``--strict-sizes`` still sees it.
  - ``board``  a failing net may be retried narrower / with a smaller
               via / tighter clearance, down to the BOARD'S OWN declared floors
               (Board Setup ``rules.min_*``). A key the board leaves unset
               falls back to the fab tier floor for that key, except clearance,
               which falls back to the Default class (#1160). Non-Default nets
               never descend below their own class (``GridRouteConfig.
               rule_floors``). What KiCad then accepts depends on the project
               writeback:
                 * WITH it (the default), a clearance descent below the
                   Default class lowers the OUTPUT project's Default class to
                   the clearance used, so the descended copper is DRC-clean
                   against the project it ships with -- not against the input
                   project when ``rules.min_clearance`` sits below the Default
                   class.
                 * WITHOUT it (``--no-fix-drc-settings``, or the GUI's "Fix DRC
                   settings after routing" unticked) the project is left as it
                   was, so every floor is what KiCad grades the INPUT project
                   at. A rule the project leaves out is graded at KiCad's
                   factory value (``KICAD_FACTORY_RULES``: track 0.2, via
                   0.5 / drill 0.3, hole-to-hole 0.25, copper-to-edge 0.5;
                   #1287, :func:`graded_board_floors`), and the clearance
                   floor is the Default class, floored at
                   ``rules.min_clearance`` (#1262,
                   :func:`floor_board_clearance_at_class`). A run whose own
                   request is already below one of these (an explicit
                   ``--clearance`` / ``--clearance-ceiling``, a
                   ``--track-width`` under the floor) keeps the request as
                   that key's floor, and nothing of that key descends.
                   ``.kicad_dru`` layer rules and pad clearance overrides keep
                   their usual precedence: they replace the base clearance
                   where they apply.
  - ``fab``    (default) additionally may go below the board's floors down to
               the fab tier floor (the pre-#857 reach), disclosed.

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

import routing_defaults as defaults
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
# The defaults live in routing_defaults (ONE place, read by the CLIs and the
# GUI alike); these names are the module's view of them.
DEFAULT_TIER = defaults.FAB_TIER
DEFAULT_ESCALATION = defaults.ESCALATION

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

#: KiCad's factory value (mm) for each rule BOARD_RULE_TO_FLOOR_KEY reads: what
#: KiCad loads, and grades the board at, for a rule the .kicad_pro leaves out
#: (#1287). These are the DEFAULT_* constants BOARD_DESIGN_SETTINGS initialises
#: from, read off pcbnew 10.0 (fix_kicad_drc_settings.pcbnew_rule_defaults reads
#: the same names live); the annular ring has no constant of its own and KiCad
#: derives it, (0.5 - 0.3) / 2. A fixed table, not a pcbnew probe, so a CLI
#: run's copper does not depend on which KiCad its interpreter carries. pcbnew
#: 9.0 differs in one key, min_track_width 0.0; 0.2 is the stricter of the two.
KICAD_FACTORY_RULES = {
    'min_clearance': 0.0,
    'min_track_width': 0.2,
    'min_via_diameter': 0.5,
    'min_through_hole_diameter': 0.3,
    'min_hole_to_hole': 0.25,
    'min_via_annular_width': 0.1,
    'min_copper_edge_clearance': 0.5,
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
_DEFAULT_TIER = DEFAULT_TIER
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
    _DEFAULT_TIER = tier or DEFAULT_TIER
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


#: The ``kicad_routing_tools`` record that says ``rules.min_clearance`` holds a
#: PAD clearance override, not a board minimum (#1369):
#: ``{"rule": <the min_clearance the #530 cap wrote, mm>,
#:    "board": <what the board's own min_clearance reads without it, mm>}``.
#: Written by the DRC-floor writebacks (fix_kicad_drc_settings
#: ``record_pad_override_floor`` / ``record_live_pad_override_floor``, through
#: tool_records); read by :func:`board_declared_min_clearance`.
PAD_OVERRIDE_FLOOR_KEY = "pad_override_min_clearance"


def board_declared_min_clearance(min_clearance, records):
    """``rules.min_clearance`` as the BOARD declares it, for
    ``--escalation board`` (#1369).

    The writeback caps ``rules.min_clearance`` at the smallest pad clearance
    override the router honoured (#530), because KiCad floors an override
    there. That number is a property of one part, not a minimum the board
    declares, yet the escalation resolver read it as "the board's own
    minimum" and rescued nets that carry no override down to it. When the
    project's ``records`` (the ``kicad_routing_tools`` namespace, read through
    tool_records) carry :data:`PAD_OVERRIDE_FLOOR_KEY` and the rule still
    holds the value that record says the cap wrote, the board's own minimum is
    the record's ``board`` value. Any other rule value -- one a later step or
    the user changed -- is the board's, and is returned as it is."""
    rec = (records or {}).get(PAD_OVERRIDE_FLOOR_KEY)
    if not isinstance(rec, dict) or not isinstance(min_clearance, (int, float)):
        return min_clearance
    try:
        rule, board = float(rec.get("rule")), float(rec.get("board"))
    except (TypeError, ValueError):
        return min_clearance
    if board > 0 and abs(float(min_clearance) - rule) <= 5e-6:
        return board
    return min_clearance


def board_floors_from_rules(rules, default_class_clearance=None):
    """Translate a ``rules.min_*`` dict (.kicad_pro) into FLOOR_KEYS vocabulary.
    Zero / absent keys are UNSET (KiCad writes 0 for 'not configured').

    ``default_class_clearance`` (#1160) is the Default net class's clearance:
    the clearance floor when ``min_clearance`` is unset. A board that leaves
    Board Setup's minimum at 0 still grades every Default net at its class,
    so ``--escalation board`` must not descend below it. interf_u
    (min_clearance 0, class 0.254) rescued 9 gaps down to 0.127 under board.
    A DECLARED min_clearance below the class still wins here; when the
    project writeback will not run, :func:`board_policy_floors` raises the
    clearance floor to the class as well (#1262)."""
    out = {}
    for rk, fk in BOARD_RULE_TO_FLOOR_KEY.items():
        v = (rules or {}).get(rk)
        if isinstance(v, (int, float)) and v > 0:
            out[fk] = float(v)
    if 'clearance' not in out and isinstance(default_class_clearance, (int, float)) \
            and default_class_clearance > 0:
        out['clearance'] = float(default_class_clearance)
    return out


def graded_board_floors(rules, announce=False):
    """What KiCad grades an UNCHANGED project's ``rules.min_*`` at, in
    FLOOR_KEYS vocabulary (#1287): a rule the project declares at its value,
    and one it leaves out at KiCad's factory value
    (:data:`KICAD_FACTORY_RULES`), which is what KiCad loads for it. A
    declared 0 is KiCad's "no check" and gives no floor. ``rules`` empty (a
    board with no project) leaves every rule out.

    :func:`board_floors_from_rules` reads the same dict for when the project
    writeback WILL run. There an unset rule bounds nothing, because the
    writeback sets it to the routed floor. Without the writeback the input
    project ships as it is, and an unset ``min_track_width`` is graded at 0.2
    on KiCad 10: a 4-layer board rescued nets at the 0.0889 fab floor under
    ``--escalation board`` and kicad-cli flagged them all (#1287)."""
    out, factory = {}, []
    rules = rules or {}
    for rk, fk in BOARD_RULE_TO_FLOOR_KEY.items():
        v = rules.get(rk)
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            v = KICAD_FACTORY_RULES.get(rk) or 0.0
            if v > 0:
                factory.append(f"{rk} at {v:g} mm")
        if v > 0:
            out[fk] = float(v)
    if announce and factory:
        print(f"  --escalation board, project writeback off: the project leaves "
              f"{len(factory)} rule(s) unset, which KiCad grades at its factory "
              f"value ({', '.join(factory)}); descents stop there (#1287)")
    return out


def project_default_class_clearance(proj):
    """The Default net class's clearance in a .kicad_pro dict, or None."""
    for cls in ((proj or {}).get('net_settings') or {}).get('classes') or ():
        if cls.get('name') == 'Default':
            v = cls.get('clearance')
            return float(v) if isinstance(v, (int, float)) and v > 0 else None
    return None


def graded_default_clearance(min_clearance, default_class_clearance):
    """The clearance KiCad grades a Default-class net at: the Default class
    floored at ``rules.min_clearance`` (design_rules.resolve step 4). Either
    may be unset (None / 0); None when both are."""
    vals = [float(v) for v in (min_clearance, default_class_clearance)
            if isinstance(v, (int, float)) and v > 0]
    return max(vals) if vals else None


def floor_board_clearance_at_class(floors, graded_clearance,
                                   requested_clearance=None, announce=True):
    """``--escalation board`` when the project writeback will NOT run (#1262).

    With the writeback, a clearance descent below the Default class is
    recorded and the output project's Default class is lowered to it
    (#1160's ratchet), so the board grades clean against the project it ships
    with. Without it (``--no-fix-drc-settings``, the GUI's "Fix DRC settings
    after routing" unticked) the input project ships unchanged and KiCad
    grades every Default net at ``graded_clearance``
    (:func:`graded_default_clearance`), so a descent below it is a KiCad
    violation the run's own ledger called legal. Measured on a SOT-23-6 strip
    board (min_clearance 0.2, Default class 0.25): the rescue laid FB at its
    0.2125 rung, check_drc at the input class found 42 violations and
    kicad-cli flagged the same copper at 0.215-0.240 mm.

    Raises the clearance floor in ``floors`` to ``graded_clearance``. A run
    whose own clearance ``requested_clearance`` is already below it (an
    explicit ``--clearance`` or ``--clearance-ceiling``) keeps the REQUEST as
    its floor -- no automatic clearance descent at all, since every step below
    the request would be graded a violation too. Callers handle the clearance
    key here INSTEAD of in :func:`drop_stale_board_floors`, which would drop
    it and send descents to the fab floor.

    Only the clearance BASE moves: non-Default nets are floored at their own
    class under every policy (``GridRouteConfig.rule_floors``), and a
    ``.kicad_dru`` layer rule or a pad clearance override replaces the base
    where it applies, at its usual precedence. Returns the new floors dict."""
    out = dict(floors or {})
    if not isinstance(graded_clearance, (int, float)) or graded_clearance <= 0:
        return out
    target = float(graded_clearance)
    below = False
    try:
        if requested_clearance is not None and \
                float(requested_clearance) < target - 1e-9:
            target, below = float(requested_clearance), True
    except (TypeError, ValueError):
        pass
    prev = out.get('clearance')
    if prev is not None and abs(prev - target) <= 1e-9:
        return out
    out['clearance'] = target
    if announce:
        if below:
            print(f"  --escalation board, project writeback off: the run's "
                  f"clearance {target:g} mm is already below the "
                  f"{float(graded_clearance):g} mm KiCad grades the unchanged "
                  f"project's Default nets at; no clearance descent this run "
                  f"(#1262)")
        else:
            print(f"  --escalation board, project writeback off: clearance "
                  f"descents stop at {target:g} mm, the clearance KiCad grades "
                  f"the unchanged project's Default nets at (was "
                  f"{'unset' if prev is None else f'{prev:g} mm'}; #1262)")
    return out


def floor_board_rules_at_graded(floors, requested, announce=True):
    """``--escalation board`` when the project writeback will NOT run, for
    every floor except clearance (#1287; :func:`floor_board_clearance_at_class`
    does clearance, #1262).

    ``floors`` are what KiCad grades the unchanged project at: on the CLI
    :func:`graded_board_floors`, in the GUI the live board's design settings,
    which hold KiCad's factory value for a rule the project leaves out. A run
    whose own request is already below one of them keeps the REQUEST as that
    key's floor. That means no automatic descent for the key, because every
    step below the request is graded a violation too.
    :func:`drop_stale_board_floors`, which the writeback-on path uses, drops
    such a key instead and sends its descents to the fab floor. A request
    of 0 or less is not a size and changes nothing (the edge clearance's 0 means
    "no edge rule, use the copper clearance"). Returns the new floors dict."""
    out = dict(floors or {})
    for name, val in (requested or {}).items():
        key = _REQUEST_FLOOR_KEY.get(name)
        if key is None or key == 'clearance' or key not in out:
            continue
        try:
            val = float(val)
        except (TypeError, ValueError):
            continue
        if val <= 0 or val >= out[key] - 1e-9:
            continue
        if announce:
            print(f"  --escalation board, project writeback off: the run's "
                  f"{name.replace('_', ' ')} {val:g} mm is already below the "
                  f"{out[key]:g} mm KiCad grades the unchanged project's "
                  f"{key} at; no {key} descent this run (#1287)")
        out[key] = val
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
    genuinely does not move" from "this table cannot see the difference"; one
    such consumer printed the first while meaning the second on every board
    above 2 copper layers. On a 6-layer board 121 escape lanes short it said
    "more layers buy NO extra lanes on a face" (issue #700).

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


def _apply_board_floors(rungs, floors=None):
    """'board' policy: no rung may sit below a floor the board declares
    (``floors`` defaults to the process-wide board floors; a caller may pass
    the per-net .kicad_dru floors on top). Raising a rung can make two rungs
    identical; collapse those."""
    floors = _BOARD_FLOORS if floors is None else floors
    if not floors:
        return rungs
    out = []
    for r in rungs:
        rr = dict(r)
        for k, v in floors.items():
            if k in rr and v is not None and rr[k] < v:
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


def escalation_rungs(copper_layer_count, tier=None, overrides=None,
                     extra_floors=None):
    """The rungs a DESCENT site may step down through: ``fab_floor_ladder``
    under ``board`` / ``fab``, and NOTHING under ``off`` (an empty list --
    every site that walks it then keeps the requested geometry and reports).
    Every site that shrinks a via or narrows a track walks this, never the
    ladder directly, so the policy is one decision in one place.

    ``extra_floors`` (a FLOOR_KEYS dict, e.g. ``GridRouteConfig.rule_floors``
    for the net being descended) raises the rungs further under ``board``:
    the .kicad_dru minimums that bind THIS net, which the process-wide board
    floors cannot carry because they are per net and per layer."""
    if not may_narrow():
        return []
    rungs = fab_floor_ladder(copper_layer_count, tier, overrides)
    # Per-net floors apply under EVERY policy; GridRouteConfig.rule_floors is
    # the policy-aware side (it omits the .kicad_dru minimums under ``fab``,
    # which may go below them, but always carries a non-Default net's own
    # class clearance -- a grading floor the writeback never lowers, #530).
    if extra_floors:
        rungs = _apply_board_floors(rungs, extra_floors)
    return rungs


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


def physical_fab_floor(copper_layer_count, overrides=None):
    """The smallest geometry the fab can PHYSICALLY make, whatever tier is
    selected: the override file when one is given (it states the user's exact
    limits), else the advanced rung. The TIER bounds what the router may
    descend to on its own (escalation_rungs); an EXPLICIT request is checked
    against this physical floor, so `--via-size 0.3` under the hard standard
    tier is accepted as asked (and announced as below the tier), not pinned up
    to 0.45 -- the request is the operator declaring their fab can do it, the
    same reading the stale-board-minimum rule gives a request below a stock
    Board Setup minimum. Grading (check_drc) uses the same floor."""
    if overrides is None:
        overrides = _DEFAULT_OVERRIDES
    if overrides:
        return _tier_rungs(copper_layer_count, 'standard', overrides)[0]
    return _tier_rungs(copper_layer_count, 'advanced', None)[0]


def fab_floor_for_param(param_name, copper_layer_count, tier=None, overrides=None):
    """The PHYSICAL fab floor for a routing parameter (physical_fab_floor), or
    None if the parameter has no fab floor. Deliberately neither the tier's
    nor the board-raised floor: a board's declared minimum bounds automatic
    DESCENTS (see set_policy_from_args) and the tier bounds them too; neither
    pins an explicit request up."""
    key = _PARAM_FLOOR_KEY.get(param_name)
    if key is None:
        return None
    if overrides is None and tier is None:
        overrides = _DEFAULT_OVERRIDES
    return physical_fab_floor(copper_layer_count, overrides).get(key)


def count_copper_layers_in_file(pcb_path):
    """Count copper layers in a .kicad_pcb (matches `(0 "F.Cu" signal)`-style layer
    defs, not pad layer lists). Returns 0 on any read error. Stdlib-only."""
    # `None` is the in-memory case, and it is the one this counter exists to
    # hand over to `count_copper_layers_in_data` -- a live GUI board has
    # `source_path is None`, not `''`. Without this it raised TypeError out of
    # `open`, which a caller reports as an internal error.
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
    is perfectly good, which is why ``fanout_clearance.resolve_drill_floors``
    says so in its own docstring.

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
    """CLI guard: pin any routing parameter that is below the PHYSICAL fab
    floor (``fab_floor_for_param`` -> ``physical_fab_floor``: the advanced rung,
    or the ``--fab-overrides`` file) UP to that floor, warn, and keep running
    (issue #237). The fab physically can't make sub-floor geometry, so rather
    than abort the run we clamp each out-of-range value to the floor and print
    a warning. Neither the selected tier nor a board minimum pins an explicit
    request: those bound automatic descents only.

    Each CLI passes the params it accepts (track_width, clearance, via_size,
    via_drill, hole_to_hole_clearance); None values skip. Returns a
    ``{param_name: pinned_floor}`` dict of the clamps applied (empty if all in
    range) so the caller can write the pinned values back onto its parsed args."""
    viols = check_param_floors(copper_layer_count, tier, overrides, **params)
    pinned = {}
    for name, val, floor in viols:
        print(f"  WARNING: --{name.replace('_', '-')} {val} is below the physical fab "
              f"floor {floor} (the advanced rung, or your --fab-overrides); "
              f"pinning up to {floor}. Pass --fab-overrides "
              f"to declare a smaller fab capability, or raise the value to silence this.")
        pinned[name] = floor
    # #1433: the size and drill floors hold one value at a time; the via must
    # also keep a copper ring around its hole.
    if 'via_size' in params and 'via_drill' in params:
        vs = pinned.get('via_size', params.get('via_size'))
        vd = pinned.get('via_drill', params.get('via_drill'))
        new_vs = via_size_for_annular_ring(vs, vd, copper_layer_count, tier,
                                           overrides)
        if new_vs is not None and vs is not None and new_vs > vs:
            ring = min_via_annular_ring(copper_layer_count, tier, overrides)
            print(f"  WARNING: --via-size {vs} with --via-drill {vd} leaves an "
                  f"annular ring of {(vs - vd) / 2.0:.4g} mm, below the "
                  f"{ring:.4g} mm the physical fab floor's via implies (a "
                  f"drill wider than its pad has no ring at all); pinning "
                  f"--via-size up to {new_vs}. Pass --fab-overrides to "
                  f"declare a smaller fab capability, or give a via that "
                  f"holds its drill to silence this.")
            pinned['via_size'] = new_vs
    return pinned


def min_via_annular_ring(copper_layer_count, tier=None, overrides=None):
    """The smallest via annular ring (mm) the PHYSICAL fab floor implies (#1433).

    The ring is (diameter - drill) / 2. The floor's smallest via -- the
    advanced rung's 0.25/0.15, or the --fab-overrides file's -- fixes it at
    0.05, which is also the smallest ring on any corpus board. Deliberately
    NOT the floor's `annular` key: that is the PTH-pad recommendation
    (0.15-0.25), which every via the router lays would fail. None when the
    floor declares no via.
    """
    dia = fab_floor_for_param('via_size', copper_layer_count, tier, overrides)
    drill = fab_floor_for_param('via_drill', copper_layer_count, tier, overrides)
    if dia is None or drill is None:
        return None
    return max(0.0, round((dia - drill) / 2.0, 6))


def via_size_for_annular_ring(via_size, via_drill, copper_layer_count,
                              tier=None, overrides=None):
    """`via_size`, pinned UP so the via keeps the floor's annular ring (#1433).

    The size/drill floors are checked one value at a time, so a via could pass
    both and have no copper ring at all: `--via-size 0.45 --via-drill 0.508`
    laid 138 vias whose drill is wider than their pad. The DRILL is the hole
    the caller asked for, so the pad grows to drill + 2 x ring. Returns the
    value unchanged when either is None or the ring already holds. Shared by
    the CLIs' enforce_fab_floors and the GUI's effective via size.
    """
    if via_size is None or via_drill is None:
        return via_size
    ring = min_via_annular_ring(copper_layer_count, tier, overrides)
    if ring is None or (via_size - via_drill) / 2.0 >= ring - 1e-9:
        return via_size
    # Rounded UP to the 0.1 um grid, never to nearest: an imperial drill
    # (0.35052 = 13.8 mil) plus 2 x 0.05 is 0.45052, which round() made
    # 0.4505 -- a ring of 0.04999, 10 nm under the floor the pin exists to
    # meet, and every pinned via graded short.
    import math
    return math.ceil((via_drill + 2.0 * ring) * 1e4 - 1e-6) / 1e4


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


def replace_power_track_rows(power_net_ids, shipped):
    """#1033: make the ledger describe the power-net copper that SHIPS.

    Width rows for power nets are recorded per routing ATTEMPT (terminal
    necks, rescue rungs, fine-pitch taps, oracle reconnects) -- retries and
    rescues included, so they count attempts, not copper. Drop every
    ``track_width`` row of a power net and add ONE
    row per power net that still ships copper under its requested width,
    from the same measurement as JSON_SUMMARY power_widths. ``shipped`` is
    [(net_id, net_name, requested, min_delivered, under_mm), ...]. Other
    kinds (vias, clearance) and other nets are untouched."""
    replace_track_rows(power_net_ids, shipped,
                       'power copper shipped under width')


def replace_track_rows(net_ids, shipped, site):
    """:func:`replace_power_track_rows` for any set of nets: drop every
    ``track_width`` row of ``net_ids`` (recorded per routing ATTEMPT) and add
    one row, labelled ``site``, per net in ``shipped`` that ships copper under
    its requested width. Found reviewing #1282: an impedance- or class-width
    net necks through the same ladder as a power net, and its rows were per
    attempt (a `plane tap` row for a tap that never shipped still failed
    --strict-sizes) or absent altogether (the neck-down ladder records none)."""
    ids = set(net_ids or ())
    _LEDGER['narrowed'] = [r for r in _LEDGER['narrowed']
                           if not (r.get('kind') == 'track_width'
                                   and r.get('net') in ids)]
    for nid, name, req, dlv, under in shipped or ():
        if dlv is None or under <= 0 or dlv >= req - 1e-9:
            continue
        _LEDGER['narrowed'].append({
            'net': int(nid), 'net_name': name, 'kind': 'track_width',
            'requested': round(float(req), 4),
            'delivered': round(float(dlv), 4),
            'site': site,
            'count': 1, 'length_mm': round(float(under), 2),
        })


def replace_via_rows(shipped, site):
    """#1371: make the ledger's via rows describe the vias that SHIP.

    ``via_diameter`` rows are recorded per placement ATTEMPT (the last-resort
    via-in-pad ladder, rescues, the plane-tap ladder, layer-swap pad vias), so
    they name vias that never shipped (a tap the weld then rejected) and count
    one placement several times, while the rung-1 via search ships small vias
    and records nothing. Drop every ``via_diameter`` row and add one row per
    (net, diameter) from the census of this run's shipped vias.
    ``shipped`` is [(net_id, net_name, requested, {diameter: count}), ...],
    from ``routing_common.new_via_shortfall``. Other kinds are untouched."""
    _LEDGER['narrowed'] = [r for r in _LEDGER['narrowed']
                           if r.get('kind') != 'via_diameter']
    for nid, name, req, sizes in shipped or ():
        for dia, n in sorted(sizes.items()):
            if dia >= req - 1e-9:
                continue
            _LEDGER['narrowed'].append({
                'net': int(nid), 'net_name': name, 'kind': 'via_diameter',
                'requested': round(float(req), 4),
                'delivered': round(float(dia), 4),
                'site': site, 'count': int(n),
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
        # WHERE those escalations landed: 'advanced', or 'board_floors' when
        # --escalation board bounded them at the board's own declaration. A
        # consumer counting fab_tier_escalations needs this to know whether the
        # run left the board's declared envelope -- see escalation_target.
        'fab_tier_target': escalation_target(),
    }


def escalation_report_line():
    """The ONE end-of-run line a reader can act on, or '' when nothing moved."""
    s = escalation_summary()
    if not s['count'] and not s['fab_tier_escalations']:
        return ''
    parts = []
    # A row with no net is run-wide (#1210: the run's clearance below the
    # board's minimum clearance), not a feature on a net.
    run_wide = [r for r in s['narrowed'] if r['net'] is None]
    per_net = [r for r in s['narrowed'] if r['net'] is not None]
    for r in run_wide:
        parts.append(f"{r['kind'].replace('_', ' ')} {r['delivered']:g} mm, below the "
                     f"{r['site']} {r['requested']:g} mm")
    if per_net:
        mins = {}
        for r in per_net:
            k = r['kind']
            mins[k] = r['delivered'] if k not in mins else min(mins[k], r['delivered'])
        kinds = ', '.join(f"smallest {k.replace('_', ' ')} {v:g} mm"
                          for k, v in sorted(mins.items()))
        parts.append(f"{sum(r['count'] for r in per_net)} feature(s) on "
                     f"{len(s['nets'])} net(s) delivered below "
                     f"the requested size ({kinds})")
    if s['fab_tier_escalations']:
        _where = ("the board's own declared floors"
                  if s['fab_tier_target'] == 'board_floors' else 'advanced')
        parts.append(f"{s['fab_tier_escalations']} fab-tier escalation(s) to {_where}")
    return (f"Design rules [--escalation {s['escalation_policy']}, --fab-tier "
            f"{s['fab_tier']}]: " + '; '.join(parts) +
            ". Details: JSON_SUMMARY design_rules.")


def narrowed_summary():
    """:func:`escalation_summary` when this run delivered anything below a
    requested size or escalated a fab tier, else None -- the JSON_SUMMARY
    ``design_rules`` block the non-route CLIs add only when there is something
    in it, so a run that narrowed nothing prints what it always printed."""
    s = escalation_summary()
    return s if (s['count'] or s['fab_tier_escalations']) else None


def finish_design_rules(args, rc=0, *, print_line=True):
    """The end-of-run disclosure every routing CLI that registers
    ``--strict-sizes`` makes: the ONE ``Design rules [...]`` line when this
    run delivered anything below a requested size or escalated a fab tier,
    and, under ``--strict-sizes``, exit code 3 for it.

    Returns the exit code the main should finish with. ``rc`` is the code it
    would finish with otherwise; a non-zero ``rc`` wins over 3, because a run
    that already failed for its own reason says more than "narrowed". route.py
    prints the line from inside batch_route (both fronts see it there), so its
    main passes ``print_line=False``.

    Found reviewing #1282: route_planes, repair_planes, route_diff and the two
    fanouts accepted ``--strict-sizes`` and never acted on it, nor printed the
    line, so a harness relying on exit 3 read a narrowed run as exact."""
    line = escalation_report_line()
    if line and print_line:
        print(f"  {line}")
    if line and not rc and getattr(args, 'strict_sizes', False):
        print(f"  --strict-sizes: {line}")
        return 3
    return rc


_escalation_lever_said = []


def escalation_target():
    """Where a standard->advanced rung change actually LANDS under the active
    policy: ``'advanced'``, or ``'board_floors'`` under ``--escalation board``.

    Under ``board`` every rung is raised to the board's own declared minimums
    (:func:`_apply_board_floors`), so a descent off rung 0 stops at what the
    BOARD declared and never reaches the advanced floor for any key the board
    declares. Reporting it as "escalated to advanced (0.25/0.15 via)" is then
    simply false: measured on a 6-layer board declaring ``min_via_diameter``
    0.4, a ``--escalation board`` run reported "645 fab-tier escalation(s) to
    advanced" while the smallest via it delivered was 0.4 -- the board's own
    number. The descent is real and still worth counting (0.4 is below the
    standard tier's 0.45), but it is bounded by the declaration, which is the
    whole point of the policy.
    """
    return 'board_floors' if _ESCALATION == 'board' else 'advanced'


def warn_fab_escalation(context):
    """Emit a one-line warning (deduped per run, per context) that a routing step
    dropped below the standard floor -- to the more-costly advanced floor, or,
    under ``--escalation board``, to the board's own declared minimums (see
    :func:`escalation_target`) -- and count it in the ledger (every call counts,
    deduped or not)."""
    if not context:
        return
    _LEDGER['fab_tier'].append(context)
    if context in _escalation_warned:
        return
    _escalation_warned.add(context)
    if escalation_target() == 'board_floors':
        # Bounded by the declaration: do NOT claim the advanced floor, and do
        # not offer --fab-overrides as the lever -- the board already states
        # the limit and the policy is already honouring it.
        print(f"  WARNING: {context}: dropped below the standard fab floor to "
              f"the BOARD's own declared minimums; --escalation board bounds it "
              f"there (it never reaches the advanced 0.25/0.15 via). Pass "
              f"--fab-tier standard for a hard floor, or --escalation off to "
              f"forbid the descent entirely")
        return
    print(f"  WARNING: {context}: escalated standard->advanced fab floor "
          f"(0.25/0.15 via etc., more costly to fab); --fab-tier auto permitted "
          f"it. Pass --fab-tier standard for a hard floor, or "
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
    # Replay knobs (#530 corpus A/B): KICAD_FAB_TIER_DEFAULT / KICAD_ESCALATION_DEFAULT
    # set the DEFAULT of the two flags when a command omits them, so a
    # cloud_replay_sets --env arm can replay pre-#857 manifests under the old
    # ladder ('auto' + 'fab') like-for-like. An explicit flag still wins; an
    # unknown value is ignored. Disclosed by the ENV KNOBS line like every
    # KICAD_* variable. Never for a real run -- pass the flags.
    _tier_dflt = os.environ.get('KICAD_FAB_TIER_DEFAULT', DEFAULT_TIER)
    if _tier_dflt not in TIERS:
        _tier_dflt = DEFAULT_TIER
    _esc_dflt = os.environ.get('KICAD_ESCALATION_DEFAULT', DEFAULT_ESCALATION)
    if _esc_dflt not in ESCALATION_POLICIES:
        _esc_dflt = DEFAULT_ESCALATION
    parser.add_argument(
        '--fab-tier', choices=list(TIERS), default=_tier_dflt,
        help=f"JLC fab capability floor (default {DEFAULT_TIER}). 'auto' = the cheap standard "
             "floor, escalating to advanced (0.25/0.15 via etc., more costly; warned "
             "and counted in the run summary) when a fine-pitch fan-out, plane tap or "
             "last-resort via cannot fit at the standard floor; 'standard' and "
             "'advanced' are HARD floors that never escalate.")
    parser.add_argument(
        '--fab-overrides', metavar='FILE', default=None,
        help="Path to a fab-floor override file (key=value lines, e.g. "
             "'via_drill = 0.15') overlaying the selected --fab-tier; only the "
             "listed values are replaced. Supplying it disables escalation (the "
             "floor becomes exactly the file + base tier). See the template "
             "fab_overrides.example.txt for the format and every key.")
    if include_escalation:
        parser.add_argument(
            '--escalation', choices=list(ESCALATION_POLICIES), default=_esc_dflt,
            help=f"How far below a REQUESTED size a failing net may be retried "
                 f"(default {DEFAULT_ESCALATION}). 'off': never -- sizes and clearances are exact, "
                 "a net that cannot complete at them fails and is reported (a wide "
                 "power/impedance net may still neck for at most "
                 f"{defaults.PAD_ENTRY_NECK_MM:g} mm of path at a pad entry). 'board': "
                 "down to the board's own declared floors (Board Setup rules.min_*; "
                 "an unset key falls back to the fab tier floor, an unset "
                 "min_clearance to the Default class). That is what KiCad's DRC "
                 "accepts once the output project's Default class is lowered to "
                 "the clearance used, which the DRC-settings writeback does; with "
                 "--no-fix-drc-settings the project is left as it was, so "
                 "descents stop at what KiCad grades it at: a rule it leaves "
                 "out at KiCad's factory value (#1287), clearance at the "
                 "Default class floored at min_clearance (#1262). 'fab': down to "
                 "the fab tier floor, below the board's own minimums. Every "
                 "descent is counted in JSON_SUMMARY design_rules.")
        parser.add_argument(
            '--clearance-ceiling', type=float, default=None, metavar='MM',
            help="Cap EVERY net class (Default included) at this copper clearance for "
                 "the run and clamp the output project's classes down to it -- the "
                 "'stock net classes are aspirational' workflow. This is what "
                 "--clearance used to do implicitly (#439); --clearance now sets only "
                 "the Default class for the run and honours the other classes, as "
                 "KiCad does.")
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
    override file (warns, falls back to the bare tier).

    'fix_drc_settings' False (the "Fix DRC settings after routing" box
    unticked) means no tab writes the live board's DRC settings, so under
    board the clearance floor is what KiCad grades the unchanged board's
    Default nets at -- 'default_class_clearance' floored at the board's
    min_clearance (#1262, :func:`board_policy_floors`, shared with the CLI).
    The other 'board_floors' are already what KiCad grades: the live design
    settings hold KiCad's factory value for a rule the project leaves out,
    which the CLI reads from :data:`KICAD_FACTORY_RULES` (#1287)."""
    tier = (config.get('fab_tier') or DEFAULT_TIER)
    path = (config.get('fab_overrides_path') or '').strip()
    overrides = {}
    if path:
        try:
            overrides = parse_fab_overrides(path)
        except OSError as exc:
            print(f"WARNING: could not read fab overrides {path}: {exc}")
    set_default_fab_tier(tier, overrides)
    policy = config.get('escalation') or DEFAULT_ESCALATION
    declared = config.get('board_floors') or {}
    requested = {k: config.get(k) for k in ('track_width', 'clearance', 'via_size',
                                            'via_drill', 'hole_to_hole_clearance',
                                            'board_edge_clearance')}
    if policy == 'board':
        floors = board_policy_floors(
            policy, declared, requested,
            # board_floors' clearance is min_clearance, else (unset) the
            # Default class already (swig_gui._board_floor_dict, #1160).
            graded_default_clearance(declared.get('clearance'),
                                     config.get('default_class_clearance')),
            writeback=bool(config.get('fix_drc_settings', True)))
    else:
        floors = drop_stale_board_floors(declared, requested)
    set_escalation_policy(policy, floors)


def fab_tier_from_args(args):
    """Resolve ``(tier, overrides_dict)`` from parsed args; load the override file
    once. The override file (if any) overlays whichever tier was selected."""
    tier = getattr(args, 'fab_tier', DEFAULT_TIER) or DEFAULT_TIER
    path = getattr(args, 'fab_overrides', None)
    if path and not os.path.isfile(path):
        raise SystemExit(f"error: --fab-overrides file not found: {path}")
    overrides = parse_fab_overrides(path) if path else {}
    return tier, overrides


def apply_clearance_ceiling(args, tool=''):
    """Cap this run's scalar ``--clearance`` at ``--clearance-ceiling``, and SAY SO.

    ``add_fab_tier_args`` registers ``--clearance-ceiling`` on every tool that
    takes ``--escalation``, which is the fanout fronts too -- but only the
    ROUTING mains ever read it, so ``bga_fanout`` and ``qfn_fanout`` ACCEPTED
    the flag and ignored it. Nothing warned, and the chain doctrine in
    CLAUDE.md says to pass the ceiling rather than a bare ``--clearance``, so
    the recommended spelling was the one that silently did nothing.

    MEASURED on butterstick (a 381-ball 0.8mm BGA): passing only
    ``--clearance-ceiling`` left the escape field at the tool's own generic
    default instead of the intended floor -- 3232 grazes, 2 dropped balls, 112
    failed plane drops and 3 of 85 GND balls pour-served, against 19 grazes, 0
    dropped and 85/85 with the same number spelled ``--clearance``.

    These fronts route to ONE scalar, so the ceiling's "cap every net class"
    reading collapses to ``min(requested, ceiling)`` -- tighten-only, exactly
    as it is for the Default class on the routing mains. The WRITEBACK half
    already matched: both fronts pass ``clamp_nondefault_netclasses=True``
    unconditionally, so only the run clearance was missing.

    Call it AFTER ``parse_args`` and BEFORE anything reads ``args.clearance``
    (on bga_fanout that means before ``enforce_fab_floors``, which would
    otherwise pin the UNCAPPED value up to a tier floor). Returns the ceiling,
    or None when the flag was omitted.
    """
    ceiling = getattr(args, 'clearance_ceiling', None)
    if ceiling is None:
        return None
    label = f"{tool}: " if tool else ""
    cur = getattr(args, 'clearance', None)
    if cur is None or ceiling < cur:
        args.clearance = ceiling
        print(f"  {label}--clearance-ceiling {ceiling}: clearance capped at it "
              f"for this run (was {cur}); the output project's classes are "
              f"clamped down to it (#439).")
    else:
        print(f"  {label}--clearance-ceiling {ceiling}: no cap applied -- "
              f"--clearance {cur} is already at or below it; the output "
              f"project's classes are still clamped to the routed floor.")
    return ceiling


def set_policy_from_args(args, pcb_path=None):
    """CLI helper: ``set_escalation_policy`` from ``--escalation`` plus the
    board's own ``rules.min_*`` read from the sibling .kicad_pro of
    ``pcb_path`` (stdlib JSON read; no parser). ``--no-fix-drc-settings``
    (``args.no_fix_drc_settings``) means the project writeback will not run:
    see :func:`board_policy_floors`. Call it after ``args.clearance`` is
    resolved (the run's clearance is the request the floor is capped at)."""
    import json
    floors = {}
    graded = None
    rules = {}
    if pcb_path:
        pro = os.path.splitext(pcb_path)[0] + '.kicad_pro'
        try:
            with open(pro, encoding='utf-8') as f:
                proj = json.load(f)
            rules = ((proj.get('board') or {}).get('design_settings') or {}).get('rules') or {}
            dflt = project_default_class_clearance(proj)
            # #1369: a min_clearance the #530 cap lowered to a pad override
            # is not the board's minimum; the records say what is.
            _declared = dict(rules)
            try:
                import tool_records
                _mc = board_declared_min_clearance(
                    rules.get('min_clearance'),
                    tool_records.effective_namespace(pcb_path))
            except Exception:                                   # noqa: BLE001
                _mc = rules.get('min_clearance')
            if _mc != rules.get('min_clearance'):
                _declared['min_clearance'] = _mc
                if getattr(args, 'escalation', None) == 'board':
                    print(f"  --escalation board: rules.min_clearance "
                          f"{rules.get('min_clearance'):g} mm is a pad clearance "
                          f"override the writeback capped the rule at, not a "
                          f"board minimum; the board's own is {_mc:g} mm (#1369)")
            floors = board_floors_from_rules(_declared, dflt)
            graded = graded_default_clearance(rules.get('min_clearance'), dflt)
        except (OSError, ValueError, AttributeError):
            # No readable project: KiCad grades such a board at its factory
            # values, which is what graded_board_floors({}) answers.
            floors = {}
            graded = None
            rules = {}
    policy = getattr(args, 'escalation', None) or DEFAULT_ESCALATION
    writeback = not getattr(args, 'no_fix_drc_settings', False)
    floors = board_policy_floors(
        policy, floors, {k: getattr(args, k, None) for k in _REQUEST_FLOOR_KEY},
        graded, writeback=writeback,
        graded_floors=(graded_board_floors(rules, announce=True)
                       if pcb_path and policy == 'board' and not writeback
                       else None))
    set_escalation_policy(policy, floors)
    return floors


def board_policy_floors(policy, floors, requested, graded_clearance,
                        writeback=True, graded_floors=None):
    """The floors ``--escalation board`` bounds descents at, for ONE run --
    shared by the CLI (:func:`set_policy_from_args`) and the GUI
    (:func:`set_fab_tier_from_config`) so the two fronts cannot disagree.

    Under any other policy the floors pass through. Under ``board`` with the
    project ``writeback``, a declared minimum the run's own request is
    already below is stale (:func:`drop_stale_board_floors`). When the
    writeback will not run, every floor is what KiCad grades the unchanged
    project at, ``graded_floors`` (:func:`graded_board_floors`, which floors a
    rule the project leaves out at KiCad's factory value, #1287), or
    ``floors`` when the caller's floors already are that (the GUI's live
    design settings). A request below one of them is that key's floor
    (:func:`floor_board_rules_at_graded`). The clearance floor is the
    clearance KiCad grades the project's Default nets at,
    ``graded_clearance`` (:func:`floor_board_clearance_at_class`, #1262)."""
    if policy != 'board':
        return dict(floors or {})
    if writeback:
        return drop_stale_board_floors(floors, requested)
    out = floor_board_rules_at_graded(
        floors if graded_floors is None else graded_floors, requested)
    return floor_board_clearance_at_class(
        out, graded_clearance, (requested or {}).get('clearance'))
