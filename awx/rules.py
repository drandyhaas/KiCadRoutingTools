#!/usr/bin/env python3
"""ONE resolver for every design-rule number the #622 chain uses.

Until this module the chain's geometry was a set of LITERALS for a 0.1 mm
process, written out five times: ``topo_strings.TRACK = 0.127``,
``braid.CLEAR = 0.105`` / ``SPEC_CLEARANCE = 0.1`` / ``VIA_SIZE = 0.25`` /
``VIA_DRILL = 0.15``, ``select_moves.BAND_TIP = 0.9`` / ``BAND_LPITCH = 0.35``,
``source_realize.FAN_TRACK`` / ``FAN_CLEAR = 0.1``, and ``track_width=0.1,
clearance=0.1`` hardcoded at six production-engine call sites.  A board whose
Default net class asks for 0.15 mm would have been planned, fanned out and
braided at 0.105 and shipped with DRC errors -- the router does not read what
the board asks for.  `rules_of(board)` reads it, once per stage-process, and
`install()` puts the answer where the modules already look.

WHAT IS AND IS NOT A RULE HERE
------------------------------
Two different kinds of number live in this module and they resolve
differently, which is the whole design:

  * a CONSTRAINT -- ``min_clearance``, a net class's ``clearance``, a
    ``.kicad_dru`` layer rule, the fab floor.  KiCad (or physics) grades
    copper against it, so the chain must never route BELOW it.  These raise
    the chain's number.
  * a PREFERENCE -- the braid's 0.127 lane track, its 0.35 lane pitch.  The
    chain picked these; no grader enforces them.  They are the starting
    value, and a constraint may raise them.

So every quantity resolves as ``max(the chain's preference, every constraint
that binds it)``.  Nothing here ever LOWERS the chain's geometry to meet a
board that asks for less: a board declaring a 0.05 mm process does not oblige
this router to use it.

PRECEDENCE, per quantity (highest-binding last; all are max()es)
----------------------------------------------------------------
clearance (the SPEC clearance: ``topo_strings.SPEC_CLEAR``,
           ``braid.SPEC_CLEARANCE``, what the fanout lays at and what
           `chain_k.sh` grades at)
    1. ``SPEC_CLEARANCE`` below -- the chain's preference, 0.1
    2. the sibling ``.kicad_pro`` **Default** net class ``clearance``.
       A declared 0.0 is UNSET, not a floor of zero (#966,
       `fix_kicad_drc_settings`: "None if unset or 0") -- the bench's own
       project declares exactly that, so on the bench this term is absent.
       Why the Default class and not the net's own: KiCad grades a pair at
       ``max(class A, class B)`` and our lanes are graded against whatever
       foreign copper they pass, most of which is Default.  Taking the
       Default class is the conservative reading.  (The braid prices ONE
       scalar for the whole bus, so one class is all it can consume; a
       per-net-class map is the next step and is recorded as debt below.)
    3. ``board.design_settings.rules.min_clearance`` -- KiCad's absolute
       board minimum, which outranks classes.
    4. the largest ``.kicad_dru`` layer-clearance rule on the two outer
       layers (``escape_moves.LAYERS``).  TIGHTEN-ONLY, composed exactly as
       the BGA fanout composes it (`bga_fanout/__init__.py`, #498): the
       braid is scalar-clearance throughout, so taking a RELAXING rule from
       one layer would under-space the other.  This is deliberately more
       conservative than KiCad's replacement semantics.
    5. ``physical_fab_floor(copper layers)['clearance']`` -- the smallest
       geometry the fab can make.  The PHYSICAL floor, not the tier's
       nominal floor, because the chain's geometry is an EXPLICIT request
       and CLAUDE.md's rule for those is "drawn as asked, floored only at
       the PHYSICAL fab floor" (the nominal 4-layer via floor is 0.45; it
       would silently triple the chain's barrels).

track (the braid's lane track: ``topo_strings.TRACK``)
    1. ``TRACK`` below -- the chain's preference, 0.127
    2. ``rules.min_track_width`` (the only thing KiCad grades a width
       against)
    3. ``physical_fab_floor(...)['track_width']``
    The Default class's ``track_width`` is deliberately NOT read here.  It
    is the designer's preferred width for their own nets, not a constraint,
    and the board carries TWO track widths ON PURPOSE (README, "One source
    for every routing number"): the fanout's stubs at ``fan_track`` and the
    braid's lanes wider.  Letting a class width drive the lane track would
    collapse that distinction -- and a 2-layer board whose Default class
    says 0.5 mm would make the bus unroutable.

fan_track (what the production engine lays the fanout stubs at)
    same ladder as ``track`` with the chain's preference ``FAN_TRACK`` 0.1.
fan_clear
    = ``clearance``.  The fanout lays at the spec; there is no second
    clearance.

via_size / via_drill
    1. ``VIA_SIZE`` 0.25 / ``VIA_DRILL`` 0.15 -- the chain's preference
    2. ``rules.min_via_diameter`` / ``max(min_via_drill,
       min_through_hole_diameter)``
    3. ``physical_fab_floor(...)['via_diameter' | 'via_drill']``
    4. and then the ANNULAR RING closes over the pair:
       ``via_size >= via_drill + 2 * rules.min_via_annular_width``.
       (The bench declares 0.05, and 0.15 + 0.10 = 0.25 exactly -- its via
       is annulus-tight, which is why nobody noticed the rule was missing.)

hole_to_hole / edge_clearance
    raw reads of ``rules.min_hole_to_hole`` / ``min_copper_edge_clearance``
    (`list_nets.board_constraint`), or None.  The braid already read these
    two off the board and applies them tighten-only against the router
    config's own defaults; that comparison is left exactly where it was, so
    this module only moves the READ into one place.

DERIVED QUANTITIES -- the formula that produced today's literal
---------------------------------------------------------------
    hug        = clearance + 5 um        -> 0.105   ``braid.CLEAR``
                 "the spec plus 5 um so a hug does not sit exactly on it"
    lane_slice = track + hug             -> 0.232   ``select_moves.NEST_IN``
                 one lane's centre-to-centre slice: two parallel tracks of
                 width w at clearance c sit at pitch w + c.
    lane_pitch = max(0.35, lane_slice)   -> 0.35    ``braid.LPITCH``,
                 ``select_moves.BAND_LPITCH``.  0.35 is a preference; the
                 RULE is that a comb can never pack tighter than one lane's
                 slice.  It binds at clearance >= 0.218.
    exit_pitch = max(0.38, lane_slice)   -> 0.38    ``braid.MINP``, same
                 reading at the exits.
    band_tip   = pitch/2 + exit_margin   -> 0.9     ``select_moves.BAND_TIP``
                 Half the destination array's ball pitch plus the fanout
                 engine's exit margin, which is select_moves' own comment
                 for the number, and it reproduces 0.9 exactly on the bench
                 (DU1's pitch is 0.8, the exit margin 0.5).  This is ARRAY
                 geometry, not a design rule, so `rules_of` resolves it only
                 when handed a destination ref.
                 AND IT IS A DEAD DEFAULT, which is worth saying plainly
                 rather than quietly resolving: the only code that reads
                 BAND_TIP (``select_moves.band_leg`` / ``band_capacity``) is
                 on the ``SPLIT_BLOCKS=1`` path, and that path's caller
                 (``fanout_from_plan.plan_state``) already overwrites it with
                 a DIFFERENT formula -- ``max(pitch_x, pitch_y) / 2 + 0.05``,
                 half a pitch plus one occupancy cell, because the under-pad
                 engine ends its stubs at the boundary cell and not at
                 exit_margin (measured 0.425 at 0.8 mm pitch).  That override
                 is the live number and is deliberately left alone: 0.05 is an
                 engine behaviour, not a design rule.  So no chain stage
                 passes ``dest_ref`` today; the field and its formula exist so
                 the literal has a written source.
    band_gap, half_sep, via_need, end_keep, margin_out
                 stay as the expressions braid/topo_strings already spell;
                 they are re-evaluated from the resolved base by `install`.

FLOAT BITS ARE PART OF THE CONTRACT
-----------------------------------
``0.1 + 0.005`` is NOT the double ``0.105`` -- it is one ULP above it -- and
``0.127 + 0.105`` is one ULP BELOW ``0.232``.  A 1-ULP difference in a
clearance is a different board: it moves a grid cell, which moves a lane,
which changes the via count.  So every quantity this module RESOLVES is
rounded to 6 decimals (the same normalization `fix_kicad_drc_settings` uses
on the writeback), which lands exactly on the literal it replaces.  The
expressions that were already written as arithmetic over the constants
(``BAND_GAP``, ``HALF_SEP``, ``VIA_NEED``, ``END_KEEP``, ``MARGIN_OUT``) are
re-evaluated in their ORIGINAL order and are NOT rounded, so they too come
out bit-identical.  `tests/test_622_rules_of.py` asserts every one of these
against the recorded literal with ``==`` on the float, not ``approx``.

USING IT
--------
Each chain stage is its own process on a board file, so each stage's entry
point does, once, near the top of ``main()``::

    import rules as _rules
    _rules.install(_rules.rules_of(board_path))

`install` writes the resolved numbers into the module-level constants the
chain already reads (``topo_strings.TRACK``, ``braid.CLEAR``, ...) and
re-evaluates the constants derived from them.  The literals stay as each
module's DEFAULT: a module imported without an install behaves exactly as it
did before this file existed, which is what makes the flag-off bench
byte-identical BY CONSTRUCTION rather than by measurement.  (Measured too --
see the README section.)

Why install-into-constants rather than making each constant a function: the
chain's consumers read these through module ATTRIBUTES at call time
(``te.VIA_SIZE``, ``br.TRACK``, ``br.CLEAR``) in ~30 places, so one install
reaches all of them, and no hot loop grows a function call.  The two modules
that bind them into locals at import time (`cut_ledger`, `collapse_dives`)
re-read after installing.

DEBT THIS FILE DOES NOT PAY (recorded, not fixed)
-------------------------------------------------
  * per-NET-CLASS clearance.  The braid prices one scalar for the whole bus.
    The bench's own DDR3_Signal class is 0.0889/0.1143 -- TIGHTER than the
    0.1/0.127 we use -- so we are conservative there, but a board whose bus
    class is WIDER than its Default class would be under-spaced.
  * ``select_moves.BAND_BLOCK_GAP = 0.30`` calls itself "the braid's
    BAND_GAP", and ``braid.BAND_GAP`` is ``TRACK + CLEAR + 0.07`` = 0.302.
    It is a stale hand-copy.  Making it follow the formula changes the
    flag-off output, so it is left literal and named here instead.
  * ``braid.BLOCK_GAP`` 0.45, ``LEG_W``, ``LEG_REQ``, ``LEG_O``,
    ``CROSS_TUBE``, ``HEAD_RUN``, ``MINP``'s 0.38 and ``LPITCH``'s 0.35 are
    preferences with no rule source; only their lane-slice floors are
    modelled.
  * the 0.025 routing grid in ``braid.setup`` is sized against the fanout's
    0.25 stub packing ("the legal minimum, track + clearance = 0.227, plus
    23 um").  That sentence is a rule computation and the grid is still a
    literal.
"""
import os
import sys
from dataclasses import dataclass, field, replace

HERE = os.path.dirname(os.path.abspath(__file__))
if os.path.join(HERE, '..', 'py_router') not in sys.path:
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

# ---------------------------------------------------------------- the chain's
# PREFERENCES.  These are the literals this module replaces, and they keep
# living here so there is exactly one home for each.  Every one is a starting
# value a board constraint may raise -- see the precedence above.
SPEC_CLEARANCE = 0.1      # the spec clearance the chain plans and grades at
TRACK = 0.127             # the braid's lane track (5 mil)
HUG_OVER = 0.005          # ...and the hug's 5 um over the spec
VIA_SIZE = 0.25
VIA_DRILL = 0.15
FAN_TRACK = 0.1           # the production engine's fanout stub width
LANE_PITCH = 0.35         # braid.LPITCH
EXIT_PITCH = 0.38         # braid.MINP
BAND_TIP = 0.9            # select_moves.BAND_TIP (SPLIT_BLOCKS path only)
EXIT_MARGIN = 0.5         # routing_defaults.BGA_EXIT_MARGIN, the fanout's
                          # exit margin -- the second half of band_tip

_ROUND = 6                # decimals every resolved quantity lands on


def _r(x):
    """The 6-decimal normalization.  See "FLOAT BITS" above -- without it
    ``clearance + HUG_OVER`` is one ULP off the 0.105 it replaces."""
    return round(float(x), _ROUND)


@dataclass(frozen=True)
class Rules:
    """The resolved design rules for one board, for one chain stage.

    Every field is millimetres.  ``hole_to_hole`` and ``edge_clearance`` are
    the board's raw declarations and may be None (the board declares none);
    every other field is always a number, because the chain's own preference
    is the floor of each.
    """
    clearance: float = SPEC_CLEARANCE
    track: float = TRACK
    via_size: float = VIA_SIZE
    via_drill: float = VIA_DRILL
    hole_to_hole: float = None
    edge_clearance: float = None
    fan_track: float = FAN_TRACK
    lane_pitch: float = LANE_PITCH
    exit_pitch: float = EXIT_PITCH
    band_tip: float = BAND_TIP
    board: str = ''
    notes: tuple = ()
    sources: dict = field(default_factory=dict)

    # -- derived -----------------------------------------------------------
    @property
    def hug(self):
        """``braid.CLEAR``: the spec plus 5 um so a hug does not sit exactly
        on it.  0.1 -> 0.105."""
        return _r(self.clearance + HUG_OVER)

    @property
    def fan_clear(self):
        """``source_realize.FAN_CLEAR``: the fanout lays at the spec."""
        return self.clearance

    @property
    def lane_slice(self):
        """``select_moves.NEST_IN``: one lane's slice, track + hug.  0.232."""
        return _r(self.track + self.hug)

    # The four expressions below are braid's/topo_strings' OWN, in their own
    # order, unrounded -- see "FLOAT BITS".  install() writes these values
    # back over the module constants.
    @property
    def band_gap(self):
        return self.track + self.hug + 0.07          # braid.BAND_GAP

    @property
    def half_sep(self):
        return (self.track + self.clearance) / 2     # braid.HALF_SEP

    @property
    def via_need(self):
        return self.via_size / 2 + self.hug + self.track / 2 + 0.03  # braid.VIA_NEED

    @property
    def end_keep(self):
        return self.track + self.hug + 0.05          # braid.END_KEEP

    @property
    def margin_out(self):
        return self.clearance + self.track / 2       # topo_strings.MARGIN_OUT

    def describe(self):
        """One line per quantity, with where the number came from -- what a
        stage prints so a reader can tell a resolved rule from a default."""
        out = [f'rules for {self.board or "(no board)"}:']
        for k in ('clearance', 'track', 'via_size', 'via_drill',
                  'hole_to_hole', 'edge_clearance', 'fan_track',
                  'lane_pitch', 'exit_pitch', 'band_tip'):
            v = getattr(self, k)
            out.append(f'  {k:16s} {"-" if v is None else v:<8} '
                       f'{self.sources.get(k, "")}')
        out.append(f'  {"hug (derived)":16s} {self.hug:<8} clearance + {HUG_OVER}')
        out.append(f'  {"lane_slice":16s} {self.lane_slice:<8} track + hug')
        for n in self.notes:
            out.append(f'  note: {n}')
        return '\n'.join(out)


DEFAULT = Rules()
"""The chain's preferences with no board consulted.

This is what every module's constant is initialized from, so a module used
without an install behaves exactly as it did when the numbers were literals.
"""


# --------------------------------------------------------------- resolution

def _pro_path(board):
    return os.path.splitext(board)[0] + '.kicad_pro'


def _copper_layer_count(board):
    """Copper layers, for the fab floor.  Cheap: counts the (layers) table
    without a full parse when it can, falls back to the parser."""
    try:
        from fab_tiers import count_copper_layers_in_file
        n = count_copper_layers_in_file(board)
        if n:
            return n
    except Exception:                                        # noqa: BLE001
        pass
    return 2


def _dru_outer_max(board, layers):
    """The largest honored ``.kicad_dru`` layer-clearance rule on ``layers``.

    Composed exactly as the BGA fanout composes it (#498): the largest rule
    on the layers this chain may use, tighten-only.  None when the board has
    no dru file or no layer rules on those layers."""
    try:
        from kicad_dru import read_board_layer_clearances
        lmap, notes = read_board_layer_clearances(board, list(layers))
    except Exception:                                        # noqa: BLE001
        return None, []
    vals = [v for l, v in (lmap or {}).items() if l in layers]
    return (max(vals) if vals else None), list(notes or [])


def _array_pitch(board, ref):
    """The smallest centre-to-centre step of ``ref``'s pad grid, or None.

    The first half of band_tip's formula.  Read off the footprint's own
    local pad coordinates, so it is pose-independent."""
    try:
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(board)
        fp = pcb.footprints.get(ref)
        if fp is None or len(fp.pads) < 2:
            return None
        best = None
        for axis in ('local_x', 'local_y'):
            vals = sorted({round(getattr(p, axis), 4) for p in fp.pads})
            for a, b in zip(vals, vals[1:]):
                d = round(b - a, 4)
                if d > 1e-6 and (best is None or d < best):
                    best = d
        return best
    except Exception:                                        # noqa: BLE001
        return None


def rules_of(board, dest_ref=None, layers=('F.Cu', 'B.Cu')):
    """Resolve this board's design rules.  See the module docstring for the
    precedence; every quantity is ``max(the chain's preference, the
    constraints that bind it)``, rounded to 6 decimals.

    ``board`` is a path to a ``.kicad_pcb`` (its siblings ``.kicad_pro`` and
    ``.kicad_dru`` are what carry the rules).  ``dest_ref``, when given,
    resolves ``band_tip`` from that footprint's ball pitch.  ``layers`` are
    the copper layers the chain routes on -- the layers a ``.kicad_dru``
    rule is taken from.

    A board that declares nothing (no project, no rules, no dru) resolves to
    `DEFAULT` plus the fab floor for its layer count, which is what the
    chain did before this module existed.
    """
    board = str(board or '')
    notes, src = [], {}

    def pick(name, pref, cands):
        """max() over (preference, constraints), recording provenance."""
        best, who = _r(pref), 'chain preference'
        for label, v in cands:
            if v is None:
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if v <= 0:            # 0 = UNSET everywhere in KiCad (#966)
                continue
            if _r(v) > best:
                best, who = _r(v), f'{label} (raised from {_r(pref)})'
        src[name] = who
        return best

    dr = None
    if board:
        try:
            from list_nets import read_design_rules
            dr = read_design_rules(board)
        except Exception as e:                               # noqa: BLE001
            notes.append(f'could not read design rules: {type(e).__name__}: {e}')

    def con(key):
        """A DRC-enforced Board Constraint (list_nets.board_constraint).

        With a RAW fallback, because the helper collects a fixed list
        (``list_nets._CONSTRAINT_FIELDS``) that does not include
        ``min_via_drill`` -- so reading it through the helper alone is a
        branch that looks live and always answers None. This repo's own
        ``fix_project_for_output`` writes that key, and the bench carries it
        (0.15), so the fallback is not hypothetical. The helper stays the
        primary path; this only reaches keys it does not collect.
        """
        if dr is None:
            return None
        try:
            from list_nets import board_constraint
            v = board_constraint(board, key, dr)
            if v is not None:
                return v
        except Exception:                                    # noqa: BLE001
            pass
        try:
            import json
            with open(_pro_path(board), encoding='utf-8') as fh:
                raw = json.load(fh)
            v = ((raw.get('board') or {}).get('design_settings') or {}) \
                .get('rules', {}).get(key)
            return float(v) if isinstance(v, (int, float)) else None
        except Exception:                                    # noqa: BLE001
            return None

    def cls(key):
        """A Default net-class value.  0 is UNSET, not a floor (#966)."""
        if dr is None:
            return None
        try:
            from list_nets import board_default_netclass_param
            return board_default_netclass_param(board, key, dr)
        except Exception:                                    # noqa: BLE001
            return None

    n_cu = _copper_layer_count(board) if board else 2
    try:
        from fab_tiers import physical_fab_floor
        fab = physical_fab_floor(n_cu)
    except Exception:                                        # noqa: BLE001
        fab = {}
        notes.append('no fab floor available')

    dru_max, dru_notes = (_dru_outer_max(board, layers) if board else (None, []))
    notes.extend(dru_notes)

    clearance = pick('clearance', SPEC_CLEARANCE, [
        ('Default net class', cls('clearance')),
        ('rules.min_clearance', con('min_clearance')),
        (f'.kicad_dru rule on {"/".join(layers)}', dru_max),
        ('fab floor', fab.get('clearance')),
    ])
    track = pick('track', TRACK, [
        ('rules.min_track_width', con('min_track_width')),
        ('fab floor', fab.get('track_width')),
    ])
    fan_track = pick('fan_track', FAN_TRACK, [
        ('rules.min_track_width', con('min_track_width')),
        ('fab floor', fab.get('track_width')),
    ])
    via_drill = pick('via_drill', VIA_DRILL, [
        ('rules.min_via_drill', con('min_via_drill')),
        ('rules.min_through_hole_diameter', con('min_through_hole_diameter')),
        ('fab floor', fab.get('via_drill')),
    ])
    annular = con('min_via_annular_width')
    via_size = pick('via_size', VIA_SIZE, [
        ('rules.min_via_diameter', con('min_via_diameter')),
        ('fab floor', fab.get('via_diameter')),
        ('annular ring over the drill',
         None if not annular else via_drill + 2 * float(annular)),
    ])

    lane_slice = _r(track + _r(clearance + HUG_OVER))
    lane_pitch = pick('lane_pitch', LANE_PITCH, [("one lane's slice", lane_slice)])
    exit_pitch = pick('exit_pitch', EXIT_PITCH, [("one lane's slice", lane_slice)])

    band_tip, tip_src = BAND_TIP, 'chain default (no destination given)'
    if dest_ref and board:
        pitch = _array_pitch(board, dest_ref)
        if pitch:
            band_tip = _r(pitch / 2 + EXIT_MARGIN)
            tip_src = f'{dest_ref} pitch {pitch} / 2 + exit margin {EXIT_MARGIN}'
        else:
            tip_src = f'chain default ({dest_ref} pitch unreadable)'
    src['band_tip'] = tip_src

    h2h = con('min_hole_to_hole')
    edge = con('min_copper_edge_clearance')
    src['hole_to_hole'] = 'rules.min_hole_to_hole' if h2h else 'board declares none'
    src['edge_clearance'] = ('rules.min_copper_edge_clearance' if edge
                             else 'board declares none')

    if dr is not None and not (dr.get('classes') or dr.get('constraints')):
        notes.append('board declares NO net class and NO board constraint -- '
                     'every number below is the chain default or the fab floor')

    return Rules(clearance=clearance, track=track, via_size=via_size,
                 via_drill=via_drill, hole_to_hole=h2h, edge_clearance=edge,
                 fan_track=fan_track, lane_pitch=lane_pitch,
                 exit_pitch=exit_pitch, band_tip=band_tip,
                 board=board, notes=tuple(notes), sources=src)


# ----------------------------------------------------------------- install

ACTIVE = None
"""The Rules the last `install` put in place (None = the modules carry their
defaults).  A module that binds constants at import time and is imported
LATE can consult this."""


def install(rules, verbose=False):
    """Write ``rules`` into the module constants the chain reads.

    Only touches modules ALREADY IMPORTED (``sys.modules``), so this file
    never imports the chain and can never make an import cycle; a stage
    installs after its own imports, which is where the constants live.
    Idempotent.  Returns the list of ``module.NAME`` it set.
    """
    global ACTIVE
    ACTIVE = rules
    done = []

    def _modules(mod):
        """Every live module object for the logical name ``mod``.

        THE `__main__` TRAP, and it is not theoretical -- it shipped in the
        first version of this file and the positive control caught it. A
        stage is run as ``python3 braid.py``, so the router's own module is
        named ``__main__``: ``sys.modules['braid']`` is absent and a plain
        ``sys.modules.get('braid')`` writes NOTHING, silently, while the
        stage prints the resolved rules it is not using. The board came out
        routed at the 0.1 mm defaults with a 0.15 line in the log.

        Both can also be live at once (a stage run as a script that ALSO
        imports the module under its own name gets two distinct module
        objects), so this returns a list and every one is written.
        """
        out = []
        m = sys.modules.get(mod)
        if m is not None:
            out.append(m)
        main = sys.modules.get('__main__')
        if main is not None and main not in out:
            f = getattr(main, '__file__', '') or ''
            if os.path.splitext(os.path.basename(f))[0] == mod:
                out.append(main)
        return out

    def put(mod, name, value):
        for m in _modules(mod):
            if not hasattr(m, name):
                continue
            setattr(m, name, value)
            if f'{mod}.{name}' not in done:
                done.append(f'{mod}.{name}')

    # topo_strings: the spec, the lane track, and the margin derived from them
    put('topo_strings', 'TRACK', rules.track)
    put('topo_strings', 'SPEC_CLEAR', rules.clearance)
    put('topo_strings', 'MARGIN_OUT', rules.margin_out)

    # braid: the five numbers and the four constants derived from them
    put('braid', 'TRACK', rules.track)
    put('braid', 'CLEAR', rules.hug)
    put('braid', 'SPEC_CLEARANCE', rules.clearance)
    put('braid', 'VIA_SIZE', rules.via_size)
    put('braid', 'VIA_DRILL', rules.via_drill)
    put('braid', 'BAND_GAP', rules.band_gap)
    put('braid', 'HALF_SEP', rules.half_sep)
    put('braid', 'VIA_NEED', rules.via_need)
    put('braid', 'END_KEEP', rules.end_keep)
    put('braid', 'LPITCH', rules.lane_pitch)
    put('braid', 'MINP', rules.exit_pitch)

    # select_moves: the band geometry (SPLIT_BLOCKS path)
    put('select_moves', 'BAND_TIP', rules.band_tip)
    put('select_moves', 'BAND_LPITCH', rules.lane_pitch)
    put('select_moves', 'NEST_IN', rules.lane_slice)

    # source_realize: the production engine's fanout geometry
    put('source_realize', 'FAN_TRACK', rules.fan_track)
    put('source_realize', 'FAN_CLEAR', rules.fan_clear)

    # the two modules that bind braid's constants into their OWN locals at
    # import time -- rebind them, in case they were imported before this call
    for mod in ('cut_ledger', 'collapse_dives'):
        for m in _modules(mod):
            for name, value in (('TRACK', rules.track), ('CLEAR', rules.hug),
                                ('VIA_SIZE', rules.via_size),
                                ('VIA_DRILL', rules.via_drill)):
                if hasattr(m, name):
                    setattr(m, name, value)
                    if f'{mod}.{name}' not in done:
                        done.append(f'{mod}.{name}')
            if hasattr(m, 'NEED'):
                # cut_ledger's own expression, NOT the rounded lane_slice: it
                # spells one lane's slice ``TRACK + CLEAR``, which is one ULP
                # below the 0.232 select_moves spells as a literal. Keeping
                # the expression keeps this tool bit-identical to what it
                # printed before -- the two spellings of one quantity are
                # themselves an instance of the drift this module ends.
                m.NEED = rules.track + rules.hug
                if f'{mod}.NEED' not in done:
                    done.append(f'{mod}.NEED')

    if verbose:
        print(rules.describe(), flush=True)
    return done


def install_for(board, dest_ref=None, verbose=False):
    """`rules_of` + `install` -- what a stage's ``main()`` calls."""
    r = rules_of(board, dest_ref=dest_ref)
    install(r, verbose=verbose)
    return r


def main(argv=None):
    """``rules.py BOARD [--dest REF] [--key clearance]`` -- what the chain
    grades at.  ``--key`` prints one bare number for a shell to read."""
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board')
    ap.add_argument('--dest', default=None, help='destination ref (band_tip)')
    ap.add_argument('--key', default=None,
                    help='print just this field, bare (for chain_k.sh)')
    a = ap.parse_args(argv)
    r = rules_of(a.board, dest_ref=a.dest)
    if a.key:
        v = getattr(r, a.key, None)
        print('' if v is None else v)
        return 0
    print(r.describe())
    return 0


if __name__ == '__main__':
    sys.exit(main())
