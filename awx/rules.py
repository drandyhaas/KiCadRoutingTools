#!/usr/bin/env python3
"""ONE definition of the topo chain's design constants.

Until this module the chain's geometry was written out five times:
``topo_strings.TRACK = 0.127``, ``braid.CLEAR = 0.105`` /
``SPEC_CLEARANCE = 0.1`` / ``VIA_SIZE = 0.25`` / ``VIA_DRILL = 0.15``,
``NEST_IN =
0.232``, ``source_realize.FAN_TRACK`` / ``FAN_CLEAR = 0.1``, plus
``track_width=0.1, clearance=0.1`` hardcoded at six production-engine call
sites and a literal ``--clearance 0.1`` in `chain_k.sh` and `grade_k.py`.
Nine of those were derived from another one by a formula that lived only in
a comment, and two of them (``NEST_IN``, ``cut_ledger.NEED``) are the same
quantity spelled two ways, one ULP apart.

This module is where they live now. It does NOT read the board.

WHY IT DOES NOT READ THE BOARD (Andy, 2026-09-15)
-------------------------------------------------
An earlier version of this file resolved the numbers from the board --
sibling ``.kicad_pro`` Default net class, ``.kicad_dru`` layer rules,
``list_nets.board_constraint``, the ``fab_tiers`` floor. That was REMOVED,
and the reason is worth keeping: **py_router already does that resolution**,
properly and in one place, and the topo chain is not a second front for it.
The chain will be DRIVEN by the main router, and when it is, the geometry
will be SUPPLIED -- see `Rules.from_router_config`, which is the whole of
the handover. Two resolvers reading the same board would be two chances to
disagree about what a board asks for; this way there is one.

So the values below are the chain's own constants, not a policy about
boards. What the board asks for is the main router's business.

THE QUANTITIES, and the formula that produced each literal
-----------------------------------------------------------
    clearance  0.1     the spec clearance: what the fanout lays at, what
                       `chain_k.sh` / `grade_k.py` grade at, and what the
                       output project records.
    hug        0.105   = clearance + 5 um  (``braid.CLEAR``) -- the spec
                       plus a hair, so a hug does not sit exactly on it.
    track      0.127   the braid's lane track (5 mil).
    fan_track  0.1     the production engine's fanout stub width. The board
                       carries TWO track widths ON PURPOSE (README, "One
                       source for every routing number"): the fanout's
                       stubs at this, the braid's lanes wider.
    fan_clear  = clearance. The fanout lays at the spec; there is no second
                       clearance.
    via_size / via_drill  0.25 / 0.15
    grid       0.025   the routing grid the chain plans and routes on (``braid.GRID``, the
                       config's ``grid_step``).
    pair_gap   = hug + a grid diagonal  (``pairs.GAP``) -- a pair's P-to-N edge
                       gap. The pose router's short test (``gap < clearance``)
                       runs on the legs it GENERATES on the grid, and at a
                       corner the inner leg lands up to a grid diagonal closer.
    lane_slice 0.232   = track + hug -- one
                       lane's centre-to-centre slice: two parallel tracks of
                       width w at clearance c sit at pitch w + c.
    lane_pitch 0.35    = max(0.35, lane_slice)  (``braid.LPITCH``). 0.35 is the chain's
                       pitch; the FLOOR is that a comb can never pack
                       tighter than one lane's slice. It binds above
                       clearance 0.218.
    exit_pitch 0.38    = max(0.38, lane_slice)  (``braid.MINP``), same
                       reading at the exits.
    hole_to_hole / edge_clearance   None by default. The braid reads these
                       two off the board itself (``list_nets.board_constraint``)
                       and applies them tighten-only; that is left exactly
                       where it was. They are fields here only so a supplied
                       router config can carry them.

    and three expressions the modules already spelled, re-evaluated here so
    the formula has one home: ``half_sep`` (= (track + clearance) / 2), ``via_need`` (= via_size/2 +
    hug + track/2 + 0.03), ``end_keep`` (= track + hug + 0.05),
    ``margin_out`` (= clearance + track/2).

FLOAT BITS ARE PART OF THE CONTRACT
-----------------------------------
``0.1 + 0.005`` is NOT the double ``0.105`` -- it is one ULP above it --
and ``0.127 + 0.105`` is one ULP BELOW ``0.232``. A 1-ULP difference in a
clearance is a different board: it moves a grid cell, which moves a lane,
which changes the via count. So every DERIVED quantity is rounded to 6
decimals (the same normalization `fix_kicad_drc_settings` uses on its
writeback), which lands exactly on the literal it replaces. The four
expressions that the modules already spelled as arithmetic are
re-evaluated in their ORIGINAL order and are NOT rounded, so they too come
out bit-identical. `tests/test_622_rules_of.py` asserts every one of these
with ``==`` on the float AND on ``.hex()``, never ``approx``.

USING IT
--------
Each chain stage is its own process, so each stage's entry point does,
once, near the top of ``main()``::

    import rules as _rules
    _rules.install_defaults()

`install` writes the values into the module-level constants the chain
already reads (``topo_strings.TRACK``, ``braid.CLEAR``, ...) and
re-evaluates the constants derived from them. The literals stay as each
module's DEFAULT, so a module imported without an install behaves exactly
as it did before this file existed -- which is what makes the chain
byte-identical BY CONSTRUCTION rather than by measurement. (Measured too;
see the README section.)

Today `install_defaults()` installs exactly what the modules already hold,
so it is inert. That is the point: it is the SEAM. When the main router
drives the topo chain it will call ``install(Rules.from_router_config(cfg))``
instead, and one call moves the whole chain onto the router's geometry.

Why install-into-constants rather than making each constant a function: the
chain's consumers read these through module ATTRIBUTES at call time
(``te.VIA_SIZE``, ``br.TRACK``, ``br.CLEAR``) in ~30 places, so one install
reaches all of them, and no hot loop grows a function call. The one module
that binds them into locals at import time (`cut_ledger`) re-reads after
installing.

DEBT THIS FILE DOES NOT PAY (recorded, not fixed)
-------------------------------------------------
  * ``braid.BLOCK_GAP`` 0.45, ``LEG_W``, ``LEG_REQ``, ``LEG_O``,
    ``CROSS_TUBE``, ``HEAD_RUN`` are constants with no formula behind them;
    only the lane-slice floors of the two pitches are modelled.
  * the 0.025 routing grid in ``braid.setup`` is sized against the fanout's
    0.25 stub packing ("the legal minimum, track + clearance = 0.227, plus
    23 um"). That sentence is a rule computation and the grid is still a
    literal -- so a supplied geometry would not move it.
  * per-NET-CLASS clearance: the braid prices ONE scalar for the whole bus.
    Whatever the main router supplies will be one number too, so a board
    whose bus class differs from its Default class needs a decision that
    does not exist yet on either side.
"""

KRT_TOOL = {'scope': [], 'kind': 'utility'}   # #937: a research tool (awx), catalogued, shown at no door
import math
import os
import sys
from dataclasses import dataclass, field

# ----------------------------------------------------------- the constants
# These are the literals this module replaces, and this is their one home.
SPEC_CLEARANCE = 0.1      # the spec clearance the chain plans and grades at
TRACK = 0.127             # the braid's lane track (5 mil)
HUG_OVER = 0.005          # ...and the hug's 5 um over the spec
VIA_SIZE = 0.25
VIA_DRILL = 0.15
GRID = 0.025              # the routing grid the chain plans and routes on (braid.setup)
FAN_TRACK = 0.1           # the production engine's fanout stub width
LANE_PITCH = 0.35         # braid.LPITCH
EXIT_PITCH = 0.38         # braid.MINP

_ROUND = 6                # decimals every derived quantity lands on


def _r(x):
    """The 6-decimal normalization. See "FLOAT BITS" above -- without it
    ``clearance + HUG_OVER`` is one ULP off the 0.105 it replaces."""
    return round(float(x), _ROUND)


@dataclass(frozen=True)
class Rules:
    """The topo chain's design constants. Millimetres.

    ``hole_to_hole`` and ``edge_clearance`` may be None (nothing supplied);
    every other field is always a number.
    """
    clearance: float = SPEC_CLEARANCE
    track: float = TRACK
    via_size: float = VIA_SIZE
    via_drill: float = VIA_DRILL
    grid: float = GRID
    hole_to_hole: float = None
    edge_clearance: float = None
    fan_track: float = FAN_TRACK
    lane_pitch: float = LANE_PITCH
    exit_pitch: float = EXIT_PITCH
    source: str = 'awx/rules.py constants'
    notes: tuple = ()

    # -- derived -----------------------------------------------------------
    @property
    def hug(self):
        """``braid.CLEAR``: the spec plus 5 um so a hug does not sit exactly
        on it. 0.1 -> 0.105."""
        return _r(self.clearance + HUG_OVER)

    @property
    def fan_clear(self):
        """``source_realize.FAN_CLEAR``: the fanout lays at the spec."""
        return self.clearance

    @property
    def pair_gap(self):
        """A pair's P-to-N edge gap: the hug and a grid diagonal (the inner leg of a turn the pose router
        generates on the grid lands up to that much closer). ``pairs.GAP``."""
        return _r(self.hug + math.sqrt(2) * self.grid)

    @property
    def lane_slice(self):
        """One lane's slice, track + hug. 0.232."""
        return _r(self.track + self.hug)

    # The five below are braid's / topo_strings' OWN expressions, in their
    # own order, unrounded -- see "FLOAT BITS". install() writes these back
    # over the module constants.
    @property
    def half_sep(self):
        return (self.track + self.clearance) / 2     # braid.HALF_SEP

    @property
    def via_need(self):
        return self.via_size / 2 + self.hug + self.track / 2 + 0.03  # braid.VIA_NEED

    @property
    def lane_min(self):
        return self.track + self.hug + 0.02          # braid.LANE_MIN

    @property
    def end_keep(self):
        return self.track + self.hug + 0.05          # braid.END_KEEP

    @property
    def margin_out(self):
        return self.clearance + self.track / 2       # topo_strings.MARGIN_OUT

    # -- the handover ------------------------------------------------------
    @classmethod
    def from_router_config(cls, cfg, fan_track=None):
        """Build the chain's rules from a py_router routing config.

        THIS IS THE SEAM, and it is deliberately the only one. When the main
        router drives the topo chain, the geometry comes from the router --
        which has already resolved it from the board's net classes, its
        ``.kicad_dru`` rules and the fab tier -- and the chain derives its
        own quantities from that by the formulas above. Nothing here reads a
        board.

        ``cfg`` is anything carrying the four names a `GridRouteConfig` has:
        ``clearance``, ``track_width``, ``via_size``, ``via_drill`` (a
        ``via_diameter`` spelling is accepted for the writer-side name).
        ``hole_to_hole_clearance`` and ``board_edge_clearance`` are carried
        across when present.

        TWO THINGS A ROUTER CONFIG CANNOT SAY, and they are not invented:

        * **the two track widths.** The chain lays the fanout's stubs
          narrower than the braid's lanes on purpose, and a router config
          has ONE ``track_width``. So the supplied width becomes BOTH unless
          the caller passes ``fan_track`` -- the split is a chain decision
          and stays an explicit argument rather than a silent ratio.

        The pitches keep their floor semantics: the chain's own 0.35 / 0.38,
        never below one lane's slice at the supplied geometry.
        """
        def get(*names):
            for n in names:
                v = getattr(cfg, n, None)
                if v is not None:
                    try:
                        return float(v)
                    except (TypeError, ValueError):
                        pass
            return None

        clearance = get('clearance')
        track = get('track_width')
        if clearance is None or track is None:
            raise ValueError(
                'from_router_config needs at least clearance and track_width; '
                f'got clearance={clearance!r} track_width={track!r}')
        clearance, track = _r(clearance), _r(track)
        via_size = get('via_size', 'via_diameter')
        via_drill = get('via_drill')
        slice_ = _r(track + _r(clearance + HUG_OVER))
        return cls(
            clearance=clearance,
            track=track,
            via_size=_r(via_size) if via_size is not None else VIA_SIZE,
            via_drill=_r(via_drill) if via_drill is not None else VIA_DRILL,
            grid=_r(get('grid_step')) if get('grid_step') is not None else GRID,
            hole_to_hole=get('hole_to_hole_clearance'),
            edge_clearance=get('board_edge_clearance'),
            fan_track=_r(fan_track) if fan_track is not None else track,
            lane_pitch=max(LANE_PITCH, slice_),
            exit_pitch=max(EXIT_PITCH, slice_),
            source=f'router config ({type(cfg).__name__})',
            notes=(() if fan_track is not None else
                   ('fan_track = the supplied track_width (the caller did '
                    'not split the chain\'s two track widths)',)))

    def describe(self):
        """One line per quantity -- what a stage prints, so the numbers it
        is using are visible rather than assumed."""
        out = [f'rules ({self.source}):']
        for k in ('clearance', 'track', 'via_size', 'via_drill', 'grid',
                  'hole_to_hole', 'edge_clearance', 'fan_track',
                  'lane_pitch', 'exit_pitch'):
            v = getattr(self, k)
            out.append(f'  {k:16s} {"-" if v is None else v}')
        out.append(f'  {"hug (derived)":16s} {self.hug}   clearance + {HUG_OVER}')
        out.append(f'  {"lane_slice":16s} {self.lane_slice}   track + hug')
        out.append(f'  {"pair_gap":16s} {self.pair_gap}   hug + a grid diagonal')
        for n in self.notes:
            out.append(f'  note: {n}')
        return '\n'.join(out)


DEFAULT = Rules()
"""The chain's constants.

Every module's constant is initialized from this, so a module used without
an install behaves exactly as it did when the numbers were literals.
"""


# --------------------------------------------------------------- installing

ACTIVE = None
"""The Rules the last `install` put in place (None = the modules carry their
defaults, which are DEFAULT's values). `active()` reads it."""


def active():
    """The rules in force in this process."""
    return ACTIVE if ACTIVE is not None else DEFAULT


def install(rules, verbose=False):
    """Write ``rules`` into the module constants the chain reads.

    Only touches modules ALREADY IMPORTED, so this file never imports the
    chain and can never make an import cycle; a stage installs after its own
    imports, which is where the constants live. Idempotent. Returns the list
    of ``module.NAME`` it set.
    """
    global ACTIVE
    ACTIVE = rules
    done = []

    def _modules(mod):
        """Every live module object for the logical name ``mod``.

        THE `__main__` TRAP, and it is not theoretical -- it shipped in the
        first version of this file and a control run caught it. A stage is
        run as ``python3 braid.py``, so the router's own module is named
        ``__main__``: ``sys.modules['braid']`` is absent and a plain
        ``sys.modules.get('braid')`` writes NOTHING, silently, while the
        stage prints the rules it is not using.

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

    # braid: the five numbers and the constants derived from them
    put('braid', 'TRACK', rules.track)
    put('braid', 'CLEAR', rules.hug)
    put('braid', 'SPEC_CLEARANCE', rules.clearance)
    put('braid', 'VIA_SIZE', rules.via_size)
    put('braid', 'VIA_DRILL', rules.via_drill)
    put('braid', 'HALF_SEP', rules.half_sep)
    put('braid', 'VIA_NEED', rules.via_need)
    put('braid', 'LANE_MIN', rules.lane_min)
    put('braid', 'END_KEEP', rules.end_keep)
    put('braid', 'LPITCH', rules.lane_pitch)
    put('braid', 'MINP', rules.exit_pitch)
    put('braid', 'GRID', rules.grid)

    # pairs: the pair gap (unless the invocation set its own, BRAID_PAIR_GAP)
    if not float(os.environ.get('BRAID_PAIR_GAP', '0') or 0):
        put('pairs', 'GAP', rules.pair_gap)

    # source_realize: the production engine's fanout geometry
    put('source_realize', 'FAN_TRACK', rules.fan_track)
    put('source_realize', 'FAN_CLEAR', rules.fan_clear)

    # the module that binds braid's constants into its OWN locals at
    # import time -- rebind it, in case it was imported before this call
    for mod in ('cut_ledger',):
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


def install_defaults(verbose=False):
    """What a stage's ``main()`` calls: put the chain's constants in place.

    Inert today -- it installs exactly what the modules already hold -- and
    that is the point. It is the seam: when the main router drives the topo
    chain, this call becomes
    ``install(Rules.from_router_config(cfg))`` and the whole chain moves
    onto the router's geometry at once.
    """
    install(DEFAULT, verbose=verbose)
    return DEFAULT


def main(argv=None):
    """``rules.py [--key clearance]`` -- the chain's constants.
    ``--key`` prints one bare number, for `chain_k.sh`."""
    import argparse
    ap = argparse.ArgumentParser(description='the topo chain design constants')
    ap.add_argument('--key', default=None,
                    help='print just this field, bare (for chain_k.sh)')
    a = ap.parse_args(argv)
    if a.key:
        v = getattr(DEFAULT, a.key, None)
        print('' if v is None else v)
        return 0
    print(DEFAULT.describe())
    return 0


if __name__ == '__main__':
    sys.exit(main())
