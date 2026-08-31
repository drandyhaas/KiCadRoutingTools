"""
Fanout-clearance placement repair: tidy decoupling caps around BGA fanout.

Issue #130: a BGA fanout drops vias near the ball field. Where a foreign-net
via lands under a decoupling cap, the via copper overlaps the cap pad -> a
real PAD-VIA DRC violation at the clearance floor. The root cause is
*placement*, so the fix is to nudge the caps.

This runs AFTER bga_fanout.py (so the real vias exist), because the fanout
usually only escapes *signal* balls; GND/power balls are connected later by
dropping vias straight down. That gives two complementary goals:

  * AVOID  - a cap pad must clear, by `clearance`, every real fanout via of a
             DIFFERENT net (these are the #130 violations), plus any foreign
             escape track on the cap's own copper side (escapes can land on the
             bottom; the under-pad fanout deliberately routes through movable
             caps' zones expecting THIS step to move them, #278), plus any
             foreign-net COMPONENT pad (#235/#275: a move may never slide a cap
             pad onto a neighbour's pad -> a PAD-PAD short). All three are
             violations to FIX when present at the seed, not just to avoid
             introducing. Same-net vias/pads are fine - a cap pad may sit right
             on one (via-in-pad / same-net copper sharing).
  * ATTRACT - pull each cap pad toward the nearest BGA ball of its OWN net, so
             that a later GND/power via dropped at the ball also lands on the
             cap pad (one shared via connects ball + cap + plane).

Move set: small nudges within a per-cap displacement budget plus 90-degree
rotations. Cost = foreign-via penetration (strong) + same-net attraction +
displacement (mild, so caps stay near their seed). The board edge,
locked-part courtyards, AND other caps' courtyards are HARD constraints: a
move may never introduce or worsen a courtyard overlap, so caps never end up
on top of each other. A cap that can't clear a foreign via within the base
budget gets its budget grown and rotations enabled until it fits or hits the
displacement cap; if it still can't, it's reported unresolved.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Set, Tuple

from kicad_parser import PCBData, Segment, find_components_by_type
import routing_defaults as defaults
from bga_fanout.grid import analyze_bga_grid
from placement.parser import extract_courtyard_bboxes, extract_locked_refs
from placement.utility import compute_footprint_bbox_local, snap_to_grid
from placement.legality import (BoardOutlineGate, PadClearanceModel, PadFloor,
                                _pad_carries_copper, format_required_clause,
                                pad_half_extents, point_to_seg_dist,
                                rect_gap, ring_is_rect,
                                rotate_local_bounds)

ROTATIONS = [0.0, 90.0, 180.0, 270.0]
EPS = 1e-6
# The keep-out written into an eff cell for a pair sharing NO copper layer
# (#731). Large and NEGATIVE, so _seg_shortfalls' EXISTING cheap bbox reject
# (min(x1, x2) > bx1 + halfw) fires on the first comparison: the layer gate
# costs the innermost loop nothing -- no extra branch, no mask row. Finite
# rather than -inf so nothing can produce a NaN if a caller sums a row.
_OFF_LAYER = -1.0e9

# The diameter to assume for a via whose size the board does not carry -- a
# literal `(size 0)` token, or a padstack whose pcbnew GetFrontWidth() came back
# 0. This is the value nudge_vias_for_unresolved hard-coded at four sites before
# #732 named it, so nothing moves.
#
# Module-private on purpose. It governs how THIS pass prices a barrel it cannot
# measure, which is an operator-facing placement decision; it is emphatically
# not a number the DRC/connectivity graders should adopt. KiCad honours a
# declared size literally: `(size 0)` is a hard `via_diameter` DRC error there
# (min 0.5000mm, actual 0.0000mm) rather than a via KiCad silently sizes for
# you, and such a barrel joins nothing -- so a grader that substituted a
# diameter would report CONNECTED what KiCad reports OPEN. Measured; see
# TestTheGradersStillReadTheBoard in tests/test_732_...py, which states the
# geometry the experiment needs (perpendicular copper-to-barrel separation,
# not stub endpoint offset) because the naive version shows nothing.
#: `_register_via`'s "no keep-out supplied, derive one" marker (#747).
#: A DISTINCT object rather than None, because a via 4-tuple's element 3 could
#: itself be None -- nothing in this module builds one, but the tuples come
#: from callers who assign the list wholesale, and `None` would then mean both
#: "derive" and "the keep-out IS None" at the same slot. Overloaded, a
#: relocated tuple carrying None either raised or had its keep-out silently
#: re-derived, which is exactly what the builder's docstring forbids. A review
#: measured both.
_DERIVE_KEEPOUT = object()

_UNREADABLE_VIA_SIZE = 0.5  # mm

# The DRILL to assume for a via whose hole the board does not carry -- a literal
# `(drill 0)` token, or a padstack whose pcbnew GetDrill() came back 0. This is
# the value valid_via_pos hard-coded at three sites before #750 named it, so
# nothing moves.
#
# Module-private for CAP_EDGE_CLEARANCE's reason below, NOT the one above it:
# _UNREADABLE_VIA_SIZE argues privacy from what the GRADERS must not adopt,
# which does not transfer -- KiCad rejects `(drill 0)` at the fab floor rather
# than reporting a barrel connected that is not. The argument that does
# transfer is CAP_EDGE_CLEARANCE's: a PLACEMENT number that happens to equal a
# fab default and must not be wired to it.
#
# And it does equal several. 0.3 is routing_defaults.VIA_DRILL, BGA_VIA_DRILL,
# and bga_fanout.constants.DEFAULT_VIA_SIZE -- that last one a SIZE, which is
# the trap. Those are the hole this toolchain DRILLS when it creates a via;
# this is a pricing assumption about a barrel this pass cannot MEASURE. They
# agree today by accident of value, and wiring them would let a change to a fab
# default silently move a hole-to-hole keep-out.
#
# What the pair prices, since the two constants are read on different paths:
# through a real _Repair an unreadable size resolves to --default-via-size, so
# at the CLI's 0.3 the assumed via is 0.3/0.3 -- a ZERO annular ring; on the
# duck-typed path it is _UNREADABLE_VIA_SIZE/this, 0.5/0.3, a 0.1 ring, which
# is exactly routing_defaults' standard via. #750 records both readings.
_UNREADABLE_VIA_DRILL = 0.3  # mm

# The cap-placement margin from Edge.Cuts when the operator names none and the
# board declares none. A PLACEMENT margin (keep a decap off the rim so it can be
# assembled and reworked), not a DRC floor -- which is why it is 0.55 and not one
# of routing_defaults' two edge numbers: BOARD_EDGE_CLEARANCE is 0.0, the SIGNAL
# sentinel meaning "no edge rule, use the copper-copper clearance" (list_nets.py
# :507-511), and adopting it here would collapse this margin to `clearance`;
# PLANE_EDGE_CLEARANCE 0.5 belongs to pours. list_nets.board_floor_knobs already
# spells this same 0.55 as its `edge_default` for the sibling placement CLIs.
CAP_EDGE_CLEARANCE = 0.55  # mm


def resolve_cap_edge_clearance(pcb_file, explicit=None):
    """The cap-placement edge margin, resolved ONCE. Returns ``(mm, source)``.

    THE ONE ANSWER for every front end (#733). The CLI, the GUI plugin and the
    animator each carried their own hard-coded 0.55 and only the CLI exposed a
    flag, so a board declaring a real copper-to-edge rule reached three different
    margins depending on which front invoked the same engine.

    TIGHTEN-ONLY, and that is deliberate rather than a softer form of the
    board-first rule `list_nets.board_floor` implements. `board_floor` is
    explicitly NOT raise-only, which is correct for a routing floor and wrong
    here, because the value it would read back is one THIS pipeline wrote:
    `fix_project_for_output` PINS `min_copper_edge_clearance` UP to the fab
    copper-to-edge floor 0.20 on every board it writes (fix_kicad_drc_settings.py
    :608 -> :749-754, the one raise-allowed key), and `bga_fanout.py` calls it
    (bga_fanout/__init__.py:4328) -- as does place_fanout_clearance.py itself.
    The documented pipeline is `bga_fanout.py -> place_fanout_clearance.py`, so
    EVERY board this pass is handed in a real chain declares >= 0.20 even when its
    author declared nothing. Read plainly board-first, that 0.20 comes back tagged
    "board constraint" -- our own default wearing the board's name -- and the cap
    margin silently drops 0.55 -> max(clearance, 0.20). 80/184 corpus boards
    declare below 0.20 (fix_kicad_drc_settings.py:356), so that is the common case,
    not a corner. Raising only is immune to the pin and still honours a board that
    genuinely wants MORE room than 0.55.

    An EXPLICIT POSITIVE value is honoured as given, in both directions: that is
    the operator overriding the pass, and --board-edge-clearance 0.2 must mean
    0.2 even though it is below the default.

    A NON-POSITIVE explicit value is UNSET, not a margin of zero. Cite the right
    precedent for that, because there are two rules in this codebase and they
    differ exactly here: `board_floor` / `board_floor_knobs` apply the
    non-positive-is-unset rule to a DECLARED value only and honour an explicit
    one unconditionally (`if explicit is not None: return float(explicit)`,
    list_nets.py:450), while `effective_board_edge_clearance` applies it to the
    CLI value too (`cli_value if (cli_value and cli_value > 0) else ...`,
    fix_kicad_drc_settings.py:357). This follows the latter, and deliberately
    diverges from `board_floor` on this one point. Not pedantry: the GUI's Min
    Edge Clearance spin control is
    CREATED at defaults.BOARD_EDGE_CLEARANCE, which is 0.0, with a range minimum
    of 0.0 -- so an operator who ticks the override box and types nothing hands
    this function a 0. Honoured literally that would drop the cap margin to
    max(clearance, 0) = the bare clearance: the exact 0.30mm under-block #733
    exists to close, re-opened through a different door and LOOSER than the
    0.55 the plugin used before this change. Caught by the #733 review.

    `source` is for disclosure only: 'cli' | 'board constraint, raised' |
    'fixed default'. There is deliberately NO 'unreadable project' source, unlike
    board_floor's -- `board_constraint` swallows its own read errors and
    `read_design_rules` swallows JSONDecodeError/OSError (list_nets.py:199), so a
    corrupt project is genuinely indistinguishable here from one that declares
    nothing. Both report 'fixed default'. Stated rather than papered over with a
    label that could never print; the VALUE is 0.55 either way, which is the safe
    direction for a raise-only rule.
    """
    if explicit is not None and explicit > 0:
        return float(explicit), 'cli'
    try:
        from list_nets import board_constraint
        declared = board_constraint(pcb_file, 'min_copper_edge_clearance')
    except Exception:                                          # noqa: BLE001
        declared = None    # e.g. pcb_file is not a path at all (an unsaved GUI
                           # board): no declaration to raise to, take the default
    # A declaration below the default cannot LOWER this margin (see above), and a
    # non-positive one is UNSET rather than a floor of zero -- KiCad writes 0 into
    # these fields for "not configured". `> CAP_EDGE_CLEARANCE` subsumes both.
    if declared is not None and declared > CAP_EDGE_CLEARANCE:
        return float(declared), 'board constraint, raised'
    return CAP_EDGE_CLEARANCE, 'fixed default'


def resolve_pair_clearance(pcb_file, clearance=None):
    """The flat different-net PAIR floor, resolved ONCE. ``(mm, source)``.

    THE ONE ANSWER for every front end, the shape #733 gave the edge margin and
    #756 gave the drill floors. This one is the last of the three, and it is the
    one that carried a defect rather than a gap (#768).

    Two branches, and they are CLAUDE.md's, not this function's invention:

      ``clearance is None``  -- the flag was OMITTED. Base is the board's own
                                Default net-class clearance ('board netclass'),
                                else ``routing_defaults.CLEARANCE``
                                ('fixed default'). Every class is then honoured
                                as declared, and the caller writes no clamp.
      ``clearance`` given    -- it is a CEILING. Base is ``min(Default class,
                                clearance)`` ('cli'), so the Default class is
                                capped exactly like every other class, and the
                                caller passes the same number as
                                ``netclass_ceiling`` so the map is capped too.

    This is `route.py`'s resolution with the same vocabulary as
    `list_nets.board_floor_knobs`, which is what the five sibling placement CLIs
    use. It is spelled here rather than delegated to `board_floor_knobs` for one
    reason: that helper has no notion of a ceiling, so it cannot express
    ``min(Default, cli)`` -- handed an explicit value it returns it verbatim as
    'cli', which is the OMITTED branch's arithmetic wearing the GIVEN branch's
    label.

    IT DIVERGES FROM route.py ON ONE INPUT, deliberately, and the docstring used
    to claim they were the same resolution: a DECLARED clearance of 0 or less.
    route.py reads that straight (`board_default_netclass_clearance` returns
    0.0, and only the hole/edge floors go through `resolve_cli_floor`'s unset
    rule), so it would price at 0. This follows `board_floor_knobs`
    (list_nets.py:352-355) and treats it as UNSET, because KiCad writes 0 into
    these fields for "not configured" and a pass that priced every pair at 0
    would move nothing and grade everything clean.

    NOT raise-only, and deliberately not wrapped at the fab floor. See
    `fab_pair_clearance_floor` below for why, and note that this is the opposite
    decision from `resolve_drill_floors` -- for a checkable reason, not a taste.
    """
    from list_nets import board_default_netclass_clearance
    import routing_defaults as _defaults
    try:
        declared = board_default_netclass_clearance(pcb_file)
    except Exception:                                          # noqa: BLE001
        declared = None    # e.g. pcb_file is not a path at all (an unsaved GUI
                           # board). Same swallow as resolve_cap_edge_clearance.
    # A non-positive declaration is UNSET, not a floor of zero -- KiCad writes 0
    # for "not configured", and `board_floor_knobs` encodes the same rule
    # (list_nets.py:352-355). Reading it straight collapses the pass to no
    # clearance at all.
    if declared is not None and declared <= 0:
        declared = None
    if clearance is None:
        if declared is not None:
            return float(declared), 'board netclass'
        return float(_defaults.CLEARANCE), 'fixed default'
    if declared is not None:
        return min(float(declared), float(clearance)), 'cli'
    return float(clearance), 'cli'


def fab_pair_clearance_floor(pcb_data):
    """``(mm, copper_layer_count)`` -- the fab copper-clearance floor for this
    board's LAYER BUCKET. For DISCLOSURE ONLY; never applied.

    Unlike the drill floors, this one MOVES with the layer count: `fab_floor_min`
    buckets at 2 (0.10mm) vs 4 (0.09mm), and 3 copper layers already rounds into
    the 4 bucket. So "is this clearance manufacturable" has a different answer on
    a 2-layer board than on a 4-layer one, and a hardcoded number would be wrong
    on one of them.

    NOT WRAPPED, and that is the checkable half. #756's rule for this function's
    two siblings is MATCH THE GRADER: raise a floor here iff `check_drc` raises
    it. `check_drc` does NOT fab-floor copper clearance. Its chain is
    `_pair_cl` (check_drc.py:2042-2047), ``max(clearance, ncl_a, ncl_b)``
    then the dru replace, and then `_pad_pair_cl` (:2049-2056) for the pad
    override -- with no `fab_floor_min` anywhere in either;
    `fab_floor_min` enters only the SIZE checks (track width / via diameter /
    via drill), and `_pin_up` covers board-edge, hole-to-hole and hole clearance
    but NOT clearance. Wrapping here would make this pass refuse cap landings
    its own grader passes -- inventing a divergence to paper over nothing.

    So the sub-fab case is DISCLOSED by the caller and priced as asked. That
    matters more since #768 than before it: the netclass tier used to rescue a
    too-small ``--clearance`` by raising the pair back up to the class, and a
    ceiling deliberately removes that rescue.

    THE GUI DOES NOT AGREE WITH THIS, and the divergence is named rather than
    papered over. Its ``_effective_geometry_floor`` ends in ``_fab_floored``,
    so a dialog clearance below the bucket floor is PINNED UP before it ever
    reaches this engine: the CLI prices 0.09 on a 2-layer board and says so,
    the GUI prices 0.10. That pin predates #768 and applies to every geometry
    knob the dialog owns, not just this one, so moving it is a change to the
    GUI's floor policy rather than to this pass -- filed, not fixed here. What
    #768 does change is that the CLI's value now reaches the pricing instead of
    being raised back up by the netclass tier, which is what makes the two
    fronts visibly differ where before they only differed in principle.
    """
    from fab_tiers import fab_floor_min
    # The SAME `.Cu` filter `resolve_drill_floors` and `_Repair.__init__` apply
    # to the SAME field, so nothing in this pass can disagree about the bucket.
    # The `or 2` fallback is check_drc's own (`len(copper_layers) if
    # copper_layers else 2`, in its size-floor block): an unreadable layer list
    # takes the CONSERVATIVE bucket, never bucket 0. check_drc counts that list
    # WITHOUT the `.Cu` filter, so the two disagree on a layer list mixing
    # copper and non-copper names -- unreachable through either parse path (the
    # text one requires '.Cu' in the name, the pcbnew one maps copper ids
    # through a canonical table), and the filter is the safer side of it.
    _cu = getattr(getattr(pcb_data, 'board_info', None), 'copper_layers', None)
    n = len([l for l in (_cu or []) if str(l).endswith('.Cu')]) or 2
    return float(fab_floor_min(n)['clearance']), n


def resolve_drill_floors(pcb_data):
    """The two DRILL-to-drill floors the via-nudge gates at (#756).

    Returns ``(via_mm, pad_mm, declared, source, fab_via, fab_pad)``:

      via_mm    via-drill <-> via-drill   (the nudger's ``H2H_VIA``)
      pad_mm    via-drill <-> pad-drill   (the nudger's ``H2H_PAD``)
      declared  the board's own ``min_hole_to_hole`` in mm, or None
      source    'board constraint' | 'fixed default' | 'unreadable project'
                (`board_floor`'s vocabulary, minus 'cli' -- there is no flag)
      fab_via   the fab floor the first two are raised FROM, and the second
      fab_pad   rung of the caller's ladder

    THE FAB TERMS ARE RETURNED, not recomputed by the caller, for two reasons
    a review found the hard way. The nudger needs them as its FALLBACK rung --
    it prefers the declared floor and re-sweeps at the fab one rather than
    abandoning a repair -- and the disclosure has to compare against the
    RESOLVED fab term rather than `defaults.HOLE_TO_HOLE_CLEARANCE`: with an
    overrides file raising the fab hole-to-hole above the packaged 0.20, a
    board declaring between the two was announced as having raised a floor the
    fab override had actually won.

    Both floors were flat literals inside `nudge_vias_for_unresolved` --
    ``H2H_VIA = 0.2``, ``H2H_PAD = 0.45`` -- hand-copied from `fab_tiers` and
    reading nothing: not the board, not ``--fab-tier``, not a
    ``--fab-overrides`` file. That is the class `list_nets.board_floor`'s own
    docstring is written about, and `bga_fanout` names it in a sibling as "the
    D9/D11 substitute-a-constant class".

    WHY THIS ONE IS WORTH READING THE BOARD FOR, when the copper-to-hole floor
    forty lines below deliberately is NOT: match the grader. `check_drc` raises
    ``hole_to_hole_clearance`` from the project (`_pin_up`, check_drc.py:3686)
    and BOTH its drill arms add that one number -- via/via at :2605, pad/via at
    :2667. So on a board declaring above the fab floor, the via/via gate was
    the ONE floor in that function sitting strictly BELOW what its own checker
    grades it at: the pass parked a relocated via at 0.20 and check_drc then
    flagged the pair. The copper-to-hole floor is a different case and stays
    flat -- see the note beside ``npth_clr``.

    ONE board rule behind BOTH numbers, and that is not an approximation.
    There is no separate pad-hole board rule to read; ``min_hole_to_hole``
    governs any two holes, which is why `list_nets.effective_floors` floors
    both of its keys at it (:124-127). The 0.45 is a FAB floor only, and it is
    why this pass is STRICTER than its own grader on the pad arm.

    HOW MUCH stricter is ``max(d, 0.45) - max(0.20, d)``, which DECAYS: 0.25 at
    a declaration of 0.20 or below, 0.15 at 0.30, and 0 at 0.45 and above. An
    earlier draft said a flat "0.25mm, and #756 does not change that"; a review
    measured it and both halves were wrong. #756 does change it, in the
    direction of agreement with the grader, and only a board declaring nothing
    keeps the full 0.25.

    RAISE-ONLY, in the code and not only in this docstring. `board_floor` is
    board-AUTHORITATIVE, not raise-only: its own docstring (list_nets.py
    :418-445) says a consumer for which downward is a FAB question must supply
    the ``max()`` itself, and cites THIS key and the qfn ``_h2h_fab`` wrap by
    name. Without it a project declaring ``min_hole_to_hole: 0.10`` would have
    this pass relocate a via to a drill pair no fab can punch.

    NOT the tighten-only shape of `resolve_cap_edge_clearance` above, and the
    difference is checkable rather than a matter of taste. That helper is
    tighten-only because `fix_project_for_output` PINS
    ``min_copper_edge_clearance`` UP on every board this pipeline writes, so a
    board-first read hands back our own default wearing the board's name.
    ``min_hole_to_hole`` gets no such pin: `apply_targets` is
    ``if cur is None or cur > target + EPS`` -- lower only, never raise
    (fix_kicad_drc_settings.py:755) -- and edge clearance is the single key
    with a raise branch above it. And `bga_fanout` passes no ``hole_to_hole``
    to `fix_project_for_output` at all, so the documented
    ``bga_fanout.py -> place_fanout_clearance.py`` chain never stamps this key.
    What a routing step upstream CAN do is write it where the board declared
    nothing (the ``cur is None`` arm), at its own resolved floor -- which is
    itself board-first plus `enforce_fab_floors`, so the ``max()`` below
    catches any lowering. That is what makes reading it back safe.

    ONE board read, TWO fab wraps -- the shape `list_nets.effective_floors`
    already spells. Hand-composed rather than routed through that helper for
    two checkable reasons, not for taste: it takes its copper-layer count from
    the pcb FILE, which is 0 for an unsaved GUI board where
    ``board_info.copper_layers`` is right; and it returns no source tag, so the
    "declared below what the fab can drill" disclosure could not be written.
    tests/test_756 pins the two compositions EQUAL on the boards that declare,
    so the reuse is checked rather than asserted in prose.

    An unsaved / in-memory board (``source_path == ""``) takes the fab floors
    and reports 'fixed default' WITHOUT looking. Not an optimisation:
    `read_design_rules("")` probes ``".kicad_pro"`` RELATIVE TO THE PROCESS CWD
    (list_nets.py:156-157), so a stray file of that name would be read as this
    board's rules. `obstacle_map.resolve_hole_clearance` guards identically
    (its `if not path: return 0.0`). The qfn/bga twins
    (qfn_fanout/__init__.py:362, bga_fanout/underpad.py:672) do NOT; filed, not
    fixed here -- and `bga_fanout/__init__.py`'s `manage_vias`, which #756 also
    touches, got the guard rather than joining them. Cited by expression rather
    than by line number: an earlier draft named :1591-1593, which is prose at
    both revisions, and the real guard has moved twice in this row already.
    """
    # `os.path` only, and locally: this module's convention is to import
    # py_router names inside the function body, and the isdir guard below
    # is the only reason this function needs the stdlib one.
    import os.path
    from fab_tiers import fab_floor_min
    # The SAME `.Cu` filter _Repair.__init__ applies to the SAME field, so the
    # two halves of this pass cannot disagree about the layer count -- and no
    # second file read, which is what keeps the `source_path == ""` path from
    # touching the disk at all. `count_copper_layers_in_file` would return 0 on
    # a live GUI board whose board_info is perfectly good.
    _cu = getattr(getattr(pcb_data, 'board_info', None), 'copper_layers', None)
    fmin = fab_floor_min(len([l for l in (_cu or []) if str(l).endswith('.Cu')]))
    # Direct indexing, not `.get(<literal>)`: FLOOR_KEYS guarantees both keys
    # and an override file can only REPLACE keys already present, so a KeyError
    # here would be a real bug -- while a `.get` default would put the 0.20 and
    # 0.45 literals straight back into this module, which is the whole issue.
    # (`fab_floor_for_param`'s alias map carries `hole_to_hole` but NOT
    # `pad_hole_to_hole`, so it is the wrong helper for this pair.)
    fab_via, fab_pad = fmin['hole_to_hole'], fmin['pad_hole_to_hole']
    path = getattr(pcb_data, 'source_path', "") or ""
    # `isdir` as well as empty: `board_floor` does `splitext(path)[0] +
    # '.kicad_pro'`, so a DIRECTORY-shaped source_path ('.', './', any folder)
    # reads that folder's project as this board's rules. Not reachable from
    # `parse_kicad_pcb` or `build_pcb_data_from_board` -- both hand over an
    # abspath to a file -- but the guard costs one stat and the failure it
    # prevents is silent.
    if not path or os.path.isdir(path):
        return fab_via, fab_pad, None, 'fixed default', fab_via, fab_pad
    from list_nets import board_floor
    # fallback=None, NOT defaults.HOLE_TO_HOLE_CLEARANCE (the qfn/bga
    # spelling): `declared` has to mean "the board said this", or the caller's
    # disclosure compares a fallback against itself and announces a board
    # declaration that does not exist.
    declared, src = board_floor(path, 'hole_to_hole', None, None)
    if declared is None:
        return fab_via, fab_pad, None, src, fab_via, fab_pad
    return (max(declared, fab_via), max(declared, fab_pad), declared, src,
            fab_via, fab_pad)


def _via_radius(via, default_size=_UNREADABLE_VIA_SIZE) -> float:
    """The copper radius of a via, in mm -- the ONE implementation of the rule.

    #732: this was spelled `v.size if v.size and v.size > 0 else
    default_via_size` in the grader and `(v.size or 0.5) / 2.0` at four nudger
    sites, so the two halves of the pass priced the same barrel differently.
    `getattr` because the nudger is public and its test harnesses pass
    duck-typed via-likes.
    """
    size = getattr(via, 'size', None)
    if not size or size <= 0:
        size = default_size
    return size / 2.0


def _via_drill_radius(via, default_drill=_UNREADABLE_VIA_DRILL) -> float:
    """The DRILL radius of a via, in mm -- the ONE implementation of the rule.

    #750: this was spelled `(x.drill or 0.3) / 2.0` at three sites inside
    valid_via_pos -- twice on `v` and once on `ov` -- while the copper radius
    in those same two gate BLOCKS already went through _via_radius. (Not on
    the same LINES: neither drill line carried a copper term. The sharper
    statement is that of the two radii summed at the capsule gate, `prad` is
    measured from the board and this one was invented.) Same shape as
    _via_radius, deliberately, including the two things that read as
    accidents:

    `getattr`, because the nudger is public and its harnesses pass duck-typed
    via-likes. This DOES widen what the pass tolerates -- the old expression
    raised AttributeError on a via-like carrying no `drill` at all, and this
    prices it at the fallback. Deliberate, and the only such object is a test
    stand-in; no parse path produces one.

    `not d or d <= 0` and NOT `or`, so a negative drill cannot take a different
    branch in the two. That is not hypothetical here: `or` keeps a negative
    value, so the drill term goes NEGATIVE and weakens the gate it is summed
    into. With both drills negative the whole hole-to-hole threshold turns
    negative and the gate can never fire at all -- measured, the pass then
    parks two 0.5mm barrels 0.05mm apart. BOTH parse paths admit a negative,
    and for two different reasons worth stating separately: the text parser's
    via-regex drill class carries a `-` and the `float()` after it checks no
    sign, while build_pcb_data_from_board's `to_mm(track.GetDrill())` applies
    no sign check either (verified on KiCad 10.0.0, which returns -0.3mm for
    such a via rather than rejecting it).

    NaN is the one remaining hole, and it is NOT closed here because it is
    not this guard's question: both spellings propagate it, every `<`
    against it is False, and the gate silently stops firing. Same before,
    same after, and the same in _via_radius.
    """
    d = getattr(via, 'drill', None)
    if not d or d <= 0:
        d = default_drill
    return d / 2.0

# These four moved to placement/legality.py, the shared home with quench.py
# (which carried byte-identical copies of the first two). Aliased rather than
# renamed: the names are used throughout this module and imported by tests.
_rotate_local_bounds = rotate_local_bounds
_ring_is_rect = ring_is_rect
_rect_gap = rect_gap
_point_to_seg_dist = point_to_seg_dist

# Objective weights. Foreign-copper clearance (via + track + pad grazes, see
# graze_penalty) dominates everything, then pulling pads onto same-net balls,
# then a mild displacement regularizer (so caps that are already fine and far
# from a same-net ball stay put). Cap-cap / locked-part overlap is a HARD
# constraint (see hard_blocked), not weighted.
VIA_WEIGHT = 50.0
ATTRACT_WEIGHT = 1.0
DISPLACEMENT_WEIGHT = 0.3


def _point_to_rect_dist(px, py, rect):
    """Distance from a point to an axis-aligned rect (0 if inside)."""
    dx = max(rect[0] - px, px - rect[2], 0.0)
    dy = max(rect[1] - py, py - rect[3], 0.0)
    return math.hypot(dx, dy)


def _pad_pair_shortfall(pads_a, pads_b, clearance, effs=None):
    """Sum of different-net pad clearance shortfalls between two movable parts'
    pad rects (#275) -- the mover-vs-mover analogue of pad_penalty. Same-net
    pad pairs are fine (shared rail copper may touch).

    `effs` (#725) is the per-pair REQUIRED clearance, `effs[ia][ib]` aligned
    with pads_a x pads_b -- a pad override / netclass / .kicad_dru layer rule
    can put a pair's requirement above (or, for a relaxing rule, below) the
    flat scalar. None keeps the flat path, which is what a board declaring
    none of them takes."""
    pen = 0.0
    for ia, (ax0, ay0, ax1, ay1, anet) in enumerate(pads_a):
        row = None if effs is None else effs[ia]
        for ib, (bx0, by0, bx1, by1, bnet) in enumerate(pads_b):
            if anet == bnet:
                continue
            gap = _rect_gap((ax0, ay0, ax1, ay1), (bx0, by0, bx1, by1))
            clr = clearance if row is None else row[ib]
            if gap < clr - EPS:
                pen += (clr - gap)
    return pen


def _segs_cross(ax, ay, bx, by, cx, cy, dx, dy):
    """True if segments AB and CD properly intersect. Collinear overlaps are
    left to the endpoint-distance fallback (an endpoint then lies ON the other
    segment, giving distance 0 anyway)."""
    def orient(px, py, qx, qy, rx, ry):
        v = (qx - px) * (ry - py) - (qy - py) * (rx - px)
        return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)
    o1 = orient(ax, ay, bx, by, cx, cy)
    o2 = orient(ax, ay, bx, by, dx, dy)
    o3 = orient(cx, cy, dx, dy, ax, ay)
    o4 = orient(cx, cy, dx, dy, bx, by)
    return o1 != o2 and o3 != o4


def _seg_to_rect_dist(x1, y1, x2, y2, rect):
    """Exact distance from a segment to an axis-aligned rect (0 if touching
    or crossing). The centre+half-diagonal model this replaces overestimates
    the keep-out of elongated pads, which both missed real grazes and
    manufactured phantom ones (#278)."""
    rx0, ry0, rx1, ry1 = rect
    if (rx0 <= x1 <= rx1 and ry0 <= y1 <= ry1) or \
       (rx0 <= x2 <= rx1 and ry0 <= y2 <= ry1):
        return 0.0
    best = float('inf')
    for ex1, ey1, ex2, ey2 in ((rx0, ry0, rx1, ry0), (rx1, ry0, rx1, ry1),
                               (rx1, ry1, rx0, ry1), (rx0, ry1, rx0, ry0)):
        if _segs_cross(x1, y1, x2, y2, ex1, ey1, ex2, ey2):
            return 0.0
        best = min(best,
                   _point_to_seg_dist(x1, y1, ex1, ey1, ex2, ey2),
                   _point_to_seg_dist(x2, y2, ex1, ey1, ex2, ey2),
                   _point_to_seg_dist(ex1, ey1, x1, y1, x2, y2),
                   _point_to_seg_dist(ex2, ey2, x1, y1, x2, y2))
    return best


class _Cap:
    """A movable cap: pad offsets + courtyard bbox, in a seed-relative frame.

    Pad offsets and board-resolved half-sizes are captured at the seed
    placement; an additional 90-degree rotation rotates the offsets and swaps
    the half-extents, which is exact for axis-aligned pads (the normal case
    for decoupling caps).
    """

    def __init__(self, fp, courtyard_local, model=None, board_copper=None):
        self.ref = fp.reference
        self.side = 'B' if (fp.layer or '').startswith('B') else 'F'
        self.seed_x, self.seed_y = fp.x, fp.y
        self.seed_rot = fp.rotation % 360
        self.x, self.y, self.rot = fp.x, fp.y, fp.rotation % 360
        # pads: (off_x, off_y, half_x, half_y, net_id) relative to fp center.
        # Copper pads only -- a footprint may define solder-paste apertures as
        # separate paste-only "pads" (e.g. gkl_misc C_0201_0603Metric), which are
        # not copper and must not enter the clearance/attraction geometry.
        self.pads = []
        # #725, mirroring PartPads (legality.py): the per-pad clearance FLOOR
        # rides in a parallel list index-aligned with self.pads, never widened
        # into the pad tuple -- pad_rects()' 5-tuple is unpacked positionally by
        # tests and by animate_fanout_clearance.py. `max_floor` is the
        # over-reach a broad phase must use (a .kicad_dru rule REPLACES, so it
        # can also LOWER a pair; a prune must never under-reach).
        # NOTE the copper filter below stays _Cap's own `endswith('.Cu')` test
        # and is deliberately NOT unified with legality._pad_carries_copper,
        # which additionally rejects np_thru_hole. Unifying would change which
        # pads enter pad_rects/pad_bbox/attraction and desynchronise the
        # n_copper cap-detection test in _Repair.__init__ -- a placement change
        # dressed as a clearance fix. Building the floors against THIS rule
        # makes the alignment true by construction.
        self.pad_floors = []
        self.max_floor = 0.0
        # Per-pad COPPER LAYER sets, index-aligned like self.pads. This is
        # what scopes a cap pad to the tracks it can actually touch (#731):
        # check_drc grades a pad-segment pair only on their SHARED copper, and
        # the netclass term is layer-blind, so without this set the pass both
        # charged phantom inner-layer pairs and missed the real B.Cu/inner
        # pairs of a through-hole pad.
        #
        # Built from the BOARD STACKUP alone, never from the clearance model
        # (#731). Which copper a pad occupies is a fact about the board, not
        # about what it declares -- and PadClearanceModel is inert unless a
        # netclass, a .kicad_dru rule or a pad override exists. Gated on the
        # model, as pad_floors legitimately is, this would be inert on exactly
        # the boards carrying the most phantom: rp2350 declares none of the
        # three, is 6 copper layers, and 2342 of its 4468 pruned pad x track
        # pairs are off-layer, carrying 4.6685mm of phantom graze against
        # 0.0142mm of real. (Both figures count same-net pairs; excluding them
        # it is 2238 of 4331. Mixing the two conventions is easy and wrong.) `board_copper` falls back to the model's copy only
        # so a caller passing a model and nothing else still resolves layers.
        _cu = list(board_copper if board_copper is not None
                   else (getattr(model, 'board_copper', None) or ()))
        from check_drc import pad_copper_layers
        self.pad_layers = []
        for p in fp.pads:
            if not any(str(l).endswith('.Cu') for l in p.layers):
                continue  # paste/mask-only aperture, not copper
            off_x = p.global_x - fp.x
            off_y = p.global_y - fp.y
            half_x, half_y = pad_half_extents(p)
            self.pads.append((off_x, off_y, half_x, half_y, p.net_id))
            # The GEOMETRY filter above stays loose on purpose (see the
            # comment), but a FLOOR must not: PadClearanceModel.pad_floor
            # reads local_clearance unconditionally, and an np_thru_hole
            # pad lists *.Cu while carrying no copper at all. Charging its
            # override would move a cap to clear copper that does not
            # exist -- and would contradict the model's own inertness
            # rule, which refuses to ACTIVATE for an NPTH-only override
            # (legality._pad_carries_copper, the watchy measurement).
            # A pad that carries no copper is graded FLAT, whole stop --
            # not merely stripped of its own override. check_drc does not
            # grade such a pad at all, so letting the PARTNER's netclass
            # through pair() would charge a keep-out that does not exist,
            # which is the same defect as the off-layer phantom #731 is
            # about. The EMPTY layer set is the marker the eff builders
            # key on; a real copper pad always resolves at least one layer.
            carries = _pad_carries_copper(p)
            # Appended ALWAYS, model or not (#731), and unconditionally
            # within the loop so the index alignment the loose geometry
            # filter buys is preserved.
            self.pad_layers.append(
                frozenset(pad_copper_layers(p, _cu)) if carries else frozenset())
            if model is not None:
                fl = model.pad_floor(p) if carries else PadFloor(0.0, 0.0, None)
                self.pad_floors.append(fl)
                mf = model.max_floor(fl)
                if mf > self.max_floor:
                    self.max_floor = mf
        self.local_bounds = courtyard_local
        # Rotation-only geometry is reused across every candidate position
        # (millions of times on a dense board), so memoize it per angle and
        # just translate per call. Keyed by rounded rotation.
        self._rect_cache: Dict[float, Tuple[float, float, float, float]] = {}
        self._pad_cache: Dict[float, list] = {}
        self._pad_bbox_cache: Dict[float, Tuple[float, float, float, float]] = {}
        # One-slot POSE memos: cost() and the shortfall helpers each rebuild
        # the same translated geometry for the same candidate 4-6 times in a
        # row (5.7M pad_rects calls on mez_rx); remember the last pose.
        self._pr_key = None
        self._pr_out: list = []
        self._rc_key = None
        self._rc_out: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    def rect(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        pose = (x, y, rot)
        if pose == self._rc_key:
            return self._rc_out
        # local_bounds is the footprint-LOCAL courtyard; rotate by the
        # absolute placement angle (matching how static obstacle rects are
        # built), not the delta from seed. The rotation is position-independent
        # -> cache it and translate.
        key = round(rot, 3)
        b = self._rect_cache.get(key)
        if b is None:
            b = _rotate_local_bounds(*self.local_bounds, rot)
            self._rect_cache[key] = b
        out = (x + b[0], y + b[1], x + b[2], y + b[3])
        self._rc_key, self._rc_out = pose, out
        return out

    def _pad_cache_for(self, rot):
        key = round((rot - self.seed_rot) % 360, 3)
        cache = self._pad_cache.get(key)
        if cache is None:
            delta = (rot - self.seed_rot) % 360
            rad = math.radians(-delta)
            c, s = math.cos(rad), math.sin(rad)
            swap = round(delta) % 180 == 90
            cache = []
            for off_x, off_y, hx, hy, net in self.pads:
                ox = off_x * c - off_y * s
                oy = off_x * s + off_y * c
                HX, HY = (hy, hx) if swap else (hx, hy)
                cache.append((ox, oy, HX, HY, net))
            self._pad_cache[key] = cache
            if cache:
                self._pad_bbox_cache[key] = (
                    min(ox - HX for ox, oy, HX, HY, net in cache),
                    min(oy - HY for ox, oy, HX, HY, net in cache),
                    max(ox + HX for ox, oy, HX, HY, net in cache),
                    max(oy + HY for ox, oy, HX, HY, net in cache))
            else:
                self._pad_bbox_cache[key] = (0.0, 0.0, 0.0, 0.0)
        return key, cache

    def pad_rects(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        pose = (x, y, rot)
        if pose == self._pr_key:
            return self._pr_out
        _key, cache = self._pad_cache_for(rot)
        out = []
        for ox, oy, HX, HY, net in cache:
            cx, cy = x + ox, y + oy
            out.append((cx - HX, cy - HY, cx + HX, cy + HY, net))
        self._pr_key, self._pr_out = pose, out
        return out

    def pad_bbox(self, x=None, y=None, rot=None):
        """Union bbox of pad_rects at a pose -- a containment-conservative
        prescreen: any pad-pair gap is >= the bbox-pair gap, so two caps whose
        pad bboxes are >= clearance apart have EXACTLY zero pad shortfall."""
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        key, _cache = self._pad_cache_for(rot)
        bb = self._pad_bbox_cache[key]
        return (x + bb[0], y + bb[1], x + bb[2], y + bb[3])


def _candidate_positions(cap, max_disp, step, grid_step):
    """Candidate cap centres, snapped on the OFFSET rather than the position.

    Same #708 correction as `quench._candidate_positions`: snapping
    `cap.seed_x + ix*step` to a lattice through board ORIGIN discards the
    seed's residue and walks the part off whatever pitch the board was laid out
    on, for nothing -- at every step this tool ships, the absolute snap removed
    zero candidates.

    Unlike quench, this site keeps the `grid_step` RASTER rather than the
    board's inferred lattice, and that asymmetry is deliberate and measured.
    This search IS the clearance repair: its whole job is to find the
    sub-millimetre pose that clears a foreign via. At `step=0.2` the candidate
    count is 81 on the raster, 29 on a 0.3175mm lattice and 9 on 0.635 -- so a
    coarse lattice would starve exactly the search that must not be starved,
    and the #313 nudge downstream of it relocates VIAS inside a 0.6mm budget --
    a fraction of a millimetre, which a 1.27 lattice could not express at all.

    The radius test now runs AFTER the snap, so `max_disp` is an exact cap. The
    `grid_step*sqrt(2)/2` overshoot the via- and track-prune margins budget for
    (see `_prune_vias` and the track channel note) is therefore gone rather
    than merely bounded; both margins gain that much slack.
    """
    seen = set()
    out = []
    n = int(max_disp / step)
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            dx = snap_to_grid(ix * step, grid_step)
            dy = snap_to_grid(iy * step, grid_step)
            if math.hypot(dx, dy) > max_disp + 1e-9:
                continue
            cx = cap.seed_x + dx
            cy = cap.seed_y + dy
            key = (round(cx, 4), round(cy, 4))
            if key not in seen:
                seen.add(key)
                out.append((cx, cy))
    return out


class _Repair:
    def __init__(self, pcb_data: PCBData, pcb_file: str,
                 clearance: float, grid_step: float,
                 board_edge_clearance: float, near_margin: float,
                 capture_radius: float, default_via_size: float,
                 cap_prefix: str, extra_locked: Set[str],
                 max_displacement_cap: float = 3.0,
                 netclass_ceiling: Optional[float] = None):
        bounds = pcb_data.board_info.board_bounds
        if bounds is None:
            raise ValueError("No board boundary (Edge.Cuts) found")
        self.board = bounds
        # Copper-to-EDGE, not a different-net pair requirement, so #725 leaves
        # it alone: it is KiCad's `min_copper_edge_clearance`, a separate rule
        # a netclass cannot express. #733 closed the three gaps this comment
        # used to list as filed: `board_edge_clearance` now reaches every front
        # end through resolve_cap_edge_clearance (the GUI passed nothing and got
        # the signature default; the animator had its own copy), and
        # nudge_vias_for_unresolved reads `self.edge_margin` below instead of
        # gating its own emitted copper at bare `clearance`.
        margin = max(clearance, board_edge_clearance)
        self.usable = (bounds[0] + margin, bounds[1] + margin,
                       bounds[2] - margin, bounds[3] - margin)
        # Real board outline / cutouts (#370 B2): `usable` is a bbox inset,
        # blind to interior cutouts and non-rectangular outlines, so a cap
        # could be nudged over a switch window or past a curved edge. When the
        # outline is not exactly the bbox (or cutouts exist), candidate rects
        # are additionally gated against the true Edge.Cuts rings in
        # _blocked_geom. Per-cap laziness: only caps whose reachable disk can
        # touch a ring pay for the exact test. The gate itself now lives in
        # placement/legality.py, shared with quench (#456 item 2).
        self.edge_gate = BoardOutlineGate(pcb_data.board_info, margin)
        # (the board-edge requirement is read back through the `edge_margin`
        # property below, which delegates to the gate rather than keeping a
        # second copy of `margin` -- see its docstring for why, #733)
        # ref -> the milled rings this mover's own pads sit inside (#628), the
        # same exemption quench takes. Needed HERE and not only at the margin-0
        # gates because this one runs at max(clearance, board_edge_clearance)
        # and reaches the swallow probe through rect_blocked, whose ring term is
        # BOOLEAN -- it returns True regardless of margin, so the "a margin-0
        # gate is numerically inert" argument that covers lock_advisor /
        # placement_state / seeder does not reach it. Seed-pose, cached, never
        # invalidated, exactly like quench's.
        self._owned_rings_cache: Dict[str, frozenset] = {}
        self._edge_active = self.edge_gate.active
        self._max_disp_cap = max_displacement_cap
        self.clearance = clearance
        # #725: `clearance` is ONE flat scalar, but KiCad's different-net
        # requirement for a pair is max(clearance, netclass a, netclass b),
        # then REPLACED by any .kicad_dru rule on the layers the two share,
        # then raised by either item's own pad `(clearance ...)` override.
        # This pass is the second channel of #697's defect: priced flat, a pair
        # separated by more than `clearance` but less than its real requirement
        # is never charged, so the repair reports nothing to fix on geometry
        # check_drc flags. PadClearanceModel resolves it exactly as check_drc
        # does and is STRICTLY INERT on a board declaring none of the three.
        # `notes` must be read BEFORE the active-drop: a FAILED read (an
        # unreadable sibling) is exactly what makes the model look inert.
        # #768: `netclass_ceiling` is the --clearance the CALLER will clamp the
        # output project's classes down to, or None when it will clamp nothing.
        # It caps the netclass tier and only that tier -- the .kicad_dru rules
        # and the pad overrides survive it, because the writeback touches
        # neither and KiCad will go on enforcing both.
        _model = PadClearanceModel.for_board(pcb_data, clearance, pcb_file,
                                             ceiling=netclass_ceiling)
        self.clearance_notes = list(_model.notes)
        self._floors = _model if _model.active else None
        # #735: the track-scoped .kicad_dru channel, on its OWN handle and
        # deliberately not folded into `self._floors`. That one is the switch
        # for every requirement in this class, and nine consumers change code
        # path when it flips; a track rule binds track-vs-track and nothing
        # else, which happens at exactly one site (`connector_clear`'s s2 arm,
        # via `track_required`). A board whose only declaration is a track rule
        # therefore keeps `self._floors is None` and every existing number.
        self._track = _model if _model.track_rules else None
        # Pad-layer scope for the non-pad items below. A via spans copper, so
        # it is scoped to ALL copper: check_drc's pad-via path passes
        # layers_b=None meaning "scope to the pad's own layers", and
        # intersecting with all-copper reproduces that exactly (blind vias
        # included). Never None here -- PadClearanceModel.pair does
        # `fb.layers or ()`, and an EMPTY shared set discards every dru rule.
        # #731: resolved through the SAME three-level fallback check_drc's
        # run_drc uses (check_drc.py: filter to .Cu -> the layers segments
        # actually sit on -> ['F.Cu','B.Cu']), because this set is what decides
        # whether layer scoping is on at all. A board whose `(layers ...)`
        # stanza yields no copper -- kicad_parser's regex has produced that in
        # the field, see its `In1(GND).Cu` note -- would otherwise switch
        # scoping off and fall back to the 'F'/'B' side collapse. That is NOT
        # the safe direction it looks like: check_drc still expands a `*.Cu`
        # pad over the segment-derived layers and grades it, while the side
        # collapse DROPS every B.Cu and inner track a through-hole cap pad
        # shares copper with -- an UNDER-block, and the very mirror bug #731
        # exists to fix. Measured on rp2350 with the copper list cleared and
        # cap pads made through-hole: 280 pairs check_drc grades that the
        # side-collapse fallback ignores, byte-identical to the pre-#731 count.
        # sorted(), not list(set(...)): string hashing is randomized per
        # process and this list feeds pad-layer expansion (the same reason
        # check_drc sorts it).
        _cu = [l for l in (pcb_data.board_info.copper_layers or [])
               if str(l).endswith('.Cu')]
        if not _cu:
            _cu = sorted({s.layer for s in pcb_data.segments
                          if (s.layer or '').endswith('.Cu')})
        if not _cu:
            _cu = ['F.Cu', 'B.Cu']
        self._all_cu = frozenset(_cu)
        # The ORDERED list is what pad-layer expansion consumes; `_all_cu` is
        # the membership test. Keeping both means `*.Cu` never resolves through
        # a set whose iteration order varies run to run.
        self._all_cu_ordered = list(_cu)
        self._via_floor: Dict[int, PadFloor] = {}
        self._seg_floor: Dict[Tuple[int, str], PadFloor] = {}
        # #617: KiCad's copper-to-hole rule is the BOARD's `min_hole_clearance`
        # whenever it declares one above the 0.20 NPTH fab floor -- the same
        # value check_drc grades at. The NPTH keep-out rects below were grown
        # to the flat floor only, so on such a board a cap pad could be parked
        # in the declared band and read clean here while the checker flagged
        # it. Raise-only and cached per board path, so a board that declares
        # nothing keeps byte-identical keep-outs. `config` is None because the
        # placement engine has no GridRouteConfig anywhere in this call chain
        # (repair_fanout_clearance takes scalars); the board read is driven by
        # pcb_data.source_path, so the declared floor arrives regardless.
        #
        # #730: THIS PASS CARRIES TWO NPTH FLOORS AND THAT IS DELIBERATE.
        # `nudge_vias_for_unresolved` resolves its own,
        # `npth_clr = max(clearance, NPTH_TO_TRACK_CLEARANCE)`, which omits the
        # board read this one performs. A reader finding them apart will want
        # to unify them; #617 already decided not to, and the decision is
        # measured. Its harness (`_via_mover_stays_flat` in
        # tests/test_617_placement_fanout_hole_clearance.py) stages a board
        # declaring 0.25, and the mover's only validated connector landing sits
        # 0.2200mm off the hole wall -- so ANY floor above 0.2200 refuses it.
        # Re-measured on this tree while closing #730: flat -> "1 move(s),
        # 1 connector(s); copper-to-hole gap 0.2200mm"; board-aware -> "no
        # clear spot", 0 and 0, and the #130 pad-via graze the pass exists to
        # remove simply persists.
        #
        # The asymmetry has a shape, not just a history. THIS site is a COST
        # MODEL -- the whole cap search descends it, so raising it makes the
        # pass SEE a shortfall and MOVE the cap out of the band. That site is
        # an ALL-OR-NOTHING GATE on a 0.6mm search, where raising it does not
        # find a better spot, it returns "no clear spot". Same rule, two
        # positions, and only one of them can afford the strictness. The file
        # already carries this shape once, for the via-vs-connector floor
        # inside that function; this is the second instance and it is now
        # written down at BOTH ends rather than only at the far one.
        #
        # STILL TRUE AFTER #756, and #756 is worth reading as the case that
        # tested it rather than as an exception to it. That change wanted the
        # via/via DRILL floor to follow the board's `min_hole_to_hole`, which
        # is a raise on this very function -- and a one-rung raise was measured
        # and REJECTED for exactly the reason stated above. A review swept 8673
        # configurations of the pass's own rig shape: 625 lost the repair at the
        # shipped 0.6mm budget, 13 of them where the abandoned landing was one
        # `check_drc` grades CLEAN.
        #
        # What shipped instead is a two-rung LADDER -- prefer the declared
        # floor, fall back to the fab floor -- so that site simply STOPPED
        # being all-or-nothing for that floor. The rule did not need bending;
        # the site changed shape. Anything that collapses the ladder back to
        # one rung re-enters this paragraph.
        #
        # An earlier draft of this note claimed the via/via drill gate was the
        # only floor in that function sitting BELOW its own grader. A review
        # refuted it TWICE over, and both refutations are recorded here rather
        # than the sentence simply being dropped:
        #
        #   * the CONNECTOR's copper-to-hole gate is flat while `check_drc`
        #     `_pin_up`s `hole_clearance` from `min_hole_clearance` (:3697),
        #     so on #617's own declaring rig the connector this pass emits
        #     grades as a 0.030mm violation;
        #   * and the VIA's own copper-to-NPTH gate is not at parity either,
        #     in the band #730 already discloses. Measured on a mid-chain
        #     project (rules 0.15, fab_floor_origin 0.35, an NPTH pad at
        #     lc 0.30): the nudge charges 0.1000 where check_drc's via arm
        #     charges 0.3000 -- the nudge UNDER-charges by 0.2000.
        #
        # Both gaps are measured, disclosed choices of #617 and #730 and are
        # still open. Neither is a counterexample to #756, and #756 is not a
        # licence to close either by raising -- the ladder is what makes the
        # drill floor affordable, and neither of those two has one.
        #
        # One correction to how the divergence was first reported (#730's
        # thread): the two do NOT differ in both directions.
        # `max(clearance, NPTH_TO_TRACK_CLEARANCE)` can never exceed
        # `max(clearance, NPTH_TO_TRACK_CLEARANCE, declared)`, which is what
        # this floor resolves to at its use site once the graded `clearance`
        # is added back. The nudger is the lower of the two, or equal, and
        # never the higher.
        from obstacle_map import resolve_hole_clearance
        self.npth_floor = max(defaults.NPTH_TO_TRACK_CLEARANCE,
                              resolve_hole_clearance(pcb_data, None))
        self.grid_step = grid_step
        self.capture_radius = capture_radius
        # cap_prefix may list several reference prefixes (e.g. "C,R" = caps and
        # resistors); str.startswith() accepts the tuple directly.
        self._cap_prefixes = tuple(p.strip() for p in str(cap_prefix).split(',')
                                   if p.strip()) or ('C',)

        courtyards = extract_courtyard_bboxes(pcb_file)
        locked = set(extract_locked_refs(pcb_file)) | extra_locked
        self.locked_refs = locked

        # --- avoidance: the REAL fanout vias (after fanout) ---
        # Each (x, y, net, keepout) where keepout = via_radius + clearance; a
        # cap pad must clear vias of a DIFFERENT net by this. Through-vias, so
        # they apply to caps on either board side.
        # #725: `keepout` is the OVER-REACH -- radius + max(clearance, this
        # via's own upper-bound floor). It feeds the prunes, the locked-part
        # warning and the animator's drawn disk, all of which may over-reach.
        # The EXACT per-(cap pad, via) value is resolved in the eff lists
        # below, never from this slot. Byte-identical (radius + clearance) when
        # the model is inert. The 4-tuple shape is pinned: tests assign
        # st.vias wholesale and animate_fanout_clearance.py unpacks it.
        self.vias: List[Tuple[float, float, int, float]] = []
        # The RADIUS, keyed on the tuple's identity. _via_effs has to strip the
        # over-reach back off element 3, and deriving the radius arithmetically
        # would silently mis-price a tuple a test assigned wholesale with a
        # different keep-out convention. Absent from this map -> graded at the
        # tuple's own keep-out slot verbatim, i.e. exactly as injected.
        #
        # The VALUE holds the tuple as well as the radius, so the map itself
        # keeps every registered tuple alive. Unlike the pad/segment floor maps
        # -- which are filled only in __init__, where self.foreign_pads and
        # self.segments hold their tuples -- this one gains entries later, when
        # relocate_vias files a moved barrel (#747; before that, the nudger did
        # it inline). A tuple that died while its id entry lived would hand a
        # recycled id ANOTHER via's radius, silently. That guarantee is also
        # what lets _register_via file WITHOUT appending, which the track
        # builder cannot do -- see its docstring.
        #
        # #732: the fallback is STORED rather than consumed inline and dropped,
        # because nudge_vias_for_unresolved needs the same answer and had no way
        # to reach it -- see via_radius() for what that cost.
        self._default_via_size = default_via_size
        self._via_radius_by_id: Dict[int, Tuple[tuple, float]] = {}
        # #779: where a RELOCATED barrel sat at the seed, for the seed
        # half of required_rows. Keyed by the id of the tuple
        # relocate_vias built, and holding that tuple as its first element
        # for the same reason the radius map does: a bare id can be
        # recycled by the very next tuple this object makes, and the entry
        # would then describe someone else's barrel. Empty on every run
        # that moves nothing, which is every tracked board at the shipped
        # defaults.
        self._via_seed_xy: Dict[int, Tuple[tuple, float, float]] = {}
        # #747: through _register_via, which is also what a via RELOCATED
        # after construction goes through -- so a barrel the pass moves cannot
        # be shaped, priced or filed differently from one present here.
        for v in pcb_data.vias:
            self.vias.append(self._register_via(v.x, v.y, v.net_id,
                                                radius=self.via_radius(v)))

        # --- avoidance: foreign-net tracks the cap's pads can actually TOUCH ---
        # Fanout escapes can land on the bottom (cap) side; attraction could
        # then pull a cap onto an escape track -> a PAD-SEGMENT violation.
        # #725: element 5 is the OVER-REACH, like self.vias[3] above, and the
        # segment's floor is keyed by its REAL layer.
        # #731: so is element 6, which used to be the 'F'/'B' board SIDE. That
        # collapse filed every non-B layer under 'F', so an In1.Cu track was
        # compared against an F-side cap pad that can never touch it -- 928
        # such pairs on orangecrab at --clearance 0.1, 0.2464mm of phantom
        # graze at the seed, and the ENTIRE bill of R17/R18/R5. check_drc
        # grades no such pair at all (check_pad_segment_overlap returns early
        # when seg.layer is not in the pad's expanded copper layers). It was
        # wrong in the other direction too: a THROUGH-HOLE cap pad's copper
        # spans every layer, and the side test DROPPED the B.Cu and inner
        # tracks it really does share copper with.
        #
        # The tuple WIDTH is unchanged (TestShapeContract pins it) and the
        # side is now derived, in one place (_seg_side), only where a cap's
        # real pad layers are unavailable.
        self.segments: List[Tuple[float, float, float, float, int, float, str]] = []
        # The floor cannot be re-derived from the tuple, so keep it on the
        # tuple's identity. The tuples live for this object's lifetime, so the
        # id is stable; a tuple a test injects is simply absent and grades
        # flat. (There is no companion layer map any more: the layer IS the
        # tuple's element 6, so there is exactly one answer to "what layer is
        # this track on". The old map was additionally gated on an ACTIVE
        # clearance model, which is what made the layer truth vanish on the
        # boards carrying the most phantom -- see _cap_seg_scope.)
        self._seg_floor_by_id: Dict[int, PadFloor] = {}
        # #736: through _register_segment, which is also what the connector
        # copper this pass DRAWS goes through -- so a track filed later cannot
        # be priced or scoped differently from one present at construction.
        for s in pcb_data.segments:
            self._register_segment(s.start_x, s.start_y, s.end_x, s.end_y,
                                   s.net_id, s.width, s.layer)

        # --- attraction: BGA balls grouped by net ---
        # A cap pad is pulled toward the nearest ball of its own net, so a via
        # dropped later at that ball also lands on the cap pad. Restricted to
        # real BGA footprints (detect_package_type == 'BGA').
        self.attract: Dict[int, List[Tuple[float, float]]] = {}
        bga_bboxes: List[Tuple[float, float, float, float]] = []
        self.bga_refs: List[str] = []
        for fp in find_components_by_type(pcb_data, 'BGA'):
            self.bga_refs.append(fp.reference)
            grid = analyze_bga_grid(fp)
            if grid is not None:
                bga_bboxes.append((grid.min_x, grid.min_y, grid.max_x, grid.max_y))
            else:
                lb = compute_footprint_bbox_local(fp)
                b = _rotate_local_bounds(*lb, fp.rotation)
                bga_bboxes.append((fp.x + b[0], fp.y + b[1],
                                   fp.x + b[2], fp.y + b[3]))
            for p in fp.pads:
                if p.net_id > 0:
                    self.attract.setdefault(p.net_id, []).append(
                        (p.global_x, p.global_y))
        # Kept for visualization (animate_fanout_clearance.py): the BGA ball-field
        # bounding boxes that define the region of interest.
        self.bga_bboxes = bga_bboxes

        # --- static obstacle rects (locked parts incl. BGAs) ---
        # and movable caps near a BGA. Courtyard collisions are checked
        # per board side only: a back-side decoupling cap legitimately sits
        # under a top-side BGA (they overlap in XY but not in copper). Via
        # disks are through-vias, so they apply to caps on either side.
        # --- avoidance: foreign-net COMPONENT pads (#235) ---
        # The cap optimizer must not slide a cap pad onto a neighbouring
        # component's pad of a different net -> a PAD-PAD short. Each entry is
        # an axis-aligned pad rect plus its net and side ('F'/'B', or None for
        # a through-hole pad that blocks both sides). Same-net pads are fine.
        self.foreign_pads: List[
            Tuple[float, float, float, float, int, Optional[str]]] = []
        # #725: index-aligned with self.foreign_pads (the 6-tuple shape is
        # pinned -- tests inject into cap_foreign_pads directly). An NPTH
        # entry gets floor None / mf 0.0: its rect is already inflated to the
        # hole floor, and the copper-to-HOLE rule is net-independent, so the
        # new machinery must never raise it via a neighbouring pad's netclass.
        # #730: the rect inflation below DOES now carry the hole pad's OWN
        # `local_clearance`, which is what makes floor None correct rather than
        # merely convenient. check_drc grades this rule at
        # `max(npth_clr, lc)` (check_drc.py:2733, #505) and `lc` on a hole pad
        # is net-independent exactly like the floor beside it -- so it belongs
        # in the GEOMETRY, with the fab floor, and not in the per-pair floor
        # machinery, which resolves netclasses. Read straight off the pad, NOT
        # through PadClearanceModel, and "is the model active?" is NOT a usable
        # proxy either. The model refuses to activate for an NPTH-ONLY override
        # (legality.py:1450-1454 gates has_overrides on _pad_carries_copper) --
        # but on the one board that matters it is active anyway and still gives
        # the hole nothing: measured on ulx3s, BAT1's two SMD pads carry
        # lc 0.700 so `model.active` is True, while AUDIO1's two NPTH holes get
        # no floor at all.
        #
        # And the REASON is not the one it looks like, which is worth stating
        # because the plausible version is wrong. `pad_floor` is NOT gated on
        # copper: called on that NPTH pad it hands back
        # `PadFloor(ncl=0.0, lc=0.4, layers=None)` -- measured. What drops the
        # override is the filing below: an NPTH entry gets floor `None`, and
        # `_fp_floor_by_id` maps only tuples that carry one, so the tuple
        # resolves through `_pair_or_flat(fa, None)` to the flat scalar and
        # `pad_floor` is never consulted for it at all. The read has to be off
        # the pad.
        # NB: self.foreign_pads (and self.segments) are read nowhere after
        # __init__ -- the pruned per-cap lists are what the sweep uses. They
        # must NOT be deleted as dead: the id-keyed floor maps below depend on
        # those tuples staying alive, and a recycled id would return ANOTHER
        # item's floor, silently, with no error.
        # #736: self.segments does GAIN entries after __init__ (the connector
        # copper this pass draws, filed by register_new_segments). It is
        # APPENDED to, never rebuilt -- precisely so the aliveness this note is
        # about survives the gain. self.foreign_pads is unchanged.
        self.foreign_pad_floors: List[Optional[PadFloor]] = []
        self.foreign_pad_mf: List[float] = []
        self.caps: Dict[str, _Cap] = {}
        self.static_rects: List[Tuple[Tuple[float, float, float, float], str]] = []
        for ref, fp in pcb_data.footprints.items():
            if not fp.pads:
                continue
            lb = courtyards.get(ref) or compute_footprint_bbox_local(fp)
            # Count COPPER pads only: paste-only apertures (split-paste 0201
            # footprints) would otherwise push a 2-terminal cap past the 2-pad
            # test and wrongly exclude it from placement (#130).
            n_copper = sum(1 for p in fp.pads
                           if any(str(l).endswith('.Cu') for l in p.layers))
            is_cap = (ref.startswith(self._cap_prefixes) and n_copper <= 2
                      and ref not in locked)
            if is_cap:
                cap = _Cap(fp, lb, self._floors, self._all_cu_ordered)
                if self._near_any(cap.rect(), bga_bboxes, near_margin):
                    self.caps[ref] = cap
                    continue
            # everything else is a static obstacle
            side = 'B' if (fp.layer or '').startswith('B') else 'F'
            b = _rotate_local_bounds(*lb, fp.rotation)
            self.static_rects.append(((fp.x + b[0], fp.y + b[1],
                                       fp.x + b[2], fp.y + b[3]), side))
            # record this part's copper pads as foreign-pad keep-outs
            from check_drc import _pad_has_no_copper
            from kicad_parser import pad_drill_circles
            for p in fp.pads:
                copper = [l for l in p.layers if str(l).endswith('.Cu')]
                if _pad_has_no_copper(p):
                    # Copper-less drilled pad (NPTH mounting hole -- an
                    # np_thru_hole lists *.Cu but carries NO ring, #370 B2):
                    # the DRILL still removes any copper closer than the
                    # NPTH-to-track floor, so a cap pad slid over it is a real
                    # fab violation regardless of net. Blocks BOTH sides
                    # (through hole); graded by inflating the rect so that
                    # rect + the flat `clearance` charged downstream lands on
                    # check_drc's own requirement; net -1 never matches a cap
                    # pad's net (even a net-tied mounting hole is not
                    # connectable copper, #328).
                    #
                    # #730: `max(self.npth_floor, lc)` is check_drc's
                    # `max(npth_clr, local_clearance)` term for term.
                    # `max(npth_floor, clearance)` -- what this resolved to
                    # before -- is ALREADY exactly check_drc.py:2714's
                    # `npth_clr = max(clearance, NPTH_TO_TRACK_CLEARANCE,
                    # hole_clearance)`, because npth_floor carries the fab
                    # floor and the board's declared min_hole_clearance and
                    # the `clearance` comes back in downstream. So `lc` was
                    # the only missing term, and this is one term added, not
                    # a re-derivation. Measured on kicad_files/ulx3s.kicad_pcb,
                    # AUDIO1's two 1.7mm NPTH holes at lc=0.400: 1.050mm
                    # modelled vs 1.250mm required at --clearance 0.10, and
                    # 1.100 vs 1.250 at 0.25. RAISE-ONLY -- `max` with a
                    # non-negative term -- so a board with no NPTH override,
                    # which is 21 of the 22 tracked ones, is byte-identical.
                    # (watchy's 8 NPTH overrides are all 0.100, below the 0.20
                    # fab floor, so that board stays inert and is this rule's
                    # negative control rather than a second example.)
                    if (p.drill or 0) > 0:
                        lc = getattr(p, 'local_clearance', 0.0) or 0.0
                        grow = max(0.0, max(self.npth_floor, lc) - clearance)
                        for hx, hy, hd in pad_drill_circles(p):
                            hr = hd / 2.0 + grow
                            self.foreign_pads.append(
                                (hx - hr, hy - hr, hx + hr, hy + hr, -1, None))
                            self.foreign_pad_floors.append(None)
                            self.foreign_pad_mf.append(0.0)
                    continue
                if not copper:
                    continue  # paste/mask-only aperture
                through = (p.drill or 0) > 0
                pside = None if through else (
                    'B' if any(str(l).startswith('B') for l in copper) else 'F')
                half_x, half_y = pad_half_extents(p)
                self.foreign_pads.append(
                    (p.global_x - half_x, p.global_y - half_y,
                     p.global_x + half_x, p.global_y + half_y,
                     p.net_id, pside))
                _fl = None if self._floors is None else self._floors.pad_floor(p)
                self.foreign_pad_floors.append(_fl)
                self.foreign_pad_mf.append(
                    0.0 if _fl is None else self._floors.max_floor(_fl))

        # Per-cap spatially-pruned neighbour lists (perf). A cap moves at most
        # max_displacement_cap from its seed, so anything whose seed gap already
        # exceeds that (plus the relevant spans / clearance) can NEVER constrain
        # it -- excluding it is exact, not an approximation. This turns the
        # per-candidate hard_blocked / penalty loops from O(all parts) into
        # O(handful), the dominant cost on dense boards (#213 profiling).
        # #725: same identity-keyed map for foreign pads -- a pad's floor
        # depends on its own `(clearance ...)` and its copper layers, neither of
        # which the 6-tuple carries.
        self._fp_floor_by_id: Dict[int, PadFloor] = {}
        if self._floors is not None:
            for _t, _f in zip(self.foreign_pads, self.foreign_pad_floors):
                if _f is not None:
                    self._fp_floor_by_id[id(_t)] = _f
        # Per-(cap, neighbour) REQUIRED-clearance memos. model.pair() is
        # pose-independent, so it is resolved ONCE per (cap, neighbour) pair
        # and reused for every candidate pose. The memos are filled lazily, so
        # the first fill for a pair does happen inside cost() -- measured on a
        # 0.5-class board: 572 of 62,565 pair_with_source calls, against
        # 251,477 cost() calls. Each memo is (source_list, rows) and is
        # rebuilt when the source list identity changes -- which is what
        # lets refresh_cap_vias hand every cap a NEW list per key without
        # clearing a memo (#775), and makes a test that injects into
        # cap_foreign_pads grade flat rather than mis-index.
        self._cap_pad_eff: Dict[str, Tuple[list, list]] = {}
        self._cap_seg_eff: Dict[str, Tuple[list, list]] = {}
        self._cap_via_eff: Dict[str, Tuple[list, list]] = {}
        self._cap_pair_eff: Dict[Tuple[str, str], list] = {}

        cap_geom: Dict[str, Tuple[float, float, float, Tuple]] = {}
        for ref, cap in self.caps.items():
            r = cap.rect()
            cx, cy = (r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0
            span = math.hypot(r[2] - cx, r[3] - cy)
            cap_geom[ref] = (cx, cy, span, r)
        # #736: KEPT, not consumed and dropped with the frame. Every prune
        # reach below is anchored on the cap's SEED pose -- which is what this
        # dict holds, because a cap's x/y/rot still equal its seed_* here --
        # and register_new_segments re-runs that same prune for copper that
        # appears AFTER construction, by which time cap.rect() is no longer the
        # seed. Stored rather than re-derived there for the reason _seg_shares
        # and via_required exist: a second derivation is a second thing to keep
        # in step.
        self._cap_geom = cap_geom

        self.cap_foreign_pads: Dict[str, List[
            Tuple[float, float, float, float, int, Optional[str]]]] = {}
        # static obstacle rects (with their global index, same side) in reach
        self.cap_static: Dict[str, List[Tuple[int, Tuple]]] = {}
        # other movable caps (refs, same side) that could ever touch this one
        self.cap_caps: Dict[str, List[str]] = {}
        # foreign-net tracks this cap's pads SHARE COPPER WITH, in reach (#731)
        self.cap_segs: Dict[str, List[Tuple]] = {}
        # ...and how many LEADING entries of each are seed-era (#736). Copper
        # register_new_segments appends is real on the final board and a
        # fiction at the seed, so the two graded poses see different lists.
        self._seed_seg_n: Dict[str, int] = {}
        # through-vias in reach (each (vx, vy, net, keepout))
        self.cap_vias: Dict[str, List[Tuple[float, float, int, float]]] = {}
        # #725: every reach below is raised by THE TWO ITEMS' OWN maxima, never
        # by a board-wide maximum -- these lists gate per-candidate loops, so a
        # board-wide bound would slow every cap for the sake of one keep-clear
        # pad. `cap.max_floor` and `foreign_pad_mf` are 0.0 on an inert board,
        # so `max(clearance, 0.0, 0.0) == clearance` and the reaches are
        # byte-identical. Left un-raised on purpose: cap_static below, which
        # gates _overlap (courtyards, not copper).
        cap_refs = list(self.caps)
        for ref, cap in self.caps.items():
            ccx, ccy, span, crect = cap_geom[ref]
            reach = max_displacement_cap + span
            cap_mf = cap.max_floor
            near_pads = []
            for j, fp_pad in enumerate(self.foreign_pads):
                px = (fp_pad[0] + fp_pad[2]) / 2.0
                py = (fp_pad[1] + fp_pad[3]) / 2.0
                phalf = math.hypot(fp_pad[2] - px, fp_pad[3] - py)
                req = max(clearance, cap_mf, self.foreign_pad_mf[j])
                if math.hypot(px - ccx, py - ccy) <= reach + req + phalf:
                    near_pads.append(fp_pad)
            self.cap_foreign_pads[ref] = near_pads

            near_static = []
            for idx, (sr, side) in enumerate(self.static_rects):
                if side != cap.side:
                    continue
                if _rect_gap(crect, sr) <= max_displacement_cap + clearance + EPS:
                    near_static.append((idx, sr))
            self.cap_static[ref] = near_static

            near_caps = []
            for oref in cap_refs:
                if oref == ref or self.caps[oref].side != cap.side:
                    continue
                # both caps can move, so the combined reach is 2x the budget
                if (_rect_gap(crect, cap_geom[oref][3])
                        <= 2 * max_displacement_cap
                        + max(clearance, cap_mf, self.caps[oref].max_floor)
                        + EPS):
                    near_caps.append(oref)
            self.cap_caps[ref] = near_caps

            # #736: through _prune_segs, the ONE spelling of this predicate.
            # It carries the comment block that used to live here, including
            # why the reach is anchored on the SEED pose and why the #731 layer
            # union keeps the list exact.
            self.cap_segs[ref] = self._prune_segs(cap, cap_geom[ref],
                                                  self.segments)
            # #736: how much of that list is SEED-ERA copper. required_rows
            # grades the seed pose as well as the final one, and a connector
            # this pass DREW did not exist at the seed -- charging it there
            # invents a row describing a pair no board ever had. Recorded per
            # cap rather than derived from a global count, because the pruned
            # lists have different lengths.
            self._seed_seg_n[ref] = len(self.cap_segs[ref])

            # #775: through _prune_vias, the ONE spelling of this predicate.
            # It carries the comment block that used to live here -- why v[3]
            # is the via's own over-reach, why #725's cap-side slack bounds
            # the pair, and how thin the exactness margin on this channel
            # really is.
            self.cap_vias[ref] = self._prune_vias(cap, cap_geom[ref],
                                                  self.vias)

        # Baseline same-side overlaps at the seed placement. Collisions are
        # scored RELATIVE to these: the repair must not introduce or worsen a
        # courtyard overlap, but it leaves pre-existing tight placements alone
        # (those aren't this issue's concern, and chasing them causes churn).
        self.base_static: Dict[Tuple[str, int], float] = {}
        for ref, cap in self.caps.items():
            seed_rect = cap.rect()
            for idx, (r, side) in enumerate(self.static_rects):
                if side == cap.side:
                    self.base_static[(ref, idx)] = self._overlap(seed_rect, r)
        self.base_cap: Dict[frozenset, float] = {}
        cap_items = list(self.caps.items())
        for i, (ra, ca) in enumerate(cap_items):
            ra_rect = ca.rect()
            for rb, cb in cap_items[i + 1:]:
                if cb.side == ca.side:
                    self.base_cap[frozenset((ra, rb))] = self._overlap(
                        ra_rect, cb.rect())
        # #441: PER-NET seed shortfall {snet: pen}, so the accept gate forbids
        # penetrating a net that was clear at the seed (not just worsening the
        # summed total).
        self.base_seg: Dict[str, Dict[int, float]] = {
            ref: self._seg_shortfalls(ref, cap, cap.x, cap.y, cap.rot)
            for ref, cap in self.caps.items()}
        # Baseline foreign-pad encroachment at the seed (#235): the repair may
        # not introduce or worsen a foreign-net pad-pad overlap, but tolerates
        # one already present at the seed (not this step's concern). Per-net (#441).
        self.base_pad: Dict[str, Dict[int, float]] = {
            ref: self._pad_shortfalls(ref, cap, cap.x, cap.y, cap.rot)
            for ref, cap in self.caps.items()}
        # Baseline foreign-VIA penetration at the seed (#445): via_penalty was
        # only a WEIGHTED cost term, so a move relieving a big track graze
        # could pay for dropping a pad onto an existing via (zynq_ad9364 R2
        # rotated onto the DDR3_VREF via -- a KiCad-confirmed pad-via short).
        # Same per-net hard gate as tracks/pads: never penetrate a via net
        # beyond its seed shortfall.
        self.base_via: Dict[str, Dict[int, float]] = {
            ref: self._via_shortfalls(ref, cap, cap.x, cap.y, cap.rot)
            for ref, cap in self.caps.items()}
        # Baseline mover-vs-mover pad encroachment at the seed (#275). The
        # cap-cap COURTYARD baseline above tolerates pre-existing overlaps,
        # but overlap depth is a poor proxy for pad geometry: two 45-degree
        # parts (fpga_sdram C11/FB1) kept courtyard overlap <= baseline while
        # their different-net pads slid into contact -- a PAD-PAD short that
        # #235's foreign_pads never sees because both parts are movers. Guard
        # at pad level, relative to the seed, over the same pruned pairs.
        self.base_cap_pad: Dict[frozenset, float] = {}
        for ref, cap in self.caps.items():
            seed_pads = cap.pad_rects()
            for oref in self.cap_caps[ref]:
                key = frozenset((ref, oref))
                if key not in self.base_cap_pad:
                    # #725: the baseline must be in the SAME currency as the
                    # candidate it gates. Priced flat while candidates are
                    # priced at the requirement, _worsens_any_net compares two
                    # different units, so the accept gate reads a pose as
                    # worse-than-seed on a pair that did not change.
                    # Measured by reverting exactly this line at HEAD: the
                    # placement count moves in a CLASS-DEPENDENT direction and
                    # not by much (0.4 identical, 0.5 gives 50 vs 48 and one
                    # more unresolved, 1.0 gives 31 vs 34) -- so the symptom is
                    # a quietly wrong answer, not a visible seizure, which is
                    # why the test asserts the CURRENCY and not a count.
                    self.base_cap_pad[key] = _pad_pair_shortfall(
                        seed_pads, self.caps[oref].pad_rects(), self.clearance,
                        self._pair_effs(ref, cap, oref, self.caps[oref]))

    @staticmethod
    def _near_any(rect, bboxes, margin):
        for b in bboxes:
            if _rect_gap(rect, b) <= margin:
                return True
        return False

    def _cap_may_reach_edge(self, ref, cap):
        """Cached prune for the real-outline gate (#370 B2): a cap moves at
        most max_displacement_cap from its seed, so only caps whose reachable
        disk can touch an Edge.Cuts ring ever need the exact ring test."""
        return self.edge_gate.may_reach(
            ref, cap.rect(cap.seed_x, cap.seed_y, cap.seed_rot),
            self._max_disp_cap)

    def _owned_rings(self, ref) -> frozenset:
        """Cached: the milled rings this mover's OWN pads sit inside (#628).

        The cap analogue of QuenchState._owned_rings, and it matters for the
        same reason: a milled contour is reclassified out of board_cutouts
        precisely BECAUSE it encloses >= 2 pad centres, so a two-pad part whose
        pads are what triggered that reclassification lives on its own relief.
        Without the exemption `_rect_edge_blocked` rejects EVERY candidate for
        it -- there is no unfreeze branch here, so the cap simply never moves.

        Pad CENTRES at the SEED pose: `pad_rects` builds each rect as
        centre +/- half-extent, so the bbox midpoint is the centre exactly.
        Seed rather than candidate pose is the anti-gaming choice quench
        documents -- ownership at the candidate pose would let any cap claim a
        ring by moving onto it.
        """
        owned = self._owned_rings_cache.get(ref)
        if owned is None:
            cap = self.caps[ref]
            pts = [((bx0 + bx1) * 0.5, (by0 + by1) * 0.5)
                   for (bx0, by0, bx1, by1, _net) in
                   cap.pad_rects(cap.seed_x, cap.seed_y, cap.seed_rot)]
            owned = self.edge_gate.rings_enclosing(pts) if pts else frozenset()
            self._owned_rings_cache[ref] = owned
        return owned

    def _rect_edge_blocked(self, rect, ref=None):
        """True when a candidate courtyard rect leaves the REAL board outline,
        enters a cutout, or comes within the edge margin of either (#370 B2 --
        the bbox `usable` inset is blind to cutouts / curved outlines).

        `ref` exempts the mover from the swallow rule on the milled rings it
        OWNS (#628); real cutouts are never exempt, by construction in
        `rings_enclosing`. `ref=None` charges every ring, the old behaviour."""
        return self.edge_gate.rect_blocked(
            rect, skip_rings=self._owned_rings(ref) if ref is not None else None)

    # ---- #725: per-pair required clearance -------------------------------
    # A via or a track carries no pad `(clearance ...)` override, so its floor
    # is a pure function of (net, layer scope) and is memoised on that key --
    # never on a list position, because the graded via list is reassigned
    # wholesale by tests and rebuilt by relocate_vias (#747), which would
    # desync any index-aligned parallel list.

    def _via_floor_for(self, net_id):
        """The PadFloor standing in for a via: its net's class floor, no pad
        override, scoped to all copper (see self._all_cu). None when inert."""
        if self._floors is None:
            return None
        f = self._via_floor.get(net_id)
        if f is None:
            f = PadFloor(self._floors.net_floor.get(net_id or 0, 0.0), 0.0,
                         self._all_cu if self._floors.layer_rules else None)
            self._via_floor[net_id] = f
        return f

    def _seg_floor_for(self, net_id, layer):
        """The PadFloor standing in for a track: its net's class floor, no pad
        override, scoped to its OWN layer -- which reproduces check_drc's
        single-layer REPLACE for pad-segment, since with one shared layer
        pads_shared_layer_clearance is all-or-nothing. None when inert."""
        if self._floors is None:
            return None
        key = (net_id, layer)
        f = self._seg_floor.get(key)
        if f is None:
            f = PadFloor(self._floors.net_floor.get(net_id or 0, 0.0), 0.0,
                         frozenset({layer}) if self._floors.layer_rules else None)
            self._seg_floor[key] = f
        return f

    def _item_reach(self, floor):
        """The over-reach for a via/track keep-out: never below the flat
        scalar, raised by that item's own upper bound. Exactly `clearance`
        when the model is inert, so the keep-outs stay byte-identical."""
        if floor is None:
            return self.clearance
        return max(self.clearance, self._floors.max_floor(floor))

    # Public resolvers. nudge_vias_for_unresolved composes its requirements
    # from these rather than re-deriving them, so there is ONE answer per pair
    # kind in this module.

    def required(self, fa, fb):
        """Required clearance between two resolved floors (either may be None
        -- an NPTH keep-out, a test-injected tuple -- which grades flat)."""
        return self._pair_or_flat(fa, fb)

    def pad_floor(self, pad):
        """The floor for a real parser Pad; None when inert."""
        return None if self._floors is None else self._floors.pad_floor(pad)

    def via_floor(self, net_id):
        """The floor standing in for a via of `net_id`; None when inert."""
        return self._via_floor_for(net_id)

    def seg_floor(self, net_id, layer):
        """The floor standing in for a track on `layer`; None when inert."""
        return self._seg_floor_for(net_id, layer)

    def via_required(self, pad_floor, via_net):
        """The required clearance for one (cap pad, via) pair.

        The offender test in nudge_vias_for_unresolved and via_penalty's
        grazing test MUST both resolve through here: if they diverge, the
        nudger chases a via the grader no longer flags (or reports a cap
        unresolved that its own nudger then refuses to see). The via eff rows
        are built from this same call, so they cannot drift from it either."""
        return self._pair_or_flat(pad_floor, self._via_floor_for(via_net))

    def track_required(self, fa, fb, net_a, net_b):
        """The required clearance between two TRACKS: `required(fa, fb)` raised
        by any binding track-scoped `.kicad_dru` rule (#735).

        THE ONLY track-vs-track pair kind in this module, which is why it is a
        separate resolver rather than an argument to `required`. KiCad's rule
        condition names a track on BOTH sides, so a pad or a via on either side
        exempts the pair -- and every other requirement resolved here has one.
        The cap-pad arms, the via arms and the broad-phase over-reach are
        exempt by that condition, not by omission.

        Composition is check_drc's, term for term: resolve the pair first, THEN
        raise. So the rule outranks the net class and the layer rule and can
        lower neither. The raise is keyed on NETS, so it applies to the flat
        fallback too -- which is the whole answer on a board whose only
        declaration is a track rule."""
        base = self._pair_or_flat(fa, fb)
        if self._track is None:
            return base
        return self._track.track_pair(net_a, net_b, base)[0]

    @property
    def track_rules_active(self) -> bool:
        """True when the board declares a track rule at all.

        Public because `nudge_vias_for_unresolved` needs it to decide whether
        a SECOND sweep rung is worth running: with no rule the strict and the
        relaxed rung ask the same question, and sweeping twice for the same
        answer would double the cost of every `no clear spot` on every
        ordinary board. Read through `getattr`, so a duck-typed st is
        single-rung."""
        return self._track is not None

    @property
    def edge_margin(self) -> float:
        """THE copper-to-Edge.Cuts requirement of this run -- max(clearance,
        board_edge_clearance) -- and the one answer in this module (#733).

        Public, and read from OUTSIDE the class, for the same reason
        via_radius and via_required are: nudge_vias_for_unresolved gates the
        vias it RELOCATES and the connector segments it DRAWS, and those two
        gates must not be weaker than the one a cap pad has to pass.

        Left divergent, they were. The cap mover inset `usable` and built
        `edge_gate` at max(clearance, board_edge_clearance) while the nudger's
        edge_ok_point / edge_ok_seg spelled a bare `clearance`, so at the CLI
        defaults (0.55 / 0.25) the pass parked a relocated via and its
        connector 0.30mm closer to Edge.Cuts than any cap is allowed to sit --
        an under-block on copper the pass itself created and the writer keeps
        (place_fanout_clearance.py:138). Identically 0.30mm on the GUI path,
        which passes no flag at all.

        DELEGATES to the gate instead of storing the number a second time.
        `edge_gate` is what the cap mover's real-outline rect tests actually
        measure against, so a private duplicate here would be this very defect
        in miniature -- two names for one requirement, one refactor from
        drifting. (It replaces `_edge_margin`, which this module read nowhere.)

        What is NOT shared with the gate is its #628 owned-milled-ring
        exemption -- and read legality.py:512-525 before restoring it here,
        because the obvious reading is wrong. That exemption is not a
        rect-modelling workaround limited to the swallow probe; it covers the
        EDGE-MARGIN test too, and exists because a part whose own PADS caused a
        contour to be reclassified as an inner milled edge would otherwise have
        every pose inside its own relief vetoed (run 20's SW2: 0 legal poses of
        14884). Ownership is defined by pads, and a via has none -- it can never
        be the part that owns a ring, so there is nothing here to exempt. A
        barrel 0.30mm from a milled slot is a real copper-to-edge violation
        whichever way the parser's slot-vs-inner-outline ambiguity resolves, so
        the nudger should stand off. The cost is that it may decline a move near
        such a ring; it prints "no clear spot", which is visible and recoverable,
        unlike silent sub-margin copper. No tracked board carries a milled
        contour at all -- tests/test_733_*.py asserts that, so the claim expires
        loudly rather than quietly.
        """
        return self.edge_gate.margin

    def via_radius(self, via) -> float:
        """The RADIUS of one parser Via -- the one answer in this module (#732).

        The only place `--default-via-size` is applied. __init__ builds the
        keep-out list from this, and nudge_vias_for_unresolved's offender test,
        its candidate validation and its #313 connectivity tolerance all read it
        back through the same call, for the same reason via_required exists.

        Left divergent, a via whose size the board does not carry got the
        grader's default_via_size/2 and the nudger's hard-coded 0.25 for the
        SAME barrel. At the CLI's 0.3 default the nudger's is LARGER, so it
        moves vias the grader never flagged and lays connector copper for
        nothing; above 0.5 the sign flips and the grader flags a cap whose
        offender list then comes back EMPTY -- the `for v in offenders:` body
        never runs, so not even the "no clear spot" line prints and the cap is
        reported unresolved forever with nothing on screen. Both signs are
        reachable from the GUI too. Until #742 the GUI fed this the Basic tab's
        via SIZE -- the diameter of the vias fanout PLACES, a different
        quantity that happened to reach the same parameter -- so a recorded run
        replayed there at a different keep-out radius. It now has its own
        control, `cap_default_via_size`, defaulting to the CLI's 0.3.

        NOT the same question as _via_radius_by_id, and deliberately a separate
        mechanism. That map answers "what radius did THIS 4-tuple get", which
        for a tuple a test assigned wholesale is its own convention and must
        stay so. This answers "how wide is this barrel" -- a fact about the
        parser object, and the only form that works for a via injected into
        pcb_data.vias AFTER construction, which the tests do.
        """
        return _via_radius(via, self._default_via_size)

    def _pair_or_flat(self, fa, fb):
        """model.pair for two floors, falling back to the flat scalar whenever
        either side has none (an NPTH keep-out rect, a test-injected tuple)."""
        if fa is None or fb is None:
            return self.clearance
        return self._floors.pair(fa, fb)

    def _cap_pad_layers(self, cap):
        """This cap's per-pad copper layer sets, or None when unavailable.

        None for a duck-typed cap, and for a board that declares no copper
        layers at all: `pad_copper_layers(p, [])` resolves nothing for a `*.Cu`
        pad, so every such pad would read as copper-less and take the
        off-layer fallback -- an UNDER-block. It hits only `*.Cu` pads (an
        `F.Cu` / `B.Cu` / `F&B.Cu` pad still resolves), which is 0 of 308 cap
        pads on orangecrab but 468 of 2186 across the tracked corpus, all
        through-hole. Scoping switches off entirely rather than per-pad,
        because a board with no copper layers has no layer question to answer.
        """
        pl = getattr(cap, 'pad_layers', None)
        # #731: anchored on `pads`, NOT on `pad_floors`. pad_layers is built
        # unconditionally and pad_floors only when the model is active, so a
        # pad_floors-anchored check would return None on every INERT board --
        # switching layer scoping off precisely where the phantom is largest.
        # `pads` is the list pad_rects() is built from, so it is the alignment
        # that actually matters.
        if pl is None or len(pl) != len(getattr(cap, 'pads', ())):
            return None
        return pl if self._all_cu else None

    @staticmethod
    def _flat_pad(cap_layers, i):
        """True when cap pad `i` carries no copper and must be graded flat."""
        return cap_layers is not None and not cap_layers[i]

    @staticmethod
    def _seg_side(layer):
        """The 'F'/'B' collapse the track prune used to be keyed on, kept ONLY
        as the fallback for a cap whose real pad layers are unavailable.
        Byte-identical to what __init__ computed before #731."""
        return 'B' if (layer or '').startswith('B') else 'F'

    def _cap_seg_scope(self, cap):
        """(per-pad copper layer sets, their union) for the TRACK channel, or
        (None, None) when layer scoping is off for this cap -- a duck-typed
        cap, a misaligned list, a board declaring no copper layers, or a cap
        with no copper-carrying pad at all. ONE decision point, so the prune
        and the eff matrix can never disagree about whether scoping is on."""
        pl = self._cap_pad_layers(cap)
        if not pl:
            return None, None
        # An EMPTY union means every pad of this cap is copper-LESS (an NPTH
        # keep-out). It must NOT collapse to "scoping off" (#731 review): that
        # switched the guard off for the TRACK channel alone, so `_flat_pad`
        # read False and those pads were charged at the partner's netclass --
        # reinstating, in one channel, exactly the phantom 4bbfa4de removed
        # from every other. Measured on a staged orangecrab with R17's pads set
        # to np_thru_hole at a Default class of 0.4: 52 track cells charged at
        # 0.4 while the pad channel correctly charged the flat 0.1. check_drc
        # grades none of them -- it skips a copper-less pad outright.
        return pl, frozenset().union(*pl)

    def _seg_shares(self, layer, mine):
        """True when a track on `layer` can touch a cap pad whose copper layer
        set is `mine`. The ONE predicate the prune, the eff matrix and the
        disclosure all resolve through, so they cannot drift (#731).

          mine is None              -> scoping off for this cap; charge, as before
          layer not in self._all_cu -> an UNKNOWN layer (including None, a
                                       test-injected 'F'/'B', and every layer
                                       on a board declaring no copper): charge.
                                       Over-blocking is the only direction that
                                       can never ship a violation.
          mine empty                -> a copper-less pad, which check_drc grades
                                       not at all (#725): shares nothing.

        Exactness of the union prune follows: _seg_shares(L, mine[i]) implies
        `L not in _all_cu or L in union`, so a pair the per-pad grader would
        charge can never be pruned away."""
        return mine is None or layer not in self._all_cu or layer in mine

    def _cap_floors_ok(self, cap):
        """True when this cap carries a usable, index-aligned floor list. A
        _Cap built without the model (or by a test) grades flat."""
        pf = getattr(cap, 'pad_floors', None)
        return (self._floors is not None and pf is not None
                and len(pf) == len(getattr(cap, 'pads', ())))

    # ---- #736: THE one way a track enters this grader --------------------
    # Two private helpers plus one public registrar, and every track this
    # module grades goes through them: the ones present at construction, and
    # the connector copper nudge_vias_for_unresolved DRAWS afterwards. Before
    # #736 both halves were inline in __init__ and existed nowhere else, so
    # the post-nudge re-grade had no way to register what the pass had just
    # created -- it re-graded against the construction-time snapshot and could
    # report a cap RESOLVED while a connector this same run emitted grazed one
    # of its pads. Same shape, and the same reason, as via_radius /
    # via_required / edge_margin above: ONE answer per pair kind in this
    # module.

    def _register_segment(self, x1, y1, x2, y2, net_id, width, layer):
        """Grade one track into the track view, and return its tuple.

        THE only construction site for a track tuple, and the only writer of
        the id-keyed floor map -- both pinned by the source guard in
        tests/test_736_fanout_clearance_regrade_view.py, which reports
        offending line numbers rather than dumping the module.

        Takes PRIMITIVE fields rather than a parser Segment because its two
        callers hold different things: __init__ has real Segments, and
        register_new_segments has the writer-shaped dicts the nudger returns.
        A builder that accepted only one of those would have left the other
        re-deriving the tuple, which is the defect being fixed.

        It files the tuple BEFORE returning, on purpose. The floor map is
        keyed on the tuple's identity, so a tuple that died while its id entry
        lived would hand a recycled id ANOTHER track's floor -- silently, with
        no error. That is the hazard the NB above self.foreign_pad_floors
        names, and a factory that only BUILT would let a caller drop the
        reference and arm it. Build-and-file is one operation here because the
        invariant says it is.

        Element 5 is the OVER-REACH -- half width plus this track's own upper
        bound (#725), which _seg_effs strips back with the same _item_reach.
        Element 6 is the track's REAL layer, never an 'F'/'B' side collapse
        (#731).
        """
        fl = self._seg_floor_for(net_id, layer)
        t = (x1, y1, x2, y2, net_id,
             width / 2.0 + self._item_reach(fl), layer)
        self.segments.append(t)
        if fl is not None:
            self._seg_floor_by_id[id(t)] = fl
        return t

    def _prune_segs(self, cap, geom, source):
        """The tracks in `source` this cap could EVER be charged against --
        THE one spelling of the per-cap track prune.

        `geom` is the cap's SEED-pose (cx, cy, span, rect) from self._cap_geom,
        never its current rect. The bound below is the reachable-DISK argument:
        a cap moves at most max_displacement_cap from its SEED, so anything
        whose seed gap already exceeds that can never constrain it -- which is
        what makes this prune EXACT rather than an approximation. A MOVED pose
        would silently redefine what "exact" means.

        The bound is stated rather than cited, because the obvious citation is
        wrong: TestPruneRadiiStayExact checks that the lists never SHRINK when
        a requirement is raised, not that the pruned grade equals the unpruned
        one. Measured instead, over all 115 caps of every tracked board:
        `2*span + clearance - grid_overshoot - R_max` has a minimum slack of
        +1.009mm and never goes negative, where R_max is the largest
        seed-centre-to-pad-corner distance across all four reachable
        rotations. That figure is at --clearance 0.1; the bound carries a
        `+ clearance` term, so it is +1.109 at the shipped 0.2 and larger
        above. The grid overshoot is real and pre-existing --
        _candidate_positions snaps AFTER its radius test, so a final pose can
        sit up to grid_step*sqrt(2)/2 past max_displacement_cap -- and the
        2*span term absorbs it with an order of magnitude to spare.

        seg[5] already carries the segment's own over-reach, so adding this
        cap's excess over the flat scalar bounds the pair.

        #731: keep a track only if SOME pad of this cap shares its copper
        layer. The union is a superset of every per-pad set (see _seg_shares),
        so this can never drop a chargeable pair -- the list stays EXACT, as
        TestPruneRadiiStayExact requires. The per-pad half is applied in
        _seg_effs, and matters only for a cap whose pads differ (an SMD pad
        beside a through-hole one).

        The `_union is None` arm is a fallback for a cap that offers no copper
        pad at all -- cap detection admits n_copper == 0 (a paste-only
        C-prefixed part). Such a cap has an empty pad_rects, so NOTHING it
        keeps is ever graded and the arm cannot change an outcome; it keeps the
        old side filter purely so the pruned list stays the size it always was.
        It is NOT reachable for an empty copper list any more: `_all_cu`
        resolves through check_drc's own three-level fallback and is never
        empty, which is deliberate -- falling back to the side collapse there
        would DROP the B.Cu and inner tracks a through-hole cap pad shares
        copper with, an under-block (measured: 280 pairs on rp2350).

        Returns a NEW list. A caller must ASSIGN it and must never extend a
        cap's existing list in place: _seg_effs memoises on the source list's
        IDENTITY, so an in-place append leaves a memo that still passes the
        identity test while being one column short, and _seg_shortfalls then
        indexes past the end of its row.
        """
        ccx, ccy, span, _crect = geom
        seg_reach = (self._max_disp_cap + 2 * span + self.clearance
                     + max(0.0, cap.max_floor - self.clearance))
        _pl, _union = self._cap_seg_scope(cap)
        near_segs = []
        for seg in source:
            layer = seg[6]
            if _union is None:
                if self._seg_side(layer) != cap.side:
                    continue
            elif layer in self._all_cu and layer not in _union:
                continue
            d = _point_to_seg_dist(ccx, ccy, seg[0], seg[1], seg[2], seg[3])
            if d <= seg_reach + seg[5]:
                near_segs.append(seg)
        return near_segs

    def register_new_segments(self, seg_dicts) -> int:
        """Take copper that appeared AFTER construction into the PAD-SEGMENT
        channel; return how many tracks were filed (#736).

        The one caller is repair_fanout_clearance, with the `new_segments` list
        nudge_vias_for_unresolved returns: the connectors that restore
        continuity to each relocated via. That copper is drawn by the pass
        itself, milliseconds before the pass grades its own work, and both
        front ends WRITE it (placement/writer.py on the CLI, the plugin's
        pcbnew mirror in the GUI). Without this the final `unresolved` list and
        required_rows' disclosure are computed against the track snapshot
        __init__ took, so a cap can be reported RESOLVED while a connector this
        same run emitted grazes one of its pads.

        Takes the writer-shaped dicts the nudger already returns
        ({'start', 'end', 'width', 'layer', 'net_id'}) rather than a tail
        slice of pcb_data.segments, so nothing here depends on that list being
        append-only.

        INCREMENTAL, not a rebuild from the board, and the reason is not only
        cost. The nudger leaves the ORIGINAL attached segments exactly where
        they are (its own docstring says so), so every tuple already filed
        still describes real copper at the same coordinates and every pruned
        membership is as valid as it was. A rebuild would additionally REPLACE
        the tuples the floor map is keyed on: the originals die, their ids can
        be recycled by the replacements, and a track is handed ANOTHER track's
        floor -- the hazard __init__'s own NB names. Appending cannot.

        Each cap that keeps new copper is REASSIGNED a new list; a cap that
        keeps none is left alone, so it keeps its list identity and therefore
        its _seg_effs memo. That is what makes this O(caps x new tracks)
        instead of a second pass over the whole board.

        Deliberately NOT called from nudge_vias_for_unresolved. That function
        is duck-typed on `st` -- nine test files (#370, #617, #725, #730,
        #732, #733, #737, #750, #756) drive it with a stand-in carrying only
        `caps`, `vias` and `graze_penalty`, and one of those carries no via
        list at all -- so every `st` read there goes through getattr with a
        flat fallback. A RESOLVER
        has an honest flat fallback; a MUTATION does not, because "silently do
        nothing" is precisely the staleness this fixes. repair_fanout_clearance
        holds a real _Repair, so the call needs no escape hatch at all.

        A connector on a layer this board never declared is KEPT and CHARGED
        rather than dropped: `_all_cu` was resolved at construction, so such a
        layer reads as UNKNOWN and _seg_shares charges it. Over-blocking is the
        only direction that can never ship a violation.

        Not idempotent, and does not need to be: one caller, one call.
        """
        fresh = [self._register_segment(sd['start'][0], sd['start'][1],
                                        sd['end'][0], sd['end'][1],
                                        sd['net_id'], sd['width'], sd['layer'])
                 for sd in seg_dicts]
        if not fresh:
            return 0
        for ref, cap in self.caps.items():
            # `.get` rather than a bare index, matching what _pair_effs and
            # the eff builders already do: st.caps / st.cap_segs / st.segments
            # are all assignable from a test on a REAL _Repair (test_725 and
            # test_732 both do it), and a cap this object never pruned for has
            # no seed geometry to measure a reach from. Skipping it leaves
            # that cap exactly as the caller built it.
            geom = self._cap_geom.get(ref)
            if geom is None or ref not in self.cap_segs:
                continue
            kept = self._prune_segs(cap, geom, fresh)
            if kept:
                self.cap_segs[ref] = self.cap_segs[ref] + kept
                self._seed_seg_n.setdefault(
                    ref, len(self.cap_segs[ref]) - len(kept))
        return len(fresh)

    # ---- #747: THE one way a via enters this grader ----------------------
    # The via channel's half of what #736 did for tracks, and for the same
    # reason. __init__ and the post-nudge relocation each built a graded via
    # tuple, and the second copy carried a hand-written restatement of the
    # radius-map invariant the first one owns -- the shape #725, #731, #732,
    # #733 and #736 have each fixed once already.

    def _register_via(self, x, y, net_id, radius=None,
                      keepout=_DERIVE_KEEPOUT):
        """Grade one via into the via view, and return its 4-tuple.

        THE only construction site for a via tuple, and the only writer of
        the id-keyed radius map -- both pinned by the source guard in
        tests/test_747_fanout_clearance_via_registrar.py, which reports
        offending line numbers rather than dumping the module.

        Takes PRIMITIVE fields rather than a parser Via because its two
        callers hold different things: __init__ has real Via objects, and
        relocate_vias has the writer-shaped dicts the nudger returns. A
        builder that accepted only one of those would have left the other
        re-deriving the tuple, which is the defect being fixed.

        TWO OPTIONAL ARGUMENTS RATHER THAN ONE, and the asymmetry with
        _register_segment is the design rather than an accident.

        `keepout=None` means BUILD one -- this radius plus this net's own
        upper bound, spelled exactly as __init__ spelled it inline. That is
        __init__'s question. Note element 3 is NOT strip-back territory on
        this channel, unlike the track one: since #732 _via_effs reads the
        radius out of the map and uses element 3 verbatim only for a tuple the
        map does not carry, which is why the map exists at all. A RELOCATION
        asks the other
        one, and must CARRY the keep-out it was handed, verbatim: a moved via
        keeps its requirement, and a tuple a test assigned wholesale carries
        its own keep-out convention, which re-deriving would silently
        re-price. Nothing equivalent arises on the track side, because the
        copper register_new_segments files never had a tuple before.

        The marker is `_DERIVE_KEEPOUT`, not None, and the distinction is
        not pedantry: element 3 of a tuple assigned wholesale by a caller can
        itself be None. Spelled with None the two meanings collide at the same
        slot, and a relocated tuple carrying None either raised or had its
        keep-out silently re-derived -- the very thing the paragraph above
        forbids. Nothing in this module builds such a tuple, so this is a
        latent hazard rather than a live bug; it is spelled away instead of
        documented, because the next caller is under no obligation to read
        this.

        `radius=None` means THIS OBJECT DOES NOT KNOW this barrel's radius,
        so file nothing -- which is not the same as a radius of zero. A tuple
        absent from the map is graded at its own keep-out slot verbatim, i.e.
        exactly as injected (see the note beside the map in __init__), so
        relocating an absent tuple must produce another absent one, or the
        move starts pricing a via that _via_effs and via_penalty's flat path
        both agreed not to price.

        Neither a radius nor a keep-out is a contract violation rather
        than a default, and is unreachable from either caller. Named rather
        than left to surface as an arithmetic error on a None.

        It files the map entry BEFORE returning, but -- unlike
        _register_segment -- it does NOT append, and that difference is safe
        here for a reason the track channel does not have. THIS map's VALUE
        holds the tuple, so filing it IS the aliveness guarantee no matter
        where the caller then puts it; the segment floor map holds a bare
        floor, so only self.segments keeps a track tuple alive and its
        builder has to append. The two callers here genuinely disagree about
        placement -- __init__ APPENDS, relocate_vias SUBSTITUTES in order --
        so an appending builder would be wrong for one of them.
        """
        if radius is None and keepout is _DERIVE_KEEPOUT:
            raise ValueError('a via tuple needs a radius or a keep-out')
        t = (x, y, net_id,
             (radius + self._item_reach(self._via_floor_for(net_id)))
             if keepout is _DERIVE_KEEPOUT else keepout)
        if radius is not None:
            self._via_radius_by_id[id(t)] = (t, radius)
        return t

    def _prune_vias(self, cap, geom, source):
        """The vias in `source` this cap could EVER be charged against --
        THE one spelling of the per-cap via prune.

        `geom` is the cap's SEED-pose (cx, cy, span, rect) from self._cap_geom,
        never its current rect, for the reason the track twin gives at length:
        the bound is the reachable-DISK argument -- a cap moves at most
        max_displacement_cap from its SEED, so anything whose seed gap already
        exceeds that can never constrain it -- and that is what makes this
        prune EXACT rather than an approximation. A MOVED pose silently
        redefines what "exact" means, and required_rows grades the via
        shortfalls at the SEED pose as well as the final one, so a moved-pose
        radius is a superset for neither.

        v[3] is the via's radius PLUS its own keep-out, so a via that can reach
        a pad must be within (move + span + that) of the seed centre. #725's
        slack adds this cap's excess over the flat scalar, which bounds the
        pair from the other side: the two terms together are `radius +
        max(clearance, the via's floor) + max(0, the cap's floor - clearance)`,
        which is at least `radius + the pair's requirement` for every pair the
        resolver can return, because a .kicad_dru rule REPLACES a pair value
        but max_floor is each item's own upper bound over all of them.

        THE MARGIN IS THINNER THAN THE TRACK CHANNEL'S, and is stated rather
        than left to be rediscovered. The residual slack is
        `span - grid_overshoot - R_max`, where R_max is the largest
        seed-centre-to-pad-corner distance over the reachable rotations AND
        the cap's own seed pose -- which is not among the four when a
        footprint sits at a non-orthogonal angle, inert on today's corpus
        where every seed_rot is a multiple of 90, and swept by the arm
        regardless -- and the overshoot is real and pre-existing (_candidate_positions snaps
        AFTER its radius test, so a final pose can sit up to
        grid_step*sqrt(2)/2 past max_displacement_cap). Measured over
        every cap of every tracked board that offers one -- 115 caps on
        5 of the 22 -- the minimum is +0.1267mm (orangecrab_ext_pll R14,
        span 0.7826, R_max 0.5852), the maximum +0.2716mm, and it is
        never negative. The track pruner records +1.009mm for the same
        quantity, because its bound carries `2*span + clearance` where this one
        carries `span`. That is a fact about the bound __init__ has always
        used, not an exposure #775 opens -- but this method now applies it at
        the post-nudge refresh as well, so the number lives with the predicate.

        NO LAYER GATE, unlike the track twin, and the reason is worth stating
        accurately rather than flatteringly. Every via in this list is priced
        AS THOUGH it spanned every copper layer -- self.vias is built from
        pcb_data.vias with no span filter and the 4-tuple does not carry
        via.layers -- so a blind or buried barrel is over-blocked here. That
        is the safe direction and it predates #775, but it is an
        approximation, not a geometric necessity: on a through-via there is
        genuinely no per-pad copper scope to intersect, and on the others
        there is one this channel does not model. Either way nothing a
        #731-style union could exclude.

        Returns a NEW list. A caller must ASSIGN it and must never extend a
        cap's existing list in place: _via_effs memoises on the source list's
        IDENTITY, so an in-place append leaves a memo that still passes the
        identity test while being one column short, and _via_shortfalls then
        indexes past the end of its row.
        """
        ccx, ccy, span, _crect = geom
        via_slack = max(0.0, cap.max_floor - self.clearance)
        near_vias = []
        for v in source:
            if math.hypot(v[0] - ccx, v[1] - ccy) <= (
                    self._max_disp_cap + span + v[3] + via_slack):
                near_vias.append(v)
        return near_vias

    def relocate_vias(self, via_moves) -> int:
        """Take the vias nudge_vias_for_unresolved just MOVED into the graded
        via view; return how many TUPLES were replaced (#747).

        The one caller is repair_fanout_clearance, with the `via_moves` list
        the nudger returns. The nudger mutates the parser's Via objects in
        place and REPORTS what it did; this is where that report lands in the
        view the re-grade and required_rows' disclosure read, so no cap is
        graded against a barrel that has moved.

        Takes the writer-shaped payload the nudger ALREADY returns --
        (old_x, old_y, {'x', 'y', 'net_id', ...}) -- so no return contract had
        to change to make this method possible. #747's own text assumed one
        did; measured, everything a relocation needs was already in the dict
        that placement/writer.py and the plugin's pcbnew mirror both consume,
        so not one of the nine test files that unpack that pair moves.

        SUBSTITUTES, and rebuilds the list once PER MOVE. Per move, because a
        via can be relocated TWICE in one call: the offender loop re-reads
        pcb_data.vias for each unresolved cap and keeps no moved-set, so a
        barrel moved for one cap is matched at its NEW position when the next
        cap flags it, and the second report's old position IS the first
        report's landing. A single pass resolving every move against the
        pre-nudge list would leave the tuple at the first hop while the board
        via sits at the second, silently. Substitution rather than a rebuild
        from the board, for the reason the track registrar is incremental:
        every tuple that did NOT move keeps its identity, and the radius map
        is keyed on exactly that.

        The list is reassigned even for a move that matched nothing, and
        the reason needs stating carefully, because the obvious version of it
        is not what happens on the real path. _via_effs revalidates its memo
        on the identity of the list it is HANDED, which on the real path is
        the per-cap pruned view, not this one -- and refresh_cap_vias
        rebuilds that on the next line regardless. The rebind matters for a
        holder that points cap_vias AT this list, which since #775 is a TEST
        idiom only -- the engine never points it here -- and the state any
        second call would find. A
        review measured the difference: mutating this in place leaves the
        end-to-end result byte-identical, and only the mechanism arms in
        tests/test_747_fanout_clearance_via_registrar.py go red. Kept
        unconditional because it is cheap and because the alternative is a
        rule about which callers are safe.

        The OLD map entry is left in place, on purpose. It is what keeps the
        old tuple alive, and dropping it would free an id the very next tuple
        this builder makes could be handed -- the hazard the map's own note
        names. The map therefore grows by one per relocation and stops being
        the same length as self.vias, which is a fact about a nudged board
        rather than a leak.

        MATCHED ON POSITION AND NET, which is what the writer's own
        removal already matches on (placement/writer.py's net_ids argument,
        and the plugin's pcbnew twin beside it). Worded to keep the wrap off a
        line that would otherwise begin with a bare statement keyword: this
        module is grepped by a dozen source guards and by every mutate_*.py
        anchor, and a line of prose that reads as code is how #756 lost four
        battery rows. Position alone relocates every
        tuple at the vacated spot, including a coincident via of a DIFFERENT
        net that has not moved at all -- a phantom at the landing and a hole
        where it really is. What this still cannot separate is two coincident
        vias of the SAME net: only tuple identity could, and the nudger has
        none to hand back through a dict two front ends also read. No tracked
        board carries either shape.

        A move that matches NOTHING is neither an error nor warned about:
        several test files inject a via into pcb_data.vias after construction,
        so the nudger legitimately relocates barrels this object never graded.
        The returned count is what tells the two apart -- it counts TUPLES
        replaced, not moves handed in.

        Leaves the per-cap view alone -- refresh_cap_vias owns it, and the
        caller calls that on the line below. A registrar that also
        refreshed would be a second place deciding what a cap can see.

        Not idempotent, and does not need to be: one caller, one call.
        """
        n = 0
        for old_x, old_y, spec in via_moves:
            net, nx, ny = spec['net_id'], spec['x'], spec['y']
            rebuilt = []
            for t in self.vias:
                if (t[2] == net and abs(t[0] - old_x) < 1e-6
                        and abs(t[1] - old_y) < 1e-6):
                    # `.get` spelled inline rather than hoisted to a local:
                    # test_725 reassigns the whole map on a real _Repair, and
                    # an alias is one refactor away from being taken before
                    # that assignment.
                    rec = self._via_radius_by_id.get(id(t))
                    moved = self._register_via(
                        nx, ny, t[2],
                        radius=None if rec is None else rec[1],
                        keepout=t[3])
                    # #779: carry the SEED position forward. `.get` with
                    # the current tuple as the fallback is what makes a
                    # SECOND relocation of the same barrel keep its
                    # ORIGINAL seed rather than the intermediate landing --
                    # and this method really can move one twice, for the
                    # reason its own docstring gives above.
                    _sd = self._via_seed_xy.get(id(t))
                    self._via_seed_xy[id(moved)] = (
                        (moved, _sd[1], _sd[2]) if _sd is not None
                        else (moved, t[0], t[1]))
                    rebuilt.append(moved)
                    n += 1
                else:
                    rebuilt.append(t)
            self.vias = rebuilt
        return n

    def refresh_cap_vias(self) -> int:
        """Re-prune the per-cap via view after the registrar moved barrels;
        return how many caps were reassigned (#775).

        The one caller is repair_fanout_clearance, on the line below the via
        registrar and under the same `if via_moves:` guard, exactly where an
        inline dict comprehension over the whole-board list used to sit. Same
        idiom as the two registrars beside it: the nudger reports, and a real
        _Repair applies the report.

        RE-PRUNES rather than de-prunes, which is the whole of #775. The old
        line pointed every cap at the whole-board list, and that is
        NUMERICALLY IDENTICAL rather than approximately so: the prune is
        exact, so a via it drops fails its keep-out test outright and
        contributes no addend at all -- not a small one -- and the kept vias
        stay in their original order, so even the float summation order is
        unchanged. Measured on kicad_files/orangecrab_ext_pll.kicad_pcb at
        --clearance 0.1, in both in-repo configurations that relocate
        barrels: graze_penalty, the via shortfalls and required_rows all
        compare EQUAL, not merely close, with 10 caps carrying a
        non-zero penalty as the positive control.

        What the de-prune cost is the eff matrix _via_effs then rebuilds per
        cap, over n_pads x n_ALL_vias instead of n_pads x n_in_reach, plus
        the same widening of via_penalty's and the shortfall loops' own
        inner passes. On that board (65 caps, 2 pads each, 136 vias) the
        matrix is 1,914 cells pruned against 17,680 de-pruned at the
        SHIPPED max_displacement_cap of 3.0, and 144 against 17,680 at
        0.0 -- and the re-grade plus the disclosure that read them took
        17.7ms against 49.5ms, and 2.8ms against 35.9ms (best of 7).
        The smaller reach makes the ratio LARGER, so the shipped default
        is the conservative figure and the forcing config is the
        flattering one.

        EVERY cap in self.caps is reassigned, and this is where the track
        registrar's freedom to SKIP does not carry over. That one may leave a
        cap it has no seed pose for exactly as the caller built it, because
        every tuple already in that cap's list still describes real copper at
        the same coordinates. Here it does not: the registrar SUBSTITUTED the
        tuples it moved, so a cap left alone keeps the PRE-move tuple -- a
        phantom via at the landing and a hole where the barrel really is,
        which is the exact failure the registrar's own docstring names about
        matching on position alone. A cap with no seed pose therefore falls
        back to a COPY of the whole list rather than being skipped:
        over-blocking is the only direction that can never ship a violation,
        and a copy rather than the list itself so that "every value in this
        view is a list this method made" holds for every cap, with no
        exception a caller has to know about.

        Does NOT touch _cap_via_eff. That memo revalidates on the identity of
        the list it is HANDED and every list here is new, so the rows rebuild
        on their own. Clearing it as well would be a second mechanism doing
        one job, and the one that goes stale first.

        Assigns PER KEY, where the line this replaced rebound the whole dict.
        That is strictly more conservative for every reader -- all of them
        index by a ref taken from self.caps -- with one consequence worth
        naming: a key left in this view for a ref that is NO LONGER in
        self.caps would keep its pre-move list rather than being dropped.
        Unreachable as the module stands, because self.caps is written once
        in __init__ and never narrowed.

        Exactly idempotent in VALUE and not in COST: a second call builds
        another set of equal lists and discards every memo again. One caller,
        one call.
        """
        n = 0
        for ref, cap in self.caps.items():
            # `.get` rather than a bare index, matching what the track
            # registrar and the eff builders already do: st.caps is assignable
            # from a test on a REAL _Repair -- test_775's own geomless-cap arm
            # does exactly that -- and a cap this object never pruned for has
            # no seed pose to measure a reach from.
            #
            # The track registrar's twin of this comment cites test_725 and
            # test_732 for the same claim. A fact-check of #775 measured that
            # those two assign cap_vias / cap_segs / segments on a real
            # _Repair but never `caps`, so this one does not repeat the
            # citation. The defensive `.get` is right either way.
            geom = self._cap_geom.get(ref)
            self.cap_vias[ref] = (list(self.vias) if geom is None
                                  else self._prune_vias(cap, geom, self.vias))
            n += 1
        return n

    def _pad_effs(self, ref, cap):
        """[cap pad i][pruned foreign pad j] required clearance, or None for
        the flat path."""
        if not self._cap_floors_ok(cap):
            return None
        src = self.cap_foreign_pads[ref]
        rec = self._cap_pad_eff.get(ref)
        if rec is not None and rec[0] is src:
            return rec[1]
        by_id = self._fp_floor_by_id
        cl = self._cap_pad_layers(cap)
        rows = [[self.clearance] * len(src) if self._flat_pad(cl, i)
                else [self._pair_or_flat(fa, by_id.get(id(t))) for t in src]
                for i, fa in enumerate(cap.pad_floors)]
        self._cap_pad_eff[ref] = (src, rows)
        return rows

    def _seg_effs(self, ref, cap):
        """[cap pad i][pruned segment j] final half-width KEEP-OUT -- the
        track's half width plus that pair's required clearance -- or
        `_OFF_LAYER` for a pair that shares no copper layer (#731). None only
        for a cap offering neither floors nor layers (the duck-typed path).

        Built even when the clearance model is INERT: the layer gate is a fact
        about the stackup, not about what the board declares. On that path
        every on-layer cell is the tuple's own t[5] BIT-EXACTLY and pair() is
        never called, so an inert board stays byte-identical."""
        src = self.cap_segs[ref]
        n = len(getattr(cap, 'pads', None) or ())
        floors_ok = self._cap_floors_ok(cap)
        cap_layers, _union = self._cap_seg_scope(cap)
        # Nothing to price AND nothing to gate -> the duck-typed flat path.
        if n == 0 or (not floors_ok and cap_layers is None):
            return None
        rec = self._cap_seg_eff.get(ref)
        if rec is not None and rec[0] is src:
            return rec[1]
        by_id = self._seg_floor_by_id
        floors = [by_id.get(id(t)) for t in src]
        # t[5] is half width + the segment's own over-reach; strip it back.
        halves = [t[5] - self._item_reach(f) for t, f in zip(src, floors)]
        rows = []
        for i in range(n):
            fa = cap.pad_floors[i] if floors_ok else None
            mine = None if cap_layers is None else cap_layers[i]
            flat_pad = self._flat_pad(cap_layers, i)
            row = []
            for j, t in enumerate(src):
                if not self._seg_shares(t[6], mine):
                    # #731: a pair sharing no copper layer is not repriced, it
                    # is REMOVED. _OFF_LAYER makes _seg_shortfalls' existing
                    # cheap bbox reject fire on its first comparison, so the
                    # gate costs the innermost loop nothing at all.
                    row.append(_OFF_LAYER)
                elif fa is None and floors[j] is None:
                    # Neither side has a floor: the pair is worth exactly the
                    # tuple's own over-reach. Taking t[5] directly rather than
                    # rebuilding it as halves[j] + clearance keeps the
                    # resolver OUT of the inert path entirely -- _pair_or_flat
                    # is not called, so an inert board cannot start consulting
                    # a model, and an injected tuple grades exactly as
                    # injected. NB this is about the CALL, not the
                    # rounding: the arithmetic round-trip turns out to be
                    # float-exact everywhere measured (0 of rp2350's 1063
                    # segment round-trips DIFFER, and 0 of orangecrab's 618),
                    # which is precisely why an equality assertion cannot tell
                    # the two forms apart and the test spies the call instead.
                    row.append(t[5])
                elif flat_pad:
                    row.append(halves[j] + self.clearance)
                else:
                    row.append(halves[j] + self._pair_or_flat(fa, floors[j]))
            rows.append(row)
        self._cap_seg_eff[ref] = (src, rows)
        return rows

    def _via_effs(self, ref, cap, vias):
        """[cap pad i][via j] final KEEP-OUT for `vias` -- the via's radius plus
        that pair's required clearance -- or None for the flat path. The
        requirement is resolved through via_required, so this can never
        disagree with the offender test in nudge_vias_for_unresolved."""
        if not self._cap_floors_ok(cap):
            return None
        rec = self._cap_via_eff.get(ref)
        if rec is not None and rec[0] is vias:
            return rec[1]
        # The radius comes from the identity map, never from arithmetic on
        # v[3]: a tuple a test assigned wholesale carries its own keep-out
        # convention, and re-deriving would mis-price it silently. Absent ->
        # graded flat, i.e. v[3] is used as-is.
        by_id = self._via_radius_by_id
        cl = self._cap_pad_layers(cap)
        rows = []
        for i, fa in enumerate(cap.pad_floors):
            flat = self._flat_pad(cl, i)
            row = []
            for v in vias:
                rec = by_id.get(id(v))
                if rec is None:
                    row.append(v[3])
                elif flat:
                    row.append(rec[1] + self.clearance)
                else:
                    row.append(rec[1] + self.via_required(fa, v[2]))
            rows.append(row)
        self._cap_via_eff[ref] = (vias, rows)
        return rows

    def _pair_effs(self, ref, cap, oref, other):
        """[this cap's pad i][other mover's pad j] required clearance, or None.
        Keyed on the ORDERED pair so the row index always addresses `cap`; the
        summed shortfall itself is symmetric, so the seed baseline is not."""
        if not (self._cap_floors_ok(cap) and self._cap_floors_ok(other)):
            return None
        key = (ref, oref)
        rows = self._cap_pair_eff.get(key)
        # The other three memos guard on their source list's identity; this one
        # has no list to key on, so it validates its own SHAPE instead. Both
        # exist for the same reason: st.caps / st.cap_* are assignable from a
        # test, and a stale memo here is an IndexError or a wrong requirement.
        if rows is not None and (len(rows) != len(cap.pad_floors)
                                 or (rows and len(rows[0]) != len(other.pad_floors))):
            rows = None
        if rows is None:
            mine, theirs = self._cap_pad_layers(cap), self._cap_pad_layers(other)
            rows = [[self.clearance
                     if (self._flat_pad(mine, i) or self._flat_pad(theirs, j))
                     else self._pair_or_flat(fa, fb)
                     for j, fb in enumerate(other.pad_floors)]
                    for i, fa in enumerate(cap.pad_floors)]
            self._cap_pair_eff[key] = rows
        return rows

    def required_rows(self, net_names=None, limit=200):
        """Rows `[cap_ref, '<kind> <partner>', mm, source]` for every pair this
        pass actually CHARGED at a requirement above the flat scalar, at the
        SEED pose or the final one (see `both` below for why both).

        Same 4-column shape as legality.grade_pad_legality's 'required', so
        legality.format_required_clause renders it unchanged. `[]` when inert.
        Only charged pairs are listed -- an in-reach pair that happens to be
        clear is not a finding, it is the normal case on a declaring board.
        The partner is named by NET, not by reference: foreign_pads / vias /
        segments carry no owning ref and their tuple shapes are pinned."""
        if self._floors is None:
            return []
        names = net_names or {}
        pair_with_source = self._floors.pair_with_source
        rows = []

        def net_label(net):
            if net == 0:
                return '<no net>'
            if net == -1:
                return '<hole>'
            return names.get(net, str(net))

        def best(floors_a, items, floor_of, net_of, only,
                 cap_layers=None, layer_of=None, side_of=None):
            """Worst requirement, with its source, over the items on `only`.

            Takes the SAME two per-pad guards the eff builders take, because a
            report that re-derives the price instead of mirroring it reports
            pairs the pass never charged. Both were measured: without the
            layer guard, up to 43% of the rows on rp2350 named a cap pad
            against a track on a layer it does not occupy -- pairs check_drc
            does not grade and `_seg_effs` prices at the flat scalar."""
            out = {}
            for t in items:
                net = net_of(t)
                if net not in only:
                    continue
                fb = floor_of(t)
                if fb is None:
                    continue
                # an SMD foreign pad on the other board side is skipped by
                # _pad_shortfalls, so it charges nothing and belongs in no row
                pside = None if side_of is None else side_of(t)
                if pside is not None and pside != cap.side:
                    continue
                layer = None if layer_of is None else layer_of(t)
                for i, fa in enumerate(floors_a):
                    # a copper-less cap pad is graded flat, so it charges
                    # nothing and belongs in no row
                    if _Repair._flat_pad(cap_layers, i):
                        continue
                    # ...and neither does a pair that shares no copper layer.
                    # THE SAME predicate _seg_effs prices with (#731), not a
                    # second derivation -- a report that re-derives reports
                    # pairs the pass never charged. `layer` is None for the
                    # pad/via kinds, and _seg_shares charges an unknown layer,
                    # so this is a no-op for them.
                    if not self._seg_shares(
                            layer, None if cap_layers is None
                            else cap_layers[i]):
                        continue
                    mm, src = pair_with_source(fa, fb)
                    if src and mm > out.get(net, (0.0, ''))[0]:
                        out[net] = (mm, src)
            return out

        for ref, cap in self.caps.items():
            if not self._cap_floors_ok(cap):
                continue
            fls = cap.pad_floors
            x, y, rot = cap.x, cap.y, cap.rot
            sx, sy, srot = cap.seed_x, cap.seed_y, cap.seed_rot

            def both(fn, seed_kw=None):
                """Nets charged at the SEED pose or the final one.

                The seed half is the point: the pass SUCCEEDS by leaving
                nothing charged, so a final-pose-only report is empty exactly
                when the raised requirement did its work -- and the operator
                could not tell a run graded at --clearance from one graded at
                five times it.

                #736: `seed_kw` scopes the SEED half to the copper that
                existed at the seed, so a connector the pass DREW is not
                charged against the cap's seed pads -- a pair no board ever
                had, the same class of phantom #731 removed from this very
                report.

                TWO of the three kinds need it, and this comment used to
                say one. The via channel DOES gain something after
                __init__: since #747 relocate_vias SUBSTITUTES a moved
                barrel at its new coordinates, so the seed half was grading
                the cap's seed pose against post-nudge via positions
                (#779). Measured on orangecrab_ext_pll at the one in-repo
                configuration that relocates barrels: 43 of 65 caps held a
                relocated barrel and 7 produced a different seed-half
                number -- discarded only because that board declares no
                floor, so best() emitted nothing.

                The PAD channel is the one that genuinely gains nothing:
                foreign pads are never added, moved or re-priced by the
                pass."""
                out = set(fn(ref, cap, sx, sy, srot, **(seed_kw or {})))
                out |= set(fn(ref, cap, x, y, rot))
                return out

            cl = self._cap_pad_layers(cap)
            for kind, items, floor_of, net_of, charged, layer_of, side_of in (
                    ('pad', self.cap_foreign_pads[ref],
                     lambda t: self._fp_floor_by_id.get(id(t)), lambda t: t[4],
                     both(self._pad_shortfalls), None, lambda t: t[5]),
                    ('via', self.cap_vias[ref],
                     lambda t: self._via_floor_for(t[2]), lambda t: t[2],
                     both(self._via_shortfalls, {'seed_pos': True}),
                     None, None),
                    ('track', self.cap_segs[ref],
                     lambda t: self._seg_floor_by_id.get(id(t)), lambda t: t[4],
                     both(self._seg_shortfalls,
                          {'upto': self._seed_seg_n.get(ref)}),
                     lambda t: t[6], None)):
                for net, (mm, src) in best(fls, items, floor_of, net_of,
                                           set(charged), cl, layer_of,
                                           side_of).items():
                    rows.append([ref, '{} {}'.format(kind, net_label(net)),
                                 round(mm, 6), src])
            # mover-vs-mover: the shortfall is a scalar per pair, so charge the
            # pair as a whole and name it by the partner's reference.
            cand = cap.pad_rects(x, y, rot)
            seed = cap.pad_rects(sx, sy, srot)
            for oref in self.cap_caps[ref]:
                other = self.caps[oref]
                if not self._cap_floors_ok(other):
                    continue
                effs = self._pair_effs(ref, cap, oref, other)
                # seed pose OR final pose, for the reason `both` gives above
                now = _pad_pair_shortfall(cand, other.pad_rects(),
                                          self.clearance, effs)
                was = _pad_pair_shortfall(
                    seed, other.pad_rects(other.seed_x, other.seed_y,
                                          other.seed_rot),
                    self.clearance, effs)
                if now <= EPS and was <= EPS:
                    continue
                mm, src = 0.0, ''
                theirs = self._cap_pad_layers(other)
                for i, fa in enumerate(fls):
                    if self._flat_pad(cl, i):
                        continue
                    for j, fb in enumerate(other.pad_floors):
                        if self._flat_pad(theirs, j):
                            continue
                        m, sc = pair_with_source(fa, fb)
                        if sc and m > mm:
                            mm, src = m, sc
                if src:
                    rows.append([ref, 'cap ' + oref, round(mm, 6), src])
        rows.sort(key=lambda r: (-r[2], r[0], r[1]))
        return rows[:limit]

    def _overlap(self, a, b):
        """Courtyard-clearance shortfall between two rects (0 if clear).

        Deliberately NOT #725-converted: a courtyard is a mechanical/assembly
        extent, not copper, so charging a netclass or a pad override here would
        move caps for a reason KiCad's DRC never raises. The cap_static prune
        that gates it is therefore still exact at the flat scalar."""
        return max(0.0, self.clearance - _rect_gap(a, b))

    def via_penalty(self, cap, x, y, rot, vias=None, ref=None):
        """Sum of foreign-net via penetration depths for a cap placement
        (how far each pad intrudes inside a different-net via's keep-out).

        vias defaults to the full board list; callers with a ref pass the
        per-cap pruned list (self.cap_vias[ref]), which is exact -- a via that
        can penetrate a pad is necessarily within the cap's reach.

        #725: `ref` keys the per-pair required-clearance memo. Without it (the
        4-positional test/default path) the flat scalar is used, which is what
        the whole-board `vias` default is for anyway."""
        vias = self.vias if vias is None else vias
        effs = None if ref is None else self._via_effs(ref, cap, vias)
        pen = 0.0
        if effs is None:
            # The tuple's slot is the prune OVER-reach, not a requirement, so
            # grading it directly would charge more than any pair needs (and
            # more than upstream's slot, which was radius + clearance). Recover
            # the radius where we know it and grade at the flat scalar, which
            # is what this path documents. A tuple absent from the map -- one a
            # test assigned wholesale -- is graded exactly as injected.
            by_id = self._via_radius_by_id
            for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
                for v in vias:
                    vx, vy, vnet, keepout = v
                    if vnet == net:
                        continue
                    rec = by_id.get(id(v))
                    ko = keepout if rec is None else rec[1] + self.clearance
                    d = _point_to_rect_dist(vx, vy, (bx0, by0, bx1, by1))
                    if d < ko - EPS:
                        pen += (ko - d)
            return pen
        # #725 active path: effs[i][j] IS the pair's keep-out (via radius +
        # required clearance), precomputed -- the tuple's own keepout slot is
        # the prune over-reach and is not the requirement.
        for i, (bx0, by0, bx1, by1, net) in enumerate(cap.pad_rects(x, y, rot)):
            row = effs[i]
            for j, (vx, vy, vnet, _ko) in enumerate(vias):
                if vnet == net:
                    continue
                ko = row[j]
                d = _point_to_rect_dist(vx, vy, (bx0, by0, bx1, by1))
                if d < ko - EPS:
                    pen += (ko - d)
        return pen

    def _via_shortfalls(self, ref, cap, x, y, rot, seed_pos=False):
        """PER-FOREIGN-NET via penetration for a placement, keyed by the via's
        net_id (#445) -- the via analogue of _seg_shortfalls/_pad_shortfalls.
        A net absent from the dict is fully clear; a positive value is a real
        PAD-VIA DRC violation. Backs the hard accept gate so a move can never
        drop a pad onto a via net that was clear at the seed, no matter how
        much track graze it relieves elsewhere (zynq_ad9364 R2 onto the
        DDR3_VREF via).

        #779: `seed_pos` charges each barrel where it sat AT THE SEED,
        which is what required_rows' seed half needs. Since #747 the via
        registrar SUBSTITUTES a relocated tuple at its new coordinates, so
        without this the seed half grades the cap's seed pose against
        post-nudge barrels -- a pair no board ever had, the same class of
        phantom #736 removed from the TRACK half of this report.

        A COORDINATE SWAP rather than a filtered list, and that is forced:
        the eff rows below are index-aligned with `vias`, so dropping or
        reordering entries would mis-index them. It is also why the track
        channel's `upto` has no analogue here -- register_new_segments
        APPENDS, so its seed-era entries are a PREFIX, while relocate_vias
        substitutes in place and leaves the length unchanged.

        Costs nothing on a run that moved nothing: the map is empty, the
        comprehension is skipped, and every lookup would have missed.

        WHAT THIS DOES **NOT** FIX, stated because the swap looks more
        complete than it is. Only the COORDINATES are seed-era; the
        MEMBERSHIP of cap_vias[ref] is still a post-nudge view, because
        refresh_cap_vias re-prunes at CURRENT positions. A barrel inside
        the cap's reach at the seed that moved OUT of it is gone from the
        list, and no coordinate swap can bring back an entry that is not
        there. #775's `the prune is EXACT` bound does not cover this
        caller: it was derived for the pose the list was pruned at.

        Measured on orangecrab_ext_pll at the relocating configuration:
        it HAPPENS, once -- C59's net-19 barrel, seed gap 3.7469 against a
        4.0326 threshold, landing gap 4.2060 -- and it costs nothing,
        because that pair charges 0.0000 at the seed pose anyway. The
        margin is max_displacement_cap (3.0mm shipped) against a sub-mm
        nudge, and it is USER-SHRINKABLE: --max-displacement-cap, and a
        GUI spin whose minimum is 0.0. So this is a bound worth knowing
        rather than a defect that has fired.
        """
        by_net: Dict[int, float] = {}
        vias = self.cap_vias[ref]
        # index-aligned seed coordinates, or None to read them off the
        # tuple as every other caller does
        at = None
        if seed_pos and self._via_seed_xy:
            _sx = self._via_seed_xy
            at = [_sx[id(t)][1:] if id(t) in _sx else (t[0], t[1])
                  for t in vias]
        effs = self._via_effs(ref, cap, vias)
        if effs is None:
            for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
                for j, (vx, vy, vnet, keepout) in enumerate(vias):
                    if vnet == net:
                        continue
                    if at is not None:
                        vx, vy = at[j]
                    d = _point_to_rect_dist(vx, vy, (bx0, by0, bx1, by1))
                    if d < keepout - EPS:
                        by_net[vnet] = by_net.get(vnet, 0.0) + (keepout - d)
            return by_net
        for i, (bx0, by0, bx1, by1, net) in enumerate(cap.pad_rects(x, y, rot)):
            row = effs[i]
            for j, (vx, vy, vnet, _ko) in enumerate(vias):
                if vnet == net:
                    continue
                if at is not None:
                    vx, vy = at[j]
                keepout = row[j]
                d = _point_to_rect_dist(vx, vy, (bx0, by0, bx1, by1))
                if d < keepout - EPS:
                    by_net[vnet] = by_net.get(vnet, 0.0) + (keepout - d)
        return by_net

    def _seg_shortfalls(self, ref, cap, x, y, rot, upto=None):
        """PER-FOREIGN-NET track clearance shortfall for a placement, over the
        tracks this cap's pads actually SHARE COPPER WITH (#731),
        keyed by the track's net_id (#441): {snet: sum of (halfw - dist) over that
        net's segments this cap penetrates}. A net absent from the dict is fully
        clear. halfw already includes the clearance, so a positive shortfall is a
        real PAD-SEGMENT DRC violation; kept PER-NET so the accept gate can forbid
        penetrating a net that was CLEAR at the seed even when the aggregate
        shortfall drops -- the zynq_ad9364 GND<->NetR2_2 repair-pass short (a move
        that relieved a 260um DDR3_BA2 graze by dropping a GND pad 85um onto a
        previously-clear NetR2_2 track looked like a net improvement to the old
        scalar baseline). Uses the per-cap pruned, layer-scoped track list."""
        by_net: Dict[int, float] = {}
        segs = self.cap_segs[ref]
        # #736: `upto` grades only the first N entries -- the SEED-ERA ones,
        # since register_new_segments APPENDS. Used by required_rows' seed
        # half, which must not charge copper this pass itself drew. A PREFIX
        # rather than a filtered subset on purpose: the eff rows below are
        # index-aligned with this list, and a subset would mis-index them.
        if upto is not None:
            segs = segs[:upto]
        effs = self._seg_effs(ref, cap)
        if effs is None:
            # No matrix: a duck-typed cap, or a real _Cap with no copper pad
            # at all (cap detection admits n_copper == 0, e.g. a paste-only
            # C-prefixed part -- whose pad_rects is empty, so this loop does
            # nothing either way). There is no layer to gate on here, and an
            # injected tuple grades exactly as injected.
            for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
                for x1, y1, x2, y2, snet, halfw, _layer in segs:
                    if snet == net:
                        continue
                    # cheap reject: the segment's bbox can't reach the pad rect
                    if (min(x1, x2) > bx1 + halfw or max(x1, x2) < bx0 - halfw
                            or min(y1, y2) > by1 + halfw
                            or max(y1, y2) < by0 - halfw):
                        continue
                    d = _seg_to_rect_dist(x1, y1, x2, y2, (bx0, by0, bx1, by1))
                    if d < halfw - EPS:
                        by_net[snet] = by_net.get(snet, 0.0) + (halfw - d)
            return by_net
        # #725 active path: effs[i][j] IS the pair's keep-out, and the cheap
        # bbox reject widens with it -- a reject left at the flat half width
        # would silently drop the pairs this fix exists to charge. Since #731
        # the row also carries the LAYER gate, as _OFF_LAYER; the same cheap
        # reject is what makes that free.
        for i, (bx0, by0, bx1, by1, net) in enumerate(cap.pad_rects(x, y, rot)):
            row = effs[i]
            for j, (x1, y1, x2, y2, snet, _hw, _layer) in enumerate(segs):
                if snet == net:
                    continue
                halfw = row[j]
                if (min(x1, x2) > bx1 + halfw or max(x1, x2) < bx0 - halfw or
                        min(y1, y2) > by1 + halfw or max(y1, y2) < by0 - halfw):
                    continue
                d = _seg_to_rect_dist(x1, y1, x2, y2, (bx0, by0, bx1, by1))
                if d < halfw - EPS:
                    by_net[snet] = by_net.get(snet, 0.0) + (halfw - d)
        return by_net

    def seg_penalty(self, ref, cap, x, y, rot):
        """Scalar sum of the per-net track clearance shortfalls (the objective
        term / graze report). See _seg_shortfalls for the per-net breakdown the
        accept gate uses."""
        return sum(self._seg_shortfalls(ref, cap, x, y, rot).values())

    def _pad_shortfalls(self, ref, cap, x, y, rot):
        """PER-FOREIGN-NET component-pad clearance shortfall (#235), keyed by the
        foreign pad's net_id (#441 per-net, mirroring _seg_shortfalls): how far
        each cap pad intrudes inside a different-net pad's clearance keep-out. A
        through-hole foreign pad blocks both sides; an SMD one only its own side.
        Same-net pads are ignored (a shared via / touching same-net copper is
        fine)."""
        by_net: Dict[int, float] = {}
        fpads = self.cap_foreign_pads[ref]
        effs = self._pad_effs(ref, cap)
        if effs is None:
            for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
                for (px0, py0, px1, py1, pnet, pside) in fpads:
                    if pnet == net:
                        continue
                    if pside is not None and pside != cap.side:
                        continue  # SMD pad on the other side
                    gap = _rect_gap((bx0, by0, bx1, by1), (px0, py0, px1, py1))
                    if gap < self.clearance - EPS:
                        by_net[pnet] = by_net.get(pnet, 0.0) + (self.clearance - gap)
            return by_net
        for i, (bx0, by0, bx1, by1, net) in enumerate(cap.pad_rects(x, y, rot)):
            row = effs[i]
            for j, (px0, py0, px1, py1, pnet, pside) in enumerate(fpads):
                if pnet == net:
                    continue
                if pside is not None and pside != cap.side:
                    continue  # SMD pad on the other side
                eff = row[j]
                gap = _rect_gap((bx0, by0, bx1, by1), (px0, py0, px1, py1))
                if gap < eff - EPS:
                    by_net[pnet] = by_net.get(pnet, 0.0) + (eff - gap)
        return by_net

    def pad_penalty(self, ref, cap, x, y, rot):
        """Scalar sum of the per-net foreign-pad shortfalls."""
        return sum(self._pad_shortfalls(ref, cap, x, y, rot).values())

    @staticmethod
    def _worsens_any_net(cand_by_net, seed_by_net):
        """True if the candidate penetrates ANY foreign net beyond its seed
        shortfall (+EPS) -- in particular, penetrates a net that was CLEAR
        (absent from seed_by_net) at the seed. Per-net so relieving net A can
        never pay for a NEW short on net B (#441)."""
        for net, pen in cand_by_net.items():
            if pen > seed_by_net.get(net, 0.0) + EPS:
                return True
        return False

    def attraction(self, cap, x, y, rot):
        """Sum over pads of the distance to the nearest same-net BGA ball,
        clamped to capture_radius (so a pad with no same-net ball within reach
        contributes a flat constant and creates no long-range pull)."""
        total = 0.0
        for (bx0, by0, bx1, by1, net) in cap.pad_rects(x, y, rot):
            balls = self.attract.get(net)
            if not balls:
                continue
            cx, cy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0
            nearest = min(math.hypot(ax - cx, ay - cy) for ax, ay in balls)
            total += min(nearest, self.capture_radius)
        return total

    def _blocked_geom(self, ref, cap, x, y, rot):
        """The cheap, purely geometric hard constraints: board edge and
        introduced/worsened same-side courtyard or mover-pad overlaps."""
        rect = cap.rect(x, y, rot)
        if (rect[0] < self.usable[0] or rect[1] < self.usable[1]
                or rect[2] > self.usable[2] or rect[3] > self.usable[3]):
            return True
        # Real outline / cutout gate (#370 B2): only when the bbox inset is
        # not exact (non-rect outline or cutouts) and this cap can reach one.
        if self._edge_active and self._cap_may_reach_edge(ref, cap) \
                and self._rect_edge_blocked(rect, ref=ref):
            return True
        # only same-side parts/caps within reach can collide (pre-pruned)
        for idx, r in self.cap_static[ref]:
            base = self.base_static.get((ref, idx), 0.0)
            if self._overlap(rect, r) > base + EPS:
                return True
        cand_pads = None
        cand_bbox = None
        for other_ref in self.cap_caps[ref]:
            pair = frozenset((ref, other_ref))
            other = self.caps[other_ref]
            if self._overlap(rect, other.rect()) > \
                    self.base_cap.get(pair, 0.0) + EPS:
                return True
            # no new/worse different-net pad encroachment against another
            # MOVER at its current pose (#275); each accepted move preserves
            # the pairwise seed baseline, so the invariant holds inductively
            # as both parts move.
            # Pad-bbox prescreen: pads are contained in their union bbox, so
            # every pad-pair gap >= the bbox gap; bboxes >= clearance apart
            # means the shortfall is exactly 0 <= base + EPS -- skip the
            # pairwise scan (the dominant cost of the candidate sweep).
            # #725: the prescreen is a SKIP, so it must widen with the pair's
            # requirement -- left at the flat scalar it silently drops exactly
            # the pairs a pad override / netclass / dru rule raises. Bounded by
            # the two movers' OWN maxima, never a board-wide one: this runs per
            # candidate pose.
            if cand_bbox is None:
                cand_bbox = cap.pad_bbox(x, y, rot)
            screen = self.clearance if self._floors is None else max(
                self.clearance, cap.max_floor, other.max_floor)
            if _rect_gap(cand_bbox, other.pad_bbox()) >= screen:
                continue
            if cand_pads is None:
                cand_pads = cap.pad_rects(x, y, rot)
            if _pad_pair_shortfall(cand_pads, other.pad_rects(),
                                   self.clearance,
                                   self._pair_effs(ref, cap, other_ref, other)) > \
                    self.base_cap_pad.get(pair, 0.0) + EPS:
                return True
        return False

    def hard_blocked(self, ref, cap, x, y, rot):
        """True if a placement leaves the board or introduces/worsens a
        same-side courtyard overlap (with a locked part OR another movable
        cap) beyond its baseline, or worsens a foreign track/pad graze past
        its seed baseline. Caps may never overlap each other's footprints,
        so any new cap-cap overlap is rejected outright."""
        if self._blocked_geom(ref, cap, x, y, rot):
            return True
        # #441 per-net: no NEW/worse overlap with ANY foreign-net track -- forbid
        # penetrating a net that was clear at the seed even if the summed shortfall
        # drops (the zynq GND<->NetR2_2 butterfly). Same for foreign component pads.
        if self._worsens_any_net(self._seg_shortfalls(ref, cap, x, y, rot),
                                 self.base_seg.get(ref, {})):
            return True
        if self._worsens_any_net(self._pad_shortfalls(ref, cap, x, y, rot),
                                 self.base_pad.get(ref, {})):
            return True
        # #445: same per-net gate for existing VIAS -- via_penalty alone is a
        # weighted objective, and a big track-graze relief could pay for
        # dropping a pad onto a via (zynq_ad9364 R2 onto DDR3_VREF).
        if self._worsens_any_net(self._via_shortfalls(ref, cap, x, y, rot),
                                 self.base_via.get(ref, {})):
            return True
        return False

    def graze_penalty(self, ref, cap, x, y, rot):
        """Total foreign-copper clearance shortfall for a placement: via
        (#130) + same-side track (#278 PAD-SEGMENT) + component pad (#275
        PAD-PAD). Anything positive is a shipped DRC violation, so all three
        are violations to FIX, not just baselines to preserve."""
        return (self.via_penalty(cap, x, y, rot, self.cap_vias[ref], ref=ref)
                + self.seg_penalty(ref, cap, x, y, rot)
                + self.pad_penalty(ref, cap, x, y, rot))

    def cost(self, ref, cap, x, y, rot):
        if self._blocked_geom(ref, cap, x, y, rot):
            return float('inf')
        seg_by_net = self._seg_shortfalls(ref, cap, x, y, rot)
        pad_by_net = self._pad_shortfalls(ref, cap, x, y, rot)
        via_by_net = self._via_shortfalls(ref, cap, x, y, rot)
        # #441/#445 per-net accept gate (mirror hard_blocked): reject a move
        # that penetrates any foreign net -- track, pad, or VIA -- beyond its
        # seed, even if the total improves.
        if (self._worsens_any_net(seg_by_net, self.base_seg.get(ref, {}))
                or self._worsens_any_net(pad_by_net, self.base_pad.get(ref, {}))
                or self._worsens_any_net(via_by_net, self.base_via.get(ref, {}))):
            return float('inf')
        seg_pen = sum(seg_by_net.values())
        pad_pen = sum(pad_by_net.values())
        disp = math.hypot(x - cap.seed_x, y - cap.seed_y)
        graze = (sum(via_by_net.values()) + seg_pen + pad_pen)
        return (VIA_WEIGHT * graze
                + ATTRACT_WEIGHT * self.attraction(cap, x, y, rot)
                + DISPLACEMENT_WEIGHT * disp)


def repair_fanout_clearance(pcb_data: PCBData, pcb_file: str,
                            clearance: Optional[float] = None,
                            netclass_ceiling: Optional[float] = None,
                            grid_step: float = 0.1,
                            board_edge_clearance: Optional[float] = None,
                            near_margin: float = 1.0,
                            capture_radius: float = 2.0,
                            default_via_size: float = 0.3,
                            step: float = 0.2,
                            max_displacement: float = 2.0,
                            max_displacement_cap: float = 3.0,
                            displacement_growth: float = 1.5,
                            allow_rotations: bool = True,
                            cap_prefix: str = "C,R,FB",
                            lock_refs: Optional[List[str]] = None,
                            max_passes: int = 30,
                            via_clear_fallback: bool = True,
                            verbose: bool = False,
                            on_move=None,
                            # progress_callback(current, total, label): the
                            # candidate-position sweep per cap x up to 30
                            # passes is the slow part, so each cap visit
                            # reports (GUI status line). None = silent.
                            progress_callback=None) -> Dict:
    """Nudge near-BGA decoupling caps off foreign-net fanout copper (vias
    #130, escape tracks #278, component pads #275) and toward same-net balls.
    Run AFTER bga_fanout.py.

    Returns a dict with 'placements' (list of {reference,new_x,new_y,
    new_rotation} for moved caps), 'resolved', 'unresolved', 'via_resolved',
    'regrazed', 'bga_refs', 'via_moves', 'new_segments', 'required' and
    'clearance_notes'.

    'resolved' and 'unresolved' are graded together, from ONE board state, at
    the very END of the pass (#746) -- so they are disjoint, and 'resolved'
    credits the via-nudge for a cap only the nudge could free. Read them as:

      resolved     was grazing at the SEED and is clean now. A subset of the
                   initial violators, so len(resolved) is at most the
                   denominator of the "resolved R/V" count printed below.
      unresolved   is grazing NOW, whatever it was at the seed. NOT a subset
                   of the initial violators: copper this pass itself drew can
                   put a cap here that started clean.
      via_resolved the caps in 'resolved' that the cap-move descent could not
                   clean and the via-nudge did -- the last resort's credit,
                   which before #746 landed in neither list.
      regrazed     the caps that were CLEAN before the nudge and are grazing
                   after it: this pass broke them. Keyed on the transition,
                   NOT on membership of the seed violators -- a cap that was
                   never broken at all is the case the 'unresolved' line above
                   exists to describe, and it must not be the one case with no
                   cause named. A subset of 'unresolved', disjoint from
                   'resolved'.

    Two things to know before treating 'regrazed' as a shipped DRC violation.
    It is graded by THIS module, which deliberately over-blocks a track on a
    layer the board never declared (see test_736's
    TestTheUnknownLayerOverBlockIsDELIBERATE), so check_drc can grade the same
    pair clean -- 'unresolved' has had that property since #736; what is new
    is a line asserting a CAUSE, which can send an operator hunting a phantom.
    And the cause it asserts is connector copper rather than the relocated via
    because valid_via_pos's cap gate is strictly stronger than via_penalty's:
    same rects, same radius, same resolver, both over post-descent poses, and
    no layer gate on either -- so a landing that would newly graze a cap is
    refused before it happens. The set difference below does not re-check
    that, which is why it is written down here.

    The two early returns below carry neither 'via_resolved' nor 'regrazed',
    exactly as they carry no 'via_moves' / 'new_segments' -- see the note there.

    via_clear_fallback (#213): when True (default), any cap the normal cost
    descent leaves grazing foreign copper is jumped to the nearest position
    that fully clears it (still respecting every hard clearance). It is
    deliberately NOT exposed on the CLI / GUI -- flip this argument in code to
    disable.

    board_edge_clearance (#733) defaults to None meaning "resolve it" -- see
    resolve_cap_edge_clearance: the board's own min_copper_edge_clearance when it
    declares MORE room than CAP_EDGE_CLEARANCE, else that 0.55. A value passed in
    is honoured exactly as given, in both directions.

    on_move, if given, is a callback invoked with the internal _Repair state
    once at the seed placement and again after every accepted cap move. It is
    used purely for visualization (animate_fanout_clearance.py) and has no
    effect on the result.
    """
    extra_locked: Set[str] = set()
    if lock_refs:
        import fnmatch
        for ref in pcb_data.footprints:
            if any(fnmatch.fnmatch(ref, pat) for pat in lock_refs):
                extra_locked.add(ref)

    # #733: ONE board-edge margin for every front end. Resolved HERE, in the
    # shared engine rather than in a CLI main(), because the GUI plugin
    # re-implements the main() layer and would otherwise keep silently taking
    # the signature default whatever the board declares. Printed for the same
    # reason the #725 clearance disclosure is printed from the engine: both
    # fronts inherit it.
    #
    # UNCONDITIONAL, including when the caller named a value. The number governs
    # where cap copper may sit relative to Edge.Cuts, and an operator reading a
    # transcript should never have to know which of three fronts invoked this to
    # know which margin it used -- printing only the resolved case would disclose
    # exactly the branch that needs no explaining.
    # #768: `clearance` defaults to None meaning "resolve it from the board",
    # exactly as board_edge_clearance has since #733 -- and for the same reason,
    # that this engine has three front ends and only one of them ever had a
    # resolution step. Printed unconditionally, for the reason spelled above the
    # edge margin: an operator reading a transcript should not have to know
    # which front invoked the pass to know what it priced at.
    # NOTE FOR DIRECT CALLERS, and it is a behaviour change: `clearance` used to
    # default to a flat 0.2 and be spent as given. It is now resolved, so an
    # explicit `clearance=X` becomes min(the board's Default class, X) -- the
    # GIVEN branch, because a caller that names a number is naming a ceiling.
    # Pass None to get the board's own class. All three in-repo fronts were
    # updated with this; an out-of-tree caller passing a number ABOVE the
    # board's Default class will now price at the class instead.
    clearance, _clr_src = resolve_pair_clearance(pcb_file, clearance)
    print(f"  cap pair clearance: {clearance}mm ({_clr_src})")
    _fab_clr, _fab_n = fab_pair_clearance_floor(pcb_data)
    if clearance < _fab_clr - 1e-9:
        # Disclosed, never applied -- see fab_pair_clearance_floor. Worth saying
        # out loud because the netclass tier used to raise a too-small value
        # back up and a ceiling stops it doing that.
        print(f"  NOTE: {clearance}mm is below the {_fab_n}-layer fab "
              f"clearance floor {_fab_clr:g}mm. Pricing at it anyway, because "
              f"check_drc grades at it too -- but nothing can etch that.")

    board_edge_clearance, _edge_src = resolve_cap_edge_clearance(
        pcb_file, board_edge_clearance)
    print(f"  board-edge margin for caps: {board_edge_clearance}mm "
          f"({_edge_src})")

    st = _Repair(pcb_data, pcb_file, clearance, grid_step,
                 board_edge_clearance, near_margin, capture_radius,
                 default_via_size, cap_prefix, extra_locked,
                 max_displacement_cap=max_displacement_cap,
                 netclass_ceiling=netclass_ceiling)

    print(f"BGAs: {', '.join(st.bga_refs) or '(none)'}  "
          f"fanout vias: {len(st.vias)}  "
          f"movable near-BGA caps: {len(st.caps)}")
    # #725: the early returns carry 'required' / 'clearance_notes' too, so a
    # caller does not have to special-case a no-op board for them. (They still
    # omit 'via_moves' / 'new_segments' -- and, since #746, 'via_resolved' /
    # 'regrazed' -- as they always have: every caller reads the dict with .get,
    # and all four describe a via-nudge that provably did not happen here.)
    if not st.vias:
        print("No vias on the board - run this AFTER bga_fanout.py.")
        return {'placements': [], 'resolved': [], 'unresolved': [],
                'bga_refs': st.bga_refs, 'required': [],
                'clearance_notes': list(st.clearance_notes)}
    if not st.caps:
        print("No movable caps near a BGA - nothing to do.")
        return {'placements': [], 'resolved': [], 'unresolved': [],
                'bga_refs': st.bga_refs, 'required': [],
                'clearance_notes': list(st.clearance_notes)}

    # Initial violators: any foreign-copper clearance shortfall (via #130,
    # track #278, pad #275) is a shipped DRC violation to fix.
    violators0 = [r for r, c in st.caps.items()
                  if st.graze_penalty(r, c, c.x, c.y, c.rot) > EPS]
    print(f"Caps initially grazing foreign copper (via/track/pad): "
          f"{len(violators0)}"
          + (f" ({', '.join(sorted(violators0))})" if violators0 else ""))

    # LOCKED parts are excluded from moving by design, but a foreign via inside
    # a locked pad's keep-out is then unfixable here -- surface it instead of
    # silently reporting a clean pass (#254: locked back-side cap under a BGA
    # via-in-pad). The fanout avoids locked copper for NEW vias; this warning
    # catches boards fanned before that fix or vias from other sources.
    locked_hits = []
    for ref in sorted(st.locked_refs):
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        for p in fp.pads:
            # #725: an np_thru_hole pad lists *.Cu and carries no copper. It
            # is still WARNED ABOUT -- upstream did, foreign_pads does, and a
            # via inside a 1.7mm mounting hole is exactly what this warning is
            # for -- but it is graded FLAT: its keep-clear override is not
            # copper and must not raise the pair. Dropping it outright (the
            # first version of this fix) removed real rows on boards that
            # declare nothing at all.
            if not any(str(l).endswith('.Cu') for l in p.layers):
                continue
            _carries = _pad_carries_copper(p)
            # ...and the RECT gets the same rect_rotation inflation _Cap and
            # foreign_pads use. This was the one pad-geometry site in the file
            # built from raw size_x/size_y, so a tilted locked pad under-blocked
            # by (sqrt(2)-1)*half along the board axes -- 0.14mm on glasgow's
            # 45-degree R9. Converting the clearance term and leaving the shape
            # would be exactly the half-conversion this change is about.
            ex, ey = pad_half_extents(p)
            rect = (p.global_x - ex, p.global_y - ey,
                    p.global_x + ex, p.global_y + ey)
            # Graded at the pair's REAL requirement, like everything else.
            # `keepout` is the prune over-reach, so re-form the pair's keep-out
            # from the via's radius. A locked fiducial's keep-clear override is
            # exactly the case this warning exists to surface, and priced flat
            # it under-reported it.
            pfl = st.pad_floor(p) if _carries else None
            _radii = st._via_radius_by_id
            for t in st.vias:
                vx, vy, vnet, keepout = t
                if vnet == p.net_id:
                    continue
                # #732: the radius comes from the MAP, never from
                # `keepout - _item_reach(...)`. That subtraction is the exact
                # arithmetic re-derivation the map's own comment forbids, and
                # it re-prices a tuple a test assigned wholesale with its own
                # keep-out convention. Absent from the map -> the tuple's own
                # keep-out slot verbatim, which is what via_penalty's flat path
                # and _via_effs already do with such a tuple. Numerically
                # identical for every via __init__ built, so this is inert on
                # every real board (the #732 test file measures that).
                _rec = _radii.get(id(t))
                ko = (keepout if _rec is None
                      else _rec[1] + st.via_required(pfl, vnet))
                if _point_to_rect_dist(vx, vy, rect) < ko - EPS:
                    locked_hits.append((ref, p.pad_number, vx, vy))
    if locked_hits:
        print(f"WARNING: {len(locked_hits)} foreign via(s) inside LOCKED parts' pad "
              f"clearance (cannot move a locked part):")
        for ref, pnum, vx, vy in locked_hits[:10]:
            print(f"    {ref}.{pnum} <-> via at ({vx:.2f},{vy:.2f})")

    if on_move is not None:
        on_move(st)  # seed frame

    budget = {r: max_displacement for r in st.caps}
    rotate = {r: False for r in st.caps}
    # process worst violators first
    order = sorted(st.caps, key=lambda r: st.graze_penalty(
        r, st.caps[r], st.caps[r].x, st.caps[r].y, st.caps[r].rot),
        reverse=True)

    # Pause the CYCLIC collector for the sweep. The candidate loops allocate
    # short-lived tuples/lists at a rate that fires gen-2 collections, and
    # each of those scans the ENTIRE process heap -- in a live GUI chain
    # session (4 prior steps of boards/fills/engines resident) the identical
    # sweep measured 5.9x slower than in a fresh process (mez_rx: 184s vs
    # 31s, same call counts) purely from GC pressure. The sweep's garbage is
    # acyclic, so reference counting reclaims it either way.
    import gc
    _gc_was_enabled = gc.isenabled()
    gc.disable()

    try:
        for pass_num in range(1, max_passes + 1):
            moves = 0
            for _oi, ref in enumerate(order):
                if progress_callback:
                    progress_callback(
                        _oi + 1, len(order),
                        f"Cap optimize pass {pass_num}: {ref}")
                cap = st.caps[ref]
                current = st.cost(ref, cap, cap.x, cap.y, cap.rot)
                rots = ROTATIONS if (allow_rotations and rotate[ref]) \
                    else [cap.rot]
                best = (current, cap.x, cap.y, cap.rot)
                for cx, cy in _candidate_positions(cap, budget[ref], step,
                                                   grid_step):
                    for rot in rots:
                        if (cx, cy, rot) == (cap.x, cap.y, cap.rot):
                            continue
                        c = st.cost(ref, cap, cx, cy, rot)
                        if c < best[0] - EPS:
                            best = (c, cx, cy, rot)
                if best[0] < current - EPS:
                    cap.x, cap.y, cap.rot = best[1], best[2], best[3]
                    moves += 1
                    if on_move is not None:
                        on_move(st)
                    if verbose:
                        print(f"  pass {pass_num}: {ref} -> "
                              f"({cap.x:.3f},{cap.y:.3f}) rot {cap.rot:g} "
                              f"cost {best[0]:.3f}")

            residual = {r: st.graze_penalty(r, st.caps[r], st.caps[r].x,
                                            st.caps[r].y, st.caps[r].rot)
                        for r in st.caps}
            still = [r for r, p in residual.items() if p > EPS]
            if not still:
                print(f"All foreign-copper grazes cleared after {pass_num} "
                      f"pass(es).")
                break
            if moves == 0:
                # stuck: escalate budget / rotations for the remaining
                # violators
                grown = False
                for r in still:
                    if not rotate[r] and allow_rotations:
                        rotate[r] = True
                        grown = True
                    elif budget[r] < max_displacement_cap - EPS:
                        budget[r] = min(budget[r] * displacement_growth,
                                        max_displacement_cap)
                        grown = True
                if not grown:
                    print(f"Stuck with {len(still)} cap(s) still grazing "
                          f"foreign copper (at the displacement cap): "
                          f"{', '.join(sorted(still))}")
                    break
                if verbose:
                    print(f"  escalating budget/rotation for {len(still)} "
                          f"cap(s)")

        # Fallback (#213): a cap may still graze a foreign via because the
        # soft cost (same-net attraction + displacement) judged the tiny
        # penetration cheaper than a clear-but-distant spot, so the greedy
        # descent never relocates it. A shipped PAD-VIA short is worse than a
        # displaced decap, so for any cap still in via-conflict, jump it to
        # the best position (within the full displacement budget) that FULLY
        # clears every foreign via -- cost() still rejects any hard-constraint
        # violation (board edge, courtyard, foreign track/pad #235), so this
        # can never introduce a new short/overlap; it only overrides the soft
        # trade-off. If no clean via-clearing spot exists the cap is left
        # as-is and reported unresolved (needs a via re-drop, not a move).
        if via_clear_fallback:
            stuck = [r for r in st.caps
                     if st.graze_penalty(r, st.caps[r], st.caps[r].x,
                                         st.caps[r].y,
                                         st.caps[r].rot) > EPS]
            rots_all = ROTATIONS if allow_rotations else None
            for _fi, ref in enumerate(stuck):
                if progress_callback:
                    progress_callback(
                        _fi + 1, len(stuck),
                        f"Cap optimize: via-clear fallback for {ref}")
                cap = st.caps[ref]
                rots = rots_all if rots_all is not None else [cap.rot]
                best = None  # (cost, x, y, rot)
                for cx, cy in _candidate_positions(cap, max_displacement_cap,
                                                   step, grid_step):
                    for rot in rots:
                        if st.graze_penalty(ref, cap, cx, cy, rot) > EPS:
                            continue
                        c = st.cost(ref, cap, cx, cy, rot)  # inf if blocked
                        if c == float('inf'):
                            continue
                        if best is None or c < best[0] - EPS:
                            best = (c, cx, cy, rot)
                if best is not None:
                    cap.x, cap.y, cap.rot = best[1], best[2], best[3]
                    disp = math.hypot(cap.x - cap.seed_x, cap.y - cap.seed_y)
                    if verbose:
                        print(f"  fallback: {ref} relocated to clear foreign "
                              f"copper at disp {disp:.2f}mm -> "
                              f"({cap.x:.3f},{cap.y:.3f}) rot {cap.rot:g}")
    finally:
        if _gc_was_enabled:
            gc.enable()

    placements = []
    for ref, cap in st.caps.items():
        moved = (abs(cap.x - cap.seed_x) > EPS or abs(cap.y - cap.seed_y) > EPS
                 or abs(cap.rot - cap.seed_rot) > EPS)
        if moved:
            placements.append({'reference': ref, 'new_x': cap.x,
                               'new_y': cap.y, 'new_rotation': cap.rot})

    # #746: ONE grader, so `resolved` and `unresolved` can never be read off
    # two different boards. They were: `resolved` was built here and never
    # refreshed, while `unresolved` was rebuilt after the via-nudge below --
    # so a cap the NUDGE freed was grazing when the credit was computed and
    # clean when the debit was, and appeared in neither list.
    #
    # Disjoint by construction, and the order of `st.caps` is preserved in
    # both: a ref that grazes NOW is unresolved and cannot also be credited;
    # only a still-clean ref that was a SEED violator is. Note the asymmetry
    # is deliberate -- `resolved` is gated on `violators0` (you cannot resolve
    # what was never broken) and `unresolved` is not (copper this pass drew
    # can break a cap that started clean, which is #736's whole finding).
    def _grade():
        res, unres = [], []
        for ref, cap in st.caps.items():
            if st.graze_penalty(ref, cap, cap.x, cap.y, cap.rot) > EPS:
                unres.append(ref)
            elif ref in violators0:
                res.append(ref)
        return res, unres

    resolved, unresolved = _grade()
    via_resolved: List[str] = []
    regrazed: List[str] = []

    # Last resort (#313): a cap still grazing at the displacement cap is
    # usually BOXED (no clear cap position exists) -- move the offending
    # fanout via instead, dragging its attached escape-segment ends.
    via_moves, new_segs = ([], [])
    if unresolved:
        via_moves, new_segs = nudge_vias_for_unresolved(st, pcb_data, clearance)
        if via_moves:
            # #736: absorb the connector tracks the nudger just DREW. They went
            # into pcb_data.segments, but st.segments / st.cap_segs /
            # st._seg_floor_by_id were snapshotted in _Repair.__init__ before
            # they existed -- so the re-grade below, and required_rows' seed +
            # final disclosure, read a track view missing exactly the copper
            # this pass is responsible for, and a cap could be reported
            # RESOLVED with one of its pads inside a connector's keep-out.
            #
            # The nudger's own connector_clear gate does NOT already close
            # this. On the connector's OWN layer the two agree exactly -- same
            # two floors through the same resolver, and connector_clear is the
            # stricter by EPS -- so an accepted same-layer connector can never
            # be charged here. It is the OTHER layers that diverge, in two
            # ways: connector_clear compares against the cap FOOTPRINT's side
            # (#738), so an inner-layer or back-side connector is never tested
            # against a THROUGH-HOLE cap pad whose copper spans every layer;
            # and a connector on a layer this board never declared is skipped
            # there while _seg_shares charges it here, which is a window #738's
            # own proposed fix does not close. Either way the gate that drew
            # the copper is not the gate that grades it, and the summary the
            # operator reads comes from this one.
            #
            # THE GUARD STAYS `via_moves` and gains no `or new_segs`: a
            # connector dict is appended only inside the same commit block that
            # appends the via move, with no continue or break between, so a
            # non-empty new_segs IMPLIES a non-empty via_moves and the disjunct
            # could never fire. No inner `if new_segs:` either -- the empty
            # case returns immediately inside register_new_segments.
            #
            # The CONVERSE is false and this is not an equivalence: a via whose
            # conn_layers is empty -- no same-net copper terminating in its
            # body, no containing same-net pad, no same-net zone -- relocates
            # with no connector at all (and, worth knowing separately, with no
            # connector validation, since `all(...)` over an empty dict is
            # vacuously True). That grows via_moves and not new_segs.
            # (writer.py does spell `if via_moves or new_segments:`, and that
            # is right THERE: it wraps two independent text rewrites and is
            # defending against a caller handing it lists it did not produce.)
            st.register_new_segments(new_segs)
            # #747: ...and the vias it MOVED, through the method that owns
            # the tuple shape, the carried keep-out and the radius map. Under
            # the same guard, and for the same reason, as the line above: the
            # nudger reports rather than mutates, so a real _Repair applies
            # the report here. Must precede the refresh below, which reads
            # the view this repairs.
            st.relocate_vias(via_moves)
            # #775: ...and re-prune the per-cap via view against the
            # list the line above just repaired, before anything
            # re-grades it. This was a wholesale reassignment onto the
            # whole-board list -- a safe SUPERSET of the pruned view (it
            # is the PRUNE that is exact), which therefore graded the same
            # and paid an eff-matrix rebuild of n_pads x n_ALL_vias per
            # cap. Must FOLLOW the relocation, which is what it reads.
            st.refresh_cap_vias()
            # base_seg / base_pad / base_via are deliberately NOT re-seeded --
            # exactly as base_via is left alone by the line above, though via
            # positions changed there too. They are read only by cost() /
            # hard_blocked, neither of which runs past this point; and a
            # baseline means "the state at the SEED", which copper this pass
            # CREATED is not part of. Folding it in would license a later move
            # to sit on the pass's own connector for free -- the #441
            # GND<->NetR2_2 failure in miniature, and silent. The consequence
            # to know: from here st.base_seg and st.cap_segs describe different
            # boards, which is what "baseline" means rather than a defect.
            #
            # #746: BOTH lists are re-graded here, from the one view the two
            # refreshes above just established. This is what closes the gap
            # this comment used to declare open ("`resolved` is computed
            # BEFORE the nudge and is not refreshed"), in both its directions:
            # a cap the NUDGE freed is now credited instead of vanishing from
            # both lists, and a cap the SWEEP cleaned that a connector then
            # grazes now leaves `resolved` instead of sitting in both.
            #
            # Both deltas are taken against the PRE-NUDGE lists and not
            # recomputed from a penalty, because "who fixed it" and "who broke
            # it" are facts about the transition, not about either end state.
            # Set semantics on lists: `st.caps` has one entry per ref, so no
            # list here carries duplicates and `in` is exact.
            #
            # `regrazed` keys on `was_grazing`, NOT on `swept`: a review of the
            # first version caught that spelling it `[r for r in swept if r not
            # in resolved]` can only ever name a cap that was ALSO a seed
            # violator -- so a cap that started perfectly clean and was broken
            # by this pass's own connector was the one case that got no cause
            # named, which is exactly the case the docstring above says exists.
            swept = resolved
            was_grazing = set(unresolved)
            resolved, unresolved = _grade()
            via_resolved = [r for r in resolved if r not in swept]
            regrazed = [r for r in unresolved if r not in was_grazing]

    # #746: the credit clause says WHO freed the cap, because `resolved` now
    # spans both mechanisms and a bare count cannot distinguish them. Both
    # clauses are suppressed when empty, so a run that never reaches the
    # via-nudge prints exactly the line it printed before.
    _credit = (f" ({len(via_resolved)} freed by via-nudge)"
               if via_resolved else "")
    print(f"Moved {len(placements)} cap(s); resolved {len(resolved)}/"
          f"{len(violators0)} initial violations{_credit}; "
          f"{len(unresolved)} unresolved.")
    if unresolved:
        print(f"  Unresolved (need manual attention): {', '.join(sorted(unresolved))}")
    if regrazed:
        print(f"  Re-grazed by this pass's own connector copper: "
              f"{', '.join(sorted(regrazed))}")
    # #725: disclose what was graded ABOVE the flat --clearance, and why.
    # Printed from the ENGINE so the CLI and the GUI plugin both inherit it.
    required = st.required_rows({n.net_id: n.name for n in pcb_data.nets.values()})
    _clause = format_required_clause({'required': required})
    if _clause:
        print(f"  above the {clearance}mm floor: {_clause}")
    for _note in st.clearance_notes:
        print(f"  pad clearance: {_note}")

    return {'placements': placements, 'resolved': resolved,
            'unresolved': unresolved, 'bga_refs': st.bga_refs,
            'via_moves': via_moves, 'new_segments': new_segs,
            'via_resolved': via_resolved, 'regrazed': regrazed,
            'required': required, 'clearance_notes': list(st.clearance_notes)}


def _point_in_poly(px, py, poly) -> bool:
    """Ray-cast point-in-polygon for zone containment (#313 pour-tie reconnect)."""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > py) != (yj > py)) and \
           (px < (xj - xi) * (py - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def nudge_vias_for_unresolved(st, pcb_data, clearance: float,
                              max_shift: float = 0.6):
    """Last resort for caps still grazing at the displacement cap (#313): the
    cap is boxed (glasgow C77: no clear cap position exists within 2mm), so
    move the OFFENDING fanout via a fraction of a millimetre instead. The
    original attached segments are NOT touched; electrical continuity is
    restored by NEW short connector segments from the via's new position back
    to the fanout stub start (the via's old position) on every layer that had
    same-net copper terminating there (plus the pad's copper layer for a
    via-in-pad). mm-exact validation throughout. Returns
    (via_moves, new_segments) for the writer:
      via_moves    = [(old_x, old_y, via dict {'x','y','size',
                       'drill','layers','net_id','tenting_attrs'})]
      new_segments = [segment dicts {'start','end','width','layer','net_id'}]
    """
    via_moves, new_segments = [], []

    unresolved = [r for r in st.caps
                  if st.graze_penalty(r, st.caps[r], st.caps[r].x,
                                      st.caps[r].y, st.caps[r].rot) > EPS]
    if not unresolved:
        return via_moves, new_segments

    # #756: the two DRILL-to-drill floors, board-first and RAISE-ONLY. They
    # were flat literals here -- `H2H_VIA = 0.2`, `H2H_PAD = 0.45` -- so on a
    # board declaring `min_hole_to_hole` ABOVE the JLC floor this pass parked a
    # relocated via at 0.20 while check_drc graded the same pair at the
    # declared value (`_pin_up`, check_drc.py:3686, feeding BOTH drill arms --
    # via/via :2605, pad/via :2667). See resolve_drill_floors.
    #
    # NOT THE ONLY SUCH FLOOR, and an earlier draft of this comment claimed it
    # was. A review refuted that by measurement: the CONNECTOR's copper-to-hole
    # gate is flat at `max(clearance, NPTH_TO_TRACK_CLEARANCE)` while check_drc
    # `_pin_up`s `hole_clearance` from the board's `min_hole_clearance`
    # (check_drc.py:3697) and both its copper-to-hole arms add it -- so on the
    # #617 rig's own declaring board, the connector this pass emits grades as a
    # 0.030mm copper-to-hole violation. That floor stays flat here, by #617's
    # measurement, and it is a KNOWN gap rather than a refutation of this one.
    # What is true of the via/via drill gate is narrower: it was below its
    # grader AND, unlike the connector's, it can be raised without cost --
    # because of the ladder below.
    #
    # BELOW the early return on purpose: a call with nothing unresolved now
    # reads no project and prints nothing.
    #
    # THE NAMES AND BOTH GATE EXPRESSIONS ARE UNCHANGED, so
    # tests/mutate_750.py's rows keep APPLYING: a `str.replace` whose needle
    # stops matching returns the file unchanged, and the battery then reports
    # SURVIVED for a mutation it never made.
    #
    # WHICH IS WHY THIS COMMENT DOES NOT QUOTE THEM. An earlier draft spelled
    # both gate expressions out here to explain the point, and thereby took
    # each of mutate_750's anchors from one occurrence to two, so four of its
    # rows stopped testing anything. Exactly the failure the paragraph above
    # describes, committed by the paragraph itself. Refer to the gates by
    # name and by line, never by their text.
    #
    # NOT SILENT, and an earlier version of this comment said it was: those
    # rows report BROKEN and `mutate_750.py:251` is
    # `return 1 if (wrong or broken) else 0`, so the battery exits NON-zero
    # and names them. The harm is that four rows tested nothing until someone
    # ran it, not that nothing would have said so. A fact-check caught the
    # overstatement by running the four rows.
    #
    # THIS IS NOT THE RAISE #617 REFUSED, and the two do not touch the same
    # gate. #617 measured the CONNECTOR's COPPER-to-hole floor on a rig
    # carrying ONE via -- the self-skip at the top of the board_via_drills
    # loop fires, so the via/via gate never executes there at all -- and that
    # rig declares `min_hole_clearance`,
    # a different key. Read the note beside `npth_clr` below, and the amended
    # asymmetry note at _Repair.__init__, before assuming this erodes either.
    #
    # MEASURED before it shipped, on a rig built for it (tests/test_756):
    # a same-net neighbour parked so the drill gate alone decides the landing,
    # board declaring 0.25. At the shipped 0.6mm budget every row KEPT its move
    # and its connector -- the landing simply moved to one that clears the
    # declared floor (drill gap 0.2100 -> 0.2615, 0.2400 -> 0.2659). Squeezed
    # to a 0.15mm budget one row DID lose its repair, and it is the 0.2100 row
    # -- the landing check_drc flags (overlap 0.0400 against its 0.0125
    # tolerance). The repair given up is one that shipped flagged copper.
    #
    # DISCLOSED: check_drc forgives 5% (`hole_to_hole_clearance * 0.05`,
    # :2603/:2664), so a landing in [0.95*D, D) is refused here and graded
    # clean there. That band is this repo's grading margin, not a fab rule --
    # kicad-cli enforces min_hole_to_hole with no forgiveness, and check_drc
    # itself warns "Route at <board_val> too, or the grade will not match what
    # was routed". Emitting into a band our own grader forgives and KiCad does
    # not is that warning reached from the emitting side. So charge D. Measured
    # above: the 0.2400 row, which sits inside that band at D=0.25, keeps its
    # repair even at the squeezed budget.
    _h2h_via, _h2h_pad, _h2h_decl, _h2h_src, _fab_via, _fab_pad = \
        resolve_drill_floors(pcb_data)

    # THE LADDER, and it is the whole reason this is safe. Rung 1 is what the
    # board asks for; rung 2 is the fab floor, i.e. EXACTLY what shipped before
    # #756. The sweep below tries rung 1 in full and only descends when it
    # returns nothing, so the pass CANNOT lose a repair it used to make -- the
    # fallback is the old behaviour, unchanged, reached by construction rather
    # than by argument.
    #
    # That matters because the plain raise was measured and it DID cost
    # repairs. A review swept 8673 configurations of this pass's own rig shape
    # (cap bar below, offending via, same-net neighbour above) at the shipped
    # 0.6mm budget: 625 lost the repair, and 13 of those had a flat landing
    # `check_drc` grades CLEAN -- the raise printing "no clear spot" while
    # leaving a violation the flat floor had CURED. One measured case: flat
    # relocated the barrel and graded 0 violations; raised abandoned it and
    # graded 2, including the 0.220mm hole-to-hole the move had been fixing.
    # The mechanism is the search order -- r ascending, then angle, FIRST
    # accept wins -- so the accepted landing is the WORST legal one, and when
    # the escape is bounded the whole reachable set can sit inside the band.
    #
    # The ladder dissolves that class rather than disclosing it. It also
    # retires the objection this change had to answer at all: the pass is no
    # longer ALL-OR-NOTHING for this floor, which is the property
    # `obstacle_map.resolve_hole_clearance` and `_Repair.__init__` both warn
    # about, so the doctrine is not bent -- it stops applying.
    #
    # Rung 2 is omitted when it would be identical, so a board that declares
    # nothing sweeps exactly once and is byte-identical in work as well as in
    # result.
    drill_ladder = [(_h2h_via, _h2h_pad)]
    if (_h2h_via, _h2h_pad) != (_fab_via, _fab_pad):
        drill_ladder.append((_fab_via, _fab_pad))

    if _h2h_src == 'board constraint' and \
            (_h2h_via > _fab_via or _h2h_pad > _fab_pad):
        # The board raised at least one of the two. Name BOTH resolved floors
        # and say which of them the board actually moved, because on a typical
        # declaration (0.20 < d <= 0.45) only the via one moves and the pad one
        # is still the fab number -- an earlier draft printed both under "from
        # the board's own min_hole_to_hole" and a review called that out as
        # attributing a fab floor to the board.
        _moved = ' and '.join(
            [n for n, a, b in (('via-hole', _h2h_via, _fab_via),
                               ('pad-hole', _h2h_pad, _fab_pad)) if a > b])
        print(f"  via-nudge drill floors: via-hole {_h2h_via:g}mm, pad-hole "
              f"{_h2h_pad:g}mm; the board's own min_hole_to_hole "
              f"{_h2h_decl:g}mm raised {_moved} (fab floor "
              f"{_fab_via:g}/{_fab_pad:g}), and is PREFERRED -- the fab floor "
              f"is still tried if nothing clears at it")
    elif _h2h_src == 'board constraint' and _h2h_decl < _fab_via:
        # Against the RESOLVED fab term, not the packaged default. A review
        # measured the difference: with an overrides file raising the fab
        # hole-to-hole to 0.30 and a board declaring 0.25, the old spelling
        # took the `if` branch and announced 0.30 as coming "from the board's
        # own min_hole_to_hole" -- the fab override had won and the board had
        # lost, and the transcript said the opposite. Never SILENTLY relaxed
        # either: a board file cannot lower a fab floor without saying so.
        print(f"  Board min_hole_to_hole {_h2h_decl:g}mm is below the "
              f"{_fab_via:g}mm fab hole-to-hole floor; using "
              f"{_fab_via:g}/{_fab_pad:g}.")
    # Printed only when the number is NOT the packaged default, which is the
    # qfn/bga convention and deliberately not `resolve_cap_edge_clearance`'s
    # unconditional one a few hundred lines above. Both conventions live in
    # this module; the divergence is chosen, not overlooked. The reason is that
    # this line fires inside a last-resort repair that most runs never reach,
    # so an unconditional print would be noise on every ordinary board.

    # Board edge / outline + NPTH floors (#370 B3): the mover has a 0.6mm
    # budget but validated candidates against copper only -- a via or its new
    # connector could be pushed off the board / into a cutout, and connectors
    # had no NPTH-hole test at all.
    # `_seg_seg_dist_coords` (#730) is the segment-to-segment measure the two
    # override gates use, and it is the SHARED helper -- geometry_utils',
    # which check_drc re-exports and placement/legality.py already calls at
    # three sites -- rather than a fifth copy. Emphatically NOT this module's
    # own `_segs_cross` shape: that snaps an orientation of 0 to "crossing", so
    # a segment whose endpoint lies exactly ON the other's line measures 0.0,
    # where `_seg_capsule_axis_dist` -- the kernel the two SIBLING hole gates
    # measure with -- uses a strict `o1*o2 < 0` and falls through to the
    # endpoint minimum. All four hole gates must agree about touching cases.
    # Measured equal to that kernel over ~800k fuzz cases: max |delta| 2.8e-14
    # at board-scale magnitudes, 2.3e-10 at |coord| ~ 1e6 mm, both many orders
    # below the gates' own 1e-4.
    from check_drc import (board_edge_geometry, _point_on_board,
                           _point_to_rings_distance, _segment_to_rings_distance,
                           point_to_pad_distance, _pad_has_no_copper,
                           segment_to_rect_distance, _seg_seg_dist_coords)
    from kicad_parser import pad_drill_capsule
    from routing_utils import into_pad_frame_point
    from single_ended_routing import _seg_foreign_hole_dist
    edge_rings, edge_outer, edge_cutouts = board_edge_geometry(
        getattr(pcb_data, 'board_info', None))
    bounds = getattr(getattr(pcb_data, 'board_info', None), 'board_bounds', None)
    # #617 deliberately leaves this at the flat fab floor. The mover searches a
    # 0.6 mm budget for a spot where BOTH the relocated via and its connector
    # back to the stub validate; raising the connector's hole floor does not
    # relocate the via somewhere better, it makes the whole search return "no
    # clear spot" and the #130 pad-via graze the pass exists to fix persists.
    # Measured on the #370-B3 harness with a declared 0.25: raised -> 0 moves,
    # 0 connectors; flat -> 1 move, 1 connector.
    #
    # #730 RE-MEASURED that, because it is the reason a whole site was dropped
    # rather than fixed, and a four-month-old number carrying that much weight
    # deserves re-derivation rather than quotation. Same harness
    # (tests/test_617_placement_fanout_hole_clearance.py's
    # `_via_mover_stays_flat`), same declared 0.25, on this tree: flat -> "1
    # move(s), 1 connector(s); copper-to-hole gap 0.2200mm"; raised -> "no
    # clear spot", 0 and 0. The connector's only validated landing sits 0.2200
    # off the wall, so ANY floor above 0.2200 kills it and 0.25 is above it.
    # #617's choice stands, and `self.npth_floor` (which DOES read the board)
    # deliberately differs from this. See the note beside that assignment.
    npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE)

    # ---- #730: the OVERRIDE hole gate ------------------------------------
    #
    # `_seg_foreign_hole_dist` returns a PURE minimum distance -- one number
    # for the whole board, with no per-hole requirement in it. That is right
    # for a flat floor and wrong for KiCad's copper-to-hole rule, which honours
    # each hole pad's OWN `(clearance ...)`. So the holes that CARRY one are
    # collected here and gated separately, ADDITIVELY: the two scalar gates
    # below keep their floors and their spellings exactly, and this list only
    # ever refuses more. That is what makes #730 raise-only.
    #
    # THE THRESHOLD READS THE BOARD, and that is the one board-derived value
    # this function is allowed. check_drc's via arm is a STEP --
    # `kicad_req = req_clr if req_clr > npth_clr + 1e-9 else clearance`
    # (check_drc.py:2851) -- whose `npth_clr` is
    # `max(clearance, NPTH_TO_TRACK_CLEARANCE, hole_clearance)`, the third term
    # being the board's declared min_hole_clearance. A threshold WITHOUT that
    # term charges `lc` on a board where the checker charges `clearance`, which
    # is the invented-refusal class #737's own comment is about. Measured
    # during review: declared 0.35, hole lc 0.30, a landing 0.200 off the wall
    # -- shipped 1 move; flat threshold 0 moves (repair LOST); board-aware
    # threshold 1 move; and check_drc grades that landing CLEAN
    # (kicad_req 0.10, overlap -0.1000). So the flat spelling reproduces, by
    # 0.10mm, exactly the failure #737 was written to avoid.
    #
    # This is NOT the read #617 refused, and the direction is the whole
    # argument. #617 refused board-deriving a FLOOR, which raises what every
    # candidate is charged. This is a THRESHOLD: raising it makes it HARDER for
    # a pad override to outrank the fab floor, so a larger declared value can
    # only ever charge LESS. Against what ships today -- no `lc` term at all --
    # the gate is raise-only either way, and the thing that keeps it from
    # costing repairs is not the monotonicity but that it fires ONLY for a
    # hole whose pad carries an explicit, author-declared keep-clear above the
    # fab floor. A per-pad declaration, not a board-wide floor: that is the
    # distinction from #617, and it is why this one is safe to raise.
    #
    # DISCLOSED, not exact. `resolve_hole_clearance` returns
    # `max(rules.min_hole_clearance, fab_floor_origin)` while check_drc's CLI
    # pins `rules` alone, so on a mid-chain board whose recorded origin exceeds
    # its current rule (the tigard 0.25 -> 0.15 chain that resolver's own
    # docstring measures) this threshold sits ABOVE the checker's, and a pad
    # with `lc` in that band is graded by the checker and not charged here.
    # #730 narrowed, not closed. Filed rather than papered over with a third
    # copper-to-hole resolver in this pass.
    #
    # The OTHER direction is reachable too, and only one way: `check_drc
    # --hole-clearance X` given explicitly above the board's own rule makes the
    # checker's npth_clr exceed this threshold, so a pad with `lc` in
    # `(npth_step, X]` is charged `lc` here and `clearance` there -- an
    # over-block. It needs a grader invoked with a floor the board does not
    # declare, which is a deliberate act rather than a chain artefact, but it
    # belongs in the disclosure beside its mirror image.
    #
    # Read through `pcb_data`, never off `st`: this function is public and its
    # harnesses pass duck types carrying `caps` and `vias` and nothing else, so
    # a `getattr(st, 'npth_floor', ...)` would silently take the default -- and
    # the default is the LOW threshold, which is the direction that costs
    # repairs. (`st.npth_floor` is also the wrong number: it omits `clearance`,
    # so it is too low at --clearance 0.3.)
    from obstacle_map import resolve_hole_clearance
    npth_step = max(npth_clr, resolve_hole_clearance(pcb_data, None))

    # The hole SET is selected with the same two predicates and the same
    # capsule geometry `single_ended_routing._foreign_hole_capsules` uses, so
    # the override gate and its two scalar siblings cannot disagree about which
    # holes EXIST -- only about what one costs. That agreement is the property
    # test_737's TestTheTwoSiblingValidatorsSeeTheSameHoles exists to hold, and
    # a re-spelt predicate is how it would be lost, silently and by
    # under-blocking. Built from `pads_by_net`, which KEEPS the pads of movable
    # caps, and NOT from `board_pads`, which drops them.
    #
    # Almost always EMPTY: 20 of the 22 tracked boards carry no NPTH override
    # at all, and watchy's 8 are 0.100, under the 0.20 fab floor. So the two
    # gates below cost a zero-length loop on every board in the corpus, and
    # the `if` in front of each is what keeps that true rather than an
    # assertion about it.
    override_holes = []
    for _hnet, _hpads in getattr(pcb_data, 'pads_by_net', {}).items():
        for _hp in _hpads:
            if (getattr(_hp, 'drill', 0) or 0) <= 0 or not _pad_has_no_copper(_hp):
                continue
            _hlc = getattr(_hp, 'local_clearance', 0.0) or 0.0
            if _hlc <= npth_step + 1e-9:
                continue
            (_h1, _h2, _hr) = pad_drill_capsule(_hp)
            override_holes.append((_h1[0], _h1[1], _h2[0], _h2[1], _hr,
                                   _hnet, _hlc))

    # REACH FILTER, and it is why a board that DOES carry an override still
    # pays almost nothing. Every point either gate tests lies within
    # `max_shift` of an existing via, and the requirement is
    # `hr + hlc + (vr | hw)`, so a hole further than
    # `hr + hlc + max_shift + widest` from EVERY via can never bind. Dropping
    # those is provably conservative -- it removes only holes no candidate can
    # reach -- and it matters: measured during review, the unfiltered gate ran
    # 104us per candidate against the existing gate's 37us, about 22% on top of
    # valid_via_pos's per-candidate cost, paid on a board where the branch
    # then fires on nothing. On the fanned ulx3s the nearest via to AUDIO1's
    # two holes is 9.283mm and 15.055mm against a 0.6mm spiral, so the filter
    # empties the list there and both gates cost one truth test.
    if override_holes:
        # `getattr`, like every other read of a via in this function: the
        # nudger is public and its harnesses pass duck-typed via-likes. A bare
        # `v.size` here would raise ONLY on a board that carries an override
        # hole -- invisible on 20 of 22 tracked boards and on every existing
        # harness, which is the worst possible place for a hard read.
        _widest = max([(getattr(v, 'size', 0.0) or 0.0) / 2.0
                       for v in pcb_data.vias]
                      + [(getattr(sg, 'width', 0.0) or 0.0) / 2.0
                         for sg in pcb_data.segments] + [0.0])
        override_holes = [
            h for h in override_holes
            if any(_point_to_seg_dist(v.x, v.y, h[0], h[1], h[2], h[3])
                   <= h[4] + h[6] + max_shift + _widest
                   for v in pcb_data.vias)]

    def override_hole_gap(net_id, x1, y1, x2, y2):
        """Worst `distance-to-hole-wall MINUS that hole's own requirement`
        over the override holes, or 1e9 when there are none.

        Reads like its two scalar siblings -- a distance the caller compares
        against its own half-extent -- so all four hole gates in this function
        are one expression shape. Same own-net exemption they make, and the
        same measure: a full SEGMENT-to-segment distance (crossing branch
        included) to the hole's capsule axis, minus the capsule radius. Not an
        endpoint test -- a connector whose MIDDLE grazes a hole must be caught,
        and its endpoints can both be clear while it does. A round drill is a
        zero-length capsule and the via gate passes a zero-length segment, so
        the point cases fall out rather than being special-cased."""
        best = 1e9
        for hx1, hy1, hx2, hy2, hr, hnet, hlc in override_holes:
            if hnet == net_id:
                continue
            d = _seg_seg_dist_coords(x1, y1, x2, y2,
                                     hx1, hy1, hx2, hy2) - hr
            if d - hlc < best:
                best = d - hlc
        return best

    def edge_ok_point(x, y, r):
        need = r + edge_margin
        if edge_rings:
            if not _point_on_board(x, y, edge_outer, edge_cutouts):
                return False
            return _point_to_rings_distance(x, y, edge_rings) >= need - 1e-6
        if bounds:
            return (x - bounds[0] >= need and bounds[2] - x >= need and
                    y - bounds[1] >= need and bounds[3] - y >= need)
        return True

    def edge_ok_seg(sx, sy, ex, ey, hw):
        need = hw + edge_margin
        if edge_rings:
            if not (_point_on_board(sx, sy, edge_outer, edge_cutouts) and
                    _point_on_board(ex, ey, edge_outer, edge_cutouts)):
                return False
            return _segment_to_rings_distance(sx, sy, ex, ey,
                                              edge_rings) >= need - 1e-6
        if bounds:
            return all(x - bounds[0] >= need and bounds[2] - x >= need and
                       y - bounds[1] >= need and bounds[3] - y >= need
                       for x, y in ((sx, sy), (ex, ey)))
        return True

    # (rect, net, cap_layer): cap pads exist only on the cap's own copper
    # side -- connector segments on OTHER layers cannot graze them (the
    # missing layer gate rejected every candidate: the connector necessarily
    # starts at the old via position, inside the grazed cap's keep-out, but
    # only the VIA barrel -- not an inner-layer connector -- conflicts there).
    # #725: every requirement below resolves through st's own resolvers, so the
    # nudger and the grader cannot disagree. `getattr` throughout -- tests pass
    # a duck-typed _FakeSt that carries none of this, and must grade flat.
    _req = getattr(st, 'required', None)
    _via_req = getattr(st, 'via_required', None)
    _pad_fl = getattr(st, 'pad_floor', None)
    _via_fl = getattr(st, 'via_floor', None)
    _seg_fl = getattr(st, 'seg_floor', None)
    _trk_req = getattr(st, 'track_required', None)
    # #735: whether a SECOND, relaxed sweep rung is worth running at all.
    _trk_on = bool(getattr(st, 'track_rules_active', False))
    _via_rad = getattr(st, 'via_radius', None)
    # #733: the board-edge requirement comes from st, so the two edge helpers
    # below cannot gate the copper this function EMITS more weakly than the cap
    # mover gates a cap pad.
    #
    # max(), not a bare read, for the same reason _item_reach never returns below
    # `self.clearance`: the flat argument is this function's own floor, and an st
    # must never LOWER this test below the clearance the caller passed.
    #
    # Defensive, and say so precisely rather than overstate it: no LIVE caller
    # can currently hand this a sub-clearance margin. `_Repair` is constructed at
    # one site, its margin is already max(clearance, ...), and the margin-ZERO
    # outline gate render_placement builds is a shallow COPY it keeps beside
    # `state.edge_gate` rather than a `_Repair` -- the #733 review checked, after
    # an earlier version of this comment claimed otherwise. A duck-typed st CAN,
    # and the #733 test exercises exactly that.
    #
    # The fallback is EXACTLY `clearance`, not CAP_EDGE_CLEARANCE -- the duck-typed
    # _FakeSt the #370/#617 harnesses pass carries no margin, and this function's
    # own flat scalar is the honest stand-in for one. Worth stating plainly: those
    # two harnesses do NOT in fact pin this. Every landing they exercise clears
    # 0.80mm, so a 0.55 fallback passes all five of their arms unchanged. That is
    # exactly why tests/test_733_*.py brackets the fallback to (0.20, 0.30] on a
    # rig built for it -- the regression a later reader would introduce by
    # "tidying" this line is one nothing else would catch.
    _edge_m = getattr(st, 'edge_margin', None)
    edge_margin = clearance if _edge_m is None else max(clearance, _edge_m)

    def req(fa, fb):
        return clearance if _req is None else _req(fa, fb)

    def via_req(pad_floor, via_net):
        return clearance if _via_req is None else _via_req(pad_floor, via_net)

    def pad_fl(p):
        return None if _pad_fl is None else _pad_fl(p)

    def via_fl(net):
        return None if _via_fl is None else _via_fl(net)

    def seg_fl(net, layer):
        return None if _seg_fl is None else _seg_fl(net, layer)

    def track_req(fa, fb, net_a, net_b):
        """The TRACK-vs-track requirement (#735), st-resolved when it can be.

        Falls back through `req`, not to `clearance`: an st that resolves pairs
        but predates this resolver must still get its netclass and layer-rule
        answer. The duck-typed harnesses carry neither, and grade flat -- the
        same contract every shim above keeps."""
        if _trk_req is None:
            return req(fa, fb)
        return _trk_req(fa, fb, net_a, net_b)

    def via_rad(v):
        """This via's RADIUS, resolved by st when it can (#732), so the offender
        test, the candidate validation and the #313 connectivity tolerance below
        cannot disagree with the keep-out list the grader was built from. Four
        sites here spelled it `(x.size or 0.5) / 2.0` and a fifth `v.size / 2.0`
        with no fallback at all, while the grader used --default-via-size.

        The fallback is reached ONLY on the duck-typed path -- the _FakeSt the
        #370 and #617 harnesses pass carries no resolver -- and only for a via
        whose size is falsy. Every via those fixtures build carries a real 0.5,
        so it is numerically unreachable there; the #732 test file pins that.
        _UNREADABLE_VIA_SIZE rather than a bare literal so the number has ONE
        name in this module. (#750 corrects two claims this docstring used to
        make: it named `defaults.UNREADABLE_VIA_SIZE`, which exists nowhere in
        the repo -- the constant is module-private here and reaches this call
        as _via_radius's default argument -- and it described the guard as
        `and ... > 0`, where _via_radius spells it `not size or size <= 0`.
        Same semantics, and worth stating correctly because the SPELLING is the
        rule: not `or`, so a negative size cannot take a different branch in
        the two. #750 applies that same rule to the drill.)"""
        if _via_rad is not None:
            return _via_rad(v)
        return _via_radius(v)

    def cap_floors_of(cap, rects):
        """This cap's per-pad floors, index-aligned with `rects` -- or None,
        which grades flat. Tests drive this function with a duck-typed cap that
        carries neither the attribute nor a matching length.

        A pad whose `pad_layers` entry is EMPTY carries no copper, and its
        entry comes back None so it grades flat here too. The eff builders key
        on that same marker; without it the nudger would charge a phantom the
        grader does not, hand itself a via nothing flagged, and print an
        operator-facing line naming a non-violation.
        """
        pf = getattr(cap, 'pad_floors', None)
        if pf is None or len(pf) != len(rects):
            return None
        pl = getattr(cap, 'pad_layers', None)
        if pl is None or len(pl) != len(pf):
            return list(pf)
        return [f if pl[i] else None for i, f in enumerate(pf)]

    # `all_cap_rects` is function-local, so it CAN carry the pad's floor.
    all_cap_rects = []
    for ref, cap in st.caps.items():
        fp = pcb_data.footprints.get(ref)
        cl = getattr(fp, 'layer', 'F.Cu') if fp is not None else 'F.Cu'
        _rects = cap.pad_rects(cap.x, cap.y, cap.rot)
        pf = cap_floors_of(cap, _rects)
        for i, (bx0, by0, bx1, by1, net) in enumerate(_rects):
            all_cap_rects.append((bx0, by0, bx1, by1, net, cl,
                                  None if pf is None else pf[i]))

    # Flatten the board's pads ONCE, in pads_by_net iteration order, resolving
    # each pad's floor and drill capsule here instead of inside the 16-angle x
    # 12-radius sweep below. Strictly cheaper than the previous per-call walk.
    board_pads = []
    for _pads in pcb_data.pads_by_net.values():
        for p in _pads:
            if getattr(p, 'component_ref', None) in st.caps:
                continue  # movable caps handled by the final rects above
            cap_ = pad_drill_capsule(p) if (p.drill and p.drill > 0) else None
            board_pads.append((p, pad_fl(p), not _pad_has_no_copper(p), cap_))

    # #750: every OTHER via's drill radius, resolved ONCE, for the same reason
    # the pads above are flattened here -- the hole-to-hole gate below runs
    # inside the candidate sweep (16 angles x up to 12 radii), so resolving
    # there costs one call per via per candidate. Measured on
    # orangecrab_ext_pll (136 vias): 6336 valid_via_pos calls, so this
    # replaces ~862k resolutions with 136.
    #
    # That parenthetical used to call orangecrab "the one tracked board that
    # actually nudges one", which #756's independent re-measurement showed is
    # not so: at the shipped defaults its `unresolved` is EMPTY at clearance
    # 0.10, 0.20 and 0.25, so this function is never entered on it. The only
    # tracked board that gets past the early return is
    # rp2350_fpga_eensy_prePlane (unresolved 1/5/7 at those three), and it
    # emits zero via moves. Pre-existing -- unchanged at 05ebfc74 -- and
    # corrected here rather than carried, since the measurement above is
    # still right and only the attribution was wrong. `tests/test_746`'s
    # corpus arm already names rp2350 as the board that reaches the nudger.
    #
    # WHAT MAKES THE HOIST SAFE is narrower than "nothing mutates
    # pcb_data.vias", and the wider claim is the one that invites the bug:
    # the nudger relocates vias IN PLACE (`v.x, v.y = nx, ny` below) and
    # processes caps one after another, so a via's COORDINATES do change
    # between iterations. This tuple caches only the DRILL RADIUS, which no
    # path writes, and the gate still reads `ov.x`/`ov.y` live. Folding the
    # coordinates in here -- the obvious tidy-up, since they are right there
    # -- is a real defect: measured on a two-cap rig, the second via lands at
    # (5.1061, 6.1061) instead of (5.0500, 6.0000).
    #
    # It also snapshots MEMBERSHIP, not just the radius: the gate used to walk
    # pcb_data.vias live. Inert today -- nothing in this function appends to,
    # slices or rebinds that list -- but a future edit that adds a via
    # mid-pass (a stitching barrel, a split) would be invisible to the
    # hole-to-hole gate and to nothing else, so add it here too.
    #
    # `via_rad(ov)` on the copper line below is deliberately NOT hoisted with
    # it: that resolver takes the operator's --default-via-size through `st`,
    # so caching it here would put a second copy of a knob-dependent answer
    # beside the one _Repair already owns -- the shape #732 removed.
    board_via_drills = [(ov, _via_drill_radius(ov)) for ov in pcb_data.vias]

    def valid_via_pos(v, nx, ny):
        vr = via_rad(v)
        vdr = _via_drill_radius(v)      # hoisted once, like vr (#750)
        vfl = via_fl(v.net_id)
        # never off the board / into a cutout / inside the edge margin (#370 B3)
        if not edge_ok_point(nx, ny, vr):
            return False
        # #737: the relocated via's COPPER against copper-less (NPTH) holes.
        # This function had no such test. Its only drilled-pad gate is the
        # DRILL-to-drill one below, which measures the DRILL, so it covered the
        # annular ring only while
        #
        #     ring = (size - drill) / 2 <= H2H_PAD - clearance
        #
        # -- 0.20 at the shipped --clearance 0.25, and 0 at clearance >= 0.45.
        # Since #756 `H2H_PAD` is `max(declared, the fab pad-hole floor)`
        # rather than a flat 0.45, so those two numbers are the values on a
        # board that declares nothing at or above 0.45 -- which is every board
        # this repo tracks. A board declaring MORE widens the window this
        # arithmetic covers, and never narrows it, because the RESOLVER is
        # raise-only.
        #
        # THAT IS A CLAIM ABOUT THE FLOOR, NOT ABOUT THE LANDING, and an
        # independent re-measurement read it the other way, so say which.
        # The emitted geometry is NOT monotone in the declaration: with two
        # offending vias in one call, via #1 can take a rung-1 landing that
        # leaves via #2 nothing at rung 1, so #2 falls back and the run's
        # WORST drill gap ends smaller than the flat floor's would have.
        # Measured over 3600 configurations: 5 of them, worst case
        # 0.5889 -> 0.2643mm. Never below the fab floor (minimum emitted gap
        # is 0.2000 on both arms) and never a lost repair, which are the two
        # properties the ladder actually guarantees.
        # Past that the pass parked ring copper inside a mounting hole's
        # keep-out, and this pass WRITES the via (placement/writer.py, and the
        # plugin's pcbnew mirror). Measured at --clearance 0.25: a 1.4/0.3 via
        # relocated to 0.150mm of copper-to-hole-wall gap, which check_drc's
        # own via arm of this rule reports as `via-hole ... 0.100mm`.
        #
        # THE FLOOR IS `clearance`, NOT `npth_clr` -- deliberately NOT the
        # sibling's, and that asymmetry is the point rather than an oversight.
        # `npth_clr` is `max(clearance, NPTH_TO_TRACK_CLEARANCE)`: a routing
        # policy for TRACKS, which is what connector_clear gates. check_drc
        # grades a VIA against a plain NPTH hole at `clearance`
        # (`kicad_req = req_clr if req_clr > npth_clr else clearance`), and
        # says why: charging a via the track floor "invents items kicad-cli
        # never reports" (crkbd, 7 phantoms). obstacle_map stamps the same
        # asymmetry -- a plain NPTH is a track keep-out, not a via-copper one.
        #
        # Charging `npth_clr` here was measured to COST THE REPAIR rather than
        # merely cost search room: at --clearance 0.1 it refuses a landing
        # 0.150mm off the hole wall that check_drc grades CLEAN, and the cap
        # then keeps the #130 pad-via graze this pass exists to remove. That is
        # exactly the failure obstacle_map.resolve_hole_clearance names for
        # this function by name -- an all-or-nothing repair whose one clearing
        # candidate must not be refused.
        #
        # A board's DECLARED min_hole_clearance stays unread here, and the
        # checkable reason is better than the #617 appeal it replaces: it
        # changes nothing in check_drc's VIA arm either. The declared floor
        # enters that arm only through the `req_clr > npth_clr` THRESHOLD, so
        # with hole_clearance 0.5 and no pad override the requirement is still
        # `clearance`. (#617 is about a different move -- it declined to RAISE
        # the connector gate above the flat fab floor; this LOWERS the via gate
        # below it, which #617 never measured. The principle is shared, the
        # floor is not.)
        #
        # At --clearance >= 0.20 the two floors are equal, so this only ever
        # differs below the fab floor. The CLI's own --help example is
        # --clearance 0.1 and it applies no fab floor at all; the GUI's spin
        # minimum is 0.05 but its value passes through _fab_floored, so the
        # reachable minimum there is the fab tier's -- 0.127 standard 2-layer,
        # 0.10 advanced 2-layer, 0.09 advanced multilayer.
        #
        # Same helper, same own-net exemption and the same 1e-4 as
        # connector_clear's gate below, so the two cannot disagree about which
        # holes EXIST -- only about what one costs a track versus a via. The
        # gate order now reads alike in both (edge, hole, cap rects, board
        # pads, vias, segments). `board_pads` was the cheaper source and the
        # wrong one: it drops the pads of movable caps, which the helper keeps.
        # That buys AGREEMENT between the siblings rather than truth -- both
        # read pre-move pad coordinates, so neither is right about a cap this
        # pass has already relocated, and `all_cap_rects` above is post-move
        # while this hole set is pre-move.
        #
        # A degenerate segment is the point case: _seg_capsule_axis_dist guards
        # L2 == 0 and its crossing test is strict, so a zero-length segment
        # cannot report a spurious crossing.
        #
        # The drill test below keeps its own floor and its net-independence:
        # drill-to-drill is a machine constraint, this is an etch constraint,
        # and check_drc's via arm makes the same own-net exemption.
        #
        # #730 is CLOSED, in the SECOND gate below rather than in this one, and
        # the gate on the next line is deliberately unchanged -- floor,
        # spelling and all. check_drc's via arm is a STEP, not a max:
        # `kicad_req = lc if lc > npth_clr else clearance`. So for
        # `clearance < lc <= npth_clr` the requirement is EXACTLY `clearance`
        # and this gate was already right (measured: clearance 0.10, lc 0.15 ->
        # the checker wants 0.100, this charges 0.100). Only an override ABOVE
        # `npth_clr` outranks it, and that is what `override_holes` carries.
        #
        # Which is why the obvious `max(clearance, lc)` here is wrong -- in ONE
        # direction, and it is the direction that costs a repair. Against the
        # step it can only ever charge MORE or the same: for `lc > npth_clr`
        # the two agree, and for `lc <= npth_clr` the checker charges
        # `clearance` while the max charges `lc`. At clearance 0.10 / lc 0.15
        # that is an over-block of 0.05 and a refusal kicad-cli never reports
        # -- the same invented-refusal class the crkbd phantoms belong to,
        # though there the trigger was the fab floor and not an override.
        # Splitting the step across two gates rather than folding it into one
        # expression is what keeps this fix RAISE-ONLY and byte-identical on
        # every board declaring no override.
        if _seg_foreign_hole_dist(pcb_data, v.net_id, nx, ny, nx, ny) < \
                clearance + vr - 1e-4:
            return False
        # ...and the step's OTHER branch: a hole whose pad declares more than
        # `npth_step` charges that override instead of `clearance`.
        # `override_holes` is empty on every tracked board, so this costs one
        # truth test there.
        if override_holes and override_hole_gap(
                v.net_id, nx, ny, nx, ny) < vr - 1e-4:
            return False
        for (bx0, by0, bx1, by1, net, _cl, pfl) in all_cap_rects:
            # via barrel spans all layers: no layer gate here
            if net != v.net_id and _point_to_rect_dist(
                    nx, ny, (bx0, by0, bx1, by1)) < vr + via_req(pfl, v.net_id):
                return False
        for p, pfl, has_cu, cap_ in board_pads:
            # rotation/shape-aware pad copper distance (#370 B3, #356
            # class: the axis-aligned size_x/size_y rect under-blocked
            # rotated pads). NPTH pads carry no copper -- drill-only.
            if (p.net_id != v.net_id and has_cu
                    and point_to_pad_distance(nx, ny, p) < vr + req(pfl, vfl)):
                return False
            if cap_ is not None:
                # slot/offset-aware drill capsule (net-INDEPENDENT floor, so
                # deliberately NOT #725-resolved -- see npth_clr above)
                (c1x, c1y), (c2x, c2y), prad = cap_
                if _point_to_seg_dist(nx, ny, c1x, c1y, c2x, c2y) < \
                        vdr + prad + H2H_PAD:
                    return False
        for ov, ovdr in board_via_drills:
            if ov is v:
                continue
            d = math.hypot(nx - ov.x, ny - ov.y)
            if ov.net_id != v.net_id and d < vr + via_rad(ov) + \
                    req(vfl, via_fl(ov.net_id)):
                return False
            # net-INDEPENDENT: two holes collide whatever they carry
            if d < vdr + ovdr + H2H_VIA:
                return False
        for s in pcb_data.segments:
            if s.net_id == v.net_id:
                continue
            if _point_to_seg_dist(nx, ny, s.start_x, s.start_y, s.end_x, s.end_y) \
                    < vr + s.width / 2.0 + req(vfl, seg_fl(s.net_id, s.layer)):
                return False
        return True

    def connector_clear(net_id, layer, width, sx, sy, ex, ey,
                        honour_track_rules=True):
        hw = width / 2.0
        # The connector is a TRACK on `layer`, so it resolves like one: net
        # classes, .kicad_dru LAYER rules, and -- since #735 -- the board's
        # TRACK-SCOPED .kicad_dru rules, through the same resolver check_drc
        # grades a seg-seg pair with.
        #
        # THE TRACK RULE REACHES ONE ARM OF THIS FUNCTION, and the arms below
        # are exempt by KiCad's own rule condition rather than by omission: it
        # names a track on BOTH sides, so a cap pad, a board pad or a via on
        # the far side takes the pair out of scope. The last loop is the only
        # place both sides are tracks. The same reasoning exempts the via
        # mover above and this module's broad-phase over-reach, whose far side
        # is always a cap pad.
        #
        # Before #735 this arm priced at the net class alone, so on a board
        # declaring a track rule the pass drew a connector closer to foreign
        # copper than its own checker accepts -- an under-block, which ships
        # the violation rather than refusing the landing.
        cfl = seg_fl(net_id, layer)
        # board edge / cutouts + NPTH drill holes at their floor (#370 B3):
        # a connector is drawn geometrically, not routed, so it must gate
        # against Edge.Cuts and copper-less holes itself.
        if not edge_ok_seg(sx, sy, ex, ey, hw):
            return False
        if _seg_foreign_hole_dist(pcb_data, net_id, sx, sy, ex, ey) < \
                npth_clr + hw - 1e-4:
            return False
        # #730, the TRACK arm: check_drc grades a track against an NPTH hole at
        # `max(npth_clr, lc)` -- a MAX, where the via gate above takes a STEP --
        # so an override above the floor this gate already charges raises it,
        # and one below it changes nothing. `override_holes` is filtered to
        # `lc > npth_step >= npth_clr`, so every member is above and the max
        # collapses to `lc` for all of them: the FILTER carries this arm, not
        # the charge. `npth_clr` itself stays the flat fab floor -- #617's
        # decision, re-measured in the note beside it.
        if override_holes and override_hole_gap(
                net_id, sx, sy, ex, ey) < hw - 1e-4:
            return False
        for (bx0, by0, bx1, by1, net, cl, pfl) in all_cap_rects:
            if cl != layer:
                continue  # cap pads only exist on the cap's own side
            if net != net_id and _seg_to_rect_dist(
                    sx, sy, ex, ey, (bx0, by0, bx1, by1)) < hw + req(pfl, cfl):
                return False
        for p, pfl, has_cu, _cap in board_pads:
            if p.net_id == net_id:
                continue
            if not _pad_on_layer(p, layer):
                continue
            if not has_cu:
                continue  # NPTH: no copper; the hole check above covers it
            # rotation-aware pad rect (#370 B3): rotate the segment into
            # the pad's frame so a tilted pad is tested against its true
            # rectangle (distance is rotation-invariant).
            rx1, ry1 = into_pad_frame_point(sx, sy, p)
            rx2, ry2 = into_pad_frame_point(ex, ey, p)
            d, _ = segment_to_rect_distance(
                rx1, ry1, rx2, ry2, p.global_x, p.global_y,
                p.size_x / 2.0, p.size_y / 2.0)
            if d < hw + req(pfl, cfl):
                return False
        for ov in pcb_data.vias:
            if ov.net_id != net_id and _point_to_seg_dist(
                    ov.x, ov.y, sx, sy, ex, ey) < via_rad(ov) + hw \
                    + req(cfl, via_fl(ov.net_id)):
                return False
        for s2 in pcb_data.segments:
            if s2.net_id == net_id or s2.layer != layer:
                continue
            if _segs_cross(sx, sy, ex, ey, s2.start_x, s2.start_y,
                           s2.end_x, s2.end_y):
                return False
            d = min(_point_to_seg_dist(sx, sy, s2.start_x, s2.start_y, s2.end_x, s2.end_y),
                    _point_to_seg_dist(ex, ey, s2.start_x, s2.start_y, s2.end_x, s2.end_y),
                    _point_to_seg_dist(s2.start_x, s2.start_y, sx, sy, ex, ey),
                    _point_to_seg_dist(s2.end_x, s2.end_y, sx, sy, ex, ey))
            sfl = seg_fl(s2.net_id, s2.layer)
            pair_req = (track_req(cfl, sfl, net_id, s2.net_id)
                        if honour_track_rules else req(cfl, sfl))
            if d < hw + s2.width / 2.0 + pair_req:
                return False
        return True

    for ref in sorted(unresolved):
        cap = st.caps[ref]
        rects = cap.pad_rects(cap.x, cap.y, cap.rot)
        # #725: this predicate MUST match via_penalty's, which is why both go
        # through st.via_required. Left at the flat scalar while the grader
        # resolves the requirement, a cap reported unresolved because of a
        # raised via would yield an EMPTY offender list -- the pass would print
        # nothing and report the cap unresolved forever.
        pfls = cap_floors_of(cap, rects)
        offenders = []
        for v in pcb_data.vias:
            vr = via_rad(v)
            for i, (bx0, by0, bx1, by1, net) in enumerate(rects):
                if v.net_id != net and _point_to_rect_dist(
                        v.x, v.y, (bx0, by0, bx1, by1)) < vr + via_req(
                            None if pfls is None else pfls[i], v.net_id) - EPS:
                    offenders.append(v)
                    break
        for v in offenders:
            # Layers needing a connector back to the stub start: every layer
            # with same-net copper terminating at the old via position, plus
            # the copper layer of a same-net pad the via sits inside
            # (via-in-pad -- the pad connection must follow the via).
            conn_layers = {}
            # #313: copper terminating within the VIA BODY was electrically
            # joined through the via and needs a connector on its layer. The old
            # 1um match missed copper offset by grid quantization or a prior
            # #280 via-nudge; the via body radius is the correct connectivity
            # tolerance (a track end buried in the via is connected).
            tol = max(1e-3, via_rad(v))
            for s in pcb_data.segments:
                if s.net_id != v.net_id:
                    continue
                if (math.hypot(s.start_x - v.x, s.start_y - v.y) < tol
                        or math.hypot(s.end_x - v.x, s.end_y - v.y) < tol):
                    w = conn_layers.get(s.layer)
                    conn_layers[s.layer] = min(w, s.width) if w else s.width
            # via-in-pad: rotation/shape-aware containment (the axis-aligned
            # size_x/size_y bbox mis-classified rotated/oval/custom pads).
            from check_drc import point_to_pad_distance
            for p in pcb_data.pads_by_net.get(v.net_id, []):
                if point_to_pad_distance(v.x, v.y, p) > 1e-6:
                    continue  # via not inside this pad's copper
                fallback = (min(conn_layers.values()) if conn_layers
                            else defaults.TRACK_WIDTH)
                pad_cu = [l for l in (p.layers or [])
                          if l.endswith('.Cu') and not l.startswith('*')]
                if not pad_cu and any(l.endswith('.Cu') for l in (p.layers or [])):
                    # THT *.Cu pad: no concrete side layer -- the pad barrel ties
                    # every layer, so reconnect on the via's own span layers
                    # (previously this pad got NO connector at all).
                    pad_cu = [l for l in (v.layers or []) if l.endswith('.Cu')] or ['F.Cu']
                for pl in pad_cu:
                    if pl not in conn_layers:
                        conn_layers[pl] = fallback
            # zone/pour ties: a via stitching a same-net pour needs a connector
            # on that layer or the pour tie is silently dropped by the move.
            for z in (getattr(pcb_data, 'zones', []) or []):
                if z.net_id != v.net_id or not getattr(z, 'polygon', None):
                    continue
                if _point_in_poly(v.x, v.y, z.polygon) and z.layer not in conn_layers:
                    conn_layers[z.layer] = (min(conn_layers.values()) if conn_layers
                                            else defaults.TRACK_WIDTH)
            # #756: TWO RUNGS, not one. `drill_ladder` is [declared] or
            # [declared, fab]; `H2H_VIA`/`H2H_PAD` are rebound per rung and the
            # two gates below read them as closure variables, so the whole
            # sweep re-runs at the looser floor only when the tighter one
            # cleared nothing. Rung 2 IS the pre-#756 behaviour, which is what
            # makes this incapable of losing a repair -- see the ladder comment
            # at the top of this function for the 8673-configuration sweep that
            # made a one-rung raise unacceptable.
            #
            # DO NOT COLLAPSE THESE INTO ONE PASS AT max(...) OR min(...).
            # max() is the one-rung raise this replaced. min() would take the
            # fab floor always and make the declaration inert. The point is
            # ORDER: prefer the board, keep the repair.
            #
            # #735: the .kicad_dru TRACK rule is an OUTER rung of the same
            # ladder, and for the same reason. A hard gate on it would trade
            # the #130 pad-via graze this pass exists to remove -- which stays
            # on the board when the sweep abandons the via -- for a
            # `segment-segment-track-rule` pair, and check_drc counts both. So
            # the rule is a PREFERENCE: sweep every drill rung honouring it,
            # and only if nothing clears, sweep again at the base requirement
            # and keep the repair. That is the same "prefer the board, keep the
            # repair" order as the rungs above, and it means this can never
            # place copper worse than the pre-#735 pass did.
            trk_ladder = [True, False] if _trk_on else [True]
            found, trk_relaxed = None, False
            for TRK_STRICT in trk_ladder:
                for H2H_VIA, H2H_PAD in drill_ladder:
                    r = 0.05
                    while found is None and r <= max_shift + 1e-9:
                        for k in range(16):
                            ang = k * math.pi / 8
                            nx = round(v.x + r * math.cos(ang), 4)
                            ny = round(v.y + r * math.sin(ang), 4)
                            if not valid_via_pos(v, nx, ny):
                                continue
                            if all(connector_clear(v.net_id, layer, w,
                                                   v.x, v.y, nx, ny, TRK_STRICT)
                                   for layer, w in conn_layers.items()):
                                found = (nx, ny)
                                break
                        r += 0.05
                    if found is not None:
                        if (H2H_VIA, H2H_PAD) != drill_ladder[0]:
                            # Said out loud: the board asked for more than this
                            # landing gives, and the pass took the landing rather
                            # than the refusal. An operator who cares can widen
                            # --max-displacement or move the neighbour.
                            print(f"  via-nudge: no spot cleared the board's "
                                  f"{_h2h_decl:g}mm min_hole_to_hole for {ref}'s "
                                  f"via at ({v.x:.2f}, {v.y:.2f}); fell back to "
                                  f"the {H2H_VIA:g}mm fab floor")
                        break
                if found is not None:
                    trk_relaxed = not TRK_STRICT
                    break
            if trk_relaxed:
                # The same disclosure the drill rung makes, for the same
                # reason: the copper about to be emitted does NOT satisfy the
                # board's track rule, and check_drc will say so. Printed here
                # because `required_rows` cannot -- it is cap-pad-anchored and
                # returns nothing at all on a board whose only declaration is
                # this rule.
                print(f"  via-nudge: no spot cleared the board's .kicad_dru "
                      f"track rule for {ref}'s via at "
                      f"({v.x:.2f}, {v.y:.2f}); took a landing at the base "
                      f"clearance rather than abandoning the repair")
            if found is None:
                print(f"  via-nudge: no clear spot for {ref}'s offending via "
                      f"at ({v.x:.2f}, {v.y:.2f}) within {max_shift}mm")
                continue
            nx, ny = found
            old = (v.x, v.y)
            for layer, w in conn_layers.items():
                sd = {'start': (old[0], old[1]), 'end': (nx, ny),
                      'width': w, 'layer': layer, 'net_id': v.net_id}
                new_segments.append(sd)
                pcb_data.segments.append(Segment(
                    start_x=old[0], start_y=old[1], end_x=nx, end_y=ny,
                    width=w, layer=layer, net_id=v.net_id))
            v.x, v.y = nx, ny
            # #747: the graded via view is NOT rebuilt here any more. This
            # function is duck-typed on `st` -- nine test files drive it with
            # a stand-in carrying only caps, vias and graze_penalty, and one
            # carrying no via list at all -- so every read of `st` goes
            # through getattr with a flat fallback. A RESOLVER has an honest
            # flat fallback; a MUTATION does not, because "silently do
            # nothing" IS the staleness that shape of defect produces. It is
            # the same rule register_new_segments' docstring states for the
            # track channel, and it is why the tuple shape, the carried
            # keep-out and the radius-map invariant now live in ONE place
            # each, on _Repair, instead of being restated here.
            #
            # The move is REPORTED below instead, and repair_fanout_clearance
            # hands the report to a real _Repair, which needs no escape
            # hatch. Deferring it cannot change one decision made here, and
            # that is checkable rather than asserted: the graded view was
            # read at exactly ONE site in this function -- the rebuild that
            # used to stand here -- and graze_penalty is consulted once, at
            # the top, before any via has moved. Every other via this
            # function reads comes from pcb_data.vias, whose Via objects the
            # line above mutates in place, so the offender loop, both drill
            # gates and the connector gate all still see each landing as it
            # happens.
            via_moves.append((old[0], old[1],
                              {'x': nx, 'y': ny, 'size': v.size, 'drill': v.drill,
                               'layers': v.layers, 'net_id': v.net_id,
                               # This via came OFF the board and goes back on:
                               # the writer DELETES it at `old` and re-emits it
                               # here, and the plugin's pcbnew twin does the
                               # same. Carry its tenting/plugging spec across or
                               # the nudge silently re-tents it (#489 s8) -- a
                               # via-in-pad needing IPC-4761 Type VII ships
                               # tented from a pass whose job is clearance.
                               # Same reason and same spelling as the rip-up
                               # restore dict, plane_io.py:777-783.
                               # `dict(...)` copies the SPEC specifically: it is
                               # the one value here a consumer could mutate back
                               # into the parser Via this pass still holds live.
                               # `layers` above is shared by reference and always
                               # has been -- a pre-existing shape, not a claim
                               # this line makes.
                               # `getattr` is belt-and-braces only. Every harness
                               # that drives this function builds a REAL Via, and
                               # Via.tenting_attrs has a default_factory, so the
                               # default is unreachable today. It is spelled this
                               # way because the function is public and takes
                               # whatever pcb_data.vias holds, which is the same
                               # reason _via_radius reads its fields defensively.
                               'tenting_attrs': dict(
                                   getattr(v, 'tenting_attrs', {}) or {})}))
            nm = pcb_data.nets[v.net_id].name if v.net_id in pcb_data.nets else v.net_id
            print(f"  via-nudge: moved {nm} via ({old[0]:.3f},{old[1]:.3f}) -> "
                  f"({nx:.3f},{ny:.3f}) to free {ref}; {len(conn_layers)} "
                  f"connector segment(s) back to the stub start")
    return via_moves, new_segments


def _pad_on_layer(pad, layer):
    layers = getattr(pad, 'layers', None) or []
    return layer in layers or '*.Cu' in layers
