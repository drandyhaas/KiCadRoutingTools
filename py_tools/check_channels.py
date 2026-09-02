#!/usr/bin/env python3
"""Are the routing channels open? The lane-ledger instrument (run-6).

Productizes the skill's per-face lane ledger (supply vs demand per escape
face of every fine-pitch part) and the anchor-pair channel widths -- the
numbers the placement gate reads before routing, and the classification
input for "is this failure floorplan-shaped": a face in deficit AT THE
FINEST LEGAL GRID is a placement/floorplan fact no routing parameter can
fix, and its `eaten_by` refs are the fix iteration's move targets.

REPORT-ONLY BY DEFAULT -- the corridor law is measured descriptive, not
prescriptive, so these numbers inform the loop and do not gate by themselves.
This line used to say "exit 0 unless the board is unreadable", which predates
two exits that now exist and is the kind of stale contract a caller reads
literally: with `--gate` the exits are **4** (NEW escape damage against
`--baseline`) and **3** (nothing had a ledger, so the gate did not run and must
not be recorded as a pass); **2** is an unreadable board or a flag outside its
domain, with or without the gate. Without `--gate` it is still exit 0
throughout.

"NEW escape damage" is three predicates, not one, and saying "a new STARVED
face" was accurate only until #847: a face can now fail the gate while still
having lanes to spare. `--json` keeps them apart (`starved_faces`,
`lost_escape_share`); the text output labels each line with which one fired.

V1 limitation, stated: supply-tap/via lane consumption is not modeled.
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import sys


# A face with no lanes left is only news when it has something to send. Below
# this demand the finding is noise: plenty of correct boards have a blocked
# face that carries one or two nets.
GATE_MIN_DEMAND = 7


def _starved_faces(ledgers, min_demand):
    """(ref, face, demand) for faces with zero supply and real demand."""
    out = []
    for ref, rows in sorted((ledgers or {}).items()):
        for r in rows:
            if r['supply_finest_grid'] == 0 and r['demand_nets'] >= min_demand:
                out.append((ref, r['face'], r['demand_nets']))
    return out


def _deficit_faces(ledgers):
    """Every face that owes more nets than it can carry AT THE FINEST GRID.

    The absolute-deficit report the ledger has always computed
    (`deficit_finest_grid`, placement/routability.py) and never surfaced
    anywhere a caller could read: the per-face lines print it, the JSON did
    not, and the gate counts only STARVED faces -- `supply == 0` and
    `demand >= --min-demand`.

    Those predicates miss the shape that actually bounds a run. Measured, run
    11: U5's east face was `demand 3 / supply 1` -- a deficit of 2, at a part
    that turned out to bound the whole board's routing -- and it sat outside
    the gate's predicate entirely, on both counts.

    DELIBERATELY NOT GATED. `tests/test_run8_starved_face_gate.py` records the
    calibration that forbids it: at `demand >= 5` six of 33 healthy in-repo
    boards fire, and on one run the HUMAN control fires the same face as the
    tool's output. A face in deficit is often a property of the DESIGN -- a
    dense part hard against an edge -- not of the placement under test. So this
    is a report, sorted worst-first, and the exit code is untouched.
    """
    out = []
    for ref, rows in sorted((ledgers or {}).items()):
        for r in rows:
            if r['deficit_finest_grid'] > 0:
                out.append({'ref': ref, 'face': r['face'],
                            'demand_nets': r['demand_nets'],
                            'supply_finest_grid': r['supply_finest_grid'],
                            'deficit_finest_grid': r['deficit_finest_grid'],
                            'deficit_routed_grid': r['deficit_routed_grid'],
                            'eaten_by': r['eaten_by'][:3]})
    out.sort(key=lambda e: (-e['deficit_finest_grid'], e['ref'], e['face']))
    return out


def lost_last_lane(ledgers, base_ledgers):
    """(ref, face, demand, was) for faces whose supply fell to ZERO.

    Deliberately NOT filtered by --min-demand. That threshold exists to keep
    the ABSOLUTE starvation report quiet on healthy boards, where a lightly
    used face with no spare lane is ordinary and says nothing about a change.
    Applied to a DELTA it only hides things: the baseline already carries the
    design's own habits, so a face that had lanes and now has none is new
    damage whatever its demand. Measured: a repair took one part's north face
    from supply 4 to supply 0 at demand 6, and the gate passed because 6 < 7.
    """
    was = {(ref, r['face']): r['supply_finest_grid']
           for ref, rows in (base_ledgers or {}).items()
           for r in rows}
    out = []
    for ref, rows in sorted((ledgers or {}).items()):
        for r in rows:
            before = was.get((ref, r['face']), 0)
            if (r['supply_finest_grid'] == 0 and r['demand_nets'] >= 1
                    and before > 0):
                out.append((ref, r['face'], r['demand_nets'], before))
    return out


#: A face carrying real demand that lost this SHARE of its escape supply is
#: new damage, whether or not it reached zero (#847).
#:
#: Calibrated, not chosen -- `tests/measure_847_calibration.py` and the JSON
#: beside it are the run. The measured separation at the shipped band:
#:
#:     glasgow wrong-basin   U1 E  supply 43 -> 28  demand 12   drop 0.349
#:     tigard damaged        U3 E  supply 27 -> 13  demand  9   drop 0.518
#:     glasgow truth restore U1 E  supply 43 -> 39  demand 12   drop 0.093
#:     both self-comparisons                                    drop 0.000
#:
#: ONE BASIS, and it has to be said: the three glasgow rows are graded at
#: track 0.0889 / clearance 0.09, which is what `rL_repair` resolves. The
#: control board declares a DIFFERENT netclass (0.2/0.2), so run with no flags
#: it is graded at another basis and the two rows would not be a delta at all.
#: At its own basis that control exits 4 -- from `lost_last_lane`, on two
#: RN parts going supply 3 -> 0, which is pre-existing behaviour this
#: predicate did not introduce and does not change. The SHARE form is silent
#: on it at either basis, which is what the 0.093 above is about.
#:
#: 0.20 sits between the control's 0.093 and the worst positive's 0.349 --
#: 2.15x above the one and 1.74x below the other -- and inside the plateau
#: 0.15-0.25 over which every pair's verdict is unchanged, so it is not the
#: edge of the range that happens to work. (An earlier draft of this comment
#: called it "the geometric midpoint of 0.093 and 0.349". It is not: that is
#: 0.180, and the arithmetic mean is 0.221. The margins above are the honest
#: statement and are what the acceptance rule asks for anyway.) A FRACTION
#: rather than a
#: lane count deliberately: a lane count would have to be scaled by face
#: length, which varies by an order of magnitude on one board, while "this
#: face lost a fifth of its escape" compares a 2mm passive with a 20mm BGA
#: edge. Rounded to 4 places before comparing, so a face is not judged on
#: floating-point noise in the last bit.
GATE_MIN_SUPPLY_DROP = 0.20


def lost_escape_share(ledgers, base_ledgers, min_demand,
                      min_drop=GATE_MIN_SUPPLY_DROP):
    """(ref, face, demand, before, now) for faces that lost a SHARE of escape.

    The magnitude form of what `lost_last_lane` asks as a zero-crossing, and
    the reason it exists is that the zero-crossing MASKS real damage (#847).

    `lost_last_lane`'s predicate is `now == 0 and before > 0`, and both
    supplies fall as the escape band deepens -- so each face contributes an
    INTERVAL of bands over which it fires, and the gate is a union of
    intervals. Measured on the run-7 wrong-basin fixture: D22's north face
    goes 16 -> 0 and fires at bands 2.0 and 2.5, then stops at 3.0, not
    because it recovered but because its BASELINE also reached 0. "The
    baseline got worse too" is not evidence that the placement under test is
    fine, and that masking is exactly why `check_channels --gate` is
    non-monotone in a parameter no caller could even set.

    It also misses the damage the issue is actually about. At the shipped band
    the wrong-basin board's U1 east face falls 43 -> 28 lanes against a demand
    of 12 -- a 35% loss of escape, invisible to every predicate here before
    this one: `_starved_faces` needs supply 0, `lost_last_lane` needs a
    crossing to 0, and the deficit form sees nothing because 28 still exceeds
    12. That face is the one #847 names.

    WHO ATE IT, corrected -- an earlier draft of this paragraph said "eaten by
    U30" and that is wrong at this band. At the shipped band U1 east is eaten
    by C14(5.0), C76(4.75) and C6(3.2); U30 is not charged there at all, and
    `tests/test_run8_starved_face_gate.py` carries a passing arm asserting
    exactly that. U30 appears only at a 2.0mm band, which is #841's story, not
    this one. The distinction is not pedantic: `eaten_by` is advertised as the
    move target, so naming the wrong part sends a fix at a part that is not in
    the way.

    DELTA ONLY. The absolute forms are unchanged: `_starved_faces` still asks
    for supply 0, and `_deficit_faces` is still a report. A share-of-escape
    question has no meaning without a baseline to be a share OF.

    Unlike `lost_last_lane` this one IS filtered by --min-demand. What that
    conjunct actually buys, stated precisely because a first draft of this
    paragraph overclaimed it: it keeps the reported list off LOW-DEMAND NOISE
    -- without it the wrong-basin pair also names demand-1 diodes whose supply
    merely halved, 8 -> 3. It is NOT what keeps the controls quiet. Measured
    on the truth-restore pair at the shipped band, at --min-demand 0, 1 and 7
    and at a drop threshold of 0.15: zero hits in every combination. The
    controls are quiet because their worst drop is 0.093, below any threshold
    considered -- the separation does that work, not the conjunct. (Graded at
    the shipped pair's basis; see the constant above for why that has to be
    said, and for what the control does at its own.)
    """
    was = {(ref, r['face']): r
           for ref, rows in (base_ledgers or {}).items()
           for r in rows}
    out = []
    for ref, rows in sorted((ledgers or {}).items()):
        for r in rows:
            b = was.get((ref, r['face']))
            if b is None:
                continue
            before, now = b['supply_finest_grid'], r['supply_finest_grid']
            if before <= 0 or now >= before or r['demand_nets'] < min_demand:
                continue
            # THE FACE MUST BE THE SAME FACE. Faces are keyed by board-absolute
            # N/S/E/W of the pad extent, so ROTATING a part swaps which
            # physical edge each name refers to -- and a rectangular part then
            # reports a large "loss" on a face nothing touched. Measured on
            # tigard's J1 (faces 6.62 and 9.64mm): a 90-degree rotation with no
            # neighbour moved takes W from supply 32 to 22, a 31% drop, and the
            # gate would name a blocker that does not exist.
            #
            # A face whose own LENGTH changed did not lose its escape to a
            # neighbour; it became a different face. That is outside what this
            # predicate is asking, so the pair is skipped rather than guessed
            # at. The two zero-crossing forms are largely immune by luck -- they
            # need an exact crossing to 0 -- but `lost_last_lane` has the same
            # exposure and is left alone here, because changing a shipped
            # predicate is not this issue's business. Disclosed, not fixed.
            if abs(r['length_mm'] - b['length_mm']) > 1e-6:
                continue
            if round(1.0 - now / before, 4) >= min_drop:
                out.append((ref, r['face'], r['demand_nets'], before, now))
    return out


def main():
    p = argparse.ArgumentParser(
        description="Per-face lane ledger + anchor channel widths.")
    p.add_argument("board")
    p.add_argument("--clearance", type=float, default=None,
                   help="mm. Default: the board's own Default net-class "
                        "clearance, else routing_defaults. A lane is "
                        "track+clearance wide, so this decides the supply "
                        "this tool reports")
    p.add_argument("--track-width", type=float, default=None,
                   help="mm. Default: the board's own Default net-class track "
                        "width, else its min_track_width, else "
                        "routing_defaults")
    p.add_argument("--grid-step", type=float, default=None,
                   help="mm raster step (default: routing_defaults). Not a "
                        "board floor -- no board declares one")
    p.add_argument("--refs", nargs="*", default=None,
                   help="Parts to ledger (default: auto -- QFN/QFP/BGA "
                        "or pad pitch below 2x the lane pitch)")
    p.add_argument("--min-extent", type=float, default=3.5,
                   help="Anchor size floor for the channel table (mm)")
    p.add_argument("--json", default=None, metavar="PATH")
    p.add_argument("--baseline", default=None, metavar="BOARD",
                   help="the board this one was derived from (typically the "
                        "damaged input). Enables the E8 gate: faces that LOSE "
                        "ESCAPE relative to it -- all of it, or a large share "
                        "of it (--min-supply-drop) -- while carrying real "
                        "demand, and did not already do so on the baseline.")
    p.add_argument("--gate", action="store_true",
                   help="exit 4 when the --baseline comparison finds NEW "
                        "escape damage -- a face that lost all of its supply, "
                        "or a --min-supply-drop share of it (default: report "
                        "only)")
    p.add_argument("--escape-band", type=float, default=None, metavar="MM",
                   help="how deep off a face to look for neighbours that eat "
                        "its lanes. Default: the board's own lane pitch via "
                        "placement.escape.escape_band (4 x pitch, floored at "
                        "1mm). #847: this was unreachable from any shipped "
                        "entry point, so the one parameter that decides the "
                        "starved-face gate could not be varied without "
                        "editing source")
    p.add_argument("--min-supply-drop", type=float,
                   default=GATE_MIN_SUPPLY_DROP, metavar="FRAC",
                   help="share of a face's escape supply that must be LOST "
                        "against --baseline before it counts as new damage, "
                        "for a face carrying at least --min-demand nets "
                        "(default: %(default)s). 0 disables the check")
    p.add_argument("--min-demand", type=int, default=GATE_MIN_DEMAND,
                   help="nets a face must carry before zero supply counts as "
                        "starvation (default: %(default)s)")
    args = p.parse_args()

    # DOMAIN CHECKS, because both of these silently turn the gate into a pass.
    # `--min-supply-drop` is a FRACTION and the message it produces is a
    # PERCENT ("lost 35% of its escape"), so `--min-supply-drop 20` is a
    # plausible misreading -- and it disabled the predicate with no warning.
    # That is the failure the exit-3 block below exists to forbid: a gate that
    # examined nothing must not answer "clean".
    if not 0.0 <= args.min_supply_drop <= 1.0:
        p.error("--min-supply-drop is a FRACTION in [0, 1], not a percent: "
                "{} would disable the check silently. Use 0.2 for 20%, or 0 "
                "to turn the check off deliberately."
                .format(args.min_supply_drop))
    if args.escape_band is not None and args.escape_band <= 0:
        p.error("--escape-band must be positive: {} makes the neighbour "
                "search band empty or inverted, so every face reports full "
                "supply and the gate passes everything."
                .format(args.escape_band))

    import routing_defaults as defaults
    from kicad_parser import parse_kicad_pcb, detect_package_type
    from placement import routability

    # BOARD-FIRST. A lane is `track + clearance` wide, so BOTH of these decide
    # how much escape supply this tool believes a face has -- and it used to
    # believe two constants. Measured 2026-08 on an unnamed run board (Default
    # netclass clearance 0.2, track 0.254): at the old 0.25/0.3 constants it
    # reports 334 escape lanes where the board's own floor gives 399 -- a
    # 65-lane understatement. That board is not in the corpus, so these two
    # figures are NOT re-derivable here and are dated rather than presented as
    # current; #841 moved every escape supply on every board, so read them for
    # the direction and the scale only. The originating report measured 304 vs 378 (74 lanes) on
    # that board at 0.2/0.2; the direction and scale agree, the exact figures
    # depend on the track width in force, so both are recorded rather than
    # blended. A phantom deficit steers a placement search at the thing that is
    # not wrong.
    from list_nets import board_floor
    clearance, clr_src = board_floor(args.board, 'clearance', args.clearance,
                                     defaults.CLEARANCE)
    track, trk_src = board_floor(args.board, 'track_width', args.track_width,
                                 defaults.TRACK_WIDTH)
    # ...AND FAB-FLOORED, because board-first alone runs D9 BACKWARDS here.
    # `board_floor` is board-authoritative, not raise-only: it returns whatever
    # the board declares once positive. For a tool that GRADES existing copper
    # that is right and is what CLAUDE.md asks for -- grading stricter than the
    # board manufactures phantom violations. This tool does not grade copper,
    # it PREDICTS routability, and a lane pitch below what any fab can etch
    # predicts capacity nobody can build.
    #
    # Measured on tigard, --refs U3, against fab floors 0.09 / 0.0762.
    # Re-measured 2026-09-02 after #841 (a neighbour is charged its pad
    # COPPER, not the bbox of its pad centres), and the ladder is now what the
    # SHIPPED CLI prints -- the two lower rungs coincide because the fab floor
    # is doing exactly the job this block exists to describe:
    #
    #   python3 -X utf8 py_tools/check_channels.py \
    #       kicad_files/tigard.kicad_pcb --refs U3 \
    #       --clearance C --track-width C
    #
    #     declared 0.2 /0.2    U3 W supply  9@finest   deficit faces 1
    #     declared 0.05/0.05   U3 W supply 22@finest   deficit faces 0
    #     declared 0.02/0.02   U3 W supply 22@finest   deficit faces 0  <- clamped
    #
    # The earlier recording (supply 29 / 120 / 242, deficit faces 3 / 1 / 1)
    # was taken at the pad-centre obstruction rect AND with the floor out of
    # the way, so it is not reproducible from this CLI at any flag. It is kept
    # in the history rather than restated as current.
    #
    # That is the OPTIMISTIC direction, and the dangerous one: the instrument
    # reports escape capacity that cannot be manufactured, and a placement
    # search is then steered AWAY from a real problem -- the mirror image of
    # the phantom deficit D9 was written to remove, and worse, because a
    # phantom deficit wastes effort while a phantom SUPPLY hides a defect.
    # `resolve_cli_floor`'s consumers get this from enforce_fab_floors; this
    # tool has no such pass, so it wraps here. Pinned regardless of SOURCE: a
    # --clearance 0.05 typed on the CLI is exactly as unetchable as a declared
    # one, which is how enforce_fab_floors treats an explicit flag too.
    from fab_tiers import fab_floor_min, count_copper_layers_in_file
    try:
        _fab = fab_floor_min(count_copper_layers_in_file(args.board))
    except Exception:                                          # noqa: BLE001
        _fab = {}
    for _nm, _key, _val, _src in (('clearance', 'clearance', clearance, clr_src),
                                  ('track width', 'track_width', track, trk_src)):
        _f = _fab.get(_key)
        if _f is not None and _val < _f - 1e-9:
            print(f"  {_nm} {_val:g}mm [{_src}] is below the {_f:g}mm fab "
                  f"floor; predicting at {_f:g}mm (no fab can etch the "
                  f"narrower lane).")
    if _fab.get('clearance') is not None:
        clearance = max(clearance, _fab['clearance'])
    if _fab.get('track_width') is not None:
        track = max(track, _fab['track_width'])
    # grid_step is a RASTER setting, not a board floor -- no board declares it,
    # so it keeps its constant and is not dressed up as resolved.
    grid = (args.grid_step if args.grid_step is not None
            else defaults.GRID_STEP)
    floors = {'clearance': {'value': clearance, 'source': clr_src},
              'track_width': {'value': track, 'source': trk_src}}

    try:
        pcb = parse_kicad_pcb(args.board)
    except Exception as exc:
        print(f"cannot parse {args.board}: {exc}", file=sys.stderr)
        return 2

    refs = args.refs
    if not refs:
        pitch_floor = 2 * (track + clearance)
        refs = []
        for ref, fp in sorted((pcb.footprints or {}).items()):
            kind = None
            try:
                kind = detect_package_type(fp)
            except Exception:
                kind = None
            if kind in ('QFN', 'QFP', 'BGA'):
                refs.append(ref)
                continue
            xs = sorted({round(p.global_x, 3) for p in (fp.pads or [])})
            gaps = [b - a for a, b in zip(xs, xs[1:]) if b - a > 1e-3]
            if gaps and min(gaps) < pitch_floor and len(fp.pads or []) >= 8:
                refs.append(ref)
        if not refs:
            print("  (no fine-pitch parts auto-detected; pass --refs)")

    # Name the source, not just the number: a reader cannot otherwise tell a
    # ledger graded at the board's own floor from one graded at this tool's
    # fallback, and those are different measurements.
    # The BAND is resolved and printed beside the floors for the same reason
    # they are: a ledger graded at one depth is not comparable with one graded
    # at another, and until #847 the band appeared in no output at all -- so
    # two runs that searched different depths looked identical on paper.
    from placement.escape import escape_band as _resolve_band
    from placement.routability import _quantized_pitch as _qp
    _band = _resolve_band(_qp(track, clearance, grid),
                          basis='quantized_lane', override=args.escape_band)
    print(f"Lane ledger of {args.board} "
          f"(track {track} [{trk_src}] clearance {clearance} [{clr_src}] "
          f"grid {grid} escape-band {_band.mm:g} [{_band.source}]; "
          f"taps NOT modeled -- v1):")
    ledgers = {}
    for ref in refs:
        rows = routability.face_lane_ledger(
            pcb, ref, clearance=clearance, track_width=track,
            grid_step=grid, escape_band_mm=args.escape_band,
            pcb_file=args.board)
        if not rows:
            continue
        ledgers[ref] = rows
        for r in rows:
            flag = ''
            if r['deficit_finest_grid'] > 0:
                flag = '  <-- DEFICIT AT FINEST GRID (floorplan-shaped)'
            elif r['deficit_routed_grid'] > 0:
                flag = '  <-- deficit at routed grid (try finer grid first)'
            eaten = (' eaten_by ' + ', '.join(
                f'{n}({v})' for n, v in r['eaten_by'][:3])
                if r['eaten_by'] else '')
            print(f"  {ref} {r['face']}: demand {r['demand_nets']} "
                  f"supply {r['supply_routed_grid']}@routed"
                  f"/{r['supply_finest_grid']}@finest{eaten}{flag}")

    # The absolute deficit, summarised (run-12 Tier 3.6). Printed whether or
    # not --baseline was given, and never gated -- see _deficit_faces.
    deficit = _deficit_faces(ledgers)
    if deficit:
        _worst = deficit[0]
        print(f"Faces in DEFICIT at the finest legal grid: {len(deficit)} "
              f"(worst {_worst['ref']} {_worst['face']}: demand "
              f"{_worst['demand_nets']} vs supply "
              f"{_worst['supply_finest_grid']}). A deficit here is a "
              f"floorplan/placement fact no routing parameter can fix; it is "
              f"REPORTED, not gated, because a dense part hard against an edge "
              f"produces one on healthy boards and on human originals too.")
        for e in deficit[:8]:
            eaten = (' eaten_by ' + ', '.join(f'{n}({v})' for n, v in e['eaten_by'])
                     if e['eaten_by'] else '')
            print(f"  {e['ref']} {e['face']}: short {e['deficit_finest_grid']} "
                  f"lane(s) (demand {e['demand_nets']}, supply "
                  f"{e['supply_finest_grid']}){eaten}")
    elif ledgers:
        print("Faces in DEFICIT at the finest legal grid: 0")

    starved = _starved_faces(ledgers, args.min_demand)
    new_starved = None
    share = []          # stays empty without --baseline; the JSON reads it
    if args.baseline:
        try:
            base_pcb = parse_kicad_pcb(args.baseline)
        except Exception as exc:
            print(f"cannot parse baseline {args.baseline}: {exc}",
                  file=sys.stderr)
            return 2
        base_ledgers = {}
        for ref in refs:
            # THE SAME BAND ON BOTH SIDES. The gate is a delta, so a
            # baseline graded at a different depth is not a baseline -- it is
            # a second measurement being subtracted from the first.
            rows = routability.face_lane_ledger(
                base_pcb, ref, clearance=clearance, track_width=track,
                grid_step=grid, escape_band_mm=args.escape_band,
                pcb_file=args.baseline)
            if rows:
                base_ledgers[ref] = rows
        was = {(r, f) for r, f, _d in _starved_faces(base_ledgers,
                                                     args.min_demand)}
        new_starved = [t for t in starved if (t[0], t[1]) not in was]

        # A face whose supply went from something to NOTHING is new damage
        # whatever its demand is. --min-demand exists to keep the ABSOLUTE
        # form quiet on healthy boards, where a lightly-used face with no lane
        # is ordinary; it has no business filtering a DELTA, because the
        # baseline already carries the design's own habits. Measured: a repair
        # took one part's north face from supply 4 to supply 0 with demand 6,
        # and the gate exited 0 because 6 < 7.
        seen = {(t[0], t[1]) for t in new_starved}
        for ref, face, dem, before in lost_last_lane(ledgers, base_ledgers):
            if (ref, face) in seen:
                continue
            new_starved.append((ref, face, dem))
            seen.add((ref, face))
            print(f"  NEW (lost its last lane): {ref} {face}: demand {dem}, "
                  f"supply {before} -> 0. Below --min-demand "
                  f"{args.min_demand}, so only the delta sees it.")

        # #847: ...and the SHARE form, which is the one that catches the
        # damage the zero-crossing masks. See `lost_escape_share`: a face can
        # lose a third of its escape without reaching zero, and until now no
        # predicate here could see that -- which is how the gate ended up
        # deciding on an escape-band constant nobody had re-derived.
        share = []
        if args.min_supply_drop > 0:
            share = lost_escape_share(ledgers, base_ledgers, args.min_demand,
                                      args.min_supply_drop)
        for ref, face, dem, before, now in share:
            if (ref, face) in seen:
                continue
            new_starved.append((ref, face, dem))
            seen.add((ref, face))
            print(f"  NEW (lost {100 * (1 - now / before):.0f}% of its "
                  f"escape): {ref} {face}: demand {dem}, supply {before} -> "
                  f"{now}. Still non-zero, so only the share form sees it.")
        # The `N NEW` token is KEPT verbatim: it is a published output shape
        # that tests and drivers parse, and rewording it to say "escape
        # damage" more precisely would have broken two of them for no gain.
        # What changed is the sentence around it -- "starved" was the whole
        # story until #847 and is now one of three channels.
        print(f"Starved faces (zero supply at the finest grid, demand >= "
              f"{args.min_demand}): {len(starved)} now, {len(was)} on the "
              f"baseline. Escape damage, ALL channels: "
              f"{len(new_starved)} NEW")
        # ONLY the zero-supply channel gets the zero-supply sentence. The
        # other two printed their own line above, with their own numbers, and
        # this loop used to reprint every one of them as "supply 0 -- nothing
        # leaves this face" -- which for a share-form hit at supply 28 is a
        # false statement in the tool's own output. `new_starved` is a merged
        # list; the message may not assume which channel put a row in it.
        _zero = {(r, f) for r, f, _d in starved}
        for ref, face, dem in new_starved:
            if (ref, face) in _zero:
                print(f"  NEW: {ref} {face}: demand {dem}, supply 0 -- "
                      f"nothing leaves this face")
        if new_starved:
            # The wording is hedged on purpose. A face at supply 0 IS
            # unrescuable; a face that lost a third of its escape and still
            # has headroom is not, and saying so would contradict this tool's
            # own deficit report two lines above -- which can read 0 while
            # this reads 2. Both are true and they answer different questions.
            print("  These faces lost escape relative to the baseline -- "
                  "readable now rather than after the retries. A face at zero "
                  "supply is one the router cannot rescue; one that merely "
                  "lost a large share still has headroom, and is a trend "
                  "rather than a wall. Check the DEFICIT line above for which "
                  "of the two you have. Move what ate the span (see eaten_by) "
                  "or reconsider the arrangement.")

    channels = routability.pair_channel_widths(
        pcb, clearance=clearance, min_extent_mm=args.min_extent,
        pcb_file=args.board)
    print(f"Anchor channels (narrowest first, {len(channels)} pair(s)):")
    for row in channels[:12]:
        parts = (' parts: ' + ', '.join(row['parts_in_channel'])
                 if row['parts_in_channel'] else '')
        print(f"  {row['a']} <-> {row['b']}: {row['channel_mm']}mm{parts}")

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump({'board': args.board, 'clearance': clearance,
                       'track_width': track, 'grid_step': grid,
                       # ...and WHERE each came from, so two ledgers are
                       # comparable only when they say the same thing here.
                       'floors': floors,
                       # #847: the depth the neighbour search ran to, and the
                       # term that set it. Two ledgers are comparable only
                       # when they agree here as well as on `floors`.
                       'escape_band': {'value': round(_band.mm, 4),
                                       'source': _band.source,
                                       'basis': _band.basis,
                                       'lanes': _band.lanes,
                                       'floor_mm': _band.floor_mm},
                       # The threshold the starvation predicate ran at. It was
                       # absent, so two JSON files taken at different
                       # --min-demand were indistinguishable.
                       'min_demand': args.min_demand,
                       'taps_not_modeled': True,
                       'ledgers': ledgers, 'channels': channels,
                       'starved_faces': starved,
                       # #847: the share form's hits, kept as their own key
                       # rather than only folded into new_starved_faces, so a
                       # reader can tell WHICH predicate fired. Merging them
                       # is how the fixture's band table came to read as one
                       # phenomenon when it was two.
                       'lost_escape_share': [list(t) for t in share] if
                       args.baseline else None,
                       'min_supply_drop': args.min_supply_drop,
                       # run-12 Tier 3.6: the ABSOLUTE deficit, which the gate's
                       # starvation predicate (supply == 0 AND demand >=
                       # --min-demand) does not cover. Report-only.
                       'deficit_faces': deficit,
                       'baseline': args.baseline,
                       # HISTORICAL NAME. Since #847 this list is the union of
                       # three predicates and a row in it may have supply left,
                       # so it is no longer only "starved" faces. Kept because
                       # renaming a published key breaks every reader; the
                       # per-predicate keys beside it are the precise ones.
                       'new_starved_faces': new_starved},
                      f, indent=1, sort_keys=True)
        print(f"  JSON -> {args.json}")
    if args.gate and new_starved:
        return 4
    if args.gate and not ledgers:
        # A gate that examined nothing must not answer "clean". This board
        # auto-detected no fine-pitch part, printed one parenthetical note,
        # and exited 0 -- which a caller reasonably recorded as "no starved
        # face". A component nothing looked at is UNEXAMINED, never clean.
        print("  GATE DID NOT RUN: no part had a lane ledger to measure "
              "(none auto-detected, none passed with --refs). This is not a "
              "pass. Name the parts whose escape faces matter -- "
              "--refs U1 U2 -- or record that this board has none.")
        return 3
    return 0


if __name__ == "__main__":
    import cli_banner
    cli_banner.install()   # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())
