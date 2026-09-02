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
literally: with `--gate` the exits are **4** (a NEW starved face against
`--baseline`) and **3** (nothing had a ledger, so the gate did not run and must
not be recorded as a pass); **2** is an unreadable board, with or without the
flag. Without `--gate` it is still exit 0 throughout.

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
                        "damaged input). Enables the E8 gate: faces that lose "
                        "ALL escape supply while carrying real demand, and did "
                        "not already do so on the baseline.")
    p.add_argument("--gate", action="store_true",
                   help="exit 4 when the --baseline comparison finds a NEW "
                        "starved face (default: report only)")
    p.add_argument("--escape-band", type=float, default=None, metavar="MM",
                   help="how deep off a face to look for neighbours that eat "
                        "its lanes. Default: the board's own lane pitch via "
                        "placement.escape.escape_band (4 x pitch, floored at "
                        "1mm). #847: this was unreachable from any shipped "
                        "entry point, so the one parameter that decides the "
                        "starved-face gate could not be varied without "
                        "editing source")
    p.add_argument("--min-demand", type=int, default=GATE_MIN_DEMAND,
                   help="nets a face must carry before zero supply counts as "
                        "starvation (default: %(default)s)")
    args = p.parse_args()

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
        print(f"Starved faces (zero supply at the finest grid, demand >= "
              f"{args.min_demand}): {len(starved)} now, {len(was)} on the "
              f"baseline, {len(new_starved)} NEW")
        for ref, face, dem in new_starved:
            print(f"  NEW: {ref} {face}: demand {dem}, supply 0 -- nothing "
                  f"leaves this face")
        if new_starved:
            print("  A face that carries real demand and has no lane left is "
                  "a placement the router cannot rescue, readable now rather "
                  "than after the retries. Move what ate the span (see "
                  "eaten_by above) or reconsider the arrangement.")

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
                       # run-12 Tier 3.6: the ABSOLUTE deficit, which the gate's
                       # starvation predicate (supply == 0 AND demand >=
                       # --min-demand) does not cover. Report-only.
                       'deficit_faces': deficit,
                       'baseline': args.baseline,
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
