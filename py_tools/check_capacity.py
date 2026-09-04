#!/usr/bin/env python3
"""Can this board hold its parts -- and if not, what are the options?

Six places in this toolchain promise the same sentence:

    the honest response to a board that is genuinely too small is to say so
    with the measured number and stop

and the measured number is computed in none of them. This computes it, plus
the other four levers a too-tight board actually has: more copper layers, a
neighbour moved off a starved face, a tighter clearance (floored at what the
fab can etch), and a smaller package.

IT REPORTS. It never refuses and it never acts -- no outline is written, no
stackup is edited, no part is moved. That is deliberate and it matches
`recommend-stackup`, which proposes the nearest workable option and says
"do not modify the board file directly -- stackup is a fab-facing decision
the user must own".

    python3 check_capacity.py board.kicad_pcb
    python3 check_capacity.py board.kicad_pcb --json capacity.json
    python3 check_capacity.py board.kicad_pcb --only grow_board add_layers

Exit codes: 0 = measured (whatever the answer), 2 = usage/load error,
3 = no outline, so there is no capacity question to ask.

There is deliberately NO exit code for "the board is too small". A gate that
refuses on this would refuse boards that route fine -- the area test is a
necessary condition, not a sufficient one -- and the executor decides.
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import sys


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Measured capacity options for a board that may be too "
                    "small for its parts. Reports; never modifies.")
    p.add_argument("board")
    p.add_argument("--json", metavar="PATH", help="Write the full report here")
    p.add_argument("--clearance", type=float, default=None,
                   help="Default: this board's own Default netclass "
                        "clearance, never a guessed round number")
    p.add_argument("--board-edge-clearance", type=float, default=None)
    p.add_argument("--track-width", type=float, default=None)
    p.add_argument("--only", nargs="+", metavar="OPTION",
                   help="Measure only these options")
    p.add_argument("--intent", default=None, metavar="JSON",
                   help="Floorplan intent; only its `assembly.sides` is read")
    p.add_argument("--assembly-sides", default=None,
                   choices=("F", "B", "both"),
                   help="Which faces the fab will populate, without an intent "
                        "file. Overrides --intent. Undeclared (the default) "
                        "charges the parts against the BUSIER face; F or B "
                        "charges the sum, because every part has to fit on "
                        "the one face that gets populated")
    p.add_argument("-q", "--quiet", action="store_true")
    a = p.parse_args(argv)

    from kicad_parser import parse_kicad_pcb
    from placement.options import OPTIONS, capacity_options, format_text

    if a.only:
        bad = [o for o in a.only if o not in OPTIONS]
        if bad:
            p.error(f"unknown option(s) {bad}; known: {sorted(OPTIONS)}")

    try:
        pcb = parse_kicad_pcb(a.board)
    except Exception as e:                       # noqa: BLE001
        print(f"check_capacity: cannot read {a.board}: {e}", file=sys.stderr)
        return 2
    if pcb.board_info.board_bounds is None:
        print("check_capacity: this board has no Edge.Cuts outline, so there "
              "is no capacity question to ask. The outline is spec-owned.",
              file=sys.stderr)
        return 3

    try:
        from list_nets import board_floor_knobs
        clearance, edge, floors = board_floor_knobs(
            a.board, clearance=a.clearance,
            board_edge_clearance=a.board_edge_clearance)
    except Exception as e:                       # noqa: BLE001
        clearance = 0.25 if a.clearance is None else a.clearance
        edge = 0.55 if a.board_edge_clearance is None else a.board_edge_clearance
        floors = {'error': f"{type(e).__name__}: {e}"}

    # #837. Flag > intent > undeclared, and the SOURCE is printed, because
    # "the busier face was charged" and "the sum was charged" are different
    # answers to the same question and a reader must not have to guess which
    # one they are looking at. No auto-discovery of a sibling intent: nothing
    # in the tree discovers one, and a verdict that changed with an invisible
    # file next to the board would be the worst of both.
    sides, sides_src = a.assembly_sides, 'cli'
    if sides is None and a.intent:
        try:
            from placement.floorplan import load_intent
            declared = load_intent(a.intent).assembly_sides()
            # `assembly_sides()` resolves an absent key to 'both', which is
            # the same arithmetic as undeclared -- but the two are different
            # STATEMENTS, so the source says which one was read.
            sides, sides_src = declared, 'intent'
        except (OSError, ValueError) as e:
            print(f"cannot load intent {a.intent}: {e}", file=sys.stderr)
            return 2
    if sides is None:
        sides_src = 'not declared'
    if not a.quiet:
        print(f"  assembly sides: {sides or 'not declared'}  [{sides_src}]"
              + ("" if sides in ('F', 'B') else
                 " -- the busier face is charged, so back-side area is "
                 "credited free"))

    opts = capacity_options(pcb, a.board, clearance=clearance,
                            board_edge_clearance=edge,
                            track_width=a.track_width,
                            assembly_sides=sides, only=a.only)
    if not a.quiet:
        print(format_text(opts))

    report = {'schema': 1, 'kind': 'capacity-options', 'board': a.board,
              'clearance': clearance, 'board_edge_clearance': edge,
              'floors': floors, 'options': opts}
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=1, sort_keys=True, default=str)
        print(f"Wrote {a.json}")

    grow = opts.get('grow_board') or {}
    print("JSON_SUMMARY: " + json.dumps({
        'board': a.board,
        'fits_by_area': grow.get('fits_by_area'),
        'utilisation': (grow.get('measured') or {}).get('utilisation'),
        # #837, forwarded explicitly: this dict is a hand-written literal, so
        # a new `measured` key does not arrive here on its own -- and a
        # utilisation whose basis the one-line machine channel cannot state is
        # a number a loop driver will compare against one taken on the other
        # basis.
        'assembly_sides': sides,
        'charged_area_is_sum':
            (grow.get('measured') or {}).get('charged_area_is_sum'),
        'shortfall_mm2_at_least':
            (grow.get('measured') or {}).get('shortfall_mm2_at_least'),
        'measured': sorted(k for k, v in opts.items() if v.get('ran')),
        'not_measured': sorted(k for k, v in opts.items() if not v.get('ran')),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
