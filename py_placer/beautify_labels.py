"""
Beautify reference-designator silkscreen (#481).

Usage:
  python beautify_labels.py input.kicad_pcb [output.kicad_pcb] [options]

Places every visible Reference label at a uniform font size and orientation,
near its own part, avoiding pads, drill holes, other labels, and the board
edge. Copper is byte-for-byte untouched -- only the Reference text sub-nodes
are edited. A label that cannot be placed legally is left exactly as it was
and reported with a reason.

See placement/labels.py for the obstacle model and degradation ladder.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement'], 'kind': 'actor'}

import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

import json
import os
import sys

from kicad_parser import parse_kicad_pcb


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Tidy reference-designator silkscreen labels (#481).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python beautify_labels.py input.kicad_pcb labeled.kicad_pcb
  python beautify_labels.py input.kicad_pcb --orientation vertical --refs "C*" "R*"

Exit codes:
  0  every in-scope label placed
  2  argument error
  3  board-state gate refused (see --allow-unplaced)
  4  completed, but >= 1 label could not be placed (left unchanged, reported)
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?",
                        help="Output file (default: input_labels.kicad_pcb)")
    parser.add_argument("--size", type=float, default=1.0,
                        help="Target label font size in mm (default: 1.0)")
    parser.add_argument("--thickness", type=float, default=0.15,
                        help="Stroke thickness at target size in mm "
                             "(default: 0.15; ratio-scaled when a label "
                             "shrinks, floor 0.1)")
    parser.add_argument("--min-size", type=float, default=0.7,
                        help="Smallest size the shrink ladder may reach in mm "
                             "(default: 0.7)")
    parser.add_argument("--size-step", type=float, default=0.05,
                        help="Shrink ladder step in mm (default: 0.05)")
    parser.add_argument("--orientation", choices=("horizontal", "vertical"),
                        default="horizontal",
                        help="Preferred label orientation (default: "
                             "horizontal). Keep-upright world angles are "
                             "always in {0, 90}: upside-down is impossible "
                             "by construction")
    parser.add_argument("--no-rotate-fallback", action="store_true",
                        help="Never try the non-preferred orientation")
    parser.add_argument("--max-distance", type=float, default=2.0,
                        help="Max distance the anchor ring may grow from the "
                             "part's courtyard in mm (default: 2.0)")
    parser.add_argument("--anchor-gap", type=float, default=0.2,
                        help="Base gap between courtyard and label in mm "
                             "(default: 0.2)")
    parser.add_argument("--silk-pad-clearance", type=float, default=0.15,
                        help="Min gap from label ink to pad copper / drill "
                             "holes in mm (default: 0.15)")
    parser.add_argument("--label-clearance", type=float, default=0.15,
                        help="Min gap between two labels in mm (default: 0.15)")
    parser.add_argument("--board-edge-clearance", type=float, default=None,
                        help="Hard clearance from the board edge in mm "
                             "(default: the board's own "
                             "min_copper_edge_clearance, else 0.55)")
    parser.add_argument("--include-hidden", action="store_true",
                        help="Also place hidden labels (they stay hidden; "
                             "only their pose/size is tidied)")
    parser.add_argument("--skip-locked", action="store_true",
                        help="Leave locked footprints' labels untouched "
                             "(off by default: a footprint lock pins the "
                             "PART, not its silkscreen)")
    parser.add_argument("--refs", nargs="+", default=None, metavar="PATTERN",
                        help="Glob patterns of references to beautify "
                             "(e.g. C* R*); out-of-scope labels stay put and "
                             "act as obstacles (default: all)")
    parser.add_argument("--report-json", metavar="PATH",
                        help="Write the full per-label results as JSON")
    parser.add_argument("--allow-unplaced", action="store_true",
                        help="Run even when the board does not look placed "
                             "(parts stacked at one coordinate) -- labels "
                             "placed around a pile are meaningless")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print each label's outcome")

    args = parser.parse_args()

    if args.output_file is None:
        base, ext = os.path.splitext(args.input_file)
        args.output_file = base + '_labels' + ext

    # This tool MUTATES the board (silkscreen only, but a replayed manifest
    # still needs the step so the next input isn't produced by nothing).
    try:
        from redo_record import record_invocation
        record_invocation()
    except Exception:
        pass

    print(f"Loading {args.input_file}...")
    pcb_data = parse_kicad_pcb(args.input_file)

    # Board-state gate (#431). allow_routed is ALWAYS on: beautify touches no
    # copper, so running it on a routed board is legitimate -- the unplaced
    # gate is the one that matters here.
    from placement.placement_state import gate_or_exit
    gate_or_exit(pcb_data, args.input_file, 'beautify_labels.py',
                 allow_unplaced=args.allow_unplaced, allow_routed=True)
    print(f"Output file: {args.output_file}")

    from placement.labels import LabelConfig, beautify_labels
    config = LabelConfig(
        size=args.size,
        thickness=args.thickness,
        min_size=args.min_size,
        size_step=args.size_step,
        orientation=args.orientation,
        allow_rotate_fallback=not args.no_rotate_fallback,
        max_distance=args.max_distance,
        anchor_gap=args.anchor_gap,
        silk_pad_clearance=args.silk_pad_clearance,
        label_clearance=args.label_clearance,
        board_edge_clearance=args.board_edge_clearance,
        include_hidden=args.include_hidden,
        skip_locked=args.skip_locked,
        refs=args.refs,
    )

    summary = {'labels_total': 0}
    out = beautify_labels(pcb_data, args.input_file, config)
    results = out['results']
    summary.update(out['summary'])

    if args.verbose:
        for res in results:
            print(f"  {res.reference}: {res.action}"
                  + (f" ({res.reason})" if res.reason else ""))

    from placement.writer import write_label_output
    write_label_output(args.input_file, args.output_file, results)
    # #441: carry the .kicad_pro/.kicad_dru siblings so later steps grade at
    # the board's real floors, not the stock netclass.
    from placement.portfolio import copy_siblings
    copy_siblings(args.input_file, args.output_file)

    unplaced = [r for r in results if r.action == 'unchanged']
    if unplaced:
        print(f"{len(unplaced)} label(s) could NOT be placed "
              f"(left byte-identical):")
        for res in unplaced:
            print(f"  {res.reference}: {res.reason}")

    if args.report_json:
        from dataclasses import asdict
        with open(args.report_json, 'w', encoding='utf-8') as f:
            json.dump([asdict(r) for r in results], f, indent=1,
                      sort_keys=True)
        print(f"Wrote {args.report_json}")

    print('JSON_SUMMARY: ' + json.dumps(summary, sort_keys=True, default=str),
          flush=True)
    # Documented in the epilog: 4 = completed but >= 1 label unplaceable, so
    # a caller can tell "tidy board" from "tidy board with leftovers" without
    # parsing the summary.
    return 4 if unplaced else 0


if __name__ == "__main__":
    import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main() or 0)
