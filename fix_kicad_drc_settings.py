#!/usr/bin/env python3
"""Fix the DRC settings of a KiCad output board so a manual DRC shows only the
relevant (routing) errors, not stock-default noise.

KiCad stores DRC *design rules* and *violation severities* in the PROJECT file
(``.kicad_pro``), NOT in the board (``.kicad_pcb``). A freshly written board gets
a project with KiCad's stock defaults, which produce two kinds of noise:

  1. ``min_hole_clearance`` defaults to 0.25 mm -- far stricter than the copper
     clearance the board was routed to -- so every fanout via drill near another
     net's copper fires a "Hole clearance" violation (hundreds of them). They are
     spurious at the real manufacturing floor; ``check_drc`` never reports them.
  2. Placement / fabrication categories (courtyard overlaps, solder-mask bridges)
     fire as errors even though they are not routing problems.

This script rewrites the sibling ``.kicad_pro`` to:
  * set ``min_hole_clearance`` to the board's copper-clearance floor (so only
    genuinely-too-close drills remain), and
  * set the courtyard / solder-mask severities to ``ignore``.

Clearance, shorting_items and unconnected_items are left untouched (KiCad shows
unconnected items in their own tab).

IMPORTANT: close the board in KiCad before running this. KiCad keeps the project
in memory and will overwrite an externally-edited ``.kicad_pro`` on save/close.

Usage:
    python3 fix_kicad_drc_settings.py board.kicad_pcb [options]

Options:
    --hole-clearance MM   Hole-clearance floor (default: the project's copper
                          clearance -- net Default class, else rules.min_clearance).
    --keep-courtyards     Do not ignore courtyard categories.
    --keep-mask           Do not ignore solder-mask bridge.
    --ignore CAT [CAT...] Additional severity categories to set to "ignore".
    --dry-run             Print what would change without writing.
"""
import argparse
import json
import os
import sys

# Severity categories treated as non-routing noise by default.
COURTYARD_CATS = ["courtyards_overlap", "malformed_courtyard",
                  "npth_inside_courtyard", "pth_inside_courtyard"]
MASK_CATS = ["solder_mask_bridge"]
# Footprint / library-geometry issues inherited from the source board's
# footprints (annular rings, pad/footprint library mismatches). The router does
# not create or fix these, so they are pure noise when reviewing a routed board
# -- on the stress boards they dominate the report (e.g. 199 annular_width + 149
# lib_footprint markers on orangecrab). Ignored by default; --keep-footprint
# restores them.
FOOTPRINT_CATS = ["annular_width", "lib_footprint_issues", "lib_footprint_mismatch"]


def find_project(path: str) -> str:
    """Return the .kicad_pro path for a .kicad_pcb / .kicad_pro / base path."""
    base, ext = os.path.splitext(path)
    pro = path if ext == ".kicad_pro" else base + ".kicad_pro"
    return pro


def project_copper_clearance(proj: dict):
    """The board's copper clearance: the Default netclass clearance, else
    rules.min_clearance. Returns None if neither is set (>0)."""
    for cls in proj.get("net_settings", {}).get("classes", []):
        if cls.get("name") == "Default" and cls.get("clearance"):
            return cls["clearance"]
    classes = proj.get("net_settings", {}).get("classes", [])
    if classes and classes[0].get("clearance"):
        return classes[0]["clearance"]
    mc = proj.get("board", {}).get("design_settings", {}).get("rules", {}).get("min_clearance")
    return mc if mc else None


def main():
    ap = argparse.ArgumentParser(description="Fix KiCad DRC settings on an output board's project file.")
    ap.add_argument("board", help="Path to the .kicad_pcb (or .kicad_pro) file")
    ap.add_argument("--hole-clearance", type=float, default=None,
                    help="Hole-clearance floor in mm (default: copper clearance)")
    ap.add_argument("--keep-courtyards", action="store_true", help="Do not ignore courtyard categories")
    ap.add_argument("--keep-mask", action="store_true", help="Do not ignore solder-mask bridge")
    ap.add_argument("--keep-footprint", action="store_true",
                    help="Do not ignore footprint/library categories (annular_width, lib_footprint_*)")
    ap.add_argument("--ignore", nargs="+", default=[], metavar="CAT",
                    help="Extra severity categories to set to ignore")
    ap.add_argument("--ignore-warnings", action="store_true",
                    help="Set every category currently at 'warning' severity to 'ignore' "
                         "(hides all warning markers; errors are untouched)")
    ap.add_argument("--dry-run", action="store_true", help="Show changes without writing")
    args = ap.parse_args()

    pro = find_project(args.board)
    if not os.path.isfile(pro):
        sys.exit(f"error: no project file found at {pro}\n"
                 f"  Open the board in KiCad once (it creates the .kicad_pro), then re-run.")

    with open(pro) as f:
        proj = json.load(f)

    ds = proj.setdefault("board", {}).setdefault("design_settings", {})
    rules = ds.setdefault("rules", {})
    sev = ds.setdefault("rule_severities", {})

    changes = []

    # 1) Hole clearance -> copper-clearance floor.
    floor = args.hole_clearance
    if floor is None:
        floor = project_copper_clearance(proj)
    if floor is None:
        print("warning: could not determine copper clearance; pass --hole-clearance MM", file=sys.stderr)
    else:
        old = rules.get("min_hole_clearance")
        if old != floor:
            changes.append(f"min_hole_clearance: {old} -> {floor} mm")
            rules["min_hole_clearance"] = floor

    # 2) Severities -> ignore for non-routing categories.
    to_ignore = list(args.ignore)
    if not args.keep_courtyards:
        to_ignore += COURTYARD_CATS
    if not args.keep_mask:
        to_ignore += MASK_CATS
    if not args.keep_footprint:
        to_ignore += FOOTPRINT_CATS
    if args.ignore_warnings:
        to_ignore += [cat for cat, s in sev.items() if s == "warning"]
    for cat in to_ignore:
        if sev.get(cat) != "ignore":
            changes.append(f"severity[{cat}]: {sev.get(cat)} -> ignore")
            sev[cat] = "ignore"

    if not changes:
        print(f"{pro}: already up to date, nothing to change.")
        return

    print(f"{pro}:")
    for c in changes:
        print(f"  {c}")

    if args.dry_run:
        print("(dry run -- not written)")
        return

    with open(pro, "w") as f:
        json.dump(proj, f, indent=2)
        f.write("\n")
    print(f"\nWrote {pro}. Clearance / shorts / unconnected are unchanged.")
    print("NOTE: if the board is open in KiCad, close it first and reopen -- "
          "KiCad overwrites the project file on save.")


if __name__ == "__main__":
    main()
