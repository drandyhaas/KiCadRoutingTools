#!/usr/bin/env python3
"""Copy a KiCad board *together with its sibling project files* (#441).

A bare ``cp src.kicad_pcb dst.kicad_pcb`` copies only the board and strands the
sibling ``dst.kicad_pro`` -- which carries the DRC floor (the Default-netclass
clearance/track/via the board was routed to). The next routing step then reads no
project, resolves its floor from the STOCK (looser) netclass, and its writeback
stamps that looser floor over copper routed tighter, so KiCad grades correct
sub-floor copper as a clearance violation (icepi_zero: a dropped 0.09 floor became
0.10 -> 160 phantom grazes graded by kicad-cli).

Use this instead of ``cp`` whenever you rename/duplicate a board mid-chain:

    python3 copy_board.py src.kicad_pcb dst.kicad_pcb

It copies ``.kicad_pcb`` and every sibling that exists (``.kicad_pro``,
``.kicad_prl``, the ``_planes.json`` plane-net manifest), so the DRC floor and
the plane declaration travel with the board. It also self-records into
the stress redo manifest (``REDO_MANIFEST``) like the routing tools, so a replayed
manifest reproduces the full copy (not just the board) -- no more floor drop.
"""
import os
import shutil
import sys

# Sibling suffixes that must travel with a board (each appended to the board's
# stem). .kicad_pro is the DRC floor (the whole point); .kicad_prl is per-board
# local state (harmless to carry); _planes.json is the plane-net manifest
# sidecar (plane_io) -- dropping it makes later wildcard route steps re-route
# plane nets as signals (#479).
SIBLING_EXTS = (".kicad_pro", ".kicad_prl", "_planes.json")


def copy_board(src_pcb: str, dst_pcb: str) -> list:
    """Copy ``src_pcb`` to ``dst_pcb`` plus every existing sibling. Returns the list
    of copied paths. Raises FileNotFoundError if the source board is missing."""
    if not src_pcb.endswith(".kicad_pcb") or not dst_pcb.endswith(".kicad_pcb"):
        raise ValueError("copy_board expects .kicad_pcb source and destination paths")
    if not os.path.isfile(src_pcb):
        raise FileNotFoundError(src_pcb)
    copied = []
    shutil.copy2(src_pcb, dst_pcb)
    copied.append(dst_pcb)
    src_base = src_pcb[: -len(".kicad_pcb")]
    dst_base = dst_pcb[: -len(".kicad_pcb")]
    for ext in SIBLING_EXTS:
        s = src_base + ext
        if os.path.isfile(s):
            shutil.copy2(s, dst_base + ext)
            copied.append(dst_base + ext)
    if not any(p.endswith(".kicad_pro") for p in copied):
        # Not fatal, but the caller should know the floor did not travel.
        print(f"WARNING: '{os.path.basename(src_pcb)}' has no sibling .kicad_pro; "
              f"copied the board only. The DRC floor is undefined for '{os.path.basename(dst_pcb)}' "
              f"and later steps will fall back to the stock netclass (#441).")
    return copied


def main() -> int:
    # Self-record like the routing tools so a replayed manifest reproduces the full
    # copy (board + siblings), not just the board.
    try:
        from redo_record import record_invocation
        record_invocation()
    except Exception:
        pass
    import argparse
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("src", help="source .kicad_pcb")
    p.add_argument("dst", help="destination .kicad_pcb")
    args = p.parse_args()
    try:
        copied = copy_board(args.src, args.dst)
    except (FileNotFoundError, ValueError) as e:
        print(f"copy_board: error: {e}", file=sys.stderr)
        return 1
    print(f"Copied {len(copied)} file(s): {', '.join(os.path.basename(c) for c in copied)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
