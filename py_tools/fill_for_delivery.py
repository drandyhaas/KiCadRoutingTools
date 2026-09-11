#!/usr/bin/env python3
"""Write a FILLED copy of a routed board, for delivery (#910).

The routed deliverable carries zone OUTLINES with no `(filled_polygon ...)`:
`kicad_writer` writes the `(fill yes ...)` properties, and nothing in the plane
path ever writes a fill. Opened in KiCad before a refill -- or graded by
`kicad-cli pcb drc` WITHOUT `--refill-zones` -- such a board reports plane-net
opens that are not real. Measured on run 25's `routed.kicad_pcb`: 5 unconnected
(all GND) without the flag, 0 with it.

This is the opt-in delivery step. It runs KiCad's own ZONE_FILLER through the
bundled interpreter, on a STAGED copy in a temp dir, and saves with
`aSkipSettings=True` (a plain `SaveBoard` rewrites the project from KiCad's
in-memory view, deleting every non-Default net class, and aborts outright on a
pre-KiCad-10 project). The destination's `.kicad_pro` is written here by
`copy_board` and is never in the fill's reach -- and this tool AUDITS that
afterwards rather than asserting it, refusing if a class went missing.

Usage:
    python3 py_tools/fill_for_delivery.py routed.kicad_pcb -o delivered.kicad_pcb

Exit codes:
    0  filled, net classes intact
    1  the fill did not run, or a net class went missing

On exit 1 the destination holds the board and its siblings COPIED but not
filled -- that copy is useful (grade it with `kicad-cli pcb drc
--refill-zones`, or press B in KiCad) and the run says so -- except on the
net-class refusal, where the destination is removed entirely, because a board
graded against rules it no longer carries is worse than no board.
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'actor'}

import _path  # noqa: F401  (#522: puts ../py_router on sys.path)

import argparse
import json
import os
import sys


def _netclass_names(pcb_path):
    """The net-class names in the sibling .kicad_pro, or None if there is none."""
    pro = os.path.splitext(pcb_path)[0] + '.kicad_pro'
    if not os.path.isfile(pro):
        return None
    try:
        with open(pro, encoding='utf-8') as fh:
            doc = json.load(fh)
    except Exception:
        return None
    nc = (doc.get('net_settings') or {}).get('classes') or []
    return {c.get('name') for c in nc if isinstance(c, dict)}


def _unconnected(pcb_path):
    """kicad-cli's unconnected count WITHOUT `--refill-zones`, or None.

    Deliberately without the flag: that is the number a reviewer sees when
    they open or grade the deliverable as shipped, and the whole point of
    this step is to make it agree with the refilled one.
    """
    import subprocess
    import tempfile
    try:
        from kicad_unconnected import find_kicad_cli
    except Exception:
        return None
    cli = find_kicad_cli()
    if cli is None:
        return None
    out = None
    try:
        with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as fh:
            out = fh.name
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json',
                        '--severity-error', '-o', out, pcb_path],
                       capture_output=True, text=True, timeout=600)
        with open(out, encoding='utf-8') as fh:
            return len(json.load(fh).get('unconnected_items') or [])
    except Exception:
        return None
    finally:
        if out:
            try:
                os.unlink(out)
            except OSError:
                pass


def main() -> int:
    from redo_record import record_invocation
    record_invocation()

    ap = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('input_file', help='routed .kicad_pcb')
    ap.add_argument('-o', '--output', required=True,
                    help='destination .kicad_pcb (siblings are copied too)')
    ap.add_argument('--timeout', type=int, default=None,
                    help='seconds to allow the KiCad fill (default: the '
                         'exact-fill budget)')
    ap.add_argument('--exit-zero', action='store_true',
                    help='report problems but exit 0')
    args = ap.parse_args()

    if not os.path.isfile(args.input_file):
        print(f"error: no such board: {args.input_file}")
        return 0 if args.exit_zero else 1
    if os.path.abspath(args.input_file) == os.path.abspath(args.output):
        # A natural thing to try, and copy_board would raise SameFileError at
        # the user. `route.py --write-fill` is the in-place form.
        print("error: -o names the input file. Use `route.py --write-fill` to "
              "fill a board in place, or give a different -o.")
        return 0 if args.exit_zero else 1

    from copy_board import copy_board
    from kicad_exact_fill import write_filled_board, EXACT_FILL_TIMEOUT

    want = _netclass_names(args.input_file)
    before = _unconnected(args.input_file)

    # Siblings FIRST (.kicad_pro / .kicad_prl / .kicad_dru / design brief):
    # the fill is graded against the project's real netclasses, and #441 is
    # what happens when a board travels without them.
    copy_board(args.input_file, args.output)

    st = write_filled_board(args.input_file, args.output, verbose=True,
                            timeout=args.timeout or EXACT_FILL_TIMEOUT,
                            project_from=args.input_file)
    if not st.ok:
        print(f"FILL NOT WRITTEN: {st.reason}"
              + (f" ({st.detail})" if st.detail else ''))
        print("  The copied board is unfilled; grade it with "
              "`kicad-cli pcb drc --refill-zones` or press B in KiCad.")
        return 0 if args.exit_zero else 1

    # The trap this step exists to avoid, checked rather than assumed: a save
    # that rewrites the project deletes every non-Default class.
    got = _netclass_names(args.output)
    if want is not None and got is not None and not want.issubset(got):
        lost = sorted(want - got)
        print(f"REFUSING: net class(es) lost in the filled output: "
              f"{', '.join(lost)}")
        # The SIBLINGS go too: a `.kicad_pro` / `.kicad_dru` left behind with
        # no board is debris a later step can pick up as a DRC floor.
        from copy_board import SIBLING_EXTS
        _stem = os.path.splitext(args.output)[0]
        for _ext in ('.kicad_pcb',) + tuple(SIBLING_EXTS):
            try:
                os.unlink(_stem + _ext)
            except OSError:
                pass
        return 0 if args.exit_zero else 1

    after = _unconnected(args.output)
    n_fill = 0
    try:
        with open(args.output, encoding='utf-8', errors='replace') as fh:
            n_fill = fh.read().count('(filled_polygon')
    except OSError:
        pass
    print(f"Filled: {args.output}")
    print(f"  filled_polygon blocks: {n_fill}")
    if want is not None:
        print(f"  net classes preserved: {len(want)}")
    if before is not None and after is not None:
        # The record that the fill did not CHANGE connectivity, only revealed
        # it -- the number a reviewer would otherwise have to take on trust.
        print(f"  unconnected (no --refill-zones): {before} -> {after}")
    return 0


if __name__ == '__main__':
    import cli_banner
    cli_banner.install()
    sys.exit(main())
