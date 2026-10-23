"""
Small file-IO helpers shared by the routing/fanout entrypoints.

Kept dependency-free (only os/shutil) so every entrypoint can import it
without pulling in heavier modules or risking a circular import.
"""
from __future__ import annotations

import os
import shutil


def passthrough_copy(input_file: str, output_file: str) -> bool:
    """Copy ``input_file`` to ``output_file`` unless they are the same file.

    The routing and fanout stages call this to write the board through
    *unchanged* when a stage produced no new copper, so a chained pipeline
    (output -> next stage's input) keeps a valid file instead of dying on a
    missing one.

    Under ``--overwrite`` (and any explicit in-place ``--output``) the output
    path *is* the input path: the board is already there, and a self-copy with
    ``shutil.copy*`` raises ``shutil.SameFileError`` (issue #249). Detect that
    with :func:`os.path.samefile` -- an inode compare that also catches the
    same-file-via-symlink/hardlink case -- and skip the copy. ``samefile``
    raises when ``output_file`` does not exist yet (the normal pass-through to a
    fresh path), so fall back to an ``abspath`` compare there.

    Returns ``True`` if a copy was made, ``False`` if it was skipped (no output
    path, or source and destination are the same file).
    """
    if not output_file:
        return False
    try:
        same = os.path.samefile(input_file, output_file)
    except OSError:
        same = os.path.abspath(input_file) == os.path.abspath(output_file)
    if same:
        return False
    # The board's siblings go with it (#441: never a board without its
    # .kicad_pro), seeded BEFORE the board exists exactly as every routing
    # writer seeds them (seed_project_for_output, #513 item 12). A bare copy
    # left the output project-less until main()'s writeback, so everything that
    # read the output's rules mid-run read NOTHING: on neo6502's GND step
    # ("nothing to route", the board passed through to the plane finalize)
    # every copper-to-hole lookup of the KiCad oracle and of the final
    # reconciliation returned 0.0 against a board declaring 0.25, and their
    # copper was held only at the 0.20 NPTH fab floor. Lazy import: this
    # module stays light for every entrypoint.
    from fix_kicad_drc_settings import seed_project_for_output
    seed_project_for_output(output_file, input_file)
    shutil.copyfile(input_file, output_file)
    return True
