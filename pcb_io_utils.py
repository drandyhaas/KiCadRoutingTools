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
    shutil.copyfile(input_file, output_file)
    return True
