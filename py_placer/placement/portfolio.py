"""Sibling copies of a written board.

`copy_siblings` copies a board's sibling files (`.kicad_pro`, `.kicad_dru`,
the design brief, ...) next to a board a placement tool wrote, so the next
step reads the same DRC floor and declarations (#441).
"""
from __future__ import annotations

import os
import shutil


def copy_siblings(src_board: str, dst_board: str) -> None:
    """Carry the project/rules siblings (#441): a board without its .kicad_pro
    grades at the stock netclass, so every written candidate gets the input's
    siblings -- the probe routes and any later adoption read them."""
    src_base = os.path.splitext(src_board)[0]
    dst_base = os.path.splitext(dst_board)[0]
    from copy_board import SIBLING_EXTS   # ONE list (#711)
    for ext in SIBLING_EXTS:
        s, d = src_base + ext, dst_base + ext
        if not os.path.isfile(s):
            continue
        # IN PLACE the sibling is already its own destination. `copyfile`
        # raises SameFileError there (PermissionError via copy2 on Windows),
        # and every in-place run crashed AFTER writing its board (e.g.
        # beautify_labels X X). `samefile`, not an abspath
        # compare, so a case-only spelling on a case-insensitive disk counts.
        if os.path.exists(d) and os.path.samefile(s, d):
            continue
        shutil.copyfile(s, d)
