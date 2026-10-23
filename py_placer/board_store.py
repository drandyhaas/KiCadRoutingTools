#!/usr/bin/env python3
"""A content-addressed board store, and the append-only ledger beside it.

A run is a sequence of attempts, most of which are reverted. Each recorded
board is kept by CONTENT, so "the board of iteration 7" is exact rather than a
path that was overwritten twice. Observed failure this replaces: a real ledger
recorded `reverted_to` values of `"wk/seed.kicad_pcb"`, `"grid back to 0.1"`
and `"iteration 7's chain, plus the Step 5c reconnect"` -- two of three
unusable by a program, and the third naming a path that three iterations had
written in turn.

Nothing here knows what a board IS. It stores bytes with their siblings and
records what produced them.
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import hashlib
import json
import os
import shutil
import time
from typing import Dict, List, Optional

# Siblings that make a board mean what it means: the project carries the DRC
# floor and the net classes, the rules file carries custom DRC. A store that
# keeps only the .kicad_pcb hands back a board that grades differently.
# ONE list, imported (#711). Every hand-written copy of this tuple is a
# place a new sibling gets stranded; there were nine of them.
from copy_board import SIBLING_EXTS as SIBLING_EXT  # noqa: E402


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


class BoardStore:
    """Boards by content hash, siblings included: put() copies the board and
    every sibling under <root>/<sha>/ (as board.kicad_pcb, board.kicad_pro,
    ...) and returns the sha."""

    def __init__(self, root: str):
        self.root = root
        os.makedirs(root, exist_ok=True)

    def _dir(self, sha: str) -> str:
        return os.path.join(self.root, sha)

    def put(self, board: str) -> str:
        sha = sha256_file(board)
        d = self._dir(sha)
        if os.path.isdir(d):
            return sha                      # content-addressed: already have it
        tmp = d + '.partial'
        os.makedirs(tmp, exist_ok=True)
        base = os.path.splitext(board)[0]
        shutil.copyfile(board, os.path.join(tmp, 'board.kicad_pcb'))
        for ext in SIBLING_EXT:
            src = base + ext
            if os.path.isfile(src):
                shutil.copyfile(src, os.path.join(tmp, 'board' + ext))
        os.replace(tmp, d)                  # atomic: a killed run leaves .partial
        return sha


class Ledger:
    """Append-only JSONL of what was tried, kept next to the store.

    JSONL, not one JSON document: a run that is killed mid-iteration leaves a
    readable ledger of everything before it, and appending never rewrites what
    is already on disk.
    """

    def __init__(self, path: str):
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)

    def append(self, entry: Dict) -> Dict:
        entry = dict(entry)
        entry.setdefault('t', time.time())
        with open(self.path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(entry, sort_keys=True) + '\n')
        return entry

    def entries(self) -> List[Dict]:
        if not os.path.isfile(self.path):
            return []
        out = []
        with open(self.path, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        doc = json.loads(line)
                    except ValueError:
                        continue            # a torn last line never loses the rest
                    # A LINE IS ONLY A ROW IF IT IS AN OBJECT (#1078). A stray
                    # `42` parses fine, and then every reader's `.get` raised,
                    # so one bad line made every `verdict` on the ledger exit 1.
                    if isinstance(doc, dict):
                        out.append(doc)
        return out

    def next_iteration(self) -> int:
        """The `iteration` a new row takes: past every number already used.

        Not `len(entries())`. A line that is not a row (a torn one, or since
        #1078 a stray `42`) is not counted, but rows written after it by the
        old count were numbered past it, so the count fell back onto a number
        a row already holds, and two rows then answered to one iteration. On a
        ledger numbered 0..n-1 this IS the count.
        """
        rows = self.entries()
        used = [e.get('iteration') for e in rows]
        return max([len(rows)] + [i + 1 for i in used
                                  if isinstance(i, int)
                                  and not isinstance(i, bool)])

    def last_accepted(self) -> Optional[Dict]:
        for e in reversed(self.entries()):
            if e.get('accepted'):
                return e
        return None
