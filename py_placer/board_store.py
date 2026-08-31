#!/usr/bin/env python3
"""A content-addressed board store, and a ledger a step back can actually use.

A convergence run is a sequence of attempts, most of which are reverted. What
makes the sequence worth keeping is the ability to go back to any point and
re-run it differently -- and that requires two things a prose ledger does not
give you:

    the board            addressed by content, so "revert to iteration 7" is
                         exact rather than a path that was overwritten twice
    the lever            as ARGV, so replaying is subprocess.run(entry.lever_argv)
                         rather than a human reading "grid back to 0.1" and
                         reconstructing the command

Observed failure this replaces: a real ledger recorded `reverted_to` values of
`"wk/seed.kicad_pcb"`, `"grid back to 0.1"` and `"iteration 7's chain, plus the
Step 5c reconnect"` -- two of three unusable by a program, and the third naming
a path that three iterations had written in turn. Nothing in that run was
replayable, so every step back was a reconstruction.

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
# place a new sibling gets stranded; there were nine of them, and
# tests/test_711_sibling_lists.py now refuses a tenth.
from copy_board import SIBLING_EXTS as SIBLING_EXT  # noqa: E402


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


class BoardStore:
    """Boards by content hash, siblings included.

    put()  -> sha, copying the board and every sibling under <root>/<sha>/
    get()  -> a path to a checked-out copy, so a caller can mutate it freely
    """

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

    def has(self, sha: str) -> bool:
        return os.path.isdir(self._dir(sha))

    def get(self, sha: str, dest: str) -> str:
        """Check a stored board out to `dest`, siblings alongside. Returns dest."""
        d = self._dir(sha)
        if not os.path.isdir(d):
            raise KeyError(f"no board {sha[:12]} in {self.root}")
        os.makedirs(os.path.dirname(os.path.abspath(dest)) or '.', exist_ok=True)
        shutil.copyfile(os.path.join(d, 'board.kicad_pcb'), dest)
        base = os.path.splitext(dest)[0]
        for ext in SIBLING_EXT:
            src = os.path.join(d, 'board' + ext)
            if os.path.isfile(src):
                shutil.copyfile(src, base + ext)
        return dest


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
                        out.append(json.loads(line))
                    except ValueError:
                        pass                # a torn last line never loses the rest
        return out

    def last_accepted(self) -> Optional[Dict]:
        for e in reversed(self.entries()):
            if e.get('accepted'):
                return e
        return None

    def counts(self) -> Dict[str, int]:
        """completion vs placement vs systemic -- 'am I converging on the
        board, its placement, or the tooling?'. A run once spent nine of
        eleven iterations on the instrument and finished with five nets
        carrying no copper; two later runs filed PLACEMENT repairs (which
        connect nothing and tune no instrument) as systemic for want of a
        kind, and the systemic-share warning cried wolf on both."""
        out = {'total': 0, 'completion': 0, 'placement': 0, 'systemic': 0,
               'accepted': 0}
        for e in self.entries():
            out['total'] += 1
            out[e.get('kind', 'completion')] = out.get(e.get('kind', 'completion'), 0) + 1
            if e.get('accepted'):
                out['accepted'] += 1
        return out


def replay_command(entry: Dict) -> List[str]:
    """The argv that produced an entry -- the whole point of recording it.

    Raises rather than guessing: an entry without a lever_argv is not
    replayable, and pretending otherwise is how a ledger becomes prose.
    """
    argv = entry.get('lever_argv')
    if not argv:
        raise ValueError(
            f"iteration {entry.get('iteration')} recorded no lever_argv, so it "
            f"cannot be replayed -- only re-read")
    return list(argv)
