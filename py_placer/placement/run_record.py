"""The placement record (#1486): what a placement run did, for a replay with
no AI.

The routing tools record their own invocations into the run's
`redo_commands.sh` (`redo_record`), which `redo_stress_test.py` replays. A
placement run is not only commands: the agent WRITES the groups file, edits
the emitted intent, sketches templates and a judge chain, and a replay needs
each as the step that read it saw it. So each placement tool records itself
here, into the JSON file `PLACEMENT_MANIFEST` names (`placement_manifest.json`
in the run directory), with the files it reads BY CONTENT:

* every argument that is an existing `.json` or `.txt` file (groups, intent,
  sketch, judge chain, a brief passed by path),
* the design brief and `mechanical.json` beside every board it is given,
  which `check_floorplan` and `board_brief` read without being told, and
* the board a tool reads (`boards=`), with the `.kicad_pro` and
  `.kicad_dru` beside it: an agent makes that board with a plain copy of the
  pile, or edits it by hand, and no recorded step replays either. An
  unchanged board costs one copy, since content is stored by digest.

A board a step writes is not snapshotted, and the run's first input is the
corpus board, named by its path. Content is stored once
per digest (`files`), and each step names the digests it read (`inputs`), so
a file the agent edits between two steps is two entries.

Recording is best-effort and silent: no failure here may affect the tool.
`PLACEMENT_MANIFEST` unset (or the null device) records nothing.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
import time
from typing import Dict, List, Optional, Sequence

ENV = 'PLACEMENT_MANIFEST'
SCHEMA = 1
KIND = 'placement-manifest'


def _inputs(argv: Sequence[str], boards: Sequence[str] = ()) -> List[str]:
    """The files a step reads that a replay must restore: `.json` and `.txt`
    arguments, the brief and mechanical file beside each board argument, and
    each of `boards` with its project and rules."""
    out: List[str] = []
    for tok in argv:
        t = tok.split('=', 1)[1] if tok.startswith('--') and '=' in tok else tok
        if t.endswith(('.json', '.txt')) and os.path.isfile(t):
            out.append(t)
        elif t.endswith('.kicad_pcb'):
            stem, d = t[:-len('.kicad_pcb')], os.path.dirname(t) or '.'
            for sib in (stem + '.design-brief.json', os.path.join(d, 'mechanical.json')):
                if os.path.isfile(sib):
                    out.append(sib)
    for b in boards:
        stem = b[:-len('.kicad_pcb')] if b.endswith('.kicad_pcb') else b
        out += [f for f in (b, stem + '.kicad_pro', stem + '.kicad_dru') if os.path.isfile(f)]
    return list(dict.fromkeys(out))


def record(tool: str, argv: Optional[Sequence[str]] = None,
           manifest: Optional[str] = None, boards: Sequence[str] = ()) -> None:
    """Append this invocation of `tool` (its script argv, without the
    interpreter) to the placement record, with the files it reads by
    content; `boards` are the board files it places, stored too. A no-op
    unless `PLACEMENT_MANIFEST` (or `manifest`) names a file."""
    path = manifest or os.environ.get(ENV)
    if not path or path in (os.devnull, '/dev/null'):
        return
    try:
        argv = list(sys.argv[1:] if argv is None else argv)
        _append(os.path.abspath(path), tool, argv, boards)
    except Exception:                                        # noqa: BLE001
        pass


def _append(path: str, tool: str, argv: List[str], boards: Sequence[str] = ()) -> None:
    lock = _Lock(path + '.lock')
    with lock:
        doc = {'schema': SCHEMA, 'kind': KIND, 'steps': [], 'files': {}}
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as fh:
                doc = json.load(fh)
        inputs: Dict[str, str] = {}
        for f in _inputs(argv, boards):
            with open(f, encoding='utf-8') as fh:
                text = fh.read()
            digest = hashlib.sha256(text.encode('utf-8')).hexdigest()
            doc['files'].setdefault(digest, text)
            inputs[f] = digest
        doc['steps'].append({'tool': tool, 'argv': argv, 'cwd': os.getcwd(),
                             'inputs': inputs, 'time': round(time.time(), 3)})
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1)
        os.replace(tmp, path)


class _Lock:
    """An exclusive lock on a sidecar file, so two tools recording at once
    (a placement and a judge in parallel) do not lose a step."""

    def __init__(self, path: str):
        self.path, self.fh = path, None

    def __enter__(self):
        self.fh = open(self.path, 'a+')
        try:
            import fcntl
            fcntl.flock(self.fh, fcntl.LOCK_EX)
        except ImportError:                                  # Windows
            import msvcrt
            msvcrt.locking(self.fh.fileno(), msvcrt.LK_LOCK, 1)
        return self

    def __exit__(self, *exc):
        try:
            self.fh.close()
        finally:
            self.fh = None


def load(path: str) -> Dict:
    """A placement record, checked to be one."""
    with open(path, encoding='utf-8') as fh:
        doc = json.load(fh)
    if doc.get('kind') != KIND or doc.get('schema') != SCHEMA:
        raise ValueError(f"{path}: not a schema-{SCHEMA} {KIND}")
    return doc


def restore_inputs(doc: Dict, step: Dict, remap=lambda p: p) -> List[str]:
    """Write the files `step` read, as it read them, at their (remapped)
    paths -- what a replay does before it runs the step. Returns the paths."""
    out = []
    for path, digest in step.get('inputs', {}).items():
        dst = remap(path)
        os.makedirs(os.path.dirname(dst) or '.', exist_ok=True)
        with open(dst, 'w', encoding='utf-8') as fh:
            fh.write(doc['files'][digest])
        out.append(dst)
    return out
