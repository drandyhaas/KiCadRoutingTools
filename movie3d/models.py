#!/usr/bin/env python3
"""The parts' real 3D models for the 3D movie: one GLB from kicad-cli, cached.

Ported from the removed py_router/stage3d/scene.py (`export_glb`,
`stage_models`) and py_router/kicad_iso_render.py (`resolve_cli`,
`model_dirs`), both still on `main` (#1081, #887).

kicad-cli exports the board's parts (no board body) as one GLB, each part a
node named by its bare reference, at the pose the exported board gives it.
The page re-poses a node by `F(now) * F(export)^-1`, which needs no knowledge
of the model's own offset (stage3d's measurement: an F.Cu part node is
`Ry(rot)`, a B.Cu one `Ry(rot) Rx(180)`).

Two traps stage3d measured are handled in py_router/model_export.py (shared
with the placer's part heights), not left to the viewer: KiCad
10 ships only `.step` models while old boards reference `.wrl`, and an
unresolved model is dropped SILENTLY -- so the board is staged with each
reference made absolute, a missing `.wrl` pointed at its `.step` twin, and a
still-missing one looked up by file name in the library, and failing that
the model the make-3d-movie skill found on the web for it (find_models.py);
`matched` says how many parts got a model.

The export is cached in movie3d/cache/<key>/ (gitignored), keyed by the
staged board text and kicad-cli's version, so a re-render does not export
again.
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'py_router'))
from model_export import (GLB_TIMEOUT_S, _cli_version, _glb_node_names,  # noqa: E402,F401
                          model_dirs, resolve_cli, stage_models)
CACHE = os.path.join(HERE, 'cache')
def export(board_text, board_dir, refs, cli=None, cache=CACHE):
    """`(info, why)`: the parts' models as one GLB, or (None, why).

    `board_text` is the board the poses were read from; `board_dir` the
    ORIGINAL board's directory (KIPRJMOD and relative paths resolve there);
    `refs` the references the movie draws. A model nothing installed
    resolves is drawn from the one recorded in find_models' cache, when the
    make-3d-movie skill found one. `info` = {'path', 'matched': [refs with a
    model], 'found': [refs drawn from a found model], 'missing': [model
    references still missing], 'cached': bool, 'note'}."""
    import find_models
    cli, why = resolve_cli(cli)
    if not cli:
        return None, why
    dirs = model_dirs(cli, board_dir)
    text, counts, missing, found_refs = stage_models(board_text, dirs, board_dir,
                                                     found=find_models.load_index())
    key = hashlib.sha1((text + '\0' + _cli_version(cli)).encode('utf-8')).hexdigest()[:16]
    out_dir = os.path.join(cache, key)
    glb = os.path.join(out_dir, 'parts.glb')
    meta = os.path.join(out_dir, 'export.json')
    note = ', '.join(s for s in (f"{counts['twin']} .wrl -> .step" if counts['twin'] else '',
                                 f"{counts['named']} found by file name" if counts['named'] else '',
                                 f"{counts['found']} found on the web" if counts['found'] else '')
                     if s)
    if os.path.isfile(glb) and os.path.isfile(meta):
        names = set(json.load(open(meta, encoding='utf-8'))['names'])
        cached = True
    else:
        tmp = tempfile.mkdtemp(prefix='movie3d_glb_')
        try:
            staged = os.path.join(tmp, 'board.kicad_pcb')
            with open(staged, 'w', encoding='utf-8') as f:
                f.write(text)
            os.makedirs(out_dir, exist_ok=True)
            argv = [cli, 'pcb', 'export', 'glb', '--no-dnp', '--subst-models',
                    '--no-board-body', '-f', '-o', glb, staged]
            try:
                r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                                   errors='replace', timeout=GLB_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                return None, f'kicad-cli pcb export glb took over {GLB_TIMEOUT_S} s'
            if r.returncode != 0 or not os.path.isfile(glb):
                blob = (r.stderr or r.stdout or '').strip()
                return None, (f'kicad-cli pcb export glb exited {r.returncode}'
                              + (': ' + blob.splitlines()[0][:160] if blob else ''))
            names = set(_glb_node_names(glb))
            with open(meta, 'w', encoding='utf-8') as f:
                json.dump({'names': sorted(names), 'cli': cli}, f)
            cached = False
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    # a duplicate reference's later blocks are keyed `REF~2`... by the parser,
    # while kicad-cli names every node by the bare reference
    matched = sorted(r for r in refs if r.split('~')[0] in names)
    found = sorted(r for r in found_refs if r in matched)
    return {'path': glb, 'matched': matched, 'found': found, 'missing': sorted(set(missing)),
            'cached': cached, 'note': note}, ''
