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
`matched` says how many parts got a model. A found model kicad-cli cannot
read would fail the whole export, so it is found and left out
(`unreadable`): only its parts are boxes.

The export is cached in movie3d/cache/<key>/ (gitignored), keyed by the
staged board text and kicad-cli's version, so a re-render does not export
again.
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'py_router'))
from model_export import (GLB_TIMEOUT_S, _cli_version, _glb_node_names,  # noqa: E402,F401
                          _library_index, _share_dirs, model_dirs, resolve_cli,
                          stage_models)
CACHE = os.path.join(HERE, 'cache')


def _export_glb(cli, text, glb):
    """kicad-cli exports the parts of board `text` to `glb`: '' when it
    did, else why not."""
    tmp = tempfile.mkdtemp(prefix='movie3d_glb_')
    try:
        staged = os.path.join(tmp, 'board.kicad_pcb')
        with open(staged, 'w', encoding='utf-8') as f:
            f.write(text)
        argv = [cli, 'pcb', 'export', 'glb', '--no-dnp', '--subst-models',
                '--no-board-body', '-f', '-o', glb, staged]
        try:
            r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                               errors='replace', timeout=GLB_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return f'kicad-cli pcb export glb took over {GLB_TIMEOUT_S} s'
        if r.returncode != 0 or not os.path.isfile(glb):
            blob = (r.stderr or r.stdout or '').strip()
            return (f'kicad-cli pcb export glb exited {r.returncode}'
                    + (': ' + blob.splitlines()[0][:160] if blob else ''))
        return ''
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _without_unreadable(cli, glb, board_text, dirs, board_dir, index, text):
    """After the export of `text` (staged with find_models' `index`)
    failed: the found models kicad-cli cannot read, and the board exported
    to `glb` with every other one. The board is exported with no found
    model first; then a share of them at a time, a share that fails halved
    again, one export per try. Returns `(unreadable: [{'model': key,
    'refs'}], why)`, why '' when `glb` was written; when the board does not
    export even with no found model, `([], why)`."""
    import find_models
    from kicad_parser import iter_footprint_blocks
    keys_of = {}
    for k in index:
        f = find_models.found_file(k, index=index)
        if f:
            keys_of.setdefault(f.replace('\\', '/'), []).append(k)
    refs_of = {}
    for _s, _e, fp, _raw, ref in iter_footprint_blocks(text):
        for p in re.findall(r'\(model\s+"([^"]+)"', fp):
            p = p.replace('\\', '/')
            if p in keys_of:
                refs_of.setdefault(p, []).append(ref)
    files = sorted(refs_of)

    def staged(keep):
        drop = {k for f in files if f not in keep for k in keys_of[f]}
        return stage_models(board_text, dirs, board_dir,
                            found={k: e for k, e in index.items() if k not in drop})[0]

    why = _export_glb(cli, staged(()), glb)
    if why:
        return [], why
    trial = glb + '.try'

    def bad_of(group):
        """The files of `group` kicad-cli cannot read; `group` fails with
        every other found model left out."""
        if len(group) == 1:
            return list(group)
        half = len(group) // 2
        return [f for part in (group[:half], group[half:])
                if _export_glb(cli, staged(part), trial) for f in bad_of(part)]

    bad = bad_of(files)
    good = [f for f in files if f not in bad]
    # found models that fail only together leave the good ones failing too:
    # all are then left out, as in the export without them that worked
    if good and not _export_glb(cli, staged(good), trial):
        os.replace(trial, glb)
    else:
        bad = files
    if os.path.exists(trial):
        os.remove(trial)
    return [{'model': '/'.join(sorted(keys_of[f])), 'refs': sorted(refs_of[f])}
            for f in sorted(bad, key=lambda f: sorted(keys_of[f]))], ''


def export(board_text, board_dir, refs, cli=None, cache=CACHE):
    """`(info, why)`: the parts' models as one GLB, or (None, why).

    `board_text` is the board the poses were read from; `board_dir` the
    ORIGINAL board's directory (KIPRJMOD and relative paths resolve there);
    `refs` the references the movie draws. A model nothing installed
    resolves is drawn from the one recorded in find_models' cache, when the
    make-3d-movie skill found one. One file kicad-cli cannot read fails the
    whole export, so when it fails with found models staged, the ones it
    cannot read are found (`_without_unreadable`) and only their parts are
    boxes. `info` = {'path', 'matched': [refs with a model], 'found': [refs
    drawn from a found model], 'missing': [model references still missing],
    'unreadable': [{'model': a found model's key, 'refs': its parts}
    kicad-cli cannot read], 'cached': bool, 'note'}."""
    import find_models
    cli, why = resolve_cli(cli)
    if not cli:
        return None, why
    dirs = model_dirs(cli, board_dir)
    index = find_models.load_index()
    text, counts, missing, found_refs = stage_models(board_text, dirs, board_dir, found=index)
    # the staged text names a found model by its path only, so a model `add`
    # replaced would otherwise be drawn from the export of the one before it
    stamps = ''.join(f'\0{p}:{os.path.getsize(p)}:{os.path.getmtime(p)}'
                     for p in sorted(set(re.findall(r'\(model\s+"([^"]+)"', text)))
                     if p.startswith(find_models.FOUND) and os.path.isfile(p))
    key = hashlib.sha1((text + '\0' + _cli_version(cli) + stamps).encode('utf-8')).hexdigest()[:16]
    out_dir = os.path.join(cache, key)
    glb = os.path.join(out_dir, 'parts.glb')
    meta = os.path.join(out_dir, 'export.json')
    note = ', '.join(s for s in (f"{counts['embedded']} embedded in the board"
                                 if counts['embedded'] else '',
                                 f"{counts['twin']} .wrl -> .step" if counts['twin'] else '',
                                 f"{counts['named']} found by file name" if counts['named'] else '',
                                 f"{counts['found']} found on the web" if counts['found'] else '')
                     if s)
    if os.path.isfile(glb) and os.path.isfile(meta):
        with open(meta, encoding='utf-8') as f:
            got = json.load(f)
        names, unreadable = set(got['names']), got.get('unreadable', [])
        cached = True
    else:
        os.makedirs(out_dir, exist_ok=True)
        unreadable = []
        failed = _export_glb(cli, text, glb)
        if failed and found_refs:
            unreadable, still = _without_unreadable(cli, glb, board_text, dirs, board_dir,
                                                    index, text)
            failed = failed if still else ''
        if failed:
            return None, failed
        names = set(_glb_node_names(glb))
        with open(meta, 'w', encoding='utf-8') as f:
            json.dump({'names': sorted(names), 'cli': cli, 'unreadable': unreadable}, f)
        cached = False
    # a duplicate reference's later blocks are keyed `REF~2`... by the parser,
    # while kicad-cli names every node by the bare reference
    matched = sorted(r for r in refs if r.split('~')[0] in names)
    boxed = {r for u in unreadable for r in u['refs']}
    found = sorted(r for r in found_refs if r in matched and r not in boxed)
    return {'path': glb, 'matched': matched, 'found': found, 'missing': sorted(set(missing)),
            'unreadable': unreadable, 'cached': cached, 'note': note}, ''
