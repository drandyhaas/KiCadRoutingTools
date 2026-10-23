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

Two traps stage3d measured are handled here, not left to the viewer: KiCad
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
import re
import shutil
import struct
import subprocess
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, 'cache')
#: kicad-cli's GLB export, measured at 3-6 s on a 60-part board: a hang guard.
GLB_TIMEOUT_S = 300
_MODEL_DIR_VARS = ('KISYS3DMOD', 'KICAD6_3DMODEL_DIR', 'KICAD7_3DMODEL_DIR',
                   'KICAD8_3DMODEL_DIR', 'KICAD9_3DMODEL_DIR', 'KICAD10_3DMODEL_DIR')
_MAC_CLI = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
_MODEL_RE = re.compile(r'(\(model\s+")([^"]+)(")')
TWIN_EXTS = ('.step', '.STEP', '.stp', '.STP')


def resolve_cli(explicit=None):
    """`(path, why)`: --kicad-cli, else $KICAD_CLI, else the repo's own
    resolver (kicad_oracle.find_kicad_cli), else the macOS app bundle, else
    `kicad-cli` on PATH."""
    for cand in (explicit, os.environ.get('KICAD_CLI')):
        if cand:
            return (cand, '') if os.path.isfile(cand) else (None, f'{cand!r} is not a file')
    try:
        from kicad_oracle import find_kicad_cli
        try:
            cli = find_kicad_cli(warn=False)
        except TypeError:
            cli = find_kicad_cli()
        if cli:
            return cli, ''
    except Exception:                                          # noqa: BLE001
        pass
    if os.path.isfile(_MAC_CLI):
        return _MAC_CLI, ''
    cli = shutil.which('kicad-cli')
    return (cli, '') if cli else (None, 'no kicad-cli (set $KICAD_CLI or --kicad-cli)')


def _share_dirs(cli):
    bindir = os.path.dirname(os.path.abspath(cli))
    prefix = os.path.dirname(bindir)
    cands = [os.path.join(prefix, 'share', 'kicad', '3dmodels'),
             os.path.join(prefix, 'SharedSupport', '3dmodels'),
             os.path.join(prefix, '3dmodels')]
    return [c for c in cands if os.path.isdir(c)]


def model_dirs(cli, board_dir):
    """What each `${...}` in a model path stands for: an environment variable
    of that name wins (as in KiCad), else the install's own 3dmodels tree;
    KIPRJMOD is the board's directory."""
    dirs = {'KIPRJMOD': board_dir}
    tree = (_share_dirs(cli) or [None])[0]
    for var in _MODEL_DIR_VARS:
        env = os.environ.get(var)
        if env and os.path.isdir(env):
            dirs[var] = env
        elif tree:
            dirs[var] = tree
    if os.environ.get('KICAD_USER_DIR'):
        dirs['KICAD_USER_DIR'] = os.environ['KICAD_USER_DIR']
    return dirs


def _resolve(path, dirs):
    return re.sub(r'\$\{(\w+)\}|\$\((\w+)\)',
                  lambda m: dirs.get(m.group(1) or m.group(2), m.group(0)), path)


_INDEX = {}


def _library_index(dirs):
    roots = sorted({v for k, v in dirs.items() if k != 'KIPRJMOD' and v and os.path.isdir(v)})
    key = tuple(roots)
    if key not in _INDEX:
        idx = {}
        for root in roots:
            for d, _sub, files in os.walk(root):
                for fn in files:
                    stem, ext = os.path.splitext(fn)
                    if ext.lower() in ('.step', '.stp', '.wrl', '.vrml'):
                        idx.setdefault(stem.lower(), []).append(os.path.join(d, fn))
        for v in idx.values():
            v.sort()
        _INDEX[key] = idx
    return _INDEX[key]


def _by_name(path, dirs):
    stem = os.path.splitext(os.path.basename(path.replace('\\', '/')))[0]
    hits = _library_index(dirs).get(stem.lower()) or []
    for ext in ('.step', '.stp', '.wrl', '.vrml'):
        for h in hits:
            if h.lower().endswith(ext):
                return h
    return None


def _locate(path, dirs, board_dir):
    """The file model reference `path` names, made absolute; else its
    `.step` twin; else a library file of its name. Returns (file, how), or
    (None, None)."""
    real = _resolve(path, dirs)
    if board_dir and not os.path.isabs(real) and '${' not in real:
        real = os.path.normpath(os.path.join(board_dir, real))
    if os.path.isfile(real):
        return real, 'as is'
    if path.lower().endswith(('.wrl', '.vrml')):
        stem = os.path.splitext(real)[0]
        for ext in TWIN_EXTS:
            if os.path.isfile(stem + ext):
                return stem + ext, 'twin'
    found = _by_name(path, dirs)
    return (found, 'named') if found else (None, None)


def _hidden(fp_text, at):
    """True when the model block starting at `at` is hidden (`hide` or
    `(hide yes)`): kicad-cli draws none, so nothing is looked for."""
    depth, i = 0, at
    while i < len(fp_text):
        c = fp_text[i]
        depth += (c == '(') - (c == ')')
        i += 1
        if depth == 0:
            break
    return bool(re.search(r'\(model\s+"[^"]*"\s+hide\b|\(hide\s+yes\)', fp_text[at:i]))


def _assembled(fp_text):
    """Whether a footprint block with no model is a part the board carries:
    a copper pad, and not DNP, board-only, or left out of the position files
    or the BOM (a test pad, a solder jumper, a printed antenna, a logo) --
    record._not_assembled's rule, read off the text."""
    if not re.search(r'\(pad\s+"[^"]*"\s+(smd|thru_hole)\b', fp_text):
        return False
    if re.search(r'\(attr\b[^)]*\b(board_only|exclude_from_pos_files|exclude_from_bom|dnp)\b',
                 fp_text):
        return False
    return not re.search(r'\(dnp\s+yes\)', fp_text)


def stage_models(board_text, dirs, board_dir, found=None):
    """Every model reference made absolute, a missing `.wrl` pointed at an
    existing `.step` twin, and a reference still missing found by its file
    name in the library dirs -- else, with `found` (find_models' index),
    the model an AI agent found for it, one per footprint (an exact match
    before an equivalent: a footprint naming two models that are both
    missing must not get both). A part whose footprint names no model at all
    takes the one recorded for its footprint's name (`find_models.
    footprint_key`), placed at the footprint's origin. Returns (text, counts
    {'twin', 'named', 'found'}, missing: the shown references nothing
    resolves, and `find_models.NO_MODEL` + the footprint name of each part
    that names none, found_refs: the parts drawn from a found model)."""
    from kicad_parser import iter_footprint_blocks
    import find_models
    counts = {'twin': 0, 'named': 0, 'found': 0}
    missing, found_refs, out, last = [], [], [], 0
    for start, end, fp, raw, key in iter_footprint_blocks(board_text):
        subs, pending = [], []
        for m in _MODEL_RE.finditer(fp):
            if _hidden(fp, m.start()):
                continue
            real, how = _locate(m.group(2), dirs, board_dir)
            if real:
                if how in counts:
                    counts[how] += 1
                subs.append((m, real))
            else:
                pending.append(m)
        got = None
        if found is not None and pending and not subs:
            cands = [(m, find_models.found_file(find_models.model_key(m.group(2)), index=found))
                     for m in pending]
            cands = [(m, f) for m, f in cands if f]
            exact = [c for c in cands if (found.get(find_models.model_key(c[0].group(2))) or {})
                     .get('match') == 'exact']
            got = (exact or cands or [None])[0]
        if got:
            subs.append(got)
            counts['found'] += 1
            found_refs.append(key)
        elif not subs:
            missing += [m.group(2) for m in pending]
        # a part whose footprint names no model at all is missing one too
        add = None
        if not _MODEL_RE.search(fp) and _assembled(fp):
            name = re.match(r'\(footprint\s+"([^"]+)"', fp).group(1)
            f = (find_models.found_file(find_models.footprint_key(name), index=found)
                 if found is not None else None)
            if f:
                add = f
                counts['found'] += 1
                found_refs.append(key)
            else:
                missing.append(find_models.NO_MODEL + name)
        out.append(board_text[last:start])
        pos = 0
        for m, real in sorted(subs, key=lambda s: s[0].start()):
            out.append(fp[pos:m.start()] + m.group(1) + real.replace('\\', '/') + m.group(3))
            pos = m.end()
        tail = fp[pos:]
        if add:     # the found model, at the footprint's origin, in its own block
            tail = (tail[:-1].rstrip() + f'\n\t\t(model "{add}"\n\t\t\t(offset\n\t\t\t\t(xyz 0 0 0)\n'
                    '\t\t\t)\n\t\t\t(scale\n\t\t\t\t(xyz 1 1 1)\n\t\t\t)\n\t\t\t(rotate\n'
                    '\t\t\t\t(xyz 0 0 0)\n\t\t\t)\n\t\t)\n\t)')
        out.append(tail)
        last = end
    out.append(board_text[last:])
    return ''.join(out), counts, missing, found_refs


def _glb_node_names(glb_path):
    with open(glb_path, 'rb') as f:
        magic, _ver, _len = struct.unpack('<III', f.read(12))
        if magic != 0x46546C67:
            return []
        clen, _ctype = struct.unpack('<II', f.read(8))
        doc = json.loads(f.read(clen))
    return [n.get('name', '') for n in doc.get('nodes', [])]


def _cli_version(cli):
    try:
        return subprocess.run([cli, 'version'], capture_output=True, text=True,
                              timeout=60).stdout.strip()
    except Exception:                                          # noqa: BLE001
        return '?'


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
