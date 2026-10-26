#!/usr/bin/env python3
"""The 3D models a board's parts are missing: what to look for, and where
the ones found are kept.

`models.export` stages each footprint's model references against the
installed library: paths made absolute, a `.wrl` pointed at its `.step`
twin, a lookup by file name, and then the models recorded here. A part
whose model is still missing is drawn as a box.

Looking for them on the web is the `make-3d-movie` skill's job
(.claude/skills/make-3d-movie/SKILL.md): Claude, in the session, searches
and reads; this tool lists what to look for and records each answer, so the
movie itself never runs an agent.

    python3 movie3d/find_models.py missing BOARD [--retry]
        JSON: the board's hints (title, company, libraries), where KiCad's
        3D library is installed, and every model still missing, with the
        parts that use it, the installed files whose names are close to it
        (a rename) and the installed folders a comparable model is likely in.
        A model recorded as not found is left out unless --retry.
    python3 movie3d/find_models.py add KEY URL --match exact|equivalent [--source S] [--note N]
        download URL and keep it as KEY's model -- https only, at most
        MAX_BYTES, a zip read in memory and only its model members taken (no
        path from the archive is used), and kept only when it starts like a
        STEP (`ISO-10303-21`) or VRML (`#VRML`) model; a GitHub or GitLab
        page link is turned into its raw-file link. Exit 1, with the reason,
        when the file is refused.
    python3 movie3d/find_models.py library KEY PATH --match exact|equivalent [--note N]
        KEY's model is an installed library file (KiCad renames models:
        `SW_CuK_JS202011CQN...` became `SW_CK_JS202011CQN...`).
    python3 movie3d/find_models.py none KEY --note WHY
        nothing suitable was found: `missing` leaves it out from now on.
    python3 movie3d/find_models.py show
        the recorded answers.

KEY is the model's file name without its extension, in lower case (the
`key` `missing` prints), so two boards naming one file share the answer. A
part whose footprint names no model at all is listed too, keyed `fp-` and
its footprint's name; the model recorded for it is placed at the
footprint's origin.
The answers and the files live in movie3d/cache/found/ (gitignored). The
page checks that a found model sits on its footprint and keeps the box when
it does not (page/movie3d.mjs `loadModels`); make_movie names those parts.
"""
import argparse
import datetime
import difflib
import io
import json
import os
import re
import ssl
import sys
import urllib.parse
import urllib.request
import zipfile

#: #937 registry: a prototype in its own directory, in no door; it reads a
#: board and writes only movie3d/cache/found/, never a board. Read by
#: krt_registry.py by AST.
KRT_TOOL = {'scope': [], 'kind': 'utility'}

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
FOUND = os.path.join(HERE, 'cache', 'found')
MAX_BYTES = 40 * 1024 * 1024
DOWNLOAD_TIMEOUT_S = 90
MODEL_EXTS = ('.step', '.stp', '.wrl', '.vrml')
_MODEL_RE = re.compile(r'\(model\s+"([^"]+)"')


def model_key(path):
    """A model's key: its file name without extension, lower case."""
    return os.path.splitext(os.path.basename(path.replace('\\', '/')))[0].lower()


#: how model_export.stage_models names a part whose footprint names no model at
#: all: this prefix and the footprint's library name
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from model_export import NO_MODEL  # noqa: E402


def footprint_key(name):
    """The key of the model for a footprint that names none: `fp-` and its
    name without the library nickname, lower case."""
    return 'fp-' + name.split(':')[-1].lower()


def _key_of(entry):
    """The key of one `missing` entry: a model path, or NO_MODEL + a
    footprint name."""
    return footprint_key(entry[len(NO_MODEL):]) if entry.startswith(NO_MODEL) else model_key(entry)


def _index_path(cache):
    return os.path.join(cache, 'index.json')


def load_index(cache=FOUND):
    try:
        with open(_index_path(cache), encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _record(key, entry, cache=FOUND):
    index = load_index(cache)
    index[key] = dict(entry, recorded=datetime.date.today().isoformat())
    os.makedirs(cache, exist_ok=True)
    tmp = _index_path(cache) + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(index, f, indent=1, sort_keys=True)
    os.replace(tmp, _index_path(cache))


def found_file(key, cache=FOUND, index=None):
    """The local file recorded for the model `key` names, or None."""
    e = (index if index is not None else load_index(cache)).get(key)
    if not e or e.get('status') != 'found':
        return None
    f = e['file'] if os.path.isabs(e['file']) else os.path.join(cache, e['file'])
    return f if os.path.isfile(f) else None


# ------------------------------------------------------------------ missing
def _prop(fp_text, name):
    m = re.search(r'\(property\s+"%s"\s+"((?:[^"\\]|\\.)*)"' % name, fp_text)
    return m.group(1) if m else ''


def describe_missing(board_text, missing):
    """{key: one missing model}: the references it was named by, and the
    parts that use it (reference, value, footprint, description, pad count,
    face)."""
    from kicad_parser import iter_footprint_blocks
    want = {}
    for p in missing:
        k = _key_of(p)
        w = want.setdefault(k, {'key': k, 'referenced_as': [], 'parts': []})
        shown = '(its footprint names no model)' if p.startswith(NO_MODEL) else p
        if shown not in w['referenced_as']:
            w['referenced_as'].append(shown)
    for _s, _e, text, raw, key in iter_footprint_blocks(board_text):
        fp = re.match(r'\(footprint\s+"([^"]+)"', text)
        keys = {model_key(p) for p in _MODEL_RE.findall(text)}
        if fp and not keys:
            keys = {footprint_key(fp.group(1))}
        keys &= set(want)
        if not keys:
            continue
        layer = re.search(r'\(layer\s+"([^"]+)"', text)
        descr = _prop(text, 'Description')
        if not descr:
            m = re.search(r'\(descr\s+"((?:[^"\\]|\\.)*)"', text)
            descr = m.group(1) if m else ''
        part = {'ref': raw or key, 'value': _prop(text, 'Value'), 'footprint': fp.group(1) if fp else '',
                'description': descr, 'pads': len(re.findall(r'\(pad\s+"', text)),
                'face': 'back' if layer and layer.group(1).startswith('B.') else 'front'}
        for k in keys:
            if len(want[k]['parts']) < 4:
                want[k]['parts'].append(part)
    return want


def board_hints(board_text, board_path):
    """What identifies the board's project: its file name, title block and
    footprint library nicknames."""
    hints = {'file': os.path.basename(board_path)}
    tb = re.search(r'\(title_block(.*?)\n\s*\)', board_text, re.S)
    if tb:
        for k in ('title', 'company', 'rev'):
            m = re.search(r'\(%s\s+"([^"]*)"' % k, tb.group(1))
            if m and m.group(1):
                hints[k] = m.group(1)
        comments = [m.group(1) for m in re.finditer(r'\(comment\s+\d+\s+"([^"]+)"', tb.group(1))]
        if comments:
            hints['comments'] = comments
    hints['footprint_libraries'] = sorted({m.group(1) for m in re.finditer(
        r'\(footprint\s+"([^":]+):', board_text)})[:12]
    return hints


def near_names(key, library_index, n=5):
    """Installed library models whose names are close to `key` (a rename)."""
    hits = difflib.get_close_matches(key, list(library_index), n=n, cutoff=0.8)
    return [library_index[h][0] for h in hits]


#: words a library folder name and a footprint name share that say nothing
#: about what the part is
_GENERIC = {'smd', 'tht', 'mm', 'vertical', 'horizontal', 'metric', 'handsolder', 'pad', 'roundrect',
            'value', 'and', 'the', 'with', 'for'}
#: a footprint's spelling -> the library folders' spelling
_ALIASES = {'sw': 'switch', 'pinheader': 'pinheader', 'pinsocket': 'pinsocket', 'led': 'led',
            'xtal': 'crystal', 'osc': 'oscillator', 'jst': 'jst', 'ic': 'package',
            'vqfn': 'qfn', 'wqfn': 'qfn', 'uqfn': 'qfn', 'tqfn': 'qfn', 'vson': 'son', 'wson': 'son',
            'tssop': 'so', 'msop': 'so', 'soic': 'so', 'ssop': 'so'}


def _words(s):
    return {_ALIASES.get(w, w) for w in re.split(r'[^a-z0-9]+', (s or '').lower())
            if len(w) >= 2 and w not in _GENERIC and not w.isdigit()}


def library_folders(item, root, n=6):
    """The installed library's `.3dshapes` folders a comparable model of a
    missing one is likely in: the folder its own path names, then the ones
    sharing words with its parts' footprint names, values and descriptions
    (USB -> Connector_USB, SW -> Button_Switch_*, PinHeader ->
    Connector_PinHeader_*)."""
    if not root or not os.path.isdir(root):
        return []
    folders = sorted(d for d in os.listdir(root) if d.endswith('.3dshapes'))
    out = [m.group(1) + '.3dshapes' for p in item['referenced_as']
           for m in [re.search(r'([^/\\]+)\.3dshapes[/\\]', p)] if m]
    out = [f for f in out if f in folders]
    words = set()
    for p in item['parts']:
        words |= _words(p['footprint'].split(':')[-1]) | _words(p['value']) | _words(p['description'])
    scored = sorted(((len(_words(f[:-len('.3dshapes')]) & words), f) for f in folders),
                    key=lambda s: (-s[0], s[1]))
    out += [f for s, f in scored if s > 0 and f not in out]
    return out[:n]


def missing_report(board, kicad_cli=None, retry=False):
    """The `missing` subcommand's document."""
    import models
    cli, why = models.resolve_cli(kicad_cli)
    if not cli:
        sys.exit(f'find_models: {why}')
    with open(board, encoding='utf-8') as f:
        text = f.read()
    board_dir = os.path.dirname(os.path.abspath(board))
    dirs = models.model_dirs(cli, board_dir)
    index = load_index()
    _staged, _counts, missing, _found = models.stage_models(text, dirs, board_dir, found=index)
    lib = models._library_index(dirs)
    root = (models._share_dirs(cli) or [None])[0]
    want = describe_missing(text, missing)
    out, skipped = [], []
    for k, w in sorted(want.items()):
        e = index.get(k)
        if e and e.get('status') == 'none' and not retry:
            skipped.append({'key': k, 'note': e.get('note', '')})
            continue
        near = near_names(k[3:] if k.startswith('fp-') else k, lib)
        out.append(dict(w, library_near=near, library_folders=library_folders(w, root),
                        **({'previously': e.get('note', '')} if e else {})))
    return {'board': board_hints(text, board), 'library_root': root, 'missing': out,
            'not_found_before': skipped}


# ------------------------------------------------------------------ add
def _ssl_context():
    """A context that verifies against a real CA bundle: python.org's macOS
    Python ships none (CERTIFICATE_VERIFY_FAILED on every https URL)."""
    try:
        import certifi
        return ssl.create_default_context(cafile=certifi.where())
    except ImportError:
        pass
    if os.path.isfile('/etc/ssl/cert.pem'):
        return ssl.create_default_context(cafile='/etc/ssl/cert.pem')
    return ssl.create_default_context()


def _model_kind(blob):
    """'.step' or '.wrl' when the bytes start like a STEP or VRML model."""
    head = blob[:64].lstrip(b'\xef\xbb\xbf \t\r\n')
    if head.startswith(b'ISO-10303-21'):
        return '.step'
    if head.startswith(b'#VRML'):
        return '.wrl'
    return None


def _raw_url(url):
    """A repository's page link turned into its raw-file link."""
    m = re.match(r'https://github\.com/([^/]+)/([^/]+)/blob/(.+)$', url)
    if m:
        return f'https://raw.githubusercontent.com/{m.group(1)}/{m.group(2)}/{m.group(3)}'
    return url.replace('/-/blob/', '/-/raw/')


def fetch_model(url, key):
    """(bytes, extension) of the model `url` serves, or (None, why)."""
    url = _raw_url(str(url).strip())
    if urllib.parse.urlparse(url).scheme != 'https':
        return None, 'not an https URL'
    req = urllib.request.Request(url, headers={'User-Agent': 'KiCadRoutingTools-movie3d'})
    try:
        with urllib.request.urlopen(req, timeout=DOWNLOAD_TIMEOUT_S, context=_ssl_context()) as r:
            blob = r.read(MAX_BYTES + 1)
    except Exception as e:                                      # noqa: BLE001
        return None, f'download failed ({str(e)[:120]})'
    if len(blob) > MAX_BYTES:
        return None, f'over {MAX_BYTES // (1024 * 1024)} MB'
    if blob[:4] == b'PK\x03\x04':
        try:
            zf = zipfile.ZipFile(io.BytesIO(blob))
        except zipfile.BadZipFile:
            return None, 'a broken zip'
        members = [i for i in zf.infolist() if i.filename.lower().endswith(MODEL_EXTS)
                   and 0 < i.file_size <= MAX_BYTES]
        members.sort(key=lambda i: (model_key(i.filename) != key,
                                    not i.filename.lower().endswith(('.step', '.stp'))))
        for i in members:
            inner = zf.read(i)
            kind = _model_kind(inner)
            if kind:
                return inner, kind
        return None, 'no STEP or VRML model in the zip'
    kind = _model_kind(blob)
    if kind:
        return blob, kind
    return None, ('an HTML page, not the file: give its raw-file URL'
                  if blob.lstrip()[:15].lower().startswith((b'<!doctype', b'<html')) else
                  'not a STEP or VRML file')


def _key(k):
    k = k.strip().lower()
    if not k or '/' in k or '\\' in k or k.startswith('.'):
        sys.exit(f'find_models: {k!r} is not a model key (the `key` that `missing` prints)')
    return k


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0], allow_abbrev=False)
    sub = ap.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('missing', help='JSON: the models the board is missing, to look for')
    m.add_argument('board')
    m.add_argument('--retry', action='store_true', help='include the models recorded as not found')
    m.add_argument('--kicad-cli')
    a = sub.add_parser('add', help="download URL as KEY's model, checked")
    a.add_argument('key')
    a.add_argument('url')
    a.add_argument('--match', choices=('exact', 'equivalent'), required=True)
    a.add_argument('--source', default='')
    a.add_argument('--note', default='')
    lb = sub.add_parser('library', help="KEY's model is an installed library file (a rename)")
    lb.add_argument('key')
    lb.add_argument('path')
    lb.add_argument('--match', choices=('exact', 'equivalent'), required=True)
    lb.add_argument('--note', default='')
    n = sub.add_parser('none', help='record that nothing suitable was found for KEY')
    n.add_argument('key')
    n.add_argument('--note', required=True)
    sub.add_parser('show', help='the recorded answers')
    args = ap.parse_args(argv)
    sys.path[:0] = [HERE, os.path.join(REPO, 'py_router')]

    if args.cmd == 'missing':
        print(json.dumps(missing_report(args.board, args.kicad_cli, args.retry), indent=1))
        return 0
    if args.cmd == 'show':
        print(json.dumps(load_index(), indent=1, sort_keys=True))
        return 0
    key = _key(args.key)
    if args.cmd == 'none':
        _record(key, {'status': 'none', 'note': args.note})
        print(f'find_models: {key}: recorded as not found')
        return 0
    if args.cmd == 'library':
        path = os.path.abspath(args.path)
        if not (os.path.isfile(path) and path.lower().endswith(MODEL_EXTS)):
            print(f'find_models: refused: {args.path} is not a .step/.stp/.wrl file', file=sys.stderr)
            return 1
        _record(key, {'status': 'found', 'file': path, 'url': '', 'source': 'installed library',
                      'match': args.match, 'note': args.note})
        print(f'find_models: {key}: the installed {os.path.basename(path)} ({args.match})')
        return 0
    blob, kind = fetch_model(args.url, key)
    if blob is None:
        print(f'find_models: refused {args.url}: {kind}', file=sys.stderr)
        return 1
    os.makedirs(os.path.join(FOUND, key), exist_ok=True)
    name = os.path.join(key, key + kind)
    with open(os.path.join(FOUND, name), 'wb') as f:
        f.write(blob)
    _record(key, {'status': 'found', 'file': name, 'url': args.url, 'source': args.source,
                  'match': args.match, 'note': args.note})
    print(f'find_models: {key}: kept {len(blob) // 1024} kB of {kind[1:].upper()} ({args.match})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
