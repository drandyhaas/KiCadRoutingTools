"""A board's KiCad schematic, read for placement grouping.

The board file says which sheet each footprint came from (its `(path ...)`
uuids, and on KiCad 8+ a `Sheetname`), and nothing about how the schematic is
DRAWN. The drawing is the designer's own grouping: a regulator drawn with its
caps, a crystal with its load caps, an LED with its resistor, the USB
connector with its CC resistors and ESD. This module reads it, for a model
deciding the groups of a placement. It never writes a schematic.

WHAT THE HUMAN BOARDS SAY
-------------------------
Measured on 239 corpus boards (35-220 parts, keyboards left out) whose
schematics were fetched from their source repositories, against where the
designers put the parts:

* How close two parts are DRAWN predicts whether the designer put them in
  the board's closest 10% of pairs with a median AUC of 0.76, against 0.60
  for how strongly the netlist connects them and 0.66 for sheet plus netlist;
  better on 88% of boards. Inside one sheet it still predicts at a fixed
  netlist distance (0.79 for parts sharing a net, 0.73 for parts sharing
  none), so it carries something the netlist does not.
* The drawn blocks this module cuts (`BLOCK_CUT_MM`, a median 3.6 parts
  each) sit 0.20 as wide on the human board as random sets of their sizes,
  against 0.45 for as many netlist clusters; tighter on 91% of boards.
* Sheets are the coarse groups: on hierarchical boards, grouping into 4-8
  groups by sheet then netlist is as tight as grouping by the drawing.

That is agreement with human layouts, not a routed result, and it is why the
output is evidence for the model's groups and not a grouping of its own.

WHAT IT READS
-------------
KiCad 6+ `.kicad_sch` (s-expressions): the sheet tree from the root, each
sheet instance's uuid path and name, every placed symbol (uuid, reference,
lib id, unit, its drawn box from its library pins and graphics), and the
free text and text boxes. A KiCad 5 `.sch` is reported, not read.

A footprint is matched to its symbol by the `(sheet prefix, symbol uuid)` of
its path (`groups.sheet_and_symbol`, the pair `replicate` maps by), so a
part matches on any board the schematic was synced to, whatever its
reference. The other units of a multi-unit part are the symbols of the same
file with its lib id and its reference.

Sheet files resolve against the directory of the sheet that names them,
which is how eeschema loads a hierarchy.

NOTES
-----
A short note (`NOTE_MAX_CHARS`) belongs to the nearest block within
`NOTE_REACH_MM`: "RTC Clock Crystal", "ESP32 3V3 LDO", "Buck Converter (12v
-> 5v)". 87% of the boards carry notes, and 28% of their blocks of three or
more parts have one beside them.

A note that asks for a placement ("C11 close to pin45", "Place R4/R5 close
to RP2040") is a LAYOUT NOTE, the designer's own proximity claim; 13% of
the boards carry one. Each is split into clauses, one per `close to` /
`next to` / ..., and a clause whose subject and partner the note NAMES (a
reference, a value one part carries, or a pin list with the part it is
`of`, else the part drawn nearest that has those pads) gives a candidate
`proximity[]` row with `max_mm` null -- the limit is the author's. A subject
the note leaves unnamed ("Place close to U6") is not guessed: the parts drawn
nearest the note are printed instead.

Every row is checked against the netlist (`_stale`), because a note is text
and survives a re-annotation that renumbers the parts it names: on
usb_fm_radio two of four pin notes name a cap on another rail. A row whose
subject is not on the named pin's net (or whose two parts share no net) is
returned as STALE, with the two-pad parts that ARE on that net, and never
as a row.

Measured, 8 corpus boards placed from a pile with the same groups, 12 seeds
each and every seed fast-routed: with the notes' rows, a noted pair met its
limit in 164 of 180 seeds, against 58 of 180 without them, at no measurable
routing cost.
"""
from __future__ import annotations

import math
import os
import re
from typing import Dict, List, Optional, Tuple

import numpy as np

#: A block is cut where the AVERAGE drawn gap between two sets of parts
#: passes this (mm, between symbol boxes). 25.4 gave blocks of a median 3.6
#: parts and the tightest blocks on the human boards for their size; single
#: linkage at any cut chained unrelated blocks along a crowded sheet.
BLOCK_CUT_MM = 25.4

#: A note belongs to the nearest block whose drawn box is within this of the
#: note's estimated text box.
NOTE_REACH_MM = 12.7

#: Longer text is a comment, not a block's title (it is still read for a
#: layout instruction).
NOTE_MAX_CHARS = 80

#: A note asking for a placement. "Do not place" is a DNP note, never this.
_LAYOUT_NOTE = re.compile(
    r"\b(close\s+to|as\s+close\s+as|closest\s+to|next\s+to|adjacent\s+to|"
    r"near(?:by)?\s+(?:to\b|the\b|pins?\b|[A-Z]{1,3}\d)|"
    r"keep\b[^.\n]{0,25}\b(?:short|close)|"
    r"short(?:est)?\s+(?:trace|track|path|loop|connection)s?)", re.I)
_PLACE_VERB = re.compile(r"\bplace[ds]?\b", re.I)
_NOT_PLACE = re.compile(r"\b(?:do\s+not|don'?t|not|no)\s+place", re.I)
_PLACE_WORD = re.compile(
    r"\b(?:close|near|next|adjacent|beside|under|right\s+at)\b", re.I)
_PINS = re.compile(
    r"\bpins?\s*[#(]?\s*(\d+(?:\s*(?:,|/|&|and|\+)\s*\d+)*)", re.I)
_TOKEN = re.compile(r"[A-Za-z][A-Za-z0-9_+\-.]*\d[A-Za-z0-9_+\-.]*")

_TOK = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()"]+')


class SchematicError(ValueError):
    """A schematic that cannot be read, named."""


# ---------------------------------------------------------------------------
# s-expressions
# ---------------------------------------------------------------------------

class _Str(str):
    """A quoted string, so a value is never mistaken for a keyword."""


def _sexp(text: str):
    stack: List[list] = [[]]
    for m in _TOK.finditer(text):
        t = m.group(0)
        if t == '(':
            stack.append([])
        elif t == ')':
            if len(stack) < 2:
                raise SchematicError('unbalanced parentheses')
            node = stack.pop()
            stack[-1].append(node)
        elif t[0] == '"':
            stack[-1].append(_Str(t[1:-1].replace('\\"', '"')
                                  .replace('\\n', '\n').replace('\\\\', '\\')))
        else:
            stack[-1].append(t)
    if len(stack) != 1 or not stack[0]:
        raise SchematicError('unbalanced parentheses')
    return stack[0][0]


def _head(n):
    if isinstance(n, list) and n and isinstance(n[0], str) \
            and not isinstance(n[0], _Str):
        return n[0]
    return None


def _kids(n, name):
    return [c for c in n[1:] if isinstance(c, list) and _head(c) == name]


def _kid(n, name):
    for c in n[1:]:
        if isinstance(c, list) and _head(c) == name:
            return c
    return None


def _num(x) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return 0.0


def _at(n) -> Tuple[float, float, float]:
    a = _kid(n, 'at')
    v = [_num(x) for x in (a[1:4] if a else ())]
    return tuple((v + [0.0, 0.0, 0.0])[:3])


def _props(n) -> Dict[str, str]:
    return {str(p[1]): str(p[2]) for p in _kids(n, 'property') if len(p) >= 3}


# ---------------------------------------------------------------------------
# one file
# ---------------------------------------------------------------------------

def _grow(box, x, y):
    if box is None:
        return [x, y, x, y]
    return [min(box[0], x), min(box[1], y), max(box[2], x), max(box[3], y)]


def _lib_extents(libsym) -> Dict[int, list]:
    """`{unit: [x0, y0, x1, y1]}` over a library symbol's pins and graphics,
    in library coordinates (y up). Unit 0 is common to every unit."""
    out: Dict[int, list] = {}

    def walk(node, unit):
        for c in node[1:]:
            if not isinstance(c, list):
                continue
            h = _head(c)
            if h == 'symbol':
                m = re.search(r'_(\d+)_(\d+)$', str(c[1]))
                walk(c, int(m.group(1)) if m else unit)
                continue
            pts = []
            if h == 'pin':
                pts.append(_at(c)[:2])
            elif h == 'rectangle':
                for k in ('start', 'end'):
                    q = _kid(c, k)
                    if q:
                        pts.append((_num(q[1]), _num(q[2])))
            elif h in ('polyline', 'bezier'):
                q = _kid(c, 'pts')
                pts += [(_num(p[1]), _num(p[2])) for p in _kids(q, 'xy')] \
                    if q else []
            elif h == 'circle':
                ce, ra = _kid(c, 'center'), _kid(c, 'radius')
                if ce and ra:
                    r = _num(ra[1])
                    pts += [(_num(ce[1]) - r, _num(ce[2]) - r),
                            (_num(ce[1]) + r, _num(ce[2]) + r)]
            elif h == 'arc':
                for k in ('start', 'mid', 'end'):
                    q = _kid(c, k)
                    if q:
                        pts.append((_num(q[1]), _num(q[2])))
            for x, y in pts:
                out[unit] = _grow(out.get(unit), x, y)
    walk(libsym, 0)
    return out


def _place(px, py, rot, mirror) -> Tuple[float, float]:
    """A library offset (y up) on the sheet (y down): KiCad's symbol
    transform -- rotation counter-clockwise on screen, then the mirror
    (`x` flips about the X axis, `y` about the Y axis)."""
    x, y = px, -py
    for _ in range((int(round(rot)) % 360) // 90):
        x, y = y, -x
    if mirror == 'x':
        y = -y
    elif mirror == 'y':
        x = -x
    return x, y


def _text_box(t, text: str) -> list:
    """An estimated box for a free text (KiCad stores the anchor only)."""
    x, y, rot = _at(t)
    h_size = w_size = 1.27
    just = set()
    eff = _kid(t, 'effects')
    if eff:
        font = _kid(eff, 'font')
        size = _kid(font, 'size') if font else None
        if size and len(size) >= 3:
            h_size, w_size = _num(size[1]) or 1.27, _num(size[2]) or 1.27
        j = _kid(eff, 'justify')
        if j:
            just = {str(v) for v in j[1:]}
    lines = text.split('\n') or ['']
    w = max(len(s) for s in lines) * w_size * 0.85
    h = len(lines) * h_size * 1.6
    x0 = 0.0 if 'left' in just else (-w if 'right' in just else -w / 2)
    y0 = -h if 'bottom' in just else (0.0 if 'top' in just else -h / 2)
    corners = [(x0, y0), (x0 + w, y0 + h)]
    if int(round(rot)) % 180 == 90:
        corners = [(cy, -cx) for cx, cy in corners]
    xs = [x + c[0] for c in corners]
    ys = [y + c[1] for c in corners]
    return [min(xs), min(ys), max(xs), max(ys)]


def read_file(path: str) -> Dict:
    """One `.kicad_sch`: its symbols, sheets and notes."""
    with open(path, encoding='utf-8', errors='replace') as fh:
        root = _sexp(fh.read())
    if _head(root) != 'kicad_sch':
        raise SchematicError(f'{os.path.basename(path)} is not a KiCad 6+ '
                             f'schematic')
    libs = {}
    lib_node = _kid(root, 'lib_symbols')
    for s in (_kids(lib_node, 'symbol') if lib_node else ()):
        libs[str(s[1])] = s
    extents: Dict[str, Dict[int, list]] = {}
    symbols = []
    for s in _kids(root, 'symbol'):
        lib_id = _kid(s, 'lib_id')
        lib_name = _kid(s, 'lib_name')
        key = str(lib_name[1]) if lib_name else (str(lib_id[1]) if lib_id
                                                 else '')
        x, y, rot = _at(s)
        mi = _kid(s, 'mirror')
        unit_n = _kid(s, 'unit')
        unit = int(_num(unit_n[1])) if unit_n else 1
        uid = _kid(s, 'uuid')
        pr = _props(s)
        if key not in extents:
            extents[key] = _lib_extents(libs[key]) if key in libs else {}
        box = None
        for u in (0, unit):
            g = extents[key].get(u)
            if not g:
                continue
            for gx, gy in ((g[0], g[1]), (g[2], g[3]), (g[0], g[3]),
                           (g[2], g[1])):
                dx, dy = _place(gx, gy, rot, str(mi[1]) if mi else '')
                box = _grow(box, x + dx, y + dy)
        ref = pr.get('Reference', '')
        lid = str(lib_id[1]) if lib_id else ''
        power = (ref.startswith('#') or lid.lower().startswith('power:')
                 or (key in libs and _kid(libs[key], 'power') is not None))
        symbols.append({'uuid': str(uid[1]) if uid else '', 'lib_id': lid,
                        'ref': ref, 'value': pr.get('Value', ''),
                        'unit': unit, 'box': box or [x, y, x, y],
                        'power': power})
    sheets = []
    for sh in _kids(root, 'sheet'):
        pr = _props(sh)
        uid = _kid(sh, 'uuid')
        sheets.append({'uuid': str(uid[1]) if uid else '',
                       'name': pr.get('Sheetname', pr.get('Sheet name', '')),
                       'file': pr.get('Sheetfile', pr.get('Sheet file', ''))})
    notes = []
    for kind in ('text', 'text_box'):
        for t in _kids(root, kind):
            text = str(t[1]) if len(t) > 1 else ''
            if not text.strip():
                continue
            if kind == 'text_box':
                x, y, _r = _at(t)
                size = _kid(t, 'size')
                box = [x, y, x + (_num(size[1]) if size else 0),
                       y + (_num(size[2]) if size else 0)]
            else:
                box = _text_box(t, text)
            notes.append({'text': text.strip(), 'box': box})
    return {'symbols': symbols, 'sheets': sheets, 'notes': notes}


# ---------------------------------------------------------------------------
# the tree
# ---------------------------------------------------------------------------

def discover(board_path: str) -> Tuple[Optional[str], List[str]]:
    """`(root schematic or None, the paths looked at)`: `<board>.kicad_sch`
    beside the board, else the one `.kicad_pro` in its folder named so."""
    stem = os.path.splitext(os.path.abspath(board_path))[0]
    looked = [stem + '.kicad_sch']
    if os.path.isfile(looked[0]):
        return looked[0], looked
    folder = os.path.dirname(stem)
    try:
        pros = sorted(n for n in os.listdir(folder)
                      if n.endswith('.kicad_pro'))
    except OSError:
        pros = []
    if len(pros) == 1:
        cand = os.path.join(folder, pros[0][:-len('.kicad_pro')] + '.kicad_sch')
        if cand not in looked:
            looked.append(cand)
            if os.path.isfile(cand):
                return cand, looked
    return None, looked


def read_tree(root_path: str) -> Dict:
    """Every file of the hierarchy under `root_path`, and its sheet
    instances `[(uuid prefix 'a/b', file path, (sheet names...))]` in
    depth-first order. A sheet file that cannot be found or read is listed
    in `missing`, never dropped silently."""
    files: Dict[str, Dict] = {}
    missing: List[str] = []
    instances: List[Tuple[str, str, Tuple[str, ...]]] = []

    def load(path):
        if path not in files:
            files[path] = read_file(path)
        return files[path]

    def walk(path, prefix, names, depth):
        instances.append((prefix, path, names))
        if depth > 16:
            return
        for sh in load(path)['sheets']:
            rel = sh['file'].replace('\\', '/')
            if not rel:
                continue
            cands = [os.path.normpath(os.path.join(os.path.dirname(path), rel)),
                     os.path.normpath(os.path.join(os.path.dirname(root_path),
                                                   rel))]
            child = next((c for c in cands if os.path.isfile(c)), None)
            if child is None:
                missing.append(rel)
                continue
            try:
                load(child)
            except (OSError, SchematicError) as exc:
                missing.append(f'{rel} ({exc})')
                continue
            walk(child, (prefix + '/' if prefix else '') + sh['uuid'],
                 names + (sh['name'],), depth + 1)

    walk(os.path.abspath(root_path), '', (), 0)
    return {'root': os.path.abspath(root_path), 'files': files,
            'instances': instances, 'missing': sorted(set(missing))}


def tree_files(root_path: str) -> Tuple[List[Tuple[str, str]], List[str]]:
    """`([(file, path relative to the root's folder)], [files outside it])`
    -- what a copy of the hierarchy must carry to resolve as it did."""
    tree = read_tree(root_path)
    base = os.path.dirname(tree['root'])
    inside, outside = [], []
    for path in tree['files']:
        rel = os.path.relpath(path, base)
        if rel.startswith('..') or os.path.isabs(rel):
            outside.append(path)
        else:
            inside.append((path, rel))
    return inside, outside


# ---------------------------------------------------------------------------
# blocks
# ---------------------------------------------------------------------------

def _gap(a, b) -> float:
    dx = max(0.0, max(a[0], b[0]) - min(a[2], b[2]))
    dy = max(0.0, max(a[1], b[1]) - min(a[3], b[3]))
    return math.hypot(dx, dy)


def _parts_gap(boxes_a, boxes_b) -> float:
    return min(_gap(a, b) for a in boxes_a for b in boxes_b)


def average_link(D: np.ndarray, cut: float) -> List[List[int]]:
    """Average-linkage clusters of a symmetric distance matrix, merged while
    the closest pair of clusters is within `cut`. Deterministic: ties go to
    the lowest index."""
    n = len(D)
    if n == 0:
        return []
    D = D.astype(float).copy()
    np.fill_diagonal(D, np.inf)
    members = {i: [i] for i in range(n)}
    while len(members) > 1:
        k = int(np.argmin(D))
        i, j = divmod(k, n)
        if not D[i, j] <= cut:
            break
        if j < i:
            i, j = j, i
        ni, nj = len(members[i]), len(members[j])
        row = (ni * D[i] + nj * D[j]) / (ni + nj)
        D[i, :] = row
        D[:, i] = row
        D[i, i] = np.inf
        D[j, :] = np.inf
        D[:, j] = np.inf
        members[i] += members.pop(j)
    return [sorted(m) for _i, m in sorted(members.items())]


# ---------------------------------------------------------------------------
# the board
# ---------------------------------------------------------------------------

def _natural(ref: str):
    return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)', ref)]


def _ref_for(tok: str, refs, values) -> Optional[str]:
    """The board ref a note's word names: a reference as the sheet spells it
    (`refs` maps it to the board's), or a part value only one part carries
    (`RP2040`, or the start of `GS2971A-IBE3`)."""
    tok = tok.strip('.,;:!?()')
    if tok in refs:
        return refs[tok]
    hit = values.get(tok.upper(), [])
    if len(hit) == 1:
        return hit[0]
    if len(tok) >= 5:
        hit = sorted({r for v, rs in values.items() if v.startswith(tok.upper())
                      for r in rs})
        if len(hit) == 1:
            return hit[0]
    return None


def _named(text: str, refs, values) -> List[Tuple[int, int, str, object]]:
    """What a note names, in order: `(start, end, 'ref', ref)` and
    `(start, end, 'pins', [numbers])`."""
    out = []
    for m in _TOKEN.finditer(text):
        hit = _ref_for(m.group(0), refs, values)
        if hit:
            out.append((m.start(), m.end(), 'ref', hit))
    for m in _PINS.finditer(text):
        out.append((m.start(), m.end(), 'pins', re.findall(r'\d+', m.group(1))))
    return sorted(out, key=lambda t: t[0])


def _keywords(sentence: str) -> List[Tuple[int, int]]:
    """Where a sentence asks for a placement: `close to`, `next to`, ... or,
    in a sentence with the verb `place` (never `do not place`), its `near` /
    `close` word."""
    hits = [m.span() for m in _LAYOUT_NOTE.finditer(sentence)]
    if not hits and _PLACE_VERB.search(sentence) \
            and not _NOT_PLACE.search(sentence):
        m = _PLACE_WORD.search(sentence)
        if m:
            hits.append(m.span())
    return hits


def _sentences(text: str) -> List[str]:
    """A note's sentences. A line break is not one: eeschema wraps a note by
    hand, so "C26 close to / MCU pin 44" is one instruction."""
    flat = ' '.join(text.split())
    return [s for s in re.split(r'(?<=[.;!?])\s+', flat) if s]


def _clauses(sentence: str, refs, values) -> List[Dict]:
    """`[{subjects, target, pins}]`, one per keyword: the parts named before
    it (since the last clause's target) are its subjects, and the first
    thing named after it -- a part, or a pin list and the part it is `of` --
    its target. "C10 close to pin 11 C11 close to pin 10" is two clauses."""
    kws = _keywords(sentence)
    items = _named(sentence, refs, values)
    out = []
    start = 0
    for k, (ks, ke) in enumerate(kws):
        stop = kws[k + 1][0] if k + 1 < len(kws) else len(sentence)
        subjects = [v for s, e, kind, v in items
                    if kind == 'ref' and start <= s and e <= ks]
        after = [t for t in items if ke <= t[0] < stop]
        target, pins, end = None, [], ke
        if after:
            s0, e0, kind, v = after[0]
            end = e0
            if kind == 'ref':
                target = v
            else:
                pins = v
                if len(after) > 1 and after[1][2] == 'ref' and re.search(
                        r'\b(?:of|on)\b', sentence[e0:after[1][0]], re.I):
                    target, end = after[1][3], after[1][1]
        out.append({'subjects': [r for r in subjects if r != target],
                    'target': target, 'pins': pins})
        start = end
    return out


def _chip_with_pads(pins, drawn, pcb_data, exclude) -> Optional[str]:
    """The part drawn nearest the note that has every pad `pins` names and
    more than three pads (a pin number on a passive names nothing)."""
    for _d, r in drawn:
        fp = pcb_data.footprints.get(r)
        pads = {str(p.pad_number) for p in (fp.pads or ())} if fp else set()
        if r not in exclude and len(pads) > 3 and all(p in pads for p in pins):
            return r
    return None


def context(pcb_data, root_path: str) -> Dict:
    """The schematic, matched to `pcb_data`'s footprints.

    Returns `parts` (`{ref: {sheet, block}}`), `sheets` (each instance with
    its blocks, their notes, and its single parts), `layout_notes` (with
    candidate `proximity[]` rows, `max_mm` null) and the match counts."""
    from placement.groups import sheet_and_symbol

    tree = read_tree(root_path)
    inst = {prefix: (path, names) for prefix, path, names in tree['instances']}
    by_uuid: Dict[str, Dict[str, dict]] = {}
    for path, f in tree['files'].items():
        for s in f['symbols']:
            if s['uuid'] and not s['power']:
                by_uuid.setdefault(path, {})[s['uuid']] = s

    on_board: Dict[str, List[Tuple[str, dict]]] = {}
    not_on_schematic = []
    copper_parts = matched_copper = 0
    for ref, fp in sorted((pcb_data.footprints or {}).items(),
                          key=lambda kv: _natural(kv[0])):
        copper = any((getattr(p, 'pad_type', '') or '') != 'np_thru_hole'
                     for p in (fp.pads or ()))
        copper_parts += bool(copper)
        prefix, sym = sheet_and_symbol(fp)
        hit = inst.get(prefix)
        s = by_uuid.get(hit[0], {}).get(sym) if hit else None
        if s is None:
            if copper:
                not_on_schematic.append(ref)
            continue
        matched_copper += bool(copper)
        on_board.setdefault(prefix, []).append((ref, s))

    refs = {r: r for r in (pcb_data.footprints or {})}
    values: Dict[str, List[str]] = {}
    for r, fp in (pcb_data.footprints or {}).items():
        v = (getattr(fp, 'value', '') or '').strip().upper()
        if v:
            values.setdefault(v, []).append(r)

    parts: Dict[str, Dict] = {}
    sheets = []
    layout_notes = []
    nb = 0
    for prefix, path, names in tree['instances']:
        f = tree['files'][path]
        members = on_board.get(prefix, [])
        sheet_name = '/'.join(names)
        boxes = []
        for ref, s in members:
            units = [s['box']]
            if s['ref'] and not s['ref'].endswith('?'):
                units += [o['box'] for o in f['symbols']
                          if o is not s and not o['power']
                          and o['ref'] == s['ref'] and o['lib_id'] == s['lib_id']]
            boxes.append(units)
        n = len(members)
        D = np.zeros((n, n))
        for i in range(n):
            for j in range(i + 1, n):
                D[i, j] = D[j, i] = _parts_gap(boxes[i], boxes[j])
        clusters = average_link(D, BLOCK_CUT_MM)

        def bbox(idx):
            b = None
            for i in idx:
                for u in boxes[i]:
                    b = _grow(_grow(b, u[0], u[1]), u[2], u[3])
            return b
        blocks = []
        for cl in clusters:
            if len(cl) < 2:
                continue
            blocks.append({'idx': cl, 'box': bbox(cl)})
        blocks.sort(key=lambda b: (round(b['box'][1] / BLOCK_CUT_MM),
                                   b['box'][0]))
        for b in blocks:
            nb += 1
            b['id'] = f'B{nb}'
            b['refs'] = sorted((members[i][0] for i in b['idx']), key=_natural)
            b['notes'] = []
        singles = sorted((members[cl[0]][0] for cl in clusters if len(cl) == 1),
                         key=_natural)
        for b in blocks:
            for r in b['refs']:
                parts[r] = {'sheet': sheet_name, 'block': b['id']}
        for r in singles:
            parts[r] = {'sheet': sheet_name, 'block': None}

        # A note names parts as the SHEET spells them: its own references
        # and values, mapped to this instance's board parts, before the
        # board's (a multi-instance sheet's file carries one instance's).
        note_refs = dict(refs)
        note_values = {k: list(v) for k, v in values.items()}
        for ref, s in members:
            if s['ref'] and not s['ref'].endswith('?'):
                note_refs[s['ref']] = ref
            v = (s['value'] or '').strip().upper()
            if v and ref not in note_values.setdefault(v, []):
                note_values[v].append(ref)
        loose = []
        for note in f['notes']:
            flat = ' '.join(note['text'].split())
            near = min(((_gap(note['box'], b['box']), k)
                        for k, b in enumerate(blocks)), default=None)
            if len(flat) <= NOTE_MAX_CHARS:
                if near is not None and near[0] <= NOTE_REACH_MM:
                    blocks[near[1]]['notes'].append(flat)
                elif members:
                    loose.append(flat)
            if members and any(_keywords(x) for x in _sentences(note['text'])):
                layout_notes.append(_layout_note(
                    note, flat, sheet_name, members, boxes, note_refs,
                    note_values, pcb_data))
        sheets.append({
            'sheet': sheet_name, 'file': os.path.relpath(
                path, os.path.dirname(tree['root'])).replace('\\', '/'),
            'parts': n,
            'blocks': [{'id': b['id'], 'refs': b['refs'], 'notes': b['notes']}
                       for b in blocks],
            'single': singles, 'notes': loose})
    # Both counts over parts with copper: a drilled mounting hole matched
    # to its symbol is still a part on the schematic, but `board_parts`
    # never counted it, and the ratio is read as a sync check.
    matched = matched_copper
    return {
        'root': os.path.basename(tree['root']),
        'files': len(tree['files']),
        'sheet_instances': len(tree['instances']),
        'missing_files': tree['missing'],
        'matched': matched,
        'board_parts': copper_parts,
        'not_on_schematic': not_on_schematic,
        'block_cut_mm': BLOCK_CUT_MM,
        'parts': parts,
        'sheets': [s for s in sheets if s['parts']],
        'layout_notes': layout_notes,
    }


def _layout_note(note, flat, sheet, members, boxes, refs, values, pcb_data):
    """One layout instruction: what it names, what is drawn nearest it, and a
    candidate `proximity[]` row (`max_mm` null) for each clause whose
    subject AND partner it names. A subject the note does not name is not
    guessed from the drawing: `drawn_nearest` is printed for the reader."""
    drawn = sorted(((min(_gap(note['box'], u) for u in boxes[i]), members[i][0])
                    for i in range(len(members))))
    nearest = [r for d, r in drawn[:3] if d <= NOTE_REACH_MM]
    named, pin_owner, pins, rows, stale, unresolved = [], [], [], [], [], 0
    why = f'schematic note: "{flat}"'
    for sentence in _sentences(note['text']):
        for c in _clauses(sentence, refs, values):
            target = c['target']
            for r in c['subjects'] + ([target] if target else []):
                if r not in named:
                    named.append(r)
            if target is None and c['pins']:
                target = _chip_with_pads(c['pins'], drawn, pcb_data,
                                         set(c['subjects']))
                if target and target not in pin_owner:
                    pin_owner.append(target)
            pins += [p for p in c['pins'] if p not in pins]
            if not c['subjects'] or target is None:
                unresolved += 1
                continue
            for subj in c['subjects']:
                if c['pins']:
                    row = {'ref': target, 'near': subj,
                           'pads': {target: list(c['pins'])},
                           'max_mm': None, 'why': why}
                else:
                    row = {'ref': subj, 'near': target,
                           'max_mm': None, 'why': why}
                bad = _stale(row, pcb_data)
                if bad:
                    stale.append(dict(row, stale=bad))
                else:
                    rows.append(row)
    return {'text': flat, 'sheet': sheet, 'named': named, 'pins': pins,
            'pin_owner': pin_owner, 'drawn_nearest': nearest, 'rows': rows,
            'stale': stale, 'unresolved_clauses': unresolved}


def _nets(fp):
    return {getattr(q, 'net_id', 0) for q in (fp.pads or ()) if getattr(q, 'net_id', 0)}


def _stale(row, pcb_data):
    """Why a note's row does not match the netlist, or None. A note is text
    and survives a re-annotation that renumbers the parts it names: on
    usb_fm_radio two of four pin notes name a cap on another rail. A pin row
    holds when the subject sits on a named pin's net; a part row when the
    two parts share a net."""
    fps = pcb_data.footprints
    if row.get('pads'):
        chip, subj = row['ref'], row['near']
        pins = set(row['pads'][chip])
        pin_nets = {q.net_id: q.net_name for q in (fps[chip].pads or ())
                    if str(q.pad_number) in pins and q.net_id}
        if not pin_nets:
            return f'{chip} has no pad {", ".join(sorted(pins))} on a net'
        if _nets(fps[subj]) & set(pin_nets):
            return None
        on = sorted({q.component_ref for nid in pin_nets
                     for q in pcb_data.nets[nid].pads
                     if q.component_ref not in (chip,)
                     and len(fps[q.component_ref].pads or ()) == 2},
                    key=_natural)
        return (f'{subj} is not on {chip} pin {", ".join(sorted(pins))}\'s net '
                f'({", ".join(sorted(set(pin_nets.values())))}); two-pad parts '
                f'on it: {", ".join(on) or "none"}')
    a, b = row['ref'], row['near']
    if _nets(fps[a]) & _nets(fps[b]):
        return None
    return f'{a} and {b} share no net'


def section(pcb_data, board_path: str, schematic: Optional[str] = None) -> Dict:
    """`context` for a board, with how the schematic was found; an error is
    returned named, never raised, so a sheet that could not read the
    schematic says so instead of reading as a board without one."""
    if schematic:
        root, looked = os.path.abspath(schematic), [schematic]
        if not os.path.isfile(root):
            return {'found': False, 'looked_for': looked,
                    'error': f'{schematic} does not exist'}
    else:
        root, looked = discover(board_path)
    if root is None:
        legacy = [os.path.splitext(p)[0] + '.sch' for p in looked
                  if os.path.isfile(os.path.splitext(p)[0] + '.sch')]
        out = {'found': False, 'looked_for': looked}
        if legacy:
            out['error'] = (f'{os.path.basename(legacy[0])} is a KiCad 5 '
                            f'schematic, which this reader does not parse')
        return out
    try:
        out = context(pcb_data, root)
    except (OSError, SchematicError, RecursionError) as exc:
        return {'found': True, 'looked_for': looked,
                'error': f'{type(exc).__name__}: {exc}'}
    out['found'] = True
    out['looked_for'] = looked
    return out
