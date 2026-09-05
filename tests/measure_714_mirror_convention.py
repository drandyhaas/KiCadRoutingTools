#!/usr/bin/env python3
"""#714 phase 0: what a layer flip actually has to do, measured -- not reasoned.

Issue #714 asks `placement.writer.write_placed_output` to be able to emit a
footprint on the other face. The failure mode is SILENT: `legality.footprint_side`
(legality.py:239-242) decides a part's side from `Footprint.layer`'s first
character alone, with no pad cross-check, so a block whose `(layer ...)` says
B.Cu while its pads still say F.Cu reports side B and computes every overlap,
courtyard and clearance number from F geometry, and nothing raises.

That makes the mirror convention a thing to MEASURE against pcbnew, not to take
from prose -- and the issue's own prose is one of the things this script
falsifies. NOT named `test_*.py`, so `tests/run_all.py` never collects it; it is
a measurement whose numbers the tests and the PR body then cite.

    python3 tests/measure_714_mirror_convention.py            # everything
    python3 tests/measure_714_mirror_convention.py --sections A,B,C
    python3 tests/measure_714_mirror_convention.py --json out.json

Sections:
  A  corpus census: every top-level child node kind of every footprint block
  B  the coordinate-literal grammar (does a text sign toggle round-trip?)
  C  fixture discrimination, under BOTH symmetry predicates
  D  generator_version per board (a SaveBoard diff is only readable on 10.0)
  E  the pcbnew mirror convention, per node kind, at five rotations
  F  involution: TOP_BOTTOM vs LEFT_RIGHT

E and F need pcbnew and re-exec into KiCad's python; A-D are pure Python and
run under any interpreter.
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, os.path.join(REPO, 'tests'))

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]

# The nodes whose numeric children are COORDINATES (mirror candidates), as
# opposed to sizes/widths/thicknesses, which must never be touched.
COORD_HEADS = ('at', 'start', 'end', 'mid', 'center', 'xy', 'offset')

# A coordinate literal, as this corpus actually spells them. Section B checks
# that this is the WHOLE grammar; a text sign toggle is an exact involution
# only over literals of this shape.
_NUM = re.compile(r'^-?\d+(\.\d+)?$')

# Fixtures named in the plan, with what each is the sole witness for.
FIXTURES = [
    ('tigard', 'J7', 'rot 0, 6 SMD pads -- the canonical measured case'),
    ('tigard', 'J1', 'rot -90, 30 THT pads -- rotation x flip composition'),
    ('tigard', 'U3', 'QFN-64-1EP, F.Paste-only aperture sub-pads'),
    ('tigard', 'LOGO2', '0 pads, fp_curve, B.Cu -- graphics with no pad signal'),
    ('glasgow_revC', 'U7', 'SOT-143 -- smallest asymmetric part'),
    ('glasgow_revC', 'SW1', 'rot 180'),
    ('glasgow_revC', 'U1', 'fp_poly + fp_circle + *.Cu'),
    ('glasgow_revC', 'R9', 'non-orthogonal rotation (45)'),
    ('orangecrab_ext_pll', 'J4', 'B.Cu -> F.Cu; (model (offset ...)); justify mirror'),
    ('orangecrab_ext_pll', 'U9', 'B.Cu; custom pad (primitives)'),
    ('rp2350_fpga_eensy_prePlane', 'U8', 'the only (drill ... (offset)) witness'),
    ('rp2350_fpga_eensy_prePlane', 'J2', 'the only footprint-internal (zone) witness'),
    ('rp2350_fpga_eensy_prePlane', 'SW1', 'fp_arc on a KiCad-10 board'),
    ('rp2350_fpga_eensy_prePlane', 'D1', 'non-orthogonal rotation (-45)'),
    ('ulx3s', 'BAT1', 'fp_arc, B.Cu -- pad-vacuous, graphics tier only'),
]

# Constructs the plan REFUSES rather than guesses at. Section A prints the
# corpus witness count for each; a count of 0 is the argument for refusing.
ZERO_COVERAGE_CANDIDATES = ('chamfer', 'rect_delta', 'fp_text_box', 'group',
                            'primitives', 'zone', 'fp_curve')


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable:
            continue
        if os.path.exists(cand):
            r = subprocess.run([cand, '-c', 'import pcbnew'], capture_output=True)
            if r.returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    # os.execv re-splits argv on spaces through the CRT on
                    # Windows ("C:\Program Files\..." tears in two).
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("ERROR: no python with pcbnew found -- sections E/F cannot run")
    sys.exit(2)


def boards():
    """The TRACKED boards. Never a plain glob -- see run_utils.corpus_boards."""
    from run_utils import corpus_boards
    out = corpus_boards()
    if not out:
        print("ERROR: git could not name the tracked corpus; refusing to "
              "measure against a set this script cannot identify")
        sys.exit(2)
    return out


def child_nodes(fp_text):
    """(head, start, end) for each TOP-LEVEL child node of a footprint block.

    Uses the shipped string-aware `find_matching_paren`, so a lone paren inside
    a property value (an MPN like "TCR2EF115,LM(CT", #113) cannot run the walk
    out of one node into the next.
    """
    from kicad_parser import find_matching_paren
    # Skip the `(footprint "name"` header: advance past the quoted name.
    i = fp_text.index('"')
    i = fp_text.index('"', i + 1) + 1
    n = len(fp_text)
    while i < n:
        c = fp_text[i]
        if c == '"':
            i += 1
            while i < n and fp_text[i] != '"':
                i += 2 if fp_text[i] == '\\' else 1
            i += 1
            continue
        if c == '(':
            end = find_matching_paren(fp_text, i)
            m = re.match(r'\(\s*([A-Za-z_][A-Za-z_0-9]*)', fp_text[i:end])
            yield (m.group(1) if m else '?'), i, end
            i = end
            continue
        i += 1


def descendant_heads(text):
    """Every `(head` anywhere inside `text`, counted. Nested, not just top level."""
    return collections.Counter(m.group(1)
                               for m in re.finditer(r'\(\s*([A-Za-z_][A-Za-z_0-9]*)',
                                                    text))


# ------------------------------------------------------------------ section A

def section_a(paths):
    from kicad_parser import iter_footprint_blocks
    top = collections.Counter()
    deep = collections.Counter()
    blocks = 0
    witnesses = collections.defaultdict(list)
    layer_tokens = collections.Counter()
    for p in paths:
        name = os.path.splitext(os.path.basename(p))[0]
        text = open(p, encoding='utf-8').read()
        for _s, _e, fp_text, _raw, key in iter_footprint_blocks(text):
            blocks += 1
            heads = {h for h, _a, _b in child_nodes(fp_text)}
            top.update(heads)
            d = descendant_heads(fp_text)
            deep.update(d)
            for k in ZERO_COVERAGE_CANDIDATES:
                if d.get(k):
                    witnesses[k].append(f"{name}:{key}")
            for m in re.finditer(r'\(layers?\s+([^)]*)\)', fp_text):
                layer_tokens.update(re.findall(r'"([^"]+)"', m.group(1)))
    print(f"A. corpus census -- {len(paths)} tracked boards, {blocks} footprint blocks")
    print("   top-level child kinds (blocks containing at least one):")
    for k, v in sorted(top.items(), key=lambda kv: -kv[1]):
        print(f"     {k:<36} {v:6d}")
    print("   layer tokens appearing inside a footprint block:")
    print("     " + ' '.join(sorted(layer_tokens)))
    print("   refusal candidates, by witness count:")
    for k in ZERO_COVERAGE_CANDIDATES:
        w = witnesses[k]
        shown = ', '.join(w[:4]) + (' ...' if len(w) > 4 else '')
        print(f"     {k:<14} {len(w):5d} block(s)   {shown or '-- NO WITNESS'}")
    return {'blocks': blocks, 'top_level_kinds': dict(top),
            'all_kinds': dict(deep), 'layer_tokens': sorted(layer_tokens),
            'refusal_witnesses': {k: witnesses[k] for k in ZERO_COVERAGE_CANDIDATES}}


# ------------------------------------------------------------------ section B

def section_b(paths):
    """Is a TEXT sign toggle an exact involution over this corpus's literals?

    It is, if and only if every coordinate literal matches `-?\\d+(\\.\\d+)?`.
    Going through float instead would rewrite `0.500` as `0.5`, and the
    round-trip gate would then be pinning the formatter, not the transform.
    """
    from kicad_parser import iter_footprint_blocks
    total = 0
    bad = []
    for p in paths:
        text = open(p, encoding='utf-8').read()
        for _s, _e, fp_text, _raw, key in iter_footprint_blocks(text):
            for head, a, b in child_nodes(fp_text):
                node = fp_text[a:b]
                for m in re.finditer(r'\(\s*(%s)\s+([^()]*?)\)'
                                     % '|'.join(COORD_HEADS), node):
                    for tok in m.group(2).split():
                        total += 1
                        if not _NUM.match(tok):
                            bad.append((os.path.basename(p), key, head,
                                        m.group(1), tok))
    print(f"B. coordinate literals: {total} tokens, {len(bad)} outside "
          f"-?\\d+(\\.\\d+)? ")
    for row in bad[:20]:
        print("     OUTSIDE GRAMMAR:", row)
    if not bad:
        print("     => a text sign toggle is an exact involution on this corpus")
    return {'coord_tokens': total, 'outside_grammar': bad[:50]}


# ------------------------------------------------------------------ section C

def _pad_key_pos(p):
    return (round(p.local_x, 6), round(p.local_y, 6))


def _pad_key_tuple(p):
    return (p.pad_number, p.pad_type, p.shape,
            round(p.local_x, 6), round(p.local_y, 6),
            round((getattr(p, 'rotation', 0.0) or 0.0) % 360, 4))


def _mirror(key, axis):
    k = list(key)
    i = 3 if len(k) > 2 and isinstance(k[0], str) else 0
    # position tuple (x, y) vs full tuple (..., x, y, ang)
    if len(k) == 2:
        xi, yi = 0, 1
    else:
        xi, yi = 3, 4
    if axis == 'y':
        k[yi] = round(-k[yi], 6)
    else:
        k[xi] = round(-k[xi], 6)
    if len(k) > 2:
        k[5] = round((-k[5]) % 360, 4)
    return tuple(k)


def section_c(paths):
    from kicad_parser import parse_kicad_pcb
    by_board = {os.path.splitext(os.path.basename(p))[0]: p for p in paths}
    cache = {}
    rows = []
    for board, ref, why in FIXTURES:
        p = by_board.get(board)
        if p is None:
            rows.append((board, ref, 'BOARD NOT TRACKED', why))
            continue
        if board not in cache:
            cache[board] = parse_kicad_pcb(p)
        fp = cache[board].footprints.get(ref)
        if fp is None:
            rows.append((board, ref, 'REF MISSING', why))
            continue
        pads = fp.pads or []
        if not pads:
            rows.append((board, ref, dict(pads=0, tier='graphic-only'), why))
            continue
        pos = sorted(_pad_key_pos(q) for q in pads)
        tup = sorted(_pad_key_tuple(q) for q in pads)
        d_pos_y = pos != sorted(_mirror(k, 'y') for k in pos)
        d_tup_y = tup != sorted(_mirror(k, 'y') for k in tup)
        d_axis = (sorted(_mirror(k, 'y') for k in pos)
                  != sorted(_mirror(k, 'x') for k in pos))
        rows.append((board, ref,
                     dict(pads=len(pads), rot=fp.rotation, layer=fp.layer,
                          D1_pos=d_pos_y, D1_tuple=d_tup_y, D2_axis=d_axis,
                          tier='pad' if d_tup_y else 'graphic-only'), why))
    print("C. fixture discrimination "
          "(D1 = the y-mirror is observable; D2 = a wrong-axis mutant is observable)")
    for board, ref, info, why in rows:
        print(f"   {board:<28} {ref:<6} {info}")
        print(f"        {why}")

    # The corpus-wide symmetric fraction under THREE predicates, because the
    # number depends on the predicate and the three answer different
    # questions. Publishing one of them alone is how a safety margin gets
    # quoted at twice its real size.
    #
    #   position          {(x, y)} closed under y -> -y
    #                     "would a bare POSITION comparison notice the mirror?"
    #                     -- the weakest, and the one the issue's 89.2% is.
    #
    #   tuple             {(num, type, shape, x, y, angle)} with the angle
    #                     NEGATED on the mirrored side
    #                     "would a full pad-tuple comparison notice an
    #                     implementation that changes nothing but the layer
    #                     token?" -- the angle term does most of the work here.
    #
    #   tuple_angle_held  the same tuple with the angle IDENTICAL on both sides
    #                     "how often is the Y-MIRROR ITSELF unobservable?"
    #                     -- i.e. against the mutant that negates angles and
    #                     layers but forgets y, which is exactly the local-X /
    #                     local-Y confusion the issue's own prose invites.
    #
    # That third one is the number an implementer needs and it is the worst of
    # the three. Measured: 90.8% / 40.3% / 76.6%. Reading 40.3% as the margin
    # against a forgotten y-mirror is off by nearly a factor of two.
    def _hold_angle(k):
        return (k[0], k[1], k[2], k[3], k[4], 0.0)

    stats = {}
    for label, keyfn, mirrorfn in (
            ('position', _pad_key_pos, lambda k: _mirror(k, 'y')),
            ('tuple', _pad_key_tuple, lambda k: _mirror(k, 'y')),
            ('tuple_angle_held', lambda q: _hold_angle(_pad_key_tuple(q)),
             lambda k: _mirror(_hold_angle(k), 'y'))):
        tot = sym = 0
        for p in paths:
            for _ref, fp in parse_kicad_pcb(p).footprints.items():
                if not fp.pads:
                    continue
                tot += 1
                ks = sorted(keyfn(q) for q in fp.pads)
                if ks == sorted(mirrorfn(k) for k in ks):
                    sym += 1
        stats[label] = (sym, tot, round(100.0 * sym / tot, 1) if tot else 0.0)
        print(f"   y-symmetric under the {label:<17} predicate: "
              f"{sym}/{tot} = {stats[label][2]}%")
    return {'fixtures': [(b, r, i if isinstance(i, dict) else str(i))
                         for b, r, i, _w in rows],
            'symmetric_fraction': stats}


# ------------------------------------------------------------------ section D

def section_d(paths):
    print("D. generator_version (a pcbnew SaveBoard diff is only readable on 10.0)")
    out = {}
    for p in paths:
        head = open(p, encoding='utf-8').read(4000)
        m = re.search(r'\(generator_version\s+"([^"]+)"', head)
        v = m.group(1) if m else '?'
        out[os.path.splitext(os.path.basename(p))[0]] = v
    for k, v in sorted(out.items(), key=lambda kv: (kv[1], kv[0])):
        print(f"     {v:<6} {k}")
    tens = [k for k, v in out.items() if v.startswith('10')]
    print(f"   => {len(tens)} of {len(out)} boards are 10.x; parity fixtures "
          f"must come from those")
    return out


# ------------------------------------------- sections E/F (need pcbnew)

def _fp_signature(pcbnew, fp):
    """Every mirror-relevant field, keyed so two states can be compared.

    ALL text-bearing nodes, not just Reference and Value. Sampling two of them
    UNDERCOUNTS, and it undercounted here: a footprint carries ~5.7 text nodes
    corpus-wide, 736 of them sit on the opposite face from their footprint's
    own `(layer ...)`, and it is the excluded ones -- `Datasheet` and
    `Description` on F.Fab -- that make LEFT_RIGHT non-involutive on tigard J1
    and LOGO2. Measured: the two-field sample reported LEFT_RIGHT failing on
    4 of 15 fixtures, the full sample reports 6 of 15. The conclusion held
    (LEFT_RIGHT is lossy) but the published count was wrong, and wrong in the
    direction that made the measurement look tidier than it was.
    """
    sig = {'layer': pcbnew.LayerName(fp.GetLayer()),
           'orient': round(fp.GetOrientationDegrees() % 360, 4), 'pads': [],
           'texts': []}
    for p in fp.Pads():
        r = p.GetFPRelativePosition()
        sig['pads'].append((p.GetNumber(), r.x, r.y,
                            round(p.GetOrientationDegrees() % 360, 4),
                            tuple(sorted(pcbnew.LayerName(l)
                                         for l in p.GetLayerSet().Seq()))))
    # Keyed by UUID, never sorted. Sorting looked harmless and was not: a
    # flip changes a text's y, angle, mirror flag and layer, so the sort order
    # changes with it and a before/after zip then pairs unrelated texts --
    # which reported `txt_lay=False` and `txt180=False` on six fixtures whose
    # texts were in fact correct. pcbnew preserves uuids across a Flip, so the
    # uuid is the one field that identifies the same node on both sides.
    # `GetFields()`, not `Reference()` + `Value()`. A footprint's fields are
    # Reference, Value, Datasheet and Description; the first two are the ones
    # a human sees and the LAST TWO are the ones that make LEFT_RIGHT
    # non-involutive on tigard J1 and LOGO2 (both sit on F.Fab and both fold
    # 180 -> 0 / 270 -> 90 under a double LEFT_RIGHT flip). Sampling the
    # visible two reported LEFT_RIGHT failing on 4 of 15 fixtures; sampling
    # all four reports 6 of 15. `GraphicalItems()` carries `fp_text` and does
    # NOT carry fields, so both sources are needed.
    if hasattr(fp, 'GetFields'):
        texts = list(fp.GetFields())
    else:
        texts = [fp.Reference(), fp.Value()]
    for it in fp.GraphicalItems():
        if hasattr(it, 'GetTextAngleDegrees'):
            texts.append(it)
    sig['texts'] = {}
    for t in texts:
        r = t.GetFPRelativePosition()
        sig['texts'][t.m_Uuid.AsString()] = (
            r.x, r.y, round(t.GetTextAngleDegrees() % 360, 4),
            bool(t.IsMirrored()), pcbnew.LayerName(t.GetLayer()))
    return sig


def _paired(before, after):
    """(before_row, after_row) for every text present on BOTH sides, by uuid."""
    return [(before['texts'][k], after['texts'][k])
            for k in before['texts'] if k in after['texts']]


def _flip(pcbnew, fp, direction):
    fp.Flip(fp.GetPosition(), direction)


def section_e(paths):
    import pcbnew
    from kicad_parser import flip_layer_token
    tb = getattr(pcbnew, 'FLIP_DIRECTION_TOP_BOTTOM', False)
    by_board = {os.path.splitext(os.path.basename(p))[0]: p for p in paths}
    print("E. the pcbnew mirror convention "
          f"(pcbnew {pcbnew.GetBuildVersion()}, FLIP_DIRECTION_TOP_BOTTOM={tb!r})")
    out = {}
    for board, ref, _why in FIXTURES:
        p = by_board.get(board)
        if p is None:
            continue
        b = pcbnew.LoadBoard(p)
        fp = b.FindFootprintByReference(ref)
        if fp is None:
            print(f"   {board}:{ref} MISSING")
            continue
        before = _fp_signature(pcbnew, fp)
        _flip(pcbnew, fp, tb)
        after = _fp_signature(pcbnew, fp)
        # The three rules, stated as checks rather than as prose.
        pad_x_fixed = all(a[1] == c[1] for a, c in zip(before['pads'], after['pads']))
        pad_y_neg = all(a[2] == -c[2] for a, c in zip(before['pads'], after['pads']))
        # ASSERTED, not merely captured. The first version of this script read
        # pad angles and pad layer sets into the signature and then checked
        # NEITHER, so "footprint and pad angles negate, 15/15" rested on a
        # check that only ever looked at the footprint's own orientation.
        pad_ang_neg = all(round((-a[3]) % 360, 4) == c[3]
                          for a, c in zip(before['pads'], after['pads']))
        pad_layers_flipped = all(
            tuple(sorted(flip_layer_token(x) for x in a[4])) == c[4]
            for a, c in zip(before['pads'], after['pads']))
        _tx = _paired(before, after)
        txt_layers_flipped = all(flip_layer_token(a[4]) == c[4] for a, c in _tx)
        layer_flipped = before['layer'][0] != after['layer'][0]
        txt_180 = all(round((180 - a[2]) % 360, 4) == c[2] for a, c in _tx)
        # The mirror FLAG toggles only for a text on a SIDED layer. Measured
        # against pcbnew 10.0.0: a text on Cmts.User / Eco1.User / Dwgs.User
        # has its y and its angle mirrored like any other and keeps its
        # justify untouched -- there is no face for it to be seen from the
        # wrong side of. 28 flip-eligible texts on the tracked corpus sit on a
        # User layer. Reported separately so the qualifier cannot be lost:
        # the unconditional form of this check is what shipped a writer that
        # added `(justify mirror)` to all of them.
        _sided = [(a, c) for a, c in _tx if a[4][:2] in ('F.', 'B.')]
        _unsided = [(a, c) for a, c in _tx if a[4][:2] not in ('F.', 'B.')]
        mirror_tog = all(a[3] != c[3] for a, c in _sided)
        mirror_held = all(a[3] == c[3] for a, c in _unsided)
        orient_neg = round((-before['orient']) % 360, 4) == after['orient']
        print(f"   {board:<28} {ref:<6} pads={len(before['pads']):3d} "
              f"{before['layer']}->{after['layer']}  "
              f"x_fixed={pad_x_fixed} y_neg={pad_y_neg} layer={layer_flipped} "
              f"pad_ang={pad_ang_neg} pad_lay={pad_layers_flipped} "
              f"txt_lay={txt_layers_flipped} txt180={txt_180} "
              f"mirror_tog={mirror_tog} mirror_held={mirror_held} "
              f"orient_neg={orient_neg}")
        # DISCRIMINATION, printed beside the verdict, because a check run on a
        # fixture that cannot see it is not evidence. Measured: `orient_neg` is
        # vacuous on 7 of 15 (rot 0 or 180, where -r == r) and `pad_y_neg` on 3
        # -- tigard LOGO2 (no pads), ulx3s BAT1 (every pad at local x=0) and
        # glasgow_revC R9 (every pad at local y=0), which is the set's ONLY
        # 45-degree fixture, so the fixture carrying non-orthogonal rotation
        # carries nothing at all about the y-mirror.
        print(f"        [live: y={any(a[2] for a in before['pads'])} "
              f"x={any(a[1] for a in before['pads'])} "
              f"rot={round(before['orient'] % 180, 4) != 0.0} "
              f"padang={any(a[3] % 180 for a in before['pads'])} "
              f"unsided_texts={len(_unsided)}]")
        out[f"{board}:{ref}"] = dict(pad_x_fixed=pad_x_fixed, pad_y_neg=pad_y_neg,
                                     pad_angles_negate=pad_ang_neg,
                                     pad_layers_flipped=pad_layers_flipped,
                                     text_layers_flipped=txt_layers_flipped,
                                     mirror_held_on_unsided=mirror_held,
                                     live_y=any(a[2] for a in before['pads']),
                                     live_x=any(a[1] for a in before['pads']),
                                     live_rot=round(before['orient'] % 180, 4) != 0.0,
                                     unsided_texts=len(_unsided),
                                     layer_flipped=layer_flipped,
                                     text_180_minus_a=txt_180,
                                     mirror_toggles=mirror_tog,
                                     orientation_negates=orient_neg,
                                     before_layer=before['layer'],
                                     after_layer=after['layer'])
    return out


def section_f(paths):
    import pcbnew
    dirs = [('TOP_BOTTOM', getattr(pcbnew, 'FLIP_DIRECTION_TOP_BOTTOM', False)),
            ('LEFT_RIGHT', getattr(pcbnew, 'FLIP_DIRECTION_LEFT_RIGHT', True))]
    by_board = {os.path.splitext(os.path.basename(p))[0]: p for p in paths}
    print("F. involution: flip twice and compare "
          "(byte-identity round-trip is only statable for an involution)")
    out = {}
    for board, ref, _why in FIXTURES:
        p = by_board.get(board)
        if p is None:
            continue
        for name, d in dirs:
            b = pcbnew.LoadBoard(p)
            fp = b.FindFootprintByReference(ref)
            if fp is None:
                continue
            s0 = _fp_signature(pcbnew, fp)
            _flip(pcbnew, fp, d)
            _flip(pcbnew, fp, d)
            ok = _fp_signature(pcbnew, fp) == s0
            out[f"{board}:{ref}:{name}"] = ok
            if not ok:
                s1 = _fp_signature(pcbnew, fp)
                diff = [(a, c) for a, c in zip(s0['texts'], s1['texts']) if a != c]
                print(f"   {board:<28} {ref:<6} {name:<11} involution=False  {diff}")
            else:
                print(f"   {board:<28} {ref:<6} {name:<11} involution=True")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--sections', default='ABCDEF')
    ap.add_argument('--json')
    args = ap.parse_args(argv)
    want = set(args.sections.upper())
    need_pcbnew = bool(want & set('EF'))
    if need_pcbnew:
        try:
            import pcbnew  # noqa: F401
        except ImportError:
            _reexec_into_kicad()

    paths = boards()
    doc = {'basis': subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=REPO,
                                   capture_output=True, text=True).stdout.strip(),
           'boards': len(paths)}
    for letter, fn in (('A', section_a), ('B', section_b), ('C', section_c),
                       ('D', section_d), ('E', section_e), ('F', section_f)):
        if letter in want:
            print()
            doc[letter] = fn(paths)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True, default=str)
        print(f"\nJSON -> {args.json}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
