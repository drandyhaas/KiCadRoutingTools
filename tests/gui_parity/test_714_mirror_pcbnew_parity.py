#!/usr/bin/env python3
"""#714 PAR: the ACCEPTANCE criterion -- our flip against KiCad's own.

    For every fixture, the footprint block `write_placed_output` emits for a
    side change is -- after a declared, enumerated normalisation -- node for
    node and number for number identical to the block pcbnew itself writes
    after `fp.Flip(pos, FLIP_DIRECTION_TOP_BOTTOM)` + `SaveBoard()`.

WHY NOT THE COMPARISON THE ISSUE SUGGESTS. #714 proposes comparing resolved
`Pad.global_x/global_y/size_x/size_y/layers` plus `footprint.layer` through both
parse paths. That is a smoke test, and it cannot see the bug it is written for:

  * `local_to_global` (kicad_parser.py:640-671) has NO mirror term -- KiCad
    stores B-side children pre-mirrored. So toggle `(layer "F.Cu")` to B.Cu and
    toggle the pad `(layers)`, but forget the local y, and every pad's GLOBAL
    position comes out exactly as before, on both parse paths, while
    `footprint.layer` reads B. Every field the issue names passes. That is the
    headline silent failure, and the suggested test is blind to it.
  * Comparing two PARSERS over one file tests the parsers, not the writer. The
    oracle has to be a different DOCUMENT: pcbnew's own flip of the original.
  * PCBData models a minority of a block. Against 1319 pad-bearing footprints
    the tracked corpus carries 1206 blocks with `fp_line`, 1215 with `model`,
    1077 with `fp_text`, 132 `fp_circle`, 85 `fp_poly`, 54 `fp_arc`, 46
    `fp_rect`. No pad field compares any of them.
  * And `fp_arc`'s start/end swap is GEOMETRICALLY INVISIBLE: mirroring
    start/mid/end without exchanging start and end yields the same curve
    through the same three points. Only a node-level comparison against KiCad's
    own serialisation can see it.

NORMALISATION, declared because every normalisation is a place to hide a bug:
numbers to integer NANOMETRES (`round(mm*1e6)`, the #493 `local_to_global`
convention -- both sides are nm-quantised so the tolerance is exactly ZERO, not
a float epsilon); angles folded to `round(a % 360, 4)`; child order sorted by
canonical content, because pcbnew re-serialises in its own order. `uuid` is
deliberately NOT normalised away -- the uuid multiset must be preserved, which
buys group membership, netlist back-annotation and #726 keying in one line.

FIXTURES ARE ALL `generator_version "10.0"` boards. 7 of the 22 tracked boards
are 9.0, and `SaveBoard` format-upgrades those, so the diff would be unreadable
for reasons unrelated to the flip.

ABSENCE IS AN ERROR, NOT A SKIP (exit 2). This is the acceptance criterion, and
`tests/mutate_714.py` uses it as a killer gate: a gate that self-skips at exit 0
reports every mutation row it guards as SURVIVED.

    python3 tests/gui_parity/test_714_mirror_pcbnew_parity.py
    python3 tests/gui_parity/test_714_mirror_pcbnew_parity.py --show 40
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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

# (board, ref) -- every board here is generator_version 10.0.
FIXTURES = [
    ('tigard', 'J7'),        # rot 0, 6 SMD pads
    ('tigard', 'J1'),        # rot -90, 30 THT pads, *.Cu-adjacent, 2 models
    ('tigard', 'U3'),        # 91 pads, F.Paste-only aperture sub-pads
    ('tigard', 'LOGO2'),     # 0 pads, fp_curve, B.Cu
    ('glasgow_revC', 'U7'),  # SOT-143, x- and y-asymmetric
    ('glasgow_revC', 'SW1'),  # rot 180
    ('glasgow_revC', 'U1'),  # fp_poly + fp_circle + *.Cu
    ('glasgow_revC', 'R9'),  # non-orthogonal rotation, 45
    ('orangecrab_ext_pll', 'J4'),   # B->F, (justify mirror), model offsets
    ('rp2350_fpga_eensy_prePlane', 'U8'),   # the only (drill (offset)) witness
    ('rp2350_fpga_eensy_prePlane', 'SW1'),  # fp_arc on a 10.0 board
    ('rp2350_fpga_eensy_prePlane', 'D1'),   # non-orthogonal rotation, -45
    ('ulx3s', 'BAT1'),       # 19 fp_arc, B.Cu -- graphics tier
    # Texts on a NON-sided layer. pcbnew mirrors their y and angle but leaves
    # `(justify ...)` alone -- there is no face to be seen from the wrong side
    # of. 28 flip-eligible texts on the tracked corpus sit on a User layer and
    # NONE of the fixtures above carries one, so this divergence shipped
    # invisibly until these two were added.
    ('orangecrab_ext_pll', 'U3'),   # 2 fp_text on Cmts.User
    ('orangecrab_ext_pll', 'J1'),   # property on Eco1.User
]

# Each fixture is compared TWICE: once against pcbnew's bare flip, and once
# against a flip COMPOSED with a rotation the caller chose. The composed pass
# exists because its absence hid a real bug -- the text-angle rule was the
# constant `180 - a`, which is the general composition only when
# `new_rot == -old_rot`, i.e. only for the bare flip this gate used to be the
# whole of. Every shipped consumer asks for something else: `perturb`'s
# `layer_flip` HOLDS the pose, so its delta is 2R, and 9 of tigard's 33 flipped
# parts shipped a reference designator rotated 180 degrees from where KiCad
# puts it -- relative to their own pads, which had rotated correctly.
COMPOSED_DELTAS = (None, 90.0, 180.0)
PASSES = [(b, r, d) for (b, r) in FIXTURES for d in COMPOSED_DELTAS]

# A run that compared nothing must fail. Floors are below today's counts so a
# board changing is not a failure, and far above zero so a vacuous run is.
MIN_FIXTURES = 36
MIN_PADS = 1200
MIN_NON_PAD_NODES = 1200
# Surfaces at least one fixture must actually carry, so the set cannot silently
# stop covering them.
REQUIRED_SURFACES = ('fp_arc', 'fp_curve', 'model', 'fp_poly', 'fp_circle')

_NUM = re.compile(r'^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$')


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
    print("ERROR: no python with pcbnew found. This is the ACCEPTANCE gate for "
          "#714; it does not self-skip, because a killer gate that exits 0 "
          "reports every mutation row it guards as SURVIVED.")
    sys.exit(2)


# ------------------------------------------------------------- s-expressions

def parse_sexpr(text):
    """Nested lists of tokens. Test-side only -- the engine has no parser."""
    out, stack, i, n = [], [], 0, len(text)
    cur = out
    while i < n:
        c = text[i]
        if c == '(':
            new = []
            cur.append(new)
            stack.append(cur)
            cur = new
            i += 1
        elif c == ')':
            cur = stack.pop()
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while j < n and text[j] != '"':
                if text[j] == '\\':
                    buf.append(text[j + 1]); j += 2
                else:
                    buf.append(text[j]); j += 1
            cur.append('"' + ''.join(buf) + '"')
            i = j + 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in '()"':
                j += 1
            cur.append(text[i:j])
            i = j
    return out[0] if len(out) == 1 else out


def _num(tok, angle=False):
    if not isinstance(tok, str) or not _NUM.match(tok):
        return tok
    v = float(tok)
    return round(v % 360, 4) if angle else int(round(v * 1e6))


def canon(node):
    """Canonical form: numbers to nm, `at` angles folded, children sorted."""
    if isinstance(node, str):
        return node
    if not node:
        return ()
    head = node[0] if isinstance(node[0], str) else canon(node[0])
    rest = node[1:]
    if head == 'at':
        vals = [_num(t) for t in rest[:2]]
        # A MISSING angle is zero, and is normalised to it. Declared because it
        # is a real normalisation and not a formatting nicety: this writer
        # PRESERVES the angle token's presence (which is what buys the
        # byte-identical round trip) while pcbnew drops a zero, so under a
        # composed rotation the same pad is `(at x y 0)` here and `(at x y)`
        # there. Same angle, different arity.
        vals += [_num(rest[2], angle=True) if len(rest) > 2 else 0.0]
        vals += [canon(t) for t in rest[3:]]
        return (head,) + tuple(vals)
    kids = [canon(t) if not isinstance(t, str) else _num(t) for t in rest]
    scalars = tuple(k for k in kids if not isinstance(k, tuple))
    subs = tuple(sorted((k for k in kids if isinstance(k, tuple)), key=repr))
    return (head,) + scalars + subs


def _block(path, ref):
    from kicad_parser import iter_footprint_blocks
    text = open(path, encoding='utf-8').read()
    for _s, _e, fp_text, _raw, key in iter_footprint_blocks(text):
        if key == ref:
            return fp_text
    return None


def _uuids(text):
    import collections
    return collections.Counter(re.findall(r'\(uuid\s+"([^"]+)"\)', text))


def _count_nodes(text):
    import collections
    return collections.Counter(re.findall(r'\(\s*([A-Za-z_][A-Za-z_0-9]*)', text))


# pcbnew's own flip LOSES A NANOMETRE on an arc's mid point, and ours does not.
# Measured: a plain `LoadBoard` + `SaveBoard` round trip preserves rp2350 SW1's
# `(mid 1.40384 0)` exactly, but `fp.Flip(...)` + `SaveBoard` writes
# `1.403839` -- pcbnew holds an arc as centre/radius/angles and re-derives mid
# from the polar form when it mirrors. This transform never touches an arc's X
# at all, so it emits the board's own value verbatim.
#
# So this is a divergence in OUR favour, and it is waived rather than matched:
# adopting pcbnew's rounding would make the transform lossy and would break the
# flip-and-flip-back byte identity `tests/test_714_flip_roundtrip.py` pins.
#
# The waiver is deliberately narrow and LOUD -- `mid` only, one nanometre only,
# every instance printed, and a cap, so it cannot quietly widen into a blanket
# float tolerance that would hide a real disagreement.
_MID_WAIVER_NM = 1
# Two arcs waive, once per pass. Stated as the product rather than as the
# number, so adding a pass does not silently push the run through a cap it
# was sitting exactly on -- it was at 6 of 6 the moment the composed
# passes landed, which is a boundary, not a margin.
MAX_WAIVED = 2 * 3 + 3


def _diff(a, b, path='', out=None, limit=40, waived=None):
    """First differing paths between two canonical trees."""
    out = [] if out is None else out
    if len(out) >= limit:
        return out
    if type(a) is not type(b):
        out.append(f"{path}: type {type(a).__name__} vs {type(b).__name__}: "
                   f"{a!r} vs {b!r}")
        return out
    if not isinstance(a, tuple):
        if a != b:
            if (waived is not None and '/mid[' in path
                    and isinstance(a, int) and isinstance(b, int)
                    and abs(a - b) <= _MID_WAIVER_NM):
                waived.append(f"{path}: {a} (ours, = the board's own value) "
                              f"vs {b} (pcbnew, re-derived from centre/angles)")
                return out
            out.append(f"{path}: {a!r} (ours) vs {b!r} (pcbnew)")
        return out
    if len(a) != len(b):
        out.append(f"{path}: arity {len(a)} vs {len(b)}: {a!r} vs {b!r}")
        return out
    for i, (x, y) in enumerate(zip(a, b)):
        head = a[0] if isinstance(a[0], str) else '?'
        _diff(x, y, f"{path}/{head}[{i}]", out, limit, waived)
        if len(out) >= limit:
            break
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--show', type=int, default=12)
    args = ap.parse_args(argv)

    import pcbnew
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output

    tb = getattr(pcbnew, 'FLIP_DIRECTION_TOP_BOTTOM', False)
    tmp = tempfile.mkdtemp(prefix='krt714par')
    problems = []
    n_fix = n_pads = n_nonpad = 0
    waived = []
    surfaces = set()
    direction_checked = False
    try:
        for board, ref, delta in PASSES:
            src = os.path.join(REPO, 'kicad_files', board + '.kicad_pcb')
            if not os.path.isfile(src):
                problems.append(f"{board}: board not tracked -- fixture stale")
                continue

            # ---- path B: pcbnew's own flip
            b = pcbnew.LoadBoard(src)
            fp = b.FindFootprintByReference(ref)
            if fp is None:
                problems.append(f"{board}:{ref} not on the board -- fixture stale")
                continue
            before_y = [p.GetFPRelativePosition().y for p in fp.Pads()]
            before_x = [p.GetFPRelativePosition().x for p in fp.Pads()]
            fp.Flip(fp.GetPosition(), tb)
            if delta is not None:
                # COMPOSED: a flip AND a rotation the caller chose, which
                # is not pcbnew's bare flip and is the case every shipped
                # consumer actually asks for.
                fp.SetOrientationDegrees(
                    round((fp.GetOrientationDegrees() + delta) % 360, 6))
            if before_y and not direction_checked:
                # The enum is KiCad 10; on 9 the second arg is a bool. Prove
                # the direction TAKEN mirrors Y, so a KiCad that changes the
                # meaning fails loudly instead of comparing a different flip.
                after_y = [p.GetFPRelativePosition().y for p in fp.Pads()]
                after_x = [p.GetFPRelativePosition().x for p in fp.Pads()]
                if after_x != before_x or after_y != [-v for v in before_y]:
                    problems.append(
                        "the flip direction taken does NOT mirror Y "
                        f"(x moved={after_x != before_x}); refusing to compare "
                        f"against a different flip")
                direction_checked = True
            kout = os.path.join(tmp, f"{board}_{ref}_kicad.kicad_pcb")
            pcbnew.SaveBoard(kout, b)

            # ---- path A: our writer. pcbnew NEGATES the orientation, and this
            # writer deliberately does not (the caller owns `new_rotation`), so
            # hand it the negated angle to compare like with like.
            fp0 = parse_kicad_pcb(src).footprints[ref]
            side = 'F' if (fp0.layer or 'F').startswith('B') else 'B'
            ours = os.path.join(tmp, f"{board}_{ref}_ours.kicad_pcb")
            write_placed_output(src, ours, [{
                'reference': ref, 'new_x': round(fp0.x, 6),
                'new_y': round(fp0.y, 6),
                'new_rotation': round(((-(fp0.rotation or 0.0))
                                       + (delta or 0.0)) % 360, 6),
                'new_side': side}])

            ta, tk = _block(ours, ref), _block(kout, ref)
            if ta is None or tk is None:
                problems.append(f"{board}:{ref} block missing from an output")
                continue
            n_fix += 1
            counts = _count_nodes(tk)
            n_pads += counts.get('pad', 0)
            n_nonpad += sum(v for k, v in counts.items()
                            if k.startswith('fp_') or k in ('model', 'property'))
            surfaces |= {k for k in counts if k in REQUIRED_SURFACES}

            if _uuids(ta) != _uuids(tk):
                problems.append(f"{board}:{ref} uuid multiset changed -- group "
                                f"membership and #726 keying rest on these")

            d = _diff(canon(parse_sexpr(ta)), canon(parse_sexpr(tk)),
                      limit=args.show, waived=waived)
            if d:
                tag = 'pure flip' if delta is None else f'flip + {delta} deg'
                problems.append(
                    f"{board}:{ref} [{tag}] {len(d)} node difference(s):")
                problems.extend("      " + x for x in d[:args.show])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print(f"coverage: {n_fix} fixtures, {n_pads} pads, {n_nonpad} non-pad nodes, "
          f"surfaces={sorted(surfaces)}")
    if n_fix < MIN_FIXTURES:
        problems.append(f"only {n_fix} fixtures compared (floor {MIN_FIXTURES})")
    if n_pads < MIN_PADS:
        problems.append(f"only {n_pads} pads compared (floor {MIN_PADS})")
    if n_nonpad < MIN_NON_PAD_NODES:
        problems.append(f"only {n_nonpad} non-pad nodes compared "
                        f"(floor {MIN_NON_PAD_NODES})")
    missing = [s for s in REQUIRED_SURFACES if s not in surfaces]
    if missing:
        problems.append(f"no fixture carries {missing} -- the set stopped "
                        f"covering a surface it claims to")
    if not direction_checked:
        problems.append("the flip direction was never verified to mirror Y")
    if waived:
        print(f"waived {len(waived)} arc-mid nanometre difference(s) -- pcbnew "
              f"re-derives an arc's mid from centre/angles when it flips and "
              f"loses 1 nm; this transform emits the board's own value:")
        for w in waived:
            print("    " + w)
    if len(waived) > MAX_WAIVED:
        problems.append(f"{len(waived)} waived differences (cap {MAX_WAIVED}) "
                        f"-- the waiver is widening into a blanket tolerance")

    if problems:
        print(f"FAIL: {len(problems)} problem(s)")
        for p in problems[:60]:
            print("  " + p)
        return 1
    print("PASS: every fixture matches pcbnew's own flip, node for node")
    return 0


if __name__ == '__main__':
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(main())
