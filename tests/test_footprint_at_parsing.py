#!/usr/bin/env python3
"""A footprint's pose is its OWN `(at ...)`, in any number spelling KiCad writes.

Two parser defects, measured against KiCad's own reader (kicad-cli / pcbnew
10.0.0) before this fix:

  1. EXPONENT ANGLES. The pose regex spelled numbers `[\\d.-]+`, which cannot
     read `1e-05`. KiCad writes that form (pcbnew 10 `SaveBoard` wrote
     `(at 162.56 39.37 1e-14)`), and this repo's writer emits it through
     `:.6g` for any |angle| < 1e-4. The footprint's own `(at ...)` was skipped
     and the FIRST CHILD's -- a property's -- was read instead: C3 parsed at
     (5.08, 3.175, 270), or was dropped with its pads. A second write to such a
     board then rewrote the PROPERTY's `(at ...)` and left the part in place.
     The same spelling dropped a pad (pad angle) and a reference label.
  2. FIRST `(at` IN THE BLOCK. A child's `(at ...)` placed before the
     footprint's own was read as the footprint's pose, and written to.

A footprint with no `(at ...)` of its own sits at the origin, as KiCad places
it, rather than at a child's pose.
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _q = os.path.join(ROOT, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

SF = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not os.path.isfile(SF):
    print('SKIP: fixture missing')
    sys.exit(77)

import kicad_parser as K                                      # noqa: E402
from kicad_parser import parse_kicad_pcb, iter_footprint_blocks  # noqa: E402
from placement.writer import write_placed_output              # noqa: E402

_sink = io.StringIO()


def quiet(f, *a, **k):
    with contextlib.redirect_stdout(_sink), contextlib.redirect_stderr(_sink):
        return f(*a, **k)


with open(SF, encoding='utf-8') as fh:
    BASE = fh.read()
BASE_PCB = quiet(parse_kicad_pcb, SF)
C3 = BASE_PCB.footprints['C3']
C3_PADS = sorted((p.pad_number, round(p.global_x, 4), round(p.global_y, 4))
                 for p in C3.pads)


def block_of(text, key='C3'):
    return next(b for b in iter_footprint_blocks(text) if b[4] == key)


def with_block(text, new_block, key='C3'):
    s, e, _t, _r, _k = block_of(text, key)
    return text[:s] + new_block + text[e:]


def save(text, name='b.kicad_pcb'):
    p = os.path.join(tempfile.mkdtemp(prefix='fpat_'), name)
    with open(p, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    return p


def parse(path):
    err = io.StringIO()
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(err):
        pcb = parse_kicad_pcb(path)
    return pcb, err.getvalue()


def pose(fp):
    return (round(fp.x, 6), round(fp.y, 6), round(fp.rotation or 0.0, 9))


def own_at(text, key='C3'):
    b = block_of(text, key)[2]
    m = K.footprint_at_match(b)
    return None if m is None else m.group(0)


C3_BLOCK = block_of(BASE)[2]


# ==========================================================================
# 1. exponent-form numbers
# ==========================================================================
print('1. exponent angles')
for angle in ('1e-05', '-1e-17', '5E-05', '-9.999999975e-06'):
    blk = C3_BLOCK.replace('(at 144.78 31.75 -90)', f'(at 144.78 31.75 {angle})', 1)
    p = save(with_block(BASE, blk))
    pcb, _e = parse(p)
    fp = pcb.footprints.get('C3')
    check(f'footprint angle {angle}: the part is read at its own pose',
          fp is not None and (round(fp.x, 6), round(fp.y, 6)) == (144.78, 31.75)
          and abs(float(angle) - (fp.rotation or 0.0)) < 1e-9,
          str(fp and pose(fp)))
    check(f'footprint angle {angle}: ...with both pads',
          fp is not None and len(fp.pads) == 2, str(fp and len(fp.pads)))

# The writer itself emits exponent form for a tiny angle.
p1 = os.path.join(tempfile.mkdtemp(prefix='fpat_w_'), 'w.kicad_pcb')
quiet(write_placed_output, SF, p1, [{'reference': 'C3', 'new_x': 146.78,
                                     'new_y': 31.75, 'new_rotation': 1e-05}])
with open(p1, encoding='utf-8') as fh:
    w1 = fh.read()
check('fixture: the writer emitted an exponent-form angle', '1e-05)' in (own_at(w1) or ''),
      str(own_at(w1)))
pcb1, _e = parse(p1)
check('a board the writer wrote at 1e-05 reads back at its pose',
      pose(pcb1.footprints['C3'])[:2] == (146.78, 31.75), str(pose(pcb1.footprints['C3'])))

# A SECOND write to that board must move the footprint, not its property.
p2 = os.path.join(tempfile.mkdtemp(prefix='fpat_w2_'), 'w2.kicad_pcb')
quiet(write_placed_output, p1, p2, [{'reference': 'C3', 'new_x': 120.0,
                                     'new_y': 40.0, 'new_rotation': 90.0}])
with open(p2, encoding='utf-8') as fh:
    w2 = fh.read()
_ref = re.search(r'\(property\s+"Reference"\s+"C3"\s+\(at\s+(\S+)\s+(\S+)',
                 block_of(w2)[2])
check('a second write rewrites the footprint\'s own (at ...)',
      (own_at(w2) or '').startswith('(at 120.000000 40.000000'), str(own_at(w2)))
check('...and leaves the Reference property where it was, relative to the part',
      _ref is not None and (_ref.group(1), _ref.group(2)) == ('5.08', '3.175'),
      str(_ref and _ref.groups()))
pcb2, _e = parse(p2)
check('...and the part reads back where it was put',
      pose(pcb2.footprints['C3']) == (120.0, 40.0, 90.0), str(pose(pcb2.footprints['C3'])))

# A pad angle and a label angle in exponent form (pcbnew writes both as float
# noise, e.g. 2.842170943e-14, after a rotate-and-back in the GUI).
_pad = re.search(r'\(pad\s+"1"[^\n]*\n\s*\(at\s+(\S+)\s+(\S+)(?:\s+(\S+))?\)', C3_BLOCK)
blk = (C3_BLOCK[:_pad.start(0)]
       + C3_BLOCK[_pad.start(0):_pad.end(0)].replace(_pad.group(0).split('(at')[1],
                                                     f' {_pad.group(1)} {_pad.group(2)} 2.842170943e-14)', 1)
       + C3_BLOCK[_pad.end(0):])
blk = blk.replace('(at 5.08 3.175 180)', '(at 5.08 3.175 1.421085472e-14)', 1)
p3 = save(with_block(BASE, blk))
pcb3, _e = parse(p3)
fp3 = pcb3.footprints.get('C3')
check('fixture: the pad and the label carry exponent angles',
      '2.842170943e-14)' in blk and '1.421085472e-14)' in blk)
check('a pad whose angle is in exponent form is still a pad',
      fp3 is not None and len(fp3.pads) == 2, str(fp3 and len(fp3.pads)))
check('a Reference label whose angle is in exponent form is still read',
      fp3 is not None and fp3.ref_label is not None, str(fp3 and fp3.ref_label))

# Edge.Cuts a footprint carries follow the footprint's pose, exponent or not.
_edge = ('\t\t(fp_line (start -1 -1) (end 1 -1) (stroke (width 0.05) (type solid)) '
         '(layer "Edge.Cuts") (uuid "edge-t"))\n')
_ins = C3_BLOCK.index('(property "Reference"')
blk0 = C3_BLOCK[:_ins] + _edge.lstrip('\t') + '\t\t' + C3_BLOCK[_ins:]
blk_e = blk0.replace('(at 144.78 31.75 -90)', '(at 144.78 31.75 1e-09)', 1)
blk_z = blk0.replace('(at 144.78 31.75 -90)', '(at 144.78 31.75 0)', 1)
seg_e = K._collect_footprint_edge_segments_by_ref(with_block(BASE, blk_e)).get('C3')
seg_z = K._collect_footprint_edge_segments_by_ref(with_block(BASE, blk_z)).get('C3')


def _close(a, b):
    return (a is not None and b is not None and len(a) == len(b) and len(a) > 0
            and all(abs(x - y) < 1e-6 for sa, sb in zip(a, b)
                    for x, y in zip(_flat(sa), _flat(sb))))


def _flat(seg):
    out = []
    for v in (seg if isinstance(seg, (list, tuple)) else [seg]):
        if isinstance(v, (list, tuple)):
            out.extend(_flat(v))
        elif isinstance(v, (int, float)):
            out.append(float(v))
    return out


check('footprint Edge.Cuts land where the part is, at an exponent angle',
      _close(seg_e, seg_z), f'{seg_e} vs {seg_z}')


# ==========================================================================
# 2. the footprint's OWN (at ...), wherever the file lists it
# ==========================================================================
print('2. a child (at ...) first')
_decoy = '(property "Decoy" "" (at 1 2 45) (layer "F.Fab") (hide yes))\n\t\t'
_i = C3_BLOCK.index('(at 144.78 31.75 -90)')
p4 = save(with_block(BASE, C3_BLOCK[:_i] + _decoy + C3_BLOCK[_i:]))
pcb4, _e = parse(p4)
fp4 = pcb4.footprints['C3']
check('a property (at ...) before the footprint\'s own is not the pose',
      pose(fp4) == (144.78, 31.75, -90.0), str(pose(fp4)))
check('...and the pads land where they did',
      sorted((p.pad_number, round(p.global_x, 4), round(p.global_y, 4)) for p in fp4.pads)
      == C3_PADS)

p5 = os.path.join(tempfile.mkdtemp(prefix='fpat_w5_'), 'w5.kicad_pcb')
quiet(write_placed_output, p4, p5, [{'reference': 'C3', 'new_x': 120.0,
                                     'new_y': 40.0, 'new_rotation': -90.0}])
with open(p5, encoding='utf-8') as fh:
    w5 = fh.read()
check('the writer moves the footprint\'s own (at ...), not the decoy\'s',
      (own_at(w5) or '').startswith('(at 120.000000 40.000000')
      and '(property "Decoy" "" (at 1 2' in w5, str(own_at(w5)))

_descr = '(descr "trap (at 9 9 45)")\n\t\t'
p6 = save(with_block(BASE, C3_BLOCK[:_i] + _descr + C3_BLOCK[_i:]))
pcb6, _e = parse(p6)
check('an (at ...) inside a quoted string is not markup',
      pose(pcb6.footprints['C3']) == (144.78, 31.75, -90.0),
      str(pose(pcb6.footprints['C3'])))

p7 = save(with_block(BASE, C3_BLOCK.replace('(at 144.78 31.75 -90)\n\t\t', '', 1)))
pcb7, err7 = parse(p7)
fp7 = pcb7.footprints.get('C3')
check('fixture: C3 has no (at ...) of its own', own_at(open(p7, encoding='utf-8').read()) is None)
check('a footprint with no (at ...) of its own sits at the origin, as KiCad places it',
      fp7 is not None and pose(fp7) == (0.0, 0.0, 0.0), str(fp7 and pose(fp7)))
check('...and says so', 'C3 has no (at x y) of its own' in err7, err7[-200:])

check('a pose that does not parse is not invented',
      K.footprint_pose('(footprint "x" (at 1 two))') is None)

# ==========================================================================
# 3. nothing else moved
# ==========================================================================
print('3. unchanged on an ordinary board')
_same = all(K.footprint_at_match(b[2]).group(0)
            == re.search(r'\(at\s+[\d.-]+\s+[\d.-]+(?:\s+[\d.-]+)?\)', b[2]).group(0)
            for b in iter_footprint_blocks(BASE))
check('on a KiCad-written board the own (at ...) IS the first one, for every footprint',
      _same)

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
