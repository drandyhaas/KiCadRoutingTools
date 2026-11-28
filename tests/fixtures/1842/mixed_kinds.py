"""#1842: the mixed via-kind fixture, generated.

The 8-layer board of `diff_pair_base.kicad_pcb` (its header, stack, outline
and project), with these parts in place of that board's:

* J1, four F.Cu pads `/K1`..`/K4` at x = 7, inside W1 -- a closed frame of
  F.Cu copper -- and over U8, one B.Cu pad that covers the whole frame. No
  F.Cu track leaves the frame and no through via fits inside it;
* J2, the same four nets on B.Cu at x = 23 in the opposite order, inside W2
  -- the same frame on B.Cu -- and under U7, one F.Cu pad that covers it;
* one In3.Cu track at x = 6, y = 6..14, inside the left frame: a
  buried via In1.Cu-In6.Cu may not sit on it, a micro via F.Cu-In1.Cu may.

So a net leaves J1 only by a micro via F.Cu-In1.Cu, reaches J2 only by a
micro via In6.Cu-B.Cu, and joins In1.Cu to In6.Cu by a buried via: three
kinds of via, which no single via span has. Deterministic: the same text
every call.

    from mixed_kinds import write
    board = write(os.path.join(tmpdir, 'in.kicad_pcb'))   # + its .kicad_pro
"""
import os
import shutil
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, 'diff_pair_base.kicad_pcb')
NETS = ['/K1', '/K2', '/K3', '/K4']
LEFT_X, RIGHT_X = 7.0, 23.0
PAD_YS = [7.0, 9.0, 11.0, 13.0]
#: the frames' inner edges: x within 2.25 of the block's centre, y 4.75..15.25
FRAME_HALF_W, FRAME_Y0, FRAME_Y1, FRAME_T = 2.5, 4.5, 15.5, 0.5
IN3_TRACK = (6.0, 6.0, 6.0, 14.0)
_NS = uuid.UUID('6f0e4bd2-6c7a-4f5e-9d2c-1842d1ff0001')


def _num(v: float) -> str:
    return f"{round(v, 4):.4f}".rstrip('0').rstrip('.')


def _pad(ref, n, x, y, w, h, layer, net):
    mask = 'F.Mask' if layer == 'F.Cu' else 'B.Mask'
    layers = f'"{layer}" "{mask}"' if layer in ('F.Cu', 'B.Cu') else f'"{layer}"'
    return (f'\n\t\t(pad "{n}" smd rect\n\t\t\t(at {_num(x)} {_num(y)})\n'
            f'\t\t\t(size {_num(w)} {_num(h)})\n\t\t\t(layers {layers})\n'
            f'\t\t\t(net "{net}")\n'
            f'\t\t\t(uuid "{uuid.uuid5(_NS, f"{ref}.{n}")}")\n\t\t)')


def _footprint(ref, pads):
    return (f'\n\t(footprint "fixture:{ref}"\n\t\t(layer "F.Cu")\n'
            f'\t\t(uuid "{uuid.uuid5(_NS, ref)}")\n\t\t(at 0 0)\n'
            f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0 0)\n'
            f'\t\t\t(layer "F.SilkS")\n\t\t\t(hide yes)\n'
            f'\t\t\t(uuid "{uuid.uuid5(_NS, ref + ".ref")}")\n'
            f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n'
            f'\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            f'\t\t(attr smd){"".join(pads)}\n\t)')


def _frame(ref, cx, layer):
    """A closed rectangular frame of four pads round the block at ``cx``."""
    w, t = FRAME_HALF_W, FRAME_T
    cy, h = (FRAME_Y0 + FRAME_Y1) / 2, FRAME_Y1 - FRAME_Y0
    sides = [(cx - w, cy, t, h), (cx + w, cy, t, h),
             (cx, FRAME_Y0, 2 * w + t, t), (cx, FRAME_Y1, 2 * w + t, t)]
    return _footprint(ref, [_pad(ref, i + 1, x, y, sw, sh, layer, 'WALL_' + ref)
                            for i, (x, y, sw, sh) in enumerate(sides)])


def parts() -> str:
    cy, h = (FRAME_Y0 + FRAME_Y1) / 2, FRAME_Y1 - FRAME_Y0 + 1.0
    out = [
        _footprint('J1', [_pad('J1', i + 1, LEFT_X, y, 0.4, 0.4, 'F.Cu', n)
                          for i, (n, y) in enumerate(zip(NETS, PAD_YS))]),
        _footprint('J2', [_pad('J2', i + 1, RIGHT_X, y, 0.4, 0.4, 'B.Cu', n)
                          for i, (n, y) in enumerate(zip(NETS, reversed(PAD_YS)))]),
        _frame('W1', LEFT_X, 'F.Cu'),
        _frame('W2', RIGHT_X, 'B.Cu'),
        _footprint('U8', [_pad('U8', 1, LEFT_X, cy, 2 * FRAME_HALF_W + 1.0, h,
                               'B.Cu', 'GND_U8')]),
        _footprint('U7', [_pad('U7', 1, RIGHT_X, cy, 2 * FRAME_HALF_W + 1.0, h,
                               'F.Cu', 'GND_U7')]),
    ]
    x1, y1, x2, y2 = IN3_TRACK
    out.append(f'\n\t(segment\n\t\t(start {_num(x1)} {_num(y1)})\n'
               f'\t\t(end {_num(x2)} {_num(y2)})\n\t\t(width 0.2)\n'
               f'\t\t(layer "In3.Cu")\n\t\t(net "GND_U8")\n'
               f'\t\t(uuid "{uuid.uuid5(_NS, "in3")}")\n\t)')
    return ''.join(out)


def write(dest: str) -> str:
    """Write the board to ``dest`` and the base's project beside it (the
    board's DRC floor). Returns ``dest``."""
    with open(BASE, encoding='utf-8') as fh:
        text = fh.read()
    head = text[:text.index('\n\t(footprint "fixture:J1"')]
    tail = text[text.index('\n\t(gr_rect'):]
    with open(dest, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(head + parts() + tail)
    shutil.copyfile(os.path.splitext(BASE)[0] + '.kicad_pro',
                    os.path.splitext(dest)[0] + '.kicad_pro')
    return dest
