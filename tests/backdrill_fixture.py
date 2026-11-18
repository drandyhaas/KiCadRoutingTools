"""Synthetic four-layer boards for the #1925 back-drill tests.

`board_text` writes a 20 x 10 mm KiCad 10 board with a stackup (F.Cu, 0.2 mm,
In1.Cu, 1.0 mm, In2.Cu, 0.2 mm, B.Cu; copper 0.035 mm) from lists of pads,
tracks and vias, so each test states its own geometry in a few lines.
"""
import os

COPPER = [('F.Cu', 0, 0.2), ('In1.Cu', 4, 1.0), ('In2.Cu', 6, 0.2), ('B.Cu', 2, None)]


def _uuid(n):
    return f'00000000-0000-4000-8000-{n:012d}'


def backdrill_token(size, start, end, token='backdrill'):
    return f'\t\t({token}\n\t\t\t(size {size})\n\t\t\t(layers "{start}" "{end}")\n\t\t)\n'


def board_text(pads=(), tracks=(), vias=(), stackup=True):
    """pads: (ref, net, x, y, layer, size); tracks: (net, x1, y1, x2, y2,
    layer, width); vias: (net, x, y, size, drill, back-drill text)."""
    n = iter(range(1, 100_000))
    out = ['(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
           '\t(generator_version "10.0")\n'
           '\t(general\n\t\t(thickness 1.54)\n\t\t(legacy_teardrops no)\n\t)\n'
           '\t(paper "A4")\n\t(layers\n'
           + ''.join(f'\t\t({i} "{name}" signal)\n' for name, i, _d in COPPER)
           + '\t\t(5 "F.SilkS" user "F.Silkscreen")\n\t\t(1 "F.Mask" user)\n'
             '\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n\t)\n\t(setup\n']
    if stackup:
        out.append('\t\t(stackup\n')
        for k, (name, _i, diel) in enumerate(COPPER):
            out.append(f'\t\t\t(layer "{name}"\n\t\t\t\t(type "copper")\n'
                       f'\t\t\t\t(thickness 0.035)\n\t\t\t)\n')
            if diel is not None:
                out.append(f'\t\t\t(layer "dielectric {k + 1}"\n\t\t\t\t(type "core")\n'
                           f'\t\t\t\t(thickness {diel})\n\t\t\t\t(material "FR4")\n'
                           f'\t\t\t\t(epsilon_r 4.5)\n\t\t\t\t(loss_tangent 0.02)\n\t\t\t)\n')
        out.append('\t\t)\n')
    out.append('\t\t(pad_to_mask_clearance 0)\n\t)\n')
    for ref, net, x, y, layer, size in pads:
        mask = layer.replace('.Cu', '.Mask')
        out.append(f'\t(footprint "T:PAD"\n\t\t(layer "{layer}")\n\t\t(uuid "{_uuid(next(n))}")\n'
                   f'\t\t(at {x} {y})\n'
                   f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 -2 0)\n'
                   f'\t\t\t(layer "F.SilkS")\n\t\t\t(hide yes)\n'
                   f'\t\t\t(uuid "{_uuid(next(n))}")\n'
                   f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n'
                   f'\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
                   f'\t\t(attr smd)\n'
                   f'\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size {size} {size})\n'
                   f'\t\t\t(layers "{layer}" "{mask}")\n\t\t\t(net "{net}")\n'
                   f'\t\t\t(uuid "{_uuid(next(n))}")\n\t\t)\n\t)\n')
    for x1, y1, x2, y2 in ((0, 0, 20, 0), (20, 0, 20, 10), (20, 10, 0, 10), (0, 10, 0, 0)):
        out.append(f'\t(gr_line\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
                   f'\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
                   f'\t\t(layer "Edge.Cuts")\n\t\t(uuid "{_uuid(next(n))}")\n\t)\n')
    for net, x1, y1, x2, y2, layer, width in tracks:
        out.append(f'\t(segment\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
                   f'\t\t(width {width})\n\t\t(layer "{layer}")\n\t\t(net "{net}")\n'
                   f'\t\t(uuid "{_uuid(next(n))}")\n\t)\n')
    for net, x, y, size, drill, bd in vias:
        out.append(f'\t(via\n\t\t(at {x} {y})\n\t\t(size {size})\n\t\t(drill {drill})\n'
                   f'{bd}\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n'
                   f'\t\t(uuid "{_uuid(next(n))}")\n\t)\n')
    out.append('\t(embedded_fonts no)\n)\n')
    return ''.join(out)


def write_board(path, **kw):
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(board_text(**kw))
    assert os.path.getsize(path) > 0
    return path
