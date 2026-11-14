"""#1842: the diff-pair via-span fixture, generated.

`diff_pair_base.kicad_pcb` is the 8-layer board with the pair `/D0_P` +
`/D0_N` (J1, J2), the full-height F.Cu wall W1 between them, and U9, the
bottom-side part whose pad field lies under everything. The field is the
bulk of the board (888 pads), so it is written here instead of being kept
in the repo: 37 x 24 GND pads 0.45 mm square on B.Cu, evenly spread from
(0.75, 0.75) to (29.25, 19.25) -- 0.34 mm apart, so no through via fits
between them. Deterministic: the same text every call.

    from diff_pair import write
    board = write(os.path.join(tmpdir, 'in.kicad_pcb'))   # + its .kicad_pro
"""
import os
import shutil
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, 'diff_pair_base.kicad_pcb')
COLS, ROWS = 37, 24
X0, X1, Y0, Y1 = 0.75, 29.25, 0.75, 19.25
SIZE = 0.45
_NS = uuid.UUID('6f0e4bd2-6c7a-4f5e-9d2c-1842d1ff0000')


def _num(v: float) -> str:
    return f"{round(v, 4):.4f}".rstrip('0').rstrip('.')


def pad_field() -> str:
    """U9's pads, numbered row by row from the top left."""
    out = []
    for j in range(ROWS):
        y = Y0 + j * (Y1 - Y0) / (ROWS - 1)
        for i in range(COLS):
            x = X0 + i * (X1 - X0) / (COLS - 1)
            n = j * COLS + i + 1
            out.append(f'\n\t\t(pad "{n}" smd rect\n\t\t\t(at {_num(x)} {_num(y)})\n'
                       f'\t\t\t(size {SIZE} {SIZE})\n\t\t\t(layers "B.Cu" "B.Mask")\n'
                       f'\t\t\t(net "GND")\n'
                       f'\t\t\t(uuid "{uuid.uuid5(_NS, f"U9.{n}")}")\n\t\t)')
    return ''.join(out)


def write(dest: str) -> str:
    """Write the board to ``dest`` and the base's project beside it (the
    board's DRC floor). Returns ``dest``."""
    with open(BASE, encoding='utf-8') as fh:
        text = fh.read()
    at = text.index('(footprint "fixture:U9"')
    at = text.index('(attr smd)', at) + len('(attr smd)')
    with open(dest, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text[:at] + pad_field() + text[at:])
    shutil.copyfile(os.path.splitext(BASE)[0] + '.kicad_pro',
                    os.path.splitext(dest)[0] + '.kicad_pro')
    return dest
