"""A four-board placement-then-copper chain with a flip to the back side,
shared by the #1081 film tests (not a test itself).

On `kicad_files/rp2350_fpga_eensy_prePlane.kicad_pcb` (63 parts, 4 on B.Cu):

    S0  copper stripped, original poses
    S1  copper stripped, C26 (F.Cu) moved             -> a synth round, side F
    S2  copper stripped, C26 + U1 (B.Cu) moved        -> a synth round, side B
    F2  S2's poses WITH the copper                    -> a copper step after
                                                         the flip

`film()` runs the REAL path: `movie_camera.synth_rounds` -> `Stage` ->
`animate_route.build_boards`. It is the fixture #1082-#1086 were reproduced on.
"""
import os
import re
import shutil
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

ORIG = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')


def strip_copper(src, dst):
    """Drop every top-level (segment|via|arc ...) block, paren-balanced."""
    txt = open(src, encoding='utf-8').read()
    out, i, n = [], 0, len(txt)
    pat = re.compile(r'\n(\s*)\((segment|via|arc)[\s(]')
    while True:
        m = pat.search(txt, i)
        if not m:
            out.append(txt[i:])
            break
        out.append(txt[i:m.start()])
        j = m.start() + 1 + len(m.group(1))
        depth = 0
        while j < n:
            c = txt[j]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    j += 1
                    break
            elif c == '"':
                j += 1
                while j < n and txt[j] != '"':
                    j += 2 if txt[j] == '\\' else 1
            j += 1
        i = j
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(''.join(out))


class Chain(object):
    """`with Chain(rot_u1=90) as c: c.boards` -- a temp dir, removed on exit."""

    def __init__(self, rot_u1=0.0):
        self.rot_u1 = rot_u1
        self.dir = None
        self.boards = None

    def __enter__(self):
        import contextlib
        import io
        from kicad_parser import parse_kicad_pcb
        from placement.writer import write_placed_output
        self.dir = tempfile.mkdtemp(prefix='t1081_')
        p = parse_kicad_pcb(ORIG)
        c26, u1 = p.footprints['C26'], p.footprints['U1']
        mv1 = [dict(reference='C26', new_x=c26.x + 3.0, new_y=c26.y + 10.0,
                    new_rotation=c26.rotation or 0.0)]
        mv2 = mv1 + [dict(reference='U1', new_x=u1.x, new_y=u1.y + 10.0,
                          new_rotation=(u1.rotation or 0.0) + self.rot_u1)]
        j = lambda n: os.path.join(self.dir, n)                 # noqa: E731
        with contextlib.redirect_stdout(io.StringIO()):
            write_placed_output(ORIG, j('f1.kicad_pcb'), mv1)
            write_placed_output(ORIG, j('F2.kicad_pcb'), mv2)
        strip_copper(ORIG, j('S0.kicad_pcb'))
        strip_copper(j('f1.kicad_pcb'), j('S1.kicad_pcb'))
        strip_copper(j('F2.kicad_pcb'), j('S2.kicad_pcb'))
        self.boards = [j('S0.kicad_pcb'), j('S1.kicad_pcb'),
                       j('S2.kicad_pcb'), j('F2.kicad_pcb')]
        return self

    def __exit__(self, *exc):
        shutil.rmtree(self.dir, ignore_errors=True)
        return False


def rip_trace(board, path, n=240, rip=24):
    """A copper trace for `board` that ROUTES, RIPS and RE-ROUTES, written to
    `path`: `n` segments landed over three events, the first `rip` of them
    ripped (retracted) and grown back as a reroute (a growth stage with its
    finished self hidden under it), then the rest of the copper and the vias.
    """
    import json
    import animate_route as A
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board)
    layers = list(pcb.board_info.copper_layers)
    segs, vias = A._board_rows(pcb, layers)
    # small events, one layer each, so each face gets a SUSTAINED run of
    # work (the 3D side rule ignores copper that lands for under 4 s)
    def by_layer(rows, name):
        out = []
        for li in sorted({r[5] for r in rows}):
            mine = [r for r in rows if r[5] == li]
            for k in range(0, len(mine), 6):
                out.append({'event': 'route', 'net_name': '%s%d' % (name, k),
                            'add_s': mine[k:k + 6]})
        return out
    ev = by_layer(segs[:n], 'n')
    ev.append({'event': 'rip', 'net_name': 'n0', 'by': 'n9',
               'del_s': segs[:rip]})
    ev.append({'event': 'reroute', 'net_name': 'n0', 'add_s': segs[:rip]})
    ev += by_layer(segs[n:], 'rest')
    ev.append({'event': 'route', 'net_name': 'vias', 'add_v': vias})
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'layers': layers, 'events': ev}, f)
    return path


def film(boards, tween=4, size=320, stage_out=None,
         traces=None, board3d='2d'):
    """`(frames, movie, stage, geom)` from the real build_boards + Stage.
    `traces` maps a step index to a trace file for that step. The board is
    the 2D X-ray unless `board3d` says otherwise, so no Node/Chromium."""
    import animate_route as A
    import movie_camera as MC
    st = MC.Stage(MC.synth_rounds(boards), '', tween=tween, quiet=True)
    held = {}
    orig = MC.Stage.attach

    def _attach(self, movie, renderer, layers):
        held['movie'] = movie
        return orig(self, movie, renderer, layers)
    MC.Stage.attach = _attach
    geom = []
    try:
        steps = [('step %d' % i, b, (traces or {}).get(i))
                 for i, b in enumerate(boards)]
        frames = A.build_boards(steps, boards[-1], size, 1, None, 2, 6,
                                stage=st, geom_out=geom,
                                stage_out=stage_out, board3d=board3d)
    finally:
        MC.Stage.attach = orig
    return frames, held['movie'], st, (geom[0] if geom else None)
