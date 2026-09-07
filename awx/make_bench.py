#!/usr/bin/env python3
"""make_bench.py -- a bench article for the K-bus chain from any board.

    python3 make_bench.py BOARD.kicad_pcb SRC DST OUT.kicad_pcb [--two-layer]
                          [--src-side F|B] [--dst-side F|B] [--rotate DEG]

The chain (`chain_k.sh`, BASE / DEST) needs three things a corpus board
does not carry: a SOURCE array already fanned out for the pair's nets
(the teeth the plan launches from), a project whose DRC floor is the one
the chain routes to, and the coherent K-ladder of its rivers. This
prepares all three the way the first bench (`fb_t2q_fresh`) was
prepared, so a second array pair is one command away:

1. `--two-layer`: the board's inner copper layers are removed from the
   layer table, with the zones on them -- the braid is a two-page
   router, and the corpus holds no 2-layer BGA-to-BGA DDR pair; a
   4-layer board with planes inside routes its signals on the outer
   layers anyway.
2. The pair's nets: every two-pad net between SRC and DST.
3. SRC is fanned out for them exactly as the chain fans out the
   destination (`fanout_from_plan.fanout_once`): the production engine,
   F/B, the braid's track / clearance / via, every foreign part
   immovable, no plane drop. A net the engine refuses is reported and
   left out of the ladder (it has no tooth to launch from).
4. The project floor: `fix_project_for_output` stamps the routed
   clearance and sizes into the sibling `.kicad_pro`, so `check_drc`
   and KiCad grade what was routed (a stock 0.2 mm class turned a clean
   0.1 mm fanout into 959 phantom violations); the article is then
   DRC-gated and the verdict printed.
5. The ladder, `<OUT stem>.ladder.txt`: the rivers the plan detects
   (`plan_state` -> detect_buses on taut paths), whole rivers only,
   largest first, singletons last -- a prefix K never splits a river.
   `coherent_nets.py` reads it beside the board.

Then: `BASE=OUT.kicad_pcb DEST=DST bash chain_k.sh TAG K ...`.

THE POSE GATE (`pose_gate.sh`): nothing in the chain may lean on the
board's axes or on which face an array sits. `--src-side` / `--dst-side`
put an array on the other face (the placement writer's mirror, the
#714 path), and every part its pads then collide with -- the decoupling
caps under a BGA sit on the far side -- goes to the other face with it,
until the article is pad-clean; `--rotate DEG` rotates the whole board
(rotate_board.py, an isometry that self-verifies). Copper already
attached to the pair's nets on the input (a fanned bench) is stripped
first, so the source is always fanned out in its final pose.
"""
import contextlib
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
from fix_kicad_drc_settings import fix_project_for_output  # noqa: E402
import braid as te  # noqa: E402
import fanout_from_plan as fp  # noqa: E402
sys.path.insert(0, os.path.join(HERE, '..', 'py_placer'))
from placement.writer import write_placed_output  # noqa: E402

PAD_PAIR = re.compile(r'Pad:\S+ \((\S+)\.\S+\) <-> Pad:\S+ \((\S+)\.\S+\)')


def two_layer(txt):
    """The board text with its inner copper layers gone: the layer-table
    lines and every zone on them (each zone block paren-balanced)."""
    txt, n_layers = re.subn(r'\n\s*\(\d+ "In\d+\.Cu" [^\n]*\)', '', txt)
    out, i, dropped = [], 0, 0
    zone = re.compile(r'\n\s*\(zone\s')
    while True:
        m = zone.search(txt, i)
        if not m:
            out.append(txt[i:])
            break
        j = m.start() + 1
        k, depth = j, 0
        while True:
            c = txt[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    k += 1
                    break
            k += 1
        block = txt[j:k]
        if re.search(r'\(layer "In\d+\.Cu"\)', block):
            out.append(txt[i:m.start()])
            dropped += 1
        else:
            out.append(txt[i:k])
        i = k
    return ''.join(out), n_layers, dropped


def pair_nets(pcb, src, dst):
    return sorted(n.name for n in pcb.nets.values()
                  if len(n.pads) == 2
                  and {p.component_ref for p in n.pads} == {src, dst})


def fanout_source(board, out, src, names):
    """fanout_from_plan.fanout_once's engine call, on the SOURCE array."""
    pcb = parse_kicad_pcb(board)
    pcb._fanout_all_foreign_immovable = True
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[src], pcb, net_filter=names, layers=list(fp.LAYERS),
        track_width=0.1, clearance=0.1, via_size=te.VIA_SIZE,
        via_drill=te.VIA_DRILL, exit_margin=0.5, escape_method='auto',
        plane_drop='off')
    if tracks:
        add_tracks_and_vias_to_pcb(
            board, out, tracks, vias_add, vias_rm,
            net_id_to_name={i: n.name for i, n in pcb.nets.items()})
    else:
        shutil.copy(board, out)
    fp.copy_pro(board, out)
    return len(tracks), len(vias_add), sorted(set(failed))


def strip_pair_copper(txt, pcb, names):
    """A fanned bench as input: the pair's segments and vias go, so the
    source is fanned out again in its final pose."""
    ids = {i for i, n in pcb.nets.items() if n.name in names}
    n0 = len(txt)
    txt = te.strip_net_items(txt, 'segment', ids, names)
    txt = te.strip_net_items(txt, 'via', ids, names)
    return txt, n0 - len(txt)


def side_of(fp_):
    return 'B' if fp_.layer.startswith('B') else 'F'


def flip_parts(board, refs, log=print):
    """Mirror `refs` to their other face in place (position and final
    angle kept), through the placement writer."""
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
        pl = [{'reference': r, 'new_x': pcb.footprints[r].x, 'new_y': pcb.footprints[r].y,
               'new_rotation': pcb.footprints[r].rotation,
               'new_side': 'F' if side_of(pcb.footprints[r]) == 'B' else 'B'}
              for r in refs]
        tmp = board + '.flip'
        write_placed_output(board, tmp, pl)
    os.replace(tmp, board)


def pad_partners(board, ref):
    """Other parts whose PADS collide with `ref`'s pads (check_drc)."""
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                        board, '--clearance', '0.1', '--clearance-margin', '0.1'],
                       capture_output=True, text=True)
    out = set()
    for a, b in PAD_PAIR.findall(r.stdout + r.stderr):
        if ref in (a, b):
            out.add(b if a == ref else a)
    return sorted(out)


def put_on_side(board, ref, side, log=print):
    """`ref` on face `side`; the parts its pads then collide with follow
    it to the other face (the caps under an array), until pad-clean."""
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    if side_of(pcb.footprints[ref]) == side:
        log(f'  {ref} already on {side}')
        return
    flip_parts(board, [ref])
    moved = [ref]
    for _round in range(4):
        hits = [r for r in pad_partners(board, ref) if r != ref]
        if not hits:
            break
        flip_parts(board, hits)
        moved += hits
    else:
        log(f'  {ref}: still colliding after 4 rounds: {hits}')
    log(f'  {ref} -> {side}: flipped {moved}')


def drc_verdict(board):
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                        board, '--clearance', '0.1', '--clearance-margin', '0.1'],
                       capture_output=True, text=True)
    out = r.stdout + r.stderr
    m = re.search(r'FOUND (\d+) DRC VIOLATIONS', out)
    return 0 if 'NO DRC VIOLATIONS' in out else (int(m.group(1)) if m else -1)


def write_ladder(board, names, log=print):
    """`<board stem>.ladder.txt`: the plan's rivers, largest first."""
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    bn = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    short = [n.split('/')[-1] for n in names]
    ok = []
    for n in short:
        try:
            te.endpoints(pcb, [n], bn)
            ok.append(n)
        except AssertionError as e:
            log(f'  {n}: no free stub at the source -- not in the ladder ({e})')
    st = fp.plan_state(pcb, ok, set())
    buses = sorted((list(b) for b in st['buses']), key=len, reverse=True)
    placed = {n for b in buses for n in b}
    single = [n for n in ok if n not in placed]
    out = os.path.splitext(board)[0] + '.ladder.txt'
    cps, tot = [], 0
    for b in buses:
        tot += len(b)
        cps.append(str(tot))
    with open(out, 'w', encoding='utf-8') as f:
        f.write(f'# Coherent K-ladder for {os.path.basename(board)}: whole rivers only\n')
        f.write('# (detect_buses on taut paths), largest first, singletons last.\n')
        f.write(f'# Checkpoints: {" ".join(cps)} | {tot + len(single)} (+singletons)\n')
        for b in buses:
            f.write(' '.join(b) + '\n')
        for n in single:
            f.write(n + '\n')
    log(f'  ladder {os.path.basename(out)}: {len(buses)} river(s) '
        f'{[len(b) for b in buses]}, {len(single)} singleton(s), {len(ok)} nets')
    return out


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board')
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('out')
    ap.add_argument('--two-layer', action='store_true')
    ap.add_argument('--src-side', choices=('F', 'B'))
    ap.add_argument('--dst-side', choices=('F', 'B'))
    ap.add_argument('--rotate', type=float)
    a = ap.parse_args(argv)
    board, src, dst, out = a.board, a.src, a.dst, a.out
    src_side, dst_side, rotate = a.src_side, a.dst_side, a.rotate
    flags = {'--two-layer'} if a.two_layer else set()
    if not out.endswith('.kicad_pcb'):
        out += '.kicad_pcb'
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    stem = out[:-len('.kicad_pcb')]
    base = stem + '_base.kicad_pcb'
    txt = open(board, encoding='utf-8').read()
    if '--two-layer' in flags:
        txt, n_l, n_z = two_layer(txt)
        print(f'two-layer: {n_l} inner layer(s) and {n_z} zone(s) on them removed')
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    for r in (src, dst):
        if r not in pcb.footprints:
            print(f'no footprint {r} on the board', file=sys.stderr)
            return 2
    names = pair_nets(pcb, src, dst)
    print(f'{len(names)} two-pad nets between {src} and {dst}')
    txt, n_cut = strip_pair_copper(txt, pcb, names)
    if n_cut:
        print(f'stripped the pair nets\' existing copper ({n_cut} chars)')
    with open(base, 'w', encoding='utf-8') as f:
        f.write(txt)
    fp.copy_pro(board, base)
    for ref, side in ((src, src_side), (dst, dst_side)):
        if side:
            if side not in ('F', 'B'):
                print(f'side must be F or B, not {side}', file=sys.stderr)
                return 2
            put_on_side(base, ref, side)
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(base)
    print(f'{os.path.basename(base)}: copper layers {pcb.board_info.copper_layers}, '
          f'{len(pcb.footprints)} parts; {src} on {side_of(pcb.footprints[src])}, '
          f'{dst} on {side_of(pcb.footprints[dst])}')
    with contextlib.redirect_stdout(sys.stderr):
        n_t, n_v, failed = fanout_source(base, out, src, names)
    print(f'{src} fanned out: {n_t} tracks, {n_v} vias'
          + (f', refused {failed}' if failed else ''))
    with contextlib.redirect_stdout(sys.stderr):
        fix_project_for_output(out, clearance=0.1, track_width=0.1,
                               via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL,
                               verbose=False)
    if rotate:
        # AFTER the fanout, copper included: the rotation is then an
        # isometry of the article the chain sees (plan + braid), and the
        # engine's own rotation sensitivity (bga_fanout's escape order
        # is not invariant -- measured: a fresh fanout on the rotated
        # board moved eight nets' vias at K15) is kept out of the gate
        rot = out[:-len('.kicad_pcb')] + '.rot.kicad_pcb'
        r = subprocess.run([sys.executable, os.path.join(HERE, 'rotate_board.py'),
                            out, rot, str(float(rotate))], capture_output=True, text=True)
        print('  ' + (r.stdout + r.stderr).strip().splitlines()[-1])
        if r.returncode:
            return 2
        fp.copy_pro(out, rot)
        os.replace(rot, out)
    n_drc = drc_verdict(out)
    print(f'{os.path.basename(out)}: {"DRC clean" if n_drc == 0 else f"{n_drc} DRC violation(s)"}'
          ' at the chain\'s floor (0.1)')
    write_ladder(out, names)
    print(f'run: BASE={out} DEST={dst} bash chain_k.sh TAG K ...')
    return 0 if n_drc == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
