#!/usr/bin/env python3
"""#1411: `bga_fanout --check-for-previous` calls a net fanned when a same-net
track END lands in the ball's copper, not only when it sits within 0.05 mm
of the ball's centre.

A route laid by another step may start anywhere inside the ball (0.15-0.2 mm
off-centre is common). The centre-window test called such a net unfanned,
re-escaped it, and dropped its existing copper from the obstacle grid as
"our own future nets", so other nets' escape vias landed on it.

Checks (on glasgow_revC U30.E8, a 0.35 mm ball, under-pad engine):
  1. The predicate: an end 0.12 mm off-centre is on the ball, one outside
     the ball's radius is not, an end within the old window still is.
  2. A same-net track starting 0.12 mm off-centre: with --check-for-previous
     the net is skipped (no copper added), as it is when the track starts
     exactly on the centre.
  3. Precondition: the same net with no track at all is escaped, so the
     skip above is the detection's doing.
  4. find_existing_fanouts (the channel path) names the off-centre net too.
  5. The skip is per BALL, not per net: on the array's many-ball ground net
     with a track on ONE ball (the first listed -- the pre-#1411 lookup kept
     only the last), --check-for-previous escapes every other ball and not
     that one; the bare board (control) escapes all of them. The per-ball
     answer: a ball-to-ball strap fans both balls, and a via in a ball fans
     it only when its net is fanned elsewhere.

    python3 tests/test_1411_fanout_detection_inside_ball.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from bga_fanout.geometry import endpoint_on_pad                # noqa: E402
from bga_fanout.reroute import find_existing_fanouts           # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
NET = '/IO_Banks/U5'                     # U30.E8, ball centre (84.5, 97.1), d 0.35
BALL = (84.5, 97.1)
ARGS = ['--component', 'U30', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
        '--layer-costs', '1.0', '4.0', '2.5', '1.0', '--track-width', '0.0762',
        '--clearance-ceiling', '0.0889', '--clearance', '0.0889',
        '--via-size', '0.25', '--via-drill', '0.15', '--nets', NET,
        '--escape-method', 'underpad']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def with_track(inp, path, a, b, net=NET):
    text = open(inp, encoding='utf-8').read()
    trk = (f'\t(segment\n\t\t(start {a[0]} {a[1]})\n\t\t(end {b[0]} {b[1]})\n'
           f'\t\t(width 0.0762)\n\t\t(layer "F.Cu")\n\t\t(net "{net}")\n'
           f'\t\t(uuid "14111411-0000-4000-8000-000000001411")\n\t)\n')
    i = text.rstrip().rfind(')')
    open(path, 'w', encoding='utf-8').write(text[:i] + trk + text[i:])
    shutil.copyfile(inp[:-len('.kicad_pcb')] + '.kicad_pro',
                    path[:-len('.kicad_pcb')] + '.kicad_pro')
    return path


def fan(inp, out, extra=(), args=ARGS, net=NET):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py',
                        inp, '-o', out] + args + list(extra), cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    line = next((l for l in r.stdout.splitlines() if 'JSON_SUMMARY' in l), '')
    js = json.loads(line.split('JSON_SUMMARY:', 1)[1]) if line else {}
    before = {(s.start_x, s.start_y, s.end_x, s.end_y, s.layer)
              for s in parse(inp).segments}
    after = parse(out)
    nid = next(i for i, n in after.nets.items() if n.name == net)
    added = [s for s in after.segments if s.net_id == nid
             and (s.start_x, s.start_y, s.end_x, s.end_y, s.layer) not in before]
    return r.returncode, js, added


def escaped_balls(balls, added):
    return {b.pad_number for b in balls for s in added
            if endpoint_on_pad(b, s.start_x, s.start_y, 0.05)
            or endpoint_on_pad(b, s.end_x, s.end_y, 0.05)}


def per_ball(inp, tmp):
    fp = parse(inp).footprints['U30']
    gnd = [p for p in fp.pads if p.net_name == 'GND']
    first = gnd[0]
    check('5. precondition: the ground net has many balls', len(gnd) >= 6,
          f'{len(gnd)} balls')
    args = list(ARGS)
    i = args.index('--nets')
    args[i + 1] = 'GND'
    args[args.index('--escape-method') + 1] = 'channel'
    trk = with_track(inp, os.path.join(tmp, 'gnd.kicad_pcb'),
                     (first.global_x, first.global_y),
                     (first.global_x, first.global_y - 1.5), net='GND')
    rc, _, added = fan(inp, os.path.join(tmp, 'gnd_bare.kicad_pcb'),
                       args=args, net='GND')
    bare = escaped_balls(gnd, added)
    check('5. control: the bare board escapes every ground ball',
          rc == 0 and bare == {p.pad_number for p in gnd},
          f'rc={rc} escaped={sorted(bare)}')
    rc, _, added = fan(trk, os.path.join(tmp, 'gnd_out.kicad_pcb'),
                       ['--check-for-previous'], args=args, net='GND')
    got = escaped_balls(gnd, added)
    check('5. the ball carrying the track is not re-escaped',
          rc == 0 and first.pad_number not in got, f'escaped={sorted(got)}')
    check('5. the net\'s other balls are still escaped (no net-wide skip)',
          len(got) >= len(gnd) - 2, f'{len(got)}/{len(gnd) - 1} others')

    # find_existing_fanouts' per-ball answer, on a synthetic array.
    def pad(n, x, net):
        return SimpleNamespace(pad_number=n, global_x=x, global_y=0.0,
                               size_x=0.35, size_y=0.35, shape='circle',
                               rect_rotation=0.0, net_id=net, net_name=f'N{net}')

    def seg(a, b, net):
        return SimpleNamespace(start_x=a, start_y=0.0, end_x=b, end_y=0.0,
                               net_id=net, layer='F.Cu')
    pads = [pad('A1', 0.0, 1), pad('A2', 0.8, 1), pad('A3', 1.6, 1),
            pad('A4', 2.4, 1), pad('B1', 4.0, 2), pad('B2', 4.8, 2)]
    vias = [SimpleNamespace(x=2.4, y=0.0, net_id=1),      # in A4, net 1 fanned
            SimpleNamespace(x=4.8, y=0.0, net_id=2)]      # in B2, net 2 not
    board = SimpleNamespace(segments=[seg(0.0, 0.8, 1)], vias=vias)
    nets, _, balls = find_existing_fanouts(
        board, SimpleNamespace(pads=pads), SimpleNamespace(pitch_x=0.8, pitch_y=0.8),
        [], return_balls=True)
    check('5. a ball-to-ball strap fans both balls, and only those',
          {(0.0, 0.0), (0.8, 0.0)} <= balls and (1.6, 0.0) not in balls,
          f'balls={sorted(balls)}')
    check('5. a via in a ball of a fanned net fans that ball',
          (2.4, 0.0) in balls)
    check('5. a via alone does not fan a ball of an unfanned net',
          (4.8, 0.0) not in balls and 2 not in nets, f'nets={sorted(nets)}')
    two = find_existing_fanouts(board, SimpleNamespace(pads=pads),
                                SimpleNamespace(pitch_x=0.8, pitch_y=0.8), [])
    check('5. the default call keeps its two-value answer',
          len(two) == 2 and two[0] == nets)


def main():
    ball = SimpleNamespace(global_x=BALL[0], global_y=BALL[1], size_x=0.35,
                           size_y=0.35, shape='circle', rect_rotation=0.0)
    check('1. an end 0.12 mm off-centre is on the ball',
          endpoint_on_pad(ball, BALL[0] + 0.12, BALL[1], 0.05))
    check('1. an end outside the ball is not',
          not endpoint_on_pad(ball, BALL[0] + 0.2, BALL[1], 0.05))
    check('1. a circle ball is a disc, not its bounding box',
          not endpoint_on_pad(ball, BALL[0] + 0.15, BALL[1] + 0.15, 0.05))
    check('1. an end inside the old centre window still is',
          endpoint_on_pad(ball, BALL[0] + 0.03, BALL[1] - 0.03, 0.05))

    tmp = tempfile.mkdtemp(prefix='t1411_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(BOARD, inp)
        off = with_track(inp, os.path.join(tmp, 'off.kicad_pcb'),
                         (BALL[0] + 0.12, BALL[1]), (BALL[0] + 0.12, BALL[1] - 1.0))
        ctr = with_track(inp, os.path.join(tmp, 'ctr.kicad_pcb'),
                         BALL, (BALL[0], BALL[1] - 1.0))

        rc, js, added = fan(ctr, os.path.join(tmp, 'ctr_out.kicad_pcb'),
                            ['--check-for-previous'])
        check('2. control: a centre-start track is skipped as fanned',
              rc == 0 and not added, f'rc={rc} added={len(added)}')
        rc, js, added = fan(off, os.path.join(tmp, 'off_out.kicad_pcb'),
                            ['--check-for-previous'])
        check('2. an off-centre-start track is skipped as fanned',
              rc == 0 and not added, f'rc={rc} added={len(added)}')
        rc, js, added = fan(inp, os.path.join(tmp, 'bare_out.kicad_pcb'))
        check('3. precondition: with no track at all the net is escaped',
              rc == 0 and added, f'rc={rc} added={len(added)}')

        pcb = parse(off)
        fp = pcb.footprints['U30']
        nid = next(i for i, n in pcb.nets.items() if n.name == NET)
        fanned, _ = find_existing_fanouts(pcb, fp, SimpleNamespace(
            pitch_x=0.5, pitch_y=0.5), [])
        check('4. find_existing_fanouts names the off-centre net',
              nid in fanned, f'fanned={sorted(fanned)}')

        per_ball(inp, tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'ALL OK')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
