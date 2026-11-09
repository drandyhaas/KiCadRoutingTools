#!/usr/bin/env python3
"""#1723: a part that stands over a board cutout with its copper on the
board -- a reverse-mount LED over the window its light shines through, a
switch round the window under it -- is not refused `outline` at its own pose.

The board is written here: 40 x 30 mm with a 3.4 x 3.2 mm window cut out of
it (a board-level Edge.Cuts rectangle), an LED-shaped part whose two pad
columns stand either side of the window and whose courtyard spans it, and a
larger part whose courtyard swallows a second window clear of its pads.
KiCad grades no courtyard against Edge.Cuts; before the fix the seat refused
both parts at their own poses, because their courtyards cross the windows.

  1. both parts admitted where they stand;
  2. the window is still an edge to their COPPER: the LED moved so a pad
     column stands in the window is refused, by the pad copper's check;
  3. it is still an edge to every OTHER part: a part seated over the
     window, its copper clear, is refused `outline`;
  4. ownership is read at the part's seed pose, never at a candidate: the
     same LED read off the board (a pile) owns no window, and the seat
     refuses it over the window, as before;
  5. place_simple's own legality predicate (`seeder.pose_ok`) agrees with
     the seat at the own pose (both admit).

    python3 tests/test_1723_window_straddle.py
"""
import contextlib
import io
import os
import sys
import tempfile
import uuid

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))

RUN_ALL_TIMEOUT = 300
failures = []
NETS = ('GND', 'DIN', 'DOUT', 'VDD')
WIN = (20.0, 15.0, 1.7, 1.6)       # window centre and half sizes
WIN2 = (32.0, 15.0, 1.0, 1.0)


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _fp(name, ref, x, y, pads, crt):
    s = (f'\t(footprint "{name}"\n\t\t(layer "F.Cu")\n\t\t(uuid "{uuid.uuid4()}")\n'
         f'\t\t(at {x} {y} 0)\n\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0 0)\n'
         f'\t\t\t(layer "F.SilkS")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t\t(effects\n'
         '\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
         f'\t\t(fp_rect\n\t\t\t(start {-crt[0]} {-crt[1]})\n\t\t\t(end {crt[0]} {crt[1]})\n'
         '\t\t\t(stroke\n\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n'
         f'\t\t\t(fill no)\n\t\t\t(layer "F.CrtYd")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    for n, px, py, sx, sy, net in pads:
        s += (f'\t\t(pad "{n}" smd rect\n\t\t\t(at {px} {py})\n\t\t\t(size {sx} {sy})\n'
              f'\t\t\t(layers "F.Cu" "F.Paste" "F.Mask")\n'
              f'\t\t\t(net {NETS.index(net) + 1} "{net}")\n'
              f'\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    return s + '\t)\n'


def _rect(cx, cy, hx, hy):
    return (f'\t(gr_rect\n\t\t(start {cx - hx} {cy - hy})\n\t\t(end {cx + hx} {cy + hy})\n'
            '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n\t\t(fill no)\n'
            f'\t\t(layer "Edge.Cuts")\n\t\t(uuid "{uuid.uuid4()}")\n\t)\n')


LED_PADS = [('1', 2.69, 0.75, 1.72, 0.82, 'VDD'), ('2', 2.69, -0.75, 1.72, 0.82, 'DOUT'),
            ('3', -2.69, -0.75, 1.72, 0.82, 'GND'), ('4', -2.69, 0.75, 1.72, 0.82, 'DIN')]
SW_PADS = [('1', -3.0, 3.0, 1.5, 1.5, 'DIN'), ('2', 3.0, 3.0, 1.5, 1.5, 'GND')]


def write_board(tmp, name, led_at=(WIN[0], WIN[1])):
    parts = (_fp('Test:LED_ReverseMount', 'LED1', led_at[0], led_at[1], LED_PADS, (3.65, 1.25))
             + _fp('Test:Switch', 'SW1', WIN2[0], 13.0, SW_PADS, (4.5, 4.5))
             + _fp('Test:R_0603', 'R1', 10, 25, [('1', -0.8, 0, 0.9, 0.95, 'DIN'),
                                                  ('2', 0.8, 0, 0.9, 0.95, 'GND')], (1.5, 0.75)))
    nets = ''.join(f'\t(net {i + 1} "{n}")\n' for i, n in enumerate(NETS))
    path = os.path.join(tmp, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
                '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n\t)\n'
                '\t(paper "A4")\n\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
                '\t\t(13 "F.Paste" user)\n\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n'
                '\t\t(5 "F.SilkS" user "F.Silkscreen")\n\t\t(25 "Edge.Cuts" user)\n'
                '\t\t(31 "F.CrtYd" user "F.Courtyard")\n\t\t(35 "F.Fab" user)\n\t)\n'
                '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n\t(net 0 "")\n' + nets + parts
                + _rect(20, 15, 20, 15) + _rect(*WIN) + _rect(*WIN2) + ')\n')
    return path


def state(path):
    import pose_score
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, path)
    pcb.source_path = path
    return quiet(pose_score.make_state, pcb, path, clearance=0.2,
                 board_edge_clearance=0.2, estimate_courtyards=True)


def main():
    from placement import seeder
    with tempfile.TemporaryDirectory(prefix='t1723_') as tmp:
        st = state(write_board(tmp, 'placed'))
        check('the board has its two windows as cutouts',
              len(st.edge_gate.cutouts) == 2, str(len(st.edge_gate.cutouts)))
        led, sw = st.parts['LED1'], st.parts['SW1']

        print('1. at their own poses')
        for r, p in (('LED1', led), ('SW1', sw)):
            v = st.candidate_veto(r, p.x, p.y, p.rot)
            check(f'{r} admitted where it stands', v is None, str(v))

        print('2. the window is an edge to the copper')
        v = st.candidate_veto('LED1', led.x + 1.2, led.y, led.rot)
        check('LED1 moved 1.2 mm east, a pad column in the window: refused for its copper',
              v is not None and v[0] == 'pad_copper_outline', str(v))
        v = st.candidate_veto('LED1', led.x + 0.05, led.y, led.rot)
        check('LED1 nudged 0.05 mm, its copper still clear: admitted', v is None, str(v))

        print('3. an edge to every other part')
        v = st.candidate_veto('R1', WIN[0], WIN[1] + 3.0, 0.0)
        check('control: R1 clear of the window admitted', v is None, str(v))
        v = st.candidate_veto('R1', WIN2[0], WIN2[1] + 1.6, 0.0)
        check('R1 with its courtyard over the other part\'s window: refused outline',
              v is not None and v[0] == 'outline', str(v))

        print('4. ownership is read at the seed pose')
        st2 = state(write_board(tmp, 'pile', led_at=(20.0, 45.0)))
        check('LED1 read off the board owns no window',
              not st2._owned_rings('LED1'), str(st2._owned_rings('LED1')))
        # pose_ok, the absolute seat predicate: candidate_valid's escape
        # branch admits any pose merely nearer the board for a part off it
        check('so the seat refuses it over the window',
              not seeder.pose_ok(st2, 'LED1', WIN[0], WIN[1], 0.0, set()))

        print('5. pose_ok agrees')
        ok = seeder.pose_ok(st, 'LED1', led.x, led.y, led.rot, set())
        check('pose_ok admits LED1 at its own pose', ok)
        ok = seeder.pose_ok(st, 'LED1', led.x + 1.2, led.y, led.rot, set())
        check('pose_ok refuses LED1 with a pad column in the window', not ok)

    print(f"\n{'PASS' if not failures else 'FAIL'}: test_1723_window_straddle "
          f"({len(failures)} failure(s))")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
