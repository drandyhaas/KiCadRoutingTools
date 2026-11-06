#!/usr/bin/env python3
"""#1722: a shield frame drawn as its fence's pads round an F.Fab outline,
with no courtyard, is a FRAME the parts under it stand inside -- not the
solid body its outline draws.

The board is written here: a 20 x 20 mm fence (12 SMD pads on one net, a
.Fab outline, no courtyard, no model) on a 40 x 40 mm board, with a 0603 and
a small chip inside it. KiCad grades such a part on its courtyard alone, so
it grades nothing inside it. Before the fix the seat estimated a courtyard
from the outline and refused both inner parts at their own poses
(`courtyard` vs the fence), and check_assembly graded the board NOT
BUILDABLE on the same outline.

  1. the fence is a container ('fence'); the same pads on two nets, and one
     net's pads in two rows only (no wall on two sides), are not -- they are
     components;
  2. the seat (`pose_score.make_state` as place_simple builds it, courtyards
     estimated) admits both inner parts where they stand, and refuses them
     against the two-net control, so the check can fail;
  3. check_assembly grades the fence board buildable and the control NOT
     BUILDABLE (exit 4) for the containment, by its reason;
  4. a fence whose room is declared (`heights[]` `underside_mm`) is a cover
     the heights judge, not a container: a part too tall for the declared
     room (less the cover margin) is refused under it, and one that fits is
     not.

    python3 tests/test_1722_shield_frame.py
"""
import contextlib
import io
import os
import sys
import tempfile
import uuid

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))

import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 300
failures = []
NETS = ('GND', 'N1', 'N2')


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _fp(name, ref, x, y, pads, crt=None, fab=None):
    """A footprint; `pads` are (number, x, y, w, h, net name)."""
    s = (f'\t(footprint "{name}"\n\t\t(layer "F.Cu")\n\t\t(uuid "{uuid.uuid4()}")\n'
         f'\t\t(at {x} {y} 0)\n\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0 0)\n'
         f'\t\t\t(layer "F.SilkS")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t\t(effects\n'
         '\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n')
    for lay, h in (('F.CrtYd', crt), ('F.Fab', fab)):
        if h:
            s += (f'\t\t(fp_rect\n\t\t\t(start {-h[0]} {-h[1]})\n\t\t\t(end {h[0]} {h[1]})\n'
                  '\t\t\t(stroke\n\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n'
                  f'\t\t\t(fill no)\n\t\t\t(layer "{lay}")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    for n, px, py, sx, sy, net in pads:
        s += (f'\t\t(pad "{n}" smd rect\n\t\t\t(at {px} {py})\n\t\t\t(size {sx} {sy})\n'
              f'\t\t\t(layers "F.Cu" "F.Paste" "F.Mask")\n'
              f'\t\t\t(net {NETS.index(net) + 1} "{net}")\n'
              f'\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    return s + '\t)\n'


def write_board(tmp, name, fence='ring'):
    """40 x 40 mm, the frame SH at (20, 20) holding C1 and U1. `fence`:
    'ring' (every pad on GND), 'two_nets' (the east wall on N2), 'rows'
    (the north and south walls only)."""
    wall = [('1', a * 6, b * 9.6, 3, 0.8, 'GND') for a in (-1, 0, 1) for b in (-1, 1)]
    side = [('1', b * 9.6, a * 6, 0.8, 3, 'N2' if (fence == 'two_nets' and b > 0)
             else 'GND') for a in (-1, 0, 1) for b in (-1, 1)]
    pads = wall + ([] if fence == 'rows' else side)
    cap = [('1', -0.8, 0, 0.9, 0.95, 'N1'), ('2', 0.8, 0, 0.9, 0.95, 'GND')]
    chip = [(str(i + 1), -1.5 + i, 1.4, 0.4, 0.8, 'N1' if i % 2 else 'N2')
            for i in range(4)]
    parts = (_fp('Test:Shield_Frame_20x20mm', 'SH', 20, 20, pads, fab=(10, 10))
             + _fp('Test:C_0603', 'C1', 16, 16, cap, crt=(1.5, 0.75))
             + _fp('Test:Chip_4x4', 'U1', 21, 22, chip, crt=(2.2, 2.2)))
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
                + '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 40 40)\n\t\t(stroke\n\t\t\t(width 0.1)\n'
                  f'\t\t\t(type default)\n\t\t)\n\t\t(fill no)\n\t\t(layer "Edge.Cuts")\n'
                  f'\t\t(uuid "{uuid.uuid4()}")\n\t)\n)\n')
    return path


def parse(path):
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, path)
    pcb.source_path = path
    return pcb


def seat(path, declared=None):
    import pose_score
    return quiet(pose_score.make_state, parse(path), path, clearance=0.2,
                 board_edge_clearance=0.3, estimate_courtyards=True,
                 declared_heights=declared)


def vetoes(st, refs=('C1', 'U1')):
    return {r: st.candidate_veto(r, st.parts[r].x, st.parts[r].y, st.parts[r].rot)
            for r in refs}


def main():
    from placement import legality
    with tempfile.TemporaryDirectory(prefix='t1722_') as tmp:
        boards = {k: write_board(tmp, k, k) for k in ('ring', 'two_nets', 'rows')}

        print('1. which ring of pads is a fence')
        for k, want in (('ring', {'SH': 'fence'}), ('two_nets', {}), ('rows', {})):
            pcb = parse(boards[k])
            got = quiet(legality.container_kinds, pcb,
                        legality.part_local_bounds(pcb, boards[k]))
            check(f'{k}: containers {want}', got == want, str(got))

        print('2. the seat at the parts\' own poses')
        st = seat(boards['ring'])
        v = vetoes(st)
        check('the fence: both inner parts admitted where they stand',
              v == {'C1': None, 'U1': None}, str(v))
        check('the fence is named a container on the state',
              st.container_kinds.get('SH') == 'fence', str(st.container_kinds))
        v = vetoes(seat(boards['two_nets']))
        check('control, two nets: both refused against the outline',
              all(x is not None and x[1] == 'SH' for x in v.values()), str(v))

        print('3. check_assembly')
        cli = [sys.executable, os.path.join(REPO, 'py_tools', 'check_assembly.py')]
        run_utils.evidence(boards['ring'], 'the fence board')
        try:
            r = run_utils.check(cli + [boards['ring']], accept=True)
            check('the fence board is buildable', 'VERDICT: buildable' in r.stdout)
        except AssertionError as ex:
            check('the fence board is buildable', False, str(ex).splitlines()[0])
        r = run_utils.check(cli + [boards['two_nets']], refuse='NOT BUILDABLE', code=4)
        check('control: two nets, NOT BUILDABLE for the containment',
              'CONTAINMENT' in r.stdout and 'BLOCK' in r.stdout)

        print('4. a declared room makes the fence a cover')
        declared = {'covers': [], 'heights': [
            {'ref': 'SH', 'underside_mm': 2.5, 'source': 'test'},
            {'ref': 'C1', 'height_mm': 0.9, 'source': 'test'},
            {'ref': 'U1', 'height_mm': 2.4, 'source': 'test'}]}
        st = seat(boards['ring'], declared)
        check('not a container once its room is declared',
              'SH' not in st.container_refs and 'SH' in st.covers,
              f'{st.container_kinds} covers {sorted(st.covers)}')
        v = vetoes(st)
        check('C1 (0.9 mm) fits under 2.5 mm of room', v['C1'] is None, str(v))
        check('U1 (2.4 mm, past the cover margin) is refused under it, against the frame',
              v['U1'] is not None and v['U1'][1] == 'SH', str(v))

    print(f"\n{'PASS' if not failures else 'FAIL'}: test_1722_shield_frame "
          f"({len(failures)} failure(s))")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
