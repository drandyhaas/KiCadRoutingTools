#!/usr/bin/env python3
"""#1724: the seat keeps courtyards one copper clearance apart, a spacing of
the placer's own that KiCad does not ask for (KiCad's rule is that two
courtyards do not overlap). A refusal for that spacing is labelled
`courtyard_gap`, and only an overlap is labelled `courtyard`, so a reader
of a refusal at a placement KiCad accepts is not told it breaks KiCad's rule.

The spacing itself is unchanged: two routed A/Bs kept it over a gap of 0.

  1. two 0603s 0.1 mm apart at a 0.2 mm seat clearance: refused
     `courtyard_gap`, naming the neighbour;
  2. the same pair overlapping by 0.1 mm: refused `courtyard`;
  3. 0.3 mm apart: admitted (the spacing is unchanged);
  4. a through-hole pair 0.1 mm apart takes the same label on its own path;
  5. `rank_poses` carries the label in its in-place refusals.

    python3 tests/test_1724_courtyard_gap_label.py
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

RUN_ALL_TIMEOUT = 120
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _fp(ref, x, y, tht=False):
    pad = ('thru_hole circle\n\t\t\t(at {px} 0)\n\t\t\t(size 0.9 0.9)\n\t\t\t(drill 0.5)\n'
           '\t\t\t(layers "*.Cu" "*.Mask")' if tht else
           'smd rect\n\t\t\t(at {px} 0)\n\t\t\t(size 0.9 0.95)\n'
           '\t\t\t(layers "F.Cu" "F.Paste" "F.Mask")')
    s = (f'\t(footprint "Test:Two_Pin"\n\t\t(layer "F.Cu")\n\t\t(uuid "{uuid.uuid4()}")\n'
         f'\t\t(at {x} {y} 0)\n\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0 0)\n'
         f'\t\t\t(layer "F.SilkS")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t\t(effects\n'
         '\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
         '\t\t(fp_rect\n\t\t\t(start -1.5 -0.75)\n\t\t\t(end 1.5 0.75)\n'
         '\t\t\t(stroke\n\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n'
         f'\t\t\t(fill no)\n\t\t\t(layer "F.CrtYd")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    for n, px, net in (('1', -0.8, 1), ('2', 0.8, 2)):
        s += (f'\t\t(pad "{n}" ' + pad.format(px=px)
              + f'\n\t\t\t(net {net} "N{net}")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    return s + '\t)\n'


def write_board(tmp, name, gap, tht=False):
    """R1 at (10, 10); R2 east of it, its courtyard `gap` mm from R1's."""
    path = os.path.join(tmp, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
                '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n\t)\n'
                '\t(paper "A4")\n\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
                '\t\t(13 "F.Paste" user)\n\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n'
                '\t\t(5 "F.SilkS" user "F.Silkscreen")\n\t\t(25 "Edge.Cuts" user)\n'
                '\t\t(31 "F.CrtYd" user "F.Courtyard")\n\t\t(35 "F.Fab" user)\n\t)\n'
                '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n\t(net 0 "")\n'
                '\t(net 1 "N1")\n\t(net 2 "N2")\n'
                + _fp('R1', 10, 10, tht) + _fp('R2', 13 + gap, 10, tht)
                + '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 30 20)\n\t\t(stroke\n'
                  '\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n\t\t(fill no)\n'
                  f'\t\t(layer "Edge.Cuts")\n\t\t(uuid "{uuid.uuid4()}")\n\t)\n)\n')
    return path


def veto(path, diag=None):
    import pose_score
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, path)
    pcb.source_path = path
    st = quiet(pose_score.make_state, pcb, path, clearance=0.2,
               board_edge_clearance=0.3)
    p = st.parts['R2']
    if diag is not None:
        quiet(pose_score.rank_poses, pcb, path, 'R2', radius=0.0, step=0.5,
              rotations=(0.0,), state=st, diagnostics=diag)
    return st.candidate_veto('R2', p.x, p.y, p.rot)


def main():
    with tempfile.TemporaryDirectory(prefix='t1724_') as tmp:
        v = veto(write_board(tmp, 'apart', 0.1))
        check('0.1 mm apart at 0.2 mm: courtyard_gap vs R1',
              v == ('courtyard_gap', 'R1'), str(v))
        v = veto(write_board(tmp, 'overlap', -0.1))
        check('overlapping by 0.1 mm: courtyard vs R1',
              v == ('courtyard', 'R1'), str(v))
        v = veto(write_board(tmp, 'clear', 0.3))
        check('0.3 mm apart: admitted, the spacing unchanged', v is None, str(v))
        v = veto(write_board(tmp, 'tht', 0.1, tht=True))
        check('through-hole, 0.1 mm apart: courtyard_gap vs R1',
              v == ('courtyard_gap', 'R1'), str(v))
        diag = {}
        veto(write_board(tmp, 'rank', 0.1), diag)
        got = [d.get('check') for d in diag.get('dropped_in_place_by') or ()]
        check('rank_poses names it in its in-place refusals',
              got == ['courtyard_gap'], str(got))

    print(f"\n{'PASS' if not failures else 'FAIL'}: test_1724_courtyard_gap_label "
          f"({len(failures)} failure(s))")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
