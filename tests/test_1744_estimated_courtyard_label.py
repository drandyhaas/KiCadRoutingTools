#!/usr/bin/env python3
"""#1744: a seat refusal labelled `courtyard` reads as KiCad's
courtyards_overlap (#1724), which KiCad can only report between two parts
that BOTH draw a courtyard. A part that draws none is seated on a stand-in
-- its #1300 estimate (the occupancy check_assembly grades it on), else its
pad box -- so an overlap involving one is labelled `estimated_courtyard`.

The refusal itself is unchanged: check_assembly's courtyard channel grades
the same occupancy, and the placer may be stricter than the grader, never
looser.

  1. two parts that draw courtyards, overlapping: `courtyard`, as before;
  2. a courtyard-less part (a .Fab body) overlapping a drawn courtyard, in
     the placer's state (estimates on): `estimated_courtyard`, naming it;
  3. the same pair in a pose tool's state (estimates off, the pad box):
     `estimated_courtyard`;
  4. two courtyard-less parts: `estimated_courtyard`, from either side;
  5. a through-hole courtyard-less part, on its own path:
     `estimated_courtyard`;
  6. the courtyard-less pair apart but inside the spacing: `courtyard_gap`;
     clear of it: admitted -- the rule refuses what it refused;
  7. `rank_poses` carries the label in its in-place refusals.

On the base code cases 2-5 and 7 read `courtyard`.

    python3 tests/test_1744_estimated_courtyard_label.py
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


def _fp(ref, x, y, court=True, tht=False):
    """A two-pin part at (x, y): pads 1.6 mm apart, a 3.0 x 1.5 mm outline
    -- drawn on F.CrtYd when `court`, else on F.Fab (no courtyard, so the
    placer seats it on its estimate)."""
    pad = ('thru_hole circle\n\t\t\t(at {px} 0)\n\t\t\t(size 0.9 0.9)\n\t\t\t(drill 0.5)\n'
           '\t\t\t(layers "*.Cu" "*.Mask")' if tht else
           'smd rect\n\t\t\t(at {px} 0)\n\t\t\t(size 0.9 0.95)\n'
           '\t\t\t(layers "F.Cu" "F.Paste" "F.Mask")')
    layer = 'F.CrtYd' if court else 'F.Fab'
    s = (f'\t(footprint "Test:Two_Pin"\n\t\t(layer "F.Cu")\n\t\t(uuid "{uuid.uuid4()}")\n'
         f'\t\t(at {x} {y} 0)\n\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0 0)\n'
         f'\t\t\t(layer "F.SilkS")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t\t(effects\n'
         '\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
         '\t\t(fp_rect\n\t\t\t(start -1.5 -0.75)\n\t\t\t(end 1.5 0.75)\n'
         '\t\t\t(stroke\n\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n'
         f'\t\t\t(fill no)\n\t\t\t(layer "{layer}")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    for n, px, net in (('1', -0.8, 1), ('2', 0.8, 2)):
        s += (f'\t\t(pad "{n}" ' + pad.format(px=px)
              + f'\n\t\t\t(net {net} "N{net}")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    return s + '\t)\n'


def write_board(tmp, name, gap, r1_court=True, r2_court=False, tht=False):
    """R1 at (10, 10); R2 east of it, its outline `gap` mm from R1's."""
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
                + _fp('R1', 10, 10, r1_court, tht)
                + _fp('R2', 13 + gap, 10, r2_court, tht)
                + '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 30 20)\n\t\t(stroke\n'
                  '\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n\t\t(fill no)\n'
                  f'\t\t(layer "Edge.Cuts")\n\t\t(uuid "{uuid.uuid4()}")\n\t)\n)\n')
    return path


def state(path, estimate=True):
    import pose_score
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, path)
    pcb.source_path = path
    st = quiet(pose_score.make_state, pcb, path, clearance=0.2,
               board_edge_clearance=0.3, estimate_courtyards=estimate)
    return pcb, st


def veto(path, ref='R2', estimate=True):
    _pcb, st = state(path, estimate)
    p = st.parts[ref]
    return st.candidate_veto(ref, p.x, p.y, p.rot)


def main():
    with tempfile.TemporaryDirectory(prefix='t1744_') as tmp:
        v = veto(write_board(tmp, 'drawn', -0.1, r2_court=True))
        check('both draw a courtyard, overlapping 0.1 mm: courtyard vs R1',
              v == ('courtyard', 'R1'), str(v))
        est = write_board(tmp, 'est', -0.1)
        _pcb, st = state(est)
        check('R2 draws none and is seated on an estimate (#1300)',
              'R2' in st.courtyard_estimates and 'R1' not in st.courtyard_estimates,
              str(sorted(st.courtyard_estimates)))
        v = veto(est)
        check('R2 (estimate) onto R1 (drawn): estimated_courtyard vs R1',
              v == ('estimated_courtyard', 'R1'), str(v))
        v = veto(est, ref='R1')
        check('R1 (drawn) onto R2 (estimate): estimated_courtyard vs R2',
              v == ('estimated_courtyard', 'R2'), str(v))
        # 0.6 mm: the estimate is the 3.0 mm .Fab body, the pad box only
        # 2.5 mm, so the pad boxes overlap too
        v = veto(write_board(tmp, 'padbox', -0.6), estimate=False)
        check('estimates off (the pad box): estimated_courtyard vs R1',
              v == ('estimated_courtyard', 'R1'), str(v))
        both = write_board(tmp, 'both', -0.1, r1_court=False)
        v = veto(both)
        check('neither draws a courtyard: estimated_courtyard vs R1',
              v == ('estimated_courtyard', 'R1'), str(v))
        v = veto(write_board(tmp, 'tht', -0.1, tht=True))
        check('through-hole, its own path: estimated_courtyard vs R1',
              v == ('estimated_courtyard', 'R1'), str(v))
        v = veto(write_board(tmp, 'gap', 0.1))
        check('estimate 0.1 mm apart at 0.2 mm: courtyard_gap vs R1',
              v == ('courtyard_gap', 'R1'), str(v))
        v = veto(write_board(tmp, 'clear', 0.3))
        check('estimate 0.3 mm apart: admitted, the rule unchanged',
              v is None, str(v))
        import pose_score
        pcb, st = state(est)
        diag = {}
        quiet(pose_score.rank_poses, pcb, est, 'R2', radius=0.0, step=0.5,
              rotations=(0.0,), state=st, diagnostics=diag)
        got = [d.get('check') for d in diag.get('dropped_in_place_by') or ()]
        check('rank_poses names it in its in-place refusals',
              got == ['estimated_courtyard'], str(got))

    print(f"\n{'PASS' if not failures else 'FAIL'}: test_1744_estimated_courtyard_label "
          f"({len(failures)} failure(s))")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
