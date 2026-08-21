#!/usr/bin/env python3
"""A part buried inside another part's body is DISCLOSED, GATED and REPAIRED.

Run 22 shipped a board every gate called buildable while RN3 sat wholly inside
U5's body and RN7 inside U6's. Prevention landed first
(`reconstruct._pair_conflicts` refuses such a candidate pose); this file covers
the rest of the loop end to end on a real board.

**The fixture is the delicate part, and two obvious ones are vacuous.** A
containment planted anywhere that also produces a PAD INTERSECTION proves
nothing: `repair_placement` already charged pad conflicts, so `legalize` moves
the part for the other reason and the containment clears as a side effect --
measured, such a fixture passes identically with the containment census
stashed out. And a marker planted inside a body proves nothing if the marker
has no `.Fab` geometry (tigard's H1..H4 are all `fab_unjudged`), because no
pair is formed at all.

So the subject here is **D3 inside U5** -- `contained 1, gating 1,
blocking_pairs []`, a containment and nothing else, which is run 22's exact
shape. A/B against the stashed census:

    with census      Containment census: 1 part(s) ... -> 1 repaired, contained 0
    without census   (silent)                          -> 0 repaired, 0 unrepairable

The control is **TP1 at the same coordinates**: same board, same host, same
geometry, only the part class differs -- contained, but `marker_class`, so it
neither gates nor moves.

Run: python3 -X utf8 tests/test_run22_containment_repair.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
for _p in ('py_router', 'py_tools', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, _p))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault('KRT_NO_BANNER', '1')

from run_utils import tool as _tool                            # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement.legality import grade_body_overlap              # noqa: E402
from placement.writer import write_placed_output               # noqa: E402
from copy_board import SIBLING_EXTS                            # noqa: E402

SKIP_EXIT = 77
BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
MARKER_BOARD = os.path.join(ROOT, 'kicad_files',
                            'orangecrab_ext_pll.kicad_pcb')

# U5's fab body centre. D3 (0.8x1.6) sits wholly inside it at frac 1.0 with no
# pad of any other part within reach -- found by sweeping every (small, host)
# pair on the board for a containment with zero pad clash.
U5_CENTRE = (62.27, 60.90)

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def _run(script, *argv):
    return subprocess.run([sys.executable, '-X', 'utf8', _tool(script), *argv],
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, timeout=1800)


def _grade(path):
    return grade_body_overlap(parse_kicad_pcb(path), 0.15, pcb_file=path)


def _summary(stdout):
    for line in stdout.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            try:
                return json.loads(line.split(':', 1)[1])
            except Exception:
                return None
    return None


def plant_at(src, dst, ref, x, y, td):
    """Put `ref` at an EXACT board position, deterministically.

    The pose is ASSERTED through `placement.writer.write_placed_output` -- no
    search, no plan -- which is what makes this a fixture rather than a hope:
    a perturbation might or might not land inside a body, and a test that only
    sometimes builds its own subject is not a test. The siblings travel with
    the board (#441) so every later step grades at the floor it was staged at.
    Returns an object with `.returncode` (0 = planted), like the subprocess
    result the callers were written against.
    """
    from types import SimpleNamespace
    try:
        fp = parse_kicad_pcb(src).footprints[ref]
        ok = write_placed_output(src, dst, [
            {'reference': ref, 'new_x': float(x), 'new_y': float(y),
             'new_rotation': float(fp.rotation)}])
        for ext in SIBLING_EXTS:
            sib = os.path.splitext(src)[0] + ext
            if os.path.isfile(sib):
                shutil.copy2(sib, os.path.splitext(dst)[0] + ext)
    except Exception as exc:                                 # noqa: BLE001
        print(f'  plant_at({ref}) failed: {type(exc).__name__}: {exc}')
        return SimpleNamespace(returncode=1)
    return SimpleNamespace(returncode=0 if ok and os.path.exists(dst) else 1)


def pose_of(path, ref):
    fp = parse_kicad_pcb(path).footprints[ref]
    return (round(fp.x, 3), round(fp.y, 3))


def main():
    if not os.path.exists(BOARD):
        print('SKIP: tigard not present')
        return SKIP_EXIT
    td = tempfile.mkdtemp()
    try:
        base = os.path.join(td, 'base.kicad_pcb')
        if _run('copy_board.py', BOARD, base).returncode != 0:
            print('SKIP: could not stage the board')
            return SKIP_EXIT
        check('the fixture board starts with no containment',
              _grade(base)['contained'] == 0)

        print('a PURE containment (no pad conflict) is disclosed and GATES')
        dmg = os.path.join(td, 'contained.kicad_pcb')
        if plant_at(base, dmg, 'D3', U5_CENTRE[0], U5_CENTRE[1],
                    td).returncode != 0 or not os.path.exists(dmg):
            print('SKIP: could not plant the fixture')
            return SKIP_EXIT
        g = _grade(dmg)
        check('D3 is wholly inside U5', g['contained'] == 1,
              str([(q.a, q.b, q.contained_frac)
                   for q in g['containment_pairs']]))
        # THE anti-vacuity assertion. If this ever fails, the fixture has
        # drifted into a pad conflict and every `repaired` check below would
        # pass with the census removed.
        check('...and it is the ONLY thing wrong -- no pad conflict',
              g['blocking'] == 0 and not g['blocking_pairs'],
              str([(q.a, q.b, q.kind) for q in g['blocking_pairs']]))
        check('...so it gates on containment alone',
              g['containment_blocking'] == 1)

        asm = _run('check_assembly.py', dmg, '--clearance', '0.15')
        check('check_assembly refuses the board', asm.returncode == 4,
              f'exit {asm.returncode}')
        check('...and says why', 'CONTAINMENT' in asm.stdout, asm.stdout[-300:])

        print('legalize REPAIRS it -- the census is what makes this pass')
        out = os.path.join(td, 'fixed.kicad_pcb')
        r = _run('place_reconstruct.py', dmg, out,
                 '--stages', 'classify,legalize', '--clearance', '0.15')
        check('the containment census fires',
              'Containment census' in r.stdout, r.stdout[-300:])
        leg = (_summary(r.stdout) or {}).get('legalize') or {}
        check('legalize repairs D3', 'D3' in set(leg.get('repaired') or []),
              str(leg))
        if os.path.exists(out):
            after = _grade(out)
            check('...and the containment is GONE from the output',
                  after['containment_blocking'] == 0,
                  str([(q.a, q.b) for q in after['containment_blocking_pairs']]))
            check('...and check_assembly now accepts it',
                  _run('check_assembly.py', out,
                       '--clearance', '0.15').returncode == 0)
        else:
            check('the output board was written', False, r.stdout[-300:])

        print('the paired control: a MARKER at the SAME pose is exempt')
        # Only the part class differs from the case above. tigard's H1..H4 draw
        # no .Fab geometry and would form no pair at all, so the marker has to
        # be TP1 -- a testpoint WITH a body -- or this proves nothing.
        mk = os.path.join(td, 'marker.kicad_pcb')
        if plant_at(base, mk, 'TP1', U5_CENTRE[0], U5_CENTRE[1],
                    td).returncode == 0 and os.path.exists(mk):
            gm = _grade(mk)
            check('TP1 is equally contained', gm['contained'] == 1,
                  str([(q.a, q.b, q.contained_frac)
                       for q in gm['containment_pairs']]))
            check('...but does NOT gate', gm['containment_blocking'] == 0,
                  str([(q.a, q.b, q.waiver) for q in gm['containment_pairs']]))
            mo = os.path.join(td, 'marker_out.kicad_pcb')
            _run('place_reconstruct.py', mk, mo,
                 '--stages', 'classify,legalize', '--clearance', '0.15')
            if os.path.exists(mo):
                check('...and legalize does not move it',
                      pose_of(mo, 'TP1') == pose_of(mk, 'TP1'),
                      f'{pose_of(mk, "TP1")} -> {pose_of(mo, "TP1")}')
        else:
            check('the marker case could be planted', False)

        print('a containment it CANNOT reach is named, never faked')
        # R18 at U3's centre needs 9.15mm to come home and the cap ladder tops
        # out at 5.0mm on a board where every other part is at its healthy
        # pose. Unlike the case above this one ALSO carries a pad conflict, so
        # it is a contract check on `repair_placement`'s honesty re-grade
        # rather than a test of the census -- it passes with the census
        # stashed out, and is kept because reporting an unmoved part as
        # `repaired` is the failure this whole channel exists to prevent.
        u3 = parse_kicad_pcb(base).footprints['U3']
        deep = os.path.join(td, 'deep.kicad_pcb')
        if plant_at(base, deep, 'R18', u3.x, u3.y, td).returncode == 0 \
                and os.path.exists(deep):
            r = _run('place_reconstruct.py', deep,
                     os.path.join(td, 'deep_out.kicad_pcb'),
                     '--stages', 'classify,legalize', '--clearance', '0.15')
            leg = (_summary(r.stdout) or {}).get('legalize') or {}
            check('it is NOT reported repaired',
                  'R18' not in set(leg.get('repaired') or []), str(leg))
            check('...it is named unrepairable',
                  'R18' in set(leg.get('unrepairable') or []), str(leg))
            check('...and the run does not exit clean', r.returncode != 0,
                  f'exit {r.returncode}')

        print('the corpus case the exemption was written for')
        if os.path.exists(MARKER_BOARD):
            go = _grade(MARKER_BOARD)
            check('orangecrab_ext_pll ships 2 real containments',
                  go['contained'] == 2, str(go['contained']))
            check('...both marker_class, so none gates',
                  go['containment_blocking'] == 0,
                  str([(q.a, q.b, q.waiver) for q in go['containment_pairs']]))
        else:
            print('  (absent)')
    finally:
        shutil.rmtree(td, ignore_errors=True)

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
