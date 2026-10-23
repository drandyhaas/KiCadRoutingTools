"""#1298: declared poses that overlap each other.

On glasgow_revC the poses compiled from mechanical.json put J5's courtyard
4.07 mm2 into MK2's (and each fiducial into a mounting hole's); on
CM5_MINIMA_3 the two module halves overlap by 14.58 mm2. `--emit-intent`
compiled them without a word, `fixed_poses[].accept_courtyard_overlap` was
read by the plan check only, so a placement seated exactly at those poses
graded NOT BUILDABLE on a pair no move can clear.

Now:
  * check_assembly --intent honours `accept_courtyard_overlap`: a listed
    pair's COURTYARD overlap is waived and listed as waived (label
    `fixed_pose_accepted`, `COURTYARD ACCEPTED BY A DECLARED POSE`, JSON
    `courtyard_fixed_pose_waived`) -- whatever the parts' lock -- and a
    drawn-body containment still gates: the waiver is courtyard only;
  * an unlisted pair still gates (the negative control);
  * check_floorplan --emit-intent writes `accept_courtyard_overlap` for
    declared poses whose courtyards overlap at those poses, with a WARNING
    naming the pairs, and names a pad conflict no waiver covers apart.

    python3 tests/test_1298_declared_pose_waivers.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _part(ref, x, y, *, pads=((-1.0, 0.6), (1.0, 0.6)), court=2.0,
          fab=1.5, locked=False, name='t:PART'):
    """An SMD part at (x, y): square pads (x offset, size) on nets N1/N2, a
    square courtyard of half-side `court` and a .Fab body of half-side
    `fab`."""
    out = (f'  (footprint "{name}" (layer "F.Cu") (at {x} {y})\n'
           + ('    (locked yes)\n' if locked else '')
           + f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
           f'    (fp_rect (start {-court} {-court}) (end {court} {court})'
           ' (stroke (width 0.05) (type default)) (layer "F.CrtYd"))\n'
           f'    (fp_rect (start {-fab} {-fab}) (end {fab} {fab})'
           ' (stroke (width 0.1) (type default)) (layer "F.Fab"))\n')
    for i, (px, size) in enumerate(pads, 1):
        out += (f'    (pad "{i}" smd rect (at {px} 0) (size {size} {size})'
                f' (layers "F.Cu") (net {i} "N{i}"))\n')
    return out + '  )\n'


def _board(path, parts):
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            '  (gr_rect (start 0 0) (end 80 40) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n'
            + ''.join(parts) + ')\n')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


#: Declared poses. A and B: courtyards overlap 0.8 mm deep (3.2 mm2), pads
#: 0.6 mm apart -- a courtyard-only pair. E and F: pads touching -- a pad
#: conflict no waiver covers. G and H: H's body wholly inside G's.
DECLARED = {'A': (10.0, 8.0), 'B': (13.2, 8.0),
            'E': (10.0, 20.0), 'F': (11.4, 20.0)}
#: Where the starting board has each part: on the board, apart.
PILE = {r: (10.0 + 8 * i, 34.0) for i, r in enumerate('ABCDEF')}
PILE.update(G=(62.0, 30.0), H=(72.0, 34.0))
BIG = dict(pads=((-3.5, 0.6), (3.5, 0.6)), court=4.5, fab=4.0)
SMALL = dict(pads=((-0.5, 0.4), (0.5, 0.4)), court=1.0, fab=0.5)


def _pile(path):
    """The run's starting board: every part on the board, apart, at a pose
    none is declared at (an off-board pile would gate on its pad copper)."""
    return _board(path, [_part(r, *PILE[r]) for r in 'ABCDEF']
                  + [_part('G', *PILE['G'], **BIG),
                     _part('H', *PILE['H'], **SMALL)])


def _seated(path, *, cd=False, gh=False, lock_ab=False):
    """A and B at their declared poses; C and D overlapping like A and B
    (moved, undeclared) when `cd`; H inside G's body (both declared there)
    when `gh`; everything else still in the pile."""
    parts = [_part('A', *DECLARED['A'], locked=lock_ab),
             _part('B', *DECLARED['B'], locked=lock_ab),
             _part('C', 25.0, 8.0) if cd else _part('C', *PILE['C']),
             _part('D', 28.2, 8.0) if cd else _part('D', *PILE['D']),
             _part('E', *PILE['E']), _part('F', *PILE['F'])]
    if gh:
        parts += [_part('G', 40, 20, **BIG), _part('H', 40, 20, **SMALL)]
    else:
        parts += [_part('G', *PILE['G'], **BIG),
                  _part('H', *PILE['H'], **SMALL)]
    return _board(path, parts)


def _intent(path, accept=(), waivers=()):
    """A minimal intent: A, B (and G, H) declared, `accept` as
    `[(ref, other)]` accept_courtyard_overlap entries."""
    rows = []
    for ref, (x, y) in sorted(DECLARED.items()):
        if ref in ('E', 'F'):
            continue
        row = {'ref': ref, 'x': x, 'y': y, 'rot': 0, 'basis': 'mechanical',
               'why': 'the enclosure fixes it'}
        acc = [o for r, o in accept if r == ref]
        if acc:
            row['accept_courtyard_overlap'] = acc
        rows.append(row)
    for ref in ('G', 'H'):
        row = {'ref': ref, 'x': 40, 'y': 20, 'rot': 0, 'basis': 'mechanical',
               'why': 'the enclosure fixes it'}
        acc = [o for r, o in accept if r == ref]
        if acc:
            row['accept_courtyard_overlap'] = acc
        rows.append(row)
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'board': 'b.kicad_pcb',
           'min_reader': 7, 'blocks': [], 'fixed_poses': rows}
    if waivers:
        doc['overlap_waivers'] = [{'pair': list(p), 'reason': 'test'}
                                  for p in waivers]
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=1)
    return path


def _assembly(td, board, intent, pile, tag):
    js = os.path.join(td, f'assembly_{tag}.json')
    argv = [sys.executable, '-X', 'utf8', tool('check_assembly.py'), board,
            '--intent', intent, '--baseline', pile, '--json', js]
    return argv, js


def test_check_assembly_honours_the_waiver(td):
    pile = _pile(os.path.join(td, 'pile.kicad_pcb'))
    seated = _seated(os.path.join(td, 'seated.kicad_pcb'))
    bare = _intent(os.path.join(td, 'bare.json'))
    acc = _intent(os.path.join(td, 'acc.json'), accept=[('A', 'B')])

    argv, js = _assembly(td, seated, bare, pile, 'bare')
    r = check(argv, refuse='COURTYARD BLOCKING', code=4)
    ok('unwaived: A<->B gates (both moved off the pile to their poses)',
       'A <-> B' in r.stdout and 'GATES (moved: A B)' in r.stdout,
       r.stdout[-600:])

    argv, js = _assembly(td, seated, acc, pile, 'acc')
    r = check(argv, accept=True)
    evidence(js, 'check_assembly --json')
    doc = json.load(open(js))
    waived = [(q['a'], q['b'], q['waiver'])
              for q in doc['courtyard_fixed_pose_waived']]
    ok('accepted: buildable, exit 0', doc.get('buildable') is True
       and r.returncode == 0, (doc.get('buildable'), r.returncode))
    ok('accepted: A<->B is listed as waived by the declared pose',
       waived == [('A', 'B', 'fixed_pose_accepted')], waived)
    ok('accepted: and said on the console',
       'COURTYARD ACCEPTED BY A DECLARED POSE (1)' in r.stdout,
       r.stdout[-600:])
    ok('accepted: no courtyard pair gates',
       doc.get('courtyard_blocking_gating') == 0,
       doc.get('courtyard_blocking_gating'))
    ok('a used waiver is not reported unused',
       doc.get('courtyard_waivers_unused') == [], doc.get(
           'courtyard_waivers_unused'))

    # The negative control: C and D overlap exactly as A and B do, moved,
    # and nobody declared them -- they gate with A<->B accepted.
    seated_cd = _seated(os.path.join(td, 'seated_cd.kicad_pcb'), cd=True)
    argv, js = _assembly(td, seated_cd, acc, pile, 'cd')
    r = check(argv, refuse='COURTYARD BLOCKING', code=4)
    doc = json.load(open(js))
    gating = [(q['a'], q['b']) for q in
              doc['courtyard_blocking_gating_pairs']]
    ok('negative control: the unlisted C<->D still gates, A<->B does not',
       gating == [('C', 'D')], gating)

    # A KiCad lock voids a CLASS waiver, never an authored one.
    seated_lk = _seated(os.path.join(td, 'seated_lk.kicad_pcb'), lock_ab=True)
    argv, js = _assembly(td, seated_lk, acc, pile, 'lk')
    r = check(argv, accept=True)
    ok('both declared parts locked: still waived (authored outranks lock)',
       json.load(open(js)).get('buildable') is True, r.stdout[-400:])


def test_courtyard_only(td):
    """The waiver is courtyard only: H wholly inside G's drawn body is a
    containment, and it still gates with G<->H accepted. An overlap_waivers
    pair -- which does exempt the body channel -- is the control."""
    pile = _pile(os.path.join(td, 'pile.kicad_pcb'))
    seated = _seated(os.path.join(td, 'seated_gh.kicad_pcb'), gh=True)
    acc = _intent(os.path.join(td, 'acc_gh.json'),
                  accept=[('A', 'B'), ('G', 'H')])
    argv, js = _assembly(td, seated, acc, pile, 'gh')
    r = check(argv, refuse='CONTAIN', code=4)
    doc = json.load(open(js))
    ok('G<->H: its courtyard pair is waived by the declared pose',
       ('G', 'H') in [(q['a'], q['b'])
                      for q in doc['courtyard_fixed_pose_waived']],
       doc['courtyard_fixed_pose_waived'])
    ok('... and H inside G\'s body still gates as a containment',
       doc.get('containment_blocking', 0) >= 1, doc.get(
           'containment_blocking'))
    wv = _intent(os.path.join(td, 'wv_gh.json'), accept=[('A', 'B')],
                 waivers=[('G', 'H')])
    argv, js = _assembly(td, seated, wv, pile, 'gh_wv')
    r = check(argv, accept=True)
    ok('control: as an overlap_waivers pair the containment is exempt',
       json.load(open(js)).get('buildable') is True, r.stdout[-400:])


def test_emit_intent_writes_the_waiver(td):
    pile = _pile(os.path.join(td, 'pile.kicad_pcb'))
    mech = os.path.join(td, 'mechanical.json')
    with open(mech, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'fixed': [
            {'ref': r, 'x': x, 'y': y, 'rot': 0, 'reason': 'enclosure'}
            for r, (x, y) in sorted(DECLARED.items())]}, fh)
    out = os.path.join(td, 'emitted.json')
    r = check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'),
               pile, '--emit-intent', out, '--mechanical', mech,
               '--allow-unplaced'], accept=True)
    evidence(out, 'the emitted intent')
    doc = json.load(open(out))
    acc = {f['ref']: f.get('accept_courtyard_overlap')
           for f in doc.get('fixed_poses') or ()}
    ok('the four declared poses are compiled', sorted(acc) == list('ABEF'),
       sorted(acc))
    ok('A accepts B (courtyards overlap at the declared poses)',
       acc.get('A') == ['B'], acc)
    ok('E accepts F too: their courtyards overlap as well', acc.get('E')
       == ['F'], acc)
    ok('the writer says so on the entry',
       '#1298' in str([f.get('note') for f in doc['fixed_poses']
                       if f['ref'] == 'A']))
    ctx = (doc.get('context') or {}).get('declared_pose_overlaps') or {}
    ok('context: both pairs accepted, E<->F also a hard (pad) conflict',
       ctx.get('accepted') == [['A', 'B'], ['E', 'F']]
       and ctx.get('hard') == [['E', 'F']], ctx)
    err = r.stderr
    ok('a WARNING names the overlapping pairs',
       'WARNING: 2 pair(s) of declared poses overlap each other' in err
       and 'A<->B' in err and 'E<->F' in err, err[-800:])
    ok('a WARNING names the pad conflict no waiver covers',
       'declared poses E and F also conflict past their courtyards' in err
       and 'pad' in err, err[-800:])
    # The emitted intent loads, and the plan check accepts A<->B now and
    # still refuses E<->F on its pads.
    plan = os.path.join(td, 'plan.json')
    r = check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'),
               pile, '--intent', out, '--plan-only', '--mechanical', mech,
               '--json', plan], refuse='plan_fixed_pose_conflict', code=4)
    lines = [ln for ln in r.stdout.splitlines()
             if ln.strip().startswith('[ERROR] plan_fixed_pose_conflict')]
    ok('plan check: only E<->F is still refused, on its pads',
       len(lines) == 1 and 'E and F' in lines[0] and 'pad' in lines[0],
       lines)


def main():
    with tempfile.TemporaryDirectory(prefix='t1298_') as td:
        test_check_assembly_honours_the_waiver(td)
        test_courtyard_only(td)
        test_emit_intent_writes_the_waiver(td)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
