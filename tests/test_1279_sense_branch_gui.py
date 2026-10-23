#!/usr/bin/env python3
"""Copper a `sense:` group holds, or KiCad locks, is never moved or re-emitted (#1279).

A `sense:<REF>.<PAD>` group names its member tracks and vias by uuid, so any
pass that deletes a member and writes it back -- even unchanged -- takes it out
of the group: the next parse refuses the branch ("member(s) ... are not tracks
or vias on the board") and every later step routes as if nothing were declared.
A KiCad lock (#521) says the same copper is pinned. Both are
`kicad_parser.is_immutable_copper`. These are the passes that moved such copper
anyway, with no pcbnew needed (the live-board twins are
tests/gui_parity/test_1279_sense_branch_live.py):

- the #313 via nudge (place_fanout_clearance's last resort for a boxed cap):
  it moved a KiCad-locked or held via and the placement writer deleted it and
  re-emitted it under a new uuid. Now the nudge leaves it and the writer
  refuses a move that names one. Rig: tests/test_741_via_nudge_tenting.py's,
  whose cap C1 is boxed by via VF1 (nudged to (11.325, 9.5) there) and C2 by
  VF2 (the unlocked control).
- the castellated-landing retract (every routing step's post-pass,
  pcb_modification.retract_castellated_landings): it pulled a track end on a
  castellated pad back from the edge by stripping the track and re-emitting
  it. Rig: the sense_star fixture with U1.1 made a castellated pad at the
  right board edge and the member trace ending 0.15 mm from the edge.

Run:
    python3 tests/test_1279_sense_branch_gui.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
import traceback

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in (TESTS_DIR, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import test_741_via_nudge_tenting as R741  # noqa: E402  (the rig, not its tests)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from run_utils import evidence  # noqa: E402

_TMP = []

#: VF1's via, as the rig writes it, and a lock token placed in it.
VF1_UUID_LINE = '\t\t(uuid "%s")\n\t)' % R741.UUID_VF1
J1_TRACK = 'dddddddd-0000-0000-0000-000000000001'


def _tmpdir():
    d = tempfile.mkdtemp(prefix='t1279g_')
    _TMP.append(d)
    return d


def _rig_text(lock_vf1=False, hold_vf1=False):
    """The #741 rig; VF1 KiCad-locked, or made a member of a `sense:J1.1`
    branch (VF1 + a member track to J1.1, a pad on VF1's net)."""
    txt = R741.RIG_BOARD
    assert txt.count(VF1_UUID_LINE) == 1 and txt.rstrip().endswith(')'), 'the #741 rig moved'
    if lock_vf1:
        txt = txt.replace(VF1_UUID_LINE, '\t\t(locked yes)\n' + VF1_UUID_LINE)
    if hold_vf1:
        j1 = '\n'.join([
            '\t(footprint "T:P"', '\t\t(layer "F.Cu")',
            '\t\t(uuid "eeeeeeee-0000-0000-0000-0000000000a1")', '\t\t(at 13 9.5)',
            '\t\t(property "Reference" "J1" (at 0 -1 0) (layer "F.SilkS") '
            '(uuid "eeeeeeee-0000-0000-0000-0000000000a2") '
            '(effects (font (size 0.5 0.5) (thickness 0.1))))',
            R741._pad('1', 0, 0, 0.6, 0.6, 5, 'VF1'), '\t)'])
        seg = ('\t(segment\n\t\t(start %s 9.5)\n\t\t(end 13 9.5)\n\t\t(width 0.2)\n'
               '\t\t(layer "F.Cu")\n\t\t(net "VF1")\n\t\t(uuid "%s")\n\t)' % (R741.VIA_X, J1_TRACK))
        grp = ('\t(group "sense:J1.1"\n\t\t(uuid "ffffffff-0000-0000-0000-000000000001")\n'
               '\t\t(members "%s" "%s")\n\t)' % (R741.UUID_VF1, J1_TRACK))
        txt = txt.rstrip()[:-1] + j1 + '\n' + seg + '\n' + grp + '\n)'
    return txt


def _nudge(text):
    """Stage `text`, run the REAL repair (nudge on, as #741 drives it) and the
    REAL writer; (input pcb, result dict, output path, console)."""
    d = _tmpdir()
    src = os.path.join(d, 'rig.kicad_pcb')
    with io.open(src, 'w', encoding='utf-8', newline='') as f:
        f.write(text)
    evidence(src, 'the staged #741 rig')
    from placement.fanout_clearance import repair_fanout_clearance
    from placement.writer import write_placed_output
    pcb = parse_kicad_pcb(src)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        res = repair_fanout_clearance(
            pcb, src, clearance=R741.CLEAR, cap_prefix='C', max_displacement=0.0,
            max_displacement_cap=0.0, max_passes=1, allow_rotations=False,
            via_clear_fallback=False)
        dst = os.path.join(d, 'out.kicad_pcb')
        write_placed_output(src, dst, res['placements'], via_moves=res.get('via_moves'),
                            new_segments=res.get('new_segments'), pcb_data=pcb)
    evidence(dst, 'the written rig')
    return pcb, res, dst, buf.getvalue()


def _via(pcb, net_name):
    hits = [v for v in pcb.vias if pcb.nets[v.net_id].name == net_name]
    assert len(hits) == 1, f'{len(hits)} vias on {net_name}'
    return hits[0]


def _moved_nets(pcb, res):
    return sorted(pcb.nets[m[2]['net_id']].name for m in (res.get('via_moves') or []))


def test_nudge_never_moves_a_locked_via():
    pcb, res, dst, log = _nudge(_rig_text(lock_vf1=True))
    assert _via(pcb, 'VF1').locked, 'VF1 is not KiCad-locked in the input'
    assert _moved_nets(pcb, res) == ['VF2'], \
        f'moved {_moved_nets(pcb, res)}; the locked VF1 must stay and the control VF2 move'
    out = parse_kicad_pcb(dst)
    vf1 = _via(out, 'VF1')
    assert (vf1.x, vf1.y, vf1.uuid, vf1.locked) == (R741.VIA_X, 9.5, R741.UUID_VF1, True), \
        f'VF1 written as ({vf1.x}, {vf1.y}) uuid {vf1.uuid} locked {vf1.locked}'
    assert 'is KiCad-locked -- not moved' in log, log[-1500:]


def test_nudge_never_moves_a_held_via():
    pcb, res, dst, log = _nudge(_rig_text(hold_vf1=True))
    br = [(b.name, b.protected) for b in pcb.sense_branches]
    assert br == [('sense:J1.1', True)] and _via(pcb, 'VF1').held_by == 'sense:J1.1', br
    assert _moved_nets(pcb, res) == ['VF2'], \
        f'moved {_moved_nets(pcb, res)}; the held VF1 must stay and the control VF2 move'
    out = parse_kicad_pcb(dst)
    vf1 = _via(out, 'VF1')
    assert (vf1.x, vf1.y, vf1.uuid) == (R741.VIA_X, 9.5, R741.UUID_VF1), \
        f'VF1 written as ({vf1.x}, {vf1.y}) uuid {vf1.uuid}'
    br = [(b.name, b.protected, b.reason) for b in out.sense_branches]
    assert br == [('sense:J1.1', True, None)], f'the written branch: {br}'
    assert 'is held by sense:J1.1 -- not moved' in log, log[-1500:]


def test_writer_refuses_a_move_of_a_locked_via():
    d = _tmpdir()
    src = os.path.join(d, 'rig.kicad_pcb')
    with io.open(src, 'w', encoding='utf-8', newline='') as f:
        f.write(_rig_text(lock_vf1=True))
    pcb = parse_kicad_pcb(src)
    v = _via(pcb, 'VF1')
    move = (v.x, v.y, {'x': v.x + 0.15, 'y': v.y, 'size': v.size, 'drill': v.drill,
                       'layers': v.layers, 'net_id': v.net_id, 'tenting_attrs': {}})
    from placement.writer import write_placed_output
    dst = os.path.join(d, 'out.kicad_pcb')
    try:
        write_placed_output(src, dst, [], via_moves=[move], pcb_data=pcb)
    except ValueError as e:
        assert 'never re-emitted (#1279)' in str(e), str(e)
    else:
        raise AssertionError('the writer re-emitted a KiCad-locked via')
    assert not os.path.exists(dst), 'the refused write left an output file'


SENSE_BOARD = os.path.join(TESTS_DIR, 'fixtures', 'sense_branch', 'sense_star.kicad_pcb')
SENSE_PRO = SENSE_BOARD[:-len('.kicad_pcb')] + '.kicad_pro'
MEMBER = '11111111-0000-0000-0000-000000000001'
GROUP_LINE = ('  (group "sense:C1.1" (uuid "33333333-0000-0000-0000-000000000001") '
              '(members "11111111-0000-0000-0000-000000000001"))\n')
U1_AT = '(footprint "T:P" (layer "F.Cu") (at 27 5)'
U1_PAD = ('(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") '
          '(net 1 "VDD")))\n  (footprint "T:P" (layer "F.Cu") (at 15 8)')
MEMBER_HEAD = '(segment (start 3 5) (end 27 5) (width 0.2)'


def _castellated(group=True, lock=False):
    """sense_star with U1 a castellated pad straddling the right edge (x 29..30)
    and the member trace ending on it at x 29.85, 0.15 mm from the edge."""
    with open(SENSE_BOARD, encoding='utf-8') as f:
        src = f.read()
    assert all(src.count(a) == 1 for a in (U1_AT, U1_PAD, MEMBER_HEAD, GROUP_LINE)), \
        'the sense_star fixture moved'
    src = src.replace(U1_AT, '(footprint "T:P" (layer "F.Cu") (at 29.5 5)')
    src = src.replace(U1_PAD, U1_PAD.replace('(size 1 1) ', '(size 1 1) (property pad_prop_castellated) '))
    head = '(segment (start 3 5) (end 29.85 5) (width 0.2)' + (' (locked yes)' if lock else '')
    src = src.replace(MEMBER_HEAD, head)
    if not group:
        src = src.replace(GROUP_LINE, '')
    d = _tmpdir()
    path = os.path.join(d, 'castel.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(src)
    shutil.copy(SENSE_PRO, path[:-len('.kicad_pcb')] + '.kicad_pro')
    return path


def _retract(path):
    """The routing mains' post-pass on `path`; (count, the member as written)."""
    from pcb_modification import retract_castellated_landings
    with contextlib.redirect_stdout(io.StringIO()):
        n = retract_castellated_landings(path, 0.3)
    out = parse_kicad_pcb(path)
    vdd = [s for s in out.segments if not s.graphic and out.nets[s.net_id].name == 'VDD']
    assert len(vdd) == 1, f'{len(vdd)} VDD tracks after the retract'
    return n, vdd[0], out


def test_castellated_retract_moves_an_ordinary_landing():
    """The rig's control: the same landing, neither locked nor held, IS
    retracted -- so the two arms below test the guard, not a miss."""
    n, seg, _ = _retract(_castellated(group=False))
    assert n == 1 and seg.end_x < 29.85 - 1e-6, (n, seg.end_x)


def test_castellated_retract_never_rewrites_a_locked_track():
    path = _castellated(group=False, lock=True)
    assert parse_kicad_pcb(path).segments[0].locked, 'the member is not KiCad-locked'
    n, seg, _ = _retract(path)
    got = (n, seg.end_x, seg.uuid, seg.locked)
    assert got == (0, 29.85, MEMBER, True), got


def test_castellated_retract_never_rewrites_a_held_track():
    n, seg, out = _retract(_castellated())
    assert n == 0 and (seg.end_x, seg.uuid) == (29.85, MEMBER), (n, seg.end_x, seg.uuid)
    br = [(b.name, b.protected, b.reason) for b in out.sense_branches]
    assert br == [('sense:C1.1', True, None)], f'the written branch: {br}'


TESTS = [
    test_nudge_never_moves_a_locked_via,
    test_nudge_never_moves_a_held_via,
    test_writer_refuses_a_move_of_a_locked_via,
    test_castellated_retract_moves_an_ordinary_landing,
    test_castellated_retract_never_rewrites_a_locked_track,
    test_castellated_retract_never_rewrites_a_held_track,
]


def main():
    evidence(os.path.join(TESTS_DIR, 'test_741_via_nudge_tenting.py'), 'the #741 rig module')
    evidence(SENSE_BOARD, 'the sense_star fixture')
    evidence(SENSE_PRO, "the sense_star fixture's .kicad_pro")
    failed = []
    try:
        for t in TESTS:
            try:
                t()
                print(f'  PASS  {t.__name__}')
            except Exception:  # noqa: BLE001
                failed.append(t.__name__)
                print(f'  FAIL  {t.__name__}')
                traceback.print_exc()
    finally:
        for d in _TMP:
            shutil.rmtree(d, ignore_errors=True)
    print(f'{len(TESTS) - len(failed)}/{len(TESTS)} passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
