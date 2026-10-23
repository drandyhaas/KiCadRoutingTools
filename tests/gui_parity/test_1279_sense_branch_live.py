#!/usr/bin/env python3
"""#1279 GUI gate: the live board keeps a `sense:` branch's copper and holds.

The GUI keeps ONE PCBData per dialog and re-reads the live board's copper
before every step (`RoutingDialog._sync_pcb_data_from_board`, the first line
of the Route tab's `_on_route`). That re-read used to build its own
Segment/Via with a subset of the fields: no uuid, no KiCad lock, no arc
pieces, and it never re-resolved `pcb_data.sense_branches`, which kept
pointing at the objects it had just thrown away. Measured on the fixture
below before the fix: after the sync the protection map was `{}` (the KiCad-
locked WALL net and the sense branch both rippable), the sense trace was a
routable terminal again, and a Route-tab run of SIG with VDD rippable put the
sense track in `segments_to_remove` -- the live board lost the sense trace,
KiCad dropped the emptied group on save, and VDD was left open at U1.1.

Arms (fixture = tests/fixtures/sense_branch/sense_star.kicad_pcb, built up
in-test):

- `sync_matches_a_fresh_build`: after the sync the tracks, arcs and vias carry
  what build_pcb_data_from_board reads (uuid, lock, layer, arc pieces, via
  type), copper graphics survive, the branches resolve against the CURRENT
  track objects, and the protection map equals a fresh build's -- a memo left
  by an earlier step included.
- `group_drawn_or_removed_in_session`: a `sense:` group created in the live
  session after the dialog opened counts at the next sync; removing it clears
  the branch and its sense pad's tag.
- `route_tab_keeps_the_branch`: the Route tab's call shape after the sync
  (SIG boxed in by VDD and a KiCad-locked wall, VDD named in
  rip_existing_nets) never strips the sense track.
- `placement_apply_keeps_the_group`: the Placement tab's Apply (Place +
  Route) keeps the live copper the result left unchanged -- uuid, arc and
  `sense:` group membership included -- replaces only what the run changed,
  and writes no lock token the result file does not carry.
- `sync_moves_footprint_graphic_copper`: a footprint whose land pattern draws
  F.Cu copper (#908: a filled tab and a stroke, the SOT89/antenna class) is
  moved and turned in the live session; after the sync the model's copper
  graphics equal a fresh build's. The sync used to keep the graphics built
  when the dialog opened, so the tab stayed where the part had been: a
  phantom obstacle there and nothing where the copper now is.

Needs pcbnew + wx; re-execs into KiCad's python. Lives in tests/gui_parity/
because run_all.py collects only tests/test_*.py.

    python3 -X utf8 tests/gui_parity/test_1279_sense_branch_live.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile
import traceback

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Newest KiCad first by NUMERIC version: a plain string sort puts KiCad\9.0
# above KiCad\10.0, so a box with both ran this under 9.0.
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]

FIXTURE = os.path.join(REPO, 'tests', 'fixtures', 'sense_branch', 'sense_star.kicad_pcb')
PRO = FIXTURE[:-len('.kicad_pcb')] + '.kicad_pro'
MEMBER = '11111111-0000-0000-0000-000000000001'
GROUP_LINE = ('  (group "sense:C1.1" (uuid "33333333-0000-0000-0000-000000000001") '
              '(members "11111111-0000-0000-0000-000000000001"))\n')
SEG_ANCHOR = '  (segment (start 3 5) (end 27 5)'

#: Built onto the fixture: SIG's two pads inside the loop VDD's routed
#: distribution copper closes with the sense trace, a KiCad-LOCKED WALL track
#: closing the loop's east side, a WALL arc (arc pieces + a second uuid), and a
#: net-less F.Cu copper graphic.
NETS_ANCHOR = '(net 0 "") (net 1 "VDD")'
EXTRA_NETS = NETS_ANCHOR + ' (net 2 "SIG") (net 3 "WALL")'
EXTRA = '''  (footprint "T:P" (layer "F.Cu") (at 10 6.6)
    (property "Reference" "J1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1))))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") (net 2 "SIG")))
  (footprint "T:P" (layer "F.Cu") (at 10 9.6)
    (property "Reference" "J2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1))))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") (net 2 "SIG")))
  (segment (start 3.2 5.4) (end 5.8 8) (width 0.3) (layer "F.Cu") (net 1) (uuid "aaaaaaaa-0000-0000-0000-000000000001"))
  (segment (start 5.8 8) (end 15 8) (width 0.3) (layer "F.Cu") (net 1) (uuid "aaaaaaaa-0000-0000-0000-000000000002"))
  (segment (start 15 8) (end 20 8) (width 0.3) (layer "F.Cu") (net 1) (uuid "aaaaaaaa-0000-0000-0000-000000000003"))
  (segment (start 21 5.6) (end 21 9.5) (width 0.2) (locked yes) (layer "F.Cu") (net 3) (uuid "99999999-0000-0000-0000-000000000001"))
  (arc (start 24 14) (mid 25 13) (end 26 14) (width 0.2) (layer "F.Cu") (net 3) (uuid "99999999-0000-0000-0000-000000000002"))
  (via (at 24 14) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 3) (uuid "99999999-0000-0000-0000-000000000003"))
  (gr_line (start 2 17) (end 8 17) (stroke (width 0.2) (type solid)) (layer "F.Cu") (uuid "99999999-0000-0000-0000-000000000004"))
'''

FAILURES = []


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + str(detail)[:600]) if detail else ''))
        FAILURES.append(what)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                # os.execv re-splits argv on spaces through the CRT on Windows.
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew + wx found")
    sys.exit(0)


def _board_text(group=True):
    with open(FIXTURE, encoding='utf-8') as f:
        src = f.read()
    assert src.count(SEG_ANCHOR) == 1 and src.count(GROUP_LINE) == 1 \
        and src.count(NETS_ANCHOR) == 1, 'the fixture moved'
    src = src.replace(NETS_ANCHOR, EXTRA_NETS, 1)
    src = src.replace(SEG_ANCHOR, EXTRA + SEG_ANCHOR, 1)
    return src if group else src.replace(GROUP_LINE, '')


def _stage(td, name, text):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    shutil.copy(PRO, path[:-len('.kicad_pcb')] + '.kicad_pro')
    return path


def _sync(pcbnew, board, pcb_data):
    """The dialog's own sync method on a stand-in `self` (the
    test_via_type_parse_parity pattern), against `board` as the live board."""
    from types import SimpleNamespace
    import kicad_routing_plugin.swig_gui as swig_gui
    real = pcbnew.GetBoard
    pcbnew.GetBoard = lambda: board
    try:
        swig_gui.RoutingDialog._sync_pcb_data_from_board(
            SimpleNamespace(pcb_data=pcb_data, _connectivity_cache={}))
    finally:
        pcbnew.GetBoard = real


def _seg_sig(pcb):
    return sorted((s.uuid, bool(s.locked), s.layer, s.net_id,
                   round(s.start_x, 6), round(s.start_y, 6),
                   round(s.end_x, 6), round(s.end_y, 6), round(s.width, 6))
                  for s in pcb.segments if not s.graphic)


def _via_sig(pcb):
    return sorted((v.uuid, bool(v.locked), tuple(v.layers), v.net_id, v.via_type,
                   round(v.x, 6), round(v.y, 6), round(v.size, 6), round(v.drill, 6),
                   tuple(sorted((v.tenting_attrs or {}).items())))
                  for v in pcb.vias)


def sync_matches_a_fresh_build(pcbnew, td):
    from kicad_parser import build_pcb_data_from_board
    from protected_nets import protection_map, cached_protection_map
    from sense_branch import branch_signature
    board = pcbnew.LoadBoard(_stage(td, 'sync', _board_text()))
    pd = build_pcb_data_from_board(board)        # what the dialog opened with
    pd._protection_map_memo = {'STALE': 'user'}  # an earlier step's memo
    _sync(pcbnew, board, pd)
    fresh = build_pcb_data_from_board(board)
    arcs = [s for s in fresh.segments if s.uuid.endswith('000000000002')]
    check(len(arcs) > 1 and any(s.locked for s in fresh.segments),
          'the fixture carries arc pieces and a KiCad-locked track',
          f'{len(arcs)} arc pieces')
    check(_seg_sig(pd) == _seg_sig(fresh),
          'after the sync every track and arc piece carries what a fresh build '
          'reads (uuid, lock, layer, geometry)',
          f'sync {_seg_sig(pd)[:4]} fresh {_seg_sig(fresh)[:4]}')
    check(_via_sig(pd) == _via_sig(fresh),
          'after the sync every via carries what a fresh build reads',
          f'sync {_via_sig(pd)} fresh {_via_sig(fresh)}')
    ng = lambda p: len([s for s in p.segments if s.graphic])  # noqa: E731
    check(ng(fresh) > 0 and ng(pd) == ng(fresh),
          'the sync keeps the copper graphics a fresh build carries',
          f'sync {ng(pd)} fresh {ng(fresh)}')
    sig = lambda p: sorted(map(branch_signature, p.sense_branches))  # noqa: E731
    check(sig(pd) == sig(fresh) and sig(pd) and sig(pd)[0][3] == '',
          'the branch re-resolves after the sync, protected', f'sync {sig(pd)} fresh {sig(fresh)}')
    ids = {id(s) for s in pd.segments}
    check(all(id(s) in ids for b in pd.sense_branches for s in b.segments),
          "the branch holds the sync's own track objects, not the replaced ones")
    member = [s for s in pd.segments if s.uuid == MEMBER]
    check(len(member) == 1 and member[0].sense == 'sense:C1.1',
          'the member track the router reads is tagged with its branch',
          [(s.uuid, s.sense) for s in member])
    want = protection_map(fresh)
    check('WALL' in want and protection_map(pd) == want
          and cached_protection_map(pd) == want,
          "the protection map (and its memo) is a fresh build's: KiCad-locked "
          "WALL and the branch's net stay protected",
          f'sync {protection_map(pd)} memo {cached_protection_map(pd)} fresh {want}')


def group_drawn_or_removed_in_session(pcbnew, td):
    from kicad_parser import build_pcb_data_from_board
    board = pcbnew.LoadBoard(_stage(td, 'session', _board_text(group=False)))
    pd = build_pcb_data_from_board(board)
    check(not pd.sense_branches, 'the board opens with no sense: group')
    track = next(t for t in board.GetTracks()
                 if t.GetClass() == 'PCB_TRACK' and str(t.m_Uuid.AsString()) == MEMBER)
    group = pcbnew.PCB_GROUP(board)
    group.SetName('sense:C1.1')
    group.AddItem(track)
    board.Add(group)
    _sync(pcbnew, board, pd)
    got = [(b.name, b.protected) for b in pd.sense_branches]
    check(got == [('sense:C1.1', True)],
          'a sense: group drawn in the session counts at the next sync', got)
    u1 = next(p for p in pd.footprints['U1'].pads if p.pad_number == '1')
    check(u1.sense == 'sense:C1.1', 'its sense pad is tagged', repr(u1.sense))
    group.RemoveAll()
    board.RemoveNative(group)
    _sync(pcbnew, board, pd)
    check(not pd.sense_branches and u1.sense == '',
          'removing the group in the session clears the branch and its pad tag',
          f'{[(b.name, b.protected) for b in pd.sense_branches]} {u1.sense!r}')


def route_tab_keeps_the_branch(pcbnew, td):
    from kicad_parser import build_pcb_data_from_board
    import route as _route
    path = _stage(td, 'route', _board_text())
    board = pcbnew.LoadBoard(path)
    pd = build_pcb_data_from_board(board)
    _sync(pcbnew, board, pd)                       # what _on_route does first
    _ok, _f, _t, data = _route.batch_route(
        path, '', ['SIG'], return_results=True, pcb_data=pd, layers=['F.Cu'],
        track_width=0.3, clearance=0.2, rip_existing_nets=['VDD'])
    stripped = [s for s in (data.get('segments_to_remove') or [])
                if (round(s.start_x, 3), round(s.start_y, 3),
                    round(s.end_x, 3), round(s.end_y, 3)) in ((3.0, 5.0, 27.0, 5.0),
                                                             (27.0, 5.0, 3.0, 5.0))]
    check(not stripped, 'the Route tab never strips the sense track after its sync',
          [(s.uuid, s.net_id) for s in stripped])
    locked_wall = [s for s in (data.get('segments_to_remove') or [])
                   if s.uuid == '99999999-0000-0000-0000-000000000001']
    check(not locked_wall, 'nor the KiCad-locked WALL track', locked_wall)


#: The result board of the Apply arm: one distribution segment rerouted (its
#: old block gone, two new ones) and one via added; everything else unchanged.
CHANGED_OLD = ('  (segment (start 15 8) (end 20 8) (width 0.3) (layer "F.Cu") (net 1) '
               '(uuid "aaaaaaaa-0000-0000-0000-000000000003"))\n')
CHANGED_NEW = ('  (segment (start 15 8) (end 17.5 9) (width 0.3) (layer "F.Cu") (net 1) '
               '(uuid "bbbbbbbb-0000-0000-0000-000000000001"))\n'
               '  (segment (start 17.5 9) (end 20 8) (width 0.3) (layer "F.Cu") (net 1) '
               '(uuid "bbbbbbbb-0000-0000-0000-000000000002"))\n'
               '  (via (at 26 14) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 3) '
               '(uuid "bbbbbbbb-0000-0000-0000-000000000003"))\n')
KEPT = ('aaaaaaaa-0000-0000-0000-000000000001', 'aaaaaaaa-0000-0000-0000-000000000002',
        '99999999-0000-0000-0000-000000000001', '99999999-0000-0000-0000-000000000002',
        '99999999-0000-0000-0000-000000000003', MEMBER)


def placement_apply_keeps_the_group(pcbnew, td):
    """Placement tab Apply (Place + Route) used to remove EVERY live track and
    recreate it from the result: new uuids, so the `sense:` group lost its
    member and KiCad dropped the emptied group on save (and, while the parser
    still marked members locked in memory, every one came back `(locked
    yes)`). Unchanged copper is kept now; only what the run changed is
    replaced."""
    import re
    from collections import Counter
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.placement_gui import PlacementTab
    text = _board_text()
    assert text.count(CHANGED_OLD) == 1, 'the Apply fixture moved'
    live_path = _stage(td, 'apply_live', text)
    result_path = _stage(td, 'apply_result', text.replace(CHANGED_OLD, CHANGED_NEW))
    result = parse_kicad_pcb(result_path)
    board = pcbnew.LoadBoard(live_path)
    added = PlacementTab._apply_copper(None, board, result, pcbnew)
    check(added == (2, 1), 'Apply adds only the copper the run changed', added)
    out = os.path.join(td, 'apply_out.kicad_pcb')
    pcbnew.SaveBoard(out, board)
    with open(out, encoding='utf-8') as f:
        saved = f.read()
    p = parse_kicad_pcb(out)
    locks = re.findall(r'\(locked(?:\s+yes)?\)', saved)
    locked = sorted({s.uuid for s in p.segments if s.locked}
                    | {v.uuid for v in p.vias if v.locked})
    check(len(locks) == 1 and locked == ['99999999-0000-0000-0000-000000000001'],
          'the saved board carries one lock token, on the WALL track the file '
          'locked; nothing the sense hold covers is locked', f'{len(locks)} token(s) on {locked}')
    uuids = Counter(s.uuid for s in p.segments if not s.graphic) + Counter(v.uuid for v in p.vias)
    check(all(uuids[u] for u in KEPT),
          'every unchanged track, arc and via keeps its uuid', sorted(u for u in KEPT if not uuids[u]))
    check(saved.count('(arc') == 1, 'the unchanged arc is still an arc', saved.count('(arc'))
    check(not uuids['aaaaaaaa-0000-0000-0000-000000000003'],
          "the segment the run replaced is gone from the live board")
    wall = [s for s in p.segments if s.uuid == '99999999-0000-0000-0000-000000000001']
    check(wall and all(s.locked for s in wall), 'the KiCad-locked WALL track is still locked')
    br = [(b.name, b.protected) for b in p.sense_branches]
    check(br == [('sense:C1.1', True)] and MEMBER in saved.split('(group', 1)[-1],
          'the sense: group survives Apply with its member, and still protects the branch', br)

    def copper(pcb):
        def k(s):
            a = (round(s.start_x, 4), round(s.start_y, 4))
            b = (round(s.end_x, 4), round(s.end_y, 4))
            return (min(a, b), max(a, b), round(s.width, 4), s.layer, pcb.nets[s.net_id].name)
        return (Counter(k(s) for s in pcb.segments if not s.graphic),
                Counter((round(v.x, 4), round(v.y, 4), round(v.size, 4), pcb.nets[v.net_id].name)
                        for v in pcb.vias))
    check(copper(p) == copper(result), "the live board's copper is the result's copper")


#: A part whose land pattern draws F.Cu copper (#908): one pad, so the copper
#: is modelled (footprint_copper_is_functional), a FILLED tab and a stroke.
TAB_PART = '''  (footprint "T:TAB" (layer "F.Cu") (at 14 16)
    (property "Reference" "Q1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1))))
    (pad "1" smd rect (at -1.5 0) (size 0.6 0.6) (layers "F.Cu" "F.Paste" "F.Mask") (net 3 "WALL"))
    (fp_rect (start -0.5 -0.8) (end 1.5 0.8) (stroke (width 0) (type solid)) (fill yes) (layer "F.Cu") (uuid "cccccccc-0000-0000-0000-000000000001"))
    (fp_line (start -1.5 0) (end -0.5 0) (stroke (width 0.25) (type solid)) (layer "F.Cu") (uuid "cccccccc-0000-0000-0000-000000000002")))
'''


def _graphic_sig(pcb):
    r = lambda v: None if v is None else round(v, 4)  # noqa: E731
    return sorted((s.owner_ref, s.layer, s.net_id, s.graphic_kind, bool(s.graphic_filled),
                   r(s.start_x), r(s.start_y), r(s.end_x), r(s.end_y), r(s.width),
                   r(s.drawn_width),
                   tuple(tuple(map(r, p)) for p in (s.graphic_ring or ())),
                   tuple(map(r, s.graphic_circle or ())))
                  for s in pcb.segments if s.graphic)


def sync_moves_footprint_graphic_copper(pcbnew, td):
    from kicad_parser import build_pcb_data_from_board, mm_to_iu
    text = _board_text().replace(SEG_ANCHOR, TAB_PART + SEG_ANCHOR, 1)
    board = pcbnew.LoadBoard(_stage(td, 'graphics', text))
    pd = build_pcb_data_from_board(board)              # what the dialog opened with
    before = _graphic_sig(pd)
    q1 = [g for g in before if g[0] == 'Q1']
    check(len(q1) >= 5 and any(g[4] for g in q1),
          "the fixture's Q1 draws modelled F.Cu copper, a filled tab included",
          f'{len(q1)} Q1 graphic segment(s)')
    fp = board.FindFootprintByReference('Q1')
    fp.SetPosition(pcbnew.VECTOR2I(mm_to_iu(18.0), mm_to_iu(15.5)))
    fp.SetOrientationDegrees(90)
    _sync(pcbnew, board, pd)
    fresh = _graphic_sig(build_pcb_data_from_board(board))
    check(fresh != before, 'the move changes what a fresh build reads (the arm is not vacuous)')
    check(_graphic_sig(pd) == fresh,
          "after the sync the model's copper graphics are a fresh build's: Q1's tab "
          "moved with Q1, the board-level graphic unchanged",
          f'sync {[g[:7] for g in _graphic_sig(pd) if g[0] == "Q1"][:2]} '
          f'fresh {[g[:7] for g in fresh if g[0] == "Q1"][:2]}')


ARMS = [sync_matches_a_fresh_build, group_drawn_or_removed_in_session,
        route_tab_keeps_the_branch, placement_apply_keeps_the_group,
        sync_moves_footprint_graphic_copper]


def main():
    try:
        import pcbnew  # noqa: F401
        import wx  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import pcbnew
    import wx
    for _p in ('', 'py_router', 'py_placer', 'py_tools'):
        _d = os.path.join(REPO, _p)
        if _d not in sys.path:
            sys.path.insert(0, _d)
    sys.path.insert(0, os.path.join(REPO, 'tests'))
    from run_utils import evidence
    evidence(FIXTURE, 'the sense_star fixture')
    evidence(PRO, "the sense_star fixture's .kicad_pro")
    print('KiCad build: %s' % pcbnew.GetBuildVersion())
    app = wx.App(False)  # noqa: F841  (before any wx object)
    td = tempfile.mkdtemp(prefix='t1279live_')
    try:
        for arm in ARMS:
            print('-- %s' % arm.__name__)
            try:
                arm(pcbnew, td)
            except Exception:  # noqa: BLE001
                traceback.print_exc()
                FAILURES.append('%s raised' % arm.__name__)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print('\n%d failure(s)' % len(FAILURES))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
