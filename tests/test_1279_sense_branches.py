#!/usr/bin/env python3
"""A `sense:<REF>.<PAD>` group keeps new copper off a Kelvin sense branch (#1279).

Fixture tests/fixtures/sense_branch/sense_star.kicad_pcb (+ its .kicad_pro):
net VDD with join pad C1.1 at (3, 5), sense pad U1.1 at (27, 5) joined by one
0.2 mm F.Cu sense trace (uuid 11111111-...), two load pads U2.1 (15, 8) /
U3.1 (20, 8) just below the trace, and the declaration

    (group "sense:C1.1" (uuid ...) (members "11111111-0000-0000-0000-000000000001"))

The control is the same board with the group line removed.

Invariants:
(a) new copper attaches at the join pad only: the pads reached from the sense
    trace without crossing C1.1 are U1.1, U2.1, U3.1 on the control and only
    U1.1 with the group; the written board re-parses with the branch still
    protected.
(b) no clearance descent: JSON_SUMMARY.design_rules narrows nothing, and the
    output .kicad_pro Default clearance and min_clearance equal the control's.
(c) no net id is rewritten: the member track's block and the group are in the
    written board byte for byte, on VDD.
(d) a plane chain (pour on B.Cu, then the route step with its in-run plane
    finalize, and repair_planes with pad repair and blocker rips) keeps the
    member copper byte for byte and the branch protected.
(e) every refusal names its reason, and route.py discloses it in
    JSON_SUMMARY.sense_branches.
(f) the GUI call shape (an in-memory PCBData, return_results=True) keeps the
    same rule and leaves every net id as it was.
(g) the member copper is HELD (`held_by`), never KiCad-locked: the run's own
    routes stay rippable, a whole-net strip of pre-existing copper is refused
    naming the group, a refused branch keeps its copper, and a branch the run
    taps is disclosed (`tapped_by_run`). Parse-path parity with
    pcbnew is tests/test_1279_sense_branches_pcbnew.py.

Run:
    python3 tests/test_1279_sense_branches.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import traceback

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import sense_branch as sb  # noqa: E402
from kicad_parser import parse_kicad_pcb, Zone  # noqa: E402
from run_utils import tool, evidence  # noqa: E402

#: Seven route/plane subprocesses on a 30 x 20 mm board, ~5 s each here.
RUN_ALL_TIMEOUT = 900

BOARD = os.path.join(TESTS_DIR, 'fixtures', 'sense_branch', 'sense_star.kicad_pcb')
PRO = BOARD[:-len('.kicad_pcb')] + '.kicad_pro'
MEMBER = '11111111-0000-0000-0000-000000000001'
GROUP_LINE = ('  (group "sense:C1.1" (uuid "33333333-0000-0000-0000-000000000001") '
              '(members "11111111-0000-0000-0000-000000000001"))\n')
GEOM = ['--layers', 'F.Cu', 'B.Cu', '--track-width', '0.3', '--clearance', '0.2']

_TMP = []


def _tmpdir():
    d = tempfile.mkdtemp(prefix='t1279_')
    _TMP.append(d)
    return d


def _src():
    with open(BOARD, encoding='utf-8') as f:
        return f.read()


def _variant(text, name='v'):
    """`text` written as a board in a fresh dir, with the fixture's .kicad_pro."""
    d = _tmpdir()
    path = os.path.join(d, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    shutil.copy(PRO, os.path.join(d, f'{name}.kicad_pro'))
    return path


def _control():
    src = _src()
    assert src.count(GROUP_LINE) == 1, 'fixture group line moved'
    return _variant(src.replace(GROUP_LINE, ''), 'control')


def _member_block(text):
    """The member track's `(segment ...)` block as written."""
    m = re.search(r'\(segment\s[^\n]*?' + re.escape(MEMBER) + r'"\)\)', text)
    assert m, 'member track block not found'
    return m.group(0)


def _run(script, args, timeout=600):
    r = subprocess.run([sys.executable, tool(script)] + args, cwd=ROOT,
                       capture_output=True, text=True, timeout=timeout)
    assert r.returncode == 0, f'{script} exit {r.returncode}:\n' + r.stdout[-3000:] + r.stderr[-3000:]
    return r.stdout


def _route(board, extra=GEOM):
    """route.py VDD to a FRESH output path; (output board, console)."""
    out = os.path.join(_tmpdir(), 'out.kicad_pcb')
    log = _run('route.py', [board, out, '--nets', 'VDD'] + list(extra))
    evidence(out, 'route.py output')
    return out, log


def _summary(log):
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', log, re.M)
    assert m, 'route.py printed no JSON_SUMMARY'
    return json.loads(m.group(1))


def _pro_clearances(board):
    with open(board[:-len('.kicad_pcb')] + '.kicad_pro', encoding='utf-8') as f:
        doc = json.load(f)
    default = next(c for c in doc['net_settings']['classes'] if c['name'] == 'Default')
    return default['clearance'], doc['board']['design_settings']['rules']['min_clearance']


def _tapped(board):
    """Pads of VDD reached from the member trace without crossing C1.1."""
    p = parse_kicad_pcb(board)
    vdd = next(i for i, n in p.nets.items() if n.name == 'VDD')
    join = next(q for q in p.footprints['C1'].pads if q.pad_number == '1')
    items = ([('seg', s) for s in p.segments if s.net_id == vdd]
             + [('via', v) for v in p.vias if v.net_id == vdd]
             + [('pad', q) for q in p.pads_by_net[vdd] if q is not join])
    seen = {i for i, (k, it) in enumerate(items) if k == 'seg' and it.uuid == MEMBER}
    assert seen, 'member trace missing'
    todo = list(seen)
    while todo:
        i = todo.pop()
        for j, other in enumerate(items):
            if j not in seen and sb._touch(items[i], other, join):
                seen.add(j)
                todo.append(j)
    return sorted(f'{it.component_ref}.{it.pad_number}' for i, (k, it) in enumerate(items)
                  if i in seen and k == 'pad')


def _grade(board, clearance='0.2'):
    c = subprocess.run([sys.executable, tool('check_connected.py'), board, '--nets', 'VDD'],
                       cwd=ROOT, capture_output=True, text=True)
    assert 'ALL NETS FULLY CONNECTED' in c.stdout, c.stdout[-2000:] + c.stderr[-1000:]
    d = subprocess.run([sys.executable, tool('check_drc.py'), board, '--clearance', clearance],
                       cwd=ROOT, capture_output=True, text=True)
    assert 'NO DRC VIOLATIONS FOUND' in d.stdout, d.stdout[-2000:] + d.stderr[-1000:]


def _branch(board_or_pcb):
    p = parse_kicad_pcb(board_or_pcb) if isinstance(board_or_pcb, str) else board_or_pcb
    assert len(p.sense_branches) == 1, p.sense_branches
    return p.sense_branches[0]


# --- (a) (b) (c): the main route -------------------------------------------

_MAIN = {}


def _main_runs():
    if not _MAIN:
        _MAIN['ctrl'] = _route(_control())
        _MAIN['sense'] = _route(BOARD)
    return _MAIN['ctrl'], _MAIN['sense']


def test_parse_resolves_the_group():
    b = _branch(BOARD)
    assert b.protected, b.reason
    assert (b.join_label(), len(b.segments), len(b.vias)) == ('C1.1', 1, 0)
    assert [f'{q.component_ref}.{q.pad_number}' for q in b.sense_pads] == ['U1.1']
    seg = b.segments[0]
    assert seg.sense == 'sense:C1.1' and seg.held_by == 'sense:C1.1', (seg.sense, seg.held_by)
    assert not seg.locked, 'the hold is not a KiCad lock'
    assert parse_kicad_pcb(_control()).sense_branches == []


def test_new_copper_joins_at_the_join_pad_only():
    (ctrl, _), (out, log) = _main_runs()
    assert _tapped(ctrl) == ['U1.1', 'U2.1', 'U3.1'], \
        f'control: the router no longer taps the sense trace ({_tapped(ctrl)}); the fixture proves nothing'
    assert _tapped(out) == ['U1.1'], f'a load reached the sense copper off C1.1: {_tapped(out)}'
    assert 'sense:C1.1: protected 1 track(s)' in log, log[-2000:]
    b = _branch(out)
    assert b.protected, f'the written board taps its own branch: {b.reason}'
    _grade(out)


def test_no_clearance_descent():
    (ctrl, ctrl_log), (out, log) = _main_runs()
    s, c = _summary(log), _summary(ctrl_log)
    dr = s.get('design_rules') or {}
    assert not dr.get('narrowed') and not dr.get('count'), dr
    assert not dr.get('fab_tier_escalations'), dr
    assert s['min_clearance_used'] == c['min_clearance_used'] == 0.2, \
        (s['min_clearance_used'], c['min_clearance_used'])
    assert _pro_clearances(out) == _pro_clearances(ctrl) == (0.2, 0.2), \
        (_pro_clearances(out), _pro_clearances(ctrl))
    assert 'OVERLAP a foreign' not in log, 'the join pad was blocked by its own branch'


def test_no_net_id_is_rewritten():
    _, (out, _log) = _main_runs()
    src = _src()
    with open(out, encoding='utf-8') as f:
        text = f.read()
    assert _member_block(src) in text, 'the member track was rewritten'
    assert text.count(MEMBER) == 2, 'member uuid not exactly once in its block and once in the group'
    assert GROUP_LINE.strip() in text, 'the group declaration was not carried'
    assert '~sense' not in text and '(locked' not in text, 'a private net or a lock token reached the file'
    p = parse_kicad_pcb(out)
    seg = next(s for s in p.segments if s.uuid == MEMBER)
    assert p.nets[seg.net_id].name == 'VDD'
    assert sorted(n.name for n in p.nets.values()) == sorted(
        n.name for n in parse_kicad_pcb(BOARD).nets.values()), 'the net table changed'


# --- (d): plane chains -----------------------------------------------------

def test_plane_chain_keeps_the_branch():
    d = _tmpdir()
    pour = os.path.join(d, 'pour.kicad_pcb')
    _run('route_planes.py', [BOARD, pour, '--nets', 'VDD', '--plane-layers', 'B.Cu',
                             '--clearance', '0.2', '--track-width', '0.3'])
    evidence(pour, 'route_planes.py output')
    block = _member_block(_src())
    # The route step, plane net in its --nets: its in-run finalize (#562).
    out, log = _route(pour, ['--layers', 'F.Cu', 'B.Cu', '--track-width', '0.3',
                             '--clearance-ceiling', '0.2'])
    assert 'Plane finalize' in log, 'the in-run plane finalize did not run'
    with open(out, encoding='utf-8') as f:
        assert block in f.read(), 'the plane finalize rewrote or dropped the member track'
    assert _tapped(out) == ['U1.1'] and _branch(out).protected, _tapped(out)
    _grade(out)
    # repair_planes: pad repair plus blocker rips, on the poured board.
    rep = os.path.join(d, 'repair.kicad_pcb')
    _run('repair_planes.py', [pour, rep, '--nets', 'VDD', '--plane-layers', 'B.Cu',
                              '--repair-pads', '--rip-blocker-nets',
                              '--clearance', '0.2', '--track-width', '0.3'], timeout=900)
    evidence(rep, 'repair_planes.py output')
    with open(rep, encoding='utf-8') as f:
        assert block in f.read(), 'repair_planes rewrote or dropped the member track'
    assert _tapped(rep) == ['U1.1'] and _branch(rep).protected, _tapped(rep)


# --- (e): refusals -----------------------------------------------------------

def _refused(text):
    p = parse_kicad_pcb(_variant(text))
    b = _branch(p)
    assert not b.protected, 'refused branch was protected'
    tagged = [x for x in p.segments + p.vias if x.sense or x.locked]
    tagged += [q for qs in p.pads_by_net.values() for q in qs if q.sense]
    assert not tagged, f'a refused branch still tagged copper: {tagged}'
    # Its resolved member copper on the join pad's net is still HELD: a
    # refused branch routes like undeclared copper, but nothing deletes it.
    held = {x.uuid for x in p.segments + p.vias if x.held_by}
    assert held <= {s.uuid for s in b.segments + b.vias}, held
    return b.reason


def _with_group(name=None, members=None):
    g = GROUP_LINE
    if name is not None:
        g = g.replace('"sense:C1.1"', f'"{name}"')
    if members is not None:
        g = g.replace('(members "11111111-0000-0000-0000-000000000001")',
                      '(members ' + ' '.join(f'"{u}"' for u in members) + ')')
    return _src().replace(GROUP_LINE, g)


def test_refusals_name_their_reason():
    cases = [
        (_with_group(name='sense:C1'), 'does not name the join pad'),
        (_with_group(name='sense:C9.1'), 'join pad C9.1 is not on the board'),
        # The board outline's gr_rect: an item KiCad keeps in the group, but not
        # copper. (A uuid matching NO item is dropped on load, by KiCad and by
        # sense_groups_from_text alike -- test_1279_sense_branch_geometry.)
        (_with_group(members=[MEMBER, '22222222-0000-0000-0000-000000000001']),
         'are not tracks or vias on the board'),
        (_with_group(name='sense:U2.1'), 'does not touch the join pad U2.1'),
        (_with_group().replace(
            '  (segment (start 3 5)',
            '  (segment (start 15 5) (end 15 8) (width 0.3) (layer "F.Cu") (net 1) '
            '(uuid "44444444-0000-0000-0000-000000000001"))\n  (segment (start 3 5)'),
         'input copper already taps it (track (15, 5)-(15, 8) on F.Cu)'),
    ]
    # A member piece that reaches a sense pad (U4.1) but not the join pad.
    loose = _with_group(members=[MEMBER, '55555555-0000-0000-0000-000000000001']).replace(
        '  (segment (start 3 5)',
        '  (footprint "T:P" (layer "F.Cu") (at 27 15)\n'
        '    (property "Reference" "U4" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1))))\n'
        '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") (net 1 "VDD")))\n'
        '  (segment (start 27 15) (end 24 15) (width 0.2) (layer "F.Cu") (net 1) '
        '(uuid "55555555-0000-0000-0000-000000000001"))\n  (segment (start 3 5)')
    cases.append((loose, 'not joined to C1.1 by member copper'))
    for text, why in cases:
        reason = _refused(text)
        assert why in reason, f'expected {why!r}, got {reason!r}'


def test_same_net_pour_over_the_branch_is_refused():
    """A VDD pour whose outline covers the member trace already joins it to
    every load the pour feeds once KiCad fills it. A pour on another layer,
    or one that covers only the join pad, leaves the branch protectable."""
    def resolve(layer, box):
        p = parse_kicad_pcb(BOARD)
        for it in p.segments:          # a fresh resolution of the same board
            it.sense, it.locked = '', False
        for q in p.pads_by_net[1]:
            q.sense = ''
        x0, y0, x1, y1 = box
        p.zones = [Zone(net_id=1, net_name='VDD', layer=layer,
                        polygon=[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])]
        return sb.resolve_sense_branches(p, sb.sense_groups_from_text(_src()))[0]
    b = resolve('F.Cu', (10, 3, 14, 7))
    assert not b.protected and 'zone on F.Cu pours over its F.Cu track' in b.reason, b.reason
    assert resolve('B.Cu', (10, 3, 14, 7)).protected
    assert resolve('F.Cu', (1, 3, 3.4, 7)).protected


def test_route_discloses_a_refused_branch():
    tapped = _variant(_with_group().replace(
        '  (segment (start 3 5)',
        '  (segment (start 15 5) (end 15 8) (width 0.3) (layer "F.Cu") (net 1) '
        '(uuid "44444444-0000-0000-0000-000000000001"))\n  (segment (start 3 5)'), 'tapped')
    out, log = _route(tapped)
    assert 'sense:C1.1: NOT protected -- input copper already taps it' in log, log[-2000:]
    # The refused branch's member is held: the late strict collapse used to
    # delete it as redundant input copper once the branch was tapped.
    with open(out, encoding='utf-8') as f:
        assert _member_block(_src()) in f.read(), 'a refused branch lost its sense trace'
    entry = _summary(log)['sense_branches']
    assert entry == [{'group': 'sense:C1.1', 'join': 'C1.1', 'net': 'VDD', 'protected': False,
                      'segments': 1, 'vias': 0, 'sense_pads': ['U1.1'],
                      'reason': 'input copper already taps it (track (15, 5)-(15, 8) on F.Cu)'}], entry


def test_refused_branch_keeps_its_copper():
    """A tapped (refused) branch is routed like undeclared copper, but its
    member copper is held: the input's VDD loop C1.1 -> (3, 6.5) -> (27, 6.5)
    -> U1.1 makes the sense trace redundant, and the end-of-run strict
    collapse (#1063) used to delete it."""
    loop = ('  (segment (start 3 5) (end 3 6.5) (width 0.3) (layer "F.Cu") (net 1) '
            '(uuid "77777777-0000-0000-0000-000000000001"))\n'
            '  (segment (start 3 6.5) (end 27 6.5) (width 0.3) (layer "F.Cu") (net 1) '
            '(uuid "77777777-0000-0000-0000-000000000002"))\n'
            '  (segment (start 27 6.5) (end 27 5) (width 0.3) (layer "F.Cu") (net 1) '
            '(uuid "77777777-0000-0000-0000-000000000003"))\n')
    board = _variant(_src().replace('  (segment (start 3 5) (end 27 5)',
                                    loop + '  (segment (start 3 5) (end 27 5)'), 'loop')
    out, log = _route(board)
    assert 'sense:C1.1: NOT protected -- input copper already taps it' in log, log[-2000:]
    with open(out, encoding='utf-8') as f:
        assert _member_block(_src()) in f.read(), 'the refused branch lost its sense trace'


def test_rule_helpers():
    """The join-pad cut-out: member copper on the pad is the pad; terminal
    copper on the pad is the join; copper off it is measured."""
    p = parse_kicad_pcb(BOARD)
    b = _branch(p)
    # The member's copper leaves C1.1 where its whole 0.2 mm width stops being
    # on the pad: centreline x = 3.5 - 0.1 (sense_branch_geometry, the clip).
    assert sb._outside_join(b.segments[0], b.join) == [(3.4, 5.0, 27.0, 5.0)]
    # A terminal ending on C1.1's centre from above: on the pad, not a tap.
    assert sb.terminal_sense_dist(p, 1, 3.0, 3.0, 3.0, 5.0, 'F.Cu', 0.15) > 0.3
    # The same terminal ending 1.5 mm along the trace: it sits on the trace.
    assert sb.terminal_sense_dist(p, 1, 4.5, 3.0, 4.5, 5.0, 'F.Cu', 0.15) < 0
    # Another net sees nothing here (its copper is foreign anyway).
    assert sb.terminal_sense_dist(p, 2, 4.5, 3.0, 4.5, 5.0, 'F.Cu', 0.15) == 1e9
    # A ratsnest (#572 oracle) link end on the trace or on U1.1 lands on C1.1.
    assert sb.join_for_point(p, 1, 15.0, 5.0, 'F.Cu') is b.join
    assert sb.join_for_point(p, 1, 27.0, 5.0, 'F.Cu') is b.join
    assert sb.join_for_point(p, 1, 15.0, 8.0, 'F.Cu') is None
    assert sb.join_for_point(p, 1, 3.0, 5.0, 'F.Cu') is None


# --- (g): the hold is not a lock -------------------------------------------

def test_hold_is_not_a_lock():
    """The members are held (no pass may rewrite or remove them), not
    KiCad-locked: the run's own routes of the net stay rippable in-run, a
    whole-net strip of its pre-existing copper is refused naming the group,
    and nothing reports it as KiCad-locked."""
    import protected_nets as pn
    from kicad_parser import is_immutable_copper
    p = parse_kicad_pcb(BOARD)
    seg = _branch(p).segments[0]
    assert is_immutable_copper(seg) and not seg.locked
    assert pn.protection_map(p).get('VDD') == 'sense:C1.1', pn.protection_map(p)
    assert pn.in_run_rip_reason(p, 'VDD') is None, "in-run rips of the run's routes refused"
    pn.clear_skipped()
    assert pn.filter_rippable_names(['VDD'], pn.protection_map(p), ['VDD'],
                                    context='t1279') == [], 'an exact name lifted the hold'
    assert pn.PROTECTED_SKIPPED.get('t1279') == {'VDD': 'sense:C1.1'}, pn.PROTECTED_SKIPPED
    pn.clear_skipped()


def test_force_reroute_names_the_group():
    _, (out, _log) = _main_runs()
    log = _run('route.py', [out, os.path.join(_tmpdir(), 'fr.kicad_pcb'), '--nets', 'VDD',
                            '--force-reroute'] + GEOM)
    assert 'excluded from --force-reroute (sense:C1.1: VDD)' in log, log[-2000:]
    assert 'KiCad-locked' not in log, 'a sense hold reported as a KiCad lock'


def test_tapped_by_run_is_disclosed():
    """JSON_SUMMARY reports the board the run ships: a branch protected at
    the start that the run's copper taps carries `tapped_by_run`."""
    from kicad_parser import Segment
    p = parse_kicad_pcb(BOARD)
    assert 'tapped_by_run' not in sb.summary_entries(p)[0]
    p.segments.append(Segment(start_x=15, start_y=5, end_x=15, end_y=8, width=0.3,
                              layer='F.Cu', net_id=1))
    entry = sb.summary_entries(p)[0]
    assert entry['protected'] and "this run's copper already taps it" in entry.get(
        'tapped_by_run', ''), entry


def test_diff_pair_terminals_skip_sense_pads():
    """A multipoint diff pair's terminals never include a sense pad: treat
    VDD as the P leg and give each VDD pad an N mate 0.5 mm below it. U1.1
    (the sense pad) used to be paired like any other pad."""
    from dataclasses import replace
    from diff_pair_multipoint import get_diff_pair_terminals
    p = parse_kicad_pcb(BOARD)
    mates = [replace(q, net_id=99, net_name='VDD_N', global_y=q.global_y + 0.5, sense='')
             for q in p.pads_by_net[1]]
    p.pads_by_net[99] = mates
    pairs = sorted(f'{a.component_ref}.{a.pad_number}' for a, _b in
                   get_diff_pair_terminals(p, 1, 99))
    assert 'U1.1' not in pairs, f'a sense pad is a diff-pair terminal: {pairs}'
    assert pairs == ['C1.1', 'U2.1', 'U3.1'], pairs


# --- (f): the GUI call shape -------------------------------------------------

def test_gui_path_routes_to_the_join():
    """An in-memory PCBData, return_results=True (the Route tab's call): the
    branch is protected for the run, no new copper lands on the sense trace
    off C1.1, and every net id is the one the board gave it."""
    import route as _route_mod
    p = parse_kicad_pcb(BOARD)
    ids_before = {id(s): s.net_id for s in p.segments}
    nets_before = sorted(p.nets)
    _ok, _f, _t, data = _route_mod.batch_route(
        BOARD, '', ['VDD'], return_results=True, pcb_data=p,
        layers=['F.Cu', 'B.Cu'], track_width=0.3, clearance=0.2)
    assert sorted(p.nets) == nets_before, 'the net table changed'
    assert all(s.net_id == ids_before[id(s)] for s in p.segments if id(s) in ids_before)
    new = [s for r in data.get('results', []) for s in (r.get('new_segments') or [])]
    assert new and all(s.net_id == 1 for s in new)
    for s in new:
        d = sb.terminal_sense_dist(p, 1, s.start_x, s.start_y, s.end_x, s.end_y,
                                   s.layer, s.width / 2.0)
        assert d >= 0.2 + s.width / 2.0 - 1e-6, \
            f'new copper ({s.start_x}, {s.start_y})-({s.end_x}, {s.end_y}) within clearance of the branch'


TESTS = [
    test_parse_resolves_the_group,
    test_rule_helpers,
    test_refusals_name_their_reason,
    test_same_net_pour_over_the_branch_is_refused,
    test_new_copper_joins_at_the_join_pad_only,
    test_no_clearance_descent,
    test_no_net_id_is_rewritten,
    test_route_discloses_a_refused_branch,
    test_refused_branch_keeps_its_copper,
    test_plane_chain_keeps_the_branch,
    test_hold_is_not_a_lock,
    test_force_reroute_names_the_group,
    test_tapped_by_run_is_disclosed,
    test_diff_pair_terminals_skip_sense_pads,
    test_gui_path_routes_to_the_join,
]


def main():
    evidence(BOARD, 'the sense_star fixture')
    evidence(PRO, "the sense_star fixture's .kicad_pro")
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
