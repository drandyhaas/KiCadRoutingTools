"""
Route / preview / undo one placement block (#459 follow-on).

`placement/groups.py` says which footprints belong together. This covers the
three things you then want to do with a block, all on `route.py`:

    --group BLOCK        scope a routing run to the block's nets
    --preview            route it and report what WOULD change, writing nothing
    --undo               remove the block's copper, back to unrouted

Two scopes, because which one is right depends on the block, measured:

    internal   every pad inside the block. A schematic sheet is 60-70% internal
               (glasgow's two 68-part sheets: 52 internal vs 23 boundary), so
               "route this sheet" is a real self-contained job.
    touching   any pad inside the block, interface nets included -- what
               --component already means, and the ONLY useful reading for a
               decap block, which is 0% internal by construction because a
               decoupling cap bridges VCC to GND and both span the board.
"""

import os
import shutil
import subprocess
import sys

#: This file routes three real chains and needs more than run_all's 600 s cap
#: under load. #718 item 4: it timed out in `run_all -j 4` on the reporter's
#: machine and PASSED alone -- machine contention, not a defect, but a killed
#: run yields no partial result and no diagnosis, so the fact was read as a
#: code fact. Measured alone here: 297 s (macOS, py3.14, with a parallel
#: suite running), so 600 s is roughly 2x headroom before contention and a
#: slower machine eat it -- which is exactly what happened. 1200 s matches
#: what test_obstacle_map_balance declares for the same reason.
#:
#: A declared budget is a claim the test makes about ITSELF, where the next
#: reader looks. Raising the global --timeout instead would hide a genuinely
#: hung test behind the slow ones.
RUN_ALL_TIMEOUT = 1200
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from group_routing import (GroupRoutingError, block_net_names, block_refs,
                           unroute_nets)
from kicad_parser import parse_kicad_pcb
from placement.groups import parse_sources

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(ROOT, 'kicad_files')
ROUTED = os.path.join(KF, 'rp2350_fpga_eensy_prePlane.kicad_pcb')   # 904 segments
FLAT = os.path.join(KF, 'tigard.kicad_pcb')                         # no sheets


def _run(*args, timeout=1800):
    return subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'route.py')] + list(args),
                          capture_output=True, text=True, cwd=ROOT, timeout=timeout)


# --- scoping ------------------------------------------------------------------

def test_internal_and_touching_differ_and_both_are_useful():
    pcb = parse_kicad_pcb(ROUTED)
    refs = block_refs(pcb, 'sheet:558c3023', parse_sources('sheet'))
    inner = block_net_names(pcb, refs, 'internal')
    outer = block_net_names(pcb, refs, 'touching')
    assert set(inner) < set(outer), "internal must be a strict subset of touching"
    assert len(inner) >= 10, f"sheet block should be substantially internal: {len(inner)}"
    print(f"  PASS: sheet block -> {len(inner)} internal of {len(outer)} touching")


def test_a_decap_block_is_mostly_boundary_nets():
    """A decoupling cap bridges a rail to GND, and board-wide nets cross the
    block boundary -- which is why 'touching' has to exist as a scope. But
    "0% internal by construction" is FALSE since f38204e tethers caps on
    their RAIL: an IC's own local regulator rail (IC + its bypass caps only,
    tigard's VREG) is legitimately 100% internal to the decap block. GND
    must never be internal; anything internal must also be touching."""
    pcb = parse_kicad_pcb(FLAT)
    refs = block_refs(pcb, 'decap:U3', parse_sources('decap'))
    inner = set(block_net_names(pcb, refs, 'internal'))
    outer = set(block_net_names(pcb, refs, 'touching'))
    assert inner == {'VREG'}, (
        f"internal nets {sorted(inner)}: expected exactly the FT2232H's "
        f"local VREG rail (its owners are U3 + its own bypass caps, all "
        f"inside the block since f38204e's rail tethering)")
    assert 'GND' not in inner and inner < outer
    assert len(outer) > 10


def test_unknown_block_names_what_exists():
    pcb = parse_kicad_pcb(FLAT)
    try:
        block_refs(pcb, 'nope', parse_sources('decap'))
    except GroupRoutingError as e:
        assert 'nope' in str(e) and 'Available' in str(e)
        return
    raise AssertionError("an unknown block must be rejected with the available list")


def test_short_name_is_accepted():
    """The banner prints sheet blocks by uuid tail; a user must be able to paste
    back what they saw."""
    pcb = parse_kicad_pcb(ROUTED)
    full = block_refs(pcb, 'sheet:558c3023', parse_sources('sheet'))
    assert full, "fixture"
    assert block_refs(pcb, '558c3023', parse_sources('sheet')) == full


def test_list_groups_reports_both_net_counts():
    r = _run(FLAT, os.devnull, '--list-groups', '--group-by', 'decap')
    assert r.returncode == 0, r.stdout[-600:]
    assert 'placement block(s)' in r.stdout
    assert 'touching=' in r.stdout and 'internal=' in r.stdout


def test_group_intersects_net_patterns_and_says_so():
    """--group composes with patterns exactly like --component: patterns given
    -> intersect. The banner must report the count actually acted on, not the
    block's full tally, or it advertises 20 nets while touching 2."""
    r = _run(ROUTED, os.devnull, '--group', 'sheet:558c3023', '--group-by', 'sheet',
             '--group-scope', 'touching', '--nets', '*USB*', '--undo')
    assert r.returncode == 0, r.stdout[-500:]
    banner = [l for l in r.stdout.splitlines() if 'block:' in l][0]
    undo = [l for l in r.stdout.splitlines() if l.startswith('Undo:')][0]
    acted = int(banner.split('acting on')[1].split()[0])
    across = int(undo.split('across')[1].split()[0])
    assert acted == across, f"banner says {acted}, undo did {across}"
    assert acted < 20, "the intersection must actually narrow the block"
    assert 'dropped by the net patterns' in banner


def test_a_boundary_net_is_not_internal():
    """The D+/D- pair leaves the USB sheet, so 'internal' excludes it. This is
    the distinction the two scopes exist for -- not a matching bug."""
    pcb = parse_kicad_pcb(ROUTED)
    refs = block_refs(pcb, 'sheet:558c3023', parse_sources('sheet'))
    inner = set(block_net_names(pcb, refs, 'internal'))
    boundary = set(block_net_names(pcb, refs, 'touching')) - inner
    assert not [n for n in inner if 'USB' in n.upper() or n.endswith(('D+', 'D-'))]
    assert [n for n in boundary if n.endswith(('D+', 'D-'))], sorted(boundary)


# --- undo ---------------------------------------------------------------------

def test_undo_removes_exactly_the_block_and_nothing_else():
    pcb = parse_kicad_pcb(ROUTED)
    refs = block_refs(pcb, 'sheet:558c3023', parse_sources('sheet'))
    nets = set(block_net_names(pcb, refs, 'internal'))
    ids = {n for n, x in pcb.nets.items() if x.name in nets}
    before_in = sum(1 for s in pcb.segments if s.net_id in ids)
    before_all = len(pcb.segments)
    assert before_in > 0, "fixture: the block must start with copper"

    fd, out = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    try:
        r = _run(ROUTED, out, '--group', 'sheet:558c3023', '--group-by', 'sheet',
                 '--group-scope', 'internal', '--undo')
        assert r.returncode == 0, r.stdout[-800:]
        after = parse_kicad_pcb(out)
        ids2 = {n for n, x in after.nets.items() if x.name in nets}
        after_in = sum(1 for s in after.segments if s.net_id in ids2)
        assert after_in == 0, f"{after_in} segment(s) of the block survived"
        # Everything NOT in the block is untouched.
        assert before_all - before_in == len(after.segments), \
            (f"collateral damage: {before_all}-{before_in} != {len(after.segments)}")
        # VIAS too -- they are removed by POSITION, which is the one path where
        # hitting another net's copper was actually plausible.
        before_v = sum(1 for v in pcb.vias if v.net_id in ids)
        assert before_v > 0, "fixture: the block must start with vias"
        assert not [v for v in after.vias if v.net_id in ids2], "block vias survived"
        assert len(pcb.vias) - before_v == len(after.vias), \
            (f"via collateral: {len(pcb.vias)}-{before_v} != {len(after.vias)}")
        print(f"  PASS: removed {before_in} segs + {before_v} vias, "
              f"{len(after.segments)} segs / {len(after.vias)} vias untouched")
    finally:
        for p in (out, os.path.splitext(out)[0] + '.kicad_pro'):
            if os.path.exists(p):
                os.unlink(p)


def test_undo_refuses_real_locked_copper():
    """#521 gives 'locked' no override anywhere else; an undo is not the place
    to invent one.

    Uses REAL `(locked yes)` copper, not a patched protection_map: patching only
    proves unroute_nets honors the string 'locked', which would still pass if the
    sentinel were wrong or locked_net_names were broken."""
    import re
    d = tempfile.mkdtemp()
    src = os.path.join(d, 'locked.kicad_pcb')
    try:
        s = open(ROUTED, encoding='utf-8').read()
        pcb0 = parse_kicad_pcb(ROUTED)
        refs = block_refs(pcb0, 'sheet:558c3023', parse_sources('sheet'))
        nets = block_net_names(pcb0, refs, 'internal')
        target_id = next(i for i, n in pcb0.nets.items() if n.name == nets[0])
        # Lock ONE segment of that net, inserted where both parse paths accept
        # it. The net token has two spellings -- `(net 103)` and KiCad 10's
        # `(net "/NAME")` -- and this corpus board uses the NAME form, so match
        # either rather than assuming the id.
        net_tok = r'\(net\s+(?:%d|"%s")\)' % (target_id, re.escape(nets[0]))
        s2, n = re.subn(r'(\(segment\s*\(start[^)]*\)\s*\(end[^)]*\)\s*'
                        r'\(width\s+[\d.]+\)\s*)(\(layer\s+"[^"]+"\)\s*%s)' % net_tok,
                        r'\1(locked yes) \2', s, count=1)
        assert n == 1, "fixture: could not lock a segment"
        open(src, 'w', encoding='utf-8', newline='').write(s2)

        pcb = parse_kicad_pcb(src)
        assert any(seg.locked for seg in pcb.segments), "fixture: parser saw no lock"
        _, rep = unroute_nets(open(src, encoding='utf-8').read(), pcb, nets, src)
        assert nets[0] in rep['locked_skipped'], \
            f"real locked copper not refused: {rep['locked_skipped']}"
        assert nets[0] not in rep['nets'], "locked net must not be stripped"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_preview_exits_zero_even_when_nets_fail():
    """redo_commands.sh runs under `set -e` and previews get recorded. A
    read-only step that changes nothing must not be the one thing that can kill
    a replay -- and a normal route.py run with failed nets exits 0 anyway."""
    r = _run(ROUTED, os.devnull, '--group', 'sheet:558c3023', '--group-by',
             'sheet', '--group-scope', 'internal', '--preview',
             '--max-iterations', '1')       # tiny budget => failures guaranteed
    assert 'PREVIEW' in r.stdout, r.stdout[-500:]
    assert r.returncode == 0, f"preview exited {r.returncode} on routing failure"


def test_undo_reports_what_it_cannot_invert():
    """Pad/target swaps have no inverse and can touch nets outside the scope.
    Saying so is part of the feature, not a footnote."""
    fd, out = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    try:
        r = _run(ROUTED, out, '--group', 'sheet:558c3023', '--group-by', 'sheet',
                 '--group-scope', 'internal', '--undo')
        assert 'swaps are NOT inverted' in r.stdout, r.stdout[-500:]
    finally:
        for p in (out, os.path.splitext(out)[0] + '.kicad_pro'):
            if os.path.exists(p):
                os.unlink(p)


def test_undo_is_a_pure_function_of_the_nets():
    """unroute_nets works on text, so it must not depend on anything but the
    named nets -- calling it twice is idempotent."""
    pcb = parse_kicad_pcb(ROUTED)
    refs = block_refs(pcb, 'sheet:558c3023', parse_sources('sheet'))
    nets = block_net_names(pcb, refs, 'internal')
    c1, r1 = unroute_nets(open(ROUTED, encoding='utf-8').read(), pcb, nets, ROUTED)
    assert r1['segments_removed'] == r1['segments_found'] > 0
    fd, tmp = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    try:
        open(tmp, 'w', encoding='utf-8').write(c1)
        _, r2 = unroute_nets(c1, parse_kicad_pcb(tmp), nets, tmp)
        assert r2['segments_found'] == 0 and r2['vias_found'] == 0, \
            "a second undo should find nothing left"
    finally:
        os.unlink(tmp)


def test_undo_refuses_to_run_unscoped():
    """A routing run with no scope means "the whole board", which is the right
    default for ROUTING. For an undo the same default erases every track, so it
    has to be asked for. Caught by wiping a board by accident."""
    r = _run(ROUTED, os.devnull, '--undo')
    assert r.returncode == 2, f"unscoped undo was allowed: {r.stdout[-400:]}"
    assert 'explicit scope' in r.stderr, r.stderr[-400:]


def test_empty_group_is_rejected_not_widened():
    """`--group "$(...)"` with a shell expansion that produced nothing is falsy,
    so the scope silently widened to '*'. That is how the above wipe happened."""
    r = _run(ROUTED, os.devnull, '--group', '', '--undo')
    assert r.returncode == 2, f"empty --group was accepted: {r.stdout[-400:]}"
    assert 'empty name' in r.stderr, r.stderr[-400:]


def test_undo_carries_the_drc_floor():
    """#441: an undo only REMOVES copper, so the input's floor is still correct
    and must travel. Stranding it makes the next step resolve a looser stock
    netclass and KiCad then reports phantom sub-clearance DRC."""
    src = os.path.join(KF, 'routed_output.kicad_pcb')
    assert os.path.isfile(os.path.splitext(src)[0] + '.kicad_pro'), "fixture"
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'undone.kicad_pcb')
        r = _run(src, out, '--nets', 'GND', '--undo')
        assert r.returncode == 0, r.stdout[-600:]
        assert os.path.isfile(os.path.splitext(out)[0] + '.kicad_pro'), \
            "the sibling .kicad_pro did not travel (#441)"
        assert 'carried the DRC floor' in r.stdout
    finally:
        shutil.rmtree(d, ignore_errors=True)


def _board_with_an_arc(dst, locked=False):
    """Copy ROUTED, converting one (segment) of a net into an equivalent (arc).

    No corpus board has arc tracks, which is exactly why the arc bug survived:
    the parser LINEARIZES arcs into pcb_data.segments, so an undo counted them,
    but the writer only deletes (segment) blocks -- so the net kept its arc and
    the run still reported success.
    """
    import re
    s = open(os.path.join(KF, 'routed_output.kicad_pcb'), encoding='utf-8').read()
    m = re.search(r'\(segment\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\(end ([-\d.]+) '
                  r'([-\d.]+)\)\s*\(width ([\d.]+)\)\s*\(layer "([^"]+)"\)\s*'
                  r'\(net (\d+)\)', s)
    x1, y1, x2, y2, w, layer, net = m.groups()
    mx, my = (float(x1) + float(x2)) / 2 + 0.01, (float(y1) + float(y2)) / 2 + 0.01
    end = s.index(')', s.index(')', s.index('(uuid', m.start())) + 1) + 1
    lock = '(locked yes) ' if locked else ''   # where both parse paths accept it
    arc = (f'(arc (start {x1} {y1}) (mid {mx:.4f} {my:.4f}) (end {x2} {y2})'
           f' (width {w}) {lock}(layer "{layer}") (net {net})'
           f' (uuid "aaaaaaaa-0000-0000-0000-00000000beef"))')
    open(dst, 'w', encoding='utf-8', newline='').write(s[:m.start()] + arc + s[end:])
    return int(net)


def test_undo_removes_arc_tracks_too():
    """The router never emits arcs; KiCad routes rounded corners as them and
    hand-routed boards are full of them. An undo that leaves them behind has not
    unrouted the net, however good its tally looks."""
    d = tempfile.mkdtemp()
    src, out = os.path.join(d, 'arc.kicad_pcb'), os.path.join(d, 'undone.kicad_pcb')
    try:
        net_id = _board_with_an_arc(src)
        pcb = parse_kicad_pcb(src)
        name = pcb.nets[net_id].name
        assert open(src, encoding='utf-8').read().count('(arc ') == 1, "fixture"
        r = _run(src, out, '--nets', name, '--undo')
        assert r.returncode == 0, r.stdout[-500:]
        assert open(out, encoding='utf-8').read().count('(arc ') == 0, \
            "the (arc ...) track survived the undo"
        after = parse_kicad_pcb(out)
        n2n = {i: x.name for i, x in after.nets.items()}
        assert not [s for s in after.segments if n2n.get(s.net_id) == name], \
            "copper remains on a net the undo reported as removed"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_undo_never_touches_a_locked_arc():
    """#521 gives locked copper no override -- including in the arc path."""
    d = tempfile.mkdtemp()
    src = os.path.join(d, 'arc.kicad_pcb')
    try:
        net_id = _board_with_an_arc(src, locked=True)
        content = open(src, encoding='utf-8').read()
        assert '(locked yes)' in content, "fixture"
        pcb = parse_kicad_pcb(src)
        from group_routing import _remove_arc_tracks
        _, n, _ = _remove_arc_tracks(content, {net_id}, {pcb.nets[net_id].name})
        assert n == 0, f"deleted {n} LOCKED arc(s); locked copper has no override"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_undo_reports_a_surviving_zone():
    """A filled pour IS copper. Claiming a net is back to 'no copper' while its
    plane is still poured is simply false."""
    # interf_u_plane is GENERATED (route_planes over the tracked
    # interf_u_unrouted root) and gitignored, so it does not exist in a fresh
    # clone -- #457 item 3. Build it on demand instead of hardcoding the path,
    # which made this test fail outright anywhere the board had not already
    # been produced by an earlier local run.
    from fixture_boards import ensure
    src = ensure('interf_u_plane.kicad_pcb')
    r = _run(src, os.devnull, '--nets', 'VCC', '--undo')
    assert r.returncode == 0, r.stdout[-400:]
    assert 'ZONE' in r.stdout, r.stdout[-600:]
    assert 'VCC' in r.stdout.split('ZONE')[1][:200]


def test_undo_default_scope_stays_inside_the_block():
    """A block's 'touching' set includes GND/VCC, so an undo at that scope strips
    them board-wide. Routing that set is right; UNDOING it is not, so the two
    operations get different defaults."""
    pcb = parse_kicad_pcb(ROUTED)
    refs = block_refs(pcb, 'sheet:558c3023', parse_sources('sheet'))
    inner = len(block_net_names(pcb, refs, 'internal'))

    r = _run(ROUTED, os.devnull, '--group', 'sheet:558c3023',
             '--group-by', 'sheet', '--undo')          # NO --group-scope
    across = int([l for l in r.stdout.splitlines()
                  if l.startswith('Undo:')][0].split('across')[1].split()[0])
    assert across == inner, f"default undo scope took {across} nets, not {inner}"

    # Explicitly asking for the wide scope is allowed, but must warn.
    r2 = _run(ROUTED, os.devnull, '--group', 'sheet:558c3023', '--group-by',
              'sheet', '--group-scope', 'touching', '--undo')
    assert 'across the whole board' in r2.stdout, r2.stdout[-500:]


def test_undo_does_not_litter_on_a_non_board_output():
    """`args.output_file[:-10]` is empty for 'nul'/os.devnull, so the sibling
    carry created a stray '.kicad_pro' dotfile in the CWD and announced it."""
    before = set(os.listdir(ROOT))
    src = os.path.join(KF, 'routed_output.kicad_pcb')
    assert os.path.isfile(os.path.splitext(src)[0] + '.kicad_pro'), "fixture"
    r = _run(src, os.devnull, '--nets', 'GND', '--undo')
    assert r.returncode == 0, r.stdout[-400:]
    new = set(os.listdir(ROOT)) - before
    assert not new, f"undo littered the repo root: {new}"
    assert 'carried the DRC floor' not in r.stdout, \
        "claimed to carry a floor to a non-board output"


def test_fuzzy_group_match_is_echoed():
    """--group U3 can resolve to decap:U3 and start deleting copper; the run has
    to say which block it picked."""
    r = _run(os.path.join(KF, 'tigard.kicad_pcb'), os.devnull,
             '--group', 'U3', '--group-by', 'decap', '--list-groups')
    assert r.returncode == 0
    r = _run(os.path.join(KF, 'tigard.kicad_pcb'), os.devnull, '--group', 'U3',
             '--group-by', 'decap', '--group-scope', 'touching', '--undo')
    assert "resolved to block 'decap:U3'" in r.stdout, r.stdout[-500:]


# --- preview ------------------------------------------------------------------

def test_preview_writes_no_board():
    out = os.path.join(tempfile.gettempdir(), 'preview_must_not_exist.kicad_pcb')
    if os.path.exists(out):
        os.unlink(out)
    r = _run(ROUTED, out, '--group', 'sheet:558c3023', '--group-by', 'sheet',
             '--group-scope', 'internal', '--preview', '--max-iterations', '50000')
    assert 'PREVIEW (no output file written)' in r.stdout, r.stdout[-600:]
    assert 'would ADD' in r.stdout
    assert not os.path.exists(out), "--preview wrote a board file"


def test_preview_of_an_undone_block_reports_real_additions():
    """End to end: undo a block, then preview routing it back. The preview must
    report the copper it would add -- a zero there means the result keys drifted
    (they are new_segments/new_vias, not segments/vias)."""
    fd, undone = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    never = os.path.join(tempfile.gettempdir(), 'preview_never.kicad_pcb')
    if os.path.exists(never):
        os.unlink(never)
    try:
        r = _run(ROUTED, undone, '--group', 'sheet:558c3023', '--group-by', 'sheet',
                 '--group-scope', 'internal', '--undo')
        assert r.returncode == 0, r.stdout[-500:]
        r = _run(undone, never, '--group', 'sheet:558c3023', '--group-by', 'sheet',
                 '--group-scope', 'internal', '--preview', '--max-iterations', '200000')
        assert 'PREVIEW' in r.stdout, r.stdout[-600:]
        line = [l for l in r.stdout.splitlines() if 'would ADD' in l][0]
        added = int(line.split('would ADD')[1].split('segment')[0].strip(' :'))
        assert added > 0, f"preview reported no additions after an undo: {line}"
        assert not os.path.exists(never)
        print(f"  PASS: {line.strip()}")
    finally:
        for p in (undone, os.path.splitext(undone)[0] + '.kicad_pro'):
            if os.path.exists(p):
                os.unlink(p)


TESTS = [
    test_internal_and_touching_differ_and_both_are_useful,
    test_a_decap_block_is_mostly_boundary_nets,
    test_unknown_block_names_what_exists,
    test_short_name_is_accepted,
    test_list_groups_reports_both_net_counts,
    test_group_intersects_net_patterns_and_says_so,
    test_a_boundary_net_is_not_internal,
    test_undo_removes_exactly_the_block_and_nothing_else,
    test_undo_refuses_real_locked_copper,
    test_preview_exits_zero_even_when_nets_fail,
    test_undo_reports_what_it_cannot_invert,
    test_undo_is_a_pure_function_of_the_nets,
    test_undo_refuses_to_run_unscoped,
    test_empty_group_is_rejected_not_widened,
    test_undo_carries_the_drc_floor,
    test_undo_removes_arc_tracks_too,
    test_undo_never_touches_a_locked_arc,
    test_undo_reports_a_surviving_zone,
    test_undo_default_scope_stays_inside_the_block,
    test_undo_does_not_litter_on_a_non_board_output,
    test_fuzzy_group_match_is_echoed,
    test_preview_writes_no_board,
    test_preview_of_an_undone_block_reports_real_additions,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
