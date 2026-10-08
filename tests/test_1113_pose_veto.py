#!/usr/bin/env python3
"""#1113: `converge poses` says WHY it dropped every candidate.

Run 39: `converge.py poses --ref U1` on a seeded StickHub board dropped all
323 candidates, 90/180/270 in place included, and said only how many. A
one-part move holds every neighbour where it is, so once the seed has packed
an IC's decaps against its pins every other rotation of it is vetoed -- and
nothing said that was the reason. What each case pins:

* `QuenchState.candidate_veto` agrees with `candidate_valid` on EVERY
  candidate (it calls it), on the courtyard path and the #1101 waived path,
  and every refusal carries a label from `VETO_CHECKS` -- never
  'unattributed', which would be a rejection path with no label.
* a labelled refusal names the check that actually refused: a pose past the
  board inset is `board_bbox`, a pose on a neighbour's courtyard is
  `courtyard` against THAT neighbour.
* `rank_poses`' `dropped_by` partitions `dropped_total`, and
  `evaluated_total` = dropped + kept.
* when only staying put survives, `converge poses` says every candidate but
  staying put was vetoed, by what, and points at rank_rotations.py.
* an empty ranking's note names the check ("vetoed by ...").
* `pose_ops.snap_candidates`' census carries `dropped_by`.

    python3 tests/test_1113_pose_veto.py
"""
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb    # noqa: E402
import pose_score                           # noqa: E402
from placement import quench                # noqa: E402

RUN_ALL_TIMEOUT = 900

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
SPLITFLAP = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
CONVERGE = os.path.join(ROOT, 'py_placer', 'converge.py')


def _state(board, **kw):
    pcb = parse_kicad_pcb(board)
    kw.setdefault('clearance', 0.2)
    kw.setdefault('board_edge_clearance', 0.3)
    return pcb, pose_score.make_state(pcb, board, **kw)


def _lattice(st, ref, radius=1.0, step=0.5):
    p = st.parts[ref]
    for dx, dy in pose_score._offsets(radius, step):
        for rot in (0.0, 90.0, 180.0, 270.0):
            yield p.x + dx, p.y + dy, rot


def _agree(st, refs, seen):
    n = 0
    for ref in refs:
        for x, y, rot in _lattice(st, ref):
            veto = st.candidate_veto(ref, x, y, rot)
            ok = st.candidate_valid(ref, x, y, rot)
            assert (veto is None) == ok, (ref, x, y, rot, veto, ok)
            if veto is not None:
                assert veto[0] in quench.VETO_CHECKS, (ref, x, y, rot, veto)
                seen[veto[0]] = seen.get(veto[0], 0) + 1
            n += 1
    return n


def test_veto_agrees_with_valid_on_every_candidate():
    seen = {}
    n = 0
    for board in (ESP, SPLITFLAP):
        _pcb, st = _state(board)
        refs = [r for r in sorted(st.parts) if not st.parts[r].locked][:12]
        n += _agree(st, refs, seen)
        # #1101's waived path asks the pad question instead of courtyards
        st.courtyards_ignored = True
        n += _agree(st, refs, seen)
    assert len(seen) >= 3, seen
    assert st._why is None      # never left armed after a call
    print(f"  PASS: veto == not valid on {n} candidates; checks seen: "
          f"{dict(sorted(seen.items()))}")


def test_a_label_names_the_check_that_refused():
    _pcb, st = _state(ESP)
    refs = [r for r in sorted(st.parts) if not st.parts[r].locked
            and st.parts[r].pin_count >= 2]
    a = refs[0]
    pa = st.parts[a]
    # far past the board inset
    v = st.candidate_veto(a, st.usable[2] + 50.0, pa.y, pa.rot)
    assert v == ('board_bbox', None), v
    # on top of an interior same-side neighbour b: refused by a NEIGHBOUR
    # check, against b
    hits = []
    for b in refs[1:]:
        pb = st.parts[b]
        if pb.side != pa.side:
            continue
        v = st.candidate_veto(a, pb.x, pb.y, pa.rot)
        if v is not None and v[0] not in ('board_bbox', 'outline'):
            assert v[1] is not None, (v, a, b)
            hits.append((b, v))
    assert hits, a
    assert any(v[1] == b for b, v in hits), hits
    # two plain SMD parts on one side: the SMD courtyard path refuses, and
    # names the neighbour (a through-hole pair takes the other path)
    smd = [r for r in refs if not st.parts[r].has_tht
           and st.parts[r].side == pa.side]
    smd_hits = []
    for x in smd[:6]:
        for y in smd:
            if x == y:
                continue
            px, py_ = st.parts[x], st.parts[y]
            w = st.candidate_veto(x, py_.x, py_.y, px.rot)
            if w == ('courtyard', y):
                smd_hits.append((x, y))
    assert smd_hits, smd[:6]
    b, v = next((b, v) for b, v in hits if v[1] == b)
    print(f"  PASS: off the board -> board_bbox; on {b} -> {v[0]} against {b}"
          f" ({len(hits)} interior neighbour(s) tried)")


def test_dropped_by_partitions_dropped_total():
    pcb, st = _state(SPLITFLAP)
    ref = next(r for r in sorted(st.parts) if not st.parts[r].locked
               and st.parts[r].pin_count >= 8)
    diag = {}
    poses = pose_score.rank_poses(pcb, SPLITFLAP, ref, radius=2.0, step=0.5,
                                  limit=10_000, state=st, diagnostics=diag)
    by = diag['dropped_by']
    assert diag['dropped_total'] > 0, diag
    assert sum(d['count'] for d in by.values()) == diag['dropped_total'], by
    assert diag['evaluated_total'] == diag['dropped_total'] + len(poses), diag
    for chk, d in by.items():
        assert chk in quench.VETO_CHECKS, chk
        assert sum(n for _r, n in d['blockers']) <= d['count'], d
    assert [d['count'] for d in by.values()] == sorted(
        (d['count'] for d in by.values()), reverse=True), by
    print(f"  PASS: {ref}: {pose_score.veto_phrase(by)} = "
          f"{diag['dropped_total']} dropped of {diag['evaluated_total']}")


def _asymmetric_part(st):
    """A movable part whose rect at its own angle + 180 differs, so a
    half-turn in place is not a no-op."""
    for ref in sorted(st.parts):
        p = st.parts[ref]
        if p.locked:
            continue
        a = p.rects(p.x, p.y, p.rot)[0]
        b = p.rects(p.x, p.y, (p.rot + 180.0) % 360)[0]
        if max(abs(u - v) for u, v in zip(a, b)) > 0.05:
            return ref
    raise AssertionError('no asymmetric part on the fixture board')


def test_only_staying_put_survives_and_the_note_says_why():
    """The board inset shrunk to one part's own courtyard: every move and
    every other rotation is vetoed by `board_bbox`, staying put is not. The
    note must say so -- and must NOT send the reader to the seed-level
    ranker: a board-term veto says nothing about neighbours packed around
    the part (`test_the_ranker_pointer_needs_a_neighbour_veto`)."""
    import converge
    _pcb, st0 = _state(ESP)
    ref = _asymmetric_part(st0)
    real = pose_score.make_state

    def shrunk(*a, **kw):
        st = real(*a, **kw)
        p = st.parts[ref]
        st.usable = tuple(p.rects(p.x, p.y, p.rot)[0])
        return st

    class _A:
        board, clearance, board_edge_clearance = ESP, 0.2, 0.3
        radius, step, limit, route = 1.0, 0.5, 12, False
    _A.ref = ref
    import contextlib
    import io
    out, err = io.StringIO(), io.StringIO()
    pose_score.make_state = shrunk
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = converge.cmd_poses(_A)
    finally:
        pose_score.make_state = real
    assert rc == 0, (rc, out.getvalue()[-600:])
    d = json.loads(out.getvalue())
    assert d['all_moves_vetoed'] is True, d
    assert [(p['dist_mm'], p['rot']) for p in d['poses']] == \
        [(0.0, st0.parts[ref].rot)], d['poses']
    assert 'board_bbox' in d['dropped_by'], d['dropped_by']
    note = d.get('note') or ''
    assert note.startswith('every candidate except staying put was vetoed'), \
        note
    assert 'board_bbox' in note and 'rank_rotations.py' not in note, note
    assert note in err.getvalue(), err.getvalue()[-600:]
    print(f"  PASS: {ref}: '{note[:90]}...'")


def test_the_ranker_pointer_needs_a_neighbour_veto():
    """`_in_place_clause` sends the reader to rank_rotations only when a
    TURNED in-place pose was refused by a neighbour (courtyard, pads, a
    body); a board term (board_bbox, outline, intent) or the part's own
    angle never does."""
    import converge

    def diag(*rows, rot=0.0):
        return {'dropped_in_place_by': [
                    {'rot': r, 'check': c, 'blockers': {}} for r, c in rows],
                'in_place_evaluated': True, 'input_rotation': rot}
    say = converge._in_place_clause
    for check in converge._NEIGHBOUR_CHECKS:
        assert 'rank_rotations.py' in say('U1', diag((90.0, check))), check
    for check in ('board_bbox', 'outline', 'intent', 'keepout_band'):
        assert check not in converge._NEIGHBOUR_CHECKS, check
        txt = say('U1', diag((90.0, check)))
        assert 'rank_rotations.py' not in txt and 'In place: ' in txt, txt
    # a neighbour veto at the part's OWN angle is not a rotation question
    assert 'rank_rotations.py' not in say('U1', diag((270.0, 'courtyard'),
                                                     rot=270.0))
    # mixed: one turned neighbour veto is enough
    assert 'rank_rotations.py' in say('U1', diag((0.0, 'board_bbox'),
                                                 (180.0, 'pads')))
    from placement import quench
    assert set(converge._NEIGHBOUR_CHECKS) <= set(quench.VETO_CHECKS), \
        set(converge._NEIGHBOUR_CHECKS) - set(quench.VETO_CHECKS)
    print(f"  PASS: the pointer follows {len(converge._NEIGHBOUR_CHECKS)} "
          f"neighbour checks and no board term")


def test_an_empty_ranking_names_the_check():
    r = subprocess.run([sys.executable, '-X', 'utf8', CONVERGE, 'poses',
                        SPLITFLAP, '--ref', 'C1', '--radius', '0.5',
                        '--step', '0.5', '--clearance', '50'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    assert r.returncode == 1, (r.returncode, r.stdout[-400:])
    d = json.loads(r.stdout)
    assert d['poses'] == [] and d['dropped_in_place'], d
    assert d['note'].startswith('no legal pose, including staying put -- '
                                'vetoed by '), d['note']
    # the part's OWN spot is named, whatever the top-3 cut kept
    assert 'In place: ' in d['note'], d['note']
    assert d['dropped_in_place_by'][0]['check'] in d['note'], d['note']
    top = next(iter(d['dropped_by']))
    assert top in d['note'] and len(d['dropped_in_place_by']) == len(
        d['dropped_in_place']), d
    print(f"  PASS: {d['note']}")


def test_snap_census_carries_dropped_by():
    from placement import pose_ops
    pcb = parse_kicad_pcb(SPLITFLAP)
    ref = 'C1'
    fp = pcb.footprints[ref]
    _cands, census = pose_ops.snap_candidates(
        SPLITFLAP, ref, rot=fp.rotation or 0.0, clearance=0.2,
        board_edge_clearance=0.3, radius=1.0, step=0.5, pcb_data=pcb)
    assert 'dropped_by' in census, census
    assert sum(d['count'] for d in census['dropped_by'].values()) == \
        census['dropped_total'], census
    print(f"  PASS: snap census: {census['dropped_total']} dropped, by "
          f"{sorted(census['dropped_by'])}")


def test_a_part_coming_home_names_the_overlap():
    """A part off the board may move back toward it (the escape rule) only
    without overlapping anything. A candidate the escape rule refuses for
    overlap must not read as `board_bbox` (phase-2 verifier: 229 such
    candidates on esp_prog were labelled board_bbox)."""
    pcb, st = _state(ESP)
    seen = {}
    n = 0
    for ref in [r for r in sorted(st.parts) if not st.parts[r].locked][:10]:
        p = st.parts[ref]
        x0, y0, r0 = p.x, p.y, p.rot
        r = p.rects(x0, y0, r0)[0]
        # just past the usable inset on the right, then slide back home
        st.apply_move(ref, x0 + (st.usable[2] - r[2]) + 0.6, y0, r0)
        p = st.parts[ref]
        try:
            for dx in (-0.2, -0.4, -0.6, -0.8):
                for dy in (-1.0, -0.5, 0.0, 0.5, 1.0):
                    x, y = p.x + dx, p.y + dy
                    v = st.candidate_veto(ref, x, y, r0)
                    assert (v is None) == st.candidate_valid(ref, x, y, r0)
                    if v is None:
                        continue
                    n += 1
                    seen[v[0]] = seen.get(v[0], 0) + 1
                    # only where the escape rule judged it: the incumbent
                    # is off the board and overlaps nothing (otherwise the
                    # ordinary path's first failing conjunct IS the reason)
                    cb, co = st._incumbent_violation(ref)
                    if (v[0] == 'board_bbox' and co <= quench.EPS_IMPROVE
                            and cb > quench.EPS_IMPROVE):
                        _b, ov = st.violation_parts(ref, x, y, r0)
                        assert ov <= quench.EPS_IMPROVE, (ref, x, y, v, ov)
        finally:
            st.apply_move(ref, x0, y0, r0)
    assert n and 'escape_overlap' in seen, seen
    print(f"  PASS: {n} refused homecomings; labels {seen}")


TESTS = [
    test_veto_agrees_with_valid_on_every_candidate,
    test_a_part_coming_home_names_the_overlap,
    test_a_label_names_the_check_that_refused,
    test_dropped_by_partitions_dropped_total,
    test_only_staying_put_survives_and_the_note_says_why,
    test_the_ranker_pointer_needs_a_neighbour_veto,
    test_an_empty_ranking_names_the_check,
    test_snap_census_carries_dropped_by,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
