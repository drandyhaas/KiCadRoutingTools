#!/usr/bin/env python3
"""The pair_order hoists are an OPTIMISATION: every number must be identical.

`pair_order` had two pose-invariant rebuilds on its hot path -- `set(state.net_refs)`
per call, and the moving part's whole `pad_globals` transform once per PARTNER. Both
are hoisted so the pin-facing cost can be evaluated inside a search loop at all.

This file exists because a speed-up that moves an inversion count is worse than no
speed-up: every measurement downstream of it -- the A/B baseline, `placement_score`'s
`pin_order_crossings`, `pose_score`'s ranking -- would be silently rebased on a
different number. A draft of the pad hoist that always treated the queried ref as
side A was 6.7x faster and reported ulx3s U2 as **358** inversions where the truth is
26, because `pair_metrics` builds `order_a` from the FIRST ref's pads and takes its
channel axis as `b - a`. That draft passed every existing test in the tree.

So the assertion here is not "the suite is green". It is: for every ref on every
board, recomputing WITHOUT the hoists gives the same integer, and for every PAIR,
the full metrics dict is equal field for field.

Run: python3 -X utf8 tests/test_893_pair_order_equivalence.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 600
RUN_ALL_FAST_OK = True

#: Small, medium and large: the per-call cost and the partner-set size both grow
#: with the board, and the pad hoist only pays on parts with many partners. A
#: one-board check would not have caught the draft described above -- esp_prog
#: showed 2 differing refs, ulx3s 3.
BOARDS = ('esp_prog.kicad_pcb', 'tigard.kicad_pcb', 'ulx3s.kicad_pcb')

TESTS = []


def _state(board_name):
    from kicad_parser import parse_kicad_pcb
    import pose_score
    path = run_utils.evidence(os.path.join(ROOT, 'kicad_files', board_name),
                              'corpus board')
    pcb = parse_kicad_pcb(path)
    return pose_score.make_state(pcb, path)


def _reference_escape_pads(part, nets, toward_xy, pose=None):
    """`_escape_pads` as it stood BEFORE the hoists, transcribed.

    Deliberately a transcription and not a call: the point is to compare the
    shipped implementation against an independent one. It is short enough that
    the transcription is auditable by eye.
    """
    x, y, rot = pose if pose is not None else (part.x, part.y, part.rot)
    tx, ty = toward_xy
    best = {}
    for gx, gy, nid in part.pad_globals(x, y, rot):
        if nid not in nets:
            continue
        d = (gx - tx) ** 2 + (gy - ty) ** 2
        cur = best.get(nid)
        if cur is None or (d, gx, gy) < cur:
            best[nid] = (d, gx, gy)
    return {nid: (gx, gy) for nid, (_, gx, gy) in best.items()}


def _reference_ref_inversions(state, ref):
    """`ref_inversions` computed with NO caches and NO hoisted pads."""
    from placement import pair_order as PO
    part = state.parts.get(ref)
    if part is None:
        return 0
    partners = set()
    for nid in part.nets:
        partners.update(state.net_refs.get(nid, ()))
    partners.discard(ref)
    total = 0
    for other in sorted(partners):
        a, b = (ref, other) if ref < other else (other, ref)
        pa, pb = state.parts[a], state.parts[b]
        shared = sorted(set(pa.nets) & set(pb.nets) & set(state.net_refs))
        if len(shared) < 2:
            continue
        import math
        ax, ay = pa.x, pa.y
        bx, by = pb.x, pb.y
        ux, uy = bx - ax, by - ay
        n = math.hypot(ux, uy)
        if n < 1e-9:
            continue
        vx, vy = -uy / n, ux / n
        ea = _reference_escape_pads(pa, set(shared), (bx, by))
        eb = _reference_escape_pads(pb, set(shared), (ax, ay))
        common = [nid for nid in shared if nid in ea and nid in eb]
        if len(common) < 2:
            continue
        order_a = sorted(common,
                         key=lambda i: (ea[i][0] * vx + ea[i][1] * vy, i))
        rank_b = {nid: i for i, nid in enumerate(
            sorted(common, key=lambda i: (eb[i][0] * vx + eb[i][1] * vy, i)))}
        total += PO._merge_count([rank_b[nid] for nid in order_a])
    return total


def test_ref_inversions_identical_per_ref():
    """Per REF, not summed. A sum can cancel two opposite errors."""
    from placement import pair_order as PO
    for board in BOARDS:
        st = _state(board)
        bad = []
        for ref in sorted(st.parts):
            got = PO.ref_inversions(st, ref)
            want = _reference_ref_inversions(st, ref)
            if got != want:
                bad.append((ref, got, want))
        assert not bad, (
            '%s: %d ref(s) disagree with the un-hoisted reference, e.g. %r'
            % (board, len(bad), bad[:5]))
        print('  %-28s %3d refs identical' % (board, len(st.parts)))


TESTS.append(test_ref_inversions_identical_per_ref)


def test_pair_metrics_dict_identical_field_for_field():
    """The whole dict, including `lis`, `nets` and #891's `ties`.

    `ref_inversions` reads only `inversions`, so an equality check through it
    alone would not notice the hoist corrupting a field some OTHER consumer
    reads -- `placement_score.pin_order_crossings` reads `inversions`, but
    `pair_length` and the review sheet read the rest.
    """
    from placement import pair_order as PO
    st = _state('tigard.kicad_pcb')
    refs = sorted(st.parts)
    checked = 0
    for i, a in enumerate(refs):
        for b in refs[i + 1:]:
            with_pads = PO.pair_metrics(
                st, a, b, pads_a=PO.part_pad_globals(st.parts[a]),
                pads_b=PO.part_pad_globals(st.parts[b]))
            without = PO.pair_metrics(st, a, b)
            assert with_pads == without, (
                'tigard %s~%s: hoisted pads changed the metrics: %r != %r'
                % (a, b, with_pads, without))
            if without is not None:
                checked += 1
    assert checked >= 20, (
        'only %d scoring pairs on tigard -- this assertion is close to vacuous;'
        ' pick a board with more shared-net pairs' % checked)
    print('  tigard: %d scoring pairs identical field for field' % checked)


TESTS.append(test_pair_metrics_dict_identical_field_for_field)


def test_hoisted_pads_at_a_hypothetical_pose():
    """The hoist must also hold at a pose the part is NOT at.

    This is the case the search actually uses -- `ref_inversions(state, ref, x,
    y, rot)` -- and the one where a stale pad cache would show up. Rotating by
    90 degrees is the move #893 is about.
    """
    from placement import pair_order as PO
    st = _state('esp_prog.kicad_pcb')
    moved = 0
    for ref in sorted(st.parts):
        part = st.parts[ref]
        rot = (part.rot + 90.0) % 360.0
        got = PO.ref_inversions(st, ref, part.x, part.y, rot)
        # Recompute by actually moving the part, then restore.
        x0, y0, r0 = part.x, part.y, part.rot
        st.apply_move(ref, part.x, part.y, rot)
        try:
            want = _reference_ref_inversions(st, ref)
        finally:
            st.apply_move(ref, x0, y0, r0)
        assert got == want, (
            'esp_prog %s at rot %g: hypothetical-pose inversions %r != %r '
            'measured after actually applying the move'
            % (ref, rot, got, want))
        if got:
            moved += 1
    assert moved >= 1, (
        'no part on esp_prog reports a non-zero inversion count at rot+90, so '
        'this test compared zeroes and asserted nothing')
    print('  esp_prog: rot+90 hypothetical poses match applied poses '
          '(%d non-zero)' % moved)


TESTS.append(test_hoisted_pads_at_a_hypothetical_pose)


def test_caches_are_not_shared_between_states():
    """Two states over different boards must not see each other's caches."""
    from placement import pair_order as PO
    a = _state('esp_prog.kicad_pcb')
    b = _state('tigard.kicad_pcb')
    ids_a = PO._scoring_net_ids(a)
    ids_b = PO._scoring_net_ids(b)
    assert ids_a is not ids_b, 'the scoring-net cache is shared between states'
    assert PO._scoring_net_ids(a) is ids_a, 'the cache is not sticky'
    print('  caches are per-state and sticky')


TESTS.append(test_caches_are_not_shared_between_states)


def main():
    failures = 0
    for fn in TESTS:
        print('%s ...' % fn.__name__)
        try:
            fn()
        except AssertionError as exc:
            failures += 1
            print('FAIL %s: %s' % (fn.__name__, exc))
    if failures:
        print('%d/%d checks FAILED' % (failures, len(TESTS)))
        return 1
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
