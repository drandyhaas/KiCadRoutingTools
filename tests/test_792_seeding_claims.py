"""The committed change detector for `tests/measure_792_seeding.py`'s rows.

The measurement itself runs three full seeds per board and takes minutes, so it
is not collected. This file reads the rows it committed and RE-DERIVES every
aggregate the PR body quotes -- it does not sample, and it does not compare a
sampled cell against a constant. `test_554_reach_regen`'s recorded scar is that
an earlier version of exactly this shape sampled two cells and checked the other
twenty-two against the baseline, so five of its seven checks were tautologies
and it printed PASS while the sweep's headline halved.

WHY THE ROWS EXIST AT ALL, rather than a `test_placement_ab.py` row.
That harness's `_run` calls `quench(...)`; nothing in the table calls
`seed_from_intent`, and `_intent_for` emits without `derive_decaps` so every
existing row's intent carries `decaps: {}` and no decap rule can arm. Hosting a
seed row means a new arm kind inside a gate seven pinned rows depend on -- a
change to the gate, made in the PR whose evidence the gate would produce. So the
properties that gate enforces -- pairing, >= 3 distinct boards, a direction
rather than an absolute -- are enforced HERE instead, and the harness is left
alone.

One caveat this file states rather than hides: the `off` arm carries NO
`decaps` key, so `off` vs `on` measures what DECLARING the key costs, not what
#792 changed. That pair is therefore RECORDED and not gated. The base-vs-head
comparison that does measure #792 needs two worktrees and is quoted in the PR
body.
"""
import json
import os
import sys

import run_utils  # noqa: E402

RUN_ALL_FAST_OK = True

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
ROWS = os.path.join(TESTS_DIR, '792_seeding_rows.jsonl')

#: The same floor `test_placement_ab.gate()` applies: a direction measured on
#: fewer than three distinct boards is a coin flip with a press release.
MIN_BOARDS = 3


def _rows():
    if not os.path.exists(ROWS):
        raise AssertionError(
            f"{ROWS} is missing. Regenerate with "
            f"`python3 tests/measure_792_seeding.py`. A missing artefact is "
            f"not a pass: the numbers this file checks would simply not be "
            f"checked.")
    out = []
    with open(ROWS, encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    assert out, ROWS
    return out


def _armed(rows):
    return [r for r in rows
            if r.get('kind') != 'provenance'
            and r.get('armed') and 'error' not in r]


def _provenance(rows):
    for r in rows:
        if r.get('kind') == 'provenance':
            return r
    raise AssertionError(
        f"{ROWS} carries no provenance row. Regenerate with "
        f"`python3 tests/measure_792_seeding.py` -- a rows file that cannot "
        f"say which tree produced it cannot be checked against one.")


def test_the_rows_describe_THIS_tree_and_this_board_set():
    """Review's sharpest finding about this file: EVERY arm passed with stage
    2.5 deleted outright, because the arms read the committed rows and the rows
    were generated before the deletion. A rows file that cannot be tied to a
    tree is a constant wearing a measurement's clothes.

    So the file records the head sha and a digest of the board set, and this
    arm refuses one that does not match. A legitimately stale file fails here
    with the regeneration command, rather than silently blessing whatever the
    engine now does.
    """
    import hashlib
    import subprocess
    prov = _provenance(_rows())
    paths = run_utils.corpus_boards()
    assert paths, 'corpus_boards() returned nothing; cannot verify provenance'
    digest = hashlib.sha256(
        '\n'.join(sorted(os.path.basename(x) for x in paths))
        .encode()).hexdigest()[:16]
    assert prov['board_digest'] == digest, (
        f"the rows were measured over a DIFFERENT board set "
        f"({prov['boards']} boards, digest {prov['board_digest']}) than this "
        f"tree has ({len(paths)}, {digest}). Re-run "
        f"`python3 tests/measure_792_seeding.py`.")
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                          capture_output=True, text=True).stdout.strip()
    if head and prov.get('head') and head != prov['head']:
        # NOT fatal: the rows legitimately outlive commits that cannot move
        # them (docs, other tests). The LIVE arm below is what catches a rows
        # file that no longer describes the engine.
        print(f"  note: rows measured at {prov['head'][:12]}, tree is at "
              f"{head[:12]} -- the live arm below is the real check")
    print(f"  PASS: rows cover {prov['boards']} board(s), digest matches this "
          f"tree's corpus")


def test_one_board_is_RE_MEASURED_live_so_a_stale_file_cannot_pass():
    """The arm that makes the others mean something.

    Every other arm here reads the file. This one re-runs the smallest armed
    board through the real `seed_from_intent` and requires the committed row to
    match -- so deleting stage 2.5, or changing what it claims, fails HERE even
    though the JSONL still says what it always said.

    esp_prog is chosen for being cheap (20 footprints) and armed.
    """
    import random
    for _d in ('py_placer', 'py_router', 'py_tools'):
        _p = os.path.join(ROOT, _d)
        if _p not in sys.path:
            sys.path.insert(0, _p)
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan as fp
    from placement import seeder

    rows = {r['board']: r for r in _armed(_rows())}
    name = next((n for n in ('esp_prog', 'lvds_converter_dualclk', 'tigard')
                 if n in rows), None)
    if name is None:
        print("  SKIP: none of the cheap boards is armed in the rows")
        return
    path = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
    if not os.path.exists(path):
        print(f"  SKIP: {name} absent")
        return
    doc = fp.emit_intent(parse_kicad_pcb(path), path, derive_decaps=True)
    res = seeder.seed_from_intent(
        parse_kicad_pcb(path), path, fp.intent_from_dict(doc, path),
        random.Random(11), group_sources=('kicad', 'sheet'))
    live = {p['reference']: [round(p['new_x'], 4), round(p['new_y'], 4),
                             round(p['new_rotation'], 3)]
            for p in res['placements']}
    recorded = rows[name]['on']['poses']
    moved = sorted(r for r in set(live) | set(recorded)
                   if live.get(r) != recorded.get(r))
    assert not moved, (
        f"{name}: the live seed disagrees with the committed row on "
        f"{len(moved)} part(s) ({moved[:6]}). Either the engine changed and "
        f"the rows are stale -- re-run tests/measure_792_seeding.py -- or the "
        f"rows describe a tree this one is not.")
    print(f"  PASS: {name} re-seeded live and matches its committed row on "
          f"{len(live)} part(s), so a stale JSONL cannot pass this file")


def test_the_rows_cover_the_tracked_corpus_and_say_what_they_skipped():
    rows = [r for r in _rows() if r.get('kind') != 'provenance']
    armed = _armed(rows)
    inert = [r['board'] for r in rows if not r.get('armed')]
    errs = [(r['board'], r['error']) for r in rows if 'error' in r]
    assert not errs, errs
    assert len(rows) >= 15, len(rows)
    assert len(armed) >= MIN_BOARDS, (len(armed), "fewer than three boards "
                                                  "arm the stage, so no "
                                                  "direction is measurable")
    # A board that cannot arm is RECORDED, not dropped -- "measured and inert"
    # and "never measured" must not look the same.
    for r in rows:
        if not r.get('armed'):
            assert r.get('reason'), r
    print(f"  PASS: {len(rows)} board(s) -- {len(armed)} arm the pin stage, "
          f"{len(inert)} recorded as inert ({', '.join(inert[:4])}...)")


def test_the_control_arm_is_never_what_changed():
    """Every claim below is a comparison BETWEEN arms of the same run, so the
    control's only job is to exist. Its poses are recorded so a later reader
    can diff them against a fresh run and see whether the seeder moved
    underneath the rows."""
    for r in _armed(_rows()):
        assert r['off']['poses'], r['board']
        assert r['off']['claimed'] == 0, (r['board'], r['off']['claimed'])
        assert r['off']['put_back'] == 0, (r['board'], r['off']['put_back'])
    print("  PASS: the control arm claims nothing and puts nothing back on "
          "every armed board")


def test_the_SHIPPING_arm_leaves_no_part_unseated_that_the_control_seated():
    """The put-back falls through to the centroid stage rather than appending
    to `unseated`, so it cannot strand a part. Re-derived per board.

    Scoped to the arm that SHIPS. The `chips` arm is excluded here and asserted
    separately below, because it does strand one -- which is a finding, not a
    reason to weaken this claim."""
    rows = _armed(_rows())
    for r in rows:
        extra = sorted(set(r['on']['unseated']) - set(r['off']['unseated']))
        assert not extra, (r['board'], extra)
    print(f"  PASS: {len(rows)} board(s), no part newly unseated by the "
          f"narrowing or the put-back")


def test_the_owner_test_arm_STRANDS_a_part_and_that_is_why_it_ships_off():
    """The measurement earning its keep.

    `decap_owner_chips` widens the pin-owner set from `U*` to the grouper's
    chip set, which gains pin sources on seven boards and loses none -- so the
    COVERAGE argument for it is clean. Seeding is not coverage. Measured, the
    widened arm strands FOUR parts across THREE boards that the control seats:
    `orangecrab_ext_pll` U4, `rp2350_fpga_eensy_prePlane` L1, and `tigard` H1
    and H3. (My own first draft of this docstring said 'one part on one
    board' -- understating the harm, which is the direction my errors lean.)
    A stranded part is a worse outcome than every gain the flag buys, and
    that is the whole reason it ships OFF rather than on.

    Asserted rather than noted, so the flag cannot be flipped on without this
    arm being confronted -- and so that if a later change fixes the stranding,
    this arm fails and the flag's justification is revisited deliberately.
    """
    rows = _armed(_rows())
    stranded = {}
    for r in rows:
        extra = sorted(set(r['chips']['unseated']) - set(r['off']['unseated']))
        if extra:
            stranded[r['board']] = extra
    assert stranded, (
        "the chips arm no longer strands anything. That is good news and it "
        "invalidates this arm's premise: re-run measure_792_seeding.py, "
        "re-read the flag's default, and update BOTH deliberately")
    assert 'orangecrab_ext_pll' in stranded, sorted(stranded)
    # The COUNT, not just the presence: a later change that strands three
    # boards instead of one must not read as the same finding.
    assert len(stranded) == 3, sorted(stranded)
    assert sum(len(v) for v in stranded.values()) == 4, stranded
    print(f"  PASS: the owner-test arm strands {stranded} -- measured, and the "
          f"reason `decap_owner_chips` defaults to False")


def test_what_DECLARING_the_key_costs_is_recorded_not_gated():
    """`off` carries no `decaps` key, so stage 2.5 does not run in it at all.
    This pair therefore measures what declaring `max_distance_mm` costs -- a
    property of #704's feature -- and NOT what #792 changed.

    The first draft of this file gated on it, and read a 32% rp2350 regression
    as a defect in this change. rp2350 has ZERO orphans, so the narrowing
    cannot touch it; the whole delta is the pin stage claiming 15 caps that
    nothing claimed before, at pin-cluster centroids, in a currency that is not
    pad-edge distance. That is worth DISCLOSING and is not this PR's to fix --
    so it is printed, with its direction, and not asserted.

    The base-vs-head comparison that does measure #792 needs two worktrees and
    is quoted in the PR body: every control arm byte-identical, no part newly
    unseated, ulx3s 181 parts moved with 19 put-backs.
    """
    rows = [r for r in _armed(_rows())
            if r['off']['pin_gap_sum'] is not None
            and r['on']['pin_gap_sum'] is not None]
    assert len(rows) >= MIN_BOARDS, len(rows)
    worse, better, same = [], [], []
    for r in rows:
        a, b = r['off']['pin_gap_sum'], r['on']['pin_gap_sum']
        bucket = (worse if b > a + 1e-6 else better if b < a - 1e-6 else same)
        bucket.append((r['board'], round(a, 1), round(b, 1)))
    # ANTI-VACUITY: if declaring the key moved NOTHING anywhere, these rows
    # are measuring an inert feature and every other arm here is decoration.
    assert better or worse, "declaring decaps moved no board's pin geometry"
    print(f"  PASS (recorded, not gated): declaring decaps.max_distance_mm "
          f"improves pin geometry on {len(better)}, worsens it on "
          f"{len(worse)}, leaves {len(same)} unchanged")
    for b, a2, b2 in sorted(worse, key=lambda t: t[1] - t[2]):
        print(f"      WORSE  {b:26} {a2} -> {b2}")
    for b, a2, b2 in sorted(better, key=lambda t: t[2] - t[1])[:3]:
        print(f"      better {b:26} {a2} -> {b2}")


def test_the_put_back_actually_fires_somewhere():
    """ANTI-VACUITY. Every claim above is compatible with a change that does
    nothing at all, so at least one board must show the put-back doing work --
    otherwise these rows are measuring an inert edit."""
    rows = _armed(_rows())
    fired = [(r['board'], r['on']['put_back']) for r in rows
             if r['on']['put_back']]
    assert fired, ("no board puts a cap back; the rows cannot distinguish this "
                   "change from no change")
    total = sum(n for _b, n in fired)
    print(f"  PASS: {len(fired)} board(s) put caps back, {total} in total "
          f"({', '.join(f'{b}:{n}' for b, n in fired[:4])})")


TESTS = [
    test_the_rows_describe_THIS_tree_and_this_board_set,
    test_one_board_is_RE_MEASURED_live_so_a_stale_file_cannot_pass,
    test_the_rows_cover_the_tracked_corpus_and_say_what_they_skipped,
    test_the_control_arm_is_never_what_changed,
    test_the_SHIPPING_arm_leaves_no_part_unseated_that_the_control_seated,
    test_the_owner_test_arm_STRANDS_a_part_and_that_is_why_it_ships_off,
    test_what_DECLARING_the_key_costs_is_recorded_not_gated,
    test_the_put_back_actually_fires_somewhere,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
