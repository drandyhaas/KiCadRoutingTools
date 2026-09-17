#!/usr/bin/env python3
"""The #703 rig, checked by REGENERATING a declared subset (not by shipping 87 rows).

WHY THIS REPLACED A COMMITTED ROWS FILE

The study's 87 rows were committed once, at 276 KB, so `docs/placement-predictors.md`
could not drift from them. drandyhaas reviewed that and asked for the file to go:
the results are regenerable, and he checked it rather than trusting the contract --
`--task esp_prog:authored` rebuilt a row in 4.4 s and every predictor,
`truth.blocking`, `truth.quality`, `argv_sha` and `poses_sha256` matched.

He is right, and the trade is worth stating plainly because it is a real cost.
With the rows gone, the doc's rho table has no automated change detector; it is
the recorded finding, and re-deriving it is an **8.8 hour** job (the sum of
`provenance.total_seconds` over the 87 rows; median 217 s, max 2012 s on
`tigard:perturb-wrong_side`). What is kept instead is a detector on the RIG:
four cheap variants, regenerated on demand, whose predictors and routed truth
are pinned here as literals.

That catches the failure that actually matters day to day -- a change to the
placement engine, the predictor extraction, or the route argv silently moving
the numbers the study measured -- without shipping the artifact.

WHAT IS COMPARED, AND WHAT DELIBERATELY IS NOT

  * `poses_sha256`, NOT the input board's raw bytes. The withdrawn rows file
    stamped `sha256(board file)`, and all four of its board hashes were CRLF
    hashes -- so the gate that checked them was green only on a Windows
    checkout and red on macOS for 3 of 4 boards. `poses_sha256` is computed
    from PARSED footprint poses and is identical either way. (Found in the same
    review; same family as the pre-existing `test_763_kicad_locate` failure,
    pointing the other way.)
  * `routed_board_sha` is NEVER compared. KiCad stamps fresh UUIDs into every
    written board, and CLAUDE.md says so in as many words: "outputs carry
    per-run random UUIDs ... never hash or whole-file-diff `.kicad_pcb` outputs
    to judge determinism". Compare the graded counts instead, which is what
    `truth` is.
  * `seconds` is recorded as measured and never asserted on -- it is a property
    of the machine, and this repo's own doctrine is that wall-clock breaks
    cross-machine determinism.

This test ROUTES, so it is an integration test: about 75 s for the four rows.

    python3 -X utf8 tests/test_703_predictor_regen.py
    python3 -X utf8 tests/test_703_predictor_regen.py --row esp_prog:authored
    python3 -X utf8 tests/stress/predictor_study.py --verify-row esp_prog:authored
"""
import argparse
import os
import shutil
import sys
import tempfile

RUN_ALL_TIMEOUT = 1200

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

#: The declared subset, and the values a correct rig reproduces.
#:
#: Four variants spanning the three generators -- the authored board, a
#: perturbed one, a quench candidate, and a second board -- chosen for cost:
#: every one routes in well under a minute. They are NOT a sample of anything
#: and no statistic is computed from them; they are a change detector.
#:
#: MEASURED, from the run recorded in the pull request. Never predicted.
#:
#: RE-RECORDED 2026-09-16 (the #958 fine-pitch tie guard, `32167508`). The
#: three esp_prog rows moved in ONE column and in ONE direction: `segments`
#: ROSE everywhere while `vias` is identical on all three and `copper_mm` is
#: identical on two. That is the EXACT MIRROR of the 2026-09-14 #958 re-record
#: below, where segments fell everywhere at identical vias and copper -- and it
#: is the intended effect: the guard refuses #958's equal-length collapses
#: inside a fine-pitch pad field, so staircases there keep their corners. Same
#: length, more legs.
#:
#:   esp_prog:authored            segs 250 -> 260   (vias 34, copper 336.58 both)
#:   esp_prog:perturb-scatter-d1  segs 296 -> 302   (vias 38, copper 361.26 both)
#:   esp_prog:portfolio-1         segs 282 -> 293, copper 362.75 -> 362.65
#:   splitflap_driver:authored    unmoved
#:
#: BISECTED, not assumed: the row passes at `549f16b5`, the guard's immediate
#: parent, and fails at `32167508`. splitflap staying put is consistent -- the
#: guard only fires where a tie's new legs cross a <=0.8mm-pitch pad field.
#:
#: portfolio-1 is again the only row whose copper moved, by -0.10mm (-0.03%),
#: and again it is the row the 2026-09-15 note calls pathological by
#: construction (the quench candidate that lands U2's tab ON Q1's pads).
#:
#: `truth.headline` did not mismatch on ANY row, so nothing became unrouted or
#: broken. MORE segments at equal copper and equal vias is a LOSS on this
#: repo's own tie-break, and it is the price the guard charges: measured on
#: ft2232h_jtag it buys back a whole net (1/34 -> 0/34 incomplete) and leaves
#: ottercast_audio unchanged at 2/158, where the previous, reverted cut had made
#: it 4/158. That trade is the commit's argument; this row is where its cost
#: shows.
#:
#: RE-RECORDED 2026-09-15 (#908, the own-pad lift reaching Phase 3). The three
#: `esp_prog` rows moved; `splitflap_driver:authored` did NOT. That split is the
#: evidence for the cause rather than a story about it: esp_prog carries EIGHT
#: footprint-copper segments (U2's SOT89 tab, the #908 population) and splitflap
#: carries none, so a change to how footprint copper is modelled can only reach
#: one of the two boards -- and only one moved.
#:
#: BISECTED over the #908 chain, one detached worktree per commit, running
#: `--row esp_prog:portfolio-1` and parsing the LEFT of `got != want` (see the
#: bisect note below -- the same trap was waiting):
#:
#:   f0838d2d  matches      (the last green commit)
#:   b0016cc3  matches      #908 partial
#:   5c00f277  matches      own-pad lift covers the via keep-out
#:   ddd55b3c  matches      the lift could never remove anything (#422 static)
#:   9272d942  35 -> 36 vias, 362.75 -> 364.33  <-- THE COMMIT
#:   07412c06  36 -> 35 vias, 364.33 -> 362.75  (settles to the recorded value)
#:
#: `9272d942` is "the bake marker is per-MAP, PHASE 3 GETS THE LIFT, and a gate
#: that pins it". Phase 3 is tap routing, which is where a multi-point net
#: places its vias -- so a lift that newly applies there is exactly a change to
#: the via count. `07412c06` (the short gate counted the tie as foreign) then
#: nudges it back. Both are #908 doing MORE of what #908 is for.
#:
#:   esp_prog:authored            34/344.06/254 -> 34/336.58/250   better
#:   esp_prog:perturb-scatter-d1  37/363.0/292  -> 38/361.26/296   mixed (+1 via)
#:   esp_prog:portfolio-1         30/350.67/269 -> 35/362.75/282   WORSE (+5 vias)
#:   splitflap_driver:authored    unmoved
#:
#: The direction is MIXED and one row is frankly worse, which is recorded here
#: rather than rounded off -- there is precedent in the 2026-08-30 note below
#: for a mixed re-record, and the same reasoning applies. `truth.headline` did
#: not mismatch on ANY row (portfolio-1 stays at 1, and that 1 is a `drc` item,
#: not `unrouted`/`broken`), so nothing became disconnected; the movement is in
#: the quality tie-break only. portfolio-1 is the pathological row by
#: construction -- the quench candidate that lands U2's SOT89 tab ON Q1's pads,
#: as its own comment below already says -- so it is the row where freeing the
#: tab's own copper has the most room to reroute, for better or worse.
#:
#: Re-recorded rather than treated as a defect because the lift is CORRECT: a
#: footprint's own copper must not cage its own pads, on the tap path as much as
#: anywhere else. If a later corpus A/B shows Phase 3 taps getting worse broadly
#: -- not on one hand-perturbed candidate -- that is the signal to revisit, and
#: this row is where it would show first.
#:
#: RE-RECORDED 2026-09-14 (#958, the smoother's second phase). `truth.quality`
#: moved on all four rows, and in ONE column: `segments` fell everywhere while
#: `vias` is identical on all four and `copper_mm` is identical on three. That
#: signature is a re-SEGMENTATION, not a different route, and it was BISECTED
#: rather than assumed -- `git bisect run` over the 146 commits since the last
#: re-record lands on `2fe97a30` ("#958: the smoother prefers fewer segments at
#: equal length, in a second greedy phase"), whose own commit message records
#: the same fingerprint on splitflap's signal step (1149 -> 968 segments, 96
#: vias in both, copper 2477.79 -> 2477.10 mm).
#:
#:   esp_prog:authored            segs 286 -> 254   (vias 34, copper 344.06 both)
#:   esp_prog:perturb-scatter-d1  segs 311 -> 292   (vias 37, copper 363.0 both)
#:   esp_prog:portfolio-1         segs 304 -> 269   (vias 30, copper 350.67 both)
#:   splitflap_driver:authored    segs 1422 -> 1155, copper 2915.15 -> 2913.82
#:
#: splitflap is the only row whose copper moved, by -1.33 mm (-0.05%), and that
#: is the SAME second-order effect the 2026-08-30 note below already records for
#: the collinear merge: re-segmentation changes rip/restore granularity for the
#: later steps of a multi-step chain, so end-to-end copper shifts a hair. The
#: three esp_prog rows are single-step and their copper is unmoved to the
#: hundredth, which is what makes that reading the one to believe.
#:
#: `truth.blocking` is 0 on all four, before and after -- no net became unrouted
#: or broken. FEWER segments at equal copper and equal vias is an improvement on
#: this repo's own tie-break, so this re-record is not papering over a loss.
#:
#: A NOTE ON BISECTING THIS FILE. The first attempt used a shell predicate
#: grepping the mismatch line for the recorded segment count -- but that line is
#: `got != want`, so the pattern matched the WANT side and every commit read as
#: good, converging on HEAD (an unrelated one-line project-writeback commit).
#: A predicate for this test must parse the left of `!=`, and must exit 125 on a
#: checkout that produced no comparison line at all, or an untestable commit is
#: silently scored as a passing one.
#:
#: RE-RECORDED 2026-09-03 (#530 decision 3). `truth.quality` moved on the three
#: esp_prog rows and the cause is named: a pad clearance OVERRIDE now REPLACES
#: the class value (KiCad 10, measured by tests/oracle/constraint_agreement.py)
#: instead of max()-ing with it, and esp_prog carries 13 pads at a 0.0508 mm
#: override -- so the router prices those pads as KiCad does and lays different
#: copper past them. `truth.blocking` is 0 on all three, as before; it had read
#: 49/25/34 for one commit because the writeback then stamped rules.min_clearance
#: 0.2193 into the project and KiCad floors an override at min_clearance (the
#: writeback now caps min_clearance at the smallest honoured override).
#:
#:   esp_prog:authored            vias 31 -> 33, copper 336.35 -> 345.52, segs 236 -> 373
#:   esp_prog:perturb-scatter-d1  vias 38 -> 29, copper 377.83 -> 341.07, segs 286 -> 288
#:   esp_prog:portfolio-1         vias 37 -> 32, copper 319.41 -> 347.03, segs 260 -> 270
#:   splitflap_driver:authored    -- unchanged (no overrides on that board)
#:
#: RE-RECORDED 2026-08-30 (second time this day). `truth.quality` moved on TWO
#: of the four rows, and the cause is named rather than assumed: #816's
#: `GRID_TIE_EPS`, which resolves a keep-out boundary cell OPEN instead of
#: letting float rounding decide it. esp_prog's geometry sits on such a tie
#: (track 0.2/2 + clearance 0.2 = 0.3 = 3 x a 0.1 grid), so the router gets a
#: slightly different -- and now position-INDEPENDENT -- set of legal cells.
#:
#:   esp_prog:authored     vias 28 -> 31, copper 320.71 -> 336.35, segs 229 -> 236
#:   esp_prog:portfolio-1  vias 30 -> 28, copper 303.26 -> 299.93, segs 252 -> 231
#:   esp_prog:perturb-scatter-d1, splitflap_driver:authored  -- unchanged
#:
#: The direction is MIXED (one worse, one better, two flat) and `truth.blocking`
#: is unchanged on all four -- no net became unrouted or broken. That is what a
#: boundary-convention change should look like: it perturbs which cells are
#: legal without making the board harder to route. Recorded here rather than
#: smoothed over, because "quality moved both ways by a few vias" is the finding.
#:
#: PREVIOUS RE-RECORD, same day:
#: RE-RECORDED 2026-08-30. `truth.quality` moved on all four rows; both causes
#: were BISECTED before re-recording, because a baseline re-recorded without a
#: named cause hides whatever moved it:
#:
#:   * vias / copper_mm -- `99196134` (#805, "commit the #189 unblock via to
#:     pcb_data when it is kept"). The unblock barrel was never appended to
#:     pcb_data, so for the whole window before its deferred multipoint result
#:     committed it blocked nobody; committing it makes it block, which changes
#:     what routes. esp_prog:authored 30 vias / 342.83 mm -> 28 / 320.71, i.e.
#:     FEWER vias and LESS copper -- better on this repo's own tie-break.
#:   * segments -- `661c88b3` (#811, the collinear merge). It is exactly
#:     copper-preserving in a single route call (measured bit-identical on two
#:     boards), so on its own it moves ONLY this column.
#:
#: One second-order effect is worth recording rather than rounding away: the
#: merge changes how copper is SEGMENTED, which changes rip/restore granularity
#: for later steps of a multi-step chain, so end-to-end copper can shift a
#: hair. Isolated on esp_prog:authored with only KICAD_MERGE_COLLINEAR toggled:
#: 320.69 -> 320.71 mm (+0.006%), segments 241 -> 229, vias unchanged. That is
#: why the copper column moved on rows where the merge is the only new input.
#: RE-RECORDED for #726 (the parser stops losing a footprint block whose
#: reference another block already claims). esp_prog carries TWO blocks named
#: `Ref*` and only the second was parsed, so this board measured as 20 parts
#: where the file has 21. All THREE esp_prog rows moved and
#: `splitflap_driver:authored` did not -- which is the signature to check
#: before believing any of this: splitflap has no duplicate reference, so a
#: change that moved it too would be a different change.
#:
#: `provenance.poses_sha256` moves on all three by construction: the pose set
#: it hashes gained a part. Two of the deltas are worth reading rather than
#: rounding away:
#:
#:   * `esp_prog:authored` -- `pad_clearance_pairs` 0 -> 1. The #697 census now
#:     reaches a part it was structurally blind to; that pair was always there.
#:     `truth` is unchanged (headline 0, vias 31, copper 336.35, segs 236), so
#:     the ROUTE is identical and only the census sees more.
#:   * `esp_prog:portfolio-1` -- `truth.headline` 3 -> 0. The quench candidate
#:     that used to route to blocking 3 now routes clean. The header's
#:     commentary about this row being "the study's headline in miniature"
#:     (23 crossings against the authored board's 53, routing WORSE) no longer
#:     holds for the blocking half, and is left standing above with this
#:     correction rather than quietly edited.
#:
#: RE-RECORDED 2026-09-10 (#908). Footprint copper -- `fp_poly` on a copper
#: layer inside a `(footprint ...)` block -- is now parsed, modelled as an
#: obstacle and graded by check_drc; before this it was invisible to all
#: three, AND the writer relocated it to silkscreen on every write. esp_prog
#: carries one: the drawn tab of U2's SOT89. All three esp_prog rows move, and
#: the two kinds of movement are worth separating:
#:
#:   * `truth.quality` on all three -- the router now has an obstacle it did
#:     not have, so it lays different copper past U2. authored
#:     39/353.93/256 -> 34/344.06/286; perturb-scatter-d1
#:     32/327.31/282 -> 37/363.0/311; portfolio-1 31/341.99/263 ->
#:     30/350.67/304. Stable across repeated runs.
#:   * `esp_prog:portfolio-1` -- `truth.headline` 0 -> 1, and it is NOT a
#:     routing failure: `blocking_by` reads unrouted 0, broken 0, **drc 1**.
#:     The quench candidate moves U2 so its tab lands ON Q1's pads, and
#:     check_drc now says so: "Pad:/DTR (Q1.2) <-> Seg:net_0 [Polygon(U2)],
#:     overlap 0.101mm". kicad-cli 10.0.0 independently reports the same
#:     candidate as `shorting_items` against Q1 pad 2 AND pad 3. So the row
#:     did not get worse -- a real defect in that placement stopped being
#:     invisible. The header's "this row is the study's headline in miniature"
#:     commentary is left standing above, with this as its second correction.
#:
EXPECTED = {
    'esp_prog:authored': dict(
        poses_sha256='67a9712d200814442b4a25cf1fa8ccd075c1968d5b08c0ad58c4662f0a479da7',
        argv_sha='52aaeed47e14fea796be12c36db09c605a4b8ec588da12bded6a2573b1c7f0b0',
        seconds=8.3,
        truth={'headline': 0,
               # 2026-09-03 (#530 auto/fab defaults): 33/345.52/373 -> 39/353.93/256
               # 2026-09-10 (#908 footprint copper): 39/353.93/256 -> 34/344.06/286
               # 2026-09-14 (#958 phase 2): segs 286 -> 254; vias/copper unmoved
               # 2026-09-15 (#908 Phase 3 lift): 34/344.06/254 -> 34/336.58/250
               # 2026-09-16 (#958 fine-pitch tie guard): segs 250 -> 260
               'quality': {'vias': 34, 'copper_mm': 336.58, 'segments': 260}},
        predictors={
            'crossings': 53, 'hpwl': 253.98092000000003,
            'halo': 127.48707486477095, 'overlap_area': 1.1400451712000104,
            'pad_copper': 0, 'pad_clearance_pairs': 1,
            'edge': 16.612682999999876, 'total': 916.1746544447492,
            'oob_count': 0,
        }),
    'esp_prog:perturb-scatter-d1': dict(
        poses_sha256='ce4d5a3803cfcf56d4ab3cfbf4f1be3ffd61242a1be78ef4ce2cae110dc96eca',
        argv_sha='52aaeed47e14fea796be12c36db09c605a4b8ec588da12bded6a2573b1c7f0b0',
        seconds=12.6,
        truth={'headline': 0,
               # 2026-09-03 (auto/fab defaults): 29/341.07/288 -> 32/327.31/282
               # 2026-09-10 (#908 footprint copper): 32/327.31/282 -> 37/363.0/311
               # 2026-09-14 (#958 phase 2): segs 311 -> 292; vias/copper unmoved
               # 2026-09-15 (#908 Phase 3 lift): 37/363.0/292 -> 38/361.26/296
               # 2026-09-16 (#958 fine-pitch tie guard): segs 296 -> 302
               'quality': {'vias': 38, 'copper_mm': 361.26, 'segments': 302}},
        predictors={
            'crossings': 50, 'hpwl': 252.34828000000005,
            'halo': 130.46454030971682, 'overlap_area': 1.1400451712000104,
            'pad_copper': 0, 'pad_clearance_pairs': 0,
            'edge': 16.498992539999946, 'total': 887.3384166729421,
            'oob_count': 0,
        }),
    # The quench candidate, kept because it is the study's own headline in
    # miniature: this placement scores 23 crossings against the authored
    # board's 53 -- the best on the slate -- and routes to blocking 3 where the
    # authored board routes to 0.
    # RE-RECORDED AGAIN for #826 (the portfolio jitter now snaps its offset to
    # the board's lattice). Second consecutive re-record of this row, and both
    # times for the same reason: it is the detector the header describes, and
    # it fired. esp_prog is authored on 0.05mm; the jitter used to hand the
    # quench a seed board at 0.525 occupancy -- below the inference floor --
    # and now hands it one at 0.825, so this candidate's poses genuinely
    # differ. Reproduced twice.
    #
    # What did NOT move, which is why the commentary below still stands:
    # `truth.headline` is still 3 and `predictors.crossings` is still 23. The
    # routed board is marginally worse this time -- vias 23 -> 25, copper
    # 263.91 -> 264.89mm, segments 203 -> 207 -- at the same blocking. That is
    # a different placement, not a better or worse fix; the escape landed
    # somewhere else.
    #
    # `esp_prog:perturb-scatter-d1` deliberately did NOT move: `perturb.py`'s
    # scatter kind calls perturb_jitter positionally and keeps the continuous
    # sampler, so its damage model is untouched. That row regenerating
    # identically is the evidence for it.
    #
    # RE-RECORDED for #708 (the seed-relative candidate snap). This row is the
    # detector the header describes -- "a change to the placement engine ...
    # silently moving the numbers the study measured" -- and it fired, which is
    # it working. The quench now offers candidates on the board's own lattice,
    # so this candidate's poses genuinely differ; `poses_sha256`, the four
    # geometry predictors and the routed `quality` moved with them, and the
    # values below are from the run, reproduced twice.
    #
    # What did NOT move, and is why the commentary above still stands:
    # `truth.headline` is still 3 and `predictors.crossings` is still 23. The
    # routed board is in fact slightly better -- vias 28 -> 23, copper
    # 299.93 -> 263.91mm, segments 231 -> 203 -- at the same blocking.
    'esp_prog:portfolio-1': dict(
        poses_sha256='d1290d938a770bb17f2c6a4869b8224eb30aeba6bb37e11c2ae2f3c6c042695b',
        argv_sha='52aaeed47e14fea796be12c36db09c605a4b8ec588da12bded6a2573b1c7f0b0',
        seconds=12.0,
        # headline 3 -> 0 (2026-09-03) -> 1 (2026-09-10). The 1 is `drc`, NOT
        # `unrouted`/`broken`: this candidate lands U2's SOT89 tab on Q1's
        # pads, and #908 is what makes that visible. See the header note.
        truth={'headline': 1,
               # 2026-09-03 (auto/fab defaults): 32/347.03/270 -> 31/341.99/263
               # 2026-09-10 (#908 footprint copper): 31/341.99/263 -> 30/350.67/304
               # 2026-09-14 (#958 phase 2): segs 304 -> 269; vias/copper unmoved
               # 2026-09-15 (#908 Phase 3 lift): 30/350.67/269 -> 35/362.75/282
               # 2026-09-16 (#958 fine-pitch tie guard): 35/362.75/282 ->
               #   35/362.65/293 (the only row whose copper moved, -0.10mm)
               'quality': {'vias': 35, 'copper_mm': 362.65, 'segments': 293}},
        predictors={
            'crossings': 23, 'hpwl': 260.0687799999999,
            'halo': 101.01900525631262, 'overlap_area': 1.0,
            'pad_copper': 0, 'pad_clearance_pairs': 0,
            'edge': 21.49349600000009, 'total': 605.1074633433927,
            'oob_count': 0,
        }),
    # A SECOND board, so a change that only moves one board's numbers cannot
    # pass. Its argv_sha differs from esp_prog's, which is the frozen-argv
    # contract visible in the data.
    'splitflap_driver:authored': dict(
        poses_sha256='8b89746bc1a8528de6349f54dd18874ca11d8c7a1f73dd657edce65d460391d9',
        argv_sha='d32ea90c2e348bd6c1fb318983e73f953f179d1ea85aeca7ad4ed4b2bed5cbdc',
        seconds=32.6,
        truth={'headline': 0,
               # 2026-09-14 (#958 phase 2): segs 1422 -> 1155, copper
               # 2915.15 -> 2913.82 (-0.05%); vias unmoved. The only row whose
               # copper moved -- see the re-record note in the header.
               'quality': {'vias': 168, 'copper_mm': 2913.82,
                           'segments': 1155}},
        predictors={
            'crossings': 300, 'hpwl': 2504.4400000000014,
            'halo': 297.4273114820511, 'overlap_area': 1.7621459846850488e-13,
            'pad_copper': 0, 'pad_clearance_pairs': 0,
            'edge': 135.1680159999995, 'total': 5872.4776824793535,
            'oob_count': 14,
        }),
}

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


def t_the_subset_is_declared_coherently():
    """Cheap, board-only assertions that need no routing at all."""
    import predictor_study as PS
    boards = {b['key'] for b in PS.CALIBRATION_CANDIDATES}
    for key, want in sorted(EXPECTED.items()):
        bk, _, variant = key.partition(':')
        check(bk in boards, f'{key}: {bk} is a declared board')
        check(variant in PS.VARIANTS, f'{key}: {variant} is a declared variant')
        check(len(want['poses_sha256']) == 64, f'{key}: poses_sha256 is a sha')
        check(set(want['predictors']) <= set(PS.PREDICTOR_KEYS),
              f'{key}: every pinned predictor is a real one')
    check(len({k.split(':')[0] for k in EXPECTED}) >= 2,
          'the subset spans at least TWO boards, so a one-board change cannot '
          'slip through')
    shas = {v['argv_sha'] for v in EXPECTED.values()}
    per_board = {}
    for k, v in EXPECTED.items():
        per_board.setdefault(k.split(':')[0], set()).add(v['argv_sha'])
    check(all(len(s) == 1 for s in per_board.values()),
          'every variant of a board pins ONE argv_sha -- the frozen-argv '
          'contract, visible in the data')
    check(len(shas) == len(per_board),
          'and different boards pin different argv_sha')


def t_regenerates(only=None):
    """The real check: rebuild each variant and diff against the literals."""
    import predictor_study as PS
    work = tempfile.mkdtemp(prefix='regen703_')
    try:
        for key, want in sorted(EXPECTED.items()):
            if only and key != only:
                continue
            bk, _, variant = key.partition(':')
            board = next(b for b in PS.CALIBRATION_CANDIDATES
                         if b['key'] == bk)
            row = PS.run_task(bk, board['file'], variant, work, 0, 900)
            bad = PS.compare_row(row, want)
            for line in bad:
                print(f'       {line}')
            check(not bad,
                  f'{key} regenerates identically '
                  f'({row["route"].get("seconds")}s vs {want["seconds"]}s '
                  f'recorded)')
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', default=None,
                    help='regenerate ONE declared row (BOARD:VARIANT)')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        for k in sorted(EXPECTED):
            print(k)
        return 0
    if a.row and a.row not in EXPECTED:
        print(f'no such declared row: {a.row}; try --list', file=sys.stderr)
        return 2
    print('t_the_subset_is_declared_coherently:')
    t_the_subset_is_declared_coherently()
    print('t_regenerates:')
    t_regenerates(a.row)
    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_703_predictor_regen: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
