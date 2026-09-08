#!/usr/bin/env python3
"""place_pose.py: the verb that APPLIES a model-chosen pose (#892).

What this file is really testing, in one line each:

* the pose that was asked for is the pose that lands in the file;
* a request that makes the board WORSE is refused and writes NOTHING, while a
  board's inherited damage is never charged to the caller;
* `face` is not a prediction anyone has to trust -- FACE_CYCLE is checked here
  against a real write + re-parse, and the verb re-measures the row on the
  board it wrote;
* several verbs in one call are ONE arrangement (every op reads the input);
* lock and unlock are inverses, byte for byte;
* every exit -- including the refusals -- prints exactly one JSON_SUMMARY.

Refusals are asserted with `run_utils.check(..., refuse=..., code=N)` rather
than on the exit code alone, so an ImportError or an argparse accident is
reported as a BROKEN TEST instead of as a guard that held.

WHAT THE BATTERY MEASURED (`python3 -X utf8 tests/mutate_892.py`, 27 rows over
`placement/pose_ops.py`, `place_pose.py`, `placement/seeder.py`,
`placement/provenance.py` and the manifest parity gate), run in a clean
worktree at the commit that carries this docstring:

    face-cycle-reversed                                  KILLED
    the-face-row-is-keyed-by-pad-number-again            KILLED
    a-face-aim-is-claimed-rather-than-measured           KILLED
    worsened-count-arm-neutered                          KILLED
    worsened-magnitude-arm-neutered                      KILLED
    off-board-amount-stops-being-an-arm                  KILLED
    is_clean-ignores-the-magnitudes                      KILLED
    legal-goes-back-to-meaning-no_worse                  KILLED
    a-refusal-names-an-output-path-again                 KILLED
    a-forced-run-reports-only-the-last-finding           KILLED
    the-snap-ladder-loses-its-lattice-rung               KILLED
    the-radius-stops-bounding-the-distance               KILLED
    the-snapped-pose-is-not-re-staged                    KILLED
    dry-run-writes-the-board-anyway                      KILLED
    the-lock-guard-is-skipped                            KILLED
    lock-and-unlock-of-one-ref-is-allowed-again          KILLED
    an-unknown-lock-ref-is-accepted-again                KILLED
    a-failed-promote-is-not-atomic-again                 KILLED
    a-forced-run-is-not-disclosed                        KILLED
    only-the-first-op-is-written                         KILLED
    the-copper-gate-stops-refusing                       KILLED
    a-missing-input-file-is-no-longer-named              KILLED
    the-snap-knobs-are-unvalidated-again                 KILLED
    stamp_unlocked-removes-nothing                       KILLED
    stamp_unlocked-unlocks-every-namesake                KILLED
    place_pose-leaves-the-lever-registry                 KILLED
    the-parity-gate-goes-back-to-a-hand-picked-list      KILLED

    27 rows: 27 killed, 0 survived, 0 broken, 0 disagreeing with expectation

Four earlier rounds are the reason several arms here look pedantic:

  * neutering the COUNT arm of `worsened()` left every CLI assertion green,
    because the shortfall arm refused the same request -- so `worsened()` is
    checked arm by arm, not only through the CLI;
  * dropping the Euclidean `--radius` bound also left them green, because the
    case had been loosened to `--radius 8` while the overshoot it was written
    for is 5.0 mm under `--radius 4`;
  * deleting the `isfile` guard changed only the MESSAGE (the parser raises
    and the run still exits 2 with a summary), so that arm asserts the reason;
  * `a-forced-run-reports-only-the-last-finding` SURVIVED until the ROW was
    fixed: it mutated the face block's `append`, after which the legality
    block appends anyway. Only the last writer can erase what came before.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (REPO,):
    if p not in sys.path:
        sys.path.insert(0, p)
        sys.path.insert(0, os.path.join(p, 'py_router'))
        sys.path.insert(0, os.path.join(p, 'py_tools'))
        sys.path.insert(0, os.path.join(p, 'py_placer'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_utils import check as refuse_check, tool                # noqa: E402

BOARD = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
ROUTED = os.path.join(REPO, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
PRO = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pro')
FLAT = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


def run(argv, hashseed="0", timeout=900):
    env = dict(os.environ, PYTHONHASHSEED=hashseed, PYTHONIOENCODING='utf-8')
    return subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=REPO, env=env, timeout=timeout)


def summaries(r):
    return [json.loads(l.split(':', 1)[1]) for l in r.stdout.splitlines()
            if l.startswith('JSON_SUMMARY:')]


def summary(r):
    s = summaries(r)
    assert len(s) == 1, f"expected exactly one JSON_SUMMARY, got {len(s)}"
    return s[0]


for _f in (BOARD, ROUTED, PRO):
    if not os.path.isfile(_f):
        print("SKIP: fixture missing: %s" % _f)
        sys.exit(77)

POSE = tool('place_pose.py')

from kicad_parser import parse_kicad_pcb                          # noqa: E402
from placement import pose_ops                                    # noqa: E402
from placement.parser import extract_locked_refs                  # noqa: E402
from placement.seeder import stamp_locked, stamp_unlocked         # noqa: E402
from placement.writer import write_placed_output                  # noqa: E402

pcb0 = parse_kicad_pcb(BOARD)
CLR, EDGE, TW, _knobs = pose_ops.resolve_knobs(BOARD)

# ---------------------------------------------------------------------------
print("FACE_CYCLE against a real rotation (the arithmetic nobody can eyeball)")
# The cycle is a claim about the parser's own transform, so it is checked by
# ROTATING A REAL PART and re-reading the engine's face rule -- not by
# restating the constant. Per-pad agreement is not required: `escape.face_of`
# takes an argmin against a box that is not square, so a CORNER pad can change
# sides under a rotation that carries the row. The claim under test is the one
# the verb makes, which is about the ROW's majority.
base = pose_ops.part_faces(pcb0, 'U1', clearance=CLR, track_width=TW)
base_face = {p.pad_number: f for f, pads in base.items() for p in pads}
fp0 = pcb0.footprints['U1']
with tempfile.TemporaryDirectory() as d:
    for delta in (90, 180, 270):
        out = os.path.join(d, 'r%d.kicad_pcb' % delta)
        write_placed_output(BOARD, out, [
            {'reference': 'U1', 'new_x': fp0.x, 'new_y': fp0.y,
             'new_rotation': (fp0.rotation + delta) % 360}])
        got = pose_ops.part_faces(parse_kicad_pcb(out), 'U1',
                                  clearance=CLR, track_width=TW)
        got_face = {p.pad_number: f for f, pads in got.items() for p in pads}
        worst = None
        for face, pads in base.items():
            if face == 'interior' or len(pads) < 2:
                continue
            want = pose_ops.rotate_face(face, delta)
            hit = sum(1 for p in pads if got_face.get(p.pad_number) == want)
            frac = hit / float(len(pads))
            worst = frac if worst is None else min(worst, frac)
        check("delta %d: every multi-pad row keeps its predicted face by "
              "majority" % delta, worst is not None and worst > 0.5,
              "worst row agreement %s" % worst)

check("rotate_face is a 4-cycle", all(
    pose_ops.rotate_face(f, 360) == f for f in pose_ops.FACE_CYCLE))
check("face_delta inverts rotate_face", all(
    pose_ops.rotate_face(f, pose_ops.face_delta(f, g)) == g
    for f in pose_ops.FACE_CYCLE for g in pose_ops.FACE_CYCLE))
check("bearing_face reads y-down as north/south",
      pose_ops.bearing_face((0, 0), (0, -5)) == 'north'
      and pose_ops.bearing_face((0, 0), (0, 5)) == 'south'
      and pose_ops.bearing_face((0, 0), (5, 0)) == 'east'
      and pose_ops.bearing_face((0, 0), (-5, 0)) == 'west')

# ---------------------------------------------------------------------------
print("worsened(): the guard itself, arm by arm")
# The CLI-level refusal fires if ANY arm reports a regression, so a test that
# only drives the CLI cannot tell which arm did the work -- measured: neutering
# the COUNT arm left every CLI assertion green, because the shortfall arm
# refused the same request. Each arm is therefore checked here directly.
_zero = {'pad_conflicts': 0, 'hole_conflicts': 0, 'oob_pad_count': 0,
         'pad_shortfall': 0.0}
check("a clean-to-clean move is not a regression",
      pose_ops.worsened(_zero, dict(_zero)) == [])
for _k in ('pad_conflicts', 'hole_conflicts', 'oob_pad_count'):
    check("%s +1 is caught" % _k,
          pose_ops.worsened(_zero, dict(_zero, **{_k: 1})) == [_k])
    check("%s -1 is NOT a refusal (an improvement is welcome)" % _k,
          pose_ops.worsened(dict(_zero, **{_k: 2}),
                            dict(_zero, **{_k: 1})) == [])
check("a deeper overlap at the same COUNT is caught",
      pose_ops.worsened(dict(_zero, pad_conflicts=1, pad_shortfall=0.1),
                        dict(_zero, pad_conflicts=1,
                             pad_shortfall=0.2)) == ['pad_shortfall'])
check("float noise in the shortfall is not a regression",
      pose_ops.worsened(_zero, dict(_zero, pad_shortfall=1e-12)) == [])
check("inherited damage carried forward unchanged is not charged",
      pose_ops.worsened(dict(_zero, pad_conflicts=3, pad_shortfall=0.5),
                        dict(_zero, pad_conflicts=3,
                             pad_shortfall=0.5)) == [])

# ---------------------------------------------------------------------------
print("set: the pose asked for is the pose in the file")
import pose_score                                                 # noqa: E402
_st = pose_score.make_state(pcb0, BOARD, clearance=CLR,
                            board_edge_clearance=EDGE)
_ranked = pose_score.rank_poses(pcb0, BOARD, 'C3', radius=2.0, step=0.5,
                                limit=3, state=_st)
assert _ranked, "no legal pose for C3 -- fixture assumption broken"
GOOD = _ranked[0]

with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'set.kicad_pcb')
    r = run([POSE, BOARD, out, 'set', 'C3', str(GOOD['x']), str(GOOD['y']),
             '--rot', str(GOOD['rot'])])
    check("exits 0", r.returncode == 0, (r.stdout + r.stderr)[-400:])
    s = summary(r)
    check("summary records the move", s['moved'] and
          s['moved'][0]['ref'] == 'C3')
    fp = parse_kicad_pcb(out).footprints['C3']
    check("the file carries the requested pose",
          abs(fp.x - GOOD['x']) < 1e-6 and abs(fp.y - GOOD['y']) < 1e-6
          and (fp.rotation % 360) == GOOD['rot'] % 360,
          "%s %s %s" % (fp.x, fp.y, fp.rotation))
    check("legality did not get worse",
          s['pad_conflicts_after'] <= s['pad_conflicts_before']
          and s['oob_pad_count_after'] <= s['oob_pad_count_before'])

    # ONE arrangement: both ops read the INPUT board, so op B may target the
    # place op A is vacating. Resolve them in sequence instead and B lands on
    # top of A's new pose -- which is the bug this asserts against.
    a0 = parse_kicad_pcb(BOARD).footprints['C3']
    b0 = parse_kicad_pcb(BOARD).footprints['C4']
    multi = os.path.join(d, 'multi.kicad_pcb')
    r = run([POSE, BOARD, multi,
             'set', 'C3', str(GOOD['x']), str(GOOD['y']), '--rot',
             str(GOOD['rot']),
             'set', 'C4', str(a0.x), str(a0.y), '--force'])
    check("two ops in one call exit 0 (forced past legality)",
          r.returncode == 0, (r.stdout + r.stderr)[-400:])
    if r.returncode == 0:
        m = parse_kicad_pcb(multi)
        check("op B landed on op A's INPUT pose, not its output pose",
              abs(m.footprints['C4'].x - a0.x) < 1e-6
              and abs(m.footprints['C4'].y - a0.y) < 1e-6,
              "C4 at %s,%s want %s,%s" % (m.footprints['C4'].x,
                                          m.footprints['C4'].y, a0.x, a0.y))
        check("op A moved too", abs(m.footprints['C3'].x - GOOD['x']) < 1e-6)
        check("C4 really did move", (b0.x, b0.y) != (a0.x, a0.y))

# ---------------------------------------------------------------------------
print("refusal: worse than the input, nothing written")
with tempfile.TemporaryDirectory() as d:
    # ON TOP OF ANOTHER 2-PAD PASSIVE, deliberately: a QFN's ORIGIN is the
    # middle of its pad ring, which is empty copper, so "drop C3 on U1" is a
    # legal pose and would have made this a test of nothing.
    victim = parse_kicad_pcb(BOARD).footprints['C4']
    out = os.path.join(d, 'bad.kicad_pcb')
    r = run([POSE, BOARD, out, 'set', 'C3', str(victim.x), str(victim.y),
             '--rot', str(victim.rotation % 360)])
    check("exits 4 (well-formed, the board said no)", r.returncode == 4,
          (r.stdout + r.stderr)[-300:])
    check("nothing was written", not os.path.exists(out))
    s = summary(r)
    check("the refusal names the categories that got worse",
          'WORSE' in (s.get('refused') or '')
          and s['pad_conflicts_after'] > s['pad_conflicts_before'])
    check("the refusal carries its exit code", s.get('exit_code') == 4)

    # ...and the same request with --force writes, and SAYS it forced.
    forced = os.path.join(d, 'forced.kicad_pcb')
    r = run([POSE, BOARD, forced, 'set', 'C3', str(victim.x), str(victim.y),
             '--rot', str(victim.rotation % 360), '--force'])
    check("--force writes anyway", r.returncode == 0 and
          os.path.isfile(forced))
    check("--force is disclosed", summary(r).get('forced') is True)

# ---------------------------------------------------------------------------
print("inherited damage is not charged to the caller")
with tempfile.TemporaryDirectory() as d:
    # Build a board that ALREADY has a pad conflict (C4 dropped on U1), then
    # ask to move an UNRELATED part to a pose that is fine. An absolute
    # legality gate refuses this; a relative one must not. This is the
    # zero-offset check: the predicate is False for parts before anything
    # moves, so "is this pose legal" cannot be the question.
    dirty = os.path.join(d, 'dirty.kicad_pcb')
    c2 = parse_kicad_pcb(BOARD).footprints['C2']
    write_placed_output(BOARD, dirty, [
        {'reference': 'C1', 'new_x': c2.x, 'new_y': c2.y,
         'new_rotation': c2.rotation % 360}])
    dpcb = parse_kicad_pcb(dirty)
    dirty_grade = pose_ops.grade(dpcb, dirty, CLR)
    check("the fixture really is dirty", dirty_grade['pad_conflicts'] > 0,
          str(dirty_grade['pad_conflicts']))
    dr = pose_score.rank_poses(dpcb, dirty, 'C3', radius=2.0, step=0.5,
                               limit=3)
    if dr:
        out = os.path.join(d, 'clean_move.kicad_pcb')
        r = run([POSE, dirty, out, 'set', 'C3', str(dr[0]['x']),
                 str(dr[0]['y']), '--rot', str(dr[0]['rot'])])
        check("a good move on a dirty board is ACCEPTED", r.returncode == 0,
              (r.stdout + r.stderr)[-300:])
        if r.returncode == 0:
            s = summary(r)
            check("and the inherited conflicts are reported, not charged",
                  s['pad_conflicts_before'] > 0
                  and s['pad_conflicts_after'] <= s['pad_conflicts_before'])
        # --strict-legal is the ABSOLUTE arm and must refuse the same move.
        out2 = os.path.join(d, 'strict.kicad_pcb')
        r2 = run([POSE, dirty, out2, 'set', 'C3', str(dr[0]['x']),
                  str(dr[0]['y']), '--rot', str(dr[0]['rot']),
                  '--strict-legal'])
        check("--strict-legal refuses it", r2.returncode == 4,
              (r2.stdout + r2.stderr)[-200:])
        check("--strict-legal wrote nothing", not os.path.exists(out2))
    else:
        check("dirty-board ranking produced a candidate", False,
              "no legal pose for C3 on the dirty fixture")

# ---------------------------------------------------------------------------
print("--near/--snap seats what the exact request could not")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'snap.kicad_pcb')
    # C4's own pose: the exact form must refuse it (pad on pad), and --near
    # must seat C3 somewhere legal within the radius.
    c4 = parse_kicad_pcb(BOARD).footprints['C4']
    tx, ty = c4.x, c4.y
    r = run([POSE, BOARD, out, 'set', 'C3', str(tx), str(ty)])
    exact_refused = r.returncode == 4
    # 8 mm, because the point aimed at is another part's pose and the room
    # nearby is genuinely occupied: at --radius 4 the snap REFUSES, which is
    # the correct answer and not what this case is testing.
    r = run([POSE, BOARD, out, 'set', 'C3', '--near', str(tx), str(ty),
             '--radius', '8'])
    if exact_refused and r.returncode == 0:
        s = summary(r)
        check("--near snapped", bool(s.get('snapped')),
              json.dumps(s.get('snapped')))
        check("the snapped pose is inside the radius, as a DISTANCE",
              (s['snapped'] or {}).get('dist_mm', 99) <= 8.0,
              str((s['snapped'] or {}).get('dist_mm')))
        check("the snapped board grades no worse",
              s['pad_conflicts_after'] <= s['pad_conflicts_before'])
        # The BOUND, at a radius the unfiltered sweep would overshoot: the
        # lattice is a square, so its ring corners reach 1.41x the radius and
        # the ranker's best answer here is 5.0 mm. Either the snap stays
        # inside the number the caller typed, or it refuses -- never a silent
        # 5 mm move under `--radius 4`. (Mutation-checked: dropping the filter
        # left every other snap assertion green.)
        r4 = run([POSE, BOARD, os.path.join(d, 'r4.kicad_pcb'), 'set', 'C3',
                  '--near', str(tx), str(ty), '--radius', '4'])
        if r4.returncode == 0:
            s4 = summary(r4)
            check("--radius 4 is a bound, not a suggestion",
                  (s4['snapped'] or {}).get('dist_mm', 99) <= 4.0,
                  str((s4['snapped'] or {}).get('dist_mm')))
        else:
            check("--radius 4 refuses rather than overshooting",
                  r4.returncode == 4, str(r4.returncode))
        fp = parse_kicad_pcb(out).footprints['C3']
        check("the file carries the SNAPPED pose, not the requested one",
              abs(fp.x - s['snapped']['to'][0]) < 1e-6
              and abs(fp.y - s['snapped']['to'][1]) < 1e-6)
    else:
        check("--near seats a point the exact form refused",
              exact_refused and r.returncode == 0,
              "exact refused=%s, near rc=%s" % (exact_refused, r.returncode))

# ---------------------------------------------------------------------------
print("rotate: absolute by default, --relative for a delta")
with tempfile.TemporaryDirectory() as d:
    r1 = parse_kicad_pcb(BOARD).footprints['R1']
    out = os.path.join(d, 'rot.kicad_pcb')
    r = run([POSE, BOARD, out, 'rotate', 'R1', '90'])
    check("rotate exits 0", r.returncode == 0, (r.stdout + r.stderr)[-300:])
    if r.returncode == 0:
        fp = parse_kicad_pcb(out).footprints['R1']
        check("absolute rotation lands exactly", (fp.rotation % 360) == 90,
              str(fp.rotation))
        check("x/y are untouched",
              abs(fp.x - r1.x) < 1e-9 and abs(fp.y - r1.y) < 1e-9)
    out2 = os.path.join(d, 'rel.kicad_pcb')
    # --force: the claim under test is the ARITHMETIC, and whether that
    # particular angle happens to graze a neighbour is a different question.
    r = run([POSE, BOARD, out2, 'rotate', 'R1', '90', '--relative',
             '--force'])
    if r.returncode == 0:
        fp = parse_kicad_pcb(out2).footprints['R1']
        check("--relative adds to the current rotation",
              (fp.rotation % 360) == ((r1.rotation + 90) % 360),
              "%s from %s" % (fp.rotation, r1.rotation))
    else:
        check("--relative run exits 0", False, (r.stdout + r.stderr)[-300:])

# ---------------------------------------------------------------------------
print("face: aimed, then MEASURED on the board it wrote")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'face.kicad_pcb')
    r = run([POSE, BOARD, out, 'face', 'U1', 'S', 'USB1', '--force'])
    check("face exits 0 under --force", r.returncode == 0,
          (r.stdout + r.stderr)[-300:])
    if r.returncode == 0:
        s = summary(r)
        op = s['ops'][0]
        check("the op records the aim and the measurement",
              'target_face' in op and 'row_on_target' in op
              and 'row_landed' in op, json.dumps(op)[:300])
        # NOT `row_on_target[1] == len(row_pads)`, which the engine assigns on
        # one line and so asserts nothing. The claim worth pinning is that the
        # row it measured is the row that was ON that face before the write.
        by_before = pose_ops.part_faces(pcb0, 'U1', clearance=CLR,
                                        track_width=TW)
        check("the measured row IS the named face's row on the input board",
              len(op['row_pad_ix']) == len(by_before[op['face']]),
              "%d vs %d" % (len(op['row_pad_ix']),
                            len(by_before[op['face']])))
        # The independent recount, deliberately NOT the engine's expression:
        # pads are matched by LOCAL coordinates, which a rotation leaves alone,
        # where the engine matches by index. Keying by pad NUMBER here (as an
        # earlier version did) mirrors the very bug the index fixed.
        fpcb = parse_kicad_pcb(out)
        by = pose_ops.part_faces(fpcb, 'U1', clearance=CLR, track_width=TW)
        landed_local = {}
        for f, pads in by.items():
            for p in pads:
                landed_local[(round(p.local_x, 4), round(p.local_y, 4))] = f
        before_pads = pcb0.footprints['U1'].pads
        row_local = [(round(before_pads[i].local_x, 4),
                      round(before_pads[i].local_y, 4))
                     for i in op['row_pad_ix']]
        hit = sum(1 for k in row_local
                  if landed_local.get(k) == op['target_face'])
        check("the CLI's count matches an independent recount",
              hit == op['row_on_target'][0],
              "%s vs %s" % (hit, op['row_on_target'][0]))

# ---------------------------------------------------------------------------
print("lock / unlock are inverses, and a lock refuses a move")
with tempfile.TemporaryDirectory() as d:
    b = os.path.join(d, 'b.kicad_pcb')
    shutil.copyfile(BOARD, b)
    orig = open(b, encoding='utf-8').read()
    n1 = stamp_locked(b, ['R1', 'C3'])
    n2 = stamp_unlocked(b, ['R1', 'C3'])
    check("stamp_unlocked inverts stamp_locked byte for byte",
          n1 == 2 and n2 == 2 and open(b, encoding='utf-8').read() == orig)
    check("unlocking what was never locked changes nothing",
          stamp_unlocked(b, ['Q1']) == 0
          and open(b, encoding='utf-8').read() == orig)

    locked = os.path.join(d, 'locked.kicad_pcb')
    r = run([POSE, BOARD, locked, 'lock', 'C3'])
    check("lock exits 0 and stamps", r.returncode == 0
          and 'C3' in extract_locked_refs(locked))
    out = os.path.join(d, 'move.kicad_pcb')
    # The move used here is GOOD -- the ranked-legal pose from the top of this
    # file -- so the only thing that can refuse it is the lock. A move that
    # legality would refuse anyway proves nothing about the lock guard.
    move = ['set', 'C3', str(GOOD['x']), str(GOOD['y']),
            '--rot', str(GOOD['rot'])]
    r = run([POSE, locked, out] + move)
    check("a locked part refuses a direct move (exit 4)", r.returncode == 4,
          (r.stdout + r.stderr)[-200:])
    check("the refusal is about the LOCK, not legality",
          'locked in the board' in (r.stdout + r.stderr))
    check("and wrote nothing", not os.path.exists(out))
    r = run([POSE, locked, out, 'unlock', 'C3'] + move)
    check("unlock in the SAME call opens it", r.returncode == 0,
          (r.stdout + r.stderr)[-300:])
    if r.returncode == 0:
        opcb = parse_kicad_pcb(out)
        check("the move landed and the lock is gone",
              abs(opcb.footprints['C3'].x - GOOD['x']) < 1e-6
              and 'C3' not in extract_locked_refs(out))

# ---------------------------------------------------------------------------
print("--dry-run writes nothing at all")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'dry.kicad_pcb')
    r = run([POSE, BOARD, out, 'set', 'C3', str(GOOD['x']), str(GOOD['y']),
             '--rot', str(GOOD['rot']), '--dry-run'])
    check("dry-run exits 0", r.returncode == 0, (r.stdout + r.stderr)[-300:])
    check("dry-run wrote no board", not os.path.exists(out))
    check("dry-run left nothing else behind", os.listdir(d) == [])
    s = summary(r)
    check("dry-run says so and reports no output path",
          s['dry_run'] is True and s['output'] is None)
    check("dry-run still grades", s['pad_conflicts_after'] is not None)

# ---------------------------------------------------------------------------
print("siblings travel with the output (#441)")
with tempfile.TemporaryDirectory() as d:
    b = os.path.join(d, 'b.kicad_pcb')
    shutil.copyfile(BOARD, b)
    shutil.copyfile(PRO, os.path.join(d, 'b.kicad_pro'))
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as f:
        f.write('(version 1)\n')
    out = os.path.join(d, 'out.kicad_pcb')
    r = run([POSE, b, out, 'rotate', 'R1', '90'])
    check("run on a board with siblings exits 0", r.returncode == 0,
          (r.stdout + r.stderr)[-300:])
    check("the .kicad_pro travelled", os.path.isfile(
        os.path.join(d, 'out.kicad_pro')))
    check("the .kicad_dru travelled", os.path.isfile(
        os.path.join(d, 'out.kicad_dru')))
    if r.returncode == 0:
        s = summary(r)
        check("the knobs came from the BOARD, not a constant",
              s['knobs']['clearance']['source'] == 'board netclass',
              json.dumps(s['knobs']))

# ---------------------------------------------------------------------------
print("the copper gate")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'r.kicad_pcb')
    ref = sorted(parse_kicad_pcb(ROUTED).footprints)[0]
    refuse_check([sys.executable, '-X', 'utf8', POSE, ROUTED, out,
                  'rotate', ref, '90'],
                 refuse='strands every track', code=3)
    check("the copper gate wrote nothing", not os.path.exists(out))
    r = run([POSE, ROUTED, out, 'rotate', ref, '90', '--allow-routed'])
    check("--allow-routed proceeds", r.returncode in (0, 4),
          (r.stdout + r.stderr)[-200:])

# ---------------------------------------------------------------------------
print("usage-shaped refusals exit 2, with the reason")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'x.kicad_pcb')
    base = [sys.executable, '-X', 'utf8', POSE, BOARD, out]
    refuse_check(base + ['set', 'NOPE', '130', '98'],
                 refuse='is not a footprint on this board', code=2)
    refuse_check(base + ['face', 'U1', 'sideways', 'USB1'],
                 refuse='is not a face', code=2)
    refuse_check(base + ['set', 'C3', '130', '98', 'set', 'C3', '131', '99'],
                 refuse='named by two ops in one call', code=2)
    refuse_check(base + ['set', 'C3', '130'],
                 refuse='needs both X and Y', code=2)
    refuse_check(base + ['set', 'C3'],
                 refuse='asks for nothing', code=2)
    refuse_check(base + ['face', 'U1', 'N', 'U1'],
                 refuse='cannot face itself', code=2)
    refuse_check(base + ['face', 'R1', 'north', 'R1'],
                 refuse='cannot face itself', code=2)
    refuse_check([sys.executable, '-X', 'utf8', POSE, BOARD, 'set', 'C3',
                  '130', '98'],
                 refuse='reads as the OUTPUT PATH here', code=2)
    check("no usage refusal wrote a board", not os.path.exists(out))

# ---------------------------------------------------------------------------
print("every exit prints exactly one JSON_SUMMARY")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'j.kicad_pcb')
    c4 = parse_kicad_pcb(BOARD).footprints['C4']
    for name, argv in (
            ('success', [POSE, BOARD, out, 'rotate', 'R1', '90']),
            ('legality refusal', [POSE, BOARD, os.path.join(d, 'k.kicad_pcb'),
                                  'set', 'C3', str(c4.x), str(c4.y),
                                  '--rot', str(c4.rotation % 360)]),
            ('usage refusal', [POSE, BOARD, os.path.join(d, 'l.kicad_pcb'),
                               'set', 'NOPE', '1', '2'])):
        r = run(argv)
        check("%s prints one JSON_SUMMARY" % name, len(summaries(r)) == 1,
              "%d found, rc=%s" % (len(summaries(r)), r.returncode))

# ---------------------------------------------------------------------------
print("the provenance regime accepts this lever and still refuses a hand script")
with tempfile.TemporaryDirectory() as d:
    from placement import provenance
    b = os.path.join(d, 'b.kicad_pcb')
    shutil.copyfile(BOARD, b)
    provenance.start_regime(d, b)
    check("place_pose.py is a registered lever",
          'place_pose.py' in provenance.LEVER_REGISTRY)
    out = os.path.join(d, 'armed.kicad_pcb')
    r = run([POSE, b, out, 'rotate', 'R1', '90'])
    check("a place_pose write is accepted under an armed regime",
          r.returncode == 0 and os.path.isfile(out),
          (r.stdout + r.stderr)[-400:])
    # The hand script this tool replaces: same write, no declared lever.
    hand = os.path.join(d, 'hand.kicad_pcb')
    try:
        write_placed_output(b, hand, [{'reference': 'R1', 'new_x': 10.0,
                                       'new_y': 10.0, 'new_rotation': 0}])
        raised = None
    except provenance.UnaidedViolation as exc:
        raised = str(exc)
    check("an undeclared write is still refused", raised is not None,
          (raised or "NO UnaidedViolation was raised")[:120])

# ---------------------------------------------------------------------------
print("the OFF-BOARD magnitude is an arm, not just the count")
with tempfile.TemporaryDirectory() as d:
    # A part already off the board, moved much FURTHER off it. The count arm
    # sees 1 -> 1 and shrugs; measured before `oob_pad_amount` was an arm, a
    # part 2.0 mm out was moved to 204.66 mm out, exit 0, `legal: true`.
    b = parse_kicad_pcb(BOARD)
    bounds = b.board_info.board_bounds
    off = os.path.join(d, 'off.kicad_pcb')
    write_placed_output(BOARD, off, [
        {'reference': 'C4', 'new_x': bounds[0] - 1.0,
         'new_y': (bounds[1] + bounds[3]) / 2.0, 'new_rotation': 0}])
    g0 = pose_ops.grade(parse_kicad_pcb(off), off, CLR)
    check("the fixture starts off-board", g0['oob_pad_count'] >= 1,
          str(g0['oob_pad_count']))
    out = os.path.join(d, 'further.kicad_pcb')
    r = run([POSE, off, out, 'set', 'C4', str(bounds[0] - 100.0),
             str((bounds[1] + bounds[3]) / 2.0)])
    check("further off-board at the same COUNT is refused", r.returncode == 4,
          (r.stdout + r.stderr)[-300:])
    check("nothing was written", not os.path.exists(out))
    s = summary(r)
    check("and the refusal names the AMOUNT",
          'oob_pad_amount' in (s.get('refused') or ''), s.get('refused'))
    check("the summary reports the amount both sides",
          s['oob_pad_amount_after'] > s['oob_pad_amount_before'])
    check("worsened() names it directly",
          pose_ops.worsened({'oob_pad_count': 1, 'oob_pad_amount': 2.0},
                            {'oob_pad_count': 1,
                             'oob_pad_amount': 204.7}) == ['oob_pad_amount'])

# ---------------------------------------------------------------------------
print("`legal` means clean; `no_worse` is the verdict the verb acts on")
with tempfile.TemporaryDirectory() as d:
    dirty = os.path.join(d, 'dirty.kicad_pcb')
    c2 = parse_kicad_pcb(BOARD).footprints['C2']
    write_placed_output(BOARD, dirty, [
        {'reference': 'C1', 'new_x': c2.x, 'new_y': c2.y,
         'new_rotation': c2.rotation % 360}])
    dr = pose_score.rank_poses(parse_kicad_pcb(dirty), dirty, 'C3',
                               radius=2.0, step=0.5, limit=3)
    if dr:
        out = os.path.join(d, 'ok.kicad_pcb')
        r = run([POSE, dirty, out, 'set', 'C3', str(dr[0]['x']),
                 str(dr[0]['y']), '--rot', str(dr[0]['rot'])])
        s = summary(r)
        check("accepted", r.returncode == 0, (r.stdout + r.stderr)[-200:])
        check("no_worse is true (that is what it was accepted on)",
              s['no_worse'] is True)
        check("legal is FALSE, because the board still is not clean",
              s['legal'] is False and s['pad_conflicts_after'] > 0,
              "legal=%s conflicts=%s" % (s['legal'],
                                         s['pad_conflicts_after']))
        check("and the summary says which is which",
              'no_worse' in (s.get('legal_basis') or ''))
    else:
        check("dirty-board ranking produced a candidate", False)

# ---------------------------------------------------------------------------
print("a refusal never names an output path")
with tempfile.TemporaryDirectory() as d:
    c4 = parse_kicad_pcb(BOARD).footprints['C4']
    out = os.path.join(d, 'never.kicad_pcb')
    r = run([POSE, BOARD, out, 'set', 'C3', str(c4.x), str(c4.y),
             '--rot', str(c4.rotation % 360)])
    s = summary(r)
    check("output is null on a legality refusal", s['output'] is None,
          str(s['output']))
    r2 = run([POSE, BOARD, out, 'set', 'NOPE', '1', '2'])
    check("output is null on a usage refusal too",
          summary(r2)['output'] is None)
    check("neither wrote", not os.path.exists(out))

# ---------------------------------------------------------------------------
print("every exit THIS TOOL decides carries a summary")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'x.kicad_pcb')
    ref = sorted(parse_kicad_pcb(ROUTED).footprints)[0]
    r = run([POSE, ROUTED, out, 'rotate', ref, '90'])
    check("the copper gate exits 3 with a summary",
          r.returncode == 3 and len(summaries(r)) == 1,
          "rc=%s summaries=%d" % (r.returncode, len(summaries(r))))
    check("and the summary says why",
          'strands every track' in (summaries(r)[0].get('refused') or ''))
    r = run([POSE, os.path.join(d, 'nope.kicad_pcb'), out, 'rotate', 'R1',
             '90'])
    check("a missing input exits 2 with a summary",
          r.returncode == 2 and len(summaries(r)) == 1,
          "rc=%s summaries=%d" % (r.returncode, len(summaries(r))))
    # The REASON, not just the code. Mutation-checked: delete the isfile()
    # guard and the parser raises instead, so the run STILL exits 2 with a
    # summary -- every arm above stays green while the message changes from
    # "is not a file" to "cannot read ... [Errno 2]". Those send a caller to
    # different places (a mistyped path vs a corrupt board), so the message is
    # the contract here, not the code.
    check("and it says the file is not there, not that it is unreadable",
          'is not a file' in (summaries(r)[0].get('refused') or ''),
          str(summaries(r)[0].get('refused')))
    # A directory that does not exist used to be a FileNotFoundError traceback
    # and an exit 1 the docstring's table does not list.
    r = run([POSE, BOARD, os.path.join(d, 'no', 'such', 'dir', 'o.kicad_pcb'),
             'rotate', 'R1', '90'])
    check("an unwritable output path exits 2, not a traceback",
          r.returncode == 2 and 'Traceback' not in (r.stdout + r.stderr),
          "rc=%s" % r.returncode)
    check("with a summary and a reason", len(summaries(r)) == 1
          and 'cannot write' in (summaries(r)[0].get('refused') or ''))

# ---------------------------------------------------------------------------
print("the snap ladder: the lattice rung answers where the ranker does not")
if os.path.isfile(FLAT):
    with tempfile.TemporaryDirectory() as d:
        # Measured by the #892 verifier on this board: `set C4 --near
        # 128.0 49.53 --radius 3` had rung 1 (rank_poses) return ZERO
        # candidates -- 625 dropped by the absolute gate -- while 236 poses on
        # the same lattice inside the same radius graded no worse.
        out = os.path.join(d, 'lad.kicad_pcb')
        r = run([POSE, FLAT, out, 'set', 'C4', '--near', '128.0', '49.53',
                 '--radius', '3'])
        check("the snap seats it", r.returncode == 0,
              (r.stdout + r.stderr)[-300:])
        if r.returncode == 0:
            s = summary(r)
            check("the census reports BOTH rungs",
                  'ranked' in (s.get('snap_census') or {})
                  and 'lattice' in (s.get('snap_census') or {}),
                  json.dumps(s.get('snap_census')))
            check("and the answer says which rung produced it",
                  (s['snapped'] or {}).get('rung') in ('ranked', 'lattice'),
                  json.dumps(s.get('snapped')))
            check("inside the radius, as a distance",
                  (s['snapped'] or {}).get('dist_mm', 99) <= 3.0)
            check("and the written board grades no worse",
                  s['no_worse'] is True)
else:
    check("flat_hierarchy fixture present", False, FLAT)

# ---------------------------------------------------------------------------
print("a symmetric row is named, not reported as a near miss")
if os.path.isfile(FLAT):
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, 'sym.kicad_pcb')
        r = run([POSE, FLAT, out, 'face', 'C1', 'N', 'C2'])
        check("a 2-pad row that cannot be aimed is refused", r.returncode == 4,
              (r.stdout + r.stderr)[-200:])
        check("and the refusal says WHY, not just where it landed",
              'cannot be aimed by rotating' in (r.stdout + r.stderr),
              (r.stdout + r.stderr)[-200:])
        s = summary(r)
        check("the op records the symmetry as a fact",
              s['ops'][0].get('row_symmetric') is True,
              json.dumps(s['ops'][0])[:200])

# ---------------------------------------------------------------------------
print("a face row is identified by PAD, not by pad number")
with tempfile.TemporaryDirectory() as d:
    # esp_prog's USB1 carries six pads numbered `0`, one on each face. A
    # `{pad_number: face}` map answers about whichever the dict saw last, so
    # the verification scored a row that had provably rotated as 0/2 and then
    # called it symmetric. 117 parts across 12 of the 22 tracked boards have
    # duplicate pad numbers, so this is the common case, not a corner.
    dup = [ref for ref, fp in parse_kicad_pcb(BOARD).footprints.items()
           if len(set(p.pad_number for p in (fp.pads or ()))) < len(fp.pads or ())]
    check("the fixture has a part with duplicate pad numbers", 'USB1' in dup,
          str(dup))
    out = os.path.join(d, 'dup.kicad_pcb')
    r = run([POSE, BOARD, out, 'face', 'USB1', 'north', 'U1', '--force'])
    check("face on it exits 0", r.returncode == 0, (r.stdout + r.stderr)[-300:])
    if r.returncode == 0:
        op = summary(r)['ops'][0]
        check("the row is measured by pad index, not by number",
              'row_pad_ix' in op and len(op['row_pad_ix']) == len(op['row_pads']))
        check("and it lands where the prediction says",
              op['row_on_target'][0] == op['row_on_target'][1],
              "%s of %s, landed %s" % (op['row_on_target'][0],
                                       op['row_on_target'][1],
                                       op['row_landed']))
        check("a row that MOVED is not called symmetric",
              op.get('row_symmetric') is False)
        # The independent recount, deliberately NOT the engine's expression:
        # match pads by their LOCAL coordinates, which a rotation leaves alone.
        before_fp = parse_kicad_pcb(BOARD).footprints['USB1']
        after_pcb = parse_kicad_pcb(out)
        by = pose_ops.part_faces(after_pcb, 'USB1', clearance=CLR,
                                 track_width=TW)
        landed_local = {}
        for f, pads in by.items():
            for p in pads:
                landed_local[(round(p.local_x, 4), round(p.local_y, 4))] = f
        row_local = [(round(before_fp.pads[i].local_x, 4),
                      round(before_fp.pads[i].local_y, 4))
                     for i in op['row_pad_ix']]
        hit = sum(1 for k in row_local
                  if landed_local.get(k) == op['target_face'])
        check("a local-coordinate recount agrees with the CLI",
              hit == op['row_on_target'][0],
              "%s vs %s" % (hit, op['row_on_target'][0]))

# ---------------------------------------------------------------------------
print("a failed promote leaves the PREVIOUS output untouched")
with tempfile.TemporaryDirectory() as d:
    import stat
    src = os.path.join(d, 'in.kicad_pcb')
    shutil.copyfile(BOARD, src)
    shutil.copyfile(PRO, os.path.join(d, 'in.kicad_pro'))
    out = os.path.join(d, 'out.kicad_pcb')
    with open(out, 'w', encoding='utf-8') as f:
        f.write('OLD-OUTPUT\n')
    pro = os.path.join(d, 'out.kicad_pro')
    with open(pro, 'w', encoding='utf-8') as f:
        f.write('OLD-PRO\n')
    os.chmod(pro, stat.S_IREAD)
    try:
        r = run([POSE, src, out, 'rotate', 'R1', '90'])
        wrote = open(out, encoding='utf-8').read()
        check("the run refuses rather than half-writing", r.returncode == 2,
              "rc=%s" % r.returncode)
        check("the OLD board is still there, byte for byte",
              wrote == 'OLD-OUTPUT\n', wrote[:40])
        check("and the summary says nothing was written",
              summary(r)['output'] is None
              and 'Nothing was written' in (summary(r).get('refused') or ''),
              str(summary(r).get('refused'))[:120])
        check("no .krt-tmp file is left behind",
              not [f for f in os.listdir(d) if f.endswith('.krt-tmp')],
              str(os.listdir(d)))
    finally:
        os.chmod(pro, stat.S_IWRITE)

# ---------------------------------------------------------------------------
print("lock and unlock hygiene")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'x.kicad_pcb')
    base = [sys.executable, '-X', 'utf8', POSE, BOARD, out]
    # The re-lock workflow this tool's own refusal message recommends: it used
    # to move the part, DROP the lock, and report `locked: ["C4"]`, because
    # apply_locks stamps then strips whatever the caller wrote.
    refuse_check(base + ['unlock', 'C3', 'rotate', 'C3', '90', 'lock', 'C3'],
                 refuse='named by both lock and unlock', code=2)
    refuse_check(base + ['lock', 'NOSUCHREF'],
                 refuse='not on this board', code=2)
    refuse_check(base + ['unlock', 'NOSUCHREF'],
                 refuse='not on this board', code=2)
    check("neither wrote a board", not os.path.exists(out))
    # And the guard that was added but never reachable in a test: an unlock
    # that does not take must refuse BEFORE the board is promoted.
    import placement.seeder as _seeder
    real = _seeder.stamp_unlocked
    locked = os.path.join(d, 'locked.kicad_pcb')
    r = run([POSE, BOARD, locked, 'lock', 'C3'])
    check("staged a locked board", r.returncode == 0)
    try:
        _seeder.stamp_unlocked = lambda *a, **k: 0      # the failure mode
        out2 = os.path.join(d, 'y.kicad_pcb')
        try:
            # The op is a NO-OP rotation (C3's own angle): the point is the
            # unlock guard, and a rotation that also fails legality would
            # refuse earlier for a different reason.
            _rot_now = parse_kicad_pcb(locked).footprints['C3'].rotation % 360
            pose_ops.apply_poses(locked, out2, [
                {'kind': 'rotate', 'ref': 'C3', 'rot': _rot_now,
                 'relative': False}], unlock_refs=['C3'])
            raised = None
        except pose_ops.PoseRefusal as exc:
            raised = exc.reason
        check("a silent unlock failure is refused", raised is not None
              and 'unlock did not take' in raised, str(raised)[:120])
        check("and nothing was promoted", not os.path.exists(out2))
    finally:
        _seeder.stamp_unlocked = real

# ---------------------------------------------------------------------------
print("the knobs that used to crash, and the ones that do not")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'k.kicad_pcb')
    base = [sys.executable, '-X', 'utf8', POSE, BOARD, out]
    # --snap-step 0 divided the sweep by zero: traceback, exit 1, NO summary.
    refuse_check(base + ['set', 'C3', '130', '98', '--snap', '--snap-step',
                         '0'],
                 refuse='must be positive', code=2)
    refuse_check(base + ['set', 'C3', '130', '98', '--radius', '-3'],
                 refuse='is not one', code=2)
    refuse_check(base + ['set', 'C3', '130', '98', '--snap-tries', '-1'],
                 refuse='is not a count', code=2)
    check("no knob refusal wrote a board", not os.path.exists(out))

# ---------------------------------------------------------------------------
print("a face naming an empty row is a TYPO (2), not a measurement (4)")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'e.kicad_pcb')
    by = pose_ops.part_faces(pcb0, 'C3', clearance=CLR, track_width=TW)
    empty = [f for f in ('north', 'south', 'east', 'west') if not by.get(f)]
    if empty:
        refuse_check([sys.executable, '-X', 'utf8', POSE, BOARD, out,
                      'face', 'C3', empty[0], 'U1'],
                     refuse='so there is no row to aim', code=2)
        check("and it wrote nothing", not os.path.exists(out))
    else:
        check("C3 has an empty face to name", False, str(sorted(by)))

# ---------------------------------------------------------------------------
print("a forced run reports EVERY finding, not the last one")
if os.path.isfile(FLAT):
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, 'both.kicad_pcb')
        c5 = parse_kicad_pcb(FLAT).footprints['C5']
        r = run([POSE, FLAT, out, 'face', 'C1', 'north', 'C2',
                 'set', 'C4', str(c5.x), str(c5.y), '--force'])
        if r.returncode == 0:
            s = summary(r)
            ref = s.get('refused') or ''
            check("the legality finding is there", 'WORSE' in ref, ref[:80])
            check("and the face finding survived beside it",
                  'C1' in ref and 'row' in ref, ref[:200])
            check("the run is marked forced", s.get('forced') is True)
        else:
            check("the forced run wrote", False, (r.stdout + r.stderr)[-200:])

# ---------------------------------------------------------------------------
print("what a refusal hands back for the next attempt")
with tempfile.TemporaryDirectory() as d:
    c4 = parse_kicad_pcb(BOARD).footprints['C4']
    out = os.path.join(d, 'n.kicad_pcb')
    r = run([POSE, BOARD, out, 'set', 'C3', str(c4.x), str(c4.y),
             '--rot', str(c4.rotation % 360)])
    s = summary(r)
    check("the refusal names a nearest candidate", s['nearest_legal'] is not None
          or (s.get('nearest_legal_census') or {}).get('ranked') == 0,
          json.dumps(s.get('nearest_legal_census')))
    check("and says what currency that is in",
          'candidate_valid' in (s.get('nearest_legal_basis') or ''))
    if s['nearest_legal']:
        check("the refusal message names it too",
              'nearest candidate' in (s.get('refused') or ''),
              (s.get('refused') or '')[-120:])

# ---------------------------------------------------------------------------
print("--near on a call carrying several ops says it did not apply")
with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'multi2.kicad_pcb')
    r = run([POSE, BOARD, out, 'set', 'C3', '--near', str(GOOD['x']),
             str(GOOD['y']), 'rotate', 'R1', '90', '--force'])
    check("the run proceeds", r.returncode == 0, (r.stdout + r.stderr)[-200:])
    if r.returncode == 0:
        s = summary(r)
        check("and the census says the snap was skipped, with the count",
              'carries 2' in ((s.get('snap_census') or {}).get('skipped') or ''),
              json.dumps(s.get('snap_census')))
        check("the operator sees it on stderr",
              '--snap/--near did not apply' in r.stderr)

# ---------------------------------------------------------------------------
print("two parts on one coordinate have no direction to aim")
with tempfile.TemporaryDirectory() as d:
    stacked = os.path.join(d, 'stacked.kicad_pcb')
    c3 = parse_kicad_pcb(BOARD).footprints['C3']
    write_placed_output(BOARD, stacked, [
        {'reference': 'C4', 'new_x': c3.x, 'new_y': c3.y,
         'new_rotation': c3.rotation % 360}])
    refuse_check([sys.executable, '-X', 'utf8', POSE, stacked,
                  os.path.join(d, 'o.kicad_pcb'), 'face', 'C3', 'north', 'C4'],
                 refuse='share a centre', code=2)

check("bearing_face breaks a diagonal tie on the x axis, deterministically",
      pose_ops.bearing_face((0, 0), (-5, -5)) == 'west'
      and pose_ops.bearing_face((0, 0), (5, 5)) == 'east'
      and pose_ops.bearing_face((0, 0), (-5, 5)) == 'west')
check("a dry run reports the path it would have written",
      True)   # asserted below against a real run

with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'w.kicad_pcb')
    r = run([POSE, BOARD, out, 'rotate', 'R1', '90', '--dry-run'])
    s = summary(r)
    check("would_write names the output path a dry run declined",
          s.get('would_write') == out and s['output'] is None,
          "%s / %s" % (s.get('would_write'), s['output']))
    check("and the lock keys are present even though none were asked for",
          'locked' in s and s['locked'] == [] and s['locked_count'] is None)

print()
print(f"{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
