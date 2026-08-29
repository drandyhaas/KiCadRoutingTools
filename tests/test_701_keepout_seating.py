#!/usr/bin/env python3
"""#701: `place_seed` honours a declared keep-out, end to end.

The board is HAND-WRITTEN rather than taken from the corpus, for one reason:
a control arm needs a board where the seat location is a THEOREM. On a corpus
board "where would this part land without the keep-out" is only answerable by
re-running the seeder, which makes the control circular. Here a single-ref
zone gets `jx, jy = (0.0, 0.0)` (seeder.py, stage 2), so the target is the
zone centre exactly and the nearest clear pose can be written down.

Every accepting arm asserts on the RE-PARSED WRITTEN BOARD, never on the JSON
summary alone -- test_630_seeder_eviction.py records that its own first
version passed while one part sat 100% inside another's courtyard, because it
believed a count. The JSON is checked too, for what it claims.

Every arm that shows a keep-out doing something has a CONTROL arm showing the
same board WITHOUT the keep-out doing the other thing. Without the control,
an assertion like "the part is not in the rect" is satisfied by a board where
the part was never going to be in the rect, and the test passes in both
directions.
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
        sys.path.insert(0, os.path.join(_p, 'py_router'))   # placement split
        sys.path.insert(0, os.path.join(_p, 'py_tools'))    # placement split
        sys.path.insert(0, os.path.join(_p, 'py_placer'))   # placement split

from kicad_parser import parse_kicad_pcb                        # noqa: E402
from placement import floorplan, legality                       # noqa: E402
import pose_score                                               # noqa: E402

RUN_ALL_TIMEOUT = 1200

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}"
          + (f" -- {detail}" if detail else ""))


# --------------------------------------------------------------------------
# fixtures -- the shape of tests/test_630_seeder_eviction.py's, on purpose
# --------------------------------------------------------------------------

def _part(ref, x, y, half_w, half_h, npads, pad_y=0.0, layer='F.Cu',
          thru=False):
    """A footprint with a (2*half_w x 2*half_h) courtyard and `npads` pads.

    `thru=True` emits drilled pads on every copper layer, which is what makes
    `footprint_has_through_pads` true and therefore what puts the part on BOTH
    faces -- the case `rule_keepout` grades from the opposite side.
    """
    if thru:
        pads = ''.join(
            f'\t\t(pad "{i + 1}" thru_hole circle\n'
            f'\t\t\t(at {i * 0.6 - 0.3} {pad_y})\n'
            f'\t\t\t(size 0.6 0.6)\n\t\t\t(drill 0.35)\n'
            f'\t\t\t(layers "*.Cu" "*.Mask")\n'
            f'\t\t\t(net {1 if i == 0 else 2} "N{1 if i == 0 else 2}")\n'
            f'\t\t\t(uuid "p{i}-{ref}")\n\t\t)\n' for i in range(npads))
    else:
        pads = ''.join(
            f'\t\t(pad "{i + 1}" smd rect\n'
            f'\t\t\t(at {i * 0.2 - 0.2} {pad_y})\n'
            f'\t\t\t(size 0.3 0.3)\n\t\t\t(layers "{layer}")\n'
            f'\t\t\t(net {1 if i == 0 else 2} "N{1 if i == 0 else 2}")\n'
            f'\t\t\t(uuid "p{i}-{ref}")\n\t\t)\n' for i in range(npads))
    return f'''\t(footprint "test:P{ref}"
\t\t(layer "{layer.split('.')[0]}.Cu")
\t\t(uuid "fp-{ref}")
\t\t(at {x} {y})
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 0)
\t\t)
\t\t(fp_rect
\t\t\t(start {-half_w} {-half_h})
\t\t\t(end {half_w} {half_h})
\t\t\t(layer "{layer.split('.')[0]}.CrtYd")
\t\t\t(uuid "cy-{ref}")
\t\t)
{pads}\t)
'''


def board(path, parts, size=(20, 12)):
    body = ('(kicad_pcb\n\t(version 20241229)\n'
            '\t(net 0 "")\n\t(net 1 "N1")\n\t(net 2 "N2")\n'
            '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end {} {})\n'
            '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n'.format(*size)
            + ''.join(parts) + ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)


SIZE = (20.0, 12.0)
#: The keep-out every arm below uses. On a 20x12 board it is well inside the
#: usable inset, so the OUTLINE can never be what refuses a pose.
KO = (8.0, 4.0, 12.0, 8.0)


def one_part_board(path, **kw):
    """U1 alone: a 4x4 square courtyard at the board centre. Square so that
    the four rotations the search tries are indistinguishable and the pose
    arithmetic below cannot be disturbed by one of them."""
    board(path, [_part('U1', 10, 6, 2.0, 2.0, 4, **kw)], size=SIZE)


def wide_zone_intent(keepouts=(), zone=(2.0, 2.0, 18.0, 10.0), refs=('U1',)):
    """A single-ref zone, so the seat target is the zone CENTRE exactly."""
    it = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
          "envelope": {"rect": [0.0, 0.0, SIZE[0], SIZE[1]],
                       "tolerance_mm": 0.5},
          "blocks": [{"name": "b", "refs": list(refs), "zone": list(zone),
                      "tolerance_mm": 0.5}]}
    if keepouts:
        it["keepouts"] = [dict(k) for k in keepouts]
    return it


def run(workdir, make_board, intent, *extra, tag='seed', polish=False):
    bpath = os.path.join(workdir, f'{tag}-in.kicad_pcb')
    ipath = os.path.join(workdir, f'{tag}-fp.json')
    out = os.path.join(workdir, f'{tag}-out.kicad_pcb')
    make_board(bpath)
    with open(ipath, 'w', encoding='utf-8') as f:
        json.dump(intent, f)
    # `--force`: these fixtures carry a handful of parts at honest
    # coordinates, so the board-state gate reads them as already PLACED and
    # exits 3. Re-seeding is exactly what the arms want, and the seat target
    # is the zone centre either way -- a single-ref zone draws no jitter.
    argv = [sys.executable, '-X', 'utf8',
            os.path.join('py_placer', 'place_seed.py'), bpath, out,
            '--intent', ipath, '--clearance', '0.2',
            '--board-edge-clearance', '0.5', '--force']
    if not polish:
        argv.append('--no-polish')
    argv.extend(extra)
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO, timeout=900,
                       env=dict(os.environ, PYTHONHASHSEED='0',
                                PYTHONIOENCODING='utf-8'))
    summary = None
    for line in r.stdout.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            summary = json.loads(line.split(':', 1)[1])
    return r, summary, out


def poses(path):
    pcb = parse_kicad_pcb(path)
    return {ref: (round(fp.x, 3), round(fp.y, 3), round(fp.rotation, 1))
            for ref, fp in sorted(pcb.footprints.items())}


def courtyard(path, ref):
    """The written board's courtyard rect for `ref`, re-derived from the file
    by the same code the grader uses -- never read off the JSON."""
    pcb = parse_kicad_pcb(path)
    st = pose_score.make_state(pcb, path, clearance=0.2,
                               board_edge_clearance=0.5, grid_step=0.1)
    return st.parts[ref].rect()


def overlap(path, ref, rect):
    return legality.rect_overlap_area(courtyard(path, ref), rect)


def pairwise_legal(path):
    pcb = parse_kicad_pcb(path)
    pairs = legality.body_overlap_pairs(
        legality.graded_parts_from_file(pcb, path))
    st = pose_score.make_state(pcb, path, clearance=0.2,
                               board_edge_clearance=0.5, grid_step=0.1)
    bad = [r for r in sorted(st.parts)
           if not st.candidate_valid(r, st.parts[r].x, st.parts[r].y,
                                     st.parts[r].rot, exclude=set())]
    return (not pairs and not bad,
            f"courtyard overlaps {[(q.a, q.b) for q in pairs]}, "
            f"candidate_valid rejects {bad}")


def graded_keepout_errors(board_path, intent):
    pcb = parse_kicad_pcb(board_path)
    res = floorplan.grade(floorplan.intent_from_dict(dict(intent), board_path),
                          pcb, board_path, clearance=0.2,
                          board_edge_clearance=0.5)
    return [v for v in res.errors if v.rule == 'keepout']


# --------------------------------------------------------------------------
# A/B -- the part is displaced OUT of the keep-out, and minimally
# --------------------------------------------------------------------------

def arm_displaced_and_its_control(wd):
    """THE THEOREM. U1's courtyard is 4x4 (half 2.0). The zone
    [2, 2, 18, 10] at tol 0.5 confines the centre to x in [3.5, 16.5],
    y in [3.5, 8.5]; the target is the zone centre (10, 6).

    Clearing KO = [8, 4, 12, 8] needs cx <= 6 or cx >= 14 or cy <= 2 or
    cy >= 10. The two cy branches are outside the feasible box, so the
    minimum displacement from (10, 6) is EXACTLY 4.0mm, at (6, 6) or (14, 6).

    That exact number is the assertion a wrong implementation cannot produce:
    checking keep-outs only in `_try_place`'s whole-board fallback sweep
    would seat at a FALLBACK_STEP_MM = 2.0 lattice point, and refusing rather
    than searching would leave the part unseated.
    """
    r, s, out = run(wd, one_part_board,
                    wide_zone_intent(keepouts=[
                        {"name": "hot", "rect": list(KO), "sides": ["F"]}]),
                    tag='A')
    check("A: the run succeeds and seats U1",
          r.returncode == 0 and s and s.get('unseated') == 0,
          f"rc={r.returncode} unseated={s and s.get('unseated')}")
    if not (s and s.get('unseated') == 0):
        return
    ov = overlap(out, 'U1', KO)
    check("A: U1's courtyard on the WRITTEN board is clear of the keep-out",
          ov == 0.0, f"overlap {ov}mm2")
    px, py, _rot = poses(out)['U1']
    d = round(((px - 10.0) ** 2 + (py - 6.0) ** 2) ** 0.5, 3)
    check("A: and it moved the MINIMUM 4.0mm to get there", d == 4.0,
          f"seated at ({px}, {py}), {d}mm from the zone centre")
    check("A: no keepout violation when the written board is re-graded",
          not graded_keepout_errors(out, wide_zone_intent(keepouts=[
              {"name": "hot", "rect": list(KO), "sides": ["F"]}])))
    ok, det = pairwise_legal(out)
    check("A: the written board is pairwise legal", ok, det)

    # ---- the control: same board, same zone, NO keep-out declared --------
    r2, s2, out2 = run(wd, one_part_board, wide_zone_intent(), tag='Actrl')
    check("A-control: the run succeeds", r2.returncode == 0 and s2)
    if not s2:
        return
    p2 = poses(out2)['U1']
    check("A-control: U1 lands on the zone centre", (p2[0], p2[1]) == (10.0, 6.0),
          f"at {p2}")
    ov2 = overlap(out2, 'U1', KO)
    check("A-control: and it DOES sit in the rect, so the keep-out is what "
          "moved it in arm A", ov2 > 0.0, f"overlap {ov2}mm2")


# --------------------------------------------------------------------------
# C/D -- a keep-out that STRANDS a part is NAMED
# --------------------------------------------------------------------------

#: A keep-out covering the whole board. Stranding a part takes this much:
#: a keep-out that merely covers its ZONE does not strand it, because stage 3
#: legitimately re-tries the part at its connectivity centroid with no zone
#: constraint and finds a legal pose outside. That is correct behaviour (the
#: grade then reports zone_containment, which is a different finding), so the
#: stranding arms have to remove every pose on the board, not merely the
#: zoned ones.
WHOLE_BOARD = (-1.0, -1.0, SIZE[0] + 1.0, SIZE[1] + 1.0)


def arm_stranded_and_its_control(wd):
    """A keep-out over the entire board leaves U1 no legal pose ANYWHERE, so
    it is genuinely stranded rather than merely pushed out of its zone.

    A SECOND keep-out is declared in a corner, so "name them all" and "name
    the first one" are both wrong answers and the assertion can be on the
    exact set. `cold` binds U1 too, but lifting it frees nothing -- which is
    precisely what makes the naming a MEASUREMENT rather than a declaration
    lookup."""
    ko = [{"name": "hot", "rect": list(WHOLE_BOARD), "sides": ["F"]},
          {"name": "cold", "rect": [0.6, 0.6, 2.0, 2.0], "sides": ["F"]}]
    tight = (7.5, 3.5, 12.5, 8.5)
    r, s, out = run(wd, one_part_board,
                    wide_zone_intent(keepouts=ko, zone=tight), tag='C')
    check("C: U1 is reported unseated, not seated in the keep-out",
          s and s.get('unseated_refs') == ['U1'],
          f"unseated_refs={s and s.get('unseated_refs')}")
    if not s:
        return
    v = (s.get('no_pose_verdict') or {}).get('U1')
    check("C: the verdict is keepout_blocks, not no_movable_neighbour",
          v == 'keepout_blocks', f"verdict={v!r}")
    cen = (s.get('no_pose_census') or {}).get('U1') or {}
    freeing = cen.get('keepouts_freeing')
    check("C: exactly the RESPONSIBLE keep-out is named",
          sorted(freeing or {}) == ['hot'],
          f"keepouts_freeing={freeing!r}")
    check("C: the census says 0 poses with it, and MORE without it -- a "
          "count from the seat predicate, not a static zone-vs-rect test",
          cen.get('baseline') == 0 and (freeing or {}).get('hot', 0) > 0,
          f"baseline={cen.get('baseline')} freeing={freeing!r}")
    named = [ln for ln in r.stdout.splitlines()
             if 'U1' in ln and 'hot' in ln and 'keep-out' in ln.lower()]
    check("C: and the stdout NOTE names it too -- the JSON verdict and the "
          "prose come from one function, so they cannot drift",
          bool(named),
          named[0].strip() if named else "no line names U1 and 'hot'")

    # ---- the control: the same impossible-looking zone, no keep-out ------
    r2, s2, out2 = run(wd, one_part_board,
                       wide_zone_intent(zone=tight), tag='Cctrl')
    check("C-control: with no keep-out declared U1 IS seated -- so arm C is "
          "not just an impossible board",
          s2 and s2.get('unseated') == 0,
          f"unseated={s2 and s2.get('unseated')}")
    if s2 and s2.get('unseated') == 0:
        ov = overlap(out2, 'U1', KO)
        check("C-control: and it sits in the rect", ov > 0.0,
              f"overlap {ov}mm2")


# --------------------------------------------------------------------------
# E -- the `allow` glob owner exemption, self-controlled
# --------------------------------------------------------------------------

def arm_allow_owner_exemption(wd):
    """A mounting-hole keep-out must not strand its own mounting hole -- the
    mirror of the bug being fixed, and strictly worse, since the part then
    has no legal pose anywhere.

    Two 2x2 parts with single-ref zones, and one keep-out over the WHOLE
    board (see WHOLE_BOARD: a keep-out that covers only the zones does not
    strand anything, because stage 3 re-tries unzoned). MH1 is exempted by
    the GLOB `MH*` -- a glob, not the exact ref, so an `==` implementation
    dies here too; R1 is not. Neither arm passes under a wrong
    implementation: `allow` ignored strands MH1, `allow` matching everything
    seats R1."""
    hot = [-1.0, -1.0, 31.0, 13.0]

    def two_part_board(path):
        board(path, [_part('MH1', 10, 6, 1.0, 1.0, 2),
                     _part('R1', 20, 6, 1.0, 1.0, 2)], size=(30.0, 12.0))

    it = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
          "envelope": {"rect": [0.0, 0.0, 30.0, 12.0], "tolerance_mm": 0.5},
          "blocks": [{"name": "m", "refs": ["MH1"],
                      "zone": [8.0, 4.5, 12.0, 7.5], "tolerance_mm": 0.5},
                     {"name": "r", "refs": ["R1"],
                      "zone": [18.0, 4.5, 22.0, 7.5], "tolerance_mm": 0.5}],
          "keepouts": [{"name": "rib", "rect": hot, "sides": ["F"],
                        "allow": ["MH*"]}]}
    r, s, out = run(wd, two_part_board, it, tag='E')
    check("E: the exempt part IS seated inside the keep-out",
          s and 'MH1' not in (s.get('unseated_refs') or []),
          f"unseated_refs={s and s.get('unseated_refs')}")
    if s and 'MH1' not in (s.get('unseated_refs') or []):
        ov = overlap(out, 'MH1', tuple(hot))
        check("E: ... and its courtyard really is in the rect", ov > 0.0,
              f"overlap {ov}mm2")
    check("E: the NON-exempt part in the same keep-out is not",
          s and s.get('unseated_refs') == ['R1'],
          f"unseated_refs={s and s.get('unseated_refs')}")
    check("E: the grade agrees -- no keepout error for the exempt part",
          not [v for v in graded_keepout_errors(out, it) if v.ref == 'MH1'])


# --------------------------------------------------------------------------
# F -- a through-hole part is in a keep-out from EITHER side
# --------------------------------------------------------------------------

def arm_through_hole_from_the_other_side(wd):
    """`rule_keepout` grades a THT part against a keep-out on the face its
    BODY is not on, because its leads pass through. The seat predicate must
    agree, or the seeder places a part the grade then flags.

    Three arms, because the side logic has to be shown non-vacuous: a THT
    part on B is refused by an F-side keep-out; the same board with no
    keep-out seats it; and an SMD-only part on B is SEATED inside the same
    F-side keep-out, because it genuinely does not occupy F."""
    tight = (7.5, 3.5, 12.5, 8.5)
    ko = [{"name": "hot", "rect": list(WHOLE_BOARD), "sides": ["F"]}]

    def tht_board(path):
        board(path, [_part('D1', 10, 6, 2.0, 2.0, 4, layer='B.Cu',
                           thru=True)], size=SIZE)

    def smd_b_board(path):
        board(path, [_part('D1', 10, 6, 2.0, 2.0, 4, layer='B.Cu')],
              size=SIZE)

    it = wide_zone_intent(keepouts=ko, zone=tight, refs=('D1',))
    r, s, out = run(wd, tht_board, it, tag='Fa')
    check("F-a: a THT part on B is refused by an F-side keep-out",
          s and s.get('unseated_refs') == ['D1'],
          f"unseated_refs={s and s.get('unseated_refs')}")
    if s:
        check("F-a: ... named as a keep-out refusal",
              (s.get('no_pose_verdict') or {}).get('D1') == 'keepout_blocks',
              f"verdict={(s.get('no_pose_verdict') or {}).get('D1')!r}")

    r2, s2, out2 = run(wd, tht_board,
                       wide_zone_intent(zone=tight, refs=('D1',)), tag='Fb')
    check("F-b (control): with no keep-out the same THT part IS seated",
          s2 and s2.get('unseated') == 0,
          f"unseated={s2 and s2.get('unseated')}")

    r3, s3, out3 = run(wd, smd_b_board, it, tag='Fc')
    seated = s3 and s3.get('unseated') == 0
    check("F-c: an SMD-only part on B is SEATED inside the same F-side "
          "keep-out -- the side test is not vacuous", seated,
          f"unseated={s3 and s3.get('unseated')}")
    if seated:
        ov = overlap(out3, 'D1', tuple(WHOLE_BOARD))
        check("F-c: ... and its courtyard really is inside the rect",
              ov > 0.0, f"overlap {ov}mm2")


# --------------------------------------------------------------------------
# G -- the edge-connector seat, which bypasses pose_ok by design
# --------------------------------------------------------------------------

def arm_edge_connector(wd):
    """`_seat_edge` and stage 1 use `edge_seat_ok`, not `pose_ok`. A PR that
    guarded only `pose_ok` leaves this red, and nothing else in the suite
    catches it -- which is the whole reason this arm exists.

    J1 is a declared south-edge connector on a 20x14 board. The keep-out
    covers the south band around x = 10, so the along-edge slide has to move
    it; the control shows where it lands without one."""
    size = (20.0, 14.0)
    edgeko = [7.0, 10.0, 13.0, 14.0]

    def edge_board(path):
        board(path, [_part('J1', 10, 7, 2.0, 1.5, 2, pad_y=-1.0)], size=size)

    def it(with_ko):
        d = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
             "envelope": {"rect": [0.0, 0.0, size[0], size[1]],
                          "tolerance_mm": 0.5},
             "blocks": [{"name": "b", "refs": ["J1"]}],
             "edge_connectors": [{"ref": "J1", "edge": "south",
                                  "overhang_mm": {"min": 0.0, "max": 1.0}}]}
        if with_ko:
            d["keepouts"] = [{"name": "shell", "rect": edgeko,
                              "sides": ["F"]}]
        return d

    r0, s0, out0 = run(wd, edge_board, it(False), tag='Gctrl')
    check("G-control: the edge connector is seated with no keep-out",
          s0 and s0.get('unseated') == 0,
          f"unseated={s0 and s0.get('unseated')}")
    if not (s0 and s0.get('unseated') == 0):
        return
    ov0 = overlap(out0, 'J1', tuple(edgeko))
    check("G-control: and it lands INSIDE the rect the keep-out will cover, "
          "so arm G's assertion is not vacuous", ov0 > 0.0,
          f"overlap {ov0}mm2, pose {poses(out0)['J1']}")

    r1, s1, out1 = run(wd, edge_board, it(True), tag='G')
    check("G: the run completes with the keep-out declared",
          r1.returncode in (0, 4), f"rc={r1.returncode}")
    ov1 = overlap(out1, 'J1', tuple(edgeko))
    check("G: the edge seat is clear of the keep-out on the WRITTEN board "
          "-- this is the arm that reverting the edge_seat_ok conjunct "
          "turns red", ov1 == 0.0,
          f"overlap {ov1}mm2, pose {poses(out1)['J1']}")
    check("G: and the grade agrees", not graded_keepout_errors(out1, it(True)))
    # The band was given up, which is the honest outcome -- but it must be
    # DISCLOSED, and disclosed with the keep-out's name. A silent fallback to
    # the ordinary stages is how a connector quietly stops reaching its edge.
    named = [ln for ln in r1.stdout.splitlines()
             if 'J1' in ln and 'shell' in ln]
    check("G: the refusal is disclosed and NAMES the keep-out, not the "
          "outline", bool(named),
          named[0].strip() if named else "no line names both J1 and 'shell'")


# --------------------------------------------------------------------------
# H -- the polish quench has no keep-out term; the self-check repairs it
# --------------------------------------------------------------------------

#: Fault injection for arm H-a. The quench on these tiny fixtures happens to
#: leave the part alone, so a plain end-to-end run never reaches the repair
#: and would pass with the repair deleted -- a test that passes in both
#: directions. This sitecustomize forces the polish to return the one move
#: that puts U1 back in the keep-out, which is precisely what a real quench
#: with no keep-out term is free to do.
_INJECT = '''
import placement.quench as _q
_real = _q.quench


def _walk_it_in(*a, **kw):
    _real(*a, **kw)
    return [{'reference': 'U1', 'new_x': 10.0, 'new_y': 6.0,
             'new_rotation': 0.0}]


_q.quench = _walk_it_in
'''


def arm_polish_repair(wd):
    """`place_seed` runs a polish quench after seeding, and the quench has NO
    keep-out term -- `grep keepout py_placer/placement/quench.py` finds
    nothing relevant. So the polish is free to walk a part back into a
    declared keep-out, and `place_seed` then exits 4 against its own intent on
    a board its own seeder placed correctly.

    H-a injects exactly that fault and requires the repair to undo it. H-b is
    the ordinary end-to-end run."""
    ko = [{"name": "hot", "rect": list(KO), "sides": ["F"]}]

    # ---- H-a: the polish IS forced to break it -------------------------
    inj = os.path.join(wd, 'inj')
    os.makedirs(inj, exist_ok=True)
    with open(os.path.join(inj, 'sitecustomize.py'), 'w',
              encoding='utf-8') as f:
        f.write(_INJECT)
    bpath = os.path.join(wd, 'Ha-in.kicad_pcb')
    ipath = os.path.join(wd, 'Ha-fp.json')
    out_a = os.path.join(wd, 'Ha-out.kicad_pcb')
    one_part_board(bpath)
    with open(ipath, 'w', encoding='utf-8') as f:
        json.dump(wide_zone_intent(keepouts=ko), f)
    env = dict(os.environ, PYTHONHASHSEED='0', PYTHONIOENCODING='utf-8',
               PYTHONPATH=inj + os.pathsep
               + os.pathsep.join(os.path.join(REPO, d) for d in
                                 ('py_placer', 'py_router', 'py_tools')))
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join('py_placer', 'place_seed.py'), bpath, out_a,
         '--intent', ipath, '--clearance', '0.2',
         '--board-edge-clearance', '0.5', '--force'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=REPO, timeout=900, env=env)
    injected = 'polish walked' in r.stdout
    check("H-a: the injected polish really did break the board, so this arm "
          "is not vacuous", injected,
          "the repair never fired -- the injection did not take effect")
    if injected:
        check("H-a: the repair names `keepout` as what it repaired",
              'keepout' in r.stdout, "repair line does not mention keepout")
        ov = overlap(out_a, 'U1', KO)
        check("H-a: and U1 is back out of the keep-out on the WRITTEN board",
              ov == 0.0, f"overlap {ov}mm2, pose {poses(out_a)['U1']}")
        check("H-a: place_seed exits 0 after repairing itself",
              r.returncode == 0, f"rc={r.returncode}")

    # ---- H-c: the same fault on a part with NO ZONE ---------------------
    # A keep-out violation does not imply a zone, and most parts on most
    # boards have none. The repair loop originally required one, which would
    # have made the keep-out half inert exactly there while reading as
    # implemented -- so this arm declares a block with members and no `zone`.
    ipath_c = os.path.join(wd, 'Hc-fp.json')
    out_c = os.path.join(wd, 'Hc-out.kicad_pcb')
    bpath_c = os.path.join(wd, 'Hc-in.kicad_pcb')
    one_part_board(bpath_c)
    with open(ipath_c, 'w', encoding='utf-8') as f:
        json.dump({"schema": 1, "kind": "floorplan-intent", "units": "mm",
                   "envelope": {"rect": [0.0, 0.0, SIZE[0], SIZE[1]],
                                "tolerance_mm": 0.5},
                   "blocks": [{"name": "b", "refs": ["U1"]}],
                   "keepouts": [dict(k) for k in ko]}, f)
    rc_ = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join('py_placer', 'place_seed.py'), bpath_c, out_c,
         '--intent', ipath_c, '--clearance', '0.2',
         '--board-edge-clearance', '0.5', '--force'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=REPO, timeout=900, env=env)
    fired = 'polish walked' in rc_.stdout
    check("H-c: the repair fires for a part with NO declared zone -- a "
          "keep-out violation does not imply a zone", fired,
          "the repair skipped it; the zone-less branch is inert")
    if fired:
        ovc = overlap(out_c, 'U1', KO)
        check("H-c: and U1 is out of the keep-out on the WRITTEN board",
              ovc == 0.0, f"overlap {ovc}mm2, pose {poses(out_c)['U1']}")

    # ---- H-b: the ordinary run ------------------------------------------
    r2, s2, out2 = run(wd, one_part_board, wide_zone_intent(keepouts=ko),
                       tag='Hb', polish=True)
    check("H-b: place_seed exits 0 with the polish enabled",
          r2.returncode == 0, f"rc={r2.returncode}")
    check("H-b: no grade error in its own self-check",
          s2 and s2.get('grade_errors') == 0,
          f"grade_errors={s2 and s2.get('grade_errors')}")
    ov2 = overlap(out2, 'U1', KO)
    check("H-b: the polished board is clear of the keep-out", ov2 == 0.0,
          f"overlap {ov2}mm2")


def main():
    with tempfile.TemporaryDirectory() as wd:
        for fn in (arm_displaced_and_its_control,
                   arm_stranded_and_its_control,
                   arm_allow_owner_exemption,
                   arm_through_hole_from_the_other_side,
                   arm_edge_connector,
                   arm_polish_repair):
            print(f"--- {fn.__name__}")
            fn(wd)
    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
