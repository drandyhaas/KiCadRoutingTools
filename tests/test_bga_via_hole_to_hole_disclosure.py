#!/usr/bin/env python3
"""Regression test for issue #618: BGA via-in-pad drill spacing is the ball
pitch, and a declared min_hole_to_hole moves NOTHING -- so the run must at
least SAY SO.

  python3 tests/test_bga_via_hole_to_hole_disclosure.py

A BGA via-in-pad sits at the ball centre, so the drill spacing the fanout can
reach is fixed by the package: `pitch - drill`. On a 0.8mm array with a 0.2mm
drill that is 0.600mm. A board declaring `min_hole_to_hole` 0.9 therefore gets
copper it cannot build -- and before this fix it got it SILENTLY: nothing in
the log, and nothing in the JSON summary either (`failed` counts balls that
found no escape, which is a different question).

Measured on the unpatched tree (glasgow_revC U30, escape_method='channel',
declared 0.9): 76 vias, 307 tracks, 1 failed, min drill gap 0.600mm, 114 via
pairs below the declared rule, and the emitted copper BYTE-FOR-BYTE identical
to the same run with nothing declared -- the declaration was inert in every
respect including the log.

This test pins the DISCLOSURE, not enforcement. Enforcing the floor takes the
violations to 0 and costs escapes (measured at declared 0.9: ulx3s U1 on the
default engine 186 -> 171 escaped / 21 -> 33 failed; glasgow_revC U30 on
channel 98 -> 63 escaped / 1 -> 36 failed), which is a policy call for the
board owner; see #618. What is pinned here:

  - a board declaring nothing is SILENT (no #618 line) and its report says so,
  - a board declaring 0.9 gets a report naming the violating BALL PADS, a
    WARNING on stdout, and the SAME copper as the silent run (report-only),
  - the default escape_method='auto' path discloses too, and the finding is
    an order of magnitude larger than the run's `failed` ledger, so `failed`
    was never going to be how a caller heard about it,
  - a 0.5mm-pitch part (orangecrab U3) discloses at a declared 0.4,
  - the CLI's JSON_SUMMARY carries `hole_to_hole` with the named pads,
  - the audit itself is pure: same input, same answer, nothing mutated,
  - and the new-vs-EXISTING-hole half is graded on a board that really has
    holes (routed_output: 383 via drills): a via clearing a real
    board hole is silent, a via OVERLAPPING one is reported with a negative
    gap (the worst violation there is -- an earlier draft dropped every
    negative gap as "co-located" and reported such a board as clean), and a
    via re-emitted exactly AT an existing hole is the same hole, not a pair.

Skips cleanly when a board is absent.
"""
import io
import json
import contextlib
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb
import bga_fanout as bf
from bga_fanout import generate_bga_fanout

GLASGOW = os.path.join(ROOT, "kicad_files", "glasgow_revC.kicad_pcb")
ORANGECRAB = os.path.join(ROOT, "kicad_files", "orangecrab_ext_pll.kicad_pcb")
SMALL = os.path.join(ROOT, "kicad_files", "qfn_interior_pads.kicad_pcb")
ROUTED = os.path.join(ROOT, "kicad_files", "routed_output.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
PARAMS = dict(track_width=0.12, clearance=0.1, via_size=0.35, via_drill=0.2)

CHECKS = []


def check(name, ok, detail=""):
    CHECKS.append((name, bool(ok)))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def _staged(tmp, board, h2h):
    """Copy `board` (+ its siblings) into tmp, declaring min_hole_to_hole."""
    dst = os.path.join(tmp, os.path.basename(board))
    shutil.copyfile(board, dst)
    pro_src = os.path.splitext(board)[0] + '.kicad_pro'
    data = {}
    if os.path.exists(pro_src):
        with open(pro_src, encoding='utf-8') as f:
            data = json.load(f)
    if h2h is not None:
        data.setdefault('board', {}).setdefault('design_settings', {}) \
            .setdefault('rules', {})['min_hole_to_hole'] = h2h
        with open(os.path.splitext(dst)[0] + '.kicad_pro', 'w',
                  encoding='utf-8') as f:
            json.dump(data, f)
    elif os.path.exists(pro_src):
        shutil.copyfile(pro_src, os.path.splitext(dst)[0] + '.kicad_pro')
    return dst


def fanout(board, ref, h2h, method):
    """Run the shared engine on a staged copy. Returns (report, said, copper)."""
    with tempfile.TemporaryDirectory() as tmp:
        pcb = parse_kicad_pcb(_staged(tmp, board, h2h))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            tracks, vias, _rm, failed = generate_bga_fanout(
                pcb.footprints[ref], pcb, layers=LAYERS,
                escape_method=method, **PARAMS)
        report = dict(getattr(bf, 'LAST_HOLE_TO_HOLE_REPORT', {}) or {})
        copper = hashlib.sha256(repr(
            sorted((round(v['x'], 6), round(v['y'], 6), v.get('size'),
                    v.get('drill'), v.get('net_id')) for v in vias)
            + sorted((t['start'], t['end'], t.get('layer'), t.get('net_id'))
                     for t in tracks)).encode()).hexdigest()[:16]
        return report, buf.getvalue(), copper, len(vias), len(tracks), len(failed)


def real_board_checks():
    if not os.path.exists(GLASGOW):
        print(f"  [SKIP] board not present: {GLASGOW}")
        return

    # --- CONTROL: the board declares nothing -------------------------------
    rep0, said0, cu0, nv0, nt0, nf0 = fanout(GLASGOW, "U30", None, 'channel')
    check("control: a board declaring nothing gets no #618 warning",
          '#618' not in said0, f"{nv0} vias, {nt0} tracks")
    check("control: ...and the report grades it at the fab default, 0 violations",
          rep0.get('source') == 'fixed default' and rep0.get('violations') == 0,
          f"source={rep0.get('source')} floor={rep0.get('floor')} "
          f"violations={rep0.get('violations')}")

    # --- THE DEFECT: the same run, with 0.9 declared ------------------------
    rep9, said9, cu9, nv9, nt9, nf9 = fanout(GLASGOW, "U30", 0.9, 'channel')
    check("#618 the declared 0.9 moves NO copper (the defect, unchanged)",
          cu9 == cu0 and (nv9, nt9, nf9) == (nv0, nt0, nf0),
          f"{nv9} vias / {nt9} tracks / {nf9} failed, sha {cu9} vs {cu0}")
    check("#618 ...and the drills really are below it",
          rep9.get('min_gap') is not None and rep9['min_gap'] < 0.9 - 1e-9,
          f"min drill gap {rep9.get('min_gap')}mm vs declared 0.9")
    check("#618 the run now DISCLOSES it in the report",
          rep9.get('violations', 0) > 0 and rep9.get('floor') == 0.9
          and rep9.get('source') == 'board constraint',
          f"{rep9.get('violations')} violating pairs at floor "
          f"{rep9.get('floor')} from {rep9.get('source')}")
    check("#618 ...and on stdout, as a WARNING",
          '#618' in said9 and 'UNMANUFACTURABLE' in said9)
    # The pads, by name -- the whole point of the disclosure.
    pads = rep9.get('pads') or []
    worst = rep9.get('worst') or []
    check("#618 ...naming the BALL PADS, not just a count",
          len(pads) >= 2 and all(p.startswith('U30/') for p in pads),
          f"{len(pads)} pads, e.g. {pads[:4]}")
    # ...capped, so a 900-ball part cannot swamp the summary line, with the
    # true total kept alongside.
    check("#618 ...capped at 24 names, with the true total in pad_count",
          len(pads) <= 24 and rep9.get('pad_count', 0) > len(pads)
          and rep9.get('by_kind', {}).get('ball_to_ball', 0) > 0,
          f"pads={len(pads)} pad_count={rep9.get('pad_count')} "
          f"by_kind={rep9.get('by_kind')}")
    check("#618 ...and the worst pairs carry both ends and the gap",
          bool(worst) and all({'gap', 'a', 'b'} <= set(w) for w in worst)
          and worst[0]['gap'] == rep9.get('min_gap'),
          f"worst[0]={worst[0] if worst else None}")
    check("#618 ...with at least one named pad printed in the WARNING",
          any(p in said9 for p in pads[:8]))

    # --- The DEFAULT engine ('auto') is the one users actually hit ----------
    repA, saidA, _cuA, nvA, _ntA, nfA = fanout(GLASGOW, "U30", 0.9, 'auto')
    check("#618 escape_method='auto' (the DEFAULT) discloses too",
          repA.get('violations', 0) > 0 and '#618' in saidA,
          f"{nvA} vias, {repA.get('violations')} violating pairs, "
          f"min gap {repA.get('min_gap')}mm")
    # The routing verdict cannot be how a caller learns about this: the run
    # EMITTED every one of these vias. `failed` counts balls that got no
    # escape, and it is an order of magnitude smaller than the finding.
    check("#618 ...and the failure ledger does not carry it",
          repA.get('violations', 0) > max(nfA, 1) * 5,
          f"{repA.get('violations')} violating pairs vs failed={nfA}")

    # --- A 0.5mm-pitch part, the issue's "fine-pitch BGA" ------------------
    if os.path.exists(ORANGECRAB):
        rep4, said4, _c4, nv4, _t4, _f4 = fanout(ORANGECRAB, "U3", 0.4, 'channel')
        check("#618 a 0.5mm-pitch BGA (orangecrab U3) discloses at 0.4",
              rep4.get('violations', 0) > 0
              and rep4.get('min_gap', 9) < 0.4 - 1e-9 and '#618' in said4,
              f"{nv4} vias, {rep4.get('violations')} pairs, "
              f"min gap {rep4.get('min_gap')}mm")
    else:
        print(f"  [SKIP] board not present: {ORANGECRAB}")


def unit_checks():
    """The audit itself, on hand-placed holes -- exact, and no routing."""
    if not os.path.exists(SMALL):
        print(f"  [SKIP] board not present: {SMALL}")
        return
    audit_via_hole_to_hole = getattr(bf, 'audit_via_hole_to_hole', None)
    if audit_via_hole_to_hole is None:
        # Unpatched tree: the audit does not exist. Record it as the failure
        # it is and let the real-board checks below report the SYMPTOM.
        check("unit: bga_fanout exposes audit_via_hole_to_hole", False,
              "no such symbol -- the engine cannot disclose anything")
        return

    def audit(h2h, vias):
        with tempfile.TemporaryDirectory() as tmp:
            pcb = parse_kicad_pcb(_staged(tmp, SMALL, h2h))
            fp = next(iter(pcb.footprints.values()))
            with contextlib.redirect_stdout(io.StringIO()):
                return audit_via_hole_to_hole(fp, pcb, vias), fp

    # Two holes 0.6mm apart, drill 0.2 -> 0.4mm edge-to-edge, far from any
    # copper on the fixture (offset well outside the board's own geometry).
    def pair(dx):
        return [{'x': 500.0, 'y': 500.0, 'size': 0.35, 'drill': 0.2, 'net_id': 0},
                {'x': 500.0 + dx, 'y': 500.0, 'size': 0.35, 'drill': 0.2,
                 'net_id': 0}]

    rep, _fp = audit(0.5, pair(0.6))
    check("unit: 0.4mm edge-to-edge under a declared 0.5 is one violation",
          rep['violations'] == 1 and abs(rep['min_gap'] - 0.4) < 1e-9
          and rep['floor'] == 0.5, f"{rep}")
    # `pads` promises BALL PADS. These holes are nowhere near a ball, so the
    # list must be EMPTY rather than carrying a coordinate string -- the pair
    # is still fully described in `worst`.
    check("unit: `pads` names ball pads only, never a bare coordinate",
          rep['pads'] == [] and rep['pad_count'] == 0
          and rep['worst'][0]['a'].startswith('(')
          and rep['by_kind'] == {'ball_to_ball': 0, 'new_to_new': 1,
                                 'against_existing': 0}, f"{rep}")
    rep, _fp = audit(None, pair(0.6))
    check("unit: the same holes are legal at the fab default 0.2",
          rep['violations'] == 0 and rep['floor'] == 0.2, f"{rep}")
    rep, _fp = audit(0.5, pair(0.75))
    check("unit: 0.55mm edge-to-edge clears the declared 0.5",
          rep['violations'] == 0 and abs(rep['min_gap'] - 0.55) < 1e-9, f"{rep}")
    # A declaration BELOW the fab floor is pinned up, never obeyed downwards.
    rep, _fp = audit(0.05, pair(0.22))
    check("unit: a sub-fab declaration is wrapped at the fab hole-to-hole floor",
          rep['floor'] >= 0.2 - 1e-9 and rep['violations'] == 1,
          f"floor={rep['floor']} violations={rep['violations']}")
    # Purity: the audit must not touch the list it is given.
    vias = pair(0.6)
    before = json.dumps(vias, sort_keys=True)
    r1, _fp = audit(0.5, vias)
    r2, _fp = audit(0.5, vias)
    check("unit: the audit is pure (input untouched, answer stable)",
          json.dumps(vias, sort_keys=True) == before
          and r1['violations'] == r2['violations']
          and r1['min_gap'] == r2['min_gap'])
    # No vias at all -> a report, never a crash.
    rep, _fp = audit(0.5, [])
    check("unit: an empty via list reports 0, not an error",
          rep.get('violations') == 0 and 'error' not in rep, f"{rep}")


def existing_hole_checks():
    """The new-vs-EXISTING-hole half, on a board that HAS holes.

    The unit checks above hand-place their holes far outside the fixture, so
    `existing` is empty and this whole branch never runs there. routed_output
    carries 383 real via drills, so the branch is exercised for what it must
    get right: a legal gap is silent, an OVERLAP is reported (an earlier draft
    dropped every negative gap as "co-located" and reported a board with
    intersecting holes as clean, `min_gap` and all), and a via re-emitted
    exactly at an existing hole is that same hole, not a pair.
    """
    if not os.path.exists(ROUTED):
        print(f"  [SKIP] board not present: {ROUTED}")
        return
    audit_via_hole_to_hole = getattr(bf, 'audit_via_hole_to_hole', None)
    if audit_via_hole_to_hole is None:
        check("unit: a new via overlapping an existing board hole is reported",
              False, "no audit_via_hole_to_hole symbol")
        return
    with tempfile.TemporaryDirectory() as tmp:
        pcb = parse_kicad_pcb(_staged(tmp, ROUTED, 0.5))
        fp = pcb.footprints['U3']

        def audit(vias):
            with contextlib.redirect_stdout(io.StringIO()):
                return audit_via_hole_to_hole(fp, pcb, list(vias))

        holes = [(v.x, v.y, (v.drill or 0) / 2.0)
                 for v in pcb.vias if (v.drill or 0) > 0]
        n_via_holes = len(holes)
        for pads in pcb.pads_by_net.values():
            for p in pads:
                if (getattr(p, 'drill', 0) or 0) > 0:
                    holes.append((p.global_x, p.global_y, p.drill / 2.0))
        floor = audit([{'x': 500.0, 'y': 500.0, 'size': 0.35, 'drill': 0.2,
                        'net_id': 0}])['floor']
        nr = 0.1                                    # the new via's drill / 2
        # An ISOLATED existing via, so the assertions can be exact counts. The
        # audit's prefilter box around a new via is floor + radii + 1mm, so
        # reject any candidate with a SECOND hole inside that box once the new
        # via sits at its furthest test offset.
        iso = None
        for (x, y, r) in holes[:n_via_holes]:
            far = r + nr + floor + 0.05             # the legal-case offset
            if all(abs(hx - (x + far)) > floor + nr + hr + 1.0
                   or abs(hy - y) > floor + nr + hr + 1.0
                   for (hx, hy, hr) in holes if (hx, hy) != (x, y)):
                iso = (x, y, r)
                break
        if iso is None:
            print("  [SKIP] no isolated existing via on the fixture")
            return
        vx, vy, vr = iso

        def one(dx):
            return [{'x': vx + dx, 'y': vy, 'size': 0.35, 'drill': 2 * nr,
                     'net_id': 0}]
        legal = audit(one(vr + nr + floor + 0.05))
        check("unit: a new via clearing a REAL board hole by 0.05mm is silent",
              legal['violations'] == 0
              and abs(legal['min_gap'] - (floor + 0.05)) < 1e-6,
              f"{n_via_holes} board vias, isolated one at "
              f"({vx:.3f},{vy:.3f}); {legal}")
        over = audit(one((vr + nr) * 0.5))          # centres inside each other
        worst_b = (over.get('worst') or [{}])[0].get('b', '')
        check("unit: a new via OVERLAPPING a real board hole is REPORTED",
              over['violations'] == 1 and (over['min_gap'] or 0) < 0
              and over['by_kind']['against_existing'] == 1
              and worst_b.startswith('an existing via @('),
              f"{over}")
        same = audit(one(0.0))                      # the SAME hole, re-emitted
        check("unit: a via re-emitted AT an existing hole is that same hole",
              same['violations'] == 0 and same['min_gap'] is None, f"{same}")


def cli_summary_check():
    """The CLI's JSON_SUMMARY must carry the finding (the planner reads it)."""
    if not os.path.exists(ORANGECRAB):
        print(f"  [SKIP] board not present: {ORANGECRAB}")
        return
    with tempfile.TemporaryDirectory() as tmp:
        board = _staged(tmp, ORANGECRAB, 0.9)
        out = os.path.join(tmp, 'out.kicad_pcb')
        cmd = [sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'py_router', 'bga_fanout.py'), board,
               '--component', 'U4', '--escape-method', 'channel',
               '--layers'] + LAYERS + [
               '--track-width', '0.12', '--clearance', '0.1',
               '--via-size', '0.35', '--via-drill', '0.2', '-o', out]
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        line = [ln for ln in p.stdout.splitlines()
                if ln.startswith('JSON_SUMMARY:')]
        if not line:
            check("CLI JSON_SUMMARY carries hole_to_hole", False,
                  f"no JSON_SUMMARY (exit {p.returncode})")
            return
        summary = json.loads(line[-1].split('JSON_SUMMARY:', 1)[1])
        h2h = summary.get('hole_to_hole') or {}
        check("CLI JSON_SUMMARY carries the hole_to_hole finding",
              h2h.get('violations', 0) > 0 and h2h.get('floor') == 0.9
              and h2h.get('source') == 'board constraint',
              f"hole_to_hole={h2h}")
        check("CLI ...naming the pad, while `failed` still reads 0",
              any(str(p_).startswith('U4/') for p_ in (h2h.get('pads') or []))
              and summary.get('failed') == 0,
              f"pads={h2h.get('pads')} failed={summary.get('failed')}")


def main():
    print("=" * 70)
    print("Issue #618: BGA via-in-pad hole-to-hole disclosure")
    print("=" * 70)
    unit_checks()
    existing_hole_checks()
    real_board_checks()
    cli_summary_check()
    n_pass = sum(1 for _n, ok in CHECKS if ok)
    print(f"\n{n_pass}/{len(CHECKS)} checks passed")
    print("=" * 70)
    return 0 if n_pass == len(CHECKS) and CHECKS else 1


if __name__ == "__main__":
    sys.exit(main())
