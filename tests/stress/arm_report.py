#!/usr/bin/env python3
"""The recovery table, measured — not assembled by hand.

    python3 -X utf8 tests/stress/arm_report.py <workdir> --result <board> \
        [--human <the original routed board>] [--out REPORT.md]

Emits one table, one row per version of the board:

    | run | recovery | home /N | vias | copper mm | buildable |

Every number comes from the instrument that owns it. That is the whole point:
the last run's table was written by hand and three of its numbers were wrong in
the flattering direction — pad CENTRES quoted as pad copper (so every excursion
was understated and the fourth off-board part vanished from a list the same
paragraph counted as four), three of six unrepairable parts quoted, and a stop
condition claimed with two of five laps used.

TRUTH OPENS AT ONE POINT, and it is marked below. Everything before it reads
only the boards in the work dir. The control's poses arrive inside the perturb
record (`original_poses`), which is why the fence forbids that file to the run
itself and allows it here.

Two rules inherited from `placement/recovery.py`, both of which the previous
arm broke:

  * displacement is measured in PAD space, not between footprint origins. A
    part rotated in place moves its origin 0.00 mm and its pads up to 49 mm,
    so an origin-distance "home" count reports a rotated part as home. The
    correct value is already computed by `displacement_to_original`.
  * `N` is the FROZEN perturbed member list, never re-derived from the board.
    Re-deriving it scores a different set of parts at every dose.

Exit: 0 wrote the report, 2 usage/missing inputs.
"""
import argparse
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ.setdefault('KRT_NO_BANNER', '1')

SKILL_SCRIPTS = os.path.join(ROOT, '.claude', 'skills',
                             'plan-pcb-placement-and-routing', 'scripts')
PY = [sys.executable, '-X', 'utf8']


def _run(args, timeout=1800):
    p = subprocess.run(PY + args, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', cwd=ROOT,
                       timeout=timeout)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def quality(board):
    """vias / copper_mm / segments, from board_score's own helper."""
    sys.path.insert(0, SKILL_SCRIPTS)
    try:
        import board_score
        return board_score.quality(board)
    except Exception as exc:                                # noqa: BLE001
        return {'error': f'{type(exc).__name__}: {exc}'}


def blocking(board, clearance, tmp):
    """board_score's `blocking_by`, i.e. the ROUTED OUTCOME with its work list.

    `unrouted` and `broken` are the two numbers the objective is stated in, and
    this report has never printed either. Its `vias`/`copper_mm` columns are
    board_score's `quality`, which that tool's own doctrine says is a tiebreak
    comparable ONLY once `blocking == 0` -- so the two routing-looking columns
    were the two that mean nothing until the number nobody printed is zero."""
    jp = os.path.join(tmp, os.path.basename(board) + '.score.json')
    args = [os.path.join(SKILL_SCRIPTS, 'board_score.py'), board,
            '--clearance', str(clearance), '--json', jp]
    code, out = _run(args)
    if not os.path.isfile(jp):
        return {'error': f'board_score exit {code}', 'log': out[-300:]}
    with open(jp, encoding='utf-8') as fh:
        doc = json.load(fh)
    return {'blocking': doc.get('blocking'),
            'by': doc.get('blocking_by') or {},
            'ungraded': doc.get('ungraded') or [],
            'unrouted_shape': (doc.get('components') or {}).get('unrouted', {}),
            'quality': doc.get('quality') or {}}


def route(board, record, clearance, tmp):
    """PRR / RR over the FROZEN denominators -- the continuous channel.

    `score_board`'s `route` block has existed, been tested and been DEAD since
    it was written: `routed_path` has never once been supplied by any caller
    (`perturb.py:556`, `perturb_batch.py:313,342`), so every record ever
    written carries `route: {ran: False}`. The result board IS the routed
    board, so `routed_path` is simply the board being reported on.

    It is printed BESIDE `unrouted`/`broken`, never instead of them: they
    disagree by construction (check_connected skips nets with no copper at all;
    connectivity_tally keeps them in the denominator) and the gap is
    informative. A counts-only headline also cannot tell "one net short" from
    "half the board", which is exactly the distinction a recovery run needs."""
    from placement import recovery as R
    from kicad_parser import parse_kicad_pcb
    try:
        jp = os.path.join(tmp, os.path.basename(board) + '.drc.json')
        _run(['py_router/check_drc.py', board, '--clearance', str(clearance),
              '--json', jp])
        dirty = R.dirty_net_ids(parse_kicad_pcb(board), jp)
        return R.score_board(board, record, routed_path=board,
                             dirty_nets=dirty).get('route') or {}
    except Exception as exc:                                   # noqa: BLE001
        return {'ran': False, 'reason': f'{type(exc).__name__}: {exc}'}


def buildable(board, clearance, tmp):
    """check_assembly's own verdict, read from its JSON rather than its text."""
    jp = os.path.join(tmp, os.path.basename(board) + '.assembly.json')
    code, out = _run(['py_tools/check_assembly.py', board, '--clearance',
                      str(clearance), '--json', jp])
    if not os.path.isfile(jp):
        return {'error': f'check_assembly exit {code}', 'log': out[-300:]}
    with open(jp, encoding='utf-8') as fh:
        doc = json.load(fh)
    # `buildable`/`verdict` are keys now; recompute only for an older JSON, and
    # say so rather than silently disagreeing with the tool.
    if 'buildable' not in doc:
        doc['buildable'] = not (doc.get('blocking')
                                or doc.get('locked_contact_pairs'))
        doc['verdict'] = 'derived (tool predates the verdict key)'
    return doc


def _collateral_refs(damaged, result, members, tol=0.05):
    """[(ref, pad_mm, rot_delta)] for parts OUTSIDE the block that moved.

    `collateral_pad_rms` has been computed since this module existed and read
    by nothing, and when runs 9 and 10 finally printed it, it was a bare
    scalar. On run 10 it was the ONLY instrument that saw the run tear two
    members out of a geometrically perfect 8-fold LED ring -- and it said
    `3.6697`, with no name attached, so nobody could act on it. A number
    without a work list is not an instrument.

    IN PAD SPACE, via `recovery.part_displacement`, which is the same measure
    the scalar aggregates -- so the names and the number are in one currency,
    and a part rotated in place (origin moves 0.00 mm, pads move plenty) is not
    silently reported as unmoved. Hand-rolling an origin distance here is the
    exact defect `tests/test_run9_arm_report.py` forbids by name.

    Measured against the DAMAGED board, not the control: these are parts the
    damage never touched, so their damaged pose IS their original one, and
    reading it needs no access to truth.
    """
    from placement import recovery as R
    from kicad_parser import parse_kicad_pcb
    try:
        a, b = parse_kicad_pcb(damaged), parse_kicad_pcb(result)
    except Exception:                                          # noqa: BLE001
        return []
    was, now = R.board_poses(a), R.board_poses(b)
    block = set(members or ())
    out = []
    for ref, pose in sorted(now.items()):
        base = was.get(ref)
        fp = (b.footprints or {}).get(ref)
        if base is None or fp is None or ref in block:
            continue
        d = R.part_displacement(fp, pose, base)
        dr = (pose[2] - base[2]) % 360.0
        dr = dr - 360.0 if dr > 180.0 else dr
        if d > tol or abs(dr) > 0.5:
            out.append((ref, d, dr))
    return sorted(out, key=lambda t: -t[1])


def _fmt(v, nd=2):
    if v is None:
        return '—'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('workdir')
    ap.add_argument('--result', required=True,
                    help='the board this run produced')
    ap.add_argument('--control', default=None,
                    help='the control board (default: the sibling '
                         '<workdir>/../_truth/, else .../_truth/<subject>/, '
                         'else the legacy <workdir>/) -- used for the fence '
                         'audit only; the poses come from the perturb record')
    ap.add_argument('--record', default=None,
                    help='the perturb record (same search as --control; '
                         'a record found INSIDE the work dir is a fence leak '
                         'and the audit below will say so)')
    ap.add_argument('--damaged', default=None,
                    help='the damaged input (default <workdir>/'
                         'perturbed.kicad_pcb)')
    ap.add_argument('--human', default=None,
                    help='the original ROUTED board, for the reference row')
    ap.add_argument('--clearance', type=float, default=None,
                    help="default: the board's own Default netclass")
    ap.add_argument('--out', default=None,
                    help='default <workdir>/REPORT.md')
    a = ap.parse_args(argv)

    wd = a.workdir
    if not os.path.isdir(wd):
        print(f'no such work dir: {wd}', file=sys.stderr)
        return 2
    damaged = a.damaged or os.path.join(wd, 'perturbed.kicad_pcb')
    # TRUTH DEFAULTS LOOK OUTSIDE THE FENCE, WHICH MEANS A SIBLING.
    #
    # These used to default straight into the work dir -- the layout
    # `perturb(control_out=None)` produces and the one the staging recipe
    # exists to avoid. `fence_audit` now treats a pose RECORD inside the fence
    # as a leak the same way it already treated the control, so this tool's own
    # defaults named a directory that the audit it runs below reports as a LEAK
    # (measured on wk/run8/glasgow_revC: exit 0 before, exit 4 after).
    #
    # `<workdir>/_truth` would be no better: it is still INSIDE the work dir,
    # so `fence_audit` walks straight into it and reports both files. Every
    # real staged run puts truth in a SIBLING of the work dir, in one of two
    # shapes, and `stage_blind.py` takes the path as an argument rather than
    # deriving it -- so try both known shapes, then fall back to the legacy
    # in-dir location so an old work dir still reports at all (the audit will
    # name it, which is the point).
    #
    #     wk/run13/glasgow  + wk/run13/_truth/            <- sibling
    #     wk/run10/smartknob + wk/run10/_truth/smartknob/ <- sibling, per subject
    _parent = os.path.dirname(os.path.abspath(wd))
    _subject = os.path.basename(os.path.abspath(wd))

    def _truth(name, *alts):
        """First existing candidate: sibling _truth, per-subject, then legacy.

        `alts` are alternative FILE names for the same artifact. stage_blind
        writes `control.kicad_pcb` for its own dose-0 copy and
        `perturbed.control.kicad_pcb` for perturb's, and run 12's truth dir
        carries both -- so a single hardcoded name resolved on some staged runs
        and not others, for a reason that has nothing to do with the layout.
        """
        for nm in (name,) + alts:
            for cand in (os.path.join(_parent, '_truth', nm),
                         os.path.join(_parent, '_truth', _subject, nm),
                         os.path.join(wd, nm)):
                if os.path.isfile(cand):
                    return cand
        return os.path.join(wd, name)          # for the missing-input message

    control = a.control or _truth('perturbed.control.kicad_pcb',
                                  'control.kicad_pcb')
    record_p = a.record or _truth('perturbed.perturb.json',
                                  'board.perturb.json')
    # `control` is checked too. It was the one input that could go missing
    # silently: the fence row degraded to "fence_audit exit 2" in the report,
    # which reads like the audit ran and found nothing.
    for p in (damaged, a.result, record_p, control):
        if not os.path.isfile(p):
            print(f'missing input: {p}', file=sys.stderr)
            return 2

    from list_nets import board_default_netclass_clearance

    def _floor(board):
        """EACH board grades at ITS OWN floor, not the damaged board's.

        The floors genuinely differ along a chain: `route.py`'s `.kicad_pro`
        writeback clamps the Default netclass DOWN to whatever was actually
        routed, so a result board legitimately carries a tighter floor than the
        input it came from. Forcing one number on every row grades the result
        STRICTER than the route used, which manufactures phantom sub-clearance
        grazes -- measured on run 10: the damaged board's floor is 0.2, its
        result board's is 0.127, and grading the result at 0.2 reported 90 DRC
        violations on a board its own run measured at 0. --clearance still
        forces one floor on every row when a comparison needs that.
        """
        if a.clearance is not None:
            return a.clearance
        return board_default_netclass_clearance(board) or 0.25

    clearance = _floor(damaged)

    tmp = os.path.join(wd, '_report')
    os.makedirs(tmp, exist_ok=True)

    # ---- board-only measurements ------------------------------------------
    rows = []
    for label, board in (('damaged input', damaged), ('this run', a.result)):
        _c = _floor(board)
        rows.append({'run': label, 'board': board, 'quality': quality(board),
                     'clearance': _c,
                     'assembly': buildable(board, _c, tmp),
                     'score': blocking(board, _c, tmp)})

    human = None
    if a.human and os.path.isfile(a.human):
        from tests.stress.compare_to_original import (profile,
                                                      original_degeneracy)
        prof = profile(a.human)
        # A "human original" with no copper cannot be a reference, and 7 of 75
        # wave references had none. Say which, rather than printing zeros as
        # though they were a comparison.
        human = {'run': 'human original', 'board': a.human, 'profile': prof,
                 'degeneracy': original_degeneracy(prof),
                 'quality': quality(a.human),
                 'clearance': _floor(a.human),
                 'assembly': buildable(a.human, _floor(a.human), tmp),
                 'score': blocking(a.human, _floor(a.human), tmp)}

    # ---- truth opens HERE, and only here ----------------------------------
    from placement import recovery as R
    from kicad_parser import parse_kicad_pcb
    with open(record_p, encoding='utf-8') as fh:
        record = json.load(fh)
    orig = {r: tuple(v) for r, v in (record.get('original_poses') or {}).items()}
    members = list((record.get('block') or {}).get('members') or [])
    n = len(members)

    def displacement(board):
        pcb = parse_kicad_pcb(board)
        return R.displacement_to_original(R.board_poses(pcb), orig,
                                          pcb.footprints, subset=members)

    for r in rows + ([human] if human else []):
        r['route'] = route(r['board'], record, r['clearance'], tmp)

    d_dmg = displacement(damaged)
    d_res = displacement(a.result)
    d0, d1 = d_dmg['perturbed_pad_rms'], d_res['perturbed_pad_rms']

    def home_count(d):
        frac = d.get('parts_home_frac')
        return None if frac is None else int(round(frac * n))

    rows[0].update(recovery=None, home=home_count(d_dmg), rms=d0,
                   median=d_dmg.get('per_part_median'),
                   collateral=d_dmg.get('collateral_pad_rms'),
                   displacement=d_dmg)
    rows[1].update(recovery=R.recovery_fraction(d0, d1),
                   home=home_count(d_res), rms=d1,
                   median=d_res.get('per_part_median'),
                   collateral=d_res.get('collateral_pad_rms'),
                   displacement=d_res)
    # `collateral_pad_rms` -- how far the run moved parts it was NOT asked to
    # move -- has been computed since this module existed and READ BY NOTHING:
    # it was serialised into REPORT.json and never looked at again. Run 9 took
    # it from 0.000 to 1.171mm, i.e. it displaced undamaged parts, and no gate
    # or report mentioned it. A recovery run that damages what it did not
    # perturb is a specific, detectable failure and it belongs in the table.
    _coll = d_res.get('collateral_pad_rms') or 0.0
    if _coll > 0.5:
        print(f"\n!! COLLATERAL DAMAGE: this run moved parts OUTSIDE the "
              f"perturbed block by {_coll:.3f}mm RMS (the damaged input's "
              f"figure was {d_dmg.get('collateral_pad_rms', 0.0):.3f}). "
              f"Parts nobody asked it to touch have been displaced.\n")
    if human:
        human.update(recovery=None, home=n, rms=0.0)

    code, fence = _run(['tests/stress/fence_audit.py', '--control', control,
                        '--workdir', wd, '--mode', 'audit'])
    fence_verdict = next((ln.strip() for ln in fence.splitlines()
                          if 'VERDICT:' in ln), f'fence_audit exit {code}')

    # ---- the table ----------------------------------------------------------
    #
    # THE OBJECTIVE IS A BOARD THAT ROUTES. This table used to lead with
    # `recovery` and `home /N`, which measure distance to the ORIGINAL POSE, and
    # on run 10 that headline said failure about a board that had gone from NOT
    # BUILDABLE to buildable, copper-free DRC 9 to 0, and pad-pair routability
    # 3.78% to 76.47% -- because its 30-part block had been solved a different
    # way. A placement that is electrically excellent but arranged differently
    # scores ~0 recovery. So: routed outcome first, legality second, effort
    # third, distance-to-truth demoted to the diagnostic block below.
    all_rows = rows + ([human] if human else [])

    def _q(r, key, nd=0):
        """Quality is a TIEBREAK, comparable only once blocking == 0 -- so it
        is parenthesised while anything is still blocking, per board_score's
        own doctrine."""
        v = (r.get('quality') or {}).get(key)
        s = _fmt(v, nd)
        return s if (r.get('score') or {}).get('blocking') == 0 else f'({s})'

    out = [f'# Recovery report — {os.path.basename(wd)}', '',
           'Each board is graded at ITS OWN Default-netclass floor '
           + ', '.join(f'({r["run"]} {r["clearance"]}mm)'
                       for r in rows + ([human] if human else []))
           + ('' if a.clearance is None else
              f' -- OVERRIDDEN to {a.clearance}mm on every row by --clearance')
           + f'. N = {n} (the frozen perturbed member list, '
             f'{d_res.get("members_scored")} scored).', '',
           '**The objective is a board that ROUTES** — zero `unrouted`, zero '
           '`broken`. Rank on `(unrouted + broken + drc, −PRR, vias, '
           'copper_mm)`. `recovery` is a DIAGNOSTIC below, never the score.',
           '',
           '| run | unrouted | broken | PRR | RR | drc | assembly | vias | '
           'copper mm |',
           '|---|---|---|---|---|---|---|---|---|']
    for r in all_rows:
        by = (r.get('score') or {}).get('by') or {}
        rt = r.get('route') or {}
        asm = r.get('assembly') or {}
        out.append('| {} | {} | {} | {} | {} | {} | {} | {} | {} |'.format(
            r['run'], _fmt(by.get('unrouted')), _fmt(by.get('broken')),
            '—' if rt.get('PRR_connected') is None
            else f'{rt["PRR_connected"]:.2f}%',
            '—' if rt.get('RR_connected') is None
            else f'{rt["RR_connected"]:.2f}%',
            _fmt(by.get('drc')),
            'buildable' if asm.get('buildable') else 'NOT BUILDABLE',
            _q(r, 'vias'), _q(r, 'copper_mm', 1)))
    out += ['',
            'Quality columns are parenthesised while `blocking > 0`: they are '
            'a tiebreak and are not comparable until it is zero. The human '
            'original is a **benchmark to approach, not a pose to match** — '
            'what it contributes is the cost floor at blocking 0, and its '
            '`recovery`/`home` cells are `1.0` and `N/N` by definition and '
            'carry no information, which is why they are not printed.',
            '',
            f'- fence: {fence_verdict}']
    for r in all_rows:
        rt = r.get('route') or {}
        if rt.get('PRR') is not None:
            out.append(f'- {r["run"]}: DRC-clean PRR {rt["PRR"]:.2f}% / NRR '
                       f'{rt.get("NRR", 0.0):.2f}% (the `_connected` columns '
                       f'above do not require DRC-cleanliness)')
        elif rt.get('ran') is False:
            out.append(f'- {r["run"]}: routed grade UNMEASURED '
                       f'({rt.get("reason")})')
    for r in all_rows:
        sc = r.get('score') or {}
        pb = (sc.get('unrouted_shape') or {}).get('placement_blocked') or {}
        if pb:
            out.append(
                f'- **{r["run"]}: {len(pb)} of {sc["by"].get("unrouted")} '
                f'unrouted net(s) are PLACEMENT-BLOCKED** — fewer than 2 pads '
                f'ON the board, so no router setting reaches them: '
                f'{" ".join((sc.get("unrouted_shape") or {}).get("placement_blocked_refs") or [])}')
        if sc.get('ungraded'):
            out.append(f'- {r["run"]}: ungraded (NOT passes) — '
                       f'{", ".join(sc["ungraded"])}')
    if human and human['degeneracy']:
        out.append(f'- **the human reference is {human["degeneracy"]}** — its '
                   f'copper cannot serve as a comparison, so read that row as '
                   f'a placement reference only.')
    for r in all_rows:
        asm = r.get('assembly') or {}
        if asm.get('error'):
            out.append(f'- {r["run"]}: assembly UNMEASURED ({asm["error"]})')
        elif not asm.get('buildable'):
            out.append(f'- {r["run"]}: {asm.get("verdict")} — blocking '
                       f'{asm.get("blocking")}, locked contacts '
                       f'{asm.get("locked_contacts")}, '
                       f'{asm.get("oob_pad_count")} part(s) with pad copper '
                       f'off-board')

    # ---- DIAGNOSTIC: did the run move the parts it was asked to move? -------
    #
    # Not the score. These answer one question -- whether the run went where it
    # was pointed -- and the answer is useful even when it is bad news about a
    # board that got better.
    out += ['', '## Diagnostic — distance to the original poses', '',
            'These are NOT the score. A run that solves the board a different '
            'way scores ~0 here and may still be the better board; what they '
            'catch is a run that WANDERED.', '',
            f'- recovery: '
            + ('—' if rows[1].get('recovery') is None
               else f'{rows[1]["recovery"]:+.4f}')
            + f'  |  home {_fmt(rows[1].get("home"))} / {n}',
            f'- distance to truth (pad RMS): damaged {d0:.4f} → '
            f'result {d1:.4f}',
            '- `home` is pad-space, at '
            f'`recovery.HOME_TOLERANCE_MM` = {R.HOME_TOLERANCE_MM} mm — a part '
            'rotated in place moves its origin 0.00 mm and its pads much '
            'further, so an origin-distance count would call it home.']
    if d_res.get('rot_mismatch_total'):
        out.append(f'- rotation mismatches: {d_res["rot_mismatch_total"]} '
                   f'({", ".join(d_res.get("rot_mismatch") or [])})')
    # collateral, WITH NAMES. It was the only instrument that saw run 10 tear
    # two members out of an intact 8-fold ring, and it reported that as the
    # bare scalar 3.6697 with nothing attached.
    _cnames = _collateral_refs(damaged, a.result, members)
    out.append(
        f'- collateral (parts OUTSIDE the perturbed block, pad RMS): '
        f'{d_dmg.get("collateral_pad_rms", 0.0):.4f} → {_coll:.4f}'
        + ('' if not _cnames else
           '\n  - moved: ' + ', '.join(f'**{r}** {d:.3f} mm'
                                       + (f' (rot {ro:+.0f}°)' if ro else '')
                                       for r, d, ro in _cnames)))
    if _coll > 0.5:
        out.append(
            f'- **COLLATERAL DAMAGE**: this run displaced parts nobody asked '
            f'it to touch. That is a real defect whatever the headline says, '
            f'and it is how an intact structure gets dismantled to buy a '
            f'legality number.')
    _usr = None
    try:
        _usr = R.unit_spread_ratio(R.board_poses(parse_kicad_pcb(a.result)),
                                   orig, members)
    except Exception:                                          # noqa: BLE001
        pass
    if _usr is not None:
        _g = R._gyration(orig, members)
        out.append(f'- unit spread ratio: {_usr:.4f} against an original '
                   f'gyration of '
                   + ('—' if _g is None else f'{_g:.3f} mm')
                   + ' (the ratio is unreadable without it: 1.03 on a 50 mm '
                     'unit means "cannot tell", not "not torn")')

    text = '\n'.join(out) + '\n'
    dest = a.out or os.path.join(wd, 'REPORT.md')
    with open(dest, 'w', encoding='utf-8') as fh:
        fh.write(text)
    print(text)
    print(f'REPORT -> {dest}')
    jp = os.path.splitext(dest)[0] + '.json'
    with open(jp, 'w', encoding='utf-8') as fh:
        json.dump({'workdir': wd, 'clearance': clearance, 'N': n,
                   'fence': fence_verdict, 'rows': all_rows}, fh, indent=1,
                  sort_keys=True, default=str)
    print(f'JSON   -> {jp}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
