#!/usr/bin/env python3
"""Audit a perturbed-corpus work dir for GROUND-TRUTH LEAKS, by content.

A recovery experiment measures how close a repaired placement lands to the
original. That number is only meaningful if the tools never saw the original.
The protocol fences the obvious carriers by NAME -- the control board, the
perturbation record, the human reference -- and a run-7 watcher found the hole
that naming leaves open:

    the STRIPPED board (the human placement with its copper removed) sat in
    every work dir. It is the perturbation's INPUT, so nobody thought of it as
    truth. Its footprint poses ARE the human placement, exactly.

Naming cannot close that hole, because the next carrier will have a different
name (a `.bak`, a `board_orig.kicad_pcb`, a candidate copied out of an archive).
So this audit ignores names and asks the only question that matters:

    does any FILE in this work dir carry the control's poses?

Boards, and pose RECORDS: a perturbation record embeds `original_poses`
verbatim, so it is ground truth in JSON clothing and gets no name exemption
either. This matters on the default path, not a contrived one --
`perturb(..., control_out=None)` writes that record BESIDE the output board,
i.e. inside the fence. Stage with `control_out=<a dir outside the work dir>`
and the record never enters.

Two modes, because the same content means opposite things before and after a run:

  --mode create   Run at work-dir creation, BEFORE any tool touches the board.
                  Every board here is an INPUT. Any board whose poses match the
                  control (other than the explicitly allowed carriers) is a
                  leak -> exit 4. Writes `.fence-manifest.json` recording the
                  files present and their hashes.

  --mode audit    Run at watcher time, AFTER the run. A board matching the
                  control now has two possible causes, and the manifest tells
                  them apart:
                    * present at creation  -> LEAK (it was an input)
                    * created by the run   -> RECOVERED, which is the experiment
                      succeeding, not a breach (a perfect reconstruction lands
                      exactly on truth -- one run-7 board did, at d1 = 0.000000)
                  Exit 4 only for the first kind.

Match is on the pose set, not on file bytes: copper, UUIDs, netclasses and
zone fills all differ between a stripped board and its control while the
placement is identical.

Usage:
    python3 -X utf8 tests/stress/fence_audit.py --control C.kicad_pcb \\
        --workdir wk/run8/<board>/ --mode create
    python3 -X utf8 tests/stress/fence_audit.py --control C.kicad_pcb \\
        --workdir wk/run8/<board>/ --mode audit [--json]

Exit codes: 0 clean, 2 usage/IO problem, 4 leak found.
"""
import argparse
import fnmatch
import hashlib
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import recovery  # noqa: E402

MANIFEST_NAME = '.fence-manifest.json'

# Carriers the protocol declares as truth ON PURPOSE, excluded from the leak
# report by name.
#
# `*.control.kicad_pcb` USED TO BE HERE AND IS GONE (run-12 Tier 0). It made
# the same bytes a LEAK or CLEAN depending only on the filename: a copy of the
# control under any other name was reported, the control itself was not --
# while `perturb()` wrote it into the output directory by default, so the most
# likely carrier of all was the one carrier this audit ignored. That
# contradicts the docstring above ("Naming cannot close that hole ... So this
# audit ignores names"), and a name-based exemption for the single most likely
# carrier is exactly the hole the module exists to close. A control INSIDE the
# work dir is now a LEAK, which is what this module says it believes.
#
# The legitimate cases are still covered: `control_real` (below) never reports
# the audit's OWN reference, and `--allow` remains for a caller that genuinely
# means it.
#
# DEFAULT_ALLOW IS EMPTY, AND THAT IS THE POINT.
#
# It used to hold `*.perturb.json`. That was harmless only while it was a lie:
# `scan()` walked `.kicad_pcb` alone, so the entry exempted a file the module
# could not open anyway. Teaching the scan to read `.json` turned an inert
# declaration into a working blind spot -- and not a hypothetical one.
# `placement.perturb.perturb(...)` with `control_out` unset writes the record
# BESIDE THE OUTPUT BOARD (perturb.py:596-602), i.e. inside the work dir, and
# that record embeds `original_poses` verbatim. So the default perturbation
# path drops complete ground truth into the fence, and this tuple told the
# audit to walk past it.
#
# The module's thesis is that a leak is CONTENT, not a filename; it already
# applies that to the control ("A control INSIDE the work dir is now a LEAK",
# above). A record is the same truth in JSON clothing, so it gets the same
# answer. The declared record lives in `_truth/`, OUTSIDE the work dir, which
# is what `control_out` is for and what the staging recipe does -- a record
# inside the fence is a staging mistake, and this is the tool that says so.
#
# `--allow` remains for a caller who genuinely means it.
DEFAULT_ALLOW = ()

# A board counts as truth-carrying when this fraction of its shared refs sit
# within POSE_TOL of the control. Not 1.0: a leak is still a leak when one
# footprint was nudged, and a repaired board that genuinely reaches truth is
# separated by the manifest (see --mode audit), not by this threshold.
MATCH_FRAC = 0.98
POSE_TOL_MM = 1e-6
POSE_TOL_DEG = 1e-3


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def pose_match(poses_a, poses_b):
    """(match_frac, shared_ref_count, worst_mm) between two pose maps."""
    shared = sorted(set(poses_a) & set(poses_b))
    if not shared:
        return 0.0, 0, None
    hits = 0
    worst = 0.0
    for ref in shared:
        ax, ay, arot = poses_a[ref][0], poses_a[ref][1], poses_a[ref][2]
        bx, by, brot = poses_b[ref][0], poses_b[ref][1], poses_b[ref][2]
        d = math.hypot(ax - bx, ay - by)
        worst = max(worst, d)
        drot = abs((arot - brot) % 360.0)
        drot = min(drot, 360.0 - drot)
        if d <= POSE_TOL_MM and drot <= POSE_TOL_DEG:
            hits += 1
    return hits / len(shared), len(shared), worst


def _json_poses(path):
    """{ref: (x, y, rot)} out of a perturbation RECORD, or None.

    The record embeds `original_poses` verbatim -- it is ground truth in JSON
    clothing, and the scan filtered on the extension before opening anything,
    so this module named a carrier it structurally could not see.

    Read BYTES and let `json.loads` sniff the encoding. Opening as utf-8 text
    made the audit defeatable by re-saving the record as UTF-16 -- same keys,
    same truth, `UnicodeDecodeError` into the bare `except`, waved through as
    "ordinary JSON". `json.loads` applies RFC-4627 BOM detection and accepts
    utf-8, utf-8-sig, utf-16 and utf-32 alike (verified), so the content test
    stops depending on how the file was saved.
    """
    try:
        with open(path, 'rb') as fh:
            doc = json.loads(fh.read())
    except Exception:                                   # noqa: BLE001
        return None
    if not isinstance(doc, dict):
        return None
    # `original_poses` ONLY. A `poses` alias was tried and dropped: across 561
    # real driver-output JSONs in this repo it matched nothing (no writer emits
    # a top-level ref->[x,y,rot] dict under that name), so it bought no
    # coverage -- while the `kind == 'record'` branch in main() is
    # unconditional, meaning anything this function claims can NEVER be classed
    # as recovery. A future tool writing its RESULT as {"poses": {...}} would
    # therefore have had a perfect reconstruction -- the case the module
    # docstring insists must not be a breach ("one run-7 board did, at
    # d1 = 0.000000") -- reported as a LEAK. Speculative coverage is not worth
    # a mechanism that misclassifies the experiment succeeding.
    for key in ('original_poses',):
        raw = doc.get(key)
        if isinstance(raw, dict) and raw:
            out = {}
            for ref, v in raw.items():
                if isinstance(v, (list, tuple)) and len(v) >= 3:
                    try:
                        out[ref] = (float(v[0]), float(v[1]),
                                    float(v[2]) % 360.0)
                    except (TypeError, ValueError):
                        continue
            if out:
                return out
    return None


#: Extensions the scan opens. This module's thesis -- "the next carrier will
#: have a different name" -- applies to EXTENSIONS exactly as it does to stems,
#: and the scan used to stop at `.kicad_pcb`.
#:
#: `.kicad_pro` earns its place the same way. The project MUST travel with a
#: staged board or it grades at the stock netclass (#441), but KiCad stores
#: `meta.filename` and `pcbnew.last_paths` inside it -- so a verbatim copy
#: put the SOURCE'S NAME and the author's directories in the work dir, and
#: this scan opened one file and returned CLEAN. A `.kicad_pro` carries no
#: poses, so it is checked for IDENTITY rather than placement: see
#: `_names_control`.
SCANNED_EXT = ('.kicad_pcb', '.json', '.kicad_pro')


#: A pad centre and a track end are "the same point" within this. Run 14
#: measured 301 of 569 netted pads sitting within 5 um of their own track
#: endpoint, so the coincidence is exact, not approximate.
COPPER_TOL_MM = 0.01
#: Below this many displaced pads the fraction is noise, not a signal.
COPPER_MIN_PADS = 5
#: Fraction of DISPLACED control pads that must still have copper at the
#: control's position. Deliberately low: on an honestly routed board the
#: expected value is ZERO, because copper laid down for a pad that has since
#: moved does not stay behind at the old coordinate. Anything materially above
#: zero is the signal, and the baseline test below is what keeps a densely
#: routed board from tripping it.
COPPER_FRAC = 0.15


def _netted_pad_xy(pcb):
    """Rounded (x, y) of every pad that carries a net."""
    out = set()
    for fp in (pcb.footprints or {}).values():
        for pad in fp.pads:
            if pad.net_id:
                out.add((round(pad.global_x, 3), round(pad.global_y, 3)))
    return out


def _copper_points(pcb):
    """Every point copper touches: segment ends and via centres."""
    pts = set()
    for s in (pcb.segments or []):
        pts.add((round(s.start_x, 3), round(s.start_y, 3)))
        pts.add((round(s.end_x, 3), round(s.end_y, 3)))
    for v in (pcb.vias or []):
        pts.add((round(v.x, 3), round(v.y, 3)))
    return pts


def copper_carries_poses(pcb, control_pads):
    """(displaced, hits, frac) -- does this board's COPPER mark the control's pads?

    `scan()` compares POSES and never opens copper, and that is a hole with a
    measured size. A subject staged from a routed corpus board keeps the human
    routing while the parts move out from under it, so the track ends go on
    marking where each part BELONGS. Run 14 shipped exactly that: fence_audit
    said CLEAN on a board whose copper located 301 of 569 pads to within 5 um,
    and the leak was only closed later, by a copper strip done for an unrelated
    reason.

    THE TRAP THIS AVOIDS: a board whose placement was successfully RECOVERED
    has its pads back at the control's coordinates, so "copper sits on control
    pads" is true of the best possible outcome as well as the worst. The
    discriminator is displacement. Score only the control pads this board has
    moved AWAY from -- if copper still sits where those pads used to be, the
    copper is telling you where they belong, and that is the leak. A recovered
    board has almost no displaced pads to score, and a board routed honestly at
    NEW positions has no copper at the old ones.
    """
    own = _netted_pad_xy(pcb)
    displaced = control_pads - own
    stayed = control_pads & own
    if len(displaced) < COPPER_MIN_PADS:
        return len(displaced), 0, 0.0, 0.0
    pts = _copper_points(pcb)
    if not pts:
        return len(displaced), 0, 0.0, 0.0

    step = int(round(COPPER_TOL_MM * 1000))

    def _covered(pads):
        """How many of these coordinates have copper sitting on them."""
        n = 0
        for (px, py) in pads:
            bx, by = int(round(px * 1000)), int(round(py * 1000))
            for dx in range(-step, step + 1):
                for dy in range(-step, step + 1):
                    if ((bx + dx) / 1000.0, (by + dy) / 1000.0) in pts:
                        n += 1
                        break
                else:
                    continue
                break
        return n

    hits = _covered(displaced)
    rate = hits / float(len(displaced))
    # THE BASELINE. Boards differ enormously in how often a track actually
    # terminates on a pad CENTRE -- one fixture here does it for 18% of pads,
    # run 14's board for 53% -- so a bare fraction is not comparable across
    # boards and any fixed threshold is arbitrary. The pads that did NOT move
    # give the board's own rate for free, and the comparison is scale-free:
    # copper laid out on the CONTROL's placement covers displaced pads at least
    # as often as it covers the ones that stayed put, while copper laid out
    # honestly on THIS placement covers the displaced positions essentially
    # never.
    base = (_covered(stayed) / float(len(stayed))) if stayed else 0.0
    return len(displaced), hits, rate, base


def _names_control(path, control_stem):
    """Strings in this project that name the control's board.

    A `.kicad_pro` carries no poses, so the leak it can commit is IDENTITY:
    KiCad writes `meta.filename`, and `pcbnew.last_paths` holds the author's
    own directories. Either one is a path back to the original placement,
    which is exactly what "the source is recorded by HASH, not by path"
    exists to prevent. Substring, not equality: `last_paths` embeds the stem
    inside a longer path.
    """
    if not control_stem:
        return []
    try:
        with open(path, encoding='utf-8') as f:
            raw = f.read()
    except Exception:                                # noqa: BLE001
        return []
    hits = []
    for line in raw.splitlines():
        if control_stem in line:
            hits.append(line.strip()[:120])
    return hits[:6]


def scan(workdir, control_poses, allow, control_pads=None,
         control_stem=None):
    """Every scannable file in workdir, scored against the control's poses."""
    rows = []
    for root, _dirs, files in os.walk(workdir):
        for name in sorted(files):
            if not name.endswith(SCANNED_EXT):
                continue
            path = os.path.join(root, name)
            rel = os.path.relpath(path, workdir).replace('\\', '/')
            if any(fnmatch.fnmatch(rel, pat) or fnmatch.fnmatch(name, pat)
                   for pat in allow):
                continue
            _copper = None
            try:
                if name.endswith('.kicad_pro'):
                    # Identity, not poses: does this project name the board
                    # the control came from? The stem is what a Read would
                    # follow back to the original placement.
                    hit = _names_control(path, control_stem)
                    if hit:
                        rows.append({
                            'file': rel, 'kind': 'project',
                            'match_frac': 1.0, 'refs_shared': 0,
                            'worst_mm': None,
                            'carries_truth': True,
                            'truth_channel': 'identity',
                            'names_source': hit,
                            'sha256': _sha256(path),
                            'mtime': os.path.getmtime(path)})
                    continue
                if name.endswith('.json'):
                    poses = _json_poses(path)
                    if poses is None:
                        continue        # ordinary JSON, not a pose carrier
                else:
                    _pcb = parse_kicad_pcb(path)
                    poses = recovery.board_poses(_pcb)
                    if control_pads:
                        _copper = copper_carries_poses(_pcb, control_pads)
            except Exception as exc:                    # unparseable is not a leak
                rows.append({'file': rel, 'error': str(exc)[:120]})
                continue
            frac, shared, worst = pose_match(poses, control_poses)
            _kind = 'record' if name.endswith('.json') else 'board'
            _by_pose = frac >= MATCH_FRAC and shared > 0
            # Materially above zero AND no rarer than on the pads that stayed
            # put. The second half is what stops a densely routed board that
            # happens to terminate on many pad centres from tripping it.
            _by_copper = bool(_copper and _copper[2] >= COPPER_FRAC
                              and _copper[2] >= _copper[3])
            row = {
                'file': rel,
                'kind': _kind,
                'match_frac': round(frac, 6),
                'refs_shared': shared,
                'worst_mm': None if worst is None else round(worst, 6),
                'carries_truth': _by_pose or _by_copper,
                # WHICH channel carries it, because the two have different
                # fixes: poses mean a control board got copied in, copper means
                # the subject was staged from a routed board and never stripped.
                'truth_channel': ('poses' if _by_pose else
                                  'copper' if _by_copper else None),
                'sha256': _sha256(path),
                'mtime': os.path.getmtime(path),
            }
            if _copper is not None:
                row['copper_displaced_pads'] = _copper[0]
                row['copper_hits_on_control'] = _copper[1]
                row['copper_frac'] = round(_copper[2], 6)
                row['copper_baseline_frac'] = round(_copper[3], 6)
            rows.append(row)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--control', required=True,
                    help='the ground-truth board (the perturbation control)')
    ap.add_argument('--workdir', required=True, help='work dir to audit')
    ap.add_argument('--mode', choices=('create', 'audit'), default='audit')
    ap.add_argument('--allow', action='append', default=[],
                    help='extra glob of a DECLARED truth carrier (repeatable)')
    ap.add_argument('--json', action='store_true', help='machine-readable report')
    args = ap.parse_args(argv)

    if not os.path.isfile(args.control):
        print(f'fence_audit: control not found: {args.control}', file=sys.stderr)
        return 2
    if not os.path.isdir(args.workdir):
        print(f'fence_audit: workdir not found: {args.workdir}', file=sys.stderr)
        return 2

    allow = list(DEFAULT_ALLOW) + list(args.allow)
    _control_pcb = parse_kicad_pcb(args.control)
    control_poses = recovery.board_poses(_control_pcb)
    control_pads = _netted_pad_xy(_control_pcb)
    # The control may itself live inside the work dir under any name; never
    # report the audit's own reference as a leak.
    control_real = os.path.realpath(args.control)

    # WHICH name is the leak: the SOURCE's, not the control's. Both stagers
    # write the control as `control.kicad_pcb`, so its own stem names
    # nothing; the source basename is recorded in `stage.json` beside it,
    # which is the whole point of recording the source by hash and name
    # THERE (outside the fence) rather than in the work dir.
    # BOTH stagers, because they record it under different names and a check
    # that knows only one is inert for the other: `stage_unaided` writes
    # stage.json/source_basename, `stage_blind` writes draw.json/source. The
    # first version read stage.json alone and returned CLEAN on a blind work
    # dir whose project said "watchy.kicad_pro".
    _stem = None
    _truth = os.path.dirname(os.path.abspath(args.control))
    for _name, _key in (('stage.json', 'source_basename'),
                        ('draw.json', 'source')):
        _p = os.path.join(_truth, _name)
        if _stem or not os.path.isfile(_p):
            continue
        try:
            with open(_p, encoding='utf-8') as _f:
                _raw = json.load(_f).get(_key) or ''
            _stem = os.path.splitext(os.path.basename(_raw))[0] or None
        except Exception:                            # noqa: BLE001
            _stem = None
    if not _stem:
        _stem = os.path.splitext(os.path.basename(args.control))[0]
    rows = [r for r in scan(args.workdir, control_poses, allow, control_pads,
                            control_stem=_stem)
            if os.path.realpath(os.path.join(args.workdir, r['file'])) != control_real]

    manifest_path = os.path.join(args.workdir, MANIFEST_NAME)
    manifest = None
    if args.mode == 'audit' and os.path.isfile(manifest_path):
        try:
            manifest = json.load(open(manifest_path, encoding='utf-8'))
        except Exception as exc:
            print(f'fence_audit: manifest unreadable ({exc}); every '
                  f'truth-carrying board is reported as a LEAK', file=sys.stderr)

    if args.mode == 'audit' and manifest is None:
        # SAY it. The unreadable-manifest path above warns; the ABSENT one said
        # nothing, so audit silently became strict where create had been silent
        # -- and a reader could not tell "no board carried truth" from "there
        # was no record to check against".
        print('fence_audit: no creation manifest in this work dir, so no board '
              'has provenance. Every truth-carrying board will be reported as '
              'a LEAK. Run --mode create BEFORE the run to record one.',
              file=sys.stderr)

    at_creation = set((manifest or {}).get('files', []))
    at_creation_shas = (manifest or {}).get('shas') or {}
    control_sha = (manifest or {}).get('control_sha256')
    leaks, recovered, errors = [], [], []
    for row in rows:
        if 'error' in row:
            errors.append(row)
        elif not row['carries_truth']:
            continue
        elif args.mode == 'create':
            leaks.append(row)
        elif row.get('kind') == 'record':
            # FIRST among the audit branches, because it is the one reason that
            # holds no matter what the manifest says. Ordered after them it was
            # unreachable in the commonest case -- no manifest -- and the row
            # came back "provenance unknown", which invites someone to fix it
            # by running --mode create. That would record the record and make
            # it permanently exempt. A pose RECORD carries `original_poses`
            # verbatim and no placement run reconstructs one, so provenance is
            # not the question.
            row['reason'] = ('a pose RECORD inside the fence -- records are '
                             'never reconstructed, so this is truth arriving, '
                             'not recovery')
            leaks.append(row)
        elif manifest is None:
            row['reason'] = 'no creation manifest -- provenance unknown'
            leaks.append(row)
        elif row['file'] in at_creation \
                and at_creation_shas.get(row['file']) == row.get('sha256'):
            # UNCHANGED since creation, so it is an input that already carried
            # truth. The sha is what makes that testable: this branch used to
            # key on the PATH alone, which meant a board the run REWROTE in
            # place -- a recovery landing on the control's poses -- was
            # reported as "present at work-dir creation". The better the run
            # did, the more certainly it fired. `shas` was recorded here for
            # exactly this and was never read.
            row['reason'] = 'present at work-dir creation, unchanged since'
            leaks.append(row)
        elif control_sha and row.get('sha256') == control_sha:
            # A BYTE COPY OF THE CONTROL is not a reconstruction. A real
            # recovery differs in copper, UUIDs and netclasses even when every
            # pose matches; identical bytes mean the file was copied, not
            # rebuilt. Without this, `cp control.kicad_pcb r4_final.kicad_pcb`
            # was classed "produced by the run" and exited 0.
            row['reason'] = ('byte-identical to the control -- copied, not '
                             'reconstructed')
            leaks.append(row)
        else:
            row['reason'] = 'produced by the run (recovery reaching truth)'
            recovered.append(row)

    if args.mode == 'create':
        with open(manifest_path, 'w', encoding='utf-8') as fh:
            json.dump({
                'control': os.path.relpath(args.control, args.workdir
                                           ).replace('\\', '/'),
                'control_sha256': _sha256(args.control),
                'allow': allow,
                'files': [r['file'] for r in rows],
                'shas': {r['file']: r.get('sha256') for r in rows},
            }, fh, indent=1, sort_keys=True)

    report = {
        'mode': args.mode,
        'workdir': args.workdir.replace('\\', '/'),
        'control': args.control.replace('\\', '/'),
        # `files_scanned`, not `boards_scanned`: the scan reads pose RECORDS as
        # well as boards, and a count that says "boards" while including
        # records is the same category error the name exemption was.
        'files_scanned': len(rows),
        'leaks': leaks,
        'recovered_to_truth': recovered,
        'unparseable': errors,
        'verdict': 'LEAK' if leaks else 'CLEAN',
    }
    if args.json:
        if args.mode == 'create':
            # CREATE runs with the fence UP. `worst_mm` IS the applied dose and
            # `match_frac` is the block size -- the two things the staging
            # script captures stdout to conceal. A leak verdict needs neither.
            #
            # `copper_displaced_pads` is the block size too, stated even more
            # directly: it counts the pads the perturbation moved. It was added
            # to this report and had to be added here in the same breath.
            _strip = ('match_frac', 'worst_mm', 'refs_shared', 'mtime',
                      'copper_displaced_pads', 'copper_hits_on_control',
                      'copper_frac', 'copper_baseline_frac')
            for _b in ('leaks', 'recovered_to_truth'):
                report[_b] = [{k: v for k, v in r.items() if k not in _strip}
                              for r in report[_b]]
            report['note'] = ('create mode: per-board metrics withheld -- they '
                              'disclose the perturbation dose and block size '
                              'while the fence is still up')
        print(json.dumps(report, indent=1, sort_keys=True))
    else:
        print(f'fence_audit [{args.mode}] {args.workdir}: '
              f'{len(rows)} file(s) scanned against {args.control}')
        for row in errors:
            print(f'  UNPARSEABLE {row["file"]}: {row["error"]}')
        for row in recovered:
            print(f'  ok  {row["file"]}: matches the control '
                  f'({row["match_frac"]:.3f}) -- {row["reason"]}')
        for row in leaks:
            # NAME THE CHANNEL. The two leaks have different causes and
            # different fixes, and printing the pose fraction for a copper leak
            # describes a number that did not fire: run 14's own board matched
            # only 0.874 of refs, well under the pose threshold, and was caught
            # entirely by its copper.
            if row.get('truth_channel') == 'copper':
                # The COUNTS are the block size, so they are withheld while the
                # fence is still up -- exactly like match_frac in the JSON. The
                # channel is what the operator needs; the number is what the
                # run must not see.
                _cnt = ('' if args.mode == 'create' else
                        f' ({row.get("copper_hits_on_control")} of '
                        f'{row.get("copper_displaced_pads")} displaced pads '
                        f'still have copper at the control\'s position)')
                _how = (f'its COPPER marks the control\'s placement{_cnt}. '
                        f'This board was staged from a ROUTED board and never '
                        f'stripped -- the tracks point at where each part '
                        f'belongs. Strip it with '
                        f'tests/stress/strip_copper_only.py')
            elif row.get('truth_channel') == 'identity':
                # A project carries no poses; what it can leak is the SOURCE'S
                # NAME, which is one Read away from the original placement.
                _how = ('NAMES THE SOURCE: '
                        + '; '.join(row.get('names_source') or [])
                        + '. A project must travel (#441) but KiCad stores '
                          'meta.filename and pcbnew.last_paths inside it -- '
                          'sanitise those before it enters the work dir')
            else:
                _how = (f'carries the control\'s placement '
                        f'({row["match_frac"]:.3f} of {row["refs_shared"]} '
                        f'refs within {POSE_TOL_MM}mm)')
            print(f'  LEAK {row["file"]}: {_how}'
                  + (f' -- {row["reason"]}' if row.get('reason') else ''))
        if leaks:
            _nb = sum(1 for r in leaks if r.get('kind') != 'record')
            _nr = len(leaks) - _nb
            _what = ' + '.join(([f'{_nb} board(s)'] if _nb else [])
                               + ([f'{_nr} pose record(s)'] if _nr else []))
            print(f'  VERDICT: LEAK -- {_what} carry ground truth inside the '
                  f'work dir. Move them outside the fence before the run -- '
                  f'that is what perturb(control_out=...) is for -- or declare '
                  f'them with --allow.')
        else:
            print('  VERDICT: CLEAN (no undeclared board OR pose record '
                  'carries the control\'s placement)')
    return 4 if leaks else 0


if __name__ == '__main__':
    sys.exit(main())
