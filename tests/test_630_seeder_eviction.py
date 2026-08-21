#!/usr/bin/env python3
"""Issue #630's own pin test, against the tool the issue names.

    "`place_seed` on a pile fixture must seat a part whose only pocket is
     blocked by one movable incumbent, without hand intervention."

Not `place_plan`, not a library call -- `place_seed`, because that is the
tool run 19 was using when it returned "no legal pose anywhere on the board"
three times and a teammate went off and wrote 221 lines of arithmetic.

The block is a THEOREM, not an observation. On a 16 x 14 board at 0.5mm edge
clearance, BIG's 10x10 courtyard confines its centre to x in [5.5, 10.5] and
y in [5.5, 8.5]. Clearing SMALL's 1x1 courtyard at the zone centre by 0.2mm
needs |cx - 8| >= 5.7 or |cy - 7| >= 5.7, and neither interval intersects
BIG's legal range. So while SMALL sits there BIG has exactly zero legal
poses, and a full rim of them once it moves.

Both parts are declared into the SAME zone, so the seeder packs them by
descending pin count -- SMALL first, because it has more pads. That is not a
contrivance: it is the ordering that produced run 19's failure (34 six-pad
diodes seated before 34 fifteen-millimetre switches, and the smalls took the
centre).
"""
import json
import os
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

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


def _part(ref, x, y, cy, npads):
    pads = ''.join(
        f'\t\t(pad "{i + 1}" smd rect\n\t\t\t(at {i * 0.2 - 0.2} 0)\n'
        f'\t\t\t(size 0.3 0.3)\n\t\t\t(layers "F.Cu")\n'
        f'\t\t\t(net {1 if i == 0 else 2} "N{1 if i == 0 else 2}")\n'
        f'\t\t\t(uuid "p{i}-{ref}")\n\t\t)\n' for i in range(npads))
    return f'''\t(footprint "test:P{ref}"
\t\t(layer "F.Cu")
\t\t(uuid "fp-{ref}")
\t\t(at {x} {y})
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 0)
\t\t)
\t\t(fp_rect
\t\t\t(start {-cy} {-cy})
\t\t\t(end {cy} {cy})
\t\t\t(layer "F.CrtYd")
\t\t\t(uuid "cy-{ref}")
\t\t)
{pads}\t)
'''


def pile_board(path):
    """Every part stacked at one coordinate: the place-from-scratch task."""
    # SMALL has MORE pads than BIG, so the zone packer seats it first.
    fps = (_part('BIG', 8, 7, 5.0, 2) + _part('SMALL', 8, 7, 0.5, 4))
    body = ('(kicad_pcb\n\t(version 20241229)\n'
            '\t(net 0 "")\n\t(net 1 "N1")\n\t(net 2 "N2")\n'
            '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 16 14)\n'
            '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n' + fps + ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)


INTENT = {
    "schema": 1, "kind": "floorplan-intent", "units": "mm",
    "envelope": {"rect": [0.0, 0.0, 16.0, 14.0], "tolerance_mm": 0.5},
    "blocks": [{"name": "everything", "refs": ["BIG", "SMALL"],
                "zone": [0.5, 0.5, 15.5, 13.5], "tolerance_mm": 0.5}],
}


def run(workdir, *extra):
    board = os.path.join(workdir, 'pile.kicad_pcb')
    intent = os.path.join(workdir, 'fp.json')
    out = os.path.join(workdir, 'seed.kicad_pcb')
    pile_board(board)
    with open(intent, 'w', encoding='utf-8') as f:
        json.dump(INTENT, f)
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join('py_placer', 'place_seed.py'), board, out,
         '--intent', intent, '--clearance', '0.2',
         '--board-edge-clearance', '0.5', '--no-polish', *extra],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=REPO, timeout=900,
        env=dict(os.environ, PYTHONHASHSEED='0', PYTHONIOENCODING='utf-8'))
    summary = None
    for line in r.stdout.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            summary = json.loads(line.split(':', 1)[1])
    return r, summary, out


# --------------------------------------------------------------------------
# THE PIN TEST
# --------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as wd:
    proc, s, out = run(wd)
    check("place_seed runs", s is not None,
          (proc.stdout[-600:] + proc.stderr[-600:]))
    if s:
        check("#630: it seats BOTH parts, with no hand intervention",
              s['unseated'] == 0 and s['placed'] == 2,
              f"placed {s['placed']}, unseated {s['unseated']} "
              f"{s.get('unseated_refs')}")
        check("and exits 0 rather than 4", proc.returncode == 0,
              f"rc={proc.returncode}")
    check("the run says which part it evicted, and what that freed",
          'evicting' in proc.stdout and 'poses at its target' in proc.stdout,
          str([l for l in proc.stdout.splitlines() if "evict" in l][:2]))

# --------------------------------------------------------------------------
# #629: the verdict NAMES its blockers, in the JSON, with the counts
# --------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as wd:
    proc, s, out = run(wd, '--evict-depth', '0')
    check("with the rung off, the seed fails the way it always did",
          s and s['unseated'] == 1 and proc.returncode == 4,
          f"unseated {s and s['unseated']}, rc={proc.returncode}")
    check("#629: the JSON_SUMMARY names the unseated ref, not just a count",
          s and s.get('unseated_refs') == ['BIG'],
          str(s and s.get('unseated_refs')))
    check("#629: `no_pose_blockers` is in the JSON_SUMMARY",
          s and 'no_pose_blockers' in s, str(sorted(s or {})))
    check("#629: it names the blocker and the poses that blocker frees",
          s and s.get('no_pose_blockers', {}).get('BIG', {}).get('SMALL', 0)
          > 0,
          str(s and s.get('no_pose_blockers')))
    check("the bare verdict is still printed, now with the census beside it",
          'no legal pose' in proc.stdout, proc.stdout[-300:])

# --------------------------------------------------------------------------
# The gate: a trade that does not improve the board must not ship
# --------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as wd:
    proc, s, out = run(wd)
    # Every eviction is recorded with the gate tuple either side, so a reader
    # can check the trade rather than trust it.
    ev = [l for l in proc.stdout.splitlines() if 'gate' in l and 'evict' in l]
    check("the eviction is RECORDED with the gate either side of it",
          bool(ev), str([l for l in proc.stdout.splitlines()
                         if 'evict' in l][:3]))

# --------------------------------------------------------------------------
# A locked incumbent is never evicted -- it is not this tool's to move
# --------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as wd:
    board = os.path.join(wd, 'pile.kicad_pcb')
    pile_board(board)
    src = open(board, encoding='utf-8').read().replace(
        '(footprint "test:PSMALL"\n\t\t(layer "F.Cu")',
        '(footprint "test:PSMALL"\n\t\t(layer "F.Cu")\n\t\t(locked yes)')
    open(board, 'w', encoding='utf-8').write(src)
    intent = os.path.join(wd, 'fp.json')
    with open(intent, 'w', encoding='utf-8') as f:
        json.dump(INTENT, f)
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join('py_placer', 'place_seed.py'), board,
         os.path.join(wd, 'o.kicad_pcb'), '--intent', intent,
         '--clearance', '0.2', '--board-edge-clearance', '0.5', '--no-polish'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=REPO, timeout=900,
        env=dict(os.environ, PYTHONIOENCODING='utf-8'))
    moved = 'SMALL' in r.stdout and 'evicting SMALL' in r.stdout
    check("a file-locked incumbent is never evicted",
          not moved, "the rung moved a part the file locked")

print(f"\n{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
