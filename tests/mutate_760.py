#!/usr/bin/env python3
"""#760 mutation battery: each row breaks ONE part of the fix and must turn
tests/test_760_hole_local_clearance.py red. A row that SURVIVES means that file
does not actually gate that part -- which is not hypothetical here:

  * the first draft used an NPTH written `('*.Cu', '*.Mask')`, which
    `_foreign_pad_arrays` admits into the COPPER channel along with its
    `local_clearance`. Every arm passed through the wrong channel and the
    battery killed 1 row of 6.
  * with that fixed, `microshift-acceptance-reverted` still SURVIVED: in the
    trade fixture the hole is the worst offender, so the RANKING already points
    the shift away from it and `clears()` never decides. That is what
    `_acceptance_gate` was added for.
  * `override-leaks-into-the-flat-floor` survived a per-hole control written
    the obvious way round (override near, plain hole far) -- the far hole is
    never the closest approach, so a board-wide max() reads identically.

Run it after touching either site:

    python3 tests/mutate_760.py
"""
import os
import subprocess
import sys

# This battery's runner is at MODULE SCOPE, so `import mutate_760` REWRITES
# ENGINE FILES. #877 was filed after a census did exactly that to six batteries
# and rewrote 13 files under py_router/ and py_placer/ -- then reported numbers
# measured against its own damage. Refusing the import outright, rather than
# hiding the runner behind `if __name__`, keeps the reason visible and points
# at the API that answers the question the importer actually had.
if __name__ != '__main__':                                 # pragma: no cover
    raise ImportError(
        'tests/mutate_760.py is a SCRIPT, not a module: importing it runs the '
        'battery and rewrites engine files in place. To read its rows, use '
        'tests/mutation_anchors.resolve_static(path), which parses the file '
        'instead of executing it.')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SER = os.path.join(ROOT, 'py_router/single_ended_routing.py')
PCM = os.path.join(ROOT, 'py_router/pcb_modification.py')
TEST = os.path.join(ROOT, 'tests/test_760_hole_local_clearance.py')

ROWS = [
  ('distance-fn-ignores-the-override', SER,
   "        d = d - np.maximum(0.0, hlc[near] - base_clearance)",
   "        d = d - 0.0 * np.maximum(0.0, hlc[near] - base_clearance)"),
  ('detector-drops-the-excess', PCM,
   "            d = d - hex_[None, :]  # #760 per-hole local_clearance excess",
   "            d = d - 0.0 * hex_[None, :]"),
  ('microshift-acceptance-reverted', PCM,
   "                                    base_clearance=npth_clr)  # #760",
   "                                    base_clearance=None)  # #760"),
  ('microshift-prefilter-reverted', PCM,
   "                                      base_clearance=npth_clr) < hole_thr",
   "                                      base_clearance=None) < hole_thr"),
  ('override-leaks-into-the-flat-floor', PCM,
   "    hole_floor = max(clearance, NPTH_TO_TRACK_CLEARANCE,\n"
   "                     resolve_hole_clearance(pcb_data, config))",
   "    hole_floor = max(clearance, NPTH_TO_TRACK_CLEARANCE,\n"
   "                     resolve_hole_clearance(pcb_data, config),\n"
   "                     float(_foreign_hole_capsules(pcb_data)[6].max())\n"
   "                     if _foreign_hole_capsules(pcb_data)[6].size else 0.0)"),
  ('window-not-widened-for-the-excess', PCM,
   "    R = max(required, hole_required + hole_excess_max) + 0.2",
   "    R = max(required, hole_required) + 0.2"),
  ('octolinear-acquires-the-term', PCM,
   "        hd = _seg_foreign_hole_dist(pcb_data, net_id, x1, y1, x2, y2)\n"
   "        return (d >= eff + w / 2.0 - 1e-4 and\n"
   "                hd >= npth_clr + w / 2.0 - 1e-4 and\n"
   "                edge_clears(x1, y1, x2, y2, w))",
   "        hd = _seg_foreign_hole_dist(pcb_data, net_id, x1, y1, x2, y2,\n"
   "                                    base_clearance=npth_clr)\n"
   "        return (d >= eff + w / 2.0 - 1e-4 and\n"
   "                hd >= npth_clr + w / 2.0 - 1e-4 and\n"
   "                edge_clears(x1, y1, x2, y2, w))"),
]

# Every anchor must match its target exactly once BEFORE anything is
# rewritten. A stale anchor otherwise reports BROKEN mid-run, after the
# witnesses have been paid for; this is the one second (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)
killed = surv = broken = 0

# A dirty target would be RESTORED to its committed text, silently destroying
# uncommitted work. Every other battery refuses; this one did not (#877).
_dirty = subprocess.run(
    ['git', 'status', '--porcelain', '--'] + sorted({r[1] for r in ROWS}),
    cwd=ROOT, capture_output=True, text=True).stdout.strip()
if _dirty:
    print('REFUSED: the files this battery rewrites have uncommitted changes.\n'
          'Restoring them writes the COMMITTED text back over your work.\n'
          + _dirty)
    sys.exit(2)

for row in ROWS:
    name, path, old, new = row[:4]
    nth = row[4] if len(row) > 4 else None
    # RAW BYTES for the restore, decoded text for the match (#877). This was
    # the worst of the four: no encoding AND no newline on either side, so it
    # round-tripped every target through the locale codec (cp1252 on Windows)
    # and translated every '\n' to os.linesep. `.gitattributes` pins `*.py text
    # eol=lf`, so a single run left every target permanently "modified".
    raw = open(path, 'rb').read()
    orig = raw.decode('utf-8').replace('\r\n', '\n')
    if nth is not None:            # replace the Nth occurrence only
        parts = orig.split(old)
        if len(parts) - 1 < nth + 1:
            print(f'  BROKEN {name}: need occurrence {nth}, found {len(parts)-1}')
            broken += 1; continue
        mut = old.join(parts[:nth+1]) + new + old.join(parts[nth+1:])
    else:
        if orig.count(old) != 1:
            print(f'  BROKEN {name}: anchor count {orig.count(old)}')
            broken += 1; continue
        mut = orig.replace(old, new)
    open(path, 'w', encoding='utf-8', newline='').write(mut)
    try:
        r = subprocess.run([sys.executable, TEST], capture_output=True, text=True,
                           cwd=ROOT, timeout=600)
        out = r.stdout + r.stderr
        if r.returncode != 0:
            n = out.count('FAIL:')
            print(f'  killed  {name}  ({n} assertion(s) red)')
            killed += 1
        else:
            print(f'  SURVIVED {name}')
            surv += 1
    finally:
        open(path, 'wb').write(raw)          # byte-exact, from what was read
print(f'\n{killed} killed, {surv} SURVIVED, {broken} broken')
sys.exit(1 if surv or broken else 0)
