#!/usr/bin/env python3
"""#900 mutation battery: each row breaks ONE part of the fix and must turn
tests/test_900_class_clearance_not_capped.py red.

The fix separates two quantities that shared one dict key: `rules.min_clearance`
(an absolute floor, capped at the smallest pad clearance override and the
smallest .kicad_dru layer rule) and the NET-CLASS clearance (the routed value,
capped by neither). Most of the rows below restore one of the four places the
capped value used to reach a class.

TWO ROWS EXIST BECAUSE THE FALLBACK HIDES THEM. `_class_clearance` falls back to
`min_clearance` so hand-built `{'min_clearance': ceiling}` dicts still work
(gui_utils, the fanout tab), which makes the no-cap case byte-identical either
way. MEASURED, because the obvious prediction was wrong both times:

  * `class-key-emitted-only-when-the-cap-fires` -- predicted "survives every
    behaviour arm, killed by the unconditional-emission assertion alone".
    It is killed by THREE: that assertion, plus both class checks in the
    .kicad_dru arm. That board carries no pad override, so the key is absent,
    so the fallback hands the class the DRU-capped floor. The dru arm turns out
    to be a second, independent witness for this row -- worth knowing before
    anyone "simplifies" it away.
  * `fallback-dropped` -- killed by exactly the two hand-built-dict arms, as
    predicted, and by nothing else. Every arm using a real `compute_targets`
    result is blind to it.

ONE EXPECTED SURVIVOR-BY-INSTRUMENT, named rather than hidden:
`live-default-class-reverted` mutates `apply_targets_to_board`, which opens with
`import pcbnew` and cannot be driven wx-free. It is killed here by the test's
SOURCE guard, not by behaviour -- a weaker instrument, and the reason
tests/gui_parity/test_900_live_class_clearance.py exists. If that row ever
starts reporting SURVIVED, the source guard has gone stale, not the fix.
"""
import os
import subprocess
import sys

# Importing this file would run the battery and rewrite an engine file in
# place. Refuse loudly and point at the API that reads rows without executing.
if __name__ != '__main__':                                 # pragma: no cover
    raise ImportError(
        'tests/mutate_900.py is a SCRIPT, not a module: importing it runs the '
        'battery and rewrites engine files in place. To read its rows, use '
        'tests/mutation_anchors.resolve_static(path), which parses the file '
        'instead of executing it.')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FKD = os.path.join(ROOT, 'py_router/fix_kicad_drc_settings.py')
TEST = os.path.join(ROOT, 'tests/test_900_class_clearance_not_capped.py')

ROWS = [
  # --- the four writers of a class clearance -----------------------------
  ('project-default-class-reverted', FKD,
   '    nc_map = {"clearance": _class_clearance(targets)}',
   '    nc_map = {"clearance": targets.get("min_clearance")}'),
  ('live-default-class-reverted', FKD,
   '    nc_map = {"SetClearance": _class_clearance(targets)}    # routed, not capped (#900)',
   '    nc_map = {"SetClearance": targets.get("min_clearance")}'),
  ('live-non-default-clamp-reverted', FKD,
   '    nd_map = {"SetClearance": _class_clearance(targets)}    # routed, not capped (#900)',
   '    nd_map = {"SetClearance": (targets or {}).get("min_clearance")}'),
  ('created-class-keeps-the-0.2-template', FKD,
   '        if nc_map.get("clearance") is not None:\n'
   '            default_cls["clearance"] = round(float(nc_map["clearance"]), 6)',
   '        if False:\n'
   '            default_cls["clearance"] = round(float(nc_map["clearance"]), 6)'),

  # --- the caps must not reach the class ---------------------------------
  ('pad-override-cap-leaks-into-the-class-again', FKD,
   '        _ovr = minima.get("min_pad_clearance_override")\n'
   '        if _ovr is not None and _ovr > 0 and clearance > _ovr:\n'
   '            targets["min_clearance"] = round(float(_ovr), 6)',
   '        _ovr = minima.get("min_pad_clearance_override")\n'
   '        if _ovr is not None and _ovr > 0 and clearance > _ovr:\n'
   '            targets["min_clearance"] = round(float(_ovr), 6)\n'
   '            targets["class_clearance"] = targets["min_clearance"]'),
  ('dru-cap-leaks-into-the-class', FKD,
   '            targets["min_clearance"] = _dru_min',
   '            targets["min_clearance"] = _dru_min\n'
   '            targets["class_clearance"] = _dru_min',
   0),                       # the fix_project_for_output site, not the board one
  ('cap-is-an-assignment-not-a-min', FKD,
   '        if _ovr is not None and _ovr > 0 and clearance > _ovr:',
   '        if _ovr is not None and _ovr > 0:'),

  # --- the non-rule key must not reach the project -----------------------
  ('rule-allow-list-dropped', FKD,
   '    for key, target in targets.items():\n'
   '        if key not in _RULE_KEYS:\n'
   '            continue          # #900: class_clearance is not a KiCad rule name\n'
   '        if target is None:',
   '    for key, target in targets.items():\n'
   '        if target is None:'),

  # --- the two the fallback hides ----------------------------------------
  ('class-key-emitted-only-when-the-cap-fires', FKD,
   '        targets["class_clearance"] = clearance',
   '        if minima.get("min_pad_clearance_override"):\n'
   '            targets["class_clearance"] = clearance'),
  ('fallback-dropped', FKD,
   '    return t.get("class_clearance", t.get("min_clearance"))',
   '    return t.get("class_clearance")'),
]

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)

killed = surv = broken = 0

# A dirty target would be RESTORED to its committed text, silently destroying
# uncommitted work (#877). Refuse instead.
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
    # RAW BYTES for the restore, decoded text for the match (#877): a
    # round-trip through the locale codec leaves every target permanently
    # "modified" under `*.py text eol=lf`.
    raw = open(path, 'rb').read()
    orig = raw.decode('utf-8').replace('\r\n', '\n')
    if nth is not None:
        parts = orig.split(old)
        if len(parts) - 1 < nth + 1:
            print(f'  BROKEN {name}: need occurrence {nth}, found {len(parts)-1}')
            broken += 1
            continue
        mut = old.join(parts[:nth + 1]) + new + old.join(parts[nth + 1:])
    else:
        if orig.count(old) != 1:
            print(f'  BROKEN {name}: anchor count {orig.count(old)}')
            broken += 1
            continue
        mut = orig.replace(old, new)
    open(path, 'w', encoding='utf-8', newline='').write(mut)
    try:
        r = subprocess.run([sys.executable, '-X', 'utf8', TEST],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=ROOT, timeout=900)
        out = r.stdout + r.stderr
        if r.returncode != 0:
            n = out.count('  FAIL ')
            print(f'  killed  {name}  ({n} assertion(s) red)')
            killed += 1
        else:
            print(f'  SURVIVED {name}')
            surv += 1
    finally:
        open(path, 'wb').write(raw)          # byte-exact, from what was read
print(f'\n{killed} killed, {surv} SURVIVED, {broken} broken')
sys.exit(1 if surv or broken else 0)
