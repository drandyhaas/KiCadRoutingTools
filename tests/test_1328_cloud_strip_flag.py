#!/usr/bin/env python3
"""cloud_replay_sets.py --strip-flag SCRIPT:FLAG: a corpus A/B of a CLI default.

A default is live on a recorded replay only where the step omitted the flag:
605 of 626 recorded qfn_fanout steps pass --clearance, so a stock replay
cannot measure #1328's new default at all. --strip-flag drops the flag from
every recorded SCRIPT step of an arm, through the arm spec's `manifest_sed`,
which modal_app applies to each CONTAINER's copy of the manifest -- the shared
kicad-corpus volume is never rewritten.

Checks (no Modal, no routing):
  1. The rewrite: `--clearance V` and `--clearance=V` go on qfn_fanout lines
     only; `--clearance-ceiling`, other tools' `--clearance` and comment lines
     stay; a bare flag goes without eating the next flag; a negative value is
     a value.
  2. A malformed spec is refused, with the reason.
  3. The census counts the lines and boards a launch would rewrite.
  4. stage_run puts the rewrite in the arm spec it hands modal_app (the Modal
     call itself is stubbed), and main() gives a stripped arm its own label.
  5. modal_app applies `manifest_sed` to the staged copy, not to /corpus.

    python3 tests/test_1328_cloud_strip_flag.py
"""
import argparse
import json
import os
import re
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tests' / 'stress'))
sys.path.insert(0, str(ROOT / 'tests' / 'stress' / 'modal_sweep'))

import cloud_replay_sets as crs                                     # noqa: E402
import sweep_lib                                                    # noqa: E402

fails = []


def check(name, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}{('  ' + str(detail)) if detail else ''}")
    if not ok:
        fails.append(name)


MANIFEST = """# cwd={stress}/runs_set1/b1
python3 -u /r/py_router/route_planes.py in.kicad_pcb s1.kicad_pcb --nets GND
python3 -u /r/py_router/qfn_fanout.py s1.kicad_pcb --output s2.kicad_pcb --component U1 --clearance 0.09 --width 0.1
python3 -u /r/py_router/qfn_fanout.py s2.kicad_pcb --output s3.kicad_pcb --clearance=0.15 --clearance-ceiling 0.2
python3 -u /r/py_router/route.py s3.kicad_pcb s4.kicad_pcb --clearance 0.1 --clearance-ceiling 0.1
# python3 /r/py_router/qfn_fanout.py old.kicad_pcb --clearance 0.2
"""

print("1. the rewrite")
subs = crs.strip_flag_subs(['qfn_fanout.py:--clearance'])
out = sweep_lib.patch_manifest(MANIFEST.format(stress='/s'), subs)
lines = out.splitlines()
check("qfn_fanout's --clearance V is gone, the rest of the line kept",
      lines[2].endswith('--component U1 --width 0.1'), lines[2])
check("--clearance=V is gone, --clearance-ceiling kept",
      lines[3].endswith('--output s3.kicad_pcb --clearance-ceiling 0.2'), lines[3])
check("route.py's --clearance is untouched", '--clearance 0.1' in lines[4], lines[4])
check("comment lines are untouched", lines[5].endswith('--clearance 0.2'), lines[5])
bare = sweep_lib.patch_manifest(
    "python3 /r/py_router/qfn_fanout.py a --allow-via-in-pad --escape-method underpad\n"
    "python3 /r/py_router/qfn_fanout.py a --extension -0.05 --width 0.1\n",
    crs.strip_flag_subs(['qfn_fanout.py:--allow-via-in-pad',
                         'qfn_fanout.py:--extension'])).splitlines()
check("a bare flag goes without eating the next flag",
      bare[0].endswith('qfn_fanout.py a --escape-method underpad'), bare[0])
check("a negative value is a value", bare[1].endswith('qfn_fanout.py a --width 0.1'),
      bare[1])

print("2. a malformed spec")
for bad in ('qfn_fanout.py', 'qfn_fanout.py:clearance', ':--clearance'):
    try:
        crs.strip_flag_subs([bad])
        check(f"{bad!r} refused", False, 'accepted')
    except SystemExit as e:
        check(f"{bad!r} refused, naming the form", 'want SCRIPT:FLAG' in str(e), e)

with tempfile.TemporaryDirectory(prefix='t1328strip_') as tmp:
    stress = Path(tmp)
    for b, text in (('b1', MANIFEST), ('b2', "# cwd={stress}/runs_set1/b2\n"
                                       "python3 /r/py_router/route.py a b\n")):
        d = stress / 'runs_set1' / b
        d.mkdir(parents=True)
        (d / 'redo_commands.sh').write_text(text.format(stress=stress))

    print("3. the census")
    n_lines, n_boards = crs.strip_flag_census(subs, ['set1'], stress)
    check("2 lines on 1 board", (n_lines, n_boards) == (2, 1), (n_lines, n_boards))

    print("4. the arm spec and the label")
    sent = []
    real = crs.sh_capture
    crs.sh_capture = lambda cmd, **kw: (sent.append(cmd) or (0, ''))
    try:
        args = argparse.Namespace(
            out=str(stress / 'wave'), workdir=str(stress / 'wave'), label='t',
            arm='', env=[], defaults=[], strip_flag=['qfn_fanout.py:--clearance'],
            limit=0, boards='', exclude='')
        crs.stage_run(args, ['set1'], stress, {})
    finally:
        crs.sh_capture = real
    arms = json.loads((stress / 'wave' / 'arms.replay.json').read_text())
    check("the Modal call was made (stubbed) with that arms file",
          bool(sent) and str(stress / 'wave' / 'arms.replay.json') in sent[0])
    check("the arm spec carries the rewrite as manifest_sed",
          arms[0].get('manifest_sed') == subs, arms[0].get('manifest_sed'))
    old_argv = sys.argv
    stops = []
    for attr in ('stage_plan', 'stage_upload', 'stage_run', 'stage_harvest'):
        stops.append((attr, getattr(crs, attr)))
    try:
        seen = {}
        crs.stage_plan = lambda a, *_: seen.setdefault('label', a.label) and {}
        crs.stage_upload = crs.stage_run = crs.stage_harvest = lambda *a, **k: None
        sys.argv = ['cloud_replay_sets.py', '--sets', 'set1', '--stress-dir', str(stress),
                    '--label', 'x', '--no-kicad', '--no-baseline', '--only', 'plan',
                    '--strip-flag', 'qfn_fanout.py:--clearance']
        crs.main()
    finally:
        sys.argv = old_argv
        for attr, fn in stops:
            setattr(crs, attr, fn)
    check("a stripped arm gets its own label", re.fullmatch(r'x-strip[0-9a-f]{6}',
                                                            seen.get('label', '')),
          seen.get('label'))

print("5. modal_app rewrites the staged copy, never /corpus")
src = (ROOT / 'tests' / 'stress' / 'modal_sweep' / 'modal_app.py').read_text()
check("manifest_sed is applied to dst/redo_commands.sh, a copytree of the corpus board",
      'shutil.copytree(Path(CORPUS) / f"runs_{set_name}" / board, dst)' in src
      and 'manifest_path = dst / "redo_commands.sh"' in src
      and 'sweep_lib.patch_manifest(text, spec["manifest_sed"])' in src)

if fails:
    print(f"\nFAILED: {fails}")
    sys.exit(1)
print("\nALL PASSED")
