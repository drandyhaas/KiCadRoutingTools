#!/usr/bin/env python3
"""#653: the routing mains must record which KICAD_* knobs were active.

Knobs are read once at import and were never echoed, so a wave launched from a
shell that exported KICAD_* inherited them SILENTLY and its logs were
indistinguishable from a clean run. Measured cost: an entire A/B baseline
(orangecrab, "33 issues") was env-contaminated -- the same commit, flags, .so
and python graded 37 in a clean environment -- and the cause could only be
INFERRED because nothing in the log recorded what was set.

Pins both halves: the startup LINE (for old logs) and the JSON_SUMMARY dict
(for harnesses), plus the filter that keeps KiCad's own install paths out.
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

import env_knobs  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok); failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


def _clean_env(**extra):
    e = {k: v for k, v in os.environ.items()
         if not (k.startswith('KICAD') or k.startswith('KRT_'))}
    e.update(extra)
    return e


def test_inventory_keeps_knobs_and_drops_install_paths():
    old = dict(os.environ)
    try:
        for k in list(os.environ):
            if k.startswith('KICAD') or k.startswith('KRT_'):
                del os.environ[k]
        check("a clean environment reports none",
              env_knobs.active_env_knobs() == {} and
              env_knobs.env_knobs_line() == "ENV KNOBS: none",
              env_knobs.env_knobs_line())
        os.environ['KICAD_RIPUP_COST'] = '7'
        os.environ['KRT_JOIN_VERIFY_DEBUG'] = '1'
        # KiCad's OWN variables: install paths, not routing knobs. Left in,
        # they bury the real knobs under a dozen lines inside the GUI.
        os.environ['KICAD9_FOOTPRINT_DIR'] = '/opt/kicad/footprints'
        os.environ['KICAD_STOCK_DATA_HOME'] = '/opt/kicad'
        got = env_knobs.active_env_knobs()
        check("routing knobs are reported",
              got.get('KICAD_RIPUP_COST') == '7' and
              got.get('KRT_JOIN_VERIFY_DEBUG') == '1', str(got))
        check("KiCad install paths are excluded",
              'KICAD9_FOOTPRINT_DIR' not in got and
              'KICAD_STOCK_DATA_HOME' not in got, str(got))
        # A knob this build does not know about must STILL be reported: the
        # point is what the shell handed the process, not what we can parse.
        os.environ['KICAD_KNOB_FROM_SOME_BRANCH'] = 'x'
        check("an unknown KICAD_* knob is still reported",
              'KICAD_KNOB_FROM_SOME_BRANCH' in env_knobs.active_env_knobs())
    finally:
        os.environ.clear(); os.environ.update(old)


def test_startup_line_is_printed_both_ways():
    """`none` must be printed too -- if silence meant clean, silence would
    again be ambiguous with 'this build does not log knobs' (the #654 lesson)."""
    tool = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_router', 'check_drc.py'), BOARD]
    r = subprocess.run(tool, capture_output=True, text=True, timeout=900,
                       env=_clean_env())
    check("clean run prints 'ENV KNOBS: none'",
          'ENV KNOBS: none' in r.stdout,
          str(r.stdout.split(chr(10))[:4]))
    r = subprocess.run(tool, capture_output=True, text=True, timeout=900,
                       env=_clean_env(KICAD_RIPUP_COST='7'))
    hits = re.findall(r'^ENV KNOBS:.*$', r.stdout, re.M)
    check("contaminated run names the knob and its value",
          len(hits) == 1 and 'KICAD_RIPUP_COST=7' in hits[0], str(hits))


def test_json_summary_carries_the_knobs():
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, 'in.kicad_pcb')
        subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'copy_board.py'),
                        BOARD, src], capture_output=True, timeout=300)
        js = os.path.join(d, 's.json')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route.py'),
             src, os.path.join(d, 'out.kicad_pcb'), '--nets', '/LED_*',
             '--layers', 'F.Cu', 'B.Cu', '--clearance', '0.2',
             '--track-width', '0.2', '--json-out', js],
            capture_output=True, text=True, timeout=1800,
            env=_clean_env(KICAD_RIPUP_COST='7'))
        check("route.py completed", r.returncode == 0, r.stdout[-300:])
        doc = json.load(open(js)) if os.path.isfile(js) else None
        doc = doc[0] if isinstance(doc, list) and doc else doc
        check("JSON_SUMMARY carries env_knobs",
              isinstance(doc, dict) and
              doc.get('env_knobs', {}).get('KICAD_RIPUP_COST') == '7',
              str(doc.get('env_knobs') if isinstance(doc, dict) else doc))


if __name__ == '__main__':
    test_inventory_keeps_knobs_and_drops_install_paths()
    test_startup_line_is_printed_both_ways()
    test_json_summary_carries_the_knobs()
    print(f"\n{passed}/{passed + failed} checks passed")
    sys.exit(1 if failed else 0)
