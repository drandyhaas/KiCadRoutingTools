#!/usr/bin/env python3
"""#1352: place_pose's stdout summary caps its per-pad and per-pair lists;
`--json PATH` keeps every entry.

On pico_ice_rev3's 97-part pile one `set` printed 315 KB to stdout: the
JSON_SUMMARY line was 291 KB, 245 KB of it the pad-edge findings and
unmeasured rows before and after, and the text report printed one line per
pad-edge finding. A refusal's one-line reason was buried in it.

The fixture is a pile made from esp_prog: every part moved off the outline
onto a grid, so nearly every pad is a pad-edge finding.

Checks:
  1. `pose_ops.stdout_summary` (pure): a list longer than the cap is cut to
     it with `<list>_count` and `<list>_truncated`; a short list and the
     scalar counts are untouched; the pad-edge grades' lists are capped the
     same way; the input is not mutated.
  2. place_pose on the pile: the stdout summary's pad-edge findings are
     capped at 20 with the full count beside them, `pad_edge_conflicts_after`
     still counts all of them, and `--json` holds every one.
  3. The text report prints 5 pad-edge findings and says how many more.
  4. A refusal (an unknown ref, exit 2) writes its summary to `--json` too.

    python3 tests/test_1352_place_pose_stdout_cap.py
"""
import contextlib
import copy
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from copy_board import copy_board                                   # noqa: E402
from kicad_parser import parse_kicad_pcb                            # noqa: E402
from placement.pose_ops import stdout_summary, STDOUT_LIST_CAP      # noqa: E402
from placement.writer import write_placed_output                    # noqa: E402
from run_utils import check as run_check, evidence                  # noqa: E402

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
STRIP = os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py')
POSE = os.path.join(ROOT, 'py_placer', 'place_pose.py')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def summary(r):
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout or '', re.M)
    return json.loads(m.group(1)) if m else {}


def main():
    print("1. stdout_summary")
    cap = STDOUT_LIST_CAP
    full = {'pad_stack_pairs_after': [['A', str(i)] for i in range(50)],
            'own_pad_short_pairs_after': [['U1.1', 'U1.2']],
            'pad_stack_count_after': 50, 'moved': ['R%d' % i for i in range(40)],
            'pad_edge_after': {'findings': [{'pad_ref': 'C%d.1' % i}
                                            for i in range(30)],
                               'unmeasured': [], 'complete': False}}
    before = copy.deepcopy(full)
    out = stdout_summary(full)
    check('a long list is cut to the cap, with its count and truncation',
          len(out['pad_stack_pairs_after']) == cap
          and out['pad_stack_pairs_after_count'] == 50
          and out['pad_stack_pairs_after_truncated'] == 50 - cap)
    check('a short list is untouched and carries no count key',
          out['own_pad_short_pairs_after'] == [['U1.1', 'U1.2']]
          and 'own_pad_short_pairs_after_count' not in out)
    check('the scalar counts and the request lists are untouched',
          out['pad_stack_count_after'] == 50 and len(out['moved']) == 40)
    pe = out['pad_edge_after']
    check('a pad-edge grade is capped the same way',
          len(pe['findings']) == cap and pe['findings_count'] == 30
          and pe['findings_truncated'] == 30 - cap and pe['unmeasured'] == []
          and pe['complete'] is False)
    check('the input is not mutated', full == before)

    work = tempfile.mkdtemp(prefix='krt1352_')
    try:
        esp = os.path.join(work, 'esp.kicad_pcb')
        quiet(copy_board, ESP, esp)
        bare = os.path.join(work, 'bare.kicad_pcb')
        run_check([sys.executable, '-X', 'utf8', STRIP, esp, bare], accept=True)
        pile = os.path.join(work, 'pile.kicad_pcb')
        refs = sorted(quiet(parse_kicad_pcb, bare).footprints)
        quiet(write_placed_output, bare, pile, [
            {'reference': r, 'new_x': 300.0 + (i % 6) * 15.0,
             'new_y': 300.0 + (i // 6) * 15.0, 'new_rotation': 0.0}
            for i, r in enumerate(refs)])
        evidence(pile, 'the pile')

        print("2. place_pose on the pile")
        full_json = os.path.join(work, 'full.json')
        r = run_check([sys.executable, '-X', 'utf8', POSE, pile,
                       os.path.join(work, 'out.kicad_pcb'), '--dry-run',
                       'lock', 'U1', '--json', full_json], accept=True)
        s = summary(r)
        with open(evidence(full_json, 'place_pose --json'),
                  encoding='utf-8') as fh:
            whole = json.load(fh)
        n = len(whole['pad_edge_after']['findings'])
        pe = s.get('pad_edge_after') or {}
        check('the pile has more pad-edge findings than the cap', n > cap,
              str(n))
        check('stdout carries the first %d, with the full count' % cap,
              len(pe.get('findings') or ()) == cap
              and pe.get('findings_count') == n
              and pe.get('findings_truncated') == n - cap,
              f"{len(pe.get('findings') or ())} / {pe.get('findings_count')}")
        check('pad_edge_conflicts_after still counts all of them',
              s.get('pad_edge_conflicts_after') == n
              == whole['pad_edge_conflicts_after'])
        check('--json carries every entry, uncapped',
              'findings_count' not in whole['pad_edge_after']
              and whole['pad_edge_after']['findings'][:cap] == pe['findings'])
        line = [ln for ln in r.stdout.splitlines()
                if ln.startswith('JSON_SUMMARY: ')][0]
        check('the stdout line is under half the full document',
              len(line) * 2 < len(json.dumps(whole)),
              f"{len(line)} vs {len(json.dumps(whole))} bytes")

        print("3. the text report")
        shown = [ln for ln in r.stdout.splitlines()
                 if re.match(r'  \S+: edge shortfall', ln)]
        check('5 pad-edge lines, then how many more',
              len(shown) == 5 and ('... and %d more' % (n - 5)) in r.stdout,
              str(len(shown)))

        print("4. a refusal")
        ref_json = os.path.join(work, 'refused.json')
        r = run_check([sys.executable, '-X', 'utf8', POSE, pile,
                       os.path.join(work, 'out2.kicad_pcb'), '--dry-run',
                       'lock', 'NOPE9', '--json', ref_json], code=2,
                      refuse='NOPE9')
        with open(evidence(ref_json, 'a refusal --json'),
                  encoding='utf-8') as fh:
            doc = json.load(fh)
        check('the refusal is in the file and on stdout',
              doc.get('exit_code') == 2 and 'NOPE9' in (doc.get('refused') or '')
              and summary(r).get('refused') == doc['refused'])
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print(f"\n{'FAILED' if failures else 'PASSED'}: {len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
