#!/usr/bin/env python3
"""An under-pad via-drop names what holds a dropped pad's via sites, and
`check_reachability --part` says what it graded (#1981).

Routed copper laid between a QFN's pins before its fan-out takes the via
sites of the pins it was not asked to route: the fan-out then drops them,
"no clear via offset", and said nothing more, so the route step that cost
them went unnamed. `check_reachability --part` read deficit 0 on that board.
It sets the routing aside and counts nets that can leave by a track, not a
via site per pin, and it did not say so (its `--pad` form prints the
`routed ... set aside` line).

The fixture is general: a 0.4 mm QFN on an in-repo board with the copper of
every net with a pad on it removed. A one-net under-pad fan-out is the
control: the pad escapes, and its via shows where the via site is. A short
track of another of the part's nets is then laid across that site, and the
same fan-out runs again.

  1. the control escapes the pad, and nothing is dropped;
  2. with the track there, the pad is dropped and the console line and
     JSON_SUMMARY `dropped_blockers` name the track's net for it;
  3. `check_reachability --part` prints the `routed ... set aside` line and
     says it counts track exits, not a via site per pin; its JSON document
     carries `graded` and `routed_set_aside`.

    python3 tests/test_1981_dropped_via_site_named.py
"""
import json
import math
import os
import re
import shutil
import sys
import tempfile
import uuid

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402

SRC = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
PART = 'U2'
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def strip_part_copper(dst):
    """SRC with the tracks, arcs and vias of every net that has a pad on
    PART removed, its project beside it."""
    from kicad_parser import parse_kicad_pcb
    ids = {p.net_id for p in parse_kicad_pcb(SRC).footprints[PART].pads if p.net_id}
    text = open(SRC, encoding='utf-8').read()
    pat = re.compile(r'\n\t\((segment|via|arc)\b')
    out, pos = [], 0
    while True:
        m = pat.search(text, pos)
        if not m:
            out.append(text[pos:])
            break
        s, depth, j = m.start() + 2, 0, m.start() + 2
        while True:
            depth += (text[j] == '(') - (text[j] == ')')
            if depth == 0:
                break
            j += 1
        nm = re.search(r'\(net (\d+)\)', text[s:j + 1])
        out.append(text[pos:s])
        if not (nm and int(nm.group(1)) in ids):
            out.append(text[s:j + 1])
        pos = j + 1
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(''.join(out))
    shutil.copyfile(os.path.splitext(SRC)[0] + '.kicad_pro',
                    os.path.splitext(dst)[0] + '.kicad_pro')


def add_track(src, dst, x0, y0, x1, y1, net_id, layer):
    text = open(src, encoding='utf-8').read().rstrip()
    assert text.endswith(')')
    seg = (f'\t(segment (start {x0:.4f} {y0:.4f}) (end {x1:.4f} {y1:.4f}) '
           f'(width 0.1) (layer "{layer}") (net {net_id}) (uuid "{uuid.uuid4()}"))\n')
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(text[:-1] + seg + ')\n')
    shutil.copyfile(os.path.splitext(src)[0] + '.kicad_pro',
                    os.path.splitext(dst)[0] + '.kicad_pro')


def fanout(board, net, out):
    r = run_utils.check([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py',
                         run_utils.evidence(board), '--component', PART,
                         '--escape-method', 'underpad', '--nets', net,
                         '--output', out], accept=True)
    text = r.stdout + r.stderr
    m = re.search(r'^JSON_SUMMARY: (.*)$', text, re.M)
    return text, (json.loads(m.group(1)) if m else {})


def main():
    from kicad_parser import parse_kicad_pcb
    tmp = tempfile.mkdtemp(prefix='t1981_')
    try:
        bare = os.path.join(tmp, 'bare.kicad_pcb')
        strip_part_copper(bare)
        pcb = parse_kicad_pcb(bare)
        fp = pcb.footprints[PART]
        # a perimeter signal pin whose net reaches another part (a net with
        # no other pad is a no-connect, which the fan-out leaves alone)
        sig = sorted((p for p in fp.pads if p.net_id and p.net_name != 'GND'
                      and max(p.size_x, p.size_y) < 1.0
                      and any(q.component_ref != PART for q in pcb.nets[p.net_id].pads)),
                     key=lambda p: int(p.pad_number) if p.pad_number.isdigit() else 0)
        pad, other = sig[0], sig[-1]
        label = f"{PART}.{pad.pad_number}"

        print(f"1. control: one-net under-pad fan-out of {label}")
        out1 = os.path.join(tmp, 'o1.kicad_pcb')
        _t, s1 = fanout(bare, pad.net_name, out1)
        new = [v for v in parse_kicad_pcb(run_utils.evidence(out1)).vias
               if v.net_id == pad.net_id]
        ok("the pad escapes, nothing dropped",
           len(new) == 1 and s1.get('dropped_pads') == [],
           f"vias {len(new)}, dropped {s1.get('dropped_pads')}")
        if len(new) != 1:
            raise SystemExit("BROKEN TEST: the control laid no via to find the site by")
        vx, vy = new[0].x, new[0].y
        d = math.hypot(vx - pad.global_x, vy - pad.global_y)
        ux, uy = (vx - pad.global_x) / d, (vy - pad.global_y) / d

        print(f"2. a track of another net across that via site")
        blocked = os.path.join(tmp, 'blocked.kicad_pcb')
        add_track(bare, blocked, vx - uy * 0.1, vy + ux * 0.1, vx + uy * 0.1, vy - ux * 0.1,
                  other.net_id, fp.layer)
        text, s2 = fanout(blocked, pad.net_name, os.path.join(tmp, 'o2.kicad_pcb'))
        ok("the pad is dropped", s2.get('dropped_pads') == [label], f"{s2.get('dropped_pads')}")
        rows = s2.get('dropped_blockers') or []
        names = [w for r in rows if r.get('pad') == label for w in r.get('blockers', {})]
        ok("JSON_SUMMARY dropped_blockers names the track's net for the pad",
           any(f"track {other.net_name}" in w for w in names), f"{rows}")
        line = next((l for l in text.splitlines()
                     if l.strip().startswith(f"{label} ({pad.net_name}):")), '')
        ok("the console line names it too", f"track {other.net_name}" in line, line.strip())

        print("3. check_reachability --part says what it graded")
        jpath = os.path.join(tmp, 'part.json')
        r = run_utils.check([sys.executable, '-X', 'utf8', 'py_tools/check_reachability.py',
                             blocked, '--part', PART, '--json-out', jpath], accept=True)
        ok("a 'routed ... set aside' line",
           re.search(r'^routed +\d+ track segment\(s\) and \d+ via\(s\) of routing set '
                     r'aside', r.stdout, re.M) is not None, r.stdout[-400:])
        ok("it says it does not count a via site per pin",
           'not a via site beside each pin' in r.stdout)
        with open(run_utils.evidence(jpath, '--json-out document'), encoding='utf-8') as fh:
            doc = json.load(fh)
        ok("the JSON document carries graded and routed_set_aside",
           doc.get('graded') == 'placement'
           and (doc.get('routed_set_aside') or {}).get('segments', 0) > 0,
           f"{doc.get('graded')} {doc.get('routed_set_aside')}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAILED: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
