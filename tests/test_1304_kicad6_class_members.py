#!/usr/bin/env python3
"""#1304: a KiCad-6 project's net-class membership (`classes[].nets`) is read.

KiCad 6 listed a class's members inside the class. KiCad 7's net_settings
schema 2 -> 3 migration turns them into `netclass_patterns`, and it runs on
every load of a file still at schema 0-2, so KiCad 10 resolves those nets to
the class. Our two project readers -- `list_nets.read_design_rules` and
`DesignRules.from_project` -- read only `netclass_assignments` and
`netclass_patterns`, so steppenprobe's Power nets (track 0.3) routed at the
Default 0.15, and every class clearance / via / width consumer missed them.

Both readers now take membership from `design_rules.project_class_membership`.

Checks (cap_chain, a project written per case):
  1. schema 0, 1 and 2: the listed nets are Power in read_design_rules,
     DesignRules.memberships / draw_size, the width and clearance maps, and
     check_drc's per-net map.
  2. schema 3 and 4, and a project with no `meta`: the key is not membership
     (KiCad ignores it there), and nothing resolves to Power.
  3. `list_nets --design-rules` names the two assigned nets.
  4. route.py with --track-width omitted draws the member net at 0.3.
  5. When a KiCad python is installed: pcbnew's GetEffectiveNetClass agrees
     with our resolution on every case (the half that pins the schema rule
     to KiCad itself; reported as not run without one).

    python3 tests/test_1304_kicad6_class_members.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
MEMBERS = ['DPA_P', 'DPA_N']
OTHERS = ['DPB_P', 'DPB_N']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def project(version):
    """A KiCad-6-shaped project: Power lists its members in the class."""
    cls = dict(clearance=0.15, track_width=0.15, via_diameter=0.6,
               via_drill=0.3, diff_pair_gap=0.25, diff_pair_width=0.2)
    ns = {'classes': [dict(cls, name='Default', priority=2147483647),
                      dict(cls, name='Power', clearance=0.2, track_width=0.3,
                           nets=list(MEMBERS), priority=0)]}
    if version is not None:
        ns['meta'] = {'version': version}
    return {'meta': {'filename': 'b.kicad_pro', 'version': 1},
            'board': {'design_settings': {'rules': {'min_clearance': 0.0}}},
            'net_settings': ns}


def stage(tmp, tag, version):
    d = os.path.join(tmp, tag)
    os.makedirs(d)
    pcb = os.path.join(d, 'b.kicad_pcb')
    copy_board(BOARD, pcb)
    with open(os.path.join(d, 'b.kicad_pro'), 'w') as f:
        json.dump(project(version), f, indent=2)
    return pcb


def resolved(pcb):
    """What every reader says about each net: {net: (assigned class,
    DesignRules classes, draw width, width-map, clearance-map, check_drc map)}."""
    from design_rules import DesignRules
    from list_nets import (read_design_rules, resolve_net_class,
                           net_track_width_map_by_id, net_clearance_map_by_id,
                           net_clearance_map)
    data = parse_kicad_pcb(pcb)
    names = {nid: n.name for nid, n in data.nets.items() if n.name}
    rules = read_design_rules(pcb)
    dr = DesignRules.from_project(data, pcb)
    widths = net_track_width_map_by_id(pcb, names)
    clrs = net_clearance_map_by_id(pcb, names)
    by_name = net_clearance_map(pcb, list(names.values()))
    out = {}
    for nid, name in names.items():
        out[name] = (resolve_net_class(name, rules),
                     sorted(dr.memberships.get(nid, ())),
                     dr.draw_size('track_width', nid),
                     widths.get(nid), clrs.get(nid), by_name.get(name))
    return out


def pcbnew_python():
    from kicad_locate import kicad_python_candidates
    for c in kicad_python_candidates(this_interpreter=sys.executable):
        if not c or not os.path.isfile(c):
            continue
        try:
            r = subprocess.run([c, '-c', 'import pcbnew'], capture_output=True,
                               timeout=120)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if r.returncode == 0:
            return c
    return None


# The effective class of a net in several classes is an aggregate KiCad names
# "Power,Default"; its first non-Default member is the one that sets the
# values (fanout_gui._get_net_classes_from_board reads it the same way).
PCBNEW_PROBE = r'''
import json, sys, pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
ns = b.GetDesignSettings().m_NetSettings
out = {}
for n in sys.argv[2:]:
    nc = ns.GetEffectiveNetClass(n)
    parts = [p.strip() for p in str(nc.GetName()).split(',')]
    nd = [p for p in parts if p != 'Default']
    out[n] = [nd[0] if nd else 'Default', nc.GetTrackWidth() / 1e6]
print(json.dumps(out))
'''


def main():
    evidence(BOARD, 'the cap_chain fixture')
    cases = [('v0', 0, True), ('v1', 1, True), ('v2', 2, True),
             ('v3', 3, False), ('v4', 4, False), ('nometa', None, False)]
    with tempfile.TemporaryDirectory() as tmp:
        boards = {}
        print('1-2. every reader, per schema version:')
        for tag, version, member in cases:
            pcb = boards[tag] = stage(tmp, tag, version)
            res = resolved(pcb)
            for n in MEMBERS:
                got = res[n]
                want = (('Power', ['Power'], 0.3, 0.3, 0.2, 0.2) if member
                        else ('Default', [], 0.15, None, None, 0.15))
                check(f"{tag}: {n} -> {'Power' if member else 'Default'}",
                      got == want, f"got {got}")
            for n in OTHERS:
                check(f"{tag}: {n} stays Default", res[n][0] == 'Default'
                      and res[n][1] == [], f"got {res[n]}")

        print('3. list_nets --design-rules names the members:')
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/list_nets.py',
                            boards['v2'], '--design-rules'], cwd=ROOT,
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace')
        check('list_nets exit 0', r.returncode == 0, r.stderr[-300:])
        check('"2 net(s) assigned to non-default classes"',
              '2 net(s) assigned to non-default classes' in r.stdout,
              r.stdout[-400:])

        print('4. route.py, --track-width omitted, draws the class width:')
        out = os.path.join(tmp, 'v2', 'routed.kicad_pcb')
        r = run_check([sys.executable, '-X', 'utf8', 'py_router/route.py',
                       boards['v2'], out, '--nets', 'DPA_P'], accept=True)
        log = r.stdout + r.stderr
        check('the class width is auto-read',
              'Auto-read netclass track widths for 2 non-Default net(s)' in log,
              log[-400:])
        evidence(out, 'the routed board')
        a, b = parse_kicad_pcb(boards['v2']), parse_kicad_pcb(out)
        old = {(s.start_x, s.start_y, s.end_x, s.end_y, s.layer) for s in a.segments}
        new = [s for s in b.segments if b.nets[s.net_id].name == 'DPA_P'
               and (s.start_x, s.start_y, s.end_x, s.end_y, s.layer) not in old]
        wide = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                   for s in new if abs(s.width - 0.3) < 1e-6)
        total = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in new)
        check('DPA_P routed', total > 0, f"{total:.2f} mm new copper")
        check('most of it at the Power class width 0.3',
              total > 0 and wide / total > 0.8,
              f"{wide:.2f} of {total:.2f} mm at 0.3")

        print('5. pcbnew agrees on every case:')
        py = pcbnew_python()
        if py is None:
            print('  [not run] no KiCad python with pcbnew on this machine')
        else:
            for tag, version, member in cases:
                r = subprocess.run([py, '-c', PCBNEW_PROBE, boards[tag]]
                                   + MEMBERS + OTHERS, capture_output=True,
                                   text=True, timeout=300)
                try:
                    kicad = json.loads(r.stdout.strip().splitlines()[-1])
                except (ValueError, IndexError):
                    check(f"{tag}: pcbnew probe ran", False,
                          (r.stdout + r.stderr)[-400:])
                    continue
                ours = {n: [v[0], v[2]] for n, v in resolved(boards[tag]).items()
                        if n in kicad}
                if version is None:
                    # KiCad loads no net_settings at all without `meta` (its
                    # factory Default, 0.2), while our readers still take the
                    # class VALUES from the file -- a separate question from
                    # membership, so only the class is compared here.
                    kicad = {n: v[0] for n, v in kicad.items()}
                    ours = {n: v[0] for n, v in ours.items()}
                check(f"{tag}: pcbnew == ours", kicad == ours,
                      f"pcbnew {kicad} ours {ours}")

    if failures:
        print(f"\nFAILED ({len(failures)}): {failures}")
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
