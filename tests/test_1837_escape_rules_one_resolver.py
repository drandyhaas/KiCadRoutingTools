"""#1837: every escape instrument prices a board through ONE resolver, and
says where each rule came from.

On a board with no `.kicad_pro`, the lane ledger (`check_floorplan --health`,
`board_brief`) and `check_reachability --part` priced lanes at
routing_defaults' track 0.3 / clearance 0.25 and did not say so, while
`check_reachability --pad` priced the same board at 0.15 / 0.2 / via 0.6 and
labelled it "fixed default". Now all three go through `escape.lane_rules`
(`list_nets.board_floor` per rule, one fallback set) and publish the source:

  * a bare board: one set of values on every reader, each 'fixed default';
  * the same board beside a project: the project's Default class, each
    'board netclass';
  * `--pad`'s knobs line, `--part`'s line and JSON, the ledger rows and the
    `--health` header all name the rules.

    python3 tests/test_1837_escape_rules_one_resolver.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import routing_defaults as defaults                            # noqa: E402
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import escape                                  # noqa: E402
from run_utils import check, evidence                         # noqa: E402

FAILS = []
CLI = os.path.join(ROOT, 'py_tools', 'check_reachability.py')
FLOORPLAN = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')
FALLBACK = {'track_width': defaults.TRACK_WIDTH,
            'clearance': defaults.CLEARANCE,
            'via_diameter': defaults.VIA_SIZE}
DECLARED = {'track_width': 0.15, 'clearance': 0.15, 'via_diameter': 0.45}
KEYS = ('track_width', 'clearance', 'via_diameter')


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def verdict(argv):
    """Run a check_reachability verdict: exit 0 (PASSABLE / not proven short)
    and 1 (CAGED / PROVEN SHORT) are both answers whose header this test
    reads; anything else, or a traceback, is a broken test."""
    r = subprocess.run([sys.executable, '-X', 'utf8', CLI] + argv,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=600)
    out = (r.stdout or '') + (r.stderr or '')
    if r.returncode not in (0, 1) or 'Traceback' in out:
        raise AssertionError(f'BROKEN TEST: {argv} exited {r.returncode}\n'
                             f'{out[-1200:]}')
    return r


def board(path):
    """U1: 8 pads at 0.5 mm pitch in two rows, each net also on X1, with X1
    and a neighbour R1 straight across U1's south face so the face is short
    at any rule set."""
    n = 8
    nets = ''.join(f'  (net {k} "S{k}")\n' for k in range(1, n + 1))
    nets += f'  (net {n + 1} "Q")\n'
    upads = ''.join(
        f'    (pad "{k}" smd rect (at {(k - 1) % 4 * 0.5} {(k - 1) // 4 * 2})'
        f' (size 0.3 0.6) (layers "F.Cu") (net {k} "S{k}"))\n'
        for k in range(1, n + 1))
    xpads = ''.join(f'    (pad "{k}" smd rect (at 0 {k - 1}) (size 0.3 0.3)'
                    f' (layers "F.Cu") (net {k} "S{k}"))\n'
                    for k in range(1, n + 1))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n' + nets +
                 '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default))'
                 ' (layer "Edge.Cuts"))\n'
                 '  (footprint "t:QFN" (layer "F.Cu") (at 10 10)\n'
                 '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
                 + upads + '  )\n'
                 '  (footprint "t:BAR" (layer "F.Cu") (at 10.75 12.9)\n'
                 '    (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))\n'
                 f'    (pad "1" smd rect (at 0 0) (size 3.0 0.5) (layers "F.Cu")'
                 f' (net {n + 1} "Q"))\n'
                 '  )\n'
                 '  (footprint "t:FAR" (layer "F.Cu") (at 30 10)\n'
                 '    (property "Reference" "X1" (at 0 0) (layer "F.SilkS"))\n'
                 + xpads + '  )\n)\n')
    return path


def main(td):
    bare_dir, proj_dir = os.path.join(td, 'bare'), os.path.join(td, 'proj')
    os.makedirs(bare_dir)
    os.makedirs(proj_dir)
    bare = board(os.path.join(bare_dir, 'b.kicad_pcb'))
    proj = board(os.path.join(proj_dir, 'b.kicad_pcb'))
    with open(os.path.join(proj_dir, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump({'net_settings': {'classes': [{
            'name': 'Default', 'clearance': DECLARED['clearance'],
            'track_width': DECLARED['track_width'],
            'via_diameter': DECLARED['via_diameter'], 'via_drill': 0.2}]}}, fh)

    # -- the resolver --------------------------------------------------------
    ok('placement.escape has the shared resolver', hasattr(escape, 'lane_rules'))
    if hasattr(escape, 'lane_rules'):
        resolver(bare, proj)
    cli(td, bare, proj)


def resolver(bare, proj):
    r = escape.lane_rules(parse_kicad_pcb(bare), bare)
    got = {k: (getattr(r, k), r.sources[k]) for k in KEYS}
    ok('bare board: lane_rules falls back to routing_defaults, each named '
       "'fixed default'",
       got == {k: (FALLBACK[k], 'fixed default') for k in KEYS}, got)
    r = escape.lane_rules(parse_kicad_pcb(proj), proj)
    got = {k: (getattr(r, k), r.sources[k]) for k in KEYS}
    ok("with a project: the Default class, each named 'board netclass'",
       got == {k: (DECLARED[k], 'board netclass') for k in KEYS},
       got)
    r = escape.lane_rules(parse_kicad_pcb(proj), proj, clearance=0.2,
                          sources={'clearance': 'board minimum clearance'})
    ok('an explicit value keeps the label its caller gave it',
       (r.clearance, r.sources['clearance'], r.sources['track_width'])
       == (0.2, 'board minimum clearance', 'board netclass'), r.sources)

    # -- the ledger publishes them -----------------------------------------
    with contextlib.redirect_stdout(io.StringIO()):
        led = escape.escape_ledger(parse_kicad_pcb(bare), pcb_file=bare,
                                   refs=['U1'])
    d = led[0].to_dict()
    ok('ledger row: lane_rules names the fallback and its source',
       d['lane_rules'] == {k: {'value': FALLBACK[k], 'source': 'fixed default'}
                           for k in KEYS}
       and abs(d['faces'][0]['lane_pitch_mm']
               - (FALLBACK['track_width'] + FALLBACK['clearance'])) < 1e-9,
       d.get('lane_rules'))


def cli(td, bare, proj):
    # -- check_reachability: --pad and --part agree, and both say so --------
    for path, want, src in ((bare, FALLBACK, 'fixed default'),
                            (proj, DECLARED, 'board netclass')):
        tag = os.path.basename(os.path.dirname(path))
        pad = verdict([path, '--pad', 'U1.1'])
        knobs = next((l for l in pad.stdout.splitlines()
                      if l.startswith('knobs')), '')
        ok(f'{tag}: --pad knobs line prices at {want} [{src}]',
           f"clearance {want['clearance']}mm, track {want['track_width']}mm, "
           f"via {want['via_diameter']}mm" in knobs
           and f'clearance: {src}' in knobs and f'track width: {src}' in knobs
           and f'via diameter: {src}' in knobs, knobs)
        js = os.path.join(td, f'{tag}_part.json')
        part = verdict([path, '--part', 'U1', '--json-out', js])
        d = json.load(open(evidence(js, f'{tag} --part JSON')))
        ok(f'{tag}: --part prices at the same rules as --pad',
           (d['rules']['track_mm'], d['rules']['clearance_mm'],
            d['rules']['via_mm'])
           == (want['track_width'], want['clearance'], want['via_diameter']),
           d['rules'])
        ok(f'{tag}: --part JSON and line name the source',
           d.get('rule_sources') == {k: src for k in KEYS}
           and f"track {want['track_width']:g} [{src}]" in part.stdout
           and f"via {want['via_diameter']:g} [{src}]" in part.stdout,
           (d.get('rule_sources'), part.stdout.splitlines()[:1]))

    # -- the --health header -------------------------------------------------
    intent = os.path.join(td, 'intent.json')
    check([sys.executable, '-X', 'utf8', FLOORPLAN, bare, '--emit-intent', intent],
          accept=True)
    h = check([sys.executable, '-X', 'utf8', FLOORPLAN, bare, '--intent',
               evidence(intent, 'emitted intent'), '--health', '--exit-zero'],
              accept=True)
    lines = h.stdout.splitlines()
    at = next((i for i, l in enumerate(lines) if 'escape lanes (' in l), None)
    ok('--health: the escape lanes block names the rules and their source',
       at is not None and 'priced at track' in lines[at + 1]
       and '[fixed default]' in lines[at + 1],
       lines[at:at + 2] if at is not None else 'no escape lanes block')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as td:
        main(td)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
