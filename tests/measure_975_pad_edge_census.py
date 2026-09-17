#!/usr/bin/env python3
"""#975: the pad-edge grade, censused, so the EdgeCopperContext refactor is
proven an identity against the commit before it -- not assumed one.

    python3 -B -X utf8 tests/measure_975_pad_edge_census.py --root <base-tree> --out base.json
    python3 -B -X utf8 tests/measure_975_pad_edge_census.py --out branch.json
    python3 -B -X utf8 tests/measure_975_pad_edge_census.py --diff base.json branch.json

NOT named `test_*`, so `run_all.py` never collects it: it needs a second tree
checked out at the parent commit, which a suite run does not have.

`--root` is the tree whose ENGINE is censused (default: this file's own repo).
The boards come from that tree too, so both arms read the same bytes as long as
neither commit touched `kicad_files/`. Run it with `-B` so neither tree gains a
`__pycache__` the other would then import.

What each board gets (every value in the JSON is the grader's own output,
floats written by repr, so equality is bit equality):

  edge@F       `grade_pad_edge_clearance(pcb, F, path)` for F in 0, .25, .55
               and 1000. At 1000 every measured pad is a finding carrying its
               own gap, so that arm is a per-pad census, not a count.
  legality@M   the edge keys of `grade_pad_legality(pcb, .25, exact=False,
               edge_margin=M, pcb_file=path)` for M in None and .55 -- the
               `board_floor_knobs` resolution and the `source` label.
  source-none  a copy of the board with no `source_path`, graded at .55: the
               path where nothing can be read.

Synthetic arms, each a copy of `kicad_files/esp_prog.kicad_pcb` in a temp dir
with ONE sibling or outline change, graded both ways: a `.kicad_dru` edge rule,
an unterminated one, a copper-only rule; a `.kicad_pro` edge floor of NaN, -1,
0 and Infinity, and whole-file `[]` / `"abc"`; an open internal Edge.Cuts line,
a round cutout, and no Edge.Cuts at all; a footprint of pad shapes no tracked
board carries (trapezoid, chamfered roundrect, custom with no primitives,
unequal-axis circle, a 33-degree oval), inside the floor so they also grade;
and a board graded against a DIFFERENT `pcb_file` than it was parsed from. An
exception is recorded as `{raised, message}`, because raising at the same
place IS the identity for those. (The pad-shape and `pcb_file` arms were added
after the phase-1 verifier committed a mutant that dropped three unmeasured
reasons and swapped `pcb_file or source_path`, which scored 180 of 180 equal.)

Temp and root paths are normalised to `<tmp>` / `<root>` so the two arms
compare. `--diff` exits 0 only when every arm is identical, and prints each arm
that is not.
"""
import argparse
import copy
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FLOORS = (0.0, 0.25, 0.55, 1000.0)
EDGE_KEYS = ('pad_edge_conflicts', 'pad_edge_shortfall', 'pad_edge_unmeasured', 'pad_edge')


def _git(root, *args):
    return subprocess.run(['git', '-C', root, *args], capture_output=True, text=True,
                          check=True).stdout


def _import_engine(root):
    for p in (root, os.path.join(root, 'py_router'), os.path.join(root, 'py_placer'),
              os.path.join(root, 'py_tools')):
        sys.path.insert(0, p)
    from kicad_parser import parse_kicad_pcb
    from placement import legality
    here = os.path.normcase(os.path.abspath(legality.__file__))
    if not here.startswith(os.path.normcase(os.path.abspath(root)) + os.sep):
        sys.exit(f'REFUSED: placement.legality came from {here}, not from --root {root}')
    return parse_kicad_pcb, legality


def _call(fn, *args, **kw):
    try:
        return fn(*args, **kw)
    except Exception as exc:  # noqa: BLE001 -- raising is part of what is compared
        return {'raised': type(exc).__name__, 'message': str(exc)}


def _board_arms(parse, legality, path):
    pcb = parse(path)
    arms = {}
    for floor in FLOORS:
        arms[f'edge@{floor}'] = _call(legality.grade_pad_edge_clearance, pcb, floor, path)
    for margin in (None, 0.55):
        out = _call(legality.grade_pad_legality, pcb, 0.25, exact=False,
                    edge_margin=margin, pcb_file=path)
        if 'raised' not in out:
            out = {k: out[k] for k in EDGE_KEYS}
        arms[f'legality@{margin}'] = out
    bare = copy.copy(pcb)
    bare.source_path = None
    arms['source-none'] = _call(legality.grade_pad_edge_clearance, bare, 0.55, None)
    return arms


def _synthetic(parse, legality, root, tmp):
    base = os.path.join(root, 'kicad_files', 'esp_prog.kicad_pcb')
    with open(base, encoding='utf-8') as stream:
        text = stream.read()
    bounds = parse(base).board_info.board_bounds
    cx, cy = (bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2
    close = text.rfind(')')

    def board(name, body=None, siblings=()):
        d = os.path.join(tmp, name)
        os.makedirs(d)
        path = os.path.join(d, 'board.kicad_pcb')
        with open(path, 'w', encoding='utf-8', newline='') as stream:
            stream.write(text if body is None else body)
        for suffix, content in siblings:
            with open(os.path.join(d, 'board' + suffix), 'w', encoding='utf-8') as stream:
                stream.write(content)
        return path

    def pro(value):
        return json.dumps({'board': {'design_settings': {'rules': {
            'min_copper_edge_clearance': value}}}})

    edge_rule = '(version 1)\n(rule "edge floor" (constraint edge_clearance (min 0.75mm)))\n'
    cases = {
        'dru-edge': board('dru-edge', siblings=[('.kicad_dru', edge_rule)]),
        'dru-unterminated': board('dru-unterminated', siblings=[
            ('.kicad_dru', '(version 1) (rule "unterminated')]),
        'dru-copper-only': board('dru-copper', siblings=[
            ('.kicad_dru', '(version 1)\n(rule "cu" (constraint clearance (min 0.25mm)))\n')]),
        'dru-unreadable+pro-nan': board('dru-bad-pro-nan', siblings=[
            ('.kicad_dru', '(version 1) (rule "unterminated'), ('.kicad_pro', pro(float('nan')))]),
        'pro-nan': board('pro-nan', siblings=[('.kicad_pro', pro(float('nan')))]),
        'pro-negative': board('pro-neg', siblings=[('.kicad_pro', pro(-1))]),
        'pro-zero': board('pro-zero', siblings=[('.kicad_pro', pro(0))]),
        'pro-infinity': board('pro-inf', siblings=[('.kicad_pro', pro(float('inf')))]),
        'pro-list': board('pro-list', siblings=[('.kicad_pro', '[]')]),
        'pro-string': board('pro-string', siblings=[('.kicad_pro', '"abc"')]),
        'outline-open-line': board('open-line', body=text[:close] + (
            '(gr_line (start 122 105.1) (end 127 105.1) '
            '(stroke (width 0.05) (type default)) (layer "Edge.Cuts"))\n') + text[close:]),
        'outline-round-cutout': board('cutout', body=text[:close] + (
            f'(gr_circle (center {cx} {cy}) (end {cx + 1} {cy}) '
            '(stroke (width 0.05) (type default)) (fill none) (layer "Edge.Cuts"))\n')
            + text[close:]),
        'outline-none': board('no-outline', body=text.replace(
            '(layer "Edge.Cuts")', '(layer "Dwgs.User")')),
    }
    # Pad shapes no tracked board carries: the grader's unmeasured and
    # approximated branches, placed near the west edge so they also grade.
    exotic = (
        '(footprint "t975:exotic" (layer "F.Cu") (at 114.6 95)\n'
        '  (property "Reference" "X975")\n'
        '  (pad "1" smd trapezoid (at 0 0) (size 0.8 0.8) (rect_delta 0 0.2) (layers "F.Cu"))\n'
        '  (pad "2" smd roundrect (at 0 2) (size 0.8 0.8) (layers "F.Cu") '
        '(roundrect_rratio 0.25) (chamfer_ratio 0.2) (chamfer top_left))\n'
        '  (pad "3" smd custom (at 0 4) (size 0.8 0.8) (layers "F.Cu") '
        '(options (clearance outline) (anchor rect)) (primitives))\n'
        '  (pad "4" smd circle (at 0 6) (size 0.6 1.2) (layers "F.Cu"))\n'
        '  (pad "5" smd oval (at 0 8 33) (size 0.6 1.2) (layers "F.Cu"))\n'
        ')\n')
    cases['pads-exotic'] = board('exotic', body=text[:close] + exotic + text[close:])
    arms = {}
    for name, path in cases.items():
        pcb = parse(path)
        arms[f'{name}:edge@0.55'] = _call(legality.grade_pad_edge_clearance, pcb, 0.55, path)
        out = _call(legality.grade_pad_legality, pcb, 0.25, exact=False, pcb_file=path)
        arms[f'{name}:legality@None'] = (out if 'raised' in out
                                         else {k: out[k] for k in EDGE_KEYS})
    # A board graded against a DIFFERENT file than the one it was parsed from:
    # `pcb_file` must win over `source_path` for every sibling read.
    parsed_from = cases['pro-zero']
    arms['pcb-file-differs:edge@0.55'] = _call(
        legality.grade_pad_edge_clearance, parse(parsed_from), 0.55, cases['dru-edge'])
    out = _call(legality.grade_pad_legality, parse(parsed_from), 0.25, exact=False,
                pcb_file=cases['outline-open-line'])
    arms['pcb-file-differs:legality@None'] = (out if 'raised' in out
                                              else {k: out[k] for k in EDGE_KEYS})
    return arms


def _normalise(doc, root, tmp):
    text = json.dumps(doc, sort_keys=True)
    for prefix, token in ((tmp, '<tmp>'), (root, '<root>')):
        for spelling in {prefix, os.path.normpath(prefix), prefix.replace('\\', '/'),
                         os.path.normpath(prefix).replace('\\', '/')}:
            text = text.replace(json.dumps(spelling)[1:-1], token)
    return json.loads(text)


def census(root, out):
    root = os.path.abspath(root)
    if os.path.normcase(os.path.abspath(out)).startswith(os.path.normcase(root) + os.sep):
        sys.exit(f'REFUSED: --out {out} is inside --root; the census must not dirty the tree it reads')
    if _git(root, 'status', '--porcelain', '--untracked-files=no').strip():
        sys.exit(f'REFUSED: {root} has uncommitted changes; census a commit, not a working copy')
    parse, legality = _import_engine(root)
    boards = sorted(_git(root, 'ls-files', 'kicad_files/*.kicad_pcb').split())
    doc = {'root_sha': _git(root, 'rev-parse', 'HEAD').strip(), 'boards': {}}
    for rel in boards:
        doc['boards'][rel] = _board_arms(parse, legality, os.path.join(root, rel))
        print(f'{rel}: censused', flush=True)
    tmp = tempfile.mkdtemp(prefix='census975_')
    try:
        doc['synthetic'] = _normalise(_synthetic(parse, legality, root, tmp), root, tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    doc['boards'] = _normalise(doc['boards'], root, tmp)
    with open(out, 'w', encoding='utf-8') as stream:
        json.dump(doc, stream, sort_keys=True, indent=1)
    n = sum(len(v) for v in doc['boards'].values()) + len(doc['synthetic'])
    print(f'{len(boards)} boards, {n} arms at {doc["root_sha"][:10]} -> {out}')


def diff(a_path, b_path):
    with open(a_path, encoding='utf-8') as stream:
        a = json.load(stream)
    with open(b_path, encoding='utf-8') as stream:
        b = json.load(stream)
    bad = 0
    total = 0
    for section in ('boards', 'synthetic'):
        left, right = a.get(section, {}), b.get(section, {})
        if set(left) != set(right):
            print(f'{section}: key sets differ: {sorted(set(left) ^ set(right))}')
            bad += 1
        for key in sorted(set(left) & set(right)):
            if section == 'boards':
                for arm in sorted(set(left[key]) | set(right[key])):
                    total += 1
                    if left[key].get(arm) != right[key].get(arm):
                        print(f'DIFFERS {key} {arm}')
                        bad += 1
            else:
                total += 1
                if left[key] != right[key]:
                    print(f'DIFFERS {key}')
                    bad += 1
    print(f'{a.get("root_sha", "?")[:10]} vs {b.get("root_sha", "?")[:10]}: '
          f'{total} arms compared, {bad} differ')
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--root', default=HERE)
    ap.add_argument('--out')
    ap.add_argument('--diff', nargs=2, metavar=('A', 'B'))
    args = ap.parse_args()
    if args.diff:
        return diff(*args.diff)
    if not args.out:
        ap.error('--out or --diff is required')
    census(args.root, args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
