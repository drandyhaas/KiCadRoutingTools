#!/usr/bin/env python3
"""route.py's end-of-run strips reach a pre-KiCad-10 file's name-ref copper (#1312).

The four passes that delete copper from the WRITTEN file after the writer --
the #1063 strict collapse, the #1166 dangling-via sweep, the #659 orphan sweep
and the output reconcile (`file_only_copper`) -- passed
remove_*_from_content the net-name map only when the file carried a KiCad-10
version stamp. remove_*_from_content resolves each block in its own dialect,
and a `(net "/name")` block can match a target only through that map. On a
`(version 20240108)` board whose copper carries name refs -- the mixed dialect
this repo's own chain writes -- every pass matched nothing and printed the
model's count: imx6ulz's strict collapse "removed 291 redundant segment(s)"
and the file kept all 3014 segments (check_weird removable-segment 144 before
and after).

The map now rides whenever the file references nets by name
(`board_uses_name_nets`, the writer's own test), and each pass prints the
count it actually stripped. Pinned here, each pass on a name-ref pre-10 file,
with the numeric file as the control:

  1. the strict collapse removes the buried wiggle from the file;
  2. the dangling-via sweep removes the one-layer via and its spur;
  3. the orphan sweep removes the pad-less island;
  4. the output reconcile's strip (`_strip_written_copper`) on a MIXED file
     removes both a name-ref and a numeric-ref block, and reports 0 for a
     target the file does not hold;
  5. every remove_*_from_content call in route.py goes through
     `_strip_written_copper`, so no new site can gate the map on its own.

    python3 tests/test_1312_late_strip_name_refs.py
"""
import ast
import os
import re
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import RoutingBoard                                  # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from improvement_gate import copper_signature                   # noqa: E402
from run_utils import evidence                                  # noqa: E402

FAILS = []
NET = '/N7'


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def _dialect(text, dialect):
    """`numeric`: as RoutingBoard writes it (KiCad 9). `names`: the same
    board stamped 20240108 with every track/via ref by NAME, the form a
    round-tripped pre-10 board carries. `k10`: names under a KiCad-10 stamp."""
    if dialect == 'numeric':
        return text
    names = dict((int(i), n) for i, n in
                 re.findall(r'\(net (\d+) "([^"]*)"\)', text))

    def by_name(m):
        return '(net "%s")' % names[int(m.group(1))]
    text = re.sub(r'\(net (\d+)\)', by_name, text)
    return text.replace('(version 20241229)',
                        '(version 20240108)' if dialect == 'names'
                        else '(version 20250114)')


def _write(rb, dialect, tmp, tag):
    path = os.path.join(tmp, f'{tag}_{dialect}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(_dialect(rb.text(), dialect))
    return path


def _counts(path):
    pcb = parse_kicad_pcb(evidence(path, 'the stripped board'))
    return len(pcb.segments), len(pcb.vias)


def _two_pads():
    rb = RoutingBoard()
    rb.pad('U1', 5, 15, NET, sx=2.0, sy=2.0)
    rb.pad('U2', 25, 15, NET)
    rb.net('/OTHER')
    return rb


def strict_collapse(tmp):
    print('1. the #1063 strict collapse strips from a name-ref pre-10 file')
    from route import _late_strict_collapse1063
    for dialect in ('numeric', 'names', 'k10'):
        rb = _two_pads()
        rb.seg(5, 15, 25, 15, NET, w=0.2)
        rb.seg(4.5, 14.7, 5.5, 14.7, NET, w=0.2)        # buried in U1's pad
        path = _write(rb, dialect, tmp, 'collapse')
        before = _counts(path)
        pcb = parse_kicad_pcb(path)
        name_of = {nid: n.name for nid, n in pcb.nets.items()}.get
        sig = copper_signature(pcb.segments, pcb.vias, name_of)
        n, v = _late_strict_collapse1063(None, path, False, None, None, sig,
                                         {NET}, False)
        after = _counts(path)
        check(f'{dialect}: the wiggle leaves the file, and the count says so',
              n == 1 and after == (before[0] - 1, before[1]),
              f'returned {n}, segments {before[0]} -> {after[0]}')


def dangling_via(tmp):
    print('2. the #1166 dangling-via sweep strips from a name-ref pre-10 file')
    from route import _late_dangling_via_sweep1166
    for dialect in ('numeric', 'names'):
        rb = _two_pads()
        rb.seg(5, 15, 25, 15, NET, w=0.2)
        rb.seg(15, 15, 15, 20, NET, w=0.2)              # spur to a via
        rb.via(15, 20, NET)                              # reached on F.Cu only
        path = _write(rb, dialect, tmp, 'dvia')
        before = _counts(path)
        n, v = _late_dangling_via_sweep1166(None, path, False, None, None, {},
                                            {NET}, False)
        after = _counts(path)
        check(f'{dialect}: the via and its spur leave the file',
              (n, v) == (1, 1) and after == (before[0] - 1, before[1] - 1),
              f'returned ({n}, {v}), {before} -> {after}')


def orphan_island(tmp):
    print('3. the #659 orphan sweep strips from a name-ref pre-10 file')
    from route import _late_orphan_sweep659
    for dialect in ('numeric', 'names'):
        rb = _two_pads()
        rb.seg(5, 15, 25, 15, NET, w=0.2)
        rb.seg(10, 25, 20, 25, NET, w=0.2)              # reaches no pad
        path = _write(rb, dialect, tmp, 'orphan')
        before = _counts(path)
        _late_orphan_sweep659(None, path, False, None, set(), False, False)
        after = _counts(path)
        check(f'{dialect}: the island leaves the file',
              after == (before[0] - 1, before[1]), f'{before} -> {after}')


def reconcile_strip(tmp):
    print('4. the output reconcile strip on a MIXED-dialect pre-10 file')
    from route import _strip_written_copper
    rb = _two_pads()
    rb.seg(5, 15, 15, 15, NET, w=0.2)
    rb.seg(15, 15, 25, 15, NET, w=0.2)
    path = _write(rb, 'names', tmp, 'mixed')
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    nid = rb.net(NET)
    # The second segment back to a numeric ref: one file, both dialects.
    text = text.replace('(start 15 15) (end 25 15) (width 0.2) (layer "F.Cu") '
                        '(net "%s")' % NET,
                        '(start 15 15) (end 25 15) (width 0.2) (layer "F.Cu") '
                        '(net %d)' % nid)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    pcb = parse_kicad_pcb(path)
    check('the fixture is mixed (one name ref, one numeric ref)',
          text.count('(net "%s")' % NET) == 1 and '(net %d)' % nid in text
          and len(pcb.segments) == 2, f'{len(pcb.segments)} segments')
    name_of = {i: n.name for i, n in pcb.nets.items()}
    ghost = type(pcb.segments[0])(start_x=1, start_y=1, end_x=2, end_y=1,
                                  width=0.2, layer='F.Cu',
                                  net_id=pcb.segments[0].net_id)
    n_s, n_v = _strip_written_copper(path, list(pcb.segments) + [ghost], [],
                                     name_of)
    check('both blocks leave the file; the ghost is not counted',
          (n_s, n_v) == (2, 0) and _counts(path) == (0, 0),
          f'returned ({n_s}, {n_v}), file {_counts(path)}')


def single_strip_site():
    print('5. every content strip in route.py goes through _strip_written_copper')
    src = open(os.path.join(ROOT, 'py_router', 'route.py'), encoding='utf-8').read()
    tree = ast.parse(src)
    helper = [fn for fn in ast.walk(tree) if isinstance(fn, ast.FunctionDef)
              and fn.name == '_strip_written_copper']
    span = (helper[0].lineno, helper[0].end_lineno) if helper else (0, -1)
    imports = [node.lineno for node in ast.walk(tree)
               if isinstance(node, ast.ImportFrom)
               and node.module == 'kicad_writer'
               and any(a.name in ('remove_segments_from_content',
                                  'remove_vias_from_content')
                       for a in node.names)]
    outside = [ln for ln in imports if not span[0] <= ln <= span[1]]
    check('the only importer is _strip_written_copper',
          imports and not outside,
          f'imports at lines {imports}, helper spans {span}')


def main():
    with tempfile.TemporaryDirectory() as tmp:
        strict_collapse(tmp)
        dangling_via(tmp)
        orphan_island(tmp)
        reconcile_strip(tmp)
    single_strip_site()
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
