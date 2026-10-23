#!/usr/bin/env python3
"""strip_copper_only drops teardrop zones and orphaned tuning patterns (#1164).

`tests/stress/strip_copper_only.py` makes the unrouted twin of a routed board.
It dropped tracks, arcs, vias and zone fills, and kept two more kinds of
routing output, each of which sits on the human layout:

* TEARDROP zones, `(zone ... (attr (teardrop (type padvia))))`. KiCad
  generates one where a track meets a pad or via; complex_hierarchy carries
  165 that locate all 68 of its pad-bearing parts. The parser read them as
  ordinary signal-net zones, so `make_unplaced` accepted the stripped board
  (`has_copper=False`) and piled the parts with the teardrops left behind.
* `(generated (type tuning_pattern) ...)`: `base_line`/`origin`/`end` are the
  human track coordinates, and every `members` uuid named a deleted track.

No tracked board carries either, so the fixture is esp_prog (unrouted, KiCad
10) with routing output injected around two GND pads: two segments, an arc, a
via, two teardrop zones, a tuning pattern over the tracks, a group of tracks,
a group holding only the tuning pattern, and three things that must SURVIVE:
a GND pour (its fill goes), a keep-out, and a group that also holds a
footprint.

Checks:
  1. the fixture carries what it claims (the parser flags exactly the two
     teardrops; the pour and the keep-out are not teardrops);
  2. the strip's summary line counts each kind it dropped;
  3. the stripped board is esp_prog + the pour (fill removed) + the keep-out
     + the surviving group, BYTE FOR BYTE -- nothing else touched;
  4. the strip's rules on a KiCad 6/7-spelled text (unquoted `tstamp` ids, a
     group `(id ...)`, a group name with a paren in it, an empty group);
  5. a board stripped of its tracks but NOT its teardrops reads `has_copper`
     (teardrops counted, and the gate message names them), and make_unplaced
     refuses it (exit 3, naming the strip tool) and writes nothing;
  6. make_unplaced accepts the strip's output and the pile carries no
     teardrop and no tuning pattern.

    python3 tests/test_1164_strip_teardrops_tuning.py
"""
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

import run_utils                                           # noqa: E402
from kicad_parser import parse_kicad_pcb                   # noqa: E402
from placement.placement_state import (assess_placement,   # noqa: E402
                                       format_report)
import strip_copper_only                                   # noqa: E402

ESP = run_utils.evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'))
STRIP = os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py')
UNPLACED = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts',
                        'make_unplaced.py')
PY = [sys.executable, '-X', 'utf8']
ANCHOR = '\t(embedded_fonts no)\n)'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f" ({detail})" if detail else ''))
    if not ok:
        failures.append(label)


def _u(tag, n):
    return '%s-0000-4000-8000-%012d' % (tag * 8, n)


def _seg(uid, a, b):
    return ('\t(segment\n\t\t(start %.4f %.4f)\n\t\t(end %.4f %.4f)\n'
            '\t\t(width 0.25)\n\t\t(layer "F.Cu")\n\t\t(net "GND")\n'
            '\t\t(uuid "%s")\n\t)\n' % (a[0], a[1], b[0], b[1], uid))


def _tri(x, y):
    return '(xy %.4f %.4f) (xy %.4f %.4f) (xy %.4f %.4f)' % (
        x, y, x + 1, y + 0.3, x + 1, y - 0.3)


def _teardrop(uid, kind, x, y):
    return ('\t(zone\n\t\t(net "GND")\n\t\t(layer "F.Cu")\n\t\t(uuid "%s")\n'
            '\t\t(name "$teardrop_%s$")\n\t\t(hatch full 0.1)\n'
            '\t\t(priority 30000)\n'
            '\t\t(attr\n\t\t\t(teardrop\n\t\t\t\t(type %s)\n\t\t\t)\n\t\t)\n'
            '\t\t(connect_pads yes\n\t\t\t(clearance 0)\n\t\t)\n'
            '\t\t(min_thickness 0.0254)\n\t\t(filled_areas_thickness no)\n'
            '\t\t(fill yes\n\t\t\t(thermal_gap 0.5)\n'
            '\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n'
            '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n\t\t)\n'
            '\t\t(filled_polygon\n\t\t\t(layer "F.Cu")\n\t\t\t(pts\n'
            '\t\t\t\t%s\n\t\t\t)\n\t\t)\n\t)\n'
            % (uid, 'padvia' if kind == 'padvia' else 'track', kind,
               _tri(x, y), _tri(x, y)))


def _zone_head(uid, layer):
    return '\t(zone\n\t\t(net "GND")\n\t\t(layer "%s")\n\t\t(uuid "%s")\n' % (layer, uid)


POUR_DEF = (_zone_head(_u('d', 1), 'B.Cu')
            + '\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0.5)\n\t\t)\n'
              '\t\t(min_thickness 0.25)\n\t\t(filled_areas_thickness no)\n'
              '\t\t(fill yes\n\t\t\t(thermal_gap 0.5)\n\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n'
              '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 100 100) (xy 140 100) (xy 140 120) (xy 100 120)\n'
              '\t\t\t)\n\t\t)\n')
POUR_FILL = ('\t\t(filled_polygon\n\t\t\t(layer "B.Cu")\n\t\t\t(pts\n'
             '\t\t\t\t(xy 101 101) (xy 139 101) (xy 139 119) (xy 101 119)\n'
             '\t\t\t)\n\t\t)\n')
KEEPOUT = ('\t(zone\n\t\t(layers "F.Cu" "B.Cu")\n\t\t(uuid "%s")\n'
           '\t\t(hatch edge 0.508)\n\t\t(connect_pads\n\t\t\t(clearance 0)\n\t\t)\n'
           '\t\t(min_thickness 0.254)\n\t\t(keepout\n\t\t\t(tracks not_allowed)\n'
           '\t\t\t(vias not_allowed)\n\t\t\t(pads allowed)\n'
           '\t\t\t(copperpour not_allowed)\n\t\t\t(footprints allowed)\n\t\t)\n'
           '\t\t(fill\n\t\t\t(thermal_gap 0.508)\n\t\t\t(thermal_bridge_width 0.508)\n\t\t)\n'
           '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 110 105) (xy 115 105) (xy 115 108) (xy 110 108)\n'
           '\t\t\t)\n\t\t)\n\t)\n' % _u('e', 1))


def _group(name, uid, members):
    return ('\t(group "%s"\n\t\t(uuid "%s")\n\t\t(members %s)\n\t)\n'
            % (name, uid, ' '.join('"%s"' % m for m in members)))


def build(text):
    """(routed text, the text a correct strip must produce)."""
    pcb = parse_kicad_pcb(ESP)
    gnd = [p for f in pcb.footprints.values() for p in f.pads
           if p.net_name == 'GND']
    a = (gnd[0].global_x, gnd[0].global_y)
    b = (gnd[1].global_x, gnd[1].global_y)
    m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    fp_uuid = re.search(r'\n\t\(footprint [^\n]*\n(?:\t\t[^\n]*\n)*?\t\t\(uuid "([^"]+)"\)',
                        text).group(1)
    s1, s2, arc, via = _u('a', 1), _u('a', 2), _u('a', 3), _u('a', 4)
    tune = _u('c', 1)
    routing = (
        _seg(s1, a, m) + _seg(s2, m, b)
        + ('\t(arc\n\t\t(start %.4f %.4f)\n\t\t(mid %.4f %.4f)\n\t\t(end %.4f %.4f)\n'
           '\t\t(width 0.25)\n\t\t(layer "F.Cu")\n\t\t(net "GND")\n\t\t(uuid "%s")\n\t)\n'
           % (b[0], b[1], b[0] + 0.5, b[1] + 0.5, b[0] + 1, b[1], arc))
        + ('\t(via\n\t\t(at %.4f %.4f)\n\t\t(size 0.6)\n\t\t(drill 0.3)\n'
           '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "GND")\n\t\t(uuid "%s")\n\t)\n'
           % (m[0], m[1], via))
        + _teardrop(_u('b', 1), 'padvia', *a)
        + _teardrop(_u('b', 2), 'track_end', *b)
        + ('\t(generated\n\t\t(uuid "%s")\n\t\t(type tuning_pattern)\n'
           '\t\t(name "Tuning Pattern")\n\t\t(layer "F.Cu")\n'
           '\t\t(base_line\n\t\t\t(pts\n\t\t\t\t(xy %.4f %.4f) (xy %.4f %.4f)\n\t\t\t)\n\t\t)\n'
           '\t\t(end\n\t\t\t(xy %.4f %.4f)\n\t\t)\n\t\t(initial_side "left")\n'
           '\t\t(last_status "tuned")\n\t\t(origin\n\t\t\t(xy %.4f %.4f)\n\t\t)\n'
           '\t\t(tuning_mode "single")\n\t\t(members "%s" "%s"\n\t\t\t"%s"\n\t\t)\n\t)\n'
           % (tune, a[0], a[1], b[0], b[1], b[0], b[1], a[0], a[1], s1, s2, arc))
        + _group('tracks', _u('f', 1), [s1, via])
        + _group('', _u('f', 2), [tune]))
    survivor = _group('mixed', _u('f', 3), [s2, fp_uuid])
    routed = text.replace(ANCHOR, routing + POUR_DEF + POUR_FILL + '\t)\n'
                          + KEEPOUT + survivor + ANCHOR)
    want = text.replace(ANCHOR, POUR_DEF + '\t)\n' + KEEPOUT + survivor + ANCHOR)
    return routed, want


def case_kicad6_spelling():
    """The membership rules on unquoted ids, `(id ...)` and a paren in a name."""
    t = ('(kicad_pcb (version 20211014)\n'
         '  (gr_line (start 0 0) (end 1 1) (layer "Edge.Cuts") (width 0.1) (tstamp 1-k))\n'
         '  (segment (start 0 0) (end 1 0) (width 0.25) (layer "F.Cu") (net 1) (tstamp 1-s))\n'
         '  (zone (net 1) (net_name "GND") (layer "F.Cu") (tstamp 1-t) (name "$teardrop_padvia$")'
         ' (priority 30000) (attr (teardrop (type padvia))) (fill yes)'
         ' (polygon (pts (xy 0 0) (xy 1 0) (xy 1 1))))\n'
         '  (zone (net 1) (net_name "GND") (layer "B.Cu") (tstamp 1-z) (name "keep (teardrop me)")'
         ' (fill yes) (polygon (pts (xy 0 0) (xy 1 0) (xy 1 1))))\n'
         '  (group "a (b)" (id 1-g1) (members 1-s 1-t))\n'
         '  (group "outer" (id 1-g2) (members 1-g1))\n'
         '  (group "art" (id 1-g3) (members 1-k 1-s))\n'
         '  (group "empty" (id 1-g4) (members))\n'
         ')\n')
    out, n = strip_copper_only.strip(t)
    check('KiCad 6/7 spelling: counts', n == {'copper': 1, 'fill': 0, 'teardrop': 1,
                                              'generated': 0, 'group': 2}, n)
    gone = ('(attr (teardrop' not in out and 'id 1-g1' not in out
            and 'id 1-g2' not in out)
    check('KiCad 6/7 spelling: the teardrop zone and both all-routing groups go',
          gone, '' if gone else out)
    kept = ('keep (teardrop me)' in out and 'id 1-g3' in out and 'id 1-g4' in out
            and '(gr_line' in out)
    check('KiCad 6/7 spelling: a pour whose NAME says teardrop, a group that '
          'keeps a member, and an empty group all stay', kept, '' if kept else out)


def main():
    with open(ESP, encoding='utf-8') as fh:
        text = fh.read()
    if ANCHOR not in text:
        raise AssertionError('BROKEN FIXTURE: esp_prog no longer ends with ' + repr(ANCHOR))
    routed, want = build(text)

    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, 'routed.kicad_pcb')
        with open(src, 'w', encoding='utf-8', newline='') as fh:
            fh.write(routed)

        # 1. the fixture carries what it claims
        pcb = parse_kicad_pcb(src)
        tear = [z for z in pcb.zones if z.teardrop]
        check('fixture: the parser flags exactly the two teardrop zones',
              sorted(z.uuid for z in tear) == [_u('b', 1), _u('b', 2)],
              [z.uuid for z in tear])
        check('fixture: the pour is a zone and not a teardrop',
              any(z.uuid == _u('d', 1) and not z.teardrop for z in pcb.zones))
        check('control: esp_prog itself has no teardrop zone and no copper',
              not any(z.teardrop for z in parse_kicad_pcb(ESP).zones)
              and not assess_placement(parse_kicad_pcb(ESP), ESP).has_copper)

        # 2-3. the strip
        dst = os.path.join(td, 'stripped.kicad_pcb')
        r = run_utils.check(PY + [STRIP, src, dst], accept=True)
        line = r.stdout.strip().splitlines()[-1]
        check('summary line counts each kind',
              line.startswith('dropped 4 copper form(s) (segment/arc/via), '
                              '1 zone fill(s), 2 teardrop zone(s), '
                              '1 generated form(s) (tuning patterns), 2 group(s);'),
              line)
        with open(run_utils.evidence(dst), encoding='utf-8', newline='') as fh:
            got = fh.read()
        check('stripped board = esp_prog + pour definition + keep-out + the '
              'group that keeps a member, byte for byte', got == want,
              'first difference at char %d' % next(
                  (i for i, (x, y) in enumerate(zip(got, want)) if x != y),
                  min(len(got), len(want))) if got != want else '')
        check('no teardrop zone and no tuning pattern survives',
              not any(z.teardrop for z in parse_kicad_pcb(dst).zones)
              and '(generated' not in got)

        # 4. KiCad 6/7 spelling, in process
        case_kicad6_spelling()

        # 5. tracks gone, teardrops kept: the input the old strip produced
        half = os.path.join(td, 'half.kicad_pcb')
        tracks_only = routed
        for head in ('segment', 'arc', 'via'):
            tracks_only = re.sub(r'\t\(%s\n(?:\t\t[^\n]*\n)*\t\)\n' % head, '',
                                 tracks_only)
        with open(half, 'w', encoding='utf-8', newline='') as fh:
            fh.write(tracks_only)
        hp = parse_kicad_pcb(half)
        st = assess_placement(hp, half)
        check('teardrops alone read as routed copper',
              (st.segments, st.vias, st.teardrops, st.has_copper) == (0, 0, 2, True),
              (st.segments, st.vias, st.teardrops, st.has_copper))
        msg = format_report(st, 'place_x')
        check('the gate names the teardrop zones', '2 teardrop zone(s)' in msg, msg)
        out = os.path.join(td, 'pile_refused.kicad_pcb')
        run_utils.check(PY + [UNPLACED, half, out], refuse='strip_copper_only.py',
                        code=3)
        check('make_unplaced refused the teardropped board and wrote nothing',
              not os.path.exists(out))

        # 6. the strip's output piles cleanly
        pile = os.path.join(td, 'pile.kicad_pcb')
        r = run_utils.check(PY + [UNPLACED, dst, pile], accept=True)
        with open(run_utils.evidence(pile), encoding='utf-8') as fh:
            ptxt = fh.read()
        ok = ('$teardrop_' not in ptxt and '(generated' not in ptxt
              and 'has_copper=False' in r.stdout)
        check('the pile carries no teardrop and no tuning pattern', ok,
              '' if ok else r.stdout[-300:])

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nPASS: teardrops and orphaned tuning patterns are stripped, counted, "
          "and refused when left behind")
    return 0


if __name__ == '__main__':
    sys.exit(main())
