#!/usr/bin/env python3
"""check_drc grades copper inside a keep-out RULE AREA (`keepout`).

The parser reads rule areas into `board_info.keepouts` and the router avoids
them, but check_drc had no pass for them, so a board whose tracks ran through
a user-drawn keep-out read "NO DRC VIOLATIONS FOUND". kicad-cli 10.0.0 did not
report `items_not_allowed` there either, so this grade is check_drc's own.

Invariants, on synthetic two-layer boards with one 10x10 mm rule area at
(10..20, 10..20) written by kicad_writer.generate_rule_area_sexpr:
1. A track through a tracks-not-allowed area on its layer: a `keepout`
   violation naming the area, the net and the layer; CLI exit 1.
2. A track outside it (copper clear of the edge): exit 0.
3. A track whose centreline is outside but whose COPPER reaches in: a hit.
4. A via inside a vias-not-allowed area: a hit, CLI exit 1.
5. A track inside an area that allows tracks (vias forbidden only): exit 0.
6. A track on a layer the area does not cover: exit 0.
7. A track inside the HOLE of a ring area: exit 0.
8. `--nets` excluding the track's net drops it.
9. `--baseline` holding the same track: accepted `inherited-keepout`, exit 0;
   a new track in the area still counts.
10. board_score counts it in `drc`; kicad_drc_compare puts it on its own
    `keepout` channel, off the copper match (kicad-cli stubbed to no items).

Run:
    python3 tests/test_checkdrc_keepout.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_tools', 'tests', os.path.join('tests', 'stress')):
    sys.path.insert(0, os.path.join(ROOT, _p))

from check_drc import run_drc  # noqa: E402
from kicad_writer import generate_rule_area_sexpr  # noqa: E402
from run_utils import check as run_check, evidence  # noqa: E402
from synth import RoutingBoard  # noqa: E402

CHECK_DRC = os.path.join(ROOT, 'py_router', 'check_drc.py')
SQUARE = [(10, 10), (20, 10), (20, 20), (10, 20)]
FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def board(work, name, segs=(), vias=(), layers=('F.Cu',),
          not_allowed=('tracks', 'vias', 'copperpour'), hole=None):
    b = RoutingBoard(w=40, h=30)
    for x1, y1, x2, y2, net, layer in segs:
        b.seg(x1, y1, x2, y2, net, w=0.3, layer=layer)
    for x, y, net in vias:
        b.via(x, y, net)
    area = generate_rule_area_sexpr(list(layers), SQUARE, 'KO1',
                                    not_allowed=not_allowed)
    if hole:
        pts = ' '.join('(xy %s %s)' % p for p in hole)
        area = area.rstrip().rstrip(')').rstrip() + (
            '\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n\t\t)\n\t)' % pts)
    b.items.append(area)
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(b.text())
    return evidence(p, 'synthetic board')


def drc(path, **kw):
    kw.setdefault('clearance', 0.2)
    kw.setdefault('check_sizes', False)
    return run_drc(path, quiet=True, print_summary=False, **kw)


def ko(rows, accepted=None):
    return [r for r in rows if r.get('type') == 'keepout'
            and r.get('accepted') == accepted]


def cli(path, *extra, **kw):
    return run_check([sys.executable, '-X', 'utf8', CHECK_DRC, path,
                      '--clearance', '0.2', '--no-size-checks', *extra], **kw)


def main():
    work = tempfile.mkdtemp(prefix='krt_keepout_')
    try:
        # the parser must see the area at all, or every "pass" below is vacuous
        from kicad_parser import parse_kicad_pcb
        b1 = board(work, 'through', segs=[(5, 15, 25, 15, '/SIG', 'F.Cu')])
        kos = parse_kicad_pcb(b1).board_info.keepouts
        check('0. the synthetic rule area parses (one area, KO1, tracks forbidden)',
              len(kos) == 1 and kos[0]['name'] == 'KO1'
              and not kos[0]['tracks_allowed'], str(kos))

        # 1
        rows = ko(drc(b1))
        check('1. a track through a tracks-not-allowed area: ONE keepout row',
              len(rows) == 1, str(rows))
        if rows:
            r = rows[0]
            check('1. ...naming the area, the net, the layer and a point inside',
                  r['area'] == 'KO1' and r['net1'] == '/SIG' and r['layer'] == 'F.Cu'
                  and r['item'] == 'track' and 10 <= r['loc'][0] <= 20
                  and abs(r['inside_mm'] - 10.0) < 1e-6, str(r))
        out = cli(b1, refuse="keep-out rule area 'KO1'", code=1).stdout
        check('1. ...the CLI exits 1 and lists it under KEEPOUT',
              'KEEPOUT violations (1)' in out and 'Track:/SIG' in out, out[-600:])
        jo = os.path.join(work, 'through.json')
        cli(b1, '--json', jo, code=1)
        doc = json.load(open(evidence(jo, 'check_drc JSON'), encoding='utf-8'))
        check('1. ...and the JSON counts it (by_type and the keepout block)',
              doc['by_type'].get('keepout') == 1
              and doc['keepout'] == {'violations': 1, 'inherited': 0}, str(doc.get('keepout')))

        # 2
        b2 = board(work, 'outside', segs=[(5, 5, 25, 5, '/SIG', 'F.Cu')])
        check('2. a track outside the area: no keepout row', not ko(drc(b2)))
        cli(b2, accept=True)
        check('2. ...CLI exit 0', True)

        # 3 -- centreline 0.1 mm outside, half width 0.15: copper 0.05 mm in
        b3 = board(work, 'graze', segs=[(5, 9.9, 25, 9.9, '/SIG', 'F.Cu')])
        rows = ko(drc(b3))
        check('3. copper reaching 0.05 mm into the area: a hit, intrusion measured',
              len(rows) == 1 and abs(rows[0]['intrusion_mm'] - 0.05) < 1e-4
              and rows[0]['inside_mm'] == 0.0, str(rows))
        b3b = board(work, 'clear', segs=[(5, 9.8, 25, 9.8, '/SIG', 'F.Cu')])
        check('3. ...copper 0.05 mm short of it: no hit', not ko(drc(b3b)))

        # 4
        b4 = board(work, 'via', vias=[(15, 15, '/SIG')])
        rows = ko(drc(b4))
        check('4. a via inside a vias-not-allowed area: a hit',
              len(rows) == 1 and rows[0]['item'] == 'via' and rows[0]['area'] == 'KO1',
              str(rows))
        cli(b4, refuse='Via:/SIG in keep-out rule area', code=1)
        check('4. ...CLI exit 1 with the via named', True)

        # 5
        b5 = board(work, 'allows_tracks', segs=[(5, 15, 25, 15, '/SIG', 'F.Cu')],
                   not_allowed=('vias',))
        check('5. a track in an area that allows tracks: no row', not ko(drc(b5)))
        cli(b5, accept=True)
        rows = ko(drc(board(work, 'allows_tracks_via', vias=[(15, 15, '/SIG')],
                            not_allowed=('vias',))))
        check('5. ...but a via in it still is', len(rows) == 1, str(rows))

        # 6
        b6 = board(work, 'other_layer', segs=[(5, 15, 25, 15, '/SIG', 'B.Cu')])
        check('6. a track on a layer the area does not cover: no row', not ko(drc(b6)))
        cli(b6, accept=True)
        rows = ko(drc(board(work, 'all_layers', segs=[(5, 15, 25, 15, '/SIG', 'B.Cu')],
                            layers=('*.Cu',))))
        check('6. ...the same track under a *.Cu area is a hit', len(rows) == 1, str(rows))

        # 7
        b7 = board(work, 'ring', segs=[(14, 15, 16, 15, '/SIG', 'F.Cu')],
                   hole=[(12, 12), (18, 12), (18, 18), (12, 18)])
        check('7. a track inside the hole of a ring area: no row', not ko(drc(b7)))
        rows = ko(drc(board(work, 'ring_hit', segs=[(11, 15, 16, 15, '/SIG', 'F.Cu')],
                            hole=[(12, 12), (18, 12), (18, 18), (12, 18)])))
        check('7. ...one that reaches into the ring is a hit',
              len(rows) == 1 and abs(rows[0]['inside_mm'] - 1.0) < 1e-6, str(rows))

        # 8
        check('8. --nets excluding the net drops the row',
              not ko(drc(b1, net_patterns=['/OTHER'])))
        check('8. ...and --nets naming it keeps it',
              len(ko(drc(b1, net_patterns=['/SIG']))) == 1)

        # 9
        rows = drc(b1, baseline=b1)
        check('9. --baseline with the same track: accepted inherited-keepout',
              not ko(rows) and len(ko(rows, 'inherited-keepout')) == 1, str(rows))
        cli(b1, '--baseline', b1, accept=True)
        b9 = board(work, 'new_track', segs=[(5, 15, 25, 15, '/SIG', 'F.Cu'),
                                            (15, 5, 15, 25, '/SIG2', 'F.Cu')])
        rows = drc(b9, baseline=b1)
        check('9. ...a NEW track in the area still counts',
              len(ko(rows)) == 1 and ko(rows)[0]['net1'] == '/SIG2'
              and len(ko(rows, 'inherited-keepout')) == 1, str(rows))

        # 10
        import importlib.util as _ilu
        _sp = _ilu.spec_from_file_location('bs_keepout', os.path.join(ROOT, 'py_tools',
                                                                      'board_score.py'))
        _bs = _ilu.module_from_spec(_sp)
        _sp.loader.exec_module(_bs)
        drc_c = _bs.score_drc(ROOT, b1, clearance=0.2)[0]
        check('10. board_score counts it in drc (blocking)',
              drc_c.get('by_type', {}).get('keepout') == 1
              and drc_c.get('count') == 1, str(drc_c))
        import kicad_drc_compare as kdc
        _real = kdc.kicad_items_for
        kdc.kicad_items_for = lambda b, c: ([], None)
        try:
            d = kdc.compare_board_data(b1)
        finally:
            kdc.kicad_items_for = _real
        check('10. kicad_drc_compare: its own keepout channel, not checkdrc_only',
              d['keepout']['check_drc'] == 1 and d['check_drc'] == 0
              and d['checkdrc_only'] == 0 and 'keepout' in kdc._SUMMARY_KEYS,
              str({k: d.get(k) for k in ('keepout', 'check_drc', 'checkdrc_only')}))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
