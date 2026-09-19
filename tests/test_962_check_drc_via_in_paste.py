#!/usr/bin/env python3
"""#962 D6: check_drc reports a via in a solder-paste opening that is not filled+capped.

The paste opening prints solder onto the via barrel. Without IPC-4761 Type VII
(filled AND capped) the solder wicks into it. KiCad has no such check (probed
on 10.0.0: no finding at any severity), so nothing reported it at all.

Invariants, on synthetic boards (one SMD pad with an F.Paste opening, net /A):
1. An unprotected same-net via in the opening is a `via-in-paste` violation:
   - CLI exit 1;
   - the row carries `penetration_mm`, `owner_ref`, the opening and the
     resolved capping/filling;
   - its JSON item has `short` false.
2. Protection is resolved TOKEN BY TOKEN: the via's own spec, then the board
   setup, then KiCad's factory value.
   - The via's own `(capping yes) (filling yes)` -> accepted
     `protected-via-in-paste`, exit 0.
   - A setup declaring filled+capped, plus a via whose own spec is only
     `(tenting ...)` -> accepted too; the tenting override must not hide the
     inherited capping and filling.
   - Only ONE of the two (capping without filling) is still a violation.
   - A legacy `(tenting front back)` setup declares neither, so it is a
     violation.
3. The barrel boundary: a via whose barrel clears the opening by 0.01 mm is not
   a hit; one that reaches 0.01 mm into it is.
4. Not every via near paste is this class:
   - a via in an F.Cu-only pad (no paste layer) is not a hit;
   - a FOREIGN-net via in the opening is not a via-in-paste row (it is a
     short and is reported as one).
5. `--baseline`: a via the baseline board already had under solder,
   unprotected, is accepted `inherited-via-in-paste`, exit 0 (matched by net
   NAME across dialects); one it had outside any opening, or filled+capped,
   is not. A NEW via at another spot still counts. A `--nets` filter excluding
   the via's net drops it. Buried vias and wrong-side blind vias are not hits.
   On a pre-KiCad-10 file (which cannot carry the per-via tokens) the via is
   accepted `undeclarable-via-in-paste` and counted on the console line and in
   the JSON (user decision).
6. kicad_drc_compare keeps these off the copper match:
   - `via_in_paste` is its own channel;
   - the copper `check_drc` count excludes them;
   - a protected via is not counted as an intentional EDGE accept.
   (kicad-cli is stubbed to "no items", so this runs without KiCad.)

Run:
    python3 tests/test_962_check_drc_via_in_paste.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests', os.path.join('tests', 'stress')):
    sys.path.insert(0, os.path.join(ROOT, _p))

from check_drc import run_drc  # noqa: E402
from run_utils import check as run_check  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def board(work, name, vias, setup='', extra='', a=1,
          pad1_layers='"F.Cu" "F.Paste" "F.Mask"', kicad10=True):
    """A 30x20 board: U1 pad 1 (1x1 mm at 10,10) opens F.Paste on /A; U1 pad 2
    (1x1 mm at 20,10) is F.Cu only, on /A too.

    `kicad10` picks the file format. A KiCad 10 file (20260206, name-only nets)
    can DECLARE per-via capping/filling; an older one (20241229, numbered
    nets) cannot, so there every unprotected via in paste is accepted
    `undeclarable-via-in-paste`. `a` is /A's net NUMBER in the old dialect (/B
    takes the other), so a baseline can number its nets differently."""
    b = 3 - a
    num = {'/A': a, '/B': b}

    def net(n):
        return '(net "%s")' % n if kicad10 else '(net %d "%s")' % (num[n], n)
    body = '\n'.join(vias)
    for n in ('/A', '/B'):
        body = body.replace('<<NET:%s>>' % n,
                            '(net "%s")' % n if kicad10 else '(net %d)' % num[n])
    nets = '' if kicad10 else ' (net 0 "") (net %d "%s") (net %d "%s")\n' % (
        min(a, b), '/A' if a < b else '/B', max(a, b), '/B' if a < b else '/A')
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(
            '(kicad_pcb (version %s) (generator "pcbnew")\n'
            ' (general (thickness 1.6)) (paper "A4")\n'
            ' (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) '
            '(31 "B.Cu" signal) (35 "F.Paste" user) '
            '(39 "F.Mask" user) (44 "Edge.Cuts" user))\n'
            ' (setup %s)\n%s'
            ' (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type solid)) '
            '(fill none) (layer "Edge.Cuts"))\n'
            ' (footprint "L:P" (layer "F.Cu") (at 15 10)\n'
            '  (property "Reference" "U1" (at 0 -3 0) (layer "F.SilkS") '
            '(effects (font (size 1 1) (thickness 0.15))))\n'
            '  (pad "1" smd rect (at -5 0) (size 1 1) (layers %s) %s)\n'
            '  (pad "2" smd rect (at 5 0) (size 1 1) (layers "F.Cu" "F.Mask") %s))\n'
            '%s\n%s\n)\n' % ('20260206' if kicad10 else '20241229', setup, nets,
                             pad1_layers, net('/A'), net('/A'), body, extra))
    return p


def via(x, y, net='/A', spec='', uid='v', layers='"F.Cu" "B.Cu"'):
    return (' (via (at %s %s) (size 0.4) (drill 0.2) (layers %s) %s'
            '<<NET:%s>> (uuid "%s"))' % (x, y, layers, spec, net, uid))


def drc(path, **kw):
    kw.setdefault('clearance', 0.1)
    kw.setdefault('clearance_margin', 0.0)
    kw.setdefault('check_sizes', False)
    return run_drc(path, quiet=True, print_summary=False, **kw)


def vip(rows, accepted=None):
    return [r for r in rows if r.get('type') == 'via-in-paste'
            and r.get('accepted') == accepted]


def main():
    work = tempfile.mkdtemp(prefix='krt962d_')
    try:
        # 1 -- an unprotected via in the opening
        b1 = board(work, 'unprot', [via(10, 10)])
        v = vip(drc(b1))
        check('1. an unprotected same-net via in the opening: ONE via-in-paste violation',
              len(v) == 1, str(v))
        if v:
            r = v[0]
            # penetration is barrel radius - distance to the opening, so a
            # centre inside the opening reads the full radius (0.2)
            check('1. ... carries penetration_mm, owner_ref, the opening and the protection',
                  abs(r['penetration_mm'] - 0.2) <= 1e-6 and r['owner_ref'] == 'U1'
                  and 'U1.1 F.Paste (pad)' == r['item2']
                  and (r['capping'], r['filling']) == ('no', 'no'), str(r))
        js = os.path.join(work, 'unprot.json')
        run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                   b1, '--clearance', '0.1', '--clearance-margin', '0', '--no-size-checks',
                   '--json', js], refuse='VIA-IN-PASTE', code=1)
        items = [i for i in json.load(open(js, encoding='utf-8'))['items']
                 if i['type'] == 'via-in-paste']
        check('1. CLI: exit 1, and the JSON item is not a short',
              len(items) == 1 and items[0]['short'] is False, str(items))

        # 2 -- protection, token by token
        b2 = board(work, 'own', [via(10, 10, spec='(capping yes) (filling yes) ')])
        rows = drc(b2)
        check('2. own (capping yes) (filling yes): accepted protected-via-in-paste, no violation',
              len(vip(rows, 'protected-via-in-paste')) == 1 and not vip(rows), str(rows))
        run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                   b2, '--clearance', '0.1', '--clearance-margin', '0', '--no-size-checks'],
                  accept=True)
        check('2. ... and the CLI exits 0 (an accepted row is not a failure)', True)
        b3 = board(work, 'setup', [via(10, 10, spec='(tenting (front no) (back no)) ')],
                   setup='(capping yes) (filling yes)')
        rows = drc(b3)
        check('2. setup filled+capped + a tenting-only own spec: accepted (per-token)',
              len(vip(rows, 'protected-via-in-paste')) == 1 and not vip(rows), str(rows))
        b4 = board(work, 'half', [via(10, 10, spec='(capping yes) ')])
        check('2. capping without filling: still a violation',
              len(vip(drc(b4))) == 1)
        b5 = board(work, 'legacy', [via(10, 10)], setup='(tenting front back)')
        check('2. a legacy (tenting front back) setup declares neither: a violation',
              len(vip(drc(b5))) == 1)

        # 3 -- the barrel boundary (opening edge x = 10.5, barrel radius 0.2)
        rows = drc(board(work, 'clear', [via(10.71, 10)]))
        check('3. barrel 0.01 mm clear of the opening: not a hit', not vip(rows),
              str(vip(rows)))
        rows = drc(board(work, 'graze', [via(10.69, 10)]))
        v = vip(rows)
        check('3. barrel 0.01 mm into the opening: a hit, penetration 0.01',
              len(v) == 1 and abs(v[0]['penetration_mm'] - 0.01) <= 0.002, str(v))

        # 4 -- not this class
        rows = drc(board(work, 'nopaste', [via(20, 10)]))
        check('4. a via in an F.Cu-only pad (no paste layer): not a hit',
              not vip(rows) and not vip(rows, 'protected-via-in-paste'), str(vip(rows)))
        rows = drc(board(work, 'foreign', [via(10, 10, net='/B')]))
        check('4. a FOREIGN-net via in the opening: no via-in-paste row, but reported',
              not vip(rows) and any(r.get('type') == 'pad-via' for r in rows),
              str([r.get('type') for r in rows]))

        # 5 -- --baseline, and --nets
        base = board(work, 'base', [via(10, 10, uid='v1')])
        cur = board(work, 'cur', [via(10, 10, uid='v1b'), via(10.3, 9.7, uid='v2')])
        rows = drc(cur, baseline=base)
        check('5. --baseline: the via the baseline had is accepted inherited-via-in-paste',
              len(vip(rows, 'inherited-via-in-paste')) == 1, str(rows))
        check('5. ... and a NEW via elsewhere in the opening still counts',
              len(vip(rows)) == 1 and abs(vip(rows)[0]['via_loc'][0] - 10.3) < 1e-6,
              str(vip(rows)))
        run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                   base, '--clearance', '0.1', '--clearance-margin', '0', '--no-size-checks',
                   '--baseline', base], accept=True)
        check('5. CLI: a board graded against itself as --baseline exits 0', True)
        rows = drc(b1, net_patterns=['/B'])
        check('5. a --nets filter that excludes the via\'s net drops it', not vip(rows))
        # inheritance is by net NAME: a baseline that numbers /A as net 2
        # an OLDER-format baseline numbering /A as net 2: names, not ids
        base_renum = board(work, 'base_renum', [via(10, 10, uid='v1')], a=2,
                           kicad10=False)
        rows = drc(cur, baseline=base_renum)
        check('5. a baseline in the other net dialect, numbering its nets '
              'differently, still inherits (by NAME)',
              len(vip(rows, 'inherited-via-in-paste')) == 1, str(rows))
        # a pre-KiCad-10 file cannot declare per-via capping/filling at all
        old = board(work, 'old_format', [via(10, 10)], kicad10=False)
        rows = drc(old)
        check('5. a pre-KiCad-10 board: an unprotected via in paste is ACCEPTED '
              'undeclarable-via-in-paste (user decision), not a violation',
              not vip(rows) and len(vip(rows, 'undeclarable-via-in-paste')) == 1
              and vip(rows, 'undeclarable-via-in-paste')[0]['format_can_declare'] is False,
              str(rows))
        jo = os.path.join(work, 'old.json')
        r = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                       old, '--clearance', '0.1', '--clearance-margin', '0', '--no-size-checks',
                       '--json', jo], accept=True)
        vj = json.load(open(jo, encoding='utf-8')).get('via_in_paste') or {}
        check('5. ... CLI exits 0, the console line and the JSON count it',
              '1 undeclarable in this file format' in r.stdout
              and vj.get('undeclarable') == 1 and vj.get('violations') == 0, str(vj))
        # ...and board_score reads that count off the same console line
        import importlib.util as _ilu
        _sp = _ilu.spec_from_file_location('bs962d', os.path.join(
            ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts',
            'board_score.py'))
        _bs = _ilu.module_from_spec(_sp)
        _sp.loader.exec_module(_bs)
        check('5. ... board_score reads the undeclarable count off that line',
              _bs._undeclarable_vias(r.stdout) == 1
              and _bs._undeclarable_vias('NO DRC VIOLATIONS FOUND!') == 0)
        # a missing --baseline is refused up front, not after a full run
        run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                   b1, '--baseline', os.path.join(work, 'no_such_board.kicad_pcb')],
                  refuse='--baseline: no such board file', code=2,
                  allow=('usage:', 'error:'))
        check('5. a --baseline that does not exist: exit 2 with the reason', True)
        # a LIVE board (no file version) asks the running pcbnew for the setters
        import types
        import fab_notes as _fn
        from kicad_parser import parse_kicad_pcb as _pk
        live = _pk(b1)
        live.kicad_version = 0
        saved = sys.modules.get('pcbnew')
        try:
            for label, attrs, want in (('with the capping setter', ('SetPrimaryDrillCappedFlag',), True),
                                       ('without it (KiCad 9)', (), False)):
                fake = types.ModuleType('pcbnew')
                fake.PCB_VIA = type('PCB_VIA', (), {a: (lambda *x: None) for a in attrs})
                sys.modules['pcbnew'] = fake
                check('5. a live board, running pcbnew %s: can declare = %s' % (label, want),
                      _fn._format_can_declare(live) is want)
            fake = types.ModuleType('pcbnew')
            sys.modules['pcbnew'] = fake
            check('5. ...a pcbnew with no PCB_VIA at all does not crash (cannot declare)',
                  _fn._format_can_declare(live) is False)
        finally:
            if saved is None:
                sys.modules.pop('pcbnew', None)
            else:
                sys.modules['pcbnew'] = saved
        # ...and only a via that was ALREADY under solder, unprotected, there
        base_nopaste = board(work, 'base_nopaste', [via(10, 10, uid='v1')],
                             pad1_layers='"F.Cu" "F.Mask"')
        rows = drc(b1, baseline=base_nopaste)
        check('5. a via the baseline had OUT of any opening (a part moved solder onto '
              'it) is NOT inherited', len(vip(rows)) == 1
              and not vip(rows, 'inherited-via-in-paste'), str(rows))
        base_prot = board(work, 'base_prot', [via(10, 10, spec='(capping yes) (filling yes) ',
                                                   uid='v1')])
        rows = drc(b1, baseline=base_prot)
        check('5. a via the baseline had FILLED+CAPPED, shipped bare, is NOT inherited '
              '(the run lost the protection)', len(vip(rows)) == 1
              and not vip(rows, 'inherited-via-in-paste'), str(rows))
        # a via only counts under F.Paste when its barrel reaches F.Cu
        rows = drc(board(work, 'buried', [via(10, 10, layers='"In1.Cu" "In2.Cu"')]))
        check('5. a BURIED In1-In2 via under the F.Paste opening is not a hit',
              not vip(rows) and not vip(rows, 'protected-via-in-paste'), str(vip(rows)))
        rows = drc(board(work, 'blind', [via(10, 10, layers='"F.Cu" "In1.Cu"')]))
        check('5. ...a BLIND F.Cu-In1 one is (its barrel opens under the paste)',
              len(vip(rows)) == 1, str(vip(rows)))

        # 6 -- kicad_drc_compare: a labelled channel, off the copper match
        import kicad_drc_compare as kdc
        _real = kdc.kicad_items_for
        kdc.kicad_items_for = lambda b, c: ([], None)
        try:
            d1 = kdc.compare_board_data(b1)
            d2 = kdc.compare_board_data(b2)
        finally:
            kdc.kicad_items_for = _real
        check('6. unprotected: via_in_paste channel counts it, copper check_drc does not',
              d1['via_in_paste']['check_drc'] == 1 and d1['check_drc'] == 0
              and d1['checkdrc_only'] == 0 and d1['via_in_paste']['kicad'] is None,
              str({k: d1[k] for k in ('via_in_paste', 'check_drc', 'checkdrc_only')}))
        check('6. protected: counted as protected, NOT as an intentional edge accept',
              d2['via_in_paste']['protected'] == 1 and d2['checkdrc_intentional_edge'] == 0,
              str({k: d2[k] for k in ('via_in_paste', 'checkdrc_intentional_edge')}))
        check('6. the channel is a persisted summary key',
              'via_in_paste' in kdc._SUMMARY_KEYS)

        # 7 -- an ACCEPTED via-in-paste does not make its net DRC-dirty
        # (placement.recovery.dirty_net_ids feeds PRR/NRR; every stamped via
        # would otherwise mark its net)
        from kicad_parser import parse_kicad_pcb
        from placement.recovery import dirty_net_ids
        pb = parse_kicad_pcb(b2)
        jd = os.path.join(work, 'dirty.json')
        with open(jd, 'w', encoding='utf-8') as fh:
            json.dump({'items': [
                {'type': 'via-in-paste', 'net1': '/A', 'accepted': 'protected-via-in-paste'},
                {'type': 'pad-via', 'net1': '/B', 'net2': '/A'}]}, fh)
        ids = {pb.nets[n].name for n in dirty_net_ids(pb, jd) if n in pb.nets}
        jd2 = os.path.join(work, 'dirty2.json')
        with open(jd2, 'w', encoding='utf-8') as fh:
            json.dump({'items': [
                {'type': 'via-in-paste', 'net1': '/A', 'accepted': 'protected-via-in-paste'}]}, fh)
        check('7. dirty_net_ids: an accepted via-in-paste row marks nothing; a real '
              'violation still marks its nets',
              not dirty_net_ids(pb, jd2) and '/A' in ids, str(ids))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
