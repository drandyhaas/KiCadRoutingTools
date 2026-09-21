#!/usr/bin/env python3
"""#962: a via the TOOL put in a pad or paste opening DECLARES Type VII.

Tool-added vias used to emit no protection token, so they inherited the board's
`(setup ...)`. On esp_prog that is `(capping no) (filling no)`, the opposite of
IPC-4761 Type VII, which is what a barrel under solder needs. The rule
(`fab_notes.via_protection_stamps`) stamps `(capping yes) (filling yes)` onto a
via only when ALL of these hold:
- it is in a same-net SMD pad or an associated paste opening;
- this run ADDED it, or a part this run moved put solder on it;
- its spec does not DECIDE capping or filling (one that only tents gets
  Type VII merged in);
- the board does not already declare Type VII.

Invariants:
1. The rule, on a real board (the run-25 esp_prog fixture):
   - a new via in U2's graphic opening and one in a pad are stamped with
     exactly TYPE_VII_STAMP;
   - a via elsewhere gets nothing;
   - a PRE-EXISTING via in a pad is kept, and so is one the tool NUDGED
     (< size/4), and both are listed `unprotected`;
   - a via whose own spec decides capping is kept and listed; a tenting-only
     one gets Type VII merged in, and the file stamper adds only the tokens a
     block lacks;
   - a board declaring filled+capped gets no stamp (the via inherits it);
   - a via laid again on the spot of an input via that carried a spec gets
     that spec BACK (`restored`), rather than shipping without it
     (`--force-reroute` / rip-up lost 5 Type VII specs that way);
   - a board whose FILE FORMAT predates per-via capping/filling (KiCad 9,
     20241229) gets no token -- KiCad 9 cannot open it -- and the via is
     counted `unstampable` and listed;
   - the BGA fanout's CHANNEL escape (in-pad vias centred on the ball) is
     stamped, not only its under-pad escape.
2. The text stamper inserts the tokens into exactly the via named by uuid, and
   the parser reads them back.
3. End to end, route.py on the fixture:
   - every via-in-pad/paste via ships filled+capped;
   - every other via ships with no token;
   - `--json-out` `via_in_pad` agrees, with `unprotected` empty.
4. The stamps survive `--write-fill`'s pcbnew re-save (NOT RUN, not a pass,
   without KiCad).
5. Structure (AST):
   - every via producer reaches the core;
   - route.py stamps AFTER its last via-changing pass (`_late_orphan_sweep659`)
     and create_plane AFTER `_finalize_plane_copper` (source order: the
     esp_prog run exercises neither ordering, since its finalize adds no
     in-pad via);
   - check_join passes each via's `tenting_attrs` to the writer;
   - every `pcbnew.PCB_VIA(` in the GUI plugin is followed by
     `apply_via_protection` in the same function.

Run:
    python3 tests/test_962_type_vii_stamp.py
"""
import ast
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb, Via  # noqa: E402
import fab_notes  # noqa: E402
from fab_notes import (TYPE_VII_STAMP, via_protection_stamps, via_snapshot,  # noqa: E402
                       ship_via_protection_file, effective_via_protection,
                       is_filled_and_capped)
from copy_board import copy_board  # noqa: E402
from run_utils import check as run_check  # noqa: E402

RUN_ALL_TIMEOUT = 1500
FAILS = []
FIX = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def net(p, name):
    return next(n for n, v in p.nets.items() if v.name == name)


def via(x, y, nid, spec=None):
    v = Via(x=x, y=y, size=0.5, drill=0.25, layers=['F.Cu', 'B.Cu'], net_id=nid)
    if spec:
        v.tenting_attrs = dict(spec)
    return v


def main():
    p = parse_kicad_pcb(FIX)
    c1 = net(p, 'Net-(C1-Pad1)')
    u2ap = [a for a in p.paste_apertures if a.owner_ref == 'U2' and a.source == 'graphic'][0]
    ax = (u2ap.bounds[0] + u2ap.bounds[2]) / 2
    ay = (u2ap.bounds[1] + u2ap.bounds[3]) / 2
    pad = next(pd for pd in p.pads_by_net[c1] if not pd.drill and pd.component_ref != 'U2')
    pnet = pad.net_id

    # 1 -- the rule
    in_ap, in_pad, away = via(ax, ay, c1), via(pad.global_x, pad.global_y, pnet), via(1, 1, c1)
    pre = via(pad.global_x, pad.global_y, pnet)
    nudged = via(pad.global_x + 0.05, pad.global_y, pnet)
    specd = via(ax, ay, c1, {'tenting': '(front yes) (back yes)'})
    snap = via_snapshot([via(pad.global_x, pad.global_y, pnet)])
    stamps, rec = via_protection_stamps([in_ap, in_pad, away], [], p)
    stamped = {id(v): s for v, s in stamps}
    check('1. a new via in U2\'s graphic opening is stamped exactly TYPE_VII',
          stamped.get(id(in_ap)) == TYPE_VII_STAMP, str(stamped.get(id(in_ap))))
    check('1. a new via in a same-net SMD pad is stamped exactly TYPE_VII',
          stamped.get(id(in_pad)) == TYPE_VII_STAMP)
    check('1. a via in no pad or opening gets nothing', id(away) not in stamped)
    check('1. the record counts the two sites and two stamps',
          rec['count'] == 2 and rec['stamped'] == 2 and not rec['unprotected'], str(rec))
    stamps, rec = via_protection_stamps([pre, nudged], snap, p)
    check('1. a PRE-EXISTING via in a pad is never stamped, and is disclosed',
          not stamps and len(rec['unprotected']) == 2
          and {u['why'] for u in rec['unprotected']}
          == {'at the spot of an input via, kept as the input had it'},
          str(rec['unprotected']))
    # stripped and laid again: the input via here carried Type VII
    snap_spec = via_snapshot([via(pad.global_x, pad.global_y, pnet, TYPE_VII_STAMP)])
    relaid = via(pad.global_x + 0.1, pad.global_y, pnet)
    stamps, rec = via_protection_stamps([relaid], snap_spec, p)
    check("1. a via laid again on an input via's spot gets that via's own spec back",
          [(id(v), sp) for v, sp in stamps] == [(id(relaid), TYPE_VII_STAMP)]
          and rec['restored'] == 1 and rec['stamped'] == 0 and not rec['unprotected'],
          str(rec))
    # a part moved ONTO an input via (place_fanout_clearance pulls cap pads
    # onto same-net vias by design): the site is this run's, so it is stamped
    onto = via(pad.global_x, pad.global_y, pnet)
    snap_off = [(pnet, pad.global_x, pad.global_y, 0.5, {}, False)]
    stamps, rec = via_protection_stamps([onto], snap_off, p)
    check('1. an input via NOT under solder before, under a pad now: stamped '
          '(site_created)', [(id(v), sp) for v, sp in stamps] == [(id(onto), TYPE_VII_STAMP)]
          and rec['site_created'] == 1 and rec['stamped'] == 1, str(rec))
    snap_on = [(pnet, pad.global_x, pad.global_y, 0.5, {}, True)]
    stamps, rec = via_protection_stamps([onto], snap_on, p)
    check('1. ... and one that was ALREADY under solder keeps what it had',
          not stamps and rec['site_created'] == 0, str(rec))
    snap_p = via_snapshot([via(pad.global_x, pad.global_y, pnet), via(1, 1, c1)], p)
    check('1. via_snapshot(vias, pcb_data) records who was under solder',
          [e[5] for e in snap_p] == [True, False], str(snap_p))
    # the format gate
    import kicad_parser as _kp
    check("1. the format gate is the parser's KiCad 10 threshold",
          fab_notes.PER_VIA_PROTECTION_MIN_VERSION == _kp.KICAD_10_MIN_VERSION)
    p9 = parse_kicad_pcb(FIX)
    p9.kicad_version = 20241229
    stamps, rec = via_protection_stamps([via(ax, ay, c1), via(pad.global_x, pad.global_y, pnet)],
                                        [], p9)
    check('1. a KiCad 9 (20241229) board: NO token written, both counted unstampable '
          'and listed', not stamps and rec['unstampable'] == 2
          and len(rec['unprotected']) == 2
          and all('predates' in u['why'] for u in rec['unprotected']), str(rec))
    # the BGA channel escape (haasoscope U3, the phase-3/4 verifier's repro)
    from bga_fanout import generate_bga_fanout
    hpath = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
    hb = parse_kicad_pcb(hpath)
    hb.kicad_version = 20260206      # as if the file were KiCad 10
    kw = dict(net_filter=['*U2A*DATA*'], primary_escape='horizontal',
              force_escape_direction=True, layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
              track_width=0.1, clearance=0.1, via_size=0.3, via_drill=0.2)
    _t, bvias, _vr, _f = generate_bga_fanout(hb.footprints['U3'], hb, **kw)
    bsites = fab_notes.via_in_pad_sites(bvias, hb.pads_by_net)
    bad = [v for v, _pd in bsites if (v.get('tenting_attrs') or {}) != TYPE_VII_STAMP]
    check('1. BGA fanout: every in-pad via it returns is stamped (channel escape too)',
          len(bsites) >= 10 and not bad, f'{len(bsites)} in-pad, {len(bad)} unstamped')
    h9 = parse_kicad_pcb(hpath)
    _t, bvias9, _vr, _f = generate_bga_fanout(h9.footprints['U3'], h9, **kw)
    check('1. ... and on the same board as its real KiCad 9 file: no via carries a token',
          not any(v.get('tenting_attrs') for v in bvias9))
    stamps, rec = via_protection_stamps([specd], [], p)
    check('1. a via whose own spec only TENTS gets Type VII merged into it '
          '(the spec says nothing about capping or filling)',
          [(id(v), sp) for v, sp in stamps]
          == [(id(specd), dict(specd.tenting_attrs, **TYPE_VII_STAMP))], str(stamps))
    decided = via(ax, ay, c1, {'capping': 'no'})
    stamps, rec = via_protection_stamps([decided], [], p)
    check('1. a via whose own spec DECIDES capping keeps it, and is disclosed',
          not stamps and rec['unprotected'][0]['why'] == 'own spec kept', str(rec))
    # a part moved onto an input via that only tents (all of orangecrab's do)
    tent = via(pad.global_x, pad.global_y, pnet, {'tenting': '(front yes) (back yes)'})
    stamps, rec = via_protection_stamps(
        [tent], [(pnet, pad.global_x, pad.global_y, 0.5, dict(tent.tenting_attrs), False)], p)
    check('1. a site a move created on a tenting-only input via: Type VII merged in',
          [sp for _v, sp in stamps] == [dict(tent.tenting_attrs, **TYPE_VII_STAMP)]
          and rec['site_created'] == 1, str(rec))
    p2 = parse_kicad_pcb(FIX)
    p2.board_info.via_protection_setup = dict(p2.board_info.via_protection_setup,
                                              capping='yes', filling='yes')
    stamps, rec = via_protection_stamps([via(ax, ay, c1)], [], p2)
    check('1. a board declaring filled+capped: no stamp (the via inherits it)',
          not stamps and rec['protected'] == 1)

    work = tempfile.mkdtemp(prefix='krt962t_')
    try:
        # 2 -- the text stamper, by uuid
        from kicad_writer import stamp_via_protection_in_content, generate_via_sexpr
        body = (generate_via_sexpr(ax, ay, 0.5, 0.25, ['F.Cu', 'B.Cu'], c1)
                + generate_via_sexpr(1, 1, 0.5, 0.25, ['F.Cu', 'B.Cu'], c1))
        import re
        uuids = re.findall(r'\(uuid "([^"]+)"\)', body)
        out, n = stamp_via_protection_in_content(body, {uuids[0]: TYPE_VII_STAMP})
        from kicad_parser import _via_blocks, _via_spec_from_block
        specs = [_via_spec_from_block(b) for _s, b in _via_blocks(out)]
        check('2. exactly the named via gets the tokens; the other none',
              n == 1 and specs == [TYPE_VII_STAMP, {}], str(specs))
        tb = generate_via_sexpr(ax, ay, 0.5, 0.25, ['F.Cu', 'B.Cu'], c1,
                                tenting_attrs={'tenting': '(front no) (back no)'})
        tu = re.findall(r'\(uuid "([^"]+)"\)', tb)[0]
        merged = dict({'tenting': '(front no) (back no)'}, **TYPE_VII_STAMP)
        out2, n2 = stamp_via_protection_in_content(tb, {tu: merged})
        spec2 = [_via_spec_from_block(b) for _s, b in _via_blocks(out2)][0]
        check('2. a tenting-only block gains ONLY the missing capping/filling; its '
              'own tenting token is not rewritten or duplicated',
              n2 == 1 and spec2 == merged and out2.count('(tenting') == 1, out2)

        # 3 -- end to end
        src = os.path.join(work, 'in.kicad_pcb')
        copy_board(FIX, src)
        out_b = os.path.join(work, 'out.kicad_pcb')
        js = os.path.join(work, 'out.json')
        run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route.py'),
                   src, '--output', out_b, '--json-out', js], accept=True, timeout=1200)
        q = parse_kicad_pcb(out_b)
        setup = q.board_info.via_protection_setup
        sites = {id(v) for v, _ in fab_notes.via_in_pad_sites(q.vias, q.pads_by_net)}
        sites |= {id(v) for v, _a, _p in fab_notes.via_paste_sites(q.vias, q)}
        bad_site = [(v.x, v.y) for v in q.vias if id(v) in sites
                    and not is_filled_and_capped(effective_via_protection(v.tenting_attrs, setup))]
        bad_other = [(v.x, v.y, v.tenting_attrs) for v in q.vias
                     if id(v) not in sites and v.tenting_attrs]
        check('3. every via-in-pad / via-in-paste ships filled+capped',
              sites and not bad_site, f'{len(sites)} sites, bad {bad_site[:3]}')
        check('3. every other via ships with NO token (inherits the board)',
              not bad_other, str(bad_other[:3]))
        vip = json.load(open(js, encoding='utf-8')).get('via_in_pad') or {}
        check('3. --json-out via_in_pad: count == stamped == sites, nothing unprotected',
              vip.get('count') == vip.get('stamped') == len(sites) and vip.get('unprotected') == [],
              str({k: vip.get(k) for k in ('count', 'stamped', 'unprotected')}))
        # D6 on the shipped board: the stamp and check_drc share
        # fab_notes.via_paste_sites, so a stamped via reads as protected
        from check_drc import run_drc
        rows = run_drc(out_b, clearance=0.1, clearance_margin=0.1, quiet=True,
                       print_summary=False, baseline=src)
        vv = [r for r in rows if r.get('type') == 'via-in-paste' and not r.get('accepted')]
        vp = [r for r in rows if r.get('accepted') == 'protected-via-in-paste']
        check('3. check_drc on the shipped board: no via-in-paste violation, and the '
              'stamped vias in paste read protected-via-in-paste',
              not vv and len(vp) >= 1, f'{len(vv)} violations, {len(vp)} protected')

        # 4 -- --write-fill's pcbnew re-save keeps them
        from kicad_exact_fill import write_filled_board
        filled = os.path.join(work, 'filled.kicad_pcb')
        copy_board(out_b, filled)
        st = write_filled_board(out_b, filled)
        if getattr(st, 'ok', False):
            f = parse_kicad_pcb(filled)
            n_cap = sum(1 for v in f.vias if is_filled_and_capped(
                effective_via_protection(v.tenting_attrs, f.board_info.via_protection_setup)))
            check('4. the stamps survive --write-fill\'s pcbnew re-save',
                  n_cap == len(sites), f'{n_cap} vs {len(sites)}')
        else:
            print('  NOT RUN: 4. write_filled_board unavailable here (%s) -- this is '
                  'not a pass' % getattr(st, 'reason', st))

        # 3c -- route_diff's CLI file stamp, end to end
        out_d = os.path.join(work, 'diff.kicad_pcb')
        run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route_diff.py'),
                   src, out_d, '--nets', '/D_*'], accept=True, timeout=1200)
        qd = parse_kicad_pcb(out_d)
        dn = {n for n, v in qd.nets.items() if v.name.startswith('/D_')}
        dv = [v for v in qd.vias if v.net_id in dn]
        dsites = {id(v): v for v, _pd in fab_notes.via_in_pad_sites(dv, qd.pads_by_net)}
        dsites.update({id(v): v for v, _a, _p in fab_notes.via_paste_sites(dv, qd)})
        check('3. route_diff: its via-in-pad/paste ships filled+capped (file stamp)',
              dsites and all(is_filled_and_capped(effective_via_protection(
                  v.tenting_attrs, qd.board_info.via_protection_setup))
                  for v in dsites.values()), f'{len(dsites)} sites')

        # 3b -- repair_planes' post-pass stamps a written board end to end too
        rec = ship_via_protection_file(out_b, via_snapshot(parse_kicad_pcb(out_b).vias), quiet=True)
        check('3. re-running the file stamp on a shipped board changes nothing '
              '(its vias are now pre-existing and already protected)',
              rec and rec['stamped'] == 0 and not rec['unprotected'], str(rec))
    finally:
        shutil.rmtree(work, ignore_errors=True)

    # 5 -- structure
    producers = {
        'py_router/route.py': '_ship_via_protection962',
        'py_router/route_diff.py': 'via_protection_stamps',
        'py_router/route_planes.py': 'via_protection_stamps',
        'py_router/repair_planes.py': 'ship_via_protection_file',
        'py_router/bga_fanout/__init__.py': 'via_protection_stamps',
        'py_router/qfn_fanout/__init__.py': 'via_protection_stamps',
        'py_tools/check_join.py': 'via_protection_stamps',
        'kicad_routing_plugin/gui_utils.py': 'via_protection_stamps',
    }
    for rel, fn in producers.items():
        tree = ast.parse(open(os.path.join(ROOT, rel), encoding='utf-8').read())
        called = any(isinstance(n, ast.Call) and (
            (isinstance(n.func, ast.Name) and fn in n.func.id)
            or (isinstance(n.func, ast.Attribute) and n.func.attr == fn)
            or (isinstance(n.func, ast.Name) and n.func.id.startswith('_vps962')))
            for n in ast.walk(tree))
        check('5. via producer %s calls %s' % (rel, fn), called)
    # the stamp runs after the last via-changing pass, by source order
    rsrc = open(os.path.join(ROOT, 'py_router', 'route.py'), encoding='utf-8').read()
    # the LAST call of each: the sweep also runs on an early path
    i_sweep = rsrc.rfind('_late_orphan_sweep659(')
    i_ship = rsrc.rfind('_ship_via_protection962(')
    check('5. route.py calls the stamp AFTER its last _late_orphan_sweep659',
          0 < i_sweep < i_ship, f'sweep {i_sweep}, stamp {i_ship}')
    psrc = open(os.path.join(ROOT, 'py_router', 'route_planes.py'), encoding='utf-8').read()
    i_fin = psrc.find('_finalize_plane_copper(', psrc.find('def create_plane'))
    i_vps = psrc.find('_vps962(all_new_vias', psrc.find('def create_plane'))
    check('5. create_plane stamps AFTER _finalize_plane_copper',
          0 < i_fin < i_vps, f'finalize {i_fin}, stamp {i_vps}')
    jtree = ast.parse(open(os.path.join(ROOT, 'py_tools', 'check_join.py'), encoding='utf-8').read())
    jcalls = [n for n in ast.walk(jtree) if isinstance(n, ast.Call)
              and getattr(n.func, 'id', getattr(n.func, 'attr', '')) == 'generate_via_sexpr']
    check('5. check_join passes a real tenting_attrs= to every generate_via_sexpr call',
          jcalls and all(any(k.arg == 'tenting_attrs' and not (
              isinstance(k.value, ast.Constant) and k.value.value is None)
              for k in c.keywords) for c in jcalls), str(len(jcalls)))
    plugin = os.path.join(ROOT, 'kicad_routing_plugin')
    missing = []
    for fn in sorted(os.listdir(plugin)):
        if not fn.endswith('.py'):
            continue
        tree = ast.parse(open(os.path.join(plugin, fn), encoding='utf-8').read())
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            src = ast.get_source_segment(open(os.path.join(plugin, fn), encoding='utf-8').read(), node) or ''
            if 'pcbnew.PCB_VIA(' in src and 'apply_via_protection' not in src:
                missing.append('%s:%s' % (fn, node.name))
            # a via SOURCE the GUI calls directly must be decided too: the
            # planes tab's GND return vias skipped the stamp route_planes
            # --add-gnd-vias applies (phase-3/4 verification)
            # a CALL, not the name: an import line alone satisfied the old
            # substring check with the call deleted (phase-5 verification)
            calls = {getattr(c.func, 'id', getattr(c.func, 'attr', ''))
                     for c in ast.walk(node) if isinstance(c, ast.Call)}
            if ('add_gnd_vias_to_existing_board' in calls
                    and not {'via_protection_stamps', 'apply_stamps_in_memory'} <= calls):
                missing.append('%s:%s (GND vias)' % (fn, node.name))
    check('5. every GUI function that builds a pcbnew.PCB_VIA applies its protection, '
          'and every one that adds GND return vias stamps them',
          not missing, str(missing))

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
