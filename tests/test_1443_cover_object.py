#!/usr/bin/env python3
"""#1443: a body a connector carries over the board -- a card in a socket, a
module on a stacking connector -- DECLARED in the intent (`covers[]`), and a
part's declared height (`heights[]`) and a height-limited keep-out.

The board is written here: a socket J1, a mounting hole H1 42 mm along its
local +x (the standoff), a card body 22 mm wide reaching 44 mm with its
underside 2.5 mm over the board, and under it a part 0.55 mm tall (its IPC
name says so), one 2.7 mm tall, and a part whose height nothing gives. No
footprint names a 3D model, so the heights are the same with KiCad installed
and without.

  1. the rows are refused, naming the field, when a number is missing or
     wrong (nothing is filled in);
  2. the judgement: the low part fits under the card, the tall one and the
     unknown one are refused, the standoff is not judged, a part past the
     card is untouched; a declared height admits the unknown part; the body
     moves and turns with the connector;
  3. the standoff is held as a template on the connector at its length;
  4. the grades: check_floorplan's `cover` rule reports a standoff off its
     length, `cover_unresolved` a connector on the other face; check_assembly
     gates the parts the card does not clear, and without the intent sees
     nothing;
  5. a keep-out's `max_height_mm` admits a part of known height under it
     and binds a taller part and an unknown one;
  6. `cover_facts.py` prints the standoff on the connector's axis;
  7. place_simple carries the standoff to its length from the connector and
     puts no part the card does not clear under it.

    python3 tests/test_1443_cover_object.py
"""
import contextlib
import io
import json
import re
import os
import shutil
import subprocess
import sys
import tempfile
import uuid

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))

RUN_ALL_TIMEOUT = 600
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _u():
    return str(uuid.uuid4())


def _fp(name, ref, x, y, rot, pads, court):
    cx, cy = court
    out = [f'\t(footprint "{name}"\n\t\t(layer "F.Cu")\n\t\t(uuid "{_u()}")\n'
           f'\t\t(at {x} {y} {rot})\n']
    for prop, val, layer in (('Reference', ref, 'F.SilkS'), ('Value', name.split(':')[-1], 'F.Fab')):
        out.append(f'\t\t(property "{prop}" "{val}"\n\t\t\t(at 0 {-cy - 1} {rot})\n'
                   f'\t\t\t(layer "{layer}")\n\t\t\t(uuid "{_u()}")\n\t\t\t(effects\n'
                   '\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t\t(thickness 0.15)\n'
                   '\t\t\t\t)\n\t\t\t)\n\t\t)\n')
    out.append(f'\t\t(fp_rect\n\t\t\t(start {-cx} {-cy})\n\t\t\t(end {cx} {cy})\n'
               '\t\t\t(stroke\n\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n'
               f'\t\t\t(fill no)\n\t\t\t(layer "F.CrtYd")\n\t\t\t(uuid "{_u()}")\n\t\t)\n')
    for num, px, py, sx, sy, kind in pads:
        if kind == 'npth':
            out.append(f'\t\t(pad "" np_thru_hole circle\n\t\t\t(at {px} {py} {rot})\n'
                       f'\t\t\t(size {sx} {sy})\n\t\t\t(drill {sx})\n'
                       f'\t\t\t(layers "*.Cu" "*.Mask")\n\t\t\t(uuid "{_u()}")\n\t\t)\n')
        else:
            out.append(f'\t\t(pad "{num}" smd rect\n\t\t\t(at {px} {py} {rot})\n'
                       f'\t\t\t(size {sx} {sy})\n\t\t\t(layers "F.Cu" "F.Paste" "F.Mask")\n'
                       f'\t\t\t(uuid "{_u()}")\n\t\t)\n')
    out.append('\t)\n')
    return ''.join(out)


def _two(name, ref, x, y):
    return _fp(name, ref, x, y, 0, [('1', -0.5, 0, 0.6, 0.6, 'smd'),
                                    ('2', 0.5, 0, 0.6, 0.6, 'smd')], (1.0, 0.6))


def board(path, h1=(52.0, 25.0), j1_rot=0, c2=(30, 30), u9=(40, 25), tp=None):
    """The board: socket J1 at (10, 25), standoff H1, C1 (0.55 mm), C2
    (2.7 mm), U9 (unknown) under the card, R5 past it."""
    socket = [(str(i + 1), 0.75 if i % 2 else -0.75, -4.5 + (i // 2) * 1.0, 0.3, 1.0, 'smd')
              for i in range(20)]
    parts = [
        _fp('Test:Socket_2x10', 'J1', 10, 25, j1_rot, socket, (2.0, 6.0)),
        _fp('MountingHole:MountingHole_3.2mm_M3', 'H1', h1[0], h1[1], 0,
            [('', 0, 0, 3.2, 3.2, 'npth')], (3.45, 3.45)),
        _two('Test:CAPC1005X55N', 'C1', 30, 20),
        _two('Test:CAPC3225X270N', 'C2', *c2),
        _fp('Test:Box_4x4', 'U9', u9[0], u9[1], 0,
            [(str(i + 1), -1.5 + i, 1.5, 0.4, 0.8, 'smd') for i in range(4)], (2.0, 2.0)),
        _two('Test:CAPC1005X55N', 'R5', 65, 25),
    ]
    if tp:
        parts.append(_fp('TestPoint:TestPoint_Pad_1.0x1.0mm', 'TP1', tp[0], tp[1], 0,
                         [('1', 0, 0, 1.0, 1.0, 'smd')], (1.5, 1.5)))
    text = ('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
            '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n\t)\n'
            '\t(paper "A4")\n\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
            '\t\t(13 "F.Paste" user)\n\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n'
            '\t\t(5 "F.SilkS" user "F.Silkscreen")\n\t\t(25 "Edge.Cuts" user)\n'
            '\t\t(31 "F.CrtYd" user "F.Courtyard")\n\t\t(35 "F.Fab" user)\n\t)\n'
            '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n'
            '\t(net 0 "")\n'
            + ''.join(parts)
            + '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 80 50)\n\t\t(stroke\n'
              '\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n\t\t(fill no)\n'
              f'\t\t(layer "Edge.Cuts")\n\t\t(uuid "{_u()}")\n\t)\n)\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


COVER = {'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 22,
         'underside_mm': 2.5, 'length_mm': 44,
         'standoffs': [{'ref': 'H1', 'length_mm': 42}], 'removable': True,
         'source': 'socket datasheet: 2.5 mm card height; single-sided card'}


def intent(**extra):
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'covers': [COVER]}
    doc.update(extra)
    return doc


def refusals():
    from placement.floorplan import IntentError, intent_from_dict
    print('1. rows are refused, naming the field')
    for label, row, want in (
            ('no source', dict(COVER, source=''), 'source'),
            ('no underside', {k: v for k, v in COVER.items() if k != 'underside_mm'},
             'underside_mm'),
            ('no side', {k: v for k, v in COVER.items() if k != 'side'}, 'side'),
            ('nothing saying whether it comes off',
             {k: v for k, v in COVER.items() if k != 'removable'}, 'removable'),
            ('removable not a yes/no', dict(COVER, removable='yes'), 'removable'),
            ('an axis off a quarter turn', dict(COVER, axis_deg=45), 'axis_deg'),
            ('a standoff with no length', dict(COVER, standoffs=[{'ref': 'H1'}]), 'length_mm'),
            ('the connector as its own standoff',
             dict(COVER, standoffs=[{'ref': 'J1', 'length_mm': 42}]), 'standoffs[0].ref'),
            ('nothing saying how far it reaches',
             {k: v for k, v in dict(COVER, standoffs=[]).items() if k != 'length_mm'},
             'length_mm')):
        try:
            intent_from_dict(intent(covers=[row]))
            check(f'refused: {label}', False, 'loaded')
        except IntentError as ex:
            check(f'refused: {label}', want in str(ex), str(ex)[:120])
    for label, rows, want in (
            ('a heights row saying nothing', [{'ref': 'U9', 'source': 'x'}], 'neither'),
            ('a heights row with no source', [{'ref': 'U9', 'height_mm': 1}], 'source'),
            ('a part declared twice', [{'ref': 'U9', 'height_mm': 1, 'source': 'a'},
                                       {'ref': 'U9', 'height_mm': 2, 'source': 'b'}], 'repeats')):
        try:
            intent_from_dict(intent(heights=rows))
            check(f'refused: {label}', False, 'loaded')
        except IntentError as ex:
            check(f'refused: {label}', want in str(ex), str(ex)[:120])
    try:
        intent_from_dict(intent(keepouts=[{'rect': [0, 0, 5, 5], 'max_height_mm': -1}]))
        check('refused: a negative max_height_mm', False, 'loaded')
    except IntentError as ex:
        check('refused: a negative max_height_mm', 'max_height_mm' in str(ex), str(ex)[:120])


def judgement(tmp):
    from kicad_parser import parse_kicad_pcb
    from placement import heights
    from placement.body import board_bodies
    from placement.floorplan import intent_from_dict
    print('2. the judgement under the declared body')
    path = board(os.path.join(tmp, 'b.kicad_pcb'))
    pcb = quiet(parse_kicad_pcb, path)
    it = intent_from_dict(intent())
    h = quiet(heights.for_board, pcb, path, it.height_declarations())
    geo = quiet(board_bodies, pcb, path)

    def pair(a, b, pa=None):
        fa, fb = pcb.footprints[a], pcb.footprints[b]
        return h.pair(a, pa or (fa.x, fa.y, fa.rotation or 0), geo[a].occupancy_local, 'F',
                      b, (fb.x, fb.y, fb.rotation or 0), geo[b].occupancy_local, 'F')
    check('J1 carries a declared body', h.covers.get('J1') is not None
          and h.covers['J1'].decl is not None, h.note)
    check('the 0.55 mm part fits under the 2.5 mm underside', pair('J1', 'C1') == 0.0,
          str(pair('J1', 'C1')))
    check('the 2.7 mm part is refused', (pair('J1', 'C2') or 0) > 0, str(pair('J1', 'C2')))
    check('the part of unknown height is refused: nothing is risked on a guess',
          (pair('J1', 'U9') or 0) > 0, str(pair('J1', 'U9')))
    check('the standoff is not judged', not pair('J1', 'H1'), str(pair('J1', 'H1')))
    check('a part past the card is untouched', not pair('J1', 'R5'), str(pair('J1', 'R5')))
    check('the pair reads the same either way round',
          pair('C2', 'J1') == pair('J1', 'C2'), f"{pair('C2', 'J1')}")
    # the body moves and turns with the connector: J1 turned 180 about its
    # origin carries the card to -x, off every part
    check('turned 180, the card leaves C2', not pair('J1', 'C2', (10, 25, 180)))
    check('moved 20 mm in +x, the card still covers C2',
          (pair('J1', 'C2', (30, 25, 0)) or 0) > 0)
    h2 = quiet(heights.for_board, pcb, path, intent_from_dict(intent(heights=[
        {'ref': 'U9', 'height_mm': 0.8, 'source': 'U9 datasheet: 0.8 mm max'}]))
        .height_declarations())
    fa, fb = pcb.footprints['J1'], pcb.footprints['U9']
    v = h2.pair('J1', (fa.x, fa.y, 0), geo['J1'].occupancy_local, 'F',
                'U9', (fb.x, fb.y, 0), geo['U9'].occupancy_local, 'F')
    check('a declared 0.8 mm height admits the part', v == 0.0, f'{v} {h2.note}')
    check('...and the run names what was declared', 'declared: 1 cover(s), 1 height(s)'
          in h2.note, h2.note)
    # a test point under the card: a probe reaches it only with the card out
    tpath = board(os.path.join(tmp, 'tp.kicad_pcb'), tp=(35, 28))
    tpcb = quiet(parse_kicad_pcb, tpath)
    tgeo = quiet(board_bodies, tpcb, tpath)
    pad = [{'ref': 'TP1', 'height_mm': 0.05, 'source': 'a bare test pad'}]

    def tp_pair(row):
        th = quiet(heights.for_board, tpcb, tpath,
                   intent_from_dict(intent(covers=[row], heights=pad)).height_declarations())
        fa, fb = tpcb.footprints['J1'], tpcb.footprints['TP1']
        return th.pair('J1', (fa.x, fa.y, 0), tgeo['J1'].occupancy_local, 'F',
                       'TP1', (fb.x, fb.y, 0), tgeo['TP1'].occupancy_local, 'F')
    check('a test point fits under a REMOVABLE card', tp_pair(COVER) == 0.0,
          str(tp_pair(COVER)))
    check('...and is refused under one that stays',
          (tp_pair(dict(COVER, removable=False)) or 0) > 0,
          str(tp_pair(dict(COVER, removable=False))))
    return path, it


def standoff(it):
    from placement.templates import member_target
    print('3. the standoff is held at its length')
    rows = it.cover_templates()
    check('one template row, anchored on the connector, H1 at 42 mm along +x',
          len(rows) == 1 and rows[0]['anchor'] == 'J1'
          and rows[0]['members'][0]['ref'] == 'H1'
          and rows[0]['members'][0]['at'] == [42.0, 0.0], json.dumps(rows))
    t = member_target((10.0, 25.0, 90.0), rows[0]['members'][0])
    check('turned 90 with the connector, the standoff turns with it (KiCad\'s sense)',
          abs(t[0] - 10.0) < 1e-6 and abs(t[1] - (25.0 - 42.0)) < 1e-6, str(t))


def grades(tmp, path):
    print('4. the grades')
    doc = intent()
    ip = os.path.join(tmp, 'intent.json')
    with open(ip, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    py = [sys.executable, '-X', 'utf8']
    cf = subprocess.run(py + [os.path.join(REPO, 'py_tools', 'check_floorplan.py'), path,
                              '--intent', ip], capture_output=True, text=True, timeout=600)
    check('check_floorplan: the standoff where the cover puts it',
          'covers[0] J1: body 22 x 44mm' in cf.stdout
          and 'H1 42.00mm along' in cf.stdout and '[cover]' not in cf.stdout,
          cf.stdout[-600:] + cf.stderr[-300:])
    moved = board(os.path.join(tmp, 'moved.kicad_pcb'), h1=(48.0, 26.0))
    cf = subprocess.run(py + [os.path.join(REPO, 'py_tools', 'check_floorplan.py'), moved,
                              '--intent', ip], capture_output=True, text=True, timeout=600)
    check('check_floorplan: a standoff off its length is named',
          'cover' in cf.stdout and 'H1, 38.00mm along the axis (declared 42mm)' in cf.stdout,
          cf.stdout[-800:])
    bad = os.path.join(tmp, 'bside.json')
    with open(bad, 'w', encoding='utf-8') as f:
        json.dump(intent(covers=[dict(COVER, side='B')]), f)
    cf = subprocess.run(py + [os.path.join(REPO, 'py_tools', 'check_floorplan.py'), path,
                              '--intent', bad], capture_output=True, text=True, timeout=600)
    check('check_floorplan: a connector on the other face is cover_unresolved',
          'cover_unresolved' in cf.stdout and 'J1 is on F.Cu, the row says B' in cf.stdout,
          cf.stdout[-800:])
    # check_assembly gates a pair a MOVE made (#1103): the run started with
    # C2 and U9 clear of the card
    base = board(os.path.join(tmp, 'base.kicad_pcb'), c2=(65, 10), u9=(65, 40))
    js = os.path.join(tmp, 'ca.json')
    ca = subprocess.run(py + [os.path.join(REPO, 'py_tools', 'check_assembly.py'), path,
                              '--intent', ip, '--json', js, '--baseline', base],
                        capture_output=True, text=True, timeout=600)
    got = json.load(open(js, encoding='utf-8')) if os.path.exists(js) else {}
    hp = {tuple(sorted((p['a'], p['b']))) for p in got.get('height_pairs') or ()}
    check('check_assembly: the parts the card does not clear are height pairs',
          hp == {('C2', 'J1'), ('J1', 'U9')}, f"{sorted(hp)} rc {ca.returncode} "
          + ca.stdout[-400:] + ca.stderr[-300:])
    check('...and gate the board, moved there from clear of it',
          ca.returncode != 0 and 'NOT BUILDABLE' in ca.stdout,
          f'rc {ca.returncode} ' + ca.stdout[-300:])
    js2 = os.path.join(tmp, 'ca2.json')
    ca = subprocess.run(py + [os.path.join(REPO, 'py_tools', 'check_assembly.py'), path,
                              '--json', js2, '--baseline', base],
                        capture_output=True, text=True, timeout=600)
    got = json.load(open(js2, encoding='utf-8')) if os.path.exists(js2) else {}
    check('without the intent nothing declares the card, and nothing is gated',
          not got.get('height_pairs') and ca.returncode == 0, f'rc {ca.returncode}')


def keepout(tmp, path):
    import pose_score
    from kicad_parser import parse_kicad_pcb
    from placement.floorplan import intent_from_dict
    print('5. a height-limited keep-out')
    it = intent_from_dict({'schema': 1, 'kind': 'floorplan-intent', 'keepouts': [
        {'name': 'under_lid', 'rect': [25, 15, 45, 35], 'sides': ['F'], 'max_height_mm': 1.0}]})
    pcb = quiet(parse_kicad_pcb, path)
    st = quiet(pose_score.make_state, pcb, path, keepouts=it.keepouts,
               declared_heights=it.height_declarations())
    bound = {r for r, ks in st.keepouts_for.items() if any(k['name'] == 'under_lid' for k in ks)}
    check('the 0.55 mm parts may stand in it', 'C1' not in bound and 'R5' not in bound,
          str(sorted(bound)))
    check('the 2.7 mm part and the unknown one are bound', {'C2', 'U9'} <= bound,
          str(sorted(bound)))


def facts(path):
    print('6. cover_facts')
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_tools',
                                                                   'cover_facts.py'),
                        path, '--refs', 'J1'], capture_output=True, text=True, timeout=600)
    check('the standoff on the connector\'s axis, 42 mm along',
          r.returncode == 0 and 'standoff? H1' in r.stdout
          and 'axis 0, 42.00mm along, +0.00mm off it' in r.stdout,
          r.stdout[-600:] + r.stderr[-300:])


def brief(tmp):
    """A design brief beside the board declares the cover and a height; the
    emitted intent carries them 1:1, and an intent without them drifts."""
    print('7. the design brief')
    path = board(os.path.join(tmp, 'br.kicad_pcb'))
    with open(os.path.join(tmp, 'br.design-brief.json'), 'w', encoding='utf-8') as f:
        json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm', 'covers': [COVER],
                   'heights': [{'ref': 'U9', 'height_mm': 0.8, 'source': 'U9 datasheet'}]}, f)
    py = [sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_tools', 'check_floorplan.py'), path]
    out = os.path.join(tmp, 'br_intent.json')
    r = subprocess.run(py + ['--emit-intent', out, '--allow-unplaced'],
                       capture_output=True, text=True, timeout=600)
    doc = json.load(open(out, encoding='utf-8')) if os.path.isfile(out) else {}
    cov = doc.get('covers') or [{}]
    check('the emitted intent carries the brief\'s cover and height, reader 15',
          r.returncode == 0 and cov[0].get('connector') == 'J1'
          and cov[0].get('underside_mm') == 2.5 and cov[0].get('removable') is True
          and (doc.get('heights') or [{}])[0].get('ref') == 'U9'
          and doc.get('min_reader', 0) >= 15, (r.stdout + r.stderr)[-600:])
    ip = os.path.join(tmp, 'br_bare.json')
    with open(ip, 'w', encoding='utf-8') as f:
        json.dump({'schema': 1, 'kind': 'floorplan-intent'}, f)
    r = subprocess.run(py + ['--intent', ip], capture_output=True, text=True, timeout=600)
    check('an intent without them reports the brief\'s cover and height as drift',
          'J1: the brief declares this in covers[]' in r.stdout
          and 'U9: the brief declares this in heights[]' in r.stdout, r.stdout[-800:])
    bad = os.path.join(tmp, 'bad.design-brief.json')
    with open(bad, 'w', encoding='utf-8') as f:
        json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                   'covers': [{k: v for k, v in COVER.items() if k != 'removable'}]}, f)
    r = subprocess.run(py + ['--brief', bad, '--emit-intent', os.path.join(tmp, 'x.json'),
                             '--allow-unplaced'], capture_output=True, text=True, timeout=600)
    check('a brief cover row missing a field is refused by the intent\'s own loader',
          r.returncode == 2 and 'removable' in (r.stdout + r.stderr), (r.stdout + r.stderr)[-400:])


def pose(tmp):
    """place_pose --intent judges a hand move by the declared cover: a part
    moved under the card where it does not fit is refused, and --snap finds
    the nearest pose clear of it."""
    print('8. place_pose')
    base = board(os.path.join(tmp, 'pp.kicad_pcb'), c2=(65, 10))
    with open(os.path.join(tmp, 'pp.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rule_severities': {
            'courtyards_overlap': 'error'}}}}, f)
    ip = os.path.join(tmp, 'pp_intent.json')
    with open(ip, 'w', encoding='utf-8') as f:
        json.dump(intent(), f)
    py = [sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_placer', 'place_pose.py'), base]

    def run(out, *extra):
        r = subprocess.run(py + [os.path.join(tmp, out)] + list(extra) + ['set', 'C2', '30', '30'],
                           capture_output=True, text=True, timeout=600)
        return r.returncode, r.stdout + r.stderr
    rc, out = run('pp0.kicad_pcb')
    check('without the intent, C2 under the card is accepted (nothing declared)', rc == 0,
          out[-400:])
    rc, out = run('pp1.kicad_pcb', '--intent', ip)
    check('with it, refused naming the pair', rc == 4 and 'height_pairs new: C2/J1' in out,
          out[-600:])
    rc, out = run('pp2.kicad_pcb', '--intent', ip, '--snap', '--radius', '10')
    m = re.search(r'C2\s+\[[^]]*\] -> \[([-\d.]+), ([-\d.]+),', out)
    check('--snap seats it clear of the card (off y 14..36)',
          rc == 0 and m is not None and not 14.0 < float(m.group(2)) < 36.0, out[-600:])


def placement(tmp):
    from kicad_parser import parse_kicad_pcb
    from placement import heights
    from placement.body import board_bodies
    from placement.floorplan import intent_from_dict
    print('9. place_simple: the standoff is carried, the card judged')
    src = board(os.path.join(tmp, 'p.kicad_pcb'), h1=(70, 45))
    ip, gp = os.path.join(tmp, 'pi.json'), os.path.join(tmp, 'pg.json')
    with open(ip, 'w', encoding='utf-8') as f:
        json.dump(intent(), f)
    with open(gp, 'w', encoding='utf-8') as f:
        json.dump({'schema': 1, 'groups': [{'name': 'all', 'why': 'one group',
                                            'refs': ['J1', 'H1', 'C1', 'C2', 'U9', 'R5']}]}, f)
    out = os.path.join(tmp, 'p_out.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_placer',
                                                                   'place_simple.py'),
                        src, out, '--groups', gp, '--intent', ip, '--seed', '0'],
                       capture_output=True, text=True, timeout=600)
    if not os.path.exists(out):
        check('place_simple placed the board', False, r.stdout[-500:] + r.stderr[-500:])
        return
    pcb = quiet(parse_kicad_pcb, out)
    geo = quiet(board_bodies, pcb, out)
    j = pcb.footprints['J1']
    pose = (j.x, j.y, j.rotation or 0.0)
    hf = pcb.footprints['H1']
    t, w = heights.along_axis(COVER, heights.to_local((hf.x, hf.y, hf.x, hf.y), pose)[:2])
    check('the standoff, started 30 mm away, ends at its length from the connector',
          abs(t - 42) < 0.3 and abs(w) < 0.3, f'{t:.2f} along, {w:+.2f} off')
    rect = heights.cover_rect(COVER)
    h = quiet(heights.for_board, pcb, out, intent_from_dict(intent()).height_declarations())
    bad = []
    for ref in ('C1', 'C2', 'U9', 'R5'):
        f = pcb.footprints[ref]
        p = (f.x, f.y, f.rotation or 0.0)
        v = h.pair('J1', pose, geo['J1'].occupancy_local, 'F', ref, p,
                   geo[ref].occupancy_local, 'F')
        if v:
            bad.append(f'{ref} {v:.2f}mm2')
    check('no part the card does not clear stands under it', not bad, ', '.join(bad))
    check('the run graded clean', 'floorplan: no error' in r.stdout
          and 'assembly: buildable' in r.stdout, r.stdout[-400:])


def main():
    tmp = tempfile.mkdtemp(prefix='t1443c_')
    try:
        refusals()
        path, it = judgement(tmp)
        standoff(it)
        grades(tmp, path)
        keepout(tmp, path)
        facts(path)
        brief(tmp)
        pose(tmp)
        placement(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('PASS' if not failures else f'FAIL ({len(failures)}): ' + '; '.join(failures))
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
