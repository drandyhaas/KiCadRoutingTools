#!/usr/bin/env python3
"""#1925 piece 1: a via's and a drilled pad's back-drills are read and kept.

KiCad 10 writes `(backdrill (size S) (layers "START" "END"))` and
`(tertiary_drill ...)` inside a via or pad block, between `(drill ...)` and
`(layers ...)`. Nothing read them, so every writer that re-emits an existing
via dropped them, and a back-drilled pad's layer set was read off the
back-drill's own `(layers ...)`.

  1. the text parser reads both tokens on vias and on pads, in KiCad's own
     three-line form and in a one-line form, in either child order;
  2. a back-drilled pad's layer set is its own, not the back-drill's;
  3. a token KiCad loads as no drill (no layers, no size, size 0) is none
     here, and one KiCad refuses to load is named;
  4. `generate_via_sexpr(..., backdrills=...)` writes them back and the
     parser reads the same value; a via with none is written byte for byte
     as before;
  5. the writers that take a via dict (`add_tracks_and_vias_to_pcb`, the
     failed-reroute restore) carry the key;
  6. every emit site carries them (AST): each `generate_via_sexpr` call, each
     via dict that carries `tenting_attrs`, each plugin function that builds
     a PCB_VIA and applies its protection.

The pcbnew half (both parse paths agree, and KiCad loads what the writer
wrote) is tests/gui_parity/test_1925_backdrill_pcbnew_parity.py.

Run with:  python3 tests/test_1925_backdrill_read_keep.py
"""
import ast
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

import run_utils  # noqa: E402

FAILS = []

#: Copper layers top to bottom with KiCad 10's layer ids, and the dielectric
#: under each (mm). Copper is 0.035 mm, so the depths of the copper surfaces
#: are F.Cu 0 / 0.035, In1 0.135 / 0.170, In2 0.370 / 0.405, In3 0.905 /
#: 0.940, In4 1.140 / 1.175, B.Cu 1.275 / 1.310.
COPPER = [('F.Cu', 0, 0.1), ('In1.Cu', 4, 0.2), ('In2.Cu', 6, 0.5),
          ('In3.Cu', 8, 0.2), ('In4.Cu', 10, 0.1), ('B.Cu', 2, None)]
NETS = ['/A', '/B', '/C', '/D', '/E', '/F', '/G', '/H', '/I']

#: One via per net, at x = 10, 20, ... on y = 10: (net, back-drill tokens as
#: KiCad writes them, type word, layer pair, layers its tracks are on).
VIAS = [
    ('/A', '', '', ('F.Cu', 'B.Cu'), ('F.Cu', 'In1.Cu')),
    ('/B', '\t\t(backdrill\n\t\t\t(size 0.5)\n\t\t\t(layers "B.Cu" "In3.Cu")\n\t\t)\n',
     '', ('F.Cu', 'B.Cu'), ('F.Cu', 'In1.Cu')),
    ('/C', '\t\t(backdrill\n\t\t\t(size 0.45)\n\t\t\t(layers "F.Cu" "In1.Cu")\n\t\t)\n'
           '\t\t(tertiary_drill\n\t\t\t(size 0.5)\n\t\t\t(layers "B.Cu" "In4.Cu")\n\t\t)\n',
     '', ('F.Cu', 'B.Cu'), ('In2.Cu', 'In3.Cu')),
    ('/D', '', '', ('F.Cu', 'B.Cu'), ('F.Cu', 'B.Cu')),
    # joined on a layer its own back-drill removes
    ('/E', '\t\t(backdrill (layers "In3.Cu" "B.Cu") (size 0.5))\n',
     '', ('F.Cu', 'B.Cu'), ('In3.Cu', 'B.Cu')),
    # via in an F.Cu pad of its net, leaving on In2.Cu
    ('/F', '', '', ('F.Cu', 'B.Cu'), ('In2.Cu',)),
    # in an In4.Cu pour of its net, with a track on F.Cu
    ('/G', '', '', ('F.Cu', 'B.Cu'), ('F.Cu',)),
    ('/H', '', ' blind', ('F.Cu', 'In2.Cu'), ('F.Cu', 'In1.Cu')),
    ('/I', '', '', ('F.Cu', 'B.Cu'), ()),
]
WANT_VIA = {
    '/B': {'backdrill': (0.5, 'B.Cu', 'In3.Cu')},
    '/C': {'backdrill': (0.45, 'F.Cu', 'In1.Cu'),
           'tertiary_drill': (0.5, 'B.Cu', 'In4.Cu')},
    '/E': {'backdrill': (0.5, 'In3.Cu', 'B.Cu')},
}
WANT_PAD = {
    '1': {'backdrill': (1.1, 'B.Cu', 'In3.Cu')},
    '2': {'backdrill': (1.15, 'F.Cu', 'In1.Cu'),
          'tertiary_drill': (1.2, 'B.Cu', 'In4.Cu')},
}


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _uuid(n):
    return f'00000000-0000-4000-8000-{n:012d}'


def board_text(stackup=True, vias=VIAS):
    """The fixture: a six-layer KiCad 10 board, one via per net of NETS with
    the copper that joins it, and a three-pin through-hole header J1 on /A
    whose pins 1 and 2 declare back-drills."""
    n = iter(range(1, 10_000))
    out = ['(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
           '\t(generator_version "10.0")\n'
           '\t(general\n\t\t(thickness 1.31)\n\t\t(legacy_teardrops no)\n\t)\n'
           '\t(paper "A4")\n\t(layers\n'
           + ''.join(f'\t\t({i} "{name}" signal)\n' for name, i, _d in COPPER)
           + '\t\t(5 "F.SilkS" user "F.Silkscreen")\n\t\t(1 "F.Mask" user)\n'
             '\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n\t)\n\t(setup\n']
    if stackup:
        out.append('\t\t(stackup\n')
        for k, (name, _i, diel) in enumerate(COPPER):
            out.append(f'\t\t\t(layer "{name}"\n\t\t\t\t(type "copper")\n'
                       f'\t\t\t\t(thickness 0.035)\n\t\t\t)\n')
            if diel is not None:
                out.append(f'\t\t\t(layer "dielectric {k + 1}"\n\t\t\t\t(type "core")\n'
                           f'\t\t\t\t(thickness {diel})\n\t\t\t\t(material "FR4")\n'
                           f'\t\t\t\t(epsilon_r 4.5)\n\t\t\t\t(loss_tangent 0.02)\n\t\t\t)\n')
        out.append('\t\t)\n')
    out.append('\t\t(pad_to_mask_clearance 0)\n\t)\n')
    # J1: pin 3 is the control, with no back-drill.
    out.append(f'\t(footprint "T:HDR3"\n\t\t(layer "F.Cu")\n\t\t(uuid "{_uuid(next(n))}")\n'
               f'\t\t(at 10 30)\n'
               f'\t\t(property "Reference" "J1"\n\t\t\t(at 0 -2 0)\n\t\t\t(layer "F.SilkS")\n'
               f'\t\t\t(hide yes)\n\t\t\t(uuid "{_uuid(next(n))}")\n'
               f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n'
               f'\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
               f'\t\t(attr through_hole)\n')
    drills = {'1': '\t\t\t(backdrill\n\t\t\t\t(size 1.1)\n\t\t\t\t(layers "B.Cu" "In3.Cu")\n\t\t\t)\n',
              '2': '\t\t\t(backdrill\n\t\t\t\t(size 1.15)\n\t\t\t\t(layers "F.Cu" "In1.Cu")\n\t\t\t)\n'
                   '\t\t\t(tertiary_drill\n\t\t\t\t(size 1.2)\n\t\t\t\t(layers "B.Cu" "In4.Cu")\n\t\t\t)\n',
              '3': ''}
    for k, num in enumerate('123'):
        out.append(f'\t\t(pad "{num}" thru_hole circle\n\t\t\t(at {2.54 * k} 0)\n'
                   f'\t\t\t(size 1.6 1.6)\n\t\t\t(drill 0.9)\n{drills[num]}'
                   f'\t\t\t(layers "*.Cu" "*.Mask")\n\t\t\t(remove_unused_layers no)\n'
                   f'\t\t\t(net "/A")\n\t\t\t(uuid "{_uuid(next(n))}")\n\t\t)\n')
    out.append('\t)\n')
    # P1: the F.Cu pad /F's via sits in.
    out.append(f'\t(footprint "T:PAD"\n\t\t(layer "F.Cu")\n\t\t(uuid "{_uuid(next(n))}")\n'
               f'\t\t(at 60 10)\n'
               f'\t\t(property "Reference" "P1"\n\t\t\t(at 0 -2 0)\n\t\t\t(layer "F.SilkS")\n'
               f'\t\t\t(hide yes)\n\t\t\t(uuid "{_uuid(next(n))}")\n'
               f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n'
               f'\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
               f'\t\t(attr smd)\n'
               f'\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1.2 1.2)\n'
               f'\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "/F")\n'
               f'\t\t\t(uuid "{_uuid(next(n))}")\n\t\t)\n\t)\n')
    for x1, y1, x2, y2 in ((0, 0, 100, 0), (100, 0, 100, 40), (100, 40, 0, 40),
                           (0, 40, 0, 0)):
        out.append(f'\t(gr_line\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
                   f'\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
                   f'\t\t(layer "Edge.Cuts")\n\t\t(uuid "{_uuid(next(n))}")\n\t)\n')
    for net, drill, word, pair, track_layers in vias:
        x = 10 * (NETS.index(net) + 1)
        # each track leaves the via its own way, so no two overlap
        for k, layer in enumerate(track_layers):
            ex, ey = ((x, 4), (x, 16), (x - 4, 10))[k]
            out.append(f'\t(segment\n\t\t(start {x} 10)\n\t\t(end {ex} {ey})\n'
                       f'\t\t(width 0.2)\n\t\t(layer "{layer}")\n\t\t(net "{net}")\n'
                       f'\t\t(uuid "{_uuid(next(n))}")\n\t)\n')
        out.append(f'\t(via{word}\n\t\t(at {x} 10)\n\t\t(size 0.6)\n\t\t(drill 0.3)\n'
                   f'{drill}\t\t(layers "{pair[0]}" "{pair[1]}")\n\t\t(net "{net}")\n'
                   f'\t\t(uuid "{_uuid(next(n))}")\n\t)\n')
    out.append(f'\t(zone\n\t\t(net "/G")\n\t\t(layer "In4.Cu")\n'
               f'\t\t(uuid "{_uuid(next(n))}")\n\t\t(hatch edge 0.5)\n'
               f'\t\t(connect_pads\n\t\t\t(clearance 0.2)\n\t\t)\n'
               f'\t\t(min_thickness 0.2)\n\t\t(filled_areas_thickness no)\n'
               f'\t\t(fill\n\t\t\t(thermal_gap 0.3)\n\t\t\t(thermal_bridge_width 0.3)\n\t\t)\n'
               f'\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 67 7) (xy 73 7) (xy 73 13) (xy 67 13)\n'
               f'\t\t\t)\n\t\t)\n\t)\n')
    out.append('\t(embedded_fonts no)\n)\n')
    return ''.join(out)


def write_board(path, **kw):
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(board_text(**kw))
    return run_utils.evidence(path, 'the fixture board')


def parse(path):
    """(PCBData, what the parser printed)."""
    from kicad_parser import parse_kicad_pcb
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        pcb = parse_kicad_pcb(path)
    return pcb, buf.getvalue()


def via_drills(pcb):
    """{net name: {token: (size, start, end)}} for the vias that carry any."""
    return {pcb.nets[v.net_id].name: {t: tuple(bd) for t, bd in v.backdrills.items()}
            for v in pcb.vias if v.backdrills}


def reads(td):
    print('1-2. the text parser')
    from kicad_parser import Backdrill
    pcb, _log = parse(write_board(os.path.join(td, 'rd.kicad_pcb')))
    ok(len(pcb.vias) == len(VIAS), f"every via is modelled: {len(pcb.vias)} of {len(VIAS)}")
    ok(via_drills(pcb) == WANT_VIA, f"the vias' back-drills: {via_drills(pcb)}")
    ok(all(isinstance(bd, Backdrill) for v in pcb.vias for bd in v.backdrills.values()),
       'each is a Backdrill')
    ok(all(v.layers == ['F.Cu', 'B.Cu'] for v in pcb.vias if v.via_type == 'through'),
       "a back-drilled via's own layer pair is still its (layers ...)")
    pads = {p.pad_number: p for p in pcb.footprints['J1'].pads}
    got = {num: {t: tuple(bd) for t, bd in p.backdrills.items()}
           for num, p in pads.items() if p.backdrills}
    ok(got == WANT_PAD, f"the pads' back-drills: {got}")
    ok(pads['1'].layers == pads['3'].layers == pads['2'].layers and pads['3'].layers,
       f"a back-drilled pad's layer set is the plain pad's: {pads['1'].layers} "
       f"vs {pads['3'].layers}")
    ok(pads['1'].drill == pads['3'].drill == 0.9 and pads['1'].size_x == 1.6,
       "and so are its drill and size")
    # JSON: a via dict that carries one must still serialise.
    doc = json.loads(json.dumps({'backdrills': dict(pcb.vias[1].backdrills)}))
    ok(doc == {'backdrills': {'backdrill': [0.5, 'B.Cu', 'In3.Cu']}},
       f"a via dict carrying a back-drill serialises to JSON: {doc}")


#: (label, the via's back-drill text, what the parser must hold, the reason it
#: must print). Each was loaded and re-saved through pcbnew 10.0.8.
MALFORMED = [
    ('after (layers), one line', None, {'backdrill': (0.5, 'B.Cu', 'In3.Cu')}, None),
    ('no (layers): KiCad drops it', '\t\t(backdrill (size 0.5))\n', {}, None),
    ('no (size): KiCad drops it', '\t\t(backdrill (layers "B.Cu" "In3.Cu"))\n', {}, None),
    ('size 0: KiCad drops it', '\t\t(backdrill (size 0) (layers "B.Cu" "In3.Cu"))\n', {}, None),
    ('one layer twice', '\t\t(backdrill (size 0.5) (layers "B.Cu" "B.Cu"))\n',
     {'backdrill': (0.5, 'B.Cu', 'B.Cu')}, None),
    ('a size of two numbers: KiCad refuses the board',
     '\t\t(backdrill (size 0.5 0.7) (layers "B.Cu" "In3.Cu"))\n', {}, 'not one number'),
]


def malformed(td):
    print('3. tokens KiCad reads as no drill, or refuses')
    for k, (label, drill, want, reason) in enumerate(MALFORMED):
        text = board_text(vias=[('/A', drill or '', '', ('F.Cu', 'B.Cu'), ('F.Cu',))])
        if drill is None:
            text, n = re.subn(r'(\t\t\(layers "F.Cu" "B.Cu"\)\n)(\t\t\(net "/A"\)\n\t\t\(uuid "[^"]+"\)\n\t\)\n\t\(zone)',
                              r'\1\t\t(backdrill (size 0.5) (layers "B.Cu" "In3.Cu"))\n\2', text)
            assert n == 1, 'BROKEN TEST: the one via was not found'
        path = os.path.join(td, f'mal{k}.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text)
        pcb, log = parse(run_utils.evidence(path))
        ok(len(pcb.vias) == 1 and via_drills(pcb) == ({'/A': want} if want else {}),
           f"{label}: {via_drills(pcb)}")
        if reason:
            ok(reason in log and 'back-drill' in log,
               f"{label}: and it is named ({reason!r} in the warning)")
        else:
            ok('back-drill' not in log, f"{label}: nothing is warned")


def writes(td):
    print('4-5. the writers')
    from kicad_parser import extract_vias
    from kicad_writer import add_tracks_and_vias_to_pcb, generate_via_sexpr
    n2n = {'/A': 1, '': 0}
    spec = dict(WANT_VIA['/C'])
    sexpr = generate_via_sexpr(1.0, 2.0, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1, net_name='/A',
                               tenting_attrs=None, via_type=None, backdrills=spec)
    got = extract_vias(sexpr, n2n)
    ok(len(got) == 1 and {t: tuple(b) for t, b in got[0].backdrills.items()} == spec,
       f"generate_via_sexpr writes both tokens and the parser reads them back: "
       f"{got[0].backdrills if got else None}")
    ok(sexpr.index('(drill 0.3)') < sexpr.index('(backdrill') < sexpr.index('(tertiary_drill')
       < sexpr.index('(layers "F.Cu" "B.Cu")'),
       'between (drill) and the via\'s (layers), where KiCad writes them')
    strip = lambda s: re.sub(r'\(uuid "[^"]+"\)', '(uuid)', s)      # noqa: E731
    plain = generate_via_sexpr(1.0, 2.0, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1, net_name='/A')
    for label, kw in (('backdrills=None', {'backdrills': None}),
                      ('backdrills={}', {'backdrills': {}})):
        again = generate_via_sexpr(1.0, 2.0, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1,
                                   net_name='/A', **kw)
        ok(strip(again) == strip(plain) and 'backdrill' not in again,
           f"a via with none ({label}) is written byte for byte as before")

    # add_tracks_and_vias_to_pcb: a re-emitted via dict carries the key.
    src = write_board(os.path.join(td, 'w_in.kicad_pcb'))
    pcb, _ = parse(src)
    name_to_id = {n.name: i for i, n in pcb.nets.items()}
    id_to_name = {i: n.name for i, n in pcb.nets.items()}
    c = next(v for v in pcb.vias if pcb.nets[v.net_id].name == '/C')
    dst = os.path.join(td, 'w_out.kicad_pcb')
    moved = {'x': 31.0, 'y': 20.0, 'size': c.size, 'drill': c.drill,
             'layers': c.layers, 'net_id': c.net_id, 'tenting_attrs': {},
             'inherit_when_unspecified': True, 'via_type': c.via_type,
             'backdrills': dict(c.backdrills)}
    added = {'x': 33.0, 'y': 20.0, 'size': 0.6, 'drill': 0.3,
             'layers': ['F.Cu', 'B.Cu'], 'net_id': c.net_id}
    assert add_tracks_and_vias_to_pcb(src, dst, [], [moved, added],
                                      net_id_to_name=id_to_name), 'BROKEN TEST: write failed'
    with open(run_utils.evidence(dst), encoding='utf-8') as fh:
        out = extract_vias(fh.read(), name_to_id)
    by = {(v.x, v.y): v for v in out}
    ok((31.0, 20.0) in by and {t: tuple(b) for t, b in by[(31.0, 20.0)].backdrills.items()}
       == WANT_VIA['/C'], 'add_tracks_and_vias_to_pcb: a re-emitted via keeps its back-drills')
    ok((33.0, 20.0) in by and by[(33.0, 20.0)].backdrills == {},
       'and a via it adds has none')
    out_pcb, _ = parse(dst)
    ok(via_drills(out_pcb) == {**WANT_VIA, '/C': WANT_VIA['/C']}
       and sum(1 for v in out_pcb.vias if v.backdrills) == len(WANT_VIA) + 1,
       "and the board's own back-drilled vias are untouched")

    # The failed-reroute restore: the vias come off the board and go back on.
    from plane_io import restore_failed_reroute_nets
    ripped = os.path.join(td, 'w_rip.kicad_pcb')
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    start = text.index('\t(via\n\t\t(at 30 10)')
    end = text.index('\n\t)\n', start) + 4
    assert '(tertiary_drill' in text[start:end], 'BROKEN TEST: not the /C via'
    with open(ripped, 'w', encoding='utf-8') as fh:
        fh.write(text[:start] + text[end:])
    lost, _ = parse(run_utils.evidence(ripped))
    assert '/C' not in via_drills(lost), 'BROKEN TEST: the /C via is still there'
    restore_failed_reroute_nets(src, ripped, [c.net_id], [], id_to_name, 0.6, 0.2)
    back, _ = parse(ripped)
    ok(via_drills(back).get('/C') == WANT_VIA['/C'],
       f"restore_failed_reroute_nets puts the via back with its back-drills: "
       f"{via_drills(back).get('/C')}")


def _trees(dirs=('py_router', 'py_placer', 'py_tools', 'kicad_routing_plugin')):
    for d in dirs:
        for dp, _, fs in os.walk(os.path.join(REPO, d)):
            for f in sorted(fs):
                if f.endswith('.py'):
                    p = os.path.join(dp, f)
                    rel = os.path.relpath(p, REPO).replace(os.sep, '/')
                    with open(p, encoding='utf-8') as fh:
                        yield rel, ast.parse(fh.read(), rel)


def emit_sites():
    print('6. every emit site carries them')
    calls = dicts = builders = 0
    for rel, tree in _trees():
        for n in ast.walk(tree):
            # every generate_via_sexpr call decides the question
            if (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                    and n.func.id == 'generate_via_sexpr'):
                calls += 1
                if 'backdrills' not in {k.arg for k in n.keywords}:
                    ok(False, f"{rel}:{n.lineno}: generate_via_sexpr with no "
                              f"backdrills= -- an existing via it re-emits loses "
                              f"its back-drills (a via the tool adds passes None)")
            # every via dict that carries a protection spec carries them too
            elif isinstance(n, ast.Dict):
                keys = {k.value for k in n.keys if isinstance(k, ast.Constant)}
                if 'tenting_attrs' in keys and 'drill' in keys:
                    dicts += 1
                    if 'backdrills' not in keys:
                        ok(False, f"{rel}:{n.lineno}: a via dict carries "
                                  f"tenting_attrs but not backdrills")
    for rel, tree in _trees(('kicad_routing_plugin',)):
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            names = {getattr(c.func, 'attr', getattr(c.func, 'id', None))
                     for c in ast.walk(fn) if isinstance(c, ast.Call)}
            if 'PCB_VIA' in names and 'apply_via_protection' in names:
                builders += 1
                if 'apply_via_backdrills' not in names:
                    ok(False, f"{rel}:{fn.lineno} {fn.name}: builds a PCB_VIA and "
                              f"applies its protection but not its back-drills")
    ok(calls >= 10, f"generate_via_sexpr calls walked: {calls} (10 or more, or the "
                    f"walk has gone stale)")
    ok(dicts >= 4, f"via dicts carrying tenting_attrs walked: {dicts} (4 or more)")
    ok(builders >= 6, f"plugin functions building a PCB_VIA walked: {builders} "
                      f"(6 or more)")


def main():
    td = tempfile.mkdtemp(prefix='krt1925_')
    try:
        reads(td)
        malformed(td)
        writes(td)
        emit_sites()
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
