#!/usr/bin/env python3
"""#1211: `place_pose replicate` repeats one placed sheet channel onto its
sibling instances.

multichannel_mixer has four instances of one channel sheet, and an agent that
wanted them laid out alike had to write its own helper: nothing in the
placement stack used the correspondence the file already carries (every
footprint's `(path "/<sheet>/<symbol>")` ends in the SAME symbol uuid in each
instance). The verb maps parts by that uuid, refuses unless the instances are
wired alike, copies the source's relative poses at one offset, and grades the
result like every other place_pose call.

The synthetic board: three instances S1/S2/S3 of one sheet (an IC, a
resistor, a cap), channel-local nets /Sk/IN and /Sk/OUT, shared GND and VCC,
and one top-level part. S1 is the placed channel; S2 and S3 are scattered.
The arms:

* the copy lands where the arithmetic says (source pose + offset, the
  source's rotation and side), with each default anchor kept in place;
* `--offset` and `--anchor` (a ref of ANY named sheet) move the copy;
* the refusals, each by its reason and exit code: nets not wired alike
  (both the shared-net and the one-to-one arms), a part missing from one
  instance, a footprint that differs, a KiCad-locked target part, an
  unknown or ambiguous sheet, and the usage errors;
* a locked part that the copy does NOT move is not refused;
* `--allow-net-mismatch REF.PAD` waives one pad wired differently by
  design, from either instance, and the refusal spells the flags it needs;
* the net map is the one under which the most pad pairs agree (#1277):
  a strap met first names itself, not the pads wired alike;
* both sheet-name spellings are read (KiCad 8+ token, KiCad 6/7 property)
  into `Footprint.sheet_name` / `sheet_file`;
* on the tracked glasgow_revC, the two IO_Buffer instances map 68 for 68 by
  symbol, and the net rule names the I2C address straps that are wired
  differently per instance (a real asymmetry, not a pairing artefact);
  waived, they correspond.

Run: python3 -X utf8 tests/test_1211_place_pose_replicate.py [case ...]
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 600

PLACE_POSE = os.path.join(ROOT, 'py_placer', 'place_pose.py')
GLASGOW = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')

SHEET = {'S1': '5a000000-0000-0000-0000-000000000001',
         'S2': '5a000000-0000-0000-0000-000000000002',
         'S3': '6b000000-0000-0000-0000-000000000003'}
SYMBOL = {'U': '11111111-1111-1111-1111-111111111111',
          'R': '22222222-2222-2222-2222-222222222222',
          'C': '33333333-3333-3333-3333-333333333333',
          'T': '44444444-4444-4444-4444-444444444444'}
#: pad offsets per part kind (local mm) and the nets each pad carries:
#: 'IN' / 'OUT' are the instance's own, the rest are shared.
KIND = {'U': ('t:U4', [(-1.5, -1.0), (-1.5, 1.0), (1.5, 1.0), (1.5, -1.0)],
              ['VCC', 'IN', 'OUT', 'GND']),
        'R': ('t:R', [(-0.75, 0.0), (0.75, 0.0)], ['IN', 'OUT']),
        'C': ('t:C', [(-0.75, 0.0), (0.75, 0.0)], ['VCC', 'GND']),
        'T': ('t:T', [(-0.75, 0.0), (0.75, 0.0)], ['VCC', 'GND'])}

#: ref -> (kind, sheet, x, y, rot). S1 is the PLACED channel.
PARTS = {
    'U1': ('U', 'S1', 15.0, 20.0, 0.0),
    'R1': ('R', 'S1', 15.0, 14.0, 0.0),
    'C1': ('C', 'S1', 10.0, 20.0, 90.0),
    'U2': ('U', 'S2', 45.0, 20.0, 0.0),
    'R2': ('R', 'S2', 51.0, 27.0, 90.0),
    'C2': ('C', 'S2', 40.0, 12.0, 0.0),
    'U3': ('U', 'S3', 75.0, 20.0, 0.0),
    'R3': ('R', 'S3', 70.0, 28.0, 0.0),
    'C3': ('C', 'S3', 80.0, 13.0, 90.0),
    'T1': ('T', None, 50.0, 35.0, 0.0),
}


def _fmt(v):
    return ('%.6f' % v).rstrip('0').rstrip('.')


def board(td, name='b', *, drop=(), locked=(), back=(), nets=None,
          fpname=None, names=None, prop_names=False, bare_text=False,
          sheet_after_pad=(), at=None):
    """Write the fixture. `nets` overrides pad nets: {(ref, pad#): net},
    and a net name the board does not have is added; `fpname` overrides
    footprint names {ref: name}; `back` puts parts on B.Cu; `names`
    overrides an instance's sheet name {sheet: name}; `prop_names` spells
    the name the KiCad 6/7 way; `bare_text` drops the Reference's
    `(effects ...)`, which the writer's side flip needs (#714);
    `sheet_after_pad` writes those refs' sheet name and file AFTER their
    first pad, where no header field lives; `at` overrides poses {ref: (x,
    y, rot)}."""
    nets = nets or {}
    fpname = fpname or {}
    names = names or {}
    at = at or {}
    all_nets = ['GND', 'VCC'] + ['/%s/%s' % (s, n) for s in sorted(SHEET)
                                 for n in ('IN', 'OUT')]
    all_nets += sorted({n for n in nets.values() if n and n not in all_nets})
    ids = {n: i + 1 for i, n in enumerate(all_nets)}
    # The property spelling is a KiCad 6/7 one: pcbnew reads it as the sheet
    # only up to file version 20230619 (SHEET_PROPERTIES_MAX_VERSION), so
    # that fixture is written at 7's version, and `prop_names='modern'` writes
    # the same property in a 9 file, where it is an ordinary user field.
    version = ('20221018' if prop_names and prop_names != 'modern'
               else '20241229')
    out = ['(kicad_pcb (version %s) (generator "pcbnew")' % version,
           '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
           ' (36 "B.SilkS" user) (37 "F.SilkS" user)'
           ' (44 "Edge.Cuts" user) (46 "B.CrtYd" user) (47 "F.CrtYd" user))',
           '  (net 0 "")'] + ['  (net %d "%s")' % (ids[n], n) for n in all_nets]
    out.append('  (gr_rect (start 0 0) (end 90 40) (stroke (width 0.1)'
               ' (type default)) (layer "Edge.Cuts"))')
    for ref, (kind, sheet, x, y, rot) in PARTS.items():
        if ref in drop:
            continue
        x, y, rot = at.get(ref, (x, y, rot))
        fp, pads, pad_nets = KIND[kind]
        fp = fpname.get(ref, fp)
        side = 'B' if ref in back else 'F'
        path = ('/%s/%s' % (SHEET[sheet], SYMBOL[kind]) if sheet
                else '/%s' % SYMBOL[kind])
        sname = names.get(sheet, sheet) if sheet else ''
        lines = ['  (footprint "%s" (layer "%s.Cu") (at %s %s %s)%s'
                 % (fp, side, _fmt(x), _fmt(y), _fmt(rot),
                    ' (locked yes)' if ref in locked else ''),
                 '    (property "Reference" "%s" (at 0 0) (layer "%s.SilkS")%s)'
                 % (ref, side, '' if bare_text else
                    ' (effects (font (size 1 1) (thickness 0.15))%s)'
                    % (' (justify mirror)' if side == 'B' else '')),
                 '    (path "%s")' % path]
        sfile = 'strip.kicad_sch' if sheet else 'top.kicad_sch'
        if prop_names == 'modern':
            sheet_lines = [
                '    (property "Sheetname" "%s" (at 0 0) (layer "F.Fab") '
                '(hide yes))' % sname,
                '    (property "Sheetfile" "%s" (at 0 0) (layer "F.Fab") '
                '(hide yes))' % sfile]
        elif prop_names:
            # KiCad 7 wrote "Sheetname"; KiCad 6 dev files "Sheet name".
            two = prop_names if isinstance(prop_names, tuple) else (
                'Sheetname', 'Sheetfile')
            sheet_lines = ['    (property "%s" "%s")' % (two[0], sname),
                           '    (property "%s" "%s")' % (two[1], sfile)]
        else:
            sheet_lines = [('    (sheetname "/%s/")' % sname if sname
                            else '    (sheetname "/")'),
                           '    (sheetfile "%s")' % sfile]
        if ref not in sheet_after_pad:
            lines.extend(sheet_lines)
        lines.append('    (fp_rect (start -2.2 -1.6) (end 2.2 1.6) (stroke '
                     '(width 0.05) (type default)) (layer "%s.CrtYd"))' % side)
        for i, ((px, py), role) in enumerate(zip(pads, pad_nets), 1):
            net = nets.get((ref, str(i)))
            if net is None:
                net = ('/%s/%s' % (sheet, role) if role in ('IN', 'OUT')
                       else role)
            # '' is no net at all: the pad carries no `(net ...)`.
            lines.append('    (pad "%d" smd rect (at %s %s %s) (size 0.6 0.6)'
                         ' (layers "%s.Cu")%s)'
                         % (i, _fmt(px), _fmt(py), _fmt(rot), side,
                            ' (net %d "%s")' % (ids[net], net) if net
                            else ''))
            if i == 1 and ref in sheet_after_pad:
                lines.extend(sheet_lines)
        lines[-1] += ')'
        out.extend(lines)
    out.append(')')
    p = os.path.join(td, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(out) + '\n')
    return p


def summary(r):
    found = [json.loads(line[len('JSON_SUMMARY: '):])
             for line in (r.stdout or '').splitlines()
             if line.startswith('JSON_SUMMARY: ')]
    assert len(found) == 1, 'expected one JSON_SUMMARY, got %d:\n%s' % (
        len(found), (r.stdout or '')[-1500:])
    return found[0]


def pose(inp, out, *op, refuse=None, code=None, accept=False, allow=()):
    return run_utils.check([sys.executable, '-X', 'utf8', PLACE_POSE, inp,
                            out] + [str(a) for a in op],
                           refuse=refuse, code=code, accept=accept,
                           allow=allow)


def poses(path):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(run_utils.evidence(path, 'written board'))
    return {r: (round(f.x, 4), round(f.y, 4), round(f.rotation % 360.0, 4),
                f.layer) for r, f in pcb.footprints.items()}


def near(a, b, eps=1e-4):
    return all(abs(p - q) <= eps for p, q in zip(a[:3], b[:3])) and (
        len(a) < 4 or len(b) < 4 or a[3] == b[3])


def test_the_copy_lands_at_source_plus_offset():
    with tempfile.TemporaryDirectory() as td:
        inp = board(td)
        out = os.path.join(td, 'out.kicad_pcb')
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 '--to', 'S3', accept=True)
        s = summary(r)
        got = poses(out)
        # Default anchor: the most pads (the U), kept where it is.
        want = {'U2': (45, 20, 0, 'F.Cu'), 'R2': (45, 14, 0, 'F.Cu'),
                'C2': (40, 20, 90, 'F.Cu'),
                'U3': (75, 20, 0, 'F.Cu'), 'R3': (75, 14, 0, 'F.Cu'),
                'C3': (70, 20, 90, 'F.Cu')}
        for ref, w in want.items():
            assert near(got[ref], w), (ref, got[ref], w)
        # Nothing outside the targets moved: the source and the top level.
        for ref in ('U1', 'R1', 'C1', 'T1'):
            k, _sh, x, y, rot = PARTS[ref]
            assert near(got[ref], (x, y, rot)), (ref, got[ref])
        rep = s['replicate'][0]
        assert rep['from']['name'] == 'S1', rep['from']
        t2, t3 = rep['to']
        assert (t2['pairs'], t2['moved'], t2['offset']) == (3, 2, [30.0, 0.0]), t2
        assert (t3['pairs'], t3['moved'], t3['offset']) == (3, 2, [60.0, 0.0]), t3
        assert t2['offset_basis'] == {'kind': 'anchor', 'source': 'U1',
                                      'target': 'U2', 'chosen': 'most pads'}, t2
        assert t2['nets_renamed'] == {'/S1/IN': '/S2/IN',
                                      '/S1/OUT': '/S2/OUT'}, t2
        assert t2['nets_shared'] == ['GND', 'VCC'], t2
        notes = {n['ref']: n for n in s['ops']}
        assert set(notes) == {'U2', 'R2', 'C2', 'U3', 'R3', 'C3'}, sorted(notes)
        assert notes['R2']['source'] == 'R1' and notes['R2']['moved']
        assert not notes['U2']['moved'], notes['U2']
        assert s['no_worse'] is True and s['legal'] is True, s
        assert 'replicate S1 -> S2: 3 parts matched by symbol, 2 moved' in r.stdout
        print('  S2/S3 at S1 + (30,0)/(60,0); U2/U3 kept; S1 and T1 untouched')


def test_offset_and_anchor_move_the_copy():
    with tempfile.TemporaryDirectory() as td:
        inp = board(td)
        out = os.path.join(td, 'o.kicad_pcb')
        pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
             '--offset', '30', '3', accept=True)
        got = poses(out)
        for ref, w in {'U2': (45, 23, 0), 'R2': (45, 17, 0),
                       'C2': (40, 23, 90)}.items():
            assert near(got[ref], w), (ref, got[ref], w)
        # --anchor names a TARGET ref: R2 keeps its (at x y), takes R1's
        # rotation, and the channel is laid around it.
        out2 = os.path.join(td, 'a.kicad_pcb')
        r = pose(inp, out2, 'replicate', '--from', 'S1', '--to', 'S2',
                 '--anchor', 'R2', accept=True)
        got = poses(out2)
        for ref, w in {'R2': (51, 27, 0), 'U2': (51, 33, 0),
                       'C2': (46, 33, 90)}.items():
            assert near(got[ref], w), (ref, got[ref], w)
        b = summary(r)['replicate'][0]['to'][0]['offset_basis']
        assert b == {'kind': 'anchor', 'source': 'R1', 'target': 'R2',
                     'chosen': 'given'}, b
        print('  --offset 30 3 and --anchor R2 land as computed')


def test_nets_not_wired_alike_are_refused():
    with tempfile.TemporaryDirectory() as td:
        # C2's VCC pad wired to the channel input: VCC is on BOTH instances,
        # so it must map to itself.
        inp = board(td, 'shared', nets={('C2', '1'): '/S2/IN'})
        out = os.path.join(td, 'x.kicad_pcb')
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='not wired alike', code=4)
        s = summary(r)
        assert 'C1.1 VCC -> C2.1 /S2/IN' in r.stderr, r.stderr[-800:]
        assert 'a shared net' in r.stderr, r.stderr[-800:]
        assert s['net_breaks'] and s['moved'] == [], s
        assert not os.path.exists(out), 'a refusal wrote the board'
        # R2's pads swapped: /S1/IN would reach /S2/IN (at U2) AND /S2/OUT.
        inp = board(td, 'swap', nets={('R2', '1'): '/S2/OUT',
                                      ('R2', '2'): '/S2/IN'})
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='not wired alike', code=4)
        assert '/S1/IN maps to /S2/OUT (R1.1)' in r.stderr, r.stderr[-800:]
        assert not os.path.exists(out)
        # Two source nets onto ONE target net (R2's OUT pad and U2's OUT pad
        # both on /S2/IN): each source net still reaches one net, so only
        # the one-to-one arm can say no.
        inp = board(td, 'merge', nets={('R2', '2'): '/S2/IN',
                                       ('U2', '3'): '/S2/IN'})
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='not wired alike', code=4)
        assert '/S2/IN is reached from /S1/IN' in r.stderr, (
            r.stderr[-800:])
        # A pad on a SOURCE channel net that maps nowhere else: copying U1
        # alone, U2's OUT pad on S1's own input.
        inp = board(td, 'onto', nets={('U2', '3'): '/S1/IN'})
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 '--only', 'U1', refuse='not wired alike', code=4)
        assert 'so only itself may map to it' in r.stderr, r.stderr[-800:]
        # A pad with no net against one with a net.
        inp = board(td, 'nonet', nets={('R2', '1'): ''})
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='not wired alike', code=4)
        assert 'one pad has no net and the other has one' in r.stderr, (
            r.stderr[-800:])
        assert not os.path.exists(out)
        print('  shared-net, function, one-to-one, source-net and no-net '
              'breaks refused at 4, pads named')


def test_a_pad_wired_differently_by_design_is_waived_per_pad():
    """`--allow-net-mismatch REF.PAD` (#1211 follow-up): one pad per flag,
    REF on the source (every target) or on one target (that one)."""
    with tempfile.TemporaryDirectory() as td:
        # C2's VCC pad wired to the channel input, and C3's likewise.
        inp = board(td, 'strap', nets={('C2', '1'): '/S2/IN',
                                       ('C3', '1'): '/S3/IN'})
        out = os.path.join(td, 'x.kicad_pcb')
        # The refusal spells the waiver it would take, by the SOURCE pad.
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 '--to', 'S3', refuse='not wired alike', code=4)
        assert '--allow-net-mismatch C1.1' in r.stderr, r.stderr[-900:]
        assert summary(r)['suggest_waivers'] == ['C1.1'], summary(r)
        # A source ref waives the pad in EVERY target.
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2', '--to',
                 'S3', '--allow-net-mismatch', 'C1.1', accept=True)
        t2, t3 = summary(r)['replicate'][0]['to']
        assert t2['nets_waived'] == [{'pads': 'C1.1 VCC -> C2.1 /S2/IN',
                                      'as_given': 'C1.1', 'needed': True}], t2
        assert [w['pads'] for w in t3['nets_waived']] == [
            'C1.1 VCC -> C3.1 /S3/IN'], t3
        assert 'waived C1.1 VCC -> C2.1 /S2/IN' in r.stdout, r.stdout[-900:]
        assert near(poses(out)['C3'], (70, 20, 90)), poses(out)['C3']
        # A TARGET ref waives it in that target only: S3 still breaks.
        r = pose(inp, out + '2', 'replicate', '--from', 'S1', '--to', 'S2',
                 '--to', 'S3', '--allow-net-mismatch', 'C2.1',
                 refuse='not wired alike', code=4)
        assert [b['pads'] for b in summary(r)['net_breaks']] == [
            'C1.1 VCC -> C3.1 /S3/IN'], summary(r)['net_breaks']
        assert not os.path.exists(out + '2')
        # A waiver the rule did not need is reported, not refused.
        r = pose(inp, out + '3', 'replicate', '--from', 'S1', '--to', 'S2',
                 '--allow-net-mismatch', 'C1.1', '--allow-net-mismatch',
                 'R1.2', accept=True)
        got = {w['as_given']: w['needed']
               for w in summary(r)['replicate'][0]['to'][0]['nets_waived']}
        assert got == {'C1.1': True, 'R1.2': False}, got
        assert 'the waiver was not needed' in r.stdout
        # A waived pad does not make a net look SHARED: C2.1 cross-wired to
        # S1's own input would otherwise force /S1/IN to map to itself.
        inp2 = board(td, 'cross', nets={('C2', '1'): '/S1/IN'})
        pose(inp2, out + '4', 'replicate', '--from', 'S1', '--to', 'S2',
             refuse='a shared net', code=4)
        pose(inp2, out + '4', 'replicate', '--from', 'S1', '--to', 'S2',
             '--allow-net-mismatch', 'C1.1', accept=True)
        # A waiver that names nothing is refused at 2, before any grade.
        for bad, why in (('C1.9', "C1 has no pad '9'"),
                         ('C1', 'expected REF.PAD'),
                         ('C3.1', 'is in neither the --from sheet nor a '
                                  '--to sheet'),
                         ('T1.1', 'is in neither the --from sheet nor a '
                                  '--to sheet')):
            pose(inp, out + '5', 'replicate', '--from', 'S1', '--to', 'S2',
                 '--allow-net-mismatch', bad, refuse=why, code=2)
        assert not os.path.exists(out + '5')
        print('  the refusal spells --allow-net-mismatch C1.1; a source ref '
              'waives every target, a target ref one; unneeded waivers '
              'reported; a waived pad shares no net; bad waivers -> 2')


def test_the_net_map_is_chosen_over_every_pair():
    """#1277: the map THE NET RULE judges pads against is the one under
    which the most pad pairs agree, chosen over all of them at once. It was
    grown first-come, so a strap met first claimed its net: with R2.1
    strapped to /S2/OUT, /S1/IN mapped to /S2/OUT and the refusal named
    R1.2, U1.2 and U1.3 -- the three pads wired ALIKE -- and following its
    advice was accepted with /S1/IN renamed to /S2/OUT."""
    from kicad_parser import parse_kicad_pcb
    from placement import replicate
    from placement.pose_ops import PoseRefusal
    with tempfile.TemporaryDirectory() as td:
        inp = board(td, 'first', nets={('R2', '1'): '/S2/OUT'})
        out = os.path.join(td, 'x.kicad_pcb')
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='not wired alike', code=4)
        s = summary(r)
        assert [b['pads'] for b in s['net_breaks']] == [
            'R1.1 /S1/IN -> R2.1 /S2/OUT'], s['net_breaks']
        assert s['suggest_waivers'] == ['R1.1'], s['suggest_waivers']
        assert '/S1/IN maps to /S2/IN (U1.2)' in r.stderr, r.stderr[-800:]
        # Following the advice maps /S1/IN where the channel wires it.
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 '--allow-net-mismatch', 'R1.1', accept=True)
        t2 = summary(r)['replicate'][0]['to'][0]
        assert t2['nets_renamed'] == {'/S1/IN': '/S2/IN',
                                      '/S1/OUT': '/S2/OUT'}, t2
        assert [(w['as_given'], w['needed']) for w in t2['nets_waived']] == [
            ('R1.1', True)], t2['nets_waived']

        def _breaks(nets):
            pcb = parse_kicad_pcb(board(td, 'm', nets=nets))
            inst = replicate.sheet_instances(pcb)
            a = replicate.resolve_sheet(pcb, 'S1', '--from', inst)
            b = replicate.resolve_sheet(pcb, 'S2', '--to', inst)
            try:
                replicate.correspond(pcb, a, b)
            except PoseRefusal as exc:
                return [x['source_pad'] for x in exc.extra['net_breaks']]
            return []
        # A tie on one side, settled by the other: /S1/IN's two pairs split
        # 1 to 1 between /S2/OUT and /S2/IN, and /S1/OUT's one voting pair
        # (U2.3 has no net, which breaks under any map and does not vote)
        # wants /S2/OUT. A per-net majority cannot see that; the map does.
        assert _breaks({('R2', '1'): '/S2/OUT', ('U2', '3'): ''}) == [
            'R1.1', 'U1.3']
        # Controls: a map with no competing pairing never reaches the solver
        # (no strap here; glasgow's straps are on its rails, which do not
        # vote), and the swap -- a true tie -- still goes to the pairings
        # met first, as before.
        real = replicate._first_best
        replicate._first_best = None
        try:
            assert _breaks({}) == []
            assert _breaks({('C2', '1'): '/S2/IN'}) == ['C1.1']
            pcb = parse_kicad_pcb(run_utils.evidence(GLASGOW, 'glasgow'))
            inst = replicate.sheet_instances(pcb)
            a = replicate.resolve_sheet(pcb, 'IO_Buffer_A', '--from', inst)
            b = replicate.resolve_sheet(pcb, 'IO_Buffer_B', '--to', inst)
            replicate.correspond(pcb, a, b, waive=replicate.resolve_waivers(
                pcb, ['U19.2', 'U20.6', 'U21.2'], a, [b])[b])
        finally:
            replicate._first_best = real
        assert _breaks({('R2', '1'): '/S2/OUT', ('R2', '2'): '/S2/IN'}) == [
            'U1.2', 'U1.3']
    print('  R2.1 strapped: only R1.1 named, its waiver maps /S1/IN to /S2/IN; '
          'a one-sided tie settled by the other side; no solver without a '
          'competing pairing; a true tie goes to the first met')


def test_a_missing_part_is_refused():
    with tempfile.TemporaryDirectory() as td:
        inp = board(td, drop=('C2',))
        out = os.path.join(td, 'x.kicad_pcb')
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='do not hold the same parts', code=4)
        s = summary(r)
        assert s['only_in_source'] == ['C1'] and s['only_in_target'] == [], s
        assert not os.path.exists(out)
        print('  C2 absent: refused at 4, C1 named')


def test_a_different_footprint_is_refused():
    with tempfile.TemporaryDirectory() as td:
        inp = board(td, fpname={'R2': 't:R0805'})
        out = os.path.join(td, 'x.kicad_pcb')
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='mapped parts differ', code=4)
        assert 'R1 is t:R but R2 is t:R0805' in r.stderr, r.stderr[-600:]
        print('  R2 on another footprint: refused at 4')


def test_a_locked_target_part():
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'x.kicad_pcb')
        # R2 must move and is locked: refused, like `set` on a locked part.
        inp = board(td, 'lockedR', locked=('R2',))
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='locked in the board', code=4)
        # The refusal names the locked part (set's own lock guard does).
        assert summary(r)['locked'] == ['R2'] and not os.path.exists(out)
        # ...and `unlock` in the same call is the escape, as for `set`.
        pose(inp, out, 'unlock', 'R2', 'replicate', '--from', 'S1', '--to',
             'S2', accept=True)
        assert near(poses(out)['R2'], (45, 14, 0))
        # A locked part the copy does NOT move (the anchor) is no obstacle.
        inp = board(td, 'lockedU', locked=('U2',))
        out2 = os.path.join(td, 'y.kicad_pcb')
        pose(inp, out2, 'replicate', '--from', 'S1', '--to', 'S2',
             accept=True)
        from placement.parser import extract_locked_refs
        assert 'U2' in extract_locked_refs(out2), 'the anchor lost its lock'
        print('  moving a locked part refused at 4; unlock frees it; a '
              'locked anchor is fine')


def test_a_partial_copy_moves_only_the_parts_it_names():
    """`--only` / `--skip` (#1211 follow-up): a source ref names its symbol
    in every target, a target ref that one part; the parts left out stay
    put and are not checked."""
    with tempfile.TemporaryDirectory() as td:
        inp = board(td)
        orig = poses(inp)
        o1 = os.path.join(td, 'o1.kicad_pcb')
        r = pose(inp, o1, 'replicate', '--from', 'S1', '--to', 'S2', '--to',
                 'S3', '--skip', 'R1', accept=True)
        got = poses(o1)
        assert near(got['R2'], orig['R2']) and near(got['R3'], orig['R3'])
        assert near(got['C2'], (40, 20, 90)) and near(got['C3'], (70, 20, 90))
        t2, t3 = summary(r)['replicate'][0]['to']
        assert (t2['pairs'], t2['left_in_place'], t3['left_in_place']) == (
            2, ['R2'], ['R3']), (t2, t3)
        assert summary(r)['replicate'][0]['selection'] == {'skip': ['R1']}
        assert 'left in place (not copied): R2' in r.stdout
        # A TARGET ref leaves that part only; --only is the complement.
        o2 = os.path.join(td, 'o2.kicad_pcb')
        pose(inp, o2, 'replicate', '--from', 'S1', '--to', 'S2', '--to',
             'S3', '--skip', 'R2', accept=True)
        got = poses(o2)
        assert near(got['R2'], orig['R2']) and near(got['R3'], (75, 14, 0))
        o3 = os.path.join(td, 'o3.kicad_pcb')
        pose(inp, o3, 'replicate', '--from', 'S1', '--to', 'S2', '--to',
             'S3', '--only', 'U1,C1', accept=True)
        assert poses(o3) == poses(o1), 'only U1,C1 != skip R1'
        # Leaving the op-amp out moves the default anchor to the largest
        # part copied into every target (R and C tie at two pads; C1 first):
        # C2 keeps its place and R2 is laid around it.
        o4 = os.path.join(td, 'o4.kicad_pcb')
        r = pose(inp, o4, 'replicate', '--from', 'S1', '--to', 'S2',
                 '--skip', 'U1', accept=True)
        b = summary(r)['replicate'][0]['to'][0]['offset_basis']
        assert (b['source'], b['target']) == ('C1', 'C2'), b
        got = poses(o4)
        assert near(got['C2'], (40, 12, 90)) and near(got['R2'], (45, 6, 0)), (
            got['C2'], got['R2'])
        assert near(got['U2'], orig['U2'])
        # A part left out needs no counterpart, may differ in footprint, and
        # cannot break the net rule.
        for kw, skip in ((dict(drop=('C2',)), 'C1'),
                         (dict(fpname={'R2': 't:R0805'}), 'R2'),
                         (dict(nets={('R2', '1'): '/S2/OUT',
                                     ('R2', '2'): '/S2/IN'}), 'R1')):
            b = board(td, 'k%s' % skip, **kw)
            pose(b, os.path.join(td, 'k.kicad_pcb'), 'replicate', '--from',
                 'S1', '--to', 'S2', '--skip', skip, accept=True)
        # ...but a rail on a part left out is still SHARED: U2.1 off VCC onto
        # a new net, with C2 (the only other VCC pad in S2) left out, does not
        # turn VCC into a channel net that may be renamed.
        b = board(td, 'rail', nets={('U2', '1'): '/S2/NEW'})
        pose(b, os.path.join(td, 'k.kicad_pcb'), 'replicate', '--from', 'S1',
             '--to', 'S2', '--skip', 'C2', refuse='VCC is on pads of BOTH '
             'instances', code=4)
        # A member the copy puts across the outline is NAMED, by part and by
        # pad: those channels carry no pair, and a refusal listing only
        # pairs left the caller unable to say which member to leave out.
        r = pose(inp, o1 + 'x', 'replicate', '--from', 'S1', '--to', 'S2',
                 '--only', 'R1', '--offset', '30', '26', refuse='WORSE',
                 code=4)
        nc = {n['ref']: n.get('new_conflicts') for n in summary(r)['ops']}
        assert set(nc) == {'R2'}, nc
        assert 'oob_pad_copper R2' in nc['R2'], nc
        assert {'pad_edge R2.1', 'pad_edge R2.2'} <= set(nc['R2']), nc
        # Usage: both flags, a ref outside the named sheets, nothing left.
        argparse_exit = ('usage:',)
        pose(inp, o1 + 'x', 'replicate', '--from', 'S1', '--to', 'S2',
             '--only', 'U1', '--skip', 'R1', refuse='both --only and --skip',
             code=2, allow=argparse_exit)
        pose(inp, o1 + 'x', 'replicate', '--from', 'S1', '--to', 'S2',
             '--skip', 'T1', refuse='--skip T1: T1 is in neither the --from '
             'sheet nor a --to sheet', code=2)
        pose(inp, o1 + 'x', 'replicate', '--from', 'S1', '--to', 'S2',
             '--skip', 'U1,R1,C1', refuse='--skip leaves nothing to copy',
             code=2)
        assert not os.path.exists(o1 + 'x')
        print('  --skip R1 / --skip R2 / --only U1,C1 land as named; a part '
              'left out is not checked, a rail on one is still shared')


def test_search_keeps_the_nearest_offset_that_grades_no_worse():
    """`--search MM` (#1211 follow-up): the copy as asked stacks R2 on T1;
    the nearest lattice offset that this call grades no worse is kept."""
    with tempfile.TemporaryDirectory() as td:
        # T1 sits exactly where R2's copy lands (S1 + (30, 0) = (45, 14)).
        inp = board(td, 'blocked', at={'T1': (45.0, 14.0, 0.0)})
        out = os.path.join(td, 'x.kicad_pcb')
        pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
             refuse='WORSE', code=4)
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 '--search', '2', accept=True)
        t = summary(r)['replicate'][0]['to'][0]
        # Within 1.0 mm only (0, +-1) and (+-1, 0) clear T1's pads; (dy, dx)
        # order puts (0, -1) first.
        assert t['search']['found'] and t['search']['delta'] == [0.0, -1.0], t
        assert t['search']['dist_mm'] == 1.0 and t['search']['tried'] > 1, t
        assert (t['offset_default'], t['offset']) == ([30.0, 0.0],
                                                       [30.0, -1.0]), t
        got = poses(out)
        for ref, w in {'U2': (45, 19, 0), 'R2': (45, 13, 0),
                       'C2': (40, 19, 90), 'T1': (45, 14, 0)}.items():
            assert near(got[ref], w), (ref, got[ref], w)
        # The anchor moved with the copy, and its note says so.
        notes = {n['ref']: n for n in summary(r)['ops']}
        assert notes['U2']['moved'] and notes['U2']['to'][:2] == [45.0, 19.0]
        assert '--search: the copy as asked was refused' in r.stdout
        # Too small a radius: refused at 4, the census and reason say so.
        r = pose(inp, out + '2', 'replicate', '--from', 'S1', '--to', 'S2',
                 '--search', '0.5', refuse='--search found no offset within '
                 '0.5 mm of S2', code=4)
        sr = summary(r)['replicate'][0]['to'][0]['search']
        assert (sr['found'], sr['candidates'], sr['tried']) == (False, 5, 5), sr
        assert not os.path.exists(out + '2')
        # A locked anchor cannot move with the copy: every offset but the
        # default is skipped, and said so.
        inp2 = board(td, 'lockedU', at={'T1': (45.0, 14.0, 0.0)},
                     locked=('U2',))
        r = pose(inp2, out + '3', 'replicate', '--from', 'S1', '--to', 'S2',
                 '--search', '1', refuse='skipped for moving a locked part',
                 code=4)
        sr = summary(r)['replicate'][0]['to'][0]['search']
        assert (sr['tried'], sr['skipped_guarded']) == (1, sr['candidates'] - 1)
        for bad, why in (('0', '--search` is a radius'),
                         ('-1', '--search` is a radius')):
            pose(inp, out + '4', 'replicate', '--from', 'S1', '--to', 'S2',
                 '--search', bad, refuse=why, code=2, allow=('usage:',))
        pose(inp, out + '4', 'replicate', '--from', 'S1', '--to', 'S2',
             '--search-step', '0.5', refuse='which this call does not ask '
             'for', code=2, allow=('usage:',))
        print('  R2 stacked on T1: --search 2 keeps (0, -1), 1.0 mm; 0.5 mm '
              'finds none (5 tried); a locked anchor skips every offset')


def test_search_fits_targets_in_order_leaving_later_ones_out():
    """Targets are fitted one at a time, and a later target's INPUT pose is
    no obstacle to an earlier one: it is about to move."""
    with tempfile.TemporaryDirectory() as td:
        # S3's R3 sits (on the input) where S2's copy of R1 lands, and T1
        # where S3's copy of R1 lands. The copy as asked stacks R3 on T1.
        inp = board(td, 'order', at={'R3': (45.0, 14.0, 0.0),
                                     'T1': (75.0, 14.0, 0.0)})
        out = os.path.join(td, 'x.kicad_pcb')
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2', '--to',
                 'S3', '--search', '2', accept=True)
        t2, t3 = summary(r)['replicate'][0]['to']
        # S2 keeps its default: R3's input pose was left out of its grade.
        assert t2['search']['delta'] == [0.0, 0.0], t2['search']
        assert t2['search']['later_left_out'] == 3, t2['search']
        assert t3['search']['delta'] == [0.0, -1.0], t3['search']
        assert t3['search']['later_left_out'] == 0, t3['search']
        got = poses(out)
        for ref, w in {'R2': (45, 14, 0), 'U2': (45, 20, 0),
                       'R3': (75, 13, 0), 'U3': (75, 19, 0)}.items():
            assert near(got[ref], w), (ref, got[ref], w)
        assert summary(r)['legal'] is True, summary(r)
        print('  S2 keeps its default though R3 sat there; S3 steps to '
              '(0, -1) off T1')


def test_unknown_and_ambiguous_sheets():
    with tempfile.TemporaryDirectory() as td:
        inp = board(td)
        out = os.path.join(td, 'x.kicad_pcb')
        r = pose(inp, out, 'replicate', '--from', 'S9', '--to', 'S2',
                 refuse='is neither a sheet name nor a sheet path', code=2)
        assert [d['name'] for d in summary(r)['sheets']] == ['S1', 'S2', 'S3']
        # A unique start of a path resolves; a shared one is ambiguous.
        pose(inp, out, 'replicate', '--from', '/5a000000-0000-0000-0000-'
             '000000000001', '--to', '6b00', accept=True)
        # ...but not a start too short to mean a path: a mistyped name must
        # not land on whichever uuid happens to begin with it.
        pose(inp, out, 'replicate', '--from', 'S1', '--to', '6b',
             refuse='is neither a sheet name nor a sheet path', code=2)
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', '5a00',
                 refuse='is ambiguous: it is the start of 2 sheet paths',
                 code=2)
        assert len(summary(r)['sheets']) == 2
        # Two instances carrying one name: ambiguous, not "the first".
        inp2 = board(td, 'dupname', names={'S3': 'S2'})
        pose(inp2, out, 'replicate', '--from', 'S1', '--to', 'S2',
             refuse='is ambiguous: 2 sheet instances carry that name', code=2)
        # Case-insensitive, slashes optional.
        pose(inp, out, 'replicate', '--from', '/s1/', '--to', 's2',
             accept=True)
        print('  unknown -> 2 with the sheets listed; ambiguous name/path -> 2')


def test_usage_errors():
    with tempfile.TemporaryDirectory() as td:
        inp = board(td)
        out = os.path.join(td, 'x.kicad_pcb')
        argparse_exit = ('usage:',)
        pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2', '--to',
             'S3', '--offset', '1', '2', refuse='would all land on that spot',
             code=2, allow=argparse_exit)
        pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
             '--anchor', 'U2', '--offset', '1', '2',
             refuse='both --offset and --anchor', code=2, allow=argparse_exit)
        pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S1',
             refuse='a sheet is not copied onto itself', code=2)
        pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2', '--anchor',
             'T1', refuse='is in neither the --from sheet nor a --to sheet',
             code=2)
        # Another op moving a SOURCE part: the copy would describe poses the
        # output does not have.
        pose(inp, out, 'set', 'R1', '16', '14', 'replicate', '--from', 'S1',
             '--to', 'S2', refuse="the copy reads the INPUT board's poses",
             code=2)
        # A target part named by a second op: one part, one pose.
        pose(inp, out, 'set', 'R2', '30', '30', 'replicate', '--from', 'S1',
             '--to', 'S2', refuse='is named by two ops in one call', code=2)
        assert not os.path.exists(out)
        print('  six usage errors refused at 2, nothing written')


def test_side_is_copied():
    with tempfile.TemporaryDirectory() as td:
        inp = board(td, back=('C2',))
        out = os.path.join(td, 'x.kicad_pcb')
        r = pose(inp, out, 'replicate', '--from', 'S1', '--to', 'S2',
                 accept=True)
        assert near(poses(out)['C2'], (40, 20, 90, 'F.Cu')), poses(out)['C2']
        t = summary(r)['replicate'][0]['to'][0]
        assert t['side_flips'] == ['C2 B->F'], t
        # A flip the writer cannot write faithfully is a REFUSAL with a
        # summary, not a traceback.
        inp = board(td, 'bare', back=('C2',), bare_text=True)
        out2 = os.path.join(td, 'y.kicad_pcb')
        r = pose(inp, out2, 'replicate', '--from', 'S1', '--to', 'S2',
                 refuse='the writer cannot write this arrangement', code=4)
        assert summary(r)['output'] is None and not os.path.exists(out2)
        print('  C2 on B.Cu is flipped to S1\'s side (F) at C1\'s rotation; '
              'an unwritable flip refused at 4')


def test_both_sheet_name_spellings_are_read():
    """`Footprint.sheet_name` / `sheet_file` as the FILE spells them, either
    form, and only from the block header: a pad cannot lend a block a sheet.
    (pcbnew's half of the parity is tests/gui_parity/
    test_1211_sheet_name_parity.py.)"""
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        new = parse_kicad_pcb(board(td, 'tok')).footprints
        old = parse_kicad_pcb(board(td, 'prop', prop_names=True)).footprints
        assert new['R2'].sheet_name == '/S2/' and old['R2'].sheet_name == 'S2', (
            new['R2'].sheet_name, old['R2'].sheet_name)
        assert new['T1'].sheet_name == '/' and old['T1'].sheet_name == '', (
            new['T1'].sheet_name, old['T1'].sheet_name)
        assert (new['R2'].sheet_file, old['R2'].sheet_file) == (
            'strip.kicad_sch', 'strip.kicad_sch'), (new['R2'].sheet_file,
                                                     old['R2'].sheet_file)
        # KiCad 6's spelling reads too; in a file past 20230619 the property
        # is a user field, as pcbnew 10 loads it, and names no sheet.
        six = parse_kicad_pcb(board(td, 'six', prop_names=(
            'Sheet name', 'Sheet file'))).footprints
        assert (six['R2'].sheet_name, six['R2'].sheet_file) == (
            'S2', 'strip.kicad_sch'), (six['R2'].sheet_name, six['R2'].sheet_file)
        late_prop = parse_kicad_pcb(board(td, 'modern', prop_names='modern')).footprints
        assert (late_prop['R2'].sheet_name, late_prop['R2'].sheet_file) == (
            '', ''), (late_prop['R2'].sheet_name, late_prop['R2'].sheet_file)
        # Escapes are undone, as pcbnew's GetSheetname() returns them.
        esc = parse_kicad_pcb(board(td, 'esc', names={'S2': 'my \\"S2\\"'}))
        assert esc.footprints['R2'].sheet_name == '/my "S2"/', (
            esc.footprints['R2'].sheet_name)
        # A sheet token AFTER the first pad is not the block's: R2 reads none.
        late = parse_kicad_pcb(board(td, 'late', sheet_after_pad=('R2',)))
        assert (late.footprints['R2'].sheet_name,
                late.footprints['R2'].sheet_file) == ('', ''), (
            late.footprints['R2'].sheet_name, late.footprints['R2'].sheet_file)
        assert late.footprints['U2'].sheet_name == '/S2/'
        out = os.path.join(td, 'x.kicad_pcb')
        pose(os.path.join(td, 'prop.kicad_pcb'), out, 'replicate', '--from',
             'S1', '--to', 'S2', accept=True)
        print('  (sheetname "/S2/") and a KiCad 6/7 (property "Sheetname" "S2")'
              ' / "Sheet name" both read, with the sheet file; the property in'
              ' a KiCad 9 file is no sheet; escapes undone')


def test_glasgow_maps_by_symbol_and_names_the_address_straps():
    """The tracked board with real repeated instances. The two IO_Buffer
    sheets hold 68 parts each, matched one for one by symbol; the copy is
    refused because the I2C expanders' / DAC's / monitor's address pins are
    strapped differently per instance -- the net rule's job is to say so."""
    from kicad_parser import parse_kicad_pcb
    from placement import replicate
    from placement.pose_ops import PoseRefusal
    pcb = parse_kicad_pcb(run_utils.evidence(GLASGOW, 'glasgow board'))
    inst = replicate.sheet_instances(pcb)
    a = replicate.resolve_sheet(pcb, 'IO_Buffer_A', '--from', inst)
    b = replicate.resolve_sheet(pcb, 'IO_Buffer_B', '--to', inst)
    assert (replicate.describe(a, inst[a])['file'],
            replicate.describe(b, inst[b])['file']) == (
        'io_buffer.kicad_sch', 'io_buffer.kicad_sch')
    ma, mb = replicate.members(pcb, a), replicate.members(pcb, b)
    assert len(ma) == len(mb) == 68 and set(ma) == set(mb), (len(ma), len(mb))
    try:
        replicate.correspond(pcb, a, b)
    except PoseRefusal as exc:
        assert exc.code == 4, exc.code
        pads = sorted(x['pads'].split(' ', 1)[0] for x in exc.extra['net_breaks'])
        assert pads == ['U19.2', 'U20.6', 'U21.2'], pads
        assert exc.extra['suggest_waivers'] == ['U19.2', 'U20.6', 'U21.2'], (
            exc.extra['suggest_waivers'])
        assert ('--allow-net-mismatch U19.2 --allow-net-mismatch U20.6 '
                '--allow-net-mismatch U21.2') in exc.reason, exc.reason
    else:
        raise AssertionError('glasgow IO_Buffer_A -> B was not refused')
    # The three waivers the refusal spelled -- by source pad, or the same
    # straps by their IO_Buffer_B pads -- make the instances correspond.
    for given in (['U19.2', 'U20.6', 'U21.2'], ['U5.2', 'U13.6', 'U12.2']):
        waive = replicate.resolve_waivers(pcb, given, a, [b])[b]
        pairs, nets = replicate.correspond(pcb, a, b, waive=waive)
        assert len(pairs) == 68, len(pairs)
        assert sorted(w['as_given'] for w in nets['waived']) == sorted(given)
        assert all(w['needed'] for w in nets['waived']), nets['waived']
    # Two of the three is not enough: the third strap still breaks.
    try:
        replicate.correspond(pcb, a, b, waive=replicate.resolve_waivers(
            pcb, ['U19.2', 'U20.6'], a, [b])[b])
    except PoseRefusal as exc:
        assert [x['source_pad'] for x in exc.extra['net_breaks']] == ['U21.2']
    else:
        raise AssertionError('a third strap passed unwaived')
    print('  68 for 68 by symbol; U19.2 / U20.6 / U21.2 named and spelled as '
          '--allow-net-mismatch; waived (from either side) they correspond')


TESTS = [
    test_the_copy_lands_at_source_plus_offset,
    test_offset_and_anchor_move_the_copy,
    test_nets_not_wired_alike_are_refused,
    test_a_pad_wired_differently_by_design_is_waived_per_pad,
    test_the_net_map_is_chosen_over_every_pair,
    test_a_missing_part_is_refused,
    test_a_different_footprint_is_refused,
    test_a_partial_copy_moves_only_the_parts_it_names,
    test_search_keeps_the_nearest_offset_that_grades_no_worse,
    test_search_fits_targets_in_order_leaving_later_ones_out,
    test_a_locked_target_part,
    test_unknown_and_ambiguous_sheets,
    test_usage_errors,
    test_side_is_copied,
    test_both_sheet_name_spellings_are_read,
    test_glasgow_maps_by_symbol_and_names_the_address_straps,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
