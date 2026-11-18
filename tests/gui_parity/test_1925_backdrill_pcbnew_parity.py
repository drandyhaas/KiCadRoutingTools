#!/usr/bin/env python3
"""#1925 parity gate: both parse paths read a via's and a pad's back-drills, KiCad loads what the writer writes, and the plugin puts them back.

`parse_kicad_pcb` reads `(backdrill ...)` / `(tertiary_drill ...)` from the
file text. `build_pcb_data_from_board` reads the padstack's secondary and
tertiary drill from live pcbnew. Arms, on the fixture of
tests/test_1925_backdrill_read_keep.py (written in KiCad's own form):

1. **Parity.** `Via.backdrills` and `Pad.backdrills` are equal on the two
   paths, item for item, and so is every pad's layer set.
2. **A native oracle.** pcbnew's own getters on the loaded board give the
   text-parsed size and layers. Two paths sharing one wrong reading would
   agree with each other and still fail here.
3. **Witness.** The vias and pads with a back-drill are exactly the ones the
   fixture declares, on both paths. A reader answering {} everywhere passes
   the parity arm; this makes it fail.
4. **The removed layers.** `via_stubs.drilled_layers` names, for each via,
   exactly the copper layers pcbnew's `IsBackdrilledOrPostMachined` answers
   True for: start to end, both inclusive.
5. **The writer's form loads.** A via `generate_via_sexpr` wrote with
   back-drills is loaded by pcbnew, saved by pcbnew, and both saves read
   back the same value.
6. **The plugin's twin.** `gui_utils.apply_via_backdrills` on a new PCB_VIA,
   saved by pcbnew, reads back the same value; a via given none is saved
   with no token.

Needs pcbnew; re-execs into KiCad's python automatically. Exits 2 when no
pcbnew python is found, rather than 0.

    python3 -X utf8 tests/gui_parity/test_1925_backdrill_pcbnew_parity.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]

FAILURES = []


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("NOT RUN: no python with pcbnew found (exit 2, not a pass)")
    sys.exit(2)


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + str(detail)) if detail else ''))
        FAILURES.append(what)


def via_map(pcb):
    return {(round(v.x, 4), round(v.y, 4)): {t: tuple(b) for t, b in v.backdrills.items()}
            for v in pcb.vias}


def pad_map(pcb):
    return {(k, p.pad_number): ({t: tuple(b) for t, b in p.backdrills.items()},
                                tuple(p.layers))
            for k, fp in pcb.footprints.items() for p in fp.pads}


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import pcbnew
    for _p in ('', 'py_router', 'py_tools', 'tests'):
        _d = os.path.join(REPO, _p)
        if _d not in sys.path:
            sys.path.insert(0, _d)
    from kicad_parser import (BACKDRILL_TOKENS, build_pcb_data_from_board,
                              parse_kicad_pcb, pcbnew_layer_maps)
    from kicad_writer import generate_via_sexpr
    from via_stubs import drilled_layers
    import test_1925_backdrill_read_keep as fx
    if not hasattr(pcbnew.PCB_VIA, 'GetSecondaryDrillSize'):
        print('SKIP: this pcbnew (%s) has no secondary-drill accessors; the '
              'tokens are KiCad 10\'s' % pcbnew.GetBuildVersion())
        return 77
    print('KiCad build: %s' % pcbnew.GetBuildVersion())
    tmp = tempfile.mkdtemp(prefix='krt1925g_')
    try:
        path = fx.write_board(os.path.join(tmp, 'fx.kicad_pcb'))
        f = parse_kicad_pcb(path)
        board = pcbnew.LoadBoard(path)
        check(board is not None, 'pcbnew loads the fixture')
        g = build_pcb_data_from_board(board)

        print('\n1. parity')
        check(via_map(f) == via_map(g), 'Via.backdrills agree via for via',
              'text=%s pcbnew=%s' % (via_map(f), via_map(g)))
        check(pad_map(f) == pad_map(g),
              'Pad.backdrills and Pad.layers agree pad for pad',
              'text=%s pcbnew=%s' % (pad_map(f), pad_map(g)))

        print('\n2. native oracle')
        name = pcbnew_layer_maps(board)[2]
        native = {}
        for t in board.GetTracks():
            if t.GetClass() != 'PCB_VIA':
                continue
            row = {}
            for token, stem in zip(BACKDRILL_TOKENS, ('Secondary', 'Tertiary')):
                size = getattr(t, 'Get%sDrillSize' % stem)()
                if size:
                    row[token] = (pcbnew.ToMM(size),
                                  name(getattr(t, 'Get%sDrillStartLayer' % stem)()),
                                  name(getattr(t, 'Get%sDrillEndLayer' % stem)()))
            native[(round(pcbnew.ToMM(t.GetPosition().x), 4),
                    round(pcbnew.ToMM(t.GetPosition().y), 4))] = row
        check(native == via_map(f), "pcbnew's own getters give the text-parsed value",
              'native=%s text=%s' % (native, via_map(f)))

        print('\n3. witness')
        want_v = {(10.0 * (fx.NETS.index(n) + 1), 10.0): d for n, d in fx.WANT_VIA.items()}
        for label, pcb in (('text', f), ('pcbnew', g)):
            got = {k: d for k, d in via_map(pcb).items() if d}
            check(got == want_v, '%s: the back-drilled vias are exactly the %d declared'
                  % (label, len(want_v)), got)
            gotp = {k[1]: d for k, (d, _l) in pad_map(pcb).items() if d}
            check(gotp == fx.WANT_PAD, '%s: the back-drilled pads are exactly the %d '
                  'declared' % (label, len(fx.WANT_PAD)), gotp)

        print('\n4. the removed layers')
        copper = list(f.board_info.copper_layers)
        ids = {n: i for i, n in ((board.GetLayerID(n), n) for n in copper)}
        by_pos = {(round(v.x, 4), round(v.y, 4)): v for v in f.vias}
        bad, seen = [], 0
        for t in board.GetTracks():
            if t.GetClass() != 'PCB_VIA':
                continue
            pos = (round(pcbnew.ToMM(t.GetPosition().x), 4),
                   round(pcbnew.ToMM(t.GetPosition().y), 4))
            kicad = [n for n in copper if t.IsBackdrilledOrPostMachined(ids[n])]
            ours = drilled_layers(by_pos[pos].backdrills, copper)
            seen += bool(kicad)
            if kicad != ours:
                bad.append((pos, kicad, ours))
        check(not bad and seen == len(fx.WANT_VIA),
              'drilled_layers is pcbnew\'s IsBackdrilledOrPostMachined, layer for '
              'layer (%d back-drilled vias)' % seen, bad)

        print('\n5. the writer\'s form loads')
        spec = dict(fx.WANT_VIA['/C'])
        nid = next(i for i, n in f.nets.items() if n.name == '/A')
        with open(path, encoding='utf-8') as fh:
            text = fh.read()
        tail = text.rstrip().rfind(')')
        ours_path = os.path.join(tmp, 'ours.kicad_pcb')
        with open(ours_path, 'w', encoding='utf-8') as fh:
            fh.write(text[:tail] + generate_via_sexpr(
                15.0, 25.0, 0.6, 0.3, ['F.Cu', 'B.Cu'], nid, net_name='/A',
                tenting_attrs=None, via_type=None, backdrills=spec) + '\n)\n')
        wb = pcbnew.LoadBoard(ours_path)
        check(wb is not None, 'pcbnew loads a board holding the writer\'s via')
        resaved = os.path.join(tmp, 'resaved.kicad_pcb')
        pcbnew.SaveBoard(resaved, wb)
        for label, p in (('as written', ours_path), ('after pcbnew\'s own save', resaved)):
            got = via_map(parse_kicad_pcb(p)).get((15.0, 25.0))
            check(got == spec, '%s: the via reads back with both back-drills' % label, got)
        check(via_map(build_pcb_data_from_board(wb)).get((15.0, 25.0)) == spec,
              'and the live board holds them')

        print('\n6. the plugin\'s twin')
        sys.path.insert(0, REPO)
        from kicad_routing_plugin.gui_utils import apply_via_backdrills
        lb = pcbnew.LoadBoard(path)
        made = []
        for x, drills in ((15.0, spec), (17.0, None), (19.0, {})):
            v = pcbnew.PCB_VIA(lb)
            v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(25.0)))
            v.SetWidth(pcbnew.F_Cu, pcbnew.FromMM(0.6))
            v.SetDrill(pcbnew.FromMM(0.3))
            v.SetNet(lb.FindNet('/A'))
            made.append(apply_via_backdrills(v, drills))
            lb.Add(v)
        check(made == [True, False, False],
              'apply_via_backdrills applies a spec and leaves a via with none alone',
              made)
        live_out = os.path.join(tmp, 'live.kicad_pcb')
        pcbnew.SaveBoard(live_out, lb)
        got = via_map(parse_kicad_pcb(live_out))
        check(got.get((15.0, 25.0)) == spec, 'the applied via is saved with both '
              'back-drills', got.get((15.0, 25.0)))
        check(got.get((17.0, 25.0)) == {} and got.get((19.0, 25.0)) == {},
              'and the vias given none are saved with no token')
        check({k: d for k, d in got.items() if d and k[1] == 10.0} == want_v,
              'the board\'s own back-drilled vias survive the save')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILURES:
        print('\nFAIL: %d: %s' % (len(FAILURES), '; '.join(FAILURES)))
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
