#!/usr/bin/env python3
"""#1268 parity gate: both parse paths read a via's span the way KiCad does.

A via has copper on part of the stack only when KiCad TYPES it blind, buried
or micro. `geometry_utils.via_on_layer` decides from `Via.via_type`, which the
text parser reads off the block's type word and `build_pcb_data_from_board`
reads from `GetViaType()`. This gate holds both against pcbnew's own answer,
`PCB_VIA.IsOnLayer`, layer by layer, on two boards:

  * TYPED -- through, blind, buried and micro vias as pcbnew saves them;
  * UNTYPED -- the same file with every type word stripped, which is what this
    repo's writer emitted for a via whose layer pair is not F.Cu/B.Cu. KiCad
    loads every one of those as a THROUGH via and snaps its pair to
    F.Cu/B.Cu; both parse paths must report the same type AND the same pair.
    (The stripped file keeps its four partial pairs, asserted, so the arm is
    not vacuous.)

A third arm, RE-CREATE, puts each via of the typed board back on a fresh board
the way the plugin does (gui_utils.set_via_layers) and holds the result to the
original copper. Its negative control is the pair-then-type order, which
pcbnew snaps to the full stack.

Needs pcbnew; re-execs into KiCad's python automatically.

    python3 -X utf8 tests/gui_parity/test_via_type_parse_parity.py
"""
import glob
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Newest KiCad first by NUMERIC version: a plain string sort puts KiCad\9.0
# above KiCad\10.0, so a box with both ran this under 9.0.
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

COPPER = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu']
# A `--layers F.Cu In2.Cu B.Cu` run's config.layers on this stack.
SHUFFLED = ['F.Cu', 'In2.Cu', 'B.Cu', 'In1.Cu', 'In3.Cu', 'In4.Cu']
FAILURES = []


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                # os.execv re-splits argv on spaces through the CRT on Windows
                # ("C:\Program Files\..." tears in two); subprocess quotes it.
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


def _build(pcbnew, path):
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(len(COPPER))
    net = pcbnew.NETINFO_ITEM(board, "SIG")
    board.Add(net)
    mm = pcbnew.FromMM
    blind = getattr(pcbnew, 'VIATYPE_BLIND', None)
    if blind is None:
        blind = pcbnew.VIATYPE_BLIND_BURIED          # KiCad 9
    buried = getattr(pcbnew, 'VIATYPE_BURIED', blind)
    specs = [(pcbnew.VIATYPE_THROUGH, pcbnew.F_Cu, pcbnew.B_Cu),
             (blind, pcbnew.F_Cu, pcbnew.In2_Cu),
             (buried, pcbnew.In1_Cu, pcbnew.In3_Cu),
             (pcbnew.VIATYPE_MICROVIA, pcbnew.F_Cu, pcbnew.In1_Cu),
             (pcbnew.VIATYPE_MICROVIA, pcbnew.B_Cu, pcbnew.In4_Cu)]
    for i, (vt, top, bot) in enumerate(specs):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(2 + 2 * i), mm(5)))
        v.SetViaType(vt)
        v.SetLayerPair(top, bot)
        try:
            v.SetWidth(pcbnew.F_Cu, mm(0.6))
        except TypeError:
            v.SetWidth(mm(0.6))
        v.SetDrill(mm(0.3))
        v.SetNet(net)
        board.Add(v)
    pcbnew.SaveBoard(path, board)


def _grade(pcbnew, label, path, expect_partial_pairs):
    from geometry_utils import via_on_layer, via_span_layer_indices
    from kicad_parser import build_pcb_data_from_board, parse_kicad_pcb

    print('\n%s' % label)
    board = pcbnew.LoadBoard(path)
    ids = {name: board.GetLayerID(name) for name in COPPER}
    truth = {}
    for t in board.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            key = (round(pcbnew.ToMM(t.GetPosition().x), 3),
                   round(pcbnew.ToMM(t.GetPosition().y), 3))
            truth[key] = {n for n in COPPER if t.IsOnLayer(ids[n])}
    text = parse_kicad_pcb(path)
    live = build_pcb_data_from_board(board)
    check(len(truth) == 5 and len(text.vias) == 5 and len(live.vias) == 5,
          '%s: five vias on every side' % label,
          'pcbnew %d, text %d, live %d' % (len(truth), len(text.vias), len(live.vias)))
    partial_pairs = sum(1 for v in text.vias if set(v.layers) != {'F.Cu', 'B.Cu'})
    check(partial_pairs == expect_partial_pairs,
          '%s: the text parse carries %d partial layer pair(s)'
          % (label, expect_partial_pairs), 'got %d' % partial_pairs)
    for side, pcb in (('text', text), ('pcbnew', live)):
        for v in pcb.vias:
            key = (round(v.x, 3), round(v.y, 3))
            want = truth.get(key)
            got = {n for n in COPPER if via_on_layer(v, n)}
            check(got == want, '%s %s: via at %s (%s, %s) is on %s'
                  % (label, side, key, v.via_type, '/'.join(v.layers),
                     sorted(want or ())), 'model says %s' % sorted(got))
            span = [SHUFFLED[i] for i in via_span_layer_indices(v, SHUFFLED, len(SHUFFLED))]
            check(set(span) == want and len(span) == len(want),
                  '%s %s: via at %s spans the same layers of a shuffled map'
                  % (label, side, key), 'got %s' % span)
    t_types = sorted((round(v.x, 3), v.via_type) for v in text.vias)
    l_types = sorted((round(v.x, 3), v.via_type) for v in live.vias)
    check(t_types == l_types, '%s: both parse paths read the same via_type' % label,
          '%s vs %s' % (t_types, l_types))
    # ...and the same layer PAIR, IN ORDER: pcbnew snaps an untyped via's
    # pair to F.Cu/B.Cu and reads every pair top layer first, and so does the
    # text parser, so a consumer reading the pair -- placement Apply keys a
    # via on the ordered pair -- sees KiCad's via on either path.
    t_pairs = sorted((round(v.x, 3), tuple(v.layers)) for v in text.vias)
    l_pairs = sorted((round(v.x, 3), tuple(v.layers)) for v in live.vias)
    check(t_pairs == l_pairs, '%s: both parse paths read the same layer pairs' % label,
          '%s vs %s' % (t_pairs, l_pairs))

    # The GUI re-reads the live board's vias before every route step
    # (RoutingDialog._sync_pcb_data_from_board) with its own Via(...) call,
    # so it is a third reader that must carry the type.
    from types import SimpleNamespace
    import kicad_routing_plugin.swig_gui as swig_gui
    fake = SimpleNamespace(pcb_data=live, _connectivity_cache={})
    live.vias = []
    real_get_board = pcbnew.GetBoard
    pcbnew.GetBoard = lambda: board
    try:
        swig_gui.RoutingDialog._sync_pcb_data_from_board(fake)
    finally:
        pcbnew.GetBoard = real_get_board
    check(len(live.vias) == 5, '%s gui-sync: the sync re-read five vias' % label,
          'got %d (the method swallows its own errors)' % len(live.vias))
    for v in live.vias:
        key = (round(v.x, 3), round(v.y, 3))
        got = {n for n in COPPER if via_on_layer(v, n)}
        check(got == truth.get(key), '%s gui-sync: via at %s (%s) is on %s'
              % (label, key, v.via_type, sorted(truth.get(key) or ())),
              'model says %s' % sorted(got))


def _recreate(pcbnew, td, path):
    """The plugin puts a via it took off the board back with a NEW PCB_VIA
    (placement Apply re-creates every via of the result; the fanout tab's
    via-nudge; a route tab re-placing one). Each must come back with the
    copper it had: gui_utils.set_via_layers, as every site now calls it."""
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.gui_utils import set_via_layers

    print('\nRE-CREATE')
    src = parse_kicad_pcb(path)
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(len(COPPER))
    bad = pcbnew.BOARD()
    bad.SetCopperLayerCount(len(COPPER))
    for b, right_order in ((board, True), (bad, False)):
        for v in src.vias:
            nv = pcbnew.PCB_VIA(b)
            nv.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(v.x), pcbnew.FromMM(v.y)))
            nv.SetDrill(pcbnew.FromMM(v.drill))
            top, bot = b.GetLayerID(v.layers[0]), b.GetLayerID(v.layers[-1])
            if right_order:
                set_via_layers(nv, top, bot, v.via_type)
            else:
                # The order the sites used to be one SetViaType away from.
                from kicad_parser import pcbnew_viatype_value
                nv.SetLayerPair(top, bot)
                if pcbnew_viatype_value(v.via_type) is not None:
                    nv.SetViaType(pcbnew_viatype_value(v.via_type))
            b.Add(nv)
    out = os.path.join(td, 'recreated.kicad_pcb')
    pcbnew.SaveBoard(out, board)
    again = pcbnew.LoadBoard(out)
    ids = {name: again.GetLayerID(name) for name in COPPER}
    want = {(round(v.x, 3), round(v.y, 3)): {n for n in COPPER if
                                              _on(v, n)} for v in src.vias}
    got = {}
    for t in again.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            got[(round(pcbnew.ToMM(t.GetPosition().x), 3),
                 round(pcbnew.ToMM(t.GetPosition().y), 3))] = \
                {n for n in COPPER if t.IsOnLayer(ids[n])}
    for key, w in sorted(want.items()):
        check(got.get(key) == w, 'RE-CREATE: via at %s comes back on %s'
              % (key, sorted(w)), 'pcbnew says %s' % sorted(got.get(key) or ()))
    # Negative control: the pair first, then the type, spans the whole stack.
    bids = {name: bad.GetLayerID(name) for name in COPPER}
    wrong = sum(1 for t in bad.GetTracks() if t.GetClass() == 'PCB_VIA'
                and {n for n in COPPER if t.IsOnLayer(bids[n])} != want.get(
                    (round(pcbnew.ToMM(t.GetPosition().x), 3),
                     round(pcbnew.ToMM(t.GetPosition().y), 3))))
    check(wrong == 4, 'RE-CREATE control: SetLayerPair before SetViaType '
          'snaps all 4 partial vias to the full stack, so the order is what '
          'the helper is for', 'got %d' % wrong)


def _on(v, layer):
    from geometry_utils import via_on_layer
    return via_on_layer(v, layer)


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import pcbnew
    for _p in ('', 'py_router', 'py_placer', 'py_tools'):
        _d = os.path.join(REPO, _p)
        if _d not in sys.path:
            sys.path.insert(0, _d)
    print('KiCad build: %s' % pcbnew.GetBuildVersion())
    with tempfile.TemporaryDirectory(prefix='t1268_') as td:
        typed = os.path.join(td, 'typed.kicad_pcb')
        _build(pcbnew, typed)
        with open(typed, encoding='utf-8') as f:
            txt = f.read()
        stripped = re.sub(r'\(via\s+(?:blind|buried|micro)(?=\s)', '(via', txt)
        check(stripped != txt and not re.search(r'\(via\s+(?:blind|buried|micro)\s', stripped),
              'the untyped fixture has no type word left')
        _pairs = re.findall(r'\(layers "([^"]+)" "([^"]+)"\)', stripped)
        check(sum(1 for a, b in _pairs if {a, b} != {'F.Cu', 'B.Cu'}) == 4,
              'the untyped fixture still writes 4 partial layer pairs')
        untyped = os.path.join(td, 'untyped.kicad_pcb')
        with open(untyped, 'w', encoding='utf-8') as f:
            f.write(stripped)
        _grade(pcbnew, 'TYPED', typed, expect_partial_pairs=4)
        # The file carries 4 partial pairs (asserted above by the strip); the
        # text parse snaps every one to F.Cu/B.Cu, as pcbnew does.
        _grade(pcbnew, 'UNTYPED', untyped, expect_partial_pairs=0)
        # Every via's pair written BOTTOM FIRST, as this repo's writer wrote
        # zynq7020_som's Type VII through vias: pcbnew reads each top layer
        # first, and the text parse must hold the same ordered pair.
        # (The fixture has no footprints: every two-name `(layers ...)` in it
        # is a via's, and pcbnew writes a via block over several lines.)
        flipped = re.sub(r'\(layers "([^"]+)" "([^"]+)"\)', r'(layers "\2" "\1")', txt)
        check(flipped != txt and re.search(r'\(layers "B\.Cu" "F\.Cu"\)', flipped),
              'the reversed fixture writes a through via B.Cu first')
        reversed_path = os.path.join(td, 'reversed.kicad_pcb')
        with open(reversed_path, 'w', encoding='utf-8') as f:
            f.write(flipped)
        _grade(pcbnew, 'REVERSED', reversed_path, expect_partial_pairs=4)
        _recreate(pcbnew, td, typed)

    print('\n%d failure(s)' % len(FAILURES))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
