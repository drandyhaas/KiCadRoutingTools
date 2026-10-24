#!/usr/bin/env python3
"""#1232 parity gate for solder-mask margins: both parse paths, plus native pcbnew as an oracle.

`parse_kicad_pcb` (file text) and `build_pcb_data_from_board` (live pcbnew)
each read `Pad.mask_margin`, `Footprint.mask_margin` and
`BoardInfo.pad_to_mask_clearance`, and `mask_apertures.resolve_mask_margin`
resolves them. On every tracked board, and on synthetic boards for what the
corpus does not carry (an explicit 0 in a 20240108 and a 20240202 file, a
clamped negative margin, a copper-less pad, a through-hole pad):

- **Parity.** compare_pcb_data reports no mask difference between the paths.
- **A native oracle.** For every pad on a mask layer, the resolved margin from
  EACH path equals pcbnew's own `PAD.GetSolderMaskExpansion(layer)`.
- **Witnesses.** esp_prog carries a board, a footprint and a pad margin, and
  glasgow a 0.375 mm pad margin; a parser that read none would pass the parity
  arm and fail here.

Needs pcbnew; re-execs into KiCad's python automatically. Exits 2 when no
pcbnew python is found, rather than 0.

    python3 -X utf8 tests/gui_parity/test_1232_mask_parity.py
"""
import glob
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]


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


FAILURES = []


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


_SYN = """(kicad_pcb (version {ver}) (generator "pcbnew") (general (thickness 1.6))
 (paper "A4") (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (39 "F.Mask" user)
 (38 "B.Mask" user) (44 "Edge.Cuts" user))
 (setup (pad_to_mask_clearance 0.05))
 (net 0 "") (net 1 "/A")
 (footprint "L:P" (layer "F.Cu") (at 20 20)
  (property "Reference" "U1" (at 0 0 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
  (solder_mask_margin 0)
  (pad "1" smd rect (at 0 0) (size 1 0.5) (layers "F.Cu" "F.Mask") (net 1 "/A"))
  (pad "2" smd rect (at 3 0) (size 1 0.5) (layers "F.Cu" "F.Mask") (net 1 "/A") (solder_mask_margin 0))
  (pad "3" smd rect (at 6 0) (size 1 0.5) (layers "F.Cu" "F.Mask") (net 1 "/A") (solder_mask_margin -0.4))
  (pad "4" smd rect (at 9 0) (size 1 0.5) (layers "F.Mask") (solder_mask_margin 0.1))
  (pad "5" thru_hole circle (at 12 0) (size 1.2 1.2) (drill 0.6) (layers "*.Cu" "*.Mask") (net 1 "/A")
   (solder_mask_margin 0.08))))
"""


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
    from kicad_parser import parse_kicad_pcb, build_pcb_data_from_board, compare_pcb_data
    from mask_apertures import resolve_mask_margin, pad_mask_layers
    from run_utils import corpus_boards

    print('KiCad build: %s' % pcbnew.GetBuildVersion())
    boards = corpus_boards()
    check(len(boards) >= 20, 'the tracked corpus is visible (%d boards)' % len(boards))
    syn_dir = tempfile.mkdtemp(prefix='krt1232g_')
    for ver in ('20240108', '20240202'):
        p = os.path.join(syn_dir, 'syn_%s.kicad_pcb' % ver)
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(_SYN.format(ver=ver))
        boards.append(p)

    seen = {'board': 0.0, 'footprint': 0.0, 'pad': 0.0}
    lids = {'F.Mask': pcbnew.F_Mask, 'B.Mask': pcbnew.B_Mask}
    for path in boards:
        name = os.path.basename(path)
        f = parse_kicad_pcb(path)
        board = pcbnew.LoadBoard(path)
        g = build_pcb_data_from_board(board)
        diffs = [d for d in compare_pcb_data(g, f) if 'mask' in d.lower()]
        check(not diffs, '%s: the parse paths agree on every mask margin' % name,
              '; '.join(diffs[:3]))
        live = {key: list(fp.Pads()) for fp, key in zip(board.GetFootprints(), list(g.footprints))}
        wrong, n = [], 0
        for side_name, pd in (('file', f), ('board', g)):
            seen['board'] = max(seen['board'], abs(pd.board_info.pad_to_mask_clearance))
            for key, fp in pd.footprints.items():
                if fp.mask_margin:
                    seen['footprint'] = max(seen['footprint'], abs(fp.mask_margin))
                for pad in fp.pads:
                    if pad.mask_margin:
                        seen['pad'] = max(seen['pad'], abs(pad.mask_margin))
                    cands = [lp for lp in live.get(key, []) if lp.GetNumber() == pad.pad_number]
                    for layer in pad_mask_layers(pad):
                        natives = {round(lp.GetSolderMaskExpansion(lids[layer]) / 1e6, 6)
                                   for lp in cands if lp.IsOnLayer(lids[layer])}
                        if not natives:
                            continue
                        n += 1
                        mine = round(resolve_mask_margin(pad, fp, pd.board_info), 6)
                        if not any(abs(mine - v) <= 1.5e-6 for v in natives):
                            wrong.append('%s %s.%s %s: %s vs pcbnew %s' % (
                                side_name, key, pad.pad_number, layer, mine, sorted(natives)))
        check(not wrong, '%s: every pad mask margin equals pcbnew GetSolderMaskExpansion (%d)'
              % (name, n), '; '.join(wrong[:3]))
    check(seen['board'] > 0 and seen['footprint'] > 0 and seen['pad'] >= 0.375,
          'witness: board, footprint and pad margins (glasgow 0.375) were read',
          str(seen))

    print('\n%d failure(s)' % len(FAILURES))
    for fl in FAILURES:
        print('  FAILED: %s' % fl)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
