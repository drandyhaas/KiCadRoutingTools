#!/usr/bin/env python3
"""A KiCad 5 (or older) board is refused, never parsed into an empty board.

Before file format 20201115 ("module -> footprint", KiCad 5.99) a board wrote
its parts as (module ...) blocks with bare net names. kicad_parser reads
neither, so such a file parsed SILENTLY as a board with no footprints and no
nets, and every CLI then reported success on nothing. The corpus never showed
it -- its 19 KiCad 4/5 sources are converted by a pcbnew round trip before
anything parses them -- but a user running the CLI on an old board (a 2021
recreation of a 1980 pinball board, say) got exactly that. The GUI is fine: it
reads the board through pcbnew.

Pinned here:
  1. every pre-20201115 version is refused, with a message that says what the
     file is and what to do (save it in KiCad 6+, or use the plugin);
  2. 20201115 itself still parses -- the cutoff is KiCad's own;
  3. the refusal is not a formality: the same text parsed past it has no
     footprints;
  4. the CLIs refuse CLEANLY -- an ERROR line, exit 1, no traceback;
  5. the (locked) spelling files used from 20210108 to 20210423 reads as a
     lock, and the seeder's unlock removes it.

    python3 tests/test_kicad5_boards_refused.py
"""
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, TESTS)

KICAD5_BOARD = """(kicad_pcb (version 20171130) (host pcbnew 5.1.9)
  (general (thickness 1.6) (drawings 4) (tracks 0) (zones 0) (modules 1) (nets 2))
  (page A4)
  (layers
    (0 F.Cu signal)
    (31 B.Cu signal)
    (44 Edge.Cuts user)
  )
  (net 0 "")
  (net 1 GND)
  (module Resistor_SMD:R_0805 (layer F.Cu) (tedit 5B36C52B) (tstamp 5E3F1A2B)
    (at 10 10)
    (fp_text reference R1 (at 0 -1.65) (layer F.SilkS) (effects (font (size 1 1) (thickness 0.15))))
    (pad 1 smd roundrect (at -0.95 0) (size 0.9 1.4) (layers F.Cu F.Paste F.Mask) (roundrect_rratio 0.25) (net 1 GND))
    (pad 2 smd roundrect (at 0.95 0) (size 0.9 1.4) (layers F.Cu F.Paste F.Mask) (roundrect_rratio 0.25) (net 1 GND))
  )
  (gr_line (start 0 0) (end 20 0) (layer Edge.Cuts) (width 0.1))
  (gr_line (start 20 0) (end 20 20) (layer Edge.Cuts) (width 0.1))
  (gr_line (start 20 20) (end 0 20) (layer Edge.Cuts) (width 0.1))
  (gr_line (start 0 20) (end 0 0) (layer Edge.Cuts) (width 0.1))
)
"""

FOOTPRINT_ERA_BOARD = """(kicad_pcb (version %d) (generator pcbnew)
  (general (thickness 1.6))
  (paper "A4")
  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (44 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "GND")
  (footprint "Resistor_SMD:R_0805" (layer "F.Cu") %s(tedit 5B36C52B) (tstamp 0a1b2c3d-1111-2222-3333-444455556666)
    (at 10 10)
    (fp_text reference "R1" (at 0 -1.65) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd roundrect (at -0.95 0) (size 0.9 1.4) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25) (net 1 "GND"))
    (pad "2" smd roundrect (at 0.95 0) (size 0.9 1.4) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25) (net 1 "GND"))
  )
  (gr_line (start 0 0) (end 20 0) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 20 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 20 20) (end 0 20) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 0 20) (end 0 0) (layer "Edge.Cuts") (width 0.1))
)
"""

REASON = 'is a KiCad 5 (or older) board'


def _write(tmp, name, text):
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(text)
    return path


def test_old_versions_refused(tmp):
    from kicad_parser import parse_kicad_pcb, UnsupportedBoardFormat
    for v in (4, 20170123, 20171130, 20201114):
        path = _write(tmp, f'v{v}.kicad_pcb',
                      KICAD5_BOARD.replace('(version 20171130)', f'(version {v})'))
        try:
            parse_kicad_pcb(path)
        except UnsupportedBoardFormat as ex:
            msg = str(ex)
            assert REASON in msg and 'KiCad 6' in msg and 'plugin' in msg, msg
            assert str(v) in msg, f'the message names the file format: {msg}'
        else:
            raise AssertionError(f'version {v} parsed instead of being refused')
    print('  refused: versions 4, 20170123, 20171130 and 20201114, each saying what to do')


def test_cutoff_still_parses(tmp):
    from kicad_parser import parse_kicad_pcb, FIRST_SUPPORTED_BOARD_VERSION
    assert FIRST_SUPPORTED_BOARD_VERSION == 20201115
    pcb = parse_kicad_pcb(_write(tmp, 'v20201115.kicad_pcb',
                                 FOOTPRINT_ERA_BOARD % (20201115, '')))
    assert list(pcb.footprints) == ['R1'] and len(pcb.footprints['R1'].pads) == 2
    print('  20201115 (the first footprint-era format) parses')


def test_refusal_prevents_an_empty_board():
    from kicad_parser import extract_footprints_and_pads, extract_nets
    nets, name_to_id = extract_nets(KICAD5_BOARD)
    fps, _ = extract_footprints_and_pads(KICAD5_BOARD, nets, name_to_id)
    assert not fps, f'the KiCad 5 text now yields footprints {list(fps)}: revisit the cutoff'
    print('  without the refusal the KiCad 5 board reads as 0 footprints')


def test_clis_refuse_cleanly(tmp):
    from run_utils import check
    board = _write(tmp, 'kicad5_cli.kicad_pcb', KICAD5_BOARD)
    for argv in ([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route.py'),
                  board, os.path.join(tmp, 'out.kicad_pcb')],
                 [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'), board]):
        r = check(argv, refuse=REASON, code=1)
        assert 'ERROR: ' in (r.stdout + r.stderr)
    print('  route.py and check_drc.py refuse with an ERROR line, exit 1, no traceback')


def test_paren_locked(tmp):
    from kicad_parser import parse_kicad_pcb
    from placement.parser import extract_locked_refs
    from placement.seeder import stamp_locked, stamp_unlocked
    path = _write(tmp, 'paren_lock.kicad_pcb', FOOTPRINT_ERA_BOARD % (20210228, '(locked) '))
    assert parse_kicad_pcb(path).footprints['R1'].locked
    assert extract_locked_refs(path) == {'R1'}
    assert stamp_locked(path, ['R1']) == 0, '(locked) is already a lock'
    assert stamp_unlocked(path, ['R1']) == 1
    assert '(locked)' not in open(path, encoding='utf-8').read()
    assert not parse_kicad_pcb(path).footprints['R1'].locked
    print('  (locked), the 2021-nightly spelling, reads, stamps and unlocks')


def main():
    tmp = tempfile.mkdtemp(prefix='kicad5_')
    try:
        test_old_versions_refused(tmp)
        test_cutoff_still_parses(tmp)
        test_refusal_prevents_an_empty_board()
        test_clis_refuse_cleanly(tmp)
        test_paren_locked(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('OK: pre-KiCad-6 boards are refused, loudly and cleanly')


if __name__ == '__main__':
    main()
