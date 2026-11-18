"""parse_kicad_pcb's content-keyed cache: a hit equals a fresh parse, is an
independent object, says what the parse said, and a changed file re-parses."""
import os
import shutil
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

import kicad_parser  # noqa: E402

BOARD = os.path.join(HERE, '..', 'kicad_files', 'esp_prog.kicad_pcb')


@pytest.fixture
def board(tmp_path, monkeypatch):
    monkeypatch.delenv('KRT_PARSE_CACHE', raising=False)
    kicad_parser.clear_parse_cache()
    dst = tmp_path / 'b.kicad_pcb'
    shutil.copyfile(BOARD, dst)
    yield str(dst)
    kicad_parser.clear_parse_cache()


def _fresh(path, monkeypatch):
    monkeypatch.setenv('KRT_PARSE_CACHE', '0')
    try:
        return kicad_parser.parse_kicad_pcb(path)
    finally:
        monkeypatch.delenv('KRT_PARSE_CACHE')


def test_hit_equals_fresh_parse_and_is_independent(board, monkeypatch):
    first = kicad_parser.parse_kicad_pcb(board)
    hit = kicad_parser.parse_kicad_pcb(board)
    assert hit is not first
    assert repr(hit) == repr(_fresh(board, monkeypatch))
    # Mutating one copy must not reach the next parse.
    hit.segments.clear()
    hit.nets.clear()
    again = kicad_parser.parse_kicad_pcb(board)
    assert again.segments and again.nets
    assert repr(again) == repr(first)


def test_changed_file_reparses(board, monkeypatch):
    before = kicad_parser.parse_kicad_pcb(board)
    with open(board, encoding='utf-8') as f:
        text = f.read()
    i = text.rindex(')')
    seg = ('  (segment (start 1 1) (end 2 2) (width 0.2) (layer "F.Cu") '
           '(net 0) (uuid "00000000-0000-0000-0000-00000000c0de"))\n')
    with open(board, 'w', encoding='utf-8') as f:
        f.write(text[:i] + seg + text[i:])
    after = kicad_parser.parse_kicad_pcb(board)
    assert len(after.segments) == len(before.segments) + 1
    assert repr(after) == repr(_fresh(board, monkeypatch))


def test_layer_arguments_are_part_of_the_key(board):
    a = kicad_parser.parse_kicad_pcb(board)
    b = kicad_parser.parse_kicad_pcb(board, guide_layer='User.3')
    assert len(kicad_parser._PARSE_CACHE) == 2
    assert repr(a.nets) == repr(b.nets)


def test_hit_repeats_what_the_parse_printed(board, capsys, monkeypatch):
    def noisy(path, guide_layer, keepout_layer, _real=kicad_parser._parse_kicad_pcb):
        print('NOTE: out')
        print('WARNING: err', file=sys.stderr)
        return _real(path, guide_layer, keepout_layer)
    monkeypatch.setattr(kicad_parser, '_parse_kicad_pcb', noisy)
    kicad_parser.parse_kicad_pcb(board)
    first = capsys.readouterr()
    kicad_parser.parse_kicad_pcb(board)
    second = capsys.readouterr()
    assert first.out == second.out == 'NOTE: out\n'
    assert first.err == second.err
    assert first.err.startswith('WARNING: err\n')


def test_off_switch(board, monkeypatch):
    monkeypatch.setenv('KRT_PARSE_CACHE', '0')
    kicad_parser.parse_kicad_pcb(board)
    assert not kicad_parser._PARSE_CACHE
