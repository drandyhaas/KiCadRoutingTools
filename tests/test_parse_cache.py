"""parse_kicad_pcb's content-keyed cache: a hit equals a fresh parse, is an
independent object, says what the parse said, and a changed file re-parses."""
import contextlib
import io
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

import kicad_parser  # noqa: E402

BOARD = os.path.join(HERE, '..', 'kicad_files', 'esp_prog.kicad_pcb')


@contextlib.contextmanager
def _board():
    """A private copy of the board, with the cache on and empty."""
    saved = os.environ.pop('KRT_PARSE_CACHE', None)
    kicad_parser.clear_parse_cache()
    tmp = tempfile.mkdtemp(prefix='t_parse_cache_')
    try:
        dst = os.path.join(tmp, 'b.kicad_pcb')
        shutil.copyfile(BOARD, dst)
        yield dst
    finally:
        kicad_parser.clear_parse_cache()
        if saved is None:
            os.environ.pop('KRT_PARSE_CACHE', None)
        else:
            os.environ['KRT_PARSE_CACHE'] = saved
        shutil.rmtree(tmp, ignore_errors=True)


def _fresh(path):
    os.environ['KRT_PARSE_CACHE'] = '0'
    try:
        return kicad_parser.parse_kicad_pcb(path)
    finally:
        del os.environ['KRT_PARSE_CACHE']


def _quiet(fn, *args, **kwargs):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        result = fn(*args, **kwargs)
    return result, out.getvalue(), err.getvalue()


def test_hit_equals_fresh_parse_and_is_independent():
    with _board() as board:
        first = kicad_parser.parse_kicad_pcb(board)
        hit = kicad_parser.parse_kicad_pcb(board)
        assert hit is not first
        assert repr(hit) == repr(_fresh(board))
        # Mutating one copy must not reach the next parse.
        hit.segments.clear()
        hit.nets.clear()
        again = kicad_parser.parse_kicad_pcb(board)
        assert again.segments and again.nets
        assert repr(again) == repr(first)


def test_changed_file_reparses():
    with _board() as board:
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
        assert repr(after) == repr(_fresh(board))


def test_layer_arguments_are_part_of_the_key():
    with _board() as board:
        a = kicad_parser.parse_kicad_pcb(board)
        b = kicad_parser.parse_kicad_pcb(board, guide_layer='User.3')
        assert len(kicad_parser._PARSE_CACHE) == 2
        assert repr(a.nets) == repr(b.nets)


def test_hit_repeats_what_the_parse_printed():
    real = kicad_parser._parse_kicad_pcb

    def noisy(path, guide_layer, keepout_layer):
        print('NOTE: out')
        print('WARNING: err', file=sys.stderr)
        return real(path, guide_layer, keepout_layer)
    with _board() as board:
        kicad_parser._parse_kicad_pcb = noisy
        try:
            _, out1, err1 = _quiet(kicad_parser.parse_kicad_pcb, board)
            _, out2, err2 = _quiet(kicad_parser.parse_kicad_pcb, board)
        finally:
            kicad_parser._parse_kicad_pcb = real
        assert out1 == out2 == 'NOTE: out\n'
        assert err1 == err2
        assert err1.startswith('WARNING: err\n')


def test_off_switch():
    with _board() as board:
        os.environ['KRT_PARSE_CACHE'] = '0'
        try:
            kicad_parser.parse_kicad_pcb(board)
        finally:
            del os.environ['KRT_PARSE_CACHE']
        assert not kicad_parser._PARSE_CACHE


TESTS = [
    test_hit_equals_fresh_parse_and_is_independent,
    test_changed_file_reparses,
    test_layer_arguments_are_part_of_the_key,
    test_hit_repeats_what_the_parse_printed,
    test_off_switch,
]


if __name__ == '__main__':
    failed = 0
    for t in TESTS:
        try:
            _quiet(t)
            print(f"PASS  {t.__name__}")
        except Exception as e:  # noqa: BLE001 - report every test
            failed += 1
            print(f"FAIL  {t.__name__}: {type(e).__name__}: {e}")
    print("FAIL" if failed else "PASS")
    sys.exit(1 if failed else 0)
