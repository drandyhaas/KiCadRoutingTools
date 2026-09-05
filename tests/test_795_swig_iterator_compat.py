"""KiCad built against a current SWIG cannot iterate a board (#795).

    Failed to read board data:
    'SwigPyIterator' object has no attribute 'next'

KiCad hand-writes the `__iter__` of the container classes whose items need a
`Cast()` -- `TRACKS` (`board.GetTracks()`), `DRAWINGS` (`board.GetDrawings()`,
`footprint.GraphicalItems()`) and `VECTOR_SHAPEPTR` -- and those generators step
the iterator with the PYTHON-2 spelling:

    def __iter__(self):
        it = self.iterator()
        try:
            while True:
                item = it.next()      # <-- py2; SWIG dropped it with py2
                yield item.Cast()
        except StopIteration:
            return

`SwigPyIterator` carried both `next()` and `__next__()` until SWIG 4.5.0
(2026-08-06) removed py2 support; only `__next__` survives. So the FIRST thing
that walks a board's tracks or drawings dies -- for us that is
`build_pcb_data_from_board`, i.e. every GUI run, on the very first click.

It is a property of the SWIG that BUILT the KiCad package, not of KiCad's version
or of the board: the reporter's KiCad 10.0.6 on Arch (swig 4.5.0-1 since
2026-08-06) fails where the same 10.0.6 built against 4.4.x works, which is why
it presents as "the last few KiCad builds broke it".

The fix restores the alias on the class (`py_router/swig_compat.py`), which
repairs every call site at once -- ours, KiCad's own python helpers, any other
plugin in the process -- rather than rewriting ~40 `for t in board.GetTracks()`
loops into index walks and missing one.

WHY THE TESTS LOOK LIKE THIS: they inject a FAKE `pcbnew` shaped like the broken
one, because the bug cannot be reproduced on a KiCad whose SWIG still ships the
alias -- which is every KiCad this repo is developed against. And they assert
BEHAVIOUR (the fake's class comes back iterable after the import) rather than
grepping the sources for `patch_swig_iterators`, which would pass just as
happily with the call sitting in a comment (the PR692 lesson).

No pcbnew, no wx, no subprocess, milliseconds. When a real pcbnew IS importable
the last test additionally round-trips a real board through the real parser.
"""
import os
import sys
import types

#: run_all's integration proxy is the literal substring 'subprocess' anywhere in
#: the source -- which this file only ever says in PROSE (it starts none). It is
#: milliseconds and shells out to nothing, so opt back into --fast; otherwise the
#: #795 gate silently stops running in the mode CI uses most.
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from swig_compat import patch_swig_iterators  # noqa: E402


# --------------------------------------------------------------------------
# A stand-in for the broken pcbnew: an iterator with only `__next__`, and a
# container whose __iter__ is KiCad's -- copied verbatim from pcbnew.py.
# --------------------------------------------------------------------------
def _make_fake_pcbnew(with_next_alias=False, with_dunder_next=True):
    class SwigPyIterator(object):
        def __init__(self, items):
            self._it = iter(items)

        def _step(self):
            return next(self._it)      # raises StopIteration at the end

        if with_dunder_next:
            __next__ = _step
        if with_next_alias:
            next = _step

    class TRACKS(object):
        """KiCad's own container: __iter__ written by hand, py2 spelling."""
        def __init__(self, items):
            self._items = list(items)

        def iterator(self):
            return SwigPyIterator(self._items)

        def __iter__(self):
            it = self.iterator()
            try:
                while True:
                    item = it.next()   # noqa: B305 -- KiCad's code, not ours
                    yield item
            except StopIteration:
                return

    mod = types.ModuleType('pcbnew')
    mod.SwigPyIterator = SwigPyIterator
    mod.TRACKS = TRACKS
    # action_plugin subclasses this at module scope.
    mod.ActionPlugin = type('ActionPlugin', (object,), {})
    return mod


def test_the_bug_is_real_in_the_stand_in():
    """Guard the guard: without the fix the fake reproduces the exact message."""
    pcbnew = _make_fake_pcbnew()
    try:
        list(pcbnew.TRACKS([1, 2, 3]))
    except AttributeError as e:
        assert "'SwigPyIterator' object has no attribute 'next'" in str(e), e
    else:
        raise AssertionError(
            "the stand-in did NOT reproduce #795 -- it cannot prove the fix")


def test_patch_restores_iteration():
    pcbnew = _make_fake_pcbnew()
    assert patch_swig_iterators(pcbnew) is True, "patch reported nothing to do"
    assert list(pcbnew.TRACKS([1, 2, 3])) == [1, 2, 3]


def test_patch_is_idempotent_and_inert_where_swig_is_fine():
    """A second call must not re-wrap, and an OLD SWIG must be left alone."""
    pcbnew = _make_fake_pcbnew()
    assert patch_swig_iterators(pcbnew) is True
    assert patch_swig_iterators(pcbnew) is False, "second call claimed a change"
    assert list(pcbnew.TRACKS([7])) == [7]

    old = _make_fake_pcbnew(with_next_alias=True)
    original = old.SwigPyIterator.next
    assert patch_swig_iterators(old) is False, "patched a SWIG that was fine"
    assert old.SwigPyIterator.next is original, "clobbered the real alias"


def test_patch_declines_what_it_cannot_fix():
    """Never raise on a module it does not understand -- it runs at import."""
    assert patch_swig_iterators(types.ModuleType('empty')) is False
    assert patch_swig_iterators(_make_fake_pcbnew(with_dunder_next=False)) is False

    class Frozen(object):        # SWIG -builtin: an immutable extension type
        __slots__ = ()
    frozen = types.ModuleType('frozen')
    frozen.SwigPyIterator = type('SwigPyIterator', (tuple,), {'__next__': None})
    assert patch_swig_iterators(frozen) is False


# --------------------------------------------------------------------------
# Wiring: importing the modules that touch a live board must ITSELF fix the
# process. Asserted by importing them against a broken fake pcbnew.
# --------------------------------------------------------------------------
def _import_against_broken_pcbnew(module_name, extra_stubs=()):
    """Import `module_name` with a BROKEN fake pcbnew installed; report whether
    the import repaired it. Restores sys.modules whatever happens."""
    package = module_name.split('.')[0]
    saved = {k: sys.modules.get(k)
             for k in ('pcbnew', module_name, package, *extra_stubs)}
    fake = _make_fake_pcbnew()
    try:
        sys.modules['pcbnew'] = fake
        for name in extra_stubs:
            sys.modules.setdefault(name, types.ModuleType(name))
        sys.modules.pop(module_name, None)
        sys.modules.pop(package, None)
        __import__(module_name)
        try:
            return list(fake.TRACKS([1, 2]))
        except AttributeError:
            return None      # still broken -- let the caller say why
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


def test_importing_the_parser_repairs_the_process():
    """kicad_parser is imported by every shipped module that walks a board."""
    assert _import_against_broken_pcbnew('kicad_parser') == [1, 2], (
        "importing kicad_parser left board iteration broken -- the #795 shim "
        "call at its module top is missing or no longer runs")


def test_importing_exact_fill_repairs_the_process():
    """Its refill SUBPROCESS imports this module, not kicad_parser, and calls
    repin_netcodes_from_file() -> board.GetTracks(). That failure is swallowed
    ('net repin skipped'), so it degrades silently rather than reporting."""
    assert _import_against_broken_pcbnew('kicad_exact_fill') == [1, 2], (
        "importing kicad_exact_fill left board iteration broken -- #795 shim "
        "call missing")


def test_importing_the_gui_entry_repairs_the_process():
    """IPC PORT. Upstream this imports `kicad_routing_plugin.action_plugin`,
    the SWIG entry, and asserts the #795 shim ran: that entry walked
    GetTracks() in _get_selected_net_names() and build_pcb_data_from_board(),
    which is what printed "Failed to read board data".

    The port DELETED that entry. `ipc_entry` reaches the board over kipy and
    never imports pcbnew, so there is no SWIG iterator for it to break -- the
    upstream assertion has no subject here. Assert the REASON instead of
    deleting the test, so the day the IPC entry grows a pcbnew import this
    fails and someone re-instates the shim check.

    The engine modules that DO use pcbnew keep their own coverage: see
    test_importing_kicad_exact_fill_repairs_the_process above.
    """
    entry = os.path.join(ROOT, 'kicad_routing_plugin', 'ipc_entry.py')
    assert os.path.isfile(entry), (
        "ipc_entry.py is missing -- this branch's GUI entry moved again, so "
        "this test no longer knows what it is guarding")
    src = open(entry, encoding='utf-8').read()
    assert 'pcbnew' not in src, (
        "the IPC entry now references pcbnew, so it CAN hit the #795 broken "
        "SwigPyIterator. Restore the upstream check against this module: "
        "_import_against_broken_pcbnew('kicad_routing_plugin.ipc_entry', "
        "extra_stubs=('wx',)) == [1, 2]")


def test_real_pcbnew_round_trip():
    """Only under a KiCad python: strip the alias off the REAL pcbnew, then
    confirm the real parser reads a real board through the shim."""
    try:
        import pcbnew
    except ImportError:
        print("  (skipped real-board round trip: no pcbnew on this python)")
        return
    import kicad_parser
    board_path = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
    if not os.path.exists(board_path):
        print("  (skipped real-board round trip: fixture missing)")
        return
    board = pcbnew.LoadBoard(board_path)
    reference = kicad_parser.build_pcb_data_from_board(board)
    assert len(reference.segments) > 0, "fixture has no tracks to iterate"

    had_alias = getattr(pcbnew.SwigPyIterator, 'next', None)
    try:
        if had_alias is not None:
            del pcbnew.SwigPyIterator.next        # simulate a current SWIG
        try:
            kicad_parser.build_pcb_data_from_board(board)
        except AttributeError as e:
            assert 'next' in str(e), e            # the #795 failure, as shipped
        else:
            raise AssertionError(
                "stripping the alias did NOT break the parse -- this python's "
                "pcbnew does not reproduce #795, so the check below is vacuous")
        assert patch_swig_iterators(pcbnew) is True
        after = kicad_parser.build_pcb_data_from_board(board)
    finally:
        if had_alias is not None:
            pcbnew.SwigPyIterator.next = had_alias
    assert (len(after.segments), len(after.vias), len(after.footprints)) == \
           (len(reference.segments), len(reference.vias),
            len(reference.footprints)), "shimmed parse differs from the native one"
    print("  (real board: %d segments, %d vias -- identical through the shim)"
          % (len(after.segments), len(after.vias)))


def main():
    test_the_bug_is_real_in_the_stand_in()
    test_patch_restores_iteration()
    test_patch_is_idempotent_and_inert_where_swig_is_fine()
    test_patch_declines_what_it_cannot_fix()
    test_importing_the_parser_repairs_the_process()
    test_importing_exact_fill_repairs_the_process()
    test_importing_the_gui_entry_repairs_the_process()
    test_real_pcbnew_round_trip()
    print("PASS: SwigPyIterator.next restored for current-SWIG KiCad (#795)")


if __name__ == '__main__':
    main()
