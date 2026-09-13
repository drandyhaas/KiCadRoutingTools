#!/usr/bin/env python3
"""The IPC read path must fill `locked` and `tenting_attrs`, like both of
main's parse paths do.

`build_pcb_data_from_board` (the kipy builder on this branch) filled NEITHER,
on segments or vias -- only footprints got `locked`. Two promises the tree
makes in CLAUDE.md were therefore false on the IPC front:

  * #521: "KiCad-LOCKED copper makes its net never-rippable with NO override."
    `protected_nets.locked_net_names` reads `segments[].locked` /
    `vias[].locked` off PCBData, so with both always False every locked net
    was rip-eligible like any other.
  * #489 s8 / #741: a via the plugin RE-PLACES (the #313 cap nudge, a rip-up,
    a tap relocation) must be handed its own protection spec back. With
    `tenting_attrs` always `{}` there was nothing to hand back -- the
    via-in-pad case (IPC-4761 Type VII filled + capped + plated) silently
    became whatever the board's `(setup ...)` says.

WHY THIS FILE AND NOT A BUILDER TEST. The kipy builder reaches a RUNNING KiCad
over a socket and cannot be driven in process (`fake_ipc_board` serves reads by
re-parsing the file, so driving it there would grade the text parser twice).
The two things the builder now composes are therefore functions --
`kipy_locked` and `kipy_via_protection_attrs` -- and this grades them against
the TEXT PARSER's answer for the same board, which is the oracle main's own
pcbnew path is held to.

    python3 -X utf8 tests/test_521_489_kipy_via_metadata.py
"""
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from kicad_parser import (kipy_locked,                        # noqa: E402
                          kipy_via_protection_attrs,
                          parse_kicad_pcb,
                          via_protection_attrs_from_path)
from run_utils import corpus_boards                            # noqa: E402

FAILS = []


def check(cond, msg):
    if not cond:
        FAILS.append(msg)
    return cond


class _StubProto:
    def __init__(self, locked):
        self.locked = locked


class _StubTrack:
    """A kipy Track: the proto carries `locked`, the WRAPPER does not expose it.

    That asymmetry is the whole reason `kipy_locked` exists -- `Via` has a
    `.locked` property and `Track`/`ArcTrack` do not, while every one of those
    protos has the field. A stub that exposed `.locked` on the wrapper would
    test the easy half only.
    """
    def __init__(self, locked_state):
        self.proto = _StubProto(locked_state)


class _StubVia:
    def __init__(self, uuid, locked=False):
        self.id = type('_Id', (), {'value': uuid})()
        self.locked = locked


def _locked_states():
    from kipy.proto.common.types import LockedState
    return LockedState


# ---------------------------------------------------------------------------
# locked
# ---------------------------------------------------------------------------
def test_kipy_locked_reads_both_shapes():
    """Stubs, and NOT because a corpus arm would be better here: measured, not
    one tracked board carries locked COPPER. All 1393 `(locked yes)` tokens in
    `kicad_files/` sit on FOOTPRINTS (glasgow_revC's 25, for instance), so the
    text parser reports 0 locked segments and 0 locked vias on every one of
    them and would be comparing nothing. The three LockedState values are the
    whole input domain, and they are all exercised below."""
    try:
        LS = _locked_states()
    except Exception as exc:                                    # noqa: BLE001
        print(f"SKIP: kipy not importable ({exc})")
        return
    check(kipy_locked(_StubTrack(LS.LS_LOCKED)) is True,
          "a track whose proto says LS_LOCKED read as unlocked -- #521's "
          "never-rippable promise is exactly this bit")
    check(kipy_locked(_StubTrack(LS.LS_UNLOCKED)) is False,
          "an unlocked track read as locked, which would freeze a net nobody "
          "pinned")
    check(kipy_locked(_StubTrack(LS.LS_UNKNOWN)) is False,
          "LS_UNKNOWN must read as NOT locked: 'the board did not say' is not "
          "'the user pinned this'")
    # The Via arm goes through the wrapper property, not the proto.
    check(kipy_locked(_StubVia('u', locked=True)) is True,
          "a kipy Via's own .locked property was not used")
    check(kipy_locked(_StubVia('u', locked=False)) is False,
          "a kipy Via reported locked when its property says otherwise")


def test_kipy_locked_never_raises_on_a_stranger():
    """Mid-parse is the wrong place to discover a kipy version change: the
    builder would lose the whole board, not one flag."""
    class _Nothing:
        pass
    check(kipy_locked(_Nothing()) is False,
          "kipy_locked raised (or answered non-False) on an object with "
          "neither a property nor a proto")


# ---------------------------------------------------------------------------
# tenting_attrs
# ---------------------------------------------------------------------------
def test_the_protection_scan_agrees_with_the_text_parser():
    """For every corpus board that declares protection on a via, the spec the
    IPC resolver hands back must be the one the text parser reports for the
    via with that uuid."""
    boards = corpus_boards()
    if not boards:
        print("SKIP: git could not list the corpus")
        return
    with_specs, checked = [], 0
    for path in boards:
        specs = via_protection_attrs_from_path(path)
        if not specs:
            continue
        pcb = parse_kicad_pcb(path)
        by_uuid = {v.uuid: v.tenting_attrs for v in pcb.vias if v.uuid}
        name = os.path.basename(path)
        hits = 0
        for uid, spec in specs.items():
            if uid not in by_uuid:
                continue          # a spec on something this build did not keep
            hits += 1
            checked += 1
            got = kipy_via_protection_attrs(_StubVia(uid), specs)
            check(got == by_uuid[uid],
                  f"{name}: via {uid[:8]} -- the IPC resolver said {got}, the "
                  f"text parser says {by_uuid[uid]}")
        if hits:
            with_specs.append(f"{name}({hits})")
    check(checked >= 5,
          f"only {checked} via(s) with a declared spec were resolved across "
          f"the corpus; this claim would be vacuous")
    print(f"  resolved {checked} declared spec(s) on: {', '.join(with_specs)}")


def test_an_unknown_via_inherits_rather_than_guessing():
    """A via added in-session has no entry in a scan of the FILE. It must read
    as unspecified -- `{}` means "inherit the board's (setup ...)", which is
    right for new copper -- and never borrow another via's spec."""
    boards = [b for b in corpus_boards() if via_protection_attrs_from_path(b)]
    if not boards:
        print("SKIP: no corpus board declares via protection")
        return
    specs = via_protection_attrs_from_path(boards[0])
    check(kipy_via_protection_attrs(_StubVia('not-a-real-uuid'), specs) == {},
          "a via absent from the file scan was handed a spec anyway")
    check(kipy_via_protection_attrs(_StubVia('anything'), {}) == {},
          "an empty scan (an unsaved board) produced a non-empty spec")


def test_the_resolver_hands_back_a_COPY():
    """The builder stores what this returns on a Via dataclass. If it were the
    scan's own dict, a consumer mutating one via's spec would rewrite it for
    every via sharing that uuid entry, and for the scan itself."""
    boards = [b for b in corpus_boards() if via_protection_attrs_from_path(b)]
    if not boards:
        print("SKIP: no corpus board declares via protection")
        return
    specs = via_protection_attrs_from_path(boards[0])
    uid = next(iter(specs))
    got = kipy_via_protection_attrs(_StubVia(uid), specs)
    got['tenting'] = '(front no) (back no)'
    again = kipy_via_protection_attrs(_StubVia(uid), specs)
    check(again != got,
          "mutating a returned spec changed the scan, so two vias share one "
          "dict and a re-place can rewrite a via nobody touched")


TESTS_TO_RUN = [
    test_kipy_locked_reads_both_shapes,
    test_kipy_locked_never_raises_on_a_stranger,
    test_the_protection_scan_agrees_with_the_text_parser,
    test_an_unknown_via_inherits_rather_than_guessing,
    test_the_resolver_hands_back_a_COPY,
]


def run():
    for t in TESTS_TO_RUN:
        print(f"--- {t.__name__}")
        t()
    if FAILS:
        for f in FAILS:
            print(f"  FAIL  {f}")
        print(f"\n{len(FAILS)} check(s) FAILED")
        return False
    print("ALL PASS  #521 locked + #489 tenting_attrs on the IPC read path")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)
