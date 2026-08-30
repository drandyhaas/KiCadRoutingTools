#!/usr/bin/env python3
"""#489 §9: --add-teardrops reached pads only; vias got teardrops on NO path.

`add_teardrops_to_pads` was the only teardrop writer and `generate_via_sexpr`
contained zero teardrop references, so track-to-via teardrops -- the standard
mitigation for drill breakout under registration error, and most valuable exactly
where this tool is weakest (a 0.1mm trace meeting a 0.25mm via pad in fine-pitch
escape) -- were never emitted.

Covers:
  * every via gains a teardrop block; the count is reported
  * a via that already has one is left alone (idempotent, no double block)
  * the block lands AFTER (uuid ...), and the boards we write stay parseable by
    us in BOTH the numeric-net (KiCad 9) and named-net (KiCad 10) forms.
    That position used to be load-bearing -- the via regexes required
    layers -> (free)? -> net -> uuid to be CONTIGUOUS, so an earlier insertion
    made our own output unreadable. #748 retired that constraint (each via is
    matched inside its own block, and every field after (layers ...) is read by
    its own token), so this is now a WRITER convention rather than a parser
    requirement. The checks stay: the position is still what KiCad itself
    writes, and a silent move is still worth catching.
  * pads are untouched by the via pass (and vice versa)

KiCad's own acceptance of that position was verified separately against pcbnew
(63/63 vias reported GetTeardropsEnabled() after injection, 0 before).

Run with:  python3 tests/test_489_via_teardrops.py
"""
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, "rust_router"))

from kicad_writer import add_teardrops_to_vias, add_teardrops_to_pads  # noqa: E402
from kicad_parser import extract_vias  # noqa: E402

V9_VIA = '''	(via
		(at 10.0 20.0)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(net 4)
		(uuid "aaaaaaaa-0000-0000-0000-000000000001")
	)
'''

V9_FREE_VIA = '''	(via
		(at 11.0 21.0)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(free yes)
		(net 4)
		(uuid "aaaaaaaa-0000-0000-0000-000000000002")
	)
'''

V10_VIA = '''	(via
		(at 12.0 22.0)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(tenting (front yes) (back yes))
		(net "GND")
		(uuid "aaaaaaaa-0000-0000-0000-000000000003")
	)
'''

ALREADY = '''	(via
		(at 13.0 23.0)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(net 4)
		(uuid "aaaaaaaa-0000-0000-0000-000000000004")
		(teardrops
			(enabled yes)
		)
	)
'''

PAD = '''		(pad "1" smd rect
			(at 0 0)
			(size 0.5 0.5)
			(layers "F.Cu" "F.Paste" "F.Mask")
			(net 4 "GND")
			(uuid "bbbbbbbb-0000-0000-0000-000000000001")
		)
'''


def run():
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    # ----------------------------------------------------- every via gets one
    board = "(kicad_pcb\n" + V9_VIA + V9_FREE_VIA + V10_VIA + ")\n"
    out, n = add_teardrops_to_vias(board)
    check(n == 3, f"expected 3 vias given teardrops, got {n}")
    check(out.count('(teardrops') == 3,
          f"expected 3 teardrop blocks, got {out.count('(teardrops')}")
    check('(enabled yes)' in out, "the injected block must enable teardrops")

    # ------------------------------------------- position: AFTER the uuid token
    for form, name in ((V9_VIA, 'numeric-net'), (V10_VIA, 'named-net')):
        one, _ = add_teardrops_to_vias("(kicad_pcb\n" + form + ")\n")
        uuid_at = one.index('(uuid ')
        td_at = one.index('(teardrops')
        check(td_at > uuid_at,
              f"{name}: teardrops must be inserted AFTER (uuid ...), else this "
              f"repo's own strict via regex stops matching")

    # ------------------- our OWN parser must still read every via after injection
    name_to_id = {'GND': 4}
    before = extract_vias(board, name_to_id)
    after = extract_vias(out, name_to_id)
    check(len(before) == 3, f"fixture sanity: expected 3 vias parsed, got {len(before)}")
    check(len(after) == len(before),
          f"parser lost vias after teardrop injection: {len(before)} -> {len(after)}")
    check(sorted(v.uuid for v in after) == sorted(v.uuid for v in before),
          "the same vias must parse before and after injection")
    check([v.free for v in sorted(after, key=lambda v: v.uuid)] == [False, True, False],
          "the (free yes) flag must survive injection")

    # ----------------------------------------------------------- idempotent
    twice, n2 = add_teardrops_to_vias(out)
    check(n2 == 0, f"a second pass must add nothing, got {n2}")
    check(twice == out, "a second pass must leave the content unchanged")
    once, n3 = add_teardrops_to_vias("(kicad_pcb\n" + ALREADY + ")\n")
    check(n3 == 0, f"a via that already has teardrops must be skipped, got {n3}")
    check(once.count('(teardrops') == 1,
          "an already-teardropped via must not get a second block")

    # ------------------------------------------------- pads and vias don't cross
    mixed = "(kicad_pcb\n" + V9_VIA + "\t(footprint \"F\"\n" + PAD + "\t)\n)\n"
    via_only, nv = add_teardrops_to_vias(mixed)
    check(nv == 1, f"via pass should touch 1 via, got {nv}")
    check(via_only.count('(teardrops') == 1,
          "the via pass must not add teardrops to pads")
    pad_only, npd = add_teardrops_to_pads(mixed)
    check(npd == 1, f"pad pass should touch 1 pad, got {npd}")
    check(pad_only.count('(teardrops') == 1,
          "the pad pass must not add teardrops to vias")
    both, _ = add_teardrops_to_pads(via_only)
    check(both.count('(teardrops') == 2,
          "running both passes should give one block on the via and one on the pad")

    # ------------------------------------------- no via in the file -> no change
    empty, n4 = add_teardrops_to_vias("(kicad_pcb\n)\n")
    check(n4 == 0 and empty == "(kicad_pcb\n)\n",
          "a board with no vias must be returned unchanged")

    # ------------------- every step that writes pad/via copper offers the flag
    # The filed gap: --add-teardrops existed on 3 of 6 CLIs, missing exactly from
    # the two scripts that place the most pad-entry copper and from plane repair.
    import re as _re
    for rel in ("py_router/route.py", "py_router/route_diff.py", "py_router/route_planes.py",
                "py_router/repair_planes.py",
                "py_router/bga_fanout/__init__.py",
                "py_router/qfn_fanout/__init__.py"):
        src = open(os.path.join(ROOT_DIR, rel), encoding="utf-8").read()
        check("--add-teardrops" in src,
              f"{rel} must offer --add-teardrops (#489 §9)")
        check(_re.search(r"add_teardrops\s*=\s*args\.add_teardrops", src)
              or "add_teardrops=add_teardrops" in src,
              f"{rel} must pass the flag through to its writer, not just parse it")

    # The shared GUI checkbox must reach the tabs that grew the flag, or the GUI
    # silently diverges from the CLI (CLAUDE.md parity rule).
    gui = open(os.path.join(ROOT_DIR, "kicad_routing_plugin", "routing_dialog.py"),
               encoding="utf-8").read()
    check(gui.count("'add_teardrops': self.add_teardrops_check.GetValue()") >= 3,
          "the fanout and planes tabs' shared params must carry add_teardrops "
          "alongside the route tab's (found "
          f"{gui.count(chr(39) + 'add_teardrops' + chr(39) + ': self.add_teardrops_check.GetValue()')})")
    # KNOWN IPC GAP, recorded rather than hidden (harness convention: a gate
    # locks in a fixed divergence, it does not pretend the divergence is gone).
    #
    # On the SWIG branch the twin is gui_utils.apply_teardrops_to_board, which
    # sets teardrop properties on PCB_VIA/PCB_TRACK through pcbnew. kipy exposes
    # NO teardrop reader or writer at all (same wall as #489 §8 via protection:
    # tenting/plugging/filling), so there is nothing for the IPC front to call --
    # this is unimplementable here, not merely unimplemented.
    #
    # CONSEQUENCE: with "Add teardrops" checked, a board routed through the CLI
    # comes back with teardrops and the same board routed through the IPC plugin
    # does not. The checkbox still reaches the ENGINE params (asserted above), so
    # nothing silently mis-routes; only the cosmetic/reliability pass is absent.
    #
    # TO CLOSE: a kipy teardrop API (or an IPC-side padstack property write),
    # then an adapter apply_teardrops(board) called where
    # move_copper_graphics_to_silkscreen already is, and turn this back into a
    # hard check.
    _gap = []
    for rel, sym in (("kicad_routing_plugin/gui_utils.py", "def apply_teardrops_to_board"),
                     ("kicad_routing_plugin/fanout_gui.py", "apply_teardrops_to_board"),
                     ("kicad_routing_plugin/planes_gui.py", "apply_teardrops_to_board")):
        src = open(os.path.join(ROOT_DIR, rel), encoding="utf-8").read()
        if sym not in src:
            _gap.append(rel)
    if _gap:
        print("  KNOWN IPC GAP (#489 §9): no GUI teardrop pass in "
              + ", ".join(_gap)
              + " -- kipy has no teardrop writer; CLI boards get teardrops, "
                "IPC-plugin boards do not.")

    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} check(s) FAILED")
        return False
    print("PASS  #489 §9 via teardrops (position, idempotence, parser survival)")
    return True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
