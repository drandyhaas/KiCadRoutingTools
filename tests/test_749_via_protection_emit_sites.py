#!/usr/bin/env python3
"""#749: the other via emit sites re-stamped a re-placed via, or ignored the
board's convention for a new one.

#741 fixed the placement via-nudge. The same two defects existed elsewhere:

  A. `add_tracks_and_vias_to_pcb` had no `tenting_attrs` parameter AT ALL, and
     it also takes `remove_vias` -- so a via removed and re-added round-tripped
     through it and came back re-stamped. Its callers split cleanly: fanout and
     route_planes emit NEW vias (which want the board's prevailing convention),
     while route.py's #666 re-emit of model copper the written file lost is a
     genuinely PRE-EXISTING via (which wants its own spec back, and inheritance
     when it had none).
  B. Three `kicad_oracle` sites passed no spec, so every via the oracle welded
     got the hardcoded front+back tenting whatever the board said.
  C. `plane_io.restore_failed_reroute_nets` carried the spec but not the
     DECISION: it is a restore, so a via that carried no spec was inheriting the
     board's `(setup ...)` and must keep inheriting rather than gain a token.
  D. All of them resolved `net_name` from a lookup that misses on net 0, which
     emitted a numeric ref next to a spec -- a via `extract_vias` could not read
     back at all (#748).

Latent on the tracked corpus (only two boards carry per-via specs, and all 206
are exactly DEFAULT_VIA_TENTING, so today's output and the correct output are
the same string). It bites a hackrf-class board: IPC-4761 Type VII, filled +
capped + plated.

Run with:  python3 tests/test_749_via_protection_emit_sites.py
"""
import ast
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import extract_vias  # noqa: E402
from kicad_writer import (DEFAULT_VIA_TENTING, add_tracks_and_vias_to_pcb,  # noqa: E402
                          via_net_name)

# A board whose own vias all declare a NON-default convention: hackrf_one's
# shape (everything explicitly off). Used here as the thing an added via must
# NOT pick up -- see A1. (Measured since: hackrf_one's own tokens are KiCad 10's
# upgrade migration writing out the factory defaults, not designer intent; its
# pristine source has 0/498 vias declaring anything.)
HOUSE_SPEC = {'covering': '(front no) (back no)', 'plugging': '(front no) (back no)'}
TYPE_VII = {'tenting': '(front no) (back no)',
            'covering': '(front yes) (back yes)',
            'plugging': '(front yes) (back yes)',
            'capping': 'yes', 'filling': 'yes'}

BOARD = '''(kicad_pcb
\t(version 20241229)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6))
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(44 "Edge.Cuts" user)
\t)
\t(setup (pad_to_mask_clearance 0))
\t(net 0 "")
\t(net 1 "/SIG")
\t(gr_line (start 0 0) (end 40 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "11111111-1111-1111-1111-111111111111"))
\t(via
\t\t(at 5.0 5.0)
\t\t(size 0.6)
\t\t(drill 0.3)
\t\t(layers "F.Cu" "B.Cu")
\t\t(covering (front no) (back no))
\t\t(plugging (front no) (back no))
\t\t(net "/SIG")
\t\t(uuid "aaaaaaaa-0000-0000-0000-000000000001")
\t)
)
'''

N2N = {0: '', 1: '/SIG'}


def _emit(tmp, vias):
    """Run add_tracks_and_vias_to_pcb and return the vias it wrote, minus the
    board's own."""
    src = os.path.join(tmp, 'in.kicad_pcb')
    dst = os.path.join(tmp, 'out.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    ok = add_tracks_and_vias_to_pcb(src, dst, [], vias, net_id_to_name=N2N)
    assert ok, "add_tracks_and_vias_to_pcb reported failure"
    with open(dst, encoding='utf-8') as f:
        out = f.read()
    got = [v for v in extract_vias(out, {'': 0, '/SIG': 1})
           if (v.x, v.y) != (5.0, 5.0)]
    return out, got


def run():
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    tmp = tempfile.mkdtemp(prefix='t749_')
    try:
        # --- A1: a NEW via emits NOTHING and inherits the board's
        # `(setup ...)`. It gets neither the writer's old hardcoded front+back
        # tenting NOR the board's prevailing per-via spec: pcbnew leaves a via
        # it adds at *_MODE_FROM_BOARD, and KiCad writes no token for such a
        # via, so anything we stamp turns an inheriting via into an OVERRIDE.
        # Measured cost of getting this wrong: three corpus boards declare
        # `(tenting (front no) (back no))` board-wide and had every added via
        # stamped tented.
        out, got = _emit(tmp, [{'x': 10.0, 'y': 10.0, 'size': 0.6, 'drill': 0.3,
                                'layers': ['F.Cu', 'B.Cu'], 'net_id': 1}])
        check(len(got) == 1, f"A1: emitted {len(got)} vias, expected 1")
        if got:
            check(got[0].tenting_attrs == {},
                  f"A1: a NEW via got {got[0].tenting_attrs}, expected NO "
                  f"protection token at all so it inherits the board policy")
            check(got[0].tenting_attrs != DEFAULT_VIA_TENTING,
                  "A1: the new via was stamped with the hardcoded front+back "
                  "tenting, which is what #489 s8 set out to stop")
            check(got[0].tenting_attrs != HOUSE_SPEC,
                  f"A1: the new via was stamped with the board's PREVAILING "
                  f"per-via spec {HOUSE_SPEC}; that is still an override, and "
                  f"on a board whose setup disagrees it is the wrong one")

        # --- A2: a PRE-EXISTING via carries its own spec back.
        _, got = _emit(tmp, [{'x': 11.0, 'y': 11.0, 'size': 0.6, 'drill': 0.3,
                              'layers': ['F.Cu', 'B.Cu'], 'net_id': 1,
                              'tenting_attrs': dict(TYPE_VII),
                              'inherit_when_unspecified': True}])
        check(len(got) == 1 and got[0].tenting_attrs == TYPE_VII,
              f"A2: a re-emitted via reported "
              f"{got[0].tenting_attrs if got else None}, expected its own "
              f"Type-VII spec {TYPE_VII}")

        # --- A3: a pre-existing via with NO spec keeps INHERITING. This is the
        # half a carried `{}` cannot express on its own: it reads as "no
        # opinion" and takes the default.
        out, got = _emit(tmp, [{'x': 12.0, 'y': 12.0, 'size': 0.6, 'drill': 0.3,
                                'layers': ['F.Cu', 'B.Cu'], 'net_id': 1,
                                'tenting_attrs': {},
                                'inherit_when_unspecified': True}])
        check(len(got) == 1 and got[0].tenting_attrs == {},
              f"A3: an inheriting via came back with "
              f"{got[0].tenting_attrs if got else None}; it must emit NO "
              f"protection token and keep inheriting `(setup ...)`")

        # --- D: net 0 must not fall to the numeric dialect on a name-net board.
        out, got = _emit(tmp, [{'x': 13.0, 'y': 13.0, 'size': 0.6, 'drill': 0.3,
                                'layers': ['F.Cu', 'B.Cu'], 'net_id': 0}])
        check('(net 0)' not in out,
              "D: a no-net via was emitted as numeric `(net 0)` into a name-net "
              "board -- the mixed-dialect state fix_mixed_net_refs.py undoes, "
              "and next to a spec it is a via extract_vias cannot read (#748)")
        check(len(got) == 1,
              f"D: the no-net via parsed back as {len(got)} vias, expected 1")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # --- D, at the resolver ---
    check(via_net_name(0, N2N) == '',
          "via_net_name must spell net 0 as '' -- net_id_to_name has no key 0 "
          "on any board, so every site fell to the numeric dialect for it")
    check(via_net_name(1, N2N) == '/SIG', "via_net_name(1) wrong")
    check(via_net_name(9, N2N) is None,
          "an id the map genuinely does not know must stay numeric, not be "
          "guessed a name")
    check(via_net_name(0, None) is None,
          "with no map at all the board is numeric-dialect; net 0 included")

    # --- C: the restore inherits ---
    tmp = tempfile.mkdtemp(prefix='t749c_')
    try:
        from plane_io import restore_failed_reroute_nets
        src = os.path.join(tmp, 'in.kicad_pcb')
        dst = os.path.join(tmp, 'out.kicad_pcb')
        # A via with NO spec of its own, on the net we will "fail to reroute".
        plain = BOARD.replace(
            '\t\t(covering (front no) (back no))\n\t\t(plugging (front no) (back no))\n', '')
        with open(src, 'w', encoding='utf-8') as f:
            f.write(plain)
        # The output is the board with that net's copper gone (it was ripped).
        with open(dst, 'w', encoding='utf-8') as f:
            f.write(plain.replace(plain[plain.index('\t(via'):plain.index('\t)\n)', plain.index('\t(via')) + 3], ')'))
        restore_failed_reroute_nets(src, dst, [1], [], N2N, 0.6, 0.2)
        with open(dst, encoding='utf-8') as f:
            out = f.read()
        got = extract_vias(out, {'': 0, '/SIG': 1})
        check(len(got) == 1,
              f"C: after restore the board has {len(got)} vias, expected 1")
        if got:
            check(got[0].tenting_attrs == {},
                  f"C: a RESTORED via that carried no spec came back with "
                  f"{got[0].tenting_attrs}; it was inheriting the board's "
                  f"`(setup ...)` and must keep inheriting")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # --- B + a change detector for every future site -------------------------
    # Structural, not textual: every generate_via_sexpr CALL under py_router
    # must decide the protection question rather than fall through to the
    # writer's default. A new emit site that forgets fails here.
    #
    # py_placer/placement/writer.py is deliberately out of scope: that is #741,
    # whose fix is not on main.
    for rel in ('py_router/kicad_oracle.py', 'py_router/plane_io.py',
                'py_router/repair_planes.py', 'py_router/output_writer.py',
                'py_router/kicad_writer.py'):
        path = os.path.join(ROOT_DIR, rel)
        tree = ast.parse(open(path, encoding='utf-8').read(), rel)
        calls = [n for n in ast.walk(tree)
                 if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name)
                 and n.func.id == 'generate_via_sexpr']
        check(bool(calls), f"{rel}: no generate_via_sexpr call found -- this "
                           f"check has gone stale and is testing nothing")
        for c in calls:
            kw = {k.arg for k in c.keywords}
            check('tenting_attrs' in kw,
                  f"{rel}:{c.lineno}: generate_via_sexpr with no "
                  f"tenting_attrs= -- it will be stamped with the hardcoded "
                  f"front+back tenting (#489 s8 / #749)")

    # route.py's #666 re-emit builds via dicts inline; both keys must be there
    # or the writer cannot tell pre-existing copper from new copper.
    src666 = open(os.path.join(ROOT_DIR, 'py_router', 'route.py'),
                  encoding='utf-8').read()
    tree = ast.parse(src666, 'route.py')
    found = False
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == '_vi666'
                   for t in node.targets):
            continue
        found = True
        keys = set()
        for d in ast.walk(node.value):
            if isinstance(d, ast.Dict):
                keys |= {k.value for k in d.keys
                         if isinstance(k, ast.Constant)}
        for want in ('tenting_attrs', 'inherit_when_unspecified'):
            check(want in keys,
                  f"route.py _vi666 (#666 re-emit) has no {want!r} key: these "
                  f"are PRE-EXISTING vias the written file lost, so they must "
                  f"carry their spec back and inherit when they had none")
    check(found, "route.py: no _vi666 assignment found -- the #666 re-emit was "
                 "renamed and this check has gone stale")

    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} check(s) FAILED")
        return False
    print("PASS  #749 via-protection emit sites (writer contract, restore "
          "inheritance, net-0 dialect, per-call coverage)")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)
