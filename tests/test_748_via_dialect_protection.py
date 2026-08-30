#!/usr/bin/env python3
"""#748: a via carrying a protection token must parse in BOTH net dialects, and
no pattern may reach out of one via into another.

`extract_vias` had two patterns. The KiCad-10 named-net one was given a flexible
gap between `(layers ...)` and the net token, for exactly the protection family
(tenting/covering/plugging/capping/filling) KiCad 10 writes there. Its KiCad-9
numeric twin kept strict field ordering, so a NUMERIC-net via carrying any
protection token matched neither pattern and vanished from the model -- not an
invisible spec but an invisible BARREL, which the router then plans tracks
through (the hazard PR #534 was written against, and the #344/#369 "forgot the
KiCad-10 twin" class inverted).

Our own writer produces that shape: `generate_via_sexpr` picks the net dialect
from `net_name` and the tokens from `tenting_attrs` INDEPENDENTLY, and
`net_id_to_name` has no key 0 on any board.

The flexible gap was not free either. Measured on main, a mixed-dialect board --
which this repo's own fanout step produces, see tests/stress/fix_mixed_net_refs.py
-- parsed a numeric via followed by a named one as TWO vias BOTH at the numeric
one's coordinates: the `.*?` ran out of the first via into the second's
`(net "...")`, inventing a barrel and swallowing a real one.

Run with:  python3 tests/test_748_via_dialect_protection.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import extract_vias, parse_kicad_pcb  # noqa: E402
from kicad_writer import generate_via_sexpr  # noqa: E402

HDR = '(kicad_pcb (version 20241229) (generator "pcbnew")\n'
SPEC = {'covering': '(front no) (back no)', 'capping': 'yes'}
TOKENS = '\t\t(covering (front no) (back no))\n\t\t(capping yes)\n'
N2I = {'': 0, '/SIG': 5, '/BUS(0)': 6, '/A"B': 7}


def via_text(numeric, token_pos='none', free=False, locked=False,
             at=(1.0, 2.0), net=(5, '/SIG'), uuid='aaaaaaaa-0000-0000-0000-000000000001'):
    net_tok = f'(net {net[0]})' if numeric else f'(net "{net[1]}")'
    s = (f'\t(via\n\t\t(at {at[0]} {at[1]})\n\t\t(size 0.6)\n\t\t(drill 0.3)\n'
         f'\t\t(layers "F.Cu" "B.Cu")\n')
    if token_pos == 'before':
        s += TOKENS
    if locked:
        s += '\t\t(locked yes)\n'
    if free:
        s += '\t\t(free yes)\n'
    s += f'\t\t{net_tok}\n'
    if token_pos == 'after':
        s += TOKENS
    if uuid:
        s += f'\t\t(uuid "{uuid}")\n'
    if token_pos == 'tail':
        s += TOKENS
    return s + '\t)\n'


def board(*vias):
    return HDR + ''.join(vias) + ')\n'


def run():
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    # --- the matrix. Every cell was measured on main; the four numeric cells
    # with a token BEFORE the net returned ZERO vias, and every "after" cell
    # parsed but lost the spec (the uuid capture failed, so the uuid-keyed
    # protection join missed).
    for numeric in (True, False):
        for pos in ('none', 'before', 'after', 'tail'):
            for free in (False, True):
                for locked in (False, True):
                    b = board(via_text(numeric, pos, free=free, locked=locked))
                    vs = extract_vias(b, N2I)
                    lbl = (f"{'numeric' if numeric else 'named'} token={pos} "
                           f"free={free} locked={locked}")
                    if len(vs) != 1:
                        check(False, f"{lbl}: parsed {len(vs)} vias, expected 1"
                                     + (" -- THE VIA VANISHED, which is an "
                                        "invisible barrel the router will plan "
                                        "tracks through" if not vs else ""))
                        continue
                    v = vs[0]
                    check(v.net_id == 5, f"{lbl}: net_id {v.net_id}, expected 5")
                    check(v.free is free, f"{lbl}: free={v.free}, expected {free}")
                    check(v.locked is locked,
                          f"{lbl}: locked={v.locked}, expected {locked}")
                    want = {} if pos == 'none' else SPEC
                    check(v.tenting_attrs == want,
                          f"{lbl}: spec {v.tenting_attrs}, expected {want}")

    # --- mixed dialect: no phantom, no swallow. On main this returned two vias
    # BOTH at (1,2) -- the named via at (20,20) never appeared at all.
    b = board(via_text(True, at=(1.0, 2.0), uuid='aaaaaaaa-0000-0000-0000-000000000001'),
              via_text(False, at=(20.0, 20.0), uuid='aaaaaaaa-0000-0000-0000-000000000020'))
    vs = extract_vias(b, N2I)
    pos = sorted((v.x, v.y) for v in vs)
    check(pos == [(1.0, 2.0), (20.0, 20.0)],
          f"mixed dialect: got vias at {pos}, expected exactly (1,2) and "
          f"(20,20) -- a repeated coordinate means the KiCad-10 pattern ran "
          f"out of one via into the next")

    # ...and with a spec on each, so the gap has something to run through.
    b = board(via_text(True, 'before', at=(1.0, 2.0),
                       uuid='bbbbbbbb-0000-0000-0000-000000000001'),
              via_text(False, 'before', at=(20.0, 20.0),
                       uuid='bbbbbbbb-0000-0000-0000-000000000020'))
    vs = extract_vias(b, N2I)
    check(sorted((v.x, v.y) for v in vs) == [(1.0, 2.0), (20.0, 20.0)],
          "mixed dialect with specs: wrong via positions")
    check(all(v.tenting_attrs == SPEC for v in vs),
          f"mixed dialect with specs: {[v.tenting_attrs for v in vs]}")

    # --- OUR OWN WRITER's numeric+spec pairing must read back. This is the
    # producer #748 names: any caller with a spec and an unresolved net name.
    emitted = generate_via_sexpr(1.0, 2.0, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5,
                                 tenting_attrs=SPEC)
    vs = extract_vias(HDR + emitted + '\n)\n', N2I)
    check(len(vs) == 1,
          f"generate_via_sexpr(numeric net, spec) -> extract_vias parsed "
          f"{len(vs)} vias; our own output must survive our own parser")
    if vs:
        check(vs[0].tenting_attrs == SPEC,
              f"writer round trip lost the spec: {vs[0].tenting_attrs}")

    # --- a uuid-less via can carry a spec now (it could not before: the join
    # was uuid-keyed). uuid-less copper is legal -- KiCad mints one on load.
    vs = extract_vias(board(via_text(True, 'before', uuid='')), N2I)
    check(len(vs) == 1 and vs[0].tenting_attrs == SPEC,
          f"uuid-less via: {len(vs)} via(s), spec "
          f"{vs[0].tenting_attrs if vs else None}")

    # --- net names the old escape-blind pattern could not read. A name with a
    # PAREN would break a naive paren-balanced block scan; a name with an
    # escaped QUOTE ended the old [^"]* capture early and lost the via.
    vs = extract_vias(board(via_text(False, net=(6, '/BUS(0)'))), N2I)
    check(len(vs) == 1 and vs[0].net_id == 6,
          f"net name with parens: {len(vs)} via(s), "
          f"net_id {vs[0].net_id if vs else None} (expected 6)")
    vs = extract_vias(board(via_text(False, net=(7, '/A\\"B'))), N2I)
    check(len(vs) == 1 and vs[0].net_id == 7,
          f"net name with an escaped quote: {len(vs)} via(s), "
          f"net_id {vs[0].net_id if vs else None} (expected 7)")

    # --- a zone keepout's (vias not_allowed) must not be read as a via.
    vs = extract_vias(HDR + '\t(zone (vias not_allowed) (via_dimensions))\n)\n', N2I)
    check(vs == [], f"(vias not_allowed) parsed as {len(vs)} via(s)")

    # --- the tracked boards that actually carry specs keep every one of them.
    for name, n_specs in (('orangecrab_ext_pll', 136),
                          ('rp2350_fpga_eensy_prePlane', 70)):
        path = os.path.join(ROOT_DIR, 'kicad_files', name + '.kicad_pcb')
        if not os.path.isfile(path):
            continue
        pcb = parse_kicad_pcb(path)
        got = sum(1 for v in pcb.vias if v.tenting_attrs)
        check(got == n_specs,
              f"{name}: {got} vias carry a protection spec, expected {n_specs}")

    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} check(s) FAILED")
        return False
    print("PASS  #748 via dialects x protection tokens (matrix, mixed-dialect "
          "isolation, writer round trip, escaped/paren net names)")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)
