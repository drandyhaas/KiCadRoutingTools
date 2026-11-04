#!/usr/bin/env python3
"""#1606: a capacitor is a capacitor behind a glued qualifier -- crkbd's
right half (`rY1`, `rC1`, `rC2`, `rU1`) is a copy of its left.

`crystals.crystal_claims` found a crystal's load caps only among references
matching `^C\\d`, so crkbd's `rY1` had none while `Y1`, wired identically,
had C1 and C2. The same test sat in `groups.is_decoupling_cap` (the decap
population: crkbd's right-half decaps `rC3`..`rC16` decoupled nothing) and
in `grouping`'s template hints (`_is_cap`, `_is_inductor`, the `U`/`IC` chip
test). All read the reference through `part_class.unqualified_ref` now,
which sets a LOWER-case qualifier glued in front of an upper-case
designator aside and nothing else: a qualifier behind a separator
(`_designator`'s `PWR_SW1`) is not read, because as a passive's designator
it would make the connector `USB_C1` and the keyboard diode `D_C1`
capacitors (both are corpus references).

crkbd is not in the repo, so tigard is copied the crkbd way: its crystal
Y1, its IC U3 and its load caps C23/C24 renamed rY1/rU3/rC23/rC24.

Cases:
* `unqualified_ref` on the copy spelling and on what it must leave alone;
* the renamed crystal gets the same claims as the original, with its caps
  under their new names;
* a renamed decap stays in the decap population with the same rails and
  owners.

    python3 -X utf8 tests/test_1606_prefixed_copy_caps.py [case ...]
"""
import contextlib
import copy
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 600

TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
_CACHE = {}


def _parse(path):
    if path not in _CACHE:
        run_utils.evidence(path, 'the board')
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            _CACHE[path] = parse_kicad_pcb(path)
    return _CACHE[path]


def _quiet(fn, *a):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a)


def _renamed(pcb, names):
    """A copy of `pcb` with each reference in `names` renamed, the way a
    board copies a half: the key, the footprint's reference, its pads'."""
    pcb = copy.deepcopy(pcb)
    # the decap view is cached on the board (`grouping.decap_view`), and a
    # deep copy carries it, keyed by the old references
    pcb.__dict__.pop('_decap_view', None)
    fps = {}
    for ref, fp in pcb.footprints.items():
        new = names.get(ref, ref)
        if new != ref:
            fp.reference = new
            for p in fp.pads:
                p.component_ref = new
        fps[new] = fp
    pcb.footprints = fps
    return pcb


def test_unqualified_ref():
    from placement.part_class import unqualified_ref
    cases = {'rC1': 'C1', 'rY1': 'Y1', 'rU1': 'U1', 'rSMDSW1': 'SMDSW1',
             'C1': 'C1', 'c1': 'c1', 'CN1': 'CN1', 'PWR_SW1': 'PWR_SW1',
             'USB_C1': 'USB_C1', 'D_C1': 'D_C1', 'HVP_C1': 'HVP_C1',
             '': '', 'rc1': 'rc1'}
    got = {r: unqualified_ref(r) for r in cases}
    assert got == cases, {r: g for r, g in got.items() if g != cases[r]}
    print(f"  PASS: unqualified_ref on {len(cases)} spellings")


def test_a_copied_crystal_keeps_its_load_caps():
    from placement.crystals import crystal_claims
    pcb = _parse(TIGARD)
    rows, notes = _quiet(crystal_claims, pcb)
    note = next(n for n in notes if n.startswith('crystal Y1 '))
    assert note.endswith('load caps C23, C24'), note
    names = {'Y1': 'rY1', 'U3': 'rU3', 'C23': 'rC23', 'C24': 'rC24'}
    rows2, notes2 = _quiet(crystal_claims, _renamed(pcb, names))
    want = note.replace('Y1', 'rY1').replace('U3', 'rU3') \
        .replace('C23', 'rC23').replace('C24', 'rC24')
    assert want in notes2, (want, notes2)

    def ren(rows):
        return sorted((names.get(r['ref'], r['ref']),
                       names.get(r['near'], r['near'])) for r in rows)
    assert ren(rows) == ren(rows2), (ren(rows), ren(rows2))
    print(f"  PASS: {want}")


def test_a_copied_decap_is_still_a_decap():
    from placement.groups import decap_population, is_decoupling_cap
    pcb = _parse(TIGARD)
    pop = _quiet(decap_population, pcb)
    cap = sorted(pop)[0]
    assert cap.startswith('C') and is_decoupling_cap(
        pcb.footprints[cap], cap), cap
    copied = _renamed(pcb, {cap: 'r' + cap})
    assert is_decoupling_cap(copied.footprints['r' + cap], 'r' + cap)
    pop2 = _quiet(decap_population, copied)
    assert pop2.get('r' + cap) == pop[cap], (pop[cap], pop2.get('r' + cap))
    assert cap not in pop2
    print(f"  PASS: r{cap} decouples {pop[cap][0]} for {pop[cap][1]}, "
          f"as {cap} does")


TESTS = [v for k, v in sorted(globals().items())
         if k.startswith('test_') and callable(v)]


if __name__ == '__main__':
    want = sys.argv[1:]
    n = 0
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(t.__name__)
        t()
        n += 1
    print(f"ALL PASS ({n} case(s))")
