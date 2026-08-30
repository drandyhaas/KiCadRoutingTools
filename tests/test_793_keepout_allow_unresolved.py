"""A keep-out `allow` glob that matches nothing is reported (#793).

The failure it catches: `allow` is fnmatched against every reference, nothing
checked that a pattern matched anything, and since #701 the SEAT SEARCH consults
the same resolver -- so a typo does not merely fail to excuse a part, it strands
the part the keep-out was drawn around.

What the author saw before, measured on splitflap_driver with a keep-out over H1
carrying `allow: ["H01"]`, in the two states an intent is read in:

  * keep-out over the part it names -> `[ERROR] keepout: H1 (F) is inside
    keep-out 'mount-NW'`. About the PART, never the exemption; the failing
    pattern reached the JSON in `expected.allow` and the printed text never.
  * keep-out over empty space -- the state a board is in BEFORE placement, which
    is when an intent is authored -> `violations 0, errors 0, pass true`,
    exit 0. Nothing at all.

So "silently" is true of the second case and understated for the first: in
neither does anything say the pattern matched nothing.

Every accepting arm has a CONTROL, because a check that reports every `allow`
would pass all the fire arms and none of the quiet ones -- and the corpus is
protected by the quiet ones (`emit_intent` writes `keepouts: []`, so every
emitted intent must stay silent).

THE MUTATION TABLE, RECORDED FROM THE RUN (`python3 tests/mutate_799.py`, which
carries both issues' rows), never predicted:

    20 rows: 16 killed, 4 survived, 0 broken, 0 disagreeing with expectation

The five rows aimed at this file are all KILLED --
`allow-unresolved-fires-at-error` (the `default=WARN` that `severity_of`'s
ERROR default would otherwise swallow),
`allow-unresolved-checks-the-tuple-not-each-pattern`,
`the-audit-uses-its-own-matcher`, `the-exemption-uses-its-own-matcher` (the two
halves of the identity claim), and `grade-does-not-raise-either-finding`.
The four survivors all belong to #799; see that file's header for their reasons.
"""
import json
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan  # noqa: E402
from placement.floorplan import ERROR, WARN  # noqa: E402

RULE = 'keepout_allow_unresolved'
SIZE = (24.0, 20.0)

passed = 0
failed = 0


def check(name, ok, detail=""):
    """Detail must read as a MEASUREMENT: it prints on OK and on FAIL alike."""
    global passed, failed
    if ok:
        passed += 1
        print(f"  OK   {name}" + (f" -- {detail}" if detail else ""))
    else:
        failed += 1
        print(f"  FAIL {name}" + (f" -- {detail}" if detail else ""))


def _part(ref, x, y, half=1.0):
    return (f'\t(footprint "test:P{ref}"\n\t\t(layer "F.Cu")\n'
            f'\t\t(uuid "fp-{ref}")\n\t\t(at {x} {y})\n'
            f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0)\n\t\t)\n'
            f'\t\t(fp_rect\n\t\t\t(start {-half} {-half})\n'
            f'\t\t\t(end {half} {half})\n\t\t\t(layer "F.CrtYd")\n'
            f'\t\t\t(uuid "cy-{ref}")\n\t\t)\n'
            f'\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.3 0.3)\n'
            f'\t\t\t(layers "F.Cu")\n\t\t\t(net 1 "N1")\n'
            f'\t\t\t(uuid "p0-{ref}")\n\t\t)\n\t)\n')


def board(path, parts):
    body = ('(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n\t(net 1 "N1")\n'
            f'\t(gr_rect\n\t\t(start 0 0)\n\t\t(end {SIZE[0]} {SIZE[1]})\n'
            '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n'
            + ''.join(parts) + ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)
    return path


def intent_doc(keepouts=(), severity=None):
    d = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
         "envelope": {"rect": [0.0, 0.0, SIZE[0], SIZE[1]],
                      "tolerance_mm": 0.5},
         "blocks": []}
    if keepouts:
        d["keepouts"] = [dict(k) for k in keepouts]
    if severity:
        d["severity"] = severity
    return d


def load(doc, wd, tag):
    p = os.path.join(wd, f'{tag}.json')
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    return floorplan.load_intent(p)


def hits(intent, pcb):
    return [v for v in floorplan.unresolved_keepout_allows(intent, pcb)
            if v.rule == RULE]


def main():
    with tempfile.TemporaryDirectory() as wd:
        # MH1 sits INSIDE the keep-out; U1 is far from it. Both states of the
        # board matter -- see the module docstring.
        b = board(os.path.join(wd, 'b.kicad_pcb'),
                  [_part('MH1', 4.0, 4.0), _part('U1', 18.0, 15.0)])
        pcb = parse_kicad_pcb(b)
        KO = [2.0, 2.0, 6.0, 6.0]

        # --- the typo, and its control -----------------------------------
        bad = load(intent_doc([{"name": "mount-NW", "rect": KO,
                                "allow": ["MH01"]}]), wd, 'bad')
        h = hits(bad, pcb)
        check("a pattern matching no reference is reported",
              len(h) == 1 and 'MH01' in h[0].message,
              h[0].message[:90] if h else "no finding raised")
        check("... and it names the keep-out it belongs to",
              bool(h) and h[0].measured.get('keepout') == 'mount-NW',
              f"measured.keepout={h[0].measured.get('keepout') if h else None}")

        good = load(intent_doc([{"name": "mount-NW", "rect": KO,
                                 "allow": ["MH1"]}]), wd, 'good')
        check("CONTROL: the corrected pattern is not reported",
              not hits(good, pcb),
              f"{len(hits(good, pcb))} finding(s) on a working allow")

        # A GLOB, not just an exact ref: an implementation comparing strings
        # with == would pass every arm above and fail here.
        glob = load(intent_doc([{"name": "mount-NW", "rect": KO,
                                 "allow": ["MH*"]}]), wd, 'glob')
        check("CONTROL: a working GLOB is not reported",
              not hits(glob, pcb), "allow=['MH*'] matches MH1")

        # --- per PATTERN, not per entry ----------------------------------
        mixed = load(intent_doc([{"name": "mount-NW", "rect": KO,
                                  "allow": ["MH1", "MH01"]}]), wd, 'mixed')
        hm = hits(mixed, pcb)
        check("a live pattern beside a dead one still reports the dead one",
              len(hm) == 1 and hm[0].measured.get('unmatched') == ['MH01'],
              f"unmatched={hm[0].measured.get('unmatched') if hm else None} "
              f"matched={hm[0].measured.get('matched') if hm else None}")

        two = load(intent_doc([{"name": "k", "rect": KO,
                                "allow": ["AA1", "BB2"]}]), wd, 'two')
        ht = hits(two, pcb)
        check("two dead patterns are reported on ONE entry, both named",
              len(ht) == 1
              and ht[0].measured.get('unmatched') == ['AA1', 'BB2'],
              f"{len(ht)} finding(s), unmatched="
              f"{ht[0].measured.get('unmatched') if ht else None}")

        # --- inertness: what protects the whole corpus -------------------
        none_ = load(intent_doc([{"name": "k", "rect": KO}]), wd, 'none')
        check("CONTROL: a keep-out with no `allow` at all is not reported",
              not hits(none_, pcb),
              "this is what keeps every emitted intent silent")
        empty = load(intent_doc(), wd, 'empty')
        check("CONTROL: an intent with no keep-outs is not reported",
              not hits(empty, pcb), "emit_intent writes keepouts: []")

        # --- severity -----------------------------------------------------
        check("the finding is WARN by default",
              bool(h) and h[0].severity == WARN,
              f"severity={h[0].severity if h else None} "
              f"(severity_of defaults to ERROR, so this needs default=WARN)")
        up = load(intent_doc([{"name": "mount-NW", "rect": KO,
                               "allow": ["MH01"]}],
                             severity={RULE: "error"}), wd, 'up')
        hu = hits(up, pcb)
        check("... and an intent can upgrade it to error",
              bool(hu) and hu[0].severity == ERROR,
              f"severity={hu[0].severity if hu else None}")

        # --- BOTH reach points, on the same board and intent --------------
        # The finding exists to reach an author. `grade` is check_floorplan,
        # where an intent is authored; `resolve_intent_gate` is what the four
        # quenching CLIs run, and where a stale glob is stranding a part now.
        g = floorplan.grade(bad, pcb, b)
        gr = [v for v in g.violations if v.rule == RULE]
        _bundle, probs = floorplan.resolve_intent_gate(bad, pcb, ())
        pr = [v for v in probs if v.rule == RULE]
        check("BOTH grade() and resolve_intent_gate() report it",
              len(gr) == 1 and len(pr) == 1
              and gr[0].message == pr[0].message,
              f"grade={len(gr)} gate={len(pr)}, messages "
              f"{'identical' if gr and pr and gr[0].message == pr[0].message else 'DIFFER'}")

        # Loud without being fatal -- asserted, not described. This is the
        # claim `board_score` used to falsify by counting warnings as blocking.
        # Loud without being fatal -- asserted on the EMPTY-SPACE keep-out,
        # which is the case that was measured silent (violations 0, exit 0) and
        # is the state a board is in when an intent is authored. The `bad`
        # fixture above cannot show this: MH1 really is inside that keep-out
        # unexempted, so `rule_keepout` fires at ERROR for a correct reason and
        # `passed` would be False for a reason that is not this finding.
        far = load(intent_doc([{"name": "empty-corner",
                                "rect": [20.0, 2.0, 23.0, 5.0],
                                "allow": ["MH01"]}]), wd, 'far')
        gf = floorplan.grade(far, pcb, b)
        gfr = [v for v in gf.violations if v.rule == RULE]
        summ = floorplan.summary(gf)
        check("over empty space -- once SILENT -- it is now reported, at warn",
              len(gfr) == 1 and gfr[0].severity == WARN,
              f"{len(gfr)} finding(s), severity="
              f"{gfr[0].severity if gfr else None}")
        check("... and it does not fail the grade: `passed` still holds",
              gf.passed and summ['errors'] == 0 and summ['warnings'] == 1,
              f"passed={gf.passed} errors={summ['errors']} "
              f"warnings={summ['warnings']}")

        # --- the resolver and the audit must not drift --------------------
        # IDENTITY, not agreement: patch the shared matcher and require the
        # audit to observe it. A private second fnmatch would not.
        real = floorplan.allow_pattern_matches
        seen = []

        def fake(pattern, ref):
            seen.append((pattern, ref))
            return True          # every pattern matches everything
        floorplan.allow_pattern_matches = fake
        try:
            patched = hits(bad, pcb)
        finally:
            floorplan.allow_pattern_matches = real
        check("the audit calls the resolver's OWN matcher",
              seen and not patched,
              f"patched matcher called {len(seen)} time(s), "
              f"{len(patched)} finding(s) with everything matching")

        # And the other direction, so the arm cannot pass by never calling it.
        seen2 = []

        def fake_never(pattern, ref):
            seen2.append((pattern, ref))
            return False         # nothing matches anything
        floorplan.allow_pattern_matches = fake_never
        try:
            starved = hits(good, pcb)
            bound = floorplan.keepouts_for_ref(
                ({"name": "k", "rect": KO, "allow": ["MH1"],
                  "sides": ("F", "B")},), 'MH1', frozenset({'F'}))
        finally:
            floorplan.allow_pattern_matches = real
        check("... and so does keepouts_for_ref, the exemption itself",
              seen2 and len(starved) == 1 and len(bound) == 1,
              f"working allow reports {len(starved)} finding(s) and binds "
              f"{len(bound)} entry(ies) when nothing matches")

    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
