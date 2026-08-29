"""#698: `place_seed --reseat REF` can accept an on-board part.

The bug had TWO reverting mechanisms and fixing either alone leaves the board
broken, so the arms below are built to tell them apart:

* **M1**, the acceptance gate (`seeder.reseat_scope`): `after[oob] <
  before[oob]`. A part that is legal and ON the board has `oob` unchanged, so an
  explicitly named part could never be re-seated whatever the search found.
* **M2**, `reconstruct.prune_assignment`, which reverts FIRST and compares a
  tuple with no intent term -- so a seat that cleared a declared keep-out reads
  as a pure hpwl loss and is undone before the gate ever runs.

Fixtures are synthetic, in the shape of `tests/test_702_quench_intent_gate.py`'s
(copied, as that file copied `test_701_keepout_seating.py`'s): on a corpus board
"where should this part have gone" is only answerable by re-running the
optimizer, which makes the control circular. Here the answer is a theorem.

**The KEEPOUT fixture's hpwl cost is a theorem, and that was measured the hard
way.** Every net is a 2-pad net between U1 and a partner; both partners sit
strictly INSIDE the keep-out rect, so any pose that clears the rect is strictly
farther from both. Swept over 20 seeds on the fixture as it stands: hpwl is
worse on **20 of 20** (6.82 to 12.39 mm -- a different value per seed, since
the seat search is seeded; 11.372 at seed 0), prune reverts on 20 of 20 without
the probe, and the pass is accepted on 20 of 20 with it.

The first version of this fixture put the partners at (3,3) and (17,9), OUTSIDE
the rect, where their bounding box contained most escape poses. HPWL was flat at
39.2 and prune reverted on **9 of 20** seeds -- so on the other 11 the seat
survived prune and only the acceptance gate refused it, and `pruned == ['U1']`
would have been a seed-0 accident asserted as a property. (That measurement was
taken at the time and the fixture has since been replaced, so it is not
re-derivable from this tree; the rebuild moved the partners, halved U1's
courtyard and added the keep-out's `allow` list.) `arm_M_seed_independence` is
what stops it recurring.

Mutation battery: `tests/mutate_698.py` -- **38 rows: 36 killed, 2 survived
(both recorded with their reason), 0 broken, 0 disagreeing with expectation.**

What it caught in THIS file, over two rounds, which is why it exists:

| mutation | why it survived |
|---|---|
| `drop-the-safety-half` | a legal seat search never worsens a hard term, so no behavioural arm could reach the conjunct. `arm_Q` drives `reseat_accept` directly. |
| `prune-refuses-every-revert` | no fixture had prune legitimately reverting a claim-bound part, so "the probe is a conjunct, not an exemption" was untested. `arm_R`. |
| `hpwl-ignores-its-net-subset` | every net on the plain fixture touched the scope ref, so `scope_hpwl` and the board-wide `hpwl` were the same number. R8/R9 fixed that. |
| `the-KEPT-note-credits-the-probe-for-every-held-revert` | **round 2** -- the arm-S check written for that very finding put U1 outside the keep-out at BOTH poses, so `undoes_intent` was False and the mutated line was unreachable. |
| `the-probe-goes-back-to-the-named-scope-only` | **round 2** -- that check asserted on a probe the TEST built, which says nothing about the one `reseat_scope` builds. It now spies the real call site. |

The round-2 pair is the lesson worth keeping: those two checks were written to
pin findings from a code review, and they were themselves vacuous. A fix is not
verified by the reviewer that motivated it.

And one the battery caught in ITSELF: the row
`the-zone-spec-forgets-the-anchor-branch` originally mutated `not any(` to
`False or not any(`, which is the same expression -- a row that could not fail,
recorded as an expected survivor. It read as a finding and was a tautology.
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
        sys.path.insert(0, os.path.join(_p, 'py_router'))
        sys.path.insert(0, os.path.join(_p, 'py_tools'))
        sys.path.insert(0, os.path.join(_p, 'py_placer'))

import run_utils                                             # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement import floorplan, reconstruct, seeder         # noqa: E402
from placement import quench as q                            # noqa: E402
from placement.writer import write_placed_output              # noqa: E402
from placement.quench import INTENT_ENFORCED_RULES           # noqa: E402

RUN_ALL_TIMEOUT = 1200

passed = failed = 0


def check(name, ok, detail=""):
    """Prints `detail` on OK and FAIL alike, so every detail must read as a
    MEASUREMENT rather than as a failure explanation."""
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}" + (f" -- {detail}" if detail
                                                    else ""))


# --------------------------------------------------------------------------
# fixtures
# --------------------------------------------------------------------------

def _part(ref, x, y, half_w, half_h, npads=2, layer='F.Cu', thru=False,
          nets=None, rot=0.0):
    fp_name = f'test:P{ref}'
    nets = nets or tuple(1 if i == 0 else 2 for i in range(npads))
    if thru:
        pads = ''.join(
            f'\t\t(pad "{i + 1}" thru_hole circle\n'
            f'\t\t\t(at {i * 0.6 - 0.3} 0)\n'
            f'\t\t\t(size 0.6 0.6)\n\t\t\t(drill 0.35)\n'
            f'\t\t\t(layers "*.Cu" "*.Mask")\n'
            f'\t\t\t(net {nets[i]} "N{nets[i]}")\n'
            f'\t\t\t(uuid "p{i}-{ref}")\n\t\t)\n' for i in range(npads))
    else:
        pads = ''.join(
            f'\t\t(pad "{i + 1}" smd rect\n'
            f'\t\t\t(at {i * 0.2 - 0.2} 0)\n'
            f'\t\t\t(size 0.3 0.3)\n\t\t\t(layers "{layer}")\n'
            f'\t\t\t(net {nets[i]} "N{nets[i]}")\n'
            f'\t\t\t(uuid "p{i}-{ref}")\n\t\t)\n' for i in range(npads))
    return f'''\t(footprint "{fp_name}"
\t\t(layer "{layer.split('.')[0]}.Cu")
\t\t(uuid "fp-{ref}")
\t\t(at {x} {y} {rot})
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 0)
\t\t)
\t\t(fp_rect
\t\t\t(start {-half_w} {-half_h})
\t\t\t(end {half_w} {half_h})
\t\t\t(layer "{layer.split('.')[0]}.CrtYd")
\t\t\t(uuid "cy-{ref}")
\t\t)
{pads}\t)
'''


def board(path, parts, size):
    body = ('(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n'
            + ''.join(f'\t(net {i} "N{i}")\n' for i in range(1, 9))
            + ('\t(gr_rect\n\t\t(start 0 0)\n\t\t(end {} {})\n'
               '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n').format(*size)
            + ''.join(parts) + ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)
    return path


def load(doc, wd, tag='fp'):
    p = os.path.join(wd, f'{tag}.json')
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    return p, floorplan.load_intent(p)


KO_SIZE = (20.0, 12.0)
KO_RECT = [8.0, 4.0, 12.0, 8.0]


def keepout_board(wd, tag='ko', ux=10.0, uy=6.0):
    """U1 (1x1 courtyard) inside keep-out `hot`; its two net partners R1 (9,5)
    and C1 (11,7) strictly INSIDE the rect and in `allow`.

    The partner placement is the fixture's whole point -- see the module
    docstring. U1's input pose is fully legal: only the declared claim says it
    is in the wrong place.
    """
    return board(os.path.join(wd, f'{tag}.kicad_pcb'),
                 [_part('U1', ux, uy, 0.5, 0.5, npads=4, nets=(1, 2, 3, 4)),
                  _part('R1', 9.0, 5.0, 0.4, 0.3, npads=2, nets=(1, 2)),
                  _part('C1', 11.0, 7.0, 0.4, 0.3, npads=2, nets=(3, 4))],
                 size=KO_SIZE)


def keepout_intent(wd, tag='ko', keepouts=True):
    ko = ([{"name": "hot", "rect": list(KO_RECT), "allow": ["R1", "C1"]}]
          if keepouts else [])
    d = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
         "envelope": {"rect": [0.0, 0.0, KO_SIZE[0], KO_SIZE[1]],
                      "tolerance_mm": 0.5},
         "blocks": [{"name": "all", "refs": ["U1", "R1", "C1"]}]}
    if ko:
        d["keepouts"] = ko
    return load(d, wd, tag)


PL_SIZE = (31.75, 14.5)


def plain_board(wd, tag='pl'):
    """A legal, on-board, MISPLACED part with NO declared claim at all -- the
    pure form of the defect. Nothing but the scope's own wirelength can carry
    it, so this arm proves the fix is not intent-dependent."""
    # R8/R9 carry net 8, which CON2 has no pad on. They are what makes
    # `scope_hpwl` differ from the board-wide `hpwl` -- without a net OUTSIDE
    # the scope the two numbers are equal and `state.hpwl(nets)` could ignore
    # its subset undetected (a mutation that survived until they were added).
    return board(os.path.join(wd, f'{tag}.kicad_pcb'),
                 [_part('CON2', 15.0, 7.0, 8.9, 1.3, npads=7, thru=True,
                        nets=(1, 2, 3, 4, 5, 6, 7)),
                  _part('U1', 4.0, 3.0, 1.5, 1.5, npads=4, nets=(1, 2, 3, 4)),
                  _part('U2', 6.0, 11.0, 1.5, 1.5, npads=3, nets=(5, 6, 7)),
                  _part('R8', 27.0, 2.0, 0.4, 0.3, npads=2, nets=(8, 8)),
                  _part('R9', 29.0, 12.5, 0.4, 0.3, npads=2, nets=(8, 8))],
                 size=PL_SIZE)


def plain_intent(wd, tag='pl'):
    return load({"schema": 1, "kind": "floorplan-intent", "units": "mm",
                 "envelope": {"rect": [0.0, 0.0, PL_SIZE[0], PL_SIZE[1]],
                              "tolerance_mm": 0.5},
                 "blocks": [{"name": "all",
                             "refs": ["CON2", "U1", "U2"]}]}, wd, tag)


def reseat(bpath, intent, refs, **kw):
    kw.setdefault('clearance', 0.2)
    kw.setdefault('board_edge_clearance', 0.5)
    kw.setdefault('grid_step', 0.1)
    kw.setdefault('seed', 0)
    return seeder.reseat_scope(parse_kicad_pcb(bpath), bpath, intent,
                               refs=refs, group_sources=(), **kw)


def graded_errors(bpath, intent, rule=None, ref=None):
    r = floorplan.grade(intent, parse_kicad_pcb(bpath), bpath)
    return [v for v in r.errors
            if (rule is None or v.rule == rule)
            and (ref is None or v.ref == ref)]


def basis_term(basis, name):
    return next((t for t in (basis.get('terms') or [])
                 if t['term'] == name), {})


# --------------------------------------------------------------------------
# arms
# --------------------------------------------------------------------------

def arm_A_keepout_accepted(wd):
    """The headline: a legal on-board part a declared keep-out says is
    misplaced is re-seated, and the basis is the INTENT count."""
    print("--- A: the keep-out fixture is accepted, on the intent basis")
    b = keepout_board(wd, 'a')
    _p, it = keepout_intent(wd, 'a')

    before_err = graded_errors(b, it, rule='keepout', ref='U1')
    check("control: the input board GRADES as a keep-out violation",
          len(before_err) == 1, f"{len(before_err)} keepout error(s) on U1")

    rep = reseat(b, it, ['U1'])
    ab = rep['accept_basis']
    check("the pass is accepted", rep['accepted'] is True,
          f"accepted={rep['accepted']} notes={rep['notes']}")
    check("the basis that fired is `intent`", ab.get('fired') == 'intent',
          f"fired={ab.get('fired')}")
    t = basis_term(ab, 'intent')
    check("the intent basis went 1 -> 0",
          (t.get('before'), t.get('after')) == (1, 0),
          f"intent {t.get('before')} -> {t.get('after')}")
    check("the policy is the explicit one",
          ab.get('policy') == 'explicit:one-term-strict', str(ab.get('policy')))
    check("U1 actually moved", rep['reseated'] == ['U1'], str(rep['reseated']))

    # The board the pass would WRITE must grade clean -- the check that makes
    # the acceptance mean something rather than merely happen.
    out = os.path.join(wd, 'a_out.kicad_pcb')
    write_placed_output(b, out, rep['moves'])
    after_err = graded_errors(out, it, rule='keepout')
    check("the WRITTEN board grades clean on keepout", not after_err,
          f"{len(after_err)} keepout error(s) after")


def arm_B_hpwl_licence(wd):
    """The escape is hpwl-WORSE and is accepted anyway. A lexicographic
    `after <= before` reads hpwl at index 5 and would refuse exactly this."""
    print("--- B: hpwl is the one LICENSED term")
    b = keepout_board(wd, 'b')
    _p, it = keepout_intent(wd, 'b')
    rep = reseat(b, it, ['U1'])
    ab = rep['accept_basis']
    hp = _idx('hpwl')
    rose = rep['gate_after'][hp] - rep['gate_before'][hp]
    check("the accepted seat RAISED hpwl", rose > 1e-9,
          f"hpwl {rep['gate_before'][hp]} -> {rep['gate_after'][hp]} "
          f"(+{rose:.3f})")
    check("accept_basis records the licence and the term it names",
          (ab.get('safety') or {}).get('licensed') == 'hpwl',
          str((ab.get('safety') or {}).get('licensed')))
    check("the safety half still passed", (ab.get('safety') or {}).get('ok'),
          str(ab.get('safety')))
    check("`hpwl` is NOT among the bases it may be accepted on",
          'hpwl' not in seeder.RESEAT_BASES, str(seeder.RESEAT_BASES))
    # The counterfactual, stated as a measurement: a lexicographic rule refuses.
    lex = tuple(rep['gate_after']) <= tuple(rep['gate_before'])
    check("a LEXICOGRAPHIC safety rule would have refused this seat", not lex,
          f"after<=before is {lex} -- which is why the rule is term-wise")


def _idx(name):
    return reconstruct.GATE_TERMS.index(name)


def arm_C_prune_is_the_other_mechanism(wd):
    """M2 alone reverts the seat. Proven by running the sweep BOTH ways on the
    same seated state -- not by reasoning about it."""
    print("--- C: prune_assignment is an independent reverting mechanism")
    b = keepout_board(wd, 'c')
    _p, it = keepout_intent(wd, 'c')

    seen = {}
    real = reconstruct.prune_assignment

    def spy(state, old, notes=None, **kw):
        # Snapshot the seated poses, then run the sweep twice from the same
        # state: once WITHOUT the probe (what shipped) and once with it.
        seated = {r: (state.parts[r].x, state.parts[r].y, state.parts[r].rot)
                  for r in old}
        probe = kw.get('intent_probe')
        kw_noprobe = dict(kw, intent_probe=None)
        seen['without'] = list(real(state, old, None, **kw_noprobe))
        for r, (x, y, rot) in seated.items():       # restore the seated poses
            state.apply_move(r, x, y, rot)
        seen['with'] = list(real(state, old, notes, **kw))
        seen['had_probe'] = probe is not None
        return seen['with']

    reconstruct.prune_assignment = spy
    try:
        rep = reseat(b, it, ['U1'])
    finally:
        reconstruct.prune_assignment = real

    check("the reseat DID hand prune an intent probe", seen.get('had_probe'),
          str(seen.get('had_probe')))
    check("WITHOUT the probe, prune reverts the seat",
          seen.get('without') == ['U1'], f"pruned={seen.get('without')}")
    check("WITH the probe, prune keeps it", seen.get('with') == [],
          f"pruned={seen.get('with')}")
    check("the KEPT move is disclosed in a note",
          any('KEPT' in n and 'U1' in n for n in rep['notes']),
          "; ".join(n for n in rep['notes'] if 'prune' in n))


def arm_D_prune_probe_inert(wd):
    """`intent_probe=None` is the original expression, character for
    character. Asserted behaviourally, since that is what callers rely on."""
    print("--- D: prune_assignment is inert without a probe")
    b = plain_board(wd, 'd')
    _p, it = plain_intent(wd, 'd')
    pcb = parse_kicad_pcb(b)
    import pose_score
    st = pose_score.make_state(pcb, b, clearance=0.2,
                               board_edge_clearance=0.5, grid_step=0.1)
    old = {'CON2': (st.parts['CON2'].x, st.parts['CON2'].y,
                    st.parts['CON2'].rot)}
    st.apply_move('CON2', 10.0, 6.0, 0.0)          # a move prune should revert
    a = reconstruct.prune_assignment(st, dict(old), None, edge_bands={})
    st.apply_move('CON2', 10.0, 6.0, 0.0)
    bnone = reconstruct.prune_assignment(st, dict(old), None, edge_bands={},
                                         intent_probe=None)
    check("omitting the kwarg and passing None agree", a == bnone,
          f"{a} vs {bnone}")
    st.apply_move('CON2', 10.0, 6.0, 0.0)
    empty = reconstruct.prune_assignment(st, dict(old), None, edge_bands={},
                                         intent_probe=lambda _r: ())
    check("an EMPTY probe vector is also inert", empty == a,
          f"{empty} vs {a}")


def arm_E_plain_board_hpwl_basis(wd):
    """The claim-free board: no zone, no keep-out, `oob` immovable. Only the
    scope's own wirelength can carry it -- so this proves the fix is not
    intent-dependent."""
    print("--- E: a board with NO declared claim, accepted on scope_hpwl")
    b = plain_board(wd, 'e')
    _p, it = plain_intent(wd, 'e')
    check("control: the board declares nothing this can gate on",
          not graded_errors(b, it), "0 grade errors on the input")
    rep = reseat(b, it, ['CON2'])
    ab = rep['accept_basis']
    check("the pass is accepted", rep['accepted'] is True,
          f"accepted={rep['accepted']}")
    check("the basis that fired is `scope_hpwl`",
          ab.get('fired') == 'scope_hpwl', str(ab.get('fired')))
    t = basis_term(ab, 'scope_hpwl')
    check("and it is a real relocation, not a shuffle",
          (t.get('before', 0) - t.get('after', 0)) > 1.0,
          f"scope_hpwl {t.get('before')} -> {t.get('after')}")
    check("the intent basis measured, and measured nothing",
          basis_term(ab, 'intent').get('before') == 0,
          f"intent={basis_term(ab, 'intent')}")
    check("`oob` did not move, which is why the OLD rule refused this",
          basis_term(ab, 'oob').get('before')
          == basis_term(ab, 'oob').get('after') == 0.0,
          f"oob={basis_term(ab, 'oob')}")

    # `scope_hpwl` must be the SCOPE's nets, not the board's. Net 8 (R8<->R9)
    # touches no scope ref, so the two numbers must differ -- without this the
    # subset argument to `state.hpwl` could be ignored undetected.
    import pose_score
    pcb = parse_kicad_pcb(b)
    st = pose_score.make_state(pcb, b, clearance=0.2,
                               board_edge_clearance=0.5, grid_step=0.1)
    board_wide = round(st.hpwl(), 3)
    scoped = seeder.scope_hpwl(st, {'CON2'})
    check("scope_hpwl is strictly less than the board-wide hpwl",
          scoped < board_wide - 1e-9,
          f"scope {scoped} vs board {board_wide} -- net 8 is outside the scope")
    check("and the basis reports the SCOPED number",
          abs(basis_term(ab, 'scope_hpwl').get('before', 0) - scoped) < 1e-6,
          f"basis before={basis_term(ab, 'scope_hpwl').get('before')} "
          f"scope_hpwl={scoped}")


def arm_F_min_gain(wd):
    """`--reseat-min-gain` gates the mm basis and NOT the count bases."""
    print("--- F: min_gain gates the millimetre basis only")
    b = plain_board(wd, 'f')
    _p, it = plain_intent(wd, 'f')
    lo = reseat(b, it, ['CON2'], min_gain=0.0)
    hi = reseat(b, it, ['CON2'], min_gain=1000.0)
    check("control: at min_gain 0 the scope_hpwl basis carries it",
          lo['accepted'] and lo['accept_basis']['fired'] == 'scope_hpwl',
          f"fired={lo['accept_basis']['fired']}")
    check("above the gain, the same pass is REFUSED",
          hi['accepted'] is False, f"accepted={hi['accepted']}")
    check("and the refusal names the min-gain, not the off-board amount",
          any('reseat-min-gain' in n for n in hi['notes']),
          "; ".join(n for n in hi['notes'] if 'REVERTED' in n)[:140])
    check("the gated basis is marked as gated, the count bases are not",
          (basis_term(hi['accept_basis'], 'scope_hpwl')['min_gain_applies']
           is True
           and basis_term(hi['accept_basis'], 'intent')['min_gain_applies']
           is False),
          "scope_hpwl gated, intent not")

    # A COUNT basis must be untouched by a millimetre threshold -- the whole
    # reason there is one flag and not one number per currency.
    kb = keepout_board(wd, 'f2')
    _p2, it2 = keepout_intent(wd, 'f2')
    k = reseat(kb, it2, ['U1'], min_gain=1000.0)
    check("a huge mm threshold does NOT block the intent COUNT basis",
          k['accepted'] and k['accept_basis']['fired'] == 'intent',
          f"accepted={k['accepted']} fired={k['accept_basis'].get('fired')}")


def arm_G_auto_scope_unchanged(wd):
    """The auto scope keeps the verbatim rule, and the new machinery is not
    merely equal there -- it is UNREACHABLE. A poison probe proves it.

    The fixture needs a genuinely OFF-OUTLINE part, or the auto census is empty
    and `reseat_scope` returns through `_empty` without ever reaching the gate
    -- which is correct behaviour on a healthy board (the census is zero on all
    33 corpus boards) and would make this arm assert nothing.
    """
    print("--- G: the auto scope is untouched, and provably so")
    b = board(os.path.join(wd, 'g.kicad_pcb'),
              [_part('U9', 20.6, 6.0, 0.5, 0.5, npads=2, nets=(1, 2)),
               _part('R9', 6.0, 6.0, 0.4, 0.3, npads=2, nets=(1, 2))],
              size=KO_SIZE)
    _p, it = load({"schema": 1, "kind": "floorplan-intent", "units": "mm",
                   "envelope": {"rect": [0.0, 0.0, KO_SIZE[0], KO_SIZE[1]],
                                "tolerance_mm": 0.5},
                   "blocks": [{"name": "all", "refs": ["U9", "R9"]}]},
                  wd, 'g')

    rep = reseat(b, it, None)                      # refs=None -> auto
    check("control: the fixture HAS an off-outline part, so the auto census "
          "is non-empty", rep['scope'] == ['U9'],
          f"scope={rep['scope']} witnesses={rep['witnesses_before']}")
    check("the scope source is the auto census",
          rep['scope_source'] == 'auto:damage_witnesses', rep['scope_source'])
    ab = rep['accept_basis']
    check("the auto policy is the old one", ab.get('policy') == 'auto:oob-strict',
          str(ab.get('policy')))
    check("the auto path measured no safety half and no intent licence",
          ab.get('safety') is None and ab.get('intent_licence') is None,
          "both None -- 'not measured' must not look like 'measured clean'")
    check("only the oob term is reported",
          [t['term'] for t in ab['terms']] == ['oob'],
          str([t['term'] for t in ab['terms']]))

    real_init = q.IntentProbe.__init__

    def poison(self, *a, **kw):
        raise AssertionError("IntentProbe was built on the AUTO path")

    q.IntentProbe.__init__ = poison
    try:
        rep2 = reseat(b, it, None)
        reached = False
    except AssertionError:
        reached = True
    finally:
        q.IntentProbe.__init__ = real_init
    check("a POISONED probe is never constructed on the auto path", not reached,
          "the auto branch cannot reach it, which no output diff can show")
    check("and the poisoned run agrees with the clean one",
          not reached and rep2['gate_after'] == rep['gate_after'],
          "gate tuples identical")


def arm_H_seat_gate_stays_disarmed(wd):
    """The probe must not arm the per-pose zone gate. `pose_score.make_state`
    withholds `intent_zones` because a monotone gate would make the re-seat
    refuse its own target; measuring is not gating."""
    print("--- H: measuring the zones does not ARM them")
    b = keepout_board(wd, 'h')
    _p, it = keepout_intent(wd, 'h')

    seen = {}
    real = q.IntentProbe.__init__

    def spy(self, state, zones=(), refs=None):
        real(self, state, zones=zones, refs=refs)
        seen['state_spec'] = dict(state._intent_spec)
        seen['probe_spec'] = {r: len(v) for r, v in self.spec.items()}
        seen['state_obj'] = state

    q.IntentProbe.__init__ = spy
    try:
        reseat(b, it, ['U1'])
    finally:
        q.IntentProbe.__init__ = real

    check("the STATE's own zone spec stayed empty",
          seen.get('state_spec') == {}, str(seen.get('state_spec')))
    check("while the PROBE bound U1's keep-out",
          seen.get('probe_spec', {}).get('U1') == 1,
          str(seen.get('probe_spec')))

    # A source guard, because the tempting "simplification" is to hand
    # `intent_zones=` to make_state and re-open the bug pose_score describes.
    #
    # Asserted on the SHAPE, not on the absence of a token. The first version
    # was `'intent_zones=' not in src` over the whole file, which is both too
    # broad (a comment mentioning the token fails it) and too weak (`**kw`, an
    # alias, or `intent_zones =` with spaces passes it). Walk the AST and look
    # at what `pose_score.make_state` is actually CALLED with.
    import ast
    src = open(os.path.join(REPO, 'py_placer', 'placement', 'seeder.py'),
               encoding='utf-8').read()
    calls, offenders, starstar = 0, [], 0
    for node in ast.walk(ast.parse(src)):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        if not (isinstance(fn, ast.Attribute) and fn.attr == 'make_state'):
            continue
        calls += 1
        for kw in node.keywords:
            if kw.arg is None:
                starstar += 1           # **kw could smuggle it in
            elif kw.arg == 'intent_zones':
                offenders.append(node.lineno)
    check("seeder.py calls pose_score.make_state at all", calls > 0,
          f"{calls} call site(s) found by AST")
    check("and no call passes intent_zones=", not offenders,
          f"offending lines: {offenders or 'none'}")
    check("and none of them splats **kwargs, which could smuggle it in",
          starstar == 0, f"{starstar} **kwargs splat(s)")
    qsrc = open(os.path.join(REPO, 'py_placer', 'placement', 'quench.py'),
                encoding='utf-8').read()
    cls = qsrc[qsrc.index('class IntentProbe'):qsrc.index('def rect_gap')
               if 'def rect_gap' in qsrc[qsrc.index('class IntentProbe'):]
               else len(qsrc)]
    body = qsrc[qsrc.index('class IntentProbe'):]
    body = body[:body.index('\n\n\n')] if '\n\n\n' in body else body
    check("IntentProbe assigns to neither _intent_spec nor _intent_active",
          ('_intent_spec =' not in body and '_intent_active =' not in body),
          f"{len(body.splitlines())} lines scanned")
    del cls


def arm_I_intent_vector_is_the_guard(wd):
    """The COUNT cannot see a keep-out A -> B hop; the VECTOR can. This is the
    trap `_IntentTerm` names, at pass level."""
    print("--- I: the vector is the guard, the count only the trigger")
    b = keepout_board(wd, 'i')
    _p, it = keepout_intent(wd, 'i')
    pcb = parse_kicad_pcb(b)
    bundle, _pr = floorplan.resolve_intent_gate(it, pcb, ())
    import pose_score
    st = pose_score.make_state(pcb, b, clearance=0.2,
                               board_edge_clearance=0.5, grid_step=0.1,
                               keepouts=it.keepouts)
    probe = q.IntentProbe(st, zones=bundle['zones'], refs={'U1'})
    check("control: U1 is bound by exactly one keep-out term",
          len(probe.spec.get('U1', ())) == 1, str(probe.spec.get('U1')))

    inside = probe.snapshot()
    # Hand-build the A -> B hop the count cannot see: two terms, one leaves,
    # one enters, count 1 -> 1.
    hop_b = {'count': 1, 'by_rule': {'keepout': 1},
             'terms': {'U1': (5.0, 0.0)}}
    hop_a = {'count': 1, 'by_rule': {'keepout': 1},
             'terms': {'U1': (0.0, 5.0)}}
    probe.spec['U1'] = probe.spec['U1'] + probe.spec['U1']   # 2 terms
    ok, risen = probe.licence(hop_b, hop_a)
    check("the COUNT is unchanged across the hop",
          hop_b['count'] == hop_a['count'] == 1, "1 -> 1")
    check("but the licence REFUSES it, naming the term that rose",
          ok is False and risen and risen[0][1] == 'keepout',
          f"risen={risen}")
    check("and a clean vector passes the same licence",
          probe.licence(inside, inside)[0] is True, "no term rose")


def arm_J_hard_terms(wd):
    """`reseat_safety_ok` is pure policy, so every licensed/hard term is
    testable directly -- `eviction_licence_ok`'s own shape and reason."""
    print("--- J: the term-wise licence, one assertion per term")
    n = len(reconstruct.GATE_TERMS)
    base = [0, 0, 0.0, 0.0, 0, 10.0, 0.0]
    check("control: an unchanged tuple is safe",
          seeder.reseat_safety_ok(base, list(base)) == (True, []),
          str(seeder.reseat_safety_ok(base, list(base))))
    for term in reconstruct.GATE_TERMS:
        worse = list(base)
        i = _idx(term)
        worse[i] = base[i] + (1 if isinstance(base[i], int) else 1.0)
        ok, rose = seeder.reseat_safety_ok(base, worse)
        if term == seeder.RESEAT_LICENSED_TERM:
            check(f"{term} may worsen (it is the licensed term)",
                  ok is True and rose == [], f"ok={ok} rose={rose}")
        else:
            check(f"{term} may NOT worsen", ok is False and rose == [term],
                  f"ok={ok} rose={rose}")
    check("every gate term is either hard or the licensed one",
          n == len([t for t in reconstruct.GATE_TERMS
                    if t == seeder.RESEAT_LICENSED_TERM
                    or not seeder.reseat_safety_ok(
                        base, [b + (1 if isinstance(b, int) else 1.0)
                               if j == _idx(t) else b
                               for j, b in enumerate(base)])[0]]),
          f"{n} terms, none unclassified")


def arm_K_schema_parity(wd):
    """`_empty` and the seated path must return the same keys. This function
    shipped a schema split once already."""
    print("--- K: both return paths carry the same keys")
    b = keepout_board(wd, 'k')
    _p, it = keepout_intent(wd, 'k')
    seated = reseat(b, it, ['U1'])
    empty = reseat(b, it, ['NOSUCHREF*'])
    check("the no-scope path is a RESULT, not an error",
          empty['accepted'] is True and empty['scope'] == [],
          f"accepted={empty['accepted']} scope={empty['scope']}")
    missing = sorted(set(seated) - set(empty))
    check("the early-out carries every key the seated path does",
          missing == ['intent_used'] or not missing,
          f"missing={missing}")
    check("including accept_basis", 'accept_basis' in empty,
          f"policy={empty['accept_basis'].get('policy')}")
    check("and its policy says which path it was",
          empty['accept_basis'].get('policy') == 'empty',
          str(empty['accept_basis'].get('policy')))


def arm_L_agreement_with_the_grade(wd):
    """Over a pose lattice, the probe's breach count and `floorplan.grade`
    must agree. If they ever disagree, the re-seat is being judged against a
    rule the grader does not have."""
    print("--- L: the probe agrees with the grade, pose by pose")
    _p, it = keepout_intent(wd, 'l')
    import pose_score
    disagree = []
    n = 0
    for ux in [x * 0.5 for x in range(4, 33)]:
        for uy in [y * 0.5 for y in range(4, 21)]:
            bp = keepout_board(wd, 'l_probe', ux=ux, uy=uy)
            pcb = parse_kicad_pcb(bp)
            bundle, _pr = floorplan.resolve_intent_gate(it, pcb, ())
            st = pose_score.make_state(pcb, bp, clearance=0.2,
                                       board_edge_clearance=0.5,
                                       grid_step=0.1, keepouts=it.keepouts)
            pr = q.IntentProbe(st, zones=bundle['zones'], refs={'U1'})
            mine = pr.snapshot()['count']
            theirs = len([v for v in graded_errors(bp, it, ref='U1')
                          if v.rule in INTENT_ENFORCED_RULES])
            n += 1
            if mine != theirs:
                disagree.append((ux, uy, mine, theirs))
    check(f"probe and grade agree on all {n} poses", not disagree,
          f"{len(disagree)} disagreement(s)"
          + (f", first {disagree[0]}" if disagree else ""))
    check("and the lattice actually straddles the keep-out",
          n > 100, f"{n} poses swept over a {KO_SIZE[0]}x{KO_SIZE[1]} board")


def arm_M_seed_independence(wd):
    """Is the keep-out fixture's behaviour a property of the BOARD or of one
    seed? The first version of this fixture failed this arm 9/20."""
    print("--- M: the fixture is seed-independent")
    b = keepout_board(wd, 'm')
    _p, it = keepout_intent(wd, 'm')
    acc = 0
    bases = set()
    for s in range(8):
        rep = reseat(b, it, ['U1'], seed=s)
        acc += bool(rep['accepted'])
        bases.add((rep['accept_basis'] or {}).get('fired'))
    check("accepted on every seed", acc == 8, f"{acc}/8 seeds accepted")
    check("and always on the same basis", bases == {'intent'}, str(bases))


def arm_N_enforced_rules_covered(wd):
    """Every rule the engine claims to enforce is exercised here, and the
    vocabulary is read FROM the engine so a new rule cannot be forgotten."""
    print("--- N: the enforced-rule vocabulary is pinned both ways")
    check("INTENT_ENFORCED_RULES is the expected three",
          set(INTENT_ENFORCED_RULES) == {'zone_containment', 'zone_exclusive',
                                         'keepout'},
          str(INTENT_ENFORCED_RULES))
    check("every enforced rule is a real floorplan rule",
          all(r in dict(floorplan.RULES) for r in INTENT_ENFORCED_RULES),
          "all present in floorplan.RULES")
    check("RESEAT_BASES has a unit for every basis",
          set(seeder.RESEAT_BASES) == set(seeder.RESEAT_BASIS_UNITS),
          str(sorted(set(seeder.RESEAT_BASES)
                     ^ set(seeder.RESEAT_BASIS_UNITS))))
    check("the licensed term is not also a basis",
          seeder.RESEAT_LICENSED_TERM not in seeder.RESEAT_BASES,
          f"{seeder.RESEAT_LICENSED_TERM} absent from RESEAT_BASES")
    non_gate = [b for b in seeder.RESEAT_BASES
                if b not in reconstruct.GATE_TERMS]
    check("the only non-tuple bases are the two this change adds",
          sorted(non_gate) == ['intent', 'scope_hpwl'], str(non_gate))


def arm_O_zone_containment_basis(wd):
    """A zone, not a keep-out: the other enforced rule that can bind a scope
    ref, through the same measurement."""
    print("--- O: a declared ZONE drives the same basis")
    b = board(os.path.join(wd, 'o.kicad_pcb'),
              [_part('U1', 30.0, 12.0, 1.0, 1.0, npads=2, nets=(1, 2)),
               _part('R1', 6.0, 6.0, 0.4, 0.3, npads=2, nets=(1, 2))],
              size=(40.0, 24.0))
    _p, it = load({"schema": 1, "kind": "floorplan-intent", "units": "mm",
                   "envelope": {"rect": [0.0, 0.0, 40.0, 24.0],
                                "tolerance_mm": 0.5},
                   "blocks": [{"name": "z", "refs": ["U1"],
                               "zone": [3.0, 3.0, 11.0, 11.0],
                               "tolerance_mm": 0.5}]}, wd, 'o')
    check("control: U1 starts outside its declared zone",
          len(graded_errors(b, it, rule='zone_containment', ref='U1')) == 1,
          "1 zone_containment error on the input")
    rep = reseat(b, it, ['U1'])
    ab = rep['accept_basis']
    check("the pass is accepted on the intent basis",
          rep['accepted'] and ab.get('fired') == 'intent',
          f"accepted={rep['accepted']} fired={ab.get('fired')}")
    check("the by-rule split names zone_containment",
          basis_term(ab, 'intent').get('before') == 1,
          f"intent {basis_term(ab, 'intent')}")
    out = os.path.join(wd, 'o_out.kicad_pcb')
    write_placed_output(b, out, rep['moves'])
    check("and the written board grades clean",
          not graded_errors(out, it, rule='zone_containment'),
          "0 zone_containment errors after")


def arm_P_end_to_end_cli(wd):
    """Through the real CLI, because everything above is in-process. A
    non-zero exit is not evidence -- assert the REASON."""
    print("--- P: the CLI, refusing and accepting for stated reasons")
    seed_py = os.path.join(REPO, 'py_placer', 'place_seed.py')
    b = keepout_board(wd, 'p')
    ipath, _it = keepout_intent(wd, 'p')
    run_utils.evidence(b, 'the reproducer board')
    run_utils.evidence(ipath, 'the intent')

    out = os.path.join(wd, 'p_out.kicad_pcb')
    r = run_utils.check([sys.executable, '-X', 'utf8', seed_py, b, out,
                         '--intent', ipath, '--clearance', '0.2',
                         '--board-edge-clearance', '0.5', '--reseat', 'U1'],
                        accept=True)
    check("the CLI accepts and names the basis on stdout",
          'accepted on intent' in r.stdout,
          [ln.strip() for ln in r.stdout.splitlines()
           if 'accepted on' in ln][:1])
    summ = json.loads([ln for ln in r.stdout.splitlines()
                       if ln.startswith('JSON_SUMMARY:')][-1]
                      .split('JSON_SUMMARY:', 1)[1])
    check("JSON_SUMMARY carries accept_basis",
          (summ.get('accept_basis') or {}).get('fired') == 'intent',
          str((summ.get('accept_basis') or {}).get('fired')))
    check("and the min-gain it was run at",
          summ.get('reseat_min_gain') == 0.0, str(summ.get('reseat_min_gain')))
    check("grade_errors reached 0", summ.get('grade_errors') == 0,
          str(summ.get('grade_errors')))

    # The refusal, for a STATED reason rather than a bare non-zero exit.
    pb = plain_board(wd, 'p2')
    ip2, _it2 = plain_intent(wd, 'p2')
    run_utils.check([sys.executable, '-X', 'utf8', seed_py, pb,
                     os.path.join(wd, 'p2_out.kicad_pcb'), '--intent', ip2,
                     '--clearance', '0.2', '--board-edge-clearance', '0.5',
                     '--reseat', 'CON2', '--reseat-min-gain', '1000'],
                    refuse='reseat-min-gain', code=4)
    check("a sub-threshold win is refused NAMING the min-gain", True,
          "exit 4 with 'reseat-min-gain' in the output")
    run_utils.check([sys.executable, '-X', 'utf8', seed_py, pb,
                     os.path.join(wd, 'p3_out.kicad_pcb'), '--intent', ip2,
                     '--reseat-min-gain', '-1', '--reseat', 'CON2'],
                    refuse='magnitude in mm', code=2)
    check("a negative min-gain is an argparse refusal, not a looser bar", True,
          "exit 2")


def arm_Q_the_conjuncts_compose(wd):
    """`reseat_accept` is pure policy, so each conjunct is testable directly --
    which is the only way to reach the ones a legal seat search cannot produce.

    Every row here exists because a mutation survived without it: deleting
    `safe`, deleting `not risen` and deleting `witness_ok` from the conjunct
    all passed the behavioural arms, since a legal seat never worsens a hard
    term and the fixtures' claims never rise.
    """
    print("--- Q: each conjunct refuses on its own")
    idx = reconstruct.GATE_TERMS.index
    before = [0, 0, 0.0, 0.0, 0, 10.0, 0.0]
    after = list(before)
    bb = {n: 0.0 for n in seeder.RESEAT_BASES}
    bb['intent'] = 1
    ba = dict(bb, intent=0)                 # one whole defect cleared

    def acc(**kw):
        kw.setdefault('scope_source', 'explicit')
        kw.setdefault('witnesses_before', ['A'])
        kw.setdefault('witnesses_after', ['A'])
        kw.setdefault('bases_before', bb)
        kw.setdefault('bases_after', ba)
        return seeder.reseat_accept(kw.pop('before', before),
                                    kw.pop('after', after), **kw)

    ok, basis = acc()
    check("control: safe, no risen claim, one basis fired -> ACCEPTED",
          ok is True and basis['fired'] == 'intent',
          f"accepted={ok} fired={basis['fired']}")

    worse = list(after)
    worse[idx('stacks')] = before[idx('stacks')] + 1
    ok2, b2 = acc(after=worse)
    check("a worsened HARD term refuses a pass whose basis fired",
          ok2 is False and b2['safety']['worsened'] == ['stacks'],
          f"accepted={ok2} worsened={b2['safety']['worsened']}")
    check("and the refusal note names the term, not the off-board amount",
          'stacks' in seeder.reseat_refusal_note(1, b2)
          and 'off-board amount' not in seeder.reseat_refusal_note(1, b2),
          seeder.reseat_refusal_note(1, b2)[:110])

    lic = list(after)
    lic[idx('hpwl')] = before[idx('hpwl')] + 5.0
    ok3, _b3 = acc(after=lic)
    check("the LICENSED term worsening does not refuse it", ok3 is True,
          f"accepted={ok3} with hpwl +5.0")

    ok4, b4 = acc(intent_risen=[('U1', 'keepout', 'hot', 0.0, 5.0)])
    check("a RISEN declared claim refuses it even though the count improved",
          ok4 is False and b4['intent_licence']['ok'] is False,
          f"accepted={ok4} risen={b4['intent_licence']['risen']}")
    check("and that refusal names the claim",
          "'hot'" in seeder.reseat_refusal_note(1, b4),
          seeder.reseat_refusal_note(1, b4)[:110])

    ok5, b5 = acc(witnesses_after=['A', 'B'])
    check("a GROWN off-outline count refuses it",
          ok5 is False and b5['witness_ok'] is False,
          f"accepted={ok5} witness_ok={b5['witness_ok']}")
    check("and that refusal names the count",
          'GREW' in seeder.reseat_refusal_note(1, b5),
          seeder.reseat_refusal_note(1, b5)[:110])

    ok6, b6 = acc(bases_after=dict(bb))     # nothing improved
    check("no basis firing refuses it", ok6 is False and b6['fired'] is None,
          f"accepted={ok6} fired={b6['fired']}")

    # The auto branch, on the same numbers, must be the OLD rule.
    ok7, b7 = acc(scope_source='auto:damage_witnesses')
    check("the auto branch ignores every basis and reads oob only",
          ok7 is False and b7['policy'] == 'auto:oob-strict',
          f"accepted={ok7} policy={b7['policy']} (oob did not move)")
    moved = list(after)
    moved[idx('oob')] = before[idx('oob')] - 1.0
    ok8, _b8 = acc(after=moved, scope_source='auto:damage_witnesses')
    check("and accepts on a strict oob improvement alone", ok8 is True,
          "oob 0.0 -> -1.0")


def arm_R_prune_still_reverts_a_claim_bound_part(wd):
    """The probe is a CONJUNCT, not an exemption: a part bound by a claim is
    still pruned when the revert does not re-break anything.

    Without this, `undoes_intent = bool(after_intent)` -- refuse EVERY revert
    of any claim-bound part -- passes the whole suite, and the sweep silently
    stops doing its job for exactly the parts this change touches.
    """
    print("--- R: a claim-bound part is still prunable")
    b = keepout_board(wd, 'r')
    _p, it = keepout_intent(wd, 'r')
    pcb = parse_kicad_pcb(b)
    bundle, _pr = floorplan.resolve_intent_gate(it, pcb, ())
    import pose_score
    st = pose_score.make_state(pcb, b, clearance=0.2,
                               board_edge_clearance=0.5, grid_step=0.1,
                               keepouts=it.keepouts)
    probe = q.IntentProbe(st, zones=bundle['zones'], refs={'U1'})

    # Both poses are OUTSIDE the keep-out, so the claim vector is 0 either way
    # -- the revert cannot re-break anything -- but the `old` pose is nearer
    # U1's net partners, so the tuple wants it back.
    home = (13.5, 6.0, 0.0)
    away = (18.0, 11.0, 0.0)
    st.apply_move('U1', *home)
    check("control: the 'home' pose breaches nothing", probe.terms('U1') == (0.0,),
          f"terms={probe.terms('U1')}")
    st.apply_move('U1', *away)
    check("control: the 'away' pose breaches nothing either",
          probe.terms('U1') == (0.0,), f"terms={probe.terms('U1')}")

    pruned = reconstruct.prune_assignment(
        st, {'U1': home}, None, edge_bands={}, intent_probe=probe.terms)
    check("prune reverts it, probe or no probe", pruned == ['U1'],
          f"pruned={pruned}")
    check("and the revert actually landed",
          abs(st.parts['U1'].x - home[0]) < 1e-9, f"x={st.parts['U1'].x}")


def arm_S_review_findings(wd):
    """Every defect the pre-push review found, pinned so it cannot come back.

    All five were in code this change added, and none was reachable from the
    behavioural arms above -- which is the argument for the review, not against
    the arms.
    """
    print("--- S: the pre-push review's findings, pinned")
    idx = reconstruct.GATE_TERMS.index
    base = [0, 0, 0.0, 0.0, 0, 10.0, 0.0]
    bb = {n: 0.0 for n in seeder.RESEAT_BASES}

    # SF-1: an evicted AUTO pass used to print both conjuncts as SATISFIED in
    # the sentence giving them as the reason for refusing.
    _ok, ab = seeder.reseat_accept(
        [0, 0, 0.0, 9.65, 0, 10.0, 0.0], [0, 0, 0.0, 0.0, 0, 10.0, 0.0],
        scope_source='auto:damage_witnesses',
        witnesses_before=['A', 'B'], witnesses_after=[])
    ab['eviction_licence'] = False
    note = seeder.reseat_refusal_note(1, ab)
    check("SF-1: an evicted auto refusal blames the eviction licence",
          'eviction licence' in note and 'strictly improves' not in note,
          note[:100])

    # SF-2: `_empty`'s basis must carry the seated path's key set, or
    # `reseat_refusal_note` KeyErrors on it.
    b = keepout_board(wd, 's')
    _p, it = keepout_intent(wd, 's')
    seated = reseat(b, it, ['U1'])
    empty = reseat(b, it, ['NOSUCHREF*'])
    missing = sorted(set(seated['accept_basis']) - set(empty['accept_basis']))
    check("SF-2: the early-out's accept_basis has the same keys", not missing,
          f"missing={missing or 'none'} ({len(empty['accept_basis'])} keys)")
    try:
        seeder.reseat_refusal_note(0, empty['accept_basis'])
        note_ok = True
    except KeyError as e:                                    # noqa: BLE001
        note_ok = f"KeyError {e}"
    check("SF-2: and the refusal note renders against it", note_ok is True,
          str(note_ok))

    # SF-3: a gain of EXACTLY --reseat-min-gain must count; the help calls it
    # "the smallest win that counts".
    ba = dict(bb, scope_hpwl=-0.5)          # gain of exactly 0.5
    for mg, want in ((0.5, True), (0.5001, False)):
        ok, _ = seeder.reseat_accept(
            base, list(base), scope_source='explicit',
            witnesses_before=[], witnesses_after=[],
            bases_before=bb, bases_after=ba, min_gain=mg)
        check(f"SF-3: gain 0.5 against --reseat-min-gain {mg} -> "
              f"{'accept' if want else 'refuse'}", ok is want, f"accepted={ok}")

    # N-1: a continuous legality basis must not fire on rounding noise.
    for delta, want in ((1e-4, False), (2e-3, True)):
        ok, _ = seeder.reseat_accept(
            base, list(base), scope_source='explicit',
            witnesses_before=[], witnesses_after=[],
            bases_before=dict(bb, overlap=0.5),
            bases_after=dict(bb, overlap=0.5 - delta))
        check(f"N-1: an overlap gain of {delta:g} mm2 -> "
              f"{'accept' if want else 'refuse'}", ok is want,
              f"accepted={ok} (MEASURE_QUANTUM={seeder.MEASURE_QUANTUM})")

    # SF-4: the KEPT note must not credit the probe for a revert the TUPLE
    # refused on its own. The case needs `undoes_intent` TRUE (the old pose is
    # INSIDE the keep-out, so restoring it re-breaks the claim) while the tuple
    # declines anyway -- here because a blocker sits on the old pose, so the
    # revert would create courtyard overlap. Both halves matter: with only the
    # first, the tuple wants the revert and the mutation is unreachable, which
    # is how the first version of this check passed while testing nothing.
    b4 = board(os.path.join(wd, 's4.kicad_pcb'),
               [_part('U1', 14.0, 2.0, 0.5, 0.5, npads=4, nets=(1, 2, 3, 4)),
                _part('R1', 9.0, 5.0, 0.4, 0.3, npads=2, nets=(1, 2)),
                _part('C1', 11.0, 7.0, 0.4, 0.3, npads=2, nets=(3, 4)),
                _part('BLK', 10.0, 6.0, 1.2, 1.2, npads=2, nets=(7, 8))],
               size=KO_SIZE)
    _p4, it4 = keepout_intent(wd, 's4')
    pcb4 = parse_kicad_pcb(b4)
    bundle4, _pr4 = floorplan.resolve_intent_gate(it4, pcb4, ())
    import pose_score
    st4 = pose_score.make_state(pcb4, b4, clearance=0.2,
                                board_edge_clearance=0.5, grid_step=0.1,
                                keepouts=it4.keepouts)
    probe4 = q.IntentProbe(st4, zones=bundle4['zones'])
    old4 = (10.0, 6.0, 0.0)                 # inside `hot`, and on top of BLK
    check("SF-4 control: the CURRENT pose breaches nothing",
          probe4.terms('U1') == (0.0,), f"terms={probe4.terms('U1')}")
    st4.apply_move('U1', *old4)
    breached = probe4.terms('U1')
    st4.apply_move('U1', 14.0, 2.0, 0.0)
    check("SF-4 control: the OLD pose breaches the keep-out",
          breached and breached[0] > 0, f"terms={breached}")
    notes4 = []
    pruned4 = reconstruct.prune_assignment(
        st4, {'U1': old4}, notes4, edge_bands={}, intent_probe=probe4.terms)
    check("SF-4: the tuple declines the revert on its own", pruned4 == [],
          f"pruned={pruned4}")
    check("SF-4: and no KEPT note credits the intent probe for it",
          not any('KEPT' in n for n in notes4), f"notes={notes4}")

    # SF-6a: the probe `reseat_scope` ACTUALLY BUILDS must cover the whole
    # board, so an evicted stranger cannot be pushed into a keep-out unseen.
    # Asserted at the real call site -- a probe this test constructs proves
    # nothing about the one the engine constructs.
    seen6 = {}
    real6 = q.IntentProbe.__init__

    def spy6(self, state, zones=(), refs=None):
        real6(self, state, zones=zones, refs=refs)
        seen6['refs_arg'] = refs
        seen6['covered'] = set(self.refs)
        seen6['parts'] = set(state.parts)

    q.IntentProbe.__init__ = spy6
    try:
        reseat(b, it, ['U1'])
    finally:
        q.IntentProbe.__init__ = real6
    check("SF-6a: reseat_scope builds the probe over the WHOLE board",
          seen6.get('refs_arg') is None
          and seen6.get('covered') == seen6.get('parts'),
          f"refs={seen6.get('refs_arg')}, "
          f"{len(seen6.get('covered') or ())} of "
          f"{len(seen6.get('parts') or ())} parts covered")
    check("SF-6a: and that is strictly more than the named scope",
          len(seen6.get('covered') or ()) > 1,
          f"scope was ['U1'], probe covers {sorted(seen6.get('covered') or ())}")


def main():
    with tempfile.TemporaryDirectory() as wd:
        arm_A_keepout_accepted(wd)
        arm_B_hpwl_licence(wd)
        arm_C_prune_is_the_other_mechanism(wd)
        arm_D_prune_probe_inert(wd)
        arm_E_plain_board_hpwl_basis(wd)
        arm_F_min_gain(wd)
        arm_G_auto_scope_unchanged(wd)
        arm_H_seat_gate_stays_disarmed(wd)
        arm_I_intent_vector_is_the_guard(wd)
        arm_J_hard_terms(wd)
        arm_K_schema_parity(wd)
        arm_L_agreement_with_the_grade(wd)
        arm_M_seed_independence(wd)
        arm_N_enforced_rules_covered(wd)
        arm_O_zone_containment_basis(wd)
        arm_P_end_to_end_cli(wd)
        arm_Q_the_conjuncts_compose(wd)
        arm_R_prune_still_reverts_a_claim_bound_part(wd)
        arm_S_review_findings(wd)
    print(f"\n{passed}/{passed + failed} checks passed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
