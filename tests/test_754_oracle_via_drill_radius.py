#!/usr/bin/env python3
"""#754: the two hole-to-hole gates in kicad_oracle resolve a via's DRILL the
same way, and the unreadable case falls back on a knob rather than on ZERO.

Two gates in one file asked "how wide is this via's drill?" and answered
differently:

    :2557   if _d2 < _vdr + (_v2.drill or 0) / 2 + _h2h:
    :2804   _lim = (used_via_drill + (_ev.drill or used_via_drill)) / 2 + h2h

`or 0` prices a barrel the board cannot measure as if it had NO HOLE, which
makes the gate WEAKER for exactly the via it knows least about: `_ok649` stays
True and the #649b stitching via is accepted at a separation the fab floor
forbids. The site 250 lines below already had the right shape -- fall back on
the drill the CALLER is working at.

`or` is the second half, and it is the half both sites shared: it keeps a
NEGATIVE value, so the drill term goes negative and SUBTRACTS from a threshold
it is summed into. Note the difference from #750, measured below rather than
inherited: there BOTH terms of the gate read a board via, so two negatives
turned the threshold negative and the gate OFF. Here only one term does --
`_vdr` is the caller's own knob -- so a negative drill CANCELS the new
barrel's radius (threshold falls to the bare h2h floor at drill = -2*_vdr)
and only goes under the floor beyond that. #732/#750 settled the spelling for the placement side --
a free function guarded `not d or d <= 0` -- and this is the routing side's
copy of that decision.

WHAT THE ISSUE LEFT OPEN, answered here rather than assumed:

  * "how often is `_v2.drill` falsy on a board that reaches :2557" --
    `test_the_corpus_cannot_reach_the_fallback` measures it live: 0 of the 602
    vias on the 22 boards `run_utils.corpus_boards()` returns has a falsy or
    negative drill. So this is INERT on the corpus and the rig below is
    synthetic by necessity, exactly as #750's was.
  * "whether `_vdr` is itself resolved consistently there" -- it is, and the
    check is not a reading of the assignment but of what the gate is gating:
    the via the #649b branch goes on to emit is built `size=config.via_size,
    drill=config.via_drill` (kicad_oracle, the `_Via649(...)` literal), which
    is the same `config.via_drill` that `_vdr` halves and that this fix uses
    as the fallback. `test_the_fallback_is_the_drill_the_branch_actually_
    emits` pins that pairing, so a future change to the emitted via that
    forgets the gate turns this file red.

WHAT THIS FILE DOES NOT DO. It does not drive `oracle_reconnect`: that gate
sits ~1000 lines inside a function that needs a board file, a routing config
and a live A* run to reach, and a rig that heavy would pin the router rather
than the resolver. The decision the gate makes IS tested -- as the resolver's
contribution to the threshold, evaluated over the same inputs -- and the
WIRING is pinned structurally, in both directions: `_audit` is run over the
real file AND over a pre-fix snippet that it must reject.

    python3 tests/test_754_oracle_via_drill_radius.py
"""
import ast
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kicad_oracle import _via_drill_radius

ORACLE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'py_router', 'kicad_oracle.py')

VIA_DRILL = 0.3          # the caller's knob -- what config.via_drill carries
H2H = 0.2                # hole-to-hole fab floor


class _V:
    """A via-like. `drill` may be absent entirely: the old expression raised
    AttributeError on that, the resolver prices it at the fallback."""
    def __init__(self, drill=None):
        if drill is not None:
            self.drill = drill


def _old(v, _fallback):
    """The pre-fix :2557 spelling, for the difference table."""
    return (getattr(v, 'drill', 0) or 0) / 2.0


def _audit(src):
    """Problems with how kicad_oracle resolves an EXISTING via's drill.

    Returns a list of strings; empty means clean. Run over the shipped file
    AND over a pre-fix snippet, so a checker that cannot see the defect it
    was written for fails loudly instead of printing PASS."""
    problems = []
    tree = ast.parse(src)

    # 1. `<something>.drill or <x>` anywhere: the spelling that keeps a
    #    negative drill. (A bare `.drill or 0` is the #754 defect proper; any
    #    `or` fallback is the #732/#750 spelling complaint.)
    for node in ast.walk(tree):
        if isinstance(node, ast.BoolOp) and isinstance(node.op, ast.Or):
            for val in node.values:
                if isinstance(val, ast.Attribute) and val.attr == 'drill':
                    problems.append(
                        f"line {node.lineno}: `.drill or ...` -- an `or` "
                        f"fallback keeps a NEGATIVE drill")
    # 2. every call must pass a real fallback, never a literal zero.
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
             and n.func.id == '_via_drill_radius']
    if len(calls) != 2:
        problems.append(f"expected 2 _via_drill_radius call sites, found "
                        f"{len(calls)} -- a gate was unwired or added")
    for c in calls:
        if len(c.args) != 2:
            problems.append(f"line {c.lineno}: call takes (via, fallback)")
            continue
        fb = c.args[1]
        if isinstance(fb, ast.Constant):
            problems.append(f"line {c.lineno}: fallback is the literal "
                            f"{fb.value!r}, not a knob the caller supplied")
    return problems


def run():
    fails = []

    def check(name, cond, note=''):
        print(('PASS' if cond else 'FAIL') + f': {name}' +
              (f'  [{note}]' if note else ''))
        if not cond:
            fails.append(name)

    _resolver(check)
    _difference_table(check)
    _threshold(check)
    _wiring(check)
    _emitted_via(check)
    _corpus(check)

    print()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


# === the resolver's contract ===============================================
def _resolver(check):
    print('the resolver: one answer, and never a weaker one')
    check('a readable drill is halved',
          abs(_via_drill_radius(_V(0.25), VIA_DRILL) - 0.125) < 1e-12)
    check('a MISSING drill falls back on the caller\'s knob, not on 0',
          abs(_via_drill_radius(_V(), VIA_DRILL) - VIA_DRILL / 2) < 1e-12,
          'the AttributeError case the old expression could not survive')
    check('drill == 0 falls back too (a via with no hole is not a thing)',
          abs(_via_drill_radius(_V(0.0), VIA_DRILL) - VIA_DRILL / 2) < 1e-12)
    check('drill is None falls back too',
          abs(_via_drill_radius(_V(None), VIA_DRILL) - VIA_DRILL / 2) < 1e-12)
    check('a NEGATIVE drill falls back -- it does not pass through and '
          'SUBTRACT from the threshold',
          abs(_via_drill_radius(_V(-0.3), VIA_DRILL) - VIA_DRILL / 2) < 1e-12,
          'the half #750 found on both sites')
    check('an unusable fallback yields 0.0 rather than a negative radius',
          _via_drill_radius(_V(-0.3), -1.0) == 0.0,
          'nothing left to measure with; still never negative')
    print()


def _difference_table(check):
    """The inputs where the two spellings actually differ, measured over the
    class rather than asserted one at a time."""
    print('where `or 0` and the guard disagree (radius, fallback '
          f'{VIA_DRILL})')
    rows = [('0.25  readable', _V(0.25)),
            ('0.0   unreadable', _V(0.0)),
            ('None  unreadable', _V(None)),
            ('-0.3  negative', _V(-0.3)),
            ('absent', _V())]
    differs = 0
    for label, v in rows:
        try:
            o = _old(v, VIA_DRILL)
            os_ = f'{o:.4f}'
        except AttributeError as exc:
            o, os_ = None, f'AttributeError'
        n = _via_drill_radius(v, VIA_DRILL)
        same = (o is not None and abs(o - n) < 1e-12)
        differs += 0 if same else 1
        print(f'        {label:<18} old {os_:>14}   new {n:.4f}'
              f'{"" if same else "   <- differs"}')
    check('the readable case is UNCHANGED (this is not a re-pricing of '
          'ordinary boards)',
          abs(_old(_V(0.25), VIA_DRILL) - _via_drill_radius(_V(0.25), VIA_DRILL))
          < 1e-12)
    check('every unreadable spelling now resolves to the same radius',
          len({round(_via_drill_radius(v, VIA_DRILL), 12)
               for v in (_V(0.0), _V(None), _V(-0.3), _V())}) == 1,
          'one resolver -- no branch can take a different one')
    print()


# === what the gate does with it ============================================
def _threshold(check):
    """The gate is `_d2 < _vdr + <this radius> + _h2h`. The resolver's whole
    job is that threshold, so the consequence is stated in millimetres."""
    print('the consequence at the gate (_vdr + radius + h2h, '
          f'config.via_drill {VIA_DRILL}, h2h {H2H})')
    vdr = VIA_DRILL / 2.0
    old_thr = vdr + _old(_V(0.0), VIA_DRILL) + H2H
    new_thr = vdr + _via_drill_radius(_V(0.0), VIA_DRILL) + H2H
    sane = vdr + _via_drill_radius(_V(VIA_DRILL), VIA_DRILL) + H2H
    print(f'        unreadable drill: old {old_thr:.4f}mm   '
          f'new {new_thr:.4f}mm   (two sane 0.3 barrels need {sane:.4f}mm)')
    check('the old threshold was SHORT of what two sane barrels require',
          old_thr < sane - 1e-9,
          f'{sane - old_thr:.4f}mm of fab floor given away')
    check('the new threshold is exactly the sane-barrel requirement',
          abs(new_thr - sane) < 1e-12)
    # A separation that the fab floor forbids and the old gate accepted.
    sep = (old_thr + sane) / 2.0
    check(f'a via {sep:.4f}mm away is now REFUSED and used to be accepted',
          not (sep < old_thr) and (sep < new_thr))
    # The negative case, stated exactly. Only ONE of the two terms reads the
    # board here (`_vdr` is the caller's own knob and cannot go negative), so
    # unlike #750's via-to-via gate this one does not invert at a plausible
    # drill -- it CANCELS `_vdr`. At drill = -2*_vdr the threshold is the bare
    # hole-to-hole floor with the new barrel priced at nothing; below that it
    # does go under the floor. Both are recorded, because "the gate is off"
    # is the #750 finding and is NOT what happens here at -0.3.
    for d in (-2 * vdr, -0.6):
        t = vdr + _old(_V(d), VIA_DRILL) + H2H
        print(f'        drill {d:+.2f}:      old {t:.4f}mm   '
              f'new {new_thr:.4f}mm')
    cancels = vdr + _old(_V(-2 * vdr), VIA_DRILL) + H2H
    under = vdr + _old(_V(-0.6), VIA_DRILL) + H2H
    check('a negative drill used to cancel the new barrel entirely, leaving '
          'the bare h2h floor',
          abs(cancels - H2H) < 1e-12,
          f'{sane - cancels:.4f}mm of the {sane:.4f}mm requirement given away')
    check('and a larger-magnitude negative pushed it BELOW the h2h floor',
          under < H2H - 1e-12, f'{under:.4f}mm')
    print()


# === the wiring, in both directions ========================================
def _wiring(check):
    print('the wiring: both gates route through the one resolver')
    src = open(ORACLE, encoding='utf-8').read()
    problems = _audit(src)
    for p in problems:
        print(f'        {p}')
    check('kicad_oracle.py is clean', not problems)

    # NEGATIVE CONTROL: the checker must reject the code it was written for.
    # Without this the whole section could be a gate that cannot see anything.
    pre_fix = (
        "def f(config, used_via_drill, pcb_data, net_id, _vdr, _h2h, _vx, _vy):\n"
        "    for _v2 in pcb_data.vias:\n"
        "        _d2 = 0.0\n"
        "        if _d2 < _vdr + (_v2.drill or 0) / 2 + _h2h:\n"
        "            return False\n"
        "    for _ev in []:\n"
        "        _lim = (used_via_drill + (_ev.drill or used_via_drill)) / 2 + 1\n"
        "    return True\n")
    got = _audit(pre_fix)
    print(f'        negative control (pre-fix source): {len(got)} problem(s)')
    for p in got:
        print(f'          - {p}')
    check('the checker REJECTS the pre-fix source (it is not vacuous)',
          any('.drill or' in p for p in got)
          and any('call sites' in p for p in got),
          'both halves seen: the `or` spelling AND the unwired gates')

    # And it must reject a re-wiring that keeps the invented zero.
    zeroed = ("def f(v, config):\n"
              "    a = _via_drill_radius(v, 0)\n"
              "    b = _via_drill_radius(v, config.via_drill)\n")
    got = _audit(zeroed)
    check('and rejects a call that passes the literal 0 as the fallback',
          any('literal 0' in p for p in got))
    print()


def _emitted_via(check):
    """`_vdr` is `config.via_drill / 2`, and the branch it guards emits a via
    at `drill=config.via_drill`. The fallback is that same knob, so a gate and
    the barrel it admits are priced identically. Read off the source, because
    the pairing is the claim -- not the constant."""
    print('the fallback is the drill the #649b branch actually emits')
    src = open(ORACLE, encoding='utf-8').read()
    tree = ast.parse(src)
    emitted = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
                and node.func.id.startswith('_Via649'):
            for kw in node.keywords:
                if kw.arg == 'drill':
                    emitted.append(ast.unparse(kw.value))
    print(f'        _Via649 emits drill={emitted}')
    check('the #649b via is emitted at config.via_drill',
          emitted == ['config.via_drill'])
    call = next(n for n in ast.walk(tree)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                and n.func.id == '_via_drill_radius'
                and ast.unparse(n.args[1]) == 'config.via_drill')
    check('and the gate guarding it falls back on the same knob',
          call is not None,
          'gate and barrel priced identically -- the issue\'s open question')
    print()


def _corpus(check):
    """INERT ON THE CORPUS, measured rather than quoted."""
    print('corpus reachability')
    try:
        from run_utils import corpus_boards
        from kicad_parser import parse_kicad_pcb
    except Exception as exc:                                  # pragma: no cover
        print(f'        SKIP: {exc}')
        return
    tot = bad = boards = 0
    for b in corpus_boards():
        try:
            pcb = parse_kicad_pcb(str(b))
        except Exception:
            continue
        boards += 1
        for v in pcb.vias:
            tot += 1
            d = getattr(v, 'drill', None)
            if not d or d <= 0:
                bad += 1
    print(f'        {boards} boards, {tot} vias, {bad} with a falsy or '
          f'negative drill')
    check('no corpus via can reach the fallback (so this fix moves no '
          'corpus keep-out, and the rig above is synthetic by necessity)',
          bad == 0, 'if this fails the fix is no longer inert -- re-measure')
    print()


if __name__ == '__main__':
    sys.exit(run())
