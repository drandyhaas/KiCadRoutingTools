#!/usr/bin/env python3
"""Protected nets are shielded from a run's rip machinery (#521).

The run-5 defect this guards: only length-matched groups and routed diff pairs
were protected, so a retry step's `--rip-existing-nets` glob happily ripped a
hand-verified single-ended net as collateral (test-board: one rail rip opened
19 pads).

NOTE: this was once driven by a `--protect-nets` flag, removed in 53a5a16e
along with the other placement-only routing flags. Protection is deliberately
FLAGLESS now (CLAUDE.md): the record lives in the sibling .kicad_pro, written
by the engine and carried down the chain, so this test seeds it there. The
contract it pins is unchanged:

  * matches are protected for the run (reason 'user'),
  * the protection persists to the output .kicad_pro, so LATER chain steps
    honor it without the flag,
  * naming the net EXACTLY (no glob) in a rip set still overrides -- 'user'
    protection is deliberate-target overridable, unlike 'locked'.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
# #522: the engine lives in py_router/, the placer in py_placer/ --
# ROOT alone does not make `import kicad_parser` work.
for _p522 in ('py_router', 'py_placer'):
    _d522 = os.path.join(ROOT, _p522)
    if _d522 not in sys.path:
        sys.path.insert(0, _d522)

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _run(board, out, extra, td):
    js = os.path.join(td, f'{os.path.basename(out)}.json')
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                        '--json-out', js] + extra,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    assert r.returncode == 0, r.stdout[-2500:]
    return json.load(open(js, encoding='utf-8')), r.stdout


def _seg_count(path, net_name):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(path)
    ids = {i for i, n in pcb.nets.items() if n.name == net_name}
    return sum(1 for s in pcb.segments if s.net_id in ids)


def test_protect_glob_rip_and_exact_override():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        staged = os.path.join(td, 'b.kicad_pcb')
        shutil.copyfile(BOARD, staged)

        # 1. commit GND copper
        r1 = os.path.join(td, 'r1.kicad_pcb')
        _run(staged, r1, ['--nets', 'GND'], td)
        gnd_before = _seg_count(r1, 'GND')
        assert gnd_before > 0, "setup: GND routed no copper"

        # 2. a later step glob-rips it -- but GND is protected.
        #    53a5a16e REMOVED --protect-nets (the last placement-only routing
        #    flags), and per CLAUDE.md protection is deliberately flagless:
        #    the record lives in the sibling .kicad_pro, written by the engine
        #    for matched groups / routed diff pairs and carried down the chain.
        #    Seeding that record IS the modern equivalent of the old flag.
        pro1 = os.path.splitext(r1)[0] + '.kicad_pro'
        proj = json.load(open(pro1, encoding='utf-8')) if os.path.isfile(pro1) else {}
        proj.setdefault('kicad_routing_tools', {})['protected_nets'] = {'GND': 'user'}
        with open(pro1, 'w', encoding='utf-8') as fh:
            json.dump(proj, fh, indent=2)
        r2 = os.path.join(td, 'r2.kicad_pcb')
        summary, out = _run(r1, r2,
                            ['--nets', '/OUT_A_PHASE_A',
                             '--rip-existing-nets', 'GN?'], td)
        assert _seg_count(r2, 'GND') == gnd_before, \
            "protected net's copper changed under a glob rip"
        skipped = summary.get('protected_skipped') or {}
        flat = {n: r for ctx in skipped.values() for n, r in ctx.items()}
        assert flat.get('GND') == 'user', \
            f"refusal must be machine-readable with reason 'user', got {skipped}"
        # 3. and it persisted: the output project carries the protection
        pro = json.load(open(os.path.join(td, 'r2.kicad_pro'), encoding='utf-8'))
        rec = (pro.get('kicad_routing_tools') or {}).get('protected_nets') or {}
        assert rec.get('GND') == 'user', f"not persisted: {rec}"
        print("  PASS: glob rip refused (reason 'user'), persisted to .kicad_pro")

        # 4. next step, WITHOUT the flag: protection comes from the project;
        #    naming GND exactly in the rip set is the deliberate override
        r3 = os.path.join(td, 'r3.kicad_pcb')
        summary3, out3 = _run(r2, r3,
                              ['--nets', '/OUT_A_PHASE_B',
                               '--rip-existing-nets', 'GND'], td)
        flat3 = {n: r
                 for ctx in (summary3.get('protected_skipped') or {}).values()
                 for n, r in ctx.items()}
        assert 'GND' not in flat3, \
            f"exact name must override 'user' protection, got {summary3.get('protected_skipped')}"
        assert 'pre-existing net(s) eligible for rip-up: GND' in out3, \
            "exact-named GND should be rip-eligible again:\n" + '\n'.join(
                l for l in out3.splitlines() if 'rip' in l.lower())[:1500]
    print("  PASS: exact name in the rip set lifts the protection")


def test_in_run_ladder_honors_exact_override():
    """Run-6 z2 defect: the pre-run filters lifted 'user' protection for an
    exactly-named net, but the in-run ladders (all six flow through
    filter_rippable_blockers) re-consulted cached_protection_map and refused
    it anyway -- 'protected_skipped {"phase3 tap cascade": {USB_DM_R: user}}'
    while --rip-existing-nets named it. Unit-pins the choke point:
    stashed exact-name overrides lift 'user' in-run; 'locked' never lifts;
    without a stash the refusal + record behave as before."""
    from types import SimpleNamespace
    from blocking_analysis import filter_rippable_blockers
    from protected_nets import (PROTECTED_SKIPPED, clear_skipped,
                                stash_rip_overrides)

    class _Net:
        def __init__(self, name):
            self.name = name

    def _pcb(protection):
        p = SimpleNamespace()
        p.nets = {1: _Net('RAIL_A'), 2: _Net('LOCKED_B'), 3: _Net('FREE_C')}
        # protection_map also folds in KiCad-LOCKED copper (#521), which reads
        # the board's segments/vias. The stub predates that and needs the
        # fields to exist; empty means "nothing locked on the board", so the
        # 'locked' reason in `protection` below is still what drives the case.
        p.segments = []
        p.vias = []
        # cached_protection_map reads _protection_map_memo (renamed from
        # _protection_map_cache); the stale name made the stub
        # silently re-derive an EMPTY map, so the filter refused
        # nothing and the test asserted against a no-op.
        p._protection_map_memo = dict(protection)
        return p

    blockers = [SimpleNamespace(net_id=i) for i in (1, 2, 3)]
    routed = {1: object(), 2: object(), 3: object()}
    prot = {'RAIL_A': 'user', 'LOCKED_B': 'locked'}

    # (a) no stash: both protected nets refused, recorded with reasons
    clear_skipped()
    pcb = _pcb(prot)
    kept, _ = filter_rippable_blockers(blockers, routed, {}, lambda n, d: n,
                                       pcb_data=pcb, context='unit cascade')
    assert {b.net_id for b in kept} == {3}, kept
    assert PROTECTED_SKIPPED.get('unit cascade') == prot, PROTECTED_SKIPPED

    # (b) exact-name stash lifts 'user', never 'locked'
    clear_skipped()
    pcb = _pcb(prot)
    stash_rip_overrides(pcb, ['RAIL_A', 'LOCKED_B', 'GLOB_*'])
    kept, _ = filter_rippable_blockers(blockers, routed, {}, lambda n, d: n,
                                       pcb_data=pcb, context='unit cascade')
    assert {b.net_id for b in kept} == {1, 3}, kept
    assert PROTECTED_SKIPPED.get('unit cascade') == {'LOCKED_B': 'locked'}, \
        PROTECTED_SKIPPED

    # (c) a glob in the stash list is NOT an override
    clear_skipped()
    pcb = _pcb({'RAIL_A': 'user'})
    stash_rip_overrides(pcb, ['RAIL_*'])
    kept, _ = filter_rippable_blockers(blockers, routed, {}, lambda n, d: n,
                                       pcb_data=pcb, context='unit cascade')
    assert {b.net_id for b in kept} == {2, 3}, kept
    print("  PASS: in-run ladder honors exact-name overrides ('locked' never)")


if __name__ == '__main__':
    test_protect_glob_rip_and_exact_override()
    test_in_run_ladder_honors_exact_override()
    print("ALL PASS")
