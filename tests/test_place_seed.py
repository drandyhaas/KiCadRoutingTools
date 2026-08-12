#!/usr/bin/env python3
"""place_seed: an unplaced pile + an intent must become a placement that
grades clean against that same intent, deterministically per seed.

The pile fixture is synthesized (every part stacked at the board center, the
test_431 trick) and the intent is emitted OFF the real placed board, so the
seeder is asked for an arrangement that provably exists."""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (REPO,):
    if p not in sys.path:
        sys.path.insert(0, p)
        sys.path.insert(0, os.path.join(p, 'py_router'))  # placement split
        sys.path.insert(0, os.path.join(p, 'py_tools'))  # placement split
        sys.path.insert(0, os.path.join(p, 'py_placer'))  # placement split

BOARD = os.path.join(REPO, "kicad_files", "splitflap_driver.kicad_pcb")

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


def run(argv, hashseed="0", timeout=1800):
    env = dict(os.environ, PYTHONHASHSEED=hashseed,
               PYTHONIOENCODING='utf-8')
    return subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=REPO, env=env,
                          timeout=timeout)


def summary(r):
    line = next(l for l in r.stdout.splitlines()
                if l.startswith('JSON_SUMMARY:'))
    return json.loads(line.split(':', 1)[1])


if not os.path.isfile(BOARD):
    print("SKIP: fixture missing")
    sys.exit(0)

from kicad_parser import parse_kicad_pcb
from placement.floorplan import emit_intent
from placement.writer import write_placed_output

with tempfile.TemporaryDirectory() as d:
    pcb = parse_kicad_pcb(BOARD)
    n_parts = sum(1 for fp in pcb.footprints.values() if fp.pads)
    b = pcb.board_info.board_bounds
    cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
    pile = os.path.join(d, 'pile.kicad_pcb')
    write_placed_output(BOARD, pile, [
        {'reference': ref, 'new_x': cx, 'new_y': cy, 'new_rotation': 0}
        for ref, fp in sorted(pcb.footprints.items()) if fp.pads])
    intent_path = os.path.join(d, 'intent.json')
    with open(intent_path, 'w', encoding='utf-8') as f:
        json.dump(emit_intent(pcb, BOARD), f)

    seed_py = os.path.join(REPO, 'py_placer', 'place_seed.py')
    out_a = os.path.join(d, 'a.kicad_pcb')
    r = run([seed_py, pile, out_a, '--intent', intent_path])
    check("seed run exits 0", r.returncode == 0,
          (r.stdout + r.stderr)[-500:])
    s = summary(r)
    check("all parts placed", s['placed'] == n_parts,
          f"{s['placed']}/{n_parts}")
    check("no unseated parts", s['unseated'] == 0)
    check("grade has zero errors", s['grade_errors'] == 0)

    # deterministic per seed, across PYTHONHASHSEED
    out_b = os.path.join(d, 'b.kicad_pcb')
    r = run([seed_py, pile, out_b, '--intent', intent_path], hashseed="12345")
    check("re-run exits 0", r.returncode == 0)
    check("same seed is byte-identical across hash seeds",
          open(out_a, 'rb').read() == open(out_b, 'rb').read())

    out_c = os.path.join(d, 'c.kicad_pcb')
    r = run([seed_py, pile, out_c, '--intent', intent_path, '--seed', '3'])
    check("--seed 3 exits 0/4", r.returncode in (0, 4), f"rc={r.returncode}")
    check("--seed 3 differs",
          open(out_a, 'rb').read() != open(out_c, 'rb').read())

    # a placed board is refused without --force
    r = run([seed_py, BOARD, os.path.join(d, 'x.kicad_pcb'),
             '--intent', intent_path])
    check("placed board refused with exit 3", r.returncode == 3,
          f"rc={r.returncode}")
    check("refusal names place_portfolio as the explore path",
          'place_portfolio' in (r.stdout + r.stderr))

    # the emitted seed is a valid input for the portfolio (composition)
    from placement.placement_state import assess_placement
    st = assess_placement(parse_kicad_pcb(out_a), out_a)
    check("seeded board assesses as PLACED", not st.unplaced,
          '; '.join(st.reasons))

    # sibling carry (d310ab3 regression): the seeder is the FIRST step that
    # touches the board, so a dropped .kicad_pro propagates a stock-netclass
    # floor through the whole chain (#441 class). Give the pile siblings and
    # the output must get them too.
    with open(os.path.join(d, 'pile.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'net_settings': {'classes': [
            {'name': 'Default', 'clearance': 0.15}]}}, f)
    with open(os.path.join(d, 'pile.kicad_dru'), 'w', encoding='utf-8') as f:
        f.write('(version 1)\n')
    out_s = os.path.join(d, 's.kicad_pcb')
    r = run([seed_py, pile, out_s, '--intent', intent_path])
    check("sibling run exits 0", r.returncode == 0)
    check("seed output carries the .kicad_pro sibling",
          os.path.isfile(os.path.join(d, 's.kicad_pro')))
    check("seed output carries the .kicad_dru sibling",
          os.path.isfile(os.path.join(d, 's.kicad_dru')))

    # ---------------------------------------------------------------- --reseat
    #
    # A part tens of millimetres off the outline is unreachable by every
    # minimal-move repair in this stack, because they all search outward from
    # the part's CURRENT pose. Measured on a real damaged board: --repair spent
    # 4m55s and attempted none of the 11 such parts. --reseat lifts them.
    print("\n--reseat")

    # (a) HEALTHY BOARD: the auto scope is empty and the pass is a no-op that
    # exits 0. This is the property that makes it safe in a default ladder --
    # the census is corpus-calibrated to zero on all 33 in-repo boards.
    r = run([seed_py, BOARD, os.path.join(d, 'h.kicad_pcb'),
             '--intent', intent_path, '--reseat', '--dry-run'])
    check("healthy board: bare --reseat exits 0", r.returncode == 0,
          f"rc={r.returncode} {(r.stdout + r.stderr)[-300:]}")
    s = summary(r)
    check("healthy board: auto scope is empty", s['scope'] == [], str(s['scope']))
    check("healthy board: no witnesses before OR after",
          s['witnesses_before'] == 0 and s['witnesses_after'] == 0)

    # (b) DAMAGED BOARD, synthesized here so the test owns its own fixture:
    # push three parts well outside the outline.
    victims = [ref for ref, fp in sorted(pcb.footprints.items())
               if fp.pads][:3]
    far_x = b[2] + 25.0
    dmg = os.path.join(d, 'dmg.kicad_pcb')
    write_placed_output(BOARD, dmg, [
        {'reference': ref, 'new_x': far_x + 5.0 * i, 'new_y': cy,
         'new_rotation': pcb.footprints[ref].rotation or 0}
        for i, ref in enumerate(victims)])

    import pose_score
    from placement import reconstruct as _rec
    _dpcb = parse_kicad_pcb(dmg)
    _dst = pose_score.make_state(_dpcb, dmg)
    wit = sorted(_rec.damage_witnesses(_dst))
    check("damage_witnesses names exactly the parts pushed off",
          wit == sorted(victims), f"{wit} vs {sorted(victims)}")

    # (c) THE LAUNDERED-BAND REGRESSION. An intent emitted off the DAMAGED
    # board must NOT convert those overhangs into declared edge bands: run 10
    # emitted bands equal to each part's damage displacement (up to 160mm on an
    # 81mm board), which blinded the repair census, the reconstruct gate and
    # the intent grade all at once. The entry is capped AND loses its `edge`.
    dmg_intent = os.path.join(d, 'dmg_intent.json')
    _di = emit_intent(_dpcb, dmg, declare_classes=True)
    with open(dmg_intent, 'w', encoding='utf-8') as f:
        json.dump(_di, f)
    _ec = {c['ref']: c for c in _di.get('edge_connectors') or ()}
    _bad = [(v, _ec[v].get('overhang_mm', {}).get('max'), _ec[v].get('edge'))
            for v in victims if v in _ec]
    check("a 25mm+ overhang is emitted WITHOUT an edge (treated as damage)",
          all(e is None for _r, _m, e in _bad) and len(_bad) == len(victims),
          str(_bad))
    check("...and its band is CAPPED, not the observed displacement",
          all(m is not None and m <= 25.0 for _r, m, _e in _bad), str(_bad))
    check("...and the entry says so in machine-readable form",
          all(_ec[v].get('overhang_capped') for v in victims if v in _ec))
    # The counterfactual, pinned because this is the kind of finding that gets
    # re-litigated: seeding WITH such a band is not a small error. Stage 1
    # places a declared edge connector with no legality gate at the middle of
    # its band, and a 160mm band threw a part THIRTY KILOMETRES off the board.
    check("the uncapped band would have been >= the displacement",
          all(m < 20.0 for _r, m, _e in _bad),
          'cap must be far below the ~25-35mm observed overhang')

    # (d) THE PASS ITSELF: auto scope, all seated, witnesses to zero.
    out_r = os.path.join(d, 'r.kicad_pcb')
    r = run([seed_py, dmg, out_r, '--intent', dmg_intent, '--reseat'])
    check("--reseat on the damaged board exits 0", r.returncode == 0,
          f"rc={r.returncode} {(r.stdout + r.stderr)[-400:]}")
    s = summary(r)
    check("--reseat auto scope is exactly the witnesses",
          s['scope'] == sorted(victims), str(s['scope']))
    check("--reseat scope_source names the census",
          s['scope_source'] == 'auto:damage_witnesses')
    check("witnesses_after is 0 -- THE number that predicts routability",
          s['witnesses_after'] == 0,
          f"{s['witnesses_before']} -> {s['witnesses_after']}")
    check("every scope part was seated", s['unseated'] == [] and
          s['reseated'] == len(victims), str(s))
    check("the gate ACCEPTED", s['accepted'] is True)
    _gb, _ga = s['gate_before'], s['gate_after']
    _oob = _rec.GATE_TERMS.index('oob')
    check("oob strictly improved", _ga[_oob] < _gb[_oob],
          f"{_gb[_oob]} -> {_ga[_oob]}")
    check("nothing ABOVE oob in the gate tuple worsened",
          all(_ga[i] <= _gb[i] for i in range(_oob)),
          f"{_gb[:_oob]} -> {_ga[:_oob]}")
    check("no scope ref kept an edge declaration",
          set(s['edge_bands_dropped']) <= set(victims))

    # (e) THE CAUSAL PROPERTY, and the reason the pass exists: every net that
    # had a pad OFF THE BOARD has none afterwards. That is the thing which
    # converts one-for-one into `unrouted` -- on the measured board the 11
    # off-board parts carried a pad on every one of the 13 nets the router
    # refused to attempt.
    #
    # `placement_blocked` (< 2 ON-BOARD pads, board_score's split) is the
    # STRICTER form and is asserted only when the fixture actually produces
    # one: whether an off-board pad drops its net below two on-board pads
    # depends on which parts got pushed off, and here they are decaps on rails
    # with many other pads. The general property above holds either way.
    from check_drc import make_off_board_test
    from net_queries import routable_pad_count

    def net_shapes(board):
        p = parse_kicad_pcb(board)
        off = make_off_board_test(p.board_info)
        dirty, blocked = set(), set()
        for nid, pads in p.pads_by_net.items():
            if nid not in p.nets or len(pads) < 2:
                continue
            if any(off(q.global_x, q.global_y) for q in pads):
                dirty.add(p.nets[nid].name)
            if routable_pad_count(p, nid, off) < 2:
                blocked.add(p.nets[nid].name)
        return dirty, blocked

    _d0, _b0 = net_shapes(dmg)
    _d1, _b1 = net_shapes(out_r)
    check("damaged board HAS nets with an off-board pad", bool(_d0),
          str(sorted(_d0)[:6]))
    check("after --reseat NO net has an off-board pad", not _d1,
          str(sorted(_d1)[:6]))
    check("placement-blocked nets do not survive the pass", not _b1,
          f"before {sorted(_b0)[:4]} after {sorted(_b1)[:4]}")

    # (f) determinism across PYTHONHASHSEED, same as the seeding path above.
    out_r2 = os.path.join(d, 'r2.kicad_pcb')
    r = run([seed_py, dmg, out_r2, '--intent', dmg_intent, '--reseat'],
            hashseed="12345")
    check("--reseat is byte-identical across hash seeds",
          r.returncode == 0
          and open(out_r, 'rb').read() == open(out_r2, 'rb').read())

    # (g) refusals are NAMED, never silent.
    r = run([seed_py, dmg, os.path.join(d, 'n.kicad_pcb'),
             '--intent', dmg_intent, '--reseat', 'NOSUCHREF', '--dry-run'])
    check("an unmatched ref is named in the notes",
          'NOSUCHREF' in r.stdout, r.stdout[-300:])
    r = run([seed_py, dmg, os.path.join(d, 'f.kicad_pcb'),
             '--intent', dmg_intent, '--reseat', '--force'])
    check("--reseat with --force is a usage error", r.returncode == 2,
          f"rc={r.returncode}")

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
