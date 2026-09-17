#!/usr/bin/env python3
"""An A/B row says which floor its KiCad numbers were graded at.

    python3 tests/test_graded_clearance_disclosed.py

`kicad_drc_compare.compare_board_data` grades at the board's OWN recorded floor
(the Default net-class clearance in its sibling `.kicad_pro`), not at the
manifest clearance it is passed. That is deliberate and measured -- #439: a
board may legitimately fine-tap below the nominal `--clearance`, and grading at
the manifest ceiling manufactured 499 phantom items on neo6502 (vs 0 at its
recorded 0.1). This gate does NOT change that.

What it pins is the hole the rule leaves: **an arm whose WRITEBACK is wrong
grades itself leniently, and nothing said so.**

MEASURED on a20_can, both arms asked for 0.254 (their route step runs
`--clearance-ceiling 0.254`):

    arm                   shipped Default class   graded at   kicad items
    v0.22.0 e1f52745      0.0508  <- the #900 bug   0.0508          0
    HEAD    d96877f1      0.254                     0.254          49

Read as a pair that is a 0 -> 49 "regression". It is two different rulers: the
same v0.22.0 COPPER graded at 0.254 has 54 kicad items, MORE than the 49. And
both rows recorded `clearance: "0.254"` -- the manifest value -- so the summary
asserted a floor neither arm had necessarily used.

The fix is disclosure, not a forced re-grade: which arm is right is a question
about the writeback, and the answer is not always the higher floor.

Rows:
 1. `compare_board_data` reports `graded_clearance` and `requested_clearance`,
    and they DIFFER on a board that ships its own floor.
 2. The policy is unchanged -- the grade still uses the recorded floor.
 3. `compare()` flags a pair whose two arms were graded at different floors.
 4. ...and does NOT flag one where they agree (the over-reach detector).
"""
import json
import io
import os
import sys
import tempfile
import contextlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools', 'tests/stress'):
    _q = os.path.join(ROOT, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

import ab_replay_grade as A  # noqa: E402

STRESS = os.path.expanduser('~/Documents/kicad_stress_test')
V220 = f'{STRESS}/cloud_rel2200-kc_e1f52745/set3/a20_can/step2_route.kicad_pcb'
HEAD = f'{STRESS}/cloud_head-kc_d96877f1/set3/a20_can/step2_route.kicad_pcb'
BASE = f'{STRESS}/boards_unrouted_set3/a20_can.kicad_pcb'

fails = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f"   [{detail}]" if detail else ''))
    if not cond:
        fails.append(name)


def _row(board, graded, drc=0, incompl=0):
    return {"board": board, "chain_complete": True, "drc": drc, "drc_real": drc,
            "conn": incompl, "nets_incomplete": incompl, "completion_pct": 100.0,
            "diff_pairs_coupled": 0, "diff_pairs_total": 0,
            "total_seconds": 1.0, "peak_rss_mb": 1.0,
            "time_by_tool": {}, "peak_by_tool": {},
            "kicad_connection_width": None, "graded_clearance": graded}


def compare_text(old_rows, new_rows):
    d = tempfile.mkdtemp(prefix='gc_')
    po, pn = os.path.join(d, 'o.json'), os.path.join(d, 'n.json')
    open(po, 'w').write(json.dumps(old_rows))
    open(pn, 'w').write(json.dumps(new_rows))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        A.compare(po, pn)
    return buf.getvalue()


def main():
    print("1/2. the shared grading core discloses its floor, policy unchanged")
    have = all(os.path.isfile(p) for p in (V220, HEAD, BASE))
    if have:
        from kicad_drc_compare import compare_board_data, KICAD_CLI
        if not os.path.exists(KICAD_CLI):
            print("  SKIP: kicad-cli absent -- rows 3/4 still run")
            have = False
    if have:
        with contextlib.redirect_stderr(io.StringIO()):
            v = compare_board_data(V220, label='a20_can', clearance=0.254, baseline=BASE)
            h = compare_board_data(HEAD, label='a20_can', clearance=0.254, baseline=BASE)
        check("both arms were ASKED for the same floor",
              v.get('requested_clearance') == h.get('requested_clearance') == 0.254,
              f"{v.get('requested_clearance')} / {h.get('requested_clearance')}"
              + (" -- the key is absent: the core does not disclose its floor"
                 if v.get('requested_clearance') is None else ""))
        # Every row tolerates a MISSING key, so the unfixed code fails each row
        # on its own terms instead of dying on float(None) at the first one --
        # a battery that crashes reports nothing about the rows it never ran.
        def _f(x):
            try:
                return float(x)
            except (TypeError, ValueError):
                return None
        vg, hg, vr = _f(v.get('graded_clearance')), _f(h.get('graded_clearance')), _f(v.get('requested_clearance'))
        check("the v0.22.0 arm was GRADED at its own lower shipped floor",
              vg is not None and vg < 0.254,
              f"graded={v.get('graded_clearance')} kicad={v.get('kicad')}")
        check("the fixed arm was graded at the floor its route step asked for",
              hg is not None and abs(hg - 0.254) < 1e-9,
              f"graded={h.get('graded_clearance')} kicad={h.get('kicad')}")
        check("policy UNCHANGED: the recorded floor still decides the grade",
              vg is not None and vr is not None and vg != vr,
              "a forced re-grade would have made these equal")
    else:
        print("  SKIP: a20_can arm boards not present -- rows 3/4 still run")

    print("3. compare() names a pair graded at two different floors")
    out = compare_text([_row('a20_can', 0.0508)], [_row('a20_can', 0.254)])
    check("FLOORS DIFFER is reported", 'FLOORS DIFFER' in out,
          [l.strip() for l in out.splitlines() if 'a20_can' in l][:1])
    check("...and it names both floors", '0.0508' in out and '0.254' in out)
    check("...and says the kicad counts are not comparable",
          'not comparable' in out)

    print("4. ...and stays quiet when the two arms agree")
    out = compare_text([_row('a20_can', 0.254)], [_row('a20_can', 0.254)])
    check("no FLOORS DIFFER when the floors match", 'FLOORS DIFFER' not in out)
    out = compare_text([_row('b', None)], [_row('b', None)])
    check("no FLOORS DIFFER for pre-schema rows that carry no floor",
          'FLOORS DIFFER' not in out)

    print(f"\n{len(fails)} failed")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
