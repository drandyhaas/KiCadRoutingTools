#!/usr/bin/env python3
"""place_portfolio smoke: boards + renders + JSON emitted, gates honest,
candidate 0 is the place_optimize identity anchor, diversity holds among the
kept slate, and the refusal exits fire (unplaced board -> 3, zero viable
candidates -> 4)."""
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


def run(argv, timeout=1800):
    return subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=REPO, timeout=timeout,
                          env=dict(os.environ, PYTHONIOENCODING='utf-8'))


if not os.path.isfile(BOARD):
    print("SKIP: fixture missing")
    sys.exit(0)

with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, 'pf')

    # ---- 1. the main run: renders on, static ranking only ------------------
    r = run([os.path.join(REPO, 'py_placer', 'place_portfolio.py'), BOARD,
             '--out-dir', out, '--seed', '0', '--candidates', '4',
             '--keep', '2', '--route-top', '0'])
    check("portfolio run exits 0", r.returncode == 0,
          (r.stdout + r.stderr)[-500:])
    doc_path = os.path.join(out, 'portfolio.json')
    check("portfolio.json exists", os.path.isfile(doc_path))
    doc = json.load(open(doc_path, encoding='utf-8'))
    check("baseline board written", os.path.isfile(
        os.path.join(out, 'baseline_quenched.kicad_pcb')))
    check("baseline render written",
          os.path.isfile(os.path.join(out, 'baseline.png')))
    check("baseline .kicad_pro sibling carried", os.path.isfile(
        os.path.join(out, 'baseline_quenched.kicad_pro'))
          == os.path.isfile(os.path.splitext(BOARD)[0] + '.kicad_pro'))

    kept = doc['kept']
    cands = {c['index']: c for c in doc['candidates']}
    check("kept is non-empty", bool(kept))
    for i in kept:
        check(f"kept cand {i} board exists",
              os.path.isfile(os.path.join(out, f'cand_{i:02d}.kicad_pcb')))
        check(f"kept cand {i} passed its gates",
              cands[i]['gates'].get('passed') is True,
              str(cands[i]['gates']))
        check(f"kept cand {i} render written",
              os.path.isfile(os.path.join(out, f'cand_{i:02d}.png')))

    # every ranked candidate is viable, and the ranking is over gate-passers
    for i in doc['ranking_static']:
        check(f"ranked cand {i} is viable",
              cands[i]['gates'].get('passed') is True)

    # ---- 2. diversity among the kept, non-backfilled slate -----------------
    from placement.portfolio import pose_distance_mm
    div = doc['diversity_mm']
    genuine = [i for i in kept if i not in doc['backfilled']]
    poses = {i: {r: tuple(p) for r, p in cands[i]['final_poses'].items()}
             for i in kept}
    base_poses = {r: tuple(p)
                  for r, p in doc['baseline']['final_poses'].items()}
    for k, i in enumerate(genuine):
        dist = pose_distance_mm(poses[i], base_poses, doc['free'])
        check(f"kept cand {i} sits >= {div}mm from the baseline",
              dist >= div - 1e-6, f"{dist:.3f}mm")
        for j in genuine[:k]:
            dist = pose_distance_mm(poses[i], poses[j], doc['free'])
            check(f"kept cands {j}/{i} sit >= {div}mm apart",
                  dist >= div - 1e-6, f"{dist:.3f}mm")

    # ---- 3. candidate 0 == place_optimize with the same knobs --------------
    opt_out = os.path.join(d, 'opt.kicad_pcb')
    r = run([os.path.join(REPO, 'py_placer', 'place_optimize.py'), BOARD, opt_out,
             '--max-displacement', '3', '--length-weight', '0.3',
             '--crossing-penalty', '30', '--halo-coef', '0.15'])
    line = next(l for l in r.stdout.splitlines()
                if l.startswith('JSON_SUMMARY:'))
    opt = json.loads(line.split(':', 1)[1])
    base_m = doc['baseline']['metrics']
    check("baseline crossings == place_optimize's",
          base_m['crossings'] == opt['crossings_after'],
          f"{base_m['crossings']} vs {opt['crossings_after']}")
    check("baseline hpwl == place_optimize's",
          abs(base_m['hpwl'] - opt['hpwl_after']) < 1e-3,
          f"{base_m['hpwl']} vs {opt['hpwl_after']}")

    # ---- 4. refusals -------------------------------------------------------
    # unplaced pile -> exit 3, nothing generated
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    pcb = parse_kicad_pcb(BOARD)
    b = pcb.board_info.board_bounds
    cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
    pile = os.path.join(d, 'pile.kicad_pcb')
    write_placed_output(BOARD, pile, [
        {'reference': ref, 'new_x': cx, 'new_y': cy, 'new_rotation': 0}
        for ref, fp in sorted(pcb.footprints.items()) if fp.pads])
    r = run([os.path.join(REPO, 'py_placer', 'place_portfolio.py'), pile,
             '--out-dir', os.path.join(d, 'pf_pile'), '--no-render'])
    check("unplaced board refused with exit 3", r.returncode == 3,
          f"rc={r.returncode}")

    # #1037: the intent gate is on errors a candidate ADDS to the input. An
    # intent error the INPUT already carries (a must_lock pattern matching
    # nothing) gates nobody -- this used to be the exit-4 case, and it was the
    # run-32 defect in miniature: no candidate could ever be admissible.
    from placement.floorplan import emit_intent
    inherited = os.path.join(d, 'inherited_intent.json')
    intent_doc = emit_intent(pcb, BOARD)
    intent_doc['must_lock'] = ['ZZNOSUCHREF*']
    with open(inherited, 'w', encoding='utf-8') as f:
        json.dump(intent_doc, f)
    r = run([os.path.join(REPO, 'py_placer', 'place_portfolio.py'), BOARD,
             '--out-dir', os.path.join(d, 'pf_inh'), '--seed', '0',
             '--candidates', '2', '--keep', '1', '--route-top', '0',
             '--no-render', '--intent', inherited])
    check("an input-inherited intent error gates nothing (exit 0)",
          r.returncode == 0 and '"input_intent_errors": 1' in r.stdout,
          f"rc={r.returncode}\n{(r.stdout + r.stderr)[-300:]}")

    # candidates that ADD errors -> every candidate gated -> exit 4. A decap
    # limit of 1.45mm holds for most of splitflap's decaps at the input (it
    # already fails C2 and C8) and a 4mm jitter moves them past it.
    bad_intent = os.path.join(d, 'bad_intent.json')
    intent_doc = emit_intent(pcb, BOARD)
    intent_doc['decaps'] = {'max_distance_mm': 1.45}
    with open(bad_intent, 'w', encoding='utf-8') as f:
        json.dump(intent_doc, f)
    r = run([os.path.join(REPO, 'py_placer', 'place_portfolio.py'), BOARD,
             '--out-dir', os.path.join(d, 'pf_bad'), '--seed', '0',
             '--candidates', '2', '--keep', '1', '--route-top', '0',
             '--no-render', '--intent', bad_intent, '--strategy', 'jitter'])
    check("all-candidates-gated run exits 4", r.returncode == 4
          and 'NEW intent error' in r.stdout,
          f"rc={r.returncode}\n{(r.stdout + r.stderr)[-300:]}")

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
