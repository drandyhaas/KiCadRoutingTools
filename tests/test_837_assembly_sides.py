#!/usr/bin/env python3
"""#837: the per-side assembly census, and the two ways of getting it wrong.

The census answers "which faces will the fab populate", and there are exactly
two traps between that question and the numbers a board hands you:

1. **Zero-pad blocks on the back.** `esp_prog` carries three OLIMEX logo
   footprints on B.Cu with no pads at all. Counting blocks it is two-sided;
   counting copper it is not, and the fab builds the second one. A census that
   reads `fp.layer` alone reports a second reflow pass for three silkscreen
   logos.

2. **`sides_occupied` is not an assembly classifier.** It answers the
   OBSTRUCTION question -- a drilled pad puts its part on both faces -- and
   under it 17 of the 22 tracked boards read double-sided, `splitflap_driver`
   (65 parts, every one on F.Cu, 24 of them through-hole) included. A
   through-hole part on the front does not demand a second reflow pass.

Both are asserted on boards where the wrong answer is a DIFFERENT number, so
neither arm can pass by coincidence. The expected counts are hand-stated
literals: re-deriving them from `assembly_census` would make this file agree
with whatever the module currently says and stop it being a change detector.
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer')]
sys.path.insert(0, HERE)

RUN_ALL_TIMEOUT = 300

FAILURES = []


def check(name, cond, detail=''):
    if cond:
        print(f"  ok   {name}")
    else:
        FAILURES.append(f"{name}: {detail}")
        print(f"  FAIL {name}: {detail}")


def board(stem):
    return os.path.join(ROOT, 'kicad_files', f'{stem}.kicad_pcb')


#: (stem, blocks F/B, pad-bearing F/B, zero-pad back, THT, sides, passes)
#: Measured at HEAD. `esp_prog` and `watchy` are the boards where the
#: pad-bearing rule CHANGES the verdict; `splitflap_driver` is where the
#: through-hole rule would; `ulx3s` is back-dominant, so an arm that assumed
#: "single-sided means front" would flag 163 of its parts.
EXPECT = [
    ('esp_prog',           (18, 3),   (18, 0),   3, 3,  'F',    1),
    ('watchy',             (85, 1),   (84, 0),   1, 5,  'F',    1),
    ('splitflap_driver',   (65, 0),   (65, 0),   0, 24, 'F',    1),
    ('ulx3s',              (67, 168), (63, 163), 5, 12, 'both', 2),
    ('glasgow_revC',       (178, 94), (172, 92), 2, 18, 'both', 2),
    ('kit-dev-coldfire-xilinx_5213',
                           (146, 14), (146, 14), 0, 41, 'both', 2),
]


def independent(pcb):
    """The same census by a different route, deliberately.

    Reads `fp.layer` and `len(fp.pads)` directly instead of calling
    `footprint_side` / `footprint_has_through_pads`. A test that calls the
    helpers the subject calls proves only that one function was invoked once.
    """
    blk = {'F': 0, 'B': 0}
    pad = {'F': 0, 'B': 0}
    zero_back, tht = 0, 0
    for fp in pcb.footprints.values():
        s = 'B' if (getattr(fp, 'layer', '') or '').startswith('B') else 'F'
        blk[s] += 1
        if not (fp.pads or ()):
            zero_back += (s == 'B')
            continue
        pad[s] += 1
        tht += any((getattr(p, 'drill', 0) or 0) > 0 for p in fp.pads)
    return blk, pad, zero_back, tht


def main():
    from kicad_parser import parse_kicad_pcb
    from placement.legality import (assembly_census, footprint_side,
                                    sides_occupied, footprint_has_through_pads)

    print("#837 assembly census")
    graded = 0
    for stem, e_blk, e_pad, e_zero, e_tht, e_sides, e_passes in EXPECT:
        path = board(stem)
        if not os.path.isfile(path):
            check(f"{stem}: fixture present", False, path)
            continue
        pcb = parse_kicad_pcb(path)
        c = assembly_census(pcb)
        i_blk, i_pad, i_zero, i_tht = independent(pcb)
        graded += 1

        check(f"{stem}: blocks {e_blk}",
              (c['blocks']['F'], c['blocks']['B']) == e_blk,
              f"got {(c['blocks']['F'], c['blocks']['B'])}")
        check(f"{stem}: pad-bearing {e_pad}",
              (c['pad_bearing']['F'], c['pad_bearing']['B']) == e_pad,
              f"got {(c['pad_bearing']['F'], c['pad_bearing']['B'])}")
        check(f"{stem}: an independent parse agrees",
              (i_blk, i_pad, i_zero, i_tht)
              == ({'F': c['blocks']['F'], 'B': c['blocks']['B']},
                  {'F': c['pad_bearing']['F'], 'B': c['pad_bearing']['B']},
                  len(c['zero_pad_back']), c['through_hole']),
              f"independent {(i_blk, i_pad, i_zero, i_tht)} vs census "
              f"{(c['blocks'], c['pad_bearing'], len(c['zero_pad_back']), c['through_hole'])}")
        check(f"{stem}: {e_zero} zero-pad back block(s)",
              len(c['zero_pad_back']) == e_zero,
              f"got {len(c['zero_pad_back'])}: {c['zero_pad_back']}")
        check(f"{stem}: {e_tht} through-hole part(s)",
              c['through_hole'] == e_tht, f"got {c['through_hole']}")
        check(f"{stem}: sides={e_sides}, {e_passes} reflow pass(es)",
              c['sides'] == e_sides and c['reflow_passes'] == e_passes,
              f"got sides={c['sides']} passes={c['reflow_passes']}")

    # --- Trap 1: the zero-pad rule must CHANGE the answer on esp_prog.
    # Without this the first arm above is satisfied by a census that never
    # looks at pads, because on 20 of 22 boards the two rules agree.
    pcb = parse_kicad_pcb(board('esp_prog'))
    naive_b = sum(1 for fp in pcb.footprints.values()
                  if footprint_side(fp) == 'B')
    check("esp_prog: the block rule and the pad rule DISAGREE",
          naive_b == 3 and assembly_census(pcb)['pad_bearing']['B'] == 0,
          f"blocks on B {naive_b}, pad-bearing B "
          f"{assembly_census(pcb)['pad_bearing']['B']}")
    check("esp_prog: so the verdict is single-sided, not double",
          assembly_census(pcb)['sides'] == 'F'
          and assembly_census(pcb)['reflow_passes'] == 1,
          str(assembly_census(pcb)))

    # --- Trap 2: `sides_occupied` would say two-sided; the census must not.
    pcb = parse_kicad_pcb(board('splitflap_driver'))
    occupied = set()
    for fp in pcb.footprints.values():
        occupied |= sides_occupied(footprint_side(fp),
                                   footprint_has_through_pads(fp))
    check("splitflap_driver: sides_occupied really does say BOTH",
          occupied == {'F', 'B'},
          f"{occupied} -- if this is not both, the arm below proves nothing")
    check("splitflap_driver: the census says F anyway",
          assembly_census(pcb)['sides'] == 'F'
          and assembly_census(pcb)['through_hole'] == 24,
          str(assembly_census(pcb)))

    # --- The CLI: additive only. The exit code must not have moved, the
    # section must be printed even when nothing is on the back, and the basis
    # must reach BOTH channels.
    import run_utils
    tool = os.path.join(ROOT, 'py_tools', 'check_assembly.py')
    with tempfile.TemporaryDirectory() as td:
        jp = os.path.join(td, 'a.json')
        env = dict(os.environ, KRT_NO_BANNER='1')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool,
                            board('splitflap_driver'), '--json', jp],
                           capture_output=True, text=True, cwd=ROOT, env=env)
        out = r.stdout + r.stderr
        check("CLI: a clean single-sided board still exits 0",
              r.returncode == 0, f"exit {r.returncode}\n{out[-800:]}")
        check("CLI: the section prints on a board with nothing on the back",
              'ASSEMBLY SIDES: F 65 / B 0' in out, out[-800:])
        check("CLI: the text channel names the counting rule",
              'per footprint BLOCK' in out, out[-800:])
        run_utils.evidence(jp, 'the assembly JSON')
        doc = json.load(open(jp, encoding='utf-8'))
        for k in ('parts_by_side', 'parts_by_side_blocks',
                  'parts_by_side_basis', 'back_side_zero_pad_blocks',
                  'through_hole_parts', 'assembly_sides', 'reflow_passes'):
            check(f"CLI: --json carries {k}", k in doc, sorted(doc))
        check("CLI: the JSON basis names #726",
              '#726' in doc.get('parts_by_side_basis', ''),
              doc.get('parts_by_side_basis'))
        # The verdict channel is untouched: this is a report, not a conjunct.
        check("CLI: the verdict did not learn about sides",
              doc.get('buildable') is True
              and 'blocking' in doc and doc['blocking'] == 0,
              f"buildable={doc.get('buildable')} blocking={doc.get('blocking')}")

    check("graded every fixture", graded == len(EXPECT), f"{graded}")
    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #837 census over {graded} "
          f"boards, {len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f"  - {f}")
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
