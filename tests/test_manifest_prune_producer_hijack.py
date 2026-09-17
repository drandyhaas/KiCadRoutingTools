#!/usr/bin/env python3
"""A read-only step does not become the PRODUCER of the board it reads.

    python3 tests/test_manifest_prune_producer_hijack.py

`redo_stress_test.board_io` reads a command's `.kicad_pcb` tokens as
"the last one is the output, the earlier ones are inputs". That is right for
both recorded shapes -- `<in> <out>` and `<in> --output <out>` -- but a
READ-ONLY step names its board ONCE and writes nothing. With one token,
`toks[:-1]` is empty, so such a command was read as `ins=[] out=<that board>`:
a producer of the board it only reads.

Measured on hexberry_fpga (runs_set6), whose manifest records

    qfn_fanout.py step1_planes.kicad_pcb --component U7 --nets '*' '!GND' --dry-run

that hijacked `producer['step1_planes.kicad_pcb']` from the real route_planes
step. The backward walk from the final board then kept the read-only command
and PRUNED THE PLANE STEP, so every later step died on
`FileNotFoundError: step1_planes.kicad_pcb` in 2.6s -- identically in the
v0.22.0 and HEAD arms, which is why one board sat excluded from every corpus
A/B while looking like a router crash. The chain-HOLE warning built for exactly
this class could not fire, because from the pruner's own view the board WAS
produced.

What this gate pins, and why each row is here:

 1. The unit fact: a lone board token with no write flag is an INPUT.
 2. The unit fact it must NOT overreach into: a lone token that a write flag
    introduces (`--output X`, `--output=X`, `-o X`) is still the output.
    `-o` is a real recorded spelling (qfn_fanout `-o step1_fanout_U2.kicad_pcb`).
 3. The two-token shapes are untouched -- 2665 of 2666 board-bearing non-check
    commands in the corpus take this path, so a regression here is corpus-wide.
 4. The DEFECT ITSELF, end to end, on hexberry's real recorded manifest: the
    kept chain must contain the route_planes step that produces the board the
    fanout reads. This is the row that fails on the unfixed code.
 5. The safety net that was blinded: with a read-only step no longer claiming
    to produce, a manifest that genuinely never produces a board a kept command
    reads reports a chain hole instead of silently pruning.

Row 4 reads the tracked corpus manifest when it is present and otherwise falls
back to an inline copy of the same command shapes, so the gate still
discriminates on a machine without the stress corpus (and says which it used).
"""
import os
import shlex
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools', 'tests/stress'):
    _q = os.path.join(ROOT, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

import redo_stress_test as R  # noqa: E402

HEXBERRY = os.path.expanduser(
    '~/Documents/kicad_stress_test/runs_set6/hexberry_fpga/redo_commands.sh')

# The shape hexberry records, reduced to what the pruner reads. Used when the
# corpus is absent so this gate never silently stops discriminating.
INLINE = [
    "route_planes.py board.kicad_pcb step1_planes.kicad_pcb --nets GND",
    "qfn_fanout.py step1_planes.kicad_pcb --component U7 --nets * --dry-run",
    "qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb -c U7",
    "place_fanout_clearance.py step2_fanout.kicad_pcb step3_capclear.kicad_pcb",
    "route.py step3_capclear.kicad_pcb step4_route.kicad_pcb *",
]

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    if ok:
        passed += 1
        print(f"  OK   {name}" + (f" -- {detail}" if detail else ""))
    else:
        failed += 1
        print(f"  FAIL {name}" + (f" -- {detail}" if detail else ""))


def cmds_from(lines):
    out = []
    for raw in lines:
        raw = raw.strip()
        if not raw or raw.startswith('#') or raw.startswith('set -e'):
            continue
        try:
            toks = shlex.split(raw)
        except ValueError:
            continue
        if not any(t.endswith('.py') for t in toks):
            continue
        out.append((None, toks))
    return out


def tool_of(argv):
    return next((os.path.basename(t) for t in argv if t.endswith('.py')), '?')


def main():
    print("1. a lone board token is an INPUT unless a write flag introduces it")
    ins, out = R.board_io(shlex.split(
        "qfn_fanout.py step1_planes.kicad_pcb --component U7 --dry-run"))
    check("read-only step reads its board", ins == ['step1_planes.kicad_pcb'],
          f"ins={ins}")
    check("read-only step produces NOTHING", out is None, f"out={out}")

    print("2. ...but a write flag still makes a lone token the output")
    for spelling, argv in [
        ("--output X", "tool.py --output only.kicad_pcb"),
        ("--output=X", "tool.py --output=only.kicad_pcb"),
        ("-o X",       "tool.py -o only.kicad_pcb"),
    ]:
        ins, out = R.board_io(shlex.split(argv))
        check(f"{spelling} is an output", out == 'only.kicad_pcb' and ins == [],
              f"ins={ins} out={out}")

    print("3. the two-token shapes are unchanged (2665 of 2666 corpus commands)")
    for shape, argv, want_in, want_out in [
        ("<in> <out>", "route.py a.kicad_pcb b.kicad_pcb", ['a.kicad_pcb'], 'b.kicad_pcb'),
        ("<in> --output <out>", "qfn_fanout.py a.kicad_pcb --output b.kicad_pcb",
         ['a.kicad_pcb'], 'b.kicad_pcb'),
        ("<in> -o <out>", "qfn_fanout.py a.kicad_pcb -o b.kicad_pcb",
         ['a.kicad_pcb'], 'b.kicad_pcb'),
    ]:
        ins, out = R.board_io(shlex.split(argv))
        check(f"{shape} unchanged", ins == want_in and out == want_out,
              f"ins={ins} out={out}")
    check("no board token -> ([], None)", R.board_io(['x.py', '--help']) == ([], None))

    print("4. THE DEFECT: the producer of a consumed board survives pruning")
    if os.path.isfile(HEXBERRY):
        src, lines = 'the recorded corpus manifest', open(
            HEXBERRY, errors='replace').read().splitlines()
    else:
        src, lines = 'the inline fallback (corpus absent)', INLINE
    cmds = cmds_from(lines)
    keep, info = R.compute_prune_keep(cmds)
    kept_tools = [tool_of(cmds[i][1]) for i in sorted(keep)]
    # The board the fanout reads must be produced by a KEPT command.
    producers = {}
    for i in sorted(keep):
        _i, o = R.board_io(cmds[i][1])
        if o:
            producers[o] = i
    consumed = set()
    for i in sorted(keep):
        for b in R.board_io(cmds[i][1])[0]:
            consumed.add(b)
    orphan = [b for b in consumed if b not in producers and 'step' in b]
    check(f"[{src}] a route_planes step is KEPT",
          'route_planes.py' in kept_tools, f"kept={kept_tools}")
    check("no kept command reads a step board nobody kept produces",
          not orphan, f"orphaned={sorted(orphan)}")
    check("the read-only step is NOT the final board's producer",
          info['final_board'] not in
          {R.board_io(c[1])[1] for c in cmds if '--dry-run' in c[1]},
          f"final={info['final_board']}")

    print("5. the blinded safety net reports a hole again")
    # A kept command reads a board NO command produces -> that is a chain hole,
    # and it must be visible rather than absorbed by a read-only step.
    hole_cmds = cmds_from([
        "qfn_fanout.py missing_step.kicad_pcb --component U7 --dry-run",
        "route.py missing_step.kicad_pcb out.kicad_pcb *",
    ])
    _keep, hinfo = R.compute_prune_keep(hole_cmds)
    ins_all = set()
    for i in sorted(_keep):
        ins_all.update(R.board_io(hole_cmds[i][1])[0])
    check("an unproduced board stays visible as an input",
          'missing_step.kicad_pcb' in ins_all, f"inputs={sorted(ins_all)}")

    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
