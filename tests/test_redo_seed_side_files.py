"""redo_stress_test --remap/--workdir must carry manifest SIDE-FILES.

seed_input_boards copies .kicad_pcb inputs into the replay dir, but a manifest
step can also take a plain file argument that lives in the original run dir and
is produced by no recorded command -- artix_devboard's
`--net-clearances net_clearances_1v8.json`. Replayed into a fresh dir that file
is absent, the step dies with FileNotFoundError, and the replay STILL exits
rc=0: the chain is silently truncated and the missing final board reads as a
routing failure.

Also pins the two things that make this safe to do by existence test:
  - numeric flag values (`--clearance 0.25`) and net names (`+3.3V`) must never
    be treated as files -- an extension-based test would match both;
  - values of OUTPUT flags must not be pre-seeded from a stale run dir.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_tools'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tests/stress'))

from redo_stress_test import seed_side_files  # noqa: E402


def main():
    failures = []
    with tempfile.TemporaryDirectory() as src, tempfile.TemporaryDirectory() as dst:
        manifest = os.path.join(src, "redo_commands.sh")
        open(manifest, "w").write("# manifest\n")
        # A real side-file the chain needs...
        open(os.path.join(src, "net_clearances_1v8.json"), "w").write('{"+1V8": 0.25}')
        # ...and a stale OUTPUT from the original run that must NOT be carried.
        open(os.path.join(src, "summary.json"), "w").write("{}")
        # A file whose name collides with a numeric flag value, to prove the
        # existence test (not an extension test) is what gates copying.
        open(os.path.join(src, "0.25"), "w").write("decoy")

        cmds = [
            (src, ["py_router/route.py", "a.kicad_pcb", "b.kicad_pcb",
                   "--net-clearances", "net_clearances_1v8.json",
                   "--clearance", "0.25", "--nets", "+3.3V",
                   "--json-out", "summary.json"]),
        ]
        seeded = seed_side_files(manifest, cmds, dst)

        if "net_clearances_1v8.json" not in seeded:
            failures.append("the --net-clearances side-file was NOT seeded "
                            f"(seeded={seeded}) -- the chain would truncate")
        elif not os.path.exists(os.path.join(dst, "net_clearances_1v8.json")):
            failures.append("seeded list claims the file but it was not copied")
        else:
            print("PASS: --net-clearances side-file carried into the replay dir")

        if "summary.json" in seeded:
            failures.append("a stale OUTPUT file (--json-out summary.json) was "
                            "pre-seeded into the replay dir")
        else:
            print("PASS: output-flag value not pre-seeded")

        # "0.25" exists as a decoy file, but it is the VALUE of --clearance.
        # Seeding it is harmless for routing, yet it shows the copy decision
        # reaching flag values it should leave alone; assert we skip it.
        if "0.25" in seeded:
            failures.append("the numeric flag value '0.25' was treated as a "
                            "file -- flag values must not be seeded")
        else:
            print("PASS: numeric flag value not treated as a file")

    if failures:
        print("\nFAIL:")
        for f in failures:
            print("  - " + f)
        return 1
    print("\nALL CHECKS PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
