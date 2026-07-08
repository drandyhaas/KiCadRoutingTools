#!/usr/bin/env python3
"""parse_manifest must treat a recorded `# cwd=` as STICKY -- it applies to every
following command until the next `# cwd=`, not just the one immediately after it.

run_limited.sh records a `# cwd=` before each command it wraps, so normally every
command carries its own and stickiness is a no-op. It matters for the de-hole
`cp <step> <final>` + follow-up splice (#334/#345): the `cp` consumes the shared
`# cwd=` and the follow-up command has none of its own. When cwd reset to None
there, the follow-up ran in the launcher's cwd instead of the run dir, so
`route.py final.kicad_pcb ...` hit FileNotFoundError on the just-cp'd board and
the board was falsely graded chain-broken under --remap (eurorack_pmod / fpga_sdram
/ kuchen in the ab_main_0708a wave).

    python3 tests/test_redo_manifest_cwd.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "tests", "stress"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "tests"))

from stress.redo_stress_test import parse_manifest  # noqa: E402


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # A de-hole-spliced manifest: one `# cwd=` shared by the `cp` and the route
    # that reads the cp'd board -- exactly the #334/#345 shape.
    manifest = """\
# cwd=/runs/eurorack_pmod
python3 route_planes.py step2.kicad_pcb step4_planes.kicad_pcb
# cwd=/runs/eurorack_pmod
python3 route_disconnected_planes.py step4_planes.kicad_pcb step5_repair.kicad_pcb
# cwd=/runs/eurorack_pmod
# de-hole (#334): original session copied this file without recording it
cp step5_repair.kicad_pcb final.kicad_pcb
python3 route.py final.kicad_pcb step6.kicad_pcb
"""
    with tempfile.NamedTemporaryFile("w", suffix=".sh", delete=False) as f:
        f.write(manifest)
        path = f.name
    try:
        cmds = parse_manifest(path)
    finally:
        os.unlink(path)

    check("command count", len(cmds) == 4)
    if len(cmds) == 4:
        (c0, a0), (c1, a1), (c2, a2), (c3, a3) = cmds
        check("route_planes cwd", c0 == "/runs/eurorack_pmod")
        check("route_disconnected_planes cwd", c1 == "/runs/eurorack_pmod")
        check("cp cwd", c2 == "/runs/eurorack_pmod" and a2[0] == "cp")
        # The follow-up route.py has NO `# cwd=` of its own -- it must INHERIT the
        # cp's cwd (sticky), not fall back to None (launcher cwd).
        check("follow-up route.py inherits cwd (sticky)",
              c3 == "/runs/eurorack_pmod" and a3[1].endswith("route.py"))

    if fails:
        print("FAIL: " + ", ".join(fails))
        return 1
    print("PASS: parse_manifest carries `# cwd=` stickily across the de-hole cp splice")
    return 0


if __name__ == "__main__":
    sys.exit(run())
