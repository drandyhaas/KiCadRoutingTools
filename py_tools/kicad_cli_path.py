#!/usr/bin/env python3
"""Print the path of kicad-cli, found the way every tool here finds it (#1418).

A stock macOS KiCad puts kicad-cli inside the app bundle, not on PATH, so a
bare `kicad-cli ...` in a skill exits 127 on a machine that has KiCad 10
installed -- and a skill whose fallback is "without kicad-cli, say it is
ungraded" then ships ungraded output. The tools never had that problem: they
resolve the binary through `kicad_oracle.find_kicad_cli` ($KICAD_CLI first,
then PATH and the packaged locations, then every versioned Windows install,
newest by numeric version). This exposes that one resolver to a shell:

    KCLI=$(python3 py_tools/kicad_cli_path.py) && "$KCLI" pcb drc board.kicad_pcb ...

stdout carries ONLY the path (no banner), so command substitution is safe.
Exit 0 when found; exit 1 with the reason on stderr when not, so the `&&`
chain stops instead of running an empty command. Only then is kicad-cli
really unavailable.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'routing', 'combined'], 'kind': 'utility'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import sys


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Print the path of kicad-cli as the repo's tools resolve "
                    "it ($KICAD_CLI, PATH, the KiCad app bundle, versioned "
                    "Windows installs). Exit 1 when none is found.")
    parser.parse_args(argv)
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        print("kicad_cli_path: kicad-cli not found ($KICAD_CLI unset or "
              "missing, not on PATH, no KiCad install found). Set KICAD_CLI "
              "to the binary's path.", file=sys.stderr)
        return 1
    print(cli)
    return 0


if __name__ == '__main__':
    sys.exit(main())
