#!/usr/bin/env python3
"""Minimize a recorded replay manifest (issue #132) to the smallest set of
commands that still reproduces the final board.

A stress run records EVERY board-mutating command, including the LLM agent's
trial-and-error: `--help` probes, retries that overwrite an output with
different parameters, and dead-end attempts whose output is never consumed.
For deterministic replay only the commands on the data-flow path to the final
output matter.

Approach: model each command by the .kicad_pcb file(s) it READS (inputs) and
the one it WRITES (output). Walking commands in order, bind every read to the
file's CURRENT producer (so an overwrite re-binds later reads to the newer
command, dropping the superseded one). Then keep the transitive closure of
producers feeding the final output; emit the kept commands in original order.

Usage:
  minimize_manifest.py <manifest.sh> [-o OUT.sh] [--final PATH] [--quiet]

With no -o, prints the minimized manifest to stdout. --final overrides the
auto-detected final board (default: output of the last mutating command).
"""
import argparse
import os
import re
import shlex
import sys


def parse_commands(path):
    """Return (header_lines, commands). Each command: dict with raw line, cwd,
    list of inputs (kicad_pcb read), output (kicad_pcb written or None)."""
    header = []
    commands = []
    cwd = None
    seen_first_cmd = False
    for raw in open(path):
        line = raw.rstrip("\n")
        stripped = line.strip()
        if stripped.startswith("# cwd="):
            cwd = stripped[len("# cwd="):]
            continue
        if not stripped or stripped.startswith("#") or stripped.startswith("set "):
            if not seen_first_cmd:
                header.append(line)
            continue
        # A command line.
        seen_first_cmd = True
        toks = shlex.split(line)
        pcbs = [t for t in toks if t.endswith(".kicad_pcb")]
        out = None
        ins = []
        if pcbs:
            # Output: value after --output/-o if present, else the 2nd positional pcb.
            out_flag = None
            for i, t in enumerate(toks):
                if t in ("--output", "-o") and i + 1 < len(toks):
                    out_flag = toks[i + 1]
                    break
            if out_flag and out_flag.endswith(".kicad_pcb"):
                out = out_flag
                ins = [p for p in pcbs if p != out]
            elif len(pcbs) >= 2:
                ins = [pcbs[0]]
                out = pcbs[1]
            else:
                ins = [pcbs[0]]  # read-only (e.g. check_*/list_nets)
        commands.append({"line": line, "cwd": cwd, "inputs": ins, "output": out})
    return header, commands


def minimize(commands, final=None):
    """Return the list of command indices to keep (sorted)."""
    producer = {}          # file -> index of command that last wrote it
    deps = {i: set() for i in range(len(commands))}  # cmd -> cmds it depends on
    for i, c in enumerate(commands):
        for f in c["inputs"]:
            if f in producer:
                deps[i].add(producer[f])
        if c["output"]:
            producer[c["output"]] = i

    # Final board: explicit, else the last command that wrote a file.
    if final is None:
        writers = [i for i, c in enumerate(commands) if c["output"]]
        if not writers:
            return []
        final_idx = writers[-1]
    else:
        final = os.path.basename(final)
        final_idx = None
        for i in range(len(commands) - 1, -1, -1):
            if commands[i]["output"] and os.path.basename(commands[i]["output"]) == final:
                final_idx = i
                break
        if final_idx is None:
            raise SystemExit(f"no command writes final board {final}")

    keep = set()
    stack = [final_idx]
    while stack:
        i = stack.pop()
        if i in keep:
            continue
        keep.add(i)
        stack.extend(deps[i])
    return sorted(keep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("manifest")
    ap.add_argument("-o", "--out")
    ap.add_argument("--final")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    # Never clobber the recorded manifest: it is the stress run's artifact.
    if args.out and os.path.abspath(args.out) == os.path.abspath(args.manifest):
        raise SystemExit("refusing to overwrite the recorded manifest; choose a "
                         "different -o path (e.g. redo_commands.min.sh)")

    header, commands = parse_commands(args.manifest)
    keep = minimize(commands, args.final)

    out_lines = list(header)
    for i in keep:
        c = commands[i]
        if c["cwd"]:
            out_lines.append(f"# cwd={c['cwd']}")
        out_lines.append(c["line"])
    text = "\n".join(out_lines) + "\n"

    n_cmd = len(commands)
    dropped = [i for i in range(n_cmd) if i not in keep]
    if not args.quiet:
        sys.stderr.write(f"{args.manifest}: {n_cmd} commands -> {len(keep)} kept, "
                         f"{len(dropped)} dropped\n")
        for i in dropped:
            c = commands[i]
            reason = ("read-only/no-op" if not c["output"]
                      else "superseded/dead-end output")
            sys.stderr.write(f"  drop [{i}] ({reason}): {c['line'][:90]}\n")

    if args.out:
        open(args.out, "w").write(text)
        os.chmod(args.out, 0o755)
        if not args.quiet:
            sys.stderr.write(f"wrote {args.out}\n")
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
