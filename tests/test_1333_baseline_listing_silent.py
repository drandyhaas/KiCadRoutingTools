#!/usr/bin/env python3
"""#1333: kicad_drc_compare --baseline printed the BASELINE board's whole
check_drc listing, unlabelled, at the head of the graded board's report.

`run_check_drc` grades in-process with `run_drc(quiet=True,
print_summary=False)`, whose contract (#383) is to print nothing so that a
caller under a thread pool needs no stdout redirect. It held only in the
success path: with violations, run_drc skipped the FAILED line and then fell
through to the full `FOUND N DRC VIOLATIONS` listing. A pile baseline opened
the report with `FOUND 729 DRC VIOLATIONS: PAD-PAD violations (729)` over a
graded board that was clean.

This test builds a board with a real short (two foreign tracks crossing) and
asserts:
  1. run_drc(quiet=True, print_summary=False) returns the violations and
     prints NOTHING;
  2. run_check_drc (the baseline's call in kicad_drc_compare) is silent too;
  3. the CLI path is unchanged: run_drc(quiet=True) still prints FAILED and
     the listing.

Needs no kicad-cli.

    python3 tests/test_1333_baseline_listing_silent.py
"""
import io
import os
import shutil
import sys
import tempfile
from contextlib import redirect_stdout

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                  # noqa: E402
from kicad_writer import generate_segment_sexpr           # noqa: E402
from check_drc import run_drc                             # noqa: E402
from kicad_drc_compare import run_check_drc               # noqa: E402
from run_utils import evidence                            # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def shorted_board(tmp):
    """cap_chain plus two crossing B.Cu tracks on two different nets."""
    path = os.path.join(tmp, 'short.kicad_pcb')
    shutil.copy(BOARD, path)
    nets = {n.name: n.net_id for n in parse_kicad_pcb(path).nets.values()}
    sx = [generate_segment_sexpr((10.0, 15.0), (20.0, 15.0), 0.2, 'B.Cu',
                                 nets['DPA_P'], 'DPA_P'),
          generate_segment_sexpr((15.0, 10.0), (15.0, 20.0), 0.2, 'B.Cu',
                                 nets['DPB_P'], 'DPB_P')]
    with open(path, encoding='utf-8') as f:
        c = f.read()
    lp = c.rfind(')')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(c[:lp] + '\n'.join(sx) + '\n' + c[lp:])
    return path


def captured(fn, *a, **kw):
    buf = io.StringIO()
    with redirect_stdout(buf):
        out = fn(*a, **kw)
    return out, buf.getvalue()


def main():
    tmp = tempfile.mkdtemp(prefix='t1333_')
    try:
        board = evidence(shorted_board(tmp))

        print("1: run_drc(quiet=True, print_summary=False) on a FAILING board")
        v, text = captured(run_drc, board, clearance=0.1, quiet=True,
                           print_summary=False)
        check("the board has violations (else this tests nothing)",
              bool(v), f"{len(v or [])} violation(s)")
        check("printed zero characters", text == '',
              f"{len(text)} chars, starts {text[:60]!r}")

        print("2: run_check_drc (kicad_drc_compare's baseline grade)")
        items, text = captured(run_check_drc, board, 0.1)
        check("returns the items", bool(items), f"{len(items or [])} item(s)")
        check("printed zero characters", text == '',
              f"{len(text)} chars, starts {text[:60]!r}")

        print("3: the CLI path is unchanged: run_drc(quiet=True)")
        v, text = captured(run_drc, board, clearance=0.1, quiet=True)
        check("prints FAILED", text.lstrip().startswith('FAILED'),
              repr(text[:40]))
        check("prints the listing", 'DRC VIOLATIONS' in text)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
