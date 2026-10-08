#!/usr/bin/env python3
"""A top-level route run starts with empty run ledgers, as a fresh parse does.

The CLI parses a new PCBData per step; the GUI keeps ONE across runs. The
run-scoped ledgers on it (`rip_up_reroute.RUN_LEDGERS`: pre-existing rip
victims, saved rip payloads, #134 refusals, #666 queued cap moves, exact-name
rip overrides, via-unblock blame, #189 shrunk-via sizes) were never cleared,
so a second GUI run read the first run's victims as its own -- pulling them
into its escalation and stale-strip scope -- and emitted shrunk vias at cells
an earlier run had registered.

Checks, through the real engines on a tiny board:
  1. batch_route (top level, final_reconcile=True) clears every ledger;
  2. a NESTED batch_route (final_reconcile=False) keeps them -- sub-runs
     share the outer run's;
  3. batch_route_diff_pairs (always top level) clears every ledger.

    python3 tests/test_run_ledgers_reset.py
"""
import contextlib
import io
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from rip_up_reroute import RUN_LEDGERS  # noqa: E402
from route import batch_route  # noqa: E402
from route_diff import batch_route_diff_pairs  # noqa: E402

BOARD = os.path.join(REPO, 'kicad_files', 'cap_chain.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f" ({detail})" if detail else ''))
    if not ok:
        failures.append(label)


def seeded():
    pcb = parse_kicad_pcb(BOARD)
    for attr in RUN_LEDGERS:
        setattr(pcb, attr, {'stale': 1})
    return pcb


def left(pcb):
    return [a for a in RUN_LEDGERS if a in vars(pcb)]


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


pcb = seeded()
quiet(batch_route, BOARD, '', [], pcb_data=pcb, return_results=True)
check('top-level batch_route clears every run ledger', not left(pcb), str(left(pcb)))

pcb = seeded()
quiet(batch_route, BOARD, '', [], pcb_data=pcb, return_results=True,
      final_reconcile=False)
check('nested batch_route keeps the outer run ledgers',
      left(pcb) == list(RUN_LEDGERS), f'{len(left(pcb))}/{len(RUN_LEDGERS)} kept')

pcb = seeded()
quiet(batch_route_diff_pairs, BOARD, '', [], pcb_data=pcb, return_results=True)
check('batch_route_diff_pairs clears every run ledger', not left(pcb), str(left(pcb)))

if failures:
    print(f'FAIL: {len(failures)}')
    sys.exit(1)
print('ALL PASS')
