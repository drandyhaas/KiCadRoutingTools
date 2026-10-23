#!/usr/bin/env python3
"""Edges spelled the other file's way: an intent's edge_connectors take the
groups file's letters (N/S/E/W), and a groups file takes the intent's names
(north/south/east/west).

The groups file spells its edge hints as letters and the intent spells them
out, and an agent writing both carries one spelling into the other: three of
four live PCB-chat placement runs on 2026-10-22 wrote `edge: 'E'` into the
intent's edge_connectors, and check_groups exited 2 on `edge_connectors[0]
(CON1): edge 'E', expected one of north, south, east, west`.

Asserts:
  1. each letter in an intent edge_connectors row loads as the edge it names;
  2. each name in a groups file loads as its letter;
  3. anything else is still refused, in both files;
  4. check_groups, end to end, reads an intent with letters and a groups
     file with names (it exited 2 before).

    python3 tests/test_edge_spelling_aliases.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

from placement.floorplan import (IntentError, intent_from_dict,    # noqa: E402
                                 KIND, SCHEMA_VERSION, READER_VERSION)
from placement.grouping import GroupsError, validate_doc           # noqa: E402
from run_utils import check, evidence                              # noqa: E402

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def intent_with_edge(edge):
    return {'schema': SCHEMA_VERSION, 'kind': KIND, 'units': 'mm',
            'min_reader': READER_VERSION,
            'edge_connectors': [{'ref': 'J1', 'edge': edge}]}


def groups_with_edge(edge):
    return {'schema': 1, 'groups': [{'name': 'io', 'refs': ['J1'], 'edge': edge}]}


def main():
    print("1: an intent takes the groups file's letters")
    for letter, name in (('N', 'north'), ('S', 'south'), ('E', 'east'), ('W', 'west')):
        try:
            got = intent_from_dict(intent_with_edge(letter)).edge_connectors[0]['edge']
        except IntentError as e:
            got = f'refused: {e}'
        ok(f"edge {letter!r} reads as {name!r}", got == name, got)

    print("2: a groups file takes the intent's names")
    for name, letter in (('north', 'N'), ('south', 'S'), ('east', 'E'), ('west', 'W')):
        try:
            got = validate_doc(groups_with_edge(name))['groups'][0]['edge']
        except GroupsError as e:
            got = f'refused: {e}'
        ok(f"edge {name!r} reads as {letter!r}", got == letter, got)

    print("3: anything else is still refused")
    for edge in ('X', 'e', 'East', 'top', 3):
        try:
            intent_from_dict(intent_with_edge(edge))
            err = None
        except IntentError as e:
            err = str(e)
        ok(f"intent edge {edge!r}", err is not None and 'expected one of north' in err,
           err or 'loaded')
        try:
            validate_doc(groups_with_edge(edge))
            err = None
        except GroupsError as e:
            err = str(e)
        ok(f"groups edge {edge!r}", err is not None and 'is not one of' in err,
           err or 'loaded')

    print("4: check_groups end to end")
    board = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
    tmp = tempfile.mkdtemp(prefix='tedge_')
    try:
        intent = os.path.join(tmp, 'intent.json')
        with open(intent, 'w', encoding='utf-8') as f:
            json.dump(dict(intent_with_edge('E'),
                           edge_connectors=[{'ref': 'CON1', 'edge': 'E'},
                                            {'ref': 'USB1', 'edge': 'W'}]), f)
        groups = os.path.join(tmp, 'groups.json')
        with open(groups, 'w', encoding='utf-8') as f:
            json.dump({'schema': 1, 'groups': [
                {'name': 'usb_in', 'refs': ['USB1'], 'edge': 'west'},
                {'name': 'out_con', 'refs': ['CON1'], 'edge': 'east'},
                {'name': 'core', 'refs': ['C1', 'C2', 'C3', 'C4', 'CON2', 'Q1', 'Q2',
                                          'R1', 'R2', 'R3', 'R4', 'Ref*', 'Ref*~2',
                                          'U1', 'U2', 'Y1']}]}, f)
        evidence(intent, 'the intent')
        evidence(groups, 'the groups file')
        r = check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'check_groups.py'),
                   board, '--groups', groups, '--intent', intent], accept=True)
        log = r.stdout + r.stderr
        ok("read, not refused", 'cannot read' not in log and 'JSON_SUMMARY:' in log,
           log[-300:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
