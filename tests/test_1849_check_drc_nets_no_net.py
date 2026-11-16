#!/usr/bin/env python3
"""#1849: `check_drc --nets` with a pattern naming no net is not a clean grade.

A mistyped or stale `--nets` pattern selected nothing, and nothing selected
printed `NO DRC VIOLATIONS FOUND!` with exit 0. Each pattern is now accounted
on its own (`net_queries.account_net_patterns`), the ones naming no net of the
board are named, `--json` carries them (`patterns_matching_no_net`), and the
exit is 2, as `check_return_vias --nets` does.

Pinned here:
  1. a pattern naming no net: named, `NO NETS GRADED` instead of the clean
     line, exit 2, and the JSON lists it with `nets_selected` 0;
  2. one pattern that names a net beside one that does not: the named net is
     graded (its violation is reported), the other is named, exit 2;
  3. control: every pattern names a net -- the same grade as (2), no note,
     the usual exit (1 here: the net has a violation), `[]` in the JSON;
  4. an unqualified pattern selects the sheet-qualified net it names, as
     route.py's `--nets` does (plain fnmatch selected nothing for it, and the
     run read clean);
  5. patterns that each name a net but together select none (an exclusion
     of the only net included) exit 2 too.
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import check, evidence  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
TOOL = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
        BOARD, '--clearance', '0.1']
#: a net of the fixture with one clearance violation against its neighbour
NET, NEIGHBOUR = 'Net-(U2A-DATA_18)', 'Net-(U2A-DATA_19)'
#: a net declared on a sheet, and its name without the sheet path
SHEET_NET, LEAF = '/fpga_adc/VLVDS', 'VLVDS'
REASON = "--nets pattern(s) matching no net of the board: 'NO_SUCH_NET'"


def _doc(path):
    with open(evidence(path, 'check_drc --json'), encoding='utf-8') as f:
        return json.load(f)


def _pairs(doc):
    return sorted((i.get('type'), i.get('net1'), i.get('net2'))
                  for i in doc['items'] if not i.get('accepted'))


def main():
    evidence(BOARD, 'fixture board')
    with tempfile.TemporaryDirectory() as td:
        js = os.path.join(td, 'drc.json')

        print("1: a pattern naming no net is named, and exits 2")
        r = check(TOOL + ['--nets', 'NO_SUCH_NET', '--json', js], refuse=REASON, code=2)
        assert 'NO NETS GRADED' in r.stdout and 'NO DRC VIOLATIONS FOUND' not in r.stdout, \
            r.stdout[-800:]
        d = _doc(js)
        assert d['patterns_matching_no_net'] == ['NO_SUCH_NET'] and d['nets_selected'] == 0, \
            (d['patterns_matching_no_net'], d['nets_selected'])
        os.remove(js)

        print("2: a matched pattern beside an unmatched one: graded and named")
        r = check(TOOL + ['--nets', NET, 'NO_SUCH_NET', '--json', js], refuse=REASON, code=2)
        assert 'the grade covers only the nets the other patterns name' in r.stdout, \
            r.stdout[-800:]
        mixed = _doc(js)
        assert mixed['patterns_matching_no_net'] == ['NO_SUCH_NET'], mixed['patterns_matching_no_net']
        assert ('segment-segment', NET, NEIGHBOUR) in _pairs(mixed), _pairs(mixed)
        os.remove(js)

        print("3: control -- every pattern names a net: no note, the usual exit")
        r = check(TOOL + ['--nets', NET, '--json', js], code=1)
        assert 'matching no net' not in r.stdout and 'NO NETS GRADED' not in r.stdout, \
            r.stdout[-800:]
        ctl = _doc(js)
        assert ctl['patterns_matching_no_net'] == [] and ctl['nets_selected'] == 1, \
            (ctl['patterns_matching_no_net'], ctl['nets_selected'])
        assert _pairs(ctl) == _pairs(mixed) and ctl['violations'] == mixed['violations'] == 1, \
            (_pairs(ctl), _pairs(mixed))
        os.remove(js)
        r = check(TOOL + ['--nets', SHEET_NET], accept=True)
        assert 'matching no net' not in r.stdout and 'NO DRC VIOLATIONS FOUND!' in r.stdout, \
            r.stdout[-800:]

        print("4: an unqualified pattern selects the sheet net it names")
        r = check(TOOL + ['--nets', LEAF, '--json', js], accept=True)
        d = _doc(js)
        assert d['nets_selected'] == 1 and d['patterns_matching_no_net'] == [], \
            (d['nets_selected'], d['patterns_matching_no_net'])
        assert 'Filtering to 1 matching nets' in r.stdout, r.stdout[-800:]
        os.remove(js)

        print("5: patterns that together select no net exit 2")
        check(TOOL + ['--nets', NET, '!' + NET],
              refuse="selects no net of the board -- no net is graded", code=2)
    print("PASS")


if __name__ == '__main__':
    main()
