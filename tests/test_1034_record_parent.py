#!/usr/bin/env python3
"""#1034 (+ #1039 item 5): `converge.py record` lineage and argv honesty.

`record` took `parent_sha` from the last ACCEPTED row, so two lineages run
side by side (the skill's 9.3d "run each candidate order as a FULL chain
lineage") chained across each other: A1, made from A0, got B0 as its parent.

Invariants:
1. The issue's three-row repro with `--parent A0.kicad_pcb` (a PATH): A1's
   parent_sha is A0's sha, parent_source "parent".
2. `--parent <sha>` (the SHA form, and a unique prefix) resolves the same.
3. With no --parent, the parent is DERIVED from the recorded --argv: the
   first existing .kicad_pcb token whose sha is in the store, skipping the
   output board -- parent_source "argv".
4. Nothing to derive from: the last accepted row, parent_source
   "last_accepted", and a NOTE on stderr that says so and names --parent.
5. A --parent that is neither a file nor a known sha exits 2 with "Nothing
   was written" and the ledger is unchanged.
6. #1039.5: a --nets value list that LOOKS shell-expanded (a glob the shell
   expanded) exits 2 with "Nothing was written"; a real net name is fine.
7. `converge.parent_score` reads the corrected chain (A1 -> A0's score).

Run:
    python3 tests/test_1034_record_parent.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from board_store import sha256_file  # noqa: E402
from run_utils import check as run_check  # noqa: E402

CONVERGE = os.path.join(ROOT, 'py_placer', 'converge.py')
FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def board(work, name):
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator "t%s"))\n' % name)
    return p


def rows(ledger):
    with open(ledger, encoding='utf-8') as fh:
        return [json.loads(l) for l in fh if l.strip()]


def record(ledger, bd, lever, *extra, argv=None, score=None, **kw):
    cmd = [sys.executable, '-X', 'utf8', CONVERGE, 'record', '--ledger',
           ledger, '--board', bd, '--kind', 'completion', '--lever', lever]
    if score is not None:
        cmd += ['--score', json.dumps(score)]
    cmd += list(extra)
    cmd += ['--argv'] + (argv or [sys.executable, '-V'])
    return run_check(cmd, **(kw or {'accept': True}))


def main():
    work = tempfile.mkdtemp(prefix='krt1034_')
    try:
        A0, B0, A1 = (board(work, n) for n in ('A0', 'B0', 'A1'))
        sA0, sB0, sA1 = (sha256_file(p) for p in (A0, B0, A1))

        # 1 -- the issue's repro, with --parent as a PATH
        L = os.path.join(work, 'l1.jsonl')
        record(L, A0, 'lineage A start', score={'blocking': 5})
        record(L, B0, 'lineage B start', score={'blocking': 9})
        r = record(L, A1, 'A1 made FROM A0', '--parent', A0)
        rw = rows(L)
        check('1. A1.parent_sha is A0 (not the last accepted B0)',
              rw[2]['parent_sha'] == sA0 and rw[2]['result_sha'] == sA1,
              '%s vs A0 %s B0 %s' % (rw[2]['parent_sha'], sA0, sB0))
        check('1. parent_source records how it was resolved',
              rw[2].get('parent_source') == 'parent', str(rw[2].get('parent_source')))
        check('1. no NOTE when the parent was given', 'NOTE' not in r.stderr)

        # 7 -- the read side follows the corrected chain
        from converge import parent_score
        check('7. parent_score(A1) is A0\'s score',
              parent_score(rw, rw[2]) == {'blocking': 5},
              str(parent_score(rw, rw[2])))

        # 2 -- the sha form, full and a unique prefix
        for label, val in (('full sha', sA0), ('prefix', sA0[:12])):
            L2 = os.path.join(work, 'l2_%s.jsonl' % label.replace(' ', ''))
            record(L2, A0, 'A start')
            record(L2, B0, 'B start')
            record(L2, A1, 'A1', '--parent', val)
            rw = rows(L2)
            check('2. --parent <%s> resolves to A0' % label,
                  rw[2]['parent_sha'] == sA0
                  and rw[2].get('parent_source') == 'parent',
                  str(rw[2]))

        # 3 -- derived from the recorded argv
        L3 = os.path.join(work, 'l3.jsonl')
        record(L3, A0, 'A start')
        record(L3, B0, 'B start')
        r = record(L3, A1, 'A1 from argv',
                   argv=[sys.executable, '-c', 'pass', A0, '--out', A1])
        rw = rows(L3)
        check('3. no --parent: derived from the first stored .kicad_pcb in '
              '--argv',
              rw[2]['parent_sha'] == sA0 and rw[2].get('parent_source') == 'argv',
              str((rw[2]['parent_sha'], rw[2].get('parent_source'))))
        # ...and the OUTPUT board in the argv is never its own parent
        L3b = os.path.join(work, 'l3b.jsonl')
        record(L3b, A1, 'A1 first')
        record(L3b, B0, 'B start')
        record(L3b, A1, 'A1 again', argv=[sys.executable, '-c', 'pass', A1])
        rw = rows(L3b)
        check('3. the output board in the argv is skipped (falls back)',
              rw[2]['parent_sha'] == sB0
              and rw[2].get('parent_source') == 'last_accepted',
              str((rw[2]['parent_sha'], rw[2].get('parent_source'))))

        # 4 -- the fallback, said out loud
        L4 = os.path.join(work, 'l4.jsonl')
        record(L4, A0, 'A start')
        record(L4, B0, 'B start')
        r = record(L4, A1, 'A1 with nothing to derive from')
        rw = rows(L4)
        check('4. fallback: the last accepted row (B0), parent_source '
              'last_accepted',
              rw[2]['parent_sha'] == sB0
              and rw[2].get('parent_source') == 'last_accepted',
              str(rw[2]))
        check('4. ...with a NOTE naming --parent',
              'NOTE' in r.stderr and '--parent' in r.stderr, r.stderr[-400:])
        check('4. the very first row has no parent and no source',
              rw[0]['parent_sha'] is None and rw[0].get('parent_source') is None,
              str(rw[0]))

        # 5 -- a bad --parent is refused, nothing written
        n = len(rows(L4))
        for bad in (os.path.join(work, 'nope.kicad_pcb'), 'deadbeef' * 8):
            record(L4, A1, 'bad parent', '--parent', bad,
                   refuse='Nothing was written', code=2)
        check('5. a bad --parent (missing path, unknown sha) writes nothing',
              len(rows(L4)) == n)

        # 6 -- #1039.5: a glob-expanded --nets
        junk = os.path.join(work, 'README.md')
        with open(junk, 'w', encoding='utf-8') as fh:
            fh.write('x')
        n = len(rows(L4))
        record(L4, A1, 'globbed nets',
               argv=[sys.executable, '-c', 'pass', '--nets', '/A', junk,
                     '--grid-step', '0.05'],
               refuse='Nothing was written', code=2)
        check('6. an existing path among the --nets values is refused',
              len(rows(L4)) == n)
        record(L4, A1, 'real nets', '--parent', A0,
               argv=[sys.executable, '-c', 'pass', '--nets', '/A', 'GND*',
                     '--grid-step', '0.05'])
        check('6. real net names (and a quoted glob) are recorded',
              len(rows(L4)) == n + 1
              and rows(L4)[-1]['lever_argv'][4:6] == ['/A', 'GND*'])
        # ...in the cwd, where a bare-name glob expands. One extension-less
        # file that happens to share a net's name is a NET (no override
        # exists for it); two such files, `-n`, or an extension is a glob.
        for nm in ('GND', 'VCC', 'notes.txt'):
            with open(os.path.join(work, nm), 'w', encoding='utf-8') as fh:
                fh.write('x')
        n = len(rows(L4))
        record(L4, A1, 'net named like a file', '--parent', A0,
               argv=[sys.executable, '-c', 'pass', '--nets', 'GND', '/A'],
               accept=True, cwd=work)
        check('6. --nets GND beside a file named GND is recorded',
              len(rows(L4)) == n + 1)
        n = len(rows(L4))
        for label, argv in (
                ('two extension-less files', ['--nets', 'GND', 'VCC']),
                ('a file extension', ['--nets', '/A', 'notes.txt']),
                ('-n spelling', ['-n', 'GND', 'VCC'])):
            record(L4, A1, 'globbed ' + label, '--parent', A0,
                   argv=[sys.executable, '-c', 'pass'] + argv,
                   refuse='Nothing was written', code=2, cwd=work)
        check('6. two files / an extension / -n are refused, nothing written',
              len(rows(L4)) == n)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    if FAILS:
        print('\nFAILED: %d check(s): %s' % (len(FAILS), FAILS))
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
