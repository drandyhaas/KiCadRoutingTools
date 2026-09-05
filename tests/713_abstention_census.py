#!/usr/bin/env python3
"""Phase 0 census for #713 item 5: on how many tracked boards does the default
emit-intent -> grade round trip report a CLEAN verdict while a channel the
intent ASKED FOR went ungraded?

`rules_skipped` holds TWO different things and only one of them is an
abstention -- the first pass of this census conflated them and reported 21/22.
floorplan.grade() writes it at two sites:

  :3088  `not _wants(intent, name)` -> `_SKIP_REASON[name]`, every one of which
         reads "the intent declares no X". NOBODY ASKED. That is honest and
         must NOT make a verdict incomplete, or every board with a minimal
         intent is permanently incomplete. EXCEPT when a withheld key disarmed
         the rule, in which case the reason gains "; the emitter WITHHELD ...".
  :3093  `_ARM[name](ctx)` -> "the intent asked, this BOARD cannot answer".
         A real abstention.

So the ungraded-but-declared channels are:
  budget_abstained         the emitter could not derive a declared key
  rules_skipped (_ARM)     the intent asked, the board cannot answer
  rules_skipped (WITHHELD) the emitter refused to derive the key this rule needs
  edge_seating_abstained   a measurement taken with no basis

and rules_skipped (not asked) is NOT one of them.

The `CMD:` banner means stdout is not bare JSON; the summary is read off the
`JSON_SUMMARY:` line and the reason strings out of the --json document.

    python3 -X utf8 tests/713_abstention_census.py             # re-record
    python3 -X utf8 tests/713_abstention_census.py --out PATH  # somewhere else

WHY THIS HAS A `main()` AND AN `--out` (#879). It was a bare top-level script
whose output path came from `__file__`, and that meant two things: IMPORTING it
ran the whole 42 s census, and it always wrote over the committed baseline.
Both blocked the drift gate that now guards it
(`tests/test_713_abstention_drift.py`), which has to re-derive the census
somewhere harmless and compare. The default path is unchanged, so a plain
invocation still re-records exactly what it always did.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
#: The committed baseline. Derived, never hardcoded: this file is committed and
#: an absolute path from the machine that first ran it would make the census
#: unrunnable anywhere else.
DEFAULT_OUT = os.path.join(HERE, '713_abstention_census.json')

WITHHELD_MARK = 'the emitter WITHHELD'

#: Every `_SKIP_REASON` value, verbatim from floorplan.py. A skip whose reason
#: is one of these (and carries no WITHHELD note) means nobody asked.
#:
#: HAND-MIRRORED, and it has already fallen out of step once: #837 added
#: `assembly_side` to `RULES` and this set was not updated, so a board with no
#: pad-bearing part had an honest skip classified as a real abstention.
#: `test_713_abstention_drift.py` asserts the two sets are equal in BOTH
#: directions in milliseconds, so the next one fails a gate rather than
#: silently changing a census (#879).
NOT_ASKED = {
    'the intent declares no envelope.rect',
    'no block declares a zone',
    'no block declares a side',
    'no block is marked exclusive',
    'the intent declares no keepouts',
    'the intent declares no edge_connectors',
    'the intent declares no decaps.max_distance_mm',
    'the intent declares no decaps.max_pin_distance_mm',
    'the intent declares no must_lock patterns',
    'the intent declares no legality_budget',
    'the intent declares no assembly.sides',
    'not requested',
}


def classify(reason):
    """-> 'withheld' | 'not_asked' | 'arm'."""
    if WITHHELD_MARK in reason:
        return 'withheld'
    if reason in NOT_ASKED:
        return 'not_asked'
    return 'arm'


def tracked_boards(repo):
    """The TRACKED board names, via git -- not os.listdir.

    Running the suite leaves 11 gitignored routing artifacts in kicad_files/
    (fanout_output*, interf_u_routed, sonde_u_routed*, ...), so a listdir
    census silently grades 33 boards before a suite run and 22 after one.
    Measured: exactly that happened between two runs of this census on the
    same tree.
    """
    return sorted(
        os.path.basename(p) for p in subprocess.run(
            ['git', 'ls-files', 'kicad_files/*.kicad_pcb'],
            capture_output=True, text=True, cwd=repo,
            check=True).stdout.split()
        if p.endswith('.kicad_pcb'))


def run(argv, repo):
    return subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=repo, timeout=900)


def summary_of(text):
    for line in text.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            return json.loads(line[len('JSON_SUMMARY:'):].strip())
    return None


def census(repo, quiet=False):
    """Grade every tracked board and RETURN the census document.

    Returning rather than writing is what lets the drift gate re-derive
    without touching the committed file.
    """
    check = os.path.join(repo, 'py_tools', 'check_floorplan.py')
    boards = tracked_boards(repo)

    def say(*a, **kw):
        if not quiet:
            print(*a, **kw)

    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for name in boards:
            board = os.path.join(repo, 'kicad_files', name)
            intent = os.path.join(tmp,
                                  name.replace('.kicad_pcb', '.intent.json'))
            doc_p = os.path.join(tmp, name.replace('.kicad_pcb', '.grade.json'))
            row = {'board': name}

            e = run([check, board, '--emit-intent', intent, '-q'], repo)
            row['emit_rc'] = e.returncode
            if not os.path.isfile(intent):
                row['emit_refused'] = (e.stdout + e.stderr).strip()[-300:]
                rows.append(row)
                say(f"  {name:42s} emit rc={e.returncode} NO INTENT",
                    flush=True)
                continue

            g = run([check, board, '--intent', intent, '--json', doc_p], repo)
            row['grade_rc'] = g.returncode
            s = summary_of(g.stdout)
            if s is None:
                row['grade_refused'] = (g.stdout + g.stderr).strip()[-300:]
                rows.append(row)
                say(f"  {name:42s} grade rc={g.returncode} NO SUMMARY",
                    flush=True)
                continue
            with open(doc_p, encoding='utf-8') as f:
                doc = json.load(f)

            buckets = {'withheld': [], 'not_asked': [], 'arm': []}
            for rule, reason in (doc.get('rules_skipped') or {}).items():
                buckets[classify(reason)].append(rule)

            row['pass'] = s.get('pass')
            row['errors'] = s.get('errors')
            row['rules_run'] = s.get('rules_run')
            row['budget_abstained'] = s.get('budget_abstained')
            row['budget_abstained_keys'] = s.get('budget_abstained_keys')
            row['edge_seating_abstained'] = s.get('edge_seating_abstained')
            row['skipped_not_asked'] = sorted(buckets['not_asked'])
            row['skipped_withheld'] = sorted(buckets['withheld'])
            row['skipped_arm'] = sorted(buckets['arm'])
            row['human_pass_line'] = any(
                'rule(s) ran, no violations' in ln
                for ln in g.stdout.splitlines())
            row['human_not_derivable'] = 'NOT DERIVABLE' in g.stdout

            # ONLY the declared-but-ungraded channels. `not_asked` is excluded
            # on purpose: it is the honest case and counting it makes every
            # board with a minimal intent permanently incomplete.
            ungraded = ((row['budget_abstained'] or 0)
                        + (row['edge_seating_abstained'] or 0)
                        + len(buckets['arm']))
            row['ungraded_declared'] = ungraded
            row['clean_but_ungraded'] = bool(row['pass']) and ungraded > 0
            rows.append(row)
            say(f"  {name:42s} rc={g.returncode} pass={row['pass']!s:5s} "
                f"err={row['errors']} run={row['rules_run']} "
                f"abst={row['budget_abstained']} "
                f"arm={len(buckets['arm'])} "
                f"wh={len(buckets['withheld'])} "
                f"na={len(buckets['not_asked'])} "
                f"{'<<< CLEAN BUT UNGRADED' if row['clean_but_ungraded'] else ''}",
                flush=True)

    graded = [r for r in rows if 'pass' in r]
    bad = [r for r in rows if r.get('clean_but_ungraded')]
    return {
        'what': 'boards where the grade reports pass:true while a channel the '
                'intent ASKED FOR went ungraded (rules_skipped "not asked" '
                'excluded -- see module docstring)',
        'boards_total': len(rows),
        'boards_graded': len(graded),
        'boards_clean_but_ungraded': len(bad),
        'names_clean_but_ungraded': [r['board'] for r in bad],
        'channels_seen': {
            'budget_abstained':
                sum(1 for r in graded if r.get('budget_abstained')),
            'rules_skipped_arm':
                sum(1 for r in graded if r.get('skipped_arm')),
            'rules_skipped_withheld':
                sum(1 for r in graded if r.get('skipped_withheld')),
            'edge_seating_abstained':
                sum(1 for r in graded if r.get('edge_seating_abstained')),
            'rules_skipped_not_asked_EXCLUDED':
                sum(1 for r in graded if r.get('skipped_not_asked')),
        },
        'rows': rows,
    }


def write(doc, path):
    """The ONE place the serialization lives, so the gate cannot come to
    disagree with the recorder about formatting."""
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1, sort_keys=True)


def main(argv=None):
    p = argparse.ArgumentParser(
        description='Census the boards whose grade reports a clean verdict '
                    'about something it never measured.')
    p.add_argument('repo', nargs='?', default=os.path.dirname(HERE),
                   help='repo root (default: the parent of tests/)')
    p.add_argument('--out', default=DEFAULT_OUT,
                   help='where to write the census (default: the committed '
                        'baseline). The drift gate points this at a temp dir, '
                        'so re-deriving cannot overwrite the very file it is '
                        'comparing against')
    p.add_argument('-q', '--quiet', action='store_true',
                   help='suppress the per-board progress lines')
    a = p.parse_args(argv)

    doc = census(a.repo, quiet=a.quiet)
    if not a.quiet:
        print('\nCENSUS: ' + json.dumps(
            {k: v for k, v in doc.items() if k != 'rows'}, indent=1))
    write(doc, a.out)
    print(f"wrote {a.out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
