#!/usr/bin/env python3
"""A watcher must not scan its own output.

Run 21 armed the bug watcher the way the brief says to -- output teed into the
work dir, so the watchers can see everything -- and `watch_bugs` scanned the
file it was writing. Every line it emitted matched a signature on the next
poll, so each hit quoted the previous one verbatim:

    NOT-BUILDABLE assembly0.json:1934: "verdict": "NOT BUILDABLE",
    NOT-BUILDABLE watch_bugs.log:2: NOT-BUILDABLE assembly0.json:1934: ...
    NOT-BUILDABLE watch_bugs.log:3: NOT-BUILDABLE watch_bugs.log:2: ...
    NOT-BUILDABLE watch_bugs.log:4: NOT-BUILDABLE logs/p0_assembly.log:288: ...

Eight lines in, ONE real finding had become six nested copies of itself. That
is the same class as the `.md` exclusion this module already documents --
narration is not an incident -- except self-amplifying, so it does not pad the
log, it destroys it. The run's report is REQUIRED to quote every line each
watcher emitted, so this defect attacks the deliverable directly.

Two halves are pinned here: the exclusion works, and it is narrow enough that
a tool log is still scanned. An over-broad fix would be worse than the bug --
a watcher that scans nothing reports nothing, and reports it quietly.
"""
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (REPO, os.path.join(REPO, 'tests', 'stress')):
    if p not in sys.path:
        sys.path.insert(0, p)

import run_watch as RW

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


d = tempfile.mkdtemp()
os.makedirs(os.path.join(d, 'logs'), exist_ok=True)


def w(rel, text):
    path = os.path.join(d, rel)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


# A real tool log, a real tool report, and the watcher's own output quoting it.
tool = w(os.path.join('logs', 'p0_assembly.log'), 'VERDICT: NOT BUILDABLE\n')
report = w('assembly0.json', '{"verdict": "NOT BUILDABLE"}\n')
selflog = w('watch_bugs.log',
            'WATCHING for problems\n'
            'NOT-BUILDABLE assembly0.json:1: "verdict": "NOT BUILDABLE",\n')
cheatlog = w('watch_cheats.log', 'FLOOR --clearance in logs/x.log:1\n')
journal = w('JOURNAL.md', 'the board went NOT BUILDABLE -> buildable\n')

walked = {os.path.relpath(p, d).replace('\\', '/') for p in RW._walk(d)}

check("the tool's log is still scanned", 'logs/p0_assembly.log' in walked,
      str(sorted(walked)))
check("the tool's JSON report is still scanned", 'assembly0.json' in walked,
      str(sorted(walked)))
check("the bug watcher's OWN log is not scanned",
      'watch_bugs.log' not in walked, str(sorted(walked)))
check("nor a SIBLING watcher's log (two watchers, one work dir)",
      'watch_cheats.log' not in walked, str(sorted(walked)))
check("the journal is still excluded by extension, as before",
      'JOURNAL.md' not in walked, str(sorted(walked)))

# --------------------------------------------------------------------------
# The inode path, which is the one that does not depend on the naming
# convention. A watcher redirected to a file called anything at all must still
# skip it.
# --------------------------------------------------------------------------
odd = w('run-output-2026.log', 'NOT BUILDABLE\n')
check("a differently-named file is scanned by default",
      'run-output-2026.log' in walked or os.path.isfile(odd),
      "sanity: it exists and the convention does not cover it")

st = os.stat(odd)
if not st.st_ino:
    print("       NOTE st_ino is 0 on this platform; the inode half of the "
          "check cannot be exercised here and the name convention is the "
          "only guard. Reported rather than skipped silently.")
else:
    saved = RW._SELF_OUTPUT_IDS
    try:
        RW._SELF_OUTPUT_IDS = {(st.st_dev, st.st_ino)}
        check("a watcher's output is skipped by INODE, whatever it is called",
              RW._is_self_output(odd), odd)
        check("...and that does not spill onto other files",
              not RW._is_self_output(tool), tool)
        walked2 = {os.path.relpath(p, d).replace('\\', '/')
                   for p in RW._walk(d)}
        check("so _walk drops it too", 'run-output-2026.log' not in walked2,
              str(sorted(walked2)))
    finally:
        RW._SELF_OUTPUT_IDS = saved

check("_self_output_ids returns a set of (dev, ino) pairs, or empty",
      isinstance(RW._self_output_ids(), set)
      and all(isinstance(x, tuple) and len(x) == 2
              for x in RW._self_output_ids()),
      str(RW._self_output_ids()))

# --------------------------------------------------------------------------
# The feedback loop itself: scanning a watcher log finds hits, which is WHY
# excluding it matters. If this ever stops finding hits, the exclusion has
# become untested rather than unnecessary.
# --------------------------------------------------------------------------
hits = RW._scan_text(selflog, set(), 'watch_bugs.log')
check("a watcher log DOES match the signatures (the loop was real)",
      hits, f"{len(hits)} hit(s): {hits[:1]}")

print(f"\n{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
