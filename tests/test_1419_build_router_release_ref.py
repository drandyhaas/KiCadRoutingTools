#!/usr/bin/env python3
"""#1419: build_router judges "my rust_router/ matches the release source"
against the remote that IS the release repository, not blindly origin/main
(which in a fork clone is the fork, stale or ahead of upstream).

Builds a throwaway git repo whose origin/main is a STALE fork and whose
`upstream` remote (URL = the release repo) holds the current crate; the working
tree equals upstream, so the answer must be True. A genuinely edited crate must
still read False, and a repo with no release remote keeps the origin/main
comparison.

    python3 tests/test_1419_build_router_release_ref.py
"""
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import build_router as b                                         # noqa: E402

failures = []


def ok(label, cond):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}")
    if not cond:
        failures.append(label)


def git(d, *a):
    subprocess.run(['git', '-C', d, '-c', 'user.email=t@t', '-c',
                    'user.name=t'] + list(a), check=True,
                   capture_output=True, text=True)


def commit(d, text):
    os.makedirs(os.path.join(d, 'rust_router'), exist_ok=True)
    with open(os.path.join(d, 'rust_router', 'lib.rs'), 'w') as f:
        f.write(text)
    git(d, 'add', '-A')
    git(d, 'commit', '-q', '-m', text)


# 1. the pure ordering
c = b._release_ref_candidates({
    'origin': 'https://github.com/someone/KiCadRoutingTools.git',
    'upstream': 'git@github.com:drandyhaas/KiCadRoutingTools.git'})
ok('release remote first',
   c[0] == 'upstream/main' and c[1:] == ['origin/main', 'main'])
c = b._release_ref_candidates(
    {'origin': 'https://github.com/drandyhaas/KiCadRoutingTools'})
ok('origin that is the release repo is not duplicated',
   c == ['origin/main', 'main'])
c = b._release_ref_candidates(
    {'origin': 'https://github.com/drandyhaas/KiCadRoutingToolsFork.git'})
ok('a look-alike repo name is not the release repo', c == ['origin/main', 'main'])

# 2. end to end on a throwaway repo
tmp = tempfile.mkdtemp(prefix='t1419_')
try:
    d = os.path.join(tmp, 'clone')
    os.makedirs(d)
    git(d, 'init', '-q', '-b', 'dev')
    commit(d, 'stale fork crate')
    git(d, 'update-ref', 'refs/remotes/origin/main', 'HEAD')
    git(d, 'remote', 'add', 'origin',
        'https://github.com/fork/KiCadRoutingTools.git')
    commit(d, 'current crate')
    git(d, 'remote', 'add', 'upstream',
        'https://github.com/drandyhaas/KiCadRoutingTools.git')
    git(d, 'update-ref', 'refs/remotes/upstream/main', 'HEAD')
    ok('tree equal to the release remote reads True (stale fork origin ignored)',
       b._crate_matches_release_source(d) is True)
    with open(os.path.join(d, 'rust_router', 'lib.rs'), 'a') as f:
        f.write('edit')
    ok('an edited crate still reads False',
       b._crate_matches_release_source(d) is False)
    git(d, 'checkout', '-q', '--', 'rust_router')
    git(d, 'remote', 'remove', 'upstream')
    ok('no release remote: origin/main (the stale fork) is the reference',
       b._crate_matches_release_source(d) is False)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

if failures:
    print('FAILED:', failures)
    sys.exit(1)
print('PASS')
