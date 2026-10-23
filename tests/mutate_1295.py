"""The #1295 mutation battery: KiCad's "keep pairs apart" rule and inDiffPair().

One row per load-bearing line, each reverting it; every row names the check
of tests/test_1295_diff_pair_rule_forms.py that must FAIL. **A row is KILLED
only when that named check prints FAIL** -- a mutant that crashes the test
(an ImportError, a traceback) before reaching it is reported BROKEN, never
KILLED, because a test that dies tests nothing (CLAUDE.md, "assert the
REASON").

NOT COVERED, said here rather than left to be assumed: the announcement
wording in kicad_dru / check_drc, `coupled_net_pairs`' P-side filter (a pair
listed twice would map to the same frozenset), and the `pair_gap_rule_floor`
per-layer loop for a layered rule without the keep-apart term (no fixture
carries one).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs the witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1295.py
    python3 tests/mutate_1295.py --row partner-not-exempt

An anchor that does not match EXACTLY ONCE is BROKEN, never skipped;
`preflight()` runs right after `ROWS`. Edits are `str.replace(old, new, 1)`;
anchors are LF and translated to the target's own ending.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
_PR = os.path.join(_ROOT, 'py_router')

TARGETS = {
    'kd': os.path.join(_PR, 'kicad_dru.py'),
    'dr': os.path.join(_PR, 'design_rules.py'),
    'rc': os.path.join(_PR, 'routing_config.py'),
    'cd': os.path.join(_PR, 'check_drc.py'),
    'rd': os.path.join(_PR, 'route_diff.py'),
    'lg': os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py'),
}

T1295 = os.path.join(_TESTS, 'test_1295_diff_pair_rule_forms.py')

# (name, target, old, new, tests, expect, the check that must FAIL)
ROWS = [
    # -- the keep-apart rule in the track channel --
    ('keep-apart-term-not-parsed', 'kd',
     '    expr = _NOT_COUPLED.sub("", expr)\n',
     '    pass\n',
     (T1295,), 'KILLED', 'the track channel reads the keep-apart rule'),
    ('partner-not-exempt', 'kd',
     '        if r.except_coupled and coupled:\n',
     '        if False:\n',
     (T1295,), 'KILLED', 'its own coupled partner is exempt'),
    ('layer-clause-ignored', 'kd',
     '        if r.layer_clause is not None and not track_rule_on_layer(r, layer):\n',
     '        if False:\n',
     (T1295,), 'KILLED', 'a layer clause binds on its layers only'),
    ('nets-never-coupled', 'kd',
     '    return bool(rv) and comp == name_b\n',
     '    return False\n',
     (T1295,), 'KILLED', 'nets_coupled: KiCad pairs'),
    ('partner-map-not-installed', 'kd',
     '    if any(r.except_coupled for r in rules):\n        config.track_partner_clearances',
     '    if False:\n        config.track_partner_clearances',
     (T1295,), 'KILLED', 'the partner map holds both pairs'),
    ('layer-clause-keeps-a-rule-out', 'kd',
     '        copper_clause = (not has_layer_clause or (\n',
     '        copper_clause = (not has_layer_clause or False and (\n',
     (T1295,), 'KILLED', 'and its (layer inner) clause'),
    # -- the pair-exact readers --
    ('pair-clearance-ignores-partner-map', 'rc',
     '            if tp is not None:\n',
     '            if False:\n',
     (T1295,), 'KILLED', 'pair_clearance: P<->N stays at the base'),
    ('check-drc-binds-the-partner', 'cd',
     '        coupled = (_track_names is not None\n',
     '        coupled = False and (_track_names is not None\n',
     (T1295,), 'KILLED', 'check_drc grades it clean'),
    ('gap-floor-binds-the-partner', 'rd',
     "            coupled=nets_coupled(getattr(_pn, 'name', None),\n",
     "            coupled=False and nets_coupled(getattr(_pn, 'name', None),\n",
     (T1295,), 'KILLED', 'both pairs couple at 0.15'),
    ('placement-binds-the-partner', 'lg',
     '            coupled=frozenset((net_a, net_b)) in self.track_partners)\n',
     '            coupled=False)\n',
     (T1295,), 'KILLED', 'placement (the fanout connector gate)'),
    # -- the resolver --
    ('negation-unsupported', 'dr',
     "    if node == ('not', ('coupled',)):\n",
     '    if False:\n',
     (T1295,), 'KILLED', 'the keep-apart rule: supported'),
    ('keep-apart-read-as-coupled', 'dr',
     '        return False, True\n',
     '        return True, False\n',
     (T1295,), 'KILLED', 'the keep-apart rule: supported'),
    ('indiffpair-unsupported', 'dr',
     "                    'memberoffootprint', 'memberofgroup', 'indiffpair'}\n",
     "                    'memberoffootprint', 'memberofgroup'}\n",
     (T1295,), 'KILLED', 'inDiffPair(): supported'),
    ('indiffpair-ignores-the-complement', 'dr',
     '    if not rv or comp not in nets:\n',
     '    if not rv:\n',
     (T1295,), 'KILLED', "inDiffPair() is KiCad's inDiffPairFunc"),
    ('indiffpair-no-base-before-underscore', 'dr',
     "    return base.endswith('_') and _wx_matches(pattern, base[:base.rfind('_')])\n",
     '    return False\n',
     (T1295,), 'KILLED', "inDiffPair() is KiCad's inDiffPairFunc"),
    ('indiffpair-fnmatch-classes', 'dr',
     '    return re.fullmatch(rx, text, re.DOTALL) is not None\n',
     '    return fnmatch.fnmatchcase(text, pattern)\n',
     (T1295,), 'KILLED', "inDiffPair() is KiCad's inDiffPairFunc"),
    ('indiffpair-blind-to-board-nets', 'dr',
     '        return bool(_call(node, a, b, layer, ctx))\n',
     '        return bool(_call(node, a, b, layer))\n',
     (T1295,), 'KILLED', "inDiffPair() is KiCad's inDiffPairFunc"),
]

sys.path.insert(0, _TESTS)
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _run(test):
    p = subprocess.run([sys.executable, '-X', 'utf8', test],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=2400, cwd=_ROOT)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def _verdict(rc, out, reason):
    """KILLED only when the named check printed FAIL; a non-zero exit with no
    such line (a crash, an import error) is BROKEN."""
    if rc == 0:
        return 'SURVIVED', ''
    fails = [ln.strip() for ln in out.splitlines() if ln.strip().startswith('FAIL ')]
    hit = [ln for ln in fails if reason in ln]
    if hit:
        return 'KILLED', hit[0][:120]
    tail = [ln.strip() for ln in out.splitlines() if ln.strip()][-2:]
    return 'BROKEN', 'exit %d without FAIL %r: %s' % (rc, reason, (fails or tail)[:2])


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit first -- this '
                  'battery restores by overwriting.' % os.path.basename(path))
            return 2
    # THE UNMUTATED BASELINE: the witness must pass as the code stands, or a
    # row it "kills" proves nothing.
    rc, out = _run(T1295)
    if rc != 0:
        print('REFUSING: the witness fails UNMUTATED -- %s' % out[-400:])
        return 2
    print('baseline: the witness passes unmutated')
    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect, reason in rows:
            path = TARGETS[tgt]
            base = orig[tgt]
            o, n = old, new
            if '\r\n' in base:
                o, n = o.replace('\n', '\r\n'), n.replace('\n', '\r\n')
            if base.count(o) != 1 or o == n:
                results.append((name, 'BROKEN', expect,
                                'anchor matched %d times' % base.count(o)))
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(o, n, 1))
            try:
                got, why = 'SURVIVED', ''
                for t in tests:
                    got, why = _verdict(*_run(t), reason)
                    if got != 'SURVIVED':
                        break
            finally:
                io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, got, expect, why))
            print('%-40s %s' % (name, got), flush=True)
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
    wrong = [r for r in results if r[1] != r[2]]
    print('')
    for name, verdict, expect, why in results:
        print('%-40s %-9s%s' % (name, verdict, '' if verdict == expect else
                                '   <-- WRONG, expected %s' % expect))
        if why:
            print('      %s' % why)
    print('\n%d rows: %d killed, %d survived, %d broken'
          % (len(results), sum(r[1] == 'KILLED' for r in results),
             sum(r[1] == 'SURVIVED' for r in results),
             sum(r[1] == 'BROKEN' for r in results)))
    return 1 if wrong else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--row', default=None, help='run only this row')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
