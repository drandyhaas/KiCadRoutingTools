#!/usr/bin/env python3
"""A `.kicad_dru` courtyard_clearance rule of min 0 or below is not named.

check_drc names every error-severity rule of a kind it has no pass for
`NOT GRADED (<kind>)`, which board_score lists and check_complete reads as
INCOMPLETE. A courtyard_clearance rule whose min is 0 or below (absent reads
as 0) -- a fab template's test-point courtyard exclusion, `(min -1mm)` --
can only relax KiCad's courtyard check: kicad-cli 10.0.6 reports the default
courtyards_overlap rows under min 0 and none for a pair a min < 0 rule binds,
even where the courtyards overlap by more than |min|. check_drc grades no
courtyard either way, so naming such a rule blocked a verdict for nothing.
A rule of min > 0 adds rows and stays named.

1. No KiCad: `kicad_dru.read_board_ungraded_kind_rules` leaves the relaxing
   rules out and keeps the min > 0 one; check_drc names only that one, and
   check_complete names no rule nothing grades on a board carrying only
   relaxing ones.
2. kicad-cli's report of a probe board (needs kicad-cli; exit 77 without it,
   after part 1): two-part pairs whose 2 x 2 mm courtyards stand -1.5 to
   +0.8 mm apart, under no rule and each rule below. Every error row a
   relaxing rule leaves is one the board reports with no rule; the min > 0
   control adds rows, so the probe can see one.

    python3 tests/test_dru_courtyard_relaxing_rule.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join('kicad_files', 'cap_chain.kicad_pcb')
FAILS = []

#: name: (constraint, condition or None, relaxes only)
RULES = {
    'tp exclusion': ('(min -1mm)', "A.Reference == 'TP*'", True),
    'zero': ('(min 0mm)', None, True),
    'max only': ('(max 1mm)', None, True),
    'unread relax': ('(min -1mm)', "A.getField('x') == 'y'", True),
    'tight': ('(min 0.5mm)', None, False),
}
#: gap between a pair's courtyards (negative: they overlap)
GAPS = (-1.5, -0.5, -0.05, 0.0, 0.2, 0.4, 0.8)
PAIRS = [(p, g) for p in ('TP', 'R') for g in GAPS]


def ok(cond, msg, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}{(': ' + str(detail)) if detail and not cond else ''}")
    if not cond:
        FAILS.append(msg)


def rule_text(name):
    cons, cond, _relax = RULES[name]
    return (f'(rule "{name}" (constraint courtyard_clearance {cons})'
            + (f' (condition "{cond}")' if cond else '') + ')\n')


def run(argv):
    r = subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO,
                       encoding='utf-8', errors='replace', timeout=900)
    acc = run_utils._accident((r.stdout or '') + (r.stderr or ''))
    if acc:
        raise AssertionError(f"BROKEN TEST: {argv} died from {acc!r}\n{r.stdout[-800:]}"
                             f"{r.stderr[-800:]}")
    return r


def stage(td, sub, names):
    d = os.path.join(td, sub)
    os.makedirs(d)
    b = os.path.join(d, 'b.kicad_pcb')
    run([os.path.join('py_router', 'copy_board.py'), BOARD, b])
    run_utils.evidence(b)
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + ''.join(rule_text(n) for n in names))
    return b


def tool(td):
    print('1. the relaxing rules are not named; the min > 0 one is')
    from kicad_dru import read_board_ungraded_kind_rules
    b = stage(td, 'all', list(RULES))
    got = read_board_ungraded_kind_rules(b)
    ok(got == [('tight', 'courtyard_clearance')],
       'read_board_ungraded_kind_rules keeps only the min > 0 rule', got)
    r = run([os.path.join('py_router', 'check_drc.py'), b])
    named = sorted(ln.split("'")[1] for ln in r.stdout.splitlines()
                   if 'NOT GRADED (courtyard_clearance)' in ln)
    ok(named == ['tight'], 'check_drc names NOT GRADED (courtyard_clearance) for it alone',
       f'{named}\n{r.stdout[-600:]}')

    relax = [n for n, (_c, _w, rel) in RULES.items() if rel]
    b = stage(td, 'relax', relax)
    r = run([os.path.join('check_complete.py'), b, '--skip-slow'])
    verdict = next((x for x in r.stdout.splitlines() if x.startswith('VERDICT')), '')
    ok(verdict and 'nothing grades' not in r.stdout,
       'check_complete names no rule nothing grades on a board with only relaxing rules',
       verdict[:400] or r.stdout[-600:])
    b = stage(td, 'tight', ['tight'])
    run_utils.check(PY + [os.path.join(REPO, 'check_complete.py'), b, '--skip-slow'],
                    refuse="nothing grades: 'tight' (courtyard_clearance)", code=4)
    ok(True, 'check_complete: INCOMPLETE naming the min > 0 rule')


def probe_board():
    out = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
           ' (general (thickness 1.6))\n (paper "A4")\n'
           ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (13 "F.Mask" user)'
           ' (25 "Edge.Cuts" user) (31 "F.CrtYd" user "F.Courtyard"))\n'
           ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n')
    for i, (p, g) in enumerate(PAIRS):
        y, x = 105 + 6 * (i % len(GAPS)), 105 + 20 * (i // len(GAPS))
        for ref, px in ((f'{p}{i + 1}', x), (f'U{i + 1}', x + 2 + g)):
            out += (f'  (footprint "T:{ref}" (layer "F.Cu") (at {px} {y})\n'
                    f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") (hide yes) '
                    f'(effects (font (size 1 1) (thickness 0.15))))\n    (attr smd)\n'
                    f'    (fp_rect (start -1 -1) (end 1 1) (stroke (width 0.05) (type solid)) '
                    f'(fill no) (layer "F.CrtYd"))\n'
                    f'    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu" "F.Mask"))\n  )\n')
    for x1, y1, x2, y2 in ((100, 100, 150, 100), (150, 100, 150, 150),
                           (150, 150, 100, 150), (100, 150, 100, 100)):
        out += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
                f'(stroke (width 0.05) (type solid)))\n')
    return out + ')\n'


def kicad_rows(td, sub, names, cli):
    """kicad-cli's error rows naming a footprint: {(sorted refs, type)}."""
    d = os.path.join(td, sub)
    os.makedirs(d)
    pcb = os.path.join(d, 'p.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as fh:
        fh.write(probe_board())
    if names:
        with open(os.path.join(d, 'p.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n' + ''.join(rule_text(n) for n in names))
    rep = pcb + '.kicad.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o', rep, pcb],
                   capture_output=True, text=True, timeout=600)
    with open(run_utils.evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
        doc = json.load(fh)
    rows = set()
    for v in doc.get('violations', []):
        refs = tuple(sorted(it['description'].split()[1] for it in v['items']
                            if it['description'].startswith('Footprint ')))
        if v['severity'] == 'error' and len(refs) == 2:
            rows.add((refs, v['type']))
    return rows, doc.get('kicad_version')


def kicad(td, cli):
    print("2. kicad-cli's courtyard rows on the probe board")
    base, ver = kicad_rows(td, 'k_none', [], cli)
    overlap = {tuple(sorted((f'{p}{i + 1}', f'U{i + 1}'))) for i, (p, g) in enumerate(PAIRS)
               if g < 0}
    ok({refs for refs, _t in base} == overlap
       and {t for _r, t in base} == {'courtyards_overlap'},
       f'kicad-cli {ver}, no rule: courtyards_overlap on each overlapping pair', sorted(base))
    for name, (_c, _w, relax) in RULES.items():
        if name == 'unread relax':
            continue                  # KiCad reads getField; its condition binds nothing here
        rows, _v = kicad_rows(td, 'k_' + name.replace(' ', '_'), [name], cli)
        if relax:
            ok(rows <= base, f"'{name}': every error row is one the board has with no rule",
               sorted(rows - base))
        else:
            ok(rows - base, f"'{name}' (control): rows the board does not have with no rule")
    rows, _v = kicad_rows(td, 'k_tp', ['tp exclusion'], cli)
    ok(not any(r[0].startswith('TP') for r, _t in rows),
       "'tp exclusion': no row on a test-point pair, overlapping by up to 1.5 mm", sorted(rows))


def main():
    td = tempfile.mkdtemp(prefix='krt_court_rule_')
    skipped = False
    try:
        tool(td)
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            kicad(td, cli)
        else:
            print('2. SKIPPED: no kicad-cli (set KICAD_CLI)')
            skipped = True
    except AssertionError as exc:
        ok(False, f'a run_utils assertion held: {str(exc)[:1500]}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'\nFAILED: {len(FAILS)}')
        return 1
    if skipped:
        print('\nPART 1 PASS')
        print('SKIP: part 2 needs kicad-cli (none found; set KICAD_CLI) -- '
              "KiCad's own courtyard rows were not checked")
        return 77
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
