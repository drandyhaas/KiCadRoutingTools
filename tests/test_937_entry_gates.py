#!/usr/bin/env python3
"""#937: the routing door's entry gate exists, and P2/P3 read the verdict.

Two acceptance criteria that nothing else pins.

FIRST, the citations. Eight sites sent a reader to a routing "Step 0", a "How
to run this skill" section or a "V1-V5 loop", and none of the three existed in
`plan-pcb-routing/SKILL.md`. Five are now true by construction and three were
re-pointed. A citation is only worth writing if something checks it still
resolves -- otherwise the next edit silently re-creates the defect this PR
exists to remove.

SECOND, `_guard_damage`. It gated P2/P3 on `blocking`, which since #918 is ONE
of check_assembly's five not_buildable conjuncts -- so a board unbuildable
through a coincident-origin stack or a containment reads `blocking` 0, and the
guard told the reader there was "no damage for this stage to repair" about a
board its own instrument had graded NOT BUILDABLE. Checked behaviourally, by
running the stage, not by reading the source: a source grep passes on a
comment.
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILLS = os.path.join(ROOT, '.claude', 'skills')
ROUTING = os.path.join(SKILLS, 'plan-pcb-routing', 'SKILL.md')
LOOP = os.path.join(SKILLS, 'plan-pcb-placement-and-routing', 'scripts',
                    'loop_driver.py')
COMBINED = os.path.join(SKILLS, 'plan-pcb-placement-and-routing', 'SKILL.md')
DRIVER = os.path.join(SKILLS, 'plan-pcb-placement', 'scripts',
                      'placement_driver.py')

RUN_ALL_TIMEOUT = 180


def _read(path):
    with open(path, encoding='utf-8', errors='replace') as fh:
        return fh.read()


def t_the_routing_skill_has_the_sections_eight_sites_cite():
    rt = _read(ROUTING)
    for needed in ('## How to run this skill', '## Step 0'):
        assert needed in rt, (
            f'plan-pcb-routing/SKILL.md has no {needed!r}. Six sites in '
            f'loop_driver.py and the combined skill send a reader there.')
    # Step 0 must be the PLACEMENT gate the citers describe ("its Step 0
    # placement gate will pass, because you just did that work"), not merely a
    # heading with the right number.
    step0 = rt.split('## Step 0', 1)[1].split('\n## ', 1)[0]
    for instrument in ('check_assembly.py', 'board_score.py'):
        assert instrument in step0, (
            f'routing Step 0 does not run {instrument}, so the loop\'s claim '
            f'that it "will pass, you just did that work" is not true by '
            f'construction -- L2 runs exactly these.')
    print('  PASS: both sections exist, and Step 0 is the placement gate')


def t_no_site_still_cites_a_routing_V1_V5_loop():
    """The three genuinely stale citations. The convergence loop those stages
    came from lives in the combined skill's references/, and `converge.py`
    already records that no routing_driver ever reached main."""
    live = []
    for path in (LOOP, COMBINED):
        for n, line in enumerate(_read(path).splitlines(), 1):
            if 'V1-V5' not in line and 'V1–V5' not in line:
                continue
            # A line EXPLAINING the retraction is not a citation to it.
            if any(w in line for w in ('said', 'used to', 'never reached',
                                       'removed', '#937')):
                continue
            live.append(f'{os.path.basename(path)}:{n}: {line.strip()[:90]}')
    assert not live, (
        'these still send a reader to a routing V1-V5 loop that does not '
        'exist:\n  ' + '\n  '.join(live))
    print('  PASS: no live citation to a routing V1-V5 loop remains')


def _p2(tmp, drc, asm=None):
    """Run P2 with the given evidence; return (rc, text)."""
    import json
    board = os.path.join(tmp, 'b.kicad_pcb')
    open(board, 'w').write('(kicad_pcb)')
    argv = [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P2',
            '--board', board]
    for flag, doc, name in (('--drc-json', drc, 'd.json'),
                            ('--assembly-json', asm, 'a.json')):
        if doc is None:
            continue
        p = os.path.join(tmp, name)
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh)
        argv += [flag, p]
    r = subprocess.run(argv, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', cwd=ROOT,
                       timeout=120)
    return r.returncode, r.stdout


def t_a_board_that_is_not_buildable_at_blocking_zero_reaches_the_repair():
    """The defect, behaviourally. `blocking` 0 with `buildable` false is a
    real tracked board (rp2350_fpga_eensy_prePlane: 0 blocking pairs, NOT
    BUILDABLE through a coincident-origin stack)."""
    with tempfile.TemporaryDirectory() as tmp:
        rc, out = _p2(tmp, {'violations': 0},
                      {'blocking': 0, 'buildable': False,
                       'verdict': 'NOT BUILDABLE'})
    assert rc != 4, (
        'P2 refused a board its own instrument graded NOT BUILDABLE, because '
        'the guard read `blocking` -- 1 of 5 not_buildable conjuncts since '
        f'#918 -- instead of the verdict.\n{out[:600]}')
    assert 'no damage for this stage to repair' not in out, out[:400]
    print('  PASS: NOT BUILDABLE at blocking 0 reaches the repair stage')


def t_a_genuinely_clean_board_is_still_refused():
    """The control. Without it the case above passes on a guard that refuses
    nothing at all."""
    with tempfile.TemporaryDirectory() as tmp:
        rc, out = _p2(tmp, {'violations': 0},
                      {'blocking': 0, 'buildable': True,
                       'verdict': 'buildable (blocking 0)'})
    assert rc == 4 and 'no damage for this stage to repair' in out, (
        f'a clean board must still be refused by P2 (rc={rc})\n{out[:600]}')
    print('  PASS: a buildable, violation-free board is still refused')


def t_p0_reads_the_documents_it_makes_you_produce():
    """P0 had both flags and opened neither."""
    import json
    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'b.kicad_pcb')
        open(board, 'w').write('(kicad_pcb)')
        d = os.path.join(tmp, 'd.json')
        a = os.path.join(tmp, 'a.json')
        json.dump({'violations': 7}, open(d, 'w'))
        json.dump({'blocking': 0, 'buildable': False,
                   'verdict': 'NOT BUILDABLE',
                   'oob_pad_copper_count': 2}, open(a, 'w'))
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P0',
             '--board', board, '--drc-json', d, '--assembly-json', a],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=120)
    for needed in ('7 violation(s)', 'NOT BUILDABLE',
                   'PAD COPPER OFF THE OUTLINE on 2'):
        assert needed in r.stdout, (
            f'P0 did not report {needed!r} from the documents it was given\n'
            f'{r.stdout[:800]}')
    assert 'THE TWO DISAGREE' not in r.stdout, (
        'both instruments read dirty here; there is no disagreement to flag')
    print('  PASS: P0 reports both instruments and the off-outline count')


def t_p0_without_evidence_still_just_asks_for_it():
    """First entry: nothing to read yet, and the stage must not pretend."""
    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'b.kicad_pcb')
        open(board, 'w').write('(kicad_pcb)')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P0',
             '--board', board],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=120)
    assert r.returncode == 0, r.stdout[:400]
    assert 'WHAT THE INSTRUMENTS SAY' not in r.stdout, (
        'P0 printed a reading with no documents to read')
    assert 'check_assembly.py' in r.stdout, 'P0 must still ask for them'
    print('  PASS: with no evidence, P0 asks for it and reports nothing')


TESTS = (t_the_routing_skill_has_the_sections_eight_sites_cite,
         t_no_site_still_cites_a_routing_V1_V5_loop,
         t_a_board_that_is_not_buildable_at_blocking_zero_reaches_the_repair,
         t_a_genuinely_clean_board_is_still_refused,
         t_p0_reads_the_documents_it_makes_you_produce,
         t_p0_without_evidence_still_just_asks_for_it)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('t_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(f'--- {fn.__name__}')
        fn()
    print('\nALL PASS')
