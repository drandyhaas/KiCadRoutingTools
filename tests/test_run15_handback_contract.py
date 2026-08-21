#!/usr/bin/env python3
"""What a delegated half is ASKED for must satisfy the next stage's gate.

This is the coverage that did not exist, and its absence is why the gap
survived: `loop_driver`'s L1 prompt asked the placement teammate to return "the
four close-out measurements with their numbers", while the very next stage, L2,
refused to open without a `check_assembly.py --json` document that the prompt
never mentioned. Delegation was tested only as text emission -- "does the word
TEAMMATE appear" -- so nothing connected the two halves of that sentence.

Both directions are asserted here, for every artifact in the contract:

  * the prompt NAMES the path (so the teammate knows where to put it), and
  * the consuming stage ACCEPTS a document at that path, and REFUSES without
    it (so the path is load-bearing rather than decorative).

The paths are named by the PARENT, not chosen by the teammate, which is what
makes this testable at all: the test can predict them.
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
DRIVER = os.path.join(ROOT, '.claude', 'skills',
                      'plan-pcb-placement-and-routing', 'scripts',
                      'loop_driver.py')


def run(args):
    r = subprocess.run([sys.executable, '-X', 'utf8', DRIVER] + args,
                       capture_output=True, text=True, timeout=600, cwd=ROOT)
    return r.returncode, r.stdout + r.stderr


def sha_of(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


class HandbackContractTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='handback_')
        self.work = os.path.join(self.tmp, 'wk')
        os.makedirs(self.work)
        self.ledger = os.path.join(self.work, 'ledger.jsonl')
        self.board = os.path.join(self.work, 'in.kicad_pcb')
        open(self.board, 'w', encoding='utf-8').close()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _ledger_with(self, *boards):
        with open(self.ledger, 'w', encoding='utf-8') as f:
            for i, b in enumerate(boards):
                f.write(json.dumps({'iteration': i, 'kind': 'placement',
                                    'accepted': True,
                                    'result_sha': sha_of(b)}) + '\n')

    # ------------------------------------------------------------------
    # L1 -> L2
    # ------------------------------------------------------------------
    def test_l1_names_every_path_l2_consumes(self):
        code, out = run(['--stage', 'L1', '--board', self.board,
                         '--ledger', self.ledger])
        self.assertEqual(code, 0, out[:400])
        w = self.work.replace('\\', '/')
        for want in (w + '/placed.kicad_pcb',
                     w + '/assembly_close.json',
                     w + '/place_close_render.json'):
            self.assertIn(want, out,
                          'the placement prompt must NAME this path: ' + want)
        self.assertIn('check_assembly.py', out)
        self.assertIn('--kind placement', out,
                      'the teammate must be told which kind to record')

    def test_what_l1_asks_for_opens_l2(self):
        """Follow the prompt, and the next gate opens -- PLUS the one thing
        L1 deliberately does NOT ask for.

        run-23: L2 also demands wk/review_handoff.md, the orchestrator's OWN
        blind-first look at the board between the halves. It is not in L1's
        teammate prompt ON PURPOSE -- a review by the agent that just placed
        the board is not independent eyes -- so this test now pins BOTH
        halves of the contract: without the review L2 refuses AND names the
        protocol; with everything L1 asked for plus the orchestrator's
        review, L2 opens.
        """
        placed = os.path.join(self.work, 'placed.kicad_pcb')
        shutil.copyfile(self.board, placed)
        rep = os.path.join(self.work, 'assembly_close.json')
        json.dump({'board': placed, 'blocking': 0, 'oob_pad_count': 0,
                   'locked_contacts': 0, 'buildable': True,
                   'verdict': 'buildable (blocking 0)'},
                  open(rep, 'w', encoding='utf-8'))
        self._ledger_with(placed)
        code, out = run(['--stage', 'L2', '--board', placed,
                         '--ledger', self.ledger, '--placement-report', rep])
        self.assertEqual(code, 4, 'L2 must still demand the orchestrator\'s '
                         'review:\n' + out[:400])
        self.assertIn('BLIND-FIRST', out)
        self.assertIn('review_handoff.md', out)
        with open(os.path.join(self.work, 'review_handoff.md'), 'w',
                  encoding='utf-8') as f:
            f.write('# handoff review (fixture)\n\nOBSERVATIONS '
                    '(blind-first): fixture board, unit test, no panels; '
                    'nothing off-outline, no pockets, no interior '
                    'connectors observed.\n\nRECONCILIATION: nothing '
                    'observed, nothing to disposition.\n')
        code, out = run(['--stage', 'L2', '--board', placed,
                         '--ledger', self.ledger, '--placement-report', rep])
        self.assertEqual(code, 0, 'L2 must open on what L1 asked for plus '
                         'the orchestrator\'s review:\n' + out[:600])
        self.assertIn('DELEGATING:', out)

    def test_l2_refuses_without_the_document_l1_was_told_to_make(self):
        placed = os.path.join(self.work, 'placed.kicad_pcb')
        shutil.copyfile(self.board, placed)
        self._ledger_with(placed)
        code, out = run(['--stage', 'L2', '--board', placed,
                         '--ledger', self.ledger])
        self.assertEqual(code, 4)
        self.assertIn('check_assembly', out,
                      'and the refusal must name the command that makes it')

    def test_l2_refuses_a_board_no_lap_recorded(self):
        placed = os.path.join(self.work, 'placed.kicad_pcb')
        shutil.copyfile(self.board, placed)
        rep = os.path.join(self.work, 'assembly_close.json')
        json.dump({'board': placed, 'blocking': 0, 'oob_pad_count': 0,
                   'locked_contacts': 0, 'buildable': True,
                   'verdict': 'buildable (blocking 0)'},
                  open(rep, 'w', encoding='utf-8'))
        with open(self.ledger, 'w', encoding='utf-8') as f:      # wrong sha
            f.write(json.dumps({'iteration': 0, 'kind': 'placement',
                                'accepted': True, 'result_sha': '0' * 64}) + '\n')
        code, out = run(['--stage', 'L2', '--board', placed,
                         '--ledger', self.ledger, '--placement-report', rep])
        self.assertEqual(code, 4)
        self.assertIn('no lap records a board with this content hash', out)

    # ------------------------------------------------------------------
    # L2 -> L3 / L5
    # ------------------------------------------------------------------
    def _l2_prompt(self):
        placed = os.path.join(self.work, 'placed.kicad_pcb')
        shutil.copyfile(self.board, placed)
        rep = os.path.join(self.work, 'assembly_close.json')
        json.dump({'board': placed, 'blocking': 0, 'oob_pad_count': 0,
                   'locked_contacts': 0, 'buildable': True,
                   'verdict': 'buildable (blocking 0)'},
                  open(rep, 'w', encoding='utf-8'))
        # run-23: L2 refuses without the blind-first handoff review. A
        # fixture exercising the PROMPT must satisfy the eyes gate the same
        # way it satisfies the four numeric keys above.
        with open(os.path.join(self.work, 'review_handoff.md'), 'w',
                  encoding='utf-8') as f:
            f.write('# handoff review (fixture)\n\n'
                    'OBSERVATIONS (blind-first): fixture board; no panels in '
                    'a unit test; nothing off-outline, no pockets, no '
                    'interior connectors observed.\n\n'
                    'RECONCILIATION: nothing observed, nothing to '
                    'disposition; the numeric gates carry this fixture.\n')
        self._ledger_with(placed)
        code, out = run(['--stage', 'L2', '--board', placed,
                         '--ledger', self.ledger, '--placement-report', rep])
        self.assertEqual(code, 0, out[:400])
        return placed, out

    def test_l2_names_every_path_l3_and_l5_consume(self):
        placed, out = self._l2_prompt()
        w = self.work.replace('\\', '/')
        for want in (w + '/routed.kicad_pcb', w + '/score.json',
                     w + '/route.log', w + '/routing_close.json'):
            self.assertIn(want, out,
                          'the routing prompt must NAME this path: ' + want)

    def test_l2_hands_down_authored_from_because_only_it_knows(self):
        placed, out = self._l2_prompt()
        w = self.work.replace('\\', '/')
        # --authored-from is "the board you were HANDED", which is the FROZEN
        # board, not the placed one. L2 no longer stamps locks back into
        # placed.kicad_pcb: doing so changed its content hash while the
        # placement half was still running, which read that as corruption and
        # reverted the file. The fab floors ride across intact because the
        # freeze is a copy_board.py + (locked yes) stamp, so this still answers
        # the question the check exists for -- what did the writeback loosen
        # against -- while naming a path the teammate could not have known.
        self.assertIn('--authored-from ' + w + '/frozen.kicad_pcb', out,
                      'the teammate cannot know the board it was handed; '
                      'without it check_complete cannot run its fab-floor '
                      'check and UNSOUND becomes unreachable')
        self.assertNotIn('--authored-from ' + placed.replace('\\', '/'), out,
                         'the PLACED board is the placement half\'s artifact '
                         'and its ledger binding; the routing half is handed '
                         'the frozen copy and must grade against that')

    def test_l2_freezes_into_a_new_file_never_in_place(self):
        placed, out = self._l2_prompt()
        w = self.work.replace('\\', '/')
        self.assertIn(w + '/frozen.kicad_pcb', out,
                      'L2 must name the file it freezes INTO')
        self.assertIn('DO NOT FREEZE IN PLACE', out,
                      'freezing in place races a still-running placement half')
        self.assertIn(w + '/freeze_refs.json', out,
                      'the freeze list is DECLARED by the placement half, not '
                      'inferred from a pose diff')
        for want in (w + '/frozen.kicad_pcb --kind placement',):
            self.assertIn(want, out,
                          'the freeze is a lap and must be recorded: ' + want)

    def test_l2_states_who_owns_each_classification(self):
        _placed, out = self._l2_prompt()
        self.assertIn('THE CLASSIFICATION DECIDES WHO FIXES IT', out)
        for shape in ('parameter', 'placement', 'floorplan'):
            self.assertIn('`%s`' % shape, out)
        self.assertIn('ends your turn', out,
                      'a placement/floorplan verdict must stop the teammate, '
                      'not become a router retry')

    def test_l2_asks_for_the_lens_verdicts_the_ledger_refuses_without(self):
        _placed, out = self._l2_prompt()
        self.assertIn('VERDICT=', out)
        self.assertIn('connectivity', out)
        self.assertIn('drc', out)
        self.assertIn('spec', out)

    # ------------------------------------------------------------------
    # the fence, on both prompts
    # ------------------------------------------------------------------
    def test_both_prompts_name_the_carriers_a_teammate_must_not_open(self):
        _c, l1 = run(['--stage', 'L1', '--board', self.board,
                      '--ledger', self.ledger])
        _placed, l2 = self._l2_prompt()
        for name, out in (('L1', l1), ('L2', l2)):
            self.assertIn('_truth/', out, name + ' must name the truth dir')
            self.assertIn('perturb.json', out,
                          name + ' must name the pose record')
            self.assertIn('ONLY board you may open', out)

    def test_both_prompts_carry_the_hand_script_disclosure_duty(self):
        """Run 19: the teammate built arrange.py v1-v5 undisclosed until
        challenged. The fence named the carriers a teammate must not OPEN,
        but said nothing about scripts a teammate might WRITE -- so the
        disclosure duty now travels in the same clause, on both prompts."""
        _c, l1 = run(['--stage', 'L1', '--board', self.board,
                      '--ledger', self.ledger])
        _placed, l2 = self._l2_prompt()
        for name, out in (('L1', l1), ('L2', l2)):
            self.assertIn('disclosed hand-assist', out,
                          name + ' must carry the hand-script disclosure duty')

    def test_l2_tells_the_teammate_how_to_hand_back_a_long_step(self):
        """A subagent cannot block on a detached process, so it must not try.

        Both routing halves in run 20 returned "still working, I'll wait" --
        because a teammate has no way to sit on a backgrounded step and be
        woken when it exits. The orchestrator armed the wait itself and resumed
        them twice, and each round trip cost a re-read of the whole context.

        The prompt now names the shape: LOG / MARKER / NEXT, and says it is a
        correct hand-back rather than a failure -- an agent that thinks it has
        failed will keep trying to finish, which is the behaviour being fixed.
        """
        _placed, out = self._l2_prompt()
        for token in ('LOG=', 'MARKER=', 'NEXT='):
            self.assertIn(token, out,
                          f'the hand-back shape must name {token}')
        self.assertIn('--deadline', out,
                      'a hand-back whose step never terminates turns one '
                      'blocked agent into two')
        self.assertIn('correct hand-back, not a', out,
                      'a half that hands back correctly must not report it as '
                      'a failure, or the parent reads it as one')
        self.assertIn('written by the STEP', out,
                      'a marker the HALF writes says the work finished when '
                      'it has not started')

    def test_the_routing_skill_documents_the_same_pattern(self):
        """The driver prompt is where a teammate reads it; the skill is where a
        human does. A pattern in only one of the two drifts."""
        sk = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                          'SKILL.md')
        with open(sk, encoding='utf-8') as fh:
            txt = fh.read()
        self.assertIn('CANNOT BLOCK ON A DETACHED PROCESS', txt)
        self.assertIn('COMPLETION MARKER', txt)
        # It has to sit WITH the deadline guidance: the two are the same
        # problem seen from the two ends, and separating them is how one gets
        # applied without the other.
        self.assertLess(abs(txt.index('CANNOT BLOCK ON A DETACHED PROCESS')
                            - txt.index('Pass `--deadline` on any budget-capable step')),
                        6000,
                        'the hand-back pattern belongs beside the --deadline '
                        'guidance -- a hand-back without a deadline is two '
                        'blocked agents instead of one')


if __name__ == '__main__':
    unittest.main(verbosity=2)
