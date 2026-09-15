"""Real-board final-publication contracts; run as a standalone test script."""
import contextlib
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
for directory in ('py_placer', 'py_router', 'py_tools', 'tests/stress'):
    sys.path.insert(0, str(ROOT / directory))
from placement import provenance as pv
from placement.publication import publish_board, PublicationError, input_identity
from placement.writer import write_placed_output
from placement.pose_ops import _promote
from kicad_parser import parse_kicad_pcb
import stage_unaided
import provenance_audit


class Publication(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='krt960_')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.work = self.root / 'work'
        self.work.mkdir()
        self.board = self.work / 'board.kicad_pcb'
        with contextlib.redirect_stdout(io.StringIO()):
            stage_unaided.stage(str(ROOT / 'kicad_files/esp_prog.kicad_pcb'),
                               str(self.board), str(self.root / 'truth'))
        self.out = self.work / 'delivered.kicad_pcb'
        self.move = [dict(reference='R1', new_x=136.4, new_y=98.8, new_rotation=270)]

    def candidate(self):
        candidate = self.root / 'candidate.kicad_pcb'
        with contextlib.redirect_stdout(io.StringIO()):
            write_placed_output(str(self.board), str(candidate), self.move)
        return candidate

    def publish(self, candidate):
        with pv.declare_lever('place_pose.py', decision_source='model'):
            return publish_board(str(candidate), str(self.out), input_file=str(self.board))

    def test_external_promotion_authorizes_destination_before_mutation(self):
        candidate = self.candidate()
        shutil.copyfile(self.board, self.out)
        before = self.out.read_bytes()
        with self.assertRaisesRegex(pv.UnaidedViolation, 'before writing'):
            _promote(str(candidate), str(self.out))
        self.assertEqual(before, self.out.read_bytes())
        self.assertEqual(pv.read_ledger(str(self.work)), [])

    def test_final_snapshot_and_model_execution_distinction(self):
        self.publish(self.candidate())
        row = pv.read_ledger(str(self.work))[-1]
        self.assertEqual(row['poses_written']['R1'], [136.4, 98.8, 270])
        self.assertEqual(row['board_sha256'], pv.sha256_file(self.out))
        self.assertEqual(row['candidate_sha256'], row['board_sha256'])
        self.assertEqual(row['decision_source'], 'model')
        self.assertEqual(row['applied_by'], 'place_pose.py')
        self.assertEqual(provenance_audit.audit(str(self.work), str(self.out))[0], 0)

    def test_ledger_failure_before_and_after_rename_restores_every_file(self):
        candidate = self.candidate()
        shutil.copyfile(self.board, self.out)
        candidate.with_suffix('.design-brief.json').write_text('{"new": true}')
        self.out.with_suffix('.design-brief.json').write_text('{"old": true}')
        before = input_identity(str(self.out))
        replace = os.replace
        for after in (False, True):
            def fail(src, dst):
                if str(dst).endswith(pv.LEDGER_NAME):
                    if after:
                        replace(src, dst)
                    raise OSError('injected ledger failure')
                return replace(src, dst)
            with patch('placement.publication.os.replace', side_effect=fail):
                with self.assertRaisesRegex(PublicationError, 'preserved or restored'):
                    self.publish(candidate)
            self.assertEqual(before, input_identity(str(self.out)))
            self.assertEqual(pv.read_ledger(str(self.work)), [])
            self.assertFalse(pv._PENDING)

    def test_interrupt_after_successful_replace_restores(self):
        candidate = self.candidate()
        shutil.copyfile(self.board, self.out)
        before = self.out.read_bytes()
        replace = os.replace
        fired = False
        def interrupt(src, dst):
            nonlocal fired
            result = replace(src, dst)
            if str(dst) == str(self.out) and not fired:
                fired = True
                raise KeyboardInterrupt('after successful syscall')
            return result
        with patch('placement.publication.os.replace', side_effect=interrupt):
            with self.assertRaises(KeyboardInterrupt):
                self.publish(candidate)
        self.assertEqual(before, self.out.read_bytes())
        self.assertEqual(pv.read_ledger(str(self.work)), [])

    def test_recovery_failure_retains_evidence_and_blocks_audit(self):
        candidate = self.candidate()
        shutil.copyfile(self.board, self.out)
        replace = os.replace
        def fail(src, dst):
            if str(dst).endswith(pv.LEDGER_NAME) or '.krt-backup-' in str(src):
                raise OSError('injected recovery denial')
            return replace(src, dst)
        with patch('placement.publication.os.replace', side_effect=fail):
            with self.assertRaises(PublicationError) as caught:
                self.publish(candidate)
        self.assertEqual(caught.exception.details['output_state'], 'partial')
        journal = Path(caught.exception.details['recovery_journal'])
        self.assertTrue(journal.is_file())
        self.assertTrue(json.loads(journal.read_text())['backups'])
        self.assertEqual(provenance_audit.audit(str(self.work), str(self.out))[0], 5)
        with self.assertRaisesRegex(PublicationError, 'recovery required'):
            self.publish(candidate)

    def test_pending_cancel_duplicate_and_missing_output(self):
        with pv.declare_lever('place_pose.py'):
            pv.record_write(str(self.board), str(self.out), self.move, pending=True)
            with self.assertRaisesRegex(RuntimeError, 'already pending'):
                pv.record_write(str(self.board), str(self.out), self.move, pending=True)
            with self.assertRaisesRegex(OSError, 'does not exist'):
                pv.commit_write(str(self.out))
            self.assertIsNotNone(pv.cancel_write(str(self.out)))
            self.assertIsNone(pv.commit_write(str(self.out)))
        self.assertEqual(pv.read_ledger(str(self.work)), [])

    def test_interrupt_after_pending_registration_cancels_owned_record(self):
        candidate = self.candidate()
        record = pv.record_write
        def interrupt(*args, **kwargs):
            record(*args, **kwargs)
            raise KeyboardInterrupt('after pending registration')
        with patch('placement.provenance.record_write', side_effect=interrupt):
            with self.assertRaises(KeyboardInterrupt):
                self.publish(candidate)
        self.assertFalse(pv._PENDING)
        self.publish(candidate)
        self.assertEqual(len(pv.read_ledger(str(self.work))), 1)

    def test_thread_declarations_do_not_leak(self):
        seen = []
        with pv.declare_lever('place_pose.py'):
            thread = threading.Thread(target=lambda: seen.append(pv.active_lever()))
            thread.start()
            thread.join()
        self.assertEqual(seen, [None])

    def test_deep_regime_and_stale_input(self):
        deep = self.work.joinpath(*(['deep'] * 30), 'x.kicad_pcb')
        self.assertEqual(pv.regime_for(str(deep)), str(self.work))
        candidate = self.candidate()
        identity = input_identity(str(self.board))
        self.board.with_suffix('.design-brief.json').write_text('{"changed": true}')
        with pv.declare_lever('place_pose.py'):
            with self.assertRaisesRegex(PublicationError, 'requirements|requirement siblings changed'):
                publish_board(str(candidate), str(self.out), input_file=str(self.board),
                              expected_input=identity)
        self.assertFalse(self.out.exists())

    def test_in_place_baseline_is_preserved_and_audits(self):
        original = pv.sha256_file(self.board)
        candidate = self.candidate()
        with pv.declare_lever('place_pose.py'):
            publish_board(str(candidate), str(self.board), input_file=str(self.board))
        manifest = json.loads((self.work / pv.REGIME_NAME).read_text())
        self.assertNotEqual(manifest['staged_board'], str(self.board))
        self.assertEqual(pv.sha256_file(manifest['staged_board']), original)
        self.assertEqual(manifest['staged_sha256'], original)
        self.assertEqual(provenance_audit.audit(str(self.work), str(self.board))[0], 0)
        self.assertEqual(parse_kicad_pcb(str(self.board)).footprints['R1'].x, 136.4)

    def test_legacy_row_cannot_certify_an_unrecorded_lock_change(self):
        self.publish(self.candidate())
        ledger = self.work / pv.LEDGER_NAME
        row = pv.read_ledger(str(self.work))[0]
        row.pop('locks_written')
        row.pop('final_snapshot')
        ledger.write_text(json.dumps(row) + '\n', encoding='utf-8')
        # Old rows remain usable for coordinates and unchanged lock state.
        self.assertEqual(provenance_audit.audit(str(self.work), str(self.out))[0], 0)
        from placement.pose_ops import apply_locks
        apply_locks(str(self.out), ['R1'], [])
        code, doc = provenance_audit.audit(str(self.work), str(self.out))
        self.assertEqual(code, 5)
        self.assertEqual(doc['unverifiable_claims'], ['R1'])


if __name__ == '__main__':
    unittest.main()
