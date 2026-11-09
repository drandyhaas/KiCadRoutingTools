"""#1655: a refused AI plan reports its outcome through the real wx tab.

Run with KiCad's bundled Python. No agent CLI runs and no board is modified.
CLI discovery and the notification dialog are stubbed; backend stream parsing,
plan validation, installation, completion and wx controls are real.
"""
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

import wx
from kicad_routing_plugin.ai_backend import BACKENDS
from kicad_routing_plugin.ai_gui import AITab


class AIPlanOutcomeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = wx.App(False)
        cls.quiet_log = wx.LogNull()

    def setUp(self):
        # CLI discovery is unrelated to completion and can probe slow shims.
        for backend in BACKENDS.values():
            patch.object(backend, 'find_cli', return_value=None).start()
        self.addCleanup(patch.stopall)
        self.frame = wx.Frame(None)
        self.logs = []
        self.tab = AITab(self.frame, 'board.kicad_pcb', self.logs.append)
        self.tab._pending_kind = 'plan'
        self.tab._elapsed_seconds = 366
        self.tab.elapsed_label.SetLabel('6m 06s')
        self.tab.output_ctrl.SetValue('Planning...\n')
        self.installed = []
        real_install = self.tab._install_plan_steps

        def install(steps):
            self.assertNotIn('Plan ready', self.tab.elapsed_label.GetLabel())
            real_install(steps)
            self.installed.append(steps)

        self.tab._install_plan_steps = install
        self.popup = patch('wx.MessageBox', return_value=wx.OK).start()

    def tearDown(self):
        self.frame.Destroy()
        self.app.ProcessPendingEvents()

    def complete(self, report, streamed=False, backend='claude'):
        state = BACKENDS[backend].stream_state()
        if backend == 'claude':
            if streamed:
                text = state.feed({'type': 'assistant', 'message': {
                    'content': [{'type': 'text', 'text': report}]}})
                self.tab._append_transcript(text)
            state.feed({'type': 'result', 'is_error': False, 'result': report})
        else:
            text = state.feed({'type': 'text', 'part': {
                'type': 'text', 'text': report}})
            if streamed:
                self.tab._append_transcript(text)
        self.tab._on_done(*state.finish(0, ''))

    def assert_no_plan(self):
        self.assertFalse(self.installed)
        self.assertFalse(self.tab.run_plan_btn.IsEnabled())
        self.assertFalse(self.tab.run_all_plan_btn.IsEnabled())
        self.assertIn('No routing plan', self.tab.elapsed_label.GetLabel())
        self.assertNotIn('AI: done', ''.join(self.logs))
        self.popup.assert_called_once()

    def test_recorded_empty_plan_is_not_reported_as_success(self):
        report = ('STOP at Step 0: placement gate fails.\n'
                  '67 of 68 parts have pad copper outside the outline.\n'
                  'Run placement first.\nRESULT={"steps":[]}')
        self.complete(report, streamed=True)
        self.assert_no_plan()
        self.assertEqual(self.tab.output_ctrl.GetValue().count(report), 1)
        self.assertIn('no usable steps', self.popup.call_args.args[0])

    def test_result_only_report_is_displayed_for_both_backends(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                self.tab._pending_kind = 'plan'
                self.tab.output_ctrl.SetValue('Planning...\n')
                report = 'Specific placement refusal\nRESULT={"steps":[]}'
                self.complete(report, backend=backend)
                self.assertEqual(self.tab.output_ctrl.GetValue().count(report), 1)

    def test_streamed_opencode_parts_are_not_repeated(self):
        # opencode's result joins its text parts; the transcript has the tool
        # lines between them, so the joined reply is not one substring of it.
        state = BACKENDS['opencode'].stream_state()
        events = [
            {'type': 'text', 'part': {'type': 'text', 'text': 'Reading the board.'}},
            {'type': 'tool_use', 'part': {'tool': 'bash', 'state': {
                'status': 'completed', 'input': {'description': 'check_assembly'}}}},
            {'type': 'text', 'part': {'type': 'text',
                                      'text': 'Ready.\nRESULT={"steps":[{"action":"route"}]}'}},
        ]
        for event in events:
            text = state.feed(event)
            if text:
                self.tab._append_transcript(text)
        self.tab._on_done(*state.finish(0, ''))
        self.assertEqual(self.tab.output_ctrl.GetValue().count('Reading the board.'), 1)
        self.assertEqual(self.tab.output_ctrl.GetValue().count('RESULT='), 1)
        self.assertEqual(len(self.installed), 1)

    def test_missing_result_notifies_and_preserves_report(self):
        report = 'The layout must be corrected before routing.'
        self.complete(report)
        self.assert_no_plan()
        self.assertIn(report, self.tab.output_ctrl.GetValue())

    def test_invalid_plan_notifies(self):
        self.complete('RESULT={"steps":[{"action":"invented"}]}')
        self.assert_no_plan()
        self.assertIn('unknown action', self.popup.call_args.args[0])

    def test_success_installs_plan_before_reporting_ready(self):
        report = 'Ready for routing.\nRESULT={"steps":[{"action":"route"}]}'
        self.complete(report)
        self.assertEqual(self.installed, [[{'action': 'route'}]])
        self.assertEqual(self.tab.plan_list.GetCount(), 1)
        self.assertTrue(self.tab.run_plan_btn.IsEnabled())
        self.assertIn('Plan ready', self.tab.elapsed_label.GetLabel())
        self.popup.assert_not_called()
        self.assertEqual(self.tab.output_ctrl.GetValue().count(report), 1)

    def test_backend_error_keeps_a_failed_status(self):
        self.tab._on_done(None, 'Provider unavailable')
        self.assertIn('Failed', self.tab.elapsed_label.GetLabel())
        self.assertIn('Provider unavailable', self.tab.output_ctrl.GetValue())
        self.popup.assert_not_called()
        self.assertFalse(self.installed)

    def test_streamed_auth_error_is_not_duplicated(self):
        error = 'Not logged in'
        self.tab.output_ctrl.SetValue(error)
        self.tab._on_done(None, error)
        self.assertEqual(self.tab.output_ctrl.GetValue().count(error), 1)
        self.assertIn('/login', self.tab.output_ctrl.GetValue())

    def test_cancel_has_a_persistent_status(self):
        self.tab._on_done(None, 'Cancelled.')
        self.assertIn('Cancelled', self.tab.elapsed_label.GetLabel())
        self.popup.assert_not_called()
        self.assertFalse(self.installed)

    def test_review_report_still_finishes_without_a_plan(self):
        self.tab._pending_kind = 'review'
        self.complete('Review report\nRESULT=PASS')
        self.assertFalse(self.installed)
        self.popup.assert_not_called()
        self.assertIn('Review report', self.tab.output_ctrl.GetValue())
        self.assertIn('Done', self.tab.elapsed_label.GetLabel())


if __name__ == '__main__':
    unittest.main()
