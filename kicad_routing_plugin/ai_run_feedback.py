"""Present AI Drive run outcomes after validating a routing plan (#1655)."""

import wx

from .ai_backend import extract_result_line
from .ai_plan import parse_plan_result


def finish_ai_run(tab, result_text, error):
    """Finish on the wx thread; CLI success alone does not make a plan usable."""
    tab._elapsed_timer.Stop()
    tab.gauge.SetValue(0)
    tab.plan_btn.Enable(tab.routing_dialog is not None)
    tab.review_btn.Enable()
    tab.diagnose_btn.Enable()
    tab.run_plan_btn.Enable(bool(tab._plan_steps))
    tab.run_all_plan_btn.Enable(tab.run_plan_btn.IsEnabled())
    tab.cancel_btn.Disable()
    kind, tab._pending_kind = tab._pending_kind, None
    elapsed = tab._elapsed_seconds

    # A result event can be the only copy of the report (e.g. after a missing
    # assistant event). Preserve it without duplicating an already streamed one.
    report = (result_text or "").strip()
    if report and report not in tab.output_ctrl.GetValue():
        tab.output_ctrl.AppendText("\n" + report + "\n")

    if error:
        cancelled = error == "Cancelled."
        status = "Cancelled" if cancelled else "Failed"
        backend = tab._runner.backend if tab._runner else tab._current_backend()
        hint = backend.auth_hint(error)
        if tab.output_ctrl.GetValue().rstrip().endswith(error.strip()):
            appended = hint
        else:
            appended = f"\n{error}{hint}"
        if appended:
            tab.output_ctrl.AppendText(appended + "\n")
        tab.elapsed_label.SetLabel(f"{status} ({elapsed}s)")
        tab.Layout()
        tab._log(f"{backend.label}: {status} after {elapsed}s: {error}")
        if not cancelled:
            wx.MessageBox(f"{error}{hint}\n\nSee the AI transcript for details.",
                          "AI analysis failed", wx.OK | wx.ICON_WARNING,
                          parent=tab)
        return

    value = extract_result_line(result_text)
    tab.parsed_ctrl.SetValue(
        value[:200] + ("..." if len(value) > 200 else "")
        if value is not None else "(no RESULT= line found)")

    if kind == "plan":
        steps, errors = (parse_plan_result(value) if value is not None
                         else (None, ["The AI response has no RESULT= plan line."]))
        for message in errors:
            tab.output_ctrl.AppendText(f"plan: {message}\n")
            tab._log(f"AI plan: {message}")
        if steps is None:
            tab.run_plan_btn.Disable()
            tab.run_all_plan_btn.Disable()
            status = f"No routing plan generated ({elapsed}s)"
            tab.elapsed_label.SetLabel(status)
            tab.Layout()
            tab.output_ctrl.AppendText("\n" + status + "\n")
            tab._log("AI: " + status)
            wx.MessageBox(
                "No routing plan was generated.\n\n" + "\n".join(errors)
                + "\n\nReview the AI transcript for the full report and "
                  "the checks to address before retrying Plan Routing.",
                "Routing plan not generated", wx.OK | wx.ICON_WARNING,
                parent=tab)
            return
        tab._install_plan_steps(steps)
        status = f"Plan ready: {len(steps)} step(s) ({elapsed}s)"
    else:
        status = f"Done ({elapsed}s)"
    tab.elapsed_label.SetLabel(status)
    tab.Layout()
    tab.output_ctrl.AppendText(f"\n--- {status} ---\n")
    tab._log("AI: " + status)
