#!/usr/bin/env python3
"""Headless AI runs may WebFetch, and a run's downloaded datasheets are deleted
when the run ends (Andy, 2026-10-15: "full webfetch ability, but clean up
afterwards").

  1. Both headless allowlists grant WebFetch: the analysis runs'
     (`ai_backend.CLAUDE_ALLOWED_TOOLS`) and the Placement tab's
     (`placement_run.PLACEMENT_ALLOWED_TOOLS`).
  2. Both contracts tell the run where a download may go: the analysis
     sentence carves the datasheet folder out of "do not modify any files", and
     the placement instructions carry `ai_backend.DATASHEET_SENTENCE`.
  3. `ai_backend.datasheet_scratch()` removes its folder on a clean exit AND on
     an exception.
  4. END TO END through the GUI's real runner (`ai_gui.AISkillRunner`, with a
     stub wx): a fake CLI writes a "datasheet" into $KRT_DATASHEET_DIR and
     reports the path; after the run the folder is gone. A runner that leaked
     the folder, or never named it to the child, fails here -- the fake CLI
     exits 3 without the variable.
"""
import json
import os
import sys
import tempfile
import types

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)


def _stub_wx():
    """Just enough of wx for ai_gui to import and for its runner to report:
    CallAfter runs inline, every other name is an inert placeholder."""
    wx = types.ModuleType('wx')

    class _Any:
        def __init__(self, *a, **k):
            pass

        def __or__(self, other):
            return self
        __ror__ = __or__

    wx.__dict__['__getattr__'] = (
        lambda name: _Any() if name.isupper() else type(name, (_Any,), {}))
    wx.CallAfter = lambda f, *a, **k: f(*a, **k)
    sys.modules['wx'] = wx


_stub_wx()
from kicad_routing_plugin import ai_backend, ai_gui, placement_run  # noqa: E402

results = []


def ok(name, cond, detail=''):
    results.append(bool(cond))
    print(f"  {'ok  ' if cond else 'FAIL'} {name}"
          f"{(' -- ' + str(detail)) if detail and not cond else ''}")


def test_allowlists():
    for label, spec in (('analysis', ai_backend.CLAUDE_ALLOWED_TOOLS),
                        ('placement', placement_run.PLACEMENT_ALLOWED_TOOLS)):
        ok(f"the {label} allowlist grants WebFetch", 'WebFetch' in spec.split(','), spec)
        ok(f"the {label} allowlist still grants WebSearch", 'WebSearch' in spec.split(','), spec)


def test_contracts():
    c = ai_backend.ANALYSIS_CONSTRAINT
    ok("the analysis contract names the datasheet folder", '$KRT_DATASHEET_DIR' in c, c)
    ok("and still forbids modifying files otherwise", 'not modify any files' in c, c)
    ok("placement_run's mirrored sentence equals ai_backend's",
       placement_run.DATASHEET_SENTENCE == ai_backend.DATASHEET_SENTENCE,
       placement_run.DATASHEET_SENTENCE)
    instr = placement_run.build_placement_instructions('/tmp/run', 'place')
    ok("the placement instructions carry DATASHEET_SENTENCE",
       ai_backend.DATASHEET_SENTENCE in instr, instr[:300])
    ok("the sentence names the variable the runner sets",
       '$' + ai_backend.DATASHEET_DIR_ENV in ai_backend.DATASHEET_SENTENCE)


def test_scratch_context():
    with ai_backend.datasheet_scratch() as d:
        open(os.path.join(d, 'x.pdf'), 'w').write('pdf')
        ok("the folder exists inside the block", os.path.isdir(d))
    ok("a clean exit removes the folder and its download", not os.path.exists(d), d)
    try:
        with ai_backend.datasheet_scratch() as d2:
            open(os.path.join(d2, 'y.pdf'), 'w').write('pdf')
            raise RuntimeError('boom')
    except RuntimeError:
        pass
    ok("an exception removes it too", not os.path.exists(d2), d2)


FAKE_CLI = r'''
import json, os, sys
d = os.environ.get("KRT_DATASHEET_DIR")
if not d or not os.path.isdir(d):
    sys.exit(3)
p = os.path.join(d, "ME6211.pdf")
open(p, "w").write("%PDF-1.4 fake datasheet")
print(json.dumps({"type": "result", "subtype": "success",
                  "result": "RESULT=" + p}), flush=True)
'''


class _FakeBackend(ai_backend.ClaudeBackend):
    """The real Claude backend's stream parsing, with a fake CLI as argv."""
    def __init__(self, script):
        super().__init__()
        self._script = script

    def build_cmd(self, cli_path, prompt, model=None, effort=None, **kw):
        return [sys.executable, self._script]


def test_runner_cleans_up():
    with tempfile.TemporaryDirectory() as td:
        script = os.path.join(td, 'fake_cli.py')
        open(script, 'w').write(FAKE_CLI)
        done = {}
        runner = ai_gui.AISkillRunner(
            'fake', on_transcript=lambda t: None,
            on_done=lambda text, err: done.update(text=text, err=err),
            backend=_FakeBackend(script))
        runner.run('prompt')
        runner._thread.join(timeout=60)
        ok("the run finished", not runner.is_running() and done, done)
        text = done.get('text') or ''
        path = ai_backend.extract_result_line(text) if text else None
        ok("the child saw $KRT_DATASHEET_DIR and downloaded into it",
           path and path.endswith('ME6211.pdf'), (text, done.get('err')))
        if path:
            ok("after the run, the download is gone", not os.path.exists(path), path)
            ok("and so is its folder", not os.path.exists(os.path.dirname(path)),
               os.path.dirname(path))


def main():
    print("headless runs: WebFetch granted, datasheet downloads cleaned up")
    test_allowlists()
    test_contracts()
    test_scratch_context()
    test_runner_cleans_up()
    passed = sum(results)
    print(f"\n{passed}/{len(results)} checks passed")
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
