# GUI/CLI engine-parity harness

Measures whether the claude tab's "run selected steps" (GUI engine calls on
the live pcbnew board) produces the same final board as the recorded stress
chain's CLI steps run file-to-file.

- `test_gui_engine_parity.py` — headless: CLI leg via subprocesses, GUI leg
  inside KiCad's bundled python (auto re-exec) driving the real tab apply
  methods on shimmed dialog instances. Prints a parity report; known
  deliberate divergences are listed in the module docstring.

Workdir: `tests/gui_parity/work/` (gitignored).
