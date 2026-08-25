# Independent Codex gloss oracle

This directory contains everything specific to the experimental Codex agent.
It is intentionally separate from `kicad_track_gloss/`: the agent is an
independent search oracle, not another entry point into the plugin engine.

## Contents

- `prompts/kicad_ai_gloss.md`: versioned agent contract.
- `schemas/scope.schema.json`: shared scope-manifest format.
- `schemas/result.schema.json`: machine-readable run-report format.
- `examples/scope-all.json`: whole-board scope.
- `examples/scope-subset.json`: mixed net/segment scope.
- `run_ai_gloss.py`: isolated launcher suitable for packaging as an `.exe`.

## Independence contract

The agent work directory receives only copies of the board, matching project
and design rules, the scope manifest, and the prompt. It must not receive the
Track Gloss source, CLI, output, fixtures, or expected scores. Comparison with
Track Gloss happens only after the agent has produced and validated its board.

The launcher uses Codex non-interactive mode in an ephemeral thread and a
workspace-write sandbox. It initializes a temporary Git repository because
Codex normally requires one. The candidate starts as a copy of the input and
the user's original is never exposed as a writable agent path. The launcher
also rejects the run if the isolated source-board copy changes.

On native Windows, the launcher retains the operator's Codex configuration and
execution rules because they provision the bounded command policy used by the
sandbox. It does not use full-access mode. Approval requests are disabled for
the non-interactive run, so a command outside the configured policy fails
closed instead of pausing the batch.

The official Codex documentation describes `codex exec`, JSONL output,
ephemeral runs, explicit sandboxes, and automation authentication:
<https://learn.chatgpt.com/docs/non-interactive-mode>. The Codex SDK is the
future integration route for a richer native UI:
<https://learn.chatgpt.com/docs/codex-sdk>.

## Prototype usage

Requirements:

- a separately installed and authenticated Codex CLI available as `codex`;
- KiCad 10 `kicad-cli` for independent before/after DRC validation;
- Python 3.10 or newer.

```powershell
python codex\run_ai_gloss.py `
  --board design.kicad_pcb `
  --project design.kicad_pro `
  --rules design.kicad_dru `
  --scope-file codex\examples\scope-all.json `
  --output design_ai_gloss.kicad_pcb `
  --kicad-cli D:\kicad\bin\kicad-cli.exe
```

The launcher publishes the candidate only when it is a readable KiCad board
and its DRC violation and unconnected-item category counts do not increase.
It also writes `<output>.codex.jsonl` and `<output>.codex-result.json` beside
the board. These artifacts record the agent transcript and deterministic host
validation; they are not inputs to Track Gloss.

## Building a Windows executable

The launcher uses only the Python standard library and can be packaged with a
tool such as PyInstaller:

```powershell
py -3.12 -m PyInstaller --onefile --name KiCadAIGloss `
  --add-data "codex/prompts;prompts" `
  --add-data "codex/schemas;schemas" `
  codex/run_ai_gloss.py
```

Do not embed a Codex login, `auth.json`, or API key in the executable. Codex
must use the operator's existing authentication or a separately provisioned
automation credential.

## Comparison protocol

For a fair experiment, first produce `Aia` in this isolated environment. Only
then run the deterministic Track Gloss scorer on `A0`, `Acli`, and `Aia`. Use
the same scope manifest for all three once scoped scoring is desired. A valid
`Aia` that is shorter than `Acli` and exposes a transformation the engine
cannot reproduce is a candidate for a new regression pattern.
