# Cloud plan+route (Modal)

Upload a KiCad board; an LLM agent on Modal plans it (the repo's own
`plan-pcb-routing` skill), routes it with the repo's CLI tools, verifies with
`check_drc` + `check_connected`, and you download the routed board plus a
deterministic replay manifest.

No Anthropic account involved: the agent is `opencode` (the stress harness's
#503 backend) talking to a **self-hosted open model on Modal** via vLLM's
OpenAI-compatible endpoint.

## One-time setup

```bash
# 1. a shared token gating the LLM endpoint (any random string)
modal secret create kicad-llm-token KICAD_LLM_TOKEN=$(openssl rand -hex 24)

# 2. deploy the LLM half (persists, scales to zero between uses)
modal deploy tests/stress/modal_route/vllm_app.py
```

## Route a board

```bash
modal run tests/stress/modal_route/route_app.py --board my_board.kicad_pcb
# -> my_board_routed/{routed.kicad_pcb, routed.kicad_pro, redo_commands.sh,
#                     SUMMARY.md, agent.log}
```

`--notes "route the RF section first"` passes extra instructions;
`--out DIR` picks the download dir. Siblings (`.kicad_pro`/`.kicad_prl`/
`.kicad_dru`) upload automatically -- they carry the DRC floor and per-layer
rules.

## Model choice

`KICAD_LLM_MODEL` / `KICAD_LLM_GPU` at **deploy** time:

| model | GPU | hot cost | per board (est) |
|---|---|---|---|
| `Qwen/Qwen3-Coder-30B-A3B-Instruct` (default) | `A100-80GB` | ~$3/h | < $5 |
| `moonshotai/Kimi-K2-Instruct` | `H100!:8` | ~$30-50/h | $30-100 |

The server scales to zero after `scaledown_window` (5 min) idle; weights are
cached in the `kicad-hf-cache` volume so later cold starts load rather than
download. The routing container itself is CPU (~$0.30/h) and is billed only
per board.

## Notes

- The board container is a **clean checkout of HEAD** (same discipline as
  `modal_sweep`; `KICAD_SWEEP_DIRTY=1` is deliberately not honored here).
- Every board-mutating tool self-records (`REDO_MANIFEST`), so a cloud-routed
  board is replayable/gradeable locally exactly like a stress-corpus board:
  `tests/stress/redo_stress_test.py redo_commands.sh --remap ...`.
- Smoke it with a SMALL board first (the modal_sweep doctrine): a tiny 2-layer
  board exercises endpoint auth, tool-calling, staging, manifest capture, and
  the download path in minutes.
- Unverified until first deploy: vLLM flag names drift between releases
  (`--tool-call-parser` choices), the opencode custom-provider config schema,
  and the Qwen3 tool-template pairing. Expect one shake-out session, like the
  sweep's.
