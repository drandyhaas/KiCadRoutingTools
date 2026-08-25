# Track Gloss stress pilot — five medium boards

Evaluation date: 2026-08-25

This bounded pilot checks the all-tracks CLI on five medium boards already
downloaded from the versioned DrAndyHaas stress manifests. The independent
Codex oracle is invoked only after a CLI copper reduction of at least 5%.

## CLI results

| Board | Copper before (mm) | Copper after (mm) | Saved (mm) | Saved (%) | Segments saved | Passes | Time (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `set10/zosko__noahfc.kicad_pcb` | 768.449 | 694.097 | 74.352 | 9.676 | 31 | 3 | 4.547 |
| `set14/tx_band_splitter.kicad_pcb` | 1868.684 | 1665.585 | 203.099 | 10.869 | 59 | 4 | 6.343 |
| `set15/moco_bkd8316.kicad_pcb` | 550.265 | 498.484 | 51.781 | 9.410 | 20 | 2 | 14.063 |
| `set17/rfswitch01.kicad_pcb` | 272.613 | 211.263 | 61.350 | 22.504 | 21 | 2 | 1.188 |
| `set22/reaction_wheel_foc.kicad_pcb` | 1019.924 | 862.585 | 157.338 | 15.426 | 75 | 3 | 5.469 |

All five boards qualified for the oracle. Across this deliberately selected
pilot, the CLI saved 547.921 mm from 4479.934 mm of straight-track copper
(12.230%) and removed 206 segments. These aggregate figures characterize only
this five-board sample, not the complete stress corpus.

## Oracle status

No valid CLI-versus-oracle comparison is available yet. All five Codex runs
were prevented from reading or modifying their isolated workspaces by the
native Windows sandbox. They returned unchanged candidates, which the corpus
driver now classifies as `oracle_invalid_no_change`; unchanged output is not
accepted as an oracle success when the CLI has already demonstrated a positive
gain.

The raw JSONL transcripts and generated summary remain outside the repository
under the selected stress root for diagnosis and a later checkpointed retry.

## Additional candidate failures

Four initially sampled medium boards were replaced so that the bounded pilot
would contain five measurable CLI results:

- `set10/vitalseptfonds__openweatherstation_mainboard.kicad_pcb`: no admissible
  unlocked, non-differential track;
- `set12/bldc_tester.kicad_pcb`: no fixed point after 16 changed passes;
- `set18/chart_plotter_hat.kicad_pcb`: no fixed point after 16 changed passes;
- `set25/opengammakit.kicad_pcb`: no fixed point after 16 changed passes.

The three convergence failures are engine findings to investigate before a
full-corpus run. They are not silently treated as zero-gain boards.
