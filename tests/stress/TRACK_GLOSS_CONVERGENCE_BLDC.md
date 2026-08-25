# Track Gloss convergence study — `bldc_tester`

Evaluation date: 2026-08-25

This single-board study uses
`tests/stress/manifest_set12.json` → `bldc_tester.kicad_pcb`, with all 850
eligible straight tracks selected. It characterizes the existing repeated
global planner without changing its stopping rule.

## Observed trajectory

| State | Copper (mm) | Cumulative gain (mm) | Internal segments |
| --- | ---: | ---: | ---: |
| Initial | 2045.986743 | 0.000000 | 850 |
| Pass 1 | 1874.479044 | 171.507699 | 727 |
| Pass 4 | 1870.645286 | 175.341457 | 721 |
| Pass 16 | 1864.610380 | 181.376363 | 713 |
| Pass 32 | 1860.211863 | 185.774880 | 698 |
| Pass 41 | 1859.331789 | 186.654954 | 691 |
| Pass 54 / fixed point | 1859.331792 after KiCad quantization | 186.654951 | 669 after application |

All 55 observed geometry signatures (initial plus 54 changed passes) were
distinct. Copper was monotonically non-increasing within floating-point noise.
The run therefore did not oscillate or undo an earlier geometry. It converged
naturally, but required 54 changed passes.

Pass 1 performed the large multi-net improvement. Most later passes changed a
single net. Pass gains remained material through the middle of the run, rose
again around passes 21–24, then fell below 0.01% near pass 31. Passes 42–54
continued reducing segment complexity at effectively equal copper length.

## Consequences for stopping

A relative copper threshold alone is not a valid convergence criterion:

- stopping after pass 4 would leave about 11.313 mm of later copper gain;
- stopping after pass 16 would leave about 5.279 mm;
- stopping after pass 32 would leave about 0.880 mm;
- stopping at the length plateau around pass 41 would discard subsequent
  equal-length segment simplifications.

The long run is serial cleanup, not a geometry cycle. The preferable engine
improvement is to batch independent safe net/layer transformations into a
complete sweep. Convergence should then mean that a complete sweep produces no
lexicographic improvement: first angle correctness and copper length, then
segment/corner complexity. A time/work/pass guard remains necessary; reaching
it must report `fixed_point=false` rather than claim convergence.

The score CLI now accepts `--max-passes N`. `--trace-passes` emits each state as
`GLOSS_PASS_JSON` on stderr while preserving the final `SCORE=` stdout
contract. The default remains 16 until the second failing board and the batching
strategy have been evaluated.
