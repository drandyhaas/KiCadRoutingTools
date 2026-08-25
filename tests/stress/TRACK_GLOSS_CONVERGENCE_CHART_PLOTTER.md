# Track Gloss convergence study — `chart_plotter_hat`

Evaluation date: 2026-08-25

This is the second former 16-pass convergence failure from the medium-board
pilot: `tests/stress/manifest_set18.json` → `chart_plotter_hat.kicad_pcb`.
All 659 straight tracks were seeded; 10 tuned tracks were protected and 649
tracks were eligible.

## Convergence and performance

With the batched engine and the CLI's 16-pass guard, the board reaches an
in-memory fixed point in 8 changed global passes and 18.085 seconds:

- straight copper: 2053.824212 → 1887.177539 mm;
- copper saved: 166.646673 mm (8.113970%);
- segments: 659 → 530 (129 removed net);
- fixed point: true.

The interactive plugin budget (four global passes, two local passes per batch)
was also replayed on the full selection. It returned a validated bounded result
in 14.304 seconds, with the same 166.646673 mm copper gain and 122 segments
removed net. `fixed_point=false` correctly records that four later equal-length
segment simplifications remain. This all-board selection is much broader than
the plugin's normal one-connection use.

## Safety finding and correction

The first replay exposed false safety on internal copper layers. Via obstacles
were built from `range(TopLayer, BottomLayer)`, but KiCad's PCB layer IDs are
not contiguous: on this board `In1.Cu` is 4 and `In2.Cu` is 6. A through via
reported endpoints 0 and 2 while its native layer set was `(0, 2, 4, 6)`.
Consequently the old model did not see vias on the internal layers.

The reader now uses each via's native KiCad `GetLayerSet()` and filters the
actual `.Cu` layers. After that correction and a zone refill, the glossed board
adds no clearance violation and no short circuit. The original board has 118
DRC violations and the glossed board 117.

One pre-existing GND zone defect changes classification: pad U1-21 starts with
an incomplete thermal (one spoke instead of two) and ends with no spoke, so
KiCad reports it as one unconnected item instead of one `starved_thermal`.
The total number of these errors remains one, but this classification change
means zone-connectivity preservation is not yet fully validated by the
API-neutral engine and requires a separate filled-zone topology guard.

