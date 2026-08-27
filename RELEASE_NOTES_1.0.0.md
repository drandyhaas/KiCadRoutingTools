# KiCad Track Gloss 1.0.0

This stable release makes one selected connection the fundamental gloss unit
and extends the same behavior safely to multi-connection selections.

- A single selected segment expands to its exact connection and follows the
  normal converged gloss path.
- Larger selections compose independently planned connections and keep the
  best compatible candidate.
- Native KiCad DRC validation is anytime: the best approved result is retained
  throughout the session and is returned when the configured deadline expires.
- The first DRC wave pairs the best geometric candidate with the conservative
  candidate. A fixed top-two cutoff can no longer hide a safe fallback.
- If no global candidate is safe, recovery proceeds directly connection by
  connection. If a safe incumbent exists, remaining alternatives may improve
  it but can never replace it with a lower-quality result.
- CLI and plugin use the same candidate arbiter and the same default minimum
  per-transformation saving of 0.2 mm.

SET21 qualification with a 20 s budget and 0.2 mm minimum saving:

- `kivu12`: 107.200000 mm saved, direct DRC acceptance;
- `polykit_x_inputboard`: 148.567303 mm and 15 segments saved;
- `led_ring_crossbar`: 0.054232 mm and one segment saved;
- `uncutgem_nv`: 77.119131 mm and eight segments saved.

The four-board total is 332.941 mm and 24 segments, with no native DRC
category increase. The previous 0.3.44 reference produced 255.767 mm and 15
segments on the same boards.

Final qualification:

- 100 unit tests;
- seven identical selection orders at 66.020888 mm;
- identical original, half-segment, and third-segment representations;
- 334 real connection scopes, 269 changes, and 269 fresh in-memory
  applications.
