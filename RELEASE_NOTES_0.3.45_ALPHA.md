# KiCad Track Gloss 0.3.45 alpha

This testing build restores the one-connection gloss as the foundation of
larger selections.

- Every selected seed expansion is retained as a distinct local connection.
- Multi-connection planning rebuilds those scopes through the exact same
  converged workflow used by a one-segment selection.
- The best compatible local composition is ranked alongside the global plan,
  so a larger selection cannot silently choose a lower-quality result.
- Native DRC recovery operates on local connections, including independent
  connections belonging to the same net, instead of treating a net as the
  smallest unit.
- Recovery first obtains a safe local base, then validates gain-ranked batches
  and bisects only rejected batches. The best already approved result is kept
  when the configured deadline is reached.
- The CLI uses the same connection-local recovery. A finite CLI time budget
  reserves half of its duration for native validation and recovery.

Targeted KiCad 10.0.5 results with a 20 s budget:

- `mydewcontroller`: 59.545967 mm, 39 segments, 31 connections retained;
- `kivu12`: 107.200000 mm, direct DRC acceptance in 2.97 s;
- `polykit_x_inputboard`: 103.327366 mm, 18 segments, 121 connections;
- `led_ring_crossbar`: one safe segment simplification retained;
- `uncutgem_nv`: 4.156764 mm and one segment retained.

Final validation: 98 unit tests, seven identical selection orders, 334 real
connection scopes, and 269 fresh in-memory plan applications.
