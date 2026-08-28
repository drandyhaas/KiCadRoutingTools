# KiCad Track Gloss 1.3.0 (testing)

- Adds fixed-point interior-segment translations and maximal local corner
  chamfers without invoking a router or grid search.
- Repeats a one-net gloss until no further safe geometric improvement remains,
  while retaining the best compatible work for multi-net time-bounded runs.
- Uses three parallel native DRC candidates and keeps every already-approved
  result as an anytime floor.
- Removes geometric intent heuristics. Only KiCad 10 locked items/groups,
  generated tracks, and KiCad-recognized differential pairs are protected.
- Verifies the exact requested copper geometry after applying it to the live
  board and rolls back on any readback mismatch.
- Removes obsolete adapter entry points and the unused conservative-candidate
  wrapper; documentation now matches the KiCad 10-only authority model.
- Keeps the 0.2 mm default minimum saving and exposes the same setting to the
  plugin and CLI.
