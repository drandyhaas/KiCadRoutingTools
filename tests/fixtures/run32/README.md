# run32 fixtures: run 32's placed glasgow_revC

Copied from `repro/run32/` on `edgehero/KiCadRoutingTools@run32-assets`, the assets #1031 was filed against. That run's work directory (`wk/run32/`) is gitignored, so the test that pins #1031 on this board used to self-skip on every clone, and no machine had the board.

| file | what it is | sha256 (of the committed LF bytes) |
|---|---|---|
| `placed_v2.kicad_pcb` | glasgow_revC as run 32's placement left it, unrouted. Its rule area is a ~1.7 mm no-track, no-via band round the outline on F.Cu and B.Cu, with parts seated against it | `ce68fd39…6514` |
| `placed_v2.kicad_pro` | its project sibling (carries the DRC floor; never copy a board without it) | `488b5933…a714` |

Reader: `test_1031_keepout_run32_repro`. It checks that the pad-legality grader names exactly the parts whose pads no track can reach through that band (C89, D12, R11, R12, U14, U9), and that it does not name D4, D6, D11 or U31, whose pads sit near the band but connected when each net was routed alone on this board.
