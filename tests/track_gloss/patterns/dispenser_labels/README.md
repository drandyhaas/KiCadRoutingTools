# `dispenser_labels` Track Gloss regression fixture

This fixture is a byte-for-byte copy of the KiCad 10 board, project, and
design-rule files supplied for the Track Gloss investigation. The source files
were copied from the local `dispenser` project on 2026-08-24; tests never save
or modify that source project.

SHA-256 fingerprints:

- `dispenser_labels.kicad_pcb`: `B276B421666B955944EFBCEE579A0D8273385DDD8E8BFBF2F41D41612518F9B1`
- `dispenser_labels.kicad_pro`: `0D9CD2BB27C0DDEE20FF389ED4927FE31DF053958787F500C5627E5AFD6B9BE0`
- `dispenser_labels.kicad_dru`: `18412D66DC4BDD8A958FFBB22BCF48804675197E902B5B15607AA16E1D38B591`

Run the real-board pattern replay with KiCad's bundled Python:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py
```

The replay loads a fresh in-memory board for every accepted modification and
never writes a PCB file.

It also treats all 706 straight tracks as one simultaneous selection. The
expected deterministic result is 4.721872 mm of copper saved and a net
reduction of 39 segments (103 removed, 64 added), invariant under board,
reverse, ascending-net, descending-net, and shuffled input orders.

The `/cpu/~{csn}` segment UUID
`58ebb541-fac6-4d02-8a68-65aca50766b5` is also a dedicated responsiveness
regression: its expanded connection contains 111 dense tuning micro-segments,
which must all be protected without invoking the geometric planner.

The VCC segment UUID `cc798608-5e9b-4c2a-9856-dde85f9d85f0` is a pad-envelope
regression. Its expanded connection must save 0.117157 mm and remove four net
segments while preserving portions of existing copper that overlap only the
adapter's conservative circular pad representation.
