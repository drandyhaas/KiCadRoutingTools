# Boards for the bus studies

These boards are the bus studies' inputs (`awx/README.md`). They are not part of the
routing test corpus (`kicad_files/`), whose tests grade every board it tracks, and a
board here is gzipped so that no census of the repository's `*.kicad_pcb` reads it:
`gunzip -c` it beside its `.kicad_pro` (the README's recipes do).

## zynq_ad9364

`zynq_ad9364.kicad_pcb.gz` and `zynq_ad9364.kicad_pro` are `ZYNQ7020_AD9364_V2` from
[kangyuzhe666/ZYNQ7010-7020_AD9363](https://github.com/kangyuzhe666/ZYNQ7010-7020_AD9363)
(`kicad/`), a Zynq-7020 (`U1`) with a DDR3 (`U2`), an AD9364 (`U5`), a USB PHY (`U4`)
and an Ethernet PHY (`U3`), routed by its designer. It is distributed under the Apache
License 2.0, a copy of which is `zynq_ad9364.LICENSE`.

Changed from the original:

- the board file re-saved by KiCad 10.0's pcbnew (its format only; the original is a
  KiCad 5 file, version 20201116);
- two zero-length lines on `Edge.Cuts`, at (0, 0) and at (59.2836, -30.48), removed;
- both files renamed from `ZYNQ7020_AD9364_V2` (the project file is otherwise unchanged);
- the board file gzipped.
