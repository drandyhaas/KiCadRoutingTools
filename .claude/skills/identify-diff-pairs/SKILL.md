---
name: identify-diff-pairs
description: Identifies differential pairs on a KiCad PCB by pin function via datasheet lookup, catching pairs whose net names don't follow P/N conventions. Recommends per-interface routing parameters (differential impedance, gap, intra-pair matching) and outputs ready-to-use route_diff.py commands.
---

# Identify Differential Pairs

When this skill is invoked with a KiCad PCB file, find all differential pairs — including ones name-based detection misses — and recommend routing parameters per interface.

## Step 1: Baseline Name-Based Detection

```bash
python3 list_nets.py path/to/file.kicad_pcb --diff-pairs
```

This catches pairs following P/N naming conventions. Record the result as the baseline.

## Step 2: Datasheet-Based Detection by Pin Function

For each IC likely to have differential interfaces (footprint/value keywords: FPGA, PHY, SERDES, USB, ETH, HDMI, LVDS, MIPI, SATA, PCIE, DDR, transceiver, redriver):

1. Check pad metadata first — KiCad symbols often carry it:

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')
fp = pcb.footprints['U3']
for pad in fp.pads:
    print(pad.pad_number, pad.pinfunction, pad.net_name)
```

Differential pin functions usually pair as `..._P`/`..._N`, `TX+`/`TX-`, `D+`/`D-`.

2. Where `pinfunction` is missing or generic, **WebSearch the part's datasheet pinout** and map differential pin pairs to pad numbers, then to nets via the pads.

3. **Trace through series passives**: if a differential pin connects to a 2-pad R/C/L, follow to the net on the component's other pad — the pair often gets its "real" net name on the far side of AC coupling caps or series resistors.

A pair is **confirmed** when both nets trace back to a documented differential pin pair. Nets that look paired only by name or topology are **suspected** — list them separately for the user to confirm.

## Step 3: Recommend Parameters per Interface

| Interface | Differential impedance | Notes |
|-----------|------------------------|-------|
| USB 2.0 / 3.x | 90 Ω | Intra-pair matching matters from USB 2.0 HS up |
| LVDS | 100 Ω | |
| Ethernet (MDI) | 100 Ω | |
| HDMI / TMDS | 100 Ω | |
| PCIe | 85 Ω | AC coupling caps in-line on TX |
| SATA | 90 Ω | |
| MIPI D-PHY | 100 Ω | |
| DDR (DQS, CK) | 100 Ω typical | Check the memory datasheet |
| CAN | 120 Ω | Low speed; tolerances relaxed |

From the impedance, derive the routing flags:

- `--impedance <Z>` computes per-layer widths and gap from the stackup (preferred when the stackup is realistic — see `/recommend-stackup`), or set `--track-width`/`--diff-pair-gap` explicitly.
- `--diff-pair-intra-match` for interfaces fast enough that P/N skew matters (USB HS+, PCIe, SATA, HDMI, LVDS at high rates).
- Note which pairs allow polarity swapping at the receiver (PCIe, most SerDes are polarity-tolerant; USB is not) — relevant to whether `--no-fix-polarity` is safe to relax.

## Step 4: Output

1. **Confirmed pairs**, grouped by interface, each group with a ready-to-run command:

```bash
python -X utf8 route_diff.py board.kicad_pcb board_usb.kicad_pcb \
    --nets "USB_DP" "USB_DM" \
    --impedance 90 --diff-pair-intra-match \
    2>&1 | tee /tmp/route_usb.txt
```

2. **Suspected pairs** with the evidence (matching pin functions but unverified part, name-only pairing), asking the user to confirm before routing them as pairs.

3. A note of any pairs found by Step 1 but *not* corroborated by pin function — possible false positives of name matching (e.g. unrelated nets that happen to end in P/N).
