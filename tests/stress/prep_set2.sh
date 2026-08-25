#!/bin/bash
# Normalize+strip every set-2 board (one pcbnew process each, segfault-safe).
# Replacement board (lpddr4_testbed) -> boards_unrouted/ (joins set 1).
# Set-2 boards -> boards_unrouted_set2/.
# Run AFTER set 1 finishes so pcbnew loads of big boards don't OOM the 8GB box.
set -u
STRESS="$HOME/Documents/kicad_stress_test"
SRC="$STRESS/sources/github_set2"
KPY="/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"
PREP="$STRESS/scripts/prep_set2.py"
SILKPY="$(cd "$(dirname "$0")" && pwd)/set_silk_ignore.py"
# Silk-DENSE boards (keyboard legends etc.) whose O(n^2) silkscreen DRC makes KiCad's
# interactive DRC crawl for minutes -- demote the silk checks to 'ignore' in their
# .kicad_pro so DRC is fast (our router never places silkscreen). Space-separated;
# add board names as needed.
SILK_IGNORE="lily58"
mkdir -p "$STRESS/boards_unrouted_set2" "$STRESS/boards_set2"

# name|source-filename-fragment|dest-dir
MAP=(
  "ulx3s|emard__ulx3s__|$STRESS/boards_unrouted_set2"
  "cparti_fpga|AAWO__CPArti|$STRESS/boards_unrouted_set2"
  "butterstick|gregdavill__butterstick__|$STRESS/boards_unrouted_set2"
  "cynthion|greatscottgadgets__cynthion|$STRESS/boards_unrouted_set2"
  "schoko|machdyne__schoko__|$STRESS/boards_unrouted_set2"
  "tinytapeout_qfn|TinyTapeout__breakout|$STRESS/boards_unrouted_set2"
  "usb_sniffer|ataradov__usb-sniffer__|$STRESS/boards_unrouted_set2"
  "ddr5_testbed|antmicro__ddr5-testbed__|$STRESS/boards_unrouted_set2"
  "caravel_nucleo|efabless__caravel_board__|$STRESS/boards_unrouted_set2"
  "system76_launch|system76__launch__|$STRESS/boards_unrouted_set2"
  "crkbd|foostan__crkbd__|$STRESS/boards_unrouted_set2"
  "sofle_pico|josefadamcik__SofleKeyboard__|$STRESS/boards_unrouted_set2"
  "lily58|kata0510__Lily58__|$STRESS/boards_unrouted_set2"
  "free_dap|ataradov__free-dap__|$STRESS/boards_unrouted_set2"
  "nitrokey_pro|Nitrokey__nitrokey-pro-hardware__|$STRESS/boards_unrouted_set2"
)

for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag destdir <<< "$entry"
  # routed-reference dir mirrors the stripped dir: boards_unrouted->boards, _set2->_set2
  case "$destdir" in
    *boards_unrouted_set2) refdir="$STRESS/boards_set2" ;;
    *boards_unrouted)      refdir="$STRESS/boards" ;;
    *)                     refdir="$STRESS/boards_set2" ;;
  esac
  # find the one source file containing the fragment (handles spaces in names)
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$refdir/$name.kicad_pcb" "$destdir/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  # copy sibling .kicad_pro next to the routed reference so list_nets finds netclasses
  profile="${srcfile%.kicad_pcb}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$refdir/$name.kicad_pro"
  # silk-heavy boards: skip KiCad's slow silkscreen DRC. Apply to whichever .kicad_pro
  # copies exist (routed reference + stripped/unrouted board); missing ones are skipped.
  case " $SILK_IGNORE " in
    *" $name "*) python3 "$SILKPY" "$refdir/$name.kicad_pro" "$destdir/$name.kicad_pro" ;;
  esac
done
echo "Done. stripped->boards_unrouted_set2/ (+ replacement in boards_unrouted/);"
echo "routed reference + .kicad_pro -> boards_set2/ (+ replacement in boards/)."
