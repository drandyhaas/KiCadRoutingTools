#!/bin/bash
# THE POSE GATE: the chain over the same pair in every pose -- both
# arrays on the front (FF, the control), the source on the back (BF),
# the destination on the back (FB), both (BB), the FF article turned
# over entirely (MM, the reflection isometry), moved on the sheet by a
# lattice offset (T, the translation isometry), and rotated by 90 / 180
# / 270 degrees. The isometries (T, R*, MM) are the SAME board, so
# their grades must equal FF's to the via and the segment: the chain is
# translation-invariant (every last-bit tie decided by a rounded key),
# runs every quarter-turn pose in one frame (flow_frame.py, chain_k.sh)
# and every mirror in one chirality (the selector's PairFrame, the
# braid's turn-over); a difference is a stage leaning on the board's
# axes or its sign. A side switch is a different article (the caps under
# an array follow it to the other face), so its grade may differ -- but
# the chain must COMPLETE it, and nothing may assume the tooth layer is
# F.
#
#   [POSES="T R90"] [GATE=name] [LADDER=file] [SHIFT="10.3 -7.7"] bash pose_gate.sh BOARD.kicad_pcb SRC DST K [K...] [-- make_bench flags]
#
# Articles are built by make_bench.py into tmp/gate/<stem>_<pose>; ONE
# ladder (LADDER=file, else the FF article's own) is placed beside every
# pose so the K prefixes name the same nets everywhere; each pose then
# runs chain_k.sh at every K. The table at the end: pose, K, open, DRC,
# vias, segments, in-band (lanes not routed at the last call), seconds;
# then the ISOMETRY VERDICT: each of T / R90 / R180 / R270 / MM against
# FF on (open, vias, segments), PASS only when all three are equal at
# every K. Exit status 1 when an isometry pose fails.
cd "$(dirname "$0")"
BOARD=$1; SRC=$2; DST=$3; shift 3
KS=()
while [ $# -gt 0 ] && [ "$1" != "--" ]; do KS+=("$1"); shift; done
[ "$1" = "--" ] && shift
STEM=tmp/gate/$(basename "${BOARD%.kicad_pcb}")
mkdir -p tmp/gate
POSES=${POSES:-"FF T MM R90 R180 R270 BF FB BB"}   # a subset by env; (macOS bash 3.2: no associative arrays)
SHIFT=${SHIFT:-"10.3 -7.7"}                         # the T pose: multiples of 0.1 mm, so every grid is its own image
GATE=${GATE:-gate}                               # the run's name: chain tags and the table
opts_of() {
  case "$1" in
    FF) echo "";; BF) echo "--src-side B";; FB) echo "--dst-side B";;
    BB) echo "--src-side B --dst-side B";;
    R*) echo "--rotate ${1#R}";;
  esac
}
for P in $POSES; do
  echo "##### building $P"
  if [ "$P" = MM ] || [ "$P" = T ]; then
    # the full mirror of the FF article, copper included -- the reflection
    # isometry (BB flips the arrays and only the parts they collide with);
    # or the FF article moved by SHIFT -- the translation isometry
    [ -f "${STEM}_FF.kicad_pcb" ] || python3 make_bench.py "$BOARD" "$SRC" "$DST" "${STEM}_FF.kicad_pcb" "$@" > /dev/null 2>&1
    if [ "$P" = MM ]; then
      python3 mirror_board.py "${STEM}_FF.kicad_pcb" "${STEM}_MM.kicad_pcb" 2>/dev/null | grep -E "wrote|FAILED" | sed 's/^/  /'
    else
      python3 translate_board.py "${STEM}_FF.kicad_pcb" "${STEM}_T.kicad_pcb" $SHIFT 2>/dev/null | grep -E "wrote|FAILED" | sed 's/^/  /'
    fi
  else
  python3 make_bench.py "$BOARD" "$SRC" "$DST" "${STEM}_$P.kicad_pcb" $(opts_of "$P") "$@" 2>/dev/null \
    | grep -E "flipped|fanned|DRC|ladder|rotation" | sed 's/^/  /'
  fi
  if [ -n "$LADDER" ]; then cp "$LADDER" "${STEM}_$P.ladder.txt";
  elif [ "$P" != FF ] && [ -f "${STEM}_FF.ladder.txt" ]; then cp "${STEM}_FF.ladder.txt" "${STEM}_$P.ladder.txt"; fi
done
TABLE=tmp/gate/$(basename "$STEM")_${GATE}_table.txt
printf '%-5s %-4s %-5s %-4s %-5s %-6s %-8s %s\n' pose K open drc vias segs in-band s | tee "$TABLE"
for P in $POSES; do
  for K in "${KS[@]}"; do
    t0=$(date +%s)
    BASE="${STEM}_$P.kicad_pcb" DEST="$DST" bash chain_k.sh "${GATE}_${P}" "$K" > "tmp/gate/${GATE}_${P}_k${K}.out" 2>&1
    t1=$(date +%s)
    G=$(grep GRADE "tmp/gate/${GATE}_${P}_k${K}.out" | tail -1)
    OPEN=$(echo "$G" | grep -o 'open=[0-9]*' | cut -d= -f2)
    DRC=$(echo "$G" | grep -o 'drc=[0-9]*' | head -1 | cut -d= -f2)
    VIAS=$(echo "$G" | grep -o 'vias=[0-9]*' | cut -d= -f2)
    SEGS=$(echo "$G" | grep -o 'segs=[0-9]*' | cut -d= -f2)
    LC=$(grep -c "last call routed" "tmp/${GATE}_${P}_k${K}.log" 2>/dev/null)
    N=$(python3 coherent_nets.py "$K" --board="${STEM}_$P.kicad_pcb" 2>/dev/null | tr ',' '\n' | grep -c .)
    IB=$(( N - ${LC:-0} - ${OPEN:-0} ))
    printf '%-5s %-4s %-5s %-4s %-5s %-6s %-8s %s\n' "$P" "$K" "${OPEN:-BROKEN}" "${DRC:--}" "${VIAS:--}" "${SEGS:--}" "$IB/$N" "$((t1 - t0))" | tee -a "$TABLE"
  done
done
# THE ISOMETRY VERDICT: T, R*, MM against FF on (open, vias, segments)
FAIL=0
for P in $POSES; do
  case "$P" in T|R*|MM) ;; *) continue;; esac
  for K in "${KS[@]}"; do
    A=$(awk -v p=FF -v k="$K" '$1==p && $2==k {print $3, $5, $6}' "$TABLE")
    B=$(awk -v p="$P" -v k="$K" '$1==p && $2==k {print $3, $5, $6}' "$TABLE")
    if [ -n "$A" ] && [ "$A" = "$B" ]; then V=PASS; else V=FAIL; FAIL=1; fi
    echo "isometry $P K$K: $V  (open vias segs: FF $A | $P $B)" | tee -a "$TABLE"
  done
done
echo "=== pose gate done $(date +%H:%M:%S)  ($TABLE)"
exit $FAIL
