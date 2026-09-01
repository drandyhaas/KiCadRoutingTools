#!/bin/bash
# General schedule-pinning post-pass: dump the braid's own plan for
# FO_BOARD, re-braid with those pages pinned via the sidecar.
# usage: run_pinpages.sh FO_BOARD OUT_TAG [K]
cd "$(dirname "$0")"
FO=$1
OUT=$2
K=${3:-28}
# outputs under tmp/ (gitignored) unless the caller passes a path
mkdir -p tmp
case "$OUT" in */*) ;; *) OUT="tmp/$OUT";; esac
NETS=$(python3 coherent_nets.py "$K")
TWO_PAGE=1 python3 braid.py --board "$FO" --dest DU1 --nets "$NETS" \
  --out /dev/null --plan-json "${OUT}_plan.json" > /dev/null 2>&1
python3 -c "
import json
plan = json.load(open('${OUT}_plan.json'))
json.dump({n: d.get('page') for n, d in plan.items()},
          open('${FO%.kicad_pcb}.pages.json', 'w'), indent=1)
print('pages pinned:', sum(1 for d in plan.values() if d.get('page')))"
TWO_PAGE=1 python3 -u braid.py --board "$FO" --dest DU1 \
  --nets "$NETS" --out "$OUT" > "${OUT}.log" 2>&1
echo "braid exit=$?"
python3 grade_k.py "${OUT}.kicad_pcb" "$NETS"
rm -f "${FO%.kicad_pcb}.pages.json"
