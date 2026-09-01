#!/bin/bash
# re-materialize the K28 44v record: sv31 comb + own attempt-1 pages
# with SA8 -> B page, braided pinned. Deterministic.
cd "$(dirname "$0")"
NETS=$(python3 coherent_nets.py 28)
FO=tmp/sv31_fo_k28.kicad_pcb
TWO_PAGE=1 python3 braid.py --board "$FO" --dest DU1 --nets "$NETS" \
  --out /dev/null --plan-json tmp/rec44_plan.json > /dev/null 2>&1
python3 -c "
import json
plan = json.load(open('tmp/rec44_plan.json'))
pages = {n: d.get('page') for n, d in plan.items()}
pages['SA8'] = 'B.Cu'
json.dump(pages, open('tmp/sv31_fo_k28.pages.json', 'w'), indent=1)"
TWO_PAGE=1 python3 -u braid.py --board "$FO" --dest DU1 \
  --nets "$NETS" --out tmp/k28_record44 > tmp/k28_record44.log 2>&1
echo "braid exit=$?"
python3 grade_k.py tmp/k28_record44.kicad_pcb "$NETS"
rm -f tmp/sv31_fo_k28.pages.json
