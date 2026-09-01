#!/bin/bash
# Profile the FULL K28 chain: cProfile the plan+fanout stage, time
# every stage separately, reuse prof_top for the hot rows.
cd "$(dirname "$0")"
K=28
NETS=$(python3 coherent_nets.py $K)
echo "=== stage 1+2: plan_ends + dest fanout (cProfile)"
TWO_PAGE=1 TP_SCOPE=split /usr/bin/time -p python3 -m cProfile \
  -o tmp/profc_fo.prof fanout_from_plan.py tmp/profc_fo_k28.kicad_pcb \
  $K --board=fb_t2q_base.kicad_pcb --no-lines --two-page \
  --no-plane-drop > tmp/profc_fo.log 2> tmp/profc_fo.time
cat tmp/profc_fo.time
echo "=== stage 3: braid (timed; profiled earlier in tmp/prof28.prof)"
TWO_PAGE=1 /usr/bin/time -p python3 braid.py \
  --board tmp/profc_fo_k28.kicad_pcb --dest DU1 --nets "$NETS" \
  --out tmp/profc_braid > tmp/profc_braid.log 2> tmp/profc_braid.time
cat tmp/profc_braid.time
echo "=== stage 4: grading (timed)"
/usr/bin/time -p python3 grade_k.py tmp/profc_braid.kicad_pcb "$NETS" \
  2> tmp/profc_grade.time
cat tmp/profc_grade.time
echo "=== fanout-stage hot rows"
python3 prof_top.py tmp/profc_fo.prof 2>&1 | head -35
