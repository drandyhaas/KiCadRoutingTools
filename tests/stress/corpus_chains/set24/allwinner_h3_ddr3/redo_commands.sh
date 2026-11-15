#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/allwinner_h3_ddr3.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb step2_fanout_DU1.kicad_pcb --component DU1 --layers F.Cu B.Cu --nets '*' '!GND' '!VCC-DRAM' --diff-pairs '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --diff-pair-gap 0.1143 --track-width 0.1143 --clearance 0.0889 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_DU1.kicad_pcb --component DU1 --layers F.Cu B.Cu --nets '*' '!GND' '!VCC-DRAM' --diff-pairs '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --diff-pair-gap 0.1143 --track-width 0.1143 --clearance 0.0889 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_DU1_v2.kicad_pcb --component DU1 --layers F.Cu B.Cu --nets '*' '!GND' '!VCC-DRAM' --diff-pairs '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --diff-pair-gap 0.1143 --track-width 0.1 --clearance 0.1 --via-size 0.3 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_DU1_v3.kicad_pcb --component DU1 --layers F.Cu B.Cu --nets '*' '!GND' '!VCC-DRAM' --diff-pairs '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --diff-pair-gap 0.1143 --track-width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_DU1_v3.kicad_pcb step2b_clearance_DU1.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2b_clearance_DU1.kicad_pcb --output step3_fanout_U1.kicad_pcb --component U1 --layers F.Cu B.Cu --nets '*' '!GND' '!VCC-DRAM' --diff-pairs '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --diff-pair-gap 0.1143 --track-width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2b_clearance_DU1.kicad_pcb --output step3_fanout_U1_v2.kicad_pcb --component U1 --layers F.Cu B.Cu --nets '*' '!GND' '!VCC-DRAM' --diff-pairs '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --diff-pair-gap 0.1143 --track-width 0.1 --clearance 0.1 --via-size 0.2 --via-drill 0.12
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2b_clearance_DU1.kicad_pcb --output step3_fanout_U1_v3.kicad_pcb --component U1 --layers F.Cu B.Cu --nets '*' '!GND' '!VCC-DRAM' --diff-pairs '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --diff-pair-gap 0.1143 --escape-method underpad --track-width 0.1 --clearance 0.1 --via-size 0.2 --via-drill 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2b_clearance_DU1.kicad_pcb --output step3_fanout_U1_v4.kicad_pcb --component U1 --layers F.Cu B.Cu --nets '*' '!GND' '!VCC-DRAM' --diff-pairs '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --diff-pair-gap 0.1143 --escape-method dogbone --track-width 0.1 --clearance 0.1 --via-size 0.2 --via-drill 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step3_fanout_U1_v4.kicad_pcb step3b_clearance_U1.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3b_clearance_U1.kicad_pcb --output step4_diff.kicad_pcb --nets '*SCKP*' '*SCKN*' '*SDQS0P*' '*SDQS0N*' '*SDQS1P*' '*SDQS1N*' --track-width 0.0889 --diff-pair-gap 0.1143 --clearance-ceiling 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --no-gnd-vias --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diff.kicad_pcb --output step5_route.kicad_pcb --nets '*' --power-nets GND VCC-DRAM --power-nets-widths 0.3 0.4 --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --length-match-group auto --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diff.kicad_pcb --output step5_route.kicad_pcb --nets '*' --power-nets GND VCC-DRAM --power-nets-widths 0.3 0.4 --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --length-match-group auto --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diff.kicad_pcb --output step5_route.kicad_pcb --nets '*' --power-nets GND VCC-DRAM --power-nets-widths 0.3 0.4 --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --length-match-group auto --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diff.kicad_pcb --output step5_route.kicad_pcb --nets '*' --power-nets GND VCC-DRAM --power-nets-widths 0.3 0.4 --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --length-match-group auto --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diff.kicad_pcb --output step5b_route_retry.kicad_pcb --nets '*' --power-nets GND VCC-DRAM --power-nets-widths 0.3 0.4 --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --length-match-group auto --max-ripup 5 --no-bga-zone --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/allwinner_h3_ddr3
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5b_route_retry.kicad_pcb --output step6_fix.kicad_pcb --nets '/DDR3 16x1/SDQS0P' '/DDR3 16x1/SDQS0N' '/DDR3 16x1/SBA0' '/DDR3 16x1/SA14' GND --power-nets GND VCC-DRAM --power-nets-widths 0.3 0.4 --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --grid-step 0.05
