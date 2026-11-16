#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/private/tmp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/usmu_smu.kicad_pcb /tmp/test_out.kicad_pcb --nets GND -10V --plane-layers In1.Cu In2.Cu --dry-run
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/usmu_smu.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step1_planes.kicad_pcb --nets GND GND GND -10V +10V +3V3 +4V5 VBUS --plane-layers F.Cu In1.Cu B.Cu In2.Cu In2.Cu In2.Cu In2.Cu In2.Cu --via-size 0.45 --via-drill 0.3 --track-width 0.25 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/usmu_smu.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step1_planes.kicad_pcb --nets GND GND GND -10V --plane-layers F.Cu In1.Cu B.Cu In2.Cu --via-size 0.45 --via-drill 0.3 --track-width 0.25 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step1_planes.kicad_pcb -o /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step2a_fanout_u3.kicad_pcb --component U3 --clearance 0.09 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step2a_fanout_u3.kicad_pcb -o /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step2b_fanout_u12.kicad_pcb --component U12 --clearance 0.09 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step2b_fanout_u12.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step2c_fanout_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step2c_fanout_clearance.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step3_diffpairs.kicad_pcb --nets USB_CONN_DP USB_CONN_DN USB_D+ USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step3_diffpairs.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --power-nets GND -10V +10V +3V3 +4V5 VBUS /+4V096 --power-nets-widths 0.4 0.4 0.4 0.4 0.4 0.5 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step3_diffpairs.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set22/usmu_smu/step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --power-nets GND -10V +10V +3V3 +4V5 VBUS /+4V096 --power-nets-widths 0.4 0.4 0.4 0.4 0.4 0.5 0.3 --max-ripup 5
