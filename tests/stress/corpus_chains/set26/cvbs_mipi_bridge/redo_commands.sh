#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/cvbs_mipi_bridge.kicad_pcb -c IC1 --nets '*' '!GND' --clearance 0.09 --grid-step 0.05 -o step1_fanout_ic1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step1_fanout_ic1.kicad_pcb step1b_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step1b_capclear.kicad_pcb step2_planes.kicad_pcb --nets GND 3V3_SYS --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2_planes.kicad_pcb step3_diffpairs.kicad_pcb '/CSI_A_*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.125 --diff-pair-gap 0.13 --impedance 100 --via-size 0.45 --via-drill 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/cvbs_mipi_bridge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.15 --grid-step 0.05 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets GND 3V3_SYS DVDD_1V8 AVDD_1V8 PVDD_1V8 --power-nets-widths 0.4 0.4 0.3 0.3 0.3
