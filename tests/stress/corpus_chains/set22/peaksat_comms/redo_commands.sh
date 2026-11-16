#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/peaksat_comms.kicad_pcb step1_planes.kicad_pcb --nets GND GND 3V3_D 3V3_RF --plane-layers In1.Cu In4.Cu In3.Cu In3.Cu --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_bga_u401.kicad_pcb --component U401 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!3V3_D' '!3V3_RF' --clearance 0.1 --track-width 0.25 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_bga_u401.kicad_pcb step2c_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2c_clearance.kicad_pcb step3a_diff_rf.kicad_pcb --nets '/RF/*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --impedance 100 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3a_diff_rf.kicad_pcb step3b_diff_can.kicad_pcb --nets '/CAN_A*' '/CAN_B*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.15 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3b_diff_can.kicad_pcb step4_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --power-nets GND 3V3_D 3V3_RF VIN --power-nets-widths 0.4 0.4 0.3 0.5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.0 1.5 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets 'Net-(C1006-Pad2)' /RF/uhf-frontend/5V_RX_U --no-bga-zone --max-ripup 5 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --track-width 0.1 --grid-step 0.05 --via-size 0.3 --via-drill 0.15 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.0 1.5 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step6_retry.kicad_pcb --nets 'Net-(C1006-Pad2)' /RF/uhf-frontend/5V_RX_U --no-bga-zone --max-ripup 5 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --track-width 0.0762 --grid-step 0.025 --via-size 0.25 --via-drill 0.15 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.0 1.5 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/peaksat_comms
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step6_retry.kicad_pcb --nets 'Net-(C1006-Pad2)' /RF/uhf-frontend/5V_RX_U --no-bga-zone --max-ripup 5 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --track-width 0.0762 --grid-step 0.025 --via-size 0.25 --via-drill 0.15 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.0 1.5 3.0 1.0
