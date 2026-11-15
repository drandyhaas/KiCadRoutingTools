#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set27/jetson_agx_csi_adapter.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers In1.Cu In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --escape-method underpad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1 --grid-step 0.05 --output step2_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --escape-method underpad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1 --grid-step 0.05 --allow-via-in-pad --output step2_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step2b_capopt.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2b_capopt.kicad_pcb step3_diffpairs.kicad_pcb --nets '/Interface/CSI*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --impedance 95 --track-width 0.182 --diff-pair-gap 0.284 --clearance-ceiling 0.15 --no-gnd-vias --output step3_diffpairs.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.15 --track-width 0.16 --via-size 0.6 --via-drill 0.3 --power-nets GND +3V3 +5V +1V8 AVDD_CAM_2V8 --power-nets-widths 0.3 0.3 0.3 0.3 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --hole-to-hole-clearance 0.2 --output step4_route.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step4b_retry.kicad_pcb --nets +3V3 /Interface/CSI4_D1_P /Interface/CSI4_D1_N --no-bga-zone --max-ripup 5 --clearance-ceiling 0.15 --track-width 0.16 --via-size 0.6 --via-drill 0.3 --power-nets GND +3V3 +5V +1V8 AVDD_CAM_2V8 --power-nets-widths 0.3 0.3 0.3 0.3 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --hole-to-hole-clearance 0.2 --ripped-route-avoidance-cost 3 --track-proximity-cost 2 --output step4b_retry.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4b_retry.kicad_pcb step4c_diffretry.kicad_pcb --nets /Interface/CSI4_D1_P /Interface/CSI4_D1_N --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --impedance 95 --track-width 0.127 --diff-pair-gap 0.127 --via-size 0.45 --via-drill 0.25 --clearance-ceiling 0.1 --no-gnd-vias --max-ripup 5 --output step4c_diffretry.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4c_diffretry.kicad_pcb step5_d7bridge.kicad_pcb --nets 'unconnected-(D7-CH1-Pad1)' 'unconnected-(D7-CH4-Pad5)' --clearance-ceiling 0.15 --track-width 0.16 --via-size 0.6 --via-drill 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --hole-to-hole-clearance 0.2 --output step5_d7bridge.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/jetson_agx_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_d7bridge.kicad_pcb step6_final.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.15 --track-width 0.16 --via-size 0.6 --via-drill 0.3 --power-nets GND +3V3 +5V +1V8 AVDD_CAM_2V8 --power-nets-widths 0.3 0.3 0.3 0.3 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --hole-to-hole-clearance 0.2 --output step6_final.kicad_pcb
