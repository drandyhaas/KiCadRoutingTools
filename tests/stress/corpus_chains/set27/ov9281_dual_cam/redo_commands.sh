#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/private/tmp/wt_baseline_0809
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set27/ov9281_dual_cam.kicad_pcb step1_planes.kicad_pcb --nets GND +1V8 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb step2a_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+1V8' --diff-pairs '*CAM_L*' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+1V8' --diff-pairs '*CAM_L*' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2a_fanout_u1.kicad_pcb --output step2b_fanout_u2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+1V8' --diff-pairs '*CAM_R*' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2b_fanout_u2.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*CAM_L*' '*CAM_R*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +1V8 +3V3 +1V2 +2V8 --power-nets-widths 0.254 0.254 0.254 0.254 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/ov9281_dual_cam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4b_route_thin.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND +1V8 +3V3 +1V2 +2V8 --power-nets-widths 0.254 0.254 0.254 0.254 0.254
