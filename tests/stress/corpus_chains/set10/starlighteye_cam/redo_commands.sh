#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/starlighteye_cam.kicad_pcb step1_planes.kicad_pcb --nets GND +1V1 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.1 --zone-clearance 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb step2_fanout_u4.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+1V1' --diff-pairs CAM_CLK_P/CAM_CLK_N CAM_D0_P/CAM_D0_N CAM_D1_P/CAM_D1_N CAM_D2_P/CAM_D2_N CAM_D3_P/CAM_D3_N --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u4.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+1V1' --diff-pairs CAM_CLK_P/CAM_CLK_N CAM_D0_P/CAM_D0_N CAM_D1_P/CAM_D1_N CAM_D2_P/CAM_D2_N CAM_D3_P/CAM_D3_N --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u4.kicad_pcb --component U4 --escape-method underpad --allow-via-in-pad --nets '*' '!GND' '!+1V1' --clearance 0.1 --width 0.1 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_u4.kicad_pcb --output step3a_fanout_u3.kicad_pcb --component U3 --nets '*' '!GND' '!+1V1' --clearance 0.1 --width 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3a_fanout_u3.kicad_pcb --output step3b_fanout_u8.kicad_pcb --component U8 --nets '*' '!GND' '!+1V1' --clearance 0.1 --width 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3b_fanout_u8.kicad_pcb step3c_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3c_capclear.kicad_pcb step4_diff.kicad_pcb CAM_CLK_P CAM_D0_P CAM_D1_P CAM_D2_P CAM_D3_P /CLK_P /D0_P /D1_P /D2_P /D3_P --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +1V1 +3.3VA +1V8 +3V3 +3.6V 'Net-(U1-VIN)' --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/starlighteye_cam
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step5b_retry_1v8.kicad_pcb --nets +1V8 --rip-existing-nets +1V8 --force-reroute --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.09 --track-width 0.0889 --via-size 0.3 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --power-nets +1V8 --power-nets-widths 0.15
