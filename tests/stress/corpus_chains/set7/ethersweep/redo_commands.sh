#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/ethersweep.kicad_pcb step1_planes.kicad_pcb --nets GND GND1 +3V3 +3.3VA +28V --plane-layers In1.Cu In1.Cu In2.Cu In2.Cu In2.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.2 --output step2a_fanout_U2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_U2.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --clearance 0.2 --output step2b_fanout_U3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U3.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' '!+3.3VA' --clearance 0.2 --output step2c_fanout_U6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U3.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' '!+3.3VA' --clearance 0.09 --output step2c_fanout_U6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U3.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' '!+3.3VA' --clearance 0.09 --track-width 0.0762 --output step2c_fanout_U6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U3.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' '!+3.3VA' --clearance 0.09 --width 0.0762 --output step2c_fanout_U6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U3.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' '!+3.3VA' --clearance 0.09 --escape-method underpad --via-size 0.35 --via-drill 0.2 --output step2c_fanout_U6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U3.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' '!+3.3VA' --clearance 0.09 --escape-method underpad --via-size 0.35 --via-drill 0.2 --allow-via-in-pad --output step2c_fanout_U6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_U6.kicad_pcb --component U7 --nets '*' '!GND' '!+3V3' '!+28V' --clearance 0.2 --output step2d_fanout_U7.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2d_fanout_U7.kicad_pcb step3_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb --nets 'Net-(J11-D+)' 'Net-(J11-D-)' 'Net-(U6-RXP)' 'Net-(U6-RXN)' 'Net-(U6-TXP)' 'Net-(U6-TXN)' RD+ RD- TD+ TD- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --output step4_diff.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --power-nets GND GND1 +3V3 +3.3VA +28V --power-nets-widths 0.3 0.25 0.3 0.25 0.4 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step5b_gndfix.kicad_pcb --nets GND --power-nets GND GND1 +3V3 +3.3VA +28V --power-nets-widths 0.3 0.25 0.3 0.25 0.4 --clearance-ceiling 0.09 --track-width 0.1 --via-size 0.35 --via-drill 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --grid-step 0.05 --max-ripup 5
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step5b_gndfix.kicad_pcb --nets GND --force-reroute --power-nets GND GND1 +3V3 +3.3VA +28V --power-nets-widths 0.3 0.25 0.3 0.25 0.4 --clearance-ceiling 0.09 --track-width 0.15 --via-size 0.35 --via-drill 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --grid-step 0.05 --max-ripup 5
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step5_route.kicad_pcb step5c_replane.kicad_pcb --nets GND GND1 +3V3 +3.3VA +28V --plane-layers In1.Cu In1.Cu In2.Cu In2.Cu In2.Cu --clearance-ceiling 0.09 --zone-clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ethersweep
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5c_replane.kicad_pcb step6_route.kicad_pcb --nets '*' --power-nets GND GND1 +3V3 +3.3VA +28V --power-nets-widths 0.3 0.25 0.3 0.25 0.4 --clearance-ceiling 0.09 --track-width 0.15 --via-size 0.35 --via-drill 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --grid-step 0.05 --max-ripup 5
