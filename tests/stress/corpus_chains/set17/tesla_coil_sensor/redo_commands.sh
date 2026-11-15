#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/tesla_coil_sensor.kicad_pcb step1_planes.kicad_pcb --nets GND '/V_{DD}' --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND '/V_{DD}' --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout_u1.kicad_pcb -c U1 --nets '*' '!GND' '!/V_{DD}' --clearance 0.09 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step2b_capcleared.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_capcleared.kicad_pcb step3_diffpairs.kicad_pcb --nets /TC1+ /TC1- /TC2+ /TC2- 'Net-(U3-IN+)' 'Net-(U3-IN-)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias --layers F.Cu In1.Cu In2.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tesla_coil_sensor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND '/V_{DD}' '/V_{BUCK_5V}' '/V_{BUS_SENSOR}' '/V_{TEMP_SENSOR}' '/V_{CURRENT_SENSOR}' +12V --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.4
