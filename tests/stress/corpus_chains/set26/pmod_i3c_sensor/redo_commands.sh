#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/pmod_i3c_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/pmod_i3c_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/pmod_i3c_sensor.kicad_pcb step1_planes.kicad_pcb --nets GND VCC /SEN1_VCC --plane-layers In1.Cu In2.Cu In2.Cu --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/pmod_i3c_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/pmod_i3c_sensor.kicad_pcb step1_planes.kicad_pcb --nets GND VCC --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/pmod_i3c_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/pmod_i3c_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/pmod_i3c_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND VCC /SEN1_VCC /SEN2_VDD /SEN3_VDD /PMOD_3V3 /LDO_OUTPUT --power-nets-widths 0.3 0.3 0.25 0.25 0.25 0.3 0.25 --max-ripup 5 --no-bga-zone
