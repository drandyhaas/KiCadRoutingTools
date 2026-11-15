#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ackack_rover
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ackack_rover
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ackack_rover
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/ackack_rover.kicad_pcb step1_planes.kicad_pcb --nets GND GNDA +3.3V +3.3VA --plane-layers In1.Cu In1.Cu In2.Cu In2.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.23 --hole-to-hole-clearance 0.25 --power-nets GND GNDA +3.3V +3.3VA --power-nets-widths 0.4 0.35 0.4 0.35
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ackack_rover
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ackack_rover
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_power.kicad_pcb --nets GND GNDA +3.3V +3.3VA /VBUS /VBUS_IN VSYS VS-BA VS-BB /SBA_RP /SBB_RP /SBC_RP --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.23 --hole-to-hole-clearance 0.25 --via-size 0.5 --via-drill 0.3 --power-nets GND GNDA +3.3V +3.3VA /VBUS /VBUS_IN VSYS VS-BA VS-BB /SBA_RP /SBB_RP /SBC_RP --power-nets-widths 0.4 0.35 0.4 0.35 0.4 0.4 0.4 0.8 0.8 0.4 0.4 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/ackack_rover
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_power.kicad_pcb step3_signal.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --via-size 0.5 --via-drill 0.3 --track-width 0.25 --power-nets GND GNDA +3.3V +3.3VA /VBUS /VBUS_IN VSYS VS-BA VS-BB /SBA_RP /SBB_RP /SBC_RP --power-nets-widths 0.4 0.35 0.4 0.35 0.4 0.4 0.4 0.8 0.8 0.4 0.4 0.4 --max-ripup 5
