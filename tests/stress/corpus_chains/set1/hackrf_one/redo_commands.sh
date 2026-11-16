#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/hackrf_one.kicad_pcb step1_planes.kicad_pcb --nets GND VCC --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets GND VCC --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U4 --output step2a_fanout_U4.kicad_pcb --nets '*' '!GND' '!VCC' --clearance 0.2 --via-size 0.5 --via-drill 0.3 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_U4.kicad_pcb --component U17 --output step2b_fanout_U17.kicad_pcb --nets '*' '!GND' '!VCC' --clearance 0.2 --via-size 0.5 --via-drill 0.3 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U17.kicad_pcb --component U18 --output step2c_fanout_U18.kicad_pcb --nets '*' '!GND' '!VCC' --clearance 0.2 --via-size 0.5 --via-drill 0.3 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_U18.kicad_pcb --component U19 --output step2d_fanout_U19.kicad_pcb --nets '*' '!GND' '!VCC' --clearance 0.2 --via-size 0.5 --via-drill 0.3 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fanout_U19.kicad_pcb --component U23 --output step2e_fanout_U23.kicad_pcb --nets '*' '!GND' '!VCC' --clearance 0.2 --via-size 0.5 --via-drill 0.3 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2e_fanout_U23.kicad_pcb --component U24 --output step2f_fanout_U24.kicad_pcb --nets '*' '!GND' '!VCC' --clearance 0.2 --via-size 0.5 --via-drill 0.3 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2f_fanout_U24.kicad_pcb step2g_capclear.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2g_capclear.kicad_pcb step3_diffpairs.kicad_pcb '/baseband/*' /mcu/usb/power/DP /mcu/usb/power/DM --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND VCC VAA /mcu/usb/power/VIN +1V8 /mcu/usb/power/VBUS /mcu/usb/power/VBAT /mcu/usb/power/VBUSCTRL --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND VCC VAA /mcu/usb/power/VIN +1V8 /mcu/usb/power/VBUS /mcu/usb/power/VBAT /mcu/usb/power/VBUSCTRL --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/hackrf_one
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND VCC VAA /mcu/usb/power/VIN +1V8 /mcu/usb/power/VBUS /mcu/usb/power/VBAT /mcu/usb/power/VBUSCTRL --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25
