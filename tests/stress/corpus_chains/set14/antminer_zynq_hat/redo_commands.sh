#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/antminer_zynq_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/antminer_zynq_hat.kicad_pcb step1_pour.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/antminer_zynq_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_pour.kicad_pcb --component IC1 --nets '*' '!GND' --clearance 0.1 --width 0.15 --output step1b_fanout_ic1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/antminer_zynq_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_ic1.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/antminer_zynq_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_capopt.kicad_pcb step2a_diff.kicad_pcb --nets '/Usb connection/USB_D+' '/Usb connection/USB_D-' --track-width 0.15 --diff-pair-gap 0.15 --clearance-ceiling 0.15 --no-gnd-vias
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/antminer_zynq_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND +3.3V +1.8V +5V +12V 2.5VCC_IO 'Net-(J13-+5V)' 'Net-(IC1-VBUS)' '/Usb connection/V_BUS' --power-nets-widths 0.4 0.4 0.3 0.4 0.3 0.3 0.3 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/antminer_zynq_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND +3.3V +1.8V +5V +12V 2.5VCC_IO 'Net-(J13-+5V)' 'Net-(IC1-VBUS)' '/Usb connection/V_BUS' --power-nets-widths 0.4 0.4 0.3 0.4 0.3 0.3 0.3 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/antminer_zynq_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND +3.3V +1.8V +5V +12V 2.5VCC_IO 'Net-(J13-+5V)' 'Net-(IC1-VBUS)' '/Usb connection/V_BUS' --power-nets-widths 0.4 0.4 0.3 0.4 0.3 0.3 0.3 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/antminer_zynq_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND +3.3V +1.8V +5V +12V 2.5VCC_IO 'Net-(J13-+5V)' 'Net-(IC1-VBUS)' '/Usb connection/V_BUS' --power-nets-widths 0.4 0.4 0.3 0.4 0.3 0.3 0.3 0.3 0.4 --max-ripup 5
