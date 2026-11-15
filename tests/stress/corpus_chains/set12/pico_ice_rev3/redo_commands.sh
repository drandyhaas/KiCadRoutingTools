#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/pico_ice_rev3.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --via-size 0.508 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2a_fanout_u3.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --clearance 0.127 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u3.kicad_pcb -o step2b_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' --clearance 0.127 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u6.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diffpairs.kicad_pcb --nets /USB_D+ /USB_D- /USB_P /USB_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1524 --diff-pair-gap 0.14097 --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --via-size 0.508 --via-drill 0.254 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diffpairs.kicad_pcb --nets /USB_D+ /USB_D- /USB_P /USB_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1524 --diff-pair-gap 0.14097 --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --via-size 0.508 --via-drill 0.254 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diffpairs.kicad_pcb --nets /USB_D+ /USB_D- /USB_P /USB_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1524 --diff-pair-gap 0.14097 --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --via-size 0.508 --via-drill 0.254 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --power-nets GND +3V3 VBUS +1V1 /VCC_PLL +1V2 --power-nets-widths 0.3 0.3 0.3 0.25 0.2 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --power-nets GND +3V3 VBUS +1V1 /VCC_PLL +1V2 --power-nets-widths 0.3 0.3 0.3 0.25 0.2 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --power-nets GND +3V3 VBUS +1V1 /VCC_PLL +1V2 --power-nets-widths 0.3 0.3 0.3 0.25 0.2 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --power-nets GND +3V3 VBUS +1V1 /VCC_PLL +1V2 --power-nets-widths 0.3 0.3 0.3 0.25 0.2 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --power-nets GND +3V3 VBUS +1V1 /VCC_PLL +1V2 --power-nets-widths 0.3 0.3 0.3 0.25 0.2 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' '!/SWCLK' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --power-nets GND +3V3 VBUS +1V1 /VCC_PLL +1V2 --power-nets-widths 0.3 0.3 0.3 0.25 0.2 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --power-nets GND +3V3 VBUS +1V1 /VCC_PLL +1V2 --power-nets-widths 0.3 0.3 0.3 0.25 0.2 0.2 --max-ripup 2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/pico_ice_rev3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --power-nets GND +3V3 VBUS +1V1 /VCC_PLL +1V2 --power-nets-widths 0.3 0.3 0.3 0.25 0.2 0.2 --max-ripup 1
