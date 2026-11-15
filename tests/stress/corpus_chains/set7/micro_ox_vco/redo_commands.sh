#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/micro_ox_vco.kicad_pcb step1_planes.kicad_pcb --nets GND +12V --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --track-width 0.25 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.1 --extension 0.2 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u6.kicad_pcb --output step2b_fanout_u8.kicad_pcb --component U8 --nets '*' '!GND' --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.1 --extension 0.2 --grid-step 0.05
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u8.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*sine_in*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4a_route_fine.kicad_pcb --nets 'Net-(U6*)' 'Net-(U8*)' '/VCO_A/*' '/VCO_B/*' /PWM /TIME_REV 'Net-(Q201-E)' 'Net-(Q301-E)' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4a_route_fine.kicad_pcb step4b_route_all.kicad_pcb --nets '*' --rip-existing-nets +5V -5V --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +12V -12V +5V -5V +5V_REF -5V_REF --power-nets-widths 0.4 0.4 0.4 0.4 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4b_route_all.kicad_pcb step4c_widen_power.kicad_pcb --nets +5V -5V --rip-existing-nets +5V -5V --force-reroute --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets +5V -5V --power-nets-widths 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/micro_ox_vco
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4c_widen_power.kicad_pcb step5_final.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +12V -12V +5V -5V +5V_REF -5V_REF --power-nets-widths 0.4 0.4 0.4 0.4 0.4 0.3 0.3
